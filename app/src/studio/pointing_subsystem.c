/*
 * Copyright (c) 2025 The Conductor Contributors
 *
 * SPDX-License-Identifier: MIT
 */

#include <zephyr/logging/log.h>

LOG_MODULE_DECLARE(zmk_studio, CONFIG_ZMK_STUDIO_LOG_LEVEL);

#include <zephyr/settings/settings.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/sensor.h>

#include <zmk/studio/rpc.h>

#include <pb_encode.h>

/* External AML control functions from input_processor_temp_layer.c */
extern bool zmk_temp_layer_get_aml_enabled(void);
extern void zmk_temp_layer_set_aml_enabled(bool enabled);

ZMK_RPC_SUBSYSTEM(pointing)

#define POINTING_RESPONSE(type, ...) ZMK_RPC_RESPONSE(pointing, type, __VA_ARGS__)

/*
 * PMW3610 CPI attribute - matches the enum in pmw3610.h
 * We define it here to avoid a direct dependency on the PMW3610 driver header.
 */
#define PMW3610_ALT_ATTR_CPI 0

/* CPI constraints for PMW3610 */
#define PMW3610_MIN_CPI 200
#define PMW3610_MAX_CPI 3200
#define PMW3610_CPI_STEP 200

/* Default CPI from devicetree (monokey_R.overlay: cpi = <800>) */
#define DEFAULT_CPI 800

/*
 * Try to get the trackball device from devicetree.
 * The conductor board defines it as &trackball in the overlay.
 */
#if DT_HAS_COMPAT_STATUS_OKAY(pixart_pmw3610_alt)
#define HAS_TRACKBALL 1
static const struct device *trackball_dev = DEVICE_DT_GET(DT_INST(0, pixart_pmw3610_alt));
#else
#define HAS_TRACKBALL 0
static const struct device *trackball_dev = NULL;
#endif

/*
 * Global variables for the studio_scaler input processor.
 * These are read by input_processor_studio_scaler.c at runtime.
 */
volatile int32_t studio_scroll_numerator = 1;
volatile int32_t studio_scroll_denominator = 1;
volatile bool studio_scroll_inverted = false;

/*
 * Persistent sensitivity settings.
 * numerator/denominator form a rational multiplier.
 * Default is 1/1 (no scaling change), CPI = 0 means "use default".
 * scroll_inverted: 0 = normal, 1 = inverted (natural scrolling)
 */
static struct {
    uint32_t cursor_numerator;
    uint32_t cursor_denominator;
    uint32_t scroll_numerator;
    uint32_t scroll_denominator;
    uint32_t cpi;
    uint32_t scroll_inverted;
    uint32_t aml_enabled;
} pointing_settings = {
    .cursor_numerator = 1,
    .cursor_denominator = 1,
    .scroll_numerator = 1,
    .scroll_denominator = 1,
    .cpi = 0,
    .scroll_inverted = 0,
    .aml_enabled = 1,
};

/* Forward declaration for the settings handler */
static int pointing_settings_set(const char *name, size_t len,
                                  settings_read_cb read_cb, void *cb_arg);

SETTINGS_STATIC_HANDLER_DEFINE(zmk_pointing_studio, "pointing/studio",
                                NULL, pointing_settings_set, NULL, NULL);

static int pointing_settings_set(const char *name, size_t len,
                                  settings_read_cb read_cb, void *cb_arg) {
    if (strcmp(name, "sensitivity") == 0) {
        if (len != sizeof(pointing_settings)) {
            /* Handle migration from old formats */
            if (len == sizeof(pointing_settings) - sizeof(uint32_t)) {
                /* Migration: missing aml_enabled field */
                int rc = read_cb(cb_arg, &pointing_settings, len);
                if (rc >= 0) {
                    pointing_settings.aml_enabled = 1;
                    LOG_INF("Migrated pointing settings (added aml_enabled)");
                }
                return rc;
            }
            if (len == sizeof(pointing_settings) - 2 * sizeof(uint32_t)) {
                /* Migration: missing both scroll_inverted and aml_enabled */
                int rc = read_cb(cb_arg, &pointing_settings, len);
                if (rc >= 0) {
                    pointing_settings.scroll_inverted = 0;
                    pointing_settings.aml_enabled = 1;
                    LOG_INF("Migrated pointing settings (old format)");
                }
                return rc;
            }
            return -EINVAL;
        }
        int rc = read_cb(cb_arg, &pointing_settings, sizeof(pointing_settings));
        if (rc >= 0) {
            LOG_INF("Loaded pointing settings: cursor=%u/%u scroll=%u/%u cpi=%u inv=%u",
                    pointing_settings.cursor_numerator,
                    pointing_settings.cursor_denominator,
                    pointing_settings.scroll_numerator,
                    pointing_settings.scroll_denominator,
                    pointing_settings.cpi,
                    pointing_settings.scroll_inverted);
        }
        return rc;
    }
    return -ENOENT;
}

static int pointing_settings_save(void) {
    return settings_save_one("pointing/studio/sensitivity",
                              &pointing_settings, sizeof(pointing_settings));
}

/*
 * Compute the effective CPI based on cursor sensitivity ratio.
 *
 * If an explicit CPI is set (pointing_settings.cpi > 0), use that directly.
 * Otherwise, compute: effective_cpi = DEFAULT_CPI * numerator / denominator
 * Then clamp to [PMW3610_MIN_CPI, PMW3610_MAX_CPI] and round to nearest step.
 */
static uint32_t compute_effective_cpi(void) {
    uint32_t cpi;

    if (pointing_settings.cpi > 0) {
        cpi = pointing_settings.cpi;
    } else {
        /* Compute from cursor ratio */
        uint32_t num = pointing_settings.cursor_numerator;
        uint32_t den = pointing_settings.cursor_denominator;
        if (den == 0) {
            den = 1;
        }
        cpi = (DEFAULT_CPI * num + den / 2) / den; /* rounded division */
    }

    /* Clamp to valid range */
    if (cpi < PMW3610_MIN_CPI) {
        cpi = PMW3610_MIN_CPI;
    }
    if (cpi > PMW3610_MAX_CPI) {
        cpi = PMW3610_MAX_CPI;
    }

    /* Round to nearest step of 200 */
    cpi = ((cpi + PMW3610_CPI_STEP / 2) / PMW3610_CPI_STEP) * PMW3610_CPI_STEP;

    return cpi;
}

/*
 * Apply the current sensitivity settings to the running system.
 * - CPI: Uses the PMW3610 sensor_attr_set API to change CPI at runtime.
 * - Scroll: Updates the global variables read by studio_scroll_scaler.
 */
static void apply_sensitivity(void) {
    uint32_t effective_cpi = compute_effective_cpi();

    LOG_INF("Applying sensitivity: cursor=%u/%u scroll=%u/%u cpi=%u inv=%u",
            pointing_settings.cursor_numerator,
            pointing_settings.cursor_denominator,
            pointing_settings.scroll_numerator,
            pointing_settings.scroll_denominator,
            effective_cpi,
            pointing_settings.scroll_inverted);

    /* Update scroll globals for the studio_scaler input processor */
    studio_scroll_numerator = (int32_t)pointing_settings.scroll_numerator;
    studio_scroll_denominator = (int32_t)pointing_settings.scroll_denominator;
    studio_scroll_inverted = (pointing_settings.scroll_inverted != 0);

    LOG_INF("Scroll scaler updated: %d/%d inverted=%d",
            (int)studio_scroll_numerator,
            (int)studio_scroll_denominator,
            (int)studio_scroll_inverted);

#if HAS_TRACKBALL
    if (trackball_dev == NULL || !device_is_ready(trackball_dev)) {
        LOG_WRN("Trackball device not ready, cannot apply CPI");
        return;
    }

    struct sensor_value val = {
        .val1 = (int32_t)effective_cpi,
        .val2 = 0,
    };

    int err = sensor_attr_set(trackball_dev, SENSOR_CHAN_ALL,
                               (enum sensor_attribute)PMW3610_ALT_ATTR_CPI, &val);
    if (err) {
        LOG_ERR("Failed to set CPI to %u: %d", effective_cpi, err);
    } else {
        LOG_INF("CPI set to %u successfully", effective_cpi);
    }
#else
    LOG_WRN("No trackball device available, CPI change not applied");
#endif
}

/*
 * Apply settings on boot after settings are loaded.
 * This ensures saved sensitivity is restored after power cycle.
 */
static int pointing_studio_init(void) {
    /* Always apply scroll settings (even default 1/1) to initialize globals */
    studio_scroll_numerator = (int32_t)pointing_settings.scroll_numerator;
    studio_scroll_denominator = (int32_t)pointing_settings.scroll_denominator;
    studio_scroll_inverted = (pointing_settings.scroll_inverted != 0);

    /* Restore AML enabled state */
    zmk_temp_layer_set_aml_enabled(pointing_settings.aml_enabled != 0);
    LOG_INF("Restored AML enabled state: %u", pointing_settings.aml_enabled);

    /* Apply CPI if non-default */
    if (pointing_settings.cursor_denominator > 0 &&
        (pointing_settings.cursor_numerator != 1 ||
         pointing_settings.cursor_denominator != 1 ||
         pointing_settings.cpi > 0)) {
        LOG_INF("Restoring saved pointing sensitivity on boot");
        apply_sensitivity();
    }
    return 0;
}

/* Run after settings subsystem is initialized (priority 91 > settings at 90) */
SYS_INIT(pointing_studio_init, APPLICATION, 91);

zmk_studio_Response get_sensitivity(const zmk_studio_Request *req) {
    LOG_DBG("get_sensitivity called");

    zmk_pointing_GetSensitivityResponse resp =
        zmk_pointing_GetSensitivityResponse_init_zero;

    resp.cursor.numerator = pointing_settings.cursor_numerator;
    resp.cursor.denominator = pointing_settings.cursor_denominator;

    /* Return scroll settings - use sign to encode inversion */
    resp.scroll.numerator = pointing_settings.scroll_numerator;
    resp.scroll.denominator = pointing_settings.scroll_denominator;

    /* Encode inversion in the sign of scroll numerator */
    if (pointing_settings.scroll_inverted) {
        /* Use negative numerator to signal inversion to the client */
        resp.scroll.numerator = -(int32_t)pointing_settings.scroll_numerator;
    }

    /* Ensure we never return 0/0 - use 1/1 as default */
    if (resp.cursor.denominator == 0) {
        resp.cursor.numerator = 1;
        resp.cursor.denominator = 1;
    }
    if (resp.scroll.denominator == 0) {
        resp.scroll.numerator = 1;
        resp.scroll.denominator = 1;
    }

    resp.cpi = compute_effective_cpi();

    LOG_INF("get_sensitivity: cursor=%u/%u scroll=%d/%u cpi=%u inv=%u",
            resp.cursor.numerator, resp.cursor.denominator,
            (int)resp.scroll.numerator, resp.scroll.denominator,
            resp.cpi, pointing_settings.scroll_inverted);

    return POINTING_RESPONSE(get_sensitivity, resp);
}

zmk_studio_Response set_sensitivity(const zmk_studio_Request *req) {
    LOG_DBG("set_sensitivity called");

    const zmk_pointing_SetSensitivityRequest *set_req =
        &req->subsystem.pointing.request_type.set_sensitivity;

    LOG_INF("set_sensitivity: cursor=%u/%u scroll=%d/%u cpi=%u",
            set_req->cursor.numerator, set_req->cursor.denominator,
            (int)set_req->scroll.numerator, set_req->scroll.denominator,
            set_req->cpi);

    /* Validate cursor: denominator must not be zero */
    if (set_req->cursor.denominator == 0) {
        LOG_WRN("set_sensitivity: cursor denominator is 0, rejecting");
        zmk_pointing_SetSensitivityResponse resp =
            zmk_pointing_SetSensitivityResponse_init_zero;
        resp.which_result = zmk_pointing_SetSensitivityResponse_err_tag;
        resp.result.err = zmk_pointing_SetSensitivityErrorCode_SET_SENSITIVITY_ERR_INVALID;
        return POINTING_RESPONSE(set_sensitivity, resp);
    }

    /* Update cursor settings */
    pointing_settings.cursor_numerator = set_req->cursor.numerator;
    pointing_settings.cursor_denominator = set_req->cursor.denominator;

    /* Update scroll settings */
    if (set_req->scroll.denominator > 0) {
        /* Decode inversion from sign of numerator */
        int32_t scroll_num = (int32_t)set_req->scroll.numerator;
        if (scroll_num < 0) {
            pointing_settings.scroll_numerator = (uint32_t)(-scroll_num);
            pointing_settings.scroll_inverted = 1;
        } else {
            pointing_settings.scroll_numerator = (uint32_t)scroll_num;
            pointing_settings.scroll_inverted = 0;
        }
        pointing_settings.scroll_denominator = set_req->scroll.denominator;
    } else if (pointing_settings.scroll_denominator == 0) {
        /* Ensure we always have a valid scroll setting */
        pointing_settings.scroll_numerator = 1;
        pointing_settings.scroll_denominator = 1;
        pointing_settings.scroll_inverted = 0;
    }
    /* else: keep existing scroll settings when not provided */

    /* Update CPI if provided (0 means "don't change, compute from ratio") */
    if (set_req->cpi > 0) {
        pointing_settings.cpi = set_req->cpi;
    } else {
        /* Clear explicit CPI so it's computed from cursor ratio */
        pointing_settings.cpi = 0;
    }

    /* Apply to running system FIRST (immediate feedback) */
    apply_sensitivity();

    /* Then persist to flash */
    int ret = pointing_settings_save();
    if (ret < 0) {
        LOG_WRN("Failed to save pointing sensitivity settings: %d", ret);
        /* Note: CPI was already applied, just storage failed */
        zmk_pointing_SetSensitivityResponse resp =
            zmk_pointing_SetSensitivityResponse_init_zero;
        resp.which_result = zmk_pointing_SetSensitivityResponse_err_tag;
        resp.result.err = zmk_pointing_SetSensitivityErrorCode_SET_SENSITIVITY_ERR_STORAGE;
        return POINTING_RESPONSE(set_sensitivity, resp);
    }

    zmk_pointing_SetSensitivityResponse resp =
        zmk_pointing_SetSensitivityResponse_init_zero;
    resp.which_result = zmk_pointing_SetSensitivityResponse_ok_tag;
    resp.result.ok = true;

    LOG_INF("set_sensitivity: success, effective CPI=%u scroll=%u/%u inv=%u",
            compute_effective_cpi(),
            pointing_settings.scroll_numerator,
            pointing_settings.scroll_denominator,
            pointing_settings.scroll_inverted);
    return POINTING_RESPONSE(set_sensitivity, resp);
}

static int pointing_settings_reset(void) {
    pointing_settings.cursor_numerator = 1;
    pointing_settings.cursor_denominator = 1;
    pointing_settings.scroll_numerator = 1;
    pointing_settings.scroll_denominator = 1;
    pointing_settings.cpi = 0;
    pointing_settings.scroll_inverted = 0;
    pointing_settings.aml_enabled = 1;

    /* Apply defaults */
    apply_sensitivity();
    zmk_temp_layer_set_aml_enabled(true);

    return pointing_settings_save();
}

ZMK_RPC_SUBSYSTEM_SETTINGS_RESET(pointing, pointing_settings_reset);

zmk_studio_Response get_auto_layer(const zmk_studio_Request *req) {
    LOG_DBG("get_auto_layer called");

    zmk_pointing_GetAutoLayerResponse resp =
        zmk_pointing_GetAutoLayerResponse_init_zero;

    resp.enabled = zmk_temp_layer_get_aml_enabled();

    LOG_INF("get_auto_layer: enabled=%d", resp.enabled);

    return POINTING_RESPONSE(get_auto_layer, resp);
}

zmk_studio_Response set_auto_layer(const zmk_studio_Request *req) {
    LOG_DBG("set_auto_layer called");

    const zmk_pointing_SetAutoLayerRequest *set_req =
        &req->subsystem.pointing.request_type.set_auto_layer;

    LOG_INF("set_auto_layer: enabled=%d", set_req->enabled);

    /* Apply immediately */
    zmk_temp_layer_set_aml_enabled(set_req->enabled);

    /* Persist to flash */
    pointing_settings.aml_enabled = set_req->enabled ? 1 : 0;
    int ret = pointing_settings_save();
    if (ret < 0) {
        LOG_WRN("Failed to save AML setting: %d", ret);
        zmk_pointing_SetAutoLayerResponse resp =
            zmk_pointing_SetAutoLayerResponse_init_zero;
        resp.which_result = zmk_pointing_SetAutoLayerResponse_err_tag;
        resp.result.err = zmk_pointing_SetAutoLayerErrorCode_SET_AUTO_LAYER_ERR_STORAGE;
        return POINTING_RESPONSE(set_auto_layer, resp);
    }

    zmk_pointing_SetAutoLayerResponse resp =
        zmk_pointing_SetAutoLayerResponse_init_zero;
    resp.which_result = zmk_pointing_SetAutoLayerResponse_ok_tag;
    resp.result.ok = true;

    LOG_INF("set_auto_layer: success, enabled=%d", set_req->enabled);
    return POINTING_RESPONSE(set_auto_layer, resp);
}

ZMK_RPC_SUBSYSTEM_HANDLER(pointing, get_sensitivity, ZMK_STUDIO_RPC_HANDLER_SECURED);
ZMK_RPC_SUBSYSTEM_HANDLER(pointing, set_sensitivity, ZMK_STUDIO_RPC_HANDLER_SECURED);
ZMK_RPC_SUBSYSTEM_HANDLER(pointing, get_auto_layer, ZMK_STUDIO_RPC_HANDLER_SECURED);
ZMK_RPC_SUBSYSTEM_HANDLER(pointing, set_auto_layer, ZMK_STUDIO_RPC_HANDLER_SECURED);

/* ===== AML (Auto Mouse Layer) RPC Handlers ===== */

extern int zmk_temp_layer_get_config(int16_t *require_prior_idle_ms_out,
                                     uint32_t *excluded_positions_out, size_t max_positions,
                                     size_t *num_positions_out);
extern int zmk_temp_layer_set_config(int16_t require_prior_idle_ms,
                                     const uint32_t *excluded_positions, size_t num_positions);

#define AML_MAX_EXCLUDED_POSITIONS 40

zmk_studio_Response get_auto_layer(const zmk_studio_Request *req) {
    zmk_pointing_GetAutoLayerResponse resp = zmk_pointing_GetAutoLayerResponse_init_zero;
    resp.enabled = true;

    int16_t idle_ms = 0;
    uint32_t positions[AML_MAX_EXCLUDED_POSITIONS];
    size_t num_positions = 0;

    int ret = zmk_temp_layer_get_config(&idle_ms, positions, AML_MAX_EXCLUDED_POSITIONS,
                                        &num_positions);
    if (ret == 0) {
        resp.require_prior_idle_ms = (uint32_t)idle_ms;
        resp.excluded_positions_count = (uint32_t)num_positions;
        for (size_t i = 0; i < num_positions && i < AML_MAX_EXCLUDED_POSITIONS; i++) {
            resp.excluded_positions[i] = positions[i];
        }
    }

    LOG_INF("get_auto_layer: idle_ms=%d excluded_count=%zu", idle_ms, num_positions);
    return POINTING_RESPONSE(get_auto_layer, resp);
}

zmk_studio_Response set_auto_layer(const zmk_studio_Request *req) {
    const zmk_pointing_SetAutoLayerRequest *set_req =
        &req->subsystem.pointing.request_type.set_auto_layer;

    zmk_pointing_SetAutoLayerResponse resp = zmk_pointing_SetAutoLayerResponse_init_zero;

    int16_t idle_ms = (int16_t)set_req->require_prior_idle_ms;

    uint32_t positions[AML_MAX_EXCLUDED_POSITIONS];
    size_t num_positions = 0;
    for (size_t i = 0; i < set_req->excluded_positions_count && i < AML_MAX_EXCLUDED_POSITIONS; i++) {
        positions[i] = set_req->excluded_positions[i];
        num_positions++;
    }

    LOG_INF("set_auto_layer: idle_ms=%d excluded_count=%zu", idle_ms, num_positions);

    int ret = zmk_temp_layer_set_config(idle_ms, positions, num_positions);
    if (ret < 0) {
        LOG_WRN("set_auto_layer: zmk_temp_layer_set_config failed: %d", ret);
        resp.which_result = zmk_pointing_SetAutoLayerResponse_err_tag;
        resp.result.err = zmk_pointing_SetAutoLayerErrorCode_SET_AUTO_LAYER_ERR_UNSUPPORTED;
        return POINTING_RESPONSE(set_auto_layer, resp);
    }

    resp.which_result = zmk_pointing_SetAutoLayerResponse_ok_tag;
    resp.result.ok = true;
    return POINTING_RESPONSE(set_auto_layer, resp);
}

ZMK_RPC_SUBSYSTEM_HANDLER(pointing, get_auto_layer, ZMK_STUDIO_RPC_HANDLER_SECURED);
ZMK_RPC_SUBSYSTEM_HANDLER(pointing, set_auto_layer, ZMK_STUDIO_RPC_HANDLER_SECURED);

static int event_mapper(const zmk_event_t *eh, zmk_studio_Notification *n) { return 0; }

ZMK_RPC_EVENT_MAPPER(pointing, event_mapper);
