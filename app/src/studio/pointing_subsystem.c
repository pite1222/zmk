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
 * Persistent sensitivity settings.
 * numerator/denominator form a rational multiplier.
 * Default is 1/1 (no scaling change), CPI = 0 means "use default".
 */
static struct {
    uint32_t cursor_numerator;
    uint32_t cursor_denominator;
    uint32_t scroll_numerator;
    uint32_t scroll_denominator;
    uint32_t cpi;
} pointing_settings = {
    .cursor_numerator = 1,
    .cursor_denominator = 1,
    .scroll_numerator = 1,
    .scroll_denominator = 1,
    .cpi = 0,
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
            return -EINVAL;
        }
        int rc = read_cb(cb_arg, &pointing_settings, sizeof(pointing_settings));
        if (rc >= 0) {
            LOG_INF("Loaded pointing settings: cursor=%u/%u scroll=%u/%u cpi=%u",
                    pointing_settings.cursor_numerator,
                    pointing_settings.cursor_denominator,
                    pointing_settings.scroll_numerator,
                    pointing_settings.scroll_denominator,
                    pointing_settings.cpi);
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
 * Uses the PMW3610 sensor_attr_set API to change CPI at runtime.
 */
static void apply_sensitivity(void) {
    uint32_t effective_cpi = compute_effective_cpi();

    LOG_INF("Applying sensitivity: cursor=%u/%u scroll=%u/%u effective_cpi=%u",
            pointing_settings.cursor_numerator,
            pointing_settings.cursor_denominator,
            pointing_settings.scroll_numerator,
            pointing_settings.scroll_denominator,
            effective_cpi);

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
    /* Settings are loaded by this point, apply them */
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

    /* Always return scroll with valid defaults so the client knows scroll is supported */
    resp.scroll.numerator = pointing_settings.scroll_numerator;
    resp.scroll.denominator = pointing_settings.scroll_denominator;

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

    LOG_INF("get_sensitivity: cursor=%u/%u scroll=%u/%u cpi=%u",
            resp.cursor.numerator, resp.cursor.denominator,
            resp.scroll.numerator, resp.scroll.denominator,
            resp.cpi);

    return POINTING_RESPONSE(get_sensitivity, resp);
}

zmk_studio_Response set_sensitivity(const zmk_studio_Request *req) {
    LOG_DBG("set_sensitivity called");

    const zmk_pointing_SetSensitivityRequest *set_req =
        &req->subsystem.pointing.request_type.set_sensitivity;

    LOG_INF("set_sensitivity: cursor=%u/%u scroll=%u/%u cpi=%u",
            set_req->cursor.numerator, set_req->cursor.denominator,
            set_req->scroll.numerator, set_req->scroll.denominator,
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

    /* Update scroll settings - if not provided (both 0), keep existing or use 1/1 default */
    if (set_req->scroll.denominator > 0) {
        pointing_settings.scroll_numerator = set_req->scroll.numerator;
        pointing_settings.scroll_denominator = set_req->scroll.denominator;
    } else if (pointing_settings.scroll_denominator == 0) {
        /* Ensure we always have a valid scroll setting */
        pointing_settings.scroll_numerator = 1;
        pointing_settings.scroll_denominator = 1;
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

    LOG_INF("set_sensitivity: success, effective CPI=%u", compute_effective_cpi());
    return POINTING_RESPONSE(set_sensitivity, resp);
}

static int pointing_settings_reset(void) {
    pointing_settings.cursor_numerator = 1;
    pointing_settings.cursor_denominator = 1;
    pointing_settings.scroll_numerator = 1;
    pointing_settings.scroll_denominator = 1;
    pointing_settings.cpi = 0;

    /* Apply default CPI */
    apply_sensitivity();

    return pointing_settings_save();
}

ZMK_RPC_SUBSYSTEM_SETTINGS_RESET(pointing, pointing_settings_reset);

ZMK_RPC_SUBSYSTEM_HANDLER(pointing, get_sensitivity, ZMK_STUDIO_RPC_HANDLER_SECURED);
ZMK_RPC_SUBSYSTEM_HANDLER(pointing, set_sensitivity, ZMK_STUDIO_RPC_HANDLER_SECURED);

static int event_mapper(const zmk_event_t *eh, zmk_studio_Notification *n) { return 0; }

ZMK_RPC_EVENT_MAPPER(pointing, event_mapper);
