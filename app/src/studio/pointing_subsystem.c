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
extern int zmk_temp_layer_set_config(int16_t idle_ms, const uint32_t *positions, size_t num_positions);
extern int zmk_temp_layer_get_config(int16_t *idle_ms, uint32_t *positions, size_t max_positions, size_t *num_positions);
extern uint16_t zmk_temp_layer_get_motion_threshold(void);
extern void zmk_temp_layer_set_motion_threshold(uint16_t threshold);
extern uint32_t zmk_temp_layer_get_deactivate_timeout(void);
extern void zmk_temp_layer_set_deactivate_timeout(uint32_t ms);

#if IS_ENABLED(CONFIG_CONDUCTOR_KEY_REPEAT)
#include <zmk/conductor_key_repeat.h>
#endif

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
 * Acceleration parameter bounds. The accel processor multiplies deltas by a
 * Q8 gain in int32 and casts back to int16, so an unbounded max_milli from a
 * buggy/hostile Studio client could overflow (gain math UB, int16 wrap =
 * cursor direction flip). Clamp like CPI is clamped via normalize_cpi().
 */
#define ACCEL_MAX_MILLI_MIN 1000 /* 1.0x — never attenuate */
#define ACCEL_MAX_MILLI_MAX 5000 /* 5.0x */
#define ACCEL_THRESHOLD_MAX 100  /* counts/poll */
#define ACCEL_RANGE_MIN 1
#define ACCEL_RANGE_MAX 200
#define ACCEL_MIN_MILLI_MIN 200  /* 0.2x — floor for the low-speed gain */
#define ACCEL_MIN_MILLI_MAX 2000 /* 2.0x — ceiling for the low-speed boost */
#define ACCEL_SLOW_RANGE_MAX 100 /* counts/poll */

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
 * Global variables for the studio_pointer_scaler input processor.
 * Active only on the precision layer override; default 1/1 = no scaling.
 */
volatile int32_t studio_pointer_numerator = 1;
volatile int32_t studio_pointer_denominator = 1;

/*
 * Global variables for the studio_accel input processor (pointer acceleration).
 * Read at runtime by input_processor_studio_accel.c. enabled=0 => passthrough.
 * max_milli is the maximum gain x1000 (2000 = 2.0x).
 */
volatile int32_t studio_accel_enabled = 0;
volatile int32_t studio_accel_max_milli = 2000;
volatile int32_t studio_accel_threshold = 4;
volatile int32_t studio_accel_range = 16;
volatile int32_t studio_accel_min_milli = 1000; /* low-speed gain x1000 (1000 = off) */
volatile int32_t studio_accel_slow_range = 6;   /* counts/poll span of min->1.0x ramp */

/*
 * Persistent sensitivity settings.
 * numerator/denominator form a rational multiplier.
 * Default is 1/1 (no scaling change), CPI = 0 means "use default".
 * scroll_inverted: 0 = normal, 1 = inverted (natural scrolling)
 */
#define AML_SETTINGS_MAX_EXCLUDED 40

static struct {
    uint32_t cursor_numerator;
    uint32_t cursor_denominator;
    uint32_t scroll_numerator;
    uint32_t scroll_denominator;
    uint32_t cpi;
    uint32_t scroll_inverted;
    uint32_t aml_enabled;
    int16_t aml_idle_ms;
    uint8_t aml_excluded_count;
    uint8_t aml_excluded_positions[AML_SETTINGS_MAX_EXCLUDED];
    uint32_t precision_numerator;
    uint32_t precision_denominator;
    uint32_t accel_enabled;
    uint32_t accel_max_milli;
    uint32_t accel_threshold;
    uint32_t accel_range;
    /* Appended in 0.6.10 (the settings migration zero-fills them from older
     * blobs; apply_accel() maps 0 to the defaults). */
    uint32_t accel_min_milli;
    uint32_t accel_slow_range;
} pointing_settings = {
    .cursor_numerator = 1,
    .cursor_denominator = 1,
    .scroll_numerator = 1,
    .scroll_denominator = 1,
    .cpi = 0,
    .scroll_inverted = 0,
    .aml_enabled = 0,
    .aml_idle_ms = 300,
    .aml_excluded_count = 0,
    .precision_numerator = 1,
    .precision_denominator = 4,
    // Shipping default (v0.6.10, hardware-tuned 2026-07-06): gentle
    // acceleration (1.3x from 14 counts/poll over 21) plus a low-speed boost
    // (1.7x under 7 counts/poll) for a lighter feel on slow movements.
    // Applies to freshly-flashed or settings-reset devices only — devices
    // upgrading with a saved blob keep their accel values and get the
    // low-speed zone OFF (see the migration seeds in pointing_settings_set).
    .accel_enabled = 1,
    .accel_max_milli = 1300,
    .accel_threshold = 14,
    .accel_range = 21,
    .accel_min_milli = 1700,
    .accel_slow_range = 7,
};

/* AML-specific persistent storage */
static struct {
    uint8_t enabled;
    int16_t idle_ms;
    uint8_t excluded_count;
    uint8_t excluded_positions[AML_SETTINGS_MAX_EXCLUDED];
    uint16_t motion_threshold;
    uint32_t deactivate_timeout_ms;
/* Ships with AML OFF: since the mouse buttons moved onto the base layer, the
 * auto mouse layer is transparent and only costs an extra layer switch per
 * ball movement. Owners who want it back use the A+M+L combo or Studio; the
 * choice persists here. */
} aml_persist = {
    .enabled = 0,
    .idle_ms = 300,
    .excluded_count = 0,
    .motion_threshold = 0,
    .deactivate_timeout_ms = 0,
};

#if IS_ENABLED(CONFIG_CONDUCTOR_KEY_REPEAT)
/* Key auto-repeat persistent storage. delay_ms / interval_ms of 0 select the
 * firmware default (see zmk_key_repeat_set_config). */
static struct {
    uint8_t enabled;
    uint32_t delay_ms;
    uint32_t interval_ms;
} __packed key_repeat_persist = {
    .enabled = 0,
    .delay_ms = 0,
    .interval_ms = 0,
};
#endif

/* Forward declaration for the settings handler */
static int pointing_settings_set(const char *name, size_t len,
                                  settings_read_cb read_cb, void *cb_arg);
static void apply_sensitivity(void);
static void apply_precision(void);
static void apply_accel(void);

SETTINGS_STATIC_HANDLER_DEFINE(zmk_pointing_studio, "pointing/studio",
                                NULL, pointing_settings_set, NULL, NULL);

static int pointing_settings_set(const char *name, size_t len,
                                  settings_read_cb read_cb, void *cb_arg) {
    if (strcmp(name, "sensitivity") == 0) {
        if (len != sizeof(pointing_settings)) {
            /* Handle migration: read what we can, zero-fill the rest */
            size_t read_len = len < sizeof(pointing_settings) ? len : sizeof(pointing_settings);
            memset(&pointing_settings, 0, sizeof(pointing_settings));
            /* Set defaults for new fields */
            pointing_settings.cursor_numerator = 1;
            pointing_settings.cursor_denominator = 1;
            pointing_settings.scroll_numerator = 1;
            pointing_settings.scroll_denominator = 1;
            pointing_settings.aml_enabled = 1;
            /* Seed accel defaults too: a blob that predates these fields must
             * get the factory acceleration instead of the zero-fill
             * (enabled=0) reaching apply_accel() below. Blobs that do contain
             * them are simply overwritten by read_cb. The low-speed zone is
             * deliberately seeded OFF (1000): devices upgrading from an older
             * blob keep their familiar slow-speed feel — the boost default is
             * for fresh/reset devices only (user decision 2026-07-06). */
            pointing_settings.accel_enabled = 1;
            pointing_settings.accel_max_milli = 1300;
            pointing_settings.accel_threshold = 14;
            pointing_settings.accel_range = 21;
            pointing_settings.accel_min_milli = 1000;
            pointing_settings.accel_slow_range = 7;
            /* Read old data over the defaults */
            int rc = read_cb(cb_arg, &pointing_settings, read_len);
            if (rc >= 0) {
                LOG_INF("Migrated pointing settings from %zu to %zu bytes", len, sizeof(pointing_settings));
                studio_scroll_numerator = (int32_t)pointing_settings.scroll_numerator;
                studio_scroll_denominator = (int32_t)pointing_settings.scroll_denominator;
                studio_scroll_inverted = (pointing_settings.scroll_inverted != 0);
                apply_sensitivity();
                apply_precision();
                apply_accel();
            }
            return rc;
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
            /* Apply immediately — settings_load() runs after SYS_INIT */
            studio_scroll_numerator = (int32_t)pointing_settings.scroll_numerator;
            studio_scroll_denominator = (int32_t)pointing_settings.scroll_denominator;
            studio_scroll_inverted = (pointing_settings.scroll_inverted != 0);
            apply_sensitivity();
            /* Also re-apply precision + accel globals: pointing_studio_init
             * (SYS_INIT) ran before settings_load(), so without this the
             * saved values sat in pointing_settings without reaching the
             * input processors until the next Studio RPC. */
            apply_precision();
            apply_accel();
            LOG_INF("Applied pointing settings from flash: inverted=%d", (int)studio_scroll_inverted);
        }
        return rc;
    }
    if (strcmp(name, "aml") == 0) {
        if (len > sizeof(aml_persist)) len = sizeof(aml_persist);
        int rc = read_cb(cb_arg, &aml_persist, len);
        if (rc >= 0) {
            LOG_INF("Loaded AML settings: enabled=%u idle_ms=%d excluded=%u motion=%u timeout=%u",
                    aml_persist.enabled, aml_persist.idle_ms, aml_persist.excluded_count,
                    aml_persist.motion_threshold, aml_persist.deactivate_timeout_ms);
            /* Apply immediately — settings_load() runs after SYS_INIT */
            zmk_temp_layer_set_aml_enabled(aml_persist.enabled != 0);
            if (aml_persist.excluded_count > 0) {
                uint32_t positions[AML_SETTINGS_MAX_EXCLUDED];
                for (size_t i = 0; i < aml_persist.excluded_count; i++) {
                    positions[i] = aml_persist.excluded_positions[i];
                }
                zmk_temp_layer_set_config(aml_persist.idle_ms, positions, aml_persist.excluded_count);
            }
            if (aml_persist.motion_threshold > 0) {
                zmk_temp_layer_set_motion_threshold(aml_persist.motion_threshold);
            }
            if (aml_persist.deactivate_timeout_ms > 0) {
                zmk_temp_layer_set_deactivate_timeout(aml_persist.deactivate_timeout_ms);
            }
            LOG_INF("Applied AML from settings: enabled=%u", aml_persist.enabled);
        }
        return rc;
    }
#if IS_ENABLED(CONFIG_CONDUCTOR_KEY_REPEAT)
    if (strcmp(name, "key_repeat") == 0) {
        if (len != sizeof(key_repeat_persist)) {
            /* Handle migration: read what we can, zero-fill the rest (0 =
             * firmware default when applied). */
            size_t read_len = len < sizeof(key_repeat_persist) ? len : sizeof(key_repeat_persist);
            memset(&key_repeat_persist, 0, sizeof(key_repeat_persist));
            int rc = read_cb(cb_arg, &key_repeat_persist, read_len);
            if (rc >= 0) {
                LOG_INF("Migrated key_repeat settings from %zu to %zu bytes", len,
                        sizeof(key_repeat_persist));
                zmk_key_repeat_set_config(key_repeat_persist.enabled != 0,
                                          key_repeat_persist.delay_ms,
                                          key_repeat_persist.interval_ms);
            }
            return rc;
        }
        int rc = read_cb(cb_arg, &key_repeat_persist, sizeof(key_repeat_persist));
        if (rc >= 0) {
            LOG_INF("Loaded key_repeat settings: enabled=%u delay=%u interval=%u",
                    key_repeat_persist.enabled, key_repeat_persist.delay_ms,
                    key_repeat_persist.interval_ms);
            /* Apply immediately — settings_load() runs after SYS_INIT */
            zmk_key_repeat_set_config(key_repeat_persist.enabled != 0,
                                      key_repeat_persist.delay_ms,
                                      key_repeat_persist.interval_ms);
        }
        return rc;
    }
#endif
    return -ENOENT;
}

/* Save sensitivity settings (original fields only, for backwards compat) */
static int pointing_settings_save(void) {
    return settings_save_one("pointing/studio/sensitivity",
                              &pointing_settings, sizeof(pointing_settings));
}

/* Public API: toggle scroll invert (natural scrolling) via a keymap behavior.
 * Mirrors the persistence path of set_sensitivity so the value survives reboot
 * and the next get_sensitivity over Studio RPC reflects the change. */
void zmk_pointing_toggle_scroll_invert(void) {
    pointing_settings.scroll_inverted = pointing_settings.scroll_inverted ? 0 : 1;
    studio_scroll_inverted = (pointing_settings.scroll_inverted != 0);
    int rc = pointing_settings_save();
    if (rc < 0) {
        LOG_WRN("Failed to persist scroll_inverted toggle: %d", rc);
    }
    LOG_INF("Toggled scroll invert: %d", (int)studio_scroll_inverted);
}

static int aml_settings_save(void) {
    return settings_save_one("pointing/studio/aml",
                              &aml_persist, sizeof(aml_persist));
}

#if IS_ENABLED(CONFIG_CONDUCTOR_KEY_REPEAT)
static int key_repeat_settings_save(void) {
    return settings_save_one("pointing/studio/key_repeat",
                              &key_repeat_persist, sizeof(key_repeat_persist));
}
#endif

void zmk_pointing_toggle_aml(void) {
    bool new_enabled = !zmk_temp_layer_get_aml_enabled();
    zmk_temp_layer_set_aml_enabled(new_enabled);
    aml_persist.enabled = new_enabled ? 1 : 0;
    int rc = aml_settings_save();
    if (rc < 0) {
        LOG_WRN("Failed to persist AML toggle: %d", rc);
    }
    LOG_INF("Toggled AML: %d", (int)new_enabled);
}

static uint32_t normalize_cpi(uint32_t cpi) {
    if (cpi < PMW3610_MIN_CPI) {
        cpi = PMW3610_MIN_CPI;
    }
    if (cpi > PMW3610_MAX_CPI) {
        cpi = PMW3610_MAX_CPI;
    }

    return ((cpi + PMW3610_CPI_STEP / 2) / PMW3610_CPI_STEP) * PMW3610_CPI_STEP;
}

/*
 * Compute the user-facing CPI based on cursor sensitivity ratio.
 *
 * If an explicit CPI is set (pointing_settings.cpi > 0), use that directly.
 * Otherwise, compute: user_cpi = DEFAULT_CPI * numerator / denominator.
 */
static uint32_t compute_user_cpi(void) {
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

    return normalize_cpi(cpi);
}

static uint32_t compute_sensor_cpi(uint32_t user_cpi) {
    uint32_t den = CONFIG_ZMK_POINTING_SENSOR_CPI_MULTIPLIER_DEN;
    if (den == 0) {
        den = 1;
    }

    uint64_t scaled = (uint64_t)user_cpi * CONFIG_ZMK_POINTING_SENSOR_CPI_MULTIPLIER_NUM;
    scaled = (scaled + den / 2) / den;

    return normalize_cpi((uint32_t)scaled);
}

/*
 * Apply the current sensitivity settings to the running system.
 * - CPI: Uses the PMW3610 sensor_attr_set API to change CPI at runtime.
 * - Scroll: Updates the global variables read by studio_scroll_scaler.
 */
static void apply_sensitivity(void) {
    uint32_t user_cpi = compute_user_cpi();
    uint32_t sensor_cpi = compute_sensor_cpi(user_cpi);

    LOG_INF("Applying sensitivity: cursor=%u/%u scroll=%u/%u cpi=%u sensor_cpi=%u inv=%u",
            pointing_settings.cursor_numerator,
            pointing_settings.cursor_denominator,
            pointing_settings.scroll_numerator,
            pointing_settings.scroll_denominator,
            user_cpi,
            sensor_cpi,
            pointing_settings.scroll_inverted);

    /* Update scroll globals for the studio_scaler input processor */
    studio_scroll_numerator = (int32_t)pointing_settings.scroll_numerator;
    studio_scroll_denominator = (int32_t)pointing_settings.scroll_denominator;
    studio_scroll_inverted = (pointing_settings.scroll_inverted != 0);

    /* Update precision globals for the studio_pointer_scaler input processor */
    studio_pointer_numerator = (int32_t)pointing_settings.precision_numerator;
    studio_pointer_denominator = (int32_t)pointing_settings.precision_denominator;

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
        .val1 = (int32_t)sensor_cpi,
        .val2 = 0,
    };

    int err = sensor_attr_set(trackball_dev, SENSOR_CHAN_ALL,
                               (enum sensor_attribute)PMW3610_ALT_ATTR_CPI, &val);
    if (err) {
        LOG_ERR("Failed to set sensor CPI to %u: %d", sensor_cpi, err);
    } else {
        LOG_INF("Sensor CPI set to %u successfully", sensor_cpi);
    }
#else
    LOG_WRN("No trackball device available, CPI change not applied");
#endif
}

/*
 * Push the stored precision-mode scale into the runtime globals read by the
 * studio_pointer_scaler input processor (default 1/4 if storage is empty).
 */
static void apply_precision(void) {
    if (pointing_settings.precision_denominator == 0) {
        pointing_settings.precision_numerator = 1;
        pointing_settings.precision_denominator = 4;
    }
    studio_pointer_numerator = (int32_t)pointing_settings.precision_numerator;
    studio_pointer_denominator = (int32_t)pointing_settings.precision_denominator;
}

/*
 * Push the stored acceleration parameters into the runtime globals read by the
 * studio_accel input processor. Fills in sane defaults for fields that were
 * zero-filled by the settings migration path (older saved blobs).
 */
static void apply_accel(void) {
    if (pointing_settings.accel_max_milli == 0) {
        pointing_settings.accel_max_milli = 2000;
    }
    if (pointing_settings.accel_range == 0) {
        pointing_settings.accel_range = 16;
    }
    if (pointing_settings.accel_min_milli == 0) {
        /* Zero = unset (old blob / old client): low-speed zone off. */
        pointing_settings.accel_min_milli = 1000;
    }
    if (pointing_settings.accel_slow_range == 0) {
        pointing_settings.accel_slow_range = 7;
    }

    /* Clamp to sane bounds; covers both the set_accel RPC path (called before
     * the settings save, so the clamped values are what get persisted) and
     * values loaded from an old/corrupt settings blob. */
    if (pointing_settings.accel_max_milli < ACCEL_MAX_MILLI_MIN) {
        pointing_settings.accel_max_milli = ACCEL_MAX_MILLI_MIN;
    }
    if (pointing_settings.accel_max_milli > ACCEL_MAX_MILLI_MAX) {
        pointing_settings.accel_max_milli = ACCEL_MAX_MILLI_MAX;
    }
    if (pointing_settings.accel_threshold > ACCEL_THRESHOLD_MAX) {
        pointing_settings.accel_threshold = ACCEL_THRESHOLD_MAX;
    }
    if (pointing_settings.accel_range < ACCEL_RANGE_MIN) {
        pointing_settings.accel_range = ACCEL_RANGE_MIN;
    }
    if (pointing_settings.accel_range > ACCEL_RANGE_MAX) {
        pointing_settings.accel_range = ACCEL_RANGE_MAX;
    }
    if (pointing_settings.accel_min_milli < ACCEL_MIN_MILLI_MIN) {
        pointing_settings.accel_min_milli = ACCEL_MIN_MILLI_MIN;
    }
    if (pointing_settings.accel_min_milli > ACCEL_MIN_MILLI_MAX) {
        pointing_settings.accel_min_milli = ACCEL_MIN_MILLI_MAX;
    }
    if (pointing_settings.accel_slow_range > ACCEL_SLOW_RANGE_MAX) {
        pointing_settings.accel_slow_range = ACCEL_SLOW_RANGE_MAX;
    }

    studio_accel_enabled = (int32_t)pointing_settings.accel_enabled;
    studio_accel_max_milli = (int32_t)pointing_settings.accel_max_milli;
    studio_accel_threshold = (int32_t)pointing_settings.accel_threshold;
    studio_accel_range = (int32_t)pointing_settings.accel_range;
    studio_accel_min_milli = (int32_t)pointing_settings.accel_min_milli;
    studio_accel_slow_range = (int32_t)pointing_settings.accel_slow_range;

    LOG_INF("Applying accel: enabled=%d max_milli=%d threshold=%d range=%d min_milli=%d slow_range=%d",
            (int)studio_accel_enabled, (int)studio_accel_max_milli,
            (int)studio_accel_threshold, (int)studio_accel_range,
            (int)studio_accel_min_milli, (int)studio_accel_slow_range);
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

    /* Apply precision scale globals (default 1/4 if storage empty) */
    apply_precision();

    /* Apply acceleration globals (defaults if storage empty) */
    apply_accel();

    /* Restore AML from dedicated aml_persist storage */
    zmk_temp_layer_set_aml_enabled(aml_persist.enabled != 0);
    if (aml_persist.excluded_count > 0) {
        uint32_t positions[AML_SETTINGS_MAX_EXCLUDED];
        for (size_t i = 0; i < aml_persist.excluded_count; i++) {
            positions[i] = aml_persist.excluded_positions[i];
        }
        zmk_temp_layer_set_config(aml_persist.idle_ms, positions, aml_persist.excluded_count);
    }
    if (aml_persist.motion_threshold > 0) {
        zmk_temp_layer_set_motion_threshold(aml_persist.motion_threshold);
    }
    if (aml_persist.deactivate_timeout_ms > 0) {
        zmk_temp_layer_set_deactivate_timeout(aml_persist.deactivate_timeout_ms);
    }
    LOG_INF("Restored AML: enabled=%u idle_ms=%d excluded=%u motion=%u timeout=%u",
            aml_persist.enabled, aml_persist.idle_ms, aml_persist.excluded_count,
            aml_persist.motion_threshold, aml_persist.deactivate_timeout_ms);

#if IS_ENABLED(CONFIG_CONDUCTOR_KEY_REPEAT)
    /* Apply key-repeat defaults on boot; the settings load (after SYS_INIT)
     * overrides these with the persisted blob if one exists. */
    zmk_key_repeat_set_config(key_repeat_persist.enabled != 0, key_repeat_persist.delay_ms,
                              key_repeat_persist.interval_ms);
#endif

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

    /* has_cursor / has_scroll must be set for nanopb to serialize these
     * submessages onto the wire. Same nanopb pitfall as the combo binding
     * fix — without these, clients receive an empty cursor/scroll and
     * the (?? 1) fallback in the client makes sensitivity look like 1/1
     * even when the keyboard has it configured differently. */
    resp.has_cursor = true;
    resp.cursor.numerator = pointing_settings.cursor_numerator;
    resp.cursor.denominator = pointing_settings.cursor_denominator;

    /* Return scroll settings - use sign to encode inversion */
    resp.has_scroll = true;
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

    resp.cpi = compute_user_cpi();

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

    LOG_INF("set_sensitivity: success, user CPI=%u scroll=%u/%u inv=%u",
            compute_user_cpi(),
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
    pointing_settings.aml_enabled = 0;
    /* Match the shipping default (see the pointing_settings initializer). */
    pointing_settings.accel_enabled = 1;
    pointing_settings.accel_max_milli = 1300;
    pointing_settings.accel_threshold = 14;
    pointing_settings.accel_range = 21;
    pointing_settings.accel_min_milli = 1700;
    pointing_settings.accel_slow_range = 7;

    /* Apply defaults */
    apply_sensitivity();
    apply_accel();
    zmk_temp_layer_set_aml_enabled(false);
    /* Clear the live AML deactivation-timeout override too (0 = DTS default),
     * so reset takes effect immediately instead of only after reboot. */
    zmk_temp_layer_set_deactivate_timeout(0);

    /* Reset AML persist to defaults */
    aml_persist.enabled = 0;
    aml_persist.idle_ms = 300;
    aml_persist.excluded_count = 0;
    aml_persist.motion_threshold = 0;
    aml_persist.deactivate_timeout_ms = 0;
    aml_settings_save();

#if IS_ENABLED(CONFIG_CONDUCTOR_KEY_REPEAT)
    /* Reset key auto-repeat to shipped defaults (disabled, firmware timings). */
    key_repeat_persist.enabled = 0;
    key_repeat_persist.delay_ms = 0;
    key_repeat_persist.interval_ms = 0;
    zmk_key_repeat_set_config(false, 0, 0);
    int kr_rc = key_repeat_settings_save();
    if (kr_rc < 0) {
        LOG_WRN("Failed to persist key_repeat reset: %d", kr_rc);
    }
#endif

    /* Always run the sensitivity save, then surface the first failure. */
    int rc = pointing_settings_save();
#if IS_ENABLED(CONFIG_CONDUCTOR_KEY_REPEAT)
    if (kr_rc < 0) {
        return kr_rc;
    }
#endif
    return rc;
}

ZMK_RPC_SUBSYSTEM_SETTINGS_RESET(pointing, pointing_settings_reset);

ZMK_RPC_SUBSYSTEM_HANDLER(pointing, get_sensitivity, ZMK_STUDIO_RPC_HANDLER_SECURED);
ZMK_RPC_SUBSYSTEM_HANDLER(pointing, set_sensitivity, ZMK_STUDIO_RPC_HANDLER_SECURED);

/* ===== Precision Scale RPC Handlers ===== */

zmk_studio_Response get_precision_scale(const zmk_studio_Request *req) {
    zmk_pointing_GetPrecisionScaleResponse resp =
        zmk_pointing_GetPrecisionScaleResponse_init_zero;

    resp.has_precision = true;
    resp.precision.numerator = pointing_settings.precision_numerator;
    resp.precision.denominator = pointing_settings.precision_denominator;
    if (resp.precision.denominator == 0) {
        resp.precision.numerator = 1;
        resp.precision.denominator = 4;
    }

    LOG_INF("get_precision_scale: %u/%u",
            resp.precision.numerator, resp.precision.denominator);
    return POINTING_RESPONSE(get_precision_scale, resp);
}

zmk_studio_Response set_precision_scale(const zmk_studio_Request *req) {
    const zmk_pointing_SetPrecisionScaleRequest *set_req =
        &req->subsystem.pointing.request_type.set_precision_scale;

    LOG_INF("set_precision_scale: %u/%u",
            set_req->precision.numerator, set_req->precision.denominator);

    if (set_req->precision.denominator == 0 || set_req->precision.numerator == 0) {
        zmk_pointing_SetPrecisionScaleResponse resp =
            zmk_pointing_SetPrecisionScaleResponse_init_zero;
        resp.which_result = zmk_pointing_SetPrecisionScaleResponse_err_tag;
        resp.result.err = zmk_pointing_SetPrecisionScaleErrorCode_SET_PRECISION_SCALE_ERR_INVALID;
        return POINTING_RESPONSE(set_precision_scale, resp);
    }

    pointing_settings.precision_numerator = set_req->precision.numerator;
    pointing_settings.precision_denominator = set_req->precision.denominator;

    /* Update runtime globals so the change takes effect immediately. */
    studio_pointer_numerator = (int32_t)pointing_settings.precision_numerator;
    studio_pointer_denominator = (int32_t)pointing_settings.precision_denominator;

    int ret = pointing_settings_save();
    if (ret < 0) {
        LOG_WRN("Failed to save precision scale: %d", ret);
        zmk_pointing_SetPrecisionScaleResponse resp =
            zmk_pointing_SetPrecisionScaleResponse_init_zero;
        resp.which_result = zmk_pointing_SetPrecisionScaleResponse_err_tag;
        resp.result.err = zmk_pointing_SetPrecisionScaleErrorCode_SET_PRECISION_SCALE_ERR_STORAGE;
        return POINTING_RESPONSE(set_precision_scale, resp);
    }

    zmk_pointing_SetPrecisionScaleResponse resp =
        zmk_pointing_SetPrecisionScaleResponse_init_zero;
    resp.which_result = zmk_pointing_SetPrecisionScaleResponse_ok_tag;
    resp.result.ok = true;
    return POINTING_RESPONSE(set_precision_scale, resp);
}

ZMK_RPC_SUBSYSTEM_HANDLER(pointing, get_precision_scale, ZMK_STUDIO_RPC_HANDLER_SECURED);
ZMK_RPC_SUBSYSTEM_HANDLER(pointing, set_precision_scale, ZMK_STUDIO_RPC_HANDLER_SECURED);

/* ===== Pointer Acceleration RPC Handlers ===== */

zmk_studio_Response get_accel(const zmk_studio_Request *req) {
    zmk_pointing_GetAccelResponse resp = zmk_pointing_GetAccelResponse_init_zero;

    /* has_accel must be set for nanopb to serialize the submessage. */
    resp.has_accel = true;
    resp.accel.enabled = (pointing_settings.accel_enabled != 0);
    resp.accel.max_milli =
        pointing_settings.accel_max_milli ? pointing_settings.accel_max_milli : 2000;
    resp.accel.threshold = pointing_settings.accel_threshold;
    resp.accel.range = pointing_settings.accel_range ? pointing_settings.accel_range : 16;
    resp.accel.min_milli =
        pointing_settings.accel_min_milli ? pointing_settings.accel_min_milli : 1000;
    resp.accel.slow_range =
        pointing_settings.accel_slow_range ? pointing_settings.accel_slow_range : 7;

    LOG_INF("get_accel: enabled=%d max_milli=%u threshold=%u range=%u min_milli=%u slow_range=%u",
            (int)resp.accel.enabled, resp.accel.max_milli, resp.accel.threshold,
            resp.accel.range, resp.accel.min_milli, resp.accel.slow_range);
    return POINTING_RESPONSE(get_accel, resp);
}

zmk_studio_Response set_accel(const zmk_studio_Request *req) {
    const zmk_pointing_SetAccelRequest *set_req =
        &req->subsystem.pointing.request_type.set_accel;

    LOG_INF("set_accel: enabled=%d max_milli=%u threshold=%u range=%u",
            (int)set_req->accel.enabled, set_req->accel.max_milli,
            set_req->accel.threshold, set_req->accel.range);

    pointing_settings.accel_enabled = set_req->accel.enabled ? 1 : 0;
    pointing_settings.accel_max_milli =
        set_req->accel.max_milli ? set_req->accel.max_milli : 2000;
    pointing_settings.accel_threshold = set_req->accel.threshold;
    pointing_settings.accel_range = set_req->accel.range ? set_req->accel.range : 16;
    /* 0 = field absent (an old Studio client) — keep the current value so
     * pre-0.6.10 clients don't silently wipe the low-speed precision zone. */
    if (set_req->accel.min_milli) {
        pointing_settings.accel_min_milli = set_req->accel.min_milli;
    }
    if (set_req->accel.slow_range) {
        pointing_settings.accel_slow_range = set_req->accel.slow_range;
    }

    /* Apply to running system FIRST (immediate feedback) */
    apply_accel();

    /* Then persist to flash */
    int ret = pointing_settings_save();
    if (ret < 0) {
        LOG_WRN("Failed to save accel settings: %d", ret);
        zmk_pointing_SetAccelResponse resp = zmk_pointing_SetAccelResponse_init_zero;
        resp.which_result = zmk_pointing_SetAccelResponse_err_tag;
        resp.result.err = zmk_pointing_SetSensitivityErrorCode_SET_SENSITIVITY_ERR_STORAGE;
        return POINTING_RESPONSE(set_accel, resp);
    }

    zmk_pointing_SetAccelResponse resp = zmk_pointing_SetAccelResponse_init_zero;
    resp.which_result = zmk_pointing_SetAccelResponse_ok_tag;
    resp.result.ok = true;
    return POINTING_RESPONSE(set_accel, resp);
}

ZMK_RPC_SUBSYSTEM_HANDLER(pointing, get_accel, ZMK_STUDIO_RPC_HANDLER_SECURED);
ZMK_RPC_SUBSYSTEM_HANDLER(pointing, set_accel, ZMK_STUDIO_RPC_HANDLER_SECURED);

#if IS_ENABLED(CONFIG_CONDUCTOR_KEY_REPEAT)
/* ===== Key Auto-Repeat (typematic) RPC Handlers ===== */

zmk_studio_Response get_key_repeat(const zmk_studio_Request *req) {
    zmk_pointing_GetKeyRepeatResponse resp = zmk_pointing_GetKeyRepeatResponse_init_zero;

    bool enabled = false;
    uint32_t delay_ms = 0, interval_ms = 0;
    zmk_key_repeat_get_config(&enabled, &delay_ms, &interval_ms);

    /* has_config must be set for nanopb to serialize the submessage. */
    resp.has_config = true;
    resp.config.enabled = enabled;
    resp.config.delay_ms = delay_ms;
    resp.config.interval_ms = interval_ms;

    LOG_INF("get_key_repeat: enabled=%d delay=%u interval=%u", (int)enabled, delay_ms, interval_ms);
    return POINTING_RESPONSE(get_key_repeat, resp);
}

zmk_studio_Response set_key_repeat(const zmk_studio_Request *req) {
    const zmk_pointing_SetKeyRepeatRequest *set_req =
        &req->subsystem.pointing.request_type.set_key_repeat;

    /* Reject a request with no config submessage: applying it would silently
     * disable key-repeat from a malformed/partial client message. */
    if (!set_req->has_config) {
        LOG_WRN("set_key_repeat: missing config, rejecting");
        zmk_pointing_SetKeyRepeatResponse resp = zmk_pointing_SetKeyRepeatResponse_init_zero;
        resp.which_result = zmk_pointing_SetKeyRepeatResponse_err_tag;
        resp.result.err = zmk_pointing_SetSensitivityErrorCode_SET_SENSITIVITY_ERR_INVALID;
        return POINTING_RESPONSE(set_key_repeat, resp);
    }

    bool enabled = set_req->config.enabled;
    uint32_t delay_ms = set_req->config.delay_ms;
    uint32_t interval_ms = set_req->config.interval_ms;

    LOG_INF("set_key_repeat: enabled=%d delay=%u interval=%u", (int)enabled, delay_ms, interval_ms);

    /* Apply to running system FIRST (immediate feedback). The setter clamps
     * delay/interval, but persist the raw requested values so a stored 0 keeps
     * meaning "firmware default" and follows future Kconfig default changes. */
    zmk_key_repeat_set_config(enabled, delay_ms, interval_ms);

    key_repeat_persist.enabled = set_req->config.enabled ? 1 : 0;
    key_repeat_persist.delay_ms = set_req->config.delay_ms;
    key_repeat_persist.interval_ms = set_req->config.interval_ms;

    int ret = key_repeat_settings_save();
    if (ret < 0) {
        LOG_WRN("Failed to save key_repeat settings: %d", ret);
        zmk_pointing_SetKeyRepeatResponse resp = zmk_pointing_SetKeyRepeatResponse_init_zero;
        resp.which_result = zmk_pointing_SetKeyRepeatResponse_err_tag;
        resp.result.err = zmk_pointing_SetSensitivityErrorCode_SET_SENSITIVITY_ERR_STORAGE;
        return POINTING_RESPONSE(set_key_repeat, resp);
    }

    zmk_pointing_SetKeyRepeatResponse resp = zmk_pointing_SetKeyRepeatResponse_init_zero;
    resp.which_result = zmk_pointing_SetKeyRepeatResponse_ok_tag;
    resp.result.ok = true;
    return POINTING_RESPONSE(set_key_repeat, resp);
}

ZMK_RPC_SUBSYSTEM_HANDLER(pointing, set_key_repeat, ZMK_STUDIO_RPC_HANDLER_SECURED);
ZMK_RPC_SUBSYSTEM_HANDLER(pointing, get_key_repeat, ZMK_STUDIO_RPC_HANDLER_SECURED);
#endif

/* ===== AML (Auto Mouse Layer) RPC Handlers ===== */

#define AML_MAX_EXCLUDED_POSITIONS 40

zmk_studio_Response get_auto_layer(const zmk_studio_Request *req) {
    zmk_pointing_GetAutoLayerResponse resp = zmk_pointing_GetAutoLayerResponse_init_zero;
    resp.enabled = zmk_temp_layer_get_aml_enabled();

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

    resp.motion_threshold = zmk_temp_layer_get_motion_threshold();
    resp.deactivate_timeout_ms = zmk_temp_layer_get_deactivate_timeout();

    LOG_INF("get_auto_layer: idle_ms=%d excluded_count=%zu motion=%u timeout=%u",
            idle_ms, num_positions, resp.motion_threshold, resp.deactivate_timeout_ms);
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

    LOG_INF("set_auto_layer: enabled=%d idle_ms=%d excluded_count=%zu motion=%u timeout=%u",
            (int)set_req->enabled, idle_ms, num_positions, set_req->motion_threshold,
            set_req->deactivate_timeout_ms);

    /* Apply excluded positions config */
    int ret = zmk_temp_layer_set_config(idle_ms, positions, num_positions);
    if (ret < 0) {
        LOG_WRN("set_auto_layer: zmk_temp_layer_set_config failed: %d", ret);
        resp.which_result = zmk_pointing_SetAutoLayerResponse_err_tag;
        resp.result.err = zmk_pointing_SetAutoLayerErrorCode_SET_AUTO_LAYER_ERR_UNSUPPORTED;
        return POINTING_RESPONSE(set_auto_layer, resp);
    }

    /* Apply motion threshold */
    if (set_req->motion_threshold > 0) {
        zmk_temp_layer_set_motion_threshold((uint16_t)set_req->motion_threshold);
    }

    /* Apply deactivation timeout unconditionally: 0 clears the runtime override
     * (falls back to the DTS default) and must take effect immediately, not just
     * after reboot, so the live layer and get_auto_layer stay consistent. */
    zmk_temp_layer_set_deactivate_timeout(set_req->deactivate_timeout_ms);

    /* Apply enabled state and persist to dedicated AML storage */
    zmk_temp_layer_set_aml_enabled(set_req->enabled);
    aml_persist.enabled = set_req->enabled ? 1 : 0;
    aml_persist.idle_ms = idle_ms;
    aml_persist.excluded_count = (uint8_t)(num_positions > AML_SETTINGS_MAX_EXCLUDED ? AML_SETTINGS_MAX_EXCLUDED : num_positions);
    for (size_t i = 0; i < aml_persist.excluded_count; i++) {
        aml_persist.excluded_positions[i] = (uint8_t)positions[i];
    }
    aml_persist.motion_threshold = (uint16_t)set_req->motion_threshold;
    aml_persist.deactivate_timeout_ms = set_req->deactivate_timeout_ms;
    int save_ret = aml_settings_save();
    LOG_INF("set_auto_layer: aml_save_ret=%d enabled=%u idle_ms=%d excluded=%u motion=%u timeout=%u size=%zu",
            save_ret, aml_persist.enabled, aml_persist.idle_ms,
            aml_persist.excluded_count, aml_persist.motion_threshold,
            aml_persist.deactivate_timeout_ms, sizeof(aml_persist));
    if (save_ret < 0) {
        LOG_ERR("set_auto_layer: aml_settings_save FAILED: %d", save_ret);
    }

    resp.which_result = zmk_pointing_SetAutoLayerResponse_ok_tag;
    resp.result.ok = true;
    return POINTING_RESPONSE(set_auto_layer, resp);
}

ZMK_RPC_SUBSYSTEM_HANDLER(pointing, get_auto_layer, ZMK_STUDIO_RPC_HANDLER_SECURED);
ZMK_RPC_SUBSYSTEM_HANDLER(pointing, set_auto_layer, ZMK_STUDIO_RPC_HANDLER_SECURED);

static int event_mapper(const zmk_event_t *eh, zmk_studio_Notification *n) { return -ENOTSUP; }

ZMK_RPC_EVENT_MAPPER(pointing, event_mapper);
