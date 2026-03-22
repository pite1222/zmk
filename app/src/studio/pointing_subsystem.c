/*
 * Copyright (c) 2025 The Conductor Contributors
 *
 * SPDX-License-Identifier: MIT
 */

#include <zephyr/logging/log.h>

LOG_MODULE_DECLARE(zmk_studio, CONFIG_ZMK_STUDIO_LOG_LEVEL);

#include <zephyr/settings/settings.h>

#include <zmk/studio/rpc.h>

#include <pb_encode.h>

ZMK_RPC_SUBSYSTEM(pointing)

#define POINTING_RESPONSE(type, ...) ZMK_RPC_RESPONSE(pointing, type, __VA_ARGS__)

/*
 * Persistent sensitivity settings.
 * numerator/denominator form a rational multiplier applied to the input_processor_scaler.
 * Default is 1/1 (no scaling change).
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
        return read_cb(cb_arg, &pointing_settings, sizeof(pointing_settings));
    }
    return -ENOENT;
}

static int pointing_settings_save(void) {
    return settings_save_one("pointing/studio/sensitivity",
                              &pointing_settings, sizeof(pointing_settings));
}

/*
 * Apply the current sensitivity settings to the running system.
 * This function should update the input processor scaler parameters
 * at runtime. The exact mechanism depends on the board's devicetree
 * configuration and input processor driver API.
 *
 * For now, the settings are persisted and will take effect on next boot
 * via the input_processor_scaler devicetree configuration.
 *
 * TODO: Implement runtime scaler update via input_processor API if available.
 */
static void apply_sensitivity(void) {
    LOG_INF("Pointing sensitivity updated: cursor=%u/%u scroll=%u/%u cpi=%u",
            pointing_settings.cursor_numerator,
            pointing_settings.cursor_denominator,
            pointing_settings.scroll_numerator,
            pointing_settings.scroll_denominator,
            pointing_settings.cpi);
}

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

    resp.cpi = pointing_settings.cpi;

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

    /* Update CPI if provided (0 means "don't change") */
    if (set_req->cpi > 0) {
        pointing_settings.cpi = set_req->cpi;
    }

    /* Persist to flash */
    int ret = pointing_settings_save();
    if (ret < 0) {
        LOG_WRN("Failed to save pointing sensitivity settings: %d", ret);
        zmk_pointing_SetSensitivityResponse resp =
            zmk_pointing_SetSensitivityResponse_init_zero;
        resp.which_result = zmk_pointing_SetSensitivityResponse_err_tag;
        resp.result.err = zmk_pointing_SetSensitivityErrorCode_SET_SENSITIVITY_ERR_STORAGE;
        return POINTING_RESPONSE(set_sensitivity, resp);
    }

    /* Apply to running system */
    apply_sensitivity();

    zmk_pointing_SetSensitivityResponse resp =
        zmk_pointing_SetSensitivityResponse_init_zero;
    resp.which_result = zmk_pointing_SetSensitivityResponse_ok_tag;
    resp.result.ok = true;

    LOG_INF("set_sensitivity: success");
    return POINTING_RESPONSE(set_sensitivity, resp);
}

static int pointing_settings_reset(void) {
    pointing_settings.cursor_numerator = 1;
    pointing_settings.cursor_denominator = 1;
    pointing_settings.scroll_numerator = 1;
    pointing_settings.scroll_denominator = 1;
    pointing_settings.cpi = 0;
    return pointing_settings_save();
}

ZMK_RPC_SUBSYSTEM_SETTINGS_RESET(pointing, pointing_settings_reset);

ZMK_RPC_SUBSYSTEM_HANDLER(pointing, get_sensitivity, ZMK_STUDIO_RPC_HANDLER_SECURED);
ZMK_RPC_SUBSYSTEM_HANDLER(pointing, set_sensitivity, ZMK_STUDIO_RPC_HANDLER_SECURED);

static int event_mapper(const zmk_event_t *eh, zmk_studio_Notification *n) { return 0; }

ZMK_RPC_EVENT_MAPPER(pointing, event_mapper);
