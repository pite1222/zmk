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
    LOG_DBG("");

    zmk_pointing_GetSensitivityResponse resp =
        zmk_pointing_GetSensitivityResponse_init_zero;

    resp.cursor.numerator = pointing_settings.cursor_numerator;
    resp.cursor.denominator = pointing_settings.cursor_denominator;
    resp.scroll.numerator = pointing_settings.scroll_numerator;
    resp.scroll.denominator = pointing_settings.scroll_denominator;
    resp.cpi = pointing_settings.cpi;

    return POINTING_RESPONSE(get_sensitivity, resp);
}

zmk_studio_Response set_sensitivity(const zmk_studio_Request *req) {
    LOG_DBG("");

    const zmk_pointing_SetSensitivityRequest *set_req =
        &req->subsystem.pointing.request_type.set_sensitivity;

    /* Validate: denominators must not be zero */
    if (set_req->cursor.denominator == 0 || set_req->scroll.denominator == 0) {
        zmk_pointing_SetSensitivityResponse resp =
            zmk_pointing_SetSensitivityResponse_init_zero;
        resp.which_result = zmk_pointing_SetSensitivityResponse_err_tag;
        resp.result.err = zmk_pointing_SetSensitivityErrorCode_SET_SENSITIVITY_ERR_INVALID;
        return POINTING_RESPONSE(set_sensitivity, resp);
    }

    /* Update in-memory settings */
    pointing_settings.cursor_numerator = set_req->cursor.numerator;
    pointing_settings.cursor_denominator = set_req->cursor.denominator;
    pointing_settings.scroll_numerator = set_req->scroll.numerator;
    pointing_settings.scroll_denominator = set_req->scroll.denominator;
    pointing_settings.cpi = set_req->cpi;

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
