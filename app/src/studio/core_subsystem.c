/*
 * Copyright (c) 2024 The ZMK Contributors
 *
 * SPDX-License-Identifier: MIT
 */

#include <zephyr/drivers/hwinfo.h>
#include <zephyr/logging/log.h>
LOG_MODULE_DECLARE(zmk, CONFIG_ZMK_LOG_LEVEL);

#include <zephyr/bluetooth/bluetooth.h>
#include <pb_encode.h>
#include <zmk/ble.h>
#include <zmk/studio/core.h>
#include <zmk/studio/rpc.h>

ZMK_RPC_SUBSYSTEM(core)

#define CORE_RESPONSE(type, ...) ZMK_RPC_RESPONSE(core, type, __VA_ARGS__)

static bool encode_device_info_name(pb_ostream_t *stream, const pb_field_t *field,
                                    void *const *arg) {
    if (!pb_encode_tag_for_field(stream, field)) {
        return false;
    }

    return pb_encode_string(stream, CONFIG_ZMK_KEYBOARD_NAME, strlen(CONFIG_ZMK_KEYBOARD_NAME));
}

#if IS_ENABLED(CONFIG_HWINFO)
static bool encode_device_info_serial_number(pb_ostream_t *stream, const pb_field_t *field,
                                             void *const *arg) {
    uint8_t id_buffer[32];
    const ssize_t id_size = hwinfo_get_device_id(id_buffer, ARRAY_SIZE(id_buffer));

    if (id_size <= 0) {
        return true;
    }

    if (!pb_encode_tag_for_field(stream, field)) {
        return false;
    }

    return pb_encode_string(stream, id_buffer, id_size);
}

#endif // IS_ENABLED(CONFIG_HWINFO)

#ifdef CONFIG_CONDUCTOR_FIRMWARE_VERSION
static bool encode_device_info_firmware_version(pb_ostream_t *stream, const pb_field_t *field,
                                                void *const *arg) {
    if (!pb_encode_tag_for_field(stream, field)) {
        return false;
    }

    return pb_encode_string(stream, CONFIG_CONDUCTOR_FIRMWARE_VERSION,
                            strlen(CONFIG_CONDUCTOR_FIRMWARE_VERSION));
}
#endif

zmk_studio_Response get_device_info(const zmk_studio_Request *req) {
    LOG_DBG("");
    zmk_core_GetDeviceInfoResponse resp = zmk_core_GetDeviceInfoResponse_init_zero;

    resp.name.funcs.encode = encode_device_info_name;
#if IS_ENABLED(CONFIG_HWINFO)
    resp.serial_number.funcs.encode = encode_device_info_serial_number;
#endif // IS_ENABLED(CONFIG_HWINFO)
#ifdef CONFIG_CONDUCTOR_FIRMWARE_VERSION
    resp.firmware_version.funcs.encode = encode_device_info_firmware_version;
#endif

    return CORE_RESPONSE(get_device_info, resp);
}

zmk_studio_Response get_lock_state(const zmk_studio_Request *req) {
    LOG_DBG("");
    zmk_core_LockState resp = zmk_studio_core_get_lock_state();

    return CORE_RESPONSE(get_lock_state, resp);
}

zmk_studio_Response reset_settings(const zmk_studio_Request *req) {
    LOG_DBG("");
    ZMK_RPC_SUBSYSTEM_SETTINGS_RESET_FOREACH(sub) {
        int ret = sub->callback();
        if (ret < 0) {
            LOG_ERR("Failed to reset settings: %d", ret);
            return CORE_RESPONSE(reset_settings, false);
        }
    }

    return CORE_RESPONSE(reset_settings, true);
}

#if IS_ENABLED(CONFIG_ZMK_BLE)
zmk_studio_Response get_ble_profiles(const zmk_studio_Request *req) {
    LOG_DBG("");
    zmk_core_GetBleProfilesResponse resp = zmk_core_GetBleProfilesResponse_init_zero;

    resp.active_index = zmk_ble_active_profile_index();
    resp.profiles_count = 0;

    for (int i = 0; i < ZMK_BLE_PROFILE_COUNT && i < 5; i++) {
        zmk_core_BleProfileInfo *p = &resp.profiles[i];
        bool is_open = zmk_ble_profile_is_open(i);

        if (!is_open) {
            char *name = zmk_ble_profile_name(i);
            if (name && name[0] != '\0') {
                strncpy(p->name, name, sizeof(p->name) - 1);
            } else {
                // No device name stored — use BLE address as fallback
                bt_addr_le_t *addr = zmk_ble_profile_address(i);
                bt_addr_le_to_str(addr, p->name, sizeof(p->name));
            }
            p->name[sizeof(p->name) - 1] = '\0';
        }
        p->connected = zmk_ble_profile_is_connected(i);
        resp.profiles_count++;
    }

    return CORE_RESPONSE(get_ble_profiles, resp);
}
#endif /* IS_ENABLED(CONFIG_ZMK_BLE) */

ZMK_RPC_SUBSYSTEM_HANDLER(core, get_device_info, ZMK_STUDIO_RPC_HANDLER_UNSECURED);
ZMK_RPC_SUBSYSTEM_HANDLER(core, get_lock_state, ZMK_STUDIO_RPC_HANDLER_UNSECURED);
ZMK_RPC_SUBSYSTEM_HANDLER(core, reset_settings, ZMK_STUDIO_RPC_HANDLER_SECURED);
#if IS_ENABLED(CONFIG_ZMK_BLE)
ZMK_RPC_SUBSYSTEM_HANDLER(core, get_ble_profiles, ZMK_STUDIO_RPC_HANDLER_UNSECURED);
#endif

static int core_event_mapper(const zmk_event_t *eh, zmk_studio_Notification *n) {
    struct zmk_studio_core_lock_state_changed *lock_ev = as_zmk_studio_core_lock_state_changed(eh);

    if (!lock_ev) {
        return -ENOTSUP;
    }

    LOG_DBG("Mapped a lock state event properly");

    *n = ZMK_RPC_NOTIFICATION(core, lock_state_changed, lock_ev->state);
    return 0;
}

ZMK_RPC_EVENT_MAPPER(core, core_event_mapper, zmk_studio_core_lock_state_changed);
