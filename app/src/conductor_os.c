/*
 * Copyright (c) 2026 The Conductor Contributors
 *
 * SPDX-License-Identifier: MIT
 */

#include <zephyr/init.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/settings/settings.h>

#include <zmk/conductor_os.h>
#include <zmk/endpoints.h>
#include <zmk/keymap.h>
#include <zmk/event_manager.h>
#include <zmk/events/endpoint_changed.h>

#if IS_ENABLED(CONFIG_ZMK_BLE)
#include <zmk/events/ble_active_profile_changed.h>
#endif

LOG_MODULE_DECLARE(zmk, CONFIG_ZMK_LOG_LEVEL);

#if IS_ENABLED(CONFIG_CONDUCTOR_OS_PROFILE)

/* Per-endpoint overlay layer ID (0 = no overlay = shared keymap). These are
 * stable zmk_keymap_layer_id values (passed straight to
 * zmk_keymap_layer_activate), NOT ordered layer indices. */
static bool pl_enabled = IS_ENABLED(CONFIG_CONDUCTOR_OS_PROFILE_DEFAULT_ON);
static uint8_t profile_layer[ZMK_ENDPOINT_COUNT];
/* The overlay layer (ID) we currently have activated (0 = none). */
static uint8_t active_overlay;

struct pl_settings_blob {
    uint8_t enabled;
    uint8_t profile_layer[ZMK_ENDPOINT_COUNT];
} __packed;

static int selected_endpoint_index(void) {
    int idx = zmk_endpoint_instance_to_index(zmk_endpoint_get_selected());
    if (idx < 0) {
        idx = 0;
    } else if (idx >= ZMK_ENDPOINT_COUNT) {
        idx = ZMK_ENDPOINT_COUNT - 1;
    }
    return idx;
}

static uint8_t sanitize_layer(uint8_t layer) {
    /* 0 = no overlay; the default layer is never used as an overlay. */
    if (layer == 0 || layer >= ZMK_KEYMAP_LAYERS_LEN || layer == zmk_keymap_layer_default()) {
        return 0;
    }
    return layer;
}

/* Swap the per-profile overlay to match the currently selected endpoint. Only
 * touches the dedicated overlay layer (activate/deactivate, non-locking) so the
 * shared base, momentary (&lt/&mo) and user-locked (&tog) layers are untouched.
 * Runs on the system workqueue (see apply_profile_layer) so all callers
 * (endpoint events, RPC set, boot) are serialized onto one thread. */
static void do_apply_profile_layer(struct k_work *work) {
    ARG_UNUSED(work);
    uint8_t want = pl_enabled ? sanitize_layer(profile_layer[selected_endpoint_index()]) : 0;
    uint8_t old = active_overlay;
    if (want == old) {
        return;
    }
    // Update the tracked overlay BEFORE touching layers, so any synchronous
    // layer_state_changed raised below computes the host-facing highest layer
    // against the new value (avoids a stale overlay flashing as highest).
    active_overlay = want;
    if (old != 0) {
        zmk_keymap_layer_deactivate(old, false);
    }
    if (want != 0) {
        zmk_keymap_layer_activate(want, false);
    }
    LOG_DBG("profile overlay %d -> %d (endpoint %d)", old, want, selected_endpoint_index());
}

static K_WORK_DELAYABLE_DEFINE(apply_work, do_apply_profile_layer);
/* Separate one-shot work for the boot fallback: a K_NO_WAIT reschedule of
 * apply_work by an early endpoint event would otherwise cancel a shared delayed
 * submission before settings_load() populates the map. */
static K_WORK_DELAYABLE_DEFINE(boot_apply_work, do_apply_profile_layer);

static void apply_profile_layer(void) { k_work_reschedule(&apply_work, K_NO_WAIT); }

static void persist_config(void) {
    struct pl_settings_blob blob = {.enabled = pl_enabled ? 1 : 0};
    memcpy(blob.profile_layer, profile_layer, sizeof(blob.profile_layer));
    int rc = settings_save_one("cond/pl", &blob, sizeof(blob));
    if (rc) {
        LOG_ERR("Failed to persist profile-layer config: %d", rc);
    }
}

bool conductor_profile_layers_enabled(void) { return pl_enabled; }

uint8_t conductor_profile_active_overlay(void) { return active_overlay; }

void conductor_profile_get_config(bool *enabled, uint8_t *map_out, size_t *len_inout,
                                  int *active_endpoint, uint8_t *active_layer) {
    if (enabled) {
        *enabled = pl_enabled;
    }
    if (map_out && len_inout) {
        size_t n = MIN(*len_inout, (size_t)ZMK_ENDPOINT_COUNT);
        memcpy(map_out, profile_layer, n);
        *len_inout = ZMK_ENDPOINT_COUNT;
    } else if (len_inout) {
        *len_inout = ZMK_ENDPOINT_COUNT;
    }
    if (active_endpoint) {
        *active_endpoint = selected_endpoint_index();
    }
    if (active_layer) {
        *active_layer = active_overlay;
    }
}

void conductor_profile_set_config(bool enabled, const uint8_t *map_in, size_t len) {
    pl_enabled = enabled;
    if (map_in) {
        size_t n = MIN(len, (size_t)ZMK_ENDPOINT_COUNT);
        for (size_t i = 0; i < n; i++) {
            profile_layer[i] = sanitize_layer(map_in[i]);
        }
    }
    apply_profile_layer();
    LOG_DBG("profile-layer config set: enabled=%d active_overlay=%d", pl_enabled, active_overlay);
    persist_config();
}

static int conductor_os_listener_cb(const zmk_event_t *eh) {
    if (as_zmk_endpoint_changed(eh)) {
        apply_profile_layer();
        return ZMK_EV_EVENT_BUBBLE;
    }
#if IS_ENABLED(CONFIG_ZMK_BLE)
    if (as_zmk_ble_active_profile_changed(eh)) {
        apply_profile_layer();
        return ZMK_EV_EVENT_BUBBLE;
    }
#endif
    return ZMK_EV_EVENT_BUBBLE;
}

ZMK_LISTENER(conductor_os, conductor_os_listener_cb);
ZMK_SUBSCRIPTION(conductor_os, zmk_endpoint_changed);
#if IS_ENABLED(CONFIG_ZMK_BLE)
ZMK_SUBSCRIPTION(conductor_os, zmk_ble_active_profile_changed);
#endif

static int conductor_os_settings_load_cb(const char *name, size_t len, settings_read_cb read_cb,
                                         void *cb_arg) {
    const char *next;
    if (settings_name_steq(name, "pl", &next) && !next) {
        struct pl_settings_blob blob;
        if (len != sizeof(blob)) {
            return -EINVAL;
        }
        int rc = read_cb(cb_arg, &blob, sizeof(blob));
        if (rc < 0) {
            return rc;
        }
        pl_enabled = blob.enabled ? true : false;
        for (size_t i = 0; i < ZMK_ENDPOINT_COUNT; i++) {
            profile_layer[i] = sanitize_layer(blob.profile_layer[i]);
        }
        LOG_INF("Loaded profile-layer config: enabled=%d", pl_enabled);
        return 0;
    }
    return -ENOENT;
}

SETTINGS_STATIC_HANDLER_DEFINE(conductor_os_settings, "cond", NULL, conductor_os_settings_load_cb,
                               NULL, NULL);

/* Boot-time fallback: apply the persisted overlay once after the endpoints have
 * settled, in case no zmk_endpoint_changed fires after settings load (e.g. the
 * selected output is already determined before our listener runs). An endpoint
 * event before this just reschedules the same work (idempotent). */
static int conductor_os_init(void) {
    k_work_schedule(&boot_apply_work, K_MSEC(1500));
    return 0;
}

SYS_INIT(conductor_os_init, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);

#else /* !IS_ENABLED(CONFIG_CONDUCTOR_OS_PROFILE) */

bool conductor_profile_layers_enabled(void) { return false; }

uint8_t conductor_profile_active_overlay(void) { return 0; }

void conductor_profile_get_config(bool *enabled, uint8_t *map_out, size_t *len_inout,
                                  int *active_endpoint, uint8_t *active_layer) {
    if (enabled) {
        *enabled = false;
    }
    if (len_inout) {
        *len_inout = 0;
    }
    if (active_endpoint) {
        *active_endpoint = 0;
    }
    if (active_layer) {
        *active_layer = 0;
    }
}

void conductor_profile_set_config(bool enabled, const uint8_t *map_in, size_t len) {
    ARG_UNUSED(enabled);
    ARG_UNUSED(map_in);
    ARG_UNUSED(len);
}

#endif /* IS_ENABLED(CONFIG_CONDUCTOR_OS_PROFILE) */
