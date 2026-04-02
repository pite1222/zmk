/*
 * Layer Notify — Send F13+layer_index key tap on layer change
 *
 * When a layer is activated, sends a brief F13-F20 keypress to notify
 * the host (macOS Conductor Overlay) of the active layer.
 *
 * F13 = layer 0, F14 = layer 1, ..., F20 = layer 7
 *
 * Copyright (c) 2026 Conductor Keyboard
 * SPDX-License-Identifier: MIT
 */

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
LOG_MODULE_DECLARE(zmk, CONFIG_ZMK_LOG_LEVEL);

#include <zmk/event_manager.h>
#include <zmk/events/layer_state_changed.h>
#include <zmk/hid.h>
#include <zmk/endpoints.h>
#include <dt-bindings/zmk/hid_usage_pages.h>
#include <dt-bindings/zmk/keys.h>

// F13=0x68, F14=0x69, ..., F20=0x6F in HID usage table
// F13-F20 are rarely used and detectable on all platforms
#define LAYER_NOTIFY_BASE_KEY 0x68  // F13

static void send_layer_key(uint8_t layer, bool pressed) {
    if (layer > 7) return;  // Only support layers 0-7

    zmk_key_t key = LAYER_NOTIFY_BASE_KEY + layer;
    if (pressed) {
        zmk_hid_keyboard_press(key);
    } else {
        zmk_hid_keyboard_release(key);
    }
    zmk_endpoint_send_report(HID_USAGE_KEY);
}

static struct k_work_delayable layer_release_work;
static uint8_t pending_release_layer = 0;

static void layer_release_handler(struct k_work *work) {
    send_layer_key(pending_release_layer, false);
}

static int layer_notify_listener(const zmk_event_t *eh) {
    struct zmk_layer_state_changed *ev = as_zmk_layer_state_changed(eh);
    if (ev == NULL) {
        return ZMK_EV_EVENT_BUBBLE;
    }

    if (ev->state) {
        // Layer activated — send F13+layer press, then release after 10ms
        LOG_DBG("Layer %d activated, sending F%d", ev->layer, 13 + ev->layer);
        send_layer_key(ev->layer, true);
        pending_release_layer = ev->layer;
        k_work_schedule(&layer_release_work, K_MSEC(10));
    }

    return ZMK_EV_EVENT_BUBBLE;
}

ZMK_LISTENER(layer_notify, layer_notify_listener);
ZMK_SUBSCRIPTION(layer_notify, zmk_layer_state_changed);

static int layer_notify_init(void) {
    k_work_init_delayable(&layer_release_work, layer_release_handler);
    LOG_INF("Layer notify initialized (F13-F20)");
    return 0;
}

SYS_INIT(layer_notify_init, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);
