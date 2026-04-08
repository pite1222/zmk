/*
 * Copyright (c) 2024 The ZMK Contributors
 *
 * SPDX-License-Identifier: MIT
 */

/*
 * BLE Pointing Keepalive
 *
 * Sends periodic zero-motion HID mouse reports while idle to prevent the host
 * OS (especially macOS) from renegotiating the BLE connection interval to a
 * slower value.  Without this, the first trackball movement after ~3 s of
 * inactivity is noticeably delayed because the host has widened the interval.
 */

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

LOG_MODULE_DECLARE(zmk, CONFIG_ZMK_LOG_LEVEL);

#include <zmk/hid.h>
#include <zmk/endpoints.h>
#include <zmk/pointing/keepalive.h>

#define KEEPALIVE_INTERVAL_MS CONFIG_ZMK_POINTING_BLE_KEEPALIVE_INTERVAL_MS
#define KEEPALIVE_MAX_IDLE_MS CONFIG_ZMK_POINTING_BLE_KEEPALIVE_MAX_IDLE_MS

static int64_t last_activity_time;
static struct k_work_delayable keepalive_work;

static void keepalive_handler(struct k_work *work) {
    int64_t idle_ms = k_uptime_get() - last_activity_time;

    if (idle_ms >= KEEPALIVE_MAX_IDLE_MS) {
        /* Idle for too long — stop keepalive to save battery. */
        return;
    }

    /* Send a zero-motion report to keep the BLE link at a fast interval. */
    zmk_hid_mouse_movement_set(0, 0);
    zmk_hid_mouse_scroll_set(0, 0);
    zmk_endpoint_send_mouse_report();

    k_work_reschedule(&keepalive_work, K_MSEC(KEEPALIVE_INTERVAL_MS));
}

void zmk_pointing_keepalive_notify_activity(void) {
    last_activity_time = k_uptime_get();
    /* Defer next keepalive — real traffic is flowing. */
    k_work_reschedule(&keepalive_work, K_MSEC(KEEPALIVE_INTERVAL_MS));
}

static int pointing_keepalive_init(void) {
    last_activity_time = k_uptime_get();
    k_work_init_delayable(&keepalive_work, keepalive_handler);
    k_work_reschedule(&keepalive_work, K_MSEC(KEEPALIVE_INTERVAL_MS));
    return 0;
}

SYS_INIT(pointing_keepalive_init, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);
