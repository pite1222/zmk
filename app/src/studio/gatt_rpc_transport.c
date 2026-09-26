/*
 * Copyright (c) 2024 The ZMK Contributors
 *
 * SPDX-License-Identifier: MIT
 */

#include <zephyr/device.h>
#include <zephyr/init.h>
#include <sys/types.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/kernel.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/sys/ring_buffer.h>

#include <zmk/ble.h>
#include <zmk/event_manager.h>
#include <zmk/events/ble_active_profile_changed.h>
#include <zmk/studio/rpc.h>

#include "uuid.h"

#include <zephyr/logging/log.h>

LOG_MODULE_DECLARE(zmk_studio, CONFIG_ZMK_STUDIO_LOG_LEVEL);

static bool handling_rx = false;

static K_SEM_DEFINE(indicate_sem, 1, 1);
static atomic_t notify_size;

/* Profile index of the host that most recently wrote an RPC request. Studio
 * responses are indicated back to this host's connection rather than the
 * active HID profile's, so Studio works over BLE regardless of which profile
 * is currently selected for output. -1 falls back to the active profile. */
static atomic_t rpc_resp_profile = ATOMIC_INIT(-1);

/* Profile index of the host with an open Studio session (-1 if none). ble.c
 * keeps this host at the preferred low-latency conn params even when it is not
 * the active HID profile: every response chunk costs an indication/confirmation
 * round trip, so a relaxed non-active link makes large responses time out. */
static atomic_t studio_profile = ATOMIC_INIT(-1);

int zmk_studio_ble_profile(void) { return (int)atomic_get(&studio_profile); }

static void studio_session_begin(struct bt_conn *conn) {
    int profile = zmk_ble_profile_index(bt_conn_get_dst(conn));
    if (profile < 0) {
        return;
    }

    if ((int)atomic_set(&studio_profile, profile) != profile) {
        LOG_DBG("Studio session started on profile %d", profile);
        zmk_ble_request_conn_param_rebalance();
    }
}

static void studio_session_end(int profile) {
    if (profile >= 0 && atomic_cas(&studio_profile, profile, -1)) {
        LOG_DBG("Studio session ended on profile %d", profile);
        zmk_ble_request_conn_param_rebalance();
    }
}

/* Returns a ref'd connection (caller unrefs) for the host that should receive
 * RPC responses, mirroring zmk_ble_active_profile_conn() semantics. Once a host
 * has sent a request, responses are pinned to that host's profile: if it is no
 * longer connected we return NULL (drop) rather than falling back to a
 * different profile, which would misroute the response to the wrong host. */
static struct bt_conn *gatt_resp_conn(void) {
    int idx = (int)atomic_get(&rpc_resp_profile);
    if (idx < 0) {
        return zmk_ble_active_profile_conn();
    }

    bt_addr_le_t *addr = zmk_ble_profile_address((uint8_t)idx);
    if (!addr || !bt_addr_le_cmp(addr, BT_ADDR_LE_ANY)) {
        return NULL;
    }

    return bt_conn_lookup_addr_le(BT_ID_DEFAULT, addr);
}

static uint16_t get_notify_size_for_conn(struct bt_conn *conn);

static void rpc_ccc_cfg_changed(const struct bt_gatt_attr *attr, uint16_t value) {
    ARG_UNUSED(attr);

    bool notif_enabled = (value == BT_GATT_CCC_INDICATE);

    LOG_INF("RPC Notifications %s", notif_enabled ? "enabled" : "disabled");

    if (!notif_enabled) {
        studio_session_end(zmk_studio_ble_profile());
    }
}

static ssize_t rpc_ccc_cfg_write(struct bt_conn *conn, const struct bt_gatt_attr *attr,
                                 uint16_t value) {
    ARG_UNUSED(attr);

    if (value == BT_GATT_CCC_INDICATE) {
        studio_session_begin(conn);
    } else {
        studio_session_end(zmk_ble_profile_index(bt_conn_get_dst(conn)));
    }

    return sizeof(value);
}

static ssize_t read_rpc_resp(struct bt_conn *conn, const struct bt_gatt_attr *attr, void *buf,
                             uint16_t len, uint16_t offset) {

    LOG_DBG("Read response for length %d at offset %d", len, offset);
    return 0;
}

static ssize_t write_rpc_req(struct bt_conn *conn, const struct bt_gatt_attr *attr, const void *buf,
                             uint16_t len, uint16_t offset, uint8_t flags) {
    if (!handling_rx) {
        return len;
    }

    int profile = zmk_ble_profile_index(bt_conn_get_dst(conn));
    if (profile >= 0) {
        atomic_set(&rpc_resp_profile, profile);
    }
    studio_session_begin(conn);
    atomic_set(&notify_size, get_notify_size_for_conn(conn));

    uint32_t copied = 0;
    struct ring_buf *rpc_buf = zmk_rpc_get_rx_buf();
    while (copied < len) {
        uint8_t *buffer;
        uint32_t claim_len = ring_buf_put_claim(rpc_buf, &buffer, len - copied);

        if (claim_len > 0) {
            memcpy(buffer, ((uint8_t *)buf) + copied, claim_len);
            copied += claim_len;
        }

        ring_buf_put_finish(rpc_buf, claim_len);
    }

    zmk_rpc_set_rx_transport(ZMK_TRANSPORT_BLE);
    zmk_rpc_rx_notify();

    return len;
}

BT_GATT_SERVICE_DEFINE(
    rpc_interface, BT_GATT_PRIMARY_SERVICE(BT_UUID_DECLARE_128(ZMK_STUDIO_BT_SERVICE_UUID)),
    BT_GATT_CHARACTERISTIC(BT_UUID_DECLARE_128(ZMK_STUDIO_BT_RPC_CHRC_UUID),
                           BT_GATT_CHRC_WRITE | BT_GATT_CHRC_READ | BT_GATT_CHRC_INDICATE,
                           BT_GATT_PERM_READ_ENCRYPT | BT_GATT_PERM_WRITE_ENCRYPT, read_rpc_resp,
                           write_rpc_req, NULL),
    BT_GATT_CCC_MANAGED(((struct _bt_gatt_ccc[]){BT_GATT_CCC_INITIALIZER(
                            rpc_ccc_cfg_changed, rpc_ccc_cfg_write, NULL)}),
                        BT_GATT_PERM_READ_ENCRYPT | BT_GATT_PERM_WRITE_ENCRYPT));

/* Largest indication payload at an ATT MTU of 247 (the usual LE maximum). */
static uint8_t indicate_buffer[244];

// Indication payload is bounded by the negotiated ATT MTU minus the opcode and
// handle; tx_max_len is the LL data length, not the ATT MTU.
static uint16_t get_notify_size_for_conn(struct bt_conn *conn) {
    uint16_t mtu = conn ? bt_gatt_get_mtu(conn) : 0;

    return MIN(MAX(mtu, 23) - 3, sizeof(indicate_buffer));
}

static void refresh_notify_size(void) {
    struct bt_conn *conn = gatt_resp_conn();

    uint16_t ns = get_notify_size_for_conn(conn);
    if (conn) {
        bt_conn_unref(conn);
    }

    atomic_set(&notify_size, ns);
}

static int gatt_start_rx() {
    refresh_notify_size();
    handling_rx = true;
    return 0;
}

static int gatt_stop_rx(void) {
    handling_rx = false;
    return 0;
}

/* Bytes staged in indicate_buffer that still have to be indicated. They are
 * already consumed from the TX ring buffer, so a failed indicate retries them. */
static uint16_t indicate_len;
static int64_t indicate_fail_since;

/* bt_gatt_indicate() fails transiently (e.g. -ENOMEM when the ATT TX pool is
 * shared with busy HID reports); retry the staged chunk instead of stalling. */
#define INDICATE_RETRY_DELAY K_MSEC(10)
#define INDICATE_RETRY_TIMEOUT_MS 2000

static void indicate_cb(struct bt_conn *conn, struct bt_gatt_indicate_params *params, uint8_t err);

static struct bt_gatt_indicate_params rpc_indicate_params = {
    .attr = &rpc_interface.attrs[1],
    .data = indicate_buffer,
    .func = indicate_cb,
};

static void drop_tx_data(struct ring_buf *tx_buf) {
    indicate_len = 0;
    indicate_fail_since = 0;
    // Consumer-side discard: ring_buf_reset() would race the RPC thread producer.
    ring_buf_get(tx_buf, NULL, ring_buf_size_get(tx_buf));
}

static void notif_rpc_tx_cb(struct k_work *work);
static K_WORK_DELAYABLE_DEFINE(notify_tx_work, notif_rpc_tx_cb);

static void notif_rpc_tx_cb(struct k_work *work) {
    struct bt_conn *conn = gatt_resp_conn();
    struct ring_buf *tx_buf = zmk_rpc_get_tx_buf();

    if (!conn) {
        LOG_WRN("No active connection for queued data, dropping");
        drop_tx_data(tx_buf);
        return;
    }

    if (k_sem_take(&indicate_sem, K_NO_WAIT) < 0) {
        // An indication is in flight; indicate_cb reschedules on completion.
        bt_conn_unref(conn);
        return;
    }

    if (indicate_len == 0) {
        uint16_t notify_size = get_notify_size_for_conn(conn);

        while (indicate_len < notify_size && ring_buf_size_get(tx_buf) > 0) {
            indicate_len +=
                ring_buf_get(tx_buf, indicate_buffer + indicate_len, notify_size - indicate_len);
        }
    }

    if (indicate_len == 0) {
        k_sem_give(&indicate_sem);
        bt_conn_unref(conn);
        return;
    }

    rpc_indicate_params.len = indicate_len;

    int err = bt_gatt_indicate(conn, &rpc_indicate_params);
    if (err == 0) {
        indicate_len = 0;
        indicate_fail_since = 0;
    } else {
        k_sem_give(&indicate_sem);

        int64_t now = k_uptime_get();
        if (indicate_fail_since == 0) {
            indicate_fail_since = now;
        }

        if (err != -ENOTCONN && now - indicate_fail_since < INDICATE_RETRY_TIMEOUT_MS) {
            LOG_DBG("Failed to indicate the response (%d), retrying", err);
            k_work_schedule(&notify_tx_work, INDICATE_RETRY_DELAY);
        } else {
            LOG_WRN("Failed to indicate the response (%d), dropping", err);
            drop_tx_data(tx_buf);
        }
    }

    bt_conn_unref(conn);
}

struct gatt_write_state {
    size_t pending_notify;
};

static void indicate_cb(struct bt_conn *conn, struct bt_gatt_indicate_params *params, uint8_t err) {
    if (err) {
        // The link is going away or ATT timed out; the next attempt drops the
        // queued data once the connection is gone or indicate keeps failing.
        LOG_WRN("RPC indication not confirmed (%u)", err);
    }

    k_sem_give(&indicate_sem);
    k_work_schedule(&notify_tx_work, K_NO_WAIT);
}

static void gatt_tx_notify(struct ring_buf *tx_buf, size_t added, bool msg_done, void *user_data) {
    struct gatt_write_state *state = (struct gatt_write_state *)user_data;

    state->pending_notify += added;

    atomic_t ns = atomic_get(&notify_size);

    // Also kick when the ring buffer is full: the RPC thread waits for space
    // without adding data, so the byte-count threshold alone could stall.
    if (msg_done || state->pending_notify > ns || ring_buf_space_get(tx_buf) == 0) {
        k_work_schedule(&notify_tx_work, K_NO_WAIT);
        state->pending_notify = 0;
    }
}

static struct gatt_write_state tx_state = {};

static void *gatt_tx_user_data(void) {
    memset(&tx_state, 0, sizeof(tx_state));

    return &tx_state;
}

ZMK_RPC_TRANSPORT(gatt, ZMK_TRANSPORT_BLE, gatt_start_rx, gatt_stop_rx, gatt_tx_user_data,
                  gatt_tx_notify);

static int gatt_rpc_listener(const zmk_event_t *eh) {
    refresh_notify_size();

#if IS_ENABLED(CONFIG_ZMK_STUDIO_LOCK_ON_DISCONNECT)
    struct bt_conn *conn = zmk_ble_active_profile_conn();

    if (!conn) {
        zmk_studio_core_lock();
    } else {
        bt_conn_unref(conn);
    }
#endif

    return 0;
}

ZMK_LISTENER(gatt_rpc_listener, gatt_rpc_listener);
ZMK_SUBSCRIPTION(gatt_rpc_listener, zmk_ble_active_profile_changed);

static void gatt_rpc_disconnected(struct bt_conn *conn, uint8_t reason) {
    studio_session_end(zmk_ble_profile_index(bt_conn_get_dst(conn)));
}

BT_CONN_CB_DEFINE(gatt_rpc_conn_callbacks) = {
    .disconnected = gatt_rpc_disconnected,
};
