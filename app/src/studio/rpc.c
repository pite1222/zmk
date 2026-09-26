/*
 * Copyright (c) 2024 The ZMK Contributors
 *
 * SPDX-License-Identifier: MIT
 */

#include "msg_framing.h"

#include <pb_encode.h>
#include <pb_decode.h>

#include <zephyr/init.h>
#include <zephyr/kernel.h>
#include <zephyr/debug/thread_analyzer.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(zmk_studio, CONFIG_ZMK_STUDIO_LOG_LEVEL);

#include <zmk/endpoints.h>
#include <zmk/event_manager.h>
#include <zmk/events/endpoint_changed.h>
#include <zmk/studio/core.h>
#include <zmk/studio/rpc.h>

ZMK_EVENT_IMPL(zmk_studio_rpc_notification);

static struct zmk_rpc_subsystem *find_subsystem_for_choice(uint8_t choice) {
    STRUCT_SECTION_FOREACH(zmk_rpc_subsystem, sub) {
        if (sub->subsystem_choice == choice) {
            return sub;
        }
    }

    return NULL;
}

zmk_studio_Response zmk_rpc_subsystem_delegate_to_subs(const struct zmk_rpc_subsystem *subsys,
                                                       const zmk_studio_Request *req,
                                                       uint8_t which_req) {
    LOG_DBG("Got subsystem func for %d", subsys->subsystem_choice);

    for (int i = subsys->handlers_start_index; i <= subsys->handlers_end_index; i++) {
        struct zmk_rpc_subsystem_handler *sub_handler;
        STRUCT_SECTION_GET(zmk_rpc_subsystem_handler, i, &sub_handler);
        if (sub_handler->request_choice == which_req) {
            if (sub_handler->security == ZMK_STUDIO_RPC_HANDLER_SECURED &&
                zmk_studio_core_get_lock_state() != ZMK_STUDIO_CORE_LOCK_STATE_UNLOCKED) {
                return ZMK_RPC_RESPONSE(meta, simple_error,
                                        zmk_meta_ErrorConditions_UNLOCK_REQUIRED);
            }
            return sub_handler->func(req);
        }
    }
    LOG_ERR("No handler func found for %d", which_req);
    return ZMK_RPC_RESPONSE(meta, simple_error, zmk_meta_ErrorConditions_RPC_NOT_FOUND);
}

static zmk_studio_Response handle_request(const zmk_studio_Request *req) {
    zmk_studio_core_reschedule_lock_timeout();
    struct zmk_rpc_subsystem *sub = find_subsystem_for_choice(req->which_subsystem);
    if (!sub) {
        LOG_WRN("No subsystem found for choice %d", req->which_subsystem);
        return ZMK_RPC_RESPONSE(meta, simple_error, zmk_meta_ErrorConditions_RPC_NOT_FOUND);
    }

    zmk_studio_Response resp = sub->func(sub, req);
    resp.type.request_response.request_id = req->request_id;

    return resp;
}

RING_BUF_DECLARE(rpc_rx_buf, CONFIG_ZMK_STUDIO_RPC_RX_BUF_SIZE);

static K_SEM_DEFINE(rpc_rx_sem, 0, 1);

static enum studio_framing_state rpc_framing_state;

static K_MUTEX_DEFINE(rpc_transport_mutex);
static struct zmk_rpc_transport *selected_transport;

/* The transport that most recently received RPC data. Responses and
 * notifications are sent back on this transport, decoupling Studio
 * connectivity from the active HID output endpoint so the host can connect
 * over USB or BLE regardless of which endpoint is currently selected for HID
 * output. Written from RX context (incl. ISR), so kept lock-free via atomic. */
static atomic_t active_rx_transport = ATOMIC_INIT(0);

void zmk_rpc_set_rx_transport(enum zmk_transport transport) {
    atomic_set(&active_rx_transport, (atomic_val_t)transport);
}

struct rpc_tx_ctx {
    struct zmk_rpc_transport *transport;
    void *user_data;
};

static struct zmk_rpc_transport *resolve_tx_transport(enum zmk_transport target) {
    STRUCT_SECTION_FOREACH(zmk_rpc_transport, t) {
        if (t->transport == target) {
            return t;
        }
    }
    return selected_transport;
}

struct ring_buf *zmk_rpc_get_rx_buf(void) { return &rpc_rx_buf; }

void zmk_rpc_rx_notify(void) { k_sem_give(&rpc_rx_sem); }

static bool rpc_read_cb(pb_istream_t *stream, uint8_t *buf, size_t count) {
    uint32_t write_offset = 0;

    do {
        uint8_t *buffer;
        uint32_t len = ring_buf_get_claim(&rpc_rx_buf, &buffer, count - write_offset);

        if (len > 0) {
            for (int i = 0; i < len; i++) {
                if (studio_framing_process_byte(&rpc_framing_state, buffer[i])) {
                    buf[write_offset++] = buffer[i];
                }
            }
        } else {
            k_sem_take(&rpc_rx_sem, K_FOREVER);
        }

        ring_buf_get_finish(&rpc_rx_buf, len);
    } while (write_offset < count && rpc_framing_state != FRAMING_STATE_EOF);

    if (rpc_framing_state == FRAMING_STATE_EOF) {
        stream->bytes_left = 0;
        return false;
    } else {
        return true;
    }
}

static pb_istream_t pb_istream_for_rx_ring_buf() {
    pb_istream_t stream = {&rpc_read_cb, NULL, SIZE_MAX};
    return stream;
}

RING_BUF_DECLARE(rpc_tx_buf, CONFIG_ZMK_STUDIO_RPC_TX_BUF_SIZE);

struct ring_buf *zmk_rpc_get_tx_buf(void) { return &rpc_tx_buf; }

static bool rpc_tx_buffer_write(pb_ostream_t *stream, const uint8_t *buf, size_t count) {
    struct rpc_tx_ctx *ctx = (struct rpc_tx_ctx *)stream->state;
    size_t written = 0;

    bool escape_byte_already_written = false;
    do {
        uint32_t write_idx = 0;

        uint8_t *write_buf;
        uint32_t claim_len = ring_buf_put_claim(&rpc_tx_buf, &write_buf, count - written);

        if (claim_len == 0) {
            // Buffer full: kick the transport so it drains, then wait for room
            // (k_yield() would spin at the lowest priority and starve idle).
            ctx->transport->tx_notify(&rpc_tx_buf, 0, false, ctx->user_data);
            k_sleep(K_MSEC(1));
            continue;
        }

        int escapes_written = 0;
        for (int i = 0; i < claim_len && write_idx < claim_len; i++) {
            uint8_t b = buf[written + i];
            switch (b) {
            case FRAMING_EOF:
            case FRAMING_ESC:
            case FRAMING_SOF:
                // Care to be taken. We may need to write the escape byte,
                // but that's the last available spot for this claim, so we track
                // if the escape has already been written in the previous iteration
                // of our loop.
                if (!escape_byte_already_written) {
                    escapes_written++;
                    write_buf[write_idx++] = FRAMING_ESC;
                    escape_byte_already_written = true;
                    if (write_idx >= claim_len) {
                        LOG_WRN("Skipping on, no room to write escape and real byte");
                        continue;
                    }
                }
            default:
                write_buf[write_idx++] = b;
                escape_byte_already_written = false;
                break;
            }
        }

        ring_buf_put_finish(&rpc_tx_buf, write_idx);

        written += (write_idx - escapes_written);

        ctx->transport->tx_notify(&rpc_tx_buf, write_idx, false, ctx->user_data);
    } while (written < count);

    return true;
}

static void rpc_tx_put_framing_byte(struct rpc_tx_ctx *ctx, uint8_t b, bool msg_done) {
    while (ring_buf_put(&rpc_tx_buf, &b, 1) == 0) {
        ctx->transport->tx_notify(&rpc_tx_buf, 0, false, ctx->user_data);
        k_sleep(K_MSEC(1));
    }

    ctx->transport->tx_notify(&rpc_tx_buf, 1, msg_done, ctx->user_data);
}

static pb_ostream_t pb_ostream_for_tx_buf(struct rpc_tx_ctx *ctx) {
    pb_ostream_t stream = {&rpc_tx_buffer_write, (void *)ctx, SIZE_MAX, 0};
    return stream;
}

static int send_response(const zmk_studio_Response *resp, enum zmk_transport target) {
    int ret = 0;
    k_mutex_lock(&rpc_transport_mutex, K_FOREVER);

    struct zmk_rpc_transport *tx = resolve_tx_transport(target);
    if (!tx) {
        goto exit;
    }

    struct rpc_tx_ctx ctx = {
        .transport = tx,
        .user_data = tx->tx_user_data ? tx->tx_user_data() : NULL,
    };

    pb_ostream_t stream = pb_ostream_for_tx_buf(&ctx);

    rpc_tx_put_framing_byte(&ctx, FRAMING_SOF, false);

    /* Now we are ready to encode the message! */
    bool status = pb_encode(&stream, &zmk_studio_Response_msg, resp);

    if (!status) {
#if !IS_ENABLED(CONFIG_NANOPB_NO_ERRMSG)
        LOG_ERR("Failed to encode the message %s", stream.errmsg);
#endif // !IS_ENABLED(CONFIG_NANOPB_NO_ERRMSG)
        ret = -EINVAL;
        goto exit;
    }

    rpc_tx_put_framing_byte(&ctx, FRAMING_EOF, true);

exit:
    k_mutex_unlock(&rpc_transport_mutex);
    return ret;
}

static void rpc_main(void) {
    for (;;) {
        pb_istream_t stream = pb_istream_for_rx_ring_buf();
        zmk_studio_Request req = zmk_studio_Request_init_zero;
#if IS_ENABLED(CONFIG_THREAD_ANALYZER)
        thread_analyzer_print(0);
#endif // IS_ENABLED(CONFIG_THREAD_ANALYZER)
        bool status = pb_decode(&stream, &zmk_studio_Request_msg, &req);

        rpc_framing_state = FRAMING_STATE_IDLE;

        if (status) {
            /* Bind the response to the transport that delivered this request,
             * captured before handling so a byte arriving on the other transport
             * mid-handler cannot redirect it. */
            enum zmk_transport req_transport =
                (enum zmk_transport)atomic_get(&active_rx_transport);

            zmk_studio_Response resp = handle_request(&req);

            int err = send_response(&resp, req_transport);
#if IS_ENABLED(CONFIG_THREAD_ANALYZER)
            thread_analyzer_print(0);
#endif // IS_ENABLED(CONFIG_THREAD_ANALYZER)
            if (err < 0) {
                LOG_ERR("Failed to send the RPC response %d", err);
            }
        } else {
            LOG_DBG("Decode failed");
        }
    }
}

K_THREAD_DEFINE(studio_rpc_thread, CONFIG_ZMK_STUDIO_RPC_THREAD_STACK_SIZE, rpc_main, NULL, NULL,
                NULL, K_LOWEST_APPLICATION_THREAD_PRIO, 0, 0);

/* Start RX on every registered transport once, so Studio RPC is reachable over
 * USB and BLE regardless of the active HID output endpoint. RX is never stopped:
 * only the connected transport actually delivers data, and TX is routed back to
 * whichever transport received the request (see resolve_tx_transport).
 *
 * The RX and TX ring buffers are shared across transports; this assumes a single
 * active Studio client at a time (the normal case). rpc_main serializes
 * responses, so back-to-back requests from one transport drain in FIFO order. */
static void start_all_transports(void) {
    STRUCT_SECTION_FOREACH(zmk_rpc_transport, t) {
        if (t->rx_start) {
            t->rx_start();
        }
    }
}

/* Track the active HID output endpoint's transport only as the default TX
 * target for notifications raised before any request has been received. */
static void refresh_selected_transport(void) {
    enum zmk_transport transport = zmk_endpoint_get_selected().transport;

    k_mutex_lock(&rpc_transport_mutex, K_FOREVER);

#if IS_ENABLED(CONFIG_ZMK_STUDIO_LOCK_ON_DISCONNECT)
    bool transport_changed = selected_transport && selected_transport->transport != transport;
#endif

    STRUCT_SECTION_FOREACH(zmk_rpc_transport, t) {
        if (t->transport == transport) {
            selected_transport = t;
            break;
        }
    }

#if IS_ENABLED(CONFIG_ZMK_STUDIO_LOCK_ON_DISCONNECT)
    /* When the active HID output endpoint moves to a different transport, the
     * previously connected Studio session is no longer reachable on it (e.g. USB
     * unplug falling back to BLE). Lock so the device isn't left unlocked for a
     * different transport's host. */
    if (transport_changed) {
        zmk_studio_core_lock();
    }
#endif

    k_mutex_unlock(&rpc_transport_mutex);
}

static int zmk_rpc_init(void) {
    int prev_choice = -1;
    struct zmk_rpc_subsystem *prev_sub = NULL;
    int i = 0;

    STRUCT_SECTION_FOREACH(zmk_rpc_subsystem_handler, handler) {
        struct zmk_rpc_subsystem *sub = find_subsystem_for_choice(handler->subsystem_choice);

        __ASSERT(sub != NULL, "RPC Handler for unknown subsystem choice %d",
                 handler->subsystem_choice);

        if (prev_choice < 0) {
            sub->handlers_start_index = i;
        } else if ((prev_choice != handler->subsystem_choice) && prev_sub) {
            prev_sub->handlers_end_index = i - 1;
            sub->handlers_start_index = i;
        }

        prev_choice = handler->subsystem_choice;
        prev_sub = sub;
        i++;
    }

    if (prev_sub) {
        prev_sub->handlers_end_index = i - 1;
    }

    start_all_transports();
    refresh_selected_transport();

    return 0;
}

SYS_INIT(zmk_rpc_init, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);

static int studio_rpc_listener_cb(const zmk_event_t *eh) {
    struct zmk_endpoint_changed *ep_changed = as_zmk_endpoint_changed(eh);
    if (ep_changed) {
        refresh_selected_transport();
        return ZMK_EV_EVENT_BUBBLE;
    }

    struct zmk_studio_rpc_notification *rpc_notify = as_zmk_studio_rpc_notification(eh);
    if (rpc_notify) {
        zmk_studio_Response resp = zmk_studio_Response_init_zero;
        resp.which_type = zmk_studio_Response_notification_tag;
        resp.type.notification = rpc_notify->notification;
        send_response(&resp, (enum zmk_transport)atomic_get(&active_rx_transport));
        return ZMK_EV_EVENT_BUBBLE;
    }

    zmk_studio_Notification n = zmk_studio_Notification_init_zero;
    STRUCT_SECTION_FOREACH(zmk_rpc_event_mapper, mapper) {
        int ret = mapper->func(eh, &n);
        if (ret >= 0) {
            raise_zmk_studio_rpc_notification(
                (struct zmk_studio_rpc_notification){.notification = n});
            break;
        }
    }

    return ZMK_EV_EVENT_BUBBLE;
}

ZMK_LISTENER(studio_rpc, studio_rpc_listener_cb);
ZMK_SUBSCRIPTION(studio_rpc, zmk_endpoint_changed);
ZMK_SUBSCRIPTION(studio_rpc, zmk_studio_rpc_notification);
