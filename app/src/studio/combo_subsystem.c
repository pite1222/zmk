/*
 * Copyright (c) 2025 The Conductor Contributors
 *
 * SPDX-License-Identifier: MIT
 */

#include <zephyr/logging/log.h>
LOG_MODULE_DECLARE(zmk_studio, CONFIG_ZMK_STUDIO_LOG_LEVEL);

#include <zephyr/settings/settings.h>

#include <drivers/behavior.h>
#include <zmk/behavior.h>
#include <zmk/studio/rpc.h>
#include <zmk/combos.h>
#include <pb_encode.h>

ZMK_RPC_SUBSYSTEM(combo)

#define COMBO_RESPONSE(type, ...) ZMK_RPC_RESPONSE(combo, type, __VA_ARGS__)

/*
 * Persistent combo storage.
 * Each entry stores the combo config in a format suitable for flash storage.
 * behavior_local_id is used instead of behavior_dev pointer.
 */
struct combo_storage_entry {
    int32_t key_positions[ZMK_COMBO_MAX_KEYS];
    int16_t key_position_len;
    int16_t require_prior_idle_ms;
    int32_t timeout_ms;
    uint32_t layer_mask;
    uint16_t behavior_local_id;
    uint32_t param1;
    uint32_t param2;
    bool slow_release;
    bool active;
};

#define MAX_STORED_COMBOS CONFIG_ZMK_COMBO_MAX_DYNAMIC_COMBOS

static struct combo_storage_entry stored_combos[MAX_STORED_COMBOS];
static int stored_combo_count = 0;
static bool combos_loaded_from_settings = false;

/* Forward declaration */
static int combo_settings_set(const char *name, size_t len,
                               settings_read_cb read_cb, void *cb_arg);

SETTINGS_STATIC_HANDLER_DEFINE(zmk_combo_studio, "combo/studio",
                                NULL, combo_settings_set, NULL, NULL);

static int combo_settings_set(const char *name, size_t len,
                               settings_read_cb read_cb, void *cb_arg) {
    if (strcmp(name, "combos") == 0) {
        if (len != sizeof(stored_combos)) {
            LOG_WRN("Combo settings size mismatch: expected %d, got %d",
                    (int)sizeof(stored_combos), (int)len);
            return -EINVAL;
        }
        int rc = read_cb(cb_arg, &stored_combos, sizeof(stored_combos));
        if (rc >= 0) {
            stored_combo_count = 0;
            for (int i = 0; i < MAX_STORED_COMBOS; i++) {
                if (stored_combos[i].active) {
                    stored_combo_count++;
                }
            }
            combos_loaded_from_settings = true;
            LOG_INF("Loaded %d combos from settings", stored_combo_count);
        }
        return rc;
    }
    return -ENOENT;
}

static int combo_settings_save(void) {
    return settings_save_one("combo/studio/combos",
                              &stored_combos, sizeof(stored_combos));
}

/*
 * Apply stored combos to the runtime combo system.
 * Replaces all current combos with the stored ones.
 */
static int apply_stored_combos(void) {
    int current_count = zmk_combo_get_count();

    /* Remove all existing combos (from last to first) */
    for (int i = current_count - 1; i >= 0; i--) {
        zmk_combo_remove(i);
    }

    /* Add stored combos */
    for (int i = 0; i < MAX_STORED_COMBOS; i++) {
        if (!stored_combos[i].active) {
            continue;
        }

        const char *behavior_name = zmk_behavior_find_behavior_name_from_local_id(
            stored_combos[i].behavior_local_id);
        if (!behavior_name) {
            LOG_WRN("Skipping stored combo %d: unknown behavior ID %d",
                    i, stored_combos[i].behavior_local_id);
            continue;
        }

        struct zmk_combo_cfg_data cfg = {
            .key_position_len = stored_combos[i].key_position_len,
            .require_prior_idle_ms = stored_combos[i].require_prior_idle_ms,
            .timeout_ms = stored_combos[i].timeout_ms,
            .layer_mask = stored_combos[i].layer_mask,
            .behavior_dev = behavior_name,
            .param1 = stored_combos[i].param1,
            .param2 = stored_combos[i].param2,
            .slow_release = stored_combos[i].slow_release,
        };
        memcpy(cfg.key_positions, stored_combos[i].key_positions,
               sizeof(int32_t) * stored_combos[i].key_position_len);

        int ret = zmk_combo_add(&cfg);
        if (ret < 0) {
            LOG_WRN("Failed to restore combo %d: %d", i, ret);
        }
    }

    LOG_INF("Applied %d stored combos", stored_combo_count);
    return 0;
}

/*
 * Snapshot current runtime combos into the stored_combos array.
 */
static int snapshot_combos_to_storage(void) {
    memset(stored_combos, 0, sizeof(stored_combos));
    stored_combo_count = 0;

    int count = zmk_combo_get_count();
    for (int i = 0; i < count && i < MAX_STORED_COMBOS; i++) {
        struct zmk_combo_cfg_data cfg;
        if (zmk_combo_get_at(i, &cfg) < 0) {
            continue;
        }

        memcpy(stored_combos[i].key_positions, cfg.key_positions,
               sizeof(int32_t) * cfg.key_position_len);
        stored_combos[i].key_position_len = cfg.key_position_len;
        stored_combos[i].require_prior_idle_ms = cfg.require_prior_idle_ms;
        stored_combos[i].timeout_ms = cfg.timeout_ms;
        stored_combos[i].layer_mask = cfg.layer_mask;
        stored_combos[i].behavior_local_id =
            zmk_behavior_get_local_id(cfg.behavior_dev);
        stored_combos[i].param1 = cfg.param1;
        stored_combos[i].param2 = cfg.param2;
        stored_combos[i].slow_release = cfg.slow_release;
        stored_combos[i].active = true;
        stored_combo_count++;
    }

    return 0;
}

/*
 * RPC Handlers
 *
 * With combo.options setting max_count:8 for key_positions and
 * max_count:16 for combos, nanopb generates fixed-size arrays
 * instead of callbacks. This makes encoding/decoding straightforward.
 */

zmk_studio_Response get_combos(const zmk_studio_Request *req) {
    LOG_DBG("get_combos called");

    zmk_combo_CombosResponse resp = zmk_combo_CombosResponse_init_zero;
    resp.max_combos = CONFIG_ZMK_COMBO_MAX_DYNAMIC_COMBOS;

    int count = zmk_combo_get_count();
    resp.combos_count = 0;

    for (int i = 0; i < count && i < 16; i++) {
        struct zmk_combo_cfg_data cfg;
        if (zmk_combo_get_at(i, &cfg) < 0) {
            continue;
        }

        zmk_combo_ComboConfig *combo_msg = &resp.combos[resp.combos_count];
        *combo_msg = (zmk_combo_ComboConfig)zmk_combo_ComboConfig_init_zero;

        /* Copy key positions */
        combo_msg->key_positions_count = cfg.key_position_len;
        for (int kp = 0; kp < cfg.key_position_len && kp < 8; kp++) {
            combo_msg->key_positions[kp] = cfg.key_positions[kp];
        }

        /* Set binding */
        combo_msg->binding.behavior_id = zmk_behavior_get_local_id(cfg.behavior_dev);
        combo_msg->binding.param1 = cfg.param1;
        combo_msg->binding.param2 = cfg.param2;

        /* Set other fields */
        combo_msg->timeout_ms = cfg.timeout_ms;
        combo_msg->require_prior_idle_ms = cfg.require_prior_idle_ms;
        combo_msg->layer_mask = cfg.layer_mask;
        combo_msg->slow_release = cfg.slow_release;

        resp.combos_count++;
    }

    LOG_INF("get_combos: returning %d combos (max %d)",
            resp.combos_count, CONFIG_ZMK_COMBO_MAX_DYNAMIC_COMBOS);

    return COMBO_RESPONSE(get_combos, resp);
}

zmk_studio_Response set_combo(const zmk_studio_Request *req) {
    LOG_DBG("set_combo called");

    const zmk_combo_SetComboRequest *set_req =
        &req->subsystem.combo.request_type.set_combo;

    int index = set_req->index;
    int count = zmk_combo_get_count();

    if (index < 0 || index >= count) {
        LOG_WRN("set_combo: invalid index %d (count=%d)", index, count);
        return COMBO_RESPONSE(set_combo,
            zmk_combo_SetComboResponse_SET_COMBO_RESP_ERR_INVALID_INDEX);
    }

    /* Decode the binding */
    zmk_behavior_local_id_t bid = set_req->combo.binding.behavior_id;
    const char *behavior_name = zmk_behavior_find_behavior_name_from_local_id(bid);
    if (!behavior_name) {
        LOG_WRN("set_combo: unknown behavior ID %d", bid);
        return COMBO_RESPONSE(set_combo,
            zmk_combo_SetComboResponse_SET_COMBO_RESP_ERR_INVALID_PARAMS);
    }

    /* Build the combo config from the request */
    struct zmk_combo_cfg_data updated = {0};

    /* Copy key positions from the request (fixed array thanks to .options) */
    updated.key_position_len = set_req->combo.key_positions_count;
    if (updated.key_position_len < 2 || updated.key_position_len > ZMK_COMBO_MAX_KEYS) {
        /* If no key positions provided, keep the current ones */
        struct zmk_combo_cfg_data current;
        if (zmk_combo_get_at(index, &current) < 0) {
            return COMBO_RESPONSE(set_combo,
                zmk_combo_SetComboResponse_SET_COMBO_RESP_ERR_GENERIC);
        }
        memcpy(updated.key_positions, current.key_positions,
               sizeof(int32_t) * current.key_position_len);
        updated.key_position_len = current.key_position_len;
    } else {
        for (int i = 0; i < updated.key_position_len; i++) {
            updated.key_positions[i] = set_req->combo.key_positions[i];
        }
    }

    updated.behavior_dev = behavior_name;
    updated.param1 = set_req->combo.binding.param1;
    updated.param2 = set_req->combo.binding.param2;
    updated.timeout_ms = set_req->combo.timeout_ms > 0 ?
                         set_req->combo.timeout_ms : 50;
    updated.require_prior_idle_ms = set_req->combo.require_prior_idle_ms;
    updated.layer_mask = set_req->combo.layer_mask;
    updated.slow_release = set_req->combo.slow_release;

    int ret = zmk_combo_set_at(index, &updated);
    if (ret < 0) {
        LOG_WRN("set_combo: failed to set combo %d: %d", index, ret);
        return COMBO_RESPONSE(set_combo,
            zmk_combo_SetComboResponse_SET_COMBO_RESP_ERR_GENERIC);
    }

    /* Snapshot and save */
    snapshot_combos_to_storage();
    ret = combo_settings_save();
    if (ret < 0) {
        LOG_WRN("set_combo: failed to save: %d", ret);
    }

    LOG_INF("set_combo: index=%d updated successfully", index);
    return COMBO_RESPONSE(set_combo,
        zmk_combo_SetComboResponse_SET_COMBO_RESP_OK);
}

zmk_studio_Response add_combo(const zmk_studio_Request *req) {
    LOG_DBG("add_combo called");

    const zmk_combo_AddComboRequest *add_req =
        &req->subsystem.combo.request_type.add_combo;

    zmk_combo_AddComboResponse resp = zmk_combo_AddComboResponse_init_zero;

    /* Decode the binding */
    zmk_behavior_local_id_t bid = add_req->combo.binding.behavior_id;
    const char *behavior_name = zmk_behavior_find_behavior_name_from_local_id(bid);
    if (!behavior_name) {
        LOG_WRN("add_combo: unknown behavior ID %d", bid);
        resp.which_result = zmk_combo_AddComboResponse_err_tag;
        resp.result.err = zmk_combo_AddComboErrorCode_ADD_COMBO_ERR_INVALID_PARAMS;
        return COMBO_RESPONSE(add_combo, resp);
    }

    /* Validate key positions */
    if (add_req->combo.key_positions_count < 2 ||
        add_req->combo.key_positions_count > ZMK_COMBO_MAX_KEYS) {
        LOG_WRN("add_combo: invalid key_positions_count %d",
                add_req->combo.key_positions_count);
        resp.which_result = zmk_combo_AddComboResponse_err_tag;
        resp.result.err = zmk_combo_AddComboErrorCode_ADD_COMBO_ERR_INVALID_PARAMS;
        return COMBO_RESPONSE(add_combo, resp);
    }

    struct zmk_combo_cfg_data cfg = {
        .key_position_len = add_req->combo.key_positions_count,
        .require_prior_idle_ms = add_req->combo.require_prior_idle_ms,
        .timeout_ms = add_req->combo.timeout_ms > 0 ? add_req->combo.timeout_ms : 50,
        .layer_mask = add_req->combo.layer_mask,
        .behavior_dev = behavior_name,
        .param1 = add_req->combo.binding.param1,
        .param2 = add_req->combo.binding.param2,
        .slow_release = add_req->combo.slow_release,
    };

    for (int i = 0; i < cfg.key_position_len; i++) {
        cfg.key_positions[i] = add_req->combo.key_positions[i];
    }

    int ret = zmk_combo_add(&cfg);
    if (ret < 0) {
        LOG_WRN("add_combo: failed: %d", ret);
        resp.which_result = zmk_combo_AddComboResponse_err_tag;
        if (ret == -ENOMEM) {
            resp.result.err = zmk_combo_AddComboErrorCode_ADD_COMBO_ERR_NO_SPACE;
        } else {
            resp.result.err = zmk_combo_AddComboErrorCode_ADD_COMBO_ERR_GENERIC;
        }
        return COMBO_RESPONSE(add_combo, resp);
    }

    /* Snapshot and save */
    snapshot_combos_to_storage();
    int save_ret = combo_settings_save();
    if (save_ret < 0) {
        LOG_WRN("add_combo: failed to save: %d", save_ret);
    }

    resp.which_result = zmk_combo_AddComboResponse_ok_tag;
    resp.result.ok = ret; /* new combo index */

    LOG_INF("add_combo: added at index %d", ret);
    return COMBO_RESPONSE(add_combo, resp);
}

zmk_studio_Response remove_combo(const zmk_studio_Request *req) {
    LOG_DBG("remove_combo called");

    const zmk_combo_RemoveComboRequest *rm_req =
        &req->subsystem.combo.request_type.remove_combo;

    int index = rm_req->index;
    int count = zmk_combo_get_count();

    if (index < 0 || index >= count) {
        LOG_WRN("remove_combo: invalid index %d (count=%d)", index, count);
        return COMBO_RESPONSE(remove_combo,
            zmk_combo_RemoveComboResponse_REMOVE_COMBO_RESP_ERR_INVALID_INDEX);
    }

    int ret = zmk_combo_remove(index);
    if (ret < 0) {
        LOG_WRN("remove_combo: failed: %d", ret);
        return COMBO_RESPONSE(remove_combo,
            zmk_combo_RemoveComboResponse_REMOVE_COMBO_RESP_ERR_GENERIC);
    }

    /* Snapshot and save */
    snapshot_combos_to_storage();
    ret = combo_settings_save();
    if (ret < 0) {
        LOG_WRN("remove_combo: failed to save: %d", ret);
    }

    LOG_INF("remove_combo: index=%d removed successfully", index);
    return COMBO_RESPONSE(remove_combo,
        zmk_combo_RemoveComboResponse_REMOVE_COMBO_RESP_OK);
}

ZMK_RPC_SUBSYSTEM_HANDLER(combo, get_combos, ZMK_STUDIO_RPC_HANDLER_SECURED);
ZMK_RPC_SUBSYSTEM_HANDLER(combo, set_combo, ZMK_STUDIO_RPC_HANDLER_SECURED);
ZMK_RPC_SUBSYSTEM_HANDLER(combo, add_combo, ZMK_STUDIO_RPC_HANDLER_SECURED);
ZMK_RPC_SUBSYSTEM_HANDLER(combo, remove_combo, ZMK_STUDIO_RPC_HANDLER_SECURED);

static int combo_settings_reset(void) {
    memset(stored_combos, 0, sizeof(stored_combos));
    stored_combo_count = 0;
    combos_loaded_from_settings = false;
    return combo_settings_save();
}

ZMK_RPC_SUBSYSTEM_SETTINGS_RESET(combo, combo_settings_reset);

static int event_mapper(const zmk_event_t *eh, zmk_studio_Notification *n) { return 0; }
ZMK_RPC_EVENT_MAPPER(combo, event_mapper);

/*
 * Late init: if combos were loaded from settings, apply them.
 * This runs after combo_init (which loads DT combos) and after
 * settings_load() has been called.
 */
static int combo_studio_late_init(void) {
    if (combos_loaded_from_settings) {
        LOG_INF("Applying %d stored combos from settings", stored_combo_count);
        apply_stored_combos();
    }
    return 0;
}

/* Run after settings are loaded (priority 99 to ensure it runs late) */
SYS_INIT(combo_studio_late_init, APPLICATION, 99);
