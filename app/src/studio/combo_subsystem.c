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

/*
 * Suppressed DT defaults ("combo/studio/suppressed"): key-position sets of
 * devicetree default combos removed via remove_combo with suppress_default.
 * They are not re-added at boot; an entry is pruned once any combo occupies
 * the same position set. Identified by position set (order-insensitive), never by
 * devicetree index, which shifts between firmware versions. Unused entries have
 * key_position_len == 0. The array size is fixed so the stored blob layout does
 * not change when the devicetree gains or loses defaults.
 */
struct combo_suppressed_entry {
    int32_t key_positions[ZMK_COMBO_MAX_KEYS];
    int16_t key_position_len;
};

#define MAX_SUPPRESSED_DEFAULTS 16

BUILD_ASSERT(ZMK_COMBOS_DT_COUNT <= MAX_SUPPRESSED_DEFAULTS,
             "More DT default combos than suppressed-default storage entries");

static struct combo_suppressed_entry suppressed_defaults[MAX_SUPPRESSED_DEFAULTS];
static int suppressed_default_count = 0;
/* One-time decode-bug heal marker (see apply_stored_combos). Loaded from
 * settings; once set, stored &none combos on DT-default positions are
 * treated as intentional user overrides again. */
static bool combo_heal_done = false;
/* Set when the healed combo blob failed to re-save: keeps the heal armed for
 * the next boot instead of stamping the marker over an unhealed store. */
static bool combo_heal_save_failed = false;

/* Only the DT defaults bound to behaviors that pre-2026-07 Studio could not
 * decode (and therefore rewrote as &none) are heal targets: the A+M+L toggle
 * and the sleep combo. &none stored on any other DT default's positions is
 * treated as an intentional user override and left alone. */
static bool combo_heal_target_default(const char *behavior_dev) {
    return behavior_dev && (strcmp(behavior_dev, "aml_toggle") == 0 ||
                            strcmp(behavior_dev, "z_so_off") == 0);
}

/* Forward declaration */
static int combo_settings_set(const char *name, size_t len,
                               settings_read_cb read_cb, void *cb_arg);
static int combo_settings_commit(void);
static int apply_stored_combos(void);

SETTINGS_STATIC_HANDLER_DEFINE(zmk_combo_studio, "combo/studio",
                                NULL, combo_settings_set, combo_settings_commit, NULL);

/* Called after settings_load() completes — all subsystems are initialized */
static int combo_settings_commit(void) {
    if (combos_loaded_from_settings || suppressed_default_count > 0) {
        LOG_INF("Settings commit: applying %d stored combos, %d suppressed defaults",
                stored_combo_count, suppressed_default_count);
        apply_stored_combos();
    }
    /* Mark the one-time decode-bug heal as done so later intentional &none
     * overrides of DT-default combos persist normally. Saving during commit
     * mirrors the behavior local_id table (behavior.c). Skipped when the
     * healed blob failed to save — the heal then retries next boot. */
    if (!combo_heal_done && !combo_heal_save_failed) {
        static const uint8_t done = 1;
        int rc = settings_save_one("combo/studio/heal1", &done, sizeof(done));
        if (rc == 0) {
            combo_heal_done = true;
        } else {
            /* Heal stays armed for the next boot; it is idempotent. */
            LOG_WRN("Failed to persist combo heal marker: %d", rc);
        }
    }
    return 0;
}

static int combo_settings_set(const char *name, size_t len,
                               settings_read_cb read_cb, void *cb_arg) {
    if (strcmp(name, "combos") == 0) {
        if (len != sizeof(stored_combos)) {
            LOG_WRN("Combo settings size mismatch: expected %d, got %d — reading what we can",
                    (int)sizeof(stored_combos), (int)len);
            /* Clear and read what we can for migration */
            memset(&stored_combos, 0, sizeof(stored_combos));
            size_t read_len = len < sizeof(stored_combos) ? len : sizeof(stored_combos);
            int rc = read_cb(cb_arg, &stored_combos, read_len);
            if (rc >= 0) {
                stored_combo_count = 0;
                for (int i = 0; i < MAX_STORED_COMBOS; i++) {
                    if (stored_combos[i].active) {
                        stored_combo_count++;
                    }
                }
                combos_loaded_from_settings = true;
                LOG_INF("Migrated %d combos from settings (will apply in late_init)", stored_combo_count);
            }
            return rc;
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
            LOG_INF("Loaded %d combos from settings (will apply in late_init)", stored_combo_count);
        }
        return rc;
    }
    if (strcmp(name, "heal1") == 0) {
        uint8_t done = 0;
        if (read_cb(cb_arg, &done, sizeof(done)) >= 0 && done) {
            combo_heal_done = true;
        }
        return 0;
    }
    if (strcmp(name, "suppressed") == 0) {
        if (len != sizeof(suppressed_defaults)) {
            LOG_WRN("Suppressed combo settings size mismatch: expected %d, got %d — ignoring",
                    (int)sizeof(suppressed_defaults), (int)len);
            return 0;
        }
        int rc = read_cb(cb_arg, &suppressed_defaults, sizeof(suppressed_defaults));
        if (rc < 0) {
            memset(suppressed_defaults, 0, sizeof(suppressed_defaults));
            suppressed_default_count = 0;
            return rc;
        }
        suppressed_default_count = 0;
        for (int i = 0; i < MAX_SUPPRESSED_DEFAULTS; i++) {
            int16_t len_i = suppressed_defaults[i].key_position_len;
            if (len_i >= 2 && len_i <= ZMK_COMBO_MAX_KEYS) {
                suppressed_default_count++;
            } else {
                suppressed_defaults[i].key_position_len = 0;
            }
        }
        LOG_INF("Loaded %d suppressed default combos from settings", suppressed_default_count);
        return 0;
    }
    return -ENOENT;
}

/* Saves the combo blob and the suppressed-defaults list together. */
static int combo_settings_save(void) {
    int rc = settings_save_one("combo/studio/combos",
                               &stored_combos, sizeof(stored_combos));
    int rc_suppressed = settings_save_one("combo/studio/suppressed",
                                          &suppressed_defaults, sizeof(suppressed_defaults));
    return rc != 0 ? rc : rc_suppressed;
}

/* True when every position in a also appears in b. */
static bool combo_positions_subset(const int32_t *a, int a_len, const int32_t *b, int b_len) {
    for (int i = 0; i < a_len; i++) {
        bool found = false;
        for (int j = 0; j < b_len; j++) {
            if (a[i] == b[j]) {
                found = true;
                break;
            }
        }
        if (!found) {
            return false;
        }
    }
    return true;
}

/* Order-insensitive set comparison of two key-position lists. Checks both
 * directions so a list with duplicates (e.g. [17,17]) never matches a set with
 * distinct members ([17,18]). */
static bool combo_positions_equal(const int32_t *a, int a_len, const int32_t *b, int b_len) {
    return a_len == b_len && combo_positions_subset(a, a_len, b, b_len) &&
           combo_positions_subset(b, b_len, a, a_len);
}

static bool combo_default_is_suppressed(const int32_t *positions, int len) {
    for (int i = 0; i < MAX_SUPPRESSED_DEFAULTS; i++) {
        if (suppressed_defaults[i].key_position_len != 0 &&
            combo_positions_equal(suppressed_defaults[i].key_positions,
                                  suppressed_defaults[i].key_position_len, positions, len)) {
            return true;
        }
    }
    return false;
}

static bool combo_positions_occupied(const int32_t *positions, int len) {
    int count = zmk_combo_get_count();
    for (int i = 0; i < count; i++) {
        struct zmk_combo_cfg_data cfg;
        if (zmk_combo_get_at(i, &cfg) < 0) {
            continue;
        }
        if (combo_positions_equal(cfg.key_positions, cfg.key_position_len, positions, len)) {
            return true;
        }
    }
    return false;
}

static bool combo_positions_match_dt_default(const int32_t *positions, int len) {
    int dt_count = zmk_combo_dt_default_count();
    for (int d = 0; d < dt_count; d++) {
        int32_t dt_positions[ZMK_COMBO_MAX_KEYS];
        int dt_len = 0;
        if (zmk_combo_dt_default_positions(d, dt_positions, &dt_len) == 0 &&
            combo_positions_equal(dt_positions, dt_len, positions, len)) {
            return true;
        }
    }
    return false;
}

/* Tombstone the DT default on this position set (remove_combo with
 * suppress_default). No-op when the set is not a DT default's or already
 * suppressed. */
static void suppress_dt_default(const int32_t *positions, int len) {
    if (!combo_positions_match_dt_default(positions, len) ||
        combo_default_is_suppressed(positions, len)) {
        return;
    }
    if (suppressed_default_count >= MAX_SUPPRESSED_DEFAULTS) {
        LOG_WRN("No space to record a suppressed DT combo");
        return;
    }
    struct combo_suppressed_entry *entry = &suppressed_defaults[suppressed_default_count];
    memset(entry, 0, sizeof(*entry));
    memcpy(entry->key_positions, positions, sizeof(int32_t) * len);
    entry->key_position_len = len;
    suppressed_default_count++;
}

/* Drop suppressed entries whose position set is occupied by a runtime combo
 * (placing any combo on those keys clears the tombstone) or no longer matches
 * any DT default (orphans after a firmware update). Keeps used entries packed
 * from index 0 with zeroed unused entries. */
static void prune_suppressed_defaults(void) {
    int kept = 0;
    for (int i = 0; i < MAX_SUPPRESSED_DEFAULTS; i++) {
        const struct combo_suppressed_entry *entry = &suppressed_defaults[i];
        if (entry->key_position_len == 0 ||
            combo_positions_occupied(entry->key_positions, entry->key_position_len) ||
            !combo_positions_match_dt_default(entry->key_positions, entry->key_position_len)) {
            continue;
        }
        if (kept != i) {
            suppressed_defaults[kept] = *entry;
        }
        kept++;
    }
    memset(&suppressed_defaults[kept], 0,
           sizeof(suppressed_defaults[0]) * (MAX_SUPPRESSED_DEFAULTS - kept));
    suppressed_default_count = kept;
}

/*
 * Apply stored combos to the runtime combo system.
 * Replaces all current combos with the stored ones.
 */
static int apply_stored_combos(void) {
    /* Don't replace DT combos if no studio combos are stored and no DT default
     * is suppressed. With suppressed defaults but zero stored combos, fall
     * through: all runtime combos are removed and only the non-suppressed DT
     * defaults are re-added below. */
    if (stored_combo_count == 0 && suppressed_default_count == 0) {
        LOG_INF("No stored combos — keeping DT defaults");
        return 0;
    }

    int current_count = zmk_combo_get_count();

    /* Remove all existing combos (from last to first) */
    for (int i = current_count - 1; i >= 0; i--) {
        zmk_combo_remove(i);
    }

    int applied_count = 0;
    int healed_count = 0;

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

        if (zmk_combo_cfg_is_dt_default(&cfg)) {
            LOG_INF("Skipping stored combo %d: matches DT default", i);
            continue;
        }

        /* One-time self-heal: pre-2026-07 Studio builds mis-decoded DT-default
         * combos bound to custom behaviors (&aml_tog, &soft_off) as "none" on
         * Read and persisted &none at the same key positions on Write — which
         * then blocked zmk_combo_add_missing_dt_defaults() forever (position
         * match). On the first boot of this firmware, drop such entries and
         * let the DT default come back below. Runs ONCE (combo/studio/heal1
         * marker, set in combo_settings_commit) and only for the two defaults
         * that could actually be corrupted (combo_heal_target_default). */
        if (!combo_heal_done && strcmp(behavior_name, "none") == 0 &&
            combo_heal_target_default(zmk_combo_dt_default_behavior_at_positions(&cfg))) {
            LOG_WRN("Dropping stored &none combo %d shadowing a DT default (Studio decode-bug heal)",
                    i);
            /* Deactivate in the staging array too, and re-save below —
             * otherwise the entry would be re-applied on the next boot once
             * the heal marker is set. */
            stored_combos[i].active = false;
            stored_combo_count--;
            healed_count++;
            continue;
        }

        int ret = zmk_combo_add(&cfg);
        if (ret < 0) {
            LOG_WRN("Failed to restore combo %d: %d", i, ret);
        } else {
            applied_count++;
        }
    }

    LOG_INF("Applied %d stored combos", applied_count);
    zmk_combo_add_missing_dt_defaults_filtered(combo_default_is_suppressed);
    /* Reconcile tombstones with the rebuilt runtime list, after the filtered
     * add-missing step has used them: drop entries on occupied keys (e.g. left
     * stale by a power loss between the combos and suppressed writes) and
     * orphans. In memory only; the next save persists the pruned list. */
    prune_suppressed_defaults();
    if (healed_count > 0) {
        int rc = combo_settings_save();
        if (rc != 0) {
            combo_heal_save_failed = true;
        }
        LOG_WRN("Healed %d corrupted combo(s); storage re-save %s (%d)", healed_count,
                rc == 0 ? "ok" : "failed", rc);
    }
    return 0;
}

/*
 * Snapshot current runtime combos into the stored_combos array.
 */
static int snapshot_combos_to_storage(void) {
    memset(stored_combos, 0, sizeof(stored_combos));
    stored_combo_count = 0;

    int count = zmk_combo_get_count();
    int stored_idx = 0;
    for (int i = 0; i < count && i < MAX_STORED_COMBOS; i++) {
        struct zmk_combo_cfg_data cfg;
        if (zmk_combo_get_at(i, &cfg) < 0) {
            continue;
        }

        if (cfg.from_dt_default || zmk_combo_cfg_is_dt_default(&cfg)) {
            continue;
        }

        if (stored_idx >= MAX_STORED_COMBOS) {
            break;
        }

        memcpy(stored_combos[stored_idx].key_positions, cfg.key_positions,
               sizeof(int32_t) * cfg.key_position_len);
        stored_combos[stored_idx].key_position_len = cfg.key_position_len;
        stored_combos[stored_idx].require_prior_idle_ms = cfg.require_prior_idle_ms;
        stored_combos[stored_idx].timeout_ms = cfg.timeout_ms;
        stored_combos[stored_idx].layer_mask = cfg.layer_mask;
        stored_combos[stored_idx].behavior_local_id =
            zmk_behavior_get_local_id(cfg.behavior_dev);
        stored_combos[stored_idx].param1 = cfg.param1;
        stored_combos[stored_idx].param2 = cfg.param2;
        stored_combos[stored_idx].slow_release = cfg.slow_release;
        stored_combos[stored_idx].active = true;
        stored_combo_count++;
        stored_idx++;
    }

    prune_suppressed_defaults();
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
        /* has_binding must be set for nanopb to serialize the submessage.
         * Without it, clients (iOS app, ZMK Studio web) receive an empty
         * binding field even though the keyboard has it configured.
         * Reported by an iOS ZMK Studio client author on Discord 2026-05-11. */
        combo_msg->has_binding = true;
        combo_msg->binding.behavior_id = zmk_behavior_get_local_id(cfg.behavior_dev);
        combo_msg->binding.param1 = cfg.param1;
        combo_msg->binding.param2 = cfg.param2;

        /* Set other fields */
        combo_msg->timeout_ms = cfg.timeout_ms;
        combo_msg->require_prior_idle_ms = cfg.require_prior_idle_ms;
        combo_msg->layer_mask = cfg.layer_mask;
        combo_msg->slow_release = cfg.slow_release;
        combo_msg->is_default = cfg.from_dt_default || zmk_combo_cfg_is_dt_default(&cfg);

        resp.combos_count++;
    }
    resp.default_suppression = true;

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

    /* Capture the positions before removal for an explicit default tombstone */
    struct zmk_combo_cfg_data removed;
    bool have_removed = zmk_combo_get_at(index, &removed) == 0;

    int ret = zmk_combo_remove(index);
    if (ret < 0) {
        LOG_WRN("remove_combo: failed: %d", ret);
        return COMBO_RESPONSE(remove_combo,
            zmk_combo_RemoveComboResponse_REMOVE_COMBO_RESP_ERR_GENERIC);
    }

    /* Only an explicit request suppresses a DT default. A plain remove (also
     * used by Studio's Write flow before re-adding) leaves no tombstone, so an
     * interrupted Write never loses a default past the next reboot. */
    if (rm_req->suppress_default && have_removed) {
        suppress_dt_default(removed.key_positions, removed.key_position_len);
    }

    /* Snapshot (prunes tombstones on occupied keys) and save */
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
    memset(suppressed_defaults, 0, sizeof(suppressed_defaults));
    suppressed_default_count = 0;
    return combo_settings_save();
}

ZMK_RPC_SUBSYSTEM_SETTINGS_RESET(combo, combo_settings_reset);

static int event_mapper(const zmk_event_t *eh, zmk_studio_Notification *n) { return -ENOTSUP; }
ZMK_RPC_EVENT_MAPPER(combo, event_mapper);

/*
 * Late init: if combos were loaded from settings, apply them.
 * This runs after combo_init (which loads DT combos) and after
 * settings_load() has been called.
 */
static int combo_studio_late_init(void) {
    /* Combo application now happens in combo_settings_commit() */
    return 0;
}

/* Run after settings are loaded (priority 99 to ensure it runs late) */
SYS_INIT(combo_studio_late_init, APPLICATION, 99);
