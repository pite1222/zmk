/*
 * Copyright (c) 2020 The ZMK Contributors
 *
 * SPDX-License-Identifier: MIT
 */

#define DT_DRV_COMPAT zmk_combos

#include <zephyr/device.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/dlist.h>
#include <zephyr/sys/util.h>
#include <zephyr/kernel.h>

#include <drivers/behavior.h>

#include <zmk/behavior.h>
#include <zmk/event_manager.h>
#include <zmk/events/position_state_changed.h>
#include <zmk/events/keycode_state_changed.h>
#include <zmk/hid.h>
#include <zmk/matrix.h>
#include <zmk/keymap.h>
#include <zmk/virtual_key_position.h>
#include <zmk/combos.h>

LOG_MODULE_DECLARE(zmk, CONFIG_ZMK_LOG_LEVEL);

#if DT_HAS_COMPAT_STATUS_OKAY(DT_DRV_COMPAT)

#if CONFIG_ZMK_COMBO_MAX_KEYS_PER_COMBO > 0

#warning                                                                                           \
    "CONFIG_ZMK_COMBO_MAX_KEYS_PER_COMBO is deprecated, and is auto-calculated from the devicetree now."

#endif

#if CONFIG_ZMK_COMBO_MAX_COMBOS_PER_KEY > 0

#warning "CONFIG_ZMK_COMBO_MAX_COMBOS_PER_KEY is deprecated, and is auto-calculated."

#endif

/*
 * Dynamic combo configuration.
 * We use a fixed-size array with ZMK_COMBO_MAX_KEYS keys per combo.
 * The first DT-defined combos are loaded at init time; additional combos
 * can be added/modified/removed at runtime via the Studio RPC.
 */
#define MAX_COMBO_KEYS ZMK_COMBO_MAX_KEYS
#define MAX_COMBOS CONFIG_ZMK_COMBO_MAX_DYNAMIC_COMBOS

struct combo_cfg {
    int32_t key_positions[MAX_COMBO_KEYS];
    int16_t key_position_len;
    int16_t require_prior_idle_ms;
    int32_t timeout_ms;
    uint32_t layer_mask;
    struct zmk_behavior_binding behavior;
    bool slow_release;
    bool active; /* whether this slot is in use */
};

struct active_combo {
    uint16_t combo_idx;
    uint16_t key_positions_pressed_count;
    struct zmk_position_state_changed_event key_positions_pressed[MAX_COMBO_KEYS];
};

/*
 * DT-defined combo data (read-only, used to initialize the dynamic array).
 */
#define COMBOS_KEYS_BYTE_ARRAY(node_id)                                                            \
    uint8_t _CONCAT(combo_prop_, node_id)[DT_PROP_LEN(node_id, key_positions)];

/* Compute the max key count from DT combos (for DT init only) */
#define DT_MAX_COMBO_KEYS sizeof(union {DT_INST_FOREACH_CHILD(0, COMBOS_KEYS_BYTE_ARRAY)})

#define PROP_BIT_AT_IDX(n, prop, idx) BIT(DT_PROP_BY_IDX(n, prop, idx))

#define NODE_PROP_BITMASK(n, prop)                                                                 \
    COND_CODE_1(DT_NODE_HAS_PROP(n, prop),                                                         \
                (DT_FOREACH_PROP_ELEM_SEP(n, prop, PROP_BIT_AT_IDX, (|))), (0))

#define COMBO_INST(n, positions)                                                                   \
    COND_CODE_1(IS_EQ(DT_PROP_LEN(n, key_positions), positions),                                   \
                (                                                                                  \
                    {                                                                              \
                        .timeout_ms = DT_PROP(n, timeout_ms),                                      \
                        .require_prior_idle_ms = DT_PROP(n, require_prior_idle_ms),                \
                        .key_positions = DT_PROP(n, key_positions),                                \
                        .key_position_len = DT_PROP_LEN(n, key_positions),                         \
                        .behavior = ZMK_KEYMAP_EXTRACT_BINDING(0, n),                              \
                        .slow_release = DT_PROP(n, slow_release),                                  \
                        .layer_mask = NODE_PROP_BITMASK(n, layers),                                \
                    }, ),                                                                          \
                ())

#define COMBO_CONFIGS_WITH_MATCHING_POSITIONS_LEN(positions, _ignore)                              \
    DT_INST_FOREACH_CHILD_VARGS(0, COMBO_INST, positions)

/* DT-defined combos (read-only, used only for initialization) */
static const struct combo_cfg dt_combos[] = {
    LISTIFY(20, COMBO_CONFIGS_WITH_MATCHING_POSITIONS_LEN, (), 0)};

#define DT_COMBO_COUNT ARRAY_SIZE(dt_combos)

/*
 * Dynamic combo array - mutable at runtime.
 * Bitmask arrays are sized for MAX_COMBOS.
 */
#define BYTES_FOR_COMBOS_MASK DIV_ROUND_UP(MAX_COMBOS, 32)

static struct combo_cfg combos[MAX_COMBOS];
static int combo_count = 0; /* number of active combos */

uint8_t pressed_keys_count = 0;
struct zmk_position_state_changed_event pressed_keys[MAX_COMBO_KEYS] = {};
uint32_t candidates[BYTES_FOR_COMBOS_MASK];
int16_t fully_pressed_combo = INT16_MAX;
uint32_t combo_lookup[ZMK_KEYMAP_LEN][BYTES_FOR_COMBOS_MASK] = {};
struct active_combo active_combos[CONFIG_ZMK_COMBO_MAX_PRESSED_COMBOS] = {};
uint8_t active_combo_count = 0;

struct k_work_delayable timeout_task;
int64_t timeout_task_timeout_at;

int64_t last_tapped_timestamp = INT32_MIN;
int64_t last_combo_timestamp = INT32_MIN;

static void store_last_tapped(int64_t timestamp) {
    if (timestamp > last_combo_timestamp) {
        last_tapped_timestamp = timestamp;
    }
}

/* Rebuild the combo_lookup table from scratch */
static void rebuild_combo_lookup(void) {
    memset(combo_lookup, 0, sizeof(combo_lookup));
    for (int i = 0; i < MAX_COMBOS; i++) {
        if (!combos[i].active) {
            continue;
        }
        for (int kp = 0; kp < combos[i].key_position_len; kp++) {
            int32_t pos = combos[i].key_positions[kp];
            if (pos >= 0 && pos < ZMK_KEYMAP_LEN) {
                sys_bitfield_set_bit((mem_addr_t)&combo_lookup[pos], i);
            }
        }
    }
}

static int initialize_combo(size_t index) {
    const struct combo_cfg *new_combo = &combos[index];
    if (!new_combo->active) {
        return 0;
    }

    for (size_t kp = 0; kp < new_combo->key_position_len; kp++) {
        int32_t pos = new_combo->key_positions[kp];
        if (pos >= 0 && pos < ZMK_KEYMAP_LEN) {
            sys_bitfield_set_bit((mem_addr_t)&combo_lookup[pos], index);
        }
    }

    return 0;
}

static bool combo_active_on_layer(const struct combo_cfg *combo, uint8_t layer) {
    if (!combo->layer_mask) {
        return true;
    }

    return combo->layer_mask & BIT(layer);
}

static bool is_quick_tap(const struct combo_cfg *combo, int64_t timestamp) {
    return (last_tapped_timestamp + combo->require_prior_idle_ms) > timestamp;
}

static int setup_candidates_for_first_keypress(int32_t position, int64_t timestamp) {
    int number_of_combo_candidates = 0;
    uint8_t highest_active_layer = zmk_keymap_highest_layer_active();

    for (int i = 0; i < MAX_COMBOS; i++) {
        if (!combos[i].active) {
            continue;
        }
        if (sys_bitfield_test_bit((mem_addr_t)&combo_lookup[position], i)) {
            const struct combo_cfg *combo = &combos[i];
            if (combo_active_on_layer(combo, highest_active_layer) &&
                !is_quick_tap(combo, timestamp)) {
                sys_bitfield_set_bit((mem_addr_t)&candidates, i);
                number_of_combo_candidates++;
            }
        }
    }

    return number_of_combo_candidates;
}

static inline uint8_t zero_one_or_more_bits(uint32_t field) {
    if (field == 0) {
        return 0;
    }
    if ((field & (field - 1)) == 0) {
        return 1;
    }
    return 2;
}

static int filter_candidates(int32_t position) {
    int matches = 0;
    for (int i = 0; i < BYTES_FOR_COMBOS_MASK; i++) {
        candidates[i] &= combo_lookup[position][i];
        if (matches < 2) {
            matches += zero_one_or_more_bits(candidates[i]);
        }
    }

    LOG_DBG("combo matches after filter %d", matches);
    return matches;
}

static int64_t first_candidate_timeout() {
    if (pressed_keys_count == 0) {
        return LONG_MAX;
    }

    int64_t first_timeout = LONG_MAX;
    for (int i = 0; i < MAX_COMBOS; i++) {
        if (!combos[i].active) {
            continue;
        }
        if (sys_bitfield_test_bit((mem_addr_t)&candidates, i)) {
            first_timeout = MIN(first_timeout, combos[i].timeout_ms);
        }
    }

    return pressed_keys[0].data.timestamp + first_timeout;
}

static inline bool candidate_is_completely_pressed(const struct combo_cfg *candidate) {
    return candidate->key_position_len == pressed_keys_count;
}

static int cleanup();

static int filter_timed_out_candidates(int64_t timestamp) {
    __ASSERT(pressed_keys_count > 0, "Searching for a candidate timeout with no keys pressed");

    int remaining_candidates = 0;
    for (int i = 0; i < MAX_COMBOS; i++) {
        if (!combos[i].active) {
            continue;
        }
        if (sys_bitfield_test_bit((mem_addr_t)&candidates, i)) {

            if (pressed_keys[0].data.timestamp + combos[i].timeout_ms > timestamp) {
                remaining_candidates++;
            } else {
                sys_bitfield_clear_bit((mem_addr_t)&candidates, i);
            }
        }
    }

    LOG_DBG(
        "after filtering out timed out combo candidates: remaining_candidates=%d timestamp=%lld",
        remaining_candidates, timestamp);

    return remaining_candidates;
}

static int capture_pressed_key(const struct zmk_position_state_changed *ev) {
    if (pressed_keys_count == MAX_COMBO_KEYS) {
        return ZMK_EV_EVENT_BUBBLE;
    }

    pressed_keys[pressed_keys_count++] = copy_raised_zmk_position_state_changed(ev);
    return ZMK_EV_EVENT_CAPTURED;
}

const struct zmk_listener zmk_listener_combo;

static int release_pressed_keys() {
    uint8_t count = pressed_keys_count;
    pressed_keys_count = 0;
    for (int i = 0; i < count; i++) {
        struct zmk_position_state_changed_event *ev = &pressed_keys[i];
        if (i == 0) {
            LOG_DBG("combo: releasing position event %d", ev->data.position);
            ZMK_EVENT_RELEASE(*ev);
        } else {
            LOG_DBG("combo: reraising position event %d", ev->data.position);
            ZMK_EVENT_RAISE(*ev);
        }
    }

    return count;
}

static inline int press_combo_behavior(int combo_idx, const struct combo_cfg *combo,
                                       int32_t timestamp) {
    struct zmk_behavior_binding_event event = {
        .position = ZMK_VIRTUAL_KEY_POSITION_COMBO(combo_idx),
        .timestamp = timestamp,
#if IS_ENABLED(CONFIG_ZMK_SPLIT)
        .source = ZMK_POSITION_STATE_CHANGE_SOURCE_LOCAL,
#endif
    };

    last_combo_timestamp = timestamp;

    return zmk_behavior_invoke_binding(&combo->behavior, event, true);
}

static inline int release_combo_behavior(int combo_idx, const struct combo_cfg *combo,
                                         int32_t timestamp) {
    struct zmk_behavior_binding_event event = {
        .position = ZMK_VIRTUAL_KEY_POSITION_COMBO(combo_idx),
        .timestamp = timestamp,
#if IS_ENABLED(CONFIG_ZMK_SPLIT)
        .source = ZMK_POSITION_STATE_CHANGE_SOURCE_LOCAL,
#endif
    };

    return zmk_behavior_invoke_binding(&combo->behavior, event, false);
}

static void move_pressed_keys_to_active_combo(struct active_combo *active_combo) {

    int combo_length = MIN(pressed_keys_count, combos[active_combo->combo_idx].key_position_len);
    for (int i = 0; i < combo_length; i++) {
        active_combo->key_positions_pressed[i] = pressed_keys[i];
    }
    active_combo->key_positions_pressed_count = combo_length;

    for (int i = 0; i + combo_length < pressed_keys_count; i++) {
        pressed_keys[i] = pressed_keys[i + combo_length];
    }

    pressed_keys_count -= combo_length;
}

static struct active_combo *store_active_combo(int32_t combo_idx) {
    for (int i = 0; i < CONFIG_ZMK_COMBO_MAX_PRESSED_COMBOS; i++) {
        if (active_combos[i].combo_idx == UINT16_MAX) {
            active_combos[i].combo_idx = combo_idx;
            active_combo_count++;
            return &active_combos[i];
        }
    }
    LOG_ERR("Unable to store combo; already %d active. Increase "
            "CONFIG_ZMK_COMBO_MAX_PRESSED_COMBOS",
            CONFIG_ZMK_COMBO_MAX_PRESSED_COMBOS);
    return NULL;
}

static void activate_combo(int combo_idx) {
    struct active_combo *active_combo = store_active_combo(combo_idx);
    if (active_combo == NULL) {
        release_pressed_keys();
        return;
    }
    move_pressed_keys_to_active_combo(active_combo);
    press_combo_behavior(combo_idx, &combos[combo_idx],
                         active_combo->key_positions_pressed[0].data.timestamp);
}

static void deactivate_combo(int active_combo_index) {
    active_combo_count--;
    if (active_combo_index != active_combo_count) {
        memcpy(&active_combos[active_combo_index], &active_combos[active_combo_count],
               sizeof(struct active_combo));
    }
    active_combos[active_combo_count] = (struct active_combo){0};
    active_combos[active_combo_count].combo_idx = UINT16_MAX;
}

static bool release_combo_key(int32_t position, int64_t timestamp) {
    for (int combo_idx = 0; combo_idx < active_combo_count; combo_idx++) {
        struct active_combo *active_combo = &active_combos[combo_idx];

        bool key_released = false;
        bool all_keys_pressed = active_combo->key_positions_pressed_count ==
                                combos[active_combo->combo_idx].key_position_len;
        bool all_keys_released = true;
        for (int i = 0; i < active_combo->key_positions_pressed_count; i++) {
            if (key_released) {
                active_combo->key_positions_pressed[i - 1] = active_combo->key_positions_pressed[i];
                all_keys_released = false;
            } else if (active_combo->key_positions_pressed[i].data.position != position) {
                all_keys_released = false;
            } else {
                key_released = true;
            }
        }

        if (key_released) {
            active_combo->key_positions_pressed_count--;
            const struct combo_cfg *c = &combos[active_combo->combo_idx];
            if ((c->slow_release && all_keys_released) || (!c->slow_release && all_keys_pressed)) {
                release_combo_behavior(active_combo->combo_idx, c, timestamp);
            }
            if (all_keys_released) {
                deactivate_combo(combo_idx);
            }
            return true;
        }
    }
    return false;
}

static int cleanup() {
    k_work_cancel_delayable(&timeout_task);
    memset(candidates, 0, BYTES_FOR_COMBOS_MASK * sizeof(uint32_t));
    if (fully_pressed_combo != INT16_MAX) {
        activate_combo(fully_pressed_combo);
        fully_pressed_combo = INT16_MAX;
    }
    return release_pressed_keys();
}

static void update_timeout_task() {
    int64_t first_timeout = first_candidate_timeout();
    if (timeout_task_timeout_at == first_timeout) {
        return;
    }
    if (first_timeout == LLONG_MAX) {
        timeout_task_timeout_at = 0;
        k_work_cancel_delayable(&timeout_task);
        return;
    }
    if (k_work_schedule(&timeout_task, K_MSEC(first_timeout - k_uptime_get())) >= 0) {
        timeout_task_timeout_at = first_timeout;
    }
}

static int position_state_down(const zmk_event_t *ev, struct zmk_position_state_changed *data) {
    int num_candidates;
    if (!pressed_keys_count) {
        num_candidates = setup_candidates_for_first_keypress(data->position, data->timestamp);
        if (num_candidates == 0) {
            return ZMK_EV_EVENT_BUBBLE;
        }
    } else {
        filter_timed_out_candidates(data->timestamp);
        num_candidates = filter_candidates(data->position);
    }

    LOG_DBG("combo: capturing position event %d", data->position);
    int ret = capture_pressed_key(data);
    update_timeout_task();

    if (num_candidates) {
        for (int i = 0; i < MAX_COMBOS; i++) {
            if (!combos[i].active) {
                continue;
            }
            if (sys_bitfield_test_bit((mem_addr_t)&candidates, i)) {
                const struct combo_cfg *candidate_combo = &combos[i];
                if (candidate_is_completely_pressed(candidate_combo)) {
                    fully_pressed_combo = i;
                    if (num_candidates == 1) {
                        cleanup();
                    }
                }

                return ret;
            }
        }
    } else {
        cleanup();
        return ret;
    }

    return -EINVAL;
}

static int position_state_up(const zmk_event_t *ev, struct zmk_position_state_changed *data) {
    int released_keys = cleanup();
    if (release_combo_key(data->position, data->timestamp)) {
        return ZMK_EV_EVENT_HANDLED;
    }
    if (released_keys > 1) {
        struct zmk_position_state_changed_event dupe_ev =
            copy_raised_zmk_position_state_changed(data);
        ZMK_EVENT_RAISE(dupe_ev);
        return ZMK_EV_EVENT_CAPTURED;
    }
    return ZMK_EV_EVENT_BUBBLE;
}

static void combo_timeout_handler(struct k_work *item) {
    if (timeout_task_timeout_at == 0 || k_uptime_get() < timeout_task_timeout_at) {
        return;
    }
    if (filter_timed_out_candidates(timeout_task_timeout_at) == 0) {
        LOG_DBG("CLEANUP!");
        cleanup();
    }

    LOG_DBG("ABOUT TO UPDATE IN TIMEOUT");
    update_timeout_task();
}

static int position_state_changed_listener(const zmk_event_t *ev) {
    struct zmk_position_state_changed *data = as_zmk_position_state_changed(ev);
    if (data == NULL) {
        return ZMK_EV_EVENT_BUBBLE;
    }

    if (data->state) {
        return position_state_down(ev, data);
    } else {
        return position_state_up(ev, data);
    }
}

static int keycode_state_changed_listener(const zmk_event_t *eh) {
    struct zmk_keycode_state_changed *ev = as_zmk_keycode_state_changed(eh);
    if (ev->state && !is_mod(ev->usage_page, ev->keycode)) {
        store_last_tapped(ev->timestamp);
    }
    return ZMK_EV_EVENT_BUBBLE;
}

int behavior_combo_listener(const zmk_event_t *eh) {
    if (as_zmk_position_state_changed(eh) != NULL) {
        return position_state_changed_listener(eh);
    } else if (as_zmk_keycode_state_changed(eh) != NULL) {
        return keycode_state_changed_listener(eh);
    }
    return ZMK_EV_EVENT_BUBBLE;
}

ZMK_LISTENER(combo, behavior_combo_listener);
ZMK_SUBSCRIPTION(combo, zmk_position_state_changed);
ZMK_SUBSCRIPTION(combo, zmk_keycode_state_changed);

/*
 * ============================================================
 * Public API for dynamic combo management (used by combo_subsystem.c)
 * ============================================================
 */

int zmk_combo_get_count(void) {
    return combo_count;
}

int zmk_combo_get_at(int index, struct zmk_combo_cfg_data *out) {
    if (index < 0 || index >= MAX_COMBOS) {
        return -EINVAL;
    }

    /* Find the Nth active combo */
    int active_idx = 0;
    for (int i = 0; i < MAX_COMBOS; i++) {
        if (!combos[i].active) {
            continue;
        }
        if (active_idx == index) {
            memcpy(out->key_positions, combos[i].key_positions,
                   sizeof(int32_t) * combos[i].key_position_len);
            out->key_position_len = combos[i].key_position_len;
            out->require_prior_idle_ms = combos[i].require_prior_idle_ms;
            out->timeout_ms = combos[i].timeout_ms;
            out->layer_mask = combos[i].layer_mask;
            out->behavior_dev = combos[i].behavior.behavior_dev;
            out->param1 = combos[i].behavior.param1;
            out->param2 = combos[i].behavior.param2;
            out->slow_release = combos[i].slow_release;
            return 0;
        }
        active_idx++;
    }
    return -EINVAL;
}

/* Helper: find the raw slot index for the Nth active combo */
static int find_slot_for_active_index(int index) {
    int active_idx = 0;
    for (int i = 0; i < MAX_COMBOS; i++) {
        if (!combos[i].active) {
            continue;
        }
        if (active_idx == index) {
            return i;
        }
        active_idx++;
    }
    return -1;
}

int zmk_combo_set_at(int index, const struct zmk_combo_cfg_data *cfg) {
    int slot = find_slot_for_active_index(index);
    if (slot < 0) {
        return -EINVAL;
    }

    if (cfg->key_position_len < 2 || cfg->key_position_len > MAX_COMBO_KEYS) {
        return -EINVAL;
    }

    if (!cfg->behavior_dev) {
        return -EINVAL;
    }

    /* Update the combo in place */
    memcpy(combos[slot].key_positions, cfg->key_positions,
           sizeof(int32_t) * cfg->key_position_len);
    combos[slot].key_position_len = cfg->key_position_len;
    combos[slot].require_prior_idle_ms = cfg->require_prior_idle_ms;
    combos[slot].timeout_ms = cfg->timeout_ms > 0 ? cfg->timeout_ms : 50;
    combos[slot].layer_mask = cfg->layer_mask;
    combos[slot].behavior.behavior_dev = cfg->behavior_dev;
    combos[slot].behavior.param1 = cfg->param1;
    combos[slot].behavior.param2 = cfg->param2;
    combos[slot].slow_release = cfg->slow_release;

    /* Rebuild lookup table */
    rebuild_combo_lookup();

    LOG_INF("Combo %d (slot %d) updated", index, slot);
    return 0;
}

int zmk_combo_add(const struct zmk_combo_cfg_data *cfg) {
    if (combo_count >= MAX_COMBOS) {
        return -ENOMEM;
    }

    if (cfg->key_position_len < 2 || cfg->key_position_len > MAX_COMBO_KEYS) {
        return -EINVAL;
    }

    if (!cfg->behavior_dev) {
        return -EINVAL;
    }

    /* Find an empty slot */
    int slot = -1;
    for (int i = 0; i < MAX_COMBOS; i++) {
        if (!combos[i].active) {
            slot = i;
            break;
        }
    }
    if (slot < 0) {
        return -ENOMEM;
    }

    memset(&combos[slot], 0, sizeof(struct combo_cfg));
    memcpy(combos[slot].key_positions, cfg->key_positions,
           sizeof(int32_t) * cfg->key_position_len);
    combos[slot].key_position_len = cfg->key_position_len;
    combos[slot].require_prior_idle_ms = cfg->require_prior_idle_ms;
    combos[slot].timeout_ms = cfg->timeout_ms > 0 ? cfg->timeout_ms : 50;
    combos[slot].layer_mask = cfg->layer_mask;
    combos[slot].behavior.behavior_dev = cfg->behavior_dev;
    combos[slot].behavior.param1 = cfg->param1;
    combos[slot].behavior.param2 = cfg->param2;
    combos[slot].slow_release = cfg->slow_release;
    combos[slot].active = true;
    combo_count++;

    /* Rebuild lookup table */
    rebuild_combo_lookup();

    LOG_INF("Combo added at slot %d (total: %d)", slot, combo_count);
    return combo_count - 1; /* return the active index */
}

int zmk_combo_remove(int index) {
    int slot = find_slot_for_active_index(index);
    if (slot < 0) {
        return -EINVAL;
    }

    combos[slot].active = false;
    memset(&combos[slot], 0, sizeof(struct combo_cfg));
    combo_count--;

    /* Rebuild lookup table */
    rebuild_combo_lookup();

    LOG_INF("Combo %d (slot %d) removed (total: %d)", index, slot, combo_count);
    return 0;
}

static int combo_init(void) {
    for (size_t i = 0; i < CONFIG_ZMK_COMBO_MAX_PRESSED_COMBOS; i++) {
        active_combos[i].combo_idx = UINT16_MAX;
    }

    /* Clear all combo slots */
    memset(combos, 0, sizeof(combos));
    combo_count = 0;

    /* Copy DT-defined combos into the dynamic array */
    for (int i = 0; i < (int)DT_COMBO_COUNT && i < MAX_COMBOS; i++) {
        memcpy(&combos[i], &dt_combos[i], sizeof(struct combo_cfg));
        /* Ensure key_positions beyond DT length are zeroed (already done by memset) */
        combos[i].active = true;
        combo_count++;
    }

    k_work_init_delayable(&timeout_task, combo_timeout_handler);
    LOG_WRN("Have %d combos (max %d)!", combo_count, MAX_COMBOS);

    /* Build lookup table */
    rebuild_combo_lookup();

    return 0;
}

SYS_INIT(combo_init, APPLICATION, CONFIG_KERNEL_INIT_PRIORITY_DEFAULT);

#endif
