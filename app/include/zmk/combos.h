/*
 * Copyright (c) 2020 The ZMK Contributors
 *
 * SPDX-License-Identifier: MIT
 */

#pragma once

#include <zephyr/devicetree.h>

#define ZMK_COMBOS_UTIL_ONE(n) +1

/* Number of combos defined in devicetree */
#define ZMK_COMBOS_DT_COUNT                                                                        \
    COND_CODE_1(DT_HAS_COMPAT_STATUS_OKAY(zmk_combos),                                             \
                (0 DT_FOREACH_CHILD_STATUS_OKAY(DT_INST(0, zmk_combos), ZMK_COMBOS_UTIL_ONE)),     \
                (0))

/*
 * Total combo capacity: max of DT-defined combos and the dynamic max.
 * This is used for virtual key position allocation.
 */
#ifdef CONFIG_ZMK_COMBO_MAX_DYNAMIC_COMBOS
#define ZMK_COMBOS_LEN MAX(ZMK_COMBOS_DT_COUNT, CONFIG_ZMK_COMBO_MAX_DYNAMIC_COMBOS)
#else
#define ZMK_COMBOS_LEN ZMK_COMBOS_DT_COUNT
#endif

/*
 * Public API for combo_subsystem.c (Studio RPC).
 * These are implemented in combo.c.
 */
#define ZMK_COMBO_MAX_KEYS 8

struct zmk_combo_cfg_data {
    int32_t key_positions[ZMK_COMBO_MAX_KEYS];
    int16_t key_position_len;
    int16_t require_prior_idle_ms;
    int32_t timeout_ms;
    uint32_t layer_mask;
    const char *behavior_dev;
    uint32_t param1;
    uint32_t param2;
    bool slow_release;
};

int zmk_combo_get_count(void);
int zmk_combo_get_at(int index, struct zmk_combo_cfg_data *out);
int zmk_combo_set_at(int index, const struct zmk_combo_cfg_data *cfg);
int zmk_combo_add(const struct zmk_combo_cfg_data *cfg);
int zmk_combo_remove(int index);
