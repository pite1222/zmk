/*
 * Copyright (c) 2026 The Conductor Contributors
 *
 * SPDX-License-Identifier: MIT
 */

#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/*
 * Conductor per-profile keymap overlay.
 *
 * Each output endpoint (USB / BLE profile N) can be assigned an "overlay" keymap
 * layer. When that endpoint becomes the selected output, the overlay layer is
 * activated on top of the shared keymap (and the previous overlay deactivated),
 * so a device gets its own keymap where it differs (e.g. Windows: thumb = Ctrl,
 * Cmd+Tab -> Alt+Tab) while &trans positions inherit the shared layers.
 *
 * Each map entry is a stable zmk layer ID (as reported by getKeymap and passed
 * to zmk_keymap_layer_activate), NOT an ordered index; value 0 means "no
 * overlay" (use the shared keymap as-is).
 * The whole feature is opt-in; with every entry 0 the behaviour is unchanged.
 */

bool conductor_profile_layers_enabled(void);

/* The overlay layer ID currently applied for the selected endpoint (0 = none).
 * Used by the keymap resolver: the overlay only overrides the default layer,
 * resolving just above it instead of at its own layer index. */
uint8_t conductor_profile_active_overlay(void);

/* Fills map_out (a caller buffer of *len_inout entries) with the per-endpoint
 * overlay-layer map, sets *len_inout to the number of entries written
 * (ZMK_ENDPOINT_COUNT), and reports the active endpoint index and the overlay
 * layer currently applied. Any out-param may be NULL. */
void conductor_profile_get_config(bool *enabled, uint8_t *map_out, size_t *len_inout,
                                  int *active_endpoint, uint8_t *active_layer);

/* Sets the runtime-enable flag and the per-endpoint overlay-layer map (each entry
 * is a layer ID clamped to a valid layer, len clamped to ZMK_ENDPOINT_COUNT),
 * re-applies the overlay for the currently selected endpoint, and persists it. */
void conductor_profile_set_config(bool enabled, const uint8_t *map_in, size_t len);
