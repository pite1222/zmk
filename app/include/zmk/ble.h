/*
 * Copyright (c) 2020 The ZMK Contributors
 *
 * SPDX-License-Identifier: MIT
 */

#pragma once

#include <zmk/keys.h>
#include <zmk/ble/profile.h>

#define ZMK_BLE_IS_CENTRAL                                                                         \
    (IS_ENABLED(CONFIG_ZMK_SPLIT_BLE) && IS_ENABLED(CONFIG_ZMK_SPLIT_ROLE_CENTRAL))

#if ZMK_BLE_IS_CENTRAL
#define ZMK_BLE_PROFILE_COUNT (CONFIG_BT_MAX_PAIRED - CONFIG_ZMK_SPLIT_BLE_CENTRAL_PERIPHERALS)
#define ZMK_SPLIT_BLE_PERIPHERAL_COUNT CONFIG_ZMK_SPLIT_BLE_CENTRAL_PERIPHERALS
#else
#define ZMK_BLE_PROFILE_COUNT CONFIG_BT_MAX_PAIRED
#endif

void zmk_ble_clear_bonds(void);
int zmk_ble_prof_next(void);
int zmk_ble_prof_prev(void);
int zmk_ble_prof_select(uint8_t index);
void zmk_ble_clear_all_bonds(void);
int zmk_ble_prof_disconnect(uint8_t index);

int zmk_ble_active_profile_index(void);
int zmk_ble_profile_index(const bt_addr_le_t *addr);
bt_addr_le_t *zmk_ble_profile_address(uint8_t index);

bt_addr_le_t *zmk_ble_active_profile_addr(void);
struct bt_conn *zmk_ble_active_profile_conn(void);

bool zmk_ble_profile_is_connected(uint8_t index);
bool zmk_ble_profile_is_open(uint8_t index);

bool zmk_ble_active_profile_is_open(void);
bool zmk_ble_active_profile_is_connected(void);
char *zmk_ble_active_profile_name(void);
char *zmk_ble_profile_name(uint8_t index);
int zmk_ble_set_profile_name(uint8_t index, const char *name);

int zmk_ble_unpair_all(void);

int zmk_ble_set_device_name(char *name);

void zmk_ble_request_conn_param_rebalance(void);

#if IS_ENABLED(CONFIG_ZMK_STUDIO_TRANSPORT_BLE)
int zmk_studio_ble_profile(void);
#else
static inline int zmk_studio_ble_profile(void) { return -1; }
#endif

#if IS_ENABLED(CONFIG_ZMK_SPLIT_ROLE_CENTRAL)
int zmk_ble_put_peripheral_addr(const bt_addr_le_t *addr);
#endif /* IS_ENABLED(CONFIG_ZMK_SPLIT_ROLE_CENTRAL) */
