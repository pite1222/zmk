/*
 * Copyright (c) 2024 The ZMK Contributors
 *
 * SPDX-License-Identifier: MIT
 */

#pragma once

/**
 * Notify the keepalive module that pointing activity occurred.
 * Resets the idle timer so keepalive reports are deferred.
 */
void zmk_pointing_keepalive_notify_activity(void);
