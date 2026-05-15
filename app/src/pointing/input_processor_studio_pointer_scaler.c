/*
 * Copyright (c) 2025 The Conductor Contributors
 *
 * SPDX-License-Identifier: MIT
 *
 * Dynamic pointer scaler input processor for ZMK Studio precision mode.
 * Reads numerator/denominator from global variables that can be changed
 * at runtime by the pointing_subsystem RPC handler.
 *
 * Intended use: place on a layer override (e.g. precision_layer) so that
 * X/Y motion is scaled while that layer is active.
 */

#define DT_DRV_COMPAT zmk_input_processor_studio_pointer_scaler

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/logging/log.h>
#include <drivers/input_processor.h>

LOG_MODULE_DECLARE(zmk, CONFIG_ZMK_LOG_LEVEL);

/*
 * External globals set by pointing_subsystem.c.
 * These are the dynamic precision-mode pointer scaling parameters.
 */
extern volatile int32_t studio_pointer_numerator;
extern volatile int32_t studio_pointer_denominator;

struct studio_pointer_scaler_config {
    uint8_t type;
    size_t codes_len;
    uint16_t codes[];
};

struct studio_pointer_scaler_data {
    int16_t remainder_x;
    int16_t remainder_y;
};

static int studio_pointer_scaler_handle_event(const struct device *dev, struct input_event *event,
                                               uint32_t param1, uint32_t param2,
                                               struct zmk_input_processor_state *state) {
    const struct studio_pointer_scaler_config *cfg = dev->config;
    struct studio_pointer_scaler_data *data = dev->data;

    if (event->type != cfg->type) {
        return ZMK_INPUT_PROC_CONTINUE;
    }

    bool matched = false;
    for (int i = 0; i < cfg->codes_len; i++) {
        if (cfg->codes[i] == event->code) {
            matched = true;
            break;
        }
    }

    if (!matched) {
        return ZMK_INPUT_PROC_CONTINUE;
    }

    int32_t num = studio_pointer_numerator;
    int32_t den = studio_pointer_denominator;

    if (den == 0) {
        den = 1;
    }
    if (num == 0) {
        num = 1;
    }

    /* Pick remainder per axis so fractional motion accumulates rather than
     * being silently dropped (otherwise small movements at high ratios
     * disappear entirely). */
    int16_t *remainder = NULL;
    if (event->code == 0x00 /* INPUT_REL_X */) {
        remainder = &data->remainder_x;
    } else if (event->code == 0x01 /* INPUT_REL_Y */) {
        remainder = &data->remainder_y;
    }

    int32_t value_mul = (int32_t)event->value * num;
    if (remainder) {
        value_mul += *remainder;
    }

    int32_t scaled = value_mul / den;
    if (remainder) {
        *remainder = (int16_t)(value_mul - (scaled * den));
    }

    LOG_DBG("studio_pointer_scaler: %d * %d/%d = %d", event->value, num, den, (int)scaled);

    event->value = (int16_t)scaled;

    return ZMK_INPUT_PROC_CONTINUE;
}

static struct zmk_input_processor_driver_api studio_pointer_scaler_driver_api = {
    .handle_event = studio_pointer_scaler_handle_event,
};

#define STUDIO_POINTER_SCALER_INST(n)                                                              \
    static struct studio_pointer_scaler_data studio_pointer_scaler_data_##n = {};                  \
    static const struct studio_pointer_scaler_config studio_pointer_scaler_config_##n = {          \
        .type = DT_INST_PROP_OR(n, type, INPUT_EV_REL),                                            \
        .codes_len = DT_INST_PROP_LEN(n, codes),                                                   \
        .codes = DT_INST_PROP(n, codes),                                                           \
    };                                                                                             \
    DEVICE_DT_INST_DEFINE(n, NULL, NULL, &studio_pointer_scaler_data_##n,                          \
                          &studio_pointer_scaler_config_##n, POST_KERNEL,                          \
                          CONFIG_KERNEL_INIT_PRIORITY_DEFAULT, &studio_pointer_scaler_driver_api);

DT_INST_FOREACH_STATUS_OKAY(STUDIO_POINTER_SCALER_INST)
