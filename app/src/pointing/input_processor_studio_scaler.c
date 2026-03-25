/*
 * Copyright (c) 2025 The Conductor Contributors
 *
 * SPDX-License-Identifier: MIT
 *
 * Dynamic scaler input processor for ZMK Studio.
 * Unlike the static zip_scroll_scaler, this processor reads its
 * numerator/denominator from a global variable that can be changed
 * at runtime by the pointing_subsystem RPC handler.
 *
 * Usage in devicetree:
 *   &studio_scroll_scaler   (no cells needed, #input-processor-cells = <0>)
 */

#define DT_DRV_COMPAT zmk_input_processor_studio_scaler

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/logging/log.h>
#include <drivers/input_processor.h>

LOG_MODULE_DECLARE(zmk, CONFIG_ZMK_LOG_LEVEL);

/*
 * External globals set by pointing_subsystem.c
 * These are the dynamic scroll scaling parameters.
 */
extern volatile int32_t studio_scroll_numerator;
extern volatile int32_t studio_scroll_denominator;
extern volatile bool studio_scroll_inverted;

struct studio_scaler_config {
    uint8_t type;
    size_t codes_len;
    uint16_t codes[];
};

struct studio_scaler_data {
    int16_t remainder_wheel;
    int16_t remainder_hwheel;
};

static int studio_scaler_handle_event(const struct device *dev, struct input_event *event,
                                       uint32_t param1, uint32_t param2,
                                       struct zmk_input_processor_state *state) {
    const struct studio_scaler_config *cfg = dev->config;
    struct studio_scaler_data *data = dev->data;

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

    /* Read the dynamic scaling parameters */
    int32_t num = studio_scroll_numerator;
    int32_t den = studio_scroll_denominator;

    if (den == 0) {
        den = 1;
    }
    if (num == 0) {
        num = 1;
    }

    /* Apply scroll inversion */
    if (studio_scroll_inverted) {
        num = -num;
    }

    /* Select the appropriate remainder based on event code */
    int16_t *remainder = NULL;
    if (event->code == 0x08 /* INPUT_REL_WHEEL */) {
        remainder = &data->remainder_wheel;
    } else if (event->code == 0x06 /* INPUT_REL_HWHEEL */) {
        remainder = &data->remainder_hwheel;
    }

    /* Scale the value with remainder tracking */
    int32_t value_mul = (int32_t)event->value * num;
    if (remainder) {
        value_mul += *remainder;
    }

    int32_t scaled = value_mul / den;
    if (remainder) {
        *remainder = (int16_t)(value_mul - (scaled * den));
    }

    LOG_DBG("studio_scaler: %d * %d/%d = %d (rem=%d)", event->value, num, den, (int)scaled,
            remainder ? *remainder : 0);

    event->value = (int16_t)scaled;

    return ZMK_INPUT_PROC_CONTINUE;
}

static struct zmk_input_processor_driver_api studio_scaler_driver_api = {
    .handle_event = studio_scaler_handle_event,
};

#define STUDIO_SCALER_INST(n)                                                                      \
    static struct studio_scaler_data studio_scaler_data_##n = {};                                   \
    static const struct studio_scaler_config studio_scaler_config_##n = {                           \
        .type = DT_INST_PROP_OR(n, type, INPUT_EV_REL),                                            \
        .codes_len = DT_INST_PROP_LEN(n, codes),                                                   \
        .codes = DT_INST_PROP(n, codes),                                                           \
    };                                                                                             \
    DEVICE_DT_INST_DEFINE(n, NULL, NULL, &studio_scaler_data_##n,                                  \
                          &studio_scaler_config_##n, POST_KERNEL,                                  \
                          CONFIG_KERNEL_INIT_PRIORITY_DEFAULT, &studio_scaler_driver_api);

DT_INST_FOREACH_STATUS_OKAY(STUDIO_SCALER_INST)
