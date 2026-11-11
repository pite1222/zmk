/*
 * Copyright (c) 2025 The Conductor Contributors
 *
 * SPDX-License-Identifier: MIT
 *
 * Dynamic pointer-acceleration input processor for ZMK Studio.
 *
 * Applies a speed-dependent gain to relative X/Y cursor motion. The gain ramps
 * linearly from 1.0x up to (max_milli/1000)x as the per-poll movement magnitude
 * crosses `threshold` (counts/poll), reaching the maximum after `range`
 * additional counts/poll. Below `slow_range` counts/poll (capped at
 * `threshold`) the gain is instead held flat at (min_milli/1000)x and then
 * hands over continuously to the fast curve (see compute_gain_q8) — below 1.0
 * decelerates slow movements for fine pointing, above 1.0 boosts them for a
 * lighter feel (min_milli == 1000 disables the low-speed zone).
 * Parameters are read from globals set at runtime by the pointing_subsystem
 * RPC handler; enabled == 0 -> passthrough (linear).
 *
 * The trackball driver reports up to one X and one Y event per poll, at a
 * near-constant sample rate while moving, so the per-poll magnitude is a usable
 * velocity proxy without an explicit dt. The driver omits an axis whose delta
 * is zero, so a poll may consist of X only, Y only, or X then Y; whichever
 * event is last carries sync=true. To keep the same gain on both axes of a
 * frame (and avoid direction distortion) the gain is derived from the previous
 * completed frame's magnitude — accumulated per event and latched at the sync
 * boundary — and applied to the current frame's X and Y (a single-poll, ~8ms
 * lag).
 *
 * While the configured `bypass-layer` (the precision layer) is active the
 * processor passes events through unchanged so it does not fight precision mode.
 *
 * Usage in devicetree:
 *   studio_accel: studio_accel {
 *       compatible = "zmk,input-processor-studio-accel";
 *       #input-processor-cells = <0>;
 *       codes = <INPUT_REL_X INPUT_REL_Y>;
 *       bypass-layer = <PRECISION_LAYER>;
 *   };
 */

#define DT_DRV_COMPAT zmk_input_processor_studio_accel

#include <stdlib.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/logging/log.h>
#include <drivers/input_processor.h>

#include <zmk/keymap.h>

LOG_MODULE_DECLARE(zmk, CONFIG_ZMK_LOG_LEVEL);

/*
 * External globals set by pointing_subsystem.c.
 * These are the dynamic acceleration parameters.
 */
extern volatile int32_t studio_accel_enabled;
extern volatile int32_t studio_accel_max_milli;  /* max gain x1000 (2000 = 2.0x) */
extern volatile int32_t studio_accel_threshold;  /* counts/poll where accel starts */
extern volatile int32_t studio_accel_range;      /* counts/poll span to reach max gain */
extern volatile int32_t studio_accel_min_milli;  /* low-speed gain x1000 (1000 = off) */
extern volatile int32_t studio_accel_slow_range; /* counts/poll span of min->1.0x ramp */

/* Q8 fixed point: 256 == 1.0x */
#define ACCEL_GAIN_UNITY 256

struct studio_accel_config {
    uint8_t type;
    int16_t bypass_layer; /* layer that disables accel while active; -1 = none */
    size_t codes_len;
    uint16_t codes[];
};

struct studio_accel_data {
    int16_t remainder_x;
    int16_t remainder_y;
    int32_t frame_mag;      /* L1 magnitude accumulated over the current frame */
    uint16_t gain_q8;       /* gain applied to the CURRENT frame (one-poll lag) */
};

/* Fast-zone gain: 1.0x up to `threshold`, then linear up to max gain over
 * `range` additional counts. */
static int32_t fast_gain_q8(int32_t magnitude, int32_t threshold, int32_t range,
                            int32_t max_milli) {
    if (magnitude <= threshold) {
        return ACCEL_GAIN_UNITY;
    }
    int32_t over = magnitude - threshold;
    if (over > range) {
        over = range;
    }
    int32_t max_q8 = (max_milli * ACCEL_GAIN_UNITY) / 1000;
    return ACCEL_GAIN_UNITY + ((max_q8 - ACCEL_GAIN_UNITY) * over) / range;
}

/* Speed -> gain curve. The gain is continuous in magnitude for every setting
 * and output (magnitude * gain) never decreases as magnitude grows, so a
 * slightly faster flick can never move the cursor less.
 *
 * Low-speed zone: flat min gain up to slow_end = min(slow_range, threshold).
 * Above it:
 *  - boost (min > 1.0): the extra (min - 1) * slow_end counts gained at the
 *    end of the slow zone are kept as a constant bonus (output slope 1.0x), so
 *    the gain decays as 1 + (min - 1) * slow_end / magnitude until the fast
 *    curve overtakes it: gain = max(fast, decaying boost).
 *  - decel (min < 1.0): linear return from min to 1.0x by threshold (or over
 *    the fast ramp when slow_end == threshold), multiplied with the fast gain;
 *    both factors are non-decreasing, so output is too. */
static uint16_t compute_gain_q8(int32_t magnitude) {
    if (studio_accel_enabled == 0) {
        return ACCEL_GAIN_UNITY;
    }

    int32_t threshold = studio_accel_threshold;
    int32_t range = studio_accel_range;
    int32_t max_milli = studio_accel_max_milli;

    if (range <= 0) {
        range = 1;
    }
    if (max_milli < 1000) {
        max_milli = 1000; /* never attenuate below 1.0x in the fast zone */
    }

    int32_t fast = fast_gain_q8(magnitude, threshold, range, max_milli);

    int32_t min_milli = studio_accel_min_milli;
    int32_t slow_end = studio_accel_slow_range;
    if (slow_end > threshold) {
        slow_end = threshold;
    }
    if (min_milli == 1000 || min_milli <= 0 || slow_end <= 0) {
        return (uint16_t)fast;
    }

    int32_t min_q8 = (min_milli * ACCEL_GAIN_UNITY) / 1000;
    if (magnitude <= slow_end) {
        return (uint16_t)min_q8;
    }

    if (min_q8 > ACCEL_GAIN_UNITY) {
        int32_t boost = ACCEL_GAIN_UNITY + ((min_q8 - ACCEL_GAIN_UNITY) * slow_end) / magnitude;
        return (uint16_t)(boost > fast ? boost : fast);
    }

    int32_t ramp_end = slow_end < threshold ? threshold : threshold + range;
    int32_t ramp = ACCEL_GAIN_UNITY;
    if (magnitude < ramp_end) {
        ramp = min_q8 + ((ACCEL_GAIN_UNITY - min_q8) * (magnitude - slow_end)) /
                            (ramp_end - slow_end);
    }
    return (uint16_t)((fast * ramp) / ACCEL_GAIN_UNITY);
}

static int16_t scale_with_remainder(int16_t value, uint16_t gain_q8, int16_t *remainder) {
    int32_t v = (int32_t)value * (int32_t)gain_q8 + (int32_t)(*remainder);
    int32_t scaled = v / ACCEL_GAIN_UNITY; /* truncates toward zero */
    *remainder = (int16_t)(v - scaled * ACCEL_GAIN_UNITY);
    return (int16_t)scaled;
}

static int studio_accel_handle_event(const struct device *dev, struct input_event *event,
                                     uint32_t param1, uint32_t param2,
                                     struct zmk_input_processor_state *state) {
    const struct studio_accel_config *cfg = dev->config;
    struct studio_accel_data *data = dev->data;

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

    /* Bypass entirely while the precision layer is active. Also drop the gain
     * back to 1.0x so motion resumes unaccelerated (and re-ramps) when the
     * precision layer is left, instead of inheriting a stale pre-precision
     * gain for one frame. */
    if (cfg->bypass_layer >= 0 && zmk_keymap_layer_active((uint8_t)cfg->bypass_layer)) {
        data->frame_mag = 0;
        data->remainder_x = 0;
        data->remainder_y = 0;
        data->gain_q8 = ACCEL_GAIN_UNITY;
        return ZMK_INPUT_PROC_CONTINUE;
    }

    if (data->gain_q8 == 0) {
        data->gain_q8 = ACCEL_GAIN_UNITY; /* first event after boot */
    }

    int16_t orig = event->value;
    if (event->code == 0x00 /* INPUT_REL_X */) {
        event->value = scale_with_remainder(orig, data->gain_q8, &data->remainder_x);
    } else if (event->code == 0x01 /* INPUT_REL_Y */) {
        event->value = scale_with_remainder(orig, data->gain_q8, &data->remainder_y);
    } else {
        return ZMK_INPUT_PROC_CONTINUE;
    }
    data->frame_mag += abs(orig);

    /* The last event of a poll carries sync regardless of axis (the driver
     * omits zero-delta axes, so X-only and Y-only frames are normal). Latch
     * the gain for the NEXT frame from this frame's combined magnitude at
     * that boundary (L1 norm is cheap and adequate). */
    if (event->sync) {
        data->gain_q8 = compute_gain_q8(data->frame_mag);
        LOG_DBG("studio_accel: mag=%d -> next gain_q8=%u", data->frame_mag, data->gain_q8);
        data->frame_mag = 0;
    }

    return ZMK_INPUT_PROC_CONTINUE;
}

static struct zmk_input_processor_driver_api studio_accel_driver_api = {
    .handle_event = studio_accel_handle_event,
};

#define STUDIO_ACCEL_INST(n)                                                                       \
    static struct studio_accel_data studio_accel_data_##n = {                                       \
        .gain_q8 = ACCEL_GAIN_UNITY,                                                               \
    };                                                                                             \
    static const struct studio_accel_config studio_accel_config_##n = {                            \
        .type = DT_INST_PROP_OR(n, type, INPUT_EV_REL),                                            \
        .bypass_layer = DT_INST_PROP_OR(n, bypass_layer, -1),                                      \
        .codes_len = DT_INST_PROP_LEN(n, codes),                                                   \
        .codes = DT_INST_PROP(n, codes),                                                           \
    };                                                                                             \
    DEVICE_DT_INST_DEFINE(n, NULL, NULL, &studio_accel_data_##n, &studio_accel_config_##n,          \
                          POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEFAULT,                        \
                          &studio_accel_driver_api);

DT_INST_FOREACH_STATUS_OKAY(STUDIO_ACCEL_INST)
