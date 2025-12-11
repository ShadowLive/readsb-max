// decimate.c: Resample magnitude samples between different rates
//
// This module provides resampling of magnitude data after IQ conversion,
// supporting both decimation (downsampling) and interpolation (upsampling).
//
// Supported decimation ratios:
//   2.88 MHz -> 2.4 MHz: 6 -> 5 samples (ratio 1.2)
//   3.2 MHz -> 2.4 MHz: 4 -> 3 samples (ratio 1.333)
//
// Supported interpolation ratios:
//   2.88 MHz -> 3.0 MHz: 24 -> 25 samples (ratio 0.96, gives exactly 3 samples/bit)
//   3.2 MHz -> 4.0 MHz: 4 -> 5 samples (ratio 0.8, gives exactly 4 samples/bit)
//
// Copyright (c) 2024, readsb contributors
// SPDX-License-Identifier: GPL-3.0-or-later

#include "readsb.h"

// Resampling modes
#define RESAMPLE_DECIMATE_2880_TO_2400  0   // 6:5 decimation
#define RESAMPLE_DECIMATE_3200_TO_2400  1   // 4:3 decimation
#define RESAMPLE_INTERPOLATE_2880_TO_3000  2   // 24:25 interpolation
#define RESAMPLE_INTERPOLATE_3200_TO_4000  3   // 4:5 interpolation

// Resampling state
struct decimate_state {
    int mode;           // Resampling mode
    int phase;          // Current phase within resampling cycle
    uint32_t residual[32];  // Residual samples from previous buffer
    int residual_count;
};

static struct decimate_state *dec_state = NULL;

// Initialize resampling
// mode: 0 = 2.88->2.4 decimate, 1 = 3.2->2.4 decimate
//       2 = 2.88->3.0 interpolate, 3 = 3.2->4.0 interpolate
int decimate_init(int mode) {
    if (dec_state) {
        free(dec_state);
    }

    dec_state = calloc(1, sizeof(struct decimate_state));
    if (!dec_state) {
        fprintf(stderr, "resample: can't allocate state\n");
        return -1;
    }

    dec_state->mode = mode;
    dec_state->phase = 0;
    dec_state->residual_count = 0;

    switch (mode) {
        case RESAMPLE_DECIMATE_2880_TO_2400:
            fprintf(stderr, "resample: initialized 2.88 MHz -> 2.4 MHz decimation (6:5)\n");
            break;
        case RESAMPLE_DECIMATE_3200_TO_2400:
            fprintf(stderr, "resample: initialized 3.2 MHz -> 2.4 MHz decimation (4:3)\n");
            break;
        case RESAMPLE_INTERPOLATE_2880_TO_3000:
            fprintf(stderr, "resample: initialized 2.88 MHz -> 3.0 MHz interpolation (24:25)\n");
            break;
        case RESAMPLE_INTERPOLATE_3200_TO_4000:
            fprintf(stderr, "resample: initialized 3.2 MHz -> 4.0 MHz interpolation (4:5)\n");
            break;
    }

    return 0;
}

void decimate_cleanup(void) {
    if (dec_state) {
        free(dec_state);
        dec_state = NULL;
    }
}

// Decimate 2.88 MHz to 2.4 MHz (6 samples -> 5 samples)
// Uses linear interpolation to produce 5 output samples from 6 input samples
// This preserves the timing information better than simple averaging
static unsigned decimate_2880_to_2400(uint16_t *in, uint16_t *out, unsigned in_samples) {
    unsigned out_samples = 0;
    unsigned i = 0;

    // Process in groups of 6 input samples -> 5 output samples
    while (i + 6 <= in_samples) {
        // Output positions at 0, 1.2, 2.4, 3.6, 4.8 input sample positions
        // Position 0: input[0]
        out[out_samples++] = in[i + 0];

        // Position 1.2: 0.8 * input[1] + 0.2 * input[2]
        out[out_samples++] = (uint16_t)((4 * (uint32_t)in[i + 1] + 1 * (uint32_t)in[i + 2]) / 5);

        // Position 2.4: 0.6 * input[2] + 0.4 * input[3]
        out[out_samples++] = (uint16_t)((3 * (uint32_t)in[i + 2] + 2 * (uint32_t)in[i + 3]) / 5);

        // Position 3.6: 0.4 * input[3] + 0.6 * input[4]
        out[out_samples++] = (uint16_t)((2 * (uint32_t)in[i + 3] + 3 * (uint32_t)in[i + 4]) / 5);

        // Position 4.8: 0.2 * input[4] + 0.8 * input[5]
        out[out_samples++] = (uint16_t)((1 * (uint32_t)in[i + 4] + 4 * (uint32_t)in[i + 5]) / 5);

        i += 6;
    }

    return out_samples;
}

// Decimate 3.2 MHz to 2.4 MHz (4 samples -> 3 samples)
// Uses linear interpolation
static unsigned decimate_3200_to_2400(uint16_t *in, uint16_t *out, unsigned in_samples) {
    unsigned out_samples = 0;
    unsigned i = 0;

    // Process in groups of 4 input samples -> 3 output samples
    while (i + 4 <= in_samples) {
        // Output positions at 0, 1.333, 2.667 input sample positions
        // Position 0: input[0]
        out[out_samples++] = in[i + 0];

        // Position 1.333: 0.667 * input[1] + 0.333 * input[2]
        out[out_samples++] = (uint16_t)((2 * (uint32_t)in[i + 1] + 1 * (uint32_t)in[i + 2]) / 3);

        // Position 2.667: 0.333 * input[2] + 0.667 * input[3]
        out[out_samples++] = (uint16_t)((1 * (uint32_t)in[i + 2] + 2 * (uint32_t)in[i + 3]) / 3);

        i += 4;
    }

    return out_samples;
}

// Interpolate 2.88 MHz to 3.0 MHz (24 samples -> 25 samples)
// This gives exactly 3.0 samples per bit - a clean integer!
// Uses linear interpolation
static unsigned interpolate_2880_to_3000(uint16_t *in, uint16_t *out, unsigned in_samples) {
    unsigned out_samples = 0;
    unsigned i = 0;

    // Ratio: 3.0/2.88 = 25/24, so every 24 input samples -> 25 output samples
    // Output positions: 0, 0.96, 1.92, 2.88, 3.84, 4.80, 5.76, 6.72, 7.68, 8.64, 9.60,
    //                   10.56, 11.52, 12.48, 13.44, 14.40, 15.36, 16.32, 17.28, 18.24,
    //                   19.20, 20.16, 21.12, 22.08, 23.04
    // Each output sample j is at input position j * 0.96

    while (i + 24 <= in_samples) {
        // Generate 25 output samples from 24 input samples using linear interpolation
        for (int j = 0; j < 25; j++) {
            // Output position in input sample space
            // pos = j * 24 / 25 = j * 0.96
            int pos_int = (j * 24) / 25;
            int frac_25 = (j * 24) % 25;  // Fractional part in 25ths

            if (frac_25 == 0 || pos_int + 1 >= 24) {
                // Exact sample position or at edge
                out[out_samples++] = in[i + pos_int];
            } else {
                // Linear interpolation: (25-frac)/25 * in[pos] + frac/25 * in[pos+1]
                uint32_t val = (25 - frac_25) * (uint32_t)in[i + pos_int] +
                               frac_25 * (uint32_t)in[i + pos_int + 1];
                out[out_samples++] = (uint16_t)(val / 25);
            }
        }
        i += 24;
    }

    return out_samples;
}

// Interpolate 3.2 MHz to 4.0 MHz (4 samples -> 5 samples)
// This gives exactly 4.0 samples per bit - a clean integer!
// Uses linear interpolation
static unsigned interpolate_3200_to_4000(uint16_t *in, uint16_t *out, unsigned in_samples) {
    unsigned out_samples = 0;
    unsigned i = 0;

    // Ratio: 4.0/3.2 = 5/4, so every 4 input samples -> 5 output samples
    // Output positions: 0, 0.8, 1.6, 2.4, 3.2
    // Each output sample j is at input position j * 0.8

    while (i + 4 <= in_samples) {
        // Position 0: exact sample
        out[out_samples++] = in[i + 0];

        // Position 0.8: 0.2 * in[0] + 0.8 * in[1]
        out[out_samples++] = (uint16_t)((1 * (uint32_t)in[i + 0] + 4 * (uint32_t)in[i + 1]) / 5);

        // Position 1.6: 0.4 * in[1] + 0.6 * in[2]
        out[out_samples++] = (uint16_t)((2 * (uint32_t)in[i + 1] + 3 * (uint32_t)in[i + 2]) / 5);

        // Position 2.4: 0.6 * in[2] + 0.4 * in[3]
        out[out_samples++] = (uint16_t)((3 * (uint32_t)in[i + 2] + 2 * (uint32_t)in[i + 3]) / 5);

        // Position 3.2: 0.8 * in[3] + 0.2 * in[4] (but we only have 4 samples, use in[3])
        // For boundary, we use last sample
        out[out_samples++] = (uint16_t)((4 * (uint32_t)in[i + 3] + 1 * (uint32_t)in[i + 3]) / 5);

        i += 4;
    }

    return out_samples;
}

// Main resampling function
// Returns number of output samples
unsigned decimate_buffer(uint16_t *in, uint16_t *out, unsigned in_samples) {
    if (!dec_state) {
        // No resampling, just copy
        memcpy(out, in, in_samples * sizeof(uint16_t));
        return in_samples;
    }

    switch (dec_state->mode) {
        case RESAMPLE_DECIMATE_2880_TO_2400:
            return decimate_2880_to_2400(in, out, in_samples);
        case RESAMPLE_DECIMATE_3200_TO_2400:
            return decimate_3200_to_2400(in, out, in_samples);
        case RESAMPLE_INTERPOLATE_2880_TO_3000:
            return interpolate_2880_to_3000(in, out, in_samples);
        case RESAMPLE_INTERPOLATE_3200_TO_4000:
            return interpolate_3200_to_4000(in, out, in_samples);
        default:
            memcpy(out, in, in_samples * sizeof(uint16_t));
            return in_samples;
    }
}

// Get the output sample rate based on current mode
// Returns 0 if no resampling active
unsigned decimate_get_output_rate(unsigned input_rate) {
    if (!dec_state) return input_rate;

    switch (dec_state->mode) {
        case RESAMPLE_DECIMATE_2880_TO_2400:
            return 2400000;
        case RESAMPLE_DECIMATE_3200_TO_2400:
            return 2400000;
        case RESAMPLE_INTERPOLATE_2880_TO_3000:
            return 3000000;
        case RESAMPLE_INTERPOLATE_3200_TO_4000:
            return 4000000;
        default:
            return input_rate;
    }
}

// Get resample mode
int decimate_get_mode(void) {
    return dec_state ? dec_state->mode : -1;
}

// Check if decimation is active
int decimate_active(void) {
    return dec_state != NULL;
}
