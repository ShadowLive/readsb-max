// decimate.h: Resample magnitude samples between different rates
//
// Supports both decimation (downsampling) and interpolation (upsampling).
//
// Decimation modes:
//   2.88 MHz -> 2.4 MHz: 6:5 ratio
//   3.2 MHz -> 2.4 MHz: 4:3 ratio
//
// Interpolation modes:
//   2.88 MHz -> 3.0 MHz: 24:25 ratio (gives exactly 3 samples/bit)
//   3.2 MHz -> 4.0 MHz: 4:5 ratio (gives exactly 4 samples/bit)
//
// Copyright (c) 2024, readsb contributors
// SPDX-License-Identifier: GPL-3.0-or-later

#ifndef DECIMATE_H
#define DECIMATE_H

// Resampling modes
#define RESAMPLE_DECIMATE_2880_TO_2400     0   // 6:5 decimation
#define RESAMPLE_DECIMATE_3200_TO_2400     1   // 4:3 decimation
#define RESAMPLE_INTERPOLATE_2880_TO_3000  2   // 24:25 interpolation
#define RESAMPLE_INTERPOLATE_3200_TO_4000  3   // 4:5 interpolation

// Initialize resampling
// mode: one of the RESAMPLE_* constants above
int decimate_init(int mode);

// Cleanup resampling state
void decimate_cleanup(void);

// Resample a buffer of magnitude samples
// Returns number of output samples
unsigned decimate_buffer(uint16_t *in, uint16_t *out, unsigned in_samples);

// Get the output sample rate based on current mode
unsigned decimate_get_output_rate(unsigned input_rate);

// Get current resample mode (-1 if not active)
int decimate_get_mode(void);

// Check if resampling is active
int decimate_active(void);

#endif // DECIMATE_H
