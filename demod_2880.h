// demod_2880.h: 2.88MHz Mode S demodulator prototypes.
//
// Copyright (c) 2024, readsb contributors
// SPDX-License-Identifier: GPL-3.0-or-later

#ifndef DEMOD_2880_H
#define DEMOD_2880_H

struct mag_buf;

// Main demodulation function for 2.88 MHz sample rate
void demodulate2880(struct mag_buf *mag);

// Mode A/C demodulation for 2.88 MHz (stub)
void demodulate2880AC(struct mag_buf *mag);

#endif // DEMOD_2880_H
