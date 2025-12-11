// demod_3000.h: 3.0 MHz Mode S demodulator header.
//
// Copyright (c) 2024, readsb contributors
// SPDX-License-Identifier: GPL-3.0-or-later

#ifndef DEMOD_3000_H
#define DEMOD_3000_H

// Main demodulation function for 3.0 MHz (interpolated from 2.88 MHz)
void demodulate3000(struct mag_buf *mag);

#endif // DEMOD_3000_H
