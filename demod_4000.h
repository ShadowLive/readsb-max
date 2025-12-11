// demod_4000.h: 4.0 MHz Mode S demodulator header.
//
// Copyright (c) 2024, readsb contributors
// SPDX-License-Identifier: GPL-3.0-or-later

#ifndef DEMOD_4000_H
#define DEMOD_4000_H

// Main demodulation function for 4.0 MHz (interpolated from 3.2 MHz)
void demodulate4000(struct mag_buf *mag);

#endif // DEMOD_4000_H
