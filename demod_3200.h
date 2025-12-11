// demod_3200.h: 3.2MHz Mode S demodulator header.
//
// Copyright (c) 2024, readsb contributors
// SPDX-License-Identifier: GPL-3.0-or-later

#ifndef DEMOD_3200_H
#define DEMOD_3200_H

struct mag_buf;

void demodulate3200(struct mag_buf *mag);
void demodulate3200AC(struct mag_buf *mag);

#endif // DEMOD_3200_H
