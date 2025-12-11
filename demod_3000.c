// demod_3000.c: 3.0MHz Mode S demodulator.
//
// This demodulator is used after interpolating 2.88 MHz -> 3.0 MHz.
// At 3.0 MHz: exactly 3 samples per bit = 1.5 samples per symbol.
// This clean integer ratio eliminates the phase quantization error
// that plagues 2.88 MHz.
//
// Copyright (c) 2024, readsb contributors
// SPDX-License-Identifier: GPL-3.0-or-later

#include "readsb.h"

// 3.0MHz sampling rate version
//
// At 3.0 MHz we have exactly 3 samples per bit.
// Each symbol is 500ns wide (1.5 samples), each sample is 333.3ns wide.
//
// At 3.0 MHz:
//   - 1 symbol = 1.5 samples
//   - 1 bit = 3 samples
//   - Preamble (8 µs) = 24 samples
//   - 112-bit message = 336 samples
//
// Since 3 samples/bit is an exact integer, phase tracking is simple.
// We use 3 phases (0-2) representing 1/3 sample offsets.

// Valid DF bitsets for message length detection
static const uint32_t valid_df_short_bitset_3000 = (1 << 0) | (1 << 4) | (1 << 5) | (1 << 11);
static const uint32_t valid_df_long_bitset_3000 = (1 << 16) | (1 << 17) | (1 << 18) | (1 << 20) | (1 << 21);

// Slice functions for 3.0 MHz
// At 3.0 MHz, one bit spans exactly 3 samples (1.5 samples per symbol)
// Manchester encoding: bit=1 means high-then-low, bit=0 means low-then-high
//
// Phase 0: bit boundary at sample start (offset 0.0)
// High symbol: 0.0-1.5, Low symbol: 1.5-3.0
// Sample 0: fully high (1.0 unit), Sample 1: 0.5 high + 0.5 low, Sample 2: fully low
static inline __attribute__((always_inline)) int slice_phase0_3000(uint16_t *m) {
    // Coefficients: +10 for full high, +5-5=0 for mixed, -10 for full low
    return 10 * m[0] - 10 * m[2];
}

// Phase 1: bit boundary 0.33 samples into sample 0 (offset 0.33)
// High: 0.33-1.83, Low: 1.83-3.33
static inline __attribute__((always_inline)) int slice_phase1_3000(uint16_t *m) {
    // Sample 0: 0.67 high, Sample 1: 0.83 high + 0.17 low, Sample 2: 0.83 low + 0.17 next
    return 7 * m[0] + 5 * m[1] - 10 * m[2] - 2 * m[3];
}

// Phase 2: bit boundary 0.67 samples into sample 0 (offset 0.67)
// High: 0.67-2.17, Low: 2.17-3.67
static inline __attribute__((always_inline)) int slice_phase2_3000(uint16_t *m) {
    // Sample 0: 0.33 high, Sample 1: fully high (1.0), Sample 2: 0.17 high + 0.83 low
    return 3 * m[0] + 10 * m[1] - 8 * m[2] - 5 * m[3];
}

// Preamble structure at 3.0 MHz
// Preamble is 8 µs = 4 pulses at 0, 1, 3.5, 4.5 µs
// At 3.0 MHz (333.3ns/sample):
//   Pulse 0: sample ~0-1 (0 µs)
//   Pulse 1: sample ~3-4 (1 µs)
//   Pulse 2: sample ~10-12 (3.5 µs)
//   Pulse 3: sample ~13-15 (4.5 µs)
//   Data starts: sample ~24 (8 µs)

#define PREAMBLE_SAMPLES_3000 24

// Slice a byte at 3.0 MHz
// At 3.0 MHz, each bit = 3 samples, so 8 bits = 24 samples exactly!
// With 3 phases, phase increment per bit = 0 (3 mod 3 = 0)
// This means starting phase is maintained throughout the byte!
static inline __attribute__((always_inline)) uint8_t slice_byte_3000(uint16_t **pPtr, int *pPhase) {
    uint8_t theByte = 0;
    uint16_t *m = *pPtr;
    int phase = *pPhase;

    // Since 3 samples/bit and 3 phases, phase stays constant within a byte
    // Bit positions: 0, 3, 6, 9, 12, 15, 18, 21 (all at same phase!)
    switch (phase) {
        case 0:
            theByte =
                (slice_phase0_3000(m+0) > 0 ? 0x80 : 0) |
                (slice_phase0_3000(m+3) > 0 ? 0x40 : 0) |
                (slice_phase0_3000(m+6) > 0 ? 0x20 : 0) |
                (slice_phase0_3000(m+9) > 0 ? 0x10 : 0) |
                (slice_phase0_3000(m+12) > 0 ? 0x08 : 0) |
                (slice_phase0_3000(m+15) > 0 ? 0x04 : 0) |
                (slice_phase0_3000(m+18) > 0 ? 0x02 : 0) |
                (slice_phase0_3000(m+21) > 0 ? 0x01 : 0);
            break;

        case 1:
            theByte =
                (slice_phase1_3000(m+0) > 0 ? 0x80 : 0) |
                (slice_phase1_3000(m+3) > 0 ? 0x40 : 0) |
                (slice_phase1_3000(m+6) > 0 ? 0x20 : 0) |
                (slice_phase1_3000(m+9) > 0 ? 0x10 : 0) |
                (slice_phase1_3000(m+12) > 0 ? 0x08 : 0) |
                (slice_phase1_3000(m+15) > 0 ? 0x04 : 0) |
                (slice_phase1_3000(m+18) > 0 ? 0x02 : 0) |
                (slice_phase1_3000(m+21) > 0 ? 0x01 : 0);
            break;

        case 2:
            theByte =
                (slice_phase2_3000(m+0) > 0 ? 0x80 : 0) |
                (slice_phase2_3000(m+3) > 0 ? 0x40 : 0) |
                (slice_phase2_3000(m+6) > 0 ? 0x20 : 0) |
                (slice_phase2_3000(m+9) > 0 ? 0x10 : 0) |
                (slice_phase2_3000(m+12) > 0 ? 0x08 : 0) |
                (slice_phase2_3000(m+15) > 0 ? 0x04 : 0) |
                (slice_phase2_3000(m+18) > 0 ? 0x02 : 0) |
                (slice_phase2_3000(m+21) > 0 ? 0x01 : 0);
            break;
    }

    // Advance by 24 samples (8 bits * 3 samples/bit)
    *pPtr = m + 24;
    // Phase stays the same (3 mod 3 = 0)
    return theByte;
}

// Score a phase by attempting to decode and checking CRC
static void score_phase_3000(int try_phase, uint16_t *pa,
                             unsigned char **bestmsg, int *bestscore, int *bestphase,
                             unsigned char **msg) {
    int score, bytelen;

    // Point to start of data (after preamble) with sample offset
    uint16_t *pPtr = pa + PREAMBLE_SAMPLES_3000 + (try_phase / 3);
    int phase = try_phase % 3;

    // Slice first byte to get DF
    (*msg)[0] = slice_byte_3000(&pPtr, &phase);

    // Check DF field
    uint32_t df = ((uint8_t)(*msg)[0]) >> 3;

    // Determine message length from DF
    if (valid_df_long_bitset_3000 & (1 << df)) {
        bytelen = MODES_LONG_MSG_BYTES;
    } else if (valid_df_short_bitset_3000 & (1 << df)) {
        bytelen = MODES_SHORT_MSG_BYTES;
    } else {
        score = -2;
        if (score > *bestscore) {
            *bestscore = score;
        }
        return;
    }

    // Slice remaining bytes
    for (int i = 1; i < bytelen; ++i) {
        (*msg)[i] = slice_byte_3000(&pPtr, &phase);
    }

    // Score the message
    score = scoreModesMessage(*msg, bytelen * 8);

    if (score > *bestscore) {
        *bestscore = score;
        *bestphase = try_phase;

        // Swap message buffers
        unsigned char *tmp = *bestmsg;
        *bestmsg = *msg;
        *msg = tmp;
    }
}

// Main demodulation function for 3.0 MHz (interpolated from 2.88 MHz)
void demodulate3000(struct mag_buf *mag) {
    uint16_t *m = mag->data;
    uint32_t mlen = mag->length;

    // Need enough samples for preamble + long message
    // At 3.0 MHz: preamble 24 samples, long msg 336 samples
    unsigned trailing = PREAMBLE_SAMPLES_3000 + (MODES_LONG_MSG_BITS * 3) + 10;

    if (mlen < trailing) return;

    uint16_t *pa = m;
    uint16_t *stop = m + mlen - trailing;

    unsigned char msg1[MODES_LONG_MSG_BYTES];
    unsigned char msg2[MODES_LONG_MSG_BYTES];
    unsigned char *msg = msg1;
    unsigned char *bestmsg = msg2;

    for (; pa < stop; pa++) {
        // Quick pre-check for preamble
        // At 3.0 MHz: pulses at ~0-1, ~3-4, ~10-12, ~13-15
        // Quiet zones: ~6-8, ~17-20
        if (!(pa[1] > pa[7] && pa[4] > pa[7] && pa[11] > pa[18] && pa[14] > pa[18])) {
            continue;
        }

        // Noise estimate from quiet samples
        int32_t base_noise = pa[6] + pa[7] + pa[8] + pa[17] + pa[18];
        int32_t ref_level = (base_noise * Modes.preambleThreshold) >> 5;

        // Check preamble magnitude
        int32_t pa_mag = pa[0] + pa[1] + pa[3] + pa[4] + pa[10] + pa[11] + pa[13] + pa[14]
                        - pa[6] - pa[7] - pa[8] - pa[17] - pa[18];

        if (pa_mag < ref_level) {
            continue;
        }

        Modes.stats_current.demod_preambles++;

        int bestscore = -10;
        int bestphase = -1;
        int msglen;

        // Try extended phases (sample offset + phase within sample)
        // Phase 0-2: offset 0, Phase 3-5: offset 1, Phase 6-8: offset 2
        for (int try_phase = 0; try_phase <= 8; try_phase++) {
            score_phase_3000(try_phase, pa, &bestmsg, &bestscore, &bestphase, &msg);
        }

        if (bestscore < 0) {
            Modes.stats_current.demod_rejected_bad++;
            continue;
        }

        // Get message length
        msglen = modesMessageLenByType(bestmsg[0] >> 3);

        // Get message buffer from network layer
        struct modesMessage *mm = netGetMM(&Modes.netMessageBuffer[0]);

        // Fill in message structure
        mm->timestamp = mag->sampleTimestamp + (pa - m) * 12e6 / 3000000;
        mm->sysTimestamp = mag->sysTimestamp;
        mm->magnitude = pa;
        mm->magnitude_count = msglen * 3 + PREAMBLE_SAMPLES_3000;
        mm->try_phase = bestphase;

        memcpy(mm->msg, bestmsg, MODES_LONG_MSG_BYTES);

        // Decode the message
        int result = decodeModesMessage(mm);
        if (result < 0) {
            if (result == -1)
                Modes.stats_current.demod_rejected_unknown_icao++;
            else
                Modes.stats_current.demod_rejected_bad++;
            continue;
        }

        Modes.stats_current.demod_accepted[mm->correctedbits]++;
        Modes.stats_current.demod_bestPhase[bestphase % 3]++;

        // Calculate signal power
        {
            double signal_power;
            uint64_t scaled_signal_power = 0;
            int signal_len = msglen * 3;

            for (int k = 0; k < signal_len; ++k) {
                uint32_t mag_val = pa[PREAMBLE_SAMPLES_3000 + k];
                scaled_signal_power += mag_val * mag_val;
            }

            signal_power = scaled_signal_power / 65535.0 / 65535.0;
            mm->signalLevel = signal_power / signal_len;
            Modes.stats_current.signal_power_sum += signal_power;
            Modes.stats_current.signal_power_count += signal_len;

            if (mm->signalLevel > Modes.stats_current.peak_signal_power)
                Modes.stats_current.peak_signal_power = mm->signalLevel;
            if (mm->signalLevel > 0.50119)
                Modes.stats_current.strong_signal_count++;
        }

        // Skip past message
        pa += msglen * 3;

        // Pass to network layer
        netUseMessage(mm);
    }

    // Drain any remaining messages in the buffer
    netDrainMessageBuffers();
}
