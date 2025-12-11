// demod_4000.c: 4.0MHz Mode S demodulator.
//
// This demodulator is used after interpolating 3.2 MHz -> 4.0 MHz.
// At 4.0 MHz: exactly 4 samples per bit = 2 samples per symbol.
// This clean integer ratio eliminates the phase quantization error
// that plagues 3.2 MHz.
//
// Copyright (c) 2024, readsb contributors
// SPDX-License-Identifier: GPL-3.0-or-later

#include "readsb.h"

// 4.0MHz sampling rate version
//
// At 4.0 MHz we have exactly 4 samples per bit.
// Each symbol is 500ns wide (2 samples), each sample is 250ns wide.
//
// At 4.0 MHz:
//   - 1 symbol = 2 samples
//   - 1 bit = 4 samples
//   - Preamble (8 µs) = 32 samples
//   - 112-bit message = 448 samples
//
// Since 4 samples/bit is an exact integer, phase tracking is simple.
// We use 4 phases (0-3) representing 1/4 sample offsets.

// Valid DF bitsets for message length detection
static const uint32_t valid_df_short_bitset_4000 = (1 << 0) | (1 << 4) | (1 << 5) | (1 << 11);
static const uint32_t valid_df_long_bitset_4000 = (1 << 16) | (1 << 17) | (1 << 18) | (1 << 20) | (1 << 21);

// Slice functions for 4.0 MHz
// At 4.0 MHz, one bit spans exactly 4 samples (2 samples per symbol)
// Manchester encoding: bit=1 means high-then-low, bit=0 means low-then-high
//
// Phase 0: bit boundary at sample start (offset 0.0)
// High symbol: samples 0-1, Low symbol: samples 2-3
static inline __attribute__((always_inline)) int slice_phase0_4000(uint16_t *m) {
    // Samples 0,1 are high, samples 2,3 are low
    return m[0] + m[1] - m[2] - m[3];
}

// Phase 1: bit boundary 0.25 samples into sample 0 (offset 0.25)
// High symbol: 0.25-2.25, Low symbol: 2.25-4.25
static inline __attribute__((always_inline)) int slice_phase1_4000(uint16_t *m) {
    // Sample 0: 0.75 high, Sample 1: fully high, Sample 2: 0.25 high + 0.75 low
    // Sample 3: fully low, Sample 4: 0.25 from next bit
    return 3 * m[0] + 4 * m[1] + 1 * m[2] - 3 * m[2] - 4 * m[3] - 1 * m[4];
}

// Phase 2: bit boundary 0.5 samples into sample 0 (offset 0.5)
// High symbol: 0.5-2.5, Low symbol: 2.5-4.5
static inline __attribute__((always_inline)) int slice_phase2_4000(uint16_t *m) {
    // Sample 0: 0.5 high, Sample 1: fully high, Sample 2: 0.5 high + 0.5 low
    // Sample 3: fully low, Sample 4: 0.5 from next bit
    return 2 * m[0] + 4 * m[1] + 2 * m[2] - 2 * m[2] - 4 * m[3] - 2 * m[4];
}

// Phase 3: bit boundary 0.75 samples into sample 0 (offset 0.75)
// High symbol: 0.75-2.75, Low symbol: 2.75-4.75
static inline __attribute__((always_inline)) int slice_phase3_4000(uint16_t *m) {
    // Sample 0: 0.25 high, Sample 1: fully high, Sample 2: 0.75 high + 0.25 low
    // Sample 3: fully low, Sample 4: 0.75 from next bit
    return 1 * m[0] + 4 * m[1] + 3 * m[2] - 1 * m[2] - 4 * m[3] - 3 * m[4];
}

// Simplified slice functions (avoiding redundant operations)
static inline __attribute__((always_inline)) int slice_p0_4000(uint16_t *m) {
    return m[0] + m[1] - m[2] - m[3];
}

static inline __attribute__((always_inline)) int slice_p1_4000(uint16_t *m) {
    // 3*m[0] + 4*m[1] - 2*m[2] - 4*m[3] - m[4]
    return 3 * m[0] + 4 * m[1] - 2 * m[2] - 4 * m[3] - m[4];
}

static inline __attribute__((always_inline)) int slice_p2_4000(uint16_t *m) {
    // 2*m[0] + 4*m[1] - 4*m[3] - 2*m[4]  (m[2] terms cancel)
    return 2 * m[0] + 4 * m[1] - 4 * m[3] - 2 * m[4];
}

static inline __attribute__((always_inline)) int slice_p3_4000(uint16_t *m) {
    // m[0] + 4*m[1] + 2*m[2] - 4*m[3] - 3*m[4]
    return m[0] + 4 * m[1] + 2 * m[2] - 4 * m[3] - 3 * m[4];
}

// Preamble structure at 4.0 MHz
// Preamble is 8 µs = 4 pulses at 0, 1, 3.5, 4.5 µs
// At 4.0 MHz (250ns/sample):
//   Pulse 0: sample ~0-1 (0 µs)
//   Pulse 1: sample ~4-5 (1 µs)
//   Pulse 2: sample ~14-15 (3.5 µs)
//   Pulse 3: sample ~18-19 (4.5 µs)
//   Data starts: sample ~32 (8 µs)

#define PREAMBLE_SAMPLES_4000 32

// Slice a byte at 4.0 MHz
// At 4.0 MHz, each bit = 4 samples, so 8 bits = 32 samples exactly!
// With 4 phases, phase increment per bit = 0 (4 mod 4 = 0)
// This means starting phase is maintained throughout the byte!
static inline __attribute__((always_inline)) uint8_t slice_byte_4000(uint16_t **pPtr, int *pPhase) {
    uint8_t theByte = 0;
    uint16_t *m = *pPtr;
    int phase = *pPhase;

    // Since 4 samples/bit and 4 phases, phase stays constant within a byte
    // Bit positions: 0, 4, 8, 12, 16, 20, 24, 28 (all at same phase!)
    switch (phase) {
        case 0:
            theByte =
                (slice_p0_4000(m+0) > 0 ? 0x80 : 0) |
                (slice_p0_4000(m+4) > 0 ? 0x40 : 0) |
                (slice_p0_4000(m+8) > 0 ? 0x20 : 0) |
                (slice_p0_4000(m+12) > 0 ? 0x10 : 0) |
                (slice_p0_4000(m+16) > 0 ? 0x08 : 0) |
                (slice_p0_4000(m+20) > 0 ? 0x04 : 0) |
                (slice_p0_4000(m+24) > 0 ? 0x02 : 0) |
                (slice_p0_4000(m+28) > 0 ? 0x01 : 0);
            break;

        case 1:
            theByte =
                (slice_p1_4000(m+0) > 0 ? 0x80 : 0) |
                (slice_p1_4000(m+4) > 0 ? 0x40 : 0) |
                (slice_p1_4000(m+8) > 0 ? 0x20 : 0) |
                (slice_p1_4000(m+12) > 0 ? 0x10 : 0) |
                (slice_p1_4000(m+16) > 0 ? 0x08 : 0) |
                (slice_p1_4000(m+20) > 0 ? 0x04 : 0) |
                (slice_p1_4000(m+24) > 0 ? 0x02 : 0) |
                (slice_p1_4000(m+28) > 0 ? 0x01 : 0);
            break;

        case 2:
            theByte =
                (slice_p2_4000(m+0) > 0 ? 0x80 : 0) |
                (slice_p2_4000(m+4) > 0 ? 0x40 : 0) |
                (slice_p2_4000(m+8) > 0 ? 0x20 : 0) |
                (slice_p2_4000(m+12) > 0 ? 0x10 : 0) |
                (slice_p2_4000(m+16) > 0 ? 0x08 : 0) |
                (slice_p2_4000(m+20) > 0 ? 0x04 : 0) |
                (slice_p2_4000(m+24) > 0 ? 0x02 : 0) |
                (slice_p2_4000(m+28) > 0 ? 0x01 : 0);
            break;

        case 3:
            theByte =
                (slice_p3_4000(m+0) > 0 ? 0x80 : 0) |
                (slice_p3_4000(m+4) > 0 ? 0x40 : 0) |
                (slice_p3_4000(m+8) > 0 ? 0x20 : 0) |
                (slice_p3_4000(m+12) > 0 ? 0x10 : 0) |
                (slice_p3_4000(m+16) > 0 ? 0x08 : 0) |
                (slice_p3_4000(m+20) > 0 ? 0x04 : 0) |
                (slice_p3_4000(m+24) > 0 ? 0x02 : 0) |
                (slice_p3_4000(m+28) > 0 ? 0x01 : 0);
            break;
    }

    // Advance by 32 samples (8 bits * 4 samples/bit)
    *pPtr = m + 32;
    // Phase stays the same (4 mod 4 = 0)
    return theByte;
}

// Score a phase by attempting to decode and checking CRC
static void score_phase_4000(int try_phase, uint16_t *pa,
                             unsigned char **bestmsg, int *bestscore, int *bestphase,
                             unsigned char **msg) {
    int score, bytelen;

    // Point to start of data (after preamble) with sample offset
    uint16_t *pPtr = pa + PREAMBLE_SAMPLES_4000 + (try_phase / 4);
    int phase = try_phase % 4;

    // Slice first byte to get DF
    (*msg)[0] = slice_byte_4000(&pPtr, &phase);

    // Check DF field
    uint32_t df = ((uint8_t)(*msg)[0]) >> 3;

    // Determine message length from DF
    if (valid_df_long_bitset_4000 & (1 << df)) {
        bytelen = MODES_LONG_MSG_BYTES;
    } else if (valid_df_short_bitset_4000 & (1 << df)) {
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
        (*msg)[i] = slice_byte_4000(&pPtr, &phase);
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

// Main demodulation function for 4.0 MHz (interpolated from 3.2 MHz)
void demodulate4000(struct mag_buf *mag) {
    uint16_t *m = mag->data;
    uint32_t mlen = mag->length;

    // Need enough samples for preamble + long message
    // At 4.0 MHz: preamble 32 samples, long msg 448 samples
    unsigned trailing = PREAMBLE_SAMPLES_4000 + (MODES_LONG_MSG_BITS * 4) + 10;

    if (mlen < trailing) return;

    uint16_t *pa = m;
    uint16_t *stop = m + mlen - trailing;

    unsigned char msg1[MODES_LONG_MSG_BYTES];
    unsigned char msg2[MODES_LONG_MSG_BYTES];
    unsigned char *msg = msg1;
    unsigned char *bestmsg = msg2;

    for (; pa < stop; pa++) {
        // Quick pre-check for preamble
        // At 4.0 MHz: pulses at ~0-1, ~4-5, ~14-15, ~18-19
        // Quiet zones: ~8-10, ~22-26
        if (!(pa[1] > pa[9] && pa[5] > pa[9] && pa[15] > pa[24] && pa[19] > pa[24])) {
            continue;
        }

        // Noise estimate from quiet samples
        int32_t base_noise = pa[8] + pa[9] + pa[10] + pa[22] + pa[23] + pa[24];
        int32_t ref_level = (base_noise * Modes.preambleThreshold) >> 5;

        // Check preamble magnitude
        int32_t pa_mag = pa[0] + pa[1] + pa[4] + pa[5] + pa[14] + pa[15] + pa[18] + pa[19]
                        - pa[8] - pa[9] - pa[10] - pa[22] - pa[23] - pa[24];

        if (pa_mag < ref_level) {
            continue;
        }

        Modes.stats_current.demod_preambles++;

        int bestscore = -10;
        int bestphase = -1;
        int msglen;

        // Try extended phases (sample offset + phase within sample)
        // Phase 0-3: offset 0, Phase 4-7: offset 1, Phase 8-11: offset 2, etc.
        for (int try_phase = 0; try_phase <= 15; try_phase++) {
            score_phase_4000(try_phase, pa, &bestmsg, &bestscore, &bestphase, &msg);
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
        mm->timestamp = mag->sampleTimestamp + (pa - m) * 12e6 / 4000000;
        mm->sysTimestamp = mag->sysTimestamp;
        mm->magnitude = pa;
        mm->magnitude_count = msglen * 4 + PREAMBLE_SAMPLES_4000;
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
        Modes.stats_current.demod_bestPhase[bestphase % 4]++;

        // Calculate signal power
        {
            double signal_power;
            uint64_t scaled_signal_power = 0;
            int signal_len = msglen * 4;

            for (int k = 0; k < signal_len; ++k) {
                uint32_t mag_val = pa[PREAMBLE_SAMPLES_4000 + k];
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
        pa += msglen * 4;

        // Pass to network layer
        netUseMessage(mm);
    }

    // Drain any remaining messages in the buffer
    netDrainMessageBuffers();
}
