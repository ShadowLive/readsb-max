// demod_2880.c: 2.88MHz Mode S demodulator.
//
// At 2.88 MHz: 2.88 samples per bit, 1.44 samples per symbol.
// We use 5 phases (0.2 sample steps) similar to 2.4 MHz.
// Phase advance per bit: 2.88/0.2 = 14.4 phases, which wraps differently than 2.4 MHz.
// After 8 bits at 2.88 samples/bit = 23.04 samples, next byte starts ~23 samples later.
//
// The key difference from 2.4 MHz:
// - 2.4 MHz: 8 bits = 19.2 samples, phases cycle 0->1->2->3->4->0
// - 2.88 MHz: 8 bits = 23.04 samples, each phase returns to itself after one byte
//
// Copyright (c) 2024, readsb contributors
// SPDX-License-Identifier: GPL-3.0-or-later

#include "readsb.h"
// Note: ML/AVC slicers are not implemented for 2.88 MHz
// Those flags only affect the 2.4 MHz demodulator

// 2.88MHz sampling rate version
//
// When sampling at 2.88MHz we have 2.88 samples per bit.
// Using 5 phases (0.2 sample = 83.33ns steps), each bit advances by
// 2.88/0.2 = 14.4 phase units. Since we use mod 5, this creates a complex pattern.
//
// The slice functions correlate a 1-0 symbol pair (Manchester encoded bit).
// Positive result = bit 1, negative result = bit 0.
//
// Coefficients computed from Manchester encoding overlap integrals:
// Phase 0: bit at sample[0], symbol transition at 1.44
// Phase 1: bit starts 0.2 samples before sample[0]
// etc.

// Valid DF bitsets for message length detection
static const uint32_t valid_df_short_bitset_2880 = (1 << 0) | (1 << 4) | (1 << 5) | (1 << 11);
static const uint32_t valid_df_long_bitset_2880 = (1 << 16) | (1 << 17) | (1 << 18) | (1 << 20) | (1 << 21);

// Slice functions for each phase
// Based on Manchester encoding overlap integrals at 2.88 MHz sample rate

// Phase 0: bit starts at sample[0] (offset 0.0)
// High: 0.0-1.44, Low: 1.44-2.88
static inline __attribute__((always_inline)) int slice_phase0_2880(uint16_t *m) {
    return 10 * m[0] - 1 * m[1] - 9 * m[2];
}

// Phase 1: bit starts 0.2 samples into sample[0] (offset 0.2)
// High: 0.2-1.64, Low: 1.64-3.08
static inline __attribute__((always_inline)) int slice_phase1_2880(uint16_t *m) {
    return 8 * m[0] + 3 * m[1] - 10 * m[2] - 1 * m[3];
}

// Phase 2: bit starts 0.4 samples into sample[0] (offset 0.4)
// High: 0.4-1.84, Low: 1.84-3.28
static inline __attribute__((always_inline)) int slice_phase2_2880(uint16_t *m) {
    return 6 * m[0] + 7 * m[1] - 10 * m[2] - 3 * m[3];
}

// Phase 3: bit starts 0.6 samples into sample[0] (offset 0.6)
// High: 0.6-2.04, Low: 2.04-3.48
static inline __attribute__((always_inline)) int slice_phase3_2880(uint16_t *m) {
    return 4 * m[0] + 10 * m[1] - 9 * m[2] - 5 * m[3];
}

// Phase 4: bit starts 0.8 samples into sample[0] (offset 0.8)
// High: 0.8-2.24, Low: 2.24-3.68
static inline __attribute__((always_inline)) int slice_phase4_2880(uint16_t *m) {
    return 2 * m[0] + 10 * m[1] - 5 * m[2] - 7 * m[3];
}

// Preamble structure at 2.88 MHz
// Preamble is 8 µs = 4 pulse pairs at 0, 1, 3.5, 4.5 µs
// At 2.88 MHz (347.2ns/sample):
//   Pulse 0: sample ~0-1 (0 µs)
//   Pulse 1: sample ~3-4 (1 µs)
//   Pulse 2: sample ~10-11 (3.5 µs)
//   Pulse 3: sample ~13-14 (4.5 µs)
//   Data starts: sample ~23 (8 µs)
//
#define PREAMBLE_SAMPLES_2880 23

// Bit position tracking for 2.88 MHz
// At 2.88 samples/bit, 8 bits span 23.04 samples
// Using integer samples: 23 samples per byte
//
// Bit offsets within a byte (starting from phase 0):
// Bit 0: sample 0, phase 0
// Bit 1: sample 2, phase 4 (0 + 2.88 = 2.88, int=2, frac=0.88 -> phase 4)
// Bit 2: sample 5, phase 4 (2.88 + 2.88 = 5.76, int=5, frac=0.76 -> phase 4)
// ... etc.
//
// Rather than tracking complex phase cycling, we use pre-computed sample offsets
// The pattern repeats every byte since 23.04 ≈ 23 samples

// Extract one byte from magnitude buffer
// Uses pre-computed sample offsets and phase indices for each bit
static inline __attribute__((always_inline)) uint8_t slice_byte_2880(uint16_t **pPtr, int *phase) {
    uint8_t theByte = 0;
    uint16_t *m = *pPtr;

    // At 2.88 MHz with 5 phases, each starting phase returns to itself after 8 bits
    // The bit positions within a byte depend on starting phase

    switch (*phase) {
        case 0:
            // Bit positions: [(0, 0), (2, 4), (5, 3), (8, 3), (11, 2), (14, 1), (17, 1), (20, 0)]
            theByte =
                (slice_phase0_2880(m+0) > 0 ? 0x80 : 0) |
                (slice_phase4_2880(m+2) > 0 ? 0x40 : 0) |
                (slice_phase3_2880(m+5) > 0 ? 0x20 : 0) |
                (slice_phase3_2880(m+8) > 0 ? 0x10 : 0) |
                (slice_phase2_2880(m+11) > 0 ? 0x08 : 0) |
                (slice_phase1_2880(m+14) > 0 ? 0x04 : 0) |
                (slice_phase1_2880(m+17) > 0 ? 0x02 : 0) |
                (slice_phase0_2880(m+20) > 0 ? 0x01 : 0);
            *phase = 0;
            *pPtr = m + 23;
            break;

        case 1:
            // Bit positions: [(0, 1), (3, 0), (5, 4), (8, 4), (11, 3), (14, 2), (17, 2), (20, 1)]
            theByte =
                (slice_phase1_2880(m+0) > 0 ? 0x80 : 0) |
                (slice_phase0_2880(m+3) > 0 ? 0x40 : 0) |
                (slice_phase4_2880(m+5) > 0 ? 0x20 : 0) |
                (slice_phase4_2880(m+8) > 0 ? 0x10 : 0) |
                (slice_phase3_2880(m+11) > 0 ? 0x08 : 0) |
                (slice_phase2_2880(m+14) > 0 ? 0x04 : 0) |
                (slice_phase2_2880(m+17) > 0 ? 0x02 : 0) |
                (slice_phase1_2880(m+20) > 0 ? 0x01 : 0);
            *phase = 1;
            *pPtr = m + 23;
            break;

        case 2:
            // Bit positions: [(0, 2), (3, 1), (6, 0), (9, 0), (11, 4), (14, 3), (17, 3), (20, 2)]
            theByte =
                (slice_phase2_2880(m+0) > 0 ? 0x80 : 0) |
                (slice_phase1_2880(m+3) > 0 ? 0x40 : 0) |
                (slice_phase0_2880(m+6) > 0 ? 0x20 : 0) |
                (slice_phase0_2880(m+9) > 0 ? 0x10 : 0) |
                (slice_phase4_2880(m+11) > 0 ? 0x08 : 0) |
                (slice_phase3_2880(m+14) > 0 ? 0x04 : 0) |
                (slice_phase3_2880(m+17) > 0 ? 0x02 : 0) |
                (slice_phase2_2880(m+20) > 0 ? 0x01 : 0);
            *phase = 2;
            *pPtr = m + 23;
            break;

        case 3:
            // Bit positions: [(0, 3), (3, 2), (6, 1), (9, 1), (12, 0), (14, 4), (17, 4), (20, 3)]
            theByte =
                (slice_phase3_2880(m+0) > 0 ? 0x80 : 0) |
                (slice_phase2_2880(m+3) > 0 ? 0x40 : 0) |
                (slice_phase1_2880(m+6) > 0 ? 0x20 : 0) |
                (slice_phase1_2880(m+9) > 0 ? 0x10 : 0) |
                (slice_phase0_2880(m+12) > 0 ? 0x08 : 0) |
                (slice_phase4_2880(m+14) > 0 ? 0x04 : 0) |
                (slice_phase4_2880(m+17) > 0 ? 0x02 : 0) |
                (slice_phase3_2880(m+20) > 0 ? 0x01 : 0);
            *phase = 3;
            *pPtr = m + 23;
            break;

        case 4:
            // Bit positions: [(0, 4), (3, 3), (6, 2), (9, 2), (12, 1), (15, 0), (18, 0), (20, 4)]
            theByte =
                (slice_phase4_2880(m+0) > 0 ? 0x80 : 0) |
                (slice_phase3_2880(m+3) > 0 ? 0x40 : 0) |
                (slice_phase2_2880(m+6) > 0 ? 0x20 : 0) |
                (slice_phase2_2880(m+9) > 0 ? 0x10 : 0) |
                (slice_phase1_2880(m+12) > 0 ? 0x08 : 0) |
                (slice_phase0_2880(m+15) > 0 ? 0x04 : 0) |
                (slice_phase0_2880(m+18) > 0 ? 0x02 : 0) |
                (slice_phase4_2880(m+20) > 0 ? 0x01 : 0);
            *phase = 4;
            *pPtr = m + 23;
            break;
    }

    return theByte;
}

// Score a phase by attempting to decode and checking CRC
// try_phase encodes both sample offset and phase within sample:
//   sample_offset = try_phase / 5
//   phase_within_sample = try_phase % 5
// This allows sub-sample precision similar to 2.4 MHz demodulator
static void score_phase_2880(int try_phase, uint16_t *pa,
                             unsigned char **bestmsg, int *bestscore, int *bestphase,
                             unsigned char **msg) {
    int score, bytelen;

    // Point to start of data (after preamble) with sample offset from try_phase
    uint16_t *pPtr = pa + PREAMBLE_SAMPLES_2880 + (try_phase / 5);
    int phase = try_phase % 5;

    // Slice first byte to get DF
    (*msg)[0] = slice_byte_2880(&pPtr, &phase);

    // Check DF field
    uint32_t df = ((uint8_t)(*msg)[0]) >> 3;

    // Determine message length from DF
    if (valid_df_long_bitset_2880 & (1 << df)) {
        bytelen = MODES_LONG_MSG_BYTES;
    } else if (valid_df_short_bitset_2880 & (1 << df)) {
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
        (*msg)[i] = slice_byte_2880(&pPtr, &phase);
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

// Main demodulation function for 2.88 MHz
void demodulate2880(struct mag_buf *mag) {
    uint16_t *m = mag->data;
    uint32_t mlen = mag->length;

    // Need enough samples for preamble + long message
    // At 2.88 MHz: preamble ~23 samples, long msg ~323 samples (112 bits * 2.88)
    unsigned trailing = PREAMBLE_SAMPLES_2880 + (MODES_LONG_MSG_BITS * 3) + 10;

    if (mlen < trailing) return;

    uint16_t *pa = m;
    uint16_t *stop = m + mlen - trailing;

    unsigned char msg1[MODES_LONG_MSG_BYTES];
    unsigned char msg2[MODES_LONG_MSG_BYTES];
    unsigned char *msg = msg1;
    unsigned char *bestmsg = msg2;

    for (; pa < stop; pa++) {
        // Quick pre-check for preamble
        // Pulse positions at 2.88 MHz: ~0-1, ~3-4, ~10-11, ~13-14
        // Quiet zones: ~6-8, ~17-19
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

        // Try extended phases like 2.4 MHz demodulator
        // try_phase encodes sample_offset (try_phase/5) + phase_within_sample (try_phase%5)
        // This provides sub-sample precision for better preamble alignment
        // Phase 0-4: offset 0, phases 0-4
        // Phase 5-9: offset 1, phases 0-4
        // Phase 10-14: offset 2, phases 0-4
        // Phase 15-19: offset 3, phases 0-4
        // Use more phases for 2.88 MHz to compensate for phase quantization error
        for (int try_phase = 0; try_phase <= 19; try_phase++) {
            score_phase_2880(try_phase, pa, &bestmsg, &bestscore, &bestphase, &msg);
        }

        // Debug removed

        if (bestscore < 0) {
            // Failed CRC output for multi-receiver correlation
            if (Modes.failed_crc_out.connections && bestmsg) {
                int score_msglen = modesMessageLenByType(bestmsg[0] >> 3);
                if (score_msglen > 0) {
                    uint32_t score_crc = modesChecksum(bestmsg, score_msglen * 8);
                    int64_t score_ts = mag->sampleTimestamp + (pa - m) * 12e6 / Modes.sample_rate;
                    double score_sig = (double)(pa[0] + pa[1] + pa[3] + pa[4]) / (4.0 * 65535.0);
                    modesSendFailedCrcOutput(bestmsg, score_msglen, score_ts, score_sig, score_crc);
                }
            }
            Modes.stats_current.demod_rejected_bad++;
            continue;
        }

        // Determine message length from DF
        msglen = modesMessageLenByType(bestmsg[0] >> 3);

        // Get message buffer from network layer (properly allocated)
        struct modesMessage *mm = netGetMM(&Modes.netMessageBuffer[0]);

        // Fill in message structure
        mm->timestamp = mag->sampleTimestamp + (pa - m) * 12e6 / Modes.sample_rate;
        mm->sysTimestamp = mag->sysTimestamp;
        mm->magnitude = pa;
        mm->magnitude_count = msglen * 12 / 5 + PREAMBLE_SAMPLES_2880;
        mm->try_phase = bestphase;

        memcpy(mm->msg, bestmsg, MODES_LONG_MSG_BYTES);

        // Decode the message
        int result = decodeModesMessage(mm);
        if (result < 0) {
            // Failed CRC output for multi-receiver correlation
            if (Modes.failed_crc_out.connections && result == -2) {
                uint32_t crc = modesChecksum(mm->msg, msglen * 8);
                double sig_est = (double)(pa[0] + pa[1] + pa[3] + pa[4]) / (4.0 * 65535.0);
                modesSendFailedCrcOutput(mm->msg, msglen, mm->timestamp, sig_est, crc);
            }
            if (result == -1)
                Modes.stats_current.demod_rejected_unknown_icao++;
            else
                Modes.stats_current.demod_rejected_bad++;
            continue;
        }

        Modes.stats_current.demod_accepted[mm->correctedbits]++;
        Modes.stats_current.demod_df_accepted[dfTypeIndex(mm->msgtype)][mm->correctedbits]++;
        Modes.stats_current.demod_bestPhase[bestphase]++;

        // Calculate signal power
        {
            double signal_power;
            uint64_t scaled_signal_power = 0;
            int signal_len = msglen * 12 / 5;

            for (int k = 0; k < signal_len; ++k) {
                uint32_t mag_val = pa[PREAMBLE_SAMPLES_2880 + k];
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

        // Skip past message (samples per bit at 2.88 MHz is 2.88, use 3 as approximation)
        pa += msglen * 12 / 5;

        // Pass to network layer
        netUseMessage(mm);
    }

    // Drain any remaining messages in the buffer
    netDrainMessageBuffers();
}

// Mode A/C demodulator for 2.88 MHz (stub - not implemented)
void demodulate2880AC(struct mag_buf *mag) {
    (void)mag;
    // Mode A/C not implemented for 2.88 MHz
}
