// demod_3200.c: 3.2MHz Mode S demodulator.
//
// This is an experimental demodulator for 3.2 MHz sample rate.
// Higher sample rate provides better timing resolution. 3.2 MHz is
// well-supported by RTL-SDR and occasional sample drops have minimal impact.
//
// Copyright (c) 2024, readsb contributors
// SPDX-License-Identifier: GPL-3.0-or-later

#include "readsb.h"

// 3.2MHz sampling rate version
//
// When sampling at 3.2MHz we have exactly 3.2 samples per bit.
// Each symbol is 500ns wide (1.6 samples), each sample is 312.5ns wide
//
// At 3.2 MHz:
//   - 1 symbol = 1.6 samples
//   - 1 bit = 3.2 samples
//   - Preamble (8 µs) = 25.6 samples (~26)
//   - 112-bit message = 358.4 samples
//
// We use 5 phases (0-4) where each phase represents 0.2 samples offset
// Phase advances by 3.2 samples per bit = 3 samples + 1 phase

// Valid DF bitsets for message length detection
// Short: DF 0, 4, 5, 11
// Long: DF 16, 17, 18, 20, 21
static const uint32_t valid_df_short_bitset_3200 = (1 << 0) | (1 << 4) | (1 << 5) | (1 << 11);
static const uint32_t valid_df_long_bitset_3200 = (1 << 16) | (1 << 17) | (1 << 18) | (1 << 20) | (1 << 21);

// Slice functions for 3.2 MHz
// At 3.2 MHz, one bit spans 3.2 samples (1.6 samples per symbol)
// Phase indicates where the bit boundary falls within the sample
//
// Following the 2.4 MHz approach: use simple 3-sample correlators where possible
// Manchester encoding: bit=1 means high-then-low, bit=0 means low-then-high
// We correlate: positive = high symbol first = bit 1
//
// At 3.2 MHz we have 1.6 samples/symbol, using 5 phases
// Phase step per bit: 3.2 samples = 3 samples + 0.2 samples = 3 samples + 1 phase

// Phase 0: bit boundary at sample start (offset 0.0)
// High symbol: 0.0-1.6, Low symbol: 1.6-3.2
static inline __attribute__((always_inline)) int slice_phase0_3200(uint16_t *m) {
    return 10 * m[0] + 2 * m[1] - 10 * m[2] - 2 * m[3];
}

// Phase 1: bit boundary 0.2 samples into sample 0 (offset 0.2)
// High: 0.2-1.8, Low: 1.8-3.4
static inline __attribute__((always_inline)) int slice_phase1_3200(uint16_t *m) {
    return 8 * m[0] + 6 * m[1] - 10 * m[2] - 4 * m[3];
}

// Phase 2: bit boundary 0.4 samples into sample 0 (offset 0.4)
// High: 0.4-2.0, Low: 2.0-3.6
static inline __attribute__((always_inline)) int slice_phase2_3200(uint16_t *m) {
    return 6 * m[0] + 10 * m[1] - 10 * m[2] - 6 * m[3];
}

// Phase 3: bit boundary 0.6 samples into sample 0 (offset 0.6)
// High: 0.6-2.2, Low: 2.2-3.8
static inline __attribute__((always_inline)) int slice_phase3_3200(uint16_t *m) {
    return 4 * m[0] + 10 * m[1] - 6 * m[2] - 8 * m[3];
}

// Phase 4: bit boundary 0.8 samples into sample 0 (offset 0.8)
// High: 0.8-2.4, Low: 2.4-4.0
static inline __attribute__((always_inline)) int slice_phase4_3200(uint16_t *m) {
    return 2 * m[0] + 10 * m[1] - 2 * m[2] - 10 * m[3];
}

// Slice a byte at 3.2 MHz
// Advances pPtr and phase appropriately
//
// At 3.2 MHz, each bit = 3.2 samples, so 8 bits = 25.6 samples
// Using 5 phases (0-4), phase increment per bit = 1 (since 3.2 = 3 + 0.2 = 3 samples + 1 phase)
//
// Sample offsets for each bit in a byte, starting from phase 0:
// Bit 0 @ phase 0, offset 0  -> phase 1 (+1 phase, +3 samples)
// Bit 1 @ phase 1, offset 3  -> phase 2 (+1 phase, +3 samples)
// Bit 2 @ phase 2, offset 6  -> phase 3 (+1 phase, +3 samples)
// Bit 3 @ phase 3, offset 9  -> phase 4 (+1 phase, +3 samples)
// Bit 4 @ phase 4, offset 12 -> phase 0 (+1 phase wraps, +4 samples)
// Bit 5 @ phase 0, offset 16 -> phase 1 (+1 phase, +3 samples)
// Bit 6 @ phase 1, offset 19 -> phase 2 (+1 phase, +3 samples)
// Bit 7 @ phase 2, offset 22 -> phase 3 (+1 phase, +3 samples)
// Total: 25 samples, next byte starts at phase 3

static inline __attribute__((always_inline)) uint8_t slice_byte_3200(uint16_t **pPtr, int *pPhase) {
    uint8_t theByte = 0;
    uint16_t *m = *pPtr;

    switch (*pPhase) {
        case 0:
            theByte =
                (slice_phase0_3200(m) > 0 ? 0x80 : 0) |
                (slice_phase1_3200(m+3) > 0 ? 0x40 : 0) |
                (slice_phase2_3200(m+6) > 0 ? 0x20 : 0) |
                (slice_phase3_3200(m+9) > 0 ? 0x10 : 0) |
                (slice_phase4_3200(m+12) > 0 ? 0x08 : 0) |
                (slice_phase0_3200(m+16) > 0 ? 0x04 : 0) |
                (slice_phase1_3200(m+19) > 0 ? 0x02 : 0) |
                (slice_phase2_3200(m+22) > 0 ? 0x01 : 0);
            *pPhase = 3;
            *pPtr = m + 25;
            break;

        case 1:
            theByte =
                (slice_phase1_3200(m) > 0 ? 0x80 : 0) |
                (slice_phase2_3200(m+3) > 0 ? 0x40 : 0) |
                (slice_phase3_3200(m+6) > 0 ? 0x20 : 0) |
                (slice_phase4_3200(m+9) > 0 ? 0x10 : 0) |
                (slice_phase0_3200(m+13) > 0 ? 0x08 : 0) |
                (slice_phase1_3200(m+16) > 0 ? 0x04 : 0) |
                (slice_phase2_3200(m+19) > 0 ? 0x02 : 0) |
                (slice_phase3_3200(m+22) > 0 ? 0x01 : 0);
            *pPhase = 4;
            *pPtr = m + 25;
            break;

        case 2:
            theByte =
                (slice_phase2_3200(m) > 0 ? 0x80 : 0) |
                (slice_phase3_3200(m+3) > 0 ? 0x40 : 0) |
                (slice_phase4_3200(m+6) > 0 ? 0x20 : 0) |
                (slice_phase0_3200(m+10) > 0 ? 0x10 : 0) |
                (slice_phase1_3200(m+13) > 0 ? 0x08 : 0) |
                (slice_phase2_3200(m+16) > 0 ? 0x04 : 0) |
                (slice_phase3_3200(m+19) > 0 ? 0x02 : 0) |
                (slice_phase4_3200(m+22) > 0 ? 0x01 : 0);
            *pPhase = 0;
            *pPtr = m + 26;
            break;

        case 3:
            theByte =
                (slice_phase3_3200(m) > 0 ? 0x80 : 0) |
                (slice_phase4_3200(m+3) > 0 ? 0x40 : 0) |
                (slice_phase0_3200(m+7) > 0 ? 0x20 : 0) |
                (slice_phase1_3200(m+10) > 0 ? 0x10 : 0) |
                (slice_phase2_3200(m+13) > 0 ? 0x08 : 0) |
                (slice_phase3_3200(m+16) > 0 ? 0x04 : 0) |
                (slice_phase4_3200(m+19) > 0 ? 0x02 : 0) |
                (slice_phase0_3200(m+23) > 0 ? 0x01 : 0);
            *pPhase = 1;
            *pPtr = m + 26;
            break;

        case 4:
            theByte =
                (slice_phase4_3200(m) > 0 ? 0x80 : 0) |
                (slice_phase0_3200(m+4) > 0 ? 0x40 : 0) |
                (slice_phase1_3200(m+7) > 0 ? 0x20 : 0) |
                (slice_phase2_3200(m+10) > 0 ? 0x10 : 0) |
                (slice_phase3_3200(m+13) > 0 ? 0x08 : 0) |
                (slice_phase4_3200(m+16) > 0 ? 0x04 : 0) |
                (slice_phase0_3200(m+20) > 0 ? 0x02 : 0) |
                (slice_phase1_3200(m+23) > 0 ? 0x01 : 0);
            *pPhase = 2;
            *pPtr = m + 26;
            break;
    }

    return theByte;
}

// Preamble structure at 3.2 MHz
// Preamble is 8 µs = 4 pulses at 0, 1, 3.5, 4.5 µs
// At 3.2 MHz (312.5ns/sample):
//   Pulse 0: sample ~0-1 (0 µs)
//   Pulse 1: sample ~3-5 (1 µs)
//   Pulse 2: sample ~11-13 (3.5 µs)
//   Pulse 3: sample ~14-16 (4.5 µs)
//   Data starts: sample ~26 (8 µs)

#define PREAMBLE_SAMPLES_3200 26

// Score a phase by attempting to decode and checking CRC
// try_phase encodes both sample offset and phase within sample:
//   sample_offset = try_phase / 5
//   phase_within_sample = try_phase % 5
// This allows sub-sample precision similar to 2.4 MHz demodulator
static void score_phase_3200(int try_phase, uint16_t *pa,
                             unsigned char **bestmsg, int *bestscore, int *bestphase,
                             unsigned char **msg) {
    int score, bytelen;

    // Point to start of data (after preamble) with sample offset from try_phase
    uint16_t *pPtr = pa + PREAMBLE_SAMPLES_3200 + (try_phase / 5);
    int phase = try_phase % 5;

    // Slice first byte to get DF
    (*msg)[0] = slice_byte_3200(&pPtr, &phase);

    // Check DF field
    uint32_t df = ((uint8_t)(*msg)[0]) >> 3;

    // Determine message length from DF
    if (valid_df_long_bitset_3200 & (1 << df)) {
        bytelen = MODES_LONG_MSG_BYTES;
    } else if (valid_df_short_bitset_3200 & (1 << df)) {
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
        (*msg)[i] = slice_byte_3200(&pPtr, &phase);
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

// Main demodulation function for 3.2 MHz
void demodulate3200(struct mag_buf *mag) {
    uint16_t *m = mag->data;
    uint32_t mlen = mag->length;

    // Need enough samples for preamble + long message
    // At 3.2 MHz: preamble ~26 samples, long msg ~359 samples
    unsigned trailing = PREAMBLE_SAMPLES_3200 + (MODES_LONG_MSG_BITS * 4);

    if (mlen < trailing) return;

    uint16_t *pa = m;
    uint16_t *stop = m + mlen - trailing;

    unsigned char msg1[MODES_LONG_MSG_BYTES];
    unsigned char msg2[MODES_LONG_MSG_BYTES];
    unsigned char *msg = msg1;
    unsigned char *bestmsg = msg2;

    for (; pa < stop; pa++) {
        // Quick pre-check for preamble
        // Pulse positions at 3.2 MHz: ~0-2, ~3-5, ~11-13, ~14-16
        // Quiet positions: ~7-9, ~18-22

        if (!(pa[1] > pa[8] && pa[4] > pa[8] && pa[12] > pa[20] && pa[15] > pa[20])) {
            continue;
        }

        // Noise estimate from quiet samples
        int32_t base_noise = pa[7] + pa[8] + pa[9] + pa[19] + pa[20];
        int32_t ref_level = (base_noise * Modes.preambleThreshold) >> 5;

        // Check preamble magnitude
        int32_t pa_mag = pa[0] + pa[1] + pa[2] + pa[4] + pa[5] + pa[11] + pa[12] + pa[13] + pa[15] + pa[16]
                        - pa[7] - pa[8] - pa[9] - pa[19] - pa[20];

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
        // Extended range for better coverage
        for (int try_phase = 0; try_phase <= 14; try_phase++) {
            score_phase_3200(try_phase, pa, &bestmsg, &bestscore, &bestphase, &msg);
        }

        if (bestscore < 0) {
            Modes.stats_current.demod_rejected_bad++;
            continue;
        }

        // Get message length using standard function
        msglen = modesMessageLenByType(bestmsg[0] >> 3);

        // Get message buffer from network layer (properly allocated)
        struct modesMessage *mm = netGetMM(&Modes.netMessageBuffer[0]);

        // Fill in message structure (netGetMM already zeros it)
        mm->timestamp = mag->sampleTimestamp + (pa - m) * 12e6 / Modes.sample_rate;
        mm->sysTimestamp = mag->sysTimestamp;
        mm->magnitude = pa;
        mm->magnitude_count = msglen * 16 / 5 + PREAMBLE_SAMPLES_3200;  // 3.2 samples per bit
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
        Modes.stats_current.demod_df_accepted[dfTypeIndex(mm->msgtype)][mm->correctedbits]++;
        Modes.stats_current.demod_bestPhase[bestphase]++;

        // Calculate signal power
        {
            double signal_power;
            uint64_t scaled_signal_power = 0;
            int signal_len = msglen * 16 / 5;  // 3.2 samples per bit

            for (int k = 0; k < signal_len; ++k) {
                uint32_t mag_val = pa[PREAMBLE_SAMPLES_3200 + k];
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

        // Skip past message (3.2 samples per bit)
        pa += msglen * 16 / 5;

        // Pass to network layer
        netUseMessage(mm);
    }

    // Drain any remaining messages in the buffer
    netDrainMessageBuffers();
}

// Mode A/C demodulator for 3.2 MHz (stub - not implemented)
void demodulate3200AC(struct mag_buf *mag) {
    (void)mag;
    // Mode A/C not implemented for 3.2 MHz
}
