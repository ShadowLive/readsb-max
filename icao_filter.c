// Part of readsb, a Mode-S/ADSB/TIS message decoder.
//
// icao_filter.c: hashtable for ICAO addresses
//
// Copyright (c) 2019 Michael Wolf <michael@mictronics.de>
//
// This code is based on a detached fork of dump1090-fa.
//
// Copyright (c) 2014,2015 Oliver Jowett <oliver@mutability.co.uk>
//
// This file is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// any later version.
//
// This file is distributed in the hope that it will be useful, but
// WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
// General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.

#include "readsb.h"

// Open-addressed hash table with linear probing.

// Maintain two tables and switch between them to age out entries.

static uint32_t filterBits;
static uint32_t filterBuckets;
static size_t filterSize;
static uint32_t *icao_filter_a;
static uint32_t *icao_filter_b;
static uint32_t *icao_filter_active;

static uint32_t occupied;

static inline uint32_t filterHash(uint32_t addr) {
    return addrHash(addr, filterBits);
}

#define EMPTY 0xFFFFFFFF
#define MINBITS 8
#define MAXBITS 20

void icaoFilterInit() {
    filterBits = MINBITS;
    filterBuckets = 1ULL << filterBits;
    filterSize = filterBuckets * sizeof(uint32_t);
    occupied = 0;
    sfree(icao_filter_a);
    sfree(icao_filter_b);
    icao_filter_a = cmalloc(filterSize);
    icao_filter_b = cmalloc(filterSize);
    memset(icao_filter_a, 0xFF, filterSize);
    memset(icao_filter_b, 0xFF, filterSize);
    icao_filter_active = icao_filter_a;
}
void icaoFilterDestroy() {
    sfree(icao_filter_a);
    sfree(icao_filter_b);
}

static void icaoFilterResize(uint32_t bits) {
    uint32_t oldBuckets = filterBuckets;
    uint32_t *oldActive = icao_filter_active;
    uint32_t *oldA = icao_filter_a;
    uint32_t *oldB = icao_filter_b;

    filterBits = bits;
    filterBuckets = 1ULL << filterBits;
    filterSize = filterBuckets * sizeof(uint32_t);

    if (filterBuckets > 256000)
        fprintf(stderr, "icao_filter: changing size to %d!\n", (int) filterBuckets);

    icao_filter_a = cmalloc(filterSize);
    icao_filter_b = cmalloc(filterSize);
    memset(icao_filter_a, 0xFF, filterSize);
    memset(icao_filter_b, 0xFF, filterSize);

    // reset occupied count
    occupied = 0;
    icao_filter_active = icao_filter_a;
    for (uint32_t i = 0; i < oldBuckets; i++) {
        if (oldActive[i] != EMPTY) {
            icaoFilterAdd(oldActive[i]);
        }
    }
    sfree(oldA);
    sfree(oldB);
}

// call this periodically:
void icaoFilterExpire() {
    if (occupied < filterBuckets / 9 && filterBits > MINBITS) {
        icaoFilterResize(filterBits - 1);
    }
    // reset occupied count
    occupied = 0;
    if (icao_filter_active == icao_filter_a) {

        memset(icao_filter_b, 0xFF, filterSize);
        icao_filter_active = icao_filter_b;
    } else {
        memset(icao_filter_a, 0xFF, filterSize);
        icao_filter_active = icao_filter_a;
    }
}

void icaoFilterAdd(uint32_t addr) {
    uint32_t h, h0;
    h0 = h = filterHash(addr);
    while (icao_filter_active[h] != EMPTY && icao_filter_active[h] != addr) {
        h = (h + 1) & (filterBuckets - 1);
        if (h == h0) {
            fprintf(stderr, "ICAO hash table full, this shouldn't happen\n");
            return;
        }
    }
    if (icao_filter_active[h] == EMPTY) {
        occupied++;
        icao_filter_active[h] = addr;
    }

    if (occupied > filterBuckets / 3 && filterBits < 20) {
        icaoFilterResize(filterBits + 1);
    }
}

int icaoFilterTest(uint32_t addr) {
    uint32_t h, h0;

    h0 = h = filterHash(addr);
    while (icao_filter_a[h] != EMPTY && icao_filter_a[h] != addr) {
        h = (h + 1) & (filterBuckets - 1);
        if (h == h0)
            break;
    }
    if (icao_filter_a[h] == addr)
        return 1;

    h = h0;
    while (icao_filter_b[h] != EMPTY && icao_filter_b[h] != addr) {
        h = (h + 1) & (filterBuckets - 1);
        if (h == h0)
            break;
    }
    if (icao_filter_b[h] == addr)
        return 1;

    return 0;
}

// Get count of unique ICAOs in the active filter
uint32_t icaoFilterCount(void) {
    uint32_t count = 0;
    // Count entries in both tables, but avoid duplicates
    // For simplicity, just count the active table
    for (uint32_t i = 0; i < filterBuckets; i++) {
        if (icao_filter_active[i] != EMPTY) {
            count++;
        }
    }
    return count;
}

// Save all ICAOs from aircraft tracking hash table to a file
// This captures ALL aircraft seen during the session, plus preserves existing cache entries
int icaoFilterSave(const char *filename) {
    if (!filename) return -1;

    // First, load existing cache entries into a set to avoid duplicates
    uint32_t *existing = NULL;
    int existingCount = 0;
    int existingCap = 0;

    FILE *fin = fopen(filename, "r");
    if (fin) {
        char line[32];
        while (fgets(line, sizeof(line), fin)) {
            uint32_t addr;
            if (sscanf(line, "%x", &addr) == 1 && addr <= 0xFFFFFF) {
                if (existingCount >= existingCap) {
                    existingCap = existingCap ? existingCap * 2 : 256;
                    existing = realloc(existing, existingCap * sizeof(uint32_t));
                }
                existing[existingCount++] = addr;
            }
        }
        fclose(fin);
    }

    FILE *f = fopen(filename, "w");
    if (!f) {
        fprintf(stderr, "icaoFilterSave: failed to open %s for writing: %s\n",
                filename, strerror(errno));
        sfree(existing);
        return -1;
    }

    int count = 0;

    // Helper to check if ICAO already written
    #define IS_WRITTEN(addr, written, wcount) ({ \
        int found = 0; \
        for (int wi = 0; wi < wcount && !found; wi++) { \
            if (written[wi] == addr) found = 1; \
        } \
        found; \
    })

    // Allocate tracking array for what we write
    int writtenCap = existingCount + 1024;
    uint32_t *written = cmalloc(writtenCap * sizeof(uint32_t));
    int writtenCount = 0;

    // First write existing cache entries (preserve them)
    for (int i = 0; i < existingCount; i++) {
        fprintf(f, "%06X\n", existing[i]);
        written[writtenCount++] = existing[i];
        count++;
    }

    // Then add new aircraft from this session
    for (int j = 0; j < Modes.acBuckets; j++) {
        for (struct aircraft *a = Modes.aircraft[j]; a; a = a->next) {
            // Save ICAOs from reliably identified aircraft
            // Include ADS-B, ADS-R, TIS-B with ICAO addresses, and Mode-S
            if (a->addrtype <= ADDR_MODE_S) {
                if (!IS_WRITTEN(a->addr, written, writtenCount)) {
                    fprintf(f, "%06X\n", a->addr);
                    if (writtenCount < writtenCap) {
                        written[writtenCount++] = a->addr;
                    }
                    count++;
                }
            }
        }
    }

    #undef IS_WRITTEN

    sfree(written);
    sfree(existing);
    fclose(f);

    if (!Modes.quiet)
        fprintf(stderr, "icaoFilterSave: saved %d ICAOs to %s\n", count, filename);

    return count;
}

// Load ICAOs from a file into the filter
int icaoFilterLoad(const char *filename) {
    if (!filename) return -1;

    FILE *f = fopen(filename, "r");
    if (!f) {
        // File not existing is OK for first run
        if (errno == ENOENT) {
            if (!Modes.quiet)
                fprintf(stderr, "icaoFilterLoad: %s does not exist (will be created on exit)\n", filename);
            return 0;
        }
        fprintf(stderr, "icaoFilterLoad: failed to open %s: %s\n",
                filename, strerror(errno));
        return -1;
    }

    int count = 0;
    char line[32];
    while (fgets(line, sizeof(line), f)) {
        uint32_t addr;
        if (sscanf(line, "%x", &addr) == 1 && addr <= 0xFFFFFF) {
            icaoFilterAdd(addr);
            count++;
        }
    }

    fclose(f);

    if (!Modes.quiet)
        fprintf(stderr, "icaoFilterLoad: loaded %d ICAOs from %s\n", count, filename);

    return count;
}

// Count number of 1 bits in a 24-bit value (Hamming weight)
static inline int popcount24(uint32_t x) {
    x &= 0xFFFFFF;
    // Use built-in if available, otherwise manual count
#if defined(__GNUC__) || defined(__clang__)
    return __builtin_popcount(x);
#else
    x = x - ((x >> 1) & 0x555555);
    x = (x & 0x333333) + ((x >> 2) & 0x333333);
    x = (x + (x >> 4)) & 0x0F0F0F;
    return (x * 0x010101) >> 16;
#endif
}

// Test with error correction using CRC validation
// For Address/Parity messages, mm->crc = true_ICAO XOR error_syndrome
// If we find a known ICAO where (mm->crc XOR known_ICAO) has small Hamming weight,
// then the message likely had that many bit errors and is valid.
// Returns the corrected ICAO if found, or 0 if not found
// Sets *corrected_bits to the number of bits that were corrected (0, 1, or 2)
uint32_t icaoFilterTestWithCorrection(uint32_t syndrome, int max_errors, int *corrected_bits) {
    *corrected_bits = 0;
    syndrome &= 0xFFFFFF;

    // First try exact match (no errors)
    if (icaoFilterTest(syndrome)) {
        return syndrome;
    }

    // Search through all known ICAOs in both filter tables
    // For each known ICAO, compute error_pattern = syndrome XOR icao
    // If error_pattern has Hamming weight <= max_errors, we found a match
    // If multiple ICAOs match at the same error level, reject as ambiguous

    uint32_t best_icao = 0;
    int best_errors = max_errors + 1;
    int match_count = 0;  // Count matches at best error level

    // Search table A
    for (uint32_t i = 0; i < filterBuckets; i++) {
        if (icao_filter_a[i] != EMPTY) {
            uint32_t known_icao = icao_filter_a[i];
            uint32_t error_pattern = syndrome ^ known_icao;
            int errors = popcount24(error_pattern);
            if (errors > 0 && errors <= max_errors) {
                if (errors < best_errors) {
                    best_icao = known_icao;
                    best_errors = errors;
                    match_count = 1;
                } else if (errors == best_errors) {
                    match_count++;  // Another match at same level - ambiguous
                }
            }
        }
    }

    // Search table B
    for (uint32_t i = 0; i < filterBuckets; i++) {
        if (icao_filter_b[i] != EMPTY) {
            uint32_t known_icao = icao_filter_b[i];
            uint32_t error_pattern = syndrome ^ known_icao;
            int errors = popcount24(error_pattern);
            if (errors > 0 && errors <= max_errors) {
                if (errors < best_errors) {
                    best_icao = known_icao;
                    best_errors = errors;
                    match_count = 1;
                } else if (errors == best_errors && known_icao != best_icao) {
                    match_count++;  // Another match at same level - ambiguous
                }
            }
        }
    }

    // Only accept if we have exactly one match at the best error level
    if (best_icao && match_count == 1) {
        *corrected_bits = best_errors;
        return best_icao;
    }

    return 0; // Not found or ambiguous
}
