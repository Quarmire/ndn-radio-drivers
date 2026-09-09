/* ndr_parse.h — division-free, single-pass NDN name parser + fused rolling-FNV LPM.
 *
 * Target constraints (AR9271 MAGPIE Xtensa is the hard case): no divide/modulo, no I/D cache, so
 * every routine here is masks/shifts only, one forward pass, sequential reads, fixed scratch, no heap.
 * VAR-NUMBER is big-endian = network order (native on the BE Xtensa). Portable C: also builds on the
 * ESP32-C5 (RISC-V) and on a host for correctness/timing.
 *
 * Belongs at ndn-radio-drivers/firmware/ath9k-htc-ndr/src/ndr_parse.{c,h}, compiled by both firmwares.
 */
#ifndef NDR_PARSE_H
#define NDR_PARSE_H
/* Fixed-width types across all three targets: host (stdint), ESP32-C5 RISC-V firmware (stdint via
 * NDR_ESP), and AR9271 MAGPIE Xtensa firmware (no libc <stdint.h> — use the ath adf_os types). */
#if defined(NDR_ESP)
#include <stdint.h>
#elif defined(__XTENSA__)
/* AR9271 MAGPIE firmware: no libc <stdint.h>. dt_defs.h supplies uint8_t/uint16_t/uint32_t (matching
 * every other firmware TU so there is no conflicting typedef); it stops at 32-bit, so add uint64_t. */
#include "dt_defs.h"
typedef unsigned long long uint64_t;
#else
#include <stdint.h>
#endif

#define NDR_NAME_MAX  96   /* longest /-joined name we render (matches LoRa NAME_MAX) */
#define NDR_MAX_DEPTH 16   /* reject-early past this many components */

#define NDR_KIND_NONE     0
#define NDR_KIND_INTEREST 'I'
#define NDR_KIND_DATA     'D'

/* Walk raw NDN-TLV bytes to the Name TLV value, single bounded pass.
 * frame/len: Wi-Fi caller passes data+32 (past 802.11+LLC); LoRa passes raw (incl. optional 0xF5 GCS).
 * On a name: sets name_val and name_len to the Name value (0x08 comp ...) and kind, returns 1. Else 0. */
int ndr_walk_to_name(const uint8_t *frame, uint32_t len,
                     const uint8_t **name_val, uint32_t *name_len, uint8_t *kind);

/* HOT PATH — the fused forward/filter decision. One pass: walk to Name, then roll FNV-1a-64 across
 * the /-joined name testing longest-prefix membership against `set` at each component boundary.
 * Returns 1 (admit: some prefix of the name is in set) / 0 (miss or no name). *kind set if a name parsed. */
int ndr_name_admits(const uint8_t *frame, uint32_t len,
                    const uint64_t *set, uint32_t set_len, uint8_t *kind);

/* VERIFICATION PATH — walk + render the /-joined name into out (<=out_cap) + full-name FNV-1a-64.
 * Returns 1 with kind, name_out_len, full_hash and depth set; 0 on no-name or over-length. */
int ndr_parse_name(const uint8_t *frame, uint32_t len,
                   uint8_t *out, uint32_t out_cap,
                   uint8_t *kind, uint32_t *name_out_len,
                   uint64_t *full_hash, uint8_t *depth);

/* FNV-1a-64 over bytes — the #44 shared keyspace hash (host `fnv1a64` agrees). */
uint64_t ndr_fnv1a64(const uint8_t *p, uint32_t n);

#endif /* NDR_PARSE_H */
