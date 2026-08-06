/*
 * Tier-0: the in-frame prefix-set Bloom filter, on the AR9271's Xtensa core.
 *
 * This is a faithful C port of `lr2021-nrf54l15-rs/src/tier0.rs`. It MUST stay bit-identical to
 * that implementation: the nRF54L15 testbed and this firmware have to agree on the wire, and the
 * measured false-positive numbers (§9 of named-filter-mac-redesign.md) were taken with the Rust
 * version. Any divergence silently breaks interop with no visible error — a frame just stops
 * matching.
 *
 * Design: `ndn-ext/crates/faces/ndn-face-monitor-wifi/docs/named-filter-mac-redesign.md` §3.
 *
 *   sender:   /A/b/c -> { /, /A, /A/b, /A/b/c } -> K bits set per prefix in an M-bit filter
 *   receiver: for each registered prefix P (mask precomputed once):
 *                 (frame & mask[P]) == mask[P]  => maybe under P  -> accept, parse
 *             else                              => DEFINITELY not -> drop, never parse
 *
 * The negative answer is exact. False positives cost a parse; false negatives cannot occur.
 *
 * WHY THIS FILE EXISTS ON THE DONGLE AND NOT THE HOST
 * ---------------------------------------------------
 * §8.2 of the design records that on commodity Wi-Fi we get Tier 0's CPU win but *not* NDN-NIC's
 * wakeup win: in monitor mode the NIC delivers everything, so the USB transfer and the host wakeup
 * have already happened before any filter of ours can run. The AR9271 is the exception. Its
 * firmware is ours, `ath_tgt_rx_tasklet()` runs on the dongle, and a frame rejected there never
 * crosses USB at all. This is the first Wi-Fi part in the stack where the paper's actual result is
 * reachable.
 */

#ifndef _NDR_TIER0_H_
#define _NDR_TIER0_H_

/*
 * The filter math is pure integer code with no firmware dependencies, so the same source builds
 * for the host self-test (which cross-checks it against the Rust implementation's vectors).
 */
#ifdef NDR_HOST_TEST
#include <stdint.h>
typedef uint8_t  a_uint8_t;
typedef uint32_t a_uint32_t;
typedef uint64_t a_uint64_t;
typedef int32_t  a_int32_t;
#else
#include <adf_os_types.h>
#endif

/* Usable filter bits: 96 (two address fields) minus the two reserved bits of octet 0. */
#define NDR_M_BITS      94

/*
 * Bit positions set per inserted prefix -- 4, MEASURED, not the 6 the formula predicts.
 *
 * The textbook optimum (M/n)*ln2 gives ~7 here. Measured at the depth cap on an nRF54L15
 * (20 000 trials per point) the optimum is k=4 (0.94% FP at depth 8, vs 1.09% at k=6). The formula
 * assumes a query's k positions are independent; at 94 bits they are not -- k=6 positions collide
 * with each other ~15% of the time, and a query whose 6 collapse to 3 distinct bits has the FP of
 * k=3. Small-m Bloom filters are their own regime. Do not "fix" this to match the formula.
 */
#define NDR_K           4

/* Deepest prefix inserted. Beyond this the filter saturates for every user of the frame. */
#define NDR_MAX_DEPTH   8

/* The two bits of octet 0 that must not be used by the filter (I/G and U/L). */
#define NDR_RESERVED_MASK0  0x03

/* A 96-bit in-frame filter: 94 usable bits plus the two reserved address bits. */
typedef struct {
	a_uint8_t b[12];
} ndr_filter_t;

/*
 * FNV-1a 64, keyed -- the same name-hash family the on-device LoRa data plane already uses, so the
 * filter shares one keyspace with the FIB and dedup rather than adding a second (open task #44).
 */
a_uint64_t ndr_name_hash(a_uint64_t key, const a_uint8_t *name, a_uint32_t len);

/* Could this frame's filter be under the prefix that `mask` was built from? */
a_int32_t ndr_may_match(const ndr_filter_t *f, const ndr_filter_t *mask);

/*
 * Truncate a registered prefix to the deepest form a sender would have inserted. Applied
 * automatically by ndr_mask_for(); exposed for the self-test. See the comment on the definition --
 * without it the depth cap yields true false negatives.
 */
a_uint32_t ndr_clamp_prefix(const a_uint8_t *prefix, a_uint32_t len);

/* The mask a receiver precomputes once per registered prefix. */
void ndr_mask_for(ndr_filter_t *out, a_uint64_t key, const a_uint8_t *prefix, a_uint32_t len);

/* Number of usable filter bits set. */
a_uint32_t ndr_popcount(const ndr_filter_t *f);

/* Lift the 94-bit filter out of a received 802.11 header's addr1||addr2. */
void ndr_filter_from_hdr(ndr_filter_t *out, const a_uint8_t *wh);

#endif /* _NDR_TIER0_H_ */
