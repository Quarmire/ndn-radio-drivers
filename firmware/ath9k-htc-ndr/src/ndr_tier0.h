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
#define NDR_M_BITS      126

/* Hashes per prefix. Must equal `K` in the normative lr2021-nrf54l15-rs/src/tier0.rs. */
#define NDR_K           4

/* Deepest prefix inserted. Beyond this the filter saturates for every user of the frame. */
#define NDR_MAX_DEPTH   8

/*
 * Admission fill cap: the most set bits a RECEIVED filter may carry and still be
 * tested against a local mask.
 *
 * ndr_may_match is a pure AND, so an all-ones filter matches every registered
 * mask at every node — a one-frame universal wake, and once the scheduler keys
 * on this field, a one-frame network-wide claim suppression (every slot reads
 * busy, presence forged for every owner). A legitimate filter at NDR_MAX_DEPTH
 * sets 30 bits; 48 leaves headroom for future class tokens and bounds a
 * just-under-cap adversary to ~(48/94)^4 per targeted prefix.
 *
 * SHARED WIRE PARAMETER. Must equal FILL_CAP in the Rust copies
 * (ndn-face-monitor-wifi/src/tier0.rs, lr2021-nrf54l15-rs/src/tier0.rs) or the
 * implementations disagree about which frames are admissible.
 */
#define NDR_FILL_CAP    64

/* Group key width. The key IS the trust context (addressing doctrine §8). */
#define NDR_KEY_LEN     16

/* The two bits of octet 0 that must not be used by the filter (I/G and U/L). */
#define NDR_RESERVED_MASK0  0x03

/* A 96-bit in-frame filter: 94 usable bits plus the two reserved address bits. */
typedef struct {
	a_uint8_t b[16];
} ndr_filter_t;

/*
 * SipHash-2-4 -- the one agreed name-hash (addressing doctrine §8, task #44).
 *
 * This replaced keyed FNV-1a-64. FNV is not a PRF and XOR-ing a key into its init state is
 * invertible from observed output, so an outsider could recover a private group's key and then
 * compute -- or deliberately collide with -- its pre-parse filter. That is exactly the guarantee
 * the group key is supposed to provide.
 *
 * ⚠ **Both implementations must agree bit-for-bit or they cannot share a group.** A filter built
 * under one hash will not match masks built under another: no partial interop, no degradation, no
 * error -- names simply stop matching. `tools/ndr_tier0_selftest.c` is what makes that impossible
 * to do by accident; run it after touching anything here.
 */
a_uint64_t ndr_siphash24(const a_uint8_t key[NDR_KEY_LEN], const a_uint8_t *data, a_uint32_t len);

a_uint64_t ndr_name_hash(const a_uint8_t key[NDR_KEY_LEN], const a_uint8_t *name, a_uint32_t len);

/* Could this frame's filter be under the prefix that `mask` was built from? */
a_int32_t ndr_may_match(const ndr_filter_t *f, const ndr_filter_t *mask);

/*
 * Truncate a registered prefix to the deepest form a sender would have inserted. Applied
 * automatically by ndr_mask_for(); exposed for the self-test. See the comment on the definition --
 * without it the depth cap yields true false negatives.
 */
a_uint32_t ndr_clamp_prefix(const a_uint8_t *prefix, a_uint32_t len);

/* The mask a receiver precomputes once per registered prefix. */
void ndr_mask_for(ndr_filter_t *out, const a_uint8_t key[NDR_KEY_LEN],
		  const a_uint8_t *prefix, a_uint32_t len);

/* Number of usable filter bits set. */
a_uint32_t ndr_popcount(const ndr_filter_t *f);

/* Lift the 94-bit filter out of a received 802.11 header's addr1||addr2. */
void ndr_filter_from_hdr(ndr_filter_t *out, const a_uint8_t *wh);

#endif /* _NDR_TIER0_H_ */
