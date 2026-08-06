/*
 * Tier-0 prefix-set Bloom filter -- AR9271 (Xtensa) port. See ndr_tier0.h.
 *
 * Bit-for-bit identical to `lr2021-nrf54l15-rs/src/tier0.rs`. `tools/ndr_tier0_selftest.c` checks
 * that against vectors generated from the Rust side; run it after touching anything here.
 */

#include "ndr_tier0.h"

/*
 * FNV-1a 64.
 *
 * The multiply is written as shift-and-add rather than `h * 0x100000001b3` deliberately. The
 * firmware links against no libgcc, so a 64-bit multiply on a 32-bit Xtensa target would emit an
 * undefined reference to __muldi3. The FNV prime factors exactly:
 *
 *     0x100000001b3 = 2^40 + 435,  435 = 2^8 + 2^7 + 2^5 + 2^4 + 2^1 + 2^0
 *
 * so the product mod 2^64 is a sum of constant shifts, which gcc inlines. This is arithmetically
 * identical to the Rust `wrapping_mul` -- it is the same value, not an approximation of it.
 */
a_uint64_t ndr_name_hash(a_uint64_t key, const a_uint8_t *name, a_uint32_t len)
{
	a_uint64_t h = ((a_uint64_t)0xcbf29ce484222325ULL) ^ key;
	a_uint32_t i;

	for (i = 0; i < len; i++) {
		h ^= (a_uint64_t)name[i];
		h = (h << 40) + (h << 8) + (h << 7) + (h << 5) + (h << 4) + (h << 1) + h;
	}

	return h;
}

/* Second-hash key derivation constant (golden ratio). */
#define NDR_KEY2_MIX  ((a_uint64_t)0x9E3779B97F4A7C15ULL)

/*
 * The K bit positions one prefix occupies.
 *
 * h1 and h2 are two INDEPENDENT keyed hashes, not the two halves of one. Splitting a single
 * FNV-1a output measured 1.3-3.4x worse at depths 4-8: FNV's high bits are its weak half, so using
 * them as the double-hashing stride correlates the K positions. A second pass over a short prefix
 * costs a few cycles and buys back the model.
 */
static void ndr_positions(a_uint8_t out[NDR_K], a_uint64_t key,
			  const a_uint8_t *prefix, a_uint32_t len)
{
	a_uint32_t h1 = (a_uint32_t)ndr_name_hash(key, prefix, len);
	/* `| 1` keeps the stride odd, so the K positions cannot collapse onto one bit. */
	a_uint32_t h2 = ((a_uint32_t)ndr_name_hash(key ^ NDR_KEY2_MIX, prefix, len)) | 1;
	a_uint32_t i;

	for (i = 0; i < NDR_K; i++)
		out[i] = (a_uint8_t)((h1 + i * h2) % NDR_M_BITS);
}

/* Bit p of the usable space maps to physical bit p+2, so bits 0 and 1 of octet 0 stay free. */
static void ndr_set_bit(ndr_filter_t *f, a_uint8_t pos)
{
	a_uint32_t p = (a_uint32_t)pos + 2;

	f->b[p / 8] |= (a_uint8_t)(1 << (p % 8));
}

/*
 * Truncate a registered prefix to the deepest form a sender would actually have inserted.
 *
 * ★ THIS IS LOAD-BEARING. Without it the depth cap produces TRUE FALSE NEGATIVES, which is the one
 * failure the whole design forbids.
 *
 * A sender inserting the prefixes of `/a/b/c/d/e/f/g/h/i` stops at the cap, so its deepest inserted
 * prefix is `/a/b/c/d/e/f/g` -- SEVEN components, not eight. A receiver registered on
 * `/a/b/c/d/e/f/g/h` therefore computes a mask over bits the sender never set, and drops a frame
 * that genuinely is under its prefix.
 *
 * The design's "zero false negatives at every depth" holds only for registrations within the cap;
 * the measured check could not see this because it only ever queried prefixes that had been
 * inserted, which makes the property tautological. Clamping here restores the guarantee: a
 * too-deep registration degrades to its 7-component ancestor, which costs extra false positives
 * (frames under the shallower prefix) and no false negatives. Tier 1/2 does the exact match --
 * which is what "deeper matching is left to the software tier" was always supposed to mean, and it
 * only works if the frame reaches that tier instead of being dropped here.
 *
 * Returns the clamped byte length.
 */
a_uint32_t ndr_clamp_prefix(const a_uint8_t *prefix, a_uint32_t len)
{
	a_uint32_t i, comps = 0;

	for (i = 1; i < len; i++) {
		if (prefix[i] == '/') {
			comps++;
			/* comps components precede this slash; the cap admits NDR_MAX_DEPTH-1. */
			if (comps >= NDR_MAX_DEPTH - 1)
				return i;
		}
	}

	return len;
}

void ndr_mask_for(ndr_filter_t *out, a_uint64_t key, const a_uint8_t *prefix, a_uint32_t len)
{
	a_uint8_t pos[NDR_K];
	a_uint32_t i;

	len = ndr_clamp_prefix(prefix, len);

	for (i = 0; i < 12; i++)
		out->b[i] = 0;

	ndr_positions(pos, key, prefix, len);
	for (i = 0; i < NDR_K; i++)
		ndr_set_bit(out, pos[i]);
}

/*
 * `false` is EXACT -- the name is definitely not under this prefix, so the frame can be dropped
 * without ever being parsed. `true` means probably, and a later tier decides.
 */
a_int32_t ndr_may_match(const ndr_filter_t *f, const ndr_filter_t *mask)
{
	a_uint32_t i;

	for (i = 0; i < 12; i++) {
		a_uint8_t want = mask->b[i];

		if (i == 0)
			want &= (a_uint8_t)~NDR_RESERVED_MASK0;

		if ((f->b[i] & want) != want)
			return 0;
	}

	return 1;
}

/*
 * 802.11 header: fc(2) dur(2) addr1(6) addr2(6) ... -- the filter is addr1 || addr2, so it starts
 * at offset 4 and runs 12 bytes. The caller must have checked the frame is at least 16 bytes.
 */
void ndr_filter_from_hdr(ndr_filter_t *out, const a_uint8_t *wh)
{
	a_uint32_t i;

	for (i = 0; i < 12; i++)
		out->b[i] = wh[4 + i];
}
