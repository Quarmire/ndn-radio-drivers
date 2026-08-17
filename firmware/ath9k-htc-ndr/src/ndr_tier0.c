/*
 * Tier-0 prefix-set Bloom filter -- AR9271 (Xtensa) port. See ndr_tier0.h.
 *
 * Bit-for-bit identical to `lr2021-nrf54l15-rs/src/tier0.rs`. `tools/ndr_tier0_selftest.c` checks
 * that against vectors generated from the Rust side; run it after touching anything here.
 */

#include "ndr_tier0.h"

/*
 * SipHash-2-4, a faithful transcription of the reference and of the Rust copy in
 * `lr2021-nrf54l15-rs/src/tier0.rs`. See ndr_tier0.h for why FNV had to go.
 *
 * Every shift and rotate below is by a CONSTANT. That is deliberate: this firmware links no libgcc,
 * so a variable 64-bit shift would emit an undefined reference to __ashldi3. The same constraint
 * killed the plain 64-bit multiply in the FNV version. SipHash needs no multiply at all, so the
 * only trap is the tail assembly -- which is why the remainder is gathered into a byte array and
 * folded in with constant shifts rather than `b << (8 * i)`.
 */
#define ROTL64(x, n) (((x) << (n)) | ((x) >> (64 - (n))))

#define SIPROUND()                                     \
	do {                                           \
		v0 += v1;                              \
		v1 = ROTL64(v1, 13);                   \
		v1 ^= v0;                              \
		v0 = ROTL64(v0, 32);                   \
		v2 += v3;                              \
		v3 = ROTL64(v3, 16);                   \
		v3 ^= v2;                              \
		v0 += v3;                              \
		v3 = ROTL64(v3, 21);                   \
		v3 ^= v0;                              \
		v2 += v1;                              \
		v1 = ROTL64(v1, 17);                   \
		v1 ^= v2;                              \
		v2 = ROTL64(v2, 32);                   \
	} while (0)

static a_uint64_t rd_le64(const a_uint8_t *p)
{
	return (a_uint64_t)p[0] | ((a_uint64_t)p[1] << 8) | ((a_uint64_t)p[2] << 16) |
	       ((a_uint64_t)p[3] << 24) | ((a_uint64_t)p[4] << 32) | ((a_uint64_t)p[5] << 40) |
	       ((a_uint64_t)p[6] << 48) | ((a_uint64_t)p[7] << 56);
}

a_uint64_t ndr_siphash24(const a_uint8_t key[NDR_KEY_LEN], const a_uint8_t *data, a_uint32_t len)
{
	a_uint64_t k0 = rd_le64(key);
	a_uint64_t k1 = rd_le64(key + 8);
	a_uint64_t v0 = 0x736f6d6570736575ULL ^ k0;
	a_uint64_t v1 = 0x646f72616e646f6dULL ^ k1;
	a_uint64_t v2 = 0x6c7967656e657261ULL ^ k0;
	a_uint64_t v3 = 0x7465646279746573ULL ^ k1;
	a_uint64_t m, last;
	a_uint8_t tail[8];
	a_uint32_t i, whole = len & ~7u, rem = len & 7u;

	for (i = 0; i < whole; i += 8) {
		m = rd_le64(data + i);
		v3 ^= m;
		SIPROUND();
		SIPROUND();
		v0 ^= m;
	}

	for (i = 0; i < 8; i++)
		tail[i] = 0;
	for (i = 0; i < rem; i++)
		tail[i] = data[whole + i];

	/* Length byte in the top octet; at most 7 remainder bytes below it, folded with constant
	 * shifts so no __ashldi3 is needed. */
	last = ((a_uint64_t)(len & 0xff) << 56) | (a_uint64_t)tail[0] |
	       ((a_uint64_t)tail[1] << 8) | ((a_uint64_t)tail[2] << 16) |
	       ((a_uint64_t)tail[3] << 24) | ((a_uint64_t)tail[4] << 32) |
	       ((a_uint64_t)tail[5] << 40) | ((a_uint64_t)tail[6] << 48);

	v3 ^= last;
	SIPROUND();
	SIPROUND();
	v0 ^= last;

	v2 ^= 0xff;
	SIPROUND();
	SIPROUND();
	SIPROUND();
	SIPROUND();

	return v0 ^ v1 ^ v2 ^ v3;
}

a_uint64_t ndr_name_hash(const a_uint8_t key[NDR_KEY_LEN], const a_uint8_t *name, a_uint32_t len)
{
	return ndr_siphash24(key, name, len);
}

/* Domain separator for the second hash, so h1 and h2 are independent PRF evaluations under
 * different keys rather than two halves of one output. Must match KEY2_DOMAIN in tier0.rs. */
static const a_uint8_t NDR_KEY2_DOMAIN[NDR_KEY_LEN] = {
	'n', 'd', 'n', '/', 't', 'i', 'e', 'r', '0', '-', 'h', '2', 0, 0, 0, 0
};

/*
 * The K bit positions one prefix occupies.
 *
 * h1 and h2 are two INDEPENDENT keyed hashes, not the two halves of one -- splitting a single
 * output correlates the K positions and measured materially worse.
 */
static void ndr_positions(a_uint8_t out[NDR_K], const a_uint8_t key[NDR_KEY_LEN],
			  const a_uint8_t *prefix, a_uint32_t len)
{
	a_uint8_t key2[NDR_KEY_LEN];
	a_uint32_t h1, h2, i;

	for (i = 0; i < NDR_KEY_LEN; i++)
		key2[i] = key[i] ^ NDR_KEY2_DOMAIN[i];

	h1 = (a_uint32_t)ndr_name_hash(key, prefix, len);
	/* `| 1` keeps the stride odd, so the K positions cannot collapse onto one bit. */
	h2 = ((a_uint32_t)ndr_name_hash(key2, prefix, len)) | 1u;

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

void ndr_mask_for(ndr_filter_t *out, const a_uint8_t key[NDR_KEY_LEN],
		  const a_uint8_t *prefix, a_uint32_t len)
{
	a_uint8_t pos[NDR_K];
	a_uint32_t i;

	len = ndr_clamp_prefix(prefix, len);

	for (i = 0; i < 16; i++)
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

	/* Fill cap before any mask test — the mask test is a pure AND and an
	 * over-full filter passes every one of them. See NDR_FILL_CAP. */
	if (ndr_popcount(f) > NDR_FILL_CAP)
		return 0;

	for (i = 0; i < 16; i++) {
		a_uint8_t want = mask->b[i];

		if (i == 0)
			want &= (a_uint8_t)~NDR_RESERVED_MASK0;

		if ((f->b[i] & want) != want)
			return 0;
	}

	return 1;
}

a_uint32_t ndr_popcount(const ndr_filter_t *f)
{
	a_uint32_t i, n = 0;

	for (i = 0; i < NDR_M_BITS; i++) {
		a_uint32_t p = i + 2; /* skip the two reserved bits of octet 0 */

		if (f->b[p / 8] & (1 << (p % 8)))
			n++;
	}

	return n;
}

/*
 * 802.11 header: fc(2) dur(2) addr1(6) addr2(6) ... -- the filter is addr1 || addr2, so it starts
 * at offset 4 and runs 12 bytes. The caller must have checked the frame is at least 16 bytes.
 */
void ndr_filter_from_hdr(ndr_filter_t *out, const a_uint8_t *wh)
{
	a_uint32_t i;

	for (i = 0; i < 16; i++)
		out->b[i] = wh[4 + i];
}
