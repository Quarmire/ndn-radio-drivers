/* The TimeToken — see ndr_time.h. */

#include <adf_os_io.h>

#include "ar5416reg.h"
#include "ndr_time.h"

struct ndr_time_state ndr_time_state = { NDR_TIME_MAGIC, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };

a_int32_t ndr_time_offset;

/*
 * Frames we have heard, so a later token's ref_idx can be matched to our own RX TSF for that frame.
 *
 * Tagged by source, because the index is a per-transmitter counter: with more than one peer on the
 * channel two transmitters' indices collide in this ring, and a reference would pair against the
 * wrong node's frame. The tag folds the source nonce, which is all the identity there is — and all
 * there should be, the addressing doctrine's source field being ephemeral by design.
 */
static a_uint32_t ring_idx[NDR_TT_RING];
static a_uint32_t ring_tag[NDR_TT_RING];
static a_uint32_t ring_tsf[NDR_TT_RING];

static void wr_be32(a_uint8_t *p, a_uint32_t v)
{
	p[0] = (a_uint8_t)(v >> 24);
	p[1] = (a_uint8_t)(v >> 16);
	p[2] = (a_uint8_t)(v >> 8);
	p[3] = (a_uint8_t)v;
}

static a_uint32_t rd_be32(const a_uint8_t *p)
{
	return ((a_uint32_t)p[0] << 24) | ((a_uint32_t)p[1] << 16) |
	       ((a_uint32_t)p[2] << 8) | (a_uint32_t)p[3];
}

a_uint32_t ndr_time_now(void)
{
	return ioread32_mac(AR_TSF_L32) + (a_uint32_t)ndr_time_offset;
}

void ndr_time_rx(const a_uint8_t *data, a_uint32_t len, a_uint32_t rx_tsf)
{
	a_uint32_t my, ref, ref_common, tag, slot;

	if (len < NDR_TT_OFF + NDR_TT_LEN)
		return;
	if (rd_be32(data + NDR_TT_OFF) != NDR_TT_MAGIC)
		return;

	my         = rd_be32(data + NDR_TT_OFF + 4);
	ref        = rd_be32(data + NDR_TT_OFF + 8);
	ref_common = rd_be32(data + NDR_TT_OFF + 12);
	ndr_time_state.rx_tokens++;

	/* addr2 -- the sender's ephemeral nonce -- folded to 32 bits. Not an identity, just enough
	 * to keep two peers' independent index spaces from being mistaken for one another. */
	tag = ((a_uint32_t)data[10] << 24) | ((a_uint32_t)data[11] << 16) |
	      ((a_uint32_t)data[12] << 8) | (a_uint32_t)data[13];
	tag ^= ((a_uint32_t)data[14] << 8) | (a_uint32_t)data[15];

	if (my) {
		slot = my & (NDR_TT_RING - 1u);
		ring_idx[slot] = my;
		ring_tag[slot] = tag;
		ring_tsf[slot] = rx_tsf;
	}

	if (!ref)
		return;

	slot = ref & (NDR_TT_RING - 1u);
	if (ring_idx[slot] != ref || ring_tag[slot] != tag)
		return; /* we never heard the frame this timestamp belongs to */
	ndr_time_state.paired++;

	{
		/*
		 * ref_common is when the peer sent that frame, in the peer's common frame;
		 * ring_tsf[slot] + offset is when we received it, in ours. The difference is the
		 * clock disagreement, less a few µs of RX latency.
		 *
		 * Signed, so this is the SHORTEST way round a wrapping counter rather than a raw
		 * magnitude -- which is the only interpretation that stays meaningful mod 2^32.
		 */
		a_int32_t delta = (a_int32_t)(ref_common -
					      (ring_tsf[slot] + (a_uint32_t)ndr_time_offset));

		ndr_time_state.last_delta = delta;

		/*
		 * ── Converge to the midpoint; do NOT adopt whoever is ahead ──────────
		 *
		 * "Adopt the leading clock", the IBSS merge rule, was tried here first and is
		 * MEASURED UNSTABLE on a 32-bit counter. Two nodes whose TSFs happened to sit
		 * ~2.02e9 µs apart -- just under 2^31 -- leapfrogged each other 2-3 times per 30 s
		 * run, each jumping forward by that same ~2.02e9. The reason is that at a separation
		 * near half the counter range "ahead by 2.02e9" and "behind by 2.27e9" are the same
		 * value mod 2^32, so adopting *re-creates* the separation instead of removing it. It
		 * is not merely ambiguous there, as an earlier comment here claimed: it is an
		 * attractor, and the pair never leaves it.
		 *
		 * Halving the difference has no such fixed point. Each node steps toward the other,
		 * so the separation halves per exchange no matter who leads -- ~21 exchanges from
		 * the worst case, well under a second at these frame rates. It also needs no leader,
		 * which is the property that matters for the doctrine: the rule is symmetric, every
		 * node runs the same one, and nothing is announced or elected.
		 *
		 * The deadband is what keeps it honest once converged. A receiver compares the
		 * peer's SEND time against its own RECEIVE time, so delta carries a systematic bias
		 * of one RX latency (~20 µs measured). Steering on that bias forever would walk both
		 * clocks backwards at ~1000 µs/s -- a 0.1% rate error, far worse than the ~1 ppm the
		 * crystals have on their own. So corrections stop once the disagreement is smaller
		 * than a slot boundary can notice: 1 TU, which is 8x the latency bias and 1/8 of a
		 * base slot. Inside the deadband the clocks simply free-run.
		 */
		if (delta > NDR_TIME_DEADBAND_US || delta < -NDR_TIME_DEADBAND_US) {
			ndr_time_offset += delta / 2;
			ndr_time_state.merges++;
		}
	}
}

void ndr_time_note_tx(a_uint32_t idx, a_uint32_t tsf)
{
	if (ndr_time_state.noted && tsf != ndr_time_state.last_tsf)
		ndr_time_state.advanced++;
	ndr_time_state.noted++;

	/*
	 * idx == 0 means the frame carried no token, so its send time is unattributable and must not
	 * become the reference -- but it still counts as a completion. That distinction is what
	 * bootstraps the whole thing: stamping is gated on having seen a completion, and the first
	 * frame out is necessarily unstamped, so a note path that ignored idx == 0 would never let
	 * the first stamp happen and no frame would ever carry a token.
	 */
	if (idx == 0)
		return;

	ndr_time_state.last_idx = idx;
	ndr_time_state.last_tsf = tsf;
}

a_uint32_t ndr_time_stamp_frame(a_uint8_t *data, a_uint32_t len)
{
	a_uint32_t idx;

	if (len < NDR_TT_OFF + NDR_TT_LEN)
		return 0;
	/* Nothing to report yet: leave the frame untouched rather than publish a zero reading. */
	if (ndr_time_state.noted == 0)
		return 0;

	idx = ++ndr_time_state.next_idx; /* 0 means "not stamped", so start at 1 */

	wr_be32(data + NDR_TT_OFF,      NDR_TT_MAGIC);
	wr_be32(data + NDR_TT_OFF + 4,  idx);                     /* this frame */
	wr_be32(data + NDR_TT_OFF + 8,  ndr_time_state.last_idx); /* the frame the TSF belongs to */
	/*
	 * COMMON time, not the raw TSF. Publishing the corrected reading is what makes the offsets
	 * composable: a receiver can compare what it reads here against its own common clock
	 * directly, with no knowledge of anyone's hardware TSF. Applying the offset here rather than
	 * at note_tx is deliberate -- it re-expresses the same past event in the node's current best
	 * time base, which is what a coordinate change should do.
	 */
	wr_be32(data + NDR_TT_OFF + 12, ndr_time_state.last_tsf + (a_uint32_t)ndr_time_offset);
	ndr_time_state.stamped++;
	return idx;
}

/*
 * Is the reported send time actually a TSF reading, rather than zero or a stale constant?
 *
 * Checks it against the live TSF in the same domain: a genuine AR_SendTimestamp is a microsecond
 * count from the same counter, so a frame that aired moments ago must sit just behind "now". A
 * constant, a zero, or a value from some unrelated counter fails this. Also requires that the
 * reading has moved at least once, which rules out a field that is written once and never updated.
 */
a_int32_t ndr_time_plausible(void)
{
	a_uint32_t now = ioread32_mac(AR_TSF_L32);
	a_uint32_t age = now - ndr_time_state.last_tsf; /* wraps correctly in u32 */

	if (ndr_time_state.noted == 0 || ndr_time_state.last_tsf == 0)
		return 0;
	if (ndr_time_state.advanced < 4)
		return 0;
	return age < 1000000u; /* aired within the last second */
}
