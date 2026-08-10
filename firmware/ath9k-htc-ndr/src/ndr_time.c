/* The TimeToken — see ndr_time.h. */

#include <adf_os_io.h>

#include "ar5416reg.h"
#include "ndr_time.h"

struct ndr_time_state ndr_time_state = { NDR_TIME_MAGIC, 0, 0, 0, 0, 0, 0 };

static void wr_be32(a_uint8_t *p, a_uint32_t v)
{
	p[0] = (a_uint8_t)(v >> 24);
	p[1] = (a_uint8_t)(v >> 16);
	p[2] = (a_uint8_t)(v >> 8);
	p[3] = (a_uint8_t)v;
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
	wr_be32(data + NDR_TT_OFF + 4,  idx);                    /* this frame */
	wr_be32(data + NDR_TT_OFF + 8,  ndr_time_state.last_idx); /* the frame the TSF belongs to */
	wr_be32(data + NDR_TT_OFF + 12, ndr_time_state.last_tsf);
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
