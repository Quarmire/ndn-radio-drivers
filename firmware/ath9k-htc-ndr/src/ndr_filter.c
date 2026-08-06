/*
 * Tier-0 receive policy -- see ndr_filter.h.
 *
 * Defaults are deliberately inert: `enabled = 0` means this firmware behaves byte-identically to
 * stock. The first flash should change nothing observable; only then do we turn the filter on and
 * measure the difference. That ordering is the whole point -- if the filtered and unfiltered builds
 * differ in any way other than the filter, the measurement is worthless.
 */

#include "ndr_filter.h"

struct ndr_cfg ndr_cfg = {
	NDR_CFG_MAGIC,
	0,          /* enabled      -- off until explicitly turned on */
	0,          /* drop_foreign -- off; see the note in ndr_rx_accept() */
	0,          /* n_masks */
	0,          /* key */
	{ { { 0 } } },
};

struct ndr_stats ndr_stats = { 0, 0, 0, 0, 0 };

/* fc(2) dur(2) addr1(6) addr2(6) -- the filter ends at offset 16. */
#define NDR_MIN_HDR  16

a_int32_t ndr_rx_accept(const a_uint8_t *data, a_uint32_t len)
{
	ndr_filter_t f;
	a_uint32_t i;

	ndr_stats.seen++;

	if (!ndr_cfg.enabled) {
		ndr_stats.passed++;
		return 1;
	}

	/*
	 * Too short to carry a filter. Pass it: a frame we cannot evaluate is not a frame we know
	 * to be irrelevant, and Tier 0's guarantee is that a *negative* is exact. Dropping on
	 * "couldn't tell" would turn a false-positive-only filter into one with false negatives,
	 * which is precisely the property the design forbids.
	 */
	if (len < NDR_MIN_HDR) {
		ndr_stats.short_frame++;
		ndr_stats.passed++;
		return 1;
	}

	ndr_filter_from_hdr(&f, data);

	/*
	 * Frames from ordinary 802.11 devices. Our senders always set I/G=group and U/L=local in
	 * octet 0 (tier0 to_wire()), so a frame without both bits set was not produced by us.
	 *
	 * This is off by default because during bring-up we want beacons and neighbours visible --
	 * and because on a shared channel this is the difference between a monitor that sees the
	 * band and one that sees only itself. Turn it on for the energy/USB measurement, where
	 * "everything that is not ours" is exactly the traffic we are trying not to pay for.
	 */
	if (ndr_cfg.drop_foreign &&
	    (f.b[0] & NDR_RESERVED_MASK0) != NDR_RESERVED_MASK0) {
		ndr_stats.dropped_foreign++;
		return 0;
	}

	/*
	 * No registered prefixes means nothing to filter against -- pass everything rather than
	 * silently blackholing the link. A misconfigured node that hears nothing is far harder to
	 * diagnose than one that is merely not filtering.
	 */
	if (ndr_cfg.n_masks == 0) {
		ndr_stats.passed++;
		return 1;
	}

	for (i = 0; i < ndr_cfg.n_masks && i < NDR_MAX_MASKS; i++) {
		if (ndr_may_match(&f, &ndr_cfg.masks[i])) {
			ndr_stats.passed++;
			return 1;
		}
	}

	ndr_stats.dropped_filter++;
	return 0;
}
