/*
 * Tier-0 receive policy for the AR9271 firmware -- the thing that makes a frame not cross USB.
 *
 * See ndr_tier0.h for why this runs on the dongle rather than the host.
 */

#ifndef _NDR_FILTER_H_
#define _NDR_FILTER_H_

#include "ndr_tier0.h"

/* Registered prefixes this node filters for. Small on purpose: the scan is linear and runs in the
 * RX path. §8.3 of the design describes the bitsliced layout that removes this limit when we need
 * more than a handful. */
#define NDR_MAX_MASKS   8

/*
 * Most usable bits a LEGITIMATE sender can set: one position per hash, per inserted prefix.
 *
 * A sender inserts at most NDR_MAX_DEPTH prefixes at NDR_K positions each, so popcount <= K*D = 32
 * (measured 29 at the cap, because positions collide). Anything denser was not produced by our
 * Tier-0 insert, so rejecting it has **zero false negatives** — the bound is exact, not heuristic.
 *
 * This matters more than it looks. A Bloom test asks "are all the mask's bits set?", which the
 * all-ones broadcast address ff:ff:ff:ff:ff:ff satisfies for EVERY mask. Without this check every
 * beacon, ARP and multicast frame on the channel is a guaranteed Tier-0 false positive, and on a
 * real channel that is a large share of the traffic. The published 99.06% rejection figure was
 * measured against random names, not against the degenerate address patterns actually on air.
 */
#define NDR_MAX_SET_BITS  (NDR_K * NDR_MAX_DEPTH)

#define NDR_CFG_MAGIC   0x4E445230  /* "NDR0" -- lets the host locate/verify this struct */

/*
 * Runtime configuration, in a single struct at a known symbol so the host can rewrite it in place
 * via WMI_ACCESS_MEMORY_CMDID (the firmware already dispatches that to dispatch_magpie_sys_cmds)
 * without a firmware rebuild. Until that channel is wired, these are compile-time defaults.
 */
struct ndr_cfg {
	a_uint32_t   magic;
	a_uint32_t   enabled;      /* 0 = stock behaviour: every frame goes to the host */
	a_uint32_t   drop_foreign; /* drop frames whose addr1 is not locally-administered group */
	a_uint32_t   n_masks;
	a_uint8_t    key[NDR_KEY_LEN]; /* group key = the trust context; must match the sender's */
	ndr_filter_t masks[NDR_MAX_MASKS];
};

/* Counters. Read back with WMI_ACCESS_MEMORY_CMDID; `dropped` is the number of USB transfers and
 * host wakeups that did not happen, which is the quantity §8.2 says we cannot get on any other
 * Wi-Fi part we own. */
struct ndr_stats {
	a_uint32_t seen;
	a_uint32_t passed;
	a_uint32_t dropped_filter;
	a_uint32_t dropped_foreign;
	a_uint32_t short_frame;
	a_uint32_t dropped_popcount;
};

extern struct ndr_cfg   ndr_cfg;
extern struct ndr_stats ndr_stats;

/*
 * Returns non-zero if this frame should be sent to the host.
 *
 * `data`/`len` are the raw 802.11 frame as received. Called from ath_tgt_rx_tasklet() on the
 * target, before the HTC/USB handoff.
 */
a_int32_t ndr_rx_accept(const a_uint8_t *data, a_uint32_t len);

#endif /* _NDR_FILTER_H_ */
