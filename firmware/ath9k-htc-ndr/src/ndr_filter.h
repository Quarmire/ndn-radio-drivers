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
	a_uint64_t   key;          /* keyed name hash; must match the sender's */
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
