/*
 * The TimeToken — common-view time carried by ordinary named data.
 *
 * Design: `ndn-face-monitor-wifi/docs/timing-rides-named-data.md`. Time is not a frame type, it is
 * an attribute any transmission can carry: if a frame reports the transmitter's TSF at the moment
 * it left the antenna, then **every named-data transmission is a timing reference for everyone who
 * overhears it** — no beacon, no timekeeper role, no announced schedule, and zero extra airtime.
 *
 * ⚠ Explicitly NOT an infrastructure beacon. An AP beacon is an AP asserting "I am the timekeeper
 * for this BSS", which is exactly the infrastructure the addressing doctrine removes. The campus-AP
 * common-view measurement in #41 was an *instrument* to prove the RX path resolves µs, never a
 * source to build on.
 *
 * ## Why this part, and why two-step
 *
 * The RTL8822E can only insert a TSF from its beacon engine, so "any data frame carries a
 * TimeToken" is unreachable there; the timing doc names two honest routes out, and one is
 * "different silicon with a TX-descriptor timestamp". The AR9271 is that silicon: its TX status
 * descriptor carries `AR_SendTimestamp`, already decoded into `ds_txstat.ts_tstamp`
 * (`ar5416_hw.c:758`), for **every** frame rather than only beacons.
 *
 * It is a *report*, not an insertion, so this is PTP's two-step shape: frame N is sent, the hardware
 * reports when it aired, and frame N+1 carries "sequence N left at TSF T". A receiver that latched
 * its own RX TSF for frame N now holds two hardware clock reads of one shared on-air event, and
 * differencing them cancels TX latency and propagation.
 *
 * ## Wire format, at body offset NDR_TT_OFF
 *
 *   u32 magic 'NDTT' | u16 ref_seq | u16 pad | u32 tx_tsf
 *
 * Big-endian, like the rest of this firmware's host protocol. `ref_seq` is the 802.11 sequence the
 * hardware actually transmitted (`ts_seqnum`), not the one the host asked for — the MAC may assign
 * its own, and the receiver reads the transmitted value.
 *
 * ⚠ The token is a clock reading, not a name or an address. It says "my TSF was X", nothing about
 * who "I" am; the transmitter stays keyed only by its ephemeral source nonce.
 */

#ifndef _NDR_TIME_H_
#define _NDR_TIME_H_

#include "ndr_tier0.h"

#define NDR_TT_MAGIC 0x4e445454u /* 'NDTT' */
#define NDR_TT_OFF   24          /* body offset: just past the 802.11 MAC header */
#define NDR_TT_LEN   12

/* Record what the hardware reported for a completed transmission. */
void ndr_time_note_tx(a_uint32_t seq, a_uint32_t tsf);

/* Stamp an outgoing frame with the most recent completed transmission's (seq, TSF). */
void ndr_time_stamp_frame(a_uint8_t *data, a_uint32_t len);

struct ndr_time_state {
	a_uint32_t magic;
	a_uint32_t last_seq;
	a_uint32_t last_tsf;
	a_uint32_t noted;    /* completions observed */
	a_uint32_t stamped;  /* frames stamped */
	a_uint32_t advanced; /* completions whose TSF differed from the previous one */
};

#define NDR_TIME_MAGIC 0x4e445432u /* "NDR2" */

extern struct ndr_time_state ndr_time_state;

/* Non-zero once the reported send times look like a live TSF in the MAC's own domain. */
a_int32_t ndr_time_plausible(void);

#endif /* _NDR_TIME_H_ */
