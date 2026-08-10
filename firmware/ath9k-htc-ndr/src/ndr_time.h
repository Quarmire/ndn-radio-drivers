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
 *   u32 magic 'NDTT' | u32 my_idx | u32 ref_idx | u32 ref_tsf
 *
 * Big-endian. The correlator is a FIRMWARE-side counter, not the 802.11 sequence: `ts_seqnum` reads
 * 0 on this path because `EN_HWSEQ` is deliberately clear, so the hardware assigns no sequence and
 * a token had no way to say which transmission its timestamp belonged to. Measured consequence —
 * completions are batched, the reference lag varied per frame, and a lag scan found no tight fit
 * (best residual sd 1550 µs, about one frame interval, against a per-sample change of ~60 µs).
 *
 * Each outgoing frame therefore carries its own `my_idx`, and the index travels on the tx buffer so
 * that the completion path can record (`my_idx`, `ts_tstamp`). A receiver builds `my_idx -> its own
 * RX TSF` from every frame it hears and pairs `ref_idx` exactly, with no lag to guess at.
 *
 * ## Measured, two nodes, ch13, 20 s, 17869 frames at ~890/s
 *
 * 16699 exact pairings (96.6% of frames overheard). The two TSFs differ by +0.98 ppm; removing that
 * linear rate leaves a residual of **sd 1.05 us, max 3.5 us** — microsecond common view between two
 * independent radios, carried entirely by ordinary data frames, with no beacon and no extra airtime.
 *
 * The counter is doing the work, not the arithmetic: shifting `ref_idx` by one in either direction
 * takes the residual from 1.05 us to 623 us / 3125 us. That cliff is the control, and it is also why
 * the earlier `ts_seqnum` version failed — with every sequence reading 0 there was nothing to pair
 * on, and the best lag-scan fit was sd 1550 us, about one frame interval.
 *
 * ⚠ Bootstrap subtlety: stamping is gated on having seen a completion, and the first frame out is
 * necessarily unstamped. So `ndr_time_note_tx()` must count an `idx == 0` completion even though it
 * refuses to make it the reference. Gating the note itself on a non-zero index deadlocks the whole
 * mechanism — measured as zero tokens on air.
 *
 * ⚠ The token is a clock reading, not a name or an address. It says "my TSF was X", nothing about
 * who "I" am; the transmitter stays keyed only by its ephemeral source nonce.
 */

#ifndef _NDR_TIME_H_
#define _NDR_TIME_H_

#include "ndr_tier0.h"

#define NDR_TT_MAGIC 0x4e445454u /* 'NDTT' */
#define NDR_TT_OFF   24          /* body offset: just past the 802.11 MAC header */
#define NDR_TT_LEN   16

/*
 * ── From a measurement to a clock ────────────────────────────────────────────
 *
 * Everything above establishes that two nodes can *observe* their clock relationship. Using it
 * needs one more step, because the lease schedule is `f(name, clock)` and two free-running TSFs are
 * not one clock: the two-node lease measured each node confined to its own name-derived window, but
 * the windows overlapped 26.2% purely because nothing tied the two time bases together.
 *
 * So each node carries a software offset and works in `common = local TSF + offset`. The token
 * publishes *common* time rather than raw TSF, which is what makes the offsets composable: what a
 * receiver reads out of a token is already in the sender's common frame.
 *
 * The merge rule is **converge to the midpoint**: each node steps half of whatever disagreement it
 * observes. No node is designated a timekeeper, no role is announced, every node runs the same
 * rule, and it is soft state in the §7 sense — a node that stops hearing anyone simply keeps its
 * offset and free-runs. That is the property that matters for the doctrine, which rules out exactly
 * the "I am the timekeeper for this BSS" assertion an AP beacon makes.
 *
 * ⚠ It is NOT the IBSS "adopt whoever is ahead" rule. That was tried and is measured unstable on a
 * 32-bit counter — see the long note at the merge site in ndr_time.c. Two nodes ~2.02e9 µs apart
 * leapfrogged each other several times per 30 s run, because at a separation near 2^31 adopting
 * re-creates the separation rather than removing it.
 *
 * ⚠ The residual bias is RX latency: a receiver compares the peer's *send* time against its own
 * *receive* time for the same frame, so delta carries a systematic ~20 µs offset. Steering on it
 * forever would walk every clock backwards, so corrections stop inside a deadband of 1 TU — big
 * enough to swallow the bias and the crystals' ~1 ppm, small enough to be invisible to an 8192 µs
 * slot boundary.
 */

/* Stop correcting once the disagreement is this small. See the merge site for why it must exceed
 * the RX-latency bias by a good margin. 1 TU: 8x the measured bias, 1/8 of a base slot. */
#define NDR_TIME_DEADBAND_US 1024

/* common time = local TSF + this. Signed, and wraps with the TSF. */
extern a_int32_t ndr_time_offset;

/* This node's common-time reading. This, not AR_TSF_L32, is what a schedule keys on. */
a_uint32_t ndr_time_now(void);

/*
 * Consume a received frame: record its index against our own RX TSF, and if it references an
 * earlier frame we also heard, merge clocks. `rx_tsf` is the hardware AR_RcvTimestamp, local domain.
 */
void ndr_time_rx(const a_uint8_t *data, a_uint32_t len, a_uint32_t rx_tsf);

/* Record what the hardware reported for a completed transmission. */
void ndr_time_note_tx(a_uint32_t idx, a_uint32_t tsf);

/* Stamp an outgoing frame with the most recent completed transmission's (index, common time). */
/* Stamps the frame and returns the index assigned to it (0 if it was not stamped). */
a_uint32_t ndr_time_stamp_frame(a_uint8_t *data, a_uint32_t len);

/*
 * Heard-frame ring, so a token's `ref_idx` can be matched to our own RX TSF for that same frame.
 * Power of two: MAGPIE has no divide. 64 entries is ~70 ms of history at the rates measured, which
 * is far more than the one-frame reference lag needs.
 */
#define NDR_TT_RING 64

struct ndr_time_state {
	a_uint32_t magic;
	a_uint32_t next_idx;  /* counter handed to the next outgoing frame */
	a_uint32_t last_idx;  /* index of the most recent completion */
	a_uint32_t last_tsf;
	a_uint32_t noted;    /* completions observed */
	a_uint32_t stamped;  /* frames stamped */
	a_uint32_t advanced; /* completions whose TSF differed from the previous one */
	a_uint32_t rx_tokens; /* tokens parsed out of received frames */
	a_uint32_t paired;    /* references matched to a frame we had heard ourselves */
	a_uint32_t merges;    /* times we adopted a peer's clock */
	a_int32_t  last_delta; /* most recent (peer common - our common), µs */
};

#define NDR_TIME_MAGIC 0x4e445432u /* "NDR2" */

extern struct ndr_time_state ndr_time_state;

/* Non-zero once the reported send times look like a live TSF in the MAC's own domain. */
a_int32_t ndr_time_plausible(void);

#endif /* _NDR_TIME_H_ */
