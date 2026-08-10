/* Hardware-scheduled TX via the MAC's quiet-time registers. See ndr_mac.h. */

#include <adf_os_io.h>

#include "ar5416reg.h"
#include "ndr_mac.h"
#include "ndr_filter.h"
#include "ndr_ctl.h"

static a_uint32_t armed_ok;

/* log2 of a power of two. 32-bit shifts are native on Xtensa; only 64-bit variable shifts would
 * need a libgcc helper this firmware does not link. */
static a_uint32_t ndr_log2(a_uint32_t v)
{
	a_uint32_t n = 0;

	while (v > 1u) {
		v >>= 1;
		n++;
	}
	return n;
}

/* Has the epoch rotated us into a different slot than the one currently armed? */
static a_int32_t ndr_lease_slot_changed(void)
{
	a_uint32_t slots = ndr_ctl_lease_override ? ndr_ctl_lease_slots : NDR_LEASE_SLOTS;
	a_uint32_t slot_tu = ndr_ctl_lease_override ? ndr_ctl_lease_slot_tu : NDR_LEASE_SLOT_TU;
	a_uint32_t per_us = slot_tu * 1024u * slots;
	a_uint32_t epoch_idx;

	if (!slots)
		return 0;
	epoch_idx = ioread32_mac(AR_TSF_L32) >> ndr_log2(per_us);
	return epoch_idx != ndr_mac_state.lease_epoch;
}

struct ndr_mac_state ndr_mac_state = { NDR_MAC_MAGIC, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };

void ndr_quiet_disarm(void)
{
	armed_ok = 0;
	iowrite32_mac(AR_TIMER_MODE, ioread32_mac(AR_TIMER_MODE) & ~AR_QUIET_TIMER_EN);
}

void ndr_quiet_rearm(void)
{
	a_uint32_t q1, q2, tsf_lo, tsf_hi;
	a_uint32_t period_us = (a_uint32_t)NDR_QUIET_PERIOD_TU * 1024u;
	a_uint32_t slot_tick_us = 1024u;
	a_uint32_t duration_tu = NDR_QUIET_DURATION_TU;
	a_uint32_t next_us;

	if (ndr_ctl_quiet_off)
		return;
	if (NDR_QUIET_PERIOD_TU == 0 && !ndr_ctl_lease_override)
		return;

	/*
	 * Arm once, then only re-arm if the hardware lost it (a MAC reset on channel change).
	 *
	 * The unconditional version was wrong in a way that looked like a hardware property: this
	 * runs from the receive path, ~100x/s on a busy channel, and each call reset
	 * AR_NEXT_QUIET_TIMER to "now + margin", continually restarting the quiet window. Measured
	 * effect: ~98% gating when 50% was configured. It is the software, not the duration field.
	 *
	 * The guard is on AR_TIMER_MODE, which is measured to read back reliably, plus a software
	 * flag so the very first arm always happens. Guarding on AR_QUIET1 bit 16 does NOT work —
	 * that register never reads back on this part, which is what made the first attempt skip
	 * the write entirely.
	 */
	/*
	 * Re-arm when the hardware lost the schedule OR when the epoch has rotated our slot.
	 *
	 * The quiet timer repeats at a FIXED phase, so it cannot express a slot that moves each
	 * period — the firmware has to re-arm once per epoch. This runs from the receive and
	 * transmit paths, which is what a node with traffic has; a node that is completely silent
	 * will not rotate until it next sends or hears something. The clean fix is an interrupt off
	 * the generic-timer block rather than piggybacking on traffic.
	 */
	if (armed_ok && (ioread32_mac(AR_TIMER_MODE) & AR_QUIET_TIMER_EN) &&
	    !ndr_lease_slot_changed())
		return;

	/*
	 * (No guard on AR_QUIET1 here.)
	 *
	 * The first version guarded on `ioread32_mac(AR_QUIET1) & AR_QUIET1_QUIET_ENABLE`, which is
	 * self-defeating: if that register reads back as anything with bit 16 set — including the
	 * all-ones a MAC register returns when the block is unclocked — the guard skips the write
	 * and the readback stays 0, which is indistinguishable from "the hardware rejected it".
	 * Measured: it made the probe report not-armed. Re-writing every time is cheap.
	 */
	ndr_mac_state.quiet1_pre = ioread32_mac(AR_QUIET1);

	/*
	 * Read the TSF and place the first quiet window a little ahead of now, so we never arm a
	 * start time that has already passed (which would defer the schedule by a full wrap of the
	 * 16-bit TU field, ~65 s).
	 *
	 * TSF counts microseconds; the quiet fields count TU (1024 µs), hence the >> 10.
	 */
	tsf_hi = ioread32_mac(AR_TSF_U32);
	tsf_lo = ioread32_mac(AR_TSF_L32);
	next_us = tsf_lo + NDR_QUIET_MARGIN_US;

	{
		/*
		 * Name-keyed lease with the epoch term:
		 *
		 *     owner(t) = ( H(name-group) + epoch(t) )  mod N          [time-slice-mac.md]
		 *
		 * The base slot is a pure function of the name; the epoch term rotates every node's
		 * slot by one per period. Both are computed, never announced.
		 *
		 * ⚠ Rotation is about FAIRNESS, not collision avoidance. Adding the same epoch to
		 * every name preserves their relative offsets, so two names that hash to the same
		 * slot still collide — for that they need different hashes, or the within-slot CCLF
		 * election. What rotation buys is that no name is permanently stuck in a particular
		 * slot, which matters because slots are not interchangeable: §5 of the filter/MAC
		 * redesign reserves every R-th slot, and a fixed assignment would permanently
		 * advantage or starve whoever landed there.
		 */
		static const char lease_prefix[] = NDR_LEASE_PREFIX;
		a_uint32_t base = 0, slots = 0, slot_us = 0, per_us = 0;

		if (ndr_ctl_lease_override) {
			/* Runtime lease over the control path: the host supplies the BASE slot (what
			 * the name would hash to); the epoch term is still applied here so both nodes
			 * rotate identically. */
			base    = ndr_ctl_lease_slot;
			slots   = ndr_ctl_lease_slots;
			slot_us = ndr_ctl_lease_slot_tu * 1024u;
		} else if (sizeof(lease_prefix) > 1) {
			base = (a_uint32_t)ndr_name_hash(ndr_cfg.key,
							 (const a_uint8_t *)lease_prefix,
							 (a_uint32_t)(sizeof(lease_prefix) - 1))
			       & NDR_LEASE_SLOT_MASK;
			slots   = NDR_LEASE_SLOTS;
			slot_us = NDR_LEASE_SLOT_US;
		}

		if (slots) {
			a_uint32_t epoch_idx, slot, next_slot, slot_start, slot_end, next_start;

			per_us = slot_us * slots;
			/* Period is a power of two by construction, so the epoch INDEX is a shift and
			 * the alignment is a mask -- no 32-bit divide (MAGPIE has no DIV32). */
			epoch_idx  = tsf_lo >> ndr_log2(per_us);
			slot       = (base + epoch_idx) & (slots - 1u);
			next_slot  = (base + epoch_idx + 1u) & (slots - 1u);
			slot_start = (epoch_idx * per_us) + slot * slot_us;
			slot_end   = slot_start + slot_us;
			next_start = ((epoch_idx + 1u) * per_us) + next_slot * slot_us;

			/*
			 * ★ Only re-arm while inside our OWN slot.
			 *
			 * Writing AR_NEXT_QUIET_TIMER clears whatever quiet window is in progress. Doing
			 * that at the epoch boundary — where the rotation is detected — ends the quiet
			 * period early and hands the node every slot up to its new one. Measured: duty
			 * 40% against a configured 25%. Re-arming inside our own slot is free, because
			 * we are entitled to transmit then anyway; and the TX path naturally runs there,
			 * since that is the only time this node can send.
			 */
			/*
			 * Enforced only once we are armed. The FIRST arm has to happen wherever we
			 * happen to be, or nothing ever arms the quiet window or the tick that drives
			 * every later rotation — and with the activity-driven re-arms removed that is a
			 * deadlock: measured as no gating at all (936 f/s, full rate). Arming from an
			 * arbitrary phase is harmless because nothing is gated yet; the first tick
			 * corrects it.
			 */
			if (armed_ok && (tsf_lo < slot_start || tsf_lo >= slot_end))
				return;

			ndr_mac_state.lease_slot  = slot;
			ndr_mac_state.lease_base  = base;
			ndr_mac_state.lease_epoch = epoch_idx;

			/*
			 * Quiet runs from the end of our slot to the start of the next one. With the
			 * epoch term that gap is a whole period normally, and ZERO on the wrap
			 * (slot N-1 -> slot 0 are adjacent in time), where the node simply transmits two
			 * slots back to back. One slot per epoch either way, so the duty is unchanged.
			 */
			if (next_start <= slot_end) {
				ndr_quiet_disarm();
				return;
			}

			/*
			 * Quiet for (next_start - slot_end), repeating every per_us + slot_us.
			 *
			 * That period is the TRUE recurrence of a rotating slot: our window advances by
			 * one slot each epoch, so successive slot starts are per_us + slot_us apart. Set
			 * it and the hardware sustains the rotation on its own between re-arms; set it to
			 * per_us instead and the quiet window (which is a whole period long) repeats
			 * immediately, gating the node ~100% -- measured as a collapse to ~5% duty.
			 * The per-epoch re-arm then only has to handle the wrap.
			 */
			slot_tick_us = slot_us;
			period_us   = per_us + slot_us;
			next_us     = slot_end;
			duration_tu = (next_start - slot_end) / 1024u;
		}
	}

	q1 = (((next_us >> 10) + 0) & AR_QUIET1_NEXT_QUIET_M) | AR_QUIET1_QUIET_ENABLE;
	q2 = ((a_uint32_t)NDR_QUIET_PERIOD_TU & AR_QUIET2_QUIET_PERIOD_M) |
	     ((duration_tu << AR_QUIET2_QUIET_DURATION_S) & AR_QUIET2_QUIET_DURATION);

	/* Duration before enable, so the window is fully described the instant it goes live. */
	iowrite32_mac(AR_QUIET2, q2);
	iowrite32_mac(AR_QUIET1, q1);

	/*
	 * ★ AR_QUIET1/2 alone do nothing on this part — measured: bit 16 of AR_QUIET1 does not read
	 * back after being written. The live path is the generic-timer block, where the quiet timer
	 * has its own enable in AR_TIMER_MODE:
	 *
	 *   AR_NEXT_QUIET_TIMER 0x8218   when the next quiet window starts
	 *   AR_QUIET_PERIOD     0x8238   how often it repeats
	 *   AR_TIMER_MODE       0x8240   AR_QUIET_TIMER_EN (0x40) actually turns it on
	 *
	 * These are full 32-bit TSF-based registers rather than the 16-bit TU fields of AR_QUIET1/2,
	 * which is also why they matter beyond just working: TU granularity (1024 us) cannot express
	 * the sub-millisecond base slots the design wants, and these can.
	 */
	iowrite32_mac(AR_NEXT_QUIET_TIMER, next_us);
	iowrite32_mac(AR_QUIET_PERIOD, period_us);
	iowrite32_mac(AR_TIMER_MODE, ioread32_mac(AR_TIMER_MODE) | AR_QUIET_TIMER_EN);

	/*
	 * Arm the lease tick a little INTO our next slot, so the re-arm it triggers happens while we
	 * are entitled to transmit -- which is the condition ndr_quiet_rearm() requires, and the
	 * reason writing AR_NEXT_QUIET_TIMER there costs nothing.
	 */
	if (period_us > NDR_QUIET_MARGIN_US) {
		iowrite32_mac(AR_NEXT_DTIM, next_us + duration_tu * 1024u + (slot_tick_us / 4u));
		iowrite32_mac(AR_DTIM_PERIOD, period_us);
		iowrite32_mac(AR_TIMER_MODE, ioread32_mac(AR_TIMER_MODE) | AR_DTIM_TIMER_EN);
		iowrite32_mac(AR_IMR_S5, ioread32_mac(AR_IMR_S5) | NDR_TICK_TRIG_BIT);
		iowrite32_mac(AR_IMR, ioread32_mac(AR_IMR) | AR_IMR_GENTMR);
		ndr_mac_state.tick_armed++;
	}

	/*
	 * Optionally drop the random backoff.
	 *
	 * The on-air measurement of the quiet->transmit boundary bottoms out at ~52 us median, and
	 * that floor is CSMA, not the scheduler: when a quiet window ends the MAC still owes DIFS
	 * plus a random backoff before its first frame. Removing the backoff isolates how much of
	 * the residual is the hardware's own boundary placement.
	 *
	 * This is a MEASUREMENT mode, not a deployment setting -- a node that ignores backoff is
	 * antisocial on a shared channel. It belongs behind a build flag and behind a quiet channel.
	 * (Note the named airtime lease makes this less reckless than it sounds: inside a granted
	 * lease the node is supposed to own the medium, so backoff is the thing being replaced.)
	 */
#ifdef NDR_NO_BACKOFF
	iowrite32_mac(AR_D_GBL_IFS_MISC,
		      ioread32_mac(AR_D_GBL_IFS_MISC) | AR_D_GBL_IFS_MISC_IGNORE_BACKOFF);
	ndr_mac_state.ifs_misc_rb = ioread32_mac(AR_D_GBL_IFS_MISC);
#endif
	ndr_mac_state.timer_mode_rb = ioread32_mac(AR_TIMER_MODE);
	armed_ok = 1;

	ndr_mac_state.arm_count++;
	ndr_mac_state.quiet1 = q1;
	ndr_mac_state.quiet2 = q2;
	ndr_mac_state.quiet1_rb = ioread32_mac(AR_QUIET1);
	ndr_mac_state.quiet2_rb = ioread32_mac(AR_QUIET2);
	ndr_mac_state.tsf_lo = tsf_lo;
	ndr_mac_state.tsf_hi = tsf_hi;
}

a_uint32_t ndr_quiet_is_armed(void)
{
	/* The generic-timer enable is the one that means anything on this part. */
	return (ndr_mac_state.timer_mode_rb & AR_QUIET_TIMER_EN) ? 1 : 0;
}

void ndr_lease_tick(void)
{
	ndr_mac_state.tick_count++;
	ndr_quiet_rearm();
}

/*
 * Recovery path, called from RX/TX. NOT the rotation trigger -- that is the tick.
 *
 * A MAC reset (the host does one on every channel change) clears AR_TIMER_MODE, AR_IMR_S5 and
 * AR_IMR, which takes the lease tick with it. A timer cannot re-arm itself once the hardware has
 * forgotten it, so something outside the timer has to notice; traffic is what is available. This is
 * two register reads in the common case.
 */
void ndr_quiet_recover(void)
{
	if (armed_ok && (ioread32_mac(AR_TIMER_MODE) & AR_QUIET_TIMER_EN))
		return;

	/* Lost it. Arm from wherever we are -- nothing is gated, so the phase is free. */
	armed_ok = 0;
	ndr_quiet_rearm();
}
