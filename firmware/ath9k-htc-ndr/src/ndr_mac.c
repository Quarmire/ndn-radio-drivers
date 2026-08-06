/* Hardware-scheduled TX via the MAC's quiet-time registers. See ndr_mac.h. */

#include <adf_os_io.h>

#include "ar5416reg.h"
#include "ndr_mac.h"

static a_uint32_t armed_ok;

struct ndr_mac_state ndr_mac_state = { NDR_MAC_MAGIC, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };

void ndr_quiet_rearm(void)
{
	a_uint32_t q1, q2, tsf_lo, tsf_hi;

	if (NDR_QUIET_PERIOD_TU == 0)
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
	if (armed_ok && (ioread32_mac(AR_TIMER_MODE) & AR_QUIET_TIMER_EN))
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

	q1 = (((tsf_lo >> 10) + 2) & AR_QUIET1_NEXT_QUIET_M) | AR_QUIET1_QUIET_ENABLE;
	q2 = ((a_uint32_t)NDR_QUIET_PERIOD_TU & AR_QUIET2_QUIET_PERIOD_M) |
	     (((a_uint32_t)NDR_QUIET_DURATION_TU << AR_QUIET2_QUIET_DURATION_S) &
	      AR_QUIET2_QUIET_DURATION);

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
	iowrite32_mac(AR_NEXT_QUIET_TIMER, tsf_lo + NDR_QUIET_MARGIN_US);
	iowrite32_mac(AR_QUIET_PERIOD, (a_uint32_t)NDR_QUIET_PERIOD_TU * 1024u);
	iowrite32_mac(AR_TIMER_MODE, ioread32_mac(AR_TIMER_MODE) | AR_QUIET_TIMER_EN);

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
