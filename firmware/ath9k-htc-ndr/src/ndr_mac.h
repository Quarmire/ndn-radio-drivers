/*
 * Hardware-scheduled transmit on the AR9271 — the named airtime lease, enforced by the MAC.
 *
 * `named-filter-mac-redesign.md` §8.5 lists "no hardware-scheduled TX" as the single biggest cost
 * of commodity Wi-Fi — *larger than any filter improvement* — because a slot has to be approximated
 * with a host-side sleep plus EDCCA-off, which is why guard bands must be milliseconds rather than
 * microseconds. This part does have the primitive:
 *
 *   AR_TSF_L32 / AR_TSF_U32  0x804c / 0x8050  the hardware TSF the schedule keys on
 *   AR_QUIET1                0x80fc           next-quiet TSF (TU) + enable
 *   AR_QUIET2                0x8100           quiet period (TU) | quiet duration (TU)
 *
 * The MAC gates its own transmissions during a quiet window. Nothing on the host is involved, so
 * the "guard band" is whatever the hardware's own timing error is rather than a scheduler's.
 *
 * ⚠ **Granularity is 1 TU = 1024 µs.** Every field above counts TU, so a lease boundary can only be
 * placed on a ~1 ms grid. That is enough to enforce a millisecond-scale lease and *not* enough for
 * the sub-millisecond base slots the design assumes (the nRF54L15 testbed measured ~40–60 µs
 * scheduled-TX guards). This is a real limit of this part, and it should be reported as such rather
 * than papered over — see the note in the README on what a finer mechanism would require.
 *
 * ⚠ A MAC reset clears these registers, and the host driver resets on every channel change. Arming
 * once at attach is therefore not enough; see ndr_quiet_rearm().
 */

#ifndef _NDR_MAC_H_
#define _NDR_MAC_H_

#include "ndr_tier0.h"

/*
 * The field definitions are present in ar5416reg.h but sit inside an `#if 0` — only the register
 * addresses survive, because the open firmware never used quiet time. Re-supply them here rather
 * than editing upstream's header, guarded so they fold away if it is ever re-enabled.
 */
#ifndef AR_QUIET1_NEXT_QUIET_M
#define AR_QUIET1_NEXT_QUIET_M   0x0000ffff  /* TSF of next quiet period (TU) */
#endif
#ifndef AR_QUIET1_QUIET_ENABLE
#define AR_QUIET1_QUIET_ENABLE   0x00010000
#endif
#ifndef AR_QUIET2_QUIET_PERIOD_M
#define AR_QUIET2_QUIET_PERIOD_M 0x0000ffff  /* periodicity (TU) */
#endif

/* How far ahead of "now" to place the first quiet window, in microseconds. */
#ifndef NDR_QUIET_MARGIN_US
#define NDR_QUIET_MARGIN_US 4096
#endif

/*
 * ── The named airtime lease ──────────────────────────────────────────────────
 *
 * The design's actual claim: a transmit grant is a lease of base slots held by a NAME, computed as
 * f(name, clock) with no negotiation. Everything above this point demonstrates the *mechanism*
 * (the MAC will gate TX against its TSF); this is the mechanism keyed on a name.
 *
 *   slot   = H(registered prefix) mod NDR_LEASE_SLOTS
 *   period = NDR_LEASE_SLOTS * NDR_LEASE_SLOT_TU        (the node owns 1 slot in every period)
 *   quiet  = the whole period EXCEPT this node's slot
 *
 * Reserved lanes are implicit and never announced — every node computes the same schedule from the
 * same name, which is the property that makes coexistence work without a coordinator.
 *
 * ⚠ **Power-of-two geometry is mandatory, not stylistic.** MAGPIE is configured with
 * XCHAL_HAVE_DIV32 = 0 and this firmware links no libgcc, so `%` on a u32 would emit an undefined
 * reference to __umodsi3. Every modulo here is a mask, which requires the slot count and the period
 * in microseconds to both be powers of two. Defaults: 4 slots x 8 TU = 32768 us exactly.
 */
#ifndef NDR_LEASE_SLOTS
#define NDR_LEASE_SLOTS 4              /* power of two */
#endif
#ifndef NDR_LEASE_SLOT_TU
#define NDR_LEASE_SLOT_TU 8            /* 8 TU = 8192 us, so period = 32768 us = 2^15 */
#endif
#define NDR_LEASE_SLOT_US   ((a_uint32_t)NDR_LEASE_SLOT_TU * 1024u)
#define NDR_LEASE_PERIOD_US (NDR_LEASE_SLOT_US * NDR_LEASE_SLOTS)
#define NDR_LEASE_SLOT_MASK (NDR_LEASE_SLOTS - 1u)
#define NDR_LEASE_PERIOD_MASK (NDR_LEASE_PERIOD_US - 1u)

/* The prefix whose lease this node holds. Empty = lease disabled, fall back to the fixed schedule. */
#ifndef NDR_LEASE_PREFIX
#define NDR_LEASE_PREFIX ""
#endif

/* Build-time lease shape, in TU. Zero period disables the whole mechanism. */
#ifndef NDR_QUIET_PERIOD_TU
#define NDR_QUIET_PERIOD_TU 0
#endif
#ifndef NDR_QUIET_DURATION_TU
#define NDR_QUIET_DURATION_TU 0
#endif

/* Observability, read from the host with WMI_ACCESS_MEMORY. */
struct ndr_mac_state {
	a_uint32_t magic;
	a_uint32_t arm_count;    /* times quiet time has been (re)armed */
	a_uint32_t quiet1;       /* last value written to AR_QUIET1 */
	a_uint32_t quiet2;       /* last value written to AR_QUIET2 */
	a_uint32_t quiet1_rb;    /* read back, to prove the write stuck */
	a_uint32_t quiet2_rb;
	a_uint32_t quiet1_pre;   /* AR_QUIET1 as read BEFORE writing — distinguishes "hardware
				  * rejected the write" from "the register reads as garbage" */
	a_uint32_t tsf_lo;       /* TSF sampled at the last arm — proves the clock runs */
	a_uint32_t tsf_hi;
	a_uint32_t ifs_misc_rb;  /* AR_D_GBL_IFS_MISC read back (backoff-disable measurement mode) */
	a_uint32_t lease_slot;   /* slot index this node computed from its prefix */
	a_uint32_t timer_mode_rb; /* AR_TIMER_MODE read back — the enable that actually matters */
};

#define NDR_MAC_MAGIC 0x4E445231  /* "NDR1" */

extern struct ndr_mac_state ndr_mac_state;

/*
 * Non-zero once AR_QUIET1 has been written AND read back with the enable bit set.
 *
 * ndr_mac_state cannot be read from the host while ath9k_htc owns the device (that needs our
 * userspace driver, which cannot bring the PHY up, so the receive path never runs and nothing ever
 * gets armed). This turns the unreadable register readback into the one signal already proven
 * observable in M1: frames arriving, or not.
 */
a_uint32_t ndr_quiet_is_armed(void);

/*
 * Arm the quiet schedule if it is configured and not already active.
 *
 * Cheap enough to call from the receive path: it reads one register and returns immediately when
 * the schedule is already in place. Calling it there is deliberate — it is the only way to survive
 * the host driver's channel-change resets without adding a timer.
 */
void ndr_quiet_rearm(void);

#endif /* _NDR_MAC_H_ */
