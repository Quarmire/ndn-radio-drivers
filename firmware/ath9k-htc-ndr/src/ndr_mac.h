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

/*
 * ── The lease tick ───────────────────────────────────────────────────────────
 *
 * The quiet timer has no interrupt of its own, so the re-arm used to ride on RX/TX activity: a
 * silent node never rotated. The same timer block carries seven other timers, and the DTIM one is
 * unused in monitor mode (no beaconing), so it is co-opted as a periodic tick:
 *
 *   AR_NEXT_DTIM 0x8214  when the next tick fires (µs, TSF)
 *   AR_DTIM_PERIOD 0x8234  how often (µs)
 *   AR_TIMER_MODE  AR_DTIM_TIMER_EN (0x20)  turns it on
 * ⚠ The interrupt does NOT arrive via AR_ISR_S2/BCNMISC. Wiring it that way was tried and measured
 * dead — the probe build passed zero frames, meaning the handler never ran. These eight timers ARE
 * the generic timers (`AR_GEN_TIMERS(i) = 0x8200 + 4i`, so DTIM is index 5), and their interrupts
 * land in AR_ISR_S5, OR-ed into the primary status as AR_ISR_GENTMR:
 *
 *   trigger bit (1 << 5) in AR_ISR_S5, read via the shadow AR_ISR_S5_S
 *   gated by (1 << 5) in AR_IMR_S5 AND AR_IMR_GENTMR in the primary mask
 *
 * Nothing else in this firmware touches AR_IMR_S5 or AR_IMR_GENTMR, and the HAL rewrites AR_IMR
 * whenever interrupts are re-set, so both are re-applied on every arm.
 *
 * ⚠ This co-opts a beacon timer. Safe in monitor mode, which is what named-data radio runs; a node
 * that also wanted to beacon would need a different timer (TIM and NDP are equally free here).
 */
#ifndef AR_NEXT_DTIM
#define AR_NEXT_DTIM        0x8214
#endif
#ifndef AR_DTIM_PERIOD
#define AR_DTIM_PERIOD      0x8234
#endif
#ifndef AR_DTIM_TIMER_EN
#define AR_DTIM_TIMER_EN    0x00000020
#endif
/* DTIM is generic timer index 5 (AR_GEN_TIMERS(5) == AR_NEXT_DTIM == 0x8214). */
#define NDR_TICK_TIMER_IDX  5
#define NDR_TICK_TRIG_BIT   (1u << NDR_TICK_TIMER_IDX)

/* Shadow secondary status 5. The header documents RAC=0xc0, S0_S=0xc4, S1_S=0xc8, S2_S=0xcc, so
 * the fifth is 0xd8. The shadow must be used, not AR_ISR_S5: AR_ISR_RAC is read-and-clear. */
#ifndef AR_ISR_S5_S
#define AR_ISR_S5_S         0x00d8
#endif
#ifndef AR_IMR_GENTMR
#define AR_IMR_GENTMR       0x10000000
#endif

/* How far ahead of "now" to place the first quiet window, in microseconds. Used by the fixed
 * (non-lease) schedule, which is armed once and then repeats in hardware. */
#ifndef NDR_QUIET_MARGIN_US
#define NDR_QUIET_MARGIN_US 4096
#endif

/*
 * Arming margin for the LEASE schedule, which is re-armed every epoch so the slot can rotate.
 *
 * This must be small. The margin exists only to avoid arming a boundary that the TSF has already
 * passed; anything larger makes the "already passed, take the next period" rule skip the current
 * epoch's quiet window entirely. Measured with the 4096 us margin inherited from the fixed
 * schedule: duty came out 41% instead of the configured 25%, because roughly one epoch in three
 * was being skipped. At 256 us the skip only happens if the boundary lands inside the arming
 * write itself.
 */
#ifndef NDR_LEASE_ARM_MARGIN_US
#define NDR_LEASE_ARM_MARGIN_US 256
#endif

/*
 * ── The named airtime lease ──────────────────────────────────────────────────
 *
 * The design's actual claim: a transmit grant is a lease of base slots held by a NAME, computed as
 * f(name, clock) with no negotiation. Everything above this point demonstrates the *mechanism*
 * (the MAC will gate TX against its TSF); this is the mechanism keyed on a name.
 *
 *   slot   = ( H(registered prefix) + epoch(t) ) mod NDR_LEASE_SLOTS
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
	a_uint32_t lease_slot;   /* slot currently armed = (base + epoch) mod N */
	a_uint32_t lease_base;   /* H(prefix) mod N -- the name's slot before rotation */
	a_uint32_t lease_epoch;  /* epoch index the armed slot was computed for */
	a_uint32_t tick_armed;   /* times the lease tick has been armed */
	a_uint32_t tick_count;   /* lease-tick interrupts serviced */
	a_uint32_t timer_mode_rb; /* AR_TIMER_MODE read back — the enable that actually matters */
	a_int32_t  time_offset;   /* the common-time offset the armed schedule was computed with */
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

/* Force the next ndr_quiet_rearm() to re-apply. Called when runtime config changes the shape. */
void ndr_quiet_disarm(void);

/*
 * Called from the interrupt path when the lease tick fires. Rotating the slot is the whole job, so
 * this is just ndr_quiet_rearm() under a name that documents where it comes from.
 */
void ndr_lease_tick(void);

/* Recovery only: re-arms if a MAC reset has wiped the schedule. Rotation is the tick's job. */
void ndr_quiet_recover(void);

#endif /* _NDR_MAC_H_ */
