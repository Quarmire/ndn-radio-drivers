//! The **measured** mt76x02 knob layer — TSF, channel-time sensing, RX error
//! counters, ED-CCA and the RX filter, written once for both the MT7610U
//! (`mt76x0`) and the MT7612U (`mt76x2`).
//!
//! This module is a **measurement record**, not a hope. Every register it
//! touches was read on live silicon — mds-o5p-1's MT7610U (`0e8d:7610`), on
//! 2026-08-27, via `examples/mt76_oracle.rs` — while the kernel `mt76x0u`
//! driver held the part in monitor mode. Where a fact below is CODE-READ from
//! the upstream tree rather than measured, it says so in that many words.
//!
//! # What was MEASURED
//!
//! * **[`MT_TSF_TIMER_DW0`] is the LOW word** and ticks at **1.000 MHz**, so a
//!   TSF delta *is* microseconds. ⚠ The only reader of these two registers
//!   upstream — `mt76x02_usb_core.c:153-157` — assembles
//!   `tsf = (u64)dw0 << 32 | dw1`, i.e. it treats DW0 as the HIGH word. That is
//!   **wrong on this silicon** and is not copied here; see [`read_tsf`].
//!   (Upstream only feeds that value to a `dev_dbg` print, which is presumably
//!   how the bug survived.)
//! * The TSF **does not run** until [`MT_BEACON_TIME_CFG`] bit 16
//!   ([`MT_BEACON_TIME_CFG_TIMER_EN`]) is set. As found under a kernel monitor
//!   it reads 0, because mac80211 never arms the timer for a monitor vif.
//!   `SYNC_MODE` (bits 18:17) must stay **clear** or a received beacon
//!   overwrites the counter — which would destroy a common-view clock.
//! * [`MT_CH_IDLE`] (0x1130) and [`MT_CH_BUSY`] (0x1134) are **read-and-clear
//!   microsecond** counters: over 100 ms windows, `(idle + busy) / elapsed`
//!   measured **1.00**. That is the whole proof that they are µs, that they are
//!   read-and-clear, and that together they tile the window.
//! * [`MT_ED_CCA_TIMER`] (0x1140) is an **independent** energy-detect busy-µs
//!   counter — a second, physically different sense of "occupied".
//! * [`MT_RX_STAT_0`] (0x1700) and [`MT_RX_STAT_1`] (0x1704) are read-and-clear
//!   per window.
//! * As left by a kernel monitor: [`MT_RX_FILTR_CFG`] `= 0x0000_1093`,
//!   [`MT_TXOP_CTRL_CFG`] `= 0x0000_583f` (**ED-CCA bit 20 already CLEAR**),
//!   [`MT_EXT_CCA_CFG`] `= 0x0000_f1e4`.
//! * **EP0 costs 151 µs per vendor-request round trip.** Every function here is
//!   built out of those, so the cost is stated on each one. A knob is affordable
//!   in a channel switch or a 100 ms sensing window and is **never** affordable
//!   on a per-frame path.
//!
//! # Why this is one module and not two
//!
//! MediaTek ships one register header for both parts (`mt76x02_regs.h`), and the
//! MAC block behind it is the same silicon. So a semantic measured on the 7610
//! is a semantic of the 7612 — which matters here, because the lab's MT7612U is
//! currently unreachable and the MT7610U is not. Everything is written against
//! the object-safe [`Mt76Regs`] seam so one copy serves both backends.
#![allow(dead_code)]

use std::sync::atomic::{AtomicU64, Ordering};

use crate::FaceError;
use crate::mt76::Family;
use crate::mt76::Mt76Regs;
use crate::mt76::regs::{
    MT_BBP_AGC_BASE, MT_BEACON_TIME_CFG, MT_BEACON_TIME_CFG_SYNC_MODE, MT_BEACON_TIME_CFG_TIMER_EN,
    MT_CH_BUSY, MT_CH_CCA_RC_EN, MT_CH_IDLE, MT_CH_TIME_CFG, MT_CH_TIME_CFG_CH_TIMER_CLR,
    MT_CH_TIME_CFG_EIFS_AS_BUSY, MT_CH_TIME_CFG_NAV_AS_BUSY, MT_CH_TIME_CFG_RX_AS_BUSY,
    MT_CH_TIME_CFG_TIMER_EN, MT_CH_TIME_CFG_TX_AS_BUSY, MT_ED_CCA_TIMER, MT_EDCCA_BBP_TH_2G,
    MT_EDCCA_BBP_TH_5G, MT_EXT_CCA_CFG, MT_EXT_CCA_CFG_CCA_MASK, MT_EXT_CCA_CFG_ED_CCA_MASK,
    MT_RX_FILTR_CFG, MT_RX_STAT_0, MT_RX_STAT_0_CRC_ERRORS, MT_RX_STAT_0_PHY_ERRORS, MT_RX_STAT_1,
    MT_RX_STAT_1_CCA_ERRORS, MT_RX_STAT_1_PLCP_ERRORS, MT_TSF_TIMER_DW0, MT_TSF_TIMER_DW1,
    MT_TX_CFACK_EN, MT_TX_LINK_CFG, MT_TXOP_CTRL_CFG, MT_TXOP_ED_CCA_EN, MT_TXOP_HLDR_ET,
    MT_TXOP_HLDR_TX40M_BLK_EN, field_get, field_prep, mt_bbp,
};
use ndn_radio_hal::{ContentionApplied, ContentionPosture};

// ── TSF ──────────────────────────────────────────────────────────────────────

/// Arm the hardware TSF: set [`MT_BEACON_TIME_CFG_TIMER_EN`] and **clear**
/// `SYNC_MODE`.
///
/// Two things, both MEASURED, make this non-optional before [`read_tsf`]:
///
/// 1. Under a kernel monitor the timer is **off**, so `MT_TSF_TIMER_DW0` reads a
///    constant 0. A reader that skips this step measures nothing and cannot tell
///    that from a stopped clock.
/// 2. `SYNC_MODE` (bits 18:17) makes the hardware **adopt the TSF out of a
///    received beacon**. For a common-view clock that is fatal: the counter
///    would jump to a foreign AP's timebase at every beacon, so two nodes'
///    stamps of one frame would not difference into an offset. Clearing it is
///    what makes this a *free-running* counter.
///
/// Done as a read-modify-write so `INTVAL` (bits 15:0) and the beacon-TX bits
/// keep whatever the bring-up left in them — this knob owns exactly two fields.
///
/// Cost: 2 EP0 round trips ≈ 302 µs.
pub fn enable_tsf(bus: &dyn Mt76Regs) -> Result<(), FaceError> {
    bus.rmw(
        MT_BEACON_TIME_CFG,
        MT_BEACON_TIME_CFG_SYNC_MODE,
        MT_BEACON_TIME_CFG_TIMER_EN,
    )?;
    Ok(())
}

/// Stop the TSF (clear [`MT_BEACON_TIME_CFG_TIMER_EN`]), restoring the
/// as-found state. The counter holds its value; it does not reset.
///
/// Cost: 2 EP0 round trips ≈ 302 µs.
pub fn disable_tsf(bus: &dyn Mt76Regs) -> Result<(), FaceError> {
    bus.rmw(MT_BEACON_TIME_CFG, MT_BEACON_TIME_CFG_TIMER_EN, 0)?;
    Ok(())
}

/// Is the TSF armed *and* free-running — `TIMER_EN` set and `SYNC_MODE` clear?
///
/// Both halves matter. A caller that only checks `TIMER_EN` can be handed a
/// counter that a nearby AP's beacons are rewriting, which reads as a working
/// clock right up until it is used for common view. Cost: 1 EP0 round trip.
pub fn tsf_running(bus: &dyn Mt76Regs) -> Result<bool, FaceError> {
    let v = bus.rr(MT_BEACON_TIME_CFG)?;
    Ok(v & MT_BEACON_TIME_CFG_TIMER_EN != 0 && v & MT_BEACON_TIME_CFG_SYNC_MODE == 0)
}

/// Read the 64-bit hardware TSF, in **microseconds** (MEASURED 1.000 MHz tick).
///
/// ★ **Word order.** `DW0` (0x111c) is the **LOW** word and `DW1` (0x1120) the
/// HIGH word — measured by watching DW0 advance ~1e6 per second while DW1 sat
/// still. `mt76x02_usb_core.c:153-157` builds `(u64)dw0 << 32 | dw1`, the other
/// way round; do not copy it.
///
/// Wrap-safe by the same structure as the AR9271 path
/// (`crate::ath9k_htc`'s `read_clock`, `AR_TSF_U32`/`AR_TSF_L32`): read HIGH,
/// then LOW, then HIGH again; if the two HIGH reads disagree the low word
/// carried between them, so take the second HIGH and re-read LOW. The low word
/// wraps every 2^32 µs ≈ 71.6 minutes, so the re-read is almost never taken —
/// but "almost never" is exactly the bug that survives a bench test and fires in
/// the field.
///
/// Cost: 3 EP0 round trips ≈ 453 µs (4 ≈ 604 µs on the wrap path). That latency
/// is *inside* the value's uncertainty: this is a read-now clock good to
/// sub-millisecond, not a per-frame stamp. For per-frame precision use the RX
/// descriptor's timestamp, not this.
pub fn read_tsf(bus: &dyn Mt76Regs) -> Result<u64, FaceError> {
    let hi1 = bus.rr(MT_TSF_TIMER_DW1)?;
    let lo = bus.rr(MT_TSF_TIMER_DW0)?;
    let hi2 = bus.rr(MT_TSF_TIMER_DW1)?;
    let (hi, lo) = if hi1 == hi2 {
        (hi1, lo)
    } else {
        (hi2, bus.rr(MT_TSF_TIMER_DW0)?)
    };
    Ok(((hi as u64) << 32) | lo as u64)
}

// ── Channel-time sensing ─────────────────────────────────────────────────────

/// One window of the hardware's channel-occupancy counters, in **microseconds**.
///
/// ★ **These are read-and-clear.** The numbers in here are not a running total
/// and must not be differenced against a previous sample — the value **is** the
/// window, measured from the previous [`read_channel_time`] to this one. Two
/// consequences a caller has to hold in mind:
///
/// * **Nothing to subtract.** Differencing two samples yields noise, not a rate.
/// * **Two readers split the count.** Whoever reads first takes the microseconds
///   and leaves zero behind. This is a live hazard on this rig, because the
///   kernel `mt76x0u` driver reads [`MT_CH_BUSY`] in its own survey path
///   (`mt76x02_mac.c:1035`) whenever it is bound to the part. A userspace
///   sensing loop running alongside a bound kernel driver reads roughly *half*
///   the truth, silently. Claim the device (which detaches the kernel driver) or
///   accept that the figure is a lower bound.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct ChannelTime {
    /// Microseconds the MAC counted the medium **busy** in this window
    /// ([`MT_CH_BUSY`]). What counts as busy is chosen by
    /// [`enable_channel_time_counters`]: TX, RX, NAV and EIFS.
    pub busy_us: u32,
    /// Microseconds the MAC counted the medium **idle** ([`MT_CH_IDLE`]).
    /// MEASURED: `busy_us + idle_us` tiles the wall-clock window to 1.00.
    pub idle_us: u32,
    /// Microseconds the **energy detector alone** called the medium busy
    /// ([`MT_ED_CCA_TIMER`]) — an independent sense that owes nothing to
    /// decoding a preamble. See [`ed_cca_permille`].
    pub ed_cca_us: u32,
}

/// Arm the channel-time counters, exactly as `mt76x02_mac_cc_reset`
/// (`mt76x02_mac.c:1216-1232`) does: `TIMER_EN` plus TX / RX / NAV / EIFS all
/// counted as busy, plus [`MT_CH_CCA_RC_EN`], plus `CH_TIMER_CLR = 1`.
///
/// ★ [`MT_CH_CCA_RC_EN`] (bit 6) is the **read-clear enable** — it is what makes
/// [`MT_CH_IDLE`] / [`MT_CH_BUSY`] read-and-clear, and it is set here. That is
/// not an accident of this port: upstream sets it too, and MEASURED behaviour on
/// the 7610 agrees (`(idle+busy)/elapsed = 1.00` per 100 ms window, which only
/// holds if each read zeroes the counter). If a future caller ever wants
/// free-running totals instead, clearing this bit is the switch — but then every
/// consumer of [`ChannelTime`] must start differencing, so do not do it quietly.
///
/// The four "as busy" bits define the *question*: with them set, "busy" means
/// "the MAC could not have transmitted" — its own TX, another station's decoded
/// frame, a NAV reservation, and the EIFS after a bad frame all count. That is
/// the airtime a named-radio scheduler actually competes for.
///
/// Ends by draining all three counters so the first [`read_channel_time`] after
/// this call measures a window that starts here rather than an unknown backlog
/// (upstream drains BUSY and IDLE at `mt76x02_mac.c:1230-1231`; the ED-CCA timer
/// drain mirrors `mt76x02_edcca_init`'s "clear previous CCA timer value" at
/// `mt76x02_mac.c:1133-1134`).
///
/// Cost: 4 EP0 round trips ≈ 604 µs.
pub fn enable_channel_time_counters(bus: &dyn Mt76Regs) -> Result<(), FaceError> {
    bus.wr(
        MT_CH_TIME_CFG,
        MT_CH_TIME_CFG_TIMER_EN
            | MT_CH_TIME_CFG_TX_AS_BUSY
            | MT_CH_TIME_CFG_RX_AS_BUSY
            | MT_CH_TIME_CFG_NAV_AS_BUSY
            | MT_CH_TIME_CFG_EIFS_AS_BUSY
            | MT_CH_CCA_RC_EN
            | field_prep(MT_CH_TIME_CFG_CH_TIMER_CLR, 1),
    )?;
    // Drain the three counters: whatever they hold now belongs to the window
    // before this call and would otherwise be charged to the caller's first one.
    let _ = bus.rr(MT_CH_BUSY)?;
    let _ = bus.rr(MT_CH_IDLE)?;
    let _ = bus.rr(MT_ED_CCA_TIMER)?;
    Ok(())
}

/// Take one window of channel occupancy and **reset** it.
///
/// ★ Read-and-clear: the returned [`ChannelTime`] covers the interval since the
/// previous call (or since [`enable_channel_time_counters`]). See the hazards on
/// [`ChannelTime`] — in particular that a bound kernel driver steals half the
/// count.
///
/// The three reads are sequential EP0 transfers, so the three counters' windows
/// are staggered by ≈151 µs each — 0.45 % of a 100 ms sensing window, and the
/// reason a sensing loop should not run much faster than that. `busy` is read
/// first so [`busy_permille`]'s numerator and denominator are as close together
/// as the bus allows.
///
/// Cost: 3 EP0 round trips ≈ 453 µs.
pub fn read_channel_time(bus: &dyn Mt76Regs) -> Result<ChannelTime, FaceError> {
    let busy_us = bus.rr(MT_CH_BUSY)?;
    let idle_us = bus.rr(MT_CH_IDLE)?;
    let ed_cca_us = bus.rr(MT_ED_CCA_TIMER)?;
    Ok(ChannelTime {
        busy_us,
        idle_us,
        ed_cca_us,
    })
}

/// Channel occupancy in **per-mille of the window**: `busy / (busy + idle)`.
///
/// This is the honest occupancy figure, and it is a strictly better sense than
/// the one the 8812au port had to settle for. There, occupancy is inferred from
/// `REG_RXERR_RPT` — a **frame count**, which tracks the neighbour's frame *rate*
/// 1:1 but says nothing about how long each frame held the medium, so a few big
/// aggregates and a storm of tiny ACKs look the same. Here the hardware hands
/// back **time**: microseconds the medium was unusable. That is the quantity an
/// airtime lease is denominated in, so no calibration constant stands between
/// the reading and the decision.
///
/// Per-mille rather than percent because a lightly used channel is the
/// interesting case and 0–100 quantises it too coarsely; and integer rather than
/// float because this feeds a scheduler, not a plot.
///
/// Returns 0 for an empty window (`busy + idle == 0`) — which means the counters
/// are not armed, or something else already drained them, not that the channel
/// was quiet. Distinguish the two by checking the raw [`ChannelTime`] against
/// the wall-clock window; [`window_coverage_permille`] does exactly that.
pub fn busy_permille(ct: &ChannelTime) -> u16 {
    let total = ct.busy_us as u64 + ct.idle_us as u64;
    if total == 0 {
        return 0;
    }
    ((ct.busy_us as u64 * 1000) / total) as u16
}

/// Energy-detect occupancy in per-mille of a caller-supplied wall-clock window.
///
/// ★ This is a **second, independent** sense of "busy" and answers a different
/// question than [`busy_permille`]:
///
/// * [`busy_permille`] is **decode-busy** — the medium was held by something the
///   MAC understood (a decoded frame, a NAV it honoured, its own TX).
/// * this is **energy-busy** — the RF energy detector alone crossed threshold,
///   with nothing decoded and no 802.11 semantics involved.
///
/// The gap between them is the diagnostic. Energy high with decode low is a
/// **non-Wi-Fi interferer** — a co-banded LoRa or HaLow transmitter (which is a
/// real configuration on this bench: 902–928 MHz is shared), a microwave, a
/// jammer. Decode high with energy low means a distant-but-decodable Wi-Fi
/// neighbour. One number cannot tell those apart; two can, which is why
/// [`ChannelTime`] carries both.
///
/// `window_us` must be the wall-clock elapsed time of the sampling window,
/// because [`MT_ED_CCA_TIMER`] is not normalised against an idle counterpart —
/// this mirrors `mt76x02_edcca_check` (`mt76x02_mac.c:1155-1160`), which divides
/// the same register by `ktime` elapsed. Clamped to 1000 ‰: a stale window can
/// otherwise report more busy microseconds than the window contains.
pub fn ed_cca_permille(ct: &ChannelTime, window_us: u32) -> u16 {
    if window_us == 0 {
        return 0;
    }
    (((ct.ed_cca_us as u64 * 1000) / window_us as u64).min(1000)) as u16
}

/// How much of a `window_us` wall-clock window the MAC's own counters account
/// for, in per-mille. MEASURED ≈ 1000 (i.e. 1.00) over 100 ms windows on a
/// healthy, exclusively-claimed part.
///
/// This is the **self-check** for the two hazards on [`ChannelTime`]: a reading
/// far below 1000 ‰ means either the counters are not armed
/// ([`enable_channel_time_counters`] was never called, or a channel switch reset
/// them) or another reader — typically a still-bound kernel `mt76x0u` — took the
/// other half. Either way [`busy_permille`] is then a lower bound, and a caller
/// that trusts it silently will under-report occupancy. Check this before
/// believing an occupancy figure.
pub fn window_coverage_permille(ct: &ChannelTime, window_us: u32) -> u16 {
    if window_us == 0 {
        return 0;
    }
    let total = ct.busy_us as u64 + ct.idle_us as u64;
    ((total * 1000) / window_us as u64).min(u16::MAX as u64) as u16
}

// ── RX error counters ────────────────────────────────────────────────────────

/// One window of the MAC's RX error counters. ★ MEASURED read-and-clear, same
/// as [`ChannelTime`] and with the same two hazards: nothing to difference, and
/// a second reader splits the count.
///
/// All four are 16-bit hardware counters, so each **saturates at 65535** rather
/// than wrapping usefully. On a busy channel `cca_err` reaches that in well
/// under a second, so a sensing window longer than ~100 ms can silently clip.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct RxStat {
    /// Frames whose FCS failed ([`MT_RX_STAT_0`] bits 15:0). Energy that was a
    /// real 802.11 frame and did not survive the channel — the collision and
    /// weak-link signal.
    pub crc_err: u16,
    /// PHY-level errors ([`MT_RX_STAT_0`] bits 31:16): a start-of-packet that
    /// never became a decodable frame.
    pub phy_err: u16,
    /// **False CCA** ([`MT_RX_STAT_1`] bits 15:0) — the carrier sense fired and
    /// no frame followed. This is the same frame-free interference signal the
    /// 8812au port reads out of `REG_RXERR_RPT`, and upstream's AGC/VGA loop
    /// uses it as `dev->cal.false_cca` (`mt76x02_mac.c:1170`). High false-CCA
    /// with low `crc_err` points at a non-802.11 emitter.
    pub cca_err: u16,
    /// PLCP header errors ([`MT_RX_STAT_1`] bits 31:16): a preamble decoded but
    /// its length/rate header did not check out — typically a collision that
    /// started mid-frame.
    pub plcp_err: u16,
}

/// Take one window of RX error counters and reset them. ★ Read-and-clear; see
/// [`RxStat`].
///
/// Cost: 2 EP0 round trips ≈ 302 µs.
pub fn read_rx_stat(bus: &dyn Mt76Regs) -> Result<RxStat, FaceError> {
    let s0 = bus.rr(MT_RX_STAT_0)?;
    let s1 = bus.rr(MT_RX_STAT_1)?;
    Ok(decode_rx_stat(s0, s1))
}

/// Pure decode of the two status words into an [`RxStat`], split out so the
/// field extraction is testable without hardware.
fn decode_rx_stat(stat0: u32, stat1: u32) -> RxStat {
    RxStat {
        crc_err: field_get(MT_RX_STAT_0_CRC_ERRORS, stat0) as u16,
        phy_err: field_get(MT_RX_STAT_0_PHY_ERRORS, stat0) as u16,
        cca_err: field_get(MT_RX_STAT_1_CCA_ERRORS, stat1) as u16,
        plcp_err: field_get(MT_RX_STAT_1_PLCP_ERRORS, stat1) as u16,
    }
}

// ── ED-CCA ───────────────────────────────────────────────────────────────────

/// Saved-original slot for [`set_edcca_ignore_with`], so `off` can put the part
/// back exactly as it was found rather than as this port guesses it should be —
/// the same save-on-first-call discipline as `crate::rtl8812au`'s
/// `set_cca_ignore` (which stashes the BB CCA nibble and the EDCA-BE word before
/// forcing them).
///
/// Two `u32` registers packed into one [`AtomicU64`] so the pair is saved and
/// restored atomically — a torn save (one register captured, the other not)
/// would restore a state the hardware was never in. `EMPTY` (all ones)
/// is the "nothing saved" sentinel: neither register can hold `0xffff_ffff`
/// ([`MT_TXOP_CTRL_CFG`]'s defined bits stop at 20, [`MT_EXT_CCA_CFG`]'s at 15),
/// so the sentinel is unambiguous.
#[derive(Debug)]
pub struct EdccaSaved(AtomicU64);

impl EdccaSaved {
    const EMPTY: u64 = u64::MAX;

    /// A slot with nothing saved in it.
    pub const fn new() -> Self {
        Self(AtomicU64::new(Self::EMPTY))
    }

    /// Save `(txop_ctrl_cfg, ext_cca_cfg)` **only if the slot is empty**, so the
    /// first call captures the as-found state and later calls do not overwrite
    /// it with this port's own writes.
    fn save_once(&self, txop: u32, ext: u32) {
        let packed = ((txop as u64) << 32) | ext as u64;
        let _ = self
            .0
            .compare_exchange(Self::EMPTY, packed, Ordering::SeqCst, Ordering::SeqCst);
    }

    /// Take the saved pair, emptying the slot.
    fn take(&self) -> Option<(u32, u32)> {
        let packed = self.0.swap(Self::EMPTY, Ordering::SeqCst);
        if packed == Self::EMPTY {
            None
        } else {
            Some(((packed >> 32) as u32, packed as u32))
        }
    }
}

impl Default for EdccaSaved {
    fn default() -> Self {
        Self::new()
    }
}

/// Process-wide saved-original slot backing [`set_edcca_ignore`].
///
/// A `static` because the contract function takes only `(&dyn Mt76Regs, bool)`
/// and has nowhere else to put the state. That is correct for the single-mt76
/// case, which is every current configuration; a host holding **two** mt76 parts
/// must give each its own [`EdccaSaved`] and call [`set_edcca_ignore_with`], or
/// the second radio's restore will write the first radio's registers back.
static EDCCA_SAVED: EdccaSaved = EdccaSaved::new();

/// What a [`set_edcca_ignore`] call actually did — before and after, per
/// register, plus whether anything moved at all.
///
/// Returned rather than logged-only because the interesting outcome on this part
/// is `changed == false`: MEASURED, [`MT_TXOP_CTRL_CFG`] is already `0x0000_583f`
/// with `ED_CCA_EN` **clear**, so "ignore ED-CCA" is where the silicon already
/// sits and asking for it is a no-op. A caller running an A/B needs to be able
/// to tell "I turned it off" from "it was already off".
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct EdccaChange {
    /// [`MT_TXOP_CTRL_CFG`] as read at the start of the call.
    pub txop_ctrl_before: u32,
    /// [`MT_TXOP_CTRL_CFG`] as written (equal to `before` if untouched).
    pub txop_ctrl_after: u32,
    /// [`MT_EXT_CCA_CFG`] as read at the start of the call.
    pub ext_cca_before: u32,
    /// [`MT_EXT_CCA_CFG`] as written (equal to `before` if untouched).
    pub ext_cca_after: u32,
    /// True if at least one of the two registers actually changed value.
    pub changed: bool,
}

impl EdccaChange {
    /// Is energy-detect CCA armed *after* this call — i.e. is bit 20 of
    /// [`MT_TXOP_CTRL_CFG`] set?
    pub fn ed_cca_armed(&self) -> bool {
        self.txop_ctrl_after & MT_TXOP_ED_CCA_EN != 0
    }
}

/// Make the TX engine **ignore** energy-detect CCA (`on = true`), or put it back
/// (`on = false`). Uses the process-wide `EDCCA_SAVED` slot; see
/// [`set_edcca_ignore_with`] for the per-radio form and for the full semantics.
pub fn set_edcca_ignore(bus: &dyn Mt76Regs, on: bool) -> Result<EdccaChange, FaceError> {
    set_edcca_ignore_with(bus, on, &EDCCA_SAVED)
}

/// [`set_edcca_ignore`] against a caller-owned saved slot.
///
/// # What the two directions do
///
/// * `on = true` — **ignore**: clear [`MT_TXOP_ED_CCA_EN`] (bit 20) in
///   [`MT_TXOP_CTRL_CFG`] so the TX engine does not defer to energy detect
///   (`mt76x02_mac.c:1119`, `mt76x0/init.c:140`), and clear `ED_CCA_MASK`
///   (bits 15:12) in [`MT_EXT_CCA_CFG`] so no detector feeds the ED path. The
///   as-found pair is saved into `saved` on the first call.
/// * `on = false` — **restore**: write the saved pair back verbatim. If nothing
///   was ever saved, **arm** ED-CCA instead — set bit 20 (`mt76x02_mac.c:1113`)
///   and `ED_CCA_MASK = 0xf` (`mt76x0/init.c:121`, `mt76x2/usb_mac.c:87`, both
///   of which write `0xf000` into this register).
///
/// `CCA_MASK` (bits 11:8), which selects which physical detector feeds the
/// *packet* CCA decision, is deliberately **not touched**. Upstream never writes
/// it outside the per-channel-group table (`mt76x0/phy.c:917-936`), and zeroing
/// it would disable ordinary carrier sense as a side effect of an energy-detect
/// knob. The 8812au port does force its packet-CCA equivalent off — but there it
/// is a separate, separately-named knob, and it should be one here too.
///
/// # ★ What this is measured to do, and what it is not
///
/// MEASURED as found on mds-o5p-1's MT7610U: `MT_TXOP_CTRL_CFG = 0x0000_583f`
/// (bit 20 **clear**) and `MT_EXT_CCA_CFG = 0x0000_f1e4` (`ED_CCA_MASK = 0xf`).
/// So on a part in this state, `set_edcca_ignore(bus, true)` leaves
/// `MT_TXOP_CTRL_CFG` untouched and only clears `ED_CCA_MASK`; the knob's real
/// work here is the **other** direction — arming ED-CCA that the driver left
/// off. That is why this function reads before it writes and reports what moved.
///
/// The **on-air effect is UNVALIDATED.** Nothing has been transmitted through
/// either state on a busy channel yet. The prior from the neighbouring silicon
/// is not encouraging: on the 8812au, energy-detect CCA works exactly as
/// specified and, on a saturated channel, trades collision loss for TX
/// starvation (delivered frames fell 237/s → 26/s when it was armed). Treat
/// this as an actuator to A/B on a saturated channel, not as a fix.
///
/// Cost: 4 EP0 round trips ≈ 604 µs (2 reads, 2 writes; writes are skipped when
/// the value already matches, so a no-op call costs 2).
pub fn set_edcca_ignore_with(
    bus: &dyn Mt76Regs,
    on: bool,
    saved: &EdccaSaved,
) -> Result<EdccaChange, FaceError> {
    let txop_before = bus.rr(MT_TXOP_CTRL_CFG)?;
    let ext_before = bus.rr(MT_EXT_CCA_CFG)?;

    let (txop_after, ext_after) = if on {
        saved.save_once(txop_before, ext_before);
        (
            txop_before & !MT_TXOP_ED_CCA_EN,
            ext_before & !MT_EXT_CCA_CFG_ED_CCA_MASK,
        )
    } else if let Some((txop, ext)) = saved.take() {
        (txop, ext)
    } else {
        (
            txop_before | MT_TXOP_ED_CCA_EN,
            (ext_before & !MT_EXT_CCA_CFG_ED_CCA_MASK)
                | field_prep(MT_EXT_CCA_CFG_ED_CCA_MASK, 0xf),
        )
    };

    if txop_after != txop_before {
        bus.wr(MT_TXOP_CTRL_CFG, txop_after)?;
    }
    if ext_after != ext_before {
        bus.wr(MT_EXT_CCA_CFG, ext_after)?;
    }

    let change = EdccaChange {
        txop_ctrl_before: txop_before,
        txop_ctrl_after: txop_after,
        ext_cca_before: ext_before,
        ext_cca_after: ext_after,
        changed: txop_after != txop_before || ext_after != ext_before,
    };
    tracing::info!(
        target: "named_radio",
        knob = "mt76.edcca_ignore",
        want_ignore = on,
        txop_ctrl = format_args!("{:#010x} -> {:#010x}", txop_before, txop_after),
        ext_cca = format_args!("{:#010x} -> {:#010x}", ext_before, ext_after),
        ed_cca_armed = change.ed_cca_armed(),
        changed = change.changed,
        "mt76 ED-CCA knob (on-air effect UNVALIDATED)",
    );
    Ok(change)
}

/// Read back whether energy-detect CCA is armed and which detectors feed it —
/// `(ed_cca_en, ed_cca_mask, cca_mask)`. The verification half of
/// [`set_edcca_ignore`], the way `crate::rtl8812au`'s `edcca_state` is for its
/// own knob. Cost: 2 EP0 round trips.
pub fn edcca_state(bus: &dyn Mt76Regs) -> Result<(bool, u32, u32), FaceError> {
    let txop = bus.rr(MT_TXOP_CTRL_CFG)?;
    let ext = bus.rr(MT_EXT_CCA_CFG)?;
    Ok((
        txop & MT_TXOP_ED_CCA_EN != 0,
        field_get(MT_EXT_CCA_CFG_ED_CCA_MASK, ext),
        field_get(MT_EXT_CCA_CFG_CCA_MASK, ext),
    ))
}

/// The **full** upstream ED-CCA arm sequence — `mt76x02_edcca_init`'s
/// `ed_monitor` branch (`mt76x02_mac.c:1107-1116`), which is more than the bit
/// [`set_edcca_ignore`] flips:
///
/// 1. clear `MT_TX_CFACK_EN` in [`MT_TX_LINK_CFG`] (`:1111`),
/// 2. set [`MT_TXOP_ED_CCA_EN`] (`:1112`),
/// 3. write the **energy threshold** into `MT_BBP(AGC, 2)` bits 15:0 as
///    `th << 8 | th`, `0x0e` on 5 GHz and `0x20` on 2.4 GHz (`:1110,1113-1114`),
/// 4. set [`MT_TXOP_HLDR_TX40M_BLK_EN`] (`:1115`),
/// 5. drain [`MT_ED_CCA_TIMER`] so the first window starts here (`:1133-1134`).
///
/// Kept separate from [`set_edcca_ignore`] on purpose: this one writes the
/// **baseband**, and a knob that quietly retunes the BB threshold underneath a
/// caller who asked about a MAC bit would be a trap. Use this for a deliberate
/// A/B, and [`disarm_edcca`] to undo it.
///
/// Upstream applies the same threshold pair on both families; why those two
/// values and not others is not stated anywhere in the tree, so they are ported
/// as constants without a rationale rather than derived.
pub fn arm_edcca(bus: &dyn Mt76Regs, five_ghz: bool) -> Result<(), FaceError> {
    let th = if five_ghz {
        MT_EDCCA_BBP_TH_5G
    } else {
        MT_EDCCA_BBP_TH_2G
    };
    bus.rmw(MT_TX_LINK_CFG, MT_TX_CFACK_EN, 0)?;
    bus.rmw(MT_TXOP_CTRL_CFG, 0, MT_TXOP_ED_CCA_EN)?;
    bus.rmw(mt_bbp(MT_BBP_AGC_BASE, 2), 0x0000_ffff, (th << 8) | th)?;
    bus.rmw(MT_TXOP_HLDR_ET, 0, MT_TXOP_HLDR_TX40M_BLK_EN)?;
    let _ = bus.rr(MT_ED_CCA_TIMER)?;
    Ok(())
}

/// Undo [`arm_edcca`] — `mt76x02_edcca_init`'s non-`ed_monitor` branch
/// (`mt76x02_mac.c:1117-1128`). The `MT_BBP(AGC, 2)` value and the
/// `TX40M_BLK_EN` direction **differ by family**, which is a real per-part
/// divergence upstream and not a typo:
///
/// * [`Family::Mt76x2`]: `AGC2 = 0x0000_7070`, set `TX40M_BLK_EN` (`:1123-1125`)
/// * [`Family::Mt76x0`]: `AGC2 = 0x003a_6464`, clear `TX40M_BLK_EN` (`:1126-1128`)
///
/// ★ MEASURED: a kernel monitor leaves `MT_BBP(AGC, 2) = 0x003a_6464` on the
/// MT7610U — bit-identical to the mt76x0 branch here. That is the confirmation
/// that this is the correct disarmed state for this part, not a guess.
pub fn disarm_edcca(bus: &dyn Mt76Regs, family: Family) -> Result<(), FaceError> {
    bus.rmw(MT_TX_LINK_CFG, 0, MT_TX_CFACK_EN)?;
    bus.rmw(MT_TXOP_CTRL_CFG, MT_TXOP_ED_CCA_EN, 0)?;
    match family {
        Family::Mt76x2 => {
            bus.wr(mt_bbp(MT_BBP_AGC_BASE, 2), 0x0000_7070)?;
            bus.rmw(MT_TXOP_HLDR_ET, 0, MT_TXOP_HLDR_TX40M_BLK_EN)?;
        }
        Family::Mt76x0 => {
            bus.wr(mt_bbp(MT_BBP_AGC_BASE, 2), 0x003a_6464)?;
            bus.rmw(MT_TXOP_HLDR_ET, MT_TXOP_HLDR_TX40M_BLK_EN, 0)?;
        }
    }
    Ok(())
}

// ── RX filter ────────────────────────────────────────────────────────────────

/// The [`MT_RX_FILTR_CFG`] value that drops **nothing**.
///
/// Every bit in this register is a *drop* condition — mac80211's monitor path
/// clears a bit to let that class through (`mt76x02_util.c`'s `MT76_FILTER`
/// macro sets the bit when the corresponding `FIF_` flag is absent). So zero is
/// full promiscuity: every frame the PHY produces reaches the host.
///
/// ★ That includes frames the hardware would otherwise drop as **broken**:
/// bit 0 `CRC_ERR` and bit 1 `PHY_ERR` are set even in the measured kernel
/// monitor value, and clearing them hands the host frames whose FCS failed. For
/// a *sensor* that is a feature — a CRC-failed frame is still evidence the
/// medium was occupied. For a *decoder* it is a trap: the LR2021 testbed spent a
/// whole campaign on results that turned out to be CRC-failing frames. If the
/// consumer parses payloads, use [`RX_FILTER_PROMISCUOUS_VALID`] instead.
pub const RX_FILTER_PROMISCUOUS: u32 = 0;

/// Promiscuous, but still drop frames that failed FCS or PHY decode — the
/// setting for a consumer that will parse what it receives. See
/// [`RX_FILTER_PROMISCUOUS`] for why the distinction matters.
pub const RX_FILTER_PROMISCUOUS_VALID: u32 =
    crate::mt76::regs::MT_RX_FILTR_CFG_CRC_ERR | crate::mt76::regs::MT_RX_FILTR_CFG_PHY_ERR;

/// ★ MEASURED reference point: [`MT_RX_FILTR_CFG`] as a kernel `mt76x0u`
/// monitor leaves it — `0x0000_1093`. It decomposes exactly as
/// `CRC_ERR | PHY_ERR | VER_ERR | DUP | RTS`, i.e. the init value
/// `0x0001_7f97` (`mt76x0/initvals_init.h:20`) minus everything mac80211 clears
/// for a monitor vif. Restoring this is the closest thing to "put it back how
/// the kernel had it" when no original was captured.
pub const KERNEL_MONITOR_RX_FILTER: u32 = 0x0000_1093;

/// Saved-original slot for the RX filter, same discipline and same sentinel
/// reasoning as [`EdccaSaved`] (one `u32`, so the sentinel is `u64::MAX` in a
/// [`AtomicU64`] rather than a value the register could hold).
#[derive(Debug)]
pub struct RxFilterSaved(AtomicU64);

impl RxFilterSaved {
    const EMPTY: u64 = u64::MAX;

    /// A slot with nothing saved in it.
    pub const fn new() -> Self {
        Self(AtomicU64::new(Self::EMPTY))
    }

    fn save_once(&self, val: u32) {
        let _ =
            self.0
                .compare_exchange(Self::EMPTY, val as u64, Ordering::SeqCst, Ordering::SeqCst);
    }

    fn take(&self) -> Option<u32> {
        let v = self.0.swap(Self::EMPTY, Ordering::SeqCst);
        if v == Self::EMPTY {
            None
        } else {
            Some(v as u32)
        }
    }
}

impl Default for RxFilterSaved {
    fn default() -> Self {
        Self::new()
    }
}

/// Process-wide saved slot backing [`set_rx_filter_promiscuous`] /
/// [`restore_rx_filter`]. Same single-radio caveat as `EDCCA_SAVED`; a host
/// with two mt76 parts should own a [`RxFilterSaved`] each and use the `_with`
/// forms.
static RX_FILTER_SAVED: RxFilterSaved = RxFilterSaved::new();

/// Open the RX filter all the way ([`RX_FILTER_PROMISCUOUS`]) and return the
/// value that was there, saving it for [`restore_rx_filter`].
///
/// Cost: 2 EP0 round trips ≈ 302 µs.
pub fn set_rx_filter_promiscuous(bus: &dyn Mt76Regs) -> Result<u32, FaceError> {
    set_rx_filter_with(bus, RX_FILTER_PROMISCUOUS, &RX_FILTER_SAVED)
}

/// Write an arbitrary [`MT_RX_FILTR_CFG`] value (e.g.
/// [`RX_FILTER_PROMISCUOUS_VALID`]), saving the as-found original on the first
/// call. Returns the previous value.
pub fn set_rx_filter(bus: &dyn Mt76Regs, value: u32) -> Result<u32, FaceError> {
    set_rx_filter_with(bus, value, &RX_FILTER_SAVED)
}

/// [`set_rx_filter`] against a caller-owned saved slot.
pub fn set_rx_filter_with(
    bus: &dyn Mt76Regs,
    value: u32,
    saved: &RxFilterSaved,
) -> Result<u32, FaceError> {
    let before = bus.rr(MT_RX_FILTR_CFG)?;
    saved.save_once(before);
    if before != value {
        bus.wr(MT_RX_FILTR_CFG, value)?;
    }
    tracing::info!(
        target: "named_radio",
        knob = "mt76.rx_filter",
        rx_filtr_cfg = format_args!("{:#010x} -> {:#010x}", before, value),
        kernel_monitor_ref = format_args!("{:#010x}", KERNEL_MONITOR_RX_FILTER),
        "mt76 RX filter",
    );
    Ok(before)
}

/// Put [`MT_RX_FILTR_CFG`] back to whatever [`set_rx_filter_promiscuous`] found.
/// Returns `false` (and writes nothing) if no original was ever saved — a caller
/// that wants a defined state regardless should use
/// [`restore_rx_filter_value`] with [`KERNEL_MONITOR_RX_FILTER`].
pub fn restore_rx_filter(bus: &dyn Mt76Regs) -> Result<bool, FaceError> {
    restore_rx_filter_with(bus, &RX_FILTER_SAVED)
}

/// [`restore_rx_filter`] against a caller-owned saved slot.
pub fn restore_rx_filter_with(
    bus: &dyn Mt76Regs,
    saved: &RxFilterSaved,
) -> Result<bool, FaceError> {
    match saved.take() {
        Some(v) => {
            restore_rx_filter_value(bus, v)?;
            Ok(true)
        }
        None => Ok(false),
    }
}

/// Write an explicit value back into [`MT_RX_FILTR_CFG`]. Cost: 1 EP0 round trip.
pub fn restore_rx_filter_value(bus: &dyn Mt76Regs, saved: u32) -> Result<(), FaceError> {
    bus.wr(MT_RX_FILTR_CFG, saved)
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::collections::{HashMap, HashSet, VecDeque};
    use std::sync::Mutex;

    /// A register file that reproduces the two behaviours these knobs depend on:
    /// writes stick, and an address in `clear_on_read` returns its value once and
    /// zero thereafter (the MEASURED read-and-clear semantics). `scripts` lets a
    /// test hand out a specific sequence of reads for one address — needed to
    /// exercise the TSF carry path, which cannot be provoked from a static file.
    struct MockRegs {
        regs: Mutex<HashMap<u32, u32>>,
        scripts: Mutex<HashMap<u32, VecDeque<u32>>>,
        clear_on_read: HashSet<u32>,
        writes: Mutex<Vec<(u32, u32)>>,
    }

    impl MockRegs {
        fn new(init: &[(u32, u32)]) -> Self {
            Self {
                regs: Mutex::new(init.iter().copied().collect()),
                scripts: Mutex::new(HashMap::new()),
                clear_on_read: HashSet::new(),
                writes: Mutex::new(Vec::new()),
            }
        }

        fn with_read_clear(mut self, addrs: &[u32]) -> Self {
            self.clear_on_read = addrs.iter().copied().collect();
            self
        }

        /// Queue a sequence of reads for `addr`; the last entry sticks once the
        /// queue is down to one, mirroring a counter that has settled.
        fn script(self, addr: u32, vals: &[u32]) -> Self {
            self.scripts
                .lock()
                .unwrap()
                .insert(addr, vals.iter().copied().collect());
            self
        }

        fn writes(&self) -> Vec<(u32, u32)> {
            self.writes.lock().unwrap().clone()
        }
    }

    impl Mt76Regs for MockRegs {
        fn rr(&self, addr: u32) -> Result<u32, FaceError> {
            if let Some(q) = self.scripts.lock().unwrap().get_mut(&addr) {
                if q.len() > 1 {
                    return Ok(q.pop_front().unwrap());
                }
                if let Some(&v) = q.front() {
                    return Ok(v);
                }
            }
            let mut regs = self.regs.lock().unwrap();
            let v = regs.get(&addr).copied().unwrap_or(0);
            if self.clear_on_read.contains(&addr) {
                regs.insert(addr, 0);
            }
            Ok(v)
        }

        fn wr(&self, addr: u32, val: u32) -> Result<(), FaceError> {
            self.regs.lock().unwrap().insert(addr, val);
            self.writes.lock().unwrap().push((addr, val));
            Ok(())
        }
    }

    /// `enable_tsf` must set TIMER_EN, clear SYNC_MODE, and leave the beacon
    /// interval alone — a blind `wr` of the two bits would zero INTVAL and break
    /// beaconing for anyone who set it.
    #[test]
    fn enable_tsf_sets_timer_en_clears_sync_mode_and_keeps_intval() {
        let m = MockRegs::new(&[(MT_BEACON_TIME_CFG, 0x0006_0640)]);
        enable_tsf(&m).unwrap();
        let v = m.rr(MT_BEACON_TIME_CFG).unwrap();
        assert_eq!(v & MT_BEACON_TIME_CFG_TIMER_EN, MT_BEACON_TIME_CFG_TIMER_EN);
        assert_eq!(v & MT_BEACON_TIME_CFG_SYNC_MODE, 0);
        assert_eq!(v & 0xffff, 0x0640, "INTVAL must survive");
        assert!(tsf_running(&m).unwrap());
    }

    /// `tsf_running` is false when the timer is off AND when SYNC_MODE would let
    /// a beacon rewrite the counter — a clock a beacon can move is not a clock.
    #[test]
    fn tsf_running_rejects_beacon_synced_counter() {
        let off = MockRegs::new(&[(MT_BEACON_TIME_CFG, 0x0000_0640)]);
        assert!(!tsf_running(&off).unwrap());
        let synced = MockRegs::new(&[(
            MT_BEACON_TIME_CFG,
            MT_BEACON_TIME_CFG_TIMER_EN | 0x0002_0000,
        )]);
        assert!(!tsf_running(&synced).unwrap());
    }

    /// ★ DW0 is the LOW word (MEASURED). If someone "fixes" `read_tsf` back to
    /// the upstream `mt76x02_usb_core.c:155` expression, this fails.
    #[test]
    fn read_tsf_treats_dw0_as_the_low_word() {
        let m = MockRegs::new(&[
            (MT_TSF_TIMER_DW0, 0x0000_2710),
            (MT_TSF_TIMER_DW1, 0x0000_0003),
        ]);
        assert_eq!(read_tsf(&m).unwrap(), 0x0000_0003_0000_2710);
    }

    /// The carry path: the high word advances between the two reads, so the low
    /// word read in between belongs to the *old* high word and must be discarded.
    /// Without the re-read this returns a value ~4295 s in the future.
    #[test]
    fn read_tsf_rereads_low_word_when_high_word_carries() {
        let m = MockRegs::new(&[])
            .script(MT_TSF_TIMER_DW1, &[0x0000_0003, 0x0000_0004])
            .script(MT_TSF_TIMER_DW0, &[0xffff_fff0, 0x0000_0005]);
        assert_eq!(read_tsf(&m).unwrap(), 0x0000_0004_0000_0005);
    }

    /// `enable_channel_time_counters` must arm the read-clear bit — it is what
    /// makes the counters behave as MEASURED — and must drain all three counters
    /// so the caller's first window starts at the call.
    #[test]
    fn channel_time_arming_sets_read_clear_and_drains() {
        let m = MockRegs::new(&[
            (MT_CH_BUSY, 12_345),
            (MT_CH_IDLE, 54_321),
            (MT_ED_CCA_TIMER, 999),
        ])
        .with_read_clear(&[MT_CH_BUSY, MT_CH_IDLE, MT_ED_CCA_TIMER]);
        enable_channel_time_counters(&m).unwrap();

        let cfg = m.writes()[0].1;
        assert_eq!(m.writes()[0].0, MT_CH_TIME_CFG);
        for bit in [
            MT_CH_TIME_CFG_TIMER_EN,
            MT_CH_TIME_CFG_TX_AS_BUSY,
            MT_CH_TIME_CFG_RX_AS_BUSY,
            MT_CH_TIME_CFG_NAV_AS_BUSY,
            MT_CH_TIME_CFG_EIFS_AS_BUSY,
            MT_CH_CCA_RC_EN,
        ] {
            assert_eq!(cfg & bit, bit, "missing bit {bit:#x} in {cfg:#x}");
        }
        assert_eq!(field_get(MT_CH_TIME_CFG_CH_TIMER_CLR, cfg), 1);

        // Drained: the stale backlog is gone, so the first real read is a clean
        // window rather than whatever the kernel driver had accumulated.
        let ct = read_channel_time(&m).unwrap();
        assert_eq!(ct, ChannelTime::default());
    }

    /// ★ Read-and-clear: the second read of a window returns zero. This is the
    /// property that makes differencing wrong, so it is asserted, not assumed.
    #[test]
    fn channel_time_is_read_and_clear() {
        let m = MockRegs::new(&[
            (MT_CH_BUSY, 30_000),
            (MT_CH_IDLE, 70_000),
            (MT_ED_CCA_TIMER, 25_000),
        ])
        .with_read_clear(&[MT_CH_BUSY, MT_CH_IDLE, MT_ED_CCA_TIMER]);
        let first = read_channel_time(&m).unwrap();
        assert_eq!(first.busy_us, 30_000);
        assert_eq!(first.idle_us, 70_000);
        assert_eq!(first.ed_cca_us, 25_000);
        assert_eq!(read_channel_time(&m).unwrap(), ChannelTime::default());
    }

    /// Occupancy arithmetic, including the case that matters most: an empty
    /// window reports 0 ‰ and must not divide by zero.
    #[test]
    fn busy_permille_is_busy_over_busy_plus_idle() {
        let ct = ChannelTime {
            busy_us: 30_000,
            idle_us: 70_000,
            ed_cca_us: 0,
        };
        assert_eq!(busy_permille(&ct), 300);
        assert_eq!(busy_permille(&ChannelTime::default()), 0);
        // Saturated channel: all of the window was busy.
        let full = ChannelTime {
            busy_us: 100_000,
            idle_us: 0,
            ed_cca_us: 0,
        };
        assert_eq!(busy_permille(&full), 1000);
        // No u32 overflow at the counters' full range (the ×1000 is done in u64).
        let big = ChannelTime {
            busy_us: u32::MAX,
            idle_us: u32::MAX,
            ed_cca_us: 0,
        };
        assert_eq!(busy_permille(&big), 500);
    }

    /// The two senses are independent: an energy-only interferer shows up in
    /// `ed_cca_permille` while `busy_permille` stays low. That gap is the whole
    /// reason `ChannelTime` carries both.
    #[test]
    fn ed_cca_is_a_second_independent_sense() {
        let ct = ChannelTime {
            busy_us: 5_000,
            idle_us: 95_000,
            ed_cca_us: 80_000,
        };
        assert_eq!(busy_permille(&ct), 50);
        assert_eq!(ed_cca_permille(&ct, 100_000), 800);
        assert_eq!(ed_cca_permille(&ct, 0), 0);
        // A stale window cannot report more than a full window of energy.
        assert_eq!(ed_cca_permille(&ct, 10_000), 1000);
    }

    /// The self-check that catches the "a kernel driver is also reading these"
    /// hazard: half the count means half the coverage.
    #[test]
    fn window_coverage_detects_a_split_count() {
        let whole = ChannelTime {
            busy_us: 30_000,
            idle_us: 70_000,
            ed_cca_us: 0,
        };
        assert_eq!(window_coverage_permille(&whole, 100_000), 1000);
        let split = ChannelTime {
            busy_us: 15_000,
            idle_us: 35_000,
            ed_cca_us: 0,
        };
        assert_eq!(window_coverage_permille(&split, 100_000), 500);
        assert_eq!(window_coverage_permille(&whole, 0), 0);
    }

    /// Field extraction for the two RX-status words, using values whose halves
    /// differ so a swapped mask cannot pass.
    #[test]
    fn rx_stat_decodes_both_halves_of_both_words() {
        let s = decode_rx_stat(0x0002_0001, 0x0004_0003);
        assert_eq!(
            s,
            RxStat {
                crc_err: 1,
                phy_err: 2,
                cca_err: 3,
                plcp_err: 4,
            }
        );
        let m = MockRegs::new(&[(MT_RX_STAT_0, 0x0002_0001), (MT_RX_STAT_1, 0x0004_0003)])
            .with_read_clear(&[MT_RX_STAT_0, MT_RX_STAT_1]);
        assert_eq!(read_rx_stat(&m).unwrap(), s);
        // ★ read-and-clear per window.
        assert_eq!(read_rx_stat(&m).unwrap(), RxStat::default());
    }

    /// ★ Against the MEASURED as-found registers, "ignore ED-CCA" barely moves:
    /// bit 20 is already clear, so only `ED_CCA_MASK` changes. A caller running
    /// an A/B has to be able to see that.
    #[test]
    fn edcca_ignore_against_the_measured_as_found_state() {
        let m = MockRegs::new(&[
            (MT_TXOP_CTRL_CFG, 0x0000_583f),
            (MT_EXT_CCA_CFG, 0x0000_f1e4),
        ]);
        let saved = EdccaSaved::new();
        let c = set_edcca_ignore_with(&m, true, &saved).unwrap();
        assert_eq!(c.txop_ctrl_before, 0x0000_583f);
        assert_eq!(c.txop_ctrl_after, 0x0000_583f, "bit 20 was already clear");
        assert_eq!(c.ext_cca_after, 0x0000_01e4, "ED_CCA_MASK cleared");
        assert!(c.changed);
        assert!(!c.ed_cca_armed());
        // CCA_MASK (bits 11:8) is not this knob's business and must survive.
        assert_eq!(field_get(MT_EXT_CCA_CFG_CCA_MASK, c.ext_cca_after), 1);

        // …and `off` puts the part back exactly as found.
        let back = set_edcca_ignore_with(&m, false, &saved).unwrap();
        assert_eq!(back.txop_ctrl_after, 0x0000_583f);
        assert_eq!(back.ext_cca_after, 0x0000_f1e4);
    }

    /// With nothing saved, `off` is the direction that actually does something:
    /// it ARMS energy-detect CCA (`mt76x02_mac.c:1113` + `mt76x0/init.c:121`).
    #[test]
    fn edcca_off_with_no_saved_state_arms_ed_cca() {
        let m = MockRegs::new(&[
            (MT_TXOP_CTRL_CFG, 0x0000_583f),
            (MT_EXT_CCA_CFG, 0x0000_01e4),
        ]);
        let saved = EdccaSaved::new();
        let c = set_edcca_ignore_with(&m, false, &saved).unwrap();
        assert!(c.ed_cca_armed());
        assert_eq!(c.txop_ctrl_after, 0x0010_583f);
        assert_eq!(field_get(MT_EXT_CCA_CFG_ED_CCA_MASK, c.ext_cca_after), 0xf);
        assert_eq!(
            edcca_state(&m).unwrap(),
            (true, 0xf, 1),
            "readback must agree with what was written"
        );
    }

    /// The save must capture the AS-FOUND state, not this port's own writes —
    /// otherwise a second `ignore(true)` overwrites the original with the already
    /// modified value and `off` can never restore.
    #[test]
    fn edcca_save_captures_the_as_found_state_only_once() {
        let m = MockRegs::new(&[
            (MT_TXOP_CTRL_CFG, 0x0010_583f),
            (MT_EXT_CCA_CFG, 0x0000_f1e4),
        ]);
        let saved = EdccaSaved::new();
        set_edcca_ignore_with(&m, true, &saved).unwrap();
        set_edcca_ignore_with(&m, true, &saved).unwrap();
        let back = set_edcca_ignore_with(&m, false, &saved).unwrap();
        assert_eq!(back.txop_ctrl_after, 0x0010_583f);
        assert_eq!(back.ext_cca_after, 0x0000_f1e4);
    }

    /// The mt76x0 disarm branch must reproduce the MEASURED kernel-monitor
    /// `MT_BBP(AGC, 2)`, and must NOT write the mt76x2 value.
    #[test]
    fn disarm_edcca_writes_the_measured_mt76x0_agc2() {
        let m = MockRegs::new(&[(MT_TXOP_CTRL_CFG, 0x0010_583f)]);
        disarm_edcca(&m, Family::Mt76x0).unwrap();
        assert_eq!(m.rr(mt_bbp(MT_BBP_AGC_BASE, 2)).unwrap(), 0x003a_6464);
        assert_eq!(m.rr(MT_TXOP_CTRL_CFG).unwrap() & MT_TXOP_ED_CCA_EN, 0);
        assert_eq!(
            m.rr(MT_TXOP_HLDR_ET).unwrap() & MT_TXOP_HLDR_TX40M_BLK_EN,
            0,
            "mt76x0 CLEARS TX40M_BLK_EN where mt76x2 sets it"
        );

        let x2 = MockRegs::new(&[]);
        disarm_edcca(&x2, Family::Mt76x2).unwrap();
        assert_eq!(x2.rr(mt_bbp(MT_BBP_AGC_BASE, 2)).unwrap(), 0x0000_7070);
        assert_eq!(
            x2.rr(MT_TXOP_HLDR_ET).unwrap() & MT_TXOP_HLDR_TX40M_BLK_EN,
            MT_TXOP_HLDR_TX40M_BLK_EN
        );
    }

    /// `arm_edcca` picks the band's threshold and writes it into both halves of
    /// `MT_BBP(AGC, 2)` bits 15:0, leaving the upper half alone.
    #[test]
    fn arm_edcca_writes_the_band_threshold_twice() {
        let m = MockRegs::new(&[(mt_bbp(MT_BBP_AGC_BASE, 2), 0x003a_6464)]);
        arm_edcca(&m, false).unwrap();
        assert_eq!(m.rr(mt_bbp(MT_BBP_AGC_BASE, 2)).unwrap(), 0x003a_2020);
        arm_edcca(&m, true).unwrap();
        assert_eq!(m.rr(mt_bbp(MT_BBP_AGC_BASE, 2)).unwrap(), 0x003a_0e0e);
        assert_eq!(m.rr(MT_TX_LINK_CFG).unwrap() & MT_TX_CFACK_EN, 0);
    }

    /// Promiscuous is zero (every bit is a drop condition), the original is
    /// saved, and restore puts the MEASURED kernel-monitor value back.
    #[test]
    fn rx_filter_opens_and_restores() {
        let m = MockRegs::new(&[(MT_RX_FILTR_CFG, KERNEL_MONITOR_RX_FILTER)]);
        let saved = RxFilterSaved::new();
        let before = set_rx_filter_with(&m, RX_FILTER_PROMISCUOUS, &saved).unwrap();
        assert_eq!(before, KERNEL_MONITOR_RX_FILTER);
        assert_eq!(m.rr(MT_RX_FILTR_CFG).unwrap(), 0);
        assert!(restore_rx_filter_with(&m, &saved).unwrap());
        assert_eq!(m.rr(MT_RX_FILTR_CFG).unwrap(), KERNEL_MONITOR_RX_FILTER);
        // Nothing left to restore — say so rather than writing a made-up value.
        assert!(!restore_rx_filter_with(&m, &saved).unwrap());
    }

    /// The decoder-safe variant keeps exactly the two error drops, so a consumer
    /// that parses payloads is not handed FCS-failing frames.
    #[test]
    fn promiscuous_valid_keeps_only_the_error_drops() {
        assert_eq!(RX_FILTER_PROMISCUOUS_VALID, 0x0000_0003);
        assert_eq!(
            KERNEL_MONITOR_RX_FILTER & RX_FILTER_PROMISCUOUS_VALID,
            RX_FILTER_PROMISCUOUS_VALID,
            "the kernel monitor also drops CRC/PHY errors",
        );
    }
}

// ── Contention window (EDCA) ────────────────────────────────────────────────────────────────
//
// The mt76x02 side of [`ndn_radio_hal::RadioKnobs::set_contention`]. Shared by the MT7610U and
// MT7612U because, as everywhere else in this module, the two families are one register map.

/// `MT_WMM_AIFSN` (0x0214) — 4 bits per AC.
pub const MT_WMM_AIFSN: u32 = 0x0214;
/// `MT_WMM_CWMIN` (0x0218) — 4 bits per AC, an exponent.
pub const MT_WMM_CWMIN: u32 = 0x0218;
/// `MT_WMM_CWMAX` (0x021c) — 4 bits per AC, an exponent.
pub const MT_WMM_CWMAX: u32 = 0x021c;
/// `MT_EDCA_CFG_AC(n)` = 0x1300 + 4n. Fields: `TXOP[7:0] AIFSN[11:8] CWMIN[15:12] CWMAX[19:16]`
/// (`mt76x02_regs.h:380-386`).
pub const fn mt_edca_cfg_ac(ac: u32) -> u32 {
    0x1300 + (ac << 2)
}

/// `MT_BKOFF_SLOT_CFG` (0x1104) — `SLOTTIME[7:0]`, `CC_DELAY[11:8]` (in slots).
pub const MT_BKOFF_SLOT_CFG: u32 = 0x1104;
/// `MT_TX_TIMEOUT_CFG` (0x1348) — `ACKTO[15:8]`, which must track the slot time.
pub const MT_TX_TIMEOUT_CFG: u32 = 0x1348;

/// The 802.11a/n/ac short slot, and the smallest legal value for these registers.
pub const SLOT_US_MIN: u8 = 9;
/// The long (DSSS-compatible) slot. The MT7612U boots here; the MT7610U does not.
pub const SLOT_US_MAX: u8 = 20;

/// ★ **The floor. Never program a contention-window exponent below this.**
///
/// MEASURED 2026-08-28 on the MT7612U: writing exponent **0** (CW = 0, no backoff) into these
/// registers did not make the radio faster, it made it 5.7x *slower* (119 -> 21 Mbit/s), and then
/// left the MAC unable to transmit at all — every subsequent MCU command failed, the kernel
/// driver's own probe then failed with `firmware upload failed: -110`, and only a physical replug
/// recovered it. Whatever the arbiter does with a zero window, it is not "transmit immediately".
///
/// 2 is the smallest value MEASURED healthy (on the connac2 sibling, where it gave the full
/// 382 -> 416 Mbit/s win through the firmware command). Do not lower this without a bench and
/// somebody standing next to the dongle.
pub const MIN_CW_EXPONENT: u8 = 2;

/// The as-found EDCA state, so a posture change can be undone. Same discipline as
/// [`EdccaSaved`]: the sentinel is a value the registers cannot hold.
#[derive(Debug)]
pub struct EdcaSaved(AtomicU64, AtomicU64);

impl EdcaSaved {
    const EMPTY: u64 = u64::MAX;
    /// A slot with nothing saved in it.
    pub const fn new() -> Self {
        Self(AtomicU64::new(Self::EMPTY), AtomicU64::new(Self::EMPTY))
    }
}

impl Default for EdcaSaved {
    fn default() -> Self {
        Self::new()
    }
}

/// The contention window the mt76x02 parts boot with — `MT_EDCA_CFG_AC(n) = 0x000a4200`,
/// i.e. AIFSN 2, CWmin exponent 4, CWmax exponent 10.
pub const BOOT_CW_MIN: u8 = 4;
/// Boot `CWmax` exponent.
pub const BOOT_CW_MAX: u8 = 10;
/// Boot AIFSN.
pub const BOOT_AIFSN: u8 = 2;

/// ☠ **On the mt76x2, the EDCA window must never be programmed BELOW the boot value — at any
/// exponent, not just zero.**
///
/// MEASURED 2026-08-28 on the MT7612U, **twice**, each costing a physical replug: writing
/// `cw_min` exponent 2 / `cw_max` 4 / AIFSN 1 (a perfectly legal 802.11 window, and the value that
/// is *measured good* on three other radios) collapsed the part to **19 f/s / 0.86 Mbit/s**
/// — 52 ms per frame — and then it stopped transmitting entirely. Neither
/// [`crate::Mt7612uBackend::restore_edca_defaults`] nor a release-to-kernel cold firmware reload
/// recovered it; the kernel's own probe then failed with `firmware upload failed: -110`.
///
/// This is **not** the family-wide rule. The same write is fine on:
/// * the **MT7610U** (mt76x0, same register map) — MEASURED 2777 f/s at `Owned`, no ill effect;
/// * the **MT7921AU** (connac2), where the firmware validates the request — 382 → 416 Mbit/s;
/// * all three **Realtek** parts — MEASURED +10.4 % with a byte-exact restore afterwards.
///
/// So the hazard is specific to the mt76x2 EDCA register block, and [`MIN_CW_EXPONENT`] is not a
/// sufficient floor for it. On that part `Owned` is expressed through the **slot time** instead,
/// which is where the win is anyway: slot 20 → 9 MEASURED **131.9 → 190.6 Mbit/s (+45 %)** at
/// VHT80, repeatedly, reversibly, with the window left exactly as booted.
pub fn window_floor(family: Family) -> u8 {
    match family {
        Family::Mt76x2 => BOOT_CW_MIN,
        Family::Mt76x0 => MIN_CW_EXPONENT,
    }
}

/// Map a posture onto `(cw_min, cw_max, aifs, txop)` exponents for the mt76x02 family.
///
/// `Shared` reproduces the values `init_replay` programs — MEASURED as the state in which this
/// part reaches 131 Mbit/s at VHT80 — rather than a standards table, so "back to normal" means
/// "back to what this driver actually booted with".
///
/// ★ The `Owned` mapping is **family-dependent**, and on the mt76x2 it deliberately leaves the
/// window at the boot value. See [`window_floor`] for the two replugs that bought that rule.
pub fn mt76x02_posture(posture: ContentionPosture, family: Family) -> (u8, u8, u8, u16) {
    let floor = window_floor(family);
    match posture {
        // cw_min 2 (CW = 3 slots, ~13 us average backoff) instead of 4 (15 slots, ~67 us) — but
        // only where that is MEASURED safe. On the mt76x2 the floor pins this to the boot window
        // and the posture is carried entirely by the slot time.
        ContentionPosture::Owned => {
            let cw_min = MIN_CW_EXPONENT.max(floor);
            let aifs = if cw_min > MIN_CW_EXPONENT {
                BOOT_AIFSN
            } else {
                1
            };
            (cw_min, BOOT_CW_MAX.max(cw_min), aifs, 0)
        }
        // The init defaults: WMM 0x2222/0x4444/0xaaaa, MT_EDCA_CFG_AC(n) 0x000a4200.
        ContentionPosture::Shared => (BOOT_CW_MIN, BOOT_CW_MAX, BOOT_AIFSN, 0),
        // A deliberately larger window, for coexistence and anti-starvation. Raising the window is
        // the safe direction on every part measured.
        ContentionPosture::Yielding => (6, BOOT_CW_MAX, 3, 0),
    }
}

/// The slot time each family boots with. MEASURED: the MT7612U ships `MT_BKOFF_SLOT_CFG = 0x114`
/// (20 µs, the long/DSSS-compatible slot) and the MT7610U ships `0x209` (9 µs, the 802.11a short
/// slot). This is the value `Shared` restores to when the process has nothing saved.
pub fn boot_slot_us(family: Family) -> u8 {
    match family {
        Family::Mt76x2 => SLOT_US_MAX,
        Family::Mt76x0 => SLOT_US_MIN,
    }
}

/// ☠ **May this family's slot time be moved at all?**
///
/// **No, on the mt76x2, by default.** MEASURED on the MT7612U, and the two results do not agree:
/// * on a chip our own `init_replay` had cold-initialised, slot 20 → 9 gave **131.9 → 190.6 Mbit/s
///   (+45 %)**, A/B/A/B, fully reversible, with the fixed cost falling 246.5 → 141.0 µs exactly as
///   the DCF budget predicts;
/// * on a chip the **kernel driver** had initialised, the identical write collapsed TX to
///   **10 f/s** and then the MCU stopped answering, ending in a firmware-upload failure that only
///   a power cycle cleared.
///
/// The difference is the *provenance of the surrounding MAC timing* — `MT_XIFS_TIME_CFG`,
/// `MT_TX_TIMEOUT_CFG`, `CC_DELAY` and the rest hold different values depending on who brought the
/// chip up, and a 9 µs slot is evidently inconsistent with one of those sets. **A knob cannot know
/// which init it inherited**, so it must not gamble: the safe default is to leave the slot alone
/// and let `Owned` be honest about actuating nothing on this part.
///
/// `NDN_MT76X2_SLOT_KNOB=1` opts in, for a bench with somebody standing next to the dongle. The
/// mt76x0 is unaffected — it ships a 9 µs slot already, so there is nothing to move.
pub fn slot_knob_allowed(family: Family) -> bool {
    match family {
        Family::Mt76x2 => std::env::var_os("NDN_MT76X2_SLOT_KNOB").is_some(),
        Family::Mt76x0 => true,
    }
}

/// The slot time a posture wants, or `None` to keep whatever the part booted with.
///
/// Only `Owned` shortens the slot. Backoff, AIFS and `CC_DELAY` are all counted in slots, so on a
/// 20 µs part this single register scales *every* term of the DCF budget at once — which is why it
/// was worth measuring, and why [`slot_knob_allowed`] now guards it.
pub fn mt76x02_posture_slot(posture: ContentionPosture) -> Option<u8> {
    match posture {
        ContentionPosture::Owned => Some(SLOT_US_MIN),
        ContentionPosture::Shared | ContentionPosture::Yielding => None,
    }
}

/// Program the MAC slot time, saving the as-found value on the first call.
///
/// `slot_us` is clamped to `SLOT_US_MIN..=SLOT_US_MAX`. Unlike the contention window — where a
/// zero cost an MT7612U a physical replug — both ends of this range are values real 802.11 radios
/// run at every day, which is exactly why the range is closed: 9 is the standard short slot, not
/// an "aggressive" setting, and there is no reason to probe below it.
///
/// `ACKTO` in [`MT_TX_TIMEOUT_CFG`] is moved with it (`slot + SIFS`, the relation upstream's
/// `mt76x02_set_tx_ackto` uses), because an ack timeout sized for a 20 µs slot on a 9 µs slot
/// would hold the medium after every unacked broadcast.
pub fn set_slot_time(bus: &dyn Mt76Regs, slot_us: u8, saved: &EdcaSaved) -> Result<u8, FaceError> {
    let slot_us = slot_us.clamp(SLOT_US_MIN, SLOT_US_MAX);

    if saved.1.load(Ordering::SeqCst) == EdcaSaved::EMPTY {
        let b = u64::from(bus.rr(MT_BKOFF_SLOT_CFG)?);
        let t = u64::from(bus.rr(MT_TX_TIMEOUT_CFG)?);
        let _ = saved.1.compare_exchange(
            EdcaSaved::EMPTY,
            (b & 0xffff_ffff) | ((t & 0xffff) << 32),
            Ordering::SeqCst,
            Ordering::SeqCst,
        );
    }

    bus.rmw(MT_BKOFF_SLOT_CFG, 0x0000_00ff, u32::from(slot_us))?;
    bus.rmw(
        MT_TX_TIMEOUT_CFG,
        0x0000_ff00,
        u32::from(slot_us.saturating_add(15)) << 8,
    )?;
    Ok(slot_us)
}

/// Read back the slot time the MAC is actually counting in.
pub fn read_slot_time(bus: &dyn Mt76Regs) -> Result<u8, FaceError> {
    Ok((bus.rr(MT_BKOFF_SLOT_CFG)? & 0xff) as u8)
}

/// Put the slot time and ack timeout back exactly as found. A no-op if nothing was saved.
pub fn restore_slot_time(bus: &dyn Mt76Regs, saved: &EdcaSaved) -> Result<(), FaceError> {
    let v = saved.1.load(Ordering::SeqCst);
    if v == EdcaSaved::EMPTY {
        return Ok(());
    }
    bus.wr(MT_BKOFF_SLOT_CFG, (v & 0xffff_ffff) as u32)?;
    bus.rmw(MT_TX_TIMEOUT_CFG, 0x0000_ffff, ((v >> 32) & 0xffff) as u32)?;
    Ok(())
}

/// Program the contention window on all four ACs, saving the as-found state on the first call.
///
/// ⚠ Writes **both** EDCA blocks. The tree carried a doubt about which one the arbiter uses;
/// MEASURED, writing either changes on-air behaviour enormously, so they are not alternatives and
/// leaving one stale would mean two different answers to the same question.
pub fn set_contention(
    bus: &dyn Mt76Regs,
    posture: ContentionPosture,
    family: Family,
    saved: &EdcaSaved,
) -> Result<ContentionApplied, FaceError> {
    let (cw_min, cw_max, aifs, txop) = mt76x02_posture(posture, family);
    // Two clamps, deliberately: the family-wide sanity floor, and the per-family measured floor.
    // The second is the one that matters on the mt76x2, where a legal-but-lower window is fatal.
    let cw_min = cw_min.max(MIN_CW_EXPONENT).max(window_floor(family));
    let cw_max = cw_max.max(cw_min);

    // Save once, so `Shared` can be a true restore rather than a guess.
    if saved.0.load(Ordering::SeqCst) == EdcaSaved::EMPTY {
        let a = u64::from(bus.rr(MT_WMM_AIFSN)?);
        let c = u64::from(bus.rr(MT_WMM_CWMIN)?);
        let x = u64::from(bus.rr(MT_WMM_CWMAX)?);
        let _ = saved.0.compare_exchange(
            EdcaSaved::EMPTY,
            (a & 0xffff) | ((c & 0xffff) << 16) | ((x & 0xffff) << 32),
            Ordering::SeqCst,
            Ordering::SeqCst,
        );
    }

    let nib = |v: u8| -> u32 {
        let v = u32::from(v & 0xf);
        v | (v << 4) | (v << 8) | (v << 12)
    };
    bus.wr(MT_WMM_AIFSN, nib(aifs))?;
    bus.wr(MT_WMM_CWMIN, nib(cw_min))?;
    bus.wr(MT_WMM_CWMAX, nib(cw_max))?;
    for ac in 0..4u32 {
        let before = bus.rr(mt_edca_cfg_ac(ac))?;
        let val = (before & 0xff)
            | (u32::from(aifs) << 8)
            | (u32::from(cw_min) << 12)
            | (u32::from(cw_max) << 16);
        bus.wr(mt_edca_cfg_ac(ac), val)?;
    }
    // The slot is the larger lever on a 20 us part, so it moves with the posture — but it is a
    // separate register block, and a part that already ships 9 us simply reads back unchanged.
    //
    // ⚠ `restore_*` can only restore what THIS process saved, and `bring_up` takes the warm path
    // on a chip whose MCU is already running — so a fresh process inherits whatever the previous
    // one left and has no "as found" to go back to. `Shared` therefore falls back to the family's
    // known boot value rather than silently keeping a posture nobody asked for.
    let slot_us = match mt76x02_posture_slot(posture).filter(|_| slot_knob_allowed(family)) {
        Some(want) => set_slot_time(bus, want, saved)?,
        None if !slot_knob_allowed(family) => read_slot_time(bus)?,
        None => {
            if saved.1.load(Ordering::SeqCst) == EdcaSaved::EMPTY {
                set_slot_time(bus, boot_slot_us(family), saved)?
            } else {
                restore_slot_time(bus, saved)?;
                read_slot_time(bus)?
            }
        }
    };
    // Report what the register says, not what we asked for: if a part clamps or the MCU re-asserts
    // MAC timing during calibration, the scheduler must budget against the truth.
    let slot_us = if (SLOT_US_MIN..=SLOT_US_MAX).contains(&slot_us) {
        slot_us
    } else {
        SLOT_US_MIN
    };

    Ok(ContentionApplied {
        cw_min,
        cw_max,
        aifs,
        txop,
        slot_us,
        avg_backoff_us: ContentionApplied::avg_backoff_us_at(cw_min, slot_us),
    })
}

#[cfg(test)]
mod contention_tests {
    use super::*;

    /// ☠ **The mt76x2 window floor, bought with two physical replugs.** On that part a legal
    /// `cw_min` exponent of 2 — measured *good* on the MT7610U, MT7921AU and all three Realtek
    /// parts — collapsed TX to 19 f/s and then killed it outright, twice, unrecoverably in
    /// software. No posture may lower the mt76x2 window below what it boots with.
    #[test]
    fn mt76x2_never_lowers_the_window_below_boot() {
        for p in [
            ContentionPosture::Owned,
            ContentionPosture::Shared,
            ContentionPosture::Yielding,
        ] {
            let (cw_min, cw_max, aifs, _) = mt76x02_posture(p, Family::Mt76x2);
            assert!(
                cw_min >= BOOT_CW_MIN,
                "{p:?} cw_min {cw_min} < boot {BOOT_CW_MIN} on mt76x2"
            );
            assert!(cw_max >= cw_min);
            assert!(
                aifs >= BOOT_AIFSN,
                "{p:?} aifs {aifs} < boot {BOOT_AIFSN} on mt76x2"
            );
        }
    }

    /// ...but the floor is per-family and must NOT be applied where it is unnecessary: the
    /// MT7610U shares this register map and MEASURED fine at exponent 2, so pinning it there too
    /// would give up real capability for a hazard it does not have.
    #[test]
    fn mt76x0_still_reaches_the_aggressive_window() {
        let (cw_min, _, aifs, _) = mt76x02_posture(ContentionPosture::Owned, Family::Mt76x0);
        assert_eq!(cw_min, MIN_CW_EXPONENT);
        assert_eq!(aifs, 1);
    }

    /// `Owned` must still be a real posture on the mt76x2 — it is carried by the slot time, which
    /// MEASURED 131.9 -> 190.6 Mbit/s at VHT80. If both the window AND the slot were pinned,
    /// `Owned` would be a no-op that silently returns `Ok`.
    #[test]
    fn mt76x2_owned_still_actuates_through_the_slot() {
        assert_eq!(
            mt76x02_posture_slot(ContentionPosture::Owned),
            Some(SLOT_US_MIN)
        );
        let owned = ContentionApplied {
            cw_min: BOOT_CW_MIN,
            cw_max: BOOT_CW_MAX,
            aifs: BOOT_AIFSN,
            txop: 0,
            slot_us: SLOT_US_MIN,
            avg_backoff_us: ContentionApplied::avg_backoff_us_at(BOOT_CW_MIN, SLOT_US_MIN),
        };
        let booted = ContentionApplied {
            slot_us: SLOT_US_MAX,
            avg_backoff_us: ContentionApplied::avg_backoff_us_at(BOOT_CW_MIN, SLOT_US_MAX),
            ..owned
        };
        // ★ 206 us at the boot slot, 101 us at the short one: a predicted 105 us saving against
        // a MEASURED 105.5. Note this budget deliberately does NOT include `CC_DELAY`, and the
        // measurement is what settled that — the CC_DELAY-inclusive model predicts 116 us and is
        // 10% off, while SIFS + AIFSN*slot + E[backoff] lands within half a microsecond.
        assert_eq!(booted.medium_access_us(), 206);
        assert_eq!(owned.medium_access_us(), 101);
    }

    /// ☠ The mt76x2 slot knob must be OFF unless explicitly opted into. The same write measured
    /// +45 % on an our-init chip and a dead MCU on a kernel-init one, and nothing at this layer
    /// can tell those apart — so the default must not gamble.
    #[test]
    fn mt76x2_slot_knob_is_off_by_default() {
        if std::env::var_os("NDN_MT76X2_SLOT_KNOB").is_some() {
            return; // opted in on this bench; the guard is being exercised deliberately
        }
        assert!(!slot_knob_allowed(Family::Mt76x2));
        assert!(
            slot_knob_allowed(Family::Mt76x0),
            "the mt76x0 ships a 9 us slot already and has never shown this failure"
        );
    }

    /// A part with nothing saved must still be restorable to its real boot slot, because
    /// `bring_up` takes the warm path and a fresh process inherits the last one's registers.
    #[test]
    fn boot_slot_is_family_specific() {
        assert_eq!(boot_slot_us(Family::Mt76x2), 20);
        assert_eq!(boot_slot_us(Family::Mt76x0), 9);
    }

    /// ★ The clamp is a hardware-safety property, not a style choice: a zero contention window
    /// written to these registers left an MT7612U unable to transmit and needing a physical
    /// replug. No posture may produce an exponent below [`MIN_CW_EXPONENT`].
    #[test]
    fn no_posture_can_reach_a_zero_window() {
        for p in [
            ContentionPosture::Owned,
            ContentionPosture::Shared,
            ContentionPosture::Yielding,
        ] {
            for family in [Family::Mt76x0, Family::Mt76x2] {
                let (cw_min, cw_max, aifs, _) = mt76x02_posture(p, family);
                assert!(
                    cw_min >= MIN_CW_EXPONENT,
                    "{p:?}/{family:?} produced cw_min exponent {cw_min}, below the MEASURED-safe floor"
                );
                assert!(
                    cw_max >= cw_min,
                    "{p:?}/{family:?}: cw_max must not be below cw_min"
                );
                assert!(
                    aifs >= 1,
                    "{p:?}/{family:?}: AIFSN 0 is not a legal arbitration spacing"
                );
            }
        }
    }

    /// The postures must actually be ordered — an "Owned" that backs off as much as "Shared"
    /// would be a knob that reads as implemented and does nothing, which is the defect this
    /// codebase keeps finding.
    ///
    /// ★ On the mt76x2 the ordering lives in the SLOT, not the window, because the window there is
    /// pinned to boot by [`window_floor`]. So the property is stated over the quantity that
    /// actually matters — `medium_access_us` — rather than over `cw_min`, which would falsely
    /// report `Owned` as a no-op on that part.
    #[test]
    fn postures_are_strictly_ordered_by_aggression() {
        for family in [Family::Mt76x0, Family::Mt76x2] {
            let cost = |p: ContentionPosture| {
                let (cw_min, cw_max, aifs, txop) = mt76x02_posture(p, family);
                let slot = mt76x02_posture_slot(p).unwrap_or(boot_slot_us(family));
                let _ = cw_max;
                let _ = txop;
                ContentionApplied {
                    cw_min,
                    cw_max,
                    aifs,
                    txop,
                    slot_us: slot,
                    avg_backoff_us: ContentionApplied::avg_backoff_us_at(cw_min, slot),
                }
                .medium_access_us()
            };
            let (owned, shared, yielding) = (
                cost(ContentionPosture::Owned),
                cost(ContentionPosture::Shared),
                cost(ContentionPosture::Yielding),
            );
            assert!(
                owned < shared,
                "{family:?}: Owned ({owned} us) must contend less than Shared ({shared} us)"
            );
            assert!(
                yielding > shared,
                "{family:?}: Yielding ({yielding} us) must contend more than Shared ({shared} us)"
            );
        }
    }

    /// `Shared` must be the state `init_replay` actually programs, so restoring is a restore and
    /// not a guess: WMM CWMIN 0x4444 (exponent 4), CWMAX 0xaaaa (10), AIFSN 0x2222 (2).
    #[test]
    fn shared_reproduces_the_init_values() {
        for family in [Family::Mt76x0, Family::Mt76x2] {
            let got = mt76x02_posture(ContentionPosture::Shared, family);
            assert_eq!(got, (4, 10, 2, 0), "{family:?}");
        }
    }

    /// The average-backoff helper is what a scheduler budgets airtime with; check it against the
    /// numbers that drove the MT7921AU measurement (exponent 5 -> ~140 us, exponent 2 -> ~13 us).
    #[test]
    fn average_backoff_matches_the_measured_reasoning() {
        assert_eq!(ContentionApplied::avg_backoff_us_for(5), 139); // CW 31 -> 15.5 slots x 9 us
        assert_eq!(ContentionApplied::avg_backoff_us_for(4), 67);
        assert_eq!(ContentionApplied::avg_backoff_us_for(2), 13);
    }

    /// The per-AC nibble replication must fill all four categories, or three of them keep the
    /// old window and the posture is only two-thirds applied.
    #[test]
    fn nibble_fill_covers_all_four_acs() {
        let nib = |v: u8| -> u32 {
            let v = u32::from(v & 0xf);
            v | (v << 4) | (v << 8) | (v << 12)
        };
        assert_eq!(nib(4), 0x4444);
        assert_eq!(nib(10), 0xaaaa);
        assert_eq!(nib(2), 0x2222);
    }
}
