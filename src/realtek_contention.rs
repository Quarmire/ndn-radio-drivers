//! Contention as a knob, for the Realtek parts.
//!
//! The mt76x02 side of this lives in [`crate::mt76::knobs`]; this is the same contract over the
//! Realtek MAC's four-AC EDCA block. It exists for two reasons.
//!
//! **The first is that the knob was missing.** Three Realtek backends implement
//! [`ndn_radio_hal::RadioKnobs`] and none of them implemented `set_contention`, so on every
//! Realtek radio in the fleet the NDR MAC's contention posture reached no actuator — the
//! `decided-but-unactuated` defect this tree keeps rediscovering.
//!
//! **The second is that it was not missing so much as misplaced.** `Rtl8812auBackend::set_cca_ignore`
//! wrote `0x005e_0002` into `REG_EDCA_BE_PARAM` — TXOP 0x5e, **ECWmax 0, ECWmin 0**, AIFS 2 µs —
//! as a side effect of a knob named for *carrier sensing*. Three things are wrong with that, in
//! increasing order of seriousness:
//!
//! 1. A caller asking to ignore ED-CCA did not ask to abolish its backoff. Two mechanisms behind
//!    one name means neither can be reasoned about, and the contention half was invisible to
//!    anything reading the knob list.
//! 2. `AIFS = 2 µs` is shorter than SIFS. This field is in microseconds on Realtek
//!    (`AIFS = SIFS + AIFSN × slot`; the stock BE value `0x005ea42b` is 0x2b = 43 = 16 + 3×9), so
//!    the write did not mean "AIFSN 2", it meant a value no 802.11 station may use.
//! 3. **A zero contention window is the exact configuration that cost an MT7612U a physical
//!    replug** (MEASURED 2026-08-28: 119 → 21 Mbit/s, then the MAC stopped transmitting and the
//!    kernel's own probe failed with `firmware upload failed: -110`). The Realtek MAC is not the
//!    mt76x02 MAC and has never been observed to fail this way — but nothing had *tested* the
//!    proposition, and the write was reachable automatically: `set_edcca_ignore` ←
//!    `apply_knobs` ← cognition's `edcca_ignore: priority == Urgent && busy >= busy_high`. It
//!    fired hardest exactly when the channel was busiest, which is when a zero window does the
//!    most harm to everyone else on it.
//!
//! So the mechanism moves here, keeps a floor, and becomes something a scheduler can ask for by
//! name. The measured capability it was reaching for — an 8812au that stops being out-competed by
//! an a81a on a busy channel — is `ContentionPosture::Owned`, now with a legal window.
//!
//! ## MEASURED 2026-08-28, RTL8812AU on mds-o5p-0, ch36, an a81a saturating the same channel
//!
//! A/B/A/B, writing the register on every arm (a control arm that skips the write inherits the
//! previous arm's state — that mistake invalidated an earlier A/B in this campaign):
//!
//! | payload | `Shared` | `Owned` | Δ |
//! |---|---|---|---|
//! | 1400 B, legacy 6M | 412, 401 f/s | 424, 429 f/s | **+4.9 %** |
//! | 200 B, `Throughput` | 1311, 1257 f/s | 1418, 1416 f/s | **+10.4 %** |
//!
//! ★ The percentage is the less interesting half. In the short-frame arm the period went
//! **779 µs → 706 µs, a 73 µs saving, against the 72 µs that `ContentionApplied::medium_access_us`
//! predicts** for 110 → 38 µs of medium access. The DCF budget is not a model of this knob, it is
//! an accurate account of it, to within a microsecond.
//!
//! Two consequences worth carrying:
//!
//! * **A contention knob is a fixed per-frame cost, so its value is set entirely by what fraction
//!   of the period is airtime.** The same 72 µs is +10 % on 200 B frames and +4.9 % at legacy 6M,
//!   and would be under 1 % on a 5650 B VHT80 PPDU. Anything choosing a posture should weigh it
//!   against frame length, not apply it unconditionally.
//! * The ~706 µs floor that remains at 200 B is **not** contention — it is USB and driver cost.
//!   Contention was 110 µs of a 779 µs period; the other 630 µs is where this part's real ceiling
//!   lives, and no EDCA value will move it.

use std::sync::atomic::{AtomicU64, Ordering};

use ndn_radio_hal::{ContentionApplied, ContentionPosture, FaceError, SIFS_US};

/// `REG_EDCA_VO_PARAM`. Layout for all four: `TXOP[31:16] | ECWmax[15:12] | ECWmin[11:8] | AIFS[7:0]`.
pub const REG_EDCA_VO_PARAM: u16 = 0x0500;
/// `REG_EDCA_VI_PARAM`.
pub const REG_EDCA_VI_PARAM: u16 = 0x0504;
/// `REG_EDCA_BE_PARAM` — best effort, the queue our data actually uses.
pub const REG_EDCA_BE_PARAM: u16 = 0x0508;
/// `REG_EDCA_BK_PARAM`.
pub const REG_EDCA_BK_PARAM: u16 = 0x050c;
/// `REG_SLOT` — MAC slot time in microseconds, one byte. `init_edca_cfg` programs 9.
pub const REG_SLOT: u16 = 0x051b;

/// The four AC parameter registers, in the order `set_contention` writes them.
pub const EDCA_REGS: [u16; 4] = [
    REG_EDCA_VO_PARAM,
    REG_EDCA_VI_PARAM,
    REG_EDCA_BE_PARAM,
    REG_EDCA_BK_PARAM,
];

/// ★ **The floor.** Identical in value and in reason to [`crate::mt76::knobs::MIN_CW_EXPONENT`]:
/// a zero contention window is not "transmit immediately", and on the one family where it was
/// MEASURED it took the radio out until it was physically unplugged. 2 is the smallest exponent
/// measured healthy anywhere in this crate.
pub const MIN_CW_EXPONENT: u8 = 2;

/// The TXOP limit to preserve when reprogramming. `0x005e` is the value the a81a's bring-up
/// leaves in the BE queue and the one the old blast path carried forward.
const DEFAULT_TXOP: u32 = 0x005e;

/// Registers a Realtek backend must expose for the contention knob. All three backends already
/// have these inherent methods with these exact signatures.
pub trait RtlEdcaRegs {
    /// Read a 32-bit MAC register.
    fn rd32(&self, addr: u16) -> Result<u32, FaceError>;
    /// Write a 32-bit MAC register.
    fn wr32(&self, addr: u16, val: u32) -> Result<(), FaceError>;
    /// Read an 8-bit MAC register.
    fn rd8(&self, addr: u16) -> Result<u8, FaceError>;
}

/// The as-found EDCA state, so a posture change can be undone. Same discipline as the mt76 side:
/// the sentinel is a value the registers cannot hold.
#[derive(Debug)]
pub struct RtlEdcaSaved(AtomicU64, AtomicU64);

impl RtlEdcaSaved {
    const EMPTY: u64 = u64::MAX;
    /// A slot with nothing saved in it.
    pub const fn new() -> Self {
        Self(AtomicU64::new(Self::EMPTY), AtomicU64::new(Self::EMPTY))
    }
}

impl Default for RtlEdcaSaved {
    fn default() -> Self {
        Self::new()
    }
}

/// Map a posture onto `(cw_min, cw_max, aifsn)` for the Realtek MAC.
///
/// `Shared` reproduces the stock `0x005ea42b` (ECWmin 4, ECWmax 10, AIFSN 3), so "back to normal"
/// means the value the vendor bring-up programs rather than a value from a standards table.
pub fn rtl_posture(posture: ContentionPosture) -> (u8, u8, u8) {
    match posture {
        // The old blast's intent, at a legal window: a short backoff, not no backoff.
        ContentionPosture::Owned => (MIN_CW_EXPONENT, 4, 1),
        // Stock: what `init_edca_cfg` and the vendor driver leave behind.
        ContentionPosture::Shared => (4, 10, 3),
        // Deliberately deferential, for coexistence and anti-starvation.
        ContentionPosture::Yielding => (6, 10, 7),
    }
}

/// Encode one AC parameter word.
///
/// ⚠ `AIFS` is in **microseconds** here, not in slots — the trap the old blast fell into. The
/// conversion is `SIFS + AIFSN × slot`, which is why this needs the slot time and why that is
/// read from the chip rather than assumed.
pub fn encode_ac(cw_min: u8, cw_max: u8, aifsn: u8, slot_us: u8, txop: u32) -> u32 {
    let aifs_us = u32::from(SIFS_US as u8) + u32::from(aifsn) * u32::from(slot_us);
    ((txop & 0xffff) << 16)
        | ((u32::from(cw_max) & 0xf) << 12)
        | ((u32::from(cw_min) & 0xf) << 8)
        | (aifs_us & 0xff)
}

/// Program the contention window on all four ACs, saving the as-found state on the first call.
///
/// ★ **The returned [`ContentionApplied`] describes the BE (best-effort) queue** — the one our data
/// frames ride. It is a single tuple and the MAC has four access categories which a real radio does
/// not boot with the same values in (MEASURED on the a81a: VO `0x002fa226`, VI `0x005ea328`,
/// BE `0x005ea42b`, BK `0x0000a44f`, each with its own AIFS and TXOP), so one tuple can only
/// honestly describe one queue.
///
/// [`ContentionPosture::Shared`] restores every AC to the value it **booted** with rather than
/// writing the computed word everywhere: a uniform rewrite would permanently destroy the vendor's
/// per-AC tuning after the first `Owned`, and those boot values are per-part facts no formula
/// predicts. A verifier that checks VO/VI/BK against [`encode_ac`] will therefore see a "mismatch"
/// on a perfectly correct restore — check **BE**.
pub fn set_contention(
    dev: &dyn RtlEdcaRegs,
    posture: ContentionPosture,
    saved: &RtlEdcaSaved,
) -> Result<ContentionApplied, FaceError> {
    let (cw_min, cw_max, aifsn) = rtl_posture(posture);
    let cw_min = cw_min.max(MIN_CW_EXPONENT);
    let cw_max = cw_max.max(cw_min);

    // Save all four ACs once, packed 16 bits of significant state each across two cells.
    if saved.0.load(Ordering::SeqCst) == RtlEdcaSaved::EMPTY {
        let mut lo = 0u64;
        let mut hi = 0u64;
        for (i, reg) in EDCA_REGS.iter().enumerate() {
            let v = u64::from(dev.rd32(*reg)?);
            if i < 2 {
                lo |= v << (32 * i);
            } else {
                hi |= v << (32 * (i - 2));
            }
        }
        let _ =
            saved
                .0
                .compare_exchange(RtlEdcaSaved::EMPTY, lo, Ordering::SeqCst, Ordering::SeqCst);
        let _ =
            saved
                .1
                .compare_exchange(RtlEdcaSaved::EMPTY, hi, Ordering::SeqCst, Ordering::SeqCst);
    }

    // Read the slot rather than assume it. Realtek bring-up programs 9, but the MT7612U taught
    // this crate that a boot-time slot is a per-part fact and not a constant.
    let slot_us = dev.rd8(REG_SLOT).unwrap_or(9);
    let slot_us = if (9..=20).contains(&slot_us) {
        slot_us
    } else {
        9
    };

    if posture == ContentionPosture::Shared {
        // A true restore, if we have one: put back exactly what the radio booted with.
        let lo = saved.0.load(Ordering::SeqCst);
        let hi = saved.1.load(Ordering::SeqCst);
        if lo != RtlEdcaSaved::EMPTY && hi != RtlEdcaSaved::EMPTY {
            for (i, reg) in EDCA_REGS.iter().enumerate() {
                let v = if i < 2 {
                    (lo >> (32 * i)) as u32
                } else {
                    (hi >> (32 * (i - 2))) as u32
                };
                dev.wr32(*reg, v)?;
            }
            return Ok(applied(cw_min, cw_max, aifsn, slot_us));
        }
    }

    for reg in EDCA_REGS {
        let txop = dev
            .rd32(reg)
            .map(|v| (v >> 16) & 0xffff)
            .unwrap_or(DEFAULT_TXOP);
        dev.wr32(reg, encode_ac(cw_min, cw_max, aifsn, slot_us, txop))?;
    }
    Ok(applied(cw_min, cw_max, aifsn, slot_us))
}

fn applied(cw_min: u8, cw_max: u8, aifs: u8, slot_us: u8) -> ContentionApplied {
    ContentionApplied {
        cw_min,
        cw_max,
        aifs,
        txop: 0,
        slot_us,
        avg_backoff_us: ContentionApplied::avg_backoff_us_at(cw_min, slot_us),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// ★ The property the old blast violated. No posture may reach a zero window, on any part.
    #[test]
    fn no_posture_can_reach_a_zero_window() {
        for p in [
            ContentionPosture::Owned,
            ContentionPosture::Shared,
            ContentionPosture::Yielding,
        ] {
            let (cw_min, cw_max, _) = rtl_posture(p);
            assert!(cw_min >= MIN_CW_EXPONENT, "{p:?} cw_min {cw_min}");
            assert!(cw_max >= cw_min, "{p:?} cw_max {cw_max} < cw_min {cw_min}");
        }
    }

    /// AIFS is microseconds, not slots. `Shared` must reproduce the stock BE word exactly, or
    /// "restore" is a guess — this is the regression test for the unit confusion in the blast.
    #[test]
    fn shared_reproduces_the_stock_be_word() {
        let (cw_min, cw_max, aifsn) = rtl_posture(ContentionPosture::Shared);
        assert_eq!(encode_ac(cw_min, cw_max, aifsn, 9, 0x005e), 0x005e_a42b);
    }

    /// The blast word, decoded, so the reason it was wrong stays in the test suite: AIFS 2 µs is
    /// below SIFS and both window exponents are zero.
    #[test]
    fn the_old_blast_word_is_not_reachable_from_any_posture() {
        const EDCA_BLAST: u32 = 0x005e_0002;
        assert_eq!(EDCA_BLAST & 0xff, 2, "AIFS 2 us, below the 16 us SIFS");
        assert_eq!((EDCA_BLAST >> 8) & 0xf, 0, "ECWmin 0");
        for p in [
            ContentionPosture::Owned,
            ContentionPosture::Shared,
            ContentionPosture::Yielding,
        ] {
            let (cw_min, cw_max, aifsn) = rtl_posture(p);
            assert_ne!(encode_ac(cw_min, cw_max, aifsn, 9, 0x005e), EDCA_BLAST);
        }
    }

    /// `Owned` must still be genuinely aggressive — the point is a legal window, not a timid one.
    /// CW 3 slots at 9 µs is ~13 µs of average backoff against the stock ~67 µs.
    #[test]
    fn owned_is_still_aggressive() {
        let a = applied(MIN_CW_EXPONENT, 4, 1, 9);
        assert_eq!(a.avg_backoff_us, 13);
        assert_eq!(a.medium_access_us(), 16 + 9 + 13);
    }
}
