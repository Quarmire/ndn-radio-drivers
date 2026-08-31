//! **The backend coverage table** (#79 / plan P3) — every radio shows a full row or a written
//! exclusion; there is no third state.
//!
//! The defect this closes: trait coverage across backends was ragged and *silent* — the rig's
//! highest-throughput radio (mt7612) declared no clock and was invisible to the time plane, and
//! nothing named that gap. A missing impl must be a visible row with a reason, not an absent
//! `None` discovered mid-campaign.
//!
//! Two enforcement mechanisms, split by what each can honestly check:
//! * **`Provided` cells are compile-time-verified**: the test instantiates a trait-bound assertion
//!   for every claimed impl, so this table cannot claim a seam the code does not have. (It CAN
//!   still miss a seam the code has — Rust cannot prove a negative — which is why exclusions are
//!   prose with a reason, reviewed, not derived.)
//! * **`Excluded` cells carry the ruling**: why, decided when, and what would lift it. An
//!   exclusion with an empty reason fails the test.
//!
//! ⚠ **The gate is ONE-DIRECTIONAL, and knowing that is part of using this table.** It prevents
//! *over*-claiming (a `Provided` with no witness fails to compile) but cannot catch *under*-claiming:
//! an `Excluded` cell whose impl quietly lands later adds to neither the `Provided` count nor the
//! witness count, so the equality assertion stays satisfied and the stale exclusion survives. That
//! is not hypothetical — the AR9271 `knobs` cell sat `Excluded("no &self RadioKnobs yet")` for the
//! whole life of a seven-method `impl RadioKnobs for Ath9kHtcBackend`. When you add an impl, come
//! here and flip its cell; nothing else will.
//!
//! ⚠ **`Provided` carries no notion of DEPTH.** One method that errors on almost every input and
//! seven hardware-validated actuators both render as `Provided`. Read the row's backend before
//! reading its cells as a capability claim.
//!
//! **P3.12 ruling — rates for MAC campaigns are PINNED, not adaptive.** The plan offered
//! "RateCalibrator wired or rates pinned per-experiment". Pinned wins for every MAC experiment:
//! an adaptive rate mid-run is a confounder (the arm's airtime changes under it, and airtime is
//! the quantity every MAC property is stated in). The mechanism is `NDN_RADIO_TX_RATE` — a
//! classified Config variable printed in the run header by `ndn-env`, so the pin is part of the
//! run's self-description. `RateCalibrator` stays a cognition-loop concern for non-campaign
//! operation; wiring it into campaign tooling is deferred *with this paragraph as the reason*.

/// One seam's status for one backend: implemented, or excluded in writing.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Seam {
    Provided,
    /// The written exclusion: why this seam is deliberately absent, and (where known) what would
    /// lift it. Empty reasons fail the coverage test.
    Excluded(&'static str),
}

/// One backend's row. `FrameIo` is the price of admission — a backend without it is not a radio
/// this crate can drive and appears with `frame_io: Excluded` and the reason.
pub struct Row {
    pub backend: &'static str,
    /// USB PID(s) this backend claims — referencing each backend's exported PID const where one
    /// exists, so this table cannot drift from the dispatch (a hand-typed copy here would be the
    /// exact silent divergence it polices).
    pub pids: &'static [u16],
    /// In the pre-registered campaign set (plan P5): a campaign radio must show a FULL row —
    /// every seam `Provided` — or be dropped from the campaign, not carried half-described.
    pub campaign: bool,
    pub frame_io: Seam,
    pub knobs: Seam,
    pub time: Seam,
    pub profile: Seam,
}

use Seam::{Excluded, Provided};

/// The table. Order: campaign radios first.
pub const COVERAGE: &[Row] = &[
    Row {
        backend: "LibUsbRtl88xxBackend (RTL8822E \"a81a\")",
        pids: &[0xa81a, 0xa811, 0x8814],
        campaign: true, // the reliable 5 GHz TX (o5p-0)
        frame_io: Provided,
        knobs: Provided,
        time: Provided,
        profile: Provided,
    },
    Row {
        backend: "Rtl8812auBackend",
        pids: &[0x8812, 0x881a],
        campaign: true, // o5p-2 + the 881a on o5p-1 (RX/light roles; brownout under sustained TX)
        frame_io: Provided,
        knobs: Provided,
        // Provided, and SHALLOW in a way this cell cannot show (see the DEPTH warning above): a real
        // per-frame RXTSFL stamp, but its clock REFERENCE is `Unknown` and so `can_common_view` is
        // false. Unlike its two siblings this port never reads a crystal cap and nobody has
        // regressed its TSF against a host or a peer, so nothing in-tree witnesses what the counter
        // runs on. One run of `examples/rxtick.rs` would settle it. See `impl RadioTime for
        // Rtl8812auBackend`.
        //
        // ⚠ **And this radio's common-view observations are LIVE anyway** — the one open gap this
        // table should not hide. `FrameIo::mesh_common_view` (#74/#75) keeps pairing a neighbour's
        // beacon TSF with our RXTSFL, and `ndn-phy-wifi`'s `medium.rs` feeds that straight into
        // `FaceScheduler::ingest_common_view` without consulting `FaceTimeProfile` at all. So the
        // profile's `false` is a declaration nothing enforces, and on an 8812au face slot epochs are
        // being disciplined from a counter whose oscillator nobody has established. Two exits, in
        // order: settle the reference (then there is no contradiction), or gate the consumer (which
        // switches the #75 leaf path off on this radio — a deliberate cost, not a side effect).
        // Written up on `FrameIo::mesh_common_view for Rtl8812auBackend`.
        time: Provided,
        profile: Provided,
    },
    // ── The 7E-A5 serial sub-GHz fleet ─────────────────────────────────────────────────────────
    // ONE backend (`LoraSerialBackend`) drives all three, so the Rust type is the same in every row
    // and the witnesses below repeat. The rows are per NODE on purpose: what a row claims is what
    // that node's `EVT_CAP` (or its pinned `RadioKindHint` profile) makes reachable, and those
    // differ sharply — the LR2021 has no `CMD_SET_FREQ`, the Heltec has neither a hardware stamp nor
    // most of the knob set, the Waveshare has the full knob set. Reading one row as a statement
    // about the type would lose exactly the distinction the capability rewrite exists to carry.
    //
    // ⚠ CORRECTION 2026-08-31: this note used to end "…and the Waveshare has the full knob set and
    // only a software counter". That is false — its TIM3 input capture landed (95/95 frames
    // hardware-stamped, 0 fallbacks) and it declares a `FreeRunRxStamp` like the LR2021's.
    Row {
        backend: "LoraSerialBackend (Waveshare SX1262)",
        pids: &[0x55d3],
        campaign: true, // campaign (c) reports Wi-Fi and LoRa separately
        frame_io: Provided,
        knobs: Provided,
        time: Provided,
        profile: Provided,
    },
    Row {
        backend: "LoraSerialBackend (XIAO nRF54L15 + LR2021)",
        // No PID recorded: this board is reached by path (/dev/ttyACM*), and its USB ids have not
        // been read off the rig. An empty list is the honest entry — a plausible-looking PID here
        // would silently claim a device the dispatch never matches.
        pids: &[],
        // Not in the pre-registered campaign set: the link is live (o5p-0 -> o5p-1 decodes on air
        // at 915 MHz FLRC), but the node has not yet been exercised through this backend end to
        // end, and its 7E-A5 v2 self-description is landing concurrently. Admit it to the campaign
        // when a run has actually driven it through FrameIo, not on the strength of a live link.
        campaign: false,
        frame_io: Provided,
        knobs: Provided,
        // ⚠ **RETRACTED 2026-08-31**, both halves of it. This cell used to read: "the only node in
        // this fleet whose clock cell means what the Realtek ones mean: a free-running per-frame
        // HARDWARE stamp (16 MHz DPPI capture, 62.5 ns MEASURED), so
        // `FaceTimeProfile::can_common_view` is true here and false on every other sub-GHz node."
        //
        // * **"the only node"** — false since the Waveshare's TIM3 input capture landed. It stamps
        //   95/95 frames in silicon at the same `LatchPoint::RadioCapture`, so on the latch axis the
        //   two nodes are now indistinguishable.
        // * **"can_common_view is true here"** — no longer true, and the reason is the point.
        //   MEASURED across two receivers of the same frames: two LR2021s give 0.81-1.86 us and the
        //   residual is FLAT against the fit span (1.11 -> 1.55 us from 1.4 s to 10.2 s); two
        //   Waveshares gave 10.5-20.4 us and GROWING (16 -> 130 us). Same latch, different
        //   references — a crystal versus an 8 MHz internal RC — so the latch point alone stopped
        //   being the test and `RadioTimeSource::reference` became the other half of it.
        //
        // Where that leaves this node — ⚠ and the middle of this cell was itself wrong for a day,
        // so read the correction with the retraction. It said: "this firmware does not implement
        // `CMD_GET_CLOCK_REF` (0x21) … and the host will not promote a reference the node has not
        // stated, because NOTHING ON THE WIRE SEPARATES this build from the pre-2026-08-28 one that
        // MEASURED +2253 ppm on the nRF's internal RC. So `can_common_view` is false here today."
        //
        // That premise is false and `git` is the witness: `EVT_CAP` and `HfclkSource::ExternalXtal`
        // entered `m6_bridge` in the SAME commit (1283b7e — which added the `ExternalXtal` LINE to the older `hw.rs`, and
        // `git log -S hfclk_source -- firmware/` returns that commit alone), and the older RC build
        // speaks 7E-A5 **v1**, which cannot emit an `EVT_CAP` at all. An LR2021 that ANSWERS
        // `CMD_GET_CAP` is therefore necessarily a crystal build, `NodeProfile.learned` already
        // records whether the record came from the device, and `assumed_clock_reference` now reads
        // it (see its doc). So this node — the fleet's only part with a this-session-measured
        // common view, 0.81/1.55/1.86 us FLAT across the fit span — keeps `can_common_view` **true**
        // whenever it has spoken for itself, and loses it only when the host had to pin a profile.
        //
        // And since 4d01eb1 it no longer relies on that inference at all: `m6_bridge` answers 0x21
        // with `[crystal][accuracy unknown]`, MEASURED on both bench boards (`02 ff ff`), which is
        // bit-for-bit the same `ClockReference` the fallback assumes. The Waveshare answers it too.
        // The Heltec SX1276 replies `EVT_UNSUPPORTED` — an honest unknown, and it has no hardware
        // latch either way.
        time: Provided,
        profile: Provided,
    },
    Row {
        backend: "LoraSerialBackend (Heltec LoRa32 V2, SX1276)",
        pids: &[],       // CP2102 bridge; ids not read off the rig (see the LR2021 row)
        campaign: false, // NEVER exercised on air — mds-o5p-2's sshd is down, so the board has not
        // been flashed or measured. Its profile is pinned from firmware source + the SX1276
        // datasheet and must be replaced by the board's own EVT_CAP.
        frame_io: Provided,
        knobs: Provided,
        time: Provided,
        profile: Provided,
    },
    Row {
        backend: "Rtl8733buBackend",
        pids: crate::RTL8733B_PIDS,
        campaign: false,
        frame_io: Provided,
        knobs: Provided,
        time: Provided,
        profile: Provided,
    },
    Row {
        backend: "Mt7921uBackend (MT7921AU, connac2 802.11ax)",
        pids: crate::MT7921U_PIDS,
        campaign: false, // new port; firmware download and RX are the on-air gate
        frame_io: Provided,
        knobs: Provided,
        // ★ The only MediaTek part here whose clock cell means what the Realtek ones mean: a
        // per-frame FreeRunRxStamp from RXD group 2, not a read-now PortTsf. `FaceTimeProfile`
        // derives `can_common_view = true` from it — the first time that is true for a
        // MediaTek radio in this crate.
        //
        // Still true after the reference axis landed (2026-08-31), and now for a stated reason
        // rather than by default: no register in this port names the oscillator, but the counter
        // MEASURED 15_007_757 ticks across 15_007_907 host microseconds = **-10.0 ppm over 15 s**,
        // three orders of magnitude tighter than an RC reference (+2253 ppm on the LR2021's,
        // ~-3100 ppm on the Waveshare's), so it declares `ClockReference::crystal()` carrying that
        // measurement. See `impl RadioTime for Mt7921uBackend`.
        time: Provided,
        profile: Provided,
    },
    Row {
        backend: "Mt7610uBackend (MT7610U, mt76x0)",
        pids: crate::MT7610U_PIDS,
        campaign: false, // new port; on-air validation is the gate, not the trait matrix
        frame_io: Provided,
        knobs: Provided,
        time: Provided, // port TSF (0x111c), MEASURED 1.000 us/tick on this exact part
        profile: Provided,
    },
    Row {
        backend: "Mt7612uBackend",
        pids: crate::MT7612U_PIDS,
        campaign: false, // off the bus since the #110 wedge; needs a replug + the 0x09a8 poll fix
        frame_io: Provided,
        knobs: Provided,
        // Flipped 2026-08-27 with the register map corrected: `impl RadioTime for
        // Mt7612uBackend` now reports a real port TSF (MT_TSF_TIMER_DW0 0x111c, armed by
        // MT_BEACON_TIME_CFG 0x1114 bit 16), MEASURED at 1.000 us/tick on this dongle. The
        // old exclusion is quoted in that impl's doc comment along with why its evidence was
        // void. `can_common_view` is still false and correctly so — a PortTsf is not a
        // per-frame stamp — which the FaceTimeProfile derivation gets right on its own.
        time: Provided,
        profile: Provided,
    },
    Row {
        backend: "Rtl8821cuBackend",
        pids: crate::RTL8821CU_PIDS,
        campaign: false,
        frame_io: Provided,
        knobs: Excluded(
            "#79/#80 ruling (P3.11 confirms): bring-up is validated RX-side only; no control write \
             has been hardware-verified, and declaring knobs without a validated actuator is the \
             decided-but-unactuated defect this project keeps re-finding. Lifted by: a bench \
             session validating set_channel/set_rate writes on the part.",
        ),
        time: Excluded(
            "Same ruling as knobs: nothing true to say until the part is on a bench. \
             (The shared RX pump was also deliberately not ported — #80 — same reason.)",
        ),
        profile: Provided,
    },
    Row {
        backend: "SerialRadioBackend (RTL8720DN)",
        pids: &[], // serial bridge — no USB PID dispatch; opened by device path
        campaign: false,
        frame_io: Provided,
        knobs: Provided,
        // Provided, and it is the HOST clock — SHALLOW in the way this cell cannot show. ⚠ Corrected
        // 2026-08-31: this backend used to declare a `FreeRunRxStamp` (so `hw_rx_stamp = true`, a
        // 1 us floor, `LatchPoint::MacDone`) for a number that is `us_ticker_read()` called by
        // SOFTWARE at the top of the vendor blob's promiscuous RX callback — the MAC's own RXTSFL is
        // in the RX descriptor and the blob drops it first. It now reports the honest host stamp,
        // like the 7E-A5 fleet's software-counter nodes. A `CLOCK_REF_XTAL`-shaped answer from this
        // part would be true and would still not earn common view: it fails the LATCH half.
        time: Provided,
        profile: Provided,
    },
    Row {
        backend: "Ath9kHtcBackend (AR9271)",
        pids: &[0x9271], // AR9271_IDS is (vid,pid) pairs; 0x9271 under Atheros VID
        campaign: false,
        frame_io: Provided, // M3: stamped RX up + TX inject (build_tx_frame_bytes); RX proven on
        // silicon, TX framing bench-uncertain (node/rate handling) — flagged at the impl.
        // STALE-EXCLUSION FIX: this cell read `Excluded("no &self RadioKnobs yet")` long after
        // `impl RadioKnobs for Ath9kHtcBackend` landed with seven actuated methods (live channel +
        // HT20/40 retune, EDCCA force-rx-clear, the SDR-measured TX-gain ladder in both index and
        // dBm, AR_RCCNT occupancy, ScheduledAt discipline, the Tier-0 name filter). The gate below
        // could not catch it: an Excluded cell whose impl quietly appears adds to neither the
        // Provided count nor the witness count, so the two stayed equal. See the module header.
        knobs: Provided,
        time: Provided, // M2: per-frame rs_tstamp is a FreeRunRxStamp common-view clock.
        profile: Provided, // wifi_monitor_2ghz_1ss (1x1 2.4 GHz 11n, ch 1..13).
    },
];

/// Render the table (the visible artifact #79 asked for; printed by the coverage test).
pub fn render() -> String {
    let mut out = String::from(
        "backend                                  campaign  FrameIo  Knobs  Time  Profile\n",
    );
    for r in COVERAGE {
        let cell = |s: &Seam| match s {
            Provided => "yes",
            Excluded(_) => "EXCL",
        };
        out.push_str(&format!(
            "{:<41}{:<10}{:<9}{:<7}{:<6}{}\n",
            r.backend,
            if r.campaign { "YES" } else { "-" },
            cell(&r.frame_io),
            cell(&r.knobs),
            cell(&r.time),
            cell(&r.profile),
        ));
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{RadioKnobs, RadioProfile, RadioTime};
    use ndn_frame_io::FrameIo;

    // Compile-time verification of every `Provided` cell: these functions only compile if the
    // impl exists, so the table cannot overclaim. One line per claimed cell, grouped per backend —
    // adding a `Provided` to the table without adding its line here fails the count check below.
    fn is_frame_io<T: FrameIo>() {}
    fn is_knobs<T: RadioKnobs>() {}
    fn is_time<T: RadioTime>() {}
    fn is_profile<T: RadioProfile>() {}

    fn compile_time_provided_cells() -> usize {
        let mut n = 0;
        macro_rules! claim {
            ($f:ident::<$t:ty>) => {{
                let _ = $f::<$t>;
                n += 1;
            }};
        }
        claim!(is_frame_io::<crate::LibUsbRtl88xxBackend>);
        claim!(is_knobs::<crate::LibUsbRtl88xxBackend>);
        claim!(is_time::<crate::LibUsbRtl88xxBackend>);
        claim!(is_profile::<crate::LibUsbRtl88xxBackend>);
        claim!(is_frame_io::<crate::Rtl8812auBackend>);
        claim!(is_knobs::<crate::Rtl8812auBackend>);
        claim!(is_time::<crate::Rtl8812auBackend>);
        claim!(is_profile::<crate::Rtl8812auBackend>);
        // Feature-gated backends: witnesses ride the gate; without the feature the cells are
        // counted as vacuously witnessed (the table describes the FULL build — campaign tooling
        // builds with these features on).
        // Three ROWS, one TYPE: the Waveshare SX1262, the nRF54L15+LR2021 bridge and the Heltec
        // SX1276 are all driven by `LoraSerialBackend`, so the same four witnesses are claimed once
        // per row. (The witness gate proves the impl exists; which of the three nodes is on the far
        // end is a runtime `NodeProfile` fact no type check can see.)
        #[cfg(feature = "lora")]
        for _node in 0..3 {
            claim!(is_frame_io::<crate::LoraSerialBackend>);
            claim!(is_knobs::<crate::LoraSerialBackend>);
            claim!(is_time::<crate::LoraSerialBackend>);
            claim!(is_profile::<crate::LoraSerialBackend>);
        }
        #[cfg(not(feature = "lora"))]
        {
            n += 12;
        }
        claim!(is_frame_io::<crate::Rtl8733buBackend>);
        claim!(is_knobs::<crate::Rtl8733buBackend>);
        claim!(is_time::<crate::Rtl8733buBackend>);
        claim!(is_profile::<crate::Rtl8733buBackend>);
        claim!(is_frame_io::<crate::Mt7921uBackend>);
        claim!(is_knobs::<crate::Mt7921uBackend>);
        claim!(is_time::<crate::Mt7921uBackend>);
        claim!(is_profile::<crate::Mt7921uBackend>);
        claim!(is_frame_io::<crate::Mt7610uBackend>);
        claim!(is_knobs::<crate::Mt7610uBackend>);
        claim!(is_time::<crate::Mt7610uBackend>);
        claim!(is_profile::<crate::Mt7610uBackend>);
        claim!(is_frame_io::<crate::Mt7612uBackend>);
        claim!(is_knobs::<crate::Mt7612uBackend>);
        claim!(is_time::<crate::Mt7612uBackend>);
        claim!(is_profile::<crate::Mt7612uBackend>);
        claim!(is_frame_io::<crate::Rtl8821cuBackend>);
        claim!(is_profile::<crate::Rtl8821cuBackend>);
        // AR9271 (ath9k_htc) — M3 FrameIo + M2 RX-stamp clock + capability profile + the full
        // knob surface (live retune, EDCCA, the measured dBm ladder, AR_RCCNT occupancy).
        claim!(is_frame_io::<crate::Ath9kHtcBackend>);
        claim!(is_knobs::<crate::Ath9kHtcBackend>);
        claim!(is_time::<crate::Ath9kHtcBackend>);
        claim!(is_profile::<crate::Ath9kHtcBackend>);
        #[cfg(feature = "serial-radio")]
        {
            claim!(is_frame_io::<crate::SerialRadioBackend>);
            claim!(is_knobs::<crate::SerialRadioBackend>);
            claim!(is_time::<crate::SerialRadioBackend>);
            claim!(is_profile::<crate::SerialRadioBackend>);
        }
        #[cfg(not(feature = "serial-radio"))]
        {
            n += 4;
        }
        n
    }

    /// The #79 gate, as a test: every row is full or excluded IN WRITING; every campaign radio is
    /// full; every `Provided` in the table has a compile-time witness above; no PID is claimed
    /// twice. Prints the table so the artifact is visible in test output.
    #[test]
    fn every_backend_shows_a_full_row_or_a_written_exclusion() {
        let mut provided_cells = 0;
        let mut pids_seen = std::collections::HashSet::new();
        for r in COVERAGE {
            for (seam, name) in [
                (&r.frame_io, "frame_io"),
                (&r.knobs, "knobs"),
                (&r.time, "time"),
                (&r.profile, "profile"),
            ] {
                match seam {
                    Provided => provided_cells += 1,
                    Excluded(reason) => {
                        assert!(
                            reason.len() > 40,
                            "{}: {name} excluded without a real written reason — that is the \
                             silent gap #79 exists to remove",
                            r.backend
                        );
                        assert!(
                            !r.campaign,
                            "{}: a CAMPAIGN radio may not carry exclusions ({name}) — full row \
                             or out of the campaign, no third option (P3 gate)",
                            r.backend
                        );
                    }
                }
            }
            for pid in r.pids {
                assert!(pids_seen.insert(*pid), "PID {pid:#06x} claimed by two rows");
            }
        }
        assert_eq!(
            provided_cells,
            compile_time_provided_cells(),
            "the table's Provided count differs from the compile-time witnesses — a cell was \
             claimed without its witness (or a witness added without its cell)"
        );
        println!("{}", render());
    }
}
