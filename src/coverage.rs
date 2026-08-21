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
        time: Provided,
        profile: Provided,
    },
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
        backend: "Rtl8733buBackend",
        pids: crate::RTL8733B_PIDS,
        campaign: false,
        frame_io: Provided,
        knobs: Provided,
        time: Provided,
        profile: Provided,
    },
    Row {
        backend: "Mt7612uBackend",
        pids: crate::MT7612U_PIDS,
        campaign: false, // off the bus since the #110 wedge; needs a replug + the 0x09a8 poll fix
        frame_io: Provided,
        knobs: Provided,
        time: Excluded(
            "MEASURED 2026-08-18: mt76x2 has no usable per-frame RX timestamp. The TSF timer \
             registers (0x1104/0x1108/0x110c) are static even after enabling MT_BEACON_TIME_CFG \
             (0x1100) bit4 TIMER_EN; no receive-latched timestamp field exists in the 36-byte RXD \
             prefix; and a per-frame register read costs ~141us (≈ the 200us common-view guard). So \
             it cannot be a common-view participant — this is architectural, not a missing bench.",
        ),
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
        backend: "Bw16SerialBackend (RTL8720DN)",
        pids: &[], // serial bridge — no USB PID dispatch; opened by device path
        campaign: false,
        frame_io: Provided,
        knobs: Provided,
        time: Provided,
        profile: Provided,
    },
    Row {
        backend: "Ath9kHtcBackend (AR9271)",
        pids: &[0x9271], // AR9271_IDS is (vid,pid) pairs; 0x9271 under Atheros VID
        campaign: false,
        frame_io: Excluded(
            "L1 transport only (USB + firmware download + HTC/WMI handshake) — the open-firmware \
             Tier-0 path (#100/ath9k-htc-ndr) runs the filter ON the dongle; this host backend \
             does not yet replace ath9k_htc as a FrameIo radio. Lifted by: the ndr firmware's TX \
             path landing.",
        ),
        knobs: Excluded(
            "Follows frame_io: control knobs presuppose a data plane to control; the AR9271's \
             knob path will be WMI commands once the ndr firmware's TX path lands.",
        ),
        time: Excluded(
            "Follows frame_io — though the AR9271 is the measured 1.05 µs TimeToken part, so this \
             is the FIRST seam to lift once the backend carries frames: the firmware TX counter \
             path is already validated on air.",
        ),
        profile: Excluded(
            "Follows frame_io: a capability declaration for a backend that cannot carry a frame \
             would be the decided-but-unactuated defect in its purest form.",
        ),
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
        #[cfg(feature = "lora")]
        {
            claim!(is_frame_io::<crate::LoraSerialBackend>);
            claim!(is_knobs::<crate::LoraSerialBackend>);
            claim!(is_time::<crate::LoraSerialBackend>);
            claim!(is_profile::<crate::LoraSerialBackend>);
        }
        #[cfg(not(feature = "lora"))]
        {
            n += 4;
        }
        claim!(is_frame_io::<crate::Rtl8733buBackend>);
        claim!(is_knobs::<crate::Rtl8733buBackend>);
        claim!(is_time::<crate::Rtl8733buBackend>);
        claim!(is_profile::<crate::Rtl8733buBackend>);
        claim!(is_frame_io::<crate::Mt7612uBackend>);
        claim!(is_knobs::<crate::Mt7612uBackend>);
        claim!(is_profile::<crate::Mt7612uBackend>);
        claim!(is_frame_io::<crate::Rtl8821cuBackend>);
        claim!(is_profile::<crate::Rtl8821cuBackend>);
        #[cfg(feature = "bw16")]
        {
            claim!(is_frame_io::<crate::Bw16SerialBackend>);
            claim!(is_knobs::<crate::Bw16SerialBackend>);
            claim!(is_time::<crate::Bw16SerialBackend>);
            claim!(is_profile::<crate::Bw16SerialBackend>);
        }
        #[cfg(not(feature = "bw16"))]
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
            for (seam, name) in
                [(&r.frame_io, "frame_io"), (&r.knobs, "knobs"), (&r.time, "time"), (&r.profile, "profile")]
            {
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
