//! **§6.3 `tests/plan_shape.rs`, scoped to M6's four bearers** (contract §5-M6): the AR9271, the
//! two serial-bridge Wi-Fi radios, the 7E-A5 LoRa fleet, and the two HaLow radios.
//!
//! Hardware-free. It reads each `Plan` the way a reviewer would and asserts the properties the
//! contract says a plan must have — the ones that are *not* already compile errors, plus a belt
//! over the ones that are (a `const` panic carries a literal message only, so the compile error
//! says *which rule* broke and not which rung).
//!
//! ⚠ Scope. §6.3's full sweep — *every* backend with a `FrameIo` impl declares a plan or a written
//! `bringup_coverage` exclusion — is M9. This file asserts what M6 actually landed.
//!
//! ⚠ The HaLow plans live in `crate::halow`, which is platform-neutral **on purpose**, so they are
//! tested on every host including this one. The Linux `open_radio` constructors that run them are
//! not reachable from a macOS bench and are not covered here — see the M6 report's open items.

use ndn_radio_drivers::halow::{Mm6108BringUp, Nrc7292BringUp, PLAN_MM6108, PLAN_NRC7292};
use ndn_radio_drivers::{Ath9kHtcBackend, PLAN_AR9271_MONITOR, PLAN_AR9271_RX};
#[cfg(feature = "lora")]
use ndn_radio_drivers::{LoraSerialBackend, PLAN_LORA_NODE};
#[cfg(feature = "serial-radio")]
use ndn_radio_drivers::{PLAN_SERIAL_BRIDGE, SerialRadioBackend};
use ndn_radio_hal::bringup::{BringUp, Plan, Role, Stage, StepClass};

// ── the ladders, transcribed independently of the plans ──────────────────────
//
// ★ Written from the ORIGINAL `open_ath9k` rather than read out of the plan, so a reordering shows
// up as a diff between two lists rather than as nothing at all. M6's claim about the AR9271 is
// transcription; the other three plans have no prior ladder to transcribe (see below).

const LADDER_AR9271: &[&str] = &[
    "download_firmware",
    "htc_init",
    "select_gain_table",
    "hw_reset",
    "connect_data_services",
    "wmi_start",
    "start_receive",
    "note_channel",
    "board_cal",
    "power_cal",
];

fn ids<B: 'static>(p: &Plan<B>) -> Vec<&'static str> {
    p.steps.iter().map(|s| s.id.as_str()).collect()
}

// ── the plans hold together ──────────────────────────────────────────────────

/// No empty `why`, no blank degradation, no duplicate id, and every `must_follow`/`must_precede`
/// names a rung the plan contains **and** is satisfied by the sequence.
#[test]
fn every_m6_plan_holds_together() {
    macro_rules! check {
        ($p:expr) => {
            $p.check()
                .unwrap_or_else(|e| panic!("{} does not hold together: {e}", $p.id))
        };
    }
    check!(PLAN_AR9271_MONITOR);
    check!(PLAN_AR9271_RX);
    #[cfg(feature = "serial-radio")]
    check!(PLAN_SERIAL_BRIDGE);
    #[cfg(feature = "lora")]
    check!(PLAN_LORA_NODE);
    check!(PLAN_NRC7292);
    check!(PLAN_MM6108);
}

/// ★ **The AR9271 plan IS `open_ath9k`'s ladder** — same rungs, same order, nothing added,
/// nothing dropped.
#[test]
fn the_ar9271_plan_is_the_transcribed_ladder() {
    assert_eq!(
        ids(&PLAN_AR9271_MONITOR),
        LADDER_AR9271,
        "{} has drifted from the ladder `open_ath9k` ran. Every rung was transcribed VERBATIM, IN \
         ORDER; a plan that 'improves' a ladder is an unmeasured change to a radio nobody at the \
         keyboard can test.",
        PLAN_AR9271_MONITOR.id
    );
}

/// ★★ **The three rungs §5-M6 names as ones NO example caller performs must be in the plan.**
///
/// This is the whole reason the AR9271 gets a plan rather than staying a function: the knowledge
/// that a high-power module needs the HIGH gain table (~50 dB), that our own firmware's Tier-0
/// name filter has to be disarmed, and that the board/OLPC cal composes with the table, lived in
/// `open_ath9k` and nowhere else — so no example could be compared against it.
#[test]
fn the_ar9271_keeps_the_three_rungs_no_example_performs() {
    for want in ["select_gain_table", "board_cal", "power_cal"] {
        assert!(
            ids(&PLAN_AR9271_MONITOR).contains(&want),
            "PLAN_AR9271_MONITOR lost `{want}` — §5-M6 names it as a step none of this part's ~20 \
             example callers perform, which is exactly why it must live in the plan"
        );
    }
}

/// ★ **The gain table is selected BEFORE the initvals are streamed.**
///
/// MEASURED: a high-power module on the NORMAL table radiates ~50 dB low. `apply_initvals` (inside
/// `hw_reset`) streams whichever table was selected, so selecting after it is selecting nothing.
/// A compile error already guards this; the belt names the rung.
#[test]
fn the_gain_table_precedes_hw_reset() {
    let seq = ids(&PLAN_AR9271_MONITOR);
    let gain = seq.iter().position(|s| *s == "select_gain_table").unwrap();
    let reset = seq.iter().position(|s| *s == "hw_reset").unwrap();
    assert!(
        gain < reset,
        "select_gain_table must precede hw_reset: `apply_initvals` streams the table hw_reset was \
         told to use, so a later selection reaches nothing and a high-power module transmits ~50 \
         dB low while every layer reports Ok"
    );
}

/// ★★ **`AR_RXDP` before `AR_CR_RXE`** — the ordering that was a paragraph in `open_ath9k`.
///
/// `wmi_start` programs the target's RX descriptor ring; `start_receive` enables host RX DMA. The
/// other order latches a stale/zero pointer and the ring never advances (MEASURED seen=0).
#[test]
fn wmi_start_precedes_start_receive() {
    let seq = ids(&PLAN_AR9271_MONITOR);
    let wmi = seq.iter().position(|s| *s == "wmi_start").unwrap();
    let rx = seq.iter().position(|s| *s == "start_receive").unwrap();
    assert!(
        wmi < rx,
        "wmi_start must precede start_receive: WMI_START_RECV programs AR_RXDP and the host's \
         AR_CR_RXE must come after it, or the ring latches a stale pointer and never advances"
    );
}

/// ★ **`rx_enable` is reachable ONLY through `Role::ReceiveOnly`** (§5-M6).
///
/// It is an alternative to `wmi_start` + `start_receive`, not a companion: it arms
/// `AR_IMR_S0 = 0x0001_0000`, i.e. no TXOK for the data queues, and it never sends `IC_UPDATE` or
/// creates the monitor vif/node. A transmitting plan that took it would block at the ring depth.
#[test]
fn rx_enable_belongs_to_receive_only() {
    assert!(
        ids(&PLAN_AR9271_RX).contains(&"rx_enable"),
        "the ReceiveOnly plan must be the one that owns `rx_enable`"
    );
    assert!(
        !ids(&PLAN_AR9271_MONITOR).contains(&"rx_enable"),
        "the transmitting plan must NOT contain `rx_enable`: it arms no TXOK for the data queues \
         and creates no target node, so injection would block at the ring depth on the 34th frame"
    );
    for tx_only in ["wmi_start", "start_receive", "board_cal", "power_cal"] {
        assert!(
            !ids(&PLAN_AR9271_RX).contains(&tx_only),
            "the ReceiveOnly plan must not contain `{tx_only}` — see PLAN_AR9271_RX's written \
             Stage::TxEnable / Stage::Calibrate exclusions"
        );
    }
}

/// The shared prefix is **literally shared**: the two AR9271 roles agree rung for rung up to the
/// point they diverge. Two ladders that were "the same except…" is the defect the contract removes.
#[test]
fn the_two_ar9271_roles_share_their_prefix() {
    let tx = ids(&PLAN_AR9271_MONITOR);
    let rx = ids(&PLAN_AR9271_RX);
    let shared = [
        "download_firmware",
        "htc_init",
        "select_gain_table",
        "hw_reset",
        "connect_data_services",
    ];
    assert_eq!(&tx[..shared.len()], &shared[..]);
    assert_eq!(&rx[..shared.len()], &shared[..]);
}

// ── every `why` is non-empty, on every M6 plan ───────────────────────────────

/// `coverage::Seam::Excluded`'s rule applied to rungs, to the §1.5 asserts and to the written
/// exclusions: a blank is a decision nobody made.
#[test]
fn every_m6_why_is_written() {
    macro_rules! whys {
        ($p:expr) => {{
            for s in $p.steps {
                assert!(
                    !s.why.trim().is_empty(),
                    "{}::{} has an empty `why`",
                    $p.id,
                    s.id
                );
            }
            for (stage, why) in $p.excluded {
                assert!(
                    !why.trim().is_empty(),
                    "{} excludes {stage:?} with no ruling",
                    $p.id
                );
            }
        }};
    }
    whys!(PLAN_AR9271_MONITOR);
    whys!(PLAN_AR9271_RX);
    #[cfg(feature = "serial-radio")]
    whys!(PLAN_SERIAL_BRIDGE);
    #[cfg(feature = "lora")]
    whys!(PLAN_LORA_NODE);
    whys!(PLAN_NRC7292);
    whys!(PLAN_MM6108);

    for a in <Ath9kHtcBackend as BringUp>::asserts() {
        assert!(!a.why.trim().is_empty(), "ath9k assert {} has no why", a.id);
    }
    for a in <Nrc7292BringUp as BringUp>::asserts() {
        assert!(!a.why.trim().is_empty(), "nrc assert {} has no why", a.id);
    }
    for a in <Mm6108BringUp as BringUp>::asserts() {
        assert!(!a.why.trim().is_empty(), "morse assert {} has no why", a.id);
    }
}

// ── the roles a part refuses, refuses by NAME ────────────────────────────────

/// A role a part does not do returns `None` — a named refusal, never a silent downgrade to a
/// different plan.
#[test]
fn unsupported_roles_are_refused_not_downgraded() {
    assert!(<Ath9kHtcBackend as BringUp>::plan(Role::TransmitOnly).is_none());
    #[cfg(feature = "serial-radio")]
    {
        assert!(<SerialRadioBackend as BringUp>::plan(Role::ReceiveOnly).is_none());
        assert!(<SerialRadioBackend as BringUp>::plan(Role::TransmitOnly).is_none());
    }
    #[cfg(feature = "lora")]
    assert!(<LoraSerialBackend as BringUp>::plan(Role::ReceiveOnly).is_none());
    assert!(<Nrc7292BringUp as BringUp>::plan(Role::ReceiveOnly).is_none());
    assert!(<Mm6108BringUp as BringUp>::plan(Role::TransmitOnly).is_none());
}

/// Every plan a part hands back is the plan for the role that was asked for. A role/plan mismatch
/// is caught by `run_plan` at runtime; catching it here costs nothing and reads better.
#[test]
fn every_plan_declares_the_role_it_was_selected_for() {
    for role in [
        Role::ReceiveOnly,
        Role::TransmitOnly,
        Role::TransmitAndReceive,
    ] {
        if let Some(p) = <Ath9kHtcBackend as BringUp>::plan(role) {
            assert_eq!(p.role, role, "{} declares the wrong role", p.id);
        }
        #[cfg(feature = "serial-radio")]
        if let Some(p) = <SerialRadioBackend as BringUp>::plan(role) {
            assert_eq!(p.role, role, "{} declares the wrong role", p.id);
        }
        #[cfg(feature = "lora")]
        if let Some(p) = <LoraSerialBackend as BringUp>::plan(role) {
            assert_eq!(p.role, role, "{} declares the wrong role", p.id);
        }
        if let Some(p) = <Nrc7292BringUp as BringUp>::plan(role) {
            assert_eq!(p.role, role, "{} declares the wrong role", p.id);
        }
        if let Some(p) = <Mm6108BringUp as BringUp>::plan(role) {
            assert_eq!(p.role, role, "{} declares the wrong role", p.id);
        }
    }
}

// ── §5-M6's own constraints, as tests ────────────────────────────────────────

/// ★★ **The serial plan must not change what goes on air.**
///
/// §5-M6: *"A serial radio changing its on-air format is a wire change affecting both ends. Do NOT
/// change what goes on air; make the plan declare the current behaviour."* So the format rung is
/// an `Assert` (a readback), never a `Required` write, and the only rung allowed to reach the wire
/// is the channel — which the caller has to ask for by naming a non-zero channel.
#[cfg(feature = "serial-radio")]
#[test]
fn the_serial_plan_declares_rather_than_writes() {
    let fmt = PLAN_SERIAL_BRIDGE
        .steps
        .iter()
        .find(|s| s.id.as_str() == "frame_format")
        .expect("the serial plan must declare its on-air format");
    assert!(
        matches!(fmt.class, StepClass::Assert),
        "`frame_format` must be an Assert — a readback. Forcing the canonical format would be a \
         WIRE CHANGE affecting a peer this process cannot see, and it is byte-identical anyway \
         because `SerialRadioBackend::open_inner` already takes `FrameFormat::default()`"
    );
    let clock = PLAN_SERIAL_BRIDGE
        .steps
        .iter()
        .find(|s| s.id.as_str() == "clock_domain")
        .expect("§5-M6 asks the serial arms for a clock domain, uniformly");
    assert!(matches!(clock.class, StepClass::Assert));
    // Exactly one rung may put bytes on the wire.
    let writers: Vec<&str> = PLAN_SERIAL_BRIDGE
        .steps
        .iter()
        .filter(|s| matches!(s.class, StepClass::Required))
        .map(|s| s.id.as_str())
        .collect();
    assert_eq!(
        writers,
        vec!["set_channel"],
        "only the channel rung may reach the wire on a serial radio; everything else declares"
    );
}

/// ★★ **Every HaLow rung is `OutOfBand`, and names who established the state.**
///
/// §5-M6 is explicit that this is better than shell history and worse than owning the sequence.
/// A rung here that claimed `Required` would be claiming this crate brought a HaLow radio up,
/// which it does not.
#[test]
fn every_halow_rung_is_out_of_band_and_names_its_establisher() {
    macro_rules! all_out_of_band {
        ($p:expr) => {
            for s in $p.steps {
                match s.class {
                    StepClass::OutOfBand { established_by } => assert!(
                        !established_by.trim().is_empty(),
                        "{}::{} is OutOfBand with no establisher named — the whole value of this \
                         plan is that it says WHO set the state it cannot verify",
                        $p.id,
                        s.id
                    ),
                    other => panic!(
                        "{}::{} is {other:?}, not OutOfBand. Nothing in this crate brings a HaLow \
                         radio up: `modprobe`, `iw`, `hostapd_s1g` and `morse_cli` do, in a shell \
                         this process never saw. Claiming otherwise is the defect the contract \
                         removes.",
                        $p.id,
                        s.id
                    ),
                }
            }
        };
    }
    all_out_of_band!(PLAN_NRC7292);
    all_out_of_band!(PLAN_MM6108);
}

/// The MM6108's split data plane is the hazard, so **both** interfaces are read back; the
/// NRC7292 has one netdev and reads back one.
#[test]
fn halow_asserts_cover_the_interfaces_that_can_silently_lie() {
    let nrc: Vec<&str> = <Nrc7292BringUp as BringUp>::asserts()
        .iter()
        .map(|a| a.id.as_str())
        .collect();
    assert_eq!(nrc, vec!["rx_is_radiotap"]);
    let morse: Vec<&str> = <Mm6108BringUp as BringUp>::asserts()
        .iter()
        .map(|a| a.id.as_str())
        .collect();
    assert_eq!(
        morse,
        vec!["rx_is_radiotap", "tx_is_radiotap"],
        "the MM6108 must read BOTH interfaces back: its TX monitor vif is what turns RECEIVE on \
         (mors->monitor_mode), so a vif that went down takes the receiver with it, silently"
    );
}

/// The LoRa plan's `OutOfBand` rungs are the three probes `open_inner` runs before `Self` exists,
/// and the contract reserves `OutOfBand` for shell tooling — so the exception is asserted here
/// rather than left to a reader to notice.
#[cfg(feature = "lora")]
#[test]
fn the_lora_plan_marks_its_pre_handle_probes_out_of_band() {
    let out_of_band: Vec<&str> = PLAN_LORA_NODE
        .steps
        .iter()
        .filter(|s| matches!(s.class, StepClass::OutOfBand { .. }))
        .map(|s| s.id.as_str())
        .collect();
    assert_eq!(
        out_of_band,
        vec!["capability_probe", "clock_reference", "params_programmed"],
        "these three are `LoraSerialBackend::open_inner`'s CMD_GET_CAP / CMD_GET_CLOCK_REF / \
         configure() exchanges, which must complete before `Self` exists and therefore cannot be \
         rungs. If this list changed, the M6 report's open item about splitting `open_inner` \
         changed with it."
    );
    assert!(
        PLAN_LORA_NODE
            .steps
            .iter()
            .any(|s| s.id.as_str() == "set_channel" && matches!(s.class, StepClass::Required)),
        "the one LoRa rung that acts must be the channel"
    );
}

/// Every M6 plan writes down what it deliberately does nothing in. A blank stage is a written
/// decision, not an absence.
#[test]
fn every_m6_plan_writes_down_its_exclusions() {
    for (id, excluded) in [
        (PLAN_AR9271_MONITOR.id, PLAN_AR9271_MONITOR.excluded),
        (PLAN_AR9271_RX.id, PLAN_AR9271_RX.excluded),
        (PLAN_NRC7292.id, PLAN_NRC7292.excluded),
        (PLAN_MM6108.id, PLAN_MM6108.excluded),
    ] {
        assert!(
            !excluded.is_empty(),
            "{id} excludes nothing at all, which no plan in this fleet honestly does"
        );
    }
    #[cfg(feature = "serial-radio")]
    assert!(
        !PLAN_SERIAL_BRIDGE.excluded.is_empty(),
        "the serial plan excludes nothing at all, which no plan in this fleet honestly does"
    );
    #[cfg(feature = "lora")]
    assert!(
        !PLAN_LORA_NODE.excluded.is_empty(),
        "the LoRa plan excludes nothing at all, which no plan in this fleet honestly does"
    );
    // The HaLow parts must say, in writing, that they set no power — because both DO have a real
    // absolute-dBm axis and a reader could otherwise infer the plan chose one.
    for (id, excluded) in [
        (PLAN_NRC7292.id, PLAN_NRC7292.excluded),
        (PLAN_MM6108.id, PLAN_MM6108.excluded),
    ] {
        assert!(
            excluded.iter().any(|(s, _)| *s == Stage::Power),
            "{id} must state that it sets no power: this bearer HAS an absolute-dBm axis, so \
             silence would read as a choice"
        );
    }
}
