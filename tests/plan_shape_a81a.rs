//! **§6.3 `tests/plan_shape.rs`, scoped to the RTL8822E / a81a** (contract §5-M4) — the radio the
//! forwarder actually runs on.
//!
//! Hardware-free. It reads the part's `Plan` the way a reviewer would and asserts the properties
//! the contract says a plan must have — the ones that are *not* already compile errors, plus a belt
//! over the ones that are (a `const` panic carries a literal message only, so the compile error
//! says *which rule* broke and not which rung).
//!
//! ⚠ Scope. §6.3's full sweep — *every* backend with a `FrameIo` impl declares a plan or a written
//! `bringup_coverage` exclusion — is M9. This file asserts what M4 actually landed.

use ndn_radio_drivers::{LibUsbRtl88xxBackend, PLAN_A81A};
use ndn_radio_hal::bringup::{BringUp, Role, StepClass};

/// The step ids in the order `bring_up` ran them, transcribed independently of the plan so that a
/// reordering shows up as a diff between two lists rather than as nothing at all.
const LADDER: &[&str] = &[
    "power_on",
    "download_firmware",
    "mac_init",
    "monitor_cfg",
    "send_general_info",
    "phy_init",
    "tune_channel",
    "fw_iqk",
    "lck",
    "fw_dpk",
    "dpk_force_bypass",
    "kfree",
    "txgapk",
    "efem_pinmux",
    "bb_tx_datapath_init",
    "rx_path_init",
    "calibrate_tx_power",
    "retune_channel",
    "btc_grant_wl",
    "thermal_reference",
];

/// The rung bodies — every `s_*` step function, and nothing else. Used by the two source-level
/// tests below, which must read CODE and not the prose that quotes the code it rejects.
fn rung_section(src: &str) -> &str {
    src.split_once("// ── the rungs ─")
        .expect("the rung section")
        .1
        .split_once("// ── the rungs, as reviewable constants")
        .expect("the end of the rung section")
        .0
}

fn ids() -> Vec<&'static str> {
    PLAN_A81A.steps.iter().map(|s| s.id.as_str()).collect()
}

fn at(id: &str) -> usize {
    ids()
        .iter()
        .position(|i| *i == id)
        .unwrap_or_else(|| panic!("PLAN_A81A has no rung `{id}`"))
}

/// The plan holds together: no empty `why`, no blank degradation, no duplicate id, and every
/// `must_follow`/`must_precede` names a rung this plan contains **and** is satisfied by the
/// sequence.
#[test]
fn the_plan_holds_together() {
    PLAN_A81A.check().unwrap_or_else(|e| {
        panic!(
            "{} does not hold together: {e}\n\
             A plan is checked before the first register write; a malformed one must never reach \
             the hardware.",
            PLAN_A81A.id
        )
    });
}

/// ★ **The plan IS the ladder** — same rungs, same order, nothing added, nothing dropped.
///
/// M4's whole claim is transcription: the sequence that has been on air since 2026-06-14 is the
/// sequence the runner walks. If this list and the plan disagree, one of them was edited by
/// somebody who could not test the radio.
#[test]
fn the_plan_is_the_transcribed_ladder() {
    assert_eq!(
        ids(),
        LADDER,
        "PLAN_A81A has drifted from the ladder `bring_up` ran. Every rung was transcribed \
         VERBATIM, IN ORDER; a plan that 'improves' a ladder is an unmeasured change to the radio \
         the forwarder runs on, and nobody at the keyboard can test it."
    );
}

/// **Every `why` is non-empty** — `coverage::Seam::Excluded`'s rule applied to rungs, to the §1.5
/// asserts, and to the written exclusions. A blank cell is a written decision, not an absence.
#[test]
fn every_why_and_every_exclusion_is_written() {
    for s in PLAN_A81A.steps {
        assert!(
            !s.why.trim().is_empty(),
            "{}::{} has no stated reason. The measurement or vendor reference that puts a rung in \
             a ladder is the only thing that stops the next person deleting it — or, worse, \
             copying it into a private bring-up.",
            PLAN_A81A.id,
            s.id
        );
    }
    for (stage, reason) in PLAN_A81A.excluded {
        assert!(
            !reason.trim().is_empty(),
            "{} excludes {stage:?} with no ruling. An empty exclusion is an absence pretending to \
             be a decision.",
            PLAN_A81A.id
        );
    }
    for a in <LibUsbRtl88xxBackend as BringUp>::asserts() {
        assert!(
            !a.why.trim().is_empty(),
            "assert `{}` at {:#06x} has no stated reason — a bus round trip nobody justified",
            a.id,
            a.reg
        );
    }
}

/// ★ **The three orderings that are MEASURED, as constraints rather than prose.**
///
/// Satisfied by position *and* declared, because an undeclared ordering is prose again: reorder the
/// rungs and the crate would still build.
#[test]
fn the_measured_orderings_are_declared_and_satisfied() {
    // ☠ 2026-06-14 bisection on the reciprocal OPi link: the BT-coex Wi-Fi grant forced BEFORE the
    // cal chain corrupts the cal/BB state and gives 0 frames decoded; granted only at the end,
    // ~850. This is the single most expensive ordering fact in this file.
    for earlier in ["fw_iqk", "txgapk", "retune_channel"] {
        assert!(
            at(earlier) < at("btc_grant_wl"),
            "btc_grant_wl runs before `{earlier}`. MEASURED: grant-before-cal → 0 frames decoded; \
             grant only at the end → ~850. Forcing the BTC indirect grant before calibration \
             corrupts the cal/BB state."
        );
    }
    let grant = &PLAN_A81A.steps[at("btc_grant_wl")];
    for earlier in ["fw_iqk", "txgapk", "retune_channel"] {
        assert!(
            grant.must_follow.iter().any(|i| i.as_str() == earlier),
            "btc_grant_wl satisfies the ordering against `{earlier}` but does not declare it"
        );
    }

    // Vendor ordering, and the driver's own doc says why: the gain-K correction must sit on the
    // TRIMMED base, so gain-K first and trim after corrects the wrong reference.
    assert!(at("kfree") < at("txgapk"));
    assert!(
        PLAN_A81A.steps[at("kfree")]
            .must_precede
            .iter()
            .any(|i| i.as_str() == "txgapk"),
        "kfree must DECLARE that it precedes txgapk — the factory trim is the base the gain-K \
         correction sits on"
    );

    // The cal chain is channel-dependent (`txgapk` literally takes the channel), so calibrating
    // before tuning would characterise a channel the radio does not end up on.
    for later in ["fw_iqk", "txgapk"] {
        assert!(at("tune_channel") < at(later));
        assert!(
            PLAN_A81A.steps[at("tune_channel")]
                .must_precede
                .iter()
                .any(|i| i.as_str() == later),
            "tune_channel must DECLARE that it precedes `{later}`"
        );
    }
}

/// **LAW 5**: a rung that polls a hardware completion bit is always `Required`. A timed-out
/// power sequence, firmware boot or auto-LLT that continued leaves a radio whose every later
/// readback is fiction.
#[test]
fn every_completion_poll_is_required() {
    // `power_on` polls each pwrseq entry to its wanted value; `download_firmware` polls BCN_VALID
    // and then REG_MCUFW_CTRL for the 0xC078 fw-ready magic; `mac_init` → `init_trx_cfg` polls
    // REG_AUTO_LLT_V1 BIT0 until the hardware link-list init self-clears.
    for id in ["power_on", "download_firmware", "mac_init"] {
        assert!(
            matches!(PLAN_A81A.steps[at(id)].class, StepClass::Required),
            "PLAN_A81A::{id} polls a hardware completion bit and is not Required (LAW 5)"
        );
    }
}

/// ★ **Every rung that swallowed a failure now names what is lost.**
///
/// The old ladder had four `tracing::warn!` swallows and one bare `if let Ok(_)` that discarded a
/// failure with no log line at all. `StepClass::BestEffort` cannot be spelled without a
/// `Degradation`, so this test is really asserting that the conversion happened on the right rungs.
#[test]
fn the_swallowed_failures_became_named_degradations() {
    for id in [
        "lck",
        "dpk_force_bypass",
        "kfree",
        "txgapk",
        "calibrate_tx_power",
        "btc_grant_wl",
        "thermal_reference",
    ] {
        let s = &PLAN_A81A.steps[at(id)];
        let StepClass::BestEffort(d) = s.class else {
            panic!(
                "PLAN_A81A::{id} swallowed its failure in the old ladder and is not BestEffort. \
                 Either it is now Required (a real decision — say so here) or a real degradation \
                 is being discarded silently again."
            )
        };
        assert!(!d.lost.trim().is_empty() && !d.still_valid_for.trim().is_empty());
    }
}

/// A part says which roles it does, by name. `None` is a named refusal, never a silent downgrade
/// into a role the caller did not ask for.
#[test]
fn the_part_names_the_one_role_it_does() {
    assert_eq!(
        <LibUsbRtl88xxBackend as BringUp>::plan(Role::TransmitAndReceive).map(|p| p.id.name),
        Some("monitor"),
        "all five `open_*` openers run one plan; that is M4's contract"
    );
    for role in [Role::ReceiveOnly, Role::TransmitOnly] {
        assert!(
            <LibUsbRtl88xxBackend as BringUp>::plan(role).is_none(),
            "{role:?} must be a named refusal on this part. One ladder is all it has ever run: an \
             RX-only variant means deleting `bb_tx_datapath_init` and the cal chain, a TX-only one \
             means deleting `monitor_cfg`, and neither has been run on this silicon. Inventing one \
             here is exactly the unmeasured ladder the contract removes."
        );
    }
}

/// ★ **§4: the refusal carries the measurement.**
///
/// This part cannot answer question (A), and the contract's answer is *"`Unprovable`, reason
/// quoted"* — because the two obvious candidate registers were already ruled out and a generic
/// shrug does not stop the next person wiring one of them up again.
#[test]
fn the_transmit_refusal_is_quoted_not_silent() {
    assert!(
        <LibUsbRtl88xxBackend as BringUp>::tx_instruments().is_empty(),
        "0x2DE0 is NOT a TX-OK counter on this part (MEASURED: it stays 0 on the working kernel \
         driver mid-transmit). Declaring it as an instrument would make every bring-up report a \
         refutation of a transmitter that works."
    );
    let reason = <LibUsbRtl88xxBackend as BringUp>::tx_unprovable_reason()
        .expect("the part must say WHY it cannot answer (A), not leave the runner's generic line");
    assert!(
        reason.contains("0x2DE0") && reason.contains("0x2d08"),
        "the quoted reason must name BOTH ruled-out registers: 0x2DE0 (not a TX-OK counter here) \
         and 0x2d08 (an RX false-alarm counter that `examples/tx_liveness.rs` prints as \"TX \
         activity\"). Got: {reason}"
    );
    assert!(
        reason.contains("witness"),
        "an unprovable (A) must point at the only thing that can answer (B)"
    );
}

/// ★ **The §1.5 assert this part exists to carry** — and that it is a READBACK, not a clear.
#[test]
fn the_txgapk_txpause_gate_is_read_back_not_cleared() {
    let asserts = <LibUsbRtl88xxBackend as BringUp>::asserts();
    let a = asserts.iter().find(|a| a.reg == 0x0522).expect(
        "REG_TXPAUSE (0x0522) is not read back. `txgapk_tx_pause` writes 0xff and the \
                 resume sits at the tail of `txgapk`, whose error the ladder swallowed with a \
                 warning — so a failed gain-K leaves the MAC transmit queues HELD and every later \
                 inject queues and dies silently.",
    );
    assert_eq!(a.want, 0x00);
    assert_eq!(a.mask, 0xff);
    assert!(
        a.why.contains("txgapk"),
        "the assert must name the routine whose error path it guards"
    );

    // ...and no rung writes the gate. The hand-rolled clear on the sibling 8812au was A/B'd INERT
    // (2789/4508/4588/4406 frames — run-to-run noise, no direction) and reverted; adding one here
    // would paper over exactly the failure this reads back.
    let src = std::fs::read_to_string(concat!(
        env!("CARGO_MANIFEST_DIR"),
        "/src/libusb_rtl88xx.rs"
    ))
    .expect("the 88xx driver source");
    // Only the rung bodies — the prose above quotes the rejected `write8(0x0522, 0x00)` by name,
    // which is the point of quoting it.
    let rungs = rung_section(&src);
    for forbidden in ["write8(0x0522", "write8(0x522"] {
        assert!(
            !rungs.contains(forbidden),
            "a rung in PLAN_A81A contains `{forbidden}`. §1.5 is explicit: read the gate back, do \
             NOT add a clear — the clear was A/B'd inert on the sibling part (2789/4508/4588/4406 \
             frames, run-to-run noise, no direction) and reverted."
        );
    }
}

/// **LAW 1 — nothing inside a bring-up may read the process environment.**
///
/// `NDN_RADIO_MINIMAL` / `NDN_RADIO_SKIP_CAL` / `NDN_RADIO_NO_EFEM` were read INSIDE the ladder
/// until M4. Configuration a caller cannot see is `load_tx_power_info` one level up: two runs that
/// look identical in their own output. They are read once now, at the wrapper boundary, and become
/// a self-labelling `Deviation`.
#[test]
fn no_rung_reads_the_environment() {
    let src = std::fs::read_to_string(concat!(
        env!("CARGO_MANIFEST_DIR"),
        "/src/libusb_rtl88xx.rs"
    ))
    .expect("the 88xx driver source");
    assert!(
        !rung_section(&src).contains("std::env"),
        "a plan step on this part reads the process environment. LAW 1: every `NDN_*` becomes a \
         request field or a `Deviation`, read once at the boundary where the caller can see it. \
         A knob whose meaning depends on the environment is hidden state by another name."
    );
}

/// ★ **The env knobs still name rungs that exist.**
///
/// Rule 3 of §1.6: edits name steps by id, so renaming or deleting a rung fails every experiment
/// touching it *loudly*. `env_deviation` is private, so the ids it uses are re-stated here — which
/// is the point: rename `txgapk` and this test fails at compile-review time instead of
/// `NDN_RADIO_SKIP_CAL` failing at `Plan::resolve` on somebody's bench at 2 a.m.
#[test]
fn every_env_deviation_names_a_real_rung() {
    // NDN_RADIO_MINIMAL
    let minimal = ["phy_init"];
    // NDN_RADIO_SKIP_CAL (and MINIMAL, which implies it)
    let skip_cal = [
        "fw_iqk",
        "lck",
        "fw_dpk",
        "dpk_force_bypass",
        "kfree",
        "txgapk",
    ];
    // NDN_RADIO_NO_EFEM
    let no_efem = ["efem_pinmux"];
    for id in minimal.iter().chain(&skip_cal).chain(&no_efem) {
        assert!(
            ids().contains(id),
            "`env_deviation` skips `{id}`, which PLAN_A81A does not contain. That is \
             `RequestError::UnknownStep` at resolve time — a knob that fails loudly, but only once \
             somebody with the hardware sets it."
        );
    }
}

/// ★ **The 0x3a00 per-rate TXAGC conflict is recorded as an OPEN QUESTION, with both citations.**
///
/// `named_radio_face.rs` writes the per-rate TXAGC table by hand because *"our
/// `bb_tx_datapath_init` never does"*; `set_tx_power`'s doc says *"per-rate diffs in the `0x3a00`
/// table are left at 0"*. Both are in the tree; a witness settles which describes what reaches the
/// air. M4 was not allowed to migrate the workaround or delete it, so the least it can do is make
/// the question undeleteable-by-accident.
#[test]
fn the_per_rate_txagc_conflict_is_written_into_the_plan() {
    let why = PLAN_A81A.steps[at("calibrate_tx_power")].why;
    for cite in [
        "named_radio_face.rs",
        "0x3a00",
        "set_tx_power",
        "write_txagc",
        "witness",
    ] {
        assert!(
            why.contains(cite),
            "the `calibrate_tx_power` rung's `why` no longer cites `{cite}`. The per-rate TXAGC \
             conflict is UNRESOLVED and needs a witness; deleting the citation deletes the only \
             record that anybody noticed."
        );
    }
}

/// The written exclusions this part carries, asserted so that deleting one is a decision rather
/// than a diff nobody reads.
#[test]
fn the_hmebox_knob_and_the_attach_stage_are_excluded_in_writing() {
    let text: String = PLAN_A81A
        .excluded
        .iter()
        .map(|(_, why)| *why)
        .collect::<Vec<_>>()
        .join("\n");
    assert!(
        text.contains("NDN_RADIO_HMEBOX_H2C"),
        "the one knob M4 could not convert must be written down. It was ADDITIVE, and §1.4 is \
         explicit that a Deviation may only subtract — putting the rung in the canonical plan \
         instead would start replaying two undecoded firmware commands on every production node."
    );
    assert!(
        text.contains("DeviceAddress::Usb"),
        "the Attach exclusion must say that the address IS reported — unlike the 8733b, this part \
         has a `DeviceSelect` and a bus address, and a report that named neither would be the \
         `DeviceAddress::Unknown` placeholder the contract calls out."
    );
}
