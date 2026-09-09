//! **§6.3 `tests/plan_shape.rs`, scoped to the one part that has a plan** (contract §5-M3).
//!
//! Hardware-free. It reads the RTL8733BU's `Plan`s the way a reviewer would and asserts the
//! properties the contract says a plan must have — the ones that are *not* already compile errors,
//! plus a belt over the ones that are.
//!
//! Why a belt over `check_or_panic`: a `const` panic carries a literal message only, so the
//! compile error says *which rule* broke and not which rung. These tests name the rung, and they
//! keep working if somebody ever deletes the `const _: ()` lines.
//!
//! ⚠ Scope. §6.3's full sweep — *every* backend with a `FrameIo` impl declares a plan or a written
//! `bringup_coverage` exclusion — is M9, and would fail today by design: twelve parts still run
//! their M2 hand-filled ladders. This file asserts what M3 actually landed.

use ndn_radio_drivers::{PLAN_8733B_MONITOR, PLAN_8733B_TX, Rtl8733buBackend};
use ndn_radio_hal::bringup::{BringUp, Role, StepClass};

/// The plans hold together: no empty `why`, no blank degradation, no duplicate id, and every
/// `must_follow`/`must_precede` names a rung this plan contains **and** is satisfied by the
/// sequence.
#[test]
fn both_plans_hold_together() {
    for plan in [&PLAN_8733B_MONITOR, &PLAN_8733B_TX] {
        plan.check().unwrap_or_else(|e| {
            panic!(
                "{} does not hold together: {e}\n\
                 A plan is checked before the first register write; a malformed one must never \
                 reach the hardware.",
                plan.id
            )
        });
    }
}

/// **Every `why` is non-empty** — `coverage::Seam::Excluded`'s rule applied to rungs, to the §1.5
/// asserts, and to the written exclusions. A blank cell is a written decision, not an absence.
#[test]
fn every_why_and_every_exclusion_is_written() {
    for plan in [&PLAN_8733B_MONITOR, &PLAN_8733B_TX] {
        for s in plan.steps {
            assert!(
                !s.why.trim().is_empty(),
                "{}::{} has no stated reason. The measurement or vendor reference that puts a rung \
                 in a ladder is the only thing that stops the next person deleting it — or, worse, \
                 copying it into a sixteenth private bring-up.",
                plan.id,
                s.id
            );
        }
        for (stage, reason) in plan.excluded {
            assert!(
                !reason.trim().is_empty(),
                "{} excludes {stage:?} with no ruling. An empty exclusion is an absence pretending \
                 to be a decision.",
                plan.id
            );
        }
    }
    for a in <Rtl8733buBackend as BringUp>::asserts() {
        assert!(
            !a.why.trim().is_empty(),
            "assert `{}` at {:#06x} has no stated reason — a bus round trip nobody justified",
            a.id,
            a.reg
        );
    }
}

/// ★ **The 19.6 dB ordering, as a constraint rather than prose.**
///
/// `tssi_setup` before `enable_tx` gives 19.6 dB of monotonic range through
/// `RadioKnobs::set_tx_power`; the other order gives 1.6 dB, because the two rungs both write the
/// datapath (0x1c38 / 0x1c84 / 0x1ca4 / 0x1e1c) while only `tssi_setup` touches 0x43xx where the
/// loop lives. This lived in a doc comment. Now it is declared, checked at plan construction, and
/// re-checked here by position.
#[test]
fn tssi_setup_precedes_enable_tx() {
    let ids: Vec<&str> = PLAN_8733B_TX.steps.iter().map(|s| s.id.as_str()).collect();
    let tssi = ids.iter().position(|i| *i == "tssi_setup").expect(
        "the TX plan must configure TSSI: it is the ONLY path that controls radiated \
                 power on this part (the TXAGC page is inert by efuse design)",
    );
    let enable = ids
        .iter()
        .position(|i| *i == "enable_tx")
        .expect("the TX plan must run enable_tx");
    assert!(
        tssi < enable,
        "PLAN_8733B_TX runs enable_tx before tssi_setup. MEASURED: TSSI first gives 19.6 dB of \
         usable range, the other order 1.6 dB — an enabled-but-unconfigured loop gives the DE no \
         authority, which reads exactly like a dead knob."
    );

    // ...and the constraint is DECLARED, not merely satisfied by luck of the ordering.
    let tssi_step = &PLAN_8733B_TX.steps[tssi];
    assert!(
        tssi_step
            .must_precede
            .iter()
            .any(|i| i.as_str() == "enable_tx"),
        "tssi_setup satisfies the ordering but does not declare it. An undeclared ordering is \
         prose again: reorder the two and the crate would still build."
    );
}

/// The two plans share one step list; the TX plan is the monitor plan plus its tail. Two ladders
/// that were "the same except…" is the defect this contract removes, so the shared prefix must be
/// literally shared and not copy-pasted.
#[test]
fn the_tx_plan_is_the_monitor_plan_plus_its_tail() {
    let mon: Vec<&str> = PLAN_8733B_MONITOR
        .steps
        .iter()
        .map(|s| s.id.as_str())
        .collect();
    let tx: Vec<&str> = PLAN_8733B_TX.steps.iter().map(|s| s.id.as_str()).collect();
    assert_eq!(
        &tx[..mon.len()],
        &mon[..],
        "the TX plan's prefix has drifted from the monitor plan. `bring_up_tx` used to CALL \
         `bring_up_monitor`, so they could not drift; if the two step lists are edited apart the \
         part is back to two bring-ups whose difference nobody can see."
    );
    assert_eq!(
        &tx[mon.len()..],
        &[
            "tssi_setup",
            "enable_tx",
            "txpause_released",
            "tssi_loop_live",
            "power_tracking"
        ],
        "the TX tail changed — check it against `bring_up_tx`'s transcribed sequence"
    );
}

/// **LAW 5**: a rung that polls a hardware completion bit is always `Required`. A timed-out boot
/// or LLT that continues leaves a radio whose every later readback is fiction.
#[test]
fn every_completion_poll_is_required() {
    // `download_firmware` polls DDMA-idle + the WINTINI_RDY handshake; `init_trx` polls
    // REG_AUTO_LLT BIT16 until it self-clears.
    for id in ["download_firmware", "init_trx"] {
        for plan in [&PLAN_8733B_MONITOR, &PLAN_8733B_TX] {
            let s = plan
                .steps
                .iter()
                .find(|s| s.id.as_str() == id)
                .unwrap_or_else(|| panic!("{} is missing `{id}`", plan.id));
            assert!(
                matches!(s.class, StepClass::Required),
                "{}::{id} polls a hardware completion bit and is not Required (LAW 5)",
                plan.id
            );
        }
    }
}

/// A part says which roles it does, by name. `None` is a named refusal, never a silent downgrade
/// into a role the caller did not ask for.
#[test]
fn the_part_names_the_roles_it_does_and_the_one_it_does_not() {
    assert_eq!(
        <Rtl8733buBackend as BringUp>::plan(Role::ReceiveOnly).map(|p| p.id.name),
        Some("monitor")
    );
    assert_eq!(
        <Rtl8733buBackend as BringUp>::plan(Role::TransmitAndReceive).map(|p| p.id.name),
        Some("tx")
    );
    assert!(
        <Rtl8733buBackend as BringUp>::plan(Role::TransmitOnly).is_none(),
        "there is no transmit-only ladder on this part — `set_monitor` sits in the middle of the \
         sequence the TX path is built on and removing it has never been measured. Say so by \
         returning None."
    );
}

/// ★ **The declared instrument is actually called.**
///
/// A capability with one definition and zero call sites is this codebase's characteristic failure
/// (`with_wide_bloom`; `init_llt`; the Tier-0 name gate). `read_tx_counters` was exactly that at
/// bring-up time: the best-calibrated counter in the fleet, named in a doc comment, asked by
/// nothing. Source-level, in the style of §6's other structural tests, because calling the probe
/// needs a radio.
#[test]
fn the_transmit_instrument_is_wired_to_a_probe() {
    let instruments = <Rtl8733buBackend as BringUp>::tx_instruments();
    assert!(
        !instruments.is_empty(),
        "the one part in the fleet with a working MAC->BB transmit counter declares no \
         TxInstrument"
    );
    assert!(
        instruments[0].name.contains("0x2de0"),
        "the instrument must name the register pair it reads, so a report says what was measured"
    );

    let src = std::fs::read_to_string(concat!(
        env!("CARGO_MANIFEST_DIR"),
        "/src/libusb_rtl8733b.rs"
    ))
    .expect("the 8733b driver source");
    assert!(
        src.contains("fn probe_tx("),
        "Rtl8733buBackend declares a TxInstrument but overrides no `BringUp::probe_tx`, so the \
         runner can only ever answer `Unprovable` and the instrument is a capability nobody calls \
         — the exact defect the contract's §4 exists to remove."
    );
    assert!(
        src.contains("fn probe_tx_counters("),
        "the probe must be a named function on the backend, callable outside a bring-up: an \
         instrument you can only reach through `run_plan` cannot be used to answer the same \
         question later, on demand"
    );
}

/// The written exclusions this part carries, asserted so that deleting one is a decision rather
/// than a diff nobody reads.
///
/// ⚠ Deliberately NOT asserted: §6.3's *"the power step is last among steps that touch the gain
/// chain"*. On this part it is FALSE and correctly so — `set_txagc_table` runs where
/// `bring_up_monitor` always ran it, and `tssi_setup` + `enable_tx` follow because that order is
/// what MEASURED 19.6 dB. Transcription beat the rule; the rule is the one that needs revisiting,
/// against a witness, not the ladder.
#[test]
fn the_attach_stage_is_excluded_in_writing() {
    for plan in [&PLAN_8733B_MONITOR, &PLAN_8733B_TX] {
        let attach = plan
            .excluded
            .iter()
            .find(|(s, _)| format!("{s:?}") == "Attach");
        let (_, why) = attach.unwrap_or_else(|| {
            panic!(
                "{} does nothing in Stage::Attach and does not say why. `Rtl8733buBackend::open` \
                 claims the FIRST match on the bus; that is a real limitation and belongs in the \
                 plan, not in somebody's memory.",
                plan.id
            )
        });
        assert!(why.contains("open_select") || why.contains("FIRST"));
    }
}
