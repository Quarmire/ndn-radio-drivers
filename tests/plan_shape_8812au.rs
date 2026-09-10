//! **§6.3 `tests/plan_shape.rs`, scoped to the RTL8812AU** (contract §5-M7) — the last part
//! migrated, and the only one whose *radiating* bring-up is measured.
//!
//! Hardware-free, and deliberately weaker than it looks. What a source-level test can check here is
//! that the plan **is the transcription**: the same rungs, in the same order, with nothing added.
//! What it cannot check is the thing that matters — that the transcription still puts frames on
//! air. That is the operator's witness run (see the "M7 acceptance" section of
//! `docs/bringup-contract.md`), and no green test in this file is a substitute for it.

use ndn_radio_drivers::{PLAN_8812AU_MONITOR, Rtl8812auBackend};
use ndn_radio_hal::bringup::{BringUp, Role, Severity, Stage, StepClass};

/// The step ids in the order the pre-M7 `bring_up_monitor` ran them, transcribed independently of
/// the plan so that a reordering shows up as a diff between two lists rather than as nothing at
/// all. ☠ Appendix A.3: the migrating commit adds, moves and removes **nothing**.
const LADDER: &[&str] = &[
    "power_on",
    "download_firmware",
    "mac_config",
    "mac_enable_dma",
    "mac_init_queues",
    "bb_config",
    "rf_config",
    "set_channel",
    "load_tx_power_info",
    "disable_edcca",
    "iq_calibrate",
    "lc_calibrate",
    "start_rx_dma",
    "set_tx_power",
];

fn ids() -> Vec<&'static str> {
    PLAN_8812AU_MONITOR
        .steps
        .iter()
        .map(|s| s.id.as_str())
        .collect()
}

fn at(id: &str) -> usize {
    ids()
        .iter()
        .position(|i| *i == id)
        .unwrap_or_else(|| panic!("PLAN_8812AU_MONITOR has no rung `{id}`"))
}

fn src() -> String {
    std::fs::read_to_string(concat!(env!("CARGO_MANIFEST_DIR"), "/src/rtl8812au.rs"))
        .expect("the 8812au driver source")
}

/// The rung *code* — every `s_*` step function, and nothing else. The source-level tests below must
/// read CODE, not the `why` prose on the `const R_*` items, which quotes the code it rejects.
fn rung_bodies(s: &str) -> &str {
    s.split_once("// ── the rungs ─")
        .expect("the rung section")
        .1
        .split_once("// ── the rungs, as reviewable constants")
        .expect("the end of the rung bodies")
        .0
}

// ── the transcription itself ─────────────────────────────────────────────────

/// ★ **The plan IS the ladder.** Not "equivalent to": the same names in the same order.
#[test]
fn the_plan_is_the_transcribed_ladder() {
    assert_eq!(
        ids(),
        LADDER,
        "PLAN_8812AU_MONITOR is no longer the sequence `bring_up_monitor` ran.\n\n\
         This is the one migration the contract's own Appendix A.3 says must add, move and remove \
         NOTHING: the 8812au is the only part in this fleet whose radiating bring-up is measured, \
         and reasoning about these radios has a ~0 % hit rate while measuring has ~100 %. If this \
         list genuinely needs to change, the change is an ON-AIR experiment at the witness, not an \
         edit to a table."
    );
}

/// The plan holds together: no empty `why`, no blank degradation, no duplicate id, and every
/// `must_follow`/`must_precede` names a rung this plan contains **and** is satisfied.
#[test]
fn the_plan_holds_together() {
    PLAN_8812AU_MONITOR.check().unwrap_or_else(|e| {
        panic!(
            "{} does not hold together: {e}\n\
             A plan is checked before the first register write; a malformed one must never reach \
             the hardware.",
            PLAN_8812AU_MONITOR.id
        )
    });
}

/// ☠ **The three steps that are NOT here, because they were MEASURED not to help.**
///
/// `init_llt` (added → no change on air), a `REG_TXPAUSE` clear (added → A/B'd inert: 2789 / 4508 /
/// 4588 / 4406 frames, run-to-run noise; `REG_TXPAUSE` reads 0x00 after three bring-ups anyway),
/// and the bulk-OUT endpoint theory (all three endpoints → 0 frames). Adding steps "just in case"
/// is the disease this contract names.
#[test]
fn the_refuted_steps_stay_out() {
    for banned in ["init_llt", "llt", "txpause_clear", "send_frame_ep"] {
        assert!(
            !ids().contains(&banned),
            "`{banned}` is a rung of PLAN_8812AU_MONITOR. It was added on plausible reasoning, \
             A/B'd against a witness, and REVERTED. The 2026-09-03 \"does not transmit\" symptom \
             was the POWER REGIME (fixed in M1 by deleting `set_tx_power`'s fallthrough), not the \
             LLT, not TXPAUSE, and not the endpoint."
        );
    }
    let s = src();
    let body = rung_bodies(&s);
    assert!(
        !body.contains("init_llt("),
        "a rung body calls `init_llt`. It has zero callers in `src/` for a measured reason."
    );
    for poke in ["write8(0x522", "write8(0x0522"] {
        assert!(
            !body.contains(poke),
            "a rung body writes REG_TXPAUSE (`{poke}`). The TXPAUSE clear is an **Assert**, not a \
             step — read the gate back, do not write it."
        );
    }
}

// ── the orderings, as constraints rather than prose ──────────────────────────

/// `mac_enable_dma` ZEROES `REG_CR` before setting `DMA_ENABLE`; `mac_init_queues` sets
/// `MACTXEN|MACRXEN` last. The reverse order silently disables RX.
#[test]
fn dma_enable_precedes_the_queues() {
    assert!(at("mac_enable_dma") < at("mac_init_queues"));
}

/// The cal chain and the tune are per-channel, and the power base is per-channel: `set_tx_power`
/// refuses outright when `cur_channel == 0`.
#[test]
fn the_channel_is_tuned_before_anything_that_depends_on_it() {
    for after in ["load_tx_power_info", "iq_calibrate", "set_tx_power"] {
        assert!(
            at("set_channel") < at(after),
            "`{after}` runs before `set_channel`, and it is channel-dependent."
        );
    }
}

/// ★ **The power step is last among everything that touches the gain chain** (§6.3). IQK is a
/// TX/RX loopback calibration that drives the gain registers and runs up to six times.
#[test]
fn tx_power_is_the_last_rung() {
    assert_eq!(
        *ids().last().expect("a non-empty plan"),
        "set_tx_power",
        "`set_tx_power` is no longer the last rung. Anything that perturbs the gain chain must run \
         BEFORE the power is set, never after."
    );
    for before in ["load_tx_power_info", "iq_calibrate", "lc_calibrate"] {
        assert!(at(before) < at("set_tx_power"));
    }
    let power_stage: Vec<_> = PLAN_8812AU_MONITOR
        .steps
        .iter()
        .filter(|s| s.stage == Stage::Power)
        .map(|s| s.id.as_str())
        .collect();
    assert_eq!(power_stage, vec!["set_tx_power"]);
}

// ── classes ──────────────────────────────────────────────────────────────────

/// **LAW 5** — a rung whose body polls a hardware completion bit is always `Required`. `power_on`
/// polls the power-sequence entries; `download_firmware` polls `WINTINI_RDY`. `start_rx_dma` is
/// Required for a different reason: without it the radio is silently deaf.
#[test]
fn the_polling_rungs_are_required() {
    for id in [
        "power_on",
        "download_firmware",
        "start_rx_dma",
        "set_tx_power",
    ] {
        let s = &PLAN_8812AU_MONITOR.steps[at(id)];
        assert_eq!(
            s.class,
            StepClass::Required,
            "`{id}` must be Required: a timed-out completion poll that continued would leave a \
             radio whose every later readback is fiction."
        );
    }
}

/// The four rungs the ladder continued past — and each one names what is lost, which is the price
/// of not being `Required`.
#[test]
fn the_best_effort_rungs_name_what_is_lost() {
    for id in [
        "load_tx_power_info",
        "disable_edcca",
        "iq_calibrate",
        "lc_calibrate",
    ] {
        match PLAN_8812AU_MONITOR.steps[at(id)].class {
            StepClass::BestEffort(d) => {
                assert!(!d.lost.trim().is_empty(), "`{id}`: blank `lost`");
                assert!(
                    !d.still_valid_for.trim().is_empty(),
                    "`{id}`: blank `still_valid_for` — if nothing survives the failure, the rung is \
                     Required."
                );
            }
            other => panic!(
                "`{id}` is {other:?}; the pre-M7 ladder continued past its failure with a warning, \
                 and changing that is a behaviour change, not a transcription."
            ),
        }
    }
}

// ── the asserts (§1.5) ───────────────────────────────────────────────────────

/// The five §1.5 readbacks for this part, **all at `Warn`**.
#[test]
fn the_five_asserts_are_present_and_warn_only() {
    let got: Vec<&str> = <Rtl8812auBackend as BringUp>::asserts()
        .iter()
        .map(|a| a.id.as_str())
        .collect();
    assert_eq!(
        got,
        vec![
            "txpause_released",
            "mac_tx_rx_enabled",
            "rx_antenna_restored",
            "cca_restored",
            "cck_rx_restored",
        ],
        "the §1.5 assert set for the 8812au is the FULL `iqk_configure_mac` quiesce list, not just \
         TXPAUSE: that function drops five things and restores them only in `iq_calibrate`'s tail \
         block, which the `iqk_tx()?` error path skips."
    );
    for a in <Rtl8812auBackend as BringUp>::asserts() {
        assert_eq!(
            a.severity,
            Severity::Warn,
            "`{}` is Fatal. §5/M-hazards: `Warn` on introduction for EVERY part; promotion needs a \
             measurement, and a readback nobody has watched fail is not allowed to refuse the radio \
             the forwarder runs on.",
            a.id
        );
        assert!(!a.why.trim().is_empty(), "`{}` has an empty `why`", a.id);
        assert_ne!(a.mask, 0, "`{}` masks off every bit it reads", a.id);
    }
}

/// The registers the asserts name are the ones `iqk_configure_mac` writes (plus `REG_CR`).
#[test]
fn the_asserts_read_back_registers_the_ladder_writes() {
    let regs: Vec<u32> = <Rtl8812auBackend as BringUp>::asserts()
        .iter()
        .map(|a| a.reg)
        .collect();
    assert_eq!(regs, vec![0x0522, 0x0100, 0x0808, 0x0838, 0x0a07]);
}

// ── §4 — the transmit question ───────────────────────────────────────────────

/// This part cannot answer even question (A) on-chip, and says so with the measurement rather than
/// with silence. It is why §5-M7's acceptance is a witness run.
#[test]
fn the_transmit_question_is_refused_with_a_reason() {
    assert!(
        <Rtl8812auBackend as BringUp>::tx_instruments().is_empty(),
        "an instrument appeared for this part. If a Jaguar1 MAC->BB counter has been ported, the \
         `tx_unprovable_reason` below is now stale and must be deleted, not left to contradict it."
    );
    let reason = <Rtl8812auBackend as BringUp>::tx_unprovable_reason()
        .expect("§4: an empty instrument set must carry the part's own measured reason");
    assert!(reason.contains("witness"));
}

// ── roles ────────────────────────────────────────────────────────────────────

/// One plan, and the other roles are **named refusals** rather than silent downgrades.
#[test]
fn only_the_measured_role_has_a_plan() {
    assert!(<Rtl8812auBackend as BringUp>::plan(Role::TransmitAndReceive).is_some());
    for r in [Role::ReceiveOnly, Role::TransmitOnly] {
        assert!(
            <Rtl8812auBackend as BringUp>::plan(r).is_none(),
            "{r:?} got a plan. One ladder is all this part has ever run; inventing a second is the \
             unmeasured bring-up this contract exists to remove."
        );
    }
}

/// A blank cell is a written decision. The three stages this ladder does nothing in each carry a
/// ruling — including the one that matters: there is no TX-enable rung, and that is measured.
#[test]
fn the_empty_stages_are_written_decisions() {
    for stage in [Stage::Attach, Stage::TxEnable, Stage::Verify] {
        let why = PLAN_8812AU_MONITOR
            .excluded
            .iter()
            .find(|(s, _)| *s == stage)
            .map(|(_, w)| *w)
            .unwrap_or_else(|| panic!("{stage:?} is neither used nor excluded in writing"));
        assert!(!why.trim().is_empty());
    }
    let txen = PLAN_8812AU_MONITOR
        .excluded
        .iter()
        .find(|(s, _)| *s == Stage::TxEnable)
        .expect("a TxEnable exclusion")
        .1;
    assert!(
        txen.contains("3074"),
        "the TxEnable exclusion must carry the reference number it rests on — 3074 frames at the \
         AR9271 witness from a calibrated bring-up plus a raw write. Without it, \"this part needs \
         no TX-enable rung\" is an opinion."
    );
}

// ── LAW 1 / LAW 3, source-level ──────────────────────────────────────────────

/// **LAW 1 — nothing inside a bring-up may read the process environment**, checked over the rung
/// bodies.
///
/// ⚠ It is a *shallow* check and this test says so, because there is a real, known violation one
/// call deep: [`Rtl8812auBackend::start_rx_dma`] reads `NDN_RXDMA_AGG`, `NDN_RX_AGG_OFF` and
/// `NDN_RX_AGG_DBG`. Hoisting them needs §1.1's `BringUpRequest`/`PartOpts`, which is not built.
/// M7 does not invent a third home for them; it pins the set so it cannot grow silently, and the
/// rung's `why` says the exception out loud.
#[test]
fn no_rung_body_reads_the_environment() {
    let s = src();
    let body = rung_bodies(&s);
    for pat in ["std::env", "env::var", "env!(", "option_env!("] {
        assert!(
            !body.contains(pat),
            "a plan rung body contains `{pat}`. LAW 1: every `NDN_*` becomes a request field or a \
             `Deviation`, read once at the boundary where the caller can see it. Configuration a \
             caller cannot see is `load_tx_power_info` one level up."
        );
    }
}

/// The known one-call-deep exception, pinned by name so it can only shrink.
#[test]
fn the_start_rx_dma_env_exception_does_not_grow() {
    let s = src();
    // ★ M8 moved this method inside a `rung! { … }` block (§5-M8: the ladder rungs are `pub` only
    // under `feature = "bench"`), so it is now `fn start_rx_dma(` at one extra level of
    // indentation. The anchors follow the code; what is pinned — the set of reads — has not moved.
    let f = s.split_once("fn start_rx_dma(").expect("start_rx_dma").1;
    let f = &f[..f.find("\n        }").expect("the end of start_rx_dma")];
    let mut found: Vec<&str> = ["NDN_RXDMA_AGG", "NDN_RX_AGG_OFF", "NDN_RX_AGG_DBG"]
        .into_iter()
        .filter(|n| f.contains(n))
        .collect();
    found.sort_unstable();
    assert_eq!(
        found,
        vec!["NDN_RXDMA_AGG", "NDN_RX_AGG_DBG", "NDN_RX_AGG_OFF"],
        "the set of environment variables `start_rx_dma` reads has CHANGED. It is a written LAW 1 \
         exception, and a written exception may shrink (by moving a knob onto the request) but \
         never grow."
    );
    assert_eq!(
        f.matches("std::env").count(),
        3,
        "`start_rx_dma` gained or lost an environment read. See above: this exception may only \
         shrink, and shrinking it means deleting this assertion along with the read."
    );
}

/// **LAW 3 / M1 — the plan CARRIES the rate-group policy; it does not re-read it.**
///
/// `NDN_AU_TXAGC12` was read from inside `set_tx_power` and silently turned 10 register writes into
/// 24 — MEASURED 246 f/s against 1 f/s at a witness. It left `ndn_env`'s `KNOWN_UNREGISTERED` list
/// by having its reader DELETED, and M7 must not bring the reader back through the plan.
#[test]
fn the_power_rung_takes_the_request_from_the_plan_not_the_environment() {
    let s = src();
    let body = rung_bodies(&s);
    assert!(
        body.contains("c.state_ref().power.requested"),
        "the `set_tx_power` rung no longer reads the request out of `Ctx`. That is where the \
         `RateGroupPolicy` travels: on the `PowerRequest` the caller made, visible in the returned \
         `AppliedPower` and in the report's write count (10 vs 24)."
    );
    assert!(
        !s.contains("var(\"NDN_AU_TXAGC12") && !s.contains("var_os(\"NDN_AU_TXAGC12"),
        "a reader for NDN_AU_TXAGC12 came back to this driver. It left `ndn_env`'s \
         KNOWN_UNREGISTERED list by having its reader DELETED, not registered."
    );
    assert!(
        !body.contains("NDN_AU_TXAGC12"),
        "a rung reaches for NDN_AU_TXAGC12. Writing the seven HT-2SS/VHT groups STOPS THIS RADIO \
         TRANSMITTING; the choice belongs at the call site, in code, where a reviewer sees it."
    );
}

/// Every `why` carries provenance, not a restatement of the function name.
#[test]
fn every_why_says_something() {
    for s in PLAN_8812AU_MONITOR.steps {
        assert!(
            s.why.trim().len() > 40,
            "rung `{}` has a `why` too short to carry a measurement or a vendor reference",
            s.id
        );
    }
}
