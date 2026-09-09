//! **§6.3 of the bring-up contract — the shape of every plan in the crate, hardware-free.**
//!
//! The per-part files (`plan_shape_8733b`, `plan_shape_a81a`, `plan_shape_8812au`,
//! `plan_shape_mt76`, `plan_shape_m6`, `plan_shape_m8`) each check that ONE part's plan is the
//! transcription it claims to be. This file checks the properties that must hold for **all
//! eighteen plans at once** (sixteen under default features; the `lora` and `serial-radio` two
//! ride their gates) and — more importantly — the ones that hold for a plan **added tomorrow by
//! someone who never read those files**.
//!
//! ## What is checked here, and what was already checked elsewhere
//!
//! `Plan::check()` already enforces non-empty `why`, no duplicate ids, and satisfied ordering, and
//! every plan in the crate wires it to a `const _: () = X.check_or_panic();`. So running it here
//! is, once again, a **belt over a compiler brace**. The belt earns its place on four counts, and
//! each is a hole the brace does not cover:
//!
//! 1. **A new plan with no `check_or_panic` line.** Nothing makes a driver author add one. Without
//!    it a blank `why` is discovered when somebody brings that radio up, not when they build.
//!    `every_plan_static_is_const_checked_and_listed_here` scans for it — and for the other half
//!    of the same hole: a plan absent from this file's own `all_plans()` list, which nothing else
//!    would notice.
//! 2. **`Assert::why` is documented as checked by `Plan::check` and is not checked by anything.**
//!    `Plan::check` walks `self.steps`; the asserts hang off the `BringUp` trait and it never sees
//!    them. Worse — ☠ **an `Assert` with `mask: 0` always passes**, because the runner evaluates
//!    `(read & mask) == (want & mask)`. A readback that cannot fail is a readback that is not
//!    being taken, reported as `ok` in every report forever. That is the `worst_receiver_rate`
//!    defect (a rule matching an arm every backend has) with a register attached.
//! 3. **`const` panics carry literal messages only.** `check_or_panic` says *which rule* broke,
//!    never which rung; `check()` says both. When it fires here the author gets the named message.
//! 4. **The cross-cutting laws** — 5 and 6 — are not plan-local at all. They are relations between
//!    a plan and the driver source behind it, and only a source scan can see them.
//!
//! ## LAW 6, and why testing it literally would be a vacuous test
//!
//! §6.3 asks: *"every `StepOutcome::Established(Fact::X)` in a plan has a matching field/variant
//! reachable in `RadioState::facts`"*. Taken literally that is **not falsifiable**:
//! `RadioState::facts` is a `Vec<Fact>`, so every `Fact` variant is reachable by construction, and
//! `run_plan` pushes every `Established(f)` into it with no way for a driver to opt out. A test
//! asserting it would pass on every possible tree — the `worst_receiver_rate` failure mode
//! exactly, which matched a `None => frame::build(..)` arm every backend has and so tested
//! nothing.
//!
//! The direction that IS falsifiable is the inverse, and it is where the real regression lives.
//! `StepOutcome` admits **one** variant per rung, so a step that both branches and decides
//! something cannot say both — and three rungs in this crate (`mt7921::s_firmware_ready`,
//! `mt76x0::s_firmware_ready`, `mt7612::s_firmware_and_init`) therefore return
//! `Branch("warm"|"cold")` and push their `Fact::Warm` into `c.state().facts` **by hand**. Hand
//! written is where things regress. So LAW 6 here is: *a rung that constructs a `Fact` must make
//! it reach the report* — by `Established`, or by that hand push. Delete the push and the run
//! still succeeds, still branches, and silently stops saying whether the chip was warm; that is
//! the founding defect (`load_tx_power_info` deciding what `set_tx_power` meant) in a new costume.
//!
//! ⚠ **What LAW 6 here does NOT check, deliberately.** A rung that writes `c.state().power` is not
//! required to also produce a `Fact::PowerReference`. Two do not (`ath9k::s_power_cal`,
//! `rtl8821c::s_tune_channel`) and they are not defective: `AppliedPower` carries its own
//! `PowerReference` and is rendered in every report, so the `Fact` would be a duplicate. A rule
//! whose violation has no consequence on air is a rule, not a guard.
//!
//! ## ☠ Two rules drafted here were FALSE on measured sequences, and were dropped rather than
//! exempted
//!
//! Both were keyed on [`Stage`], which the HAL says in as many words is *"a rendering /
//! `--stop-after` label ONLY. Not an enforcement device."* Writing them was the fastest way to
//! find out it meant it:
//!
//! * *"no `Stage::Calibrate` rung after the `Stage::Power` rung"* — `PLAN_8733B_TX` puts
//!   `set_txagc_table` before `tssi_setup` deliberately: on that part the per-rate TXAGC page is
//!   MEASURED INERT (efuse `power_track_type = 4`) and the TSSI DE is the actuator.
//! * *"a `ReceiveOnly` plan holds no `Stage::TxEnable` rung"* — `PLAN_8733B_MONITOR` holds
//!   `enable_tx_path`, because the old `bring_up_monitor` ran it and several instruments inject to
//!   the MAC from a monitor handle. It does not complete the on-air path; `tssi_setup` +
//!   `enable_tx` do, and those are what the two roles differ by.
//!
//! Adding either as an exemption would have made the guard say something false about a radio in
//! order to keep a rule that was never true. Neither survives; what replaced them is in
//! `the_power_rung_declares_its_ordering` and in the table/static/role link inside
//! `every_backend_declares_a_plan_or_a_written_exclusion`.
//!
//! ## §6.3 bullets not implemented here, and where they live instead
//!
//! * *"every `Gate`/`Deviation` env name is registered in `ndn_env::KNOWN`"* — `ndn_env`'s own
//!   ratchet test is the registry gate, and `plan_shape_m8::law_1_*` is what stops a rung reading
//!   the environment at all. Duplicating the registry check here would be a third copy of a rule
//!   that already fires twice.

use ndn_radio_drivers::bringup_coverage::{BRINGUP_COVERAGE, PlanCoverage};
use ndn_radio_hal::bringup::plan::{Assert, Plan, StepId};
use ndn_radio_hal::bringup::{Role, Severity, Stage, StepClass};
use std::path::{Path, PathBuf};

mod common;
use common::{blank_literals, calls, functions, rust_sources};

// ─────────────────────────────────────────────────────────────────────────────
// Erasing the plans: `Plan<B>` is generic, and these checks are not
// ─────────────────────────────────────────────────────────────────────────────

#[derive(Debug)]
struct EStep {
    id: &'static str,
    stage: Stage,
    class: StepClass,
    why: &'static str,
    follow: &'static [StepId],
    precede: &'static [StepId],
}

#[derive(Debug)]
struct EPlan {
    /// `PlanId::part` — the string a report prints.
    part: &'static str,
    name: &'static str,
    ver: u16,
    /// The `static` this plan is bound to, for a message a reader can grep.
    binding: &'static str,
    role: Role,
    steps: Vec<EStep>,
    excluded: &'static [(Stage, &'static str)],
    check: Result<(), String>,
}

impl EPlan {
    fn label(&self) -> String {
        format!(
            "{} ({}/{}@v{})",
            self.binding, self.part, self.name, self.ver
        )
    }
}

macro_rules! erase {
    ($binding:ident) => {{
        let p: &Plan<_> = &ndn_radio_drivers::$binding;
        EPlan {
            part: p.id.part,
            name: p.id.name,
            ver: p.id.ver,
            binding: stringify!($binding),
            role: p.role,
            steps: p
                .steps
                .iter()
                .map(|s| EStep {
                    id: s.id.0,
                    stage: s.stage,
                    class: s.class,
                    why: s.why,
                    follow: s.must_follow,
                    precede: s.must_precede,
                })
                .collect(),
            excluded: p.excluded,
            check: p.check().map_err(|e| e.to_string()),
        }
    }};
    ($binding:ident, $path:path) => {{
        let p: &Plan<_> = &$path;
        EPlan {
            part: p.id.part,
            name: p.id.name,
            ver: p.id.ver,
            binding: stringify!($binding),
            role: p.role,
            steps: p
                .steps
                .iter()
                .map(|s| EStep {
                    id: s.id.0,
                    stage: s.stage,
                    class: s.class,
                    why: s.why,
                    follow: s.must_follow,
                    precede: s.must_precede,
                })
                .collect(),
            excluded: p.excluded,
            check: p.check().map_err(|e| e.to_string()),
        }
    }};
}

/// Every plan in the crate. **A plan missing from this list is invisible to this file**, which is
/// why `every_plan_static_in_the_crate_is_listed_here` reads them out of the source instead of
/// trusting the list.
fn all_plans() -> Vec<EPlan> {
    // `mut` is used only by the feature-gated pushes below.
    #[allow(unused_mut)]
    let mut v = vec![
        erase!(PLAN_AR9271_MONITOR),
        erase!(PLAN_AR9271_RX),
        erase!(PLAN_8733B_MONITOR),
        erase!(PLAN_8733B_TX),
        erase!(PLAN_A81A),
        erase!(PLAN_8812AU_MONITOR),
        erase!(PLAN_8821CU_MONITOR),
        erase!(PLAN_8821CU_FW_STA),
        erase!(PLAN_8821CU_NO_TXEN),
        erase!(PLAN_8821CU_STATION_REGS),
        erase!(PLAN_8821CU_IBSS),
        erase!(PLAN_MT7610U),
        erase!(PLAN_MT7612U),
        erase!(PLAN_MT7921AU),
        erase!(PLAN_NRC7292, ndn_radio_drivers::halow::PLAN_NRC7292),
        erase!(PLAN_MM6108, ndn_radio_drivers::halow::PLAN_MM6108),
    ];
    #[cfg(feature = "serial-radio")]
    v.push(erase!(PLAN_SERIAL_BRIDGE));
    #[cfg(feature = "lora")]
    v.push(erase!(PLAN_LORA_NODE));
    v
}

/// Plans whose `static` is behind a cargo feature, so an absence from [`all_plans`] under default
/// features is a declared absence rather than an omission.
const FEATURE_GATED_PLANS: &[(&str, &str)] = &[
    ("PLAN_SERIAL_BRIDGE", "serial-radio"),
    ("PLAN_LORA_NODE", "lora"),
];

/// The §1.5 readbacks, erased the same way. Keyed by the part they belong to.
struct EAssert {
    id: &'static str,
    reg: u32,
    want: u32,
    mask: u32,
    why: &'static str,
    severity: Severity,
}

fn erase_asserts<B: 'static>(part: &'static str, a: &[Assert<B>]) -> Vec<(&'static str, EAssert)> {
    a.iter()
        .map(|x| {
            (
                part,
                EAssert {
                    id: x.id.0,
                    reg: x.reg,
                    want: x.want,
                    mask: x.mask,
                    why: x.why,
                    severity: x.severity,
                },
            )
        })
        .collect()
}

fn all_asserts() -> Vec<(&'static str, EAssert)> {
    use ndn_radio_hal::bringup::plan::BringUp;
    let mut v = Vec::new();
    macro_rules! take {
        ($name:literal, $t:ty) => {
            v.extend(erase_asserts($name, <$t as BringUp>::asserts()))
        };
    }
    take!("Ath9kHtcBackend", ndn_radio_drivers::Ath9kHtcBackend);
    take!("Rtl8733buBackend", ndn_radio_drivers::Rtl8733buBackend);
    take!(
        "LibUsbRtl88xxBackend",
        ndn_radio_drivers::LibUsbRtl88xxBackend
    );
    take!("Rtl8812auBackend", ndn_radio_drivers::Rtl8812auBackend);
    take!("Rtl8821cuBackend", ndn_radio_drivers::Rtl8821cuBackend);
    take!("Mt7610uBackend", ndn_radio_drivers::Mt7610uBackend);
    take!("Mt7612uBackend", ndn_radio_drivers::Mt7612uBackend);
    take!("Mt7921uBackend", ndn_radio_drivers::Mt7921uBackend);
    take!("Nrc7292BringUp", ndn_radio_drivers::halow::Nrc7292BringUp);
    take!("Mm6108BringUp", ndn_radio_drivers::halow::Mm6108BringUp);
    #[cfg(feature = "serial-radio")]
    take!("SerialRadioBackend", ndn_radio_drivers::SerialRadioBackend);
    #[cfg(feature = "lora")]
    take!("LoraSerialBackend", ndn_radio_drivers::LoraSerialBackend);
    v
}

// ─────────────────────────────────────────────────────────────────────────────
// The driver sources
// ─────────────────────────────────────────────────────────────────────────────

fn src_files() -> Vec<PathBuf> {
    let mut v = Vec::new();
    rust_sources(&Path::new(env!("CARGO_MANIFEST_DIR")).join("src"), &mut v);
    v.sort();
    assert!(
        v.len() > 15,
        "only {} driver sources found — every scan below would be vacuous",
        v.len()
    );
    v
}

fn src_blanked() -> Vec<(PathBuf, String)> {
    src_files()
        .into_iter()
        .map(|p| {
            let s = std::fs::read_to_string(&p).expect("read driver source");
            (p, blank_literals(&s))
        })
        .collect()
}

/// One `Step { … }` literal read out of the source: its id, its class, and the `run:` function.
///
/// The runtime plans give id and class already; what only the source gives is **which driver
/// function a rung runs**, which is the hinge of LAW 5.
///
/// ★ `blank_literals` preserves byte offsets, so the brace counting runs over the BLANKED text
/// (where a `"{"` inside a `why` cannot derail it) while `StepId("…")` is read out of the RAW
/// text at the same span. That is the whole reason the lexer blanks in place instead of deleting.
#[derive(Debug, Clone)]
struct SrcStep {
    id: String,
    class: String,
    run: String,
}

fn src_steps(raw: &str, blanked: &str) -> Vec<SrcStep> {
    let mut out = Vec::new();
    let mut from = 0usize;
    while let Some(rel) = blanked[from..].find("Step {") {
        let at = from + rel;
        from = at + 1;
        let (start, body) = common::body_after(blanked, at);
        if body.is_empty() {
            continue;
        }
        let raw_body = &raw[start..start + body.len()];
        let field = |src: &str, k: &str| -> Option<String> {
            let i = src.find(k)? + k.len();
            Some(
                src[i..]
                    .chars()
                    .skip_while(|c| c.is_whitespace())
                    .take_while(|c| c.is_alphanumeric() || *c == '_')
                    .collect(),
            )
        };
        let (Some(run), Some(class)) = (field(body, "run:"), field(body, "class: StepClass::"))
        else {
            continue;
        };
        if run.is_empty() {
            continue;
        }
        let id = raw_body
            .find("id: StepId(\"")
            .map(|i| {
                let s = &raw_body[i + "id: StepId(\"".len()..];
                s[..s.find('"').unwrap_or(0)].to_string()
            })
            .unwrap_or_default();
        out.push(SrcStep { id, class, run });
    }
    out
}

// ─────────────────────────────────────────────────────────────────────────────
// 1. Every plan holds together — the named form of what `check_or_panic` says in literals
// ─────────────────────────────────────────────────────────────────────────────

/// A `why` bar. §6.3 asks only for **non-empty**, which `Plan::check` already enforces; the bar
/// here is 60 characters, because "non-empty" is satisfied by `"required"` and by `"TODO"`. It is
/// the same discipline `coverage.rs` applies to `Seam::Excluded` (>40) and
/// `bringup_coverage.rs` to its exclusions (>80). The crate's SHORTEST `why` today is 109
/// characters, so there is real headroom and this bar cannot be met by accident.
const WHY_MIN: usize = 60;

#[test]
fn every_plan_holds_together() {
    let plans = all_plans();
    assert!(
        plans.len() >= 16,
        "only {} plans erased — the file-wide checks would be nearly vacuous",
        plans.len()
    );
    let mut rungs = 0usize;
    for p in &plans {
        if let Err(e) = &p.check {
            panic!("{} does not hold together: {e}", p.label());
        }
        assert!(
            !p.steps.is_empty(),
            "{} has no rungs — an empty plan reports `CANONICAL` and brings nothing up",
            p.label()
        );
        for (i, s) in p.steps.iter().enumerate() {
            rungs += 1;
            assert!(
                s.why.trim().len() >= WHY_MIN,
                "{}::{} has a {}-character `why`: {:?}. Every rung states the MEASUREMENT or the \
                 vendor reference that puts it there — that sentence is the only thing standing \
                 between this sequence and the sixteen private ladders it replaced",
                p.label(),
                s.id,
                s.why.trim().len(),
                s.why
            );
            // Duplicate ids and ordering are also `Plan::check`'s job; re-stated here because a
            // `const` panic cannot name the rung and this message can.
            assert!(
                p.steps[..i].iter().all(|q| q.id != s.id),
                "{} names `{}` twice — a `Deviation` skipping it would hit whichever came first, \
                 and the digest could not tell the two runs apart",
                p.label(),
                s.id
            );
            for want in s.follow {
                let at = p.steps.iter().position(|q| q.id == want.0);
                assert!(
                    at.is_some_and(|j| j < i),
                    "{}::{} must_follow `{}`, and the sequence does not satisfy it. Ordering \
                     constraints exist for MEASURED reasons — TSSI before enable_tx gives 19.6 dB \
                     of usable range, the other order 1.6 dB",
                    p.label(),
                    s.id,
                    want.0
                );
            }
            for want in s.precede {
                let at = p.steps.iter().position(|q| q.id == want.0);
                assert!(
                    at.is_some_and(|j| j > i),
                    "{}::{} must_precede `{}`, and the sequence does not satisfy it",
                    p.label(),
                    s.id,
                    want.0
                );
            }
        }
        // `OutOfBand` is honest for HaLow and a lie anywhere else, so the establisher it names
        // is the whole content of the claim: it is the only place a reader can go to find out
        // what actually configured the radio. A blank one turns the rung into `Done` with extra
        // steps.
        for s in &p.steps {
            if let StepClass::OutOfBand { established_by } = s.class {
                assert!(
                    established_by.trim().len() >= 8,
                    "{}::{} is `OutOfBand` and names no establisher. The rung asserts that some \
                     OTHER process brought this radio up; without a name for that process the \
                     report says work was done and points at nobody",
                    p.label(),
                    s.id
                );
            }
        }
        for (stage, ruling) in p.excluded {
            assert!(
                ruling.trim().len() >= WHY_MIN,
                "{} excludes {stage:?} with a {}-character ruling: {ruling:?}. Same discipline as \
                 `coverage::Seam::Excluded` — a blank cell is a written decision, not an absence",
                p.label(),
                ruling.trim().len()
            );
        }
    }
    assert!(
        rungs > 80,
        "only {rungs} rungs across {} plans — the erasure is broken and every check above passed \
         on nothing",
        plans.len()
    );
    println!("{} plans, {rungs} rungs, all hold together", plans.len());
}

/// ★ **Every `Plan` static in the crate is bound to a `const _: () = …check_or_panic();`, and is
/// listed in [`all_plans`].**
///
/// The first half is the hole `Plan::check` cannot cover: nothing makes a driver author add the
/// const assertion, and without it a malformed plan is a runtime surprise on a radio instead of a
/// build failure on a laptop. The second half is the hole THIS FILE cannot cover: a plan absent
/// from `all_plans` is invisible to every check above, silently.
#[test]
fn every_plan_static_is_const_checked_and_listed_here() {
    let mut statics: Vec<(String, String)> = Vec::new(); // (PLAN_NAME, backing const)
    let mut const_checked: Vec<String> = Vec::new();
    for (p, blanked) in src_blanked() {
        let file = p.file_name().unwrap().to_string_lossy().to_string();
        for line in blanked.lines() {
            let l = line.trim();
            if let Some(rest) = l.strip_prefix("pub static PLAN_")
                && let Some((name, tail)) = rest.split_once(':')
            {
                let backing = tail
                    .rsplit_once('=')
                    .map(|(_, b)| b.trim().trim_end_matches(';').to_string())
                    .unwrap_or_default();
                statics.push((format!("PLAN_{name}"), backing));
            }
            if let Some(rest) = l.strip_prefix("const _: () = ")
                && let Some(name) = rest.strip_suffix(".check_or_panic();")
            {
                const_checked.push(name.to_string());
            }
            let _ = &file;
        }
    }
    assert!(
        statics.len() >= 18,
        "found only {} `pub static PLAN_*` in the crate — the scan is broken",
        statics.len()
    );

    for (name, backing) in &statics {
        assert!(
            const_checked.contains(backing) || const_checked.contains(name),
            "`{name}` (= `{backing}`) has no `const _: () = {backing}.check_or_panic();`. Without \
             it a blank `why`, a duplicate rung id or a violated ordering is discovered when \
             somebody brings that radio UP, not when they build — and the ordering constraints \
             are the ones carrying measurements like 19.6 dB vs 1.6 dB"
        );
    }

    let listed: Vec<&'static str> = all_plans().iter().map(|p| p.binding).collect();
    for (name, _) in &statics {
        if listed.iter().any(|l| l == name) {
            continue;
        }
        let feat = FEATURE_GATED_PLANS
            .iter()
            .find(|(n, _)| n == name)
            .unwrap_or_else(|| {
                panic!(
                    "`{name}` is a plan in this crate and `all_plans()` does not list it, so \
                     NOTHING in this file checks it: not its `why`s, not its ordering, not LAW 5, \
                     not LAW 6. Add it"
                )
            })
            .1;
        assert!(
            !match feat {
                "lora" => cfg!(feature = "lora"),
                "serial-radio" => cfg!(feature = "serial-radio"),
                other => panic!("FEATURE_GATED_PLANS names unknown feature `{other}`"),
            },
            "`{name}` is declared gated on `{feat}`, the feature is ON, and it is still missing \
             from `all_plans()`"
        );
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// 2. The asserts — §1.5's half of the contract, which `Plan::check` never sees
// ─────────────────────────────────────────────────────────────────────────────

/// ☠ **A readback that cannot fail.** `run_plan` evaluates `(read & mask) == (want & mask)`, so
/// `mask: 0` is `true` for every possible chip state: the report says `ok`, forever, and the
/// invariant is not being checked. That is `worst_receiver_rate`'s vacuous-rule defect with a
/// register attached, and nothing else in the tree would notice it.
///
/// Also: every `Assert::why` non-empty. Its doc says *"An empty `why` fails `Plan::check`"* —
/// which is false. `Plan::check` walks `self.steps`; asserts hang off the `BringUp` trait and it
/// never sees them. This test is what makes the doc true.
#[test]
fn every_assert_can_actually_fail_and_says_why() {
    let asserts = all_asserts();
    assert!(
        asserts.len() >= 14,
        "only {} asserts erased across the fleet — the check would be nearly vacuous",
        asserts.len()
    );
    for (part, a) in &asserts {
        assert_ne!(
            a.mask, 0,
            "{part}::{} reads {:#06x} under mask 0 — `(read & 0) == (want & 0)` is TRUE for every \
             possible chip state, so this readback reports `ok` in every report forever and \
             checks nothing. §1.5's whole point is reading back a gate the ladder WROTE",
            a.id, a.reg
        );
        assert!(
            a.why.trim().len() >= WHY_MIN,
            "{part}::{} has a {}-character `why`: {:?}. An assert costs a bus round trip on every \
             bring-up; the `why` is what justifies it and what an operator reads when it fires",
            a.id,
            a.why.trim().len(),
            a.why
        );
        assert_eq!(
            a.want & !a.mask,
            0,
            "{part}::{} wants {:#x} but masks {:#x} — the bits outside the mask are never \
             compared, so the `want` states an expectation this assert does not test",
            a.id,
            a.want,
            a.mask
        );
        // §1.5: `Warn` on introduction for every part; promotion to `Fatal` is per part and needs
        // a measurement. This is not a prohibition on `Fatal` — it is a demand that a `Fatal`
        // readback say, in its own `why`, what run watched it fire.
        if a.severity == Severity::Fatal {
            assert!(
                a.why.contains("MEASURED") || a.why.contains("measured"),
                "{part}::{} is `Fatal` and its `why` cites no measurement. §1.5: `Warn` on \
                 introduction for every part, promoted per part ONLY with a run where it actually \
                 fired — a readback nobody has watched fail is not allowed to refuse a radio",
                a.id
            );
        }
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// 3. Every backend declares a plan for every role, or a written exclusion
// ─────────────────────────────────────────────────────────────────────────────

/// A `FrameIo` backend whose bring-up belongs to a DIFFERENT type. Each entry says which, and
/// why — because "it has no plan" and "its plan lives next door" are different states and only
/// one of them is a gap.
const BRINGUP_DELEGATES: &[(&str, &str, &str)] = &[
    (
        "Bw16SerialBackend",
        "SerialRadioBackend",
        "A thin wrapper (capability, clock domain, power axis) over the same 7E-A5 transport. The \
         BRING-UP is identical — `PLAN_SERIAL_BRIDGE` runs on `SerialRadioBackend` — which is why \
         there is one plan and not two nearly identical ones.",
    ),
    (
        "Esp32SerialBackend",
        "SerialRadioBackend",
        "The other wrapper over the same transport. It differs from the BW16 in what it can DO (LE \
         CODED PHY, CSI, a real hardware RX stamp) and not in how it is brought up.",
    ),
    (
        "Nrc7292FrameIo",
        "Nrc7292BringUp",
        "The Linux AF_PACKET data plane. Its bring-up is `PLAN_NRC7292`, which hangs off a marker \
         type carrying the `HalowIfaces` snapshot — because every rung is `OutOfBand` and \
         validates a netdev that `modprobe`/`iw` created before this process started, so there is \
         nothing for the data-plane type itself to bring up.",
    ),
    (
        "MorseFrameIo",
        "Mm6108BringUp",
        "Same split, and here the marker type carries BOTH interfaces: on the MM6108 it is the TX \
         monitor vif that turns RECEIVE on, so the plan asserts both `/sys/class/net/<if>/type` \
         values and a single-interface data-plane type could not express that.",
    ),
];

/// ★ §6.3's first bullet, in both directions.
///
/// * Every type with an `impl BringUp` has a `bringup_coverage` cell for **all three** roles —
///   the gap `coverage.rs` structurally cannot close for itself, because Rust offers no way to
///   enumerate the impls of a trait and ask what is missing. A source scan can.
/// * Every type with an `impl FrameIo` — i.e. every radio this crate can drive — either has a
///   `BringUp` impl or a written delegation. `Bw16SerialBackend` and `MorseFrameIo` are real
///   radios with no plan of their own, and before this list nothing said so.
#[test]
fn every_backend_declares_a_plan_or_a_written_exclusion() {
    let mut bringup_impls: Vec<String> = Vec::new();
    let mut frame_io_impls: Vec<String> = Vec::new();
    for (_, blanked) in src_blanked() {
        for (marker, out) in [
            ("impl BringUp for ", &mut bringup_impls),
            ("impl FrameIo for ", &mut frame_io_impls),
        ] {
            let mut from = 0usize;
            while let Some(rel) = blanked[from..].find(marker) {
                let at = from + rel + marker.len();
                from = at;
                let name: String = blanked[at..]
                    .chars()
                    .take_while(|c| c.is_alphanumeric() || *c == '_')
                    .collect();
                if !name.is_empty() && !out.contains(&name) {
                    out.push(name);
                }
            }
        }
    }
    assert!(
        bringup_impls.len() >= 12 && frame_io_impls.len() >= 14,
        "scan found {} BringUp and {} FrameIo impls — broken, and every assertion below would \
         pass on nothing",
        bringup_impls.len(),
        frame_io_impls.len()
    );

    for part in &bringup_impls {
        for role in [
            Role::ReceiveOnly,
            Role::TransmitOnly,
            Role::TransmitAndReceive,
        ] {
            let cell = BRINGUP_COVERAGE
                .iter()
                .find(|c| c.part == part && c.role == role);
            let Some(cell) = cell else {
                panic!(
                    "`impl BringUp for {part}` exists and `bringup_coverage` has no cell for \
                     {role:?}. A caller asking for that role gets `PlanError::NoPlan` and no way \
                     to tell a decision from an omission — which is the state HaLow was in, \
                     refusing two roles with `(role == TransmitAndReceive).then_some(..)` and \
                     naming neither"
                )
            };
            if let PlanCoverage::Excluded(reason) = cell.status {
                assert!(
                    reason.trim().len() > 80,
                    "{part} / {role:?} is excluded without a real written reason"
                );
            }
        }
    }

    for io in &frame_io_impls {
        if bringup_impls.contains(io) {
            continue;
        }
        let Some((_, to, reason)) = BRINGUP_DELEGATES.iter().find(|(f, _, _)| f == io) else {
            panic!(
                "`impl FrameIo for {io}` — this crate can drive that radio — and it has neither \
                 an `impl BringUp` nor an entry in `BRINGUP_DELEGATES`. Either it has a plan, or \
                 somebody must write down whose plan brings it up"
            )
        };
        assert!(
            bringup_impls.contains(&to.to_string()),
            "{io} is declared to delegate its bring-up to `{to}`, which has no `impl BringUp` — \
             the delegation points at nothing"
        );
        assert!(
            reason.trim().len() > 80,
            "the {io} -> {to} delegation has no real written reason"
        );
    }

    // ★ Close the loop: the table's `plan:` string names a `static`, that static declares a
    // `role`, and `run_plan` refuses a run whose asked-for role differs (`PlanError::RoleMismatch`).
    // Before this, the table's `plan:` field was decorative — it could name any plan at all, and a
    // mismatch would surface as a bring-up that fails on hardware, at a customer, at 3 a.m.
    //
    // ⚠ Deliberately NOT keyed on `Stage`. A stage is a rendering label — the HAL says so in as
    // many words — and two rules drafted against it here were both FALSE on measured sequences:
    // `PLAN_8733B_TX` puts `Stage::Power` before `Stage::Calibrate` (the TXAGC page is inert,
    // TSSI DE is the actuator) and `PLAN_8733B_MONITOR` carries a `Stage::TxEnable` rung in a
    // `ReceiveOnly` plan (`enable_tx_path` was in the old `bring_up_monitor`, and it does not
    // complete the on-air path — `tssi_setup` + `enable_tx` do, and those are what the roles
    // differ by). Both were dropped rather than exempted.
    let plans = all_plans();
    let mut linked = 0usize;
    for c in BRINGUP_COVERAGE {
        let PlanCoverage::Provided = c.status else {
            continue;
        };
        let Some(p) = plans.iter().find(|p| p.binding == c.plan) else {
            // A plan behind a cargo feature that is off; `bringup_coverage`'s own test is what
            // establishes that such a row is declared rather than missing.
            assert!(
                FEATURE_GATED_PLANS.iter().any(|(n, _)| *n == c.plan),
                "`bringup_coverage` says {} / {:?} is served by `{}`, and there is no such plan \
                 in this crate",
                c.part,
                c.role,
                c.plan
            );
            continue;
        };
        linked += 1;
        assert_eq!(
            p.role, c.role,
            "`bringup_coverage` says {} serves {:?} with `{}`, and that plan declares \
             `role: {:?}`. `run_plan` refuses a run whose role differs from its plan's \
             (`PlanError::RoleMismatch`), so this pairing fails on hardware and nowhere else",
            c.part, c.role, c.plan, p.role
        );
    }
    assert!(
        linked >= 12,
        "only {linked} coverage cells linked to a plan static — the `plan:` strings and the \
         bindings have drifted apart and this check just passed on nothing"
    );

    // A stale delegate is a radio nobody is looking at. Both halves must still be real types.
    for (from, to, _) in BRINGUP_DELEGATES {
        assert!(
            frame_io_impls.contains(&from.to_string()),
            "`{from}` is listed as delegating its bring-up to `{to}`, and it no longer has an \
             `impl FrameIo` — delete the entry rather than leaving a name nothing checks"
        );
    }
    println!(
        "{} BringUp impls x 3 roles covered; {} FrameIo impls, {} delegating",
        bringup_impls.len(),
        frame_io_impls.len(),
        BRINGUP_DELEGATES.len()
    );
}

// ─────────────────────────────────────────────────────────────────────────────
// 4. LAW 5 — a completion poll is never best-effort
// ─────────────────────────────────────────────────────────────────────────────

/// §6.3's completion-poll lexicon, verbatim. A function whose body names one of these is polling
/// a hardware completion bit.
const POLL_TOKENS: &[&str] = &[
    "poll32",
    "LLT_NO_ACTIVE",
    "AUTO_LLT",
    "WINTINI_RDY",
    "mcu_responsive",
];

/// A part's source files. LAW 5 is a relation between a plan and the driver behind it, and driver
/// method names collide across parts (`mac_config` exists three times), so the call graph is built
/// **per part**, over exactly the files that part's rungs can reach.
const PART_SOURCES: &[(&str, &[&str])] = &[
    ("ath9k", &["src/ath9k_htc.rs"]),
    ("halow", &["src/halow.rs"]),
    ("8733b", &["src/libusb_rtl8733b.rs"]),
    ("a81a", &["src/libusb_rtl88xx.rs"]),
    ("8812au", &["src/rtl8812au.rs"]),
    ("serial", &["src/serial_radio.rs"]),
    ("lora", &["src/lora_serial.rs"]),
    (
        "8821c",
        &[
            "src/rtl8821c/mod.rs",
            "src/rtl8821c/mac.rs",
            "src/rtl8821c/fw.rs",
            "src/rtl8821c/coex.rs",
            "src/rtl8821c/phy.rs",
            "src/rtl8821c/rf.rs",
        ],
    ),
    // The mt76x0 and mt7612 ports SHARE `src/mt76/`, so it appears in both groups — a shared
    // helper reached from either part's rungs is that part's poll too.
    ("mt7610", &["src/mt76x0/mod.rs", "src/mt76/mod.rs"]),
    ("mt7612", &["src/mt7612/mod.rs", "src/mt76/mod.rs"]),
    ("mt7921", &["src/mt7921/mod.rs", "src/connac2/mod.rs"]),
];

/// ★ **The written exceptions to LAW 5**: a completion poll reached from a rung that is NOT
/// `Required`. Each must say why the class stands, and what would lift it.
const LAW5_BEST_EFFORT: &[(&str, &str, &str, &str)] = &[(
    "8821c",
    "coex_grant_wl",
    "ltecoex_wait_ready",
    "☠ LAW 5 IS RIGHT HERE AND THE CLASS STANDS ANYWAY, on provenance grounds. \
     `ltecoex_wait_ready` polls `LTECOEX_CTRL` for `LTECOEX_READY` (1000 tries) before every \
     LTE-coex indirect read/write, and this rung's OWN degradation text says the failure mode is \
     that `a PTA-gated antenna is indistinguishable from a dead one at the host` — i.e. exactly \
     the silent failure LAW 5 exists to stop. It is `BestEffort` because that is what the ladder \
     was: transcribed verbatim from `eprintln!(\"8821cu coex/grant-WL failed\")`, on the ONE part \
     in this crate whose bring-up has never been shown to work end to end (440 + 439 injected \
     frames, zero at a witness). Promoting it to `Required` would turn a warning into a refused \
     radio on that part, which is an unmeasured behaviour change nobody here can check. LIFTED BY: \
     the bench session that first gets this port to a witness — promote it there, with the run.",
)];

/// ★ **Completion polls that no rung reaches**, with the ruling. Listed so they are visibly ruled
/// out rather than silently ignored by a reachability computation.
const LAW5_OFF_PLAN: &[(&str, &str, &str)] = &[(
    "8812au",
    "llt_write",
    "`llt_write` polls `REG_LLT_INIT[31:30]` for `LLT_NO_ACTIVE` and is reached only from \
     `init_llt`, which is DELIBERATELY NOT A RUNG on this part. `init_llt` was ADDED during the \
     2026-09-03 hunt for the \"does not transmit\" symptom and MEASURED to change nothing; the \
     cause was the power regime (M1). The contract names it among the refuted steps and \
     `plan_shape_8812au` fails if it comes back. So this poll is correctly unreachable, and the \
     entry exists so that `no rung reaches it` reads as a ruling rather than as a gap in the \
     reachability walk. LIFTED BY: nothing short of a measurement showing `init_llt` matters.",
)];

/// Build a name -> body map for one part's sources, and a rung-fn -> class map from its `Step`
/// literals.
fn part_graph(files: &[&str]) -> (Vec<(String, usize, String)>, Vec<SrcStep>) {
    let root = Path::new(env!("CARGO_MANIFEST_DIR"));
    let mut fns = Vec::new();
    let mut steps = Vec::new();
    for f in files {
        let p = root.join(f);
        let Ok(raw) = std::fs::read_to_string(&p) else {
            continue;
        };
        let blanked = blank_literals(&raw);
        fns.extend(functions(&blanked));
        steps.extend(src_steps(&raw, &blanked));
    }
    (fns, steps)
}

/// Does `from` reach `target` through same-part function calls?
fn reaches(
    fns: &[(String, usize, String)],
    from: &str,
    target: &str,
    seen: &mut Vec<String>,
) -> bool {
    if from == target {
        return true;
    }
    if seen.iter().any(|s| s == from) {
        return false;
    }
    seen.push(from.to_string());
    let Some((_, _, body)) = fns.iter().find(|(n, _, _)| n == from) else {
        return false;
    };
    if body.contains(&format!(".{target}(")) || calls(body, target) {
        return true;
    }
    for (n, _, _) in fns {
        if n != from
            && (body.contains(&format!(".{n}(")) || calls(body, n))
            && reaches(fns, n, target, seen)
        {
            return true;
        }
    }
    false
}

#[test]
fn law_5_a_completion_poll_is_never_best_effort() {
    let mut links = 0usize;
    let mut off_plan_seen: Vec<(String, String)> = Vec::new();
    let mut exceptions_seen: Vec<(String, String, String)> = Vec::new();

    for (part, files) in PART_SOURCES {
        let (fns, steps) = part_graph(files);
        assert!(
            !fns.is_empty(),
            "{part}: no functions parsed from {files:?} — the path list is stale and LAW 5 is \
             vacuous for this part"
        );
        let polls: Vec<&(String, usize, String)> = fns
            .iter()
            .filter(|(_, _, body)| POLL_TOKENS.iter().any(|t| body.contains(t)))
            .collect();
        for (poll, _, _) in &polls {
            let reaching: Vec<&SrcStep> = steps
                .iter()
                .filter(|s| reaches(&fns, &s.run, poll, &mut Vec::new()))
                .collect();
            if reaching.is_empty() {
                off_plan_seen.push((part.to_string(), poll.clone()));
                continue;
            }
            for s in reaching {
                links += 1;
                if s.class == "Required" {
                    continue;
                }
                let rung = if s.id.is_empty() { &s.run } else { &s.id };
                let excused = LAW5_BEST_EFFORT
                    .iter()
                    .any(|(p, r, f, _)| p == part && r == rung && f == poll);
                assert!(
                    excused,
                    "LAW 5: `{part}::{poll}` polls a hardware completion bit and is reached from \
                     rung `{rung}`, which is `{}`. A bring-up that WARNS past a completion \
                     timeout keeps writing into a chip that never finished the previous step, and \
                     every rung after it is operating on a machine in a state nobody described. \
                     Make it `Required`, or add a written entry to `LAW5_BEST_EFFORT` saying why \
                     the class stands and what would lift it",
                    s.class
                );
                exceptions_seen.push((part.to_string(), rung.clone(), poll.clone()));
            }
        }
    }

    // ⚠ Vacuity floor. If `reaches` silently stops working, every poll becomes "off plan" and this
    // test passes on a tree with the defect in it — the exact way `worst_receiver_rate` was wrong.
    assert!(
        links >= 8,
        "only {links} (poll -> rung) links found across the fleet. The call-graph walk is broken, \
         so LAW 5 just checked nothing"
    );
    assert!(
        exceptions_seen.len() == LAW5_BEST_EFFORT.len(),
        "the LAW 5 exception list has {} entries and {} fired: {exceptions_seen:?}. An entry that \
         no longer fires is a standing licence — delete it",
        LAW5_BEST_EFFORT.len(),
        exceptions_seen.len()
    );
    for (part, poll, _) in LAW5_OFF_PLAN {
        assert!(
            off_plan_seen.iter().any(|(p, f)| p == part && f == poll),
            "`{part}::{poll}` is listed as a completion poll that no rung reaches, and it is now \
             reachable (or gone). Rewrite the entry"
        );
    }
    for (part, poll) in &off_plan_seen {
        assert!(
            LAW5_OFF_PLAN.iter().any(|(p, f, _)| p == part && f == poll),
            "`{part}::{poll}` polls a hardware completion bit and NO rung reaches it. That is \
             allowed — a knob, a bench helper, or a step deliberately kept out of the plan — but \
             it must be written down in `LAW5_OFF_PLAN`, or a rung that quietly stopped being \
             reachable looks exactly like one that was never meant to be"
        );
    }
    println!(
        "LAW 5: {links} poll->rung links, {} written exceptions, {} off-plan polls",
        LAW5_BEST_EFFORT.len(),
        off_plan_seen.len()
    );
}

// ─────────────────────────────────────────────────────────────────────────────
// 5. LAW 6 — a fact a rung decides reaches the report
// ─────────────────────────────────────────────────────────────────────────────

/// ★ **A rung that constructs a `Fact` must make it reach `RadioState::facts`.**
///
/// See the module header for why the literal reading of §6.3's LAW 6 is not falsifiable and this
/// one is. The two channels a rung may use:
///
/// * `Ok(StepOutcome::Established(fact))` — the runner hoists it (`run_plan` step 3);
/// * `c.state().facts.push(fact)` — the HAND channel, used by the three warm/cold rungs, because
///   `StepOutcome` admits ONE variant per rung and those rungs must also return
///   `Branch("warm"|"cold")`, which is the operator-facing answer.
///
/// A rung that does neither has decided something that changes what every later API means and
/// left it out of the account. `load_tx_power_info` is what that costs: same call, same argument,
/// same `Ok(())`, ~18-33 dB apart.
#[test]
fn law_6_a_fact_a_rung_decides_reaches_the_report() {
    let mut fact_rungs = 0usize;
    let mut warm_rungs = 0usize;
    for (p, blanked) in src_blanked() {
        // ⚠ The RELATIVE PATH, not the basename: `mod.rs` names three different ports in this
        // crate and `mod.rs:2476` sends a reader to the wrong radio.
        let file = p
            .strip_prefix(env!("CARGO_MANIFEST_DIR"))
            .unwrap_or(&p)
            .display()
            .to_string();
        for (name, line, body) in functions(&blanked) {
            // Only rung bodies: a `Fact::` inside a `Step::why` is prose (and was blanked away
            // anyway); a `Fact::` in a report helper is not a rung deciding something.
            if !body.contains("StepOutcome") {
                continue;
            }
            let makes_fact = body.contains("Fact::");
            let reaches_report =
                body.contains("StepOutcome::Established(") || body.contains("facts.push(");
            if makes_fact {
                fact_rungs += 1;
                assert!(
                    reaches_report,
                    "LAW 6: {file}:{line} `{name}` builds a `Fact` and neither returns it as \
                     `StepOutcome::Established(..)` nor pushes it into `c.state().facts`. The rung \
                     has decided something that changes what a later API means, and the report \
                     will not say so — which is `load_tx_power_info` exactly: same call, same \
                     argument, same `Ok(())`, ~18-33 dB apart"
                );
            }
            // The concrete instance, with the better message: warm/cold is decided by a ROUND
            // TRIP and is invisible to every later register read.
            if body.contains("state().warm = ") {
                warm_rungs += 1;
                assert!(
                    makes_fact && reaches_report,
                    "LAW 6: {file}:{line} `{name}` sets `RadioState::warm` and produces no \
                     `Fact::Warm` in `facts`. Warm/cold changes what EVERY rung after it means \
                     and no later register read reveals it — on the mt76 family it decides \
                     whether a captured cold stream was replayed at all. ☠ And it must have been \
                     decided by `mcu_responsive()`'s round trip, never by a status latch: \
                     inferring it from status bits was wrong in BOTH directions and cost a replug \
                     each time"
                );
            }
        }
    }
    // ⚠ Vacuity floor, twice over: `StepOutcome` gating the loop, and `Fact::` surviving the
    // lexer. If either stops matching, this test passes on any tree at all.
    assert!(
        fact_rungs >= 10,
        "only {fact_rungs} rungs were seen constructing a `Fact` — the crate has at least eleven \
         (firmware x5, power reference x5, gain table, warm x3). The scan is broken and LAW 6 \
         just checked nothing"
    );
    assert_eq!(
        warm_rungs, 3,
        "expected exactly the three warm/cold decision RUNGS — `mt7921::s_firmware_ready`, \
         `mt76x0::s_firmware_ready` and `mt7612::s_firmware_and_init` (which writes \
         `state().warm` twice, once per arm, inside ONE rung) — and found {warm_rungs}. If this \
         went to zero the assertion above it is dead and nothing checks the hand-pushed \
         `Fact::Warm` at all"
    );
    println!("LAW 6: {fact_rungs} fact-producing rungs, {warm_rungs} warm/cold decisions");
}

// ─────────────────────────────────────────────────────────────────────────────
// 6. The power rung's position is a CONSTRAINT, not an accident of array order
// ─────────────────────────────────────────────────────────────────────────────

/// §6.3: *"the power step is last among steps that touch the gain chain"*.
///
/// ☠ **As stated, that rule is FALSE on this fleet, and the counterexample is measured.** In
/// `PLAN_8733B_TX` the `Stage::Power` rung `set_txagc_table` comes BEFORE `tssi_setup`, on
/// purpose: on that part the per-rate TXAGC page is MEASURED INERT (efuse `power_track_type = 4`)
/// and the knob that actually moves power is the TSSI DE, which `tssi_setup` arms afterwards — and
/// TSSI must itself precede `enable_tx` (19.6 dB of usable range against 1.6 dB). A guard written
/// to §6.3's letter would fail that plan and the only ways to make it pass would be to reorder a
/// measured sequence or to relabel a stage. Both are worse than the rule.
///
/// ⚠ So this test enforces the **checkable** half, which is also the half that regresses: whether
/// the ordering was WRITTEN DOWN. Which rungs "touch the gain chain" is a per-silicon fact no
/// source scan can decide — on the 8812au it is `load_tx_power_info` + the IQK/LCK pair, on the
/// AR9271 the board cal, on the 8733b the TSSI loop. But a `Stage::Power` rung that declares no
/// `must_follow` sits where it sits because of the order somebody typed the array in, and moving a
/// line then silently reorders the radio. Every power rung in the crate declares one today; this
/// keeps it that way, and `Plan::check` turns the declaration into an enforced constraint.
#[test]
fn the_power_rung_declares_its_ordering() {
    let mut power_rungs = 0usize;
    for p in all_plans() {
        for s in &p.steps {
            if s.stage != Stage::Power {
                continue;
            }
            power_rungs += 1;
            assert!(
                !s.follow.is_empty(),
                "{}::{} is the power rung and declares no `must_follow`. Its position is then an \
                 accident of the order the step array was typed in, and moving a line reorders \
                 the radio silently. The gain chain has to be written down: on the RTL8812AU \
                 `set_tx_power` follows `load_tx_power_info`, `iq_calibrate` and `lc_calibrate`, \
                 and that is what stops the calibrated and raw regimes — ~18-33 dB apart — from \
                 being reachable by the same call",
                p.label(),
                s.id
            );
        }
    }
    assert!(
        power_rungs >= 5,
        "only {power_rungs} `Stage::Power` rungs found across the fleet — the erasure is broken"
    );
}
