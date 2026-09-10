//! **§1.4 the plan · §1.5 the asserts · §1.6 the deviation · and the runner that executes one.**
//!
//! Specification: `ndn-radio-drivers/docs/bringup-contract.md`. Measurement that forced it:
//! `docs/bringup-root-cause-2026-09-03.md`.
//!
//! ## What this is for
//!
//! Defect class 2 of the root cause: *several bring-up sequences per part, with no owner*. Sixteen
//! hand-rolled RTL8812AU ladders differed by a ~20 dB power regime and nobody could see which one
//! a number came from. A [`Plan`] is one ordered, named, reviewable sequence per part per
//! [`Role`]; [`run_plan`] executes it into the [`BringUpReport`] that says which one ran.
//!
//! ## The three properties that make it more than a table
//!
//! 1. **Every rung states why it is there.** [`Step::why`] is `&'static str` and
//!    [`Plan::check`] rejects an empty one. This is `coverage::Seam::Excluded`'s discipline —
//!    a blank cell is a written decision, not an absence — applied to rungs.
//! 2. **Ordering that the sequence alone does not explain is a constraint, not prose.**
//!    `tssi_setup.must_precede = ["enable_tx"]` because TSSI-first gives 19.6 dB of usable range
//!    and the other order 1.6 dB. That sentence used to live in a doc comment.
//! 3. **A failure still says how far it got.** A [`StepClass::Required`] failure returns
//!    [`BringUpFailure`] carrying the **partial** report — today a failed bring-up is a bare
//!    `FaceError` and a day of bisection.
//!
//! ## Steps may be subtracted from outside the driver crate. They may never be added.
//!
//! There is no `Plan::then()`, no `push`, no `append`. [`Deviation`] can [`skip`](Deviation::skip)
//! and [`stop_after`](Deviation::stop_after); adding a rung means editing the `static`, where a
//! reviewer sees it and where the `why` is required. `init_llt` and the TXPAUSE clear were both
//! added on plausible reasoning, both A/B'd, both **reverted**. Adding steps "just in case" is the
//! disease.
//!
//! ## ⚠ Adoption status
//!
//! **One part declares a [`Plan`]: the RTL8733BU** (M3 — `PLAN_8733B_MONITOR` /
//! `PLAN_8733B_TX` in `ndn-radio-drivers/src/libusb_rtl8733b.rs`). It is also the only part that
//! implements [`BringUp::probe_tx`], because it is the only one with a calibrated MAC→BB counter.
//! Every other part still runs its M2 hand-filled ladder; M4–M7 migrate them. Said plainly, per
//! the house rule about capabilities nobody calls.
//!
//! ## Deviations from the spec's §1.4 sketch, recorded rather than silent
//!
//! * **[`StepId`] is a newtype**, where the sketch writes `id: &'static str`. It is
//!   `#[repr(transparent)]` over the same `&'static str` and `From<&'static str>` converts, so
//!   every literal in the sketch still compiles; what it buys is that `must_follow`, a
//!   [`PlanEdit`] and a [`StepRecord`] cannot be crossed with an arbitrary string.
//! * **[`Degradation`] is a struct**, where the sketch writes `BestEffort { degrades: &str }`.
//!   Two fields, both checked non-empty: what is lost, and what the run is still good for. A
//!   `let _ = step` with no declared consequence is not expressible.
//! * **[`StepOutcome::Guard`] carries a name** beside the boxed value, so the report can record
//!   *which* guard the handle owns without downcasting it.
//! * **[`PlanRun`], not `BringUpRequest`.** §1.1 is not built in this pass; `BringUpRequest` is
//!   the type that owns `from_env` (LAW 1), `validate`, `PowerRequest`, `ProofRequirement`,
//!   `PumpPolicy` and `PartOpts`. When it lands it grows `fn plan_run(&self) -> PlanRun` and this
//!   becomes its projection. Named differently so the two cannot be confused in the meantime.
//! * **[`DeviationOp::Poke`] and [`DeviationOp::RegulatoryOverride`] are recorded, not executed**
//!   by the runner — a poke needs register access the HAL cannot name, and the override is
//!   consumed by `PowerRequest::Raw` at the knob. Both change the digest.

use std::any::Any;
use std::sync::Arc;
use std::time::{Instant, SystemTime};

use crate::FaceError;

use super::{
    AssertRecord, BringUpFailure, BringUpReport, DeviationOp, Fact, PlanId, ProofRequirement,
    Provenance, RadioState, RequestError, Role, Severity, Stage, StepClass, StepOutcomeRecord,
    StepRecord, TxInstrument, TxProbe, TxProof, Warning,
};

// ─────────────────────────────────────────────────────────────────────────────
// Identity
// ─────────────────────────────────────────────────────────────────────────────

/// The stable name of one rung. Used by the report, by a [`Deviation`], by `--skip`, and by the
/// [`BringUpReport::plan_digest`].
///
/// It is the *contract* between a plan and every experiment that touches it: rename a step and
/// every deviation naming it fails loudly at [`Plan::resolve`], with
/// [`RequestError::UnknownStep`]. That is exactly what did not happen when `8307161` changed
/// `lc_calibrate` on 2026-08-31 and sixteen private ladders did not notice.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, PartialOrd, Ord)]
#[repr(transparent)]
pub struct StepId(pub &'static str);

impl StepId {
    pub const fn as_str(&self) -> &'static str {
        self.0
    }
}

impl From<&'static str> for StepId {
    fn from(s: &'static str) -> Self {
        Self(s)
    }
}

impl std::fmt::Display for StepId {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.write_str(self.0)
    }
}

impl PartialEq<str> for StepId {
    fn eq(&self, other: &str) -> bool {
        self.0 == other
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// §1.4 — the rung
// ─────────────────────────────────────────────────────────────────────────────

/// **What is lost when a [`StepClass::BestEffort`] rung fails.**
///
/// Every bare `let _ = self.something()` in today's ladders discards a real degradation silently.
/// This type is the price of continuing past a failure: name the loss, and name what the run is
/// still good for. [`Plan::check`] rejects either field empty, so "best effort" cannot mean
/// "nobody thought about it".
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Degradation {
    /// What the caller no longer has — in operator terms, not register terms. Not "failed":
    /// *what is missing on air afterwards*.
    pub lost: &'static str,
    /// What the run is still honestly good for. Forces the author to decide whether continuing is
    /// truthful; if nothing survives, the rung is [`StepClass::Required`].
    pub still_valid_for: &'static str,
}

impl Degradation {
    pub const fn new(lost: &'static str, still_valid_for: &'static str) -> Self {
        Self {
            lost,
            still_valid_for,
        }
    }
}

/// **What a rung did.** The live form: [`Guard`](Self::Guard) owns a value and therefore cannot be
/// `Clone`. The report keeps [`StepOutcomeRecord`], which is the same shape minus the value.
pub enum StepOutcome {
    Done,
    /// A step that branched internally says which way. Branching *between* steps is deliberately
    /// not expressible: a warm/cold decision lives INSIDE one named step and reports here, so a
    /// plan is one sequence and not a tree nobody can review.
    ///
    /// ☠ On the mt76 family that decision must use `mcu_responsive()`'s round trip, **not**
    /// `firmware_running()`'s status latch — inferring liveness from status bits was wrong in
    /// BOTH directions and cost a replug each time.
    Branch(&'static str),
    /// A fact that changes what a later API means. **LAW 6: every such step must return one, and
    /// every [`Fact`] lands in [`RadioState::facts`]** — the runner does that, not the driver.
    /// `load_tx_power_info` is the founding case: it decided, invisibly, what `set_tx_power` meant.
    Established(Fact),
    /// A live guard the handle must own — the 8733b `PowerTracker`, whose drop stops a thread.
    /// The value moves into [`Guards`]; the name goes in the report.
    Guard(&'static str, Box<dyn Any + Send + Sync>),
    /// The step decided, from state the plan can see, that it had nothing to do. Distinct from a
    /// deviation skip: this is the plan's own judgement and needs no operator.
    Skipped(&'static str),
}

impl std::fmt::Debug for StepOutcome {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::Done => f.write_str("Done"),
            Self::Branch(b) => write!(f, "Branch({b:?})"),
            Self::Established(x) => write!(f, "Established({x:?})"),
            Self::Guard(n, _) => write!(f, "Guard({n:?}, <live>)"),
            Self::Skipped(s) => write!(f, "Skipped({s:?})"),
        }
    }
}

/// What a running step may see and change. It is deliberately small: a step gets the regime being
/// assembled and nothing else.
///
/// **LAW 1 — nothing inside a bring-up may read the process environment.** Every `NDN_*` a driver
/// reads inside a ladder today belongs on the request, which is read once, at the top, where the
/// caller can see it. A step that calls `std::env::var` re-creates `load_tx_power_info`: hidden
/// state deciding what a later call means.
pub struct Ctx<'a> {
    step: StepId,
    state: &'a mut RadioState,
    warnings: &'a mut Vec<Warning>,
}

impl Ctx<'_> {
    /// The rung currently executing. Warnings are attributed to it automatically.
    pub fn step(&self) -> StepId {
        self.step
    }
    /// The regime being assembled. A step that decides something a later API depends on writes it
    /// here **and** returns [`StepOutcome::Established`].
    pub fn state(&mut self) -> &mut RadioState {
        self.state
    }
    /// Read-only view, for a step that only needs to know what an earlier rung established.
    pub fn state_ref(&self) -> &RadioState {
        self.state
    }
    /// Record a degradation the step chose to continue past. Attributed to this rung.
    pub fn warn(&mut self, degrades: impl Into<String>) {
        self.warnings
            .push(Warning::new(self.step.as_str(), degrades));
    }
}

/// **One rung.** A name, a class, the reason it is there, the ordering it depends on, and the code.
///
/// ★ `run` takes `&Arc<B>`, not `&B`: `Rtl8733buBackend::bring_up_tx_tracked` is
/// `self: &Arc<Self> -> Result<PowerTracker>` and the tracker is a live thread guard. A contract
/// that cannot express the one part already meeting it is not a contract.
pub struct Step<B: 'static> {
    /// Stable id. Used by the report, by a [`Deviation`], by `--skip`, and by the digest.
    pub id: StepId,
    /// A rendering / `--stop-after` label **only**. Not an enforcement device: any sequence can be
    /// made phase-monotone by relabelling. Ordering is enforced by
    /// [`must_follow`](Self::must_follow) / [`must_precede`](Self::must_precede).
    pub stage: Stage,
    pub class: StepClass,
    /// ★ The measurement or vendor reference that puts this rung here. **An empty `why` fails
    /// [`Plan::check`].**
    pub why: &'static str,
    /// Rungs that must appear EARLIER in this plan. Ordering the sequence alone does not explain,
    /// checked at plan construction — so "TSSI before enable_tx" (19.6 dB of usable range against
    /// 1.6 dB) stops being prose.
    pub must_follow: &'static [StepId],
    /// Rungs that must appear LATER in this plan.
    pub must_precede: &'static [StepId],
    pub run: fn(&Arc<B>, &mut Ctx<'_>) -> Result<StepOutcome, FaceError>,
}

impl<B: 'static> std::fmt::Debug for Step<B> {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("Step")
            .field("id", &self.id)
            .field("stage", &self.stage)
            .field("class", &self.class)
            .finish_non_exhaustive()
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// §1.5 — the asserts
// ─────────────────────────────────────────────────────────────────────────────

/// **Read back every gate you write.** The half of the contract that reads the hardware rather
/// than describing it.
///
/// The founding case is `REG_TXPAUSE` on the RTL8812AU: `iqk_configure_mac` drops five things and
/// restores them only in `iq_calibrate`'s tail block, which the `iqk_tx()?` error path skips, and
/// the IQK retry loop can re-save an already-paused value and faithfully restore it. **MEASURED
/// `0x00 / 0x00 / 0x00` over three bring-ups** — structurally real, not firing today. So it is an
/// assert, *not* a rung: the hand-rolled `write8(0x522, 0x00)` was A/B'd inert
/// (2789 / 4508 / 4588 / 4406 frames — run-to-run noise, no direction) and reverted.
///
/// The identical one-line assert catches a **live** defect on a different driver:
/// `libusb_rtl88xx::txgapk_tx_pause()` writes `0x0522 = 0xff` and never resumes — the restore sits
/// at the tail of `txgapk`, whose error is swallowed with a `tracing::warn!`.
///
/// ⚠ **`Warn` on introduction for every part.** Promoted to [`Severity::Fatal`] per part only with
/// a measurement; a readback nobody has watched fail is not allowed to refuse a radio.
pub struct Assert<B: 'static> {
    pub id: StepId,
    /// The register, for the report. Rendering only — [`read`](Self::read) is what runs.
    pub reg: u32,
    pub read: fn(&B) -> Result<u32, FaceError>,
    pub want: u32,
    /// Bits that matter. `(read & mask) == (want & mask)`.
    pub mask: u32,
    /// ★ Why this invariant is worth a bus round trip. **An empty `why` fails [`Plan::check`].**
    pub why: &'static str,
    pub severity: Severity,
}

impl<B: 'static> std::fmt::Debug for Assert<B> {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("Assert")
            .field("id", &self.id)
            .field("reg", &format_args!("{:#06x}", self.reg))
            .field("want", &format_args!("{:#x}", self.want))
            .field("severity", &self.severity)
            .finish_non_exhaustive()
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// §1.6 — the deviation
// ─────────────────────────────────────────────────────────────────────────────

/// One edit to a canonical plan, **with the reason it was made**.
///
/// ★ `why` is a plain `String`, not `Option<String>`: there is no way to spell an edit without a
/// reason. It lands in [`BringUpReport::deviations`] and in the digest.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct PlanEdit {
    pub op: DeviationOp,
    /// What justifies *this* edit. Non-empty, checked by [`Plan::resolve`].
    pub why: String,
}

/// **How a bench experiment departs from the canonical plan, without forking it.**
///
/// Deviation is *supported*, precisely so that experiments stop being implemented as a second
/// bring-up. Five of the sixteen hand-rolled 8812au files exist only to deviate, and they are how
/// everything in this crate got measured. The three-arm bisect that located the 2026-09-03 defect
/// (`skip:pwrinfo` / `skip:edcca` / `skip:iqkloop`) is a shell loop over this type, not three new
/// files.
///
/// Four rules make it a deviation rather than a fork with extra steps:
///
/// 1. **[`question`](Self::question) is mandatory** and non-empty.
/// 2. **It lands in [`BringUpReport::deviations`] and changes [`BringUpReport::plan_digest`]** —
///    a bench number and a production number can never be silently compared.
/// 3. **Edits name steps by id**, so renaming or deleting a rung fails every experiment touching
///    it loudly, at [`Plan::resolve`].
/// 4. **It is constructible from a string** ([`Deviation::parse`]), so `NDN_BRINGUP_DEVIATE`
///    becomes a shell loop. ⚠ This type does **not** read the environment (LAW 1): §1.1's
///    `BringUpRequest::from_env` is the one place allowed to, and it hands the string here.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Deviation {
    question: String,
    edits: Vec<PlanEdit>,
}

impl Deviation {
    /// ★ The question this deviation exists to answer. Rendered as a banner and carried in the
    /// report forever. An empty one is rejected by [`Plan::resolve`], not here, so that a builder
    /// chain stays a builder chain.
    pub fn new(question: impl Into<String>) -> Self {
        Self {
            question: question.into(),
            edits: Vec::new(),
        }
    }

    /// Do not run this rung. **The exact operation that found the 2026-09-03 defect.**
    pub fn skip(mut self, id: impl Into<String>, why: impl Into<String>) -> Self {
        self.edits.push(PlanEdit {
            op: DeviationOp::Skip(id.into()),
            why: why.into(),
        });
        self
    }

    /// Stop after the **last rung this plan labels with that [`Stage`]**. ⚠ `Stage` is a label,
    /// not a monotone gate (Appendix A.2) — a plan's labels need not be sorted — so the cut is
    /// defined by the label itself, and naming a stage the plan does not use is
    /// [`RequestError::UnknownStep`] rather than a silent no-cut.
    pub fn stop_after(mut self, stage: Stage, why: impl Into<String>) -> Self {
        self.edits.push(PlanEdit {
            op: DeviationOp::StopAfter(stage),
            why: why.into(),
        });
        self
    }

    /// A raw register poke. ⚠ **Recorded, not executed by [`run_plan`]** — the HAL cannot name a
    /// register bus. It still changes the digest, because a run that pokes is not the same run.
    pub fn poke(mut self, addr: u32, val: u32, width: u8, why: impl Into<String>) -> Self {
        self.edits.push(PlanEdit {
            op: DeviationOp::Poke { addr, val, width },
            why: why.into(),
        });
        self
    }

    /// ⚠ The regulatory override, recorded. **Recorded, not executed by [`run_plan`]**: the
    /// authority is consumed by `PowerRequest::Raw` at the knob, which is the only thing that can
    /// leave the regulatory scale.
    pub fn regulatory_override(
        mut self,
        authority: crate::bringup::RfAuthority,
        why: impl Into<String>,
    ) -> Self {
        self.edits.push(PlanEdit {
            op: DeviationOp::RegulatoryOverride { authority },
            why: why.into(),
        });
        self
    }

    /// **Combine two deviations that are both in force.**
    ///
    /// ★ M8. `BringUpRequest::from_env` can be handed two at once — a part-specific one
    /// (`NDN_RADIO_SKIP_CAL` on the a81a, `NDN_8733B_NO_TSSI` on the 8733b) and the generic
    /// `NDN_BRINGUP_DEVIATE`. Before this, one of them had to win, and a dropped `skip` is a run
    /// whose report does not describe it. Both questions are kept, joined, because both were
    /// asked; both edit lists are kept, in order, because [`Plan::resolve`] validates every id and
    /// names an unknown one loudly rather than ignoring it.
    pub fn merge(mut self, other: Self) -> Self {
        self.question = format!("{} · {}", self.question, other.question);
        self.edits.extend(other.edits);
        self
    }

    pub fn question(&self) -> &str {
        &self.question
    }
    pub fn edits(&self) -> &[PlanEdit] {
        &self.edits
    }

    /// Parse the `NDN_BRINGUP_DEVIATE` form: `"<question>|<op>[,<op>…]"`, where an op is
    /// `skip:<step-id>` or `stop-after:<stage>`, each optionally `=<why for this edit>`.
    ///
    /// A missing `|` is [`RequestError::MissingWhy`]: rule 1 says a deviation with no stated
    /// reason does not compile, and the shell form is the one place that could smuggle one in. An
    /// edit with no `=` inherits the question, which is non-empty by construction.
    ///
    /// ⚠ `poke` and `regulatory-override` are deliberately **not** parseable from a string: a raw
    /// register write and a regulatory override are decisions, not shell arguments. Build them
    /// with [`Deviation::poke`] / [`Deviation::regulatory_override`].
    pub fn parse(spec: &str) -> Result<Self, RequestError> {
        let (question, ops) = spec
            .split_once('|')
            .ok_or_else(|| RequestError::MissingWhy {
                what: format!("deviation {spec:?} (expected \"<question>|<op>,<op>…\")"),
            })?;
        let question = question.trim();
        if question.is_empty() {
            return Err(RequestError::MissingWhy {
                what: format!("deviation {spec:?}"),
            });
        }
        let mut dev = Deviation::new(question);
        for raw in ops.split(',').map(str::trim).filter(|s| !s.is_empty()) {
            let (op, why) = match raw.split_once('=') {
                Some((op, why)) => (op.trim(), why.trim().to_string()),
                None => (raw, question.to_string()),
            };
            let (kind, arg) = op.split_once(':').ok_or_else(|| {
                RequestError::UnknownStep(format!(
                    "{op} (expected \"skip:<id>\" or \"stop-after:<stage>\")"
                ))
            })?;
            match kind {
                "skip" => dev = dev.skip(arg.trim().to_string(), why),
                "stop-after" | "stop_after" => {
                    let stage = parse_stage(arg.trim()).ok_or_else(|| {
                        RequestError::UnknownStep(format!("stage {:?}", arg.trim()))
                    })?;
                    dev = dev.stop_after(stage, why);
                }
                other => {
                    return Err(RequestError::UnknownStep(format!(
                        "{other} (expected \"skip\" or \"stop-after\")"
                    )));
                }
            }
        }
        Ok(dev)
    }
}

fn parse_stage(s: &str) -> Option<Stage> {
    let n = s.to_ascii_lowercase().replace(['-', '_'], "");
    Some(match n.as_str() {
        "attach" => Stage::Attach,
        "poweron" => Stage::PowerOn,
        "firmware" => Stage::Firmware,
        "macinit" => Stage::MacInit,
        "phyinit" => Stage::PhyInit,
        "tune" => Stage::Tune,
        "calibrate" => Stage::Calibrate,
        "txenable" => Stage::TxEnable,
        "rxenable" => Stage::RxEnable,
        "power" => Stage::Power,
        "posture" => Stage::Posture,
        "verify" => Stage::Verify,
        _ => return None,
    })
}

/// A [`Deviation`] **resolved against a concrete [`Plan`]**: every `skip` here names a rung that
/// exists, and every edit has a non-empty reason. Produced only by [`Plan::resolve`], which is
/// where the two failures are named ([`RequestError::UnknownStep`],
/// [`RequestError::MissingWhy`]) rather than silently ignored.
#[derive(Clone, Debug, PartialEq, Eq, Default)]
pub struct PlanEdits {
    question: String,
    edits: Vec<PlanEdit>,
    skip: Vec<StepId>,
    /// The named stage and the resolved index of the last rung carrying it.
    stop_after: Option<(Stage, usize)>,
}

impl PlanEdits {
    /// The canonical plan: no edits.
    pub fn canonical() -> Self {
        Self::default()
    }
    pub fn is_canonical(&self) -> bool {
        self.edits.is_empty()
    }
    pub fn edits(&self) -> &[PlanEdit] {
        &self.edits
    }
    pub fn question(&self) -> &str {
        &self.question
    }
    /// `Some(why)` if a deviation removed this rung.
    pub fn skip_reason(&self, id: StepId) -> Option<&str> {
        if !self.skip.contains(&id) {
            return None;
        }
        self.edits
            .iter()
            .find(|e| matches!(&e.op, DeviationOp::Skip(s) if s == id.as_str()))
            .map(|e| e.why.as_str())
    }
    /// The self-labelling provenance this run carries forever.
    pub fn provenance(&self) -> Provenance {
        if self.is_canonical() {
            Provenance::Canonical
        } else {
            Provenance::Deviated {
                question: self.question.clone(),
                ops: self.edits.iter().map(|e| e.op.clone()).collect(),
            }
        }
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// §1.4 — the plan
// ─────────────────────────────────────────────────────────────────────────────

/// A plan that does not hold together. Rejected before the first register write.
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum PlanError {
    /// A rung with no stated reason.
    EmptyWhy { plan: PlanId, step: StepId },
    /// A `BestEffort` rung whose declared consequence is blank.
    EmptyDegradation { plan: PlanId, step: StepId },
    /// Two rungs share an id, so a deviation naming it is ambiguous and the digest lies.
    DuplicateStep { plan: PlanId, step: StepId },
    /// An ordering constraint naming a rung this plan does not contain.
    UnknownConstraint {
        plan: PlanId,
        step: StepId,
        names: &'static str,
        kind: &'static str,
    },
    /// An ordering constraint the sequence violates.
    OutOfOrder {
        plan: PlanId,
        step: StepId,
        other: StepId,
        kind: &'static str,
    },
    /// The plan was built for a different role than the run asked for.
    RoleMismatch {
        plan: PlanId,
        planned: Role,
        asked: Role,
    },
    /// No plan is declared for the requested role.
    NoPlan { part: &'static str, role: Role },
}

impl std::fmt::Display for PlanError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::EmptyWhy { plan, step } => write!(
                f,
                "{plan}: step `{step}` has an empty `why`. Every rung states the measurement or \
                 vendor reference that puts it there — a rung nobody justified is how a ladder \
                 accumulates steps that were added on plausible reasoning and never A/B'd."
            ),
            Self::EmptyDegradation { plan, step } => write!(
                f,
                "{plan}: step `{step}` is BestEffort with a blank Degradation. Continuing past a \
                 failure without naming what is lost is the bare `let _ =` this contract removes."
            ),
            Self::DuplicateStep { plan, step } => write!(
                f,
                "{plan}: duplicate step id `{step}`. Ids are the handle a deviation and the digest \
                 use; two rungs with one name makes `skip:{step}` ambiguous."
            ),
            Self::UnknownConstraint {
                plan,
                step,
                names,
                kind,
            } => write!(
                f,
                "{plan}: step `{step}` declares {kind} `{names}`, which this plan does not contain. \
                 Either the constraint is stale or the rung was renamed — this is the failure that \
                 did NOT happen when `8307161` changed `lc_calibrate` and sixteen private ladders \
                 did not notice."
            ),
            Self::OutOfOrder {
                plan,
                step,
                other,
                kind,
            } => write!(
                f,
                "{plan}: step `{step}` declares {kind} `{other}`, and the sequence violates it. \
                 Ordering constraints exist for measured reasons — e.g. TSSI before enable_tx \
                 gives 19.6 dB of usable range, the other order 1.6 dB. This used to be prose in a \
                 doc comment; it is now a constraint, and it is violated."
            ),
            Self::RoleMismatch {
                plan,
                planned,
                asked,
            } => write!(
                f,
                "{plan} is the {planned:?} plan but the run asked for {asked:?}. One role per \
                 plan: `bring_up_monitor` vs `bring_up_tx` as separate functions is the fork this \
                 replaces, not a thing to reintroduce inside one."
            ),
            Self::NoPlan { part, role } => write!(
                f,
                "{part} declares no plan for {role:?}. A part that cannot do a role says so by \
                 returning None, and the caller gets a named error instead of a silent downgrade."
            ),
        }
    }
}

impl std::error::Error for PlanError {}

impl From<PlanError> for FaceError {
    fn from(e: PlanError) -> Self {
        FaceError::Io(std::io::Error::new(
            std::io::ErrorKind::InvalidData,
            e.to_string(),
        ))
    }
}

impl From<RequestError> for FaceError {
    fn from(e: RequestError) -> Self {
        FaceError::Io(std::io::Error::new(
            std::io::ErrorKind::InvalidInput,
            e.to_string(),
        ))
    }
}

/// **One ordered, named, reviewable bring-up sequence, for one part in one [`Role`].**
///
/// Declared as a `static` in the driver that owns the part. Steps may be subtracted from outside
/// the crate (via [`Deviation`]); they may never be added — there is no `then`, `push` or `append`.
pub struct Plan<B: 'static> {
    pub id: PlanId,
    pub role: Role,
    pub steps: &'static [Step<B>],
    /// Stages this part deliberately does nothing in, with the ruling. Same discipline as
    /// `coverage::Seam::Excluded`: **a blank cell is a written decision, not an absence.**
    pub excluded: &'static [(Stage, &'static str)],
}

impl<B: 'static> std::fmt::Debug for Plan<B> {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("Plan")
            .field("id", &self.id)
            .field("role", &self.role)
            .field("steps", &self.steps.len())
            .finish_non_exhaustive()
    }
}

const fn str_eq(a: &str, b: &str) -> bool {
    let (a, b) = (a.as_bytes(), b.as_bytes());
    if a.len() != b.len() {
        return false;
    }
    let mut i = 0;
    while i < a.len() {
        if a[i] != b[i] {
            return false;
        }
        i += 1;
    }
    true
}

impl<B: 'static> Plan<B> {
    /// **Plan construction, checked.** Run by [`run_plan`] before the first register write, and by
    /// the per-part shape test.
    ///
    /// * every [`Step::why`] non-empty, and every `BestEffort` [`Degradation`] fully stated;
    /// * no duplicate ids;
    /// * every `must_follow` / `must_precede` names a rung this plan contains, and the sequence
    ///   satisfies it.
    pub fn check(&self) -> Result<(), PlanError> {
        for (i, s) in self.steps.iter().enumerate() {
            if s.why.trim().is_empty() {
                return Err(PlanError::EmptyWhy {
                    plan: self.id,
                    step: s.id,
                });
            }
            if let StepClass::BestEffort(d) = s.class
                && (d.lost.trim().is_empty() || d.still_valid_for.trim().is_empty())
            {
                return Err(PlanError::EmptyDegradation {
                    plan: self.id,
                    step: s.id,
                });
            }
            if self.steps[..i].iter().any(|p| p.id == s.id) {
                return Err(PlanError::DuplicateStep {
                    plan: self.id,
                    step: s.id,
                });
            }
            for want in s.must_follow {
                match self.steps.iter().position(|p| p.id == *want) {
                    None => {
                        return Err(PlanError::UnknownConstraint {
                            plan: self.id,
                            step: s.id,
                            names: want.as_str(),
                            kind: "must_follow",
                        });
                    }
                    Some(j) if j >= i => {
                        return Err(PlanError::OutOfOrder {
                            plan: self.id,
                            step: s.id,
                            other: *want,
                            kind: "must_follow",
                        });
                    }
                    Some(_) => {}
                }
            }
            for want in s.must_precede {
                match self.steps.iter().position(|p| p.id == *want) {
                    None => {
                        return Err(PlanError::UnknownConstraint {
                            plan: self.id,
                            step: s.id,
                            names: want.as_str(),
                            kind: "must_precede",
                        });
                    }
                    Some(j) if j <= i => {
                        return Err(PlanError::OutOfOrder {
                            plan: self.id,
                            step: s.id,
                            other: *want,
                            kind: "must_precede",
                        });
                    }
                    Some(_) => {}
                }
            }
        }
        Ok(())
    }

    /// The same rules as [`check`](Self::check), evaluable in a `const` context, so a driver can
    /// turn a malformed plan into a **compile error**:
    ///
    /// ```ignore
    /// const _: () = PLAN_8733B_TX.check_or_panic();
    /// ```
    ///
    /// ⚠ `const` panics carry literal messages only, so this says *which rule* broke but not which
    /// rung. [`check`](Self::check) is the one with the message that names the step and what will
    /// go wrong on air; the pair is kept honest by `check_and_const_check_agree`.
    pub const fn check_or_panic(&self) {
        let n = self.steps.len();
        let mut i = 0;
        while i < n {
            let s = &self.steps[i];
            if s.why.is_empty() {
                panic!("PLAN: a Step has an empty `why` — every rung states why it is there");
            }
            if let StepClass::BestEffort(d) = s.class
                && (d.lost.is_empty() || d.still_valid_for.is_empty())
            {
                panic!("PLAN: a BestEffort Step has a blank Degradation — name what is lost");
            }
            let mut j = 0;
            while j < i {
                if str_eq(self.steps[j].id.0, s.id.0) {
                    panic!("PLAN: duplicate step id");
                }
                j += 1;
            }
            let mut k = 0;
            while k < s.must_follow.len() {
                let want = s.must_follow[k].0;
                let mut at = usize::MAX;
                let mut m = 0;
                while m < n {
                    if str_eq(self.steps[m].id.0, want) {
                        at = m;
                        m = n;
                    } else {
                        m += 1;
                    }
                }
                if at == usize::MAX {
                    panic!("PLAN: must_follow names a step this plan does not contain");
                }
                if at >= i {
                    panic!("PLAN: must_follow is violated by the sequence");
                }
                k += 1;
            }
            let mut k = 0;
            while k < s.must_precede.len() {
                let want = s.must_precede[k].0;
                let mut at = usize::MAX;
                let mut m = 0;
                while m < n {
                    if str_eq(self.steps[m].id.0, want) {
                        at = m;
                        m = n;
                    } else {
                        m += 1;
                    }
                }
                if at == usize::MAX {
                    panic!("PLAN: must_precede names a step this plan does not contain");
                }
                if at <= i {
                    panic!("PLAN: must_precede is violated by the sequence");
                }
                k += 1;
            }
            i += 1;
        }
    }

    /// Resolve a [`Deviation`] against this plan. **Every `skip` must name a rung that exists and
    /// every edit must state a reason**, or the caller gets the error by name.
    pub fn resolve(&self, dev: Option<&Deviation>) -> Result<PlanEdits, RequestError> {
        let Some(dev) = dev else {
            return Ok(PlanEdits::canonical());
        };
        if dev.question.trim().is_empty() {
            return Err(RequestError::MissingWhy {
                what: format!("the deviation applied to {}", self.id),
            });
        }
        let mut out = PlanEdits {
            question: dev.question.clone(),
            edits: Vec::new(),
            skip: Vec::new(),
            stop_after: None,
        };
        for e in &dev.edits {
            if e.why.trim().is_empty() {
                return Err(RequestError::MissingWhy {
                    what: format!("edit {:?} of deviation {:?}", e.op, dev.question),
                });
            }
            match &e.op {
                DeviationOp::Skip(name) => {
                    let step = self
                        .steps
                        .iter()
                        .find(|s| s.id.as_str() == name.as_str())
                        .ok_or_else(|| RequestError::UnknownStep(name.clone()))?;
                    out.skip.push(step.id);
                }
                DeviationOp::StopAfter(stage) => {
                    let at = self
                        .steps
                        .iter()
                        .rposition(|s| s.stage == *stage)
                        .ok_or_else(|| {
                            RequestError::UnknownStep(format!(
                                "stage {stage:?} — no rung in {} carries that label",
                                self.id
                            ))
                        })?;
                    out.stop_after = Some(match out.stop_after {
                        // Two stop-afters: the earlier cut wins — the run stops at the first one.
                        Some(prev) if prev.1 <= at => prev,
                        _ => (*stage, at),
                    });
                }
                // Recorded, not executed by the runner. See the module header.
                DeviationOp::Poke { .. } | DeviationOp::RegulatoryOverride { .. } => {}
            }
            out.edits.push(e.clone());
        }
        Ok(out)
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Guards
// ─────────────────────────────────────────────────────────────────────────────

/// Live guards a plan produced, owned by the handle and dropped with it.
///
/// The founding case is the 8733b `PowerTracker`: an untracked TX plan fades as the PA heats, so
/// tracking is **always on** and is delivered as a guard rather than left to a caller's choice.
#[derive(Default)]
pub struct Guards {
    items: Vec<(StepId, &'static str, Box<dyn Any + Send + Sync>)>,
}

impl Guards {
    pub fn new() -> Self {
        Self::default()
    }
    pub fn len(&self) -> usize {
        self.items.len()
    }
    pub fn is_empty(&self) -> bool {
        self.items.is_empty()
    }
    /// The names, in the order the plan produced them. What the report records.
    pub fn names(&self) -> Vec<&'static str> {
        self.items.iter().map(|(_, n, _)| *n).collect()
    }
    /// Take the first guard of type `T` — how a driver gets its `PowerTracker` back out.
    pub fn take<T: Any + Send + Sync>(&mut self) -> Option<Box<T>> {
        let i = self.items.iter().position(|(_, _, g)| g.is::<T>())?;
        let (_, _, g) = self.items.remove(i);
        g.downcast::<T>().ok()
    }
    fn push(&mut self, step: StepId, name: &'static str, g: Box<dyn Any + Send + Sync>) {
        self.items.push((step, name, g));
    }
}

impl std::fmt::Debug for Guards {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("Guards")
            .field("held", &self.names())
            .finish()
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// The runner
// ─────────────────────────────────────────────────────────────────────────────

/// Everything [`run_plan`] needs that is not the plan itself.
///
/// ⚠ **This is not §1.1's `BringUpRequest`.** That type owns `from_env` (LAW 1 — the ONE place
/// allowed to read `NDN_*`), `PowerRequest`, `PumpPolicy` and `PartOpts`, and it lives in
/// `ndn-radio-drivers` because `PartOpts` names driver-owned types. Each part's
/// `bring_up_planned` is the projection from one to the other: it takes the request's fields as
/// arguments and builds this.
#[derive(Clone)]
#[non_exhaustive]
pub struct PlanRun {
    pub part: &'static str,
    pub device: super::DeviceAddress,
    /// The regime the plan starts from and fills in. [`RadioState::role`] is the role the plan is
    /// selected for — one source of truth, not two.
    pub state: RadioState,
    pub deviation: Option<Deviation>,
    /// §4 — what must be PROVEN about the transmitter before the handle is returned. Validated
    /// against [`RadioState::role`] and [`BringUp::tx_instruments`] **before the first register
    /// write**, so an unsatisfiable request is a named error and never a silent pass.
    pub proof: ProofRequirement,
    /// The peer that answers (B). Required by, and only by,
    /// [`ProofRequirement::WitnessOrFail`] — a bring-up on a single host cannot prove radiation.
    pub witness: Option<WitnessOracle>,
}

/// What a peer reports back about our probes. **Only a peer can mint
/// [`TxProof::WitnessDecoded`]**, so this crosses the boundary from the caller, never from a
/// register.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct WitnessReport {
    pub sent: u32,
    pub heard: u32,
    pub rssi_dbm: Option<i8>,
}

/// A peer that can be asked how many of our probes it decoded.
///
/// `Rtl8733buBackend::bring_up_tx_until(verify)` is the founding case: its `verify` closure is
/// exactly this — external feedback, because "no on-chip signal reports radiated power on this
/// part". Boxed rather than a generic parameter so [`PlanRun`] stays a concrete type that
/// `open_radio(pid, …)` can build behind runtime PID dispatch.
pub type WitnessOracle = Arc<dyn Fn() -> Result<WitnessReport, FaceError> + Send + Sync>;

impl std::fmt::Debug for PlanRun {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("PlanRun")
            .field("part", &self.part)
            .field("device", &self.device)
            .field("state", &self.state)
            .field("deviation", &self.deviation)
            .field("proof", &self.proof)
            .field("witness", &self.witness.is_some())
            .finish()
    }
}

impl PlanRun {
    pub fn new(part: &'static str, device: super::DeviceAddress, state: RadioState) -> Self {
        Self {
            part,
            device,
            state,
            deviation: None,
            // §4's production default: probe if the part offers one, say so honestly otherwise.
            proof: ProofRequirement::BestAvailable,
            witness: None,
        }
    }
    pub fn with_deviation(mut self, d: Deviation) -> Self {
        self.deviation = Some(d);
        self
    }
    pub fn with_proof(mut self, proof: ProofRequirement) -> Self {
        self.proof = proof;
        self
    }
    /// Supply the peer that answers (B). Without one,
    /// [`ProofRequirement::WitnessOrFail`] is [`RequestError::Unsatisfiable`] — named, not ignored.
    pub fn with_witness(mut self, oracle: WitnessOracle) -> Self {
        self.witness = Some(oracle);
        self
    }
    pub fn role(&self) -> Role {
        self.state.role
    }
}

/// Implemented once per part. **Deliberately not object-safe**: the factory dispatches on PID to a
/// concrete type, and `&'static Plan<Self>` buys hardware-free inspection of the sequence — a plan
/// can be reviewed, diffed and shape-tested with no radio attached.
///
/// ⚠ **One driver implements this** — `Rtl8733buBackend` (M3). It is otherwise exercised by this
/// module's tests.
pub trait BringUp: Sized + Send + Sync + 'static {
    /// `None` means this part does not do this role — a named refusal, not a silent downgrade.
    fn plan(role: Role) -> Option<&'static Plan<Self>>;

    /// §1.5 readbacks, run after the last rung. **`Warn` on introduction for every part.**
    fn asserts() -> &'static [Assert<Self>] {
        &[]
    }

    /// §4. What could answer "did the MAC key the transmitter?" on this part. An empty slice is a
    /// real answer — the RTL8812AU has no ported Jaguar1 MAC→BB counter — and the runner reports
    /// it as [`TxProof::Unprovable`] rather than as silence.
    fn tx_instruments() -> &'static [TxInstrument] {
        &[]
    }

    /// §4 — **why** this part cannot answer question (A), in the part's own words.
    ///
    /// `None` (the default) leaves the runner's generic sentence. A part that has *measured* its
    /// own blindness overrides this so the refusal carries the measurement instead of a shrug —
    /// §4's table says "`Unprovable`, reason quoted", and a quoted reason is what stops the next
    /// person wiring up the register that was already ruled out. The RTL8822E is the founding
    /// case: `0x2DE0` is NOT a TX-OK counter there (it stays 0 on the working kernel driver
    /// mid-transmit) and `0x2d08` is an RX false-alarm counter, which an example printed as "TX
    /// activity".
    ///
    /// Only consulted when [`tx_instruments`](Self::tx_instruments) is empty: a part that declares
    /// an instrument answers with the instrument, not with prose.
    fn tx_unprovable_reason() -> Option<&'static str> {
        None
    }

    /// **Take question (A): key the transmitter `probes` times and difference `instrument`.**
    ///
    /// `None` — the default — means this part declares no probe, and the runner answers
    /// [`TxProof::Unprovable`] naming the instrument rather than claiming a reading it did not
    /// take. `Some(Err(_))` means the probe was attempted and could not be completed, which is
    /// also `Unprovable`, with the error carried as a warning.
    ///
    /// ⚠ **A probe transmits.** It is the only thing in this module that puts energy on the air,
    /// which is why it is the part's own function: only the part knows how to key its own
    /// transmitter, and only the part's author can judge what a probe frame should be. The runner
    /// owns the *decision* about whether the answer is acceptable, never the transmission.
    ///
    /// ⚠ It answers (A) — *did the MAC key the transmitter?* — and **not** (B).
    /// `ath9k_htc.rs:2655`, MEASURED: the MAC can key the transmitter (TFCNT advances, TXOK
    /// completes) while *"nothing coherent radiates — a witness at inches decoded 0 of our
    /// frames."* A part that can only answer (A) must not be able to spell (B).
    fn probe_tx(
        self: &Arc<Self>,
        instrument: &TxInstrument,
        probes: u16,
    ) -> Option<Result<TxProbe, FaceError>> {
        let _ = (instrument, probes);
        None
    }

    /// See [`run_plan`] for why the large `Err` variant is deliberate.
    #[allow(clippy::result_large_err)]
    fn bring_up(
        self: &Arc<Self>,
        run: &PlanRun,
    ) -> Result<(BringUpReport, Guards), BringUpFailure> {
        let role = run.role();
        match Self::plan(role) {
            Some(plan) => run_plan(self, plan, run),
            None => Err(fail_before_first_rung(
                run,
                PlanId {
                    part: run.part,
                    name: "<none>",
                    ver: 0,
                },
                PlanError::NoPlan {
                    part: run.part,
                    role,
                },
            )),
        }
    }
}

/// **Execute a plan into a [`BringUpReport`].** No per-part knowledge: check the plan, resolve the
/// deviation, walk the rungs, record every outcome and elapsed time, run the asserts, answer the
/// transmit question, build the report, `emit()` it.
///
/// A [`StepClass::Required`] failure returns [`BringUpFailure`] carrying the **partial** report —
/// which rung, in which stage, with everything established up to it.
/// ⚠ `clippy::result_large_err` is allowed deliberately: the `Err` variant is large **because it
/// carries the partial report**, which is the whole point of §3 — a failed bring-up that says how
/// far it got instead of a bare `FaceError`. Boxing it would put an indirection on exactly the
/// path an operator reads at 3 a.m.; a bring-up failure is not a hot path.
#[allow(clippy::result_large_err)]
pub fn run_plan<B: BringUp>(
    b: &Arc<B>,
    plan: &'static Plan<B>,
    run: &PlanRun,
) -> Result<(BringUpReport, Guards), BringUpFailure> {
    // 1. The plan holds together — before the first register write.
    if let Err(e) = plan.check() {
        return Err(fail_before_first_rung(run, plan.id, e));
    }
    if plan.role != run.role() {
        return Err(fail_before_first_rung(
            run,
            plan.id,
            PlanError::RoleMismatch {
                plan: plan.id,
                planned: plan.role,
                asked: run.role(),
            },
        ));
    }

    // 1b. §4 / LAW 4 — the transmit question is answerable AS ASKED, checked before the first
    // register write. A radio that intends to transmit may not decline to look, and a requirement
    // this part cannot satisfy is named here rather than discovered as a silent pass at the end.
    if let Err(e) = validate_proof(
        run.role(),
        &run.proof,
        run.witness.is_some(),
        B::tx_instruments(),
    ) {
        let mut r = blank_report(
            run,
            plan.id,
            Vec::new(),
            Vec::new(),
            &PlanEdits::canonical(),
        );
        r.plan_digest = r.compute_digest();
        return Err(BringUpFailure {
            report: r,
            failed_at: "proof::validate",
            source: e.into(),
        });
    }

    // 2. The deviation names rungs that exist and states its reasons.
    let edits = match plan.resolve(run.deviation.as_ref()) {
        Ok(e) => e,
        Err(e) => {
            let mut r = blank_report(
                run,
                plan.id,
                Vec::new(),
                Vec::new(),
                &PlanEdits::canonical(),
            );
            r.plan_digest = r.compute_digest();
            return Err(BringUpFailure {
                report: r,
                failed_at: "plan::resolve",
                source: e.into(),
            });
        }
    };

    let started_at = SystemTime::now();
    let t0 = Instant::now();
    let mut state = run.state.clone();
    let mut warnings: Vec<Warning> = Vec::new();
    let mut steps: Vec<StepRecord> = Vec::new();
    let mut guards = Guards::new();
    let cut = edits.stop_after.map(|(_, at)| at);

    // 3. The rungs.
    for (i, s) in plan.steps.iter().enumerate() {
        if let Some(why) = edits.skip_reason(s.id) {
            steps.push(record(s, StepOutcomeRecord::Skipped(why.to_string()), 0));
            continue;
        }
        if cut.is_some_and(|c| i > c) {
            steps.push(record(
                s,
                StepOutcomeRecord::Skipped(format!(
                    "stop-after {:?}",
                    edits.stop_after.expect("a cut implies a stop_after").0
                )),
                0,
            ));
            continue;
        }

        let mut ctx = Ctx {
            step: s.id,
            state: &mut state,
            warnings: &mut warnings,
        };
        let t = Instant::now();
        let outcome = (s.run)(b, &mut ctx);
        let us = t.elapsed().as_micros() as u64;

        match outcome {
            Ok(StepOutcome::Done) => steps.push(record(s, StepOutcomeRecord::Done, us)),
            Ok(StepOutcome::Branch(w)) => steps.push(record(s, StepOutcomeRecord::Branch(w), us)),
            Ok(StepOutcome::Established(f)) => {
                // LAW 6 — every Fact lands in RadioState::facts. The runner does it, so a driver
                // cannot establish something and forget to report it.
                if !state.facts.contains(&f) {
                    state.facts.push(f.clone());
                }
                steps.push(record(s, StepOutcomeRecord::Established(f), us));
            }
            Ok(StepOutcome::Guard(name, g)) => {
                guards.push(s.id, name, g);
                steps.push(record(s, StepOutcomeRecord::Guard(name), us));
            }
            Ok(StepOutcome::Skipped(w)) => {
                steps.push(record(s, StepOutcomeRecord::Skipped(w.to_string()), us))
            }
            Err(e) => {
                steps.push(record(s, StepOutcomeRecord::Failed(e.to_string()), us));
                match s.class {
                    // ★ A failure still says how far it got.
                    StepClass::Required => {
                        let mut r = blank_report(run, plan.id, steps, Vec::new(), &edits);
                        r.state = state;
                        r.warnings = warnings;
                        r.elapsed = t0.elapsed();
                        r.started_at = started_at;
                        r.plan_digest = r.compute_digest();
                        return Err(BringUpFailure {
                            report: r,
                            failed_at: s.id.0,
                            source: e,
                        });
                    }
                    StepClass::BestEffort(d) => {
                        warnings.push(Warning::new(
                            s.id.0,
                            format!("{} — still valid for {} ({e})", d.lost, d.still_valid_for),
                        ));
                    }
                    // A readback that could not be taken is not a readback that passed.
                    StepClass::Assert => {
                        warnings.push(Warning::new(
                            s.id.0,
                            format!(
                                "in-ladder assert could not be read ({e}); the invariant it \
                                     guards is UNCHECKED, not satisfied"
                            ),
                        ));
                    }
                    // The work happened out of band; if validating it fails, the radio is not up
                    // and continuing would be a claim about a sequence we did not run.
                    StepClass::OutOfBand { established_by } => {
                        let mut r = blank_report(run, plan.id, steps, Vec::new(), &edits);
                        r.state = state;
                        r.warnings = warnings;
                        r.warnings.push(Warning::new(
                            s.id.0,
                            format!(
                                "out-of-band setup by `{established_by}` did not validate; this \
                                 plan does not own the sequence and cannot repair it"
                            ),
                        ));
                        r.elapsed = t0.elapsed();
                        r.started_at = started_at;
                        r.plan_digest = r.compute_digest();
                        return Err(BringUpFailure {
                            report: r,
                            failed_at: s.id.0,
                            source: e,
                        });
                    }
                }
            }
        }
    }

    // 4. §1.5 — read back every gate the ladder wrote.
    let mut asserts: Vec<AssertRecord> = Vec::new();
    for a in B::asserts() {
        let (read, ok, err) = match (a.read)(b) {
            Ok(v) => (v, (v & a.mask) == (a.want & a.mask), None),
            Err(e) => (0, false, Some(e)),
        };
        asserts.push(AssertRecord {
            id: a.id.0,
            reg: a.reg,
            read,
            want: a.want,
            ok,
            severity: a.severity,
        });
        if ok {
            continue;
        }
        let detail = match &err {
            Some(e) => format!("could not be read ({e})"),
            None => format!(
                "read {read:#x}, want {:#x} under mask {:#x}",
                a.want, a.mask
            ),
        };
        match a.severity {
            Severity::Warn => warnings.push(Warning::new(
                a.id.0,
                format!("{:#06x} {detail} — {}", a.reg, a.why),
            )),
            Severity::Fatal => {
                let mut r = blank_report(run, plan.id, steps, asserts, &edits);
                r.state = state;
                r.warnings = warnings;
                r.elapsed = t0.elapsed();
                r.started_at = started_at;
                r.plan_digest = r.compute_digest();
                let source = err.unwrap_or_else(|| {
                    FaceError::Io(std::io::Error::new(
                        std::io::ErrorKind::InvalidData,
                        format!("assert `{}` at {:#06x}: {detail} — {}", a.id, a.reg, a.why),
                    ))
                });
                return Err(BringUpFailure {
                    report: r,
                    failed_at: a.id.0,
                    source,
                });
            }
        }
    }

    // 5. §4 — answer the transmit question, including "I cannot".
    let (tx, refusal) = tx_proof(
        b,
        state.role,
        &run.proof,
        run.witness.as_ref(),
        &mut warnings,
    );
    if let Some(source) = refusal {
        let mut r = blank_report(run, plan.id, steps, asserts, &edits);
        r.state = state;
        r.warnings = warnings;
        r.tx = tx;
        r.elapsed = t0.elapsed();
        r.started_at = started_at;
        r.plan_digest = r.compute_digest();
        return Err(BringUpFailure {
            report: r,
            failed_at: "tx_proof",
            source,
        });
    }

    let mut report = blank_report(run, plan.id, steps, asserts, &edits);
    report.state = state;
    report.warnings = warnings;
    report.tx = tx;
    report.started_at = started_at;
    report.elapsed = t0.elapsed();
    report.plan_digest = report.compute_digest();
    report.emit();
    Ok((report, guards))
}

fn record<B: 'static>(s: &Step<B>, outcome: StepOutcomeRecord, us: u64) -> StepRecord {
    StepRecord {
        id: s.id,
        stage: s.stage,
        class: s.class,
        outcome,
        elapsed_us: us,
    }
}

/// **How many times a probe keys the transmitter.**
///
/// Small on purpose: a probe is the only thing in this module that puts energy on the air, and a
/// bring-up is not a throughput test. Eight is enough to separate a live counter from a stuck one
/// on the fleet's calibrated instrument (the 8733b's `0x2de0/0x2de2` pair MEASURED exactly +50
/// across 50 injects — one count per transmit request, so any count above one is decisive) and
/// small enough that the airtime is invisible beside the ladder's own milliseconds.
pub const TX_PROBE_COUNT: u16 = 8;

/// §4 / LAW 4 — is the transmit question answerable AS ASKED? Run before the first register write.
fn validate_proof(
    role: Role,
    proof: &ProofRequirement,
    has_witness: bool,
    instruments: &'static [TxInstrument],
) -> Result<(), RequestError> {
    if !role.transmits() {
        // Every requirement is legal on a receiver; none of them is taken. `None` exists FOR this.
        return Ok(());
    }
    match proof {
        // LAW 4. "I did not check" is how this stack shipped a 20 dB deficit and must not be
        // expressible on a radio that intends to transmit.
        ProofRequirement::None => Err(RequestError::ProofDeclinedWhileTransmitting),
        ProofRequirement::MacKeyedOrFail if instruments.is_empty() => {
            Err(RequestError::Unsatisfiable {
                what: "ProofRequirement::MacKeyedOrFail",
                because: "this part declares no TxInstrument, so question (A) is unanswerable \
                          on-chip. Ask for BestAvailable and read the reason, or prove (B) with \
                          WitnessOrFail.",
            })
        }
        ProofRequirement::WitnessOrFail { .. } if !has_witness => {
            Err(RequestError::Unsatisfiable {
                what: "ProofRequirement::WitnessOrFail",
                because: "no witness oracle was supplied (`PlanRun::with_witness`). Only a peer can \
                      mint TxProof::WitnessDecoded — a bring-up on a single host cannot prove \
                      radiation, and a design that claims otherwise is lying.",
            })
        }
        _ => Ok(()),
    }
}

/// §4: answer the transmit question, including "I cannot".
///
/// Returns the proof and, when the requirement was not met, the error that fails the bring-up.
/// **`Unprovable` is a success value**: a radio must not refuse to come up because its silicon is
/// blind. What is removed is the *silence*, and the ability to intend transmission while declining
/// to look.
///
/// ⚠ **Deviation from §4, recorded rather than silent.** The spec says a `Refuted` instrument is
/// "fatal except under `ProofRequirement::None`". Here it is fatal under
/// [`ProofRequirement::MacKeyedOrFail`] — where the caller asked for a hard gate — and **recorded
/// with a loud warning under [`ProofRequirement::BestAvailable`]**, the production default. The
/// reason is §1.5's own rule for the other readback mechanism: *Warn on introduction for every
/// part; promoted to Fatal per part only with a measurement.* No probe in this fleet has ever been
/// taken at bring-up, so nobody knows this instrument's false-negative rate, and a first
/// introduction that can refuse a working radio is the failure mode the contract exists to avoid.
/// §6.5's hardware test is where `TxProof != Refuted` is asserted and the promotion is earned.
fn tx_proof<B: BringUp>(
    b: &Arc<B>,
    role: Role,
    proof: &ProofRequirement,
    witness: Option<&WitnessOracle>,
    warnings: &mut Vec<Warning>,
) -> (TxProof, Option<FaceError>) {
    if !role.transmits() {
        return (TxProof::NotRequested, None);
    }
    let hard = matches!(proof, ProofRequirement::MacKeyedOrFail);

    // (B). Only a peer can answer it, so the oracle is asked and nothing here interprets a register.
    if let ProofRequirement::WitnessOrFail {
        witness: id,
        min_heard,
    } = proof
    {
        let Some(oracle) = witness else {
            // Unreachable — `validate_proof` refused this before the first rung. Named, not
            // unwrapped: an invariant asserted in two places is an invariant.
            return (
                TxProof::Unprovable {
                    instrument: None,
                    reason: "WitnessOrFail with no witness oracle reached the probe stage; \
                             validate_proof should have refused it before the first rung",
                },
                Some(FaceError::Io(std::io::Error::new(
                    std::io::ErrorKind::InvalidInput,
                    "WitnessOrFail without a witness oracle",
                ))),
            );
        };
        return match oracle() {
            Ok(r) if r.heard >= *min_heard => (
                TxProof::WitnessDecoded {
                    witness: id.clone(),
                    sent: r.sent,
                    heard: r.heard,
                    rssi_dbm: r.rssi_dbm,
                },
                None,
            ),
            Ok(r) => (
                TxProof::WitnessDecoded {
                    witness: id.clone(),
                    sent: r.sent,
                    heard: r.heard,
                    rssi_dbm: r.rssi_dbm,
                },
                Some(FaceError::Io(std::io::Error::new(
                    std::io::ErrorKind::InvalidData,
                    format!(
                        "witness {} decoded {}/{} probes, below the required {min_heard} — \
                         the radio came up and did not demonstrably radiate",
                        id.0, r.heard, r.sent
                    ),
                ))),
            ),
            Err(e) => (
                TxProof::Unprovable {
                    instrument: None,
                    reason: "the witness could not be asked, so (B) is unanswered — which is not \
                             the same as answered NO",
                },
                Some(e),
            ),
        };
    }

    // (A). The part's own instrument, taken by the part's own probe.
    let Some(instrument) = B::tx_instruments().first().copied() else {
        return (
            TxProof::Unprovable {
                instrument: None,
                // The part's own measured refusal if it has one, so the reason carries the
                // measurement rather than a generic shrug (§4: "`Unprovable`, reason quoted").
                reason: B::tx_unprovable_reason().unwrap_or(
                    "this part declares no MAC->BB transmit counter (`tx_instruments()` is \
                     empty), so question (A) is unanswerable on-chip. Prove with a witness.",
                ),
            },
            None,
        );
    };
    let Some(taken) = b.probe_tx(&instrument, TX_PROBE_COUNT) else {
        let unprovable = TxProof::Unprovable {
            instrument: Some(instrument),
            reason: "this part names an instrument but implements no `BringUp::probe_tx`, so no \
                     reading was taken. Ask the instrument directly, or use a witness.",
        };
        // Under MacKeyedOrFail this is a caller error `validate_proof` cannot see: the part
        // declared an instrument, so the request looked satisfiable.
        let refusal = hard.then(|| {
            FaceError::Io(std::io::Error::new(
                std::io::ErrorKind::Unsupported,
                format!(
                    "MacKeyedOrFail: {} is declared but this part implements no probe_tx",
                    instrument.name
                ),
            ))
        });
        return (unprovable, refusal);
    };
    match taken {
        Ok(p) if p.delta > 0 => (
            TxProof::MacKeyed {
                instrument,
                probes: p.probes,
                delta: p.delta,
                idle_control: p.idle_control,
            },
            None,
        ),
        Ok(p) => {
            let detail = format!(
                "{} did not move across {} probes (delta 0{}). The MAC never keyed the \
                 transmitter: the failure is upstream of the PHY — queue, descriptor or doorbell — \
                 not on the air.",
                instrument.name,
                p.probes,
                match p.idle_control {
                    Some(c) => format!(", idle control +{c}"),
                    None => String::new(),
                }
            );
            if !hard {
                warnings.push(Warning::new("tx_proof", format!("REFUTED — {detail}")));
            }
            let refusal = hard.then(|| {
                FaceError::Io(std::io::Error::new(
                    std::io::ErrorKind::InvalidData,
                    detail.clone(),
                ))
            });
            (TxProof::Refuted { instrument, detail }, refusal)
        }
        Err(e) => {
            warnings.push(Warning::new(
                "tx_proof",
                format!(
                    "the transmit probe could not be taken ({e}); question (A) is UNANSWERED, \
                     not answered NO"
                ),
            ));
            let unprovable = TxProof::Unprovable {
                instrument: Some(instrument),
                reason: "the probe was attempted and could not be completed; see the warning for \
                         the error. Unanswered is not the same as answered NO.",
            };
            (unprovable, hard.then_some(e))
        }
    }
}

fn blank_report(
    run: &PlanRun,
    plan: PlanId,
    steps: Vec<StepRecord>,
    asserts: Vec<AssertRecord>,
    edits: &PlanEdits,
) -> BringUpReport {
    let mut r = BringUpReport::hand_filled(run.part, plan, run.device.clone(), run.state.clone());
    r.provenance = edits.provenance();
    r.deviations = edits.edits().to_vec();
    r.steps = steps;
    r.asserts = asserts;
    r.tx = TxProof::Unprovable {
        instrument: None,
        reason: "the plan did not reach the transmit probe",
    };
    r
}

fn fail_before_first_rung(run: &PlanRun, plan: PlanId, e: PlanError) -> BringUpFailure {
    let mut r = blank_report(run, plan, Vec::new(), Vec::new(), &PlanEdits::canonical());
    r.plan_digest = r.compute_digest();
    BringUpFailure {
        report: r,
        failed_at: "plan::check",
        source: e.into(),
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Tests
// ─────────────────────────────────────────────────────────────────────────────
//
// These are the only consumers of this module today. They stand in for the per-part
// `tests/plan_shape.rs` of §6.3 until a driver declares a plan (M3).

#[cfg(test)]
mod tests {
    use super::*;
    use crate::Bandwidth;
    use crate::bringup::{AppliedPower, PowerRequest, RateState, WitnessId};
    use std::sync::Mutex;
    use std::sync::atomic::{AtomicU32, Ordering};

    // ── a backend with no hardware ───────────────────────────────────────────

    #[derive(Default)]
    struct Fake {
        /// Rung ids that should return an error.
        fail_at: Mutex<Vec<&'static str>>,
        /// Rung ids that actually executed, in order — the ground truth the report is checked
        /// against, so a report that *claims* a step ran is not self-certifying.
        ran: Mutex<Vec<&'static str>>,
        /// Stands in for `REG_TXPAUSE`: written by `lc_calibrate`, read back by the assert.
        txpause: AtomicU32,
    }

    impl Fake {
        fn new(fail_at: &[&'static str]) -> Arc<Self> {
            Arc::new(Self {
                fail_at: Mutex::new(fail_at.to_vec()),
                txpause: AtomicU32::new(0x3f),
                ..Default::default()
            })
        }
        fn enter(&self, id: &'static str) -> Result<(), FaceError> {
            self.ran.lock().unwrap().push(id);
            if self.fail_at.lock().unwrap().contains(&id) {
                return Err(FaceError::Io(std::io::Error::other(format!(
                    "{id} refused"
                ))));
            }
            Ok(())
        }
        fn ran(&self) -> Vec<&'static str> {
            self.ran.lock().unwrap().clone()
        }
    }

    fn s_power_on(b: &Arc<Fake>, _c: &mut Ctx<'_>) -> Result<StepOutcome, FaceError> {
        b.enter("power_on")?;
        Ok(StepOutcome::Done)
    }
    fn s_pwrinfo(b: &Arc<Fake>, c: &mut Ctx<'_>) -> Result<StepOutcome, FaceError> {
        b.enter("load_tx_power_info")?;
        let r = crate::bringup::PowerReference::FusedBase {
            base_index: 27,
            channel: c.state_ref().channel,
        };
        Ok(StepOutcome::Established(Fact::PowerReference(r)))
    }
    fn s_tssi(b: &Arc<Fake>, _c: &mut Ctx<'_>) -> Result<StepOutcome, FaceError> {
        b.enter("tssi_setup")?;
        Ok(StepOutcome::Done)
    }
    fn s_enable_tx(b: &Arc<Fake>, _c: &mut Ctx<'_>) -> Result<StepOutcome, FaceError> {
        b.enter("enable_tx")?;
        Ok(StepOutcome::Guard("PowerTracker", Box::new(Tracker(7))))
    }
    fn s_edcca(b: &Arc<Fake>, c: &mut Ctx<'_>) -> Result<StepOutcome, FaceError> {
        b.enter("edcca")?;
        // A step that branches internally reports which way AND leaves the fact in the regime, so
        // the report and the branch cannot disagree.
        c.state().warm = Some(true);
        c.warn("EDCCA left at whatever the last process wrote");
        Ok(StepOutcome::Branch("warm"))
    }
    fn s_lc_cal(b: &Arc<Fake>, _c: &mut Ctx<'_>) -> Result<StepOutcome, FaceError> {
        b.enter("lc_calibrate")?;
        b.txpause.store(0x00, Ordering::SeqCst);
        Ok(StepOutcome::Done)
    }

    #[derive(Debug, PartialEq)]
    struct Tracker(u8);

    const STEPS: &[Step<Fake>] = &[
        Step {
            id: StepId("power_on"),
            stage: Stage::PowerOn,
            class: StepClass::Required,
            why: "the part is in reset until the power sequence runs; every later write is a no-op",
            must_follow: &[],
            must_precede: &[],
            run: s_power_on,
        },
        Step {
            id: StepId("load_tx_power_info"),
            stage: Stage::Calibrate,
            class: StepClass::Required,
            why: "MEASURED 2026-09-03: without it set_tx_power writes the RAW TXAGC axis, \
                  a different power regime from the fused base. It decides what a later call MEANS.",
            must_follow: &[StepId("power_on")],
            must_precede: &[],
            run: s_pwrinfo,
        },
        Step {
            id: StepId("tssi_setup"),
            stage: Stage::Calibrate,
            class: StepClass::Required,
            why: "MEASURED: TSSI before enable_tx gives 19.6 dB of usable range; the other order \
                  gives 1.6 dB.",
            must_follow: &[],
            must_precede: &[StepId("enable_tx")],
            run: s_tssi,
        },
        Step {
            id: StepId("edcca"),
            stage: Stage::Posture,
            class: StepClass::BestEffort(Degradation::new(
                "the contention posture stays whatever the last process left",
                "delivery and range numbers; NOT throughput (MEASURED 2.5x swing by run order)",
            )),
            why: "EDCCA leaked across processes on the MT7610U and decided throughput by run \
                  order; pinning it at bring-up is the fix.",
            must_follow: &[],
            must_precede: &[],
            run: s_edcca,
        },
        Step {
            id: StepId("lc_calibrate"),
            stage: Stage::Calibrate,
            class: StepClass::Required,
            why: "vendor sequence; also the rung whose teardown 8307161 changed on 2026-08-31, \
                  which is why txpause_released is asserted rather than re-written.",
            must_follow: &[],
            must_precede: &[],
            run: s_lc_cal,
        },
        Step {
            id: StepId("enable_tx"),
            stage: Stage::TxEnable,
            class: StepClass::Required,
            why: "the transmit path is gated until this runs; the tracker it returns must outlive \
                  the ladder or the PA fades as it heats.",
            must_follow: &[StepId("tssi_setup")],
            must_precede: &[],
            run: s_enable_tx,
        },
    ];

    const PLAN: Plan<Fake> = Plan {
        id: PlanId {
            part: "FAKE",
            name: "txrx",
            ver: 1,
        },
        role: Role::TransmitAndReceive,
        steps: STEPS,
        excluded: &[(
            Stage::Firmware,
            "this part has no downloadable firmware; there is nothing to load and nothing to poll",
        )],
    };

    /// ★ **A malformed plan is a compile error, not a runtime one.** This is the call site the
    /// drivers copy in M3; it is why `check_or_panic` is a `const fn`.
    const _: () = PLAN.check_or_panic();

    static PLAN_TXRX: Plan<Fake> = PLAN;

    impl BringUp for Fake {
        fn plan(role: Role) -> Option<&'static Plan<Self>> {
            match role {
                Role::TransmitAndReceive => Some(&PLAN_TXRX),
                // A part that does not do a role says so. `None` is a named refusal.
                _ => None,
            }
        }
        fn asserts() -> &'static [Assert<Self>] {
            &[Assert {
                id: StepId("txpause_released"),
                reg: 0x0522,
                read: |b: &Fake| Ok(b.txpause.load(Ordering::SeqCst)),
                want: 0x00,
                mask: 0xff,
                why: "0x3f is aborted-IQK residue, 0xff aborted-LCK. MEASURED 0x00/0x00/0x00 over \
                      three bring-ups: structurally real, not firing today. Assert it; do not add \
                      a write8, which was A/B'd inert and reverted.",
                severity: Severity::Warn,
            }]
        }
    }

    fn state(role: Role) -> RadioState {
        RadioState {
            channel: 6,
            bw: Bandwidth::Bw20,
            format: "RawNdn(0x8624)",
            role,
            power: AppliedPower::no_actuator(PowerRequest::NoActuator),
            rate: RateState::unreported(),
            warm: None,
            contention: None,
            pump: crate::bringup::PumpPolicy::CallerOwns,
            facts: Vec::new(),
        }
    }

    fn run(role: Role) -> PlanRun {
        PlanRun::new(
            "FAKE",
            crate::bringup::DeviceAddress::Usb("1-3.2".into()),
            state(role),
        )
    }

    // ── the canonical path ───────────────────────────────────────────────────

    #[test]
    fn a_canonical_run_records_every_rung_and_hoists_every_fact() {
        let b = Fake::new(&[]);
        let (rep, mut guards) = b.bring_up(&run(Role::TransmitAndReceive)).unwrap();

        assert_eq!(b.ran().len(), PLAN_TXRX.steps.len());
        assert_eq!(rep.steps.len(), PLAN_TXRX.steps.len());
        assert!(matches!(rep.provenance, Provenance::Canonical));
        assert!(rep.deviations.is_empty());

        // LAW 6 — every Established(Fact) lands in RadioState::facts, done by the runner so a
        // driver cannot establish something and forget to report it.
        assert!(rep.state.facts.iter().any(|f| matches!(
            f,
            Fact::PowerReference(crate::bringup::PowerReference::FusedBase { base_index: 27, .. })
        )));

        // The guard the plan produced is on the handle, not dropped at the end of the ladder.
        assert_eq!(guards.names(), vec!["PowerTracker"]);
        assert_eq!(*guards.take::<Tracker>().unwrap(), Tracker(7));

        // A degradation a step chose to continue past is named, not silent.
        assert!(rep.warnings.iter().any(|w| w.at == "edcca"));
        assert_eq!(
            rep.state.warm,
            Some(true),
            "what a step wrote into Ctx::state is reported"
        );
        // §1.5 — the gate the ladder wrote was read back.
        assert_eq!(rep.asserts.len(), 1);
        assert!(rep.asserts[0].ok);
        // §4 — the transmit question gets an answer, including "I cannot".
        assert!(matches!(rep.tx, TxProof::Unprovable { .. }));
    }

    #[test]
    fn a_role_the_part_does_not_declare_is_refused_by_name() {
        let b = Fake::new(&[]);
        let e = b.bring_up(&run(Role::ReceiveOnly)).unwrap_err();
        assert_eq!(e.failed_at, "plan::check");
        assert!(e.source.to_string().contains("declares no plan"));
        assert!(
            b.ran().is_empty(),
            "no rung may run for a plan that does not exist"
        );
    }

    // ── ordering constraints, rejected at plan construction ──────────────────

    fn bad_plan(steps: &'static [Step<Fake>]) -> Plan<Fake> {
        Plan {
            id: PlanId {
                part: "FAKE",
                name: "bad",
                ver: 1,
            },
            role: Role::TransmitAndReceive,
            steps,
            excluded: &[],
        }
    }

    const OUT_OF_ORDER: &[Step<Fake>] = &[
        Step {
            id: StepId("enable_tx"),
            stage: Stage::TxEnable,
            class: StepClass::Required,
            why: "non-empty",
            must_follow: &[],
            must_precede: &[],
            run: s_enable_tx,
        },
        Step {
            id: StepId("tssi_setup"),
            stage: Stage::Calibrate,
            class: StepClass::Required,
            why: "non-empty",
            must_follow: &[],
            must_precede: &[StepId("enable_tx")],
            run: s_tssi,
        },
    ];

    const UNKNOWN_CONSTRAINT: &[Step<Fake>] = &[Step {
        id: StepId("tssi_setup"),
        stage: Stage::Calibrate,
        class: StepClass::Required,
        why: "non-empty",
        must_precede: &[StepId("enbale_tx")], // typo, as a rename would look
        must_follow: &[],
        run: s_tssi,
    }];

    const DUPLICATE: &[Step<Fake>] = &[
        Step {
            id: StepId("tssi_setup"),
            stage: Stage::Calibrate,
            class: StepClass::Required,
            why: "non-empty",
            must_follow: &[],
            must_precede: &[],
            run: s_tssi,
        },
        Step {
            id: StepId("tssi_setup"),
            stage: Stage::Calibrate,
            class: StepClass::Required,
            why: "non-empty",
            must_follow: &[],
            must_precede: &[],
            run: s_tssi,
        },
    ];

    const EMPTY_WHY: &[Step<Fake>] = &[Step {
        id: StepId("tssi_setup"),
        stage: Stage::Calibrate,
        class: StepClass::Required,
        why: "",
        must_follow: &[],
        must_precede: &[],
        run: s_tssi,
    }];

    const BLANK_DEGRADATION: &[Step<Fake>] = &[Step {
        id: StepId("edcca"),
        stage: Stage::Posture,
        class: StepClass::BestEffort(Degradation::new("", "")),
        why: "non-empty",
        must_follow: &[],
        must_precede: &[],
        run: s_edcca,
    }];

    #[test]
    fn an_ordering_violation_is_rejected_at_plan_construction() {
        // MEASURED: TSSI before enable_tx gives 19.6 dB of usable range, the other order 1.6 dB.
        // That ordering was prose in a doc comment. It is now a constraint, and here it is broken.
        let p = bad_plan(OUT_OF_ORDER);
        let e = p.check().unwrap_err();
        assert!(
            matches!(&e, PlanError::OutOfOrder { step, other, kind, .. }
                     if step.0 == "tssi_setup" && other.0 == "enable_tx" && *kind == "must_precede"),
            "{e:?}"
        );
        assert!(e.to_string().contains("19.6 dB"));
    }

    #[test]
    fn a_constraint_naming_a_step_that_does_not_exist_is_rejected() {
        // The failure that did NOT happen when 8307161 renamed a rung under sixteen ladders.
        let e = bad_plan(UNKNOWN_CONSTRAINT).check().unwrap_err();
        assert!(
            matches!(&e, PlanError::UnknownConstraint { names, .. } if *names == "enbale_tx"),
            "{e:?}"
        );
    }

    #[test]
    fn duplicate_ids_are_rejected_because_the_digest_and_skip_key_on_them() {
        assert!(matches!(
            bad_plan(DUPLICATE).check().unwrap_err(),
            PlanError::DuplicateStep { .. }
        ));
    }

    #[test]
    fn every_why_is_non_empty() {
        // The shape rule itself …
        assert!(matches!(
            bad_plan(EMPTY_WHY).check().unwrap_err(),
            PlanError::EmptyWhy { .. }
        ));
        // … and the rule applied to the plans this module actually declares. `tests/plan_shape.rs`
        // (§6.3) does this for every part once a driver declares one.
        PLAN_TXRX
            .check()
            .expect("the module's own plan must pass its own gate");
        for s in PLAN_TXRX.steps {
            assert!(!s.why.trim().is_empty(), "{} has an empty why", s.id);
        }
        for a in <Fake as BringUp>::asserts() {
            assert!(!a.why.trim().is_empty(), "{} has an empty why", a.id);
        }
        for (_, why) in PLAN_TXRX.excluded {
            assert!(
                !why.trim().is_empty(),
                "a written exclusion must be written"
            );
        }
    }

    #[test]
    fn a_best_effort_rung_must_name_what_is_lost() {
        // A `let _ = step` with no declared consequence is not expressible: `BestEffort` carries a
        // `Degradation`, and a blank one does not pass the gate.
        assert!(matches!(
            bad_plan(BLANK_DEGRADATION).check().unwrap_err(),
            PlanError::EmptyDegradation { .. }
        ));
    }

    #[test]
    fn a_malformed_plan_stops_the_runner_before_the_first_register_write() {
        let b = Fake::new(&[]);
        let p: &'static Plan<Fake> = Box::leak(Box::new(bad_plan(OUT_OF_ORDER)));
        let e = run_plan(&b, p, &run(Role::TransmitAndReceive)).unwrap_err();
        assert_eq!(e.failed_at, "plan::check");
        assert!(
            b.ran().is_empty(),
            "a plan that does not hold together may not touch the radio"
        );
        assert!(e.report.steps.is_empty());
    }

    #[test]
    fn check_and_const_check_agree() {
        // `check_or_panic` is a second implementation of the same rules (a `const fn` cannot build
        // the message that names the rung). Drift between them would silently weaken the
        // compile-time gate, so they are held together here.
        let bad: [&'static [Step<Fake>]; 5] = [
            OUT_OF_ORDER,
            UNKNOWN_CONSTRAINT,
            DUPLICATE,
            EMPTY_WHY,
            BLANK_DEGRADATION,
        ];
        let prev = std::panic::take_hook();
        std::panic::set_hook(Box::new(|_| {}));
        for steps in bad {
            let p = bad_plan(steps);
            assert!(p.check().is_err());
            let caught =
                std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| p.check_or_panic()));
            assert!(caught.is_err(), "const gate missed what check() caught");
        }
        std::panic::set_hook(prev);
        assert!(PLAN_TXRX.check().is_ok());
        PLAN_TXRX.check_or_panic();
    }

    // ── a Required failure carries the partial report ────────────────────────

    #[test]
    fn a_required_failure_returns_the_partial_report() {
        let b = Fake::new(&["lc_calibrate"]);
        let e = b.bring_up(&run(Role::TransmitAndReceive)).unwrap_err();

        // ★ Which rung, in which stage, with everything established up to it — instead of a bare
        // FaceError and a day of bisection.
        assert_eq!(e.failed_at, "lc_calibrate");
        assert_eq!(
            e.report.steps.iter().map(|s| s.id.0).collect::<Vec<_>>(),
            vec![
                "power_on",
                "load_tx_power_info",
                "tssi_setup",
                "edcca",
                "lc_calibrate"
            ],
            "the partial report stops where the ladder stopped, and says so"
        );
        assert!(matches!(
            e.report.steps.last().unwrap().outcome,
            StepOutcomeRecord::Failed(_)
        ));
        // Everything established before the failure survives into the report.
        assert!(e.report.state.facts.iter().any(|f| matches!(
            f,
            Fact::PowerReference(crate::bringup::PowerReference::FusedBase { .. })
        )));
        assert!(e.report.warnings.iter().any(|w| w.at == "edcca"));
        assert!(e.report.plan_digest != 0);
        // The rung after the failure never ran.
        assert!(!b.ran().contains(&"enable_tx"));
        assert!(e.to_string().contains("lc_calibrate"));
    }

    #[test]
    fn a_best_effort_failure_warns_and_the_ladder_continues() {
        let b = Fake::new(&["edcca"]);
        let (rep, _g) = b.bring_up(&run(Role::TransmitAndReceive)).unwrap();
        assert!(
            b.ran().contains(&"enable_tx"),
            "BestEffort must not abort the ladder"
        );
        let w = rep
            .warnings
            .iter()
            .find(|w| w.at == "edcca")
            .expect("a BestEffort failure names its consequence");
        assert!(w.degrades.contains("contention posture"));
        assert!(w.degrades.contains("still valid for"));
    }

    // ── deviation: in the report, and in the digest ──────────────────────────

    #[test]
    fn a_deviation_lands_in_the_report_and_changes_the_digest() {
        let canonical = Fake::new(&[])
            .bring_up(&run(Role::TransmitAndReceive))
            .unwrap()
            .0;

        // The exact operation that found the 2026-09-03 defect.
        let dev = Deviation::new("does the fused base cost us the link?").skip(
            "load_tx_power_info",
            "bisect arm 2 of 3: the other two arms both skip calibration and both transmit",
        );
        let deviated = Fake::new(&[])
            .bring_up(&run(Role::TransmitAndReceive).with_deviation(dev))
            .unwrap()
            .0;

        // 1. It is in the report, edit by edit, with its own reason.
        assert_eq!(deviated.deviations.len(), 1);
        assert_eq!(
            deviated.deviations[0].op,
            DeviationOp::Skip("load_tx_power_info".into())
        );
        assert!(deviated.deviations[0].why.contains("bisect arm 2"));
        assert!(matches!(
            &deviated.provenance,
            Provenance::Deviated { question, .. } if question.contains("fused base")
        ));

        // 2. It changed the digest, so a bench number and a production number cannot be compared
        //    by accident.
        assert_ne!(canonical.plan_digest, deviated.plan_digest);

        // 3. It is visible without knowing the bug exists.
        assert!(deviated.render().contains("DEVIATED"));
        assert!(deviated.render().contains("bisect arm 2"));
        let d = canonical.diff(&deviated);
        assert!(d.iter().any(|x| x.field == "deviations"));
        assert!(d.iter().any(|x| x.field == "steps"));

        // 4. The rung really did not run, and the record says why.
        let rec = deviated
            .steps
            .iter()
            .find(|s| s.id.0 == "load_tx_power_info")
            .unwrap();
        assert!(
            matches!(&rec.outcome, StepOutcomeRecord::Skipped(w) if w.contains("bisect arm 2"))
        );
    }

    #[test]
    fn the_stated_reason_is_part_of_the_digest() {
        let one = Fake::new(&[])
            .bring_up(&run(Role::TransmitAndReceive).with_deviation(
                Deviation::new("q").skip("edcca", "checking whether EDCCA is what silences TX"),
            ))
            .unwrap()
            .0;
        let two = Fake::new(&[])
            .bring_up(
                &run(Role::TransmitAndReceive)
                    .with_deviation(Deviation::new("q").skip("edcca", "saving 40 ms at bring-up")),
            )
            .unwrap()
            .0;
        // Same edit, different question being asked: different runs, different buckets.
        assert_ne!(one.plan_digest, two.plan_digest);
    }

    #[test]
    fn a_deviation_naming_a_step_that_does_not_exist_is_refused() {
        let b = Fake::new(&[]);
        let e = b
            .bring_up(&run(Role::TransmitAndReceive).with_deviation(
                Deviation::new("q").skip("load_tx_powr_info", "typo, as a rename would look"),
            ))
            .unwrap_err();
        assert_eq!(e.failed_at, "plan::resolve");
        assert!(e.source.to_string().contains("no such step"));
        assert!(b.ran().is_empty());
    }

    #[test]
    fn an_edit_with_no_stated_reason_is_refused() {
        let ok = PLAN_TXRX
            .resolve(Some(&Deviation::new("q").skip("edcca", "why")))
            .unwrap();
        assert_eq!(ok.question(), "q");
        assert!(!ok.is_canonical());

        let e = PLAN_TXRX
            .resolve(Some(&Deviation::new("q").skip("edcca", "  ")))
            .unwrap_err();
        assert!(matches!(e, RequestError::MissingWhy { .. }), "{e:?}");
        let e = PLAN_TXRX
            .resolve(Some(&Deviation::new("   ").skip("edcca", "why")))
            .unwrap_err();
        assert!(matches!(e, RequestError::MissingWhy { .. }), "{e:?}");
    }

    #[test]
    fn stop_after_cuts_the_ladder_and_records_the_rest_as_skipped() {
        let b = Fake::new(&[]);
        let (rep, guards) = b
            .bring_up(&run(Role::TransmitAndReceive).with_deviation(
                Deviation::new("how far can we get before the transmit path is armed?").stop_after(
                    Stage::Posture,
                    "everything after Posture is the TX arm we are isolating",
                ),
            ))
            .unwrap();
        // Stage is a label, not a monotone gate: the cut is the LAST rung at or before Posture.
        assert!(!b.ran().contains(&"enable_tx"));
        assert!(guards.is_empty());
        assert!(matches!(
            &rep.steps.last().unwrap().outcome,
            StepOutcomeRecord::Skipped(w) if w.contains("stop-after")
        ));
    }

    #[test]
    fn the_env_form_parses_and_refuses_a_deviation_with_no_question() {
        let d = Deviation::parse("does the fused base cost us the link?|skip:load_tx_power_info")
            .unwrap();
        assert_eq!(d.edits().len(), 1);
        assert!(
            d.edits()[0].why.contains("fused base"),
            "an edit inherits the question"
        );

        let d = Deviation::parse("q|skip:edcca=arm 2,stop-after:calibrate").unwrap();
        assert_eq!(d.edits().len(), 2);
        assert_eq!(d.edits()[0].why, "arm 2");
        assert_eq!(d.edits()[1].op, DeviationOp::StopAfter(Stage::Calibrate));

        // Rule 1: a deviation with no stated reason does not parse.
        assert!(matches!(
            Deviation::parse("skip:edcca"),
            Err(RequestError::MissingWhy { .. })
        ));
        assert!(matches!(
            Deviation::parse("q|nudge:edcca"),
            Err(RequestError::UnknownStep(_))
        ));
        // A stage this plan does not label is refused, not silently a no-cut: a `--stop-after`
        // that quietly runs the whole ladder is the `ndn_env` `Unrecognised` lesson again.
        let e = PLAN_TXRX
            .resolve(Some(&Deviation::parse("q|stop-after:firmware").unwrap()))
            .unwrap_err();
        assert!(e.to_string().contains("no rung"), "{e}");
    }

    // ── §1.5 asserts ─────────────────────────────────────────────────────────

    #[test]
    fn a_failed_readback_is_recorded_and_warned_not_swallowed() {
        // Skip the rung that clears the gate; the assert must notice, at Warn.
        let b = Fake::new(&[]);
        let (rep, _g) = b
            .bring_up(
                &run(Role::TransmitAndReceive).with_deviation(
                    Deviation::new(
                        "does the ladder still leave TXPAUSE clear if lc_calibrate does not run?",
                    )
                    .skip("lc_calibrate", "isolating which rung clears the gate"),
                ),
            )
            .unwrap();
        assert_eq!(rep.asserts.len(), 1);
        assert!(!rep.asserts[0].ok);
        assert_eq!(rep.asserts[0].read, 0x3f);
        assert!(rep.warnings.iter().any(|w| w.at == "txpause_released"));
        // Warn on introduction: it reports, it does not refuse the radio.
        assert!(rep.render().contains("MISMATCH"));
    }

    // A second part, whose one assert has been promoted to Fatal by measurement.
    struct FakeFatal;
    static PLAN_FATAL: Plan<FakeFatal> = Plan {
        id: PlanId {
            part: "FAKE2",
            name: "rx",
            ver: 1,
        },
        role: Role::ReceiveOnly,
        steps: &[Step {
            id: StepId("mac_init"),
            stage: Stage::MacInit,
            class: StepClass::Required,
            why: "nothing receives until the MAC is configured",
            must_follow: &[],
            must_precede: &[],
            run: |_b: &Arc<FakeFatal>, _c: &mut Ctx<'_>| Ok(StepOutcome::Done),
        }],
        excluded: &[],
    };

    impl BringUp for FakeFatal {
        fn plan(_role: Role) -> Option<&'static Plan<Self>> {
            Some(&PLAN_FATAL)
        }
        fn asserts() -> &'static [Assert<Self>] {
            &[Assert {
                id: StepId("mac_rx_enabled"),
                reg: 0x0100,
                read: |_b: &FakeFatal| Ok(0x00),
                want: 0x3c,
                mask: 0xff,
                why: "promoted to Fatal by measurement: with MACRXEN clear this part receives \
                      nothing and every number taken through it is zero",
                severity: Severity::Fatal,
            }]
        }
    }

    #[test]
    fn a_fatal_readback_fails_the_bring_up_with_the_partial_report() {
        let b = Arc::new(FakeFatal);
        let mut r = run(Role::ReceiveOnly);
        r.part = "FAKE2";
        let e = b.bring_up(&r).unwrap_err();
        assert_eq!(e.failed_at, "mac_rx_enabled");
        // It got all the way through the ladder, and the report says so.
        assert_eq!(e.report.steps.len(), 1);
        assert!(!e.report.asserts[0].ok);
        assert!(e.source.to_string().contains("0x0100"));
    }

    // ── §4, the transmit question ────────────────────────────────────────────
    //
    // A part that intends to transmit may not decline to look; a part whose silicon is blind says
    // so and still comes up; and only a peer can mint (B).

    /// A part that CAN answer (A): one instrument, one probe, a settable counter movement.
    struct FakeProbe {
        delta: AtomicU32,
        probed: AtomicU32,
    }

    impl FakeProbe {
        fn new(delta: u32) -> Arc<Self> {
            Arc::new(Self {
                delta: AtomicU32::new(delta),
                probed: AtomicU32::new(0),
            })
        }
    }

    fn s_probe_enable(_b: &Arc<FakeProbe>, _c: &mut Ctx<'_>) -> Result<StepOutcome, FaceError> {
        Ok(StepOutcome::Done)
    }

    static PLAN_PROBE: Plan<FakeProbe> = Plan {
        id: PlanId {
            part: "FAKEPROBE",
            name: "tx",
            ver: 1,
        },
        role: Role::TransmitAndReceive,
        steps: &[Step {
            id: StepId("enable_tx"),
            stage: Stage::TxEnable,
            class: StepClass::Required,
            why: "the transmit path is gated until this runs",
            must_follow: &[],
            must_precede: &[],
            run: s_probe_enable,
        }],
        excluded: &[],
    };

    const INSTRUMENT: TxInstrument = TxInstrument {
        name: "fake_counter 0x2de0/0x2de2",
        cost_us: 400,
    };

    impl BringUp for FakeProbe {
        fn plan(role: Role) -> Option<&'static Plan<Self>> {
            (role == Role::TransmitAndReceive).then_some(&PLAN_PROBE)
        }
        fn tx_instruments() -> &'static [TxInstrument] {
            &[INSTRUMENT]
        }
        fn probe_tx(
            self: &Arc<Self>,
            _instrument: &TxInstrument,
            probes: u16,
        ) -> Option<Result<TxProbe, FaceError>> {
            self.probed.fetch_add(u32::from(probes), Ordering::SeqCst);
            Some(Ok(TxProbe {
                probes,
                delta: self.delta.load(Ordering::SeqCst) as u16,
                idle_control: Some(0),
            }))
        }
    }

    fn probe_run(proof: ProofRequirement) -> PlanRun {
        PlanRun::new(
            "FAKEPROBE",
            crate::bringup::DeviceAddress::Usb("1-3.2".into()),
            state(Role::TransmitAndReceive),
        )
        .with_proof(proof)
    }

    #[test]
    fn a_transmitting_role_may_not_decline_to_look() {
        // LAW 4. This is the shape in which the stack shipped a ~20 dB deficit: intending to
        // transmit while declining to check.
        let b = FakeProbe::new(8);
        let e = b
            .bring_up(&probe_run(ProofRequirement::None))
            .expect_err("ProofRequirement::None is illegal on a transmitting role");
        assert_eq!(e.failed_at, "proof::validate");
        assert!(
            e.report.steps.is_empty(),
            "refused before the first register write, not after the ladder"
        );
        assert_eq!(
            b.probed.load(Ordering::SeqCst),
            0,
            "a refused request never keys the transmitter"
        );

        // ...and it is exactly what `Role::ReceiveOnly` exists to spell.
        let mut rx = probe_run(ProofRequirement::None);
        rx.state.role = Role::ReceiveOnly;
        assert!(
            super::validate_proof(
                Role::ReceiveOnly,
                &ProofRequirement::None,
                false,
                &[INSTRUMENT]
            )
            .is_ok()
        );
    }

    #[test]
    fn an_instrument_that_moves_is_a_mac_keyed_proof() {
        let b = FakeProbe::new(8);
        let (rep, _g) = b
            .bring_up(&probe_run(ProofRequirement::MacKeyedOrFail))
            .unwrap();
        match rep.tx {
            TxProof::MacKeyed {
                instrument,
                probes,
                delta,
                idle_control,
            } => {
                assert_eq!(instrument.name, INSTRUMENT.name);
                assert_eq!(probes, super::TX_PROBE_COUNT);
                assert_eq!(delta, 8);
                assert_eq!(idle_control, Some(0));
            }
            other => panic!("expected MacKeyed, got {other:?}"),
        }
        assert_eq!(
            b.probed.load(Ordering::SeqCst),
            u32::from(super::TX_PROBE_COUNT),
            "the probe actually ran — a declared instrument nobody calls is this codebase's \
             characteristic defect"
        );
    }

    #[test]
    fn a_refuted_instrument_is_recorded_by_default_and_fatal_only_when_asked() {
        // BestAvailable (the production default): recorded loudly, does NOT refuse the radio.
        // ⚠ This is the recorded deviation from §4 — see `tx_proof`'s doc. §1.5's rule is that a
        // readback is introduced at Warn and promoted per part with a measurement.
        let b = FakeProbe::new(0);
        let (rep, _g) = b
            .bring_up(&probe_run(ProofRequirement::BestAvailable))
            .unwrap();
        assert!(matches!(rep.tx, TxProof::Refuted { .. }));
        assert!(
            rep.warnings.iter().any(|w| w.at == "tx_proof"),
            "a refuted transmitter is never silent"
        );

        // MacKeyedOrFail: the caller asked for a hard gate and gets one, with the partial report.
        let b = FakeProbe::new(0);
        let e = b
            .bring_up(&probe_run(ProofRequirement::MacKeyedOrFail))
            .expect_err("a counter that did not move must fail a MacKeyedOrFail bring-up");
        assert_eq!(e.failed_at, "tx_proof");
        assert!(matches!(e.report.tx, TxProof::Refuted { .. }));
        assert_eq!(
            e.report.steps.len(),
            1,
            "the partial report still says how far it got"
        );
        assert!(e.source.to_string().contains("upstream of the PHY"));
    }

    #[test]
    fn a_proof_this_part_cannot_take_is_named_before_the_first_rung() {
        // `Fake` declares no instrument. Asking it for MacKeyedOrFail is unsatisfiable, and the
        // ndn_env `Unrecognised` lesson applies: a request that quietly does nothing is worse than
        // a refused one.
        let b = Fake::new(&[]);
        let e = b
            .bring_up(&run(Role::TransmitAndReceive).with_proof(ProofRequirement::MacKeyedOrFail))
            .expect_err("MacKeyedOrFail on a blind part is unsatisfiable");
        assert_eq!(e.failed_at, "proof::validate");
        assert!(e.source.to_string().contains("MacKeyedOrFail"));
        assert!(
            b.ran().is_empty(),
            "nothing runs on an unsatisfiable request"
        );

        // BestAvailable on the same blind part SUCCEEDS, saying so. A radio must not refuse to
        // come up because its silicon cannot watch itself.
        let b = Fake::new(&[]);
        let (rep, _g) = b.bring_up(&run(Role::TransmitAndReceive)).unwrap();
        assert!(matches!(
            rep.tx,
            TxProof::Unprovable {
                instrument: None,
                ..
            }
        ));
    }

    #[test]
    fn only_a_peer_can_mint_a_witness_proof() {
        let witness = WitnessId("ar9271-bench".into());

        // No oracle: named, before the first rung.
        let b = FakeProbe::new(8);
        let e = b
            .bring_up(&probe_run(ProofRequirement::WitnessOrFail {
                witness: witness.clone(),
                min_heard: 1,
            }))
            .expect_err("WitnessOrFail with no witness is unsatisfiable");
        assert_eq!(e.failed_at, "proof::validate");
        assert!(
            e.source.to_string().contains("only a peer")
                || e.source.to_string().contains("Only a peer")
        );

        // The peer heard us: (B), the only real answer, and the runner did not invent it.
        let b = FakeProbe::new(8);
        let heard: WitnessOracle = Arc::new(|| {
            Ok(WitnessReport {
                sent: 50,
                heard: 49,
                rssi_dbm: Some(-71),
            })
        });
        let (rep, _g) = b
            .bring_up(
                &probe_run(ProofRequirement::WitnessOrFail {
                    witness: witness.clone(),
                    min_heard: 40,
                })
                .with_witness(heard),
            )
            .unwrap();
        assert_eq!(
            rep.tx,
            TxProof::WitnessDecoded {
                witness: witness.clone(),
                sent: 50,
                heard: 49,
                rssi_dbm: Some(-71),
            }
        );
        assert_eq!(
            b.probed.load(Ordering::SeqCst),
            0,
            "(B) is not (A): asking the peer does not also take the on-chip reading"
        );

        // The peer did not: the bring-up fails and the report carries what the peer actually said.
        let b = FakeProbe::new(8);
        let deaf: WitnessOracle = Arc::new(|| {
            Ok(WitnessReport {
                sent: 50,
                heard: 0,
                rssi_dbm: None,
            })
        });
        let e = b
            .bring_up(
                &probe_run(ProofRequirement::WitnessOrFail {
                    witness: witness.clone(),
                    min_heard: 40,
                })
                .with_witness(deaf),
            )
            .expect_err("a witness that heard nothing must fail a WitnessOrFail bring-up");
        assert_eq!(e.failed_at, "tx_proof");
        assert!(matches!(
            e.report.tx,
            TxProof::WitnessDecoded { heard: 0, .. }
        ));
    }

    #[test]
    fn a_receive_only_role_is_not_asked_to_prove_transmission() {
        let b = Arc::new(FakeFatal);
        // The proof question is answered from the role, so `NotRequested` and "we did not look"
        // are different states. (This part's Fatal assert is what fails; the proof is computed
        // only on the success path, so check the function directly.)
        let mut warnings = Vec::new();
        assert!(matches!(
            super::tx_proof(
                &b,
                Role::ReceiveOnly,
                &ProofRequirement::BestAvailable,
                None,
                &mut warnings
            ),
            (TxProof::NotRequested, None)
        ));
        assert!(warnings.is_empty());

        // A part with no instrument, asked for the best available, says what it cannot do.
        let blind = Fake::new(&[]);
        assert!(matches!(
            super::tx_proof(
                &blind,
                Role::TransmitOnly,
                &ProofRequirement::BestAvailable,
                None,
                &mut warnings
            ),
            (
                TxProof::Unprovable {
                    instrument: None,
                    ..
                },
                None
            )
        ));
    }
}
