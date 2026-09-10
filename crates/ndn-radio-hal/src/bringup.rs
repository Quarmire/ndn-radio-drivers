//! **The bring-up contract** — `docs/bringup-contract.md`, M0 + the M1/M2 surface.
//!
//! Companion measurement: `docs/bringup-root-cause-2026-09-03.md`.
//!
//! ## What forced this module
//!
//! `Rtl8812auBackend::set_tx_power(idx)` meant **two different physical powers**, decided by
//! whether `load_tx_power_info()` had run three calls earlier:
//!
//! * calibration loaded → `index_base(path, rate, ch) + (idx − 63)` ≈ **27** on the ch6 adapter;
//! * not loaded → it fell through to `set_tx_power_raw(idx)` → a flat **63** on ten registers.
//!
//! Same call, same argument, same `Ok(())`, **two different power regimes**. ⚠ The SIZE of the
//! gap is channel-dependent and UNVERIFIED — fused base 27 on ch6, 44 on ch149; MEASURED
//! 2026-09-04 at ch149 the Ceiling-vs-Raw difference is ~0 dB. The figures below came from a
//! KERNEL witness later shown to be broken:
//! raw 63 → 2301 frames at −85.6 dBm; raw 55 → **0**. `nav_probe` never loaded calibration (ran
//! hot, "worked"); `bring_up_monitor` did (ran at the fused base, "did not work"). The node binary
//! goes through the calibrated path, so **the shipped node and every bench example were not the
//! same transmitter.**
//!
//! ## What this module does NOT conclude
//!
//! 1. that ≈ 27 is wrong. It is probably the correct regulatory answer. **The default stays
//!    calibrated**, and nothing here runs raw by default.
//! 2. that the scale should be renumbered. Setting `RadioCapability::max_tx_power = 27` would make
//!    `RadioPolicy::decide_power` (which computes `max − backoff`) emit ~27, which the driver
//!    renders as `base + (27 − 63)` → `clamp(0)` → a silent near-zero radio: the same defect
//!    reintroduced by the cure. The physical point is reported in
//!    [`PowerReference::FusedBase`], which is a *fact*, not a renumbering.
//! 3. that a report catches bugs. It does not — see the contract's §7. Visibility is not
//!    detection. Four things here are enforcement: the deleted fallthrough, [`RfAuthority`]'s
//!    unconstructibility from library code, the non-optional report on `OpenRadio`, and (later)
//!    the asserts + TX probe, which read the hardware back. Judge the rest as documentation with
//!    a better type.
//!
//! ## Scope
//!
//! **M0 + M1 + M2** (the power vocabulary, the knob signature, the hand-filled report) live here;
//! **§1.4 the plan, §1.5 the asserts, §1.6 the deviation, and the runner** live in [`plan`](crate::bringup::plan) and
//! are re-exported from this module.
//!
//! Every part now runs a [`Plan`](crate::bringup::plan::Plan) (M3-M7), and §1.1's `BringUpRequest` — the one place
//! allowed to read the environment — exists as of M8.
//!
//! ⚠ It lives in `ndn-radio-drivers`, not here, and the reason is written out on it: its
//! `PartOpts` names driver-owned types (the AR9271 gain table and cal policy, the RTL8821CU plan
//! variant) and this crate must not depend on the driver crate. [`PlanRun`](crate::bringup::plan::PlanRun) stays
//! the runner's input and is that request's projection.
//!
//! Types marked **UNWIRED** have no consumer and are named as such rather than left to be
//! discovered — this codebase's characteristic failure is a capability nothing calls
//! (`with_wide_bloom`: one definition, zero call sites).

use std::time::{Duration, SystemTime};

use crate::{Bandwidth, RadioCapability};

/// **§1.4 the plan, §1.5 the asserts, §1.6 the deviation, and the runner.** Split into its own
/// file because it is the half of the contract that *executes* rather than describes; everything
/// in it is re-exported here, so the spec's `bringup::Step` / `bringup::run_plan` spellings hold.
pub mod plan;

pub use plan::{
    Assert, BringUp, Ctx, Degradation, Deviation, Guards, Plan, PlanEdit, PlanEdits, PlanError,
    PlanRun, Step, StepId, StepOutcome, TX_PROBE_COUNT, WitnessOracle, WitnessReport, run_plan,
};

// ─────────────────────────────────────────────────────────────────────────────
// §1.2 — Power
// ─────────────────────────────────────────────────────────────────────────────

/// **What the caller wants, in words that have exactly one physical meaning per part.**
///
/// This is the type that abolishes `set_tx_power(idx: u32)`. A bare index with no frame around it
/// is the thing being removed: it could not say whether it meant the regulatory scale or the raw
/// chip scale, and on the RTL8812AU it silently meant both.
///
/// ⚠ **Not `Copy`**, although the contract's §1.2 sketch derives it: [`Raw`](Self::Raw) carries an
/// [`RfAuthority`], which owns two `String`s (who granted the deviation, and why). Keeping the
/// operator's own words in the value — so they can be printed verbatim in every report that used
/// it — is worth more than `Copy`. Deviation from the spec sketch, recorded here rather than
/// silently.
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum PowerRequest {
    /// "As loud as this part will legally go." The top of the part's own declared scale:
    /// [`RadioCapability::max_tx_power`], or the top of [`RadioCapability::tx_power_dbm`]. On a
    /// fused part this **is** the regulatory base — those are the same point, and the API does not
    /// pretend otherwise.
    ///
    /// Carries the part's rate-group policy for the same reason [`Index`](Self::Index) does: on
    /// the 8812au this request writes the calibrated per-rate ladder.
    Ceiling(RateGroupPolicy),
    /// An index on the **declared** scale — i.e. what `RadioPolicy::decide_power` returns. Clamped
    /// into `[min_tx_power, max_tx_power]`; the clamp is REPORTED ([`AppliedPower::clamped`]).
    ///
    /// ★ The second field is [`RateGroupPolicy`], and it is here because of **LAW 3: a knob may
    /// not branch on the environment.** `Rtl8812auBackend::set_tx_power` used to read
    /// `NDN_AU_TXAGC12` from inside itself and silently turn 10 register writes into 24. That is
    /// the same hidden-state disease as `load_tx_power_info`, one level down. Use
    /// [`PowerRequest::index`] for the measured default.
    Index(u8, RateGroupPolicy),
    /// Absolute dBm. Requires [`RadioCapability::tx_power_dbm`] to be `Some` on the part, else the
    /// knob refuses by name rather than approximating.
    Dbm(i8),
    /// ⚠ **Off the regulatory scale.** Raw chip TXAGC, calibration bypassed; may exceed licensed
    /// EIRP. Not constructible without an [`RfAuthority`], which no library code can mint — see
    /// [`RfAuthority::from_env`].
    Raw { idx: u8, authority: RfAuthority },
    /// The part has no power actuator (`RadioCapability::power_actuated == false`: MT7612U,
    /// MT7921AU). Explicit, so "we did not set power" and "this part has no power knob" are
    /// different states rather than the same `Ok(())`.
    NoActuator,
}

impl PowerRequest {
    /// [`Ceiling`](Self::Ceiling) with the MEASURED rate-group policy. The production spelling.
    pub const fn ceiling() -> Self {
        Self::Ceiling(RateGroupPolicy::FiveMeasured)
    }
    /// [`Index`](Self::Index) with the MEASURED rate-group policy. The production spelling, and
    /// what `RadioPolicy::decide_power`'s `Option<u8>` becomes.
    pub const fn index(idx: u8) -> Self {
        Self::Index(idx, RateGroupPolicy::FiveMeasured)
    }
    /// The rate-group policy this request carries. `FiveMeasured` for requests where the concept
    /// does not apply (`Dbm`, `Raw`, `NoActuator`) — the raw writer covers exactly the five
    /// measured groups, so the answer is accurate rather than a filler.
    pub const fn rate_groups(&self) -> RateGroupPolicy {
        match self {
            Self::Ceiling(p) | Self::Index(_, p) => *p,
            _ => RateGroupPolicy::FiveMeasured,
        }
    }
    /// `true` for the one request that leaves the regulatory scale. Used by the bring-up warn and
    /// by anything that must refuse to compare a hot run with a calibrated one.
    pub const fn is_off_scale(&self) -> bool {
        matches!(self, Self::Raw { .. })
    }
    /// ⚠ **The raw chip axis, and only with the operator's written permission.**
    ///
    /// The sanctioned way for a bench tool to reach [`Raw`](Self::Raw): it goes through
    /// [`RfAuthority::from_env`], so `NDN_RF_UNRESTRICTED="<operator>:<reason>"` must be set and
    /// the operator's own words end up printed in the report. No library code can call this
    /// successfully without that variable, which is the property being defended.
    ///
    /// Returns the `Unsupported` error naming the variable when no authority is granted — an
    /// actionable refusal rather than a silent downgrade to the calibrated scale (or, as before,
    /// a silent *upgrade* off it).
    pub fn raw_from_env(idx: u8) -> Result<Self, crate::FaceError> {
        match RfAuthority::from_env() {
            Some(authority) => Ok(Self::Raw { idx, authority }),
            None => Err(power_unsupported(
                "PowerRequest::Raw leaves the regulatory scale (raw chip TXAGC, off the \
                 than the fused base on the RTL8812AU, may exceed licensed EIRP). It needs an \
                 explicit operator decision: set NDN_RF_UNRESTRICTED=\"<operator>:<reason>\" — the \
                 reason is printed verbatim in every report of the run.",
            )),
        }
    }

    /// The index this request asks for on the part's own scale, if it names one.
    pub const fn requested_index(&self) -> Option<u8> {
        match self {
            Self::Index(i, _) | Self::Raw { idx: i, .. } => Some(*i),
            _ => None,
        }
    }
}

/// **Which per-rate TXAGC groups the calibrated write covers** (RTL8812AU / Jaguar1).
///
/// ☠ MEASURED against a witness receiver, and this is settled:
///
/// * **12 groups**: offered 711 f/s, **on air 7 frames in 11 s = 1 f/s**;
/// * **5 groups**: offered 2805 f/s, **on air 2704 frames in 11 s = 246 f/s**.
///
/// Writing the seven HT-2SS/VHT groups *stops this radio transmitting* — a 250× difference on air
/// that the offered rate could not see at all.
///
/// ⚠ The ADDRESSES are not in doubt: this driver's own `PROGS_5G` writes all 24 of them,
/// `phy_reg.bin` boots them at the flat TXAGC packing `0x12121212`, the mainline kernel writes
/// per-rate ladders into them on this dongle, and `Hal8812PhyReg.h:178` names `0xc34`
/// `rTxAGC_A_MCS11_MCS8_JAguar`. What is in doubt is the **value** `index_base(..) + offset`
/// computes for the 2SS/VHT rate codes. [`AllTwelveUnderInvestigation`](Self::AllTwelveUnderInvestigation)
/// is an open investigation and needs a witness receiver in the loop.
///
/// A report that hides which of the two ran overclaims: the 8812au writes **5 of the 12** Jaguar1
/// groups, and the reader must be able to see that.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Default)]
pub enum RateGroupPolicy {
    /// The five groups MEASURED to transmit: CCK, OFDM 18-6, OFDM 54-24, HT MCS0-3, HT MCS4-7,
    /// on both RF paths — ten registers.
    #[default]
    FiveMeasured,
    /// All twelve Jaguar1 groups — twenty-four registers. **Do not select this without a witness
    /// receiver: the transmitter cannot detect the failure.**
    AllTwelveUnderInvestigation,
}

impl RateGroupPolicy {
    /// How many `(path-A, path-B)` group pairs this policy writes.
    pub const fn group_pairs(&self) -> usize {
        match self {
            Self::FiveMeasured => 5,
            Self::AllTwelveUnderInvestigation => 12,
        }
    }
}

/// **What the number written is referenced to.** This is the field the 2026-09-03 day of bisection
/// was missing: `ref=FusedBase / span 21..27` against `ref=ChipRaw / span 63..63` is the top line
/// of a diff, and no reader needs to know the bug exists to see that two runs disagree.
///
/// **There is deliberately no `Uncalibrated` variant.** The 8812au's uncalibrated path *is* the raw
/// path — its old tail literally called `set_tx_power_raw`. Giving it a third name would recreate
/// the ambiguity with better spelling. It is [`ChipRaw`](Self::ChipRaw), and therefore requires
/// authority.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum PowerReference {
    /// EFUSE per-channel per-rate calibration: **the adapter's own regulatory point.**
    /// `base_index` is that adapter's fused base for the representative rate on this channel.
    FusedBase { base_index: u8, channel: u8 },
    /// A driver/vendor TXAGC reference with no per-adapter fuse read — the a81a's `0x18e8`
    /// reference index, the 8733b's TSSI DE, the AR9271's gain LUT. Monotone and characterised,
    /// but not fused.
    ///
    /// `slope_db_per_idx` is `Some` **only where MEASURED** (a81a: ~0.22 dB/step over 20..=63,
    /// B210, three scrambled passes). `None` everywhere else — an invented slope is worse than
    /// none, because a link-budget calculation will believe it.
    DriverReference {
        source: &'static str,
        slope_db_per_idx: Option<f32>,
    },
    /// ⚠ Raw chip TXAGC, calibration bypassed. **May exceed licensed EIRP.** Requires an
    /// [`RfAuthority`]; a bring-up that resolves to this emits a `tracing::warn!`.
    ChipRaw,
    /// A real absolute axis (nl80211, Morse, NRC, LoRa, the ESP32-C5's quarter-dBm knob).
    AbsoluteDbm,
    /// No actuator. `writes` is empty and `actuated` is false.
    NoActuator,
}

impl PowerReference {
    /// `true` for the reference that is off the regulatory scale — the warn/diff trigger.
    pub const fn is_off_scale(&self) -> bool {
        matches!(self, Self::ChipRaw)
    }
    /// A short tag for `render`/`diff` and for structured log fields.
    pub const fn tag(&self) -> &'static str {
        match self {
            Self::FusedBase { .. } => "FusedBase",
            Self::DriverReference { .. } => "DriverReference",
            Self::ChipRaw => "ChipRaw",
            Self::AbsoluteDbm => "AbsoluteDbm",
            Self::NoActuator => "NoActuator",
        }
    }
}

/// One register the power knob actually wrote.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct PowerWrite {
    pub reg: u32,
    pub value: u8,
    /// The rate group this register carries (`"OFDM 18-6"`, `"HT MCS0-3"`, `"TSSI DE"`, …).
    pub group: &'static str,
    /// RF path (0 = A, 1 = B). `0` on single-chain parts.
    pub path: u8,
}

/// **What the actuator actually did.** Returned by [`crate::RadioKnobs::set_tx_power`]; carried in
/// [`RadioState::power`].
///
/// The point is that `requested` and `reference` travel together. `Ceiling` alone does not say
/// whether ≈27 or 63 reached silicon; `Ceiling` + `FusedBase{27}` does.
#[derive(Clone, Debug, PartialEq)]
pub struct AppliedPower {
    /// What the caller asked for, verbatim — including the [`RfAuthority`]'s words when the answer
    /// left the regulatory scale.
    pub requested: PowerRequest,
    /// What the number written is referenced to. **The field the bisection was missing.**
    pub reference: PowerReference,
    /// The API-scale index used after clamping. **NOT a register value.**
    pub index_requested: u8,
    /// `true` if `min_tx_power` or a driver clamp moved the request. The a81a clamps to `20..=63`
    /// because below ~20 its gain chain inverts to ~11 dB **above** the calibrated maximum
    /// (MEASURED on a B210, three scrambled passes, ±0.1 dB) — a "very low power" request would
    /// radiate the loudest signal the chip can make.
    pub clamped: bool,
    /// Every register actually written, in order.
    ///
    /// ☠ On the calibrated 8812au these values DIFFER per rate group
    /// (`index_base(path, rate, ch) + offset`), so there is no single "index written" and the
    /// report must not invent one. Ten entries versus twenty-four is also where
    /// [`RateGroupPolicy`] becomes visible without knowing the flag exists.
    pub writes: Vec<PowerWrite>,
    /// `Some` only when every write carries the same value (the raw/flat case). `None` otherwise —
    /// read [`writes`](Self::writes) / [`index_span`](Self::index_span).
    pub index_written: Option<u8>,
    /// `(min, max)` over `writes`. `None` when `writes` is empty.
    pub index_span: Option<(u8, u8)>,
    /// Absolute power **only where the part has a real dBm axis**. Never inferred from an index.
    ///
    /// ★ No Wi-Fi part in this fleet has a real dBm axis, and `db_per_power_idx` is a global 0.5
    /// that no part obeys (MEASURED 0.22 on the a81a, 0.111–0.155 on the 8733b). Inventing a
    /// figure here is worse than `None`, because a planner budgets link margin from it.
    pub dbm: Option<i8>,
    /// `false` = the write did not reach silicon (no actuator, or the part declined).
    pub actuated: bool,
}

impl AppliedPower {
    /// Build from the writes, deriving `index_written` / `index_span` so a driver cannot get the
    /// two out of step with `writes`. The **only** constructor drivers should use.
    pub fn from_writes(
        requested: PowerRequest,
        reference: PowerReference,
        index_requested: u8,
        clamped: bool,
        writes: Vec<PowerWrite>,
    ) -> Self {
        let span = writes.iter().fold(None::<(u8, u8)>, |acc, w| match acc {
            None => Some((w.value, w.value)),
            Some((lo, hi)) => Some((lo.min(w.value), hi.max(w.value))),
        });
        let flat = span.filter(|(lo, hi)| lo == hi).map(|(v, _)| v);
        Self {
            requested,
            reference,
            index_requested,
            clamped,
            actuated: !writes.is_empty(),
            index_written: flat,
            index_span: span,
            writes,
            dbm: None,
        }
    }

    /// An applied power on a real absolute axis. `dbm` is what the radio **reported applying**,
    /// never the request — firmware and regulatory tables clamp (30 dBm applies as 27).
    pub fn absolute_dbm(requested: PowerRequest, applied: i8, clamped: bool) -> Self {
        Self {
            requested,
            reference: PowerReference::AbsoluteDbm,
            index_requested: applied.max(0) as u8,
            clamped,
            writes: Vec::new(),
            index_written: None,
            index_span: None,
            dbm: Some(applied),
            actuated: true,
        }
    }

    /// The part has no power actuator. Distinct from "we did not set power".
    pub fn no_actuator(requested: PowerRequest) -> Self {
        Self {
            requested,
            reference: PowerReference::NoActuator,
            index_requested: 0,
            clamped: false,
            writes: Vec::new(),
            index_written: None,
            index_span: None,
            dbm: None,
            actuated: false,
        }
    }

    /// Attach a measured absolute power. Callers must pass a value the **hardware reported**, not
    /// one computed from an index.
    pub fn with_measured_dbm(mut self, dbm: i8) -> Self {
        self.dbm = Some(dbm);
        self
    }

    /// One line, for `render` and for a bench print.
    pub fn render(&self) -> String {
        let span = match self.index_span {
            Some((lo, hi)) if lo == hi => format!("{lo}..{hi}"),
            Some((lo, hi)) => format!("{lo}..{hi}"),
            None => "-".to_string(),
        };
        let refs = match self.reference {
            PowerReference::FusedBase {
                base_index,
                channel,
            } => {
                format!("FusedBase{{base:{base_index}, ch:{channel}}}")
            }
            PowerReference::DriverReference {
                source,
                slope_db_per_idx,
            } => match slope_db_per_idx {
                Some(s) => format!("DriverReference{{{source}, {s:.2} dB/idx}}"),
                None => format!("DriverReference{{{source}, slope unmeasured}}"),
            },
            PowerReference::ChipRaw => "ChipRaw".to_string(),
            PowerReference::AbsoluteDbm => "AbsoluteDbm".to_string(),
            PowerReference::NoActuator => "NoActuator".to_string(),
        };
        let dbm = match self.dbm {
            Some(d) => format!("{d}"),
            None => "None".to_string(),
        };
        let auth = match &self.requested {
            PowerRequest::Raw { authority, .. } => {
                format!("  ⚠ AUTHORITY {}", authority.render())
            }
            _ => String::new(),
        };
        format!(
            "ref={refs}  req={:?}  clamped={}  dbm={dbm}  {}\n         writes {} regs  idx span {span}   [{:?}]{auth}",
            self.requested,
            if self.clamped { "YES" } else { "no" },
            if self.actuated {
                "actuated"
            } else {
                "NOT ACTUATED"
            },
            self.writes.len(),
            self.requested.rate_groups(),
        )
    }
}

/// ⚠ **An explicit operator decision to transmit off the regulatory scale.** Carries who and why,
/// and the string is printed verbatim in every report that used it.
///
/// ★ **Unconstructible from library code, by the type system rather than by discipline.** The
/// fields are private and there is no `Default`, no `new`, and no public struct literal: the only
/// ways in are [`from_env`](Self::from_env) and, under `feature = "bench"`, `bench`.
/// So no `RadioPolicy`, no face, and no forwarder can reach the raw axis — the regulatory ceiling
/// is enforced, not merely documented.
///
/// `nav_probe`'s hot run would still be **allowed** (it was a legitimate bench experiment) and
/// would have been visibly a regulatory deviation from its first line of output.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct RfAuthority {
    granted_by: String,
    reason: String,
}

impl RfAuthority {
    /// `NDN_RF_UNRESTRICTED="<operator>:<reason>"`. Registered in `ndn_env` as
    /// `Class::DebugBisect`, so it already appears in the run header flagged as a confounder.
    ///
    /// Returns `None` when the variable is unset, empty, or carries no `:` separator — an
    /// authority with no stated reason is not an authority.
    pub fn from_env() -> Option<Self> {
        let v = std::env::var("NDN_RF_UNRESTRICTED").ok()?;
        let (who, why) = v.split_once(':')?;
        let (who, why) = (who.trim(), why.trim());
        if who.is_empty() || why.is_empty() {
            return None;
        }
        Some(Self {
            granted_by: who.to_string(),
            reason: why.to_string(),
        })
    }

    /// Bench-only constructor. Compiled only under `feature = "bench"`, which examples that need
    /// the raw axis declare in `required-features`.
    #[cfg(feature = "bench")]
    pub fn bench(granted_by: impl Into<String>, reason: impl Into<String>) -> Self {
        Self {
            granted_by: granted_by.into(),
            reason: reason.into(),
        }
    }

    /// Who granted it.
    pub fn granted_by(&self) -> &str {
        &self.granted_by
    }
    /// Why. Printed verbatim; never summarised.
    pub fn reason(&self) -> &str {
        &self.reason
    }
    /// `<who>:"<why>"`, the form the report prints.
    pub fn render(&self) -> String {
        format!("{}:{:?}", self.granted_by, self.reason)
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// §1.1 fragments the report needs
// ─────────────────────────────────────────────────────────────────────────────

/// What the radio was brought up to do. Replaces `bring_up_monitor` vs `bring_up_tx` as *separate
/// functions*, and `NDN_8733B_RX_ONLY` as a hidden fork.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Role {
    ReceiveOnly,
    TransmitOnly,
    TransmitAndReceive,
}

impl Role {
    /// `true` if this role intends to put energy on the air — the gate LAW 4 keys on.
    pub const fn transmits(&self) -> bool {
        matches!(self, Self::TransmitOnly | Self::TransmitAndReceive)
    }
}

/// Who owns the RX pump, and how deep. Replaces the four different pump owners.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum PumpPolicy {
    /// The factory started a pump this deep.
    Start(usize),
    /// The caller runs its own receive loop.
    CallerOwns,
    /// No pump (`NDN_NO_PUMP`: a pure TX-blast node, whose bulk-IN threads would otherwise contend
    /// with inject for USB bandwidth).
    None,
}

/// Where the radio physically is. The HAL cannot name `DeviceSelect` (that is a USB concept owned
/// by the driver crate), so this is the bearer-agnostic form a report prints.
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum DeviceAddress {
    /// USB topological address, `"<bus>-<port>[.<port>…]"`.
    Usb(String),
    /// A serial device path.
    Serial(String),
    /// A kernel netdev (HaLow, mac80211).
    NetDev(String),
    /// Loopback / simulation — no hardware.
    Synthetic,
    /// The opener did not resolve one. Honest; not a placeholder for "first on the bus".
    Unknown,
}

impl std::fmt::Display for DeviceAddress {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::Usb(s) => write!(f, "{s}"),
            Self::Serial(s) => write!(f, "{s}"),
            Self::NetDev(s) => write!(f, "{s}"),
            Self::Synthetic => write!(f, "synthetic"),
            Self::Unknown => write!(f, "?"),
        }
    }
}

/// Identifies the sequence that ran. `ver` is bumped when the steps change, so an on-air number
/// recorded beside a `PlanId` cannot be silently compared across a ladder change.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct PlanId {
    pub part: &'static str,
    pub name: &'static str,
    pub ver: u16,
}

impl std::fmt::Display for PlanId {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "{}/{}@v{}", self.part, self.name, self.ver)
    }
}

/// A rendering / `--stop-after` label **only**. It is not an enforcement device: any sequence can
/// be made phase-monotone by relabelling. Ordering is enforced by step-id constraints (M3+).
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord)]
pub enum Stage {
    Attach,
    PowerOn,
    Firmware,
    MacInit,
    PhyInit,
    Tune,
    Calibrate,
    TxEnable,
    RxEnable,
    Power,
    Posture,
    Verify,
}

/// Why a rung is there, and what its failure costs.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum StepClass {
    /// Failure aborts. **LAW 5: a step that polls a hardware completion bit is always `Required`.**
    Required,
    /// Failure warns and continues, and MUST name what is lost. Replaces every bare `let _ =` in
    /// today's ladders, each of which discards a real degradation silently.
    ///
    /// ★ The payload is a [`Degradation`], not a bare string: a rung whose failure has **no
    /// declared consequence is not expressible**. Both of its fields are checked non-empty by
    /// [`Plan::check`](plan::Plan::check).
    BestEffort(Degradation),
    /// A readback asserting an invariant the steps above should have established.
    Assert,
    /// The work happened OUT OF BAND (`modprobe` / `iw` / `hostapd_s1g` / `morse_cli`) and this
    /// step only validates it. Honest for HaLow; a lie anywhere else.
    OutOfBand { established_by: &'static str },
}

/// A fact a step established that changes what a later API means. `load_tx_power_info` is the
/// founding case: it decided, invisibly, what `set_tx_power` meant.
#[derive(Clone, Debug, PartialEq)]
pub enum Fact {
    PowerReference(PowerReference),
    Warm(bool),
    /// e.g. the ath9k high-power gain table selected from `eeprom_tx_gain_type`.
    GainTable(&'static str),
    Firmware {
        name: &'static str,
        ready: bool,
    },
}

/// What a step did, **as recorded in the report**.
///
/// ⚠ Named `StepOutcomeRecord`, not `StepOutcome`: the contract's `StepOutcome` has a
/// `Guard(Box<dyn Any + Send + Sync>)` arm that cannot be `Clone`, and a report must be. The live
/// guard moves onto the handle; the report keeps only its name. `StepOutcome` is left free for the
/// M3 runner.
#[derive(Clone, Debug, PartialEq)]
pub enum StepOutcomeRecord {
    Done,
    /// A step that branched internally says which way. Branching *between* steps is not
    /// expressible; a warm/cold decision lives INSIDE one named step and reports here.
    Branch(&'static str),
    /// **LAW 6: every `Fact` must land in [`RadioState::facts`].**
    Established(Fact),
    /// A live guard the handle owns (the 8733b `PowerTracker`), by name. The live value moves
    /// into [`Guards`] on the handle; the report keeps only the name.
    Guard(&'static str),
    /// Why it did not run. Owned, because a [`Deviation`]'s `why` is the
    /// operator's own words and arrives as a `String`.
    Skipped(String),
    /// The step failed. Carries the rendered error, so a partial report is readable without the
    /// original `FaceError`.
    Failed(String),
}

/// One executed rung.
#[derive(Clone, Debug, PartialEq)]
pub struct StepRecord {
    pub id: plan::StepId,
    pub stage: Stage,
    pub class: StepClass,
    pub outcome: StepOutcomeRecord,
    pub elapsed_us: u64,
}

impl StepRecord {
    /// A rung that ran to completion, hand-filled by an M2 bring-up.
    pub fn done(id: impl Into<plan::StepId>, stage: Stage) -> Self {
        Self {
            id: id.into(),
            stage,
            class: StepClass::Required,
            outcome: StepOutcomeRecord::Done,
            elapsed_us: 0,
        }
    }
    pub fn with_elapsed(mut self, us: u64) -> Self {
        self.elapsed_us = us;
        self
    }
    pub fn with_class(mut self, class: StepClass) -> Self {
        self.class = class;
        self
    }
    pub fn established(id: impl Into<plan::StepId>, stage: Stage, fact: Fact) -> Self {
        Self {
            id: id.into(),
            stage,
            class: StepClass::Required,
            outcome: StepOutcomeRecord::Established(fact),
            elapsed_us: 0,
        }
    }
    pub fn skipped(id: impl Into<plan::StepId>, stage: Stage, why: &'static str) -> Self {
        Self {
            id: id.into(),
            stage,
            class: StepClass::BestEffort(plan::Degradation {
                lost: why,
                still_valid_for: "the rungs that did run; see `steps`",
            }),
            outcome: StepOutcomeRecord::Skipped(why.to_string()),
            elapsed_us: 0,
        }
    }
}

/// How loudly a failed readback speaks. **`Warn` on introduction for every part**; promoted to
/// `Fatal` per part only with a measurement.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Severity {
    Warn,
    Fatal,
}

/// One readback, and whether the hardware agreed with the ladder.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct AssertRecord {
    pub id: &'static str,
    pub reg: u32,
    pub read: u32,
    pub want: u32,
    pub ok: bool,
    pub severity: Severity,
}

/// One erased edit to a plan. **Erased only** — these cross the `open_radio(pid, &req)` boundary,
/// where the backend type is not known (a typed `InsertAfter(Step<B>)` would make the request
/// generic over `B` and destroy the runtime PID dispatch; typed insertion lives in `bench`).
///
/// [`Skip`](Self::Skip) and [`StopAfter`](Self::StopAfter) are **applied** by
/// [`run_plan`]. [`Poke`](Self::Poke) and
/// [`RegulatoryOverride`](Self::RegulatoryOverride) are **recorded but not executed by the
/// runner** — a poke needs register access the HAL cannot name, and the override is consumed by
/// [`PowerRequest::Raw`] at the knob. Both still change the digest, which is the part that
/// matters: a run that pokes is not the same run.
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum DeviationOp {
    /// The exact operation that found the 2026-09-03 defect: three arms plus a confirm.
    Skip(String),
    StopAfter(Stage),
    /// Raw register poke after the plan completes.
    Poke {
        addr: u32,
        val: u32,
        width: u8,
    },
    /// ⚠ The regulatory override, recorded.
    RegulatoryOverride {
        authority: RfAuthority,
    },
}

/// Whether this run was the canonical sequence, and if not, what question it existed to answer.
///
/// ★ Self-labelling: any measurement taken through a deviated bring-up carries its own asterisk,
/// forever, in its own output — and the [`BringUpReport::plan_digest`] differs, so a bench number
/// and a production number can never be silently compared.
#[derive(Clone, Debug, PartialEq)]
pub enum Provenance {
    Canonical,
    Deviated {
        question: String,
        ops: Vec<DeviationOp>,
    },
}

/// The rate the radio was left at: the wire code **and** what it decodes to, because a DESC/TXWI
/// code alone has cost this project days.
#[derive(Clone, Debug, PartialEq)]
pub struct RateState {
    /// The chip's own rate code (`DESC_RATE_*`, TXWI rate word), as written.
    pub code: u32,
    /// What that code means in words: `"legacy OFDM 6 Mb/s"`, `"HT MCS7"`, `"SF7/125 kHz"`.
    pub decoded: String,
    /// `None` unless the part has a real dBm axis. Same rule as [`AppliedPower::dbm`].
    pub dbm_equivalent: Option<i8>,
}

impl RateState {
    pub fn new(code: u32, decoded: impl Into<String>) -> Self {
        Self {
            code,
            decoded: decoded.into(),
            dbm_equivalent: None,
        }
    }
    /// "the radio did not report a rate" — honest, and different from rate 0.
    pub fn unreported() -> Self {
        Self {
            code: 0,
            decoded: "unreported".to_string(),
            dbm_equivalent: None,
        }
    }
}

/// The instrument that could answer "did the MAC key the transmitter?" — question **(A)**.
/// Question **(B)**, "did anything coherent radiate?", is answerable only by a peer.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct TxInstrument {
    /// e.g. `"read_tx_counters 0x2de0/0x2de2"`.
    pub name: &'static str,
    /// Roughly what one probe costs, microseconds. `morse_cli stats` is ~10.6 ms; NRC's
    /// `show mac tx stats` is ≥ 300 ms, which is why it is `Unprovable` at open.
    pub cost_us: u32,
}

/// A peer that decoded our probes. Minted only from a peer's report — a bring-up on a single host
/// cannot prove radiation, and a design that claims otherwise is lying.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct WitnessId(pub String);

/// **(A) and (B) are different questions, and only (A) is answerable on-chip.**
///
/// > `ath9k_htc.rs:2655`, MEASURED: without `WMI_TARGET_IC_UPDATE` the descriptor chain-select is 0
/// > and *"the MAC keys the transmitter (TFCNT advances, TXOK completes) but nothing coherent
/// > radiates — a witness at inches decoded 0 of our frames."*
///
/// There is no `None`/`Option` here: silence is the defect. A part that cannot answer says so.
#[derive(Clone, Debug, PartialEq)]
pub enum TxProof {
    /// [`Role::ReceiveOnly`].
    NotRequested,
    MacKeyed {
        instrument: TxInstrument,
        probes: u16,
        delta: u16,
        idle_control: Option<u16>,
    },
    /// (A) declined, **with the reason**. A first-class success value, not an omission.
    Unprovable {
        instrument: Option<TxInstrument>,
        reason: &'static str,
    },
    /// (B). The only real answer, and only a peer can mint it.
    WitnessDecoded {
        witness: WitnessId,
        sent: u32,
        heard: u32,
        rssi_dbm: Option<i8>,
    },
    /// The instrument ran and said NO.
    Refuted {
        instrument: TxInstrument,
        detail: String,
    },
}

impl TxProof {
    /// The one-line form `render` prints.
    pub fn render(&self) -> String {
        match self {
            Self::NotRequested => "not requested (ReceiveOnly)".to_string(),
            Self::MacKeyed {
                instrument,
                probes,
                delta,
                idle_control,
            } => format!(
                "MAC-KEYED via {} — {probes} probes, counter +{delta}{}",
                instrument.name,
                match idle_control {
                    Some(c) => format!(", idle control +{c}"),
                    None => String::new(),
                }
            ),
            Self::Unprovable { instrument, reason } => match instrument {
                Some(i) => format!("UNPROVABLE ({}) — {reason}", i.name),
                None => format!("UNPROVABLE — {reason}"),
            },
            Self::WitnessDecoded {
                witness,
                sent,
                heard,
                rssi_dbm,
            } => format!(
                "WITNESS {} decoded {heard}/{sent}{}",
                witness.0,
                match rssi_dbm {
                    Some(r) => format!(" at {r} dBm"),
                    None => String::new(),
                }
            ),
            Self::Refuted { instrument, detail } => {
                format!("REFUTED by {} — {detail}", instrument.name)
            }
        }
    }
}

/// **What must be PROVEN about the transmitter before the handle is returned** (contract §4).
///
/// The whole point of the enum is that *"I did not check"* is spellable only on a radio that does
/// not intend to transmit. This stack shipped a ~20 dB power deficit for a day because a
/// transmitting bring-up returned `Ok(())` and nobody, including the driver, had looked.
///
/// ⚠ Only (A) — *did the MAC key the transmitter?* — is answerable on-chip.
/// [`WitnessOrFail`](Self::WitnessOrFail) is the only variant that touches (B), *did anything
/// coherent radiate?*, and it can only be answered by a peer.
#[derive(Clone, Debug, PartialEq, Eq, Default)]
pub enum ProofRequirement {
    /// ★ LAW 4: legal ONLY with [`Role::ReceiveOnly`]; validation rejects it otherwise.
    None,
    /// Probe if this part offers one; [`TxProof::Unprovable`] otherwise, and bring-up **succeeds**
    /// saying so. **The production default** — a radio must not refuse to come up because its
    /// silicon is blind.
    #[default]
    BestAvailable,
    /// The instrument must move, or bring-up fails. Requesting it of a part with no instrument is
    /// [`RequestError::Unsatisfiable`], never a silent pass.
    MacKeyedOrFail,
    /// A named witness must decode the probes, or bring-up fails. The only setting proving (B).
    /// This is `Rtl8733buBackend::bring_up_tx_until(verify)` — which existed on one part, unused by
    /// the factory — generalised and given a home.
    WitnessOrFail { witness: WitnessId, min_heard: u32 },
}

impl std::fmt::Display for ProofRequirement {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::None => f.write_str("none (ReceiveOnly only)"),
            Self::BestAvailable => f.write_str("best-available"),
            Self::MacKeyedOrFail => f.write_str("mac-keyed-or-fail"),
            Self::WitnessOrFail { witness, min_heard } => {
                write!(f, "witness {} must decode >= {min_heard}", witness.0)
            }
        }
    }
}

/// One reading of a [`TxInstrument`], taken across a burst of probe transmits.
///
/// The part owns the probe because only the part knows how to key its own transmitter; the runner
/// owns the *decision* about whether the answer is acceptable. `idle_control` is the same
/// difference taken across an interval with no probes, and it is what distinguishes "the counter
/// moved because we transmitted" from "the counter free-runs" — the Morse instrument is the
/// fleet's best-calibrated one precisely because somebody took that control (+200/200 with a +0
/// idle control).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct TxProbe {
    /// How many times the probe keyed the transmitter.
    pub probes: u16,
    /// The instrument's movement across those probes.
    pub delta: u16,
    /// The same difference across an idle interval, if the part took one. `None` is honest;
    /// a fabricated zero is not.
    pub idle_control: Option<u16>,
}

/// A degradation the bring-up chose to continue past. Every one names what is lost.
#[derive(Clone, Debug, PartialEq)]
pub struct Warning {
    /// The step or knob that degraded.
    pub at: &'static str,
    /// What is lost as a consequence — not "failed", but *what the caller no longer has*.
    pub degrades: String,
}

impl Warning {
    pub fn new(at: &'static str, degrades: impl Into<String>) -> Self {
        Self {
            at,
            degrades: degrades.into(),
        }
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// §3 — The report
// ─────────────────────────────────────────────────────────────────────────────

/// The regime a bring-up left the radio in.
#[derive(Clone, Debug)]
pub struct RadioState {
    pub channel: u8,
    pub bw: Bandwidth,
    pub format: &'static str,
    pub role: Role,
    /// ★ §2 — the field the 2026-09-03 bisection was missing.
    pub power: AppliedPower,
    /// The DESC/TXWI code **and** what it decodes to.
    pub rate: RateState,
    /// `Option`: only the mt76 family establishes warm/cold. Asserting `false` elsewhere would be
    /// a claim.
    pub warm: Option<bool>,
    /// `Option`: not every part actuates EDCA. `None` = inherited/unknown, which on USB is a real
    /// and hazardous state (MEASURED: a 2.5× throughput swing on the MT7610U decided by run order).
    pub contention: Option<crate::ContentionApplied>,
    pub pump: PumpPolicy,
    /// **LAW 6** — every `Established(Fact)` lands here.
    pub facts: Vec<Fact>,
}

/// **The account of how a radio was brought up.** Returned by every `bring_up_*`, carried on
/// `OpenRadio`, and printed at INFO on every open.
///
/// A bring-up that returns `Ok(())` and nothing else is defect class 3 of the root-cause doc: it
/// leaves the operator to bisect for a day what one line of output could have said.
#[derive(Clone, Debug)]
#[non_exhaustive]
pub struct BringUpReport {
    pub part: &'static str,
    pub device: DeviceAddress,
    pub plan: PlanId,
    /// FNV-1a over the ordered `(step id, ran|skipped)` sequence ACTUALLY executed, plus the power
    /// reference and the rate-group policy.
    ///
    /// ★ The answer to "which bring-up did you use?" — the question with no answer today. **Record
    /// it beside every on-air number.**
    pub plan_digest: u64,
    pub provenance: Provenance,
    /// ★ **Every edit this run applied to the canonical plan, each with its own non-empty `why`.**
    /// [`Provenance::Deviated`] says *that* the run departed and what question it was asking;
    /// this says, edit by edit, *what was changed and on what grounds*. Both feed
    /// [`plan_digest`](Self::plan_digest), so a bench number and a production number can never be
    /// compared by accident.
    pub deviations: Vec<plan::PlanEdit>,
    pub steps: Vec<StepRecord>,
    pub asserts: Vec<AssertRecord>,
    pub state: RadioState,
    pub tx: TxProof,
    /// Post-bring-up, for the PHY actually configured. `None` only where the part declares no
    /// capability at all.
    pub capability: Option<RadioCapability>,
    pub warnings: Vec<Warning>,
    pub started_at: SystemTime,
    pub elapsed: Duration,
}

impl BringUpReport {
    /// A hand-filled report for an M2 bring-up: the sequence has not moved into a `Plan` yet, so
    /// the driver states the regime it knows it left the radio in.
    ///
    /// ⚠ **This is a claim the driver makes about itself** (contract §7.2). A step whose closure
    /// writes the wrong register still reports `Done`. What it removes is the *silence*.
    pub fn hand_filled(
        part: &'static str,
        plan: PlanId,
        device: DeviceAddress,
        state: RadioState,
    ) -> Self {
        let mut r = Self {
            part,
            device,
            plan,
            plan_digest: 0,
            provenance: Provenance::Canonical,
            deviations: Vec::new(),
            steps: Vec::new(),
            asserts: Vec::new(),
            state,
            tx: TxProof::Unprovable {
                instrument: None,
                reason: "hand-filled M2 report: no TX probe has been run",
            },
            capability: None,
            warnings: Vec::new(),
            started_at: SystemTime::now(),
            elapsed: Duration::ZERO,
        };
        r.plan_digest = r.compute_digest();
        r
    }

    /// A loopback / simulation handle's report. No hardware; the honest answer to every question.
    pub fn synthetic(part: &'static str) -> Self {
        Self::hand_filled(
            part,
            PlanId {
                part,
                name: "synthetic",
                ver: 0,
            },
            DeviceAddress::Synthetic,
            RadioState {
                channel: 0,
                bw: Bandwidth::Bw20,
                format: "synthetic",
                role: Role::TransmitAndReceive,
                power: AppliedPower::no_actuator(PowerRequest::NoActuator),
                rate: RateState::unreported(),
                warm: None,
                contention: None,
                pump: PumpPolicy::CallerOwns,
                facts: Vec::new(),
            },
        )
    }

    pub fn with_steps(mut self, steps: Vec<StepRecord>) -> Self {
        for s in &steps {
            if let StepOutcomeRecord::Established(f) = &s.outcome {
                if !self.state.facts.contains(f) {
                    self.state.facts.push(f.clone());
                }
            }
        }
        self.steps = steps;
        self.plan_digest = self.compute_digest();
        self
    }
    pub fn with_tx(mut self, tx: TxProof) -> Self {
        self.tx = tx;
        self
    }
    pub fn with_capability(mut self, cap: RadioCapability) -> Self {
        self.capability = Some(cap);
        self
    }
    pub fn with_warning(mut self, w: Warning) -> Self {
        self.warnings.push(w);
        self
    }
    pub fn with_provenance(mut self, p: Provenance) -> Self {
        self.provenance = p;
        self.plan_digest = self.compute_digest();
        self
    }
    pub fn with_elapsed(mut self, d: Duration) -> Self {
        self.elapsed = d;
        self
    }
    /// Replace the recorded power. Used where the knob runs *after* the ladder (the `NDN_TX_PWR`
    /// arms of `open_radio`), so the report carries what was finally applied.
    pub fn with_power(mut self, p: AppliedPower) -> Self {
        self.state.power = p;
        self.plan_digest = self.compute_digest();
        self
    }

    /// FNV-1a over what actually ran plus the power regime. Two runs that differ in either differ
    /// here, which is the whole point: a bench number and a production number cannot be compared
    /// by accident.
    pub fn compute_digest(&self) -> u64 {
        const OFFSET: u64 = 0xcbf2_9ce4_8422_2325;
        const PRIME: u64 = 0x0000_0100_0000_01b3;
        let mut h = OFFSET;
        let mut eat = |b: &[u8]| {
            for &c in b {
                h ^= c as u64;
                h = h.wrapping_mul(PRIME);
            }
        };
        eat(self.plan.part.as_bytes());
        eat(self.plan.name.as_bytes());
        eat(&self.plan.ver.to_le_bytes());
        for s in &self.steps {
            eat(s.id.0.as_bytes());
            eat(match s.outcome {
                StepOutcomeRecord::Skipped(_) => b"skip",
                StepOutcomeRecord::Failed(_) => b"fail",
                _ => b"ran",
            });
        }
        eat(self.state.power.reference.tag().as_bytes());
        eat(format!("{:?}", self.state.power.requested.rate_groups()).as_bytes());
        if let Provenance::Deviated { question, ops } = &self.provenance {
            eat(b"deviated");
            eat(question.as_bytes());
            eat(format!("{ops:?}").as_bytes());
        }
        // ★ The stated reason is part of the digest, deliberately. Two runs that skipped the same
        // rung for different reasons were asking different questions, and their numbers should not
        // land in the same bucket. The cost is that editing a `why` moves the digest; that is the
        // intended direction (§1.6 rule 2).
        for e in &self.deviations {
            eat(b"edit");
            eat(format!("{:?}", e.op).as_bytes());
            eat(e.why.as_bytes());
        }
        h
    }

    /// The block printed at INFO on every open.
    ///
    /// The two 2026-09-03 runs, rendered, are the whole design in eight lines — and
    /// `ref=FusedBase / span 21..27` against `ref=ChipRaw / span 63..63` is visible to a reader who
    /// has never heard of the bug.
    pub fn render(&self) -> String {
        let mut o = String::new();
        let prov = match &self.provenance {
            Provenance::Canonical => "CANONICAL".to_string(),
            Provenance::Deviated { .. } => "DEVIATED".to_string(),
        };
        o.push_str(&format!(
            "radio {} {}  plan {}  digest {:#018x}  {prov}   {:.2} s\n",
            self.part,
            self.device,
            self.plan,
            self.plan_digest,
            self.elapsed.as_secs_f64()
        ));
        if let Provenance::Deviated { question, ops } = &self.provenance {
            o.push_str(&format!("  deviation {question:?}  ops: {ops:?}\n"));
        }
        for e in &self.deviations {
            o.push_str(&format!("    edit {:?} — {}\n", e.op, e.why));
        }
        o.push_str(&format!(
            "  ch {} / {:?}   format {}   role {:?}   pump {:?}\n",
            self.state.channel, self.state.bw, self.state.format, self.state.role, self.state.pump
        ));
        o.push_str(&format!("  power  {}\n", self.state.power.render()));
        o.push_str(&format!(
            "  rate   {} (code {:#x})\n",
            self.state.rate.decoded, self.state.rate.code
        ));
        if let Some(w) = self.state.warm {
            o.push_str(&format!("  warm   {w}\n"));
        }
        if !self.state.facts.is_empty() {
            o.push_str(&format!("  facts  {:?}\n", self.state.facts));
        }
        if !self.steps.is_empty() {
            let line: Vec<String> = self
                .steps
                .iter()
                .map(|s| match &s.outcome {
                    StepOutcomeRecord::Skipped(_) => format!("{}:skip", s.id),
                    StepOutcomeRecord::Failed(_) => format!("{}:FAIL", s.id),
                    StepOutcomeRecord::Branch(b) => format!("{}:{b}", s.id),
                    _ => s.id.to_string(),
                })
                .collect();
            o.push_str(&format!("  steps  {}\n", line.join(" · ")));
        }
        if !self.asserts.is_empty() {
            let line: Vec<String> = self
                .asserts
                .iter()
                .map(|a| {
                    format!(
                        "{:#06x}={:#04x} {}",
                        a.reg,
                        a.read,
                        if a.ok { "ok" } else { "MISMATCH" }
                    )
                })
                .collect();
            o.push_str(&format!("  assert {}\n", line.join(" · ")));
        }
        o.push_str(&format!("  tx     {}\n", self.tx.render()));
        for w in &self.warnings {
            o.push_str(&format!("  ⚠ {} — {}\n", w.at, w.degrades));
        }
        o
    }

    /// Structured `tracing` fields → OTLP, plus the `warn!` the contract requires whenever the
    /// resolved reference is the raw/chip-max axis (§2.5.3).
    ///
    /// ⚠ The HAL has no `tracing` dependency, so this emits through `eprintln!` only when the
    /// reference is off-scale. **The `tracing::warn!` itself lives at the driver call site**
    /// (`open_radio`), which is where a `tracing` subscriber exists. Named rather than
    /// hidden: this method is the fallback, not the mechanism.
    pub fn emit(&self) {
        if self.state.power.reference.is_off_scale() {
            eprintln!(
                "⚠ RF OFF THE REGULATORY SCALE — {} {} is transmitting on the raw chip axis; \
                 this may exceed licensed EIRP. {}",
                self.part,
                self.device,
                self.state.power.render()
            );
        }
    }

    /// ★ **Detection by contrast** — the operation you reflexively perform when one run works and
    /// one does not, and the one that would have ended 2026-09-03 in minutes.
    pub fn diff(&self, other: &Self) -> Vec<Difference> {
        let mut d = Vec::new();
        let mut push = |field: &'static str, a: String, b: String| {
            if a != b {
                d.push(Difference { field, a, b });
            }
        };
        push("part", self.part.into(), other.part.into());
        push("plan", self.plan.to_string(), other.plan.to_string());
        push(
            "plan_digest",
            format!("{:#018x}", self.plan_digest),
            format!("{:#018x}", other.plan_digest),
        );
        push(
            "provenance",
            format!("{:?}", self.provenance),
            format!("{:?}", other.provenance),
        );
        push(
            "deviations",
            format!("{:?}", self.deviations),
            format!("{:?}", other.deviations),
        );
        // ★ Power first among the state fields in construction order, because it is the field that
        // cost a day. `reference` is separated from the request deliberately: two runs can agree on
        // `Ceiling` and disagree by 33 dB.
        push(
            "power.reference",
            format!("{:?}", self.state.power.reference),
            format!("{:?}", other.state.power.reference),
        );
        push(
            "power.requested",
            format!("{:?}", self.state.power.requested),
            format!("{:?}", other.state.power.requested),
        );
        push(
            "power.index_span",
            format!("{:?}", self.state.power.index_span),
            format!("{:?}", other.state.power.index_span),
        );
        push(
            "power.clamped",
            self.state.power.clamped.to_string(),
            other.state.power.clamped.to_string(),
        );
        push(
            "power.dbm",
            format!("{:?}", self.state.power.dbm),
            format!("{:?}", other.state.power.dbm),
        );
        push(
            "power.rate_groups",
            format!("{:?}", self.state.power.requested.rate_groups()),
            format!("{:?}", other.state.power.requested.rate_groups()),
        );
        push(
            "channel",
            self.state.channel.to_string(),
            other.state.channel.to_string(),
        );
        push(
            "bw",
            format!("{:?}", self.state.bw),
            format!("{:?}", other.state.bw),
        );
        push(
            "format",
            self.state.format.into(),
            other.state.format.into(),
        );
        push(
            "role",
            format!("{:?}", self.state.role),
            format!("{:?}", other.state.role),
        );
        push(
            "rate",
            self.state.rate.decoded.clone(),
            other.state.rate.decoded.clone(),
        );
        push(
            "warm",
            format!("{:?}", self.state.warm),
            format!("{:?}", other.state.warm),
        );
        push(
            "pump",
            format!("{:?}", self.state.pump),
            format!("{:?}", other.state.pump),
        );
        push("tx", self.tx.render(), other.tx.render());
        // Steps, by id and disposition — the bisect axis.
        let seq = |r: &Self| {
            r.steps
                .iter()
                .map(|s| match s.outcome {
                    StepOutcomeRecord::Skipped(_) => format!("{}:skip", s.id),
                    StepOutcomeRecord::Failed(_) => format!("{}:FAIL", s.id),
                    _ => s.id.to_string(),
                })
                .collect::<Vec<_>>()
                .join(",")
        };
        push("steps", seq(self), seq(other));
        d
    }
}

/// One field on which two runs disagree.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Difference {
    pub field: &'static str,
    pub a: String,
    pub b: String,
}

impl std::fmt::Display for Difference {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "{}: {} ≠ {}", self.field, self.a, self.b)
    }
}

/// ★ A failed bring-up carries **the partial report**: which step, in which stage, with everything
/// established up to it. Today a failed bring-up is a bare `FaceError` and a day of bisection.
///
/// ★ Produced by [`run_plan`] on a [`StepClass::Required`] failure. ⚠ The M2
/// `bring_up_*` functions still return `Result<_, FaceError>` and do not yet produce one — no
/// driver has adopted a `Plan` (that is M3+).
#[derive(Debug)]
pub struct BringUpFailure {
    pub report: BringUpReport,
    pub failed_at: &'static str,
    pub source: crate::FaceError,
}

impl std::fmt::Display for BringUpFailure {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(
            f,
            "bring-up failed at {}: {}\n{}",
            self.failed_at,
            self.source,
            self.report.render()
        )
    }
}

impl std::error::Error for BringUpFailure {}

impl BringUpFailure {
    /// A failure that happened **before the first rung** — the device could not be claimed, the
    /// firmware file could not be read, the PID does not dispatch.
    ///
    /// ★ M8. `open_radio` returns `BringUpFailure`, not `FaceError`, so *every* way of failing to
    /// get a radio produces the same shape. Without this, "the plan failed at `rf_config`" and
    /// "there was no dongle on the bus" would be different kinds of answer again, and the second
    /// one would say nothing about which part the caller was even asking for. The report is
    /// [`BringUpReport::synthetic`]: no rung ran, and it says so.
    pub fn not_opened(
        part: &'static str,
        failed_at: &'static str,
        source: crate::FaceError,
    ) -> Self {
        Self {
            report: BringUpReport::synthetic(part),
            failed_at,
            source,
        }
    }

    /// **Print the partial report, then narrow to the error a `FaceError` signature can carry.**
    ///
    /// §3's whole point is that a failed bring-up says how far it got. `FaceError` cannot hold a
    /// report, so any conversion to one must *emit* it first or the account is lost — which is the
    /// old defect one level up. Both channels are used deliberately: `emit()` puts it on the
    /// tracing span tree, and the `eprintln!` is for the bench runs that install no subscriber,
    /// which is most of them.
    ///
    /// This is the M6/M7 `drop_partial_report` / `ath9k_drop_partial_report` pair, generalised —
    /// they were two copies of the same six lines, differing only in the part name, which the
    /// report already carries.
    pub fn into_face_error(self) -> crate::FaceError {
        self.report.emit();
        eprintln!(
            "{} bring-up FAILED at `{}` — the partial report:\n{}",
            self.report.part,
            self.failed_at,
            self.report.render()
        );
        self.source
    }
}

/// So `open_radio(...)?` still works in the many callers whose signature is
/// `Result<_, FaceError>` — **and the partial report is printed on the way through** rather than
/// dropped. See [`BringUpFailure::into_face_error`]: the conversion is deliberately not silent,
/// because a silent one would reintroduce exactly what §3 removes.
impl From<BringUpFailure> for crate::FaceError {
    fn from(f: BringUpFailure) -> Self {
        f.into_face_error()
    }
}

/// The error the caller gets by name, never a silent downgrade.
///
/// ⚠ `BringUpRequest::validate` (contract §1.1) is **not built in this pass**; what does use this
/// today is [`Plan::resolve`](plan::Plan::resolve), which rejects a deviation naming a step that
/// does not exist or an edit with no stated reason.
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum RequestError {
    /// `ProofRequirement::None` with a transmitting role. LAW 4.
    ProofDeclinedWhileTransmitting,
    /// e.g. `Dbm` on a part with `tx_power_dbm: None`. **Named at validation, never a silent pass.**
    Unsatisfiable {
        what: &'static str,
        because: &'static str,
    },
    /// A deviation naming a step that does not exist — the `ndn_env` `Unrecognised` lesson: a
    /// misspelled knob that quietly does nothing is worse than no knob. This is what did **not**
    /// happen when `8307161` changed `lc_calibrate` on 2026-08-31 and sixteen private ladders did
    /// not notice.
    UnknownStep(String),
    /// ★ A deviation, or one of its edits, with no stated reason (§1.6 rule 1). `what` names the
    /// edit. A departure nobody justified is a fork with better spelling.
    MissingWhy { what: String },
}

impl std::fmt::Display for RequestError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::ProofDeclinedWhileTransmitting => {
                write!(
                    f,
                    "ProofRequirement::None is illegal on a transmitting role"
                )
            }
            Self::Unsatisfiable { what, because } => {
                write!(f, "{what} is unsatisfiable: {because}")
            }
            Self::UnknownStep(s) => write!(
                f,
                "no such step: {s} — a deviation must name a rung that exists, so renaming or \
                 deleting a step fails every experiment touching it loudly, at plan construction"
            ),
            Self::MissingWhy { what } => write!(
                f,
                "{what} has no stated reason. A deviation records what question it exists to \
                 answer; without one it is a private fork, which is how sixteen 8812au ladders \
                 came to differ by ~20 dB with nobody able to see it."
            ),
        }
    }
}

impl std::error::Error for RequestError {}

/// The message every power knob uses when the caller asked for the calibrated scale and the part
/// has no calibration resolved. **This string is the fix**: it names the alternative rather than
/// silently becoming it.
pub const NO_CALIBRATION_MSG: &str = "TX power requested on the calibrated scale but no EFUSE calibration is loaded — \
     load_tx_power_info() has not run or failed. This part's uncalibrated write is the RAW TXAGC \
     axis (may exceed licensed EIRP); ask for it explicitly with \
     PowerRequest::Raw + NDN_RF_UNRESTRICTED.";

/// Build the `Unsupported` error a power knob returns when it declines. Kept here so all thirteen
/// impls decline in the same words and a grep finds every one of them.
pub fn power_unsupported(msg: impl AsRef<str>) -> crate::FaceError {
    crate::FaceError::Io(std::io::Error::new(
        std::io::ErrorKind::Unsupported,
        msg.as_ref().to_string(),
    ))
}

#[cfg(test)]
mod tests {
    use super::*;

    fn state(power: AppliedPower) -> RadioState {
        RadioState {
            channel: 6,
            bw: Bandwidth::Bw20,
            format: "RawNdn(0x8624)",
            role: Role::TransmitAndReceive,
            power,
            rate: RateState::new(4, "legacy OFDM 6 Mb/s"),
            warm: None,
            contention: None,
            pump: PumpPolicy::Start(8),
            facts: Vec::new(),
        }
    }

    fn calibrated() -> AppliedPower {
        AppliedPower::from_writes(
            PowerRequest::ceiling(),
            PowerReference::FusedBase {
                base_index: 27,
                channel: 6,
            },
            63,
            false,
            vec![
                PowerWrite {
                    reg: 0xc24,
                    value: 27,
                    group: "OFDM 18-6",
                    path: 0,
                },
                PowerWrite {
                    reg: 0xe24,
                    value: 21,
                    group: "OFDM 18-6",
                    path: 1,
                },
            ],
        )
    }

    fn raw() -> AppliedPower {
        AppliedPower::from_writes(
            PowerRequest::Raw {
                idx: 63,
                authority: RfAuthority {
                    granted_by: "pmle".into(),
                    reason: "bench SDR characterisation".into(),
                },
            },
            PowerReference::ChipRaw,
            63,
            false,
            vec![
                PowerWrite {
                    reg: 0xc24,
                    value: 63,
                    group: "flat",
                    path: 0,
                },
                PowerWrite {
                    reg: 0xe24,
                    value: 63,
                    group: "flat",
                    path: 1,
                },
            ],
        )
    }

    /// The per-rate calibrated write has NO single "index written"; the flat raw one does.
    #[test]
    fn index_written_is_none_when_the_writes_differ() {
        assert_eq!(calibrated().index_written, None);
        assert_eq!(calibrated().index_span, Some((21, 27)));
        assert_eq!(raw().index_written, Some(63));
        assert_eq!(raw().index_span, Some((63, 63)));
    }

    /// Never inferred from an index — no Wi-Fi part in this fleet has a real dBm axis.
    #[test]
    fn dbm_is_none_unless_the_part_has_an_absolute_axis() {
        assert_eq!(calibrated().dbm, None);
        assert_eq!(raw().dbm, None);
        assert_eq!(
            AppliedPower::absolute_dbm(PowerRequest::Dbm(20), 17, true).dbm,
            Some(17)
        );
    }

    /// ★ The 2026-09-03 pair. The two runs agree on the request and differ by ~33 dB; the diff must
    /// say so on its own, without the reader knowing the bug exists.
    #[test]
    fn diff_separates_the_two_2026_09_03_runs() {
        let a = BringUpReport::hand_filled(
            "RTL8812AU",
            PlanId {
                part: "rtl8812au",
                name: "monitor",
                ver: 1,
            },
            DeviceAddress::Usb("1-3.2".into()),
            state(calibrated()),
        );
        let b = BringUpReport::hand_filled(
            "RTL8812AU",
            PlanId {
                part: "rtl8812au",
                name: "monitor",
                ver: 1,
            },
            DeviceAddress::Usb("1-3.2".into()),
            state(raw()),
        );
        let d = a.diff(&b);
        let fields: Vec<&str> = d.iter().map(|x| x.field).collect();
        assert!(fields.contains(&"power.reference"), "{d:?}");
        assert!(fields.contains(&"power.index_span"), "{d:?}");
        assert!(fields.contains(&"plan_digest"), "{d:?}");
        // Same plan, same channel — the differences are exactly the power regime.
        assert!(!fields.contains(&"plan"));
        assert!(!fields.contains(&"channel"));
    }

    /// The digest is what makes a bench number and a production number non-comparable by accident.
    #[test]
    fn digest_tracks_the_power_reference_and_the_rate_group_policy() {
        let base = BringUpReport::hand_filled(
            "RTL8812AU",
            PlanId {
                part: "rtl8812au",
                name: "monitor",
                ver: 1,
            },
            DeviceAddress::Usb("1-3.2".into()),
            state(calibrated()),
        );
        let hot = base.clone().with_power(raw());
        assert_ne!(base.plan_digest, hot.plan_digest);

        let twelve = base.clone().with_power(AppliedPower::from_writes(
            PowerRequest::Index(63, RateGroupPolicy::AllTwelveUnderInvestigation),
            PowerReference::FusedBase {
                base_index: 27,
                channel: 6,
            },
            63,
            false,
            vec![PowerWrite {
                reg: 0xc24,
                value: 27,
                group: "OFDM 18-6",
                path: 0,
            }],
        ));
        assert_ne!(base.plan_digest, twelve.plan_digest);
    }

    /// LAW 6: an `Established(Fact)` reaches `RadioState::facts` without the driver restating it.
    #[test]
    fn established_facts_land_in_radio_state() {
        let r = BringUpReport::hand_filled(
            "RTL8812AU",
            PlanId {
                part: "rtl8812au",
                name: "monitor",
                ver: 1,
            },
            DeviceAddress::Usb("1-3.2".into()),
            state(calibrated()),
        )
        .with_steps(vec![StepRecord::established(
            "load_tx_power_info",
            Stage::Calibrate,
            Fact::PowerReference(PowerReference::FusedBase {
                base_index: 27,
                channel: 6,
            }),
        )]);
        assert_eq!(r.state.facts.len(), 1);
    }

    /// The render carries the reference, the span and the rate-group policy on one line.
    #[test]
    fn render_names_the_regime() {
        let r = BringUpReport::hand_filled(
            "RTL8812AU",
            PlanId {
                part: "rtl8812au",
                name: "monitor",
                ver: 1,
            },
            DeviceAddress::Usb("1-3.2".into()),
            state(calibrated()),
        );
        let s = r.render();
        assert!(s.contains("FusedBase"), "{s}");
        assert!(s.contains("21..27"), "{s}");
        assert!(s.contains("FiveMeasured"), "{s}");
        let hot = r.with_power(raw()).render();
        assert!(hot.contains("ChipRaw"), "{hot}");
        assert!(hot.contains("AUTHORITY pmle"), "{hot}");
    }

    /// `RfAuthority` has no `Default` and no public field: this test is the compile-time claim
    /// written down. It can only be built from the environment or under `feature = "bench"`.
    #[test]
    fn rf_authority_is_not_mintable_from_a_literal() {
        // Inside this crate a struct literal still compiles (private fields are crate-visible);
        // outside it, neither this nor `RfAuthority::default()` exists. The property under test is
        // that there is no *public* constructor other than the two named ones.
        let a = RfAuthority {
            granted_by: "pmle".into(),
            reason: "bench".into(),
        };
        assert_eq!(a.render(), "pmle:\"bench\"");
    }
}
