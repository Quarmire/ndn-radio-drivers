# The bring-up contract

**`ndn-radio-drivers/docs/bringup-contract.md`** — design specification, 2026-09-03.
Companion to `docs/bringup-root-cause-2026-09-03.md` (the measurement that forced it).

> "The messiness of having many ways to bring something up and then forgetting which ones worked and
> the nuances of each is why we are here now."

This document is the contract, not a patch. It is written so that implementing it is transcription.
Where two candidate designs conflicted, one is chosen and the rejected one is named in Appendix A
with the reason — merging them was the failure mode to avoid.

---

## 0. What this settles, and what it is founded on

**The measured defect.** `Rtl8812auBackend::set_tx_power(idx)` (`src/rtl8812au.rs:6479`) means two
different physical powers, decided by whether `load_tx_power_info()` ran three calls earlier:

* calibration loaded → `offset = idx - TXAGC_MAX(63)`, and each rate group gets
  `(index_base(path, rate, ch) + offset).clamp(0, 63)` — ≈ 27 on the ch6 adapter at `idx = 0x3f`;
* not loaded → the function falls through, at `src/rtl8812au.rs:6595`, to
  `set_tx_power_raw(idx)` — a flat 63 on ten registers.

Same call, same argument, same `Ok(())`, ~18–33 dB apart. The sweep at the witness is decisive:
raw 63 → 2301 frames at −85.6 dBm; raw 55 → **0**. The calibrated regime sits 36 steps below max.

**Three things this contract does NOT conclude**, because they are unmeasured:

1. that ≈ 27 is wrong. It is probably the correct regulatory answer. **The default stays calibrated.**
2. that the other parts share the split. They do not, as far as the tree shows: `libusb_rtl88xx`
   (a81a) has a single `set_tx_power` with no calibration branch (`:3488`), and the 8733b *returns*
   its `read_tx_power_info` to the caller instead of stashing it. A sibling driver already meets this
   contract, which is why it is meetable rather than aspirational.
3. that a report catches bugs. It does not. See §7.

**Four defect classes to remove** (from the root-cause doc, §"What this says about the design"):

| # | property | removed by | grade |
|---|---|---|---|
| 1 | API meaning depends on hidden earlier state | §2 — `PowerRequest` in, `AppliedPower` out, fallthrough deleted | **removed, at the type level** |
| 2 | several sequences per part, nuances stored in examples | §1.4 plan + §6 `no_consumer_hand_rolls_a_ladder` | **removed for the sequence; partial for the nuances** |
| 3 | bring-up returns `Ok(())` | §3 `BringUpReport`, and `BringUpFailure` carries the partial one | **removed** |
| 4 | no transmit evidence | §4 `TxProof` / `TxInstrument` / `ProofRequirement` | **removed for (A); NOT removed for (B)** |

---

## 1. The contract

New module `crates/ndn-radio-hal/src/bringup.rs`. It lives in the HAL for the same reason
`OpenRadio` does: a driver *constructs* a report, a face and a bench harness *consume* one, and
neither should need the other to name the types.

### 1.1 What the caller asks for

```rust
/// **Everything a bring-up is allowed to depend on.**
///
/// LAW 1 — *nothing inside a bring-up may read the process environment.* Every `NDN_*` a driver
/// reads today inside a ladder (`NDN_8733B_RX_ONLY`, `NDN_NO_PUMP`, `NDN_ATH9K_PUMP`, `NDN_TX_PWR`,
/// `NDN_RADIO_BW`, `NDN_CCA_OFF`, `NDN_RADIO_SKIP_CAL`, `NDN_RADIO_NO_EFEM`, `NDN_AU_TXAGC12`)
/// becomes a field here or a `Deviation` (§1.6). Configuration a caller cannot see is the same
/// defect as `load_tx_power_info`, one level up. `from_env()` is the ONE place that reads them.
#[derive(Clone, Debug)]
#[non_exhaustive]
pub struct BringUpRequest {
    pub channel: u8,
    pub bw: Bandwidth,
    pub format: FrameFormat,
    /// Replaces `NDN_8733B_RX_ONLY`, the ath9k `rx_enable` second path, and `bring_up_monitor`
    /// vs `bring_up_tx` as *separate functions*.
    pub role: Role,
    /// ★ The regulatory decision, named at the call site. No `Default` — see §2.
    pub power: PowerRequest,
    /// What must be PROVEN before the handle is returned (§4).
    pub proof: ProofRequirement,
    /// Replaces the four different pump owners.
    pub pump: PumpPolicy,
    /// `None` on USB means "inherit whatever the last process left" — MEASURED as a 2.5x
    /// throughput swing on the MT7610U decided by run order. Production should pin.
    pub contention: Option<ContentionPosture>,
    /// A named, self-labelling departure from the canonical plan (§1.6). `None` = canonical.
    pub deviation: Option<Deviation>,
}

impl BringUpRequest {
    /// The only environment read in the crate. Registered names only; an unknown `NDN_*` is
    /// already reported by `ndn_env` as `Class::Unrecognised`.
    pub fn from_env(channel: u8) -> Self;
    /// Validation, run by `run_plan` BEFORE the first register write. Returns the caller's error
    /// by name — never a silent downgrade.
    pub fn validate(&self, cap: &RadioCapability) -> Result<(), RequestError>;
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Role { ReceiveOnly, TransmitOnly, TransmitAndReceive }

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum PumpPolicy { Start(usize), CallerOwns, None }

#[derive(Clone, Debug, PartialEq, Eq)]
pub enum RequestError {
    /// `ProofRequirement::None` with a transmitting role. See LAW 4.
    ProofDeclinedWhileTransmitting,
    /// e.g. `MacKeyedOrFail` on a part whose `tx_instruments()` is empty; or `Dbm` on a part with
    /// `tx_power_dbm: None`. **Named at validation, never a silent pass.**
    Unsatisfiable { what: &'static str, because: &'static str },
    /// A `Deviation` naming a step that does not exist — the `ndn-env` `Unrecognised` lesson:
    /// a misspelled knob that quietly does nothing is worse than no knob.
    UnknownStep(String),
}
```

### 1.2 Power types

Full semantics in §2; the types belong here.

```rust
/// **What the caller wants, in words that have exactly one physical meaning per part.**
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum PowerRequest {
    /// "As loud as this part will legally go." The top of the part's own declared scale:
    /// `RadioCapability::max_tx_power`, or the top of `tx_power_dbm`. On a fused part this IS
    /// the regulatory base — those are the same point, and the API does not pretend otherwise.
    Ceiling,
    /// An index on the declared scale, i.e. what `RadioPolicy::decide_power` returns. Clamped
    /// into `[min_tx_power, max_tx_power]`; the clamp is REPORTED (`AppliedPower::clamped`).
    Index(u8),
    /// Absolute dBm. Requires `RadioCapability::tx_power_dbm.is_some()`, else `Unsatisfiable`.
    Dbm(i8),
    /// ⚠ **Off the regulatory scale.** Raw chip TXAGC / calibration bypassed. Not constructible
    /// without an `RfAuthority`, which no library code can mint (§2.4).
    Raw { idx: u8, authority: RfAuthority },
    /// The part has no power actuator (`power_actuated == false`: MT7612U, MT7921AU). Explicit,
    /// so "we did not set power" and "this part has no power knob" are different states.
    NoActuator,
}

/// **What the actuator actually did.** Returned by the knob; carried in the report.
#[derive(Clone, Debug, PartialEq)]
pub struct AppliedPower {
    pub requested: PowerRequest,
    pub reference: PowerReference,
    /// The API-scale index used after clamping. NOT a register value.
    pub index_requested: u8,
    /// `true` if `min_tx_power`/a driver clamp moved the request (a81a clamps to `20..=63`,
    /// `libusb_rtl88xx.rs:3488`, because below ~20 the gain chain inverts ~11 dB ABOVE the
    /// calibrated maximum — MEASURED on a B210, three scrambled passes, ±0.1 dB).
    pub clamped: bool,
    /// Every register actually written, in order. ☠ On the calibrated 8812au these values
    /// DIFFER per rate group (`index_base(path, rate, ch) + offset`), so there is no single
    /// "index written" and the report must not invent one.
    pub writes: Vec<PowerWrite>,
    /// `Some` only when every write carries the same value (the raw/flat case). `None`
    /// otherwise — read `writes`/`index_span`.
    pub index_written: Option<u8>,
    /// `(min, max)` over `writes`. `None` when `writes` is empty.
    pub index_span: Option<(u8, u8)>,
    /// Absolute power ONLY where the part has a real dBm axis. Never inferred from an index:
    /// `RadioCapability::tx_power_dbm`'s own doc — "an invented figure is worse than `None`".
    pub dbm: Option<i8>,
    /// `false` = the write did not reach silicon.
    pub actuated: bool,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct PowerWrite { pub reg: u32, pub value: u8, pub group: &'static str, pub path: u8 }

/// **What the number written is referenced to.** This is the field the 2026-09-03 day of
/// bisection was missing.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum PowerReference {
    /// EFUSE per-channel per-rate calibration: the adapter's own regulatory point.
    FusedBase { base_index: u8, channel: u8 },
    /// A driver/vendor TXAGC reference with no per-adapter fuse read — the a81a's `0x18e8`
    /// reference index, the 8733b's TSSI DE. Monotone and characterised, but not fused.
    /// `slope_db_per_idx` is `Some` only where MEASURED (a81a: ~0.22 dB/step over 20..=63).
    DriverReference { source: &'static str, slope_db_per_idx: Option<f32> },
    /// ⚠ Raw chip TXAGC, calibration bypassed. May exceed licensed EIRP. Requires `RfAuthority`.
    ChipRaw,
    /// A real absolute axis (nl80211, Morse, NRC, LoRa).
    AbsoluteDbm,
    /// No actuator. `writes` is empty and `actuated` is false.
    NoActuator,
}
```

**There is deliberately no `Uncalibrated` reference.** The 8812au's uncalibrated path *is* the raw
path — `set_tx_power`'s tail literally calls `set_tx_power_raw`. Giving it a third name would
re-create the ambiguity with better spelling. It becomes `ChipRaw`, and therefore requires authority
(§2.3).

### 1.3 The knob

```rust
// crates/ndn-radio-hal/src/lib.rs — RadioKnobs
// was: fn set_tx_power(&self, _idx: u32) -> Result<(), FaceError>          (:1053)
fn set_tx_power(&self, req: PowerRequest) -> Result<AppliedPower, FaceError>;
// unchanged, still the alternative for parts with a dBm axis:
fn set_tx_power_dbm(&self, dbm: i8) -> Result<i8, FaceError>;               (:1071)
```

Return-type-only changes are source-compatible at `x.set_tx_power(p)?;` and `let _ = …` sites, which
is nearly all of the 225 references across 63 files. The argument change is mechanical
(`idx` → `PowerRequest::Index(idx as u8)`), and it is deliberate: a `u32` index with no frame is the
thing being abolished. **Eleven of thirteen `RadioKnobs` impls take the `Unsupported` default and do
not change at all.**

The same change is applied to the *inherent* methods, because that is where the ambiguity physically
lives: `Rtl8812auBackend::set_tx_power` (`rtl8812au.rs:6479`),
`LibUsbRtl88xxBackend::set_tx_power` (`:3488`), `Rtl8733buBackend::set_tx_power_idx` (`:3645`),
`SerialBackend::set_tx_power_pct` (`serial_radio.rs:543`), `mt76x0::phy::set_tx_power*`.

**LAW 2 — no power knob may fall through to another regime.** `Rtl8812auBackend::set_tx_power`'s
tail (`rtl8812au.rs:6595`) is deleted. With no calibration loaded and `PowerRequest::Ceiling` or
`Index`, it returns:

```rust
Err(FaceError::Unsupported(
    "TX power requested on the calibrated scale but no EFUSE calibration is loaded — \
     load_tx_power_info() has not run or failed. This part's uncalibrated write is the RAW \
     TXAGC axis (~18-33 dB hotter, may exceed licensed EIRP); ask for it explicitly with \
     PowerRequest::Raw + NDN_RF_UNRESTRICTED."))
```

This one deletion is the fix. Everything else in this document exists so it cannot come back in a
different shape.

**LAW 3 — a knob may not branch on the environment.** `set_tx_power`'s `NDN_AU_TXAGC12` read
(`rtl8812au.rs:6556`) is a *second* live hidden-state fork inside the very function being fixed: it
silently changes 10 register writes into 24. It becomes a request field with a written default:

```rust
/// ☠ MEASURED against a witness: 12 groups = 7 frames on air in 11 s (1 f/s); 5 groups = 2704
/// frames (246 f/s). The ADDRESSES are not in doubt (`PROGS_5G` writes all 24; the mainline
/// kernel writes per-rate ladders into them on this dongle; `Hal8812PhyReg.h:178`); the VALUE
/// `index_base(..)+offset` computes for the 2SS/VHT rate codes is. `AllTwelve` is an open
/// investigation and requires a witness receiver in the loop.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Default)]
pub enum RateGroupPolicy { #[default] FiveMeasured, AllTwelveUnderInvestigation }
```

carried on `PowerRequest`'s part-specific extension (`Rtl8812auPowerOpts`, a field of
`BringUpRequest::part_opts: PartOpts`) and echoed in `AppliedPower::writes` — where 10 entries vs 24
is visible without knowing the flag exists.

### 1.4 The plan

```rust
/// One rung. A name, a class, the reason it is there, and the code.
pub struct Step<B: 'static> {
    /// Stable id. Used by the report, by `Deviation`, by `--skip`, and by the digest.
    pub id: &'static str,
    /// A rendering/`--stop-after` label ONLY. It is not an enforcement device — see Appendix A.2.
    pub stage: Stage,
    pub class: StepClass,
    /// ★ The measurement or vendor reference that puts it here. **An empty `why` fails the gate.**
    pub why: &'static str,
    /// Ordering the sequence alone does not explain, checked at plan construction, so
    /// "TSSI before enable_tx" (19.6 dB vs 1.6 dB of range) stops being prose.
    pub must_follow: &'static [&'static str],
    pub must_precede: &'static [&'static str],
    /// ★ `&Arc<B>`, not `&B`: `Rtl8733buBackend::bring_up_tx_tracked` is
    /// `self: &Arc<Self> -> Result<PowerTracker>` (`libusb_rtl8733b.rs:1488`) and the tracker is a
    /// live thread guard. A step may return one; the runner keeps it (see `StepOutcome::Guard`).
    pub run: fn(&Arc<B>, &mut Ctx) -> Result<StepOutcome, FaceError>,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord)]
pub enum Stage { Attach, PowerOn, Firmware, MacInit, PhyInit, Tune, Calibrate,
                 TxEnable, RxEnable, Power, Posture, Verify }

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum StepClass {
    /// Failure aborts. **LAW 5: a step that polls a hardware completion bit is always `Required`**
    /// (LLT/AUTO_LLT, firmware `WINTINI_RDY`, MCU round-trip). Enforced in §6.
    Required,
    /// Failure warns and continues, and MUST name what is lost. Replaces every bare `let _ =` in
    /// today's ladders, each of which discards a real degradation silently.
    BestEffort { degrades: &'static str },
    /// A readback asserting an invariant the steps above should have established (§1.5).
    Assert,
    /// The work happened OUT OF BAND (modprobe / `iw` / `hostapd_s1g` / `morse_cli`) and this step
    /// only validates it. Honest for HaLow; a lie anywhere else.
    OutOfBand { established_by: &'static str },
}

pub enum StepOutcome {
    Done,
    /// A step that branched internally says which way. Branching BETWEEN steps is not expressible;
    /// a warm/cold decision lives INSIDE one named step and reports here.
    Branch(&'static str),
    /// A fact that changes what a later API means. **LAW 6: every such step must return one, and
    /// every `Fact` must land in `RadioState`.** `load_tx_power_info` is the founding case.
    Established(Fact),
    /// A live guard the handle must own (the 8733b `PowerTracker`). Moved into `OpenRadio`.
    Guard(Box<dyn std::any::Any + Send + Sync>),
    Skipped(&'static str),
}

#[derive(Clone, Debug, PartialEq)]
pub enum Fact {
    PowerReference(PowerReference),
    Warm(bool),
    /// e.g. the ath9k high-power gain table selected from `eeprom_tx_gain_type`.
    GainTable(&'static str),
    Firmware { name: &'static str, ready: bool },
}

pub struct Plan<B: 'static> {
    pub id: PlanId,                       // { part: &'static str, name: &'static str, ver: u16 }
    pub role: Role,
    pub steps: &'static [Step<B>],
    /// Stages this part deliberately does nothing in, with the ruling. Same discipline as
    /// `coverage::Seam::Excluded`: a blank cell is a written decision, not an absence.
    pub excluded: &'static [(Stage, &'static str)],
}

/// Implemented once per part. **Deliberately not object-safe**: the factory dispatches on PID to a
/// concrete type, and `&'static Plan<Self>` buys hardware-free inspection of the sequence.
pub trait BringUp: Sized + Send + Sync + 'static {
    fn plan(role: Role) -> Option<&'static Plan<Self>>;
    fn asserts() -> &'static [Assert<Self>];
    fn tx_instruments() -> &'static [TxInstrument];
    fn bring_up(self: &Arc<Self>, req: &BringUpRequest)
        -> Result<(BringUpReport, Guards), BringUpFailure>
    {
        run_plan(self, Self::plan(req.role).ok_or(/* Unsatisfiable */)?, req)
    }
}

pub fn run_plan<B: BringUp>(b: &Arc<B>, plan: &'static Plan<B>, req: &BringUpRequest)
    -> Result<(BringUpReport, Guards), BringUpFailure>;
```

`run_plan` is ~150 lines with no per-part knowledge: validate the request, walk the steps applying
the deviation, record every outcome and elapsed time, run the asserts, run the TX probe, build the
report, `emit()` it.

**Steps may be subtracted from outside the driver crate. They may never be added.** There is no
`Plan::then()`, no `push`, no `append`. Adding a rung means editing the `static` — where a reviewer
sees it, and where the `why` is required.

### 1.5 Asserts — read back every gate you write

```rust
pub struct Assert<B: 'static> {
    pub id: &'static str,
    pub reg: u32,
    pub read: fn(&B) -> Result<u32, FaceError>,
    pub want: u32,
    pub mask: u32,
    pub why: &'static str,
    /// `Warn` on introduction for every part; promoted to `Fatal` per part only with a
    /// measurement. See §5/M-hazards — this rule is not optional.
    pub severity: Severity,
}
```

This is the half of the contract that reads the hardware back rather than describing it. The 8812au
set is the full `iqk_configure_mac` quiesce list (`rtl8812au.rs:5454`), not just TXPAUSE — it drops
five things and restores them only in `iq_calibrate`'s tail block, which the `iqk_tx()?` error path
skips:

| id | reg | want | why (abridged) |
|---|---|---|---|
| `txpause_released` | `0x0522` | `0x00` | `0x3f` = aborted-IQK residue, `0xff` = aborted-LCK (`rf_ab.rs:555`). **MEASURED 0x00/0x00/0x00 over three runs — structurally real, not firing today.** Assert it; do not add a `write8`, which was A/B'd inert (2789/4508/4588/4406 = noise) and reverted. |
| `mac_tx_rx_enabled` | `0x0100` | `MACTXEN\|MACRXEN` | `read_cr():6056` exists and has one caller, an example. |
| `rx_antenna_restored` | `0x0808` | saved | dropped by `iqk_configure_mac`, restored only in the tail block |
| `cca_restored` | `0x0838` | saved | ditto |
| `cck_rx_restored` | `0x0a07` | saved | ditto |

The identical `txpause_released` assert, one line, catches a **live** defect on a different driver:
`libusb_rtl88xx::txgapk_tx_pause():3962` writes `0x0522 = 0xff` and never resumes — the restore sits
at the tail of `txgapk`, whose error `bring_up:558` swallows with `tracing::warn!`.

### 1.6 Deviation — how a bench experiment departs, without forking

Deviation is *supported*, precisely so that experiments stop being implemented as a second bring-up.
Five of the sixteen hand-rolled 8812au files exist only to deviate, and they are how everything in
this crate got measured.

```rust
#[derive(Clone, Debug)]
pub struct Deviation {
    /// ★ REQUIRED, not `Option`. What question this exists to answer. Rendered as a banner.
    pub question: String,
    pub ops: Vec<DeviationOp>,
}

/// **Erased ops only** — these cross the `open_radio(pid, &req)` boundary, where the backend type
/// is not known. (A typed `InsertAfter(Step<B>)` cannot: it would make `BringUpRequest` generic
/// over `B` and destroy the runtime PID dispatch. Typed insertion lives in `bench`, below.)
#[derive(Clone, Debug)]
pub enum DeviationOp {
    /// The exact operation that found the 2026-09-03 defect. Three arms plus a confirm.
    Skip(String),
    StopAfter(Stage),
    /// Raw register poke after the plan completes: `pwr_sweep8812au`'s `0xc24/0xc28/0xe24/0xe28`,
    /// `reg_probe`'s argv pokes, `hwbeacon8733b`'s `0x0420/0x0100`.
    Poke { addr: u32, val: u32, width: u8 },
    /// ⚠ The regulatory override, recorded (§2.4).
    RegulatoryOverride { authority: RfAuthority },
}

/// Typed, arbitrary steps — everything the erased set cannot express (`schedtx8733b`'s ~60
/// comparator writes with backend methods, `single_tone`/`single_carrier`). Compiled only under
/// `feature = "bench"`, `required-features` on the examples that use it.
#[cfg(feature = "bench")]
pub mod bench {
    pub fn run_with<B: BringUp>(b: &Arc<B>, req: &BringUpRequest,
                                extra: &[(&'static str, Step<B>)], question: &str)
        -> Result<(BringUpReport, Guards), BringUpFailure>;
    /// Full register access for an already-open radio. Unrestricted; that is the point.
    pub fn regs<B: BringUp>(b: &Arc<B>) -> RegAccess<'_, B>;
}
```

Four rules make this a deviation rather than a fork with extra steps:

1. **`question` is mandatory.** A deviation with no stated reason does not compile.
2. **It lands in `report.provenance` and changes `plan_digest`.** A bench number and a production
   number can never be silently compared: the digests differ.
3. **Ops name steps by id.** Rename or delete a step and every experiment touching it fails *loudly
   at plan construction* — which is exactly what did not happen when `8307161` changed
   `lc_calibrate` on 2026-08-31 and sixteen private ladders did not notice.
4. **It is constructible from the environment**: `NDN_BRINGUP_DEVIATE="skip:load_tx_power_info"`,
   registered in `ndn_env` as `Class::DebugBisect`, which already prints it in the run header and
   already flags it as a confounder. A three-arm bisect becomes a shell loop, not three new files.

### 1.7 One factory, one handle

```rust
/// Replaces `open_named_radio`, `open_ath9k`, `Bw16SerialBackend::open_radio`,
/// `Esp32SerialBackend::open_c5_radio`, the LoRa opener that lives inside an example,
/// `RadioMediumFaceFactory::build_bearer`'s arms, `ndn-fwd::build_rtl8812au`/`build_rtl8822e`,
/// and the five a81a openers.
pub fn open_radio(pid: u16, sel: &DeviceSelect, req: &BringUpRequest)
    -> Result<OpenRadio, BringUpFailure>;

pub struct OpenRadio {
    pub io: Arc<dyn FrameIo>,
    pub knobs: Option<Arc<dyn RadioKnobs>>,
    pub time: Option<Arc<dyn RadioTime>>,
    pub profile: Option<Arc<dyn RadioProfile>>,
    /// ★ NOT an `Option`. A handle with no account of how it was brought up is the thing this
    /// contract removes. Fields become private; `OpenRadio::synthetic(report)` is the only other
    /// constructor (loopback/sim).
    report: BringUpReport,
    /// Live guards the plan produced (8733b `PowerTracker`). Dropped with the handle.
    guards: Guards,
}
impl OpenRadio { pub fn report(&self) -> &BringUpReport; /* accessors for io/knobs/time/profile */ }
```

---

## 2. Power semantics, settled

### 2.1 The scale is not renumbered. Here is why.

`Rtl8812auBackend::set_tx_power` computes `offset = idx - TXAGC_MAX(63)` and writes
`index_base(path, rate, ch) + offset`. Therefore **on the calibrated API scale, `idx = 63` *is* the
fused regulatory base.** `RadioCapability::max_tx_power: 63` (`hal/src/lib.rs:2378`, documented "the
calibrated/regulatory ceiling") and the driver already agree.

A tempting "fix" is to set `max_tx_power = 27` so the capability reports the fused base. **Rejected,
and it must stay rejected**: `RadioPolicy::decide_power` computes `max_tx_power - backoff` and clamps
into `[min, max]`, so it would emit ~27, which the driver renders as `base + (27 − 63) = base − 36`
→ `clamp(0)` → a silent near-zero-power radio. That is one number meaning two things on two scales —
the exact defect being abolished — introduced by the cure. The *physical* point is reported in
`PowerReference::FusedBase { base_index, channel }`, which is a fact, not a renumbering.

### 2.2 The three questions a caller can ask

| the caller means | writes | on the 8812au (ch6) | who asks |
|---|---|---|---|
| "as loud as this part will legally go" | `PowerRequest::Ceiling` | calibrated scale index 63 → per-rate `index_base + 0` ≈ **27 in silicon** | production bring-up; `wireless_node` |
| "the regulatory base" | **the same request.** On a fused part these are one point, and the API does not invent a distinction | ≈ 27 | — |
| "back off N steps from legal max" | `PowerRequest::Index(i)` from `RadioPolicy::decide_power` | `index_base + (i − 63)`, clamped 0..63 | cognition |
| "absolute power" | `PowerRequest::Dbm(d)` | `Unsatisfiable` — this part has no dBm axis | HaLow / LoRa / nl80211 parts |
| "chip maximum, off the regulatory scale" | `PowerRequest::Raw { idx, authority }` | flat 63 on ten registers, **≈ +18–33 dB** | bench, with a written justification |

On a part whose ceiling is a `DriverReference` rather than a fuse (a81a, 8733b), `Ceiling` means the
top of the characterised monotone region — for the a81a, index 63 with `min_tx_power = 20`, because
below 20 that chip's gain chain inverts ~11 dB *above* its calibrated maximum (measured on a B210).
The `clamped` flag says when the request was moved.

### 2.3 Who decides what

* **The driver declares what the axis IS** — `max_tx_power`, `min_tx_power`, `db_per_power_idx`,
  `power_actuated`, `tx_power_dbm`, and now the resolved `PowerReference`. It may never choose to
  leave the frame.
* **Cognition chooses the point on the axis.** `RadioPolicy::decide_power(cap, mcs, rssi) -> Option<u8>`
  is **untouched** — not wrapped, not duplicated. It already clamps to `[min, max]` and already
  returns `None` rather than guessing when `db_per_power_idx` is unmeasured.
* **Only the operator chooses the axis**, and only in writing.

### 2.4 `RfAuthority`

```rust
/// ⚠ An explicit operator decision to transmit off the regulatory scale. Carries who and why.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct RfAuthority { pub granted_by: String, pub reason: String }

impl RfAuthority {
    /// `NDN_RF_UNRESTRICTED="<operator>:<reason>"` (registered `Class::DebugBisect`), or
    /// `bench::authority()` under `feature = "bench"`. **There is no other constructor**, so no
    /// library code — and in particular no `RadioPolicy` — can reach the raw axis. The string is
    /// printed verbatim in every report that used it.
    pub fn from_env() -> Option<Self>;
}
```

The regulatory ceiling is enforced by the type system rather than by discipline. `nav_probe`'s hot
run would still be *allowed* — it was a legitimate bench experiment — and would have been visibly a
regulatory deviation from its first line of output.

### 2.5 How it appears

Four places, none optional:

1. `report.state.power` — reference, index requested, clamp, every register written, span, dBm-or-None.
2. The `ndn-env` run header, which already prints classified variables: `NDN_RF_UNRESTRICTED` and
   `NDN_BRINGUP_DEVIATE` are both there, both flagged as confounders.
3. A `tracing::warn!` at bring-up whenever `reference == ChipRaw`.
4. `BringUpReport::diff` — the operation you reflexively perform when one run works and one does not.

---

## 3. The report

```rust
#[derive(Clone, Debug)]
#[non_exhaustive]
pub struct BringUpReport {
    pub part: &'static str,
    pub device: DeviceAddress,
    pub plan: PlanId,
    /// FNV over the ordered (step id, ran|skipped) sequence ACTUALLY executed, plus the power
    /// reference. ★ The answer to "which bring-up did you use?" — the question with no answer
    /// today. Record it beside every on-air number.
    pub plan_digest: u64,
    pub provenance: Provenance,
    pub steps: Vec<StepRecord>,          // { id, stage, class, outcome, elapsed_us }
    pub asserts: Vec<AssertRecord>,      // { id, reg, read, want, ok, severity }
    pub state: RadioState,
    pub tx: TxProof,
    pub capability: RadioCapability,     // post-bring-up, for the PHY actually configured
    pub warnings: Vec<Warning>,
    pub started_at: SystemTime,
    pub elapsed: Duration,
}

#[derive(Clone, Debug)]
pub struct RadioState {
    pub channel: u8,
    pub bw: Bandwidth,
    pub format: FrameFormat,
    pub role: Role,
    pub power: AppliedPower,                    // ★ §2
    pub rate: RateState,                        // the DESC/TXWI code AND what it decodes to
    pub warm: Option<bool>,                     // Option: only mt76 establishes this
    pub contention: Option<ContentionApplied>,  // Option: not every part actuates EDCA
    pub pump: PumpPolicy,
    pub facts: Vec<Fact>,                       // LAW 6 — every Established(Fact) lands here
}

#[derive(Clone, Debug)]
pub enum Provenance {
    Canonical,
    /// ★ Self-labelling: any measurement taken through a deviated bring-up carries its own
    /// asterisk, forever, in its own output.
    Deviated { question: String, ops: Vec<DeviationOp> },
}

pub struct BringUpFailure {
    /// ★ The PARTIAL report: which step, in which stage, with everything established up to it.
    /// Today a failed bring-up is a bare `FaceError` and a day of bisection.
    pub report: BringUpReport,
    pub failed_at: &'static str,
    pub source: FaceError,
}

impl BringUpReport {
    pub fn render(&self) -> String;                       // the block printed at INFO on every open
    pub fn emit(&self);                                   // structured `tracing` fields → OTLP
    pub fn diff(&self, other: &Self) -> Vec<Difference>;  // ★ detection-by-contrast
}
```

**Which fields are `Option`, and why** — an `Option` here means *this part genuinely cannot answer*,
never *nobody wired it up*:

| field | `Option`? | reason |
|---|---|---|
| `state.power.dbm` | yes | only parts with a real dBm axis (Morse, NRC, LoRa, nl80211). Inventing one is worse than `None`. |
| `state.power.index_written` | yes | `None` whenever the writes differ per rate group — the calibrated 8812au. `index_span` + `writes` carry the truth. |
| `state.warm` | yes | only the mt76 family establishes warm/cold; asserting `false` elsewhere would be a claim. |
| `state.contention` | yes | not every part actuates EDCA; `None` = inherited/unknown, which on USB is a real and hazardous state. |
| `state.rate.dbm_equivalent`, `capability.db_per_power_idx` | yes | unchanged HAL policy: unmeasured stays `None`. |
| `tx` | **no** | `TxProof` always has a value — including `Unprovable { reason }`. Silence is the defect. |
| `report` on `OpenRadio` | **no** | the point of the exercise. |
| `plan_digest`, `provenance`, `steps`, `asserts` | **no** | every part can produce these; they are host-side. |

**Rendered, this is what the two 2026-09-03 runs look like** (the whole design, in eight lines):

```
radio RTL8812AU 1-3.2  plan rtl8812au/monitor@v1  digest 0x9f13c02a  CANONICAL   3.31 s
  ch 6 / Bw20   format RawNdn(0x8624)   role TxRx   pump 8
  power  ref=FusedBase{base:27, ch:6}  req=Ceiling(63)  clamped=no  dbm=None  actuated
         writes 10 regs (5 groups x 2 paths)  idx span 21..27   [RateGroupPolicy::FiveMeasured]
  rate   DESC_RATE_6M (legacy OFDM 6 Mb/s)
  assert 0x0522=0x00 ok · 0x0100=0x3c ok · 0x0808 ok · 0x0838 ok · 0x0a07 ok
  tx     UNPROVABLE — no MAC->BB counter ported for Jaguar1; CCX is lossy (1-4 records per
         ~2300 armed) and needs a running RX pump. Prove with a witness.

radio RTL8812AU 1-3.2  plan rtl8812au/monitor@v1  digest 0x41ba7d68  DEVIATED    3.28 s
  deviation "does the fused base cost us the link?"  ops: skip:load_tx_power_info
  power  ref=ChipRaw  req=Raw(63)  ⚠ AUTHORITY pmle:"bench SDR characterisation"
         writes 10 regs  idx span 63..63
```

`ref=FusedBase / span 21..27` against `ref=ChipRaw / span 63..63` is the top line of a `diff`. No
reader needs to know the bug exists to see that two runs disagree.

---

## 4. Transmit evidence

**(A) and (B) are different questions, and only (A) is answerable on-chip.**

> `ath9k_htc.rs:2655`, MEASURED: without `WMI_TARGET_IC_UPDATE` the descriptor chain-select is 0 and
> *"the MAC keys the transmitter (TFCNT advances, TXOK completes) but nothing coherent radiates — a
> witness at inches decoded 0 of our frames."*

A part that can only answer (A) must not be able to spell (B).

```rust
#[derive(Clone, Debug)]
pub enum TxProof {
    NotRequested,                                                     // Role::ReceiveOnly
    MacKeyed { instrument: TxInstrument, probes: u16, delta: u16, idle_control: Option<u16> },
    /// (A) declined, WITH THE REASON. A first-class success value, not an omission.
    Unprovable { instrument: TxInstrument, reason: &'static str },
    /// (B). The only real answer, and only a peer can mint it.
    WitnessDecoded { witness: WitnessId, sent: u32, heard: u32, rssi_dbm: Option<i8> },
    /// The instrument ran and said NO. Fatal except under `ProofRequirement::None`.
    Refuted { instrument: TxInstrument, detail: String },
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub enum ProofRequirement {
    /// ★ LAW 4: legal ONLY with `Role::ReceiveOnly`; `validate()` rejects it otherwise.
    /// "I did not check" is how this stack shipped a 20 dB deficit and must not be expressible
    /// on a radio that intends to transmit.
    None,
    /// Probe if an instrument exists; `Unprovable` otherwise and bring-up SUCCEEDS saying so.
    /// **Production default** — a radio must not refuse to come up because its silicon is blind.
    BestAvailable,
    /// The counter must move by exactly the probe count, or bring-up fails. Requesting it of a
    /// part with no instrument is `RequestError::Unsatisfiable`, never a silent pass.
    MacKeyedOrFail,
    /// A named witness must decode the probes, or bring-up fails. The only setting proving (B).
    /// This is `Rtl8733buBackend::bring_up_tx_until(verify)` — which already exists, on one part,
    /// unused by the factory — generalised and given a home.
    WitnessOrFail { witness: WitnessId, min_heard: u32 },
}
```

### Per part: what can be proven, what cannot

| part | instrument | cost | proves | contract's answer |
|---|---|---|---|---|
| **RTL8733BU** | `read_tx_counters` → `0x2de0/0x2de2` (`libusb_rtl8733b.rs:4374`). **MEASURED +50 across 50 injects** | 2 control reads | (A) | `MacKeyed`. ⚠ **OFDM pair only** — CCK lives at `0x2de4/0x2de6`, unimplemented; a CCK frame reads +0. Recorded in the instrument's doc. |
| **Morse MM6108** | `morse_cli stats -m -s` `TX Total`. MEASURED +200/200, **with a +0 idle control** | ~10.6 ms | (A) | `MacKeyed`. Best-calibrated counter in the fleet and today **it exists only inside a doc comment** — `tx_instruments()` forces it into being a function. ☠ Never use `/sys/class/net/*/statistics` here (measured to lie in both directions). |
| **LoRa serial** | `EVT_TXDONE(ok=1)`, already per-frame inside `inject` | free | (A) | `MacKeyed`. The model: a CAD/LBT deferral surfaces as a *failure*, not a swallow. |
| **MT7921AU** | `mt_mib_sdr14`/`sdr15` (`connac2/regs.rs:815`) | 2 reads | (A) | `MacKeyed`. ⚠ **UNMEASURED** whether these sit behind the `mt_mib_scr1` duration gate — the instrument carries that warning and the first implementation must check it before trusting a zero. |
| **MT7610U / MT7612U** | `MT_TX_STAT_FIFO` 0x1718 (transcribed, zero readers) | 1 read per pop | (A) | `MacKeyed`. ⚠ read-and-clear, and a bound kernel `mt76x0u` steals roughly half. |
| **NRC7292** | `show mac tx stats` (`nrc7292.rs:858`) | **≥ 300 ms** | (A) | `Unprovable("≥300 ms — too slow for bring-up")` at open; probe on demand. |
| **AR9271** | HTC credit reclaim past the ~34-frame ring | free | (A), **and measured insufficient for (B) on this very part** | `Unprovable`, with the counterexample as the reason. This is the fleet's own discipline and it is kept in the enum. |
| **RTL8812AU** | none ported. CCX/`SPE_RPT` exists but is lossy (1–4 records per ~2300 armed) and needs a running RX pump — unavailable under `Role::TransmitOnly` | — | — | `Unprovable("no Jaguar1 MAC→BB counter ported; CCX lossy + needs RX pump")`. `REG_TXPKT_EMPTY 0x041A` is a candidate (mainline reads it on this dongle) with **unmeasured** semantics — a written next step, not a claim. |
| **RTL8822E / a81a** | ruled out by measurement: `libusb_rtl88xx.rs:30` — *"`0x2DE0` is NOT a TX-OK counter"* | — | — | `Unprovable`, reason quoted. ☠ Fix in passing: `examples/tx_liveness.rs:53` prints `0x2d08` (an **RX** false-alarm counter) as "TX activity". |
| **RTL8821CU** | no `RadioKnobs` impl at all; sets `SPE_RPT` on every frame and discards the C2H | — | — | `Unprovable`. Decided-but-unactuated, textbook. |
| **ESP32-C5 / BW16** | none — no TX counter exists in the 7E-A5 protocol | — | — | `Unprovable("needs a firmware opcode, not a driver change")`. |
| **AF_PACKET** | `sendto` proves the kernel took an skb | — | nothing | `Unprovable`. `morse_mon_xmit` (`dev_kfree_skb`; `NETDEV_TX_OK`) is the standing proof that this is worthless — and `MorseFrameIo::new`'s refusal of that netdev at construction is the one place in the tree that *prevents* rather than detects this class. |

**(B) is not removed, on purpose.** `WitnessDecoded` is mintable only from a peer report. A bring-up
on a single host cannot prove radiation, and a design that claims otherwise is lying. What the
contract removes is the *silence*, and the ability to intend transmission while declining to look.

---

## 5. Migration

Ordered so that **the fix ships before the machinery**, and the part currently known to radiate is
migrated last. The workspace is mid-extraction (`refactor/ndn-radio` phase 3b moves
`ndn-phy-wifi`); M4–M6 touch that crate and must be sequenced with it.

| | work | ships | what can break |
|---|---|---|---|
| **M0** | `ndn-radio-hal::bringup` types, `render`/`emit`/`diff`. Nothing calls it. | day 1 | nothing |
| **M1** | ★ **The bug fix, standalone.** `PowerRequest`/`AppliedPower`/`PowerReference`/`RfAuthority`; `RadioKnobs::set_tx_power -> AppliedPower` (13 impls, 11 unchanged defaults) and the inherent methods; **delete the fallthrough at `rtl8812au.rs:6595`**; `NDN_AU_TXAGC12` → `RateGroupPolicy`. | day 2 | `.map(|()| …)` sites (rare) break; `let _ =` sites now discard something they should log — grep and fix. **Any A/B spanning M1 is invalid**: cognition's `apply_knobs` starts recording *applied* rather than *requested*. Re-baseline. |
| **M2** | `BringUpReport` returned by the *existing* `bring_up_*` functions, hand-filled. **No sequence moves.** Callers get the regime object before any hardware risk is taken. | day 2–3 | nothing on air |
| **M3** | **8733BU first** — the only part already layered, the only working `read_tx_counters`, a 20/20 cold-bring-up baseline to regress against. One `Plan`, three `Role`s: `bring_up_monitor` → `ReceiveOnly`; `bring_up_tx` + `bring_up_tx_tracked` → `TransmitAndReceive` with `PowerTracker` **always on** (an untracked TX plan fades as the PA heats — a defect, not a caller's choice, delivered via `StepOutcome::Guard`); `bring_up_tx_until` → `ProofRequirement::WitnessOrFail`. `tssi_setup` gets `must_precede: ["enable_tx"]`. | day 3–4 | if any caller violates the TSSI ordering today it starts failing at plan construction — correctly, and it will read as a regression |
| **M4** | **a81a / 88xx.** One plan; delete `open`, `open_pid`, `open_pid_select`, `open_monitor_pid`, `open_monitor_pid_select`, and **`open_monitor(channel)` — deleted, not deprecated**: it is the first-Realtek-on-the-bus opener the other four exist to route around, live in `radio-ping:172` and `ndn-phy-wifi/src/lib.rs:777,791`. `NDN_RADIO_SKIP_CAL`/`NDN_RADIO_NO_EFEM` become request fields. Add the `txgapk` TXPAUSE assert. Gate: register-write trace byte-diffed against the pre-migration path with the existing `golden/*.usbmon.txt` + `replay_full.rs`. | day 4–5 | 64 files reference this backend; bench scripts using `open_monitor(ch)` must now name a device. That is the point. ⚠ `replay_full.rs`/`bisect_bringup.rs`/`force_golden_flood.rs` live in the example corpus M8 rewrites — **freeze them until M4 is gated.** |
| **M5** | **mt76x0 / mt7612 / mt7921.** `bring_up` + `setup_monitor_rx` + tune become one plan, so `MT_MAC_SYS_CTRL = ENABLE_TX\|ENABLE_RX` can no longer be missing; warm/cold stays *inside* one step as `Branch("warm")`. MT7612U and RTL8821CU get factory arms (closes #110). The 8821CU's four mutually exclusive, never-scored TX-radiate theories (`NDN_RADIO_STA`, the `NDN_RADIO_NO_TXEN` golden block, `NDN_RADIO_STAREGS`, `NDN_RADIO_IBSS`) become four named plan variants **scored once against a witness in one bench session**, then promoted or deleted. | day 5–6 | ☠ warm/cold must use `mcu_responsive()`'s round trip, not `firmware_running()`'s latch (mt76x0 still uses the latch — a live divergence this touches). ☠ Do **not** helpfully pin EDCA on the MT7612U: both actuators are replug-hazardous there. ☠ Never replay registers to "recover" a quiet MCU — that has wedged this part three times. |
| **M6** | **AR9271, serial, HaLow, LoRa.** `open_ath9k` becomes an arm, keeping the three steps **none of its ~20 example callers do**: the high-power gain table, the Tier-0 filter disable at `0x0050_cf44`, the board/OLPC cal. `rx_enable` gets `Role::ReceiveOnly` and stops being reachable by accident. Serial arms gain channel, `RawNdn(0x8624)`, `NDN_RADIO_BW` and a clock domain uniformly. LoRa gets its first `OpenRadio` constructor. HaLow's plan is `OutOfBand` steps + asserts — the report names `modprobe`/`iw`/`hostapd_s1g`/`morse_cli` as unverified provenance, which is better than shell history and worse than a real plan; the spec says so rather than pretending. | day 6–7 | serial radios changing on-air format is a **wire change** — separate, witnessed commit, both ends together |
| **M7** | **8812AU last, behind a witness.** `bring_up_monitor` (`:6889`) transcribed verbatim into `PLAN_8812AU_MONITOR`; the five asserts added at `Warn`. Acceptance is the regression test for *this* bug: at the AR9271 witness, `Raw(0x3f)` and `Ceiling` must both be heard, must differ by the predicted 18–33 dB, and **both reports must name their reference**. Reference number: the calibrated-bring-up + raw-write result, 3074 frames. | day 7 | highest risk in the migration; see Appendix A.3 |
| **M8** | **Consumers.** `factory.rs`, `ndn-fwd::radio_face`, `ndn-radio-node` (×2), `radio-ping`, `wireless_node` → `open_radio`. Sub-steps drop `pub` → `pub(crate)`, re-exported only under `feature = "bench"`. Examples, by the inventory's own classification (below). Delete `bring_up_monitor`, `bring_up_tx*`, `open_monitor*`, `open_ath9k`, `open_named_radio`. | day 8–9 | the three production paths that bypass `open_named_radio` today silently lose `start_pump`/`apply_bw_override`/`NDN_TX_PWR`/`NDN_CCA_OFF`; they now **gain** all four — right direction, still a behaviour change on production nodes |
| **M9** | The tests of §6, plus `src/bringup_coverage.rs` (one row per part × role, `Provided`/`Excluded(reason)`, modelled on `coverage.rs` **and inheriting its written one-directional caveat**). | day 9 | flushes remaining exclusions into writing |

### The 16 hand-rolled 8812au consumers

The inventory already classifies them; the classification *is* the migration.

* **(c) — copies with no stated reason (9 files):** `burst_fork`, `size_fork`, `reach_fork`,
  `fec_fork`, `monitor_roundtrip`, `nan_ndp`, `nan_s23`, `filter_cpu`, `sense_probe`. Delete ~14
  lines each, call `open_radio`. Mechanical.
* **(a) — private fixes for shared-path defects (3):** `nav_probe`, and the fix halves of
  `inject8812au` / `pwr_sweep8812au`. The fix moves, the experiment stays.
  `write8(0x522, 0x00)` becomes an **assert, not a step** (A/B'd inert, reverted).
  `send_frame_ep(0x04)` and the comment *"endpoint 0x04 is the one that radiated"* are **deleted** —
  refuted (all three endpoints, 0 frames). `set_tx_power` after cal becomes plan order.
* **(b) — instruments (4 + 2 halves):** `cr_probe`, `whoami8812au`, `edcca_probe`, `reg_probe`, the
  endpoint sweep, the TXAGC/`0x80c` sweep. Kept, rebuilt on `Deviation` + `bench::regs`. They stop
  being bring-ups.
* **Also (a), and it is a named driver defect worked around in an example:**
  `named_radio_face.rs:109-119` clears `0x1c90[15]` and writes the per-rate TXAGC table
  `0x3a04…0x3a3c` because *"our `bb_tx_datapath_init` never does"*. ⚠ It **conflicts** with the
  driver's own doc at `libusb_rtl88xx.rs:3488` ("Per-rate diffs in the `0x3a00` table are left at 0,
  so every rate transmits at this reference power"). One of the two is wrong. Resolve it against a
  witness during M4 and write the answer into a step's `why` or into `Plan::excluded` — do not
  migrate the workaround silently.

### `usb_probe.rs`

1072 lines, ~27 flags, doc comment *"List USB devices"*. It should **not** be ported: it is 27
deviations wearing a trench coat, and it is already a hand-written plan interpreter
(`--power-on --fw --mac-init --phy --cal --rx --inject` is the stage list;
`--noiqk --nobbtx --nocca --clearcal --txblock` is `--skip`;
`--forcemac --forcebb --txpwr --qsel --ep --tone --maxgain` is `--poke`). Split three ways:

* a flag encoding a **measured fact** → a plan step with that fact as its `why`;
* a flag encoding a **live question** → a `Deviation` with a `question`;
* a flag encoding a **refuted hypothesis** (`--txfix`, `--bbfix`, `--replayh2c`, `--replayinit`,
  `--useinit`) → **deleted**, with the refutation written into the step it was testing.

Result: `bringup_probe.rs` (~150 lines: run any part's plan, `--skip`, `--stop-after`, `--poke`,
`--report json`) plus `regs.rs`.

### The examples whose on-air numbers predate `8307161`

`size_fork`, `burst_fork`, `reach_fork`, `fec_fork` carry delivery claims that are only consistent
with a ladder taken **before 2026-08-31**, when `lc_calibrate` still un-paused unconditionally. And
independently of that: **every one of the sixteen hand-rolled files ran in the raw regime**, while
the node binary (`wireless_node.rs:209` → `open_named_radio`) ran calibrated. So range, delivery
ratio, throughput, rate-adaptation and contention results taken with those examples were measured on
a transmitter up to ~20 dB hotter than the node that ships.

**Rule for the migration**: on conversion, each of those four files gets its historical numbers
struck through in its doc comment with `⚠ pre-8307161, raw regime — re-take`, and the re-take
records `plan_digest` + `power.reference` beside the number. This is bookkeeping the contract makes
possible; it does not do it for you.

Also re-run, for the same reason: the 2026-09-03 P-0/P-1 result *"no Realtek part has been shown to
carry the 190-bit filter frame"* (`ndn-phy-wifi/docs/p8-header-passthrough.md`) — measured through
`open_named_radio`, i.e. calibrated, on a link with zero margin. Very likely a link-budget result,
not a header result. The ath9k and mt76 arms went through mac80211 and are unaffected.

---

## 6. The tests that bite

All source-level tests follow `tests/one_frame_builder.rs`: recursive `rust_sources`, `body_after`
brace-balance extraction, named exemptions listed in the file so the list itself is the review, and
a failure message that says what will go wrong on air. They are not elegant; they are the kind of
guard that would have saved two silent regressions.

### 6.1 `tests/no_hand_rolled_ladder.rs` — the structural test (the one that matters)

Scans `ndn-radio-drivers/{examples,tests}`, `ndn-radio/`, `ndn-ext/`, `ndn-fwd/`. Any function body
outside `ndn-radio-drivers/src/` that calls **three or more** ladder primitives
(`power_on`, `download_firmware`, `mac_config`, `mac_enable_dma`, `mac_init_queues`, `init_llt`,
`bb_config`, `rf_config`, `iq_calibrate`, `lc_calibrate`, `start_rx_dma`, `init_trx`, `enable_tx_path`,
`tssi_setup`, `phy_init`, `hw_reset`, `wmi_start`, `setup_monitor_rx`, `mac_init`) is an offender.
Exemptions are a `const EXEMPT` list of `(file, fn, reason)`.

> **fails with:** *"`<file>::<fn>` composes its own bring-up ladder from N driver primitives
> (`power_on`, `mac_config`, `iq_calibrate`, …). A hand-rolled ladder is how the fleet ended up
> with 16 different 8812au bring-ups whose only difference was a ~20 dB power regime nobody could
> see. Call `open_radio(pid, &sel, &req)`; to depart from the canonical plan use
> `BringUpRequest::deviation` (which records the departure in the report and the digest); to add a
> rung, edit the part's `PLAN_*` in `ndn-radio-drivers/src/` where the `why` is required."*

After M8 the sub-steps are `pub(crate)`, so this test becomes a *belt* over a compiler *brace* — and
it stays, because it also catches a step re-exported under `bench` being used outside a bench
example.

### 6.2 `tests/power_has_one_meaning.rs`

Three assertions, all source-level:

1. **No fallthrough.** No `fn set_tx_power*` body may contain a call to `set_tx_power_raw` /
   `set_tx_power_idx` / another power writer on a non-error path.
   > *"`rtl8812au::set_tx_power` falls through to a different power regime when calibration is
   > absent. That is the 2026-09-03 defect verbatim: same call, same argument, same `Ok(())`,
   > ~18–33 dB apart. Return `Err(Unsupported)` naming `PowerRequest::Raw`."*
2. **No `std::env` inside a power writer or a plan step.**
   > *"`<fn>` reads `<VAR>`. A knob whose meaning depends on the environment is hidden state by
   > another name (`NDN_AU_TXAGC12` silently turns 10 register writes into 24). Put it in
   > `BringUpRequest`/`PartOpts` so it appears in the report."*
3. **Every `PowerReference::FusedBase` construction is in a driver, never in a face or example** —
   only the part that read the fuse may claim one.

### 6.3 `tests/plan_shape.rs` — hardware-free, per part

* every backend with a `FrameIo` impl declares `plan(role)` for every role it supports, or a
  `bringup_coverage` row with a written exclusion;
* **every `Step::why` is non-empty** (`Seam::Excluded`'s rule, applied to rungs);
* no duplicate step ids; `must_follow`/`must_precede` are satisfiable and satisfied;
* **LAW 5**: every driver function whose body contains a completion poll (`poll32`, `LLT_NO_ACTIVE`,
  `AUTO_LLT`, `WINTINI_RDY`, `mcu_responsive`) appears in its part's plan as `Required`;
* **LAW 6**: every `StepOutcome::Established(Fact::X)` in a plan has a matching field/variant reachable
  in `RadioState::facts`;
* the power step is last among steps that touch the gain chain;
* every `Gate`/`Deviation` env name is registered in `ndn_env::KNOWN` (or in its
  `KNOWN_UNREGISTERED` baseline, where `NDN_AU_TXAGC12` sits today).

> *"`PLAN_8733B_TX` runs `enable_tx` before `tssi_setup`. MEASURED: TSSI first gives 19.6 dB of
> usable range, the other order 1.6 dB. This ordering was prose in a doc comment; it is now a
> constraint, and it is violated."*

### 6.4 `tests/report_is_not_droppable.rs`

`OpenRadio::report` is not `Option`; no `src/` call site of `set_tx_power` discards the
`AppliedPower` with a bare `let _ =` (examples may).

> *"`<file>:<line>` discards the applied power. The applied value is the only thing that
> distinguishes the fused regulatory base from raw chip maximum on this part."*

### 6.5 `tests/bringup_hw.rs` — `#[ignore]`, the first hardware test in the workspace

For each attached part: run the canonical plan, assert every `Assert`, assert
`TxProof != Refuted`, print the report, and — for the 8812au — the M7 acceptance:
`Ceiling` and `Raw` are both heard at the witness, differ by ≥ 15 dB, and report different
`PowerReference`s.

> *"the 8812au reported `PowerReference::FusedBase{base:27}` and `ChipRaw` for the same measured
> RSSI. One of the two reports is lying about which regime it ran in — the defect the contract
> exists to prevent, now inside the contract."*

Today **no test in the workspace opens hardware and nothing regression-tests a bring-up.** That is
why `init_llt` could sit with zero callers in `src/` for months.

---

## 7. What this does NOT fix

Stated plainly, because the alternative is an eleventh unread surface.

1. **Visibility is not detection.** Nothing here would have *caught* the 2026-09-03 bug.
   `ref=FusedBase{27}` alarms only a reader who already knows 27 is ~20 dB below what this link
   needs — and no part of this system knows what the link needs. The honest claim is: **this bug
   would have been located in minutes instead of a day, and it cannot recur in the same shape.** A
   novel ambiguity in a knob nobody has thought about will still cost a bisect; the answer to that
   is `NDN_BRINGUP_DEVIATE`, not the report.
2. **The report is a claim the driver makes about itself.** A `Step` whose closure writes the wrong
   register still reports `Done`. Only four things here are *enforcement*: deleting the fallthrough
   (compiler + §6.2), `RfAuthority`'s unconstructibility in library code, the non-optional `report`
   on `OpenRadio`, and the asserts + TX probe, which read the hardware back. Judge the rest as
   documentation with a better type.
3. **(B) is not proven, ever, by a bring-up.** Radiation needs a second radio. The contract can
   only refuse to *claim* it.
4. **It does not decide whether ≈ 27 is the correct EIRP answer.** Unmeasured, and deliberately
   left so.
5. ~~**The bench link is still marginal.**~~ ☠ **RETRACTED 2026-09-04.** Everything on this bench is
   a few feet apart, and at the CALIBRATED base on our own libusb stack with matched frame formats
   the link delivers 1000/1000 byte-exact. The "marginal link" story came from three broken
   witnesses (a format mismatch in `au_witness`, an AR9271 libusb RX that delivers nothing, and a
   kernel radiotap reading 20 dB below this bench's documented −60..−67 dBm). What IS still true:
   **a power question needs a witness you have verified independently first.**
6. **The 5-vs-12 TXAGC group question stays open.** The contract records which policy ran; it does
   not tell you why `index_base + offset` lands somewhere the 2SS/VHT rates will not transmit from.
7. **Whether the other parts share the calibrated/raw split is unswept.** The a81a and 8733b look
   clean by inspection; nobody has measured it.
8. **The coverage gates are one-directional**, exactly as `coverage.rs`'s own header says: they stop
   over-claiming and cannot catch a stale exclusion (the AR9271 `knobs` cell sat `Excluded` for the
   whole life of a seven-method impl). §6.1 and §6.2 are *link/source* properties and are stronger;
   `bringup_coverage.rs` is not.
9. **HaLow does not get a real plan.** Its bring-up is out-of-band and will stay so; `OutOfBand` +
   asserts says *who* brought the radio up, which is better than shell history and worse than
   owning the sequence.
10. **It does not re-take the dated measurements.** It makes them re-takeable, and it makes a
    re-take self-describing. Somebody still has to go to the bench.
11. **M8 is ~37 example files and several production call sites of mechanical change**, landing in a
    crate that is simultaneously being extracted (`refactor/ndn-radio` phase 3b). If M8 never gets
    scheduled, M1–M2 still stand on their own — that severability is deliberate and is the main
    reason the migration is ordered this way.

---

## Appendix A — conflicts resolved, and why

**A.1 — `max_tx_power` is NOT redefined to the fused base.** The evidence-first proposal's §5 called
the 8812au's `max_tx_power: 63` a scale mismatch and prescribed changing it to ~27. Rejected on the
arithmetic at `rtl8812au.rs:6484`: `offset = idx − 63`, so 63 *is* the fused base on the calibrated
scale, and `decide_power` (which computes `max − backoff`) would then emit 27 → `base − 36` →
`clamp(0)` → a silent near-zero radio. The fact belongs in `PowerReference`, not in a renumbering.

**A.2 — the 12-phase enum is a label, not a gate.** The one-ladder proposal made plans
phase-monotone. Rejected as enforcement: any sequence can be made monotone by relabelling, and its
own text does exactly that (the 88xx's second `set_channel_bw20` relabelled `TxEnable`). Ordering is
enforced by `must_follow`/`must_precede` on step ids, checked at plan construction and in §6.3.
`Stage` survives for `render()` and `--stop-after`, which is all it was ever good for.

**A.3 — the ladder is transcribed, not redesigned, and the 8812au goes last.** The strongest
objection to any plan-shaped contract is that migrating the one 8812au sequence measured to put 3733
frames on air, into a table someone *believes* is equivalent, is exactly the reasoning-about-radios
that has a ~0 % hit rate. Mitigations: M7 is last; the migrating commit adds, moves and removes
nothing; the gate is a golden-trace byte diff plus a witness frame count, not a green test. The
precedent is `init_llt` and the TXPAUSE clear — both added on plausible reasoning, both A/B'd, both
**reverted**. Adding steps "just in case" is the disease.

**A.4 — `Deviation` crosses the factory boundary erased.** A typed `InsertAfter(Step<B>)` would make
`BringUpRequest` generic over the backend and destroy the runtime PID dispatch in
`open_radio(pid, …)`. Erased ops (`Skip`, `StopAfter`, `Poke`, `RegulatoryOverride`) cover the bisect
and the sweeps; typed insertion lives in `bench::run_with`, where the concrete type is known.

**A.5 — steps take `&Arc<B>` and may return a guard.** Anything less cannot express
`bring_up_tx_tracked` (`self: &Arc<Self> -> Result<PowerTracker>`), and a contract that cannot
express the one part that already meets it is not a contract.
---

## M7 acceptance — the RTL8812AU, at the AR9271 witness

*Appended 2026-09-03, when M7 landed. This section is the acceptance, not a design change: nothing
above it is revised. It exists because §5-M7's gate is **an on-air run a person performs**, and
until somebody performs it the migration is unverified — every claim in the M7 commit is a
compile-time or source-level property, and this part is the one whose radiating bring-up is
measured.*

### What is being tested, and what is not

**Tested:** that `PLAN_8812AU_MONITOR` still transmits, and that the two power regimes the
2026-09-03 bisection could not tell apart are now (a) both reachable, (b) still ~18–33 dB apart on
air, and (c) **each labelled in its own report**. That last clause is the whole contract in one
line: a frame count with no `PowerReference` beside it is the number that cost a day.

**Not tested:** whether ≈ 27 is the correct EIRP answer (§7.4 — deliberately unmeasured), whether
the *other* parts share the split (§7.7), and anything about the sequence beyond "it still works".
A green run does not license reordering a rung; Appendix A.3 still holds.

### Preconditions

1. **Two hosts.** A transmitter with an 8812au-family dongle and a witness with the AR9271. ☠ The
   lab inventory moves between sessions — run `lsusb | grep -e 0bda -e 0cf3` on **both** hosts and
   read **every** line before assuming which dongle is where. My notes are evidence, not truth.
2. **The kernel is not holding the 8812au.** `sudo modprobe -r rtw88_8812au`, and open with
   `NDN_RADIO_NO_RESET=1` so `claim`'s USB reset does not re-enumerate the device into the kernel
   driver's hands. (On NixOS the `/run/modprobe.d` blacklist is ignored; `NDN_RADIO_NO_RESET=1` is
   the reliable lever.)
3. **The AR9271 firmware path.** `NDN_ATH9K_FW=~/ath9k-fw/target_firmware/build/k2/htc_9271.fw` —
   the image is not embedded. ⚠ Unbinding `ath9k_htc` leaves the part unable to take firmware again
   until it is physically replugged; unbind once, at the start.
4. **A link with margin, on ch6.** ★ This is the precondition people will skip and it decides the
   run. §7.5: *the bench link is still marginal — the sweep closes only at chip maximum, at
   −85.6 dBm.* At that margin the calibrated arm reads 0 frames and the acceptance's first clause
   ("**both** must be heard") fails for a reason that has nothing to do with M7. **Shorten the
   path** — same room, a metre or two, clear line of sight — until the calibrated arm decodes, then
   do not move anything for the rest of the run. Both arms must see one unmoved geometry or the dB
   difference means nothing.

### The commands

**On the witness (AR9271), for each arm:**

```sh
sudo NDN_ATH9K_FW=$HOME/ath9k-fw/target_firmware/build/k2/htc_9271.fw \
     NDN_WITNESS_DEV=ath9k \
     LD_LIBRARY_PATH=$(nix path-info nixpkgs#libusb1)/lib \
     ./target/debug/examples/au_witness 6 20
```

Counts, per second, the frames whose payload carries the flood's `0x42` filler, and prints the
mean/sd/min/max RSSI of *those frames only*. (`NDN_WITNESS_DEV=ath9k` runs `open_ath9k`, i.e.
`PLAN_AR9271_MONITOR`. Its `recv_frame` surfaces only ethertype-0x8624 NDN frames by design, so on
this arm the "all traffic" column is not ambient Wi-Fi.)

**On the transmitter, arm A — `Ceiling`, the regime the shipped node runs in:**

```sh
sudo NDN_RADIO_NO_RESET=1 \
     LD_LIBRARY_PATH=$(nix path-info nixpkgs#libusb1)/lib \
     ./target/debug/examples/tx_flood_8812au 20 6
```

**On the transmitter, arm B — `Raw(0x3f)`, off the regulatory scale:**

```sh
sudo NDN_RADIO_NO_RESET=1 \
     NDN_RADIO_TX_RAW=63 \
     NDN_RF_UNRESTRICTED="<operator>:M7 acceptance, bringup-contract §5-M7, bench only" \
     LD_LIBRARY_PATH=$(nix path-info nixpkgs#libusb1)/lib \
     ./target/debug/examples/tx_flood_8812au 20 6
```

⚠ Arm B transmits **off the regulatory scale** and may exceed licensed EIRP. `NDN_RF_UNRESTRICTED`
is the only way to reach it, it is mandatory, and the words you put in it are printed verbatim in
the report and in a `tracing::warn!` for the life of the run. Without it, `raw_from_env` refuses by
name and the run aborts — it does **not** quietly fall back to the calibrated regime, because a
silent fallback is the 2026-09-03 defect wearing a hat.

★ Both arms run **the same plan**, through `bring_up_planned`, and differ in exactly one thing: the
`PowerRequest` the last rung actuates. That is deliberate. Using a hand-rolled ladder for one arm
would compare two different bring-ups and prove nothing about the transcription.

### Pass criteria (§5-M7)

Run A, then B, then A again — three runs, so a drift in the room is visible as a difference between
the two A's rather than being charged to B.

| # | criterion | where to read it |
|---|---|---|
| 1 | **Both arms are heard.** `ours > 0` at the witness on A *and* on B. | `au_witness`: `ON AIR: <n> frames in 20s` |
| 2 | **They differ by 18–33 dB.** `mean(RSSI_B) − mean(RSSI_A)` lands in `[18, 33]`. | `au_witness`: `RSSI (ours, n=…): mean … dBm` |
| 3 | **Both reports name their reference, and they differ.** A prints `ref=FusedBase{base:<≈27>, ch:6}`; B prints `ref=ChipRaw` **and** the `⚠ AUTHORITY` line with the operator's words. | `tx_flood_8812au`'s `power  ref=…` line |
| 4 | **The two runs are not confusable.** `digest` differs between A and B (the power reference feeds it), and `writes 10 regs` on both — ten, not twenty-four, i.e. `RateGroupPolicy::FiveMeasured`. | the report header and its `power` line |
| 5 | **The plan ran canonically and whole.** `CANONICAL`, and the step line shows all fourteen rungs with no `:FAIL` and no `:skip`. | the report's step line |
| 6 | **The five asserts are quiet on 2.4 GHz.** Every one reads `ok`, none reads `MISMATCH`: `0x0522=0x00`, `0x0100` with bits 6+7 set, `0x0808=0x33`, `0x0838` low nibble `0x4`, `0x0a07=0x01`. | the report's `assert` line, and the `⚠` warning lines under it |

**Reference number.** The comparable figure from before the migration is **3074 frames** — a
calibrated bring-up followed by a raw write, i.e. arm B's regime reached the long way round. Arm B
should land in that neighbourhood on a comparable link and window. It is a sanity anchor and **not**
a threshold: the witness, the geometry and the channel occupancy all moved between then and now, and
criterion 1 (heard at all) plus criterion 2 (the difference) are what the acceptance turns on.

### Reading a failure

* **Criterion 1 fails on A only (B heard, A silent).** Almost certainly §7.5, the marginal link —
  not a regression. Shorten the path and re-run. If A is still silent at a metre while B is loud,
  *that* is a real finding and the next step is `NDN_BRINGUP_DEVIATE` against the plan, not a source
  read.
* **Criterion 1 fails on B (nothing heard at all).** The transcription broke something. Bisect with
  `NDN_BRINGUP_DEVIATE="skip:<rung>"`; each rung has an id and a `why`, which is the entire reason
  they do. ☠ Do **not** "fix" it by adding `init_llt`, a `write8(0x522, 0x00)`, or a different
  bulk-OUT endpoint. All three were tried, A/B'd, and reverted — see §5-M7 and the plan's own
  header. The 2026-09-03 symptom of this exact shape was the **power regime**.
* **Criterion 2 fails low (< 18 dB).** Either the witness is saturating — check `max` against `mean`
  and move it further away, not closer — or the calibrated arm is not on the fused base. Read
  criterion 3 before concluding anything about dB.
* **Criterion 3 fails: both arms print the same reference.** ★ This is the failure the contract was
  written to make visible. One of the two reports is lying about which regime it ran in, and the
  handle is back to meaning two different physical powers with one label. Stop; this is more serious
  than a silent radio.
* **Criterion 6: `cck_rx_restored` warns.** On **ch6 it is a real finding** — the CCK RX path byte
  `0x0a07` was left at the IQK quiesce value `0x0f`. On a **5 GHz** channel it is expected and
  documented: `PROGS_5G` legitimately programs `0x0a04 = 0x0fff000c`, so `0x0f` is both the restored
  value and the quiesce value there and no static `want` can separate them. Do not "fix" it by
  changing `want` to `0x0f` — that would silence the assert on the one band where it works.

### After the run

Record, beside the two frame counts: the `plan_digest`, the `power ref=` line, and the witness
RSSI means. §3 exists so that this is one copy-paste rather than a reconstruction, and §7.10's
point is that the number is only re-takeable if it says which bring-up produced it.

If the run passes, `txpause_released` and `mac_tx_rx_enabled` become candidates for promotion from
`Warn` to `Fatal` on this part — but only with their own measurement (a run where they *do* fire),
per §1.5. A readback nobody has watched fail is still not allowed to refuse the radio the forwarder
runs on.

---

## M8 acceptance — the consumers

M8 has no on-air acceptance of its own, and that is the honest statement rather than a gap. It
moved every consumer onto one sequence per part; it did not change any sequence. The transmitter
question belongs to M7's acceptance above, and it is still the one that has to be run.

What M8 *can* be checked against, all hardware-free and all in the test suite:

| claim | guard |
|---|---|
| there is exactly ONE factory, and every arm runs its part's plan | `tests/plan_shape_m8.rs::open_radio_is_the_one_door` |
| the six deleted openers stay deleted, workspace-wide | `::the_deleted_openers_stay_deleted` |
| LAW 1 — one reader for every bring-up knob | `::only_from_env_reads_the_bring_up_knobs` |
| every ladder rung is `pub` only under `feature = "bench"` | `::every_rung_is_declared_through_the_rung_macro` |
| the five refuted `usb_probe` flags are gone AND their refutations are in the plan | `::the_refuted_usb_probe_flags_are_deleted_and_their_refutations_recorded` |
| `usb_probe` is split, and `regs.rs` gets its radio from the plan | `::usb_probe_is_split_into_a_plan_runner_and_a_register_surface` |
| no consumer composes a ladder out of driver primitives | `tests/no_hand_rolled_ladder.rs` (§6.1) |

### ⚠ What a deployed node will do differently

§5-M8 predicted three of these and asked that they be said out loud. They are also on
`open_radio`'s own doc comment, where someone reading the code will meet them.

1. **The three production paths that bypassed `open_named_radio` gain four behaviours.**
   `ndn-fwd::radio_face` (both arms), `ndn-radio-node` (both arms) and `radio-ping` hand-rolled
   `open_monitor*` + a post-hoc knob call, so they silently ran with **no RX pump**, **no
   `NDN_RADIO_BW`**, **no `NDN_TX_PWR`** and **no `NDN_CCA_OFF`**. They now get all four. Right
   direction; still a change on a node in the field, and the RX pump in particular changes USB
   contention on a TX-heavy node.
2. **`NDN_CCA_OFF` reaches three parts instead of one.** It meant "disable carrier sense" on the
   RTL8812AU and *nothing at all* on the RTL8733BU and RTL8822E, both of which implement the knob.
   That asymmetry is the divergence this milestone exists to remove, so it is now uniform — and
   applied *with a `Warning` in the report naming the part*, because a transmitter that has stopped
   deferring must be visible to a reader of the run's own output.
3. **`NDN_TX_PWR` rides into the RTL8812AU plan** instead of being applied after it. Same actuator,
   same value; what changes is that the request is now in the report and in `plan_digest`.
4. **`NDN_RADIO_TX_2T` moved from `ndn-fwd` to `PartOpts`,** so it now applies on every path that
   opens an 8812au rather than on the one that remembered to read it. ⚠ Two chains at high power is
   where this class of dongle browns off a 500 mA USB2 bus mid-transmit — MEASURED on the sibling
   a81a — so it is recorded in the report as a warning naming that risk.
5. **`NDN_ATH9K_HIGHPWR` is no longer read inside `apply_initvals`.** It was `||`-ed in there on top
   of the flag `select_gain_table` sets, so the gain table could disagree with what the report said
   it was — on the one knob that moves this part by ~12 dB. The variable still works, through
   `BringUpRequest::from_env` → `GainTableChoice::ForceHigh`.
6. **`NDN_RADIO_FORCE_FW` is no longer read inside two MT7612U rung bodies** (`load_rom_patch`,
   `load_firmware`). They read the flag `bring_up_planned` stashes. A caller that invokes those two
   methods directly, outside a plan, loses the environment override and must pass the request field.

### What did NOT change, deliberately

* **Every register sequence.** No plan gained, lost or reordered a rung in M8.
* **The RTL8733BU `PowerTracker` is still leaked** (`std::mem::forget`), exactly as
  `open_named_radio` did. §1.7 gives `OpenRadio` a `guards` field; it is not built, and carrying the
  guard on the handle would *change* behaviour — thermal tracking would stop the moment a caller
  dropped the radio.
* **The raw power regime of every converted bench instrument.** `burst_fork`, `size_fork`,
  `reach_fork`, `fec_fork`, `monitor_roundtrip`, `nan_ndp`, `nan_s23`, `filter_cpu`, `nav_probe`,
  `inject8812au`, `edcca_probe`, `reg_probe`, `cr_probe` and `whoami8812au` all kept
  `PowerRequest::raw_from_env(0x3f)`. Moving them to `Ceiling` would have invalidated every number
  they have ever produced by moving the transmitter instead of the measurement.
* **The eleven converted AR9271 instruments keep `GainTableChoice::ForceNormal` +
  `Ath9kCalPolicy::Never`**, which is byte-identical to never selecting a table. `FromEeprom` on a
  high-power module is MEASURED ~50 dB louder; that is the correct production answer and an
  unmeasured change to a bench instrument.

### M7 acceptance — first run, 2026-09-03: criteria 3-6 PASS, 1-2 blocked

Run on mds-o5p-2 (RTL8812AU), both arms through `bring_up_planned`, differing only in the
`PowerRequest` the last rung actuates.

**Arm A — `Ceiling` (the regime the shipped node runs in):**
```
radio RTL8812AU  plan rtl8812au/monitor@v2  digest 0xf2012a79bbd1e88f  CANONICAL   1.03 s
  power  ref=FusedBase{base:27, ch:6}  req=Ceiling(FiveMeasured)  clamped=no  dbm=None  actuated
  steps  power_on · download_firmware · mac_config · mac_enable_dma · mac_init_queues · bb_config ·
         rf_config · set_channel · load_tx_power_info · disable_edcca ·
         iq_calibrate:all four paths converged · lc_calibrate · start_rx_dma · set_tx_power
  assert 0x0522=0x00 ok · 0x0100=0x6ff ok · 0x0808=0x33 ok · 0x0838=0x6c89b44 ok · 0x0a07=0x01 ok
injected 7073 frames
```

**Arm B — `Raw(0x3f)`:** identical plan, `digest 0xdd9db5a1b8fb5249`, `ref=ChipRaw`,
`writes 10 regs  idx span 63..63  [FiveMeasured]`, all five asserts `ok`, 7083 frames injected, and
a `⚠ RF OFF THE REGULATORY SCALE` line carrying `pmle:"M7 acceptance, bringup-contract 5-M7, bench
only"` verbatim.

| # | criterion | verdict |
|---|---|---|
| 3 | both reports name their reference, and they differ | ✅ `FusedBase{base:27, ch:6}` vs `ChipRaw` + the AUTHORITY line |
| 4 | the two runs are not confusable | ✅ digests differ (`0xf2012a79…` / `0xdd9db5a1…`); `writes 10 regs` on both ⇒ `FiveMeasured`, not 24 |
| 5 | the plan ran canonically and whole | ✅ `CANONICAL`, all 14 rungs, no `:FAIL`, no `:skip` |
| 6 | the five asserts are quiet on 2.4 GHz | ✅ all `ok` — including `0x0522=0x00`, i.e. the TXPAUSE retry-poisoning hazard is still not firing |
| 1 | both arms heard at the witness | ⛔ **blocked** |
| 2 | they differ by 18–33 dB | ⛔ **blocked** |

**⚠ CORRECTED 2026-09-04 — reason (1) below was WRONG.** The link is not marginal: at the calibrated
base, our stack both ends, matched formats, it delivers 1000/1000 byte-exact at bench range. Criteria
1–2 are **runnable** and remain owed; they were not run because every witness tried that day was
broken (see the root-cause doc's instrument table). Reason (2) stands as written. The original text
is kept below because the reasoning it encodes — "at zero margin every power question looks like a
transmit failure" — is sound, and was applied to a margin that did not exist.

**Why 1–2 were not run, as recorded on 2026-09-03:**

1. **The bench link has no margin.** Independently MEASURED the same day: raw idx 63 → 2301 frames
   at −85.6 dBm; idx 55 → **0**. Arm A's ≈27 is 36 steps below maximum, so criterion 1 fails at this
   geometry for a reason that has nothing to do with the transcription. The acceptance's own
   precondition ("shorten the path until the calibrated arm decodes") requires physically moving
   hardware.
2. **The witness node became unreachable mid-run.** mds-o5p-1's sshd began closing connections
   during banner exchange (TCP accepts, ping fine) — resource exhaustion from accumulated detached
   `ssh -f` sessions and `nohup`ed `tshark` processes from this session's earlier campaigns. The
   second AR9271 (on the C4, mds-05) was tried as a substitute and heard **0 frames even on arm B**,
   i.e. it is out of RF range of mds-o5p-2 — a geometry fact, not a result.

**So what is and is not established.** The four criteria that test *the migration* — that the
transcription is faithful, that the plan is canonical and whole, that the asserts read the hardware
back, and that the two regimes are distinguishable in the report and the digest — all pass. The two
that test *the power difference on air* are unrun, and the direction of that difference was already
MEASURED independently today (2301 frames vs 0, eight index steps apart). **Criteria 1–2 remain
owed**, and they need a link with margin and both nodes up. Do not record M7 as fully accepted until
they are run.

---

## M7 acceptance — SECOND run, 2026-09-04: criterion 1 PASSES, criterion 2 is WRONG AS WRITTEN

The first run could not complete criteria 1–2 and blamed a marginal link. **Both of those were my
errors.** The link is fine; the witness was broken; and criterion 2 asks for a number that does not
exist at the channel it was run on.

### The instrument changed, and the old one is unusable

`au_witness` + the AR9271 **cannot serve as this acceptance's witness today**:

* `NDN_WITNESS_DEV=ath9k` brings the part up **once per physical replug**. A second bring-up without
  one fails hard: `failed_at: "hw_reset"`, `Failed("ath9k_htc: htc recv: Operation timed out")` —
  the documented "cannot take firmware again after unbind" trap, now visible in the report instead
  of as a mystery.
* On the one bring-up it does get, its RX delivers **0 frames, not even ambient**.
* The a81a arm sees 1277–1385 ambient frames and **0 of ours**, in `Raw80211` *and* in the
  transmitter-matching `RawNdn{0x8624}`. Cause unresolved; both failure modes are written into the
  file. **Do not use `au_witness` for a link or power question until it is root-caused.**

**Use `ndn-phy-wifi/examples/wide_profile_onair.rs`** — our libusb driver on BOTH ends
(`WIDE_PID=<hex>`), matched `FrameFormat`, and it now reports RSSI of our frames only:

```sh
# RX (a81a, mds-o5p-0)
sudo -E WIDE_PID=a81a WIDE_CH=149 WIDE_MODE=rx WIDE_N=50 ./wide_profile_onair
# TX arm A — Ceiling (the shipped node's regime)
sudo -E NDN_RADIO_TX_RATE=4 WIDE_PID=8812 WIDE_CH=149 WIDE_MODE=tx WIDE_N=50 ./wide_profile_onair
# TX arm B — Raw, needs the written authority
sudo -E NDN_RF_UNRESTRICTED="<op>:<reason>" NDN_RADIO_TX_RATE=4 WIDE_PID=8812 WIDE_CH=149 \
     WIDE_MODE=tx WIDE_N=50 ./wide_profile_onair
```

### Measured, RTL8812AU → a81a, ch149, one unmoved geometry

| arm | request | TXAGC written | frames | byte-exact | RSSI (ours) |
|---|---|---|---|---|---|
| **A** | `Ceiling` | **44** (fused base @ ch149) | 850 | **850** | **mean −54.5** (min −61, max −50) |
| **B** | `Raw(0x3f)` + authority | **63** | 900 | **900** | **mean −56.4** (min −59, max −53) |
| **C** | `Index(20)` | **0..2** | **0** | — | no frames |

* **Criterion 1 — both arms heard: ✅ PASS.** 850 and 900 frames, every one byte-exact.
* **Criterion 2 — "they differ by 18–33 dB": ❌ NOT OBSERVED. −1.9 dB.** Arm C is the control that
  makes this meaningful: a deep back-off to `idx 0..2` kills the link outright, so the actuator
  works and the receiver is not blind to power. Between **44 and 63 there is no measurable
  difference at ch149.**
* Criteria 3–6 pass as in the first run (references differ, digests differ, `CANONICAL`, asserts ok).

### What criterion 2 should say

The regime split is **real and proven by the reports** (`FusedBase{44}` vs `ChipRaw`), and the API
defect it caused is fixed. But **its on-air magnitude was never measured** — the "18–33 dB" came
from a kernel witness now shown to be broken, and the fused base is **channel-dependent**: 27 on ch6,
44 on ch149. At ch149 the gap is 19 index steps at the top of the TXAGC curve and measures ~0 dB.

So the criterion is restated: **the two arms must be distinguishable in the REPORT, and any dB claim
must name its channel and its witness.** Every `~18-33 dB` string has been removed from the code
(it was in a user-facing error message). What is still owed is a ch6 run — where the gap is 36 steps
— on a working witness; that needs a 2.4 GHz receiver on our own stack, which the AR9271 fault
currently denies.

