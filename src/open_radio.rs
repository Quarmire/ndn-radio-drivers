//! **§1.1 and §1.7 of the bring-up contract: one request, one factory, one handle.**
//!
//! This module is M8. Everything above it — the plans (M3–M7), the report (M2), the power types
//! (M1) — was machinery with no single door. This is the door.
//!
//! What it replaces, by name: `open_named_radio(pid, channel)`, `open_ath9k(channel)`,
//! `LibUsbRtl88xxBackend::{open_monitor, open_monitor_pid, open_monitor_pid_select}`,
//! `Rtl8812auBackend::bring_up_monitor`, `Rtl8733buBackend::{bring_up_monitor, bring_up_tx,
//! bring_up_tx_tracked, bring_up_tx_until}`, and `Rtl8821cuBackend::open_monitor`.
//!
//! ## The defect being removed
//!
//! On 2026-09-03 the node binary and sixteen bench examples held *indistinguishable* handles to
//! transmitters ~20 dB apart, because each had assembled its own bring-up and the difference lived
//! in whether one call had run three calls earlier. The plans removed the per-part divergence. The
//! remaining divergence was at the *consumer* boundary: six production paths, each reading a
//! different subset of the environment, each starting (or forgetting) the pump, each applying (or
//! not) the width and power overrides. `open_radio` is the single place all of that happens.
//!
//! ## Where `BringUpRequest` lives, and why it is not in the HAL
//!
//! ⚠ **Contract §1.1 puts `BringUpRequest` in `ndn-radio-hal::bringup`, beside `BringUpReport`.
//! It is here instead.** The reason is [`PartOpts`]: the AR9271 arm needs a
//! [`GainTableChoice`](crate::GainTableChoice) and an [`Ath9kCalPolicy`](crate::Ath9kCalPolicy),
//! and the RTL8821CU arm needs an [`Rtl8821cVariant`](crate::Rtl8821cVariant) — all three are
//! driver-owned types, and the HAL must not depend on the driver crate (that is the layering
//! `OpenRadio`'s own doc comment records getting wrong once already). The alternatives were to
//! duplicate three enums into the HAL, or to add a fourth argument to `open_radio` and break
//! §1.7's signature. Naming the deviation here beats merging the two designs silently, which is
//! the failure mode the contract's Appendix A exists to prevent.
//!
//! Nothing is lost by the move: every consumer of a request (`factory.rs`, `ndn-fwd::radio_face`,
//! `ndn-radio-node`, `radio-ping`, `wireless_node`) already depends on this crate, because the
//! thing it is asking for is a driver.
//!
//! ## LAW 1
//!
//! [`BringUpRequest::from_env`] is **the one function in the workspace allowed to read `NDN_*`
//! for a bring-up.** Everything else takes a request. A knob read anywhere else is the
//! `load_tx_power_info` defect with better spelling.

use std::sync::Arc;

use ndn_radio_hal::bringup::{
    BringUpFailure, BringUpReport, Deviation, PowerRequest, ProofRequirement, PumpPolicy, Role,
    WitnessOracle,
};
use ndn_radio_hal::{Bandwidth, OpenRadio, RadioKnobs};

use crate::{
    AR9271_IDS, Ath9kBringUpOpts, Ath9kCalPolicy, Ath9kHtcBackend, DeviceSelect, FaceError,
    FrameFormat, GainTableChoice, LibUsbRtl88xxBackend, MT7610U_PIDS, MT7612U_PIDS, MT7921U_PIDS,
    Mt7610uBackend, Mt7612uBackend, Mt7921uBackend, NDN_ETHERTYPE, RTL8733B_PIDS, RTL8812AU_PIDS,
    RTL8821CU_PIDS, Rtl8733buBackend, Rtl8812auBackend, Rtl8821cVariant, Rtl8821cuBackend,
};

// ─────────────────────────────────────────────────────────────────────────────
// §1.1 — what the caller asks for
// ─────────────────────────────────────────────────────────────────────────────

/// **Everything a bring-up is allowed to depend on.**
///
/// Build one with [`BringUpRequest::new`] (a caller that knows what it wants) or
/// [`BringUpRequest::from_env`] (a node binary or bench tool that must honour the fleet's `NDN_*`
/// knobs). Pass it to [`open_radio`].
///
/// ⚠ There is deliberately **no `Default`**. `power` is a regulatory decision and §2 refuses to
/// have one chosen by omission; `new` makes the caller name a channel and hands them
/// [`PowerRequest::ceiling`], which is a decision with a name.
#[derive(Clone)]
#[non_exhaustive]
pub struct BringUpRequest {
    pub channel: u8,
    /// `NDN_RADIO_BW`. Applied through `RadioKnobs::set_channel` **after** the plan on every arm
    /// that has a power/channel knob — which is the ordering the narrowband path requires (on the
    /// 8733b the 5/10 MHz BB registers must be written after the RF registers). Two arms
    /// deliberately do not honour it; see [`open_radio`].
    pub bw: Bandwidth,
    /// The one canonical on-air format, so any two radios opened this way interoperate by
    /// construction. Changing it is a **wire change** and needs both ends and a witness.
    pub format: FrameFormat,
    /// Replaces `bring_up_monitor` vs `bring_up_tx` as separate functions, and
    /// `NDN_8733B_RX_ONLY` / `NDN_ATH9K_RX_ONLY` as hidden forks.
    pub role: Role,
    /// ★ The regulatory decision, named at the call site.
    ///
    /// On the RTL8812AU this rides **into** the plan (`PLAN_8812AU_MONITOR`'s `set_tx_power` rung
    /// reads it back out of the context), so the request is in the digest. On every other part
    /// the plan establishes its own reference and this is applied afterwards through
    /// `RadioKnobs::set_tx_power` — but **only when it names a number**: [`PowerRequest::Ceiling`]
    /// and [`PowerRequest::NoActuator`] mean "whatever the plan left", which is what
    /// `open_named_radio` did when `NDN_TX_PWR` was unset.
    pub power: PowerRequest,
    /// §4 — what must be PROVEN about the transmitter before the handle is returned.
    pub proof: ProofRequirement,
    /// The peer that answers question (B). Required by, and only by,
    /// [`ProofRequirement::WitnessOrFail`] — this is what `Rtl8733buBackend::bring_up_tx_until`'s
    /// `verify` closure becomes.
    pub witness: Option<WitnessOracle>,
    /// Replaces the four different pump owners. `NDN_NO_PUMP` is [`PumpPolicy::None`];
    /// `NDN_RX_PUMP_DEPTH` is the depth.
    pub pump: PumpPolicy,
    /// `NDN_ASYNC_PUMP` — the libusb async submit-ahead RX pump (~2× the sync pump on the
    /// 8812au) instead of the synchronous `read_bulk` one.
    pub async_pump: bool,
    /// `NDN_TX_PUMP` — pipelined TX depth for the two MediaTek arms that have one. `None` = each
    /// arm's own MEASURED default (8 on the MT7610U, 32 on the MT7612U); `Some(0)` restores the
    /// synchronous path for an A/B.
    pub tx_pump: Option<usize>,
    /// ⚠ `NDN_CCA_OFF` — force carrier sense off so this radio transmits regardless of a busy
    /// medium (the doctrine's monitor-mode-without-CSMA sender, where the slot rather than CSMA is
    /// the collision avoidance). See [`open_radio`] for the behaviour change M8 makes here.
    pub cca_off: bool,
    /// `NDN_BRINGUP_DEVIATE`, merged with whatever part-specific deviation
    /// [`PartOpts`] carries.
    pub deviation: Option<Deviation>,
    /// Part-specific inputs. One field per part, because the alternative — an enum — makes a
    /// request that can only describe one radio, and a node opens several.
    pub part: PartOpts,
}

/// The part-specific half of a request: facts only one arm reads.
///
/// Flat, not an enum, so one request can be handed to `open_radio` for several different PIDs on
/// the same host without being rebuilt per part.
#[derive(Clone, Debug, Default)]
#[non_exhaustive]
pub struct PartOpts {
    /// **AR9271.** The firmware bytes. Read at the caller boundary and handed down, because a rung
    /// that opened a path out of an env var would be `load_tx_power_info` with better spelling.
    /// `None` makes the AR9271 arm refuse by name.
    pub ath9k_firmware: Option<Arc<Vec<u8>>>,
    /// **AR9271.** `NDN_ATH9K_HIGHPWR` / `NDN_ATH9K_NORMPWR` / neither (read the EEPROM).
    /// MEASURED: a high-power module on the NORMAL table radiates ~50 dB low.
    pub ath9k_gain_table: GainTableChoice,
    /// **AR9271.** The board + OLPC power cal.
    pub ath9k_cal: Ath9kCalPolicy,
    /// **AR9271.** `NDN_ATH9K_PUMP` — the concurrent RX pump is **opt-in on this part only**: the
    /// 8-reader HTC bulk-IN pattern is not load-tested here, so the default is the on-demand
    /// single blocking bulk-IN read that `FrameIo::recv_frame` does. Stated as a field rather than
    /// left as an asymmetry between two arms of one factory.
    pub ath9k_pump: bool,
    /// **AR9271.** `NDN_ATH9K_RX_ONLY`, the part-specific spelling. Kept separate from
    /// [`BringUpRequest::role`] on purpose — see [`PartOpts::rx_only_8733b`].
    pub rx_only_ath9k: bool,
    /// **RTL8733BU.** `NDN_8733B_RX_ONLY`.
    ///
    /// ⚠ These two `rx_only_*` flags are **not** folded into [`BringUpRequest::role`], and that is
    /// deliberate. Deployed scripts set them, and they are part-specific by name; a unified
    /// `NDN_RADIO_RX_ONLY` that reached every part would turn a witness node's RTL8812AU into a
    /// `PlanError::NoPlan` at bring-up, because `PLAN_8812AU_MONITOR` has no `ReceiveOnly` role.
    /// `NDN_RADIO_RX_ONLY` exists and *does* set `role` for every part — a caller that wants the
    /// uniform meaning asks for it by that name and gets the named refusal where it does not hold.
    pub rx_only_8733b: bool,
    /// **MT7610U / MT7612U / MT7921AU.** `NDN_RADIO_FORCE_FW` — take the cold firmware path even
    /// when the MCU answers. ☠ Warm/cold must be decided by `mcu_responsive()`'s round trip, never
    /// by a status latch, and registers are never replayed to "recover" a quiet MCU.
    pub mt76_force_cold: bool,
    /// **RTL8821CU.** Which of the four mutually exclusive, never-scored TX-radiate hypotheses to
    /// run. Each is an UNTESTED HYPOTHESIS awaiting one bench session against a witness; the
    /// variant is in the `PlanId` and therefore in the digest, so a hypothesis run can never be
    /// silently compared with a canonical one.
    pub rtl8821c_variant: Rtl8821cVariant,
    /// **RTL8822E (a81a).** `NDN_RADIO_MINIMAL` / `NDN_RADIO_SKIP_CAL` / `NDN_RADIO_NO_EFEM`,
    /// already assembled into a self-labelling deviation by the driver that owns those rung ids.
    pub a81a_deviation: Option<Deviation>,
    /// **RTL8733BU.** `NDN_8733B_NO_TSSI`, likewise.
    pub rtl8733b_deviation: Option<Deviation>,
    /// **RTL8812AU.** `NDN_RADIO_TX_2T` — drive BOTH antenna paths (`0x80c[15:0] = 0x3333`
    /// instead of `0x1111`).
    ///
    /// ⚠ Two chains is where this part browns out on a 500 mA USB2 bus (MEASURED on the a81a: the
    /// dongle drops off the bus mid-transmit, which reads as an IQK/EVM fault and is a
    /// power-delivery one). It is a bring-up field rather than a `RadioKnobs` seam because there
    /// is no portable "how many chains" knob — and it was read directly in `ndn-fwd::radio_face`
    /// before M8, i.e. on exactly one of the several paths that open this part.
    pub rtl8812au_tx_2t: bool,
}

/// Hand-written because [`WitnessOracle`] is a boxed closure and cannot derive `Debug` — the same
/// reason `PlanRun` writes its own. The witness is reported as present or absent, which is the
/// only thing about it a reader can act on.
impl std::fmt::Debug for BringUpRequest {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("BringUpRequest")
            .field("channel", &self.channel)
            .field("bw", &self.bw)
            .field("format", &self.format)
            .field("role", &self.role)
            .field("power", &self.power)
            .field("proof", &self.proof)
            .field("witness", &self.witness.is_some())
            .field("pump", &self.pump)
            .field("async_pump", &self.async_pump)
            .field("tx_pump", &self.tx_pump)
            .field("cca_off", &self.cca_off)
            .field("deviation", &self.deviation)
            .field("part", &self.part)
            .finish()
    }
}

impl BringUpRequest {
    /// A production request: this channel, 20 MHz, the canonical on-air format, transmit and
    /// receive, at the part's regulatory ceiling, with the factory's RX pump.
    pub fn new(channel: u8) -> Self {
        Self {
            channel,
            bw: Bandwidth::Bw20,
            format: FrameFormat::RawNdn {
                ethertype: NDN_ETHERTYPE,
            },
            role: Role::TransmitAndReceive,
            // ★ §2: the default stays CALIBRATED. The contract does not conclude that the fused
            // base is wrong — it is probably the correct regulatory answer — only that a caller
            // must be able to see which regime it got.
            power: PowerRequest::ceiling(),
            proof: ProofRequirement::BestAvailable,
            witness: None,
            pump: PumpPolicy::Start(DEFAULT_PUMP_DEPTH),
            async_pump: false,
            tx_pump: None,
            cca_off: false,
            deviation: None,
            part: PartOpts::default(),
        }
    }

    /// **LAW 1 — the one place a bring-up's configuration is read from the environment.**
    ///
    /// Every `NDN_*` that any of the six deleted openers used to read, read exactly once, here,
    /// and turned into a field. A caller that wants none of it calls [`new`](Self::new).
    ///
    /// The AR9271 firmware is read from disk here for the same reason: the bytes are an input, and
    /// a rung that opened a path out of an env var would be the hidden-state defect one level
    /// down. A missing or unreadable `NDN_ATH9K_FW` is left as `None` and refused **by the AR9271
    /// arm** rather than failing every other part's open.
    pub fn from_env(channel: u8) -> Self {
        let mut r = Self::new(channel);

        // ── width ────────────────────────────────────────────────────────────────────────────
        if let Ok(v) = std::env::var("NDN_RADIO_BW") {
            match v.trim() {
                "5" => r.bw = Bandwidth::Nb5,
                "10" => r.bw = Bandwidth::Nb10,
                "20" => r.bw = Bandwidth::Bw20,
                "40" => r.bw = Bandwidth::Bw40,
                "80" => r.bw = Bandwidth::Bw80,
                other => tracing::warn!("NDN_RADIO_BW={other}: expected 5|10|20|40|80, ignoring"),
            }
        }

        // ── role ─────────────────────────────────────────────────────────────────────────────
        if std::env::var_os("NDN_RADIO_RX_ONLY").is_some() {
            r.role = Role::ReceiveOnly;
        }
        r.part.rx_only_8733b = std::env::var_os("NDN_8733B_RX_ONLY").is_some();
        r.part.rx_only_ath9k = std::env::var_os("NDN_ATH9K_RX_ONLY").is_some();

        // ── power ────────────────────────────────────────────────────────────────────────────
        // ★ `NDN_RADIO_TX_RAW` + `NDN_RF_UNRESTRICTED` is the ONLY route off the regulatory scale,
        // and it needs the operator's own words, which end up printed in every report of the run.
        // A refusal is loud rather than a silent downgrade to the calibrated scale.
        if let Some(idx) = std::env::var("NDN_RADIO_TX_RAW")
            .ok()
            .and_then(|s| s.parse::<u8>().ok())
        {
            match PowerRequest::raw_from_env(idx) {
                Ok(p) => r.power = p,
                Err(e) => {
                    let msg = format!(
                        "NDN_RADIO_TX_RAW={idx} REFUSED, bringing up on the calibrated scale: {e}"
                    );
                    tracing::warn!(target: "named_radio", "{msg}");
                    eprintln!("{msg}");
                }
            }
        } else if let Some(p) = std::env::var("NDN_TX_PWR")
            .ok()
            .and_then(|s| s.parse::<u8>().ok())
        {
            r.power = PowerRequest::index(p);
        }

        // ── pump ─────────────────────────────────────────────────────────────────────────────
        r.pump = if std::env::var_os("NDN_NO_PUMP").is_some() {
            PumpPolicy::None
        } else {
            PumpPolicy::Start(env_pump_depth())
        };
        r.async_pump = std::env::var_os("NDN_ASYNC_PUMP").is_some();
        r.tx_pump = std::env::var("NDN_TX_PUMP")
            .ok()
            .and_then(|s| s.parse::<usize>().ok());
        r.cca_off = std::env::var_os("NDN_CCA_OFF").is_some();

        // ── deviation ────────────────────────────────────────────────────────────────────────
        match std::env::var("NDN_BRINGUP_DEVIATE") {
            Ok(spec) => match Deviation::parse(&spec) {
                Ok(d) => r.deviation = Some(d),
                Err(e) => {
                    let msg = format!(
                        "NDN_BRINGUP_DEVIATE={spec:?} is malformed and was IGNORED — the run is \
                         CANONICAL, not deviated: {e}"
                    );
                    tracing::warn!(target: "named_radio", "{msg}");
                    eprintln!("{msg}");
                }
            },
            Err(_) => r.deviation = None,
        }
        r.part.a81a_deviation = crate::a81a_env_deviation();
        r.part.rtl8733b_deviation = crate::rtl8733b_env_deviation();

        // ── part-specific ────────────────────────────────────────────────────────────────────
        r.part.mt76_force_cold = mt76_force_cold_from_env();
        r.part.rtl8812au_tx_2t = std::env::var_os("NDN_RADIO_TX_2T").is_some();
        r.part.rtl8821c_variant = Rtl8821cVariant::from_env();
        r.part.ath9k_pump = std::env::var_os("NDN_ATH9K_PUMP").is_some();
        r.part.ath9k_gain_table = if std::env::var_os("NDN_ATH9K_HIGHPWR").is_some() {
            GainTableChoice::ForceHigh
        } else if std::env::var_os("NDN_ATH9K_NORMPWR").is_some() {
            GainTableChoice::ForceNormal
        } else {
            GainTableChoice::FromEeprom
        };
        // ⚠ `NDN_ATH9K_SETPOWER` is NOT read, and never was — `open_ath9k`'s pre-M6 doc comment
        // advertised it beside code that only tested `NDN_ATH9K_SETBOARD`. Making the name live
        // would turn the PA cal ON, on air, on any node whose scripts already set it, on a change
        // nobody at this keyboard can measure. Recorded, not fixed.
        r.part.ath9k_cal = if std::env::var_os("NDN_ATH9K_NO_CAL").is_some()
            || r.part.ath9k_gain_table == GainTableChoice::ForceNormal
        {
            Ath9kCalPolicy::Never
        } else if std::env::var_os("NDN_ATH9K_SETBOARD").is_some() {
            Ath9kCalPolicy::Always
        } else {
            Ath9kCalPolicy::WhenHighPower
        };
        // ⚠ EXPERIMENTAL on this HT20-class part: cal convergence at 40 MHz is unverified. It is
        // read AFTER `NDN_RADIO_BW` and overrides it, which is what `open_ath9k` did.
        if std::env::var_os("NDN_ATH9K_HT40").is_some() {
            r.bw = Bandwidth::Bw40;
        }
        r.part.ath9k_firmware =
            std::env::var("NDN_ATH9K_FW")
                .ok()
                .and_then(|p| match std::fs::read(&p) {
                    Ok(b) => Some(Arc::new(b)),
                    Err(e) => {
                        let msg = format!("NDN_ATH9K_FW={p}: cannot read the AR9271 firmware: {e}");
                        tracing::warn!(target: "named_radio", "{msg}");
                        eprintln!("{msg}");
                        None
                    }
                });

        r
    }

    /// Receive only. The witness/observer spelling — and on the parts that have a `ReceiveOnly`
    /// plan it also skips the calibration, which is both the slow part and the variable part.
    pub fn receive_only(mut self) -> Self {
        self.role = Role::ReceiveOnly;
        self.part.rx_only_8733b = true;
        self.part.rx_only_ath9k = true;
        self
    }

    /// The caller runs its own receive loop; the factory starts no pump.
    pub fn caller_owns_pump(mut self) -> Self {
        self.pump = PumpPolicy::CallerOwns;
        self
    }

    pub fn with_power(mut self, power: PowerRequest) -> Self {
        self.power = power;
        self
    }

    pub fn with_bw(mut self, bw: Bandwidth) -> Self {
        self.bw = bw;
        self
    }

    pub fn with_format(mut self, format: FrameFormat) -> Self {
        self.format = format;
        self
    }

    pub fn with_role(mut self, role: Role) -> Self {
        self.role = role;
        self
    }

    /// §1.6 — depart from the canonical plan, on the record. The departure lands in
    /// `report.provenance`, in `report.deviations` and in `plan_digest`, so a bench number and a
    /// production number can never be silently compared.
    pub fn with_deviation(mut self, d: Deviation) -> Self {
        self.deviation = Some(d);
        self
    }

    /// §4 — what must be proven about the transmitter before the handle is returned.
    /// [`ProofRequirement::WitnessOrFail`] additionally needs [`with_witness`](Self::with_witness);
    /// asking for it without one is a named error before the first register write, not a silent
    /// pass.
    pub fn with_proof(mut self, proof: ProofRequirement) -> Self {
        self.proof = proof;
        self
    }

    /// The peer that answers question (B) — *did anything coherent radiate?* Only a peer can
    /// answer it: MEASURED on the AR9271, the MAC can key the transmitter while a witness at
    /// inches decodes zero frames.
    pub fn with_witness(mut self, oracle: WitnessOracle) -> Self {
        self.witness = Some(oracle);
        self
    }

    /// The deviation actually in force for this arm: the generic one merged with the part's own.
    fn deviation_for(&self, part_specific: Option<Deviation>) -> Option<Deviation> {
        match (self.deviation.clone(), part_specific) {
            (Some(a), Some(b)) => Some(a.merge(b)),
            (Some(a), None) => Some(a),
            (None, b) => b,
        }
    }
}

/// **The ONE reader of `NDN_RADIO_FORCE_FW`** — take the cold firmware path even when the MCU
/// answers.
///
/// It is a free function rather than an inline `env::var_os` inside
/// [`BringUpRequest::from_env`] because three `bring_up(channel)` wrappers (MT7610U, MT7612U,
/// MT7921AU) also need it at their own caller boundary, and LAW 1 is about there being ONE reader,
/// not about where the reader lives. A bench instrument driving `bring_up_planned` directly calls
/// this for the same reason.
///
/// ☠ It must never be read from inside a rung, and it no longer is: the value is stashed on the
/// backend by `bring_up_planned` and the rungs read the stash. Warm/cold is decided by
/// `mcu_responsive()`'s round trip, never by a status latch, and registers are never replayed to
/// "recover" a quiet MCU.
pub fn mt76_force_cold_from_env() -> bool {
    std::env::var_os("NDN_RADIO_FORCE_FW").is_some()
}

/// RX-pump reader-thread / transfer-pool count when the caller does not name one.
pub(crate) const DEFAULT_PUMP_DEPTH: usize = 8;

fn env_pump_depth() -> usize {
    std::env::var("NDN_RX_PUMP_DEPTH")
        .ok()
        .and_then(|s| s.parse().ok())
        .filter(|&n| n > 0)
        .unwrap_or(DEFAULT_PUMP_DEPTH)
}

// ─────────────────────────────────────────────────────────────────────────────
// §1.7 — one factory
// ─────────────────────────────────────────────────────────────────────────────

/// **The standardized way to open a named-data radio.** Dispatch by USB product id to the right
/// chip-specific backend, run *that* chip's `Plan`, apply the request's width / power / carrier
/// sense, start the pump the request asked for, and return the handle with the report the plan
/// produced.
///
/// ```no_run
/// # use ndn_radio_drivers::{BringUpRequest, DeviceSelect, open_radio};
/// let req = BringUpRequest::from_env(6);
/// let radio = open_radio(0xa81a, &DeviceSelect::from_env(), &req)?;
/// println!("{}", radio.report().render());
/// # Ok::<(), ndn_radio_hal::bringup::BringUpFailure>(())
/// ```
///
/// ## What the error carries
///
/// [`BringUpFailure`], not `FaceError` — so a failed open says **how far it got**: which rung, in
/// which stage, with everything established up to it. `?` into a `FaceError` signature still works
/// (there is a `From`), and the conversion **prints the partial report** on the way through rather
/// than dropping it. Failures before the first rung (no dongle on the bus, no firmware file, an
/// undispatchable PID) come back as [`BringUpFailure::not_opened`], which at least names the part
/// the caller was asking for.
///
/// ## ⚠ Behaviour changes M8 makes, on purpose
///
/// The contract's §5-M8 predicted three of these and asked that they be said out loud.
///
/// 1. **Three production paths bypassed `open_named_radio`** and hand-rolled a two-call ladder:
///    `ndn-fwd::radio_face`, `ndn-radio-node`, `radio-ping`. They silently had no RX pump, no
///    `NDN_RADIO_BW`, no `NDN_TX_PWR` and no `NDN_CCA_OFF`. They now get all four. Right
///    direction; still a change on a deployed node.
/// 2. **`NDN_CCA_OFF` now reaches every arm whose backend implements the knob** (RTL8812AU,
///    RTL8733BU, RTL8822E), not the 8812au alone. It is applied *and recorded as a
///    [`Warning`](ndn_radio_hal::Warning) naming the part*, because a radio transmitting into a
///    busy medium is a thing the operator must be able to see in the report. Before M8 the same
///    variable meant "disable carrier sense" on one part and nothing at all on two others, which
///    is the divergence this milestone exists to remove.
/// 3. **`NDN_TX_PWR` now rides *into* the RTL8812AU plan** rather than being applied after it. The
///    actuator and the value are identical; what changes is that the request is now in the report
///    and in `plan_digest`, so the calibrated and raw regimes cannot be confused again.
/// 4. **The RTL8733BU's `PowerTracker` is leaked, exactly as before.** §1.7 gives `OpenRadio` a
///    `guards` field; it is not built, and carrying the guard on the handle would *change*
///    behaviour — thermal tracking would stop the moment a caller dropped the radio. Reproducing
///    `open_named_radio`'s `std::mem::forget` keeps the deployed behaviour and is recorded as an
///    open item rather than quietly improved.
///
/// ## What is NOT applied where, and why
///
/// * **MT7612U — no width override.** `NDN_RADIO_BW` would call `set_channel` a second time, which
///   on this part replays the whole captured 226-op channel stream to change nothing (the two
///   captured programs are COUPLED channel/width pairs). A knob that re-runs a replay to change
///   nothing is worse than no knob.
/// * **MT7612U — no contention actuator, ever.** ☠ Both are replug-hazardous there: a legal
///   `cw_min` of 2, MEASURED good on five other parts, killed this one twice past the reach of our
///   restore, our cold bring-up and the kernel driver's probe alike.
/// * **RTL8821CU — no width, no power, no carrier sense.** It has no `RadioKnobs` impl at all, so
///   none of the three has an actuator to reach. Saying so is the point.
/// * **AR9271 — width is a plan input, not a post-plan knob.** Its `set_channel` validates a
///   same-channel apply; a live retune on this part is `hw_reset`, i.e. re-open.
#[allow(clippy::result_large_err)]
pub fn open_radio(
    pid: u16,
    sel: &DeviceSelect,
    req: &BringUpRequest,
) -> Result<OpenRadio, BringUpFailure> {
    // ── AR9271 (ath9k_htc) ───────────────────────────────────────────────────────────────────
    //
    // The one Wi-Fi part whose FIRMWARE is ours, so Tier-0 can reject a frame before it crosses
    // USB and TX can be scheduled off the hardware TSF.
    if AR9271_IDS.iter().any(|&(_, p)| p == pid) {
        return open_ar9271(req);
    }

    // ── RTL8731BU / RTL8733BU (halmac_87xx, 1x1 802.11n) ─────────────────────────────────────
    //
    // The one part whose bring-up does NOT collapse into "monitor mode and you're done":
    // *radiating* additionally needs the full cal (IQK -> TXGAPK -> DPK, then the datapath TXAGC
    // block the cal zeroes) plus a background power-tracking loop. `Role::TransmitAndReceive` is
    // `PLAN_8733B_TX`, which is all of it; `Role::ReceiveOnly` is `PLAN_8733B_MONITOR`.
    if RTL8733B_PIDS.contains(&pid) {
        return open_rtl8733b(req);
    }

    // ── MT7610U (mt76x0u, 1x1 dual-band 11ac) ────────────────────────────────────────────────
    if MT7610U_PIDS.contains(&pid) {
        return open_mt7610u(sel, req);
    }
    // ── MT7921AU (connac2, 2x2 802.11ax) ─────────────────────────────────────────────────────
    if MT7921U_PIDS.contains(&pid) {
        return open_mt7921au(sel, req);
    }
    // ── MT7612U (mt76x2u, 2x2 802.11ac) ──────────────────────────────────────────────────────
    if MT7612U_PIDS.contains(&pid) {
        return open_mt7612u(req);
    }
    // ── RTL8821CU (rtw88 8821c/8811cu) ───────────────────────────────────────────────────────
    if RTL8821CU_PIDS.contains(&pid) {
        return open_rtl8821cu(req);
    }
    // ── RTL8822E (a81a) ──────────────────────────────────────────────────────────────────────
    if matches!(pid, 0xa81a | 0xa811 | 0x8814) {
        return open_rtl8822e(pid, sel, req);
    }
    // ── RTL8812AU ────────────────────────────────────────────────────────────────────────────
    if RTL8812AU_PIDS.contains(&pid) {
        return open_rtl8812au(sel, req);
    }

    // Dispatch by PID, do not silently fall through: an unknown pid must NOT open the first
    // 8812au on the bus.
    Err(BringUpFailure::not_opened(
        "unknown",
        "open_radio::dispatch",
        FaceError::Io(std::io::Error::other(format!(
            "open_radio: pid 0x{pid:04x} is not a dispatchable radio \
             (supported: 8822E 0xa81a/0xa811/0x8814, 8812AU {RTL8812AU_PIDS:#06x?}, \
             8733BU {RTL8733B_PIDS:#06x?}, 8821CU {RTL8821CU_PIDS:#06x?}, \
             MT7610U {MT7610U_PIDS:#06x?}, MT7612U {MT7612U_PIDS:#06x?}, \
             MT7921AU {MT7921U_PIDS:#06x?}, plus the AR9271 set)"
        ))),
    ))
}

// ─────────────────────────────────────────────────────────────────────────────
// the arms
// ─────────────────────────────────────────────────────────────────────────────

fn open_ar9271(req: &BringUpRequest) -> Result<OpenRadio, BringUpFailure> {
    let Some(firmware) = req.part.ath9k_firmware.clone() else {
        return Err(BringUpFailure::not_opened(
            "AR9271",
            "open_radio::ar9271::firmware",
            FaceError::Io(std::io::Error::other(
                "the AR9271 firmware is not embedded (it lives at \
                 ~/ath9k-fw/target_firmware/build/k2/htc_9271.fw on the node). Set \
                 NDN_ATH9K_FW=<path to htc_9271-1.4.0.fw>, or fill \
                 BringUpRequest::part.ath9k_firmware.",
            )),
        ));
    };
    // ★ §5-M6: `rx_enable` is reachable ONLY through `Role::ReceiveOnly` — `PLAN_AR9271_RX` arms
    // `AR_IMR_S0 = 0x0001_0000` (no TXOK on the data ACs) and sends no `IC_UPDATE`/vif-node, so a
    // transmitter on it blocks at the ring depth on frame 34.
    let role = if req.part.rx_only_ath9k {
        Role::ReceiveOnly
    } else {
        req.role
    };
    let dev = Arc::new(
        Ath9kHtcBackend::open()
            .map_err(|e| BringUpFailure::not_opened("AR9271", "open_radio::ar9271::open", e))?,
    );
    let opts = Ath9kBringUpOpts {
        // `Arc<Vec<u8>>` in the request so one request can open several dongles without a copy
        // per open; the backend takes ownership of the bytes, so this is the one clone.
        firmware: (*firmware).clone(),
        gain_table: req.part.ath9k_gain_table,
        cal: req.part.ath9k_cal,
    };
    let (mut report, guards) = dev.bring_up_planned(
        req.channel,
        req.bw,
        role,
        opts,
        req.deviation.clone(),
        req.proof.clone(),
    )?;
    debug_assert!(guards.is_empty(), "the AR9271 plans produce no guards");

    // RX delivery: the default is the on-demand path (`FrameIo::recv_frame` does a single blocking
    // bulk-IN read when no pump is marked) — proven to read 802.11 on this HTC pipe. The
    // concurrent pump is opt-in on this part; see `PartOpts::ath9k_pump`.
    match (req.pump, req.part.ath9k_pump) {
        (PumpPolicy::Start(depth), true) => {
            start_pump(&dev, depth, req.async_pump);
            report.state.pump = PumpPolicy::Start(depth);
        }
        (p, _) => {
            report.state.pump = if p == PumpPolicy::None {
                p
            } else {
                PumpPolicy::CallerOwns
            }
        }
    }
    // ⚠ No post-plan width or power knob on this part: its `set_channel` validates a same-channel
    // apply (a live retune is `hw_reset`, i.e. re-open), and power comes from the gain TABLE the
    // plan picked plus the optional OLPC cal, both of which are already in the report.
    let report = report;
    crate::emit_bringup(&report);
    Ok(OpenRadio {
        io: dev.clone(),
        knobs: Some(dev.clone()),
        time: Some(dev.clone()),
        profile: Some(dev),
        report,
    })
}

fn open_rtl8733b(req: &BringUpRequest) -> Result<OpenRadio, BringUpFailure> {
    // No `DeviceSelect` arm: `Rtl8733buBackend::open` claims the first match and has no
    // `open_select` sibling. Fine while a host carries one f72b; a second would need it added.
    let d = Arc::new(
        Rtl8733buBackend::open()
            .map_err(|e| BringUpFailure::not_opened("RTL8733BU", "open_radio::8733b::open", e))?
            .with_format(req.format),
    );
    // `NDN_8733B_RX_ONLY` stops at monitor RX and skips the cal — a witness/receiver node does not
    // need the TX path, and the cal is both the slow part and the variable part.
    let role = if req.part.rx_only_8733b {
        Role::ReceiveOnly
    } else {
        req.role
    };
    let (report, guards) = d.bring_up_planned(
        req.channel,
        role,
        req.deviation_for(req.part.rtl8733b_deviation.clone()),
        req.proof.clone(),
        req.witness.clone(),
    )?;
    // ⚠ The plan's `Guards` — the `PowerTracker` — are LEAKED, not dropped, and this is the
    // pre-M8 behaviour reproduced deliberately (see `open_radio`'s behaviour-change note 4).
    // Dropping the guard would stop thermal tracking the moment the handle went out of scope, and
    // this part fades as the PA heats without it.
    std::mem::forget(guards);
    finish(d, req, report, "RTL8733BU", KnobSet::Full)
}

fn open_mt7610u(sel: &DeviceSelect, req: &BringUpRequest) -> Result<OpenRadio, BringUpFailure> {
    let d = Arc::new(
        Mt7610uBackend::open_selected(sel.clone())
            .map_err(|e| BringUpFailure::not_opened("MT7610U", "open_radio::mt7610u::open", e))?
            .with_format(req.format),
    );
    let (mut report, guards) = d.bring_up_planned(
        req.channel,
        req.role,
        req.part.mt76_force_cold,
        req.deviation.clone(),
        req.proof.clone(),
    )?;
    debug_assert!(guards.is_empty(), "this plan produces no guards");
    apply_bw(d.as_ref(), req, &mut report);
    // ★★ Pipelined TX. MEASURED 2026-08-31: `inject`'s synchronous `write_bulk` costs
    // `~295 + 0.031*B us` per PPDU — a width-INDEPENDENT constant plus USB bus time — capping the
    // part near 3000 PPDU/s and pinning every channel width to the same period. Worth ~+24% at
    // 1400 B under `Shared` and the full lever under an aggressive posture (peak ~250 Mbit/s at
    // `Owned` + 11400 B + Bw80 + VHT MCS9). ⚠ Frames may be reordered across pump threads, which is
    // why it is a knob rather than unconditional.
    let tx_depth = req.tx_pump.unwrap_or(8);
    if tx_depth > 0 {
        std::mem::forget(d.spawn_tx_pump(tx_depth));
    }
    finish(d, req, report, "MT7610U", KnobSet::Full)
}

fn open_mt7921au(sel: &DeviceSelect, req: &BringUpRequest) -> Result<OpenRadio, BringUpFailure> {
    let d = Arc::new(
        Mt7921uBackend::open_selected(sel.clone())
            .map_err(|e| BringUpFailure::not_opened("MT7921AU", "open_radio::mt7921au::open", e))?
            .with_format(req.format),
    );
    let (mut report, guards) = d.bring_up_planned(
        req.channel,
        req.role,
        req.part.mt76_force_cold,
        req.deviation.clone(),
        req.proof.clone(),
    )?;
    debug_assert!(guards.is_empty(), "this plan produces no guards");
    apply_bw(d.as_ref(), req, &mut report);
    finish(d, req, report, "MT7921AU", KnobSet::Full)
}

fn open_mt7612u(req: &BringUpRequest) -> Result<OpenRadio, BringUpFailure> {
    // No `DeviceSelect` arm: `Mt7612uBackend::open` claims the first match and has no
    // `open_select` sibling, so `NDN_USB_ADDR`/`NDN_USB_INDEX` do not reach this part. Stated
    // rather than silently ignored — a host with two MT7612Us cannot pin one today.
    let d = Arc::new(
        Mt7612uBackend::open()
            .map_err(|e| BringUpFailure::not_opened("MT7612U", "open_radio::mt7612u::open", e))?
            .with_format(req.format),
    );
    let (report, guards) = d.bring_up_planned(
        req.channel,
        req.role,
        req.part.mt76_force_cold,
        req.deviation.clone(),
        req.proof.clone(),
    )?;
    debug_assert!(guards.is_empty(), "this plan produces no guards");
    let tx_depth = req.tx_pump.unwrap_or(32);
    if tx_depth > 0 {
        d.spawn_tx_pump(tx_depth);
    }
    // ⚠ `KnobSet::NoWidth`: see `open_radio`'s "what is NOT applied where". And ☠ nothing here
    // touches the contention actuators, which are replug-hazardous on this part.
    finish(d, req, report, "MT7612U", KnobSet::NoWidth)
}

fn open_rtl8821cu(req: &BringUpRequest) -> Result<OpenRadio, BringUpFailure> {
    // ☠ Read this before trusting a transmit result from this arm: the part has never been
    // observed to radiate, and the four mutually exclusive explanations are four named plan
    // variants, each an UNTESTED HYPOTHESIS awaiting one bench session against a witness.
    // Receiving is a different matter and does work — `bb_rx_path_enable` was the fix.
    let d = Arc::new(
        Rtl8821cuBackend::open()
            .map_err(|e| BringUpFailure::not_opened("RTL8821CU", "open_radio::8821cu::open", e))?
            .with_format(req.format),
    );
    let (mut report, guards) =
        d.bring_up_planned(req.channel, req.part.rtl8821c_variant, req.proof.clone())?;
    debug_assert!(guards.is_empty(), "this plan produces no guards");
    // ⚠ No `RadioKnobs` impl at all on this backend, so neither the width, the power nor the
    // carrier-sense request has an actuator to reach. The TXAGC index the ladder writes at
    // `tune_channel` is in the report and nothing can move it afterwards. Saying so here is the
    // point — a knob that silently does nothing is worse than no knob.
    warn_unactuated(req, &mut report, "RTL8821CU", "it implements no RadioKnobs");
    if let PumpPolicy::Start(depth) = req.pump {
        d.spawn_rx_pump(depth);
        report.state.pump = PumpPolicy::Start(depth);
    } else {
        report.state.pump = req.pump;
    }
    crate::emit_bringup(&report);
    Ok(OpenRadio {
        io: d.clone(),
        knobs: None,
        time: None,
        profile: Some(d),
        report,
    })
}

fn open_rtl8822e(
    pid: u16,
    sel: &DeviceSelect,
    req: &BringUpRequest,
) -> Result<OpenRadio, BringUpFailure> {
    let d = Arc::new(
        LibUsbRtl88xxBackend::open_pid_select(pid, sel)
            .map_err(|e| BringUpFailure::not_opened("RTL8822E", "open_radio::8822e::open", e))?
            .with_format(req.format),
    );
    let (report, guards) = d.bring_up_planned(
        req.channel,
        req.role,
        req.deviation_for(req.part.a81a_deviation.clone()),
        req.proof.clone(),
    )?;
    debug_assert!(guards.is_empty(), "this plan produces no guards");
    finish(d, req, report, "RTL8822E", KnobSet::Full)
}

fn open_rtl8812au(sel: &DeviceSelect, req: &BringUpRequest) -> Result<OpenRadio, BringUpFailure> {
    // Force the canonical format — this backend's own default is `Raw80211`, for the NAN path.
    let d = Arc::new(
        Rtl8812auBackend::open_select(sel)
            .map_err(|e| BringUpFailure::not_opened("RTL8812AU", "open_radio::8812au::open", e))?
            .with_format(req.format),
    );
    // ★ This is the ONE arm where the power request rides INTO the plan: `PLAN_8812AU_MONITOR`'s
    // last rung reads it back out of the context, so the regime is in the report and in the
    // digest. It is also the part where the two regimes are different (the gap is CHANNEL-DEPENDENT and its size is UNVERIFIED (fused base is 27 on ch6 and 44 on ch149; MEASURED 2026-09-04 at ch149 the Ceiling-vs-Raw difference is ~0 dB, not tens)) and the API used to
    // pick between them by hidden state.
    let (report, guards) = d.bring_up_planned(
        req.channel,
        req.role,
        req.power.clone(),
        req.deviation.clone(),
        req.proof.clone(),
    )?;
    debug_assert!(guards.is_empty(), "this plan produces no guards");
    // `NDN_RADIO_TX_2T` — no `RadioKnobs` seam exists for the chain count, so it is applied here
    // from the request rather than read from the environment by whichever consumer remembered to.
    let mut report = report;
    if req.part.rtl8812au_tx_2t {
        match d.set_tx_2t(true) {
            Ok(()) => report.warnings.push(ndn_radio_hal::Warning::new(
                "tx_2t",
                "RTL8812AU: BOTH antenna paths driven (0x80c=0x3333). ⚠ MEASURED on the sibling \
                 a81a: two chains at high power browns this class of dongle off a 500 mA USB2 bus \
                 mid-transmit, which reads as an IQK/EVM fault and is a power-delivery one",
            )),
            Err(e) => report.warnings.push(ndn_radio_hal::Warning::new(
                "tx_2t",
                format!("RTL8812AU: two-chain TX could NOT be enabled: {e}"),
            )),
        }
    }
    // `KnobSet::NoPower`: the plan already applied it.
    finish(d, req, report, "RTL8812AU", KnobSet::NoPower)
}

// ─────────────────────────────────────────────────────────────────────────────
// the shared tail
// ─────────────────────────────────────────────────────────────────────────────

/// Which of the request's post-plan knobs an arm can actuate. Named rather than open-coded per
/// arm, because "which arm applies which override" *was* the divergence.
#[derive(Clone, Copy, PartialEq, Eq)]
enum KnobSet {
    /// Width, power and carrier sense all reach an actuator.
    Full,
    /// Everything but the width. MT7612U: `set_channel` replays a 226-op captured stream and the
    /// width is not independently selectable anyway.
    NoWidth,
    /// Everything but the power, because the plan already applied the request.
    NoPower,
}

/// The tail every USB arm shares: post-plan width, post-plan power, carrier sense, the pump, the
/// INFO/WARN block, and the handle.
///
/// One function rather than eight copies, because eight copies with slightly different subsets is
/// exactly what `open_named_radio` had and what M8 removes.
fn finish<B>(
    d: Arc<B>,
    req: &BringUpRequest,
    mut report: BringUpReport,
    part: &'static str,
    knobs: KnobSet,
) -> Result<OpenRadio, BringUpFailure>
where
    B: crate::FrameIo
        + RadioKnobs
        + ndn_radio_hal::RadioTime
        + ndn_radio_hal::RadioProfile
        + crate::rx_pump::Pumpable
        + 'static,
{
    if knobs != KnobSet::NoWidth {
        apply_bw(d.as_ref(), req, &mut report);
    }
    if knobs != KnobSet::NoPower {
        report = apply_power(d.as_ref(), req, part, report);
    }
    apply_cca(d.as_ref(), req, part, &mut report);
    match req.pump {
        PumpPolicy::Start(depth) => {
            start_pump(&d, depth, req.async_pump);
            report.state.pump = PumpPolicy::Start(depth);
        }
        // `NDN_NO_PUMP` — a pure TX-blast node needs no RX, and the pump's bulk-IN threads
        // otherwise contend with inject for USB bandwidth (MEASURED: an 8812au TX collapses to
        // ~250 f/s under heavy RX while the pump drains thousands of frames/s).
        p => report.state.pump = p,
    }
    crate::emit_bringup(&report);
    Ok(OpenRadio {
        io: d.clone(),
        knobs: Some(d.clone()),
        time: Some(d.clone()),
        profile: Some(d),
        report,
    })
}

/// `NDN_RADIO_BW` — bring a radio up at a non-default channel width, applied through
/// `RadioKnobs::set_channel` AFTER the chip's own bring-up. That ordering is what the narrowband
/// path requires: on the 8733b the 5/10 MHz BB registers must be written after the RF registers
/// or, in the vendor's words, the MAC rate is right but nothing comes out of the RF.
///
/// Narrowband trades rate for link budget — a quarter-clocked 5 MHz channel puts the same energy
/// in a quarter of the bandwidth, so the noise floor drops ~6 dB.
fn apply_bw(knobs: &dyn RadioKnobs, req: &BringUpRequest, report: &mut BringUpReport) {
    if req.bw == Bandwidth::Bw20 {
        return;
    }
    // `eprintln!` as well as the report: an operator running a bring-up binary that never installs
    // a subscriber would otherwise see NOTHING — and this message exists precisely to stop a
    // narrowband run from silently measuring 20 MHz twice, which is how the first attempt at that
    // experiment was lost on 2026-08-24.
    match knobs.set_channel(req.channel, req.bw) {
        Ok(()) => {
            eprintln!("width: channel {} set to {:?}", req.channel, req.bw);
            report.state.bw = req.bw;
        }
        Err(e) => {
            eprintln!("width {:?} NOT APPLIED: {e}", req.bw);
            report.warnings.push(ndn_radio_hal::Warning::new(
                "bw",
                format!(
                    "requested {:?}, refused: {e} — this run is at {:?}",
                    req.bw, report.state.bw
                ),
            ));
        }
    }
}

/// Apply the request's power to an open radio and fold the result into the report.
///
/// ★ The old form was `let _ = d.set_tx_power(p);` — a knob whose result was discarded, on the one
/// part where the applied value is the only thing distinguishing the fused regulatory base from
/// raw chip maximum.
///
/// [`PowerRequest::Ceiling`] and [`PowerRequest::NoActuator`] are **not** applied here: they mean
/// "whatever the plan established", which is what `open_named_radio` did when `NDN_TX_PWR` was
/// unset. Applying a `Ceiling` after the plan would be a second, unmeasured write to a chain the
/// plan just finished calibrating.
fn apply_power(
    knobs: &dyn RadioKnobs,
    req: &BringUpRequest,
    part: &'static str,
    report: BringUpReport,
) -> BringUpReport {
    match &req.power {
        PowerRequest::Ceiling(_) | PowerRequest::NoActuator => return report,
        _ => {}
    }
    match knobs.set_tx_power(req.power.clone()) {
        Ok(applied) => report.with_power(applied),
        Err(e) => {
            tracing::warn!(
                target: "named_radio",
                part, request = ?req.power, error = %e,
                "the power request was refused — the radio is at whatever power its bring-up left, \
                 which the report names"
            );
            report.with_warning(ndn_radio_hal::Warning::new(
                "power",
                format!("requested {:?}, refused: {e}", req.power),
            ))
        }
    }
}

/// ⚠ `NDN_CCA_OFF` — full carrier sense off (EDCCA + OFDM packet CCA), so this radio transmits
/// regardless of a busy medium.
///
/// **M8 behaviour change**: before this, the same variable disabled carrier sense on the RTL8812AU
/// and did nothing at all on the RTL8733BU and RTL8822E, both of which implement the knob. It now
/// reaches all three — and is recorded as a `Warning` naming the part, because a transmitter that
/// has stopped deferring is a thing a reader of the report must be able to see.
fn apply_cca(
    knobs: &dyn RadioKnobs,
    req: &BringUpRequest,
    part: &'static str,
    report: &mut BringUpReport,
) {
    if !req.cca_off {
        return;
    }
    match knobs.set_edcca_ignore(true) {
        Ok(()) => {
            tracing::warn!(
                target: "named_radio", part,
                "CARRIER SENSE OFF — this radio transmits into a busy medium. Collision avoidance \
                 is whatever the MAC layer above provides (a named airtime slot), not CSMA."
            );
            report.warnings.push(ndn_radio_hal::Warning::new(
                "cca_off",
                format!("{part}: carrier sense disabled at bring-up (NDN_CCA_OFF)"),
            ));
        }
        Err(e) => report.warnings.push(ndn_radio_hal::Warning::new(
            "cca_off",
            format!("{part}: carrier sense could NOT be disabled: {e}"),
        )),
    }
}

/// Record, in the report, every override this arm cannot actuate — instead of ignoring it.
fn warn_unactuated(
    req: &BringUpRequest,
    report: &mut BringUpReport,
    part: &'static str,
    why: &'static str,
) {
    let mut asked: Vec<String> = Vec::new();
    if req.bw != Bandwidth::Bw20 {
        asked.push(format!("width {:?}", req.bw));
    }
    if !matches!(
        req.power,
        PowerRequest::Ceiling(_) | PowerRequest::NoActuator
    ) {
        asked.push(format!("power {:?}", req.power));
    }
    if req.cca_off {
        asked.push("carrier sense off".into());
    }
    if !asked.is_empty() {
        report.warnings.push(ndn_radio_hal::Warning::new(
            "unactuated",
            format!("{part}: {} NOT applied — {why}", asked.join(", ")),
        ));
    }
}

/// Start the RX pump for a backend. The pump lives for the process.
fn start_pump<B: crate::rx_pump::Pumpable>(backend: &Arc<B>, depth: usize, async_pump: bool) {
    if async_pump {
        std::mem::forget(crate::rx_pump::spawn_rx_pump_async(backend, depth));
    } else {
        std::mem::forget(crate::rx_pump::spawn_rx_pump(backend, depth));
    }
}
