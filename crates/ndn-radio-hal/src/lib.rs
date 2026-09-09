//! Layer: spec — the data-plane radio HAL contract.
//!
//! The bearer-agnostic transmit/receive seam the connectionless radio faces are
//! built on: [`TxIntent`] states *what* a transmit should achieve, a backend
//! resolves it to its PHY (802.11 maps it to an [`McsDescriptor`] via
//! [`McsDescriptor::for_intent`]); [`InjectFrame`]/[`CapturedFrame`] are the
//! inject/capture units; [`FrameIo`] is the radio trait, and [`WifiRadio`] the
//! WiFi-only escape hatch for injecting at an exact 802.11 rate. Pure types +
//! traits — no I/O, no framing. The on-air framing, radiotap codec, and the
//! reusable AF_PACKET/loopback backends live in `ndn-frame-io`, which re-exports
//! this contract so its public surface is unchanged.

use async_trait::async_trait;
use bytes::Bytes;

/// Re-exported so backend/face authors can name the id type without depending
/// on `ndn-transport` directly.
pub use ndn_transport::{FaceError, FaceId};

/// **The bring-up contract** (`ndn-radio-drivers/docs/bringup-contract.md`): the power vocabulary
/// ([`PowerRequest`] in, [`AppliedPower`] out, [`PowerReference`] saying what the number is
/// referenced to) and the [`BringUpReport`] every `bring_up_*` now returns.
///
/// It lives in the HAL for the same reason [`OpenRadio`] does: a driver *constructs* a report, a
/// face and a bench harness *consume* one, and neither should need the other to name the types.
pub mod bringup;

pub use bringup::{
    AppliedPower, Assert, BringUp, BringUpFailure, BringUpReport, Ctx, Degradation, Deviation,
    DeviationOp, DeviceAddress, Difference, Fact, Guards, Plan, PlanEdit, PlanEdits, PlanError,
    PlanId, PlanRun, PowerReference, PowerRequest, PowerWrite, ProofRequirement, Provenance,
    PumpPolicy, RadioState, RateGroupPolicy, RateState, RequestError, RfAuthority, Role, Severity,
    Stage, Step, StepClass, StepId, StepOutcome, StepOutcomeRecord, StepRecord, TX_PROBE_COUNT,
    TxInstrument, TxProbe, TxProof, Warning, WitnessId, WitnessOracle, WitnessReport,
    power_unsupported, run_plan,
};

/// Re-exported link-timestamp vocabulary (from the named-time core). A backend
/// stamps a [`CapturedFrame`] with a [`LinkStamp`] carrying its clock domain and
/// honest precision; the generic time layer consumes it. See ADR 0007.
pub use ndn_time::{
    ClockDomainId, ClockReference, ClockReferenceKind, LatchPoint, LinkStamp, RadioClockKind,
    RadioTimeSource, RateMeasurement, RateWitness,
};

/// The 802.11 broadcast address — the default destination when no name-group is
/// configured (every monitor receiver keeps the frame).
pub const BROADCAST: [u8; 6] = [0xff; 6];
/// A locally-administered unicast default source. Monitor injection places no
/// meaning on the source MAC (the NDN name is the addressing), but a well-formed
/// 802.11 header needs one; this is `02:'N':'D':'N':00:01`, never a host MAC.
pub const DEFAULT_SRC: [u8; 6] = [0x02, 0x4e, 0x44, 0x4e, 0x00, 0x01];

/// The 802.11n/ac rate to inject a frame at — what defeats the legacy-rate wall.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct McsDescriptor {
    /// Modulation-and-coding index. HT: 0–7 (1 stream) / 8–15 (2 streams).
    /// VHT 1SS at 20 MHz: 0–8 (MCS9 needs ≥40 MHz).
    pub index: u8,
    /// Request the 400 ns short guard interval (≈11% faster, needs good SNR).
    pub short_gi: bool,
    /// Inject as 802.11ac (VHT) instead of 802.11n (HT). VHT adds 256-QAM
    /// (MCS8/9) and a more efficient PHY header.
    pub vht: bool,
    /// Spatial streams. **VHT only** (1 or 2) — selects the VHT-1SS vs VHT-2SS
    /// rate code. For HT the stream count is carried by `index` (0–7 = 1 stream,
    /// 8–15 = 2 streams), so this is ignored. Requires the 2-stream TX path
    /// (`0x820=0x31`, set in `set_channel_bw20`).
    pub nss: u8,
    /// Space-Time Block Coding: Alamouti-encode **one** spatial stream across
    /// both TX antennas (A+B, always enabled here via `0x820=0x31`). Pure TX
    /// diversity — it doubles the air time per bit but turns a 2-antenna chip
    /// into a far more robust single-stream transmitter, with **no receiver
    /// feedback**. That makes it ideal for broadcast NDN, where there are no
    /// ACKs to drive retransmission. Only valid for a 1-stream rate (HT MCS0–7
    /// or VHT `nss == 1`); the descriptor bit is suppressed for 2-stream rates
    /// (STBC + 2 spatial streams is not an 802.11 mode this chip transmits).
    pub stbc: bool,
    /// Low-Density Parity-Check coding: use the LDPC FEC encoder instead of the
    /// mandatory binary convolutional code (BCC). Stronger error correction
    /// (~1.5–2 dB coding gain) for the same rate — directly useful on the lossy,
    /// un-retransmitted broadcast channel. Both endpoints advertise/honour it in
    /// the HT-SIG / VHT-SIG; the receiver must support LDPC RX (the kernel
    /// rtl8812eu does). Independent of `stbc` — they compose.
    pub ldpc: bool,
    /// Transmit as **802.11ax (HE)** instead of HT/VHT — the mode that unlocks the two HE reach levers
    /// below. Selected like [`vht`](Self::vht) selects 11ac; a non-HE bearer ignores it (falls back to
    /// its best mode). Only meaningful when the radio advertises [`RateCapability::he_cap`].
    pub he: bool,
    /// **HE Dual-Carrier Modulation** — map each data bit onto two widely-spaced subcarriers. Halves the
    /// rate but buys frequency diversity + ~a few dB of robustness against narrowband fades: a pure
    /// reach/robustness lever for the un-ACKed broadcast channel, the HE sibling of [`stbc`](Self::stbc).
    /// Requires [`he`](Self::he); ignored otherwise.
    pub dcm: bool,
    /// **HE Extended-Range Single-User (ER-SU)** — a long-range HE PPDU with a repeated, 3 dB-boosted
    /// preamble for ~2–4 dB better receiver sensitivity. The strongest single-frame reach mode an HE PHY
    /// offers, above HT+STBC+LDPC. Requires [`he`](Self::he). Only an HE receiver can decode it (like VHT
    /// vs 11n), so it is a lever cognition opts into for a known-HE reach, not a broadcast default.
    pub er_su: bool,
}

impl McsDescriptor {
    /// A conservative, widely-decodable default (HT MCS1, long GI).
    pub const CONSERVATIVE: McsDescriptor = McsDescriptor {
        index: 1,
        short_gi: false,
        vht: false,
        nss: 1,
        stbc: false,
        ldpc: false,
        he: false,
        dcm: false,
        er_su: false,
    };

    /// An 802.11n (HT) rate at `index`, long GI (index 8–15 = 2 streams).
    pub const fn ht(index: u8) -> Self {
        McsDescriptor {
            index,
            short_gi: false,
            vht: false,
            nss: 1,
            stbc: false,
            ldpc: false,
            he: false,
            dcm: false,
            er_su: false,
        }
    }

    /// An 802.11ac (VHT) single-stream rate at `index`, long GI.
    pub const fn vht(index: u8) -> Self {
        McsDescriptor {
            index,
            short_gi: false,
            vht: true,
            nss: 1,
            stbc: false,
            ldpc: false,
            he: false,
            dcm: false,
            er_su: false,
        }
    }

    /// An 802.11ac (VHT) **2-stream** rate at `index`, long GI.
    pub const fn vht_2ss(index: u8) -> Self {
        McsDescriptor {
            index,
            short_gi: false,
            vht: true,
            nss: 2,
            stbc: false,
            ldpc: false,
            he: false,
            dcm: false,
            er_su: false,
        }
    }

    /// Enable [`stbc`](Self::stbc) (space-time diversity over both antennas).
    /// Chainable: `McsDescriptor::ht(5).with_stbc()`.
    pub const fn with_stbc(mut self) -> Self {
        self.stbc = true;
        self
    }

    /// Enable [`ldpc`](Self::ldpc) (LDPC FEC instead of BCC). Chainable:
    /// `McsDescriptor::vht(7).with_ldpc()`.
    pub const fn with_ldpc(mut self) -> Self {
        self.ldpc = true;
        self
    }

    /// An **802.11ax (HE)** single-stream rate at `index`, long GI — the base for the HE reach levers.
    pub const fn he(index: u8) -> Self {
        McsDescriptor {
            index,
            short_gi: false,
            vht: false,
            nss: 1,
            stbc: false,
            ldpc: false,
            he: true,
            dcm: false,
            er_su: false,
        }
    }

    /// Enable HE [`dcm`](Self::dcm) (dual-carrier modulation — frequency-diversity reach). Forces
    /// [`he`](Self::he). Chainable: `McsDescriptor::he(0).with_dcm()`.
    pub const fn with_dcm(mut self) -> Self {
        self.he = true;
        self.dcm = true;
        self
    }

    /// Enable HE [`er_su`](Self::er_su) (extended-range single-user — the strongest single-frame reach
    /// mode). Forces [`he`](Self::he). Chainable: `McsDescriptor::he(0).with_er_su()`.
    pub const fn with_er_su(mut self) -> Self {
        self.he = true;
        self.er_su = true;
        self
    }
}

impl Default for McsDescriptor {
    fn default() -> Self {
        Self::CONSERVATIVE
    }
}

/// What a transmit should *achieve*, independent of how a given PHY achieves it
/// — the bearer-agnostic transmit contract carried on every [`InjectFrame`].
/// 802.11 maps it to an MCS + coding ([`McsDescriptor::for_intent`]); a LoRa
/// bearer would map it to a spreading factor, an SDR to its own waveform. On a
/// broadcast, un-ACKed medium *reliability* is the primary axis — there is no
/// per-receiver feedback to rate-adapt against, so the caller states intent and
/// the backend (or the cognitive plane) resolves it for the hardware.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct TxIntent {
    /// The robustness objective — the axis that dominates on a no-ARQ broadcast.
    pub reliability: Reliability,
    /// Who the frame is for — every receiver in range, or a name-group.
    pub reach: Reach,
}

/// The robustness objective of a [`TxIntent`].
#[derive(Clone, Copy, Debug, PartialEq, Eq, Default)]
pub enum Reliability {
    /// Maximum robustness — lowest-order modulation + strongest FEC + diversity
    /// coding where the PHY offers it. Discovery, beacons, control: anything the
    /// farthest / worst receiver must still decode. (802.11: base MCS + STBC + LDPC.)
    MostRobust,
    /// A widely-decodable balance — the default when there is no measured link.
    #[default]
    Balanced,
    /// Favour throughput on a link known to be good (measured RSSI headroom).
    Throughput,
}

/// Who a [`TxIntent`] is addressed to.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Default)]
pub enum Reach {
    /// Every receiver in range; no per-receiver adaptation is possible.
    #[default]
    Broadcast,
    /// A name-group; adaptation may target the group's worst member.
    Group,
}

impl TxIntent {
    /// **Must this frame go out at the radio's most universally decodable rate?**
    ///
    /// ★ The single home for a doctrine that was previously re-typed per backend and, as a
    /// result, silently absent from most of them. `MostRobust` means "the worst receiver in
    /// earshot must decode this" — cooperative reports, discovery, control — so the frame is
    /// forced to the **basic rate** (legacy OFDM 6 Mbps on Wi-Fi), whatever rate the control
    /// plane last stored, exactly as 802.11 sends beacons and probes at a basic rate. HT-only
    /// SGI / LDPC / STBC must be suppressed with it: a legacy OFDM PPDU carries no HT-SIG or
    /// VHT-SIG to signal them.
    ///
    /// ⚠ **What ignoring it costs, MEASURED.** An HT/VHT/HE PPDU excludes every receiver without
    /// that decoder *by construction*. The RTL8812AU's 5 GHz golden-trace RX demodulates legacy
    /// OFDM but not HT-MCS, and the a81a's userspace bring-up raises one RX chain — so a peer
    /// transmitting 2-stream MCS9 in good faith produced a real one-way link: drone→GCS perfect,
    /// GCS→drone nothing. Reports and discovery going out at the last throughput rate is that
    /// failure reintroduced in exactly the traffic the worst-receiver work exists to protect.
    ///
    /// The ENCODING stays per-backend — a Realtek DESC code, an mt76x02 TXWI word and a connac2
    /// rate word are three different things — but the DECISION is this one predicate. Every
    /// backend's disposition is recorded in `ndn_radio_drivers::coverage::TX_INTENT`.
    pub fn needs_basic_rate(&self) -> bool {
        self.reliability == Reliability::MostRobust
    }

    /// Maximum-robustness broadcast — the discovery / beacon / control default,
    /// and what a NAN or unmeasured face should use.
    pub const ROBUST: TxIntent = TxIntent {
        reliability: Reliability::MostRobust,
        reach: Reach::Broadcast,
    };
    /// A widely-decodable balance broadcast.
    pub const CONSERVATIVE: TxIntent = TxIntent {
        reliability: Reliability::Balanced,
        reach: Reach::Broadcast,
    };
    /// Broadcast at a stated reliability.
    pub const fn broadcast(reliability: Reliability) -> Self {
        TxIntent {
            reliability,
            reach: Reach::Broadcast,
        }
    }
}

impl Default for TxIntent {
    fn default() -> Self {
        TxIntent::CONSERVATIVE
    }
}

impl McsDescriptor {
    /// Resolve a bearer-agnostic [`TxIntent`] to a concrete 802.11 rate for a
    /// radio that supports up to `max_index` (single-stream HT) and, if
    /// `vht_cap`, 802.11ac. Maps the reliability axis: `MostRobust` → base rate
    /// with STBC + LDPC diversity (ideal for un-ACKed broadcast), `Balanced` → a
    /// conservative mid rate, `Throughput` → the top validated rate + short GI.
    /// This is the 802.11 mapping of the transmit intent; another bearer maps it
    /// differently. An exact WiFi rate (fixed-rate benches, the cognitive face)
    /// travels the [`WifiRadio::inject_at`] path instead — not on the seam.
    ///
    /// `he_cap` unlocks the two 802.11ax reach levers for `MostRobust`: on an HE radio the base rate goes
    /// out as **HE ER-SU + DCM** (~2–4 dB more reach than HT+STBC+LDPC). Since only an HE receiver can
    /// decode ER-SU (as only 11ac decodes VHT), pass `he_cap` `true` only for a known-HE reach; the
    /// worst-overheard-receiver legacy gate still forces legacy when a legacy-only RX is advertised.
    pub fn for_intent(
        intent: &TxIntent,
        max_index: u8,
        vht_cap: bool,
        he_cap: bool,
    ) -> McsDescriptor {
        match intent.reliability {
            Reliability::MostRobust if he_cap => McsDescriptor::he(0).with_er_su().with_dcm(),
            Reliability::MostRobust => McsDescriptor::ht(0).with_stbc().with_ldpc(),
            Reliability::Balanced => McsDescriptor::CONSERVATIVE,
            Reliability::Throughput => {
                // **The ceiling is the caller's, clamped only by what the MODE structurally allows**
                // (#83). This used to be `max_index.min(MAX_RELIABLE_MCS)`, and `MAX_RELIABLE_MCS`
                // is a figure validated on one chip (the RTL8812EU) — so every radio's own declared
                // `RateCapability::Wifi { max_mcs }` was silently overridden by a different part's
                // calibration. The mt7612 declares 9 and was being given 7.
                //
                // What legitimately belongs here is the *standard's* structural limit, which is not
                // a per-chip fact: single-stream HT tops out at MCS7, single-stream VHT at MCS8
                // (MCS9 needs >=40 MHz). Without this, de-globalising the ceiling would let a
                // caller declaring 9 request MCS9 as an HT rate, which is not a rate that exists.
                let mode_ceiling = if vht_cap { 8 } else { 7 };
                let idx = max_index.min(mode_ceiling);
                let base = if vht_cap {
                    McsDescriptor::vht(idx)
                } else {
                    McsDescriptor::ht(idx)
                };
                McsDescriptor {
                    short_gi: true,
                    ..base
                }
            }
        }
    }
}

/// How the face picks the injection MCS for each frame.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum McsPolicy {
    /// Always inject at this rate.
    Fixed(McsDescriptor),
    /// Pick the MCS from the most recently observed RSSI ([`mcs_for_rssi`]).
    /// This is the content-centric replacement for MAC rate-adaptation
    /// feedback: the feedback is the RSSI of frames we hear, not link-layer
    /// ACKs. (Phase 2.)
    Adaptive,
}

impl Default for McsPolicy {
    fn default() -> Self {
        McsPolicy::Fixed(McsDescriptor::CONSERVATIVE)
    }
}

/// Highest MCS the userspace **RTL8812EU** driver is *validated* to deliver today.
///
/// **This is one chip's calibration, and is not a workspace-wide ceiling** (#83). It used to be
/// applied inside [`McsDescriptor::for_intent`] to every radio, silently overriding each part's own
/// declared [`RateCapability::Wifi`] `max_mcs` — the mt7612 declares 9 and was handed 7. It now
/// serves only as the conservative default where no capability is in hand: the radiotap
/// header-only hint in `frame::build`, and the loopback bus. Anywhere a `RadioCapability` exists,
/// use [`RadioCapability::max_mcs`] instead.
///
/// The PA is non-linear at the BPSK/QPSK operating power, so 16-QAM and up
/// (MCS3+) smear (bad EVM) unless backed off. `calibrate_tx_power` now writes a
/// per-rate **backoff** into the `0x3a00` TXAGC diff table (MCS3 −6, MCS4/5
/// −11, MCS6/7 −14) so each rate transmits in its linear region while MCS0–2
/// keep full power — the mechanism the stock driver gets from DPK. With backoff
/// in place this ceiling is the highest rate *confirmed on-air*. Once the
/// on-air TX-power gate was found — the BB NCTL TX-power push
/// (`set_txagc_to_hw`, NCTL reg 0x38 via the 0x1700/0x1704 port), which the
/// abbreviated init skipped and which had left TX ~50 dB low — the link reaches
/// full power (−22 dBm, kernel-level). A full-power MCS sweep vs the OPi
/// receiver then decodes the **entire 11n single-stream range**: MCS2 95%,
/// MCS4–6 ~95–97%, **MCS7 (64-QAM 5/6) 66%**. So the ceiling is the 11n max, 7.
/// (Higher needs 2-stream / VHT / wider bandwidth — separate work.)
///
/// `set_txagc_to_hw`: crate::LibUsbRtl88xxBackend::set_txagc_to_hw
pub const MAX_RELIABLE_MCS: u8 = 7;

/// Map an observed RSSI (dBm) to an 802.11n single-stream 20 MHz MCS index,
/// capped at [`MAX_RELIABLE_MCS`].
///
/// A monotone heuristic over typical 11n receiver-sensitivity thresholds: the
/// stronger the signal we hear from the neighbourhood, the more aggressive the
/// rate we inject at. This is the kernel of [`McsPolicy::Adaptive`]; the
/// "MCS climbs as nodes approach" behaviour is validated on hardware, but the
/// mapping itself is unit-tested here. The ceiling reflects the *verified*
/// reliable rate, not the 11n maximum — see [`MAX_RELIABLE_MCS`].
pub fn mcs_for_rssi(rssi_dbm: i8) -> u8 {
    let raw = match rssi_dbm {
        r if r >= -55 => 7,
        r if r >= -62 => 6,
        r if r >= -68 => 5,
        r if r >= -72 => 4,
        r if r >= -76 => 3,
        r if r >= -80 => 2,
        r if r >= -84 => 1,
        _ => 0,
    };
    raw.min(MAX_RELIABLE_MCS)
}

/// PHY data rate (bits/s) of an 802.11n single-stream 20 MHz MCS, long guard
/// interval — the per-MCS modulation/coding rate table. Used to surface the
/// link's achievable rate as a cross-layer signal (`LinkSignals.observed_tput_bps`)
/// so measured strategies can prefer faster neighbours. This is the PHY rate,
/// an upper bound on goodput, not a measured throughput.
pub fn mcs_phy_rate_bps(mcs_index: u8) -> u32 {
    match mcs_index {
        0 => 6_500_000,
        1 => 13_000_000,
        2 => 19_500_000,
        3 => 26_000_000,
        4 => 39_000_000,
        5 => 52_000_000,
        6 => 58_500_000,
        _ => 65_000_000, // MCS7 (and any out-of-range, clamped to the top rate)
    }
}

/// PHY data rate (bits/s) of an **802.11ah (S1G / HaLow)** MCS at a given channel width — the
/// sub-GHz counterpart of [`mcs_phy_rate_bps`], which is the 11n 20 MHz table and is wrong here by
/// roughly 5–30×.
///
/// `bw_mhz` is the received/transmitted width (1/2/4/8/16) as the S1G radiotap TLV reports it
/// (`ndn_frame_io::radiotap::S1gInfo::bandwidth_mhz`); `short_gi` selects the 4 µs guard interval
/// over the 8 µs one. Returns `None` for a combination S1G does not define — **MCS 9 at 1 MHz *and*
/// at 2 MHz**, and **MCS 10 at anything other than 1 MHz** — because a rate that does not exist must
/// not be answered with a number. The 2 MHz exclusion is *derived* from the rate equation (a
/// fractional bits-per-symbol result is the standard's own reason for the hole); the 1 MHz one has
/// to be named, because it divides evenly. Both are pinned by an exhaustive test.
///
/// # How this is derived, so it can be audited rather than trusted
///
/// S1G is 802.11ac downclocked by 10, so nothing here is a memorised table: it is computed from
/// the same three quantities the OFDM rate equation always uses.
///
/// ```text
///   rate = N_sd × N_bpscs × R / T_sym
///
///   N_sd    data subcarriers:  24 (1 MHz), 52 (2), 108 (4), 234 (8), 468 (16)
///   N_bpscs bits per subcarrier per stream: 1 (BPSK) 2 (QPSK) 4 (16-QAM) 6 (64-QAM) 8 (256-QAM)
///   R       coding rate: 1/2, 2/3, 3/4, 5/6
///   T_sym   40 µs long GI / 36 µs short GI  (802.11ac's 4 µs/3.6 µs × 10)
/// ```
///
/// Cross-check: 2 MHz MCS0 long GI = 52 × 1 × 1/2 ÷ 40 µs = **650 kbit/s**, which is 802.11ac's
/// 20 MHz MCS0 (6.5 Mbit/s) divided by 10 — the downclocking relationship, recovered rather than
/// assumed. Single spatial stream only (`N_ss = 1`); both HaLow parts in this rig are 1×1.
///
/// ★ **MCS10 is below MCS0, not above MCS9.** It is 1 MHz-only BPSK 1/2 with **2× repetition**, so
/// it is half of MCS0 — the reach rate. Any code that treats the S1G MCS index as a monotone
/// ladder (as `mcs_for_rssi` does for 11n) is wrong at its top end; that is why
/// `RadioCapability::rate.max_mcs` must be 7 for these radios and MCS10 reached by name.
///
/// ⚠ **UNVERIFIED on hardware.** These are the standard's own numbers as derived above; nothing in
/// this rig has metered an S1G link against them. What *is* measured is the ordering they imply —
/// on the MM6108, walking injected MCS 0 → 7 took delivered throughput 2.15 → 7.06 Mbit/s.
pub fn s1g_phy_rate_bps(mcs: u8, bw_mhz: u8, short_gi: bool) -> Option<u32> {
    // (data subcarriers) per channel width.
    let n_sd: u32 = match bw_mhz {
        1 => 24,
        2 => 52,
        4 => 108,
        8 => 234,
        16 => 468,
        _ => return None,
    };
    // (bits per subcarrier, coding numerator, coding denominator) per MCS.
    let (bpscs, num, den): (u32, u32, u32) = match mcs {
        0 => (1, 1, 2),  // BPSK   1/2
        1 => (2, 1, 2),  // QPSK   1/2
        2 => (2, 3, 4),  // QPSK   3/4
        3 => (4, 1, 2),  // 16-QAM 1/2
        4 => (4, 3, 4),  // 16-QAM 3/4
        5 => (6, 2, 3),  // 64-QAM 2/3
        6 => (6, 3, 4),  // 64-QAM 3/4
        7 => (6, 5, 6),  // 64-QAM 5/6
        8 => (8, 3, 4),  // 256-QAM 3/4
        9 => (8, 5, 6),  // 256-QAM 5/6
        10 => (1, 1, 2), // BPSK 1/2 with 2x repetition — handled below
        _ => return None,
    };
    // The two NAMED holes, which must stay named — neither is recoverable from arithmetic.
    // MCS9 is undefined at 1 MHz for Nss=1; MCS10 exists only at 1 MHz.
    if (mcs == 9 && bw_mhz == 1) || (mcs == 10 && bw_mhz != 1) {
        return None;
    }
    // ★ ...and one hole that IS derivable, which the named list had missed. If
    // `n_sd * bpscs * num` does not divide by `den` the mode yields a fractional number of coded
    // bits per OFDM symbol, which is the standard's own reason for excluding it — so a
    // non-integer result means "undefined", not "round it".
    //
    // ⚠ The case this exists for: **MCS9 at 2 MHz**. S1G MCS9 is 11ac 20 MHz MCS9 downclocked, and
    // 11ac excludes MCS9 at 20 MHz for Nss=1 *and* Nss=2 — at S1G, 1 MHz and 2 MHz. Only the 1 MHz
    // half was written down, so `s1g_phy_rate_bps(9, 2, false)` computed 52*8*5/6 = 346 (truncated
    // from 346.67) and returned a confident 8.65 Mbit/s for a mode no radio can transmit.
    //
    // Note the asymmetry, because it is why the named list cannot be deleted: 1 MHz MCS9 is
    // 24*8*5/6 = 160 exactly, so divisibility does NOT catch it. The two rules are complementary,
    // not redundant, and `the_derived_holes_are_exactly_the_standards_holes` pins the union.
    let prod = u64::from(n_sd) * u64::from(bpscs) * u64::from(num);
    let den = u64::from(den);
    if prod % den != 0 {
        return None;
    }
    // Symbol time in nanoseconds: 802.11ac's 4 µs / 3.6 µs downclocked by 10.
    let t_sym_ns: u64 = if short_gi { 36_000 } else { 40_000 };
    let bits_per_symbol = prod / den;
    // MCS10 repeats each symbol twice, halving the rate.
    let reps: u64 = if mcs == 10 { 2 } else { 1 };
    Some((bits_per_symbol * 1_000_000_000 / (t_sym_ns * reps)) as u32)
}

/// One frame as injected: the (LP-framed) NDN payload, the PHY rate, and the
/// 802.11 address fields. Under the Tier-0 layout `dst`/`src` are the two halves of the
/// name's prefix-set filter (`addr1 ‖ addr2`) and `addr3` the ephemeral nonce; otherwise
/// broadcast + the default source. Never a host MAC.
#[derive(Clone, Debug)]
pub struct InjectFrame {
    pub payload: Bytes,
    /// What this transmit should achieve — a bearer-agnostic [`TxIntent`]. The
    /// backend resolves it to its own PHY rate ([`McsDescriptor::for_intent`] for
    /// 802.11); the seam itself no longer names an MCS.
    pub tx: TxIntent,
    /// 802.11 destination (`addr1`): a name-group MAC, a Tier-0 prefix-set filter's
    /// high half, or broadcast.
    pub dst: [u8; 6],
    /// 802.11 source (`addr2`): name-derived, a Tier-0 filter's low half, or [`DEFAULT_SRC`].
    pub src: [u8; 6],
    /// 802.11 `addr3`. `None` ⇒ the legacy layout (`addr3 = dst`, the BSSID slot). `Some`
    /// carries the **ephemeral source nonce** when `addr1 ‖ addr2` is a Tier-0 prefix-set
    /// filter (which consumes the source field), preserving per-transmitter RSSI keying
    /// (mac-addressing-doctrine §2). Never a host MAC.
    pub addr3: Option<[u8; 6]>,
    /// **The extra Blur region** (64 bits): the additive second projection that layers on top of the
    /// base 126-bit filter in `dst‖src‖addr3[0:4]`, giving the 190-bit Wi-Fi filter.
    ///
    /// ★ It is `extra`, not `addr4`, **on purpose**: there is exactly ONE wire mapping
    /// (`extra[0..6] → addr4`, `extra[6..8] → QoS Control`) and it lives in
    /// `ndn_frame_io::frame::build_dot11`. Naming the seam after a header field is what let two
    /// backends grow their own 3-address builder and silently drop the region; naming it after the
    /// *filter* means a backend cannot map it without going through the one builder.
    ///
    /// `None` ⇒ the base 3-address frame every bearer shares. Only the `RawNdn` arm of `build_dot11`
    /// consumes it; setting it flips the frame to ToDS=FromDS=1 QoS-Data+HTC.
    pub extra: Option<[u8; 8]>,
    /// **HT Control** (4 bytes): the exact-match fingerprint (24 bits, little-endian) plus the
    /// extra-region bitmap byte (`tier0::WIDE_PROFILE_MARKER`). Rides the +HTC/Order bit. `None` ⇒
    /// base layout. Set together with [`extra`](Self::extra) — the two are the pushed-header fields.
    pub htc: Option<[u8; 4]>,
}

impl InjectFrame {
    /// A broadcast frame from the default source — the addressing-agnostic case
    /// (every monitor receiver keeps it). Grouped faces fill `dst`/`src` instead.
    pub fn broadcast(payload: Bytes, tx: TxIntent) -> Self {
        Self {
            payload,
            tx,
            dst: BROADCAST,
            src: DEFAULT_SRC,
            addr3: None,
            extra: None,
            htc: None,
        }
    }
}

/// One frame as captured: the NDN payload recovered from the on-air frame, plus
/// what the headers told us. The NDN layer forwards on the *name* inside
/// `payload`; the rest are link-layer hints, never the addressing.
#[derive(Clone, Debug)]
pub struct CapturedFrame {
    pub payload: Bytes,
    /// Source address (`addr2`) — name-derived or the default source, never a
    /// host MAC. Reported upward as the (host-free) reassembly stream key.
    pub addr: Option<[u8; 6]>,
    /// Destination group (`addr1`) — the name-group MAC or broadcast. Used for
    /// the receive-side name pre-filter. Under Tier-0 this is the prefix-set filter's
    /// high half (`addr1`); the low half is [`addr`](Self::addr) (`addr2`), so
    /// `group ‖ addr` reconstruct the 12-byte filter.
    pub group: Option<[u8; 6]>,
    /// 802.11 `addr3` as received. Under the Tier-0 layout this is the sender's ephemeral
    /// source nonce (`addr1 ‖ addr2` being the prefix-set filter); `None` if the backend
    /// did not surface it (the legacy layout duplicates `dst` here, carrying no new info).
    pub addr3: Option<[u8; 6]>,
    /// **The extra Blur region** as received, reassembled from `addr4 ‖ QoS Control` (64 bits) by
    /// the one wire mapping in `ndn_frame_io::frame::parse_dot11`. `None` on a base 3-address frame
    /// or a backend that does not surface it — such a receiver simply never reads the extra bits
    /// (over-accept, never a false negative), which is what lets a 190-bit sender and a base-only
    /// receiver share one airspace.
    pub extra: Option<[u8; 8]>,
    /// **HT Control** as received: the exact-match fingerprint + the extra-region bitmap. `None`
    /// unless the frame carried the +HTC/Order bit. Whether the `extra` bytes may be *tested* is the
    /// bitmap's answer, not `extra.is_some()` — see `tier0::extra_regions_usable`.
    pub htc: Option<[u8; 4]>,
    /// Per-frame RSSI in dBm from radiotap, if measured.
    pub rssi_dbm: Option<i8>,
    /// MCS index the frame was received at, if radiotap reported it.
    pub mcs_index: Option<u8>,
    /// Hardware receive timestamp, if the backend latched one (radiotap TSFT on
    /// a monitor NIC, a NIC PHC, an on-chip counter). Carries its clock domain
    /// and honest precision; `None` when the backend has no hardware stamp (a
    /// software-timestamped or loopback frame). This is the named-time "Cut 1"
    /// seam — the input to time-transfer measurement.
    pub stamp: Option<LinkStamp>,
    /// Per-frame PHY quality from the receiver's own status report, when the backend surfaces it.
    ///
    /// `rssi_dbm` says how LOUD the frame arrived; this says how CLEAN it was, which is what
    /// actually predicts whether a rate decodes. `None` on backends that do not report it.
    pub phy: Option<PhyMetrics>,
}

/// Per-frame PHY quality, read from the chip's own receive status report.
///
/// Every field is independently optional: a chip may report some and not others, and a field the
/// hardware marks as "not measured" is `None` rather than a sentinel that reads like data.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct PhyMetrics {
    /// Signal-to-noise ratio of the strongest path, dB.
    pub snr_db: Option<i8>,
    /// Error-vector magnitude of the strongest path, dB (negative; closer to 0 is worse).
    pub evm_db: Option<i8>,
    /// Carrier frequency offset **residual**, Hz — see the warning on the parse site: this is
    /// `cfo_tail`, what is left AFTER the receiver's carrier tracking, not the static offset.
    pub cfo_hz: Option<i32>,
}

/// The radio behind a `MonitorWifiFace`: inject a frame at a chosen rate, and
/// yield captured frames. `recv_frame` has a single consumer (the face's reader
/// task); `inject` may be called concurrently and must synchronise internally.
#[async_trait]
pub trait FrameIo: Send + Sync + 'static {
    /// **What radio is this?** — the seam that stops a face from having to guess.
    ///
    /// ★ Added 2026-08-31 to close a capability leak that had survived two attempts to fix it.
    /// An `Arc<dyn FrameIo>` could not be asked what it was, so every constructor that takes one
    /// had to *invent* a [`RadioCapability`]. The production node did exactly that: it opened an
    /// RTL8822E — which implements [`RadioProfile`], [`RadioKnobs`] and [`RadioTime`] — and then
    /// built its face from the bare `dyn FrameIo`, so the radio's own profile was dropped and a
    /// placeholder declaring `max_mcs 9 / max_nss 2 / max_bw 2` went on air over a part that
    /// receives ONE spatial stream at MCS 7. Advertising streams a radio cannot receive is the
    /// MEASURED cause of a one-way link.
    ///
    /// The capability-complete path (`RadioBearer::from_open`) already existed and was documented
    /// as "precisely the leak the opener was created to close" — and had **zero** production
    /// callers, because using it meant threading four handles through every call site. So the fix
    /// for the unactuated contract was itself unactuated. This method makes the answer reachable
    /// from the one handle every face already holds, which is why it is here and not in a
    /// wider-but-optional interface.
    ///
    /// `None` means "I genuinely cannot say" and the caller's assertion stands. Every backend in
    /// `ndn-radio-drivers` implements [`RadioProfile`] (14 of 14 at the time of writing) and
    /// overrides this with `Some`; the default exists for test doubles and for any future backend
    /// that has no self-description.
    ///
    /// ⚠ Deliberately NOT named `capability`: [`RadioProfile::capability`] already exists, and a
    /// same-named method on a second trait would make every existing `backend.capability()` call
    /// site ambiguous.
    fn radio_capability(&self) -> Option<RadioCapability> {
        None
    }

    /// Transmit `frame.payload` on the medium for `frame.tx`. Fire-and-forget,
    /// unacknowledged — like all broadcast injection.
    async fn inject(&self, frame: InjectFrame) -> Result<(), FaceError>;

    /// Transmit a batch of frames, bundling runs that share dst/src/tx into one
    /// **A-MSDU** (link-layer bundling — one PHY preamble for many NDN packets,
    /// no Block-Ack needed) where the backend supports it. The default sends each
    /// individually; the RTL8812EU backend overrides this with A-MSDU. Used by
    /// the face-level batcher (`MonitorWifiFace::with_amsdu_batching`).
    async fn inject_batch(&self, frames: Vec<InjectFrame>) -> Result<(), FaceError> {
        for f in frames {
            self.inject(f).await?;
        }
        Ok(())
    }

    /// Inject one frame at an exact rate = [`set_rate`](Self::set_rate) then
    /// [`inject`](Self::inject). Derived; a driver need not (and should not) override it.
    async fn inject_at(&self, frame: InjectFrame, mcs: McsDescriptor) -> Result<(), FaceError> {
        self.set_rate(mcs)?;
        self.inject(frame).await
    }

    /// A batch, each frame at its own exact rate. **Overridable, and overridden**: the AF_PACKET
    /// backend implements this as real A-MSDU aggregation (one QoS-Data MPDU per RA, greedily
    /// packed) — the big airtime lever at S1G. So call this method rather than looping `set_rate` +
    /// `inject` yourself, or you get the default body and the aggregation silently disappears.
    ///
    /// It lives on `FrameIo` (not `WifiRadio`) precisely because faces hold `Arc<dyn FrameIo>`;
    /// see the note on [`WifiRadio`].
    async fn inject_batch_at(
        &self,
        frames: Vec<(InjectFrame, McsDescriptor)>,
    ) -> Result<(), FaceError> {
        for (f, mcs) in frames {
            self.set_rate(mcs)?;
            self.inject(f).await?;
        }
        Ok(())
    }

    /// **Place a frame on air at an absolute instant** on the clock `domain` — the hardware side of a
    /// named airtime lease. `target_tick` is a value in `domain` (a clock the radio exposes via
    /// [`RadioTime`]); the backend transmits when its own clock reaches it, so the frame lands in its
    /// slot without the host's sleep+inject jitter. This is the write-once seam the scheduler
    /// ([`FaceScheduler`]) actuates for any radio that offers hardware scheduling — the ESP32-C5 over
    /// its `T_INJECT_ABS`, an ath9k over quiet-time, a PIO/optical face over its timer.
    ///
    /// **Default = inject now**, ignoring the schedule: a radio with no scheduled-TX engine relies on
    /// the scheduler's software gate (sleep-until-slot) instead, so the default is correct for it — it
    /// is only ever called after that gate has already waited. A backend advertises the hardware path
    /// via [`RadioKnobs::tx_discipline`] returning [`TxDiscipline::ScheduledAt`]; the scheduler checks
    /// that and skips its own sleep when the radio can place the frame itself.
    async fn inject_at_clock(
        &self,
        frame: InjectFrame,
        _target_tick: u64,
        _domain: ClockDomainId,
    ) -> Result<(), FaceError> {
        self.inject(frame).await
    }

    /// **Place a frame on air `delay_us` from now**, timed on the *device's own* clock — the relative,
    /// reconcile-free sibling of [`inject_at_clock`](Self::inject_at_clock). Because the delay is applied
    /// against the radio's own timebase, no host↔device clock-offset conversion is needed, which is what
    /// lets [`FaceScheduler::slot_wait`] drive a hardware-scheduled bearer directly. `delay_us == 0` is
    /// inject-now. **Default = inject now** (a radio with no scheduled-TX engine ignores the delay; the
    /// scheduler's software gate has already waited for it). The ESP32-C5 backs it with its `T_INJECT_AT`.
    async fn inject_after(&self, frame: InjectFrame, _delay_us: u64) -> Result<(), FaceError> {
        self.inject(frame).await
    }

    /// **Does this backend actually place TX in time?** `false` (the default) means
    /// [`inject_after`](Self::inject_after) / [`inject_at_clock`](Self::inject_at_clock) fall
    /// through to plain [`inject`](Self::inject) — the frame goes out NOW and the delay is ignored.
    ///
    /// ⚠ This exists because a scheduler cannot safely infer the capability from
    /// [`RadioKnobs::tx_discipline`]. A backend that declares `ScheduledAt` without implementing
    /// the seam is worse than one that declares nothing: the caller hands it a delay, skips its own
    /// software gate believing the hardware will place the frame, and the default implementation
    /// transmits immediately — ungated, with no slot discipline at all.
    ///
    /// **Override this to `true` only in the same impl block that overrides `inject_after`.** The
    /// two must move together; declaring the discipline is not enough.
    fn schedules_tx(&self) -> bool {
        false
    }

    /// Await the next frame captured on the medium. A node never hears its own
    /// transmissions (half-duplex radio); the backend filters those.
    async fn recv_frame(&self) -> Result<CapturedFrame, FaceError>;

    /// Set the radio's current transmit rate as **state** — the exact 802.11 rate
    /// every subsequent [`inject`](Self::inject) transmits at, until changed. This is
    /// how the cognitive control plane actuates rate: one call, not a per-frame
    /// argument (*"rate is bearer state"*). The default is a no-op — a bearer that
    /// resolves [`InjectFrame::tx`] itself (LoRa/BLE) ignores it. A Wi-Fi backend
    /// stores it and its `inject` uses it, falling back to intent resolution before
    /// the first `set_rate`. Cheap and non-blocking (a stored value, no I/O).
    fn set_rate(&self, mcs: McsDescriptor) -> Result<(), FaceError> {
        let _ = mcs;
        Ok(())
    }

    /// The latest **mesh common-view** observation from a neighbour's hardware-TSF-stamped timing beacon
    /// (#74/#75): the transmitter's hardware TSF from the beacon body, paired with our hardware RX stamp
    /// of the same on-air event, restricted to *mesh* transmitters (a locally-administered BSSID — our
    /// ephemeral nonces, not infrastructure APs), plus the emitter's advertised network-time belief if
    /// the beacon carried one (for multi-hop composition). `count` increments per observation so a
    /// consumer can poll for a fresh one. Default `None` — only a backend that latches a hardware RX
    /// TSF and parses beacon timestamps returns anything.
    ///
    /// ## ⚠ A hardware latch is NOT sufficient, and this seam does not enforce that
    ///
    /// "latches a hardware RX TSF" was the whole common-view test until 2026-08-31, when two
    /// receivers of the same frames MEASURED it wrong: same latch point, 0.81-1.86 us and FLAT
    /// against the fit span on a crystal, 10.5-20.4 us and GROWING on an RC. The test is now BOTH
    /// halves — see [`FaceTimeProfile::can_common_view`], which ANDs the latch with
    /// [`ClockReference::holds_rate`] on one source.
    ///
    /// This method predates that and still carries only the latch half. **A backend may return
    /// observations while its own [`FaceTimeProfile::can_common_view`] is `false`** — that is not a
    /// contradiction to fix here, because the two answer different questions: this one is "did I
    /// pair a peer stamp with my own?", the profile's is "may anyone difference my counter with
    /// someone else's?". The second is the one a discipline loop needs, and it is the CONSUMER's to
    /// ask: a caller feeding these into a scheduler's clock (`FaceScheduler::ingest_common_view`)
    /// should gate on `FaceTimeProfile::derive(...).can_common_view` for the same radio first, and
    /// today no caller does. `Rtl8812auBackend` is the live instance — it serves this seam with an
    /// `Unknown` reference; see the ⚠ block on `impl RadioTime for Rtl8812auBackend` in
    /// `ndn-radio-drivers`.
    fn mesh_common_view(&self) -> Option<MeshCv> {
        None
    }
}

/// A mesh common-view observation (see [`FrameIo::mesh_common_view`]).
#[derive(Clone, Copy, Debug)]
pub struct MeshCv {
    /// The transmitter's hardware TSF (µs) from the beacon body.
    pub peer_tsf: u64,
    /// Our hardware RX stamp (RXTSFL, µs) of that same on-air frame.
    pub our_rxtsfl: u64,
    /// Increments per observation — poll to detect a fresh one.
    pub count: u64,
    /// The transmitter's BSSID (its ephemeral nonce; locally administered).
    pub bssid: [u8; 6],
    /// The transmitter's advertised network-time belief (#75), if the beacon carried one after its
    /// timestamp. `None` for a bare #74 beacon → the receiver treats the transmitter as a stratum-0 ref.
    pub belief: Option<ndn_time::RefBelief>,
}

// **`WifiRadio` was removed here** (#83). It had become `pub trait WifiRadio: FrameIo {}` — an
// empty marker with an empty impl on every backend, naming "a Wi-Fi radio" and constraining
// nothing. A marker that constrains nothing cannot be violated, so it carried no guarantee; it only
// split the world into radios a `dyn WifiRadio` caller could accept and radios it could not, which
// is a restriction with no compensating meaning.
//
// It emptied out because of a real trap: `inject_at`/`inject_batch_at` used to live on it, and a
// face holding `Arc<dyn FrameIo>` could not reach them, so it silently got a hand-rolled copy of
// the *default* body and missed `AfPacketBackend`'s A-MSDU-aggregating override (#82 part 1). The
// fix was to move them onto `FrameIo` — the object-safe seam a face actually holds must be the one
// carrying the overridable behaviour — which left this trait with nothing.
//
// "This is a Wi-Fi radio" is now answered by `RadioCapability::kind`, which is data the cognition
// layer can read, rather than a type-level assertion nothing checks.

// ---------------------------------------------------------------------------
// Radio control plane: the stateful-knob seam + the capability descriptor.
// ---------------------------------------------------------------------------

/// Channel bandwidth, uniform across backends. The numeric `code()` matches the
/// cognition plane's `TxParams.bw` / `RadioCapability.max_bw` encoding and the
/// RTL `ChannelBw` discriminants: `0=20, 1=40, 2=80, 3=10MHz, 4=5MHz`.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Default)]
pub enum Bandwidth {
    /// 20 MHz (standard).
    #[default]
    Bw20,
    /// 40 MHz.
    Bw40,
    /// 80 MHz (VHT).
    Bw80,
    /// 10 MHz narrowband (non-standard; longer range / lower rate).
    Nb10,
    /// 5 MHz narrowband.
    Nb5,
}

impl Bandwidth {
    /// Numeric code shared with `TxParams.bw` / `RadioCapability.max_bw`.
    pub fn code(self) -> u8 {
        match self {
            Bandwidth::Bw20 => 0,
            Bandwidth::Bw40 => 1,
            Bandwidth::Bw80 => 2,
            Bandwidth::Nb10 => 3,
            Bandwidth::Nb5 => 4,
        }
    }

    /// Inverse of [`code`](Self::code); unknown codes fall back to 20 MHz.
    pub fn from_code(c: u8) -> Self {
        match c {
            1 => Bandwidth::Bw40,
            2 => Bandwidth::Bw80,
            3 => Bandwidth::Nb10,
            4 => Bandwidth::Nb5,
            _ => Bandwidth::Bw20,
        }
    }

    /// **The physical channel width in MHz — the only correct ordering axis.**
    ///
    /// ☠ [`code`](Self::code) is a wire/register encoding and is **NOT ordered by width**:
    /// `Bw20=0, Bw40=1, Bw80=2, Nb10=3, Nb5=4`, so the two NARROWBAND codes sort *above* 80 MHz.
    /// Anything that compares or arithmetics on the code is silently wrong for narrowband:
    /// cognition's contention response was `bw = bw.saturating_sub(1)`, which **widens** 5 MHz to
    /// 10 MHz, and its VHT inference was `max_bw >= 2`, which reads a 10 MHz channel as
    /// VHT-capable. Both were latent only because **no backend declares code 3 or 4** — the axis
    /// punishes honesty, since declaring narrowband would make the planner pick 5 MHz as its
    /// *default* width and then "narrow" toward 80. Use `mhz()` for every comparison.
    pub fn mhz(self) -> u16 {
        match self {
            Bandwidth::Bw20 => 20,
            Bandwidth::Bw40 => 40,
            Bandwidth::Bw80 => 80,
            Bandwidth::Nb10 => 10,
            Bandwidth::Nb5 => 5,
        }
    }

    /// The next narrower width, or `None` at the narrowest — the correct "back off under
    /// contention" step, replacing arithmetic on [`code`](Self::code).
    pub fn narrower(self) -> Option<Bandwidth> {
        match self {
            Bandwidth::Bw80 => Some(Bandwidth::Bw40),
            Bandwidth::Bw40 => Some(Bandwidth::Bw20),
            // ☠ **STOPS AT 20 MHz — narrowband is not reachable by decrement.**
            //
            // This returned `Some(Nb10)` for one day and it was a LIVE BUG, introduced by the
            // same change that fixed the code-vs-width axis. Chain: the AR9271 declares
            // `max_bw: 0`; cognition takes `bw = cap.max_bw()` and narrows on any channel at or
            // above `busy_high` (default 50%); `Bw20.narrower()` handed back `Nb10`;
            // `ath9k_htc::set_channel` computes `want_ht40 = matches!(bw, Bw40)`, sees no change,
            // and returns `Ok(())` — so `apply_knobs` records 10 MHz as APPLIED and, because it
            // dedupes on the last value, never asks again. The radio sits at 20 MHz forever while
            // the control plane and the bandit's airtime proxy believe 10.
            //
            // 5/10 MHz are a distinct PHY mode a radio must actually program, not one step down a
            // ladder. No backend declares them, and one that did would need an explicit request —
            // never a contention response that walked off the end of the Wi-Fi widths.
            Bandwidth::Bw20 => None,
            // A radio ALREADY in narrowband may step down within it.
            Bandwidth::Nb10 => Some(Bandwidth::Nb5),
            Bandwidth::Nb5 => None,
        }
    }
}

#[cfg(test)]
mod tx_intent_predicate {
    use super::*;

    /// ★ The basic-rate doctrine, as a test. It was documented four times in four backends and
    /// absent from ten of fifteen; this pins the decision itself so the per-backend encodings have
    /// one thing to agree with.
    #[test]
    fn only_most_robust_demands_the_basic_rate() {
        assert!(
            TxIntent::ROBUST.needs_basic_rate(),
            "discovery/control must be universally decodable"
        );
        for r in [Reliability::Balanced, Reliability::Throughput] {
            let i = TxIntent {
                reliability: r,
                reach: Reach::Broadcast,
            };
            assert!(
                !i.needs_basic_rate(),
                "{r:?} must NOT be forced to the basic rate — that would cap the link at 6 Mbps"
            );
        }
    }
}

#[cfg(test)]
mod bandwidth_axis {
    use super::*;

    /// ★ Pins the trap: the numeric code is NOT the width order. If someone "tidies" `code()` into
    /// ascending width they will silently change a wire/register encoding; if someone compares
    /// codes they get narrowband backwards. This test exists so both mistakes fail loudly.
    #[test]
    fn the_code_axis_is_not_the_width_axis() {
        assert!(
            Bandwidth::Nb5.code() > Bandwidth::Bw80.code(),
            "codes are a wire encoding; 5 MHz sorting above 80 MHz is the trap this pins"
        );
        assert!(
            Bandwidth::Nb5.mhz() < Bandwidth::Bw80.mhz(),
            "mhz() is the ordering axis and must be monotone in real width"
        );
        // Narrowing must always reduce width, from every starting point.
        for b in [
            Bandwidth::Bw80,
            Bandwidth::Bw40,
            Bandwidth::Bw20,
            Bandwidth::Nb10,
            Bandwidth::Nb5,
        ] {
            if let Some(n) = b.narrower() {
                assert!(
                    n.mhz() < b.mhz(),
                    "narrower() widened {b:?} -> {n:?} ({} -> {} MHz)",
                    b.mhz(),
                    n.mhz()
                );
            }
        }
        // ★ Narrowing must never LEAVE the Wi-Fi widths: reaching Nb10 from Bw20 made cognition
        // request a mode the radio cannot program, which `ath9k_htc::set_channel` then accepted
        // with `Ok(())`. Narrowband is entered deliberately or not at all.
        assert_eq!(
            Bandwidth::Bw20.narrower(),
            None,
            "narrowing past 20 MHz must not fall into narrowband"
        );
        assert_eq!(
            Bandwidth::Nb10.narrower(),
            Some(Bandwidth::Nb5),
            "a radio already in narrowband may still step down within it"
        );

        // The old arithmetic, shown failing, so the reason is not forgotten.
        let five = Bandwidth::Nb5.code();
        assert_eq!(
            Bandwidth::from_code(five.saturating_sub(1)),
            Bandwidth::Nb10,
            "code-1 on 5 MHz yields 10 MHz — the 'narrowing' that widens"
        );
    }
}

/// The uniform stateful-knob surface every userspace radio backend exposes to
/// the named-radio control plane. Implementors are wrapped behind a
/// `RadioActuators` adapter (see `control.rs`) so a single generic actuator can
/// drive any radio.
///
/// Only [`set_channel`](Self::set_channel) is required — a radio that cannot at
/// least tune is not useful. **Every other knob defaults to an `Unsupported` refusal**, so a port
/// can land RX/TX first and grow contention/power control later *without the unported knobs
/// silently reporting success*.
///
/// ★ That default was `Ok(())` until 2026-08-31, on 7 of 18 knobs, and the cost is recorded on
/// [`set_tx_power`](Self::set_tx_power): a silent success made the MT7612U and MT7921AU "accept"
/// every power back-off cognition asked for although neither has a power actuator, `apply_knobs`
/// recorded the request as applied, and the bandit's footprint term was rewarded for a spatial
/// reuse that never physically happened. The same shape was live on `set_tx_csd` (un-overridden by
/// 12 of 13 backends) and `set_tx_hold` (11 of 13) — the latter is the slot MAC's queue gate, so
/// on those parts the queue bled into the next owner's slot while the scheduler believed it had
/// been held. `apply_knobs` already degrades per knob and updates its cache only on success, so an
/// honest default makes the false "applied" record impossible for free.
///
/// The one deliberate exception is
/// [`configure_name_filter`](Self::configure_name_filter), whose contract names a real fallback
/// (the host filters in software), so its `Ok(())` is a behaviour rather than a pretence. Per-frame
/// rate/STBC/LDPC/short-GI/NSS is NOT here; that travels with each
/// [`InjectFrame`]`.mcs` on the data plane.
pub trait RadioKnobs: Send + Sync {
    /// Tune to `channel` at bandwidth `bw`. Returns an error if the radio cannot
    /// reach that channel/width (e.g. a port that has only captured one channel).
    fn set_channel(&self, channel: u8, bw: Bandwidth) -> Result<(), FaceError>;

    /// **Set TX power, and say what that meant.** ([`PowerRequest`] in, [`AppliedPower`] out.)
    ///
    /// ☠ **This signature is the 2026-09-03 fix.** It used to be `set_tx_power(idx: u32) ->
    /// Result<(), FaceError>`, and on the RTL8812AU that one call meant **two different physical
    /// powers** — decided by whether `load_tx_power_info()` had run three calls earlier:
    ///
    /// * calibration loaded → `index_base(path, rate, ch) + (idx − 63)` ≈ **27** on the ch6 adapter;
    /// * not loaded → the function fell through to `set_tx_power_raw(idx)` → a flat **63**.
    ///
    /// Same call, same argument, same `Ok(())`, **two different power regimes**. ⚠ The size of the gap is channel-dependent and UNVERIFIED (base 27 on ch6, 44 on ch149; ~0 dB measured at ch149, 2026-09-04). The figures below came from a KERNEL witness later shown to be broken:
    /// raw 63 → 2301 frames at −85.6 dBm, raw 55 → **0**. The bench examples ran hot and the node
    /// binary ran at the fused base, so *they were not the same transmitter* — and nothing in the
    /// code, the logs or the capability declaration said so.
    ///
    /// **LAW 2 — no power knob may fall through to another regime.** With the calibrated scale
    /// asked for and no calibration resolved, an implementation returns `Err`
    /// ([`bringup::NO_CALIBRATION_MSG`]) **naming [`PowerRequest::Raw`]**. It never silently
    /// becomes raw.
    ///
    /// **LAW 3 — a knob may not branch on the environment.** Everything that used to be read from
    /// inside an implementation (`NDN_AU_TXAGC12`, which silently turned 10 register writes into
    /// 24) is a field on the request ([`RateGroupPolicy`]) and is echoed in the returned
    /// [`AppliedPower`].
    ///
    /// The scale is **not renumbered**: on a fused part `max_tx_power` *is* the regulatory base on
    /// the calibrated scale, and `RadioPolicy::decide_power` (`max − backoff`) stays untouched. The
    /// physical point is reported in [`PowerReference::FusedBase`], which is a fact rather than a
    /// renumbering — see the contract's §2.1 for why the tempting `max_tx_power = 27` reintroduces
    /// the same defect via the cure.
    ///
    /// ★ **Default: `Unsupported`, not `Ok(())`.** It used to be a silent success, which made this
    /// the single most misleading seam in the crate: the MT7612U and MT7921AU have **no power
    /// actuator at all**, yet accepted every back-off cognition asked for. `apply_knobs` then
    /// recorded the request as applied, the contextual bandit's footprint term was rewarded for a
    /// spatial reuse that never physically happened, and nothing upstream could tell. A knob that
    /// cannot act must say so — that is the whole point of a capability seam. See
    /// [`RadioCapability::power_actuated`] for the declarative half of the same fact, and
    /// [`PowerRequest::NoActuator`] for the request that states it.
    ///
    /// ⚠ **Any on-air A/B spanning this change is invalid.** `apply_knobs` now records the
    /// *applied* power (what the radio said it did) where it used to record the *requested* index.
    /// Re-baseline rather than comparing across it.
    fn set_tx_power(&self, _req: PowerRequest) -> Result<AppliedPower, FaceError> {
        Err(bringup::power_unsupported(
            "radio exposes no TX-power control",
        ))
    }

    /// Set TX power on the **absolute dBm scale**, returning the power actually
    /// applied (which may be clamped below `dbm` by a regulatory/BCF table in the
    /// driver or firmware — always believe the returned value, not the request).
    ///
    /// This is the portable power knob: unlike [`set_tx_power`](Self::set_tx_power)
    /// it means the same thing on every bearer, so cognition can reason in link
    /// budget (dB of margin) rather than in chip register units. A radio advertises
    /// support via [`RadioCapability::tx_power_dbm`]; the two knobs are alternatives,
    /// and a backend implements whichever its hardware actually exposes.
    ///
    /// Default: `Unsupported`, so a caller can fall back to the index scale.
    fn set_tx_power_dbm(&self, _dbm: i8) -> Result<i8, FaceError> {
        Err(FaceError::Io(std::io::Error::new(
            std::io::ErrorKind::Unsupported,
            "radio exposes no absolute dBm TX-power control",
        )))
    }

    /// Per-format OFDM PPDU counters `(ok, err)`, if the radio reports them. Default: `None`.
    ///
    /// The receive-side loss a frame count cannot show. A missing frame is invisible from the RX
    /// side — it simply never arrives — so "17% did not turn up" cannot distinguish a collision
    /// from a marginal link from a transmitter that never sent. `err` counts PPDUs the PHY BEGAN to
    /// demodulate and failed, which is exactly the collision/marginal-decode signature.
    ///
    /// Free-running and wrapping: take differences over an interval, never absolute values.
    fn read_ofdm_counters(&self) -> Result<Option<(u16, u16)>, FaceError> {
        Ok(None)
    }

    /// **Did the MAC even ask the baseband to transmit?** Returns `(tx_en, tx_on)` — free-running
    /// counters of MAC→baseband transmit *requests* and baseband→RF *keys*. Default: `None`.
    ///
    /// This is the register read that separates "we never asked" from "we asked and the air ate
    /// it", which no frame count or delivery ratio can distinguish. On the RTL8733BU it settled
    /// four questions in one session that had each survived days of on-air guessing:
    ///
    /// * a hardware TSF comparator fired 20/20 while `tx_en` moved **+0** — the trigger was real
    ///   but the queue behind it was empty, which no amount of on-air measurement could have shown;
    /// * 50 ordinary injects moved it **+50**, calibrating the counter exactly;
    /// * ordinary injection transmits **1.00** times per logical frame — no retry waste, refuting
    ///   a plausible airtime theory;
    /// * a reserved-page release path was **lossy below a ~1 ms inter-release gap** (0.26 → 1.00),
    ///   a rate limit invisible to delivery ratios because the frames were never sent at all.
    ///
    /// Reach for this FIRST on any "it does not transmit" question; it collapses the search space
    /// from the whole radio to one side of the MAC/PHY boundary.
    fn read_tx_counters(&self) -> Result<Option<(u16, u16)>, FaceError> {
        Ok(None)
    }

    /// **How hard this radio should fight for the medium** — the contention window, as a posture
    /// rather than a register value.
    ///
    /// ★ Contention is a *knob*, and on a slotted named-data MAC it is the actuator for the slot
    /// decision. The claimable name-slot ([[token-concept-named-radio]]) makes a node the grant
    /// holder for `(name, t mod N)`; inside a slot it owns, CSMA backoff buys nothing — the
    /// schedule already provides collision freedom — and every microsecond of it is pure
    /// overhead. Outside its slot, or in an open slot being contested by demand, the node should
    /// back off normally so the election works. One knob, driven by the scheduler.
    ///
    /// MEASURED on the MT7921AU (ch36, VHT MCS9 2SS/80 MHz/SGI, 9000 B): moving `cw_min` from the
    /// firmware default (exponent 5, CW = 31 slots, ~140 µs average backoff) to exponent 2
    /// (CW = 3) took offered throughput from **382 to 416 Mbit/s**. On a medium the node has
    /// been granted, that ~140 µs is the single largest per-frame cost there is.
    ///
    /// ⚠ **This is not "turn contention off".** Zeroing the window is measured *harmful*: on the
    /// MT7612U, writing CW exponent 0 into the EDCA registers dropped throughput 5.7× and left
    /// the MAC unable to transmit at all, beyond software recovery. [`Owned`](ContentionPosture::Owned)
    /// therefore means *minimal sane* backoff, not none, and implementors must clamp.
    ///
    /// ★ **Read [`ContentionApplied::slot_us`], do not assume 9 µs.** Every term of the budget is
    /// counted in slots and the slot is a per-part fact: MEASURED, the MT7612U boots at **20 µs**
    /// while the MT7610U, MT7921AU and the Realtek parts run 9. The same window exponent is
    /// therefore 67 µs of average backoff on one radio and 150 µs on another.
    ///
    /// MEASURED on the RTL8812AU (ch36, contended, 200 B frames): `Shared` → `Owned` moved the
    /// per-frame period 779 → 706 µs, a **73 µs** saving against the **72 µs**
    /// [`ContentionApplied::medium_access_us`] predicts. The budget is an accurate account of this
    /// knob, not an approximation of it — but it is a *fixed* cost, so it is +10 % on short frames
    /// and under 1 % on a full-size VHT80 PPDU. Weigh it against frame length.
    ///
    /// Default: `Unsupported`, so a radio without an EDCA surface says so rather than pretending.
    fn set_contention(&self, _posture: ContentionPosture) -> Result<ContentionApplied, FaceError> {
        Err(FaceError::Io(std::io::Error::new(
            std::io::ErrorKind::Unsupported,
            "radio exposes no contention-window control",
        )))
    }

    /// **Hold or release transmissions at the MAC**, if the radio can. Default: no-op.
    ///
    /// The hardware side of a slot MAC. A software gate can only stop *us calling inject*; frames
    /// already queued in the MAC still go out, and land in whoever owns the next slot — the exact
    /// bleed a slot schedule exists to prevent, charged to a name that did not cause it.
    ///
    /// ⚠ MEASURED SEMANTICS on the RTL8733BU (`REG_TXPAUSE`), which a caller must design around:
    ///
    /// * It **HOLDS, it does not drop.** A held queue drains when released — ~20 KB on that part —
    ///   so this is only safe where the burst lands in a window you own. In a slot MAC that is
    ///   exactly right (hold while waiting, release at the start of our own turn); as a general
    ///   "be quiet now" it is not, because the quiet is repaid with interest.
    /// * **~126 us per write** over USB, so it shapes windows, not microslots. Against a 20 ms
    ///   slot that is 0.6%; against a 250 us one it is most of the budget.
    /// * Throughput under a 50% duty cycle measured **13.9%**, not 50% — queue drain and refill
    ///   dominate. Budget from the measurement, not from the duty.
    ///
    /// A radio with no such gate leaves the default and relies on the scheduler's software wait,
    /// which is correct for it — the default must never be "pretend it worked".
    fn set_tx_hold(&self, _hold: bool) -> Result<(), FaceError> {
        Err(FaceError::Io(std::io::Error::new(
            std::io::ErrorKind::Unsupported,
            "radio exposes no MAC transmit-hold gate",
        )))
    }

    /// Enable cyclic-shift diversity on the second chain (1-stream robustness via
    /// antenna diversity). Default: no-op (not supported / single-chain).
    fn set_tx_csd(&self, _on: bool) -> Result<(), FaceError> {
        Err(FaceError::Io(std::io::Error::new(
            std::io::ErrorKind::Unsupported,
            "radio exposes no cyclic-shift-diversity control (single chain, or not ported)",
        )))
    }

    /// **The clear-channel threshold, in true dBm** — above `l2h` the medium counts as busy, below
    /// `h2l` it counts as idle again (the hysteresis pair).
    ///
    /// ★ This is the *other* half of spatial reuse and the only RX-side knob in the fleet that is
    /// already denominated in dBm rather than in chip units. Cognition should move it **together
    /// with** the power decision: backing off power without raising the defer threshold shrinks who
    /// hears this node while leaving it just as deferential to everyone else, and the concurrency
    /// never appears. A node that trims 10 dB of transmit power and raises its floor by the same
    /// 10 dB has actually claimed reuse; one that only does the first has just reduced its own
    /// reach.
    ///
    /// Default: `Unsupported`. A radio that cannot express a threshold in dBm must say so rather
    /// than accept a number it will silently reinterpret.
    fn set_edcca_threshold_dbm(&self, _l2h: i8, _h2l: i8) -> Result<(), FaceError> {
        Err(FaceError::Io(std::io::Error::new(
            std::io::ErrorKind::Unsupported,
            "radio exposes no dBm clear-channel threshold",
        )))
    }

    /// Ignore EDCCA / listen-before-talk so TX proceeds under channel contention. Default: no-op.
    /// (A LoRa radio maps this to its LBT toggle.)
    fn set_edcca_ignore(&self, _on: bool) -> Result<(), FaceError> {
        Err(FaceError::Io(std::io::Error::new(
            std::io::ErrorKind::Unsupported,
            "radio exposes no ED-CCA threshold control",
        )))
    }

    /// **Switch the radio's modulation**, returning the mode actually in effect.
    ///
    /// ★ The capability this whole trait was missing. Modulation is a runtime command
    /// (`SetPacketType` on an LR20xx; `RegOpMode` on an SX127x), so it is a *knob* cognition
    /// actuates — the same shape as MCS or spreading factor — and not a property of the node. A
    /// backend that hard-codes one modulation at bring-up and reports it as its identity has
    /// converted a dial into a fact.
    ///
    /// **Believe the return, not the request**, exactly as with
    /// [`set_tx_power_dbm`](Self::set_tx_power_dbm): the chip may refuse a mode the radio
    /// advertises (a band/PA combination it cannot serve, a mode its calibration was not built
    /// for), and the honest answer is the mode it is running now.
    ///
    /// ⚠ **A successful switch invalidates the radio's whole [`RadioCapability`].** Payload cap,
    /// rate model, spreading-factor span, scheduling granularity and even the band are per-PHY, so
    /// a caller must re-read [`RadioProfile::capability`] afterwards and replace what it held —
    /// never patch the field it thinks changed. It must also re-assert its intended
    /// channel/rate/power: none of them survives a modulation change.
    ///
    /// Default: `Unsupported`, so a single-modulation radio refuses rather than pretending.
    fn set_phy(&self, _mode: PhyMode) -> Result<PhyMode, FaceError> {
        Err(FaceError::Io(std::io::Error::new(
            std::io::ErrorKind::Unsupported,
            "radio exposes no runtime modulation control",
        )))
    }

    /// **Install an autonomous frequency-hopping plan**: the carrier list (Hz), how long to dwell
    /// on each (`period`, counted in [`HopCapability::period_unit`]), and whether to arm it.
    ///
    /// Distinct from [`set_channel`](Self::set_channel) + a software schedule, and distinct from
    /// [`RadioCapability::retune_us`]: this hands the *radio* a list it walks by itself — on the
    /// LR20xx and the SX1276, within a single packet, at a dwell no host command could reach. A
    /// caller checks [`RadioCapability::hop`] first: `None` means the radio has no sequencer, and
    /// [`HopCapability::max_list_len`] bounds `freqs_hz`.
    ///
    /// Default: `Unsupported`. A radio with no sequencer must refuse, because a silent success
    /// would leave a planner believing its frames are spread across a band they never left.
    fn set_hop_plan(
        &self,
        _ctrl: HopControl,
        _period: u16,
        _freqs_hz: &[u32],
    ) -> Result<(), FaceError> {
        Err(FaceError::Io(std::io::Error::new(
            std::io::ErrorKind::Unsupported,
            "radio exposes no autonomous frequency-hopping plan",
        )))
    }

    /// **Set the receive front end's sensitivity posture** — [`RxGain::Auto`] (the part's own
    /// default/AGC) or [`RxGain::Boosted`] (its highest manual gain).
    ///
    /// A posture rather than a number **because the wire already is one**: every firmware in the
    /// 7E-A5 fleet defines `CMD_SET_RX_GAIN` as a single boolean byte, and the LR2021 firmware
    /// records why it refuses to expose its chip's 0..13 manual ladder through that byte — `1`
    /// would mean "boosted" on one node and the *lowest* manual step on another, an inversion this
    /// rig has already paid for once on a TX-power knob. There is consequently no scale to
    /// reconcile between parts, only the two positions below, and those do mean the same thing
    /// everywhere.
    ///
    /// ⚠ It is **not** a link-budget knob: see [`RxGain::Boosted`]. Reason in postures, not dB.
    ///
    /// Default: `Unsupported`.
    fn set_rx_gain(&self, _gain: RxGain) -> Result<(), FaceError> {
        Err(FaceError::Io(std::io::Error::new(
            std::io::ErrorKind::Unsupported,
            "radio exposes no receive-gain control",
        )))
    }

    /// Set the LoRa **spreading factor** (7–12) — the sub-GHz reach/rate dial, the direct analogue
    /// of Wi-Fi MCS: each step up trades throughput for link budget (≈ doubling airtime, ≈ +2.5 dB
    /// sensitivity). No-op default; only a [`RadioKind::Lora`] radio acts on it. Cognition drives
    /// this the way it drives MCS — down for close/bulk, up for far/urgent.
    fn set_spreading_factor(&self, _sf: u8) -> Result<(), FaceError> {
        Err(FaceError::Io(std::io::Error::new(
            std::io::ErrorKind::Unsupported,
            "radio has no spreading factor (not a LoRa PHY)",
        )))
    }

    /// Set the LoRa **coding rate** (`1`=4/5 … `4`=4/8) — a robustness/FEC dial (more coding = more
    /// resilience to interference, at the cost of airtime). No-op default.
    fn set_coding_rate(&self, _cr: u8) -> Result<(), FaceError> {
        Err(FaceError::Io(std::io::Error::new(
            std::io::ErrorKind::Unsupported,
            "radio has no settable coding rate",
        )))
    }

    /// Set the LoRa channel **bandwidth in kHz** (125 / 250 / 500) — a rate/range axis orthogonal to
    /// spreading factor (wider = faster but noisier / shorter). No-op default.
    fn set_bandwidth_khz(&self, _khz: u32) -> Result<(), FaceError> {
        Err(FaceError::Io(std::io::Error::new(
            std::io::ErrorKind::Unsupported,
            "radio has no kHz-granular bandwidth control (see set_channel for Wi-Fi widths)",
        )))
    }

    /// The transmit-timing discipline this radio can *promise* (named-time Cut 2) — the capability
    /// beacon slots / the URLLC lane / TSCH-by-name read to know how tightly airtime is bounded.
    /// Default [`TxDiscipline::BestEffort`]; a radio that can suppress CSMA backoff on owned
    /// spectrum (EDCCA-ignore + single-frame injection) reports [`TxDiscipline::PromptBounded`].
    fn tx_discipline(&self) -> TxDiscipline {
        TxDiscipline::BestEffort
    }

    /// Read a **frame-free occupancy counter**: a free-running hardware count of
    /// channel activity the radio maintains without the host decoding frames
    /// (#30). Two reads across a window, differenced, give a frames/s rate the
    /// cognition plane maps to channel-busy% (`ChannelOccupancy::from_activity`
    /// in `ndn-radio-cognition`). Returns `Ok(None)` by default — a radio that
    /// can't sense occupancy this way is honest about it, and the sampler skips
    /// it. On the 8812au this is `REG_RXERR_RPT` (`0x0664`), validated to track
    /// the decoded-frame rate ~1:1.
    fn read_channel_activity(&self) -> Result<Option<u16>, FaceError> {
        Ok(None)
    }
}

/// How hard a radio should compete for the medium — the input to
/// [`RadioKnobs::set_contention`].
///
/// Stated as a posture rather than a contention-window exponent because the mapping is
/// chip-specific (a firmware command on connac2, two different register blocks on mt76x02) and
/// because the *scheduler* knows the situation while only the driver knows the safe range.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Default)]
pub enum ContentionPosture {
    /// **This node holds the transmit grant for the current slot.** Back off as little as the
    /// hardware safely allows: collision freedom is coming from the schedule, so CSMA backoff is
    /// pure overhead. MEASURED worth ~9% throughput on the MT7921AU and far more on a busy
    /// channel, where the default window is most of the per-frame cost.
    ///
    /// ⚠ Never "no backoff" — see the warning on [`RadioKnobs::set_contention`].
    Owned,
    /// **Ordinary contention**: the standards-default window. The right posture in an open or
    /// claimable slot where several names may have data and the CCLF election needs collisions to
    /// resolve, and the only honest posture on a channel shared with other networks.
    #[default]
    Shared,
    /// **Deliberately yield**: a larger window than default, so other transmitters win the medium.
    /// For coexistence, for letting a starving neighbour through (the anti-starvation case the
    /// slot MAC exists to fix), and for politeness on a band shared with a co-banded bearer.
    Yielding,
}

/// What a radio actually applied for a [`ContentionPosture`] — believe this, not the request.
///
/// The posture is advisory: a driver clamps to what its silicon tolerates, and the caller needs
/// the real numbers to reason about airtime. `cw_min`/`cw_max` are **exponents** (CW = 2^n − 1).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct ContentionApplied {
    /// Contention-window minimum exponent actually programmed.
    pub cw_min: u8,
    /// Contention-window maximum exponent actually programmed.
    pub cw_max: u8,
    /// Arbitration inter-frame spacing, in slots.
    pub aifs: u8,
    /// TXOP limit in 32 µs units; 0 = one PPDU per medium acquisition.
    pub txop: u16,
    /// The slot time the MAC is actually counting backoff in, in microseconds.
    ///
    /// ★ Not a constant. MEASURED 2026-08-28: the MT7612U boots with a **20 µs** slot
    /// (`MT_BKOFF_SLOT_CFG` = 0x114) while the MT7610U ships 9 (0x209) and the MT7921AU programs
    /// 9. Backoff, AIFS and the whole DCF budget scale linearly in this number, so a window
    /// exponent alone does not say what a posture costs — an exponent of 4 is 67 µs of average
    /// backoff on one part and 150 µs on another. Every airtime budget must read this field
    /// rather than assume the 802.11a short slot.
    pub slot_us: u8,
    /// The average backoff this window implies, in microseconds, at [`Self::slot_us`] — the number
    /// the scheduler actually cares about, precomputed so every caller does not re-derive it.
    pub avg_backoff_us: u32,
}

impl ContentionApplied {
    /// Average backoff for a `cw_min` exponent at a given slot: `((2^n − 1) / 2) × slot_us`.
    pub const fn avg_backoff_us_at(cw_min: u8, slot_us: u8) -> u32 {
        let cw = (1u32 << cw_min) - 1;
        cw * slot_us as u32 / 2
    }

    /// Average backoff at the 802.11a short slot. Prefer [`Self::avg_backoff_us_at`] with the
    /// slot the part actually reports — this shorthand is only correct on a 9 µs part.
    pub const fn avg_backoff_us_for(cw_min: u8) -> u32 {
        Self::avg_backoff_us_at(cw_min, 9)
    }

    /// The full DCF budget one medium acquisition costs: `SIFS + AIFSN×slot + E[backoff]`.
    /// This is the quantity a slot/token scheduler must reserve per transmission, and the one
    /// that MEASURED at 225 µs of the ~248 µs fixed per-PPDU cost on the MT7612U.
    pub const fn medium_access_us(&self) -> u32 {
        SIFS_US + (self.aifs as u32) * (self.slot_us as u32) + self.avg_backoff_us
    }
}

/// Short inter-frame space for OFDM PHYs, in microseconds. The one genuinely fixed term in the
/// DCF budget — slot time and contention window are both knobs, this is not.
pub const SIFS_US: u32 = 16;

/// What the transmit path can *promise* about when a frame leaves the antenna — a named-time
/// Cut-2 capability the protocol reads, never a chipset register. A beacon slot or the URLLC lane
/// asks for a discipline and reads its bound; *how* a backend delivers it (EDCCA-ignore on owned
/// spectrum, a hardware scheduled-TX engine) stays below this seam, exactly as an MCS stays below
/// [`TxIntent`].
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum TxDiscipline {
    /// No timing promise — kernel Wi-Fi, or any congested medium where CSMA backoff is unbounded.
    BestEffort,
    /// The frame leaves within `max_delay_ns` of the request — what EDCCA-ignore + single-MPDU
    /// injection deliver on owned spectrum (bounded contention).
    PromptBounded {
        /// Upper bound, ns, from request to on-air.
        max_delay_ns: u64,
    },
    /// The frame leaves at a *scheduled instant*, accurate to `granularity_ns` — a PIO/optical face
    /// or a future scheduled-TX radio (the `LatchPoint::ScheduledTx` class).
    ScheduledAt {
        /// Scheduling granularity, ns.
        granularity_ns: u64,
    },
}

/// A radio's named-time surface: which link clocks it exposes and how to read the readable
/// ones. Implemented per backend so `ndn-time` can, uniformly across heterogeneous radios,
/// learn the domain RX [`LinkStamp`]s live in, that clock's honest quality, and compute a
/// frame's age via a read-now clock — without special-casing any backend.
///
/// Grounded in hardware reality: a radio may expose several link clocks of different quality
/// (an always-on free-run per-frame RX stamp, a gated/beacon-resynced port TSF, a host
/// software stamp). A backend enumerates them via [`RadioTimeSource`] rather than pretending
/// to have one canonical TSF. Default impl reports nothing — a port that has not wired up its
/// timekeeping yet is honest about having none.
pub trait RadioTime: Send + Sync {
    /// The link clocks this radio exposes, best-first (the per-frame RX-stamp clock first).
    ///
    /// ★ Each source states **two** independent things: where it latches (`kind`/`latch`, and
    /// `precision_ns` as the per-stamp half-width) and what its counter runs on
    /// ([`RadioTimeSource::reference`]). The second defaults to [`ClockReference::unknown`] and must
    /// be stated deliberately, with the evidence in a comment beside the call — a hardware latch on
    /// an unestablished oscillator earns nothing, by design.
    fn time_sources(&self) -> Vec<RadioTimeSource> {
        Vec::new()
    }

    /// What this radio can do to its own clock RATE, if anything (`None` = no steering).
    ///
    /// Offset (phase) discipline is a software correction applied to readings. This is the other
    /// axis: physically changing how fast the counter runs, so a corrected offset stays corrected
    /// instead of re-accumulating. A radio with a crystal trim reports its measured range and
    /// resolution here.
    fn clock_steering(&self) -> Option<ClockSteering> {
        None
    }

    /// Steer this radio's clock rate by `ppm` **relative to its power-on calibration**, returning
    /// the ppm actually applied — which will differ from the request, because the trim is a
    /// quantised, usually non-linear control. Believe the returned value.
    ///
    /// Relative, not absolute: a clock's rate has no meaning except against another clock, so the
    /// caller owns the reference (e.g. a common-view comparison against a peer) and this only
    /// applies the correction it asks for.
    fn steer_clock_ppm(&self, _ppm: f32) -> Result<f32, FaceError> {
        Err(FaceError::Io(std::io::Error::new(
            std::io::ErrorKind::Unsupported,
            "radio exposes no clock-rate steering",
        )))
    }

    /// Read the current value of a `read_now` clock, selected by `domain`, if this radio has
    /// one. Returns `Ok(None)` when the domain is unknown or the radio has only per-frame
    /// stamps (no readable clock). The value is in that domain's raw ticks.
    fn read_clock(&self, _domain: ClockDomainId) -> Result<Option<u64>, FaceError> {
        Ok(None)
    }
}

/// What a radio can do to its own clock rate ([`RadioTime::clock_steering`]).
///
/// Populate from MEASUREMENT, not from a datasheet: both fields feed a discipline loop that will
/// believe them. In particular `range_ppm` should be the span actually swept and verified — a trim
/// register's full range is usually wider than the part of it anyone has characterised.
///
/// ⚠ This describes the **actuator**, never the plant. It says how far and how finely the rate can
/// be MOVED; it says nothing about where the rate is, how far it wanders, or what the counter is
/// derived from. `None` therefore covers both "no trim, excellent crystal" (the NRC7292, MEASURED
/// -35 ppm and unsteerable) and "no trim, RC oscillator" — which is why the reference is a separate
/// declaration on each source ([`ClockReference`]) and not inferred from this.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ClockSteering {
    /// Usable steering range, +-ppm about the power-on calibration.
    pub range_ppm: f32,
    /// Smallest change the trim can make, ppm. The discipline floor is about half this, and if it
    /// is far larger than the sensor's noise the loop is actuator-limited.
    pub resolution_ppm: f32,
}

/// A face's named-time service profile (design §15) — **trait-derived, not a static table**.
///
/// Rather than a per-driver lookup of "what can this radio do for time," the profile is
/// *computed* from the capability traits a backend already implements: its [`RadioTime`] link
/// clocks and its [`RadioTime`]/[`RadioKnobs::tx_discipline`] transmit discipline. A new radio
/// gains a correct time profile the moment it reports its clocks and discipline — nothing here
/// needs editing. The timekeeper reads this to decide what a face may contribute: whether it can
/// source common-view (needs a shared-counter RX stamp **on a reference that holds a rate**), how
/// tightly it stamps arrivals, and how bounded its transmit timing is.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct FaceTimeProfile {
    /// The best (tightest, best-first) link clock the face exposes, or `None` if it stamps
    /// nothing. A per-frame [`RadioClockKind::FreeRunRxStamp`] beats a gated [`RadioClockKind::PortTsf`]
    /// beats a [`RadioClockKind::HostRecv`] software stamp.
    pub best_clock: Option<RadioClockKind>,
    /// The precision (ns) of that best clock's stamps, or `None` if the face stamps nothing. This
    /// is the floor on any offset uncertainty the face can produce (design §9 self-consistency).
    pub stamp_precision_ns: Option<u32>,
    /// The transmit-timing discipline the face can promise (Cut 2).
    pub tx_discipline: TxDiscipline,
    /// The face latches a per-frame free-running RX stamp in hardware — the LATCH half of the
    /// common-view test, on its own.
    ///
    /// Its own field because it is a real and separately useful fact: a consumer that wanted "does
    /// this radio stamp arrivals in silicon" must still be able to ask exactly that. Losing that
    /// question would only move the lie — a part that latches in hardware on a bad reference is a
    /// thing the fleet contains, and both halves of it are true.
    ///
    /// This is precisely what `can_common_view` used to mean, and on its own it MEASURED wrong: two
    /// Waveshare SX1262 nodes both satisfy it and their common view is 10-20 us and growing with
    /// the fit span, against 1 us and flat for two LR2021s. See [`ClockReference`].
    pub hw_rx_stamp: bool,
    /// What the best clock's counter is DERIVED FROM, or `None` if the face stamps nothing —
    /// the second axis, and the one the latch point cannot see. `None` and
    /// `Some(ClockReferenceKind::Unknown)` are different facts: "no clock at all" versus "a clock
    /// whose oscillator nobody has established".
    pub clock_reference: Option<ClockReference>,
    /// Whether the face can contribute common-view observations. **Both halves are required, on the
    /// same source:**
    ///
    /// * a per-frame RX stamp on a free-running counter ([`RadioClockKind::FreeRunRxStamp`]) — a
    ///   gated port TSF or a host stamp does not qualify, because the stamp's own jitter swamps the
    ///   inter-receiver offset; **and**
    /// * a reference that actually holds a rate ([`ClockReference::holds_rate`]) — because two
    ///   receivers' stamps are differenced with the offset and the fitted drift removed, and on a
    ///   wandering reference the fit does not stay fitted, so the residual grows with the span
    ///   instead of settling at the stamp precision.
    ///
    /// An [`ClockReferenceKind::Unknown`] reference does **not** qualify. That is the point: the
    /// capability is earned by evidence, and a radio that has never had its oscillator established
    /// says so rather than defaulting into a claim.
    pub can_common_view: bool,
    /// What the face can do to its own clock RATE, if anything. A face that can both observe
    /// common-view offsets AND steer its rate can hold a correction rather than re-applying it.
    pub steering: Option<ClockSteering>,
}

impl FaceTimeProfile {
    /// Derive the profile from a radio's time surface and transmit discipline (design §15). Pass
    /// the backend's own [`RadioTime`] and the [`TxDiscipline`] it promises. `time_sources()` is
    /// best-first, so the head is the best clock.
    pub fn derive(time: &dyn RadioTime, tx_discipline: TxDiscipline) -> Self {
        let sources = time.time_sources();
        let best = sources.first();
        let best_clock = best.map(|s| s.kind);
        let stamp_precision_ns = best.map(|s| s.precision_ns);
        let clock_reference = best.map(|s| s.reference);
        // The LATCH half, kept separately answerable: a per-frame free-running hardware stamp.
        let hw_rx_stamp = sources
            .iter()
            .any(|s| s.kind == RadioClockKind::FreeRunRxStamp);
        // Common view needs the latch half AND the REFERENCE half, and — this is why it is one
        // predicate over one source rather than two `any()`s — it needs them **on the same clock**.
        // A radio with a hardware-stamped RC counter and a crystal-referenced port TSF satisfies
        // both conditions separately and can still not difference anything with anyone.
        // (design §M3 / measure::common_view; the reference half is the 2026-08-31 two-receiver
        // measurement recorded on `ClockReference`.)
        let can_common_view = sources
            .iter()
            .any(|s| s.kind == RadioClockKind::FreeRunRxStamp && s.reference.holds_rate());
        Self {
            best_clock,
            stamp_precision_ns,
            tx_discipline,
            hw_rx_stamp,
            clock_reference,
            can_common_view,
            steering: time.clock_steering(),
        }
    }
}

/// A radio's static capability profile — band, rates, channels, duty cycle, etc. Implemented
/// per backend so the heterogeneous-radio selection layer reasons about every radio uniformly
/// (which one to pick for a given reach/rate/airtime) instead of hard-coding per-driver
/// knowledge. The companion to [`RadioTime`] (dynamic clocks) on the static-capability axis.
pub trait RadioProfile: Send + Sync {
    /// This radio's capability. Every radio declares one — there is no sensible default.
    fn capability(&self) -> RadioCapability;
}

/// RF band — the coarse range/penetration axis used for heterogeneous radio
/// selection (sub-GHz reaches far / penetrates; 5/6 GHz is bulk; 60 GHz is dense).
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum Band {
    Sub1GHz,
    Band2_4GHz,
    Band5GHz,
    Band6GHz,
    Band60GHz,
}

impl Band {
    /// Relative range/penetration rank (higher = reaches further / penetrates more).
    pub fn range_rank(self) -> u8 {
        match self {
            Band::Sub1GHz => 4,
            Band::Band2_4GHz => 3,
            Band::Band5GHz => 2,
            Band::Band6GHz => 1,
            Band::Band60GHz => 0,
        }
    }
}

/// What kind of radio this is — selects the regime and whether it can transmit.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum RadioKind {
    /// Commodity Wi-Fi in monitor/injection mode (the load-bearing data radio).
    WifiMonitor,
    /// Sub-GHz long-range / low-rate (LoRa-class) — heterogeneous coordination/ambient.
    Lora,
    /// 802.11ah HaLow sub-GHz.
    WifiHaLow,
    /// Bluetooth LE broadcast face.
    Ble,
    /// Software-defined radio used **RX-only as a spectrum instrument** (the richest
    /// `SenseSource`: real PSD/occupancy, interference ID, DFS radar detection,
    /// a calibrated witness for our own TX). Not a data transmitter here — the
    /// SDR-as-modem arc stays the frontier.
    Sdr,
    Other,
}

/// Whether a radio exports channel-state information to the host, and at what granularity — the
/// axis the named-time / sensing plane needs to know per port. Assessed on real hardware:
/// commodity Realtek Wi-Fi is [`None`](Self::None) — its only on-chip CSI is compressed 802.11
/// beamforming feedback (angles for TxBF on >=2-antenna parts, N/A on the 1x1 8733b), never a
/// host-visible H-matrix. Full per-subcarrier CSI needs a CSI-tool NIC (Atheros/Intel) or an SDR.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Default)]
pub enum CsiSupport {
    /// No host-visible channel state beyond the per-frame RSSI/MCS already on `CapturedFrame`.
    #[default]
    None,
    /// Coarse per-path channel quality recoverable from the RX phystatus (RSSI, CFO, EVM) — a
    /// sensing hint, not a full channel estimate.
    Coarse,
    /// Full per-subcarrier CSI (the H-matrix) — an SDR or a CSI-tool NIC.
    PerSubcarrier,
}

// ---------------------------------------------------------------------------
// Modulation as a runtime capability (7E-A5 v3).
// ---------------------------------------------------------------------------

/// **The modulation a radio is running — a knob, not an identity.**
///
/// ★ This exists to undo a design error. The LR2021 runs FLRC because its bring-up calls
/// `set_packet_type(PacketType::Flrc)` **once**, and that one-time choice was encoded as *what the
/// node is* ("radio kind 2 = LR2021-FLRC", with "LR2021-LoRa" as a separate kind). It is not an
/// identity: `SetPacketType` is a runtime command, and the same silicon is a LoRa modem, a BLE
/// modem, a Z-Wave modem or a Wi-SUN modem depending on one byte. Modulation is therefore something
/// cognition **actuates**, exactly like MCS or spreading factor — and it is fleet-wide, not an
/// LR2021 special case: the SX1262 does LoRa + GFSK, the SX1276 does LoRa + FSK + OOK.
///
/// ## The numbering is the LR20xx `SetPacketType` numbering, deliberately
///
/// A portable enum needs *some* code space, and inventing a fresh one would mean two translations
/// (host↔wire and wire↔chip) where one is needed. This is the LR20xx table (datasheet Table 8-1),
/// which is also what the vendored driver's `PacketType` uses
/// (`firmware/lr2021-nrf54l15-rs/vendor/lr2021/src/cmd/cmd_common.rs`), so on the part with the
/// most modes the mapping is the identity. **Other chips map at their own boundary**: an SX1276
/// backend translates `Fsk`/`Ook` into its `RegOpMode` bits, and nothing about this enum claims
/// every part can reach every code — that is what [`PhyModeSet`] is for.
///
/// The vendor crate's names for the codes that differ from the datasheet's: `1 = FskGeneric`,
/// `2 = FskLegacy`, `4 = Ranging`, `13 = Zigbee`.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum PhyMode {
    /// `0x0` — LoRa chirp spread spectrum. The only mode in this table with a spreading factor.
    Lora,
    /// `0x1` — the generic (fully parameterised) FSK packet engine.
    FskGeneric,
    /// `0x2` — the legacy/compatibility FSK packet engine (the datasheet's plain "FSK").
    Fsk,
    /// `0x3` — Bluetooth Low Energy.
    Ble,
    /// `0x4` — round-trip-time-of-flight ranging (the vendor crate's `Ranging`).
    RtToF,
    /// `0x5` — FLRC: a fixed-rate, non-LoRa modulation. No spreading factor, and its coding-rate
    /// code space is its own (`0 = 1/2, 1 = 3/4, 2 = off, 3 = 2/3`), not LoRa's.
    Flrc,
    /// `0x6` — BPSK (transmit-only on the LR20xx: a Sigfox-class uplink modulation).
    Bpsk,
    /// `0x7` — LR-FHSS: long-range frequency-hopping spread spectrum.
    LrFhss,
    /// `0x8` — wireless M-Bus.
    WMBus,
    /// `0x9` — Wi-SUN.
    WiSun,
    /// `0xA` — on-off keying.
    Ook,
    /// `0xB` — raw/unframed PHY access (the vendor crate's `Raw`).
    Raw,
    /// `0xC` — Z-Wave.
    ZWave,
    /// `0xD` — O-QPSK / IEEE 802.15.4 (the vendor crate's `Zigbee`).
    OQpsk154,
    /// A code this host does not know. Carried rather than collapsed, so a node running a mode
    /// added after this build still reports *something true* instead of being read as LoRa.
    Unknown(u8),
}

impl PhyMode {
    /// Decode a `SetPacketType` value.
    pub fn from_code(c: u8) -> Self {
        match c {
            0 => PhyMode::Lora,
            1 => PhyMode::FskGeneric,
            2 => PhyMode::Fsk,
            3 => PhyMode::Ble,
            4 => PhyMode::RtToF,
            5 => PhyMode::Flrc,
            6 => PhyMode::Bpsk,
            7 => PhyMode::LrFhss,
            8 => PhyMode::WMBus,
            9 => PhyMode::WiSun,
            10 => PhyMode::Ook,
            11 => PhyMode::Raw,
            12 => PhyMode::ZWave,
            13 => PhyMode::OQpsk154,
            other => PhyMode::Unknown(other),
        }
    }

    /// The `SetPacketType` value.
    pub fn code(self) -> u8 {
        match self {
            PhyMode::Lora => 0,
            PhyMode::FskGeneric => 1,
            PhyMode::Fsk => 2,
            PhyMode::Ble => 3,
            PhyMode::RtToF => 4,
            PhyMode::Flrc => 5,
            PhyMode::Bpsk => 6,
            PhyMode::LrFhss => 7,
            PhyMode::WMBus => 8,
            PhyMode::WiSun => 9,
            PhyMode::Ook => 10,
            PhyMode::Raw => 11,
            PhyMode::ZWave => 12,
            PhyMode::OQpsk154 => 13,
            PhyMode::Unknown(c) => c,
        }
    }

    /// The [`PhyModeSet`] bit for this mode, or `0` for a code that does not fit a `u32` bitmap.
    /// Every code in the LR20xx table is < 32, so only a wild [`Unknown`](Self::Unknown) can miss.
    pub fn bit(self) -> u32 {
        let c = self.code();
        if c < 32 { 1u32 << c } else { 0 }
    }

    /// **Does "spreading factor" mean anything in this mode?** True only for [`Lora`](Self::Lora).
    ///
    /// The distinction is load-bearing rather than cosmetic: on a fixed-rate mode the fleet's
    /// `[sf, bw, cr]` byte positions are re-keyed to that modulation's own code space, so composing
    /// them from a LoRa-shaped plan silently re-modulates the link. LR-FHSS is deliberately `false`
    /// — it is a hopping GMSK mode with a coding rate and no SF.
    pub fn has_spreading_factor(self) -> bool {
        matches!(self, PhyMode::Lora)
    }

    /// **This host KNOWS this mode has no spreading factor** — true for every named mode except
    /// [`Lora`](Self::Lora), and deliberately **false** for [`Unknown`](Self::Unknown).
    ///
    /// The asymmetry is the point. It is used to *override* a radio that declares an SF span in a
    /// mode that cannot have one (a firmware that forgot to zero its span across a PHY switch would
    /// otherwise let a host push a LoRa `[sf, bw, cr]` triple into a fixed-rate packet engine, whose
    /// byte positions mean something else entirely). Overriding needs certainty, and about a mode
    /// this build has never heard of there is none — there the radio's own declaration is the only
    /// information available, so it stands.
    pub fn known_without_spreading_factor(self) -> bool {
        !matches!(self, PhyMode::Lora | PhyMode::Unknown(_))
    }
}

/// **The set of modulations one radio can be commanded into**, as a bitmap over
/// [`PhyMode::code`] — `bit N set == SetPacketType value N is usable on this radio`.
///
/// A set rather than a list because that is exactly what the wire carries (`EVT_CAP.phy_bitmap`,
/// a `u32`), and because the question a planner asks is membership: *can this radio be a BLE
/// modem?* A radio that has never described its modes reports [`empty`](Self::empty), which is the
/// honest "I cannot say" — never a fabricated single entry.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Hash)]
pub struct PhyModeSet(u32);

impl PhyModeSet {
    /// No modes described.
    pub const fn empty() -> Self {
        Self(0)
    }

    /// From the raw wire bitmap.
    pub const fn from_bits(bits: u32) -> Self {
        Self(bits)
    }

    /// The raw wire bitmap.
    pub const fn bits(self) -> u32 {
        self.0
    }

    /// The one-entry set for a radio that runs exactly one modulation — what a node predating the
    /// capability reports, and the *only* honest synthesis for one.
    pub fn single(mode: PhyMode) -> Self {
        Self(mode.bit())
    }

    /// Add a mode.
    pub fn with(self, mode: PhyMode) -> Self {
        Self(self.0 | mode.bit())
    }

    /// Is `mode` reachable on this radio?
    pub fn contains(self, mode: PhyMode) -> bool {
        let b = mode.bit();
        b != 0 && self.0 & b != 0
    }

    /// How many modes are in the set.
    pub fn len(self) -> u32 {
        self.0.count_ones()
    }

    /// Is the set empty (the radio has not described its modulations)?
    pub fn is_empty(self) -> bool {
        self.0 == 0
    }

    /// **More than one modulation is reachable** — i.e. modulation is genuinely a knob on this
    /// radio and not a fact about it. The predicate a caller uses before planning a PHY switch.
    pub fn is_agile(self) -> bool {
        self.0.count_ones() > 1
    }

    /// The modes in the set, ascending by code.
    pub fn iter(self) -> impl Iterator<Item = PhyMode> {
        (0u8..32)
            .filter(move |b| self.0 & (1u32 << b) != 0)
            .map(PhyMode::from_code)
    }
}

/// What a hop plan's period is counted in — see [`HopCapability`].
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum HopPeriodUnit {
    /// **LoRa symbols.** The SX127x counts FHSS dwell in symbols (`RegHopPeriod`), and so does the
    /// LR20xx LoRa hop counter, so on a LoRa-modulation radio a period is a symbol count — which
    /// means its wall-clock value moves with SF and bandwidth and cannot be cached as a duration.
    LoraSymbols,
    /// Microseconds on the radio's own clock.
    Microseconds,
    /// The radio implements a hop plan and this host has not established what its period counts.
    /// A caller must not convert this into a dwell.
    Unspecified,
}

/// **Can this radio hop by itself, and how far can a plan go?** — deliberately NOT
/// [`RadioCapability::retune_us`].
///
/// The two answer different questions and conflating them is how a planner ends up believing a
/// radio cannot hop when it hops better than any of them:
///
/// * `retune_us` is *"what does a **host-commanded** `SetRfFrequency` cost"* — MEASURED on the
///   7E-A5 fleet at 5.6 ms (Heltec SX1276), 52.8 ms (LR2021) and 161 ms (Waveshare SX1262, a full
///   image calibration). It is a host↔device round trip and it bounds hopping *between* packets.
/// * This is *"does the radio walk a frequency list **autonomously**, inside a packet"* — which the
///   LR20xx and the SX1276 both do, at a dwell no host command could ever reach. There is no value
///   of `retune_us` that expresses it, which is why it is a separate capability rather than a
///   smaller number in the same field.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub struct HopCapability {
    /// The radio hops **within a single packet**, driven by its own sequencer, with no host command
    /// per hop. `false` means a hop plan exists but is applied per packet.
    pub intra_packet: bool,
    /// Longest frequency list the radio will accept. `40` on the 7E-A5 fleet — the bound the wire
    /// contract pins for `CMD_SET_HOP`.
    pub max_list_len: u8,
    /// What [`RadioKnobs::set_hop_plan`]'s `period` is counted in.
    pub period_unit: HopPeriodUnit,
}

/// Whether a hop plan is armed — the `hop_ctrl` byte of a hop-plan command.
///
/// Two states only, and both are actuated: the code space is not a place to park intentions. A
/// *per-packet* hop schedule needs no wire state at all — that is [`RadioKnobs::set_channel`] plus
/// a schedule — so the only thing an autonomous hop plan adds is the in-packet sequencer.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Hash)]
pub enum HopControl {
    /// Disarm hopping; the radio stays on its tuned carrier. The list is still installed.
    #[default]
    Off = 0,
    /// Walk the installed list autonomously, advancing every `period`.
    On = 1,
}

/// **Receive front-end sensitivity, as a posture** — the input to [`RadioKnobs::set_rx_gain`].
///
/// A posture and not a number, for the same reason [`ContentionPosture`] is: the *scales* are
/// per-chip (the SX126x has two LNA register values, the LR20xx a 0..13 manual ladder, the SX127x
/// a 3-bit `RegLna` field) and there is no common unit between them. What IS common — and what all
/// three of our firmwares already implement, as one boolean byte — is the two-position choice
/// below. That is the portable statement.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Hash)]
pub enum RxGain {
    /// Hand the front end back to the part's own default: AGC on the LR20xx, the power-saving LNA
    /// setting on the SX126x. Lower sensitivity, lower current, and no chance of desensing on a
    /// strong nearby transmitter.
    #[default]
    Auto,
    /// Pin the front end at the highest gain the part offers. Buys sensitivity on a marginal link
    /// and costs current; on a loud channel it can make things worse, not better.
    ///
    /// ⚠ **This is not a dB knob.** The delta is a per-part fact and only one of ours has a
    /// figure at all (the SX126x's boosted LNA is a datasheet ~+3 dB); the SX127x and the LR20xx
    /// deltas are unmeasured here. Do not convert this into link budget.
    Boosted,
    /// ★ **Deliberately desensitised, for spatial reuse** — raise the detection floor so this node
    /// stops deferring to transmitters it has no interest in.
    ///
    /// This variant exists because the set above could only ever say "hear the same" or "hear
    /// more": every position was at or above the part's default, so **no node in this fleet could
    /// be told to hear less.** That made spatial reuse structurally impossible, because it has two
    /// halves and we only had one. Backing TX power off shrinks who *hears you*; raising the
    /// detection floor shrinks who *you defer to*. With only the first, a node in a dense cell
    /// still yields to every distant transmitter it can hear, and the concurrency the back-off was
    /// supposed to buy never materialises.
    ///
    /// Bounded and reversible: the driver owns the floor and clamps (the Realtek IGI ceiling is the
    /// vendor's `IGI_MAX = 0x3e`), and [`RxGain::Auto`] must always restore.
    ///
    /// ⚠ Not a dB knob either — the per-part dB delta must be measured before any policy converts
    /// it into link budget. Where a radio has a *genuinely* dBm-denominated defer threshold, use
    /// [`RadioKnobs::set_edcca_threshold_dbm`] instead; the two are complementary, not
    /// alternatives, and the a81a has both.
    Reduced,
}

/// Per-radio capability descriptor — the single switch between homogeneous
/// (NDNPIPES: identical capabilities → channel assignment + spatial reuse) and
/// heterogeneous (NDN-CRAHNs: divergent capabilities → object→radio mapping by
/// fit) regimes. Generalizes the `LinkProfile` cost prior.
///
/// A radio's peak-rate ceiling, keyed by bearer — the static-capability peer of the actuator-side
/// `RateParams`. A consumer reads the rate/reach tradeoff through [`RadioCapability::rate_rank`]
/// (bearer-agnostic) and the Wi-Fi ceilings through the typed accessors; no bearer's rate model is
/// baked into the capability's fields (LoRa has no `max_mcs`, Wi-Fi no spreading factor).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum RateCapability {
    /// No transmit-rate ceiling — an RX-only sensor, or a single-fixed-rate bearer.
    None,
    /// Wi-Fi 802.11: max MCS index, spatial streams, and channel-bandwidth code (0=20…4=5).
    Wifi {
        max_mcs: u8,
        max_nss: u8,
        max_bw: u8,
    },
    /// LoRa sub-GHz: the spreading-factor span (the reach↔rate range; lower SF = faster).
    Lora { min_sf: u8, max_sf: u8 },
}

/// Carries the bearer-specific rate ceiling ([`RateCapability`], read via [`rate_rank`] and the
/// typed accessors) **and** the bearer-agnostic operational axes a cognitive plane needs to place
/// work on a heterogeneous radio: its timing model, duty-cycle ceiling, on-air payload cap, and
/// duplex. Those are what let LoRa, an SDR, or a future PHY be *described* rather than special-cased.
///
/// [`rate_rank`]: RadioCapability::rate_rank
// `PartialEq` so a caller's asserted capability can be checked against the radio's declared one
// (`RadioBearer::effective_cap`) — an assertion that is never compared to the hardware is how
// `agile` became decorative.
#[derive(Clone, Debug, PartialEq)]
pub struct RadioCapability {
    pub kind: RadioKind,
    /// The RF band(s) this radio can operate on — several parts are dual-band (2.4 + 5 GHz),
    /// so this is a set, not one band. Best-first for range is [`range_rank`](Self::range_rank).
    pub bands: Vec<Band>,
    /// Bearer-specific peak-rate ceiling. Read the rate/reach tradeoff via
    /// [`rate_rank`](Self::rate_rank); the Wi-Fi ceilings via [`max_mcs`](Self::max_mcs) /
    /// [`max_nss`](Self::max_nss) / [`max_bw`](Self::max_bw); the LoRa span via
    /// [`sf_range`](Self::sf_range).
    pub rate: RateCapability,
    /// This Wi-Fi radio can transmit **802.11ax (HE)**, unlocking the HE reach levers (ER-SU + DCM) in
    /// [`McsDescriptor::for_intent`] and the `he`/`dcm`/`er_su` descriptor flags. `false` on HT/VHT-only
    /// parts and every non-Wi-Fi bearer. Set via [`with_he`](Self::with_he); read via [`he_cap`](Self::he_cap).
    pub he_cap: bool,
    /// Channels this radio may use.
    pub channels: Vec<u8>,
    /// Max TX-power index (chip TXAGC scale) = the *calibrated/regulatory ceiling*.
    /// The power knob backs off below this; it is never exceeded. This is also a
    /// capability item peers can learn (reach class).
    ///
    /// Opaque and nonlinear — see [`tx_power_dbm`](Self::tx_power_dbm) for the
    /// portable alternative, which is preferred whenever the radio offers it.
    pub max_tx_power: u8,
    /// **The lowest index that is still monotone.** `None` = the scale is usable to 0.
    ///
    /// ★ Not a formality. MEASURED on the a81a: below index 20 the part *inverts* — commanded
    /// power rises again, peaking ~11 dB ABOVE the calibrated maximum. The ESP32-C5 showed the
    /// same class of fault. A back-off that walks past this floor does the opposite of what the
    /// policy decided, loudly, into the channel it was trying to protect.
    pub min_tx_power: Option<u8>,
    /// **dB per index step on this part's scale.** `None` = unmeasured, or measured non-linear.
    ///
    /// ★ This replaces a single global constant (`DB_PER_POWER_IDX = 0.5`) that **no radio in the
    /// fleet obeyed**: MEASURED 0.22 dB/step on the a81a and 0.111–0.155 on the RTL8733BU, so a
    /// decided 18 dB back-off was rendered as 1.5–5 dB of actual back-off, silently, and
    /// differently per part. A policy that reasons in dB must convert with the part's own number
    /// or not convert at all — hence `Option`, and hence [`RadioPolicy`] declining to guess when
    /// it is `None` rather than substituting a plausible one.
    pub db_per_power_idx: Option<f32>,
    /// **Does the power knob reach silicon?** `false` = the radio reports a power range it cannot
    /// actually act on.
    ///
    /// ★ MEASURED true of the MT7612U and MT7921AU, which have no power actuator whatsoever. They
    /// carry a `max_tx_power` because the capability struct demands one, and before this field
    /// existed there was no way to say "that number is decorative". Consumers that reward a
    /// *decision* — notably the contextual bandit's interference-footprint term — must gate on
    /// this, or they learn from an outcome the hardware never produced.
    pub power_actuated: bool,
    /// Absolute TX-power control range in dBm, when the radio exposes one
    /// ([`RadioKnobs::set_tx_power_dbm`]). `None` = index-only control via
    /// [`max_tx_power`](Self::max_tx_power).
    ///
    /// This is the bearer-portable power axis: dBm means the same thing on an
    /// 802.11ah chip, a LoRa modem, and a BLE part, so a planner can budget link
    /// margin in dB instead of guessing at chip register units. Populate it only
    /// when the numbers are real (a driver knob or nl80211 that reports the
    /// applied value) — a fabricated range is worse than `None`, because the
    /// planner will believe it.
    pub tx_power_dbm: Option<DbmRange>,
    /// **Measured** cost of changing channel, microseconds. `None` = never measured.
    ///
    /// This replaces a `agile: bool` ("can retune quickly") that was consumed by nothing and, worse,
    /// was *backwards*: it read `true` on every Wi-Fi monitor radio — the parts whose `set_channel`
    /// is a ~16 ms blocking call — and `false` on LoRa. A planner that had believed it would have
    /// chosen exactly the wrong radio to hop.
    ///
    /// A number rather than a flag because "agile" is not a property of the radio, it is a relation
    /// between the radio and the dwell you intend to use: 16 ms is nothing against a 10 s dwell and
    /// fatal against a 20 ms slot. [`can_hop`](Self::can_hop) is that comparison, and is the only
    /// honest way to answer the question the boolean was pretending to.
    ///
    /// Populate only from a real measurement, per the same rule as
    /// [`tx_power_dbm`](Self::tx_power_dbm): an invented figure is worse than `None`, because
    /// downstream code will believe it.
    pub retune_us: Option<u32>,
    /// RX-only — participates in sensing/reception, never selected for TX (e.g. SDR
    /// sensor). Such radios still contribute to macrodiversity reception pooling.
    pub rx_only: bool,
    // **No `timing: TimingModel` here, deliberately** (#90). It carried AlwaysOn/DutyCycled,
    // had zero readers, and was *false* where it mattered: LoRa was marked `DutyCycled` while our
    // firmware sits in continuous RX. It also restated, badly, a constraint this struct already
    // expresses correctly — `duty_cycle_max` below is the regulatory TX-airtime ceiling, is
    // genuinely consumed by the planner, and is what actually limits LoRa. Two different concepts
    // (RX wake schedule vs TX airtime budget) had been collapsed into one, and the collapsed one
    // was wrong.
    //
    // When a radio that really duty-cycles its receiver exists (#100's wake-up radio, a BLE or
    // ESP32 backend), reintroduce it as something a rendezvous layer *reads* — not as a label.
    /// Regulatory / policy ceiling on the fraction of airtime this radio may use
    /// (`1.0` = unrestricted; LoRa sub-GHz is ~`0.01`). A broadcast rate planner
    /// must respect it.
    pub duty_cycle_max: f32,
    /// Largest on-air payload one frame carries (bytes) — the fragmentation MTU
    /// the link service targets (WiFi ~1500+, ESP-NOW 250, LoRa ~256).
    pub max_payload: usize,
    /// Half-duplex: cannot receive while transmitting (a node never hears its own
    /// TX). True for essentially every single-antenna packet radio.
    pub half_duplex: bool,
    /// Whether this radio exports channel-state information to the host (assessed per port).
    pub csi: CsiSupport,
    /// **The modulations this radio can be commanded into** — see [`PhyMode`]. Empty
    /// ([`PhyModeSet::empty`]) means the radio has never described its modes, which is a different
    /// statement from "one mode": a planner must read it as "I cannot say", never as a set of one.
    ///
    /// Read [`PhyModeSet::is_agile`] before planning a switch, and drive it with
    /// [`RadioKnobs::set_phy`].
    pub phy_modes: PhyModeSet,
    /// **The modulation in effect right now**, or `None` for a radio that does not report one.
    ///
    /// ★ Every other field of this struct is read *in the context of this one*. `max_payload`, the
    /// `rate` model (including whether a spreading factor exists at all), `bands` and the timing
    /// granularity are all **per-PHY**: an LR2021 in FLRC carries 47 bytes and has no SF, while the
    /// same silicon in LoRa carries far more and spans SF7..SF12. So a PHY switch does not patch
    /// fields — it replaces the whole capability.
    pub phy_current: Option<PhyMode>,
    /// **Autonomous frequency hopping**, if the radio has a sequencer of its own — explicitly not
    /// [`retune_us`](Self::retune_us), which prices a *host-commanded* retune. See [`HopCapability`]
    /// for why the two cannot be one field. `None` = the radio does not hop by itself (or has never
    /// said).
    pub hop: Option<HopCapability>,
}

/// The span of absolute TX powers a radio can actually be commanded to, in dBm.
///
/// Bearer-agnostic by construction: it carries no chip, driver, or PHY concept —
/// only the two numbers a link-budget calculation needs. A Wi-Fi part, a LoRa
/// modem, and a HaLow chip all describe themselves the same way here.
///
/// `max` is the *commandable* ceiling, which is not necessarily what the radiates:
/// firmware and regulatory tables clamp further, which is why
/// [`RadioKnobs::set_tx_power_dbm`] returns the applied value rather than `()`.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct DbmRange {
    /// Lowest commandable power (dBm).
    pub min: i8,
    /// Highest commandable power (dBm) — the ceiling cognition backs off *from*.
    pub max: i8,
}

impl DbmRange {
    /// A range, normalised so `min <= max` even if handed to us reversed.
    pub fn new(min: i8, max: i8) -> Self {
        if min <= max {
            Self { min, max }
        } else {
            Self { min: max, max: min }
        }
    }

    /// Clamp a requested power into this range. Callers still treat the value
    /// returned by the actuator as authoritative — this only avoids commanding
    /// something the radio has already told us it cannot do.
    pub fn clamp(&self, dbm: i8) -> i8 {
        dbm.clamp(self.min, self.max)
    }

    /// Total control span in dB — how much link budget this knob can trade.
    pub fn span_db(&self) -> u8 {
        (self.max as i16 - self.min as i16).unsigned_abs() as u8
    }
}

impl RadioCapability {
    /// Declare that this radio has absolute dBm power control over `range`.
    ///
    /// Deliberately a builder rather than a constructor argument: the honest
    /// source of a dBm range is whatever *found* the control at runtime (a probed
    /// driver knob, an nl80211 query), not a table compiled into a capability
    /// preset. The presets therefore leave it `None` and the layer that discovers
    /// the knob attaches the range it actually observed.
    /// Declare the modulations this radio can run and which one is in effect.
    ///
    /// A builder for the same reason [`with_tx_power_dbm`](Self::with_tx_power_dbm) is: the honest
    /// source is whatever *asked the radio* at runtime (the 7E-A5 fleet reads both out of the
    /// node's own `EVT_CAP`), never a table keyed on a part number. A preset that has not asked
    /// leaves the set empty and says "I cannot say".
    pub fn with_phy(mut self, modes: PhyModeSet, current: PhyMode) -> Self {
        self.phy_modes = modes.with(current);
        self.phy_current = Some(current);
        self
    }

    /// Declare an autonomous hop sequencer. See [`HopCapability`] — this is not `retune_us`.
    pub fn with_hop(mut self, hop: HopCapability) -> Self {
        self.hop = Some(hop);
        self
    }

    /// **Does this radio hop on its own inside a packet?** `false` for a radio with no sequencer
    /// *and* for one that has never said — the two are distinguished by [`hop`](Self::hop) being
    /// `None`, and neither may be reported as a capability.
    pub fn hops_intra_packet(&self) -> bool {
        self.hop.is_some_and(|h| h.intra_packet)
    }

    pub fn with_tx_power_dbm(mut self, range: DbmRange) -> Self {
        self.tx_power_dbm = Some(range);
        self
    }

    /// Declare 802.11ax (HE) transmit capability — chainable on a Wi-Fi preset (e.g. the ESP32-C5:
    /// `wifi_monitor_dual_1ss(chs).with_he()`). Unlocks the HE reach levers (ER-SU + DCM). Leave it off
    /// for HT/VHT-only parts so `for_intent(MostRobust)` keeps the universally-decodable HT+STBC+LDPC path.
    pub fn with_he(mut self) -> Self {
        self.he_cap = true;
        self
    }

    /// Whether this radio can transmit 802.11ax (HE) — the gate for the ER-SU / DCM reach levers.
    pub fn he_cap(&self) -> bool {
        self.he_cap
    }

    /// The best (furthest-reaching / most-penetrating) band-rank this radio can use — the max
    /// of [`Band::range_rank`] over its [`bands`](Self::bands). Used by the heterogeneous-radio
    /// selection layer to rank a dual-band radio by its most capable band. 0 if bandless.
    pub fn range_rank(&self) -> u8 {
        self.bands.iter().map(|b| b.range_rank()).max().unwrap_or(0)
    }

    /// Bearer-agnostic peak-throughput rank in `[0, 1]` — the *rate* axis of the reach/rate
    /// heterogeneous-selection tradeoff, comparable across Wi-Fi, LoRa, and future PHYs. Wi-Fi
    /// scales with MCS + spatial streams; LoRa is orders of magnitude slower so it lands near zero
    /// (a lower min SF nudges it up); a rate-less radio (SDR sensor) is zero.
    pub fn rate_rank(&self) -> f32 {
        match self.rate {
            RateCapability::Wifi {
                max_mcs, max_nss, ..
            } => (max_mcs as f32 / 9.0 + (max_nss.saturating_sub(1)) as f32 / 3.0) / 2.0,
            RateCapability::Lora { min_sf, .. } => (12u8.saturating_sub(min_sf)) as f32 / 100.0,
            RateCapability::None => 0.0,
        }
    }

    /// Wi-Fi max MCS index (0 for a non-Wi-Fi radio).
    pub fn max_mcs(&self) -> u8 {
        match self.rate {
            RateCapability::Wifi { max_mcs, .. } => max_mcs,
            _ => 0,
        }
    }

    /// Wi-Fi max spatial streams (1 for a non-Wi-Fi radio).
    pub fn max_nss(&self) -> u8 {
        match self.rate {
            RateCapability::Wifi { max_nss, .. } => max_nss,
            _ => 1,
        }
    }

    /// Clamp the Wi-Fi rate ceilings to declared per-radio maxima (config `max-mcs` / `max-nss`).
    /// Each argument is an optional cap that can only *lower* the existing ceiling; `None` leaves it.
    /// A no-op on non-Wi-Fi radios. Lets an operator pin a radio — e.g. an af-packet TX to
    /// single-stream ≤ MCS 7 — declaratively and statically, independent of (and before) any
    /// neighbour's advertised RX capability. The cognition policy already reads these ceilings, so
    /// the cap takes effect with no further plumbing.
    pub fn with_wifi_caps(mut self, max_mcs: Option<u8>, max_nss: Option<u8>) -> Self {
        if let RateCapability::Wifi {
            max_mcs: m,
            max_nss: n,
            ..
        } = &mut self.rate
        {
            if let Some(cap) = max_mcs {
                *m = (*m).min(cap);
            }
            if let Some(cap) = max_nss {
                *n = (*n).min(cap).max(1);
            }
        }
        self
    }

    /// Wi-Fi max channel-bandwidth code (0 = 20 MHz, for a non-Wi-Fi radio too).
    pub fn max_bw(&self) -> u8 {
        match self.rate {
            RateCapability::Wifi { max_bw, .. } => max_bw,
            _ => 0,
        }
    }

    /// **The rate this radio should use for an observed RSSI** — [`mcs_for_rssi`] clamped to *this*
    /// radio's declared ceiling, not to another chip's calibration (#83).
    ///
    /// The free [`mcs_for_rssi`] is a monotone heuristic over 11n receiver-sensitivity thresholds
    /// and knows nothing about which part is transmitting. Every radio in this workspace declares a
    /// different ceiling (mt7612 9, the 1SS Realteks 8, the RTL8812EU's validated 7), and the
    /// adaptive policy has to respect the one it is actually driving.
    ///
    /// `None` for a radio with no Wi-Fi rate ceiling (LoRa, an RX-only sensor) — asking for an MCS
    /// there is a category error, not a number to guess at.
    pub fn mcs_for_rssi(&self, rssi_dbm: i8) -> Option<u8> {
        match self.rate {
            RateCapability::Wifi { max_mcs, .. } => Some(mcs_for_rssi(rssi_dbm).min(max_mcs)),
            _ => None,
        }
    }

    /// **Can this radio usefully hop on a `dwell_us` dwell?** — the question `agile: bool` was
    /// pretending to answer without reference to a dwell.
    ///
    /// `None` when [`retune_us`](Self::retune_us) has never been measured: an unmeasured radio
    /// yields "I cannot say", never a guess. Callers decide what to do with that — a planner should
    /// treat it as "do not hop" and a bring-up tool as "go measure it".
    ///
    /// The threshold is a quarter of the dwell. Retuning is dead air: at 1/4 the schedule spends a
    /// fifth of its life deaf, which is already a poor trade, and the ~16 ms Wi-Fi figure against a
    /// 20 ms slot is 80% — the incompatibility recorded in #97, now enforced rather than commented.
    pub fn can_hop(&self, dwell_us: u64) -> Option<bool> {
        let retune = u64::from(self.retune_us?);
        Some(retune.saturating_mul(4) <= dwell_us)
    }

    /// Fraction of a `dwell_us` dwell lost to retuning (`0.0`–`1.0`), or `None` if unmeasured.
    /// The honest cost line for a hop plan: multiply through to see what FHSS is charging.
    pub fn retune_overhead(&self, dwell_us: u64) -> Option<f32> {
        let retune = f64::from(self.retune_us?);
        let dwell = dwell_us.max(1) as f64;
        Some((retune / dwell).min(1.0) as f32)
    }

    /// LoRa spreading-factor span `(min, max)`, or `None` for a non-LoRa radio.
    pub fn sf_range(&self) -> Option<(u8, u8)> {
        match self.rate {
            RateCapability::Lora { min_sf, max_sf } => Some((min_sf, max_sf)),
            _ => None,
        }
    }

    /// ⚠ **THESE ARE ONE PART'S NUMBERS. A backend that inherits them is claiming another radio's
    /// measurements as its own** — silently, because the result looks like a plausible value rather
    /// than a wrong one. `retune_us` here was measured on a single radio (#97); `max_tx_power` and
    /// the `rate` ceiling are generic. Since #83 de-globalised the rate ceiling, a radio's declared
    /// `max_mcs`/`max_nss` is AUTHORITATIVE for rate selection, so inheriting them is load-bearing.
    ///
    /// Audited 2026-08-25. Overriding with their own measurements: the RTL8733BU (`retune_us`
    /// 15_500 measured, rate 1x1 HT7) and the RTL8812EU/8822E (rate 1x1 HT7 — it had inherited
    /// 2-stream MCS9, contradicting a MEASURED field failure where one RX chain caused a one-way
    /// link). Explicitly declaring their own rate: mt7612 (9, legitimate 2x2), rtl8821c.
    /// STILL INHERITING and unverified: rtl8812au's 2x2 MCS9 claim, and `retune_us`/`max_tx_power`
    /// on every Wi-Fi backend except the 8733b.
    ///
    /// If you add a backend: measure, or inherit knowingly and say so.
    /// A commodity 5 GHz Wi-Fi monitor radio (our RTL8812EU/8822E data radio).
    pub fn wifi_monitor_5ghz(channels: Vec<u8>) -> Self {
        Self {
            kind: RadioKind::WifiMonitor,
            he_cap: false,
            bands: vec![Band::Band5GHz],
            rate: RateCapability::Wifi {
                max_mcs: 9,
                max_nss: 2,
                max_bw: 2,
            },
            channels,
            max_tx_power: 63,
            // Unmeasured by default: a preset must not invent a power scale. Backends that have
            // MEASURED theirs override these (see `RadioCapability::db_per_power_idx`).
            min_tx_power: None,
            db_per_power_idx: None,
            power_actuated: true,
            tx_power_dbm: None,
            retune_us: Some(16_000), // measured: set_channel is a ~16 ms blocking call (#97)
            rx_only: false,
            duty_cycle_max: 1.0,
            max_payload: 1500,
            half_duplex: true,
            csi: CsiSupport::None,
            phy_modes: PhyModeSet::empty(),
            phy_current: None,
            hop: None,
        }
    }

    /// A 2.4 GHz Wi-Fi monitor radio — our MT7612U (mt76x2u, 2x2 11n on 2.4 GHz).
    /// TX-capable in principle; today only channel 6 / 20 MHz is captured (the
    /// `RadioKnobs` impl errors on other channels), so callers usually pass
    /// `channels = vec![6]` and `max_bw` stays 0 until wider widths are ported.
    pub fn wifi_monitor_2ghz(channels: Vec<u8>) -> Self {
        Self {
            kind: RadioKind::WifiMonitor,
            he_cap: false,
            bands: vec![Band::Band2_4GHz],
            rate: RateCapability::Wifi {
                max_mcs: 7,
                max_nss: 2,
                max_bw: 0,
            },
            channels,
            max_tx_power: 63,
            // Unmeasured by default: a preset must not invent a power scale. Backends that have
            // MEASURED theirs override these (see `RadioCapability::db_per_power_idx`).
            min_tx_power: None,
            db_per_power_idx: None,
            power_actuated: true,
            tx_power_dbm: None,
            retune_us: Some(16_000), // measured: set_channel is a ~16 ms blocking call (#97)
            rx_only: false,
            duty_cycle_max: 1.0,
            max_payload: 1500,
            half_duplex: true,
            csi: CsiSupport::None,
            phy_modes: PhyModeSet::empty(),
            phy_current: None,
            hop: None,
        }
    }

    /// A single-chain (1x1) 5 GHz Wi-Fi monitor radio — the RTL8731BU (halmac_87xx, 1x1 11ac).
    /// One spatial stream, so `max_nss = 1`; at 20 MHz the top reliable VHT-1SS rate is MCS8
    /// (MCS9 needs >=40 MHz). (This part is dual-band; the single `band` field reports its
    /// primary 5 GHz use — a known limitation of the one-band capability model.)
    pub fn wifi_monitor_5ghz_1ss(channels: Vec<u8>) -> Self {
        Self {
            rate: RateCapability::Wifi {
                max_mcs: 8,
                max_nss: 1,
                max_bw: 2,
            },
            ..Self::wifi_monitor_5ghz(channels)
        }
    }

    /// A single-chain (1x1) 2.4 GHz Wi-Fi monitor radio — e.g. the RTL8720DN (BW16) serial
    /// board: 1 stream, 11n MCS0-7, 20 MHz.
    pub fn wifi_monitor_2ghz_1ss(channels: Vec<u8>) -> Self {
        Self {
            rate: RateCapability::Wifi {
                max_mcs: 7,
                max_nss: 1,
                max_bw: 0,
            },
            ..Self::wifi_monitor_2ghz(channels)
        }
    }

    /// A single-chain (1x1) **dual-band** (2.4 + 5 GHz) Wi-Fi monitor radio — e.g. the ESP32-C5 over
    /// the serial bridge. Unlike the RTL parts above, the capability model's `bands` set carries both,
    /// so the planner can actually pick a 5 GHz channel. `channels` spans both bands (e.g.
    /// `[1, 6, 11, 36, 40, 44, 48]`); the backend's `set_channel` switches band by channel number.
    /// One antenna => 1 spatial stream, 11n MCS0-7, 20 MHz base.
    pub fn wifi_monitor_dual_1ss(channels: Vec<u8>) -> Self {
        Self {
            bands: vec![Band::Band2_4GHz, Band::Band5GHz],
            rate: RateCapability::Wifi {
                max_mcs: 7,
                max_nss: 1,
                max_bw: 0,
            },
            ..Self::wifi_monitor_2ghz(channels)
        }
    }

    /// A Wi-Fi HaLow (802.11ah / S1G) monitor radio — our Newracom NRC7292 on
    /// the sub-GHz band. Same 802.11-family framing and monitor-injection model
    /// as the 2.4/5 GHz backends (so it pools uniformly), but on the ~900 MHz S1G
    /// PHY: narrow channels, single stream, and a longer link budget for range.
    /// `channels` are the driver's US alias numbers (e.g. 161 = 925 MHz). The
    /// NRC7292 supports S1G MCS 0–10; rate is set by the on-chip MAC, so the
    /// injection radiotap names no MCS ([`FrameFormat::RawNdnS1g`]).
    pub fn wifi_halow_s1g(channels: Vec<u8>) -> Self {
        Self {
            kind: RadioKind::WifiMonitor,
            he_cap: false,
            bands: vec![Band::Sub1GHz],
            rate: RateCapability::Wifi {
                max_mcs: 10, // S1G MCS0–10 (MCS10 = 1 MHz-only rep-coded BPSK)
                max_nss: 1,
                max_bw: 0, // 1/2/4 MHz S1G widths; we run the base 1 MHz-equiv slot
            },
            channels,
            max_tx_power: 63,
            // Unmeasured by default: a preset must not invent a power scale. Backends that have
            // MEASURED theirs override these (see `RadioCapability::db_per_power_idx`).
            min_tx_power: None,
            db_per_power_idx: None,
            power_actuated: true,
            tx_power_dbm: None,
            retune_us: None, // not measured on the MM6108/NRC7292
            rx_only: false,
            // S1G is a licence-exempt sub-GHz band but, unlike the LoRa ISM path,
            // 802.11ah uses CSMA/CA (listen-before-talk), not a hard duty cycle.
            duty_cycle_max: 1.0,
            max_payload: 1500,
            half_duplex: true,
            csi: CsiSupport::None,
            phy_modes: PhyModeSet::empty(),
            phy_current: None,
            hop: None,
        }
    }

    /// A sub-GHz LoRa-class radio (long range, low rate).
    ///
    /// ⚠ **A preset, and every number in it is one board's guess.** `duty_cycle_max: 0.01` is the
    /// ETSI EU868 1% figure and is wrong anywhere FCC 15.247 applies (US 902–928 has no duty
    /// fraction); `max_payload: 256` is above what any firmware in this rig actually carries
    /// (`MAX_LORA_PAYLOAD` is 240 and an LR2021 FLRC frame is 48); and the 10–22 dBm span is the
    /// SX1262's, not the SX1276's or the LR2021's. It is kept **unchanged** because callers depend on
    /// its exact values, but a backend that knows its own node should build with
    /// [`lora_with`](Self::lora_with) instead of inheriting these.
    ///
    /// ★ **Deprecated as of this run, and the reason is that it has no real callers left.** Every
    /// sub-GHz radio in the rig now describes itself over 7E-A5 `EVT_CAP` and is built through
    /// [`lora_with`](Self::lora_with); what still calls this are `#[cfg(test)]` fixtures and one
    /// simulated radio — none of them a radio, so none of them harmed by the three wrong numbers.
    /// The values are **unchanged** precisely so those tests keep passing; the attribute exists to
    /// stop the next backend from inheriting an ETSI duty cycle in an FCC band.
    #[deprecated(
        note = "a preset of one board's guesses: ETSI 0.01 duty in an FCC band, max_payload 256 \
                above every real firmware, and an SX1262-only 10-22 dBm span. Use \
                RadioCapability::lora_with and fill it from what the radio reports about itself."
    )]
    pub fn lora(channels: Vec<u8>) -> Self {
        Self {
            kind: RadioKind::Lora,
            he_cap: false,
            bands: vec![Band::Sub1GHz],
            // SX126x spreading-factor span 7–12 (the reach↔rate range).
            rate: RateCapability::Lora {
                min_sf: 7,
                max_sf: 12,
            },
            channels,
            max_tx_power: 63,
            // Unmeasured by default: a preset must not invent a power scale. Backends that have
            // MEASURED theirs override these (see `RadioCapability::db_per_power_idx`).
            min_tx_power: None,
            db_per_power_idx: None,
            power_actuated: true,
            // SX126x PA span (the backend clamps to this and sends CMD_SET_PWR): absolute dBm, so the
            // policy backs off from the ceiling for spatial reuse just like on the Wi-Fi path.
            tx_power_dbm: Some(DbmRange::new(10, 22)),
            retune_us: None, // SetRfFrequency is fast, but we have not measured it
            rx_only: false,
            // Sub-GHz is duty-cycle-limited (~1%) and needs a windowed rendezvous;
            // tiny frames, half-duplex.
            duty_cycle_max: 0.01,
            max_payload: 256,
            half_duplex: true,
            csi: CsiSupport::None,
            phy_modes: PhyModeSet::empty(),
            phy_current: None,
            hop: None,
        }
    }

    /// **A sub-GHz packet radio described by the radio itself**, rather than by a preset.
    ///
    /// The parameterised sibling of [`lora`](Self::lora), for a backend that learns its node's real
    /// capability at runtime (the 7E-A5 serial fleet learns all of this from `EVT_CAP`). Every axis a
    /// preset had to guess is an argument here:
    ///
    /// * `rate` — [`RateCapability::Lora`] with the node's true SF span, or [`RateCapability::None`]
    ///   for a genuinely fixed-rate modulation (FLRC), which is a different statement from "SF 7".
    /// * `max_payload` — the REAL end-to-end cap, which on a fixed-frame PHY is far below 256.
    /// * `duty_cycle_max` — a regulatory fact about the *band*, not about the family: 0.01 for ETSI
    ///   EU868, 1.0 under FCC 15.247 digital modulation.
    ///
    /// `tx_power_dbm` is deliberately absent: attach it with
    /// [`with_tx_power_dbm`](Self::with_tx_power_dbm) *only* when the radio reported a real range, so
    /// a node that has never declared one advertises `None` instead of inheriting another part's span.
    ///
    /// `retune_us` is left `None` **by this constructor**, so a backend that has not timed its radio
    /// makes [`can_hop`](Self::can_hop) answer "I cannot say" rather than guess. A backend that HAS
    /// timed it assigns the field afterwards, per node — the 7E-A5 serial fleet does, and the three
    /// nodes differ by 29× (5.6 ms on the Heltec SX1276, 52.8 ms on the LR2021, 161 ms on the
    /// Waveshare SX1262, whose retune runs a full image calibration), which is exactly why this
    /// constructor must not carry a family-wide number of its own.
    pub fn lora_with(
        kind: RadioKind,
        bands: Vec<Band>,
        channels: Vec<u8>,
        rate: RateCapability,
        max_payload: usize,
        duty_cycle_max: f32,
    ) -> Self {
        Self {
            kind,
            he_cap: false,
            bands,
            rate,
            channels,
            max_tx_power: 63,
            // Unmeasured by default: a preset must not invent a power scale. Backends that have
            // MEASURED theirs override these (see `RadioCapability::db_per_power_idx`).
            min_tx_power: None,
            db_per_power_idx: None,
            power_actuated: true,
            tx_power_dbm: None, // attach only from a real declared range
            retune_us: None,    // not measured on any module in the sub-GHz fleet
            rx_only: false,
            duty_cycle_max,
            max_payload,
            half_duplex: true,
            csi: CsiSupport::None,
            phy_modes: PhyModeSet::empty(),
            phy_current: None,
            hop: None,
        }
    }

    /// An RX-only SDR spectrum sensor.
    pub fn sdr_sensor(channels: Vec<u8>) -> Self {
        Self {
            kind: RadioKind::Sdr,
            he_cap: false,
            bands: vec![Band::Band5GHz],
            rate: RateCapability::None, // RX-only instrument — no transmit rate
            channels,
            max_tx_power: 0,
            // Unmeasured by default: a preset must not invent a power scale. Backends that have
            // MEASURED theirs override these (see `RadioCapability::db_per_power_idx`).
            min_tx_power: None,
            db_per_power_idx: None,
            power_actuated: true,
            tx_power_dbm: None,
            retune_us: None, // not measured
            rx_only: true,
            // A spectrum instrument: always listening, never transmits.
            duty_cycle_max: 1.0,
            max_payload: 0,
            half_duplex: false,
            csi: CsiSupport::PerSubcarrier,
            phy_modes: PhyModeSet::empty(),
            phy_current: None,
            hop: None,
        }
    }
}

/// **A fully-capable named-data radio handle** (#78).
///
/// Lives here, beside the traits it aggregates, rather than in `ndn-radio-drivers` — a driver crate
/// constructs one and a face crate consumes one, and neither should have to depend on the other to
/// name the type. (My first attempt put it in the drivers crate, which made `RadioBearer::from_open`
/// require an optional dependency the face only has under a feature flag. Wrong layer.)
///
/// The problem it solves: a standardized opener that returns only `Arc<dyn FrameIo>` drops
/// `RadioKnobs`, `RadioTime` and `RadioProfile`, so any caller wanting control or timing must bypass
/// it and name a concrete backend — reintroducing the very leak the opener exists to close.
///
/// **Four `Option`s rather than a `trait NamedRadio: FrameIo + RadioKnobs + RadioTime +
/// RadioProfile`.** The supertrait reads better and is wrong for this hardware: the capability matrix
/// is genuinely ragged (MT7612U has no `RadioTime`/`RadioProfile`; RTL8821CU has only `FrameIo`), so
/// a supertrait forces stubs that return plausible nonsense. A `None` meaning "this radio genuinely
/// cannot" is worth more than an `Ok(())` that lies — this codebase has a name for the latter, and a
/// tracker full of it.
pub struct OpenRadio {
    /// Bearer-agnostic data plane. Always present — it is what "a radio" means here.
    pub io: std::sync::Arc<dyn FrameIo>,
    /// Channel / TX power / contention control.
    pub knobs: Option<std::sync::Arc<dyn RadioKnobs>>,
    /// Hardware timestamping and the TSF common-view clock.
    pub time: Option<std::sync::Arc<dyn RadioTime>>,
    /// Declared capability + calibration, for the cognition layer.
    pub profile: Option<std::sync::Arc<dyn RadioProfile>>,
    /// ★ **NOT an `Option`.** A handle with no account of how it was brought up is exactly what the
    /// bring-up contract removes: on 2026-09-03 the node binary and every bench example held
    /// indistinguishable handles to transmitters ~20 dB apart. Read it with
    /// [`report`](Self::report); build a hardware-free one with [`BringUpReport::synthetic`].
    ///
    /// ⚠ The other four fields stay `pub` in this pass. The contract's §1.7 makes them private
    /// behind accessors as part of the `open_radio` factory (M8), which is out of scope here —
    /// named rather than quietly skipped. What *is* enforced already: an `OpenRadio` cannot be
    /// constructed without a report.
    pub report: bringup::BringUpReport,
}

impl OpenRadio {
    /// The data plane alone, for callers that genuinely only send and receive.
    ///
    /// **Not a migration shim.** A caller reaching for this because it is convenient is starting the
    /// capability leak over again; reach for it only when the narrowing is the actual intent.
    pub fn io(&self) -> std::sync::Arc<dyn FrameIo> {
        std::sync::Arc::clone(&self.io)
    }

    /// **How this radio was brought up, and into which power regime.** The answer to "which
    /// bring-up did you use?" — the question that had no answer on 2026-09-03.
    pub fn report(&self) -> &bringup::BringUpReport {
        &self.report
    }

    /// A loopback / simulation handle: real `io`, an honest synthetic report, no hardware.
    pub fn synthetic(io: std::sync::Arc<dyn FrameIo>, part: &'static str) -> Self {
        Self {
            io,
            knobs: None,
            time: None,
            profile: None,
            report: bringup::BringUpReport::synthetic(part),
        }
    }
}

/// **A radio's declared rate ceiling must be the one that governs it** (#83).
///
/// `McsDescriptor::for_intent` clamped every radio to `MAX_RELIABLE_MCS`, a figure validated on the
/// RTL8812EU — so each part's own `RateCapability::Wifi { max_mcs }` was declared and then silently
/// overridden by a different chip's calibration. This is the same shape as the `agile` defect: a
/// per-chip fact asserted globally.
///
/// The clamp was load-bearing in one respect, which is why removing it needed care rather than
/// deletion: single-stream HT has no MCS above 7, so a radio declaring 9 would otherwise have been
/// handed a rate that does not exist. That limit is the *standard's*, not a chip's, and stays.
#[cfg(test)]
mod ceiling_tests {
    use super::*;

    #[test]
    fn a_radios_own_ceiling_governs_its_rate_not_another_chips_calibration() {
        let throughput = TxIntent {
            reliability: Reliability::Throughput,
            reach: Reach::Broadcast,
        };

        // The mt7612 declares MCS9 and is VHT-capable. It was being handed 7 — two rates below what
        // it advertises — because of a constant calibrated on a Realtek part.
        let mt = RadioCapability::wifi_monitor_5ghz(vec![36]);
        assert_eq!(mt.max_mcs(), 9, "fixture: this constructor declares 9");
        let d = McsDescriptor::for_intent(&throughput, mt.max_mcs(), true, false);
        assert_eq!(
            d.index, 8,
            "VHT 1SS tops at MCS8 (9 needs >=40 MHz), NOT at the 8812EU's 7"
        );
        assert!(d.vht);

        // Without VHT the structural HT limit still binds — this is the part of the old clamp that
        // was doing real work, and it is a property of 802.11, not of any chip.
        let d = McsDescriptor::for_intent(&throughput, 9, false, false);
        assert_eq!(d.index, 7, "single-stream HT has no rate above MCS7");
        assert!(!d.vht);

        // A part that genuinely validates lower keeps its lower ceiling: the capability is
        // authoritative in both directions, which is the whole point of de-globalising it.
        let d = McsDescriptor::for_intent(&throughput, 4, true, false);
        assert_eq!(
            d.index, 4,
            "a conservative radio must not be pushed up to the mode ceiling"
        );

        // Robust/Balanced are rate-class decisions, not ceiling decisions, and are unaffected.
        let robust = TxIntent {
            reliability: Reliability::MostRobust,
            reach: Reach::Broadcast,
        };
        assert_eq!(McsDescriptor::for_intent(&robust, 9, true, false).index, 0);
    }

    #[test]
    fn adaptive_rate_is_capped_per_radio() {
        // A strong signal asks for MCS7; a radio that declares 4 must still get 4.
        let mut cap = RadioCapability::wifi_monitor_5ghz(vec![36]);
        cap.rate = RateCapability::Wifi {
            max_mcs: 4,
            max_nss: 1,
            max_bw: 0,
        };
        assert_eq!(
            cap.mcs_for_rssi(-40),
            Some(4),
            "clamped to this radio's ceiling"
        );
        assert_eq!(
            cap.mcs_for_rssi(-90),
            Some(0),
            "and a weak link still drops to the floor"
        );

        // Asking a LoRa radio for an MCS is a category error, not a number to guess at.
        // (The preset is deprecated; this test pins its behaviour, which is why it may still call it.)
        #[allow(deprecated)]
        let lora = RadioCapability::lora(vec![0]);
        assert_eq!(lora.mcs_for_rssi(-40), None);
    }
}

#[cfg(test)]
mod tx_hold_default {
    use super::*;

    struct Bare;
    impl RadioKnobs for Bare {
        fn set_channel(&self, _c: u8, _bw: Bandwidth) -> Result<(), FaceError> {
            Ok(())
        }
    }

    /// A radio with no transmit gate must **not report success for a hold it cannot perform**.
    ///
    /// ★ This test previously asserted `is_ok()`, under a doc comment demanding both "never error"
    /// and "never report success for a hold it cannot perform" — which `Result<(), FaceError>`
    /// cannot express, because `Ok(())` IS reporting success. The two clauses were written when an
    /// `Err` was assumed to abort something. It does not: the scheduler calls this as
    /// `let _ = k.set_tx_hold(..)` (`sched.rs:614, :623`) and falls back to its software wait
    /// regardless, which is exactly the behaviour the old comment wanted to protect.
    ///
    /// Why it matters that this is honest: `set_tx_hold` is un-overridden by 11 of 13 backends, and
    /// it is the slot MAC's queue gate — "anything already queued would otherwise drain into
    /// another owner's slot". On a silent `Ok(())` the scheduler believed the queue was held on
    /// every mt76 and both HaLow parts while it was in fact bleeding into the next owner's slot.
    #[test]
    fn default_tx_hold_refuses_rather_than_claiming_a_hold_it_cannot_perform() {
        let b = Bare;
        for r in [b.set_tx_hold(true), b.set_tx_hold(false)] {
            match r {
                Err(FaceError::Io(io)) => {
                    assert_eq!(io.kind(), std::io::ErrorKind::Unsupported)
                }
                other => panic!("expected an Unsupported refusal, got {other:?}"),
            }
        }
    }
}

#[cfg(test)]
mod phy_capability {
    use super::*;

    /// **The code space is the LR20xx `SetPacketType` table**, and every code round-trips. Pinned
    /// against the vendored driver's `PacketType`
    /// (`firmware/lr2021-nrf54l15-rs/vendor/lr2021/src/cmd/cmd_common.rs`), whose names for the
    /// three codes that differ from the datasheet's are `FskLegacy = 2`, `Ranging = 4`,
    /// `Zigbee = 13`.
    #[test]
    fn phy_mode_codes_are_the_setpackettype_table() {
        let table = [
            (0u8, PhyMode::Lora),
            (1, PhyMode::FskGeneric),
            (2, PhyMode::Fsk),
            (3, PhyMode::Ble),
            (4, PhyMode::RtToF),
            (5, PhyMode::Flrc),
            (6, PhyMode::Bpsk),
            (7, PhyMode::LrFhss),
            (8, PhyMode::WMBus),
            (9, PhyMode::WiSun),
            (10, PhyMode::Ook),
            (11, PhyMode::Raw),
            (12, PhyMode::ZWave),
            (13, PhyMode::OQpsk154),
        ];
        for (code, mode) in table {
            assert_eq!(PhyMode::from_code(code), mode, "code {code}");
            assert_eq!(mode.code(), code, "{mode:?}");
            assert_eq!(mode.bit(), 1u32 << code);
        }
        // An unknown code is CARRIED, not collapsed into LoRa — a node running a mode added after
        // this build still reports something true.
        assert_eq!(PhyMode::from_code(30), PhyMode::Unknown(30));
        assert_eq!(PhyMode::Unknown(30).code(), 30);
        // Only LoRa has a spreading factor. LR-FHSS is hopping GMSK and deliberately does not.
        assert!(PhyMode::Lora.has_spreading_factor());
        for m in [PhyMode::Flrc, PhyMode::LrFhss, PhyMode::Ble, PhyMode::Fsk] {
            assert!(!m.has_spreading_factor(), "{m:?}");
            assert!(m.known_without_spreading_factor(), "{m:?}");
        }
        // The override predicate is asymmetric ON PURPOSE: it is used to contradict a radio that
        // declares an SF span in a mode that cannot have one, and contradicting takes certainty.
        assert!(!PhyMode::Lora.known_without_spreading_factor());
        assert!(
            !PhyMode::Unknown(20).known_without_spreading_factor(),
            "about a mode this build has never seen, the radio's own declaration is all there is"
        );
    }

    #[test]
    fn a_phy_set_distinguishes_unknown_from_single_mode() {
        // Empty is "I cannot say" — NOT a set of one, and it must not answer `contains`.
        let unknown = PhyModeSet::empty();
        assert!(unknown.is_empty() && !unknown.is_agile());
        assert!(!unknown.contains(PhyMode::Lora));

        // One mode: the radio is described and is NOT agile — a switch cannot be planned.
        let fixed = PhyModeSet::single(PhyMode::Flrc);
        assert!(!fixed.is_empty() && !fixed.is_agile());
        assert!(fixed.contains(PhyMode::Flrc) && !fixed.contains(PhyMode::Lora));
        assert_eq!(fixed.len(), 1);

        // Two or more: modulation is genuinely a knob here.
        let agile = fixed.with(PhyMode::Lora).with(PhyMode::Ble);
        assert!(agile.is_agile() && agile.len() == 3);
        assert_eq!(
            agile.iter().collect::<Vec<_>>(),
            vec![PhyMode::Lora, PhyMode::Ble, PhyMode::Flrc],
            "ascending by code"
        );
        // The bitmap IS the wire field.
        assert_eq!(agile.bits(), (1 << 0) | (1 << 3) | (1 << 5));
        assert_eq!(PhyModeSet::from_bits(agile.bits()), agile);
    }

    /// A preset that has never asked the radio must not claim a modulation, and the builder must
    /// make the current mode a member of the set even if the caller forgot it.
    #[test]
    fn capability_reports_no_phy_until_a_radio_describes_one() {
        let bare = RadioCapability::wifi_monitor_5ghz(vec![36]);
        assert!(bare.phy_modes.is_empty());
        assert_eq!(bare.phy_current, None);
        assert_eq!(bare.hop, None);
        assert!(!bare.hops_intra_packet());

        let described = bare
            .clone()
            .with_phy(PhyModeSet::single(PhyMode::Lora), PhyMode::Flrc);
        assert_eq!(described.phy_current, Some(PhyMode::Flrc));
        assert!(
            described.phy_modes.contains(PhyMode::Flrc),
            "the mode in effect is always a member of the set"
        );
        assert!(described.phy_modes.is_agile());
    }

    /// **A hop capability is not a retune cost.** A radio may hop inside a packet while its
    /// host-commanded retune is 161 ms; the two fields answer different questions and neither
    /// substitutes for the other.
    #[test]
    fn hop_capability_is_orthogonal_to_retune_cost() {
        let mut cap = RadioCapability::wifi_monitor_5ghz(vec![36]);
        cap.retune_us = Some(160_866); // a Waveshare-class host-commanded retune: hopeless
        assert_eq!(cap.can_hop(100_000), Some(false));
        assert!(
            !cap.hops_intra_packet(),
            "and it has no sequencer of its own"
        );

        let cap = cap.with_hop(HopCapability {
            intra_packet: true,
            max_list_len: 40,
            period_unit: HopPeriodUnit::LoraSymbols,
        });
        assert!(cap.hops_intra_packet());
        assert_eq!(
            cap.can_hop(100_000),
            Some(false),
            "the host-commanded retune is unchanged by it — one does not imply the other"
        );
        assert_eq!(cap.hop.unwrap().max_list_len, 40);
    }

    struct Bare;
    impl RadioKnobs for Bare {
        fn set_channel(&self, _c: u8, _bw: Bandwidth) -> Result<(), FaceError> {
            Ok(())
        }
    }

    /// The three new knobs default to a REFUSAL, never a silent success: a radio with one
    /// modulation, no hop sequencer and no gain control must say so, because a caller acts on the
    /// answer (re-reading a capability, believing frames are spread across a band).
    #[test]
    fn the_new_knobs_refuse_rather_than_pretend() {
        let b = Bare;
        for e in [
            b.set_phy(PhyMode::Lora).err(),
            b.set_hop_plan(HopControl::On, 4, &[915_000_000]).err(),
            b.set_rx_gain(RxGain::Boosted).err(),
            // ★ Extended 2026-08-31 from 3 knobs to all of them. The old name said "the NEW
            // knobs", which was the confession: the convention was applied to each knob as it was
            // written and never retro-applied, leaving 7 silently succeeding. A knob added later
            // and left on the default is now caught here rather than in a bandit reward.
            b.set_tx_hold(true).err(),
            b.set_tx_csd(true).err(),
            b.set_edcca_ignore(true).err(),
            b.set_spreading_factor(9).err(),
            b.set_coding_rate(5).err(),
            b.set_bandwidth_khz(125).err(),
            b.set_contention(ContentionPosture::Owned).err(),
            b.set_tx_power_dbm(10).err(),
        ] {
            match e {
                Some(FaceError::Io(io)) => {
                    assert_eq!(io.kind(), std::io::ErrorKind::Unsupported)
                }
                other => panic!("expected an Unsupported refusal, got {other:?}"),
            }
        }
    }
}

#[cfg(test)]
mod face_time_profile_tests {
    use super::*;

    /// A radio with exactly one clock, so a test can vary one axis at a time.
    struct OneClock(Vec<RadioTimeSource>);
    impl RadioTime for OneClock {
        fn time_sources(&self) -> Vec<RadioTimeSource> {
            self.0.clone()
        }
    }

    fn hw(reference: ClockReference) -> OneClock {
        OneClock(vec![
            RadioTimeSource::free_run_rx_stamp(ClockDomainId(1), 1_000).with_reference(reference),
        ])
    }

    fn derive(r: &OneClock) -> FaceTimeProfile {
        FaceTimeProfile::derive(r, TxDiscipline::BestEffort)
    }

    /// ★ The matrix the whole change exists for. `can_common_view` is the AND of the latch point
    /// and the reference; three of these four rows used to come out `true`.
    #[test]
    fn common_view_needs_the_latch_point_and_the_reference() {
        // hardware latch + crystal => true. The only row that earns it.
        let p = derive(&hw(ClockReference::crystal()));
        assert!(p.can_common_view);
        assert!(p.hw_rx_stamp);
        assert_eq!(p.best_clock, Some(RadioClockKind::FreeRunRxStamp));

        // hardware latch + RC => false. MEASURED: two Waveshare SX1262s, both stamping 95/95 frames
        // in silicon, 10-20 us of common view GROWING with the fit span.
        let p = derive(&hw(ClockReference::rc_oscillator()));
        assert!(
            !p.can_common_view,
            "an RC reference cannot hold a common view"
        );
        assert!(
            p.hw_rx_stamp,
            "and the latch fact must survive: it still stamps in hardware"
        );

        // hardware latch + unknown => false. An unestablished reference earns nothing; this is the
        // default a bare `free_run_rx_stamp` gives, so forgetting withholds rather than grants.
        let p = derive(&hw(ClockReference::unknown()));
        assert!(!p.can_common_view, "unknown must not silently qualify");
        assert!(p.hw_rx_stamp);
        assert_eq!(
            p.clock_reference.map(|r| r.kind),
            Some(ClockReferenceKind::Unknown)
        );

        // software stamp + crystal => false. The reference half cannot rescue the latch half
        // either; the host clock's own reference is fine and its jitter is the problem.
        let p = derive(&OneClock(vec![
            RadioTimeSource::host_recv(ClockDomainId(0)).with_reference(ClockReference::crystal()),
        ]));
        assert!(!p.can_common_view, "a host-recv stamp is not a common view");
        assert!(!p.hw_rx_stamp);
        assert_eq!(p.best_clock, Some(RadioClockKind::HostRecv));
    }

    /// Both conditions on the SAME clock. A radio whose hardware stamp runs on an RC while a
    /// *different* counter is crystal-referenced satisfies each half separately and can still not
    /// difference anything with anyone — two `any()`s would have passed this.
    #[test]
    fn the_two_halves_must_meet_on_one_clock() {
        let r = OneClock(vec![
            RadioTimeSource::free_run_rx_stamp(ClockDomainId(1), 1_000)
                .with_reference(ClockReference::rc_oscillator()),
            RadioTimeSource::port_tsf(ClockDomainId(2)).with_reference(ClockReference::crystal()),
        ]);
        assert!(!derive(&r).can_common_view);
    }

    /// A radio that stamps nothing says `None`, which is a different fact from "a clock whose
    /// oscillator nobody established".
    #[test]
    fn no_clock_at_all_is_not_an_unknown_reference() {
        let p = FaceTimeProfile::derive(&OneClock(Vec::new()), TxDiscipline::BestEffort);
        assert_eq!(p.clock_reference, None);
        assert_eq!(p.best_clock, None);
        assert!(!p.hw_rx_stamp);
        assert!(!p.can_common_view);
    }

    /// The measured figure rides along, so a report can show the evidence next to the verdict.
    #[test]
    fn the_measurement_survives_the_derivation() {
        let m = RateMeasurement::new(-0.338, 0.0, RateWitness::PeerUnit);
        let p = derive(&hw(ClockReference::crystal().measured(m)));
        assert_eq!(p.clock_reference.and_then(|r| r.measured), Some(m));
        assert!(p.can_common_view);
    }
}

#[cfg(test)]
mod s1g_rate_tests {
    use super::*;

    /// The anchor that makes the whole derivation checkable: S1G is 802.11ac downclocked by 10,
    /// so every 2 MHz S1G rate must be exactly a tenth of the corresponding 11ac 20 MHz rate.
    #[test]
    fn two_mhz_is_exactly_a_tenth_of_11ac_20mhz() {
        // 802.11ac 20 MHz 1SS long GI, MCS0..7 (bits/s) — the same ladder `mcs_phy_rate_bps` has.
        for mcs in 0..=7u8 {
            let ac = mcs_phy_rate_bps(mcs);
            let s1g = s1g_phy_rate_bps(mcs, 2, false).expect("MCS0-7 exist at 2 MHz");
            assert_eq!(s1g, ac / 10, "S1G 2 MHz MCS{mcs} must be 11ac/10");
        }
    }

    /// ★ The reason `RadioCapability::rate.max_mcs` must be 7 and not 10 on these radios: the S1G
    /// ladder is NOT monotone at its top. MCS10 is the most robust mode, at half of MCS0.
    #[test]
    fn mcs10_is_the_slowest_rate_not_the_fastest() {
        let mcs0 = s1g_phy_rate_bps(0, 1, false).unwrap();
        let mcs7 = s1g_phy_rate_bps(7, 1, false).unwrap();
        let mcs10 = s1g_phy_rate_bps(10, 1, false).unwrap();
        assert!(mcs10 < mcs0, "MCS10 ({mcs10}) is below MCS0 ({mcs0})");
        assert_eq!(mcs10 * 2, mcs0, "2x repetition ⇒ exactly half");
        assert!(mcs7 > mcs0);
    }

    /// ★ The audit that lets the divisibility rule stand in for the standard's exclusion list.
    ///
    /// Walks every (width, MCS) pair the table can express and asserts the derived rule rejects
    /// **exactly** MCS9 at 1 and 2 MHz, plus the one named hole (MCS10 off 1 MHz). If a future edit
    /// to `n_sd` or the coding table made the rule reject something else, this fails rather than
    /// silently pruning a real rate.
    #[test]
    fn the_derived_holes_are_exactly_the_standards_holes() {
        let mut missing = Vec::new();
        for &bw in &[1u8, 2, 4, 8, 16] {
            for mcs in 0u8..=10 {
                if s1g_phy_rate_bps(mcs, bw, false).is_none() {
                    missing.push((mcs, bw));
                }
            }
        }
        missing.sort_unstable();
        assert_eq!(
            missing,
            vec![(9, 1), (9, 2), (10, 2), (10, 4), (10, 8), (10, 16)],
            "the rate table's holes drifted"
        );
    }

    /// A combination the standard does not define must be `None`, never a plausible number.
    #[test]
    fn undefined_combinations_are_none() {
        assert_eq!(s1g_phy_rate_bps(9, 1, false), None, "no MCS9 at 1 MHz");
        // ★ The regression this test did not previously cover. S1G MCS9 is 11ac 20 MHz MCS9
        // downclocked, and 11ac excludes it for Nss=1 — at S1G that is 1 MHz *and* 2 MHz. This
        // used to return Some(8_650_000) from a truncated 346.67 bits/symbol.
        assert_eq!(
            s1g_phy_rate_bps(9, 2, false),
            None,
            "no MCS9 at 2 MHz either"
        );
        assert_eq!(s1g_phy_rate_bps(10, 2, false), None, "MCS10 is 1 MHz only");
        assert_eq!(s1g_phy_rate_bps(11, 2, false), None, "no MCS above 10");
        assert_eq!(
            s1g_phy_rate_bps(0, 20, false),
            None,
            "20 MHz is not an S1G width"
        );
        assert_eq!(
            s1g_phy_rate_bps(0, 3, false),
            None,
            "3 MHz is not an S1G width"
        );
    }

    /// Monotone in width, and short GI is faster than long by exactly 40/36.
    #[test]
    fn rate_scales_with_width_and_guard_interval() {
        let widths = [1u8, 2, 4, 8, 16];
        let mut last = 0;
        for w in widths {
            let r = s1g_phy_rate_bps(0, w, false).unwrap();
            assert!(r > last, "wider must be faster: {w} MHz");
            last = r;
        }
        let long = s1g_phy_rate_bps(7, 8, false).unwrap();
        let short = s1g_phy_rate_bps(7, 8, true).unwrap();
        // 40 µs -> 36 µs symbol time.
        assert_eq!(short as u64 * 36, long as u64 * 40);
    }

    /// The headline numbers a reader can look up: 1 MHz MCS0 = 300 kbit/s, 2 MHz MCS0 = 650 kbit/s,
    /// 8 MHz MCS7 = 29.25 Mbit/s (all long GI, 1 spatial stream — the last being 802.11ac's
    /// 80 MHz 292.5 Mbit/s downclocked by 10, the same relationship checked above).
    #[test]
    fn spot_values_match_the_standard_table() {
        assert_eq!(s1g_phy_rate_bps(0, 1, false), Some(300_000));
        assert_eq!(s1g_phy_rate_bps(10, 1, false), Some(150_000));
        assert_eq!(s1g_phy_rate_bps(0, 2, false), Some(650_000));
        assert_eq!(s1g_phy_rate_bps(7, 8, false), Some(29_250_000));
    }
}
