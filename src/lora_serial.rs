//! Serial-bridged **sub-GHz LoRa-family** radio backend — one [`FrameIo`] for a fleet of nodes.
//!
//! This started as *the Waveshare SX1262 driver* and had that one board's facts compiled into it:
//! a 10–22 dBm power span, SF 7–12, `ts_ms` milliseconds, a 3 s transmit timeout, and an
//! unconditional `SET_BEACON`/`SET_FREQ`/`SET_MOD`/`SET_PWR` at open. Three more nodes now speak the
//! same `7E A5 | type | len | payload | xor-crc` wire protocol and *none* of those facts hold on all
//! of them — the LR2021 has no spreading factor, stamps at 16 MHz, and its `CMD_SET_FREQ` was
//! MEASURED to permanently break its transmit path. A driver that assumes is therefore a driver that
//! damages hardware.
//!
//! So the shape is inverted: **the node describes itself and the host obeys**. At open the backend
//! asks `CMD_GET_CAP` and parses [`NodeProfile`] out of `EVT_CAP`; every later decision — which
//! commands to send at all, how to clamp a frequency or a power, what a frame's timestamp *means*,
//! how long a transmission may take, whether TX can be scheduled — reads that profile. A node that
//! does not answer (old firmware) gets a written-down legacy profile, so an un-reflashed dongle opens
//! exactly as it did before.
//!
//! ## The protocol (7E-A5 v3)
//!
//! ```text
//!   0x7E 0xA5 <type> <len> <payload…> <crc>        crc = XOR of type, len and every payload byte
//! ```
//!
//! Strictly request/response, one command in flight — see [`CmdPort`]. Each version only ADDS
//! opcodes; a node that does not implement one answers `EVT_UNSUPPORTED (0x8F) [cmd, reason]`
//! rather than going silent, which is why an unsupported knob fails fast here instead of burning a
//! timeout.
//!
//! ## What v3 adds, and the design error it fixes
//!
//! ★ **Modulation was being treated as identity.** The LR2021 runs FLRC because its bring-up calls
//! `set_packet_type(Flrc)` **once**, and v2 wrote that one-time choice into `EVT_CAP.radio_kind`:
//! kind 2 meant "LR2021-FLRC" and kind 3 "LR2021-LoRa" — two *kinds* for one chip. `SetPacketType`
//! is a runtime command with fourteen modes, so modulation is something cognition **actuates**,
//! exactly like MCS or spreading factor. And it is fleet-wide, not an LR2021 special case: the
//! SX1262 does LoRa + GFSK and the SX1276 LoRa + FSK + OOK.
//!
//! So `radio_kind` names the **part** again (0 SX1262, 1 SX1276, 2 LR2021) and the mode moved to
//! [`NodeProfile::phy_current`], beside the set of modes the node can be commanded into. Three
//! opcodes and one event carry it:
//!
//! * `CMD_SET_PHY 0x1D` — switch modulation. **Replies with a whole new `EVT_CAP`**, because
//!   `max_payload`, the SF span, the rate model, `sched_gran_ns` and the band are all per-PHY; the
//!   host REPLACES its [`NodeProfile`] wholesale and never patches a field.
//! * `CMD_SET_HOP 0x1E` — install an autonomous hop plan, which `retune_us` structurally cannot
//!   express (that prices a *host-commanded* retune).
//! * `CMD_TX_AT_ABS 0x1F` — transmit at an absolute instant on the node's own clock, so the host's
//!   serial latency stops landing inside the placement.
//! * `EVT_PHY_ERR 0x8D` — a PHY the node advertises that the chip refused at runtime.
//!
//! **Back-compat runs both ways and is not optional.** A v2 node's 29-byte `EVT_CAP` still parses
//! (its PHY is recovered from the v2 `radio_kind`, and its mode set is the one entry that firmware
//! can actually reach), a node that answers no `CMD_GET_CAP` at all still falls back exactly as
//! before, and a v2 *host* reading a v3 node still decodes byte 1 correctly.
//!
//! ## What is deliberately NOT here
//!
//! * **No Tier-0 name filter.** The in-frame filter is retired (relevance = parse the name); this bearer does
//!   bearer, on purpose — see that method for the keyspace ruling.
//! * **No fabricated capability numbers.** Where a node's firmware has never told us its power range
//!   or its scheduling granularity, the profile carries `0` and the corresponding knob reports
//!   `Unsupported`. A believable wrong number is worse than a refusal, because the planner acts on it.

use std::os::unix::io::RawFd;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

use async_trait::async_trait;
use ndn_frame_io::{
    CapturedFrame, ClockDomainId, ClockReference, ClockReferenceKind, FrameIo, InjectFrame,
    LatchPoint, LinkStamp, PhyMetrics, RadioCapability, RadioClockKind, RadioProfile, RadioTime,
    RadioTimeSource, RateMeasurement, RateWitness,
};
use ndn_radio_hal::bringup::{
    AppliedPower, BringUp, Ctx, Fact, Plan, PlanId, PlanRun, PowerRequest, PumpPolicy, RadioState,
    Role, Stage, Step, StepClass, StepId, StepOutcome,
};
use ndn_radio_hal::{
    Band, Bandwidth, DbmRange, HopCapability, HopControl, HopPeriodUnit, PhyMode, PhyModeSet,
    RadioKind, RadioKnobs, RateCapability, RxGain, TxDiscipline,
};
use ndn_transport::FaceError;
use tokio::sync::{Mutex as AsyncMutex, mpsc};

/// Baud the USB-serial bridge (and thus the host) runs at. Every node in the fleet uses it.
pub const LORA_BAUD: u32 = 115_200;

/// Host clock domain for `HostRecv` stamps ("LORA") — one per host process, shared by every
/// serial LoRa node on it, because it *is* one counter (this process's monotonic clock).
const HOST_CLOCK_DOMAIN: ClockDomainId = ClockDomainId(0x4C4F_5241);

// ---------------------------------------------------------------------------
// 7E-A5 v2 — the wire contract. Mirrors firmware/{waveshare-lora-rs,heltec-lora-rs,
// lr2021-nrf54l15-rs}. Opcodes keep their meaning forever; v2 only adds.
// ---------------------------------------------------------------------------
const SYNC0: u8 = 0x7E;
const SYNC1: u8 = 0xA5;

// ---- Host -> node ----
const CMD_TX: u8 = 0x01; //            payload = frame bytes
const CMD_SET_FREQ: u8 = 0x02; //      payload = u32 BE Hz
const CMD_SET_MOD: u8 = 0x03; //       payload = [sf, bw_code, cr_code]  (bw_code is PER NODE — see `bw_to_fw`)
const CMD_SET_PWR: u8 = 0x04; //       payload = [i8 dBm]
const CMD_SET_SYNC: u8 = 0x05; //      payload = [sx127x sync byte]
const CMD_GET_INFO: u8 = 0x06; //      payload = []                     -> EVT_INFO
const CMD_SET_BEACON: u8 = 0x07; //    payload = [enabled(0/1)]  (opt [enabled, period_mult])
const CMD_CAD: u8 = 0x08; //           payload = []                     -> EVT_CAD  [busy]
const CMD_GET_RSSI: u8 = 0x09; //      payload = []                     -> EVT_RSSI [rssi i16 BE]
const CMD_SET_CAD_CFG: u8 = 0x0A; //   payload = [sym, det_peak, det_min]
const CMD_SET_LBT_CFG: u8 = 0x0B; //   payload = [cw_ms(2 BE), max_backoff, max_attempts]
const CMD_SET_PREAMBLE: u8 = 0x0C; //  payload = [preamble(2 BE)]
const CMD_SF_SCAN: u8 = 0x0D; //       payload = []                     -> EVT_SF_DETECTED [sf | 0]
const CMD_TX_LBT: u8 = 0x0E; //        payload = frame bytes; firmware runs atomic CAD+backoff+key-up
const CMD_SET_NAME_FILTER: u8 = 0x0F; // payload = [u64 BE hash]*  (empty clears -> pass-all)
const CMD_SET_RELAY: u8 = 0x10; //       payload = [u64 BE hash]*  (relay set; empty clears)
const CMD_DATAPLANE: u8 = 0x11; //       payload = [cs_serve, dedup, hop_on, hop_base_ch, hop_span]
const CMD_SET_SENSE_CFG: u8 = 0x12; //   payload = [rssi_thresh i16 BE, cad_repeat]
const CMD_GET_STATS: u8 = 0x13; //       payload = []                   -> EVT_STATS
const CMD_RESET_STATS: u8 = 0x14; //     payload = []                   (clear all counters)
const CMD_SET_DEBUG: u8 = 0x15; //       payload = [on]                 (toggle EVT_LOG diagnostics)
const CMD_ENTER_BOOTLOADER: u8 = 0x16; // payload = [0xB0,0x07]         (jump to the ROM bootloader)
// --- v2 additions ---
const CMD_READ_CLOCK: u8 = 0x17; //      payload = []                   -> EVT_CLOCK
const CMD_TX_AT: u8 = 0x18; //           payload = [delay_us u32 BE][frame] -> EVT_TXDONE
const CMD_GET_CAP: u8 = 0x1A; //         payload = []                   -> EVT_CAP
const CMD_SENSE: u8 = 0x1B; //           payload = []                   -> EVT_SENSE
/// **Receive front-end gain** — `[0 = automatic AGC / power-saving LNA | 1 = boosted]` -> `EVT_INFO`.
///
/// ★ All three firmwares have implemented this and advertised bit 28 since before this run, and
/// nothing in the host tree could send it: an actuator with no caller. See
/// [`LoraSerialBackend::set_rx_gain_mode`] and [`RadioKnobs::set_rx_gain`] for why one boolean byte
/// is the whole portable surface (the LR2021 firmware documents the inversion that a per-chip gain
/// ladder through this byte would create).
const CMD_SET_RX_GAIN: u8 = 0x1C;
// --- v3 additions: modulation, hopping, and the absolute transmit instant ---
/// **Switch the modulation** — `[packet_type u8]` -> a whole new `EVT_CAP`.
///
/// The reply is the node's complete capability *for the PHY it is now in*, because `max_payload`,
/// the SF span, the rate model, `sched_gran_ns` and the band are all per-PHY. The host REPLACES its
/// [`NodeProfile`] with it and never patches a field. See [`LoraSerialBackend::set_phy_mode`].
const CMD_SET_PHY: u8 = 0x1D;
/// **Install an autonomous hop plan** — `[hop_ctrl u8][hop_period u16 BE][n u8][freq_hz u32 BE]*n`
/// (`n <= `[`HOP_LIST_MAX`]) -> `EVT_INFO`. See [`LoraSerialBackend::set_hop_plan_hz`].
const CMD_SET_HOP: u8 = 0x1E;
/// **Transmit at an absolute instant on the node's own clock** —
/// `[target_ticks u64 BE][frame bytes]` -> `EVT_TXDONE`, ticks in [`NodeProfile::stamp_hz`].
///
/// ★ The whole reason it exists: `CMD_TX_AT`'s delay is counted from when the FIRMWARE processes
/// the arm, so the host→device serial latency lands inside the placement. MEASURED on the LR2021's
/// absolute-boundary slot train — 45/45 fired, mean gap 2 399 818 ticks against 2 400 000 nominal
/// (within 11 µs over 44 slots, so accuracy is excellent) but **jitter sd 553 µs / p2p 1875 µs**
/// against a declared 50 µs `sched_gran_ns`. That number is the serial round trip, not the radio:
/// the same node's `CMD_GET_INFO` round trip has a p2p of **550 µs**. As exercised, host-armed
/// relative scheduling is therefore WORSE than the software path (sd 553 vs 155 µs), because it
/// pays an extra round trip to learn a "now" that has already moved. Naming the instant removes the
/// host's latency from the answer entirely.
const CMD_TX_AT_ABS: u8 = 0x1F;
/// **`CMD_GET_CLOCK_REF` — what is this node's counter DERIVED FROM?** `[]` -> [`EVT_CLOCK_REF`].
///
/// The companion to `CMD_READ_CLOCK`: that one returns the counter, this one says what it is
/// counting. `EVT_CAP.stamp_kind` answers a THIRD question — where the stamp is *latched* — and the
/// two are independent, which is the whole reason this opcode exists. The Waveshare is the proof:
/// it latched in hardware (TIM3 input capture, 95/95 frames) while running on an internal RC
/// MEASURED at ~-3100 ppm against its peer, and its common view was ~16 us and growing with the fit
/// span. Moving the same latch onto the board's crystal took it to ~1.1 us and flat. A host reading
/// only `stamp_kind` cannot tell those two builds apart.
///
/// ★ **Past the end of the u32 `cmd_bitmap`** (like `CMD_GET_HOPTRACE` 0x20), so it is discovered by
/// ASKING, not by a capability bit. A node that does not implement it answers `EVT_UNSUPPORTED` —
/// or, on the Heltec, acks unknown commands with `EVT_INFO` and this simply times out. Every one of
/// those outcomes must be read as **unknown**, never as "fine".
const CMD_GET_CLOCK_REF: u8 = 0x21;

// ---- Node -> host ----
const EVT_RX: u8 = 0x81; //     payload = [rssi i16 BE, snr i16 BE, ts u32 BE, frame bytes]
//                              ★ `ts` is in EVT_CAP.stamp_hz ticks — NOT milliseconds, NOT µs.
const EVT_TXDONE: u8 = 0x82; // payload = [ok, attempts]
const EVT_INFO: u8 = 0x83; //   payload = [status, sync(2), errors(2), freq(4), sf, bw, cr, pwr, lost(2), cad_busy(2), defer(2)]
const EVT_LOG: u8 = 0x84; //    payload = ascii (unsolicited; never satisfies a command wait)
const EVT_CAD: u8 = 0x85; //    payload = [busy(0/1)]
const EVT_RSSI: u8 = 0x86; //   payload = [rssi i16 BE]
const EVT_SF_DETECTED: u8 = 0x87; // payload = [sf | 0]
const EVT_TX_STARTED: u8 = 0x88; //  payload = [airtime_ms u16 BE] — emitted just before key-up
const EVT_STATS: u8 = 0x89; //  payload = [rx(4), filtered(4), deduped(4), served(4), relayed(4), cad_busy(2), defer(2)]
//                              ★ 24 bytes on the Heltec and the LR2021; **32** on the Waveshare, whose
//                              v2 tail adds [chip_rx(2), chip_crc_err(2), chip_hdr_err(2), rx_trunc(2)].
//                              Parse by LENGTH, never by node identity — see `NdnStats`.
// --- v2 additions ---
const EVT_CLOCK: u8 = 0x8A; //  payload = [ticks u64 BE]   units = EVT_CAP.stamp_hz
const EVT_CAP: u8 = 0x8B; //    payload = 34 bytes in v3, 29 from a v2 node; all multi-byte fields
//                              BIG-ENDIAN (see `NodeProfile::parse`, which accepts both)
const EVT_SENSE: u8 = 0x8C; //  payload = [activity u16 BE, rssi i16 BE]
/// **v3.** `[requested_phy u8, chip_status u8]` — a PHY the node *advertises* that the chip refused
/// at runtime, carrying the chip's literal status byte rather than a host-invented reason.
///
/// A different answer from `EVT_UNSUPPORTED`: that one means the node never had the mode; this one
/// means it has it and the silicon said no *this time* (a band/PA combination it cannot serve, a
/// calibration it lacks). Both are definite, so both end a command wait immediately.
const EVT_PHY_ERR: u8 = 0x8D;
const EVT_UNSUPPORTED: u8 = 0x8F; // payload = [cmd, reason] — the node does not implement `cmd`
/// **Reply to [`CMD_GET_CLOCK_REF`]** — `[ref_class u8][accuracy_ppm u16 BE]`.
///
/// `ref_class`: 0 = unknown, 1 = internal RC, 2 = crystal/TCXO. `accuracy_ppm` is
/// [`CLOCK_ACCURACY_UNKNOWN`] when the node has not measured itself against a standard — which is
/// the normal case, and the node refusing to invent a figure rather than a shortfall.
///
/// Registered fleet-wide at 0x91 in all four firmwares (`fleet_event_numbering` in
/// `firmware/lr2021-nrf54l15-rs/src/serial.rs` is the registry that keeps the numbering honest after
/// `EVT_RX_STAMP` was born on top of `EVT_HOPTRACE`'s 0x8E).
const EVT_CLOCK_REF: u8 = 0x91;
/// [`EVT_CLOCK_REF`] `ref_class`: the node does not know what its counter runs on.
const CLOCK_REF_UNKNOWN: u8 = 0;
/// [`EVT_CLOCK_REF`] `ref_class`: an internal RC oscillator.
const CLOCK_REF_RC: u8 = 1;
/// [`EVT_CLOCK_REF`] `ref_class`: a crystal or TCXO.
const CLOCK_REF_XTAL: u8 = 2;
/// [`EVT_CLOCK_REF`] `accuracy_ppm` sentinel: the node has not measured its own accuracy.
const CLOCK_ACCURACY_UNKNOWN: u16 = 0xFFFF;
/// **v3.** `[frame_stamp_kind u8, reason u8]` — **this one frame's** `ts` is not the stamp the
/// node's `EVT_CAP` advertises.
///
/// ★ The reason it must exist at all: `stamp_kind` in `EVT_CAP` is a capability of the NODE, and the
/// Waveshare's per-frame verdict can be worse than it. Its capture is discarded whenever the edge
/// cannot be attributed to the frame being reported — no edge, a timer overcapture, two edges in one
/// window, an ISR latency past half a wrap, or (the common one on a busy channel) a *coalesced*
/// window in which the chip completed more than one packet, so the edge belongs to an earlier frame
/// than the payload. The `ts` field is then a software read, and the layout is unchanged, so nothing
/// about the `EVT_RX` itself distinguishes the two.
///
/// It arrives **immediately before** the `EVT_RX` it qualifies, with nothing else written to the
/// link in between, on a single-producer in-order transport. [`handle_event`] therefore stashes it
/// and applies it to the next `EVT_RX`, which is the only reason the node's degrade reaches a
/// consumer: routing it to the reply channel (where an unmatched type is skipped) and printing it
/// under `if debug` — the previous treatment for any unknown event — meant a degraded frame was
/// consumed as a 1 µs `RadioCapture` and folded into common view, silently, on a normal run.
///
/// A host that does not know the opcode is no worse off than before; it keeps the node-level kind.
const EVT_RX_STAMP: u8 = 0x90;

/// **Longest hop list `CMD_SET_HOP` carries**, from the wire contract (`n <= 40`) — and the same
/// bound the two parts that hop autonomously enforce. The host refuses a longer plan rather than
/// letting the node truncate it silently, because a truncated hop list is a plan whose dwell
/// pattern the two ends no longer agree on.
pub const HOP_LIST_MAX: usize = 40;

/// `EVT_UNSUPPORTED` reason codes — **the fleet-wide space**, written down once here so it stops
/// drifting. The LR2021 firmware once numbered these 1/2/3 = not-implemented/param/hardware, which
/// swapped `NO_HARDWARE` and `BAD_LENGTH` relative to the two LoRa nodes; nothing on the wire
/// distinguished the conventions, so a `CMD_TX_AT` refusal decoded as "short payload".
mod unsup {
    /// The node does not know the opcode at all — an older firmware, or a different radio kind.
    pub const UNKNOWN_OPCODE: u8 = 0x01;
    /// The opcode is understood; this board's hardware cannot do it. No firmware fixes it.
    pub const NO_HARDWARE: u8 = 0x02;
    /// The opcode is understood; the payload did not satisfy its argument requirements.
    pub const BAD_LENGTH: u8 = 0x03;
    /// An argument outside the range the node's `EVT_CAP` advertises.
    pub const OUT_OF_RANGE: u8 = 0x04;

    /// Render a reason byte for a log line. Unknown codes print as the raw number rather than being
    /// forced into a name they may not have.
    pub fn name(r: u8) -> &'static str {
        match r {
            UNKNOWN_OPCODE => "unknown-opcode",
            NO_HARDWARE => "no-hardware",
            BAD_LENGTH => "bad-length",
            OUT_OF_RANGE => "out-of-range",
            _ => "?",
        }
    }
}

/// The 7E-A5 protocol version this host speaks and understands in `EVT_CAP[0]`.
///
/// **v3** adds `CMD_SET_PHY`/`CMD_SET_HOP`/`CMD_TX_AT_ABS`, `EVT_PHY_ERR`, and the `EVT_CAP` tail
/// (`phy_bitmap` + `phy_current`) that makes modulation a capability instead of an identity. It
/// also **redefines `EVT_CAP[1]`**: `radio_kind` now names the PART (0 = SX1262, 1 = SX1276,
/// 2 = LR2021) rather than the part-plus-mode, retiring v2's separate "3 = LR2021-LoRa" code.
///
/// Back-compat runs in both directions and is not optional: a v2 node's 29-byte `EVT_CAP` still
/// parses here (see [`NodeProfile::parse`]), and a v2 host reading a v3 node still gets a sane
/// `radio_kind` out of byte 1 because the part codes 0/1/2 kept their meaning.
pub const PROTO_VER: u8 = 3;

/// The `EVT_CAP` payload length **this host emits and parses as v3** — the v2 29 bytes plus
/// `[29..33] phy_bitmap u32 BE` and `[33] phy_current u8`.
const CAP_LEN_V3: usize = 34;
/// The v2 `EVT_CAP` payload length. Still parsed, forever: a node that has not been reflashed is a
/// node this host must keep opening.
const CAP_LEN_V2: usize = 29;

/// **The payload cap both LoRa nodes actually carry end to end, 247 bytes** — MEASURED, in the sense
/// that both boards report exactly this in their own `EVT_CAP`: the Waveshare's `max_payload = 0x00F7`
/// and the Heltec's `RX_MAX = 247` (`firmware/{waveshare-lora-rs,heltec-lora-rs}`). It is the binding
/// **serial-framing** limit, not a radio one: 7E-A5 carries a single-byte `len`, so 255 is the
/// absolute frame ceiling and `CMD_TX_AT` spends 4 of those bytes on its delay word — 247 + 4 = 251
/// still fits, so the scheduled-TX path costs no payload.
///
/// It was 240 until this run, which is *below* both nodes: 7 bytes a frame thrown away, and — worse —
/// [`LoraSerialBackend::max_payload`] takes the smaller side, so the constant silently pinned the
/// PHY's MTU under the real cap even after a node truthfully declared 247.
///
/// **Still a ceiling, not the cap.** The effective per-frame budget is
/// `min(MAX_LORA_PAYLOAD, NodeProfile::max_payload)` — see [`LoraSerialBackend::max_payload`] —
/// because a node may be far smaller: the LR2021 runs a fixed 48-byte FLRC frame with one byte of
/// in-frame length, so its cap is 47 and *that* is the number that must win.
pub const MAX_LORA_PAYLOAD: usize = LORA_NODE_RX_MAX as usize;

/// The end-to-end payload cap of a **v2 LoRa node** (Waveshare SX1262, Heltec SX1276), from the
/// boards' own `EVT_CAP`. One constant with two roles — the host's frame budget above and the pinned
/// fallback for a node that has not answered `CMD_GET_CAP` — so the two can never disagree, which is
/// the second-source-of-truth this replaces. **Not** a fleet-wide value: the LR2021 declares 47.
const LORA_NODE_RX_MAX: u16 = 247;

// ---------------------------------------------------------------------------
// **MEASURED retune cost, per modem** — the wall-clock a `set_channel` costs *this host*. Feeds
// `NodeProfile::retune_us` -> `RadioCapability::retune_us` -> `can_hop`/`retune_overhead`.
//
// Method, identical on all three: send `CMD_SET_FREQ`, wait for the node's `EVT_INFO`, timed
// host-side; n = 8–10 per node with the carrier **alternating** between two values so no sample is a
// no-op retune; spreads were sub-millisecond, which is what one expects of deterministic firmware
// rather than of noise. These are therefore **host-observed command round trips and include the
// 115 200-baud serial hop** — which is correct, because that is what a hop actually costs the caller:
// the radio is off-channel for the whole interval, not just for the PLL/image-cal part of it.
//
// The three differ by 29×, which is exactly why a family-wide preset could never carry this number.
// ---------------------------------------------------------------------------

/// Heltec LoRa32 V2 (ESP32 + SX1276), `lora-phy` `set_frequency`. The only node in the fleet that can
/// hop on a 100 ms dwell.
const RETUNE_US_SX1276: u32 = 5_597;
/// XIAO nRF54L15 + LR2021 (FLRC): standby -> `set_rf` -> re-arm RX, the sequence whose *absence* used
/// to break this node's transmit path for good.
const RETUNE_US_LR2021_FLRC: u32 = 52_798;
/// Waveshare USB-TO-LoRa (GD32 + SX1262).
///
/// **RE-MEASURED 2026-08-28 after the firmware learned to skip the image calibration:
/// 160 866 µs → 82 810 µs** (n=8; min 82 695, max 83 175; alternating 915 ↔ 903 MHz). `sx1262.rs`
/// now memoizes the calibrated band and re-runs `CalibrateImage` only when the target leaves it, and
/// the ~78 ms that vanished is exactly that calibration — `CMD_SET_FREQ` now costs what every other
/// `SET_*` on this node costs.
///
/// An out-of-band retune cannot be measured because it is correctly REFUSED: the firmware clamps to
/// the band it declares in `EVT_CAP` (902–928), so **no legal request reaches the calibration path**,
/// which makes this one figure the honest cost of every hop this node can be asked to perform.
///
/// ⚠ **This constant went 2× stale within hours of first being measured**, because a firmware change
/// moved it. It is a HOST-observed round trip (it includes the serial hop), so the device cannot
/// report it alone — but the device could report its *internal* retune cost and let the host add its
/// own floor, which would keep the two in sync automatically. Until then, **re-measure it on every
/// firmware change**: `/tmp/retune2.pl <dev> <n> <hexA> <hexB>`.
const RETUNE_US_SX1262: u32 = 82_810;

/// Legacy TX-power span, dBm — what this backend assumed for every node before `EVT_CAP` existed
/// (and what [`RadioCapability::lora`] advertises). Used only for a node that reports no range of
/// its own, and only on the *index* knob, which is a back-off dial where clamping low is safe.
const LEGACY_PWR_MIN_DBM: i8 = 10;
const LEGACY_PWR_MAX_DBM: i8 = 22;

/// **The end-to-end payload cap of a node running PRE-v2 firmware — 64, not 240.**
///
/// A profile's `max_payload` is defined as `min(TX accept, RX buffer, on-air PDU)`, and on the old
/// Waveshare firmware those three disagree badly: it accepts 240 bytes on `CMD_TX` and transmits
/// them, but its receive path is `rxbuf = [0u8; 64]` with the event buffer sized `8 + 64`, so
/// anything longer comes back **silently truncated** — the frame still arrives, so no delivery
/// number shows it. Reporting 240 here would let the face size an MTU that corrupts on receive,
/// which is the exact defect class this profile exists to stop.
///
/// A node whose firmware carries the fix reports its real cap in `EVT_CAP` and never reaches this
/// constant. This is only the honest floor for a dongle nobody has reflashed.
const LEGACY_RX_TRUNCATION_CAP: u16 = 64;

/// The Waveshare/firmware channel convention: channel index -> carrier = `(850 + ch)` MHz
/// (ch 18 = 868 EU, ch 65 = 915 US, ch 78 = 928 = the US band edge). Kept so cognition keeps
/// thinking in channels.
fn channel_to_hz(ch: u8) -> u32 {
    (850 + ch as u32) * 1_000_000
}

/// Inverse of [`channel_to_hz`], saturating below 850 MHz.
fn hz_to_channel(hz: u32) -> u8 {
    ((hz / 1_000_000).saturating_sub(850)).min(255) as u8
}

/// The three LoRa channel bandwidths this protocol can name, in kHz, indexed by the **host** code
/// carried in [`LoraParams::bw`]. Airtime is computed from these, so they are the single source.
const BW_KHZ: [u32; 3] = [125, 250, 500];

/// Host bandwidth code (0/1/2 = 125/250/500 kHz) -> the byte **this node's firmware** expects in
/// `CMD_SET_MOD[1]`.
///
/// ⚠ **The code space is per-node and the mismatch is silent.** The Waveshare firmware passes the
/// byte straight through to the SX1262's `SetModulationParams`, whose LoRa bandwidth codes are
/// `0x04/0x05/0x06`. The Heltec firmware instead decodes `0/1/2` (`bw_from` in
/// `firmware/heltec-lora-rs/src/main.rs`) and maps **anything else to 125 kHz** — so sending the
/// SX1262 codes there silently pins the radio to 125 kHz forever, with `EVT_INFO` reporting the
/// value the host asked for. Nothing on the wire distinguishes the two conventions, which is why
/// this is keyed on the node's own declared [`LoraRadioKind`] and pinned by a table test.
fn bw_to_fw(kind: LoraRadioKind, bw: u8) -> u8 {
    let bw = bw.min(2);
    match kind {
        // SX127x path (our Heltec firmware): the wire code IS the host code.
        LoraRadioKind::Sx1276 => bw,
        // SX126x path (Waveshare) and the default for anything that has not told us otherwise:
        // the SX1262 LoRa modulation-bandwidth register values.
        _ => 0x04 + bw,
    }
}

/// A `HostRecv` [`LinkStamp`]: nanoseconds since process start (monotonic), latched when the serial
/// line delivered the frame — the coarsest but honest time a serial bridge can offer.
fn host_stamp() -> LinkStamp {
    static START: std::sync::OnceLock<Instant> = std::sync::OnceLock::new();
    let start = START.get_or_init(Instant::now);
    LinkStamp::new(
        start.elapsed().as_nanos() as u64,
        HOST_CLOCK_DOMAIN,
        LatchPoint::HostRecv.precision_floor_ns(),
        LatchPoint::HostRecv,
    )
}

/// A per-device clock domain, derived from the port path.
///
/// Two dongles on one host are two different physical counters, so they MUST NOT share a domain —
/// `ndn-time` would otherwise subtract stamps from unrelated oscillators and call the difference an
/// offset. Same construction as [`crate::bw16_clock_domain`], tagged `"LR"`.
pub fn lora_clock_domain(path: &str) -> ClockDomainId {
    let mut h: u32 = 0x811c_9dc5;
    for b in path.bytes() {
        h ^= b as u32;
        h = h.wrapping_mul(0x0100_0193);
    }
    ClockDomainId((h & 0x00ff_ffff) | 0x4C52_0000) // "LR" tag in the top bytes
}

// ---------------------------------------------------------------------------
// The node profile — the ONE place a node describes itself.
// ---------------------------------------------------------------------------

/// **Which PART is on the far end of the serial link** (`EVT_CAP[1]`) — the silicon, not the mode
/// it happens to be running.
///
/// ★ **This changed in v3, and the change is the point.** v2 encoded the LR2021's bring-up choice
/// as identity: kind 2 meant "LR2021-FLRC" and kind 3 "LR2021-LoRa", two *kinds* for one chip.
/// Modulation is a runtime command (`SetPacketType`) and therefore a knob, so it moved to
/// [`NodeProfile::phy_current`] where cognition can actuate it, and this enum went back to naming
/// the part. A v2 host reading a v3 node still decodes byte 1 correctly, because the part codes
/// 0/1/2 never moved.
///
/// It selects the register conventions the wire bytes use (see [`bw_to_fw`]) and, together with the
/// current PHY, what the `CMD_SET_MOD` triple means.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum LoraRadioKind {
    /// Semtech SX1262 (the Waveshare USB-TO-LoRa dongle). LoRa + GFSK.
    Sx1262,
    /// Semtech SX1276 (the Heltec LoRa32 V2). LoRa + FSK + OOK.
    Sx1276,
    /// Semtech LR2021 (the XIAO nRF54L15 bridge) — the 14-mode part: LoRa, FSK, BLE, FLRC, BPSK,
    /// LR-FHSS, WM-Bus, Wi-SUN, OOK, Z-Wave, O-QPSK/802.15.4 and ranging, all reachable at runtime.
    Lr2021,
    /// A code this host does not know. Treated conservatively (SX126x wire conventions).
    Unknown(u8),
}

impl LoraRadioKind {
    /// **The v2 code for "LR2021 running FLRC"** — kept so pinned fallbacks and old host code
    /// compile and still pattern-match. It is the same *part* as [`Lr2021`](Self::Lr2021); what the
    /// name added is now [`NodeProfile::phy_current`].
    #[deprecated(note = "v2 conflated the part with its bring-up modulation. Match on \
                LoraRadioKind::Lr2021 and read NodeProfile::phy_current (PhyMode::Flrc) instead.")]
    #[allow(non_upper_case_globals)]
    pub const Lr2021Flrc: LoraRadioKind = LoraRadioKind::Lr2021;

    /// **The v2 code for "LR2021 running LoRa"** — same part, different mode. See
    /// [`Lr2021Flrc`](Self::Lr2021Flrc).
    #[deprecated(note = "v2 conflated the part with its bring-up modulation. Match on \
                LoraRadioKind::Lr2021 and read NodeProfile::phy_current (PhyMode::Lora) instead.")]
    #[allow(non_upper_case_globals)]
    pub const Lr2021Lora: LoraRadioKind = LoraRadioKind::Lr2021;

    /// Decode `EVT_CAP[1]`.
    ///
    /// **3 still decodes**, to the same part: it was v2's "LR2021-LoRa", and a node running that
    /// firmware must keep opening. The mode half of it is recovered by
    /// [`phy_from_v2_radio_kind`], which is where the 2-vs-3 distinction now lives.
    pub fn from_code(c: u8) -> Self {
        match c {
            0 => LoraRadioKind::Sx1262,
            1 => LoraRadioKind::Sx1276,
            2 => LoraRadioKind::Lr2021,
            // v2's retired "LR2021-LoRa". The part is the same; see `phy_from_v2_radio_kind`.
            3 => LoraRadioKind::Lr2021,
            other => LoraRadioKind::Unknown(other),
        }
    }

    /// The `EVT_CAP[1]` code. The LR2021 always emits **2** now — the part code — never v2's 3.
    pub fn code(self) -> u8 {
        match self {
            LoraRadioKind::Sx1262 => 0,
            LoraRadioKind::Sx1276 => 1,
            LoraRadioKind::Lr2021 => 2,
            LoraRadioKind::Unknown(c) => c,
        }
    }

    /// **The modulations this part can be commanded into**, as a fallback for a node whose firmware
    /// does not report a `phy_bitmap` — i.e. every v2 node.
    ///
    /// Deliberately the *one mode it is running*, not the datasheet's list. The part can do more
    /// (the SX1276 does FSK and OOK; the LR2021 does fourteen), but a v2 firmware implements no
    /// `CMD_SET_PHY`, so none of those modes is **reachable** on that node — and a capability that
    /// cannot be actuated is exactly the defect this pass exists to remove. A node that can really
    /// switch says so itself, in `EVT_CAP.phy_bitmap`.
    pub fn phy_fallback(self, current: PhyMode) -> PhyModeSet {
        PhyModeSet::single(current)
    }
}

/// **Recover the modulation a v2 node was running from its v2 `radio_kind` byte.**
///
/// The one place v2's part/mode conflation is unwound: `2` was "LR2021-FLRC" and `3` was
/// "LR2021-LoRa", while `0`/`1` were LoRa modems whose firmware only ever ran LoRa. This is a
/// decode of what the byte MEANT, not a guess about what the silicon can do.
fn phy_from_v2_radio_kind(code: u8) -> PhyMode {
    match code {
        0 | 1 => PhyMode::Lora, // SX1262, SX1276 — every v2 firmware in this fleet ran LoRa
        2 => PhyMode::Flrc,     // v2 "LR2021-FLRC"
        3 => PhyMode::Lora,     // v2 "LR2021-LoRa", the kind v3 retires
        // ★ A kind this host has never seen. `Lora` was the old catch-all and it is an
        // **invention**: a v2 node reporting an unknown part is not evidence that it modulates like
        // the ones we know, and a wrong `phy_current` is not inert — it decides
        // `has_spreading_factor` (hence `clamp_sf`, `tx_timeout`'s airtime budget and the hop
        // period's unit) and it seeds `phy_modes`. [`PHY_UNKNOWN_CODE`] is the honest answer, and
        // the LR2021 firmware's own decode of this rule (`serial::phy_of_v2_radio_kind`) returns
        // `None` here for the same reason.
        _ => PhyMode::Unknown(PHY_UNKNOWN_CODE),
    }
}

/// The `phy_current` for a node whose modulation this host cannot determine.
///
/// **`0xFF` is not a `SetPacketType` value and never can be** — the chip's field is four bits wide,
/// so the whole code space is `0x0..=0xF` — which is exactly what makes it usable as "unknown"
/// rather than as a mode. Every consequence is a refusal rather than a guess:
/// `PhyMode::bit()` is 0 above 31, so it contributes nothing to a [`PhyModeSet`] and
/// [`PhyModeSet::contains`] is false for it, which means [`LoraSerialBackend::set_phy_mode`] refuses
/// every switch and [`NodeProfile::phy_agile`] is false; `has_spreading_factor()` is false, so no
/// `[sf, bw, cr]` triple is composed for it; and `hop_capability`'s period unit is `Unspecified`.
const PHY_UNKNOWN_CODE: u8 = 0xFF;

/// What the `ts` field of `EVT_RX` actually is (`EVT_CAP[16]`).
///
/// The whole point of the field: **the units and the trustworthiness are per node**. The Waveshare
/// fills it with a millisecond software counter; the LR2021 fills the same field with a 16 MHz
/// hardware capture. Reading one as the other is a 16 000× error, so nothing here converts without
/// consulting this.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum StampKind {
    /// No per-frame stamp at all; the `ts` field carries nothing.
    NoStamp,
    /// The node has no hardware stamp and says so — the host latches arrival instead.
    HostRecv,
    /// A software counter on the device (an MCU tick / millisecond count). Monotonic-ish, but it
    /// runs in the firmware's scheduler, so it is NOT a common-view-grade counter.
    SoftwareCounter,
    /// A free-running **hardware** capture latched by the radio/timer peripheral. This is the one
    /// that unlocks common view.
    HardwareFreeRun,
    /// A code this host does not know — treated as [`NoStamp`](Self::NoStamp).
    Unknown(u8),
}

impl StampKind {
    /// Decode `EVT_CAP[16]`.
    pub fn from_code(c: u8) -> Self {
        match c {
            0 => StampKind::NoStamp,
            1 => StampKind::HostRecv,
            2 => StampKind::SoftwareCounter,
            3 => StampKind::HardwareFreeRun,
            other => StampKind::Unknown(other),
        }
    }

    /// The `EVT_CAP[16]` code.
    pub fn code(self) -> u8 {
        match self {
            StampKind::NoStamp => 0,
            StampKind::HostRecv => 1,
            StampKind::SoftwareCounter => 2,
            StampKind::HardwareFreeRun => 3,
            StampKind::Unknown(c) => c,
        }
    }
}

/// **What one node on the 7E-A5 link can actually do** — parsed from its `EVT_CAP`, or pinned from a
/// written-down constant for firmware that predates `CMD_GET_CAP`.
///
/// Every field is either something the device told us or something recorded in this file with a
/// citation. Nothing here is inferred from a family name: a "LoRa dongle" is not a power range, and
/// "it's sub-GHz" is not a frequency span.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct NodeProfile {
    /// Protocol version the node speaks (`EVT_CAP[0]`; [`PROTO_VER`] here).
    pub proto_ver: u8,
    /// Which modem is on the far end.
    pub radio_kind: LoraRadioKind,
    /// Lowest carrier the node will accept, Hz.
    pub freq_min_hz: u32,
    /// Highest carrier the node will accept, Hz.
    pub freq_max_hz: u32,
    /// Lowest commandable TX power, **real dBm** (never a chip register unit).
    pub pwr_min_dbm: i8,
    /// Highest commandable TX power, real dBm. `pwr_min == pwr_max == 0` means *unknown*, and
    /// [`dbm_range`](Self::dbm_range) then reports `None` rather than a fabricated span.
    pub pwr_max_dbm: i8,
    /// Ticks per second of the `EVT_RX` `ts` field. `0` = no per-frame stamp.
    pub stamp_hz: u32,
    /// What that stamp *is*.
    pub stamp_kind: StampKind,
    /// The REAL end-to-end payload cap: `min(TX accept, RX buffer, on-air PDU)`.
    pub max_payload: u16,
    /// Bit N set == opcode N implemented. All 7E-A5 opcodes are < 32.
    pub cmd_bitmap: u32,
    /// Spreading-factor span. `0/0` when the node has no spreading factor (FLRC).
    pub sf_min: u8,
    /// Spreading-factor span, upper end.
    pub sf_max: u8,
    /// Hardware-scheduled-TX granularity, ns. `0` = the node cannot place a frame in time.
    pub sched_gran_ns: u32,
    /// **The modulations this node can be commanded into** (`EVT_CAP[29..33]`, v3): bit N set means
    /// `SetPacketType` value N — i.e. [`PhyMode::code`] N — is usable here.
    ///
    /// For a v2 node this is synthesised as the single mode it is running, and that is the honest
    /// answer rather than a shortfall: a v2 firmware implements no `CMD_SET_PHY`, so no other mode
    /// is *reachable* on it however many the silicon supports.
    pub phy_bitmap: u32,
    /// **The modulation in effect right now** (`EVT_CAP[33]`, v3), or — on a v2 node — the mode
    /// recovered from its `radio_kind` byte by [`phy_from_v2_radio_kind`].
    ///
    /// ★ Read every other field of this profile *in the context of this one*. `max_payload`, the
    /// SF span, the rate model, `sched_gran_ns` and the band are per-PHY, which is why a
    /// `CMD_SET_PHY` replies with a whole new `EVT_CAP` and the host replaces this struct wholesale.
    pub phy_current: PhyMode,
    /// `true` when this came from the device's own `EVT_CAP`; `false` when it is a host-side
    /// fallback. Surfaced so a bring-up tool can tell "the node said so" from "we assumed".
    pub learned: bool,
}

impl NodeProfile {
    /// **Parse an `EVT_CAP` payload — 34 bytes (v3) or 29 (v2).** `None` if it is short or the
    /// version is one this host does not speak: a truncated capability is not a capability.
    ///
    /// ## Both directions, and neither is optional
    ///
    /// * A **v3** payload carries the real `phy_bitmap` and `phy_current` in its tail.
    /// * A **v2** payload (29 bytes) still parses, and must: the fleet has three firmwares and they
    ///   are not reflashed together. Its `phy_current` is recovered from the v2 `radio_kind` byte
    ///   ([`phy_from_v2_radio_kind`] — 2 was "LR2021-FLRC", 3 was "LR2021-LoRa", 0/1 were LoRa
    ///   modems) and its `phy_bitmap` is the **one-entry** set for that mode, because a v2 firmware
    ///   has no `CMD_SET_PHY` and therefore no second mode a host could actually reach.
    ///
    /// **The length decides the layout, not the version byte.** A node declaring v3 in 29 bytes has
    /// sent a frame no parser should accept, so it is refused rather than read with a synthesised
    /// tail; a 34-byte payload is read as v3 whatever byte 0 says, because the tail is *there*.
    pub fn parse(p: &[u8]) -> Option<Self> {
        if p.len() < CAP_LEN_V2 {
            return None;
        }
        let u32be = |o: usize| u32::from_be_bytes([p[o], p[o + 1], p[o + 2], p[o + 3]]);
        let u16be = |o: usize| u16::from_be_bytes([p[o], p[o + 1]]);
        let proto_ver = p[0];
        if proto_ver > PROTO_VER {
            // A newer node may have re-laid the tail of this struct; refuse rather than mis-read it.
            return None;
        }
        let has_tail = p.len() >= CAP_LEN_V3;
        if proto_ver >= 3 && !has_tail {
            // v3 without its tail is a malformed frame, not an older node.
            return None;
        }
        let (phy_bitmap, phy_current) = if has_tail {
            (u32be(29), PhyMode::from_code(p[33]))
        } else {
            let m = phy_from_v2_radio_kind(p[1]);
            (PhyModeSet::single(m).bits(), m)
        };
        Some(Self {
            proto_ver,
            radio_kind: LoraRadioKind::from_code(p[1]),
            freq_min_hz: u32be(2),
            freq_max_hz: u32be(6),
            pwr_min_dbm: p[10] as i8,
            pwr_max_dbm: p[11] as i8,
            stamp_hz: u32be(12),
            stamp_kind: StampKind::from_code(p[16]),
            max_payload: u16be(17),
            cmd_bitmap: u32be(19),
            sf_min: p[23],
            sf_max: p[24],
            sched_gran_ns: u32be(25),
            phy_bitmap,
            phy_current,
            learned: true,
        })
    }

    /// Serialise to the **v2** 29-byte `EVT_CAP` payload — the wire form a pre-v3 node emits.
    ///
    /// Kept at 29 bytes and at this signature on purpose: it is what a host-side emulator of a *v2*
    /// node must produce, and the back-compat tests need to build exactly that. The PHY tail is not
    /// representable here, so the version byte is emitted as `min(proto_ver, 2)` — 29 bytes with a
    /// `3` in byte 0 is a frame [`parse`](Self::parse) rightly refuses, and emitting one would be
    /// this host manufacturing the malformed input it guards against. Use
    /// [`to_cap_payload_v3`](Self::to_cap_payload_v3) for a v3 node.
    pub fn to_cap_payload(&self) -> [u8; CAP_LEN_V2] {
        let mut p = [0u8; CAP_LEN_V2];
        p[0] = self.proto_ver.min(2);
        p[1] = self.radio_kind.code();
        p[2..6].copy_from_slice(&self.freq_min_hz.to_be_bytes());
        p[6..10].copy_from_slice(&self.freq_max_hz.to_be_bytes());
        p[10] = self.pwr_min_dbm as u8;
        p[11] = self.pwr_max_dbm as u8;
        p[12..16].copy_from_slice(&self.stamp_hz.to_be_bytes());
        p[16] = self.stamp_kind.code();
        p[17..19].copy_from_slice(&self.max_payload.to_be_bytes());
        p[19..23].copy_from_slice(&self.cmd_bitmap.to_be_bytes());
        p[23] = self.sf_min;
        p[24] = self.sf_max;
        p[25..29].copy_from_slice(&self.sched_gran_ns.to_be_bytes());
        p
    }

    /// Serialise to the **v3** 34-byte `EVT_CAP` payload: the v2 bytes, then
    /// `[29..33] phy_bitmap u32 BE` and `[33] phy_current`.
    pub fn to_cap_payload_v3(&self) -> [u8; CAP_LEN_V3] {
        let mut p = [0u8; CAP_LEN_V3];
        let head = {
            let mut h = self.to_cap_payload();
            h[0] = self.proto_ver.max(3); // this layout IS v3, whatever the source profile said
            h
        };
        p[..CAP_LEN_V2].copy_from_slice(&head);
        p[29..33].copy_from_slice(&self.phy_bitmap.to_be_bytes());
        p[33] = self.phy_current.code();
        p
    }

    /// The modulations this node can be commanded into.
    pub fn phy_modes(&self) -> PhyModeSet {
        // The mode in effect is always a member, even if a firmware forgot to set its own bit.
        PhyModeSet::from_bits(self.phy_bitmap).with(self.phy_current)
    }

    /// **Is modulation actually a knob on this node?** True only when it advertises more than one
    /// mode AND implements the opcode that switches them — the same "a number without its actuator
    /// is not a capability" rule as [`schedules_tx`](Self::schedules_tx) and
    /// [`retune_us`](Self::retune_us).
    pub fn phy_agile(&self) -> bool {
        self.phy_modes().is_agile() && self.supports(CMD_SET_PHY)
    }

    /// **What this node's autonomous hop sequencer can do** — `None` when it has none.
    ///
    /// Gated on the node advertising `CMD_SET_HOP`, because a hop plan the host cannot install is
    /// not a capability. The period unit is decided by the **current PHY**: a LoRa-modulation node
    /// counts its dwell in symbols (the SX127x's `RegHopPeriod`, and the LR20xx LoRa hop counter),
    /// and on any other modulation this host has not established what the byte counts — which is
    /// [`HopPeriodUnit::Unspecified`], a value a caller must not turn into a dwell, rather than a
    /// plausible guess at microseconds.
    pub fn hop_capability(&self) -> Option<HopCapability> {
        self.supports(CMD_SET_HOP).then(|| HopCapability {
            // The two parts in this fleet that hop autonomously both hop WITHIN a packet; that is
            // the whole reason this capability is not `retune_us`.
            intra_packet: true,
            max_list_len: HOP_LIST_MAX as u8,
            period_unit: if self.phy_current.has_spreading_factor() {
                HopPeriodUnit::LoraSymbols
            } else {
                HopPeriodUnit::Unspecified
            },
        })
    }

    /// Does this node implement `cmd`? Every 7E-A5 opcode is < 32, so an opcode outside that is a
    /// host bug, not an unsupported command — report it as unsupported anyway rather than panic.
    pub fn supports(&self, cmd: u8) -> bool {
        cmd < 32 && self.cmd_bitmap & (1u32 << cmd) != 0
    }

    /// The absolute-dBm control range, or `None` when the node has never told us one. `None` is what
    /// makes [`RadioKnobs::set_tx_power_dbm`] refuse rather than clamp into an invented span.
    pub fn dbm_range(&self) -> Option<DbmRange> {
        (self.pwr_min_dbm != 0 || self.pwr_max_dbm != 0)
            .then(|| DbmRange::new(self.pwr_min_dbm, self.pwr_max_dbm))
    }

    /// Clamp a requested dBm into the node's range; falls back to the legacy 10–22 dBm span when the
    /// node reports none, because the index knob still has to send *some* byte and a low clamp is the
    /// safe direction for a back-off dial.
    pub fn clamp_dbm(&self, dbm: i8) -> i8 {
        match self.dbm_range() {
            Some(r) => r.clamp(dbm),
            None => dbm.clamp(LEGACY_PWR_MIN_DBM, LEGACY_PWR_MAX_DBM),
        }
    }

    /// Clamp a carrier into the node's frequency span. A zero/degenerate span means "unknown", and
    /// the request passes through untouched rather than being pinned to 0 Hz.
    pub fn clamp_hz(&self, hz: u32) -> u32 {
        if self.freq_min_hz == 0 || self.freq_max_hz < self.freq_min_hz {
            hz
        } else {
            hz.clamp(self.freq_min_hz, self.freq_max_hz)
        }
    }

    /// Clamp a spreading factor into the node's span; `None` when the node has no SF at all, which
    /// is a different answer from "SF 7" and must not be collapsed into one.
    pub fn clamp_sf(&self, sf: u8) -> Option<u8> {
        // The SAME predicate as `has_spreading_factor`, not a second copy of half of it: these two
        // are the read and the write side of one decision, and the bug they guard is a node whose
        // span and whose modulation disagree.
        self.has_spreading_factor()
            .then(|| sf.clamp(self.sf_min, self.sf_max))
    }

    /// Does the `sf` byte of `CMD_SET_MOD`/`EVT_INFO` actually mean a spreading factor on this node?
    ///
    /// `false` for a fixed-rate modulation (FLRC), where the fleet's byte positions are reused with
    /// chip-specific meanings — see [`LoraSerialBackend::send_mod`]. It gates the whole triple, not
    /// just the `sf` byte, because `cr` is re-keyed on such a node too.
    ///
    /// **Two declarations must agree, and the stricter one wins.** The node states a span *and* a
    /// modulation, and after `CMD_SET_PHY` those are two chances to be inconsistent — a firmware
    /// that switched to FLRC and forgot to zero `sf_min` would let this host push a LoRa triple
    /// into a fixed-rate packet engine, which is exactly the 10× silent re-modulation this gate
    /// exists to prevent. A mode this host does not recognise does not override the span, because
    /// overriding needs certainty; see [`PhyMode::known_without_spreading_factor`].
    pub fn has_spreading_factor(&self) -> bool {
        self.sf_min > 0
            && self.sf_max >= self.sf_min
            && !self.phy_current.known_without_spreading_factor()
    }

    /// Nanoseconds per `ts` tick, or `None` when the node stamps nothing. 16 MHz -> 63 ns (the
    /// integer-ns rounding of a MEASURED 62.5 ns tick).
    pub fn tick_ns(&self) -> Option<u32> {
        (self.stamp_hz > 0)
            .then(|| ((1_000_000_000f64 / self.stamp_hz as f64).round() as u32).max(1))
    }

    /// Can the host read this node's clock on demand (`CMD_READ_CLOCK` -> `EVT_CLOCK`)?
    pub fn has_readable_clock(&self) -> bool {
        self.supports(CMD_READ_CLOCK) && self.stamp_hz > 0
    }

    /// **Does this node place TX in time?** A declared granularity AND at least one opcode that
    /// actuates it — the relative `CMD_TX_AT` or the absolute `CMD_TX_AT_ABS`. A node claiming a
    /// granularity without either is inconsistent, and believing it is exactly the failure
    /// [`FrameIo::schedules_tx`] warns about: the caller would skip its own software gate and the
    /// frame would go out ungated, now.
    pub fn schedules_tx(&self) -> bool {
        self.sched_gran_ns > 0 && (self.supports(CMD_TX_AT) || self.supports(CMD_TX_AT_ABS))
    }

    /// **Does this node accept an ABSOLUTE transmit instant** (`CMD_TX_AT_ABS`)?
    ///
    /// ★ The distinction is not cosmetic, it is the placement error. `CMD_TX_AT`'s delay is counted
    /// from when the *firmware* processes the arm, so the host→device serial latency is inside the
    /// answer: MEASURED sd **553 µs**, p2p **1875 µs**, against a declared 50 µs `sched_gran_ns` —
    /// the same magnitude as that node's 550 µs `CMD_GET_INFO` round-trip p2p, which is the tell.
    /// Naming an instant on the node's own counter removes the host from the measurement.
    pub fn schedules_tx_abs(&self) -> bool {
        self.sched_gran_ns > 0 && self.supports(CMD_TX_AT_ABS)
    }

    /// **Will `inject_after(delay_us)` actually place the frame in time?** `false` means it falls
    /// through to inject-now and the delay is discarded — the caller's software gate is the only
    /// thing holding the slot. Shared with [`FrameIo::inject_after`] so the answer and the behaviour
    /// are the same expression, not two that must be kept in step.
    ///
    /// Either opcode serves: with only `CMD_TX_AT_ABS` the backend converts the delay against the
    /// node's clock, which is a round trip the relative opcode does not need — but it is still
    /// placement, so this stays `true` and [`schedules_tx`](Self::schedules_tx) does not become a
    /// claim the seam cannot honour.
    pub fn schedules_after(&self, delay_us: u64) -> bool {
        delay_us > 0
            && self.schedules_tx()
            && (self.supports(CMD_TX_AT) || self.has_readable_clock())
    }

    /// **Will `inject_at_clock` actually place the frame in time?** The target must be in this
    /// node's own domain — a tick in someone else's domain is not a time on this radio — and the
    /// node must be able to act on an instant: either directly (`CMD_TX_AT_ABS`) or by converting
    /// it into a delay, which needs both the relative opcode and a readable clock.
    pub fn schedules_at_clock(&self, own_domain: bool) -> bool {
        own_domain
            && (self.schedules_tx_abs()
                || (self.schedules_tx() && self.supports(CMD_TX_AT) && self.has_readable_clock()))
    }

    /// The [`TxDiscipline`] this profile implies — [`TxDiscipline::ScheduledAt`] only when
    /// [`schedules_tx`](Self::schedules_tx) holds, so the declared discipline and the implemented
    /// seam are one decision. [`RadioKnobs::tx_discipline`] is this method; nothing re-derives it.
    pub fn tx_discipline(&self) -> TxDiscipline {
        if self.schedules_tx() {
            TxDiscipline::ScheduledAt {
                granularity_ns: self.sched_gran_ns as u64,
            }
        } else {
            TxDiscipline::BestEffort
        }
    }

    /// **What a channel change costs on this node, µs** — the MEASURED figure for its modem
    /// (`RETUNE_US_SX1276`, `RETUNE_US_LR2021_FLRC`, `RETUNE_US_SX1262`), or `None` when it has
    /// never been measured on that part. This is what makes [`RadioCapability::can_hop`] answerable
    /// on this bearer at all; before it, every LoRa node answered "I cannot say".
    ///
    /// **Gated on `CMD_SET_FREQ`.** A node that does not implement it cannot hop *at any dwell*, and
    /// there is no cost value that says so — the honest answer is `None`, which the HAL documents a
    /// planner must treat as "do not hop". Reporting 52 798 µs for the pre-v2 `m6_bridge`, whose
    /// `set_channel` refuses outright, would instead have told a planner it may hop on a long enough
    /// dwell. Same shape as [`schedules_tx`](Self::schedules_tx): a number without its actuator is
    /// not a capability.
    pub fn retune_us(&self) -> Option<u32> {
        if !self.supports(CMD_SET_FREQ) {
            return None;
        }
        match self.radio_kind {
            LoraRadioKind::Sx1276 => Some(RETUNE_US_SX1276),
            LoraRadioKind::Sx1262 => Some(RETUNE_US_SX1262),
            // ★ **Per PART and per PHY.** 52 798 µs was measured on the LR2021 *in FLRC*, and the
            // retune path is not shared across modulations — the standby → `set_rf` → re-arm
            // sequence runs through the packet engine that is loaded. So the measurement is claimed
            // only for the mode it was taken in; the same chip in LoRa has never been timed, and
            // inheriting the FLRC number for it would be one part's measurement worn by another
            // configuration. An unknown code is a part this host has never seen at all.
            LoraRadioKind::Lr2021 if self.phy_current == PhyMode::Flrc => {
                Some(RETUNE_US_LR2021_FLRC)
            }
            LoraRadioKind::Lr2021 | LoraRadioKind::Unknown(_) => None,
        }
    }

    /// **The effective per-frame payload budget for this node** — the smaller of the host's
    /// one-packet-per-frame ceiling ([`MAX_LORA_PAYLOAD`]) and what the node declared it carries end
    /// to end. The smaller side always wins: the host ceiling stops a node's optimistic number from
    /// exceeding what this driver frames, and the node's cap stops the ceiling from over-filling a
    /// 47-byte FLRC frame. [`LoraSerialBackend::max_payload`] is this method.
    pub fn frame_budget(&self) -> usize {
        MAX_LORA_PAYLOAD.min(self.max_payload as usize)
    }

    /// The channels this node can actually be tuned to, in the `(850 + ch)` MHz convention.
    /// `tuned` is the fallback when the node's frequency span names no channel in that convention
    /// (e.g. a 2.4 GHz LR2021) — reporting the one channel we know is live beats reporting none.
    pub fn channels(&self, tuned: u8) -> Vec<u8> {
        let chans: Vec<u8> = (0u8..=255)
            .filter(|&ch| {
                let hz = channel_to_hz(ch);
                hz >= self.freq_min_hz && hz <= self.freq_max_hz
            })
            .collect();
        if chans.is_empty() { vec![tuned] } else { chans }
    }

    /// The regulatory TX-airtime ceiling for the band this carrier sits in.
    ///
    /// **`0.01` is the ETSI EU868 figure and is simply wrong in the US.** FCC part 15.247 imposes no
    /// duty fraction on a digitally-modulated 902–928 MHz system (it constrains power and, for
    /// frequency hoppers, dwell — neither is a duty cycle), so a US-915 node that declared 1% was
    /// telling the planner it may use 1/100th of the airtime it actually may. Decided from the
    /// carrier rather than baked into a constructor.
    pub fn duty_cycle_max(&self, carrier_hz: u32) -> f32 {
        match carrier_hz {
            863_000_000..=870_000_000 => 0.01, // ETSI EN 300 220 g1 band: 1% duty cycle
            902_000_000..=928_000_000 => 1.0,  // FCC 15.247 digital modulation: no duty fraction
            _ => 1.0, // unknown band: do not invent a restriction the planner would obey
        }
    }

    /// The RF band(s) this node's frequency span lands in — the coarse reach axis.
    pub fn bands(&self) -> Vec<Band> {
        let mut v = Vec::new();
        if self.freq_min_hz < 1_000_000_000 {
            v.push(Band::Sub1GHz);
        }
        if self.freq_max_hz >= 2_400_000_000 {
            v.push(Band::Band2_4GHz);
        }
        if v.is_empty() {
            v.push(Band::Sub1GHz);
        }
        v
    }
}

/// Build a `cmd_bitmap` from a list of opcodes.
const fn cmd_bits(cmds: &[u8]) -> u32 {
    let mut m = 0u32;
    let mut i = 0;
    while i < cmds.len() {
        m |= 1u32 << cmds[i];
        i += 1;
    }
    m
}

/// **Which node to assume when its firmware predates `CMD_GET_CAP`.**
///
/// A hint is a *fallback*, never an override: [`LoraSerialBackend::open_as`] still asks the device
/// first, and a real `EVT_CAP` always wins. Each variant's profile is written down below with the
/// source of every number.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum RadioKindHint {
    /// The Waveshare USB-TO-LoRa dongle (GD32 + SX1262) running `firmware/waveshare-lora-rs`.
    WaveshareSx1262,
    /// The Heltec LoRa32 V2 (ESP32 + SX1276) running `firmware/heltec-lora-rs`.
    HeltecSx1276,
    /// The XIAO nRF54L15 + LR2021 bridge running `firmware/lr2021-nrf54l15-rs` (`m6_bridge`).
    Lr2021Flrc,
}

impl RadioKindHint {
    /// The pinned profile for this node. See [`NodeProfile::legacy_sx1262`] and friends.
    pub fn profile(self) -> NodeProfile {
        match self {
            RadioKindHint::WaveshareSx1262 => NodeProfile::legacy_sx1262(),
            RadioKindHint::HeltecSx1276 => NodeProfile::heltec_sx1276(),
            RadioKindHint::Lr2021Flrc => NodeProfile::lr2021_flrc(),
        }
    }
}

impl NodeProfile {
    /// **The legacy profile** — exactly what this backend assumed about every node before `EVT_CAP`
    /// existed, so an un-reflashed Waveshare dongle opens and behaves as it always has.
    ///
    /// Sources, field by field: the 902–928 MHz span is the image calibration the firmware performs
    /// (`firmware/waveshare-lora-rs/src/sx1262.rs`, "US 902-928 image cal"); 10–22 dBm is the span
    /// this file has always clamped to and that [`RadioCapability::lora`] advertises; `stamp_hz =
    /// 1000` / `SoftwareCounter` is the firmware's `ts_ms` MCU millisecond counter; the command set
    /// is opcodes `0x01..=0x16`, every one of which that firmware implements; SF 7–12 is the SX1262
    /// LoRa span. `sched_gran_ns = 0` because that firmware has no scheduled-TX path at all — not
    /// because it was not measured.
    pub fn legacy_sx1262() -> Self {
        Self {
            proto_ver: 1, // pre-v2: this node never answered CMD_GET_CAP
            radio_kind: LoraRadioKind::Sx1262,
            freq_min_hz: 902_000_000,
            freq_max_hz: 928_000_000,
            pwr_min_dbm: LEGACY_PWR_MIN_DBM,
            pwr_max_dbm: LEGACY_PWR_MAX_DBM,
            stamp_hz: 1_000,
            stamp_kind: StampKind::SoftwareCounter,
            // NOT `MAX_LORA_PAYLOAD` (240) — that is what this firmware ACCEPTS on TX; its receive
            // path truncates at 64 and says nothing. See `LEGACY_RX_TRUNCATION_CAP`.
            max_payload: LEGACY_RX_TRUNCATION_CAP,
            cmd_bitmap: cmd_bits(&[
                CMD_TX,
                CMD_SET_FREQ,
                CMD_SET_MOD,
                CMD_SET_PWR,
                CMD_SET_SYNC,
                CMD_GET_INFO,
                CMD_SET_BEACON,
                CMD_CAD,
                CMD_GET_RSSI,
                CMD_SET_CAD_CFG,
                CMD_SET_LBT_CFG,
                CMD_SET_PREAMBLE,
                CMD_SF_SCAN,
                CMD_TX_LBT,
                CMD_SET_NAME_FILTER,
                CMD_SET_RELAY,
                CMD_DATAPLANE,
                CMD_SET_SENSE_CFG,
                CMD_GET_STATS,
                CMD_RESET_STATS,
                CMD_SET_DEBUG,
                CMD_ENTER_BOOTLOADER,
            ]),
            sf_min: 7,
            sf_max: 12,
            sched_gran_ns: 0,
            // A pre-v2 firmware has no CMD_SET_PHY, so LoRa is not merely what it runs — it is the
            // only mode reachable on it. One entry, and the SX1262's GFSK mode is deliberately NOT
            // listed: unreachable is not available.
            phy_bitmap: PhyModeSet::single(PhyMode::Lora).bits(),
            phy_current: PhyMode::Lora,
            learned: false,
        }
    }

    /// The Heltec LoRa32 V2 (ESP32 + SX1276) as `firmware/heltec-lora-rs` currently stands.
    ///
    /// ★ **No longer a guess.** This profile was written blind (the board's sshd was down and it had
    /// never been flashed); the board is now flashed with v2 and answers `CMD_GET_CAP`, and three
    /// fields below were wrong. They are corrected here, each corroborated by the board's own
    /// `EVT_CAP` — which is also the reason this fallback is now nearly unreachable: a v2 board
    /// describes itself and none of this is consulted.
    ///
    /// * **2–20 dBm, not 2–17.** The board reports 20. The V2 wires the antenna to PA_BOOST and
    ///   `lora-phy` is configured with `tx_boost: true`, whose clamp is the +20 dBm PA_BOOST interval
    ///   (`PWR_MIN_DBM`/`PWR_MAX_DBM` in that firmware) — the datasheet's "+17 unless PA_DAC" reading
    ///   was of the wrong PA path, and it under-declared the link budget by 3 dB.
    /// * **`stamp_hz` = 1 MHz, not 1 kHz.** The firmware stamps with `embassy_time::TICK_HZ`, which
    ///   esp-rtos selects as `tick-hz-1_000_000`; the old `ts_ms` reading was a **1000× tick-scale
    ///   error**, exactly the class this whole profile exists to prevent.
    /// * **`max_payload` 247** — the firmware's `RX_MAX`, now the same `LORA_NODE_RX_MAX` the
    ///   Waveshare reports, rather than a host constant that happened to sit nearby.
    ///
    /// The command set is the opcodes that firmware actually decodes (note `CMD_SET_SYNC` is
    /// **absent** — it acks unknown commands with `EVT_INFO`, so without this gate the host would
    /// "successfully" set a sync word that was never applied).
    pub fn heltec_sx1276() -> Self {
        Self {
            proto_ver: 1,
            radio_kind: LoraRadioKind::Sx1276,
            freq_min_hz: 902_000_000,
            freq_max_hz: 928_000_000,
            pwr_min_dbm: 2,
            // 20, not 17: the flashed board's EVT_CAP says so (PA_BOOST with lora-phy `tx_boost`).
            pwr_max_dbm: 20,
            // embassy-time TICK_HZ under esp-rtos `tick-hz-1_000_000`, per the board's EVT_CAP —
            // NOT the millisecond counter this fallback used to assume.
            stamp_hz: 1_000_000,
            stamp_kind: StampKind::SoftwareCounter,
            max_payload: LORA_NODE_RX_MAX,
            cmd_bitmap: cmd_bits(&[
                CMD_TX,
                CMD_SET_FREQ,
                CMD_SET_MOD,
                CMD_SET_PWR,
                CMD_GET_INFO,
                CMD_CAD,
                CMD_GET_RSSI,
                CMD_TX_LBT,
                CMD_GET_STATS,
            ]),
            sf_min: 7,
            sf_max: 12,
            sched_gran_ns: 0,
            // The SX1276 also does FSK and OOK, and that firmware exposes no way to reach either.
            phy_bitmap: PhyModeSet::single(PhyMode::Lora).bits(),
            phy_current: PhyMode::Lora,
            learned: false,
        }
    }

    /// The XIAO nRF54L15 + LR2021 bridge (`m6_bridge`), FLRC at 915 MHz.
    ///
    /// ★ **`CMD_SET_FREQ` is deliberately absent from the bitmap.** Retuning that firmware was
    /// MEASURED to permanently break its transmit path, so the capability gate is the mechanism that
    /// keeps a generic host — which would otherwise send `SET_FREQ` at every open — from bricking the
    /// link. The frequency span is pinned to the one carrier the board is known-good on for the same
    /// reason.
    ///
    /// `stamp_hz = 16_000_000` is MEASURED, not assumed: 15 frames at ~1 s spacing gave deltas of
    /// 16 616 401..16 625 857 ticks, and `firmware/lr2021-nrf54l15-rs/src/timing.rs` declares
    /// `TICKS_PER_US = 16`. That 62.5 ns hardware capture is what makes this node the only one in the
    /// fleet that can source common view. `max_payload = 47` is `flrc_link::PAYLOAD_MAX` — the fixed
    /// 48-byte on-air PDU (`FRAME_LEN`) **minus the in-frame length byte**, far below the 255 bytes
    /// its serial parser would accept, and the smaller side is the one that must be reported. It read
    /// 48 until this run, one byte over what the frame can actually carry; the flashed board's own
    /// `EVT_CAP` reports 47 and corroborates the correction. Power: this firmware implements
    /// `CMD_SET_PWR` but has
    /// never declared or measured a dBm span, so the range is left **unknown (0/0)** and the absolute
    /// power knob refuses rather than clamping into an invented one.
    ///
    /// ⚠ This profile describes the **pre-v2 `m6_bridge`** — the build that shipped before
    /// `CMD_GET_CAP`, and the only situation in which this fallback is ever used. That build also
    /// numbered `EVT_STATS` as 0x87, which the fleet assigns to `EVT_SF_DETECTED`, so
    /// [`ndn_stats`](LoraSerialBackend::ndn_stats) times out against it; the host deliberately does
    /// not shim that, because a compatibility shim would entrench the divergence in a second place.
    /// A board carrying the v2 firmware answers `CMD_GET_CAP` and none of this is consulted — which
    /// is the point: the numbers here are a floor for an un-reflashed board, not a claim about what
    /// that firmware is today.
    pub fn lr2021_flrc() -> Self {
        Self {
            proto_ver: 1,
            radio_kind: LoraRadioKind::Lr2021,
            freq_min_hz: 915_000_000,
            freq_max_hz: 915_000_000,
            pwr_min_dbm: 0, // UNKNOWN — never declared in firmware, never measured
            pwr_max_dbm: 0,
            stamp_hz: 16_000_000, // MEASURED (62.5 ns); timing.rs TICKS_PER_US = 16
            stamp_kind: StampKind::HardwareFreeRun,
            // flrc_link::PAYLOAD_MAX = FRAME_LEN (48) − the in-frame length byte. Corroborated by
            // the flashed board's EVT_CAP, which reports 47.
            max_payload: 47,
            cmd_bitmap: cmd_bits(&[
                CMD_TX,
                CMD_SET_PWR,
                CMD_GET_INFO,
                CMD_GET_RSSI,
                CMD_SET_NAME_FILTER,
                CMD_SET_RELAY,
                CMD_DATAPLANE,
                CMD_GET_STATS,
                CMD_RESET_STATS,
            ]),
            sf_min: 0, // FLRC has no spreading factor — not "SF 7", none
            sf_max: 0,
            // ★ The part runs fourteen modulations and this firmware reaches ONE. `phy_bitmap` is
            // what a host can actuate, not what the datasheet lists: the pre-v2 `m6_bridge` calls
            // `set_packet_type(Flrc)` once at bring-up and implements no CMD_SET_PHY, so FLRC is
            // the node's entire modulation set until it is reflashed. Listing LoRa/BLE/Z-Wave here
            // would advertise a knob with no actuator — the exact defect this pass removes.
            phy_bitmap: PhyModeSet::single(PhyMode::Flrc).bits(),
            phy_current: PhyMode::Flrc,
            // 0 because the PRE-V2 build had no scheduled-TX path — not a statement about
            // `m6_bridge` today, which implements `CMD_TX_AT` and declares 50 000 ns in its own
            // `EVT_CAP`. A board reaching this fallback is by definition running the older image.
            sched_gran_ns: 0,
            learned: false,
        }
    }
}

/// Resolve the profile to use: the device's own `EVT_CAP` if it answered, else the pinned hint, else
/// the legacy profile. Split out as a pure function so the fallback path is testable without hardware.
fn resolve_profile(cap: Option<&[u8]>, hint: Option<RadioKindHint>) -> NodeProfile {
    cap.and_then(NodeProfile::parse).unwrap_or_else(|| {
        hint.map(RadioKindHint::profile)
            .unwrap_or_else(NodeProfile::legacy_sx1262)
    })
}

/// **Decode [`EVT_CLOCK_REF`] into a [`ClockReference`].** `None` if the payload is short — a
/// truncated capability is not a capability, exactly as [`NodeProfile::parse`] holds.
///
/// The `ref_class` byte is taken at face value and nothing is inferred around it: a node saying
/// `CLOCK_REF_UNKNOWN` stays unknown here, because "the node looked and could not tell" is a real
/// answer and is not improved by the host guessing on its behalf.
///
/// `accuracy_ppm` is a magnitude the NODE measured about itself. It is carried as
/// [`RateWitness::NodeReported`] and never mistaken for a figure taken on this side; when it reads
/// [`CLOCK_ACCURACY_UNKNOWN`] there is simply no measurement, which is the normal case (the
/// Waveshare deliberately reports the sentinel rather than publishing the -1.6 ppm relative trim
/// between two of its boards as if it were an accuracy spec).
fn parse_clock_ref(p: &[u8]) -> Option<ClockReference> {
    if p.len() < 3 {
        return None;
    }
    let kind = match p[0] {
        CLOCK_REF_XTAL => ClockReferenceKind::Crystal,
        CLOCK_REF_RC => ClockReferenceKind::RcOscillator,
        CLOCK_REF_UNKNOWN => ClockReferenceKind::Unknown,
        // A class this host does not speak. Refuse it rather than round it to something flattering:
        // an unrecognised code is not evidence of a good reference.
        _ => ClockReferenceKind::Unknown,
    };
    let r = ClockReference {
        kind,
        measured: None,
    };
    let ppm = u16::from_be_bytes([p[1], p[2]]);
    Some(if ppm == CLOCK_ACCURACY_UNKNOWN {
        r
    } else {
        r.measured(RateMeasurement::new(
            f32::from(ppm),
            0.0, // the wire carries no observation span
            RateWitness::NodeReported,
        ))
    })
}

/// **What to assume about a node's clock reference when it will not say.**
///
/// The rule: a host-side assumption may only ever WITHHOLD a capability — *unless the device itself
/// put the deciding fact on the wire*, in which case reading that fact is not an assumption at all.
///
/// ## Why there is a fallback at all
///
/// [`CMD_GET_CLOCK_REF`] is how a node states this, and it is asked at open. Not every firmware in
/// the fleet answers it — the Heltec SX1276 replies `EVT_UNSUPPORTED`, and any board still carrying
/// an older image says nothing — so for those this is the only answer available. It is the same
/// shape as [`RadioKindHint`]: a written-down fallback, consulted only when the device itself is
/// silent, and overridden the instant it speaks.
///
/// ## Why this takes the whole profile and not just the modem
///
/// [`NodeProfile::learned`] is the fact that decides the LR2021 row below: `true` means every field
/// came out of the device's own `EVT_CAP`, `false` means the host pinned it from a
/// [`RadioKindHint`] because nothing answered. Keying on `radio_kind` alone threw that away — and on
/// this fleet it is not decoration, it is the whole answer.
///
/// The modem is still not the oscillator: `EVT_CAP[1]` names the *radio chip*, while the counter
/// belongs to the MCU beside it. So a row may only move in the safe direction on the strength of the
/// modem byte; the LR2021 row moves on something else, spelled out below.
///
/// * [`LoraRadioKind::Sx1262`] -> **RC oscillator.** The only SX1262 node here is the Waveshare
///   USB-TO-LoRa dongle, whose entire timebase (SysTick, TIM2's deadline, TIM3's capture) descends
///   from the GD32's 8 MHz **HSI RC** on every build that predates the crystal switch — MEASURED
///   ~-3100 ppm relative between two dongles, with the two-receiver residual GROWING from 16 to 130
///   us with the fit span. Wrong here costs a capability the node might have had, which is the
///   direction to be wrong in. And a dongle carrying the newer firmware answers
///   [`CMD_GET_CLOCK_REF`] with the crystal it actually probed, so this is never consulted for it.
/// * [`LoraRadioKind::Sx1276`] -> **unknown.** Nothing in this tree records what an ESP32's
///   `embassy_time` tick runs on, and this node stamps with a software counter anyway, so it
///   advertises no `FreeRunRxStamp` and the reference gates nothing.
/// * [`LoraRadioKind::Lr2021`] -> **crystal iff this profile was LEARNED**, unknown otherwise.
///
/// ## ★ Why an `EVT_CAP` from an LR2021 is itself a statement about its oscillator
///
/// The XIAO nRF54L15 bridge runs its timebase on the HFXO: `firmware/lr2021-nrf54l15-rs/src/hw.rs`
/// forces `HfclkSource::ExternalXtal` for every binary that reports a timestamp — precisely because
/// the internal RC it replaced MEASURED **+2253 / +2019 ppm** against a precise host cadence
/// (`hw.rs`, two on-air runs), where the same node on the crystal reads **+16.7 ppm**. The
/// two-receiver residual on that crystal is 0.81-1.86 us and stays FLAT from a 1.4 s fit span to
/// 10.2 s, against 10.5-20.4 us GROWING to 130 us for two RC-referenced Waveshares.
///
/// What was missing was never the evidence — it was a way to tell WHICH BUILD is on the far end,
/// since the RC build had an identical `stamp_kind` and an identical 16 MHz `stamp_hz`. This file
/// used to assert that "nothing on the wire separates the two". That is FALSE, and the repository
/// is the witness (re-run these; they are the whole justification for this row):
///
/// ```text
/// $ git log --oneline -S hfclk_source -- firmware/    # the only commit that ever set the source
/// 1283b7e feat(lora-family): 7E-A5 v2 - the radio describes itself, and three correctness fixes
/// $ git show 1283b7e~1:firmware/lr2021-nrf54l15-rs/src/bin/m6_bridge.rs | grep -c EVT_CAP
/// 0
/// $ git show 1283b7e:firmware/lr2021-nrf54l15-rs/src/bin/m6_bridge.rs   | grep -c EVT_CAP
/// 3
/// ```
///
/// The `HfclkSource::ExternalXtal` LINE was added to `src/hw.rs` by 1283b7e — the file itself is
/// older (added by ade3f4d, five commits earlier), so audit the line and not the file:
///
/// ```text
/// $ git log --oneline -S hfclk_source -- firmware/     # the line, everywhere: one commit
/// 1283b7e
/// $ git log --oneline --diff-filter=A -- firmware/lr2021-nrf54l15-rs/src/hw.rs   # the FILE
/// ade3f4d
/// ```
///
/// The build before 1283b7e opened with `embassy_nrf::init(Default::default())`, i.e. the internal
/// RC, and speaks 7E-A5 **v1**, which has no `CMD_GET_CAP` and no `EVT_CAP` frame to answer with.
/// `m6_bridge` is the only binary in that firmware that speaks this protocol at all. (`m1_bare`
/// still does not boot through `hw::init_peripherals()`, by design — it initialises nothing and
/// emits no `EVT_CAP`, so nothing here rests on it.) So, on this fleet:
///
/// > an LR2021 that ANSWERED `CMD_GET_CAP` is necessarily running a build whose HFXO line is in it.
///
/// That is a fact about the far end carried by a frame the far end sent — not a hope about what its
/// firmware ought to be doing — which is why it may promote where the modem byte may not. A node
/// that did NOT answer (`learned == false`: [`open_as`](LoraSerialBackend::open_as), a pre-v2 image,
/// a silent port) is exactly the case that cannot be told apart, and it stays
/// [`ClockReference::unknown`].
///
/// ⚠ The two ways this stops holding, neither of which is this function's to guess: an LR2021 build
/// that emits `EVT_CAP` while running the RC (nothing in this tree does — and if one is ever built,
/// give it a `CMD_GET_CLOCK_REF` handler rather than a special case here), and any node that answers
/// [`CMD_GET_CLOCK_REF`], whose answer is parsed first in `open_inner` so this is never consulted.
///
/// ## When this row is reached at all, after 4d01eb1
///
/// `m6_bridge` gained a `CMD_GET_CLOCK_REF` handler and both boards on the bench were flashed and
/// MEASURED answering `02 ff ff` — crystal, accuracy not measured. A board carrying that image never
/// reaches this function. It stays because a fallback's job is the board that has NOT been reflashed
/// (and `open_as`, and a port that says nothing), and because the promotion above and the node's own
/// answer now agree exactly — `ClockReference::crystal()` with `measured: None` — so the two paths
/// cannot drift into disagreeing about the same board.
fn assumed_clock_reference(prof: &NodeProfile) -> ClockReference {
    match prof.radio_kind {
        LoraRadioKind::Sx1262 => ClockReference::rc_oscillator().measured(RateMeasurement::new(
            -3100.0,
            0.0, // "~-3100 ppm relative between two dongles"; the run records no span
            RateWitness::PeerUnit,
        )),
        // The device's own `EVT_CAP` is the witness — see "★" above. The CLASS is what that fact
        // supports, and only the class is claimed: `measured: None`, deliberately. The +16.7 ppm on
        // record for this board is one session's comparison and the tree does not agree with itself
        // on what it was taken against (`README.md:36` reads it as a host cadence, the node's own
        // firmware commit calls it inter-node), so it is not a rate this host can stand behind.
        // Note the same choice on the wire: the node's `CMD_GET_CLOCK_REF` handler answers
        // `[crystal][CLOCK_ACCURACY_UNKNOWN]` rather than publishing that figure — so a reflashed
        // board and this fallback now produce the IDENTICAL `ClockReference`, which is the shape a
        // fallback should have.
        LoraRadioKind::Lr2021 if prof.learned => ClockReference::crystal(),
        // Everything else: Sx1276 (Heltec/ESP32), an un-learned LR2021, and any modem byte this host
        // does not recognise. Unknown, for the reasons above, and never promoted from this side.
        _ => ClockReference::unknown(),
    }
}

// ---------------------------------------------------------------------------
// Airtime — the physics that bounds every transmit wait.
// ---------------------------------------------------------------------------

/// **LoRa time-on-air, milliseconds** (Semtech AN1200.13 / SX1276 datasheet §4.1.1.7).
///
/// ```text
///   T_sym     = 2^SF / BW
///   T_preamble= (n_preamble + 4.25) · T_sym
///   n_payload = 8 + max(ceil((8·PL − 4·SF + 28 + 16·CRC − 20·IH) / (4·(SF − 2·DE))), 0) · (CR + 4)
/// ```
///
/// with an explicit header (`IH = 0`), CRC on, and the low-data-rate optimisation `DE = 1` where the
/// standard requires it (SF11/SF12 at 125 kHz). This is why a fixed 3 s transmit timeout was a bug
/// rather than a rounding error: at SF12/125 kHz/CR 4-5 a 240-byte frame is **~8.5 s** on air, so the
/// host abandoned every long-reach transmission it ever asked for, ~5.5 s before the radio finished.
fn lora_airtime_ms(sf: u8, bw_khz: u32, cr: u8, payload: usize, preamble: u16) -> u64 {
    let sf = sf.clamp(6, 12) as f64;
    let bw = (bw_khz.max(1) * 1000) as f64;
    let t_sym = 2f64.powf(sf) / bw; // seconds
    // Low-data-rate optimisation is mandatory where a symbol exceeds 16 ms — SF11/12 at 125 kHz.
    let de = if t_sym > 0.016 { 1.0 } else { 0.0 };
    let cr = cr.clamp(1, 4) as f64;
    let num = 8.0 * payload as f64 - 4.0 * sf + 28.0 + 16.0; // explicit header, CRC on
    let den = 4.0 * (sf - 2.0 * de);
    let n_payload = 8.0 + (num / den).ceil().max(0.0) * (cr + 4.0);
    let n_preamble = preamble as f64 + 4.25;
    (((n_preamble + n_payload) * t_sym) * 1000.0).ceil() as u64
}

/// Slack added on top of a frame's airtime before a transmit is called lost: the serial round trip,
/// the firmware's standby/apply/re-arm, and the host scheduler. Generous on purpose — the cost of
/// waiting too long is latency, the cost of waiting too little is a phantom transmit failure.
const AIRTIME_SLACK: Duration = Duration::from_millis(2_000);
/// Floor under any transmit wait: the pre-existing 3 s budget, which is known to work for the short
/// low-SF frames the fleet runs today. Airtime only ever raises it.
const TXDONE_MIN_TIMEOUT: Duration = Duration::from_millis(3_000);
/// How long to wait for a knob to be acknowledged. Each `SET_*` is standby -> apply -> re-arm RX ->
/// reply, all well under this.
const INFO_TIMEOUT: Duration = Duration::from_millis(1_000);
/// How long to wait for an open-path self-description probe: `CMD_GET_CAP` -> `EVT_CAP`, and
/// `CMD_GET_CLOCK_REF` -> `EVT_CLOCK_REF`. One short reply from an idle node; a node that does not
/// implement the opcode costs exactly this once, then falls back to what the host has written down.
const CAP_TIMEOUT: Duration = Duration::from_millis(600);
/// How long to wait for a `CMD_SET_PHY`. Longer than [`INFO_TIMEOUT`] because the node re-programs
/// its packet engine — standby, `SetPacketType`, re-apply modulation/packet params, re-arm RX —
/// before it can describe itself again, and a modulation change is the one knob on this bearer that
/// is genuinely a re-initialisation rather than a register write.
const PHY_SWITCH_TIMEOUT: Duration = Duration::from_millis(2_000);
/// How long to wait for a *second* `EVT_CAP` after a `CMD_SET_PHY` whose first one named the wrong
/// mode. Sized to one serial round trip, not to a retry: it only covers the case where an
/// unsolicited capability push (see [`handle_event`]) reached the channel before the real reply.
const CAP_RACE_WINDOW: Duration = Duration::from_millis(400);
/// How many times to re-send an idempotent knob whose reply never came. See `exec_idempotent`.
const KNOB_ATTEMPTS: usize = 4;

/// The firmware's listen-before-talk tunables, mirrored host-side so the transmit deadline can
/// include the worst-case contention budget instead of a magic constant. Defaults match
/// `firmware/waveshare-lora-rs` (`Csma::new`).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
struct LbtCfg {
    cw_ms: u16,
    max_backoff: u8,
    max_attempts: u8,
}

impl Default for LbtCfg {
    fn default() -> Self {
        Self {
            cw_ms: 20,
            max_backoff: 4,
            max_attempts: 6,
        }
    }
}

impl LbtCfg {
    /// Worst-case time the firmware may spend sensing and backing off before it keys up (or gives up):
    /// `attempts × cw × 2^backoff`, the bound the firmware's own loop enforces.
    fn worst_case(&self) -> Duration {
        let per = self.cw_ms as u64 * (1u64 << self.max_backoff.min(16));
        Duration::from_millis(per.saturating_mul(self.max_attempts.max(1) as u64))
    }
}

/// Radio parameters programmed over the binary protocol at open. Two nodes must agree on frequency,
/// SF, BW, CR and sync word to hear each other; [`Default`] is 915 MHz (US) / SF7 / 125 kHz / 4-5 /
/// private sync — matching the firmware defaults and the Heltec interop node.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct LoraParams {
    /// Spreading factor 7–12. Higher = longer range, exponentially slower. Ignored by a node with no
    /// spreading factor (FLRC).
    pub sf: u8,
    /// **Host** bandwidth code: 0 = 125 kHz, 1 = 250 kHz, 2 = 500 kHz. Translated to the node's own
    /// register convention by [`bw_to_fw`] — the two are NOT the same byte.
    pub bw: u8,
    /// Coding-rate code: 1 = 4/5 … 4 = 4/8.
    pub cr: u8,
    /// TX channel index; carrier = `(850 + tx_ch)` MHz.
    pub tx_ch: u8,
    /// RX channel index; LoRa is half-duplex so this tracks `tx_ch`.
    pub rx_ch: u8,
    /// TX power, dBm.
    pub pwr: u8,
    /// LoRa sync word in the SX127x single-byte convention: 0x12 private / 0x34 public.
    pub sync: u8,
    /// Emit the firmware's on-air heartbeat beacon. Off by default when a host drives the node (its
    /// own traffic is liveness enough); set true to keep the node discoverable on-air.
    pub beacon: bool,
    /// Preamble length in symbols — mirrors the firmware default so airtime is computed from the
    /// value actually programmed (see [`LoraSerialBackend::set_preamble`]).
    pub preamble: u16,
}

impl Default for LoraParams {
    fn default() -> Self {
        Self {
            sf: 7,
            bw: 0,
            cr: 1,
            tx_ch: 65, // 915 MHz (US ISM)
            rx_ch: 65,
            pwr: 22,
            sync: 0x12,
            beacon: false, // host-driven: silence the firmware beacon at open
            preamble: 8,   // firmware default
        }
    }
}

impl LoraParams {
    /// Channel bandwidth in kHz for the current [`bw`](Self::bw) code.
    pub fn bw_khz(&self) -> u32 {
        BW_KHZ[(self.bw as usize).min(BW_KHZ.len() - 1)]
    }
}

/// A raw serial tty opened with libc termios — a `cfmakeraw` / `CLOCAL` / `8N1` port, exactly what
/// `stty raw clocal` gives. The `serialport` crate does not exchange bytes with the CH340 bridge
/// on aarch64-musl (the OPi target), so we own the termios setup ourselves; this also drops the
/// `serialport` dependency for the LoRa backend and cross-compiles cleanly.
struct SerialFd {
    fd: RawFd,
}

impl SerialFd {
    fn open(path: &str, baud: u32) -> std::io::Result<Self> {
        let cpath = std::ffi::CString::new(path)
            .map_err(|_| std::io::Error::other("path has a NUL byte"))?;
        // O_NONBLOCK during open avoids blocking on carrier-detect; cleared right after so reads
        // then block under VMIN/VTIME control.
        let fd = unsafe {
            libc::open(
                cpath.as_ptr(),
                libc::O_RDWR | libc::O_NOCTTY | libc::O_NONBLOCK,
            )
        };
        if fd < 0 {
            return Err(std::io::Error::last_os_error());
        }
        unsafe {
            let fl = libc::fcntl(fd, libc::F_GETFL);
            libc::fcntl(fd, libc::F_SETFL, fl & !libc::O_NONBLOCK);
        }
        let me = SerialFd { fd };
        me.set_termios(baud)?;
        Ok(me)
    }

    fn set_termios(&self, baud: u32) -> std::io::Result<()> {
        let speed: libc::speed_t = match baud {
            9600 => libc::B9600,
            19200 => libc::B19200,
            38400 => libc::B38400,
            57600 => libc::B57600,
            _ => libc::B115200,
        };
        unsafe {
            let mut t: libc::termios = std::mem::zeroed();
            if libc::tcgetattr(self.fd, &mut t) != 0 {
                return Err(std::io::Error::last_os_error());
            }
            libc::cfmakeraw(&mut t); // 8N1, no echo, no canon, no flow xlate
            libc::cfsetispeed(&mut t, speed);
            libc::cfsetospeed(&mut t, speed);
            t.c_cflag |= libc::CLOCAL | libc::CREAD; // ignore modem lines, enable receiver
            t.c_cflag &= !libc::CRTSCTS; // no hardware flow control
            t.c_cc[libc::VMIN] = 0; // read returns after VTIME even with no data…
            t.c_cc[libc::VTIME] = 2; // …a 0.2 s inter-read timeout
            if libc::tcsetattr(self.fd, libc::TCSANOW, &t) != 0 {
                return Err(std::io::Error::last_os_error());
            }
            libc::tcflush(self.fd, libc::TCIOFLUSH);
        }
        Ok(())
    }

    fn flush_input(&self) {
        unsafe {
            libc::tcflush(self.fd, libc::TCIFLUSH);
        }
    }

    fn read(&self, buf: &mut [u8]) -> std::io::Result<usize> {
        loop {
            let n =
                unsafe { libc::read(self.fd, buf.as_mut_ptr() as *mut libc::c_void, buf.len()) };
            if n < 0 {
                let e = std::io::Error::last_os_error();
                if e.kind() == std::io::ErrorKind::Interrupted {
                    continue;
                }
                return Err(e);
            }
            return Ok(n as usize);
        }
    }

    fn write_all(&self, mut buf: &[u8]) -> std::io::Result<()> {
        while !buf.is_empty() {
            let n = unsafe { libc::write(self.fd, buf.as_ptr() as *const libc::c_void, buf.len()) };
            if n < 0 {
                let e = std::io::Error::last_os_error();
                if e.kind() == std::io::ErrorKind::Interrupted {
                    continue;
                }
                return Err(e);
            }
            buf = &buf[n as usize..];
        }
        Ok(())
    }

    fn flush(&self) -> std::io::Result<()> {
        unsafe {
            libc::tcdrain(self.fd);
        }
        Ok(())
    }

    fn try_clone(&self) -> std::io::Result<SerialFd> {
        let fd = unsafe { libc::dup(self.fd) };
        if fd < 0 {
            Err(std::io::Error::last_os_error())
        } else {
            Ok(SerialFd { fd })
        }
    }
}

impl Drop for SerialFd {
    fn drop(&mut self) {
        unsafe {
            libc::close(self.fd);
        }
    }
}

/// The command side of the link: the port, plus the firmware's replies to whatever we last sent.
///
/// **This protocol is strictly request/response, one command in flight.** Every handler in the
/// firmware runs to completion before the main loop drains the USART again — and the MCU's USART has
/// no FIFO, so a second command sent while the first is still being serviced is not queued, it is
/// destroyed by an overrun that neither side reports. A blocking transmit is the worst case (seconds
/// at high SF), but even `SET_MOD` (standby -> apply -> re-arm -> reply) is long enough to swallow the
/// command behind it. So: hold this lock, send, and wait for the reply.
struct CmdPort {
    port: SerialFd,
    /// `(event type, payload)` for every non-RX event the reader parses.
    resp: std::sync::mpsc::Receiver<(u8, Vec<u8>)>,
}

/// Send one command and wait for the node's reply, with no other command in flight.
///
/// A free function rather than a method so the async data path can hand it to `spawn_blocking`
/// without borrowing `&self` across an await point.
///
/// `tx_wait`: when the node emits `EVT_TX_STARTED [airtime_ms]` it is telling us the airtime it is
/// about to spend — better than any host-side estimate, because it knows the parameters actually
/// programmed. Consume it and re-base the deadline on it. (This event used to be silently discarded.)
fn exec_on(
    cmd: &Mutex<CmdPort>,
    typ: u8,
    payload: &[u8],
    expect: u8,
    timeout: Duration,
    tx_wait: bool,
) -> Result<Vec<u8>, FaceError> {
    let cmd = cmd.lock().unwrap();
    // Drop replies to earlier commands (e.g. one that timed out) so we read *this* one's.
    while cmd.resp.try_recv().is_ok() {}
    send_cmd(&cmd.port, typ, payload).map_err(|e| io_err(format!("lora cmd {typ:#04x}: {e}")))?;
    let mut deadline = Instant::now() + timeout;
    loop {
        let left = deadline.saturating_duration_since(Instant::now());
        if left.is_zero() {
            return Err(io_err(format!(
                "lora cmd {typ:#04x}: no reply in {timeout:?}"
            )));
        }
        match cmd.resp.recv_timeout(left) {
            Ok((t, p)) if t == expect => return Ok(p),
            // The node answering "I do not implement that" is a definite NO — return it now rather
            // than burning the whole timeout waiting for a reply that will never come.
            Ok((EVT_UNSUPPORTED, p)) if p.first() == Some(&typ) => {
                return Err(unsupported(format!(
                    "node does not implement 7E-A5 command {typ:#04x} (reason {})",
                    p.get(1).copied().unwrap_or(0)
                )));
            }
            // A PHY the node ADVERTISES that the chip refused this time — equally definite, and a
            // different fact from `EVT_UNSUPPORTED`: the mode exists, the silicon said no. Carries
            // the chip's literal status byte so the refusal can be diagnosed rather than guessed at.
            //
            // ★ **This arm is terminal because it fires only when the error arrives FIRST.** The v3
            // contract splits the two things a node can refuse:
            //
            // * `EVT_PHY_ERR` **alone** — the chip would not bring the mode up. The node reverted;
            //   the profile this host holds is still correct, and returning here is right.
            // * `EVT_CAP` **then** `EVT_PHY_ERR` — the mode came up and the chip would not arm its
            //   receiver (the LR-FHSS case). The switch HAPPENED, so the CAP is the reply: it is
            //   matched by the `t == expect` arm above, the profile is replaced, and the trailing
            //   error is a diagnostic that `handle_event` logs.
            //
            // Treating the second ordering as a refusal would leave this host on the old mode's
            // `max_payload` and rate model while the node ran the new one — so the node emits the
            // capability first, and this arm must not be widened to swallow it.
            Ok((EVT_PHY_ERR, p)) if typ == CMD_SET_PHY => {
                return Err(unsupported(format!(
                    "node refused PHY {:#04x} at runtime (chip status {:#04x})",
                    p.first().copied().unwrap_or(0),
                    p.get(1).copied().unwrap_or(0)
                )));
            }
            Ok((EVT_TX_STARTED, p)) if tx_wait && p.len() >= 2 => {
                // The device's own airtime figure, in ms. Re-base the deadline on it.
                let airtime = u16::from_be_bytes([p[0], p[1]]) as u64;
                deadline = Instant::now() + Duration::from_millis(airtime) + AIRTIME_SLACK;
            }
            Ok(_) => continue, // some other event slipped in; keep waiting for ours
            Err(std::sync::mpsc::RecvTimeoutError::Timeout) => {
                return Err(io_err(format!(
                    "lora cmd {typ:#04x}: no reply in {timeout:?}"
                )));
            }
            Err(std::sync::mpsc::RecvTimeoutError::Disconnected) => {
                return Err(FaceError::Closed);
            }
        }
    }
}

/// Wait for one more event of type `expect` **without sending anything**, up to `timeout`.
///
/// Exists for exactly one situation: a `CMD_SET_PHY` whose `EVT_CAP` reply may have been beaten to
/// the channel by an *unsolicited* `EVT_CAP` (a node re-publishing its capability when its
/// self-measured scheduling granularity moves — see [`handle_event`]). The first CAP is then a true
/// statement about the node and a wrong answer to the question asked, so the caller gives the real
/// reply one short window rather than believing the racer. Returns `None` on timeout, which is not
/// an error: it means no second CAP came, so the first one *was* the reply.
fn recv_on(cmd: &Mutex<CmdPort>, expect: u8, timeout: Duration) -> Option<Vec<u8>> {
    let cmd = cmd.lock().unwrap();
    let deadline = Instant::now() + timeout;
    loop {
        let left = deadline.saturating_duration_since(Instant::now());
        if left.is_zero() {
            return None;
        }
        match cmd.resp.recv_timeout(left) {
            Ok((t, p)) if t == expect => return Some(p),
            Ok(_) => continue,
            Err(_) => return None,
        }
    }
}

/// A 7E-A5 serial radio node (Waveshare SX1262, Heltec SX1276, or LR2021 bridge) reached over its
/// serial port. What it can do is read from its [`NodeProfile`], not from its family name.
pub struct LoraSerialBackend {
    cmd: Arc<Mutex<CmdPort>>,
    rx: AsyncMutex<mpsc::UnboundedReceiver<CapturedFrame>>,
    /// Behind a mutex because [`RadioKnobs`] retunes the module at runtime; `capability()` and
    /// `params()` reflect the live values.
    params: Arc<Mutex<LoraParams>>,
    /// Learned at open from `EVT_CAP` (or pinned). Shared with the reader thread, which needs
    /// `stamp_kind`/`stamp_hz` to interpret every frame's `ts` field.
    profile: Arc<Mutex<NodeProfile>>,
    /// #52: when set, `inject` uses the firmware's atomic listen-before-talk (`CMD_TX_LBT`) instead
    /// of a plain blind `CMD_TX`. Off by default so a plain link on any node still works.
    lbt: AtomicBool,
    /// Mirror of the firmware's LBT tunables, so the transmit deadline includes the contention budget.
    lbt_cfg: Mutex<LbtCfg>,
    /// This device's own clock domain (per port — two nodes are two counters).
    device_domain: ClockDomainId,
    /// **What this node's stamp counter is derived from.** Learned at open from
    /// [`CMD_GET_CLOCK_REF`] where the firmware implements it, else
    /// [`assumed_clock_reference`]'s written-down, demote-only fallback.
    ///
    /// Not a field of [`NodeProfile`] on purpose: that struct is the `EVT_CAP` record, every field
    /// of which is on the wire and round-trips through `to_cap_payload`. This is a *separate* wire
    /// answer (or, failing that, a host assumption), and putting it in the capability record would
    /// let a fallback ride out of `to_cap_payload` looking like something a node said.
    ///
    /// Fixed for the lifetime of the handle: it is a property of the silicon and its boot-time clock
    /// selection, and unlike `EVT_CAP` no node re-publishes it.
    clock_reference: ClockReference,
}

impl LoraSerialBackend {
    /// Open the node at `path` with the default (915 MHz / SF7) parameters, learning its capability
    /// from `EVT_CAP`.
    pub fn open(path: &str) -> Result<Self, FaceError> {
        Self::open_inner(path, LoraParams::default(), None)
    }

    /// Open the node at `path`, learn its capability, program `params` (only the parts it supports,
    /// clamped into the ranges it declares), and spawn the reader that parses events and hands
    /// received NDN frames up as [`CapturedFrame`]s.
    pub fn open_with(path: &str, params: LoraParams) -> Result<Self, FaceError> {
        Self::open_inner(path, params, None)
    }

    /// Open a node whose firmware **predates `CMD_GET_CAP`**, pinning `hint`'s written-down profile
    /// as the fallback. The device is still asked first and its own `EVT_CAP` always wins — the hint
    /// only decides what to assume when nothing answers.
    pub fn open_as(path: &str, hint: RadioKindHint) -> Result<Self, FaceError> {
        Self::open_inner(path, LoraParams::default(), Some(hint))
    }

    /// [`open_as`](Self::open_as) with explicit radio parameters.
    pub fn open_as_with(
        path: &str,
        params: LoraParams,
        hint: RadioKindHint,
    ) -> Result<Self, FaceError> {
        Self::open_inner(path, params, Some(hint))
    }

    fn open_inner(
        path: &str,
        mut params: LoraParams,
        hint: Option<RadioKindHint>,
    ) -> Result<Self, FaceError> {
        let port = SerialFd::open(path, LORA_BAUD)
            .map_err(|e| io_err(format!("lora open {path}: {e}")))?;
        // Drop the boot chatter before anyone parses it. The port open does not reset the MCU (DTR is
        // not wired to nRST), so a short settle + flush suffices.
        port.flush_input();
        std::thread::sleep(Duration::from_millis(200));

        let reader = port
            .try_clone()
            .map_err(|e| io_err(format!("lora clone: {e}")))?;
        let (txch, rxch) = mpsc::unbounded_channel();
        let (respch, resprx) = std::sync::mpsc::channel();
        let profile = Arc::new(Mutex::new(
            hint.map(RadioKindHint::profile)
                .unwrap_or_else(NodeProfile::legacy_sx1262),
        ));
        let device_domain = lora_clock_domain(path);
        {
            let profile = Arc::clone(&profile);
            std::thread::spawn(move || reader_loop(reader, txch, respch, profile, device_domain));
        }
        let cmd = Arc::new(Mutex::new(CmdPort { port, resp: resprx }));

        // ── The keystone: ask the node what it is, BEFORE sending it anything else. ──
        // A node that does not implement CMD_GET_CAP answers nothing (old firmware) or
        // EVT_UNSUPPORTED (v2 firmware without the opcode); both land on the fallback.
        let cap = exec_on(&cmd, CMD_GET_CAP, &[], EVT_CAP, CAP_TIMEOUT, false).ok();
        let learned = resolve_profile(cap.as_deref(), hint);
        *profile.lock().unwrap() = learned;

        // ── The second question, and it is a different one. ──
        //
        // `EVT_CAP.stamp_kind` said where this node LATCHES a timestamp. This asks what the counter
        // it latches is DERIVED FROM — the axis that decides whether two nodes' stamps of one frame
        // can be differenced at all. A node that does not implement the opcode answers
        // `EVT_UNSUPPORTED` (terminal, so it costs nothing), acks it as an unknown command and
        // times out once at `CAP_TIMEOUT` (the Heltec), or says nothing. All three are UNKNOWN, and
        // unknown falls back to `assumed_clock_reference` — which withholds, except where the
        // node's OWN `EVT_CAP` (already in `learned`, hence the whole profile) is itself the
        // deciding wire fact. Order matters: a real `EVT_CLOCK_REF` always wins over the fallback.
        let clock_reference = exec_on(
            &cmd,
            CMD_GET_CLOCK_REF,
            &[],
            EVT_CLOCK_REF,
            CAP_TIMEOUT,
            false,
        )
        .ok()
        .as_deref()
        .and_then(parse_clock_ref)
        .unwrap_or_else(|| assumed_clock_reference(&learned));

        // Only now program the radio, and only with what this node implements.
        {
            let g = cmd.lock().unwrap();
            configure(&g.port, &mut params, &learned)?;
        }

        Ok(Self {
            cmd,
            rx: AsyncMutex::new(rxch),
            params: Arc::new(Mutex::new(params)),
            profile,
            lbt: AtomicBool::new(false),
            lbt_cfg: Mutex::new(LbtCfg::default()),
            device_domain,
            clock_reference,
        })
    }

    /// **What this node told us it can do** — the single source every knob, timeout and capability on
    /// this backend reads. `learned == false` means the node never answered `CMD_GET_CAP` and this is
    /// the host's written-down fallback.
    pub fn profile(&self) -> NodeProfile {
        *self.profile.lock().unwrap()
    }

    /// The clock domain this node's hardware RX stamps live in (per port).
    pub fn device_clock_domain(&self) -> ClockDomainId {
        self.device_domain
    }

    /// **What this node's stamp counter runs on** — from its own [`CMD_GET_CLOCK_REF`] answer where
    /// the firmware implements it, else the demote-only host fallback ([`assumed_clock_reference`]).
    ///
    /// Surfaced so a bring-up tool can show the reference beside the latch point, which is the pair
    /// that decides `FaceTimeProfile::can_common_view`. A `RateWitness::NodeReported` measurement
    /// means the node measured itself; anything else means this host did.
    pub fn clock_reference(&self) -> ClockReference {
        self.clock_reference
    }

    /// The effective per-frame payload cap: the smaller of the host's one-packet-per-frame budget
    /// ([`MAX_LORA_PAYLOAD`]) and what the node says it can carry end to end — see
    /// [`NodeProfile::frame_budget`], which is where that decision lives.
    pub fn max_payload(&self) -> usize {
        self.profile().frame_budget()
    }

    /// The radio parameters currently programmed (reflects runtime [`RadioKnobs`] changes).
    pub fn params(&self) -> LoraParams {
        *self.params.lock().unwrap()
    }

    /// #52: route `inject` through the node's atomic listen-before-talk (`CMD_TX_LBT`) — carrier sense
    /// + backoff before every key-up.
    pub fn set_lbt(&self, on: bool) {
        self.lbt.store(on, Ordering::Relaxed);
    }

    /// Whether listen-before-talk is currently armed.
    pub fn lbt(&self) -> bool {
        self.lbt.load(Ordering::Relaxed)
    }

    /// #52: tune the LBT contention window (ms), max backoff exponent, and max attempts at runtime —
    /// no reflash (Tier 2). Bigger `cw_ms` = better fairness (nodes separate more) at higher latency.
    /// The values are mirrored host-side so the transmit deadline covers the contention budget.
    pub fn set_lbt_cfg(
        &self,
        cw_ms: u16,
        max_backoff: u8,
        max_attempts: u8,
    ) -> Result<(), FaceError> {
        let p = [(cw_ms >> 8) as u8, cw_ms as u8, max_backoff, max_attempts];
        self.exec_idempotent(CMD_SET_LBT_CFG, &p, EVT_INFO)?;
        *self.lbt_cfg.lock().unwrap() = LbtCfg {
            cw_ms,
            max_backoff,
            max_attempts,
        };
        Ok(())
    }

    /// #52: tune CAD sensitivity (symbol-count code, detector peak/min) at runtime — no reflash.
    pub fn set_cad_cfg(&self, sym: u8, det_peak: u8, det_min: u8) -> Result<(), FaceError> {
        self.exec_idempotent(CMD_SET_CAD_CFG, &[sym, det_peak, det_min], EVT_INFO)?;
        Ok(())
    }

    /// **Tune the channel-busy sense** (`CMD_SET_SENSE_CFG`, 0x12) — an energy-detect threshold in dBm
    /// OR'd into the CAD decision, plus how many CAD samples to OR per sense.
    ///
    /// This opcode has existed in the firmware since #52 and had **no host constant at all**, so the
    /// only sense the host could ask for was the compiled-in default (CAD-only, one sample). The
    /// energy detector is what catches *non-LoRa* interference — the co-band HaLow case — which CAD
    /// structurally cannot see, because CAD looks for LoRa chirps.
    pub fn set_sense_cfg(&self, rssi_thresh_dbm: i16, cad_repeat: u8) -> Result<(), FaceError> {
        let p = [
            (rssi_thresh_dbm >> 8) as u8,
            rssi_thresh_dbm as u8,
            cad_repeat.max(1),
        ];
        self.exec_idempotent(CMD_SET_SENSE_CFG, &p, EVT_INFO)?;
        Ok(())
    }

    /// **Set the LoRa preamble length in symbols** (`CMD_SET_PREAMBLE`, 0x0C) — declared in this file
    /// since #52 and never once sent. It is a real reach/airtime dial: a longer preamble buys a
    /// receiver more chances to detect the frame (and is what a duty-cycled listener needs), at the
    /// cost of airtime on every transmission. Kept in [`LoraParams`] so the airtime-derived transmit
    /// timeout tracks it.
    pub fn set_preamble(&self, symbols: u16) -> Result<(), FaceError> {
        self.exec_idempotent(
            CMD_SET_PREAMBLE,
            &[(symbols >> 8) as u8, symbols as u8],
            EVT_INFO,
        )?;
        self.params.lock().unwrap().preamble = symbols;
        Ok(())
    }

    /// **The FLRC rate ladder**, in the LR2021's own code space (`CMD_SET_MOD` on a fixed-rate node).
    ///
    /// The portable [`RadioKnobs`] rate knobs deliberately refuse on a node with no spreading factor
    /// (see [`send_mod`](Self::send_mod)): the fleet's `[sf, bw, cr]` byte positions carry
    /// chip-specific meanings there, and composing them from a LoRa-shaped plan silently
    /// re-modulates the link. That leaves the LR2021's rate genuinely settable but unreachable
    /// through the typed API, so this is the reachable path — and it names the code space instead of
    /// pretending it is the LoRa one.
    ///
    /// `bitrate` is the chip's own `FlrcBitrate` code (`0 = 2.6 Mbit/s` … `7 = 260 kbit/s`, the
    /// firmware's default being 0) and `coding` its `FlrcCr` code (`0 = 1/2, 1 = 3/4, 2 = FEC off,
    /// 3 = 2/3` — counter-intuitive, and it is what the silicon uses). Both ends of a link must be
    /// moved together: a rate change is not negotiated on air.
    ///
    /// Refused on any node that does have a spreading factor, where these bytes would be read as one.
    pub fn set_flrc_rate(&self, bitrate: u8, coding: u8) -> Result<(), FaceError> {
        let prof = self.profile();
        if prof.has_spreading_factor() {
            return Err(unsupported(
                "this node's CMD_SET_MOD bytes are a LoRa [sf, bw, cr]; use the RadioKnobs rate \
                 methods instead"
                    .into(),
            ));
        }
        if !prof.supports(CMD_SET_MOD) {
            return Err(unsupported(
                "this node does not implement CMD_SET_MOD".into(),
            ));
        }
        // Byte 1 is the fleet's `bw` slot; the LR2021 ignores it (FLRC bandwidth follows the rung),
        // and it is sent as 0 to match what that node reports back in EVT_INFO.
        self.exec_idempotent(CMD_SET_MOD, &[bitrate, 0, coding], EVT_INFO)?;
        Ok(())
    }

    /// **Scan for a peer's spreading factor** (`CMD_SF_SCAN`, 0x0D) — also declared and never sent.
    /// Returns the SF detected on air, or `None` when the scan found nothing.
    ///
    /// This is the missing half of the SF reach dial: two nodes at different SFs are quasi-orthogonal
    /// and simply cannot hear each other, so an SF split is unrecoverable *in band* — a receiver-side
    /// detect is the only way back without a rendezvous SF.
    pub fn sf_scan(&self) -> Result<Option<u8>, FaceError> {
        let p = self.exec(CMD_SF_SCAN, &[], EVT_SF_DETECTED, INFO_TIMEOUT)?;
        Ok(p.first().copied().filter(|&sf| sf != 0))
    }

    /// **Set the LoRa sync word at runtime** (`CMD_SET_SYNC`, 0x05) — 0x12 private / 0x34 public in
    /// the SX127x single-byte convention. Previously reachable only via the one shot inside `open`,
    /// which made a sync word a boot-time property of the process rather than a live knob.
    pub fn set_sync_word(&self, sync: u8) -> Result<(), FaceError> {
        self.exec_idempotent(CMD_SET_SYNC, &[sync], EVT_INFO)?;
        self.params.lock().unwrap().sync = sync;
        Ok(())
    }

    /// **Read the channel-busy sense** (`CMD_SENSE`, 0x1B): `(activity, rssi_dbm)`.
    ///
    /// `activity` is a FREE-RUNNING, WRAPPING count of channel-busy observations — differences over a
    /// window are meaningful, absolute values are not. `rssi` is the instantaneous channel RSSI in
    /// dBm. Backs [`RadioKnobs::read_channel_activity`].
    pub fn sense(&self) -> Result<(u16, i16), FaceError> {
        let p = self.exec(CMD_SENSE, &[], EVT_SENSE, INFO_TIMEOUT)?;
        if p.len() < 4 {
            return Err(io_err(format!("EVT_SENSE short: {} bytes", p.len())));
        }
        Ok((
            u16::from_be_bytes([p[0], p[1]]),
            i16::from_be_bytes([p[2], p[3]]),
        ))
    }

    // ── v3: modulation, hopping, and the receive front end ──────────────────────────────────

    /// **Switch this node's modulation** (`CMD_SET_PHY`, 0x1D), returning the mode actually in
    /// effect. Backs [`RadioKnobs::set_phy`].
    ///
    /// ★ **The reply is a whole new `EVT_CAP` and this method REPLACES the stored [`NodeProfile`]
    /// with it — it never patches a field.** That is not tidiness, it is correctness: `max_payload`,
    /// the SF span, the rate model, `sched_gran_ns` and the band are all *per-PHY*. An LR2021 in
    /// FLRC carries 47 bytes and has no spreading factor; the same silicon in LoRa carries far more
    /// and spans SF7..SF12. A host that kept the old payload cap across a switch to FLRC would hand
    /// the face an MTU the frame cannot hold, which is the silent-corruption class this profile
    /// exists to stop.
    ///
    /// **Believe the return, not the request.** The node may run a mode other than the one asked
    /// for: it can refuse outright (`EVT_PHY_ERR`, surfaced here as an `Unsupported` error carrying
    /// the chip's own status byte), and it can answer a `CMD_SET_PHY` with a capability that names
    /// a different mode, which is a refusal it chose to express as a fact.
    ///
    /// ⚠ **Re-assert your channel, rate and power afterwards.** None of them survives a modulation
    /// change — the parameters are per-PHY too. This method clamps the host's *mirror* of them into
    /// the new profile's spans so `params()` cannot describe something the node is not doing, but a
    /// clamp is not a command: nothing has been re-sent to the radio.
    pub fn set_phy_mode(&self, mode: PhyMode) -> Result<PhyMode, FaceError> {
        self.require(CMD_SET_PHY)?;
        let advertised = self.profile().phy_modes();
        if !advertised.contains(mode) {
            return Err(unsupported(format!(
                "this node advertises {:?}, not {mode:?}",
                advertised.iter().collect::<Vec<_>>()
            )));
        }
        // Not `exec_idempotent`: a PHY switch re-programs the packet engine, so a blind re-send
        // after a lost reply would re-run it while the first one is still settling.
        let cap = exec_on(
            &self.cmd,
            CMD_SET_PHY,
            &[mode.code()],
            EVT_CAP,
            PHY_SWITCH_TIMEOUT,
            false,
        )?;
        let mut learned = NodeProfile::parse(&cap).ok_or_else(|| {
            io_err(format!(
                "CMD_SET_PHY: unparseable EVT_CAP ({} bytes)",
                cap.len()
            ))
        })?;
        if learned.phy_current != mode {
            // Either a refusal, or an unsolicited EVT_CAP that raced our request onto the channel
            // (see `recv_on`). Give the real reply one short window before concluding it was a
            // refusal; if nothing else arrives, the capability we hold is the truth either way.
            if let Some(p2) = recv_on(&self.cmd, EVT_CAP, CAP_RACE_WINDOW)
                && let Some(second) = NodeProfile::parse(&p2)
            {
                learned = second;
            }
        }
        self.install_profile(learned);
        Ok(learned.phy_current)
    }

    /// **Install an autonomous hop plan** (`CMD_SET_HOP`, 0x1E): `ctrl` arms or disarms the
    /// sequencer, `period` is the dwell in the units [`NodeProfile::hop_capability`] reports, and
    /// `freqs_hz` is the carrier list (at most [`HOP_LIST_MAX`]). Backs
    /// [`RadioKnobs::set_hop_plan`].
    ///
    /// This is **not** [`set_channel`](RadioKnobs::set_channel) in a loop and it is not what
    /// [`RadioCapability::retune_us`] prices. Those measure a host-commanded retune — 5.6 ms on the
    /// Heltec, 52.8 ms on the LR2021, 161 ms on the Waveshare — and bound hopping *between*
    /// packets. This hands the node a list its own sequencer walks, on the two parts that hop
    /// **inside** a packet, at a dwell no serial command could reach.
    ///
    /// Every carrier is checked against the node's declared span before anything goes on the wire:
    /// a hop list containing one frequency the node will not accept is a plan whose dwell pattern
    /// the two ends silently stop agreeing on.
    pub fn set_hop_plan_hz(
        &self,
        ctrl: HopControl,
        period: u16,
        freqs_hz: &[u32],
    ) -> Result<(), FaceError> {
        self.require(CMD_SET_HOP)?;
        if freqs_hz.len() > HOP_LIST_MAX {
            return Err(unsupported(format!(
                "hop list of {} exceeds the {HOP_LIST_MAX}-entry wire bound",
                freqs_hz.len()
            )));
        }
        if matches!(ctrl, HopControl::On) && freqs_hz.is_empty() {
            return Err(unsupported(
                "cannot arm a hop plan with an empty frequency list".into(),
            ));
        }
        let prof = self.profile();
        for &hz in freqs_hz {
            if prof.clamp_hz(hz) != hz {
                return Err(unsupported(format!(
                    "hop carrier {hz} Hz is outside this node's {}–{} Hz range",
                    prof.freq_min_hz, prof.freq_max_hz
                )));
            }
        }
        self.exec_idempotent(CMD_SET_HOP, &hop_payload(ctrl, period, freqs_hz), EVT_INFO)?;
        Ok(())
    }

    /// **Set the receive front end's gain posture** (`CMD_SET_RX_GAIN`, 0x1C). Backs
    /// [`RadioKnobs::set_rx_gain`].
    ///
    /// ★ All three firmwares have implemented this opcode and advertised bit 28 for as long as
    /// `EVT_CAP` has existed, and until this run **nothing in the host tree could send it** — an
    /// actuator with no caller, which is precisely the class of defect this pass is about.
    ///
    /// One boolean byte, fleet-wide: `0` = the part's own default (AGC on the LR2021, the
    /// power-saving LNA on the SX126x), `1` = its highest manual gain. The LR2021 firmware records
    /// why it will not expose its chip's 0..13 manual ladder through this byte — `1` would mean
    /// "boosted" on one node and the *lowest* manual step on another, an inversion this rig has
    /// already paid for once on a TX-power knob.
    pub fn set_rx_gain_mode(&self, gain: RxGain) -> Result<(), FaceError> {
        self.exec_idempotent(CMD_SET_RX_GAIN, &[rx_gain_byte(gain)], EVT_INFO)?;
        Ok(())
    }

    /// **Replace the stored profile wholesale** and re-clamp the host's mirror of the radio
    /// parameters into whatever the new PHY allows.
    ///
    /// Wholesale, never field-by-field: see [`set_phy_mode`](Self::set_phy_mode). The `params`
    /// clamp exists so `params()` cannot report a spreading factor or a carrier the node no longer
    /// has — it does **not** transmit anything, because the node has already re-programmed itself.
    fn install_profile(&self, learned: NodeProfile) {
        *self.profile.lock().unwrap() = learned;
        reconcile_params(&mut self.params.lock().unwrap(), &learned);
    }

    /// **Read the node's clock** (`CMD_READ_CLOCK`, 0x17) in its own ticks — the units are
    /// [`NodeProfile::stamp_hz`], the same as the `ts` on every `EVT_RX`.
    pub fn read_device_clock(&self) -> Result<u64, FaceError> {
        let p = self.exec(CMD_READ_CLOCK, &[], EVT_CLOCK, INFO_TIMEOUT)?;
        if p.len() < 8 {
            return Err(io_err(format!("EVT_CLOCK short: {} bytes", p.len())));
        }
        Ok(u64::from_be_bytes([
            p[0], p[1], p[2], p[3], p[4], p[5], p[6], p[7],
        ]))
    }

    /// One-shot carrier-sense (`CMD_CAD`): `true` when the channel is busy.
    pub fn cad(&self) -> Result<bool, FaceError> {
        let p = self.exec(CMD_CAD, &[], EVT_CAD, INFO_TIMEOUT)?;
        Ok(p.first().copied().unwrap_or(0) != 0)
    }

    /// Instantaneous channel RSSI in dBm (`CMD_GET_RSSI`).
    pub fn channel_rssi(&self) -> Result<i16, FaceError> {
        let p = self.exec(CMD_GET_RSSI, &[], EVT_RSSI, INFO_TIMEOUT)?;
        if p.len() < 2 {
            return Err(io_err(format!("EVT_RSSI short: {} bytes", p.len())));
        }
        Ok(i16::from_be_bytes([p[0], p[1]]))
    }

    /// #52: read the firmware's CSMA observability counters `(cad_busy, deferred)` from `EVT_INFO`.
    /// `cad_busy` climbing with `deferred` near zero means backoff is finding the channel and winning.
    pub fn csma_counters(&self) -> Result<(u16, u16), FaceError> {
        let p = self.exec(CMD_GET_INFO, &[], EVT_INFO, INFO_TIMEOUT)?;
        // EVT_INFO tail (#52): ... lost(2), cad_busy(2), defer(2) → the last 4 bytes.
        if p.len() >= 19 {
            let cad = u16::from_be_bytes([p[15], p[16]]);
            let def = u16::from_be_bytes([p[17], p[18]]);
            Ok((cad, def))
        } else {
            Ok((0, 0)) // pre-#52 firmware: no counters
        }
    }
}

impl LoraSerialBackend {
    // ---- #52 on-device NDN data plane (all runtime; no reflash) ----

    /// Install the RX name filter as a set of **prefixes** (a FIB): an Interest is delivered up only if
    /// one of `prefixes` is a name-prefix of it (NDN longest-prefix match in firmware); the rest are
    /// dropped at the antenna. So `["ndn/lora-cog/A"]` covers every `ndn/lora-cog/A/...` under it. Empty
    /// clears the filter (pass-all). Each prefix is hashed with the SAME [`name_hash`] the firmware's
    /// rolling per-component hash lands on at that boundary, so the keyspaces match (#44).
    ///
    /// ⚠ This is **not** the retired in-frame name filter; it is this bearer's own body-prefix mechanism on this
    /// type for why the two cannot be bridged.
    pub fn set_name_filter(&self, prefixes: &[&[u8]]) -> Result<(), FaceError> {
        self.exec_idempotent(CMD_SET_NAME_FILTER, &hash_payload(prefixes), EVT_INFO)?;
        Ok(())
    }

    /// Install the relay set as a set of **prefixes**: a frame is re-broadcast AND delivered
    /// (cooperative forwarding) if one of `prefixes` is a name-prefix of it. Empty clears it.
    pub fn set_relay(&self, prefixes: &[&[u8]]) -> Result<(), FaceError> {
        self.exec_idempotent(CMD_SET_RELAY, &hash_payload(prefixes), EVT_INFO)?;
        Ok(())
    }

    /// Toggle the data-centric features: Content-Store serve (answer Interests from the on-device
    /// cache), duplicate suppression, and name-keyed frequency hopping (`base_ch`, `span`).
    pub fn set_dataplane(
        &self,
        cs_serve: bool,
        dedup: bool,
        hop_on: bool,
        hop_base_ch: u8,
        hop_span: u8,
    ) -> Result<(), FaceError> {
        let p = [
            cs_serve as u8,
            dedup as u8,
            hop_on as u8,
            hop_base_ch,
            hop_span,
        ];
        self.exec_idempotent(CMD_DATAPLANE, &p, EVT_INFO)?;
        Ok(())
    }

    /// Toggle firmware EVT_LOG diagnostics (data-plane decision traces) at runtime.
    pub fn set_debug(&self, on: bool) -> Result<(), FaceError> {
        self.exec_idempotent(CMD_SET_DEBUG, &[on as u8], EVT_INFO)?;
        Ok(())
    }

    /// Clear the data-plane + CSMA observability counters (re-baseline before a test window).
    pub fn reset_ndn_stats(&self) -> Result<(), FaceError> {
        self.exec_idempotent(CMD_RESET_STATS, &[], EVT_INFO)?;
        Ok(())
    }

    /// Read the on-device data-plane counters — proof each offload path actually fired on air — plus
    /// the **PHY** counters in the v2 tail where the node sends them (see [`NdnStats`]). Length-driven:
    /// a 24-byte reply parses exactly as before with the tail `None`.
    pub fn ndn_stats(&self) -> Result<NdnStats, FaceError> {
        let p = self.exec(CMD_GET_STATS, &[], EVT_STATS, INFO_TIMEOUT)?;
        NdnStats::parse(&p).ok_or_else(|| io_err(format!("EVT_STATS short: {} bytes", p.len())))
    }

    /// Jump the dongle into the GD32 ROM UART bootloader so `stm32flash` can reflash over this SAME
    /// CH343/USB port — no ST-Link, no BOOT0 jumper, no replug (reflash in place on the OPi). This is
    /// fire-and-forget: the firmware branches away immediately, so there is no reply to await; the next
    /// thing on the wire is the ROM bootloader's autobaud. Follow with e.g.
    /// `stm32flash -w firmware.bin -v -g 0x08000000 /dev/ttyACM0`. Guarded by a 2-byte magic so a
    /// corrupt frame cannot trigger it.
    pub fn enter_bootloader(&self) -> Result<(), FaceError> {
        self.require(CMD_ENTER_BOOTLOADER)?;
        let cmd = self.cmd.lock().unwrap();
        send_cmd(&cmd.port, CMD_ENTER_BOOTLOADER, &[0xB0, 0x07])
            .map_err(|e| io_err(format!("lora enter_bootloader: {e}")))?;
        Ok(())
    }

    /// Toggle the firmware's on-air heartbeat beacon at runtime (off by default under host control).
    pub fn set_beacon(&self, on: bool) -> Result<(), FaceError> {
        self.exec_idempotent(CMD_SET_BEACON, &[on as u8], EVT_INFO)?;
        self.params.lock().unwrap().beacon = on;
        Ok(())
    }

    // ---- command plumbing ----

    /// Refuse a command this node does not implement, **before** it goes on the wire. This is the
    /// capability gate doing its job: `CMD_SET_FREQ` on the LR2021 is not a command that fails, it is
    /// a command that permanently breaks the transmit path.
    fn require(&self, cmd: u8) -> Result<(), FaceError> {
        if self.profile().supports(cmd) {
            Ok(())
        } else {
            Err(unsupported(format!(
                "this node does not implement 7E-A5 command {cmd:#04x}"
            )))
        }
    }

    /// Send an **idempotent** command, retrying if the node never answers.
    ///
    /// The firmware polls its FIFO-less USART from the main loop, so a byte that lands while it is
    /// busy (an SPI `poll_rx`, a previous handler, a transmission) is simply gone — commands are lost
    /// at a measurable rate. Re-sending a `SET_*` is harmless because it re-asserts a state rather
    /// than causing an event, so retry it. `CMD_TX` gets no retry: a lost *reply* is indistinguishable
    /// from a lost *command*, and re-sending would risk a duplicate frame on air. An
    /// `EVT_UNSUPPORTED` is a definite answer, so it is not retried either.
    fn exec_idempotent(&self, typ: u8, payload: &[u8], expect: u8) -> Result<Vec<u8>, FaceError> {
        self.require(typ)?;
        let mut last = None;
        for _ in 0..KNOB_ATTEMPTS {
            match exec_on(&self.cmd, typ, payload, expect, INFO_TIMEOUT, false) {
                Ok(v) => return Ok(v),
                Err(e) if is_unsupported(&e) => return Err(e),
                Err(e) => last = Some(e),
            }
        }
        Err(last.unwrap_or_else(|| io_err(format!("lora cmd {typ:#04x}: no attempt made"))))
    }

    /// One command, one reply, capability-gated. Blocking by construction — see [`CmdPort`].
    fn exec(
        &self,
        typ: u8,
        payload: &[u8],
        expect: u8,
        timeout: Duration,
    ) -> Result<Vec<u8>, FaceError> {
        self.require(typ)?;
        exec_on(&self.cmd, typ, payload, expect, timeout, false)
    }

    /// The **async** command path: run the blocking request/response off the runtime's worker.
    ///
    /// ★ This is why it exists. `inject` used to call the blocking `exec` directly from an
    /// `async fn`, so a transmission parked a tokio worker thread for the frame's whole airtime —
    /// seconds at high SF. On a multi-radio node that stalls every other future scheduled on that
    /// worker, including the Wi-Fi radios' own I/O; the LoRa radio is the slowest bearer in the rig
    /// and was the one holding the runtime hostage. Outside a runtime (a plain `block_on` harness)
    /// there is no pool to move to, so it runs inline rather than panicking.
    async fn exec_async(
        &self,
        typ: u8,
        payload: Vec<u8>,
        expect: u8,
        timeout: Duration,
        tx_wait: bool,
    ) -> Result<Vec<u8>, FaceError> {
        self.require(typ)?;
        let cmd = Arc::clone(&self.cmd);
        match tokio::runtime::Handle::try_current() {
            Ok(_) => tokio::task::spawn_blocking(move || {
                exec_on(&cmd, typ, &payload, expect, timeout, tx_wait)
            })
            .await
            .map_err(|e| io_err(format!("lora cmd {typ:#04x}: blocking task: {e}")))?,
            Err(_) => exec_on(&cmd, typ, &payload, expect, timeout, tx_wait),
        }
    }

    /// [`read_device_clock`](Self::read_device_clock) off the async worker — the clock read is a
    /// serial round trip, and blocking a runtime thread on one inside `inject_at_clock` would
    /// reintroduce, in miniature, the stall the transmit path was just fixed for.
    async fn read_device_clock_async(&self) -> Result<u64, FaceError> {
        let p = self
            .exec_async(CMD_READ_CLOCK, Vec::new(), EVT_CLOCK, INFO_TIMEOUT, false)
            .await?;
        if p.len() < 8 {
            return Err(io_err(format!("EVT_CLOCK short: {} bytes", p.len())));
        }
        Ok(u64::from_be_bytes([
            p[0], p[1], p[2], p[3], p[4], p[5], p[6], p[7],
        ]))
    }

    /// **Send `CMD_TX_AT_ABS` (0x1F): the frame plus the instant it is to leave**, in the node's own
    /// `stamp_hz` ticks. `lead_us` is what the caller believes the wait will be, and is used only to
    /// size the reply deadline (0 = unknown, in which case the node's own `EVT_TX_STARTED` re-bases
    /// it).
    ///
    /// The payload is `8 + frame`, four bytes more than `CMD_TX_AT`'s delay word. 7E-A5 frames a
    /// single-byte length, so the largest payload in this fleet still fits exactly:
    /// `MAX_LORA_PAYLOAD (247) + 8 = 255`. There is no headroom left, which is why the assertion is
    /// pinned by a test rather than left to be rediscovered by a truncated frame on air.
    async fn inject_at_abs(
        &self,
        frame: InjectFrame,
        target_tick: u64,
        lead_us: u64,
    ) -> Result<(), FaceError> {
        let cap = self.max_payload();
        if frame.payload.len() > cap {
            return Err(io_err(format!(
                "lora payload {} > {cap}",
                frame.payload.len()
            )));
        }
        let mut p = Vec::with_capacity(8 + frame.payload.len());
        p.extend_from_slice(&target_tick.to_be_bytes());
        p.extend_from_slice(&frame.payload);
        let timeout =
            self.tx_timeout(frame.payload.len()) + Duration::from_micros(lead_us.min(60_000_000));
        let reply = self
            .exec_async(CMD_TX_AT_ABS, p, EVT_TXDONE, timeout, true)
            .await?;
        match reply.first() {
            Some(1) => Ok(()),
            _ => Err(io_err("lora absolute scheduled TX reported failure".into())),
        }
    }

    /// **How long a transmission may honestly take**, from this node's live modulation and this
    /// frame's length — not a fixed constant.
    ///
    /// The old 3 s was a constant chosen for SF7-class frames; at SF12/125 kHz a 240-byte frame is
    /// ~8.5 s on air, so every long-reach transmission the stack ever asked for was abandoned before
    /// the radio finished. `TXDONE_MIN_TIMEOUT` keeps the old floor for short frames. A node with no
    /// spreading factor (FLRC) has no LoRa airtime to compute, and its frames are sub-millisecond, so
    /// the floor is the whole budget there.
    fn tx_timeout(&self, payload_len: usize) -> Duration {
        let p = self.params();
        let prof = self.profile();
        // ★ Keyed on the PHY IN EFFECT, not on the part. The same LR2021 that has no LoRa airtime
        // in FLRC has a full SF12 airtime budget the moment `set_phy(Lora)` succeeds, and reading
        // the part name instead would leave a 8.5 s frame on a 3 s timeout.
        let base = if prof.phy_current.has_spreading_factor() && prof.sf_max > 0 {
            Duration::from_millis(lora_airtime_ms(
                p.sf,
                p.bw_khz(),
                p.cr,
                payload_len,
                p.preamble,
            )) + AIRTIME_SLACK
        } else {
            AIRTIME_SLACK
        };
        base.max(TXDONE_MIN_TIMEOUT)
    }

    /// The transmit budget with listen-before-talk armed: airtime plus the firmware's own worst-case
    /// sense-and-backoff bound (`attempts × cw × 2^backoff`), rather than a magic 5 s.
    fn lbt_timeout(&self, payload_len: usize) -> Duration {
        self.tx_timeout(payload_len) + self.lbt_cfg.lock().unwrap().worst_case()
    }

    /// Push the current SF/BW/CR triple to the node (any of them changing needs the full set).
    /// Silently correct on a node with no `CMD_SET_MOD`: `require` refuses instead.
    ///
    /// ⚠ **Refuses outright on a node with no spreading factor** (`sf_min == 0`). `CMD_SET_MOD`'s
    /// three bytes are only portable between nodes that share the LoRa meaning of them. On the
    /// LR2021 the same positions are `[bitrate_rung, _, flrc_cr]` (`m6_bridge.rs`
    /// `bitrate_of_code`/`cr_of_code`), so this host's default `sf = 7` decodes there as
    /// `FlrcBitrate::Br0260` — a 10x rate drop from the firmware's `Br2600` default — and `cr = 1`
    /// decodes as `FlrcCr::Cr34` rather than LoRa 4/5. Sending the triple blind would silently
    /// re-modulate a link the host believes it left alone. The node's rate is reachable through the
    /// device's own code space, not through this one.
    fn send_mod(&self) -> Result<(), FaceError> {
        let prof = self.profile();
        if !prof.has_spreading_factor() {
            return Err(unsupported(
                "CMD_SET_MOD's [sf, bw, cr] bytes carry chip-specific meanings on a node with no \
                 spreading factor; refusing rather than re-modulating the link blind"
                    .into(),
            ));
        }
        let (sf, bw, cr) = {
            let p = self.params.lock().unwrap();
            (p.sf, bw_to_fw(prof.radio_kind, p.bw), p.cr)
        };
        self.exec_idempotent(CMD_SET_MOD, &[sf, bw, cr], EVT_INFO)?;
        Ok(())
    }
}

/// On-device NDN data-plane counters (from `EVT_STATS`). Each is a monotonic count since the last
/// [`reset_ndn_stats`](LoraSerialBackend::reset_ndn_stats) — nonzero proves the corresponding offload
/// path fired on air, not just that the code compiled.
///
/// ## Two layouts, one parser
///
/// `EVT_STATS` is **24 bytes on the Heltec and the LR2021 and 32 on the Waveshare**, whose firmware
/// appends its modem's own PHY counters. The tail is therefore `Option`, decided by the reply's
/// length and never by which node we think we are talking to; a 24-byte reply parses exactly as it
/// always did with every v2 field `None`, which is what keeps this host compatible with all three
/// firmwares at once.
///
/// The tail matters because the first five counters are all **post-decode**: a frame that failed CRC
/// never reaches the data plane and increments nothing, so on a 24-byte node "quiet channel" and
/// "channel we are failing to decode" produce identical numbers. [`chip_crc_err`](Self::chip_crc_err)
/// is the first thing on this bearer that tells them apart.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct NdnStats {
    /// Frames the data plane classified (parsed as NDNLPv2/NDN-TLV and named).
    pub rx: u32,
    /// Interests dropped by the name-hash filter (name not in the allow-set).
    pub filtered: u32,
    /// Frames dropped as duplicates (name already in the dedup ring).
    pub deduped: u32,
    /// Interests answered from the on-device Content Store (host never woke).
    pub served: u32,
    /// Frames re-broadcast by the relay set (cooperative forwarding).
    pub relayed: u32,
    /// CSMA: times the channel was sensed busy before a key-up.
    pub cad_busy: u16,
    /// CSMA: times a transmission was deferred after exhausting backoff.
    pub defer: u16,

    // ---- v2 tail (`EVT_STATS` bytes 24..32) — `None` when the node replied with 24 bytes ----
    /// **The modem's own** received-packet counter (SX126x `GetStats.nbPktReceived`). Free-running
    /// `u16`, wraps — difference two reads, never read an absolute.
    pub chip_rx: Option<u16>,
    /// The modem's CRC-failure counter (`GetStats.nbPktCrcError`) — a packet whose preamble and
    /// header decoded and whose payload did not. Collisions and marginal links live here, and
    /// nowhere else on this bearer: the firmware's `poll_rx` drops such a frame and returns nothing.
    pub chip_crc_err: Option<u16>,
    /// The modem's header-error counter (`GetStats.nbPktHeaderErr`) — demodulation began and the
    /// LoRa explicit header itself did not survive it.
    pub chip_hdr_err: Option<u16>,
    /// Frames whose true on-air length exceeded the node's RX buffer and were truncated. **Should be
    /// 0**; anything else means a peer is transmitting past the `max_payload` this node advertises,
    /// which is the silent-corruption failure the v2 capability protocol exists to stop.
    pub rx_trunc: Option<u16>,

    // ---- v3 tail (`EVT_STATS` bytes 32..44) — the EVIDENCE for `stamp_kind = 3` ----------------
    //
    // These were on the wire and reached no reader: the parser stopped at 32. A capability byte
    // whose supporting counters nothing decodes is a claim, not a measurement.
    /// Frames whose `ts` was the hardware capture.
    pub hw_stamped: Option<u16>,
    /// Frames that fell back to the node's software read — each announced individually by an
    /// `EVT_RX_STAMP` immediately before its `EVT_RX`.
    pub hw_stamp_sw: Option<u16>,
    /// Of those, the ones discarded for **mis-attribution**: more than one edge in the capture
    /// window, or a chip packet count that did not advance by exactly one across it (a frame that
    /// arrived while the IRQ line was already high, so the payload is a later frame's than the
    /// edge). Non-zero is the node refusing to guess, not an error.
    pub hw_stamp_ambig: Option<u16>,
    /// Timer overcaptures — **an edge was LOST**, which is worse than a stamp being imprecise.
    /// Free-running on the node; a `CMD_RESET_STATS` deliberately does not touch it.
    pub hw_stamp_over: Option<u16>,
    /// Worst capture-ISR entry latency, µs — how wrong a software stamp taken *in the ISR* would
    /// have been, i.e. what the hardware path bought.
    pub hw_stamp_lat_us: Option<u16>,
    /// `|micros64()/1000 − millis()|` on the node: its microsecond clock against its millisecond
    /// one. Both are divided from the same oscillator, so this measures neither — it catches a
    /// **lost timer overflow**, whose signature is a jump of 65 that never comes back.
    pub clock_skew_ms: Option<u16>,
}

impl NdnStats {
    /// Parse an `EVT_STATS` payload: 24 bytes (v1), 32 (v2) or 44 (v3). `None` if shorter than 24 —
    /// a truncated counter block is not a counter block, and zeroing the missing fields would invent
    /// traffic.
    ///
    /// A reply **longer** than the newest layout is accepted and its excess ignored, so a future
    /// firmware that appends another counter does not break this host; a length between two
    /// generations keeps the fields it covers and drops the partial tail rather than reading half a
    /// counter. Every generation appends, which is what makes that possible.
    pub fn parse(p: &[u8]) -> Option<Self> {
        if p.len() < 24 {
            return None;
        }
        let u32be = |o: usize| u32::from_be_bytes([p[o], p[o + 1], p[o + 2], p[o + 3]]);
        let u16be = |o: usize| u16::from_be_bytes([p[o], p[o + 1]]);
        let v2 = p.len() >= 32;
        let v3 = p.len() >= 44;
        Some(Self {
            rx: u32be(0),
            filtered: u32be(4),
            deduped: u32be(8),
            served: u32be(12),
            relayed: u32be(16),
            cad_busy: u16be(20),
            defer: u16be(22),
            chip_rx: v2.then(|| u16be(24)),
            chip_crc_err: v2.then(|| u16be(26)),
            chip_hdr_err: v2.then(|| u16be(28)),
            rx_trunc: v2.then(|| u16be(30)),
            hw_stamped: v3.then(|| u16be(32)),
            hw_stamp_sw: v3.then(|| u16be(34)),
            hw_stamp_ambig: v3.then(|| u16be(36)),
            hw_stamp_over: v3.then(|| u16be(38)),
            hw_stamp_lat_us: v3.then(|| u16be(40)),
            clock_skew_ms: v3.then(|| u16be(42)),
        })
    }

    /// **The `(ok, err)` PPDU pair [`RadioKnobs::read_ofdm_counters`] is defined as**, or `None` from
    /// a node that reports no PHY counters.
    ///
    /// `err` is `nbPktCrcError + nbPktHeaderErr`: both are receptions the PHY *began to demodulate
    /// and failed*, which is precisely the HAL's definition and precisely the collision / marginal-
    /// decode signature. They are summed because the HAL pair has one error slot and both halves
    /// answer the same question; the split stays visible on [`chip_crc_err`](Self::chip_crc_err) and
    /// [`chip_hdr_err`](Self::chip_hdr_err) for anyone who needs it.
    ///
    /// ⚠ **One thing here is not measured and must not be presented as if it were.** The SX126x
    /// datasheet (§13.5.5) does not say whether `nbPktReceived` counts *only* good packets or *all*
    /// receptions including the errored ones, and nothing in this rig has established it. Both are
    /// the chip's own numbers and neither is invented, but if `nbPktReceived` is the total then a
    /// consumer computing `err / (ok + err)` double-counts the denominator and its loss figure is a
    /// **lower bound** — never an over-claim. That is one transmit experiment away from being
    /// settled (send a known count of deliberately-corrupted frames and read both counters); until
    /// it is, this doc is the disclosure rather than a silent assumption.
    pub fn phy_counters(&self) -> Option<(u16, u16)> {
        let ok = self.chip_rx?;
        let err = self
            .chip_crc_err?
            .saturating_add(self.chip_hdr_err.unwrap_or(0));
        Some((ok, err))
    }
}

/// FNV-1a/64 over a name's bytes — the keyspace **this bearer's firmware** uses for its name filter,
/// relay set and Content Store. This MUST stay byte-for-byte identical to `ndn_embedded::pit::fnv1a64`
/// (which the firmware uses), or the host and node would disagree on which names an entry covers.
///
/// ⚠ **This is not the project's Tier-0 keyspace.** The MAC-layer prefix-set filter hashes keyed
/// SipHash-2-4 (this bearer's own keyspace, distinct from the retired in-frame filter).
pub fn name_hash(name: &[u8]) -> u64 {
    const OFFSET: u64 = 0xcbf2_9ce4_8422_2325;
    const PRIME: u64 = 0x0000_0100_0000_01b3;
    let mut hash = OFFSET;
    for &b in name {
        hash ^= b as u64;
        hash = hash.wrapping_mul(PRIME);
    }
    hash
}

/// Frame a `CMD_SET_HOP` payload: `[hop_ctrl][hop_period u16 BE][n][freq_hz u32 BE]*n`.
///
/// A free function so the wire layout can be pinned by a test without a serial port — the layout is
/// a contract with three firmware repos, and a host that only ever agrees with itself is how a
/// silently-truncated hop list becomes a link whose two ends disagree about the dwell pattern.
fn hop_payload(ctrl: HopControl, period: u16, freqs_hz: &[u32]) -> Vec<u8> {
    let mut p = Vec::with_capacity(4 + 4 * freqs_hz.len());
    p.push(ctrl as u8);
    p.extend_from_slice(&period.to_be_bytes());
    p.push(freqs_hz.len() as u8);
    for &hz in freqs_hz {
        p.extend_from_slice(&hz.to_be_bytes());
    }
    p
}

/// The one `CMD_SET_RX_GAIN` byte, fleet-wide: `0` = the part's own default (AGC / power-saving
/// LNA), `1` = its highest manual gain. There is no third value, on any of the three firmwares.
///
/// ★ [`RxGain::Reduced`] therefore maps to `0`, the default, and **not** to some invented
/// lower-than-default setting. These sub-GHz parts expose "default or boosted" and nothing below
/// default: the SX126x has two LNA register values, the SX127x a 3-bit `RegLna` where the useful
/// span is upward, and the LR20xx an AGC that already backs itself off. Returning the default is
/// the closest honest position — it is the least sensitive the part will go — and a caller wanting
/// real desensitisation for spatial reuse must use a bearer that has it (the a81a's IGI, or its
/// dBm `set_edcca_threshold_dbm`). Silently returning `1` here would have made "hear less" mean
/// "hear more" — the inverted-knob class of failure this fleet has already shipped once, on the
/// ESP32-C5's TX power.
fn rx_gain_byte(gain: RxGain) -> u8 {
    match gain {
        RxGain::Auto | RxGain::Reduced => 0,
        RxGain::Boosted => 1,
    }
}

/// **Re-clamp the host's mirror of the radio parameters into a freshly-installed profile.**
///
/// Called after a `CMD_SET_PHY` replaces the whole [`NodeProfile`]: SF, carrier and power spans are
/// all per-PHY, so a mirror carried across a modulation change can describe something the node is
/// not doing. It clamps and does **not** transmit — the node has already re-programmed itself, and
/// a caller that wants specific parameters in the new mode must re-assert them.
fn reconcile_params(p: &mut LoraParams, prof: &NodeProfile) {
    if let Some(sf) = prof.clamp_sf(p.sf) {
        p.sf = sf;
    }
    let hz = prof.clamp_hz(channel_to_hz(p.tx_ch));
    p.tx_ch = hz_to_channel(hz);
    p.rx_ch = p.tx_ch;
    p.pwr = prof.clamp_dbm(p.pwr.min(i8::MAX as u8) as i8).max(0) as u8;
}

/// Pack a list of names into a `[u64 BE hash]*` payload for CMD_SET_NAME_FILTER / CMD_SET_RELAY.
fn hash_payload(names: &[&[u8]]) -> Vec<u8> {
    let mut p = Vec::with_capacity(names.len() * 8);
    for n in names {
        p.extend_from_slice(&name_hash(n).to_be_bytes());
    }
    p
}

/// Frame one command/event as `7E A5 | type | len | payload | xor-crc` and write it in one call.
fn send_cmd(port: &SerialFd, typ: u8, payload: &[u8]) -> std::io::Result<()> {
    let mut f = Vec::with_capacity(5 + payload.len());
    f.push(SYNC0);
    f.push(SYNC1);
    f.push(typ);
    f.push(payload.len() as u8);
    let mut crc = typ ^ (payload.len() as u8);
    for &b in payload {
        f.push(b);
        crc ^= b;
    }
    f.push(crc);
    port.write_all(&f)?;
    port.flush()
}

/// Program frequency, modulation, power and sync word at open — **only what `prof` says this node
/// implements, clamped into the ranges it declares**, and `params` is corrected in place so the
/// host's idea of the radio matches what was actually sent.
///
/// The unconditional version of this function is what made one backend dangerous across the fleet: it
/// sent `SET_FREQ` to every node at every open, and on the LR2021 that command was MEASURED to
/// permanently break the transmit path. It also sent `SET_MOD` to a radio with no LoRa modulation and
/// `SET_SYNC` to firmware that acks unknown commands with `EVT_INFO` — a knob that reports success and
/// actuates nothing.
///
/// Still fire-and-forget with a settle rather than a reply wait: each handler runs to completion
/// before the FIFO-less USART is drained again, `SET_FREQ` runs a full image calibration, and blasting
/// these back-to-back overruns the port and silently corrupts whatever follows.
fn configure(
    port: &SerialFd,
    params: &mut LoraParams,
    prof: &NodeProfile,
) -> Result<(), FaceError> {
    let mkerr = |e: std::io::Error| io_err(format!("lora configure: {e}"));
    let settle = || std::thread::sleep(Duration::from_millis(150));

    // Beacon state first: a host silencing it must not be beaten by a stray beacon that would fire
    // during the slower radio reconfiguration below.
    if prof.supports(CMD_SET_BEACON) {
        send_cmd(port, CMD_SET_BEACON, &[params.beacon as u8]).map_err(mkerr)?;
        settle();
    } else {
        params.beacon = false; // no beacon engine on this node; do not claim one is running
    }

    if prof.supports(CMD_SET_FREQ) {
        let hz = prof.clamp_hz(channel_to_hz(params.tx_ch));
        params.tx_ch = hz_to_channel(hz);
        params.rx_ch = params.tx_ch;
        send_cmd(port, CMD_SET_FREQ, &hz.to_be_bytes()).map_err(mkerr)?;
        settle();
    } else {
        // The node cannot retune. Report the carrier it is fixed on, not the one we wanted.
        if prof.freq_min_hz != 0 {
            params.tx_ch = hz_to_channel(prof.freq_min_hz);
            params.rx_ch = params.tx_ch;
        }
    }

    // Only on a node whose `sf` byte really is a spreading factor. On an LR2021 the same three
    // positions are `[flrc_bitrate_rung, _, flrc_cr]`, so pushing this host's `sf = 7` default there
    // would decode as `Br0260` and drop a VERIFIED-LIVE 2.6 Mbit/s link by 10x at open() time,
    // reporting success. See `LoraSerialBackend::send_mod`.
    if prof.supports(CMD_SET_MOD) && prof.has_spreading_factor() {
        if let Some(sf) = prof.clamp_sf(params.sf) {
            params.sf = sf;
        }
        send_cmd(
            port,
            CMD_SET_MOD,
            &[params.sf, bw_to_fw(prof.radio_kind, params.bw), params.cr],
        )
        .map_err(mkerr)?;
        settle();
    }

    if prof.supports(CMD_SET_PWR) {
        let dbm = prof.clamp_dbm(params.pwr.min(i8::MAX as u8) as i8);
        params.pwr = dbm.max(0) as u8;
        send_cmd(port, CMD_SET_PWR, &[dbm as u8]).map_err(mkerr)?;
        settle();
    }

    if prof.supports(CMD_SET_SYNC) {
        send_cmd(port, CMD_SET_SYNC, &[params.sync]).map_err(mkerr)?;
        settle();
    }

    if prof.supports(CMD_SET_PREAMBLE) {
        send_cmd(
            port,
            CMD_SET_PREAMBLE,
            &[(params.preamble >> 8) as u8, params.preamble as u8],
        )
        .map_err(mkerr)?;
        settle();
    }
    Ok(())
}

/// **Turn an `EVT_RX` `ts` field into a [`LinkStamp`], using the node's declared units.**
///
/// This is the conversion the old code did not have. It read the field as milliseconds because the
/// Waveshare firmware happens to fill it that way, and then discarded it and stamped `HostRecv`
/// anyway. On the LR2021 the same four bytes are 16 MHz hardware ticks — reading them as µs is 16×
/// wrong and as ms is 16 000× wrong — so the units come from [`NodeProfile::stamp_hz`], never from
/// this file's memory of one board.
///
/// Only a **hardware** free-running stamp is promoted to the device clock domain. A
/// `SoftwareCounter` is a firmware-scheduler tick: real, but its jitter is the firmware's main loop,
/// so publishing it as a link clock would let a common-view computation difference two nodes'
/// scheduler latencies and call the result a clock offset.
///
/// The latch point is [`LatchPoint::RadioCapture`] — a radio peripheral's own hardware edge capture,
/// which is exactly what a `stamp_kind = 3` node reports and a **different physical event** from
/// `MacDone`, not a relabelling of it: no host MAC pipeline is in the path and the error budget is
/// set by the capture counter's tick.
///
/// This closes a 16× loss. `LinkStamp::new` clamps `precision_ns` up to the latch point's floor, and
/// `MacDone`'s floor is 1 µs — a figure calibrated for 802.11's microsecond TSF register. Under it,
/// the LR2021's MEASURED 62.5 ns capture was published here as 1 000 ns while
/// [`RadioTime::time_sources`] advertised the true 63 ns tick, so the two disagreed and every offset
/// estimate built on a per-frame stamp silently inherited the wider figure. `RadioCapture`'s floor
/// is 10 ns — one tick of a 100 MHz capture timer — which admits the measured 63 ns and still clamps
/// an over-claim. Mislabelling this as `PhyPreamble` to slip under the clamp would have been gaming
/// the guard rather than fixing it.
/// ★ `frame_kind` is the node's verdict about **this one frame** ([`EVT_RX_STAMP`]), or `None` when
/// it said nothing — which means the frame agrees with the capability. Both must be
/// `HardwareFreeRun` for the stamp to reach the device clock domain: the capability alone is a claim
/// individual frames are allowed to violate, and a per-frame note claiming *better* than the
/// capability is a contradiction, so the pair is combined by taking the worse of the two rather than
/// by letting either override.
///
/// Keying on the node-level kind alone was the defect: every frame from a self-test-passing dongle
/// was published as a 1 µs `RadioCapture`, including the ones whose firmware had already computed,
/// counted and announced that their `ts` was a poll-loop software read — wrong by up to a whole poll
/// gap (~22 ms), which is four orders of magnitude past what the latch point claims.
fn rx_stamp(
    prof: &NodeProfile,
    domain: ClockDomainId,
    ts_raw: u32,
    frame_kind: Option<StampKind>,
) -> LinkStamp {
    let degraded = matches!(frame_kind, Some(k) if k != StampKind::HardwareFreeRun);
    match prof.stamp_kind {
        StampKind::HardwareFreeRun if prof.stamp_hz > 0 && !degraded => LinkStamp::new(
            ts_raw as u64,
            domain,
            prof.tick_ns().unwrap_or(1),
            LatchPoint::RadioCapture,
        ),
        _ => host_stamp(),
    }
}

/// Background reader: accumulate serial bytes, parse protocol frames, and hand each received frame
/// up as a `CapturedFrame` stamped per the node's profile and carrying its RSSI **and SNR**. Non-RX
/// events go to whoever is blocked in `exec_on`; a crc miss resyncs on the next sync word.
fn reader_loop(
    port: SerialFd,
    tx: mpsc::UnboundedSender<CapturedFrame>,
    resp: std::sync::mpsc::Sender<(u8, Vec<u8>)>,
    profile: Arc<Mutex<NodeProfile>>,
    domain: ClockDomainId,
) {
    let debug = std::env::var_os("LORA_DEBUG").is_some();
    let mut acc: Vec<u8> = Vec::new();
    let mut tmp = [0u8; 512];
    // The per-frame stamp verdict from an `EVT_RX_STAMP`, waiting for the `EVT_RX` it qualifies.
    // Lives here rather than in `handle_event` because this loop is the single decoder for one
    // port: the note and its frame are adjacent on an in-order link, and this is the one place that
    // ordering is observable.
    let mut pending_stamp: Option<StampKind> = None;
    loop {
        match port.read(&mut tmp) {
            Ok(n) if n > 0 => {
                acc.extend_from_slice(&tmp[..n]);
                loop {
                    match next_event(&acc) {
                        EvParse::Event {
                            typ,
                            payload,
                            consumed,
                        } => {
                            handle_event(
                                typ,
                                &payload,
                                &tx,
                                &resp,
                                &profile,
                                domain,
                                debug,
                                &mut pending_stamp,
                            );
                            acc.drain(..consumed);
                            if tx.is_closed() {
                                return;
                            }
                        }
                        EvParse::Drop { consumed } => {
                            acc.drain(..consumed);
                        }
                        EvParse::Need => break,
                    }
                }
                if acc.len() > 8192 {
                    acc.clear(); // bound the buffer mid-desync
                }
            }
            Ok(_) => {}
            Err(ref e) if e.kind() == std::io::ErrorKind::TimedOut => {}
            Err(_) => return,
        }
    }
}

/// Dispatch one decoded event: an `EVT_RX` becomes a `CapturedFrame`; others are replies.
#[allow(clippy::too_many_arguments)]
fn handle_event(
    typ: u8,
    payload: &[u8],
    tx: &mpsc::UnboundedSender<CapturedFrame>,
    resp: &std::sync::mpsc::Sender<(u8, Vec<u8>)>,
    profile: &Mutex<NodeProfile>,
    domain: ClockDomainId,
    debug: bool,
    pending_stamp: &mut Option<StampKind>,
) {
    // ★ **The per-frame stamp verdict, held for the frame it qualifies.** Unsolicited chatter like
    // `EVT_LOG`: it must never satisfy a command wait, and it is not a reply, so it does not reach
    // the response channel at all. The binding to its frame is ordering — the node writes it
    // immediately before that `EVT_RX` with nothing in between — so it is consumed by the next
    // `EVT_RX` and dropped by anything else, which is the conservative direction: a note whose frame
    // never arrived would otherwise degrade an unrelated later frame.
    if typ == EVT_RX_STAMP {
        let kind = payload.first().copied().map(StampKind::from_code);
        if debug {
            eprintln!(
                "lora RX_STAMP frame_kind={:?} reason={} (this frame's ts is NOT the advertised \
                 hardware capture)",
                kind,
                payload.get(1).copied().unwrap_or(0),
            );
        }
        *pending_stamp = kind;
        return;
    }
    // Everything that is not a received frame is a reply to a command we sent; route it to the
    // caller blocked in `exec_on` so the next command only goes out once this one is serviced.
    if typ != EVT_RX {
        // The note's frame did not follow it. Drop it rather than let it qualify a later one.
        *pending_stamp = None;
        if debug {
            match typ {
                EVT_TXDONE => {
                    eprintln!("lora TXDONE ok={}", payload.first().copied().unwrap_or(0))
                }
                EVT_INFO => eprintln!("lora INFO {}", hex(payload)),
                EVT_LOG => eprintln!("lora LOG: {}", String::from_utf8_lossy(payload)),
                EVT_CAP => eprintln!("lora CAP {}", hex(payload)),
                // Named rather than left to the `other` arm: this is the node reporting that the
                // silicon refused a mode it advertises, and on the LR-FHSS RX-arm path it is the
                // whole measurement — an anonymous "EVT 0x8d" is exactly the line an operator
                // scrolls past.
                EVT_PHY_ERR => eprintln!(
                    "lora PHY_ERR phy={:#04x} chip_status={:#04x} (chip_mode={}, cmd_status={})",
                    payload.first().copied().unwrap_or(0),
                    payload.get(1).copied().unwrap_or(0),
                    payload.get(1).copied().unwrap_or(0) & 0x0F,
                    payload.get(1).copied().unwrap_or(0) >> 4,
                ),
                EVT_UNSUPPORTED => {
                    let reason = payload.get(1).copied().unwrap_or(0);
                    eprintln!(
                        "lora UNSUPPORTED cmd={:#04x} reason={reason} ({})",
                        payload.first().copied().unwrap_or(0),
                        unsup::name(reason)
                    )
                }
                other => eprintln!("lora EVT {other:#04x} {}", hex(payload)),
            }
        }
        // ★ **An `EVT_CAP` is a fact about the node whenever it arrives, solicited or not.**
        //
        // The Waveshare re-publishes its capability when its self-measured scheduling granularity
        // moves materially — nobody asked, and the old profile is stale from that instant. So the
        // stored `NodeProfile` is updated HERE, in the one thread that decodes the link, which
        // makes the reader the single writer and removes the update from every command path.
        //
        // **This cannot race a command's reply-matching**, by three separate properties:
        //
        // * the update happens *before* the event is offered to `resp`, so a caller that is woken
        //   by this very CAP already sees the profile it describes;
        // * `exec_on` drains the response channel before it sends, so a CAP that arrived while no
        //   command was in flight can never be mistaken for the next command's reply;
        // * a CAP that lands mid-command while some *other* reply is awaited hits `Ok(_) =>
        //   continue` and is skipped, having already done its work here.
        //
        // The one case left is a caller waiting for `EVT_CAP` itself — only `CMD_GET_CAP` and
        // `CMD_SET_PHY` — which an unsolicited push could satisfy with a true but off-topic answer.
        // `set_phy_mode` handles that explicitly with `recv_on`; `open_inner`'s `CMD_GET_CAP` does
        // not care, because either capability is the node describing itself right now.
        //
        // A payload that does not parse is dropped rather than allowed to blank a good profile: a
        // truncated capability is not a capability.
        if typ == EVT_CAP
            && let Some(fresh) = NodeProfile::parse(payload)
        {
            *profile.lock().unwrap() = fresh;
            if debug {
                eprintln!(
                    "lora CAP applied: phy {:?}, max_payload {}, sched_gran {} ns",
                    fresh.phy_current, fresh.max_payload, fresh.sched_gran_ns
                );
            }
        }
        // A LOG line is unsolicited chatter, not a reply — never let it satisfy a wait.
        if typ != EVT_LOG {
            let _ = resp.send((typ, payload.to_vec()));
        }
        return;
    }
    // EVT_RX = [rssi i16 BE, snr i16 BE, ts u32 BE, frame bytes] — payload starts at offset 8.
    if payload.len() >= 8 {
        let rssi = i16::from_be_bytes([payload[0], payload[1]]);
        let snr = i16::from_be_bytes([payload[2], payload[3]]);
        let ts = u32::from_be_bytes([payload[4], payload[5], payload[6], payload[7]]);
        let ndn = &payload[8..];
        let prof = *profile.lock().unwrap();
        // Consumed here, exactly once: the note qualifies this frame and no other.
        let frame_kind = pending_stamp.take();
        if debug {
            eprintln!(
                "lora RX [{rssi} dBm, SNR {snr} dB, ts {ts} @{} Hz, kind {:?}] {} bytes",
                prof.stamp_hz,
                frame_kind.unwrap_or(prof.stamp_kind),
                ndn.len()
            );
        }
        let cap = CapturedFrame {
            payload: ndn.to_vec().into(),
            addr: None,
            group: None,
            addr3: None,
            extra: None, // LoRa has no 802.11 wide-profile fields
            htc: None,
            rssi_dbm: Some(rssi.clamp(i8::MIN as i16, i8::MAX as i16) as i8),
            mcs_index: None,
            stamp: Some(rx_stamp(&prof, domain, ts, frame_kind)),
            // ★ Per-frame SNR, which this backend parsed and then threw away. On LoRa it is THE
            // signal cognition needs: the chip demodulates well below the noise floor (down to
            // ~-20 dB SNR at SF12), so RSSI alone says nothing about whether a rate will decode —
            // a -120 dBm frame at +5 dB SNR is healthy and a -100 dBm frame at -15 dB is not.
            // `PhyMetrics` already had the field; nothing had to change to carry it.
            phy: Some(PhyMetrics {
                snr_db: Some(snr.clamp(i8::MIN as i16, i8::MAX as i16) as i8),
                evm_db: None,
                cfo_hz: None,
            }),
        };
        let _ = tx.send(cap);
    }
}

fn hex(b: &[u8]) -> String {
    b.iter().map(|x| format!("{x:02x}")).collect()
}

/// Outcome of trying to parse one event from the front of the accumulator.
enum EvParse {
    Event {
        typ: u8,
        payload: Vec<u8>,
        consumed: usize,
    },
    Drop {
        consumed: usize,
    },
    Need,
}

/// Parse one `7E A5 | type | len | payload | crc` frame from the front of `buf`.
fn next_event(buf: &[u8]) -> EvParse {
    let Some(start) = buf.windows(2).position(|w| w == [SYNC0, SYNC1]) else {
        // No sync word; keep only a possible trailing lone SYNC0 for the next read.
        let keep = if buf.last() == Some(&SYNC0) { 1 } else { 0 };
        return if buf.len() > keep {
            EvParse::Drop {
                consumed: buf.len() - keep,
            }
        } else {
            EvParse::Need
        };
    };
    if start > 0 {
        return EvParse::Drop { consumed: start }; // discard leading garbage
    }
    if buf.len() < 4 {
        return EvParse::Need; // sync(2) + type + len
    }
    let typ = buf[2];
    let len = buf[3] as usize;
    let total = 4 + len + 1; // + crc
    if buf.len() < total {
        return EvParse::Need;
    }
    let payload = &buf[4..4 + len];
    let mut crc = typ ^ (len as u8);
    for &b in payload {
        crc ^= b;
    }
    if crc != buf[4 + len] {
        return EvParse::Drop { consumed: 2 }; // crc miss — skip past this sync word and resync
    }
    EvParse::Event {
        typ,
        payload: payload.to_vec(),
        consumed: total,
    }
}

fn io_err(msg: String) -> FaceError {
    FaceError::Io(std::io::Error::other(msg))
}

/// The honest refusal: this node does not have that knob. Distinguished from an I/O failure so a
/// caller can fall back rather than retry.
fn unsupported(msg: String) -> FaceError {
    FaceError::Io(std::io::Error::new(std::io::ErrorKind::Unsupported, msg))
}

fn is_unsupported(e: &FaceError) -> bool {
    matches!(e, FaceError::Io(io) if io.kind() == std::io::ErrorKind::Unsupported)
}

#[async_trait]
impl FrameIo for LoraSerialBackend {
    /// This radio's own capability, so a face built from the bare `dyn FrameIo` does not have to
    /// invent one. Delegates to this type's [`RadioProfile`] — the single source of truth.
    fn radio_capability(&self) -> Option<ndn_radio_hal::RadioCapability> {
        Some(<Self as ndn_radio_hal::RadioProfile>::capability(self))
    }
    async fn inject(&self, frame_in: InjectFrame) -> Result<(), FaceError> {
        // The payload is the NDN packet itself; the firmware frames it. `dst`/`src`/`tx` carry no
        // link addressing on this bearer, so they are advisory only.
        let cap = self.max_payload();
        if frame_in.payload.len() > cap {
            return Err(io_err(format!(
                "lora payload {} > {cap} (one frame; node cap {})",
                frame_in.payload.len(),
                self.profile().max_payload
            )));
        }
        // #52: with LBT on, the firmware runs an atomic CAD → backoff → key-up and replies
        // [sent, attempts]; sent=0 means the channel stayed busy for all attempts (DEFERRED), which we
        // surface as a TX failure so the caller re-expresses (same as a lost frame).
        let (typ, timeout) = if self.lbt.load(Ordering::Relaxed) {
            (CMD_TX_LBT, self.lbt_timeout(frame_in.payload.len()))
        } else {
            (CMD_TX, self.tx_timeout(frame_in.payload.len()))
        };
        let reply = self
            .exec_async(typ, frame_in.payload.to_vec(), EVT_TXDONE, timeout, true)
            .await?;
        match reply.first() {
            Some(1) => Ok(()),
            _ => Err(io_err("lora TX reported failure/deferred".into())),
        }
    }

    async fn recv_frame(&self) -> Result<CapturedFrame, FaceError> {
        let mut rx = self.rx.lock().await;
        rx.recv().await.ok_or(FaceError::Closed)
    }

    /// **Does this node place TX in time?** Read from the profile: it needs both a declared
    /// `sched_gran_ns` and the `CMD_TX_AT` opcode that actuates it. Moves with
    /// [`inject_after`](Self::inject_after) by construction, because both call
    /// [`NodeProfile::schedules_tx`] — the two cannot drift apart, which is the trap this method's
    /// HAL documentation warns about.
    fn schedules_tx(&self) -> bool {
        self.profile().schedules_tx()
    }

    /// Place the frame on air `delay_us` from now, on the **node's own** timebase (`CMD_TX_AT`), so no
    /// host↔device clock reconcile is needed. Falls through to [`inject`](Self::inject) when this node
    /// has no scheduled-TX engine — the caller's software gate has already waited in that case.
    async fn inject_after(&self, frame: InjectFrame, delay_us: u64) -> Result<(), FaceError> {
        // One predicate, shared with `schedules_tx()` and `tx_discipline()`: what the backend claims
        // and what it does are the same expression, so they cannot drift.
        let prof = self.profile();
        if !prof.schedules_after(delay_us) {
            return self.inject(frame).await;
        }
        let cap = self.max_payload();
        if frame.payload.len() > cap {
            return Err(io_err(format!(
                "lora payload {} > {cap}",
                frame.payload.len()
            )));
        }
        // A node that names instants but has no relative opcode: turn "in `delay_us`" into an
        // instant on its own counter and use the absolute seam. Costs the clock round trip the
        // relative opcode does not need, which is why the relative path stays preferred here — a
        // delay counted by the firmware from its own "now" is exactly what was asked for.
        if !prof.supports(CMD_TX_AT) {
            let now = self.read_device_clock_async().await?;
            let ticks = (delay_us as u128 * prof.stamp_hz as u128 / 1_000_000u128) as u64;
            return self
                .inject_at_abs(frame, now.wrapping_add(ticks), delay_us)
                .await;
        }
        let delay = delay_us.min(u32::MAX as u64) as u32;
        let mut p = Vec::with_capacity(4 + frame.payload.len());
        p.extend_from_slice(&delay.to_be_bytes());
        p.extend_from_slice(&frame.payload);
        let timeout =
            self.tx_timeout(frame.payload.len()) + Duration::from_micros(delay_us.min(60_000_000));
        let reply = self
            .exec_async(CMD_TX_AT, p, EVT_TXDONE, timeout, true)
            .await?;
        match reply.first() {
            Some(1) => Ok(()),
            _ => Err(io_err("lora scheduled TX reported failure".into())),
        }
    }

    /// **Place the frame at an absolute instant on this node's clock.**
    ///
    /// Two paths, and which one runs is the difference between a 50 µs slot and a 2 ms one:
    ///
    /// * **`CMD_TX_AT_ABS` (v3, preferred)** — the host names the instant and sends nothing else.
    ///   The node's own counter decides when to key up, so the host is not in the measurement.
    /// * **`CMD_READ_CLOCK` + `CMD_TX_AT` (v2 fallback)** — read the node's clock, turn the gap
    ///   into a delay in µs through [`NodeProfile::stamp_hz`], send that. The tick→µs conversion is
    ///   why this can never be a constant: the same `target_tick` is a millisecond count on one node
    ///   in this fleet and a 62.5 ns count on another.
    ///
    /// ★ **Do not "simplify" the first path away.** The relative opcode's delay is counted from
    /// when the *firmware* processes the arm, so the host→device serial latency lands inside the
    /// placement. MEASURED on the LR2021, an absolute-boundary slot train: 45/45 fired and the mean
    /// gap was 2 399 818 ticks against 2 400 000 nominal — within 11 µs over 44 slots, so the
    /// *accuracy* was never the problem — while the jitter was **sd 553 µs, p2p 1875 µs** against a
    /// declared 50 µs `sched_gran_ns`. That is the serial link, not the radio: the same node's
    /// `CMD_GET_INFO` round trip has a p2p of 550 µs, the same number. As exercised, host-armed
    /// relative scheduling was therefore *worse than the software path* (sd 553 vs 155 µs), because
    /// it pays an extra round trip to learn a "now" that has already moved by the time it is used.
    ///
    /// Falls through to plain injection when the node can act on neither, or the target is in a
    /// domain that is not its own — a tick in someone else's domain is not a time on this radio.
    async fn inject_at_clock(
        &self,
        frame: InjectFrame,
        target_tick: u64,
        domain: ClockDomainId,
    ) -> Result<(), FaceError> {
        let prof = self.profile();
        if !prof.schedules_at_clock(domain == self.device_domain) {
            return self.inject(frame).await;
        }
        if prof.schedules_tx_abs() {
            // No clock read: the instant is the message.
            return self.inject_at_abs(frame, target_tick, 0).await;
        }
        let now = self.read_device_clock_async().await?;
        let delta_ticks = target_tick.saturating_sub(now);
        if delta_ticks == 0 {
            return self.inject(frame).await; // already due (or past) — do not wait a wrap
        }
        let delay_us = (delta_ticks as u128 * 1_000_000u128 / prof.stamp_hz.max(1) as u128) as u64;
        self.inject_after(frame, delay_us).await
    }
}

/// The node's named-time surface, **derived from its profile and its clock reference**.
///
/// This used to be a hard-coded "the LoRa bridge reports no hardware timestamp". That was true of the
/// Waveshare and false of the LR2021, whose 16 MHz DPPI capture is the single highest-value timing
/// capability in the rig.
///
/// ★ **A `FreeRunRxStamp` is no longer sufficient for
/// [`can_common_view`](ndn_radio_hal::FaceTimeProfile::can_common_view), and this fleet is why.**
/// The Waveshare's TIM3 input capture landed and is real — 95/95 frames hardware-stamped, 0
/// fallbacks, 0 mis-attributions — so it now declares exactly the same `FreeRunRxStamp` at exactly
/// the same latch point as the LR2021. MEASURED across two receivers of the same frames, they are
/// not the same at all: 0.81-1.86 us for two LR2021s and FLAT against the fit span, against
/// 10.5-20.4 us for two Waveshares and GROWING with it. The difference is the reference — a crystal
/// versus an 8 MHz internal RC — and it lives in [`RadioTimeSource::reference`], asked for over the
/// wire with [`CMD_GET_CLOCK_REF`] and assumed conservatively when the node will not say.
impl RadioTime for LoraSerialBackend {
    fn time_sources(&self) -> Vec<RadioTimeSource> {
        let prof = self.profile();
        let mut v = Vec::new();
        // Only a HARDWARE free-running stamp qualifies. `stamp_kind` 1 (host-recv) and 2 (a firmware
        // software counter) must NOT produce one — and note WHY, because the reason changed on
        // 2026-08-31 and the old one is still quotable and now wrong. A `FreeRunRxStamp` no longer
        // turns straight into `can_common_view = true`: `FaceTimeProfile::derive` requires the latch
        // AND a reference that holds a rate, on the same source. What survives that change is this:
        // `FreeRunRxStamp` is the LATCH HALF, and declaring it is declaring that the counter was
        // latched by hardware with no software in the path. A firmware scheduler tick is not, so
        // publishing one here would put two nodes' firmware main-loop latencies into a field whose
        // whole meaning is "this number did not pass through software", and a later
        // `EVT_CLOCK_REF` saying `CLOCK_REF_XTAL` — a true statement about the oscillator — would
        // then complete the predicate and hand a software counter a common view. The reference half
        // does not rescue a false latch, and it must not be relied on to: keep this gate on
        // `HardwareFreeRun` alone.
        //
        // There is no `RadioClockKind` for a device software counter, and inventing one of the
        // existing kinds for it would be worse than reporting only the host stamp. (`Bw16SerialBackend`
        // in `serial_radio.rs` is held to the same rule for the same reason.)
        if prof.stamp_kind == StampKind::HardwareFreeRun
            && let Some(tick_ns) = prof.tick_ns()
        {
            v.push(RadioTimeSource {
                kind: RadioClockKind::FreeRunRxStamp,
                domain: self.device_domain,
                // The node's own hardware edge capture, not an 802.11 MAC completion — the same
                // latch point `rx_stamp` publishes, so the per-frame stamp and this advertisement
                // cannot disagree about how good the clock is.
                latch: LatchPoint::RadioCapture,
                // 1e9 / stamp_hz — 16 MHz gives 63 ns (a MEASURED 62.5 ns tick, rounded to integer ns).
                precision_ns: tick_ns,
                tick_ns,
                monotonic: true,
                read_now: prof.has_readable_clock(),
                // The OTHER axis, and the one `stamp_kind` cannot see: what that counter counts.
                // Note that this is deliberately independent of everything above it — a node can
                // move its whole timebase from an RC to a crystal without a single byte of `EVT_CAP`
                // changing, which is exactly what the Waveshare did.
                reference: self.clock_reference,
            });
        }
        // Always available, always honest, always last: the host clock the serial line is read on.
        v.push(RadioTimeSource::host_recv(HOST_CLOCK_DOMAIN));
        v
    }

    fn read_clock(&self, domain: ClockDomainId) -> Result<Option<u64>, FaceError> {
        if domain == HOST_CLOCK_DOMAIN {
            return Ok(Some(host_stamp().raw));
        }
        if domain == self.device_domain && self.profile().has_readable_clock() {
            return self.read_device_clock().map(Some);
        }
        Ok(None)
    }
}

impl RadioProfile for LoraSerialBackend {
    /// Built from the node's own `EVT_CAP`, not from a preset.
    ///
    /// What each field now comes from, versus the `RadioCapability::lora(vec![tx_ch])` it replaces:
    /// `channels` is the node's real frequency span rather than the one channel it happens to be
    /// tuned to; `rate` is its real SF span, or [`RateCapability::None`] for a fixed-rate FLRC node
    /// that has no spreading factor at all; `max_payload` is the smaller of the host budget and the
    /// node's end-to-end cap (the preset's `256` is larger than either); `tx_power_dbm` is attached
    /// only when the node declared a real range; `duty_cycle_max` follows the band the carrier is
    /// in, so a US-915 node stops claiming the ETSI 1% ceiling; and `retune_us` is the MEASURED cost
    /// of a channel change on this node's modem, which the preset left `None` on every LoRa radio.
    fn capability(&self) -> RadioCapability {
        capability_from(&self.profile(), self.params().tx_ch)
    }
}

/// [`RadioProfile::capability`] as a pure function of the profile and the tuned channel — so the
/// construction the stack actually consumes can be exercised without a serial port, instead of being
/// re-implemented (and therefore only ever agreeing with itself) in a test.
fn capability_from(prof: &NodeProfile, tuned: u8) -> RadioCapability {
    let rate = match prof.clamp_sf(prof.sf_min) {
        Some(_) => RateCapability::Lora {
            min_sf: prof.sf_min,
            max_sf: prof.sf_max,
        },
        // FLRC is a single fixed rate: not "SF 7", no rate ceiling to reason about.
        None => RateCapability::None,
    };
    let mut cap = RadioCapability::lora_with(
        // Every code in `LoraRadioKind` is a sub-GHz long-range/low-rate part, which is what
        // `RadioKind::Lora` classifies — including the LR2021's FLRC mode. The modulation
        // difference is carried by `rate`, where a planner can actually read it.
        RadioKind::Lora,
        prof.bands(),
        prof.channels(tuned),
        rate,
        prof.frame_budget(),
        prof.duty_cycle_max(channel_to_hz(tuned)),
    );
    // `max_tx_power` is documented as a chip TXAGC *index* ceiling, and both LoRa constructors
    // fill it with 63 — a number no node in this fleet has, because `CMD_SET_PWR`'s wire byte is
    // an i8 **dBm**: the index scale and the dBm scale are the same scale here. 63 made the
    // cognition index back-off inert (`decide_power` returns `63 − backoff_idx`, which
    // `set_tx_power` then clamps straight back up to the PA maximum for any realistic back-off).
    // Deriving it through `clamp_dbm` — the very function the actuator uses — makes the declared
    // ceiling provably the one that will be enforced, on an EVT_CAP range or the legacy fallback
    // alike, so the two can never disagree.
    cap.max_tx_power = prof.clamp_dbm(i8::MAX).max(0) as u8;
    // MEASURED per modem, so `can_hop`/`retune_overhead` are answerable on this bearer for the
    // first time — and the answer is not one answer: at a 100 ms dwell the Heltec (5.6 ms) passes
    // and the Waveshare (161 ms, a full image calibration) does not. `None` where the part has
    // never been timed, or where the node cannot retune at all. See `NodeProfile::retune_us`.
    cap.retune_us = prof.retune_us();
    // ★ Modulation as a capability. `phy_current` is what every other field above must be read
    // against — the payload cap, the rate model and the band all changed with it — and `phy_modes`
    // is what a planner may switch TO. A v2 node lands here with a one-entry set, which is the
    // honest statement: without `CMD_SET_PHY` no other mode is reachable on it.
    cap = cap.with_phy(prof.phy_modes(), prof.phy_current);
    // The hop sequencer, deliberately beside `retune_us` and deliberately not it: one prices a
    // host-commanded retune, the other says the radio walks a list by itself inside a packet.
    if let Some(h) = prof.hop_capability() {
        cap = cap.with_hop(h);
    }
    match prof.dbm_range() {
        Some(r) => cap.with_tx_power_dbm(r),
        None => cap,
    }
}

/// Control plane: the sub-GHz dials, actuated at runtime as binary commands the firmware applies
/// live. Every one is gated on the node's capability bitmap first, so a knob this node does not have
/// **refuses** instead of transmitting a command that another board in the fleet would answer.
impl RadioKnobs for LoraSerialBackend {
    fn set_channel(&self, channel: u8, _bw: Bandwidth) -> Result<(), FaceError> {
        let prof = self.profile();
        if !prof.supports(CMD_SET_FREQ) {
            // A node that cannot retune must say so. Silently succeeding would let a hopping plan
            // believe it moved; on the LR2021, actually sending the command would break TX for good.
            return if channel == self.params().tx_ch {
                Ok(())
            } else {
                Err(unsupported(format!(
                    "this node is fixed at {} Hz and does not implement CMD_SET_FREQ",
                    prof.freq_min_hz
                )))
            };
        }
        let want = channel_to_hz(channel);
        let hz = prof.clamp_hz(want);
        if hz != want {
            return Err(unsupported(format!(
                "channel {channel} = {want} Hz is outside this node's {}–{} Hz range",
                prof.freq_min_hz, prof.freq_max_hz
            )));
        }
        // LoRa is half-duplex on a single carrier: point TX and RX at it together.
        self.exec_idempotent(CMD_SET_FREQ, &hz.to_be_bytes(), EVT_INFO)?;
        let mut p = self.params.lock().unwrap();
        p.tx_ch = channel;
        p.rx_ch = channel;
        Ok(())
    }

    /// The index knob. This bearer's "index" has always been dBm in disguise (the wire byte is an
    /// i8 dBm), so the returned [`AppliedPower`] reports [`PowerReference::AbsoluteDbm`] and a real
    /// `dbm` — on LoRa the absolute axis is not a fiction, it is the wire format.
    ///
    /// ⚠ There is no calibrated/raw split here and no raw axis to reach: `Raw` writes the same
    /// dBm byte, clamped by the node's declared range like everything else.
    fn set_tx_power(&self, req: PowerRequest) -> Result<AppliedPower, FaceError> {
        let want: i8 = match &req {
            // "as loud as this part will legally go" = the top of the node's declared dBm range.
            PowerRequest::Ceiling(_) => {
                self.profile().dbm_range().map(|r| r.max).unwrap_or(i8::MAX)
            }
            PowerRequest::Index(i, _) => (*i).min(i8::MAX as u8) as i8,
            PowerRequest::Raw { idx, .. } => (*idx).min(i8::MAX as u8) as i8,
            PowerRequest::Dbm(d) => *d,
            PowerRequest::NoActuator => {
                return Err(unsupported(
                    "lora: PowerRequest::NoActuator, but CMD_SET_PWR DOES actuate power.".into(),
                ));
            }
        };
        let dbm = self.profile().clamp_dbm(want);
        self.exec_idempotent(CMD_SET_PWR, &[dbm as u8], EVT_INFO)?;
        self.params.lock().unwrap().pwr = dbm.max(0) as u8;
        Ok(AppliedPower::absolute_dbm(req.clone(), dbm, dbm != want))
    }

    /// **The portable absolute-power knob**, returning the power actually applied.
    ///
    /// This closes a capability lie: [`RadioCapability::lora`] has always advertised
    /// `DbmRange::new(10, 22)` — a promise that `set_tx_power_dbm` works on this bearer — while the
    /// method was left at its `Unsupported` default, so a planner that budgeted link margin in dB got
    /// an error from the one radio class whose power knob really is absolute dBm. Now the range comes
    /// from the node (`EVT_CAP`), the request is clamped into it, and the clamped value is returned;
    /// a node that has never declared a range gets a refusal rather than a clamp into an invented one.
    fn set_tx_power_dbm(&self, dbm: i8) -> Result<i8, FaceError> {
        let prof = self.profile();
        let Some(range) = prof.dbm_range() else {
            return Err(unsupported(
                "this node has not declared a dBm TX-power range (EVT_CAP pwr_min/max = 0)".into(),
            ));
        };
        let applied = range.clamp(dbm);
        self.exec_idempotent(CMD_SET_PWR, &[applied as u8], EVT_INFO)?;
        self.params.lock().unwrap().pwr = applied.max(0) as u8;
        Ok(applied)
    }

    /// **Ignore listen-before-talk.** The inverse of the firmware's LBT toggle: EDCCA-ignore ON means
    /// transmit without sensing, which on this bearer is exactly LBT OFF.
    ///
    /// The old body was a no-op whose comment claimed "the open firmware transmits without
    /// listen-before-talk (standard LoRa TX), so EDCCA is effectively always ignored" — contradicted
    /// on the same page by `CMD_TX_LBT` and by this type's own `set_lbt`. A scheduler that turned
    /// EDCCA-ignore on to claim an owned slot got silence and kept contending.
    fn set_edcca_ignore(&self, on: bool) -> Result<(), FaceError> {
        if !on && !self.profile().supports(CMD_TX_LBT) {
            // Asking for listen-before-talk on a node that has no LBT path: refuse rather than arm a
            // flag whose transmit opcode does not exist.
            return Err(unsupported(
                "this node does not implement CMD_TX_LBT, so carrier sense cannot be enabled"
                    .into(),
            ));
        }
        self.set_lbt(!on);
        Ok(())
    }

    /// **The portable modulation knob** — [`LoraSerialBackend::set_phy_mode`], which is where the
    /// reasoning lives. Returns the mode actually in effect, and REPLACES this backend's stored
    /// [`NodeProfile`] with the fresh `EVT_CAP` the node replies, so a caller must re-read
    /// [`RadioProfile::capability`] afterwards rather than patching what it held.
    fn set_phy(&self, mode: PhyMode) -> Result<PhyMode, FaceError> {
        self.set_phy_mode(mode)
    }

    /// **The portable hop-plan knob** — [`LoraSerialBackend::set_hop_plan_hz`]. Gated on the node
    /// advertising `CMD_SET_HOP` (0x1E); a node without it refuses rather than leaving a planner
    /// believing its frames are spread across a band they never left.
    fn set_hop_plan(
        &self,
        ctrl: HopControl,
        period: u16,
        freqs_hz: &[u32],
    ) -> Result<(), FaceError> {
        self.set_hop_plan_hz(ctrl, period, freqs_hz)
    }

    /// **The portable receive-gain knob** — [`LoraSerialBackend::set_rx_gain_mode`]. A posture, not
    /// a dB figure: the wire is one boolean byte on every node in this fleet.
    fn set_rx_gain(&self, gain: RxGain) -> Result<(), FaceError> {
        self.set_rx_gain_mode(gain)
    }

    fn set_spreading_factor(&self, sf: u8) -> Result<(), FaceError> {
        let Some(sf) = self.profile().clamp_sf(sf) else {
            return Err(unsupported(
                "this node has no spreading factor (fixed-rate modulation)".into(),
            ));
        };
        self.params.lock().unwrap().sf = sf;
        self.send_mod()
    }

    /// LoRa coding rate 4/5..4/8. Refused on a fixed-rate node **before** the cached params move,
    /// so a rejected knob never leaves the host believing a value the node does not hold — the FLRC
    /// `cr` code space is a different one (`0 = 1/2, 1 = 3/4, 2 = off, 3 = 2/3`).
    fn set_coding_rate(&self, cr: u8) -> Result<(), FaceError> {
        if !self.profile().has_spreading_factor() {
            return Err(unsupported(
                "this node's CMD_SET_MOD cr byte is not a LoRa coding rate".into(),
            ));
        }
        let cr = cr.clamp(1, 4);
        self.params.lock().unwrap().cr = cr;
        self.send_mod()
    }

    /// Channel bandwidth. Bearer-agnostic in principle, but it can only travel inside the
    /// `CMD_SET_MOD` triple, so it inherits that opcode's gate — see [`Self::send_mod`].
    fn set_bandwidth_khz(&self, khz: u32) -> Result<(), FaceError> {
        if !self.profile().has_spreading_factor() {
            return Err(unsupported(
                "this node has no CMD_SET_MOD bandwidth byte the host may write portably".into(),
            ));
        }
        let bw = match khz {
            0..=180 => 0,   // 125 kHz
            181..=360 => 1, // 250 kHz
            _ => 2,         // 500 kHz
        };
        self.params.lock().unwrap().bw = bw;
        self.send_mod()
    }

    /// From the profile: [`TxDiscipline::ScheduledAt`] only when the node declares a real scheduling
    /// granularity AND implements `CMD_TX_AT`; otherwise the honest [`TxDiscipline::BestEffort`] —
    /// a serial bridge plus a duty-cycled medium makes the on-air instant loose.
    fn tx_discipline(&self) -> TxDiscipline {
        self.profile().tx_discipline()
    }

    /// The frame-free occupancy counter, from `CMD_SENSE`'s free-running `activity` count.
    ///
    /// ⚠ `Ok(None)` is terminal for the caller: `spawn_occupancy_sampler` polls once and **kills its
    /// task** on the first `None`, so a node that can sense must not answer `None` on a transient
    /// serial hiccup — hence a read error propagates as `Err` (the sampler retries) and only a node
    /// with no `CMD_SENSE` at all returns `None`.
    fn read_channel_activity(&self) -> Result<Option<u16>, FaceError> {
        if !self.profile().supports(CMD_SENSE) {
            return Ok(None);
        }
        self.sense().map(|(activity, _rssi)| Some(activity))
    }

    /// **The modem's own PPDU counters, where the node reports them** —
    /// `(nbPktReceived, nbPktCrcError + nbPktHeaderErr)` out of the `EVT_STATS` v2 tail; `None` from a
    /// node whose reply is 24 bytes (the Heltec and the LR2021 today) or that has no `CMD_GET_STATS`.
    ///
    /// It was unconditionally `None` until this run, and the reasoning was right about the wrong
    /// candidates: `EVT_STATS.rx` counts frames that already decoded AND parsed as NDN (neither all
    /// receptions nor a PHY count), `EVT_INFO.errors` is the SX1262 `GetDeviceErrors` word (PA ramp /
    /// PLL / XOSC — chip health, not reception), and `cad_busy` is a TX-side observation. None of
    /// those is `(ok, err)`. The v2 tail is a **fourth** source that had not existed: the SX126x's own
    /// counters, where a CRC failure — a reception the PHY began and lost, which the firmware
    /// otherwise drops in silence — is finally visible. See [`NdnStats::phy_counters`] for the exact
    /// mapping and for the one datasheet ambiguity that mapping does not resolve.
    ///
    /// Costs a serial round trip (`CMD_GET_STATS`), like [`read_channel_activity`](Self::read_channel_activity).
    /// The counters are free-running `u16` and wrap: difference two reads, per the HAL contract.
    fn read_ofdm_counters(&self) -> Result<Option<(u16, u16)>, FaceError> {
        if !self.profile().supports(CMD_GET_STATS) {
            return Ok(None);
        }
        Ok(self.ndn_stats()?.phy_counters())
    }

    /// `None`, deliberately.
    ///
    /// The HAL defines this as `(tx_en, tx_on)` — MAC→baseband transmit *requests* versus
    /// baseband→RF *keys*, whose whole value is separating "we never asked" from "we asked and the
    /// air ate it". This firmware exposes no such pair: `EVT_TXDONE` is a per-command reply rather
    /// than a free-running counter, and `(cad_busy, defer)` are contention outcomes on the far side of
    /// that boundary. A pair that looked like the register read but answered a different question
    /// would be worse than none, because this counter is precisely what one reaches for to *stop*
    /// guessing.
    fn read_tx_counters(&self) -> Result<Option<(u16, u16)>, FaceError> {
        Ok(None)
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// M6 · §1.4 — THE PLAN.  The 7E-A5 sub-GHz serial fleet (SX1262 / SX1276 / LR2021).
// ─────────────────────────────────────────────────────────────────────────────
//
// Specification: `docs/bringup-contract.md` §1.4/§5-M6 — *"LoRa gets its first `OpenRadio`
// constructor (today the only one is built inside `ndn-phy-lora/examples/lora_face_node.rs`)."*
//
// That example is the whole reason this exists. It hand-assembles an `OpenRadio` from four clones
// of the backend and fills the last field with `BringUpReport::synthetic("LoRa serial node")` —
// a report that says, correctly, that no ladder in this workspace brought the radio up. It is the
// only place in the tree that opens a LoRa radio as a full handle, so every other consumer either
// invents its own four-clone block or throws the knobs, clock and profile away.
//
// ⚠ **What this plan honestly is, and is not.** This part DOES have a real bring-up sequence —
// `CMD_GET_CAP` -> `CMD_GET_CLOCK_REF` -> `configure()` — and it runs inside
// [`LoraSerialBackend::open_inner`], BEFORE `Self` exists. `run_plan` hands a rung `&Arc<B>`, so
// those three exchanges cannot be rungs without splitting the constructor, and splitting a
// working probe order on a radio nobody at this keyboard can test is precisely the unmeasured
// change the house rules forbid. So they are declared with [`StepClass::OutOfBand`] naming
// `open_inner` as the establisher, and the rung READS BACK what the exchange concluded:
// learned-or-fallback profile, the clock reference, the programmed parameters.
//
// ★ **This is the one place `OutOfBand` is used for work inside this crate**, and the contract
// reserves it for `modprobe`/`iw`/`hostapd_s1g`/`morse_cli`. The reason is written above rather
// than hidden; folding the three probes into rungs is an OPEN ITEM, and it needs `open_inner`
// split into "construct the handle" and "interrogate the node" first.
//
// The one rung that acts is `set_channel`, and it acts only when the caller names a channel.

type Lo = LoraSerialBackend;

fn s_lora_capability(b: &Arc<Lo>, c: &mut Ctx<'_>) -> Result<StepOutcome, FaceError> {
    let prof = b.profile();
    if !prof.learned {
        // ☠ The trap this rung exists to surface. Before `EVT_CAP`, every node in this fleet
        // claimed to be an SX1262 — and `CMD_SET_MOD`'s bandwidth byte is PER-NODE, so a host
        // driving an LR2021 on an SX1262 fallback detunes it to 7.81 kHz and produces a
        // convincing, entirely fake interop failure.
        c.warn(
            "the node never answered CMD_GET_CAP: this handle is running on the HOST's written-down \
             fallback profile, not on anything the node said about itself. Firmware predating the \
             7E-A5 v2 capability protocol, or a node that acked the opcode as unsupported. Every \
             rate, power and bandwidth number below is then an ASSUMPTION — and the fallback is a \
             legacy SX1262, on which CMD_SET_MOD's bandwidth byte means something different from \
             what an LR2021 or a Heltec SX1276 expects",
        );
        return Ok(StepOutcome::Branch("host-fallback-profile"));
    }
    Ok(StepOutcome::Branch("node-reported (EVT_CAP)"))
}

fn s_lora_clock_reference(b: &Arc<Lo>, c: &mut Ctx<'_>) -> Result<StepOutcome, FaceError> {
    // The pair that decides `FaceTimeProfile::can_common_view` is (latch point, reference), and
    // the reference is the half that was being inferred until `CMD_GET_CLOCK_REF` existed. Putting
    // it in the report is what lets a common-view number be read beside the clock it was taken on.
    let r = b.clock_reference();
    c.warn(format!(
        "clock reference: {r:?} (latch point travels per frame; the PAIR is what decides whether \
         two nodes' stamps of one frame can be differenced at all)"
    ));
    Ok(StepOutcome::Done)
}

fn s_lora_params(b: &Arc<Lo>, c: &mut Ctx<'_>) -> Result<StepOutcome, FaceError> {
    let p = b.params();
    let prof = b.profile();
    // The rate this radio was actually left at, in words as well as codes — §3's `RateState` rule
    // exists because a bare modulation code has cost this project days.
    c.state().rate = ndn_radio_hal::RateState::new(
        u32::from(p.sf),
        if prof.has_spreading_factor() {
            format!(
                "LoRa SF{}/{} kHz CR 4/{} , preamble {} sym",
                p.sf,
                p.bw_khz(),
                4 + p.cr,
                p.preamble
            )
        } else {
            format!(
                "no spreading factor on this node (FLRC or FSK) — bw code {}, preamble {} sym",
                p.bw, p.preamble
            )
        },
    );
    // ★ The one radio family in the fleet with a REAL absolute dBm axis at bring-up, and the
    // contract's rule is that `dbm` is populated only where such an axis exists.
    c.state().power =
        AppliedPower::absolute_dbm(PowerRequest::Dbm(p.pwr as i8), p.pwr as i8, false);
    Ok(StepOutcome::Established(Fact::PowerReference(
        ndn_radio_hal::bringup::PowerReference::AbsoluteDbm,
    )))
}

fn s_lora_set_channel(b: &Arc<Lo>, c: &mut Ctx<'_>) -> Result<StepOutcome, FaceError> {
    let ch = c.state_ref().channel;
    if ch == 0 {
        return Ok(StepOutcome::Skipped(
            "no channel named by the caller — the node stays on the carrier `configure()` \
             programmed from `LoraParams`, which is what every LoRa opener did before M6",
        ));
    }
    if ch == b.params().tx_ch {
        return Ok(StepOutcome::Skipped(
            "already on the requested carrier — `configure()` programmed it from `LoraParams`, so \
             re-sending CMD_SET_FREQ would cost a retune (MEASURED 82,810 us on the Waveshare) for \
             no change",
        ));
    }
    RadioKnobs::set_channel(b.as_ref(), ch, c.state_ref().bw)?;
    Ok(StepOutcome::Done)
}

const LORA_R_CAPABILITY: Step<Lo> = Step {
    id: StepId("capability_probe"),
    stage: Stage::Attach,
    class: StepClass::OutOfBand {
        established_by: "LoraSerialBackend::open_inner's CMD_GET_CAP -> EVT_CAP exchange, which \
                         must complete before the handle exists",
    },
    why: "★ The keystone of the 7E-A5 v2 protocol: ask the node what it IS before sending it \
          anything else. This rung reads back WHICH answer the handle is running on — the node's \
          own `EVT_CAP`, or the host's written-down fallback. ☠ It matters because before v2 every \
          node claimed to be an SX1262, and `CMD_SET_MOD`'s bandwidth byte is PER-NODE: driving an \
          LR2021 on an SX1262 fallback detunes it to 7.81 kHz and fakes a cross-vendor interop \
          failure. `cmd_bitmap` says 'implemented', NOT 'means the same thing'.",
    must_follow: &[],
    must_precede: &[],
    run: s_lora_capability,
};

const LORA_R_CLOCK_REFERENCE: Step<Lo> = Step {
    id: StepId("clock_reference"),
    stage: Stage::Attach,
    class: StepClass::OutOfBand {
        established_by: "LoraSerialBackend::open_inner's CMD_GET_CLOCK_REF -> EVT_CLOCK_REF \
                         exchange (or the demote-only host fallback when the node does not \
                         implement the opcode)",
    },
    why: "`EVT_CAP.stamp_kind` says where a node LATCHES; this says what the latched counter is \
          DERIVED FROM, and the PAIR is what decides `can_common_view`. It belongs in the report \
          because a common-view number is meaningless without the clock it was taken on — the \
          Waveshare's own history is the argument (1.09 us on the crystal against ~36 us of \
          software-stamp floor on the HSI RC, same part, same code).",
    must_follow: &[StepId("capability_probe")],
    must_precede: &[],
    run: s_lora_clock_reference,
};

const LORA_R_PARAMS: Step<Lo> = Step {
    id: StepId("params_programmed"),
    stage: Stage::Tune,
    class: StepClass::OutOfBand {
        established_by: "LoraSerialBackend::open_inner's `configure()`, which programs only the \
                         parameters the learned profile says the node implements, clamped into \
                         the ranges it declares",
    },
    why: "reads back the modulation, carrier and power the handle was left on and puts them in the \
          report as a decoded `RateState` plus a REAL absolute-dBm `AppliedPower`. This fleet is \
          the one place in the workspace where `AppliedPower::dbm` is honestly `Some`: LoRa power \
          is an absolute axis the node declares, not a chip index somebody converted.",
    must_follow: &[StepId("capability_probe")],
    must_precede: &[],
    run: s_lora_params,
};

const LORA_R_SET_CHANNEL: Step<Lo> = Step {
    id: StepId("set_channel"),
    stage: Stage::Tune,
    class: StepClass::Required,
    why: "the one rung here that puts bytes on the wire, and only when the caller names a channel \
          that differs from the one `configure()` already programmed. ⚠ A retune is not free on \
          this bearer — MEASURED 82,810 us on the Waveshare after the CalibrateImage skip, and \
          160,866 us before it — so a redundant re-send is refused rather than paid for. A node \
          that cannot retune says so through `RadioKnobs::set_channel`'s own refusal instead of \
          silently succeeding, which on the LR2021 would break TX for good.",
    must_follow: &[StepId("params_programmed")],
    must_precede: &[],
    run: s_lora_set_channel,
};

const LORA_STEPS: &[Step<Lo>] = &[
    LORA_R_CAPABILITY,
    LORA_R_CLOCK_REFERENCE,
    LORA_R_PARAMS,
    LORA_R_SET_CHANNEL,
];

const LORA_PLAN: Plan<Lo> = Plan {
    id: PlanId {
        part: "lora-7ea5",
        name: "node",
        ver: 1,
    },
    role: Role::TransmitAndReceive,
    steps: LORA_STEPS,
    excluded: &[
        (
            Stage::Firmware,
            "the node runs flashed firmware and there is no download path over this wire. The one \
             reflash mechanism that exists — CMD_ENTER_BOOTLOADER + stm32flash over CH343 — is an \
             operator action, not a bring-up rung, and it costs a reset round trip.",
        ),
        (
            Stage::PowerOn,
            "opening the port does NOT reset the MCU (DTR is not wired to nRST). The node free-runs \
             whatever it booted; `open_inner` flushes 200 ms of boot chatter and interrogates it.",
        ),
        (
            Stage::Calibrate,
            "no host-driven calibration exists on this bearer. ☠ And image calibration is \
             deliberately NOT re-run per tune: skipping it is what took the retune from 160,866 us \
             to 82,810 us, a measured property of the shipped firmware.",
        ),
    ],
};

const _: () = LORA_PLAN.check_or_panic();

/// The 7E-A5 sub-GHz fleet's one plan — see the M6 block above for what it declares and what it
/// only reads back.
pub static PLAN_LORA_NODE: Plan<Lo> = LORA_PLAN;

impl BringUp for LoraSerialBackend {
    fn plan(role: Role) -> Option<&'static Plan<Self>> {
        match role {
            Role::TransmitAndReceive => Some(&PLAN_LORA_NODE),
            // A named refusal. LoRa is half-duplex on one carrier and the firmware returns to RX
            // after every transmit; a one-way role would be a claim about a radio this crate does
            // not control.
            Role::ReceiveOnly | Role::TransmitOnly => None,
        }
    }

    fn tx_unprovable_reason() -> Option<&'static str> {
        Some(
            "the 7E-A5 protocol has no TX counter to difference. `CMD_TX_AT_ABS` returns the \
             ACTUAL on-air instant per frame (MEASURED sd 0.7 us on the LR2021), which proves \
             PLACEMENT of a transmit this host requested — not that a probe burst radiated. \
             Question (B) on this bearer is answered by a peer, the way the common-view results \
             were",
        )
    }
}

impl LoraSerialBackend {
    /// **M6 — the first `OpenRadio` constructor for a LoRa radio.**
    ///
    /// Until now the only place in the workspace that assembled one was
    /// `ndn-phy-lora/examples/lora_face_node.rs`, by hand, with
    /// `BringUpReport::synthetic("LoRa serial node")` in the report slot — so every other consumer
    /// either copied that block or opened the backend as a bare `dyn FrameIo` and threw the knobs,
    /// the clock and the profile away.
    ///
    /// All four handles are the same instance: this backend implements `FrameIo`, `RadioKnobs`,
    /// `RadioTime` and `RadioProfile`, and the whole radio travels to the face.
    ///
    /// `channel == 0` leaves the node on the carrier `params` programmed — the pre-M6 behaviour.
    pub fn open_radio(
        path: &str,
        params: LoraParams,
        channel: u8,
    ) -> Result<ndn_radio_hal::OpenRadio, FaceError> {
        let dev = Arc::new(Self::open_with(path, params)?);
        let run = PlanRun::new(
            "LoRa 7E-A5 node",
            ndn_radio_hal::DeviceAddress::Serial(path.to_string()),
            RadioState {
                channel: if channel == 0 {
                    dev.params().tx_ch
                } else {
                    channel
                },
                // ⚠ `ndn_radio_hal::Bandwidth` enumerates Wi-Fi widths and cannot express a LoRa
                // 125/250/500 kHz channel at all. `Bw20` here is a placeholder the type forces,
                // NOT a claim: the real width is in `RateState`, decoded, from `LoraParams::bw`.
                bw: Bandwidth::Bw20,
                format: "RawNdn (NDNLPv2/NDN-TLV over the 7E-A5 wire)",
                role: Role::TransmitAndReceive,
                power: AppliedPower::no_actuator(PowerRequest::NoActuator),
                rate: ndn_radio_hal::RateState::unreported(),
                warm: None,
                contention: None,
                pump: PumpPolicy::CallerOwns,
                facts: Vec::new(),
            },
        );
        let (report, guards) = <Self as BringUp>::bring_up(&dev, &run).map_err(|f| {
            eprintln!(
                "LoRa bring-up FAILED at `{}` — the partial report:\n{}",
                f.failed_at,
                f.report.render()
            );
            f.source
        })?;
        debug_assert!(guards.is_empty(), "the LoRa plan produces no guards");
        // ⚠ No second `emit()`: `run_plan` already emitted this report. The capability is
        // attached afterwards because the runner is part-agnostic and cannot know it — the same
        // ordering every other part uses.
        let report = report.with_capability(ndn_radio_hal::RadioProfile::capability(dev.as_ref()));
        Ok(ndn_radio_hal::OpenRadio {
            io: dev.clone(),
            knobs: Some(dev.clone()),
            time: Some(dev.clone()),
            profile: Some(dev),
            report,
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Build a wire frame the way a node would emit an event.
    fn wire(typ: u8, payload: &[u8]) -> Vec<u8> {
        let mut v = vec![SYNC0, SYNC1, typ, payload.len() as u8];
        v.extend_from_slice(payload);
        let mut crc = typ ^ (payload.len() as u8);
        for &b in payload {
            crc ^= b;
        }
        v.push(crc);
        v
    }

    // ── framing (unchanged behaviour, still pinned) ─────────────────────────────────────────────

    #[test]
    fn parses_a_clean_rx_event() {
        // rssi = -31, snr = 9, payload "hi"
        let mut p = Vec::new();
        p.extend_from_slice(&(-31i16).to_be_bytes());
        p.extend_from_slice(&9i16.to_be_bytes());
        p.extend_from_slice(b"hi");
        let buf = wire(EVT_RX, &p);
        match next_event(&buf) {
            EvParse::Event {
                typ,
                payload,
                consumed,
            } => {
                assert_eq!(typ, EVT_RX);
                assert_eq!(consumed, buf.len());
                assert_eq!(i16::from_be_bytes([payload[0], payload[1]]), -31);
                assert_eq!(&payload[4..], b"hi");
            }
            _ => panic!("expected an event"),
        }
    }

    #[test]
    fn skips_leading_garbage_then_parses() {
        let mut buf = vec![0x00, 0x11, 0x22];
        buf.extend_from_slice(&wire(EVT_TXDONE, &[1]));
        match next_event(&buf) {
            EvParse::Drop { consumed } => buf.drain(..consumed),
            _ => panic!("expected drop of garbage"),
        };
        match next_event(&buf) {
            EvParse::Event { typ, payload, .. } => {
                assert_eq!(typ, EVT_TXDONE);
                assert_eq!(payload, vec![1]);
            }
            _ => panic!("expected event after garbage"),
        }
    }

    #[test]
    fn partial_frame_needs_more() {
        let full = wire(EVT_RX, &[0, 0, 0, 0, b'x']);
        assert!(matches!(next_event(&full[..6]), EvParse::Need));
    }

    #[test]
    fn crc_miss_is_dropped_and_resyncs() {
        let mut buf = wire(EVT_RX, &[0, 0, 0, 0, b'a']);
        let last = buf.len() - 1;
        buf[last] ^= 0xff; // wreck the crc
        let good = wire(EVT_TXDONE, &[1]);
        buf.extend_from_slice(&good);
        match next_event(&buf) {
            EvParse::Drop { consumed } => {
                assert_eq!(consumed, 2);
                buf.drain(..consumed);
            }
            _ => panic!("expected drop of corrupt frame"),
        }
        loop {
            match next_event(&buf) {
                EvParse::Event { typ, .. } => {
                    assert_eq!(typ, EVT_TXDONE);
                    break;
                }
                EvParse::Drop { consumed } => {
                    buf.drain(..consumed);
                }
                EvParse::Need => panic!("lost the good frame"),
            }
        }
    }

    #[test]
    fn command_frame_round_trips_through_parser() {
        // A command we send should parse back with the same type/payload (same framing both ways).
        let payload = 915_000_000u32.to_be_bytes();
        let buf = wire(CMD_SET_FREQ, &payload);
        match next_event(&buf) {
            EvParse::Event {
                typ, payload: got, ..
            } => {
                assert_eq!(typ, CMD_SET_FREQ);
                assert_eq!(
                    u32::from_be_bytes([got[0], got[1], got[2], got[3]]),
                    915_000_000
                );
            }
            _ => panic!("expected event"),
        }
    }

    #[test]
    fn channel_maps_to_us_and_eu() {
        assert_eq!(channel_to_hz(65), 915_000_000);
        assert_eq!(channel_to_hz(18), 868_000_000);
        assert_eq!(hz_to_channel(915_000_000), 65);
        assert_eq!(hz_to_channel(928_000_000), 78);
    }

    // ── EVT_CAP -> NodeProfile ─────────────────────────────────────────────────────────────────

    /// The 29-byte layout is a contract with three firmware repos; pin it byte by byte from a
    /// hand-built payload rather than from `to_cap_payload` (which would only test itself).
    #[test]
    fn evt_cap_parses_into_a_node_profile() {
        let mut p = Vec::new();
        p.push(2); // [0]    proto_ver
        p.push(2); // [1]    radio_kind = the LR2021 PART (v3); in v2 this byte read "LR2021-FLRC"
        p.extend_from_slice(&915_000_000u32.to_be_bytes()); // [2..6]   freq_min
        p.extend_from_slice(&928_000_000u32.to_be_bytes()); // [6..10]  freq_max
        p.push((-9i8) as u8); // [10]   pwr_min dBm (negative, so this pins the i8 decode)
        p.push(22); // [11]   pwr_max dBm
        p.extend_from_slice(&16_000_000u32.to_be_bytes()); // [12..16] stamp_hz
        p.push(3); // [16]   stamp_kind = hardware free-running
        p.extend_from_slice(&48u16.to_be_bytes()); // [17..19] max_payload
        p.extend_from_slice(&cmd_bits(&[CMD_TX, CMD_TX_AT, CMD_READ_CLOCK]).to_be_bytes()); // [19..23]
        p.push(0); // [23]   sf_min (none)
        p.push(0); // [24]   sf_max
        p.extend_from_slice(&62_500u32.to_be_bytes()); // [25..29] sched_gran_ns
        assert_eq!(p.len(), CAP_LEN_V2, "a v2 EVT_CAP is exactly 29 bytes");

        let prof = NodeProfile::parse(&p).expect("29 bytes parse");
        assert_eq!(prof.proto_ver, 2);
        assert_eq!(prof.radio_kind, LoraRadioKind::Lr2021);
        assert_eq!(
            prof.phy_current,
            PhyMode::Flrc,
            "a v2 payload has no PHY tail, so the mode is recovered from radio_kind 2"
        );
        assert_eq!(
            prof.phy_modes(),
            PhyModeSet::single(PhyMode::Flrc),
            "and the set is exactly one entry — a v2 node implements no CMD_SET_PHY, so no other \
             mode is REACHABLE on it however many the silicon has"
        );
        assert!(!prof.phy_agile());
        assert_eq!(prof.freq_min_hz, 915_000_000);
        assert_eq!(prof.freq_max_hz, 928_000_000);
        assert_eq!(prof.pwr_min_dbm, -9);
        assert_eq!(prof.pwr_max_dbm, 22);
        assert_eq!(prof.stamp_hz, 16_000_000);
        assert_eq!(prof.stamp_kind, StampKind::HardwareFreeRun);
        assert_eq!(prof.max_payload, 48);
        assert_eq!(prof.sched_gran_ns, 62_500);
        assert!(prof.learned, "an EVT_CAP-derived profile is learned");
        assert!(prof.supports(CMD_TX) && prof.supports(CMD_TX_AT));
        assert!(
            !prof.supports(CMD_SET_FREQ),
            "an unset bit must read as unsupported — this is the gate that keeps SET_FREQ off the \
             LR2021, where it was measured to permanently break TX"
        );
        assert_eq!(prof.clamp_sf(7), None, "sf 0/0 means NO spreading factor");
        // Round-trips through the host-side serialiser, so an emulator and the parser agree.
        assert_eq!(&prof.to_cap_payload()[..], &p[..]);
    }

    #[test]
    fn a_short_or_future_evt_cap_is_refused_rather_than_misread() {
        let good = NodeProfile::legacy_sx1262().to_cap_payload();
        assert!(
            NodeProfile::parse(&good[..28]).is_none(),
            "28 bytes is not a capability"
        );
        let mut future = good;
        future[0] = PROTO_VER + 1;
        assert!(
            NodeProfile::parse(&future).is_none(),
            "a newer node may have re-laid the tail; refuse rather than mis-read it"
        );
    }

    // ── the legacy fallback ────────────────────────────────────────────────────────────────────

    #[test]
    fn an_unanswered_get_cap_falls_back_to_the_legacy_profile() {
        // No EVT_CAP, no hint: exactly today's assumptions, so an un-reflashed dongle opens as it did.
        let p = resolve_profile(None, None);
        assert!(!p.learned, "a fallback must not claim the node said so");
        assert_eq!(p.radio_kind, LoraRadioKind::Sx1262);
        assert_eq!(p.dbm_range(), Some(DbmRange::new(10, 22)));
        assert_eq!(p.clamp_sf(7), Some(7));
        // 64, NOT `MAX_LORA_PAYLOAD` (240). The pre-v2 firmware accepts 240 on TX and truncates RX
        // at 64 with no error, so 240 would hand the face an MTU that corrupts silently on receive.
        // A capability must be the end-to-end cap, not the friendlier of the two directions.
        assert_eq!(p.max_payload, LEGACY_RX_TRUNCATION_CAP);
        assert!((p.max_payload as usize) < MAX_LORA_PAYLOAD);
        assert!(p.supports(CMD_SET_FREQ) && p.supports(CMD_SET_MOD) && p.supports(CMD_SET_SYNC));
        assert!(!p.supports(CMD_GET_CAP) && !p.supports(CMD_SENSE) && !p.supports(CMD_TX_AT));
    }

    #[test]
    fn a_hint_pins_the_fallback_and_evt_cap_still_wins() {
        let pinned = resolve_profile(None, Some(RadioKindHint::Lr2021Flrc));
        assert_eq!(pinned.radio_kind, LoraRadioKind::Lr2021);
        assert_eq!(pinned.phy_current, PhyMode::Flrc);
        assert!(
            !pinned.supports(CMD_SET_FREQ),
            "the pinned LR2021 profile must never let SET_FREQ onto the wire"
        );
        assert_eq!(
            pinned.dbm_range(),
            None,
            "no measured dBm span => no dBm knob"
        );

        // The device's own answer overrides the hint, even a contradictory one.
        let cap = NodeProfile::legacy_sx1262().to_cap_payload();
        let learned = resolve_profile(Some(&cap), Some(RadioKindHint::Lr2021Flrc));
        assert_eq!(learned.radio_kind, LoraRadioKind::Sx1262);
        assert!(learned.learned);
    }

    #[test]
    fn the_pinned_heltec_profile_refuses_the_knobs_its_firmware_lacks() {
        let p = RadioKindHint::HeltecSx1276.profile();
        assert!(p.supports(CMD_SET_MOD) && p.supports(CMD_TX_LBT));
        assert!(
            !p.supports(CMD_SET_SYNC),
            "that firmware acks unknown commands with EVT_INFO, so an ungated SET_SYNC would \
             report success and actuate nothing"
        );
        assert!(!p.supports(CMD_SET_BEACON) && !p.supports(CMD_SET_NAME_FILTER));
    }

    // ── stamp units ───────────────────────────────────────────────────────────────────────────

    #[test]
    fn stamp_units_come_from_the_profile_not_from_one_boards_memory() {
        let dom = ClockDomainId(0x1234_5678);

        // 16 MHz hardware capture: raw ticks in the DEVICE domain, 63 ns per tick (62.5 rounded).
        let lr = RadioKindHint::Lr2021Flrc.profile();
        assert_eq!(lr.tick_ns(), Some(63));
        let s = rx_stamp(&lr, dom, 16_625_857, None);
        assert_eq!(s.domain, dom);
        assert_eq!(
            s.raw, 16_625_857,
            "raw ticks are published as ticks, never scaled"
        );

        // The same field on the Waveshare is a millisecond SOFTWARE counter. It must NOT be promoted
        // to the device domain — reading LR2021 ticks as ms would be 16000x wrong, and publishing a
        // firmware scheduler tick as a link clock would let common view difference two main loops.
        let ws = NodeProfile::legacy_sx1262();
        assert_eq!(ws.tick_ns(), Some(1_000_000));
        let s = rx_stamp(&ws, dom, 1234, None);
        assert_eq!(s.domain, HOST_CLOCK_DOMAIN);
        assert_eq!(s.latch, LatchPoint::HostRecv);
    }

    // ── the per-frame stamp verdict (EVT_RX_STAMP, 0x90) ────────────────────────────────────────

    /// Build the eight-byte `EVT_RX` header the firmware emits, plus a payload.
    fn rx_payload(ts: u32, body: &[u8]) -> Vec<u8> {
        let mut p = Vec::new();
        p.extend_from_slice(&(-42i16).to_be_bytes());
        p.extend_from_slice(&7i16.to_be_bytes());
        p.extend_from_slice(&ts.to_be_bytes());
        p.extend_from_slice(body);
        p
    }

    /// ★ **A frame the node says is software-stamped must not be published as a 1 µs radio
    /// capture.**
    ///
    /// This is the consumer half of the firmware's per-frame degrade. The node computes it, counts
    /// it and announces it; before this, the host keyed only on the NODE-level `stamp_kind`, so
    /// every frame from a self-test-passing dongle became `LatchPoint::RadioCapture` at the
    /// capture's tick — including the ones whose `ts` was a poll-loop software read, wrong by up to
    /// a whole poll gap. Exercised through `handle_event`, the real reader dispatch.
    #[test]
    fn a_per_frame_degrade_note_stops_that_frame_reaching_the_device_clock() {
        let (txf, mut rxf) = mpsc::unbounded_channel();
        let (resp, resp_rx) = std::sync::mpsc::channel();
        let stored = Mutex::new(RadioKindHint::Lr2021Flrc.profile());
        let dom = ClockDomainId(0x1234);
        assert_eq!(
            stored.lock().unwrap().stamp_kind,
            StampKind::HardwareFreeRun
        );
        let mut pending = None;

        // 1. A clean frame: no note, so it is the capture the capability advertises.
        handle_event(
            EVT_RX,
            &rx_payload(1_000, b"clean"),
            &txf,
            &resp,
            &stored,
            dom,
            false,
            &mut pending,
        );
        let clean = rxf.try_recv().unwrap().stamp.unwrap();
        assert_eq!(clean.latch, LatchPoint::RadioCapture);
        assert_eq!(clean.domain, dom);
        assert_eq!(clean.raw, 1_000);

        // 2. A note (kind = 2 software counter, reason = 6 coalesced) then its frame.
        handle_event(
            EVT_RX_STAMP,
            &[StampKind::SoftwareCounter.code(), 6],
            &txf,
            &resp,
            &stored,
            dom,
            false,
            &mut pending,
        );
        assert_eq!(pending, Some(StampKind::SoftwareCounter));
        handle_event(
            EVT_RX,
            &rx_payload(2_000, b"degraded"),
            &txf,
            &resp,
            &stored,
            dom,
            false,
            &mut pending,
        );
        let degraded = rxf.try_recv().unwrap().stamp.unwrap();
        assert_eq!(
            degraded.latch,
            LatchPoint::HostRecv,
            "a software fallback is not a radio capture"
        );
        assert_eq!(
            degraded.domain, HOST_CLOCK_DOMAIN,
            "and it must not enter the device clock domain, where common view would difference it"
        );

        // 3. The note is consumed by exactly one frame; the next is clean again.
        assert_eq!(pending, None);
        handle_event(
            EVT_RX,
            &rx_payload(3_000, b"clean again"),
            &txf,
            &resp,
            &stored,
            dom,
            false,
            &mut pending,
        );
        assert_eq!(
            rxf.try_recv().unwrap().stamp.unwrap().latch,
            LatchPoint::RadioCapture,
            "one note qualifies one frame, not the rest of the run"
        );

        // The note is unsolicited chatter: it must never reach the reply channel, where it could
        // satisfy — or be skipped by — a command wait.
        assert!(
            resp_rx.try_recv().is_err(),
            "EVT_RX_STAMP is not a reply to anything"
        );
    }

    /// A note whose frame never arrives must not qualify some later frame. The pairing is ordering
    /// on an in-order link, so anything else on the wire in between breaks it — and the safe
    /// reading of a broken pairing is "this note describes nothing".
    #[test]
    fn a_stranded_note_is_dropped_rather_than_applied_to_the_wrong_frame() {
        let (txf, mut rxf) = mpsc::unbounded_channel();
        let (resp, _resp_rx) = std::sync::mpsc::channel();
        let stored = Mutex::new(RadioKindHint::Lr2021Flrc.profile());
        let dom = ClockDomainId(0x1234);
        let mut pending = None;

        handle_event(
            EVT_RX_STAMP,
            &[StampKind::SoftwareCounter.code(), 1],
            &txf,
            &resp,
            &stored,
            dom,
            false,
            &mut pending,
        );
        handle_event(
            EVT_TXDONE,
            &[1, 0],
            &txf,
            &resp,
            &stored,
            dom,
            false,
            &mut pending,
        );
        assert_eq!(pending, None, "the pairing was broken; the note is stale");
        handle_event(
            EVT_RX,
            &rx_payload(4_000, b"unrelated"),
            &txf,
            &resp,
            &stored,
            dom,
            false,
            &mut pending,
        );
        assert_eq!(
            rxf.try_recv().unwrap().stamp.unwrap().latch,
            LatchPoint::RadioCapture
        );
    }

    /// The two kinds are combined by taking the WORSE of the pair. A per-frame note claiming a
    /// hardware capture from a node that advertises a software counter is a contradiction, and the
    /// resolution is never to promote: the capability is what was earned by a boot self-test.
    #[test]
    fn a_per_frame_note_can_only_ever_lower_the_claim() {
        let dom = ClockDomainId(0x1234_5678);
        let ws = NodeProfile::legacy_sx1262();
        assert_eq!(ws.stamp_kind, StampKind::SoftwareCounter);
        assert_eq!(
            rx_stamp(&ws, dom, 1234, Some(StampKind::HardwareFreeRun)).latch,
            LatchPoint::HostRecv
        );
        let lr = RadioKindHint::Lr2021Flrc.profile();
        assert_eq!(
            rx_stamp(&lr, dom, 1234, Some(StampKind::HardwareFreeRun)).latch,
            LatchPoint::RadioCapture,
            "a note that agrees with the capability changes nothing"
        );
        for k in [
            StampKind::NoStamp,
            StampKind::HostRecv,
            StampKind::SoftwareCounter,
            StampKind::Unknown(9),
        ] {
            assert_eq!(
                rx_stamp(&lr, dom, 1234, Some(k)).latch,
                LatchPoint::HostRecv,
                "{k:?} is not a hardware capture"
            );
        }
    }

    /// The evidence for `stamp_kind = 3` has to be readable. These six counters were on the wire
    /// from the Waveshare and the parser stopped at 32 bytes, so nothing decoded them.
    #[test]
    fn the_v3_stats_tail_is_parsed_and_older_layouts_still_are() {
        let mut p = vec![0u8; 44];
        p[32..34].copy_from_slice(&1234u16.to_be_bytes()); // hw_stamped
        p[34..36].copy_from_slice(&12u16.to_be_bytes()); // hw_stamp_sw
        p[36..38].copy_from_slice(&5u16.to_be_bytes()); // hw_stamp_ambig
        p[38..40].copy_from_slice(&0u16.to_be_bytes()); // hw_stamp_over
        p[40..42].copy_from_slice(&7u16.to_be_bytes()); // hw_stamp_lat_us
        p[42..44].copy_from_slice(&1u16.to_be_bytes()); // clock_skew_ms
        let v3 = NdnStats::parse(&p).unwrap();
        assert_eq!(v3.hw_stamped, Some(1234));
        assert_eq!(v3.hw_stamp_sw, Some(12));
        assert_eq!(v3.hw_stamp_ambig, Some(5));
        assert_eq!(v3.hw_stamp_over, Some(0));
        assert_eq!(v3.hw_stamp_lat_us, Some(7));
        assert_eq!(v3.clock_skew_ms, Some(1));

        // A v2 node reports None rather than a fabricated 0 — "not reported" is not "it never
        // happened", which is the whole reason these are Option.
        let v2 = NdnStats::parse(&p[..32]).unwrap();
        assert_eq!(v2.chip_rx, Some(0));
        assert_eq!(v2.hw_stamped, None);
        assert_eq!(v2.hw_stamp_ambig, None);
        let v1 = NdnStats::parse(&p[..24]).unwrap();
        assert_eq!(v1.chip_rx, None);
        assert_eq!(v1.hw_stamped, None);
        // A longer reply than we know is still accepted, excess ignored.
        let mut longer = p.clone();
        longer.extend_from_slice(&[0xAA; 8]);
        assert_eq!(NdnStats::parse(&longer).unwrap().hw_stamped, Some(1234));
        assert!(NdnStats::parse(&p[..23]).is_none());
    }

    /// **The measured 62.5 ns tick must survive the latch-point clamp.**
    ///
    /// `LinkStamp::new` raises `precision_ns` to the latch point's floor, and `MacDone`'s floor is
    /// the 1 µs of an 802.11 TSF register. Stamping this bearer `MacDone` published the LR2021's
    /// hardware capture as 1 000 ns — 16x worse than the hardware, and in disagreement with the
    /// 63 ns `time_sources` advertises for the very same clock. `RadioCapture` (floor 10 ns) is the
    /// latch point that actually describes a radio peripheral's edge capture.
    ///
    /// Both directions are pinned: the per-frame stamp AND the advertisement, because the bug this
    /// replaces was precisely the two of them disagreeing.
    #[test]
    fn a_hardware_capture_publishes_its_measured_precision_not_the_tsft_floor() {
        let dom = ClockDomainId(0x1234_5678);
        let lr = RadioKindHint::Lr2021Flrc.profile();
        let s = rx_stamp(&lr, dom, 16_625_857, None);
        assert_eq!(
            s.latch,
            LatchPoint::RadioCapture,
            "a device hardware capture is not an 802.11 MAC completion"
        );
        assert_eq!(
            s.precision_ns, 63,
            "the MEASURED 16 MHz tick survives; under MacDone this was clamped to 1000"
        );
        // The guard is still a guard: RadioCapture's floor is 10 ns, not 1 ns.
        assert!(LatchPoint::RadioCapture.precision_floor_ns() == 10);
        // A software counter is still refused the device domain entirely.
        let ws = NodeProfile::legacy_sx1262();
        assert_eq!(rx_stamp(&ws, dom, 1234, None).latch, LatchPoint::HostRecv);
    }

    /// The same `Fake` the fleet's `time_sources` is built from: a node profile decides the LATCH,
    /// a [`ClockReference`] decides what the counter RUNS ON, and both are varied independently.
    struct Fake(NodeProfile, ClockDomainId, ClockReference);
    impl RadioTime for Fake {
        fn time_sources(&self) -> Vec<RadioTimeSource> {
            let mut v = Vec::new();
            if self.0.stamp_kind == StampKind::HardwareFreeRun
                && let Some(t) = self.0.tick_ns()
            {
                v.push(RadioTimeSource {
                    kind: RadioClockKind::FreeRunRxStamp,
                    domain: self.1,
                    latch: LatchPoint::RadioCapture,
                    precision_ns: t,
                    tick_ns: t,
                    monotonic: true,
                    read_now: self.0.has_readable_clock(),
                    reference: self.2,
                });
            }
            v.push(RadioTimeSource::host_recv(HOST_CLOCK_DOMAIN));
            v
        }
    }

    /// ★ **The predicate, as a matrix.** A hardware latch alone used to grant common view; it takes
    /// the latch AND a reference that holds a rate. Every row below was a `true` before this change
    /// except the first, and the fleet contains a live example of each.
    #[test]
    fn common_view_needs_the_latch_point_and_the_reference() {
        use ndn_radio_hal::FaceTimeProfile;

        let dom = ClockDomainId(9);
        let hw_node = RadioKindHint::Lr2021Flrc.profile(); // stamp_kind 3, 16 MHz
        let derive = |p: NodeProfile, r: ClockReference| {
            FaceTimeProfile::derive(&Fake(p, dom, r), TxDiscipline::BestEffort)
        };

        // hardware latch + crystal => TRUE. Two LR2021s on HFXO: 0.81-1.86 us of common view, flat
        // against the fit span.
        let f = derive(hw_node, ClockReference::crystal());
        assert!(f.can_common_view, "a hardware latch on a crystal earns it");
        assert!(f.hw_rx_stamp);
        assert_eq!(f.stamp_precision_ns, Some(63));
        assert_eq!(f.best_clock, Some(RadioClockKind::FreeRunRxStamp));

        // hardware latch + RC => FALSE. Two Waveshares on the 8 MHz HSI: the SAME latch point and
        // the same 95/95 hardware stamps, 10-20 us of common view GROWING with the fit span.
        let f = derive(hw_node, ClockReference::rc_oscillator());
        assert!(
            !f.can_common_view,
            "an RC reference cannot hold a common view"
        );
        assert!(
            f.hw_rx_stamp,
            "and the latch fact survives — it really does stamp in hardware"
        );
        assert_eq!(f.best_clock, Some(RadioClockKind::FreeRunRxStamp));

        // hardware latch + unknown => FALSE. Any node that will not answer CMD_GET_CLOCK_REF.
        let f = derive(hw_node, ClockReference::unknown());
        assert!(!f.can_common_view, "unknown must not silently qualify");
        assert!(f.hw_rx_stamp);
        assert_eq!(
            f.clock_reference.map(|r| r.kind),
            Some(ClockReferenceKind::Unknown)
        );

        // software stamp (or none) + crystal => FALSE. The reference cannot rescue the latch: two
        // nodes would be differencing their firmware main-loop latencies.
        for kind in [
            StampKind::NoStamp,
            StampKind::HostRecv,
            StampKind::SoftwareCounter,
        ] {
            let mut p = hw_node;
            p.stamp_kind = kind;
            let f = derive(p, ClockReference::crystal());
            assert!(!f.can_common_view, "{kind:?} must not claim common view");
            assert!(!f.hw_rx_stamp, "{kind:?} is not a hardware latch");
            assert_eq!(f.best_clock, Some(RadioClockKind::HostRecv));
        }
    }

    /// The wire answer, decoded. `ref_class` is taken at face value, the accuracy sentinel means
    /// "no measurement" rather than "zero ppm", and a short frame is refused.
    #[test]
    fn evt_clock_ref_decodes_what_the_node_said() {
        let unk = CLOCK_ACCURACY_UNKNOWN.to_be_bytes();

        let xtal = parse_clock_ref(&[CLOCK_REF_XTAL, unk[0], unk[1]]).unwrap();
        assert_eq!(xtal.kind, ClockReferenceKind::Crystal);
        assert_eq!(xtal.measured, None, "the sentinel is NOT a measurement");
        assert!(xtal.holds_rate());

        let rc = parse_clock_ref(&[CLOCK_REF_RC, unk[0], unk[1]]).unwrap();
        assert_eq!(rc.kind, ClockReferenceKind::RcOscillator);
        assert!(!rc.holds_rate());

        // A node that looked and could not tell stays unknown — the host does not improve on it.
        let none = parse_clock_ref(&[CLOCK_REF_UNKNOWN, unk[0], unk[1]]).unwrap();
        assert_eq!(none.kind, ClockReferenceKind::Unknown);
        assert!(!none.holds_rate());

        // A class this host does not speak is not evidence of a good reference.
        assert_eq!(
            parse_clock_ref(&[0x7f, unk[0], unk[1]]).unwrap().kind,
            ClockReferenceKind::Unknown
        );

        // A real self-reported figure rides along, labelled as the node's own claim.
        let m = parse_clock_ref(&[CLOCK_REF_XTAL, 0x00, 0x14])
            .unwrap()
            .measured
            .unwrap();
        assert_eq!(m.ppm, 20.0);
        assert_eq!(m.witness, RateWitness::NodeReported);

        // A truncated capability is not a capability.
        assert_eq!(parse_clock_ref(&[CLOCK_REF_XTAL, 0x00]), None);
        assert_eq!(parse_clock_ref(&[]), None);
    }

    /// ★ **The fallback withholds unless the DEVICE supplied the deciding fact.** A profile the host
    /// pinned (`learned == false`) can never earn common view from this side; the only promotion in
    /// the table rides on a frame the node actually sent.
    #[test]
    fn the_host_side_assumption_may_only_demote_unless_the_node_spoke() {
        // Nothing the host pinned may hold a rate — including an LR2021, whose pre-v2 image is
        // exactly the RC build that cannot be told apart any other way.
        for kind in [
            LoraRadioKind::Sx1262,
            LoraRadioKind::Sx1276,
            LoraRadioKind::Lr2021,
            LoraRadioKind::Unknown(0x5a),
        ] {
            let mut p = NodeProfile::legacy_sx1262();
            p.radio_kind = kind;
            p.learned = false;
            assert!(
                !assumed_clock_reference(&p).holds_rate(),
                "{kind:?}: a host-pinned profile must never grant common view"
            );
        }
        // The Waveshare's demotion is not merely "unknown": the RC is named, with its measurement.
        let mut ws_p = NodeProfile::legacy_sx1262();
        ws_p.learned = true; // even when it DID answer: the SX1262 row never promotes
        let ws = assumed_clock_reference(&ws_p);
        assert_eq!(ws.kind, ClockReferenceKind::RcOscillator);
        assert_eq!(ws.measured.map(|m| m.ppm), Some(-3100.0));
        assert_eq!(ws.measured.map(|m| m.witness), Some(RateWitness::PeerUnit));
        // And the node's own answer overrides it in the direction the host would not go alone.
        let unk = CLOCK_ACCURACY_UNKNOWN.to_be_bytes();
        assert!(
            parse_clock_ref(&[CLOCK_REF_XTAL, unk[0], unk[1]])
                .unwrap()
                .holds_rate(),
            "a reflashed Waveshare earns it back from its own mouth"
        );
    }

    /// ★ **LEARNED is not the same node as PINNED, and the LR2021 is where that bites.** An
    /// `EVT_CAP` from an LR2021 can only have come from a post-1283b7e build, and that build forces
    /// `HfclkSource::ExternalXtal` (the pre-v2 image speaks 7E-A5 v1 and cannot emit an `EVT_CAP` at
    /// all) — so the record's origin, not its `radio_kind`, is what decides the reference. The
    /// regression this pins: `assumed_clock_reference` used to take `radio_kind` alone, and the
    /// fleet's only part with a this-session-measured common view reported `can_common_view = false`.
    #[test]
    fn a_learned_lr2021_cap_is_itself_the_crystal_evidence() {
        // Pinned from the host (open_as / pre-v2 firmware / silent port): still unknown.
        let pinned = RadioKindHint::Lr2021Flrc.profile();
        assert!(
            !pinned.learned,
            "the pinned LR2021 profile is a host fallback"
        );
        let r = assumed_clock_reference(&pinned);
        assert_eq!(r.kind, ClockReferenceKind::Unknown);
        assert!(!r.holds_rate(), "a node that never spoke earns nothing");
        assert_eq!(r.measured, None);

        // The SAME board, once it answers CMD_GET_CAP: the EVT_CAP is the wire fact.
        let mut spoke = pinned;
        spoke.learned = true;
        let r = assumed_clock_reference(&spoke);
        assert_eq!(r.kind, ClockReferenceKind::Crystal);
        assert!(r.holds_rate());
        // The CLASS is claimed and the RATE is not: the +16.7 ppm on record is one session's
        // comparison this host cannot stand behind. That is also what the node itself now says, so
        // the fallback and a reflashed board's own answer are the SAME value.
        assert_eq!(r.measured, None, "class provable, number not");
        let unk = CLOCK_ACCURACY_UNKNOWN.to_be_bytes();
        assert_eq!(
            parse_clock_ref(&[CLOCK_REF_XTAL, unk[0], unk[1]]),
            Some(r),
            "a reflashed LR2021 answers exactly what this fallback assumed"
        );

        // ...and it is a real capability, not just a field: both halves of the predicate land on the
        // one source, so the part that MEASURED 0.81/1.55/1.86 us of common view reports it.
        use ndn_radio_hal::FaceTimeProfile;
        let f = FaceTimeProfile::derive(
            &Fake(spoke, ClockDomainId(11), assumed_clock_reference(&spoke)),
            TxDiscipline::BestEffort,
        );
        assert!(f.hw_rx_stamp, "16 MHz DPPI capture, MEASURED 62.5 ns");
        assert!(
            f.can_common_view,
            "a learned LR2021 must not lose the capability it measured"
        );
        // And the pinned one does not, on the same latch, from the same code path.
        let f = FaceTimeProfile::derive(
            &Fake(pinned, ClockDomainId(11), assumed_clock_reference(&pinned)),
            TxDiscipline::BestEffort,
        );
        assert!(f.hw_rx_stamp, "the latch fact is unchanged by who said it");
        assert!(!f.can_common_view, "an unstated reference earns nothing");
    }

    /// The wire answer still outranks the fallback in BOTH directions — including down. A board
    /// that answers `CLOCK_REF_RC` is an RC node however good its `EVT_CAP` looked.
    #[test]
    fn the_nodes_own_answer_outranks_the_learned_inference() {
        let mut spoke = RadioKindHint::Lr2021Flrc.profile();
        spoke.learned = true;
        assert!(assumed_clock_reference(&spoke).holds_rate());
        let unk = CLOCK_ACCURACY_UNKNOWN.to_be_bytes();
        // `open_inner` parses this first and only falls back when it is `None`.
        let said = parse_clock_ref(&[CLOCK_REF_RC, unk[0], unk[1]]).unwrap();
        assert_eq!(said.kind, ClockReferenceKind::RcOscillator);
        assert!(!said.holds_rate(), "the node's own word demotes it again");
    }

    // ── scheduling ────────────────────────────────────────────────────────────────────────────

    #[test]
    fn schedules_tx_needs_both_a_granularity_and_the_opcode() {
        let mut p = RadioKindHint::Lr2021Flrc.profile();
        assert!(!p.schedules_tx(), "m6_bridge has neither");

        // A granularity with no CMD_TX_AT is the exact trap FrameIo::schedules_tx warns about: the
        // caller would skip its software gate and inject_after would fall through to inject-now.
        p.sched_gran_ns = 62_500;
        assert!(
            !p.schedules_tx(),
            "a declared granularity is not an actuator"
        );

        p.cmd_bitmap |= cmd_bits(&[CMD_TX_AT]);
        assert!(p.schedules_tx());

        // And the discipline is the SAME predicate rather than a second copy of it — asserted
        // through `NodeProfile::tx_discipline`, which is what `RadioKnobs::tx_discipline` returns.
        assert_eq!(
            p.tx_discipline(),
            TxDiscipline::ScheduledAt {
                granularity_ns: 62_500
            }
        );
    }

    // ── the airtime-derived transmit timeout ──────────────────────────────────────────────────

    #[test]
    fn airtime_matches_the_semtech_formula() {
        // SF7/125 kHz/CR 4-5, 48-byte payload, 8-symbol preamble: ~97 ms.
        let t = lora_airtime_ms(7, 125, 1, 48, 8);
        assert!((90..=105).contains(&t), "SF7 48B was {t} ms");
        // SF12/125 kHz/CR 4-5, a full 240-byte frame: ~8.5 s — nearly 3x the old fixed 3 s timeout.
        let t12 = lora_airtime_ms(12, 125, 1, 240, 8);
        assert!((8_000..=9_200).contains(&t12), "SF12 240B was {t12} ms");
        assert!(
            Duration::from_millis(t12) > TXDONE_MIN_TIMEOUT,
            "this is the bug: a fixed 3 s timeout abandons an SF12 frame mid-flight"
        );
        // Monotone in every axis that costs airtime.
        assert!(lora_airtime_ms(12, 125, 1, 240, 8) > lora_airtime_ms(11, 125, 1, 240, 8));
        assert!(lora_airtime_ms(9, 125, 1, 240, 8) > lora_airtime_ms(9, 250, 1, 240, 8));
        assert!(lora_airtime_ms(9, 125, 4, 240, 8) > lora_airtime_ms(9, 125, 1, 240, 8));
        assert!(lora_airtime_ms(9, 125, 1, 240, 64) > lora_airtime_ms(9, 125, 1, 240, 8));
    }

    #[test]
    fn the_lbt_budget_comes_from_the_firmwares_own_bound() {
        // attempts x cw x 2^backoff, the loop bound the firmware enforces — not a magic 5 s.
        assert_eq!(
            LbtCfg::default().worst_case(),
            Duration::from_millis(6 * 20 * 16)
        );
    }

    // ── the bandwidth code space (a live Heltec bug) ──────────────────────────────────────────

    /// The `CMD_SET_MOD[1]` byte means different things on different nodes and **nothing on the wire
    /// says which**. The Waveshare firmware hands the byte straight to the SX1262
    /// (`SetModulationParams` LoRa BW codes `0x04/0x05/0x06`); the Heltec firmware decodes `0/1/2`
    /// and maps anything else to 125 kHz. Sending SX1262 codes to the Heltec therefore pins it to
    /// 125 kHz forever while `EVT_INFO` echoes the value the host asked for — a silent, invisible
    /// mismatch. This table is the pin.
    #[test]
    fn bandwidth_codes_are_per_node_and_pinned() {
        let table = [
            // (node kind,                  host code, wire byte)
            (LoraRadioKind::Sx1262, 0u8, 0x04u8),
            (LoraRadioKind::Sx1262, 1, 0x05),
            (LoraRadioKind::Sx1262, 2, 0x06),
            (LoraRadioKind::Sx1276, 0, 0x00),
            (LoraRadioKind::Sx1276, 1, 0x01),
            (LoraRadioKind::Sx1276, 2, 0x02),
            (LoraRadioKind::Lr2021, 0, 0x04),
            (LoraRadioKind::Unknown(9), 2, 0x06),
        ];
        for (kind, host, wire) in table {
            assert_eq!(
                bw_to_fw(kind, host),
                wire,
                "{kind:?} host bw code {host} must go on the wire as {wire:#04x}"
            );
        }
        // Out-of-range host codes saturate at 500 kHz rather than wrapping into a foreign register.
        assert_eq!(bw_to_fw(LoraRadioKind::Sx1262, 7), 0x06);
        assert_eq!(bw_to_fw(LoraRadioKind::Sx1276, 7), 0x02);
        // And the kHz table the airtime formula reads is the same three widths.
        assert_eq!(BW_KHZ, [125, 250, 500]);
    }

    /// **The `sf` byte is not portable.** A node with `sf_min == 0` reuses the `CMD_SET_MOD`
    /// positions for its own modulation: on the LR2021 `[0]` is an FLRC bitrate rung and `[2]` an
    /// FLRC coding rate (`m6_bridge.rs::bitrate_of_code`/`cr_of_code`). This host's default `sf = 7`
    /// decodes there as rung 7 = `FlrcBitrate::Br0260`, ten times slower than the firmware's
    /// `Br2600` default — so an unguarded `configure()` would re-modulate a verified-live link at
    /// open() and report success. The gate is `has_spreading_factor`, and it covers the whole
    /// triple because `cr` is re-keyed on such a node too.
    #[test]
    fn the_set_mod_triple_is_gated_on_a_real_spreading_factor() {
        let flrc = RadioKindHint::Lr2021Flrc.profile();
        assert!(!flrc.has_spreading_factor());
        assert_eq!(flrc.clamp_sf(7), None, "no SF is not SF 7");

        for lora in [NodeProfile::legacy_sx1262(), NodeProfile::heltec_sx1276()] {
            assert!(lora.has_spreading_factor());
            assert_eq!(lora.clamp_sf(7), Some(7));
        }

        // The live v2 LR2021 DOES advertise CMD_SET_MOD, so `supports()` alone is not the gate —
        // that is exactly why the bug was reachable.
        let live_flrc = NodeProfile::parse(&{
            let mut c = flrc.to_cap_payload();
            c[19..23].copy_from_slice(&(flrc.cmd_bitmap | 1 << CMD_SET_MOD).to_be_bytes());
            c
        })
        .expect("29 bytes parse");
        assert!(live_flrc.supports(CMD_SET_MOD));
        assert!(!live_flrc.has_spreading_factor());
    }

    // ── capability from the profile ───────────────────────────────────────────────────────────

    /// **The declared index ceiling must be the ceiling the actuator enforces.** Both LoRa
    /// constructors fill `max_tx_power` with 63, a chip TXAGC index scale none of these nodes has —
    /// `CMD_SET_PWR` carries an i8 dBm. With 63 declared, cognition's `decide_power` hands
    /// `63 − backoff_idx` to `set_tx_power`, which clamps it straight back to the PA maximum, so the
    /// back-off lever is inert for any realistic margin. `capability()` derives the ceiling through
    /// `clamp_dbm` instead — the same function the actuator calls, so the two cannot disagree.
    #[test]
    fn the_index_power_ceiling_is_the_one_the_actuator_enforces() {
        for prof in [
            NodeProfile::legacy_sx1262(),
            NodeProfile::heltec_sx1276(),
            RadioKindHint::Lr2021Flrc.profile(),
        ] {
            let ceiling = prof.clamp_dbm(i8::MAX).max(0) as u8;
            assert!(
                ceiling > 0 && ceiling < 63,
                "{prof:?} would inherit a fabricated 63-step scale"
            );
            // The ceiling is a fixed point of the clamp: asking for it applies it exactly.
            assert_eq!(prof.clamp_dbm(ceiling as i8), ceiling as i8);
            // And a 6 dB back-off from it lands 6 dB down rather than clamping back up.
            assert_eq!(
                prof.clamp_dbm(ceiling as i8 - 6),
                ceiling as i8 - 6,
                "{prof:?}: the back-off must bite"
            );
        }
    }

    #[test]
    fn capability_is_built_from_the_profile_not_from_the_lora_preset() {
        // Through `capability_from`, which IS `RadioProfile::capability` — not a re-statement of it.
        let lr = RadioKindHint::Lr2021Flrc.profile();
        let cap = capability_from(&lr, 65);
        assert_eq!(cap.rate, RateCapability::None, "FLRC has no SF span");
        assert_eq!(cap.sf_range(), None);
        assert_eq!(
            cap.max_payload, 47,
            "the REAL cap: FRAME_LEN 48 minus the in-frame length byte — not the preset's 256, and \
             not the 48 this fallback claimed before the board reported for itself"
        );
        assert_eq!(
            cap.channels,
            vec![65],
            "a fixed-carrier node has one channel"
        );
        assert_eq!(cap.tx_power_dbm, None, "no declared range => no dBm claim");
        assert_eq!(
            cap.duty_cycle_max, 1.0,
            "FCC 15.247 has no duty fraction; the preset's ETSI 0.01 is wrong here"
        );
        // The PRE-v2 m6_bridge has no CMD_SET_FREQ, so it reports no retune cost — see
        // `retune_is_measured_per_modem_and_gated_on_the_actuator`.
        assert_eq!(
            cap.retune_us, None,
            "a node that cannot retune reports no retune cost"
        );
        assert_eq!(
            cap.can_hop(20_000),
            None,
            "and can_hop must answer 'cannot say', which the HAL documents a planner treats as \
             'do not hop'"
        );

        // The Waveshare, by contrast, keeps a real SF span, a real dBm range, and a real band plan.
        let ws = NodeProfile::legacy_sx1262();
        let cap = capability_from(&ws, 65);
        assert_eq!(cap.sf_range(), Some((7, 12)));
        // The legacy profile's honest end-to-end cap, not `RadioCapability::lora`'s 256 and not the
        // 240 this firmware accepts on TX — its RX truncates at 64. See `LEGACY_RX_TRUNCATION_CAP`.
        assert_eq!(cap.max_payload, LEGACY_RX_TRUNCATION_CAP as usize);
        assert_eq!(cap.tx_power_dbm, Some(DbmRange::new(10, 22)));
        assert_eq!(cap.channels.first().copied(), Some(52), "902 MHz");
        assert_eq!(cap.channels.last().copied(), Some(78), "928 MHz band edge");
        assert_eq!(
            cap.retune_us,
            Some(82_810),
            "and this one CAN retune — at the price of a full image calibration"
        );
    }

    #[test]
    fn duty_cycle_follows_the_band_not_the_family() {
        let p = NodeProfile::legacy_sx1262();
        assert_eq!(p.duty_cycle_max(868_000_000), 0.01, "ETSI EN 300 220: 1%");
        assert_eq!(
            p.duty_cycle_max(915_000_000),
            1.0,
            "FCC 15.247: no duty fraction"
        );
        assert_eq!(p.duty_cycle_max(928_000_000), 1.0);
    }

    #[test]
    fn power_clamps_come_from_the_node() {
        let ws = NodeProfile::legacy_sx1262();
        assert_eq!(ws.clamp_dbm(30), 22);
        assert_eq!(ws.clamp_dbm(-5), 10);
        let heltec = RadioKindHint::HeltecSx1276.profile();
        assert_eq!(
            heltec.clamp_dbm(22),
            20,
            "the SX1276's PA_BOOST ceiling as the flashed board reports it (2..20), not the \
             SX1262's 22 and not the 17 this fallback guessed from the wrong PA path"
        );
        assert_eq!(
            heltec.clamp_dbm(0),
            2,
            "and the floor is the node's, not the legacy 10"
        );
        // An undeclared range refuses rather than inventing one, but the index knob still clamps
        // conservatively so it can send a byte at all.
        let lr = RadioKindHint::Lr2021Flrc.profile();
        assert_eq!(lr.dbm_range(), None);
        assert_eq!(lr.clamp_dbm(99), LEGACY_PWR_MAX_DBM);
    }

    #[test]
    fn frequency_clamps_and_channel_lists_come_from_the_node() {
        let ws = NodeProfile::legacy_sx1262();
        assert_eq!(ws.clamp_hz(868_000_000), 902_000_000, "below the US span");
        assert_eq!(ws.clamp_hz(915_000_000), 915_000_000);
        assert_eq!(ws.channels(65).len(), 27, "902..=928 MHz inclusive");
        // A node with a degenerate span reports the one channel it is known-good on.
        let lr = RadioKindHint::Lr2021Flrc.profile();
        assert_eq!(lr.channels(65), vec![65]);
    }

    #[test]
    fn the_fnv_keyspace_stays_byte_identical_to_the_firmwares() {
        // Pinned vectors: if this changes, the host and the node disagree about which names a
        // filter/relay/CS entry covers, and the failure is silent.
        assert_eq!(name_hash(b""), 0xcbf2_9ce4_8422_2325);
        assert_eq!(name_hash(b"a"), 0xaf63_dc4c_8601_ec8c);
        // And the payload packer is 8 big-endian bytes per prefix.
        let p = hash_payload(&[b"ndn/x".as_slice(), b"ndn/y".as_slice()]);
        assert_eq!(p.len(), 16);
        assert_eq!(&p[..8], &name_hash(b"ndn/x").to_be_bytes());
    }
    // ── the MEASURED retune cost ──────────────────────────────────────────────────────────────

    /// **The three nodes differ by 29×, so no family-wide number could have been right.** Each
    /// constant is a host-observed `CMD_SET_FREQ` -> `EVT_INFO` round trip (n = 8-10, alternating
    /// carriers so nothing is a no-op, sub-millisecond spreads). This is what makes `can_hop`
    /// answerable on this bearer at all.
    #[test]
    fn retune_is_measured_per_modem_and_gated_on_the_actuator() {
        // The two nodes whose pinned fallback implements CMD_SET_FREQ.
        let heltec = RadioKindHint::HeltecSx1276.profile();
        assert_eq!(heltec.retune_us(), Some(5_597));
        let ws = NodeProfile::legacy_sx1262();
        assert_eq!(ws.retune_us(), Some(82_810));

        // The LR2021, once its v2 firmware advertises the opcode its pre-v2 build lacked.
        let mut lr = RadioKindHint::Lr2021Flrc.profile();
        assert_eq!(
            lr.retune_us(),
            None,
            "no CMD_SET_FREQ means it cannot hop at any dwell — a cost figure would say the \
             opposite for a long enough one"
        );
        lr.cmd_bitmap |= cmd_bits(&[CMD_SET_FREQ]);
        assert_eq!(lr.retune_us(), Some(52_798));

        // ★ The whole point, asked through the capability the stack actually consumes: at one dwell
        // the fleet does not answer with one voice.
        let hop = |p: &NodeProfile| capability_from(p, 65).can_hop(100_000);
        assert_eq!(hop(&heltec), Some(true), "5.6 ms x4 fits a 100 ms dwell");
        assert_eq!(hop(&lr), Some(false), "52.8 ms x4 does not");
        assert_eq!(
            hop(&ws),
            Some(false),
            "82.8 ms of a 100 ms dwell leaves nothing to use the channel with"
        );
        // And the cost line a hop plan is charged. Re-measured after the firmware learned to skip the
        // image calibration (160 866 -> 82 810 µs), so the Waveshare now spends ~83% of a 100 ms dwell
        // tuning rather than more than all of it. Still not hop-capable here — the value of the number
        // is that it says so quantitatively instead of by a boolean.
        let overhead = capability_from(&ws, 65).retune_overhead(100_000).unwrap();
        assert!(
            (overhead - 0.828_10).abs() < 1e-4,
            "overhead was {overhead}"
        );

        // An unknown modem code stays unmeasured rather than inheriting a neighbour's number.
        let mut unknown = ws;
        unknown.radio_kind = LoraRadioKind::from_code(9);
        assert_eq!(unknown.retune_us(), None);
        assert_eq!(hop(&unknown), None);
        // ★ As does the LR2021's LoRa mode — and in v3 that is said by the PHY, not by a second
        // "radio kind". 52 798 µs was measured with the FLRC packet engine loaded; the retune
        // sequence runs through whatever engine is loaded, so the same chip in LoRa is untimed and
        // must report so rather than wear the FLRC number.
        let mut lr_lora = lr;
        lr_lora.phy_current = PhyMode::Lora;
        assert_eq!(lr_lora.radio_kind, LoraRadioKind::Lr2021, "same PART");
        assert_eq!(lr_lora.retune_us(), None, "different PHY, unmeasured");
    }

    // ── the frame budget ──────────────────────────────────────────────────────────────────────

    /// `MAX_LORA_PAYLOAD` sat at 240 while both LoRa nodes carry 247 end to end — 7 bytes a frame
    /// discarded, and the PHY's MTU pinned under the real cap because the budget takes the smaller
    /// side. Raised to the number the boards report; the smaller side must still win.
    #[test]
    fn the_frame_budget_takes_the_smaller_side() {
        assert_eq!(
            MAX_LORA_PAYLOAD, 247,
            "both LoRa nodes' EVT_CAP max_payload"
        );
        assert_eq!(
            LORA_NODE_RX_MAX as usize, MAX_LORA_PAYLOAD,
            "one constant, not two"
        );
        // 7E-A5 frames a single-byte length, and CMD_TX_AT prepends 4 delay bytes: the largest
        // payload must still fit a scheduled transmit.
        assert!(MAX_LORA_PAYLOAD + 4 <= 255, "247 + delay word still frames");

        // A node at the ceiling gets the ceiling…
        assert_eq!(RadioKindHint::HeltecSx1276.profile().frame_budget(), 247);
        // …a smaller node wins…
        assert_eq!(
            RadioKindHint::Lr2021Flrc.profile().frame_budget(),
            47,
            "the FLRC frame is 47 bytes and nothing may raise that"
        );
        assert_eq!(
            NodeProfile::legacy_sx1262().frame_budget(),
            LEGACY_RX_TRUNCATION_CAP as usize,
            "the un-reflashed dongle truncates RX at 64 and says nothing"
        );
        // …and so does the host, if a node ever over-claims.
        let mut liar = RadioKindHint::HeltecSx1276.profile();
        liar.max_payload = 4096;
        assert_eq!(liar.frame_budget(), MAX_LORA_PAYLOAD);
    }

    // ── EVT_STATS: 24 bytes or 32 ─────────────────────────────────────────────────────────────

    /// The Waveshare appends its modem's own counters; the other two do not. Both must parse, and
    /// which one arrived must be visible rather than guessed from the node's identity.
    #[test]
    fn evt_stats_parses_both_the_24_and_the_32_byte_layout() {
        let mut v1 = Vec::new();
        v1.extend_from_slice(&7u32.to_be_bytes()); //   [0..4]   rx
        v1.extend_from_slice(&3u32.to_be_bytes()); //   [4..8]   filtered
        v1.extend_from_slice(&2u32.to_be_bytes()); //   [8..12]  deduped
        v1.extend_from_slice(&1u32.to_be_bytes()); //   [12..16] served
        v1.extend_from_slice(&5u32.to_be_bytes()); //   [16..20] relayed
        v1.extend_from_slice(&11u16.to_be_bytes()); //  [20..22] cad_busy
        v1.extend_from_slice(&4u16.to_be_bytes()); //   [22..24] defer
        assert_eq!(v1.len(), 24);

        let a = NdnStats::parse(&v1).expect("24 bytes is a complete v1 reply");
        assert_eq!(
            (a.rx, a.filtered, a.deduped, a.served, a.relayed),
            (7, 3, 2, 1, 5)
        );
        assert_eq!((a.cad_busy, a.defer), (11, 4));
        assert_eq!(
            a.chip_rx, None,
            "a 24-byte node reports no PHY counters — not zero of them"
        );
        assert_eq!(a.phy_counters(), None);

        // The Waveshare's v2 tail, at the offsets its README pins.
        let mut v2 = v1.clone();
        v2.extend_from_slice(&900u16.to_be_bytes()); // [24..26] chip_rx
        v2.extend_from_slice(&80u16.to_be_bytes()); //  [26..28] chip_crc_err
        v2.extend_from_slice(&20u16.to_be_bytes()); //  [28..30] chip_hdr_err
        v2.extend_from_slice(&0u16.to_be_bytes()); //   [30..32] rx_trunc
        assert_eq!(v2.len(), 32);

        let b = NdnStats::parse(&v2).expect("32 bytes parse");
        // Bytes 0..24 are byte-identical to v1 — the tail must not shift anything.
        assert_eq!(
            (b.rx, b.filtered, b.deduped, b.served, b.relayed),
            (7, 3, 2, 1, 5)
        );
        assert_eq!((b.cad_busy, b.defer), (11, 4));
        assert_eq!(b.chip_rx, Some(900));
        assert_eq!(b.chip_crc_err, Some(80));
        assert_eq!(b.chip_hdr_err, Some(20));
        assert_eq!(b.rx_trunc, Some(0));
        // (ok, err): err sums the two ways a reception the PHY BEGAN can fail.
        assert_eq!(b.phy_counters(), Some((900, 100)));

        // Shorter than 24 is not a counter block; a partial tail is dropped rather than half-read;
        // longer than 32 is a future firmware appending a counter and must not break this host.
        assert_eq!(NdnStats::parse(&v1[..23]), None);
        assert_eq!(NdnStats::parse(&v2[..29]).unwrap().chip_rx, None);
        let mut v3 = v2.clone();
        v3.extend_from_slice(&[0xAB, 0xCD]);
        assert_eq!(
            NdnStats::parse(&v3).unwrap().phy_counters(),
            Some((900, 100))
        );
    }

    // ── scheduled TX lights up the whole seam at once ─────────────────────────────────────────

    /// **The day a firmware starts reporting `sched_gran_ns > 0`, four things must move together.**
    /// A node that declares the discipline without the seam is worse than one that declares nothing
    /// (`FrameIo::schedules_tx`): the caller skips its own software gate believing the hardware will
    /// place the frame, and the fall-through transmits immediately, ungated. All four read the same
    /// predicate here, so they cannot come apart.
    #[test]
    fn a_non_zero_granularity_lights_up_every_scheduling_surface() {
        // A synthetic node: an LR2021 whose firmware has grown CMD_TX_AT on an MCU timer.
        let mut p = RadioKindHint::Lr2021Flrc.profile();
        p.cmd_bitmap |= cmd_bits(&[CMD_TX_AT, CMD_READ_CLOCK]);
        p.sched_gran_ns = 1_000; // 1 µs

        assert!(p.schedules_tx(), "1: FrameIo::schedules_tx");
        assert!(
            p.schedules_after(250_000),
            "2: inject_after places the frame"
        );
        assert!(
            !p.schedules_after(0),
            "…but delay 0 is inject-now on every bearer"
        );
        assert!(p.schedules_at_clock(true), "3: inject_at_clock, own domain");
        assert!(
            !p.schedules_at_clock(false),
            "a tick in another domain is not a time on this radio"
        );
        assert_eq!(
            p.tx_discipline(), // 4
            TxDiscipline::ScheduledAt {
                granularity_ns: 1_000
            }
        );

        // Reading the clock is not optional for the ABSOLUTE path: without CMD_READ_CLOCK there is
        // no way to turn a target tick into a delay, so that one surface goes dark on its own.
        let mut no_clock = p;
        no_clock.cmd_bitmap &= !(1u32 << CMD_READ_CLOCK);
        assert!(no_clock.schedules_tx() && no_clock.schedules_after(250_000));
        assert!(!no_clock.schedules_at_clock(true));

        // And the two ways to be inconsistent stay dark on all four.
        let mut label_only = p; // granularity, no actuator
        label_only.cmd_bitmap &= !(1u32 << CMD_TX_AT);
        let mut opcode_only = p; // actuator, no granularity
        opcode_only.sched_gran_ns = 0;
        for bad in [label_only, opcode_only] {
            assert!(!bad.schedules_tx());
            assert!(!bad.schedules_after(250_000));
            assert!(!bad.schedules_at_clock(true));
            assert_eq!(bad.tx_discipline(), TxDiscipline::BestEffort);
        }

        // The three flashed nodes today: all three firmwares report 0, so all four surfaces are off.
        for shipped in [
            NodeProfile::legacy_sx1262(),
            NodeProfile::heltec_sx1276(),
            NodeProfile::lr2021_flrc(),
        ] {
            assert_eq!(shipped.sched_gran_ns, 0);
            assert_eq!(shipped.tx_discipline(), TxDiscipline::BestEffort);
        }
    }

    // ── v3: modulation is a knob, not an identity ─────────────────────────────────────────────

    /// **The 34-byte v3 layout is a contract with three firmware repos**; pin it byte by byte from
    /// a hand-built payload rather than from the serialiser, which would only test itself.
    #[test]
    fn a_v3_evt_cap_carries_the_phy_set_and_the_phy_in_effect() {
        let mut p = Vec::new();
        p.push(3); // [0]    proto_ver = 3
        p.push(2); // [1]    radio_kind = the LR2021 PART (never "the LR2021 in FLRC")
        p.extend_from_slice(&2_400_000_000u32.to_be_bytes()); // [2..6]   freq_min
        p.extend_from_slice(&2_483_500_000u32.to_be_bytes()); // [6..10]  freq_max
        p.push(0); // [10]   pwr_min
        p.push(0); // [11]   pwr_max
        p.extend_from_slice(&16_000_000u32.to_be_bytes()); // [12..16] stamp_hz
        p.push(3); // [16]   stamp_kind = hardware free-running
        p.extend_from_slice(&47u16.to_be_bytes()); // [17..19] max_payload (the FLRC frame)
        p.extend_from_slice(
            &cmd_bits(&[
                CMD_TX,
                CMD_TX_AT,
                CMD_TX_AT_ABS,
                CMD_READ_CLOCK,
                CMD_SET_PHY,
            ])
            .to_be_bytes(),
        ); // [19..23]
        p.push(0); // [23]   sf_min — FLRC has none
        p.push(0); // [24]   sf_max
        p.extend_from_slice(&50_000u32.to_be_bytes()); // [25..29] sched_gran_ns (the MEASURED 50 µs)
        let modes = PhyModeSet::single(PhyMode::Lora)
            .with(PhyMode::Flrc)
            .with(PhyMode::Ble);
        p.extend_from_slice(&modes.bits().to_be_bytes()); // [29..33] phy_bitmap
        p.push(PhyMode::Flrc.code()); //                     [33]     phy_current
        assert_eq!(p.len(), CAP_LEN_V3, "a v3 EVT_CAP is exactly 34 bytes");

        let prof = NodeProfile::parse(&p).expect("34 bytes parse");
        assert_eq!(prof.proto_ver, 3);
        assert_eq!(
            prof.radio_kind,
            LoraRadioKind::Lr2021,
            "byte 1 names the PART now; v2's separate 'LR2021-LoRa' kind is retired"
        );
        assert_eq!(prof.phy_current, PhyMode::Flrc);
        assert_eq!(prof.phy_modes(), modes);
        assert!(
            prof.phy_agile(),
            "three modes AND CMD_SET_PHY: modulation is genuinely a knob here"
        );
        // A mode is only reachable if it is in the set.
        assert!(prof.phy_modes().contains(PhyMode::Ble));
        assert!(!prof.phy_modes().contains(PhyMode::ZWave));
        // Round-trips through the v3 serialiser, so an emulator and the parser agree.
        assert_eq!(&prof.to_cap_payload_v3()[..], &p[..]);

        // The actuator gate is the same rule as everywhere else in this file: a set without the
        // opcode that switches it is not a capability.
        let mut no_opcode = prof;
        no_opcode.cmd_bitmap &= !(1u32 << CMD_SET_PHY);
        assert!(no_opcode.phy_modes().is_agile());
        assert!(
            !no_opcode.phy_agile(),
            "three advertised modes and nothing that reaches them is not agility"
        );
    }

    /// **A v2 node must keep opening, and must land somewhere honest in the PHY model.** The whole
    /// fleet is not reflashed at once; a host that could only talk to v3 firmware would strand two
    /// of three boards.
    #[test]
    fn a_v2_evt_cap_still_parses_and_maps_into_the_phy_model() {
        for (kind_byte, part, mode) in [
            (0u8, LoraRadioKind::Sx1262, PhyMode::Lora),
            (1, LoraRadioKind::Sx1276, PhyMode::Lora),
            (2, LoraRadioKind::Lr2021, PhyMode::Flrc), // v2 "LR2021-FLRC"
            (3, LoraRadioKind::Lr2021, PhyMode::Lora), // v2 "LR2021-LoRa" — retired, still decodes
        ] {
            let mut cap = NodeProfile::legacy_sx1262().to_cap_payload();
            cap[0] = 2;
            cap[1] = kind_byte;
            let prof = NodeProfile::parse(&cap).expect("29 bytes still parse");
            assert_eq!(prof.proto_ver, 2);
            assert_eq!(prof.radio_kind, part, "radio_kind byte {kind_byte}");
            assert_eq!(
                prof.phy_current, mode,
                "the mode half of v2's conflated byte {kind_byte}"
            );
            assert_eq!(
                prof.phy_modes(),
                PhyModeSet::single(mode),
                "one entry: without CMD_SET_PHY no other mode is reachable"
            );
            assert!(!prof.phy_agile());
            assert_eq!(prof.hop_capability(), None, "and no hop plan can be sent");
        }
    }

    /// **A v2 `radio_kind` this host has never seen gets NO modulation, not a plausible one.**
    ///
    /// `Lora` used to be the catch-all, and it is an invention: nothing about an unknown part says
    /// it modulates like the three we know. The rest of the body is still true and is still read —
    /// the band, the PA range, the stamp and the opcode set were all stated by the node — so this is
    /// a refusal of one field, not of the capability.
    #[test]
    fn an_unknown_v2_radio_kind_yields_no_phy_rather_than_a_guess() {
        let mut cap = NodeProfile::legacy_sx1262().to_cap_payload();
        cap[0] = 2;
        cap[1] = 7; // a part number no v2 firmware in this fleet ever emitted
        let prof = NodeProfile::parse(&cap).expect("the other 28 bytes are still a capability");
        assert_eq!(prof.radio_kind, LoraRadioKind::Unknown(7));
        assert!(
            matches!(prof.phy_current, PhyMode::Unknown(_)),
            "an unknown part must not be reported as a LoRa node"
        );
        // Every downstream consequence is a refusal rather than a guess.
        assert!(prof.phy_modes().is_empty(), "no mode is claimed reachable");
        assert!(!prof.phy_agile());
        assert!(!prof.phy_current.has_spreading_factor());
        assert_eq!(
            prof.hop_capability().map(|h| h.period_unit),
            None,
            "no CMD_SET_HOP on a v2 node — and were there one, its period unit is Unspecified"
        );
        // ...but the node's OWN declared SF span still governs, and that is deliberate:
        // `PhyMode::known_without_spreading_factor` is false for `Unknown`, so an unrecognised
        // modulation never overrides a span the node stated for itself. Not knowing the mode is a
        // reason to stop inventing one, not a reason to discard what the node did say.
        assert_eq!(prof.clamp_sf(9), Some(9));
        // ...and the band/power/stamp the node really did state survive intact.
        let known = NodeProfile::legacy_sx1262();
        assert_eq!(prof.freq_min_hz, known.freq_min_hz);
        assert_eq!(prof.pwr_max_dbm, known.pwr_max_dbm);
        assert_eq!(prof.stamp_hz, known.stamp_hz);
    }

    /// A node declaring v3 in 29 bytes has emitted a frame no parser should accept — refuse it
    /// rather than reading it with a synthesised tail, which would report a PHY set as fact.
    #[test]
    fn a_v3_cap_without_its_tail_is_refused() {
        let mut prof = RadioKindHint::Lr2021Flrc.profile();
        prof.proto_ver = 3;
        let mut short = prof.to_cap_payload().to_vec();
        short[0] = 3; // undo the serialiser's honest downgrade, to build the malformed frame
        assert_eq!(short.len(), CAP_LEN_V2);
        assert!(
            NodeProfile::parse(&short).is_none(),
            "v3 without its PHY tail is malformed, not 'an older node'"
        );
        // …and the serialiser refuses to MAKE that frame in the first place.
        assert_eq!(
            prof.to_cap_payload()[0],
            2,
            "29 bytes are a v2 frame whatever the profile says"
        );
        assert_eq!(prof.to_cap_payload_v3()[0], 3);
        // A version beyond this host's is still refused, tail or no tail.
        let mut future = prof.to_cap_payload_v3();
        future[0] = PROTO_VER + 1;
        assert!(NodeProfile::parse(&future).is_none());
    }

    /// The v2 kind names must keep compiling AND keep matching — pinned fallbacks, old host code
    /// and anything that pattern-matched on them still work, they just name the part now.
    #[test]
    #[allow(deprecated)]
    fn the_deprecated_v2_kind_aliases_still_decode() {
        assert_eq!(LoraRadioKind::Lr2021Flrc, LoraRadioKind::Lr2021);
        assert_eq!(LoraRadioKind::Lr2021Lora, LoraRadioKind::Lr2021);
        // Still usable in pattern position, which is what "alias, not deletion" has to mean.
        let k = LoraRadioKind::from_code(3);
        assert!(matches!(k, LoraRadioKind::Lr2021Lora));
        assert_eq!(
            k.code(),
            2,
            "but it SERIALISES as the part code, never as 3"
        );
    }

    /// ★ **The heart of this pass: a PHY switch replaces the capability, it does not patch it.**
    ///
    /// The same silicon in FLRC and in LoRa disagrees about the payload cap, whether a spreading
    /// factor exists at all, the rate model and the band. A host that kept one field across the
    /// switch would hand the face an MTU the frame cannot hold — the silent-corruption class this
    /// profile exists to stop — or compute a 3 s timeout for an 8.5 s transmission.
    #[test]
    fn a_phy_switch_changes_the_whole_capability_coherently() {
        // Before: the LR2021 in FLRC, as its v3 firmware describes itself.
        let mut flrc = RadioKindHint::Lr2021Flrc.profile();
        flrc.proto_ver = 3;
        flrc.learned = true;
        flrc.phy_bitmap = PhyModeSet::single(PhyMode::Flrc).with(PhyMode::Lora).bits();
        flrc.cmd_bitmap |= cmd_bits(&[CMD_SET_PHY, CMD_SET_FREQ]);
        let before = capability_from(&flrc, 65);
        assert_eq!(before.phy_current, Some(PhyMode::Flrc));
        assert_eq!(before.max_payload, 47, "the fixed FLRC frame");
        assert_eq!(before.rate, RateCapability::None);
        assert_eq!(before.sf_range(), None, "FLRC has no spreading factor");
        assert!(before.phy_modes.is_agile());

        // The node's reply to CMD_SET_PHY(LoRa): a WHOLE new EVT_CAP, not a patch.
        let after_cap = NodeProfile {
            phy_current: PhyMode::Lora,
            max_payload: LORA_NODE_RX_MAX, // LoRa carries far more than the 48-byte FLRC PDU
            sf_min: 7,
            sf_max: 12,
            sched_gran_ns: 50_000,
            ..flrc
        }
        .to_cap_payload_v3();
        let after_prof = NodeProfile::parse(&after_cap).expect("the reply parses");
        let after = capability_from(&after_prof, 65);

        assert_eq!(after.phy_current, Some(PhyMode::Lora));
        assert_eq!(
            after.max_payload, 247,
            "every consumer of the MTU must move with the PHY"
        );
        assert_eq!(
            after.sf_range(),
            Some((7, 12)),
            "…and so must the rate model"
        );
        assert!(after_prof.has_spreading_factor());
        assert_eq!(
            after.phy_modes, before.phy_modes,
            "the SET is a property of the part and does not move"
        );
        assert_ne!(
            after, before,
            "the capability as a whole changed — nothing here may be carried across"
        );
        // And the retune cost, which was MEASURED in FLRC, does not follow the chip into LoRa.
        assert_eq!(before.retune_us, Some(RETUNE_US_LR2021_FLRC));
        assert_eq!(after.retune_us, None, "unmeasured in this mode");

        // The host's mirror of the parameters is re-clamped into the new spans and NOT re-sent.
        let mut params = LoraParams {
            sf: 7,
            pwr: 22,
            tx_ch: 65,
            ..LoraParams::default()
        };
        reconcile_params(&mut params, &flrc);
        assert_eq!(
            params.sf, 7,
            "a node with no SF span leaves the mirror alone rather than zeroing it"
        );
        reconcile_params(&mut params, &after_prof);
        assert_eq!(params.sf, 7, "and SF7 is inside the new SF7..SF12 span");
        let mut narrow = after_prof;
        narrow.sf_min = 9;
        narrow.sf_max = 12;
        reconcile_params(&mut params, &narrow);
        assert_eq!(params.sf, 9, "clamped up into the mode the node is now in");
    }

    /// A mode the node does not advertise is refused **before it reaches the wire**, and the
    /// refusal names what it does advertise. Same gate as `CMD_SET_FREQ` on the LR2021: the point
    /// of a capability bitmap is that the host does not have to find out on air.
    #[test]
    fn switching_to_an_unadvertised_phy_is_refused_locally() {
        let prof = RadioKindHint::Lr2021Flrc.profile();
        assert!(!prof.phy_modes().contains(PhyMode::Ble));
        assert!(
            !prof.supports(CMD_SET_PHY),
            "the pre-v2 build has no such opcode"
        );
        // Both guards must hold independently: an advertised set with no opcode is not agility,
        // and an opcode with an unadvertised mode is not a switch.
        let mut opcode_only = prof;
        opcode_only.cmd_bitmap |= cmd_bits(&[CMD_SET_PHY]);
        assert!(!opcode_only.phy_agile(), "one mode is not a choice");
        assert!(!opcode_only.phy_modes().contains(PhyMode::Ble));
    }

    /// **A node whose PHY and whose SF span disagree must lose the `CMD_SET_MOD` triple.**
    ///
    /// The two are separate declarations, so a firmware that switched to a fixed-rate mode and left
    /// `sf_min` at 7 would let this host push a LoRa `[sf, bw, cr]` into a packet engine that reads
    /// byte 0 as a bitrate rung — the verified 10× silent re-modulation. The stricter declaration
    /// wins. An UNRECOGNISED mode does not override, because overriding takes certainty.
    #[test]
    fn a_phy_that_cannot_have_an_sf_overrides_a_declared_span() {
        let mut lying = NodeProfile::legacy_sx1262();
        assert!(lying.has_spreading_factor(), "SF7..SF12 in LoRa: fine");

        lying.phy_current = PhyMode::Flrc; // the span was never zeroed
        assert!(
            !lying.has_spreading_factor(),
            "a mode with no spreading factor overrides the leftover span"
        );
        assert_eq!(
            lying.clamp_sf(9),
            None,
            "and the two halves stay one decision"
        );
        assert_eq!(
            capability_from(&lying, 65).rate,
            RateCapability::None,
            "so the capability does not advertise a rate ladder that would re-modulate the link"
        );

        // The same for every other named non-LoRa mode…
        for m in [PhyMode::Ble, PhyMode::LrFhss, PhyMode::Ook, PhyMode::ZWave] {
            let mut p = lying;
            p.phy_current = m;
            assert!(!p.has_spreading_factor(), "{m:?}");
        }
        // …but NOT for a mode this build has never heard of: there the node's own declared span is
        // the only information there is, so it stands.
        let mut future = lying;
        future.phy_current = PhyMode::Unknown(20);
        assert!(future.has_spreading_factor());
        assert_eq!(future.clamp_sf(99), Some(12));
    }

    // ── v3: the absolute transmit seam ────────────────────────────────────────────────────────

    /// **`CMD_TX_AT_ABS` is what makes the absolute path independent of the host.** MEASURED, the
    /// relative opcode places a slot with sd 553 µs / p2p 1875 µs against a declared 50 µs
    /// granularity, because its delay is counted from when the FIRMWARE processes the arm — the
    /// same magnitude as that node's 550 µs command round-trip p2p. Naming an instant removes the
    /// host from the answer, and these predicates are what select it.
    #[test]
    fn the_absolute_opcode_takes_over_the_at_clock_seam() {
        let mut p = RadioKindHint::Lr2021Flrc.profile();
        p.sched_gran_ns = 50_000; // the node's own MEASURED figure

        // v2 node: relative opcode + a readable clock. The absolute path works, at the price of a
        // round trip whose jitter lands in the placement.
        p.cmd_bitmap |= cmd_bits(&[CMD_TX_AT, CMD_READ_CLOCK]);
        assert!(p.schedules_tx() && p.schedules_after(250_000));
        assert!(p.schedules_at_clock(true));
        assert!(!p.schedules_tx_abs(), "no 0x1F yet");

        // v3 node: it names instants. The absolute seam no longer needs the clock read at all.
        let mut v3 = p;
        v3.cmd_bitmap |= cmd_bits(&[CMD_TX_AT_ABS]);
        assert!(v3.schedules_tx_abs());
        let mut no_clock = v3;
        no_clock.cmd_bitmap &= !(1u32 << CMD_READ_CLOCK);
        assert!(
            no_clock.schedules_at_clock(true),
            "0x1F needs no CMD_READ_CLOCK — that round trip WAS the jitter"
        );
        // …whereas without 0x1F, losing the clock takes the absolute seam down, as it always did.
        let mut v2_no_clock = p;
        v2_no_clock.cmd_bitmap &= !(1u32 << CMD_READ_CLOCK);
        assert!(!v2_no_clock.schedules_at_clock(true));

        // A domain that is not this node's is never a time on this radio, however good the opcode.
        assert!(!v3.schedules_at_clock(false));

        // An absolute-only node (no relative opcode) still schedules — inject_after converts —
        // so `schedules_tx()` stays a claim the seam can honour rather than one it cannot.
        let mut abs_only = v3;
        abs_only.cmd_bitmap &= !(1u32 << CMD_TX_AT);
        assert!(abs_only.schedules_tx());
        assert!(abs_only.schedules_after(250_000));
        assert!(abs_only.schedules_at_clock(true));
        assert_eq!(
            abs_only.tx_discipline(),
            TxDiscipline::ScheduledAt {
                granularity_ns: 50_000
            }
        );
        // And with neither the clock nor the relative opcode, the relative surface goes dark on
        // its own rather than silently transmitting now while claiming a slot.
        let mut abs_no_clock = abs_only;
        abs_no_clock.cmd_bitmap &= !(1u32 << CMD_READ_CLOCK);
        assert!(!abs_no_clock.schedules_after(250_000));
    }

    /// The absolute opcode spends **8** payload bytes on the instant where the relative one spends
    /// 4, and 7E-A5 frames a single-byte length. The fleet's largest payload fits exactly, with no
    /// headroom — pin it here rather than rediscovering it as a truncated frame on air.
    #[test]
    fn an_absolute_transmit_still_frames_the_largest_payload() {
        assert_eq!(MAX_LORA_PAYLOAD + 8, 255, "247 + a u64 tick = exactly 255");
        assert_eq!(
            MAX_LORA_PAYLOAD + 4,
            251,
            "the relative form keeps four bytes of slack"
        );
    }

    // ── v3: hopping, which is NOT the retune cost ─────────────────────────────────────────────

    /// **`retune_us` cannot express a radio that hops inside a packet**, which is why the hop
    /// capability is a separate field and not a smaller number in that one. The LR2021's MEASURED
    /// host-commanded retune is 52 798 µs — hopeless at any slot dwell — while the same part walks
    /// a frequency list autonomously mid-frame.
    #[test]
    fn hopping_is_a_capability_distinct_from_the_retune_cost() {
        let mut p = RadioKindHint::Lr2021Flrc.profile();
        p.cmd_bitmap |= cmd_bits(&[CMD_SET_FREQ]);
        assert_eq!(p.retune_us(), Some(RETUNE_US_LR2021_FLRC));
        assert_eq!(
            p.hop_capability(),
            None,
            "a measured retune cost says nothing about an autonomous sequencer"
        );
        assert_eq!(capability_from(&p, 65).can_hop(100_000), Some(false));

        // Advertise 0x1E and the second, independent answer appears.
        p.cmd_bitmap |= cmd_bits(&[CMD_SET_HOP]);
        let hop = p.hop_capability().expect("0x1E is the actuator");
        assert!(hop.intra_packet);
        assert_eq!(hop.max_list_len as usize, HOP_LIST_MAX);
        assert_eq!(
            hop.period_unit,
            HopPeriodUnit::Unspecified,
            "FLRC has no symbol to count, and this host has not established what the byte means \
             there — a caller must not turn that into a dwell"
        );
        let cap = capability_from(&p, 65);
        assert!(cap.hops_intra_packet());
        assert_eq!(
            cap.can_hop(100_000),
            Some(false),
            "and the host-commanded retune is UNCHANGED by it — one does not imply the other"
        );

        // In a LoRa modulation the period is a symbol count, which is why it cannot be cached as a
        // duration: its wall-clock value moves with SF and bandwidth.
        let mut lora = p;
        lora.phy_current = PhyMode::Lora;
        lora.sf_min = 7;
        lora.sf_max = 12;
        assert_eq!(
            lora.hop_capability().unwrap().period_unit,
            HopPeriodUnit::LoraSymbols
        );
    }

    /// The `CMD_SET_HOP` wire layout, pinned from the host side.
    #[test]
    fn the_hop_plan_payload_matches_the_wire_contract() {
        let freqs = [903_000_000u32, 915_000_000, 927_000_000];
        let p = hop_payload(HopControl::On, 4, &freqs);
        assert_eq!(p.len(), 1 + 2 + 1 + 4 * 3);
        assert_eq!(p[0], 1, "hop_ctrl: on");
        assert_eq!(u16::from_be_bytes([p[1], p[2]]), 4, "period, big-endian");
        assert_eq!(p[3], 3, "n");
        assert_eq!(
            u32::from_be_bytes([p[4], p[5], p[6], p[7]]),
            903_000_000,
            "and every carrier is a u32 BE Hz"
        );
        assert_eq!(hop_payload(HopControl::Off, 0, &[])[0], 0);
        // The wire's own bound: a longer list would be truncated by the node, and a truncated hop
        // list is a plan the two ends stop agreeing on.
        assert_eq!(HOP_LIST_MAX, 40);
        let full: Vec<u32> = (0..HOP_LIST_MAX as u32).map(|i| 903_000_000 + i).collect();
        assert_eq!(hop_payload(HopControl::On, 4, &full).len(), 4 + 4 * 40);
    }

    // ── v3: the receive-gain knob that had no host surface at all ─────────────────────────────

    /// One boolean byte on every node in the fleet — `0` = the part's own default (AGC on the
    /// LR2021, the power-saving LNA on the SX126x), `1` = its highest manual gain. That the wire
    /// has no *scale* is what makes this portable at all; the dB delta is per part and mostly
    /// unmeasured, so the knob is a posture and never a link-budget figure.
    #[test]
    fn rx_gain_is_one_boolean_byte_fleet_wide() {
        assert_eq!(rx_gain_byte(RxGain::Auto), 0);
        assert_eq!(rx_gain_byte(RxGain::Boosted), 1);
        assert_eq!(RxGain::default(), RxGain::Auto, "the part's own default");
        // All three flashed firmwares advertise the opcode (bit 28) — it simply had no caller.
        assert_eq!(CMD_SET_RX_GAIN, 0x1C);
        assert_eq!(1u32 << CMD_SET_RX_GAIN, 0x1000_0000);
    }

    // ── v3: an unsolicited EVT_CAP ────────────────────────────────────────────────────────────

    /// ★ **A capability the node pushes without being asked must land.** The Waveshare re-publishes
    /// its `EVT_CAP` when its self-measured scheduling granularity moves materially; a host that
    /// only applied capabilities arriving as *replies* would keep planning slots against a number
    /// the node has already abandoned.
    ///
    /// Exercised through `handle_event` — the real reader dispatch — not a re-statement of it.
    #[test]
    fn an_unsolicited_evt_cap_updates_the_stored_profile() {
        let (txf, _rxf) = mpsc::unbounded_channel();
        let (resp, resp_rx) = std::sync::mpsc::channel();
        let stored = Mutex::new(RadioKindHint::Lr2021Flrc.profile());
        let dom = ClockDomainId(0x1234);
        assert_eq!(stored.lock().unwrap().sched_gran_ns, 0);

        // The node decides its granularity moved and says so, unprompted.
        let mut fresh = RadioKindHint::Lr2021Flrc.profile();
        fresh.proto_ver = 3;
        fresh.sched_gran_ns = 50_000;
        fresh.cmd_bitmap |= cmd_bits(&[CMD_TX_AT, CMD_TX_AT_ABS]);
        fresh.max_payload = 47;
        handle_event(
            EVT_CAP,
            &fresh.to_cap_payload_v3(),
            &txf,
            &resp,
            &stored,
            dom,
            false,
            &mut None,
        );

        let now = *stored.lock().unwrap();
        assert!(now.learned, "an EVT_CAP-derived profile is learned");
        assert_eq!(now.sched_gran_ns, 50_000);
        assert!(
            now.schedules_tx() && now.schedules_tx_abs(),
            "and every surface derived from it moves with it, unasked"
        );
        assert_eq!(
            capability_from(&now, 65).phy_current,
            Some(PhyMode::Flrc),
            "capability() reflects it too — it is a pure function of the profile"
        );

        // It is ALSO still offered as a reply, so a CMD_GET_CAP in flight is not starved…
        assert_eq!(resp_rx.try_recv().unwrap().0, EVT_CAP);

        // …and a payload that does not parse must never blank a good profile.
        handle_event(
            EVT_CAP,
            &[0x03, 0x02],
            &txf,
            &resp,
            &stored,
            dom,
            false,
            &mut None,
        );
        assert_eq!(
            stored.lock().unwrap().sched_gran_ns,
            50_000,
            "a truncated capability is not a capability"
        );
        assert_eq!(resp_rx.try_recv().unwrap().0, EVT_CAP);
    }
}
