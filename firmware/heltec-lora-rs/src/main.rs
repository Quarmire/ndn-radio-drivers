//! Heltec WiFi LoRa 32 V2 (ESP32 + SX1276) — open Rust firmware, named-radio node C (task #54).
//!
//! A host-driven LoRa modem speaking the **7E-A5 v3** serial protocol, at feature parity with the
//! Waveshare SX1262 node (`../waveshare-lora-rs`) so one host driver treats the two as peers. The
//! air side is plain standard LoRa — no proprietary header — so it interoperates with any
//! SX127x/SX126x peer.
//!
//! # ★ What changed in the 2026-08-28 H pass (H1..H5): hop-event timestamping
//!
//! An LR2021 and this SX1276 both do LoRa intra-packet frequency hopping and **cannot hop with each
//! other**, and the failure survives a ONE-ENTRY hop list — the machinery runs but the carrier never
//! moves — so it is not the sequence, not the phase and not the frame format. The two disagree
//! about *when a hop boundary falls*, and a period sweep over 2/4/8/16 has already been ruled out on
//! air. ★ The way to see it is that **each node timestamps its OWN hop events**: no cross-vendor
//! reception is needed to compare the two timelines, which matters because the link that would
//! carry the comparison is the very thing that is broken. Full argument at [`HopTrace`].
//!
//! * **H1 — the stamp.** `Instant::now()` as the FIRST statement of [`Radio::service_hop`], ahead of
//!   that function's four SPI transactions (~32 µs of byte time), on the same `embassy-time` counter
//!   `EVT_RX.ts` is filled from and `CMD_READ_CLOCK` returns. ⚠ What remains between it and the true
//!   RF hop instant is stated term by term at [`HopTrace`] and is **NOT MEASURED**; nothing is
//!   folded into the stamp to cover it.
//! * **H2 — [`CMD_GET_HOPTRACE`] (0x20) / [`EVT_HOPTRACE`] (0x8E).** A 32-entry free-running ring,
//!   most recent last, that reading does not clear; ticks and `stamp_hz` both on the wire so the
//!   HOST does the unit conversion and a 16 MHz node need not lose resolution to match this 1 MHz
//!   one.
//! * **H3 — cancel safety is structurally unchanged.** Neither addition to `service_hop` is an
//!   `await`, so its suspension points are byte-for-byte what they were and it is still awaited as a
//!   bare statement at both call sites, never as a `select` branch.
//! * **H4 — TX vs listening.** In FHSS the receiver hops too, so a trace read after a frame would
//!   otherwise mix the frame's own hops with the idle-RX hops around it. Bit 7 of `idx`
//!   ([`HOPTRACE_TX_FLAG`]) is set from the CALL SITE — `fire_tx` vs the main loop — with no extra
//!   register read.
//! * **H5 — the self-description.** `CMD_BITMAP` is a u32 and is FULL at 0x1F, and 0x20 is bit 32;
//!   [`CMD_BITMAP_EXT`] carries opcodes 32..63 rather than widening `EVT_CAP`'s frozen 34-byte
//!   layout, which would desynchronise this node from the LR2021 it exists to be compared with.
//!
//! # What changed in the 2026-08-28 v3 pass (C1..C5)
//!
//! v3's premise is that **a modulation is a knob, not an identity**. `EVT_CAP` therefore describes
//! the PHY currently in effect, `radio_kind` names the PART, and `CMD_SET_PHY` answers with a whole
//! new `EVT_CAP` rather than a patch.
//!
//! * **★ C1 — `CMD_SET_HOP` (0x1E): real SX1276 intra-packet frequency hopping.** `lora-phy` 3.0.1
//!   knows the `FhssChangeChannel` interrupt exists and exposes no hopping API at all — no
//!   `RegHopPeriod`, no hop list — and masks the interrupt in every radio mode. This firmware
//!   reaches past it to `RegHopPeriod` (0x24), `RegFrf` (0x06-0x08) and `RegHopChannel` (0x1C), maps
//!   **DIO1 (GPIO35)** to the hop interrupt, and services it in ~32 µs. See [`Hop`],
//!   [`Radio::service_hop`] and [`Radio::hop_irq`]; the cancel-safety argument is in
//!   `service_hop`'s doc comment and is the C3 discipline unchanged.
//! * **C2 — `CMD_SET_PHY` (0x1D) + `EVT_CAP.phy_bitmap`.** The SX1276 does LoRa, FSK and OOK; this
//!   pass brings up LoRa and [`PHY_BITMAP`] advertises exactly that one bit, with what FSK/OOK would
//!   actually take written down beside it. A correct one-entry bitmap beats a fictional three-entry
//!   one, and a PHY the node does advertise but the chip refuses answers the new `EVT_PHY_ERR`
//!   (0x8D) carrying the chip's literal `RegOpMode`.
//! * **C3 — `CMD_TX_AT_ABS` (0x1F): scheduled TX against an ABSOLUTE target** on the counter
//!   `CMD_READ_CLOCK` returns. MEASURED, the relative opcode's placement is capped by the host's
//!   serial latency (LR2021 slot train: mean within 11 µs over 44 slots but sd 553 µs / p2p
//!   1875 µs, matching that node's 550 µs round trip); this node's own round trip has a measured
//!   mean of 10 733 µs against a 99 µs firmware granularity. The absolute target divides it out.
//! * **C4 — the 5.6 ms retune is still 5.6 ms.** Everything C1-C3 adds is gated on hopping being
//!   enabled; with it off, not one extra SPI transaction and not one extra UART byte reaches the
//!   `CMD_SET_FREQ` path. With it on, the cost is three transactions (~24 µs of SPI byte time) in
//!   the `arm_rx` that follows — quantified at [`Radio::apply_hop`] and in the README.
//! * **C5 — `CMD_BITMAP` rebuilt** (0x1D/0x1E/0x1F set) and the compile-time table checks extended
//!   to the PHY surface, the hop table and the new bitmap.
//!
//! Frame shape: `7E A5 | type | len | payload[len] | xor-crc`, crc = XOR of type, len and payload.
//! UART0 (the CP2102 link) is owned RAW for the binary protocol: **nothing prints text on it** in
//! normal operation, or the framing desyncs. `esp-backtrace` still prints on panic, when the link is
//! lost anyway.
//!
//! # What changed in the 2026-08-28 correctness pass (C1..C7)
//!
//! * **C1 — bandwidth was a silent no-op.** The host sends the SX1262 modulation codes 0x04/0x05/
//!   0x06 while this firmware decoded 1/2 as 250/500 kHz, so *every* setting fell through to
//!   125 kHz. The canonical wire space is now pinned (see [`bw_hz_of_code`]) and both spaces decode.
//! * **C2 — CAD was inert.** `lora-phy`'s `cad()` returns `InvalidRadioMode` unless `prepare_for_cad`
//!   has run, and only `prepare_for_cad` remaps DIO0 from RxDone to CadDone; the old loop armed
//!   `RxMode::Continuous` every iteration, so `CMD_CAD`/`CMD_TX_LBT` measured nothing and the LBT
//!   "timed out, treat as clear" arm fired every time. [`Radio::cad`] now does the full sequence.
//! * **C3 — the RX future was cancelled mid-flow.** See [`Radio`] for the cancel-safe restructuring.
//! * **C4 — `EVT_INFO`/`EVT_STATS` were fabricated.** Both are now real; the two fields the SX1276
//!   genuinely cannot supply are named at [`send_info`].
//! * **C5 — serial loss was invisible.** [`uart_reader`] counts every `RxError` (FIFO overflow
//!   included) into `SERIAL_LOST`, reported as `EVT_INFO.lost`.
//! * **C6 — LBT backoff was deterministic.** Now an xorshift32 seeded from the ESP32 RNG *and* the
//!   chip-unique eFuse MAC, so two Heltecs cannot pick the same backoff sequence ([`Csma::seed`]).
//! * **C7 — no build attribution.** [`BUILD_ID`] is stamped in by `build.rs` and emitted as an
//!   `EVT_LOG` at boot and before every `CMD_GET_INFO` reply.
//!
//! One further bug found while fixing those: `UartTx::write_async` returns the number of bytes it
//! could fit in the 128-byte TX FIFO and the old `send_frame` ignored it, so any event longer than
//! the free FIFO space was silently truncated on the wire. [`write_all`] loops.
//!
//! # What changed in the 2026-08-28 parity pass (P1..P6)
//!
//! Three opcodes the v2 survey had recorded as impossible on this radio turned out to be possible;
//! one really is impossible and now says so in one place instead of falling through.
//!
//! * **P1 — `CMD_TX_AT` (0x18) is real.** The SX1276 has no delayed key-up engine, but the MCU is
//!   the queue. The frame is programmed into the chip *ahead* of its deadline and only the
//!   `RegOpMode <- TX` write is left to fire, so the schedule is not paid for with the whole
//!   programming burst. See [`SchedTx`] for the two-phase state machine, [`SCHED_GRAN_NS`] for the
//!   granularity arithmetic, and [`SchedTx`]'s cancel-safety note for why it joins the C3
//!   discipline rather than breaking it.
//! * **P2 — `CMD_SET_CAD_CFG` (0x0A) is real.** The survey said "the SX1276 has no SetCadParams
//!   equivalent", which is true of the *command set* and false of the *hardware*:
//!   `RegDetectOptimize` (0x31) and `RegDetectionThreshold` (0x37) are the detector. One of the
//!   three fleet fields has no counterpart here and is ignored, loudly — see the handler.
//! * **P3 — `CMD_SET_RX_GAIN` (0x1C) is real**, via `RegLna` (0x0C), with the Waveshare node's
//!   boolean payload. `lora-phy` rewrites that register on every arm, so the knob is re-applied
//!   after each one instead of written once (the difference between a knob and a decoration).
//! * **P4 — `CMD_ENTER_BOOTLOADER` (0x16) stays `EVT_UNSUPPORTED`** and is now the only opcode in
//!   the v2 space that is: the CP2102 pulls DTR/RTS and the ESP32 ROM downloader comes up in
//!   hardware, so there is nothing for firmware to do.
//! * **P5 — the 5.6 ms retune is protected.** MEASURED 2026-08-28; see [`CMD_SET_FREQ`]'s handler.
//!   None of P1-P3 adds an SPI transaction to that path unless the host has actually set the knob.
//! * **P6 — the compile-time table checks were extended** to cover the new bitmap, the scheduling
//!   arithmetic and the detector/LNA encodings (the latter two live in [`regs`]).

#![no_std]
#![no_main]

/// The on-device NDN data plane, **shared by path with `waveshare-lora-rs` rather than copied**, so
/// name-hash / filter / dedup / CS semantics cannot drift between nodes that must interoperate.
/// (`lr2021-nrf54l15-rs`'s `m6_bridge` includes the same file the same way.)
#[path = "../../waveshare-lora-rs/src/ndn.rs"]
pub mod ndn;
mod regs;

use core::cell::RefCell;
use core::fmt::Write as FmtWrite;
use core::sync::atomic::{AtomicU32, Ordering};

use embassy_executor::Spawner;
use embassy_futures::select::{select, select3, select4, Either, Either3, Either4};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::Channel;
use embassy_time::{Delay, Duration, Instant, Timer};
use esp_backtrace as _;
use esp_hal::clock::CpuClock;
use esp_hal::gpio::{Input, InputConfig, Level, Output, OutputConfig, Pull};
use esp_hal::spi::master::{Config as SpiConfig, Spi};
use esp_hal::spi::Mode;
use esp_hal::time::Rate;
use esp_hal::timer::timg::TimerGroup;
use esp_hal::uart::{Config as UartConfig, Uart, UartRx, UartTx};
use esp_hal::Async;
use lora_phy::iv::GenericSx127xInterfaceVariant;
use lora_phy::mod_params::{
    Bandwidth, CodingRate, ModulationParams, PacketParams, PacketStatus, RadioError, RadioMode,
    RxMode, SpreadingFactor,
};
use lora_phy::mod_traits::{IrqState, RadioKind};
use lora_phy::sx127x::{Config as Sx127xConfig, Sx127x, Sx1276};
use static_cell::StaticCell;

use regs::{Bus, SharedSpi};

esp_bootloader_esp_idf::esp_app_desc!();

/// Git identity of the source this image was built from (`build.rs`, bug C7).
const BUILD_ID: &str = env!("HELTEC_BUILD_ID");

// =================================================================================================
// 7E-A5 v3 wire protocol. Opcode numbering is FLEET-WIDE — the Waveshare node is canonical and this
// file must not diverge from `../waveshare-lora-rs/src/main.rs`.
// =================================================================================================
const SYNC0: u8 = 0x7E;
const SYNC1: u8 = 0xA5;

// --- Host -> firmware ---
const CMD_TX: u8 = 0x01; //            payload = LoRa frame bytes
const CMD_SET_FREQ: u8 = 0x02; //      payload = u32 BE Hz
const CMD_SET_MOD: u8 = 0x03; //       payload = [sf, bw_code, cr_code]
const CMD_SET_PWR: u8 = 0x04; //       payload = [i8 dBm]
const CMD_SET_SYNC: u8 = 0x05; //      payload = [sx127x sync byte]
const CMD_GET_INFO: u8 = 0x06; //      payload = []
const CMD_SET_BEACON: u8 = 0x07; //    payload = [enabled] or [enabled, period_mult]
const CMD_CAD: u8 = 0x08; //           payload = []            -> EVT_CAD [busy]
const CMD_GET_RSSI: u8 = 0x09; //      payload = []            -> EVT_RSSI [rssi i16 BE]
const CMD_SET_CAD_CFG: u8 = 0x0A; //   payload = [sym, det_peak, det_min] -> EVT_INFO (P2)
const CMD_SET_LBT_CFG: u8 = 0x0B; //   payload = [cw_ms u16 BE, max_backoff, max_attempts]
const CMD_SET_PREAMBLE: u8 = 0x0C; //  payload = [preamble u16 BE]
const CMD_SF_SCAN: u8 = 0x0D; //       payload = []            -> EVT_SF_DETECTED [sf | 0]
const CMD_TX_LBT: u8 = 0x0E; //        payload = LoRa frame bytes; atomic CAD + backoff + key-up
const CMD_SET_NAME_FILTER: u8 = 0x0F; // payload = [u64 BE hash]*  (empty clears -> pass-all)
const CMD_SET_RELAY: u8 = 0x10; //       payload = [u64 BE hash]*  (relay set; empty clears)
const CMD_DATAPLANE: u8 = 0x11; //       payload = [cs_serve, dedup, hop_on, hop_base_ch, hop_span]
const CMD_SET_SENSE_CFG: u8 = 0x12; //   payload = [rssi_thresh i16 BE, cad_repeat]
const CMD_GET_STATS: u8 = 0x13; //       payload = []  -> EVT_STATS (24 B)
const CMD_RESET_STATS: u8 = 0x14; //     payload = []
const CMD_SET_DEBUG: u8 = 0x15; //       payload = [on]
const CMD_ENTER_BOOTLOADER: u8 = 0x16; // GD32-only -> EVT_UNSUPPORTED here
const CMD_READ_CLOCK: u8 = 0x17; //      payload = []  -> EVT_CLOCK [ticks u64 BE]
const CMD_TX_AT: u8 = 0x18; //           payload = [delay_us u32 BE][frame] -> EVT_TXDONE (P1)
const CMD_GET_CAP: u8 = 0x1A; //         payload = []  -> EVT_CAP (29 B)
const CMD_SENSE: u8 = 0x1B; //           payload = []  -> EVT_SENSE [activity u16 BE, rssi i16 BE]
// Outside the v2 block (0x17..0x1B) so it cannot collide with a future fleet assignment there; the
// host discovers it from EVT_CAP's cmd_bitmap, which is what the bitmap is for. Payload shape is
// the Waveshare node's, byte for byte, because that node is the reference for this opcode.
const CMD_SET_RX_GAIN: u8 = 0x1C; //     payload = [0 = power-saving | 1 = boosted] -> EVT_INFO (P3)

// --- 7E-A5 v3 (2026-08-28). Fleet-wide numbering; the contract is identical on every node. ---
const CMD_SET_PHY: u8 = 0x1D; //   payload = [packet_type u8]                     -> EVT_CAP  (C2)
const CMD_SET_HOP: u8 = 0x1E; //   payload = [hop_ctrl, hop_period u16 BE, n, freq_hz u32 BE * n]
//                                 n <= HOP_MAX                                   -> EVT_INFO (C1)
const CMD_TX_AT_ABS: u8 = 0x1F; // payload = [target_ticks u64 BE][frame]         -> EVT_TXDONE (C3)

// --- 7E-A5 v3, H-pass (2026-08-28): hop-event timestamping. Fleet-wide numbering; the LR2021 node
// implements the SAME two opcodes with the SAME encoding, which is the entire point — see
// [`HopTrace`] for what question this exists to answer.
const CMD_GET_HOPTRACE: u8 = 0x20; // payload = []                              -> EVT_HOPTRACE (H2)

// --- Firmware -> host ---
const EVT_RX: u8 = 0x81; //    [rssi i16 BE, snr i16 BE, ts u32 BE, LoRa bytes]; ts unit = CAP.stamp_hz
const EVT_TXDONE: u8 = 0x82; // [ok, attempts]  (attempts = 0 for a plain CMD_TX)
const EVT_INFO: u8 = 0x83; //  19 B: [status, sync(2), errors(2), freq(4), sf, bw, cr, pwr, lost(2),
//                             cad_busy(2), defer(2)]. FIXED WIDTH: the host reads cad_busy/defer as
//                             the last 4 bytes, so nothing may ever be appended. New counters go in
//                             EVT_STATS.
const EVT_LOG: u8 = 0x84; //   ascii
const EVT_CAD: u8 = 0x85; //   [busy(0/1)]
const EVT_RSSI: u8 = 0x86; //  [rssi i16 BE] dBm
const EVT_SF_DETECTED: u8 = 0x87; // [sf | 0 = none]
const EVT_TX_STARTED: u8 = 0x88; //  [airtime_ms u16 BE] — emitted just before key-up
const EVT_STATS: u8 = 0x89; //       24 B: [rx(4), filtered(4), deduped(4), served(4), relayed(4),
//                                   cad_busy(2), defer(2)]
const EVT_CLOCK: u8 = 0x8A; //       [ticks u64 BE], unit = EVT_CAP.stamp_hz
const EVT_CAP: u8 = 0x8B; //         34 B in v3 (29 in v2 + phy_bitmap/phy_current), see `send_cap`
const EVT_SENSE: u8 = 0x8C; //       [activity u16 BE, rssi i16 BE]
const EVT_PHY_ERR: u8 = 0x8D; //     [requested_phy, chip_status] — v3. A PHY this node ADVERTISES in
//                                   EVT_CAP.phy_bitmap that the chip then refused, carrying the
//                                   chip's literal status byte (RegOpMode here). A PHY that is NOT
//                                   in the bitmap is a different answer and gets the fleet's
//                                   existing one: EVT_UNSUPPORTED [0x1D, OUT_OF_RANGE].
const EVT_HOPTRACE: u8 = 0x8E; //     [stamp_hz u32 BE][n u8][ (idx u8, t_ticks u32 BE) ]*n — H2.
//                                   `stamp_hz` is THIS node's own tick rate (== EVT_CAP.stamp_hz),
//                                   never converted to microseconds in firmware: the host divides,
//                                   so a 16 MHz node does not have to throw away resolution to
//                                   match a 1 MHz one. `n` most-recent-LAST, capped at
//                                   HOPTRACE_MAX = 32 => 4 + 1 + 32*5 = 165 B, one frame.
//                                   `idx` bits 5:0 = the hop index the event moved TO (this node:
//                                   RegHopChannel bits 5:0, verbatim); bit 7 = HOPTRACE_TX_FLAG,
//                                   set iff the hop was taken while TRANSMITTING (H4).
const EVT_UNSUPPORTED: u8 = 0x8F; // [cmd, reason] — never silence, never a fake success

// EVT_UNSUPPORTED reason codes (shared with the Waveshare node).
const UNSUP_UNKNOWN_OPCODE: u8 = 0x01;
const UNSUP_NO_HARDWARE: u8 = 0x02;
const UNSUP_BAD_LENGTH: u8 = 0x03;
const UNSUP_OUT_OF_RANGE: u8 = 0x04;

/// **The self-description bitmap** (`EVT_CAP.cmd_bitmap`): bit N set <=> opcode N is implemented and
/// will act. It is also this firmware's own "is this opcode known?" oracle in [`handle_cmd`]'s
/// catch-all, so the bitmap and the dispatcher physically cannot drift apart.
///
/// ```text
/// set   0x01..0x15  TX .. SET_DEBUG — every opcode in that run, SET_CAD_CFG (0x0A) included (P2)
///       0x17 0x18   READ_CLOCK, TX_AT (P1 — a real MCU-queued scheduled TX; sched_gran_ns > 0)
///       0x1A 0x1B   GET_CAP, SENSE
///       0x1C        SET_RX_GAIN (P3 — RegLna, the Waveshare's boolean payload)
///       0x1D        SET_PHY (C2 — the selection machinery; this node's phy_bitmap is LoRa only)
///       0x1E        SET_HOP (C1 — real SX1276 intra-packet FHSS, RegHopPeriod + RegFrf on IRQ)
///       0x1F        TX_AT_ABS (C3 — absolute-target scheduled TX on this node's own clock)
/// clear 0x00        not an opcode
///       0x16        ENTER_BOOTLOADER — GD32 ROM-bootloader opcode; the ESP32 is entered by the
///                                      CP2102 pulling DTR/RTS, so no firmware opcode exists (P4)
///       0x19        unassigned
/// ```
/// bits 1-21 = 0x003F_FFFE, bits 23-24 = 0x0180_0000, bits 26-31 = 0xFC00_0000
///   =>  0xFDBF_FFFE.
///
/// ☠ **This word is FULL at opcode 0x1F, and `CMD_GET_HOPTRACE` is 0x20 = 32.** It cannot be
/// advertised here and is not: `EVT_CAP.cmd_bitmap` is a u32 in a layout frozen at 34 bytes, so
/// widening it would be a fleet-wide wire change, not a firmware change, and it is not one this
/// pass is entitled to make. See [`CMD_BITMAP_EXT`] for where bit 32 actually lives and how a host
/// discovers 0x20.
const CMD_BITMAP: u32 = 0xFDBF_FFFE;

/// **Opcodes 32..63**, bit `(op - 32)`. The continuation of [`CMD_BITMAP`] past the u32 that
/// `EVT_CAP` can carry.
///
/// It exists because the 32-bit self-description ran out exactly one opcode before this pass needed
/// it, and the three ways out are not equal:
///
/// * **Widen `EVT_CAP`.** Its v3 layout is 34 bytes and every node in the fleet plus the host
///   parser agree on it. Changing it here alone would desynchronise this node from the LR2021 —
///   the very node this trace exists to be COMPARED with. Refused.
/// * **Renumber `CMD_GET_HOPTRACE` into a gap** (0x19 is free). Refused for a different reason: the
///   opcode number is part of the contract stated for BOTH nodes, and a node that answered a
///   different opcode than its counterpart would make the comparison harness node-specific.
/// * **Say the true thing in a place that has room.** This word is emitted in the `CMD_SET_DEBUG`
///   dump (a free-form `EVT_LOG`, no frozen layout), and the primary discovery path is the one the
///   7E-A5 rule set already guarantees: *send it and read the answer.* A node that does not
///   implement 0x20 replies `EVT_UNSUPPORTED [0x20, UNKNOWN_OPCODE]`; one that implements it but
///   cannot stamp its hops replies `EVT_UNSUPPORTED [0x20, NO_HARDWARE]`; this node replies
///   `EVT_HOPTRACE`. Three distinguishable answers, no silence, nothing fabricated.
///
/// ⚠ The dispatcher's catch-all oracle still reads [`CMD_BITMAP`] only, guarded by `f.typ < 32`.
/// That guard is load-bearing in Rust (`1u32 << 32` panics in debug and is UB-adjacent in release),
/// and it is also why 0x20 MUST have an explicit `match` arm above the catch-all — which it has.
const CMD_BITMAP_EXT: u32 = 1 << (CMD_GET_HOPTRACE as u32 - 32);

/// `1 << opcode`, so the compile-time bitmap checks below read as opcodes rather than as hex.
const fn cmd_bit(op: u8) -> u32 {
    1u32 << (op as u32)
}

/// **The bitmap table test** (P6). The dispatcher's catch-all already uses `CMD_BITMAP` as its
/// "known opcode?" oracle, so a bit and its handler cannot drift; these assertions pin the other
/// direction — that the bitmap says what this pass actually implemented, and that the literal above
/// still equals the opcodes it claims. Compile-time, because `#[cfg(test)]` never runs on a
/// `no_std`/`no_main` xtensa target.
const _: () = {
    assert!(CMD_BITMAP & 1 == 0); // opcode 0 does not exist
    // Implemented in this pass.
    assert!(CMD_BITMAP & cmd_bit(CMD_SET_CAD_CFG) != 0); // P2
    assert!(CMD_BITMAP & cmd_bit(CMD_TX_AT) != 0); // P1
    assert!(CMD_BITMAP & cmd_bit(CMD_SET_RX_GAIN) != 0); // P3
    // Genuinely absent: there is no firmware side to it at all.
    assert!(CMD_BITMAP & cmd_bit(CMD_ENTER_BOOTLOADER) == 0); // P4
    // Implemented in the v3 pass.
    assert!(CMD_BITMAP & cmd_bit(CMD_SET_PHY) != 0); // C2
    assert!(CMD_BITMAP & cmd_bit(CMD_SET_HOP) != 0); // C1
    assert!(CMD_BITMAP & cmd_bit(CMD_TX_AT_ABS) != 0); // C3
    // Unassigned opcodes must stay clear or the host will send them.
    assert!(CMD_BITMAP & cmd_bit(0x19) == 0);
    // Everything the pre-existing firmware implements is still claimed.
    assert!(CMD_BITMAP & cmd_bit(CMD_TX) != 0);
    assert!(CMD_BITMAP & cmd_bit(CMD_SET_FREQ) != 0);
    assert!(CMD_BITMAP & cmd_bit(CMD_SET_SYNC) != 0);
    assert!(CMD_BITMAP & cmd_bit(CMD_TX_LBT) != 0);
    assert!(CMD_BITMAP & cmd_bit(CMD_SET_DEBUG) != 0);
    assert!(CMD_BITMAP & cmd_bit(CMD_READ_CLOCK) != 0);
    assert!(CMD_BITMAP & cmd_bit(CMD_GET_CAP) != 0);
    assert!(CMD_BITMAP & cmd_bit(CMD_SENSE) != 0);
    // The literal and the opcode list are two statements of the same fact; make them agree.
    assert!(CMD_BITMAP == 0x003F_FFFE | 0x0180_0000 | 0xFC00_0000);
    // The catch-all in `handle_cmd` reads the bitmap with `CMD_BITMAP >> f.typ` behind an
    // `f.typ < 32` guard, so every opcode the u32 bitmap claims must be inside it.
    assert!(CMD_TX_AT_ABS < 32);
    // ★ H5 — the bitmap really is full at 0x1F, and 0x20 really is the first opcode past its end.
    // These two assertions are what stop a later pass from "just setting the bit" (there is none)
    // or from renumbering 0x20 into the 0x19 gap without noticing the contract it shares with the
    // LR2021 node.
    assert!(CMD_BITMAP & cmd_bit(0x1F) != 0); // the top bit of the u32 is taken
    assert!(CMD_GET_HOPTRACE == 32); // ...so this one cannot be represented in it at all
    // The continuation word says the same thing the dispatcher does: 0x20 is implemented here.
    assert!(CMD_BITMAP_EXT & (1 << (CMD_GET_HOPTRACE as u32 - 32)) != 0);
    // Nothing above 0x20 is claimed yet; a stray bit would advertise an opcode with no handler.
    assert!(CMD_BITMAP_EXT == 0x0000_0001);
    // ★ The host's `NodeProfile::schedules_tx()` requires BOTH the bit and a non-zero granularity,
    // and treats one without the other as an inconsistent node. Fail the build instead.
    assert!((CMD_BITMAP & cmd_bit(CMD_TX_AT) != 0) == (SCHED_GRAN_NS > 0));
};

/// Largest LoRa frame this firmware will receive and report — the **real end-to-end cap**, and what
/// `EVT_CAP.max_payload` advertises.
///
/// It is set by the SERIAL FRAMING, not the radio: an event's `len` field is one byte, so an EVT_RX
/// payload is at most 255 B, of which 8 are the rssi/snr/timestamp header => 247 B of LoRa frame.
/// The SX1276 FIFO (256 B) and the LoRa PDU (255 B) are both larger and `CMD_TX` accepts up to
/// 255 B, so 247 is the binding constraint and therefore the honest number to report.
const RX_MAX: usize = 247;

/// Buffer the radio receives into. Larger than [`RX_MAX`] on purpose: `get_rx_payload` errors out if
/// `RegRxNbBytes` exceeds the buffer, so accepting the full LoRa PDU lets us count an oversize frame
/// (`rx_trunc`) instead of turning it into a radio error we cannot explain.
const RX_BUF: usize = 255;

/// `EVT_RX`/`EVT_CLOCK` timestamp rate. `embassy-time`'s `TICK_HZ` is a compile-time constant of the
/// driver actually linked (esp-rtos selects `tick-hz-1_000_000`), so this is a source constant, not
/// an assumption: EVT_CAP reports whatever the built image really counts in.
const STAMP_HZ: u32 = embassy_time::TICK_HZ as u32;

// =================================================================================================
// P1 — scheduled TX (`CMD_TX_AT`): the granularity arithmetic
// =================================================================================================
//
// Every term below is a SOURCE CONSTANT of this build or of the crate it links, not an estimate.
// The one term that is neither is named as such and is rounded UP, because `EVT_CAP.sched_gran_ns`
// is believed by the host's planner: `NodeProfile::schedules_tx()` gates a whole scheduling mode on
// it, and a granularity that is too small is a lie the planner cannot detect.

/// Nanoseconds per `embassy-time` tick, from the tick rate of the driver actually linked. This is
/// the quantum `Timer::at` can resolve: it fires on the first tick at or after the target, so the
/// worst-case lateness it contributes is one whole tick.
const TICK_NS: u32 = 1_000_000_000 / STAMP_HZ;

/// SPI bus rate, as configured in [`main`] (`SpiConfig::with_frequency(Rate::from_mhz(2))`). Changing
/// it there changes the number below, which is the point of deriving rather than writing 8000.
const SPI_HZ: u32 = 2_000_000;

/// The key-up write is `[RegOpMode | 0x80, LoRaMode::Tx]` — 2 bytes. That really is all `lora-phy`'s
/// `do_tx()` does on an SX127x: `enable_rf_switch_tx()` is a no-op for the interface variant this
/// firmware builds (both RF-switch pins are `None`), and `wait_on_busy()` is a no-op for the whole
/// sx127x family (no BUSY pin). Verified in `lora-phy 3.0.1` `src/sx127x/mod.rs` and `src/iv.rs`.
const KEYUP_BYTES: u32 = 2;
const KEYUP_SPI_NS: u32 = ((KEYUP_BYTES as u64 * 8 * 1_000_000_000) / SPI_HZ as u64) as u32;

/// PA ramp-up. `lora-phy` programs `RegPaRamp` from `set_tx_power_and_ramp_time(.., is_tx_prep =
/// true)`, which is what [`Radio::stage_tx`] passes, and that arm selects `RampTime::Ramp40Us`.
/// So 40 µs of ramp sits between `RegOpMode <- TX` and the carrier reaching full power.
const PA_RAMP_NS: u32 = 40_000;

/// Everything between the timer expiring and the first SPI clock edge: the embassy wake, the
/// `select` resolving, the branch dispatch, and the NSS assert / `flush` that the 8 µs of raw byte
/// time above does not cover.
///
/// **None of it is measured on this board.** Rather than estimate it, it is given a budget at least
/// as large as the sum of every term that *is* derived (1 + 8 + 40 = 49 µs), rounded to 50 µs —
/// the most conservative allocation that still leaves the declaration useful. Replace it with a
/// measured p99 the first time a scheduled TX is timed on hardware; the firmware already computes
/// the error against the same counter and reports it (see the `SchedTx::Staged` arm in [`main`]).
const SCHED_SLACK_NS: u32 = 50_000;

/// **`EVT_CAP.sched_gran_ns`** = 1 000 + 8 000 + 40 000 + 50 000 = **99 000 ns (99 µs)**.
///
/// Read it as: ask this node to transmit at instant T on its own clock (the same counter `EVT_RX`
/// stamps with and `CMD_READ_CLOCK` returns) and the carrier comes up within ~99 µs after T. It is
/// a one-sided bound — the firmware never keys up EARLY — and it does not include the airtime of
/// the frame itself, which `EVT_TX_STARTED` reports separately.
///
/// What it is NOT: a hardware guarantee. There is no TSF comparator on an SX1276. If a long
/// SF12 reception, a relay transmit or a host command is in flight when the deadline arrives, the
/// frame goes out late and the firmware says so rather than pretending (see `SCHED_LATE`).
const SCHED_GRAN_NS: u32 = TICK_NS + KEYUP_SPI_NS + PA_RAMP_NS + SCHED_SLACK_NS;

/// Fixed part of the pre-deadline work, in µs. Two things happen at the staging point: the
/// `EVT_TX_STARTED` push into the UART's TX FIFO, and the ~30 short register transactions
/// [`Radio::stage_tx`] issues (standby, modulation, PA, packet params, channel, FIFO pointers, IRQ
/// mapping). The register burst is about 80 bytes on the wire, ~320 µs of byte time at 2 MHz; the
/// UART push is 7 bytes into a FIFO that is normally empty. Rounded UP to 1.5 ms to absorb
/// per-transaction NSS framing, executor scheduling, and a TX FIFO that is not empty — none of
/// which is measured on this board.
const STAGE_FIXED_US: u64 = 1_500;

/// Per-payload-byte part of the same burst: 8 bits at 2 MHz is 4 µs, taken as 5 µs for slack. At the
/// 247-byte maximum the whole budget is 1 500 + 1 235 = 2 735 µs.
const STAGE_PER_BYTE_US: u64 = 5;

/// Longest delay `CMD_TX_AT` will accept, in µs.
///
/// Not a hardware limit — the frame just sits in RAM while the node keeps receiving. It is the
/// HOST's limit, made explicit: `lora_serial.rs::inject_after` bounds its own reply timeout at
/// `delay_us.min(60_000_000)`, so a longer schedule is one the host has already decided it will not
/// wait for. Accepting it would guarantee a host-side timeout and then a frame going out with
/// nobody listening for the reply. Answered `EVT_UNSUPPORTED[0x18, OUT_OF_RANGE]` instead.
const SCHED_MAX_DELAY_US: u32 = 60_000_000;

/// **The scheduling table test** (P6): the arithmetic above, pinned at compile time.
const _: () = {
    assert!(STAMP_HZ == 1_000_000); // esp-rtos selects tick-hz-1_000_000; if that changes, re-derive
    assert!(TICK_NS == 1_000);
    assert!(KEYUP_SPI_NS == 8_000); // 2 bytes * 8 bits / 2 MHz
    assert!(PA_RAMP_NS == 40_000); // RampTime::Ramp40Us
    assert!(SCHED_GRAN_NS == 99_000);
    // The unmeasured slack must never be the small part of the budget; that is the whole discipline.
    assert!(SCHED_SLACK_NS >= TICK_NS + KEYUP_SPI_NS + PA_RAMP_NS);
    // A granularity finer than the clock that measures it would be unfalsifiable.
    assert!(SCHED_GRAN_NS >= TICK_NS);
    // The staging budget has to cover the largest frame this node will ever schedule.
    assert!(STAGE_FIXED_US + STAGE_PER_BYTE_US * (RX_MAX as u64) < 4_000);
    assert!(STAGE_PER_BYTE_US * 1_000 >= 8 * 1_000_000_000u64 / SPI_HZ as u64); // >= real byte time
};

// =================================================================================================
// C3 — absolute-target scheduled TX (`CMD_TX_AT_ABS`), and what the relative opcode really costs
// =================================================================================================
//
// [`SCHED_GRAN_NS`] above is a FIRMWARE-side figure: the distance between the deadline the firmware
// holds and the instant the carrier comes up. It is the same for `CMD_TX_AT` and `CMD_TX_AT_ABS`,
// because both leave `Radio::fire_tx` exactly one 2-byte `RegOpMode <- TX` write to perform, and
// [`report_sched_error`] measures it against the same counter on both paths. What differs is where
// the deadline COMES FROM, and that is the whole point of the new opcode:
//
// * `CMD_TX_AT [delay_us]` — the deadline is `Instant::now() + delay` evaluated when the FIRMWARE
//   dequeues the command. Everything between the host reading `CMD_READ_CLOCK` and this firmware
//   parsing the arm — USB, the CP2102, the 115200 line, the UART FIFO, `CMDQ` — lands directly in
//   the host's placement, and the firmware cannot see it, so it reports a small error for a frame
//   that went out in the wrong place. MEASURED on the LR2021 node: an absolute-boundary slot train
//   held its mean gap to within 11 µs over 44 slots but had per-slot **sd 553 µs / p2p 1875 µs**
//   against a declared 50 µs, exactly matching that node's 550 µs `CMD_GET_INFO` p2p round trip.
// * `CMD_TX_AT_ABS [target_ticks]` — the deadline is a point on THIS NODE's clock, the one
//   `CMD_READ_CLOCK` returns and `EVT_RX.ts` stamps with. Serial latency then decides only whether
//   the arm ARRIVES in time, not where the frame lands. It divides out instead of adding in.
//
/// **MEASURED 2026-08-28, this node: the `CMD_GET_INFO` round-trip floor, mean 10 733 µs** (n=8..10,
/// host command to `EVT_INFO`, the same harness that measured the 5 597 µs retune).
///
/// This is the term `CMD_TX_AT` adds to the host's placement and `CMD_TX_AT_ABS` does not. It is
/// used here as a **bound on the one-way host->node transit**, which is conservative for the
/// transit (the round trip contains it twice, plus the firmware's own reply work) — but ⚠ it is a
/// bound on the TYPICAL value, not a p99: **the spread was not measured on this node.** Only the
/// mean was, so no p99, sd or peak-to-peak figure is claimed here or in the README. Bounding the
/// spread needs the same slot-train harness that produced the LR2021's sd 553 µs.
///
/// It is deliberately NOT in `EVT_CAP`: the v3 layout is fixed at 34 bytes and `sched_gran_ns`
/// means the firmware-side granularity on every node. It is reported in the `CMD_SET_DEBUG` dump
/// so a host that wants it can read it off the wire.
const SERIAL_RTT_MEAN_US: u32 = 10_733;

/// **The absolute-target range check.** A target more than [`SCHED_MAX_DELAY_US`] in the future is
/// refused for exactly the reason the relative opcode's bound exists — the host has already decided
/// it will not wait that long — and a target in the PAST is not an error at all: it fires now, and
/// [`report_sched_error`] states on the wire how late it was. That split ("past -> fire now,
/// absurd -> OUT_OF_RANGE") is the v3 contract, and it is the honest one: a deadline that has
/// already gone is information the host needs measured, not a refusal it has to interpret.
const _: () = {
    // The two scheduled-TX opcodes must share one bound, or "how far ahead can I schedule?" would
    // have two different answers depending on which opcode the host reached for.
    assert!(SCHED_MAX_DELAY_US == 60_000_000);
    // The measured serial round trip is >100x the firmware granularity. That ratio IS the argument
    // for the absolute opcode; if it ever stops being true, this pass needs re-justifying.
    assert!(SERIAL_RTT_MEAN_US as u64 * 1_000 > 100 * SCHED_GRAN_NS as u64);
};

// =================================================================================================
// C2 — the PHY-selection surface (`CMD_SET_PHY`, `EVT_CAP.phy_bitmap`)
// =================================================================================================
//
// ★ The design error v3 fixes is treating a modulation as an IDENTITY. `SetPacketType` on an
// LR2021 is a runtime command with 14 modes, and the SX1276 has the same shape in register form:
// **`RegOpMode` bit 7 `LongRangeMode` selects LoRa vs FSK/OOK, and `ModulationType` (bits 6:5 of
// `RegOpMode` in FSK/OOK mode) picks FSK from OOK.** So this part is a three-PHY radio and the
// wire has to be able to say so.
//
// What this pass actually brings up is **LoRa, and only LoRa**, and [`PHY_BITMAP`] says exactly
// that — one bit. The rule is that a correct one-entry bitmap beats a fictional three-entry one:
// advertising FSK would promise a host a PHY that has no packet engine here at all. `lora-phy`
// 3.0.1 is a LoRa-only driver (`init_lora`, LoRa-only `ModulationParams`/`PacketParams`, LoRa-only
// IRQ handling), so FSK/OOK is not a knob that is merely unexposed — it is a second packet engine
// with its own register page (`RegBitrate` 0x02/0x03, `RegFdev` 0x04/0x05, `RegSyncConfig` 0x27,
// `RegPacketConfig1/2` 0x30/0x31, a FIFO-threshold-driven TX/RX loop instead of a one-shot FIFO,
// and a completely different DIO map) that would have to be written from nothing. That is a pass
// of its own, not a corner of this one.
//
// ⚠ WHAT REMAINS, named so it is not lost: bring up FSK (`packet_type` 2) and OOK (`packet_type`
// 10) as raw-register PHYs behind the machinery below, then set their bits in [`PHY_BITMAP`] and
// give `send_cap` per-PHY `max_payload` / `sf_min` / `sf_max` / `sched_gran_ns`. The switch itself
// additionally requires the chip to be in SLEEP: `LongRangeMode` (`RegOpMode` bit 7) is documented
// as modifiable only in sleep mode, so a real LoRa<->FSK transition is
// `set_sleep -> rewrite RegOpMode -> re-init the new engine`, which is why `CMD_SET_PHY` returns a
// WHOLE new `EVT_CAP` rather than a patch.

/// `SetPacketType` value 0 — LoRa. The numbering is the LR2021's `PacketType` table, which v3 makes
/// the fleet-wide PHY namespace, so `phy_bitmap` bit N means the same thing on every node.
const PHY_LORA: u8 = 0x0;
/// `SetPacketType` value 2 — (G)FSK. Reachable on this silicon, NOT implemented here; the bit stays
/// clear. Named only so the constant that would set it exists next to the reason it is not set.
#[allow(dead_code)]
const PHY_FSK: u8 = 0x2;
/// `SetPacketType` value 10 — OOK. Same status as [`PHY_FSK`].
#[allow(dead_code)]
const PHY_OOK: u8 = 0xA;

/// **`EVT_CAP.phy_bitmap`** — bit N set <=> `CMD_SET_PHY N` is usable on this node. One bit: LoRa.
const PHY_BITMAP: u32 = 1u32 << PHY_LORA;

/// **`EVT_CAP.phy_current`** — the `SetPacketType` value in effect. Constant on this node until FSK
/// or OOK is brought up; it is a field rather than a literal at the call site so that the day a
/// second PHY lands, the thing that has to change is a variable and not a wire layout.
const PHY_CURRENT: u8 = PHY_LORA;

/// The PHY-surface table test.
const _: () = {
    assert!(PHY_BITMAP == 0x0000_0001); // LoRa, and nothing this pass did not build
    assert!(PHY_BITMAP & (1u32 << PHY_FSK) == 0); // not brought up: the bit must stay clear
    assert!(PHY_BITMAP & (1u32 << PHY_OOK) == 0);
    assert!(PHY_BITMAP & (1u32 << PHY_CURRENT) != 0); // a node must be running a PHY it advertises
    assert!(PHY_LORA < 32 && PHY_FSK < 32 && PHY_OOK < 32); // the bitmap is 32 bits wide
};

// =================================================================================================
// C1 — intra-packet frequency hopping (`CMD_SET_HOP`)
// =================================================================================================

/// Hop-list capacity, from the v3 contract (`n <= 40`). 40 u32 words of precomputed `RegFrf` plus
/// 40 of the host's own Hz values is 320 bytes of static state — cheap, and holding both is what
/// lets the debug dump quote a channel back in the units the host sent it in.
const HOP_MAX: usize = 40;

/// `CMD_SET_HOP.hop_ctrl`: 0 = hopping off (`RegHopPeriod <- 0`), 1 = on. Every other value is
/// `OUT_OF_RANGE` rather than folded into "on" — the same discipline `CMD_SET_RX_GAIN` applies to
/// its boolean, and for the same reason: a host that meant some third mode must be told it was not
/// understood instead of silently getting this one.
const HOP_OFF: u8 = 0;
const HOP_ON: u8 = 1;

/// `RegHopPeriod` is **8 bits wide** while the wire field is a u16 BE. A value above 255 is
/// therefore refused, never truncated: silently writing `period & 0xFF` would hop 256x faster than
/// the host asked and desynchronise the pair with nothing on the wire to say why.
const HOP_PERIOD_MAX: u16 = 255;

/// The hop-table test.
const _: () = {
    assert!(HOP_MAX == 40); // the v3 contract's n <= 40
    assert!(HOP_MAX * 4 + 4 <= 255); // [ctrl][period u16][n] + 4*n must fit one 255-byte frame
    assert!(HOP_PERIOD_MAX == 255); // == the width of RegHopPeriod
    assert!(HOP_OFF != HOP_ON);
};

// =================================================================================================
// ★ H1-H4 — hop-event timestamping (`CMD_GET_HOPTRACE` / `EVT_HOPTRACE`)
// =================================================================================================

/// Ring depth, from the H2 contract. 32 events at `hop_period = 8` / SF7 / BW125 (T_sym 1.024 ms,
/// one hop per 8.192 ms) is **262 ms** of timeline — several whole frames — and 32 * 5 + 5 = 165 B,
/// which fits the single 255-byte serial payload the event is defined to be.
const HOPTRACE_MAX: usize = 32;

/// H4 — bit 7 of the `idx` byte: **this hop was taken while the chip was TRANSMITTING.**
///
/// Without it a trace read after a frame is useless, because in FHSS the RECEIVER hops too: the
/// interrupt fires every `hop_period` symbols forever, so a frame's own hops arrive embedded in an
/// unbroken run of idle-RX hops and nothing in a raw count separates them. That is exactly the
/// confusion that made `Hop::hops` misleading.
///
/// Set iff the event was serviced from [`Radio::fire_tx`] — i.e. the chip was in TX with a frame in
/// flight and `RegOpMode` had been written to TX by `do_tx`. Clear means **not transmitting**: the
/// main loop serviced it, which is idle RX or a reception in progress. It is deliberately a
/// two-valued fact and not a mode enumeration, because "was the carrier ours?" is the only
/// distinction the timing question needs and it is the only one the firmware can state without
/// reading another register.
///
/// Bits 6 and 5:0: `RegHopChannel`'s `FhssPresentChannel` field is 6 bits, so bit 6 is spare and
/// bit 7 cannot collide with a legitimate index.
const HOPTRACE_TX_FLAG: u8 = 0x80;

/// Bytes per entry on the wire: `idx u8` + `t_ticks u32 BE`.
const HOPTRACE_ENTRY_BYTES: usize = 5;

/// **The hop-trace table test** (H5). The wire arithmetic and the flag layout, pinned at compile
/// time — `#[cfg(test)]` cannot run on this `no_std`/`no_main` xtensa target.
const _: () = {
    assert!(HOPTRACE_MAX == 32); // the H2 contract
    assert!(HOPTRACE_ENTRY_BYTES == 5); // idx u8 + t_ticks u32 BE
    // The whole event must fit ONE 255-byte serial payload: [stamp_hz u32][n u8] + n entries.
    assert!(4 + 1 + HOPTRACE_MAX * HOPTRACE_ENTRY_BYTES == 165);
    assert!(4 + 1 + HOPTRACE_MAX * HOPTRACE_ENTRY_BYTES <= 255);
    // H4's flag must not collide with an index. RegHopChannel's channel field is 6 bits wide, and
    // `regs::HOP_CHANNEL_MASK` is the authority on that, so derive the check from it rather than
    // from 0x3F written twice.
    assert!(HOPTRACE_TX_FLAG & regs::HOP_CHANNEL_MASK == 0);
    assert!(HOPTRACE_TX_FLAG == 0x80);
    // The trace is stamped from the SAME counter EVT_RX.ts and CMD_READ_CLOCK use, and EVT_HOPTRACE
    // declares that counter's rate in its own first field. Both are u32 on the wire, so they wrap
    // together and a host can compare them without a unit conversion. (STAMP_HZ itself is pinned to
    // 1 000 000 by the scheduling table above; this pins that the trace shares it.)
    assert!(STAMP_HZ == embassy_time::TICK_HZ as u32);
};

/// **H1/H2 — the hop-event ring.** 32 free-running entries, oldest overwritten, and reading does
/// NOT clear it (the `EVT_SENSE.activity` contract).
///
/// # What this instrument is for
///
/// An LR2021 and this SX1276 both do LoRa intra-packet frequency hopping and cannot hop with each
/// other, and the failure survives a **one-entry** hop list — the hop machinery runs but the carrier
/// never moves — so it is not the sequence, not the phase, and not the frame format (a plain
/// non-hopping receiver decodes either part's hop-mode TX perfectly). What is left is that the two
/// disagree about **when a hop boundary falls**. A period sweep over 2/4/8/16 has already been ruled
/// out on air.
///
/// ★ The thing that makes it measurable at all: **each node timestamps its OWN hop events.** No
/// cross-vendor reception is needed to compare the two timelines, which matters because the link
/// that would have carried the comparison is the very thing that is broken.
///
/// At SF7 / BW 125 kHz a symbol is 2^7/125000 = 1.024 ms, so a nominal 8-symbol hop period is
/// **8.192 ms** and both parts should show that interval. Whatever differs — the interval, the
/// instant of the first hop relative to the start of a frame, or whether hops continue between
/// frames — is the answer.
///
/// # Where the stamp is taken, and what sits between it and the RF boundary (H1)
///
/// The stamp is `Instant::now()` as the **first statement of [`Radio::service_hop`]**, before any
/// of that function's four SPI transactions — those are ~32 µs of byte time at 2 MHz and would
/// otherwise sit between the RF boundary and the timestamp. It is the same `embassy-time` counter
/// `EVT_RX.ts` is filled from and `CMD_READ_CLOCK` returns, so a hop and a frame arrival on this
/// node share one timebase with no conversion.
///
/// **⚠ The offset from the true RF hop instant is NOT MEASURED.** Stating it is the whole point of
/// the instrument, so here is the path, term by term, and which terms are which:
///
/// | term | value | measured? |
/// |---|---|---|
/// | SX1276 RF hop boundary → DIO1 asserts | unknown | **no** — the datasheet gives no figure and there is no register to read it from |
/// | DIO1 high → GPIO interrupt → esp-hal waker → embassy executor → this task resumes | unknown | **no** — never timed on this board |
/// | task resumes → `Instant::now()` | ≤ 1 tick = 1 µs | derived (`STAMP_HZ`) |
/// | main task busy elsewhere when DIO1 asserts | 0 … milliseconds | **no** — see below |
///
/// The last row is the one that can dominate and it is not bounded by anything small: the hop
/// branch is a `select` arm of a single-threaded loop, so if DIO1 asserts while that task is inside
/// a bare SPI or UART statement, the stamp waits for it. The known worst cases in this firmware are
/// `stage_tx`'s ~30-transaction register burst (≈ 1.5-2.7 ms, budgeted at [`STAGE_FIXED_US`]) and
/// `write_all` draining a full 128-byte UART TX FIFO at 115200 (≈ 11 ms). Neither is timed against
/// a hop.
///
/// ★ **The trace makes its own offset falsifiable, which is why it is still worth taking.** Every
/// constant part of that path cancels in a DIFFERENCE of two stamps, so consecutive entries measure
/// the hop INTERVAL with only the variable part left in, and the spread of those intervals over a
/// quiet run is a direct upper bound on the jitter of everything above. Read the interval first;
/// trust an absolute offset only after it has been measured.
///
/// # ☠ A term that is a LOSS, not a latency: a hop can be missing from this ring entirely
///
/// The four rows above are all delays. There is a fifth term and it is different in kind, so it is
/// stated separately rather than folded into the table: **a hop that coincides with RxDone, TxDone
/// or HeaderValid produces no entry at all.** `lora-phy`'s `process_irq_event` is called with
/// `clear_interrupts = true` on the RX and TX paths and clears `RegIrqFlags` with `0xFF`, hop flag
/// included, so when the DIO0 branch of the `select` wins that round the hop's retune is skipped and
/// nothing reaches `service_hop` to be stamped. It is a pre-existing property of the C1 hop path
/// (documented in the README as "one hop can be swallowed"), not something this instrument added —
/// but it lands on the instrument, and a reader differencing consecutive stamps across the gap would
/// read **2 × the hop period and believe it**.
///
/// ★ **`idx` is what makes it detectable, and this is the case that pays for carrying the chip's raw
/// counter instead of a software one:** the modem kept counting through the swallowed hop, so the
/// next entry's `FhssPresentChannel` arrives **2 higher, not 1**. The host's rule is therefore not
/// "difference consecutive stamps" but *"difference consecutive stamps and divide by the `idx`
/// step"* — and any interval whose `idx` step is not 1 is a gap, not a measurement.
///
/// ⚠ The loss is **correlated with frames**, not random: it happens exactly when the modem raises a
/// packet-boundary interrupt, which is the region of the timeline the hop question is about. Its
/// rate has never been counted. The LR2021 node fails differently in the same situation — it records
/// the entry but may stamp it with the frame's instant instead of the hop's, and counts how often
/// (`hoptrace::HopTrace::coalesced`) — so **neither node's trace should be taken while it is also
/// carrying traffic if the timeline can be taken on an idle-hopping node instead.**
///
/// A tighter stamp is possible in principle — a custom GPIO ISR that samples the counter before the
/// executor runs — and is deliberately NOT done here: esp-hal's `Input::wait_for_high` owns that
/// pin's interrupt to drive the async waker, so installing a handler on it would replace the
/// mechanism the cancel-safe hop path (H3) is built on. If the interval jitter above turns out to
/// matter, that is the next step, and it is a pass of its own.
///
/// # What is NOT folded in
///
/// Nothing. No guessed constant offset is added to a stamp, and no interpolation to a symbol
/// boundary is attempted. The event carries the node's own raw counter value and its own tick rate,
/// and the host does the arithmetic — which is also why `stamp_hz` is on the wire instead of a
/// firmware-side conversion to microseconds, which would throw away resolution on a 16 MHz node to
/// match this 1 MHz one.
struct HopTrace {
    /// `Instant::now().as_ticks() as u32` at the top of `service_hop`, in `STAMP_HZ` units. The low
    /// 32 bits, exactly like `EVT_RX.ts`, so the two wrap together (~71 min at 1 MHz).
    t: [u32; HOPTRACE_MAX],
    /// `RegHopChannel` bits 5:0 verbatim, OR'd with [`HOPTRACE_TX_FLAG`] when the hop was taken
    /// while transmitting.
    ///
    /// ☠ **This byte is NOT the same quantity as the LR2021's, though the layout, the mask and the
    /// flag bit are identical.** Here it is the CHIP's own counter, read out of `RegHopChannel`;
    /// there it is a FIRMWARE count of recorded hop interrupts, because the LR2021 exposes no
    /// hop-index register at all (`SetLoraHopping` is commented out of its command spec and there
    /// is no `GetLoraHopStatus`). Two consequences the host must hold on to:
    ///
    /// * only THIS node's `idx` can reveal a hop the MCU coalesced (the field jumps by more than 1)
    ///   or a per-packet reset of the modem's counter — the other node's advances once per
    ///   *recorded* event by construction and can show neither;
    /// * therefore ★ **compare `t_ticks` across the two nodes; compare `idx` only within one.** A
    ///   difference in the first `idx` value between the parts is a fact about the LR2021
    ///   firmware's counter, not about its silicon.
    idx: [u8; HOPTRACE_MAX],
    /// Next slot to write.
    head: usize,
    /// Entries held, saturating at [`HOPTRACE_MAX`]. Not a count of hops ever taken — that is
    /// `Hop::hops`, which the host differences.
    len: usize,
}

impl HopTrace {
    const fn new() -> Self {
        Self { t: [0; HOPTRACE_MAX], idx: [0; HOPTRACE_MAX], head: 0, len: 0 }
    }

    /// Record one hop event. **Pure arithmetic on RAM — no SPI, no await, no allocation** — which
    /// is what lets it be called from inside `service_hop` without adding a suspension point to a
    /// future that must run to completion (H3).
    ///
    /// `chan` is `RegHopChannel & HOP_CHANNEL_MASK` as read by the caller; this function does not
    /// re-read it and does not mask it again beyond dropping the flag bit's room, so the value
    /// stored is the chip's, not a derived one.
    fn push(&mut self, t: Instant, chan: u8, tx_active: bool) {
        let mut b = chan & regs::HOP_CHANNEL_MASK;
        if tx_active {
            b |= HOPTRACE_TX_FLAG;
        }
        self.t[self.head] = t.as_ticks() as u32;
        self.idx[self.head] = b;
        self.head = (self.head + 1) % HOPTRACE_MAX;
        if self.len < HOPTRACE_MAX {
            self.len += 1;
        }
    }

    /// Serialise into an `EVT_HOPTRACE` payload: `[stamp_hz u32 BE][n u8][idx u8, t u32 BE]*n`,
    /// **most recent LAST**. Returns the used length of `out`.
    ///
    /// Reading does not disturb the ring — no head reset, no length reset, no clear — so two reads
    /// with hops in between overlap, and the host differences or de-duplicates on the stamps. That
    /// is the `EVT_SENSE.activity` contract, and it is what makes a read safe to repeat while a
    /// measurement is running.
    fn encode(&self, out: &mut [u8; 4 + 1 + HOPTRACE_MAX * HOPTRACE_ENTRY_BYTES]) -> usize {
        out[0..4].copy_from_slice(&STAMP_HZ.to_be_bytes());
        out[4] = self.len as u8;
        // Oldest first: with a full ring the oldest entry is the one `head` is about to overwrite.
        let start = (self.head + HOPTRACE_MAX - self.len) % HOPTRACE_MAX;
        for k in 0..self.len {
            let i = (start + k) % HOPTRACE_MAX;
            let o = 5 + k * HOPTRACE_ENTRY_BYTES;
            out[o] = self.idx[i];
            out[o + 1..o + 5].copy_from_slice(&self.t[i].to_be_bytes());
        }
        5 + self.len * HOPTRACE_ENTRY_BYTES
    }

    /// How many of the entries held were taken while TRANSMITTING (H4). Reported in the
    /// `CMD_SET_DEBUG` dump so the TX/idle-RX split is visible without pulling the whole trace.
    fn tx_flagged(&self) -> usize {
        let start = (self.head + HOPTRACE_MAX - self.len) % HOPTRACE_MAX;
        let mut c = 0;
        for k in 0..self.len {
            if self.idx[(start + k) % HOPTRACE_MAX] & HOPTRACE_TX_FLAG != 0 {
                c += 1;
            }
        }
        c
    }
}

/// SX1276 LoRa spreading factors this firmware operates. SF6 is excluded: it requires an implicit
/// header, which would break interop with the fleet's explicit-header frames.
const SF_MIN: u8 = 7;
const SF_MAX: u8 = 12;

/// The band this node is built and matched for — the same 902-928 MHz US ISM window the Waveshare
/// nodes advertise, so a host can plan a common channel across the fleet.
///
/// NOT the SX1276's silicon range (137-1020 MHz): this board is the 915 MHz Heltec variant, whose
/// PA matching network, SAW filter and antenna are tuned for 902-928. Advertising the silicon range
/// would invite the host to tune somewhere this board radiates almost nothing.
const FREQ_MIN_HZ: u32 = 902_000_000;
const FREQ_MAX_HZ: u32 = 928_000_000;

/// TX power range in **real dBm**, the interval `lora-phy`'s `Sx1276::set_tx_power` clamps to when
/// `tx_boost` is on — which it is, because the Heltec V2 wires the antenna to PA_BOOST (the RFO pin
/// is not connected). Source: `lora_phy::sx127x::sx1276`, "Output via PA_BOOST: [2, 20] dBm";
/// above +17 dBm the driver enables PaDac 20 dBm mode and raises OCP to 240 mA.
const PWR_MIN_DBM: i8 = 2;
const PWR_MAX_DBM: i8 = 20;

// =================================================================================================
// C1 — the canonical bandwidth code space, pinned.
// =================================================================================================

/// **The canonical 7E-A5 bandwidth code: `0 = 125 kHz, 1 = 250 kHz, 2 = 500 kHz`.**
///
/// This is the space the host already computes in (`lora_serial.rs::set_bandwidth_khz` maps kHz to
/// 0/1/2) and the space `EVT_INFO.bw` reports back, so it is the one the whole fleet pins to.
///
/// Bug C1: the host then ran that through `bw_to_fw`, which re-encoded it into the **SX1262
/// modulation-code** space (0x04/0x05/0x06) before putting it on the wire, while this firmware
/// decoded `1 => 250k, 2 => 500k, _ => 125k`. Every value therefore fell through to 125 kHz —
/// `set_bandwidth_khz` was a no-op and 250/500 kHz were unreachable from the host.
///
/// The fix decodes **both** spaces. They are disjoint (0/1/2 vs 4/5/6), so accepting the legacy
/// SX1262 codes as well costs nothing and means a host that has not yet been updated still gets the
/// bandwidth it asked for instead of silently getting 125 kHz. `EVT_INFO` always answers in the
/// canonical space, so the host can see which width was actually applied.
const fn bw_hz_of_code(code: u8) -> u32 {
    match code {
        1 | 0x05 => 250_000, // canonical 1 | legacy SX1262 BW_250
        2 | 0x06 => 500_000, // canonical 2 | legacy SX1262 BW_500
        _ => 125_000,        // canonical 0 | legacy SX1262 BW_125 (0x04) | anything unknown
    }
}

/// Inverse of [`bw_hz_of_code`] in the canonical space — what `EVT_INFO.bw` reports.
const fn bw_code_of_hz(hz: u32) -> u8 {
    match hz {
        250_000 => 1,
        500_000 => 2,
        _ => 0,
    }
}

/// **The table test that pins the code space**, checked by the compiler on every single build.
///
/// A `#[cfg(test)]` module could not do this job: the crate is `no_std`/`no_main` for
/// `xtensa-esp32-none-elf`, so `cargo test` cannot build it and such a test would never run. A
/// `const` block is evaluated during the real firmware build, so this mapping cannot regress
/// unnoticed the way C1 did.
const _: () = {
    assert!(bw_hz_of_code(0) == 125_000);
    assert!(bw_hz_of_code(1) == 250_000);
    assert!(bw_hz_of_code(2) == 500_000);
    // Legacy SX1262 modulation codes the shipped host emits — the exact values that made C1 silent.
    assert!(bw_hz_of_code(0x04) == 125_000);
    assert!(bw_hz_of_code(0x05) == 250_000);
    assert!(bw_hz_of_code(0x06) == 500_000);
    // Unknown codes must fail safe to the narrowest, most robust width.
    assert!(bw_hz_of_code(0x7F) == 125_000);
    // Round-trip: what we decode is what EVT_INFO reports back.
    assert!(bw_code_of_hz(bw_hz_of_code(0)) == 0);
    assert!(bw_code_of_hz(bw_hz_of_code(1)) == 1);
    assert!(bw_code_of_hz(bw_hz_of_code(2)) == 2);
    assert!(bw_code_of_hz(bw_hz_of_code(0x05)) == 1);
    assert!(bw_code_of_hz(bw_hz_of_code(0x06)) == 2);
};

fn bw_from(code: u8) -> Bandwidth {
    match bw_hz_of_code(code) {
        250_000 => Bandwidth::_250KHz,
        500_000 => Bandwidth::_500KHz,
        _ => Bandwidth::_125KHz,
    }
}

fn bw_hz(bw: Bandwidth) -> u32 {
    match bw {
        Bandwidth::_250KHz => 250_000,
        Bandwidth::_500KHz => 500_000,
        _ => 125_000,
    }
}

fn sf_from(n: u8) -> SpreadingFactor {
    match n {
        7 => SpreadingFactor::_7,
        8 => SpreadingFactor::_8,
        9 => SpreadingFactor::_9,
        10 => SpreadingFactor::_10,
        11 => SpreadingFactor::_11,
        12 => SpreadingFactor::_12,
        _ => SpreadingFactor::_9, // out of range -> the fleet default, reported back in EVT_INFO
    }
}

fn sf_num(sf: SpreadingFactor) -> u8 {
    match sf {
        SpreadingFactor::_5 => 5,
        SpreadingFactor::_6 => 6,
        SpreadingFactor::_7 => 7,
        SpreadingFactor::_8 => 8,
        SpreadingFactor::_9 => 9,
        SpreadingFactor::_10 => 10,
        SpreadingFactor::_11 => 11,
        SpreadingFactor::_12 => 12,
    }
}

fn cr_from(code: u8) -> CodingRate {
    match code {
        2 => CodingRate::_4_6,
        3 => CodingRate::_4_7,
        4 => CodingRate::_4_8,
        _ => CodingRate::_4_5,
    }
}

fn cr_num(cr: CodingRate) -> u8 {
    match cr {
        CodingRate::_4_5 => 1,
        CodingRate::_4_6 => 2,
        CodingRate::_4_7 => 3,
        CodingRate::_4_8 => 4,
    }
}

/// LoRa symbol duration in microseconds: `2^SF / BW`. Sets the CAD watchdog, which must therefore
/// scale with the modulation — the old fixed 60 ms bound was shorter than two SF12/125 kHz symbols.
fn tsym_us(sf: u8, bw_hz: u32) -> u64 {
    ((1u64 << sf) * 1_000_000) / bw_hz as u64
}

/// LoRa time-on-air in whole milliseconds (rounded up), explicit header + CRC on. Same Semtech
/// formula and same integer arithmetic as the Waveshare node's `sx1262::airtime_ms`, so the two
/// report comparable numbers. Feeds `EVT_TX_STARTED` and the TX watchdog.
fn airtime_ms(sf: u8, bw_hz: u32, cr: u8, payload_len: usize, preamble: u16) -> u32 {
    let sf_i = sf as i64;
    let de: i64 = if sf >= 11 && bw_hz == 125_000 { 1 } else { 0 };
    let cr_i = cr as i64;
    let pl = payload_len as i64;
    let num = 8 * pl - 4 * sf_i + 28 + 16;
    let den = 4 * (sf_i - 2 * de);
    let steps = if num <= 0 || den <= 0 { 0 } else { num.div_euclid(den) + i64::from(num % den != 0) };
    let payload_sym = 8 + steps * (cr_i + 4);
    let tsym_us: u64 = ((1u64 << sf) * 1_000_000) / bw_hz as u64;
    let preamble_us = tsym_us * (4 * preamble as u64 + 17) / 4;
    let payload_us = tsym_us * payload_sym as u64;
    ((preamble_us + payload_us) / 1000 + 1) as u32
}

// =================================================================================================
// Host link
// =================================================================================================

/// A parsed host command frame. `len <= 255` because the wire length field is one byte.
#[derive(Clone, Copy)]
struct Frame {
    typ: u8,
    len: usize,
    payload: [u8; 255],
}

/// Command frames the UART reader task hands to the radio task. Depth 4 plus the UART's 128-byte
/// hardware FIFO is the slack that keeps host commands alive while a long SF12 transmission runs.
static CMDQ: Channel<CriticalSectionRawMutex, Frame, 4> = Channel::new();

/// C5: host bytes lost — every `RxError` the reader saw (a FIFO overflow is one error, not one byte,
/// so treat this as "the link dropped something", not as a byte count). Reported as `EVT_INFO.lost`.
/// Should stay 0; a climbing count is the link telling you it is losing commands, which used to be
/// completely invisible because `read_frame` did `if let Ok(n) = ...` and discarded every `Err`.
static SERIAL_LOST: AtomicU32 = AtomicU32::new(0);

/// Incremental `7E A5 | type | len | payload | crc` parser. Resyncs on the sync bytes and validates
/// the XOR checksum, so a dropped byte costs at most one frame.
struct Parser {
    state: u8,
    typ: u8,
    len: usize,
    idx: usize,
    crc: u8,
    buf: [u8; 255],
}

impl Parser {
    const fn new() -> Self {
        Self { state: 0, typ: 0, len: 0, idx: 0, crc: 0, buf: [0; 255] }
    }

    fn push(&mut self, b: u8) -> Option<Frame> {
        match self.state {
            0 => {
                if b == SYNC0 {
                    self.state = 1;
                }
            }
            1 => {
                self.state = if b == SYNC1 {
                    2
                } else if b == SYNC0 {
                    1
                } else {
                    0
                }
            }
            2 => {
                self.typ = b;
                self.crc = b;
                self.state = 3;
            }
            3 => {
                self.len = b as usize;
                self.crc ^= b;
                self.idx = 0;
                self.state = if self.len == 0 { 5 } else { 4 };
            }
            4 => {
                if self.idx < self.buf.len() {
                    self.buf[self.idx] = b;
                }
                self.idx += 1;
                self.crc ^= b;
                if self.idx >= self.len {
                    self.state = 5;
                }
            }
            5 => {
                self.state = 0;
                if b == self.crc {
                    let mut f = Frame { typ: self.typ, len: self.len, payload: [0; 255] };
                    f.payload[..self.len].copy_from_slice(&self.buf[..self.len]);
                    return Some(f);
                }
            }
            _ => self.state = 0,
        }
        None
    }
}

/// Drains UART0's RX FIFO continuously and hands complete command frames to the radio task.
///
/// **Why its own task (C5).** The 128-byte RX FIFO holds ~11 ms of traffic at 115200. A single SF12
/// transmission can occupy the radio task for seconds, so a main loop that only reads the UART
/// between radio operations loses every command that arrives during a transmission — the same defect
/// the Waveshare node fixed with a USART ISR. Here the reader runs independently, so a host command
/// is taken off the wire while the radio is busy and is executed the moment the radio is free.
///
/// Every `RxError` is counted (C5). `read_async` resets the FIFO itself after an overflow, so
/// continuing the loop is the correct recovery; the parser resyncs on the next `7E A5`.
#[embassy_executor::task]
async fn uart_reader(mut rx: UartRx<'static, Async>) {
    let mut parser = Parser::new();
    let mut buf = [0u8; 64];
    loop {
        match rx.read_async(&mut buf).await {
            Ok(n) => {
                for &b in &buf[..n] {
                    if let Some(f) = parser.push(b) {
                        // `send` (not `try_send`) so a burst of commands queues instead of being
                        // dropped; if the radio task is deep in a transmission the FIFO absorbs the
                        // overflow and the error path above counts what genuinely could not fit.
                        CMDQ.send(f).await;
                    }
                }
            }
            Err(_) => {
                SERIAL_LOST.fetch_add(1, Ordering::Relaxed);
            }
        }
    }
}

/// `write_async` only takes what fits in the 128-byte TX FIFO and returns how much that was; the old
/// `send_frame` ignored the count, so any event bigger than the free space was silently truncated on
/// the wire (a 247-byte EVT_RX would lose its tail). Loop until the slice is gone.
async fn write_all(tx: &mut UartTx<'static, Async>, mut bytes: &[u8]) {
    while !bytes.is_empty() {
        match tx.write_async(bytes).await {
            Ok(0) => break, // cannot make progress; dropping is better than spinning forever
            Ok(n) => bytes = &bytes[n..],
            Err(_) => break, // TxError is an uninhabited enum on this HAL; here for completeness
        }
    }
}

async fn send_frame(tx: &mut UartTx<'static, Async>, typ: u8, payload: &[u8]) {
    let len = payload.len() as u8;
    let mut crc = typ ^ len;
    for &b in payload {
        crc ^= b;
    }
    write_all(tx, &[SYNC0, SYNC1, typ, len]).await;
    if !payload.is_empty() {
        write_all(tx, payload).await;
    }
    write_all(tx, &[crc]).await;
}

/// Formats into a fixed stack buffer so EVT_LOG payloads can be built with `write!`.
struct BufWriter {
    buf: [u8; 96],
    pos: usize,
}
impl BufWriter {
    fn new() -> Self {
        Self { buf: [0; 96], pos: 0 }
    }
    fn as_slice(&self) -> &[u8] {
        &self.buf[..self.pos]
    }
}
impl core::fmt::Write for BufWriter {
    fn write_str(&mut self, s: &str) -> core::fmt::Result {
        for &b in s.as_bytes() {
            if self.pos < self.buf.len() {
                self.buf[self.pos] = b;
                self.pos += 1;
            }
        }
        Ok(())
    }
}

// =================================================================================================
// Radio
// =================================================================================================

/// **C1 — the intra-packet frequency-hopping state**, and the reason this node is the one that got
/// the feature.
///
/// # Why here
///
/// This board retunes in **5 597 µs** (MEASURED 2026-08-28, host command to `EVT_INFO`), against
/// 52 798 µs on the LR2021 and 160 866 µs on the Waveshare SX1262. But a *host-driven* retune is
/// still a serial round trip, and the measured floor for one of those on this node is 10 733 µs —
/// two orders of magnitude longer than a LoRa symbol. Hopping INSIDE a packet cannot be done from
/// a host at all; it has to be done by the modem, with the MCU only feeding it the next carrier.
/// That is precisely what the SX1276's FHSS mode is: `RegHopPeriod` symbols of dwell, then a
/// `FhssChangeChannel` interrupt, and the host has one hop period to write the next `RegFrf`.
///
/// # The three registers, and what `lora-phy` had to do with it: nothing
///
/// * `RegHopPeriod` (0x24) — dwell in symbols, 0 = off. `lora-phy` has no entry for this register.
/// * `RegFrf` (0x06/0x07/0x08) — the carrier, rewritten on each interrupt from [`Hop::frf`].
/// * `RegHopChannel` (0x1C) — the modem's own hop counter (bits 5:0) and its PLL-lock verdict
///   (bit 7). Read on each interrupt, because the CHIP's counter is the authority on which entry of
///   the list is next; a software counter of our own would drift the moment one interrupt was
///   serviced late or a packet restarted the sequence.
///
/// # The index semantics, and where they come from
///
/// `(RegHopChannel & 0x3F)` is used directly as the index into the hop list, modulo its length.
/// That is the Semtech reference driver's own pattern — `SX1276OnDio1Irq` clears the flag and hands
/// `SX1276Read(REG_LR_HOPCHANNEL) & RFLR_HOPCHANNEL_CHANNEL_MASK` to the application, which calls
/// `SetChannel(HoppingFrequencies[index])`. It is a SOURCE pattern, not a measurement: this
/// firmware has not yet watched a hop sequence on air, and the on-air confirmation (that both ends
/// of a pair walk the same list in the same order) is the first thing to check when this reaches
/// hardware.
///
/// # ⚠ One hop can be swallowed, and the sequence still does not desync
///
/// `lora-phy`'s `process_irq_event` is called with `clear_interrupts = true` on both the RX and the
/// TX path, and it clears `RegIrqFlags` with `0xFF` — the hop flag included. So if RxDone or TxDone
/// and a `FhssChangeChannel` land close enough together that the DIO0 branch of a `select` wins,
/// that one hop's `RegFrf` write is skipped and the modem spends that dwell on the previous
/// carrier. **This is exactly why the list index comes from `RegHopChannel` and not from a counter
/// of ours:** the chip kept counting, so the very next serviced hop lands on the right entry and
/// the sequence re-synchronises by itself. The cost is one dwell, not a desynchronised link. Left
/// as is rather than engineered around, because the alternative is taking over IRQ-flag clearing
/// from the driver on the two paths this firmware most depends on.
///
/// # ⚠ What enabling this costs, stated up front
///
/// In FHSS the RECEIVER hops too — it must, to follow the transmitter — so a hopping node is deaf
/// to a non-hopping peer, and the interrupt fires every `period` symbols forever, not only during a
/// transmission. At SF9/125 kHz (T_sym 4.096 ms) with `period = 16` that is ~15 interrupts a second
/// at ~40 µs of SPI each: nothing. At `period = 1` with SF7/500 kHz (T_sym 256 µs) it is ~3 900 a
/// second, ~16% of the SPI budget, and the node has other work. The firmware does not forbid it;
/// it is the host's dial and this is what it costs.
struct Hop {
    /// Precomputed `RegFrf` words — [`regs::frf_of_hz`] evaluated ONCE, at `CMD_SET_HOP`, so the
    /// interrupt service performs three register writes and no arithmetic at all.
    frf: [u32; HOP_MAX],
    /// The same channels in the host's own units, kept only so the debug dump can quote a hop back
    /// in Hz instead of in synthesiser words.
    hz: [u32; HOP_MAX],
    /// Length of the list. 0 when the host has never set one.
    n: usize,
    /// `RegHopPeriod` in symbols, as programmed. 0 <=> hopping is off in the chip.
    period: u8,
    /// True between `CMD_SET_HOP 1` and `CMD_SET_HOP 0`. **This is the flag that gates every extra
    /// SPI transaction and every extra `select` branch in the firmware** — while it is false, C1
    /// costs exactly nothing, which is what protects the 5.6 ms retune (C4).
    enabled: bool,
    /// Hop interrupts serviced, wrapping. The host differences it, like `EVT_SENSE.activity`.
    hops: u32,
    /// Hops on which `RegHopChannel` bit 7 said the PLL had not locked in time — i.e. the modem
    /// missed the hop. A real, chip-sourced failure count, not an inference.
    pll_timeouts: u32,
    /// The last list index actually programmed, for the debug dump.
    last_idx: u8,
    /// ★ H1/H2 — the timestamped hop-event ring. See [`HopTrace`], which carries the whole
    /// argument for why it exists and where in the path the stamp is taken.
    trace: HopTrace,
    /// ☠ **The carrier to go back to when hopping is switched off.**
    ///
    /// Enabling overwrites `st.p.freq_hz` with `hz[0]`, because hop channel 0 is where a packet
    /// starts. Nothing used to remember what it overwrote, so disabling left the node on the hop
    /// list's base instead of the frequency the host had configured — MEASURED: with a list starting
    /// at 902 MHz, a node told `CMD_SET_FREQ 915` then hop-on then hop-off transmitted at 902 while
    /// `CMD_TX` answered `ok=1` and every peer heard nothing.
    ///
    /// 0 means "never enabled", in which case there is nothing to restore.
    base_hz: u32,
}

impl Hop {
    const fn new() -> Self {
        Self {
            frf: [0; HOP_MAX],
            hz: [0; HOP_MAX],
            n: 0,
            period: 0,
            enabled: false,
            hops: 0,
            pll_timeouts: 0,
            last_idx: 0,
            trace: HopTrace::new(),
            base_hz: 0,
        }
    }
}

type Iv = GenericSx127xInterfaceVariant<Output<'static>, Input<'static>>;
type Chip = Sx127x<SharedSpi, Iv, Sx1276>;

/// The SX1276, driven through `lora-phy`'s **`RadioKind` trait** rather than its `LoRa` facade.
///
/// # C3 — why this cannot cancel mid-flow
///
/// `lora-phy` documents on `process_irq_event`: *"NB! Do not await this future in a select branch as
/// interrupting it mid-flow could cause radio lock up."* The old main loop did exactly that —
/// `select(read_frame(..), lora.rx(..))` — and `LoRa::rx` is `do_rx()` followed by
/// `wait_for_irq()`/`process_irq_event()`/`get_rx_payload()`. A host byte arriving at the wrong
/// moment dropped that future in the middle of an SPI transaction or between reading the IRQ flags
/// and clearing them.
///
/// It cannot be fixed while `LoRa` owns the sequence, because `LoRa` exposes no way to wait for the
/// IRQ separately from consuming it. `RadioKind` does: `do_rx`, `await_irq`, `process_irq_event`,
/// `get_rx_payload` and `get_rx_packet_status` are all public. So the loop is split at the only
/// cancel-safe seam:
///
/// ```text
///   arm_rx()      <- awaited BARE, never inside a select; chip enters RXCONTINUOUS
///   select(host command, wait_irq())    <- the ONLY cancellable await
///   take_rx()     <- awaited BARE, runs to completion once DIO0 is already high
/// ```
///
/// [`Radio::wait_irq`] is nothing but `Input::wait_for_high()` on DIO0: no SPI, no chip state, no
/// driver state. Dropping it unlistens a GPIO interrupt and nothing else, and because the event is
/// **level**-triggered (not edge), re-arming it while DIO0 is still high fires immediately — so a
/// cancelled wait cannot lose an interrupt either. Every future that does touch SPI
/// (`arm_rx`, `take_rx`, `transmit`, `cad`) is awaited as a bare statement and runs to completion.
///
/// The two places that still need a *timeout* (CAD and TX, where a lost IRQ would otherwise hang the
/// node forever) race the timer against `await_irq` **only**, never against the SPI phase.
///
/// # P1 — the scheduled TX joins that discipline, it does not break it
///
/// [`Radio::stage_tx`] and [`Radio::fire_tx`] are the only futures P1 adds that touch SPI, and both
/// are awaited as **bare statements** in the timer arm of [`main`]'s loop, *after* the `select` has
/// already resolved. Neither is ever a `select` branch. What the deadline timer races is
/// `CMDQ.receive()` (a channel) and — only while no frame is staged — `Radio::wait_irq` (a GPIO
/// level wait). While a frame IS staged the loop drops to a two-way `select` that omits `wait_irq`
/// entirely, so `process_irq_event` cannot be entered with the mode mirror pointing at `Transmit`
/// while the chip idles in standby. See the `SchedTx` handling in [`main`].
struct Radio {
    chip: Chip,
    /// Second handle on the same SPI bus, for the registers `lora-phy` does not expose. See
    /// [`regs`].
    raw: SharedSpi,
    /// Mirror of the chip's mode; `RadioKind` is stateless, so the caller must track it (this is
    /// exactly what `LoRa` does internally).
    mode: RadioMode,
    /// P2 — host-set CAD/preamble detector, as `(full RegDetectOptimize byte, RegDetectionThreshold)`.
    /// `None` until `CMD_SET_CAD_CFG` arrives, and while it is `None` not one extra SPI transaction
    /// is issued anywhere (which is what keeps the 5.6 ms retune at 5.6 ms — see P5).
    ///
    /// It has to be re-applied rather than written once: `lora-phy`'s `set_modulation_params`
    /// overwrites BOTH registers from the spreading factor on every call, and `arm_rx`, `cad` and
    /// `stage_tx` all call it. A write-once knob here would have been inert — decided, tested,
    /// documented and reaching no actuator.
    det: Option<(u8, u8)>,
    /// P3 — host-set `RegLna`, or `None` to leave `lora-phy`'s own arm-time write alone. Same
    /// re-apply reasoning: `do_rx` and `do_cad` each write `RegLna` from the fixed
    /// `Sx127xConfig::rx_boost` flag every single time they run.
    lna: Option<u8>,
    /// ★ C1 — **DIO1**, the pin the hop interrupt arrives on. `lora-phy` never sees it: its
    /// `GenericSx127xInterfaceVariant` is built with DIO0 and the two RF-switch outputs only, so
    /// this `Input` is owned here and waited on directly.
    ///
    /// **The board fact:** the Heltec WiFi LoRa 32 V2 wires SX1276 DIO1 to **GPIO35**. That is not
    /// read off a datasheet — it is this repository's own attested pinout, from the working
    /// RadioLib firmware `firmware/heltec-lora-node/heltec-lora-node.ino:27` (`#define LORA_DIO1
    /// 35`, passed to `new Module(LORA_CS, LORA_DIO0, LORA_RST, LORA_DIO1)`). DIO2 is NOT wired to
    /// any pin this project has ever verified, which is the whole reason the mapping below is on
    /// DIO1: on the SX1276 DIO2 would have been the cheaper choice (all three of its encodings mean
    /// FhssChangeChannel, so it needs no mapping write at all), but a GPIO number nobody here has
    /// confirmed is an invention, and an invented pin is a hop interrupt that never arrives.
    ///
    /// GPIO35 is one of the ESP32's input-only pins (34-39) with no internal pull resistors, which
    /// is exactly right for an IRQ line the SX1276 drives push-pull.
    hop_irq: Input<'static>,
    /// C1 — the hop list and its counters. See [`Hop`].
    hop: Hop,
}

/// What resolved the wait inside [`Radio::fire_tx`]. Collapses the two `select` shapes — three-way
/// while hopping, two-way otherwise — into one value to dispatch on, and, by being a plain local,
/// guarantees the `select`'s borrows are gone before a SPI-touching future is awaited (C3).
enum FireWake {
    Irq(Result<(), RadioError>),
    /// C1 — `FhssChangeChannel` on DIO1: the carrier has to move mid-packet.
    Hop,
    Timeout,
}

/// The `RegLna` byte `lora-phy` writes on every `do_rx`/`do_cad` in THIS build, because [`main`]
/// constructs `Sx127xConfig { .., rx_boost: true }` and the driver then uses
/// `LnaGain::G1.boosted_value()`. An override equal to this needs no write at all, which is why
/// `CMD_SET_RX_GAIN 1` costs zero SPI and only `CMD_SET_RX_GAIN 0` adds a transaction.
const LORA_PHY_ARM_LNA: u8 = regs::lna_reg(true);

const _: () = {
    assert!(LORA_PHY_ARM_LNA == 0x23);
    assert!(regs::lna_reg(false) != LORA_PHY_ARM_LNA); // the knob must be able to change something
};

impl Radio {
    /// Reset + LoRa-mode init. Mirrors `LoRa::init` / `do_cold_start` step for step.
    async fn init(&mut self) -> Result<(), RadioError> {
        self.chip.reset(&mut Delay).await?; // also enters sleep, which latches the LoRa mode bit
        self.chip.set_standby().await?;
        self.mode = RadioMode::Standby;
        self.chip.init_lora(false).await?; // private network => sync word 0x12, matching the fleet
        self.chip.set_tx_power_and_ramp_time(0, None, false).await?;
        self.chip.set_irq_params(Some(RadioMode::Standby)).await?;
        Ok(())
    }

    async fn to_standby(&mut self) -> Result<(), RadioError> {
        if self.mode != RadioMode::Standby {
            self.chip.set_standby().await?;
            self.mode = RadioMode::Standby;
        }
        Ok(())
    }

    /// Arm continuous RX. Awaited bare — never inside a `select` (C3).
    async fn arm_rx(
        &mut self,
        m: &ModulationParams,
        rx_pkt: &PacketParams,
        freq_hz: u32,
    ) -> Result<(), RadioError> {
        self.to_standby().await?;
        self.chip.set_modulation_params(m).await?;
        // P2: `set_modulation_params` just overwrote RegDetectOptimize/RegDetectionThreshold from
        // the SF. Put the host's detector back. Zero SPI while `det` is None (the default).
        self.apply_detector().await?;
        self.chip.set_packet_params(rx_pkt).await?;
        self.chip.set_channel(freq_hz).await?;
        self.mode = RadioMode::Receive(RxMode::Continuous);
        self.chip.set_irq_params(Some(self.mode)).await?;
        // C1: `set_irq_params` just rewrote RegIrqFlagsMask from the radio mode, and every one of
        // its arms MASKS FhssChangedChannel. Put the hop interrupt back. Zero SPI while hopping is
        // off (the default), which is what keeps this — the retune path — at 5.6 ms.
        self.apply_hop().await?;
        self.chip.do_rx(RxMode::Continuous).await?;
        // P3: `do_rx` wrote RegLna from `Sx127xConfig::rx_boost`. Put the host's gain back. Zero SPI
        // while `lna` is None or equals what the driver already wrote.
        self.apply_lna().await
    }

    /// Re-apply the host-set detector pair, if there is one. Two plain writes and no read — the
    /// read-modify-write that preserved RegDetectOptimize's reserved bits happened once, when
    /// `CMD_SET_CAD_CFG` was handled.
    async fn apply_detector(&mut self) -> Result<(), RadioError> {
        if let Some((opt, thresh)) = self.det {
            self.raw
                .reapply_detector(opt, thresh)
                .await
                .map_err(|_| RadioError::SPI)?;
        }
        Ok(())
    }

    /// Re-apply the host-set LNA byte, if it differs from what `lora-phy` writes on every arm.
    async fn apply_lna(&mut self) -> Result<(), RadioError> {
        if let Some(v) = self.lna {
            if v != LORA_PHY_ARM_LNA {
                self.raw
                    .write_reg(regs::REG_LNA, v)
                    .await
                    .map_err(|_| RadioError::SPI)?;
            }
        }
        Ok(())
    }

    // ---------------------------------------------------------------------------------------
    // C1 — the hop path
    // ---------------------------------------------------------------------------------------

    /// Re-open the hop interrupt after `lora-phy` has closed it.
    ///
    /// `set_irq_params` rewrites `RegIrqFlagsMask` wholesale in every one of its four arms and
    /// masks `FhssChangedChannel` in all of them, so this has to run after each arm rather than
    /// once. **Costs exactly zero SPI transactions while hopping is off** — the single `if` is the
    /// whole of C1's footprint on the retune path (C4).
    async fn apply_hop(&mut self) -> Result<(), RadioError> {
        if self.hop.enabled {
            self.raw.unmask_hop_irq().await.map_err(|_| RadioError::SPI)?;
        }
        Ok(())
    }

    /// **The only cancel-safe await C1 adds: DIO1 going high.**
    ///
    /// Byte for byte the same construction as [`Radio::wait_irq`] — `Input::wait_for_high` on a
    /// GPIO — so the C3 argument transfers unchanged: it touches no SPI, no chip state and no
    /// driver state, dropping it unlistens a GPIO interrupt and nothing else, and because esp-hal
    /// implements it as a **level** event (`Event::HighLevel`), re-arming it while DIO1 is still
    /// high fires immediately, so a cancelled wait cannot lose a hop either.
    ///
    /// (Kept for symmetry and documentation. The main loop's four-way `select` calls
    /// `radio.hop_irq.wait_for_high()` as a field borrow instead, because two `&mut radio` method
    /// calls in one expression would not borrow-check against `radio.chip.await_irq()`.)
    #[allow(dead_code)]
    async fn hop_wait(&mut self) {
        self.hop_irq.wait_for_high().await
    }

    /// **Service one `FhssChangeChannel` interrupt: retune the carrier to the next hop.**
    ///
    /// Four SPI transactions, 8 bytes, ~32 µs of byte time at 2 MHz, and no arithmetic — the
    /// `RegFrf` word was computed when the host set the list. The order is the Semtech reference
    /// driver's: clear the flag, ask the chip which hop it is on, program that hop's carrier.
    ///
    /// # Why this cannot cancel an SPI future mid-flow (the C3 discipline, unbroken)
    ///
    /// This future is **never a `select` branch**. It is awaited as a bare statement in both places
    /// a hop can arrive — the main loop's `Wake::Hop` arm and the `FireWake::Hop` arm inside
    /// [`Radio::fire_tx`] — in each case *after* the `select` has already resolved, so nothing can
    /// drop it partway. What the selects race is only ever GPIO level waits and timers.
    ///
    /// The flag is cleared FIRST, and with `RegIrqFlags <- 0x02` rather than `<- 0xFF`: writing a 0
    /// to a bit of that register leaves it alone, so a latched RxDone or TxDone that arrived while
    /// we were servicing the hop survives to be consumed by `lora-phy`'s `process_irq_event`.
    /// Clearing first is also what drops DIO1 before we return, so the level wait cannot re-fire on
    /// the hop we just handled.
    ///
    /// # ★ H1 — where the timestamp is taken, and what is still between it and the RF boundary
    ///
    /// `Instant::now()` is the **first statement of this function**, before the flag-clear write.
    /// That is as early as this function can be: the four SPI transactions below are ~32 µs of byte
    /// time at 2 MHz and would otherwise sit inside the measurement. It is the same `embassy-time`
    /// counter `EVT_RX.ts` is filled from and `CMD_READ_CLOCK` returns, so hops and frame arrivals
    /// on this node share one timebase and `EVT_HOPTRACE.stamp_hz` describes both.
    ///
    /// ⚠ What remains between the true RF hop instant and this stamp is **NOT MEASURED**: the
    /// SX1276's own RF-boundary → DIO1 delay (no datasheet figure, no register to read it from),
    /// and the DIO1 edge → GPIO interrupt → esp-hal waker → embassy executor → task-resume path
    /// (never timed on this board). Both are stated as unknown rather than budgeted, because a
    /// guessed offset folded into a stamp is exactly what would make this trace unable to answer a
    /// timing question. On top of them sits a variable term that can dominate: this is a `select`
    /// arm of a single-threaded loop, so a hop that lands while the task is inside a bare SPI or
    /// UART statement waits for it (worst known: ~11 ms for `write_all` draining a full UART TX
    /// FIFO). The constant terms cancel in the DIFFERENCE of two stamps, so the hop INTERVAL — the
    /// number the SX1276-vs-LR2021 comparison actually turns on — carries only the variable part,
    /// and its spread over a quiet run bounds that part directly. Full accounting at [`HopTrace`].
    ///
    /// # ★ H4 — `tx_active`
    ///
    /// `true` iff the caller is [`Radio::fire_tx`], i.e. the chip is in TX with a frame in flight;
    /// `false` from the main loop, which is idle RX or a reception in progress. It is recorded in
    /// bit 7 of the trace's `idx` byte ([`HOPTRACE_TX_FLAG`]) because in FHSS the receiver hops too,
    /// so a frame's own hops arrive embedded in an unbroken run of idle-RX hops and nothing in a
    /// raw count separates them. The caller is the authority on this and no register is read for it.
    ///
    /// # ★ H3 — this still cannot cancel an SPI future mid-flow
    ///
    /// Nothing added here is an `await`. `Instant::now()` is a register read on the MCU and
    /// [`HopTrace::push`] is arithmetic on RAM; neither introduces a suspension point, so the set
    /// of places this future can be suspended is byte-for-byte what it was, and it is still awaited
    /// as a bare statement in both call sites — never as a `select` branch.
    async fn service_hop(&mut self, tx_active: bool) -> Result<(), esp_hal::spi::Error> {
        // ★ H1: THE STAMP. Nothing may be inserted above this line.
        let t = Instant::now();
        self.raw
            .write_reg(regs::REG_IRQ_FLAGS, regs::IRQ_FHSS_CHANGE_CHANNEL)
            .await?;
        let hc = self.raw.read_reg(regs::REG_HOP_CHANNEL).await?;
        // H2: recorded from the read the service path already performs — no second SPI transaction
        // is added for the trace — and recorded HERE, before the retune, so a `write_frf` that then
        // fails on the bus loses the retune but not the evidence that the boundary happened.
        //
        // The value stored is `RegHopChannel`'s own 6-bit FhssPresentChannel field, NOT
        // `field % n`. That matters for the experiment this instrument exists for: with a
        // ONE-ENTRY hop list — where the machinery runs but the carrier can never move — the
        // modulo'd list index is 0 forever and would hide the chip's hop counter entirely, while
        // the raw field still counts 0,1,2,… and wraps at 64. It is also literally what the H2
        // contract asks for ("what RegHopChannel reports on the SX1276"). The retune below keeps
        // using `% n`, which is the Semtech reference driver's pattern and unchanged.
        self.hop.trace.push(t, hc & regs::HOP_CHANNEL_MASK, tx_active);
        if hc & regs::HOP_CHANNEL_PLL_TIMEOUT != 0 {
            // The modem itself says the synthesiser did not lock in time for this hop. Counted
            // rather than inferred, and surfaced in the CMD_SET_DEBUG dump.
            self.hop.pll_timeouts = self.hop.pll_timeouts.wrapping_add(1);
        }
        // An empty list can only be reached by a stale latched flag (hopping was just turned off
        // while one was pending). Clearing it was the whole job; writing RegFrf from `frf[0] == 0`
        // would command a 0 Hz carrier. The event is still in the trace above — it did happen, and
        // silently dropping it would put a gap in a timeline whose gaps are the measurement.
        if self.hop.n == 0 {
            return Ok(());
        }
        let idx = (hc & regs::HOP_CHANNEL_MASK) as usize % self.hop.n;
        self.raw.write_frf(self.hop.frf[idx]).await?;
        self.hop.hops = self.hop.hops.wrapping_add(1);
        self.hop.last_idx = idx as u8;
        Ok(())
    }

    /// Turn hopping on or off in the chip. The list itself is host state and lives in [`Hop`]; this
    /// is only the three registers.
    ///
    /// `map_dio1` runs ONCE per enable rather than per arm: `lora-phy`'s three `RegDioMapping1`
    /// writes all preserve bits 5:4 (`& 0x3f` in the Transmit and CAD arms, `& 0x3f & 0xfc` in the
    /// Receive arm — pinned by a compile-time assertion in [`regs`]), so nothing it does can
    /// un-map DIO1.
    ///
    /// Disabling puts all three registers back: `RegHopPeriod <- 0` stops the modem hopping,
    /// `mask_hop_irq` re-masks the interrupt AND clears any flag still latched, and DIO1 goes back
    /// to its power-on `RxTimeout` mapping. That ordering matters — a latched flag left behind
    /// after the branch that services it has been removed from the `select` would hold DIO1 high
    /// forever.
    async fn set_hop_regs(&mut self, enable: bool, period: u8) -> Result<(), esp_hal::spi::Error> {
        if enable {
            self.raw.map_dio1(regs::DIO1_FHSS_CHANGE_CHANNEL).await?;
            self.raw.write_reg(regs::REG_HOP_PERIOD, period).await?;
            self.raw.unmask_hop_irq().await
        } else {
            self.raw.write_reg(regs::REG_HOP_PERIOD, 0).await?;
            self.raw.mask_hop_irq().await?;
            self.raw.map_dio1(regs::DIO1_RX_TIMEOUT).await.map(|_| ())
        }
    }

    /// A cancel-safe await: DIO0 going high. No SPI, no state. (C1 added a second one of
    /// exactly the same shape on DIO1 — see [`Radio::hop_wait`].)
    async fn wait_irq(&mut self) -> Result<(), RadioError> {
        self.chip.await_irq().await
    }

    /// Consume a pending RX interrupt. Awaited bare, after [`Radio::wait_irq`] has already resolved,
    /// so it never suspends waiting for hardware that has not happened yet.
    ///
    /// `Ok(None)` means the interrupt was a HeaderValid/preamble event (or nothing we act on) and the
    /// chip is still in RXCONTINUOUS — keep listening.
    async fn take_rx(
        &mut self,
        rx_pkt: &PacketParams,
        buf: &mut [u8],
    ) -> Result<Option<(u8, PacketStatus)>, RadioError> {
        match self.chip.process_irq_event(self.mode, None, true).await? {
            Some(IrqState::Done) => {
                let n = self.chip.get_rx_payload(rx_pkt, buf).await?;
                let status = self.chip.get_rx_packet_status().await?;
                Ok(Some((n, status)))
            }
            _ => Ok(None),
        }
    }

    /// **Program the chip for one transmission, but do not key up.** Everything `LoRa::prepare_for_tx`
    /// does, minus `do_tx()` — the packet params are built here with the real payload length
    /// (`PacketParams::set_payload_length` is crate-private, `create_packet_params` is not).
    ///
    /// Splitting the transmit here is what makes `CMD_TX_AT` honest (P1). Run whole from a deadline,
    /// this burst is ~30 register transactions plus the payload FIFO write — over 2 ms for a
    /// 247-byte frame, and payload-length dependent, so a schedule paid for with it would be a
    /// schedule with a 2 ms error that varies with the frame. Run *ahead* of the deadline, it leaves
    /// [`Radio::fire_tx`] one 2-byte SPI write, and the error collapses to [`SCHED_GRAN_NS`].
    ///
    /// On return the mode mirror says `Transmit` while the chip is physically in STANDBY with the
    /// frame in its FIFO and DIO0 mapped to TxDone. That divergence is deliberate and is exactly why
    /// [`main`] must not race `wait_irq` while a frame is staged.
    async fn stage_tx(
        &mut self,
        m: &ModulationParams,
        pwr: i32,
        preamble: u16,
        payload: &[u8],
        freq_hz: u32,
    ) -> Result<(), RadioError> {
        self.to_standby().await?;
        self.chip.set_modulation_params(m).await?;
        self.chip
            .set_tx_power_and_ramp_time(pwr, Some(m), true)
            .await?;
        let pkt = self.chip.create_packet_params(
            preamble,
            false,
            payload.len() as u8,
            true,
            false,
            m,
        )?;
        self.chip.set_packet_params(&pkt).await?;
        self.chip.set_channel(freq_hz).await?;
        self.chip.set_payload(payload).await?;
        self.mode = RadioMode::Transmit;
        self.chip.set_irq_params(Some(self.mode)).await?;
        // C1: same as `arm_rx` — the Transmit arm masks FhssChangedChannel too, and a transmission
        // is precisely where intra-packet hopping has to work. The trailing `RegIrqFlags <- 0x02`
        // inside `unmask_hop_irq` also guarantees DIO1 is LOW on entry to the staged state, which
        // is what lets the main loop drop the hop branch while a frame is staged.
        self.apply_hop().await?;
        Ok(())
        // NOTE: RegDetectOptimize/RegDetectionThreshold were just overwritten from the SF by
        // `set_modulation_params` and are deliberately NOT restored here — they steer the receive
        // and CAD detector only, and the `arm_rx` that follows every transmission re-applies them.
    }

    /// **Key up a staged frame and wait for TxDone.** Returns the instant the `RegOpMode <- TX`
    /// write completed, measured on the SAME counter `EVT_RX` stamps with and `CMD_READ_CLOCK`
    /// returns, so a scheduling error computed from it is in the host's own declared units.
    ///
    /// The TxDone wait is bounded by the frame's own computed airtime plus a wide margin: with only
    /// DIO0 wired, a missed interrupt would otherwise park the node forever and the host would see
    /// no reply at all. The timeout races `await_irq` only — never the SPI phase (C3).
    async fn fire_tx(&mut self, air_ms: u32) -> (Instant, Result<(), RadioError>) {
        let keyed_res = self.chip.do_tx().await;
        // Sampled immediately after the write returns, before anything else can run: this is the
        // observable that makes SCHED_GRAN_NS falsifiable instead of a claim.
        let keyed = Instant::now();
        if let Err(e) = keyed_res {
            let _ = self.chip.set_standby().await;
            self.mode = RadioMode::Standby;
            return (keyed, Err(e));
        }

        let deadline = Instant::now() + Duration::from_millis(air_ms as u64 * 2 + 500);
        loop {
            // ★ C1: while hopping is on, `FhssChangeChannel` fires DURING this transmission — that
            // IS intra-packet hopping — arriving on DIO1 while TxDone is still pending on DIO0.
            // Racing it here is safe for exactly the reason `await_irq` is: both are GPIO level
            // waits that touch no SPI and no driver state, so dropping either costs nothing. The
            // result is bound to a local FIRST, so every borrow the `select` held is released
            // before `service_hop` — a SPI-touching future — is awaited as a bare statement.
            let ev = if self.hop.enabled {
                match select3(
                    self.chip.await_irq(),
                    self.hop_irq.wait_for_high(),
                    Timer::at(deadline),
                )
                .await
                {
                    Either3::First(r) => FireWake::Irq(r),
                    Either3::Second(()) => FireWake::Hop,
                    Either3::Third(_) => FireWake::Timeout,
                }
            } else {
                match select(self.chip.await_irq(), Timer::at(deadline)).await {
                    Either::First(r) => FireWake::Irq(r),
                    Either::Second(_) => FireWake::Timeout,
                }
            };
            match ev {
                // Retune to the next hop and go straight back to waiting for TxDone. The chip stays
                // in TX throughout; only the carrier moves.
                FireWake::Hop => {
                    // H4: `true` — the chip is in TX with this frame in flight, so every hop
                    // serviced here is an INTRA-PACKET hop of ours and is flagged as such in the
                    // trace. This is the only call site that can say that.
                    if self.service_hop(true).await.is_err() {
                        // Cannot keep hopping if the bus will not carry the retune, and cannot keep
                        // racing a pin we can no longer clear. Give up hopping, not the frame.
                        self.hop.enabled = false;
                        let _ = self.raw.write_reg(regs::REG_HOP_PERIOD, 0).await;
                    }
                }
                FireWake::Irq(Err(e)) => {
                    let _ = self.chip.set_standby().await;
                    self.mode = RadioMode::Standby;
                    return (keyed, Err(e));
                }
                FireWake::Irq(Ok(())) => {
                    match self.chip.process_irq_event(self.mode, None, true).await {
                        Ok(Some(IrqState::Done | IrqState::PreambleReceived)) => {
                            self.mode = RadioMode::Standby;
                            return (keyed, Ok(()));
                        }
                        Ok(None) => continue,
                        Err(e) => {
                            let _ = self.chip.set_standby().await;
                            self.mode = RadioMode::Standby;
                            return (keyed, Err(e));
                        }
                    }
                }
                // TxDone never asserted within twice the computed airtime. Recover rather than hang.
                FireWake::Timeout => {
                    let _ = self.chip.set_standby().await;
                    let _ = self.chip.set_irq_params(None).await; // clears RegIrqFlags
                    self.mode = RadioMode::Standby;
                    return (keyed, Err(RadioError::TransmitTimeout));
                }
            }
        }
    }

    /// Transmit one frame now and wait for TxDone: stage, then fire. Every immediate transmit in the
    /// firmware and every scheduled one go through the SAME programming code, so the two can never
    /// put the chip in different states.
    async fn transmit(
        &mut self,
        m: &ModulationParams,
        pwr: i32,
        preamble: u16,
        payload: &[u8],
        air_ms: u32,
        freq_hz: u32,
    ) -> Result<(), RadioError> {
        self.stage_tx(m, pwr, preamble, payload, freq_hz).await?;
        self.fire_tx(air_ms).await.1
    }

    /// Give up a staged frame without keying up. **No SPI**: the chip really is in standby (that is
    /// what [`Radio::stage_tx`] left it in), so only the mode mirror has to be corrected. Being
    /// SPI-free is what makes it safe to call from anywhere in the command path.
    fn abandon_staged_tx(&mut self) {
        self.mode = RadioMode::Standby;
    }

    /// One channel-activity detection at the current modulation.
    ///
    /// **C2**: `prepare_for_cad`'s two jobs are what the old code skipped — it puts the driver in
    /// `RadioMode::ChannelActivityDetection` (without which `cad()` returns `InvalidRadioMode`
    /// immediately, having touched no hardware) *and* remaps DIO0 from RxDone to CadDone (without
    /// which the CAD-done interrupt can never reach the MCU). The old firmware did neither, so its
    /// CAD always failed instantly, the LBT loop took its "error => treat as clear" branch on the
    /// first attempt, and `CMD_CAD` always answered "clear". Both halves are done here.
    ///
    /// Still bounded by a timer, because a lost CadDone would hang the command loop; the bound is
    /// derived from the symbol time so it scales with SF/BW instead of the old fixed 60 ms (which is
    /// shorter than a single SF12/125 kHz symbol pair and would have timed out every time).
    async fn cad(
        &mut self,
        m: &ModulationParams,
        tsym_us: u64,
        freq_hz: u32,
    ) -> Result<bool, RadioError> {
        self.to_standby().await?;
        self.chip.set_modulation_params(m).await?;
        self.apply_detector().await?; // P2 — the detector IS the thing CAD uses; see `apply_detector`
        self.chip.set_channel(freq_hz).await?;
        self.mode = RadioMode::ChannelActivityDetection;
        self.chip.set_irq_params(Some(self.mode)).await?;
        self.chip.do_cad(m).await?;
        // P3: `do_cad` writes RegLna too, and does it immediately before entering CAD, so the gain
        // has to go back afterwards. That lands ~8 µs (one 2-byte SPI write at 2 MHz) into a
        // detection window that is two symbols long — 2 ms at SF7/125 kHz, more above — so it
        // governs essentially the whole window. Stated rather than glossed: the first ~0.4% of the
        // window runs at the driver's gain, not the host's.
        self.apply_lna().await?;

        // SX1276 CAD is ~2 symbols; allow 8 plus a fixed slack for SPI and scheduling.
        let bound = Duration::from_micros(tsym_us * 8 + 5_000);
        let out = match select(self.chip.await_irq(), Timer::after(bound)).await {
            Either::First(Ok(())) => {
                let mut detected = false;
                match self
                    .chip
                    .process_irq_event(self.mode, Some(&mut detected), true)
                    .await
                {
                    Ok(_) => Ok(detected),
                    Err(e) => Err(e),
                }
            }
            Either::First(Err(e)) => Err(e),
            Either::Second(_) => Err(RadioError::ReceiveTimeout),
        };
        // The SX1276 drops to standby by itself when CAD finishes; make our mirror agree, and clear
        // any latched flag if we bailed out on the timer.
        let _ = self.chip.set_standby().await;
        if out.is_err() {
            let _ = self.chip.set_irq_params(None).await;
        }
        self.mode = RadioMode::Standby;
        out
    }
}


// =================================================================================================
// Firmware state
// =================================================================================================

/// Radio knobs the host can change at runtime. Mirrors what was actually programmed, so `EVT_INFO`
/// reports the applied value rather than the requested one — which is why it is `Copy`: a knob the
/// chip rejects is rolled back before the reply is built (see [`rebuild`]).
#[derive(Clone, Copy)]
struct Params {
    freq_hz: u32,
    sf: SpreadingFactor,
    bw: Bandwidth,
    cr: CodingRate,
    pwr: i32,
}

/// Carrier-sense / LBT state, all host-tunable so calibration never needs a reflash.
struct Csma {
    preamble: u16,
    /// Contention window (ms), max backoff exponent, max attempts before DEFERRED.
    lbt_cw: u32,
    lbt_max_backoff: u8,
    lbt_max_attempts: u8,
    /// Backoff PRNG (xorshift32) — C6. Seeded in [`Csma::seed`].
    rng: u32,
    /// Free-running, wrapping count of channel-busy observations. Never cleared by CMD_RESET_STATS,
    /// because `EVT_SENSE.activity` is defined as a free-running counter the host differences;
    /// `cad_busy_base` re-baselines the *view* EVT_INFO/EVT_STATS report instead.
    cad_busy: u16,
    cad_busy_base: u16,
    defer: u16,
    /// Energy-detect threshold in dBm OR'd into the busy sense; `i16::MIN` disables it.
    rssi_thresh: i16,
    /// CAD samples per sense (OR'd). More cuts false negatives at the cost of airtime.
    cad_repeat: u8,
}

impl Csma {
    fn new() -> Self {
        Self {
            preamble: 8,
            // The initial backoff window must be about a CAD slot (SF10 CAD is ~33 ms) for two nodes
            // to separate; runtime-tunable via CMD_SET_LBT_CFG.
            lbt_cw: 20,
            lbt_max_backoff: 4,
            lbt_max_attempts: 6,
            rng: 0x1234_5678,
            cad_busy: 0,
            cad_busy_base: 0,
            defer: 0,
            rssi_thresh: i16::MIN, // energy-detect off by default (CAD only)
            cad_repeat: 1,
        }
    }

    /// C6 — seed the backoff PRNG so two Heltecs cannot back off identically.
    ///
    /// The old code had no PRNG at all: `8 + attempt*6` ms, so every node in the cell picked the
    /// same instants and LBT could not separate them. Two independent sources are mixed:
    ///
    /// * the **eFuse base MAC**, which is unique per chip and therefore guarantees two boards differ
    ///   even if the RNG returns the same value on both;
    /// * the **ESP32 RNG** plus the boot-time tick, which decorrelate successive boots of one board.
    ///
    /// (The ESP32's RNG is only a certified TRNG with the RF subsystem running, which this firmware
    /// does not start. That is why the MAC is mixed in rather than trusted alone — the property LBT
    /// actually needs is *inter-node* difference, and the MAC supplies it unconditionally.)
    fn seed(&mut self) {
        let mac = esp_hal::efuse::base_mac_address();
        let b = mac.as_bytes();
        let mut s = u32::from_be_bytes([b[2], b[3], b[4], b[5]]);
        s ^= esp_hal::rng::Rng::new().random();
        s ^= Instant::now().as_ticks() as u32;
        self.rng = if s == 0 { 0xA5A5_5A5A } else { s };
    }

    fn next_rand(&mut self) -> u32 {
        let mut x = self.rng;
        x ^= x << 13;
        x ^= x >> 17;
        x ^= x << 5;
        self.rng = x;
        x
    }

    /// The resettable view of `cad_busy` that EVT_INFO/EVT_STATS report.
    fn cad_busy_view(&self) -> u16 {
        self.cad_busy.wrapping_sub(self.cad_busy_base)
    }
}

/// **P1 — the one-deep scheduled-TX slot**, and the two-phase state machine behind `CMD_TX_AT`.
///
/// ```text
///  Idle  --CMD_TX_AT-->  Pending{fire_at, stage_at}  --at stage_at-->  Staged{fire_at}
///                             ^                                            |
///                             |  a host command arrives (chip reprogrammed) |
///                             +--------------------------------------------+
///                                                                          |  at fire_at
///                                                                    do_tx() --> Idle
/// ```
///
/// **Pending** is the frame waiting while the node keeps *listening*: the chip is in RXCONTINUOUS
/// and nothing about it has changed. **Staged** is the frame programmed into the chip, which does
/// cost reception — so the window in which this node is deaf is not "until the deadline" but only
/// the programming budget derived in [`SchedTx::pending`], 1.5-2.7 ms. A 30-second schedule costs
/// 2.7 ms of deafness, not 30 seconds.
///
/// One deep on purpose: a second `CMD_TX_AT` while a frame is armed is REFUSED (`EVT_TXDONE [0,0]`)
/// rather than silently replacing the first, because dropping an accepted frame with no word to the
/// host is the failure mode the whole 7E-A5 v2 rule set exists to prevent.
#[derive(Clone, Copy)]
enum SchedTx {
    Idle,
    /// Accepted; the chip is still in RX and the frame waits in [`Sched::buf`].
    Pending {
        fire_at: Instant,
        stage_at: Instant,
        len: usize,
        air_ms: u32,
    },
    /// Programmed and holding in standby with the payload in the FIFO; only `do_tx()` remains.
    Staged {
        fire_at: Instant,
        len: usize,
        air_ms: u32,
    },
}

impl SchedTx {
    /// Arm a frame, deriving when the programming burst has to start so that only the key-up write
    /// is left at `fire_at`. The budget covers the `EVT_TX_STARTED` push as well as the register
    /// burst: [`STAGE_FIXED_US`] + [`STAGE_PER_BYTE_US`] per byte, 1.5-2.7 ms. If
    /// the deadline is already inside it, `stage_at` saturates to boot time and staging happens on
    /// the next pass through the loop (i.e. immediately, and late — which the firmware then reports).
    fn pending(fire_at: Instant, len: usize, air_ms: u32) -> Self {
        let budget = Duration::from_micros(STAGE_FIXED_US + STAGE_PER_BYTE_US * len as u64);
        let stage_at = Instant::from_ticks(fire_at.as_ticks().saturating_sub(budget.as_ticks()));
        SchedTx::Pending { fire_at, stage_at, len, air_ms }
    }

    /// The next instant this slot needs the main loop to wake, if any.
    fn wake_at(&self) -> Option<Instant> {
        match self {
            SchedTx::Idle => None,
            SchedTx::Pending { stage_at, .. } => Some(*stage_at),
            SchedTx::Staged { fire_at, .. } => Some(*fire_at),
        }
    }

    /// True while the chip is programmed for the staged frame and must not be touched: no RX arm,
    /// no beacon, and no `wait_irq` branch in the main `select`.
    fn owns_radio(&self) -> bool {
        matches!(self, SchedTx::Staged { .. })
    }

    /// True from `CMD_TX_AT` until the frame goes out or fails.
    fn is_armed(&self) -> bool {
        !matches!(self, SchedTx::Idle)
    }
}

/// The scheduled-TX slot plus the frame it holds. Its own struct so the 255-byte buffer lives beside
/// the state that says whether it is valid.
struct Sched {
    state: SchedTx,
    buf: [u8; 255],
}

/// What woke the main loop. Collapses the two `select` shapes (three-way normally, two-way while a
/// frame is staged and the DIO0 branch must not exist) into one thing to dispatch on.
enum Wake {
    Cmd(Frame),
    /// `true` if `await_irq` resolved cleanly.
    Irq(bool),
    /// C1 — `FhssChangeChannel` on DIO1: the modem wants the next hop's carrier.
    Hop,
    Timer,
}

struct State {
    p: Params,
    csma: Csma,
    plane: ndn::DataPlane,
    debug: bool,
    beacon_on: bool,
    beacon_period: Duration,
    beacon_seq: u32,
    /// C4: a real count of radio-layer failures, reported as `EVT_INFO.errors`. NOT the chip's own
    /// device-error word — the SX1276 has no equivalent of the SX1262's `GetDeviceErrors` (0x17), so
    /// there is nothing on this chip to read. Documented in `send_info`.
    radio_errors: u16,
    /// Frames received that were larger than `RX_MAX` and so could not be framed into an EVT_RX.
    rx_trunc: u16,
    /// P1 — the one-deep `CMD_TX_AT` slot.
    sched: Sched,
    /// P1 — scheduled frames whose key-up landed further than [`SCHED_GRAN_NS`] from the instant the
    /// host asked for, i.e. the count of times this node did not meet the granularity it advertises.
    /// Surfaced in the `CMD_SET_DEBUG` chip dump; every occurrence also emits an `EVT_LOG` carrying
    /// the signed error, whether or not debug is on.
    sched_late: u16,
}

// =================================================================================================
// Entry point
// =================================================================================================

static SPI_BUS: StaticCell<RefCell<Bus>> = StaticCell::new();

#[esp_rtos::main]
async fn main(spawner: Spawner) {
    let peri = esp_hal::init(esp_hal::Config::default().with_cpu_clock(CpuClock::max()));
    esp_alloc::heap_allocator!(size: 32 * 1024);
    let timg0 = TimerGroup::new(peri.TIMG0);
    let sw = esp_hal::interrupt::software::SoftwareInterruptControl::new(peri.SW_INTERRUPT);
    esp_rtos::start(timg0.timer0, sw.software_interrupt0);

    // UART0 (CP2102) raw for the binary protocol.
    let uart = Uart::new(peri.UART0, UartConfig::default().with_baudrate(115200))
        .unwrap()
        .with_rx(peri.GPIO3)
        .with_tx(peri.GPIO1)
        .into_async();
    let (uart_rx, mut uart_tx) = uart.split();
    spawner.spawn(uart_reader(uart_rx).unwrap());

    // SX1276 over SPI2 (Heltec V2: SCK 5, MISO 19, MOSI 27, NSS 18, RST 14, DIO0 26).
    let spi = Spi::new(
        peri.SPI2,
        SpiConfig::default()
            .with_frequency(Rate::from_mhz(2))
            .with_mode(Mode::_0),
    )
    .unwrap()
    .with_sck(peri.GPIO5)
    .with_miso(peri.GPIO19)
    .with_mosi(peri.GPIO27)
    .into_async();
    let cs = Output::new(peri.GPIO18, Level::High, OutputConfig::default());
    let bus: &'static RefCell<Bus> = SPI_BUS.init(RefCell::new(Bus { spi, cs }));
    let dev = SharedSpi(bus);
    let raw = SharedSpi(bus);

    let reset = Output::new(peri.GPIO14, Level::High, OutputConfig::default());
    let dio0 = Input::new(peri.GPIO26, InputConfig::default().with_pull(Pull::None));
    // ★ C1 — DIO1 (GPIO35) is taken by THIS firmware, not by lora-phy, and carries
    // `FhssChangeChannel`. The pin number is this repo's attested Heltec V2 pinout
    // (`firmware/heltec-lora-node/heltec-lora-node.ino:27`), not a datasheet guess; see
    // `Radio::hop_irq`. GPIO34-39 are input-only and have no internal pulls, hence `Pull::None` —
    // which is also correct electrically, since the SX1276 drives DIO1 push-pull.
    let dio1 = Input::new(peri.GPIO35, InputConfig::default().with_pull(Pull::None));
    let iv = GenericSx127xInterfaceVariant::new(reset, dio0, None, None).unwrap();
    // tx_boost: the Heltec V2 routes the antenna to PA_BOOST, so this is a board fact, not a
    // choice — and it is what makes PWR_MIN_DBM/PWR_MAX_DBM = [2, 20] the honest range in EVT_CAP.
    // rx_boost turns on the SX1276's LNA boost (~+3 dB sensitivity), matching the Waveshare node's
    // default so a link budget measured on one node transfers to the other.
    let config = Sx127xConfig { chip: Sx1276, tcxo_used: false, tx_boost: true, rx_boost: true };
    let mut radio = Radio {
        chip: Sx127x::new(dev, iv, config),
        raw,
        mode: RadioMode::Sleep,
        det: None, // P2 — until CMD_SET_CAD_CFG, the detector is whatever lora-phy derives from SF
        lna: None, // P3 — until CMD_SET_RX_GAIN, the LNA is whatever `rx_boost` gives
        hop_irq: dio1,
        hop: Hop::new(), // C1 — until CMD_SET_HOP, hopping is off and costs nothing at all
    };
    radio.init().await.expect("sx1276 init");

    let mut st = State {
        p: Params {
            freq_hz: 915_000_000,
            sf: SpreadingFactor::_9,
            bw: Bandwidth::_125KHz,
            cr: CodingRate::_4_5,
            pwr: 17,
        },
        csma: Csma::new(),
        plane: ndn::DataPlane::new(),
        debug: false,
        beacon_on: false,
        beacon_period: Duration::from_secs(10),
        beacon_seq: 0,
        radio_errors: 0,
        rx_trunc: 0,
        sched: Sched { state: SchedTx::Idle, buf: [0; 255] },
        sched_late: 0,
    };
    st.csma.seed();

    let (mut mdltn, mut rx_pkt) = build_params(&radio, &st).expect("modulation params");
    let mut rxbuf = [0u8; RX_BUF];
    let mut rx_armed = false;
    let mut next_beacon = Instant::now() + st.beacon_period;

    // Announce readiness: the build identity first (C7), then the structured info.
    send_build_id(&mut uart_tx).await;
    send_info(&mut uart_tx, &mut radio, &st).await;

    loop {
        let mut arm_failed = false;
        // P1: a STAGED frame owns the chip — it is idle in standby with the payload already in the
        // FIFO and DIO0 mapped to TxDone. Arming RX here would overwrite exactly that.
        if !rx_armed && !st.sched.state.owns_radio() {
            match radio.arm_rx(&mdltn, &rx_pkt, st.p.freq_hz).await {
                Ok(()) => rx_armed = true,
                Err(_) => {
                    st.radio_errors = st.radio_errors.saturating_add(1);
                    arm_failed = true;
                }
            }
        }

        // When the timer branch should fire: the EARLIEST of the deadlines that exist. A failed arm
        // needs a short retry tick — with the chip not in RX, DIO0 can never assert, so without one
        // the node would sit deaf until a host command happened to arrive. The tick goes through the
        // SAME select as everything else, so a wedged radio still leaves the host link responsive.
        // The branch itself re-checks which deadline is actually due.
        let mut wake_at = Instant::now() + Duration::from_secs(3600);
        if arm_failed {
            wake_at = wake_at.min(Instant::now() + Duration::from_millis(250));
        }
        if st.beacon_on {
            wake_at = wake_at.min(next_beacon);
        }
        if let Some(t) = st.sched.state.wake_at() {
            wake_at = wake_at.min(t);
        }

        // The ONLY cancellable await (C3): a queued host command, the DIO0 level, or a deadline.
        // None of them touches SPI or driver state, so dropping any of them is free.
        //
        // ★ P1: while a frame is staged the DIO0 branch is DROPPED, not guarded. The chip is idle in
        // standby with DIO0 mapped to TxDone, so `wait_irq` could only resolve on a stale level —
        // and `take_rx` would then run `process_irq_event` with the mode mirror reading `Transmit`.
        // Deleting the branch deletes the case; there is nothing left to get the guard wrong about.
        //
        // ★ C1: the hop branch is added only while hopping is ENABLED, and never while a frame is
        // staged. Staged means the chip is idle in standby with no packet in flight, so no hop can
        // occur — and `stage_tx`'s `apply_hop` cleared any latched flag on the way in, so DIO1 is
        // low and cannot hold a branch that is no longer there. With hopping off, the shapes below
        // are byte-for-byte the ones this loop has always had.
        let wake = if st.sched.state.owns_radio() {
            match select(CMDQ.receive(), Timer::at(wake_at)).await {
                Either::First(f) => Wake::Cmd(f),
                Either::Second(_) => Wake::Timer,
            }
        } else if radio.hop.enabled {
            // `radio.chip.await_irq()` and `radio.hop_irq.wait_for_high()` are written as DISJOINT
            // FIELD borrows rather than through `Radio::wait_irq`/`Radio::hop_wait`, because two
            // `&mut radio` method calls in one expression would not borrow-check. Both are still
            // nothing but GPIO level waits.
            match select4(
                CMDQ.receive(),
                radio.chip.await_irq(),
                radio.hop_irq.wait_for_high(),
                Timer::at(wake_at),
            )
            .await
            {
                Either4::First(f) => Wake::Cmd(f),
                Either4::Second(res) => Wake::Irq(res.is_ok()),
                Either4::Third(()) => Wake::Hop,
                Either4::Fourth(_) => Wake::Timer,
            }
        } else {
            match select3(CMDQ.receive(), radio.wait_irq(), Timer::at(wake_at)).await {
                Either3::First(f) => Wake::Cmd(f),
                Either3::Second(res) => Wake::Irq(res.is_ok()),
                Either3::Third(_) => Wake::Timer,
            }
        };

        match wake {
            // ---- host command ----
            Wake::Cmd(f) => {
                // A staged frame cannot survive a host command: most opcodes reprogram the chip, and
                // tracking which ones is precisely the bookkeeping `rx_armed = false` below already
                // refuses to do. Demote to Pending — the frame keeps its deadline and is programmed
                // again from scratch. `abandon_staged_tx` touches no SPI (the chip really is in
                // standby), so this cannot leave a half-done transaction behind.
                if let SchedTx::Staged { fire_at, len, air_ms } = st.sched.state {
                    radio.abandon_staged_tx();
                    st.sched.state = SchedTx::pending(fire_at, len, air_ms);
                }
                handle_cmd(&f, &mut radio, &mut st, &mut uart_tx, &mut mdltn, &mut rx_pkt).await;
                // Almost every command puts the chip in standby (or reprograms it); re-arm rather
                // than track which ones did. The cost is ~10 register writes.
                rx_armed = false;
            }

            // ---- radio interrupt ----
            Wake::Irq(ok) => {
                if !ok {
                    st.radio_errors = st.radio_errors.saturating_add(1);
                    rx_armed = false;
                    continue;
                }
                match radio.take_rx(&rx_pkt, &mut rxbuf).await {
                    Ok(Some((n, status))) => {
                        let n = n as usize;
                        let disturbed =
                            on_rx(&mut radio, &mut st, &mut uart_tx, &mdltn, &rxbuf, n, &status)
                                .await;
                        if disturbed {
                            rx_armed = false;
                        }
                    }
                    // HeaderValid / nothing to do: still in RXCONTINUOUS, keep listening.
                    Ok(None) => {}
                    Err(_) => {
                        st.radio_errors = st.radio_errors.saturating_add(1);
                        rx_armed = false;
                    }
                }
            }

            // ---- ★ C1: the hop interrupt ----
            //
            // Awaited as a BARE statement, after the `select` has already resolved — never as a
            // select branch (C3). `rx_armed` is deliberately NOT cleared: rewriting `RegFrf` under
            // a packet in flight is the entire point, the modem stays in RXCONTINUOUS, and
            // re-arming would abort the very reception the hop exists to follow.
            Wake::Hop => {
                // H4: `false` — the main loop only runs while no transmission is in flight
                // (`fire_tx` does not return until TxDone, a timeout or an error), so a hop
                // serviced here was taken while LISTENING: idle RX, or a reception in progress.
                if radio.service_hop(false).await.is_err() {
                    st.radio_errors = st.radio_errors.saturating_add(1);
                    // A hop we could not service leaves DIO1 latched high, and the branch above
                    // would then spin the loop. Turning hopping off removes the branch, which is
                    // the one recovery that cannot spin whatever the bus is doing.
                    radio.hop.enabled = false;
                    let _ = radio.raw.write_reg(regs::REG_HOP_PERIOD, 0).await;
                    send_frame(&mut uart_tx, EVT_LOG, b"hop service failed; hopping disabled").await;
                }
            }

            // ---- a deadline: scheduled TX, the beacon, or the RX re-arm retry tick ----
            Wake::Timer => {
                // P1 first: it is the only one of the three that has a real deadline. Both arms
                // below await a SPI-touching future as a BARE statement, after the select has
                // already resolved — never as a select branch (C3).
                let sched_now = st.sched.state;
                match sched_now {
                    SchedTx::Pending { fire_at, stage_at, len, air_ms }
                        if Instant::now() >= stage_at =>
                    {
                        // ★ EVT_TX_STARTED goes out HERE — at the staging point, 1-2.7 ms before
                        // key-up — and neither of the two obvious alternatives is legal:
                        //
                        //  * NOT at the deadline. `write_all` only returns once the UART's 128-byte
                        //    TX FIFO has taken the bytes; if a preceding EVT_RX still occupies it,
                        //    that wait is up to 128 B / 115200 8N1 = 11 ms. Unbounded against a
                        //    99 µs budget, so it cannot sit between the timer and `do_tx`.
                        //  * NOT at acceptance. The host re-bases its reply deadline the moment it
                        //    sees this event: `lora_serial.rs::exec_on` sets it to
                        //    `now + airtime + AIRTIME_SLACK`, and AIRTIME_SLACK is 2 s. Emitting at
                        //    acceptance would make every schedule longer than ~2 s time out at the
                        //    host — the very delays SCHED_MAX_DELAY_US (60 s) exists to allow.
                        //
                        // Emitted from here it is both out of the timing path and close enough to
                        // key-up that the host's re-based deadline is correct. The staging budget in
                        // `SchedTx::pending` includes this write; see STAGE_FIXED_US.
                        send_frame(
                            &mut uart_tx,
                            EVT_TX_STARTED,
                            &(air_ms.min(u16::MAX as u32) as u16).to_be_bytes(),
                        )
                        .await;
                        let r = radio
                            .stage_tx(
                                &mdltn,
                                st.p.pwr,
                                st.csma.preamble,
                                &st.sched.buf[..len],
                                st.p.freq_hz,
                            )
                            .await;
                        rx_armed = false; // the chip left RXCONTINUOUS either way
                        match r {
                            Ok(()) => st.sched.state = SchedTx::Staged { fire_at, len, air_ms },
                            Err(_) => {
                                st.radio_errors = st.radio_errors.saturating_add(1);
                                st.sched.state = SchedTx::Idle;
                                // Never silence: the host is waiting on an EVT_TXDONE for this frame.
                                send_frame(&mut uart_tx, EVT_TXDONE, &[0, 0]).await;
                            }
                        }
                    }
                    SchedTx::Staged { fire_at, air_ms, .. } if Instant::now() >= fire_at => {
                        let (keyed, res) = radio.fire_tx(air_ms).await;
                        st.sched.state = SchedTx::Idle;
                        rx_armed = false;
                        if res.is_err() {
                            st.radio_errors = st.radio_errors.saturating_add(1);
                        }
                        report_sched_error(&mut uart_tx, &mut st, fire_at, keyed).await;
                        send_frame(&mut uart_tx, EVT_TXDONE, &[res.is_ok() as u8, 0]).await;
                    }
                    _ => {}
                }

                if st.beacon_on && Instant::now() >= next_beacon {
                    next_beacon = Instant::now() + st.beacon_period;
                    // A beacon that falls inside an armed scheduled TX is SKIPPED, not queued: the
                    // scheduled frame has a deadline and the beacon does not, and transmitting here
                    // would either overrun `stage_at` or overwrite a staged FIFO. `next_beacon` is
                    // advanced first so a suppressed beacon cannot spin the loop on a past deadline.
                    if !st.sched.state.is_armed() {
                        let mut msg = BufWriter::new();
                        let _ = write!(msg, "LORA-BEACON seq={}", st.beacon_seq);
                        st.beacon_seq = st.beacon_seq.wrapping_add(1);
                        let air = airtime_ms(
                            sf_num(st.p.sf),
                            bw_hz(st.p.bw),
                            cr_num(st.p.cr),
                            msg.as_slice().len(),
                            st.csma.preamble,
                        );
                        let ok = radio
                            .transmit(
                                &mdltn,
                                st.p.pwr,
                                st.csma.preamble,
                                msg.as_slice(),
                                air,
                                st.p.freq_hz,
                            )
                            .await
                            .is_ok();
                        if !ok {
                            st.radio_errors = st.radio_errors.saturating_add(1);
                        }
                        send_frame(&mut uart_tx, EVT_TXDONE, &[ok as u8, 0]).await;
                        rx_armed = false;
                    }
                }
            }
        }
    }
}

/// **P1 — measure the schedule against the clock the host was given.**
///
/// `keyed` is sampled inside [`Radio::fire_tx`] the instant the `RegOpMode <- TX` write returns, on
/// the same `embassy-time` counter that stamps `EVT_RX` and that `CMD_READ_CLOCK` reports — so the
/// error below is in the units `EVT_CAP.stamp_hz` declares, and the host can compare it directly
/// against the `sched_gran_ns` the same `EVT_CAP` advertises.
///
/// An error larger than the advertised granularity is the node failing its own published spec, so it
/// is reported UNCONDITIONALLY (not only under `CMD_SET_DEBUG`) and counted. That is the whole point
/// of declaring a granularity: it has to be falsifiable from the wire.
async fn report_sched_error(
    tx: &mut UartTx<'static, Async>,
    st: &mut State,
    fire_at: Instant,
    keyed: Instant,
) {
    // Clamped before the µs conversion so the multiply cannot overflow on a wild outlier.
    let err_ticks = (keyed.as_ticks() as i64 - fire_at.as_ticks() as i64)
        .clamp(-1_000_000_000, 1_000_000_000);
    let err_us = err_ticks * 1_000_000 / STAMP_HZ as i64;
    let gran_us = (SCHED_GRAN_NS / 1_000) as i64;
    let late = err_us > gran_us || err_us < -gran_us;
    if late {
        st.sched_late = st.sched_late.saturating_add(1);
    }
    if late || st.debug {
        let mut lg = BufWriter::new();
        let _ = write!(lg, "sched err={err_us}us gran={gran_us}us late={}", st.sched_late);
        send_frame(tx, EVT_LOG, lg.as_slice()).await;
    }
}

/// Rebuild the modulation + RX packet parameters from the current knobs.
fn build_params(radio: &Radio, st: &State) -> Option<(ModulationParams, PacketParams)> {
    let m = radio
        .chip
        .create_modulation_params(st.p.sf, st.p.bw, st.p.cr, st.p.freq_hz)
        .ok()?;
    let rx = radio
        .chip
        .create_packet_params(st.csma.preamble, false, RX_BUF as u8, true, false, &m)
        .ok()?;
    Some((m, rx))
}

// =================================================================================================
// RX path
// =================================================================================================

/// Classify a received frame by NAME and act. Returns true if the radio was taken out of RX (a
/// Content-Store serve or a relay transmits, so the caller must re-arm).
async fn on_rx(
    radio: &mut Radio,
    st: &mut State,
    tx: &mut UartTx<'static, Async>,
    m: &ModulationParams,
    rxbuf: &[u8; RX_BUF],
    n: usize,
    status: &PacketStatus,
) -> bool {
    // The serial framing cannot carry more than RX_MAX bytes of LoRa payload (8-byte EVT_RX header
    // + one-byte length field). Count it rather than truncating: a truncated frame would reach the
    // host looking like a real, complete one.
    if n > RX_MAX {
        st.rx_trunc = st.rx_trunc.saturating_add(1);
        return false;
    }
    let now_ms = Instant::now().as_millis() as u32;
    let mut serve = [0u8; ndn::CS_MAX_LEN];
    let mut serve_len = 0usize;
    let (deliver, relay) = match st.plane.on_rx(&rxbuf[..n], now_ms) {
        ndn::RxAction::Drop => (false, false),
        ndn::RxAction::Serve(data) => {
            let k = data.len().min(serve.len());
            serve[..k].copy_from_slice(&data[..k]);
            serve_len = k;
            (false, false)
        }
        ndn::RxAction::Deliver => (true, false),
        ndn::RxAction::RelayAndDeliver => (true, true),
    };

    if st.debug {
        let mut lg = BufWriter::new();
        let _ = write!(lg, "rx n={n} serve={} relay={relay} deliver={deliver}", serve_len > 0);
        send_frame(tx, EVT_LOG, lg.as_slice()).await;
    }

    let mut disturbed = false;
    // Content-Store hit: serve the cached Data ourselves (with LBT); the host never wakes.
    if serve_len > 0 {
        lbt_tx(radio, st, m, &serve[..serve_len]).await;
        disturbed = true;
    }
    // Relay-set match: re-broadcast for cooperative forwarding, then also deliver.
    if relay {
        lbt_tx(radio, st, m, &rxbuf[..n]).await;
        disturbed = true;
    }
    if deliver {
        let ts = Instant::now().as_ticks() as u32;
        let mut ev = [0u8; 8 + RX_MAX];
        ev[0..2].copy_from_slice(&status.rssi.to_be_bytes());
        ev[2..4].copy_from_slice(&status.snr.to_be_bytes());
        ev[4..8].copy_from_slice(&ts.to_be_bytes());
        ev[8..8 + n].copy_from_slice(&rxbuf[..n]);
        send_frame(tx, EVT_RX, &ev[..8 + n]).await;
    }
    disturbed
}

// =================================================================================================
// Listen-before-talk
// =================================================================================================

/// Read the instantaneous channel RSSI in dBm.
///
/// `RegRssiValue` only means anything while the modem is in an RX mode, so the chip is armed first
/// and given a moment to settle. This is the one measurement the old firmware faked: `CMD_GET_RSSI`
/// answered a hardcoded `[0, 0]`, which reads as a 0 dBm carrier — the strongest signal the radio
/// could ever see.
async fn sense_rssi_dbm(
    radio: &mut Radio,
    m: &ModulationParams,
    rx_pkt: &PacketParams,
    freq_hz: u32,
) -> Option<i16> {
    if radio.mode != RadioMode::Receive(RxMode::Continuous) {
        radio.arm_rx(m, rx_pkt, freq_hz).await.ok()?;
        // The SX1276's RSSI averager needs a few symbols of RX before it is meaningful.
        Timer::after(Duration::from_millis(2)).await;
    }
    radio.raw.rssi_inst_dbm(freq_hz).await.ok()
}

/// Atomic listen-before-talk: a random backoff BEFORE each sense (CSMA/CA, so no node
/// deterministically captures the channel), transmit on a clear sense, give up after
/// `lbt_max_attempts`. Shared by `CMD_TX_LBT` and the firmware-served Content-Store / relay paths.
/// Returns `(sent, attempts)`.
async fn lbt_tx(
    radio: &mut Radio,
    st: &mut State,
    m: &ModulationParams,
    payload: &[u8],
) -> (bool, u8) {
    let mut attempt = 0u8;
    let mut sent = false;
    while attempt < st.csma.lbt_max_attempts.max(1) {
        // C6: a real random backoff. The old code used `8 + attempt*6` ms with no PRNG at all.
        // The shift is hard-capped at 10 regardless of what the host sets: `lbt_max_backoff` is a
        // raw byte off the wire, and `u32 << 32` is an arithmetic overflow, not a wide window. 10
        // already gives a 1024x window (20 s at the default cw), far past anything useful.
        let shift = (attempt as u32).min(st.csma.lbt_max_backoff as u32).min(10);
        let window = (st.csma.lbt_cw << shift).max(1);
        let wait = st.csma.next_rand() % window;
        Timer::after(Duration::from_millis(wait as u64)).await;

        // Sense = CAD (N repeats, OR'd) OR an RSSI energy detect (catches non-LoRa interference).
        let mut busy = false;
        let tsym = tsym_us(sf_num(st.p.sf), bw_hz(st.p.bw));
        for _ in 0..st.csma.cad_repeat.max(1) {
            match radio.cad(m, tsym, st.p.freq_hz).await {
                Ok(true) => {
                    busy = true;
                    break;
                }
                Ok(false) => {}
                Err(_) => {
                    // A CAD that errors or times out tells us nothing about the channel. Count it
                    // as a radio error and treat the sense as inconclusive-clear so LBT degrades to
                    // a plain transmit instead of wedging the node.
                    st.radio_errors = st.radio_errors.saturating_add(1);
                    break;
                }
            }
        }
        if !busy && st.csma.rssi_thresh > i16::MIN {
            let pkt = radio
                .chip
                .create_packet_params(st.csma.preamble, false, RX_BUF as u8, true, false, m)
                .ok();
            if let Some(pkt) = pkt {
                if let Some(r) = sense_rssi_dbm(radio, m, &pkt, st.p.freq_hz).await {
                    busy = r >= st.csma.rssi_thresh;
                }
            }
        }

        if !busy {
            let air = airtime_ms(
                sf_num(st.p.sf),
                bw_hz(st.p.bw),
                cr_num(st.p.cr),
                payload.len(),
                st.csma.preamble,
            );
            sent = radio
                .transmit(m, st.p.pwr, st.csma.preamble, payload, air, st.p.freq_hz)
                .await
                .is_ok();
            if !sent {
                st.radio_errors = st.radio_errors.saturating_add(1);
            }
            break;
        }
        st.csma.cad_busy = st.csma.cad_busy.wrapping_add(1);
        attempt += 1;
    }
    if !sent {
        st.csma.defer = st.csma.defer.wrapping_add(1);
    }
    (sent, attempt)
}

// =================================================================================================
// Events
// =================================================================================================

/// C7: the build this image was made from, so an on-air result is attributable.
async fn send_build_id(tx: &mut UartTx<'static, Async>) {
    let mut log = BufWriter::new();
    let _ = write!(log, "heltec-lora-rs build={BUILD_ID} proto=3 stamp_hz={STAMP_HZ}");
    send_frame(tx, EVT_LOG, log.as_slice()).await;
}

/// `EVT_INFO` — 19 bytes, every one of them real (C4).
///
/// The old version wrote a zeroed array with the sync word hardcoded to 0x12 and
/// status/errors/lost/cad_busy/defer permanently 0, so the host was reading a constant and calling
/// it a measurement.
///
/// * `status` = the SX1276's **`RegOpMode` (0x01)**, read live. This chip has no analogue of the
///   SX1262's `GetStatus()` byte; `RegOpMode` is the closest real thing (bit 7 LongRangeMode, bits
///   2:0 mode: 0 SLEEP, 1 STDBY, 3 TX, 5 RXCONTINUOUS, 7 CAD).
/// * `sync` = **`RegSyncWord` (0x39)** read back, in the low byte. The SX127x sync word is one byte
///   where the SX1262's is two, so the high byte is always 0 — that is the chip, not a placeholder.
/// * `errors` = this firmware's own count of radio-layer failures. **It is not the chip's device
///   errors**: the SX1276 has no equivalent of the SX1262's `GetDeviceErrors` (0x17), so there is no
///   such register to read on this radio. A firmware error count is the honest substitute and is
///   labelled as such here and in the README.
/// * `lost` / `cad_busy` / `defer` are the real counters maintained by [`uart_reader`] and
///   [`lbt_tx`].
async fn send_info(tx: &mut UartTx<'static, Async>, radio: &mut Radio, st: &State) {
    let status = radio.raw.read_reg(regs::REG_OP_MODE).await.unwrap_or(0);
    let sync = radio.raw.read_reg(regs::REG_SYNC_WORD).await.unwrap_or(0);
    let lost = SERIAL_LOST.load(Ordering::Relaxed).min(u16::MAX as u32) as u16;
    let f = st.p.freq_hz.to_be_bytes();
    let info = [
        status,
        0, // sync high byte: the SX127x sync word is 8 bits wide, so this is structurally 0
        sync,
        (st.radio_errors >> 8) as u8,
        st.radio_errors as u8,
        f[0],
        f[1],
        f[2],
        f[3],
        sf_num(st.p.sf),
        bw_code_of_hz(bw_hz(st.p.bw)),
        cr_num(st.p.cr),
        st.p.pwr as i8 as u8,
        (lost >> 8) as u8,
        lost as u8,
        (st.csma.cad_busy_view() >> 8) as u8,
        st.csma.cad_busy_view() as u8,
        (st.csma.defer >> 8) as u8,
        st.csma.defer as u8,
    ];
    send_frame(tx, EVT_INFO, &info).await;
}

/// `EVT_STATS` — 24 bytes, all real (C4). The old handler replied `[0u8; 24]` with a comment saying
/// it existed only so the host would not time out.
async fn send_stats(tx: &mut UartTx<'static, Async>, st: &State) {
    let mut p = [0u8; 24];
    p[0..4].copy_from_slice(&st.plane.rx.to_be_bytes());
    p[4..8].copy_from_slice(&st.plane.filtered.to_be_bytes());
    p[8..12].copy_from_slice(&st.plane.deduped.to_be_bytes());
    p[12..16].copy_from_slice(&st.plane.served.to_be_bytes());
    p[16..20].copy_from_slice(&st.plane.relayed.to_be_bytes());
    p[20..22].copy_from_slice(&st.csma.cad_busy_view().to_be_bytes());
    p[22..24].copy_from_slice(&st.csma.defer.to_be_bytes());
    send_frame(tx, EVT_STATS, &p).await;
}

/// **EVT_CAP — the one place this node describes itself** (7E-A5 **v3**). 34 bytes, every multi-byte
/// field big-endian. Every value is a source constant or a verified property of this build; nothing
/// is guessed, because a fabricated number is worse than 0 — the host believes it.
///
/// ★ **This describes the CURRENT PHY.** `max_payload`, `sf_min`/`sf_max`, the band and
/// `sched_gran_ns` are all properties of the modulation in effect, not of the board — which is why
/// `CMD_SET_PHY` answers with a WHOLE new `EVT_CAP` and the host replaces its profile rather than
/// patching fields. On this node there is one PHY to describe, so the values below are constant;
/// the day FSK lands they stop being (see [`PHY_BITMAP`]).
///
/// ⚠ **`radio_kind` names the PART, not the mode** in v3: 0 = SX1262, 1 = SX1276, 2 = LR2021. That
/// is a redefinition for the LR2021 (whose v2 kinds 2 and 3 encoded FLRC vs LoRa as separate
/// identities) and a no-op here — this node reported 1 under v2 and reports 1 under v3 — so a v2
/// host reading this v3 node still gets a sane kind.
///
/// ```text
///  [0]      proto_ver     = 3
///  [1]      radio_kind    = 1 (SX1276 — the PART)
///  [2..6]   freq_min_hz   = 902_000_000  } the band this BOARD is matched for, not the SX1276's
///  [6..10]  freq_max_hz   = 928_000_000  } 137-1020 MHz silicon range (see FREQ_MIN_HZ)
///  [10]     pwr_min_dbm   = 2            } real dBm: the PA_BOOST interval lora-phy's Sx1276
///  [11]     pwr_max_dbm   = 20           } set_tx_power clamps to (PWR_MIN_DBM / PWR_MAX_DBM)
///  [12..16] stamp_hz      = embassy-time TICK_HZ (1_000_000 with esp-rtos's tick-hz-1_000_000)
///  [16]     stamp_kind    = 2 (SOFTWARE counter: the MCU reads its own monotonic clock when the
///                              DIO0 interrupt wakes it. The SX1276 has NO RX-time capture
///                              register, so a hardware stamp is impossible on this radio.)
///  [17..19] max_payload   = 247 (RX_MAX — the binding serial-framing limit)
///  [19..23] cmd_bitmap    = CMD_BITMAP
///  [23]     sf_min        = 7   } SF6 needs an implicit header and would break fleet interop
///  [24]     sf_max        = 12  }
///  [25..29] sched_gran_ns = SCHED_GRAN_NS = 99_000 (P1). There is still no TSF comparator and no
///                               delayed key-up on an SX1276; what there is, is an MCU that programs
///                               the frame ahead of the deadline and leaves one 2-byte SPI write to
///                               fire. The number is a SUM of source constants — timer tick + that
///                               write + the PA ramp + an unmeasured-overhead budget rounded up —
///                               derived term by term at `SCHED_GRAN_NS`. The firmware measures its
///                               own error against the same counter and reports every miss.
///                               ★ v3: this is the FIRMWARE-side granularity and it is the same on
///                               `CMD_TX_AT` and `CMD_TX_AT_ABS`. What the absolute opcode removes
///                               is the host->node serial transit, which the relative one adds to
///                               the host's placement and which no field here can express — see
///                               `SERIAL_RTT_MEAN_US`.
///  [29..33] phy_bitmap    = PHY_BITMAP = 0x0000_0001 (bit N <=> SetPacketType N is usable). One
///                               bit, because this pass brought up one PHY. (C2)
///  [33]     phy_current   = PHY_CURRENT = 0 (LoRa)
/// ```
async fn send_cap(tx: &mut UartTx<'static, Async>) {
    let mut c = [0u8; 34];
    c[0] = 3;
    c[1] = 1; // radio_kind: SX1276 — the PART (v3 numbering)
    c[2..6].copy_from_slice(&FREQ_MIN_HZ.to_be_bytes());
    c[6..10].copy_from_slice(&FREQ_MAX_HZ.to_be_bytes());
    c[10] = PWR_MIN_DBM as u8;
    c[11] = PWR_MAX_DBM as u8;
    c[12..16].copy_from_slice(&STAMP_HZ.to_be_bytes());
    c[16] = 2; // stamp_kind: software counter
    c[17..19].copy_from_slice(&(RX_MAX as u16).to_be_bytes());
    c[19..23].copy_from_slice(&CMD_BITMAP.to_be_bytes());
    c[23] = SF_MIN;
    c[24] = SF_MAX;
    c[25..29].copy_from_slice(&SCHED_GRAN_NS.to_be_bytes());
    c[29..33].copy_from_slice(&PHY_BITMAP.to_be_bytes()); // C2
    c[33] = PHY_CURRENT;
    send_frame(tx, EVT_CAP, &c).await;
}

/// **★ H2 — `EVT_HOPTRACE` (0x8E): this node's own hop timeline.**
///
/// ```text
///  [0..4]  stamp_hz  = STAMP_HZ, THIS node's tick rate (== EVT_CAP.stamp_hz, == EVT_RX.ts units)
///  [4]     n         = entries returned, 0..=HOPTRACE_MAX (32), MOST RECENT LAST
///  then n * {
///    [0]     idx     = bits 5:0 RegHopChannel's FhssPresentChannel, verbatim;
///                      bit 7 HOPTRACE_TX_FLAG = the hop was taken while TRANSMITTING (H4)
///    [1..5]  t_ticks = Instant::now() at the top of `service_hop`, low 32 bits, in stamp_hz units
///  }
/// ```
///
/// `stamp_hz` is on the wire, and the ticks are NOT converted to microseconds in firmware, for a
/// reason that is the whole design: the host divides. A firmware-side conversion would force a
/// 16 MHz node to throw away resolution to match this 1 MHz one, and the comparison between the two
/// nodes is the measurement.
///
/// **`n = 0` is a real answer, not a failure**: it means this node has not serviced a hop since
/// boot — the ring is only ever written by [`Radio::service_hop`]. Reading does not clear it and
/// neither does `CMD_SET_HOP 0`: a trace read after a run is precisely the use case, and the
/// alternative would delete the measurement at the moment it was taken. (At `hop_period = 8` /
/// SF7 / BW125 the 32-entry ring turns over in 262 ms, so a stale entry from an earlier run cannot
/// survive into a new one anyway — and if one did, its timestamp would say so.)
///
/// ★ **The LR2021 node implements the same rule** (`hoptrace::HopTrace::arm`): arming a plan clears
/// the ring, disarming does not. That agreement is load-bearing, not tidiness. If one node cleared
/// on `CMD_SET_HOP 0` and the other did not, a harness that stopped the hopping before pulling the
/// timeline would get a full trace from one part and `n = 0` from the other — and `n = 0` from a
/// part that hops is indistinguishable, at the host, from **"this part does not signal its hops at
/// all"**, which is a conclusion the measurement protocol explicitly draws. The one difference
/// that survives is `idx`, and it is called out below.
///
/// This node never answers `EVT_UNSUPPORTED [0x20, NO_HARDWARE]`: it can stamp its hops, on the
/// same counter it stamps everything else with. That reply is reserved for a node that genuinely
/// cannot, and the rule it enforces — never a fabricated timeline — is why `n = 0` is returned
/// plainly instead of being padded into something that looks like data.
async fn send_hoptrace(tx: &mut UartTx<'static, Async>, hop: &Hop) {
    let mut p = [0u8; 4 + 1 + HOPTRACE_MAX * HOPTRACE_ENTRY_BYTES];
    let n = hop.trace.encode(&mut p);
    send_frame(tx, EVT_HOPTRACE, &p[..n]).await;
}

/// `EVT_PHY_ERR` — a PHY this node ADVERTISES that the chip then refused, carrying the chip's own
/// status byte (`RegOpMode` here: bit 7 LongRangeMode, bits 2:0 the mode). Never used for a PHY
/// outside `phy_bitmap` — that is a host asking for something never claimed, and it gets the
/// fleet's existing `EVT_UNSUPPORTED [cmd, OUT_OF_RANGE]`.
async fn send_phy_err(tx: &mut UartTx<'static, Async>, phy: u8, chip_status: u8) {
    send_frame(tx, EVT_PHY_ERR, &[phy, chip_status]).await;
}

async fn send_unsupported(tx: &mut UartTx<'static, Async>, cmd: u8, reason: u8) {
    send_frame(tx, EVT_UNSUPPORTED, &[cmd, reason]).await;
}

// =================================================================================================
// Command dispatch
// =================================================================================================

async fn handle_cmd(
    f: &Frame,
    radio: &mut Radio,
    st: &mut State,
    tx: &mut UartTx<'static, Async>,
    mdltn: &mut ModulationParams,
    rx_pkt: &mut PacketParams,
) {
    let buf = &f.payload[..f.len];
    match f.typ {
        // ---- transmit ----
        CMD_TX => {
            let air = airtime_ms(
                sf_num(st.p.sf),
                bw_hz(st.p.bw),
                cr_num(st.p.cr),
                f.len,
                st.csma.preamble,
            );
            let ok = radio
                .transmit(mdltn, st.p.pwr, st.csma.preamble, buf, air, st.p.freq_hz)
                .await
                .is_ok();
            if !ok {
                st.radio_errors = st.radio_errors.saturating_add(1);
            }
            send_frame(tx, EVT_TXDONE, &[ok as u8, 0]).await;
        }
        CMD_TX_LBT => {
            let air = airtime_ms(
                sf_num(st.p.sf),
                bw_hz(st.p.bw),
                cr_num(st.p.cr),
                f.len,
                st.csma.preamble,
            );
            send_frame(
                tx,
                EVT_TX_STARTED,
                &(air.min(u16::MAX as u32) as u16).to_be_bytes(),
            )
            .await;
            let mut copy = [0u8; 255];
            copy[..f.len].copy_from_slice(buf);
            let (sent, attempts) = lbt_tx(radio, st, mdltn, &copy[..f.len]).await;
            send_frame(tx, EVT_TXDONE, &[sent as u8, attempts]).await;
        }

        // ---- knobs ----
        // ★ P5 — THE FASTEST RETUNE IN THE FLEET. MEASURED 2026-08-28, host command to EVT_INFO,
        // n=8..10 with alternating values so nothing was a no-op:
        //
        //     Heltec SX1276   5 597 µs        <- this path
        //     LR2021 FLRC    52 798 µs        (9.4x slower)
        //     Waveshare SX1262 160 866 µs / 160 150 µs on two independent dongles  (29x slower)
        //
        // That single number is what makes this the only node in the fleet that can plausibly do
        // name-keyed frequency hopping at a useful dwell. It is cheap for a specific reason: NOTHING
        // here talks to the chip. `rebuild` is pure arithmetic (`create_modulation_params` /
        // `create_packet_params` build structs), and the new frequency reaches the SX1276 in the
        // `arm_rx` the main loop performs next — three `RegFrf*` writes inside a re-arm the loop was
        // going to do anyway.
        //
        // ⚠ DO NOT add an image calibration, a standby round-trip, a `set_channel` "to make it take
        // effect now", or a read-back verify to this handler or to `arm_rx`. Each is one chip
        // round-trip, and enough of them is how the SX1262 node arrived at 161 ms. Anything added to
        // P1-P3 was deliberately kept off this path: `apply_detector` and `apply_lna` issue ZERO SPI
        // transactions until the host has actually set those knobs.
        CMD_SET_FREQ if f.len >= 4 => {
            let prev = st.p;
            st.p.freq_hz = u32::from_be_bytes([buf[0], buf[1], buf[2], buf[3]]);
            if !rebuild(radio, st, mdltn, rx_pkt) {
                st.p = prev; // rejected: report what the radio is really on, not what was asked
            }
            // ★ C1's invariant, held from this side: hop channel 0 IS the base frequency, so a
            // retune moves hop[0] with it and `EVT_INFO.freq`, the modulation params and the
            // channel a packet actually starts on stay one value. Deliberately NOT an EVT_LOG
            // warning about the list and the base diverging: an EVT_LOG here would be ~40 bytes
            // ahead of the EVT_INFO this retune is timed to, i.e. ~3.5 ms of 115200 line time on
            // a 5 597 µs path. This is one shift and one divide, no SPI and no UART, and it is
            // skipped entirely while hopping is off — the P5 budget below is untouched.
            if radio.hop.enabled && radio.hop.n > 0 {
                radio.hop.hz[0] = st.p.freq_hz;
                radio.hop.frf[0] = regs::frf_of_hz(st.p.freq_hz);
            }
            send_info(tx, radio, st).await;
        }
        // C1: `bw_hz_of_code` decodes the canonical 0/1/2 AND the legacy SX1262 0x04/0x05/0x06.
        CMD_SET_MOD if f.len >= 3 => {
            let prev = st.p;
            st.p.sf = sf_from(buf[0]);
            st.p.bw = bw_from(buf[1]);
            st.p.cr = cr_from(buf[2]);
            if !rebuild(radio, st, mdltn, rx_pkt) {
                st.p = prev;
            }
            send_info(tx, radio, st).await;
        }
        CMD_SET_PWR if f.len >= 1 => {
            // Clamp to the range EVT_CAP advertises so the mirror EVT_INFO reports cannot lie about
            // what the PA was actually programmed with.
            st.p.pwr = (buf[0] as i8).clamp(PWR_MIN_DBM, PWR_MAX_DBM) as i32;
            send_info(tx, radio, st).await;
        }
        // Full parity with the SX1262 node: any sync byte, written straight to RegSyncWord. lora-phy
        // only offers public/private via `init_lora`, which is why this goes through the raw handle.
        CMD_SET_SYNC if f.len >= 1 => {
            let _ = radio.to_standby().await;
            if radio.raw.write_reg(regs::REG_SYNC_WORD, buf[0]).await.is_err() {
                st.radio_errors = st.radio_errors.saturating_add(1);
            }
            send_info(tx, radio, st).await;
        }
        CMD_SET_PREAMBLE if f.len >= 2 => {
            let prev = st.csma.preamble;
            st.csma.preamble = u16::from_be_bytes([buf[0], buf[1]]).max(1);
            if !rebuild(radio, st, mdltn, rx_pkt) {
                st.csma.preamble = prev;
            }
            send_info(tx, radio, st).await;
        }
        // Note: `next_beacon` lives in the main loop and is not reset here, so enabling the beacon
        // on a node that has been quiet fires one immediately (its deadline is already past) and
        // then settles into the period. That is deliberate — the first beacon confirms the knob
        // took effect without waiting a full period for evidence.
        CMD_SET_BEACON if f.len >= 1 => {
            st.beacon_on = buf[0] != 0;
            if f.len >= 2 {
                // Second byte scales a 10 s base period (min x1). The Waveshare counts main-loop
                // iterations; this node has a real clock, so the period is real time.
                st.beacon_period = Duration::from_secs(10 * buf[1].max(1) as u64);
            }
            send_info(tx, radio, st).await;
        }

        // ---- sensing ----
        CMD_CAD => {
            let tsym = tsym_us(sf_num(st.p.sf), bw_hz(st.p.bw));
            let busy = match radio.cad(mdltn, tsym, st.p.freq_hz).await {
                Ok(b) => {
                    if b {
                        st.csma.cad_busy = st.csma.cad_busy.wrapping_add(1);
                    }
                    b as u8
                }
                Err(_) => {
                    st.radio_errors = st.radio_errors.saturating_add(1);
                    0
                }
            };
            send_frame(tx, EVT_CAD, &[busy]).await;
        }
        CMD_GET_RSSI => {
            let r = sense_rssi_dbm(radio, mdltn, rx_pkt, st.p.freq_hz)
                .await
                .unwrap_or_else(|| {
                    st.radio_errors = st.radio_errors.saturating_add(1);
                    i16::MIN
                });
            send_frame(tx, EVT_RSSI, &r.to_be_bytes()).await;
        }
        // Sweep SF7..12 by CAD and report whichever a transmitter is actually using (ASFS
        // primitive). Only meaningful now that CAD works at all (C2).
        CMD_SF_SCAN => {
            let mut found = 0u8;
            for s in SF_MIN..=SF_MAX {
                let Ok(m) = radio
                    .chip
                    .create_modulation_params(sf_from(s), st.p.bw, st.p.cr, st.p.freq_hz)
                else {
                    continue;
                };
                if matches!(radio.cad(&m, tsym_us(s, bw_hz(st.p.bw)), st.p.freq_hz).await, Ok(true)) {
                    found = s;
                    break;
                }
            }
            let _ = rebuild(radio, st, mdltn, rx_pkt); // restore the operating modulation
            send_frame(tx, EVT_SF_DETECTED, &[found]).await;
        }
        // ★ P2 — the SX1276 CAD detector, which the v2 survey recorded as absent.
        //
        // What is TRUE: the SX1276 has no `SetCadParams` (0x88) command, and no register that sets a
        // CAD SYMBOL COUNT — its detection window is fixed by the modem.
        // What is FALSE: that there is therefore nothing to tune. `RegDetectOptimize` (0x31) and
        // `RegDetectionThreshold` (0x37) ARE the detector; `lora-phy` just leaves them unreachable,
        // which is exactly what the raw `regs::SharedSpi` handle exists for.
        //
        // Field by field, so that nothing is fabricated:
        //   sym      -> **IGNORED**. No SX1276 counterpart, per the paragraph above. The
        //               firmware-side half of the same idea — repeat the CAD N times — already has
        //               its own opcode in `CMD_SET_SENSE_CFG`'s `cad_repeat`; quietly re-pointing
        //               `sym` at it would be an invented mapping, so `sym` does nothing here.
        //   det_peak -> RegDetectionThreshold, clamped to the datasheet's [0x0A, 0x0C] span.
        //   det_min  -> RegDetectOptimize bits 2:0, clamped to the two legal codes {0x03, 0x05}.
        //               ⚠ On this chip that field is an SF-class detector selector, NOT the SX1262's
        //               "minimum symbol recognition value" — no such counter exists here. Spelled
        //               out in `regs::clamp_det_opt` rather than hidden behind a scaling formula.
        //
        // A clamp is never silent: if any byte did not land as sent, an EVT_LOG says what was
        // actually written. The pair is then re-applied after every `set_modulation_params` (see
        // `Radio::apply_detector`), without which this knob would be inert — the characteristic
        // defect of this stack, and the reason the survey's "unsupported" was worth re-checking.
        CMD_SET_CAD_CFG if f.len >= 3 => {
            let thresh = regs::clamp_det_thresh(buf[1]);
            let opt = regs::clamp_det_opt(buf[2]);
            let _ = radio.to_standby().await;
            let written = radio.raw.set_detector(opt, thresh).await;
            match written {
                Ok(full) => radio.det = Some((full, thresh)),
                Err(_) => st.radio_errors = st.radio_errors.saturating_add(1),
            }
            if buf[0] != 0 || buf[1] != thresh || buf[2] != opt {
                let mut lg = BufWriter::new();
                let _ = write!(
                    lg,
                    "cad_cfg sym={} ignored, opt=0x{opt:02X} thr=0x{thresh:02X}",
                    buf[0]
                );
                send_frame(tx, EVT_LOG, lg.as_slice()).await;
            }
            send_info(tx, radio, st).await;
        }
        CMD_SET_LBT_CFG if f.len >= 4 => {
            st.csma.lbt_cw = u16::from_be_bytes([buf[0], buf[1]]) as u32;
            st.csma.lbt_max_backoff = buf[2];
            st.csma.lbt_max_attempts = buf[3];
            send_info(tx, radio, st).await;
        }
        // ★ P3 — receive gain, via RegLna (0x0C). Payload shape is the Waveshare node's byte for
        // byte, because that node is the reference for this opcode: one boolean, 0 = the chip's
        // power-saving default, anything else = boosted (LnaBoostHf = 11, 150% LNA current, ~+3 dB
        // sensitivity). Both bytes come from `regs::lna_reg`, which also records why the SX1276's
        // six-step LnaGain field is deliberately not exposed as a second byte.
        //
        // Stored, not merely written: `lora-phy` rewrites RegLna from its fixed `rx_boost` flag on
        // every `do_rx` AND every `do_cad`, so a write-once knob would survive exactly until the
        // next arm. `Radio::apply_lna` puts it back after each one, and costs nothing at all unless
        // the host selected power-saving (the boosted byte is identical to the driver's own).
        CMD_SET_RX_GAIN if f.len >= 1 => {
            // Exactly one byte, and only 0 and 1 are defined fleet-wide (0 = the chip's power-on
            // default, 1 = boosted). Anything else is OUT_OF_RANGE rather than folded into
            // "boosted": plain `buf[0] != 0` would hand a host that sent 2 meaning some third gain
            // step the boosted setting, with nothing on the wire to say it had been misunderstood.
            // The other two nodes already refuse it — the LR2021 because it HAS a 0..13 manual
            // ladder and will not let `1` mean "boosted" here and "lowest manual gain" there — so
            // accepting it here would make one byte mean different things on different nodes, which
            // is how a capability statement stops being true.
            if buf[0] > 1 {
                send_unsupported(tx, CMD_SET_RX_GAIN, UNSUP_OUT_OF_RANGE).await;
                return;
            }
            let v = regs::lna_reg(buf[0] != 0);
            radio.lna = Some(v);
            // Written immediately as well as on every future arm, so the change takes effect for the
            // RX already in progress rather than at the next re-arm. RegLna is read/write in every
            // mode, so this needs no standby round-trip (and must not take one — see P5).
            if radio.raw.write_reg(regs::REG_LNA, v).await.is_err() {
                st.radio_errors = st.radio_errors.saturating_add(1);
            }
            send_info(tx, radio, st).await;
        }
        CMD_SET_SENSE_CFG if f.len >= 3 => {
            st.csma.rssi_thresh = i16::from_be_bytes([buf[0], buf[1]]);
            st.csma.cad_repeat = buf[2].max(1);
            send_info(tx, radio, st).await;
        }

        // ---- on-device NDN data plane ----
        CMD_SET_NAME_FILTER => {
            let mut hashes = [0u64; 24];
            let count = (f.len / 8).min(hashes.len());
            for (i, h) in hashes[..count].iter_mut().enumerate() {
                let o = i * 8;
                *h = u64::from_be_bytes([
                    buf[o], buf[o + 1], buf[o + 2], buf[o + 3],
                    buf[o + 4], buf[o + 5], buf[o + 6], buf[o + 7],
                ]);
            }
            st.plane.set_filter(&hashes[..count]);
            send_info(tx, radio, st).await;
        }
        CMD_SET_RELAY => {
            let mut hashes = [0u64; 24];
            let count = (f.len / 8).min(hashes.len());
            for (i, h) in hashes[..count].iter_mut().enumerate() {
                let o = i * 8;
                *h = u64::from_be_bytes([
                    buf[o], buf[o + 1], buf[o + 2], buf[o + 3],
                    buf[o + 4], buf[o + 5], buf[o + 6], buf[o + 7],
                ]);
            }
            st.plane.set_relay(&hashes[..count]);
            send_info(tx, radio, st).await;
        }
        CMD_DATAPLANE if f.len >= 5 => {
            st.plane.set_cs_serve(buf[0] != 0);
            st.plane.set_dedup(buf[1] != 0);
            st.plane.set_hop(buf[2] != 0, buf[3], buf[4]);
            send_info(tx, radio, st).await;
        }

        // ---- observability ----
        CMD_GET_STATS => send_stats(tx, st).await,
        CMD_RESET_STATS => {
            st.plane.reset_stats();
            // Re-baseline the resettable VIEW; `cad_busy` itself keeps free-running because
            // EVT_SENSE.activity is defined as a free-running counter the host differences.
            st.csma.cad_busy_base = st.csma.cad_busy;
            st.csma.defer = 0;
            st.rx_trunc = 0;
            st.radio_errors = 0;
            SERIAL_LOST.store(0, Ordering::Relaxed);
            send_info(tx, radio, st).await;
        }
        CMD_SET_DEBUG if f.len >= 1 => {
            st.debug = buf[0] != 0;
            if st.debug {
                // A live chip-state dump, so a debugging session starts from what the SX1276 is
                // actually doing rather than from what the firmware believes. RegModemStat is the
                // register that answers "is the modem seeing anything?": bit0 signal detected,
                // bit1 signal synchronised, bit2 RX ongoing, bit3 header info valid, bit4 modem
                // clear. EVT_INFO has no room for it (its 19-byte layout is frozen), so it goes
                // here.
                let op = radio.raw.read_reg(regs::REG_OP_MODE).await.unwrap_or(0);
                let ms = radio.raw.read_reg(regs::REG_MODEM_STAT).await.unwrap_or(0);
                let sy = radio.raw.read_reg(regs::REG_SYNC_WORD).await.unwrap_or(0);
                let mut lg = BufWriter::new();
                let _ = write!(
                    lg,
                    "chip op_mode=0x{op:02X} modem_stat=0x{ms:02X} sync=0x{sy:02X} err={} trunc={}",
                    st.radio_errors, st.rx_trunc
                );
                send_frame(tx, EVT_LOG, lg.as_slice()).await;
                // P1-P3 knob state, on its own line because `BufWriter` is 96 bytes. A `0` means
                // "the host has never set this knob", i.e. the chip is running whatever `lora-phy`
                // derives — not "the knob is zero".
                let (dopt, dthr) = radio.det.unwrap_or((0, 0));
                let lna = radio.lna.unwrap_or(0);
                let mut lg2 = BufWriter::new();
                let _ = write!(
                    lg2,
                    "knobs det=0x{dopt:02X}/0x{dthr:02X} lna=0x{lna:02X} sched_late={} gran={}us",
                    st.sched_late,
                    SCHED_GRAN_NS / 1_000
                );
                send_frame(tx, EVT_LOG, lg2.as_slice()).await;
                // C1 — the hop list and what the modem has actually done with it. `pll_to` is the
                // chip's own verdict (RegHopChannel bit 7), not an inference: a non-zero count is
                // the synthesiser failing to settle inside the dwell, i.e. hop_period too short
                // for this SF/BW.
                let mut lg3 = BufWriter::new();
                let _ = write!(
                    lg3,
                    "hop on={} n={} per={} hops={} pll_to={} idx={} hz0={}",
                    radio.hop.enabled as u8,
                    radio.hop.n,
                    radio.hop.period,
                    radio.hop.hops,
                    radio.hop.pll_timeouts,
                    radio.hop.last_idx,
                    radio.hop.hz[0]
                );
                send_frame(tx, EVT_LOG, lg3.as_slice()).await;
                // C2/C3 — the PHY surface, and the one number EVT_CAP has no field for: the
                // MEASURED serial round trip the relative scheduled-TX opcode pays and the
                // absolute one does not.
                let mut lg4 = BufWriter::new();
                let _ = write!(
                    lg4,
                    "phy cur={PHY_CURRENT} bitmap=0x{PHY_BITMAP:08X} rtt_mean={SERIAL_RTT_MEAN_US}us"
                );
                send_frame(tx, EVT_LOG, lg4.as_slice()).await;
                // ★ H2/H5 — the hop trace's own state, and the one self-description word EVT_CAP
                // has no room for. `trace` is how many of the 32 ring slots are filled (NOT a hop
                // count — that is `hops` on the line above, which the host differences); `txf` is
                // how many of them were taken while transmitting, which is the H4 split visible
                // without pulling the whole trace. `bitmap_ext` advertises opcodes 32..63, i.e.
                // 0x20 CMD_GET_HOPTRACE — see CMD_BITMAP_EXT for why it cannot live in EVT_CAP.
                let txf = radio.hop.trace.tx_flagged();
                let mut lg5 = BufWriter::new();
                let _ = write!(
                    lg5,
                    "hoptrace n={}/{HOPTRACE_MAX} txf={txf} stamp_hz={STAMP_HZ} bitmap_ext=0x{CMD_BITMAP_EXT:08X}",
                    radio.hop.trace.len
                );
                send_frame(tx, EVT_LOG, lg5.as_slice()).await;
            }
            send_info(tx, radio, st).await;
        }
        CMD_GET_INFO => {
            // C7: state what is running before stating what it is doing.
            send_build_id(tx).await;
            send_info(tx, radio, st).await;
        }

        // ---- 7E-A5 v2 ----
        // The SAME counter EVT_RX stamps with, at full 64-bit width (the EVT_RX field is its low 32
        // bits and wraps every ~71 min at 1 MHz). Units are EVT_CAP.stamp_hz.
        CMD_READ_CLOCK => {
            send_frame(tx, EVT_CLOCK, &Instant::now().as_ticks().to_be_bytes()).await;
        }
        // ★ P1 — scheduled TX. `[delay_us u32 BE][frame]`.
        //
        // The SX1276 has no TSF comparator and no delayed key-up, so the MCU is the queue. The
        // earlier reading — "an embassy timer fires the SPI write whenever the executor gets round
        // to it, which is exactly the jitter TX_AT exists to remove" — was right about running the
        // WHOLE transmit from the deadline (~2 ms, and payload-length dependent) and wrong about
        // what has to run there. `Radio::stage_tx` programs the chip AHEAD of time and leaves
        // `Radio::fire_tx` a single 2-byte `RegOpMode <- TX` write, so what the deadline buys is one
        // tick of timer quantisation, 8 µs of SPI and 40 µs of PA ramp: `SCHED_GRAN_NS`, 99 µs,
        // derived term by term where that constant is defined. That is a real scheduled TX.
        //
        // The delay is measured from `Instant::now()` — the SAME counter EVT_RX stamps with and
        // CMD_READ_CLOCK returns — so the host's `inject_at_clock` tick arithmetic and this delay
        // live in one timebase, and `report_sched_error` can state the achieved error in those same
        // units after the fact.
        CMD_TX_AT if f.len >= 5 => {
            let delay_us = u32::from_be_bytes([buf[0], buf[1], buf[2], buf[3]]);
            let frame = &buf[4..f.len];
            if delay_us > SCHED_MAX_DELAY_US {
                send_unsupported(tx, CMD_TX_AT, UNSUP_OUT_OF_RANGE).await;
            } else if st.sched.state.is_armed() {
                // The slot is one deep, and a second request is REFUSED rather than allowed to
                // replace the first: replacing would drop an already-accepted frame with no word to
                // the host. `[0, 0]` is the same "did not go" the host's `inject_after` already maps
                // to an error, so this needs no new wire vocabulary.
                send_frame(tx, EVT_TXDONE, &[0, 0]).await;
            } else {
                let air = airtime_ms(
                    sf_num(st.p.sf),
                    bw_hz(st.p.bw),
                    cr_num(st.p.cr),
                    frame.len(),
                    st.csma.preamble,
                );
                // NOTE: `EVT_TX_STARTED` is NOT emitted here. It goes out at the staging point,
                // 1-2.7 ms before key-up — see the `SchedTx::Pending` arm in `main`, which explains
                // why neither acceptance nor the deadline itself is a legal place for it.
                st.sched.buf[..frame.len()].copy_from_slice(frame);
                let fire_at = Instant::now() + Duration::from_micros(delay_us as u64);
                st.sched.state = SchedTx::pending(fire_at, frame.len(), air);
            }
        }
        // ---- 7E-A5 v3 ----
        // ★ C2 — `CMD_SET_PHY [packet_type]`. Modulation is a KNOB, not an identity.
        //
        // Two answers, and the split is the contract's:
        //   * the requested PHY is NOT in `EVT_CAP.phy_bitmap` -> `EVT_UNSUPPORTED [0x1D,
        //     OUT_OF_RANGE]`. The host asked for something this node never claimed; that is the
        //     fleet's existing vocabulary for exactly that, and it needs no new event.
        //   * it IS in the bitmap but the chip does not corroborate -> `EVT_PHY_ERR [phy,
        //     RegOpMode]`. That path is LIVE on this node, not decoration: selecting LoRa reads
        //     `RegOpMode` back and checks bit 7 `LongRangeMode`, so a chip that has lost LoRa mode
        //     (or an SPI bus that has stopped answering) is reported with the chip's literal status
        //     byte instead of a cheerful EVT_CAP.
        //
        // A switch to the PHY already running is a verify, not a re-init: `LongRangeMode` is only
        // writable in SLEEP, so a real transition would cost a sleep + full re-init + a rebuilt
        // parameter set, and paying that to arrive where we already are would be a knob that
        // damages the thing it claims to set.
        CMD_SET_PHY if f.len >= 1 => {
            let want = buf[0];
            if want >= 32 || (PHY_BITMAP >> want) & 1 == 0 {
                send_unsupported(tx, CMD_SET_PHY, UNSUP_OUT_OF_RANGE).await;
                return;
            }
            match radio.raw.read_reg(regs::REG_OP_MODE).await {
                // Bit 7 LongRangeMode set == the modem really is in LoRa, which is the only PHY
                // this node advertises. Anything else is the chip refusing, reported as such.
                Ok(op) if op & 0x80 != 0 => send_cap(tx).await,
                Ok(op) => {
                    st.radio_errors = st.radio_errors.saturating_add(1);
                    send_phy_err(tx, want, op).await;
                }
                Err(_) => {
                    // The status byte could not be read at all. `0x00` is sent because that is what
                    // this firmware actually knows — nothing — and an EVT_LOG says so, rather than
                    // letting a bus failure masquerade as a chip state (0x00 is also a legal
                    // RegOpMode value, FSK+SLEEP, which is exactly why it must not go out alone).
                    st.radio_errors = st.radio_errors.saturating_add(1);
                    send_frame(tx, EVT_LOG, b"set_phy: RegOpMode readback failed (SPI)").await;
                    send_phy_err(tx, want, 0x00).await;
                }
            }
        }

        // ---- ★ C1 — `CMD_SET_HOP [hop_ctrl][hop_period u16 BE][n][freq_hz u32 BE]*n` ----
        //
        // Real SX1276 intra-packet frequency hopping, reaching past `lora-phy` to the three
        // registers it does not model (`RegHopPeriod` 0x24, `RegFrf` 0x06-0x08, `RegHopChannel`
        // 0x1C) and to the interrupt it masks in every mode (`FhssChangedChannel`). The mechanism,
        // the DIO1 choice and the cancel-safety argument are all documented at [`Hop`],
        // [`Radio::service_hop`] and [`Radio::hop_irq`].
        //
        // Every refusal below is a knob the hardware cannot do, answered rather than fudged:
        //   * `hop_ctrl` outside {0, 1} — an unknown mode is not "on".
        //   * `hop_period` above 255 — `RegHopPeriod` is EIGHT bits. Truncating a u16 would hop up
        //     to 256x faster than asked and desynchronise the pair silently.
        //   * `hop_period` 0 with hopping on — 0 is how the register means "off"; asking for both
        //     at once is a contradiction, not a default.
        //   * `n` above HOP_MAX, or 0 with hopping on.
        //   * any channel outside the 902-928 MHz window this BOARD is matched for. The SX1276
        //     silicon reaches 137-1020 MHz; the Heltec V2's PA match, SAW filter and antenna do
        //     not, so a hop out there is a hop into a channel this node barely radiates on.
        //
        // ★ THE INVARIANT: **hop channel 0 is the base frequency.** Enabling sets `st.p.freq_hz`
        // from `list[0]`, so `EVT_INFO.freq`, the modulation parameters and the channel a packet
        // actually starts on are one value and cannot drift. `CMD_SET_FREQ` maintains the same
        // invariant from the other side.
        CMD_SET_HOP if f.len >= 4 => {
            let ctrl = buf[0];
            let period = u16::from_be_bytes([buf[1], buf[2]]);
            let n = buf[3] as usize;
            let on = ctrl == HOP_ON;
            if ctrl > HOP_ON || n > HOP_MAX || (on && (n == 0 || period == 0 || period > HOP_PERIOD_MAX))
            {
                send_unsupported(tx, CMD_SET_HOP, UNSUP_OUT_OF_RANGE).await;
                return;
            }
            if f.len < 4 + 4 * n {
                send_unsupported(tx, CMD_SET_HOP, UNSUP_BAD_LENGTH).await;
                return;
            }
            // Validate the WHOLE list before committing any of it: a half-applied hop list is a
            // sequence the two ends of a link no longer agree on.
            for i in 0..n {
                let o = 4 + i * 4;
                let hz = u32::from_be_bytes([buf[o], buf[o + 1], buf[o + 2], buf[o + 3]]);
                if hz < FREQ_MIN_HZ || hz > FREQ_MAX_HZ {
                    send_unsupported(tx, CMD_SET_HOP, UNSUP_OUT_OF_RANGE).await;
                    return;
                }
            }
            for i in 0..n {
                let o = 4 + i * 4;
                let hz = u32::from_be_bytes([buf[o], buf[o + 1], buf[o + 2], buf[o + 3]]);
                radio.hop.hz[i] = hz;
                // The one place the synthesiser word is computed. Doing it here is what leaves the
                // interrupt service with three register writes and no arithmetic.
                radio.hop.frf[i] = regs::frf_of_hz(hz);
            }
            // The command is a FULL statement of the hop configuration, list included, so a disable
            // that carries n = 0 clears the list. That is the wire's own shape — every `CMD_SET_HOP`
            // sends the list it wants — and it means the firmware never holds a stale sequence the
            // host has stopped believing in.
            radio.hop.n = n;
            // `period` is only range-checked on the enable path, so mirror what the REGISTER will
            // actually hold rather than a truncated u16: `set_hop_regs(false, ..)` writes 0.
            radio.hop.period = if on { period as u8 } else { 0 };
            // The chip is put in standby first for the same reason every other register knob here
            // is: a mode change underneath a half-written hop configuration is not worth the SPI it
            // would save.
            let _ = radio.to_standby().await;
            match radio.set_hop_regs(on, period as u8).await {
                Ok(()) => {
                    radio.hop.enabled = on;
                    if on {
                        // The invariant. `rebuild` is pure arithmetic — no SPI — and the new
                        // carrier reaches the chip in the `arm_rx` the main loop performs next.
                        let prev = st.p;
                        // Remember what hop[0] is about to displace, so the disable path can put it
                        // back. Only on the FIRST enable — a second `CMD_SET_HOP 1` must not record
                        // a hop frequency as the base.
                        if radio.hop.base_hz == 0 {
                            radio.hop.base_hz = prev.freq_hz;
                        }
                        st.p.freq_hz = radio.hop.hz[0];
                        if !rebuild(radio, st, mdltn, rx_pkt) {
                            st.p = prev;
                        }
                    } else {
                        // ☠ **MEASURED 2026-08-28: disabling hopping must put the CARRIER back, and
                        // this branch did not exist.** `service_hop` walks `RegFrf` across the list
                        // during a packet, and `set_hop_regs(false, 0)` only clears `RegHopPeriod` —
                        // it leaves `RegFrf` wherever the last hop parked it. The node then keeps
                        // transmitting and receiving on a hop frequency while `EVT_INFO.freq` still
                        // reports the base and `CMD_TX` still answers `ok=1`, so nothing on the wire
                        // says anything is wrong. Every peer simply goes deaf.
                        //
                        // It cost a long misdiagnosis: Heltec -> Waveshare measured 3/3 before any
                        // hop list existed and 0/4 after, which looks exactly like a cross-vendor PHY
                        // incompatibility — a whole packet-parameter investigation went into it. An
                        // explicit `CMD_SET_FREQ` restoring it to 4/4 is what identified the real
                        // cause.
                        //
                        // Restoring `hop.base_hz` and not `st.p.freq_hz` is the whole point: the
                        // enable path already overwrote `st.p.freq_hz` with `hz[0]`, so re-asserting
                        // that would faithfully put the node back on the hop list's base — which is
                        // exactly the bug, one indirection along. (First fix attempt did precisely
                        // that and still measured 0/4.)
                        if radio.hop.base_hz != 0 {
                            st.p.freq_hz = radio.hop.base_hz;
                            radio.hop.base_hz = 0;
                        }
                        let _ = rebuild(radio, st, mdltn, rx_pkt);
                    }
                }
                Err(_) => {
                    // Half-configured is the one state that must not persist: it would leave DIO1
                    // mapped to an interrupt nothing services. Force the disable path and say so.
                    st.radio_errors = st.radio_errors.saturating_add(1);
                    radio.hop.enabled = false;
                    let _ = radio.set_hop_regs(false, 0).await;
                    // Same carrier restore as the disable branch above, and for the same measured
                    // reason: this path forces hopping off, so it inherits the defect where
                    // `RegFrf` stays parked on whatever hop it stopped at.
                    let _ = rebuild(radio, st, mdltn, rx_pkt);
                    send_frame(tx, EVT_LOG, b"set_hop: SPI error, hopping left disabled").await;
                }
            }
            // Re-assert the invariant unconditionally, in the ONE direction that is always right:
            // hop[0] follows whatever frequency the radio is actually on. On the normal path this
            // is a no-op (the base was just set from hop[0]); if `rebuild` rejected the new base
            // and rolled it back, this is what stops the list and the modem disagreeing about
            // where a packet starts. Pure arithmetic, no SPI.
            if radio.hop.enabled && radio.hop.n > 0 {
                radio.hop.hz[0] = st.p.freq_hz;
                radio.hop.frf[0] = regs::frf_of_hz(st.p.freq_hz);
            }
            send_info(tx, radio, st).await;
        }

        // ---- ★ C3 — `CMD_TX_AT_ABS [target_ticks u64 BE][frame]` ----
        //
        // The same one-deep slot and the same two-phase stage/fire machine as `CMD_TX_AT` (P1) —
        // one radio, one staged frame — with the deadline read straight off the wire instead of
        // being derived from `Instant::now()` at command-processing time. `target_ticks` is on the
        // counter `CMD_READ_CLOCK` returns and `EVT_RX.ts` stamps with, at its full 64-bit width.
        //
        // Why this opcode exists is MEASURED, not argued: on the LR2021 an absolute-boundary slot
        // train fired 45/45 with a mean gap within 11 µs of nominal over 44 slots, but per-slot
        // sd 553 µs / p2p 1875 µs — the relative opcode's delay starts when the FIRMWARE processes
        // the arm, so host->device serial latency lands directly in the placement. This node's own
        // serial round trip has a MEASURED mean of 10 733 µs, over 100x its 99 µs firmware
        // granularity. An absolute target divides all of that out: the host says *when*, and its
        // own lateness can only cost it the deadline, never move the frame.
        //
        //   target in the PAST -> fire now (and `report_sched_error` states how late on the wire)
        //   target further than SCHED_MAX_DELAY_US ahead -> EVT_UNSUPPORTED [0x1F, OUT_OF_RANGE]
        CMD_TX_AT_ABS if f.len >= 9 => {
            let target = u64::from_be_bytes([
                buf[0], buf[1], buf[2], buf[3], buf[4], buf[5], buf[6], buf[7],
            ]);
            let frame = &buf[8..f.len];
            let now = Instant::now();
            let target_at = Instant::from_ticks(target);
            if target_at > now + Duration::from_micros(SCHED_MAX_DELAY_US as u64) {
                // Same bound as the relative opcode, so "how far ahead can I schedule?" has one
                // answer on this node whichever opcode the host reaches for.
                send_unsupported(tx, CMD_TX_AT_ABS, UNSUP_OUT_OF_RANGE).await;
            } else if st.sched.state.is_armed() {
                // One deep, and a second request is REFUSED rather than allowed to replace an
                // already-accepted frame. Identical to CMD_TX_AT's rule and for the same reason.
                send_frame(tx, EVT_TXDONE, &[0, 0]).await;
            } else {
                let air = airtime_ms(
                    sf_num(st.p.sf),
                    bw_hz(st.p.bw),
                    cr_num(st.p.cr),
                    frame.len(),
                    st.csma.preamble,
                );
                st.sched.buf[..frame.len()].copy_from_slice(frame);
                // A target already gone fires at once, and the target is passed through UNCLAMPED
                // on purpose. `SchedTx::pending` saturates `stage_at` to boot time, so `wake_at`
                // is already past, the loop stages on its next pass and keys up immediately —
                // and because `fire_at` is still the instant the HOST asked for,
                // `report_sched_error` measures the real lateness against it and puts it on the
                // wire. Clamping to `now` would have made every late frame report a ~0 µs error,
                // which is the one thing a schedule must never do.
                st.sched.state = SchedTx::pending(target_at, frame.len(), air);
            }
        }

        CMD_GET_CAP => send_cap(tx).await,

        // ---- ★ H2 — `CMD_GET_HOPTRACE` (0x20) -> `EVT_HOPTRACE` (0x8E) ----
        //
        // Read this node's own hop timeline. The point of the opcode is that it needs NO
        // cross-vendor reception: the SX1276 and the LR2021 cannot hop with each other, so the
        // link that would have carried a comparison is the very thing under test — but each node
        // can state its own hop instants on its own clock, and the host puts the two side by side.
        //
        // Free-running, wrapping, and reading does NOT clear it (the `EVT_SENSE.activity`
        // contract), so a read is safe to repeat while a measurement is running: two reads with
        // hops in between simply overlap and the host de-duplicates on the stamps.
        //
        // ⚠ 0x20 is opcode 32 and `EVT_CAP.cmd_bitmap` is a u32 that is FULL at 0x1F, so this arm
        // must sit above the catch-all — the catch-all's oracle is that bitmap, guarded by
        // `f.typ < 32`, and would answer UNKNOWN_OPCODE for an opcode that is in fact implemented
        // here. See [`CMD_BITMAP_EXT`] for why the bitmap was not widened and how a host discovers
        // this opcode instead.
        CMD_GET_HOPTRACE => send_hoptrace(tx, &radio.hop).await,

        // Channel occupancy. `activity` is the free-running, wrapping CAD-busy counter — the host
        // differences two reads over a window, never reads it as an absolute. This node advances it
        // from every channel-busy observation it makes: the CADs inside the LBT loop, CMD_CAD, and
        // the `cad_repeat` CADs this command performs itself, so polling CMD_SENSE alone gives a
        // usable busy fraction even when the node is not transmitting.
        CMD_SENSE => {
            let tsym = tsym_us(sf_num(st.p.sf), bw_hz(st.p.bw));
            for _ in 0..st.csma.cad_repeat.max(1) {
                match radio.cad(mdltn, tsym, st.p.freq_hz).await {
                    Ok(true) => st.csma.cad_busy = st.csma.cad_busy.wrapping_add(1),
                    Ok(false) => {}
                    Err(_) => st.radio_errors = st.radio_errors.saturating_add(1),
                }
            }
            let r = sense_rssi_dbm(radio, mdltn, rx_pkt, st.p.freq_hz)
                .await
                .unwrap_or_else(|| {
                    st.radio_errors = st.radio_errors.saturating_add(1);
                    i16::MIN // sentinel: no channel RSSI could be read, NOT a 0 dBm carrier
                });
            let mut p = [0u8; 4];
            p[0..2].copy_from_slice(&st.csma.cad_busy.to_be_bytes());
            p[2..4].copy_from_slice(&r.to_be_bytes());
            send_frame(tx, EVT_SENSE, &p).await;
        }
        // P4 — GD32-only: that node reboots into its ROM UART bootloader so `stm32flash` can reflash over
        // the same USB link. The ESP32 has no such opcode and needs none — the CP2102 pulls
        // DTR/RTS to drive GPIO0/EN and the ROM downloader comes up in hardware, which is what
        // `espflash` uses. Nothing here could implement it.
        CMD_ENTER_BOOTLOADER => {
            send_unsupported(tx, CMD_ENTER_BOOTLOADER, UNSUP_NO_HARDWARE).await
        }

        // NEVER silence (7E-A5 v2 rule): a command that returns nothing costs the host its whole
        // retry budget and then a failure, for what is really a one-frame answer. `CMD_BITMAP` is
        // the oracle for which reason applies, so the dispatcher and the bitmap EVT_CAP advertises
        // cannot drift apart:
        //   * the opcode is in the bitmap => we implement it, so we only got here because a length
        //     guard above rejected the payload -> BAD_LENGTH;
        //   * it is not                     -> UNKNOWN_OPCODE.
        //
        // ★ H5: the oracle now spans BOTH self-description words. `CMD_BITMAP` covers opcodes
        // 0..31 and `CMD_BITMAP_EXT` covers 32..63; the `< 32` / `< 64` guards are load-bearing,
        // because `1u32 << 32` is not 0 in Rust — it panics in debug and is a compile error as a
        // constant. Splitting the shift this way is what keeps the property the original comment
        // claims: the dispatcher and the advertised bitmaps cannot drift apart, now including the
        // opcodes that no longer fit in the one word `EVT_CAP` can carry.
        _ => {
            let known = if f.typ < 32 {
                (CMD_BITMAP >> f.typ) & 1 != 0
            } else if f.typ < 64 {
                (CMD_BITMAP_EXT >> (f.typ - 32)) & 1 != 0
            } else {
                false
            };
            let reason = if known { UNSUP_BAD_LENGTH } else { UNSUP_UNKNOWN_OPCODE };
            send_unsupported(tx, f.typ, reason).await;
        }
    }
}

/// Recompute the modulation and RX packet parameters in place after a knob change.
///
/// Returns false if the chip rejects the combination (`lora-phy` refuses 250/500 kHz below 400 MHz,
/// and SF6 without an implicit header). The caller then rolls the knobs back, so the radio and the
/// `EVT_INFO` mirror can never disagree — reporting a bandwidth the modem is not actually using is
/// exactly the failure mode C1 was.
#[must_use]
fn rebuild(
    radio: &Radio,
    st: &mut State,
    mdltn: &mut ModulationParams,
    rx_pkt: &mut PacketParams,
) -> bool {
    match build_params(radio, st) {
        Some((m, r)) => {
            *mdltn = m;
            *rx_pkt = r;
            true
        }
        None => {
            st.radio_errors = st.radio_errors.saturating_add(1);
            false
        }
    }
}
