//! Heltec WiFi LoRa 32 V2 (ESP32 + SX1276) — open Rust firmware, named-radio node C (task #54).
//!
//! A host-driven LoRa modem speaking the **7E-A5 v2** serial protocol, at feature parity with the
//! Waveshare SX1262 node (`../waveshare-lora-rs`) so one host driver treats the two as peers. The
//! air side is plain standard LoRa — no proprietary header — so it interoperates with any
//! SX127x/SX126x peer.
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
use embassy_futures::select::{select, select3, Either, Either3};
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
// 7E-A5 v2 wire protocol. Opcode numbering is FLEET-WIDE — the Waveshare node is canonical and this
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
const CMD_SET_CAD_CFG: u8 = 0x0A; //   payload = [sym, det_peak, det_min] -> EVT_UNSUPPORTED here
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
const CMD_TX_AT: u8 = 0x18; //           payload = [delay_us u32 BE][frame] -> EVT_UNSUPPORTED here
const CMD_GET_CAP: u8 = 0x1A; //         payload = []  -> EVT_CAP (29 B)
const CMD_SENSE: u8 = 0x1B; //           payload = []  -> EVT_SENSE [activity u16 BE, rssi i16 BE]

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
const EVT_CAP: u8 = 0x8B; //         29 B, see `send_cap`
const EVT_SENSE: u8 = 0x8C; //       [activity u16 BE, rssi i16 BE]
const EVT_UNSUPPORTED: u8 = 0x8F; // [cmd, reason] — never silence, never a fake success

// EVT_UNSUPPORTED reason codes (shared with the Waveshare node).
const UNSUP_UNKNOWN_OPCODE: u8 = 0x01;
const UNSUP_NO_HARDWARE: u8 = 0x02;
const UNSUP_BAD_LENGTH: u8 = 0x03;

/// **The self-description bitmap** (`EVT_CAP.cmd_bitmap`): bit N set <=> opcode N is implemented and
/// will act. It is also this firmware's own "is this opcode known?" oracle in [`handle_cmd`]'s
/// catch-all, so the bitmap and the dispatcher physically cannot drift apart.
///
/// ```text
/// set   0x01..0x09  TX, SET_FREQ, SET_MOD, SET_PWR, SET_SYNC, GET_INFO, SET_BEACON, CAD, GET_RSSI
///       0x0B..0x15  SET_LBT_CFG .. SET_DEBUG
///       0x17        READ_CLOCK
///       0x1A 0x1B   GET_CAP, SENSE
/// clear 0x00        not an opcode
///       0x0A        SET_CAD_CFG      — no SX1276 equivalent of SetCadParams(0x88); see handle_cmd
///       0x16        ENTER_BOOTLOADER — GD32 ROM-bootloader opcode; the ESP32 is entered by the
///                                      CP2102 pulling DTR/RTS, so no firmware opcode exists
///       0x18        TX_AT            — no scheduled-TX engine on the SX1276 (sched_gran_ns = 0)
///       0x19, 0x1C..0x1F unassigned / other nodes' local extensions
/// ```
/// bits 1-9 = 0x0000_03FE, bits 11-21 = 0x003F_F800, bit 23 = 0x0080_0000,
/// bits 26-27 = 0x0C00_0000  =>  0x0CBF_FBFE.
const CMD_BITMAP: u32 = 0x0CBF_FBFE;

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
struct Radio {
    chip: Chip,
    /// Second handle on the same SPI bus, for the registers `lora-phy` does not expose. See
    /// [`regs`].
    raw: SharedSpi,
    /// Mirror of the chip's mode; `RadioKind` is stateless, so the caller must track it (this is
    /// exactly what `LoRa` does internally).
    mode: RadioMode,
}

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
        self.chip.set_packet_params(rx_pkt).await?;
        self.chip.set_channel(freq_hz).await?;
        self.mode = RadioMode::Receive(RxMode::Continuous);
        self.chip.set_irq_params(Some(self.mode)).await?;
        self.chip.do_rx(RxMode::Continuous).await
    }

    /// The one cancel-safe await in the whole firmware: DIO0 going high. No SPI, no state.
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

    /// Transmit one frame and wait for TxDone. Mirrors `LoRa::prepare_for_tx` + `LoRa::tx`, except
    /// that the packet params are built here with the real payload length (`PacketParams::
    /// set_payload_length` is crate-private, `create_packet_params` is not).
    ///
    /// The TxDone wait is bounded by the frame's own computed airtime plus a wide margin: with only
    /// DIO0 wired, a missed interrupt would otherwise park the node forever and the host would see
    /// no reply at all. The timeout races `await_irq` only — never the SPI phase (C3).
    async fn transmit(
        &mut self,
        m: &ModulationParams,
        pwr: i32,
        preamble: u16,
        payload: &[u8],
        air_ms: u32,
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
        self.chip.do_tx().await?;

        let deadline = Instant::now() + Duration::from_millis(air_ms as u64 * 2 + 500);
        loop {
            match select(self.chip.await_irq(), Timer::at(deadline)).await {
                Either::First(Err(e)) => {
                    let _ = self.chip.set_standby().await;
                    self.mode = RadioMode::Standby;
                    return Err(e);
                }
                Either::First(Ok(())) => {
                    match self.chip.process_irq_event(self.mode, None, true).await {
                        Ok(Some(IrqState::Done | IrqState::PreambleReceived)) => {
                            self.mode = RadioMode::Standby;
                            return Ok(());
                        }
                        Ok(None) => continue,
                        Err(e) => {
                            let _ = self.chip.set_standby().await;
                            self.mode = RadioMode::Standby;
                            return Err(e);
                        }
                    }
                }
                // TxDone never asserted within twice the computed airtime. Recover rather than hang.
                Either::Second(_) => {
                    let _ = self.chip.set_standby().await;
                    let _ = self.chip.set_irq_params(None).await; // clears RegIrqFlags
                    self.mode = RadioMode::Standby;
                    return Err(RadioError::TransmitTimeout);
                }
            }
        }
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
        self.chip.set_channel(freq_hz).await?;
        self.mode = RadioMode::ChannelActivityDetection;
        self.chip.set_irq_params(Some(self.mode)).await?;
        self.chip.do_cad(m).await?;

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
    let iv = GenericSx127xInterfaceVariant::new(reset, dio0, None, None).unwrap();
    // tx_boost: the Heltec V2 routes the antenna to PA_BOOST, so this is a board fact, not a
    // choice — and it is what makes PWR_MIN_DBM/PWR_MAX_DBM = [2, 20] the honest range in EVT_CAP.
    // rx_boost turns on the SX1276's LNA boost (~+3 dB sensitivity), matching the Waveshare node's
    // default so a link budget measured on one node transfers to the other.
    let config = Sx127xConfig { chip: Sx1276, tcxo_used: false, tx_boost: true, rx_boost: true };
    let mut radio = Radio { chip: Sx127x::new(dev, iv, config), raw, mode: RadioMode::Sleep };
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
        if !rx_armed {
            match radio.arm_rx(&mdltn, &rx_pkt, st.p.freq_hz).await {
                Ok(()) => rx_armed = true,
                Err(_) => {
                    st.radio_errors = st.radio_errors.saturating_add(1);
                    arm_failed = true;
                }
            }
        }

        // When the third branch should fire. A failed arm needs a short retry tick — with the chip
        // not in RX, DIO0 can never assert, so without one the node would sit deaf until a host
        // command happened to arrive. The tick goes through the SAME select as everything else, so
        // a wedged radio still leaves the host link fully responsive.
        let wake_at = if arm_failed {
            Instant::now() + Duration::from_millis(250)
        } else if st.beacon_on {
            next_beacon
        } else {
            Instant::now() + Duration::from_secs(3600)
        };

        // The ONLY cancellable await (C3): a queued host command, the DIO0 level, or the beacon
        // timer. None of the three touches SPI or driver state, so dropping any of them is free.
        match select3(CMDQ.receive(), radio.wait_irq(), Timer::at(wake_at)).await {
            // ---- host command ----
            Either3::First(f) => {
                handle_cmd(&f, &mut radio, &mut st, &mut uart_tx, &mut mdltn, &mut rx_pkt).await;
                // Almost every command puts the chip in standby (or reprograms it); re-arm rather
                // than track which ones did. The cost is ~10 register writes.
                rx_armed = false;
            }

            // ---- radio interrupt ----
            Either3::Second(res) => {
                if res.is_err() {
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

            // ---- beacon, or the RX re-arm retry tick ----
            Either3::Third(_) => {
                if st.beacon_on && Instant::now() >= next_beacon {
                    next_beacon = Instant::now() + st.beacon_period;
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
                        .transmit(&mdltn, st.p.pwr, st.csma.preamble, msg.as_slice(), air, st.p.freq_hz)
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
    let _ = write!(log, "heltec-lora-rs build={BUILD_ID} proto=2 stamp_hz={STAMP_HZ}");
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

/// **EVT_CAP — the one place this node describes itself** (7E-A5 v2). 29 bytes, every multi-byte
/// field big-endian. Every value is a source constant or a verified property of this build; nothing
/// is guessed, because a fabricated number is worse than 0 — the host believes it.
///
/// ```text
///  [0]      proto_ver     = 2
///  [1]      radio_kind    = 1 (SX1276)
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
///  [25..29] sched_gran_ns = 0   no scheduled-TX engine exists on the SX1276 (no TSF comparator, no
///                               delayed key-up); 0 is the truth, not a placeholder.
/// ```
async fn send_cap(tx: &mut UartTx<'static, Async>) {
    let mut c = [0u8; 29];
    c[0] = 2;
    c[1] = 1; // radio_kind: SX1276
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
    c[25..29].copy_from_slice(&0u32.to_be_bytes());
    send_frame(tx, EVT_CAP, &c).await;
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
        CMD_SET_FREQ if f.len >= 4 => {
            let prev = st.p;
            st.p.freq_hz = u32::from_be_bytes([buf[0], buf[1], buf[2], buf[3]]);
            if !rebuild(radio, st, mdltn, rx_pkt) {
                st.p = prev; // rejected: report what the radio is really on, not what was asked
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
        // The SX1262's CAD detector is configured with SetCadParams(0x88): symbol count, detPeak,
        // detMin. The SX1276 has NO such registers — its CAD window and detector thresholds are
        // fixed by the modem's own SF-derived settings (RegDetectOptimize/RegDetectionThreshold are
        // demodulator constants lora-phy programs from the SF, not CAD tuning). There is nothing
        // honest to store, so this is UNSUPPORTED rather than a silently-ignored write.
        // `cad_repeat`, the firmware-side half of the same knob, IS supported via CMD_SET_SENSE_CFG.
        CMD_SET_CAD_CFG => send_unsupported(tx, CMD_SET_CAD_CFG, UNSUP_NO_HARDWARE).await,
        CMD_SET_LBT_CFG if f.len >= 4 => {
            st.csma.lbt_cw = u16::from_be_bytes([buf[0], buf[1]]) as u32;
            st.csma.lbt_max_backoff = buf[2];
            st.csma.lbt_max_attempts = buf[3];
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
        // No scheduled-TX engine exists on this radio: the SX1276 has no TSF comparator and no
        // delayed key-up, and the MCU cannot make one honest (an embassy timer fires the SPI write
        // whenever the executor gets round to it, which is exactly the jitter TX_AT exists to
        // remove). Answered explicitly so the reason is NO_HARDWARE, not UNKNOWN_OPCODE; EVT_CAP's
        // bit 24 is clear and sched_gran_ns is 0 for the same reason.
        CMD_TX_AT => send_unsupported(tx, CMD_TX_AT, UNSUP_NO_HARDWARE).await,
        CMD_GET_CAP => send_cap(tx).await,
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
        // GD32-only: that node reboots into its ROM UART bootloader so `stm32flash` can reflash over
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
        _ => {
            let known = f.typ < 32 && (CMD_BITMAP >> f.typ) & 1 != 0;
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
