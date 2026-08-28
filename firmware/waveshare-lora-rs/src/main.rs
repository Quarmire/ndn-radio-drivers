//! Waveshare USB-LoRa (GD32F103C8 + SX1262) — open Rust firmware.
//!
//! Bring-up in stages, each validating a subsystem before the next:
//!   1. blink the RXD LED (PA6) — toolchain + openocd RDP-unlock flash + the chip runs Rust. ✓
//!   2. USART1 (PA9/PA10) → CH343 → host echo — clocks + host serial path. ✓
//!   3. SX1262 over SPI2 — standard LoRa TX/RX, full bidirectional interop with the Heltec node. ✓
//!   4. named-radio serial bridge protocol + full knob exposure — THIS stage.
//!
//! Stage 4 turns the dongle into a standard-LoRa modem the OPi host drives over the CH343. A small
//! binary framing (`7E A5` sync, type, len, payload, XOR-CRC) carries host commands (transmit a
//! frame; set frequency / SF-BW-CR / power / sync word; query info) and firmware events (received
//! frame with RSSI+SNR; TX-done; info; ascii log). No proprietary header — the air side is plain
//! LoRa, so it interoperates with any SX127x/SX126x peer (verified against the Heltec).
//!
//! ## 7E-A5 **v2** (2026-08-28) — this node is the fleet reference
//!
//! The framing is unchanged and every existing opcode keeps its meaning; v2 only ADDS
//! self-description, and makes two silent failures loud:
//!
//!  * **CMD_GET_CAP → EVT_CAP** (`send_cap`): the ONE place this node describes itself — band, real
//!    dBm range, timestamp rate and kind, the true `max_payload`, an exact `cmd_bitmap`, the SF span,
//!    and `sched_gran_ns = 0` because there is no scheduled-TX engine here.
//!  * **CMD_READ_CLOCK → EVT_CLOCK**, the same µs counter EVT_RX stamps with, at full 64-bit width.
//!  * **CMD_SENSE → EVT_SENSE**, free-running channel-busy count + instantaneous RSSI.
//!  * **CMD_TX_AT → EVT_UNSUPPORTED**, explicitly: no scheduled TX on this radio.
//!  * Nothing answers with **silence** any more — an unknown or badly-argued command gets
//!    `EVT_UNSUPPORTED [cmd, reason]` instead of costing the host four retries and a timeout.
//!
//! Two correctness fixes ship with it:
//!  * **C1** — the RX path buffered 64 bytes while CMD_TX accepted 240 and the host face declared an
//!    MTU of 200, so every received frame over 64 B was silently truncated. RX now runs to `RX_MAX`
//!    (247, the serial framing's real ceiling) and counts anything longer.
//!  * **C2** — the on-device NDN data plane recognised only the ASCII demo wire, so every offload
//!    path was INERT on the real face. It now parses NDNLPv2/NDN-TLV (see `ndn.rs`).

#![no_std]
#![no_main]
#![allow(dead_code)]

mod ndn;
mod sx1262;

use core::cell::UnsafeCell;
use core::fmt::Write as FmtWrite;
use core::sync::atomic::{AtomicU32, AtomicUsize, Ordering};
use cortex_m::peripheral::syst::SystClkSource;
use cortex_m_rt::{entry, exception};
use embedded_hal::spi::MODE_0;
use nb::block;
use panic_halt as _;
use stm32f1xx_hal::{
    pac,
    pac::interrupt,
    prelude::*,
    serial::{Config, Serial},
    spi::Spi,
};

use sx1262::Sx1262;

/// Host-byte ring, filled by the USART1 interrupt and drained by the main loop.
///
/// **This is what makes the host link reliable.** The USART has no FIFO and holds exactly one byte:
/// at 115200 a new byte lands every ~87 µs, which is far shorter than a `poll_rx` SPI transaction, a
/// command handler, or a transmission (up to 2 s). Polling `rx.read()` from the main loop therefore
/// *destroys* any command that arrives while the firmware is busy — measured on hardware at ~50% loss
/// for a 37-byte `CMD_TX`. An interrupt is the only way to take the byte within its 87 µs window, so
/// the ISR captures it here and the main loop drains at its leisure.
///
/// Single-producer (ISR) / single-consumer (main loop), so the two indices need no lock: each side
/// only writes its own, and Acquire/Release pairs order the data against them.
const RING_SZ: usize = 512;

struct Ring {
    buf: UnsafeCell<[u8; RING_SZ]>,
    /// Written only by the ISR.
    head: AtomicUsize,
    /// Written only by the main loop.
    tail: AtomicUsize,
    /// Bytes dropped because the ring was full, or overruns the ISR saw — reported via GET_INFO so
    /// a lossy link is visible instead of silent.
    lost: AtomicU32,
}

// SAFETY: the indices are atomic and each side writes only its own; `buf` is only touched at the
// slot the owning side's index points to, which the other side never reads until the index moves.
unsafe impl Sync for Ring {}

impl Ring {
    const fn new() -> Self {
        Self {
            buf: UnsafeCell::new([0; RING_SZ]),
            head: AtomicUsize::new(0),
            tail: AtomicUsize::new(0),
            lost: AtomicU32::new(0),
        }
    }

    /// ISR side: take one byte. Drops (and counts) it if the consumer has fallen behind.
    fn push(&self, b: u8) {
        let h = self.head.load(Ordering::Relaxed);
        let next = (h + 1) % RING_SZ;
        if next == self.tail.load(Ordering::Acquire) {
            self.lost.fetch_add(1, Ordering::Relaxed);
            return;
        }
        unsafe { (*self.buf.get())[h] = b };
        self.head.store(next, Ordering::Release);
    }

    /// Main-loop side: take one byte if the ISR has left us any.
    fn pop(&self) -> Option<u8> {
        let t = self.tail.load(Ordering::Relaxed);
        if t == self.head.load(Ordering::Acquire) {
            return None;
        }
        let b = unsafe { (*self.buf.get())[t] };
        self.tail.store((t + 1) % RING_SZ, Ordering::Release);
        Some(b)
    }

    fn lost(&self) -> u32 {
        self.lost.load(Ordering::Relaxed)
    }
}

static RING: Ring = Ring::new();

/// USART1 RX: take the byte out of the one-byte data register before the next one overwrites it.
///
/// Reading DR *after* SR is also what clears an overrun (ORE). That matters: a latched ORE stops the
/// peripheral delivering anything further, so missing this would take the host link down for good
/// rather than costing a single byte.
/// 1 kHz SysTick → the free-running millisecond clock behind `millis()`/`micros()` (EVT_RX
/// timestamps). Also carries the ms counter's own wrap into `MILLIS_HI`, which is what lets
/// `micros64` (CMD_READ_CLOCK) be a genuine 64-bit monotonic µs clock instead of a u32 that silently
/// restarts every ~71 minutes.
#[exception]
fn SysTick() {
    if MILLIS.fetch_add(1, Ordering::Relaxed) == u32::MAX {
        MILLIS_HI.fetch_add(1, Ordering::Relaxed);
    }
}

#[interrupt]
fn USART1() {
    // SAFETY: after init, this ISR is the only code that touches USART1's SR/DR.
    let usart = unsafe { &*pac::USART1::ptr() };
    let sr = usart.sr.read();
    if sr.ore().bit_is_set() {
        RING.lost.fetch_add(1, Ordering::Relaxed);
    }
    if sr.rxne().bit_is_set() || sr.ore().bit_is_set() {
        let b = usart.dr.read().dr().bits() as u8;
        RING.push(b);
    }
}

// --- Wire protocol tags ---
const SYNC0: u8 = 0x7E;
const SYNC1: u8 = 0xA5;
// Host -> firmware commands.
const CMD_TX: u8 = 0x01; //   payload = LoRa frame bytes
const CMD_SET_FREQ: u8 = 0x02; // payload = u32 BE Hz
const CMD_SET_MOD: u8 = 0x03; //  payload = [sf, bw_code, cr_code]
const CMD_SET_PWR: u8 = 0x04; //  payload = [i8 dBm]
const CMD_SET_SYNC: u8 = 0x05; // payload = [sx127x sync byte]
const CMD_GET_INFO: u8 = 0x06; // payload = []
const CMD_SET_BEACON: u8 = 0x07; // payload = [enabled(0/1)] or [enabled, period_mult]
// #52 additions.
const CMD_CAD: u8 = 0x08; //          payload = []            → EVT_CAD [busy]
const CMD_GET_RSSI: u8 = 0x09; //     payload = []            → EVT_RSSI [rssi i16 BE]
const CMD_SET_CAD_CFG: u8 = 0x0A; //  payload = [sym, det_peak, det_min]
const CMD_SET_LBT_CFG: u8 = 0x0B; //  payload = [cw_ms(2 BE), max_backoff, max_attempts]
const CMD_SET_PREAMBLE: u8 = 0x0C; // payload = [preamble(2 BE)]
const CMD_SF_SCAN: u8 = 0x0D; //      payload = []            → EVT_SF_DETECTED [sf | 0]
const CMD_TX_LBT: u8 = 0x0E; //       payload = LoRa frame bytes; atomic CAD+backoff+key-up
// #52 data-centric offload config (the host installs name-hash routes; all default inert).
const CMD_SET_NAME_FILTER: u8 = 0x0F; // payload = [u64 BE hash]*  (empty clears → pass-all)
const CMD_SET_RELAY: u8 = 0x10; //       payload = [u64 BE hash]*  (relay set; empty clears)
const CMD_DATAPLANE: u8 = 0x11; //       payload = [cs_serve, dedup, hop_on, hop_base_ch, hop_span]
const CMD_SET_SENSE_CFG: u8 = 0x12; //   payload = [rssi_thresh i16 BE, cad_repeat] (energy-detect + N-CAD)
const CMD_GET_STATS: u8 = 0x13; //       payload = []  → EVT_STATS
const CMD_RESET_STATS: u8 = 0x14; //     payload = []  (clear all counters)
const CMD_SET_DEBUG: u8 = 0x15; //       payload = [on] (toggle EVT_LOG diagnostics — no reflash)
const CMD_ENTER_BOOTLOADER: u8 = 0x16; // payload = [0xB0,0x07] guard → jump to the GD32 ROM UART
//                                       bootloader on USART1, so `stm32flash` reflashes over the SAME
//                                       CH343/USB link — no ST-Link, no BOOT0 pin, no replug.
// --- 7E-A5 v2 (fleet-wide self-description; the Waveshare is the reference node) ---
const CMD_READ_CLOCK: u8 = 0x17; //  payload = []  → EVT_CLOCK [ticks u64 BE], units = EVT_CAP.stamp_hz
const CMD_TX_AT: u8 = 0x18; //       payload = [delay_us u32 BE][frame] — NOT IMPLEMENTED on this node;
//                                   answered EVT_UNSUPPORTED (see the explicit arm in `handle_cmd`).
const CMD_GET_CAP: u8 = 0x1A; //     payload = []  → EVT_CAP (29 bytes)
const CMD_SENSE: u8 = 0x1B; //       payload = []  → EVT_SENSE [activity u16 BE, rssi i16 BE]
// --- Waveshare-local extension. Outside the v2 block (0x17..0x1B) so it cannot collide with a future
// fleet assignment there; the host discovers it from EVT_CAP's cmd_bitmap, which is what the bitmap
// is for.
const CMD_SET_RX_GAIN: u8 = 0x1C; // payload = [0 = power-saving | 1 = boosted] → EVT_INFO
// Firmware -> host events.
const EVT_RX: u8 = 0x81; //    payload = [rssi i16 BE, snr i16 BE, ts_us u32 BE, LoRa bytes]
//                             ts_us is MICROseconds (`micros()`), not ms — the field was mislabelled
//                             `ts_ms` here and on the host. EVT_CAP.stamp_hz states the true rate.
const EVT_TXDONE: u8 = 0x82; //payload = [ok, attempts]  (attempts=0 for a plain CMD_TX)
const EVT_INFO: u8 = 0x83; //  payload = [status, sync(2), errors(2), freq(4), sf, bw, cr, pwr, lost(2), cad_busy(2), defer(2)]
//                             19 bytes, FIXED: the host reads cad_busy/defer as the LAST 4 bytes, so
//                             nothing may ever be appended here. New counters go in EVT_STATS.
const EVT_LOG: u8 = 0x84; //   payload = ascii
const EVT_CAD: u8 = 0x85; //   payload = [busy(0/1)]
const EVT_RSSI: u8 = 0x86; //  payload = [rssi i16 BE]
const EVT_SF_DETECTED: u8 = 0x87; // payload = [sf | 0 = none]
const EVT_TX_STARTED: u8 = 0x88; //  payload = [airtime_ms u16 BE] — emitted just before key-up
const EVT_STATS: u8 = 0x89; //       payload = [rx(4), filtered(4), deduped(4), served(4), relayed(4),
//                                   cad_busy(2), defer(2)  <- v1 ends at 24 B; a v1 host stops here
//                                   chip_rx(2), chip_crc_err(2), chip_hdr_err(2), rx_trunc(2)] = 32 B
const EVT_CLOCK: u8 = 0x8A; //       payload = [ticks u64 BE] (µs; see EVT_CAP.stamp_hz)
const EVT_CAP: u8 = 0x8B; //         payload = 29 bytes, all multi-byte fields BIG-ENDIAN (see `send_cap`)
const EVT_SENSE: u8 = 0x8C; //       payload = [activity u16 BE, rssi i16 BE]
const EVT_UNSUPPORTED: u8 = 0x8F; // payload = [cmd, reason] — never silence, never a fake success

// EVT_UNSUPPORTED reason codes.
const UNSUP_UNKNOWN_OPCODE: u8 = 0x01; // this firmware does not know the opcode at all
const UNSUP_NO_HARDWARE: u8 = 0x02; //   opcode understood, but this radio/firmware has no such engine
const UNSUP_BAD_LENGTH: u8 = 0x03; //    opcode understood, payload does not satisfy its argument
//                                       requirements (too short, or a guard magic wrong)
const UNSUP_OUT_OF_RANGE: u8 = 0x04; //  argument outside the range EVT_CAP advertises

/// **The self-description bitmap** (EVT_CAP `cmd_bitmap`): bit N set ⇔ opcode N is implemented and
/// will act. The host uses it to decide what it may send, so it must be EXACT — and it is also the
/// firmware's own "is this a known opcode?" oracle in `handle_cmd`, so the bitmap and the dispatcher
/// physically cannot drift apart.
///
/// Set: 0x01..=0x17 (every command from CMD_TX through CMD_READ_CLOCK) = bits 1..23 → `0x00FF_FFFE`
///      0x1A CMD_GET_CAP  → bit 26 → `0x0400_0000`
///      0x1B CMD_SENSE    → bit 27 → `0x0800_0000`
///      0x1C CMD_SET_RX_GAIN → bit 28 → `0x1000_0000`
/// Clear: bit 0 (no opcode 0), bit 24 (0x18 CMD_TX_AT — no scheduled-TX engine; answered
///        EVT_UNSUPPORTED), bit 25 (0x19 unassigned), bits 29..31 (unassigned).
/// Total = 0x1CFF_FFFE.
const CMD_BITMAP: u32 = 0x1CFF_FFFE;

/// Largest LoRa frame this firmware will receive and report — the REAL end-to-end cap, and what
/// EVT_CAP advertises as `max_payload`.
///
/// It is set by the SERIAL FRAMING, not the radio: an event's `len` field is one byte, so an
/// EVT_RX payload can be at most 255 B, of which 8 are the rssi/snr/timestamp header → 247 B of
/// frame. The SX1262 FIFO (256 B) and the LoRa PDU (255 B) are both larger, and `CMD_TX` accepts up
/// to 255 B, so 247 is the binding constraint and therefore the honest number to report.
///
/// (Before 2026-08-28 this was 64 while TX accepted 240: every received frame over 64 B was
/// silently truncated — bug C1.
///
/// **RAM budget worked to**, all measured on the built image, not estimated: the linker gives 20 KB
/// − 8 B (the boot-flag slot) = 20 472 B. `.bss` is 536 B (the 512 B host ring + its indices);
/// everything else is stack, and `main`'s frame — which holds `rxbuf`, the EVT_RX scratch, the
/// parser, the CS-serve buffer and the whole `DataPlane` — measures 0xcb0 = 3 248 B, with no callee
/// frame above 0x2c. Peak ≈ 3.8 KB of 20.4 KB, so ~16.5 KB spare. That is what paid for `rxbuf`
/// 64→247, the EVT_RX scratch 72→255, and `ndn::CS_MAX_LEN` 96→192.)
const RX_MAX: usize = 247;

/// Runtime diagnostics toggle (CMD_SET_DEBUG) — emit EVT_LOG traces of data-plane decisions on demand,
/// no reflash. Off by default (quiet link).
static DEBUG: core::sync::atomic::AtomicBool = core::sync::atomic::AtomicBool::new(false);
fn debug_on() -> bool {
    DEBUG.load(Ordering::Relaxed)
}

/// #52 runtime state for carrier-sense/LBT (bundled so the command handler stays legible). Every
/// field is host-tunable so tuning needs a serial command, not a reflash.
struct Csma {
    /// LoRa preamble length (symbols) — mirrors the value pushed into the radio.
    preamble: u16,
    /// CAD config: cadSymbolNum code (0..4 → 1/2/4/8/16 syms), detector peak/min.
    cad_sym: u8,
    cad_peak: u8,
    cad_min: u8,
    /// LBT: contention window (ms), max backoff exponent, max attempts before DEFERRED.
    lbt_cw: u32,
    lbt_max_backoff: u8,
    lbt_max_attempts: u8,
    /// Backoff PRNG (xorshift32), seeded once from the SX1262 hardware RNG.
    rng: u32,
    /// Channel-busy observations — **free-running and wrapping, never cleared**, because EVT_SENSE
    /// (`activity`) is defined as a free-running counter the host differences over a window. One
    /// increment site, one piece of state.
    cad_busy: u16,
    /// Baseline subtracted for the *resettable* EVT_INFO / EVT_STATS view. CMD_RESET_STATS moves this
    /// forward instead of zeroing `cad_busy`, so re-baselining the stats view cannot make EVT_SENSE
    /// appear to run backwards.
    cad_busy_base: u16,
    defer: u16,
    /// Energy-detect threshold (dBm) OR'd into the busy sense; `i16::MIN` disables it. Catches
    /// non-LoRa interference. Runtime-tunable (CMD_SET_SENSE_CFG).
    rssi_thresh: i16,
    /// CAD samples per sense (OR'd) — more cuts false-negatives at the cost of airtime. Default 1.
    cad_repeat: u8,
}
impl Csma {
    fn new() -> Self {
        Self {
            preamble: 8,
            cad_sym: 0x02,      // 4 symbols
            cad_peak: 0x18,     // 24 — a mid default; tune on air per SF
            cad_min: 0x0A,      // 10
            lbt_cw: 20, // ms — the initial-backoff window must be ~a CAD slot (SF10 CAD ≈ 33 ms) to
            lbt_max_backoff: 4, // separate two nodes; runtime-tunable via CMD_SET_LBT_CFG, no reflash
            lbt_max_attempts: 6,
            rng: 0x1234_5678,
            cad_busy: 0,
            cad_busy_base: 0,
            defer: 0,
            rssi_thresh: i16::MIN, // energy-detect disabled by default (CAD only)
            cad_repeat: 1,
        }
    }
    /// The resettable CAD-busy view reported by EVT_INFO / EVT_STATS.
    fn cad_busy_view(&self) -> u16 {
        self.cad_busy.wrapping_sub(self.cad_busy_base)
    }
    fn next_rand(&mut self) -> u32 {
        let mut x = self.rng;
        x ^= x << 13;
        x ^= x >> 17;
        x ^= x << 5;
        self.rng = x;
        x
    }
}

/// Free-running millisecond clock (SysTick ISR), and its wrap count.
static MILLIS: AtomicU32 = AtomicU32::new(0);
static MILLIS_HI: AtomicU32 = AtomicU32::new(0);
fn millis() -> u32 {
    MILLIS.load(Ordering::Relaxed)
}

/// SysTick reload: 8000 cycles = 1 ms at the 8 MHz HSI core clock, so the down-counter resolves
/// 1/8000 ms = 0.125 µs and the value below is in whole MICROseconds.
const SYSTICK_RELOAD: u32 = 8_000 - 1;
/// **`stamp_hz` = 1_000_000.** This is the unit of the EVT_RX timestamp and of EVT_CLOCK, and it is
/// microseconds, not milliseconds: `micros64` adds `RELOAD - CVR` SysTick cycles / 8 to `ms * 1000`.
/// The old `ts_ms` label on EVT_RX was simply wrong.
const STAMP_HZ: u32 = 1_000_000;

/// 64-bit monotonic microsecond clock — the counter EVT_RX stamps with and CMD_READ_CLOCK returns
/// (#41 common-view timing needs sub-ms). Combines the ms tick (plus its wrap count) with the SysTick
/// down-counter. Retries if a tick lands mid-read.
///
/// This is a SOFTWARE counter (EVT_CAP `stamp_kind = 2`): the MCU reads it when it notices the
/// SX1262's RxDone IRQ in the poll loop, not a hardware capture at the air interface. Its resolution
/// is ~1 µs but its ACCURACY against the air is bounded by the poll interval, so do not read it as an
/// LR2021-style hardware RX stamp.
fn micros64() -> u64 {
    const SYST_CVR: *const u32 = 0xE000_E018 as *const u32; // SysTick current-value register
    loop {
        let hi1 = MILLIS_HI.load(Ordering::Relaxed);
        let ms = MILLIS.load(Ordering::Relaxed);
        let cvr = unsafe { core::ptr::read_volatile(SYST_CVR) } & 0x00FF_FFFF;
        if MILLIS.load(Ordering::Relaxed) == ms && MILLIS_HI.load(Ordering::Relaxed) == hi1 {
            let ms64 = ((hi1 as u64) << 32) | ms as u64;
            return ms64 * 1000 + (SYSTICK_RELOAD.saturating_sub(cvr) / 8) as u64;
        }
    }
}

/// The low 32 bits of [`micros64`] — the EVT_RX `ts_us` field. Exactly the truncation of the 64-bit
/// clock (`(hi << 32) * 1000` is a multiple of 2^32), so the two never disagree; it wraps every
/// ~71 min, which is fine for relative timing and is why CMD_READ_CLOCK returns the full 64 bits.
fn micros() -> u32 {
    micros64() as u32
}

// Heartbeat-beacon base period in main-loop iterations (~seconds; the loop is SPI-poll bound).
// The beacon is runtime-toggleable via CMD_SET_BEACON and defaults OFF, so a fresh/reset dongle stays
// quiet; enable on-air discovery explicitly with CMD_SET_BEACON[1].
const BEACON_BASE_PERIOD: u32 = 250_000;

/// Formats into a fixed stack buffer so we can build payloads/logs with `write!`.
struct BufWriter {
    buf: [u8; 64],
    pos: usize,
}
impl BufWriter {
    fn new() -> Self {
        Self {
            buf: [0; 64],
            pos: 0,
        }
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

/// Incremental parser for the `7E A5 | type | len | payload | crc` framing. Resyncs on the sync
/// bytes and validates the XOR checksum, so a dropped byte costs at most one frame.
struct Parser {
    state: u8,
    typ: u8,
    len: u8,
    idx: usize,
    crc: u8,
    buf: [u8; 255],
}
impl Parser {
    fn new() -> Self {
        Self {
            state: 0,
            typ: 0,
            len: 0,
            idx: 0,
            crc: 0,
            buf: [0; 255],
        }
    }
    /// Feed one byte; returns `Some((type, payload_len))` when a valid frame completes.
    fn push(&mut self, b: u8) -> Option<(u8, usize)> {
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
                };
            }
            2 => {
                self.typ = b;
                self.crc = b;
                self.state = 3;
            }
            3 => {
                self.len = b;
                self.crc ^= b;
                self.idx = 0;
                self.state = if b == 0 { 5 } else { 4 };
            }
            4 => {
                if self.idx < self.buf.len() {
                    self.buf[self.idx] = b;
                }
                self.crc ^= b;
                self.idx += 1;
                if self.idx >= self.len as usize {
                    self.state = 5;
                }
            }
            5 => {
                self.state = 0;
                if b == self.crc {
                    return Some((self.typ, self.len as usize));
                }
            }
            _ => self.state = 0,
        }
        None
    }
}

/// Frame one event onto the wire via a byte sink.
fn send_frame<F: FnMut(u8)>(mut out: F, typ: u8, payload: &[u8]) {
    out(SYNC0);
    out(SYNC1);
    out(typ);
    out(payload.len() as u8);
    let mut crc = typ ^ (payload.len() as u8);
    for &b in payload {
        out(b);
        crc ^= b;
    }
    out(crc);
}

#[entry]
fn main() -> ! {
    let dp = pac::Peripherals::take().unwrap();
    let mut cp = cortex_m::Peripherals::take().unwrap();

    let mut flash = dp.FLASH.constrain();
    let rcc = dp.RCC.constrain();
    let clocks = rcc.cfgr.freeze(&mut flash.acr);

    // 1 kHz SysTick for the millisecond clock (EVT_RX timestamps). Core clock is 8 MHz HSI (the same
    // assumption sx1262.rs makes for its busy-wait delays), so reload = 8000 - 1.
    cp.SYST.set_clock_source(SystClkSource::Core);
    cp.SYST.set_reload(8_000 - 1);
    cp.SYST.clear_current();
    cp.SYST.enable_counter();
    cp.SYST.enable_interrupt();

    let mut afio = dp.AFIO.constrain();
    let mut gpioa = dp.GPIOA.split();
    let mut gpiob = dp.GPIOB.split();

    // PB4 (RF switch) is JNTRST — free it by disabling JTAG while keeping SWD (PA13/PA14) for reflash.
    let (_pa15, _pb3, pb4) = afio.mapr.disable_jtag(gpioa.pa15, gpiob.pb3, gpiob.pb4);

    // USART1: TX=PA9, RX=PA10, 115200 → CH343 → host.
    let utx = gpioa.pa9.into_alternate_push_pull(&mut gpioa.crh);
    let urx = gpioa.pa10;
    let serial = Serial::new(
        dp.USART1,
        (utx, urx),
        &mut afio.mapr,
        Config::default().baudrate(115_200.bps()),
        &clocks,
    );
    let (mut tx, mut rx) = serial.split();
    // Take host bytes in an ISR, not from the main loop — see [`Ring`]. From here on nothing calls
    // `rx.read()`; the ISR owns the data register and the main loop drains RING.
    rx.listen();
    unsafe { pac::NVIC::unmask(pac::Interrupt::USART1) };

    // SPI2: SCK=PB13, MISO=PB14, MOSI=PB15; NSS=PB12 by hand.
    let sck = gpiob.pb13.into_alternate_push_pull(&mut gpiob.crh);
    let miso = gpiob.pb14;
    let mosi = gpiob.pb15.into_alternate_push_pull(&mut gpiob.crh);
    let spi = Spi::spi2(dp.SPI2, (sck, miso, mosi), MODE_0, 1.MHz(), clocks);
    let nss = gpiob.pb12.into_push_pull_output(&mut gpiob.crh);

    // SX1262 control lines.
    let rst = gpioa.pa4.into_push_pull_output(&mut gpioa.crl);
    let busy = gpiob.pb1.into_floating_input(&mut gpiob.crl);
    let dio1 = gpiob.pb0.into_floating_input(&mut gpiob.crl);
    let rfsw = pb4.into_push_pull_output(&mut gpiob.crl);

    let mut radio = Sx1262::new(spi, nss, rst, busy, dio1, rfsw);

    // Current knob state (mirrors the chip so GET_INFO can report it).
    let mut freq: u32 = 915_000_000;
    let mut sf: u8 = 7;
    let mut bw: u8 = sx1262::BW_125;
    let mut cr: u8 = sx1262::CR_4_5;
    let mut pwr: i8 = 22;

    let diag = radio.init(freq, sf, bw, cr);
    // #52 CSMA state; seed the backoff PRNG from the SX1262 hardware RNG (leaves the chip in standby).
    let mut csma = Csma::new();
    let seed = radio.hw_random();
    if seed != 0 {
        csma.rng = seed;
    }
    // #52 data-centric offload: name-hash filter / Content Store / relay / hop. All inert until the
    // host installs routes, so a fresh dongle behaves exactly like the plain modem.
    let mut plane = ndn::DataPlane::new();
    radio.start_rx();

    // Announce readiness (ascii log the host can print, plus a structured INFO).
    {
        let mut log = BufWriter::new();
        let _ = write!(
            log,
            "waveshare-lora-rs stage4: init sync=0x{:04X} err=0x{:04X}",
            diag.sync_readback, diag.device_errors
        );
        send_frame(
            |b| {
                let _ = block!(tx.write(b));
            },
            EVT_LOG,
            log.as_slice(),
        );
    }

    let mut parser = Parser::new();
    // C1: sized to RX_MAX (247), the real end-to-end cap. This was 64 while CMD_TX accepted 240 and
    // the host face declared an MTU of 200 — every received frame over 64 B was silently truncated,
    // so any real NDN packet was corrupted on RX with nothing reporting it.
    let mut rxbuf = [0u8; RX_MAX];
    // Frames the radio reported LONGER than RX_MAX (so still truncated). Should stay 0 — reported in
    // EVT_STATS so a peer transmitting past our advertised max_payload is visible instead of silent.
    let mut rx_trunc: u16 = 0;
    // Default OFF: a fresh/reset dongle stays quiet (no stray beacon before a host attaches). Opt in
    // on-air discovery with CMD_SET_BEACON[1] (or the host's LoraParams.beacon = true).
    let mut beacon_enabled = false;
    let mut beacon_period = BEACON_BASE_PERIOD;
    let mut beacon_ctr: u32 = 0;
    let mut beacon_seq: u32 = 0;

    loop {
        // 1) Drain host bytes the ISR has buffered. Nothing is lost while we are busy below.
        while let Some(b) = RING.pop() {
            if let Some((typ, len)) = parser.push(b) {
                handle_cmd(
                    typ,
                    len,
                    &parser.buf,
                    &mut radio,
                    &mut tx,
                    &mut freq,
                    &mut sf,
                    &mut bw,
                    &mut cr,
                    &mut pwr,
                    &mut beacon_enabled,
                    &mut beacon_period,
                    &mut csma,
                    &mut plane,
                    &mut rx_trunc,
                );
            }
        }

        // 2) Classify a received frame by NAME (data-centric offload). With no host-installed filter /
        //    CS / relay it always Delivers — identical to the plain modem; features light up on opt-in.
        if let Some(pkt) = radio.poll_rx(&mut rxbuf) {
            let ts = micros();
            let n = core::cmp::min(pkt.len as usize, rxbuf.len());
            // `pkt.len` is the TRUE on-air length. With rxbuf at RX_MAX this can only trip if a peer
            // ignores our advertised max_payload; count it rather than corrupt the frame in silence.
            if pkt.len as usize > rxbuf.len() {
                rx_trunc = rx_trunc.wrapping_add(1);
            }
            // Copy any CS-serve payload out so we don't hold the data-plane borrow across the TX below.
            let mut serve = [0u8; ndn::CS_MAX_LEN];
            let mut serve_len = 0usize;
            let (deliver, relay) = match plane.on_rx(&rxbuf[..n], millis()) {
                ndn::RxAction::Drop => (false, false),
                ndn::RxAction::Serve(data) => {
                    let m = data.len().min(serve.len());
                    serve[..m].copy_from_slice(&data[..m]);
                    serve_len = m;
                    (false, false)
                }
                ndn::RxAction::Deliver => (true, false),
                ndn::RxAction::RelayAndDeliver => (true, true),
            };
            if debug_on() {
                let mut lg = BufWriter::new();
                let _ = write!(
                    lg,
                    "rx n={n} serve={} relay={relay} deliver={deliver}",
                    serve_len > 0
                );
                send_frame(
                    |b| {
                        let _ = block!(tx.write(b));
                    },
                    EVT_LOG,
                    lg.as_slice(),
                );
            }
            // Content-Store hit: serve the cached Data ourselves (LBT), the host never wakes.
            if serve_len > 0 {
                let _ = lbt_tx(&mut radio, &mut csma, &serve[..serve_len]);
                radio.start_rx();
            }
            // Relay: re-broadcast (LBT) for cooperative forwarding, then also deliver.
            if relay {
                let _ = lbt_tx(&mut radio, &mut csma, &rxbuf[..n]);
                radio.start_rx();
            }
            if deliver {
                // 8 header bytes + up to RX_MAX frame bytes = 255, the largest payload the one-byte
                // `len` field of the 7E-A5 framing can carry. That is what sets RX_MAX.
                let mut ev = [0u8; 8 + RX_MAX];
                ev[0..2].copy_from_slice(&pkt.rssi_dbm.to_be_bytes());
                ev[2..4].copy_from_slice(&pkt.snr_db.to_be_bytes());
                ev[4..8].copy_from_slice(&ts.to_be_bytes());
                ev[8..8 + n].copy_from_slice(&rxbuf[..n]);
                send_frame(
                    |b| {
                        let _ = block!(tx.write(b));
                    },
                    EVT_RX,
                    &ev[..8 + n],
                );
            }
        }

        // 3) Optional heartbeat beacon (host-toggleable via CMD_SET_BEACON) so TX is exercised and
        //    the node is discoverable on-air without a host driving it.
        if beacon_enabled {
            beacon_ctr += 1;
            if beacon_ctr >= beacon_period {
                beacon_ctr = 0;
                let mut msg = BufWriter::new();
                let _ = write!(msg, "LORA-BEACON seq={}", beacon_seq);
                let ok = radio.transmit(msg.as_slice());
                radio.start_rx();
                send_frame(
                    |b| {
                        let _ = block!(tx.write(b));
                    },
                    EVT_TXDONE,
                    &[ok as u8],
                );
                beacon_seq = beacon_seq.wrapping_add(1);
            }
        } else {
            beacon_ctr = 0;
        }
    }
}

/// **One channel-busy observation**, and the SINGLE site that advances [`Csma::cad_busy`].
///
/// Sense = `cad_repeat` CADs OR'd, then the RSSI energy-detect threshold (which catches non-LoRa
/// interference a LoRa CAD is blind to). The caller must already have put the chip in standby and
/// programmed the CAD parameters.
///
/// ⚠ **Why every sensing path routes through here.** `EVT_SENSE.activity` is contracted as a
/// free-running count of channel-busy observations that the host differences over a window. It used
/// to be incremented only inside the LBT backoff loop, so a node that was not transmitting never
/// advanced it — a host polling `CMD_SENSE` on an idle dongle read a *saturated* channel as
/// permanently free, silently, and the occupancy sampler it feeds would have believed that. The
/// counter can only be honest if it counts the observations the host asks for as well as the ones
/// the transmit path makes for itself.
fn sense_busy<SPI, NSS, RST, BSY, DIO1, RFSW, E>(
    radio: &mut Sx1262<SPI, NSS, RST, BSY, DIO1, RFSW>,
    csma: &mut Csma,
) -> bool
where
    SPI: embedded_hal::blocking::spi::Transfer<u8, Error = E>
        + embedded_hal::blocking::spi::Write<u8, Error = E>,
    NSS: embedded_hal::digital::v2::OutputPin,
    RST: embedded_hal::digital::v2::OutputPin,
    RFSW: embedded_hal::digital::v2::OutputPin,
    BSY: embedded_hal::digital::v2::InputPin,
    DIO1: embedded_hal::digital::v2::InputPin,
{
    let mut busy = false;
    for _ in 0..csma.cad_repeat.max(1) {
        if radio.do_cad() {
            busy = true;
            break;
        }
    }
    if !busy && csma.rssi_thresh > i16::MIN {
        busy = radio.rssi_busy(csma.rssi_thresh);
    }
    if busy {
        csma.cad_busy = csma.cad_busy.wrapping_add(1);
    }
    busy
}

/// The atomic listen-before-talk TX loop (#52): a random backoff BEFORE each CAD (CSMA/CA, so no node
/// deterministically captures the channel), transmit on a clear sense, give up after
/// `lbt_max_attempts`. Shared by `CMD_TX_LBT` and the firmware-served Content-Store / relay paths.
/// Leaves the chip in standby (the caller re-arms RX). Returns `(sent, attempts)`.
fn lbt_tx<SPI, NSS, RST, BSY, DIO1, RFSW, E>(
    radio: &mut Sx1262<SPI, NSS, RST, BSY, DIO1, RFSW>,
    csma: &mut Csma,
    payload: &[u8],
) -> (bool, u8)
where
    SPI: embedded_hal::blocking::spi::Transfer<u8, Error = E>
        + embedded_hal::blocking::spi::Write<u8, Error = E>,
    NSS: embedded_hal::digital::v2::OutputPin,
    RST: embedded_hal::digital::v2::OutputPin,
    RFSW: embedded_hal::digital::v2::OutputPin,
    BSY: embedded_hal::digital::v2::InputPin,
    DIO1: embedded_hal::digital::v2::InputPin,
{
    radio.standby();
    radio.set_cad_params(csma.cad_sym, csma.cad_peak, csma.cad_min);
    let mut attempt: u8 = 0;
    let mut sent = false;
    while attempt < csma.lbt_max_attempts {
        let shift = (attempt as u32).min(csma.lbt_max_backoff as u32);
        let window = (csma.lbt_cw << shift).max(1);
        let wait_ms = csma.next_rand() % window;
        cortex_m::asm::delay(wait_ms.saturating_mul(8_000)); // 8 MHz core → 8000 cycles/ms
        if !sense_busy(radio, csma) {
            sent = radio.transmit(payload);
            break;
        }
        attempt += 1;
    }
    if !sent {
        csma.defer = csma.defer.wrapping_add(1);
    }
    (sent, attempt)
}

/// Apply a decoded host command and emit its acknowledging event.
#[allow(clippy::too_many_arguments)]
fn handle_cmd<SPI, NSS, RST, BSY, DIO1, RFSW, E, TX>(
    typ: u8,
    len: usize,
    buf: &[u8; 255],
    radio: &mut Sx1262<SPI, NSS, RST, BSY, DIO1, RFSW>,
    tx: &mut TX,
    freq: &mut u32,
    sf: &mut u8,
    bw: &mut u8,
    cr: &mut u8,
    pwr: &mut i8,
    beacon_enabled: &mut bool,
    beacon_period: &mut u32,
    csma: &mut Csma,
    plane: &mut ndn::DataPlane,
    rx_trunc: &mut u16,
) where
    SPI: embedded_hal::blocking::spi::Transfer<u8, Error = E>
        + embedded_hal::blocking::spi::Write<u8, Error = E>,
    NSS: embedded_hal::digital::v2::OutputPin,
    RST: embedded_hal::digital::v2::OutputPin,
    RFSW: embedded_hal::digital::v2::OutputPin,
    BSY: embedded_hal::digital::v2::InputPin,
    DIO1: embedded_hal::digital::v2::InputPin,
    TX: embedded_hal::serial::Write<u8>,
{
    let mut put = |b: u8| {
        let _ = block!(tx.write(b));
    };
    let lost = RING.lost().min(u16::MAX as u32) as u16;
    match typ {
        CMD_TX => {
            let ok = radio.transmit(&buf[..len]);
            radio.start_rx();
            send_frame(&mut put, EVT_TXDONE, &[ok as u8, 0]);
        }
        // #52: atomic listen-before-talk. CAD → HW-RNG backoff → key-up, all on the MCU so no serial
        // round-trip sits inside the sense-then-transmit window. Replies [sent, attempts].
        CMD_TX_LBT => {
            let air = sx1262::airtime_ms(*sf, *bw, *cr, len as u8, csma.preamble);
            let air16 = (air.min(u16::MAX as u32) as u16).to_be_bytes();
            send_frame(&mut put, EVT_TX_STARTED, &air16);
            let (sent, attempt) = lbt_tx(radio, csma, &buf[..len]);
            radio.start_rx();
            send_frame(&mut put, EVT_TXDONE, &[sent as u8, attempt]);
        }
        // #52: one sense at the current modulation → busy/clear (sensing, not the access loop).
        // Routed through `sense_busy` so it counts into `activity` like every other observation, and
        // so a host-driven CAD honours `cad_repeat`/`rssi_thresh` exactly as the LBT loop does.
        CMD_CAD => {
            radio.standby();
            radio.set_cad_params(csma.cad_sym, csma.cad_peak, csma.cad_min);
            let busy = sense_busy(radio, csma);
            radio.start_rx();
            send_frame(&mut put, EVT_CAD, &[busy as u8]);
        }
        // #52: instantaneous channel RSSI (RX stays armed).
        CMD_GET_RSSI => {
            let r = radio.rssi_inst();
            send_frame(&mut put, EVT_RSSI, &r.to_be_bytes());
        }
        // #52: sweep SF7..12 by CAD, report whichever a transmitter is actually using (ASFS primitive).
        CMD_SF_SCAN => {
            radio.standby();
            radio.set_cad_params(csma.cad_sym, csma.cad_peak, csma.cad_min);
            let mut found = 0u8;
            let mut s = 7u8;
            while s <= 12 {
                radio.set_modulation(s, *bw, *cr);
                if radio.do_cad() {
                    found = s;
                    break;
                }
                s += 1;
            }
            radio.set_modulation(*sf, *bw, *cr); // restore the operating SF
            radio.start_rx();
            send_frame(&mut put, EVT_SF_DETECTED, &[found]);
        }
        // The band is enforced, not merely advertised: `set_frequency` hard-codes the 902-928 MHz
        // image calibration, so tuning outside it yields a mis-calibrated receiver. Refusing keeps
        // EVT_CAP's freq_min/freq_max a TRUE statement and tells the host why, where silently
        // accepting would have left it believing a carrier the radio cannot properly hear.
        CMD_SET_FREQ if len >= 4 => {
            let want = u32::from_be_bytes([buf[0], buf[1], buf[2], buf[3]]);
            if !(sx1262::FREQ_MIN_HZ..=sx1262::FREQ_MAX_HZ).contains(&want) {
                send_frame(
                    &mut put,
                    EVT_UNSUPPORTED,
                    &[CMD_SET_FREQ, UNSUP_OUT_OF_RANGE],
                );
                return;
            }
            *freq = want;
            radio.standby();
            radio.set_frequency(*freq);
            radio.start_rx();
            send_info(
                &mut put,
                radio,
                *freq,
                *sf,
                *bw,
                *cr,
                *pwr,
                lost,
                csma.cad_busy_view(),
                csma.defer,
            );
        }
        // Same contract as the frequency: SF is refused outside the advertised sf_min..sf_max rather
        // than applied, so EVT_CAP's span stays true. (SF5/SF6 are chip-supported but need different
        // sync-word handling and do not interop with the SX127x peers in this fleet.)
        CMD_SET_MOD if len >= 3 => {
            if !(sx1262::SF_MIN..=sx1262::SF_MAX).contains(&buf[0]) {
                send_frame(
                    &mut put,
                    EVT_UNSUPPORTED,
                    &[CMD_SET_MOD, UNSUP_OUT_OF_RANGE],
                );
                return;
            }
            *sf = buf[0];
            *bw = buf[1];
            *cr = buf[2];
            radio.standby();
            radio.set_modulation(*sf, *bw, *cr);
            radio.start_rx();
            send_info(
                &mut put,
                radio,
                *freq,
                *sf,
                *bw,
                *cr,
                *pwr,
                lost,
                csma.cad_busy_view(),
                csma.defer,
            );
        }
        CMD_SET_PWR if len >= 1 => {
            radio.standby();
            // `set_power` clamps to the SX1262's real SetTxParams range and returns what it applied,
            // so the mirror EVT_INFO reports (and EVT_CAP's advertised range) cannot lie.
            *pwr = radio.set_power(buf[0] as i8);
            radio.start_rx();
            send_info(
                &mut put,
                radio,
                *freq,
                *sf,
                *bw,
                *cr,
                *pwr,
                lost,
                csma.cad_busy_view(),
                csma.defer,
            );
        }
        CMD_SET_SYNC if len >= 1 => {
            radio.standby();
            radio.set_sync(buf[0]);
            radio.start_rx();
            send_info(
                &mut put,
                radio,
                *freq,
                *sf,
                *bw,
                *cr,
                *pwr,
                lost,
                csma.cad_busy_view(),
                csma.defer,
            );
        }
        // #52 Tier 2: tune CAD/LBT/preamble at runtime — so calibration never needs a reflash.
        CMD_SET_CAD_CFG if len >= 3 => {
            csma.cad_sym = buf[0];
            csma.cad_peak = buf[1];
            csma.cad_min = buf[2];
            send_info(
                &mut put,
                radio,
                *freq,
                *sf,
                *bw,
                *cr,
                *pwr,
                lost,
                csma.cad_busy_view(),
                csma.defer,
            );
        }
        CMD_SET_LBT_CFG if len >= 4 => {
            csma.lbt_cw = u16::from_be_bytes([buf[0], buf[1]]) as u32;
            csma.lbt_max_backoff = buf[2];
            csma.lbt_max_attempts = buf[3];
            send_info(
                &mut put,
                radio,
                *freq,
                *sf,
                *bw,
                *cr,
                *pwr,
                lost,
                csma.cad_busy_view(),
                csma.defer,
            );
        }
        CMD_SET_PREAMBLE if len >= 2 => {
            csma.preamble = u16::from_be_bytes([buf[0], buf[1]]).max(1);
            radio.standby();
            radio.set_preamble(csma.preamble);
            radio.start_rx();
            send_info(
                &mut put,
                radio,
                *freq,
                *sf,
                *bw,
                *cr,
                *pwr,
                lost,
                csma.cad_busy_view(),
                csma.defer,
            );
        }
        CMD_SET_BEACON if len >= 1 => {
            *beacon_enabled = buf[0] != 0;
            if len >= 2 {
                // Optional second byte scales the base period (min ×1).
                *beacon_period = BEACON_BASE_PERIOD.saturating_mul(buf[1].max(1) as u32);
            }
            send_info(
                &mut put,
                radio,
                *freq,
                *sf,
                *bw,
                *cr,
                *pwr,
                lost,
                csma.cad_busy_view(),
                csma.defer,
            );
        }
        // #52 data-centric offload config. Filter/relay payloads are a list of u64 BE name-hashes.
        CMD_SET_NAME_FILTER => {
            let mut hashes = [0u64; 24];
            let count = (len / 8).min(hashes.len());
            for (i, h) in hashes[..count].iter_mut().enumerate() {
                let o = i * 8;
                *h = u64::from_be_bytes([
                    buf[o],
                    buf[o + 1],
                    buf[o + 2],
                    buf[o + 3],
                    buf[o + 4],
                    buf[o + 5],
                    buf[o + 6],
                    buf[o + 7],
                ]);
            }
            plane.set_filter(&hashes[..count]);
            send_info(
                &mut put,
                radio,
                *freq,
                *sf,
                *bw,
                *cr,
                *pwr,
                lost,
                csma.cad_busy_view(),
                csma.defer,
            );
        }
        CMD_SET_RELAY => {
            let mut hashes = [0u64; 24];
            let count = (len / 8).min(hashes.len());
            for (i, h) in hashes[..count].iter_mut().enumerate() {
                let o = i * 8;
                *h = u64::from_be_bytes([
                    buf[o],
                    buf[o + 1],
                    buf[o + 2],
                    buf[o + 3],
                    buf[o + 4],
                    buf[o + 5],
                    buf[o + 6],
                    buf[o + 7],
                ]);
            }
            plane.set_relay(&hashes[..count]);
            send_info(
                &mut put,
                radio,
                *freq,
                *sf,
                *bw,
                *cr,
                *pwr,
                lost,
                csma.cad_busy_view(),
                csma.defer,
            );
        }
        CMD_DATAPLANE if len >= 5 => {
            plane.set_cs_serve(buf[0] != 0);
            plane.set_dedup(buf[1] != 0);
            plane.set_hop(buf[2] != 0, buf[3], buf[4]);
            send_info(
                &mut put,
                radio,
                *freq,
                *sf,
                *bw,
                *cr,
                *pwr,
                lost,
                csma.cad_busy_view(),
                csma.defer,
            );
        }
        // #52 tunables + diagnostics (all runtime — no reflash).
        CMD_SET_SENSE_CFG if len >= 3 => {
            csma.rssi_thresh = i16::from_be_bytes([buf[0], buf[1]]);
            csma.cad_repeat = buf[2].max(1);
            send_info(
                &mut put,
                radio,
                *freq,
                *sf,
                *bw,
                *cr,
                *pwr,
                lost,
                csma.cad_busy_view(),
                csma.defer,
            );
        }
        // EVT_STATS v2 = 32 bytes. Bytes 0..24 are byte-identical to v1 and the existing host parser
        // checks `len < 24` (not `!= 24`), so a v1 host reads it unchanged and simply ignores the
        // tail; the host contract gains the extra fields when it is updated to read them.
        CMD_GET_STATS => {
            let chip = radio.get_stats();
            let mut p = [0u8; 32];
            p[0..4].copy_from_slice(&plane.rx.to_be_bytes());
            p[4..8].copy_from_slice(&plane.filtered.to_be_bytes());
            p[8..12].copy_from_slice(&plane.deduped.to_be_bytes());
            p[12..16].copy_from_slice(&plane.served.to_be_bytes());
            p[16..20].copy_from_slice(&plane.relayed.to_be_bytes());
            p[20..22].copy_from_slice(&csma.cad_busy_view().to_be_bytes());
            p[22..24].copy_from_slice(&csma.defer.to_be_bytes());
            // --- v2 tail: the SX126x's OWN counters (P5). A CRC failure used to vanish inside
            // `poll_rx` with nothing counting it, so RX loss was invisible; chip_crc_err is now the
            // difference between "quiet channel" and "channel we are failing to decode".
            p[24..26].copy_from_slice(&chip.pkt_received.to_be_bytes());
            p[26..28].copy_from_slice(&chip.pkt_crc_error.to_be_bytes());
            p[28..30].copy_from_slice(&chip.pkt_header_error.to_be_bytes());
            // Frames whose on-air length exceeded RX_MAX and were therefore truncated. Should stay 0
            // — a non-zero value means a peer is transmitting past our advertised max_payload.
            p[30..32].copy_from_slice(&rx_trunc.to_be_bytes());
            send_frame(&mut put, EVT_STATS, &p);
        }
        CMD_RESET_STATS => {
            plane.reset_stats();
            // Re-baseline the resettable VIEW; `cad_busy` itself keeps free-running for EVT_SENSE.
            csma.cad_busy_base = csma.cad_busy;
            csma.defer = 0;
            *rx_trunc = 0;
            radio.reset_stats(); // zero the chip's packet counters too
            send_info(
                &mut put,
                radio,
                *freq,
                *sf,
                *bw,
                *cr,
                *pwr,
                lost,
                csma.cad_busy_view(),
                csma.defer,
            );
        }
        CMD_SET_DEBUG if len >= 1 => {
            DEBUG.store(buf[0] != 0, Ordering::Relaxed);
            send_info(
                &mut put,
                radio,
                *freq,
                *sf,
                *bw,
                *cr,
                *pwr,
                lost,
                csma.cad_busy_view(),
                csma.defer,
            );
        }
        CMD_GET_INFO => {
            send_info(
                &mut put,
                radio,
                *freq,
                *sf,
                *bw,
                *cr,
                *pwr,
                lost,
                csma.cad_busy_view(),
                csma.defer,
            );
        }
        // Software DFU: arm the boot flag and system-reset. The #[pre_init] hook below re-enters as
        // the ROM UART bootloader from a CLEAN reset state, so `stm32flash` reflashes over the same
        // CH343/USB link — no ST-Link, no BOOT0 jumper, no replug. Guarded by a 2-byte magic so a
        // stray frame can't trigger it. (A direct in-app jump does NOT work on this GD32: the bootloader
        // needs reset-default clocks/peripherals, and the boot ROM bounces back to flash when it sees
        // the app's live state — hence the reset round-trip.)
        CMD_ENTER_BOOTLOADER if len >= 2 && buf[0] == 0xB0 && buf[1] == 0x07 => {
            unsafe { core::ptr::write_volatile(BOOT_FLAG_ADDR as *mut u32, BOOT_MAGIC) };
            cortex_m::asm::dsb();
            cortex_m::peripheral::SCB::sys_reset(); // -> ! ; pre_init handles the rest post-reset
        }
        // 7E-A5 v2 §P2: the SAME counter EVT_RX stamps with, at full 64-bit width (the EVT_RX field
        // is its low 32 bits and wraps every ~71 min). Units are EVT_CAP.stamp_hz = 1 MHz.
        CMD_READ_CLOCK => {
            send_frame(&mut put, EVT_CLOCK, &micros64().to_be_bytes());
        }
        // 7E-A5 v2 §P4: this firmware has NO scheduled-TX engine. The SX1262 can be armed from a
        // DIO/timeout but nothing here implements a delayed key-up, and the MCU has no TX timer, so
        // there is no honest way to serve a `delay_us`. Answer explicitly rather than falling through
        // the catch-all, so the reason is NO_HARDWARE (a real capability statement) and not
        // UNKNOWN_OPCODE (which would suggest a firmware too old to know the opcode). EVT_CAP's
        // cmd_bitmap bit 24 is clear and sched_gran_ns is 0 for the same reason.
        CMD_TX_AT => {
            send_frame(&mut put, EVT_UNSUPPORTED, &[CMD_TX_AT, UNSUP_NO_HARDWARE]);
        }
        // 7E-A5 v2 §P1: the one place this node describes itself.
        CMD_GET_CAP => {
            send_cap(&mut put);
        }
        // 7E-A5 v2 §P3: channel occupancy, reusing state that already exists — `cad_busy` is the
        // free-running CAD-busy counter the LBT loop increments, and the RSSI is the same
        // instantaneous read CMD_GET_RSSI and the energy-detect sense config use. Nothing duplicated.
        CMD_SENSE => {
            // Make an OBSERVATION, then report the counter — do not merely read it. The counter is
            // free-running and the host differences two reads, so a `CMD_SENSE` that sensed nothing
            // would return the same value forever on a node that is not transmitting, and the
            // difference would say "channel free" no matter how busy the air was.
            radio.standby();
            radio.set_cad_params(csma.cad_sym, csma.cad_peak, csma.cad_min);
            sense_busy(radio, csma);
            radio.start_rx();
            // RSSI after RX is re-armed: `rssi_inst` outside an RX mode is not a channel measurement.
            let r = radio.rssi_inst();
            let mut p = [0u8; 4];
            p[0..2].copy_from_slice(&csma.cad_busy.to_be_bytes());
            p[2..4].copy_from_slice(&r.to_be_bytes());
            send_frame(&mut put, EVT_SENSE, &p);
        }
        // Waveshare-local (P6): pick the LNA gain. 1/default = boosted (this firmware's default,
        // ~+3 dB sensitivity), 0 = the chip's power-saving power-on default.
        CMD_SET_RX_GAIN if len >= 1 => {
            radio.standby();
            radio.set_rx_gain(if buf[0] == 0 {
                sx1262::RX_GAIN_POWER_SAVING
            } else {
                sx1262::RX_GAIN_BOOSTED
            });
            radio.start_rx();
            send_info(
                &mut put,
                radio,
                *freq,
                *sf,
                *bw,
                *cr,
                *pwr,
                lost,
                csma.cad_busy_view(),
                csma.defer,
            );
        }
        // NEVER silence (7E-A5 v2 rule): a command that returns nothing costs the host its full retry
        // budget and then a failure, for what is really a one-frame answer. Two distinct reasons, and
        // `CMD_BITMAP` is the oracle for which — so the dispatcher and the bitmap EVT_CAP advertises
        // cannot drift apart:
        //   * the opcode is in the bitmap ⇒ we implement it, so we only got here because a length
        //     guard above rejected the payload  → BAD_LENGTH;
        //   * it is not                        → UNKNOWN_OPCODE.
        _ => {
            let known = typ < 32 && (CMD_BITMAP >> typ) & 1 != 0;
            let reason = if known {
                UNSUP_BAD_LENGTH
            } else {
                UNSUP_UNKNOWN_OPCODE
            };
            send_frame(&mut put, EVT_UNSUPPORTED, &[typ, reason]);
        }
    }
}

/// **EVT_CAP — the one place this node describes itself** (7E-A5 v2). 29 bytes, every multi-byte
/// field big-endian. Every value below comes from a source constant or a verified property of this
/// firmware; where nothing is known the field is 0 and says so, because a fabricated number is worse
/// than 0 — the host believes it.
///
/// ```text
///  [0]      proto_ver     = 2
///  [1]      radio_kind    = 0 (SX1262)
///  [2..6]   freq_min_hz   = 902_000_000   } the band this firmware image-calibrates for; NOT the
///  [6..10]  freq_max_hz   = 928_000_000   } SX1262's 150-960 MHz silicon range (sx1262::FREQ_*_HZ)
///  [10]     pwr_min_dbm   = -9            } real dBm, the SetTxParams range `set_power` clamps to
///  [11]     pwr_max_dbm   = 22            } (sx1262::PWR_*_DBM) — not a chip register unit
///  [12..16] stamp_hz      = 1_000_000     MICROseconds; see STAMP_HZ / `micros64`
///  [16]     stamp_kind    = 2 (software counter — the MCU reads its own clock when it notices
///                              RxDone in the poll loop; there is NO hardware capture on this radio)
///  [17..19] max_payload   = 247           RX_MAX — the binding serial-framing limit, below the
///                                         255 B CMD_TX accept and the 255 B LoRa PDU
///  [19..23] cmd_bitmap    = CMD_BITMAP
///  [23]     sf_min        = 7             } sx1262::SF_MIN/SF_MAX — the SFs this firmware operates
///  [24]     sf_max        = 12            } and CMD_SF_SCAN sweeps
///  [25..29] sched_gran_ns = 0             this firmware exposes NO scheduled-TX engine (see
///                                         CMD_TX_AT above); 0 is the truth, not a placeholder
/// ```
fn send_cap<F: FnMut(u8)>(put: F) {
    let mut c = [0u8; 29];
    c[0] = 2; // proto_ver
    c[1] = 0; // radio_kind: SX1262
    c[2..6].copy_from_slice(&sx1262::FREQ_MIN_HZ.to_be_bytes());
    c[6..10].copy_from_slice(&sx1262::FREQ_MAX_HZ.to_be_bytes());
    c[10] = sx1262::PWR_MIN_DBM as u8;
    c[11] = sx1262::PWR_MAX_DBM as u8;
    c[12..16].copy_from_slice(&STAMP_HZ.to_be_bytes());
    c[16] = 2; // stamp_kind: software counter
    c[17..19].copy_from_slice(&(RX_MAX as u16).to_be_bytes());
    c[19..23].copy_from_slice(&CMD_BITMAP.to_be_bytes());
    c[23] = sx1262::SF_MIN;
    c[24] = sx1262::SF_MAX;
    c[25..29].copy_from_slice(&0u32.to_be_bytes()); // sched_gran_ns: none
    send_frame(put, EVT_CAP, &c);
}

/// Reset-surviving handshake between CMD_ENTER_BOOTLOADER and [`maybe_enter_bootloader`]. The slot is
/// the top 4 bytes of SRAM, carved out of the linker's RAM region in `memory.x` (so nothing else uses
/// it) and retained across a SYSRESETREQ.
const BOOT_FLAG_ADDR: u32 = 0x2000_4FF8;
const BOOT_MAGIC: u32 = 0xB007_10AD;

/// Runs before RAM init on every reset (`bl __pre_init`, stack already valid). If CMD_ENTER_BOOTLOADER
/// armed the flag, jump to the GD32/STM32F1 system-memory ROM bootloader (`0x1FFF_F000`) NOW — while
/// every clock and peripheral is still at its reset default, which is exactly the state the bootloader
/// expects. We clear the flag first so a normal reset afterwards boots the app. Touches only a fixed
/// address + inline asm — no statics (which are not yet initialised here), as `#[pre_init]` requires.
#[cortex_m_rt::pre_init]
unsafe fn maybe_enter_bootloader() {
    if core::ptr::read_volatile(BOOT_FLAG_ADDR as *const u32) == BOOT_MAGIC {
        core::ptr::write_volatile(BOOT_FLAG_ADDR as *mut u32, 0);
        const SYSMEM_BASE: u32 = 0x1FFF_F000;
        let sp = core::ptr::read_volatile(SYSMEM_BASE as *const u32);
        let entry = core::ptr::read_volatile((SYSMEM_BASE + 4) as *const u32);
        core::arch::asm!(
            "msr msp, {sp}",
            "bx  {entry}",
            sp = in(reg) sp,
            entry = in(reg) entry,
            options(noreturn),
        );
    }
}

/// Emit an INFO event snapshotting chip status + the current knobs.
fn send_info<SPI, NSS, RST, BSY, DIO1, RFSW, E, F>(
    put: F,
    radio: &mut Sx1262<SPI, NSS, RST, BSY, DIO1, RFSW>,
    freq: u32,
    sf: u8,
    bw: u8,
    cr: u8,
    pwr: i8,
    lost: u16,
    cad_busy: u16,
    defer: u16,
) where
    SPI: embedded_hal::blocking::spi::Transfer<u8, Error = E>
        + embedded_hal::blocking::spi::Write<u8, Error = E>,
    NSS: embedded_hal::digital::v2::OutputPin,
    RST: embedded_hal::digital::v2::OutputPin,
    RFSW: embedded_hal::digital::v2::OutputPin,
    BSY: embedded_hal::digital::v2::InputPin,
    DIO1: embedded_hal::digital::v2::InputPin,
    F: FnMut(u8),
{
    let status = radio.get_status();
    let sync = radio.read_sync();
    let errors = radio.get_device_errors();
    let f = freq.to_be_bytes();
    let payload = [
        status,
        (sync >> 8) as u8,
        sync as u8,
        (errors >> 8) as u8,
        errors as u8,
        f[0],
        f[1],
        f[2],
        f[3],
        sf,
        bw,
        cr,
        pwr as u8,
        // Host bytes the ISR had to drop (ring full) or an overrun it caught. Should stay 0; a
        // climbing count is the link telling you it is losing commands, which used to be invisible.
        (lost >> 8) as u8,
        lost as u8,
        // #52 CSMA observability: CAD-busy (defers sensed) and DEFERRED transmissions. Cannot tune
        // carrier-sense blind — a climbing cad_busy with flat defer means backoff is working.
        (cad_busy >> 8) as u8,
        cad_busy as u8,
        (defer >> 8) as u8,
        defer as u8,
    ];
    send_frame(put, EVT_INFO, &payload);
}
