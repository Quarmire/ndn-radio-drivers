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
//!  * Nothing answers with **silence** any more — an unknown or badly-argued command gets
//!    `EVT_UNSUPPORTED [cmd, reason]` instead of costing the host four retries and a timeout.
//!
//! Two correctness fixes ship with it:
//!  * **C1** — the RX path buffered 64 bytes while CMD_TX accepted 240 and the host face declared an
//!    MTU of 200, so every received frame over 64 B was silently truncated. RX now runs to `RX_MAX`
//!    (247, the serial framing's real ceiling) and counts anything longer.
//!  * **C2** — the on-device NDN data plane recognised only the ASCII demo wire, so every offload
//!    path was INERT on the real face. It now parses NDNLPv2/NDN-TLV (see `ndn.rs`).
//!
//! ## Scheduled TX (2026-08-28) — CMD_TX_AT is real on this node
//!
//! v2 shipped `CMD_TX_AT → EVT_UNSUPPORTED[NO_HARDWARE]`, reasoning that the SX1262 has no delayed
//! key-up engine. True — but the wrong conclusion, because on this dongle **the MCU is the transmit
//! queue**: nothing reaches the air except through `transmit()`, so a GD32 timer compare that
//! releases the key-up at a deadline IS the scheduler, and it is scheduled on the same microsecond
//! counter EVT_RX stamps with. See [`Sched`] for the state machine, [`SCHED_GRAN_NS`] for the
//! granularity arithmetic, and `sx1262::stage_tx` for why the slow work happens before the deadline.
//!
//! ## 7E-A5 **v3** (2026-08-28) — the PHY is a knob, and the deadline is absolute
//!
//! v3 corrects a design error and closes a measured gap. Framing and every v2 opcode are unchanged.
//!
//!  * **`CMD_SET_PHY` (0x1D) — modulation is a KNOB, not an identity.** `SetPacketType` is a runtime
//!    command on every part in this fleet: the SX1262 does LoRa *and* (G)FSK, the SX1276 adds OOK,
//!    the LR2021 a dozen more. v2 encoded one part's boot-time choice as its `radio_kind`,
//!    which made "the same chip in another mode" look like a different radio. `radio_kind` now names
//!    the **part** (0 = SX1262, 1 = SX1276, 2 = LR2021) and the mode is selected at runtime.
//!  * **EVT_CAP describes the CURRENT PHY** and grows to 34 bytes: `phy_bitmap` (which
//!    `SetPacketType` values this node brings up, in the LR20xx wire numbering) and `phy_current`.
//!    `max_payload`, `sf_min`/`sf_max`, the airtime model and the usable `cmd_bitmap` are all
//!    per-PHY — a GFSK SX1262 has no spreading factor and no CAD — so `CMD_SET_PHY` replies with the
//!    WHOLE new EVT_CAP and the host replaces its profile rather than patching fields.
//!  * **`CMD_TX_AT_ABS` (0x1F) — an absolute deadline.** `CMD_TX_AT`'s delay is counted from when
//!    the FIRMWARE decodes the arm, so the host→device serial latency lands inside the placement.
//!    Measured on the LR2021: 45/45 slots fired with a mean gap 182 ticks off 2 400 000 nominal
//!    (accuracy is excellent) but a jitter sd of 553 µs against a declared 50 µs granularity — the
//!    same number as that node's 550 µs `CMD_GET_INFO` round-trip spread. Placing the deadline on
//!    the node's own `micros64()` removes the term entirely.
//!  * **`EVT_PHY_ERR` (0x8D)** `[requested_phy, chip_status]` — a PHY this node advertises that the
//!    silicon declined at runtime, carrying the chip's literal `GetStatus` byte.
//!  * **`CMD_SET_HOP` (0x1E) → `EVT_UNSUPPORTED[0x1E, NO_HARDWARE]`**, from an explicit arm: the
//!    SX126x has no intra-packet FHSS engine, so this node is structurally outside the hopping pair.
//!
//! ## Hardware RX timestamping (2026-08-28) — `stamp_kind` 2 → 3
//!
//! The `EVT_RX` timestamp is now latched **in silicon at the DIO1 edge** by a TIM3_CH3 input
//! capture, instead of being read by the MCU in the poll loop after `poll_rx` had finished its SPI
//! work. The unit is unchanged (1 µs, [`STAMP_HZ`]); what changes is accuracy. The old stamp landed
//! `(19 + n) × 8 µs` of SPI after `RxDone` — ~280 µs for a 16-byte frame, ~2.1 ms for a 247-byte
//! one — which is not jitter around a constant but a **bias that grows with frame length**, and a
//! length-coupled bias is exactly the term that cannot cancel in a two-way exchange.
//!
//! Three things make that claim safe to put on the wire:
//!
//!  * **One counter, and only when it has earned it.** TIM3 replaces SysTick as the microsecond
//!    clock, so `EVT_RX.ts`, `EVT_CLOCK`, `CMD_TX_AT_ABS`'s deadline and the capture register are
//!    all the same free-running counter with the same epoch. The host declares ONE clock domain per
//!    port and then differences a received stamp against a `CMD_READ_CLOCK` read; two "1 MHz"
//!    counters would satisfy every unit check on the wire and put an arbitrary offset into that
//!    subtraction. ★ But the boot rate measurement is now an ACTUATOR, not a diagnostic: if TIM3 is
//!    not ticking at [`STAMP_HZ`], `micros64()` goes back to being SysTick-derived and the node
//!    refuses the hardware stamp, so `stamp_hz` always describes the counter the node is really
//!    reading. See [`capture::choose_timebase`]. SysTick keeps `millis()` either way.
//!  * **An attribution rule with TWO halves, not a hope.** The timer's half — the capture is read
//!    inside `poll_rx` between `GetIrqStatus` and `ClearIrqStatus`, in a window `Sx1262::clear_irq`
//!    opens — counts edges and overcaptures ([`capture::classify`]). It is not sufficient: DIO1 is
//!    level-latched, so a frame arriving while the line is already high raises **no edge** while the
//!    chip's buffer and packet status advance to describe it, and the timer then sees one clean edge
//!    belonging to an earlier frame. The chip's own completed-packet count, differenced across the
//!    window, is the second half ([`capture::attribute`]). A capture that fails either is DISCARDED,
//!    never reported as approximate, and the frame falls back to the software read with an
//!    `EVT_RX_STAMP` saying so *for that frame*.
//!  * **A boot self-test.** `stamp_kind = 3` is what makes the host publish `LatchPoint::
//!    RadioCapture` and set `can_common_view`, so the byte is gated on `rxstamp::init` proving the
//!    two GD32 flag behaviours the design depends on, plus the tick rate — not on the code
//!    compiling. A failed self-test leaves the node advertising 2, honestly.
//!
//! What is **not** measured, and is not folded into any number, is the offset between the frame
//! arriving in the air and the DIO1 edge: RF front-end group delay (which moves with bandwidth),
//! the SX1262's own demodulate-and-flag latency, and above all the untrimmed 8 MHz HSI RC
//! oscillator that clocks this counter. See [`capture`]'s table.
//!
//! A retune also stopped costing 161 ms: three independent latency subtractions showed the extra
//! ~80 ms was a second **TCXO startup** forced by a redundant `CalibrateImage`, not the calibration
//! itself — `sx1262::set_frequency` now skips it while the target stays inside the calibrated band.

#![no_std]
#![no_main]
#![allow(dead_code)]

mod ndn;
mod rxstamp;
mod sx1262;

/// The pure half of the RX-timestamp contract, re-exported so the device modules can say
/// `crate::capture`. It lives in `src/lib.rs`'s tree rather than here because a `no_main` binary
/// cannot be built for a hosted target and therefore cannot be tested; see [`mod@capture`] and
/// `lib.rs` for the split.
pub use waveshare_lora_rs::capture;

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
/// 1 kHz SysTick → the free-running millisecond clock behind [`millis`].
///
/// **It is normally not behind `micros()` any more.** The microsecond clock is TIM3 (see
/// [`rxstamp::ticks64`]), because the RX timestamp is a TIM3 capture register and putting the
/// capture and the clock on two different counters would give the host one declared clock domain
/// containing two epochs — an error no unit check on the wire could catch.
///
/// ★ **"Normally" is load-bearing.** [`systick_micros64`] is still built and still exact, because a
/// boot measurement that finds TIM3 ticking at the wrong rate must be able to *act*: the node moves
/// its clock back here rather than publishing microseconds that are not microseconds. So this ISR
/// keeps carrying the millisecond counter's own wrap into `MILLIS_HI`, which is what makes that
/// fallback a genuine 64-bit monotonic µs clock instead of a u32 that restarts every ~71 minutes.
#[exception]
fn SysTick() {
    if MILLIS.fetch_add(1, Ordering::Relaxed) == u32::MAX {
        MILLIS_HI.fetch_add(1, Ordering::Relaxed);
    }
}

/// TIM3: the microsecond clock's 16-bit overflow, and the DIO1 input capture that stamps a received
/// frame. Both live in [`rxstamp::on_tim3_irq`]; this is only the vector.
#[interrupt]
fn TIM3() {
    rxstamp::on_tim3_irq();
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
const CMD_TX_AT: u8 = 0x18; //       payload = [delay_us u32 BE][frame] → EVT_TXDONE when it airs.
//                                   Scheduled on `micros64()`, the SAME counter EVT_RX stamps with
//                                   and CMD_READ_CLOCK returns, so a host converts freely between
//                                   "when it arrived" and "when to send". See [`Sched`].
const CMD_GET_CAP: u8 = 0x1A; //     payload = []  → EVT_CAP (29 bytes)
const CMD_SENSE: u8 = 0x1B; //       payload = []  → EVT_SENSE [activity u16 BE, rssi i16 BE]
// --- Waveshare-local extension. Outside the v2 block (0x17..0x1B) so it cannot collide with a future
// fleet assignment there; the host discovers it from EVT_CAP's cmd_bitmap, which is what the bitmap
// is for.
const CMD_SET_RX_GAIN: u8 = 0x1C; // payload = [0 = power-saving | 1 = boosted] → EVT_INFO
// --- 7E-A5 v3 (the PHY becomes a knob; the scheduled deadline becomes absolute) ---
const CMD_SET_PHY: u8 = 0x1D; //     payload = [packet_type u8, LR20xx wire numbering] → EVT_CAP
//                                   (the WHOLE new capability record — every field is per-PHY)
const CMD_SET_HOP: u8 = 0x1E; //     [hop_ctrl u8][hop_period u16 BE][n u8][freq_hz u32 BE]*n
//                                   → EVT_UNSUPPORTED[0x1E, NO_HARDWARE] on this node; see the arm
const CMD_TX_AT_ABS: u8 = 0x1F; //   payload = [target_ticks u64 BE][frame] → EVT_TXDONE when it airs
// Firmware -> host events.
const EVT_RX: u8 = 0x81; //    payload = [rssi i16 BE, snr i16 BE, ts_us u32 BE, LoRa bytes]
//                             ts_us is MICROseconds, not ms — the field was mislabelled `ts_ms` here
//                             and on the host. EVT_CAP.stamp_hz states the true rate.
//                             The value is the TIM3_CH3 capture latched at the SX1262's DIO1 edge
//                             (EVT_CAP.stamp_kind = 3). When a capture cannot be attributed to THIS
//                             frame it is discarded and `ts` is the software read instead — in which
//                             case an EVT_RX_STAMP says so, for that frame, immediately before this
//                             event. The layout is unchanged either way.
const EVT_TXDONE: u8 = 0x82; //payload = [ok, attempts]  (attempts=0 for a plain CMD_TX)
//                             + an 8-byte Waveshare-local tail ONLY for a CMD_TX_AT reply:
//                             [late_us u32 BE, keyup_us u32 BE] — how far past the requested instant
//                             SetTx was issued, and the chip's measured key-up. A host that reads
//                             only byte 0/1 (as ndn-radio-drivers does) is unaffected.
const EVT_INFO: u8 = 0x83; //  payload = [status, sync(2), errors(2), freq(4), sf, bw, cr, pwr, lost(2), cad_busy(2), defer(2)]
//                             19 bytes, FIXED: the host reads cad_busy/defer as the LAST 4 bytes, so
//                             nothing may ever be appended here. New counters go in EVT_STATS.
const EVT_LOG: u8 = 0x84; //   payload = ascii
const EVT_CAD: u8 = 0x85; //   payload = [busy(0/1)]
const EVT_RSSI: u8 = 0x86; //  payload = [rssi i16 BE]
const EVT_SF_DETECTED: u8 = 0x87; // payload = [sf | 0 = none]
const EVT_TX_STARTED: u8 = 0x88; //  payload = [airtime_ms u16 BE] — emitted just before key-up
const EVT_STATS: u8 = 0x89; //       payload = 44 B in v3 (32 in v2); the offsets are named in [`stats`], which the
//                                   emitter indexes with and the README's table transcribes, so the
//                                   wire and the documentation cannot drift.
const EVT_CLOCK: u8 = 0x8A; //       payload = [ticks u64 BE] (µs; see EVT_CAP.stamp_hz)
const EVT_CAP: u8 = 0x8B; //         payload = 34 bytes in v3 (29 in v2 + the PHY tail), all
//                                   multi-byte fields BIG-ENDIAN (see `send_cap`)
const EVT_SENSE: u8 = 0x8C; //       payload = [activity u16 BE, rssi i16 BE]
const EVT_PHY_ERR: u8 = 0x8D; //     payload = [requested_phy u8, chip_status u8] — a PHY this node
//                                   ADVERTISES that the chip refused at runtime, with the SX126x's
//                                   literal GetStatus byte (see `sx1262::cmd_status_ok`)
const EVT_UNSUPPORTED: u8 = 0x8F; // payload = [cmd, reason] — never silence, never a fake success
const EVT_RX_STAMP: u8 = 0x90; //    payload = [frame_stamp_kind u8, reason u8] — emitted
//                                   IMMEDIATELY BEFORE the EVT_RX it qualifies, and ONLY when that
//                                   frame's `ts` is NOT the hardware capture EVT_CAP advertises.
//                                   See [`send_rx_stamp_note`] for why the degradation has to be
//                                   per-frame and why it could not go in the EVT_RX header.
//                                   ☠ **0x8E is TAKEN** — it is the fleet's EVT_HOPTRACE on the
//                                   LR2021 and the Heltec, and this event sat on it. The collision
//                                   was silent in exactly the way the fleet's event-number registry
//                                   (`fleet_event_numbering` in lr2021-nrf54l15-rs/src/serial.rs)
//                                   exists to prevent, because that test listed only that node's own
//                                   constants: `tools/hoptrace.py` probes for hop support by sending
//                                   CMD_GET_HOPTRACE and accepting the first 0x8E or 0x8F, so one
//                                   degraded frame inside its 3 s window would have made a Waveshare
//                                   — which structurally cannot hop — answer "yes, and here is a
//                                   2-byte hop timeline". 0x90 is past the end of the v3 event space
//                                   and is now listed in that registry so the next collision fails a
//                                   build instead of a bench run.

/// **EVT_STATS (0x89) v3 — the byte offsets, named once.**
///
/// Every field is BIG-ENDIAN and unsigned. `CMD_GET_STATS` indexes the payload with these constants
/// and the README's STATS table is a transcription of this block, so a host reader written against
/// the documentation is written against the emitter.
///
/// Bytes `0..V1_LEN` are byte-identical to the v1 layout and `0..V2_LEN` to the v2 one. The host
/// parser length-checks `< 24` rather than `!= 24`, reads the v2 tail only when the reply is at
/// least 32 bytes, and explicitly accepts a LONGER reply and ignores the excess — so v1, v2 and v3
/// hosts all read a v3 node correctly and each sees exactly the fields it knows. That is why every
/// generation of this record appends rather than interleaves.
///
/// The v2 tail exists because RX loss was otherwise invisible: `poll_rx` drops a CRC failure and
/// returns `None`, so without the chip's own counters a quiet channel and a channel we are failing
/// to decode look identical from the host. The v3 tail exists for the same class of reason one
/// layer up — `stamp_kind = 3` claims a hardware RX timestamp, and without these counters a stamp
/// that quietly fell back to software would look exactly like one that did not.
///
/// Reset semantics differ per field and matter to anyone differencing them:
///   * `RX`..`RELAYED` and `DEFER` are firmware counters zeroed by `CMD_RESET_STATS` (0x14);
///   * `CAD_BUSY` here is the *resettable view* (`Csma::cad_busy_view`) — `CMD_RESET_STATS` moves a
///     baseline forward, while the free-running counter EVT_SENSE reports keeps advancing, so the
///     two never disagree about direction;
///   * `CHIP_RX`/`CHIP_CRC_ERR`/`CHIP_HDR_ERR` are the SX126x's own 16-bit `GetStats` registers,
///     zeroed on the chip by `CMD_RESET_STATS` (which issues `ResetStats`) and by any hard reset;
///   * `RX_TRUNC`, `HW_STAMPED`, `HW_STAMP_SW` and `HW_STAMP_AMBIG` are firmware counters, also
///     zeroed by `CMD_RESET_STATS`;
///   * `HW_STAMP_LAT_US` is a WATERMARK (worst case since the last reset), re-baselined by
///     `CMD_RESET_STATS`;
///   * `HW_STAMP_OVER` is free-running and is NOT reset — see its own note;
///   * `CLOCK_SKEW_MS` is an instantaneous read, not a counter, and reset means nothing to it.
mod stats {
    /// `rx` u32 — frames the on-device data plane classified.
    pub const RX: usize = 0;
    /// `filtered` u32 — dropped because no installed prefix covers the name.
    pub const FILTERED: usize = 4;
    /// `deduped` u32 — dropped as a duplicate Data object.
    pub const DEDUPED: usize = 8;
    /// `served` u32 — answered from the on-device Content Store.
    pub const SERVED: usize = 12;
    /// `relayed` u32 — re-broadcast by the relay set.
    pub const RELAYED: usize = 16;
    /// `cad_busy` u16 — channel sensed busy (resettable view).
    pub const CAD_BUSY: usize = 20;
    /// `defer` u16 — transmissions abandoned after the LBT backoff budget.
    pub const DEFER: usize = 22;
    /// End of the v1 payload. A v1 host stops here.
    pub const V1_LEN: usize = 24;
    /// `chip_rx` u16 — SX126x GetStats `nbPktReceived`.
    pub const CHIP_RX: usize = 24;
    /// `chip_crc_err` u16 — SX126x GetStats `nbPktCrcError`. The ONLY place a failed decode shows.
    pub const CHIP_CRC_ERR: usize = 26;
    /// `chip_hdr_err` u16 — SX126x GetStats `nbPktHeaderErr`.
    pub const CHIP_HDR_ERR: usize = 28;
    /// `rx_trunc` u16 — frames whose on-air length exceeded `RX_MAX`. Should stay 0.
    pub const RX_TRUNC: usize = 30;
    /// End of the v2 payload.
    pub const V2_LEN: usize = 32;

    // ---- v3 tail: is the hardware RX stamp actually working? ------------------------------------
    //
    // `stamp_kind` in EVT_CAP is a claim about the NODE. These five are the evidence for it, and
    // they are here because the alternative was for a degraded stamp to be indistinguishable from a
    // good one on the wire. `HW_STAMPED` and `HW_STAMP_SW` partition every frame `poll_rx`
    // delivered — counted before the data plane classifies it, so a frame the on-device filter
    // drops still lands here. Difference their sum against `CHIP_RX` above and the result is frames
    // the SX126x saw and the firmware never got.
    //
    // ★ That aggregate is no longer the ONLY detector for two packets coalescing into one `RxDone`,
    // and it never was a per-frame one. `poll_rx` now differences the chip's own completed-packet
    // count across each capture window and discards a stamp whose window did not hold exactly one
    // (`capture::attribute`), so a coalesced frame lands in `HW_STAMP_SW` and `HW_STAMP_AMBIG` at
    // the instant it happens. The overcapture counter is still NOT that detector and must not be
    // read as one: a frame arriving while DIO1 is still high produces no edge at all.

    /// `hw_stamped` u16 — frames whose `ts` is the hardware capture.
    pub const HW_STAMPED: usize = 32;
    /// `hw_stamp_sw` u16 — frames that fell back to the software read. Each one that was delivered
    /// to the host was also announced individually by an `EVT_RX_STAMP` immediately before its
    /// `EVT_RX`.
    pub const HW_STAMP_SW: usize = 34;
    /// `hw_stamp_ambig` u16 — of those, the ones discarded for MIS-ATTRIBUTION: the window held
    /// more than one DIO1 edge (the capture register holds the LATEST edge on this part, so the
    /// stamp may belong to a later frame), or the chip's completed-packet count did not advance by
    /// exactly one (a frame arrived while DIO1 was already high, so the payload is a later frame's
    /// than the edge). Both are the same failure — a plausible timestamp on the wrong frame — and
    /// both are discarded rather than reported. **This counter's meaning widened when the coalescing
    /// case became detectable per frame**; it used to count only the multi-edge case.
    pub const HW_STAMP_AMBIG: usize = 36;
    /// `hw_stamp_over` u16 — timer overcaptures (`CC3OF`). **An edge was LOST**, which is worse
    /// than a stamp being imprecise. Should stay 0. Free-running: `CMD_RESET_STATS` does not touch
    /// it, because `rxstamp::take` reads it as a difference against the window's baseline and
    /// moving one side of a difference under the reader is how a counter starts lying.
    pub const HW_STAMP_OVER: usize = 38;
    /// `hw_stamp_lat_us` u16 — worst timer-ISR entry latency in µs, saturating. **This is the
    /// measurement that says whether the hardware path was worth building**: it is how wrong a
    /// software stamp taken in the ISR would have been, and the stamp it replaced was taken later
    /// still, out in the poll loop after the SPI readback.
    pub const HW_STAMP_LAT_US: usize = 40;
    /// `clock_skew_ms` u16 — `|micros64()/1000 − millis()|`, the disagreement between the TIM3
    /// microsecond clock and the SysTick millisecond clock.
    ///
    /// They are divided from the same 8 MHz HSI, so this checks neither oscillator; what it catches
    /// is a **lost TIM3 overflow**, whose signature is unmistakable — the value jumps by 65 (one
    /// 65.536 ms wrap) and stays there. ⚠ On a node whose boot rate check moved the clock to
    /// [`systick_micros64`], the two sides are the same counter and this is structurally 0 — it is
    /// then vacuous rather than reassuring, and the boot log's `clk=` says which node you have. That is the one failure the 16-bit clock has that SysTick's
    /// 1 ms quantum does not, so it is instrumented rather than argued away. A steady 0 or 1 is
    /// expected: the two counters start a few hundred µs apart at boot.
    pub const CLOCK_SKEW_MS: usize = 42;
    /// Total v3 payload length.
    pub const LEN: usize = 44;
}

// The layout is contiguous, in order, and exactly 44 bytes — asserted rather than trusted, because
// the README's byte offsets are being read by another agent as the contract.
const _: () = assert!(stats::FILTERED == stats::RX + 4);
const _: () = assert!(stats::DEDUPED == stats::FILTERED + 4);
const _: () = assert!(stats::SERVED == stats::DEDUPED + 4);
const _: () = assert!(stats::RELAYED == stats::SERVED + 4);
const _: () = assert!(stats::CAD_BUSY == stats::RELAYED + 4);
const _: () = assert!(stats::DEFER == stats::CAD_BUSY + 2);
const _: () = assert!(stats::V1_LEN == stats::DEFER + 2);
const _: () = assert!(stats::CHIP_RX == stats::V1_LEN);
const _: () = assert!(stats::CHIP_CRC_ERR == stats::CHIP_RX + 2);
const _: () = assert!(stats::CHIP_HDR_ERR == stats::CHIP_CRC_ERR + 2);
const _: () = assert!(stats::RX_TRUNC == stats::CHIP_HDR_ERR + 2);
const _: () = assert!(stats::V2_LEN == stats::RX_TRUNC + 2);
const _: () = assert!(stats::HW_STAMPED == stats::V2_LEN);
const _: () = assert!(stats::HW_STAMP_SW == stats::HW_STAMPED + 2);
const _: () = assert!(stats::HW_STAMP_AMBIG == stats::HW_STAMP_SW + 2);
const _: () = assert!(stats::HW_STAMP_OVER == stats::HW_STAMP_AMBIG + 2);
const _: () = assert!(stats::HW_STAMP_LAT_US == stats::HW_STAMP_OVER + 2);
const _: () = assert!(stats::CLOCK_SKEW_MS == stats::HW_STAMP_LAT_US + 2);
const _: () = assert!(stats::LEN == stats::CLOCK_SKEW_MS + 2);
const _: () = assert!(stats::LEN == 44 && stats::V2_LEN == 32 && stats::V1_LEN == 24);
// The whole record still has to fit the framing's one-byte `len`.
const _: () = assert!(stats::LEN <= 255);

// EVT_UNSUPPORTED reason codes.
const UNSUP_UNKNOWN_OPCODE: u8 = 0x01; // this firmware does not know the opcode at all
const UNSUP_NO_HARDWARE: u8 = 0x02; //   opcode understood, but this radio/firmware has no such engine
const UNSUP_BAD_LENGTH: u8 = 0x03; //    opcode understood, payload does not satisfy its argument
//                                       requirements (too short, or a guard magic wrong)
const UNSUP_OUT_OF_RANGE: u8 = 0x04; //  argument outside the range EVT_CAP advertises

// =====================================================================================================
// 7E-A5 v3 §PHY — the modulation is a KNOB, and the two numberings that describe it
// =====================================================================================================
//
// **The design error v3 undoes.** `SetPacketType` is a runtime command on every part in this fleet,
// so which modulation a node is running is a *state*, actuated like MCS or spreading factor — not an
// identity. v2 encoded one part's boot-time choice into `radio_kind`, which made the same silicon in
// a second mode look like a different radio. `radio_kind` now names the PART; the mode is
// `phy_current`, and `phy_bitmap` says which modes can be reached from here.
//
// ⚠ **Two numberings, and they disagree on both modes this part has.** The wire uses the LR20xx
// `SetPacketType` values; the SX126x has its own (`sx1262::SX126X_PKT_*`). LoRa is 0x0 on the wire
// and 0x01 on the chip; FSK is 0x2 on the wire and 0x00 on the chip. Note what that means: passing a
// wire value straight to the chip would select GFSK when the host asked for LoRa, and the chip would
// accept it — the failure is entirely silent and shows up only as an air link that never forms. The
// translation therefore happens exactly once, here at the protocol boundary, and the table below is
// asserted at build time rather than trusted.

/// The LR20xx `SetPacketType` values, which are what travels on the wire (datasheet Table 8-1; the
/// vendored crate's `PacketType` in `vendor/lr2021/src/cmd/cmd_common.rs` enumerates the same set).
/// Listed in full — including the twelve this part cannot do — so `phy_bitmap` can be read against
/// the whole space instead of against a list that only contains the answers.
mod wire_phy {
    pub const LORA: u8 = 0x0;
    // 0x1 is unassigned in the table.
    pub const FSK: u8 = 0x2;
    pub const BLE: u8 = 0x3;
    pub const RTTOF: u8 = 0x4;
    pub const FLRC: u8 = 0x5;
    pub const BPSK: u8 = 0x6;
    pub const LR_FHSS: u8 = 0x7;
    pub const WM_BUS: u8 = 0x8;
    pub const WISUN: u8 = 0x9;
    pub const OOK: u8 = 0xA;
    pub const RAW: u8 = 0xB;
    pub const Z_WAVE: u8 = 0xC;
    pub const O_QPSK_15_4: u8 = 0xD;
}

/// Returned by [`wire_to_chip_phy`] for a wire PHY this node cannot enter.
const PHY_CHIP_NONE: u8 = 0xFF;

/// Wire `SetPacketType` value -> the SX126x's own `SetPacketType` argument.
///
/// Only the two modes this firmware actually brings up map. Everything else — including LR-FHSS,
/// which the SX1262 silicon does have but only as a transmit-only mode that builds its own hop
/// sequence — returns [`PHY_CHIP_NONE`], and `CMD_SET_PHY` refuses it. A mode that cannot receive
/// is not a PHY this node can advertise.
const fn wire_to_chip_phy(wire: u8) -> u8 {
    match wire {
        wire_phy::LORA => sx1262::SX126X_PKT_LORA,
        wire_phy::FSK => sx1262::SX126X_PKT_GFSK,
        _ => PHY_CHIP_NONE,
    }
}

/// The inverse, for reporting `phy_current` from whatever the driver holds.
const fn chip_to_wire_phy(chip: u8) -> u8 {
    match chip {
        sx1262::SX126X_PKT_LORA => wire_phy::LORA,
        sx1262::SX126X_PKT_GFSK => wire_phy::FSK,
        _ => PHY_CHIP_NONE,
    }
}

// --- The mapping table, pinned. This is the one piece of v3 whose failure mode is silent. ---------
// The two numberings really are different on both modes (if they ever coincided the rest of these
// assertions would pass vacuously, so assert the disagreement itself first):
const _: () = assert!(wire_phy::LORA != sx1262::SX126X_PKT_LORA);
const _: () = assert!(wire_phy::FSK != sx1262::SX126X_PKT_GFSK);
// The two rows that exist:
const _: () = assert!(wire_to_chip_phy(wire_phy::LORA) == sx1262::SX126X_PKT_LORA); // 0x0 -> 0x01
const _: () = assert!(wire_to_chip_phy(wire_phy::FSK) == sx1262::SX126X_PKT_GFSK); //  0x2 -> 0x00
// ...and they round-trip, so `phy_current` reports back what `CMD_SET_PHY` was given:
const _: () = assert!(chip_to_wire_phy(wire_to_chip_phy(wire_phy::LORA)) == wire_phy::LORA);
const _: () = assert!(chip_to_wire_phy(wire_to_chip_phy(wire_phy::FSK)) == wire_phy::FSK);
// Every OTHER wire value in the LR20xx table is refused, named one by one rather than by a range, so
// adding a mode to `wire_to_chip_phy` without adding it to `PHY_BITMAP` fails the build:
const _: () = assert!(wire_to_chip_phy(0x1) == PHY_CHIP_NONE);
const _: () = assert!(wire_to_chip_phy(wire_phy::BLE) == PHY_CHIP_NONE);
const _: () = assert!(wire_to_chip_phy(wire_phy::RTTOF) == PHY_CHIP_NONE);
const _: () = assert!(wire_to_chip_phy(wire_phy::FLRC) == PHY_CHIP_NONE);
const _: () = assert!(wire_to_chip_phy(wire_phy::BPSK) == PHY_CHIP_NONE);
const _: () = assert!(wire_to_chip_phy(wire_phy::LR_FHSS) == PHY_CHIP_NONE);
const _: () = assert!(wire_to_chip_phy(wire_phy::WM_BUS) == PHY_CHIP_NONE);
const _: () = assert!(wire_to_chip_phy(wire_phy::WISUN) == PHY_CHIP_NONE);
const _: () = assert!(wire_to_chip_phy(wire_phy::OOK) == PHY_CHIP_NONE);
const _: () = assert!(wire_to_chip_phy(wire_phy::RAW) == PHY_CHIP_NONE);
const _: () = assert!(wire_to_chip_phy(wire_phy::Z_WAVE) == PHY_CHIP_NONE);
const _: () = assert!(wire_to_chip_phy(wire_phy::O_QPSK_15_4) == PHY_CHIP_NONE);
const _: () = assert!(wire_to_chip_phy(0x0E) == PHY_CHIP_NONE);
const _: () = assert!(wire_to_chip_phy(0x0F) == PHY_CHIP_NONE);

/// **EVT_CAP `phy_bitmap`** — bit N set ⇔ wire `SetPacketType` value N is usable on this node.
///
/// Bit 0 (LoRa) + bit 2 (FSK) = **0x0000_0005**. Both are brought up in
/// `sx1262::Sx1262::apply_phy`, both receive, and both have a real airtime model here; nothing is
/// claimed on the strength of the silicon's datasheet alone. The bitmap and the mapping cannot
/// disagree — the assertion below derives one from the other.
const PHY_BITMAP: u32 = (1u32 << wire_phy::LORA) | (1u32 << wire_phy::FSK);
const _: () = assert!(PHY_BITMAP == 0x0000_0005);
const _: () = assert!(PHY_BITMAP.count_ones() == 2);
// Every advertised bit maps to a chip value, and every unadvertised one does not. Written as an
// explicit loop over the whole u32 so a bit added to either side alone cannot pass.
const _: () = {
    let mut i = 0u8;
    while i < 32 {
        let advertised = (PHY_BITMAP >> i) & 1 != 0;
        assert!(advertised == (wire_to_chip_phy(i) != PHY_CHIP_NONE));
        i += 1;
    }
};

/// The per-PHY half of EVT_CAP. **`max_payload`, the SF span and the usable command set all move
/// with the PHY** — which is why `CMD_SET_PHY` answers with a whole new capability record and the
/// host replaces its profile instead of patching fields.
struct PhyCaps {
    sf_min: u8,
    sf_max: u8,
    max_payload: u16,
    cmd_bitmap: u32,
}

/// The capability record for a wire PHY. Only called for PHYs in [`PHY_BITMAP`].
///
/// **Band**: both PHYs report `sx1262::FREQ_MIN_HZ`..`FREQ_MAX_HZ`, and that is not a copy-paste —
/// the binding constraint is `CalibrateImage`, which takes a frequency *band* and is independent of
/// the packet type, so the two modes genuinely share it.
///
/// **`max_payload`**: 247 in both, and for the same reason in both — the 7E-A5 framing's one-byte
/// `len` minus EVT_RX's 8-byte header. The radio limits are larger and therefore not binding (LoRa
/// PDU 255, `sx1262::GFSK_PDU_MAX` 255).
const fn phy_caps(wire: u8) -> PhyCaps {
    match wire {
        wire_phy::FSK => PhyCaps {
            // GFSK has no spreading factor. 0/0 is the explicit "this PHY has none" — not a
            // leftover LoRa span that a planner would read as a real knob.
            sf_min: 0,
            sf_max: 0,
            max_payload: RX_MAX as u16,
            cmd_bitmap: CMD_BITMAP_FSK,
        },
        // LoRa, and the default for anything else — `CMD_SET_PHY` never admits another value.
        _ => PhyCaps {
            sf_min: sx1262::SF_MIN,
            sf_max: sx1262::SF_MAX,
            max_payload: RX_MAX as u16,
            cmd_bitmap: CMD_BITMAP_LORA,
        },
    }
}

// --- The v3 capability surface, checked at build time. -------------------------------------------
//
// `#[cfg(test)]` cannot do this job: the crate is `no_std`/`no_main` for thumbv7m, so a test harness
// never runs. A `const` block does — it is evaluated by the same compiler invocation that produces
// the firmware, on the real constants the emitter indexes with, and a violation is a build failure
// rather than a test somebody forgot to run.
const _: () = {
    // The LoRa profile is the v2 one, unchanged.
    let l = phy_caps(wire_phy::LORA);
    assert!(l.sf_min == sx1262::SF_MIN && l.sf_max == sx1262::SF_MAX);
    assert!(l.cmd_bitmap == CMD_BITMAP);
    assert!(l.max_payload == RX_MAX as u16);
    // The GFSK profile differs in exactly the ways GFSK differs from LoRa, and agrees where the
    // constraint is shared.
    let f = phy_caps(wire_phy::FSK);
    assert!(f.sf_min == 0 && f.sf_max == 0); // no spreading factor exists in this modem
    assert!(f.cmd_bitmap & opbit(CMD_CAD) == 0); // no CAD either
    assert!(f.cmd_bitmap & opbit(CMD_SF_SCAN) == 0);
    assert!(f.cmd_bitmap & opbit(CMD_SET_MOD) == 0);
    assert!(f.cmd_bitmap & opbit(CMD_SET_CAD_CFG) == 0);
    assert!(f.cmd_bitmap & opbit(CMD_TX) != 0); // ...but it still transmits and receives
    assert!(f.cmd_bitmap & opbit(CMD_SENSE) != 0); // and still senses, via the energy detector
    // Both PHYs are bound by the SERIAL framing, not by the radio, so they land on the same number
    // for the same reason — this is the assertion that catches someone "fixing" one of them to a
    // radio limit (255 in both cases) and quietly breaking EVT_RX.
    assert!(f.max_payload == l.max_payload);
    assert!(f.max_payload as usize + 8 == 255);
};

// The two airtime models are genuinely different arithmetic, not a shared formula with a parameter:
// a 32-byte frame is ~72 ms at SF7/BW125 and ~8 ms at 50 kbps GFSK. `airtime_ms_for` picks between
// them, and EVT_TX_STARTED (which the host re-bases its reply deadline on) carries the result — so
// picking wrong would misstate a transmission by nearly an order of magnitude.
const _: () = assert!(sx1262::gfsk_airtime_ms(32, 8) == 8);
const _: () = assert!(sx1262::airtime_ms(7, sx1262::BW_125, sx1262::CR_4_5, 32, 8) == 72);
const _: () = assert!(
    sx1262::airtime_ms(7, sx1262::BW_125, sx1262::CR_4_5, 32, 8) > 5 * sx1262::gfsk_airtime_ms(32, 8)
);

// `cmd_status_ok` is the oracle behind EVT_PHY_ERR, so pin the SX126x `GetStatus` decode against the
// datasheet's cmdStatus values (bits [3:1]) rather than against the shift that implements it.
// STBY_RC (chipMode 2) with each cmdStatus in turn:
const _: () = assert!(sx1262::cmd_status_ok(0x2 << 4 | 0x2 << 1)); //  data available
const _: () = assert!(sx1262::cmd_status_ok(0x2 << 4 | 0x6 << 1)); //  command TX done
const _: () = assert!(!sx1262::cmd_status_ok(0x2 << 4 | 0x3 << 1)); // command timeout
const _: () = assert!(!sx1262::cmd_status_ok(0x2 << 4 | 0x4 << 1)); // command processing error
const _: () = assert!(!sx1262::cmd_status_ok(0x2 << 4 | 0x5 << 1)); // failure to execute command
// ...and the chip-mode bits must not leak into the verdict: the same cmdStatus in TX mode (5) reads
// the same way.
const _: () = assert!(!sx1262::cmd_status_ok(0x5 << 4 | 0x5 << 1));
const _: () = assert!(sx1262::cmd_status_ok(0x5 << 4 | 0x6 << 1));

/// Energy-detect threshold used when the host has not set one **and the current PHY has no CAD**.
///
/// In LoRa, sensing is CAD and the RSSI threshold is an optional addition for non-LoRa interference,
/// so leaving it disabled is a real choice. In GFSK there is no CAD at all, so a disabled threshold
/// would make `sense_busy` structurally incapable of ever returning busy — an LBT that always
/// transmits and a `CMD_SENSE` that reports a saturated channel as free, both silently. −95 dBm is
/// well above this receiver's noise floor at 117.3 kHz and well below any frame worth deferring for.
/// A host-set `rssi_thresh` always wins; this only replaces the *absence* of one.
const GFSK_FALLBACK_RSSI_THRESH: i16 = -95;

/// The effective energy-detect threshold for one sense: the host's if it set one, otherwise the
/// GFSK fallback when the PHY has no CAD, otherwise disabled.
fn effective_rssi_thresh(host_thresh: i16, has_cad: bool) -> i16 {
    if host_thresh > i16::MIN {
        host_thresh
    } else if has_cad {
        i16::MIN
    } else {
        GFSK_FALLBACK_RSSI_THRESH
    }
}

/// **The self-description bitmap** (EVT_CAP `cmd_bitmap`): bit N set ⇔ opcode N is implemented and
/// will act. The host uses it to decide what it may send, so it must be EXACT — and it is also the
/// firmware's own "is this a known opcode?" oracle in `handle_cmd`, so the bitmap and the dispatcher
/// physically cannot drift apart.
///
/// Set: 0x01..=0x17 (every command from CMD_TX through CMD_READ_CLOCK) = bits 1..23 → `0x00FF_FFFE`
///      0x18 CMD_TX_AT    → bit 24 → `0x0100_0000`  (scheduled TX; see [`Sched`])
///      0x1A CMD_GET_CAP  → bit 26 → `0x0400_0000`
///      0x1B CMD_SENSE    → bit 27 → `0x0800_0000`
///      0x1C CMD_SET_RX_GAIN → bit 28 → `0x1000_0000`
///      0x1D CMD_SET_PHY  → bit 29 → `0x2000_0000`  (v3: the PHY knob)
///      0x1F CMD_TX_AT_ABS → bit 31 → `0x8000_0000` (v3: absolute-deadline scheduled TX)
/// Clear: bit 0 (no opcode 0), bit 25 (0x19 unassigned), and **bit 30 (0x1E CMD_SET_HOP), which
///        this node understands and refuses** — see [`REFUSED_OPCODES`].
/// Total = 0xBDFF_FFFE.
const CMD_BITMAP: u32 = 0xBDFF_FFFE;

/// Opcodes with a `handle_cmd` arm that exists ONLY to refuse them, by name and with a reason.
/// They are deliberately absent from [`CMD_BITMAP`] — the bitmap means "implemented and will act" —
/// so the assertion below is what keeps a refusal from being mistaken for an implementation.
const REFUSED_OPCODES: u32 = opbit(CMD_SET_HOP);
const _: () = assert!(CMD_BITMAP & REFUSED_OPCODES == 0);

/// **Opcodes that have no actuator in GFSK.** Every one of them is a LoRa-modem function:
/// `SET_MOD` carries `[sf, bw, cr]` and GFSK has none of the three; `CAD`, `SET_CAD_CFG` and
/// `SF_SCAN` all rest on `SetCad`, which correlates against a LoRa preamble and does not exist in
/// the GFSK modem (see `sx1262::supports_cad`).
///
/// They are cleared from the `cmd_bitmap` EVT_CAP reports while the node is in GFSK — that is what
/// "EVT_CAP describes the CURRENT PHY" means for the command surface — and the dispatcher answers
/// them `UNSUPPORTED[.., NO_HARDWARE]` there, so a host that ignores the bitmap still gets a true
/// answer rather than a knob that silently does nothing.
const CMD_LORA_ONLY: u32 =
    opbit(CMD_SET_MOD) | opbit(CMD_CAD) | opbit(CMD_SET_CAD_CFG) | opbit(CMD_SF_SCAN);
const CMD_BITMAP_LORA: u32 = CMD_BITMAP;
const CMD_BITMAP_FSK: u32 = CMD_BITMAP & !CMD_LORA_ONLY;
// The per-PHY bitmap is a SUBSET of the firmware's, and drops exactly the four opcodes named above.
const _: () = assert!(CMD_BITMAP_FSK & !CMD_BITMAP == 0);
const _: () = assert!(CMD_LORA_ONLY.count_ones() == 4);
const _: () = assert!(CMD_BITMAP_FSK.count_ones() == CMD_BITMAP.count_ones() - 4);
// The literal the README's CAP table prints, pinned here so the documentation cannot drift from the
// emitter — the same reason the `stats` offsets are asserted.
const _: () = assert!(CMD_BITMAP_FSK == 0xBDFF_DAF6);
// Nothing in v3 is LoRa-only: the PHY knob and both scheduling opcodes must survive the switch, or
// a node that entered GFSK could not be told to leave it.
const _: () = assert!(CMD_BITMAP_FSK & opbit(CMD_SET_PHY) != 0);
const _: () = assert!(CMD_BITMAP_FSK & opbit(CMD_TX_AT) != 0);
const _: () = assert!(CMD_BITMAP_FSK & opbit(CMD_TX_AT_ABS) != 0);

/// **B4: compile-time proof that the bitmap is the dispatcher.** `IMPLEMENTED_OPCODES` lists one
/// entry per `handle_cmd` match arm; the assertion below fails the build if the two ever disagree.
/// Without it the bitmap is a hand-maintained claim about code somewhere else — exactly the kind of
/// capability statement the host has no way to check and every reason to believe.
const fn opbit(op: u8) -> u32 {
    1u32 << op
}
const IMPLEMENTED_OPCODES: u32 = opbit(CMD_TX)
    | opbit(CMD_SET_FREQ)
    | opbit(CMD_SET_MOD)
    | opbit(CMD_SET_PWR)
    | opbit(CMD_SET_SYNC)
    | opbit(CMD_GET_INFO)
    | opbit(CMD_SET_BEACON)
    | opbit(CMD_CAD)
    | opbit(CMD_GET_RSSI)
    | opbit(CMD_SET_CAD_CFG)
    | opbit(CMD_SET_LBT_CFG)
    | opbit(CMD_SET_PREAMBLE)
    | opbit(CMD_SF_SCAN)
    | opbit(CMD_TX_LBT)
    | opbit(CMD_SET_NAME_FILTER)
    | opbit(CMD_SET_RELAY)
    | opbit(CMD_DATAPLANE)
    | opbit(CMD_SET_SENSE_CFG)
    | opbit(CMD_GET_STATS)
    | opbit(CMD_RESET_STATS)
    | opbit(CMD_SET_DEBUG)
    | opbit(CMD_ENTER_BOOTLOADER)
    | opbit(CMD_READ_CLOCK)
    | opbit(CMD_TX_AT)
    | opbit(CMD_GET_CAP)
    | opbit(CMD_SENSE)
    | opbit(CMD_SET_RX_GAIN)
    | opbit(CMD_SET_PHY)
    | opbit(CMD_TX_AT_ABS);
const _: () = assert!(CMD_BITMAP == IMPLEMENTED_OPCODES);
// 29 opcodes are implemented, and opcode 0 must never be claimed (there is no command 0).
const _: () = assert!(CMD_BITMAP.count_ones() == 29);
const _: () = assert!(CMD_BITMAP & 1 == 0);
// The dispatcher's "is this a known opcode?" test shifts by `typ`, so every claimed bit must be
// below 32 or that test would be undefined for it. 0x1F is the last opcode the one-byte type field
// can carry into this bitmap at all — a v4 command past it needs a wider oracle, not another bit.
const _: () = assert!(CMD_TX_AT_ABS < 32);
const _: () = assert!(CMD_TX_AT_ABS == 31);

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
/// parser, the CS-serve buffer, the scheduled-TX queue and the whole `DataPlane` — measures
/// 3 536 B at v3 (3 528 at v2; 3 248 before [`Sched`] added its 251-byte frame buffer), with no
/// callee frame above 140 B. Peak ≈ 3.7 KB of 20.4 KB, so ~16 KB spare. That is what paid for
/// `rxbuf` 64→247, the EVT_RX scratch 72→255, and `ndn::CS_MAX_LEN` 96→192.)
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

/// Free-running millisecond clock (SysTick ISR). Wraps every ~49.7 days, which is far outside any
/// window this firmware measures; its one consumer is the Content Store's 30 s freshness check.
static MILLIS: AtomicU32 = AtomicU32::new(0);
/// [`MILLIS`]'s own wrap count (every ~49.7 days), so [`systick_micros64`] is 64-bit monotonic.
static MILLIS_HI: AtomicU32 = AtomicU32::new(0);
fn millis() -> u32 {
    MILLIS.load(Ordering::Relaxed)
}

/// SysTick reload: 8000 cycles = 1 ms at the 8 MHz HSI core clock.
const SYSTICK_RELOAD: u32 = 8_000 - 1;

/// **`stamp_hz` = 1_000_000.** The unit of the EVT_RX timestamp, of EVT_CLOCK and of
/// `CMD_TX_AT_ABS`'s deadline: MICROseconds, not milliseconds. (The old `ts_ms` label on EVT_RX was
/// simply wrong.) Defined once in [`capture`] and mirrored here, pinned below, because the same
/// figure has to be the TIM3 tick rate, the capture rate and the wire field simultaneously.
///
/// ⚠ **Do not "conveniently" rescale this.** `CMD_TX_AT`'s delay is microseconds *by definition*
/// while `CMD_TX_AT_ABS`, `EVT_CLOCK` and `EVT_RX.ts` are all `stamp_hz` ticks — and `Sched` mixes
/// them in one `deadline` field. They coincide only because the rate is exactly 1 MHz. Changing it
/// would rescale three wire fields, leave the fourth behind, and silently invalidate
/// `SCHED_GRAN_NS`, `SCHED_GATE_US` and `SCHED_MAX_DELAY_US`, which are all µs/ns constants.
const STAMP_HZ: u32 = 1_000_000;
const _: () = assert!(STAMP_HZ == capture::STAMP_HZ);
// TIM3 (the clock and the capture) and TIM2 (the deadline compare) must tick at the same rate, or a
// deadline computed on one and released by the other acquires a scale factor nothing would report.
const _: () = assert!(STAMP_HZ == SCHED_TIMER_HZ);

/// **The node's one microsecond clock**: 64-bit, monotonic, free-running from boot — and the SAME
/// counter the RX capture register belongs to.
///
/// It was SysTick-derived (`ms * 1000 + (RELOAD − CVR)/8`) until the hardware RX stamp landed. The
/// reason it had to move is not accuracy — SysTick and TIM3 are the same 8 MHz HSI divided twice, so
/// there is no drift term between them at all, only a ±1 µs phase dither. It is **counter
/// identity**. The host declares one `ClockDomainId` per serial port and then uses it for three
/// different things: the domain of every `EVT_RX` stamp, the domain `CMD_READ_CLOCK` answers in, and
/// the domain `CMD_TX_AT_ABS` schedules against. If the stamp came from TIM3's capture while this
/// function still read SysTick, all three would carry values from two counters whose epochs differ
/// by an arbitrary offset — both "1 MHz", so no unit check on the wire could catch it, and the
/// failure would hide in the scheduling half while common view kept working.
///
/// **The one thing this trades away, stated rather than glossed:** TIM3 is 16-bit, so the clock now
/// depends on an overflow interrupt 15.26 times a second, and a lost overflow costs 65.536 ms where
/// a lost SysTick tick cost 1 ms. The probability is far lower — you would need 65 ms of masked
/// interrupts against 1 ms, and this firmware has no critical sections anywhere — but the quantum is
/// 65× larger, so it is instrumented (`stats::CLOCK_SKEW_MS`) rather than argued away. `ticks64`
/// also folds a *pending-but-unserviced* overflow, which the SysTick version could not do: it never
/// read COUNTFLAG.
///
/// ★ **Which counter this is, is decided at boot by a measurement.** [`rxstamp::ticks64`] reads
/// TIM3 only if `rxstamp::init` measured it ticking at [`STAMP_HZ`]; otherwise it reads
/// [`systick_micros64`], the pre-capture clock. Either way the unit is a real microsecond and
/// `EVT_CAP.stamp_hz` describes the counter actually being read — which is the property that broke
/// when the rate check reached nothing but a capability byte. The capture is only claimed on the
/// TIM3 branch, so counter identity above still holds in both.
///
/// 48 bits of microseconds ≈ 8.9 years, so it is a genuine monotonic u64 for `CMD_READ_CLOCK`.
fn micros64() -> u64 {
    rxstamp::ticks64()
}

/// **The pre-capture microsecond clock, kept as the fallback a detected rate fault actuates.**
///
/// `MILLIS * 1000 + (RELOAD − CVR)/8` — the SysTick counter read down to the cycle, extended by the
/// ISR's millisecond count. Called by [`rxstamp::ticks64`] and by nothing else.
///
/// ☠ **Why it had to come back.** `rxstamp::init` measures TIM3's rate against SysTick precisely
/// because a GD32 clock-tree or prescaler divergence is plausible and is a factor of 2 or 8. When
/// that check fired, the only thing it moved was one capability byte: the node lowered `stamp_kind`
/// to 2 — honest about the latch point — and went on using the mis-rated counter as its microsecond
/// clock while `EVT_CAP.stamp_hz` still said 1 000 000. `EVT_CLOCK` would have read 2× fast,
/// `CMD_TX_AT` would have aired at half the requested delay (its `delay_us` is microseconds *by
/// definition*, so publishing the measured rate instead would not have fixed it), and `late_us`,
/// `keyup_us` and the re-published `sched_gran_ns` would all have inherited the factor. Before the
/// capture landed, that fault was structurally impossible here — this function is derived from the
/// *core* clock via SysTick, not from the APB1 timer tree the self-test was written to distrust.
///
/// It is **not** a second clock domain in the sense the host cares about: exactly one of the two is
/// live for the whole run, chosen at boot by [`capture::choose_timebase`] before the first reader,
/// and the choice is reported in the boot `EVT_LOG`. What is given up on this path is the capture:
/// an edge latched in TIM3 ticks cannot be published against a SysTick epoch, so `stamp_kind` drops
/// to 2 and the host routes every frame to `host_stamp()`.
fn systick_micros64() -> u64 {
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
/// clock, so the two never disagree; it wraps every ~71 min, which is fine for relative timing and
/// is why CMD_READ_CLOCK returns the full 64 bits.
fn micros() -> u32 {
    micros64() as u32
}

// =====================================================================================================
// Scheduled TX (CMD_TX_AT 0x18)
// =====================================================================================================
//
// **Why this node can do it at all.** The SX1262 has no delayed key-up engine: `SetTx` starts the
// transmitter now, and the only "later" the chip understands is an RX timeout. But nothing reaches
// the air here except through the MCU, so the MCU *is* the transmit queue, and a GD32 timer compare
// that releases `SetTx` at a deadline is a real scheduled transmission rather than a re-labelled
// busy-wait. Answering EVT_UNSUPPORTED[NO_HARDWARE] was honest about the radio and wrong about the
// node.
//
// **The shape of the problem is the TCXO.** A cold key-up out of STDBY_RC costs
// `sx1262::TCXO_STARTUP_US` = 78.125 ms (the DIO3 startup timeout) — the same quantum that shows up
// three times over in the measured knob latencies (see `sx1262::set_frequency`). Scheduling on top
// of that would give a granularity of tens of milliseconds. So the design is: do every slow step
// AHEAD of the deadline (`sx1262::stage_tx`, which leaves the chip in STDBY_XOSC with the crystal
// still running), and leave exactly one four-byte SPI transaction to perform at it.
//
// **And it must not eat the host link.** USART1 has no FIFO; the ISR rescues each byte into `RING`,
// which holds 512 bytes ~= 44 ms of continuous 115200 traffic. So the scheduler is a state machine
// the main loop *services*, never a delay it *waits out*: while `Armed` the radio stays in RX and
// the loop runs normally; the only blocking stretches are the staging SPI (~2.7 ms for a full frame)
// and the final `SCHED_GATE_US` = 4 ms handed to the hardware timer.

/// The frame a `CMD_TX_AT` can carry: the command payload is at most 255 bytes (the framing's `len`
/// is one byte) and 4 of them are the delay, so 251 is the exact ceiling, not a chosen one.
const SCHED_FRAME_MAX: usize = 251;

/// The frame a `CMD_TX_AT_ABS` can carry: the same 255-byte command payload, minus the 8-byte
/// absolute target. **247** — which is `RX_MAX` and for exactly the same reason, the framing's
/// one-byte `len`. Also derived rather than chosen.
const SCHED_ABS_FRAME_MAX: usize = 247;
const _: () = assert!(8 + SCHED_ABS_FRAME_MAX == 255);
// Both entry points share `Sched::buf`, so the absolute path must fit the buffer the relative one
// sizes. It does, with room to spare — but asserted, because the copy below indexes with it.
const _: () = assert!(SCHED_ABS_FRAME_MAX <= SCHED_FRAME_MAX);

/// TIM2 tick rate. 1 MHz makes one tick one microsecond, matching `micros64`'s unit exactly, so the
/// deadline arithmetic never converts between timebases.
const SCHED_TIMER_HZ: u32 = 1_000_000;

/// How long before the deadline the frame is pushed into the SX1262.
///
/// Must exceed the staging cost: `write_buffer` of 251 bytes at SCK = 1 MHz is 253 bytes x 8 us =
/// 2.02 ms, plus SetStandby/SetPacketParams/ClearIrq (~12 bytes) and the `wait_busy` poll's 10 us
/// granularity — call it 2.7 ms worst case. 8 ms gives ~3x margin and is still far inside the ring's
/// 44 ms. It is a starting value, not an assumption: `Sched::observe_stage` raises it if a staging
/// pass ever measures longer (which is what would happen if STDBY_XOSC did not keep the crystal
/// alive and staging had to absorb a 78 ms TCXO restart).
const SCHED_LEAD_US: u32 = 8_000;

/// Ceiling on the adaptive lead, so a pathological measurement cannot make the node refuse to
/// schedule anything. 200 ms comfortably covers a TCXO restart plus a full frame.
const SCHED_LEAD_MAX_US: u32 = 200_000;

/// The last stretch, handed to the hardware timer. Bounded by TIM2's 16-bit ARR (65 535 us) and kept
/// far below it: this is the only interval in which the main loop stops draining `RING`, so 4 ms of
/// the ring's 44 ms is the budget it may spend to buy a hardware-precise release.
const SCHED_GATE_US: u32 = 4_000;
const _: () = assert!(SCHED_GATE_US <= 65_535);

/// Iteration ceiling on the UIF spin, so a TIM2 that never fires (a clock-gating mistake, a future
/// peripheral init that disables it) costs one late transmission instead of hanging the firmware
/// with the host link un-drained. The loop is ~10 cycles at 8 MHz = 1.25 us, so 200 000 iterations
/// is ~250 ms — far above any legitimate [`SCHED_GATE_US`] wait and far below "forever".
const SCHED_GATE_SPIN_MAX: u32 = 200_000;

/// Bound on the post-`SetTx` BUSY measurement, so a wedged chip cannot hang the loop. Larger than a
/// TCXO restart (78.125 ms) on purpose — the point of the measurement is to SEE one if it happens.
const SCHED_KEYUP_TIMEOUT_US: u64 = 250_000;

/// Largest `CMD_TX_AT` delay this node accepts: **60 s**, matching the Heltec and the host.
///
/// The bound is not arbitrary — it is the host's own reply-timeout cap. `inject_after` waits
/// `tx_timeout(len) + delay_us.min(60_000_000)` (`ndn-radio-drivers/src/lora_serial.rs`), so a
/// longer schedule is one the caller has already decided it will not wait for.
///
/// Without a bound the field is a `u32` of microseconds: a host bug could arm this node for over an
/// hour, and because the queue is one deep and [`Sched::pending`] refuses while it is occupied,
/// every subsequent `CMD_TX_AT` would be answered `EVT_TXDONE[0, 0]` for that whole hour with no
/// way to clear it short of a reset. Refused with `UNSUP_OUT_OF_RANGE` rather than clamped, on the
/// same rule the other two nodes use: a frame placed in a slot other than the one the host asked
/// for is worse than a frame not sent.
const SCHED_MAX_DELAY_US: u32 = 60_000_000;

/// Everything about the release instant that is not the chip: TIM2's 1 us tick, the 1 us truncation
/// in `micros64` (it reports whole microseconds), and the ~12-cycle UIF poll at 8 MHz = 1.5 us,
/// rounded up to 2. TIM2 and TIM3 share the APB1 timer clock, so the deadline and the gate quantise
/// identically rather than merely commensurately — which is one term less than when the deadline
/// came off SysTick.
const SCHED_JITTER_US: u32 = 1 + 1 + 2;

/// **`sched_gran_ns`** — EVT_CAP [25..29], and the number a planner will believe.
///
/// The contract it states: *`SetTx` is issued at the requested instant, and the first symbol leaves
/// within `sched_gran_ns` of it.* The fixed part is not compensated away, because a compensation
/// derived from an unmeasured latency would be a bias dressed as precision.
///
/// ```text
///   TIM2 tick                  1 us   1 MHz timer; a compare cannot resolve finer than one tick
///   deadline quantization      1 us   micros64() reports whole us (one TIM3 tick)
///   UIF poll loop              2 us   read TIM2.SR + test + branch ~= 12 cycles at 8 MHz = 1.5 us,
///                                     rounded up                          } SCHED_JITTER_US = 4 us
///   SetTx SPI transaction     50 us   4 bytes at SCK = 1 MHz = 8 us/byte = 32 us, plus the two NSS
///                                     edges and the call boundary; rounded up
///   SetTx -> transmitter     100 us   BUSY-high while the chip processes SetTx out of STDBY_XOSC.
///                                     The datasheet does not state this crisply for a TCXO part, so
///                                     it is rounded UP generously -- and it is the ONE term that is
///                                     measured at runtime (`Sched::observe_keyup`), which can only
///                                     raise what EVT_CAP reports, never lower it
///   PA ramp                  200 us   sx1262::TX_RAMP_US -- SetTxParams ramp code 0x04 (SET_RAMP_200U)
///                                     ------
///                                     354 us
/// ```
///
/// Rounded UP to **400 us = 400 000 ns**. Where a term is uncertain it is rounded up, and the single
/// most uncertain term is replaced by a measurement as soon as the node schedules anything.
const SCHED_KEYUP_DESIGN_US: u32 = 50 + 100; // SPI transaction + SetTx -> transmitter
const SCHED_GRAN_NS: u32 = 400_000;

/// **Which of the two scheduling opcodes [`SCHED_GRAN_NS`] describes, and what the other one costs.**
///
/// Every term in the table above is internal — a timer tick, an SPI transaction, a PA ramp. None of
/// them involves the host. That is the whole granularity of `CMD_TX_AT_ABS` (0x1F), whose deadline
/// is a `micros64()` instant the host names outright, so nothing about when the command arrived can
/// move the release. **`sched_gran_ns` in EVT_CAP is that figure**, and it is what a planner should
/// build slots against.
///
/// `CMD_TX_AT` (0x18) releases just as precisely, but against a different reference: its delay is
/// counted from the moment the FIRMWARE decodes the arm, so the host→device transport sits between
/// "when the host meant" and "when the node starts counting". The node cannot measure that term —
/// it never sees the host's clock — so it is BOUNDED here from the closest thing this node has
/// measured: its own command round trips. `CMD_GET_INFO` on this dongle has a 4 758 µs mean floor
/// with a **sub-millisecond spread** (2026-08-28, n = 8..10), and the spread is what lands in the
/// placement — a constant offset a host can calibrate away, a varying one it cannot. So:
///
/// ```text
///   CMD_TX_AT_ABS (0x1F)   400 000 ns   derived above, entirely internal        <- EVT_CAP
///   CMD_TX_AT     (0x18)   400 000 ns + up to ~1 000 000 ns of host transport
/// ```
///
/// The same effect was MEASURED end-to-end on the LR2021, which has a 50 µs granularity and a 550 µs
/// `CMD_GET_INFO` spread: an absolute-boundary slot train fired 45/45 with a mean gap 182 ticks off
/// 2 400 000 nominal — the accuracy is excellent — while the jitter came out at sd 553 µs, the
/// round-trip number rather than the granularity. Host-armed relative scheduling was WORSE than that
/// node's software path (sd 553 vs 155 µs) purely because it pays an extra round trip. **Prefer
/// 0x1F.** 0x18 stays for hosts that have not moved.
const SCHED_REL_PLACEMENT_BOUND_NS: u32 = 1_000_000;
// The relative path can only be worse than the absolute one, never better.
const _: () = assert!(SCHED_REL_PLACEMENT_BOUND_NS > 0);
// The published figure must never be below the terms it is built from.
const _: () =
    assert!(SCHED_GRAN_NS >= (SCHED_JITTER_US + SCHED_KEYUP_DESIGN_US + sx1262::TX_RAMP_US) * 1000);

// --- Invariants the build enforces, because every one of them is a claim the host acts on. ---

// `NodeProfile::schedules_tx()` on the host is `sched_gran_ns > 0 && supports(CMD_TX_AT)`. The two
// halves of that must agree HERE, or the node advertises a scheduler with no actuator (or an
// actuator with no declared granularity) — precisely the drift `FrameIo::schedules_tx` warns about.
const _: () = assert!((CMD_BITMAP & opbit(CMD_TX_AT) != 0) == (SCHED_GRAN_NS > 0));
// ...and the same for the absolute entry point, which is the one `sched_gran_ns` actually describes.
const _: () = assert!((CMD_BITMAP & opbit(CMD_TX_AT_ABS) != 0) == (SCHED_GRAN_NS > 0));

// The one-byte `len` of the 7E-A5 framing is what bounds both directions, and both bounds are
// derived from it rather than chosen: EVT_RX spends 8 bytes on rssi/snr/timestamp, CMD_TX_AT spends
// 4 on the delay.
const _: () = assert!(8 + RX_MAX == 255);
const _: () = assert!(4 + SCHED_FRAME_MAX == 255);
// ...and the scheduled frame must still be a legal LoRa PDU.
const _: () = assert!(SCHED_FRAME_MAX <= 255);

// The gate must fit inside the staged window, and the adaptive lead inside its ceiling.
const _: () = assert!(SCHED_GATE_US < SCHED_LEAD_US);
const _: () = assert!(SCHED_LEAD_US < SCHED_LEAD_MAX_US);

/// How long `RING` can absorb host traffic with nobody draining it: 512 bytes at 115200 8N1
/// (10 bits per byte) = 44.4 ms. The scheduler's only non-draining stretch is the hardware gate, so
/// the assertion below is what keeps "a scheduled transmit must not block command intake" a
/// property of the code rather than a note in a comment.
const RING_DRAIN_BUDGET_US: u32 = ((RING_SZ as u64) * 10 * 1_000_000 / 115_200) as u32; // 44_444
const _: () = assert!(SCHED_GATE_US * 4 < RING_DRAIN_BUDGET_US);

/// **B3: when a refined `sched_gran_ns` is worth an unsolicited EVT_CAP.**
///
/// `Sched::gran_ns` sharpens the published figure from real key-up timing, but the host reads
/// EVT_CAP once at open and never again — so until now the refinement was unreachable and the host
/// went on planning against the derived 400 µs no matter what the hardware turned out to do. The
/// node therefore re-publishes EVT_CAP, unsolicited, when the measured figure has moved materially
/// from what was last published.
///
/// **25 %**, and the trigger is self-limiting rather than merely rate-limited: `gran_ns()` is the
/// maximum of a constant and a running worst-case, so it never decreases, and each re-publish must
/// clear 1.25x the last one. From 400 µs to the ceiling a stuck-BUSY measurement can reach
/// ([`SCHED_KEYUP_TIMEOUT_US`] + the fixed terms ≈ 250 ms) is a factor of 626, i.e. at most
/// ⌈log₁.₂₅ 626⌉ = **29 events in the lifetime of the node**, however many frames it schedules.
const CAP_REPUBLISH_RATIO_PCT: u32 = 25;
/// Belt-and-braces floor between two unsolicited EVT_CAPs, so even a pathological sequence cannot
/// put one inside a burst of scheduled frames. 5 s.
const CAP_REPUBLISH_MIN_US: u64 = 5_000_000;

#[derive(Clone, Copy, PartialEq, Eq)]
enum SchedState {
    /// Nothing scheduled.
    Idle,
    /// A deadline and a frame are held; the radio is still in RX and the node behaves normally.
    Armed,
    /// The frame is in the SX1262's buffer and the chip sits in STDBY_XOSC. Only `SetTx` remains.
    Staged,
}

/// The one-deep scheduled-TX queue and its self-measurement.
struct Sched {
    state: SchedState,
    /// Absolute release instant on the `micros64()` timebase — the SAME clock EVT_RX stamps with and
    /// CMD_READ_CLOCK returns, which is the whole point: the host converts between "when it arrived"
    /// and "when to send" without reconciling two clocks.
    deadline: u64,
    len: usize,
    /// Adaptive staging lead; starts at [`SCHED_LEAD_US`].
    lead_us: u32,
    /// Worst staging duration observed (us).
    stage_us: u32,
    /// Worst key-up observed (us): `micros64()` across `tx_issue` plus the chip's BUSY-high window.
    keyup_us: u32,
    /// Worst lateness observed (us): how far past `deadline` `SetTx` was actually issued.
    late_us: u32,
    /// Scheduled transmissions released so far.
    fired: u32,
    /// The `sched_gran_ns` value most recently PUBLISHED in an EVT_CAP — the baseline the B3
    /// re-publish trigger measures against. Starts at the derived figure, which is what a node that
    /// has never been asked for its capabilities would answer.
    published_gran_ns: u32,
    /// `micros64()` of the last EVT_CAP emission (solicited or not), for the rate limit.
    last_cap_us: u64,
    /// Airtime of the queued frame, whole ms — computed at arm time, where the modulation
    /// parameters are in scope, and carried here so `sched_service` can announce it without
    /// needing `sf`/`bw`/`cr`/`preamble` threaded through its signature.
    air_ms: u16,
    buf: [u8; SCHED_FRAME_MAX],
}

impl Sched {
    const fn new() -> Self {
        Self {
            state: SchedState::Idle,
            deadline: 0,
            len: 0,
            lead_us: SCHED_LEAD_US,
            stage_us: 0,
            keyup_us: 0,
            late_us: 0,
            fired: 0,
            published_gran_ns: SCHED_GRAN_NS,
            last_cap_us: 0,
            air_ms: 0,
            buf: [0; SCHED_FRAME_MAX],
        }
    }

    fn pending(&self) -> bool {
        !matches!(self.state, SchedState::Idle)
    }

    /// Fold a staging measurement in, and widen the lead if staging turned out to cost more than the
    /// lead allowed for. 1.5x the observed cost, so the next arm has margin rather than exactly
    /// enough. This is what makes the STDBY_XOSC assumption safe to hold: if it is wrong the first
    /// scheduled frame is late, says so in EVT_TXDONE, and the ones after it are on time.
    fn observe_stage(&mut self, us: u32) {
        if us > self.stage_us {
            self.stage_us = us;
        }
        let want = us.saturating_add(us / 2);
        if want > self.lead_us {
            self.lead_us = want.min(SCHED_LEAD_MAX_US);
        }
    }

    fn observe_keyup(&mut self, keyup_us: u32, late_us: u32) {
        if keyup_us > self.keyup_us {
            self.keyup_us = keyup_us;
        }
        if late_us > self.late_us {
            self.late_us = late_us;
        }
        self.fired = self.fired.wrapping_add(1);
    }

    /// What EVT_CAP publishes as `sched_gran_ns`: the derived design figure until this node has
    /// actually released a scheduled frame, and the MEASURED key-up once it has, whichever is
    /// larger. A capability can get more conservative from evidence; it can never get more
    /// optimistic from it.
    fn gran_ns(&self) -> u32 {
        let measured = self
            .keyup_us
            .saturating_add(SCHED_JITTER_US)
            .saturating_add(sx1262::TX_RAMP_US)
            .saturating_mul(1000);
        if measured > SCHED_GRAN_NS {
            measured
        } else {
            SCHED_GRAN_NS
        }
    }

    /// Record that `gran_ns` has just gone out on the wire, so the B3 trigger measures against what
    /// the host actually holds rather than against the last thing that was computed.
    fn note_cap_published(&mut self, gran_ns: u32, now: u64) {
        self.published_gran_ns = gran_ns;
        self.last_cap_us = now;
    }

    /// **B3's trigger.** True when the live figure differs from the published one by more than
    /// [`CAP_REPUBLISH_RATIO_PCT`] AND the rate limit has elapsed. The comparison is written both
    /// ways even though `gran_ns()` is monotone non-decreasing: a one-directional test would quietly
    /// become wrong the day the derivation gains a term that can fall.
    fn cap_republish_due(&self, gran_ns: u32, now: u64) -> bool {
        if now.saturating_sub(self.last_cap_us) < CAP_REPUBLISH_MIN_US {
            return false;
        }
        let published = self.published_gran_ns.max(1) as u64;
        let live = gran_ns as u64;
        let up = live.saturating_mul(100) > published.saturating_mul(100 + CAP_REPUBLISH_RATIO_PCT as u64);
        let down = live.saturating_mul(100 + CAP_REPUBLISH_RATIO_PCT as u64) < published.saturating_mul(100);
        up || down
    }
}

/// **The deadline timer: TIM2, one-pulse, 1 MHz.**
///
/// The microsecond clock is TIM3 and must stay free-running, so the scheduler needs its own
/// compare; TIM2 is otherwise unused on this board. Both are on the APB1 timer clock, so a deadline
/// computed on TIM3 and released by TIM2 quantises identically and carries no scale factor.
///
/// It is driven through the PAC rather than the HAL timer wrapper because the one thing that matters
/// here is that arming and polling are a handful of register accesses with nothing between the
/// compare firing and NSS going low.
struct SchedTimer;

impl SchedTimer {
    /// Enable TIM2 and prescale it to [`SCHED_TIMER_HZ`]. Returns the prescaler actually programmed
    /// so the caller can report it: at the 8 MHz HSI core clock APB1 is undivided, so the timer
    /// clock is 8 MHz and PSC = 7, giving exactly 1 us per tick.
    ///
    /// SAFETY: TIM2 is owned by the caller (it holds `pac::TIM2`) and RCC's APB1ENR is only touched
    /// here, once, before the main loop starts.
    fn init(pclk1_tim_hz: u32) -> u16 {
        let psc = (pclk1_tim_hz / SCHED_TIMER_HZ).saturating_sub(1) as u16;
        unsafe {
            (*pac::RCC::ptr())
                .apb1enr
                .modify(|_, w| w.tim2en().set_bit());
            let t = &*pac::TIM2::ptr();
            t.cr1.reset();
            t.psc.write(|w| w.psc().bits(psc));
            t.arr.write(|w| w.arr().bits(0xFFFF));
            t.egr.write(|w| w.ug().set_bit()); // load PSC/ARR
            t.sr.write(|w| w.uif().clear_bit()); // UG raised UIF; start clean
        }
        psc
    }

    /// Arm a one-shot for `us` microseconds. `us` must be non-zero and <= [`SCHED_GATE_US`].
    fn arm(us: u16) {
        unsafe {
            let t = &*pac::TIM2::ptr();
            t.cr1.write(|w| w.cen().clear_bit());
            t.arr.write(|w| w.arr().bits(us.saturating_sub(1)));
            t.egr.write(|w| w.ug().set_bit()); // reload counter + ARR now
            t.sr.write(|w| w.uif().clear_bit()); // and clear the UIF that UG just set
            t.cr1.write(|w| w.opm().set_bit().cen().set_bit());
        }
    }

    /// Has the one-shot fired? The hardware clears CEN itself in one-pulse mode.
    fn expired() -> bool {
        unsafe { (*pac::TIM2::ptr()).sr.read().uif().bit_is_set() }
    }
}

// Heartbeat-beacon base period in main-loop iterations (~seconds; the loop is SPI-poll bound).
// The beacon is runtime-toggleable via CMD_SET_BEACON and defaults OFF, so a fresh/reset dongle stays
// quiet; enable on-air discovery explicitly with CMD_SET_BEACON[1].
const BEACON_BASE_PERIOD: u32 = 250_000;

/// Formats into a fixed stack buffer so we can build payloads/logs with `write!`.
///
/// ⚠ `write_str` DROPS anything past the end — a formatted line longer than this is truncated with
/// no error, since `core::fmt::Write` has nowhere to report one and an EVT_LOG is not worth failing
/// a boot over. 96 bytes, sized so the longest line here (the boot diagnostic, ~61 characters with
/// every field at its widest) has real margin rather than fitting exactly. The RAM budget carries it
/// easily: peak stack measures ~4.2 KB of 20.4 KB.
const LOG_BUF: usize = 96;

struct BufWriter {
    buf: [u8; LOG_BUF],
    pos: usize,
}
impl BufWriter {
    fn new() -> Self {
        Self {
            buf: [0; LOG_BUF],
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

/// **Announce, for ONE frame, that its `EVT_RX.ts` is not the hardware capture this node
/// advertises.** Emits nothing when the stamp is good, which is the normal case.
///
/// ⚠ **Why this is an event and not a byte in the EVT_RX header.** `stamp_kind` in EVT_CAP is a
/// capability of the NODE; using it to cover a mixture of hardware captures and software fallbacks
/// is exactly what the LR2021's distinct `HwStamp` type exists to prevent. The obvious fix — a
/// validity byte in the EVT_RX header — is not free here: `8 + RX_MAX == 255` **exactly**, so a
/// ninth header byte costs `max_payload` 247 → 246, and `max_payload` is an EVT_CAP-advertised
/// quantity that peers size their frames against. Paying that for a byte that is 0 on every frame
/// in normal operation is the wrong trade; a separate event costs nothing on the common path and
/// nothing in the MTU.
///
/// The binding to the frame is ordering: this goes out immediately before that frame's `EVT_RX` on
/// a single-producer, in-order link, with nothing else written to the USART in between. The
/// reference host (`ndn-radio-drivers`' `lora_serial.rs`) consumes it exactly that way: it stashes
/// the note and applies it to the next `EVT_RX`, so a degraded frame is published with
/// `LatchPoint::HostRecv` instead of a 1 µs `RadioCapture`.
///
/// ☠ **That host change was not optional, and its absence was the defect.** This event alone did
/// nothing: an un-updated host routes any non-`EVT_RX` opcode to the command-reply channel, where an
/// unmatched type is skipped with `Ok(_) => continue`, and its only other treatment was an
/// `eprintln!` inside `if debug` — so on a normal run the note was *completely silent*, not
/// "logged", and every degraded frame was consumed as a 1 µs radio capture and folded into common
/// view. A per-frame degrade that no consumer reads is not a degrade. An older host is no worse off
/// than before (it ignores 0x90 and keeps its node-level `stamp_kind`), and the aggregate is in
/// EVT_STATS either way — but the fix is on the path the host actually reads.
///
/// The test is [`capture::note_needed`] — "this frame's kind differs from the node's advertised
/// kind" — not merely "the stamp was degraded". On a node whose boot self-test failed, EVT_CAP
/// already says `stamp_kind = 2` and every frame agrees with it, so a note per frame would repeat
/// the capability instead of qualifying it.
///
/// Byte 0 is the `stamp_kind` **of this frame**, in the same fleet vocabulary as `EVT_CAP[16]` —
/// deliberately, so a reader needs one vocabulary and not two. Byte 1 is
/// [`capture::Degrade`]'s code.
fn send_rx_stamp_note<F: FnMut(u8)>(put: F, v: capture::StampVerdict) {
    let advertised = capture::stamp_kind_byte(rxstamp::hw_stamp_live());
    if !capture::note_needed(v, advertised) {
        return;
    }
    let reason = v.degrade().map_or(0, |d| d.code());
    send_frame(put, EVT_RX_STAMP, &[v.frame_stamp_kind(), reason]);
}

/// Polls allowed for `HSERDY`. The RM quotes a few ms of crystal start-up; each iteration is a
/// volatile register read plus a branch, so at 8 MHz this is tens of milliseconds — generously past
/// any real oscillator, and still a hard bound. **The loop is bounded because an unbounded one is
/// how a dongle with no crystal becomes an ST-Link job.**
const HSE_PROBE_POLLS: u32 = 400_000;

/// Turn HSE on, see whether it ever reports ready, turn it off again if not. Returns true iff
/// `HSERDY` asserted.
///
/// This is the GUARD on the `use_hse` switch, not a diagnostic: the HAL's own `use_hse` waits on
/// `HSERDY` with no bound, so it must never be reached on a board whose crystal is absent or dead.
/// On the true path `HSEON` is deliberately left set, so the HAL finds the oscillator already
/// running and its wait returns at once.
fn probe_hse() -> bool {
    let rcc = unsafe { &*pac::RCC::ptr() };
    rcc.cr.modify(|_, w| w.hseon().set_bit());
    let mut ready = false;
    for _ in 0..HSE_PROBE_POLLS {
        if rcc.cr.read().hserdy().bit_is_set() {
            ready = true;
            break;
        }
    }
    if !ready {
        // Leave the bit exactly as it was found rather than powering an oscillator that answered
        // nothing for the rest of the run.
        rcc.cr.modify(|_, w| w.hseon().clear_bit());
    }
    ready
}

#[entry]
fn main() -> ! {
    let dp = pac::Peripherals::take().unwrap();
    let mut cp = cortex_m::Peripherals::take().unwrap();

    let mut flash = dp.FLASH.constrain();

    // ★ **Is there an 8 MHz crystal on this board?** — asked, not assumed, and asked in the one way
    // that cannot brick the dongle.
    //
    // This node's whole timebase (SysTick, TIM2's deadline, TIM3's capture) descends from the 8 MHz
    // **HSI**, an internal RC good to about +/-1%. Two of these dongles were MEASURED at ~-3100 ppm
    // relative, and that wander — not the stamp — is now what limits their common view, since the
    // TIM3 capture stamps 95/95 frames with no fallback. Moving to HSE would be the next real gain,
    // but only if the part is populated, and nothing in the repo records whether it is.
    //
    // ☠ The obvious way to find out is the dangerous one: `cfgr.use_hse(8.MHz())` spins forever on
    // `HSERDY` when no crystal is fitted, and firmware that hangs in clock init never reaches
    // `CMD_ENTER_BOOTLOADER` — the dongle would then need an ST-Link to recover, and the whole point
    // of the self-DFU path is that it does not. So the question is asked by a BOUNDED probe that
    // leaves the clock alone, and its answer GUARDS the switch below: `use_hse` is only ever reached
    // on a part that has already been seen to assert `HSERDY`, and a dongle with no crystal (or a
    // dead one) silently keeps the HSI it has always run on.
    //
    // MEASURED 2026-08-31 on the o5p-0 dongle: `hse=1` — the crystal is fitted and starts.
    let hse_present = probe_hse();

    let rcc = dp.RCC.constrain();
    // **`sysclk` is pinned at 8 MHz on BOTH paths, deliberately.** Every timing constant in this
    // firmware descends from it — SysTick's 8000-cycle reload, TIM2's `psc = pclk1_tim/1 MHz - 1`,
    // TIM3's identical prescaler and therefore `capture::STAMP_HZ` — so raising the core clock here
    // would silently rescale the scheduler and the RX timestamp together. The gain being sought is
    // not speed, it is the REFERENCE: the same 8 MHz from a crystal rather than from an RC.
    let clocks = if hse_present {
        rcc.cfgr
            .use_hse(8.MHz())
            .sysclk(8.MHz())
            .freeze(&mut flash.acr)
    } else {
        rcc.cfgr.freeze(&mut flash.acr)
    };

    // 1 kHz SysTick — the MILLISECOND clock only. The microsecond clock and the RX capture are TIM3
    // (see `rxstamp` and `micros64`), which is why this no longer feeds EVT_RX timestamps. Core clock
    // is 8 MHz HSI (the same assumption sx1262.rs makes for its busy-wait delays), so reload = 7999;
    // named rather than repeated as a literal, since the doc on the constant carries the arithmetic.
    cp.SYST.set_clock_source(SystClkSource::Core);
    cp.SYST.set_reload(SYSTICK_RELOAD);
    cp.SYST.clear_current();
    cp.SYST.enable_counter();
    cp.SYST.enable_interrupt();

    // TIM2 = the scheduled-TX deadline compare (CMD_TX_AT). Taken here so ownership is explicit even
    // though the register block is reached through the PAC pointer inside `SchedTimer`.
    let _tim2 = dp.TIM2;
    let sched_psc = SchedTimer::init(clocks.pclk1_tim().raw());

    // TIM3 = the microsecond clock AND the DIO1 input capture that stamps received frames — one
    // counter for both, which is the property the host's single clock domain per port depends on.
    // Taken here for the same reason as TIM2: the register block is reached through a PAC pointer
    // inside `rxstamp`, so ownership would otherwise be invisible. This must run BEFORE anything
    // calls `micros64()`, which now reads it — and AFTER SysTick is running, because the boot
    // self-test measures TIM3's rate against it.
    let _tim3 = dp.TIM3;
    let cap = rxstamp::init(clocks.pclk1_tim().raw());

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
    // 7E-A5 v3: the PHY in effect, in the WIRE numbering. `Sx1262::apply_phy` is the actuator and
    // `Sx1262::packet_type()` is the chip-side truth; this is the mirror EVT_CAP/EVT_INFO report,
    // kept in the same shape as the freq/sf/bw/cr/pwr mirrors beside it. `init` brings the chip up
    // in LoRa, so they agree from the first instruction.
    let mut phy: u8 = wire_phy::LORA;

    let diag = radio.init(freq, sf, bw, cr);
    debug_assert!(chip_to_wire_phy(radio.packet_type()) == phy);
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
            "ws-lora v3 sync=0x{:04X} err=0x{:04X} psc={} phy={} physt=0x{:02X} ok={}",
            diag.sync_readback,
            diag.device_errors,
            sched_psc,
            phy,
            diag.phy_status,
            sx1262::cmd_status_ok(diag.phy_status) as u8
        );
        send_frame(
            |b| {
                let _ = block!(tx.write(b));
            },
            EVT_LOG,
            log.as_slice(),
        );
    }

    // The capture self-test, as its own line rather than appended to the one above: a `BufWriter`
    // silently TRUNCATES past LOG_BUF, and this is the evidence for the `stamp_kind` byte the host
    // gates common view on — the last diagnostic that should be lost to a formatting overflow.
    //
    // `st` is the bit field from `rxstamp::SelfTest::bits` (0x1F = clean pass), `tps` the TIM3 ticks
    // counted across exactly one SysTick period (expect 1000; 0 means SysTick, the reference, never
    // moved), and `kind` the byte EVT_CAP will actually advertise. A node that comes up with
    // `kind=2` here is saying, at boot, that its hardware stamp did not prove out — which is a fact
    // about this GD32, not about the build.
    //
    // ★ `clk` is the counter `micros64()` actually reads: `tim3` when the rate measurement above
    // agreed with STAMP_HZ, `systick` when it did not. That second value is the whole point of
    // measuring the rate — a detected clock-tree fault moves the node's timebase instead of quietly
    // rescaling EVT_CLOCK, EVT_RX.ts and both CMD_TX_AT deadlines — so it is stated at boot rather
    // than left to be inferred from a `tps` an operator would have to know the band for.
    {
        let mut log = BufWriter::new();
        let _ = write!(
            log,
            "rxstamp psc={} st=0x{:02X} tps={} kind={} clk={} hse={}",
            cap.psc,
            cap.self_test.bits(),
            cap.self_test.ticks_per_ms,
            capture::stamp_kind_byte(rxstamp::hw_stamp_live()),
            if rxstamp::clock_is_capture_timer() {
                "tim3"
            } else {
                "systick"
            },
            // 1 = an 8 MHz crystal is fitted and started, and the core is running FROM it;
            // 0 = none answered and the core stayed on the HSI RC.
            hse_present as u8,
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
    // Hardware-RX-stamp outcomes, per frame. See `capture::Tally` and the EVT_STATS v3 tail.
    let mut stamps = capture::Tally::default();
    // Default OFF: a fresh/reset dongle stays quiet (no stray beacon before a host attaches). Opt in
    // on-air discovery with CMD_SET_BEACON[1] (or the host's LoraParams.beacon = true).
    // One-deep scheduled-TX queue (CMD_TX_AT). Idle until the host arms it, so a node that never
    // schedules behaves exactly as before.
    let mut sched = Sched::new();
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
                    &mut phy,
                    &mut beacon_enabled,
                    &mut beacon_period,
                    &mut csma,
                    &mut plane,
                    &mut rx_trunc,
                    &mut stamps,
                    &mut sched,
                );
                // A single command can cost 80+ ms (any SET_* re-arms RX and pays a TCXO startup), so
                // service the deadline INSIDE the drain loop: a burst of queued commands must not sit
                // between a scheduled frame and the instant it was promised.
                sched_service(&mut radio, &mut tx, &mut sched, phy);
            }
        }

        // 1b) Advance the scheduled TX. Returns immediately unless a deadline is close.
        sched_service(&mut radio, &mut tx, &mut sched, phy);

        // 2) Classify a received frame by NAME (data-centric offload). With no host-installed filter /
        //    CS / relay it always Delivers — identical to the plain modem; features light up on opt-in.
        // While a scheduled frame is `Staged` the chip sits in STDBY_XOSC with the payload loaded —
        // it is not receiving, and an SPI status read here would only add jitter ahead of the key-up.
        // While merely `Armed` the radio is still in RX and this runs exactly as usual.
        if sched.state != SchedState::Staged {
            if let Some(pkt) = radio.poll_rx(&mut rxbuf) {
                // ★ **The timestamp.** `pkt.stamp` was latched in silicon at the DIO1 edge and read
                // inside `poll_rx` while the chip's IRQ status still said `RxDone`, so it is
                // attributable to THIS frame or it is nothing. When it is nothing we fall back to
                // reading the clock here — the same TIM3 counter, just microseconds-to-milliseconds
                // later — and the frame is announced as software-stamped rather than passed off as
                // a capture. There is no third option: a plausible timestamp on the wrong frame is
                // the worst failure a measurement instrument has.
                let ts = match pkt.stamp.ticks() {
                    Some(t) => t as u32,
                    None => micros(),
                };
                stamps.observe(pkt.stamp);
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
                    // A degraded stamp is announced for THIS frame, immediately before it. See
                    // [`send_rx_stamp_note`].
                    send_rx_stamp_note(
                        |b| {
                            let _ = block!(tx.write(b));
                        },
                        pkt.stamp,
                    );
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
        }

        // 3) Optional heartbeat beacon (host-toggleable via CMD_SET_BEACON) so TX is exercised and
        //    the node is discoverable on-air without a host driving it.
        if beacon_enabled && !sched.pending() {
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
/// **In GFSK there is no CAD** — `SetCad` is a LoRa-modem function — so the CAD half is skipped
/// entirely (issuing it there would earn a command error and be read back as "channel clear"), and
/// the energy detector carries the whole sense. That makes the threshold load-bearing rather than
/// optional in that PHY, which is what [`effective_rssi_thresh`] supplies when the host has set
/// none. Same counter, same contract, one less mechanism.
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
    let has_cad = radio.supports_cad();
    let mut busy = false;
    if has_cad {
        for _ in 0..csma.cad_repeat.max(1) {
            if radio.do_cad() {
                busy = true;
                break;
            }
        }
    }
    let thresh = effective_rssi_thresh(csma.rssi_thresh, has_cad);
    if !busy && thresh > i16::MIN {
        busy = radio.rssi_busy(thresh);
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
    if radio.supports_cad() {
        radio.set_cad_params(csma.cad_sym, csma.cad_peak, csma.cad_min);
    }
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

/// **Advance the scheduled-TX state machine by one main-loop pass.** Never blocks for the scheduled
/// delay — that is the whole contract with the host link (see the `Sched` block for why).
///
/// * `Armed`   — the radio is still in RX and the node behaves exactly as if nothing were pending.
///               Once the deadline is within `lead_us`, stage the frame (a ~2.7 ms SPI burst).
/// * `Staged`  — the frame is in the chip and only `SetTx` remains. Poll until the deadline is
///               within `SCHED_GATE_US`, then hand the remainder to TIM2 and release on its compare.
///
/// A deadline that is already in the past at either step is not an error: the frame goes at the
/// first opportunity and the lateness is measured and reported, which is also what `delay_us = 0`
/// means ("now"). Silently pretending it was on time is the one thing that would make the number
/// EVT_CAP publishes a lie.
///
/// No LBT here, deliberately: a listen-before-talk backoff would move the transmission off the
/// instant that was asked for, and a scheduled TX exists precisely because the caller has already
/// decided when the air is theirs.
#[allow(clippy::too_many_arguments)]
fn sched_service<SPI, NSS, RST, BSY, DIO1, RFSW, E, TX>(
    radio: &mut Sx1262<SPI, NSS, RST, BSY, DIO1, RFSW>,
    tx: &mut TX,
    sched: &mut Sched,
    phy: u8,
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
    match sched.state {
        SchedState::Idle => {}
        SchedState::Armed => {
            let now = micros64();
            if sched.deadline.saturating_sub(now) > sched.lead_us as u64 {
                return; // plenty of time; keep receiving and keep draining the host link
            }
            let t0 = micros64();
            let settled = radio.stage_tx(&sched.buf[..sched.len]);
            let t1 = micros64();
            sched.observe_stage(us_since(t0, t1));
            if !settled && debug_on() {
                let mut lg = BufWriter::new();
                let _ = write!(lg, "sched: BUSY did not settle after staging");
                send_frame(
                    |b| {
                        let _ = block!(tx.write(b));
                    },
                    EVT_LOG,
                    lg.as_slice(),
                );
            }
            // ★ `EVT_TX_STARTED` goes out HERE — at the staging point, `lead_us` (8 ms nominal,
            // 200 ms ceiling) before key-up — for the same reason the Heltec emits from its own
            // staging point, and NOT from either of the two obvious alternatives:
            //
            //  * NOT between the gate and `tx_issue`. `send_frame` blocks on the USART until the
            //    bytes are out, which at 115200 8N1 is ~87 us per byte — an unbounded term against
            //    the 400 us this node declares as `sched_gran_ns`, i.e. the precise lie that field
            //    exists to prevent.
            //  * NOT at acceptance. The host REPLACES its reply deadline with
            //    `now + airtime + AIRTIME_SLACK` (2 s) the moment it sees this event, so announcing
            //    at acceptance would make every schedule longer than ~2 s time out at the host —
            //    and this node accepts up to `SCHED_MAX_DELAY_US` (60 s).
            //
            // From here it is out of the timing path and at most `SCHED_LEAD_MAX_US` (200 ms)
            // before key-up, so the host's re-based deadline is still correct.
            send_frame(
                |b| {
                    let _ = block!(tx.write(b));
                },
                EVT_TX_STARTED,
                &sched.air_ms.to_be_bytes(),
            );
            sched.state = SchedState::Staged;
        }
        SchedState::Staged => {
            let now = micros64();
            let rem = sched.deadline.saturating_sub(now);
            if rem > SCHED_GATE_US as u64 {
                return; // still waiting; the loop keeps draining RING
            }
            // Hand the last stretch to the hardware compare. `rem == 0` means the deadline has
            // already passed (a long command ran, or the host asked for `delay_us = 0`) — release
            // immediately and let `late_us` report it.
            if rem > 0 {
                SchedTimer::arm(rem as u16);
                let mut spin = 0u32;
                while !SchedTimer::expired() {
                    spin += 1;
                    if spin > SCHED_GATE_SPIN_MAX {
                        break; // dead timer: release now and let `late_us` say how late
                    }
                }
            }
            let t_issue = micros64();
            radio.tx_issue();
            // The chip's own key-up indicator: BUSY is high while SetTx is processed and drops when
            // the transmitter is running. Timing it here is what turns `sched_gran_ns` from a
            // derivation into a measurement — and it is exactly where a TCXO restart would show up
            // as ~78 ms if `stage_tx`'s STDBY_XOSC ever failed to keep the crystal alive.
            let mut t_up = t_issue;
            while radio.busy_high() {
                t_up = micros64();
                if t_up.saturating_sub(t_issue) > SCHED_KEYUP_TIMEOUT_US {
                    break;
                }
            }
            // If BUSY had not risen yet on the first read this under-measures — which is harmless,
            // because `Sched::gran_ns` floors the published granularity at the derived
            // [`SCHED_GRAN_NS`] and only ever raises it. The case that matters, a 78 ms TCXO
            // restart, cannot be missed: BUSY is high for the whole of it.
            let keyup_us = us_since(t_issue, micros64().max(t_up));
            let late_us = us_since(sched.deadline, t_issue);
            let ok = radio.wait_txdone();
            radio.start_rx();
            sched.observe_keyup(keyup_us, late_us);
            sched.state = SchedState::Idle;
            // EVT_TXDONE with the Waveshare-local scheduling tail. `attempts` is 0 because a
            // scheduled TX makes exactly one, by definition.
            let mut p = [0u8; 10];
            p[0] = ok as u8;
            p[1] = 0;
            p[2..6].copy_from_slice(&late_us.to_be_bytes());
            p[6..10].copy_from_slice(&keyup_us.to_be_bytes());
            send_frame(
                |b| {
                    let _ = block!(tx.write(b));
                },
                EVT_TXDONE,
                &p,
            );
            // ★ B3: the measurement that just happened may have moved `sched_gran_ns`. The host read
            // EVT_CAP once, at open, and will never ask again — so if the refined figure is
            // materially different from what it holds, say so unsolicited. Emitted HERE, after the
            // EVT_TXDONE and with the radio already back in RX, so it is outside every timing path:
            // `send_frame` blocks on the USART (~87 us/byte), which is exactly why it must never sit
            // between the gate and the key-up.
            let g = sched.gran_ns();
            let now = micros64();
            if sched.cap_republish_due(g, now) {
                send_cap(
                    |b| {
                        let _ = block!(tx.write(b));
                    },
                    g,
                    phy,
                );
                sched.note_cap_published(g, now);
            }
        }
    }
}

/// **Airtime for the CURRENT PHY**, in whole milliseconds — the number EVT_TX_STARTED carries and
/// the host re-bases its reply deadline on.
///
/// The two modems do not share a model: LoRa's is symbol arithmetic over SF/BW/CR, GFSK's is a bit
/// count over a fixed bitrate. Reporting the LoRa figure for a GFSK frame would overstate a 32-byte
/// transmission by roughly 8x (≈60 ms at SF7/BW125 against 7.4 ms at 50 kbps) — which is not a
/// cosmetic error, because a lease or a slot plan sized from it would reserve airtime nobody uses.
fn airtime_ms_for(phy: u8, sf: u8, bw: u8, cr: u8, len: u8, preamble: u16) -> u32 {
    if phy == wire_phy::LORA {
        sx1262::airtime_ms(sf, bw, cr, len, preamble)
    } else {
        sx1262::gfsk_airtime_ms(len, preamble)
    }
}

/// Elapsed microseconds from `a` to `b`, saturating at 0 and at `u32::MAX`. The clock is monotonic
/// 64-bit, so `b < a` only happens where the "elapsed" is genuinely zero (a deadline already past).
fn us_since(a: u64, b: u64) -> u32 {
    b.saturating_sub(a).min(u32::MAX as u64) as u32
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
    phy: &mut u8,
    beacon_enabled: &mut bool,
    beacon_period: &mut u32,
    csma: &mut Csma,
    plane: &mut ndn::DataPlane,
    rx_trunc: &mut u16,
    stamps: &mut capture::Tally,
    sched: &mut Sched,
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
            let air = airtime_ms_for(*phy, *sf, *bw, *cr, len as u8, csma.preamble);
            let air16 = (air.min(u16::MAX as u32) as u16).to_be_bytes();
            send_frame(&mut put, EVT_TX_STARTED, &air16);
            let (sent, attempt) = lbt_tx(radio, csma, &buf[..len]);
            radio.start_rx();
            send_frame(&mut put, EVT_TXDONE, &[sent as u8, attempt]);
        }
        // #52: one sense at the current modulation → busy/clear (sensing, not the access loop).
        // Routed through `sense_busy` so it counts into `activity` like every other observation, and
        // so a host-driven CAD honours `cad_repeat`/`rssi_thresh` exactly as the LBT loop does.
        // `SetCad` correlates against a LoRa preamble; the GFSK modem has no such function, and the
        // chip would answer a command error that reads back as "channel clear". Refused by name.
        // (`CMD_SENSE` still works in GFSK — it falls back to the energy detector, which is real
        // there; what does not exist is *this* mechanism, so this is the opcode that goes away.)
        CMD_CAD if *phy != wire_phy::LORA => {
            send_frame(&mut put, EVT_UNSUPPORTED, &[CMD_CAD, UNSUP_NO_HARDWARE]);
        }
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
        // A sweep of SF7..12 by CAD needs both a spreading factor and a CAD. GFSK has neither.
        CMD_SF_SCAN if *phy != wire_phy::LORA => {
            send_frame(&mut put, EVT_UNSUPPORTED, &[CMD_SF_SCAN, UNSUP_NO_HARDWARE]);
        }
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
            // Returns whether an image calibration was actually needed. Inside 902-928 MHz — the only
            // band this arm accepts — it never is after `init`, which is what took the measured
            // retune from 161 ms (two TCXO startups) to one. See `sx1262::set_frequency`.
            let recalibrated = radio.set_frequency(*freq);
            radio.start_rx();
            if debug_on() {
                let mut lg = BufWriter::new();
                let _ = write!(lg, "retune {} recal={}", *freq, recalibrated);
                send_frame(&mut put, EVT_LOG, lg.as_slice());
            }
            send_info(
                &mut put,
                radio,
                *phy,
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
        // `[sf, bw_code, cr_code]` names three LoRa concepts. GFSK has none of them, and this node
        // deliberately does NOT reinterpret the three bytes as a bitrate/deviation/bandwidth triple:
        // overloading a fleet-wide opcode with node-local meaning is how a shared contract stops
        // being shared. This firmware runs ONE GFSK profile (50 kbps / 25 kHz / 117.3 kHz, the
        // LoRaWAN FSK point — see `sx1262`'s GFSK constants); a GFSK modulation knob needs its own
        // fleet opcode assignment, not this one's bytes.
        CMD_SET_MOD if *phy != wire_phy::LORA => {
            send_frame(&mut put, EVT_UNSUPPORTED, &[CMD_SET_MOD, UNSUP_NO_HARDWARE]);
        }
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
                *phy,
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
                *phy,
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
                *phy,
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
        // Storing CAD detector parameters in a PHY with no CAD would be a knob that accepts a value
        // and actuates nothing — the exact failure mode this fleet keeps finding. Refused.
        CMD_SET_CAD_CFG if *phy != wire_phy::LORA => {
            send_frame(&mut put, EVT_UNSUPPORTED, &[CMD_SET_CAD_CFG, UNSUP_NO_HARDWARE]);
        }
        CMD_SET_CAD_CFG if len >= 3 => {
            csma.cad_sym = buf[0];
            csma.cad_peak = buf[1];
            csma.cad_min = buf[2];
            send_info(
                &mut put,
                radio,
                *phy,
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
                *phy,
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
                *phy,
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
                *phy,
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
                *phy,
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
                *phy,
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
                *phy,
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
                *phy,
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
        // EVT_STATS v3 = 44 bytes. Bytes 0..24 are byte-identical to v1 and 0..32 to v2; the host
        // parser checks `len < 24` (not `!= 24`), reads the v2 tail only at >= 32, and accepts a
        // longer reply while ignoring the excess. So v1, v2 and v3 hosts all read this correctly.
        CMD_GET_STATS => {
            let chip = radio.get_stats();
            let mut p = [0u8; stats::LEN];
            p[stats::RX..stats::RX + 4].copy_from_slice(&plane.rx.to_be_bytes());
            p[stats::FILTERED..stats::FILTERED + 4].copy_from_slice(&plane.filtered.to_be_bytes());
            p[stats::DEDUPED..stats::DEDUPED + 4].copy_from_slice(&plane.deduped.to_be_bytes());
            p[stats::SERVED..stats::SERVED + 4].copy_from_slice(&plane.served.to_be_bytes());
            p[stats::RELAYED..stats::RELAYED + 4].copy_from_slice(&plane.relayed.to_be_bytes());
            p[stats::CAD_BUSY..stats::CAD_BUSY + 2]
                .copy_from_slice(&csma.cad_busy_view().to_be_bytes());
            p[stats::DEFER..stats::DEFER + 2].copy_from_slice(&csma.defer.to_be_bytes());
            // --- v2 tail: the SX126x's OWN counters (P5). A CRC failure used to vanish inside
            // `poll_rx` with nothing counting it, so RX loss was invisible; chip_crc_err is now the
            // difference between "quiet channel" and "channel we are failing to decode".
            p[stats::CHIP_RX..stats::CHIP_RX + 2].copy_from_slice(&chip.pkt_received.to_be_bytes());
            p[stats::CHIP_CRC_ERR..stats::CHIP_CRC_ERR + 2]
                .copy_from_slice(&chip.pkt_crc_error.to_be_bytes());
            p[stats::CHIP_HDR_ERR..stats::CHIP_HDR_ERR + 2]
                .copy_from_slice(&chip.pkt_header_error.to_be_bytes());
            // Frames whose on-air length exceeded RX_MAX and were therefore truncated. Should stay 0
            // — a non-zero value means a peer is transmitting past our advertised max_payload.
            p[stats::RX_TRUNC..stats::RX_TRUNC + 2].copy_from_slice(&rx_trunc.to_be_bytes());
            // --- v3 tail: the evidence for `stamp_kind = 3`. Without these a stamp that quietly
            // fell back to the software read would look exactly like one that did not, and the
            // capability byte would be a claim with nothing behind it.
            let (over, lat) = rxstamp::counters();
            p[stats::HW_STAMPED..stats::HW_STAMPED + 2].copy_from_slice(&stamps.hw.to_be_bytes());
            p[stats::HW_STAMP_SW..stats::HW_STAMP_SW + 2].copy_from_slice(&stamps.sw.to_be_bytes());
            p[stats::HW_STAMP_AMBIG..stats::HW_STAMP_AMBIG + 2]
                .copy_from_slice(&stamps.ambig.to_be_bytes());
            p[stats::HW_STAMP_OVER..stats::HW_STAMP_OVER + 2]
                .copy_from_slice(&(over.min(u16::MAX as u32) as u16).to_be_bytes());
            p[stats::HW_STAMP_LAT_US..stats::HW_STAMP_LAT_US + 2]
                .copy_from_slice(&(lat.min(u16::MAX as u32) as u16).to_be_bytes());
            // The two clocks compared. Same oscillator, so this measures neither of them — it
            // catches a LOST TIM3 OVERFLOW, whose signature is a jump of 65 (one 65.536 ms wrap)
            // that never comes back. Computed here rather than continuously because a 64-bit
            // division is ~15 µs at 8 MHz and this is a once-per-query diagnostic, not a hot path.
            let skew_ms = {
                let us_ms = (micros64() / 1000) as u32;
                let ms = millis();
                us_ms.abs_diff(ms).min(u16::MAX as u32) as u16
            };
            p[stats::CLOCK_SKEW_MS..stats::CLOCK_SKEW_MS + 2]
                .copy_from_slice(&skew_ms.to_be_bytes());
            send_frame(&mut put, EVT_STATS, &p);
        }
        CMD_RESET_STATS => {
            plane.reset_stats();
            // Re-baseline the resettable VIEW; `cad_busy` itself keeps free-running for EVT_SENSE.
            csma.cad_busy_base = csma.cad_busy;
            csma.defer = 0;
            *rx_trunc = 0;
            stamps.reset();
            // The latency WATERMARK is re-baselined; the overcapture count is not. `rxstamp::take`
            // reads overcaptures as a difference against the window's own baseline, and moving one
            // side of a difference under the reader is how a counter starts lying.
            rxstamp::reset_lat_watermark();
            radio.reset_stats(); // zero the chip's packet counters too
            send_info(
                &mut put,
                radio,
                *phy,
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
                *phy,
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
                *phy,
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
        // 7E-A5 v2 §P4: **scheduled TX**. `payload = [delay_us u32 BE][frame]`; the frame airs
        // `delay_us` after this command is decoded, on the `micros64()` timebase — the same counter
        // EVT_RX stamps with and CMD_READ_CLOCK returns. `delay_us = 0` means now. The reply is a
        // single EVT_TXDONE emitted when the frame actually goes (see `sched_service`), never here:
        // answering at arm time would report a transmission that has not happened.
        //
        // `len >= 5` = 4 delay bytes and at least one frame byte; a shorter payload falls through to
        // the catch-all and is answered BAD_LENGTH, because bit 24 of CMD_BITMAP is now set.
        CMD_TX_AT if len >= 5 => {
            if sched.pending() {
                // One slot, and the frame already in it was committed first. Refusing keeps the
                // 1-reply-per-command contract (ok = 0 is exactly true: this frame did not air)
                // without silently displacing something the host is already waiting on.
                send_frame(&mut put, EVT_TXDONE, &[0, 0]);
                return;
            }
            let delay_us = u32::from_be_bytes([buf[0], buf[1], buf[2], buf[3]]);
            if delay_us > SCHED_MAX_DELAY_US {
                // Bounded, not clamped — see SCHED_MAX_DELAY_US. An unbounded u32 of microseconds
                // could occupy the one-deep slot for over an hour, refusing every later
                // CMD_TX_AT with no way to clear it.
                send_frame(&mut put, EVT_UNSUPPORTED, &[CMD_TX_AT, UNSUP_OUT_OF_RANGE]);
                return;
            }
            let n = (len - 4).min(SCHED_FRAME_MAX);
            sched.buf[..n].copy_from_slice(&buf[4..4 + n]);
            sched.len = n;
            // Airtime is computed HERE, where the live modulation parameters are in scope, and
            // announced later from the staging point in `sched_service`.
            sched.air_ms = airtime_ms_for(*phy, *sf, *bw, *cr, n as u8, csma.preamble)
                .min(u16::MAX as u32) as u16;
            sched.deadline = micros64().saturating_add(delay_us as u64);
            sched.state = SchedState::Armed;
        }
        // 7E-A5 v3 §B2: **scheduled TX against an ABSOLUTE instant**.
        // `payload = [target_ticks u64 BE][frame]`, on the `micros64()` timebase — the same counter
        // EVT_RX stamps with and CMD_READ_CLOCK returns at full width. The reply is a single
        // EVT_TXDONE when the frame actually goes, exactly as for CMD_TX_AT.
        //
        // **Why this opcode exists.** CMD_TX_AT's delay starts counting when the FIRMWARE decodes
        // the arm, so everything between the host's intent and that moment — serial transmission,
        // ring drain, whatever command ran before it — lands inside the placement. That was measured
        // on the LR2021 as a 553 µs jitter sd against a 50 µs declared granularity, matching that
        // node's 550 µs command round-trip spread rather than its release mechanism. An absolute
        // target removes the term: the host names an instant on the node's own clock, and how long
        // the request took to arrive stops mattering. See [`SCHED_REL_PLACEMENT_BOUND_NS`].
        //
        // `len >= 9` = 8 target bytes and at least one frame byte; shorter falls through to the
        // catch-all and is answered BAD_LENGTH, because bit 31 of CMD_BITMAP is set.
        CMD_TX_AT_ABS if len >= 9 => {
            if sched.pending() {
                // Same one-deep contract as CMD_TX_AT: refuse rather than displace a frame the host
                // is already waiting on. `ok = 0` is exactly true — this frame did not air.
                send_frame(&mut put, EVT_TXDONE, &[0, 0]);
                return;
            }
            let target = u64::from_be_bytes([
                buf[0], buf[1], buf[2], buf[3], buf[4], buf[5], buf[6], buf[7],
            ]);
            let now = micros64();
            // "Absurd" is bounded SYMMETRICALLY at SCHED_MAX_DELAY_US (60 s). Far in the future is
            // the same hazard CMD_TX_AT guards against — a one-deep slot occupied for an hour. Far
            // in the PAST is the new one, and it is the more informative failure: the only way to be
            // a minute behind a monotonic counter the host just read is to be converting from a
            // different timebase, and firing immediately would hide that behind a frame that looks
            // like it worked. A target merely *slightly* past is not an error at all — it is a
            // deadline that slipped, and it fires at once with `late_us` reporting by how much,
            // which is also what `delay_us = 0` means on the relative opcode.
            let skew = if target > now { target - now } else { now - target };
            if skew > SCHED_MAX_DELAY_US as u64 {
                send_frame(
                    &mut put,
                    EVT_UNSUPPORTED,
                    &[CMD_TX_AT_ABS, UNSUP_OUT_OF_RANGE],
                );
                return;
            }
            let n = (len - 8).min(SCHED_ABS_FRAME_MAX);
            sched.buf[..n].copy_from_slice(&buf[8..8 + n]);
            sched.len = n;
            sched.air_ms = airtime_ms_for(*phy, *sf, *bw, *cr, n as u8, csma.preamble)
                .min(u16::MAX as u32) as u16;
            sched.deadline = target;
            sched.state = SchedState::Armed;
        }
        // 7E-A5 v3 §B1: **the PHY is a knob.** `[packet_type]` in the LR20xx WIRE numbering, which
        // is not the SX126x's — `wire_to_chip_phy` is the only place the two meet and the mapping is
        // asserted at build time.
        //
        // The reply is the WHOLE new EVT_CAP, because `max_payload`, the SF span and the usable
        // command set are all properties of the mode rather than of the node: the host replaces its
        // profile instead of patching fields.
        CMD_SET_PHY if len >= 1 => {
            let want = buf[0];
            // Only PHYs this node actually brings up. LR-FHSS is the interesting refusal: the
            // SX1262 silicon has it, but transmit-only and with its own internal hop sequence, so it
            // is not something a bidirectional bearer can advertise.
            if want >= 32 || (PHY_BITMAP >> want) & 1 == 0 {
                send_frame(
                    &mut put,
                    EVT_UNSUPPORTED,
                    &[CMD_SET_PHY, UNSUP_OUT_OF_RANGE],
                );
                return;
            }
            // A queued scheduled frame was accepted under the OLD medium — its airtime was computed
            // there and the host is holding a re-based deadline from it. Rather than air it on a
            // modulation nobody asked for, cancel it and close its contract the same way a second
            // CMD_TX_AT is closed: EVT_TXDONE[0, 0], where `ok = 0` is exactly true.
            if sched.pending() {
                sched.state = SchedState::Idle;
                send_frame(&mut put, EVT_TXDONE, &[0, 0]);
            }
            let prev = *phy;
            let status = radio.apply_phy(
                wire_to_chip_phy(want),
                *freq,
                *sf,
                *bw,
                *cr,
                *pwr,
            );
            if !sx1262::cmd_status_ok(status) {
                // The node advertised this PHY and the silicon declined it. Put the radio back where
                // it was and report the chip's LITERAL status byte — a firmware opinion about what
                // went wrong would be worth less than the byte itself.
                radio.apply_phy(wire_to_chip_phy(prev), *freq, *sf, *bw, *cr, *pwr);
                radio.start_rx();
                send_frame(&mut put, EVT_PHY_ERR, &[want, status]);
                return;
            }
            *phy = want;
            radio.start_rx();
            let g = sched.gran_ns();
            send_cap(&mut put, g, *phy);
            sched.note_cap_published(g, micros64());
        }
        // 7E-A5 v3 §B4: **an explicit refusal, not a fall-through.**
        //
        // `CMD_SET_HOP` installs an intra-packet frequency-hopping table. The SX126x has no such
        // engine: unlike the SX127x — which has `RegHopPeriod` and an `FhssChangeChannel` interrupt
        // that steps a host-supplied table mid-frame — the SX126x's only hopping is inside the
        // LR-FHSS packet type, which is transmit-only and builds its own sequence rather than taking
        // one. So this node is STRUCTURALLY excluded from the hopping pair, and that exclusion is
        // written here as its own arm with `NO_HARDWARE` so it reads as a decision rather than as an
        // opcode nobody got round to. (`CMD_DATAPLANE`'s `hop_*` fields are a different mechanism
        // entirely — a name-keyed choice of which channel to sit on BETWEEN frames, which needs no
        // hardware sequencer.)
        //
        // Bit 30 of CMD_BITMAP stays CLEAR: the bitmap means "implemented and will act", and this
        // will not. `REFUSED_OPCODES` asserts the two cannot drift.
        CMD_SET_HOP => {
            send_frame(&mut put, EVT_UNSUPPORTED, &[CMD_SET_HOP, UNSUP_NO_HARDWARE]);
        }
        // 7E-A5 v2 §P1 / v3 §B1: the one place this node describes itself — for the PHY it is in.
        CMD_GET_CAP => {
            let g = sched.gran_ns();
            send_cap(&mut put, g, *phy);
            // Remember what the host now holds, so the unsolicited re-publish (B3) measures a
            // refined granularity against the figure actually delivered.
            sched.note_cap_published(g, micros64());
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
            if radio.supports_cad() {
                radio.set_cad_params(csma.cad_sym, csma.cad_peak, csma.cad_min);
            }
            sense_busy(radio, csma);
            radio.start_rx();
            // RSSI after RX is re-armed: `rssi_inst` outside an RX mode is not a channel measurement.
            let r = radio.rssi_inst();
            let mut p = [0u8; 4];
            p[0..2].copy_from_slice(&csma.cad_busy.to_be_bytes());
            p[2..4].copy_from_slice(&r.to_be_bytes());
            send_frame(&mut put, EVT_SENSE, &p);
        }
        // Waveshare-local (P6): pick the LNA gain. `[0]` = the chip's power-saving power-on default,
        // `[1]` = boosted (this firmware's default, ~+3 dB sensitivity for ~+2 mA in RX).
        //
        // **Payload shape, for the two firmwares mirroring this opcode:** exactly one byte, and only
        // 0 or 1 are defined. Anything else is OUT_OF_RANGE rather than folded into "boosted" — a
        // host that sends 2 meaning something has been misunderstood, and a knob that quietly
        // reinterprets its argument is how a capability statement stops being true.
        CMD_SET_RX_GAIN if len >= 1 => {
            if buf[0] > 1 {
                send_frame(
                    &mut put,
                    EVT_UNSUPPORTED,
                    &[CMD_SET_RX_GAIN, UNSUP_OUT_OF_RANGE],
                );
                return;
            }
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
                *phy,
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

/// **EVT_CAP — the one place this node describes itself** (7E-A5 **v3**). 34 bytes, every
/// multi-byte field big-endian. Every value below comes from a source constant or a verified
/// property of this firmware; where nothing is known the field is 0 and says so, because a
/// fabricated number is worse than 0 — the host believes it.
///
/// ★ **This record describes the CURRENT PHY, not the node.** `max_payload`, `sf_min`/`sf_max` and
/// `cmd_bitmap` all move with `phy_current` — the same SX1262 in GFSK has no spreading factor, no
/// CAD and a different airtime model. So `CMD_SET_PHY` replies with the WHOLE record and the host
/// replaces its profile; nothing here is safe to patch field-by-field.
///
/// **Two v2 fields changed meaning, both compatibly.** `proto_ver` is 3, and `radio_kind` now names
/// the PART (0 = SX1262, 1 = SX1276, 2 = LR2021) rather than a part-and-mode pair — for this node
/// the byte is unchanged at 0, so a v2 host reading a v3 SX1262 still gets the right answer, reads
/// the 29 bytes it knows and ignores the 5-byte tail.
///
/// ```text
///  [0]      proto_ver     = 3
///  [1]      radio_kind    = 0 (the PART: SX1262)
///  [2..6]   freq_min_hz   = 902_000_000   } the band this firmware image-calibrates for; NOT the
///  [6..10]  freq_max_hz   = 928_000_000   } SX1262's 150-960 MHz silicon range (sx1262::FREQ_*_HZ)
///  [10]     pwr_min_dbm   = -9            } real dBm, the SetTxParams range `set_power` clamps to
///  [11]     pwr_max_dbm   = 22            } (sx1262::PWR_*_DBM) — not a chip register unit
///  [12..16] stamp_hz      = 1_000_000     MICROseconds; see STAMP_HZ / `micros64`
///  [16]     stamp_kind    = 3 (hardware free-running — TIM3_CH3 latches the counter at the SX1262's
///                              DIO1 edge; see [`rxstamp`]) **IF the boot self-test passed**,
///                              otherwise 2. The byte is [`capture::stamp_kind_byte`] of
///                              [`rxstamp::hw_stamp_live`], never a literal
///  [17..19] max_payload   = 247           PER-PHY. Both PHYs land on RX_MAX because the binding
///                                         limit is the serial framing in both; the radio caps
///                                         (255 B LoRa PDU, 255 B GFSK PDU) are larger
///  [19..23] cmd_bitmap    PER-PHY         CMD_BITMAP in LoRa; in GFSK the four LoRa-modem opcodes
///                                         (SET_MOD/CAD/SET_CAD_CFG/SF_SCAN) are cleared
///  [23]     sf_min        PER-PHY         LoRa 7 / GFSK 0 } sx1262::SF_MIN/SF_MAX in LoRa — the SFs
///  [24]     sf_max        PER-PHY         LoRa 12 / GFSK 0} this firmware operates and SF_SCAN
///                                         sweeps. 0/0 in GFSK is "this PHY has no such knob"
///  [25..29] sched_gran_ns = 400_000       Both CMD_TX_AT_ABS (0x1F) and CMD_TX_AT (0x18) are
///                                         implemented: the MCU timer releases the key-up. The
///                                         figure is derived term by term in [`SCHED_GRAN_NS`],
///                                         describes the ABSOLUTE path (see
///                                         [`SCHED_REL_PLACEMENT_BOUND_NS`] for what the relative
///                                         one adds), and is RAISED to the measured key-up once this
///                                         node has released a scheduled frame ([`Sched::gran_ns`])
///  [29..33] phy_bitmap    = 0x0000_0005   v3 tail: bit N ⇔ wire SetPacketType value N is usable.
///                                         Bit 0 LoRa + bit 2 FSK ([`PHY_BITMAP`])
///  [33]     phy_current                   v3 tail: the wire SetPacketType value in effect now
/// ```
fn send_cap<F: FnMut(u8)>(put: F, sched_gran_ns: u32, phy: u8) {
    let caps = phy_caps(phy);
    let mut c = [0u8; CAP_LEN];
    c[0] = 3; // proto_ver
    c[1] = 0; // radio_kind: the PART — SX1262
    c[2..6].copy_from_slice(&sx1262::FREQ_MIN_HZ.to_be_bytes());
    c[6..10].copy_from_slice(&sx1262::FREQ_MAX_HZ.to_be_bytes());
    c[10] = sx1262::PWR_MIN_DBM as u8;
    c[11] = sx1262::PWR_MAX_DBM as u8;
    // ★ The timestamp pair, written together by [`capture::encode_stamp_fields`] rather than
    // indexed by hand, because they must agree: the host's common-view predicate is
    // `stamp_kind == HardwareFreeRun && stamp_hz > 0`, so a hardware kind with a zero rate would
    // advertise a capability and then fail the predicate with nothing saying why.
    //
    // **The kind is a measurement, not a constant.** `rxstamp::hw_stamp_live` is the boot
    // self-test's verdict — it proved on this silicon that a capture raises CC3IF, that reading
    // CCR3 clears it, that a second capture raises CC3OF, that a write clears that, and that the
    // counter advances at the declared rate. A `3` makes the host publish
    // `LatchPoint::RadioCapture`, a 1 µs `stamp_precision_ns` in place of the 1 ms host-receive
    // floor, and `can_common_view = true` — a 1000x tightening of a number the timekeeper acts on.
    // That is not a claim to make on the strength of the code having compiled, and a GD32 that
    // diverges from the STM32 flag semantics says so here instead of silently stamping garbage.
    //
    // `stamp_hz` stays 1 MHz either way, and that is now a fact rather than an excuse. It used to
    // rest on "when the self-test fails it is the LATCH POINT that got worse, not the rate" — true
    // of the four flag checks and FALSE of the fifth, which is the one that fires precisely when the
    // rate is wrong. A 2 MHz TIM3 would have been detected, dropped this byte to 2, and gone on
    // being the node's microsecond clock at a declared 1 MHz. `capture::choose_timebase` closes it:
    // the capture timer is only ever `micros64()` when its rate was MEASURED at STAMP_HZ, and
    // otherwise the clock is SysTick, which counts microseconds by construction. So whichever
    // counter is live, this field describes it. Zeroing it would tell the host there is no per-frame
    // stamp at all, which is a different and false claim.
    capture::encode_stamp_fields(
        &mut c,
        STAMP_HZ,
        capture::stamp_kind_byte(rxstamp::hw_stamp_live()),
    );
    c[17..19].copy_from_slice(&caps.max_payload.to_be_bytes());
    c[19..23].copy_from_slice(&caps.cmd_bitmap.to_be_bytes());
    c[23] = caps.sf_min;
    c[24] = caps.sf_max;
    c[25..29].copy_from_slice(&sched_gran_ns.to_be_bytes());
    // --- v3 tail ---
    c[CAP_PHY_BITMAP_OFF..CAP_PHY_BITMAP_OFF + 4].copy_from_slice(&PHY_BITMAP.to_be_bytes());
    c[CAP_PHY_CURRENT_OFF] = phy;
    send_frame(put, EVT_CAP, &c);
}

/// EVT_CAP layout constants. The record is fixed-width and the host slices fields by offset, so the
/// offsets are named once, indexed with by the emitter above, and transcribed by the README's table.
///
/// v2's 29 bytes are unchanged in position and (except `proto_ver` and `radio_kind`'s *meaning*) in
/// value, so a v2 host reads a v3 node correctly and ignores the tail. That back-compat is the
/// reason the new fields are appended rather than interleaved.
const CAP_LEN_V2: usize = 29;
const CAP_SCHED_GRAN_OFF: usize = 25;
const CAP_PHY_BITMAP_OFF: usize = 29;
const CAP_PHY_CURRENT_OFF: usize = 33;
const CAP_LEN: usize = 34;
const _: () = assert!(CAP_SCHED_GRAN_OFF + 4 == CAP_LEN_V2);
const _: () = assert!(CAP_PHY_BITMAP_OFF == CAP_LEN_V2);
const _: () = assert!(CAP_PHY_CURRENT_OFF == CAP_PHY_BITMAP_OFF + 4);
const _: () = assert!(CAP_LEN == CAP_PHY_CURRENT_OFF + 1);
const _: () = assert!(CAP_LEN == 34);
// The whole record must still fit the framing's one-byte `len`, with room for the sync/type/crc.
const _: () = assert!(CAP_LEN <= 255);
// ★ The two fields `capture::encode_stamp_fields` writes are the only ones this file does not index
// by hand, so they were also the only ones not tied to the record by an assertion: a reorder here
// would have silently written the stamp rate over `pwr_max_dbm`/`max_payload` with nothing failing.
// Pinned to the same layout the emitter above lays out, and to the host's `p[16]` slice.
const _: () = assert!(capture::CAP_STAMP_HZ_OFF == 12);
const _: () = assert!(capture::CAP_STAMP_KIND_OFF == 16);
const _: () = assert!(capture::CAP_STAMP_KIND_OFF == capture::CAP_STAMP_HZ_OFF + 4);
const _: () = assert!(capture::CAP_STAMP_KIND_OFF < CAP_LEN_V2);

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
///
/// **The `sf`/`bw`/`cr` bytes are PHY-dependent and report 0 in GFSK**, matching the `sf_min`/
/// `sf_max` of 0 that EVT_CAP advertises there. The caller's mirrors keep their LoRa values so a
/// switch back restores them; what would be wrong is putting `sf = 7` on the wire while the radio is
/// running a modem that has no spreading factor at all. The record stays 19 bytes — it is frozen,
/// the host reads `cad_busy`/`defer` as the last four — so `phy` is reported by EVT_CAP, not here.
fn send_info<SPI, NSS, RST, BSY, DIO1, RFSW, E, F>(
    put: F,
    radio: &mut Sx1262<SPI, NSS, RST, BSY, DIO1, RFSW>,
    phy: u8,
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
    // Read back from whichever register file the CURRENT PHY uses (LoRa 0x0740 / GFSK 0x06C0), so
    // this stays a genuine read-back rather than a read of the other modem's leftovers.
    let sync = radio.read_sync();
    let errors = radio.get_device_errors();
    let f = freq.to_be_bytes();
    let lora = phy == wire_phy::LORA;
    let (sf, bw, cr) = if lora { (sf, bw, cr) } else { (0, 0, 0) };
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
