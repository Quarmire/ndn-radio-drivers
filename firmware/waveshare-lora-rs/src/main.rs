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

#![no_std]
#![no_main]
#![allow(dead_code)]

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
/// 1 kHz SysTick → the free-running millisecond clock behind `millis()` (EVT_RX timestamps).
#[exception]
fn SysTick() {
    MILLIS.fetch_add(1, Ordering::Relaxed);
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
// Firmware -> host events.
const EVT_RX: u8 = 0x81; //    payload = [rssi i16 BE, snr i16 BE, ts_ms u32 BE, LoRa bytes]
const EVT_TXDONE: u8 = 0x82; //payload = [ok, attempts]  (attempts=0 for a plain CMD_TX)
const EVT_INFO: u8 = 0x83; //  payload = [status, sync(2), errors(2), freq(4), sf, bw, cr, pwr, lost(2), cad_busy(2), defer(2)]
const EVT_LOG: u8 = 0x84; //   payload = ascii
const EVT_CAD: u8 = 0x85; //   payload = [busy(0/1)]
const EVT_RSSI: u8 = 0x86; //  payload = [rssi i16 BE]
const EVT_SF_DETECTED: u8 = 0x87; // payload = [sf | 0 = none]
const EVT_TX_STARTED: u8 = 0x88; //  payload = [airtime_ms u16 BE] — emitted just before key-up

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
    /// Observability counters, reported in EVT_INFO (you cannot tune CSMA blind).
    cad_busy: u16,
    defer: u16,
}
impl Csma {
    fn new() -> Self {
        Self {
            preamble: 8,
            cad_sym: 0x02, // 4 symbols
            cad_peak: 0x18, // 24 — a mid default; tune on air per SF
            cad_min: 0x0A, // 10
            lbt_cw: 5, // ms
            lbt_max_backoff: 4, // window up to 5<<4 = 80 ms
            lbt_max_attempts: 6,
            rng: 0x1234_5678,
            cad_busy: 0,
            defer: 0,
        }
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

/// Free-running millisecond clock (SysTick ISR), for the EVT_RX hardware timestamp.
static MILLIS: AtomicU32 = AtomicU32::new(0);
fn millis() -> u32 {
    MILLIS.load(Ordering::Relaxed)
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
        Self { buf: [0; 64], pos: 0 }
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
        Self { state: 0, typ: 0, len: 0, idx: 0, crc: 0, buf: [0; 255] }
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
    radio.start_rx();

    // Announce readiness (ascii log the host can print, plus a structured INFO).
    {
        let mut log = BufWriter::new();
        let _ = write!(
            log,
            "waveshare-lora-rs stage4: init sync=0x{:04X} err=0x{:04X}",
            diag.sync_readback, diag.device_errors
        );
        send_frame(|b| { let _ = block!(tx.write(b)); }, EVT_LOG, log.as_slice());
    }

    let mut parser = Parser::new();
    let mut rxbuf = [0u8; 64];
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
                );
            }
        }

        // 2) Deliver any received LoRa frame with RSSI/SNR + a hardware arrival timestamp (#52).
        if let Some(pkt) = radio.poll_rx(&mut rxbuf) {
            let ts = millis();
            let n = core::cmp::min(pkt.len as usize, rxbuf.len());
            let mut ev = [0u8; 72];
            ev[0..2].copy_from_slice(&pkt.rssi_dbm.to_be_bytes());
            ev[2..4].copy_from_slice(&pkt.snr_db.to_be_bytes());
            ev[4..8].copy_from_slice(&ts.to_be_bytes());
            ev[8..8 + n].copy_from_slice(&rxbuf[..n]);
            send_frame(|b| { let _ = block!(tx.write(b)); }, EVT_RX, &ev[..8 + n]);
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
                send_frame(|b| { let _ = block!(tx.write(b)); }, EVT_TXDONE, &[ok as u8]);
                beacon_seq = beacon_seq.wrapping_add(1);
            }
        } else {
            beacon_ctr = 0;
        }
    }
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
            radio.standby();
            radio.set_cad_params(csma.cad_sym, csma.cad_peak, csma.cad_min);
            let mut attempt: u8 = 0;
            let mut sent = false;
            while attempt < csma.lbt_max_attempts {
                if !radio.do_cad() {
                    sent = radio.transmit(&buf[..len]);
                    break;
                }
                csma.cad_busy = csma.cad_busy.wrapping_add(1);
                let shift = (attempt as u32).min(csma.lbt_max_backoff as u32);
                let window = (csma.lbt_cw << shift).max(1);
                let wait_ms = csma.next_rand() % window;
                cortex_m::asm::delay(wait_ms.saturating_mul(8_000)); // 8 MHz core → 8000 cycles/ms
                attempt += 1;
            }
            if !sent {
                csma.defer = csma.defer.wrapping_add(1);
            }
            radio.start_rx();
            send_frame(&mut put, EVT_TXDONE, &[sent as u8, attempt]);
        }
        // #52: one CAD at the current modulation → busy/clear (sensing, not the access loop).
        CMD_CAD => {
            radio.standby();
            radio.set_cad_params(csma.cad_sym, csma.cad_peak, csma.cad_min);
            let busy = radio.do_cad();
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
        CMD_SET_FREQ if len >= 4 => {
            *freq = u32::from_be_bytes([buf[0], buf[1], buf[2], buf[3]]);
            radio.standby();
            radio.set_frequency(*freq);
            radio.start_rx();
            send_info(&mut put, radio, *freq, *sf, *bw, *cr, *pwr, lost, csma.cad_busy, csma.defer);
        }
        CMD_SET_MOD if len >= 3 => {
            *sf = buf[0];
            *bw = buf[1];
            *cr = buf[2];
            radio.standby();
            radio.set_modulation(*sf, *bw, *cr);
            radio.start_rx();
            send_info(&mut put, radio, *freq, *sf, *bw, *cr, *pwr, lost, csma.cad_busy, csma.defer);
        }
        CMD_SET_PWR if len >= 1 => {
            *pwr = buf[0] as i8;
            radio.standby();
            radio.set_power(*pwr);
            radio.start_rx();
            send_info(&mut put, radio, *freq, *sf, *bw, *cr, *pwr, lost, csma.cad_busy, csma.defer);
        }
        CMD_SET_SYNC if len >= 1 => {
            radio.standby();
            radio.set_sync(buf[0]);
            radio.start_rx();
            send_info(&mut put, radio, *freq, *sf, *bw, *cr, *pwr, lost, csma.cad_busy, csma.defer);
        }
        // #52 Tier 2: tune CAD/LBT/preamble at runtime — so calibration never needs a reflash.
        CMD_SET_CAD_CFG if len >= 3 => {
            csma.cad_sym = buf[0];
            csma.cad_peak = buf[1];
            csma.cad_min = buf[2];
            send_info(&mut put, radio, *freq, *sf, *bw, *cr, *pwr, lost, csma.cad_busy, csma.defer);
        }
        CMD_SET_LBT_CFG if len >= 4 => {
            csma.lbt_cw = u16::from_be_bytes([buf[0], buf[1]]) as u32;
            csma.lbt_max_backoff = buf[2];
            csma.lbt_max_attempts = buf[3];
            send_info(&mut put, radio, *freq, *sf, *bw, *cr, *pwr, lost, csma.cad_busy, csma.defer);
        }
        CMD_SET_PREAMBLE if len >= 2 => {
            csma.preamble = u16::from_be_bytes([buf[0], buf[1]]).max(1);
            radio.standby();
            radio.set_preamble(csma.preamble);
            radio.start_rx();
            send_info(&mut put, radio, *freq, *sf, *bw, *cr, *pwr, lost, csma.cad_busy, csma.defer);
        }
        CMD_SET_BEACON if len >= 1 => {
            *beacon_enabled = buf[0] != 0;
            if len >= 2 {
                // Optional second byte scales the base period (min ×1).
                *beacon_period = BEACON_BASE_PERIOD.saturating_mul(buf[1].max(1) as u32);
            }
            send_info(&mut put, radio, *freq, *sf, *bw, *cr, *pwr, lost, csma.cad_busy, csma.defer);
        }
        CMD_GET_INFO => {
            send_info(&mut put, radio, *freq, *sf, *bw, *cr, *pwr, lost, csma.cad_busy, csma.defer);
        }
        _ => {}
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
