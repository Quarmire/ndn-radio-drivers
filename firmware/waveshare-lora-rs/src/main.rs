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

use core::fmt::Write as FmtWrite;
use cortex_m_rt::entry;
use embedded_hal::spi::MODE_0;
use nb::block;
use panic_halt as _;
use stm32f1xx_hal::{
    pac,
    prelude::*,
    serial::{Config, Serial},
    spi::Spi,
};

use sx1262::Sx1262;

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
// Firmware -> host events.
const EVT_RX: u8 = 0x81; //    payload = [rssi i16 BE, snr i16 BE, LoRa bytes]
const EVT_TXDONE: u8 = 0x82; //payload = [ok]
const EVT_INFO: u8 = 0x83; //  payload = [status, sync(2), errors(2), freq(4), sf, bw, cr, pwr]
const EVT_LOG: u8 = 0x84; //   payload = ascii

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

    let mut flash = dp.FLASH.constrain();
    let rcc = dp.RCC.constrain();
    let clocks = rcc.cfgr.freeze(&mut flash.acr);

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
        // 1) Drain all pending host bytes (no delay: the F1 USART has no FIFO).
        while let Ok(b) = rx.read() {
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
                );
            }
        }

        // 2) Deliver any received LoRa frame with RSSI/SNR.
        if let Some(pkt) = radio.poll_rx(&mut rxbuf) {
            let n = core::cmp::min(pkt.len as usize, rxbuf.len());
            let mut ev = [0u8; 68];
            ev[0..2].copy_from_slice(&pkt.rssi_dbm.to_be_bytes());
            ev[2..4].copy_from_slice(&pkt.snr_db.to_be_bytes());
            ev[4..4 + n].copy_from_slice(&rxbuf[..n]);
            send_frame(|b| { let _ = block!(tx.write(b)); }, EVT_RX, &ev[..4 + n]);
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
    match typ {
        CMD_TX => {
            let ok = radio.transmit(&buf[..len]);
            radio.start_rx();
            send_frame(&mut put, EVT_TXDONE, &[ok as u8]);
        }
        CMD_SET_FREQ if len >= 4 => {
            *freq = u32::from_be_bytes([buf[0], buf[1], buf[2], buf[3]]);
            radio.standby();
            radio.set_frequency(*freq);
            radio.start_rx();
            send_info(&mut put, radio, *freq, *sf, *bw, *cr, *pwr);
        }
        CMD_SET_MOD if len >= 3 => {
            *sf = buf[0];
            *bw = buf[1];
            *cr = buf[2];
            radio.standby();
            radio.set_modulation(*sf, *bw, *cr);
            radio.start_rx();
            send_info(&mut put, radio, *freq, *sf, *bw, *cr, *pwr);
        }
        CMD_SET_PWR if len >= 1 => {
            *pwr = buf[0] as i8;
            radio.standby();
            radio.set_power(*pwr);
            radio.start_rx();
            send_info(&mut put, radio, *freq, *sf, *bw, *cr, *pwr);
        }
        CMD_SET_SYNC if len >= 1 => {
            radio.standby();
            radio.set_sync(buf[0]);
            radio.start_rx();
            send_info(&mut put, radio, *freq, *sf, *bw, *cr, *pwr);
        }
        CMD_SET_BEACON if len >= 1 => {
            *beacon_enabled = buf[0] != 0;
            if len >= 2 {
                // Optional second byte scales the base period (min ×1).
                *beacon_period = BEACON_BASE_PERIOD.saturating_mul(buf[1].max(1) as u32);
            }
            send_info(&mut put, radio, *freq, *sf, *bw, *cr, *pwr);
        }
        CMD_GET_INFO => {
            send_info(&mut put, radio, *freq, *sf, *bw, *cr, *pwr);
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
    ];
    send_frame(put, EVT_INFO, &payload);
}
