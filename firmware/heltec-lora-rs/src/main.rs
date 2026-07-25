//! Heltec WiFi LoRa 32 V2 (ESP32 + SX1276) — Rust named-radio node C (task #54).
//!
//! Stage 3: a host-driven LoRa modem speaking the SAME `7E A5` serial protocol as the Waveshare
//! `waveshare-lora-rs` firmware, so the OPi host drives it with `LoraSerialBackend` unchanged — the
//! third NDN node in the N=3 LBT test alongside the two SX1262 dongles.
//!
//! lora-phy is embedded-hal-async, so this runs on the embassy executor (esp-rtos). UART0 (the CP2102
//! link) is owned RAW for the binary protocol — no esp-println in normal operation (only esp-backtrace
//! on panic), or its text would corrupt the framing. Frame shape: `7E A5 | type | len | payload |
//! xor-crc`, crc = type ^ len ^ payload. A PERSISTENT parser survives select-drops so an RX packet
//! arriving mid-frame can't desync the host link (it self-resyncs on the next 7E A5 anyway).

#![no_std]
#![no_main]

use embassy_executor::Spawner;
use embassy_futures::select::{select, Either};
use embassy_time::{Delay, Duration, Timer};
use embedded_hal_bus::spi::ExclusiveDevice;
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
use lora_phy::mod_params::{Bandwidth, CodingRate, ModulationParams, PacketParams, RxMode, SpreadingFactor};
use lora_phy::sx127x::{Config as Sx127xConfig, Sx127x, Sx1276};
use lora_phy::LoRa;

esp_bootloader_esp_idf::esp_app_desc!();

// --- serial framing (must match src/lora_serial.rs) ---
const SYNC0: u8 = 0x7E;
const SYNC1: u8 = 0xA5;
// host -> firmware
const CMD_TX: u8 = 0x01;
const CMD_SET_FREQ: u8 = 0x02;
const CMD_SET_MOD: u8 = 0x03;
const CMD_SET_PWR: u8 = 0x04;
const CMD_GET_INFO: u8 = 0x06;
const CMD_CAD: u8 = 0x08;
const CMD_GET_RSSI: u8 = 0x09;
const CMD_TX_LBT: u8 = 0x0E;
const CMD_GET_STATS: u8 = 0x13;
// firmware -> host
const EVT_RX: u8 = 0x81;
const EVT_STATS: u8 = 0x89;
const EVT_TXDONE: u8 = 0x82;
const EVT_INFO: u8 = 0x83;
const EVT_CAD: u8 = 0x85;
const EVT_RSSI: u8 = 0x86;

const MAX_LORA: u8 = 255;

/// Radio parameters the host can change at runtime.
struct Params {
    freq_hz: u32,
    sf: SpreadingFactor,
    bw: Bandwidth,
    cr: CodingRate,
    pwr: i32,
    preamble: u16,
}
impl Params {
    fn default() -> Self {
        Self {
            freq_hz: 915_000_000,
            sf: SpreadingFactor::_9,
            bw: Bandwidth::_125KHz,
            cr: CodingRate::_4_5,
            pwr: 17,
            preamble: 8,
        }
    }
    fn bw_hz(&self) -> u32 {
        match self.bw {
            Bandwidth::_125KHz => 125_000,
            Bandwidth::_250KHz => 250_000,
            Bandwidth::_500KHz => 500_000,
            _ => 125_000,
        }
    }
    fn cr_code(&self) -> u8 {
        match self.cr {
            CodingRate::_4_5 => 1,
            CodingRate::_4_6 => 2,
            CodingRate::_4_7 => 3,
            CodingRate::_4_8 => 4,
        }
    }
}

fn sf_from(n: u8) -> SpreadingFactor {
    match n {
        7 => SpreadingFactor::_7,
        8 => SpreadingFactor::_8,
        10 => SpreadingFactor::_10,
        11 => SpreadingFactor::_11,
        12 => SpreadingFactor::_12,
        _ => SpreadingFactor::_9,
    }
}
fn bw_from(code: u8) -> Bandwidth {
    match code {
        1 => Bandwidth::_250KHz,
        2 => Bandwidth::_500KHz,
        _ => Bandwidth::_125KHz,
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

/// A parsed host command frame.
struct Frame {
    typ: u8,
    len: usize,
    payload: [u8; 260],
}

/// Persistent `7E A5 | type | len | payload | crc` parser — state survives across select-drops.
struct Parser {
    state: u8,
    typ: u8,
    len: usize,
    idx: usize,
    crc: u8,
    buf: [u8; 260],
}
impl Parser {
    fn new() -> Self {
        Self { state: 0, typ: 0, len: 0, idx: 0, crc: 0, buf: [0; 260] }
    }
    fn push(&mut self, b: u8) -> Option<Frame> {
        match self.state {
            0 => {
                if b == SYNC0 {
                    self.state = 1;
                }
            }
            1 => self.state = if b == SYNC1 { 2 } else if b == SYNC0 { 1 } else { 0 },
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
                    let mut f = Frame { typ: self.typ, len: self.len, payload: [0; 260] };
                    f.payload[..self.len.min(260)].copy_from_slice(&self.buf[..self.len.min(260)]);
                    return Some(f);
                }
            }
            _ => self.state = 0,
        }
        None
    }
}

async fn send_frame(tx: &mut UartTx<'_, Async>, typ: u8, payload: &[u8]) {
    let mut hdr = [SYNC0, SYNC1, typ, payload.len() as u8];
    let _ = tx.write_async(&hdr).await;
    let mut crc = typ ^ (payload.len() as u8);
    for &b in payload {
        crc ^= b;
    }
    if !payload.is_empty() {
        let _ = tx.write_async(payload).await;
    }
    let _ = tx.write_async(&[crc]).await;
    let _ = &mut hdr;
}

async fn send_info(tx: &mut UartTx<'_, Async>, p: &Params) {
    // [status, sync(2), errors(2), freq(4 BE), sf, bw_code, cr, pwr, lost(2), cad_busy(2), defer(2)]
    let hz = p.freq_hz;
    let sf_n = match p.sf {
        SpreadingFactor::_5 => 5,
        SpreadingFactor::_6 => 6,
        SpreadingFactor::_7 => 7,
        SpreadingFactor::_8 => 8,
        SpreadingFactor::_9 => 9,
        SpreadingFactor::_10 => 10,
        SpreadingFactor::_11 => 11,
        SpreadingFactor::_12 => 12,
    };
    let bw_code = match p.bw {
        Bandwidth::_250KHz => 1,
        Bandwidth::_500KHz => 2,
        _ => 0,
    };
    let mut info = [0u8; 19];
    info[2] = 0x12; // sync
    info[5] = (hz >> 24) as u8;
    info[6] = (hz >> 16) as u8;
    info[7] = (hz >> 8) as u8;
    info[8] = hz as u8;
    info[9] = sf_n;
    info[10] = bw_code;
    info[11] = p.cr_code();
    info[12] = p.pwr as u8;
    send_frame(tx, EVT_INFO, &info).await;
}

/// Read UART bytes through the persistent parser until a full frame arrives.
async fn read_frame(rx: &mut UartRx<'_, Async>, parser: &mut Parser) -> Frame {
    let mut buf = [0u8; 64];
    loop {
        if let Ok(n) = rx.read_async(&mut buf).await {
            for &b in &buf[..n] {
                if let Some(f) = parser.push(b) {
                    return f;
                }
            }
        }
    }
}

type LoraDev<'a> = LoRa<
    Sx127x<
        ExclusiveDevice<Spi<'a, Async>, Output<'a>, Delay>,
        GenericSx127xInterfaceVariant<Output<'a>, Input<'a>>,
        Sx1276,
    >,
    Delay,
>;

/// (Re)build modulation + packet params from the current Params.
fn build_params(
    lora: &mut LoraDev<'_>,
    p: &Params,
) -> Option<(ModulationParams, PacketParams, PacketParams)> {
    let mdltn = lora.create_modulation_params(p.sf, p.bw, p.cr, p.freq_hz).ok()?;
    let tx_pkt = lora.create_tx_packet_params(p.preamble, false, true, false, &mdltn).ok()?;
    let rx_pkt = lora
        .create_rx_packet_params(p.preamble, false, MAX_LORA, true, false, &mdltn)
        .ok()?;
    Some((mdltn, tx_pkt, rx_pkt))
}

#[esp_rtos::main]
async fn main(_spawner: Spawner) {
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
    let (mut uart_rx, mut uart_tx) = uart.split();

    // SX1276 over SPI2.
    let spi = Spi::new(
        peri.SPI2,
        SpiConfig::default().with_frequency(Rate::from_mhz(2)).with_mode(Mode::_0),
    )
    .unwrap()
    .with_sck(peri.GPIO5)
    .with_miso(peri.GPIO19)
    .with_mosi(peri.GPIO27)
    .into_async();
    let cs = Output::new(peri.GPIO18, Level::High, OutputConfig::default());
    let spi_dev = ExclusiveDevice::new(spi, cs, Delay).unwrap();
    let reset = Output::new(peri.GPIO14, Level::High, OutputConfig::default());
    let dio0 = Input::new(peri.GPIO26, InputConfig::default().with_pull(Pull::None));
    let iv = GenericSx127xInterfaceVariant::new(reset, dio0, None, None).unwrap();
    let config = Sx127xConfig { chip: Sx1276, tcxo_used: false, tx_boost: true, rx_boost: false };
    let mut lora = LoRa::new(Sx127x::new(spi_dev, iv, config), false, Delay)
        .await
        .expect("lora init");

    let mut p = Params::default();
    let (mut mdltn, mut tx_pkt, mut rx_pkt) = build_params(&mut lora, &p).expect("params");
    let mut parser = Parser::new();

    send_info(&mut uart_tx, &p).await; // announce ready
    let mut rxbuf = [0u8; 255];

    loop {
        // Arm continuous RX, then race host commands against a received LoRa frame.
        if lora.prepare_for_rx(RxMode::Continuous, &mdltn, &rx_pkt).await.is_err() {
            Timer::after(Duration::from_millis(5)).await;
        }
        match select(read_frame(&mut uart_rx, &mut parser), lora.rx(&rx_pkt, &mut rxbuf)).await {
            // ---- host command ----
            Either::First(f) => {
                let mut rebuild = false;
                match f.typ {
                    CMD_SET_FREQ if f.len >= 4 => {
                        p.freq_hz = ((f.payload[0] as u32) << 24)
                            | ((f.payload[1] as u32) << 16)
                            | ((f.payload[2] as u32) << 8)
                            | f.payload[3] as u32;
                        rebuild = true;
                        send_info(&mut uart_tx, &p).await;
                    }
                    CMD_SET_MOD if f.len >= 3 => {
                        p.sf = sf_from(f.payload[0]);
                        p.bw = bw_from(f.payload[1]);
                        p.cr = cr_from(f.payload[2]);
                        rebuild = true;
                        send_info(&mut uart_tx, &p).await;
                    }
                    CMD_SET_PWR if f.len >= 1 => {
                        p.pwr = f.payload[0] as i8 as i32;
                        send_info(&mut uart_tx, &p).await;
                    }
                    CMD_TX | CMD_TX_LBT => {
                        let mut ok = 1u8;
                        let mut attempts = 0u8;
                        if f.typ == CMD_TX_LBT {
                            // LBT: CAD up to 8 times, backing off; defer if the channel stays busy.
                            let mut clear = false;
                            for a in 0..8u8 {
                                attempts = a + 1;
                                match lora.cad(&mdltn).await {
                                    Ok(false) => {
                                        clear = true;
                                        break;
                                    }
                                    _ => Timer::after(Duration::from_millis(8 + (a as u64) * 6)).await,
                                }
                            }
                            if !clear {
                                ok = 0;
                            }
                        }
                        if ok == 1 {
                            let r = lora
                                .prepare_for_tx(&mdltn, &mut tx_pkt, p.pwr, &f.payload[..f.len])
                                .await;
                            ok = if r.is_ok() && lora.tx().await.is_ok() { 1 } else { 0 };
                        }
                        send_frame(&mut uart_tx, EVT_TXDONE, &[ok, attempts]).await;
                    }
                    CMD_CAD => {
                        let busy = matches!(lora.cad(&mdltn).await, Ok(true)) as u8;
                        send_frame(&mut uart_tx, EVT_CAD, &[busy]).await;
                    }
                    CMD_GET_RSSI => {
                        // lora-phy has no bare RSSI read; report 0 (host tolerates it).
                        send_frame(&mut uart_tx, EVT_RSSI, &[0, 0]).await;
                    }
                    CMD_GET_STATS => {
                        // Node C runs no on-device data plane (host cognition owns NDN), but the host
                        // waits for EVT_STATS — reply all-zero counters so it doesn't time out.
                        send_frame(&mut uart_tx, EVT_STATS, &[0u8; 24]).await;
                    }
                    // GET_INFO + every other SET (beacon/cad/lbt cfg/preamble/name-filter/relay/
                    // dataplane/stats/debug/bootloader): ack with EVT_INFO. NDN data-plane logic lives
                    // in the host cognition for node C.
                    _ => send_info(&mut uart_tx, &p).await,
                }
                if rebuild {
                    if let Some((m, t, r)) = build_params(&mut lora, &p) {
                        mdltn = m;
                        tx_pkt = t;
                        rx_pkt = r;
                    }
                }
            }
            // ---- LoRa frame received ----
            Either::Second(Ok((n, status))) => {
                let n = n as usize;
                if n <= rxbuf.len() {
                    // EVT_RX = [rssi i16 BE, snr i16 BE, ts_ms u32 BE, LoRa bytes]
                    let ts = embassy_time::Instant::now().as_millis() as u32;
                    let mut ev = [0u8; 8 + 255];
                    ev[0] = (status.rssi >> 8) as u8;
                    ev[1] = status.rssi as u8;
                    ev[2] = (status.snr >> 8) as u8;
                    ev[3] = status.snr as u8;
                    ev[4] = (ts >> 24) as u8;
                    ev[5] = (ts >> 16) as u8;
                    ev[6] = (ts >> 8) as u8;
                    ev[7] = ts as u8;
                    ev[8..8 + n].copy_from_slice(&rxbuf[..n]);
                    send_frame(&mut uart_tx, EVT_RX, &ev[..8 + n]).await;
                }
            }
            Either::Second(Err(_)) => {}
        }
        let _ = p.bw_hz();
    }
}
