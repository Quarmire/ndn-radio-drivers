//! Heltec WiFi LoRa 32 V2 (ESP32 + SX1276) — Rust named-radio node C (task #54).
//!
//! Staged bring-up:
//!   1. boot + serial banner. ✓
//!   2. SX1276 over SPI via lora-phy — LoRa TX beacon, interop with the SX1262 dongles.  <-- THIS
//!   3. 7E-A5 serial-bridge protocol (matches `waveshare-lora-rs`) — host drives it as a LoRa modem.
//!   4. reuse `ndn-embedded` for the on-device data plane.
//!
//! lora-phy is embedded-hal-async, so this runs on the embassy executor under esp-rtos.

#![no_std]
#![no_main]

use embassy_executor::Spawner;
use embassy_time::{Delay, Duration, Timer};
use embedded_hal_bus::spi::ExclusiveDevice;
use esp_backtrace as _;
use esp_hal::clock::CpuClock;
use esp_hal::gpio::{Input, InputConfig, Level, Output, OutputConfig, Pull};
use esp_hal::spi::master::{Config as SpiConfig, Spi};
use esp_hal::spi::Mode;
use esp_hal::time::Rate;
use esp_hal::timer::timg::TimerGroup;
use esp_println::println;
use lora_phy::iv::GenericSx127xInterfaceVariant;
use lora_phy::sx127x::{Config as Sx127xConfig, Sx127x, Sx1276};
use lora_phy::{
    mod_params::{Bandwidth, CodingRate, SpreadingFactor},
    LoRa,
};

esp_bootloader_esp_idf::esp_app_desc!();

// --- Heltec V2 SX1276 pinout ---
// SPI: SCK5 / MISO19 / MOSI27 / CS18 ; control: RST14, DIO0-26.
const FREQ_HZ: u32 = 915_000_000; // Waveshare channel 65
const TX_DBM: i32 = 17;

#[esp_rtos::main]
async fn main(_spawner: Spawner) {
    let p = esp_hal::init(esp_hal::Config::default().with_cpu_clock(CpuClock::max()));
    esp_alloc::heap_allocator!(size: 32 * 1024);
    let timg0 = TimerGroup::new(p.TIMG0);
    let sw = esp_hal::interrupt::software::SoftwareInterruptControl::new(p.SW_INTERRUPT);
    esp_rtos::start(timg0.timer0, sw.software_interrupt0);

    println!("heltec-lora-rs: stage 2 (lora-phy SX1276) boot");

    // SPI2 on the SX1276 pins, async.
    let spi = Spi::new(
        p.SPI2,
        SpiConfig::default()
            .with_frequency(Rate::from_mhz(2))
            .with_mode(Mode::_0),
    )
    .unwrap()
    .with_sck(p.GPIO5)
    .with_miso(p.GPIO19)
    .with_mosi(p.GPIO27)
    .into_async();

    let cs = Output::new(p.GPIO18, Level::High, OutputConfig::default());
    let spi_dev = ExclusiveDevice::new(spi, cs, Delay).unwrap();

    let reset = Output::new(p.GPIO14, Level::High, OutputConfig::default());
    let dio0 = Input::new(p.GPIO26, InputConfig::default().with_pull(Pull::None));
    let iv = GenericSx127xInterfaceVariant::new(reset, dio0, None, None).unwrap();

    // Heltec V2 wires the SX1276 through PA_BOOST.
    let config = Sx127xConfig {
        chip: Sx1276,
        tcxo_used: false,
        tx_boost: true,
        rx_boost: false,
    };
    let mut lora = LoRa::new(Sx127x::new(spi_dev, iv, config), false, Delay)
        .await
        .expect("lora init");

    let mdltn = lora
        .create_modulation_params(
            SpreadingFactor::_9,
            Bandwidth::_125KHz,
            CodingRate::_4_5,
            FREQ_HZ,
        )
        .expect("mod params");
    let mut tx_pkt = lora
        .create_tx_packet_params(8, false, true, false, &mdltn)
        .expect("tx pkt params");

    println!("heltec-lora-rs: SX1276 up, beaconing @ {} Hz SF9 BW125", FREQ_HZ);
    let mut n = 0u32;
    loop {
        let mut msg = *b"heltec-rs test 0000";
        let d = &mut msg[15..19];
        d[0] = b'0' + ((n / 1000) % 10) as u8;
        d[1] = b'0' + ((n / 100) % 10) as u8;
        d[2] = b'0' + ((n / 10) % 10) as u8;
        d[3] = b'0' + (n % 10) as u8;
        match lora.prepare_for_tx(&mdltn, &mut tx_pkt, TX_DBM, &msg).await {
            Ok(()) => match lora.tx().await {
                Ok(()) => println!("TX beacon {}", n),
                Err(e) => println!("tx err {:?}", e),
            },
            Err(e) => println!("prepare_for_tx err {:?}", e),
        }
        n = n.wrapping_add(1);
        Timer::after(Duration::from_millis(3000)).await;
    }
}
