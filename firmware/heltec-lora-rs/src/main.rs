//! Heltec WiFi LoRa 32 V2 (ESP32 + SX1276) — Rust named-radio node C (task #54).
//!
//! Staged bring-up, each stage validated before the next:
//!   1. boot + serial banner — toolchain + esp-hal init + CP2102 serial path.  <-- THIS STAGE
//!   2. SX1276 over SPI via lora-phy — standard LoRa TX/RX, interop with the SX1262 dongles.
//!   3. 7E-A5 serial-bridge protocol (matches `waveshare-lora-rs`) — host drives it as a LoRa modem.
//!   4. reuse `ndn-embedded` for the on-device data plane.

#![no_std]
#![no_main]

use esp_backtrace as _;
use esp_hal::clock::CpuClock;
use esp_hal::delay::Delay;
use esp_hal::main;
use esp_println::println;

esp_bootloader_esp_idf::esp_app_desc!();

#[main]
fn main() -> ! {
    let _p = esp_hal::init(esp_hal::Config::default().with_cpu_clock(CpuClock::max()));
    let delay = Delay::new();
    let mut tick = 0u32;
    loop {
        println!("heltec-lora-rs: boot ok (stage 1), tick {}", tick);
        tick = tick.wrapping_add(1);
        delay.delay_millis(1000);
    }
}
