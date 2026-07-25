//! Kick a Waveshare LoRa dongle into its GD32 ROM UART bootloader over USB — no ST-Link.
//!
//! Sends `CMD_ENTER_BOOTLOADER`; the firmware branches to the system-memory bootloader on USART1, so
//! the SAME CH343/USB port (`/dev/ttyACM0`) is now speaking the STM32 bootloader protocol. Follow with:
//!   stm32flash -w firmware.bin -v -g 0x08000000 /dev/ttyACM0
//! (or use the reflash-over-usb.sh helper, which does both steps). Requires firmware built with
//! CMD_ENTER_BOOTLOADER — i.e. any firmware from the self-DFU bootstrap flash onward.

use ndn_radio_drivers::LoraSerialBackend;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let path = std::env::args().nth(1).unwrap_or_else(|| "/dev/ttyACM0".into());
    let dev = LoraSerialBackend::open(&path)?;
    dev.enter_bootloader()?;
    println!("[{path}] CMD_ENTER_BOOTLOADER sent — dongle is now in the ROM UART bootloader.");
    println!("  next: stm32flash -w firmware.bin -v -g 0x08000000 {path}");
    Ok(())
}
