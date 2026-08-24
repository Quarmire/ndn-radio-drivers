//! **AR9271 EEPROM/OTP cal read over libusb — foundation of the OLPC `set_board_values` port (M1).**
//!
//! The low-TX bug: `hw_reset` skips `set_board_values`, so the PA runs uncalibrated-low. That cal is in
//! the AR9271 OTP, memory-mapped at `AR5416_EEPROM_OFFSET` and read by register reads (ath9k
//! `ath9k_hw_usb_gen_fill_eeprom`): struct word `w` = `reg_read(0x2000 + ((w+64)<<2)) & 0xffff`.
//! Validates the 4k EEPROM by its **checksum** (16-bit sum over `length` words == 0xffff — the honest
//! proof the read is complete+correct) and decodes the power-relevant fields the port needs.
//!
//! ```sh
//! sudo NDN_ATH9K_FW=/tmp/htc_9271.fw /tmp/ath9k_eeprom_dump
//! ```
use std::process::ExitCode;

use ndn_radio_drivers::Ath9kHtcBackend;

const EEP_OFF: u32 = 0x2000;
const EEP_S: u32 = 2;
const USB_W0: u32 = 64; // USB struct start word
const N: usize = 400; // > 376-word 4k struct

fn main() -> ExitCode {
    let fw = std::fs::read(std::env::var("NDN_ATH9K_FW").expect("NDN_ATH9K_FW")).expect("fw");
    let mut dev = Ath9kHtcBackend::open().expect("open");
    dev.download_firmware(&fw).and_then(|_| dev.htc_init()).expect("transport");
    dev.hw_reset(2412).and_then(|_| dev.connect_data_services()).expect("bring-up");
    dev.wmi_start().expect("wmi_start");

    // Struct words (baseEepHeader starts at struct word 0 = register word 64).
    let rw = |dev: &Ath9kHtcBackend, w: usize| (dev.reg_read(EEP_OFF + (((w as u32) + USB_W0) << EEP_S)).unwrap_or(0xffff_ffff) & 0xffff) as u16;
    let s: Vec<u16> = (0..N).map(|w| rw(&dev, w)).collect();

    // Magic sits at register word 0 (before the struct's +64 base).
    let magic = (dev.reg_read(EEP_OFF).unwrap_or(0) & 0xffff) as u16;
    let length = s[0] as usize; // baseEepHeader.length (words)
    let checksum = s[1];
    let version = s[2];
    println!("magic(reg 0x2000)={magic:#06x}  length={length}w  checksum={checksum:#06x}  version={version:#06x} (EEP ver {})", version >> 12);

    // ★ Checksum is XOR (ath9k_hw_nvram_validate_checksum: `sum ^= eepdata[i]`), over `length` words,
    // == 0xffff. The honest proof the read is complete + correct.
    let xsum: u16 = (0..length.min(N)).fold(0u16, |a, i| a ^ s[i]);
    println!("checksum: XOR over {length}w = {xsum:#06x}  → {}", if xsum == 0xffff { "VALID ✓ (read is complete + correct)" } else { "INVALID (read/framing wrong)" });

    // Bytes helper (LE within each 16-bit word).
    let byte = |off: usize| -> u8 { let w = s[off / 2]; if off & 1 == 0 { (w & 0xff) as u8 } else { (w >> 8) as u8 } };

    // base_eep_header_4k = 32 bytes: rxMask@18 txMask@19 deviceType@30 txGainType@31 macAddr@12..17.
    println!("\nbase header: txGainType={} (0=normal,1=high)  rxMask={:#04x} txMask={:#04x}  regDmn=[{:#06x},{:#06x}]",
        byte(31), byte(18), byte(19), s[4], s[5]);
    let mac: Vec<String> = (12..18).map(|b| format!("{:02x}", byte(b))).collect();
    println!("  macAddr={}  deviceType={}", mac.join(":"), byte(30));

    // modalHeader starts after base(32B) + custData(20B) = byte 52.
    let m = 52usize;
    let antctrl_common = (s[(m + 4) / 2] as u32) | ((s[(m + 6) / 2] as u32) << 16);
    println!("\nmodal @byte {m}:");
    println!("  antCtrlCommon = {antctrl_common:#010x}");
    println!("  switchSettling={:#04x} txRxAttenCh0={} rxTxMarginCh0={}", byte(m + 9), byte(m + 10), byte(m + 11));
    println!("  xpdGain={:#04x} xpd={:#04x} pdGainOverlap={}", byte(m + 20), byte(m + 21), byte(m + 24));
    // ob/db PA bias: ob_0/ob_1 packed @ modal byte 25, db1 @ 26, xpaBiasLvl @ 27.
    let obdb = byte(m + 25);
    println!("  ob_0={} ob_1={}  db1(byte)={:#04x}  xpaBiasLvl={}", obdb & 0xf, obdb >> 4, byte(m + 26), byte(m + 27));

    println!("\n→ M1 DONE: the AR9271 cal EEPROM reads over libusb (checksum {}). The OLPC set_board_values",
        if xsum == 0xffff { "valid" } else { "??" });
    println!("  port now has its input: analog cfg (antCtrl/ob/db/gain) + the target-power tables follow.");
    let _ = dev.detach();
    ExitCode::SUCCESS
}
