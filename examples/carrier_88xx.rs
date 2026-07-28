//! Emit a single OFDM subcarrier (offset from the LO) on the RTL8812EU/8822E for N
//! seconds — an SDR I/Q-quality check. Unlike a pure LO tone, this drives the BB→DAC→RF
//! path, so the captured spectrum shows (a) the wanted subcarrier at +Δf, (b) its I/Q
//! IMAGE at −Δf (whose depth vs the wanted = the I/Q gain/phase balance the TX IQK
//! corrects), and (c) LO leakage / carrier feedthrough at DC (the I/Q DC offset the IQK
//! also nulls). Run with vs without `NDN_RADIO_SKIP_CAL=1` to see whether the FW IQK
//! actually improves image rejection.
//!   args: [secs=10] [ch=149]
use ndn_radio_drivers::LibUsbRtl88xxBackend;
fn main() -> Result<(), Box<dyn std::error::Error>> {
    let secs: u64 = std::env::args().nth(1).and_then(|s| s.parse().ok()).unwrap_or(10);
    let ch: u8 = std::env::args().nth(2).and_then(|s| s.parse().ok()).unwrap_or(149);

    // Stage 1: claim only (open_pid, no bring_up) — isolates a USB/claim fault from a
    // bring-up/cal fault. Print the raw error (with errno) at each stage.
    eprintln!("[stage] open_pid(0xa81a) ...");
    let d = match LibUsbRtl88xxBackend::open_pid(0xa81a) {
        Ok(d) => {
            eprintln!("[stage] open_pid OK (claimed)");
            d
        }
        Err(e) => {
            eprintln!("[stage] open_pid FAILED: {e:?}");
            return Err(e.into());
        }
    };
    // Stage 2: first vendor control read (chip_info).
    eprintln!("[stage] chip_info() ...");
    match d.chip_info() {
        Ok(info) => eprintln!("[stage] chip_info OK: chip_id={:#04x}", info.chip_id),
        Err(e) => {
            eprintln!("[stage] chip_info FAILED: {e:?}");
            return Err(e.into());
        }
    }
    // Stage 3: full bring-up.
    eprintln!("[stage] bring_up(ch{ch}) ...");
    if let Err(e) = d.bring_up(ch) {
        eprintln!("[stage] bring_up FAILED: {e:?}");
        return Err(e.into());
    }
    eprintln!("[stage] bring_up OK");

    // NDN_NO_TONE: bring_up then just hold (NO TX enable) — proves whether the
    // device stays on the USB bus without keying the PA (brownout isolation).
    if std::env::var("NDN_NO_TONE").is_ok() {
        eprintln!("[stage] NO_TONE: holding {secs}s with TX OFF (bus-stability check)");
        std::thread::sleep(std::time::Duration::from_secs(secs));
        eprintln!("[stage] held {secs}s, still alive; chip_id re-read: {:?}", d.chip_info().map(|i| i.chip_id));
        return Ok(());
    }
    eprintln!("[stage] single_tone(true) ...");
    if let Err(e) = d.single_tone(true) {
        eprintln!("[stage] single_tone FAILED: {e:?}");
        return Err(e.into());
    }
    eprintln!("[stage] single_tone OK");
    if std::env::var("NDN_NO_CARRIER").is_err() {
        eprintln!("[stage] single_carrier(true) ...");
        if let Err(e) = d.single_carrier(true) {
            eprintln!("[stage] single_carrier FAILED: {e:?}");
            return Err(e.into());
        }
        eprintln!("[stage] single_carrier OK");
    }
    let cal = if std::env::var("NDN_RADIO_SKIP_CAL").is_ok() {
        "SKIP_CAL"
    } else {
        "cal"
    };
    println!("single_carrier ON ch{ch} [{cal}] for {secs}s");
    std::thread::sleep(std::time::Duration::from_secs(secs));
    d.single_carrier(false)?;
    d.single_tone(false)?;
    println!("single_carrier OFF");
    Ok(())
}
