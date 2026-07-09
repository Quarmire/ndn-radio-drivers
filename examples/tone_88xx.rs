//! Emit a continuous CW carrier on the RTL8812EU (88xx) for N seconds — an SDR radiation check.
use ndn_radio_drivers::LibUsbRtl88xxBackend;
fn main() -> Result<(), Box<dyn std::error::Error>> {
    let secs: u64 = std::env::args().nth(1).and_then(|s| s.parse().ok()).unwrap_or(10);
    let ch: u8 = std::env::args().nth(2).and_then(|s| s.parse().ok()).unwrap_or(36);
    // Pin the 8812EU (0xa81a) — the 8812AU (0x881a) is also in RTL88XX_PIDS but is a different chip.
    let d = LibUsbRtl88xxBackend::open_monitor_pid(0xa81a, ch)?;
    d.single_tone(true)?;
    println!("single_tone ON ch{ch} for {secs}s");
    std::thread::sleep(std::time::Duration::from_secs(secs));
    d.single_tone(false)?;
    println!("single_tone OFF");
    Ok(())
}
