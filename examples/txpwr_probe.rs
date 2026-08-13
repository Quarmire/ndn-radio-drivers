//! **Does `set_tx_power` actually stick?** — the no-RF discriminator for the claim-C power gate.
//!
//! The 2026-08-13 sweep commanded five TXAGC indices on the a81a and measured zero RF effect. The
//! register story: `set_tx_power` writes the OFDM/CCK power references (`0x18e8`/`0x18a0` + path
//! B), but `thermal_track` (the ~2 s watchdog) rewrites those same registers from `tx_ref_base` —
//! which `set_tx_power` did not update. This probe reads the register field before, right after,
//! and 6 s after (two watchdog ticks past) a `set_tx_power(4)`:
//!
//!   * pre-fix signature : after = 4, settled = calibration value (the watchdog won)
//!   * post-fix signature: after = 4, settled = 4 ± a small thermal delta (the watchdog preserves)
//!
//! Run on the OPi (kernel driver off the device): `sudo ./txpwr_probe [pid-hex]`

fn main() -> Result<(), Box<dyn std::error::Error>> {
    use std::time::Duration;

    let pid = u16::from_str_radix(
        &std::env::args().nth(1).unwrap_or_else(|| "a81a".into()),
        16,
    )?;
    // Full monitor bring-up (including calibration + watchdog) — the probe must see the same
    // runtime the campaign sees, or it proves nothing about the campaign's failure.
    let dev = std::sync::Arc::new(
        ndn_radio_drivers::LibUsbRtl88xxBackend::open_monitor_pid(pid, 149)?,
    );
    let _wd = dev.spawn_watchdog();

    let ofdm_ref = |d: &ndn_radio_drivers::LibUsbRtl88xxBackend| -> Result<u32, _> {
        d.read32(0x18e8).map(|v| (v & 0x0001_fc00) >> 10)
    };

    let before = ofdm_ref(&dev)?;
    dev.set_tx_power(4)?;
    let after = ofdm_ref(&dev)?;
    std::thread::sleep(Duration::from_secs(6)); // two watchdog ticks
    let settled = ofdm_ref(&dev)?;

    println!("OFDM ref [16:10] of 0x18e8:");
    println!("  before set_tx_power(4) : {before}");
    println!("  immediately after      : {after}");
    println!("  after 6 s (2 wd ticks) : {settled}");
    if after != 4 {
        println!("VERDICT: the WRITE itself did not land — different defect than the watchdog.");
    } else if settled.abs_diff(4) <= 3 {
        println!("VERDICT: STICKS — the watchdog preserves the request (fix working).");
    } else {
        println!(
            "VERDICT: CLOBBERED — the watchdog restored {settled}; tx_ref_base is not anchored \
             to the request (the pre-fix defect)."
        );
    }
    Ok(())
}
