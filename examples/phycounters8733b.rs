//! Do the per-format PHY counters (`REG_RXERR_RPT`) actually separate formats, or do they just
//! move together with "the medium was busy"? That is the whole claim, so it gets tested.
//!
//! Run on 2.4 GHz, because the test needs both modulations to exist: drive OFDM traffic and CCK
//! traffic past the receiver in turn and check that `ofdm_ok` moves for one and `cck_ok` for the
//! other. If both counters move for both arms, the per-format claim is false and these are just
//! another occupancy number.
//!
//! Prints per-interval DELTAS (the counters are free-running 16-bit and wrap), alongside
//! `read_channel_activity` (0x2c08) so the existing occupancy knob can be compared against them.
use ndn_radio_drivers::{RadioKnobs, Rtl8733buBackend};
use std::time::Instant;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let ch: u8 = std::env::args().nth(1).and_then(|s| s.parse().ok()).unwrap_or(11);
    let secs: u64 = std::env::args().nth(2).and_then(|s| s.parse().ok()).unwrap_or(40);
    let dev = Rtl8733buBackend::open()?;
    dev.bring_up_monitor(ch)?;
    println!("phy counters on ch{ch} for {secs}s  (t, then per-interval deltas)");
    println!("{:>6} {:>8} {:>8} {:>8} {:>8} {:>8} {:>8} {:>8} {:>8} {:>8} {:>9}",
             "t", "ofdm_ok", "ofdm_err", "ofdm_fa", "cck_ok", "cck_err", "cck_fa",
             "ht_ok", "ht_err", "ht_fa", "cca(2c08)");
    let mut prev = dev.read_phy_counters()?;
    let mut prev_cca = 0u16;
    let start = Instant::now();
    while start.elapsed().as_secs() < secs {
        std::thread::sleep(std::time::Duration::from_millis(1000));
        let c = dev.read_phy_counters()?;
        let cca = dev.read_channel_activity()?.unwrap_or(0);
        let d = |now: u16, was: u16| now.wrapping_sub(was);
        println!("{:>6.1} {:>8} {:>8} {:>8} {:>8} {:>8} {:>8} {:>8} {:>8} {:>8} {:>9}",
                 start.elapsed().as_secs_f64(),
                 d(c.ofdm_ok, prev.ofdm_ok), d(c.ofdm_err, prev.ofdm_err), d(c.ofdm_fa, prev.ofdm_fa),
                 d(c.cck_ok, prev.cck_ok), d(c.cck_err, prev.cck_err), d(c.cck_fa, prev.cck_fa),
                 d(c.ht_ok, prev.ht_ok), d(c.ht_err, prev.ht_err), d(c.ht_fa, prev.ht_fa),
                 d(cca, prev_cca));
        prev = c;
        prev_cca = cca;
    }
    // NAV_UPPER round-trip: quantisation is 128 us, so readback should be the request floored.
    println!("\n=== NAV_UPPER (0x0652, unit 128 us) ===");
    let orig = dev.nav_upper_us()?;
    for us in [0u32, 512, 2048, 8192, 30000, 60000] {
        dev.set_nav_upper_us(us)?;
        println!("  set {us:>6} us -> reads {:>6} us", dev.nav_upper_us()?);
    }
    dev.set_nav_upper_us(orig)?;
    println!("  restored {} us", dev.nav_upper_us()?);
    Ok(())
}
