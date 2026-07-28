//! Measure the **microsecond** common-view precision achievable from the hardware RX timestamp (#41),
//! via the driver's beacon side channel. Every 802.11 beacon on-channel carries the transmitter's
//! hardware TSF; we pair it with OUR hardware RX stamp (RXTSFL) of the same frame. The jitter of
//! `beacon_tsf − our_rxtsfl` across consecutive beacons — its first difference, which removes the
//! constant offset and slow crystal drift — is the common-view precision: two nodes tracking the SAME
//! beacon are mutually aligned to this. This is the µs path the software-embedded TimeBeacon (TX-
//! latency-bound at ~ms) cannot reach.
//!
//! Run on each node against the same AP (5 GHz; ch40 had a campus AP in #41):
//!   sudo ./beacon_cv 40 20
//! Compare the per-BSSID `first-diff std` — that is the number. Match the BSSID across both nodes to
//! confirm they track the same reference.

use std::collections::HashMap;
use std::sync::Arc;
use std::time::{Duration, Instant};

use ndn_radio_drivers::LibUsbRtl88xxBackend;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let ch: u8 = std::env::args().nth(1).and_then(|s| s.parse().ok()).unwrap_or(40);
    let secs: u64 = std::env::args().nth(2).and_then(|s| s.parse().ok()).unwrap_or(20);
    let pid: u16 = std::env::var("NDN_PID")
        .ok()
        .and_then(|s| u16::from_str_radix(s.trim_start_matches("0x"), 16).ok())
        .unwrap_or(0xa81a);

    let d = Arc::new(LibUsbRtl88xxBackend::open_monitor_pid(pid, ch)?);
    let _pump = d.spawn_rx_pump(8);
    println!("beacon_cv: pid={pid:04x} ch{ch} secs={secs} — measuring hardware common-view jitter…");

    // Per-BSSID series of (beacon_tsf, our_rxtsfl), collected by polling the side channel.
    let mut series: HashMap<[u8; 6], Vec<(u64, u64)>> = HashMap::new();
    let mut last_count = 0u64;
    let deadline = Instant::now() + Duration::from_secs(secs);
    while Instant::now() < deadline {
        if let Some((btsf, rxtsfl, count, bssid)) = d.beacon_common_view() {
            if count != last_count {
                last_count = count;
                series.entry(bssid).or_default().push((btsf, rxtsfl));
            }
        }
        std::thread::sleep(Duration::from_millis(1));
    }

    // Report every AP we tracked, best-sampled first.
    let mut rows: Vec<_> = series.into_iter().collect();
    rows.sort_by_key(|(_, v)| std::cmp::Reverse(v.len()));
    if rows.is_empty() {
        println!("no beacons heard on ch{ch} — is there an AP on this channel? try 149/157/…");
        return Ok(());
    }
    println!("\nBSSID              beacons  first-diff-std   spread    (common-view precision)");
    for (bssid, v) in &rows {
        if v.len() < 3 {
            continue;
        }
        // offset_k = beacon_tsf − rxtsfl (µs, wrapping); first difference removes constant + drift.
        let offs: Vec<i64> = v.iter().map(|&(b, r)| (b as i64).wrapping_sub(r as i64)).collect();
        let mut diffs: Vec<i64> = offs.windows(2).map(|w| w[1] - w[0]).collect();
        // Drop a lone 2^32 step if the 32-bit RXTSFL wrapped mid-run (a real hardware artifact, not
        // clock jitter): keep only |d| < 100 ms.
        diffs.retain(|d| d.abs() < 100_000);
        if diffs.len() < 2 {
            continue;
        }
        let mean = diffs.iter().sum::<i64>() as f64 / diffs.len() as f64;
        let var = diffs.iter().map(|&x| (x as f64 - mean).powi(2)).sum::<f64>() / diffs.len() as f64;
        let (min, max) = (*diffs.iter().min().unwrap(), *diffs.iter().max().unwrap());
        let mac = bssid.iter().map(|b| format!("{b:02x}")).collect::<Vec<_>>().join(":");
        println!(
            "{mac}  {:>7}  {:>10.2} µs  {:>6} µs",
            v.len(),
            var.sqrt(),
            max - min,
        );
    }
    Ok(())
}
