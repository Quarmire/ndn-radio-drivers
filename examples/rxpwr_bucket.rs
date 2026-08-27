//! Receive side of the TX-power sweep: bucket per-frame RSSI by the index carried in the payload.
//!
//! Runs on whatever radio has HEADROOM at the link's operating point — that is the whole reason this
//! exists. The kernel `rtw88_8812au` radiotap meter reports only -18/-16/-14 dBm here (2 dB
//! quantisation, 6 dB total range) and disagrees with the a81a by 54 dB at the same location, so it
//! cannot show a 14-27 dB commanded change; the a81a's per-frame RSSI spans -86..-66 on this link and
//! can. Attribution is by `payload[0]`/`payload[1]`, so no MAC filter and no timing alignment.
//!
//! Usage: sudo NDN_PID=a81a ./rxpwr_bucket [channel] [seconds]

use std::collections::BTreeMap;
use std::time::{Duration, Instant};

#[tokio::main(flavor = "multi_thread", worker_threads = 2)]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    let ch: u8 = std::env::args()
        .nth(1)
        .and_then(|s| s.parse().ok())
        .unwrap_or(36);
    let secs: u64 = std::env::args()
        .nth(2)
        .and_then(|s| s.parse().ok())
        .unwrap_or(120);
    let pid: u16 = match std::env::var("NDN_PID").as_deref() {
        Ok("a81a") | Err(_) => 0xa81a,
        Ok(p) => u16::from_str_radix(p.trim_start_matches("0x"), 16)?,
    };
    let d = ndn_radio_drivers::open_named_radio(pid, ch)?.io;
    let deadline = Instant::now() + Duration::from_secs(secs);
    // (knob, index) -> (sum, n, min, max)
    let mut acc: BTreeMap<(u8, u8), (i64, u64, i32, i32)> = BTreeMap::new();

    while Instant::now() < deadline {
        if let Ok(Ok(f)) = tokio::time::timeout(Duration::from_millis(50), d.recv_frame()).await {
            let p = &f.payload;
            // The sweep's frames are the ones padded with 0xC3; anything else on-channel is ambient.
            if p.len() >= 3 && p[2] == 0xC3 && p[1] < 10 {
                if let Some(r) = f.rssi_dbm {
                    let e = acc
                        .entry((p[1], p[0]))
                        .or_insert((0, 0, i32::MAX, i32::MIN));
                    e.0 += r as i64;
                    e.1 += 1;
                    e.2 = e.2.min(r as i32);
                    e.3 = e.3.max(r as i32);
                }
            }
        }
    }

    println!(
        "=== per-index RSSI (0=ref 0x4308, 1=table 0x3a00, 2=datapath 0x1e4x, 3=RF 0x01[4:0], 4=swing 0x18a0) ==="
    );
    let mut last_knob = 255u8;
    for ((knob, idx), (sum, n, mn, mx)) in &acc {
        if *knob != last_knob {
            println!("  --- knob {knob}");
            last_knob = *knob;
        }
        println!(
            "    idx 0x{idx:02x}  n={n:<5} mean={:7.2} dBm  min={mn} max={mx}",
            *sum as f64 / *n as f64
        );
    }
    if acc.is_empty() {
        println!("  (nothing decoded — check channel, and that the TX side actually radiated)");
    }
    Ok(())
}
