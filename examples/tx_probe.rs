//! Minimal TX-radiation isolation probe over the production `open_named_radio` path.
//! TX floods a tagged NDN payload; RX counts received frames per tag and prints EVERY second
//! (incremental — a late hang/flush can't hide the answer). Answers exactly: does chip A's TX
//! radiate to chip B's RX on this channel?
//!
//!   sudo NDN_ROLE=tx NDN_PID=au   NDN_TAG=2 /tmp/txprobe 40 20   # 8812au floods, tag 2
//!   sudo NDN_ROLE=rx NDN_PID=a81a           /tmp/txprobe 40 20   # a81a counts what it hears

use std::sync::Arc;
use std::sync::atomic::{AtomicU64, Ordering};
use std::time::{Duration, Instant};

use ndn_frame_io::{BROADCAST, InjectFrame, TxIntent};

fn env(k: &str) -> Option<String> { std::env::var(k).ok() }

#[tokio::main(flavor = "multi_thread", worker_threads = 2)]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    let ch: u8 = std::env::args().nth(1).and_then(|s| s.parse().ok()).unwrap_or(40);
    let secs: u64 = std::env::args().nth(2).and_then(|s| s.parse().ok()).unwrap_or(20);
    let role = env("NDN_ROLE").unwrap_or_else(|| "rx".into());
    let tag: u8 = env("NDN_TAG").and_then(|s| s.parse().ok()).unwrap_or(2);
    let pid: u16 = match env("NDN_PID").as_deref() {
        Some("au") => 0x8812,
        Some("vs") => 0x881a,
        Some(p) => u16::from_str_radix(p.trim_start_matches("0x"), 16)?,
        None => 0xa81a,
    };
    let d = ndn_radio_drivers::open_named_radio(pid, ch)?;
    let deadline = Instant::now() + Duration::from_secs(secs);
    println!("tx_probe role={role} pid=0x{pid:04x} ch{ch} secs={secs} tag={tag}");

    if role == "tx" {
        let src = [0x02, b'M', b'D', b'R', tag, 0x01];
        let pad = vec![0u8; 900];
        let mut sent = 0u64;
        let mut last = Instant::now();
        while Instant::now() < deadline {
            let mut payload = Vec::with_capacity(902);
            payload.push(0xA0 | tag);
            payload.push(tag);
            payload.extend_from_slice(&pad);
            // Bound each inject so a blocking write can't wedge the loop silently — we want to SEE the rate.
            match tokio::time::timeout(Duration::from_millis(200),
                d.inject(InjectFrame { payload: payload.into(), tx: TxIntent::CONSERVATIVE, dst: BROADCAST, src })).await {
                Ok(_) => sent += 1,
                Err(_) => {} // inject exceeded 200ms — FIFO stalled; keep counting elapsed
            }
            if last.elapsed() >= Duration::from_secs(1) {
                println!("  TX sent={sent} ({:.0}/s)", sent as f64 / (secs as f64 - deadline.saturating_duration_since(Instant::now()).as_secs_f64()).max(0.001));
                last = Instant::now();
            }
        }
        println!("=== TX DONE sent={sent} ===");
    } else {
        let counts = [Arc::new(AtomicU64::new(0)), Arc::new(AtomicU64::new(0)), Arc::new(AtomicU64::new(0)), Arc::new(AtomicU64::new(0))];
        let rx = {
            let (d, counts) = (d.clone(), counts.clone());
            tokio::spawn(async move {
                while Instant::now() < deadline {
                    if let Ok(Ok(f)) = tokio::time::timeout(Duration::from_millis(20), d.recv_frame()).await {
                        let p = &f.payload;
                        if p.len() >= 2 && (p[0] & 0xf0) == 0xA0 {
                            let t = (p[1] & 0x03) as usize;
                            counts[t].fetch_add(1, Ordering::Relaxed);
                        }
                    }
                }
            })
        };
        while Instant::now() < deadline {
            tokio::time::sleep(Duration::from_secs(1)).await;
            let c: Vec<u64> = counts.iter().map(|a| a.load(Ordering::Relaxed)).collect();
            println!("  RX heard tag0={} tag1={} tag2={} tag3={}", c[0], c[1], c[2], c[3]);
        }
        let _ = rx.await;
        let c: Vec<u64> = counts.iter().map(|a| a.load(Ordering::Relaxed)).collect();
        println!("=== RX DONE tag0={} tag1={} tag2={} tag3={} ===", c[0], c[1], c[2], c[3]);
    }
    Ok(())
}
