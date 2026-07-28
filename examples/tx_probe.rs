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
        // TX-only sender (never runs a concurrent RX task, so it can't hit the a81a's TX+RX libusb
        // contention). NDN_MODE=slotted → transmit only in this node's slot of an N=3 superframe (all
        // three disjoint, collision-free); NDN_MODE=contention → transmit a random ~1/3 of slots
        // (independent → same-slot collisions ~2/9). A neutral RX observer compares total received.
        let slotted = env("NDN_MODE").as_deref() != Some("contention");
        let slot_us: u64 = env("NDN_SLOT_US").and_then(|s| s.parse().ok()).unwrap_or(20_000);
        let my_slot = (tag as u64) % 3;
        let mut rng: u64 = (std::process::id() as u64).wrapping_mul(0x9E37_79B9) ^ (tag as u64 + 1);
        let mut coin = || { rng ^= rng << 13; rng ^= rng >> 7; rng ^= rng << 17; rng % 3 == my_slot };
        let src = [0x02, b'M', b'D', b'R', tag, 0x01];
        let pad = vec![0u8; 900];
        let mut sent = 0u64;
        let mut last = Instant::now();
        let (mut cur_epoch, mut tx_this) = (u64::MAX, false);
        let now_us = || std::time::SystemTime::now().duration_since(std::time::UNIX_EPOCH).map(|d| d.as_micros() as u64).unwrap_or(0);
        while Instant::now() < deadline {
            let epoch = now_us() / slot_us;
            if epoch != cur_epoch {
                cur_epoch = epoch;
                tx_this = if slotted { epoch % 3 == my_slot } else { coin() };
            }
            if !tx_this {
                tokio::time::sleep(Duration::from_micros(500)).await;
            } else {
                let mut payload = Vec::with_capacity(902);
                payload.push(0xA0 | tag);
                payload.push(tag);
                payload.extend_from_slice(&pad);
                match tokio::time::timeout(Duration::from_millis(200),
                    d.inject(InjectFrame { payload: payload.into(), tx: TxIntent::CONSERVATIVE, dst: BROADCAST, src })).await {
                    Ok(_) => sent += 1,
                    Err(_) => {}
                }
                tokio::task::yield_now().await;
            }
            if last.elapsed() >= Duration::from_secs(1) {
                println!("  TX sent={sent} mode={}", if slotted { "slotted" } else { "contention" });
                last = Instant::now();
            }
        }
        println!("=== TX DONE sent={sent} mode={} ===", if slotted { "slotted" } else { "contention" });
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
