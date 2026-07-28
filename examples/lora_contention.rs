//! #54 — does LBT (firmware CAD-before-TX) earn its keep at N=3 on LoRa?
//! Each node TXes a tagged frame every `NDN_TX_MS` (offered load high enough to collide) while its RX
//! task counts frames heard per source. Compare total network delivery with LBT off vs on:
//!   at N=2 on a clean channel LBT was counterproductive (added deferral, starved the weaker node,
//!   lora-lbt-cant-rescue-n2); at N=3 collisions should rise enough that carrier-sense wins.
//!
//!   sudo NDN_LBT=0 NDN_TAG=0 /tmp/loracon /dev/ttyACM0 30   # o5p-0 Waveshare
//!   sudo NDN_LBT=0 NDN_TAG=1 /tmp/loracon /dev/ttyACM0 30   # o5p-1 Waveshare
//!   sudo NDN_LBT=0 NDN_TAG=2 /tmp/loracon /dev/ttyUSB0 30   # o5p-1 Heltec
//! then all three with NDN_LBT=1.

use std::sync::Arc;
use std::sync::atomic::{AtomicU64, Ordering};
use std::time::{Duration, Instant};

use ndn_frame_io::{FrameIo, InjectFrame, TxIntent};
use ndn_radio_drivers::LoraSerialBackend;

fn env(k: &str) -> Option<String> { std::env::var(k).ok() }

#[tokio::main(flavor = "multi_thread", worker_threads = 2)]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    let path = std::env::args().nth(1).unwrap_or_else(|| "/dev/ttyACM0".into());
    let secs: u64 = std::env::args().nth(2).and_then(|s| s.parse().ok()).unwrap_or(30);
    let tag: u8 = env("NDN_TAG").and_then(|s| s.parse().ok()).unwrap_or(0);
    let tx_ms: u64 = env("NDN_TX_MS").and_then(|s| s.parse().ok()).unwrap_or(120);
    let lbt = env("NDN_LBT").map(|v| v == "1" || v == "true").unwrap_or(false);

    let dev = Arc::new(LoraSerialBackend::open(&path)?);
    // CAD sensitivity (SX126x): lower det_peak = more sensitive → detects a busy channel more readily,
    // so LBT actually defers. Defaults tuned sensitive; override via NDN_CAD_{SYM,PEAK,MIN}. Must be set
    // before enabling LBT. (deferred stayed 0 with the firmware defaults → LBT was a silent no-op.)
    if lbt {
        let sym: u8 = env("NDN_CAD_SYM").and_then(|s| s.parse().ok()).unwrap_or(4);
        let peak: u8 = env("NDN_CAD_PEAK").and_then(|s| s.parse().ok()).unwrap_or(18);
        let min: u8 = env("NDN_CAD_MIN").and_then(|s| s.parse().ok()).unwrap_or(10);
        if let Err(e) = dev.set_cad_cfg(sym, peak, min) { eprintln!("set_cad_cfg: {e:?}"); }
    }
    dev.set_lbt(lbt);
    // Move off ch65 (915 MHz) — it sits in the 902-928 HaLow band and collapses when HaLow runs
    // nearby (lora-halow-band-sharing). Default the band edge ch78 = 928 MHz, clear of the HaLow work.
    let ch: u8 = env("NDN_LORA_CH").and_then(|s| s.parse().ok()).unwrap_or(78);
    {
        use ndn_radio_hal::{Bandwidth, RadioKnobs};
        dev.set_channel(ch, Bandwidth::Bw20).map_err(|e| format!("set_channel {ch}: {e:?}"))?;
    }
    println!("lora_contention tag={tag} path={path} ch={ch}(={}MHz) secs={secs} tx_ms={tx_ms} LBT={}",
        850 + ch as u32, if lbt {"ON"} else {"off"});

    let counts = [Arc::new(AtomicU64::new(0)), Arc::new(AtomicU64::new(0)), Arc::new(AtomicU64::new(0))];
    let deadline = Instant::now() + Duration::from_secs(secs);

    // RX task: tally frames per source. Payload = "NDNL<tag> <seq>".
    let rx = {
        let (dev, counts) = (dev.clone(), counts.clone());
        tokio::spawn(async move {
            while Instant::now() < deadline {
                if let Ok(Ok(f)) = tokio::time::timeout(Duration::from_millis(500), dev.recv_frame()).await {
                    let p = &f.payload;
                    if p.len() >= 5 && &p[0..4] == b"NDNL" {
                        let t = (p[4] - b'0') as usize;
                        if t < 3 && t != tag as usize {
                            counts[t].fetch_add(1, Ordering::Relaxed);
                        }
                    }
                }
            }
        })
    };

    // TX loop: offer a tagged frame every tx_ms (firmware does CAD-before-TX when LBT is on).
    let mut sent = 0u64;
    let mut txfail = 0u64;
    let mut seq = 0u32;
    while Instant::now() < deadline {
        let body = format!("NDNL{} {}", tag, seq);
        seq += 1;
        match dev.inject(InjectFrame::broadcast(body.into_bytes().into(), TxIntent::CONSERVATIVE)).await {
            Ok(()) => sent += 1,
            Err(_) => txfail += 1,
        }
        tokio::time::sleep(Duration::from_millis(tx_ms)).await;
    }
    rx.abort();

    let (c0, c1, c2) = (counts[0].load(Ordering::Relaxed), counts[1].load(Ordering::Relaxed), counts[2].load(Ordering::Relaxed));
    let (cad_busy, deferred) = dev.csma_counters().unwrap_or((0, 0));
    println!("=== tag={tag} LBT={} === sent={sent} txfail={txfail} heard_from_peers={} (n0={c0} n1={c1} n2={c2}) cad_busy={cad_busy} deferred={deferred}",
        if lbt {"ON"} else {"off"}, c0 + c1 + c2);
    use std::io::Write;
    let _ = std::io::stdout().flush();
    Ok(())
}
