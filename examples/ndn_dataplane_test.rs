//! On-air validation of the firmware's on-device NDN data plane (#53).
//!
//! The firmware `ndn.rs` data-centric offload (name-hash RX filter, dedup, relay/hop, Content-Store
//! serve) ships DEFAULT-INERT — so a green build proves nothing about whether the paths actually fire
//! on real radios. This drives each one over the two Waveshare SX1262 dongles and reads the firmware's
//! own `EVT_STATS` counters back as proof.
//!
//! Two roles, one carrier, no host-side data plane on the TX node (it is a pure blaster):
//!   rx <path> <scenario> [secs]  — configure the data plane, reset counters, listen, print stats
//!   tx <path> <scenario> [secs]  — blast the scenario's scripted frame sequence
//!
//! Scenarios (wire shape `KIND|SRC|SF|NAME[|payload]`, the same the cognition demo uses):
//!   filter  RX allows only `ndn/keep`; TX alternates keep/drop  → `filtered` counts the dropped ones,
//!           and `recv_frame` (host wake = Deliver only) sees just the kept ones.
//!   dedup   RX dedup on; TX repeats one name                    → `deduped` = repeats, host sees 1.
//!   relay   RX relays `ndn/relayme`; TX sends it                → `relayed` climbs (re-broadcast+deliver).
//!   cs      RX serves from cache; TX sends D then I (same name) → `served` = Interests answered from
//!           cache (the host never wakes for a served Interest; it DOES wake for the cached Data).
//!
//! The two numbers cross-check: a counter is the firmware's self-report; the host `delivered` count is
//! an independent observation of what the firmware chose to pass up. They must agree with the model.

use std::sync::Arc;
use std::sync::atomic::{AtomicU32, Ordering};
use std::time::{Duration, Instant};

use ndn_frame_io::{FrameIo, InjectFrame, TxIntent};
use ndn_radio_drivers::LoraSerialBackend;
use ndn_radio_hal::{Bandwidth, RadioKnobs};

const CHANNEL: u8 = 65; // both ends on one carrier (~915 MHz); fixed, no cognition here.
const SF: u8 = 9;
const BW_KHZ: u32 = 125;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let rt = tokio::runtime::Builder::new_multi_thread().enable_all().build()?;
    rt.block_on(run())
}

/// Pin both nodes to identical LoRa params so the link actually closes.
fn pin_params(dev: &LoraSerialBackend) -> Result<(), Box<dyn std::error::Error>> {
    dev.set_lbt(false);
    dev.set_channel(CHANNEL, Bandwidth::from_code(0))?;
    dev.set_spreading_factor(SF)?;
    dev.set_bandwidth_khz(BW_KHZ)?;
    Ok(())
}

async fn run() -> Result<(), Box<dyn std::error::Error>> {
    let mut args = std::env::args().skip(1);
    let role = args.next().unwrap_or_else(|| "rx".into());
    let path = args.next().unwrap_or_else(|| "/dev/ttyACM0".into());
    let scenario = args.next().unwrap_or_else(|| "filter".into());
    let secs: u64 = args.next().and_then(|s| s.parse().ok()).unwrap_or(25);

    let dev = Arc::new(LoraSerialBackend::open(&path)?);
    pin_params(&dev)?;
    println!("[{role}/{scenario}] open OK on {path} — ch{CHANNEL} SF{SF} BW{BW_KHZ}");

    match role.as_str() {
        "tx" => tx(dev, &scenario, secs).await,
        "rx" => rx(dev, &scenario, secs).await,
        other => Err(format!("role must be tx|rx, got {other}").into()),
    }
}

/// Send one broadcast app frame.
async fn send(dev: &LoraSerialBackend, wire: &str) {
    let f = InjectFrame::broadcast(wire.as_bytes().to_vec().into(), TxIntent::CONSERVATIVE);
    if let Err(e) = dev.inject(f).await {
        eprintln!("  tx {wire:?} failed: {e}");
    }
}

async fn tx(dev: Arc<LoraSerialBackend>, scenario: &str, secs: u64) -> Result<(), Box<dyn std::error::Error>> {
    let deadline = Instant::now() + Duration::from_secs(secs);
    // The firmware's dedup ring persists across runs (the dongle never reboots), so a fixed dedup name
    // would already be "seen" from a prior sweep and the very first frame would be dropped. Mint a
    // per-run-unique name so the first frame is genuinely new (passes) and only the repeats dedup.
    let nonce = std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .map(|d| d.as_nanos())
        .unwrap_or(0);
    let dup_name = format!("ndn/dup/{nonce}");
    let mut n = 0u32;
    while Instant::now() < deadline {
        match scenario {
            // Alternate an allowed name and a blocked one — the filter should split them.
            "filter" => {
                send(&dev, &format!("I|T|{SF}|ndn/keep")).await;
                send(&dev, &format!("I|T|{SF}|ndn/drop")).await;
            }
            // Same (run-unique) name every time — all but the first are duplicates.
            "dedup" => send(&dev, &format!("I|T|{SF}|{dup_name}")).await,
            // A name in the relay set — each should be re-broadcast and delivered.
            "relay" => send(&dev, &format!("I|T|{SF}|ndn/relayme")).await,
            // Prime the cache with Data, then ask for it — the RX node answers from cache.
            "cs" => {
                send(&dev, &format!("D|T|{SF}|ndn/cached|hello")).await;
                tokio::time::sleep(Duration::from_millis(350)).await;
                send(&dev, &format!("I|T|{SF}|ndn/cached")).await;
            }
            other => return Err(format!("unknown scenario {other}").into()),
        }
        n += 1;
        // ~2 rounds/s — leave airtime for the RX node's relay/serve re-transmissions.
        tokio::time::sleep(Duration::from_millis(450)).await;
    }
    println!("[tx/{scenario}] done — {n} rounds sent");
    Ok(())
}

async fn rx(dev: Arc<LoraSerialBackend>, scenario: &str, secs: u64) -> Result<(), Box<dyn std::error::Error>> {
    // The firmware DataPlane persists across host restarts (the dongle never reboots between runs), so
    // establish the COMPLETE state each time — clear the knobs this scenario doesn't use, or the prior
    // scenario's filter/dedup would leak in and contaminate the result.
    dev.set_name_filter(&[])?; // clear filter (pass-all)
    dev.set_relay(&[])?; // clear relay set
    dev.set_dataplane(false, false, false, 0, 0)?; // cs off, dedup off, hop off
    match scenario {
        "filter" => dev.set_name_filter(&[b"ndn/keep"])?,
        "dedup" => dev.set_dataplane(false, true, false, 0, 0)?, // dedup on
        "relay" => dev.set_relay(&[b"ndn/relayme"])?,
        "cs" => dev.set_dataplane(true, false, false, 0, 0)?, // cs_serve on
        other => return Err(format!("unknown scenario {other}").into()),
    }
    dev.reset_ndn_stats()?;
    println!("[rx/{scenario}] configured + counters zeroed — listening {secs}s");

    // Count frames the firmware chose to Deliver (host wake). Independent of the counters.
    let delivered = Arc::new(AtomicU32::new(0));
    {
        let dev = dev.clone();
        let delivered = delivered.clone();
        tokio::spawn(async move {
            while let Ok(f) = dev.recv_frame().await {
                let _wire = String::from_utf8_lossy(&f.payload);
                delivered.fetch_add(1, Ordering::Relaxed);
            }
        });
    }

    tokio::time::sleep(Duration::from_secs(secs)).await;

    let s = dev.ndn_stats()?;
    let d = delivered.load(Ordering::Relaxed);
    println!("\n[rx/{scenario}] === RESULT ===");
    println!("  counters: rx={} filtered={} deduped={} served={} relayed={}", s.rx, s.filtered, s.deduped, s.served, s.relayed);
    println!("  host delivered (Deliver wakes): {d}");

    // Judge against the model for this scenario.
    let (ok, why) = match scenario {
        "filter" => (
            s.filtered > 0 && d > 0,
            "filtered>0 (drops blocked at antenna) AND delivered>0 (keeps passed up)",
        ),
        "dedup" => (
            s.deduped > 0 && d >= 1 && d <= 3,
            "deduped>0 (repeats dropped) AND host saw ~1 (first only)",
        ),
        "relay" => (s.relayed > 0, "relayed>0 (frames re-broadcast + delivered)"),
        "cs" => (s.served > 0, "served>0 (Interests answered from on-device cache)"),
        _ => (false, ""),
    };
    println!("  model: {why}");
    println!("  VERDICT: {}", if ok { "PASS ✅" } else { "FAIL ❌" });
    Ok(())
}
