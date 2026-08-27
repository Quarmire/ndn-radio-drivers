//! Does `REG_TXPAUSE` actually SHAPE AIRTIME, or only defer it?
//!
//! The earlier TXPAUSE measurement could not answer this and said so: because a paused queue HOLDS
//! rather than drops, the delivered COUNT is conserved and only timing moves — so a count-based
//! receiver is structurally blind to shaping. This uses the receiver that is not blind: the f72b
//! latches a hardware RX timestamp (RXTSFL) per frame, so we can look at WHEN frames landed.
//!
//! Analysis is the inter-arrival gap distribution. If gating shapes airtime, arrivals cluster into
//! bursts separated by gaps near the closed half-period, i.e. the distribution is bimodal and a
//! clear "silent window" appears. If the gate only defers, arrivals stay evenly spaced and the
//! large-gap bucket stays empty — same total count either way, which is exactly why counting failed.
use ndn_radio_drivers::{FrameIo, Rtl8733buBackend};
use std::sync::Arc;

#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    let ch: u8 = std::env::args()
        .nth(1)
        .and_then(|s| s.parse().ok())
        .unwrap_or(36);
    let secs: u64 = std::env::args()
        .nth(2)
        .and_then(|s| s.parse().ok())
        .unwrap_or(20);
    let dev = Arc::new(Rtl8733buBackend::open()?);
    dev.bring_up_monitor(ch)?;
    let _pump = dev.spawn_rx_pump(4);
    println!("ch{ch}: collecting RX hardware stamps for {secs}s");

    let mut stamps: Vec<u64> = Vec::new();
    let deadline = std::time::Instant::now() + std::time::Duration::from_secs(secs);
    while std::time::Instant::now() < deadline {
        if let Ok(Ok(f)) =
            tokio::time::timeout(std::time::Duration::from_millis(300), dev.recv_frame()).await
        {
            if let Some(s) = f.stamp {
                stamps.push(s.raw);
            }
        }
    }
    println!("frames with hardware stamps: {}", stamps.len());
    if stamps.len() < 20 {
        println!("too few to analyse — is the transmitter running?");
        return Ok(());
    }
    stamps.sort_unstable();
    // RXTSFL is a microsecond counter that wraps at 2^32; ignore the wrap by dropping negatives.
    let mut gaps: Vec<u64> = stamps
        .windows(2)
        .map(|w| w[1].wrapping_sub(w[0]))
        .filter(|&g| g < 5_000_000)
        .collect();
    gaps.sort_unstable();
    let pct = |p: f64| gaps[((gaps.len() - 1) as f64 * p) as usize];
    println!(
        "inter-arrival gaps (us): p50={} p90={} p99={} max={}",
        pct(0.5),
        pct(0.9),
        pct(0.99),
        gaps[gaps.len() - 1]
    );
    // Bucket by decade so a silent window shows up as mass in the millisecond buckets.
    let buckets = [100u64, 500, 1_000, 5_000, 10_000, 20_000, 50_000, u64::MAX];
    let names = [
        "<100us", "<500us", "<1ms", "<5ms", "<10ms", "<20ms", "<50ms", ">=50ms",
    ];
    let mut counts = [0usize; 8];
    for g in &gaps {
        for (i, b) in buckets.iter().enumerate() {
            if g < b {
                counts[i] += 1;
                break;
            }
        }
    }
    println!("gap histogram:");
    for (n, c) in names.iter().zip(counts) {
        if c > 0 {
            println!(
                "  {n:>8} {c:>6}  {:.1}%",
                100.0 * c as f64 / gaps.len() as f64
            );
        }
    }
    let big: usize = counts[3..].iter().sum();
    println!(
        "\ngaps >= 1ms: {big} ({:.1}%) — a shaped duty cycle puts real mass here; \
              pure deferral does not.",
        100.0 * big as f64 / gaps.len() as f64
    );
    Ok(())
}
