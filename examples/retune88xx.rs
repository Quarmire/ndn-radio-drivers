//! Real `set_channel` cost for the RTL8812EU/8822E (a81a) — it was inheriting another part's.
//!
//! Same method as `retune8733b.rs`: `retune_us` feeds `can_hop`/`retune_overhead`, so an inherited
//! figure means hop decisions for THIS radio are sized by a different chip's timing. Reports the
//! distribution, because a hop budget is set by the tail rather than the median.
use ndn_radio_drivers::{LibUsbRtl88xxBackend, RadioKnobs};
use ndn_radio_hal::Bandwidth;
use std::time::Instant;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let n: usize = std::env::args()
        .nth(1)
        .and_then(|s| s.parse().ok())
        .unwrap_or(40);
    let dev = LibUsbRtl88xxBackend::open_monitor(36)?;
    for (label, chans) in [
        ("5 GHz same-band (36<->40)", vec![36u8, 40]),
        ("5 GHz wider hop (36<->161)", vec![36, 161]),
    ] {
        let mut t = Vec::with_capacity(n);
        for i in 0..n {
            let t0 = Instant::now();
            // Trait method explicitly: the backend has an inherent `set_channel` taking ChannelBw
            // that would otherwise shadow it, and the trait path is what the scheduler calls.
            RadioKnobs::set_channel(&dev, chans[i % chans.len()], Bandwidth::Bw20)?;
            t.push(t0.elapsed().as_micros());
        }
        t.sort_unstable();
        println!(
            "{label:<28} p50={:>7} p90={:>7} max={:>7} us",
            t[t.len() / 2],
            t[t.len() * 9 / 10],
            t[t.len() - 1]
        );
    }
    RadioKnobs::set_channel(&dev, 36, Bandwidth::Bw20)?;
    Ok(())
}
