//! How long does `set_channel` actually take on THIS radio?
//!
//! The 8733b inherits `retune_us: Some(16_000)` from the generic 5 GHz capability helper — a figure
//! measured on a DIFFERENT part (#97). `RadioCapability::can_hop` divides by it, so a planner would
//! be deciding whether this radio can hop using another chip's timing. That field's own rule is
//! "populate only from a real measurement", and inheritance quietly launders someone else's.
//!
//! Measures same-band and cross-band retunes separately, since a band switch re-runs more of the
//! RF path, and reports the distribution rather than a single mean — a hop budget cares about the
//! tail, not the average.
use ndn_radio_drivers::{RadioKnobs, Rtl8733buBackend};
use ndn_radio_hal::Bandwidth;
use std::time::Instant;

fn stats(mut v: Vec<u128>) -> (u128, u128, u128) {
    v.sort_unstable();
    (v[v.len() / 2], v[v.len() * 9 / 10], v[v.len() - 1])
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let n: usize = std::env::args()
        .nth(1)
        .and_then(|s| s.parse().ok())
        .unwrap_or(40);
    let dev = Rtl8733buBackend::open()?;
    dev.bring_up_monitor(36)?;

    for (label, chans) in [
        ("5 GHz same-band (36<->40)", vec![36u8, 40]),
        ("5 GHz wider hop (36<->161)", vec![36, 161]),
        ("cross-band (36<->6)", vec![36, 6]),
    ] {
        let mut t = Vec::with_capacity(n);
        for i in 0..n {
            let ch = chans[i % chans.len()];
            let t0 = Instant::now();
            dev.set_channel(ch, Bandwidth::Bw20)?;
            t.push(t0.elapsed().as_micros());
        }
        let (p50, p90, max) = stats(t);
        println!("{label:<28} p50={p50:>7} us  p90={p90:>7} us  max={max:>7} us");
    }
    dev.set_channel(36, Bandwidth::Bw20)?;
    Ok(())
}
