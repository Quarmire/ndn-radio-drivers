//! What is one RX-stamp tick actually worth?
//!
//! `RadioTimeSource::free_run_rx_stamp(domain, 1_000)` declares 1000 ns. The port TSF turned out to
//! be 4 us/tick (8e95608); this checks whether the per-frame RX stamp has the same scale error,
//! which would silently multiply every duration derived from RX stamps by 4.
//!
//! Method: hold a wall-clock window open and compare it to the span of stamps collected inside it.
//! No transmitter assumptions — just the receiver's own clock against the host's.
use ndn_radio_drivers::{FrameIo, Rtl8733buBackend};
use std::sync::Arc;
use std::time::Instant;

#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    let ch: u8 = std::env::args().nth(1).and_then(|s| s.parse().ok()).unwrap_or(36);
    let secs: u64 = std::env::args().nth(2).and_then(|s| s.parse().ok()).unwrap_or(20);
    let dev = Arc::new(Rtl8733buBackend::open()?);
    dev.bring_up_monitor(ch)?;
    let _pump = dev.spawn_rx_pump(4);
    // Bracket by the FIRST and LAST stamped frame, not by the window: a transmitter that stops
    // early leaves dead time at the end, and dividing by the whole window inflates the tick.
    // (A first version did exactly that and reported 4883 ns instead of ~4000.)
    let (mut first, mut last, mut n) = (None::<(u64, u128)>, 0u64, 0usize);
    let mut last_host = 0u128;
    let t0 = Instant::now();
    while t0.elapsed().as_secs() < secs {
        if let Ok(Ok(f)) = tokio::time::timeout(std::time::Duration::from_millis(300), dev.recv_frame()).await
            && let Some(s) = f.stamp
        {
            first.get_or_insert((s.raw, t0.elapsed().as_micros()));
            last = s.raw;
            last_host = t0.elapsed().as_micros();
            n += 1;
        }
    }
    match first {
        Some((f0, h0)) if n > 100 => {
            let ticks = last.wrapping_sub(f0) as f64;
            let host_us = (last_host - h0) as f64;
            println!("frames={n} host_span={:.2}s stamp_ticks={ticks:.0}", host_us / 1e6);
            println!("=> {:.1} ns per tick (declared 1000)", host_us * 1000.0 / ticks);
        }
        _ => println!("too few stamped frames ({n}) — need a transmitter on ch{ch}"),
    }
    Ok(())
}
