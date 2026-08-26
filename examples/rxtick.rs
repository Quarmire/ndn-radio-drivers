//! What is one RX-stamp tick worth, on ANY backend? — the generalised tick audit.
//!
//! `RadioTimeSource::free_run_rx_stamp(domain, tick_ns)` is a DECLARATION, and four backends
//! declare 1_000 without anyone having measured it. On the RTL8733BU both its clocks turned out to
//! be 4_000, which silently made every duration derived from RX stamps 4x short — including a
//! shipped shaping figure. 802.11 TSF is *specified* as 1 us, so several of these are probably
//! right; the point is to know which, not to assume.
//!
//! Method: bracket by the FIRST and LAST stamped frame (never the capture window — a transmitter
//! that stops early leaves dead time and inflates the result), and compare to the host clock.
//! Needs a transmitter of our frame format on the channel.
use ndn_radio_drivers::FrameIo;
use std::time::Instant;

#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    let pid = u16::from_str_radix(
        std::env::args().nth(1).unwrap_or_else(|| "f72b".into()).trim_start_matches("0x"),
        16,
    )?;
    let ch: u8 = std::env::args().nth(2).and_then(|s| s.parse().ok()).unwrap_or(36);
    let secs: u64 = std::env::args().nth(3).and_then(|s| s.parse().ok()).unwrap_or(20);
    let r = ndn_radio_drivers::open_named_radio(pid, ch)?;
    let declared = r
        .time
        .as_ref()
        .and_then(|t| t.time_sources().first().map(|s| s.tick_ns))
        .unwrap_or(0);
    println!("pid {pid:04x} ch{ch}: declares {declared} ns/tick");

    let (mut first, mut last, mut last_host, mut n) = (None::<(u64, u128)>, 0u64, 0u128, 0usize);
    let t0 = Instant::now();
    while t0.elapsed().as_secs() < secs {
        if let Ok(Ok(f)) =
            tokio::time::timeout(std::time::Duration::from_millis(300), r.io.recv_frame()).await
            && let Some(s) = f.stamp
        {
            first.get_or_insert((s.raw, t0.elapsed().as_micros()));
            last = s.raw;
            last_host = t0.elapsed().as_micros();
            n += 1;
        }
    }
    match first {
        Some((f0, h0)) if n > 100 && last > f0 => {
            let ticks = last.wrapping_sub(f0) as f64;
            let host_us = (last_host - h0) as f64;
            let measured = host_us * 1000.0 / ticks;
            println!("frames={n} host_span={:.2}s ticks={ticks:.0}", host_us / 1e6);
            println!(
                "=> MEASURED {measured:.1} ns/tick  ({:.2}x the declared {declared})",
                measured / f64::from(declared.max(1))
            );
        }
        _ => println!("too few stamped frames ({n}) — is a transmitter running on ch{ch}?"),
    }
    Ok(())
}
