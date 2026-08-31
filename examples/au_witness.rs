//! **Witness receiver** — count the frames a transmitter actually puts ON AIR, per second.
//!
//! ## Why this exists
//!
//! Every throughput number this project has taken from the RTL8812AU is an *offered* rate: the
//! host's `inject` call count. That is not a transmission count, and on 2026-08-28 the difference
//! became decisive — a contention A/B at maximum contrast offered **4741 f/s** against a medium
//! that can carry at most **2436 f/s** at that posture (`rf_ab` gate G1). A bulk-OUT queue absorbs
//! writes, and this radio implements no `read_tx_counters()`, so nothing on the transmitter can
//! distinguish "the MAC transmitted" from "the host handed USB a buffer". Three INDISTINGUISHABLE
//! contention verdicts are equally consistent with a metric blind to the effect.
//!
//! The firmware route to a TX counter is closed for now: C2H reports must be enabled by an H2C
//! command and this driver has no H2C path (MEASURED: 0 C2H buffers in 12 s of flooding with
//! `NDN_C2H_DBG=1`).
//!
//! So: a second radio counts. On mds-o5p-0 the a81a sits on the same host as the 8812AU, and it
//! reaches ~2020 f/s of receive with a MEASURED coefficient of variation of 0.3 %, which is a far
//! better instrument than the thing being measured.
//!
//! ```text
//! # on the witness (a81a), then start the 8812AU flood:
//! au_witness 36 20
//! ```
//!
//! Prints per-second buckets, because an aggregate cannot tell a steady rate from a hard stall —
//! the lesson that turned an intractable "5x variance" into a one-run diagnosis on this fleet.
fn main() -> Result<(), Box<dyn std::error::Error>> {
    tokio::runtime::Runtime::new()?.block_on(run())
}

async fn run() -> Result<(), Box<dyn std::error::Error>> {
    use ndn_frame_io::{FrameFormat, FrameIo};
    use ndn_radio_drivers::LibUsbRtl88xxBackend;
    use std::time::{Duration, Instant};

    let mut a = std::env::args().skip(1);
    let ch: u8 = a.next().and_then(|s| s.parse().ok()).unwrap_or(36);
    let secs: u64 = a.next().and_then(|s| s.parse().ok()).unwrap_or(20);
    // The flood's payload filler. Counting only frames carrying it keeps ambient traffic out.
    let fill: u8 = std::env::var("NDN_WITNESS_FILL")
        .ok()
        .and_then(|s| u8::from_str_radix(s.trim_start_matches("0x"), 16).ok())
        .unwrap_or(0x42);

    // ★ Either radio can witness. A null from one receiver is not evidence until the pair has
    // been shown to hear each other at all — swap the roles and the positive control is free.
    let which = std::env::var("NDN_WITNESS_DEV").unwrap_or_else(|_| "xx".into());
    let dev: std::sync::Arc<dyn FrameIo> = if which == "au" {
        use ndn_radio_drivers::Rtl8812auBackend;
        let d = std::sync::Arc::new(Rtl8812auBackend::open()?.with_format(FrameFormat::Raw80211));
        d.bring_up_monitor(ch)?;
        d.spawn_rx_pump(8);
        d
    } else {
        let d = std::sync::Arc::new(
            LibUsbRtl88xxBackend::open_monitor(ch)?.with_format(FrameFormat::Raw80211),
        );
        d.spawn_rx_pump(8);
        d
    };
    println!(
        "witness: {which} monitor ch{ch}, {secs}s, counting frames whose payload is 0x{fill:02x} filler\n"
    );

    let mut buckets: Vec<(u32, u32)> = Vec::with_capacity(secs as usize);
    for _ in 0..secs {
        let stop = Instant::now() + Duration::from_secs(1);
        let (mut total, mut ours) = (0u32, 0u32);
        while Instant::now() < stop {
            let left = stop.saturating_duration_since(Instant::now());
            if left.is_zero() {
                break;
            }
            if let Ok(Ok(f)) =
                tokio::time::timeout(left.min(Duration::from_millis(200)), dev.recv_frame()).await
            {
                total += 1;
                // Instrument-first: show what is actually arriving, so a matcher that finds
                // nothing can be distinguished from a transmitter that sends nothing.
                if std::env::var_os("NDN_WITNESS_DUMP").is_some() && total <= 12 {
                    let p = &f.payload;
                    eprintln!(
                        "    rx len={} rssi={:?} head={:02x?}",
                        p.len(),
                        f.rssi_dbm,
                        &p[..p.len().min(28)]
                    );
                }
                // ⚠ Search ANYWHERE in the buffer, not at offset 0. The witness runs in
                // Raw80211 so `payload` is the whole 802.11 frame — header first, filler after —
                // and matching at the start found nothing while the transmitter was working fine.
                // 16 consecutive filler bytes is enough that ambient traffic will not match.
                if f.payload.windows(16).any(|w| w.iter().all(|&b| b == fill)) {
                    ours += 1;
                }
            }
        }
        buckets.push((ours, total));
    }
    let ours: u32 = buckets.iter().map(|b| b.0).sum();
    let total: u32 = buckets.iter().map(|b| b.1).sum();
    println!("  per-second (ours/total):");
    println!(
        "    {}",
        buckets
            .iter()
            .map(|(o, t)| format!("{o}/{t}"))
            .collect::<Vec<_>>()
            .join("  ")
    );
    println!(
        "\n  ON AIR: {ours} frames in {secs}s = {:.0} f/s   (all traffic: {total} = {:.0} f/s)",
        ours as f64 / secs as f64,
        total as f64 / secs as f64
    );
    println!("  ⚠ This is a RECEIVED count — a lower bound on what was transmitted, since the");
    println!("    witness has its own RX ceiling. Compare arms, not absolutes.");
    Ok(())
}
