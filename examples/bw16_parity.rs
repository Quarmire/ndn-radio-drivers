//! **BW16 (RTL8720DN) named-radio parity check** — drive every HAL seam on the real board.
//!
//! The wire protocol can be exercised with a Python script; this exercises the *contract*: the same
//! `FrameIo` / `RadioKnobs` / `RadioTime` / `RadioProfile` methods cognition calls. If a knob
//! actuates here, it actuates for the forwarder.
//!
//! Witness the TX side with an ESP32-C5 running `firmware/esp32c5-ndn`: its per-frame PHY metadata
//! (RSSI, noise, rate code, PHY format) reports what actually reached the air, which is the only way
//! to tell a knob that moved a register from a knob that moved the radio.
//!
//! ```sh
//! cargo run --example bw16_parity --features serial-radio -- /dev/cu.usbserial-XXXX
//! ```
use std::time::Duration;

use bytes::Bytes;
use ndn_radio_drivers::Bw16SerialBackend;
use ndn_radio_hal::{
    Bandwidth, FrameIo, InjectFrame, McsDescriptor, RadioKnobs, RadioProfile, RadioTime, TxIntent,
};

async fn burst(
    dev: &Bw16SerialBackend,
    tag: &str,
    n: usize,
) -> Result<(), Box<dyn std::error::Error>> {
    for i in 0..n {
        let payload = format!("\x05\x10{tag}{i:02}");
        dev.inject(InjectFrame::broadcast(
            Bytes::copy_from_slice(payload.as_bytes()),
            TxIntent::CONSERVATIVE,
        ))
        .await?;
        tokio::time::sleep(Duration::from_millis(12)).await;
    }
    Ok(())
}

#[tokio::main(flavor = "multi_thread")]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    let port = std::env::args()
        .nth(1)
        .unwrap_or_else(|| "/dev/cu.usbserial-11110".into());
    let dev = Bw16SerialBackend::open(&port)?;

    // --- profile: the planner's view of this radio -------------------------
    let cap = dev.capability();
    println!("BW16 open on {port}");
    println!("  bands       {:?}", cap.bands);
    println!("  channels    {:?}", cap.channels);
    println!("  max HT MCS  {:?}", cap.max_mcs());
    println!("  tx discipline {:?}", dev.tx_discipline());
    println!("  time sources  {:?}", dev.time_sources());

    // Fully-qualified: the inherent 1-arg `set_channel` shadows the trait method.
    RadioKnobs::set_channel(&dev, 6, Bandwidth::Bw20)?;

    // --- the rate lever, through FrameIo::set_rate (what cognition calls) ---
    println!("\nrate lever (FrameIo::set_rate — witness the PHY format on a C5):");
    for idx in [0u8, 3, 7] {
        dev.set_rate(McsDescriptor::ht(idx))?;
        burst(&dev, &format!("R{idx}"), 20).await?;
        println!("  requested HT MCS{idx}");
    }

    // --- the power lever, with a register readback as the proof ------------
    println!("\npower lever (RadioKnobs::set_tx_power, then read the registers back):");
    for idx in [24u32, 60, 100] {
        dev.set_tx_power(ndn_radio_hal::PowerRequest::index(idx as u8))?;
        tokio::time::sleep(Duration::from_millis(200)).await;
        match dev.read_txpower().await {
            Some((st, live)) => println!(
                "  set {idx:3} -> status {st}, live TXAGC: CCK {:?} OFDM {:?} HT {:?}",
                &live[0..4],
                &live[4..12],
                &live[12..20]
            ),
            None => println!("  set {idx:3} -> no readback (old firmware?)"),
        }
        burst(&dev, &format!("P{idx}"), 15).await?;
    }
    dev.set_txpower(0xFF)?; // hand power back to the driver

    // --- occupancy ---------------------------------------------------------
    tokio::time::sleep(Duration::from_millis(500)).await;
    println!("\nchannel activity: {:?}", dev.read_channel_activity()?);

    // --- scheduled TX, and what the device says it actually managed --------
    println!("\nscheduled TX (FrameIo::inject_after, 20 ms out):");
    for i in 0..8u32 {
        dev.inject_after(
            InjectFrame::broadcast(
                Bytes::copy_from_slice(format!("\x05\x10SCHED{i:02}").as_bytes()),
                TxIntent::CONSERVATIVE,
            ),
            20_000,
        )
        .await?;
        tokio::time::sleep(Duration::from_millis(120)).await;
    }
    let mut errs = Vec::new();
    while let Ok(Some((target, actual))) =
        tokio::time::timeout(Duration::from_millis(300), dev.inner().recv_tx_confirm()).await
    {
        errs.push(actual as i64 - target as i64);
    }
    if errs.is_empty() {
        println!("  no T_TXTIME confirmations");
    } else {
        let mean = errs.iter().sum::<i64>() as f64 / errs.len() as f64;
        println!(
            "  n={} scheduling error mean {mean:.1} us (max {}, min {})",
            errs.len(),
            errs.iter().max().unwrap(),
            errs.iter().min().unwrap()
        );
    }

    // --- RX: per-frame metadata off the air --------------------------------
    println!("\nRX for 5 s (inject 0x8624 frames from another radio to populate this):");
    let deadline = tokio::time::Instant::now() + Duration::from_secs(5);
    let (mut n, mut rssi_sum) = (0u32, 0i32);
    let mut rates = std::collections::BTreeMap::<String, u32>::new();
    let mut stamps: Vec<u64> = Vec::new();
    // The latch point of the first stamp, so the number below is printed with what it MEANS. The
    // BW16 is host-stamped (its `T_RX_TS` value is a software counter read inside the vendor blob's
    // RX callback, not a MAC latch), so its `raw` is nanoseconds since process start; the C5's is a
    // device µs counter. Assuming one unit for both is how a 1000x error gets printed as a fact.
    let mut latch = None;
    while tokio::time::Instant::now() < deadline {
        match tokio::time::timeout(Duration::from_millis(400), dev.recv_frame()).await {
            Ok(Ok(cap)) => {
                n += 1;
                if let Some(r) = cap.rssi_dbm {
                    rssi_sum += r as i32;
                }
                *rates
                    .entry(match cap.mcs_index {
                        Some(m) => format!("HT MCS{m}"),
                        None => "legacy".into(),
                    })
                    .or_default() += 1;
                if let Some(s) = cap.stamp {
                    stamps.push(s.raw);
                    latch.get_or_insert((s.latch, s.domain));
                }
            }
            _ => continue,
        }
    }
    println!("  frames {n}");
    if n > 0 {
        println!("  mean rssi {:.1} dBm", rssi_sum as f64 / n as f64);
        println!("  rates {rates:?}");
    }
    if stamps.len() > 2 {
        let monotonic = stamps.windows(2).all(|w| w[1] >= w[0]);
        let gaps: Vec<i64> = stamps
            .windows(2)
            .map(|w| w[1] as i64 - w[0] as i64)
            .collect();
        let med = {
            let mut g = gaps.clone();
            g.sort_unstable();
            g[g.len() / 2]
        };
        println!(
            "  rx stamps {:?} monotonic={monotonic}, median gap {med} raw units",
            latch
        );
    }
    Ok(())
}
