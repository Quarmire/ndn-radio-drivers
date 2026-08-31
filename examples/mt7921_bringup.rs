//! **MT7921AU bring-up gate** — the on-silicon test for the connac2 port, run against
//! mds-o5p-3's `0e8d:7961`.
//!
//! Stages, each printing a verdict so a failure names the stage rather than the driver:
//!   1. **open** — claim the class `ff/ff/ff` WLAN interface *only*. Interfaces 0-2 are a
//!      Bluetooth radio owned by `btusb` and must still be bound when this exits; the harness
//!      checks that separately, because quietly stealing someone's Bluetooth would be a rude
//!      way to pass a Wi-Fi test.
//!   2. **identity** — `MT_HW_CHIPID` must read `0x7961` and `MT_HW_REV` `0x8a10`, the latter
//!      matching the `hw_sw_ver` in the patch blob's own header.
//!   3. **firmware** — the patch (92,192 B) and RAM code (791,588 B) download. This is the
//!      stage with real risk: it is the one that, on the older mt76 parts, wedged a dongle
//!      whenever it ran against an MCU that was already up.
//!   4. **monitor + tune**.
//!   5. ★ **RX and the timestamp.** The headline: RXD group 2 carries a per-frame hardware
//!      stamp (`mt7921/mac.c:307-309`). Nothing on the mt76x0/mt76x2 parts does. This measures
//!      how many frames actually carry it, and whether it advances at the rate a TSF should —
//!      which is what decides whether this radio can source common view or merely claims to.
//!   6. **traits** — the derived `FaceTimeProfile`, including `can_common_view`.
//!
//!   sudo ./mt7921_bringup [channel] [seconds]
use ndn_radio_drivers::Mt7921uBackend;
use ndn_radio_hal::{Bandwidth, FaceTimeProfile, RadioKnobs, RadioProfile, RadioTime};
use std::time::{Duration, Instant};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    tokio::runtime::Runtime::new()?.block_on(run())
}

async fn run() -> Result<(), Box<dyn std::error::Error>> {
    let mut a = std::env::args().skip(1);
    let channel: u8 = a.next().and_then(|s| s.parse().ok()).unwrap_or(6);
    let secs: u64 = a.next().and_then(|s| s.parse().ok()).unwrap_or(15);

    println!("── 1. open (WLAN interface only) ───────────────────────");
    // Raw80211, not the default RawNdn{0x8624}: this is an RX *proof*, and the NDN-ethertype
    // filter would correctly reject every ambient frame, making a working receiver read as dead.
    // (It did exactly that on the MT7610U — 1150 raw units pulled off USB, 0 frames delivered.)
    let dev = Mt7921uBackend::open()?.with_format(ndn_frame_io::FrameFormat::Raw80211);
    println!("   opened");

    println!("── 2. identity ─────────────────────────────────────────");
    let chipid = dev.rr(0x7001_0200)?;
    let hwrev = dev.rr(0x7001_0204)?;
    println!("   MT_HW_CHIPID = {chipid:#010x}   MT_HW_REV = {hwrev:#010x}");
    if chipid & 0xffff != 0x7961 {
        println!("   ✗ not an MT7921 — stopping");
        return Ok(());
    }
    println!(
        "   firmware_state = {:?}  running={}",
        dev.firmware_state(),
        dev.firmware_running()
    );

    println!("── 3. firmware download ────────────────────────────────");
    let t = Instant::now();
    match dev.bring_up() {
        Ok(()) => println!("   bring_up OK in {:?}", t.elapsed()),
        Err(e) => {
            println!("   ✗ bring_up failed after {:?}: {e}", t.elapsed());
            println!("   state now: {:?}", dev.firmware_state());
            return Ok(());
        }
    }
    println!(
        "   firmware_state = {:?}  running={}",
        dev.firmware_state(),
        dev.firmware_running()
    );

    println!("── 4. monitor + tune ───────────────────────────────────");
    // ★ Tune BEFORE monitor, not after. This backend rejects the other order outright, and it
    // is right to: the connac2 sniffer configuration carries its own copy of the channel, so a
    // monitor set up first would be pinned to a stale one — the same class of bug as the
    // MT7612U's channel replay silently overwriting its RX filter, caught here by a check
    // instead of by a week of "the radio hears nothing".
    let t = Instant::now();
    match RadioKnobs::set_channel(&dev, channel, Bandwidth::Bw20) {
        Ok(()) => println!("   set_channel({channel}, Bw20) in {:?}", t.elapsed()),
        Err(e) => {
            println!("   ✗ set_channel failed: {e}");
            return Ok(());
        }
    }
    dev.setup_monitor_rx()?;
    println!("   rx_health: {}", dev.rx_health()?);
    println!("   MAC = {:?}", dev.mac_address());

    println!("── 6. traits ───────────────────────────────────────────");
    let srcs = RadioTime::time_sources(&dev);
    for s in &srcs {
        println!("   clock: {:?}", s);
    }
    let prof = FaceTimeProfile::derive(&dev, RadioKnobs::tx_discipline(&dev));
    println!("   FaceTimeProfile = {prof:?}");
    println!(
        "   ★ can_common_view = {}  {}",
        prof.can_common_view,
        match (prof.can_common_view, prof.hw_rx_stamp) {
            (true, _) => "(a MediaTek first)",
            // The two ways to fail it, which used to be one: no hardware latch at all, or a
            // hardware latch whose reference nobody has established.
            (false, true) => "(the RXD stamp is wired; its clock REFERENCE is not established)",
            (false, false) => "(the RXD stamp is not wired)",
        }
    );
    println!("   clock reference = {:?}", prof.clock_reference);
    let cap = RadioProfile::capability(&dev);
    println!("   capability = {cap:?}");
    println!("   he_cap = {}", cap.he_cap());

    println!("── 5. RX for {secs}s ───────────────────────────────────");
    let dev = std::sync::Arc::new(dev);
    let _pumps = dev.spawn_rx_pump(8);
    let t = Instant::now();
    let (mut n, mut stamped) = (0u64, 0u64);
    let mut rssi: Vec<i8> = Vec::new();
    let mut first_stamp: Option<(u64, Instant)> = None;
    let mut last_stamp: Option<(u64, Instant)> = None;
    let mut modes: std::collections::BTreeMap<String, u32> = Default::default();
    while t.elapsed() < Duration::from_secs(secs) {
        match tokio::time::timeout(
            Duration::from_millis(500),
            ndn_radio_hal::FrameIo::recv_frame(dev.as_ref()),
        )
        .await
        {
            Ok(Ok(f)) => {
                n += 1;
                if let Some(r) = f.rssi_dbm {
                    rssi.push(r);
                }
                *modes
                    .entry(match f.mcs_index {
                        Some(m) => format!("mcs{m}"),
                        None => "legacy/none".into(),
                    })
                    .or_default() += 1;
                if let Some(st) = f.stamp {
                    stamped += 1;
                    let now = Instant::now();
                    if first_stamp.is_none() {
                        first_stamp = Some((st.raw, now));
                    }
                    last_stamp = Some((st.raw, now));
                }
                if n <= 3 {
                    println!(
                        "   frame {n}: {} B rssi={:?} mcs={:?} stamp={:?}",
                        f.payload.len(),
                        f.rssi_dbm,
                        f.mcs_index,
                        f.stamp
                    );
                }
            }
            Ok(Err(e)) => println!("   recv error: {e}"),
            Err(_) => {}
        }
    }
    let el = t.elapsed().as_secs_f64();
    println!("   {n} frames in {el:.1}s = {:.1} f/s", n as f64 / el);
    println!(
        "   ★ {stamped}/{n} carried a per-frame RX stamp ({:.0}%)",
        if n > 0 {
            stamped as f64 * 100.0 / n as f64
        } else {
            0.0
        }
    );
    if let (Some((t0, i0)), Some((t1, i1))) = (first_stamp, last_stamp)
        && i1 > i0
    {
        let host_us = i1.duration_since(i0).as_micros() as f64;
        let ticks = t1.wrapping_sub(t0) as f64;
        println!(
            "   ★ stamp rate: {ticks} ticks over {host_us:.0} host us = {:.4} MHz \
             (1.0 would confirm a microsecond TSF)",
            ticks / host_us
        );
    }
    if !rssi.is_empty() {
        rssi.sort_unstable();
        println!(
            "   RSSI dBm: min {} p50 {} max {} (n={})",
            rssi[0],
            rssi[rssi.len() / 2],
            rssi[rssi.len() - 1],
            rssi.len()
        );
    }
    println!("   rates: {modes:?}");
    println!("   RX_RAW_FRAMES = {}", ndn_radio_drivers::rx_raw_frames());
    Ok(())
}
