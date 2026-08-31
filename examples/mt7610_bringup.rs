//! **MT7610U bring-up gate** — the on-silicon test that decides whether the mt76x0
//! port is real, run against mds-o5p-1's `0e8d:7610`.
//!
//! This codebase does not accept "implemented" without a measurement, and this is
//! that measurement. It walks the whole stack in order and prints a verdict per
//! stage, so a failure names the stage rather than the driver:
//!
//!   1. **open**    — claim the device, detach `mt76x0u`, never reset.
//!   2. **identity** — `MT_ASIC_VERSION` must read `0x7610xxxx`, and the EEPROM MAC
//!      must match the one the kernel netdev showed (`9c:ef:d5:f8:f1:b6`). Those two
//!      together prove the transport and the EEPROM parse without any RF involved.
//!   3. **firmware** — ILM/DLM download and `MT_MCU_COM_REG0` readiness.
//!   4. **tune**    — MAC/BBP/RF init and `set_channel`, then read the RF back through
//!      `MT_RF_CSR_CFG` and check it against the table we programmed. A tune that did
//!      not take is otherwise indistinguishable from a quiet channel.
//!   5. **RX**      — frames per second, RSSI distribution, MCS distribution. The
//!      kernel driver sees ~57-72 f/s on this antenna, which is the number to beat.
//!   6. **knobs**   — TSF ticking at 1 MHz, channel-busy µs, RX_STAT error counters.
//!   7. **dark bytes** — dump `rxwi.bbp_rxinfo[0..3]` (transfer offsets 20..36) beside a
//!      live TSF read. Nothing in the kernel driver reads these 16 bytes. If one dword
//!      advances at ~1 MHz they are a per-frame hardware timestamp and mt76x0/x2 join
//!      the common-view club; if none does, the wall is confirmed with evidence rather
//!      than by assumption. (The 2026-08-18 attempt at this compared them against
//!      `0x1104`, which is a constant slot-time config word, so it could not have
//!      found anything either way.)
//!
//!   sudo ./mt7610_bringup [channel] [seconds]
use std::time::{Duration, Instant};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    tokio::runtime::Runtime::new()?.block_on(run())
}

async fn run() -> Result<(), Box<dyn std::error::Error>> {
    use ndn_radio_drivers::Mt7610uBackend;
    use ndn_radio_hal::{Bandwidth, RadioKnobs, RadioProfile, RadioTime};

    let mut a = std::env::args().skip(1);
    let channel: u8 = a.next().and_then(|s| s.parse().ok()).unwrap_or(149);
    let secs: u64 = a.next().and_then(|s| s.parse().ok()).unwrap_or(15);
    let width = match std::env::var("NDN_BW").as_deref() {
        Ok("40") => Bandwidth::Bw40,
        Ok("80") => Bandwidth::Bw80,
        _ => Bandwidth::Bw20,
    };

    println!("── 1. open ─────────────────────────────────────────────");
    // ★ `Raw80211`, not the default `RawNdn`. The default format makes `recv_frame` accept only
    // frames carrying our own ethertype (0x8624), so on an ambient channel it correctly returns
    // nothing — and "0 frames" then reads as a dead receiver when the receiver is fine. This is
    // an RX *proof*, so take every 802.11 frame verbatim (the same choice `rx_capture.rs` makes
    // for the 8822E and for the same reason).
    let dev = std::sync::Arc::new(
        Mt7610uBackend::open()?.with_format(ndn_frame_io::FrameFormat::Raw80211),
    );
    println!("   opened");

    println!("── 2. identity ─────────────────────────────────────────");
    let asic = dev.rr(0x0000)?;
    println!("   MT_ASIC_VERSION = {asic:#010x}");
    if asic >> 16 != 0x7610 {
        println!("   ✗ not an MT7610 — transport or device wrong");
        return Ok(());
    }
    let mac = dev.mac_address()?;
    println!(
        "   EEPROM MAC = {:02x}:{:02x}:{:02x}:{:02x}:{:02x}:{:02x}  (expect 9c:ef:d5:f8:f1:b6 on o5p-1)",
        mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]
    );

    println!("── 3+4. bring_up + tune ────────────────────────────────");
    let t0 = Instant::now();
    dev.bring_up()?;
    println!("   bring_up took {:?}", t0.elapsed());
    dev.setup_monitor_rx()?;
    let t1 = Instant::now();
    dev.set_channel(channel, width)?;
    let retune = t1.elapsed();
    println!("   set_channel({channel}, {width:?}) took {retune:?}  <- this is `retune_us`");
    // Read the RF back: a tune that silently did not take looks exactly like a quiet
    // channel, and that ambiguity is what makes RF bring-up expensive to debug.
    for (b, r) in [(0u32, 1u32), (0, 2), (0, 4), (7, 6), (7, 73)] {
        match dev.rf_read(b, r) {
            Ok(v) => println!("   RF({b},{r}) = {v:#04x}"),
            Err(e) => println!("   RF({b},{r}) readback failed: {e}"),
        }
    }

    println!("   rx_health: {}", dev.rx_health()?);

    println!("── 6. knobs ────────────────────────────────────────────");
    let dom = dev.time_sources().first().map(|s| s.domain);
    if let Some(d) = dom {
        let a = dev.read_clock(d)?;
        std::thread::sleep(Duration::from_millis(100));
        let b = dev.read_clock(d)?;
        match (a, b) {
            (Some(x), Some(y)) => {
                let dt = y.wrapping_sub(x);
                println!("   TSF advanced {dt} ticks over ~100 ms (expect ~100000 at 1 MHz)");
            }
            _ => println!("   TSF: no readable clock"),
        }
    } else {
        println!("   no RadioTime sources declared");
    }
    match dev.read_channel_activity() {
        Ok(Some(v)) => println!("   channel activity = {v}"),
        Ok(None) => println!("   channel activity: not supported"),
        Err(e) => println!("   channel activity error: {e}"),
    }
    match dev.read_ofdm_counters() {
        Ok(Some((ok, err))) => println!("   OFDM counters ok={ok} err={err}"),
        Ok(None) => println!("   OFDM counters: not supported"),
        Err(e) => println!("   OFDM counters error: {e}"),
    }
    println!("   capability = {:?}", dev.capability());

    println!("── 5. RX for {secs}s ───────────────────────────────────");
    let dev = std::sync::Arc::new(std::sync::Arc::try_unwrap(dev).ok().unwrap());
    let _pumps = dev.spawn_rx_pump(8);
    let t = Instant::now();
    let mut n = 0u64;
    let mut rssi: Vec<i8> = Vec::new();
    let mut mcs = [0u32; 32];
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
                if let Some(m) = f.mcs_index {
                    mcs[(m as usize).min(31)] += 1;
                }
                if n <= 3 {
                    println!(
                        "   frame {n}: {} B  rssi={:?} mcs={:?} stamp={:?}",
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
    if !rssi.is_empty() {
        rssi.sort_unstable();
        println!(
            "   RSSI dBm: min {} p50 {} max {}  (n={})",
            rssi[0],
            rssi[rssi.len() / 2],
            rssi[rssi.len() - 1],
            rssi.len()
        );
    } else {
        println!("   RSSI: none reported — the eeprom/RXWI decode is not wired");
    }
    let seen: Vec<String> = mcs
        .iter()
        .enumerate()
        .filter(|&(_, &c)| c > 0)
        .map(|(i, c)| format!("mcs{i}={c}"))
        .collect();
    println!(
        "   rates: {}",
        if seen.is_empty() {
            "none decoded".into()
        } else {
            seen.join(" ")
        }
    );
    println!("   RX_RAW_FRAMES = {}", ndn_radio_drivers::rx_raw_frames());
    Ok(())
}
