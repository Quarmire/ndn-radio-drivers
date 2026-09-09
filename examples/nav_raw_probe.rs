//! **Does the radio's own MAC preserve a chosen Duration/ID?** — the silicon half of #96.
//!
//! The earlier answer came from two different places and they do not measure the same thing:
//!
//!  * Through **our libusb driver** the RTL8812AU overwrites the injected Duration (beacon → 60 µs,
//!    QoS-data → 124 µs, MEASURED 6058/6058) unless the TX descriptor's `NAVUSEHDR` bit is set, and
//!    with it every one of 6144 frames carried our `0x1234` verbatim. That is a *hardware* verdict:
//!    nothing between the descriptor and the air rewrote the field.
//!  * Through **mac80211** (af-packet monitor injection on ath9k_htc and mt76x0u) the field came out
//!    314 µs and 0 µs respectively. That is *not* a hardware verdict — `ieee80211_duration()`
//!    recomputes `hdr->duration_id` in software on the way down, so the silicon was never asked.
//!
//! This binary closes that gap for the mt76 parts, whose userspace driver bypasses mac80211
//! entirely: it hand-builds a complete 802.11 data frame carrying `NDN_DUR` and ships it through
//! `FrameFormat::Raw80211`, which is a verbatim passthrough. Whatever a neutral monitor reads back
//! is what the MT7610U's MAC did with it.
//!
//! ⚠ Probe with a value the MAC could not itself produce. A group-addressed frame's *correct*
//! computed Duration is 0, so reading 0 off a broadcast proves nothing on its own — the injected
//! value is non-zero precisely so that "unchanged" and "recomputed" are distinguishable.
//!
//!   sudo -E ./nav_raw_probe [channel] [secs]
//! env: NDN_DUR (hex, default 1234), NDN_TX_MCS (default 4)
use ndn_frame_io::{FrameFormat, FrameIo, InjectFrame, Reliability, TxIntent};
use ndn_radio_drivers::Mt7610uBackend;
use ndn_radio_hal::McsDescriptor;
use std::sync::Arc;
use std::time::{Duration, Instant};

/// Locally administered, and distinct from `nav_probe`'s `…88:12` so a capture can tell the two
/// transmitters apart if both ever run on one channel. Not a host identity.
const SRC: [u8; 6] = [0x02, 0x4e, 0x44, 0x4e, 0x88, 0x13];

fn build_data(dur: u16, seq: u16) -> Vec<u8> {
    let mut f = Vec::with_capacity(24 + 64);
    f.extend_from_slice(&[0x08, 0x00]); // FC: data, ToDS=FromDS=0
    f.extend_from_slice(&dur.to_le_bytes()); // ← the field under test
    f.extend_from_slice(&[0x03, 0x11, 0x22, 0x33, 0x44, 0x55]); // addr1 (group)
    f.extend_from_slice(&SRC); // addr2
    f.extend_from_slice(&[0x03, 0xbb, 0xcc, 0xdd, 0xee, 0xff]); // addr3
    // A FIXED, distinctive sequence number (0xABC), not a counter: the second field under test.
    // `tier0.rs` reserves SeqCtrl for "LP reassembly", but the RX header walk never reads it (it
    // steps over SeqCtrl via `hdr_len`), and the Realtek TX path sets HWSEQ_EN so the MAC writes it
    // regardless. If a MAC leaves this value alone it is 16 more recyclable bits.
    let _ = seq;
    f.extend_from_slice(&(0x0abcu16 << 4).to_le_bytes()); // seq ctl
    f.extend_from_slice(&[0x42u8; 64]);
    f
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    tokio::runtime::Runtime::new()?.block_on(run())
}

async fn run() -> Result<(), Box<dyn std::error::Error>> {
    let mut a = std::env::args().skip(1);
    let channel: u8 = a.next().and_then(|s| s.parse().ok()).unwrap_or(6);
    let secs: u64 = a.next().and_then(|s| s.parse().ok()).unwrap_or(15);
    let dur = u16::from_str_radix(
        std::env::var("NDN_DUR")
            .unwrap_or_else(|_| "1234".into())
            .trim_start_matches("0x"),
        16,
    )?;
    let mcs: u8 = std::env::var("NDN_TX_MCS")
        .ok()
        .and_then(|v| v.parse().ok())
        .unwrap_or(4);

    let dev = Arc::new(Mt7610uBackend::open()?.with_format(FrameFormat::Raw80211));
    // ★ M5: `bring_up(ch)` IS the plan — firmware/init + monitor RX + the tune, one sequence.
    dev.bring_up(channel)?;
    let mut m = McsDescriptor::ht(mcs);
    m.nss = 1; // 1x1 silicon
    FrameIo::set_rate(dev.as_ref(), m)?;
    println!(
        "mt7610u ch{channel} HT MCS{mcs}, injecting Duration={dur:#06x} ({dur} µs) for {secs}s"
    );

    let t0 = Instant::now();
    let (mut n, mut seq) = (0u64, 0u16);
    while t0.elapsed() < Duration::from_secs(secs) {
        let f = InjectFrame {
            payload: build_data(dur, seq).into(),
            tx: TxIntent {
                reliability: Reliability::Balanced,
                ..Default::default()
            },
            dst: [0x03, 0x11, 0x22, 0x33, 0x44, 0x55],
            src: SRC,
            addr3: None,
            extra: None,
            htc: None,
        };
        if FrameIo::inject(dev.as_ref(), f).await.is_ok() {
            n += 1;
        }
        seq = seq.wrapping_add(1);
        tokio::time::sleep(Duration::from_micros(2000)).await;
    }
    println!("done: {n} frames in {:.2}s", t0.elapsed().as_secs_f64());
    Ok(())
}
