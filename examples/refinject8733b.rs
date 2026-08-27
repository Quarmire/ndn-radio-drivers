//! Reference transmitter for the 8733b absolute TX-power calibration (substitution method).
//!
//! Injects OUR on-air format at **6 Mbps OFDM** — the rate the 8733b transmits at, which matters
//! because TX power is per-rate on these parts — from a mac80211 radio whose applied power
//! nl80211 reports. Measured on the same meter, through the same channel and rate, the unknown
//! (path loss + meter offset) cancels in the difference:
//!
//!     K            = P_ref - RSSI_ref
//!     TX_dut(idx)  = RSSI_dut(idx) + K
//!
//! Payload/src are tagged (`p[0]=idx`, `p[1]=knob`, `p[2]=0xC3`) so `rxpwr_bucket` buckets these
//! arms exactly like the DUT's.
//!
//! Usage: sudo ./refinject8733b <iface> <knob> <idx> [count] [size]
#[cfg(target_os = "linux")]
#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    use bytes::Bytes;
    use ndn_frame_io::AfPacketBackend;
    use ndn_radio_drivers::{BROADCAST, FrameFormat, InjectFrame, TxIntent, frame, radiotap};

    let mut a = std::env::args().skip(1);
    let iface = a
        .next()
        .ok_or("usage: refinject8733b <iface> <knob> <idx> [count] [size]")?;
    let knob: u8 = a.next().and_then(|s| s.parse().ok()).unwrap_or(5);
    let idx: u8 = a.next().and_then(|s| s.parse().ok()).unwrap_or(0);
    let count: usize = a.next().and_then(|s| s.parse().ok()).unwrap_or(1200);
    let size: usize = a.next().and_then(|s| s.parse().ok()).unwrap_or(300);
    // radiotap RATE in 500 kbps units: 12 = 6 Mbps OFDM (matches the 8733b), 2 = 1 Mbps CCK.
    let rate: u8 = a.next().and_then(|s| s.parse().ok()).unwrap_or(12);
    // Microseconds between frames. 0 = flood. A LOW-RATE tick matters when this is used as a shared
    // clock source: a flooding source saturates the medium it is meant to time, which silently
    // turns a MAC experiment into an interference experiment.
    let pace_us: u64 = a.next().and_then(|s| s.parse().ok()).unwrap_or(0);

    let fmt = FrameFormat::RawNdn {
        ethertype: ndn_radio_drivers::NDN_ETHERTYPE,
    };
    let backend = AfPacketBackend::new(&iface, fmt)?;
    let mut p = vec![0xC3u8; size];
    p[0] = idx;
    p[1] = knob;
    p[2] = 0xC3;
    // p[4..8] = little-endian frame sequence. Two receivers hearing the SAME transmission can then
    // match frame-for-frame, which is what makes a common-view clock comparison possible: the
    // transmitter's own clock cancels and only the ratio of the two receivers' crystals is left.
    let f = InjectFrame {
        payload: Bytes::from(p.clone()),
        tx: TxIntent::CONSERVATIVE,
        dst: BROADCAST,
        src: [0x02, 0x50, 0x33, 0x02, knob, idx],
        addr3: None,
        addr4: None,
        htc: None,
    };
    // Explicit legacy RATE header: `frame::build` would emit the HT TX header and the reference
    // would secretly transmit at a different rate than the DUT, which is the one thing this
    // measurement cannot tolerate.
    let hdr = radiotap::build_tx_legacy(rate);
    let dot11 = frame::build_dot11(fmt, &f)?;
    let mut buf = Vec::with_capacity(hdr.len() + dot11.len());
    buf.extend_from_slice(&hdr);
    buf.extend_from_slice(&dot11);
    let mut sent = 0usize;
    let mut first_err = None;
    for seq in 0..count {
        // Rebuild per frame so the sequence advances; the radiotap+dot11 prefix is unchanged.
        let mut pl = p.clone();
        pl[4..8].copy_from_slice(&(seq as u32).to_le_bytes());
        let fseq = InjectFrame {
            payload: Bytes::from(pl),
            ..f.clone()
        };
        let mut dot11 = frame::build_dot11(fmt, &fseq)?;
        // NDN_NAV_US sets the 802.11 Duration field (header bytes 2..4) so this can act as a NAV
        // interferer: a receiver that honours a decoded NAV must defer for that long.
        if let Ok(us) = std::env::var("NDN_NAV_US") {
            let d: u16 = us.parse().unwrap_or(0);
            if dot11.len() >= 4 {
                dot11[2..4].copy_from_slice(&d.to_le_bytes());
            }
        }
        let mut buf = Vec::with_capacity(hdr.len() + dot11.len());
        buf.extend_from_slice(&hdr);
        buf.extend_from_slice(&dot11);
        if pace_us > 0 {
            tokio::time::sleep(std::time::Duration::from_micros(pace_us)).await;
        }
        match backend.inject_raw(&buf).await {
            Ok(()) => sent += 1,
            // Report WHY, once. A bare "sent=0" is indistinguishable from a dead radio, and cost a
            // debugging round when the monitor interface had simply been left DOWN.
            Err(e) => {
                first_err.get_or_insert(e.to_string());
            }
        }
    }
    if let Some(e) = first_err {
        eprintln!(
            "  ref: {} of {count} injects failed, first error: {e}",
            count - sent
        );
    }
    println!("  ref knob={knob} idx={idx} rate={rate}(x500kbps): sent={sent}/{count}");
    Ok(())
}

#[cfg(not(target_os = "linux"))]
fn main() {
    eprintln!("refinject8733b requires Linux AF_PACKET monitor-mode injection (CAP_NET_RAW).");
    std::process::exit(1);
}
