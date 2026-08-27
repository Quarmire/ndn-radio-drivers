//! Emit a **known-good** stream of 1×1 / HT20 frames on a kernel monitor interface, so a DUT's HT
//! demodulator can be tested against traffic whose validity is independently established.
//!
//! This is the instrument the 8733b HT-RX question lacked in 2026-07 and which left it parked as
//! INCONCLUSIVE rather than answered. Back then the HT source was one of our own userspace Realtek
//! drivers, so "the 8733b saw no HT" could not be separated from "no valid HT PPDU was ever emitted"
//! — there was no verified-good 1SS/HT20 transmitter co-located with the DUT. Here the transmitter
//! is **mac80211** (which builds the PPDU from the radiotap MCS field, and is not our code), and a
//! third kernel-driven radio in monitor mode reads the MCS back off the air, so what is on the air
//! is established without reference to any driver under test.
//!
//! Two arms, because a silent DUT is only interpretable if the same path is known to carry anything:
//!   `ht<N>`  — HT MCS N, 1 spatial stream, 20 MHz, long GI (the treatment)
//!   `vht<N>` — VHT MCS N, 1 spatial stream, 20 MHz, long GI (802.11ac)
//!   `legacy` — a legacy OFDM rate via the radiotap RATE field (the positive control: proves this
//!              transmitter reaches this DUT at all, so an HT-only null means the HT demod, not the link)
//!
//! Prereqs: `iw dev <if> set type monitor && ip link set <if> up && iw dev <if> set channel <ch>`
//! Usage:   sudo ./inject_ht <iface> <ht0..ht7|legacy> [count] [size]

#[cfg(target_os = "linux")]
#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    use bytes::Bytes;
    use ndn_frame_io::AfPacketBackend;
    use ndn_radio_drivers::{
        BROADCAST, DEFAULT_SRC, FrameFormat, FrameIo, InjectFrame, McsDescriptor, TxIntent, frame,
        radiotap,
    };

    let mut args = std::env::args().skip(1);
    let iface = args.next().unwrap_or_else(|| {
        eprintln!("usage: inject_ht <iface> <ht0..ht7|legacy> [count] [size]");
        std::process::exit(2);
    });
    let mode = args.next().unwrap_or_else(|| "ht0".into());
    let count: usize = args.next().and_then(|s| s.parse().ok()).unwrap_or(4000);
    let size: usize = args.next().and_then(|s| s.parse().ok()).unwrap_or(400);

    let fmt = FrameFormat::RawNdn {
        ethertype: ndn_radio_drivers::NDN_ETHERTYPE,
    };
    let backend = AfPacketBackend::new(&iface, fmt)?;
    let payload = Bytes::from(vec![0xA5u8; size]);
    let f = InjectFrame::broadcast(payload, TxIntent::CONSERVATIVE);

    println!(
        "inject_ht iface={iface} mode={mode} count={count} size={size}\n\
         src={DEFAULT_SRC:02x?} dst={BROADCAST:02x?}  (filter a witness on the src MAC)"
    );

    if let Some(n) = mode.strip_prefix("vht") {
        // VHT cannot go through `build_at` either: that path calls the HT radiotap builder and
        // drops `McsDescriptor::vht` on the floor, so a "VHT" frame would silently go out as HT
        // and the arm would measure nothing. Build the VHT radiotap header explicitly.
        let index: u8 = n.parse().unwrap_or(0);
        let hdr = radiotap::build_tx_vht(index, 1, false);
        let dot11 = frame::build_dot11(fmt, &f)?;
        let mut buf = Vec::with_capacity(hdr.len() + dot11.len());
        buf.extend_from_slice(&hdr);
        buf.extend_from_slice(&dot11);
        for _ in 0..count {
            backend.inject_raw(&buf).await?;
        }
    } else if mode == "legacy" {
        // The control arm cannot go through `frame::build`/`build_at`: those always emit the HT
        // radiotap TX header (only the EspNow format takes the legacy branch), which would make the
        // "legacy" arm secretly HT and destroy the comparison. Build the legacy RATE header
        // explicitly and hand the whole buffer to `inject_raw`.
        let hdr = radiotap::build_tx_legacy(12); // 12 × 500 kbps = 6 Mbps OFDM
        let dot11 = frame::build_dot11(fmt, &f)?;
        let mut buf = Vec::with_capacity(hdr.len() + dot11.len());
        buf.extend_from_slice(&hdr);
        buf.extend_from_slice(&dot11);
        for _ in 0..count {
            backend.inject_raw(&buf).await?;
        }
    } else {
        let index: u8 = mode.trim_start_matches("ht").parse().unwrap_or(0);
        // nss=1 / vht=false / 20 MHz: the ONLY HT this 1×1 part could ever demodulate. Ambient HT is
        // mostly 2×2 or 40/80 MHz and is physically undecodable here, which is why counting ambient
        // HT frames was never going to answer this question.
        let mcs = McsDescriptor {
            index,
            short_gi: false,
            vht: false,
            nss: 1,
            stbc: false,
            ldpc: false,
        };
        backend.set_rate(mcs)?;
        for _ in 0..count {
            backend.inject(f.clone()).await?;
        }
    }
    println!("done: {count} frames emitted");
    Ok(())
}

#[cfg(not(target_os = "linux"))]
fn main() {
    eprintln!("inject_ht requires Linux AF_PACKET monitor-mode injection (CAP_NET_RAW).");
    std::process::exit(1);
}
