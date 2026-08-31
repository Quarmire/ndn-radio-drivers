//! Flood-inject on the RTL8812AU as fast as possible for N seconds — an SDR TX-radiation check.
use bytes::Bytes;
use ndn_frame_io::{BROADCAST, DEFAULT_SRC, FrameIo, InjectFrame, TxIntent};
use ndn_radio_drivers::Rtl8812auBackend;
use std::time::{Duration, Instant};
fn main() -> Result<(), Box<dyn std::error::Error>> {
    let secs: u64 = std::env::args()
        .nth(1)
        .and_then(|s| s.parse().ok())
        .unwrap_or(20);
    let ch: u8 = std::env::args()
        .nth(2)
        .and_then(|s| s.parse().ok())
        .unwrap_or(36);
    let rt = tokio::runtime::Builder::new_current_thread()
        .enable_all()
        .build()?;
    rt.block_on(async move {
        // ★ `FrameFormat::Raw80211` is this backend's DEFAULT (`rtl8812au.rs:4477`), and it means
        // "the payload IS the frame, verbatim" — so a `vec![0x42; N]` filler was being transmitted
        // as a malformed 802.11 MANAGEMENT frame with protocol version 2 and a **unicast**
        // `addr1 = 42:42:42:42:42:42`. The `dst: BROADCAST` below was ignored entirely.
        //
        // A unicast frame expects an ACK. It never arrived, so the MAC ran the retry ladder and
        // doubled the contention window to CWmax on every retry; once latched, ~15 retries x
        // ~4.6 ms of backoff is ~70 ms per frame, the bulk-OUT FIFO backs up, and `write_bulk`
        // hits its 100 ms `TX_TIMEOUT` — MEASURED as a dead-flat **10 f/s** after the first second.
        // The a81a looked healthy on identical air only because its backend defaults to `RawNdn`
        // (`libusb_rtl88xx.rs:386`) and therefore actually broadcasts.
        let d = std::sync::Arc::new(Rtl8812auBackend::open()?.with_format(ndn_frame_io::FrameFormat::default()));
        d.bring_up_monitor(ch)?;
        // ★ `NDN_POSTURE=owned|shared|yielding` — the contention knob, so a WITNESS receiver can
        // measure what it does on air. Every prior contention A/B on this radio used the offered
        // rate, which is blind here: 711 vs 2805 offered was 1 vs 246 f/s on air.
        if let Ok(v) = std::env::var("NDN_POSTURE") {
            use ndn_radio_hal::{ContentionPosture, RadioKnobs};
            let posture = match v.trim().to_ascii_lowercase().as_str() {
                "owned" => ContentionPosture::Owned,
                "yielding" => ContentionPosture::Yielding,
                _ => ContentionPosture::Shared,
            };
            let a = RadioKnobs::set_contention(d.as_ref(), posture)?;
            eprintln!(
                "contention: {posture:?} -> cw {}..{} aifsn {} slot {} us => medium access {} us",
                a.cw_min,
                a.cw_max,
                a.aifs,
                a.slot_us,
                a.medium_access_us()
            );
        }
        println!("8812AU flood ch{ch} for {secs}s (legacy 6M)");
        // addr3: None = the legacy layout (no Tier-0 filter in addr1‖addr2, so no displaced nonce).
        let frame = InjectFrame {
            payload: Bytes::from(vec![0x42u8; 1400]),
            tx: TxIntent::ROBUST,
            // ★ `NDN_AU_UNICAST=1` addresses the frame to a real unicast MAC instead of broadcast.
            //
            // This tests whether the CCX transmit report is structurally a UNICAST mechanism: a
            // broadcast frame is never acknowledged, so the MAC has nothing to report about it,
            // which would explain why arming 1-in-1 still produced ~0 reports.
            //
            // Safe here specifically because `build_txdesc` sets RETRY_LIMIT_ENABLE with
            // DATA_RETRY_LIMIT = 0 — one attempt per frame, no retry ladder. (The retry ladder on
            // an unacknowledged unicast frame is what collapsed this radio to 10 f/s earlier, and
            // that was a MALFORMED frame with the payload as its own header; this is a properly
            // built data frame that simply has a unicast addr1.)
            dst: if std::env::var_os("NDN_AU_UNICAST").is_some() {
                [0x00, 0xe0, 0x4c, 0x11, 0x22, 0x33]
            } else {
                BROADCAST
            },
            src: DEFAULT_SRC,
            addr3: None,
            addr4: None,
            htc: None,
        };
        // ★ CCX FIFO indices. The firmware's report drain (code 0x7A38) reads MAC 0x047E as the
        // READ index and 0x047F as the WRITE index, and exits immediately when they are equal —
        // the FIFO is a 16x8 B ring at XDATA 0x8000. So:
        //   write index ADVANCES  => the MAC IS generating CCX records; the firmware drain or our
        //                            RX path is where they are being lost.
        //   write index STATIC    => the MAC is not generating them at all, and no amount of
        //                            firmware-side work will help — the arming is upstream.
        // Two byte reads, no writes.
        // ★ C2H reports arrive on the RX path. A transmit-only flood never reads the bulk-IN
        // endpoint, so the firmware's reports are generated, drained, and then dropped on the floor
        // by the host — which is exactly why arming the descriptor bit appeared to do nothing.
        let _pump = std::env::var_os("NDN_AU_TXRPT").is_some().then(|| {
            eprintln!("(RX pump started: TX reports come back over the RX path)");
            d.spawn_rx_pump(4)
        });
        // ★ Arm the MAC report engine (REG_TX_RPT_CTRL 0x04EC, sourced from the Jaguar1
        // rtl8821ae reg.h). The descriptor bit alone tags frames; this is what makes the MAC
        // actually emit records. Restored on the way out.
        if std::env::var_os("NDN_AU_TXRPT_ARM").is_some() {
            d.arm_tx_report()?;
            eprintln!(
                "TX report engine ARMED: 0x04EC={:#04x} 0x04ED={:#04x} 0x04F0={:#06x}",
                d.read8(0x04ec)?, d.read8(0x04ed)?, d.read16(0x04f0)?
            );
        }
        let ccx_before = (d.read8(0x047e)?, d.read8(0x047f)?);
        let end = Instant::now() + Duration::from_secs(secs);
        let mut n = 0u64;
        while Instant::now() < end {
            let _ = d.inject(frame.clone()).await;
            n += 1;
        }
        let ccx_after = (d.read8(0x047e)?, d.read8(0x047f)?);
        let (seen, ok, retries) = d.tx_report_counters();
        let _ = d.disarm_tx_report();
        println!("injected {n} frames");
        if seen > 0 {
            println!(
                "TX REPORTS: {seen} seen, {ok} delivered ({:.1}%), {retries} total retries \
                 ({:.2}/report)",
                100.0 * ok as f64 / seen as f64,
                retries as f64 / seen as f64
            );
        }
        println!(
            "CCX FIFO idx (0x047E read / 0x047F write): {:#04x}/{:#04x} -> {:#04x}/{:#04x}  => {}",
            ccx_before.0, ccx_before.1, ccx_after.0, ccx_after.1,
            if ccx_after.1 != ccx_before.1 {
                "WRITE INDEX MOVED: the MAC generated CCX records"
            } else {
                "write index STATIC: the MAC generated NO records"
            }
        );
        Ok::<(), Box<dyn std::error::Error>>(())
    })
}
