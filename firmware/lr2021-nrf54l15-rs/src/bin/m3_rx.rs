//! **M3 RX** — receive the `m3_tx` beacon and report a delivery ratio, not a blink.
//!
//! Tracks the transmitter's sequence number, so the summary line carries received / expected / lost
//! and the RSSI of the last frame. A link that "works" at 40% is a very different result from one
//! that works at 99%, and only the second is a foundation for the M4/M5 timing measurements.

#![no_std]
#![no_main]

use defmt_rtt as _;
use embassy_executor::Spawner;
use embassy_time::{Duration, Timer};
use panic_probe as _;

use lr2021_nrf54l15_rs::{flrc_link, hw};

const TAG: &[u8] = b"NDN-M3";

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let p = embassy_nrf::init(Default::default());
    let (mut radio, _timing, _uart) = hw::init(p);

    radio.reset().await.unwrap();
    Timer::after(Duration::from_millis(50)).await;
    match radio.get_version().await {
        Ok(mut v) => defmt::info!("m3_rx: LR2021 fw {}.{} ok={}", v.major(), v.minor(), v.status().is_ok()),
        Err(e) => defmt::panic!("m3_rx: no radio: {}", defmt::Debug2Format(&e)),
    }

    flrc_link::configure(&mut radio).await.expect("FLRC configure");
    radio.set_rx_continous().await.expect("set_rx_continuous");
    defmt::info!(
        "m3_rx: FLRC {=u32} Hz, 2.6 Mbit/s, syncword {=u32:#010x} — listening",
        flrc_link::FREQ_HZ,
        flrc_link::SYNCWORD
    );

    let (mut got, mut bad, mut first, mut last_seq) = (0u32, 0u32, None::<u32>, 0u32);
    let mut buf = [0u8; 64];

    loop {
        let irq = match radio.get_and_clear_irq().await {
            Ok(i) => i,
            Err(e) => {
                defmt::error!("m3_rx: irq read: {}", defmt::Debug2Format(&e));
                Timer::after(Duration::from_millis(10)).await;
                continue;
            }
        };

        if irq.crc_error() {
            // Counted, not ignored: CRC errors mean the link is marginal, which is a different
            // diagnosis from silence (wrong frequency / syncword / antenna port).
            bad = bad.wrapping_add(1);
        }

        if irq.rx_done() {
            let len = radio.get_rx_pkt_len().await.unwrap_or(0) as usize;
            let n = len.min(buf.len());
            if radio.rd_rx_fifo_to(&mut buf[..n]).await.is_ok() && n >= 10 && &buf[4..10] == TAG {
                let seq = u32::from_be_bytes([buf[0], buf[1], buf[2], buf[3]]);
                got = got.wrapping_add(1);
                if first.is_none() {
                    first = Some(seq);
                    defmt::info!("m3_rx: FIRST FRAME, seq {}", seq);
                }
                last_seq = seq;
                if got % 25 == 0 {
                    let expected = seq.wrapping_sub(first.unwrap_or(seq)).wrapping_add(1);
                    let rssi = radio.get_rssi_inst().await.unwrap_or(0);
                    defmt::info!(
                        "m3_rx: got {} / expected {} (lost {}), crc_err {}, last seq {}, rssi_raw {}",
                        got,
                        expected,
                        expected.saturating_sub(got),
                        bad,
                        last_seq,
                        rssi
                    );
                }
            }
            // Continuous RX stays armed; nothing to re-issue.
        }

        Timer::after(Duration::from_millis(2)).await;
    }
}
