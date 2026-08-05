//! **M3 TX** — beacon a counted FLRC frame every 200 ms.
//!
//! Pair with `m3_rx` on the other kit. The payload is a 4-byte big-endian sequence number followed
//! by a fixed tag, so the receiver can report not just *that* it heard something but **which**
//! frames it missed — a delivery ratio, not a liveness blink. That distinction is what makes M3 a
//! measurement instead of a demo.

#![no_std]
#![no_main]

use defmt_rtt as _;
use embassy_executor::Spawner;
use embassy_time::{Duration, Timer};
use panic_probe as _;

use lr2021_nrf54l15_rs::{flrc_link, hw};

/// Marks our frames in a capture and guards against decoding someone else's traffic as ours.
const TAG: &[u8] = b"NDN-M3";

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let p = embassy_nrf::init(Default::default());
    let (mut radio, _dio) = hw::init(p);

    radio.reset().await.unwrap();
    Timer::after(Duration::from_millis(50)).await;
    match radio.get_version().await {
        Ok(mut v) => defmt::info!("m3_tx: LR2021 fw {}.{} ok={}", v.major(), v.minor(), v.status().is_ok()),
        Err(e) => defmt::panic!("m3_tx: no radio: {}", defmt::Debug2Format(&e)),
    }

    flrc_link::configure(&mut radio).await.expect("FLRC configure");
    defmt::info!(
        "m3_tx: FLRC {=u32} Hz, 2.6 Mbit/s, syncword {=u32:#010x}, {} dBm — beaconing",
        flrc_link::FREQ_HZ,
        flrc_link::SYNCWORD,
        flrc_link::TX_POWER_DBM
    );

    let mut seq: u32 = 0;
    loop {
        let mut frame = [0u8; 4 + 6];
        frame[..4].copy_from_slice(&seq.to_be_bytes());
        frame[4..].copy_from_slice(TAG);

        if let Err(e) = radio.wr_tx_fifo_from(&frame).await {
            defmt::error!("m3_tx: fifo write: {}", defmt::Debug2Format(&e));
        } else if let Err(e) = radio.set_tx(0).await {
            defmt::error!("m3_tx: set_tx: {}", defmt::Debug2Format(&e));
        }

        if seq % 25 == 0 {
            defmt::info!("m3_tx: sent seq {}", seq);
        }
        seq = seq.wrapping_add(1);
        Timer::after(Duration::from_millis(200)).await;
    }
}
