//! **#108 — verify the TRANSMITTER, locally, with no radio link involved.**
//!
//! Every experiment in this chase has assumed the transmitter actually put `00,01,…,2f` on the air
//! and asked what the channel or the receiver did to it. That assumption has never been tested. It
//! is now the largest untested surface, because the failure is **deterministic** (the role swap
//! reproduces it byte for byte) and **immune to every receiver-side and PHY parameter** — bitrate
//! over an 8× range, coding rate, carrier offset over ±60 kHz, payload transition density.
//!
//! This binary needs one board and no link. It checks three things the FIFO API can answer directly:
//!
//! 1. **Does the write land?** `get_tx_fifo_lvl()` after writing `FRAME_LEN` bytes must equal
//!    `FRAME_LEN`. A short count means the SPI bulk write is truncating — which would explain a
//!    frame that is correct for its first several bytes and garbage afterwards, at *any* PHY
//!    setting, in *both* directions.
//! 2. **Does the modem consume exactly that?** After `tx_done` the level must be back to 0. A
//!    non-zero remainder means the modem transmitted fewer bytes than we queued.
//! 3. **Do the FIFO error flags fire?** `get_fifo_irq()` exposes `has_underflow()` / `has_overflow()`
//!    and **we have never once read them.** A TX underflow — the modem draining the FIFO faster than
//!    it is fed — produces exactly this failure shape: a correct prefix, then whatever the empty
//!    FIFO returns. Reading an error flag beats inferring one.
//!
//! Whatever the answer, it partitions the problem: a clean pass here moves the fault decisively to
//! the modem or the air, and a failure here means seven RF mechanisms were investigated for a bug
//! on the SPI bus.

#![no_std]
#![no_main]

use defmt_rtt as _;
use embassy_executor::Spawner;
use embassy_time::{Duration, Timer};
use panic_probe as _;

use lr2021_nrf54l15_rs::{flrc_link, hw};

const N: usize = flrc_link::FRAME_LEN as usize;

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let p = embassy_nrf::init(Default::default());
    let (mut radio, _t, _u) = hw::init(p);

    radio.reset().await.unwrap();
    Timer::after(Duration::from_millis(50)).await;
    if radio.get_version().await.is_err() {
        defmt::panic!("m112: no radio");
    }
    flrc_link::configure(&mut radio).await.expect("configure");
    flrc_link::set_crc_off(&mut radio).await.expect("crc off");

    let mut frame = [0u8; N];
    for (i, b) in frame.iter_mut().enumerate() {
        *b = i as u8;
    }

    defmt::info!("m112_tx_selftest: FRAME_LEN={=usize}, no link involved", N);

    for round in 0..5u32 {
        let _ = radio.clear_tx_fifo().await;
        let empty = radio.get_tx_fifo_lvl().await.unwrap_or(0xffff);

        if radio.wr_tx_fifo_from(&frame).await.is_err() {
            defmt::error!("round {=u32}: wr_tx_fifo_from FAILED", round);
            continue;
        }
        // (1) the write must have landed in full.
        let after_write = radio.get_tx_fifo_lvl().await.unwrap_or(0xffff);

        let _ = radio.get_and_clear_irq().await;
        if radio.set_tx(0).await.is_err() {
            defmt::error!("round {=u32}: set_tx FAILED", round);
            continue;
        }

        // Wait for the transmission to actually finish before reading anything back — polling the
        // level mid-transmit would measure the drain, not the result.
        let mut waited = 0u32;
        loop {
            match radio.get_and_clear_irq().await {
                Ok(irq) if irq.tx_done() => break,
                _ => {}
            }
            Timer::after(Duration::from_micros(200)).await;
            waited += 1;
            if waited > 5000 {
                defmt::warn!("round {=u32}: tx_done never arrived (1 s)", round);
                break;
            }
        }

        // (2) the modem must have consumed exactly what we queued.
        let after_tx = radio.get_tx_fifo_lvl().await.unwrap_or(0xffff);
        // (3) the error flags nobody has ever read.
        let (tx_irq, rx_irq) = radio
            .get_fifo_irq()
            .await
            .unwrap_or((Default::default(), Default::default()));

        defmt::info!(
            "round {=u32}: lvl empty={=u16} after_write={=u16} (want {=usize}) after_tx={=u16} (want 0)",
            round,
            empty,
            after_write,
            N,
            after_tx
        );
        defmt::info!(
            "   TX fifo flags: underflow={=bool} overflow={=bool} full={=bool} empty={=bool}",
            tx_irq.has_underflow(),
            tx_irq.has_overflow(),
            tx_irq.has_full(),
            tx_irq.has_empty()
        );
        defmt::info!(
            "   RX fifo flags: underflow={=bool} overflow={=bool}",
            rx_irq.has_underflow(),
            rx_irq.has_overflow()
        );

        if after_write as usize != N {
            defmt::error!(
                "*** TX FIFO WRITE TRUNCATED: queued {=usize}, chip holds {=u16} ***",
                N,
                after_write
            );
        }
        Timer::after(Duration::from_millis(100)).await;
    }

    defmt::info!("m112 DONE");
    loop {
        Timer::after(Duration::from_secs(5)).await;
    }
}
