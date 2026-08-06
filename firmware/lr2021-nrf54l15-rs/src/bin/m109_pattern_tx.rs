//! **#108 clean-slate TX — a known constant pattern, no encoding of any kind.**
//!
//! Sends `0x00, 0x01, 0x02, … 0x2F` (48 bytes, one full [`flrc_link::FRAME_LEN`] frame). **No
//! whitening, no Bloom filter, no ASCII name, no CRC.** An incrementing ramp is chosen deliberately:
//! every byte is distinct, so a receiver can tell a *shift* from an *inversion* from a *repeat* by
//! inspection, none of which the previous derived payloads made distinguishable.
//!
//! Pair with `m109_pattern_rx`.

#![no_std]
#![no_main]

use defmt_rtt as _;
use embassy_executor::Spawner;
use embassy_time::{Duration, Timer};
use panic_probe as _;

use lr2021_nrf54l15_rs::{flrc_link, hw};

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let p = embassy_nrf::init(Default::default());
    let (mut radio, _t, _u) = hw::init(p);

    radio.reset().await.unwrap();
    Timer::after(Duration::from_millis(50)).await;
    if radio.get_version().await.is_err() {
        defmt::panic!("m109_tx: no radio");
    }
    flrc_link::configure(&mut radio).await.expect("configure");
    if option_env!("PHY_CRC_ON").is_none() {
        flrc_link::set_crc_off(&mut radio).await.expect("crc off");
    }

    let mut frame = [0u8; flrc_link::FRAME_LEN as usize];
    for (i, b) in frame.iter_mut().enumerate() {
        *b = i as u8;
    }
    // **#108 CDR test.** Build with `PHY_WHITEN=1` to whiten the ramp before transmitting.
    //
    // The raw ramp is very nearly the worst possible payload for clock recovery: `0x00` is eight
    // consecutive zero bits, `0x01`/`0x02`/`0x04` seven each, and the whole low end of the sequence
    // is transition-starved. The length sweep showed 8-byte frames arrive PERFECT while anything
    // longer breaks around byte 9-15 — consistent with the receiver coasting through the preamble's
    // timing estimate and then slipping once the data stops providing edges. A slipped CDR in GMSK
    // shows up as exactly the polarity inversion the XOR-mask readout found.
    //
    // Whitening is the standard fix for precisely this, so it doubles as the test: same frame, same
    // length, same PHY, only the transition density changes.
    if option_env!("PHY_WHITEN").is_some() {
        flrc_link::whiten(&mut frame);
        defmt::info!("m109_pattern_tx: payload WHITENED (transition-density test)");
    }
    defmt::info!(
        "m109_pattern_tx: sending 0x00..0x{=u8:02x} ({} bytes), NO whitening, NO CRC",
        (flrc_link::FRAME_LEN - 1) as u8,
        flrc_link::FRAME_LEN
    );

    loop {
        let _ = radio.clear_tx_fifo().await;
        if radio.wr_tx_fifo_from(&frame).await.is_ok() {
            let _ = radio.set_tx(0).await;
        }
        Timer::after(Duration::from_millis(20)).await;
    }
}
