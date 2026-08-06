//! **#108 — stop guessing at the corruption and read it out.**
//!
//! We know the transmitted bytes exactly (`00,01,…,2f` from `m109_pattern_tx`), so the corruption
//! is not a mystery to be theorised about: it is `received XOR expected`, and we can simply print
//! it. Six mechanisms have now been proposed for this bug — DC imbalance, polarity-ambiguous
//! syncword, front-end overload, FIFO transfer, missing TCXO, carrier frequency offset — and the
//! frequency sweep (`m110`) came back **flat across ±60 kHz**, killing the sixth. Each of those was
//! plausible and consistent with the evidence, and each was wrong. So: measure the fault itself.
//!
//! ## The one question this asks
//!
//! **Is the XOR mask the SAME from frame to frame?**
//!
//! | mask across frames | what it means |
//! |---|---|
//! | **identical** | the fault is **deterministic** — a coding/whitening/scrambler mismatch between TX and RX, i.e. a *configuration* bug in our own setup, not the channel |
//! | varies, but always runs of `00` and `ff` | polarity slips at random positions — a demod/tracking fault |
//! | unstructured noise | genuine on-air corruption, and only then is RF the right place to look |
//!
//! That first outcome is the one worth chasing hardest, because the m109 dump already hinted at it:
//! the "corrupt" bytes were the ramp *bit-inverted*, and a run of `ff` in an XOR mask is exactly
//! what an unmatched whitening LFSR looks like where its output happens to be all-ones.
//!
//! The mask is compared against the first frame's, byte for byte, and the count of frames matching
//! exactly is reported — a single scalar that answers the question without eyeballing hex.

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
        defmt::panic!("m111: no radio");
    }
    flrc_link::configure(&mut radio).await.expect("configure");
    flrc_link::set_crc_off(&mut radio).await.expect("crc off");
    radio.set_rx_continous().await.expect("rx");

    defmt::info!("m111_xor_mask_rx: printing received XOR expected. Is the mask stable?");

    let mut first = [0u8; N];
    let mut have_first = false;
    let (mut n, mut same) = (0u32, 0u32);

    loop {
        let irq = match radio.get_and_clear_irq().await {
            Ok(i) => i,
            Err(_) => continue,
        };
        if !irq.rx_done() {
            Timer::after(Duration::from_micros(200)).await;
            continue;
        }
        let mut b = [0u8; N];
        let ok = radio.rd_rx_fifo_to(&mut b).await.is_ok();
        let _ = radio.clear_rx_fifo().await;
        if !ok {
            continue;
        }

        // The fault, isolated: what the channel (or our own config) did to each byte.
        let mut mask = [0u8; N];
        for i in 0..N {
            mask[i] = b[i] ^ (i as u8);
        }

        n += 1;
        if !have_first {
            first = mask;
            have_first = true;
            defmt::info!("reference mask (frame 1):");
            defmt::info!("   [00..16] {=[u8]:#04x}", mask[..16]);
            defmt::info!("   [16..32] {=[u8]:#04x}", mask[16..32]);
            defmt::info!("   [32..48] {=[u8]:#04x}", mask[32..48]);
        } else {
            let agree = mask.iter().zip(first.iter()).filter(|(a, c)| a == c).count();
            if agree == N {
                same += 1;
            }
            if n <= 5 {
                defmt::info!(
                    "frame {=u32}: mask agrees with reference in {=usize}/{=usize} bytes",
                    n,
                    agree,
                    N
                );
                defmt::info!("   [00..16] {=[u8]:#04x}", mask[..16]);
                defmt::info!("   [16..32] {=[u8]:#04x}", mask[16..32]);
                defmt::info!("   [32..48] {=[u8]:#04x}", mask[32..48]);
            }
        }

        if n == 60 {
            defmt::info!(
                "m111 RESULT: {=u32}/{=u32} frames had a mask IDENTICAL to the first",
                same,
                n - 1
            );
            defmt::info!("  identical => deterministic config fault (whitening/coding), not the channel");
            defmt::info!("  varying   => a real demod/RF fault; only then is RF the right place to look");
            loop {
                Timer::after(Duration::from_secs(5)).await;
            }
        }
    }
}
