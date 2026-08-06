//! **#108 — sweep the field Semtech's BLE "frequency drift" workaround pokes.**
//!
//! `lr20xx_workarounds_bluetooth_le_phy_coded_frequency_drift()` (Lora-net/usp, `lr20xx_workarounds.c`)
//! is a single masked register write:
//!
//! ```text
//!   addr 0x00F30C28, mask 0x1F << 5, value 30 << 5
//! ```
//!
//! — a 5-bit field at bits 9:5 driven to 30 (of a maximum 31), in the `0xF30xxx` modem block that
//! also holds the LoRa SX1276-compatibility, OOK-threshold and RTToF-deviation workarounds. Its
//! stated purpose is "to support **frequency drift**" on the LE coded PHYs, and it is applied after
//! the modulation and packet params are configured.
//!
//! That is the closest thing in Semtech's entire tree to what #108 now looks like: a receiver that
//! locks correctly off the syncword and then walks around the constellation
//! (`00 -> 55 -> ff -> aa -> 00`) a few tens of microseconds later, on the 2.4 GHz path only. The
//! field is nominally BLE, and it may well do nothing for FLRC — but it is documented, bounded, and
//! costs one sweep.
//!
//! **Self-contained**: configures once, then walks the field 0..=31 in a single flash, scoring
//! frames at each value. Pair with `PHY_CR=none PHY_PAT=00` so the score is the clean all-zeros
//! readout rather than the ramp seen through a FEC decoder.
//!
//! Reads the field *before* touching it, so we learn what FLRC's own default is — which is worth
//! knowing regardless of whether the sweep finds anything.

#![no_std]
#![no_main]

use defmt_rtt as _;
use embassy_executor::Spawner;
use embassy_time::{Duration, Timer};
use panic_probe as _;

use lr2021_nrf54l15_rs::{flrc_link, hw};

/// The register Semtech's BLE frequency-drift workaround writes.
const DRIFT_REG: u32 = 0x00F3_0C28;
const DRIFT_MASK: u32 = 0x1F << 5;

const N_PER_STEP: u32 = 24;
const STEP_TIMEOUT_MS: u64 = 2500;

fn expect(i: usize) -> u8 {
    match option_env!("PHY_PAT") {
        Some(v) if matches!(v.as_bytes(), b"00") => 0x00,
        Some(v) if matches!(v.as_bytes(), b"ff") => 0xff,
        Some(v) if matches!(v.as_bytes(), b"55") => 0x55,
        Some(v) if matches!(v.as_bytes(), b"aa") => 0xaa,
        _ => i as u8,
    }
}

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let p = embassy_nrf::init(Default::default());
    let (mut radio, _t, _u) = hw::init(p);

    radio.reset().await.unwrap();
    Timer::after(Duration::from_millis(50)).await;
    if radio.get_version().await.is_err() {
        defmt::panic!("m114: no radio");
    }
    flrc_link::configure(&mut radio).await.expect("configure");
    if option_env!("PHY_CRC_ON").is_none() {
        flrc_link::set_crc_off(&mut radio).await.expect("crc off");
    }

    // What does FLRC leave this at? Worth recording even if the sweep finds nothing.
    let before = radio.rd_reg(DRIFT_REG).await.unwrap_or(0xFFFF_FFFF);
    defmt::info!(
        "m114: {=u32:#010x} = {=u32:#010x}, drift field (9:5) = {=u32}  [Semtech's BLE value is 30]",
        DRIFT_REG,
        before,
        (before & DRIFT_MASK) >> 5
    );
    defmt::info!("columns: field | frames | lead_avg | lead_max | hits_avg/48 | perfect");

    let mut best = (0u32, 0u32);

    // **Reconfigure before every step.** The first run of this sweep died after field=1: low values
    // kill reception outright, and evidently leave the modem in a state the next step cannot recover
    // from, so every subsequent value silently reported nothing. Re-running configure() each time
    // makes each point independent, which is what a sweep has to be to mean anything.
    //
    // Range starts at 16 for the same reason — 0 and 1 are known-destructive and there is nothing to
    // learn from re-confirming that.
    for field in 16..=31u32 {
        if flrc_link::configure(&mut radio).await.is_err() {
            defmt::info!("  field={=u32}: reconfigure FAILED", field);
            continue;
        }
        if option_env!("PHY_CRC_ON").is_none() {
            let _ = flrc_link::set_crc_off(&mut radio).await;
        }
        if radio.wr_reg_mask(DRIFT_REG, DRIFT_MASK, field << 5).await.is_err() {
            defmt::info!("  field={=u32}: register write FAILED", field);
            continue;
        }
        let _ = radio.clear_rx_fifo().await;
        let _ = radio.get_and_clear_irq().await;
        if radio.set_rx_continous().await.is_err() {
            continue;
        }

        let (mut n, mut lead_sum, mut lead_max, mut hits_sum, mut perfect) = (0u32, 0u32, 0u32, 0u32, 0u32);
        let deadline = embassy_time::Instant::now() + Duration::from_millis(STEP_TIMEOUT_MS);
        while n < N_PER_STEP && embassy_time::Instant::now() < deadline {
            let irq = match radio.get_and_clear_irq().await {
                Ok(i) => i,
                Err(_) => continue,
            };
            if !irq.rx_done() {
                Timer::after(Duration::from_micros(200)).await;
                continue;
            }
            let mut b = [0u8; flrc_link::FRAME_LEN as usize];
            let ok = radio.rd_rx_fifo_to(&mut b).await.is_ok();
            let _ = radio.clear_rx_fifo().await;
            if !ok {
                continue;
            }
            let lead = b.iter().enumerate().take_while(|(i, v)| **v == expect(*i)).count() as u32;
            let hits = b.iter().enumerate().filter(|(i, v)| **v == expect(*i)).count() as u32;
            n += 1;
            lead_sum += lead;
            lead_max = lead_max.max(lead);
            hits_sum += hits;
            if hits as usize == flrc_link::FRAME_LEN as usize {
                perfect += 1;
            }
        }

        if n == 0 {
            defmt::info!("  field={=u32}: 0 frames", field);
            continue;
        }
        let h = hits_sum / n;
        defmt::info!(
            "  field={=u32} n={=u32} lead_avg={=u32} lead_max={=u32} hits_avg={=u32}/48 perfect={=u32}",
            field,
            n,
            lead_sum / n,
            lead_max,
            h,
            perfect
        );
        if h > best.1 {
            best = (field, h);
        }
    }

    defmt::info!("m114 DONE. best field={=u32} hits_avg={=u32}/48", best.0, best.1);
    loop {
        Timer::after(Duration::from_secs(5)).await;
    }
}
