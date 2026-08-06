//! **#108 — ask the chip what went wrong instead of inferring it.**
//!
//! `GetErrors` (datasheet §6.7.4) reports every calibration and PLL failure since startup, and we
//! have never once read it. Its bits map directly onto the things this whole chase has been
//! *guessing* at:
//!
//! | bit | flag | why it matters here |
//! |---|---|---|
//! | 2 | `PLL_LOCK_ERR` | "PLL did not lock ... too high or too low a frequency, or PLL not calibrated" |
//! | 5 | `PLL_CALIB_ERR` | PLL calibration unavailable |
//! | 6 | `AAF_CALIB_ERR` | anti-aliasing filter calibration unavailable |
//! | 7 | `IMG_CALIB_ERR` | **image rejection (IQ comp)** calibration unavailable |
//! | 9 | `RXFREQ_NO_FE_CAL_ERR` | **"Front End calibration was not available for Rx operation with specified RF frequency"** |
//! | 11 | `PA_OFFSET_CALIB_ERR` | PA offset calibration unavailable |
//!
//! Bit 9 is the sharpest: it is the chip telling us, in one bit, that the RX front end has no
//! calibration for the frequency we asked it to receive on. §6.4 says the device boots calibrated
//! at 915 MHz and must be re-calibrated for any change over 10 MHz — so if our HF calibration is
//! not taking effect, this is where it shows up as a fact rather than a theory.
//!
//! Run on both paths and compare: LF is known-good (48/48), so its error word is the control. Any
//! bit set on HF and clear on LF is a lead; a clean HF word rules the whole calibration story out.
//!
//! Errors are read **after** `configure()` so they reflect our real setup, and again after a TX and
//! an RX, since some flags (PA OCP/OVP, PLL lock) can only assert once the PA or receiver runs.

#![no_std]
#![no_main]

use defmt_rtt as _;
use embassy_executor::Spawner;
use embassy_time::{Duration, Timer};
use panic_probe as _;

use lr2021_nrf54l15_rs::{flrc_link, hw};

fn report(tag: &str, e: &lr2021::system::ErrorsRsp) {
    defmt::info!("--- GetErrors [{=str}] ---", tag);
    if e.none() {
        defmt::info!("    (no errors pending)");
        return;
    }
    // The crate exposes two flags the datasheet's Table 6-43 does not list (ppf_calib, src_calib),
    // so print them too rather than assuming the doc is the complete set.
    let f: [(&str, bool); 14] = [
        ("HF_XOSC_START", e.hf_xosc_start()),
        ("LF_XOSC_START", e.lf_xosc_start()),
        ("PLL_LOCK", e.pll_lock()),
        ("LF_RC_CALIB", e.lf_rc_calib()),
        ("HF_RC_CALIB", e.hf_rc_calib()),
        ("PLL_CALIB", e.pll_calib()),
        ("AAF_CALIB", e.aaf_calib()),
        ("IMG_CALIB", e.img_calib()),
        ("CHIP_BUSY", e.chip_busy()),
        ("RXFREQ_NO_FE_CAL", e.rxfreq_no_fe_cal()),
        ("MEAS_UNIT_ADC_CALIB", e.meas_unit_adc_calib()),
        ("PA_OFFSET_CALIB", e.pa_offset_calib()),
        ("PPF_CALIB", e.ppf_calib()),
        ("SRC_CALIB", e.src_calib()),
    ];
    for (n, set) in f {
        if set {
            defmt::error!("    {=str}", n);
        }
    }
}

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let p = embassy_nrf::init(Default::default());
    let (mut radio, _t, _u) = hw::init(p);

    radio.reset().await.unwrap();
    Timer::after(Duration::from_millis(50)).await;
    if radio.get_version().await.is_err() {
        defmt::panic!("m113: no radio");
    }

    defmt::info!(
        "m113_errors: freq={=u32} Hz, path={=str}",
        flrc_link::FREQ_HZ,
        if option_env!("PHY_HF").is_some() { "HF" } else { "LF" }
    );

    // Clear first, so what we read afterwards is attributable to OUR configuration rather than to
    // whatever the boot-time self-calibration left behind.
    // The crate has no `clear_errors()` wrapper, so issue ClearErrors (0x01 0x11, Table 6-41)
    // directly. §6.7.3: it clears all error flags at once and does NOT clear the Error IRQ.
    let _ = radio.cmd_wr(&[0x01, 0x11]).await;

    // The standalone calibration block that lived here is gone: `configure()` now does this
    // itself, so testing it here would confound the thing being measured.
    flrc_link::configure(&mut radio).await.expect("configure");
    match radio.get_errors().await {
        Ok(e) => report("after configure", &e),
        Err(_) => defmt::error!("get_errors failed"),
    }

    // A TX: PA OCP/OVP and PLL lock can only assert once the PA actually runs.
    let frame = [0u8; flrc_link::FRAME_LEN as usize];
    let _ = radio.clear_tx_fifo().await;
    let _ = radio.wr_tx_fifo_from(&frame).await;
    let _ = radio.set_tx(0).await;
    Timer::after(Duration::from_millis(50)).await;
    match radio.get_errors().await {
        Ok(e) => report("after TX", &e),
        Err(_) => defmt::error!("get_errors failed"),
    }

    // An RX: bit 9 (RXFREQ_NO_FE_CAL_ERR) is specifically about *Rx operation* at this frequency.
    let _ = radio.set_rx_continous().await;
    Timer::after(Duration::from_millis(200)).await;
    match radio.get_errors().await {
        Ok(e) => report("after RX entry", &e),
        Err(_) => defmt::error!("get_errors failed"),
    }

    defmt::info!("m113 DONE");
    loop {
        Timer::after(Duration::from_secs(5)).await;
    }
}
