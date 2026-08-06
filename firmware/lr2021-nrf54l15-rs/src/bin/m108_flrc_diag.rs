//! **#108 — FLRC receive diagnostics, using the chip's OWN counters.**
//!
//! Written after realising the earlier binaries drove this radio by pattern-matching a driver API
//! instead of reading the command spec. Three mistakes, all correctable from `spec/commands.yaml`:
//!
//! 1. Length came from the **generic** `GetRxPktLength`, not FLRC's `GetFlrcPacketStatus.pkt_len`.
//!    The generic value was garbage, which is what made frames appear to be 96 bytes.
//! 2. `GetFlrcRxStats` exists and reports **`pkt_rx` / `crc_error` / `len_error` separately** — the
//!    exact instrument for "why do long frames fail", and it was never used. `len_error` rising
//!    means the length parameterisation is wrong; `crc_error` rising means the *link* is bad. Those
//!    are opposite diagnoses and guessing between them wasted a run.
//! 3. **RSSI is `−value/2` dBm**, per the spec's `rssi_avg`/`rssi_sync` fields. Every "rssi_raw 190"
//!    reported so far is really **−95 dBm** — which is near FLRC sensitivity at 2.6 Mbit/s, not the
//!    "strong bench link" those write-ups claimed.
//!
//! `rssi_sync` is latched at syncword detection and `rssi_avg` over the packet, so a frame that
//! syncs and then fails CRC still yields a usable signal level — which is what tells us whether the
//! failures are marginal SNR or a configuration error.

#![no_std]
#![no_main]

use defmt_rtt as _;
use embassy_executor::Spawner;
use embassy_time::{Duration, Timer};
use panic_probe as _;

use lr2021_nrf54l15_rs::{flrc_link, hw};

/// Spec: actual signal power is `−value/2` dBm.
fn dbm(raw: u16) -> i16 {
    -((raw as i16) / 2)
}

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let p = embassy_nrf::init(Default::default());
    let (mut radio, _t, _u) = hw::init(p);

    radio.reset().await.unwrap();
    Timer::after(Duration::from_millis(50)).await;
    if radio.get_version().await.is_err() {
        defmt::panic!("m108: no radio");
    }
    flrc_link::configure(&mut radio).await.expect("configure");
    radio.set_rx_continous().await.expect("rx");

    defmt::info!(
        "m108_flrc_diag: FLRC {=u32} Hz, TX side sends 23-43 B frames. Reading the chip's own counters.",
        flrc_link::FREQ_HZ
    );

    // Dump EVERY error flag once at start-up. The earlier probe read GetErrors but only looked at
    // chip_busy and pll_lock — and `lf_xosc_start` is precisely the flag that says "there is a TCXO
    // here and you never enabled it". Reading a status word and inspecting two of its bits is how a
    // diagnostic misses the answer it already fetched.
    if let Ok(e) = radio.get_errors().await {
        defmt::info!(
            "  ERRORS  hf_xosc={} lf_xosc={} pll_lock={} lf_rc_cal={} hf_rc_cal={} pll_cal={} aaf_cal={} img_cal={} chip_busy={} rxfreq_no_fe_cal={}",
            e.hf_xosc_start(), e.lf_xosc_start(), e.pll_lock(), e.lf_rc_calib(), e.hf_rc_calib(),
            e.pll_calib(), e.aaf_calib(), e.img_calib(), e.chip_busy(), e.rxfreq_no_fe_cal()
        );
    }

    loop {
        Timer::after(Duration::from_secs(3)).await;

        match radio.get_flrc_rx_stats().await {
            Ok(mut s) => defmt::info!(
                "  RX STATS  pkt_rx={=u16}  crc_error={=u16}  len_error={=u16}",
                s.pkt_rx(),
                s.crc_error(),
                s.len_error()
            ),
            Err(e) => defmt::error!("  rx_stats: {}", defmt::Debug2Format(&e)),
        }

        match radio.get_flrc_packet_status().await {
            Ok(mut st) => defmt::info!(
                "  LAST PKT  flrc_pkt_len={=u16}  rssi_avg={=i16} dBm  rssi_sync={=i16} dBm  sw_num={=u8}",
                st.pkt_len(),
                dbm(st.rssi_avg()),
                dbm(st.rssi_sync()),
                st.sw_num()
            ),
            Err(e) => defmt::error!("  pkt_status: {}", defmt::Debug2Format(&e)),
        }

        // With the PHY CRC off, dump the bytes and compare against what m106_shadow_tx builds:
        //   [0..12] Tier-0 filter (byte 0 low bits forced to 0b11 = locally-administered group)
        //   [12]    name length
        //   [13..]  "/00nn/...." ASCII, then zero padding to FRAME_LEN
        // Intact ASCII here means the modulation path is sound and only the CRC block was at fault.
        let mut b = [0u8; 24];
        if radio.rd_rx_fifo_to(&mut b).await.is_ok() {
            // De-whiten before inspecting. Whitening is self-inverse, but it is applied over the
            // whole frame, so a partial read de-whitens correctly only from offset 0 — which this is.
            flrc_link::whiten(&mut b);
            defmt::info!("  PAYLOAD  filt0={=u8:#04x} name_len={=u8} name={=[u8]:a}", b[0], b[12], &b[13..24]);
        }
        let _ = radio.clear_rx_fifo().await;
    }
}
