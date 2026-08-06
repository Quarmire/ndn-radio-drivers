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

        // The generic command the earlier binaries used, for comparison — expected to disagree.
        let generic = radio.get_rx_pkt_len().await.unwrap_or(0xffff);
        defmt::info!("  generic get_rx_pkt_len={=u16}  (what m3/m4/m5/m106 used)", generic);
    }
}
