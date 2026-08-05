//! **M6 — the host bridge.** Makes this node a peer of the Waveshare and Heltec LoRa nodes.
//!
//! Speaks the same 7E-A5 protocol ([`serial`]) over **UART20**, which the XIAO's onboard CMSIS-DAP
//! probe bridges to `/dev/ttyACM0` — so the existing host driver reaches it with no new transport.
//!
//! What parity buys: the rig's five sub-GHz/2.4 GHz nodes (2 LR2021 + 2 Waveshare + 1 Heltec) become
//! one addressable fleet, which is what the N≥3 MAC experiments need — the claimable-slot and
//! hidden-terminal tests (#94/#95) cannot run on a two-node link.
//!
//! Two deliberate differences from the Waveshare node, both recorded in [`serial`]:
//!
//! - `EVT_RX.ts_us` carries the **hardware capture** from M4 (DPPI-latched at the DIO edge, 62.5 ns
//!   resolution), not a software millisecond counter. Same field, far better number.
//! - LoRa-only commands (spreading factor, CAD config, SF scan) are answered with
//!   [`EVT_UNSUPPORTED`] rather than ignored. A host that assumes them gets an error instead of
//!   silence, which is the difference between a diagnosable bug and a mystery.

#![no_std]
#![no_main]

use defmt_rtt as _;
use embassy_executor::Spawner;
use embassy_nrf::buffered_uarte::BufferedUarte;
use embassy_nrf::uarte;
use embedded_io_async::{Read, Write};
use embassy_time::{Duration, Timer};
use panic_probe as _;

use lr2021_nrf54l15_rs::serial::{self, Parser};
use lr2021_nrf54l15_rs::timing::RxCapture;
use lr2021_nrf54l15_rs::{flrc_link, hw};

/// The on-device NDN data plane, **shared by path with `waveshare-lora-rs` rather than copied**.
///
/// Filter, dedup and relay decisions must be byte-identical across nodes that interoperate: two
/// implementations that agree today would drift, and the failure mode is a node silently dropping
/// traffic its neighbour forwards — indistinguishable from a link problem.
#[path = "../../../waveshare-lora-rs/src/ndn.rs"]
pub mod ndn;

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let p = embassy_nrf::init(Default::default());
    let (mut radio, timing, up) = hw::init(p);

    let mut ucfg = uarte::Config::default();
    ucfg.baudrate = uarte::Baudrate::Baud115200;
    // **Buffered**, not raw. The first version used single-byte `Uarte::read` in the same loop that
    // polls the radio over SPI, and dropped host commands: at 115200 a byte is ~87 µs, and an SPI
    // IRQ poll easily exceeds that, so bytes arriving mid-poll were simply lost (`CMD_GET_INFO`
    // vanished while later commands got through). This is the identical defect that cost the
    // Waveshare firmware ~50% of its commands (task #17) — an interrupt/DMA-backed ring is the fix
    // there and here. Any host-facing serial loop that also drives SPI needs one.
    static mut RXB: [u8; 512] = [0; 512];
    static mut TXB: [u8; 512] = [0; 512];
    let (rxb_, txb_) = unsafe { (&mut *core::ptr::addr_of_mut!(RXB), &mut *core::ptr::addr_of_mut!(TXB)) };
    let mut uart = BufferedUarte::new(up.uart, up.rx, up.tx, hw::Irqs, ucfg, rxb_, txb_);

    radio.reset().await.unwrap();
    Timer::after(Duration::from_millis(50)).await;
    let (fw_major, fw_minor) = match radio.get_version().await {
        Ok(mut v) => (v.major(), v.minor()),
        Err(e) => defmt::panic!("m6: no radio: {}", defmt::Debug2Format(&e)),
    };

    flrc_link::configure(&mut radio).await.expect("FLRC configure");
    let cap = RxCapture::new(timing);
    radio.set_rx_continous().await.expect("rx");

    defmt::info!("m6_bridge: up — 7E-A5 on UART20 @115200, FLRC {=u32} Hz", flrc_link::FREQ_HZ);

    let mut dp = ndn::DataPlane::new();
    let mut parser = Parser::new();
    let mut freq = flrc_link::FREQ_HZ;
    let mut pwr = flrc_link::TX_POWER_DBM;
    let (mut n_rx, mut n_tx) = (0u32, 0u32);

    let mut rxb = [0u8; 64];
    let mut frame = [0u8; 256];
    let mut out = [0u8; 320];

    loop {
        // ── host → node: drain whatever the UART has, without blocking the radio ───────────────
        let mut byte = [0u8; 1];
        if embassy_time::with_timeout(Duration::from_millis(1), uart.read_exact(&mut byte)).await.is_ok() {
            if let Some((typ, pl)) = parser.push(byte[0]) {
                match typ {
                    serial::CMD_TX => {
                        let n = pl.len().min(frame.len());
                        frame[..n].copy_from_slice(&pl[..n]);
                        let _ = radio.clear_tx_fifo().await;
                        let ok = radio.wr_tx_fifo_from(&frame[..n]).await.is_ok()
                            && radio.set_tx(0).await.is_ok();
                        n_tx = n_tx.wrapping_add(1);
                        let k = serial::encode(&mut out, serial::EVT_TXDONE, &[ok as u8, 0]);
                        let _ = uart.write(&out[..k]).await;
                        // Continuous RX is dropped by a transmit; re-arm or the node goes deaf
                        // after its first frame — a failure that looks like "the link died".
                        let _ = radio.set_rx_continous().await;
                    }
                    serial::CMD_SET_FREQ if pl.len() >= 4 => {
                        freq = u32::from_be_bytes([pl[0], pl[1], pl[2], pl[3]]);
                        let _ = radio.set_rf(freq).await;
                        let _ = radio.set_rx_continous().await;
                    }
                    serial::CMD_SET_PWR if !pl.is_empty() => {
                        pwr = pl[0] as i8;
                        let _ = radio.set_tx_params(pwr, lr2021::radio::RampTime::Ramp16u).await;
                    }
                    serial::CMD_GET_INFO => {
                        let f = freq.to_be_bytes();
                        let body = [0u8, fw_major, fw_minor, f[0], f[1], f[2], f[3], pwr as u8, 0];
                        let k = serial::encode(&mut out, serial::EVT_INFO, &body);
                        let _ = uart.write(&out[..k]).await;
                    }
                    serial::CMD_GET_RSSI => {
                        let r = radio.get_rssi_inst().await.unwrap_or(0) as i16;
                        let k = serial::encode(&mut out, serial::EVT_RSSI, &r.to_be_bytes());
                        let _ = uart.write(&out[..k]).await;
                    }
                    serial::CMD_SET_NAME_FILTER | serial::CMD_SET_RELAY => {
                        let mut hashes = [0u64; 8];
                        let n = (pl.len() / 8).min(hashes.len());
                        for i in 0..n {
                            let mut h = [0u8; 8];
                            h.copy_from_slice(&pl[i * 8..i * 8 + 8]);
                            hashes[i] = u64::from_be_bytes(h);
                        }
                        if typ == serial::CMD_SET_NAME_FILTER {
                            dp.set_filter(&hashes[..n]);
                        } else {
                            dp.set_relay(&hashes[..n]);
                        }
                    }
                    serial::CMD_DATAPLANE if pl.len() >= 2 => {
                        dp.set_cs_serve(pl[0] != 0);
                        dp.set_dedup(pl[1] != 0);
                    }
                    serial::CMD_GET_STATS => {
                        let mut body = [0u8; 8];
                        body[..4].copy_from_slice(&n_rx.to_be_bytes());
                        body[4..].copy_from_slice(&n_tx.to_be_bytes());
                        let k = serial::encode(&mut out, serial::EVT_STATS, &body);
                        let _ = uart.write(&out[..k]).await;
                    }
                    serial::CMD_RESET_STATS => {
                        n_rx = 0;
                        n_tx = 0;
                        dp.reset_stats();
                    }
                    other => {
                        // Answered, not ignored — see the module note.
                        let k = serial::encode(&mut out, serial::EVT_UNSUPPORTED, &[other, 1]);
                        let _ = uart.write(&out[..k]).await;
                    }
                }
            }
        }

        // ── node → host: a captured frame, carrying the HARDWARE timestamp ─────────────────────
        if let Ok(irq) = radio.get_and_clear_irq().await {
            if irq.rx_done() {
                let ts = cap.hw_stamp().ticks;
                let len = radio.get_rx_pkt_len().await.unwrap_or(0) as usize;
                let n = len.min(rxb.len());
                if radio.rd_rx_fifo_to(&mut rxb[..n]).await.is_ok() && n > 0 {
                    n_rx = n_rx.wrapping_add(1);
                    let rssi = radio.get_rssi_inst().await.unwrap_or(0) as i16;
                    let mut body = [0u8; 8 + 64];
                    body[..2].copy_from_slice(&rssi.to_be_bytes());
                    body[2..4].copy_from_slice(&0i16.to_be_bytes()); // SNR: FLRC reports none
                    body[4..8].copy_from_slice(&ts.to_be_bytes());
                    let m = n.min(64);
                    body[8..8 + m].copy_from_slice(&rxb[..m]);
                    let k = serial::encode(&mut out, serial::EVT_RX, &body[..8 + m]);
                    let _ = uart.write(&out[..k]).await;
                }
            }
        }
    }
}
