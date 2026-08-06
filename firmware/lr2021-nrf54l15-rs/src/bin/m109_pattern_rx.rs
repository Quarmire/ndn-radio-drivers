//! **#108 clean-slate RX — dump exactly what arrives, interpret nothing.**
//!
//! Receives the `m109_pattern_tx` ramp and prints all 48 bytes raw: **no de-whitening, no CRC, no
//! field parsing.** Expected content is literally `00 01 02 … 2f`, so the failure mode reads itself
//! off the dump:
//!
//! | what the dump shows | what it means |
//! |---|---|
//! | `00 01 02 … 2f` | the link is clean and every previous "corruption" was in our own encoding |
//! | values still ramping but offset by k | a byte/bit **shift** in the FIFO read |
//! | `ff fe fd …` | wholesale **inversion** |
//! | ramp restarts partway (`… 07 00 01 …`) | the FIFO is being **re-read from the base**, not streamed |
//! | ramp correct then diverges at a fixed index | a genuine **truncation/boundary** at that index |
//! | unrelated values | real on-air corruption |
//!
//! Also prints the chip's own `pkt_len` and RSSI so the dump is anchored to what the modem thought
//! it received.

#![no_std]
#![no_main]

use defmt_rtt as _;
use embassy_executor::Spawner;
use embassy_time::{Duration, Timer};
use panic_probe as _;

use lr2021_nrf54l15_rs::{flrc_link, hw};

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
        defmt::panic!("m109_rx: no radio");
    }
    flrc_link::configure(&mut radio).await.expect("configure");
    if option_env!("PHY_CRC_ON").is_none() {
        flrc_link::set_crc_off(&mut radio).await.expect("crc off");
    }
    radio.set_rx_continous().await.expect("rx");

    defmt::info!("m109_pattern_rx: expecting 00 01 02 .. 2f verbatim");

    let mut n = 0u32;
    loop {
        if let Ok(irq) = radio.get_and_clear_irq().await {
            if !irq.rx_done() {
                Timer::after(Duration::from_millis(1)).await;
                continue;
            }
            let mut b = [0u8; flrc_link::FRAME_LEN as usize];
            let ok = radio.rd_rx_fifo_to(&mut b).await.is_ok();
            let _ = radio.clear_rx_fifo().await;
            if !ok {
                continue;
            }
            // Undo the transmitter's whitening when testing with `PHY_WHITEN=1`: the LFSR mask is
            // XORed, so applying it a second time recovers the plain ramp and every score below
            // stays directly comparable to the un-whitened runs.
            if option_env!("PHY_WHITEN").is_some() {
                flrc_link::whiten(&mut b);
            }
            n += 1;
            if n <= 6 {
                let plen = radio.get_flrc_packet_status().await.map(|mut s| s.pkt_len()).unwrap_or(0);
                let rssi = radio
                    .get_flrc_packet_status()
                    .await
                    .map(|mut s| dbm(s.rssi_sync()))
                    .unwrap_or(0);
                defmt::info!("rx#{=u32} pkt_len={=u16} rssi={=i16}dBm", n, plen, rssi);
                // Chunked with saturating bounds: a fixed 16-byte slice panics for any frame
                // shorter than 16, which silently killed the short-length rungs of the #108 length
                // sweep — the run printed one header line and died looking like "no frames".
                let mut off = 0usize;
                while off < b.len() {
                    let end = (off + 16).min(b.len());
                    defmt::info!("   [{=usize}..{=usize}] {=[u8]:#04x}", off, end, b[off..end]);
                    off = end;
                }
                // How many leading bytes match the expected ramp?
                let expect = |i: usize| -> u8 {
                    match option_env!("PHY_PAT") {
                        Some(v) if matches!(v.as_bytes(), b"00") => 0x00,
                        Some(v) if matches!(v.as_bytes(), b"ff") => 0xff,
                        Some(v) if matches!(v.as_bytes(), b"55") => 0x55,
                        Some(v) if matches!(v.as_bytes(), b"aa") => 0xaa,
                        _ => i as u8,
                    }
                };
                let good = b.iter().enumerate().take_while(|(i, v)| **v == expect(*i)).count();
                defmt::info!("   leading bytes matching 00,01,02,...: {=usize}", good);
            }
        }
    }
}
