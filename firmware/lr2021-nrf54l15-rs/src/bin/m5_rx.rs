//! **M4 — the number this board was acquired for: the RX-timestamp floor.**
//!
//! ## What is actually being measured, and why this design
//!
//! Every received frame is stamped **twice, from one timer**:
//!
//! - `CC[0]` is written by **DPPI**, at the instant DIO8 rises. No CPU, no interrupt, no scheduler.
//! - `CC[1]` is written by the **CPU**, at the moment the async task wakes and gets to run.
//!
//! `CC[1] − CC[0]` is therefore the *entire* software path: GPIOTE interrupt latency, executor
//! wake-up, and whatever else the CPU was doing. Its **peak-to-peak spread is the jitter the
//! hardware capture removes** — and it is exactly the error a MAC guard band would otherwise have to
//! cover.
//!
//! Both stamps come from the **same** timer on purpose. Comparing against a second clock (say
//! `embassy_time`, on GRTC) would fold two free-running oscillators' relative drift into the number,
//! and at microsecond magnitudes that term is not negligible. One timer, two capture registers, no
//! drift — the difference is pure software latency and nothing else.
//!
//! ## What this does *not* yet establish
//!
//! The inter-arrival spread of the hardware stamps is also reported, but it is **an upper bound on
//! the capture jitter, not the capture jitter itself**: it contains the transmitter's own software
//! timer jitter, which dominates. Isolating the receiver's true floor needs a *hardware-scheduled*
//! transmitter — which is M5. Reported here so the bound is on record, and labelled so it is not
//! mistaken for the floor.
//!
//! Also unaddressed by design: the DIO edge marks packet-done *inside the radio*, not the first
//! on-air symbol. A constant offset between those cancels in a two-way exchange; only a *variable*
//! one would hurt, and that variation is part of what the inter-arrival figure bounds.

#![no_std]
#![no_main]

use defmt_rtt as _;
use embassy_executor::Spawner;
use embassy_time::{Duration, Timer};
use panic_probe as _;

use lr2021_nrf54l15_rs::timing::{RxCapture, Spread};
use lr2021_nrf54l15_rs::{flrc_link, hw};

const TAG: &[u8] = b"NDN-M4";
/// Report every this many stamped frames.
const REPORT_EVERY: u32 = 100;

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let p = embassy_nrf::init(Default::default());
    let (mut radio, timing, _uart) = hw::init(p);

    radio.reset().await.unwrap();
    Timer::after(Duration::from_millis(50)).await;
    match radio.get_version().await {
        Ok(mut v) => defmt::info!("m5_rx: LR2021 fw {}.{} ok={}", v.major(), v.minor(), v.status().is_ok()),
        Err(e) => defmt::panic!("m5_rx: no radio: {}", defmt::Debug2Format(&e)),
    }

    flrc_link::configure(&mut radio).await.expect("FLRC configure");

    // Build the capture path AFTER the radio is configured: DIO8 is shared, and taking it as a
    // GPIOTE channel while the driver is still toggling it during bring-up invites a spurious edge.
    let mut cap = RxCapture::new(timing);

    radio.set_rx_continous().await.expect("set_rx_continuous");
    defmt::info!(
        "m5_rx: FLRC {=u32} Hz — DPPI capture armed (TIMER20 @1MHz, GPIOTE20 rising on DIO8)",
        flrc_link::FREQ_HZ
    );

    let (mut got, mut bad, mut gaps) = (0u32, 0u32, 0u32);
    let mut latency = Spread::default();   // CC[1] - CC[0]: the software path we are replacing
    // Inter-arrival is accumulated ONLY across consecutive sequence numbers. A missed frame would
    // otherwise contribute a ~2x period sample and swamp the spread with loss instead of jitter —
    // which is exactly what happened in the first run (p2p 1.29M ticks, all of it dropped frames).
    let mut interarrival = Spread::default();
    let mut prev: Option<(u32, lr2021_nrf54l15_rs::timing::HwStamp)> = None;
    let mut buf = [0u8; 64];

    loop {
        // The hardware stamp is already latched by the time this returns. Everything after this
        // point — including the SPI transactions below — happens too late to corrupt it.
        cap.wait_edge().await;
        let sw = cap.sw_stamp();
        let hw = cap.hw_stamp();

        let irq = match radio.get_and_clear_irq().await {
            Ok(i) => i,
            Err(e) => {
                defmt::error!("m5_rx: irq read: {}", defmt::Debug2Format(&e));
                continue;
            }
        };
        if irq.crc_error() {
            bad = bad.wrapping_add(1);
        }
        if !irq.rx_done() {
            continue; // an edge that was not a completed frame: not a timing sample
        }

        let len = radio.get_rx_pkt_len().await.unwrap_or(0) as usize;
        let n = len.min(buf.len());
        if radio.rd_rx_fifo_to(&mut buf[..n]).await.is_err() || n < 10 || &buf[4..10] != TAG {
            continue;
        }

        let seq = u32::from_be_bytes([buf[0], buf[1], buf[2], buf[3]]);
        got = got.wrapping_add(1);
        latency.push(sw.since(hw));
        match prev {
            Some((pseq, phw)) if seq == pseq.wrapping_add(1) => interarrival.push(hw.since(phw)),
            Some(_) => gaps = gaps.wrapping_add(1),
            None => {}
        }
        prev = Some((seq, hw));

        if got % REPORT_EVERY == 0 {
            defmt::info!(
                "m5_rx n={} | SW-PATH LATENCY (what HW capture removes) min={=u32} mean={=u32} max={=u32} p2p={=u32} ticks @{=u32} ns",
                latency.n,
                latency.min,
                latency.mean(),
                latency.max,
                latency.peak_to_peak(),
                lr2021_nrf54l15_rs::timing::TICK_NS
            );
            defmt::info!(
                "        | TX-INSTANT SPREAD (consecutive only) min={=u32} mean={=u32} max={=u32} p2p={=u32} ticks | crc_err {}",
                interarrival.min,
                interarrival.mean(),
                interarrival.max,
                interarrival.peak_to_peak(),
                bad
            );
            defmt::info!("        | consecutive pairs n={} , sequence gaps {}", interarrival.n, gaps);
        }
    }
}
