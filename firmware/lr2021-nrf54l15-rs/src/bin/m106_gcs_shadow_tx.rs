//! **#106 (GCS) TX — same name-diverse traffic, but the filter is a body-carried GCS.**
//!
//! The Bloom variant (`m106_shadow_tx`) packs the prefix-set into fixed address-style bytes, as Wi-Fi
//! does. FLRC has no address fields, so here the filter rides the frame **body** as a Golomb-Coded
//! Set — the cascade's `FLAG_BODY_PREFIX` tier, and the same filter the LoRa face carries. The point
//! of the pair is that the receiver's `false_neg` counter must still read 0: a smaller, sequential
//! encoding must not cost the zero-false-negative guarantee.
//!
//! Frame layout: `[gcs_len 1][gcs bytes][name_len 1][name…]`, whitened, zero-padded to `FRAME_LEN`.
//! The name travels only so the receiver can compute ground truth (the oracle); in production the
//! GCS runs before the name is parsed.

#![no_std]
#![no_main]

use defmt_rtt as _;
use embassy_executor::Spawner;
use embassy_time::{Duration, Timer};
use panic_probe as _;

use lr2021_nrf54l15_rs::gcs::{GcsFilter, GCS_MAX_BYTES};
use lr2021_nrf54l15_rs::{flrc_link, hw};

/// Shared group key — receiver must use the same one.
pub const KEY: [u8; 16] = *b"ndn/tier0-group!";
/// Distinct top-level namespaces in the traffic mix.
pub const NAMESPACES: u32 = 16;

/// Build `/ns{nn}/d{dd}/leaf{ll}`-style names of varying depth into `buf`; returns the length.
/// Byte-identical to `m106_shadow_tx::make_name`, so the two experiments send the same traffic.
pub fn make_name(buf: &mut [u8], ns: u32, seq: u32) -> usize {
    let depth = 2 + (seq % 5) as usize; // 2..6 components — a realistic spread
    let mut n = 0;
    for c in 0..depth {
        buf[n] = b'/';
        n += 1;
        let v = match c {
            0 => ns,
            _ => seq.wrapping_add(c as u32 * 7),
        };
        for shift in [12, 8, 4, 0] {
            let d = ((v >> shift) & 0xf) as u8;
            buf[n] = if d < 10 { b'0' + d } else { b'a' + d - 10 };
            n += 1;
        }
    }
    n
}

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let p = embassy_nrf::init(Default::default());
    let (mut radio, _t, _u) = hw::init(p);

    radio.reset().await.unwrap();
    Timer::after(Duration::from_millis(50)).await;
    if radio.get_version().await.is_err() {
        defmt::panic!("m106_gcs_tx: no radio");
    }
    flrc_link::configure(&mut radio).await.expect("configure");
    defmt::info!(
        "m106_gcs_shadow_tx: {} namespaces, depth 2-6, body GCS (FLAG_BODY_PREFIX), 20 ms",
        NAMESPACES
    );

    let mut seq: u32 = 0;
    let mut nb = [0u8; 64];
    loop {
        let ns = seq % NAMESPACES;
        let n = make_name(&mut nb, ns, seq);

        // Compile the body-prefix GCS for this name, then lay the frame out around it.
        let f = GcsFilter::from_name(&KEY, &nb[..n]);
        let mut gcswire = [0u8; GCS_MAX_BYTES + 1];
        let g = f.to_wire(&mut gcswire);

        let mut frame = [0u8; flrc_link::FRAME_LEN as usize];
        // [gcs_len][gcs][name_len][name] — skip the send if it would not fit (keeps names bounded).
        if 2 + g + n <= frame.len() {
            frame[0] = g as u8;
            frame[1..1 + g].copy_from_slice(&gcswire[..g]);
            frame[1 + g] = n as u8;
            frame[2 + g..2 + g + n].copy_from_slice(&nb[..n]);

            // Whiten last, over the WHOLE frame including the zero padding — the parts that starve the
            // demodulator of transitions.
            flrc_link::whiten(&mut frame);
            // #108: settle the synthesizer before keying the PA, or the carrier drifts under the payload.
            flrc_link::settle_before_tx(&mut radio).await;
            let _ = radio.clear_tx_fifo().await;
            if radio.wr_tx_fifo_from(&frame).await.is_ok() {
                let _ = radio.set_tx(0).await;
            }
        }
        if seq % 500 == 0 {
            defmt::info!("m106_gcs_shadow_tx: sent {} (gcs {} B)", seq, g);
        }
        seq = seq.wrapping_add(1);
        Timer::after(Duration::from_millis(20)).await;
    }
}
