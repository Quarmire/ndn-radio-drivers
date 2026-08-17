//! **#106 TX — traffic with real name diversity, so a reject ratio means something.**
//!
//! Cycles [`NAMESPACES`] distinct namespaces at varying depth. The receiver registers only a couple
//! of them, which is what makes the measurement interpretable: a node on a broadcast medium hears
//! everything and wants a small fraction, and the *size of that fraction* is what sets how much work
//! Tier-0 can save. Sending one namespace would make the reject ratio a tautology.
//!
//! Frame layout: `[filter 16][name_len 1][name…]`. The filter is carried explicitly here because
//! this PHY has no 802.11 header to hide it in — on Wi-Fi the 126-bit Blur rides
//! `addr1‖addr2‖addr3[0..4]`, which are transmitted regardless. The name travels too, only so the
//! receiver can compute **ground truth**; in production Tier-0 runs before the name is parsed at all.

#![no_std]
#![no_main]

use defmt_rtt as _;
use embassy_executor::Spawner;
use embassy_time::{Duration, Timer};
use panic_probe as _;

use lr2021_nrf54l15_rs::tier0::PrefixFilter;
use lr2021_nrf54l15_rs::{flrc_link, hw};

/// Shared group key — receiver must use the same one.
pub const KEY: [u8; 16] = *b"ndn/tier0-group!";
/// Distinct top-level namespaces in the traffic mix.
pub const NAMESPACES: u32 = 16;

/// Build `/ns{nn}/d{dd}/leaf{ll}`-style names of varying depth into `buf`; returns the length.
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
        defmt::panic!("m106_tx: no radio");
    }
    flrc_link::configure(&mut radio).await.expect("configure");
    defmt::info!("m106_shadow_tx: {} namespaces, depth 2-6, 20 ms", NAMESPACES);

    let mut seq: u32 = 0;
    let mut nb = [0u8; 64];
    loop {
        let ns = seq % NAMESPACES;
        let n = make_name(&mut nb, ns, seq);

        let mut f = PrefixFilter::new();
        f.insert_name(&KEY, &nb[..n]);

        // Fixed-size frames: pad to FRAME_LEN. The name length travels in byte 16 (just past the
        // 16-byte filter), so the receiver knows how much of the padding to ignore.
        let mut frame = [0u8; flrc_link::FRAME_LEN as usize];
        frame[..16].copy_from_slice(&f.to_wire());
        frame[16] = n as u8;
        frame[17..17 + n].copy_from_slice(&nb[..n]);

        // pld_len is the TRANSMIT length in this role — see flrc_link::set_payload_len.
        // Whiten last, over the WHOLE frame including the sparse filter and the zero padding —
        // those are precisely the parts that starve the demodulator of transitions.
        flrc_link::whiten(&mut frame);
        // #108: settle the synthesizer before keying the PA, or the carrier drifts ~27 kHz under
        // the payload and the receiver decodes a rotating constellation instead of a name.
        flrc_link::settle_before_tx(&mut radio).await;
        let _ = radio.clear_tx_fifo().await;
        if radio.wr_tx_fifo_from(&frame).await.is_ok() {
            let _ = radio.set_tx(0).await;
        }
        if seq % 500 == 0 {
            defmt::info!("m106_shadow_tx: sent {}", seq);
        }
        seq = seq.wrapping_add(1);
        Timer::after(Duration::from_millis(20)).await;
    }
}
