//! **#106 (GCS) RX — body-GCS filter in SHADOW MODE: evaluate, do not act, compare to truth.**
//!
//! The GCS twin of `m106_shadow_rx`. The filter now rides the frame body, so the receiver's only
//! pre-parse input is the `[gcs_len][gcs bytes]` prefix; it reconstructs a [`GcsFilter`] and tests it
//! against the registered prefixes. Everything else is identical, including the one thing that
//! matters: **`false_neg` must read 0**. A sequential-decode GCS packs fewer bits than the Bloom, and
//! this is the proof it buys that for free — a `false` from [`GcsFilter::may_match`] is exact, so a
//! wanted frame is never dropped.
//!
//! | GCS says | truth (name really under a registered prefix) | classification |
//! |---|---|---|
//! | reject | no  | **true reject** — the work the filter claims to do |
//! | reject | yes | **FALSE NEGATIVE** — must never happen; invalidates the filter |
//! | accept | yes | true accept |
//! | accept | no  | false positive — costs a parse, the on-air FP rate |

#![no_std]
#![no_main]

use defmt_rtt as _;
use embassy_executor::Spawner;
use embassy_time::{Duration, Timer};
use panic_probe as _;

use lr2021_nrf54l15_rs::gcs::GcsFilter;
use lr2021_nrf54l15_rs::{flrc_link, hw};

const KEY: [u8; 16] = *b"ndn/tier0-group!";

/// The prefixes this node registers — its "FIB". Two of the transmitter's sixteen namespaces, so
/// ~1/8 of traffic is genuinely wanted and the rest is exactly what the filter exists to discard.
const REGISTERED: [&[u8]; 2] = [b"/0003", b"/000a"];

/// Largest frame the transmitter sends (gcs_len + gcs + name_len + name, padded to FRAME_LEN).
const MAX_FRAME: usize = 48;

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let p = embassy_nrf::init(Default::default());
    let (mut radio, _t, _u) = hw::init(p);

    radio.reset().await.unwrap();
    Timer::after(Duration::from_millis(50)).await;
    if radio.get_version().await.is_err() {
        defmt::panic!("m106_gcs_rx: no radio");
    }
    flrc_link::configure(&mut radio).await.expect("configure");
    radio.set_rx_continous().await.expect("rx");

    defmt::info!(
        "m106_gcs_shadow_rx: SHADOW MODE — {} registered prefixes, body GCS evaluated but NOT acted on",
        REGISTERED.len()
    );

    let (mut seen, mut t_rej, mut t_acc) = (0u32, 0u32, 0u32);
    let mut crc_bad = 0u32;
    let (mut false_neg, mut false_pos, mut true_acc) = (0u32, 0u32, 0u32);
    let mut buf = [0u8; 96];

    loop {
        if let Ok(irq) = radio.get_and_clear_irq().await {
            // Reject CRC failures before touching the payload — a corrupt frame is not an irrelevant
            // frame, and counting it as one flatters the reject ratio (the lesson from the Bloom RX).
            if irq.crc_error() {
                crc_bad = crc_bad.wrapping_add(1);
                if crc_bad % 100 == 1 {
                    defmt::warn!(
                        "m106_gcs: crc_error frames {=u32} (rx_done also set: {}) — valid so far {=u32}",
                        crc_bad, irq.rx_done(), seen
                    );
                }
                let _ = radio.clear_rx_fifo().await;
                continue;
            }
            if !irq.rx_done() {
                Timer::after(Duration::from_millis(1)).await;
                continue;
            }
            let len = radio.get_rx_pkt_len().await.unwrap_or(0) as usize;
            let n = len.min(MAX_FRAME);
            let read_ok = radio.rd_rx_fifo_to(&mut buf[..n]).await.is_ok();
            // Resynchronise after every packet — variable-length frames leave residue that shifts the
            // next packet by a boundary otherwise (the RX twin of the M5 TX-FIFO bug).
            let _ = radio.clear_rx_fifo().await;
            if !read_ok || n < 3 {
                continue;
            }
            // De-whiten: the transmitter whitens the whole frame; XOR the same mask to recover it.
            flrc_link::whiten(&mut buf[..n]);

            // Parse [gcs_len][gcs][name_len][name] with bounds checks — a malformed length must not
            // panic and must not be scored (it is not a filter verdict).
            let g = buf[0] as usize;
            if 2 + g > n {
                continue;
            }
            let gcs = &buf[1..1 + g];
            let name_len = buf[1 + g] as usize;
            if 2 + g + name_len > n {
                continue;
            }
            let name = &buf[2 + g..2 + g + name_len];

            // ── GCS: the only input is the [n][gaps] wire (after the 1-byte length). No name parse. ─
            let filter = GcsFilter::from_wire(gcs);
            let gcs_accept = REGISTERED.iter().any(|p| filter.may_match(&KEY, p));

            // ── Ground truth: real prefix match on the name (the oracle, not available in prod) ─
            let truth = REGISTERED
                .iter()
                .any(|p| name.len() >= p.len() && &name[..p.len()] == *p);

            seen = seen.wrapping_add(1);
            if seen <= 5 {
                defmt::info!(
                    "m106_gcs rx#{=u32}: pkt_len={=usize} gcs_len={=usize} name_len={=usize} name={=[u8]:a} gcs={} truth={}",
                    seen, n, g, name_len, name, gcs_accept, truth
                );
            }
            match (gcs_accept, truth) {
                (false, false) => t_rej = t_rej.wrapping_add(1),
                (false, true) => {
                    false_neg = false_neg.wrapping_add(1);
                    t_rej = t_rej.wrapping_add(1);
                    defmt::error!("m106_gcs: FALSE NEGATIVE — GCS would have dropped a wanted frame");
                }
                (true, true) => {
                    t_acc = t_acc.wrapping_add(1);
                    true_acc = true_acc.wrapping_add(1);
                }
                (true, false) => {
                    t_acc = t_acc.wrapping_add(1);
                    false_pos = false_pos.wrapping_add(1);
                }
            }

            if seen % 250 == 0 {
                let reject_ppm = (t_rej as u64 * 1_000_000) / seen as u64;
                let irrelevant = seen - true_acc;
                let fp_ppm = if t_acc > 0 { (false_pos as u64 * 1_000_000) / t_acc as u64 } else { 0 };
                let fp_of_irrelevant_ppm =
                    if irrelevant > 0 { (false_pos as u64 * 1_000_000) / irrelevant as u64 } else { 0 };
                defmt::info!(
                    "m106_gcs seen={=u32} | REJECT RATIO {=u64} ppm ({=u32} frames never parsed) | wanted {=u32}",
                    seen, reject_ppm, t_rej, true_acc
                );
                defmt::info!(
                    "     | FP over irrelevant {=u64} ppm ({=u32}/{=u32}) <- compare to the bench curve",
                    fp_of_irrelevant_ppm, false_pos, irrelevant
                );
                defmt::info!(
                    "     | FP over accepted {=u64} ppm ({=u32}/{=u32} parses wasted) | FALSE NEGATIVES {=u32}",
                    fp_ppm, false_pos, t_acc, false_neg
                );
                defmt::info!("     | crc-failed frames rejected before the filter: {=u32}", crc_bad);
            }
        }
    }
}
