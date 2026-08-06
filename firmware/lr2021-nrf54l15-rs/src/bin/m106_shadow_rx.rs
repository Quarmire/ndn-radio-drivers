//! **#106 RX — Tier-0 in SHADOW MODE: evaluate, do not act, and compare against ground truth.**
//!
//! Tier-0's justification has two halves, and only one of them can be shown by a passive counter:
//! it rejects most irrelevant frames, and **it never drops a wanted one**. No deployment that acts
//! on the filter can measure the second — you cannot count what you silently discarded. Shadow mode
//! is the fix: run the filter on every frame, ignore its verdict, let everything through, then check
//! the verdict against the truth.
//!
//! Per frame:
//!
//! | Tier-0 says | truth (name really under a registered prefix) | classification |
//! |---|---|---|
//! | reject | no  | **true reject** — the work Tier-0 claims to do |
//! | reject | yes | **FALSE NEGATIVE** — must never happen; invalidates the filter |
//! | accept | yes | true accept |
//! | accept | no  | false positive — costs a parse, the on-air FP rate |
//!
//! Truth is computed by actually prefix-matching the name that travels in the frame. In production
//! the name is not available before the filter runs — it is carried here *only* so the experiment
//! has an independent oracle.
//!
//! **`false_neg` must read 0.** It is reported on its own line, not folded into a percentage, so it
//! cannot be averaged into invisibility.

#![no_std]
#![no_main]

use defmt_rtt as _;
use embassy_executor::Spawner;
use embassy_time::{Duration, Timer};
use panic_probe as _;

use lr2021_nrf54l15_rs::tier0::PrefixFilter;
use lr2021_nrf54l15_rs::{flrc_link, hw};

const KEY: u64 = 0x8624_4E44_5F4B_4559;

/// The prefixes this node registers — its "FIB". Two of the transmitter's sixteen namespaces, so
/// ~1/8 of traffic is genuinely wanted and the rest is exactly what Tier-0 exists to discard.
const REGISTERED: [&[u8]; 2] = [b"/0003", b"/000a"];

/// Largest frame the transmitter sends: 12 filter + 1 length + up to 30 name bytes.
const MAX_FRAME: usize = 48;

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let p = embassy_nrf::init(Default::default());
    let (mut radio, _t, _u) = hw::init(p);

    radio.reset().await.unwrap();
    Timer::after(Duration::from_millis(50)).await;
    if radio.get_version().await.is_err() {
        defmt::panic!("m106_rx: no radio");
    }
    flrc_link::configure(&mut radio).await.expect("configure");
    radio.set_rx_continous().await.expect("rx");

    // Masks are precomputed once — the receiver-side cost per frame is then two u64 AND-compares
    // per registered prefix, which is the whole point of doing this before the parse.
    let masks: [PrefixFilter; REGISTERED.len()] =
        core::array::from_fn(|i| PrefixFilter::mask_for(KEY, REGISTERED[i]));

    defmt::info!(
        "m106_shadow_rx: SHADOW MODE — {} registered prefixes, filter evaluated but NOT acted on",
        REGISTERED.len()
    );

    let (mut seen, mut t_rej, mut t_acc) = (0u32, 0u32, 0u32);
    let mut crc_bad = 0u32;
    let (mut false_neg, mut false_pos, mut true_acc) = (0u32, 0u32, 0u32);
    let mut buf = [0u8; 96];

    loop {
        if let Ok(irq) = radio.get_and_clear_irq().await {
            // **Reject CRC failures before touching the payload.** Omitting this fed corrupted
            // frames straight into the filter: the garbage decoded as bit-inverted ASCII (0xd0 = ~'/',
            // 0xcd = ~'2'), name_len came out plausible, and the experiment reported a confident
            // reject ratio over frames that were never valid. A name filter must never evaluate a
            // frame that failed integrity — a corrupt frame is not an irrelevant frame, and counting
            // it as one flatters the reject ratio.
            if irq.crc_error() {
                crc_bad = crc_bad.wrapping_add(1);
                if crc_bad % 100 == 1 {
                    defmt::warn!(
                        "m106: crc_error frames {=u32} (rx_done also set: {}) — valid so far {=u32}",
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
            // Clamp to the largest frame this experiment sends. `get_rx_pkt_len` was observed
            // returning oversized values once the FIFO held residue, and an oversized read pulls in
            // the *next* packet's bytes.
            let len = radio.get_rx_pkt_len().await.unwrap_or(0) as usize;
            let n = len.min(MAX_FRAME);
            let read_ok = radio.rd_rx_fifo_to(&mut buf[..n]).await.is_ok();
            // **Resynchronise after every packet.** `rd_rx_fifo_to` leaves anything we did not read,
            // so with VARIABLE-LENGTH frames the residue shifts the next packet and every field
            // lands one boundary off — names decode as garbage and nothing ever matches. The earlier
            // fixed-length binaries survived this only by accident of alignment. Exactly the RX twin
            // of the M5 TX-FIFO bug: it presents as a matching failure while being a buffer one.
            let _ = radio.clear_rx_fifo().await;
            if !read_ok || n < 14 {
                continue;
            }
            let name_len = buf[12] as usize;
            if 13 + name_len > n {
                continue;
            }
            let name = &buf[13..13 + name_len];

            // ── Tier-0: the only input is the 12 bytes. No parse. ──────────────────────────────
            let mut frame_filter = PrefixFilter::new();
            frame_filter.0.copy_from_slice(&buf[..12]);
            let t0_accept = masks.iter().any(|m| frame_filter.may_match(m));

            // ── Ground truth: real prefix match on the name (the oracle, not available in prod) ─
            let truth = REGISTERED
                .iter()
                .any(|p| name.len() >= p.len() && &name[..p.len()] == *p);

            seen = seen.wrapping_add(1);
            // Diagnostic: the first few names as received, so a TX/RX disagreement about the frame
            // layout shows up as data rather than as a mysterious zero.
            if seen <= 5 {
                defmt::info!(
                    "m106 rx#{=u32}: pkt_len={=usize} name_len={=usize} name={=[u8]:a} t0={} truth={}",
                    seen, n, name_len, name, t0_accept, truth
                );
            }
            match (t0_accept, truth) {
                (false, false) => t_rej = t_rej.wrapping_add(1),
                (false, true) => {
                    false_neg = false_neg.wrapping_add(1);
                    t_rej = t_rej.wrapping_add(1);
                    defmt::error!("m106: FALSE NEGATIVE — Tier-0 would have dropped a wanted frame");
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
                let fp_ppm = if t_acc > 0 {
                    (false_pos as u64 * 1_000_000) / t_acc as u64
                } else {
                    0
                };
                defmt::info!(
                    "m106 seen={=u32} | REJECT RATIO {=u64} ppm ({=u32} frames never parsed) | wanted {=u32}",
                    seen,
                    reject_ppm,
                    t_rej,
                    true_acc
                );
                defmt::info!(
                    "     | on-air FP {=u64} ppm ({=u32}/{=u32} accepted were irrelevant) | FALSE NEGATIVES {=u32}",
                    fp_ppm,
                    false_pos,
                    t_acc,
                    false_neg
                );
                defmt::info!("     | crc-failed frames rejected before the filter: {=u32}", crc_bad);
            }
        }
    }
}
