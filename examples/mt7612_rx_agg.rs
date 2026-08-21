//! **Does MT7612U USB RX pack several frames into one bulk-IN transfer?**
//!
//! The question left open by #80. `Mt7612uBackend::decode_rx` treats a whole transfer as ONE RX
//! unit — it slices `[MT76_RXD_LEN .. len-4]` and ignores the MT_RX_INFO length in the first 4
//! bytes. The shared pump's `Pumpable::parse_transfer` returns a `Vec` precisely because chips
//! aggregate, and our own RTL8821CU driver demonstrably walks multiple units per transfer. If mt76
//! does the same, `decode_rx` is dropping every unit after the first *and* mis-parsing the
//! concatenation as one oversized frame — RX loss hiding behind a link that looks like it works.
//!
//! This does not guess. It reads raw bulk-IN transfers and walks each one as a chain of RX units,
//! using the 16-bit little-endian length mt76 puts at the head of each (`mt76u_get_rx_entry_len`
//! reads exactly that). It reports the distribution of units-per-transfer and whether the declared
//! lengths tile the buffer — which is the check that the length field means what we think it does.
//!
//! Read the output like this:
//!   * `units/transfer` always 1  ⇒ no aggregation; `decode_rx` is correct as written.
//!   * `units/transfer` > 1       ⇒ `decode_rx` loses frames; parse_transfer must walk the chain.
//!   * `TILING MISMATCH` frequent ⇒ the length field is NOT what this assumes; do not act on the
//!     unit counts above it — the instrument is wrong, which is the failure mode to suspect first.
//!
//! ## RESOLVED 2026-08-18 — bring-up works on the C4 SuperSpeed bus; aggregation answered
//!
//! The earlier "BLOCKED as of 2026-08-10" note (a deterministic `fw chunk 1` upload timeout on
//! minidronesys-05, ODROID-C4, SuperSpeed) is **stale**. Measured 2026-08-18 on that exact host:
//! `Mt7612uBackend::bring_up` completes cleanly — 5929 register writes + 472 MCU commands, 0
//! errors; the MCU comes up running and RX frames are captured. The firmware upload is not blocked.
//!
//! The inter-chunk handshake reads that the old note blamed did **not** fail: `busy_errs=0` and
//! `drain_errs=0` across the whole upload. One latent fragility remains worth noting: the
//! `MT_FCE_PSE_CTRL_GO` (0x09a8) "drain" bit never actually clears, so the upload relies on a fixed
//! ~200ms wait rather than a real handshake edge. It works here, but it is a timing assumption, not
//! a confirmation — if a faster or slower host ever regresses the upload, this is the first suspect.
//!
//! And the aggregation question this example asks is answered: **NO aggregation** — exactly 1 RX
//! unit per bulk-IN transfer, so `decode_rx`'s single-unit treatment is correct as written.
//!
//! Run (needs the kernel driver off the device, and CAP_NET_RAW/root):
//! ```sh
//! echo -n 2-1.3.4:1.0 | sudo tee /sys/bus/usb/drivers/mt76x2u/unbind
//! sudo ./mt7612_rx_agg [secs]
//! ```

fn main() -> Result<(), Box<dyn std::error::Error>> {
    use ndn_radio_drivers::Mt7612uBackend;
    use ndn_radio_drivers::rx_pump::Pumpable;
    use std::collections::BTreeMap;
    use std::time::{Duration, Instant};

    let secs: u64 = std::env::args().nth(1).and_then(|s| s.parse().ok()).unwrap_or(20);

    println!("opening MT7612U ...");
    let dev = Mt7612uBackend::open()?;
    println!("bring-up (firmware + MAC/BB init) ...");
    dev.bring_up()?;
    println!("chip 0x{:04x}", dev.chip_id()?);
    dev.set_channel_ch6()?;
    dev.setup_monitor_rx()?;
    dev.pause_drain(true); // the init drain would steal the transfers we want to inspect

    // The port (#80) is what makes these reachable without a bespoke accessor.
    let handle = dev.pump_handle();
    let ep = dev.pump_bulk_in();
    println!("listening on bulk-IN ep {ep:#04x} for {secs}s (ch6, monitor)\n");

    /// mt76 RX unit header: 4-byte MT_RX_INFO (the first 2 bytes are a little-endian length),
    /// then RXWI + the 802.11 frame, then a 4-byte FCE trailer. `MT76_RXD_LEN` (36) = info + RXWI.
    const INFO_LEN: usize = 4;
    const FCE_LEN: usize = 4;

    let mut buf = vec![0u8; 32768];
    let mut units_hist: BTreeMap<usize, u64> = BTreeMap::new();
    let mut sizes: Vec<usize> = Vec::new();
    let (mut transfers, mut mismatches, mut total_units) = (0u64, 0u64, 0u64);

    let end = Instant::now() + Duration::from_secs(secs);
    while Instant::now() < end {
        let n = match handle.read_bulk(ep, &mut buf, Duration::from_millis(200)) {
            Ok(n) if n > 0 => n,
            _ => continue,
        };
        transfers += 1;
        sizes.push(n);

        // Walk the transfer as a chain of declared-length units.
        let (mut off, mut units, mut ok) = (0usize, 0usize, true);
        while off + INFO_LEN <= n {
            let dma_len = u16::from_le_bytes([buf[off], buf[off + 1]]) as usize;
            if dma_len == 0 {
                break; // padding to the end of the transfer
            }
            // unit = info + declared body + FCE trailer, 4-byte aligned (mt76 pads units).
            let unit = (INFO_LEN + dma_len + FCE_LEN).div_ceil(4) * 4;
            if off + unit > n {
                ok = false; // the declared length runs past the buffer → our reading is wrong
                break;
            }
            units += 1;
            off += unit;
        }
        if !ok {
            mismatches += 1;
        }
        total_units += units as u64;
        *units_hist.entry(units).or_default() += 1;
    }

    println!("=== MT7612U bulk-IN aggregation ===");
    println!("transfers            : {transfers}");
    if transfers == 0 {
        println!("\nNo transfers. Nothing was on air, or the device is still kernel-bound.");
        return Ok(());
    }
    sizes.sort_unstable();
    let avg: usize = sizes.iter().sum::<usize>() / sizes.len();
    println!(
        "transfer bytes       : min {} med {} max {} avg {avg}",
        sizes[0],
        sizes[sizes.len() / 2],
        sizes[sizes.len() - 1]
    );
    println!("total RX units       : {total_units}");
    println!("units/transfer       :");
    for (u, c) in &units_hist {
        println!("   {u:>3} unit(s) : {c:>7} transfers");
    }
    println!(
        "TILING MISMATCH      : {mismatches} ({:.1}%)",
        100.0 * mismatches as f64 / transfers as f64
    );

    let multi: u64 = units_hist.iter().filter(|(u, _)| **u > 1).map(|(_, c)| *c).sum();
    println!("\n--- verdict ---");
    if mismatches * 5 > transfers {
        println!(
            "INCONCLUSIVE: {mismatches}/{transfers} transfers did not tile. The 16-bit head is not \
             the unit length this assumes — fix the instrument before trusting any count above."
        );
    } else if multi == 0 {
        println!(
            "NO AGGREGATION: every transfer carried exactly one RX unit. `decode_rx`'s \
             whole-transfer-is-one-frame assumption holds; #80's worry does not apply to this chip."
        );
    } else {
        println!(
            "AGGREGATION CONFIRMED: {multi}/{transfers} transfers carried >1 RX unit \
             ({:.2} units/transfer average). `decode_rx` returns only the first, so \
             `Pumpable::parse_transfer` must walk the chain — the rest are being dropped today.",
            total_units as f64 / transfers as f64
        );
    }
    Ok(())
}
