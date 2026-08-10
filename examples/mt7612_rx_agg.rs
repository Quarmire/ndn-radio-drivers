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
//! ## BLOCKED as of 2026-08-10 — this has not yet produced a measurement
//!
//! On minidronesys-05 (ODROID-C4, SuperSpeed) `Mt7612uBackend::bring_up` fails
//! **deterministically** in the firmware upload: `fw chunk 1 (dst 0x838f8) bulk: Operation timed
//! out`, twice, at the identical address. Chunk 0 uploads fine. It is not this example, not the #80
//! RX-pump port (whose diff does not touch the upload path), and not a kernel-re-grab race —
//! `drivers_autoprobe=0` changed nothing.
//!
//! The diagnosis, from our own code plus the kernel's log: the inter-chunk handshake polls
//! `MT_FCE_PSE_CTRL_GO` (0x09a8) to wait for the FCE to pick up and drain a chunk, and the kernel
//! driver reports `vendor request req:06 off:09a8 failed:-110` on the same register — **the control
//! read itself times out on this host**. Our poll loops use `.unwrap_or(false)`, so a failed read is
//! indistinguishable from "not ready": both waits run their full timeout, the code writes the
//! advance anyway, and the next chunk NAKs. `mcu_fw_send_data`'s own comment already warns this
//! handshake is fragile on "fast USB stacks"; the C4's SuperSpeed path is evidently faster still.
//!
//! Fixing it means distinguishing a failed poll from a negative one and reacting (retry/settle)
//! instead of advancing blind — and each attempt wedges the dongle (`ASIC revision: ffffffff`) until
//! it is physically replugged, so it needs a session with someone at the bench.
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
