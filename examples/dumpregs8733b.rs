//! Dump the 8733b's TX-path registers after bring_up_monitor and diff against the
//! vendor driver's live values (captured via procfs on the OPi at ch1 monitor). The
//! DIFFs point at whatever TX-enable my driver is missing (the hardware radiates under
//! the vendor; mine doesn't, with a verified witness).
//!   cargo run --example dumpregs8733b -- 1

fn main() -> Result<(), Box<dyn std::error::Error>> {
    use ndn_radio_drivers::Rtl8733buBackend;
    let ch: u8 = std::env::args().nth(1).and_then(|s| s.parse().ok()).unwrap_or(1);
    let dut = Rtl8733buBackend::open()?;
    dut.bring_up_monitor(ch)?;
    dut.set_tx_power_idx(0x7f)?;
    dut.write8(0x0522, 0x00)?;

    // Vendor reference values (OPi, rtl8733bu, monitor ch1, TX-configured).
    let refs: &[(u16, u32, &str)] = &[
        (0x1800, 0x0003_3312, "RF mode table (2=TX)"),
        (0x1884, 0x0000_0000, "path select"),
        (0x2a00, 0x0400_1418, "BB CCK TX (bit1=0=on)"),
        (0x2a24, 0x0000_0000, "CCK CCA"),
        (0x3a00, 0xc0c0_c0c0, "per-rate TXAGC CCK"),
        (0x3a04, 0x0808_0c0c, "per-rate TXAGC OFDM"),
        (0x4308, 0x5c54_5c50, "TXAGC ref"),
        (0x0608, 0x9000_0041, "RCR"),
        (0x0818, 0x6300_56f1, "chan/BW"),
        (0x0c10, 0x0000_00b0, "chan/BW"),
        (0x0c24, 0x4060_00ff, "chan/BW"),
        (0x0884, 0x1de7_134f, "chan/BW"),
        (0x1908, 0x0077_779a, "chan/BW"),
        (0x1e70, 0x0000_1000, "BB TX-block"),
        (0x1c3c, 0x0105_1f43, "BB per-TX"),
    ];
    println!("reg     mine        vendor(ch1)  ");
    for (a, v, note) in refs {
        let mine = dut.read32(*a)?;
        let d = if mine != *v { "  <<< DIFF" } else { "  ok" };
        println!("0x{a:04x}  0x{mine:08x}  0x{v:08x}{d}  [{note}]");
    }
    println!("--- RF (path A) ---");
    for rf in [0x00u32, 0x01, 0x18, 0x56, 0x63, 0x8f, 0xde, 0xdf] {
        println!("  RF[0x{rf:02x}] = 0x{:05x}", dut.rf_read(rf)?);
    }
    Ok(())
}
