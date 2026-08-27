//! Why does the 8733b deliver ambient frames after `probe8733b`'s hand-rolled sequence but not
//! after `bring_up_monitor` (RAW_PULL=0 through `open_named_radio`)? This splits the two candidate
//! faults apart on real silicon instead of by reading code:
//!
//!   stage A = `bring_up_monitor` then a DIRECT `capture()` — no pump involved.
//!             0 frames here ⇒ the fault is in the bring-up sequence.
//!             frames here ⇒ the bring-up is fine and the fault is in the RX pump.
//!
//! Then, if A is the culprit, re-run the bring-up in pieces: `bring_up_monitor` ends with four
//! steps `probe8733b` never performs (`set_monitor`, `enable_tx_path`, `set_txagc_table`,
//! `configure_trsw`). Stage B replays the probe's own prefix and captures after EACH of those, so
//! the exact step that kills RX names itself.
//!
//!   sudo ./rx8733b_bisect [channel]

use ndn_radio_drivers::Rtl8733buBackend;

fn cap(dev: &Rtl8733buBackend, label: &str) -> usize {
    // Same 2 s ambient window probe8733b's M8 uses, so the numbers are directly comparable.
    let mut total = 0;
    for _ in 0..20 {
        total += dev.capture(100).map(|v| v.len()).unwrap_or(0);
    }
    println!("    capture after {label:<24} = {total} frames");
    total
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let ch: u8 = std::env::args()
        .nth(1)
        .and_then(|s| s.parse().ok())
        .unwrap_or(6);

    println!("stage A: bring_up_monitor({ch}) + direct capture (pump bypassed)");
    let dev = Rtl8733buBackend::open()?;
    dev.bring_up_monitor(ch)?;
    let a = cap(&dev, "bring_up_monitor");
    drop(dev);

    if a > 0 {
        println!("\n=> bring-up is FINE; the fault is in the RX pump path.");
        return Ok(());
    }

    println!(
        "\nstage B: probe8733b's prefix, then bring_up_monitor's four extra steps one at a time"
    );
    let dev = Rtl8733buBackend::open()?;
    dev.power_on()?;
    dev.fw_dl_setup()?;
    dev.download_firmware()?;
    dev.mac_config()?;
    dev.bb_config()?;
    dev.rf_config()?;
    dev.init_trx()?;
    let _ = dev.rfk_init();
    dev.tune_channel(ch)?;
    let base = cap(&dev, "probe prefix (baseline)");

    dev.set_monitor()?;
    cap(&dev, "+ set_monitor");
    dev.enable_tx_path()?;
    cap(&dev, "+ enable_tx_path");
    dev.set_txagc_table(0x2d)?;
    cap(&dev, "+ set_txagc_table");
    let _ = dev.configure_trsw(true);
    cap(&dev, "+ configure_trsw(true)");

    if base == 0 {
        println!(
            "\n=> even the baseline prefix saw nothing this run — the chip state carried over \
                  from the previous process. Power-cycle/replug and re-run before concluding."
        );
    }
    Ok(())
}
