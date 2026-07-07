//! Trace RF register 0x01 (the RF TX AGC) through each bring-up step, to find where it
//! is set / stays 0. The vendor holds it at 0x1a (radiating); mine idles at 0 (dead TX).
//!   cargo run --example rf01trace

fn main() -> Result<(), Box<dyn std::error::Error>> {
    use ndn_radio_drivers::Rtl8733buBackend;
    let d = Rtl8733buBackend::open()?;
    let rf01 = |d: &Rtl8733buBackend| d.rf_read(0x01).map(|v| v & 0xfffff).unwrap_or(0xf_ffff);
    let rf56 = |d: &Rtl8733buBackend| d.rf_read(0x56).map(|v| v & 0xfffff).unwrap_or(0xf_ffff);

    d.power_on()?;
    d.fw_dl_setup()?;
    d.download_firmware()?;
    d.mac_config()?;
    d.bb_config()?;
    println!("after bb_config:   RF01=0x{:05x}  RF56=0x{:05x}", rf01(&d), rf56(&d));
    d.rf_config()?;
    println!("after rf_config:   RF01=0x{:05x}  RF56=0x{:05x}", rf01(&d), rf56(&d));
    d.init_trx()?;
    println!("after init_trx:    RF01=0x{:05x}", rf01(&d));
    d.tune_channel(1)?;
    println!("after tune_ch1:    RF01=0x{:05x}", rf01(&d));
    d.set_monitor()?;
    println!("after set_monitor: RF01=0x{:05x}", rf01(&d));
    d.enable_tx_path()?;
    println!("after enable_tx:   RF01=0x{:05x}", rf01(&d));
    // Try a manual write here and see if it sticks at this point.
    d.set_rf_txagc(0x1a)?;
    println!("after set_rf_txagc(0x1a): RF01=0x{:05x}", rf01(&d));
    Ok(())
}
