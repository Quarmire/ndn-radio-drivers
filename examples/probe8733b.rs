//! M1 hardware probe: open an RTL8731BU/8733BU and read its chip identity.
//! Run: `cargo run --example probe8733b` (needs USB access — sudo on Linux).

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let dev = ndn_radio_drivers::Rtl8733buBackend::open()?;
    let v = dev.chip_version()?;
    println!(
        "RTL8733B probe: chip_id=0x{:02x}  cut=0x{:x}  sys_cfg1=0x{:08x}  bulk_out=0x{:02x} bulk_in=0x{:02x}  alive={}",
        v.chip_id,
        v.chip_ver,
        v.sys_cfg1,
        dev.bulk_out(),
        dev.bulk_in(),
        v.looks_alive(),
    );
    if !v.looks_alive() {
        eprintln!("warning: registers read back dead (all-ones/zero) — device may be unclaimed or in a bootstrap mode");
    }
    Ok(())
}
