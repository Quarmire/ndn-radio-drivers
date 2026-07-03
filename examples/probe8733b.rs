//! M1+M2 hardware probe: open an RTL8731BU/8733BU, read its chip identity, then
//! run the card-enable power-on sequence and confirm the MAC came up.
//! Run: `cargo run --example probe8733b` (needs USB access — sudo on Linux).

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let dev = ndn_radio_drivers::Rtl8733buBackend::open()?;

    // M1: identify.
    let v = dev.chip_version()?;
    println!(
        "M1  chip_id=0x{:02x}  cut=0x{:x}  sys_cfg1=0x{:08x}  eps out=0x{:02x}/in=0x{:02x}  alive={}",
        v.chip_id, v.chip_ver, v.sys_cfg1, dev.bulk_out(), dev.bulk_in(), v.looks_alive(),
    );

    // M2: power on.
    let cr_before = dev.read_cr()?;
    let func_before = dev.read_sys_func_en()?;
    print!("M2  before: CR=0x{cr_before:02x} SYS_FUNC_EN=0x{func_before:02x}  → power_on … ");
    match dev.power_on() {
        Ok(()) => {
            let cr = dev.read_cr()?;
            let func = dev.read_sys_func_en()?;
            let ok = dev.is_powered()?;
            println!("done. after: CR=0x{cr:02x} SYS_FUNC_EN=0x{func:02x}  powered={ok}");
            if !ok {
                eprintln!("warning: sequence completed but MAC does not read as powered");
            }

            // M3 groundwork: firmware image + header decode + secure path.
            let fw = ndn_radio_drivers::Rtl8733buBackend::firmware();
            let h = ndn_radio_drivers::Rtl8733buBackend::fw_header()?;
            let secure = dev.fw_secure()?;
            println!(
                "M3  fw {} B  sig=0x{:04x} v{}.{}  dmem={}B@0x{:08x} imem={}B@0x{:08x} emem={}B  hdr_ok={}  secure_path={}",
                fw.len(), h.signature, h.version, h.subversion,
                h.dmem_size, h.dmem_addr, h.imem_size, h.imem_addr, h.emem_size,
                h.nonsecure_len() == fw.len() as u32,
                secure,
            );
        }
        Err(e) => println!("FAILED: {e}"),
    }
    Ok(())
}
