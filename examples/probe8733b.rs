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

            // M4: TX-descriptor + reserved-page write, verified by BCN_VALID.
            print!("M4  fw_dl_setup … ");
            dev.fw_dl_setup()?;
            // Diagnostic: did the setup registers actually stick?
            println!(
                "\n    regs after setup: CR=0x{:02x} CR+1=0x{:02x} RQPN=0x{:08x} DWBCN0=0x{:08x} EXT_FUNC=0x{:08x} EXT_CLK=0x{:02x}",
                dev.read8(0x0100)?, dev.read8(0x0101)?, dev.read32(0x0200)?,
                dev.read32(0x0208)?, dev.read32(0x1000)?, dev.read8(0x1008)?,
            );
            let page = [0xA5u8; 128];
            match dev.dl_rsvd_page(0, &page) {
                Ok(()) => println!("dl_rsvd_page(128B @ pg0): BCN_VALID ✓ (reserved-page path works)"),
                Err(e) => println!("dl_rsvd_page: {e}"),
            }
            // M5: full firmware download + boot the WLAN CPU.
            print!("M5  download_firmware … ");
            match dev.download_firmware() {
                Ok(()) => {
                    println!(
                        "✓ FW BOOTED (MCUFW_CTRL=0x{:04x}, FW_DBG7=0x{:08x})",
                        dev.read16(0x0080)?,
                        dev.read32(0x10AC)?
                    );
                    // M6a: MAC register table.
                    print!("M6a mac_config … ");
                    dev.mac_config()?;
                    println!(
                        "✓ (0x002=0x{:02x} exp C3, 0x520=0x{:02x} exp 6f, 0x4CA=0x{:02x} exp 3f)",
                        dev.read8(0x002)?,
                        dev.read8(0x520)?,
                        dev.read8(0x4CA)?
                    );
                    // M6b: baseband (phy_reg + agc_tab).
                    print!("M6b bb_config … ");
                    dev.bb_config()?;
                    println!(
                        "✓ (0x804=0x{:08x} exp d6300000, 0x808=0x{:08x} exp 60156024)",
                        dev.read32(0x804)?,
                        dev.read32(0x808)?
                    );
                    // M6c: RF/radioA.
                    print!("M6c rf_config … ");
                    dev.rf_config()?;
                    println!(
                        "✓ (RF-0x00=0x{:05x}, RF-0x18=0x{:05x})",
                        dev.rf_read(0x00)?,
                        dev.rf_read(0x18)?
                    );
                    // M6 tail: normal-mode TRX/queue init (auto-LLT completion validates).
                    print!("M6d init_trx … ");
                    match dev.init_trx() {
                        Ok(()) => println!(
                            "✓ LLT done (CR=0x{:02x} exp ff, RQPN=0x{:08x})",
                            dev.read8(0x0100)?,
                            dev.read32(0x0200)?
                        ),
                        Err(e) => println!("FAILED: {e}"),
                    }
                    // RF-K init: load the NCTL/KIP microcode (prerequisite for IQK/DPK).
                    print!("rfk_init … ");
                    dev.rfk_init()?;
                    println!("✓ (cal-init microcode loaded)");
                    // Tune to a 2.4 GHz channel (prerequisite for calibration convergence).
                    print!("set_channel(6) … ");
                    dev.set_channel(6)?;
                    println!("✓ (RF18=0x{:05x})", dev.rf_read(0x18)?);
                    if std::env::var("NCTLDBG").is_ok() {
                        dev.nctl_debug()?;
                    }
                    // M7: full IQK calibration (LOK → TXK → RXK → apply coefficients).
                    print!("M7 phy_iq_calibrate … ");
                    match dev.phy_iq_calibrate() {
                        Ok(info) => println!(
                            "✓ txxy=[{:03x},{:03x}] rxxy_s=[{:03x},{:03x}] rxxy_l=[{:03x},{:03x}] rxk_ok={}",
                            info.txxy[0][0], info.txxy[0][1],
                            info.rxxy[0][0][0], info.rxxy[0][1][0],
                            info.rxxy[0][0][1], info.rxxy[0][1][1],
                            info.rxk_ok,
                        ),
                        Err(e) => println!("FAILED: {e}"),
                    }
                    // M7 DPK: gain-loss/auto-AGC + per-path DO_DPK.
                    print!("M7 phy_dpk … ");
                    match dev.phy_dpk() {
                        Ok((agc, fail)) => println!("✓ agc_done={agc} dpk_fail={fail} (agc 1=ok, fail 0=ok)"),
                        Err(e) => println!("FAILED: {e}"),
                    }
                }
                Err(e) => {
                    println!("FAILED: {e}");
                    // Is the CPU actually executing? Sample the PC counter (FW_DBG7).
                    for _ in 0..4 {
                        println!(
                            "    MCUFW=0x{:04x} FW_DBG7=0x{:08x}",
                            dev.read16(0x0080)?,
                            dev.read32(0x10AC)?
                        );
                    }
                }
            }
        }
        Err(e) => println!("FAILED: {e}"),
    }
    Ok(())
}
