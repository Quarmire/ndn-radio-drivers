//! **Which vendor request actually powers on an MT7921AU?**
//!
//! `mt792xu_mcu_power_on` (`mt792x_usb.c:214-231`) sends `MT_VEND_POWER_ON` (0x04) with
//! `USB_DIR_OUT | MT_USB_TYPE_VENDOR`, where `MT_USB_TYPE_VENDOR = USB_TYPE_VENDOR | 0x1f`
//! (`mt792x.h:555`) — i.e. `bmRequestType = 0x5f`, whose low five bits are a **vendor-specific
//! recipient**, not the standard "device" recipient 0. Our transport uses the plain `0x40`,
//! which MEASURABLY works for register reads (`MT_HW_CHIPID` returned `0x7961` through it).
//!
//! Those two facts do not settle whether POWER_ON also tolerates `0x40`, and the port's first
//! bring-up attempt timed out waiting for `FW_PWR_ON`. So: try each form, poll `MT_CONN_ON_MISC`
//! after it, and let the register say which one the bootrom honours. Reasoning about this
//! silicon has a poor track record; a four-transfer experiment does not.
//!
//!   sudo ./mt7921_pwr
use rusb::{Context, UsbContext};
use std::time::{Duration, Instant};

const VID: u16 = 0x0e8d;
const PID: u16 = 0x7961;
const MT_VEND_READ_EXT: u8 = 0x63;
const MT_VEND_POWER_ON: u8 = 0x04;
const MT_CONN_ON_MISC: u32 = 0x7c06_00f0;
const FW_PWR_ON: u32 = 1 << 0;
const FW_N9_RDY: u32 = 1 << 1;
const T: Duration = Duration::from_millis(500);

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let ctx = Context::new()?;
    let dev = ctx
        .devices()?
        .iter()
        .find(|d| {
            d.device_descriptor()
                .map(|s| s.vendor_id() == VID && s.product_id() == PID)
                .unwrap_or(false)
        })
        .ok_or("no 0e8d:7961")?;
    // The WLAN function is the class ff/ff/ff interface; 0-2 are Bluetooth and belong to btusb.
    let cfg = dev.active_config_descriptor()?;
    let wlan_if = cfg
        .interfaces()
        .flat_map(|i| i.descriptors())
        .find(|d| d.class_code() == 0xff)
        .map(|d| d.interface_number())
        .ok_or("no vendor-class interface")?;
    let h = dev.open()?;
    let _ = h.set_auto_detach_kernel_driver(true);
    h.claim_interface(wlan_if)?;
    println!("claimed WLAN interface {wlan_if}");

    let rr = |a: u32| -> Result<u32, rusb::Error> {
        let mut b = [0u8; 4];
        h.read_control(
            0xc0,
            MT_VEND_READ_EXT,
            (a >> 16) as u16,
            (a & 0xffff) as u16,
            &mut b,
            T,
        )?;
        Ok(u32::from_le_bytes(b))
    };
    let show = |tag: &str| match rr(MT_CONN_ON_MISC) {
        Ok(v) => println!(
            "   [{tag}] MT_CONN_ON_MISC = {v:#010x}  FW_PWR_ON={} FW_N9_RDY={}",
            v & FW_PWR_ON != 0,
            v & FW_N9_RDY != 0
        ),
        Err(e) => println!("   [{tag}] read failed: {e}"),
    };

    println!(
        "MT_HW_CHIPID = {:#010x}  MT_HW_REV = {:#010x}",
        rr(0x7001_0200)?,
        rr(0x7001_0204)?
    );
    show("as found");

    // Each candidate is (bmRequestType, wValue, wIndex). Upstream is (0x5f, 0x0, 0x1); the rest
    // vary one axis at a time so a pass identifies WHICH axis mattered, not merely that one combo
    // worked.
    let candidates: [(u8, u16, u16, &str); 4] = [
        (
            0x40,
            0x0,
            0x1,
            "plain 0x40, val 0 idx 1 (what the port sends today)",
        ),
        (
            0x5f,
            0x0,
            0x1,
            "upstream 0x5f, val 0 idx 1 (mt792x_usb.c:218-220)",
        ),
        (0x5e, 0x0, 0x1, "UHW 0x5e, val 0 idx 1"),
        (
            0x5f,
            0x1,
            0x0,
            "0x5f with value/index swapped — rules out a transcription slip",
        ),
    ];
    for (req_type, val, idx, label) in candidates {
        println!("\n== POWER_ON via {label}");
        match h.write_control(req_type, MT_VEND_POWER_ON, val, idx, &[], T) {
            Ok(_) => println!("   control write accepted"),
            Err(e) => {
                println!("   control write FAILED: {e}");
                continue;
            }
        }
        let t = Instant::now();
        let mut on = false;
        while t.elapsed() < Duration::from_millis(500) {
            if matches!(rr(MT_CONN_ON_MISC), Ok(v) if v & FW_PWR_ON != 0) {
                on = true;
                break;
            }
            std::thread::sleep(Duration::from_millis(10));
        }
        show("after");
        if on {
            println!(
                "   ★★ FW_PWR_ON came up after {:?} — THIS is the form the bootrom honours",
                t.elapsed()
            );
            return Ok(());
        }
        println!("   no FW_PWR_ON within 500 ms");
    }
    println!("\nNone of the four forms raised FW_PWR_ON. The next suspects, in order:");
    println!(
        "  * the chip needs a WFSYS reset first (upstream skips it only when FW_N9_RDY is set,"
    );
    println!(
        "    and FW_N9_RDY reads 0 here — but 'not ready' is not the same as 'freshly reset');"
    );
    println!(
        "  * upstream's probe opens with usb_reset_device(), which this port refuses on purpose;"
    );
    println!("  * MT_CONN_ON_MISC is not readable before power-on and the poll can never succeed.");
    Ok(())
}
