//! Send the exact H2C firmware command sequence the vendor issues during init (which my
//! bring-up skips), then inject. H2C = write EXT box then MAIN box, per command, in order.
//! Values byte-swapped from the usbmon LE display so write32 emits the vendor's bytes.
use ndn_radio_drivers::Rtl8733buBackend;
use std::time::{Duration, Instant};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let d = Rtl8733buBackend::open()?;
    d.bring_up_monitor(36)?;
    // (ext_reg, ext_val, main_reg, main_val) — matches vendor usbmon bytes exactly.
    let h2c: &[(u16, u32, u16, u32)] = &[
        (0x01f0, 0x0000_0011, 0x01d0, 0x0300_004c),
        (0x01f0, 0x0000_0011, 0x01d0, 0x0300_004c),
        (0x01f4, 0x0000_0000, 0x01d4, 0x0000_016d),
        (0x01f8, 0x0000_0000, 0x01d8, 0x0000_006d),
        (0x01fc, 0x0000_0000, 0x01dc, 0x0000_0060),
    ];
    for &(er, ev, mr, mv) in h2c {
        let _ = d.write32(er, ev);
        let _ = d.write32(mr, mv);
        std::thread::sleep(Duration::from_millis(10));
    }
    d.write8(0x0522, 0x00)?;
    println!("sent {} H2C cmds; RF18=0x{:05x}; inject", h2c.len(), d.rf_read(0x18)?);
    let mut f = vec![0x08u8, 0, 0, 0];
    f.extend_from_slice(&[0xff; 6]);
    f.extend_from_slice(&[0x02, 0x4d, 0x59, 0x44, 0x52, 0x56]);
    f.extend_from_slice(&[0x02, 0x4d, 0x59, 0x44, 0x52, 0x56]);
    f.extend_from_slice(&[0, 0]);
    f.extend_from_slice(b"MYDRV8733-INJECT");
    let deadline = Instant::now() + Duration::from_secs(15);
    let mut s = 0u16;
    while Instant::now() < deadline {
        let _ = d.inject_raw(&f, 0x04, s);
        s = s.wrapping_add(1);
        std::thread::sleep(Duration::from_millis(15));
    }
    println!("sent {s}");
    Ok(())
}
