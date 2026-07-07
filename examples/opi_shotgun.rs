//! Shotgun: apply every vendor-only init write (from the write-seq diff, excl cal + efuse)
//! after bring-up, then inject. If it radiates, the TX gate is in this set → bisect.
use ndn_radio_drivers::Rtl8733buBackend;
use std::time::{Duration, Instant};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let ch: u8 = 36;
    let d = Rtl8733buBackend::open()?;
    d.bring_up_monitor(ch)?;
    // (addr, width_bytes, value)
    let writes: &[(u16, u8, u32)] = &[
        (0x0049, 1, 0x00000000),
        (0x0091, 1, 0x00000086),
        (0x00ac, 4, 0x02000080),
        (0x00cd, 1, 0x00000000),
        (0x00f5, 1, 0x00000000),
        (0x1004, 1, 0x00000002),
        (0x103c, 4, 0xff1b8d00),
        (0x1103, 1, 0x0000000c),
    ];
    for &(a, w, v) in writes {
        let _ = match w {
            1 => d.write8(a, v as u8),
            2 => d.write16(a, v as u16),
            _ => d.write32(a, v),
        };
    }
    d.write8(0x0522, 0x00)?;
    println!("shotgun applied {} writes; RF18=0x{:05x}; inject", writes.len(), d.rf_read(0x18)?);
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
