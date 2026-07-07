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
        (0x1428, 4, 0xffffff1f),
        (0x142c, 4, 0xffffff1f),
        (0x1430, 4, 0x00030000),
        (0x1448, 1, 0x00000006),
        (0x144a, 1, 0x00000006),
        (0x144c, 1, 0x00000006),
        (0x144e, 1, 0x00000006),
        (0x1610, 2, 0x0000873f),
        (0x1612, 2, 0x00005e05),
        (0x1614, 4, 0xffffffff),
        (0x169c, 2, 0x0000fc00),
        (0x3c68, 4, 0x002c0000),
        (0x3d54, 4, 0x80000000),
        (0x3d80, 4, 0x00300300),
        (0x3dfc, 4, 0x00010000),
        (0x3ee0, 4, 0x41640900),
        (0x3ee4, 4, 0x00000800),
        (0x3f20, 4, 0x00000000),
        (0x4200, 4, 0x00000000),
        (0x4204, 4, 0x00000000),
        (0x4208, 4, 0x00000000),
        (0x420c, 4, 0x00000000),
        (0x4210, 4, 0x00000000),
        (0x4214, 4, 0x00000000),
        (0x4218, 4, 0x00000000),
        (0x421c, 4, 0x00000000),
        (0x4220, 4, 0x00000000),
        (0x4224, 4, 0x00000000),
        (0x4228, 4, 0x00000000),
        (0x422c, 4, 0x00000000),
        (0x4230, 4, 0x00000000),
        (0x4234, 4, 0x00000000),
        (0x4238, 4, 0x00000000),
        (0x423c, 4, 0x00000000),
        (0x4c68, 4, 0x00200000),
        (0x4ccc, 4, 0x0c000000),
        (0x4cfc, 4, 0xabf40300),
        (0x4d54, 4, 0x80000000),
        (0x4d5c, 4, 0x8dd80b00),
        (0x4d80, 4, 0x00000000),
        (0x4d98, 4, 0x00410100),
        (0x4dfc, 4, 0x00010000),
        (0x4e1c, 4, 0x03200000),
        (0x4fbc, 4, 0x00000000),
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
