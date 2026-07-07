//! Proper halmac H2C box protocol (rtw_halmac_send_h2c): before each command poll
//! REG_HMETFR(0x1cc) BIT(box)==0 (FW consumed prev), write EXT(0x1f0+box*4) then
//! MAIN(0x1d0+box*4), rotate box 0..3. Sends the vendor's init H2C (0x4c/0x6d/0x60),
//! then injects. Prints HMETFR so we can see the FW consuming (bit clearing).
use ndn_radio_drivers::Rtl8733buBackend;
use std::time::{Duration, Instant};

fn send_h2c(d: &Rtl8733buBackend, boxn: &mut u16, h2c: [u8; 8]) {
    // handshake: wait until FW has read this box (HMETFR bit clear)
    let mut waited = 0;
    for _ in 0..100 {
        match d.read32(0x01cc) {
            Ok(v) if v & (1 << *boxn) == 0 => break,
            _ => {
                waited += 1;
                std::thread::sleep(Duration::from_millis(1));
            }
        }
    }
    let ext = u32::from_le_bytes([h2c[4], h2c[5], h2c[6], h2c[7]]);
    let main = u32::from_le_bytes([h2c[0], h2c[1], h2c[2], h2c[3]]);
    let _ = d.write32(0x01f0 + boxn.wrapping_mul(4), ext);
    let _ = d.write32(0x01d0 + boxn.wrapping_mul(4), main);
    let after = d.read32(0x01cc).unwrap_or(0xffff) & 0xf;
    println!("h2c cmd 0x{:02x} -> box{boxn} (waited {waited}ms), HMETFR[3:0]=0x{after:x}", h2c[0]);
    *boxn = (*boxn + 1) % 4;
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let d = Rtl8733buBackend::open()?;
    d.bring_up_monitor(36)?;
    let mut boxn = 0u16;
    println!("HMETFR before: 0x{:x}", d.read32(0x01cc)? & 0xf);
    send_h2c(&d, &mut boxn, [0x4c, 0x00, 0x00, 0x03, 0x11, 0x00, 0x00, 0x00]);
    send_h2c(&d, &mut boxn, [0x6d, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]);
    send_h2c(&d, &mut boxn, [0x6d, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]);
    send_h2c(&d, &mut boxn, [0x60, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]);
    d.write8(0x0522, 0x00)?;
    println!("injecting; RF18=0x{:05x}", d.rf_read(0x18)?);
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
    println!("done {s}");
    Ok(())
}
