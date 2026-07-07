//! Full ordered replay of the vendor's init: power-on + FW download (my code), then replay
//! every vendor control-write in captured order (MAC/BB/RF/channel/monitor config), then
//! inject. Tests whether the TX gate is order-dependent — the last black-box avenue.
use ndn_radio_drivers::Rtl8733buBackend;
use std::time::{Duration, Instant};

const REPLAY: &[u8] = include_bytes!("../fw/rtl8733b/vendor_init_replay.bin");

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let d = Rtl8733buBackend::open()?;
    // minimal base: power + firmware (replay skips power/efuse/MCUFWDL regions)
    d.power_on()?;
    d.fw_dl_setup()?;
    d.download_firmware()?;
    // replay vendor config writes in order (addr u16 LE, width u8, pad, value u32 LE)
    let n = REPLAY.len() / 8;
    for i in 0..n {
        let b = &REPLAY[i * 8..i * 8 + 8];
        let addr = u16::from_le_bytes([b[0], b[1]]);
        let width = b[2];
        let val = u32::from_le_bytes([b[4], b[5], b[6], b[7]]);
        let _ = match width {
            1 => d.write8(addr, val as u8),
            2 => d.write16(addr, val as u16),
            _ => d.write32(addr, val),
        };
    }
    let rf18 = d.rf_read(0x18).unwrap_or(0);
    println!("replayed {n} writes; RF18=0x{rf18:05x}; injecting");
    d.write8(0x0522, 0x00)?;
    let mut f = vec![0x08u8, 0, 0, 0];
    f.extend_from_slice(&[0xff; 6]);
    f.extend_from_slice(&[0x02, 0x4d, 0x59, 0x44, 0x52, 0x56]);
    f.extend_from_slice(&[0x02, 0x4d, 0x59, 0x44, 0x52, 0x56]);
    f.extend_from_slice(&[0, 0]);
    f.extend_from_slice(b"MYDRV8733-INJECT");
    let deadline = Instant::now() + Duration::from_secs(12);
    let mut s = 0u16;
    while Instant::now() < deadline {
        let _ = d.inject_raw(&f, 0x04, s);
        s = s.wrapping_add(1);
        std::thread::sleep(Duration::from_millis(15));
    }
    println!("sent {s}");
    Ok(())
}
