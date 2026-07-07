//! M10: read + hex-dump the 8733b physical efuse (tx-power cal + RF/PA/Xtal trim source).
//!   sudo ./opi_efuse

use ndn_radio_drivers::Rtl8733buBackend;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let d = Rtl8733buBackend::open()?;
    d.power_on()?;
    let n: u16 = std::env::args().nth(1).and_then(|s| s.parse().ok()).unwrap_or(512);
    let phys = d.read_efuse(n)?;
    // Decode the header-encoded physical efuse into the logical map (standard Realtek
    // 1-byte/2-byte-header block format; word_en bit=0 → word present).
    let mut logi = vec![0xffu8; 0x200];
    let mut i = 0usize;
    while i < phys.len() {
        let hdr = phys[i];
        i += 1;
        if hdr == 0xff {
            break;
        }
        let (offset, word_en) = if (hdr & 0x1f) == 0x0f {
            if i >= phys.len() {
                break;
            }
            let h2 = phys[i];
            i += 1;
            ((((h2 & 0xf0) >> 1) | ((hdr & 0xe0) >> 5)) as usize, h2 & 0x0f)
        } else {
            (((hdr & 0xf0) >> 4) as usize, hdr & 0x0f)
        };
        for w in 0..4usize {
            if word_en & (1 << w) == 0 {
                for b in 0..2 {
                    if i < phys.len() {
                        let a = offset * 8 + w * 2 + b;
                        if a < logi.len() {
                            logi[a] = phys[i];
                        }
                        i += 1;
                    }
                }
            }
        }
    }
    println!("== LOGICAL efuse map ==");
    for (r, chunk) in logi.chunks(16).enumerate() {
        if chunk.iter().all(|&b| b == 0xff) {
            continue;
        }
        print!("{:03x}: ", r * 16);
        for b in chunk {
            print!("{b:02x} ");
        }
        println!();
    }
    println!("# tx_pwr_calibrate_rate(0xC8)=0x{:02x}", logi[0xC8]);
    Ok(())
}
