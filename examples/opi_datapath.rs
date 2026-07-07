//! THE datapath test (learned from the 88xx bb_tx_datapath_init): after bring-up, write the
//! 29 BB 0x1800-0x1fff registers where my final state differs from the vendor's radiating
//! final state — notably the 0x1e40-0x1e60 TXAGC by-rate power table, which my bring-up
//! leaves at ZERO. Then inject; capture on wlu1 to confirm radiation.
use ndn_radio_drivers::Rtl8733buBackend;
use std::time::{Duration, Instant};

// (addr, vendor-final-value) — the diff of my bring_up vs vendor final BB state.
const DATAPATH: &[(u16, u32)] = &[
    (0x180c, 0x17f43863), (0x18ac, 0x00065a60), (0x1968, 0x36632640),
    (0x1c38, 0xffb5005e), (0x1c3c, 0x01051f43), (0x1c80, 0x0f38e000), (0x1c84, 0x24512054),
    (0x1ca4, 0xe0000000), (0x1d70, 0x2020201c), (0x1e1c, 0x8400b000),
    (0x1e40, 0xfffeffff), (0x1e44, 0x2824201c), (0x1e48, 0x3834302c), (0x1e50, 0x2824201c),
    (0x1e54, 0x3834302c), (0x1e58, 0xfe44403c), (0x1e5c, 0xc13c00ff), (0x1e60, 0x4440413f),
    (0x1e88, 0x0000fc1c), (0x1e8c, 0x00007000), (0x1eb8, 0x00000b00),
    (0x1ed4, 0x800c0040), (0x1ed8, 0x8005000c), (0x1edc, 0x80020005), (0x1ee0, 0x80000002),
    (0x1ee4, 0xf0000000), (0x1ef0, 0x30000a80), (0x1ef4, 0x40001266), (0x1ef8, 0x3b000100),
];

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let ch: u8 = std::env::args().nth(1).and_then(|s| s.parse().ok()).unwrap_or(36);
    let d = Rtl8733buBackend::open()?;
    d.bring_up_monitor(ch)?;
    // Apply the datapath delta (subset via ONLY=0x1e40 to bisect: env ONLY writes just the TXAGC block).
    let only_txagc = std::env::var("TXAGC").is_ok();
    for &(a, v) in DATAPATH {
        if only_txagc && !(0x1e40..=0x1e60).contains(&a) { continue; }
        d.write32(a, v)?;
    }
    println!("applied {} datapath regs; 0x1e44 now=0x{:08x}", DATAPATH.len(), d.read32(0x1e44)?);
    // 88xx ordering: datapath init THEN re-tune the channel (re-locks RF/BB), then re-assert.
    if std::env::var("NORETUNE").is_err() {
        d.tune_channel(ch)?;
        for &(a, v) in DATAPATH { d.write32(a, v)?; }
        println!("re-tuned ch{ch}; 0x1e44=0x{:08x}", d.read32(0x1e44)?);
    }
    d.write8(0x0522, 0x00)?;
    let mut f = vec![0x08u8, 0, 0, 0];
    f.extend_from_slice(&[0xff; 6]);
    f.extend_from_slice(&[0x02, 0x4d, 0x59, 0x44, 0x52, 0x56]);
    f.extend_from_slice(&[0x02, 0x4d, 0x59, 0x44, 0x52, 0x56]);
    f.extend_from_slice(&[0, 0]);
    f.extend_from_slice(b"MYDRV8733-INJECT");
    // Finding 4: the kernel writes 0x1e40-0x1e60 (power-tracking) CONTINUOUSLY during TX.
    // Re-assert the whole datapath block before each frame in case a FW DM resets it.
    let deadline = Instant::now() + Duration::from_secs(12);
    let mut s = 0u16;
    while Instant::now() < deadline {
        for &(a, v) in DATAPATH { let _ = d.write32(a, v); }
        let _ = d.inject_raw(&f, 0x04, s);
        s = s.wrapping_add(1);
        std::thread::sleep(Duration::from_millis(15));
    }
    println!("sent {s}");
    Ok(())
}
