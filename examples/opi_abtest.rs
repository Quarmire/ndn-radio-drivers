//! Within-one-boot A/B: after a single bring-up, Phase A injects PLAIN (no datapath) with
//! src ...AA, then apply the BB datapath delta and Phase B injects with src ...BB. One
//! witness capture scores AA vs BB by source MAC — eliminating the boot-fragility confound.
//!   score: tcpdump -e -r rad.pcap | grep -c "44:52:AA"  (plain)  vs  "44:52:BB" (datapath)
use ndn_radio_drivers::Rtl8733buBackend;
use std::time::{Duration, Instant};

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

fn frame(tag: u8) -> Vec<u8> {
    let mut f = vec![0x08u8, 0, 0, 0];
    f.extend_from_slice(&[0xff; 6]);
    f.extend_from_slice(&[0x02, 0x4d, 0x59, 0x44, 0x52, tag]); // src ...tag
    f.extend_from_slice(&[0x02, 0x4d, 0x59, 0x44, 0x52, tag]);
    f.extend_from_slice(&[0, 0]);
    f.extend_from_slice(b"MYDRV8733-INJECT");
    f
}

fn burst(d: &Rtl8733buBackend, tag: u8, secs: u64) {
    let fr = frame(tag);
    let deadline = Instant::now() + Duration::from_secs(secs);
    let mut s = 0u16;
    while Instant::now() < deadline {
        let _ = d.inject_raw(&fr, 0x04, s);
        s = s.wrapping_add(1);
        std::thread::sleep(Duration::from_millis(15));
    }
    println!("  burst tag=0x{tag:02x} sent {s}");
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let ch: u8 = std::env::args().nth(1).and_then(|s| s.parse().ok()).unwrap_or(36);
    let secs: u64 = std::env::args().nth(2).and_then(|s| s.parse().ok()).unwrap_or(12);
    let d = Rtl8733buBackend::open()?;
    d.bring_up_monitor(ch)?;
    d.write8(0x0522, 0x00)?;
    // Phase A: PLAIN (no datapath), src ...AA
    println!("Phase A PLAIN (src ..AA)");
    burst(&d, 0xAA, secs);
    // Phase B: apply datapath + re-tune, src ...BB
    for &(a, v) in DATAPATH { d.write32(a, v)?; }
    d.tune_channel(ch)?;
    for &(a, v) in DATAPATH { d.write32(a, v)?; }
    d.write8(0x0522, 0x00)?;
    println!("Phase B DATAPATH (src ..BB); 0x1e44=0x{:08x}", d.read32(0x1e44)?);
    burst(&d, 0xBB, secs);
    Ok(())
}
