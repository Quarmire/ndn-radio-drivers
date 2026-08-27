//! Turn the TX-power knob — the real one this time.
//!
//! Sweeps the **TSSI target power** (`set_tssi_target`, signed, `dBm = q*0.25 + 16.0`) with the
//! TSSI loop enabled, and stamps each frame so `rxpwr_bucket` on the a81a can bucket RSSI by arm.
//!
//! FALSIFIABLE PREDICTION, written before the run: output should be FLAT across the high targets
//! (the loop is commanding more than the PA can deliver, which is why every previous sweep — all of
//! which lived at +16..+31.75 dBm — read as inert), then fall roughly linearly at 0.25 dB per step
//! once the target drops below the achievable maximum. A flat line across ALL arms falsifies this.
use bytes::Bytes;
use ndn_radio_drivers::{BROADCAST, FrameIo, InjectFrame, Rtl8733buBackend, TxIntent};

// dBm targets: +27.25 (the bring-up default, saturated) down to -16.00.
const ARMS: [i8; 10] = [45, 0, -16, -32, -48, -64, -80, -96, -112, -128];
// NDN_8733B_KNOB=de sweeps the TSSI *DE* instead (0.125 dB/step, the vendor's normal-operation
// power write) rather than the MP-mode target byte. Arms span the full signed range = about +-16 dB.
// MEASURED: the DE sign is INVERTED — raising it makes the loop believe it is over-transmitting,
// so it backs the PA off. de=+64 dropped the received level 10 dB while every NEGATIVE arm sat
// flat at the ~17 dBm PA ceiling. So the useful range is POSITIVE. Arm 9 repeats arm 0 as a
// return-to-baseline control: if it does not come back, the trend is drift, not the knob.
const DE_ARMS: [i8; 10] = [0, 16, 32, 48, 64, 80, 96, 112, 127, 0];

#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    let ch: u8 = std::env::args()
        .nth(1)
        .and_then(|s| s.parse().ok())
        .unwrap_or(36);
    let n: u32 = std::env::args()
        .nth(2)
        .and_then(|s| s.parse().ok())
        .unwrap_or(600);
    let dev = Rtl8733buBackend::open()?;
    dev.bring_up_tx(ch)?;
    println!("=== TSSI target sweep, ch{ch}, {n} frames/arm ===");
    let de_mode = std::env::var("NDN_8733B_KNOB").as_deref() == Ok("de");
    // ⚠ The DE is an offset INSIDE the closed loop: with the loop disabled there is nothing for it
    // to actuate. A first DE sweep was run with 0x4318 field=0 and was rightly flat — it tested
    // nothing. Enable the loop before sweeping, and print the field so the arm is self-verifying.
    if de_mode {
        dev.set_tssi_enabled(true)?;
        let t = dev.read32(0x4318)?;
        println!(
            "  TSSI loop enabled for DE sweep: 0x4318=0x{t:08x} field={}",
            (t >> 28) & 0x7
        );
        if (t >> 28) & 0x7 != 7 {
            return Err("TSSI loop did not enable — DE sweep would be meaningless".into());
        }
    }
    let arms: &[i8] = if de_mode { &DE_ARMS } else { &ARMS };
    for (i, &q) in arms.iter().enumerate() {
        if de_mode {
            dev.set_tssi_de(q)?;
        } else {
            dev.set_tssi_target(q)?;
        }
        let t = dev.read32(0x4318)?;
        if de_mode {
            println!(
                "  arm {i}: de={q:4} -> {:+6.2} dB offset   0x4334=0x{:08x}  0x4318=0x{t:08x}",
                q as f32 * 0.125,
                dev.read32(0x4334)?
            );
        } else {
            println!(
                "  arm {i}: q={q:4} -> target {:6.2} dBm   0x4318=0x{t:08x} (tssi_field={})",
                Rtl8733buBackend::tssi_target_dbm(q),
                (t >> 28) & 0x7
            );
        }
        let mut p = vec![0xC3u8; 300];
        p[0] = i as u8; // arm index
        p[1] = if de_mode { 8 } else { 7 }; // 7 = TSSI target, 8 = TSSI DE
        let f = InjectFrame {
            payload: Bytes::from(p),
            tx: TxIntent::CONSERVATIVE,
            dst: BROADCAST,
            src: [0x02, 0x50, 0x33, 0x02, if de_mode { 8 } else { 7 }, i as u8],
            addr3: None,
            addr4: None,
            htc: None,
        };
        tokio::time::sleep(std::time::Duration::from_millis(500)).await;
        for _ in 0..n {
            dev.inject(f.clone()).await?;
        }
    }
    println!("=== SWEEP DONE ===");
    Ok(())
}
