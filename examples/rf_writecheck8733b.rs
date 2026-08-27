//! Do the RTL8733BU's RF-register writes actually land?
//!
//! Six gain controls have now measured flat on air, but the single most diagnostic result is that
//! RF `0x01[4:0] = 0` — which per the driver's own note leaves the PA with zero input so nothing
//! radiates — transmitted at full strength. There are only two explanations: the write never
//! reaches the RF register, or that register is not the gain. This settles it with no RF involved,
//! by reading the value back through `rf_read`.
//!
//! Also dumps RF 0x00 (mode/enable) and 0x18 (channel/bandwidth) as a control: 0x18 is written by
//! every `tune_channel` and channel switching demonstrably works, so if 0x18 reads back correctly
//! while 0x01 does not, the fault is specific to that register rather than to the LSSI path.
//!
//! Usage: sudo ./rf_writecheck8733b [channel]

use ndn_radio_drivers::Rtl8733buBackend;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let ch: u8 = std::env::args()
        .nth(1)
        .and_then(|s| s.parse().ok())
        .unwrap_or(36);
    let dev = Rtl8733buBackend::open()?;
    dev.bring_up_tx(ch)?;

    println!(
        "baseline: RF0x00={:05x} RF0x01={:05x} RF0x18={:05x}",
        dev.rf_read(0x00).unwrap_or(0),
        dev.rf_read(0x01).unwrap_or(0),
        dev.rf_read(0x18).unwrap_or(0)
    );

    for v in [0x00u8, 0x1f, 0x0a, 0x1a] {
        dev.set_rf_txagc(v)?;
        let rb = dev.rf_read(0x01).unwrap_or(0);
        println!(
            "  set_rf_txagc(0x{v:02x}) -> RF0x01={rb:05x} field[4:0]={:02x}  {}",
            rb & 0x1f,
            if (rb & 0x1f) as u8 == (v & 0x1f) {
                "MATCHES"
            } else {
                "*** MISMATCH — write did not land ***"
            }
        );
    }
    // Leave it where the vendor rests it rather than at whatever the loop above ended on.
    dev.set_rf_txagc(0x1a)?;
    println!("restored RF0x01 -> {:05x}", dev.rf_read(0x01).unwrap_or(0));
    Ok(())
}
