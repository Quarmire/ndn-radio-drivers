//! Which TX-power REGIME is this part in? — the diagnostic the whole TX-power hunt was missing.
//!
//! The vendor driver decides between thermal tracking and the TSSI closed loop from ONE efuse
//! nibble, and then *refuses to write TXAGC at all* in the TSSI regimes:
//!
//! ```c
//! /* halrf_tssi_8733b.c: */
//! odm_efuse_logical_map_read(dm, 1, 0xc8, &pg_tmp);
//! rf->power_track_type = (((pg_tmp >> 4) & 0xf) == 0xf) ? 0 : ((pg_tmp >> 4) & 0xf);
//!
//! /* phydm_hal_api8733b.c, config_phydm_write_txagc_8733b + set_txagc_to_hw: */
//! if (rf->power_track_type >= 4 && rf->power_track_type <= 7)
//!         return false;                   /* <-- TSSI owns the power; TXAGC is NOT written */
//! ```
//!
//! So "the whole TXAGC page is inert" — which this port measured over ~66k frames and 9 candidate
//! registers — is the EXPECTED behaviour of a part whose efuse selects a TSSI regime. It was never
//! evidence of broken silicon or a missing magic register. OpenIPC's devourer reports the mirror
//! image on the RTL8822E: there the efuse mode-select is forced to *thermal* (`0x1e7c[30]`=0), and
//! TXAGC writes are the live knob.
//!
//! This probe reads the deciding byte and the registers each regime actually uses. Run it before
//! any further power work — it says which knob is even connected.
use ndn_radio_drivers::Rtl8733buBackend;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let ch: u8 = std::env::args()
        .nth(1)
        .and_then(|s| s.parse().ok())
        .unwrap_or(36);
    let dev = Rtl8733buBackend::open()?;
    let log = Rtl8733buBackend::decode_efuse_pub(&dev.read_efuse(512)?);

    let pg = *log.get(0xc8).unwrap_or(&0xff);
    let nib = (pg >> 4) & 0xf;
    let track = if nib == 0xf { 0 } else { nib };
    let thermal = *log.get(0xba).unwrap_or(&0xff);
    println!("efuse logical 0xc8 = 0x{pg:02x}  -> power_track_type = {track}");
    println!("efuse logical 0xba = 0x{thermal:02x}  (thermal ref)");
    println!(
        "REGIME: {}",
        match track {
            4..=7 =>
                "TSSI closed loop  => vendor SUPPRESSES all TXAGC writes; power is set by the TSSI DE",
            0 => "thermal tracking  => TXAGC table/ref IS the knob",
            _ => "other/unknown     => cross-check against the vendor tables",
        }
    );

    // Bring the BB up but do NOT run enable_tx / tssi_setup: we want the COLD register state, so
    // that "is the TSSI loop already running before we touch anything?" is answerable.
    dev.bring_up_monitor(ch)?;
    for (name, addr) in [
        ("0x4318 TSSI ctrl", 0x4318u16),
        ("0x4308 txagc ref", 0x4308),
        ("0x4334 DE path-A", 0x4334),
        ("0x4344 DE path-B", 0x4344),
        ("0x3a00 rate tbl", 0x3a00),
        ("0x3a04 ofdm tbl", 0x3a04),
    ] {
        let v = dev.read32(addr)?;
        println!("  {name} = 0x{v:08x}");
    }
    let t = dev.read32(0x4318)?;
    println!(
        "\nTSSI enable field 0x4318[30:28] = {} (7 = loop enabled)",
        (t >> 28) & 0x7
    );
    let de = (dev.read32(0x4334)? >> 20) & 0xff;
    println!(
        "TSSI DE 0x4334[27:20] = 0x{de:02x} ({} as s8) — signed, this is the offset the loop applies",
        de as u8 as i8
    );
    Ok(())
}
