//! MT7610U (`mt76x0`) **RF** initialisation tables, transcribed from the mainline
//! Linux mt76 driver: `drivers/net/wireless/mediatek/mt76/mt76x0/initvals_phy.h`
//! (the read-only reference tree under `scratchpad/mt76-src/`).
//!
//! Everything here is **CODE-READ** from that header — none of it is measured.
//! The MT7610U RF (bank/register CSR space, reached through
//! [`PhyBus::rf_wr`](super::phy::PhyBus) or, on USB, the MCU `wr_rp` register-pair
//! path) has no public documentation whatsoever: these tables and the per-row
//! `/* ... */` comments Ralink left in them are the only description of the part
//! that exists. They are therefore reproduced **verbatim**, comments included,
//! rather than being folded into anything more structured.
//!
//! ## What the transcription changes, and what it does not
//! * `MT_RF(bank, reg)` (`mt76x0/phy.h:21`) is `(bank << 16) | reg`. Rust has no
//!   macro in a `const` table position that reads as well as a literal, so each
//!   row carries the **resolved** address with the symbolic form in a trailing
//!   comment — `(0x0005_0002, 0x0C), // MT_RF(5, 2)`. The comment is the thing to
//!   diff against upstream.
//! * Value bytes keep upstream's hex casing (`0x0C`, `0xDD`) so a row is a
//!   character-for-character match against the header; the resolved addresses are
//!   lower-case, per Rust convention.
//! * The two frequency plans (`mt76x0_frequency_plan`, `mt76x0_sdm_frequency_plan`)
//!   and `mt76x0_sdm_channel` live in [`super::freq_plan`], not here.
//!
//! ## Who consumes each table
//! Upstream line numbers are from `mt76x0/phy.c`.
//! * [`RF_CENTRAL_TAB`] + [`RF_2G_CHANNEL_0_TAB`] + [`RF_5G_CHANNEL_0_TAB`] +
//!   [`RF_VGA_CHANNEL_0_TAB`] — one-shot RF bring-up in `mt76x0_phy_rf_init`
//!   (`phy.c:1157`). The first two go through `mt76x0_rf_patch_reg_array`
//!   (`phy.c:1114`), which overrides three rows per chip variant; for the **USB
//!   MT7610U** all three overrides are no-ops against the table values
//!   (`MT_RF(0, 3)`→`0x73`, `MT_RF(0, 21)`→`0x12`, `MT_RF(5, 2)`→`0x0c`), so the
//!   tables can be written as-is on this part. Do not assume that for MT7610E /
//!   MT7630, which take different values there.
//! * [`RF_BW_SWITCH_TAB`] and [`RF_BAND_SWITCH_TAB`] — replayed twice: filtered to
//!   `RF_BW_20` / `RF_G_BAND` at init (`phy.c:1168`, `phy.c:1178`), then filtered
//!   to the live bandwidth/band on every channel change in
//!   `mt76x0_phy_set_chan_rf_params` (`phy.c:338`, `phy.c:351`).
//! * [`RF_EXT_PA_TAB`] — applied on a channel change **only** when the EEPROM says
//!   this board has an external PA for the band (`mt76x02_ext_pa_enabled`,
//!   `phy.c:375`). Every row is 5 GHz (`RF_A_BAND_*`); a 2.4 GHz external-PA board
//!   gets only the `MT_RF_MISC` bit, no RF rows.
//!
//! ## The bw/band match rules are not the same in each table
//! Worth stating because the flags overlap and the three loops differ:
//! * bw-switch at channel time (`phy.c:338`) matches a row when the row's
//!   `bw_band` equals the requested bandwidth exactly (the `RF_BW_*`-only rows for
//!   `MT_RF(7, 76)` / `MT_RF(7, 77)`), **or** when the low byte equals the
//!   bandwidth *and* the high byte intersects the band.
//! * band-switch (`phy.c:351`) and ext-PA (`phy.c:375`) match on any band-bit
//!   intersection alone.
//! * bw-switch at *init* (`phy.c:1168`) uses a third rule: `bw_band == RF_BW_20`,
//!   or `bw_band` contains both `RF_G_BAND` and `RF_BW_20`.
//!
//! Address validity: `mt76x0_rf_csr_wr` (`phy.c:32`) rejects `bank > 8` or
//! `reg > 127`, with `MT_RF_BANK(o) = o >> 16` and `MT_RF_REG(o) = o & 0xff`
//! (`mt76x0/phy.h:22`). The tests below hold every row in this file to that bound.

/// One conditional RF register write: apply `value` to `rf_bank_reg` when the
/// caller's bandwidth/band selector matches `bw_band`.
///
/// Mirror of `struct mt76x0_rf_switch_item` (`mt76x0/phy.h:47`), field names
/// identical. `bw_band` packs two independent things: the **low** byte is a
/// `RF_BW_*` bandwidth mask and the **high** byte a `RF_*_BAND*` band mask, and a
/// row may carry either or both — which is exactly why the three consumers listed
/// in the module docs can filter the same table three different ways.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct RfSwitch {
    /// `MT_RF(bank, reg)` = `(bank << 16) | reg`.
    pub rf_bank_reg: u32,
    /// `RF_BW_*` in the low byte, `RF_*_BAND*` in the high byte.
    pub bw_band: u16,
    /// The byte written to the RF CSR.
    pub value: u8,
}

// ── bw/band selector bits (`mt76x0/phy.h:9-19`) ──────────────────────────────
// High byte: band. Note RF_A_BAND and the RF_A_BAND_{LB,MB,HB,11J} sub-bands are
// *separate* bits, not a hierarchy — a caller on 5 GHz sets RF_A_BAND plus the one
// sub-band bit its channel falls in, so a table row tagged plain RF_A_BAND and a
// row tagged RF_A_BAND_MB can both match the same channel.

/// 2.4 GHz.
pub const RF_G_BAND: u16 = 0x0100;
/// 5 GHz, any sub-band.
pub const RF_A_BAND: u16 = 0x0200;
/// 5 GHz low sub-band.
pub const RF_A_BAND_LB: u16 = 0x0400;
/// 5 GHz middle sub-band.
pub const RF_A_BAND_MB: u16 = 0x0800;
/// 5 GHz high sub-band.
pub const RF_A_BAND_HB: u16 = 0x1000;
/// Japan 4.9 GHz (802.11j).
pub const RF_A_BAND_11J: u16 = 0x2000;

// Low byte: bandwidth.

/// 20 MHz.
pub const RF_BW_20: u16 = 1;
/// 40 MHz.
pub const RF_BW_40: u16 = 2;
/// 10 MHz (half-rate; never selected by this port).
pub const RF_BW_10: u16 = 4;
/// 80 MHz.
pub const RF_BW_80: u16 = 8;

/// Bank 0 — the shared/central RF block (VCO, band-gap, LDO, PLL, LO buffer, test
/// ports, ADC-DAC). Written first in `mt76x0_phy_rf_init` (`phy.c:1161`), via
/// `mt76x0_rf_patch_reg_array`; on USB MT7610U the patcher leaves every value in
/// this table unchanged (see the module docs).
///
/// **44 rows.** `mt76x0_rf_central_tab`, `mt76x0/initvals_phy.h:11`.
pub const RF_CENTRAL_TAB: &[(u32, u32)] = &[
    (0x0000_0001, 0x01), // MT_RF(0, 1)
    (0x0000_0002, 0x11), // MT_RF(0, 2)
    // R3 ~ R7: VCO Cal
    (0x0000_0003, 0x73), // MT_RF(0, 3) — VCO Freq Cal
    (0x0000_0004, 0x30), // MT_RF(0, 4) — R4 b<7>=1, VCO cal
    (0x0000_0005, 0x00), // MT_RF(0, 5)
    (0x0000_0006, 0x41), // MT_RF(0, 6)
    (0x0000_0007, 0x00), // MT_RF(0, 7)
    (0x0000_0008, 0x00), // MT_RF(0, 8)
    (0x0000_0009, 0x00), // MT_RF(0, 9)
    (0x0000_000a, 0x0C), // MT_RF(0, 10)
    (0x0000_000b, 0x00), // MT_RF(0, 11)
    (0x0000_000c, 0x00), // MT_RF(0, 12)
    // BG
    (0x0000_000d, 0x00), // MT_RF(0, 13)
    (0x0000_000e, 0x00), // MT_RF(0, 14)
    (0x0000_000f, 0x00), // MT_RF(0, 15)
    // LDO
    (0x0000_0013, 0x20), // MT_RF(0, 19)
    (0x0000_0014, 0x22), // MT_RF(0, 20)
    (0x0000_0015, 0x12), // MT_RF(0, 21)
    (0x0000_0017, 0x00), // MT_RF(0, 23)
    (0x0000_0018, 0x33), // MT_RF(0, 24)
    (0x0000_0019, 0x00), // MT_RF(0, 25)
    // PLL
    (0x0000_001a, 0x00), // MT_RF(0, 26)
    (0x0000_001b, 0x00), // MT_RF(0, 27)
    (0x0000_001c, 0x00), // MT_RF(0, 28)
    (0x0000_001d, 0x00), // MT_RF(0, 29)
    (0x0000_001e, 0x00), // MT_RF(0, 30)
    (0x0000_001f, 0x00), // MT_RF(0, 31)
    (0x0000_0020, 0x00), // MT_RF(0, 32)
    (0x0000_0021, 0x00), // MT_RF(0, 33)
    (0x0000_0022, 0x00), // MT_RF(0, 34)
    (0x0000_0023, 0x00), // MT_RF(0, 35)
    (0x0000_0024, 0x00), // MT_RF(0, 36)
    (0x0000_0025, 0x00), // MT_RF(0, 37)
    // LO Buffer
    (0x0000_0026, 0x2F), // MT_RF(0, 38)
    // Test Ports
    (0x0000_0040, 0x00), // MT_RF(0, 64)
    (0x0000_0041, 0x80), // MT_RF(0, 65)
    (0x0000_0042, 0x01), // MT_RF(0, 66)
    (0x0000_0043, 0x04), // MT_RF(0, 67)
    // ADC-DAC
    (0x0000_0044, 0x00), // MT_RF(0, 68)
    (0x0000_0045, 0x08), // MT_RF(0, 69)
    (0x0000_0046, 0x08), // MT_RF(0, 70)
    (0x0000_0047, 0x40), // MT_RF(0, 71)
    (0x0000_0048, 0xD0), // MT_RF(0, 72)
    (0x0000_0049, 0x93), // MT_RF(0, 73)
];

/// Bank 5 — the 2.4 GHz RF chain (RX/TX logic, LDO, RX, LOGEN, TX). Written at
/// init (`phy.c:1163`, through the patcher) and re-written whole whenever the band
/// is switched to 2.4 GHz in `mt76x0_phy_set_band` (`phy.c:210`), which then adds
/// `MT_RF(5, 0) = 0x45` / `MT_RF(6, 0) = 0x44` to enable bank 5 and idle bank 6.
///
/// **68 rows.** `mt76x0_rf_2g_channel_0_tab`, `mt76x0/initvals_phy.h:65`.
pub const RF_2G_CHANNEL_0_TAB: &[(u32, u32)] = &[
    // RX logic operation
    (0x0005_0002, 0x0C), // MT_RF(5, 2) — 5G+2G
    (0x0005_0003, 0x00), // MT_RF(5, 3)
    // TX logic operation
    (0x0005_0004, 0x00), // MT_RF(5, 4)
    (0x0005_0005, 0x84), // MT_RF(5, 5)
    (0x0005_0006, 0x02), // MT_RF(5, 6)
    // LDO
    (0x0005_0007, 0x00), // MT_RF(5, 7)
    (0x0005_0008, 0x00), // MT_RF(5, 8)
    (0x0005_0009, 0x00), // MT_RF(5, 9)
    // RX
    (0x0005_000a, 0x51), // MT_RF(5, 10)
    (0x0005_000b, 0x22), // MT_RF(5, 11)
    (0x0005_000c, 0x22), // MT_RF(5, 12)
    (0x0005_000d, 0x0F), // MT_RF(5, 13)
    (0x0005_000e, 0x47), // MT_RF(5, 14)
    (0x0005_000f, 0x25), // MT_RF(5, 15)
    (0x0005_0010, 0xC7), // MT_RF(5, 16)
    (0x0005_0011, 0x00), // MT_RF(5, 17)
    (0x0005_0012, 0x00), // MT_RF(5, 18)
    (0x0005_0013, 0x30), // MT_RF(5, 19)
    (0x0005_0014, 0x33), // MT_RF(5, 20)
    (0x0005_0015, 0x02), // MT_RF(5, 21)
    (0x0005_0016, 0x32), // MT_RF(5, 22)
    (0x0005_0017, 0x00), // MT_RF(5, 23)
    (0x0005_0018, 0x25), // MT_RF(5, 24)
    (0x0005_001a, 0x00), // MT_RF(5, 26)
    (0x0005_001b, 0x12), // MT_RF(5, 27)
    (0x0005_001c, 0x0F), // MT_RF(5, 28)
    (0x0005_001d, 0x00), // MT_RF(5, 29)
    // LOGEN
    (0x0005_001e, 0x51), // MT_RF(5, 30)
    (0x0005_001f, 0x35), // MT_RF(5, 31)
    (0x0005_0020, 0x31), // MT_RF(5, 32)
    (0x0005_0021, 0x31), // MT_RF(5, 33)
    (0x0005_0022, 0x34), // MT_RF(5, 34)
    (0x0005_0023, 0x03), // MT_RF(5, 35)
    (0x0005_0024, 0x00), // MT_RF(5, 36)
    // TX
    (0x0005_0025, 0xDD), // MT_RF(5, 37)
    (0x0005_0026, 0xB3), // MT_RF(5, 38)
    (0x0005_0027, 0x33), // MT_RF(5, 39)
    (0x0005_0028, 0xB1), // MT_RF(5, 40)
    (0x0005_0029, 0x71), // MT_RF(5, 41)
    (0x0005_002a, 0xF2), // MT_RF(5, 42)
    (0x0005_002b, 0x47), // MT_RF(5, 43)
    (0x0005_002c, 0x77), // MT_RF(5, 44)
    (0x0005_002d, 0x0E), // MT_RF(5, 45)
    (0x0005_002e, 0x10), // MT_RF(5, 46)
    (0x0005_002f, 0x00), // MT_RF(5, 47)
    (0x0005_0030, 0x53), // MT_RF(5, 48)
    (0x0005_0031, 0x03), // MT_RF(5, 49)
    (0x0005_0032, 0xEF), // MT_RF(5, 50)
    (0x0005_0033, 0xC7), // MT_RF(5, 51)
    (0x0005_0034, 0x62), // MT_RF(5, 52)
    (0x0005_0035, 0x62), // MT_RF(5, 53)
    (0x0005_0036, 0x00), // MT_RF(5, 54)
    (0x0005_0037, 0x00), // MT_RF(5, 55)
    (0x0005_0038, 0x0F), // MT_RF(5, 56)
    (0x0005_0039, 0x0F), // MT_RF(5, 57)
    (0x0005_003a, 0x16), // MT_RF(5, 58)
    (0x0005_003b, 0x16), // MT_RF(5, 59)
    (0x0005_003c, 0x10), // MT_RF(5, 60)
    (0x0005_003d, 0x10), // MT_RF(5, 61)
    (0x0005_003e, 0xD0), // MT_RF(5, 62)
    (0x0005_003f, 0x6C), // MT_RF(5, 63)
    (0x0005_0040, 0x58), // MT_RF(5, 64)
    (0x0005_0041, 0x58), // MT_RF(5, 65)
    (0x0005_0042, 0xF2), // MT_RF(5, 66)
    (0x0005_0043, 0xE8), // MT_RF(5, 67)
    (0x0005_0044, 0xF0), // MT_RF(5, 68)
    (0x0005_0045, 0xF0), // MT_RF(5, 69)
    (0x0005_007f, 0x04), // MT_RF(5, 127)
];

/// Bank 6 — the 5 GHz RF chain. Written at init (`phy.c:1165`) and re-written
/// whole on a switch to 5 GHz in `mt76x0_phy_set_band` (`phy.c:219`), which then
/// adds `MT_RF(5, 0) = 0x44` / `MT_RF(6, 0) = 0x45` — the mirror of the 2 GHz case.
///
/// Sparser than the 2.4 GHz table: several bank-6 registers are deliberately left
/// alone here because [`RF_BAND_SWITCH_TAB`] and [`RF_EXT_PA_TAB`] set them
/// per-sub-band (`MT_RF(6, 12/17/24/39/42/45/50..59/127)`).
///
/// **38 rows.** `mt76x0_rf_5g_channel_0_tab`, `mt76x0/initvals_phy.h:142`.
pub const RF_5G_CHANNEL_0_TAB: &[(u32, u32)] = &[
    // RX logic operation
    (0x0006_0002, 0x0C), // MT_RF(6, 2)
    (0x0006_0003, 0x00), // MT_RF(6, 3)
    // TX logic operation
    (0x0006_0004, 0x00), // MT_RF(6, 4)
    (0x0006_0005, 0x84), // MT_RF(6, 5)
    (0x0006_0006, 0x02), // MT_RF(6, 6)
    // LDO
    (0x0006_0007, 0x00), // MT_RF(6, 7)
    (0x0006_0008, 0x00), // MT_RF(6, 8)
    (0x0006_0009, 0x00), // MT_RF(6, 9)
    // RX
    (0x0006_000a, 0x00), // MT_RF(6, 10)
    (0x0006_000b, 0x01), // MT_RF(6, 11)
    (0x0006_000d, 0x23), // MT_RF(6, 13)
    (0x0006_000e, 0x00), // MT_RF(6, 14)
    (0x0006_000f, 0x04), // MT_RF(6, 15)
    (0x0006_0010, 0x22), // MT_RF(6, 16)
    (0x0006_0012, 0x08), // MT_RF(6, 18)
    (0x0006_0013, 0x00), // MT_RF(6, 19)
    (0x0006_0014, 0x00), // MT_RF(6, 20)
    (0x0006_0015, 0x00), // MT_RF(6, 21)
    (0x0006_0016, 0xFB), // MT_RF(6, 22)
    // LOGEN5G
    (0x0006_0019, 0x76), // MT_RF(6, 25)
    (0x0006_001a, 0x24), // MT_RF(6, 26)
    (0x0006_001b, 0x04), // MT_RF(6, 27)
    (0x0006_001c, 0x00), // MT_RF(6, 28)
    (0x0006_001d, 0x00), // MT_RF(6, 29)
    // TX
    (0x0006_0025, 0xBB), // MT_RF(6, 37)
    (0x0006_0026, 0xB3), // MT_RF(6, 38)
    (0x0006_0028, 0x33), // MT_RF(6, 40)
    (0x0006_0029, 0x33), // MT_RF(6, 41)
    (0x0006_002b, 0x03), // MT_RF(6, 43)
    (0x0006_002c, 0xB3), // MT_RF(6, 44)
    (0x0006_002e, 0x17), // MT_RF(6, 46)
    (0x0006_002f, 0x0E), // MT_RF(6, 47)
    (0x0006_0030, 0x10), // MT_RF(6, 48)
    (0x0006_0031, 0x07), // MT_RF(6, 49)
    (0x0006_003e, 0x00), // MT_RF(6, 62)
    (0x0006_003f, 0x00), // MT_RF(6, 63)
    (0x0006_0040, 0xF1), // MT_RF(6, 64)
    (0x0006_0041, 0x0F), // MT_RF(6, 65)
];

/// Bank 7 — RX VGA / gain-control block, band-independent. Written once at init
/// (`phy.c:1166`). Upstream's `/* E3 CR */` marks these as the E3 chip revision's
/// values; no other revision's table survives in the driver.
///
/// **35 rows.** `mt76x0_rf_vga_channel_0_tab`, `mt76x0/initvals_phy.h:189`.
pub const RF_VGA_CHANNEL_0_TAB: &[(u32, u32)] = &[
    // E3 CR
    (0x0007_0000, 0x47), // MT_RF(7, 0)
    (0x0007_0001, 0x00), // MT_RF(7, 1)
    (0x0007_0002, 0x00), // MT_RF(7, 2)
    (0x0007_0003, 0x00), // MT_RF(7, 3)
    (0x0007_0004, 0x00), // MT_RF(7, 4)
    (0x0007_000a, 0x13), // MT_RF(7, 10)
    (0x0007_000b, 0x0F), // MT_RF(7, 11)
    (0x0007_000c, 0x13), // MT_RF(7, 12)
    (0x0007_000d, 0x13), // MT_RF(7, 13)
    (0x0007_000e, 0x13), // MT_RF(7, 14)
    (0x0007_000f, 0x20), // MT_RF(7, 15)
    (0x0007_0010, 0x22), // MT_RF(7, 16)
    (0x0007_0011, 0x7C), // MT_RF(7, 17)
    (0x0007_0012, 0x00), // MT_RF(7, 18)
    (0x0007_0013, 0x00), // MT_RF(7, 19)
    (0x0007_0014, 0x00), // MT_RF(7, 20)
    (0x0007_0015, 0xF1), // MT_RF(7, 21)
    (0x0007_0016, 0x11), // MT_RF(7, 22)
    (0x0007_0017, 0xC2), // MT_RF(7, 23)
    (0x0007_0018, 0x41), // MT_RF(7, 24)
    (0x0007_0019, 0x20), // MT_RF(7, 25)
    (0x0007_001a, 0x40), // MT_RF(7, 26)
    (0x0007_001b, 0xD7), // MT_RF(7, 27)
    (0x0007_001c, 0xA2), // MT_RF(7, 28)
    (0x0007_001d, 0x60), // MT_RF(7, 29)
    (0x0007_001e, 0x49), // MT_RF(7, 30)
    (0x0007_001f, 0x20), // MT_RF(7, 31)
    (0x0007_0020, 0x44), // MT_RF(7, 32)
    (0x0007_0021, 0xC1), // MT_RF(7, 33)
    (0x0007_0022, 0x60), // MT_RF(7, 34)
    (0x0007_0023, 0xC0), // MT_RF(7, 35)
    (0x0007_003d, 0x01), // MT_RF(7, 61)
    (0x0007_0048, 0x3C), // MT_RF(7, 72)
    (0x0007_0049, 0x34), // MT_RF(7, 73)
    (0x0007_004a, 0x00), // MT_RF(7, 74)
];

/// Bandwidth-dependent RF writes — filter analogue bandwidth (`MT_RF(7, 6/7/8)`),
/// LO/PLL loop settings (`MT_RF(7, 58/59/60)`) and the `MT_RF(7, 76/77)` pair.
///
/// The last six rows carry **no band bit at all** (`RF_BW_20`/`40`/`80` alone) and
/// so are matched by the exact-equality arm of `phy.c:339`, unlike every other row.
/// Note there is no `RF_BW_80` row for `RF_G_BAND`, and no `RF_BW_10` row anywhere.
///
/// **41 rows.** `mt76x0_rf_bw_switch_tab`, `mt76x0/initvals_phy.h:228`.
/// Columns upstream: `bank, reg | bw/band | value`.
// `rustfmt::skip`: left one Rust row per upstream row on purpose. Without it
// rustfmt breaks each `RfSwitch` across five lines, and the file stops being
// diffable against `initvals_phy.h` — which is the only reason to trust it.
#[rustfmt::skip]
pub const RF_BW_SWITCH_TAB: &[RfSwitch] = &[
    RfSwitch { rf_bank_reg: 0x0000_0011, bw_band: RF_G_BAND | RF_BW_20, value: 0x00 }, // MT_RF(0, 17)
    RfSwitch { rf_bank_reg: 0x0000_0011, bw_band: RF_G_BAND | RF_BW_40, value: 0x00 }, // MT_RF(0, 17)
    RfSwitch { rf_bank_reg: 0x0000_0011, bw_band: RF_A_BAND | RF_BW_20, value: 0x00 }, // MT_RF(0, 17)
    RfSwitch { rf_bank_reg: 0x0000_0011, bw_band: RF_A_BAND | RF_BW_40, value: 0x00 }, // MT_RF(0, 17)
    RfSwitch { rf_bank_reg: 0x0000_0011, bw_band: RF_A_BAND | RF_BW_80, value: 0x00 }, // MT_RF(0, 17)
    RfSwitch { rf_bank_reg: 0x0007_0006, bw_band: RF_G_BAND | RF_BW_20, value: 0x40 }, // MT_RF(7, 6)
    RfSwitch { rf_bank_reg: 0x0007_0006, bw_band: RF_G_BAND | RF_BW_40, value: 0x1C }, // MT_RF(7, 6)
    RfSwitch { rf_bank_reg: 0x0007_0006, bw_band: RF_A_BAND | RF_BW_20, value: 0x40 }, // MT_RF(7, 6)
    RfSwitch { rf_bank_reg: 0x0007_0006, bw_band: RF_A_BAND | RF_BW_40, value: 0x20 }, // MT_RF(7, 6)
    RfSwitch { rf_bank_reg: 0x0007_0006, bw_band: RF_A_BAND | RF_BW_80, value: 0x10 }, // MT_RF(7, 6)
    RfSwitch { rf_bank_reg: 0x0007_0007, bw_band: RF_G_BAND | RF_BW_20, value: 0x40 }, // MT_RF(7, 7)
    RfSwitch { rf_bank_reg: 0x0007_0007, bw_band: RF_G_BAND | RF_BW_40, value: 0x20 }, // MT_RF(7, 7)
    RfSwitch { rf_bank_reg: 0x0007_0007, bw_band: RF_A_BAND | RF_BW_20, value: 0x40 }, // MT_RF(7, 7)
    RfSwitch { rf_bank_reg: 0x0007_0007, bw_band: RF_A_BAND | RF_BW_40, value: 0x20 }, // MT_RF(7, 7)
    RfSwitch { rf_bank_reg: 0x0007_0007, bw_band: RF_A_BAND | RF_BW_80, value: 0x10 }, // MT_RF(7, 7)
    RfSwitch { rf_bank_reg: 0x0007_0008, bw_band: RF_G_BAND | RF_BW_20, value: 0x03 }, // MT_RF(7, 8)
    RfSwitch { rf_bank_reg: 0x0007_0008, bw_band: RF_G_BAND | RF_BW_40, value: 0x01 }, // MT_RF(7, 8)
    RfSwitch { rf_bank_reg: 0x0007_0008, bw_band: RF_A_BAND | RF_BW_20, value: 0x03 }, // MT_RF(7, 8)
    RfSwitch { rf_bank_reg: 0x0007_0008, bw_band: RF_A_BAND | RF_BW_40, value: 0x01 }, // MT_RF(7, 8)
    RfSwitch { rf_bank_reg: 0x0007_0008, bw_band: RF_A_BAND | RF_BW_80, value: 0x00 }, // MT_RF(7, 8)
    RfSwitch { rf_bank_reg: 0x0007_003a, bw_band: RF_G_BAND | RF_BW_20, value: 0x40 }, // MT_RF(7, 58)
    RfSwitch { rf_bank_reg: 0x0007_003a, bw_band: RF_G_BAND | RF_BW_40, value: 0x40 }, // MT_RF(7, 58)
    RfSwitch { rf_bank_reg: 0x0007_003a, bw_band: RF_A_BAND | RF_BW_20, value: 0x40 }, // MT_RF(7, 58)
    RfSwitch { rf_bank_reg: 0x0007_003a, bw_band: RF_A_BAND | RF_BW_40, value: 0x40 }, // MT_RF(7, 58)
    RfSwitch { rf_bank_reg: 0x0007_003a, bw_band: RF_A_BAND | RF_BW_80, value: 0x10 }, // MT_RF(7, 58)
    RfSwitch { rf_bank_reg: 0x0007_003b, bw_band: RF_G_BAND | RF_BW_20, value: 0x40 }, // MT_RF(7, 59)
    RfSwitch { rf_bank_reg: 0x0007_003b, bw_band: RF_G_BAND | RF_BW_40, value: 0x40 }, // MT_RF(7, 59)
    RfSwitch { rf_bank_reg: 0x0007_003b, bw_band: RF_A_BAND | RF_BW_20, value: 0x40 }, // MT_RF(7, 59)
    RfSwitch { rf_bank_reg: 0x0007_003b, bw_band: RF_A_BAND | RF_BW_40, value: 0x40 }, // MT_RF(7, 59)
    RfSwitch { rf_bank_reg: 0x0007_003b, bw_band: RF_A_BAND | RF_BW_80, value: 0x10 }, // MT_RF(7, 59)
    RfSwitch { rf_bank_reg: 0x0007_003c, bw_band: RF_G_BAND | RF_BW_20, value: 0xAA }, // MT_RF(7, 60)
    RfSwitch { rf_bank_reg: 0x0007_003c, bw_band: RF_G_BAND | RF_BW_40, value: 0xAA }, // MT_RF(7, 60)
    RfSwitch { rf_bank_reg: 0x0007_003c, bw_band: RF_A_BAND | RF_BW_20, value: 0xAA }, // MT_RF(7, 60)
    RfSwitch { rf_bank_reg: 0x0007_003c, bw_band: RF_A_BAND | RF_BW_40, value: 0xAA }, // MT_RF(7, 60)
    RfSwitch { rf_bank_reg: 0x0007_003c, bw_band: RF_A_BAND | RF_BW_80, value: 0xAA }, // MT_RF(7, 60)
    RfSwitch { rf_bank_reg: 0x0007_004c, bw_band: RF_BW_20, value: 0x40 }, // MT_RF(7, 76)
    RfSwitch { rf_bank_reg: 0x0007_004c, bw_band: RF_BW_40, value: 0x40 }, // MT_RF(7, 76)
    RfSwitch { rf_bank_reg: 0x0007_004c, bw_band: RF_BW_80, value: 0x10 }, // MT_RF(7, 76)
    RfSwitch { rf_bank_reg: 0x0007_004d, bw_band: RF_BW_20, value: 0x40 }, // MT_RF(7, 77)
    RfSwitch { rf_bank_reg: 0x0007_004d, bw_band: RF_BW_40, value: 0x40 }, // MT_RF(7, 77)
    RfSwitch { rf_bank_reg: 0x0007_004d, bw_band: RF_BW_80, value: 0x10 }, // MT_RF(7, 77)
];

/// Band-dependent RF writes, applied on every channel change (`phy.c:351`) and,
/// filtered to `RF_G_BAND`, once at init (`phy.c:1178`).
///
/// Two shapes coexist here: coarse `RF_G_BAND`/`RF_A_BAND` rows that pick the
/// active chain (`MT_RF(0, 16/18)`, `MT_RF(6, 127)`, `MT_RF(7, 5/9/70/71/78/79)`)
/// and fine `RF_A_BAND_{LB,MB,HB,11J}` rows that trim the 5 GHz chain per
/// sub-band. Because a 5 GHz caller sets `RF_A_BAND` *and* its sub-band bit, both
/// shapes fire — and the ordering in this table is therefore load-bearing.
///
/// **43 rows.** `mt76x0_rf_band_switch_tab`, `mt76x0/initvals_phy.h:273`.
/// Columns upstream: `bank, reg | bw/band | value`.
// `rustfmt::skip`: left one Rust row per upstream row on purpose. Without it
// rustfmt breaks each `RfSwitch` across five lines, and the file stops being
// diffable against `initvals_phy.h` — which is the only reason to trust it.
#[rustfmt::skip]
pub const RF_BAND_SWITCH_TAB: &[RfSwitch] = &[
    RfSwitch { rf_bank_reg: 0x0000_0010, bw_band: RF_G_BAND, value: 0x20 }, // MT_RF(0, 16)
    RfSwitch { rf_bank_reg: 0x0000_0010, bw_band: RF_A_BAND, value: 0x20 }, // MT_RF(0, 16)
    RfSwitch { rf_bank_reg: 0x0000_0012, bw_band: RF_G_BAND, value: 0x00 }, // MT_RF(0, 18)
    RfSwitch { rf_bank_reg: 0x0000_0012, bw_band: RF_A_BAND, value: 0x00 }, // MT_RF(0, 18)
    RfSwitch { rf_bank_reg: 0x0000_0027, bw_band: RF_G_BAND, value: 0x36 }, // MT_RF(0, 39)
    RfSwitch { rf_bank_reg: 0x0000_0027, bw_band: RF_A_BAND_LB, value: 0x34 }, // MT_RF(0, 39)
    RfSwitch { rf_bank_reg: 0x0000_0027, bw_band: RF_A_BAND_MB, value: 0x33 }, // MT_RF(0, 39)
    RfSwitch { rf_bank_reg: 0x0000_0027, bw_band: RF_A_BAND_HB, value: 0x31 }, // MT_RF(0, 39)
    RfSwitch { rf_bank_reg: 0x0000_0027, bw_band: RF_A_BAND_11J, value: 0x36 }, // MT_RF(0, 39)
    RfSwitch { rf_bank_reg: 0x0006_000c, bw_band: RF_A_BAND_LB, value: 0x44 }, // MT_RF(6, 12)
    RfSwitch { rf_bank_reg: 0x0006_000c, bw_band: RF_A_BAND_MB, value: 0x44 }, // MT_RF(6, 12)
    RfSwitch { rf_bank_reg: 0x0006_000c, bw_band: RF_A_BAND_HB, value: 0x55 }, // MT_RF(6, 12)
    RfSwitch { rf_bank_reg: 0x0006_000c, bw_band: RF_A_BAND_11J, value: 0x44 }, // MT_RF(6, 12)
    RfSwitch { rf_bank_reg: 0x0006_0011, bw_band: RF_A_BAND_LB, value: 0x02 }, // MT_RF(6, 17)
    RfSwitch { rf_bank_reg: 0x0006_0011, bw_band: RF_A_BAND_MB, value: 0x00 }, // MT_RF(6, 17)
    RfSwitch { rf_bank_reg: 0x0006_0011, bw_band: RF_A_BAND_HB, value: 0x00 }, // MT_RF(6, 17)
    RfSwitch { rf_bank_reg: 0x0006_0011, bw_band: RF_A_BAND_11J, value: 0x05 }, // MT_RF(6, 17)
    RfSwitch { rf_bank_reg: 0x0006_0018, bw_band: RF_A_BAND_LB, value: 0xA1 }, // MT_RF(6, 24)
    RfSwitch { rf_bank_reg: 0x0006_0018, bw_band: RF_A_BAND_MB, value: 0x41 }, // MT_RF(6, 24)
    RfSwitch { rf_bank_reg: 0x0006_0018, bw_band: RF_A_BAND_HB, value: 0x21 }, // MT_RF(6, 24)
    RfSwitch { rf_bank_reg: 0x0006_0018, bw_band: RF_A_BAND_11J, value: 0xE1 }, // MT_RF(6, 24)
    RfSwitch { rf_bank_reg: 0x0006_0027, bw_band: RF_A_BAND_LB, value: 0x36 }, // MT_RF(6, 39)
    RfSwitch { rf_bank_reg: 0x0006_0027, bw_band: RF_A_BAND_MB, value: 0x34 }, // MT_RF(6, 39)
    RfSwitch { rf_bank_reg: 0x0006_0027, bw_band: RF_A_BAND_HB, value: 0x32 }, // MT_RF(6, 39)
    RfSwitch { rf_bank_reg: 0x0006_0027, bw_band: RF_A_BAND_11J, value: 0x37 }, // MT_RF(6, 39)
    RfSwitch { rf_bank_reg: 0x0006_002a, bw_band: RF_A_BAND_LB, value: 0xFB }, // MT_RF(6, 42)
    RfSwitch { rf_bank_reg: 0x0006_002a, bw_band: RF_A_BAND_MB, value: 0xF3 }, // MT_RF(6, 42)
    RfSwitch { rf_bank_reg: 0x0006_002a, bw_band: RF_A_BAND_HB, value: 0xEB }, // MT_RF(6, 42)
    RfSwitch { rf_bank_reg: 0x0006_002a, bw_band: RF_A_BAND_11J, value: 0xEB }, // MT_RF(6, 42)
    RfSwitch { rf_bank_reg: 0x0006_007f, bw_band: RF_G_BAND, value: 0x84 }, // MT_RF(6, 127)
    RfSwitch { rf_bank_reg: 0x0006_007f, bw_band: RF_A_BAND, value: 0x04 }, // MT_RF(6, 127)
    RfSwitch { rf_bank_reg: 0x0007_0005, bw_band: RF_G_BAND, value: 0x40 }, // MT_RF(7, 5)
    RfSwitch { rf_bank_reg: 0x0007_0005, bw_band: RF_A_BAND, value: 0x00 }, // MT_RF(7, 5)
    RfSwitch { rf_bank_reg: 0x0007_0009, bw_band: RF_G_BAND, value: 0x00 }, // MT_RF(7, 9)
    RfSwitch { rf_bank_reg: 0x0007_0009, bw_band: RF_A_BAND, value: 0x00 }, // MT_RF(7, 9)
    RfSwitch { rf_bank_reg: 0x0007_0046, bw_band: RF_G_BAND, value: 0x00 }, // MT_RF(7, 70)
    RfSwitch { rf_bank_reg: 0x0007_0046, bw_band: RF_A_BAND, value: 0x6D }, // MT_RF(7, 70)
    RfSwitch { rf_bank_reg: 0x0007_0047, bw_band: RF_G_BAND, value: 0x00 }, // MT_RF(7, 71)
    RfSwitch { rf_bank_reg: 0x0007_0047, bw_band: RF_A_BAND, value: 0xB0 }, // MT_RF(7, 71)
    RfSwitch { rf_bank_reg: 0x0007_004e, bw_band: RF_G_BAND, value: 0x00 }, // MT_RF(7, 78)
    RfSwitch { rf_bank_reg: 0x0007_004e, bw_band: RF_A_BAND, value: 0x55 }, // MT_RF(7, 78)
    RfSwitch { rf_bank_reg: 0x0007_004f, bw_band: RF_G_BAND, value: 0x00 }, // MT_RF(7, 79)
    RfSwitch { rf_bank_reg: 0x0007_004f, bw_band: RF_A_BAND, value: 0x55 }, // MT_RF(7, 79)
];

/// External-PA trim, applied on a channel change **only** on boards whose EEPROM
/// declares an external PA for the band (`mt76x02_ext_pa_enabled`, `phy.c:375`),
/// after `MT_RF_MISC` bit 2 (A band) or bit 3 (G band) is set.
///
/// Every row is `RF_A_BAND_{LB,MB,HB,11J}`, so on a 2.4 GHz external-PA board this
/// table contributes nothing — only the `MT_RF_MISC` bit is set. Upstream carries
/// no per-row comments for this table; none are invented here.
///
/// **44 rows.** `mt76x0_rf_ext_pa_tab`, `mt76x0/initvals_phy.h:586`.
// `rustfmt::skip`: left one Rust row per upstream row on purpose. Without it
// rustfmt breaks each `RfSwitch` across five lines, and the file stops being
// diffable against `initvals_phy.h` — which is the only reason to trust it.
#[rustfmt::skip]
pub const RF_EXT_PA_TAB: &[RfSwitch] = &[
    RfSwitch { rf_bank_reg: 0x0006_002d, bw_band: RF_A_BAND_LB, value: 0x63 }, // MT_RF(6, 45)
    RfSwitch { rf_bank_reg: 0x0006_002d, bw_band: RF_A_BAND_MB, value: 0x43 }, // MT_RF(6, 45)
    RfSwitch { rf_bank_reg: 0x0006_002d, bw_band: RF_A_BAND_HB, value: 0x33 }, // MT_RF(6, 45)
    RfSwitch { rf_bank_reg: 0x0006_002d, bw_band: RF_A_BAND_11J, value: 0x73 }, // MT_RF(6, 45)
    RfSwitch { rf_bank_reg: 0x0006_0032, bw_band: RF_A_BAND_LB, value: 0x02 }, // MT_RF(6, 50)
    RfSwitch { rf_bank_reg: 0x0006_0032, bw_band: RF_A_BAND_MB, value: 0x02 }, // MT_RF(6, 50)
    RfSwitch { rf_bank_reg: 0x0006_0032, bw_band: RF_A_BAND_HB, value: 0x02 }, // MT_RF(6, 50)
    RfSwitch { rf_bank_reg: 0x0006_0032, bw_band: RF_A_BAND_11J, value: 0x02 }, // MT_RF(6, 50)
    RfSwitch { rf_bank_reg: 0x0006_0033, bw_band: RF_A_BAND_LB, value: 0x02 }, // MT_RF(6, 51)
    RfSwitch { rf_bank_reg: 0x0006_0033, bw_band: RF_A_BAND_MB, value: 0x02 }, // MT_RF(6, 51)
    RfSwitch { rf_bank_reg: 0x0006_0033, bw_band: RF_A_BAND_HB, value: 0x02 }, // MT_RF(6, 51)
    RfSwitch { rf_bank_reg: 0x0006_0033, bw_band: RF_A_BAND_11J, value: 0x02 }, // MT_RF(6, 51)
    RfSwitch { rf_bank_reg: 0x0006_0034, bw_band: RF_A_BAND_LB, value: 0x08 }, // MT_RF(6, 52)
    RfSwitch { rf_bank_reg: 0x0006_0034, bw_band: RF_A_BAND_MB, value: 0x08 }, // MT_RF(6, 52)
    RfSwitch { rf_bank_reg: 0x0006_0034, bw_band: RF_A_BAND_HB, value: 0x08 }, // MT_RF(6, 52)
    RfSwitch { rf_bank_reg: 0x0006_0034, bw_band: RF_A_BAND_11J, value: 0x08 }, // MT_RF(6, 52)
    RfSwitch { rf_bank_reg: 0x0006_0035, bw_band: RF_A_BAND_LB, value: 0x08 }, // MT_RF(6, 53)
    RfSwitch { rf_bank_reg: 0x0006_0035, bw_band: RF_A_BAND_MB, value: 0x08 }, // MT_RF(6, 53)
    RfSwitch { rf_bank_reg: 0x0006_0035, bw_band: RF_A_BAND_HB, value: 0x08 }, // MT_RF(6, 53)
    RfSwitch { rf_bank_reg: 0x0006_0035, bw_band: RF_A_BAND_11J, value: 0x08 }, // MT_RF(6, 53)
    RfSwitch { rf_bank_reg: 0x0006_0036, bw_band: RF_A_BAND_LB, value: 0x0A }, // MT_RF(6, 54)
    RfSwitch { rf_bank_reg: 0x0006_0036, bw_band: RF_A_BAND_MB, value: 0x0A }, // MT_RF(6, 54)
    RfSwitch { rf_bank_reg: 0x0006_0036, bw_band: RF_A_BAND_HB, value: 0x0A }, // MT_RF(6, 54)
    RfSwitch { rf_bank_reg: 0x0006_0036, bw_band: RF_A_BAND_11J, value: 0x0A }, // MT_RF(6, 54)
    RfSwitch { rf_bank_reg: 0x0006_0037, bw_band: RF_A_BAND_LB, value: 0x0A }, // MT_RF(6, 55)
    RfSwitch { rf_bank_reg: 0x0006_0037, bw_band: RF_A_BAND_MB, value: 0x0A }, // MT_RF(6, 55)
    RfSwitch { rf_bank_reg: 0x0006_0037, bw_band: RF_A_BAND_HB, value: 0x0A }, // MT_RF(6, 55)
    RfSwitch { rf_bank_reg: 0x0006_0037, bw_band: RF_A_BAND_11J, value: 0x0A }, // MT_RF(6, 55)
    RfSwitch { rf_bank_reg: 0x0006_0038, bw_band: RF_A_BAND_LB, value: 0x05 }, // MT_RF(6, 56)
    RfSwitch { rf_bank_reg: 0x0006_0038, bw_band: RF_A_BAND_MB, value: 0x05 }, // MT_RF(6, 56)
    RfSwitch { rf_bank_reg: 0x0006_0038, bw_band: RF_A_BAND_HB, value: 0x05 }, // MT_RF(6, 56)
    RfSwitch { rf_bank_reg: 0x0006_0038, bw_band: RF_A_BAND_11J, value: 0x05 }, // MT_RF(6, 56)
    RfSwitch { rf_bank_reg: 0x0006_0039, bw_band: RF_A_BAND_LB, value: 0x05 }, // MT_RF(6, 57)
    RfSwitch { rf_bank_reg: 0x0006_0039, bw_band: RF_A_BAND_MB, value: 0x05 }, // MT_RF(6, 57)
    RfSwitch { rf_bank_reg: 0x0006_0039, bw_band: RF_A_BAND_HB, value: 0x05 }, // MT_RF(6, 57)
    RfSwitch { rf_bank_reg: 0x0006_0039, bw_band: RF_A_BAND_11J, value: 0x05 }, // MT_RF(6, 57)
    RfSwitch { rf_bank_reg: 0x0006_003a, bw_band: RF_A_BAND_LB, value: 0x05 }, // MT_RF(6, 58)
    RfSwitch { rf_bank_reg: 0x0006_003a, bw_band: RF_A_BAND_MB, value: 0x03 }, // MT_RF(6, 58)
    RfSwitch { rf_bank_reg: 0x0006_003a, bw_band: RF_A_BAND_HB, value: 0x02 }, // MT_RF(6, 58)
    RfSwitch { rf_bank_reg: 0x0006_003a, bw_band: RF_A_BAND_11J, value: 0x07 }, // MT_RF(6, 58)
    RfSwitch { rf_bank_reg: 0x0006_003b, bw_band: RF_A_BAND_LB, value: 0x05 }, // MT_RF(6, 59)
    RfSwitch { rf_bank_reg: 0x0006_003b, bw_band: RF_A_BAND_MB, value: 0x03 }, // MT_RF(6, 59)
    RfSwitch { rf_bank_reg: 0x0006_003b, bw_band: RF_A_BAND_HB, value: 0x02 }, // MT_RF(6, 59)
    RfSwitch { rf_bank_reg: 0x0006_003b, bw_band: RF_A_BAND_11J, value: 0x07 }, // MT_RF(6, 59)

];

#[cfg(test)]
mod tests {
    use super::*;

    /// `MT_RF_BANK(offset)` — `mt76x0/phy.h:22`.
    const fn bank(rf_bank_reg: u32) -> u32 {
        rf_bank_reg >> 16
    }

    /// `MT_RF_REG(offset)` — `mt76x0/phy.h:23`. Masks a *byte*, not the low half
    /// word, so an address with junk in bits 8..15 would silently alias a valid
    /// register rather than being rejected; [`no_junk_between_bank_and_reg`]
    /// covers that gap.
    const fn reg(rf_bank_reg: u32) -> u32 {
        rf_bank_reg & 0xff
    }

    /// Row counts, hand-counted from the upstream header, one table per line.
    /// A transcription that drops or duplicates a row is the failure mode these
    /// tables are most exposed to, and the only one a compiler cannot catch.
    #[test]
    fn row_counts_match_upstream() {
        assert_eq!(RF_CENTRAL_TAB.len(), 44, "mt76x0_rf_central_tab");
        assert_eq!(RF_2G_CHANNEL_0_TAB.len(), 68, "mt76x0_rf_2g_channel_0_tab");
        assert_eq!(RF_5G_CHANNEL_0_TAB.len(), 38, "mt76x0_rf_5g_channel_0_tab");
        assert_eq!(
            RF_VGA_CHANNEL_0_TAB.len(),
            35,
            "mt76x0_rf_vga_channel_0_tab"
        );
        assert_eq!(RF_BW_SWITCH_TAB.len(), 41, "mt76x0_rf_bw_switch_tab");
        assert_eq!(RF_BAND_SWITCH_TAB.len(), 43, "mt76x0_rf_band_switch_tab");
        assert_eq!(RF_EXT_PA_TAB.len(), 44, "mt76x0_rf_ext_pa_tab");
    }

    /// Every address in this file must survive `mt76x0_rf_csr_wr`'s validity
    /// check (`phy.c:32`): `bank <= 8` and `reg <= 127`. A row that fails would
    /// be dropped with `-EINVAL` on MMIO, or — worse, on our USB path, which goes
    /// through the MCU register-pair protocol and never sees that check — written
    /// to whatever the truncated address resolves to.
    #[test]
    fn addresses_within_csr_bounds() {
        let pair_tabs: [(&str, &[(u32, u32)]); 4] = [
            ("RF_CENTRAL_TAB", RF_CENTRAL_TAB),
            ("RF_2G_CHANNEL_0_TAB", RF_2G_CHANNEL_0_TAB),
            ("RF_5G_CHANNEL_0_TAB", RF_5G_CHANNEL_0_TAB),
            ("RF_VGA_CHANNEL_0_TAB", RF_VGA_CHANNEL_0_TAB),
        ];
        for (name, tab) in pair_tabs {
            for &(addr, val) in tab {
                assert!(
                    bank(addr) <= 8,
                    "{name}: bank {} > 8 in {addr:#010x}",
                    bank(addr)
                );
                assert!(
                    reg(addr) <= 127,
                    "{name}: reg {} > 127 in {addr:#010x}",
                    reg(addr)
                );
                assert!(
                    val <= 0xff,
                    "{name}: value {val:#x} is not a byte at {addr:#010x}"
                );
            }
        }

        let switch_tabs: [(&str, &[RfSwitch]); 3] = [
            ("RF_BW_SWITCH_TAB", RF_BW_SWITCH_TAB),
            ("RF_BAND_SWITCH_TAB", RF_BAND_SWITCH_TAB),
            ("RF_EXT_PA_TAB", RF_EXT_PA_TAB),
        ];
        for (name, tab) in switch_tabs {
            for item in tab {
                let a = item.rf_bank_reg;
                assert!(bank(a) <= 8, "{name}: bank {} > 8 in {a:#010x}", bank(a));
                assert!(reg(a) <= 127, "{name}: reg {} > 127 in {a:#010x}", reg(a));
            }
        }
    }

    /// `MT_RF_REG` masks `& 0xff`, so bits 8..15 of an address are silently
    /// discarded by the hardware path. Any set bit there is a transcription
    /// slip in the resolved literal, not something upstream ever emits.
    #[test]
    fn no_junk_between_bank_and_reg() {
        let addrs = RF_CENTRAL_TAB
            .iter()
            .chain(RF_2G_CHANNEL_0_TAB)
            .chain(RF_5G_CHANNEL_0_TAB)
            .chain(RF_VGA_CHANNEL_0_TAB)
            .map(|&(a, _)| a)
            .chain(
                RF_BW_SWITCH_TAB
                    .iter()
                    .chain(RF_BAND_SWITCH_TAB)
                    .chain(RF_EXT_PA_TAB)
                    .map(|i| i.rf_bank_reg),
            );
        for a in addrs {
            assert_eq!(a & 0x0000_ff00, 0, "{a:#010x} has bits set in 8..15");
        }
    }

    /// Every switch row must be reachable: a row with no band bit and no
    /// bandwidth bit could never match any of the three filters, and a row with
    /// `bw_band == 0` would match the `rf_bw == 0` case by accident.
    #[test]
    fn switch_rows_are_selectable() {
        const BANDS: u16 =
            RF_G_BAND | RF_A_BAND | RF_A_BAND_LB | RF_A_BAND_MB | RF_A_BAND_HB | RF_A_BAND_11J;
        const BWS: u16 = RF_BW_20 | RF_BW_40 | RF_BW_10 | RF_BW_80;

        for item in RF_BW_SWITCH_TAB {
            assert_ne!(
                item.bw_band & BWS,
                0,
                "{:#010x} has no RF_BW_* bit",
                item.rf_bank_reg
            );
        }
        for item in RF_BAND_SWITCH_TAB.iter().chain(RF_EXT_PA_TAB) {
            assert_ne!(
                item.bw_band & BANDS,
                0,
                "{:#010x} has no band bit",
                item.rf_bank_reg
            );
            assert_eq!(
                item.bw_band & BWS,
                0,
                "{:#010x} unexpectedly carries a bandwidth bit",
                item.rf_bank_reg
            );
        }
        for item in RF_BW_SWITCH_TAB
            .iter()
            .chain(RF_BAND_SWITCH_TAB)
            .chain(RF_EXT_PA_TAB)
        {
            assert_eq!(
                item.bw_band & !(BANDS | BWS),
                0,
                "{:#010x} has an unknown bw_band bit",
                item.rf_bank_reg
            );
        }
    }

    /// The banks each table owns, as the module docs claim. Cheap, and it catches
    /// a row pasted into the wrong table — the second-most-likely transcription
    /// slip after a dropped row.
    #[test]
    fn tables_stay_in_their_banks() {
        assert!(RF_CENTRAL_TAB.iter().all(|&(a, _)| bank(a) == 0));
        assert!(RF_2G_CHANNEL_0_TAB.iter().all(|&(a, _)| bank(a) == 5));
        assert!(RF_5G_CHANNEL_0_TAB.iter().all(|&(a, _)| bank(a) == 6));
        assert!(RF_VGA_CHANNEL_0_TAB.iter().all(|&(a, _)| bank(a) == 7));
        // The switch tables deliberately span banks 0, 6 and 7.
        assert!(
            RF_BW_SWITCH_TAB
                .iter()
                .all(|i| matches!(bank(i.rf_bank_reg), 0 | 7))
        );
        assert!(
            RF_BAND_SWITCH_TAB
                .iter()
                .all(|i| matches!(bank(i.rf_bank_reg), 0 | 6 | 7))
        );
        assert!(RF_EXT_PA_TAB.iter().all(|i| bank(i.rf_bank_reg) == 6));
    }

    /// Within a reg-pair table upstream writes each register exactly once; a
    /// duplicate would mean a row was pasted twice. (The switch tables *do*
    /// repeat an address, once per bw/band combination, so they are excluded.)
    #[test]
    fn reg_pair_tables_have_no_duplicate_addresses() {
        for (name, tab) in [
            ("RF_CENTRAL_TAB", RF_CENTRAL_TAB),
            ("RF_2G_CHANNEL_0_TAB", RF_2G_CHANNEL_0_TAB),
            ("RF_5G_CHANNEL_0_TAB", RF_5G_CHANNEL_0_TAB),
            ("RF_VGA_CHANNEL_0_TAB", RF_VGA_CHANNEL_0_TAB),
        ] {
            let mut seen: Vec<u32> = tab.iter().map(|&(a, _)| a).collect();
            let total = seen.len();
            seen.sort_unstable();
            seen.dedup();
            assert_eq!(seen.len(), total, "{name} has a duplicate address");
        }
    }

    /// Upstream's `mt76x0_rf_patch_reg_array` (`phy.c:1114`) overrides three rows
    /// by chip variant. For the USB MT7610U the override values equal what the
    /// tables already hold, which is why [`super::phy`] may write these tables
    /// unpatched. Pin that equality: if a future re-transcription changes one of
    /// these bytes, the no-patch shortcut silently becomes wrong.
    #[test]
    fn usb_mt7610u_patch_values_are_already_in_the_tables() {
        let find =
            |tab: &[(u32, u32)], addr: u32| tab.iter().find(|&&(a, _)| a == addr).map(|&(_, v)| v);
        // MT_RF(0, 3) -> 0x73 on USB (phy.c:1126).
        assert_eq!(find(RF_CENTRAL_TAB, 0x0000_0003), Some(0x73));
        // MT_RF(0, 21) -> 0x12 unless MT7610E (phy.c:1130).
        assert_eq!(find(RF_CENTRAL_TAB, 0x0000_0015), Some(0x12));
        // MT_RF(5, 2) -> 0x0c unless MT7630/MT7610E (phy.c:1136).
        assert_eq!(find(RF_2G_CHANNEL_0_TAB, 0x0005_0002), Some(0x0C));
    }
}
