//! MT7610U (`mt76x0`) **EEPROM / eFuse** image: how to get it off the part, and
//! what every byte in it means.
//!
//! Upstream reference (read-only GPL tree under `scratchpad/mt76-src/`):
//! `mt76x0/eeprom.c`, `mt76x0/eeprom.h`, `mt76x02_eeprom.c`, `mt76x02_eeprom.h`.
//! The field offsets are `enum mt76x02_eeprom_field` (`mt76x02_eeprom.h:12-96`)
//! and every `MT_EE_*` below carries that line.
//!
//! ## MEASURED vs CODE-READ
//! **CODE-READ, all of it.** No MT7610U EEPROM image has been dumped off the
//! target silicon yet — this file exists so that when one *is* dumped it can be
//! diffed field-by-field against a parse whose every offset names an upstream
//! constant. Treat every value this module returns as unvalidated until that diff
//! is done. The one thing carried in from measurement is the cost model: an EP0
//! vendor-request round trip on this part is **151 µs**, which is why the whole
//! image is read once at bring-up and then parsed from RAM, and why nothing here
//! is ever called on a per-frame path.
//!
//! ## Two ways to read the image, and which one upstream actually uses
//! 1. **eFuse register path** ([`read_efuse_image`]) — `MT_EFUSE_CTRL` (0x0024) +
//!    `MT_EFUSE_DATA(0..3)` (0x0028..0x0034), 16 bytes per kick.
//!    `mt76x02_eeprom.c:11-43`. ★ This is the *only* path mainline mt76 uses for
//!    mt76x0: `mt76x0_load_eeprom` (`mt76x0/eeprom.c:293-310`) calls
//!    `mt76_eeprom_init`, which on USB reads device-tree/platform data only
//!    (`eeprom.c:542-550`) and therefore finds nothing, then falls through to
//!    `mt76x02_get_efuse_data(dev, 0, ..., 512, MT_EE_READ)`.
//! 2. **USB EEPROM shadow** ([`read_shadow_image`]) — `MT_VEND_READ_EEPROM`
//!    (bRequest `0x09`, `mt76.h:637`), 4 bytes per control-IN. Reachable in
//!    mainline only through `MT_VEND_ADDR(EEPROM, n)` addressing (`mt76.h:626-630`,
//!    dispatched at `usb.c:96-99`) — which **no caller in the tree uses**. It is a
//!    legacy/vendor-driver path. Whether it returns anything on the MT7610U is an
//!    **OPEN question**; the mt7612 backend in this crate uses the same request
//!    successfully (`src/mt7612/mod.rs:439`), which is the only reason to try it.
//!
//! [`Mt76x0Eeprom::load`] therefore tries the shadow **first but validated** —
//! the image must pass [`image_looks_valid`] (`mt76x0_check_eeprom`,
//! `mt76x0/eeprom.c:273-291`: chip id `0x7610`/`0x7650` at offset 0, or at
//! `MT_EE_PCI_ID` if offset 0 is zero) — and falls back to the eFuse path
//! otherwise. A shadow that answers with zeros or garbage cannot poison the
//! parse, which is the whole point of validating before accepting.
//!
//! ## Sign encodings: three different ones, none of them two's complement
//! This is the single easiest thing to get wrong in this file, so all three are
//! isolated as named helpers with their upstream line:
//!   * [`s6_to_s8`] (`mt76x0/eeprom.h:26-33`) — a genuine **6-bit two's
//!     complement** value. Used for the per-rate power bytes.
//!   * [`sign_extend`] (`mt76x02_eeprom.h:136-144`) — **sign-magnitude, inverted**:
//!     bit `size-1` is the sign and it means *positive*; a clear sign bit means
//!     the magnitude is negated. Used for the temperature and frequency offsets.
//!   * [`rate_power_val`] (`mt76x02_eeprom.h:154-160`) — bit 7 is an *enable*
//!     ("this delta is programmed"), bit 6 is the sign in the [`sign_extend`]
//!     sense, bits 5:0 the magnitude. Used for the bandwidth power deltas.
//!
//! ## Units
//! Every power number in this module is in **0.5 dB steps**, not dBm. That is not
//! a guess: mac80211 reports the part's TX power as
//! `DIV_ROUND_UP(phy->txpower_cur + delta, 2)` (`mac80211.c:1809`), where
//! `txpower_cur` is the max of the per-rate table. [`Mt76x0Eeprom::target_power_dbm`]
//! is the one accessor that does that division and returns real dBm; everything
//! else is named `_half_db` so a caller cannot mistake the domain.
#![allow(dead_code)]

use std::sync::atomic::{AtomicU8, Ordering};
use std::time::{Duration, Instant};

use ndn_radio_hal::Bandwidth;

use crate::FaceError;
use crate::mt76::Mt76Regs;
use crate::mt76::regs::{
    MT_EFUSE_CTRL, MT_EFUSE_CTRL_AIN, MT_EFUSE_CTRL_AOUT, MT_EFUSE_CTRL_KICK, MT_EFUSE_CTRL_MODE,
    field_prep, mt_efuse_data,
};

fn err(msg: String) -> FaceError {
    FaceError::Io(std::io::Error::other(msg))
}

// ── Image geometry ───────────────────────────────────────────────────────────

/// EEPROM image size for this part. `mt76x0/eeprom.h:16` (`MT76X0_EEPROM_SIZE`).
pub const MT76X0_EEPROM_SIZE: usize = 512;

/// Highest EEPROM layout version this parse claims to understand.
/// `mt76x0/eeprom.h:15` (`MT76X0U_EE_MAX_VER`). Upstream only *warns* past it.
pub const MT76X0U_EE_MAX_VER: u8 = 0x0c;

// ── Field offsets — `enum mt76x02_eeprom_field`, mt76x02_eeprom.h:12-96 ──────
// Byte offsets into the image, not register addresses. Upstream's accessor
// `mt76x02_eeprom_get` (mt76x02_eeprom.h:162-170) reads a little-endian u16 and
// rejects odd offsets; the odd-numbered fields below are therefore always reached
// as the *high byte* of the even word one below them, and each says so.

pub const MT_EE_CHIP_ID: usize = 0x000; // mt76x02_eeprom.h:13
pub const MT_EE_VERSION: usize = 0x002; // mt76x02_eeprom.h:14 — hi=version, lo=FAE
pub const MT_EE_MAC_ADDR: usize = 0x004; // mt76x02_eeprom.h:15 — 6 bytes
pub const MT_EE_PCI_ID: usize = 0x00a; // mt76x02_eeprom.h:16
pub const MT_EE_ANTENNA: usize = 0x022; // mt76x02_eeprom.h:17
pub const MT_EE_CFG1_INIT: usize = 0x024; // mt76x02_eeprom.h:18
pub const MT_EE_NIC_CONF_0: usize = 0x034; // mt76x02_eeprom.h:19
pub const MT_EE_NIC_CONF_1: usize = 0x036; // mt76x02_eeprom.h:20
pub const MT_EE_COUNTRY_REGION_5GHZ: usize = 0x038; // mt76x02_eeprom.h:21 (byte)
pub const MT_EE_COUNTRY_REGION_2GHZ: usize = 0x039; // mt76x02_eeprom.h:22 (byte)
/// Crystal/frequency trim. ⚠ Same offset as [`MT_EE_XTAL_TRIM_1`] — upstream
/// names 0x03a twice (`mt76x02_eeprom.h:23` and `:26`); both names are kept
/// rather than deduplicated, because the two consumers read it differently
/// (mt76x0 takes the low byte as a frequency offset; mt76x2 takes the whole word
/// as a crystal trim, `mt76x2/usb_mac.c:24`).
pub const MT_EE_FREQ_OFFSET: usize = 0x03a; // mt76x02_eeprom.h:23
pub const MT_EE_NIC_CONF_2: usize = 0x042; // mt76x02_eeprom.h:24

pub const MT_EE_XTAL_TRIM_1: usize = 0x03a; // mt76x02_eeprom.h:26 — see above
pub const MT_EE_XTAL_TRIM_2: usize = 0x09e; // mt76x02_eeprom.h:27 (mt76x2 only)

/// lo = 2.4 GHz LNA gain, hi = 5 GHz LNA gain for the low group. `:29`
pub const MT_EE_LNA_GAIN: usize = 0x044; // mt76x02_eeprom.h:29
/// 2.4 GHz RSSI offsets, one byte per RX chain (lo = chain 0, hi = chain 1). `:30`
pub const MT_EE_RSSI_OFFSET_2G_0: usize = 0x046; // mt76x02_eeprom.h:30
/// ⚠ Only its **high** byte is used, and it is not an RSSI offset at all:
/// it is [`MT_EE_LNA_GAIN_5GHZ_1`]. `mt76x02_eeprom.c:111-112`.
pub const MT_EE_RSSI_OFFSET_2G_1: usize = 0x048; // mt76x02_eeprom.h:31
pub const MT_EE_LNA_GAIN_5GHZ_1: usize = 0x049; // mt76x02_eeprom.h:32 (byte)
pub const MT_EE_RSSI_OFFSET_5G_0: usize = 0x04a; // mt76x02_eeprom.h:33
/// ⚠ As with the 2G twin: only its high byte matters, as
/// [`MT_EE_LNA_GAIN_5GHZ_2`]. `mt76x02_eeprom.c:114-115`.
pub const MT_EE_RSSI_OFFSET_5G_1: usize = 0x04c; // mt76x02_eeprom.h:34
pub const MT_EE_LNA_GAIN_5GHZ_2: usize = 0x04d; // mt76x02_eeprom.h:35 (byte)

/// 40 MHz power delta: lo = 2.4 GHz, hi = 5 GHz. `mt76x0/eeprom.c:140-144`.
pub const MT_EE_TX_POWER_DELTA_BW40: usize = 0x050; // mt76x02_eeprom.h:37
/// ⚠ Double duty. Named for the 80 MHz delta, but mt76x0 never reads it as one
/// (the 80 MHz delta comes from `MT_EE_5G_TARGET_POWER >> 8`,
/// `mt76x0/eeprom.c:136`). It is instead the **base of the 2.4 GHz per-channel-group
/// target-power byte array** — `addr = MT_EE_TX_POWER_DELTA_BW80 + offset`,
/// `mt76x0/eeprom.c:243` — which is why the groups at [`MT_EE_TX_POWER_0_START_2G`]
/// and [`MT_EE_TX_POWER_1_START_2G`] fall inside its reach.
pub const MT_EE_TX_POWER_DELTA_BW80: usize = 0x052; // mt76x02_eeprom.h:38

pub const MT_EE_TX_POWER_EXT_PA_5G: usize = 0x054; // mt76x02_eeprom.h:40 (mt76x2 only)

pub const MT_EE_TX_POWER_0_START_2G: usize = 0x056; // mt76x02_eeprom.h:42
pub const MT_EE_TX_POWER_1_START_2G: usize = 0x05c; // mt76x02_eeprom.h:43

/// `MT_TX_POWER_GROUP_SIZE_5G` — mt76x02_eeprom.h:46.
pub const MT_TX_POWER_GROUP_SIZE_5G: usize = 5;
/// `MT_TX_POWER_GROUPS_5G` — mt76x02_eeprom.h:47.
pub const MT_TX_POWER_GROUPS_5G: usize = 6;
pub const MT_EE_TX_POWER_0_START_5G: usize = 0x062; // mt76x02_eeprom.h:48
pub const MT_EE_TSSI_SLOPE_2G: usize = 0x06e; // mt76x02_eeprom.h:49

pub const MT_EE_TX_POWER_0_GRP3_TX_POWER_DELTA: usize = 0x074; // mt76x02_eeprom.h:51
/// ⚠ Double duty like [`MT_EE_TX_POWER_DELTA_BW80`]: `+ 2 + offset` is the base
/// of the **5 GHz per-channel-group target-power byte array**
/// (`mt76x0/eeprom.c:264`), so the array actually starts at 0x078.
pub const MT_EE_TX_POWER_0_GRP4_TSSI_SLOPE: usize = 0x076; // mt76x02_eeprom.h:52

pub const MT_EE_TX_POWER_1_START_5G: usize = 0x080; // mt76x02_eeprom.h:54

pub const MT_EE_TX_POWER_CCK: usize = 0x0a0; // mt76x02_eeprom.h:56
pub const MT_EE_TX_POWER_OFDM_2G_6M: usize = 0x0a2; // mt76x02_eeprom.h:57
pub const MT_EE_TX_POWER_OFDM_2G_24M: usize = 0x0a4; // mt76x02_eeprom.h:58
pub const MT_EE_TX_POWER_OFDM_5G_6M: usize = 0x0b2; // mt76x02_eeprom.h:59
pub const MT_EE_TX_POWER_OFDM_5G_24M: usize = 0x0b4; // mt76x02_eeprom.h:60
pub const MT_EE_TX_POWER_HT_MCS0: usize = 0x0a6; // mt76x02_eeprom.h:61
pub const MT_EE_TX_POWER_HT_MCS4: usize = 0x0a8; // mt76x02_eeprom.h:62
pub const MT_EE_TX_POWER_HT_MCS8: usize = 0x0aa; // mt76x02_eeprom.h:63
pub const MT_EE_TX_POWER_HT_MCS12: usize = 0x0ac; // mt76x02_eeprom.h:64
pub const MT_EE_TX_POWER_VHT_MCS8: usize = 0x0be; // mt76x02_eeprom.h:65

/// lo = 2.4 GHz target power, hi = [`MT_EE_TEMP_OFFSET`]. `mt76x0/eeprom.c:86`.
pub const MT_EE_2G_TARGET_POWER: usize = 0x0d0; // mt76x02_eeprom.h:67
pub const MT_EE_TEMP_OFFSET: usize = 0x0d1; // mt76x02_eeprom.h:68 (byte)
/// lo = 5 GHz target power, hi = the **80 MHz** power delta. `mt76x0/eeprom.c:136`.
pub const MT_EE_5G_TARGET_POWER: usize = 0x0d2; // mt76x02_eeprom.h:69
/// First of seven consecutive 5 GHz TSSI channel bounds (0x0d4..=0x0da),
/// read as a byte array by `mt76x02_eeprom_copy` at `mt76x0/phy.c:711`.
pub const MT_EE_TSSI_BOUND1: usize = 0x0d4; // mt76x02_eeprom.h:70
pub const MT_EE_TSSI_BOUND2: usize = 0x0d6; // mt76x02_eeprom.h:71
pub const MT_EE_TSSI_BOUND3: usize = 0x0d8; // mt76x02_eeprom.h:72
/// lo = TSSI bound 4, hi = [`MT_EE_FREQ_OFFSET_COMPENSATION`]. `mt76x0/eeprom.c:103`.
pub const MT_EE_TSSI_BOUND4: usize = 0x0da; // mt76x02_eeprom.h:73
pub const MT_EE_FREQ_OFFSET_COMPENSATION: usize = 0x0db; // mt76x02_eeprom.h:74 (byte)
pub const MT_EE_TSSI_BOUND5: usize = 0x0dc; // mt76x02_eeprom.h:75
/// Base of the 2.4 GHz per-rate power table (5 words, 0x0de..0x0e7).
/// The 5 GHz table has **no symbolic name upstream** — see [`MT_EE_TX_POWER_BYRATE_5G`].
pub const MT_EE_TX_POWER_BYRATE_BASE: usize = 0x0de; // mt76x02_eeprom.h:76

pub const MT_EE_TSSI_SLOPE_5G: usize = 0x0f0; // mt76x02_eeprom.h:78
pub const MT_EE_RF_TEMP_COMP_SLOPE_5G: usize = 0x0f2; // mt76x02_eeprom.h:79
pub const MT_EE_RF_TEMP_COMP_SLOPE_2G: usize = 0x0f4; // mt76x02_eeprom.h:80

pub const MT_EE_RF_2G_TSSI_OFF_TXPOWER: usize = 0x0f6; // mt76x02_eeprom.h:82
pub const MT_EE_RF_2G_RX_HIGH_GAIN: usize = 0x0f8; // mt76x02_eeprom.h:83
pub const MT_EE_RF_5G_GRP0_1_RX_HIGH_GAIN: usize = 0x0fa; // mt76x02_eeprom.h:84
pub const MT_EE_RF_5G_GRP2_3_RX_HIGH_GAIN: usize = 0x0fc; // mt76x02_eeprom.h:85
pub const MT_EE_RF_5G_GRP4_5_RX_HIGH_GAIN: usize = 0x0fe; // mt76x02_eeprom.h:86

/// ★ **Unnamed upstream.** `mt76x0_get_tx_power_per_rate` writes the 5 GHz
/// per-rate addresses as bare literals — `0x120`, `0x122`, `0x124`, `0x126`
/// (`mt76x0/eeprom.c:168,174,180,186`) — with no `MT_EE_*` constant anywhere in
/// the tree. Named here so the layout is greppable; the literals are upstream's.
pub const MT_EE_TX_POWER_BYRATE_5G: usize = 0x120;
/// ★ Also unnamed upstream: the VHT MCS 8/9 pair, literal `0x12c` at
/// `mt76x0/eeprom.c:192`. Note this is *not* [`MT_EE_TX_POWER_VHT_MCS8`] (0x0be),
/// which mt76x0 never reads.
pub const MT_EE_TX_POWER_BYRATE_VHT_5G: usize = 0x12c;

pub const MT_EE_BT_RCAL_RESULT: usize = 0x138; // mt76x02_eeprom.h:88
pub const MT_EE_BT_VCDL_CALIBRATION: usize = 0x13c; // mt76x02_eeprom.h:89
pub const MT_EE_BT_PMUCFG: usize = 0x13e; // mt76x02_eeprom.h:90

pub const MT_EE_USAGE_MAP_START: usize = 0x1e0; // mt76x02_eeprom.h:92
pub const MT_EE_USAGE_MAP_END: usize = 0x1fc; // mt76x02_eeprom.h:93
/// `MT_EFUSE_USAGE_MAP_SIZE` — mt76x02_eeprom.h:118. 0x1fc - 0x1e0 + 1 = 29.
pub const MT_EFUSE_USAGE_MAP_SIZE: usize = MT_EE_USAGE_MAP_END - MT_EE_USAGE_MAP_START + 1;
/// `MT_MAP_READS = DIV_ROUND_UP(MT_EFUSE_USAGE_MAP_SIZE, 16)` — mt76x0/eeprom.c:18.
/// = ceil(29 / 16) = 2. Pinned by a test rather than computed, so the constant is
/// readable and a layout change cannot silently shift it.
const MT_MAP_READS: usize = 2;

// ── NIC_CONF bitfields — mt76x02_eeprom.h:98-116 ─────────────────────────────

pub const MT_EE_ANTENNA_DUAL: u16 = 0x8000; // BIT(15) — mt76x02_eeprom.h:98

pub const MT_EE_NIC_CONF_0_RX_PATH: u16 = 0x000f; // GENMASK(3,0) — :100
pub const MT_EE_NIC_CONF_0_TX_PATH: u16 = 0x00f0; // GENMASK(7,4) — :101
pub const MT_EE_NIC_CONF_0_PA_TYPE: u16 = 0x0300; // GENMASK(9,8) — :102
pub const MT_EE_NIC_CONF_0_PA_INT_2G: u16 = 0x0100; // BIT(8) — :103
pub const MT_EE_NIC_CONF_0_PA_INT_5G: u16 = 0x0200; // BIT(9) — :104
pub const MT_EE_NIC_CONF_0_PA_IO_CURRENT: u16 = 0x0400; // BIT(10) — :105
pub const MT_EE_NIC_CONF_0_BOARD_TYPE: u16 = 0x3000; // GENMASK(13,12) — :106

pub const MT_EE_NIC_CONF_1_HW_RF_CTRL: u16 = 0x0001; // BIT(0) — :108
pub const MT_EE_NIC_CONF_1_TEMP_TX_ALC: u16 = 0x0002; // BIT(1) — :109
pub const MT_EE_NIC_CONF_1_LNA_EXT_2G: u16 = 0x0004; // BIT(2) — :110
pub const MT_EE_NIC_CONF_1_LNA_EXT_5G: u16 = 0x0008; // BIT(3) — :111
/// TSSI / automatic-level-control enable. Gates *both* power paths:
/// with it set the per-rate deltas are used raw and the target power comes from
/// the 2G/5G target-power words; with it clear the bandwidth delta is applied and
/// the target power comes from the per-channel-group byte arrays.
/// `mt76x0/eeprom.h:35-39` (`mt76x0_tssi_enabled`).
pub const MT_EE_NIC_CONF_1_TX_ALC_EN: u16 = 0x2000; // BIT(13) — :112

pub const MT_EE_NIC_CONF_2_ANT_OPT: u16 = 0x0008; // BIT(3) — :114
pub const MT_EE_NIC_CONF_2_ANT_DIV: u16 = 0x0010; // BIT(4) — :115
pub const MT_EE_NIC_CONF_2_XTAL_OPTION: u16 = 0x0600; // GENMASK(10,9) — :116

// ── Sign / validity encodings ────────────────────────────────────────────────

/// `mt76x02_field_valid` — mt76x02_eeprom.h:131-134. A byte is "programmed" iff
/// it is neither 0 nor 0xff, the two states a blank or shorted eFuse cell takes.
pub const fn field_valid(val: u8) -> bool {
    val != 0 && val != 0xff
}

/// `s6_to_s8` — mt76x0/eeprom.h:26-33. Genuine 6-bit two's complement: bit 5 is
/// the sign, bits 4:0 the magnitude. Takes a `u32` because upstream feeds it
/// `val` and `val >> 8` from a `u16` without masking first.
pub const fn s6_to_s8(val: u32) -> i8 {
    let mut ret = (val & 0x3f) as i8;
    if ret & 0x20 != 0 {
        // ret is in 32..=63 here, so this cannot overflow i8.
        ret -= 0x40;
    }
    ret
}

/// `mt76x02_sign_extend` — mt76x02_eeprom.h:136-144.
///
/// ★ Despite the name this is **not** sign extension. It is sign-magnitude with
/// an *inverted* sign bit: bit `size-1` set means the value is **positive**, and
/// a clear sign bit means the magnitude is negated. Ported exactly; a "fix" to
/// ordinary two's complement would silently invert every temperature and
/// frequency trim on the part.
pub fn sign_extend(val: u32, size: u32) -> i32 {
    let sign_bit = 1u32 << (size - 1);
    let sign = val & sign_bit != 0;
    let magnitude = (val & (sign_bit - 1)) as i32;
    if sign { magnitude } else { -magnitude }
}

/// `mt76x02_sign_extend_optional` — mt76x02_eeprom.h:146-152. Bit `size` is an
/// enable: clear means "no value programmed", which reads as 0 rather than as a
/// magnitude of 0.
pub fn sign_extend_optional(val: u32, size: u32) -> i32 {
    if val & (1u32 << size) != 0 {
        sign_extend(val, size)
    } else {
        0
    }
}

/// `mt76x02_rate_power_val` — mt76x02_eeprom.h:154-160. Decodes one bandwidth
/// power-delta byte: bit 7 = programmed, bit 6 = sign (set → positive, per
/// [`sign_extend`]), bits 5:0 = magnitude in 0.5 dB steps.
pub fn rate_power_val(val: u8) -> i8 {
    if !field_valid(val) {
        return 0;
    }
    // Range is ±63 by construction (6-bit magnitude), so the cast cannot truncate.
    sign_extend_optional(u32::from(val), 7) as i8
}

/// Band split for this part: 2.4 GHz is channels 1..=14, everything above is
/// 5 GHz. Upstream carries the band in `struct ieee80211_channel`; a userspace
/// driver only ever has the channel number, so the split is made explicit here
/// and used consistently by every accessor that upstream keys on `chan->band`.
pub const fn is_5ghz_channel(chan: u8) -> bool {
    chan > 14
}

// ── Board type ───────────────────────────────────────────────────────────────

/// `enum mt76x02_board_type` — mt76x02_eeprom.h:126-129, decoded from
/// [`MT_EE_NIC_CONF_0_BOARD_TYPE`].
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum BoardType {
    /// Field value 1 — 2.4 GHz only.
    Band2GhzOnly,
    /// Field value 2 — 5 GHz only.
    Band5GhzOnly,
    /// Anything else (0 or 3), which `mt76x02_eeprom_parse_hw_cap`
    /// (`mt76x02_eeprom.c:72-88`) treats as dual-band.
    DualBand(u16),
}

impl BoardType {
    /// Whether the board declares 2.4 GHz support. `mt76x02_eeprom.c:76-87`.
    pub const fn has_2ghz(self) -> bool {
        matches!(self, BoardType::Band2GhzOnly | BoardType::DualBand(_))
    }
    /// Whether the board declares 5 GHz support. `mt76x02_eeprom.c:76-87`.
    pub const fn has_5ghz(self) -> bool {
        matches!(self, BoardType::Band5GhzOnly | BoardType::DualBand(_))
    }
}

// ── Per-rate power table ─────────────────────────────────────────────────────

/// `struct mt76x02_rate_power` — mt76x02.h:75-85. Per-rate TX power in
/// **0.5 dB steps**, relative to the channel's target power.
///
/// Upstream overlays the four arrays with a flat `s8 all[30]` through a union and
/// walks that for the offset/limit/max operations. A union needs `unsafe` in
/// Rust, so [`iter`](Self::iter) and [`map_all`](Self::map_all) walk the four
/// arrays **in declaration order** instead — `cck`, `ofdm`, `ht`, `vht`, which is
/// exactly the order the union lays them out (4 + 8 + 16 + 2 = 30). The order is
/// load-bearing only for [`max`](Self::max), and pinned by a test.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct RatePower {
    /// CCK 1M, 2M, 5.5M, 11M.
    pub cck: [i8; 4],
    /// OFDM 6M, 9M, 12M, 18M, 24M, 36M, 48M, 54M.
    pub ofdm: [i8; 8],
    /// HT/VHT MCS 0..15. ⚠ `mt76x0_get_tx_power_per_rate` only fills 0..=7
    /// (`mt76x0/eeprom.c:179-189`) — this is a 1×1 part, there is no second
    /// stream — so 8..=15 stay 0 and then receive the bandwidth delta like every
    /// other entry, because upstream's offset walk covers the whole union.
    pub ht: [i8; 16],
    /// VHT MCS 8, 9 (5 GHz only). `mt76x0/eeprom.c:191-194`.
    pub vht: [i8; 2],
}

impl RatePower {
    /// The 30 entries in union order — `cck`, `ofdm`, `ht`, `vht`.
    pub fn iter(&self) -> impl Iterator<Item = i8> + '_ {
        self.cck
            .iter()
            .chain(self.ofdm.iter())
            .chain(self.ht.iter())
            .chain(self.vht.iter())
            .copied()
    }

    /// Apply `f` to all 30 entries in union order.
    pub fn map_all(&mut self, mut f: impl FnMut(i8) -> i8) {
        for v in self
            .cck
            .iter_mut()
            .chain(self.ofdm.iter_mut())
            .chain(self.ht.iter_mut())
            .chain(self.vht.iter_mut())
        {
            *v = f(*v);
        }
    }

    /// `mt76x02_add_rate_power_offset` — mt76x02_phy.c:84-91.
    ///
    /// ⚠ Deviation, deliberate: upstream's `r->all[i] += offset` wraps on `s8`.
    /// This **saturates**. A wrap here would turn a small over-range into a
    /// maximum-power write, which is the one failure mode worth spending a
    /// behavioural difference on; with real EEPROM values (magnitudes ≤ 63 plus a
    /// delta ≤ 63) neither path is reachable.
    pub fn add_offset(&mut self, offset: i8) {
        self.map_all(|v| v.saturating_add(offset));
    }

    /// `mt76x02_limit_rate_power` — mt76x02_phy.c:74-82.
    pub fn limit(&mut self, limit: i8) {
        self.map_all(|v| if v > limit { limit } else { v });
    }

    /// `mt76x02_get_max_rate_power` — mt76x02_phy.c:62-72. Seeded at **0**, not
    /// at the first entry, so the result is never negative even if every rate is.
    pub fn max(&self) -> i8 {
        self.iter().fold(0i8, i8::max)
    }
}

// ── Reading the image: the eFuse register path ───────────────────────────────

/// `enum mt76x02_eeprom_modes` — mt76x02_eeprom.h:121-124. Goes into
/// [`MT_EFUSE_CTRL_MODE`] (bits 7:6).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum EfuseMode {
    /// `MT_EE_READ` (0) — the *logical* image, i.e. the eFuse after the block
    /// remapping the hardware applies. This is what the parse wants.
    Read,
    /// `MT_EE_PHYSICAL_READ` (1) — raw cells, bypassing the remap. Used only to
    /// read the usage map in [`efuse_physical_size_check`].
    PhysicalRead,
}

impl EfuseMode {
    const fn code(self) -> u32 {
        match self {
            EfuseMode::Read => 0,
            EfuseMode::PhysicalRead => 1,
        }
    }
}

/// How long to wait for `MT_EFUSE_CTRL_KICK` to clear. Upstream polls for
/// 1000 ms (`mt76x02_eeprom.c:26`, `mt76_poll_msec(..., 1000)`); 200 ms is used
/// here because on USB each poll iteration already costs a 151 µs EP0 round trip,
/// so a stuck eFuse should fail fast rather than block bring-up for a second per
/// block × 32 blocks.
const EFUSE_KICK_TIMEOUT: Duration = Duration::from_millis(200);

/// Read one 16-byte eFuse block. Port of `mt76x02_efuse_read`,
/// `mt76x02_eeprom.c:11-43`.
///
/// Sequence, exactly upstream's: clear AIN+MODE in `MT_EFUSE_CTRL`, set
/// `AIN = addr & ~0xf` (blocks are 16-byte aligned) and `MODE`, set `KICK`, write;
/// poll `KICK` clear; re-read the control register; if all six `AOUT` bits are set
/// the block is unprogrammed and reads as 0xff; otherwise the four
/// `MT_EFUSE_DATA(n)` words are the block, little-endian.
///
/// Cost: 2 + (poll iterations) + 1 + 4 EP0 round trips ≈ 8 × 151 µs ≈ 1.2 ms per
/// block, so a full 512-byte image is ~39 ms. Bring-up only.
pub fn efuse_read_block(
    regs: &dyn Mt76Regs,
    addr: u16,
    mode: EfuseMode,
    out: &mut [u8; 16],
) -> Result<(), FaceError> {
    let mut val = regs.rr(MT_EFUSE_CTRL)?;
    val &= !(MT_EFUSE_CTRL_AIN | MT_EFUSE_CTRL_MODE);
    val |= field_prep(MT_EFUSE_CTRL_AIN, u32::from(addr) & !0xf);
    val |= field_prep(MT_EFUSE_CTRL_MODE, mode.code());
    val |= MT_EFUSE_CTRL_KICK;
    regs.wr(MT_EFUSE_CTRL, val)?;

    let deadline = Instant::now() + EFUSE_KICK_TIMEOUT;
    loop {
        if regs.rr(MT_EFUSE_CTRL)? & MT_EFUSE_CTRL_KICK == 0 {
            break;
        }
        if Instant::now() >= deadline {
            return Err(err(format!(
                "mt76x0 efuse: KICK never cleared for block {addr:#06x} (mode {mode:?}) \
                 within {EFUSE_KICK_TIMEOUT:?}"
            )));
        }
        std::thread::sleep(Duration::from_millis(1));
    }

    // Upstream does udelay(2) here (mt76x02_eeprom.c:29) before re-reading. On
    // USB the control read below is itself a 151 µs round trip — 75× the settle
    // time — so no explicit delay is inserted; the bus provides it.
    val = regs.rr(MT_EFUSE_CTRL)?;
    if val & MT_EFUSE_CTRL_AOUT == MT_EFUSE_CTRL_AOUT {
        // mt76x02_eeprom.c:32-35 — AOUT all-ones means "no such block".
        out.fill(0xff);
        return Ok(());
    }

    for i in 0..4u32 {
        let word = regs.rr(mt_efuse_data(i))?;
        let at = (i as usize) * 4;
        out[at..at + 4].copy_from_slice(&word.to_le_bytes());
    }
    Ok(())
}

/// `mt76x02_get_efuse_data` — mt76x02_eeprom.c:57-69. Fills `buf` in 16-byte
/// blocks starting at `base`; a trailing partial block is **not** read, exactly
/// as upstream (`for (i = 0; i + 16 <= len; i += 16)`).
pub fn efuse_get_data(
    regs: &dyn Mt76Regs,
    base: u16,
    buf: &mut [u8],
    mode: EfuseMode,
) -> Result<(), FaceError> {
    let mut i = 0usize;
    let mut block = [0u8; 16];
    while i + 16 <= buf.len() {
        efuse_read_block(regs, base + i as u16, mode, &mut block)?;
        buf[i..i + 16].copy_from_slice(&block);
        i += 16;
    }
    Ok(())
}

/// `mt76x0_efuse_physical_size_check` — mt76x0/eeprom.c:19-46.
///
/// Reads the eFuse **usage map** (`0x1e0..=0x1fc`, 29 bytes) in physical mode and
/// counts the run of unwritten (zero) cells. If fewer than 5 cells outside that
/// run have been programmed, upstream declares the part to be carrying the
/// factory default EEPROM and refuses to use it — a blank image would parse into
/// plausible-looking nonsense, which is worse than not coming up.
///
/// Returns the number of programmed cells on success, so a caller can log how
/// close to the threshold the part is.
pub fn efuse_physical_size_check(regs: &dyn Mt76Regs) -> Result<usize, FaceError> {
    let mut data = [0u8; MT_MAP_READS * 16];
    efuse_get_data(
        regs,
        MT_EE_USAGE_MAP_START as u16,
        &mut data,
        EfuseMode::PhysicalRead,
    )?;

    // Upstream initialises start/end to 0 and only assigns `start` on the first
    // zero byte (`if (!start)`), which works because MT_EE_USAGE_MAP_START is
    // itself non-zero. Kept literally, including the "no zero byte at all"
    // case, where cnt_free ends up 1 rather than 0.
    let (mut start, mut end) = (0usize, 0usize);
    for (i, &b) in data.iter().take(MT_EFUSE_USAGE_MAP_SIZE).enumerate() {
        if b == 0 {
            if start == 0 {
                start = MT_EE_USAGE_MAP_START + i;
            }
            end = MT_EE_USAGE_MAP_START + i;
        }
    }
    let cnt_free = end + 1 - start;
    let used = MT_EFUSE_USAGE_MAP_SIZE.saturating_sub(cnt_free);
    if used < 5 {
        return Err(err(format!(
            "mt76x0 efuse: driver does not support default EEPROM \
             ({used} of {MT_EFUSE_USAGE_MAP_SIZE} usage-map cells programmed, need 5) \
             — mt76x0/eeprom.c:39"
        )));
    }
    Ok(used)
}

/// Read the whole 512-byte image over the eFuse register path, **without**
/// [`efuse_physical_size_check`]. The escape hatch for bring-up: if the size
/// check rejects a part this still shows what the eFuse actually holds, which is
/// the first thing to look at when the check is the thing that is wrong.
pub fn read_efuse_image(regs: &dyn Mt76Regs) -> Result<[u8; MT76X0_EEPROM_SIZE], FaceError> {
    let mut img = [0u8; MT76X0_EEPROM_SIZE];
    efuse_get_data(regs, 0, &mut img, EfuseMode::Read)?;
    Ok(img)
}

// ── Reading the image: the USB EEPROM-shadow path ────────────────────────────

/// One `MT_VEND_READ_EEPROM` control-IN. Implemented by the backend that owns the
/// USB handle, so this module never touches libusb.
///
/// The transfer is `bmRequestType = 0xc0`, `bRequest = 0x09` (`mt76.h:637`),
/// `wValue = 0`, `wIndex = offset`, 4 bytes in, little-endian — the same shape
/// [`crate::Mt7612uBackend`] already uses (`src/mt7612/mod.rs:437-445`).
pub trait EepromShadow {
    /// Read the 4 bytes at byte `offset` of the EEPROM shadow.
    fn read_eeprom_dword(&self, offset: u16) -> Result<u32, FaceError>;
}

/// Adapter so a caller can pass a closure where an [`EepromShadow`] is wanted:
/// `ShadowFn(|off| backend.read_efuse(off))`. A blanket `impl<F: Fn(..)>` is
/// deliberately avoided — it would collide with any concrete backend impl.
pub struct ShadowFn<F>(pub F);

impl<F: Fn(u16) -> Result<u32, FaceError>> EepromShadow for ShadowFn<F> {
    fn read_eeprom_dword(&self, offset: u16) -> Result<u32, FaceError> {
        (self.0)(offset)
    }
}

/// Read the whole 512-byte image through the USB EEPROM shadow: 128 control-IN
/// round trips ≈ 19 ms at the MEASURED 151 µs/transfer. Bring-up only.
///
/// ★ Whether this path returns anything on the MT7610U is **OPEN** — see the
/// module header. Always validate the result with [`image_looks_valid`] before
/// trusting it; [`Mt76x0Eeprom::load`] does.
pub fn read_shadow_image(shadow: &dyn EepromShadow) -> Result<[u8; MT76X0_EEPROM_SIZE], FaceError> {
    let mut img = [0u8; MT76X0_EEPROM_SIZE];
    let mut off = 0usize;
    while off < MT76X0_EEPROM_SIZE {
        let word = shadow.read_eeprom_dword(off as u16)?;
        img[off..off + 4].copy_from_slice(&word.to_le_bytes());
        off += 4;
    }
    Ok(img)
}

/// `mt76x0_check_eeprom` — mt76x0/eeprom.c:273-291. The chip id at offset 0, or
/// at [`MT_EE_PCI_ID`] when offset 0 is zero, must be `0x7610` (MT7610) or
/// `0x7650` (MT7650). Anything else means the image is not this part's.
pub fn image_looks_valid(raw: &[u8]) -> bool {
    let le16 = |off: usize| -> u16 {
        u16::from_le_bytes([
            raw.get(off).copied().unwrap_or(0xff),
            raw.get(off + 1).copied().unwrap_or(0xff),
        ])
    };
    let mut val = le16(MT_EE_CHIP_ID);
    if val == 0 {
        val = le16(MT_EE_PCI_ID);
    }
    matches!(val, 0x7650 | 0x7610)
}

/// Which path an image came off the part by.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum EepromSource {
    /// `MT_VEND_READ_EEPROM` control transfers — the legacy/vendor path.
    UsbShadow,
    /// `MT_EFUSE_CTRL` register kicks — what mainline mt76 uses.
    Efuse,
}

// ── The parsed image ─────────────────────────────────────────────────────────

/// A parsed MT7610U EEPROM image.
///
/// Holds the raw 512 bytes plus the handful of fields whose decode is
/// non-obvious enough to be worth doing once (`mt76x0_eeprom_init` →
/// `mt76x0_set_freq_offset` / `mt76x0_set_temp_offset` / `mt76x0_read_rx_gain`,
/// `mt76x0/eeprom.c:312-347`). Everything else is read from the raw image on
/// demand through a named `MT_EE_*` offset — deliberately, so the field's
/// provenance is in the accessor rather than in a constructor a reader has to
/// scroll back to.
pub struct Mt76x0Eeprom {
    raw: [u8; MT76X0_EEPROM_SIZE],
    /// The image handed to [`parse`](Self::parse) was shorter than 512 bytes and
    /// the tail was filled with 0xff. Nothing rejects it — a short image is a
    /// bring-up reality, and 0xff is what an unprogrammed cell reads anyway — but
    /// the flag is exposed so a caller can say so in a log.
    short_image: bool,
    mac: [u8; 6],
    lna_2g: i8,
    lna_5g: [i8; 3],
    rssi_offset_2g: [i8; 2],
    rssi_offset_5g: [i8; 2],
    temp_offset: i16,
    freq_offset: u8,
    /// Last channel handed to [`set_channel`](Self::set_channel), used only to
    /// pick the 5 GHz LNA sub-band group in [`rssi_dbm`](Self::rssi_dbm), which
    /// gets a bare `is_5ghz` flag and cannot pick it otherwise. 0 = not yet set.
    cur_chan: AtomicU8,
}

impl Clone for Mt76x0Eeprom {
    fn clone(&self) -> Self {
        Self {
            raw: self.raw,
            short_image: self.short_image,
            mac: self.mac,
            lna_2g: self.lna_2g,
            lna_5g: self.lna_5g,
            rssi_offset_2g: self.rssi_offset_2g,
            rssi_offset_5g: self.rssi_offset_5g,
            temp_offset: self.temp_offset,
            freq_offset: self.freq_offset,
            cur_chan: AtomicU8::new(self.cur_chan.load(Ordering::Relaxed)),
        }
    }
}

impl std::fmt::Debug for Mt76x0Eeprom {
    /// The raw 512 bytes are omitted — [`raw`](Self::raw) is there for a hex dump.
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("Mt76x0Eeprom")
            .field("chip_id", &format_args!("{:#06x}", self.chip_id()))
            .field("version", &self.version())
            .field("fae", &self.fae())
            .field("mac", &format_args!("{:02x?}", self.mac))
            .field("board_type", &self.board_type())
            .field("tssi_enabled", &self.tssi_enabled())
            .field("ext_pa_2g", &self.ext_pa_enabled(false))
            .field("ext_pa_5g", &self.ext_pa_enabled(true))
            .field("lna_2g", &self.lna_2g)
            .field("lna_5g", &self.lna_5g)
            .field("rssi_offset_2g", &self.rssi_offset_2g)
            .field("rssi_offset_5g", &self.rssi_offset_5g)
            .field("temp_offset", &self.temp_offset)
            .field("freq_offset", &self.freq_offset)
            .field("short_image", &self.short_image)
            .finish()
    }
}

impl Mt76x0Eeprom {
    /// Image size this parse expects. See [`MT76X0_EEPROM_SIZE`].
    pub const SIZE: usize = MT76X0_EEPROM_SIZE;

    /// Parse a raw image. A slice shorter than 512 bytes is 0xff-padded (and
    /// flagged, see [`is_short_image`](Self::is_short_image)); a longer one is
    /// truncated. Infallible on purpose: this must be able to look at a bad dump.
    pub fn parse(raw: &[u8]) -> Self {
        let mut img = [0xffu8; MT76X0_EEPROM_SIZE];
        let n = raw.len().min(MT76X0_EEPROM_SIZE);
        img[..n].copy_from_slice(&raw[..n]);

        let mut ee = Self {
            raw: img,
            short_image: raw.len() < MT76X0_EEPROM_SIZE,
            mac: [0; 6],
            lna_2g: 0,
            lna_5g: [0; 3],
            rssi_offset_2g: [0; 2],
            rssi_offset_5g: [0; 2],
            temp_offset: 0,
            freq_offset: 0,
            cur_chan: AtomicU8::new(0),
        };

        // mt76x0/eeprom.c:333-334 — six bytes at MT_EE_MAC_ADDR, in wire order.
        ee.mac
            .copy_from_slice(&ee.raw[MT_EE_MAC_ADDR..MT_EE_MAC_ADDR + 6]);

        // mt76x02_get_rx_gain — mt76x02_eeprom.c:102-127.
        let lna = ee.word(MT_EE_LNA_GAIN);
        ee.lna_2g = (lna & 0xff) as u8 as i8;
        ee.lna_5g[0] = (lna >> 8) as u8 as i8;
        // The two upper-group 5 GHz LNA gains are the *high bytes* of the two
        // words nominally called RSSI_OFFSET_*_1 — i.e. MT_EE_LNA_GAIN_5GHZ_1
        // (0x049) and MT_EE_LNA_GAIN_5GHZ_2 (0x04d). mt76x02_eeprom.c:111-115.
        ee.lna_5g[1] = ee.byte(MT_EE_LNA_GAIN_5GHZ_1) as i8;
        ee.lna_5g[2] = ee.byte(MT_EE_LNA_GAIN_5GHZ_2) as i8;
        // Unprogrammed upper groups fall back to the low group. :117-121.
        if !field_valid(ee.lna_5g[1] as u8) {
            ee.lna_5g[1] = ee.lna_5g[0];
        }
        if !field_valid(ee.lna_5g[2] as u8) {
            ee.lna_5g[2] = ee.lna_5g[0];
        }

        // mt76x0_read_rx_gain — mt76x0/eeprom.c:110-128. One byte per RX chain,
        // and anything outside ±10 dB is treated as unprogrammed (:123-124).
        // The bound is written as upstream writes it (`val < -10 || val > 10`)
        // rather than as a range test, so the line diffs against eeprom.c:123.
        #[allow(clippy::manual_range_contains)]
        let clamp = |v: u8| -> i8 {
            let v = v as i8;
            if v < -10 || v > 10 { 0 } else { v }
        };
        let off2g = ee.word(MT_EE_RSSI_OFFSET_2G_0);
        ee.rssi_offset_2g = [clamp(off2g as u8), clamp((off2g >> 8) as u8)];
        let off5g = ee.word(MT_EE_RSSI_OFFSET_5G_0);
        ee.rssi_offset_5g = [clamp(off5g as u8), clamp((off5g >> 8) as u8)];

        // mt76x0_set_temp_offset — mt76x0/eeprom.c:82-91. Default is -10, not 0.
        let temp = ee.byte(MT_EE_TEMP_OFFSET);
        ee.temp_offset = if field_valid(temp) {
            sign_extend(u32::from(temp), 8) as i16
        } else {
            -10
        };

        // mt76x0_set_freq_offset — mt76x0/eeprom.c:93-108. Note the subtraction is
        // on a u8 and upstream lets it wrap; kept (wrapping_sub) rather than
        // clamped, because the consumer at mt76x0/phy.c:1190-1191 caps it at 0xbf
        // on the way into the RF register anyway.
        let mut fo = ee.byte(MT_EE_FREQ_OFFSET);
        if !field_valid(fo) {
            fo = 0;
        }
        let mut comp = ee.byte(MT_EE_FREQ_OFFSET_COMPENSATION);
        if !field_valid(comp) {
            comp = 0;
        }
        ee.freq_offset = fo.wrapping_sub(sign_extend(u32::from(comp), 8) as u8);

        ee
    }

    /// Read the image off the part and parse it.
    ///
    /// Tries `shadow` first **only if it yields an image that passes
    /// [`image_looks_valid`]**, then falls back to the eFuse register path
    /// (`efuse_physical_size_check` + `mt76x02_get_efuse_data`,
    /// `mt76x0/eeprom.c:293-310`), which is what mainline actually uses. Warns —
    /// does not fail — on an EEPROM layout version above
    /// [`MT76X0U_EE_MAX_VER`], matching `mt76x0/eeprom.c:326-329`.
    pub fn load(
        regs: &dyn Mt76Regs,
        shadow: Option<&dyn EepromShadow>,
    ) -> Result<(Self, EepromSource), FaceError> {
        if let Some(sh) = shadow {
            match read_shadow_image(sh) {
                Ok(img) if image_looks_valid(&img) => {
                    let ee = Self::parse(&img);
                    ee.warn_on_version();
                    return Ok((ee, EepromSource::UsbShadow));
                }
                Ok(img) => tracing::debug!(
                    chip_id = format_args!("{:#06x}", u16::from_le_bytes([img[0], img[1]])),
                    "mt76x0 eeprom: USB shadow did not look like an MT7610/7650 image, \
                     falling back to efuse"
                ),
                Err(e) => tracing::debug!(
                    error = %e,
                    "mt76x0 eeprom: USB shadow read failed, falling back to efuse"
                ),
            }
        }

        let used = efuse_physical_size_check(regs)?;
        let img = read_efuse_image(regs)?;
        tracing::debug!(
            usage_map_cells = used,
            valid = image_looks_valid(&img),
            "mt76x0 eeprom: read 512 B over the efuse register path"
        );
        let ee = Self::parse(&img);
        ee.warn_on_version();
        Ok((ee, EepromSource::Efuse))
    }

    fn warn_on_version(&self) {
        if self.version() > MT76X0U_EE_MAX_VER {
            tracing::warn!(
                version = self.version(),
                max = MT76X0U_EE_MAX_VER,
                "mt76x0 eeprom: unsupported EEPROM version (mt76x0/eeprom.c:326)"
            );
        }
    }

    // ── Raw access ───────────────────────────────────────────────────────────

    /// The raw 512-byte image, for a hex dump or a diff against silicon.
    pub fn raw(&self) -> &[u8; MT76X0_EEPROM_SIZE] {
        &self.raw
    }

    /// See [`short_image`](Self#structfield.short_image).
    pub fn is_short_image(&self) -> bool {
        self.short_image
    }

    /// One byte of the image. Out of range reads 0xff — the unprogrammed-cell
    /// value — rather than panicking, so a truncated dump cannot take down a
    /// radio thread.
    pub fn byte(&self, off: usize) -> u8 {
        self.raw.get(off).copied().unwrap_or(0xff)
    }

    /// `mt76x02_eeprom_get` — mt76x02_eeprom.h:162-170. Little-endian u16.
    ///
    /// Upstream returns -1 for an odd offset; here an odd offset simply reads the
    /// unaligned pair, which no caller in this port does (the odd `MT_EE_*`
    /// fields are all reached through [`byte`](Self::byte) instead).
    pub fn word(&self, off: usize) -> u16 {
        u16::from_le_bytes([self.byte(off), self.byte(off + 1)])
    }

    /// `mt76x02_eeprom_copy` — mt76x02_eeprom.c:45-54.
    pub fn copy_from(&self, off: usize, dest: &mut [u8]) -> Result<(), FaceError> {
        if off + dest.len() > MT76X0_EEPROM_SIZE {
            return Err(err(format!(
                "mt76x0 eeprom: copy of {} B at {off:#05x} runs past the {MT76X0_EEPROM_SIZE} B image",
                dest.len()
            )));
        }
        dest.copy_from_slice(&self.raw[off..off + dest.len()]);
        Ok(())
    }
}

// ── Identity, capability and board configuration ─────────────────────────────

impl Mt76x0Eeprom {
    /// `MT_EE_CHIP_ID` — 0x7610 on this part, 0x7650 on the MT7650 sibling.
    pub fn chip_id(&self) -> u16 {
        self.word(MT_EE_CHIP_ID)
    }

    /// `MT_EE_PCI_ID` — the fallback identity when [`chip_id`](Self::chip_id) is
    /// zero. `mt76x0/eeprom.c:277-280`.
    pub fn pci_id(&self) -> u16 {
        self.word(MT_EE_PCI_ID)
    }

    /// EEPROM layout version — the **high** byte of `MT_EE_VERSION`.
    /// `mt76x0/eeprom.c:322-324`.
    pub fn version(&self) -> u8 {
        (self.word(MT_EE_VERSION) >> 8) as u8
    }

    /// FAE (factory) revision — the **low** byte of `MT_EE_VERSION`. Logged only.
    pub fn fae(&self) -> u8 {
        self.word(MT_EE_VERSION) as u8
    }

    /// Factory MAC address, `MT_EE_MAC_ADDR`. `mt76x0/eeprom.c:333`.
    pub fn mac_addr(&self) -> [u8; 6] {
        self.mac
    }

    pub fn nic_conf0(&self) -> u16 {
        self.word(MT_EE_NIC_CONF_0)
    }

    pub fn nic_conf1(&self) -> u16 {
        self.word(MT_EE_NIC_CONF_1)
    }

    pub fn nic_conf2(&self) -> u16 {
        self.word(MT_EE_NIC_CONF_2)
    }

    /// `MT_EE_NIC_CONF_1` with an unprogrammed low byte zeroed —
    /// `mt76x0_set_chip_cap`, `mt76x0/eeprom.c:67-68`. The low byte carries the
    /// HW-RF-control and external-LNA bits, and 0x00/0xff there means "not
    /// programmed", not "all features present".
    pub fn nic_conf1_sanitised(&self) -> u16 {
        let v = self.nic_conf1();
        if field_valid((v & 0xff) as u8) {
            v
        } else {
            v & 0xff00
        }
    }

    /// `mt76x02_eeprom_parse_hw_cap` — mt76x02_eeprom.c:72-88.
    pub fn board_type(&self) -> BoardType {
        let field = (self.nic_conf0() & MT_EE_NIC_CONF_0_BOARD_TYPE) >> 12;
        match field {
            1 => BoardType::Band2GhzOnly,
            2 => BoardType::Band5GhzOnly,
            other => BoardType::DualBand(other),
        }
    }

    /// RX chain count, `MT_EE_NIC_CONF_0_RX_PATH`. Upstream flags anything > 1 as
    /// an invalid stream count on this 1×1 part (`mt76x0/eeprom.c:77-79`).
    pub fn rx_path(&self) -> u8 {
        (self.nic_conf0() & MT_EE_NIC_CONF_0_RX_PATH) as u8
    }

    /// TX chain count, `MT_EE_NIC_CONF_0_TX_PATH`. See [`rx_path`](Self::rx_path).
    pub fn tx_path(&self) -> u8 {
        ((self.nic_conf0() & MT_EE_NIC_CONF_0_TX_PATH) >> 4) as u8
    }

    /// Whether the stream counts are declared at all — upstream skips the
    /// tx/rx-path sanity check when the high byte of NIC_CONF_0 is unprogrammed.
    /// `mt76x0/eeprom.c:74-75`.
    pub fn stream_counts_valid(&self) -> bool {
        field_valid((self.nic_conf0() >> 8) as u8)
    }

    /// `mt76x02_ext_pa_enabled` — mt76x02_eeprom.c:91-100. ⚠ The bit is
    /// `PA_INT_*`: it is set when the PA is **internal**, so an external PA is the
    /// bit being *clear*. [`super::initvals_phy::RF_EXT_PA_TAB`] is applied on a
    /// channel change only when this is true (`mt76x0/phy.c:362`).
    pub fn ext_pa_enabled(&self, is_5ghz: bool) -> bool {
        let bit = if is_5ghz {
            MT_EE_NIC_CONF_0_PA_INT_5G
        } else {
            MT_EE_NIC_CONF_0_PA_INT_2G
        };
        self.nic_conf0() & bit == 0
    }

    /// `mt76x0_tssi_enabled` — mt76x0/eeprom.h:35-39. Gates both TX-power paths;
    /// see [`MT_EE_NIC_CONF_1_TX_ALC_EN`].
    pub fn tssi_enabled(&self) -> bool {
        self.nic_conf1() & MT_EE_NIC_CONF_1_TX_ALC_EN != 0
    }

    /// `MT_EE_NIC_CONF_1_HW_RF_CTRL` — a board whose RF is driven by hardware
    /// pins. Upstream logs "driver does not support HW RF ctrl" and carries on
    /// (`mt76x0/eeprom.c:70-72`); nothing here acts on it either.
    pub fn hw_rf_ctrl(&self) -> bool {
        self.nic_conf1_sanitised() & MT_EE_NIC_CONF_1_HW_RF_CTRL != 0
    }

    /// Temperature-compensated ALC, `MT_EE_NIC_CONF_1_TEMP_TX_ALC`.
    pub fn temp_tx_alc(&self) -> bool {
        self.nic_conf1_sanitised() & MT_EE_NIC_CONF_1_TEMP_TX_ALC != 0
    }

    /// External LNA present for the band (`MT_EE_NIC_CONF_1_LNA_EXT_2G/5G`).
    pub fn ext_lna_enabled(&self, is_5ghz: bool) -> bool {
        let bit = if is_5ghz {
            MT_EE_NIC_CONF_1_LNA_EXT_5G
        } else {
            MT_EE_NIC_CONF_1_LNA_EXT_2G
        };
        self.nic_conf1_sanitised() & bit != 0
    }

    /// `MT_EE_ANTENNA`, written into `MT_CMB_CTRL` by `mt76x0_phy_ant_select`
    /// (`mt76x0/phy.c:428,465`) after bits 14 and 12 are cleared.
    pub fn antenna(&self) -> u16 {
        self.word(MT_EE_ANTENNA)
    }

    /// Dual-antenna board, `MT_EE_ANTENNA_DUAL`. `mt76x0/phy.c:441`.
    pub fn antenna_dual(&self) -> bool {
        self.antenna() & MT_EE_ANTENNA_DUAL != 0
    }

    /// `MT_EE_CFG1_INIT`, written straight into `MT_CSR_EE_CFG1`.
    /// `mt76x0/phy.c:429,466`.
    pub fn cfg1_init(&self) -> u16 {
        self.word(MT_EE_CFG1_INIT)
    }

    /// Antenna diversity is on iff `!ANT_OPT && ANT_DIV`. `mt76x0/phy.c:443-444`.
    pub fn antenna_diversity(&self) -> bool {
        let c2 = self.nic_conf2();
        c2 & MT_EE_NIC_CONF_2_ANT_OPT == 0 && c2 & MT_EE_NIC_CONF_2_ANT_DIV != 0
    }

    /// `MT_EE_NIC_CONF_2_XTAL_OPTION`, bits 10:9.
    pub fn xtal_option(&self) -> u8 {
        ((self.nic_conf2() & MT_EE_NIC_CONF_2_XTAL_OPTION) >> 9) as u8
    }

    /// Regulatory region bytes: 5 GHz then 2.4 GHz. Read for completeness — this
    /// driver does no regulatory enforcement of its own.
    pub fn country_region(&self) -> (u8, u8) {
        (
            self.byte(MT_EE_COUNTRY_REGION_5GHZ),
            self.byte(MT_EE_COUNTRY_REGION_2GHZ),
        )
    }
}

// ── RX gain: LNA, RSSI offset, and the dBm conversion ────────────────────────

impl Mt76x0Eeprom {
    /// Tell the parse which channel the radio is on, so [`rssi_dbm`](Self::rssi_dbm)
    /// can pick the right 5 GHz LNA group. Interior mutability (`&self`) because
    /// the EEPROM is shared read-only behind the backend while RX runs.
    pub fn set_channel(&self, chan: u8) {
        self.cur_chan.store(chan, Ordering::Relaxed);
    }

    /// The channel last given to [`set_channel`](Self::set_channel); 0 if none.
    pub fn current_channel(&self) -> u8 {
        self.cur_chan.load(Ordering::Relaxed)
    }

    /// 2.4 GHz LNA gain byte (`MT_EE_LNA_GAIN` low). `mt76x02_eeprom.c:108`.
    pub fn lna_gain_2g(&self) -> i8 {
        self.lna_2g
    }

    /// The three 5 GHz LNA gains — low (≤ ch 64), middle (≤ ch 128), high.
    /// `mt76x02_eeprom.c:109-121`.
    pub fn lna_gain_5g(&self) -> [i8; 3] {
        self.lna_5g
    }

    /// `mt76x02_get_lna_gain` — mt76x02_eeprom.c:130-146. Picks the band and, in
    /// 5 GHz, the sub-band group by channel; an unprogrammed (0xff) gain reads 0.
    pub fn lna_gain(&self, chan: u8) -> i8 {
        let lna = if !is_5ghz_channel(chan) {
            self.lna_2g
        } else if chan <= 64 {
            self.lna_5g[0]
        } else if chan <= 128 {
            self.lna_5g[1]
        } else {
            self.lna_5g[2]
        };
        if lna as u8 == 0xff { 0 } else { lna }
    }

    /// Per-RX-chain RSSI offset for the band, already clamped to ±10 dB.
    /// `mt76x0_read_rx_gain`, mt76x0/eeprom.c:121-127.
    pub fn rssi_offset(&self, is_5ghz: bool) -> [i8; 2] {
        if is_5ghz {
            self.rssi_offset_5g
        } else {
            self.rssi_offset_2g
        }
    }

    /// Convert a raw RXWI RSSI byte to dBm.
    ///
    /// ★ **The formula** — `mt76x02_mac_get_rssi`, mt76x02_mac.c:760-769,
    /// reached from `mt76x02_mac_process_rx` at mt76x02_mac.c:861:
    ///
    /// ```text
    /// signal = (s8)rxwi.rssi[chain] + cal.rx.rssi_offset[chain] - cal.rx.lna_gain
    /// ```
    ///
    /// where `rssi_offset` is the per-chain byte pair for the band (clamped to
    /// ±10 dB at parse time, `mt76x0/eeprom.c:123-124`) and `lna_gain` is the
    /// band/sub-band LNA gain from [`lna_gain`](Self::lna_gain). The result is
    /// dBm directly — there is no scaling step.
    ///
    /// ⚠ `raw` is declared `u8` in `struct mt76x02_rxwi` (mt76x02_mac.h:105) but
    /// is **signed**: upstream passes it into an `s8` parameter. It is reinterpreted
    /// as `i8` here.
    ///
    /// ⚠ Contract note: the third parameter is the **RX chain index**, not a
    /// bandwidth index — `rssi_offset[]` is indexed by chain
    /// (`MT_MAX_CHAINS`), never by bandwidth. On this 1×1 part only chain 0 is
    /// populated; chains ≥ 2 have no offset byte and read 0.
    ///
    /// The 5 GHz LNA group comes from [`set_channel`](Self::set_channel); with no
    /// channel set it defaults to the low group (`lna_5g[0]`), which is what a
    /// channel of 0 selects in [`lna_gain`](Self::lna_gain) anyway.
    pub fn rssi_dbm(&self, raw: u8, is_5ghz: bool, chain: u8) -> i8 {
        let chan = self.cur_chan.load(Ordering::Relaxed);
        let chan = match (is_5ghz, is_5ghz_channel(chan)) {
            // Cached channel agrees with the caller's band — use it.
            (true, true) | (false, false) => chan,
            // It does not (or none is set): fall back to a representative channel
            // for the requested band, so the band pick is never wrong.
            (true, false) => 36,
            (false, true) => 6,
        };
        self.rssi_dbm_on_channel(raw, chan, chain)
    }

    /// [`rssi_dbm`](Self::rssi_dbm) with the channel given explicitly — the exact
    /// form, with no dependence on the cached channel.
    pub fn rssi_dbm_on_channel(&self, raw: u8, chan: u8, chain: u8) -> i8 {
        let offsets = self.rssi_offset(is_5ghz_channel(chan));
        let offset = offsets.get(chain as usize).copied().unwrap_or(0);
        // Upstream's s8 arithmetic wraps; done in i16 and clamped here, because a
        // wrapped RSSI feeds mcs_for_rssi and would pick a rate for a signal that
        // does not exist. With real values (|raw| ≤ 128, |offset| ≤ 10,
        // |lna| ≤ 127) the clamp is unreachable in the negative direction.
        let signal = i16::from(raw as i8) + i16::from(offset) - i16::from(self.lna_gain(chan));
        signal.clamp(i16::from(i8::MIN), i16::from(i8::MAX)) as i8
    }

    /// Temperature offset in °C, `MT_EE_TEMP_OFFSET`, defaulting to -10 when
    /// unprogrammed. Consumed by the temperature sensor read at
    /// `mt76x0/phy.c:1038`: `temp = (35 * (val - temp_offset)) / 10 + 25`.
    pub fn temp_offset(&self) -> i16 {
        self.temp_offset
    }

    /// Crystal frequency trim, `MT_EE_FREQ_OFFSET` minus the sign-magnitude
    /// compensation byte at `MT_EE_FREQ_OFFSET_COMPENSATION`
    /// (`mt76x0/eeprom.c:93-108`). Written to `MT_RF(0, 22)` capped at 0xbf
    /// (`mt76x0/phy.c:1190-1191`) — the cap belongs to the PHY, not here.
    pub fn freq_offset(&self) -> u8 {
        self.freq_offset
    }
}

// ── TX power ─────────────────────────────────────────────────────────────────

/// `struct mt76x0_chan_map` — mt76x0/eeprom.c:203-214. Maps a channel to a byte
/// offset into the per-channel-group target-power array for its band. The first
/// seven rows are 2.4 GHz, the rest 5 GHz; the lookup is `chan <= row.chan`, and
/// an exact hit selects the array's **high** byte instead of the low one.
const CHAN_MAP: &[(u8, u8)] = &[
    (2, 0),
    (4, 2),
    (6, 4),
    (8, 6),
    (10, 8),
    (12, 10),
    (14, 12),
    (38, 0),
    (44, 2),
    (48, 4),
    (54, 6),
    (60, 8),
    (64, 10),
    (102, 12),
    (108, 14),
    (112, 16),
    (118, 18),
    (124, 20),
    (128, 22),
    (134, 24),
    (140, 26),
    (151, 28),
    (157, 30),
    (161, 32),
    (167, 34),
    (171, 36),
    (175, 38),
];

impl Mt76x0Eeprom {
    /// `mt76x0_get_delta` — mt76x0/eeprom.c:130-150. The bandwidth power delta in
    /// 0.5 dB steps: 80 MHz from the high byte of `MT_EE_5G_TARGET_POWER`,
    /// 40 MHz from `MT_EE_TX_POWER_DELTA_BW40` (low byte 2.4 GHz, high byte
    /// 5 GHz), and **zero for 20 MHz**.
    ///
    /// The two narrowband widths this HAL can name (`Nb10`, `Nb5`) have no
    /// EEPROM delta and take the 20 MHz path — upstream has no case for them.
    pub fn power_delta(&self, chan: u8, bw: Bandwidth) -> i8 {
        let val = match bw {
            Bandwidth::Bw80 => (self.word(MT_EE_5G_TARGET_POWER) >> 8) as u8,
            Bandwidth::Bw40 => {
                let data = self.word(MT_EE_TX_POWER_DELTA_BW40);
                if is_5ghz_channel(chan) {
                    (data >> 8) as u8
                } else {
                    data as u8
                }
            }
            Bandwidth::Bw20 | Bandwidth::Nb10 | Bandwidth::Nb5 => return 0,
        };
        rate_power_val(val)
    }

    /// Per-rate TX power for `chan` at 20 MHz. See
    /// [`tx_power_per_rate_bw`](Self::tx_power_per_rate_bw) for the full form and
    /// for what the numbers mean.
    pub fn tx_power_per_rate(&self, chan: u8) -> RatePower {
        self.tx_power_per_rate_bw(chan, Bandwidth::Bw20)
    }

    /// `mt76x0_get_tx_power_per_rate` — mt76x0/eeprom.c:152-198.
    ///
    /// Returns a [`RatePower`]: per-rate power in **0.5 dB steps**, *relative* to
    /// the channel's target power. The absolute value of a rate is
    /// `target_power_half_db(chan, bw) + rate_entry`, still in half-dB; divide by
    /// 2 for dBm, or use [`target_power_dbm`](Self::target_power_dbm) for the
    /// ceiling.
    ///
    /// Layout — five words per band, each byte a 6-bit two's complement value
    /// ([`s6_to_s8`]) covering **two** adjacent rates:
    ///
    /// | rates | 2.4 GHz | 5 GHz |
    /// |---|---|---|
    /// | CCK 1/2, 5.5/11 | `MT_EE_TX_POWER_BYRATE_BASE` (0x0de) | same word |
    /// | OFDM 6/9, 12/18 | +2 (0x0e0) | 0x120 |
    /// | OFDM 24/36, 48/54 | +4 (0x0e2) | 0x122 |
    /// | HT/VHT MCS 0/1, 2/3 | +6 (0x0e4) | 0x124 |
    /// | HT/VHT MCS 4/5, 6/7 | +8 (0x0e6) | 0x126 |
    /// | VHT MCS 8, 9 | — | 0x12c |
    ///
    /// ⚠ The CCK word is read from the 2.4 GHz base **in both bands**
    /// (`mt76x0/eeprom.c:163` has no `is_2ghz` selector), which is harmless
    /// because CCK does not exist on 5 GHz but does mean the 5 GHz `cck` entries
    /// carry 2.4 GHz values. Ported as-is.
    ///
    /// Finally the bandwidth delta is added to **all 30** entries, and only when
    /// TSSI is *disabled* (`mt76x0/eeprom.c:196-197`) — with TSSI on, the closed
    /// loop applies its own correction and a static delta would double-count.
    pub fn tx_power_per_rate_bw(&self, chan: u8, bw: Bandwidth) -> RatePower {
        let is_2ghz = !is_5ghz_channel(chan);
        let mut t = RatePower::default();

        // cck 1M, 2M, 5.5M, 11M — mt76x0/eeprom.c:162-165.
        let val = u32::from(self.word(MT_EE_TX_POWER_BYRATE_BASE));
        t.cck[0] = s6_to_s8(val);
        t.cck[1] = t.cck[0];
        t.cck[2] = s6_to_s8(val >> 8);
        t.cck[3] = t.cck[2];

        let addr_for = |off_2g: usize, addr_5g: usize| -> usize {
            if is_2ghz {
                MT_EE_TX_POWER_BYRATE_BASE + off_2g
            } else {
                addr_5g
            }
        };

        // ofdm 6M, 9M, 12M, 18M — :167-171.
        let val = u32::from(self.word(addr_for(2, MT_EE_TX_POWER_BYRATE_5G)));
        t.ofdm[0] = s6_to_s8(val);
        t.ofdm[1] = t.ofdm[0];
        t.ofdm[2] = s6_to_s8(val >> 8);
        t.ofdm[3] = t.ofdm[2];

        // ofdm 24M, 36M, 48M, 54M — :173-177.
        let val = u32::from(self.word(addr_for(4, MT_EE_TX_POWER_BYRATE_5G + 2)));
        t.ofdm[4] = s6_to_s8(val);
        t.ofdm[5] = t.ofdm[4];
        t.ofdm[6] = s6_to_s8(val >> 8);
        t.ofdm[7] = t.ofdm[6];

        // ht-vht mcs 1ss 0, 1, 2, 3 — :179-183.
        let val = u32::from(self.word(addr_for(6, MT_EE_TX_POWER_BYRATE_5G + 4)));
        t.ht[0] = s6_to_s8(val);
        t.ht[1] = t.ht[0];
        t.ht[2] = s6_to_s8(val >> 8);
        t.ht[3] = t.ht[2];

        // ht-vht mcs 1ss 4, 5, 6 (and 7 — upstream's comment stops at 6) — :185-189.
        let val = u32::from(self.word(addr_for(8, MT_EE_TX_POWER_BYRATE_5G + 6)));
        t.ht[4] = s6_to_s8(val);
        t.ht[5] = t.ht[4];
        t.ht[6] = s6_to_s8(val >> 8);
        t.ht[7] = t.ht[6];

        // vht mcs 8, 9 5GHz — :191-194. Read unconditionally, as upstream does.
        let val = u32::from(self.word(MT_EE_TX_POWER_BYRATE_VHT_5G));
        t.vht[0] = s6_to_s8(val);
        t.vht[1] = s6_to_s8(val >> 8);

        let delta = if self.tssi_enabled() {
            0
        } else {
            self.power_delta(chan, bw)
        };
        t.add_offset(delta);
        t
    }

    /// `mt76x0_get_power_info` — mt76x0/eeprom.c:200-271. The channel's **target
    /// power** in 0.5 dB steps: the value written to `MT_TX_ALC_CFG_0`
    /// `CH_INIT_0/1` (6 bits) and added as a flat offset to the whole per-rate
    /// table (`mt76x0_phy_set_txpower`, `mt76x0/phy.c:844-859`).
    ///
    /// Two completely different sources depending on
    /// [`tssi_enabled`](Self::tssi_enabled):
    ///
    /// * **TSSI on** — `target = (2G/5G_TARGET_POWER & 0xff) - rate.ofdm[7]`, then
    ///   plus the bandwidth delta (:219-230). The `ofdm[7]` term makes this
    ///   depend on the rate table for the *same* channel and bandwidth, which is
    ///   why it is recomputed here rather than taken as a parameter; upstream gets
    ///   the same coupling by calling `get_tx_power_per_rate` immediately before
    ///   (`phy.c:849-850`).
    /// * **TSSI off** — a per-channel-group byte from the array based at
    ///   [`MT_EE_TX_POWER_DELTA_BW80`] (2.4 GHz) or
    ///   [`MT_EE_TX_POWER_0_GRP4_TSSI_SLOPE`]` + 2` (5 GHz), selected through
    ///   [`CHAN_MAP`]; out-of-range (`< 0 || > 0x3f`) falls back to **5**
    ///   (:269-270). No bandwidth delta is added on this path — the delta is
    ///   already in the per-rate table.
    ///
    /// ⚠ Upstream's 5 GHz `switch` at :245-263 overrides the offset for channels
    /// 42/58/106/122/155. Every one of those overrides reproduces the value
    /// [`CHAN_MAP`] already yields, so the switch is a no-op; it is kept because
    /// its *reason* is undetermined (probably a defensive restatement for the
    /// 80 MHz centre channels) and dropping it would be inventing.
    pub fn target_power_half_db(&self, chan: u8, bw: Bandwidth) -> i8 {
        let is_5ghz = is_5ghz_channel(chan);

        if self.tssi_enabled() {
            let data = self.word(if is_5ghz {
                MT_EE_5G_TARGET_POWER
            } else {
                MT_EE_2G_TARGET_POWER
            });
            let ofdm7 = self.tx_power_per_rate_bw(chan, bw).ofdm[7];
            // C truncates both of these into an s8; wrapping keeps that exactly.
            let target = ((data & 0xff) as i16 - i16::from(ofdm7)) as i8;
            return target.wrapping_add(self.power_delta(chan, bw));
        }

        let (idx, mut offset) = CHAN_MAP
            .iter()
            .find(|&&(c, _)| chan <= c)
            .map_or((0u32, CHAN_MAP[0].1), |&(c, off)| {
                (u32::from(chan == c), off)
            });

        let addr = if !is_5ghz {
            MT_EE_TX_POWER_DELTA_BW80 + usize::from(offset)
        } else {
            // mt76x0/eeprom.c:245-263 — see the ⚠ note above.
            match chan {
                42 => offset = 2,
                58 => offset = 8,
                106 => offset = 14,
                122 => offset = 20,
                155 => offset = 30,
                _ => {}
            }
            MT_EE_TX_POWER_0_GRP4_TSSI_SLOPE + 2 + usize::from(offset)
        };

        let data = self.word(addr);
        let tp = (data >> (8 * idx)) as u8 as i8;
        // Kept in upstream's form (`*tp < 0 || *tp > 0x3f`) — eeprom.c:269.
        #[allow(clippy::manual_range_contains)]
        let out = if tp < 0 || tp > 0x3f { 5 } else { tp };
        out
    }

    /// Absolute TX-power ceiling for `chan` at 20 MHz, in **dBm**.
    ///
    /// This is the one accessor in the module that leaves the 0.5 dB domain. It
    /// reproduces the chain upstream reports to userspace: take the per-rate
    /// table, add the channel's target power to every entry
    /// (`mt76x0_phy_set_txpower`, `mt76x0/phy.c:852`), take the maximum
    /// (`mt76x02_get_max_rate_power`, `mt76x02_phy.c:62`), and halve it with
    /// round-up (`mt76_get_txpower`, `mac80211.c:1809`). The antenna-path delta in
    /// that expression is `mt76_tx_power_path_delta(1) == 0` on this 1×1 part
    /// (`mt76.h:1506-1512`), so it drops out.
    pub fn target_power_dbm(&self, chan: u8) -> i8 {
        self.max_rate_power_half_db(chan, Bandwidth::Bw20)
            .saturating_add(1)
            / 2
    }

    /// The strongest rate's absolute power for `chan`/`bw`, in 0.5 dB steps —
    /// i.e. `target_power_half_db + max(rate table)`, the `txpower_cur` upstream
    /// caches at `mt76x0/phy.c:854`.
    pub fn max_rate_power_half_db(&self, chan: u8, bw: Bandwidth) -> i8 {
        let mut t = self.tx_power_per_rate_bw(chan, bw);
        t.add_offset(self.target_power_half_db(chan, bw));
        t.max()
    }
}

// ── TSSI calibration inputs (consumed by the PHY, parsed here) ───────────────

impl Mt76x0Eeprom {
    /// The seven 5 GHz TSSI channel bounds, `MT_EE_TSSI_BOUND1`..0x0da, read as a
    /// byte array by `mt76x02_eeprom_copy` at `mt76x0/phy.c:711-713`.
    pub fn tssi_bounds_5g(&self) -> [u8; 7] {
        let mut out = [0u8; 7];
        for (i, b) in out.iter_mut().enumerate() {
            *b = self.byte(MT_EE_TSSI_BOUND1 + i);
        }
        out
    }

    /// TSSI slope and offset for `chan` — the EEPROM half of
    /// `mt76x0_phy_get_delta_power`, `mt76x0/phy.c:707-733`. Returns
    /// `(slope, offset)`; the PHY does the rest of that arithmetic.
    ///
    /// 5 GHz picks a slope word by walking [`tssi_bounds_5g`](Self::tssi_bounds_5g)
    /// until `chan <= bound[i]` or the bound is 0.
    ///
    /// ⚠ Two upstream oddities, both ported rather than corrected:
    ///   1. If no bound matches, `i` ends at 7 and the read lands at
    ///      `MT_EE_TSSI_SLOPE_5G + 14` = 0x0fe, which is
    ///      [`MT_EE_RF_5G_GRP4_5_RX_HIGH_GAIN`] — one word past the slope array.
    ///   2. The 5 GHz offset is biased down by 256 when it is in 64..=127 **or**
    ///      has bit 7 set (`phy.c:723-725`) — the 64..=127 half of that condition
    ///      maps a positive byte to a large negative number and its reason is
    ///      undetermined. The 2.4 GHz path (`phy.c:729-731`) uses only bit 7.
    pub fn tssi_slope_offset(&self, chan: u8) -> (u8, i16) {
        let (val, offset) = if is_5ghz_channel(chan) {
            let bounds = self.tssi_bounds_5g();
            let mut i = bounds.len();
            for (n, &b) in bounds.iter().enumerate() {
                if chan <= b || b == 0 {
                    i = n;
                    break;
                }
            }
            let val = self.word(MT_EE_TSSI_SLOPE_5G + i * 2);
            let mut off = i32::from(val >> 8);
            if (64..=127).contains(&off) || off & 0x80 != 0 {
                off -= 256;
            }
            (val, off)
        } else {
            let val = self.word(MT_EE_TSSI_SLOPE_2G);
            let mut off = i32::from(val >> 8);
            if off & 0x80 != 0 {
                off -= 256;
            }
            (val, off)
        };
        ((val & 0xff) as u8, offset as i16)
    }
}

// ── Tests ────────────────────────────────────────────────────────────────────
//
// Everything here runs with **no dongle attached**. The point is that when the
// MT7610U's real image is finally dumped it can be diffed against a parse whose
// behaviour is already pinned: a synthetic 512-byte image with a distinct value
// in every field, plus a mock `Mt76Regs` that implements the eFuse kick protocol
// so the read path itself is exercised, not just the byte arithmetic.

#[cfg(test)]
mod tests {
    use super::*;
    use crate::mt76::regs::{MT_EFUSE_DATA_BASE, field_get};
    use std::sync::Mutex;

    fn put16(img: &mut [u8; MT76X0_EEPROM_SIZE], off: usize, v: u16) {
        img[off..off + 2].copy_from_slice(&v.to_le_bytes());
    }

    /// A 512-byte image with a distinguishable value in every field this module
    /// parses. TSSI is **off** (NIC_CONF_1 bit 13 clear).
    fn synthetic_image() -> [u8; MT76X0_EEPROM_SIZE] {
        let mut img = [0xffu8; MT76X0_EEPROM_SIZE];

        put16(&mut img, MT_EE_CHIP_ID, 0x7610);
        put16(&mut img, MT_EE_VERSION, 0x0c03); // version 0x0c, fae 0x03
        img[MT_EE_MAC_ADDR..MT_EE_MAC_ADDR + 6].copy_from_slice(&[0, 0x0c, 0x43, 0x76, 0x10, 0x01]);
        put16(&mut img, MT_EE_PCI_ID, 0x7610);
        put16(&mut img, MT_EE_ANTENNA, 0x8001); // MT_EE_ANTENNA_DUAL | 1
        put16(&mut img, MT_EE_CFG1_INIT, 0x1234);

        // rx_path 1, tx_path 1, PA_INT_2G set (internal 2.4 GHz PA),
        // PA_INT_5G clear (external 5 GHz PA), board type 0 => dual band.
        put16(&mut img, MT_EE_NIC_CONF_0, 0x0111);
        // TSSI off; LNA_EXT_2G set so the low byte is "programmed".
        put16(&mut img, MT_EE_NIC_CONF_1, 0x0004);
        // ANT_DIV set, ANT_OPT clear => diversity on; XTAL_OPTION = 1.
        put16(&mut img, MT_EE_NIC_CONF_2, 0x0210);
        img[MT_EE_COUNTRY_REGION_5GHZ] = 0x01;
        img[MT_EE_COUNTRY_REGION_2GHZ] = 0x02;

        put16(&mut img, MT_EE_FREQ_OFFSET, 0x0020); // low byte = 32

        put16(&mut img, MT_EE_LNA_GAIN, 0x0a08); // lna_2g = 8, lna_5g[0] = 10
        put16(&mut img, MT_EE_RSSI_OFFSET_2G_0, 0x03fe); // chain0 = -2, chain1 = +3
        put16(&mut img, MT_EE_RSSI_OFFSET_2G_1, 0x0b00); // high byte = lna_5g[1] = 11
        put16(&mut img, MT_EE_RSSI_OFFSET_5G_0, 0x01fb); // chain0 = -5, chain1 = +1
        put16(&mut img, MT_EE_RSSI_OFFSET_5G_1, 0x0c00); // high byte = lna_5g[2] = 12

        // BW40 delta: 2.4 GHz 0xc5 => enable|positive|5 = +5;
        //             5 GHz   0x83 => enable|negative|3 = -3.
        put16(&mut img, MT_EE_TX_POWER_DELTA_BW40, 0x83c5);

        // 2.4 GHz per-channel-group target power. Group for ch 5/6 is offset 4,
        // i.e. the word at 0x056: low byte 20 (ch < 6), high byte 30 (ch == 6).
        img[MT_EE_TX_POWER_DELTA_BW80 + 4] = 20;
        img[MT_EE_TX_POWER_DELTA_BW80 + 5] = 30;
        // 5 GHz group for ch 36/38 is offset 0, the word at 0x078.
        img[MT_EE_TX_POWER_0_GRP4_TSSI_SLOPE + 2] = 34;
        img[MT_EE_TX_POWER_0_GRP4_TSSI_SLOPE + 3] = 35;

        // 2G target power 40, temp offset 0x85 => sign-magnitude +5.
        put16(&mut img, MT_EE_2G_TARGET_POWER, 0x8528);
        // 5G target power 44, BW80 delta 0x88 => enable|negative|8 = -8.
        put16(&mut img, MT_EE_5G_TARGET_POWER, 0x882c);

        // Seven 5 GHz TSSI bounds, then the frequency-offset compensation byte
        // 0x83 => sign-magnitude +3, so freq_offset = 32 - 3 = 29.
        for (i, v) in [50u8, 64, 100, 128, 149, 165, 0].iter().enumerate() {
            img[MT_EE_TSSI_BOUND1 + i] = *v;
        }
        img[MT_EE_FREQ_OFFSET_COMPENSATION] = 0x83;
        put16(&mut img, MT_EE_TSSI_SLOPE_2G, 0x9b40); // slope 0x40, offset 0x9b-256
        put16(&mut img, MT_EE_TSSI_SLOPE_5G, 0x1050); // slope 0x50, offset 0x10

        // 2.4 GHz per-rate table.
        put16(&mut img, MT_EE_TX_POWER_BYRATE_BASE, 0x0a05); // cck  5,5,10,10
        put16(&mut img, MT_EE_TX_POWER_BYRATE_BASE + 2, 0x0b06); // ofdm 6,6,11,11
        put16(&mut img, MT_EE_TX_POWER_BYRATE_BASE + 4, 0x0c07); // ofdm 7,7,12,12
        put16(&mut img, MT_EE_TX_POWER_BYRATE_BASE + 6, 0x0d08); // ht   8,8,13,13
        put16(&mut img, MT_EE_TX_POWER_BYRATE_BASE + 8, 0x0e09); // ht   9,9,14,14
        // 5 GHz per-rate table.
        put16(&mut img, MT_EE_TX_POWER_BYRATE_5G, 0x1110); // ofdm 16,16,17,17
        put16(&mut img, MT_EE_TX_POWER_BYRATE_5G + 2, 0x1312); // ofdm 18,18,19,19
        put16(&mut img, MT_EE_TX_POWER_BYRATE_5G + 4, 0x1514); // ht   20,20,21,21
        put16(&mut img, MT_EE_TX_POWER_BYRATE_5G + 6, 0x1716); // ht   22,22,23,23
        put16(&mut img, MT_EE_TX_POWER_BYRATE_VHT_5G, 0x0201); // vht  1, 2

        img
    }

    // ── Offsets ─────────────────────────────────────────────────────────────

    /// Pin every `MT_EE_*` offset against `mt76x02_eeprom.h:12-96` verbatim.
    /// This is the table the silicon dump will be diffed against, so a typo here
    /// is the most expensive bug in the file.
    #[test]
    fn field_offsets_match_upstream() {
        /// Assert one constant against its upstream literal, keeping the name in
        /// the failure message.
        macro_rules! pin {
            ($($c:ident => $v:expr),* $(,)?) => {$(
                assert_eq!($c, $v, concat!(stringify!($c), " moved"));
                assert!($c < MT76X0_EEPROM_SIZE, concat!(stringify!($c), " is outside the image"));
            )*};
        }

        pin! {
            MT_EE_CHIP_ID => 0x000,
            MT_EE_VERSION => 0x002,
            MT_EE_MAC_ADDR => 0x004,
            MT_EE_PCI_ID => 0x00a,
            MT_EE_ANTENNA => 0x022,
            MT_EE_CFG1_INIT => 0x024,
            MT_EE_NIC_CONF_0 => 0x034,
            MT_EE_NIC_CONF_1 => 0x036,
            MT_EE_COUNTRY_REGION_5GHZ => 0x038,
            MT_EE_COUNTRY_REGION_2GHZ => 0x039,
            MT_EE_FREQ_OFFSET => 0x03a,
            MT_EE_XTAL_TRIM_1 => 0x03a,
            MT_EE_NIC_CONF_2 => 0x042,
            MT_EE_LNA_GAIN => 0x044,
            MT_EE_RSSI_OFFSET_2G_0 => 0x046,
            MT_EE_RSSI_OFFSET_2G_1 => 0x048,
            MT_EE_LNA_GAIN_5GHZ_1 => 0x049,
            MT_EE_RSSI_OFFSET_5G_0 => 0x04a,
            MT_EE_RSSI_OFFSET_5G_1 => 0x04c,
            MT_EE_LNA_GAIN_5GHZ_2 => 0x04d,
            MT_EE_TX_POWER_DELTA_BW40 => 0x050,
            MT_EE_TX_POWER_DELTA_BW80 => 0x052,
            MT_EE_TX_POWER_EXT_PA_5G => 0x054,
            MT_EE_TX_POWER_0_START_2G => 0x056,
            MT_EE_TX_POWER_1_START_2G => 0x05c,
            MT_EE_TX_POWER_0_START_5G => 0x062,
            MT_EE_TSSI_SLOPE_2G => 0x06e,
            MT_EE_TX_POWER_0_GRP3_TX_POWER_DELTA => 0x074,
            MT_EE_TX_POWER_0_GRP4_TSSI_SLOPE => 0x076,
            MT_EE_TX_POWER_1_START_5G => 0x080,
            MT_EE_XTAL_TRIM_2 => 0x09e,
            MT_EE_TX_POWER_CCK => 0x0a0,
            MT_EE_TX_POWER_OFDM_2G_6M => 0x0a2,
            MT_EE_TX_POWER_OFDM_2G_24M => 0x0a4,
            MT_EE_TX_POWER_HT_MCS0 => 0x0a6,
            MT_EE_TX_POWER_HT_MCS4 => 0x0a8,
            MT_EE_TX_POWER_HT_MCS8 => 0x0aa,
            MT_EE_TX_POWER_HT_MCS12 => 0x0ac,
            MT_EE_TX_POWER_OFDM_5G_6M => 0x0b2,
            MT_EE_TX_POWER_OFDM_5G_24M => 0x0b4,
            MT_EE_TX_POWER_VHT_MCS8 => 0x0be,
            MT_EE_2G_TARGET_POWER => 0x0d0,
            MT_EE_TEMP_OFFSET => 0x0d1,
            MT_EE_5G_TARGET_POWER => 0x0d2,
            MT_EE_TSSI_BOUND1 => 0x0d4,
            MT_EE_TSSI_BOUND2 => 0x0d6,
            MT_EE_TSSI_BOUND3 => 0x0d8,
            MT_EE_TSSI_BOUND4 => 0x0da,
            MT_EE_FREQ_OFFSET_COMPENSATION => 0x0db,
            MT_EE_TSSI_BOUND5 => 0x0dc,
            MT_EE_TX_POWER_BYRATE_BASE => 0x0de,
            MT_EE_TSSI_SLOPE_5G => 0x0f0,
            MT_EE_RF_TEMP_COMP_SLOPE_5G => 0x0f2,
            MT_EE_RF_TEMP_COMP_SLOPE_2G => 0x0f4,
            MT_EE_RF_2G_TSSI_OFF_TXPOWER => 0x0f6,
            MT_EE_RF_2G_RX_HIGH_GAIN => 0x0f8,
            MT_EE_RF_5G_GRP0_1_RX_HIGH_GAIN => 0x0fa,
            MT_EE_RF_5G_GRP2_3_RX_HIGH_GAIN => 0x0fc,
            MT_EE_RF_5G_GRP4_5_RX_HIGH_GAIN => 0x0fe,
            MT_EE_BT_RCAL_RESULT => 0x138,
            MT_EE_BT_VCDL_CALIBRATION => 0x13c,
            MT_EE_BT_PMUCFG => 0x13e,
            MT_EE_USAGE_MAP_START => 0x1e0,
            MT_EE_USAGE_MAP_END => 0x1fc,
            // ★ Unnamed upstream — bare literals at mt76x0/eeprom.c:168 and :192.
            MT_EE_TX_POWER_BYRATE_5G => 0x120,
            MT_EE_TX_POWER_BYRATE_VHT_5G => 0x12c,
        }

        // The three fields upstream reaches as a HIGH byte must sit exactly one
        // above their even word, or the `>> 8` decodes silently read the wrong
        // cell — mt76x02_eeprom.c:111-115, mt76x0/eeprom.c:86,103.
        assert_eq!(MT_EE_LNA_GAIN_5GHZ_1, MT_EE_RSSI_OFFSET_2G_1 + 1);
        assert_eq!(MT_EE_LNA_GAIN_5GHZ_2, MT_EE_RSSI_OFFSET_5G_1 + 1);
        assert_eq!(MT_EE_TEMP_OFFSET, MT_EE_2G_TARGET_POWER + 1);
        assert_eq!(MT_EE_FREQ_OFFSET_COMPENSATION, MT_EE_TSSI_BOUND4 + 1);

        assert_eq!(MT_EFUSE_USAGE_MAP_SIZE, 29);
        assert_eq!(MT_MAP_READS, MT_EFUSE_USAGE_MAP_SIZE.div_ceil(16));
    }

    // ── Sign encodings ──────────────────────────────────────────────────────

    /// [`s6_to_s8`] is genuine 6-bit two's complement — 0x20 is the sign bit and
    /// bits above 5 are ignored, because upstream feeds it an unmasked `val >> 8`.
    #[test]
    fn s6_to_s8_is_twos_complement() {
        assert_eq!(s6_to_s8(0x00), 0);
        assert_eq!(s6_to_s8(0x1f), 31); // largest positive
        assert_eq!(s6_to_s8(0x20), -32); // most negative
        assert_eq!(s6_to_s8(0x3f), -1);
        assert_eq!(s6_to_s8(0xff05), 5, "bits above 5 must be ignored");
    }

    /// [`sign_extend`] is sign-**magnitude with an inverted sign bit**, not sign
    /// extension. This test is the guard against a future "cleanup" to
    /// `as i8`/`i32::from_le_bytes`, which would flip the sign of every
    /// temperature and frequency trim.
    #[test]
    fn sign_extend_is_inverted_sign_magnitude() {
        // size 8: bit 7 set => POSITIVE.
        assert_eq!(sign_extend(0x85, 8), 5);
        assert_eq!(sign_extend(0x05, 8), -5, "clear sign bit negates");
        assert_eq!(sign_extend(0x80, 8), 0);
        assert_eq!(sign_extend(0x00, 8), 0);
        // Two's complement would have said -123 for 0x85; it must not.
        assert_ne!(sign_extend(0x85, 8), -123);
    }

    /// [`rate_power_val`]: bit 7 enables, bit 6 signs, bits 5:0 are magnitude.
    #[test]
    fn rate_power_val_decodes_enable_sign_magnitude() {
        assert_eq!(rate_power_val(0x00), 0, "0 is unprogrammed");
        assert_eq!(rate_power_val(0xff), 0, "0xff is unprogrammed");
        assert_eq!(rate_power_val(0x05), 0, "bit 7 clear => not programmed");
        assert_eq!(rate_power_val(0xc5), 5, "enable | sign | 5");
        assert_eq!(rate_power_val(0x85), -5, "enable | no sign | 5");
        assert_eq!(rate_power_val(0x88), -8);
    }

    /// The [`RatePower`] walk order must match upstream's `s8 all[30]` union
    /// layout, because [`RatePower::max`] and [`RatePower::add_offset`] are
    /// defined over it.
    #[test]
    fn rate_power_walks_the_union_in_order() {
        let t = RatePower {
            cck: [1, 2, 3, 4],
            ofdm: [5, 6, 7, 8, 9, 10, 11, 12],
            ht: [
                13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28,
            ],
            vht: [29, 30],
        };
        let all: Vec<i8> = t.iter().collect();
        assert_eq!(all.len(), 30, "union is s8 all[30] — mt76x02.h:83");
        assert_eq!(all, (1..=30).collect::<Vec<i8>>());
        assert_eq!(t.max(), 30);

        let mut t2 = t;
        t2.add_offset(-40);
        assert_eq!(t2.cck[0], -39);
        assert_eq!(t2.max(), 0, "max is seeded at 0 — mt76x02_phy.c:64");

        let mut t3 = t;
        t3.limit(10);
        assert_eq!(t3.ofdm[7], 10);
        assert_eq!(t3.cck[0], 1, "limit only clamps downward");
    }

    // ── Identity and configuration ──────────────────────────────────────────

    #[test]
    fn parses_identity_and_board_configuration() {
        let ee = Mt76x0Eeprom::parse(&synthetic_image());

        assert_eq!(ee.chip_id(), 0x7610);
        assert_eq!(ee.pci_id(), 0x7610);
        assert_eq!(
            ee.version(),
            0x0c,
            "version is the HIGH byte of MT_EE_VERSION"
        );
        assert_eq!(ee.fae(), 0x03, "fae is the LOW byte");
        assert!(ee.version() <= MT76X0U_EE_MAX_VER);
        assert_eq!(ee.mac_addr(), [0, 0x0c, 0x43, 0x76, 0x10, 0x01]);
        assert!(!ee.is_short_image());

        assert_eq!(ee.board_type(), BoardType::DualBand(0));
        assert!(ee.board_type().has_2ghz() && ee.board_type().has_5ghz());
        assert_eq!(ee.rx_path(), 1);
        assert_eq!(ee.tx_path(), 1);
        assert!(ee.stream_counts_valid());

        // PA_INT_2G set => the 2.4 GHz PA is internal => no external PA.
        assert!(!ee.ext_pa_enabled(false));
        // PA_INT_5G clear => external 5 GHz PA => RF_EXT_PA_TAB applies.
        assert!(ee.ext_pa_enabled(true));

        assert!(!ee.tssi_enabled());
        assert!(!ee.hw_rf_ctrl());
        assert!(ee.ext_lna_enabled(false));
        assert!(!ee.ext_lna_enabled(true));

        assert!(ee.antenna_dual());
        assert_eq!(ee.cfg1_init(), 0x1234);
        assert!(ee.antenna_diversity());
        assert_eq!(ee.xtal_option(), 1);
        assert_eq!(ee.country_region(), (0x01, 0x02));
    }

    /// An all-0xff NIC_CONF_1 low byte must be zeroed before the feature bits are
    /// read — `mt76x0_set_chip_cap`, mt76x0/eeprom.c:67-68.
    #[test]
    fn unprogrammed_nic_conf1_low_byte_is_masked_off() {
        let mut img = synthetic_image();
        put16(&mut img, MT_EE_NIC_CONF_1, 0x20ff);
        let ee = Mt76x0Eeprom::parse(&img);
        assert_eq!(ee.nic_conf1_sanitised(), 0x2000);
        assert!(
            !ee.hw_rf_ctrl(),
            "0xff must not read as every feature present"
        );
        assert!(ee.tssi_enabled(), "the high byte is untouched by the mask");
    }

    #[test]
    fn board_type_field_selects_the_bands() {
        let mut img = synthetic_image();
        for (field, want) in [
            (1u16, BoardType::Band2GhzOnly),
            (2, BoardType::Band5GhzOnly),
            (0, BoardType::DualBand(0)),
            (3, BoardType::DualBand(3)),
        ] {
            put16(&mut img, MT_EE_NIC_CONF_0, 0x0111 | (field << 12));
            assert_eq!(Mt76x0Eeprom::parse(&img).board_type(), want);
        }
    }

    // ── RX gain ─────────────────────────────────────────────────────────────

    #[test]
    fn parses_lna_gains_and_rssi_offsets() {
        let ee = Mt76x0Eeprom::parse(&synthetic_image());

        assert_eq!(ee.lna_gain_2g(), 8);
        assert_eq!(ee.lna_gain_5g(), [10, 11, 12]);
        // The sub-band split is <= 64, <= 128, above — mt76x02_eeprom.c:136-144.
        assert_eq!(ee.lna_gain(6), 8);
        assert_eq!(ee.lna_gain(36), 10);
        assert_eq!(ee.lna_gain(64), 10);
        assert_eq!(ee.lna_gain(100), 11);
        assert_eq!(ee.lna_gain(128), 11);
        assert_eq!(ee.lna_gain(149), 12);

        assert_eq!(ee.rssi_offset(false), [-2, 3]);
        assert_eq!(ee.rssi_offset(true), [-5, 1]);
    }

    /// Unprogrammed upper-group 5 GHz LNA gains fall back to the low group
    /// (`mt76x02_eeprom.c:117-121`), and an 0xff gain reads 0 at use
    /// (`mt76x02_eeprom.c:145`).
    #[test]
    fn lna_gain_fallbacks() {
        let mut img = synthetic_image();
        put16(&mut img, MT_EE_RSSI_OFFSET_2G_1, 0xff00); // lna_5g[1] unprogrammed
        put16(&mut img, MT_EE_RSSI_OFFSET_5G_1, 0x0000); // lna_5g[2] unprogrammed
        let ee = Mt76x0Eeprom::parse(&img);
        assert_eq!(ee.lna_gain_5g(), [10, 10, 10]);

        put16(&mut img, MT_EE_LNA_GAIN, 0xff08); // lna_5g[0] = 0xff
        let ee = Mt76x0Eeprom::parse(&img);
        assert_eq!(ee.lna_gain(36), 0, "0xff LNA gain reads as 0, not as -1");
    }

    /// RSSI offsets outside ±10 dB are discarded — mt76x0/eeprom.c:123-124.
    #[test]
    fn rssi_offsets_outside_ten_db_are_zeroed() {
        let mut img = synthetic_image();
        put16(&mut img, MT_EE_RSSI_OFFSET_2G_0, 0xf50f); // +15 and -11
        let ee = Mt76x0Eeprom::parse(&img);
        assert_eq!(ee.rssi_offset(false), [0, 0]);

        put16(&mut img, MT_EE_RSSI_OFFSET_2G_0, 0xf60a); // +10 and -10, both kept
        let ee = Mt76x0Eeprom::parse(&img);
        assert_eq!(ee.rssi_offset(false), [10, -10]);
    }

    /// ★ The load-bearing one: `signal = (s8)raw + rssi_offset[chain] - lna_gain`
    /// (`mt76x02_mac.c:760-769`). `CapturedFrame.rssi_dbm` and therefore the whole
    /// rate-adaptation loop hang off this.
    #[test]
    fn rssi_dbm_is_raw_plus_offset_minus_lna() {
        let ee = Mt76x0Eeprom::parse(&synthetic_image());
        let raw = 0xc0u8; // -64 as i8

        // 2.4 GHz chain 0: -64 + (-2) - 8 = -74.
        assert_eq!(ee.rssi_dbm_on_channel(raw, 6, 0), -74);
        // 2.4 GHz chain 1 uses the second offset byte: -64 + 3 - 8 = -69.
        assert_eq!(ee.rssi_dbm_on_channel(raw, 6, 1), -69);
        // A chain with no offset byte contributes 0.
        assert_eq!(ee.rssi_dbm_on_channel(raw, 6, 2), -72);

        // 5 GHz, one per LNA sub-band group: -64 - 5 - {10,11,12}.
        assert_eq!(ee.rssi_dbm_on_channel(raw, 36, 0), -79);
        assert_eq!(ee.rssi_dbm_on_channel(raw, 100, 0), -80);
        assert_eq!(ee.rssi_dbm_on_channel(raw, 149, 0), -81);

        // A positive raw byte is still signed: 0x0a = +10.
        assert_eq!(ee.rssi_dbm_on_channel(0x0a, 6, 0), 0);
    }

    /// The cached channel picks the 5 GHz sub-band for the bool-only entry point,
    /// and a stale channel from the other band never leaks into the answer.
    #[test]
    fn rssi_dbm_uses_the_cached_channel_for_the_five_ghz_group() {
        let ee = Mt76x0Eeprom::parse(&synthetic_image());
        let raw = 0xc0u8;

        assert_eq!(ee.current_channel(), 0);
        assert_eq!(ee.rssi_dbm(raw, true, 0), -79, "unset => low 5 GHz group");

        ee.set_channel(149);
        assert_eq!(ee.rssi_dbm(raw, true, 0), -81);
        // Asking for 2.4 GHz while parked on ch149 must not use a 5 GHz LNA.
        assert_eq!(ee.rssi_dbm(raw, false, 0), -74);

        ee.set_channel(6);
        assert_eq!(ee.rssi_dbm(raw, false, 0), -74);
        assert_eq!(ee.rssi_dbm(raw, true, 0), -79, "5 GHz falls back to ch36");
    }

    // ── Trims ───────────────────────────────────────────────────────────────

    #[test]
    fn parses_temperature_and_frequency_trims() {
        let ee = Mt76x0Eeprom::parse(&synthetic_image());
        assert_eq!(ee.temp_offset(), 5, "0x85 => sign-magnitude +5");
        // 0x20 (32) minus the sign-magnitude compensation 0x83 (+3) = 29.
        assert_eq!(ee.freq_offset(), 29);

        let mut img = synthetic_image();
        img[MT_EE_TEMP_OFFSET] = 0xff;
        assert_eq!(
            Mt76x0Eeprom::parse(&img).temp_offset(),
            -10,
            "unprogrammed temp offset defaults to -10, not 0 — mt76x0/eeprom.c:90"
        );

        let mut img = synthetic_image();
        put16(&mut img, MT_EE_FREQ_OFFSET, 0x0000); // unprogrammed
        img[MT_EE_FREQ_OFFSET_COMPENSATION] = 0x00; // unprogrammed
        assert_eq!(Mt76x0Eeprom::parse(&img).freq_offset(), 0);
    }

    #[test]
    fn parses_tssi_bounds_and_slopes() {
        let ee = Mt76x0Eeprom::parse(&synthetic_image());
        assert_eq!(ee.tssi_bounds_5g(), [50, 64, 100, 128, 149, 165, 0]);

        // 2.4 GHz: slope is the low byte, offset the high byte biased by bit 7.
        assert_eq!(ee.tssi_slope_offset(6), (0x40, 0x9b - 256));
        // 5 GHz ch36 <= bound[0] = 50 => index 0 => MT_EE_TSSI_SLOPE_5G.
        assert_eq!(ee.tssi_slope_offset(36), (0x50, 0x10));
    }

    // ── TX power ────────────────────────────────────────────────────────────

    #[test]
    fn per_rate_power_reads_the_right_band_table() {
        let ee = Mt76x0Eeprom::parse(&synthetic_image());

        let t = ee.tx_power_per_rate(6);
        assert_eq!(t.cck, [5, 5, 10, 10]);
        assert_eq!(t.ofdm, [6, 6, 11, 11, 7, 7, 12, 12]);
        assert_eq!(t.ht[..8], [8, 8, 13, 13, 9, 9, 14, 14]);
        assert_eq!(t.ht[8..], [0; 8], "1×1 part: MCS 8..15 are never filled");
        assert_eq!(t.vht, [1, 2]);

        let t = ee.tx_power_per_rate(36);
        assert_eq!(t.ofdm, [16, 16, 17, 17, 18, 18, 19, 19]);
        assert_eq!(t.ht[..8], [20, 20, 21, 21, 22, 22, 23, 23]);
        assert_eq!(
            t.cck,
            [5, 5, 10, 10],
            "CCK is read from the 2.4 GHz base in both bands — mt76x0/eeprom.c:163"
        );
    }

    /// The bandwidth delta is added to **all 30** entries, including the eight
    /// unfilled HT slots — mt76x02_phy.c:88 walks the whole union.
    #[test]
    fn bandwidth_delta_offsets_the_whole_table() {
        let ee = Mt76x0Eeprom::parse(&synthetic_image());

        let t = ee.tx_power_per_rate_bw(6, Bandwidth::Bw40);
        assert_eq!(t.cck, [10, 10, 15, 15], "2.4 GHz BW40 delta is +5");
        assert_eq!(t.ht[8..], [5; 8], "the zero slots receive the delta too");

        let t = ee.tx_power_per_rate_bw(36, Bandwidth::Bw40);
        assert_eq!(t.ofdm[0], 13, "5 GHz BW40 delta is -3");

        let t = ee.tx_power_per_rate_bw(36, Bandwidth::Bw80);
        assert_eq!(
            t.ofdm[0], 8,
            "BW80 delta is the high byte of 5G_TARGET_POWER: -8"
        );

        for bw in [Bandwidth::Bw20, Bandwidth::Nb10, Bandwidth::Nb5] {
            assert_eq!(ee.power_delta(6, bw), 0, "{bw:?} has no EEPROM delta");
        }
    }

    /// With TSSI on, the closed loop owns the correction and the static delta
    /// must not be applied — mt76x0/eeprom.c:196.
    #[test]
    fn tssi_suppresses_the_static_bandwidth_delta() {
        let mut img = synthetic_image();
        put16(&mut img, MT_EE_NIC_CONF_1, 0x2004);
        let ee = Mt76x0Eeprom::parse(&img);
        assert!(ee.tssi_enabled());
        assert_eq!(
            ee.tx_power_per_rate_bw(6, Bandwidth::Bw40).cck,
            [5, 5, 10, 10]
        );
    }

    /// The non-TSSI target power walks [`CHAN_MAP`]: an exact channel hit takes
    /// the high byte of the group word, anything below it the low byte.
    #[test]
    fn target_power_selects_the_channel_group_byte() {
        let ee = Mt76x0Eeprom::parse(&synthetic_image());
        let bw = Bandwidth::Bw20;

        assert_eq!(ee.target_power_half_db(6, bw), 30, "exact hit => high byte");
        assert_eq!(ee.target_power_half_db(5, bw), 20, "below => low byte");
        assert_eq!(ee.target_power_half_db(36, bw), 34);
        assert_eq!(ee.target_power_half_db(38, bw), 35);
        // Past the last row (175): offset falls back to chan_map[0].offset = 0
        // and idx to 0 — mt76x0/eeprom.c:239-240.
        assert_eq!(ee.target_power_half_db(200, bw), 34);
        // An unprogrammed group byte (0xff => -1) falls back to 5 — :269-270.
        assert_eq!(ee.target_power_half_db(2, bw), 5);
        // No bandwidth delta on this path.
        assert_eq!(ee.target_power_half_db(6, Bandwidth::Bw40), 30);
    }

    /// With TSSI on the target power comes from the band target-power word minus
    /// `ofdm[7]`, plus the bandwidth delta — mt76x0/eeprom.c:219-230.
    #[test]
    fn target_power_with_tssi_is_target_minus_ofdm7() {
        let mut img = synthetic_image();
        put16(&mut img, MT_EE_NIC_CONF_1, 0x2004);
        let ee = Mt76x0Eeprom::parse(&img);

        // 2.4 GHz: 0x28 (40) - ofdm[7] (12) = 28.
        assert_eq!(ee.target_power_half_db(6, Bandwidth::Bw20), 28);
        assert_eq!(
            ee.target_power_half_db(6, Bandwidth::Bw40),
            33,
            "+5 BW40 delta"
        );
        // 5 GHz: 0x2c (44) - ofdm[7] (19) = 25.
        assert_eq!(ee.target_power_half_db(36, Bandwidth::Bw20), 25);
        assert_eq!(
            ee.target_power_half_db(36, Bandwidth::Bw80),
            17,
            "-8 BW80 delta"
        );
    }

    /// The half-dB → dBm step, i.e. what mac80211 would report.
    #[test]
    fn absolute_power_leaves_the_half_db_domain_once() {
        let ee = Mt76x0Eeprom::parse(&synthetic_image());
        // ch6: max rate entry 14 + target 30 = 44 half-dB => 22 dBm.
        assert_eq!(ee.max_rate_power_half_db(6, Bandwidth::Bw20), 44);
        assert_eq!(ee.target_power_dbm(6), 22);
        // ch36: max rate entry 23 + target 34 = 57 half-dB => ceil(57/2) = 29.
        assert_eq!(ee.max_rate_power_half_db(36, Bandwidth::Bw20), 57);
        assert_eq!(ee.target_power_dbm(36), 29);
    }

    // ── Image handling ──────────────────────────────────────────────────────

    #[test]
    fn short_and_oversized_images_are_handled_without_panicking() {
        let ee = Mt76x0Eeprom::parse(&[0x10, 0x76]);
        assert!(ee.is_short_image());
        assert_eq!(ee.chip_id(), 0x7610);
        assert_eq!(ee.byte(0x1ff), 0xff, "the tail is 0xff-padded");
        assert_eq!(
            ee.word(MT76X0_EEPROM_SIZE),
            0xffff,
            "out of range never panics"
        );

        let long = vec![0u8; MT76X0_EEPROM_SIZE * 2];
        assert!(!Mt76x0Eeprom::parse(&long).is_short_image());
    }

    #[test]
    fn image_validity_matches_upstreams_check() {
        let img = synthetic_image();
        assert!(image_looks_valid(&img));

        // Chip id 0 falls back to MT_EE_PCI_ID — mt76x0/eeprom.c:277-280.
        let mut img2 = img;
        put16(&mut img2, MT_EE_CHIP_ID, 0);
        assert!(image_looks_valid(&img2));

        put16(&mut img2, MT_EE_PCI_ID, 0x7662);
        assert!(!image_looks_valid(&img2), "an MT7662 image is not ours");
        assert!(!image_looks_valid(&[0xff; MT76X0_EEPROM_SIZE]));
        assert!(!image_looks_valid(&[]));

        let mut img3 = img;
        put16(&mut img3, MT_EE_CHIP_ID, 0x7650);
        assert!(image_looks_valid(&img3), "the MT7650 sibling is accepted");
    }

    #[test]
    fn copy_from_is_bounds_checked() {
        let ee = Mt76x0Eeprom::parse(&synthetic_image());
        let mut bounds = [0u8; 7];
        ee.copy_from(MT_EE_TSSI_BOUND1, &mut bounds).unwrap();
        assert_eq!(bounds, [50, 64, 100, 128, 149, 165, 0]);
        let mut too_big = [0u8; 8];
        assert!(ee.copy_from(MT76X0_EEPROM_SIZE - 4, &mut too_big).is_err());
    }

    // ── The eFuse read path, against a mock register bus ─────────────────────

    /// A register bus that implements the eFuse kick protocol over a byte image,
    /// so [`efuse_read_block`] and everything above it can be exercised with no
    /// dongle attached. Deliberately strict: it records every AIN it is asked for
    /// so the test can assert the 16-byte block walk.
    struct MockEfuse {
        image: Vec<u8>,
        /// Make every block report `AOUT` all-ones (an unprogrammed part).
        aout_all_ones: bool,
        state: Mutex<MockState>,
    }

    #[derive(Default)]
    struct MockState {
        ctrl: u32,
        data: [u32; 4],
        ains: Vec<u32>,
        modes: Vec<u32>,
    }

    impl MockEfuse {
        fn new(image: Vec<u8>) -> Self {
            Self {
                image,
                aout_all_ones: false,
                state: Mutex::new(MockState::default()),
            }
        }
    }

    impl Mt76Regs for MockEfuse {
        fn rr(&self, addr: u32) -> Result<u32, FaceError> {
            let s = self.state.lock().unwrap();
            Ok(match addr {
                MT_EFUSE_CTRL => s.ctrl,
                a if (MT_EFUSE_DATA_BASE..MT_EFUSE_DATA_BASE + 16).contains(&a) => {
                    s.data[((a - MT_EFUSE_DATA_BASE) / 4) as usize]
                }
                _ => 0,
            })
        }

        fn wr(&self, addr: u32, val: u32) -> Result<(), FaceError> {
            if addr != MT_EFUSE_CTRL {
                return Ok(());
            }
            let mut s = self.state.lock().unwrap();
            if val & MT_EFUSE_CTRL_KICK == 0 {
                s.ctrl = val;
                return Ok(());
            }
            let ain = field_get(MT_EFUSE_CTRL_AIN, val);
            s.ains.push(ain);
            s.modes.push(field_get(MT_EFUSE_CTRL_MODE, val));
            for i in 0..4usize {
                let mut w = [0u8; 4];
                for (j, b) in w.iter_mut().enumerate() {
                    *b = self
                        .image
                        .get(ain as usize + i * 4 + j)
                        .copied()
                        .unwrap_or(0xff);
                }
                s.data[i] = u32::from_le_bytes(w);
            }
            // The kick completes immediately; AOUT reports whether the block exists.
            s.ctrl = val & !MT_EFUSE_CTRL_KICK;
            if self.aout_all_ones {
                s.ctrl |= MT_EFUSE_CTRL_AOUT;
            } else {
                s.ctrl &= !MT_EFUSE_CTRL_AOUT;
            }
            Ok(())
        }
    }

    #[test]
    fn efuse_path_reads_the_image_in_aligned_sixteen_byte_blocks() {
        let img = synthetic_image();
        let mock = MockEfuse::new(img.to_vec());

        let got = read_efuse_image(&mock).unwrap();
        assert_eq!(got, img);

        let s = mock.state.lock().unwrap();
        assert_eq!(s.ains.len(), MT76X0_EEPROM_SIZE / 16);
        assert!(
            s.ains
                .iter()
                .enumerate()
                .all(|(i, &a)| a as usize == i * 16),
            "blocks must be walked in order at 16-byte alignment"
        );
        assert!(
            s.modes.iter().all(|&m| m == EfuseMode::Read.code()),
            "the image read uses MT_EE_READ, not the physical mode"
        );
    }

    /// `AOUT` all-ones means the block does not exist and reads 0xff —
    /// mt76x02_eeprom.c:32-35.
    #[test]
    fn efuse_reports_unprogrammed_blocks_as_all_ones() {
        let mut mock = MockEfuse::new(synthetic_image().to_vec());
        mock.aout_all_ones = true;
        let got = read_efuse_image(&mock).unwrap();
        assert_eq!(got, [0xffu8; MT76X0_EEPROM_SIZE]);
    }

    /// The AIN field is `addr & ~0xf`, so an unaligned request still lands on the
    /// containing block — mt76x02_eeprom.c:21.
    #[test]
    fn efuse_block_address_is_masked_to_sixteen() {
        let mock = MockEfuse::new(synthetic_image().to_vec());
        let mut block = [0u8; 16];
        efuse_read_block(&mock, 0x0d7, EfuseMode::Read, &mut block).unwrap();
        assert_eq!(mock.state.lock().unwrap().ains, vec![0x0d0]);
        assert_eq!(block[..2], synthetic_image()[0x0d0..0x0d2]);
    }

    /// The usage-map check reads 32 bytes from 0x1e0 in **physical** mode and
    /// refuses a part with fewer than 5 programmed cells — mt76x0/eeprom.c:19-46.
    #[test]
    fn physical_size_check_rejects_a_blank_usage_map() {
        let mock = MockEfuse::new(synthetic_image().to_vec());
        // The synthetic map is all 0xff: no free run, so 28 cells count as used.
        assert_eq!(efuse_physical_size_check(&mock).unwrap(), 28);
        assert_eq!(
            mock.state.lock().unwrap().modes,
            vec![EfuseMode::PhysicalRead.code(); MT_MAP_READS]
        );

        let mut blank = synthetic_image();
        blank[MT_EE_USAGE_MAP_START..=MT_EE_USAGE_MAP_END].fill(0);
        let mock = MockEfuse::new(blank.to_vec());
        let e = efuse_physical_size_check(&mock).unwrap_err();
        assert!(
            format!("{e}").contains("does not support default EEPROM"),
            "unexpected error: {e}"
        );

        // Exactly 5 programmed cells is the boundary and must pass.
        let mut edge = synthetic_image();
        edge[MT_EE_USAGE_MAP_START..=MT_EE_USAGE_MAP_END].fill(0);
        for i in 0..5 {
            edge[MT_EE_USAGE_MAP_START + i] = 0xa5;
        }
        assert_eq!(
            efuse_physical_size_check(&MockEfuse::new(edge.to_vec())).unwrap(),
            5
        );
    }

    // ── Source selection ────────────────────────────────────────────────────

    #[test]
    fn load_prefers_a_valid_usb_shadow_and_falls_back_to_efuse() {
        let img = synthetic_image();
        let mock = MockEfuse::new(img.to_vec());

        let shadow_of = |src: [u8; MT76X0_EEPROM_SIZE]| {
            ShadowFn(move |off: u16| {
                let o = off as usize;
                Ok(u32::from_le_bytes([
                    src[o],
                    src[o + 1],
                    src[o + 2],
                    src[o + 3],
                ]))
            })
        };

        // A shadow that returns the real image is used, and the eFuse is untouched.
        let sh = shadow_of(img);
        let (ee, src) = Mt76x0Eeprom::load(&mock, Some(&sh)).unwrap();
        assert_eq!(src, EepromSource::UsbShadow);
        assert_eq!(ee.mac_addr(), [0, 0x0c, 0x43, 0x76, 0x10, 0x01]);
        assert!(mock.state.lock().unwrap().ains.is_empty());

        // A shadow that answers with zeros must NOT poison the parse.
        let sh = shadow_of([0u8; MT76X0_EEPROM_SIZE]);
        let (ee, src) = Mt76x0Eeprom::load(&mock, Some(&sh)).unwrap();
        assert_eq!(src, EepromSource::Efuse);
        assert_eq!(ee.chip_id(), 0x7610);

        // So must a shadow that errors outright.
        let sh = ShadowFn(|_off: u16| Err(err("no such endpoint".into())));
        let (_, src) = Mt76x0Eeprom::load(&mock, Some(&sh)).unwrap();
        assert_eq!(src, EepromSource::Efuse);

        // And with no shadow at all, the eFuse path is what mainline uses.
        let (ee, src) = Mt76x0Eeprom::load(&mock, None).unwrap();
        assert_eq!(src, EepromSource::Efuse);
        assert_eq!(ee.version(), 0x0c);
    }

    /// `load` propagates the usage-map refusal rather than parsing a blank image
    /// into plausible nonsense; `read_efuse_image` is the documented escape hatch.
    #[test]
    fn load_refuses_a_default_eeprom_but_the_raw_read_still_works() {
        let mut blank = synthetic_image();
        blank[MT_EE_USAGE_MAP_START..=MT_EE_USAGE_MAP_END].fill(0);
        let mock = MockEfuse::new(blank.to_vec());

        assert!(Mt76x0Eeprom::load(&mock, None).is_err());
        let raw = read_efuse_image(&mock).unwrap();
        assert_eq!(Mt76x0Eeprom::parse(&raw).chip_id(), 0x7610);
    }
}
