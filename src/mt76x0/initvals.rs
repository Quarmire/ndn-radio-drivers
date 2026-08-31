//! MT7610U (`mt76x0`) MAC/BBP initialisation register tables, transcribed
//! row-for-row from the upstream GPL mt76 driver's `mt76x0/initvals.h` and
//! `mt76x0/initvals_init.h`. Nothing here is invented; nothing here is omitted.
//!
//! | table | upstream name | file | rows |
//! |---|---|---|---|
//! | [`BBP_SWITCH_TAB`] | `mt76x0_bbp_switch_tab` | `mt76x0/initvals.h:14` | 48 |
//! | [`COMMON_MAC_REG_TABLE`] | `common_mac_reg_table` | `mt76x0/initvals_init.h:14` | 31 |
//! | [`MT76X0_MAC_REG_TABLE`] | `mt76x0_mac_reg_table` | `mt76x0/initvals_init.h:48` | 35 |
//! | [`MT76X0_BBP_INIT_TAB`] | `mt76x0_bbp_init_tab` | `mt76x0/initvals_init.h:86` | 58 |
//! | [`MT76X0_DCOC_TAB`] | `mt76x0_dcoc_tab` | `mt76x0/initvals_init.h:147` | 9 |
//!
//! Those five are *every* table in the two headers (`grep '^static const'`), and
//! the row counts are asserted in the tests at the bottom of this file.
//!
//! ## Addresses are resolved literals, deliberately
//! Upstream writes `MT_BBP(AGC, 4)` and `MT_TX_PWR_CFG_0`. Here each row carries the
//! **numeric** address with the original symbol in a trailing comment, resolved
//! against `mt76x02_regs.h` — `MT_BBP(_type, _n) = MT_BBP_<type>_BASE + (n << 2)`,
//! bases at `mt76x02_regs.h:604-617`, plain registers at their `#define` lines.
//! Two reasons, both about failure modes rather than taste:
//!   1. a reader can diff a row against the upstream header by eye, and
//!   2. one mistyped base cannot silently shift every row that shares it. A wrong
//!      hex digit in an RF table is not a compile error and not a crash — it is a
//!      radio that quietly does not work, which is the most expensive kind of bug
//!      this driver can have.
//!
//! ## MEASURED vs CODE-READ
//! **CODE-READ:** every constant below is upstream's, taken on faith except where
//! this section says otherwise.
//!
//! **MEASURED** (mds-o5p-1's MT7610U via `examples/mt76_oracle.rs`, 2026-08-27):
//! three of these registers were read back out of a *kernel-driven* monitor
//! interface — i.e. state the in-tree driver put there from these same upstream
//! tables — and match this transcription exactly:
//!
//! | register | measured | this file |
//! |---|---|---|
//! | `0x2308` `MT_BBP(AGC, 2)` | `0x003A6464` | [`MT76X0_BBP_INIT_TAB`] |
//! | `0x1104` `MT_BKOFF_SLOT_CFG` | `0x00000209` | [`COMMON_MAC_REG_TABLE`] |
//! | `0x1340` `MT_TXOP_CTRL_CFG` | `0x0000583F` | [`COMMON_MAC_REG_TABLE`] |
//!
//! `0x2308` is the load-bearing one: it is a `MT_BBP(...)` row, so it confirms the
//! AGC base (`0x2300`) and the `n << 2` arithmetic on real silicon, not just on
//! paper. A `#[test]` below pins all three so a future edit cannot drift off the
//! measurement.
//!
//! Two *other* measured registers disagree with these tables. Neither is a
//! transcription error and neither should be "fixed":
//!   * `MT_RX_FILTR_CFG 0x1400` measured `0x00001093`, table says `0x00017F97` — the
//!     RX filter is rewritten when the interface enters monitor mode, long after
//!     init. The table value is the reset-time value.
//!   * `MT_EXT_CCA_CFG 0x141c` measured `0x0000F1E4` and appears in no table at all:
//!     upstream sets its top nibble by hand right after the MAC tables
//!     (`mt76x0/init.c:120`, `mt76_set(dev, MT_EXT_CCA_CFG, 0xf000)`), which is the
//!     `F` in bits 15:12 of the measurement.
//!
//! ## How upstream applies these (order is not arbitrary)
//! `mt76x0/init.c:83` wraps each `mt76_reg_pair` table in `RANDOM_WRITE`, which is
//! `mt76_wr_rp(dev, MT_MCU_MEMMAP_WLAN, tab, ARRAY_SIZE(tab))` with
//! `MT_MCU_MEMMAP_WLAN = 0x410000` (`mt76x02_mcu.h:19`) — the MCU register-pair
//! path, *not* one EP0 vendor write per row. That is a performance decision worth
//! preserving: an EP0 round trip on this part is MEASURED at 151 µs, so writing the
//! 181 rows here one-at-a-time would cost ~20 ms of pure control traffic.
//!
//! Upstream's order:
//!   * BBP (`mt76x0/init.c:87-108`): wait for BBP ready, then
//!     [`MT76X0_BBP_INIT_TAB`], then the `RF_G_BAND | RF_BW_20` subset of
//!     [`BBP_SWITCH_TAB`], then [`MT76X0_DCOC_TAB`].
//!   * MAC (`mt76x0/init.c:109-115`): [`COMMON_MAC_REG_TABLE`], then
//!     [`MT76X0_MAC_REG_TABLE`] — the second table deliberately overwrites rows of
//!     the first (`MT_PBF_CFG`, `MT_TX_SW_CFG0/1`, `MT_HT_BASIC_RATE`,
//!     `MT_TXOP_HLDR_ET`), so applying them out of order silently leaves the
//!     generic mt76x02 values in place on a 7610.

use super::initvals_phy::{RF_A_BAND, RF_BW_20, RF_BW_40, RF_BW_80, RF_G_BAND};

/// One row of [`BBP_SWITCH_TAB`]: a BBP register value that is only correct for a
/// particular set of (band, bandwidth) combinations.
///
/// Flattened from upstream's `struct mt76x0_bbp_switch_item` (`mt76x0/phy.h:42`),
/// which nests a `struct mt76_reg_pair`. The nesting carries no information, so the
/// three fields are inlined here; `reg`/`val` are upstream's `reg_pair.reg` /
/// `reg_pair.value`.
pub struct BbpSwitch {
    /// Bitwise OR of the `RF_*_BAND` and `RF_BW_*` flags this row is valid for.
    ///
    /// This is a *set of allowed conditions*, not a condition to equal — see
    /// [`BbpSwitch::matches`] for the (non-obvious) selection rule.
    pub bw_band: u16,
    /// Resolved BBP register address (the `MT_BBP(unit, n)` in the row comment).
    pub reg: u32,
    /// Value to write.
    pub val: u32,
}

impl BbpSwitch {
    /// Does this row apply to the band/bandwidth described by `query`?
    ///
    /// The rule reads backwards at first glance: the row's `bw_band` must be a
    /// **superset** of the query, i.e. `query & bw_band == query`. Both upstream
    /// call sites spell this out longhand and both agree:
    ///   * `mt76x0/phy.c:409` — `if ((rf_bw_band & item->bw_band) != rf_bw_band) continue;`
    ///   * `mt76x0/init.c:101` — `if (((RF_G_BAND | RF_BW_20) & item->bw_band) == (RF_G_BAND | RF_BW_20))`
    ///
    /// So a query of `RF_G_BAND | RF_BW_20` selects the row tagged
    /// `RF_G_BAND | RF_BW_20 | RF_BW_40` (which covers both widths) but *not* the
    /// row tagged `RF_G_BAND | RF_BW_40`. Getting the polarity backwards would
    /// program 40 MHz AGC values into a 20 MHz channel: legal writes, bad receiver.
    pub const fn matches(&self, query: u16) -> bool {
        (query & self.bw_band) == query
    }
}

/// Per-(band, bandwidth) BBP overrides — `mt76x0_bbp_switch_tab`
/// (`mt76x0/initvals.h:14`).
///
/// Applied twice on different queries: once at init with `RF_G_BAND | RF_BW_20` to
/// leave the BBP in a sane 2.4 GHz/20 MHz state (`mt76x0/init.c:97-103`), and again
/// on every channel change with the actual band/width
/// (`mt76x0_phy_set_chan_bbp_params`, `mt76x0/phy.c:399-423`). Select rows with
/// [`BbpSwitch::matches`].
///
/// ⚠ One row is **not** written verbatim by upstream: `MT_BBP(AGC, 8)` (`0x2320`)
/// has its `MT_BBP_AGC_GAIN` field (`GENMASK(14, 8)`, `mt76x02_regs.h:636`) reduced
/// by `2 * cal.rx.lna_gain` before the write (`mt76x0/phy.c:411-419`). The raw
/// table value is stored here; the LNA correction belongs in the PHY code, not in
/// the table.///
/// `#[rustfmt::skip]`: one row per line is load-bearing here. rustfmt would split
/// each row across five lines, turning 48 auditable rows into 240 and making an
/// eyeball diff against `mt76x0/initvals.h` impossible — which is the only review
/// that can catch a wrong digit in a table like this.
#[rustfmt::skip]
pub const BBP_SWITCH_TAB: &[BbpSwitch] = &[
    BbpSwitch { bw_band: RF_G_BAND | RF_BW_20 | RF_BW_40, reg: 0x2310, val: 0x1FEDA049 }, // MT_BBP(AGC, 4)
    BbpSwitch { bw_band: RF_A_BAND | RF_BW_20 | RF_BW_40 | RF_BW_80, reg: 0x2310, val: 0x1FECA054 }, // MT_BBP(AGC, 4)
    BbpSwitch { bw_band: RF_G_BAND | RF_BW_20 | RF_BW_40, reg: 0x2318, val: 0x00000045 }, // MT_BBP(AGC, 6)
    BbpSwitch { bw_band: RF_A_BAND | RF_BW_20 | RF_BW_40 | RF_BW_80, reg: 0x2318, val: 0x0000000A }, // MT_BBP(AGC, 6)
    BbpSwitch { bw_band: RF_G_BAND | RF_BW_20 | RF_BW_40, reg: 0x2320, val: 0x16344EF0 }, // MT_BBP(AGC, 8)
    BbpSwitch { bw_band: RF_A_BAND | RF_BW_20 | RF_BW_40 | RF_BW_80, reg: 0x2320, val: 0x122C54F2 }, // MT_BBP(AGC, 8)
    BbpSwitch { bw_band: RF_G_BAND | RF_BW_20, reg: 0x2330, val: 0x05052879 }, // MT_BBP(AGC, 12)
    BbpSwitch { bw_band: RF_G_BAND | RF_BW_40, reg: 0x2330, val: 0x050528F9 }, // MT_BBP(AGC, 12)
    BbpSwitch { bw_band: RF_A_BAND | RF_BW_20 | RF_BW_40 | RF_BW_80, reg: 0x2330, val: 0x050528F9 }, // MT_BBP(AGC, 12)
    BbpSwitch { bw_band: RF_G_BAND | RF_BW_20 | RF_BW_40, reg: 0x2334, val: 0x35050004 }, // MT_BBP(AGC, 13)
    BbpSwitch { bw_band: RF_A_BAND | RF_BW_20 | RF_BW_40 | RF_BW_80, reg: 0x2334, val: 0x2C3A0406 }, // MT_BBP(AGC, 13)
    BbpSwitch { bw_band: RF_G_BAND | RF_BW_20 | RF_BW_40, reg: 0x2338, val: 0x310F2E3C }, // MT_BBP(AGC, 14)
    BbpSwitch { bw_band: RF_A_BAND | RF_BW_20 | RF_BW_40 | RF_BW_80, reg: 0x2338, val: 0x310F2A3F }, // MT_BBP(AGC, 14)
    BbpSwitch { bw_band: RF_G_BAND | RF_BW_20 | RF_BW_40, reg: 0x2368, val: 0x007C2005 }, // MT_BBP(AGC, 26)
    BbpSwitch { bw_band: RF_A_BAND | RF_BW_20 | RF_BW_40 | RF_BW_80, reg: 0x2368, val: 0x007C2005 }, // MT_BBP(AGC, 26)
    BbpSwitch { bw_band: RF_G_BAND | RF_BW_20 | RF_BW_40, reg: 0x236c, val: 0x000000E1 }, // MT_BBP(AGC, 27)
    BbpSwitch { bw_band: RF_A_BAND | RF_BW_20 | RF_BW_40 | RF_BW_80, reg: 0x236c, val: 0x000000EC }, // MT_BBP(AGC, 27)
    BbpSwitch { bw_band: RF_G_BAND | RF_BW_20, reg: 0x2370, val: 0x00060806 }, // MT_BBP(AGC, 28)
    BbpSwitch { bw_band: RF_G_BAND | RF_BW_40, reg: 0x2370, val: 0x00050806 }, // MT_BBP(AGC, 28)
    BbpSwitch { bw_band: RF_A_BAND | RF_BW_40, reg: 0x2370, val: 0x00060801 }, // MT_BBP(AGC, 28)
    BbpSwitch { bw_band: RF_A_BAND | RF_BW_20 | RF_BW_80, reg: 0x2370, val: 0x00060806 }, // MT_BBP(AGC, 28)
    BbpSwitch { bw_band: RF_A_BAND | RF_BW_20 | RF_BW_40 | RF_BW_80, reg: 0x2970, val: 0x0000008A }, // MT_BBP(RXO, 28)
    BbpSwitch { bw_band: RF_G_BAND | RF_BW_20 | RF_BW_40, reg: 0x237c, val: 0x00000E23 }, // MT_BBP(AGC, 31)
    BbpSwitch { bw_band: RF_A_BAND | RF_BW_20 | RF_BW_40 | RF_BW_80, reg: 0x237c, val: 0x00000E13 }, // MT_BBP(AGC, 31)
    BbpSwitch { bw_band: RF_G_BAND | RF_BW_20 | RF_BW_40, reg: 0x2380, val: 0x00003218 }, // MT_BBP(AGC, 32)
    BbpSwitch { bw_band: RF_A_BAND | RF_BW_20 | RF_BW_40 | RF_BW_80, reg: 0x2380, val: 0x0000181C }, // MT_BBP(AGC, 32)
    BbpSwitch { bw_band: RF_G_BAND | RF_BW_20 | RF_BW_40, reg: 0x2384, val: 0x00003240 }, // MT_BBP(AGC, 33)
    BbpSwitch { bw_band: RF_A_BAND | RF_BW_20 | RF_BW_40 | RF_BW_80, reg: 0x2384, val: 0x00003218 }, // MT_BBP(AGC, 33)
    BbpSwitch { bw_band: RF_G_BAND | RF_BW_20, reg: 0x238c, val: 0x11111616 }, // MT_BBP(AGC, 35)
    BbpSwitch { bw_band: RF_G_BAND | RF_BW_40, reg: 0x238c, val: 0x11111516 }, // MT_BBP(AGC, 35)
    BbpSwitch { bw_band: RF_A_BAND | RF_BW_20 | RF_BW_40 | RF_BW_80, reg: 0x238c, val: 0x11111111 }, // MT_BBP(AGC, 35)
    BbpSwitch { bw_band: RF_G_BAND | RF_BW_20, reg: 0x239c, val: 0x2A2A3036 }, // MT_BBP(AGC, 39)
    BbpSwitch { bw_band: RF_G_BAND | RF_BW_40, reg: 0x239c, val: 0x2A2A2C36 }, // MT_BBP(AGC, 39)
    BbpSwitch { bw_band: RF_A_BAND | RF_BW_20 | RF_BW_40 | RF_BW_80, reg: 0x239c, val: 0x2A2A2A2A }, // MT_BBP(AGC, 39)
    BbpSwitch { bw_band: RF_G_BAND | RF_BW_20, reg: 0x23ac, val: 0x27273438 }, // MT_BBP(AGC, 43)
    BbpSwitch { bw_band: RF_G_BAND | RF_BW_40, reg: 0x23ac, val: 0x27272D38 }, // MT_BBP(AGC, 43)
    BbpSwitch { bw_band: RF_A_BAND | RF_BW_20 | RF_BW_40 | RF_BW_80, reg: 0x23ac, val: 0x27271A1A }, // MT_BBP(AGC, 43)
    BbpSwitch { bw_band: RF_G_BAND | RF_BW_20 | RF_BW_40, reg: 0x23cc, val: 0x17171C1C }, // MT_BBP(AGC, 51)
    BbpSwitch { bw_band: RF_A_BAND | RF_BW_20 | RF_BW_40 | RF_BW_80, reg: 0x23cc, val: 0xFFFFFFFF }, // MT_BBP(AGC, 51)
    BbpSwitch { bw_band: RF_G_BAND | RF_BW_20, reg: 0x23d4, val: 0x26262A2F }, // MT_BBP(AGC, 53)
    BbpSwitch { bw_band: RF_G_BAND | RF_BW_40, reg: 0x23d4, val: 0x2626322F }, // MT_BBP(AGC, 53)
    BbpSwitch { bw_band: RF_A_BAND | RF_BW_20 | RF_BW_40 | RF_BW_80, reg: 0x23d4, val: 0xFFFFFFFF }, // MT_BBP(AGC, 53)
    BbpSwitch { bw_band: RF_G_BAND | RF_BW_20 | RF_BW_40, reg: 0x23dc, val: 0x40404040 }, // MT_BBP(AGC, 55)
    BbpSwitch { bw_band: RF_A_BAND | RF_BW_20 | RF_BW_40 | RF_BW_80, reg: 0x23dc, val: 0xFFFFFFFF }, // MT_BBP(AGC, 55)
    BbpSwitch { bw_band: RF_G_BAND | RF_BW_20 | RF_BW_40, reg: 0x23e8, val: 0x00001010 }, // MT_BBP(AGC, 58)
    BbpSwitch { bw_band: RF_A_BAND | RF_BW_20 | RF_BW_40 | RF_BW_80, reg: 0x23e8, val: 0x00000000 }, // MT_BBP(AGC, 58)
    BbpSwitch { bw_band: RF_G_BAND | RF_BW_20 | RF_BW_40, reg: 0x2800, val: 0x3D5000E0 }, // MT_BBP(RXFE, 0)
    BbpSwitch { bw_band: RF_A_BAND | RF_BW_20 | RF_BW_40 | RF_BW_80, reg: 0x2800, val: 0x895000E0 }, // MT_BBP(RXFE, 0)
];

/// MAC registers common to all mt76x02 parts — `common_mac_reg_table`
/// (`mt76x0/initvals_init.h:14`). Written first; [`MT76X0_MAC_REG_TABLE`]
/// then overrides the 7610-specific subset.
#[rustfmt::skip]
pub const COMMON_MAC_REG_TABLE: &[(u32, u32)] = &[
    (0x041c, 0xF8F0E8E0), // MT_BCN_OFFSET(0)
    (0x0420, 0x6F77D0C8), // MT_BCN_OFFSET(1)
    (0x1408, 0x0000013F), // MT_LEGACY_BASIC_RATE
    (0x140c, 0x00008003), // MT_HT_BASIC_RATE
    (0x1004, 0x00000000), // MT_MAC_SYS_CTRL
    (0x1400, 0x00017F97), // MT_RX_FILTR_CFG
    (0x1104, 0x00000209), // MT_BKOFF_SLOT_CFG
    (0x1330, 0x00000000), // MT_TX_SW_CFG0
    (0x1334, 0x00080606), // MT_TX_SW_CFG1
    (0x1350, 0x00001020), // MT_TX_LINK_CFG
    (0x1348, 0x000A2090), // MT_TX_TIMEOUT_CFG
    (0x1018, 0x000A1FFF), // MT_MAX_LEN_CFG; value = 0xa0fff | 0x00001000
    (0x102c, 0x7F031E46), // MT_LED_CFG
    (0x0408, 0x1FBF1F1F), // MT_PBF_TX_MAX_PCNT
    (0x040c, 0x0000FE9F), // MT_PBF_RX_MAX_PCNT
    (0x134c, 0x47D01F0F), // MT_TX_RETRY_CFG
    (0x1404, 0x00000013), // MT_AUTO_RSP_CFG
    (0x1364, 0x07F40003), // MT_CCK_PROT_CFG
    (0x1368, 0x07F42004), // MT_OFDM_PROT_CFG
    (0x0404, 0x00F40006), // MT_PBF_CFG
    (0x0208, 0x00000030), // MT_WPDMA_GLO_CFG
    (0x1374, 0x01742004), // MT_GF20_PROT_CFG
    (0x1378, 0x03F42084), // MT_GF40_PROT_CFG
    (0x136c, 0x01742004), // MT_MM20_PROT_CFG
    (0x1370, 0x03F42084), // MT_MM40_PROT_CFG
    (0x1340, 0x0000583F), // MT_TXOP_CTRL_CFG
    (0x1344, 0x00FFFF20), // MT_TX_RTS_CFG
    (0x1380, 0x002400CA), // MT_EXP_ACK_TIME
    (0x1608, 0x00000002), // MT_TXOP_HLDR_ET
    (0x1100, 0x33A41010), // MT_XIFS_TIME_CFG
    (0x1204, 0x00000000), // MT_PWR_PIN_CFG
];

/// MT7610-specific MAC registers — `mt76x0_mac_reg_table`
/// (`mt76x0/initvals_init.h:48`). Applied immediately after
/// [`COMMON_MAC_REG_TABLE`] and intentionally overwrites several of its rows.
///
/// Contains three rows upstream writes as bare addresses with no `MT_*` symbol and
/// no comment (`0xa44`, `0x150C`, `0x1238`). Their meaning is not determinable from
/// the tree; they are ported faithfully and flagged in the row comments.
#[rustfmt::skip]
pub const MT76X0_MAC_REG_TABLE: &[(u32, u32)] = &[
    (0x0124, 0xA0040080), // MT_IOCFG_6
    (0x0400, 0x00080C00), // MT_PBF_SYS_CTRL
    (0x0404, 0x77723C1F), // MT_PBF_CFG
    (0x0800, 0x00000001), // MT_FCE_PSE_CTRL
    (0x1030, 0xAAA99887), // MT_AMPDU_MAX_LEN_20M1S
    (0x1330, 0x00000601), // MT_TX_SW_CFG0
    (0x1334, 0x00040000), // MT_TX_SW_CFG1
    (0x1338, 0x00000000), // MT_TX_SW_CFG2
    (0x0a44, 0x00000000), // raw 0xa44 upstream (no MT_* symbol; purpose undocumented)
    (0x0260, 0x00000000), // MT_HEADER_TRANS_CTRL_REG
    (0x0250, 0x00000000), // MT_TSO_CTRL
    (0x1218, 0x00500055), // MT_BB_PA_MODE_CFG1
    (0x1220, 0x00500055), // MT_RF_PA_MODE_CFG1
    (0x13b0, 0x2F2F000C), // MT_TX_ALC_CFG_0
    (0x13c0, 0x00000000), // MT_TX0_BB_GAIN_ATTEN
    (0x1314, 0x3A3A3A3A), // MT_TX_PWR_CFG_0
    (0x1318, 0x3A3A3A3A), // MT_TX_PWR_CFG_1
    (0x131c, 0x3A3A3A3A), // MT_TX_PWR_CFG_2
    (0x1320, 0x3A3A3A3A), // MT_TX_PWR_CFG_3
    (0x1324, 0x3A3A3A3A), // MT_TX_PWR_CFG_4
    (0x13d4, 0x3A3A3A3A), // MT_TX_PWR_CFG_7
    (0x13d8, 0x0000003A), // MT_TX_PWR_CFG_8
    (0x13dc, 0x0000003A), // MT_TX_PWR_CFG_9
    (0x150c, 0x00000002), // raw 0x150C upstream (no MT_* symbol; purpose undocumented); ⚠ DUPLICATE: MT_PN_PAD_MODE below writes 0x3 to this same address and wins. Upstream does exactly this (initvals_init.h:72 vs :79); kept so the write sequence matches the in-tree driver
    (0x1238, 0x001700C8), // raw 0x1238 upstream (no MT_* symbol; purpose undocumented)
    (0x006c, 0x00A647B6), // MT_LDO_CTRL_0
    (0x0070, 0x6B006464), // MT_LDO_CTRL_1
    (0x140c, 0x00004003), // MT_HT_BASIC_RATE
    (0x1410, 0x000001FF), // MT_HT_CTRL_CFG
    (0x1608, 0x00000000), // MT_TXOP_HLDR_ET
    (0x150c, 0x00000003), // MT_PN_PAD_MODE; ⚠ DUPLICATE: overwrites the raw 0x150C row above (0x2 -> 0x3). This is the value the hardware ends up with
    (0x13e0, 0xE3F42004), // MT_TX_PROT_CFG6
    (0x13e4, 0xE3F42084), // MT_TX_PROT_CFG7
    (0x13e8, 0xE3F42104), // MT_TX_PROT_CFG8
    (0x1358, 0xEDCBA980), // MT_VHT_HT_FBK_CFG1
];

/// Baseband init values — `mt76x0_bbp_init_tab` (`mt76x0/initvals_init.h:86`).
/// Written after `mt76x0_phy_wait_bbp_ready` succeeds and before the
/// [`BBP_SWITCH_TAB`] subset (`mt76x0/init.c:87-103`); writing it earlier races the
/// BBP coming out of reset.
#[rustfmt::skip]
pub const MT76X0_BBP_INIT_TAB: &[(u32, u32)] = &[
    (0x2004, 0x00000002), // MT_BBP(CORE, 1)
    (0x2010, 0x00000000), // MT_BBP(CORE, 4)
    (0x2060, 0x00000000), // MT_BBP(CORE, 24)
    (0x2080, 0x4003000A), // MT_BBP(CORE, 32)
    (0x20a8, 0x00000000), // MT_BBP(CORE, 42)
    (0x20b0, 0x00000000), // MT_BBP(CORE, 44)
    (0x212c, 0x0FDE8081), // MT_BBP(IBI, 11)
    (0x2300, 0x00021400), // MT_BBP(AGC, 0)
    (0x2304, 0x00000003), // MT_BBP(AGC, 1)
    (0x2308, 0x003A6464), // MT_BBP(AGC, 2)
    (0x233c, 0x88A28CB8), // MT_BBP(AGC, 15)
    (0x2358, 0x00001E21), // MT_BBP(AGC, 22)
    (0x235c, 0x0000272C), // MT_BBP(AGC, 23)
    (0x2360, 0x00002F3A), // MT_BBP(AGC, 24)
    (0x2364, 0x8000005A), // MT_BBP(AGC, 25)
    (0x2368, 0x007C2005), // MT_BBP(AGC, 26)
    (0x2384, 0x00003238), // MT_BBP(AGC, 33)
    (0x2388, 0x000A0C0C), // MT_BBP(AGC, 34)
    (0x2394, 0x2121262C), // MT_BBP(AGC, 37)
    (0x23a4, 0x38383E45), // MT_BBP(AGC, 41)
    (0x23e4, 0x00001010), // MT_BBP(AGC, 57)
    (0x23ec, 0xBAA20E96), // MT_BBP(AGC, 59)
    (0x23fc, 0x00000001), // MT_BBP(AGC, 63)
    (0x2400, 0x00280403), // MT_BBP(TXC, 0)
    (0x2404, 0x00000000), // MT_BBP(TXC, 1)
    (0x2504, 0x00000012), // MT_BBP(RXC, 1)
    (0x2508, 0x00000011), // MT_BBP(RXC, 2)
    (0x250c, 0x00000005), // MT_BBP(RXC, 3)
    (0x2510, 0x00000000), // MT_BBP(RXC, 4)
    (0x2514, 0xF977C4EC), // MT_BBP(RXC, 5)
    (0x251c, 0x00000090), // MT_BBP(RXC, 7)
    (0x2620, 0x00000000), // MT_BBP(TXO, 8)
    (0x2700, 0x00000000), // MT_BBP(TXBE, 0)
    (0x2710, 0x00000004), // MT_BBP(TXBE, 4)
    (0x2718, 0x00000000), // MT_BBP(TXBE, 6)
    (0x2720, 0x00000014), // MT_BBP(TXBE, 8)
    (0x2724, 0x20000000), // MT_BBP(TXBE, 9)
    (0x2728, 0x00000000), // MT_BBP(TXBE, 10)
    (0x2730, 0x00000000), // MT_BBP(TXBE, 12)
    (0x2734, 0x00000000), // MT_BBP(TXBE, 13)
    (0x2738, 0x00000000), // MT_BBP(TXBE, 14)
    (0x273c, 0x00000000), // MT_BBP(TXBE, 15)
    (0x2740, 0x00000000), // MT_BBP(TXBE, 16)
    (0x2744, 0x00000000), // MT_BBP(TXBE, 17)
    (0x2804, 0x00008800), // MT_BBP(RXFE, 1)
    (0x280c, 0x00000000), // MT_BBP(RXFE, 3)
    (0x2810, 0x00000000), // MT_BBP(RXFE, 4)
    (0x2934, 0x00000192), // MT_BBP(RXO, 13)
    (0x2938, 0x00060612), // MT_BBP(RXO, 14)
    (0x293c, 0xC8321B18), // MT_BBP(RXO, 15)
    (0x2940, 0x0000001E), // MT_BBP(RXO, 16)
    (0x2944, 0x00000000), // MT_BBP(RXO, 17)
    (0x2948, 0xCC00A993), // MT_BBP(RXO, 18)
    (0x294c, 0xB9CB9CB9), // MT_BBP(RXO, 19)
    (0x2950, 0x26C00057), // MT_BBP(RXO, 20)
    (0x2954, 0x00000001), // MT_BBP(RXO, 21)
    (0x2960, 0x00000006), // MT_BBP(RXO, 24)
    (0x2970, 0x0000003F), // MT_BBP(RXO, 28)
];

/// DC-offset-cancellation calibration setup — `mt76x0_dcoc_tab`
/// (`mt76x0/initvals_init.h:147`). All rows are in the BBP `CAL` block
/// (base `0x2c00`). Written last in the BBP sequence (`mt76x0/init.c:105`).
///
/// Upstream gives no explanation for these nine values and no code reads them back;
/// they are the DCOC block's expected starting state. Ported as-is.
#[rustfmt::skip]
pub const MT76X0_DCOC_TAB: &[(u32, u32)] = &[
    (0x2cbc, 0x000010F0), // MT_BBP(CAL, 47)
    (0x2cc0, 0x00008080), // MT_BBP(CAL, 48)
    (0x2cc4, 0x00000F07), // MT_BBP(CAL, 49)
    (0x2cc8, 0x00000040), // MT_BBP(CAL, 50)
    (0x2ccc, 0x00000404), // MT_BBP(CAL, 51)
    (0x2cd0, 0x00080803), // MT_BBP(CAL, 52)
    (0x2cd4, 0x00000704), // MT_BBP(CAL, 53)
    (0x2cd8, 0x00002828), // MT_BBP(CAL, 54)
    (0x2cdc, 0x00005050), // MT_BBP(CAL, 55)
];

#[cfg(test)]
mod tests {
    use super::*;

    /// Row counts, hand-counted from the upstream headers. A dropped or duplicated
    /// row during a re-transcription is invisible by inspection but fatal on air,
    /// so it gets an assertion rather than trust.
    #[test]
    fn table_lengths_match_upstream() {
        // mt76x0/initvals.h:15-82 — counted 48 rows.
        assert_eq!(BBP_SWITCH_TAB.len(), 48);
        // mt76x0/initvals_init.h:15-45 — counted 31 rows.
        assert_eq!(COMMON_MAC_REG_TABLE.len(), 31);
        // mt76x0/initvals_init.h:49-83 — counted 35 rows.
        assert_eq!(MT76X0_MAC_REG_TABLE.len(), 35);
        // mt76x0/initvals_init.h:87-144 — counted 58 rows.
        assert_eq!(MT76X0_BBP_INIT_TAB.len(), 58);
        // mt76x0/initvals_init.h:148-156 — counted 9 rows.
        assert_eq!(MT76X0_DCOC_TAB.len(), 9);
    }

    /// No table may contain register address 0.
    ///
    /// `0x0000` is `MT_ASIC_VERSION` (`mt76x02_regs.h:15`), a read-only ID register
    /// — it is never an init target. An address of 0 therefore means a symbol
    /// failed to resolve during transcription and defaulted to nothing, which is
    /// exactly the silent shift this file's literal-address style exists to catch.
    #[test]
    fn no_table_contains_a_zero_address() {
        for row in BBP_SWITCH_TAB {
            assert_ne!(row.reg, 0, "BBP_SWITCH_TAB has a zero address");
        }
        for tab in [
            COMMON_MAC_REG_TABLE,
            MT76X0_MAC_REG_TABLE,
            MT76X0_BBP_INIT_TAB,
            MT76X0_DCOC_TAB,
        ] {
            for &(reg, _) in tab {
                assert_ne!(reg, 0, "a reg-pair table has a zero address");
            }
        }
    }

    /// Address-space sanity: BBP tables must land in the BBP window, MAC tables
    /// must not, and every address must be 4-byte aligned.
    ///
    /// This is the guard against a wrong `MT_BBP_*_BASE`: the BBP block runs
    /// `0x2000..0x3000` (`mt76x02_regs.h:604-617`), so a base typo that pushed rows
    /// out of that window — or a `n << 2` written as `n` — fails here instead of on
    /// the air.
    #[test]
    fn addresses_are_in_the_right_window_and_aligned() {
        for row in BBP_SWITCH_TAB {
            assert_eq!(row.reg & 3, 0, "unaligned BBP address {:#06x}", row.reg);
            assert!(
                (0x2000..0x3000).contains(&row.reg),
                "BBP_SWITCH_TAB address {:#06x} is outside the BBP window",
                row.reg
            );
        }
        for tab in [MT76X0_BBP_INIT_TAB, MT76X0_DCOC_TAB] {
            for &(reg, _) in tab {
                assert_eq!(reg & 3, 0, "unaligned BBP address {reg:#06x}");
                assert!(
                    (0x2000..0x3000).contains(&reg),
                    "BBP address {reg:#06x} is outside the BBP window"
                );
            }
        }
        for tab in [COMMON_MAC_REG_TABLE, MT76X0_MAC_REG_TABLE] {
            for &(reg, _) in tab {
                assert_eq!(reg & 3, 0, "unaligned MAC address {reg:#06x}");
                assert!(reg < 0x2000, "MAC address {reg:#06x} is in the BBP window");
            }
        }
    }

    /// Pin the three rows MEASURED on mds-o5p-1's MT7610U (see the module header).
    ///
    /// These are the only entries in this file confirmed against real silicon, so
    /// they are the anchor: `0x2308` in particular validates the `MT_BBP(AGC, n)`
    /// base arithmetic, not just one value.
    #[test]
    fn measured_rows_match_the_hardware_readback() {
        let find =
            |tab: &[(u32, u32)], reg: u32| tab.iter().find(|&&(r, _)| r == reg).map(|&(_, v)| v);
        // MT_BBP(AGC, 2) — read back 0x003A6464 from a kernel-driven monitor vif.
        assert_eq!(find(MT76X0_BBP_INIT_TAB, 0x2308), Some(0x003A6464));
        // MT_BKOFF_SLOT_CFG — read back 0x00000209.
        assert_eq!(find(COMMON_MAC_REG_TABLE, 0x1104), Some(0x0000_0209));
        // MT_TXOP_CTRL_CFG — read back 0x0000583F.
        assert_eq!(find(COMMON_MAC_REG_TABLE, 0x1340), Some(0x0000_583F));
    }

    /// The [`BbpSwitch::matches`] superset rule, pinned against upstream's two call
    /// sites so a "simplification" to `bw_band == query` or `& != 0` gets caught.
    #[test]
    #[rustfmt::skip]
    fn bbp_switch_selection_is_superset_not_equality() {
        let row_g_20_40 = BbpSwitch { bw_band: RF_G_BAND | RF_BW_20 | RF_BW_40, reg: 0x2310, val: 0 };
        let row_g_40 = BbpSwitch { bw_band: RF_G_BAND | RF_BW_40, reg: 0x2330, val: 0 };
        let row_a = BbpSwitch { bw_band: RF_A_BAND | RF_BW_20 | RF_BW_40 | RF_BW_80, reg: 0x2310, val: 0 };

        // A 2.4 GHz / 20 MHz query takes the row that covers both widths...
        assert!(row_g_20_40.matches(RF_G_BAND | RF_BW_20));
        // ...but not the 40 MHz-only row (this is the case `& != 0` would get wrong)...
        assert!(!row_g_40.matches(RF_G_BAND | RF_BW_20));
        // ...and not the A-band row.
        assert!(!row_a.matches(RF_G_BAND | RF_BW_20));

        // And a 40 MHz query takes both G-band rows (equality would miss the first).
        assert!(row_g_20_40.matches(RF_G_BAND | RF_BW_40));
        assert!(row_g_40.matches(RF_G_BAND | RF_BW_40));

        // The init-time query from mt76x0/init.c:101 selects exactly the G/20 rows.
        let n_init = BBP_SWITCH_TAB
            .iter()
            .filter(|r| r.matches(RF_G_BAND | RF_BW_20))
            .count();
        assert!(n_init > 0 && n_init < BBP_SWITCH_TAB.len());
    }
}
