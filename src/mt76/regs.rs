//! The **shared mt76x02 register map**, resolved to literal `u32` values.
//!
//! One map serves both parts on this rig: the **MT7610U** (`0e8d:7610`,
//! `mt76x0u`, 1×1 dual-band) ported in [`crate::mt76x0`], and the **MT7612U**
//! (`0e8d:7612`, `mt76x2u`, 2×2) already live in [`crate::mt7612`]. MediaTek's
//! `mt76x02` core is literally the same MAC/BBP block on both, which is why the
//! knob layer in [`super::knobs`] can be written once against
//! [`Mt76Regs`](super::Mt76Regs) instead of twice.
//!
//! Transcribed from the mainline mt76 tree, `drivers/net/wireless/mediatek/mt76`:
//!   * `mt76x02_regs.h` — the bulk of it (register offsets + bitfields),
//!   * `mt76x02_mcu.h` / `mt76x2/mcu.h` / `mt76x0/mcu.h` — the MCU block,
//!   * `mt76x0/phy.h` — the RF-bank addressing helpers and the mt76x0 RF field
//!     masks (RF registers are 8-bit, so those masks are `u8`).
//!
//! Every constant carries its upstream `file:line`. Upstream writes bitfields as
//! `BIT(n)` / `GENMASK(hi, lo)`; here each is the **literal mask**, with the bit
//! range restated in the comment so a reader can check the arithmetic without
//! re-deriving it. Nothing in this file is invented: where two upstream names
//! resolve to the same address (there are several), both names are kept and the
//! collision is called out rather than silently deduplicated.
//!
//! # MEASURED correctness oracle (mds-o5p-1's MT7610U, 2026-08-27)
//!
//! These came off the target silicon via `examples/mt76_oracle.rs`, not from
//! reading code, and any change to this file must keep agreeing with them:
//!
//! | what | measured | agrees with |
//! |---|---|---|
//! | [`MT_TSF_TIMER_DW0`] is the **LOW** TSF word, ticking at **1.000 MHz** | 1 µs/tick | ⚠ *contradicts* `mt76x02_usb_core.c:155-157`, see below |
//! | TSF only runs once [`MT_BEACON_TIME_CFG`] bit 16 ([`MT_BEACON_TIME_CFG_TIMER_EN`]) is set | reads 0 in kernel monitor | — |
//! | [`MT_CH_IDLE`] / [`MT_CH_BUSY`] are **read-and-clear µs** counters | (idle+busy)/elapsed = 1.00 over 100 ms | — |
//! | [`MT_RX_STAT_0`] / [`MT_RX_STAT_1`] are read-and-clear per window | — | — |
//! | [`MT_BKOFF_SLOT_CFG`] under a kernel monitor | `0x0000_0209` | `mt76x0/initvals_init.h:21` — bit-identical |
//! | [`MT_TXOP_CTRL_CFG`] under a kernel monitor | `0x0000_583f` | `mt76x0/initvals_init.h:40` — bit-identical |
//! | [`MT_EXT_CCA_CFG`] under a kernel monitor | `0x0000_f1e4` | `mt76x0/phy.c:917-921` group 0 + `ED_CCA_MASK=0xf` — exact |
//! | [`MT_RX_FILTR_CFG`] under a kernel monitor | `0x0000_1093` | `initvals_init.h:20` (`0x0001_7f97`) minus mac80211's monitor clears — exact, see [`measured`] |
//! | [`MT_MAC_SYS_CTRL`] under a kernel monitor | `0x0c` | `ENABLE_TX \| ENABLE_RX` |
//! | [`mt_bbp`]`(`[`MT_BBP_AGC_BASE`]`, 2)` under a kernel monitor | `0x003a_6464` | `mt76x02_mac.c:1126` (the *non*-mt76x2, ED-CCA-off branch) |
//!
//! ★ **`MT_TSF_TIMER_DW0` word order.** `mt76x02_usb_core.c:155-157` builds the
//! TSF as `tsf = (u64)dw0 << 32 | dw1`, i.e. it treats DW0 as the HIGH word.
//! That is **wrong on this silicon** — measured, DW0 is the low word and DW1 the
//! high word, so the correct assembly is `(dw1 as u64) << 32 | dw0 as u64`. Do
//! not copy the upstream expression. (Upstream only ever uses that value in a
//! `dev_dbg` print, which is presumably why the bug survived.)
//!
//! ★ **EP0 costs 151 µs per vendor-request round trip.** Every constant here is
//! reached through one of those. A register read is therefore fine in a channel
//! switch or a 100 ms sensing window and *never* fine on a per-frame path — the
//! [`measured::EP0_ROUND_TRIP_US`] budget is the reason the knob layer caches.
#![allow(dead_code)]

// ── Field helpers ────────────────────────────────────────────────────────────
// Upstream's FIELD_PREP/FIELD_GET, which the init tables and the PHY code lean
// on constantly. `mask` must be non-zero: a zero mask makes `trailing_zeros()`
// return 32 and the shift overflows (a panic in const eval, a debug panic at
// runtime). No caller in this port passes one.

/// Place `value` into the bit range described by `mask` (upstream `FIELD_PREP`).
pub const fn field_prep(mask: u32, value: u32) -> u32 {
    (value << mask.trailing_zeros()) & mask
}

/// Extract the bit range described by `mask` from `value` (upstream `FIELD_GET`).
pub const fn field_get(mask: u32, value: u32) -> u32 {
    (value & mask) >> mask.trailing_zeros()
}

/// 8-bit `FIELD_PREP`, for the RF-bank registers (which are one byte wide).
pub const fn field_prep_u8(mask: u8, value: u8) -> u8 {
    (value << mask.trailing_zeros()) & mask
}

/// 8-bit `FIELD_GET`, for the RF-bank registers.
pub const fn field_get_u8(mask: u8, value: u8) -> u8 {
    (value & mask) >> mask.trailing_zeros()
}

// ── System / identity / power ────────────────────────────────────────────────

/// ASIC version + revision. The top half is the part number (`0x7610` /
/// `0x7612`), the bottom half the revision — this is how the backend tells the
/// two silicon families apart before any other register is touched.
pub const MT_ASIC_VERSION: u32 = 0x0000; // mt76x02_regs.h:9

/// Silicon revision E3, from the low half of [`MT_ASIC_VERSION`]. mt76x2u picks
/// a different DLM load address at ≥ E3; recorded here because the same decode
/// serves both parts.
pub const MT76XX_REV_E3: u32 = 0x22; // mt76x02_regs.h:11
/// Silicon revision E4 — see [`MT76XX_REV_E3`].
pub const MT76XX_REV_E4: u32 = 0x33; // mt76x02_regs.h:12

/// Combo (WLAN/BT) control. Polled during power-on for the two ready bits below:
/// until the crystal is ready and the PLL is locked, MAC register writes do not
/// stick.
pub const MT_CMB_CTRL: u32 = 0x0020; // mt76x02_regs.h:14
pub const MT_CMB_CTRL_XTAL_RDY: u32 = 0x0040_0000; // BIT(22) — mt76x02_regs.h:15
pub const MT_CMB_CTRL_PLL_LD: u32 = 0x0080_0000; // BIT(23) — mt76x02_regs.h:16

/// Bluetooth-coexistence config word 0. Cleared of [`MT_COEXCFG0_COEX_EN`] during
/// mt76x0 init so the (absent) BT side cannot arbitrate our airtime away.
pub const MT_COEXCFG0: u32 = 0x0040; // mt76x02_regs.h:30
pub const MT_COEXCFG0_COEX_EN: u32 = 0x0000_0001; // BIT(0) — mt76x02_regs.h:31
/// Coex word 3. mt76x0 init writes it; upstream gives no reason and neither can
/// we — ported faithfully, meaning unknown.
pub const MT_COEXCFG3: u32 = 0x004c; // mt76x02_regs.h:38

/// LDO control 0/1 — written by the mt76x0 power-on path. Upstream supplies no
/// field breakdown, so the values are opaque magic; ported as-is.
pub const MT_LDO_CTRL_0: u32 = 0x006c; // mt76x02_regs.h:40
pub const MT_LDO_CTRL_1: u32 = 0x0070; // mt76x02_regs.h:41

/// The WLAN function gate: enable, clock-enable and RF/digital reset for the
/// whole WLAN subsystem. This is the first register the bring-up writes.
pub const MT_WLAN_FUN_CTRL: u32 = 0x0080; // mt76x02_regs.h:33
pub const MT_WLAN_FUN_CTRL_WLAN_EN: u32 = 0x0000_0001; // BIT(0) — mt76x02_regs.h:34
pub const MT_WLAN_FUN_CTRL_WLAN_CLK_EN: u32 = 0x0000_0002; // BIT(1) — mt76x02_regs.h:35
pub const MT_WLAN_FUN_CTRL_WLAN_RESET_RF: u32 = 0x0000_0004; // BIT(2) — mt76x02_regs.h:36
/// ⚠ BIT(3) is **overloaded by part**: on MT76x0 it is the digital WLAN reset,
/// on MT76x2 it is a 20 MHz clock enable. This port is MT76x0, so
/// [`MT_WLAN_FUN_CTRL_WLAN_RESET`] is the meaning that applies; the mt76x2 alias
/// is kept so the shared code can be read against either datasheet.
pub const MT_WLAN_FUN_CTRL_WLAN_RESET: u32 = 0x0000_0008; // BIT(3), MT76x0 — mt76x02_regs.h:43
pub const MT_WLAN_FUN_CTRL_CSR_F20M_CKEN: u32 = 0x0000_0008; // BIT(3), MT76x2 — mt76x02_regs.h:44
pub const MT_WLAN_FUN_CTRL_PCIE_CLK_REQ: u32 = 0x0000_0010; // BIT(4) — mt76x02_regs.h:46
pub const MT_WLAN_FUN_CTRL_FRC_WL_ANT_SEL: u32 = 0x0000_0020; // BIT(5) — mt76x02_regs.h:47
pub const MT_WLAN_FUN_CTRL_INV_ANT_SEL: u32 = 0x0000_0040; // BIT(6) — mt76x02_regs.h:48
pub const MT_WLAN_FUN_CTRL_WAKE_HOST: u32 = 0x0000_0080; // BIT(7) — mt76x02_regs.h:49
pub const MT_WLAN_FUN_CTRL_THERM_RST: u32 = 0x0000_0100; // BIT(8), MT76x2 — mt76x02_regs.h:51
pub const MT_WLAN_FUN_CTRL_THERM_CKEN: u32 = 0x0000_0200; // BIT(9), MT76x2 — mt76x02_regs.h:52
/// ⚠ On MT76x0 bits 8-31 are the GPIO block, which **overlaps** the MT76x2
/// thermal bits above. Use only the set that matches the silicon in hand.
pub const MT_WLAN_FUN_CTRL_GPIO_IN: u32 = 0x0000_ff00; // GENMASK(15,8), MT76x0 — mt76x02_regs.h:54
pub const MT_WLAN_FUN_CTRL_GPIO_OUT: u32 = 0x00ff_0000; // GENMASK(23,16), MT76x0 — mt76x02_regs.h:55
pub const MT_WLAN_FUN_CTRL_GPIO_OUT_EN: u32 = 0xff00_0000; // GENMASK(31,24), MT76x0 — mt76x02_regs.h:56

// ── Crystal / clock trim (MT76x0 only) ───────────────────────────────────────
// The XO block is where a frequency-discipline knob would actuate (see
// `crate::freq_discipline`); recorded now, unused by the initial port.

pub const MT_XO_CTRL0: u32 = 0x0100; // mt76x02_regs.h:61
pub const MT_XO_CTRL1: u32 = 0x0104; // mt76x02_regs.h:62
/// ⚠ Same address as [`MT_XO_CTRL1`] — upstream names 0x0104 twice. The EEPROM
/// path calls it `MT_CSR_EE_CFG1` (`mt76x0/eeprom.c` reads the EE config from
/// it), the clock path calls it `MT_XO_CTRL1`. One register, two readers.
pub const MT_CSR_EE_CFG1: u32 = 0x0104; // mt76x02_regs.h:59
pub const MT_XO_CTRL2: u32 = 0x0108; // mt76x02_regs.h:63
pub const MT_XO_CTRL3: u32 = 0x010c; // mt76x02_regs.h:64
pub const MT_XO_CTRL4: u32 = 0x0110; // mt76x02_regs.h:65
pub const MT_XO_CTRL5: u32 = 0x0114; // mt76x02_regs.h:67
pub const MT_XO_CTRL5_C2_VAL: u32 = 0x0000_7f00; // GENMASK(14,8) — mt76x02_regs.h:68
pub const MT_XO_CTRL6: u32 = 0x0118; // mt76x02_regs.h:70
pub const MT_XO_CTRL6_C2_CTRL: u32 = 0x0000_7f00; // GENMASK(14,8) — mt76x02_regs.h:71
pub const MT_XO_CTRL7: u32 = 0x011c; // mt76x02_regs.h:73

/// GPIO/IO configuration 6 — touched by the mt76x0 init with an opaque value.
/// Ported faithfully; upstream states no reason.
pub const MT_IOCFG_6: u32 = 0x0124; // mt76x02_regs.h:75

// ── eFuse (EEPROM backing store) ─────────────────────────────────────────────
// The EEPROM content is read 16 bytes at a time: program the block address into
// AIN, set KICK, poll KICK clear, then read the four MT_EFUSE_DATA words.

pub const MT_EFUSE_CTRL: u32 = 0x0024; // mt76x02_regs.h:18
pub const MT_EFUSE_CTRL_AOUT: u32 = 0x0000_003f; // GENMASK(5,0) — mt76x02_regs.h:19
pub const MT_EFUSE_CTRL_MODE: u32 = 0x0000_00c0; // GENMASK(7,6) — mt76x02_regs.h:20
pub const MT_EFUSE_CTRL_LDO_OFF_TIME: u32 = 0x0000_3f00; // GENMASK(13,8) — mt76x02_regs.h:21
pub const MT_EFUSE_CTRL_LDO_ON_TIME: u32 = 0x0000_c000; // GENMASK(15,14) — mt76x02_regs.h:22
pub const MT_EFUSE_CTRL_AIN: u32 = 0x03ff_0000; // GENMASK(25,16) — mt76x02_regs.h:23
pub const MT_EFUSE_CTRL_KICK: u32 = 0x4000_0000; // BIT(30) — mt76x02_regs.h:24
pub const MT_EFUSE_CTRL_SEL: u32 = 0x8000_0000; // BIT(31) — mt76x02_regs.h:25

/// Base of the four-word (16 byte) eFuse read window. mt76x02_regs.h:27
pub const MT_EFUSE_DATA_BASE: u32 = 0x0028;
/// `MT_EFUSE_DATA(_n)` — mt76x02_regs.h:28. `_n` is 0..=3.
pub const fn mt_efuse_data(n: u32) -> u32 {
    MT_EFUSE_DATA_BASE + (n << 2)
}

// ── USB DMA ──────────────────────────────────────────────────────────────────

/// USB DMA config. ★ On **MT76x0 this is plain MMIO at 0x0238** and is reached
/// with an ordinary register read/write (`mt76x0/usb.c:50,59` and
/// `mt76x0/usb_mcu.c:92`). On MT76x2U the equivalent lives in USB **CFG space**
/// at [`MT_USB_U3DMA_CFG`] behind `MT_VEND_ADDR(CFG, …)`
/// (`mt76x2/usb_init.c:15`). Same bitfields, different door — which is exactly
/// the kind of divergence the [`Mt76Regs`](super::Mt76Regs) seam exists to hide.
pub const MT_USB_DMA_CFG: u32 = 0x0238; // mt76x02_regs.h:161
/// The MT76x2U CFG-space address of the same config word. mt76x02_regs.h:77
pub const MT_USB_U3DMA_CFG: u32 = 0x9018;

// The bitfields below are shared by both addresses (mt76x02_regs.h:78-90).
pub const MT_USB_DMA_CFG_RX_BULK_AGG_TOUT: u32 = 0x0000_00ff; // GENMASK(7,0) — :78
pub const MT_USB_DMA_CFG_RX_BULK_AGG_LMT: u32 = 0x0000_ff00; // GENMASK(15,8) — :79
pub const MT_USB_DMA_CFG_UDMA_TX_WL_DROP: u32 = 0x0001_0000; // BIT(16) — :80
pub const MT_USB_DMA_CFG_WAKE_UP_EN: u32 = 0x0002_0000; // BIT(17) — :81
pub const MT_USB_DMA_CFG_RX_DROP_OR_PAD: u32 = 0x0004_0000; // BIT(18) — :82
pub const MT_USB_DMA_CFG_TX_CLR: u32 = 0x0008_0000; // BIT(19) — :83
pub const MT_USB_DMA_CFG_TXOP_HALT: u32 = 0x0010_0000; // BIT(20) — :84
pub const MT_USB_DMA_CFG_RX_BULK_AGG_EN: u32 = 0x0020_0000; // BIT(21) — :85
pub const MT_USB_DMA_CFG_RX_BULK_EN: u32 = 0x0040_0000; // BIT(22) — :86
pub const MT_USB_DMA_CFG_TX_BULK_EN: u32 = 0x0080_0000; // BIT(23) — :87
/// One bit per bulk-OUT endpoint that the hardware may use. Six OUT endpoints
/// exist on this part (MEASURED: `0x04..0x09`), so the full mask is `0x3f`.
pub const MT_USB_DMA_CFG_EP_OUT_VALID: u32 = 0x3f00_0000; // GENMASK(29,24) — :88
pub const MT_USB_DMA_CFG_RX_BUSY: u32 = 0x4000_0000; // BIT(30) — :89
pub const MT_USB_DMA_CFG_TX_BUSY: u32 = 0x8000_0000; // BIT(31) — :90

// ── MTCMOS power island (MT76x0) ─────────────────────────────────────────────
// Note the 5-nibble address: this one is above the 64 KiB MAC window and is
// reached through the same vendor request with a non-zero wValue.

pub const MT_WLAN_MTC_CTRL: u32 = 0x0001_0148; // mt76x02_regs.h:92
pub const MT_WLAN_MTC_CTRL_MTCMOS_PWR_UP: u32 = 0x0000_0001; // BIT(0) — :93
pub const MT_WLAN_MTC_CTRL_PWR_ACK: u32 = 0x0000_1000; // BIT(12) — :94
pub const MT_WLAN_MTC_CTRL_PWR_ACK_S: u32 = 0x0000_2000; // BIT(13) — :95
pub const MT_WLAN_MTC_CTRL_BBP_MEM_PD: u32 = 0x000f_0000; // GENMASK(19,16) — :96
pub const MT_WLAN_MTC_CTRL_PBF_MEM_PD: u32 = 0x0010_0000; // BIT(20) — :97
pub const MT_WLAN_MTC_CTRL_FCE_MEM_PD: u32 = 0x0020_0000; // BIT(21) — :98
pub const MT_WLAN_MTC_CTRL_TSO_MEM_PD: u32 = 0x0040_0000; // BIT(22) — :99
pub const MT_WLAN_MTC_CTRL_BBP_MEM_RB: u32 = 0x0100_0000; // BIT(24) — :100
pub const MT_WLAN_MTC_CTRL_PBF_MEM_RB: u32 = 0x0200_0000; // BIT(25) — :101
pub const MT_WLAN_MTC_CTRL_FCE_MEM_RB: u32 = 0x0400_0000; // BIT(26) — :102
pub const MT_WLAN_MTC_CTRL_TSO_MEM_RB: u32 = 0x0800_0000; // BIT(27) — :103
pub const MT_WLAN_MTC_CTRL_STATE_UP: u32 = 0x1000_0000; // BIT(28) — :104

// ── Interrupts (PCIe-relevant; USB polls instead) ────────────────────────────
// The USB path never enables these, but mt76x0 init writes MT_INT_MASK_CSR = 0
// to make sure nothing is armed, so the block is transcribed.

pub const MT_INT_SOURCE_CSR: u32 = 0x0200; // mt76x02_regs.h:106
pub const MT_INT_MASK_CSR: u32 = 0x0204; // mt76x02_regs.h:107

/// `MT_INT_RX_DONE(_n)` — mt76x02_regs.h:109.
pub const fn mt_int_rx_done(n: u32) -> u32 {
    1 << n
}
pub const MT_INT_RX_DONE_ALL: u32 = 0x0000_0003; // GENMASK(1,0) — :110
pub const MT_INT_TX_DONE_ALL: u32 = 0x0000_3ff0; // GENMASK(13,4) — :111
/// `MT_INT_TX_DONE(_n)` — mt76x02_regs.h:112.
pub const fn mt_int_tx_done(n: u32) -> u32 {
    1 << (n + 4)
}
pub const MT_INT_RX_COHERENT: u32 = 0x0001_0000; // BIT(16) — :113
pub const MT_INT_TX_COHERENT: u32 = 0x0002_0000; // BIT(17) — :114
pub const MT_INT_ANY_COHERENT: u32 = 0x0004_0000; // BIT(18) — :115
pub const MT_INT_MCU_CMD: u32 = 0x0008_0000; // BIT(19) — :116
pub const MT_INT_TBTT: u32 = 0x0010_0000; // BIT(20) — :117
pub const MT_INT_PRE_TBTT: u32 = 0x0020_0000; // BIT(21) — :118
pub const MT_INT_TX_STAT: u32 = 0x0040_0000; // BIT(22) — :119
pub const MT_INT_AUTO_WAKEUP: u32 = 0x0080_0000; // BIT(23) — :120
pub const MT_INT_GPTIMER: u32 = 0x0100_0000; // BIT(24) — :121
pub const MT_INT_RXDELAYINT: u32 = 0x0400_0000; // BIT(26) — :122
pub const MT_INT_TXDELAYINT: u32 = 0x0800_0000; // BIT(27) — :123

// ── WPDMA ────────────────────────────────────────────────────────────────────
// mt76x0/usb.c polls TX_DMA_BUSY / RX_DMA_BUSY here when stopping the DMA and
// sets TX_DMA_EN / RX_DMA_EN when starting it — the USB path uses this register
// even though the ring registers themselves are PCIe-only.

pub const MT_WPDMA_GLO_CFG: u32 = 0x0208; // mt76x02_regs.h:125
pub const MT_WPDMA_GLO_CFG_TX_DMA_EN: u32 = 0x0000_0001; // BIT(0) — :126
pub const MT_WPDMA_GLO_CFG_TX_DMA_BUSY: u32 = 0x0000_0002; // BIT(1) — :127
pub const MT_WPDMA_GLO_CFG_RX_DMA_EN: u32 = 0x0000_0004; // BIT(2) — :128
pub const MT_WPDMA_GLO_CFG_RX_DMA_BUSY: u32 = 0x0000_0008; // BIT(3) — :129
pub const MT_WPDMA_GLO_CFG_DMA_BURST_SIZE: u32 = 0x0000_0030; // GENMASK(5,4) — :130
pub const MT_WPDMA_GLO_CFG_TX_WRITEBACK_DONE: u32 = 0x0000_0040; // BIT(6) — :131
pub const MT_WPDMA_GLO_CFG_BIG_ENDIAN: u32 = 0x0000_0080; // BIT(7) — :132
pub const MT_WPDMA_GLO_CFG_HDR_SEG_LEN: u32 = 0x0000_ff00; // GENMASK(15,8) — :133
pub const MT_WPDMA_GLO_CFG_CLK_GATE_DIS: u32 = 0x4000_0000; // BIT(30) — :134
pub const MT_WPDMA_GLO_CFG_RX_2B_OFFSET: u32 = 0x8000_0000; // BIT(31) — :135

pub const MT_WPDMA_RST_IDX: u32 = 0x020c; // mt76x02_regs.h:137
pub const MT_WPDMA_DELAY_INT_CFG: u32 = 0x0210; // mt76x02_regs.h:139

pub const MT_TX_RING_BASE: u32 = 0x0300; // mt76x02_regs.h:169
pub const MT_RX_RING_BASE: u32 = 0x03c0; // mt76x02_regs.h:170
/// Hardware TX queue index the MCU's in-band commands ride on. mt76x02_regs.h:172
pub const MT_TX_HW_QUEUE_MCU: u32 = 8;
/// Hardware TX queue index for management frames. mt76x02_regs.h:173
pub const MT_TX_HW_QUEUE_MGMT: u32 = 9;

// ── WMM (global per-AC parameters) ───────────────────────────────────────────
// Four ACs are packed as nibbles into each of AIFSN/CWMIN/CWMAX, and as 16-bit
// halves into a pair of TXOP words. The per-AC copy of the same knobs also
// exists at MT_EDCA_CFG_AC(n) below; the mt76x0 init writes both.

pub const MT_WMM_AIFSN: u32 = 0x0214; // mt76x02_regs.h:141
pub const MT_WMM_AIFSN_MASK: u32 = 0x0000_000f; // GENMASK(3,0) — :142
/// `MT_WMM_AIFSN_SHIFT(_n)` — mt76x02_regs.h:143.
pub const fn mt_wmm_aifsn_shift(n: u32) -> u32 {
    n * 4
}

pub const MT_WMM_CWMIN: u32 = 0x0218; // mt76x02_regs.h:145
pub const MT_WMM_CWMIN_MASK: u32 = 0x0000_000f; // GENMASK(3,0) — :146
/// `MT_WMM_CWMIN_SHIFT(_n)` — mt76x02_regs.h:147.
pub const fn mt_wmm_cwmin_shift(n: u32) -> u32 {
    n * 4
}

pub const MT_WMM_CWMAX: u32 = 0x021c; // mt76x02_regs.h:149
pub const MT_WMM_CWMAX_MASK: u32 = 0x0000_000f; // GENMASK(3,0) — :150
/// `MT_WMM_CWMAX_SHIFT(_n)` — mt76x02_regs.h:151.
pub const fn mt_wmm_cwmax_shift(n: u32) -> u32 {
    n * 4
}

pub const MT_WMM_TXOP_BASE: u32 = 0x0220; // mt76x02_regs.h:153
/// `MT_WMM_TXOP(_n)` — mt76x02_regs.h:154. Two ACs share each word.
pub const fn mt_wmm_txop(n: u32) -> u32 {
    MT_WMM_TXOP_BASE + ((n / 2) << 2)
}
/// `MT_WMM_TXOP_SHIFT(_n)` — mt76x02_regs.h:155.
pub const fn mt_wmm_txop_shift(n: u32) -> u32 {
    (n & 1) * 16
}
pub const MT_WMM_TXOP_MASK: u32 = 0x0000_ffff; // GENMASK(15,0) — :156

/// ⚠ MT76x0-only name for 0x0230, which on the FCE path is
/// [`MT_FCE_DMA_ADDR`]. Upstream defines both at the same address on consecutive
/// lines, and **both uses are real**: `mt76x0/init.c:133` does
/// `rmw(MT_WMM_CTRL, 0x3ff, 0x201)` as ordinary MMIO, while the firmware
/// download writes the FCE DMA descriptor to the same offset through the
/// `MT_VEND_WRITE_FCE` vendor request (`mt76x02_usb_mcu.c:15,232`), i.e. a
/// different door onto a different meaning. Which one you get depends on the
/// transfer you use, not on the address.
pub const MT_WMM_CTRL: u32 = 0x0230; // mt76x02_regs.h:158

// ── FCE (the frame/command engine that fronts the USB bulk pipes) ────────────

/// FCE DMA descriptor address, written during the firmware download. ⚠ Same
/// address as [`MT_WMM_CTRL`] — see that constant for why both are real.
/// Reached through the `MT_VEND_WRITE_FCE` vendor request, **not** a plain
/// register write; `mt76x02_usb_mcu.c:15` re-defines the same 0x0230 locally
/// and `:232` writes `dst_addr` to it. mt76x02_regs.h:159
pub const MT_FCE_DMA_ADDR: u32 = 0x0230;
/// Length half of the same descriptor. Upstream writes `len << 16`
/// (`mt76x02_usb_mcu.c:235`), i.e. the length lives in the **upper** half word.
/// mt76x02_regs.h:160
pub const MT_FCE_DMA_LEN: u32 = 0x0234;

pub const MT_TSO_CTRL: u32 = 0x0250; // mt76x02_regs.h:163
pub const MT_HEADER_TRANS_CTRL_REG: u32 = 0x0260; // mt76x02_regs.h:164

/// Microsecond-cycle configuration: how many MAC clocks make one µs. The
/// [`MT_CH_IDLE`] / [`MT_CH_BUSY`] counters and the TSF are derived from this,
/// which is why they came out at exactly 1.000 MHz on the measurement.
pub const MT_US_CYC_CFG: u32 = 0x02a4; // mt76x02_regs.h:166
pub const MT_US_CYC_CNT: u32 = 0x0000_00ff; // GENMASK(7,0) — :167

pub const MT_FCE_PSE_CTRL: u32 = 0x0800; // mt76x02_regs.h:242
pub const MT_FCE_PARAMETERS: u32 = 0x0804; // mt76x02_regs.h:243
pub const MT_FCE_CSO: u32 = 0x0808; // mt76x02_regs.h:244

/// L2 padding/stuffing control. `WR_MPDU_LEN_EN` is the bit the mt76x0 init
/// clears so the hardware does not rewrite the MPDU length field of a frame we
/// injected — load-bearing for host-built frames.
pub const MT_FCE_L2_STUFF: u32 = 0x080c; // mt76x02_regs.h:246
pub const MT_FCE_L2_STUFF_HT_L2_EN: u32 = 0x0000_0001; // BIT(0) — :247
pub const MT_FCE_L2_STUFF_QOS_L2_EN: u32 = 0x0000_0002; // BIT(1) — :248
pub const MT_FCE_L2_STUFF_RX_STUFF_EN: u32 = 0x0000_0004; // BIT(2) — :249
pub const MT_FCE_L2_STUFF_TX_STUFF_EN: u32 = 0x0000_0008; // BIT(3) — :250
pub const MT_FCE_L2_STUFF_WR_MPDU_LEN_EN: u32 = 0x0000_0010; // BIT(4) — :251
pub const MT_FCE_L2_STUFF_MVINV_BSWAP: u32 = 0x0000_0020; // BIT(5) — :252
pub const MT_FCE_L2_STUFF_TS_CMD_QSEL_EN: u32 = 0x0000_ff00; // GENMASK(15,8) — :253
pub const MT_FCE_L2_STUFF_TS_LEN_EN: u32 = 0x00ff_0000; // GENMASK(23,16) — :254
pub const MT_FCE_L2_STUFF_OTHER_PORT: u32 = 0x0300_0000; // GENMASK(25,24) — :255

pub const MT_FCE_WLAN_FLOW_CONTROL1: u32 = 0x0824; // mt76x02_regs.h:257

pub const MT_TX_CPU_FROM_FCE_BASE_PTR: u32 = 0x09a0; // mt76x02_regs.h:259
pub const MT_TX_CPU_FROM_FCE_MAX_COUNT: u32 = 0x09a4; // mt76x02_regs.h:260
/// Descriptor-index doorbell: written 1 after each firmware chunk to hand it to
/// the FCE. The [`crate::mt7612`] backend calls this same register
/// `MT_FCE_PSE_CTRL_GO`; the upstream name is the one below and both refer to
/// 0x09a8. mt76x02_regs.h:261
pub const MT_TX_CPU_FROM_FCE_CPU_DESC_IDX: u32 = 0x09a8;
/// Alias for [`MT_TX_CPU_FROM_FCE_CPU_DESC_IDX`], matching the name the sibling
/// MT7612U backend already uses so the two ports read alike.
pub const MT_FCE_PSE_CTRL_GO: u32 = 0x09a8;

pub const MT_FCE_PDMA_GLOBAL_CONF: u32 = 0x09c4; // mt76x02_regs.h:262
pub const MT_FCE_SKIP_FS: u32 = 0x0a6c; // mt76x02_regs.h:263
pub const MT_PAUSE_ENABLE_CONTROL1: u32 = 0x0a38; // mt76x02_regs.h:265

// ── PBF (packet buffer / queue enables) ──────────────────────────────────────

pub const MT_PBF_SYS_CTRL: u32 = 0x0400; // mt76x02_regs.h:175
pub const MT_PBF_SYS_CTRL_MCU_RESET: u32 = 0x0000_0001; // BIT(0) — :176
pub const MT_PBF_SYS_CTRL_DMA_RESET: u32 = 0x0000_0002; // BIT(1) — :177
pub const MT_PBF_SYS_CTRL_MAC_RESET: u32 = 0x0000_0004; // BIT(2) — :178
pub const MT_PBF_SYS_CTRL_PBF_RESET: u32 = 0x0000_0008; // BIT(3) — :179
pub const MT_PBF_SYS_CTRL_ASY_RESET: u32 = 0x0000_0010; // BIT(4) — :180

pub const MT_PBF_CFG: u32 = 0x0404; // mt76x02_regs.h:182
pub const MT_PBF_CFG_TX0Q_EN: u32 = 0x0000_0001; // BIT(0) — :183
pub const MT_PBF_CFG_TX1Q_EN: u32 = 0x0000_0002; // BIT(1) — :184
pub const MT_PBF_CFG_TX2Q_EN: u32 = 0x0000_0004; // BIT(2) — :185
pub const MT_PBF_CFG_TX3Q_EN: u32 = 0x0000_0008; // BIT(3) — :186
pub const MT_PBF_CFG_RX0Q_EN: u32 = 0x0000_0010; // BIT(4) — :187
pub const MT_PBF_CFG_RX_DROP_EN: u32 = 0x0000_0100; // BIT(8) — :188

pub const MT_PBF_TX_MAX_PCNT: u32 = 0x0408; // mt76x02_regs.h:190
pub const MT_PBF_RX_MAX_PCNT: u32 = 0x040c; // mt76x02_regs.h:191

/// Base of the per-beacon-slot offset table (into the beacon SRAM at
/// [`MT_BEACON_BASE`]). mt76x02_regs.h:193
pub const MT_BCN_OFFSET_BASE: u32 = 0x041c;
/// `MT_BCN_OFFSET(_n)` — mt76x02_regs.h:194.
pub const fn mt_bcn_offset(n: u32) -> u32 {
    MT_BCN_OFFSET_BASE + (n << 2)
}

pub const MT_RXQ_STA: u32 = 0x0430; // mt76x02_regs.h:196
pub const MT_TXQ_STA: u32 = 0x0434; // mt76x02_regs.h:197

// ── RF register access ───────────────────────────────────────────────────────
// Two distinct doors exist. MT_RF_CSR_CFG is the legacy one-byte-at-a-time CSR
// path (bank/reg/data + KICK, poll KICK clear); MT_RF_CTRL/MT_RF_DATA_* is the
// newer wide path. mt76x0 reaches the RF banks through the MCU register-pair
// protocol with the MT_MCU_MEMMAP_RF base instead of either of these, but the
// CSR path is still used for a few direct pokes.

pub const MT_RF_CSR_CFG: u32 = 0x0500; // mt76x02_regs.h:198
pub const MT_RF_CSR_CFG_DATA: u32 = 0x0000_00ff; // GENMASK(7,0) — :199
pub const MT_RF_CSR_CFG_REG_ID: u32 = 0x0000_7f00; // GENMASK(14,8) — :200
pub const MT_RF_CSR_CFG_REG_BANK: u32 = 0x0003_8000; // GENMASK(17,15) — :201
pub const MT_RF_CSR_CFG_WR: u32 = 0x4000_0000; // BIT(30) — :202
pub const MT_RF_CSR_CFG_KICK: u32 = 0x8000_0000; // BIT(31) — :203

pub const MT_RF_BYPASS_0: u32 = 0x0504; // mt76x02_regs.h:205
pub const MT_RF_BYPASS_1: u32 = 0x0508; // mt76x02_regs.h:206
pub const MT_RF_SETTING_0: u32 = 0x050c; // mt76x02_regs.h:207
pub const MT_RF_MISC: u32 = 0x0518; // mt76x02_regs.h:209
pub const MT_RF_DATA_WRITE: u32 = 0x0524; // mt76x02_regs.h:210

pub const MT_RF_CTRL: u32 = 0x0528; // mt76x02_regs.h:212
pub const MT_RF_CTRL_ADDR: u32 = 0x0000_0fff; // GENMASK(11,0) — :213
pub const MT_RF_CTRL_WRITE: u32 = 0x0000_1000; // BIT(12) — :214
pub const MT_RF_CTRL_BUSY: u32 = 0x0000_2000; // BIT(13) — :215
pub const MT_RF_CTRL_IDX: u32 = 0x0001_0000; // BIT(16) — :216

pub const MT_RF_DATA_READ: u32 = 0x052c; // mt76x02_regs.h:218

// ── MCU ──────────────────────────────────────────────────────────────────────

pub const MT_MCU_RESET_CTL: u32 = 0x070c; // mt76x02_mcu.h:11
/// MT76x2's CPU control / clock gate. Not written by the mt76x0 path (its ROM
/// takes a different route), recorded because the shared code names it.
pub const MT_MCU_CPU_CTL: u32 = 0x0704; // mt76x2/mcu.h:12
pub const MT_MCU_CLOCK_CTL: u32 = 0x0708; // mt76x2/mcu.h:13
pub const MT_MCU_INT_LEVEL: u32 = 0x0718; // mt76x02_mcu.h:12

/// Firmware-alive mailbox: `mt76x0_firmware_running()` is exactly
/// `rr(MT_MCU_COM_REG0) == 1` (`mt76x0/mcu.h:43`), so this is the one register
/// the MCU load polls.
pub const MT_MCU_COM_REG0: u32 = 0x0730; // mt76x02_mcu.h:13
pub const MT_MCU_COM_REG1: u32 = 0x0734; // mt76x02_mcu.h:14
pub const MT_MCU_COM_REG2: u32 = 0x0738; // mt76x02_mcu.h:15
pub const MT_MCU_COM_REG3: u32 = 0x073c; // mt76x02_mcu.h:16

/// ⚠ `mt76x02_regs.h:220-223` names the very same four words `MT_COM_REG0..3`.
/// Both spellings are kept because both appear in upstream call sites.
pub const MT_COM_REG0: u32 = 0x0730; // mt76x02_regs.h:220
pub const MT_COM_REG1: u32 = 0x0734; // mt76x02_regs.h:221
pub const MT_COM_REG2: u32 = 0x0738; // mt76x02_regs.h:222
pub const MT_COM_REG3: u32 = 0x073c; // mt76x02_regs.h:223

pub const MT_MCU_PCIE_REMAP_BASE1: u32 = 0x0740; // mt76x2/mcu.h:14
pub const MT_MCU_PCIE_REMAP_BASE2: u32 = 0x0744; // mt76x2/mcu.h:15
pub const MT_MCU_PCIE_REMAP_BASE3: u32 = 0x0748; // mt76x2/mcu.h:16
pub const MT_MCU_PCIE_REMAP_BASE4: u32 = 0x074c; // mt76x02_mcu.h:21

pub const MT_MCU_SEMAPHORE_00: u32 = 0x07b0; // mt76x02_mcu.h:23
pub const MT_MCU_SEMAPHORE_01: u32 = 0x07b4; // mt76x02_mcu.h:24
pub const MT_MCU_SEMAPHORE_02: u32 = 0x07b8; // mt76x02_mcu.h:25
pub const MT_MCU_SEMAPHORE_03: u32 = 0x07bc; // mt76x02_mcu.h:26

// MCU address-space bases. These are *not* MAC registers — they are the `base`
// argument of the MCU register-pair protocol (see `crate::mt76x0::mcu`), which
// the firmware adds to each pair's offset before it does the access.

/// Base the MCU adds for ordinary MAC/BBP register pairs. mt76x02_mcu.h:19
pub const MT_MCU_MEMMAP_WLAN: u32 = 0x0041_0000;
/// Base the MCU adds to reach the RF banks. `mt76x0/mcu.h:20`
pub const MT_MCU_MEMMAP_RF: u32 = 0x8000_0000;
/// Upstream notes the BBP shares the MAC register space, so the would-be
/// `MT_MCU_MEMMAP_BBP = 0x40000000` is **commented out** at `mt76x0/mcu.h:18`.
/// Recorded here as documentation only — do not use it as a base.
pub const MT_MCU_MEMMAP_BBP_UNUSED: u32 = 0x4000_0000;

/// Instruction-memory load address. mt76x02_mcu.h:28
pub const MT_MCU_ILM_ADDR: u32 = 0x0008_0000;
/// Data-memory offset added to the ILM address on mt76x0. `mt76x0/mcu.h:15`
pub const MT_MCU_DLM_OFFSET: u32 = 0x0008_0000;
/// Size of the interrupt-vector block that is written last (and separately) to
/// start the firmware. `mt76x0/mcu.h:14`
pub const MT_MCU_IVB_SIZE: u32 = 0x40;
/// `MT_MCU_IVB_ADDR` = `MT_MCU_ILM_ADDR + 0x54000 - MT_MCU_IVB_SIZE`
/// (`mt76x0/pci_mcu.c:11`) = 0xd3fc0. The magic 0x54000 offset is unexplained
/// upstream; ported faithfully.
///
/// ⚠ **This is the PCI path's constant and the USB path does not use it.**
/// `mt76x0/usb_mcu.c:25-49` splits the first [`MT_MCU_IVB_SIZE`] bytes off the
/// firmware payload and ships them with vendor request `0x12` rather than
/// writing them to an address. A USB port that programs this address is
/// following the wrong driver.
pub const MT_MCU_IVB_ADDR: u32 = MT_MCU_ILM_ADDR + 0x54000 - MT_MCU_IVB_SIZE;

/// Largest in-band MCU command payload. mt76x02_mcu.h:18
pub const MT_INBAND_PACKET_MAX_LEN: usize = 192;

// ── LED ──────────────────────────────────────────────────────────────────────

pub const MT_LED_CTRL: u32 = 0x0770; // mt76x02_regs.h:225
/// `MT_LED_CTRL_REPLAY(_n)` — mt76x02_regs.h:226.
pub const fn mt_led_ctrl_replay(n: u32) -> u32 {
    1 << (8 * n)
}
/// `MT_LED_CTRL_POLARITY(_n)` — mt76x02_regs.h:227.
pub const fn mt_led_ctrl_polarity(n: u32) -> u32 {
    1 << (1 + 8 * n)
}
/// `MT_LED_CTRL_TX_BLINK_MODE(_n)` — mt76x02_regs.h:228.
pub const fn mt_led_ctrl_tx_blink_mode(n: u32) -> u32 {
    1 << (2 + 8 * n)
}
/// `MT_LED_CTRL_KICK(_n)` — mt76x02_regs.h:229.
pub const fn mt_led_ctrl_kick(n: u32) -> u32 {
    1 << (7 + 8 * n)
}

pub const MT_LED_TX_BLINK_0: u32 = 0x0774; // mt76x02_regs.h:231
pub const MT_LED_TX_BLINK_1: u32 = 0x0778; // mt76x02_regs.h:232

pub const MT_LED_S0_BASE: u32 = 0x077c; // mt76x02_regs.h:234
/// `MT_LED_S0(_n)` — mt76x02_regs.h:235. Note the stride is **8**, not 4.
pub const fn mt_led_s0(n: u32) -> u32 {
    MT_LED_S0_BASE + 8 * n
}
pub const MT_LED_S1_BASE: u32 = 0x0780; // mt76x02_regs.h:236
/// `MT_LED_S1(_n)` — mt76x02_regs.h:237.
pub const fn mt_led_s1(n: u32) -> u32 {
    MT_LED_S1_BASE + 8 * n
}
pub const MT_LED_STATUS_OFF: u32 = 0xff00_0000; // GENMASK(31,24) — :238
pub const MT_LED_STATUS_ON: u32 = 0x00ff_0000; // GENMASK(23,16) — :239
pub const MT_LED_STATUS_DURATION: u32 = 0x0000_ff00; // GENMASK(15,8) — :240

/// The other LED control word, in the MAC block. mt76x02_regs.h:292
pub const MT_LED_CFG: u32 = 0x102c;

// ── MAC core ─────────────────────────────────────────────────────────────────

/// MAC CSR 0 — read during bring-up as a liveness probe on the MAC block.
pub const MT_MAC_CSR0: u32 = 0x1000; // mt76x02_regs.h:267

/// The TX/RX master enable. ★ MEASURED `0x0c` under a kernel monitor =
/// `ENABLE_TX | ENABLE_RX`, which is the value a working capture needs.
pub const MT_MAC_SYS_CTRL: u32 = 0x1004; // mt76x02_regs.h:269
pub const MT_MAC_SYS_CTRL_RESET_CSR: u32 = 0x0000_0001; // BIT(0) — :270
pub const MT_MAC_SYS_CTRL_RESET_BBP: u32 = 0x0000_0002; // BIT(1) — :271
pub const MT_MAC_SYS_CTRL_ENABLE_TX: u32 = 0x0000_0004; // BIT(2) — :272
pub const MT_MAC_SYS_CTRL_ENABLE_RX: u32 = 0x0000_0008; // BIT(3) — :273

/// Our own MAC address, low 4 bytes. Written from the EEPROM at bring-up.
pub const MT_MAC_ADDR_DW0: u32 = 0x1008; // mt76x02_regs.h:275
/// High 2 bytes (bits 15:0) plus the unicast-to-me byte mask.
pub const MT_MAC_ADDR_DW1: u32 = 0x100c; // mt76x02_regs.h:276
pub const MT_MAC_ADDR_DW1_U2ME_MASK: u32 = 0x00ff_0000; // GENMASK(23,16) — :277

pub const MT_MAC_BSSID_DW0: u32 = 0x1010; // mt76x02_regs.h:279
pub const MT_MAC_BSSID_DW1: u32 = 0x1014; // mt76x02_regs.h:280
pub const MT_MAC_BSSID_DW1_ADDR: u32 = 0x0000_ffff; // GENMASK(15,0) — :281
pub const MT_MAC_BSSID_DW1_MBSS_MODE: u32 = 0x0003_0000; // GENMASK(17,16) — :282
pub const MT_MAC_BSSID_DW1_MBEACON_N: u32 = 0x001c_0000; // GENMASK(20,18) — :283
pub const MT_MAC_BSSID_DW1_MBSS_LOCAL_BIT: u32 = 0x0020_0000; // BIT(21) — :284
pub const MT_MAC_BSSID_DW1_MBSS_MODE_B2: u32 = 0x0040_0000; // BIT(22) — :285
pub const MT_MAC_BSSID_DW1_MBEACON_N_B3: u32 = 0x0080_0000; // BIT(23) — :286
pub const MT_MAC_BSSID_DW1_MBSS_IDX_BYTE: u32 = 0x0700_0000; // GENMASK(26,24) — :287

/// Maximum RX/TX frame length, and the A-MPDU length exponent. The named-radio
/// path cares because oversize MPDUs are one of the throughput levers.
pub const MT_MAX_LEN_CFG: u32 = 0x1018; // mt76x02_regs.h:289
pub const MT_MAX_LEN_CFG_AMPDU: u32 = 0x0000_3000; // GENMASK(13,12) — :290

pub const MT_AMPDU_MAX_LEN_20M1S: u32 = 0x1030; // mt76x02_regs.h:294
pub const MT_AMPDU_MAX_LEN_20M2S: u32 = 0x1034; // mt76x02_regs.h:295
pub const MT_AMPDU_MAX_LEN_40M1S: u32 = 0x1038; // mt76x02_regs.h:296
pub const MT_AMPDU_MAX_LEN_40M2S: u32 = 0x103c; // mt76x02_regs.h:297
pub const MT_AMPDU_MAX_LEN: u32 = 0x1040; // mt76x02_regs.h:298

pub const MT_WCID_DROP_BASE: u32 = 0x106c; // mt76x02_regs.h:300
/// `MT_WCID_DROP(_n)` — mt76x02_regs.h:301. 32 station indices per word.
pub const fn mt_wcid_drop(n: u32) -> u32 {
    MT_WCID_DROP_BASE + ((n >> 5) * 4)
}
/// `MT_WCID_DROP_MASK(_n)` — mt76x02_regs.h:302.
pub const fn mt_wcid_drop_mask(n: u32) -> u32 {
    1 << (n % 32)
}

pub const MT_BCN_BYPASS_MASK: u32 = 0x108c; // mt76x02_regs.h:304

pub const MT_MAC_APC_BSSID_BASE: u32 = 0x1090; // mt76x02_regs.h:306
/// `MT_MAC_APC_BSSID_L(_n)` — mt76x02_regs.h:307.
pub const fn mt_mac_apc_bssid_l(n: u32) -> u32 {
    MT_MAC_APC_BSSID_BASE + (n * 8)
}
/// `MT_MAC_APC_BSSID_H(_n)` — mt76x02_regs.h:308.
pub const fn mt_mac_apc_bssid_h(n: u32) -> u32 {
    MT_MAC_APC_BSSID_BASE + (n * 8) + 4
}
pub const MT_MAC_APC_BSSID_H_ADDR: u32 = 0x0000_ffff; // GENMASK(15,0) — :309
pub const MT_MAC_APC_BSSID0_H_EN: u32 = 0x0001_0000; // BIT(16) — :310

// ── Timing: IFS, slot, channel-time counters, TSF ────────────────────────────
// This block is the whole reason the mt76x0 is interesting for named-radio
// airtime work: idle/busy are honest microsecond counters, and the TSF is a
// 1 MHz free-running clock once armed.

/// SIFS/XIFS/EIFS timing. `OFDM_SIFS` is the field a slot-timing knob moves.
pub const MT_XIFS_TIME_CFG: u32 = 0x1100; // mt76x02_regs.h:312
pub const MT_XIFS_TIME_CFG_CCK_SIFS: u32 = 0x0000_00ff; // GENMASK(7,0) — :313
pub const MT_XIFS_TIME_CFG_OFDM_SIFS: u32 = 0x0000_ff00; // GENMASK(15,8) — :314
pub const MT_XIFS_TIME_CFG_OFDM_XIFS: u32 = 0x000f_0000; // GENMASK(19,16) — :315
pub const MT_XIFS_TIME_CFG_EIFS: u32 = 0x1ff0_0000; // GENMASK(28,20) — :316
pub const MT_XIFS_TIME_CFG_BB_RXEND_EN: u32 = 0x2000_0000; // BIT(29) — :317

/// Backoff slot time (µs) + CCA delay. ★ MEASURED `0x0000_0209` under a kernel
/// monitor, bit-identical to `mt76x0/initvals_init.h:21`: slot = 9 µs,
/// CC_DELAY = 2.
pub const MT_BKOFF_SLOT_CFG: u32 = 0x1104; // mt76x02_regs.h:319
pub const MT_BKOFF_SLOT_CFG_SLOTTIME: u32 = 0x0000_00ff; // GENMASK(7,0) — :320
pub const MT_BKOFF_SLOT_CFG_CC_DELAY: u32 = 0x0000_0f00; // GENMASK(11,8) — :321

/// Arms the channel-time counters. ★ MEASURED: with `TIMER_EN` set and
/// TX/RX/NAV/EIFS counted as busy, `(`[`MT_CH_IDLE`]`+`[`MT_CH_BUSY`]`)/elapsed`
/// = 1.00 over a 100 ms window — i.e. the counters tile the window exactly and
/// occupancy needs no calibration factor.
pub const MT_CH_TIME_CFG: u32 = 0x110c; // mt76x02_regs.h:323
pub const MT_CH_TIME_CFG_TIMER_EN: u32 = 0x0000_0001; // BIT(0) — :324
pub const MT_CH_TIME_CFG_TX_AS_BUSY: u32 = 0x0000_0002; // BIT(1) — :325
pub const MT_CH_TIME_CFG_RX_AS_BUSY: u32 = 0x0000_0004; // BIT(2) — :326
pub const MT_CH_TIME_CFG_NAV_AS_BUSY: u32 = 0x0000_0008; // BIT(3) — :327
pub const MT_CH_TIME_CFG_EIFS_AS_BUSY: u32 = 0x0000_0010; // BIT(4) — :328
pub const MT_CH_TIME_CFG_MDRDY_CNT_EN: u32 = 0x0000_0020; // BIT(5) — :329
/// ⚠ Upstream spells this one without the `_TIME_CFG_` infix. Same register.
pub const MT_CH_CCA_RC_EN: u32 = 0x0000_0040; // BIT(6) — :330
/// Bits 9:8 — clear-on-read policy for the channel timers.
pub const MT_CH_TIME_CFG_CH_TIMER_CLR: u32 = 0x0000_0300; // GENMASK(9,8) — :331
/// Bits 11:10 — clear-on-read policy for the MDRDY counter.
pub const MT_CH_TIME_CFG_MDRDY_CLR: u32 = 0x0000_0c00; // GENMASK(11,10) — :332

pub const MT_PBF_LIFE_TIMER: u32 = 0x1110; // mt76x02_regs.h:334

/// Beacon/TSF timing. ★ MEASURED: bit 16 ([`MT_BEACON_TIME_CFG_TIMER_EN`]) is
/// what starts the TSF; as found under a kernel monitor it is **clear**, which
/// is why [`MT_TSF_TIMER_DW0`] reads 0 there. ★ And `SYNC_MODE` (bits 18:17)
/// must be **cleared**, or received beacons overwrite the TSF and the
/// common-view clock jumps.
pub const MT_BEACON_TIME_CFG: u32 = 0x1114; // mt76x02_regs.h:336
pub const MT_BEACON_TIME_CFG_INTVAL: u32 = 0x0000_ffff; // GENMASK(15,0) — :337
pub const MT_BEACON_TIME_CFG_TIMER_EN: u32 = 0x0001_0000; // BIT(16) — :338
pub const MT_BEACON_TIME_CFG_SYNC_MODE: u32 = 0x0006_0000; // GENMASK(18,17) — :339
pub const MT_BEACON_TIME_CFG_TBTT_EN: u32 = 0x0008_0000; // BIT(19) — :340
pub const MT_BEACON_TIME_CFG_BEACON_TX: u32 = 0x0010_0000; // BIT(20) — :341
pub const MT_BEACON_TIME_CFG_TSF_COMP: u32 = 0xff00_0000; // GENMASK(31,24) — :342

pub const MT_TBTT_SYNC_CFG: u32 = 0x1118; // mt76x02_regs.h:344

/// ★ The **LOW** 32 bits of the 64-bit TSF, ticking at 1.000 MHz (MEASURED).
/// Assemble as `((dw1 as u64) << 32) | dw0 as u64`. Do **not** copy
/// `mt76x02_usb_core.c:155-157`, which has the two words the other way round.
pub const MT_TSF_TIMER_DW0: u32 = 0x111c; // mt76x02_regs.h:345
/// The **HIGH** 32 bits of the TSF — see [`MT_TSF_TIMER_DW0`].
pub const MT_TSF_TIMER_DW1: u32 = 0x1120; // mt76x02_regs.h:346

/// Time remaining to the next TBTT, in units of 32 µs
/// (`mt76x02_usb_core.c:152-153` multiplies the field by 32).
pub const MT_TBTT_TIMER: u32 = 0x1124; // mt76x02_regs.h:347
pub const MT_TBTT_TIMER_VAL: u32 = 0x0001_ffff; // GENMASK(16,0) — :348

pub const MT_INT_TIMER_CFG: u32 = 0x1128; // mt76x02_regs.h:350
pub const MT_INT_TIMER_CFG_PRE_TBTT: u32 = 0x0000_ffff; // GENMASK(15,0) — :351
pub const MT_INT_TIMER_CFG_GP_TIMER: u32 = 0xffff_0000; // GENMASK(31,16) — :352

pub const MT_INT_TIMER_EN: u32 = 0x112c; // mt76x02_regs.h:354
pub const MT_INT_TIMER_EN_PRE_TBTT_EN: u32 = 0x0000_0001; // BIT(0) — :355
pub const MT_INT_TIMER_EN_GP_TIMER_EN: u32 = 0x0000_0002; // BIT(1) — :356

/// ★ **Read-and-clear** microseconds-idle since the previous read (MEASURED).
/// Pair with [`MT_CH_BUSY`] and divide by the wall-clock window to get channel
/// occupancy — this is the mt76 equivalent of the 8812au's `REG_RXERR_RPT`
/// frame-free sensing path, only honest in absolute µs.
pub const MT_CH_IDLE: u32 = 0x1130; // mt76x02_regs.h:358
/// ★ **Read-and-clear** microseconds-busy since the previous read (MEASURED).
pub const MT_CH_BUSY: u32 = 0x1134; // mt76x02_regs.h:359
/// Busy µs on the *secondary* (extension) channel, for 40 MHz operation.
pub const MT_EXT_CH_BUSY: u32 = 0x1138; // mt76x02_regs.h:360
/// ★ An **independent** energy-detect busy-µs counter (MEASURED). Upstream's
/// ED-CCA loop reads it once per window and computes `busy*100/active`
/// (`mt76x02_mac.c:1152-1158`), so it too is read-and-clear.
pub const MT_ED_CCA_TIMER: u32 = 0x1140; // mt76x02_regs.h:361

/// Live MAC state: is a TX or an RX in progress right now.
pub const MT_MAC_STATUS: u32 = 0x1200; // mt76x02_regs.h:363
pub const MT_MAC_STATUS_TX: u32 = 0x0000_0001; // BIT(0) — :364
pub const MT_MAC_STATUS_RX: u32 = 0x0000_0002; // BIT(1) — :365

pub const MT_PWR_PIN_CFG: u32 = 0x1204; // mt76x02_regs.h:367
pub const MT_AUX_CLK_CFG: u32 = 0x120c; // mt76x02_regs.h:368

// PA-mode configuration, part of the mt76x0 TX-power path.
pub const MT_BB_PA_MODE_CFG0: u32 = 0x1214; // mt76x02_regs.h:370
pub const MT_BB_PA_MODE_CFG1: u32 = 0x1218; // mt76x02_regs.h:371
pub const MT_RF_PA_MODE_CFG0: u32 = 0x121c; // mt76x02_regs.h:372
pub const MT_RF_PA_MODE_CFG1: u32 = 0x1220; // mt76x02_regs.h:373
pub const MT_RF_PA_MODE_ADJ0: u32 = 0x1228; // mt76x02_regs.h:375
pub const MT_RF_PA_MODE_ADJ1: u32 = 0x122c; // mt76x02_regs.h:376
pub const MT_DACCLK_EN_DLY_CFG: u32 = 0x1264; // mt76x02_regs.h:378

// ── EDCA (per-AC contention parameters) ──────────────────────────────────────
// The per-AC mirror of the WMM block above. This is the actuator a named
// airtime-lease MAC reaches for: AIFSN/CWMIN/CWMAX/TXOP per access category.

pub const MT_EDCA_CFG_BASE: u32 = 0x1300; // mt76x02_regs.h:380
/// `MT_EDCA_CFG_AC(_n)` — mt76x02_regs.h:381. `_n` is the AC index 0..=3.
pub const fn mt_edca_cfg_ac(n: u32) -> u32 {
    MT_EDCA_CFG_BASE + (n << 2)
}
/// TXOP limit in units of 32 µs.
pub const MT_EDCA_CFG_TXOP: u32 = 0x0000_00ff; // GENMASK(7,0) — :382
pub const MT_EDCA_CFG_AIFSN: u32 = 0x0000_0f00; // GENMASK(11,8) — :383
/// Contention-window minimum, as the **exponent** (CW = 2^n − 1).
pub const MT_EDCA_CFG_CWMIN: u32 = 0x0000_f000; // GENMASK(15,12) — :384
/// Contention-window maximum, as the exponent.
pub const MT_EDCA_CFG_CWMAX: u32 = 0x000f_0000; // GENMASK(19,16) — :385

// ── TX power ─────────────────────────────────────────────────────────────────
// Per-rate power, packed several rates to a word. ⚠ The block is NOT
// contiguous: 0..4 sit at 0x1314..0x1324, then 7/8/9 jump to 0x13d4/8/c, and
// there is no _5 or _6 anywhere upstream. The two "_EXT" words at 0x1390/0x1394
// extend _0 and _1 for the second chain.

pub const MT_TX_PWR_CFG_0: u32 = 0x1314; // mt76x02_regs.h:387
pub const MT_TX_PWR_CFG_1: u32 = 0x1318; // mt76x02_regs.h:388
pub const MT_TX_PWR_CFG_2: u32 = 0x131c; // mt76x02_regs.h:389
pub const MT_TX_PWR_CFG_3: u32 = 0x1320; // mt76x02_regs.h:390
pub const MT_TX_PWR_CFG_4: u32 = 0x1324; // mt76x02_regs.h:391
// (no MT_TX_PWR_CFG_5 / _6 upstream — the numbering skips)
pub const MT_TX_PWR_CFG_7: u32 = 0x13d4; // mt76x02_regs.h:406
pub const MT_TX_PWR_CFG_8: u32 = 0x13d8; // mt76x02_regs.h:407
pub const MT_TX_PWR_CFG_9: u32 = 0x13dc; // mt76x02_regs.h:408
pub const MT_TX_PWR_CFG_0_EXT: u32 = 0x1390; // mt76x02_regs.h:472
pub const MT_TX_PWR_CFG_1_EXT: u32 = 0x1394; // mt76x02_regs.h:473

/// Antenna/TRSW pin control. `TXANT`/`RXANT` are 1×1 on the MT7610U, so only
/// bit 0 of each nibble is meaningful here.
pub const MT_TX_PIN_CFG: u32 = 0x1328; // mt76x02_regs.h:392
pub const MT_TX_PIN_CFG_TXANT: u32 = 0x0000_000f; // GENMASK(3,0) — :393
pub const MT_TX_PIN_CFG_RXANT: u32 = 0x0000_0f00; // GENMASK(11,8) — :394
pub const MT_TX_PIN_RFTR_EN: u32 = 0x0001_0000; // BIT(16) — :395
pub const MT_TX_PIN_TRSW_EN: u32 = 0x0004_0000; // BIT(18) — :396

/// Which band the MAC believes it is on, and whether the primary channel is the
/// upper half of a 40 MHz pair. Written by every channel switch.
pub const MT_TX_BAND_CFG: u32 = 0x132c; // mt76x02_regs.h:398
pub const MT_TX_BAND_CFG_UPPER_40M: u32 = 0x0000_0001; // BIT(0) — :399
pub const MT_TX_BAND_CFG_5G: u32 = 0x0000_0002; // BIT(1) — :400
pub const MT_TX_BAND_CFG_2G: u32 = 0x0000_0004; // BIT(2) — :401

pub const MT_TX_SW_CFG0: u32 = 0x1330; // mt76x02_regs.h:410
pub const MT_TX_SW_CFG1: u32 = 0x1334; // mt76x02_regs.h:411
pub const MT_TX_SW_CFG2: u32 = 0x1338; // mt76x02_regs.h:412
pub const MT_TX_SW_CFG3: u32 = 0x1478; // mt76x02_regs.h:550

/// TXOP truncation + the **ED-CCA enable**. ★ MEASURED `0x0000_583f` under a
/// kernel monitor, bit-identical to `mt76x0/initvals_init.h:40`:
/// `TRUN_EN = 0x3f`, `EXT_CCA_DLY = 0x58`, and `ED_CCA_EN` **clear**. Setting
/// bit 20 is what arms energy-detect CCA (`mt76x02_mac.c:1113`) — the same knob
/// that, on the 8812au, was measured to trade collision loss for TX starvation
/// on a saturated channel. It is an actuator, not a rescue.
pub const MT_TXOP_CTRL_CFG: u32 = 0x1340; // mt76x02_regs.h:414
pub const MT_TXOP_TRUN_EN: u32 = 0x0000_003f; // GENMASK(5,0) — :415
pub const MT_TXOP_EXT_CCA_DLY: u32 = 0x0000_ff00; // GENMASK(15,8) — :416
pub const MT_TXOP_ED_CCA_EN: u32 = 0x0010_0000; // BIT(20) — :417

pub const MT_TX_RTS_CFG: u32 = 0x1344; // mt76x02_regs.h:419
pub const MT_TX_RTS_CFG_RETRY_LIMIT: u32 = 0x0000_00ff; // GENMASK(7,0) — :420
/// RTS threshold in bytes. Bits 23:8, so the largest expressible threshold is
/// 65535 — which is how "RTS off" is written.
pub const MT_TX_RTS_CFG_THRESH: u32 = 0x00ff_ff00; // GENMASK(23,8) — :421
pub const MT_TX_RTS_FALLBACK: u32 = 0x0100_0000; // BIT(24) — :422

pub const MT_TX_TIMEOUT_CFG: u32 = 0x1348; // mt76x02_regs.h:424
pub const MT_TX_TIMEOUT_CFG_ACKTO: u32 = 0x0000_ff00; // GENMASK(15,8) — :425

pub const MT_TX_RETRY_CFG: u32 = 0x134c; // mt76x02_regs.h:427
/// Link config; carries the CF-ACK enable that the ED-CCA path toggles
/// (`mt76x02_mac.c:1112`, `:1119`).
pub const MT_TX_LINK_CFG: u32 = 0x1350; // mt76x02_regs.h:428
pub const MT_TX_CFACK_EN: u32 = 0x0000_1000; // BIT(12) of MT_TX_LINK_CFG — :429

pub const MT_VHT_HT_FBK_CFG0: u32 = 0x1354; // mt76x02_regs.h:430
pub const MT_VHT_HT_FBK_CFG1: u32 = 0x1358; // mt76x02_regs.h:431
pub const MT_LG_FBK_CFG0: u32 = 0x135c; // mt76x02_regs.h:432
pub const MT_LG_FBK_CFG1: u32 = 0x1360; // mt76x02_regs.h:433
pub const MT_HT_FBK_TO_LEGACY: u32 = 0x1384; // mt76x02_regs.h:403
pub const MT_TX_MPDU_ADJ_INT: u32 = 0x1388; // mt76x02_regs.h:404
pub const MT_EXP_ACK_TIME: u32 = 0x1380; // mt76x02_regs.h:470
pub const MT_PIFS_TX_CFG: u32 = 0x13ec; // mt76x02_regs.h:510

pub const MT_TX_FBK_LIMIT: u32 = 0x1398; // mt76x02_regs.h:475
pub const MT_TX_FBK_LIMIT_MPDU_FBK: u32 = 0x0000_00ff; // GENMASK(7,0) — :476
pub const MT_TX_FBK_LIMIT_AMPDU_FBK: u32 = 0x0000_ff00; // GENMASK(15,8) — :477
pub const MT_TX_FBK_LIMIT_MPDU_UP_CLEAR: u32 = 0x0001_0000; // BIT(16) — :478
pub const MT_TX_FBK_LIMIT_AMPDU_UP_CLEAR: u32 = 0x0002_0000; // BIT(17) — :479
pub const MT_TX_FBK_LIMIT_RATE_LUT: u32 = 0x0004_0000; // BIT(18) — :480

// ── Protection (RTS/CTS, NAV, TXOP-allow) ────────────────────────────────────
// One register per protection mode, all sharing the field layout below. #96
// measured that stock Wi-Fi does NOT honour the NAV we put in an injected
// frame's Duration field — these registers govern what *this* MAC emits and
// respects, and are the reason a lease must be self-enforced rather than
// delegated to the Duration field.

pub const MT_PROT_CFG_RATE: u32 = 0x0000_ffff; // GENMASK(15,0) — :435
pub const MT_PROT_CFG_CTRL: u32 = 0x0003_0000; // GENMASK(17,16) — :436
pub const MT_PROT_CFG_NAV: u32 = 0x000c_0000; // GENMASK(19,18) — :437
pub const MT_PROT_CFG_TXOP_ALLOW: u32 = 0x03f0_0000; // GENMASK(25,20) — :438
pub const MT_PROT_CFG_RTS_THRESH: u32 = 0x0400_0000; // BIT(26) — :439

pub const MT_CCK_PROT_CFG: u32 = 0x1364; // mt76x02_regs.h:441
pub const MT_OFDM_PROT_CFG: u32 = 0x1368; // mt76x02_regs.h:442
pub const MT_MM20_PROT_CFG: u32 = 0x136c; // mt76x02_regs.h:443
pub const MT_MM40_PROT_CFG: u32 = 0x1370; // mt76x02_regs.h:444
pub const MT_GF20_PROT_CFG: u32 = 0x1374; // mt76x02_regs.h:445
pub const MT_GF40_PROT_CFG: u32 = 0x1378; // mt76x02_regs.h:446

// The same fields again, spelled as individual bits (upstream keeps both sets).
pub const MT_PROT_RATE: u32 = 0x0000_ffff; // GENMASK(15,0) — :448
pub const MT_PROT_CTRL_RTS_CTS: u32 = 0x0001_0000; // BIT(16) — :449
pub const MT_PROT_CTRL_CTS2SELF: u32 = 0x0002_0000; // BIT(17) — :450
pub const MT_PROT_NAV_SHORT: u32 = 0x0004_0000; // BIT(18) — :451
pub const MT_PROT_NAV_LONG: u32 = 0x0008_0000; // BIT(19) — :452
pub const MT_PROT_TXOP_ALLOW_CCK: u32 = 0x0010_0000; // BIT(20) — :453
pub const MT_PROT_TXOP_ALLOW_OFDM: u32 = 0x0020_0000; // BIT(21) — :454
pub const MT_PROT_TXOP_ALLOW_MM20: u32 = 0x0040_0000; // BIT(22) — :455
pub const MT_PROT_TXOP_ALLOW_MM40: u32 = 0x0080_0000; // BIT(23) — :456
pub const MT_PROT_TXOP_ALLOW_GF20: u32 = 0x0100_0000; // BIT(24) — :457
pub const MT_PROT_TXOP_ALLOW_GF40: u32 = 0x0200_0000; // BIT(25) — :458
pub const MT_PROT_RTS_THR_EN: u32 = 0x0400_0000; // BIT(26) — :459

/// Protection-rate codes, as written into [`MT_PROT_CFG_RATE`].
pub const MT_PROT_RATE_CCK_11: u32 = 0x0003; // mt76x02_regs.h:460
pub const MT_PROT_RATE_OFDM_6: u32 = 0x2000; // mt76x02_regs.h:461
pub const MT_PROT_RATE_OFDM_24: u32 = 0x2004; // mt76x02_regs.h:462
pub const MT_PROT_RATE_DUP_OFDM_24: u32 = 0x2084; // mt76x02_regs.h:463
pub const MT_PROT_RATE_SGI_OFDM_24: u32 = 0x2104; // mt76x02_regs.h:464

pub const MT_PROT_TXOP_ALLOW_ALL: u32 = 0x03f0_0000; // GENMASK(25,20) — :465
/// `MT_PROT_TXOP_ALLOW_ALL & ~MM40 & ~GF40` (mt76x02_regs.h:466-468) — bits
/// 20,21,22,24.
pub const MT_PROT_TXOP_ALLOW_BW20: u32 =
    MT_PROT_TXOP_ALLOW_ALL & !MT_PROT_TXOP_ALLOW_MM40 & !MT_PROT_TXOP_ALLOW_GF40; // 0x0170_0000

pub const MT_TX_PROT_CFG6: u32 = 0x13e0; // mt76x02_regs.h:506
pub const MT_TX_PROT_CFG7: u32 = 0x13e4; // mt76x02_regs.h:507
pub const MT_TX_PROT_CFG8: u32 = 0x13e8; // mt76x02_regs.h:508

pub const MT_PROT_AUTO_TX_CFG: u32 = 0x1648; // mt76x02_regs.h:557
pub const MT_PROT_AUTO_TX_CFG_PROT_PADJ: u32 = 0x0000_0f00; // GENMASK(11,8) — :558
pub const MT_PROT_AUTO_TX_CFG_AUTO_PADJ: u32 = 0x0f00_0000; // GENMASK(27,24) — :559

// ── TX gain / ALC (automatic level control) ──────────────────────────────────
// ⚠ Two upstream aliasing hazards live in this block, both flagged below.

pub const MT_TX0_RF_GAIN_CORR: u32 = 0x13a0; // mt76x02_regs.h:482
pub const MT_TX1_RF_GAIN_CORR: u32 = 0x13a4; // mt76x02_regs.h:483
/// ⚠ 0x13a8 is defined **twice on consecutive lines** upstream
/// (mt76x02_regs.h:484 and :485, the second tagged `/* MT76x0 */`) and then a
/// **third** time as [`MT_TX_ALC_CFG_2`] at :496. One register, three names.
pub const MT_TX0_RF_GAIN_ATTEN: u32 = 0x13a8; // mt76x02_regs.h:484,485

pub const MT_TX_ALC_CFG_0: u32 = 0x13b0; // mt76x02_regs.h:487
pub const MT_TX_ALC_CFG_0_CH_INIT_0: u32 = 0x0000_003f; // GENMASK(5,0) — :488
pub const MT_TX_ALC_CFG_0_CH_INIT_1: u32 = 0x0000_3f00; // GENMASK(13,8) — :489
pub const MT_TX_ALC_CFG_0_LIMIT_0: u32 = 0x003f_0000; // GENMASK(21,16) — :490
pub const MT_TX_ALC_CFG_0_LIMIT_1: u32 = 0x3f00_0000; // GENMASK(29,24) — :491

pub const MT_TX_ALC_CFG_1: u32 = 0x13b4; // mt76x02_regs.h:493
pub const MT_TX_ALC_CFG_1_TEMP_COMP: u32 = 0x0000_003f; // GENMASK(5,0) — :494

/// ⚠ Same address as [`MT_TX0_RF_GAIN_ATTEN`] (0x13a8). mt76x02_regs.h:496
pub const MT_TX_ALC_CFG_2: u32 = 0x13a8;
pub const MT_TX_ALC_CFG_2_TEMP_COMP: u32 = 0x0000_003f; // GENMASK(5,0) — :497

pub const MT_TX_ALC_CFG_3: u32 = 0x13ac; // mt76x02_regs.h:499
/// ⚠ Same address as [`MT_TX0_BB_GAIN_ATTEN`] (0x13c0). mt76x02_regs.h:500
pub const MT_TX_ALC_CFG_4: u32 = 0x13c0;
pub const MT_TX_ALC_CFG_4_LOWGAIN_CH_EN: u32 = 0x8000_0000; // BIT(31) — :501
/// The MT76x0 spelling of 0x13c0 — see [`MT_TX_ALC_CFG_4`]. mt76x02_regs.h:502
pub const MT_TX0_BB_GAIN_ATTEN: u32 = 0x13c0;

pub const MT_TX_ALC_VGA3: u32 = 0x13c8; // mt76x02_regs.h:504

// ── RX filtering / auto-response / CCA ───────────────────────────────────────

/// ★ RX filter. **Every bit is a DROP bit**: set = discard that class
/// (`mt76x02_util.c:213-232` sets a bit when the corresponding `FIF_*` pass flag
/// is *absent*). Clearing bits is how a monitor sees more, not less.
///
/// MEASURED `0x0000_1093` under a kernel monitor, and it decomposes exactly:
/// the init value is `0x0001_7f97` (`mt76x0/initvals_init.h:20`), and mac80211's
/// monitor path clears PROMISC (bit 2, `mt76x0/main.c:84` — clear it *to* be
/// promiscuous) plus the CONTROL group `ACK|CTS|CFEND|CFACK|BA|CTRL_RSV` (bits
/// 8,9,10,11,14,16) and PSPOLL (bit 13). What survives is bits 0,1,4,7,12 =
/// `CRC_ERR|PHY_ERR|VER_ERR|DUP|RTS` — note RTS and BAR are *not* in upstream's
/// CONTROL group, so RTS stays dropped. A named-radio capture that wants
/// corrupt frames must additionally clear CRC_ERR and PHY_ERR.
pub const MT_RX_FILTR_CFG: u32 = 0x1400; // mt76x02_regs.h:512
pub const MT_RX_FILTR_CFG_CRC_ERR: u32 = 0x0000_0001; // BIT(0) — :514
pub const MT_RX_FILTR_CFG_PHY_ERR: u32 = 0x0000_0002; // BIT(1) — :515
/// Set = drop frames not addressed to us. **Clear it to be promiscuous.**
pub const MT_RX_FILTR_CFG_PROMISC: u32 = 0x0000_0004; // BIT(2) — :516
pub const MT_RX_FILTR_CFG_OTHER_BSS: u32 = 0x0000_0008; // BIT(3) — :517
pub const MT_RX_FILTR_CFG_VER_ERR: u32 = 0x0000_0010; // BIT(4) — :518
pub const MT_RX_FILTR_CFG_MCAST: u32 = 0x0000_0020; // BIT(5) — :519
pub const MT_RX_FILTR_CFG_BCAST: u32 = 0x0000_0040; // BIT(6) — :520
pub const MT_RX_FILTR_CFG_DUP: u32 = 0x0000_0080; // BIT(7) — :521
pub const MT_RX_FILTR_CFG_CFACK: u32 = 0x0000_0100; // BIT(8) — :522
pub const MT_RX_FILTR_CFG_CFEND: u32 = 0x0000_0200; // BIT(9) — :523
pub const MT_RX_FILTR_CFG_ACK: u32 = 0x0000_0400; // BIT(10) — :524
pub const MT_RX_FILTR_CFG_CTS: u32 = 0x0000_0800; // BIT(11) — :525
pub const MT_RX_FILTR_CFG_RTS: u32 = 0x0000_1000; // BIT(12) — :526
pub const MT_RX_FILTR_CFG_PSPOLL: u32 = 0x0000_2000; // BIT(13) — :527
pub const MT_RX_FILTR_CFG_BA: u32 = 0x0000_4000; // BIT(14) — :528
pub const MT_RX_FILTR_CFG_BAR: u32 = 0x0000_8000; // BIT(15) — :529
pub const MT_RX_FILTR_CFG_CTRL_RSV: u32 = 0x0001_0000; // BIT(16) — :530

/// Hardware auto-ACK/auto-CTS. A named-radio face that does not want the MAC
/// answering on its behalf clears [`MT_AUTO_RSP_EN`].
pub const MT_AUTO_RSP_CFG: u32 = 0x1404; // mt76x02_regs.h:532
pub const MT_AUTO_RSP_EN: u32 = 0x0000_0001; // BIT(0) — :533
pub const MT_AUTO_RSP_PREAMB_SHORT: u32 = 0x0000_0010; // BIT(4) — :534

pub const MT_LEGACY_BASIC_RATE: u32 = 0x1408; // mt76x02_regs.h:535
pub const MT_HT_BASIC_RATE: u32 = 0x140c; // mt76x02_regs.h:536
pub const MT_HT_CTRL_CFG: u32 = 0x1410; // mt76x02_regs.h:538

pub const MT_RX_PARSER_CFG: u32 = 0x1418; // mt76x02_regs.h:539
/// Set the NAV from *every* received frame, not only ones addressed to us.
pub const MT_RX_PARSER_RX_SET_NAV_ALL: u32 = 0x0000_0001; // BIT(0) — :540

/// Extension-channel CCA source selection. ★ MEASURED `0x0000_f1e4` under a
/// kernel monitor, and it decomposes exactly to `mt76x0/phy.c:917-921` channel
/// group 0 (`CCA0=0, CCA1=1, CCA2=2, CCA3=3, CCA_MASK=BIT(0)`) with
/// `ED_CCA_MASK = 0xf`. The four CCA fields choose which physical detector
/// feeds which logical channel slot; the group index is derived from where the
/// primary sits inside the operating bandwidth.
pub const MT_EXT_CCA_CFG: u32 = 0x141c; // mt76x02_regs.h:542
pub const MT_EXT_CCA_CFG_CCA0: u32 = 0x0000_0003; // GENMASK(1,0) — :543
pub const MT_EXT_CCA_CFG_CCA1: u32 = 0x0000_000c; // GENMASK(3,2) — :544
pub const MT_EXT_CCA_CFG_CCA2: u32 = 0x0000_0030; // GENMASK(5,4) — :545
pub const MT_EXT_CCA_CFG_CCA3: u32 = 0x0000_00c0; // GENMASK(7,6) — :546
pub const MT_EXT_CCA_CFG_CCA_MASK: u32 = 0x0000_0f00; // GENMASK(11,8) — :547
pub const MT_EXT_CCA_CFG_ED_CCA_MASK: u32 = 0x0000_f000; // GENMASK(15,12) — :548

pub const MT_PN_PAD_MODE: u32 = 0x150c; // mt76x02_regs.h:552

/// TXOP holder / 40 MHz blocking. The ED-CCA path sets
/// [`MT_TXOP_HLDR_TX40M_BLK_EN`] on mt76x2 but **clears** it on mt76x0
/// (`mt76x02_mac.c:1122-1128`) — a real per-part divergence, not a typo.
pub const MT_TXOP_HLDR_ET: u32 = 0x1608; // mt76x02_regs.h:554
pub const MT_TXOP_HLDR_TX40M_BLK_EN: u32 = 0x0000_0002; // BIT(1) — :555

// ── Statistics counters ──────────────────────────────────────────────────────
// ★ MEASURED: MT_RX_STAT_0 and _1 are read-and-clear per window, so a sensing
// loop reads them once per interval and the value IS the interval's count.

pub const MT_RX_STAT_0: u32 = 0x1700; // mt76x02_regs.h:561
pub const MT_RX_STAT_0_CRC_ERRORS: u32 = 0x0000_ffff; // GENMASK(15,0) — :562
pub const MT_RX_STAT_0_PHY_ERRORS: u32 = 0xffff_0000; // GENMASK(31,16) — :563

pub const MT_RX_STAT_1: u32 = 0x1704; // mt76x02_regs.h:565
/// False-CCA count for the window — the frame-free occupancy signal, the same
/// quantity the 8812au port reads out of `REG_RXERR_RPT`. Upstream's AGC/VGA
/// loop uses it as `dev->cal.false_cca` (`mt76x0/phy.c`, `mt76x02_mac.c:1170`).
pub const MT_RX_STAT_1_CCA_ERRORS: u32 = 0x0000_ffff; // GENMASK(15,0) — :566
pub const MT_RX_STAT_1_PLCP_ERRORS: u32 = 0xffff_0000; // GENMASK(31,16) — :567

pub const MT_RX_STAT_2: u32 = 0x1708; // mt76x02_regs.h:569
pub const MT_RX_STAT_2_DUP_ERRORS: u32 = 0x0000_ffff; // GENMASK(15,0) — :570
pub const MT_RX_STAT_2_OVERFLOW_ERRORS: u32 = 0xffff_0000; // GENMASK(31,16) — :571

pub const MT_TX_STA_0: u32 = 0x170c; // mt76x02_regs.h:573
pub const MT_TX_STA_0_BEACONS: u32 = 0xffff_0000; // GENMASK(31,16) — :574
pub const MT_TX_STA_1: u32 = 0x1710; // mt76x02_regs.h:576
pub const MT_TX_STA_2: u32 = 0x1714; // mt76x02_regs.h:577

/// Per-MPDU TX status FIFO. Pop while [`MT_TX_STAT_FIFO_VALID`] is set; each pop
/// yields success/aggregation/WCID/rate for one transmitted frame. This is the
/// mt76 equivalent of the Realtek TxReport path — a real per-frame delivery
/// signal rather than an inference.
pub const MT_TX_STAT_FIFO: u32 = 0x1718; // mt76x02_regs.h:579
pub const MT_TX_STAT_FIFO_VALID: u32 = 0x0000_0001; // BIT(0) — :580
pub const MT_TX_STAT_FIFO_SUCCESS: u32 = 0x0000_0020; // BIT(5) — :581
pub const MT_TX_STAT_FIFO_AGGR: u32 = 0x0000_0040; // BIT(6) — :582
pub const MT_TX_STAT_FIFO_ACKREQ: u32 = 0x0000_0080; // BIT(7) — :583
pub const MT_TX_STAT_FIFO_WCID: u32 = 0x0000_ff00; // GENMASK(15,8) — :584
pub const MT_TX_STAT_FIFO_RATE: u32 = 0xffff_0000; // GENMASK(31,16) — :585

pub const MT_TX_AGG_STAT: u32 = 0x171c; // mt76x02_regs.h:587
pub const MT_TX_AGG_CNT_BASE0: u32 = 0x1720; // mt76x02_regs.h:589
pub const MT_MPDU_DENSITY_CNT: u32 = 0x1740; // mt76x02_regs.h:590
pub const MT_TX_AGG_CNT_BASE1: u32 = 0x174c; // mt76x02_regs.h:591
/// `MT_TX_AGG_CNT(_id)` — mt76x02_regs.h:593-595. Note the split: ids 0..7 sit
/// at BASE0, ids ≥ 8 at BASE1 (0x1740 in between is the density counter).
pub const fn mt_tx_agg_cnt(id: u32) -> u32 {
    if id < 8 {
        MT_TX_AGG_CNT_BASE0 + (id << 2)
    } else {
        MT_TX_AGG_CNT_BASE1 + ((id - 8) << 2)
    }
}

pub const MT_TX_STAT_FIFO_EXT: u32 = 0x1798; // mt76x02_regs.h:597
pub const MT_TX_STAT_FIFO_EXT_RETRY: u32 = 0x0000_00ff; // GENMASK(7,0) — :598
pub const MT_TX_STAT_FIFO_EXT_PKTID: u32 = 0x0000_ff00; // GENMASK(15,8) — :599

// ── WCID (per-station) tables, keys, beacon SRAM, thermal ────────────────────
// A named-data radio does not keep a peer table (source is an ephemeral nonce),
// so these are transcribed for completeness and for the broadcast/WCID-0 slot
// the injection path needs — not because a station table is planned.

pub const MT_WCID_TX_RATE_BASE: u32 = 0x1c00; // mt76x02_regs.h:601
/// `MT_WCID_TX_RATE(_i)` — mt76x02_regs.h:602. Stride 8.
pub const fn mt_wcid_tx_rate(i: u32) -> u32 {
    MT_WCID_TX_RATE_BASE + (i << 3)
}

pub const MT_WCID_ADDR_BASE: u32 = 0x1800; // mt76x02_regs.h:643
/// `MT_WCID_ADDR(_n)` — mt76x02_regs.h:644. Stride 8 (6-byte MAC + BA mask).
pub const fn mt_wcid_addr(n: u32) -> u32 {
    MT_WCID_ADDR_BASE + n * 8
}

pub const MT_SRAM_BASE: u32 = 0x4000; // mt76x02_regs.h:646

pub const MT_WCID_KEY_BASE: u32 = 0x8000; // mt76x02_regs.h:648
/// `MT_WCID_KEY(_n)` — mt76x02_regs.h:649. Stride 32.
pub const fn mt_wcid_key(n: u32) -> u32 {
    MT_WCID_KEY_BASE + n * 32
}

pub const MT_WCID_IV_BASE: u32 = 0xa000; // mt76x02_regs.h:651
/// `MT_WCID_IV(_n)` — mt76x02_regs.h:652. Stride 8.
pub const fn mt_wcid_iv(n: u32) -> u32 {
    MT_WCID_IV_BASE + n * 8
}

pub const MT_WCID_ATTR_BASE: u32 = 0xa800; // mt76x02_regs.h:654
/// `MT_WCID_ATTR(_n)` — mt76x02_regs.h:655. Stride 4.
pub const fn mt_wcid_attr(n: u32) -> u32 {
    MT_WCID_ATTR_BASE + n * 4
}
pub const MT_WCID_ATTR_PAIRWISE: u32 = 0x0000_0001; // BIT(0) — :657
pub const MT_WCID_ATTR_PKEY_MODE: u32 = 0x0000_000e; // GENMASK(3,1) — :658
pub const MT_WCID_ATTR_BSS_IDX: u32 = 0x0000_0070; // GENMASK(6,4) — :659
pub const MT_WCID_ATTR_RXWI_UDF: u32 = 0x0000_0380; // GENMASK(9,7) — :660
pub const MT_WCID_ATTR_PKEY_MODE_EXT: u32 = 0x0000_0400; // BIT(10) — :661
pub const MT_WCID_ATTR_BSS_IDX_EXT: u32 = 0x0000_0800; // BIT(11) — :662
pub const MT_WCID_ATTR_WAPI_MCBC: u32 = 0x0000_8000; // BIT(15) — :663
pub const MT_WCID_ATTR_WAPI_KEYID: u32 = 0xff00_0000; // GENMASK(31,24) — :664

pub const MT_SKEY_BASE_0: u32 = 0xac00; // mt76x02_regs.h:666
pub const MT_SKEY_BASE_1: u32 = 0xb400; // mt76x02_regs.h:667
/// `MT_SKEY_0(_bss,_idx)` — mt76x02_regs.h:668.
pub const fn mt_skey_0(bss: u32, idx: u32) -> u32 {
    MT_SKEY_BASE_0 + (4 * bss + idx) * 32
}
/// `MT_SKEY_1(_bss,_idx)` — mt76x02_regs.h:669.
pub const fn mt_skey_1(bss: u32, idx: u32) -> u32 {
    MT_SKEY_BASE_1 + (4 * (bss & 7) + idx) * 32
}
/// `MT_SKEY(_bss,_idx)` — mt76x02_regs.h:670. Bit 3 of `bss` selects the bank.
pub const fn mt_skey(bss: u32, idx: u32) -> u32 {
    if bss & 8 != 0 {
        mt_skey_1(bss, idx)
    } else {
        mt_skey_0(bss, idx)
    }
}

pub const MT_SKEY_MODE_BASE_0: u32 = 0xb000; // mt76x02_regs.h:672
pub const MT_SKEY_MODE_BASE_1: u32 = 0xb3f0; // mt76x02_regs.h:673
/// `MT_SKEY_MODE_0(_bss)` — mt76x02_regs.h:674.
pub const fn mt_skey_mode_0(bss: u32) -> u32 {
    MT_SKEY_MODE_BASE_0 + ((bss / 2) << 2)
}
/// `MT_SKEY_MODE_1(_bss)` — mt76x02_regs.h:675.
pub const fn mt_skey_mode_1(bss: u32) -> u32 {
    MT_SKEY_MODE_BASE_1 + (((bss & 7) / 2) << 2)
}
/// `MT_SKEY_MODE(_bss)` — mt76x02_regs.h:676.
pub const fn mt_skey_mode(bss: u32) -> u32 {
    if bss & 8 != 0 {
        mt_skey_mode_1(bss)
    } else {
        mt_skey_mode_0(bss)
    }
}
pub const MT_SKEY_MODE_MASK: u32 = 0x0000_000f; // GENMASK(3,0) — :677
/// `MT_SKEY_MODE_SHIFT(_bss,_idx)` — mt76x02_regs.h:678.
pub const fn mt_skey_mode_shift(bss: u32, idx: u32) -> u32 {
    4 * (idx + 4 * (bss & 1))
}

/// Beacon-frame SRAM window, indexed through [`mt_bcn_offset`].
pub const MT_BEACON_BASE: u32 = 0xc000; // mt76x02_regs.h:680

/// On-die temperature. ⚠ 5-nibble address, above the MAC window — reached the
/// same way as [`MT_WLAN_MTC_CTRL`]. The reading is a raw 7-bit code, not °C;
/// upstream converts it with a per-part EEPROM slope, so treat the raw value as
/// an index until that path is ported.
pub const MT_TEMP_SENSOR: u32 = 0x0001_d000; // mt76x02_regs.h:682
pub const MT_TEMP_SENSOR_VAL: u32 = 0x0000_007f; // GENMASK(6,0) — :683

// ── BBP (baseband) ───────────────────────────────────────────────────────────
// Upstream reaches the baseband with `MT_BBP(_type, _n)` = `MT_BBP_##_type##_BASE
// + (_n << 2)` (mt76x02_regs.h:619), which token-pastes a *type name*. Rust has
// no token paste, so the base is passed as a value: `mt_bbp(MT_BBP_AGC_BASE, 8)`
// reads as `MT_BBP(AGC, 8)`. All fourteen bases are transcribed (the brief
// mentions ten; the header defines fourteen).

pub const MT_BBP_CORE_BASE: u32 = 0x2000; // mt76x02_regs.h:604
pub const MT_BBP_IBI_BASE: u32 = 0x2100; // mt76x02_regs.h:605
pub const MT_BBP_AGC_BASE: u32 = 0x2300; // mt76x02_regs.h:606
pub const MT_BBP_TXC_BASE: u32 = 0x2400; // mt76x02_regs.h:607
pub const MT_BBP_RXC_BASE: u32 = 0x2500; // mt76x02_regs.h:608
pub const MT_BBP_TXO_BASE: u32 = 0x2600; // mt76x02_regs.h:609
pub const MT_BBP_TXBE_BASE: u32 = 0x2700; // mt76x02_regs.h:610
pub const MT_BBP_RXFE_BASE: u32 = 0x2800; // mt76x02_regs.h:611
pub const MT_BBP_RXO_BASE: u32 = 0x2900; // mt76x02_regs.h:612
pub const MT_BBP_DFS_BASE: u32 = 0x2a00; // mt76x02_regs.h:613
pub const MT_BBP_TR_BASE: u32 = 0x2b00; // mt76x02_regs.h:614
pub const MT_BBP_CAL_BASE: u32 = 0x2c00; // mt76x02_regs.h:615
pub const MT_BBP_DSC_BASE: u32 = 0x2e00; // mt76x02_regs.h:616
pub const MT_BBP_PFMU_BASE: u32 = 0x2f00; // mt76x02_regs.h:617

/// `MT_BBP(_type, _n)` — mt76x02_regs.h:619, with the base passed by value.
pub const fn mt_bbp(base: u32, n: u32) -> u32 {
    base + (n << 2)
}

/// `MT_BBP(CORE, 1)` bits 4:3 — the baseband bandwidth selector, written by
/// every bandwidth change (`mt76x02_phy.c:143`).
pub const MT_BBP_CORE_R1_BW: u32 = 0x0000_0018; // GENMASK(4,3) — :621

/// `MT_BBP(AGC, 0)` bits 9:8 — which 20 MHz slot inside the operating bandwidth
/// carries the primary channel (`mt76x02_phy.c:145`).
pub const MT_BBP_AGC_R0_CTRL_CHAN: u32 = 0x0000_0300; // GENMASK(9,8) — :623
/// `MT_BBP(AGC, 0)` bits 14:12 — the AGC's bandwidth (`mt76x02_phy.c:144`).
pub const MT_BBP_AGC_R0_BW: u32 = 0x0000_7000; // GENMASK(14,12) — :624

// AGC R4/R5 — the LNA gain table, three gain steps per register.
pub const MT_BBP_AGC_LNA_HIGH_GAIN: u32 = 0x003f_0000; // GENMASK(21,16) — :627
pub const MT_BBP_AGC_LNA_MID_GAIN: u32 = 0x0000_3f00; // GENMASK(13,8) — :628
pub const MT_BBP_AGC_LNA_LOW_GAIN: u32 = 0x0000_003f; // GENMASK(5,0) — :629
// AGC R6/R7 — the fourth (ultra-low) step.
pub const MT_BBP_AGC_LNA_ULOW_GAIN: u32 = 0x0000_003f; // GENMASK(5,0) — :632
// AGC R8/R9 — the live gain the VGA loop moves.
pub const MT_BBP_AGC_LNA_GAIN_MODE: u32 = 0x0000_00c0; // GENMASK(7,6) — :635
/// `MT_BBP(AGC, 8)` bits 14:8 — the VGA gain that `mt76x0_phy_set_gain_val`
/// writes (`mt76x0/phy.c:1060`) and `mt76x02_init_agc_gain` snapshots
/// (`mt76x02_phy.c:195-197`). This is the RX-sensitivity actuator.
pub const MT_BBP_AGC_GAIN: u32 = 0x0000_7f00; // GENMASK(14,8) — :636

/// `MT_BBP(AGC, 20)` bits 7:0 / 15:8 — per-chain raw RSSI readback.
pub const MT_BBP_AGC20_RSSI0: u32 = 0x0000_00ff; // GENMASK(7,0) — :638
pub const MT_BBP_AGC20_RSSI1: u32 = 0x0000_ff00; // GENMASK(15,8) — :639

/// `MT_BBP(TXBE, 0)` bits 1:0 — the TX-side copy of the primary-channel slot
/// (`mt76x02_phy.c:146`); must be kept consistent with
/// [`MT_BBP_AGC_R0_CTRL_CHAN`].
pub const MT_BBP_TXBE_R0_CTRL_CHAN: u32 = 0x0000_0003; // GENMASK(1,0) — :641

// ── RF banks (MT76x0) ────────────────────────────────────────────────────────
// RF registers are addressed as a (bank, reg) pair packed into one u32 and are
// **8 bits wide**, hence the u8 masks. `mt76x0::phy` reaches them through the
// MCU register-pair protocol with base MT_MCU_MEMMAP_RF.

/// `MT_RF(bank, reg)` — `mt76x0/phy.h:21`.
pub const fn mt_rf(bank: u32, reg: u32) -> u32 {
    (bank << 16) | reg
}
/// `MT_RF_BANK(offset)` — `mt76x0/phy.h:22`.
pub const fn mt_rf_bank(offset: u32) -> u32 {
    offset >> 16
}
/// `MT_RF_REG(offset)` — `mt76x0/phy.h:23`. ⚠ Masks with **0xff**, not 0xffff,
/// so a register index above 255 would silently truncate. Ported as upstream
/// writes it.
pub const fn mt_rf_reg(offset: u32) -> u32 {
    offset & 0xff
}

// VCO / calibration timing fields (mt76x0/phy.h:25-30).
pub const MT_RF_VCO_BP_CLOSE_LOOP: u8 = 0x08; // BIT(3)
pub const MT_RF_VCO_BP_CLOSE_LOOP_MASK: u8 = 0x0f; // GENMASK(3,0)
pub const MT_RF_VCO_CAL_MASK: u8 = 0x07; // GENMASK(2,0)
/// Not a mask — the literal value 3 written into [`MT_RF_START_TIME_MASK`].
pub const MT_RF_START_TIME: u8 = 0x3; // mt76x0/phy.h:28
pub const MT_RF_START_TIME_MASK: u8 = 0x07; // GENMASK(2,0) — mt76x0/phy.h:29
pub const MT_RF_SETTLE_TIME_MASK: u8 = 0x70; // GENMASK(6,4) — mt76x0/phy.h:30

// PLL / sigma-delta fields — these are the destinations of every `pll_r*` field
// in `crate::mt76x0::freq_plan::FreqItem` (mt76x0/phy.h:32-40).
pub const MT_RF_PLL_DEN_MASK: u8 = 0x1f; // GENMASK(4,0)
pub const MT_RF_PLL_K_MASK: u8 = 0x1f; // GENMASK(4,0)
pub const MT_RF_SDM_RESET_MASK: u8 = 0x80; // BIT(7)
pub const MT_RF_SDM_MASH_PRBS_MASK: u8 = 0x7c; // GENMASK(6,2)
pub const MT_RF_SDM_BP_MASK: u8 = 0x02; // BIT(1)
pub const MT_RF_ISI_ISO_MASK: u8 = 0xc0; // GENMASK(7,6)
pub const MT_RF_PFD_DLY_MASK: u8 = 0x30; // GENMASK(5,4)
pub const MT_RF_CLK_SEL_MASK: u8 = 0x0c; // GENMASK(3,2)
pub const MT_RF_XO_DIV_MASK: u8 = 0x03; // GENMASK(1,0)

// ── ED-CCA loop thresholds ───────────────────────────────────────────────────
// Not registers — the software thresholds `mt76x02_mac.c:1139-1143` applies to
// the MT_ED_CCA_TIMER reading. Kept here so the knob layer does not reinvent
// them.

/// Busy percentage above which the ED-CCA loop counts a "triggered" window.
pub const MT_EDCCA_TH: u32 = 92; // mt76x02_mac.c:1139
/// Consecutive triggered windows before TX is blocked. mt76x02_mac.c:1140
pub const MT_EDCCA_BLOCK_TH: u32 = 2;
/// Consecutive triggered windows before the loop leaves learning mode.
pub const MT_EDCCA_LEARN_TH: u32 = 50; // mt76x02_mac.c:1141
/// False-CCA count that, with the AGC at its lowest gain, ends learning mode.
pub const MT_EDCCA_LEARN_CCA: u32 = 180; // mt76x02_mac.c:1142
/// Learning-mode timeout, in seconds (upstream writes `20 * HZ`).
pub const MT_EDCCA_LEARN_TIMEOUT_SECS: u64 = 20; // mt76x02_mac.c:1143

/// The two ED-CCA energy thresholds written into `MT_BBP(AGC, 2)` bits 15:0 as
/// `ed_th << 8 | ed_th` when ED-CCA is armed (`mt76x02_mac.c:1110,1115-1116`):
/// `0x0e` on 5 GHz, `0x20` on 2.4 GHz.
pub const MT_EDCCA_BBP_TH_5G: u32 = 0x0e;
/// See [`MT_EDCCA_BBP_TH_5G`].
pub const MT_EDCCA_BBP_TH_2G: u32 = 0x20;

// ── MEASURED live values (the oracle) ────────────────────────────────────────

/// Values read off **mds-o5p-1's MT7610U on 2026-08-27** with
/// `examples/mt76_oracle.rs`, while the kernel `mt76x0u` driver held the part in
/// monitor mode. They are recorded as constants — not as prose — so a bring-up
/// can assert against them and so a future reader can tell measurement from
/// inference. Reasoning about these radios has a ~0% hit rate; measurement has
/// ~100%.
pub mod measured {
    /// TSF tick rate once [`super::MT_BEACON_TIME_CFG_TIMER_EN`] is set: exactly
    /// 1 µs per tick, so a TSF delta *is* microseconds.
    pub const TSF_TICK_HZ: u32 = 1_000_000;

    /// One EP0 vendor-request round trip. A register access is affordable in a
    /// channel switch or a 100 ms sensing window and is **never** affordable on
    /// a per-frame path.
    pub const EP0_ROUND_TRIP_US: u32 = 151;

    /// [`super::MT_RX_FILTR_CFG`] as left by the kernel monitor.
    pub const KERNEL_MONITOR_RX_FILTR_CFG: u32 = 0x0000_1093;
    /// [`super::MT_MAC_SYS_CTRL`] as left by the kernel monitor: TX+RX enabled.
    pub const KERNEL_MONITOR_MAC_SYS_CTRL: u32 = 0x0000_000c;
    /// [`super::MT_EXT_CCA_CFG`] as left by the kernel monitor.
    pub const KERNEL_MONITOR_EXT_CCA_CFG: u32 = 0x0000_f1e4;
    /// [`super::MT_TXOP_CTRL_CFG`] as left by the kernel monitor — identical to
    /// `mt76x0/initvals_init.h:40`.
    pub const KERNEL_MONITOR_TXOP_CTRL_CFG: u32 = 0x0000_583f;
    /// `MT_BBP(AGC, 2)` as left by the kernel monitor — the ED-CCA-off value
    /// from `mt76x02_mac.c:1126`.
    pub const KERNEL_MONITOR_BBP_AGC2: u32 = 0x003a_6464;
    /// [`super::MT_BKOFF_SLOT_CFG`] as left by the kernel monitor — identical to
    /// `mt76x0/initvals_init.h:21` (9 µs slot).
    pub const KERNEL_MONITOR_BKOFF_SLOT_CFG: u32 = 0x0000_0209;

    /// The value `mt76x0/initvals_init.h:20` writes to
    /// [`super::MT_RX_FILTR_CFG`] before mac80211 relaxes it. Recorded because
    /// [`KERNEL_MONITOR_RX_FILTR_CFG`] is derived from it exactly.
    pub const INITVAL_RX_FILTR_CFG: u32 = 0x0001_7f97;
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The address arithmetic upstream expresses as macros, checked against
    /// hand-computed values so a typo in a base cannot pass silently.
    #[test]
    fn address_helpers_match_upstream_macros() {
        // MT_BBP(AGC, 2) — the ED-CCA register whose live value we measured.
        assert_eq!(mt_bbp(MT_BBP_AGC_BASE, 2), 0x2308);
        // MT_BBP(CORE, 1) / MT_BBP(TXBE, 0), the bandwidth pair.
        assert_eq!(mt_bbp(MT_BBP_CORE_BASE, 1), 0x2004);
        assert_eq!(mt_bbp(MT_BBP_TXBE_BASE, 0), 0x2700);
        // RF (bank, reg) packing and its two inverses.
        assert_eq!(mt_rf(5, 0x2c), 0x0005_002c);
        assert_eq!(mt_rf_bank(mt_rf(5, 0x2c)), 5);
        assert_eq!(mt_rf_reg(mt_rf(5, 0x2c)), 0x2c);
        // Non-uniform strides.
        assert_eq!(mt_efuse_data(3), 0x0034);
        assert_eq!(mt_edca_cfg_ac(3), 0x130c);
        assert_eq!(mt_tx_agg_cnt(7), 0x173c);
        assert_eq!(mt_tx_agg_cnt(8), 0x174c);
        assert_eq!(mt_wcid_addr(2), 0x1810);
        assert_eq!(mt_led_s0(1), 0x0784);
    }

    /// The composed masks upstream builds with `&~`.
    #[test]
    fn composed_masks() {
        assert_eq!(MT_PROT_TXOP_ALLOW_BW20, 0x0170_0000);
        assert_eq!(MT_MCU_IVB_ADDR, 0x000d_3fc0);
    }

    /// `field_prep`/`field_get` round-trip on the fields the knob layer moves.
    #[test]
    fn field_helpers_round_trip() {
        assert_eq!(field_prep(MT_EDCA_CFG_AIFSN, 7), 0x0000_0700);
        assert_eq!(field_get(MT_EDCA_CFG_AIFSN, 0x0000_0700), 7);
        assert_eq!(field_prep(MT_BBP_AGC_GAIN, 0x2a), 0x0000_2a00);
        assert_eq!(field_get(MT_BBP_AGC_GAIN, 0x0000_2a00), 0x2a);
    }

    /// ★ The measured [`MT_EXT_CCA_CFG`] must decompose to `mt76x0/phy.c:917-921`
    /// channel-group 0 plus `ED_CCA_MASK = 0xf`. If this test ever fails, either
    /// a mask above is wrong or the silicon is not the one we measured.
    #[test]
    fn measured_ext_cca_cfg_decomposes_to_group_zero() {
        let v = measured::KERNEL_MONITOR_EXT_CCA_CFG;
        assert_eq!(field_get(MT_EXT_CCA_CFG_CCA0, v), 0);
        assert_eq!(field_get(MT_EXT_CCA_CFG_CCA1, v), 1);
        assert_eq!(field_get(MT_EXT_CCA_CFG_CCA2, v), 2);
        assert_eq!(field_get(MT_EXT_CCA_CFG_CCA3, v), 3);
        assert_eq!(field_get(MT_EXT_CCA_CFG_CCA_MASK, v), 0b0001);
        assert_eq!(field_get(MT_EXT_CCA_CFG_ED_CCA_MASK, v), 0xf);
    }

    /// ★ The measured [`MT_TXOP_CTRL_CFG`] must decompose to the init-table
    /// value with ED-CCA **off**.
    #[test]
    fn measured_txop_ctrl_cfg_decomposes() {
        let v = measured::KERNEL_MONITOR_TXOP_CTRL_CFG;
        assert_eq!(field_get(MT_TXOP_TRUN_EN, v), 0x3f);
        assert_eq!(field_get(MT_TXOP_EXT_CCA_DLY, v), 0x58);
        assert_eq!(v & MT_TXOP_ED_CCA_EN, 0, "ED-CCA must be off as found");
    }

    /// ★ The measured [`MT_RX_FILTR_CFG`] must equal the init value minus
    /// exactly the bits mac80211's monitor path clears — PROMISC
    /// (`mt76x0/main.c:84`) and the CONTROL/PSPOLL group
    /// (`mt76x02_util.c:223-229`). RTS survives because upstream's CONTROL group
    /// does not list it.
    #[test]
    fn measured_rx_filtr_cfg_is_initval_minus_monitor_clears() {
        let cleared = MT_RX_FILTR_CFG_PROMISC
            | MT_RX_FILTR_CFG_ACK
            | MT_RX_FILTR_CFG_CTS
            | MT_RX_FILTR_CFG_CFEND
            | MT_RX_FILTR_CFG_CFACK
            | MT_RX_FILTR_CFG_BA
            | MT_RX_FILTR_CFG_CTRL_RSV
            | MT_RX_FILTR_CFG_PSPOLL;
        assert_eq!(
            measured::INITVAL_RX_FILTR_CFG & !cleared,
            measured::KERNEL_MONITOR_RX_FILTR_CFG
        );
        // And RTS is still dropped, which is why a monitor sees no RTS frames.
        assert_ne!(
            measured::KERNEL_MONITOR_RX_FILTR_CFG & MT_RX_FILTR_CFG_RTS,
            0
        );
    }

    /// ★ The measured [`MT_MAC_SYS_CTRL`] is exactly TX+RX enabled.
    #[test]
    fn measured_mac_sys_ctrl_is_tx_plus_rx() {
        assert_eq!(
            measured::KERNEL_MONITOR_MAC_SYS_CTRL,
            MT_MAC_SYS_CTRL_ENABLE_TX | MT_MAC_SYS_CTRL_ENABLE_RX
        );
    }
}
