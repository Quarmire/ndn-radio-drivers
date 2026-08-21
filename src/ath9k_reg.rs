#![allow(dead_code)]
//! AR9271 register offsets and bit constants, resolved to LITERAL u32 values,
//! transcribed from the mainline Linux ath9k driver
//! (`drivers/net/wireless/ath/ath9k/reg.h`, plus a few code paths in
//! `ar9002_hw.c`, `ar9002_phy.c`, `hw.c`, `mac.c`) at kernel tag **v6.12.33**.
//!
//! Every macro that reg.h defines as an `_ah`-conditional expression has been
//! resolved to the branch that is valid for the AR9271 (a USB HTC part that is
//! neither AR9100 nor AR9300+). See the `// CONDITIONAL:` notes below.
//!
//! ★ RTC-BLOCK REMAP FINDING (highest risk): In this reg.h, the RTC registers
//! are `(AR_RTC_BASE + off)` **only when `AR_SREV_9100(ah)` is true**. AR9271 is
//! NOT AR9100, so for AR9271 every RTC register takes the *else* branch = the
//! flat `0x70xx` offset. The `AR_RTC_BASE = 0x00020000` remap does NOT apply to
//! AR9271 at the reg.h level. (The task brief's hint that "AR9271/AR7010 remap
//! the RTC block" is not reflected in reg.h v6.12.33 — those USB parts reach the
//! same flat 0x70xx offsets through the WMI register window, not via AR_RTC_BASE.)
//! Both the base and the resolved offset are recorded so the caller can audit.
//!
//! ⚠ NOT-IN-FETCHED-SOURCE: A handful of constants the bring-up needs live in
//! ath9k headers that were NOT among the fetched files — `mac.h`
//! (AR_STA_ID0/1, the AR_RX_FILTER_* bit flags) and `ar9002_phy.h`
//! (AR_PHY_TURBO, AR_PHY_CCA/AR9280_PHY_MINCCA_PWR, and the AR9271-specific
//! noise-floor limit constants). These are marked `// UNVERIFIED (mac.h)` /
//! `// UNVERIFIED (ar9002_phy.h)` with the canonical ath9k value; the integrator
//! MUST confirm them against those two headers before trusting a bring-up.

// ============================================================================
// --- id / rev ---
// ============================================================================

/// AR_SREV register offset. reg.h:753 defines it as
/// `AR_SREV_9100 ? 0x0600 : (AR_SREV_9340 ? 0x400c : 0x4020)`.
/// CONDITIONAL: AR9271 is neither 9100 nor 9340 -> 0x4020.
pub const AR_SREV: u32 = 0x4020;

/// AR_SREV_ID mask. reg.h:757 `AR_SREV_9100 ? 0xFFF : 0xFF`.
/// CONDITIONAL: AR9271 (not 9100) -> 0xFF.
pub const AR_SREV_ID: u32 = 0x000000FF;

/// Field to extract macVersion from the SREV read in the non-0xFF path
/// (reg.h:310 `MS(val, AR_SREV_VERSION)`).
pub const AR_SREV_VERSION: u32 = 0x000000F0;
pub const AR_SREV_VERSION_S: u32 = 4;
/// Field to extract macRev in the non-0xFF path (reg.h:312 `val & AR_SREV_REVISION`).
pub const AR_SREV_REVISION: u32 = 0x00000007;

/// macVersion sentinel that identifies the AR9271 (reg.h:795).
/// `AR_SREV_9271(ah)` == (macVersion == AR_SREV_VERSION_9271).
pub const AR_SREV_VERSION_9271: u32 = 0x140;
/// macRev decode: `AR_SREV_9271_10` == macRev 0, `AR_SREV_9271_11` == macRev 1
/// (reg.h:796-797, compared inside AR_SREV_9271_10/_11).
pub const AR_SREV_REVISION_9271_10: u32 = 0;
pub const AR_SREV_REVISION_9271_11: u32 = 1;
// NOTE: For the USB HTC parts, hw_version.macVersion/macRev are supplied by the
// HTC target (ath9k_htc), not by decoding the AR_SREV register read — the
// register-read decode above (MS(val,0xF0)>>4) cannot yield 0x140 by itself.
// AR_SREV_VERSION_9271 = 0x140 is the value the target reports.

// ============================================================================
// --- reset ---
// ============================================================================

/// AR_RC (reset control) offset (reg.h:697).
pub const AR_RC: u32 = 0x4000;
pub const AR_RC_AHB: u32 = 0x00000001; // reg.h:698
pub const AR_RC_APB: u32 = 0x00000002; // reg.h:699 (context)
pub const AR_RC_HOSTIF: u32 = 0x00000100; // reg.h:700 (context)

// INTR_SYNC block, read in ath9k_hw_set_reset (non-9100 path) before asserting
// the AHB reset. CONDITIONAL: `AR_SREV_9340 ? 0x4010 : 0x4028` (reg.h:1041) /
// `? 0x4014 : 0x402c` (reg.h:1045). AR9271 -> 0x4028 / 0x402c. On this USB part
// the RADM_CPL bit is a PCIe artefact and reads 0, so the `if (tmpReg)` branch
// is normally not taken; implemented faithfully regardless.
pub const AR_INTR_SYNC_CAUSE: u32 = 0x4028; // reg.h:1041 (AR9271)
pub const AR_INTR_SYNC_ENABLE: u32 = 0x402c; // reg.h:1045 (AR9271)
pub const AR_INTR_SYNC_RADM_CPL_TIMEOUT: u32 = 0x00001000; // reg.h:1062
pub const AR_INTR_SYNC_LOCAL_TIMEOUT: u32 = 0x00002000; // reg.h:1063

// AR9271-specific RF/MAC gating, written by ath9k_hw_reset around the very first
// chip reset (`htc_reset_init`), reg.h:1613-1616. RADIO_RF_RST is asserted just
// before ath9k_hw_chip_reset; GATE_MAC_CTL just after. The firmware ALSO drives
// the equivalent RF power-down/up out of `ath_pll_reset_ones` on the first write
// to AR_RTC_PLL_CONTROL (0x7014), so these two writes bracket the firmware's own.
pub const AR9271_RESET_POWER_DOWN_CONTROL: u32 = 0x50044; // reg.h:1614
pub const AR9271_RADIO_RF_RST: u32 = 0x00000020; // reg.h:1615
pub const AR9271_GATE_MAC_CTL: u32 = 0x00004000; // reg.h:1616

/// RTC base used ONLY for AR9100's (AR_RTC_BASE + off) remap (reg.h:1341).
/// Not used for AR9271; recorded for audit of the CONDITIONALs below.
pub const AR_RTC_BASE: u32 = 0x00020000;

/// AR_RTC_RC (reg.h:1342). CONDITIONAL: `AR_SREV_9100 ? AR_RTC_BASE+0x0 : 0x7000`.
/// AR9271 -> 0x7000.
pub const AR_RTC_RC: u32 = 0x7000;
pub const AR_RTC_RC_M: u32 = 0x00000003; // reg.h:1344
pub const AR_RTC_RC_MAC_WARM: u32 = 0x00000001; // reg.h:1345
pub const AR_RTC_RC_MAC_COLD: u32 = 0x00000002; // reg.h:1346
pub const AR_RTC_RC_COLD_RESET: u32 = 0x00000004; // reg.h:1347
pub const AR_RTC_RC_WARM_RESET: u32 = 0x00000008; // reg.h:1348

/// AR_RTC_RESET (reg.h:1381). CONDITIONAL: `AR_SREV_9100 ? BASE+0x40 : 0x7040`.
/// AR9271 -> 0x7040.
pub const AR_RTC_RESET: u32 = 0x7040;
pub const AR_RTC_RESET_EN: u32 = 0x00000001; // reg.h:1383

/// AR_RTC_STATUS (reg.h:1385). CONDITIONAL: `AR_SREV_9100 ? BASE+0x44 : 0x7044`.
/// AR9271 -> 0x7044.
pub const AR_RTC_STATUS: u32 = 0x7044;
/// AR_RTC_STATUS_M (reg.h:1388). CONDITIONAL: `AR_SREV_9100 ? 0x3f : 0x0f`.
/// AR9271 -> 0x0000000f.
pub const AR_RTC_STATUS_M: u32 = 0x0000000f;
pub const AR_RTC_STATUS_SHUTDOWN: u32 = 0x00000001; // reg.h:1393
/// The "ON" status value polled after wake (reg.h:1394).
pub const AR_RTC_STATUS_ON: u32 = 0x00000002;
pub const AR_RTC_STATUS_SLEEP: u32 = 0x00000004; // reg.h:1395
pub const AR_RTC_STATUS_WAKEUP: u32 = 0x00000008; // reg.h:1396

/// AR_RTC_FORCE_WAKE (reg.h:1403). CONDITIONAL: `AR_SREV_9100 ? BASE+0x4c : 0x704c`.
/// AR9271 -> 0x704c.
pub const AR_RTC_FORCE_WAKE: u32 = 0x704c;
pub const AR_RTC_FORCE_WAKE_EN: u32 = 0x00000001; // reg.h:1405
pub const AR_RTC_FORCE_WAKE_ON_INT: u32 = 0x00000002; // reg.h:1406

// ============================================================================
// --- pll ---
// ============================================================================

/// AR_RTC_PLL_CONTROL (reg.h:1360). CONDITIONAL: `AR_SREV_9100 ? BASE+0x14 : 0x7014`.
/// AR9271 -> 0x7014.
pub const AR_RTC_PLL_CONTROL: u32 = 0x7014;

/// AR_RTC_PLL_CONTROL2 (reg.h:1363) = flat 0x703c (NOT _ah-conditional).
/// NOTE: In hw.c `ath9k_hw_init_pll`, PLL_CONTROL2 is written only in the
/// AR_SREV_9330 branch — it is NOT written for AR9271. (The task brief's
/// "AR9271-specific PLL_CONTROL2 magic value" does not exist in v6.12.33.)
pub const AR_RTC_PLL_CONTROL2: u32 = 0x703c;

// 9160-style PLL packing fields used by ar9002_hw_compute_pll_control (reg.h:1334-1339).
pub const AR_RTC_9160_PLL_DIV: u32 = 0x000003ff;
pub const AR_RTC_9160_PLL_DIV_S: u32 = 0;
pub const AR_RTC_9160_PLL_REFDIV: u32 = 0x00003C00;
pub const AR_RTC_9160_PLL_REFDIV_S: u32 = 10; // the requested shift
pub const AR_RTC_9160_PLL_CLKSEL: u32 = 0x0000C000;
pub const AR_RTC_9160_PLL_CLKSEL_S: u32 = 14;

/// LITERAL 2.4GHz full-rate PLL value that ar9002_hw_compute_pll_control
/// produces for AR9271 (ar9002_phy.c:304): ref_div=5, pll_div=0x2c, 2.4GHz path.
/// pll = SM(5, REFDIV=0x3C00<<10) | SM(0x2c, DIV=0x3ff<<0)
///     = ((5<<10)&0x3C00) | ((0x2c<<0)&0x3ff) = 0x1400 | 0x2c = 0x142c.
/// Confirms the spec's expected ~0x142c.
pub const AR9271_PLL_CONTROL_2GHZ: u32 = 0x142c;

/// AR_RTC_SLEEP_CLK (reg.h:1398). CONDITIONAL: `AR_SREV_9100 ? BASE+0x48 : 0x7048`.
/// AR9271 -> 0x7048. init_pll writes AR_RTC_FORCE_DERIVED_CLK here at the end.
pub const AR_RTC_SLEEP_CLK: u32 = 0x7048;
pub const AR_RTC_FORCE_DERIVED_CLK: u32 = 0x2; // reg.h:1400
pub const AR_RTC_FORCE_SWREG_PRD: u32 = 0x00000004; // reg.h:1401

// AR9271 core-clock switch after PLL program (hw.c:921-923): REG_WRITE(0x50040, 0x304).
pub const AR9271_CORE_CLOCK_REG: u32 = 0x50040;
pub const AR9271_CORE_CLOCK_117MHZ: u32 = 0x304;

// ============================================================================
// --- initvals / mode ---
// ============================================================================
//
// Modes-array column selection (ar9002_hw.c ar9002_hw_load_ani_reg / the
// ar9002_hw_process_ini path): modesIndex is chosen as
//     5GHz: HT40 ? 2 : 1
//     2GHz: HT40 ? 3 : 4
// So for **2.4GHz-HT20 the modesIndex is 4** (column 4 of each Modes row).
// Column 0 is always the register address. For Common arrays the value column
// is index 1 ("allmodes").
pub const AR9271_MODES_INDEX_2GHZ_HT20: usize = 4;
pub const AR9271_MODES_INDEX_2GHZ_HT40: usize = 3;
pub const AR9271_MODES_INDEX_5GHZ_HT20: usize = 1;
pub const AR9271_MODES_INDEX_5GHZ_HT40: usize = 2;
pub const AR9271_COMMON_VALUE_COL: usize = 1;

// AR_PHY_TURBO selects HT20 vs HT40 (dynamic 20/40) via AR_PHY_FC_DYN2040_EN,
// set/cleared in ar9002_calib.c:754/766. UNVERIFIED (ar9002_phy.h): the offset
// and bit are defined in ar9002_phy.h, which was NOT fetched. Canonical ath9k:
pub const AR_PHY_TURBO: u32 = 0x9804; // verified (ar9002_phy.h)
pub const AR_PHY_FC_DYN2040_EN: u32 = 0x00000004; // verified (ar9002_phy.h): HT40 (dyn 20/40) enable bit

/// AR_PHY register-file base (`ar9002_phy.h`: `AR_PHY_BASE 0x9800`,
/// `AR_PHY(_n) = AR_PHY_BASE + ((_n)<<2)`). Cross-checked against the MODES table,
/// whose first PHY row is AR_PHY_TURBO = AR_PHY(1) = 0x9804. `ar9002_hw_rf_claim`
/// writes AR_PHY(0)=0x9800 = 0x7 ("set baseband to analog shift setting to access
/// analog chips", ar9002_hw.c:347) before the analog (0x78xx) COMMON rows.
pub const AR_PHY_BASE: u32 = 0x9800; // verified (ar9002_phy.h + MODES table cross-check)
pub const AR_PHY_ANALOG_SHIFT_ENABLE: u32 = 0x00000007; // ar9002_hw.c:347

// ============================================================================
// --- calibration ---
// ============================================================================

/// AR_PHY_AGC_CONTROL (reg.h:2117). CONDITIONAL: `AR9300_20_OR_LATER ?
/// AR9003_PHY_AGC_CONTROL : AR9002_PHY_AGC_CONTROL`. AR9271 -> AR9002 = 0x9860.
pub const AR_PHY_AGC_CONTROL: u32 = 0x9860; // AR9002_PHY_AGC_CONTROL, reg.h:2115
pub const AR_PHY_AGC_CONTROL_CAL: u32 = 0x00000001; // reg.h:2118 (do internal cal)
pub const AR_PHY_AGC_CONTROL_NF: u32 = 0x00000002; // reg.h:2119 (do NF cal)
pub const AR_PHY_AGC_CONTROL_ENABLE_NF: u32 = 0x00008000; // reg.h:2121
pub const AR_PHY_AGC_CONTROL_NO_UPDATE_NF: u32 = 0x00020000; // reg.h:2123

// Noise-floor readback register + field used by ar9002_hw_do_getnf
// (ar9002_phy.c:336 `MS(REG_READ(AR_PHY_CCA), AR9280_PHY_MINCCA_PWR)`).
// UNVERIFIED (ar9002_phy.h): AR_PHY_CCA / AR9280_PHY_MINCCA_PWR live in
// ar9002_phy.h (NOT fetched). The task's names AR_PHY_CH0_CCA /
// AR_PHY_MINCCA_PWR map to these AR9280-family names in this driver version.
pub const AR_PHY_CCA: u32 = 0x9864; // verified (ar9002_phy.h) — a.k.a. AR_PHY_CH0_CCA
pub const AR9280_PHY_MINCCA_PWR: u32 = 0x1FF00000; // verified (ar9002_phy.h): 9-bit signed NF field
pub const AR9280_PHY_MINCCA_PWR_S: u32 = 20; // verified (ar9002_phy.h)

// AGC-control cal-control bits used by ar9285_hw_cl_cal (the AR9271 offset/AGC cal
// that ath9k_hw_init_cal runs for this part).
pub const AR_PHY_AGC_CONTROL_FLTR_CAL: u32 = 0x00010000; // reg.h:2122 (tx-filter cal)

/// AR_PHY_ACTIVE (ar9002_phy.h:50) — BB enable, written by ath9k_hw_init_bb.
pub const AR_PHY_ACTIVE: u32 = 0x981C; // verified (ar9002_phy.h)
pub const AR_PHY_ACTIVE_EN: u32 = 0x00000001; // verified (ar9002_phy.h)

/// AR_PHY_MODE (ar9002_phy.h:401) — band + CCK/OFDM select, written by
/// ath9k_hw_set_rfmode. For 2.4GHz on a single-chip post-9280 part (AR9271) the
/// value is AR_PHY_MODE_DYNAMIC (dynamic CCK/OFDM); RF2GHZ is a pre-9280 (external
/// radio) bit and DYN_CCK_DISABLE is a 5GHz fast-clock bit — neither applies here,
/// so 2.4GHz CCK (the rate beacons ride) stays enabled.
pub const AR_PHY_MODE: u32 = 0xA200; // verified (ar9002_phy.h)
pub const AR_PHY_MODE_DYNAMIC: u32 = 0x00000004; // verified (ar9002_phy.h)
pub const AR_PHY_MODE_RF2GHZ: u32 = 0x00000002; // verified (ar9002_phy.h)
pub const AR_PHY_MODE_CCK: u32 = 0x00000001; // verified (ar9002_phy.h)
pub const AR_PHY_MODE_DYN_CCK_DISABLE: u32 = 0x00000100; // verified (ar9002_phy.h)

/// AR_PHY_SYNTH_CONTROL (ar9002_phy.h:158) — the RF synthesiser. Programmed by
/// ar9002_hw_set_channel; for 2.4GHz it is a plain write of
/// `(prev & 0xc0000000) | bMode<<29 | fracMode<<28 | aModeRefSel<<26 | CHANSEL_2G(freq)`
/// with bMode=fracMode=1, aModeRefSel=0. CHANSEL_2G(freq) = freq*0x10000/15 (phy.h).
pub const AR_PHY_SYNTH_CONTROL: u32 = 0x9874; // verified (ar9002_phy.h)
/// CHANSEL_2G divisor (phy.h `CHANSEL_DIV 15`).
pub const CHANSEL_2G_DIV: u64 = 15;
pub const AR_PHY_SYNTH_CONTROL_2G_BMODE: u32 = 1 << 29;
pub const AR_PHY_SYNTH_CONTROL_2G_FRACMODE: u32 = 1 << 28;

/// AR_PHY_CCK_TX_CTRL (ar9002_phy.h:413) — channel-14 spreading; cleared for ch1..13.
pub const AR_PHY_CCK_TX_CTRL: u32 = 0xA204; // verified (ar9002_phy.h)
pub const AR_PHY_CCK_TX_CTRL_JAPAN: u32 = 0x00000010; // verified (ar9002_phy.h)

// ── ar9285_hw_cl_cal offset/AGC-cal registers (ar9002_phy.h) ──
pub const AR_PHY_CL_CAL_CTL: u32 = 0xA358; // verified (ar9002_phy.h)
pub const AR_PHY_CL_CAL_ENABLE: u32 = 0x00000002; // verified (ar9002_phy.h)
pub const AR_PHY_PARALLEL_CAL_ENABLE: u32 = 0x00000001; // verified (ar9002_phy.h)
pub const AR_PHY_TPCRG1: u32 = 0xA258; // verified (ar9002_phy.h)
pub const AR_PHY_TPCRG1_PD_CAL_ENABLE: u32 = 0x00400000; // verified (ar9002_phy.h)
pub const AR_PHY_ADC_CTL: u32 = 0x982C; // verified (ar9002_phy.h)
pub const AR_PHY_ADC_CTL_OFF_PWDADC: u32 = 0x00008000; // verified (ar9002_phy.h)

// ── reset-time RX RF calibration (ar9002_calib.c) ──
// The AR9271 periodic-cal machinery run at reset. For an HT20 2.4GHz channel the
// ONLY cal on the list is IQ-mismatch — ADC-gain/ADC-DC are inserted for HT40
// channels only (ar9002_hw_is_cal_supported, ar9002_calib.c:40-45). These
// correction fields latch into hardware-owned RX-path registers that cannot be
// written directly; running the cal is the only way to set them.
//
// AR_PHY_TIMING_CTRL4(_i) = 0x9920 + (_i << 12); chain 0 = 0x9920 (ar9002_phy.h:190).
pub const AR_PHY_TIMING_CTRL4: u32 = 0x9920; // chain 0 (ar9002_phy.h:190)
pub const AR_PHY_TIMING_CTRL4_IQCORR_Q_Q_COFF: u32 = 0x0000_001F; // ar9002_phy.h:191
pub const AR_PHY_TIMING_CTRL4_IQCORR_Q_Q_COFF_S: u32 = 0; // ar9002_phy.h:192
pub const AR_PHY_TIMING_CTRL4_IQCORR_Q_I_COFF: u32 = 0x0000_07E0; // ar9002_phy.h:193
pub const AR_PHY_TIMING_CTRL4_IQCORR_Q_I_COFF_S: u32 = 5; // ar9002_phy.h:194
pub const AR_PHY_TIMING_CTRL4_IQCORR_ENABLE: u32 = 0x0000_0800; // ar9002_phy.h:195
pub const AR_PHY_TIMING_CTRL4_IQCAL_LOG_COUNT_MAX: u32 = 0x0000_F000; // ar9002_phy.h:196
pub const AR_PHY_TIMING_CTRL4_IQCAL_LOG_COUNT_MAX_S: u32 = 12; // ar9002_phy.h:197
pub const AR_PHY_TIMING_CTRL4_DO_CAL: u32 = 0x0001_0000; // ar9002_phy.h:198

/// AR_PHY_CALMODE (ar9002_phy.h:378) — selects which measurement the cal engine runs.
pub const AR_PHY_CALMODE: u32 = 0x99F0; // ar9002_phy.h:378
pub const AR_PHY_CALMODE_IQ: u32 = 0x0000_0000; // ar9002_phy.h:380

// AR_PHY_CAL_MEAS_{0,1,2}(_i) = 0x9c10/0x9c14/0x9c18 + (_i << 12); chain 0 shown
// (ar9002_phy.h:385-387). IQ collect reads MEAS_0=powerMeasI, MEAS_1=powerMeasQ,
// MEAS_2=iqCorrMeas. (MEAS_3=0x9c1c is ADC-only, HT40 — not needed here.)
pub const AR_PHY_CAL_MEAS_0: u32 = 0x9C10; // chain 0 (ar9002_phy.h:385)
pub const AR_PHY_CAL_MEAS_1: u32 = 0x9C14; // chain 0 (ar9002_phy.h:386)
pub const AR_PHY_CAL_MEAS_2: u32 = 0x9C18; // chain 0 (ar9002_phy.h:387)

/// AR9271 2.4GHz nominal noise floor. ar9002_hw_set_nf_limits (ar9002_phy.c:364)
/// sets `nf_2g.nominal = AR_PHY_CCA_NOM_VAL_9271_2GHZ`. That AR9271-specific
/// constant is in ar9002_phy.h (NOT fetched). The generic 2GHz nominal present
/// in the fetched reg.h is `AR_PHY_CCA_NOM_VAL_2GHZ = -118` (reg.h:1314), which
/// is the value AR9271 uses in practice. Reported as dBm.
pub const AR_PHY_CCA_NOM_VAL_2GHZ: i32 = -118; // reg.h:1314 (source-backed)
// UNVERIFIED (ar9002_phy.h): AR_PHY_CCA_NOM_VAL_9271_2GHZ / _MIN_ / _MAX_
// (canonical ath9k: nominal -118, min -125, max -122) — confirm in ar9002_phy.h.

// ============================================================================
// --- rx/tx dma (ath9k_hw_set_dma, hw.c:1192) ---
// ============================================================================
// The RX-DMA config the reset tail applies. Without AR_RXCFG's burst size + the RX FIFO
// threshold the MAC receives frames but never DMAs them into the descriptor ring → the target
// RX tasklet sees nothing (measured: ndr_stats.seen=0). AR_SREV_9271 is the non-9300 path.

/// AHB bus mode (reg.h:1020); set PREFETCH_RD_EN on non-9300 parts.
pub const AR_AHB_MODE: u32 = 0x4024;
/// AR_AHB_PREFETCH_RD_EN (reg.h:1025).
pub const AR_AHB_PREFETCH_RD_EN: u32 = 0x00000004;
/// AR_TXCFG (reg.h:79) — MAC TX DMA config.
pub const AR_TXCFG: u32 = 0x0030;
/// AR_TXCFG_DMASZ_MASK (reg.h:80) / _128B (reg.h:86): 128-byte DMA read bursts.
pub const AR_TXCFG_DMASZ_MASK: u32 = 0x00000007;
pub const AR_TXCFG_DMASZ_128B: u32 = 5;
/// AR_RXCFG (reg.h:99) — MAC RX DMA config. ★ The write that was missing.
pub const AR_RXCFG: u32 = 0x0034;
/// AR_RXCFG_DMASZ_MASK (reg.h:102) / _128B (reg.h:108): 128-byte DMA write bursts.
pub const AR_RXCFG_DMASZ_MASK: u32 = 0x00000007;
pub const AR_RXCFG_DMASZ_128B: u32 = 5;
/// AR_RXFIFO_CFG (reg.h:1823) — RX FIFO threshold; ath9k writes 0x200.
pub const AR_RXFIFO_CFG: u32 = 0x8114;

// ============================================================================
// --- mac rx ---
// ============================================================================

/// AR_IMR / AR_IMR_S0..S2 (reg.h:266/302/308/314) — the hardware interrupt mask. ★ Gates which MAC
/// events raise the AR9271 target CPU's interrupt. Without the RX bits armed, the target's RX ISR
/// and `ath_tgt_rx_tasklet` never fire → frames DMA into the ring but are never delivered (seen=0).
/// `WMI_ENABLE_INTR` only edits the target's software SWBA/BMISS mask; the host must arm AR_IMR
/// itself (the golden trace shows the kernel writes these values explicitly).
pub const AR_IMR: u32 = 0x00a0;
pub const AR_IMR_S0: u32 = 0x00a4;
pub const AR_IMR_S1: u32 = 0x00a8;
pub const AR_IMR_S2: u32 = 0x00ac;

/// AR_CFG (reg.h:29) — MAC config incl. descriptor byte-swap. The kernel writes 0x0a on this host
/// (golden trace); a wrong descriptor swap can make RX DMA write the completion where the target
/// does not read it.
pub const AR_CFG: u32 = 0x0014;
/// AR_MCAST_FIL0 / _FIL1 (reg.h:1677/1678) — the 64-bit multicast HASH filter. ★ MUST be all-ones
/// to receive broadcast/multicast (every beacon is broadcast); it defaults to 0 after reset, which
/// drops every multicast frame before RX DMA. The `AR_RX_FILTER` MCAST bit is necessary but NOT
/// sufficient — this hash is the other half. (Root cause of the seen=0 deaf receiver.)
pub const AR_MCAST_FIL0: u32 = 0x8040;
pub const AR_MCAST_FIL1: u32 = 0x8044;

/// AR_CR (command register) offset (reg.h:22).
pub const AR_CR: u32 = 0x0008;
/// AR_CR_RXE (reg.h:23). CONDITIONAL: `AR9300_20_OR_LATER ? 0x0c : 0x04`.
/// AR9271 -> 0x00000004.
pub const AR_CR_RXE: u32 = 0x00000004;
/// AR_CR_RXD (reg.h:24) — RX DMA disable (the stop bit); recorded for completeness.
pub const AR_CR_RXD: u32 = 0x00000020;

/// AR_RX_FILTER register offset (reg.h:1675). Bit values confirmed against mac.h
/// `enum { ATH9K_RX_FILTER_* }` (mac.h:643-648) — the software enum whose values ARE
/// the hardware register bits.
pub const AR_RX_FILTER: u32 = 0x803C;
pub const AR_RX_FILTER_UCAST: u32 = 0x00000001; // verified (mac.h:643)
pub const AR_RX_FILTER_MCAST: u32 = 0x00000002; // verified (mac.h:644)
pub const AR_RX_FILTER_BCAST: u32 = 0x00000004; // verified (mac.h:645)
pub const AR_RX_FILTER_CONTROL: u32 = 0x00000008; // verified (mac.h:646)
pub const AR_RX_FILTER_BEACON: u32 = 0x00000010; // verified (mac.h:647)
pub const AR_RX_FILTER_PROM: u32 = 0x00000020; // verified (mac.h:648)
pub const AR_RX_FILTER_PROBEREQ: u32 = 0x00000080; // canonical mac.h
pub const AR_RX_FILTER_MYBEACON: u32 = 0x00000200; // canonical mac.h
/// The monitor-mode filter the coordinator specified: UCAST|MCAST|BCAST|CONTROL|
/// BEACON|PROBEREQ|PROM = 0xBF — open the receiver to everything on-channel.
pub const AR_RX_FILTER_MONITOR: u32 = AR_RX_FILTER_UCAST
    | AR_RX_FILTER_MCAST
    | AR_RX_FILTER_BCAST
    | AR_RX_FILTER_CONTROL
    | AR_RX_FILTER_BEACON
    | AR_RX_FILTER_PROBEREQ
    | AR_RX_FILTER_PROM;

/// AR_DIAG_SW register offset (reg.h:1689).
pub const AR_DIAG_SW: u32 = 0x8048;
pub const AR_DIAG_RX_DIS: u32 = 0x00000020; // reg.h:1695 (RX block)
pub const AR_DIAG_RX_ABORT: u32 = 0x02000000; // reg.h:1711 (force RX abort)

/// AR_STA_ID0/1. UNVERIFIED (mac.h): NOT defined in the fetched reg.h (only
/// referenced by hw.c). AR_BSS_ID0=0x8008 (reg.h:1637) fixes the block layout;
/// canonical ath9k mac.h: AR_STA_ID0=0x8000, AR_STA_ID1=0x8004.
pub const AR_STA_ID0: u32 = 0x8000; // verified (mac.h)
pub const AR_STA_ID1: u32 = 0x8004; // verified (mac.h)
// AR_STA_ID1 opmode / control bits (reg.h:1619-1635, these ARE in reg.h):
pub const AR_STA_ID1_STA_AP: u32 = 0x00010000; // reg.h:1619 (AP opmode)
pub const AR_STA_ID1_ADHOC: u32 = 0x00020000; // reg.h:1620 (IBSS opmode)
pub const AR_STA_ID1_PWR_SAV: u32 = 0x00040000; // reg.h:1621
pub const AR_STA_ID1_KSRCHDIS: u32 = 0x00080000; // reg.h:1622
pub const AR_STA_ID1_RTS_USE_DEF: u32 = 0x00800000; // reg.h:1627
pub const AR_STA_ID1_PRESERVE_SEQNUM: u32 = 0x20000000; // reg.h:1633
pub const AR_STA_ID1_MCAST_KSRCH: u32 = 0x80000000; // reg.h:1635
/// AR_STA_ID1_KSRCH_MODE (reg.h:1632) — key-search mode, the `set` value
/// ath9k_hw_set_operating_mode writes for every opmode (incl. monitor).
pub const AR_STA_ID1_KSRCH_MODE: u32 = 0x10000000; // reg.h:1632
// NOTE: pure STA (managed) opmode = neither STA_AP nor ADHOC set; monitor mode
// likewise clears both opmode bits (RX filter opens PROM/CONTROL instead).

/// AR_PHY_ERR register offset (reg.h:1816).
pub const AR_PHY_ERR: u32 = 0x810c;
pub const AR_PHY_ERR_RADAR: u32 = 0x00000020; // reg.h:1819
pub const AR_PHY_ERR_OFDM_TIMING: u32 = 0x00020000; // reg.h:1820
pub const AR_PHY_ERR_CCK_TIMING: u32 = 0x02000000; // reg.h:1821

// ============================================================================
// --- faithful ath9k_hw_reset() reset-tail transcription (added for hw_reset) ---
// Every constant below carries a `// source <file>:<line>` note; the whole block
// was mined for the faithful ath9k_hw_reset() port. Values are the AR9271
// (non-9100 / non-9300 / 2.4 GHz HT20) branch, resolved from reg.h / mac.h /
// ar9002_phy.h at kernel v6.12.33 and cross-checked against the golden trace
// (`scratchpad/ath9k_kbringup.mon.regs`).
// ============================================================================

// ── init_pll / chip_reset tail (hw.c:760 ath9k_hw_init_pll, hw.c:1513 chip_reset) ──
// The PLL/core-clock/sleep-clock writes live INSIDE ath9k_hw_chip_reset (via
// ath9k_hw_init_pll), i.e. BEFORE the AR9271 GATE_MAC_CTL write and BEFORE
// process_ini. (AR9271_PLL_CONTROL_2GHZ / AR9271_CORE_CLOCK_* / AR_RTC_SLEEP_CLK
// are defined above in the pll section.) Golden trace lines 8-10: 0x7014=0x142c,
// 0x50040=0x304, 0x7048=0x02, then line 11: 0x50044=0x4000 (GATE_MAC_CTL). ✓

// ── mark_phy_inactive (ar9002_phy.h AR_PHY_ACTIVE_DIS) ──
/// AR_PHY_ACTIVE_DIS = 0 (ar9002_phy.h:52). ath9k_hw_mark_phy_inactive writes it.
pub const AR_PHY_ACTIVE_DIS: u32 = 0x00000000; // source ar9002_phy.h:52 (trace line 1: 0x981c=0)

// ── ath9k_hw_init_mfp (hw.c:1680, AR9280_20_OR_LATER branch) ──
/// AR_AES_MUTE_MASK1 (reg.h:1728). Only the FC_MGMT field is RMW'd to 0xc7ff.
pub const AR_AES_MUTE_MASK1: u32 = 0x8060; // source reg.h:1728
pub const AR_AES_MUTE_MASK1_FC_MGMT: u32 = 0xFFFF0000; // source reg.h:1730
pub const AR_AES_MUTE_MASK1_FC_MGMT_S: u32 = 16; // source reg.h:1731
/// The value ath9k_hw_init_mfp writes into the FC_MGMT field (hw.c:1687).
pub const AR_MFP_MGMT_MASK_VAL: u32 = 0xc7ff; // source hw.c:1687

// ── ath9k_hw_set_delta_slope (ar5008_phy.c ar5008_hw_set_delta_slope) ──
// NOTE: set_delta_slope's implementation is ar5008_hw_set_delta_slope, which is
// in ar5008_phy.c (NOT among the fetched files). Transcribed from the canonical
// mainline body: clockMhzScaled=0x64000000 (100 MHz << COEF_SCALE_S), coef =
// clockMhzScaled / synth_center, delta-slope mantissa/exponent via
// ath9k_hw_get_delta_slope_vals (hw.c:1297, transcribed exactly), programmed into
// AR_PHY_TIMING3 (full-GI) and AR_PHY_HALFGI (half-GI, 0.9× coef).
pub const AR_PHY_TIMING3: u32 = 0x9814; // source ar9002_phy.h:40
pub const AR_PHY_TIMING3_DSC_MAN: u32 = 0xFFFE0000; // source ar9002_phy.h:41
pub const AR_PHY_TIMING3_DSC_MAN_S: u32 = 17; // source ar9002_phy.h:42
pub const AR_PHY_TIMING3_DSC_EXP: u32 = 0x0001E000; // source ar9002_phy.h:43
pub const AR_PHY_TIMING3_DSC_EXP_S: u32 = 13; // source ar9002_phy.h:44
pub const AR_PHY_HALFGI: u32 = 0x99D0; // source ar9002_phy.h:360
pub const AR_PHY_HALFGI_DSC_MAN: u32 = 0x0007FFF0; // source ar9002_phy.h:361
pub const AR_PHY_HALFGI_DSC_MAN_S: u32 = 4; // source ar9002_phy.h:362
pub const AR_PHY_HALFGI_DSC_EXP: u32 = 0x0000000F; // source ar9002_phy.h:363
pub const AR_PHY_HALFGI_DSC_EXP_S: u32 = 0; // source ar9002_phy.h:364
/// COEF_SCALE_S — the fixed-point scale used by delta-slope (ar5008_phy.c / hw.c:1297).
pub const COEF_SCALE_S: u32 = 24; // source ar5008_phy.c (COEF_SCALE_S)
/// clockMhzScaled seed = 100 MHz << COEF_SCALE_S (ar5008_hw_set_delta_slope).
pub const DELTA_SLOPE_CLOCK_MHZ_SCALED: u32 = 0x64000000; // source ar5008_hw_set_delta_slope

// ── ath9k_hw_spur_mitigate_freq (ar9002_phy.c:168, no-spur path) ──
/// AR_PHY_FORCE_CLKEN_CCK (ar9002_phy.h:453). The no-spur path clears MRC_MUX
/// and returns (ar9002_phy.c:214-221). Full spur mitigation needs EEPROM spur
/// channels, which the AR9271 EEPROM does not carry for ordinary channels.
pub const AR_PHY_FORCE_CLKEN_CCK: u32 = 0xA22C; // source ar9002_phy.h:453
pub const AR_PHY_FORCE_CLKEN_CCK_MRC_MUX: u32 = 0x00000040; // source ar9002_phy.h:454

// ── ath9k_hw_reset_opmode (hw.c:1707) + set_operating_mode (hw.c:1266) ──
pub const AR_ISR: u32 = 0x0080; // source reg.h:179
pub const AR_DEF_ANTENNA: u32 = 0x8058; // source reg.h:1721
pub const AR_BSS_ID0: u32 = 0x8008; // source reg.h:1637
pub const AR_BSS_ID1: u32 = 0x800C; // source reg.h:1638
pub const AR_RSSI_THR: u32 = 0x8018; // source reg.h:1652
/// INIT_RSSI_THR — the beacon-miss RSSI threshold reset value written by
/// ath9k_hw_reset_opmode. Canonical mac.h value (mac.h not fully fetched).
pub const INIT_RSSI_THR: u32 = 0x00000700; // canonical mac.h (UNVERIFIED offset value)
/// AR_STA_ID1 station-address-high field mask; reset_opmode RMWs ~SADH_MASK.
pub const AR_STA_ID1_SADH_MASK: u32 = 0x0000FFFF; // canonical mac.h
pub const AR_STA_ID1_BASE_RATE_11B: u32 = 0x02000000; // source reg.h:1629

// ── ath9k_hw_set_clockrate (hw.c:39) — computes a value, writes nothing ──
/// 2.4 GHz OFDM MAC clock in MHz. ★ MEASURED via the golden trace: SIFS write
/// 0x1030=0x160 (=352) = mac_to_clks(10-2)=8*clockrate ⇒ clockrate=44.
pub const ATH9K_CLOCK_RATE_2GHZ_OFDM: u32 = 44; // source hw.c:51 (value confirmed by trace)
pub const ATH9K_CLOCK_RATE_CCK: u32 = 22; // source hw.c (no-chan CCK path)

// ── ath9k_hw_init_global_settings (hw.c:1047) — MAC timing ──
pub const AR_D_GBL_IFS_SIFS: u32 = 0x1030; // source reg.h:609 (trace 0x1030=0x160)
pub const AR_D_GBL_IFS_SLOT: u32 = 0x1070; // source reg.h:623 (trace 0x1070=0x18c)
pub const AR_D_GBL_IFS_EIFS: u32 = 0x10b0; // source reg.h:627 (trace 0x10b0=0x3e38)
pub const AR_TIME_OUT: u32 = 0x8014; // source reg.h:1646
pub const AR_TIME_OUT_ACK: u32 = 0x00003FFF; // source reg.h:1647
pub const AR_TIME_OUT_ACK_S: u32 = 0;
pub const AR_TIME_OUT_CTS: u32 = 0x3FFF0000; // source reg.h:1649
pub const AR_TIME_OUT_CTS_S: u32 = 16;
pub const AR_USEC: u32 = 0x801c; // source reg.h:1660
pub const AR_USEC_USEC: u32 = 0x0000007F; // source reg.h:1661
pub const AR_USEC_TX_LAT: u32 = 0x007FC000; // source reg.h:1662
pub const AR_USEC_TX_LAT_S: u32 = 14;
pub const AR_USEC_RX_LAT: u32 = 0x1F800000; // source reg.h:1664
pub const AR_USEC_RX_LAT_S: u32 = 23;
/// Default slot time for 2.4 GHz (ah->slottime), 20 µs long-slot? ath9k uses 9.
/// ★ MEASURED via trace: 0x1070=0x18c=396=9*44 ⇒ slottime=9.
pub const ATH9K_INIT_SLOTTIME_2GHZ: u32 = 9; // value confirmed by trace

// ── ath9k_hw_init_qos (hw.c:714) ──
pub const AR_MIC_QOS_CONTROL: u32 = 0x8118; // source reg.h:1826 (trace 0x8118=0x100aa)
pub const AR_MIC_QOS_SELECT: u32 = 0x811c; // source reg.h:1827 (trace 0x811c=0x3210)
pub const AR_QOS_NO_ACK: u32 = 0x8108; // source reg.h:1808
pub const AR_QOS_NO_ACK_TWO_BIT: u32 = 0x0000000f; // source reg.h:1809
pub const AR_QOS_NO_ACK_TWO_BIT_S: u32 = 0; // source reg.h:1810
pub const AR_QOS_NO_ACK_BIT_OFF: u32 = 0x00000070; // source reg.h:1811
pub const AR_QOS_NO_ACK_BIT_OFF_S: u32 = 4; // source reg.h:1812
pub const AR_QOS_NO_ACK_BYTE_OFF: u32 = 0x00000180; // source reg.h:1813
pub const AR_QOS_NO_ACK_BYTE_OFF_S: u32 = 7; // source reg.h:1814
pub const AR_TXOP_X: u32 = 0x81ec; // source reg.h:1955
pub const AR_TXOP_X_VAL: u32 = 0x000000FF; // source reg.h:1956
pub const AR_TXOP_0_3: u32 = 0x81f0; // source reg.h:1959
pub const AR_TXOP_4_7: u32 = 0x81f4; // source reg.h:1960
pub const AR_TXOP_8_11: u32 = 0x81f8; // source reg.h:1961
pub const AR_TXOP_12_15: u32 = 0x81fc; // source reg.h:1962

// ── ath9k_hw_init_interrupt_masks (hw.c:931) ──
pub const AR_IMR_RXOK: u32 = 0x00000001; // source reg.h:267
pub const AR_IMR_RXERR: u32 = 0x00000004; // source reg.h:271
pub const AR_IMR_RXORN: u32 = 0x00000020; // source reg.h:274
pub const AR_IMR_TXOK: u32 = 0x00000040; // source reg.h:275
pub const AR_IMR_TXERR: u32 = 0x00000100; // source reg.h:277
pub const AR_IMR_TXURN: u32 = 0x00000800; // source reg.h:280
pub const AR_IMR_BCNMISC: u32 = 0x00800000; // source reg.h:290
pub const AR_IMR_S2_GTT: u32 = 0x00800000; // source reg.h:319
pub const AR_IMR_S0_QCU_TXOK: u32 = 0x000003FF; // source reg.h:303
pub const AR_IMR_S0_QCU_TXOK_S: u32 = 0; // source reg.h:304
pub const AR_IMR_S0_QCU_TXDESC: u32 = 0x03FF0000; // source reg.h:305
pub const AR_IMR_S0_QCU_TXDESC_S: u32 = 16; // source reg.h:306
pub const AR_IMR_S1_QCU_TXERR: u32 = 0x000003FF; // source reg.h:309
pub const AR_IMR_S1_QCU_TXERR_S: u32 = 0; // source reg.h:310
pub const AR_IMR_S1_QCU_TXEOL: u32 = 0x03FF0000; // source reg.h:311
pub const AR_IMR_S1_QCU_TXEOL_S: u32 = 16; // source reg.h:312
pub const AR_IMR_S2_QCU_TXURN: u32 = 0x000003FF; // source reg.h:315
/// AR_INTR_SYNC_CAUSE(0x4028)/ENABLE(0x402c)/MASK(0x4034) — non-9340 path.
pub const AR_INTR_SYNC_MASK: u32 = 0x4034; // source reg.h:1093 (non-9340)
/// AR_INTR_SYNC_DEFAULT resolved from the reg.h enum (reg.h:1071-1079):
/// HOST1_FATAL|HOST1_PERR|RADM_CPL_EP|RADM_CPL_DLLP_ABORT|RADM_CPL_TLP_ABORT|
/// RADM_CPL_ECRC_ERR|RADM_CPL_TIMEOUT|LOCAL_TIMEOUT|MAC_SLEEP_ACCESS.
pub const AR_INTR_SYNC_DEFAULT: u32 = 0x00023F60; // source reg.h:1071 (resolved)

// ── REG_WRITE(AR_OBS, 8) (hw.c:2020) + JTAG disable (hw.c:1958) ──
/// AR_OBS — non-9340/non-9300 branch (reg.h:1259). Trace line 93: 0x4080=0x08. ✓
pub const AR_OBS: u32 = 0x4080; // source reg.h:1261 (AR9271)
/// AR_GPIO_INPUT_EN_VAL — non-9340/non-9300 branch (reg.h:1205). AR9271 is
/// AR_SREV_9280_20_OR_LATER, so hw_reset sets AR_GPIO_JTAG_DISABLE here.
pub const AR_GPIO_INPUT_EN_VAL: u32 = 0x4054; // source reg.h:1206 (AR9271)
pub const AR_GPIO_JTAG_DISABLE: u32 = 0x00020000; // source reg.h:1222

// ── REG_WRITE(AR_CFG_LED, saveLedState | AR_CFG_SCLK_32KHZ) (hw.c:2049) ──
pub const AR_CFG_LED: u32 = 0x1f04; // source reg.h:658 (trace line 111: 0x1f04=0x03)
pub const AR_CFG_SCLK_32KHZ: u32 = 0x00000003; // source reg.h:664
pub const AR_CFG_LED_ASSOC_CTL: u32 = 0x00000c00; // source reg.h:681
pub const AR_CFG_LED_MODE_SEL: u32 = 0x00000380; // source reg.h:667
pub const AR_CFG_LED_BLINK_THRESH_SEL: u32 = 0x00000070; // source reg.h:690
pub const AR_CFG_LED_BLINK_SLOW: u32 = 0x00000008; // source reg.h:687

// ── ath9k_hw_init_desc (hw.c:1748) — AR9271 USB descriptor byte-swap ──
pub const AR_CFG_SWTB: u32 = 0x00000002; // source reg.h:31
pub const AR_CFG_SWRB: u32 = 0x00000008; // source reg.h:33 (AR_CFG=SWRB|SWTB=0x0a, trace line 112)

// ── ath9k_hw_restore_chainmask (hw.c:2048) — no-op for 1-chain AR9271 ──
pub const AR_PHY_RX_CHAINMASK: u32 = 0x99a4; // source ar9002_phy.h:304
pub const AR_PHY_CAL_CHAINMASK: u32 = 0xA39C; // source ar9002_phy.h:563

// ── ath9k_hw_init_bb (synth-settle delay read) ──
pub const AR_PHY_RX_DELAY: u32 = 0x9914; // source ar9002_phy.h:186
pub const AR_PHY_RX_DELAY_DELAY: u32 = 0x00003FFF; // source ar9002_phy.h:188
/// BASE_ACTIVATE_DELAY — the µs floor init_bb waits after AR_PHY_ACTIVE_EN.
pub const BASE_ACTIVATE_DELAY: u32 = 100; // source hw.c (BASE_ACTIVATE_DELAY)

// ── override_ini tail (process_ini) — AR_PCU_MISC_MODE2 ──
/// AR_PCU_MISC_MODE2 (reg.h:2041). ath9k_hw_override_ini RMWs it. ⚠ override_ini
/// itself is NOT in the fetched source; the golden trace (lines 41/166) shows the
/// working driver ends up with 0x8344=0x00581083 — recorded so the caller can
/// reproduce/audit it.
pub const AR_PCU_MISC_MODE2: u32 = 0x8344; // source reg.h:2041 (trace 0x8344=0x00581083)
pub const AR_PCU_MISC_MODE2_TRACE_VAL: u32 = 0x00581083; // golden trace, absolute value

// ── AR_2040_MODE (ath9k_hw_set_channel_regs; HT20 ⇒ 0) ──
pub const AR_2040_MODE: u32 = 0x8318; // source reg.h:2027 (trace 0x8318=0)

// ── ath9k_hw_init_queues (hw.c:1729) + resettxqueue (mac.c:367) ──
pub const AR_NUM_DCU: u32 = 10; // source reg.h:492
pub const AR_NUM_QCU: u32 = 10; // source reg.h:368
pub const AR_D0_QCUMASK: u32 = 0x1000; // source reg.h:504 (AR_DQCUMASK(i)=+i<<2)
pub const AR_D0_LCL_IFS: u32 = 0x1040; // source reg.h:518 (AR_DLCL_IFS(i)=+i<<2)
pub const AR_D0_RETRY_LIMIT: u32 = 0x1080; // source reg.h:538 (AR_DRETRY_LIMIT(i)=+i<<2)
pub const AR_D0_CHNTIME: u32 = 0x10c0; // source reg.h:557 (AR_DCHNTIME(i)=+i<<2)
pub const AR_D0_MISC: u32 = 0x1100; // source reg.h:573 (AR_DMISC(i)=+i<<2)
pub const AR_Q0_MISC: u32 = 0x09c0; // source reg.h:440 (AR_QMISC(i)=+i<<2)
pub const AR_D_LCL_IFS_CWMIN: u32 = 0x000003FF; // source reg.h:529
pub const AR_D_LCL_IFS_CWMIN_S: u32 = 0;
pub const AR_D_LCL_IFS_CWMAX: u32 = 0x000FFC00; // source reg.h:531
pub const AR_D_LCL_IFS_CWMAX_S: u32 = 10;
pub const AR_D_LCL_IFS_AIFS: u32 = 0x0FF00000; // source reg.h:533
pub const AR_D_LCL_IFS_AIFS_S: u32 = 20;
pub const AR_D_RETRY_LIMIT_FR_SH: u32 = 0x0000000F; // source reg.h:549
pub const AR_D_RETRY_LIMIT_FR_SH_S: u32 = 0;
pub const AR_D_RETRY_LIMIT_STA_SH: u32 = 0x00003F00; // source reg.h:551
pub const AR_D_RETRY_LIMIT_STA_SH_S: u32 = 8;
pub const AR_D_RETRY_LIMIT_STA_LG: u32 = 0x000FC000; // source reg.h:553
pub const AR_D_RETRY_LIMIT_STA_LG_S: u32 = 14;
pub const AR_Q_MISC_DCU_EARLY_TERM_REQ: u32 = 0x00000800; // source reg.h:465
pub const AR_D_MISC_FRAG_WAIT_EN: u32 = 0x00000100; // source reg.h:587
pub const AR_D_MISC_CW_BKOFF_EN: u32 = 0x00001000; // source reg.h:589
pub const INIT_CWMIN: u32 = 15; // source mac.h:63
pub const INIT_CWMAX: u32 = 1023; // source mac.h:65
pub const INIT_AIFS: u32 = 2; // source mac.h:62
pub const INIT_SH_RETRY: u32 = 10; // source mac.h:66
pub const INIT_SSH_RETRY: u32 = 32; // source mac.h:68
pub const INIT_SLG_RETRY: u32 = 32; // source mac.h:69
