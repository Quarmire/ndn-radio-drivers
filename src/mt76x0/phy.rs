//! MT7610U (`mt76x0`) **RF/PHY programming** — the code that turns "the chip is
//! powered and the MAC is initialised" into "the radio is tuned to a channel and
//! radiating at a known power".
//!
//! Ported from the mainline Linux mt76 driver, read-only reference tree under
//! `scratchpad/mt76-src/`: `mt76x0/phy.c` (1215 lines), `mt76x0/phy.h`,
//! `mt76x0/eeprom.c`, `mt76x02_phy.c`, `mt76x02_eeprom.c`. Every non-obvious
//! constant or sequence below carries its upstream `file:line`.
//!
//! ## MEASURED vs CODE-READ
//! Almost everything here is **CODE-READ**. The MT7610U RF is undocumented; the
//! only description of it that exists is `initvals_phy.h` plus the order in which
//! `phy.c` replays it. Nothing in this file has been confirmed on air yet.
//!
//! The handful of facts that *are* **MEASURED** on mds-o5p-1's MT7610U
//! (`examples/mt76_oracle.rs`, 2026-08-27) and that constrain this code:
//!   * An EP0 vendor-request round trip is **151 µs**
//!     ([`measured::EP0_ROUND_TRIP_US`](crate::mt76::regs::measured::EP0_ROUND_TRIP_US)).
//!     That is the unit of cost for *every* function here. A full
//!     [`set_channel`] is a few hundred register accesses — tens of milliseconds
//!     — which is fine for a channel switch and ruinous on any per-frame path.
//!     It also makes upstream's `mt76_poll(..., 100)` (poll every 10 µs for
//!     100 µs) meaningless over USB: one read already overruns the budget. See
//!     [`poll`].
//!   * `MT_BBP(AGC, 2)` reads `0x003a6464` and `MT_EXT_CCA_CFG` reads
//!     `0x0000f1e4` under the kernel monitor, i.e. the BBP is alive at the
//!     addresses this file writes.
//!
//! ## Two ways to reach an RF register, and why both are here
//! The RF banks are *not* MMIO. Upstream offers two transports and picks between
//! them **on bus type, not on capability** (`mt76x0/phy.c:105`, `:124`):
//!   * `mt76_is_usb()` → the **MCU register-pair path**: hand `{reg, value}`
//!     pairs to the firmware with base [`MT_MCU_MEMMAP_RF`] and let it do the
//!     CSR poking. Requires running firmware (`MT76_STATE_MCU_RUNNING` is
//!     `WARN_ON_ONCE`-asserted at `phy.c:111`).
//!   * MMIO → the **direct CSR path** through [`MT_RF_CSR_CFG`] (`0x0500`), a
//!     kick/poll handshake (`mt76x0_rf_csr_wr`, `phy.c:20`).
//!
//! Whether the CSR path *also* works over USB on this part is an **open question
//! we intend to measure**, and it decides the risk profile of live retuning: if
//! it works, a channel change is pure MMIO and survives a wedged MCU; if it does
//! not, every retune is a firmware round trip and a firmware crash costs us the
//! radio. So both are implemented here — [`rf_csr_wr`] / [`rf_csr_rr`] and
//! [`rf_mcu_wr`] — the active one is reported by [`PhyBus::rf_path`], and the
//! default is [`RfPath::Mcu`], the one upstream actually uses on USB. Do not
//! flip the default on a hunch; flip it on a measurement.
//!
//! ## What is deliberately *not* here
//!   * **`mt76x0_phy_vco_cal`** does not exist upstream. VCO calibration on this
//!     part is two things: the firmware command `MCU_CAL_VCO` with the channel
//!     number (`phy.c:872`, `phy.c:1041`) and the `vcocal_en` bit `MT_RF(0, 4)<7>`
//!     (`phy.c:1204`, `phy.c:1006`). Both are ported; there is no third thing.
//!   * **RXDCOC / LOFT / TXIQ** are likewise not separate host sequences. They
//!     are firmware calibrations behind `CMD_CALIBRATION_OP`; `MCU_CAL_FULL`
//!     (`0xff`) runs the ladder and `MCU_CAL_RXDCOC` is issued on its own
//!     afterwards (`phy.c:903-909`). See [`calibrate`].
//!   * **DFS AGC adjustment** (`mt76x02_phy_dfs_adjust_agc`, called from
//!     `phy.c:1064` on radar channels) is not ported: a monitor-mode named-radio
//!     face does not run the DFS state machine, and porting half of it would be
//!     worse than not having it.
//!   * **`is_mt7630` / `is_mt7610e`** branches are resolved at port time to their
//!     MT7610U (USB) values and the dead arm dropped, with the upstream line
//!     cited at each site. This is a USB MT7610U driver.
#![allow(dead_code)]

use std::thread::sleep;
use std::time::Duration;

use ndn_radio_hal::Bandwidth;

use crate::FaceError;
use crate::mt76::Mt76Regs;
use crate::mt76::regs::{
    self, MT_BBP_AGC_BASE, MT_BBP_AGC_GAIN, MT_BBP_AGC_R0_BW, MT_BBP_AGC_R0_CTRL_CHAN,
    MT_BBP_CORE_BASE, MT_BBP_CORE_R1_BW, MT_BBP_IBI_BASE, MT_BBP_TXBE_BASE,
    MT_BBP_TXBE_R0_CTRL_CHAN, MT_CMB_CTRL, MT_COEXCFG0, MT_COEXCFG3, MT_CSR_EE_CFG1,
    MT_EXT_CCA_CFG, MT_EXT_CCA_CFG_CCA_MASK, MT_EXT_CCA_CFG_CCA0, MT_EXT_CCA_CFG_CCA1,
    MT_EXT_CCA_CFG_CCA2, MT_EXT_CCA_CFG_CCA3, MT_MAC_SYS_CTRL, MT_MAC_SYS_CTRL_ENABLE_RX,
    MT_MAC_SYS_CTRL_ENABLE_TX, MT_MCU_MEMMAP_RF, MT_RF_BYPASS_0, MT_RF_CSR_CFG, MT_RF_CSR_CFG_DATA,
    MT_RF_CSR_CFG_KICK, MT_RF_CSR_CFG_REG_BANK, MT_RF_CSR_CFG_REG_ID, MT_RF_CSR_CFG_WR, MT_RF_MISC,
    MT_RF_PA_MODE_CFG0, MT_RF_PA_MODE_CFG1, MT_RF_SDM_BP_MASK, MT_RF_SDM_MASH_PRBS_MASK,
    MT_RF_SDM_RESET_MASK, MT_RF_SETTING_0, MT_RX_STAT_1, MT_RX_STAT_1_CCA_ERRORS, MT_TX_ALC_CFG_0,
    MT_TX_ALC_CFG_0_CH_INIT_0, MT_TX_ALC_CFG_0_CH_INIT_1, MT_TX_ALC_CFG_0_LIMIT_0,
    MT_TX_ALC_CFG_0_LIMIT_1, MT_TX_ALC_CFG_1, MT_TX_ALC_CFG_1_TEMP_COMP, MT_TX_ALC_VGA3,
    MT_TX_BAND_CFG, MT_TX_BAND_CFG_2G, MT_TX_BAND_CFG_5G, MT_TX_BAND_CFG_UPPER_40M,
    MT_TX_PWR_CFG_0, MT_TX_PWR_CFG_1, MT_TX_PWR_CFG_2, MT_TX_PWR_CFG_3, MT_TX_PWR_CFG_4,
    MT_TX_PWR_CFG_7, MT_TX_PWR_CFG_8, MT_TX_PWR_CFG_9, MT_TX0_RF_GAIN_ATTEN, MT_TX0_RF_GAIN_CORR,
    MT_WLAN_FUN_CTRL, field_get, field_prep, mt_bbp, mt_rf, mt_rf_bank, mt_rf_reg,
};

use super::eeprom::Mt76x0Eeprom;
use super::freq_plan::{self, FreqItem};
use super::initvals::BBP_SWITCH_TAB;
use super::initvals_phy::{
    RF_2G_CHANNEL_0_TAB, RF_5G_CHANNEL_0_TAB, RF_A_BAND, RF_BAND_SWITCH_TAB, RF_BW_20, RF_BW_40,
    RF_BW_80, RF_BW_SWITCH_TAB, RF_CENTRAL_TAB, RF_EXT_PA_TAB, RF_G_BAND, RF_VGA_CHANNEL_0_TAB,
};
use super::mcu::RegPair;

// ── Small local plumbing ────────────────────────────────────────────────────

/// Build the crate's error type. Every failure in this module is an I/O failure
/// against the dongle, so they all funnel through here rather than each site
/// re-spelling `FaceError::Io(io::Error::other(...))`.
fn phy_err(msg: String) -> FaceError {
    FaceError::Io(std::io::Error::other(msg))
}

/// Upstream's `usleep_range(lo, hi)`. We sleep the low bound: the caller's real
/// cost is the surrounding USB traffic, not this.
fn usleep(us: u64) {
    sleep(Duration::from_micros(us));
}

// ── BBP addresses used below, resolved once ─────────────────────────────────
// `MT_BBP(unit, n)` = base + (n << 2) (`mt76x02_regs.h:619`). Spelled out here so
// the call sites read like upstream's `MT_BBP(CORE, 34)` instead of a hex soup.

/// `MT_BBP(CORE, 0)` — the BBP version register polled by [`wait_bbp_ready`].
const BBP_CORE_0: u32 = mt_bbp(MT_BBP_CORE_BASE, 0);
/// `MT_BBP(CORE, 1)` — bandwidth field + the ch14 Japan TX filter bit.
const BBP_CORE_1: u32 = mt_bbp(MT_BBP_CORE_BASE, 1);
/// `MT_BBP(CORE, 4)` — bit 0 is the BBP software reset.
const BBP_CORE_4: u32 = mt_bbp(MT_BBP_CORE_BASE, 4);
/// `MT_BBP(CORE, 34)` — the TSSI/temperature measurement request register.
const BBP_CORE_34: u32 = mt_bbp(MT_BBP_CORE_BASE, 34);
/// `MT_BBP(CORE, 35)` — the result of a [`BBP_CORE_34`] measurement.
const BBP_CORE_35: u32 = mt_bbp(MT_BBP_CORE_BASE, 35);
/// `MT_BBP(IBI, 9)` — saved/forced around the firmware calibration ladder.
const BBP_IBI_9: u32 = mt_bbp(MT_BBP_IBI_BASE, 9);
/// `MT_BBP(AGC, 0)` — RX path select, AGC bandwidth, primary-channel slot.
const BBP_AGC_0: u32 = mt_bbp(MT_BBP_AGC_BASE, 0);
/// `MT_BBP(AGC, 8)` — the live VGA gain for chain 0. The RX-sensitivity knob.
const BBP_AGC_8: u32 = mt_bbp(MT_BBP_AGC_BASE, 8);
/// `MT_BBP(AGC, 9)` — the same for chain 1 (snapshotted but unused on a 1×1).
const BBP_AGC_9: u32 = mt_bbp(MT_BBP_AGC_BASE, 9);
/// `MT_BBP(TXBE, 0)` — TX-side copy of the primary-channel slot.
const BBP_TXBE_0: u32 = mt_bbp(MT_BBP_TXBE_BASE, 0);
/// `MT_BBP(TXBE, 4)` — bits 1:0 encode a TX digital-gain step read by the TSSI
/// loop (`mt76x0/phy.c:761`).
const BBP_TXBE_4: u32 = mt_bbp(MT_BBP_TXBE_BASE, 4);
/// `MT_BBP(TXBE, 5)` — bits 1:0 select the TX DAC count.
const BBP_TXBE_5: u32 = mt_bbp(MT_BBP_TXBE_BASE, 5);
/// `MT_BBP(TXBE, 6)` — bit 31 forces TX from DAC0, used by the TSSI DC cal.
const BBP_TXBE_6: u32 = mt_bbp(MT_BBP_TXBE_BASE, 6);

// ── MCU commands / calibration ids ──────────────────────────────────────────

/// `CMD_FUN_SET_OP` — `mt76x02_mcu.h:31`.
pub const CMD_FUN_SET_OP: u8 = 1;
/// `CMD_CALIBRATION_OP` — `mt76x02_mcu.h:49`.
pub const CMD_CALIBRATION_OP: u8 = 31;

/// `enum mcu_function` — `mt76x02_mcu.h:62`. Only `BW_SETTING` is used here.
/// ⚠ Upstream's enum collides on purpose: `BW_SETTING = 2` and
/// `USB2_SW_DISCONNECT = 2` are the same number.
pub const FUNC_Q_SELECT: u32 = 1;
/// See [`FUNC_Q_SELECT`]. Selects the baseband bandwidth on the USB path.
pub const FUNC_BW_SETTING: u32 = 2;

/// `enum mcu_calibrate` — **`mt76x0/mcu.h:22`**.
///
/// ⚠ These ids are *not* the mt76x2 ones. `mt76x2/mcu.h:27` defines a different
/// enum with the same `MCU_CAL_` prefix in which `MCU_CAL_RXDCOC = 3` and
/// `MCU_CAL_LC = 6`. Using the x2 numbering on a 7610 would ask the firmware for
/// the wrong calibration and it would not necessarily complain.
pub mod mcu_cal {
    /// Resistor calibration. Power-on only.
    pub const R: u32 = 1;
    /// RX DC-offset cancellation.
    pub const RXDCOC: u32 = 2;
    /// Local-oscillator / VCO loop calibration.
    pub const LC: u32 = 3;
    /// TX LO feed-through.
    pub const LOFT: u32 = 4;
    /// TX IQ imbalance.
    pub const TXIQ: u32 = 5;
    /// Bandwidth calibration.
    pub const BW: u32 = 6;
    /// Digital pre-distortion.
    pub const DPD: u32 = 7;
    /// RX IQ imbalance.
    pub const RXIQ: u32 = 8;
    /// TX DC-offset cancellation.
    pub const TXDCOC: u32 = 9;
    /// RX group delay.
    pub const RX_GROUP_DELAY: u32 = 10;
    /// TX group delay.
    pub const TX_GROUP_DELAY: u32 = 11;
    /// VCO retune for a channel. Argument is the channel number.
    pub const VCO: u32 = 12;
    /// "No signal" variant.
    pub const NO_SIGNAL: u32 = 0xfe;
    /// Run the whole ladder. Argument encodes the band/sub-band.
    pub const FULL: u32 = 0xff;
}

// ── EEPROM word addresses (mt76x02_eeprom.h:12-95) ──────────────────────────
// phy.rs reads the EEPROM **image** through [`PhyBus::eeprom_word`] rather than
// through the parsed [`Mt76x0Eeprom`], so that this file stays independent of
// how `eeprom.rs` chooses to model the part. Each address is upstream's.

/// Antenna configuration word. Bit 15 is [`EE_ANTENNA_DUAL`].
const EE_ANTENNA: u16 = 0x022;
/// Copied verbatim into [`MT_CSR_EE_CFG1`] by [`ant_select`].
const EE_CFG1_INIT: u16 = 0x024;
/// RX/TX path counts, internal-PA flags, board type.
const EE_NIC_CONF_0: u16 = 0x034;
/// Bit 13 (`TX_ALC_EN`) is the TSSI-closed-loop enable.
const EE_NIC_CONF_1: u16 = 0x036;
/// Crystal frequency offset (`xo_cxo`), written to `MT_RF(0, 22)`.
const EE_FREQ_OFFSET: u16 = 0x03a;
/// Antenna-diversity option bits.
const EE_NIC_CONF_2: u16 = 0x042;
/// 2.4 GHz LNA gain (low byte) + 5 GHz group-0 LNA gain (high byte).
const EE_LNA_GAIN: u16 = 0x044;
/// Per-chain RSSI offsets, 2.4 GHz.
const EE_RSSI_OFFSET_2G_0: u16 = 0x046;
/// High byte carries the 5 GHz group-1 LNA gain.
const EE_RSSI_OFFSET_2G_1: u16 = 0x048;
/// Per-chain RSSI offsets, 5 GHz.
const EE_RSSI_OFFSET_5G_0: u16 = 0x04a;
/// High byte carries the 5 GHz group-2 LNA gain.
const EE_RSSI_OFFSET_5G_1: u16 = 0x04c;
/// 40 MHz power delta: low byte 2.4 GHz, high byte 5 GHz.
const EE_TX_POWER_DELTA_BW40: u16 = 0x050;
/// Base of the 2.4 GHz per-channel target-power byte array. Also the 80 MHz
/// delta address upstream reuses (`mt76x0/eeprom.c:243`).
const EE_TX_POWER_DELTA_BW80: u16 = 0x052;
/// 2.4 GHz TSSI slope (low byte) / offset (high byte).
const EE_TSSI_SLOPE_2G: u16 = 0x06e;
/// Base of the 5 GHz per-channel target-power array (`+2 + offset`).
const EE_TX_POWER_0_GRP4_TSSI_SLOPE: u16 = 0x076;
/// 2.4 GHz TSSI target power (low byte); high byte is the temperature offset.
const EE_2G_TARGET_POWER: u16 = 0x0d0;
/// 5 GHz TSSI target power (low byte); high byte is the 80 MHz power delta.
const EE_5G_TARGET_POWER: u16 = 0x0d2;
/// First of seven 5 GHz TSSI channel bounds, read as a byte array.
const EE_TSSI_BOUND1: u16 = 0x0d4;
/// High byte is a signed correction to [`EE_FREQ_OFFSET`].
const EE_TSSI_BOUND4: u16 = 0x0da;
/// Base of the per-rate power table (CCK first).
const EE_TX_POWER_BYRATE_BASE: u16 = 0x0de;
/// First of eight 5 GHz TSSI slope/offset words, indexed by the bound search.
const EE_TSSI_SLOPE_5G: u16 = 0x0f0;

/// 5 GHz per-rate power lives at raw addresses upstream spells as bare integers
/// (`mt76x0/eeprom.c:168,174,180,186,192`) with no name in
/// `enum mt76x02_eeprom_field`. Reproduced as literals for that reason.
const EE_5G_BYRATE_OFDM_6M: u16 = 0x120;
/// See [`EE_5G_BYRATE_OFDM_6M`].
const EE_5G_BYRATE_OFDM_24M: u16 = 0x122;
/// See [`EE_5G_BYRATE_OFDM_6M`].
const EE_5G_BYRATE_MCS0: u16 = 0x124;
/// See [`EE_5G_BYRATE_OFDM_6M`].
const EE_5G_BYRATE_MCS4: u16 = 0x126;
/// See [`EE_5G_BYRATE_OFDM_6M`]. VHT MCS 8/9.
const EE_5G_BYRATE_VHT_MCS8: u16 = 0x12c;

/// `MT_EE_ANTENNA_DUAL` — bit 15 of [`EE_ANTENNA`] (`mt76x02_eeprom.h:98`).
const EE_ANTENNA_DUAL: u16 = 1 << 15;
/// `MT_EE_NIC_CONF_0_PA_INT_2G` (`mt76x02_eeprom.h:103`).
const EE_NIC_CONF_0_PA_INT_2G: u16 = 1 << 8;
/// `MT_EE_NIC_CONF_0_PA_INT_5G` (`mt76x02_eeprom.h:104`).
const EE_NIC_CONF_0_PA_INT_5G: u16 = 1 << 9;
/// `MT_EE_NIC_CONF_0_BOARD_TYPE` (`mt76x02_eeprom.h:106`).
const EE_NIC_CONF_0_BOARD_TYPE: u16 = 0x3000;
/// `MT_EE_NIC_CONF_1_TX_ALC_EN` (`mt76x02_eeprom.h:112`) — the TSSI enable.
const EE_NIC_CONF_1_TX_ALC_EN: u16 = 1 << 13;
/// `MT_EE_NIC_CONF_2_ANT_OPT` (`mt76x02_eeprom.h:114`).
const EE_NIC_CONF_2_ANT_OPT: u16 = 1 << 3;
/// `MT_EE_NIC_CONF_2_ANT_DIV` (`mt76x02_eeprom.h:115`).
const EE_NIC_CONF_2_ANT_DIV: u16 = 1 << 4;

// ── The bus seam ────────────────────────────────────────────────────────────

/// Which transport an RF-bank access takes.
///
/// The distinction is not cosmetic: [`RfPath::Mcu`] makes every RF write a
/// firmware round trip, so a wedged MCU costs the radio its ability to retune,
/// while [`RfPath::Csr`] is plain MMIO and would survive that. Upstream chooses
/// on bus type alone (`mt76x0/phy.c:105`), never having asked whether the CSR
/// path works over USB. We keep the question open and answerable.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Default)]
pub enum RfPath {
    /// MCU register-pair protocol with base [`MT_MCU_MEMMAP_RF`]. What upstream
    /// uses on USB, and the default here. Requires running firmware.
    #[default]
    Mcu,
    /// Direct [`MT_RF_CSR_CFG`] kick/poll. **Opt-in and unmeasured on USB.**
    Csr,
}

/// What [`phy`](self) needs from a backend.
///
/// The four methods the port contract fixes ([`rf_wr`](Self::rf_wr),
/// [`rf_rr`](Self::rf_rr), [`mcu_wr_rp`](Self::mcu_wr_rp),
/// [`eeprom`](Self::eeprom)) are required. The three that follow are **added by
/// this file** and all carry defaults, so a backend written to the bare contract
/// still compiles — but a backend that does not override
/// [`mcu_send`](Self::mcu_send) and [`eeprom_word`](Self::eeprom_word) has no
/// calibration and no EEPROM-derived power, and will say so at runtime rather
/// than radiate something wrong.
pub trait PhyBus: Mt76Regs {
    /// Write one 8-bit RF-bank register, `bank_reg` being `MT_RF(bank, reg)`.
    ///
    /// The backend implements whichever transport it reports from
    /// [`rf_path`](Self::rf_path). Everything in this module goes through the
    /// dispatcher [`rf_wr`] rather than calling this directly.
    fn rf_wr(&self, bank_reg: u32, val: u8) -> Result<(), FaceError>;

    /// Read one 8-bit RF-bank register.
    ///
    /// On [`RfPath::Mcu`] this needs the MCU *read* register-pair command, which
    /// the contract does not put on this trait — hence it stays the backend's
    /// job. On [`RfPath::Csr`] this file can do it itself ([`rf_csr_rr`]).
    fn rf_rr(&self, bank_reg: u32) -> Result<u8, FaceError>;

    /// Send `{base + reg, value}` pairs through the MCU random-write command.
    fn mcu_wr_rp(&self, base: u32, pairs: &[RegPair]) -> Result<(), FaceError>;

    /// The parsed EEPROM. Retained because the contract fixes it and because a
    /// backend must own one anyway; note that this file reads EEPROM *words*
    /// through [`eeprom_word`](Self::eeprom_word) instead, so that the PHY math
    /// stays a line-for-line mirror of upstream's `mt76x02_eeprom_get(dev, ADDR)`
    /// calls and does not couple to `eeprom.rs`'s parse shape.
    fn eeprom(&self) -> &Mt76x0Eeprom;

    // ── added by phy.rs, all defaulted ──────────────────────────────────────

    /// Which RF transport [`rf_wr`](Self::rf_wr) / [`rf_rr`](Self::rf_rr) use,
    /// and therefore which one this module drives. Default [`RfPath::Mcu`].
    fn rf_path(&self) -> RfPath {
        RfPath::Mcu
    }

    /// Send one MCU command (`cmd` from `enum mcu_cmd`, `mt76x02_mcu.h:30`),
    /// optionally waiting for the response.
    ///
    /// Required by [`calibrate`] and by the USB bandwidth select
    /// ([`bbp_set_bw`]); there is no register-level substitute for either. The
    /// default fails loudly instead of silently skipping calibration, because a
    /// radio that never calibrated still transmits — badly, and without telling
    /// anyone.
    fn mcu_send(&self, _cmd: u8, _data: &[u8], _wait_resp: bool) -> Result<(), FaceError> {
        Err(phy_err(
            "mt76x0::phy: backend does not implement PhyBus::mcu_send; \
             calibration and USB bandwidth select are unavailable"
                .into(),
        ))
    }

    /// One little-endian 16-bit word of the EEPROM image at byte offset `addr`.
    ///
    /// Mirrors `mt76x02_eeprom_get` (`mt76x02_eeprom.h:162`), **including** its
    /// rejection of odd addresses — upstream returns `-1` there, which every
    /// caller consumes as `0xffff`. The default returns `0xffff` for everything,
    /// which every `mt76x02_field_valid` check below treats as "absent", so an
    /// unwired backend degrades to neutral defaults rather than to garbage.
    fn eeprom_word(&self, _addr: u16) -> u16 {
        0xffff
    }
}

/// Namespace for the PHY helpers, per the port contract. Stateless: every
/// operation takes `&dyn PhyBus` and any persistent tracking lives in the
/// caller's [`CalState`] / [`AgcState`].
pub struct Mt76x0Phy;

impl Mt76x0Phy {
    /// See [`init_rf`].
    pub fn init_rf(bus: &dyn PhyBus) -> Result<(), FaceError> {
        init_rf(bus)
    }
    /// See [`set_channel`].
    pub fn set_channel(bus: &dyn PhyBus, channel: u8, bw: Bandwidth) -> Result<(), FaceError> {
        set_channel(bus, channel, bw)
    }
    /// See [`set_tx_power`].
    pub fn set_tx_power(
        bus: &dyn PhyBus,
        channel: u8,
        target_dbm: Option<i8>,
        index: Option<u8>,
    ) -> Result<i8, FaceError> {
        set_tx_power(bus, channel, target_dbm, index)
    }
    /// See [`calibrate`].
    pub fn calibrate(bus: &dyn PhyBus, channel: u8, full: bool) -> Result<(), FaceError> {
        calibrate(bus, channel, full)
    }
}

// ── EEPROM field decoding (mt76x02_eeprom.h inlines) ────────────────────────

/// `mt76x02_eeprom_get` (`mt76x02_eeprom.h:162`). Odd addresses are rejected
/// upstream with `-1`; we return the same bit pattern.
fn ee(bus: &dyn PhyBus, addr: u16) -> u16 {
    if addr & 1 != 0 {
        return 0xffff;
    }
    bus.eeprom_word(addr)
}

/// One EEPROM *byte*, for the arrays upstream reads with `mt76x02_eeprom_copy`
/// (e.g. the seven TSSI bounds at [`EE_TSSI_BOUND1`], `mt76x0/phy.c:711`).
fn ee_byte(bus: &dyn PhyBus, addr: u16) -> u8 {
    let word = ee(bus, addr & !1);
    if addr & 1 == 0 {
        (word & 0xff) as u8
    } else {
        (word >> 8) as u8
    }
}

/// `mt76x02_field_valid` (`mt76x02_eeprom.h:131`): an EEPROM byte carries data
/// only if it is neither all-zero nor all-ones.
const fn field_valid(val: u8) -> bool {
    val != 0 && val != 0xff
}

/// `mt76x02_sign_extend` (`mt76x02_eeprom.h:136`).
///
/// ⚠ This is **not** two's-complement sign extension, despite the name. The top
/// bit is a *positive* flag and the rest is a magnitude: `sign ? +val : -val`.
/// Ported exactly; getting this "right" would silently invert every EEPROM
/// correction on the part.
const fn sign_extend(val: u32, size: u32) -> i32 {
    let sign = val & (1 << (size - 1)) != 0;
    let mag = (val & ((1 << (size - 1)) - 1)) as i32;
    if sign { mag } else { -mag }
}

/// `mt76x02_sign_extend_optional` (`mt76x02_eeprom.h:146`): bit `size` enables
/// the field; without it the correction is zero.
const fn sign_extend_optional(val: u32, size: u32) -> i32 {
    if val & (1 << size) != 0 {
        sign_extend(val, size)
    } else {
        0
    }
}

/// `mt76x02_rate_power_val` (`mt76x02_eeprom.h:154`). Bit 7 enables, bit 6 is
/// the positive flag, bits 5:0 are a magnitude in **0.5 dB** steps.
const fn rate_power_val(val: u8) -> i8 {
    if !field_valid(val) {
        return 0;
    }
    sign_extend_optional(val as u32, 7) as i8
}

/// `s6_to_s8` (`mt76x0/eeprom.h:26`): sign-extend a 6-bit two's-complement
/// per-rate power delta. Unlike [`sign_extend`] this one *is* two's complement.
const fn s6_to_s8(val: u32) -> i8 {
    let ret = (val & 0x3f) as i8;
    if ret & 0x20 != 0 { ret - 0x40 } else { ret }
}

/// Is the closed-loop TSSI (automatic level control) enabled on this board?
/// `mt76x0_tssi_enabled`, `mt76x0/eeprom.h:35`.
pub fn tssi_enabled(bus: &dyn PhyBus) -> bool {
    ee(bus, EE_NIC_CONF_1) & EE_NIC_CONF_1_TX_ALC_EN != 0
}

/// Does the board have an **external** PA for this band?
/// `mt76x02_ext_pa_enabled`, `mt76x02_eeprom.c:91`. Note the inversion: the
/// EEPROM bit means "PA is *internal*".
pub fn ext_pa_enabled(bus: &dyn PhyBus, is_5ghz: bool) -> bool {
    let conf0 = ee(bus, EE_NIC_CONF_0);
    if is_5ghz {
        conf0 & EE_NIC_CONF_0_PA_INT_5G == 0
    } else {
        conf0 & EE_NIC_CONF_0_PA_INT_2G == 0
    }
}

/// `(has_2ghz, has_5ghz)` from the EEPROM board type
/// (`mt76x02_eeprom_parse_hw_cap`, `mt76x02_eeprom.c:72`). `BOARD_TYPE_2GHZ = 1`,
/// `BOARD_TYPE_5GHZ = 2`, anything else means dual-band.
pub fn hw_cap(bus: &dyn PhyBus) -> (bool, bool) {
    let val = ee(bus, EE_NIC_CONF_0);
    match (val & EE_NIC_CONF_0_BOARD_TYPE) >> 12 {
        2 => (false, true),
        1 => (true, false),
        _ => (true, true),
    }
}

// ── RF register access ──────────────────────────────────────────────────────

/// Poll `addr` until `val & mask == want`, giving up after `tries` reads.
///
/// Upstream is `mt76_poll(dev, reg, mask, val, timeout_us)`, which reads every
/// 10 µs for `timeout_us` (`phy.c:37` passes 100 µs). That budget is meaningless
/// here: one EP0 read is **151 µs MEASURED**, so upstream's 100 µs timeout is
/// already exceeded by the first read. We therefore count *reads*, not
/// microseconds, and default to 10 — about 1.5 ms of wall clock, which is the
/// same order as the register access itself and far more generous than upstream.
pub fn poll<R: Mt76Regs + ?Sized>(
    regs: &R,
    addr: u32,
    mask: u32,
    want: u32,
    tries: u32,
) -> Result<bool, FaceError> {
    for _ in 0..tries.max(1) {
        if regs.rr(addr)? & mask == want {
            return Ok(true);
        }
    }
    Ok(false)
}

/// Write an RF-bank register over the **direct CSR path**
/// (`mt76x0_rf_csr_wr`, `mt76x0/phy.c:20`).
///
/// Opt-in: nothing in this module calls it unless [`PhyBus::rf_path`] reports
/// [`RfPath::Csr`]. Whether it works at all on the USB part is the open question
/// this function exists to let us answer — it needs only MMIO, so it can be
/// driven from a probe program against a bare [`Mt76Regs`].
pub fn rf_csr_wr<R: Mt76Regs + ?Sized>(regs: &R, bank_reg: u32, val: u8) -> Result<(), FaceError> {
    let bank = mt_rf_bank(bank_reg);
    let reg = mt_rf_reg(bank_reg);
    // phy.c:32. ⚠ Upstream permits bank == 8, but MT_RF_CSR_CFG_REG_BANK is
    // GENMASK(17, 15) — three bits — so an 8 would not fit the field. No table
    // in this port uses a bank above 7; the check is ported as written.
    if reg > 127 || bank > 8 {
        return Err(phy_err(format!(
            "mt76x0 RF CSR: bad address bank={bank} reg={reg} (from {bank_reg:#010x})"
        )));
    }
    if !poll(regs, MT_RF_CSR_CFG, MT_RF_CSR_CFG_KICK, 0, 10)? {
        return Err(phy_err(format!(
            "mt76x0 RF CSR write {bank}:{reg}: KICK never cleared"
        )));
    }
    regs.wr(
        MT_RF_CSR_CFG,
        field_prep(MT_RF_CSR_CFG_DATA, val as u32)
            | field_prep(MT_RF_CSR_CFG_REG_BANK, bank)
            | field_prep(MT_RF_CSR_CFG_REG_ID, reg)
            | MT_RF_CSR_CFG_WR
            | MT_RF_CSR_CFG_KICK,
    )
}

/// Read an RF-bank register over the direct CSR path
/// (`mt76x0_rf_csr_rr`, `mt76x0/phy.c:59`).
///
/// Three EP0 round trips minimum (poll, kick, poll) plus the result read — call
/// it ~600 µs. The bank/reg readback check is upstream's: the register latches
/// the address it actually serviced, so a mismatch means the kick raced.
pub fn rf_csr_rr<R: Mt76Regs + ?Sized>(regs: &R, bank_reg: u32) -> Result<u8, FaceError> {
    let bank = mt_rf_bank(bank_reg);
    let reg = mt_rf_reg(bank_reg);
    if reg > 127 || bank > 8 {
        return Err(phy_err(format!(
            "mt76x0 RF CSR: bad address bank={bank} reg={reg} (from {bank_reg:#010x})"
        )));
    }
    if !poll(regs, MT_RF_CSR_CFG, MT_RF_CSR_CFG_KICK, 0, 10)? {
        return Err(phy_err(format!(
            "mt76x0 RF CSR read {bank}:{reg}: KICK never cleared before kick"
        )));
    }
    regs.wr(
        MT_RF_CSR_CFG,
        field_prep(MT_RF_CSR_CFG_REG_BANK, bank)
            | field_prep(MT_RF_CSR_CFG_REG_ID, reg)
            | MT_RF_CSR_CFG_KICK,
    )?;
    if !poll(regs, MT_RF_CSR_CFG, MT_RF_CSR_CFG_KICK, 0, 10)? {
        return Err(phy_err(format!(
            "mt76x0 RF CSR read {bank}:{reg}: KICK never cleared after kick"
        )));
    }
    let val = regs.rr(MT_RF_CSR_CFG)?;
    if field_get(MT_RF_CSR_CFG_REG_ID, val) == reg && field_get(MT_RF_CSR_CFG_REG_BANK, val) == bank
    {
        Ok(field_get(MT_RF_CSR_CFG_DATA, val) as u8)
    } else {
        Err(phy_err(format!(
            "mt76x0 RF CSR read {bank}:{reg}: readback addressed {:#010x}",
            val
        )))
    }
}

/// Write an RF-bank register over the **MCU register-pair path**
/// (`mt76x0_rf_wr`, `mt76x0/phy.c:105`). Firmware must be running.
pub fn rf_mcu_wr(bus: &dyn PhyBus, bank_reg: u32, val: u8) -> Result<(), FaceError> {
    bus.mcu_wr_rp(
        MT_MCU_MEMMAP_RF,
        &[RegPair {
            reg: bank_reg,
            value: val as u32,
        }],
    )
}

/// Write one RF register through whichever path [`PhyBus::rf_path`] reports.
///
/// Everything in this module writes RF through here so that flipping the path is
/// one decision in one place.
pub fn rf_wr(bus: &dyn PhyBus, bank_reg: u32, val: u8) -> Result<(), FaceError> {
    match bus.rf_path() {
        RfPath::Csr => rf_csr_wr(bus, bank_reg, val),
        RfPath::Mcu => bus.rf_wr(bank_reg, val),
    }
}

/// Read one RF register through the active path.
pub fn rf_rr(bus: &dyn PhyBus, bank_reg: u32) -> Result<u8, FaceError> {
    match bus.rf_path() {
        RfPath::Csr => rf_csr_rr(bus, bank_reg),
        RfPath::Mcu => bus.rf_rr(bank_reg),
    }
}

/// `mt76x0_rf_rmw` (`mt76x0/phy.c:140`). Returns the value written.
///
/// ⚠ Upstream's argument order reads as `(mask, val)` but the body is
/// `val |= ret & ~mask` — i.e. `val` is **already positioned**, not a field value
/// to be shifted. Ported with the same contract.
pub fn rf_rmw(bus: &dyn PhyBus, bank_reg: u32, mask: u8, val: u8) -> Result<u8, FaceError> {
    let old = rf_rr(bus, bank_reg)?;
    let new = val | (old & !mask);
    rf_wr(bus, bank_reg, new)?;
    Ok(new)
}

/// `mt76x0_rf_set` (`mt76x0/phy.c:155`) — OR bits in, mask zero.
pub fn rf_set(bus: &dyn PhyBus, bank_reg: u32, val: u8) -> Result<u8, FaceError> {
    rf_rmw(bus, bank_reg, 0, val)
}

/// `mt76x0_rf_clear` (`mt76x0/phy.c:161`) — clear the bits in `mask`.
pub fn rf_clear(bus: &dyn PhyBus, bank_reg: u32, mask: u8) -> Result<u8, FaceError> {
    rf_rmw(bus, bank_reg, mask, 0)
}

/// `RF_RANDOM_WRITE` (`mt76x0/phy.c:178`) — apply a whole `(MT_RF(b,r), value)`
/// table.
///
/// On [`RfPath::Mcu`] this is **one** MCU command carrying every pair, which is
/// the entire reason upstream has the macro: [`RF_CENTRAL_TAB`] alone is 44 rows,
/// and at 151 µs per EP0 write a row-at-a-time apply would cost ~6.6 ms for that
/// table and ~30 ms for a full [`init_rf`]. On [`RfPath::Csr`] there is no batch
/// form and it degrades to one write per row (`mt76x0_phy_rf_csr_wr_rp`,
/// `phy.c:167`).
pub fn rf_wr_table(bus: &dyn PhyBus, tab: &[(u32, u32)]) -> Result<(), FaceError> {
    match bus.rf_path() {
        RfPath::Csr => {
            for &(reg, val) in tab {
                rf_csr_wr(bus, reg, val as u8)?;
            }
            Ok(())
        }
        RfPath::Mcu => {
            let pairs: Vec<RegPair> = tab
                .iter()
                .map(|&(reg, value)| RegPair { reg, value })
                .collect();
            bus.mcu_wr_rp(MT_MCU_MEMMAP_RF, &pairs)
        }
    }
}

// ── MCU helpers ─────────────────────────────────────────────────────────────

/// `mt76x02_mcu_function_select` (`mt76x02_mcu.c:82`): two little-endian words,
/// `{id, value}`, under `CMD_FUN_SET_OP`. Everything except `Q_SELECT` waits for
/// the response.
pub fn mcu_function_select(bus: &dyn PhyBus, func: u32, val: u32) -> Result<(), FaceError> {
    let mut msg = [0u8; 8];
    msg[0..4].copy_from_slice(&func.to_le_bytes());
    msg[4..8].copy_from_slice(&val.to_le_bytes());
    bus.mcu_send(CMD_FUN_SET_OP, &msg, func != FUNC_Q_SELECT)
}

/// `mt76x02_mcu_calibrate` (`mt76x02_mcu.c:117`): `{type, param}` under
/// `CMD_CALIBRATION_OP`, always waiting for the response.
///
/// The `MT_MCU_COM_REG0` handshake around it is `is_mt76x2e`-only
/// (`mt76x02_mcu.c:126,129,137`) and is therefore not ported.
pub fn mcu_calibrate(bus: &dyn PhyBus, cal: u32, param: u32) -> Result<(), FaceError> {
    let mut msg = [0u8; 8];
    msg[0..4].copy_from_slice(&cal.to_le_bytes());
    msg[4..8].copy_from_slice(&param.to_le_bytes());
    bus.mcu_send(CMD_CALIBRATION_OP, &msg, true)
}

// ── Bring-up ────────────────────────────────────────────────────────────────

/// `mt76x0_phy_wait_bbp_ready` (`mt76x0/phy.c:185`). Returns the BBP version.
///
/// The readiness test is `val && ~val` — neither all-zero nor all-ones, i.e. the
/// same "is this a real readback or a dead bus" test the rest of the rig uses.
/// Upstream loops 20 times with no delay between reads; over USB each read is
/// already 151 µs, so 20 tries is ~3 ms.
pub fn wait_bbp_ready(bus: &dyn PhyBus) -> Result<u32, FaceError> {
    for _ in 0..20 {
        let val = bus.rr(BBP_CORE_0)?;
        if val != 0 && val != u32::MAX {
            return Ok(val);
        }
    }
    Err(phy_err(
        "mt76x0: BBP is not ready (MT_BBP(CORE,0) dead)".into(),
    ))
}

/// `mt76x0_rf_patch_reg_array` (`mt76x0/phy.c:1116`), resolved for **USB
/// MT7610U**.
///
/// Upstream rewrites three rows per chip variant before applying a table. On
/// this part the substitutions are:
///   * `MT_RF(0, 3)` → `0x73` — the `!mt76_is_mmio` arm, `phy.c:1133`.
///   * `MT_RF(0, 21)` → `0x12` — the `!is_mt7610e` arm, `phy.c:1140`.
///   * `MT_RF(5, 2)` → `0x0c` — the neither-7630-nor-7610e arm, `phy.c:1148`.
///
/// All three happen to equal what [`RF_CENTRAL_TAB`] / [`RF_2G_CHANNEL_0_TAB`]
/// already hold, so on a 7610U this is a no-op — but it is applied rather than
/// asserted, because "happens to equal" is exactly the kind of claim that stops
/// being true when someone edits a table.
fn patch_reg_array(bus: &dyn PhyBus, tab: &[(u32, u32)]) -> Result<(), FaceError> {
    for &(reg, val) in tab {
        let val = if reg == mt_rf(0, 3) {
            0x73
        } else if reg == mt_rf(0, 21) {
            0x12
        } else if reg == mt_rf(5, 2) {
            0x0c
        } else {
            val as u8
        };
        rf_wr(bus, reg, val)?;
    }
    Ok(())
}

/// `mt76x02_phy_set_rxpath` (`mt76x02_phy.c:12`).
///
/// Bit 3 of `MT_BBP(AGC, 0)` selects two RX chains; bit 4 is cleared
/// unconditionally. The MT7610U is 1×1, so `rx_chains` is 1 and both bits clear.
/// Upstream's trailing read-back (`mt76x02_phy.c:30`) exists only to order the
/// write against an `mb()` on MMIO and is dropped: a USB control transfer is
/// already ordered by completion.
pub fn set_rxpath(bus: &dyn PhyBus, rx_chains: u8) -> Result<(), FaceError> {
    let mut val = bus.rr(BBP_AGC_0)?;
    val &= !(1 << 4);
    if rx_chains == 2 {
        val |= 1 << 3;
    } else {
        val &= !(1 << 3);
    }
    bus.wr(BBP_AGC_0, val)
}

/// `mt76x02_phy_set_txdac` (`mt76x02_phy.c:34`). Two TX chains light both DACs;
/// the 1×1 MT7610U clears the field.
pub fn set_txdac(bus: &dyn PhyBus, tx_chains: u8) -> Result<(), FaceError> {
    if tx_chains == 2 {
        bus.rmw(BBP_TXBE_5, 0, 0x3)?;
    } else {
        bus.rmw(BBP_TXBE_5, 0x3, 0)?;
    }
    Ok(())
}

/// `mt76x0_phy_ant_select` (`mt76x0/phy.c:426`) — antenna / coexistence wiring
/// from the EEPROM.
///
/// Faithful port with the `is_mt7630` arm (`phy.c:462`) dropped. Two details
/// worth not "cleaning up":
///   * `ee_ant` has bits 14 and 12 cleared and is then written **whole** into the
///     low half of [`MT_CMB_CTRL`] — the EEPROM word is the register value.
///   * In the single-antenna 2.4 GHz-only case `coex3 |= BIT(1)` sets a bit that
///     the preceding `coex3 &= ~GENMASK(5, 2)` did *not* clear, so bit 1 is OR'd
///     onto whatever the hardware already had. Upstream does this; we do too.
pub fn ant_select(bus: &dyn PhyBus) -> Result<(), FaceError> {
    let mut ee_ant = ee(bus, EE_ANTENNA);
    let ee_cfg1 = ee(bus, EE_CFG1_INIT);
    let nic_conf2 = ee(bus, EE_NIC_CONF_2);
    let (has_2ghz, has_5ghz) = hw_cap(bus);

    let mut wlan = bus.rr(MT_WLAN_FUN_CTRL)?;
    let mut coex3 = bus.rr(MT_COEXCFG3)?;

    ee_ant &= !((1 << 14) | (1 << 12));
    wlan &= !((1 << 6) | (1 << 5));
    coex3 &= !0x3c; // GENMASK(5, 2)

    if ee_ant & EE_ANTENNA_DUAL != 0 {
        // dual antenna mode — phy.c:441
        let ant_div =
            (nic_conf2 & EE_NIC_CONF_2_ANT_OPT == 0) && (nic_conf2 & EE_NIC_CONF_2_ANT_DIV != 0);
        if ant_div {
            ee_ant |= 1 << 12;
        } else {
            coex3 |= 1 << 4;
        }
        coex3 |= 1 << 3;
        if has_2ghz {
            wlan |= 1 << 6;
        }
    } else {
        // single antenna mode — phy.c:452
        if has_5ghz {
            coex3 |= (1 << 3) | (1 << 4);
        } else {
            wlan |= 1 << 6;
            coex3 |= 1 << 1;
        }
    }

    bus.wr(MT_WLAN_FUN_CTRL, wlan)?;
    bus.rmw(MT_CMB_CTRL, 0xffff, ee_ant as u32)?;
    bus.rmw(MT_CSR_EE_CFG1, 0xffff, ee_cfg1 as u32)?;
    bus.rmw(MT_COEXCFG0, 1 << 2, 0)?;
    bus.wr(MT_COEXCFG3, coex3)
}

/// `mt76x0_phy_rf_init` (`mt76x0/phy.c:1157`) — the one-shot RF bring-up.
///
/// Order is load-bearing: central block, then the 2.4 GHz channel-0 defaults,
/// then the 5 GHz and VGA tables, then the `RF_BW_20` / `RF_G_BAND` slices of the
/// switch tables (so the part comes up in a defined 2.4 GHz / 20 MHz state), then
/// crystal trim, then the DAC reset pulse, then `vcocal_en`.
///
/// The bw-switch filter at init (`phy.c:1171`) is a **third** rule, different
/// from the one [`set_chan_rf_params`] uses: a row matches if its `bw_band` is
/// exactly `RF_BW_20`, or if it contains both `RF_G_BAND` and `RF_BW_20`.
fn rf_init(bus: &dyn PhyBus) -> Result<(), FaceError> {
    patch_reg_array(bus, RF_CENTRAL_TAB)?;
    patch_reg_array(bus, RF_2G_CHANNEL_0_TAB)?;
    rf_wr_table(bus, RF_5G_CHANNEL_0_TAB)?;
    rf_wr_table(bus, RF_VGA_CHANNEL_0_TAB)?;

    for item in RF_BW_SWITCH_TAB {
        let g20 = RF_G_BAND | RF_BW_20;
        if item.bw_band == RF_BW_20 || (g20 & item.bw_band) == g20 {
            rf_wr(bus, item.rf_bank_reg, item.value)?;
        }
    }

    for item in RF_BAND_SWITCH_TAB {
        if item.bw_band & RF_G_BAND != 0 {
            rf_wr(bus, item.rf_bank_reg, item.value)?;
        }
    }

    // Frequency calibration (phy.c:1186):
    //   E1: B0.R22<6:0>: xo_cxo<6:0>
    //   E2: B0.R21<0>: xo_cxo<0>, B0.R22<7:0>: xo_cxo<8:1>
    // Upstream writes only R22 and clamps to 0xbf, then reads it straight back.
    // The read-back result is discarded; UNVERIFIED (phy.c:1192) why it is there
    // — likely a latch/settle requirement. Kept, because it costs one round trip
    // and removing it is exactly the kind of "harmless" edit that breaks silicon.
    let freq_offset = freq_offset(bus);
    rf_wr(bus, mt_rf(0, 22), freq_offset.min(0xbf))?;
    let _ = rf_rr(bus, mt_rf(0, 22))?;

    // "Reset procedure DAC during power-up: set / clear / set B0.R73<7>"
    // (phy.c:1194). The doubled set is upstream's; do not fold it.
    rf_set(bus, mt_rf(0, 73), 1 << 7)?;
    rf_clear(bus, mt_rf(0, 73), 1 << 7)?;
    rf_set(bus, mt_rf(0, 73), 1 << 7)?;

    // vcocal_en: initiate VCO calibration, self-clearing on completion
    // (phy.c:1203).
    rf_set(bus, mt_rf(0, 4), 0x80)?;
    Ok(())
}

/// `mt76x0_phy_init` (`mt76x0/phy.c:1207`) — everything the PHY needs once,
/// after the MAC/BBP tables and before the first [`set_channel`].
///
/// Named `init_rf` by the port contract; it is the whole `mt76x0_phy_init`, not
/// just the RF-table part. The periodic calibration work upstream schedules here
/// (`INIT_DELAYED_WORK`, `phy.c:1209`) is *not* started: see [`AgcState`] and
/// [`calibration_tick`] for the pieces, which this port leaves for the caller to
/// drive on its own clock.
///
/// Hard-codes the 1×1 chain counts of the MT7610U.
pub fn init_rf(bus: &dyn PhyBus) -> Result<(), FaceError> {
    ant_select(bus)?;
    rf_init(bus)?;
    set_rxpath(bus, 1)?;
    set_txdac(bus, 1)?;
    Ok(())
}

// ── EEPROM-derived RX calibration ───────────────────────────────────────────

/// Crystal frequency offset written to `MT_RF(0, 22)`
/// (`mt76x0_set_freq_offset`, `mt76x0/eeprom.c:93`).
///
/// ⚠ `caldata->freq_offset` is `u8` and upstream *subtracts* a
/// [`sign_extend`]-decoded correction from it, so the arithmetic wraps by
/// design. `wrapping_sub` reproduces that.
pub fn freq_offset(bus: &dyn PhyBus) -> u8 {
    let mut val = (ee(bus, EE_FREQ_OFFSET) & 0xff) as u8;
    if !field_valid(val) {
        val = 0;
    }
    let mut corr = (ee(bus, EE_TSSI_BOUND4) >> 8) as u8;
    if !field_valid(corr) {
        corr = 0;
    }
    val.wrapping_sub(sign_extend(corr as u32, 8) as u8)
}

/// Temperature reference for [`temp_sensor`]
/// (`mt76x0_set_temp_offset`, `mt76x0/eeprom.c:82`). Defaults to `-10`.
pub fn temp_offset(bus: &dyn PhyBus) -> i16 {
    let val = (ee(bus, EE_2G_TARGET_POWER) >> 8) as u8;
    if field_valid(val) {
        sign_extend(val as u32, 8) as i16
    } else {
        -10
    }
}

/// The LNA gain and per-chain RSSI offsets for a channel.
///
/// `mt76x0_read_rx_gain` (`mt76x0/eeprom.c:110`) + `mt76x02_get_rx_gain`
/// (`mt76x02_eeprom.c:102`) + `mt76x02_get_lna_gain` (`mt76x02_eeprom.c:130`).
/// Pure EEPROM arithmetic — no register traffic — which is why [`set_channel`]
/// can hoist it to the top instead of calling it mid-sequence as upstream does at
/// `phy.c:1002`.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct RxGain {
    /// LNA gain for the band/sub-band of the channel, in dB.
    pub lna_gain: u8,
    /// Per-chain RSSI offset, clamped to ±10 dB (`mt76x0/eeprom.c:123`).
    pub rssi_offset: [i8; 2],
}

/// See [`RxGain`].
pub fn rx_gain(bus: &dyn PhyBus, channel: u8) -> RxGain {
    let is_5ghz = channel > 14;

    let val = ee(bus, EE_LNA_GAIN);
    let lna_2g = (val & 0xff) as u8;
    let mut lna_5g = [(val >> 8) as u8, 0u8, 0u8];
    lna_5g[1] = (ee(bus, EE_RSSI_OFFSET_2G_1) >> 8) as u8;
    lna_5g[2] = (ee(bus, EE_RSSI_OFFSET_5G_1) >> 8) as u8;
    if !field_valid(lna_5g[1]) {
        lna_5g[1] = lna_5g[0];
    }
    if !field_valid(lna_5g[2]) {
        lna_5g[2] = lna_5g[0];
    }

    let rssi_word = if is_5ghz {
        ee(bus, EE_RSSI_OFFSET_5G_0)
    } else {
        ee(bus, EE_RSSI_OFFSET_2G_0)
    };

    // mt76x02_eeprom.c:136 — the 5 GHz LNA is grouped by channel, not by band.
    let lna = if !is_5ghz {
        lna_2g
    } else if channel <= 64 {
        lna_5g[0]
    } else if channel <= 128 {
        lna_5g[1]
    } else {
        lna_5g[2]
    };

    let mut rssi_offset = [0i8; 2];
    for (i, slot) in rssi_offset.iter_mut().enumerate() {
        let v = (rssi_word >> (8 * i)) as u8 as i8;
        *slot = if !(-10..=10).contains(&v) { 0 } else { v };
    }

    RxGain {
        lna_gain: if lna != 0xff { lna } else { 0 },
        rssi_offset,
    }
}

// ── Per-rate TX power ───────────────────────────────────────────────────────

/// `struct mt76x02_rate_power` (`mt76x02.h:75`) — per-rate TX power in **0.5 dB
/// steps**, as signed deltas from the ALC target in [`MT_TX_ALC_CFG_0`].
///
/// Upstream overlays these arrays with `s8 all[30]` so the offset/limit/max
/// helpers can walk them as one buffer; [`RatePower::iter_mut`] is that view
/// without the union. Field widths are upstream's: `ht` is 16 wide even though a
/// 1×1 part only ever fills 0..8, and `vht` is only 2 (MCS 8 and 9 — MCS 0..7 are
/// read out of the `ht` array by both the EEPROM parse and the TSSI loop).
///
/// ⚠ Duplicates what `eeprom.rs`'s `Mt76x0Eeprom::tx_power_per_rate` will
/// probably produce. It lives here because this file must not depend on a shape
/// it cannot see; if the two agree after integration, delete one.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct RatePower {
    /// CCK 1M, 2M, 5.5M, 11M.
    pub cck: [i8; 4],
    /// OFDM 6M…54M.
    pub ofdm: [i8; 8],
    /// HT/VHT 1SS MCS 0..15 (only 0..7 are meaningful on a 1×1).
    pub ht: [i8; 16],
    /// VHT MCS 8 and 9.
    pub vht: [i8; 2],
}

impl RatePower {
    /// Upstream's `all[30]` view, in declaration order.
    fn iter_mut(&mut self) -> impl Iterator<Item = &mut i8> {
        self.cck
            .iter_mut()
            .chain(self.ofdm.iter_mut())
            .chain(self.ht.iter_mut())
            .chain(self.vht.iter_mut())
    }

    /// Read-only counterpart of [`iter_mut`](Self::iter_mut).
    fn iter(&self) -> impl Iterator<Item = i8> + '_ {
        self.cck
            .iter()
            .chain(self.ofdm.iter())
            .chain(self.ht.iter())
            .chain(self.vht.iter())
            .copied()
    }

    /// `mt76x02_add_rate_power_offset` (`mt76x02_phy.c:84`). Units are 0.5 dB.
    ///
    /// Deviation: upstream's `s8 += int` wraps. We saturate. The two differ only
    /// when `offset` plus a rate delta leaves ±127 half-dB (±63.5 dB), which
    /// takes a corrupt EEPROM; saturating there is strictly the safer answer.
    pub fn add_offset(&mut self, offset: i8) {
        for v in self.iter_mut() {
            *v = v.saturating_add(offset);
        }
    }

    /// `mt76x02_limit_rate_power` (`mt76x02_phy.c:74`). `limit` is 0.5 dB steps.
    pub fn limit(&mut self, limit: i32) {
        for v in self.iter_mut() {
            if (*v as i32) > limit {
                *v = limit.clamp(i8::MIN as i32, i8::MAX as i32) as i8;
            }
        }
    }

    /// `mt76x02_get_max_rate_power` (`mt76x02_phy.c:62`). Note the seed is `0`,
    /// not `i8::MIN`: an all-negative table reports 0, as upstream does.
    pub fn max(&self) -> i8 {
        self.iter().fold(0i8, |a, b| a.max(b))
    }
}

/// `mt76x0_get_delta` (`mt76x0/eeprom.c:130`) — the bandwidth power delta.
fn power_delta(bus: &dyn PhyBus, bw: Bandwidth, is_5ghz: bool) -> i8 {
    let val = match bw {
        Bandwidth::Bw80 => (ee(bus, EE_5G_TARGET_POWER) >> 8) as u8,
        Bandwidth::Bw40 => {
            let data = ee(bus, EE_TX_POWER_DELTA_BW40);
            if is_5ghz {
                (data >> 8) as u8
            } else {
                (data & 0xff) as u8
            }
        }
        _ => return 0,
    };
    rate_power_val(val)
}

/// `mt76x0_get_tx_power_per_rate` (`mt76x0/eeprom.c:152`).
///
/// The 5 GHz addresses (`0x120`…`0x12c`) are bare integers upstream with no
/// symbolic name; see [`EE_5G_BYRATE_OFDM_6M`]. Each EEPROM byte is a 6-bit
/// two's-complement delta in 0.5 dB steps, and each byte covers **two** rates.
pub fn tx_power_per_rate(bus: &dyn PhyBus, channel: u8, bw: Bandwidth) -> RatePower {
    let is_2ghz = channel <= 14;
    let mut t = RatePower::default();

    // cck 1M, 2M, 5.5M, 11M
    let val = ee(bus, EE_TX_POWER_BYRATE_BASE) as u32;
    t.cck[0] = s6_to_s8(val);
    t.cck[1] = t.cck[0];
    t.cck[2] = s6_to_s8(val >> 8);
    t.cck[3] = t.cck[2];

    // ofdm 6M, 9M, 12M, 18M
    let addr = if is_2ghz {
        EE_TX_POWER_BYRATE_BASE + 2
    } else {
        EE_5G_BYRATE_OFDM_6M
    };
    let val = ee(bus, addr) as u32;
    t.ofdm[0] = s6_to_s8(val);
    t.ofdm[1] = t.ofdm[0];
    t.ofdm[2] = s6_to_s8(val >> 8);
    t.ofdm[3] = t.ofdm[2];

    // ofdm 24M, 36M, 48M, 54M
    let addr = if is_2ghz {
        EE_TX_POWER_BYRATE_BASE + 4
    } else {
        EE_5G_BYRATE_OFDM_24M
    };
    let val = ee(bus, addr) as u32;
    t.ofdm[4] = s6_to_s8(val);
    t.ofdm[5] = t.ofdm[4];
    t.ofdm[6] = s6_to_s8(val >> 8);
    t.ofdm[7] = t.ofdm[6];

    // ht-vht mcs 1ss 0, 1, 2, 3
    let addr = if is_2ghz {
        EE_TX_POWER_BYRATE_BASE + 6
    } else {
        EE_5G_BYRATE_MCS0
    };
    let val = ee(bus, addr) as u32;
    t.ht[0] = s6_to_s8(val);
    t.ht[1] = t.ht[0];
    t.ht[2] = s6_to_s8(val >> 8);
    t.ht[3] = t.ht[2];

    // ht-vht mcs 1ss 4, 5, 6
    let addr = if is_2ghz {
        EE_TX_POWER_BYRATE_BASE + 8
    } else {
        EE_5G_BYRATE_MCS4
    };
    let val = ee(bus, addr) as u32;
    t.ht[4] = s6_to_s8(val);
    t.ht[5] = t.ht[4];
    t.ht[6] = s6_to_s8(val >> 8);
    t.ht[7] = t.ht[6];

    // vht mcs 8, 9 (5 GHz table regardless of band — upstream reads 0x12c
    // unconditionally, eeprom.c:192)
    let val = ee(bus, EE_5G_BYRATE_VHT_MCS8) as u32;
    t.vht[0] = s6_to_s8(val);
    t.vht[1] = s6_to_s8(val >> 8);

    // With closed-loop TSSI the hardware handles the bandwidth delta itself.
    let delta = if tssi_enabled(bus) {
        0
    } else {
        power_delta(bus, bw, !is_2ghz)
    };
    t.add_offset(delta);
    t
}

/// `mt76x0_get_power_info` (`mt76x0/eeprom.c:200`) — the absolute ALC target
/// power for a channel, in **0.5 dB steps**, 0..0x3f.
///
/// Two paths. With closed-loop TSSI the target comes from the band's target-power
/// EEPROM byte less the 54M OFDM rate delta, plus the bandwidth delta. Without
/// it, a per-channel byte array is indexed through `chan_map`, whose `idx` trick
/// is worth spelling out: `idx` is 1 only on an *exact* channel match, and it
/// selects the high byte of the word — so each 16-bit entry holds the power for
/// two adjacent channels.
///
/// Out-of-range results fall back to `5` (2.5 dB), upstream's `eeprom.c:269`.
pub fn target_power(bus: &dyn PhyBus, channel: u8, bw: Bandwidth, rates: &RatePower) -> i8 {
    let is_5ghz = channel > 14;

    if tssi_enabled(bus) {
        let data = if is_5ghz {
            ee(bus, EE_5G_TARGET_POWER)
        } else {
            ee(bus, EE_2G_TARGET_POWER)
        };
        let tp = ((data & 0xff) as i32) - rates.ofdm[7] as i32;
        return (tp + power_delta(bus, bw, is_5ghz) as i32).clamp(-128, 127) as i8;
    }

    /// `struct mt76x0_chan_map` (`mt76x0/eeprom.c:203`): the last channel of each
    /// EEPROM slot, and that slot's byte offset.
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

    let mut idx = 0u32;
    // eeprom.c:239 — if nothing matched, offset falls back to the first entry's.
    let mut offset = CHAN_MAP[0].1;
    for &(map_chan, map_offset) in CHAN_MAP {
        if channel <= map_chan {
            idx = u32::from(channel == map_chan);
            offset = map_offset;
            break;
        }
    }

    let addr = if !is_5ghz {
        EE_TX_POWER_DELTA_BW80 + offset as u16
    } else {
        // eeprom.c:245 — five 80 MHz-centre channels get a hand-written offset
        // that overrides the chan_map lookup entirely.
        let offset = match channel {
            42 => 2,
            58 => 8,
            106 => 14,
            122 => 20,
            155 => 30,
            _ => offset,
        };
        EE_TX_POWER_0_GRP4_TSSI_SLOPE + 2 + offset as u16
    };

    let data = ee(bus, addr);
    let tp = ((data >> (8 * idx)) & 0xff) as u8 as i8;
    // Upstream spells this `if (*tp < 0 || *tp > 0x3f)` (eeprom.c:269).
    if !(0..=0x3f).contains(&tp) { 5 } else { tp }
}

/// `mt76x02_tx_power_mask` (`mt76x02_phy.c:50`) — four 6-bit per-rate powers,
/// one per byte lane.
const fn tx_power_mask(v1: i8, v2: i8, v3: i8, v4: i8) -> u32 {
    ((v1 as u32) & 0x3f)
        | (((v2 as u32) & 0x3f) << 8)
        | (((v3 as u32) & 0x3f) << 16)
        | (((v4 as u32) & 0x3f) << 24)
}

/// `mt76x02_phy_set_txpower` (`mt76x02_phy.c:93`) — program the ALC target and
/// the eight per-rate power registers.
///
/// `txp_0` / `txp_1` are the per-chain ALC targets in 0.5 dB steps; on a 1×1 part
/// both callers pass the same value.
///
/// ⚠ `MT_TX_PWR_CFG_3` and `_4` re-pack `ht[0..7]`, the same values already in
/// `_1` and `_2`, and `_8` / `_9` re-pack `vht[0..1]` already in `_7`. Those
/// registers are the **STBC** rate tables in the datasheet-less register map and
/// upstream fills them with the non-STBC values. UNVERIFIED
/// (`mt76x02_phy.c:109-120`): whether that is intentional (STBC tracks the base
/// rate) or a long-standing copy-paste. Ported as written.
pub fn phy_set_txpower(
    bus: &dyn PhyBus,
    t: &RatePower,
    txp_0: u32,
    txp_1: u32,
) -> Result<(), FaceError> {
    bus.rmw(
        MT_TX_ALC_CFG_0,
        MT_TX_ALC_CFG_0_CH_INIT_0,
        field_prep(MT_TX_ALC_CFG_0_CH_INIT_0, txp_0),
    )?;
    bus.rmw(
        MT_TX_ALC_CFG_0,
        MT_TX_ALC_CFG_0_CH_INIT_1,
        field_prep(MT_TX_ALC_CFG_0_CH_INIT_1, txp_1),
    )?;

    bus.wr(
        MT_TX_PWR_CFG_0,
        tx_power_mask(t.cck[0], t.cck[2], t.ofdm[0], t.ofdm[2]),
    )?;
    bus.wr(
        MT_TX_PWR_CFG_1,
        tx_power_mask(t.ofdm[4], t.ofdm[6], t.ht[0], t.ht[2]),
    )?;
    bus.wr(
        MT_TX_PWR_CFG_2,
        tx_power_mask(t.ht[4], t.ht[6], t.ht[8], t.ht[10]),
    )?;
    bus.wr(
        MT_TX_PWR_CFG_3,
        tx_power_mask(t.ht[12], t.ht[14], t.ht[0], t.ht[2]),
    )?;
    bus.wr(MT_TX_PWR_CFG_4, tx_power_mask(t.ht[4], t.ht[6], 0, 0))?;
    bus.wr(
        MT_TX_PWR_CFG_7,
        tx_power_mask(t.ofdm[7], t.vht[0], t.ht[7], t.vht[1]),
    )?;
    bus.wr(
        MT_TX_PWR_CFG_8,
        tx_power_mask(t.ht[14], 0, t.vht[0], t.vht[1]),
    )?;
    bus.wr(
        MT_TX_PWR_CFG_9,
        tx_power_mask(t.ht[7], 0, t.vht[0], t.vht[1]),
    )
}

/// Program TX power, returning the ALC target actually applied in **half-dB**
/// (0.5 dB) steps.
///
/// This is the honest unit. See [`set_tx_power`] for the dBm-rounded wrapper and
/// for what the number does and does not mean.
///
/// Ports `mt76x0_phy_set_txpower` (`mt76x0/phy.c:844`) with the two knobs the
/// port contract asks for grafted on:
///   * `index` — the **opaque** knob. Overrides the EEPROM-derived ALC target
///     with a raw 0..63 index. Bypasses `mt76x0_get_power_info` entirely; the
///     per-rate deltas still come from the EEPROM.
///   * `target_half_dbm` — the **limit**, upstream's `dev->txpower_conf` (which
///     mac80211 fills as `power_level * 2`, `mt76x0/main.c:72` — confirming the
///     0.5 dB unit). Every per-rate value is clamped to it before the deltas are
///     re-referenced to the ALC target.
///
/// The ALC target is clamped to `MT_TX_ALC_CFG_0_CH_INIT_0`'s six bits, i.e.
/// 0..63 → 0…+31.5. The neighbouring `LIMIT_0` / `LIMIT_1` fields
/// (`mt76x02_regs.h:490`) are the hardware ceiling; mt76x0 never writes them, and
/// `mt76x0/initvals.h`'s MAC table leaves them at `0x2f` (23.5) — see
/// [`set_alc_limits`] if you want to move them.
pub fn set_tx_power_half_dbm(
    bus: &dyn PhyBus,
    channel: u8,
    bw: Bandwidth,
    target_half_dbm: Option<i32>,
    index: Option<u8>,
) -> Result<i8, FaceError> {
    let mut t = tx_power_per_rate(bus, channel, bw);
    let info = match index {
        Some(i) => (i as i8).clamp(0, 0x3f),
        None => target_power(bus, channel, bw, &t).clamp(0, 0x3f),
    };

    // phy.c:852-855: shift the per-rate deltas up to absolute power, clamp them
    // against the regulatory/user limit, then shift them back down — so what
    // reaches MT_TX_PWR_CFG_* is a delta from CH_INIT again, but a *limited* one.
    t.add_offset(info);
    // 63 half-dB = 31.5 dB is the widest the 6-bit field can express, so it is
    // the natural "no limit" value rather than i32::MAX.
    t.limit(target_half_dbm.unwrap_or(0x3f));
    let max_applied = t.max();
    t.add_offset(-info);

    phy_set_txpower(bus, &t, info as u32, info as u32)?;
    Ok(max_applied)
}

/// Program TX power and return the dBm applied.
///
/// The port contract's entry point. `bw` is not in the signature, so 20 MHz is
/// assumed for the bandwidth power delta — call [`set_tx_power_half_dbm`]
/// directly when the radio is on 40/80 MHz, or the delta will be wrong by the
/// EEPROM's BW40/BW80 correction (a few dB).
///
/// ## Units, and what the return value is worth
/// Everything in the mt76x02 power path is **0.5 dB steps**. `target_dbm` is
/// multiplied by 2 to become upstream's `txpower_conf`; `index` is written raw
/// into the 6-bit ALC target. The returned `i8` is the applied level **rounded
/// down to whole dBm**, so a 0.5 dB step is invisible through this signature —
/// [`set_tx_power_half_dbm`] returns the full resolution.
///
/// ⚠ The dBm axis is **UNCALIBRATED on this part**. What is programmed is an ALC
/// target referenced to the board's EEPROM calibration, not a measured power at
/// the antenna. This function reports the number it programmed, not a number
/// anyone has seen on a spectrum analyser. Until an SDR measurement exists, treat
/// it the way the RTL8733B port treats TSSI-DE: monotonic and repeatable, with
/// no trustworthy absolute reference.
pub fn set_tx_power(
    bus: &dyn PhyBus,
    channel: u8,
    target_dbm: Option<i8>,
    index: Option<u8>,
) -> Result<i8, FaceError> {
    let half = set_tx_power_half_dbm(
        bus,
        channel,
        Bandwidth::Bw20,
        target_dbm.map(|d| d as i32 * 2),
        index,
    )?;
    Ok(half.div_euclid(2))
}

/// Write the hardware ALC ceiling, `MT_TX_ALC_CFG_0` bits 21:16 / 29:24
/// (`mt76x02_regs.h:490`), in 0.5 dB steps.
///
/// Not part of any upstream mt76x0 path — `mt76x0/initvals.h` sets the whole
/// register to `0x2F2F000C` (limits 23.5 dB, target 6 dB) and nothing moves the
/// limits afterwards. Exposed because it is the only *hard* stop in the TX power
/// chain, and a named-radio deployment may want one below the board default.
pub fn set_alc_limits(bus: &dyn PhyBus, limit_0: u8, limit_1: u8) -> Result<(), FaceError> {
    bus.rmw(
        MT_TX_ALC_CFG_0,
        MT_TX_ALC_CFG_0_LIMIT_0 | MT_TX_ALC_CFG_0_LIMIT_1,
        field_prep(MT_TX_ALC_CFG_0_LIMIT_0, (limit_0 & 0x3f) as u32)
            | field_prep(MT_TX_ALC_CFG_0_LIMIT_1, (limit_1 & 0x3f) as u32),
    )?;
    Ok(())
}

// ── Channel selection ───────────────────────────────────────────────────────

/// `mt76x0_phy_set_band` (`mt76x0/phy.c:205`) — the RF-side band switch.
///
/// Reloads the whole channel-0 table for the band, flips the two LO-buffer
/// registers `MT_RF(5, 0)` / `MT_RF(6, 0)`, and sets the band's TX VGA and gain
/// correction. The `0x45`/`0x44` pair swapping between bands is the 2.4 GHz and
/// 5 GHz LO paths trading places.
fn set_band(bus: &dyn PhyBus, is_5ghz: bool) -> Result<(), FaceError> {
    if is_5ghz {
        rf_wr_table(bus, RF_5G_CHANNEL_0_TAB)?;
        rf_wr(bus, mt_rf(5, 0), 0x44)?;
        rf_wr(bus, mt_rf(6, 0), 0x45)?;
        bus.wr(MT_TX_ALC_VGA3, 0x0000_0005)?;
        bus.wr(MT_TX0_RF_GAIN_CORR, 0x0101_0102)?;
    } else {
        rf_wr_table(bus, RF_2G_CHANNEL_0_TAB)?;
        rf_wr(bus, mt_rf(5, 0), 0x45)?;
        rf_wr(bus, mt_rf(6, 0), 0x44)?;
        bus.wr(MT_TX_ALC_VGA3, 0x0005_0007)?;
        bus.wr(MT_TX0_RF_GAIN_CORR, 0x003E_0002)?;
    }
    Ok(())
}

/// Program the PLL registers R37…R24 from a [`FreqItem`]
/// (`mt76x0_phy_set_chan_rf_params`, `mt76x0/phy.c:260-332`).
///
/// This is the fiddly part of the port: each `pll_r*` field of the row is one
/// named bit-slice of one RF register, and the slices are written with
/// read-modify-write in a fixed order. Every write below carries its upstream
/// line. Three of them are not plain field writes:
///   * **R30<7> `sdm_reset_n`** — on the SDM (fractional-N) plan upstream ignores
///     the row's value and *pulses* the bit clear-then-set (`phy.c:282-287`),
///     taking the modulator out of reset. On the integer-N plan it RMWs the row
///     value instead. Collapsing these two arms into one RMW leaves the SDM in
///     reset and the PLL unlocked.
///   * **`pll_n`** spans R29<7:0> plus R30<0> (`phy.c:302-307`).
///   * **`pll_sdm_k`** spans R26, R27 and R28<1:0> — 18 bits (`phy.c:321-328`).
fn set_chan_pll_params(bus: &dyn PhyBus, item: &FreqItem, b_sdm: bool) -> Result<(), FaceError> {
    rf_wr(bus, mt_rf(0, 37), item.pll_r37)?; // phy.c:260
    rf_wr(bus, mt_rf(0, 36), item.pll_r36)?; // phy.c:261
    rf_wr(bus, mt_rf(0, 35), item.pll_r35)?; // phy.c:262
    rf_wr(bus, mt_rf(0, 34), item.pll_r34)?; // phy.c:263
    rf_wr(bus, mt_rf(0, 33), item.pll_r33)?; // phy.c:264

    // R32<7:5> — phy.c:266. Row values are pre-shifted, hence the raw 0xe0 mask.
    rf_rmw(bus, mt_rf(0, 32), 0xe0, item.pll_r32_b7b5)?;
    // R32<4:0> pll_den (Denomina - 8) — phy.c:270
    rf_rmw(
        bus,
        mt_rf(0, 32),
        regs::MT_RF_PLL_DEN_MASK,
        item.pll_r32_b4b0,
    )?;

    // R31<7:5> — phy.c:274
    rf_rmw(bus, mt_rf(0, 31), 0xe0, item.pll_r31_b7b5)?;
    // R31<4:0> pll_k (Nominator) — phy.c:278
    rf_rmw(bus, mt_rf(0, 31), regs::MT_RF_PLL_K_MASK, item.pll_r31_b4b0)?;

    // R30<7> sdm_reset_n — phy.c:281
    if b_sdm {
        rf_clear(bus, mt_rf(0, 30), MT_RF_SDM_RESET_MASK)?;
        rf_set(bus, mt_rf(0, 30), MT_RF_SDM_RESET_MASK)?;
    } else {
        rf_rmw(bus, mt_rf(0, 30), MT_RF_SDM_RESET_MASK, item.pll_r30_b7)?;
    }

    // R30<6:2> sdmmash_prbs,sin — phy.c:294
    rf_rmw(
        bus,
        mt_rf(0, 30),
        MT_RF_SDM_MASH_PRBS_MASK,
        item.pll_r30_b6b2,
    )?;
    // R30<1> sdm_bp — phy.c:299. Note the << 1: this field is NOT pre-shifted.
    rf_rmw(bus, mt_rf(0, 30), MT_RF_SDM_BP_MASK, item.pll_r30_b1 << 1)?;

    // R30<0> R29<7:0> (hex) pll_n — phy.c:303
    rf_wr(bus, mt_rf(0, 29), (item.pll_n & 0xff) as u8)?;
    rf_rmw(bus, mt_rf(0, 30), 0x1, ((item.pll_n >> 8) & 0x1) as u8)?; // phy.c:306

    // R28<7:6> isi_iso — phy.c:310
    rf_rmw(
        bus,
        mt_rf(0, 28),
        regs::MT_RF_ISI_ISO_MASK,
        item.pll_r28_b7b6,
    )?;
    // R28<5:4> pfd_dly — phy.c:314
    rf_rmw(
        bus,
        mt_rf(0, 28),
        regs::MT_RF_PFD_DLY_MASK,
        item.pll_r28_b5b4,
    )?;
    // R28<3:2> clksel option — phy.c:318
    rf_rmw(
        bus,
        mt_rf(0, 28),
        regs::MT_RF_CLK_SEL_MASK,
        item.pll_r28_b3b2,
    )?;

    // R28<1:0> R27<7:0> R26<7:0> (hex) sdm_k — phy.c:322
    rf_wr(bus, mt_rf(0, 26), (item.pll_sdm_k & 0xff) as u8)?;
    rf_wr(bus, mt_rf(0, 27), ((item.pll_sdm_k >> 8) & 0xff) as u8)?;
    rf_rmw(bus, mt_rf(0, 28), 0x3, ((item.pll_sdm_k >> 16) & 0x3) as u8)?; // phy.c:327

    // R24<1:0> xo_div — phy.c:331
    rf_rmw(
        bus,
        mt_rf(0, 24),
        regs::MT_RF_XO_DIV_MASK,
        item.pll_r24_b1b0,
    )?;
    Ok(())
}

/// `mt76x0_phy_set_chan_rf_params` (`mt76x0/phy.c:232`) — the RF half of a
/// channel change.
///
/// `channel` here is the **centre** channel, not the control channel: the caller
/// has already folded the 40/80 MHz offset in (see [`set_channel_ext`]).
/// `rf_bw_band` packs `RF_BW_*` in the low byte and `RF_*_BAND*` in the high.
///
/// Note the band handling: the caller's band bits are *replaced* by the frequency
/// plan row's `band` (`phy.c:253`), which is where the 5 GHz LB/MB/HB sub-band
/// distinction comes from. The three table loops that follow each use a different
/// match rule — see the comments and `initvals_phy.rs`'s module docs.
fn set_chan_rf_params(bus: &dyn PhyBus, channel: u8, rf_bw_band: u16) -> Result<(), FaceError> {
    let rf_bw = rf_bw_band & 0x00ff;
    let b_sdm = freq_plan::uses_sdm(channel);

    let item = freq_plan::lookup(channel, b_sdm).ok_or_else(|| {
        phy_err(format!(
            "mt76x0: no PLL frequency plan for centre channel {channel}"
        ))
    })?;

    // phy.c:253 — the caller's band bits (`rf_bw_band & 0xff00`) are *discarded*
    // here: the plan row is authoritative and is what carries the 5 GHz
    // LB/MB/HB/11J sub-band distinction the switch tables below match on.
    // Upstream keeps the caller's bits only when no row matches, which it
    // reaches by falling out of the loop with an uninitialised `freq_item`; this
    // port errors on that path instead, so the caller's band bits are dead.
    let rf_band = item.band as u16;
    set_chan_pll_params(bus, item, b_sdm)?;

    // Bandwidth switch — phy.c:338. Two arms: a row whose whole bw_band equals
    // the requested bandwidth (the band-agnostic RF_BW_*-only rows), or a row
    // whose low byte equals it and whose band bits intersect the channel's band.
    for item in RF_BW_SWITCH_TAB {
        if rf_bw == item.bw_band
            || (rf_bw == (item.bw_band & 0xff) && (rf_band & item.bw_band) != 0)
        {
            rf_wr(bus, item.rf_bank_reg, item.value)?;
        }
    }

    // Band switch — phy.c:351. Any band-bit intersection.
    for item in RF_BAND_SWITCH_TAB {
        if item.bw_band & rf_band != 0 {
            rf_wr(bus, item.rf_bank_reg, item.value)?;
        }
    }

    bus.rmw(MT_RF_MISC, 0xc, 0)?; // phy.c:359

    let is_5ghz = rf_band & RF_G_BAND == 0;
    if ext_pa_enabled(bus, is_5ghz) {
        // MT_RF_MISC (0x0518): [2] external A-band PA enable, [3] external
        // G-band PA enable — phy.c:363.
        if rf_band & RF_A_BAND != 0 {
            bus.rmw(MT_RF_MISC, 0, 1 << 2)?;
        } else {
            bus.rmw(MT_RF_MISC, 0, 1 << 3)?;
        }
        for item in RF_EXT_PA_TAB {
            if item.bw_band & rf_band != 0 {
                rf_wr(bus, item.rf_bank_reg, item.value)?;
            }
        }
    }

    // phy.c:382. The MT_TX_ALC_CFG_1 masks are AND-only — they clear the atten
    // mode and the "Tx Inc dcoc" enable while preserving the temperature
    // compensation in bits 7:0. The two magic constants are upstream's and are
    // undocumented beyond the comment; ported verbatim.
    if rf_band & RF_G_BAND != 0 {
        bus.wr(MT_TX0_RF_GAIN_ATTEN, 0x6370_7400)?;
        // "Set Atten mode = 2 For G band, Disable Tx Inc dcoc."
        let mac_reg = bus.rr(MT_TX_ALC_CFG_1)? & 0x8964_00FF;
        bus.wr(MT_TX_ALC_CFG_1, mac_reg)?;
    } else {
        bus.wr(MT_TX0_RF_GAIN_ATTEN, 0x686A_7800)?;
        // "Set Atten mode = 0. For Ext A band, Disable Tx Inc dcoc Cal."
        let mac_reg = bus.rr(MT_TX_ALC_CFG_1)? & 0x8904_00FF;
        bus.wr(MT_TX_ALC_CFG_1, mac_reg)?;
    }
    Ok(())
}

/// `mt76x0_phy_set_chan_bbp_params` (`mt76x0/phy.c:399`) — the BBP half.
///
/// The row filter is the *superset* rule (`BbpSwitch::matches`), not equality.
/// One row is special: `MT_BBP(AGC, 8)` carries a VGA gain that must be reduced
/// by twice the EEPROM LNA gain before the write (`phy.c:411-419`), because the
/// table value assumes no external LNA.
fn set_chan_bbp_params(bus: &dyn PhyBus, rf_bw_band: u16, lna_gain: u8) -> Result<(), FaceError> {
    for item in BBP_SWITCH_TAB {
        if !item.matches(rf_bw_band) {
            continue;
        }
        if item.reg == BBP_AGC_8 {
            let gain = field_get(MT_BBP_AGC_GAIN, item.val) as u8;
            // u8 arithmetic upstream, and it can wrap on a board whose LNA gain
            // exceeds half the table gain. wrapping_sub keeps the same result.
            let gain = gain.wrapping_sub(lna_gain.wrapping_mul(2));
            let val = (item.val & !MT_BBP_AGC_GAIN) | field_prep(MT_BBP_AGC_GAIN, gain as u32);
            bus.wr(item.reg, val)?;
        } else {
            bus.wr(item.reg, item.val)?;
        }
    }
    Ok(())
}

/// `mt76x0_phy_bbp_set_bw` (`mt76x0/phy.c:472`) — the USB bandwidth select.
///
/// On USB the bandwidth is set through the firmware (`BW_SETTING`); the MMIO
/// path instead writes `MT_TX_SW_CFG0` directly (`phy.c:973-979`), which is why
/// this is the USB-only arm. The encoding `{20:0, 40:1, 80:2, 10:4}` is
/// upstream's local enum at `phy.c:475` — note that 10 MHz is **4**, not 3.
pub fn bbp_set_bw(bus: &dyn PhyBus, bw: Bandwidth) -> Result<(), FaceError> {
    let code = match bw {
        Bandwidth::Bw20 => 0u32,
        Bandwidth::Bw40 => 1,
        Bandwidth::Bw80 => 2,
        Bandwidth::Nb10 => 4,
        Bandwidth::Nb5 => {
            // phy.c:495 returns without acting for 5 MHz / 160 / 80+80, leaving
            // the BBP on its previous bandwidth. Silently doing nothing is worse
            // than saying so.
            return Err(phy_err(
                "mt76x0: 5 MHz bandwidth is not supported by the BBP".into(),
            ));
        }
    };
    mcu_function_select(bus, FUNC_BW_SETTING, code)
}

/// `mt76x02_phy_set_bw` (`mt76x02_phy.c:124`) — the BBP core/AGC bandwidth pair
/// plus the primary-channel slot `ctrl` (which 20 MHz sub-slot of the operating
/// bandwidth carries the control channel).
fn x02_set_bw(bus: &dyn PhyBus, bw: Bandwidth, ctrl: u8) -> Result<(), FaceError> {
    let (core_val, agc_val) = match bw {
        Bandwidth::Bw80 => (3u32, 7u32),
        Bandwidth::Bw40 => (2, 3),
        _ => (0, 1),
    };
    bus.rmw(
        BBP_CORE_1,
        MT_BBP_CORE_R1_BW,
        field_prep(MT_BBP_CORE_R1_BW, core_val),
    )?;
    bus.rmw(
        BBP_AGC_0,
        MT_BBP_AGC_R0_BW,
        field_prep(MT_BBP_AGC_R0_BW, agc_val),
    )?;
    bus.rmw(
        BBP_AGC_0,
        MT_BBP_AGC_R0_CTRL_CHAN,
        field_prep(MT_BBP_AGC_R0_CTRL_CHAN, ctrl as u32),
    )?;
    bus.rmw(
        BBP_TXBE_0,
        MT_BBP_TXBE_R0_CTRL_CHAN,
        field_prep(MT_BBP_TXBE_R0_CTRL_CHAN, ctrl as u32),
    )?;
    Ok(())
}

/// `mt76x02_phy_set_band` (`mt76x02_phy.c:150`) — the MAC-side band select and
/// the 40 MHz primary-is-upper flag.
fn x02_set_band(bus: &dyn PhyBus, is_5ghz: bool, primary_upper: bool) -> Result<(), FaceError> {
    if is_5ghz {
        bus.rmw(MT_TX_BAND_CFG, MT_TX_BAND_CFG_2G, 0)?;
        bus.rmw(MT_TX_BAND_CFG, 0, MT_TX_BAND_CFG_5G)?;
    } else {
        bus.rmw(MT_TX_BAND_CFG, 0, MT_TX_BAND_CFG_2G)?;
        bus.rmw(MT_TX_BAND_CFG, MT_TX_BAND_CFG_5G, 0)?;
    }
    bus.rmw(
        MT_TX_BAND_CFG,
        MT_TX_BAND_CFG_UPPER_40M,
        field_prep(MT_TX_BAND_CFG_UPPER_40M, u32::from(primary_upper)),
    )?;
    Ok(())
}

/// `ext_cca_chan[]` (`mt76x0/phy.c:916`) — the per-slot CCA antenna permutation
/// written to [`MT_EXT_CCA_CFG`], indexed by the primary-channel slot.
///
/// MEASURED cross-check: the kernel monitor leaves `MT_EXT_CCA_CFG = 0x0000f1e4`
/// on this part. The low 12 bits of entry `[0]` are `0x1e4`
/// (`CCA0=0,CCA1=1,CCA2=2,CCA3=3,MASK=BIT(0)`), and the `0xf000` in the top
/// nibble is the `ED_CCA_MASK` that `mt76x0_init_mac_registers` sets separately
/// (`mt76x0/init.c:121`). So this table and the measured value agree.
const EXT_CCA_CHAN: [u32; 4] = [
    field_prep(MT_EXT_CCA_CFG_CCA0, 0)
        | field_prep(MT_EXT_CCA_CFG_CCA1, 1)
        | field_prep(MT_EXT_CCA_CFG_CCA2, 2)
        | field_prep(MT_EXT_CCA_CFG_CCA3, 3)
        | field_prep(MT_EXT_CCA_CFG_CCA_MASK, 1 << 0),
    field_prep(MT_EXT_CCA_CFG_CCA0, 1)
        | field_prep(MT_EXT_CCA_CFG_CCA1, 0)
        | field_prep(MT_EXT_CCA_CFG_CCA2, 2)
        | field_prep(MT_EXT_CCA_CFG_CCA3, 3)
        | field_prep(MT_EXT_CCA_CFG_CCA_MASK, 1 << 1),
    field_prep(MT_EXT_CCA_CFG_CCA0, 2)
        | field_prep(MT_EXT_CCA_CFG_CCA1, 3)
        | field_prep(MT_EXT_CCA_CFG_CCA2, 1)
        | field_prep(MT_EXT_CCA_CFG_CCA3, 0)
        | field_prep(MT_EXT_CCA_CFG_CCA_MASK, 1 << 2),
    field_prep(MT_EXT_CCA_CFG_CCA0, 3)
        | field_prep(MT_EXT_CCA_CFG_CCA1, 2)
        | field_prep(MT_EXT_CCA_CFG_CCA2, 1)
        | field_prep(MT_EXT_CCA_CFG_CCA3, 0)
        | field_prep(MT_EXT_CCA_CFG_CCA_MASK, 1 << 3),
];

/// The 5 GHz 80 MHz channel groups: `(first control channel, centre channel)`.
///
/// Upstream never needs this table — mac80211 hands it `chandef.center_freq1`
/// already resolved. This port takes only a control channel, so the centre has to
/// be derived. The four control channels of a group are `first, first+4, +8, +12`
/// and the centre is `first + 6`, which is exactly what upstream's
/// `channel += 6 - ch_group_index * 4` computes from the centre frequency
/// (`phy.c:959-962`) — the two derivations agree by construction, and the unit
/// test below checks that they do.
const VHT80_GROUPS: &[(u8, u8)] = &[
    (36, 42),
    (52, 58),
    (100, 106),
    (116, 122),
    (132, 138),
    (149, 155),
    (165, 171),
];

/// Where the secondary 20 MHz channel of an HT40 pair sits by convention.
///
/// Not upstream: mac80211 supplies this as `center_freq1`. `true` means the
/// secondary channel is **above** the control channel (HT40+), which is
/// upstream's `ch_group_index == 0`.
///
/// The convention is the standard channel pairing: 2.4 GHz channels 1–7 pair
/// upward and 8–14 downward; 5 GHz channels pair as (36,40), (44,48), … with the
/// lower member of each pair going upward. Pass an explicit value to
/// [`set_channel_ext`] rather than relying on this when the regulatory domain or
/// the peer disagrees.
pub fn ht40_secondary_above(channel: u8) -> bool {
    if channel <= 14 {
        channel <= 7
    } else if channel >= 149 {
        ((channel - 149) / 4).is_multiple_of(2)
    } else {
        ((channel.saturating_sub(36)) / 4).is_multiple_of(2)
    }
}

/// `mt76x0_phy_set_channel` (`mt76x0/phy.c:913`) — retune the radio.
///
/// `channel` is the **control** channel. For 40 MHz the secondary-channel side is
/// taken from [`ht40_secondary_above`]; for 80 MHz the group is looked up in
/// [`VHT80_GROUPS`]. Pass `secondary_above` explicitly to override the former.
///
/// ## Bandwidth support
/// * **20 MHz, 2.4 and 5 GHz** — full support.
/// * **40 MHz, 2.4 and 5 GHz** — full support.
/// * **80 MHz, 5 GHz only** — supported. The 1×1 MT7610U *is* a VHT80 part
///   (AC433) and upstream carries `RF_A_BAND | RF_BW_80` rows throughout
///   `RF_BW_SWITCH_TAB` and `BBP_SWITCH_TAB`. There is no `RF_G_BAND | RF_BW_80`
///   row anywhere, so 80 MHz in 2.4 GHz is rejected rather than programmed with
///   whatever the 40 MHz rows happen to leave behind.
/// * **10 MHz / 5 MHz** — rejected. The BBP has a 10 MHz code (`phy.c:491`) but
///   no RF or BBP switch-table rows exist for it in `initvals_phy.h`, so
///   selecting it would tune the synthesiser for 20 MHz and narrow only the
///   digital filter. Upstream never selects it either.
///
/// ## What this does *not* do
/// Upstream ends by scheduling `cal_work` (`phy.c:1014`). This port does not own
/// a timer; drive [`calibration_tick`] yourself if you want the AGC/TSSI loop.
/// Upstream also returns early during a scan (`phy.c:1007`), skipping AGC init,
/// calibration and TX power. Set `scan` to get that behaviour.
pub fn set_channel_ext(
    bus: &dyn PhyBus,
    channel: u8,
    bw: Bandwidth,
    secondary_above: Option<bool>,
    scan: bool,
) -> Result<(), FaceError> {
    let is_5ghz = channel > 14;
    let mut rf_bw_band = if channel <= 14 { RF_G_BAND } else { RF_A_BAND };

    // phy.c:949 — resolve the centre channel and the primary-channel slot.
    let (centre, ch_group_index) = match bw {
        Bandwidth::Bw20 => {
            rf_bw_band |= RF_BW_20;
            (channel, 0u8)
        }
        Bandwidth::Bw40 => {
            rf_bw_band |= RF_BW_40;
            let above = secondary_above.unwrap_or_else(|| ht40_secondary_above(channel));
            // phy.c:950-956: group 0 = secondary above (centre is control + 2),
            // group 1 = secondary below (centre is control - 2).
            if above {
                (channel + 2, 0u8)
            } else {
                (
                    channel.checked_sub(2).ok_or_else(|| {
                        phy_err(format!("mt76x0: channel {channel} has no HT40- pair"))
                    })?,
                    1u8,
                )
            }
        }
        Bandwidth::Bw80 => {
            if !is_5ghz {
                return Err(phy_err(
                    "mt76x0: 80 MHz exists only on 5 GHz (no RF_G_BAND|RF_BW_80 \
                     rows in initvals_phy.h)"
                        .into(),
                ));
            }
            rf_bw_band |= RF_BW_80;
            let (first, centre) = VHT80_GROUPS
                .iter()
                .copied()
                .find(|&(first, _)| {
                    channel >= first && channel < first + 16 && (channel - first).is_multiple_of(4)
                })
                .ok_or_else(|| {
                    phy_err(format!("mt76x0: channel {channel} is in no 80 MHz group"))
                })?;
            (centre, (channel - first) / 4)
        }
        Bandwidth::Nb10 | Bandwidth::Nb5 => {
            return Err(phy_err(format!(
                "mt76x0: {bw:?} has no RF/BBP switch-table rows in initvals_phy.h; \
                 only 20/40 MHz (2.4+5 GHz) and 80 MHz (5 GHz) are programmable"
            )));
        }
    };

    // Pure-EEPROM lookups, hoisted. Upstream calls mt76x0_read_rx_gain in the
    // middle of the sequence (phy.c:1002) but it touches no registers, so the
    // ordering carries no meaning.
    let gain = rx_gain(bus, channel);

    // phy.c:971 — USB takes the firmware bandwidth select; MMIO writes
    // MT_TX_SW_CFG0 instead. This is the USB driver.
    bbp_set_bw(bus, bw)?;

    x02_set_bw(bus, bw, ch_group_index)?;
    x02_set_band(bus, is_5ghz, ch_group_index & 1 != 0)?; // phy.c:982

    bus.rmw(
        MT_EXT_CCA_CFG,
        MT_EXT_CCA_CFG_CCA0
            | MT_EXT_CCA_CFG_CCA1
            | MT_EXT_CCA_CFG_CCA2
            | MT_EXT_CCA_CFG_CCA3
            | MT_EXT_CCA_CFG_CCA_MASK,
        EXT_CCA_CHAN[(ch_group_index & 3) as usize],
    )?;

    set_band(bus, is_5ghz)?;
    set_chan_rf_params(bus, centre, rf_bw_band)?;

    // "Set Japan Tx filter at channel 14" — phy.c:996. Upstream tests the
    // *centre* channel here (the variable it has been rewriting), which for
    // 20 MHz is the control channel; no HT40 pair centres on 14.
    if centre == 14 {
        bus.rmw(BBP_CORE_1, 0, 0x20)?;
    } else {
        bus.rmw(BBP_CORE_1, 0x20, 0)?;
    }

    set_chan_bbp_params(bus, rf_bw_band, gain.lna_gain)?;

    // "enable vco" — phy.c:1005. The self-clearing vcocal_en bit again.
    rf_set(bus, mt_rf(0, 4), 1 << 7)?;
    if scan {
        return Ok(());
    }

    // The caller owns the AGC state; this call is here for its register side
    // effect (upstream's mt76x02_init_agc_gain snapshots MT_BBP(AGC, 8/9) and
    // resets the low-gain tracker) and its result is deliberately discarded.
    let _ = init_agc_gain(bus)?;
    calibrate(bus, channel, false)?;
    set_tx_power_half_dbm(bus, channel, bw, None, None)?;
    Ok(())
}

/// The port contract's channel entry point: [`set_channel_ext`] with the
/// conventional HT40 pairing and no scan short-circuit.
pub fn set_channel(bus: &dyn PhyBus, channel: u8, bw: Bandwidth) -> Result<(), FaceError> {
    set_channel_ext(bus, channel, bw, None, false)
}

// ── Calibration ─────────────────────────────────────────────────────────────

/// The calibration values that must survive between calls.
///
/// Upstream keeps these in `struct mt76x02_calibration` (`mt76x02.h:37`); this
/// port is stateless, so a caller that wants the TSSI or temperature loops must
/// own one of these and pass it in. [`calibrate`] runs against a throwaway.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct CalState {
    /// TSSI DC reference measured by [`tssi_dc_calibrate`] (`cal.tssi_dc`).
    pub tssi_dc: i16,
    /// Running TSSI error target in Q12 (`cal.tssi_target`).
    pub tssi_target: i32,
    /// Last temperature that triggered a full recalibration (`cal.temp`).
    pub temp: i8,
    /// Last temperature that triggered a VCO recalibration (`cal.temp_vco`).
    pub temp_vco: i8,
}

/// `mt76x0_phy_calibrate` (`mt76x0/phy.c:861`) — the firmware calibration ladder.
///
/// `full` is the port contract's name for upstream's `power_on`: it selects the
/// extra one-time steps that only make sense on a cold radio. The ladder is:
///
/// | step | when | argument |
/// |---|---|---|
/// | `MCU_CAL_R` | `full` only | 0 |
/// | `MCU_CAL_VCO` | `full` only | channel number |
/// | TSSI DC ([`tssi_dc_calibrate`]) | `full` **and** closed-loop TSSI enabled | — |
/// | `MCU_CAL_FULL` | always | band/sub-band code, below |
/// | `MCU_CAL_LC` | always | `1` on 5 GHz, `0` on 2.4 GHz |
/// | `MCU_CAL_RXDCOC` | always | 1 |
///
/// The `MCU_CAL_FULL` argument is `0x600` on 2.4 GHz and, on 5 GHz, `0x701` /
/// `0x801` / `0x901` for channels below 100 / below 140 / above (`phy.c:892`).
/// UNVERIFIED (`phy.c:893-901`): the encoding of those constants. The low byte
/// tracks the band and the high nibble the 5 GHz sub-band, which is consistent
/// with nothing else in the driver; ported as literals.
///
/// `MT_TX_ALC_CFG_0` is zeroed across the ladder (so the ALC does not chase the
/// calibration tones) and `MT_BBP(IBI, 9)` is forced to `0xffffff7e` and restored
/// — both saved and put back afterwards. There is a **15–20 ms** sleep after
/// `MCU_CAL_LC`; a channel change therefore costs at least that.
///
/// There are no separate host-side RXDCOC / LOFT / TXIQ sequences on this part:
/// `MCU_CAL_LOFT` (4) and `MCU_CAL_TXIQ` (5) exist in the firmware's command set
/// (`mt76x0/mcu.h:26`) but no upstream mt76x0 code issues them — `MCU_CAL_FULL`
/// runs them.
pub fn calibrate(bus: &dyn PhyBus, channel: u8, full: bool) -> Result<(), FaceError> {
    let mut state = CalState::default();
    calibrate_with(bus, channel, full, &mut state)
}

/// [`calibrate`], keeping the TSSI DC reference it measures.
pub fn calibrate_with(
    bus: &dyn PhyBus,
    channel: u8,
    full: bool,
    state: &mut CalState,
) -> Result<(), FaceError> {
    let is_5ghz = channel > 14;

    if full {
        mcu_calibrate(bus, mcu_cal::R, 0)?;
        mcu_calibrate(bus, mcu_cal::VCO, channel as u32)?;
        usleep(10);

        if tssi_enabled(bus) {
            // RX only while the DC reference is taken, then TX+RX back on.
            bus.wr(MT_MAC_SYS_CTRL, MT_MAC_SYS_CTRL_ENABLE_RX)?;
            state.tssi_dc = tssi_dc_calibrate(bus, is_5ghz)?;
            bus.wr(
                MT_MAC_SYS_CTRL,
                MT_MAC_SYS_CTRL_ENABLE_TX | MT_MAC_SYS_CTRL_ENABLE_RX,
            )?;
        }
    }

    let tx_alc = bus.rr(MT_TX_ALC_CFG_0)?;
    bus.wr(MT_TX_ALC_CFG_0, 0)?;
    usleep(500);

    let reg_val = bus.rr(BBP_IBI_9)?;
    bus.wr(BBP_IBI_9, 0xffff_ff7e)?;

    let val = if is_5ghz {
        if channel < 100 {
            0x701
        } else if channel < 140 {
            0x801
        } else {
            0x901
        }
    } else {
        0x600
    };

    mcu_calibrate(bus, mcu_cal::FULL, val)?;
    mcu_calibrate(bus, mcu_cal::LC, u32::from(is_5ghz))?;
    sleep(Duration::from_millis(15));

    bus.wr(BBP_IBI_9, reg_val)?;
    bus.wr(MT_TX_ALC_CFG_0, tx_alc)?;
    mcu_calibrate(bus, mcu_cal::RXDCOC, 1)
}

// ── TSSI (closed-loop TX power) ─────────────────────────────────────────────

/// `mt76x0_phy_tssi_dc_calibrate` (`mt76x0/phy.c:503`) — measure the TSSI
/// detector's zero-signal DC level.
///
/// The TSSI ADC is read by asking `MT_BBP(CORE, 34)` for a measurement and
/// polling bit 4 for completion, then reading `MT_BBP(CORE, 35)`. The request
/// code differs by band (`0x80055` on 5 GHz, `0x80050` on 2.4). Around it the
/// ADDA is bypassed, the BBP is software-reset, TX is forced from DAC0 with no
/// signal, and everything is unwound afterwards.
///
/// The `MT_RF(0, 67)` clear/restore on 5 GHz (`phy.c:508`, `phy.c:538`) is
/// UNVERIFIED: bank-0 register 67 is undocumented; the low nibble is cleared for
/// the measurement and restored to `0x4`, not to what it was.
pub fn tssi_dc_calibrate(bus: &dyn PhyBus, is_5ghz: bool) -> Result<i16, FaceError> {
    if is_5ghz {
        rf_clear(bus, mt_rf(0, 67), 0xf)?;
    }

    // bypass ADDA control
    bus.wr(MT_RF_SETTING_0, 0x6000_2237)?;
    bus.wr(MT_RF_BYPASS_0, 0xffff_ffff)?;

    // bbp sw reset
    bus.rmw(BBP_CORE_4, 0, 1)?;
    usleep(500);
    bus.rmw(BBP_CORE_4, 1, 0)?;

    let val = if is_5ghz { 0x80055 } else { 0x80050 };
    bus.wr(BBP_CORE_34, val)?;

    // enable TX with DAC0 input
    bus.wr(BBP_TXBE_6, 1 << 31)?;

    // Upstream's mt76_poll_msec(..., 200) — 200 ms of 10 ms polls. Over USB each
    // read is 151 µs, so 20 reads is a comparable wall-clock budget without a
    // sleep loop.
    poll(bus, BBP_CORE_34, 1 << 4, 0, 20)?;
    let tssi_dc = (bus.rr(BBP_CORE_35)? & 0xff) as i16;

    // stop bypass ADDA / stop TX / bbp sw reset
    bus.wr(MT_RF_BYPASS_0, 0)?;
    bus.wr(BBP_TXBE_6, 0)?;
    bus.rmw(BBP_CORE_4, 0, 1)?;
    usleep(500);
    bus.rmw(BBP_CORE_4, 1, 0)?;

    if is_5ghz {
        rf_rmw(bus, mt_rf(0, 67), 0xf, 0x4)?;
    }
    Ok(tssi_dc)
}

/// `mt76x0_phy_tssi_adc_calibrate` (`mt76x0/phy.c:542`) — read the TSSI level of
/// the frame the hardware last transmitted, plus three "packet info" bytes that
/// describe which rate it was sent at.
///
/// Returns `(ltssi, info)`. On 5 GHz the level is biased by +128
/// (`phy.c:558`) — UNVERIFIED why; ported.
fn tssi_adc_calibrate(bus: &dyn PhyBus, is_5ghz: bool) -> Result<(i16, [u8; 3]), FaceError> {
    let val = if is_5ghz { 0x80055 } else { 0x80050 };
    bus.wr(BBP_CORE_34, val)?;

    if !poll(bus, BBP_CORE_34, 1 << 4, 0, 20)? {
        bus.rmw(BBP_CORE_34, 1 << 4, 0)?;
        return Err(phy_err("mt76x0: TSSI ADC measurement timed out".into()));
    }

    let mut ltssi = (bus.rr(BBP_CORE_35)? & 0xff) as i16;
    if is_5ghz {
        ltssi += 128;
    }

    let mut info = [0u8; 3];
    // packet info modes #1, #2, #3 — phy.c:561-571
    for (i, slot) in info.iter_mut().enumerate() {
        bus.wr(BBP_CORE_34, 0x80041 + i as u32)?;
        *slot = (bus.rr(BBP_CORE_35)? & 0xff) as u8;
    }
    Ok((ltssi, info))
}

/// `mt76x0_phy_get_rf_pa_mode` (`mt76x0/phy.c:576`) — two bits per rate out of
/// [`MT_RF_PA_MODE_CFG0`] (CCK/OFDM) or [`MT_RF_PA_MODE_CFG1`] (HT/VHT).
fn get_rf_pa_mode(bus: &dyn PhyBus, index: u32, tx_rate: u32) -> Result<u8, FaceError> {
    let reg = if index == 1 {
        MT_RF_PA_MODE_CFG1
    } else {
        MT_RF_PA_MODE_CFG0
    };
    let val = bus.rr(reg)?;
    Ok(((val >> (tx_rate * 2)) & 3) as u8)
}

/// `mt76x0_phy_get_target_power` (`mt76x0/phy.c:586`) — what the last transmitted
/// frame *should* have measured, from its rate and the per-rate power table.
///
/// The OFDM branch's rate→index map (`phy.c:606-632`) is the 802.11a
/// `RATE` field's Gray-ish ordering (`0xb`→6M, `0xf`→9M, `0xa`→12M, …), not a
/// linear index; it is transcribed verbatim.
fn get_target_power(
    bus: &dyn PhyBus,
    tx_mode: u8,
    info: &[u8; 3],
    rates: &RatePower,
) -> Result<(i8, i8), FaceError> {
    let cur_power = (bus.rr(MT_TX_ALC_CFG_0)? & MT_TX_ALC_CFG_0_CH_INIT_0) as i8;

    let (target_power, target_pa_power) = match tx_mode {
        0 => {
            // cck rates
            let tx_rate = ((info[0] & 0x60) >> 5) as usize;
            (
                cur_power.saturating_add(rates.cck[tx_rate]),
                get_rf_pa_mode(bus, 0, tx_rate as u32)? as i8,
            )
        }
        1 => {
            // ofdm rates
            let tx_rate = (info[0] & 0xf0) >> 4;
            let index: usize = match tx_rate {
                0xb => 0,
                0xf => 1,
                0xa => 2,
                0xe => 3,
                0x9 => 4,
                0xd => 5,
                0x8 => 6,
                0xc => 7,
                _ => return Err(phy_err(format!("mt76x0 TSSI: bad OFDM rate {tx_rate:#x}"))),
            };
            (
                cur_power.saturating_add(rates.ofdm[index]),
                get_rf_pa_mode(bus, 0, index as u32 + 4)? as i8,
            )
        }
        4 => {
            // vht rates
            let tx_rate = (info[1] & 0xf) as usize;
            if tx_rate > 9 {
                return Err(phy_err(format!("mt76x0 TSSI: bad VHT rate {tx_rate}")));
            }
            let delta = if tx_rate > 7 {
                rates.vht[tx_rate - 8]
            } else {
                rates.ht[tx_rate]
            };
            (
                cur_power.saturating_add(delta),
                get_rf_pa_mode(bus, 1, tx_rate as u32)? as i8,
            )
        }
        _ => {
            // ht rates
            let tx_rate = (info[1] & 0x7f) as usize;
            if tx_rate > 9 {
                return Err(phy_err(format!("mt76x0 TSSI: bad HT rate {tx_rate}")));
            }
            (
                cur_power.saturating_add(rates.ht[tx_rate]),
                get_rf_pa_mode(bus, 1, tx_rate as u32)? as i8,
            )
        }
    };
    Ok((target_power, target_pa_power))
}

/// `mt76x0_phy_lin2db` (`mt76x0/phy.c:667`) — a fixed-point linear→dB conversion.
///
/// Normalises `val << 4` into `[2^15, 2^16)` while tracking the exponent, applies
/// a two-segment piecewise-linear log approximation, then scales by
/// `6 + 2^-6 + 2^-7 ≈ 6.0234` and shifts down by 10. Returns `-10000` for an
/// out-of-range input, upstream's sentinel. Every constant is upstream's; no
/// attempt is made to explain 47104 / 38400 / 23040.
///
/// **Units.** One octave of input is exactly `2^15` of `ret` before the scale, so
/// a doubling (6.0206 dB) moves the result by `32768 * 6.0234 / 1024 ≈ 193`
/// counts — i.e. the output is **1/32 dB**, confirmed by the unit test below.
/// That matters downstream: [`get_delta_power`] multiplies this by the EEPROM's
/// `tssi_slope` byte to reach its 1/8192 dB working unit, which puts the nominal
/// slope at `8192 / 32 = 256` — just off the top of the 8-bit field, so a
/// nominal board should read ~255. That last step is an **inference** from the
/// arithmetic, not something anyone has read off a real EEPROM.
fn lin2db(val: u16) -> i16 {
    let mut mantissa = (val as u32) << 4;
    let mut exp: i32 = -4;

    while mantissa < (1 << 15) {
        mantissa <<= 1;
        exp -= 1;
        if exp < -20 {
            return -10000;
        }
    }
    while mantissa > 0xffff {
        mantissa >>= 1;
        exp += 1;
        if exp > 20 {
            return -10000;
        }
    }

    let m = mantissa as i32;
    // s(15,0)
    let mut data = if mantissa <= 47104 {
        m + (m >> 3) + (m >> 4) - 38400
    } else {
        m - (m >> 3) - (m >> 6) - 23040
    };
    data = data.max(0);

    let ret = ((15 + exp) << 15) + data;
    let ret = (ret << 2) + (ret << 1) + (ret >> 6) + (ret >> 7);
    (ret >> 10) as i16
}

/// `mt76x0_phy_get_delta_power` (`mt76x0/phy.c:696`) — turn a measured TSSI level
/// into a correction for `MT_TX_ALC_CFG_1`'s temperature-compensation field.
///
/// Works in Q12 dB (`target_power << 12`). The magic addends are upstream's own
/// comments: `29491 = 3.6 × 8192`, `4424 = 0.54 × 8192`, `6554 = 0.8 × 8192`,
/// `49152 = 6 dB × 8192`. Result is clamped to the field's signed 6-bit range,
/// −32…31.
///
/// UNVERIFIED (`phy.c:794-806`): the sign-flip damping — when the new target and
/// the stored one have opposite signs and both are inside ±4096, the correction
/// is either zeroed or accepted depending on their sum. It reads as hysteresis
/// against a dithering ALC; ported exactly.
#[allow(clippy::too_many_arguments)]
fn get_delta_power(
    bus: &dyn PhyBus,
    is_5ghz: bool,
    channel: u8,
    tx_mode: u8,
    target_power: i8,
    target_pa_power: i8,
    ltssi: i16,
    state: &mut CalState,
) -> Result<i8, FaceError> {
    let mut tssi_target = (target_power as i32) << 12;

    let (tssi_slope, tssi_offset) = if is_5ghz {
        // Seven channel bounds as a byte array at MT_EE_TSSI_BOUND1 — phy.c:711.
        let mut i = 0usize;
        while i < 7 {
            let bound = ee_byte(bus, EE_TSSI_BOUND1 + i as u16);
            if channel <= bound || bound == 0 {
                break;
            }
            i += 1;
        }
        let val = ee(bus, EE_TSSI_SLOPE_5G + (i as u16) * 2);
        let mut off = (val >> 8) as i32;
        if (64..=127).contains(&off) || (off & 0x80) != 0 {
            off -= 0x100;
        }
        ((val & 0xff) as i32, off)
    } else {
        let val = ee(bus, EE_TSSI_SLOPE_2G);
        let mut off = (val >> 8) as i32;
        if off & 0x80 != 0 {
            off -= 0x100;
        }
        ((val & 0xff) as i32, off)
    };

    match target_pa_power {
        1 => {
            if !is_5ghz {
                tssi_target += 29491; // 3.6 * 8192
            }
            // fallthrough to 0: nothing more
        }
        0 => {}
        _ => tssi_target += 4424, // 0.54 * 8192
    }

    if tx_mode == 0 {
        // phy.c:747. The is_mt7630 && mmio arm is dropped.
        let data = bus.rr(BBP_CORE_1)?;
        if data & (1 << 5) != 0 {
            tssi_target += 6554; // 0.8 * 8192
        }
    }

    let data = bus.rr(BBP_TXBE_4)?;
    match data & 0x3 {
        1 => tssi_target -= 49152, // -6 dB * 8192
        2 => tssi_target -= 98304, // -12 dB * 8192
        3 => tssi_target += 49152, // +6 dB * 8192
        _ => {}
    }

    // ⚠ Upstream passes `ltssi - dev->cal.tssi_dc` (an `s16`) straight into a
    // `u16` parameter (`phy.c:776`), so a below-DC reading wraps to a huge value
    // rather than clamping. Reproduced exactly: the saturation tests further
    // down are written against that behaviour, and clamping here would change
    // which branch of them fires.
    let lin = ltssi.wrapping_sub(state.tssi_dc) as u16;
    let mut tssi_db = (lin2db(lin) as i32) * tssi_slope;
    if is_5ghz {
        tssi_db += (tssi_offset - 50) << 10; // offset s4.3
        tssi_target -= tssi_db;
        if ltssi > 254 && tssi_target > 0 {
            tssi_target = 0; // upper saturate
        }
    } else {
        tssi_db += tssi_offset << 9; // offset s3.4
        tssi_target -= tssi_db;
        // upper-lower saturate
        if (ltssi > 126 && tssi_target > 0) || ((ltssi - state.tssi_dc) < 1 && tssi_target < 0) {
            tssi_target = 0;
        }
    }

    if (state.tssi_target ^ tssi_target) < 0
        && state.tssi_target > -4096
        && state.tssi_target < 4096
        && tssi_target > -4096
        && tssi_target < 4096
    {
        if (tssi_target < 0 && tssi_target + state.tssi_target > 0)
            || (tssi_target > 0 && tssi_target + state.tssi_target <= 0)
        {
            tssi_target = 0;
        } else {
            state.tssi_target = tssi_target;
        }
    } else {
        state.tssi_target = tssi_target;
    }

    // round to the nearest compensation code, then to Q0
    if tssi_target > 0 {
        tssi_target += 2048;
    } else {
        tssi_target -= 2048;
    }
    tssi_target >>= 12;

    // Read back the current 6-bit signed compensation and add to it.
    let mut cur = field_get(MT_TX_ALC_CFG_1_TEMP_COMP, bus.rr(MT_TX_ALC_CFG_1)?) as i32;
    if cur & (1 << 5) != 0 {
        cur -= 1 << 6;
    }
    Ok((cur + tssi_target).clamp(-32, 31) as i8)
}

/// `mt76x0_phy_tssi_calibrate` (`mt76x0/phy.c:824`) — one iteration of the
/// closed-loop TX power correction.
///
/// Only meaningful when [`tssi_enabled`] and only after the radio has actually
/// transmitted something (the measurement is of the *last* transmitted frame).
/// Failures are the normal case on an idle radio, so upstream returns silently;
/// here they surface as `Ok(false)`.
pub fn tssi_calibrate(
    bus: &dyn PhyBus,
    channel: u8,
    rates: &RatePower,
    state: &mut CalState,
) -> Result<bool, FaceError> {
    let is_5ghz = channel > 14;
    let Ok((ltssi, info)) = tssi_adc_calibrate(bus, is_5ghz) else {
        return Ok(false);
    };

    let tx_mode = info[0] & 0x7;
    let Ok((target_power, target_pa_power)) = get_target_power(bus, tx_mode, &info, rates) else {
        return Ok(false);
    };

    let val = get_delta_power(
        bus,
        is_5ghz,
        channel,
        tx_mode,
        target_power,
        target_pa_power,
        ltssi,
        state,
    )?;
    bus.rmw(
        MT_TX_ALC_CFG_1,
        MT_TX_ALC_CFG_1_TEMP_COMP,
        field_prep(MT_TX_ALC_CFG_1_TEMP_COMP, (val as i32) as u32),
    )?;
    Ok(true)
}

/// `mt76x0_phy_temp_sensor` (`mt76x0/phy.c:1018`) — the *open*-loop alternative
/// to [`tssi_calibrate`], used on boards without closed-loop TSSI.
///
/// So yes: this part does have temperature compensation, and it takes one of two
/// forms depending on the EEPROM's `TX_ALC_EN` bit. Reads the on-die temperature
/// through the same `MT_BBP(CORE, 34/35)` measurement port with three RF
/// registers temporarily repurposed (and restored). Re-runs `MCU_CAL_VCO` on a
/// >20 °C drift and a whole [`calibrate`] on >30 °C.
///
/// Returns the temperature in °C. The `35/10` slope and `+25` offset are
/// upstream's (`phy.c:1038`) and are UNVERIFIED against any datasheet.
pub fn temp_sensor(
    bus: &dyn PhyBus,
    channel: u8,
    state: &mut CalState,
) -> Result<Option<i8>, FaceError> {
    let rf_b7_73 = rf_rr(bus, mt_rf(7, 73))?;
    let rf_b0_66 = rf_rr(bus, mt_rf(0, 66))?;
    let rf_b0_67 = rf_rr(bus, mt_rf(0, 67))?;

    rf_wr(bus, mt_rf(7, 73), 0x02)?;
    rf_wr(bus, mt_rf(0, 66), 0x23)?;
    rf_wr(bus, mt_rf(0, 67), 0x01)?;

    bus.wr(BBP_CORE_34, 0x0008_0055)?;
    let measured = if poll(bus, BBP_CORE_34, 1 << 4, 0, 20)? {
        let raw = (bus.rr(BBP_CORE_35)? & 0xff) as u8 as i8 as i32;
        let val = (35 * (raw - temp_offset(bus) as i32)) / 10 + 25;
        let val = val.clamp(-128, 127) as i8;

        if (val as i32 - state.temp_vco as i32).abs() > 20 {
            mcu_calibrate(bus, mcu_cal::VCO, channel as u32)?;
            state.temp_vco = val;
        }
        if (val as i32 - state.temp as i32).abs() > 30 {
            calibrate_with(bus, channel, false, state)?;
            state.temp = val;
        }
        Some(val)
    } else {
        bus.rmw(BBP_CORE_34, 1 << 4, 0)?;
        None
    };

    rf_wr(bus, mt_rf(7, 73), rf_b7_73)?;
    rf_wr(bus, mt_rf(0, 66), rf_b0_66)?;
    rf_wr(bus, mt_rf(0, 67), rf_b0_67)?;
    Ok(measured)
}

// ── AGC / RX gain tracking ──────────────────────────────────────────────────

/// The AGC tracking state upstream keeps in `struct mt76x02_calibration`.
///
/// Owned by the caller because this port has no timer of its own. Seed it with
/// [`init_agc_gain`] after every channel change and feed it to
/// [`update_channel_gain`] roughly once a second (upstream's
/// `MT_CALIBRATE_INTERVAL` is `HZ`, and the gain work runs at `4 * HZ`,
/// `mt76x0/phy.c:1112`).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct AgcState {
    /// `MT_BBP(AGC, 8/9)` gain as found right after a channel change.
    pub gain_init: [u8; 2],
    /// The gain currently programmed, before [`AgcState::gain_adjust`].
    pub gain_cur: [u8; 2],
    /// The VGA back-off the false-CCA loop has dialled in.
    pub gain_adjust: i8,
    /// True once [`AgcState::gain_adjust`] has hit its ceiling.
    pub lowest_gain: bool,
    /// `-1` until the first [`update_channel_gain`]; then 0/1/2 by RSSI band.
    pub low_gain: i8,
    /// The last false-CCA count read from [`MT_RX_STAT_1`].
    pub false_cca: u16,
    /// The average RSSI the caller last supplied.
    pub avg_rssi_all: i8,
}

impl Default for AgcState {
    fn default() -> Self {
        Self {
            gain_init: [0; 2],
            gain_cur: [0; 2],
            gain_adjust: 0,
            lowest_gain: false,
            low_gain: -1,
            false_cca: 0,
            avg_rssi_all: 0,
        }
    }
}

/// `mt76x02_init_agc_gain` (`mt76x02_phy.c:193`) — snapshot the AGC gain the
/// channel program left behind, which becomes the ceiling the loop works down
/// from.
pub fn init_agc_gain(bus: &dyn PhyBus) -> Result<AgcState, FaceError> {
    let g0 = field_get(MT_BBP_AGC_GAIN, bus.rr(BBP_AGC_8)?) as u8;
    let g1 = field_get(MT_BBP_AGC_GAIN, bus.rr(BBP_AGC_9)?) as u8;
    Ok(AgcState {
        gain_init: [g0, g1],
        gain_cur: [g0, g1],
        ..AgcState::default()
    })
}

/// `mt76x0_phy_set_gain_val` (`mt76x0/phy.c:1056`) — push the tracked gain into
/// the hardware. The DFS arm (`phy.c:1062`) is not ported; see the module docs.
pub fn set_gain_val(bus: &dyn PhyBus, state: &AgcState) -> Result<(), FaceError> {
    let gain = state.gain_cur[0].wrapping_sub(state.gain_adjust as u8);
    bus.rmw(
        BBP_AGC_8,
        MT_BBP_AGC_GAIN,
        field_prep(MT_BBP_AGC_GAIN, gain as u32),
    )?;
    Ok(())
}

/// `mt76x02_phy_adjust_vga_gain` (`mt76x02_phy.c:169`) — the false-CCA feedback
/// loop. Returns whether the gain changed.
///
/// Reading [`MT_RX_STAT_1`] **clears** it (MEASURED: `MT_RX_STAT_0/1` are
/// read-and-clear per window), so this is inherently a "since last call" measure
/// and the caller must not also be sampling that register for occupancy — the two
/// consumers would steal each other's counts.
pub fn adjust_vga_gain(bus: &dyn PhyBus, state: &mut AgcState) -> Result<bool, FaceError> {
    let limit: i8 = if state.low_gain > 0 { 16 } else { 4 };
    let false_cca = field_get(MT_RX_STAT_1_CCA_ERRORS, bus.rr(MT_RX_STAT_1)?) as u16;
    state.false_cca = false_cca;

    let mut changed = false;
    if false_cca > 800 && state.gain_adjust < limit {
        state.gain_adjust += 2;
        changed = true;
    } else if (false_cca < 10 && state.gain_adjust > 0)
        || (state.gain_adjust >= limit && false_cca < 500)
    {
        state.gain_adjust -= 2;
        changed = true;
    }

    state.lowest_gain = state.gain_adjust >= limit;
    Ok(changed)
}

/// `mt76x02_get_rssi_gain_thresh` (`mt76x02_phy.h:11`).
const fn rssi_gain_thresh(bw: Bandwidth) -> i8 {
    match bw {
        Bandwidth::Bw80 => -62,
        Bandwidth::Bw40 => -65,
        _ => -68,
    }
}

/// `mt76x02_get_low_rssi_gain_thresh` (`mt76x02_phy.h:24`).
const fn low_rssi_gain_thresh(bw: Bandwidth) -> i8 {
    match bw {
        Bandwidth::Bw80 => -76,
        Bandwidth::Bw40 => -79,
        _ => -82,
    }
}

/// `mt76x0_phy_update_channel_gain` (`mt76x0/phy.c:1067`) — pick a coarse gain
/// band from the average RSSI, then let [`adjust_vga_gain`] trim inside it.
///
/// `avg_rssi_all` is upstream's `mt76_get_min_avg_rssi`, i.e. the weakest station
/// we are tracking; this port has no station table, so the caller supplies it.
/// `None` uses upstream's fallback of −75 dBm (`phy.c:1076`).
pub fn update_channel_gain(
    bus: &dyn PhyBus,
    state: &mut AgcState,
    avg_rssi_all: Option<i8>,
    bw: Bandwidth,
) -> Result<(), FaceError> {
    let avg = avg_rssi_all.filter(|&v| v != 0).unwrap_or(-75);
    state.avg_rssi_all = avg;

    let low_gain = i8::from(avg > rssi_gain_thresh(bw)) + i8::from(avg > low_rssi_gain_thresh(bw));

    let gain_change = state.low_gain < 0 || ((state.low_gain & 2) ^ (low_gain & 2)) != 0;
    state.low_gain = low_gain;

    if !gain_change {
        if adjust_vga_gain(bus, state)? {
            set_gain_val(bus, state)?;
        }
        return Ok(());
    }

    state.gain_adjust = if low_gain == 2 { 0 } else { 10 };
    let gain_delta: u8 = if low_gain == 2 { 10 } else { 0 };

    state.gain_cur[0] = state.gain_init[0].wrapping_sub(gain_delta);
    set_gain_val(bus, state)?;

    // clear false CCA counters — phy.c:1098 (the read is the clear)
    let _ = bus.rr(MT_RX_STAT_1)?;
    Ok(())
}

/// `mt76x0_phy_calibration_work` (`mt76x0/phy.c:1101`) — one tick of the periodic
/// PHY maintenance, for a caller that wants to drive it.
///
/// Upstream runs this at `4 * MT_CALIBRATE_INTERVAL`, i.e. every 4 s. Doing the
/// gain update and the TSSI/temperature step costs on the order of 20–30 EP0
/// round trips (~4 ms), so a 4 s cadence is nearly free; do not run it per frame.
#[allow(clippy::too_many_arguments)]
pub fn calibration_tick(
    bus: &dyn PhyBus,
    channel: u8,
    bw: Bandwidth,
    avg_rssi_all: Option<i8>,
    rates: &RatePower,
    agc: &mut AgcState,
    cal: &mut CalState,
) -> Result<(), FaceError> {
    update_channel_gain(bus, agc, avg_rssi_all, bw)?;
    if tssi_enabled(bus) {
        tssi_calibrate(bus, channel, rates, cal)?;
    } else {
        temp_sensor(bus, channel, cal)?;
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The 40/80 MHz centre-channel derivation in [`set_channel_ext`] must agree
    /// with upstream's arithmetic on `chandef.center_freq1`
    /// (`mt76x0/phy.c:950-963`), which this port cannot use because it takes no
    /// centre frequency. Reproduce upstream's formula from the standard centre
    /// and check the two land on the same channel.
    #[test]
    fn vht80_centre_matches_upstream_arithmetic() {
        for &(first, centre) in VHT80_GROUPS {
            for k in 0..4u8 {
                let ctrl = first + 4 * k;
                // upstream: ch_group_index = (freq - freq1 + 30) / 20, with
                // freq = 5000 + 5*ctrl and freq1 = 5000 + 5*centre.
                let freq = 5000i32 + 5 * ctrl as i32;
                let freq1 = 5000i32 + 5 * centre as i32;
                let group = (freq - freq1 + 30) / 20;
                assert_eq!(group, k as i32, "group index for ch{ctrl}");
                // upstream: channel += 6 - ch_group_index * 4
                let upstream_centre = ctrl as i32 + 6 - group * 4;
                assert_eq!(upstream_centre, centre as i32, "centre for ch{ctrl}");
                // ours
                assert_eq!((ctrl - first) / 4, k);
            }
        }
    }

    /// Every 80 MHz centre must have a PLL program, or `set_channel` on VHT80
    /// fails at the last moment with a lookup miss.
    #[test]
    fn vht80_centres_have_a_frequency_plan() {
        for &(_, centre) in VHT80_GROUPS {
            assert!(
                freq_plan::lookup(centre, freq_plan::uses_sdm(centre)).is_some(),
                "no PLL plan for 80 MHz centre channel {centre}"
            );
        }
    }

    /// Same for every 40 MHz centre the conventional pairing can produce.
    #[test]
    fn ht40_centres_have_a_frequency_plan() {
        let mut checked = 0;
        for ch in 1..=14u8 {
            let centre = if ht40_secondary_above(ch) {
                ch + 2
            } else {
                ch - 2
            };
            assert!(
                freq_plan::lookup(centre, freq_plan::uses_sdm(centre)).is_some(),
                "no PLL plan for 2.4 GHz HT40 centre {centre} (control {ch})"
            );
            checked += 1;
        }
        for &ch in &[
            36u8, 40, 44, 48, 52, 56, 60, 64, 100, 104, 149, 153, 157, 161,
        ] {
            let centre = if ht40_secondary_above(ch) {
                ch + 2
            } else {
                ch - 2
            };
            assert!(
                freq_plan::lookup(centre, freq_plan::uses_sdm(centre)).is_some(),
                "no PLL plan for 5 GHz HT40 centre {centre} (control {ch})"
            );
            checked += 1;
        }
        assert_eq!(checked, 28);
    }

    /// The conventional pairing: the lower member of each standard pair takes the
    /// secondary above it, the upper member below.
    #[test]
    fn ht40_pairing_is_the_standard_one() {
        assert!(ht40_secondary_above(1));
        assert!(ht40_secondary_above(7));
        assert!(!ht40_secondary_above(8));
        assert!(!ht40_secondary_above(13));
        for &(lo, hi) in &[(36u8, 40u8), (44, 48), (52, 56), (149, 153), (157, 161)] {
            assert!(ht40_secondary_above(lo), "ch{lo} should be HT40+");
            assert!(!ht40_secondary_above(hi), "ch{hi} should be HT40-");
        }
    }

    /// [`sign_extend`] is sign-magnitude with an inverted sense, not two's
    /// complement. Pin it, because "fixing" it is the obvious wrong move.
    #[test]
    fn sign_extend_is_upstreams_odd_one() {
        // size 8: bit 7 set means positive.
        assert_eq!(sign_extend(0x85, 8), 5);
        assert_eq!(sign_extend(0x05, 8), -5);
        assert_eq!(sign_extend(0x80, 8), 0);
        assert_eq!(sign_extend(0x00, 8), 0);
    }

    /// [`s6_to_s8`], by contrast, *is* two's complement over six bits.
    #[test]
    fn s6_to_s8_is_twos_complement() {
        assert_eq!(s6_to_s8(0x00), 0);
        assert_eq!(s6_to_s8(0x1f), 31);
        assert_eq!(s6_to_s8(0x20), -32);
        assert_eq!(s6_to_s8(0x3f), -1);
        // high bits are ignored
        assert_eq!(s6_to_s8(0xff3f), -1);
    }

    /// `mt76x02_rate_power_val`: bit 7 enables, bit 6 is the positive flag.
    #[test]
    fn rate_power_val_decodes() {
        assert_eq!(rate_power_val(0x00), 0); // invalid
        assert_eq!(rate_power_val(0xff), 0); // invalid
        assert_eq!(rate_power_val(0x7f), 0); // valid but not enabled
        assert_eq!(rate_power_val(0xc4), 4); // enabled, positive
        assert_eq!(rate_power_val(0x84), -4); // enabled, negative
    }

    /// The four-lane 6-bit packing of the per-rate power registers.
    #[test]
    fn tx_power_mask_packs_six_bit_lanes() {
        assert_eq!(tx_power_mask(0, 0, 0, 0), 0);
        assert_eq!(tx_power_mask(1, 2, 3, 4), 0x0403_0201);
        // negatives keep their low six bits, as FIELD_PREP would
        assert_eq!(tx_power_mask(-1, 0, 0, 0), 0x0000_003f);
        // anything above six bits is dropped, not carried into the next lane
        assert_eq!(tx_power_mask(0x7f, 0, 0, 0), 0x0000_003f);
    }

    /// [`lin2db`] against upstream's own algorithm re-implemented independently,
    /// plus its out-of-range sentinel.
    #[test]
    fn lin2db_matches_reference() {
        // Zero can never normalise into range and must hit the sentinel.
        assert_eq!(lin2db(0), -10000);
        // Monotonic over the useful range: more linear power, more dB.
        let mut prev = i16::MIN;
        for v in [1u16, 2, 4, 8, 16, 64, 256, 1024, 4096, 16384, 65535] {
            let db = lin2db(v);
            assert!(db > prev, "lin2db({v}) = {db} not > {prev}");
            prev = db;
        }
        // A doubling is 6.0206 dB and the unit is 1/32 dB, so ~193 counts.
        let d = lin2db(2048) - lin2db(1024);
        assert!(
            (190..=196).contains(&d),
            "one octave gave {d} counts, want ~193"
        );
        // The same everywhere on the curve, not just at one point.
        assert_eq!(lin2db(2048) - lin2db(1024), lin2db(16384) - lin2db(8192));
    }

    /// [`RatePower`] walks all 30 entries in upstream's `all[]` order.
    #[test]
    fn rate_power_offset_and_limit_cover_every_entry() {
        let mut t = RatePower::default();
        t.add_offset(5);
        assert_eq!(t.iter().count(), 30);
        assert!(t.iter().all(|v| v == 5));
        assert_eq!(t.max(), 5);
        t.limit(3);
        assert!(t.iter().all(|v| v == 3));
        // max() seeds at 0, so an all-negative table reports 0 (upstream's
        // behaviour, not a bug in this port).
        let mut neg = RatePower::default();
        neg.add_offset(-4);
        assert_eq!(neg.max(), 0);
    }

    /// The `MT_EXT_CCA_CFG` slot-0 program must match what the kernel monitor
    /// leaves on the live part (MEASURED `0x0000f1e4`; the `0xf000` nibble is the
    /// separately-set ED_CCA mask, so compare the low twelve bits).
    #[test]
    fn ext_cca_slot0_matches_measured_hardware() {
        assert_eq!(EXT_CCA_CHAN[0] & 0x0fff, 0x1e4);
        assert_eq!(
            crate::mt76::regs::measured::KERNEL_MONITOR_EXT_CCA_CFG & 0x0fff,
            EXT_CCA_CHAN[0] & 0x0fff
        );
    }

    /// The BBP addresses this file resolves by hand.
    #[test]
    fn bbp_addresses_resolve() {
        assert_eq!(BBP_CORE_0, 0x2000);
        assert_eq!(BBP_CORE_1, 0x2004);
        assert_eq!(BBP_CORE_4, 0x2010);
        assert_eq!(BBP_CORE_34, 0x2088);
        assert_eq!(BBP_CORE_35, 0x208c);
        assert_eq!(BBP_IBI_9, 0x2124);
        assert_eq!(BBP_AGC_0, 0x2300);
        assert_eq!(BBP_AGC_8, 0x2320);
        assert_eq!(BBP_AGC_9, 0x2324);
        assert_eq!(BBP_TXBE_0, 0x2700);
        assert_eq!(BBP_TXBE_4, 0x2710);
        assert_eq!(BBP_TXBE_5, 0x2714);
        assert_eq!(BBP_TXBE_6, 0x2718);
    }

    /// Every RF address this file writes must be inside the CSR path's addressing
    /// limits (`bank <= 8`, `reg <= 127`), or the opt-in CSR path would reject a
    /// write the MCU path accepts and the two paths would not be interchangeable.
    #[test]
    fn hand_written_rf_addresses_are_csr_addressable() {
        let used = [
            mt_rf(0, 4),
            mt_rf(0, 22),
            mt_rf(0, 24),
            mt_rf(0, 26),
            mt_rf(0, 27),
            mt_rf(0, 28),
            mt_rf(0, 29),
            mt_rf(0, 30),
            mt_rf(0, 31),
            mt_rf(0, 32),
            mt_rf(0, 33),
            mt_rf(0, 34),
            mt_rf(0, 35),
            mt_rf(0, 36),
            mt_rf(0, 37),
            mt_rf(0, 66),
            mt_rf(0, 67),
            mt_rf(0, 73),
            mt_rf(5, 0),
            mt_rf(5, 2),
            mt_rf(6, 0),
            mt_rf(7, 73),
        ];
        for addr in used {
            assert!(mt_rf_bank(addr) <= 8, "bank of {addr:#x}");
            assert!(mt_rf_reg(addr) <= 127, "reg of {addr:#x}");
        }
    }

    /// The mt76x0 calibration ids are *not* the mt76x2 ones. Pin the two that
    /// differ, since a silent mismatch would ask the firmware for the wrong
    /// calibration.
    #[test]
    fn mcu_cal_ids_are_the_mt76x0_enum() {
        assert_eq!(mcu_cal::R, 1);
        assert_eq!(mcu_cal::RXDCOC, 2); // mt76x2 calls this 3
        assert_eq!(mcu_cal::LC, 3); // mt76x2 calls this 6
        assert_eq!(mcu_cal::VCO, 12);
        assert_eq!(mcu_cal::FULL, 0xff);
    }

    /// The AGC gain-band thresholds, per bandwidth.
    #[test]
    fn agc_thresholds() {
        assert_eq!(rssi_gain_thresh(Bandwidth::Bw20), -68);
        assert_eq!(rssi_gain_thresh(Bandwidth::Bw40), -65);
        assert_eq!(rssi_gain_thresh(Bandwidth::Bw80), -62);
        assert_eq!(low_rssi_gain_thresh(Bandwidth::Bw20), -82);
        assert_eq!(low_rssi_gain_thresh(Bandwidth::Bw40), -79);
        assert_eq!(low_rssi_gain_thresh(Bandwidth::Bw80), -76);
    }
}
