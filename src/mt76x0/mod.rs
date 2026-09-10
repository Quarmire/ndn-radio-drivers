//! Userspace libusb backend for the **MT7610U** (`0e8d:7610`, `mt76x0u`) — MediaTek's
//! **1×1 dual-band 802.11ac** dongle, and the second member of the shared
//! [`mt76x02`](crate::mt76) family in this crate beside the 2×2
//! [`MT7612U`](crate::Mt7612uBackend).
//!
//! Where the mt7612 backend is a *replay* of a captured kernel init stream, this one is a
//! **port**: every register write here comes from the upstream GPL `mt76` tree with the
//! `file:line` beside it, so a channel this driver has never seen still tunes. The two
//! share [`crate::mt76::regs`] because they are literally the same MAC/BBP block.
//!
//! # MEASURED vs CODE-READ
//!
//! Everything under "MEASURED" came off **mds-o5p-1's MT7610U on 2026-08-27** via
//! `examples/mt76_oracle.rs`, on the target silicon, with the kernel `mt76x0u` driver
//! holding the part in monitor mode. Everything else is a transcription of upstream and
//! has **not** been confirmed against hardware. Reasoning about these radios has a ~0% hit
//! rate here and measurement ~100%, so the distinction is kept in the code, not in prose
//! somewhere else.
//!
//! **MEASURED, and this file agrees with all of it:**
//!   * **Endpoints** (interface 0, class `ff/02/ff`): bulk IN `0x84` (`MT_EP_IN_PKT_RX`),
//!     `0x85` (`MT_EP_IN_CMD_RESP`); bulk OUT `0x04`..`0x09` = `MT_EP_OUT_INBAND_CMD`,
//!     `AC_BE`, `AC_BK`, `AC_VI`, `AC_VO`, `HCCA` in `enum mt76u_out_ep` order
//!     (`mt76.h:652-660`). All 512 B (high speed). TX therefore goes out on **`0x05`**
//!     (AC_BE), not on the command pipe.
//!   * **`MT_TSF_TIMER_DW0` (0x111c) is the LOW word and ticks at 1.000 MHz** once
//!     `MT_BEACON_TIME_CFG` bit 16 (`TIMER_EN`) is set and `SYNC_MODE [18:17]` is cleared.
//!     As found under a kernel monitor it reads a static 0 because `TIMER_EN` is clear —
//!     which is why [`enable_tsf`](Mt7610uBackend::enable_tsf) exists and why
//!     [`setup_monitor_rx`](Mt7610uBackend::setup_monitor_rx) calls it.
//!     (`mt76x02_usb_core.c:155-157` assembles the TSF as `dw0 << 32 | dw1`, i.e. it treats
//!     DW0 as the HIGH word. That is **wrong on this silicon**; it feeds a `dev_dbg` and is
//!     validated by nothing. The correct assembly lives in [`crate::mt76::knobs`].)
//!   * **`MT_CH_IDLE` / `MT_CH_BUSY` are read-and-clear microsecond counters** —
//!     `(idle + busy) / elapsed` measured 1.00 over 100 ms windows. So
//!     [`read_channel_activity`](RadioKnobs::read_channel_activity) returns a *level* for
//!     the window since the last read, not a free-running count to difference. See the note
//!     on that method: it is a deliberate departure from the trait's usual contract, forced
//!     by the hardware.
//!   * **`MT_RX_STAT_0` / `_1` are read-and-clear per window** too.
//!   * **`MT_USB_DMA_CFG` is plain MMIO at `0x0238` on this part** (the mt76x2's CFG-space
//!     `0x9018` reads 0 here) and lives at `0x00c00000` under the kernel: `TX_BULK_EN` and
//!     `RX_BULK_EN` set, **`RX_BULK_AGG_EN` (bit 21) CLEAR**. Upstream clears it
//!     deliberately — *"disable AGGR_BULK_RX in order to receive one frame in each rx urb
//!     and avoid copies"* (`mt76x0/usb.c:55-58`) — so **one RX unit per bulk-IN transfer**
//!     is a measured fact, not an open question. [`parse_transfer`] still walks the DMA
//!     length field so a device that ever *did* aggregate would be handled rather than
//!     silently truncated; it simply never loops twice today.
//!   * **The USB EEPROM shadow is populated**: 512/512 bytes over `MT_VEND_READ_EEPROM`,
//!     word 0 = `0x7610`, bytes 4..9 = the netdev MAC exactly. No `MT_EFUSE_CTRL` port is
//!     needed, and the read works **before** any firmware download — which is why
//!     [`open`](Mt7610uBackend::open) parses the EEPROM rather than
//!     [`bring_up`](Mt7610uBackend::bring_up).
//!   * **The direct RF CSR path works over USB.** `MT_RF_CSR_CFG` reads returned exactly
//!     the values in `mt76x0_rf_central_tab` plus the `rf_bw_switch_tab` entry matching the
//!     interface's live channel/width. Upstream branches on *bus* rather than on capability
//!     (`mt76x0/phy.c:86-97` routes USB through the MCU register-pair path), so this had
//!     never been settled. [`PhyBus::rf_wr`]/[`PhyBus::rf_rr`] therefore use the direct
//!     path — one EP0 round trip instead of an MCU command — and
//!     [`PhyBus::mcu_wr_rp`] is still there for code that prefers upstream's route.
//!   * **EP0 costs 151 µs per vendor-request round trip.** Nothing on a per-frame path may
//!     read a register. That is why the TX and RX hot paths in this file touch only bulk
//!     endpoints, and why [`RadioTime::time_sources`] declares **151 µs**, not 1 µs, as the
//!     precision of the port TSF: the tick is 1 µs but *the read* is what costs, and the
//!     read is the only way to obtain a value.
//!   * Kernel-monitor golden reference: `MT_RX_FILTR_CFG = 0x0000_1093`,
//!     `MT_MAC_SYS_CTRL = 0x0c`, `MT_EXT_CCA_CFG = 0x0000_f1e4`,
//!     `MT_TXOP_CTRL_CFG = 0x0000_583f`, `MT_BBP(AGC,2) = 0x003a_6464`,
//!     `MT_BKOFF_SLOT_CFG = 0x0000_0209`. Recorded as constants in
//!     [`crate::mt76::regs::measured`].
//!
//! **CODE-READ, unvalidated on silicon:** the bring-up ordering, the firmware download, the
//! MAC/BBP/RF init tables, the TXWI layout, the per-frame RSSI calibration, and every
//! number in [`initvals`], [`initvals_phy`], [`freq_plan`]. `examples/mt7610_bringup.rs` is
//! the gate that turns those into measurements.
//!
//! # What this part is
//!
//! **1×1.** One spatial stream, HT MCS 0-7. It is 11ac silicon (VHT-1SS), but no VHT
//! transmit has been confirmed here, so [`declared_capability`] says HT-7/1SS/20 MHz and
//! nothing more. An over-declared capability is worse than a modest one — the planner
//! believes it, and the worst-receiver rate cap aims traffic at whatever a radio claims.
//!
//! # Deliberate omissions, so they are visible rather than lost
//!
//!   * **40/80 MHz.** The RF and BBP switch tables carry `RF_BW_40`/`RF_BW_80` rows and
//!     `mt76x0_phy_set_channel` handles both, but it needs the *control-channel offset*
//!     (`chandef->center_freq1`, `mt76x0/phy.c:949-968`) to pick `ch_group_index`, and the
//!     [`RadioKnobs::set_channel`] seam carries only `(channel, bw)`. Rather than guess an
//!     offset, [`set_channel`](RadioKnobs::set_channel) long **rejected** anything but 20 MHz.
//!     ★ RESOLVED 2026-08-31: `ht40_secondary_above`/`VHT80_GROUPS` derive that offset, and a
//!     WITNESS RECEIVER confirmed 20/40/80 MHz PPDUs on air (100% of frames in each arm), so the
//!     refusal is gone and [`declared_capability`] reports `max_bw: 2`. Declaration and actuator
//!     still agree — they now agree at 80 MHz.
//!   * **The WCID / shared-key table wipe** (`mt76x0/init.c:198-203`: 64 shared keys +
//!     256 WCIDs). ~1300 EP0 round trips ≈ 0.2 s, and nothing in this driver's RX or TX
//!     path reads the WCID table — TX uses the no-station WCID `0xff` and RX is
//!     promiscuous. Skipped, and said so rather than silently dropped.
//!   * **Beacon config / pre-TBTT** (`mt76x02u_init_beacon_config`). We never beacon; we do
//!     set `MT_BEACON_TIME_CFG`'s `TIMER_EN` ourselves, which is the only part of that
//!     block the TSF needs.
//!   * **`PhyMetrics`.** Left `None`. The RXWI's four `bbp_rxinfo` dwords
//!     (`mt76x02_mac.h:107`, transfer offsets 20..36) are the only plausible source of
//!     SNR/EVM/CFO on this part and **nothing upstream reads them**. Manufacturing a field
//!     out of an undecoded dword is precisely the defect this codebase spends its time
//!     removing; `examples/mt7610_bringup.rs` stage 7 dumps those bytes to settle it.
#![allow(dead_code)]

use std::sync::Arc;
use std::sync::Mutex;
use std::sync::atomic::{AtomicU8, AtomicU16, AtomicU64, Ordering};
use std::time::Duration;

use async_trait::async_trait;
use rusb::{Context, DeviceHandle};

use ndn_radio_hal::bringup::{
    AppliedPower, Assert, BringUp, BringUpFailure, BringUpReport, Ctx, Degradation, Deviation,
    Fact, Guards, Plan, PlanId, PlanRun, PowerReference, PowerRequest, PowerWrite,
    ProofRequirement, PumpPolicy, RadioState, Role, Severity, Stage, Step, StepClass, StepId,
    StepOutcome,
};
use ndn_radio_hal::{
    Band, Bandwidth, CsiSupport, RadioCapability, RadioKind, RadioKnobs, RadioProfile, RadioTime,
    RateCapability, TxDiscipline,
};
use ndn_time::{ClockDomainId, RadioTimeSource};

use crate::mt76::{Family, Mt76Regs, knobs, regs, transport::Mt76Usb};
use crate::usb_select::DeviceSelect;
use crate::{CapturedFrame, FaceError, FrameFormat, FrameIo, InjectFrame, McsDescriptor};

pub mod eeprom;
pub mod freq_plan;
pub mod initvals;
pub mod initvals_phy;
pub mod mcu;
pub mod phy;

use eeprom::Mt76x0Eeprom;
use mcu::{McuBus, RegPair};
use phy::PhyBus;

// ── Identity ────────────────────────────────────────────────────────────────

/// MediaTek's USB vendor id. The MT7610U also ships behind `0x148f` (Ralink) and a dozen
/// OEM ids (`mt76x0/usb.c:14-43`); this port targets the `0e8d:7610` on mds-o5p-1, which is
/// the part every measurement in this file was taken on.
pub const MEDIATEK_VID: u16 = 0x0e8d;

/// MT7610U in Wi-Fi mode. Deliberately **one** id: the sibling ids upstream lists
/// (`0x7630`, `0x7650`, the Ralink-vendor clones) are the same silicon but have never been
/// in this lab, and a PID list is a claim about what has been tried.
pub const MT7610U_PIDS: &[u16] = &[0x7610];

/// Vendored MCU firmware (`linux-firmware`'s `mt7610u.bin`; see `fw/mt76x0/`).
/// `mt76x0/usb_mcu.c:67-83` prefers `mt7610e.bin` and falls back to this one — the USB
/// image is the correct choice for a USB part, so there is only one blob here.
const MT7610U_FIRMWARE: &[u8] = include_bytes!("../../fw/mt76x0/mt7610u.bin");

// ── USB framing constants (dma.h:46-49) ─────────────────────────────────────

/// `MT_DMA_HDR_LEN` — the 4-byte USB DMA header in front of every RX unit. Its low 16 bits
/// are `dma_len`: the byte count of everything after it (RXWI + MPDU + FCE trailer), always
/// a multiple of 4 (`usb.c:471-480`).
const MT_DMA_HDR_LEN: usize = 4;
/// `MT_RX_RXWI_LEN` — `struct mt76x02_rxwi` (`mt76x02_mac.h:97-108`).
const MT_RX_RXWI_LEN: usize = 32;
/// `MT_FCE_INFO_LEN` — the 4-byte FCE trailer after the MPDU.
const MT_FCE_INFO_LEN: usize = 4;
/// Bytes of descriptor before the 802.11 header: DMA header + RXWI = 36.
const RXD_LEN: usize = MT_DMA_HDR_LEN + MT_RX_RXWI_LEN;

// Absolute offsets of the RXWI fields **within the bulk-IN transfer** (i.e. RXWI offset
// plus `MT_DMA_HDR_LEN`), from `struct mt76x02_rxwi`:
//   __le32 rxinfo; __le32 ctl; __le16 tid_sn; __le16 rate; u8 rssi[4]; __le32 bbp_rxinfo[4];
const RXWI_RXINFO: usize = 4;
const RXWI_CTL: usize = 8;
const RXWI_RATE: usize = 14;
const RXWI_RSSI0: usize = 16;
/// The four undecoded `bbp_rxinfo` dwords — see the module header's `PhyMetrics` note.
const RXWI_BBP_RXINFO: usize = 20;

/// `struct mt76x02_txwi` (`mt76x02_mac.h:135-148`) is 20 bytes:
/// `flags:2 rate:2 ack_ctl:1 wcid:1 len_ctl:2 iv:4 eiv:4 aid:1 txstream:1 ctl2:1 pktid:1`.
const TXWI_LEN: usize = 20;

// `enum mt76u_out_ep` / `enum mt76u_in_ep` ordinals (`mt76.h:646-660`). The transport
// resolves an ordinal to the descriptor's actual endpoint address, so nothing here hard-codes
// `0x05` and a device that enumerates its pipes in a different order still works.
const EP_OUT_INBAND_CMD: usize = 0;
const EP_OUT_AC_BE: usize = 1;
/// The four access-category bulk-OUT endpoints, mt76 indices 1..=4 (BE, BK, VI, VO).
///
/// ★ Why this exists: MEASURED on the sibling MT7612U, the per-frame cost is a rock-steady
/// **~290 µs** that is *not* airtime (the variable part tracks airtime exactly), *not* host USB
/// dispatch (eight pipelined writer threads do not move it), and *not* DCF backoff (removing it
/// made throughput 5.7x worse). One thing it could still be is per-queue serialisation in the
/// device's TX DMA/MAC — and every mt76 backend here has always used **one** endpoint, AC_BE,
/// so that hypothesis has never been tested. Four hardware queues, four bulk pipes.
///
/// The arithmetic that makes it matter: at ~3000 PPDU/s, 400 Mbit/s needs 16.7 kB per frame,
/// which exceeds the 11454 B VHT MPDU limit. A single MPDU per PPDU cannot reach it at this
/// frame rate, so either the PPDU rate rises (this) or MPDUs share a PPDU (aggregation).
const EP_OUT_ACS: [usize; 4] = [1, 2, 3, 4];
const EP_IN_PKT_RX: usize = 0;
const EP_IN_CMD_RESP: usize = 1;

/// `CMD_FUN_SET_OP` — `enum mcu_cmd` (`mt76x02_mcu.h:30-52`).
const MCU_CMD_FUN_SET_OP: u8 = 1;
/// `Q_SELECT` — `enum mcu_function` (`mt76x02_mcu.h:62-69`). Selects the firmware's
/// transmit-queue mapping. Kept local rather than reached for in `mcu.rs`: the MCU module's
/// contract is the transport (`mcu_send`/`wr_rp`/`load_firmware`), and which *command* a
/// caller sends is the caller's business.
const MCU_FUNC_Q_SELECT: u32 = 1;

/// TX bulk timeout. Generous: a stalled AC queue is a real condition, not a fast failure.
const BULK_TX_TIMEOUT: Duration = Duration::from_secs(1);
/// One-shot RX read timeout when no pump is running (`recv_frame`'s slow path).
const BULK_RX_TIMEOUT: Duration = Duration::from_millis(200);
/// MCU command-response read window — upstream's `mt76u_bulk_msg(..., 500, ...)`
/// (`mt76x02_usb_mcu.c:95`).
const MCU_RESP_TIMEOUT: Duration = Duration::from_millis(500);

fn io_err(what: String) -> FaceError {
    FaceError::Io(std::io::Error::other(what))
}

fn usb_err(e: rusb::Error) -> FaceError {
    FaceError::Io(std::io::Error::other(format!("mt7610u usb: {e}")))
}

// ── Rate words ──────────────────────────────────────────────────────────────

/// `enum mt76_phy_type` (`mt76.h:338-343`) — the TXWI/RXWI rate word's PHY field.
const MT_PHY_TYPE_CCK: u16 = 0;
const MT_PHY_TYPE_OFDM: u16 = 1;
const MT_PHY_TYPE_HT: u16 = 2;
const MT_PHY_TYPE_HT_GF: u16 = 3;
const MT_PHY_TYPE_VHT: u16 = 4;

/// A legacy (non-HT) transmit rate, as the **worst-receiver doctrine** needs one.
///
/// Broadcast has no ACK and therefore no rate feedback, so a frame that every neighbour
/// must decode — discovery, reception reports, anything advertising capability — has to go
/// out at a rate no receiver can fail to demodulate for want of an HT decoder. That is what
/// this enum is for; it is not a throughput knob.
///
/// Values are the `hw_value`s of `mt76x02_rates` (`mt76x02_util.c:10-30`): CCK indices 0-3
/// with `MT_PHY_TYPE_CCK`, OFDM indices 0-7 with `MT_PHY_TYPE_OFDM`. (Upstream's
/// `+= 4` on 2.4 GHz in `mt76x02_mac_process_rate` is a *mac80211 rate-table* index fixup on
/// the report path, not part of the hardware value — do not apply it here.)
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum LegacyRate {
    /// DSSS 1 Mbps, long preamble — 2.4 GHz only.
    Cck1,
    /// DSSS 2 Mbps.
    Cck2,
    /// CCK 5.5 Mbps.
    Cck5_5,
    /// CCK 11 Mbps.
    Cck11,
    /// OFDM 6 Mbps — the universally-decodable default on both bands, and the only legacy
    /// rate that exists on 5 GHz alongside its faster siblings.
    Ofdm6,
    /// OFDM 9 Mbps.
    Ofdm9,
    /// OFDM 12 Mbps.
    Ofdm12,
    /// OFDM 18 Mbps.
    Ofdm18,
    /// OFDM 24 Mbps.
    Ofdm24,
    /// OFDM 36 Mbps.
    Ofdm36,
    /// OFDM 48 Mbps.
    Ofdm48,
    /// OFDM 54 Mbps.
    Ofdm54,
}

impl LegacyRate {
    /// The TXWI rate word for this rate (BW 20, long GI, no STBC/LDPC — none of which
    /// apply to a legacy PPDU).
    pub const fn rate_val(self) -> u16 {
        let (phy, idx): (u16, u16) = match self {
            LegacyRate::Cck1 => (MT_PHY_TYPE_CCK, 0),
            LegacyRate::Cck2 => (MT_PHY_TYPE_CCK, 1),
            LegacyRate::Cck5_5 => (MT_PHY_TYPE_CCK, 2),
            LegacyRate::Cck11 => (MT_PHY_TYPE_CCK, 3),
            LegacyRate::Ofdm6 => (MT_PHY_TYPE_OFDM, 0),
            LegacyRate::Ofdm9 => (MT_PHY_TYPE_OFDM, 1),
            LegacyRate::Ofdm12 => (MT_PHY_TYPE_OFDM, 2),
            LegacyRate::Ofdm18 => (MT_PHY_TYPE_OFDM, 3),
            LegacyRate::Ofdm24 => (MT_PHY_TYPE_OFDM, 4),
            LegacyRate::Ofdm36 => (MT_PHY_TYPE_OFDM, 5),
            LegacyRate::Ofdm48 => (MT_PHY_TYPE_OFDM, 6),
            LegacyRate::Ofdm54 => (MT_PHY_TYPE_OFDM, 7),
        };
        (phy << 13) | idx
    }

    /// Nominal PHY rate in units of 100 kbit/s (`mt76x02_rates`' `bitrate`) — for logging
    /// and airtime estimates, never for a decision the hardware should make.
    pub const fn bitrate_100kbps(self) -> u16 {
        match self {
            LegacyRate::Cck1 => 10,
            LegacyRate::Cck2 => 20,
            LegacyRate::Cck5_5 => 55,
            LegacyRate::Cck11 => 110,
            LegacyRate::Ofdm6 => 60,
            LegacyRate::Ofdm9 => 90,
            LegacyRate::Ofdm12 => 120,
            LegacyRate::Ofdm18 => 180,
            LegacyRate::Ofdm24 => 240,
            LegacyRate::Ofdm36 => 360,
            LegacyRate::Ofdm48 => 480,
            LegacyRate::Ofdm54 => 540,
        }
    }
}

/// Encode an [`McsDescriptor`] into the mt76x02 TXWI rate word (`__le16` at TXWI offset 2).
///
/// Layout, `mt76x02_mac.h:86-92`:
/// `index[5:0] | LDPC[6] | BW[8:7] | SGI[9] | STBC[10] | LDPC_EXSYM[11] | PHY[15:13]`.
/// Built exactly as `mt76x02_mac_tx_rate_val` (`mt76x02_mac.c:180-226`) builds it, plus the
/// STBC/LDPC bits `mt76x02_mac_write_txwi` ORs in afterwards (`:403-406`).
///
/// ★ **This part is 1×1**, so the descriptor is clamped rather than trusted:
///   * HT indices are clamped to **0-7**. MCS 8-15 *are* a 2-stream rate on this encoding
///     (`nss = 1 + (idx >> 3)`, `mt76x02_mac.c:196`) — asking a one-chain radio for one
///     produces a PPDU it cannot build.
///   * VHT `nss` is clamped to 1 (`index[5:4] = nss - 1`, `MT_RATE_INDEX_VHT_NSS`).
///   * `stbc` is dropped: `mt76x02_mac.c:405` only sets it when `nss == 1`, but STBC needs
///     **two** transmit chains to Alamouti-encode across — a single-chain part has nothing
///     to spread the stream over, so honouring the flag here would set a bit that the
///     baseband cannot act on and the receiver would be told to expect.
///   * `ldpc` is dropped: `mt76x02_mac.c:403` gates the LDPC bit on `is_mt76x2(dev)`.
///     Upstream states no reason and neither can we — ported faithfully.
///
/// ★ **`bw_code` fills `MT_RXWI_RATE_BW`, bits [8:7]** (`mt76x02_mac.h:88`), values from
/// `enum mt76x2_phy_bandwidth`: 0 = 20, 1 = 40, 2 = 80 MHz.
///
/// ☠ **This field was left at 0 and that made the width knob a no-op.** Bandwidth on an mt76x02
/// part has TWO actuators — the channel program (BBP/RF) and this rate word — and upstream sets
/// both from one chandef (`mt76x02_mac_tx_rate_val`). Widening only the channel transmits **20 MHz
/// PPDUs on a wider channel**, which is exactly why a historical Bw80 run measured 2719 f/s against
/// Bw20's 2732 and was read as "80 MHz buys nothing". The channel moved; the transmitter did not.
///
/// Clamped by PHY type, as upstream clamps it: legacy has no wide PPDU (always 0), HT has no
/// 80 MHz (max 1), VHT may use all three. A rate word wider than the tuned baseband is a malformed
/// PPDU, so callers pass the width the radio is *currently* tuned to.
const fn rate_bw_field(phy: u16, bw_code: u8) -> u16 {
    let max = match phy {
        MT_PHY_TYPE_VHT => 2u8,
        MT_PHY_TYPE_HT => 1,
        _ => 0,
    };
    let bw = if bw_code > max { max } else { bw_code };
    (bw as u16) << 7
}

fn mt76_rate_val(m: &McsDescriptor, bw_code: u8) -> u16 {
    let (phy, idx): (u16, u16) = if m.vht {
        // VHT: index[3:0] = MCS, index[5:4] = NSS-1 (MT_RATE_INDEX_VHT_*, mt76x02_mac.h:94).
        (MT_PHY_TYPE_VHT, u16::from(m.index) & 0x0f)
    } else {
        (MT_PHY_TYPE_HT, u16::from(m.index.min(7)))
    };
    let mut v = idx | (phy << 13) | rate_bw_field(phy, bw_code);
    if m.short_gi {
        v |= 1 << 9;
    }
    v
}

/// A decoded RXWI rate word.
///
/// It exists because **the HAL has nowhere to carry two of these fields upward**:
/// [`CapturedFrame`] has `mcs_index` and `rssi_dbm`, and [`ndn_radio_hal::PhyMetrics`] has
/// SNR/EVM/CFO — neither carries channel bandwidth or the guard interval. Rather than drop
/// the two bits on the floor or invent a field, they are decoded here, exposed through
/// [`decode_rate_word`], and unit-tested without hardware.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct RxRate {
    /// PHY type: `MT_PHY_TYPE_{CCK,OFDM,HT,HT_GF,VHT}`.
    pub phy: u8,
    /// The raw 6-bit index field, before any per-PHY interpretation.
    pub index: u8,
    /// MCS index for HT (0-7 on this part) or VHT (0-9); `None` for CCK/OFDM, which have
    /// no MCS — reporting a legacy rate index as an MCS is how a legacy frame comes to look
    /// like an HT one in a rate histogram.
    pub mcs: Option<u8>,
    /// Spatial streams, for VHT only (HT carries the stream count inside the index).
    pub nss: u8,
    /// Channel bandwidth code, `enum mt76x2_phy_bandwidth` (`mt76x02_mac.h:112-116`):
    /// 0 = 20, 1 = 40, 2 = 80 MHz.
    pub bw: u8,
    /// 400 ns short guard interval.
    pub short_gi: bool,
    /// Space-time block coding.
    pub stbc: bool,
    /// LDPC FEC instead of BCC.
    pub ldpc: bool,
}

/// Decode an RXWI/TXWI rate word — the inverse of [`mt76_rate_val`], following
/// `mt76x02_mac_process_rate` (`mt76x02_mac.c:654-720`).
pub const fn decode_rate_word(v: u16) -> RxRate {
    let phy = ((v >> 13) & 0x7) as u8;
    let index = (v & 0x3f) as u8;
    let (mcs, nss) = match phy as u16 {
        MT_PHY_TYPE_HT | MT_PHY_TYPE_HT_GF => (Some(index), 1),
        // VHT splits the index: MCS in [3:0], NSS-1 in [5:4] (mt76x02_mac.h:94-95).
        MT_PHY_TYPE_VHT => (Some(index & 0x0f), ((index >> 4) & 0x3) + 1),
        _ => (None, 1),
    };
    RxRate {
        phy,
        index,
        mcs,
        nss,
        bw: ((v >> 7) & 0x3) as u8,
        short_gi: v & (1 << 9) != 0,
        stbc: v & (1 << 10) != 0,
        ldpc: v & (1 << 6) != 0,
    }
}

// ── RX descriptor decode (pure, so it is testable with no dongle) ────────────

/// `MT_RXINFO_*` bits used here (`mt76x02_mac.h:46-74`).
const MT_RXINFO_CRCERR: u32 = 1 << 8;
const MT_RXINFO_L2PAD: u32 = 1 << 14;
/// `MT_RXWI_CTL_MPDU_LEN` = `GENMASK(29, 16)` (`mt76x02_mac.h:80`).
const MT_RXWI_CTL_MPDU_LEN: u32 = 0x3fff_0000;

/// One RX unit's descriptor, decoded but not yet assembled into a frame.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
struct RxUnit {
    /// Bytes to advance to reach the next unit in the transfer (`4 + dma_len`).
    unit_len: usize,
    /// MPDU length from `MT_RXWI_CTL_MPDU_LEN`, **excluding** the L2 pad (upstream subtracts
    /// it before trimming — `mt76x02_mac.c:825,834`).
    mpdu_len: usize,
    /// 802.11 header length derived from the frame-control bits.
    hdr_len: usize,
    /// 0, or 2 when `MT_RXINFO_L2PAD` says the hardware inserted alignment padding between
    /// the 802.11 header and the body.
    pad: usize,
    /// Raw TXWI/RXWI rate word.
    rate: u16,
    /// `rxwi->rssi[0]` — the raw per-chain RSSI byte, before the EEPROM's LNA/offset
    /// calibration turns it into dBm.
    rssi_raw: u8,
    /// The hardware says this MPDU failed its FCS check.
    crc_err: bool,
}

/// 802.11 header length from the frame-control octets — the shape of `ieee80211_hdrlen`.
///
/// Needed only to place the L2 pad, which sits *between* the header and the body, so a
/// wrong answer misaligns the payload rather than merely mislabelling it. Base 24, plus
/// `addr4` when ToDS and FromDS are both set, plus QoS Control on a QoS-Data subtype, plus
/// HT Control on the Order/+HTC bit (data frames: QoS only; management: any).
const fn dot11_hdr_len(fc0: u8, fc1: u8) -> usize {
    let ftype = (fc0 >> 2) & 0x3;
    let subtype = fc0 >> 4;
    match ftype {
        // Control: only the three long-header subtypes matter for alignment.
        1 => match subtype {
            0xc | 0xd => 10, // CTS, ACK
            _ => 16,         // PS-Poll, RTS, CF-End, BlockAck(Req)
        },
        // Data.
        2 => {
            let mut n = 24;
            if fc1 & 0x03 == 0x03 {
                n += 6; // addr4
            }
            if subtype & 0x08 != 0 {
                n += 2; // QoS Control
                if fc1 & 0x80 != 0 {
                    n += 4; // HT Control (+HTC)
                }
            }
            n
        }
        // Management (and the 802.11 "extension" type, which we do not decode).
        _ => {
            if fc1 & 0x80 != 0 {
                28
            } else {
                24
            }
        }
    }
}

/// Decode the descriptor of the RX unit starting at `buf[0]`, or `None` if `buf` is too
/// short or the DMA length is malformed.
///
/// Mirrors `mt76u_get_rx_entry_len` (`usb.c:465-481`) for the length validation and
/// `mt76x02_mac_process_rx` (`mt76x02_mac.c:771-877`) for the field extraction.
fn decode_rx_unit(buf: &[u8]) -> Option<RxUnit> {
    const MIN_LEN: usize = MT_DMA_HDR_LEN + MT_RX_RXWI_LEN + MT_FCE_INFO_LEN;
    if buf.len() < MIN_LEN {
        return None;
    }
    // usb.c:471 — the DMA length is the LOW 16 bits of the header, counting everything
    // after it; it must be non-zero, 4-aligned, and fit in the transfer.
    let dma_len = u16::from_le_bytes([buf[0], buf[1]]) as usize;
    if dma_len == 0 || dma_len % 4 != 0 || dma_len + MT_DMA_HDR_LEN > buf.len() {
        return None;
    }
    let rxinfo = u32::from_le_bytes(buf[RXWI_RXINFO..RXWI_RXINFO + 4].try_into().ok()?);
    let ctl = u32::from_le_bytes(buf[RXWI_CTL..RXWI_CTL + 4].try_into().ok()?);
    let rate = u16::from_le_bytes([buf[RXWI_RATE], buf[RXWI_RATE + 1]]);
    let rssi_raw = buf[RXWI_RSSI0];

    let mpdu_len = ((ctl & MT_RXWI_CTL_MPDU_LEN) >> 16) as usize;
    let pad = if rxinfo & MT_RXINFO_L2PAD != 0 { 2 } else { 0 };
    if buf.len() < RXD_LEN + 2 {
        return None;
    }
    let hdr_len = dot11_hdr_len(buf[RXD_LEN], buf[RXD_LEN + 1]);
    // A truncated or nonsense MPDU length is a descriptor we do not understand; drop the
    // unit rather than slice past the end of the frame region.
    if mpdu_len < hdr_len || RXD_LEN + mpdu_len + pad > MT_DMA_HDR_LEN + dma_len {
        return None;
    }
    Some(RxUnit {
        unit_len: MT_DMA_HDR_LEN + dma_len,
        mpdu_len,
        hdr_len,
        pad,
        rate,
        rssi_raw,
        crc_err: rxinfo & MT_RXINFO_CRCERR != 0,
    })
}

/// Assemble the bare 802.11 MPDU of `u` out of `buf`, removing the L2 pad.
///
/// `mt76x02_remove_hdr_pad` slides the header forward over the pad; the same thing, done by
/// copy because we do not own the transfer buffer. When there is no pad (our own frames
/// always have a 4-aligned header, so `L2PAD` is clear for them) this is one contiguous
/// copy.
fn rx_mpdu(buf: &[u8], u: &RxUnit) -> Option<Vec<u8>> {
    let start = RXD_LEN;
    if u.pad == 0 {
        return buf.get(start..start + u.mpdu_len).map(<[u8]>::to_vec);
    }
    let hdr = buf.get(start..start + u.hdr_len)?;
    let body_start = start + u.hdr_len + u.pad;
    let body = buf.get(body_start..body_start + (u.mpdu_len - u.hdr_len))?;
    let mut out = Vec::with_capacity(u.mpdu_len);
    out.extend_from_slice(hdr);
    out.extend_from_slice(body);
    Some(out)
}

// ── RX counters ─────────────────────────────────────────────────────────────

/// A window of the RX pump's own bookkeeping, from [`Mt7610uBackend::rx_stats_reset`].
///
/// These are **software** counts of what came off USB — they are not the hardware's
/// `MT_RX_STAT_*` registers, which [`RadioKnobs::read_ofdm_counters`] reads. Both are
/// needed: the registers say what the PHY saw, these say what actually reached the host.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct RxStats {
    /// RX units pulled off bulk-IN and successfully decoded as descriptors.
    pub units: u64,
    /// Of those, units the hardware flagged `MT_RXINFO_CRCERR`.
    pub crc_errors: u64,
    /// Units carrying an L2 alignment pad (a QoS-Data frame from someone else's network).
    pub l2pad: u64,
    /// Transfers whose descriptor did not decode — short reads, malformed DMA lengths.
    /// A climbing count here means the framing assumption is wrong, not that the air is quiet.
    pub undecodable: u64,
    /// Units that parsed all the way to a [`CapturedFrame`] in our wire format.
    pub accepted: u64,
}

// ── The backend ─────────────────────────────────────────────────────────────

/// A claimed MT7610U: the shared mt76 USB transport, the parsed EEPROM calibration, the
/// current tune, the RX pipeline, and this device's TSF clock domain.
pub struct Mt7610uBackend {
    /// The shared [`crate::mt76`] USB transport — EP0 register access plus the endpoint map.
    usb: Mt76Usb,
    /// Factory calibration, parsed once at [`open`](Self::open). The EEPROM shadow is served
    /// by the USB bridge and needs no firmware (MEASURED), so it is available from the
    /// moment the interface is claimed — which is what lets [`PhyBus::eeprom`] hand out a
    /// plain reference instead of an `Option` that half the code would have to unwrap.
    eeprom: Mt76x0Eeprom,
    /// Currently tuned channel (0 = never tuned). Read by the RX path to pick the EEPROM's
    /// 2.4 vs 5 GHz RSSI calibration, and by the power knobs, which are per-channel.
    channel: AtomicU8,
    /// Currently tuned bandwidth, as [`Bandwidth::code`]. Always 0 today — see the module
    /// header on why 40/80 MHz are refused rather than half-supported.
    bw: AtomicU8,
    /// TXWI rate word every subsequent [`FrameIo::inject`] transmits at, or `None` before
    /// the control plane has decided one. Rate is bearer *state*, not a per-frame argument.
    cur_rate: Mutex<Option<u16>>,
    /// Wire frame format (NDN ethertype by default).
    format: FrameFormat,
    /// 12-bit 802.11 sequence counter for injected frames.
    seq: AtomicU16,
    /// MCU command sequence, 1..=15 and never 0 (`mt76x02_usb_mcu.c:83-86`).
    mcu_seq: AtomicU8,
    /// Serialises RF CSR access. `mt76x0_rf_csr_wr`/`_rr` take `dev->phy_mutex` around the
    /// kick-poll-write-poll sequence (`mt76x0/phy.c:36,55`) because two interleaved kicks
    /// return each other's data.
    rf_lock: Mutex<()>,
    /// The shared RX pipeline (queue + wake + pumped flag) — the same
    /// [`RxPumpState`](crate::rx_pump::RxPumpState) every USB backend here uses.
    rx: crate::rx_pump::RxPumpState,
    /// How many AC bulk-OUT pipes to spread transmits over (1 = AC_BE only, the old behaviour).
    tx_queues: std::sync::atomic::AtomicU8,
    /// Round-robin cursor for [`Mt7610uBackend::tx_endpoint`].
    tx_rr: std::sync::atomic::AtomicU32,
    /// Sender into the pipelined TX pump, when one is running
    /// ([`spawn_tx_pump`](Self::spawn_tx_pump)). `None` = the old one-transfer-at-a-time path.
    ///
    /// ★ MEASURED 2026-08-31: the synchronous `write_bulk` in `inject` serialises one bulk at a
    /// time and costs **≈295 + 0.031·B µs** — a large width-INDEPENDENT constant plus a bus-time
    /// term (0.031 µs/B ≈ 258 Mbit/s, i.e. one in-flight bulk on USB 2.0 HS). MEASURED: 64 B at
    /// VHT MCS9/80 MHz carries ~1 µs of airtime and still took 305 µs/frame; 1400 B took 342 µs,
    /// and the 37 µs difference is exactly the per-byte term. The governing model is
    /// `period = max(PPDU + DCF, USB_serial(B))` — a MAX, not a sum. Because the USB term is
    /// width-independent, it pins every width to the same period whenever it dominates, which is
    /// precisely why 20/40/80 MHz measured identically at 1400 B. Same defect the MT7612U already
    /// fixed; see `mt7612::spawn_tx_pump`.
    tx_sender: Mutex<Option<std::sync::mpsc::SyncSender<Vec<u8>>>>,
    /// Bytes / frames the pump has actually handed to USB — the honest throughput figure.
    /// Counting `inject` returns instead would measure the speed of a channel send.
    tx_bytes: AtomicU64,
    tx_count: AtomicU64,
    /// As-found EDCA state, so a contention posture can be undone.
    edca_saved: crate::mt76::knobs::EdcaSaved,
    /// A-MPDU probe state: `(wcid, ba_window)`, or `(0xff, 0)` for off. See
    /// [`Mt7610uBackend::enable_ampdu`].
    ampdu: std::sync::atomic::AtomicU16,
    /// The domain this device's port TSF lives in. Per **device**, not per driver: two
    /// MT7610Us on one host have unrelated counters and must never be differenced.
    tsf_domain: ClockDomainId,
    // Software RX bookkeeping — see [`RxStats`].
    rx_units: AtomicU64,
    rx_crc_errors: AtomicU64,
    rx_l2pad: AtomicU64,
    rx_undecodable: AtomicU64,
    rx_accepted: AtomicU64,
    /// The same accepted-unit count, on its own window, so
    /// [`RadioKnobs::read_ofdm_counters`] and [`rx_stats_reset`](Mt7610uBackend::rx_stats_reset)
    /// do not drain each other's counter — two consumers sampling at different cadences
    /// would otherwise each see a fraction of the traffic and neither would be wrong-looking.
    rx_ok_window: AtomicU64,
    /// Stop flag for the **bring-up-duration** bulk-IN drain (M5).
    ///
    /// ★ It lives on the backend, and not in a `StepOutcome::Guard`, so that the drain covers
    /// EXACTLY the rungs it covered before the plan existed. A guard is owned by the returned
    /// handle, which would silently extend the drain across `monitor_rx` and `tune_channel` —
    /// rungs it never covered — and leave a thread eating received frames for the life of the
    /// process. Two rungs (`rx_drain` / `rx_drain_stop`) bracket it instead, and
    /// [`bring_up_planned`](Mt7610uBackend::bring_up_planned) sets it unconditionally afterwards
    /// so a failure partway through the ladder does not leak the thread either.
    bringup_drain: Arc<std::sync::atomic::AtomicBool>,
    /// `NDN_RADIO_FORCE_FW`, read ONCE at the wrapper boundary (LAW 1) and stashed here for the
    /// one rung that branches on it.
    ///
    /// ⚠ Stated plainly: this is driver state a rung reads, which is the shape of thing this
    /// contract removes — but it is *set from an argument at the caller boundary*, never from the
    /// environment inside the ladder, and the rung reports which way it went as its own
    /// `StepOutcome::Branch`. §1.1's `BringUpRequest` is where the flag belongs for good; the
    /// `Ctx` a rung is handed carries `RadioState` and nothing else, so until that type exists
    /// there is no other channel from the boundary to the rung.
    force_cold_fw: std::sync::atomic::AtomicBool,
}

impl Mt7610uBackend {
    // ── Open ────────────────────────────────────────────────────────────────

    /// Claim the first MT7610U on the bus.
    ///
    /// **Never resets.** A blind `handle.reset()` is what wedges these parts, and it is not
    /// needed: the warm-reopen guard in [`bring_up`](Self::bring_up) — not a bus reset — is
    /// what keeps a second run from re-downloading firmware over a live MCU. (Upstream's
    /// `mt76x0u_probe` does call `usb_reset_device`, `mt76x0/usb.c:248`; we deliberately do
    /// not follow it there.)
    pub fn open() -> Result<Self, FaceError> {
        Self::open_selected(DeviceSelect::from_env())
    }

    /// Claim a specific MT7610U — by USB bus:port (`"1-1.4"`, stable across reboots) or by
    /// enumeration index (`"#1"`). The selector is how a node with two identical dongles
    /// pins the spare instead of stealing the one carrying a live kernel link; the
    /// transport runs [`crate::usb_select::check_live_link`] before claiming.
    pub fn open_selected(sel: DeviceSelect) -> Result<Self, FaceError> {
        let usb = Mt76Usb::open_with(MEDIATEK_VID, MT7610U_PIDS, Family::Mt76x0, "MT7610U", &sel)?;

        // Identity first: everything downstream (the EEPROM layout, the RF tables, the
        // capability) is specific to this silicon, and reading `MT_ASIC_VERSION` costs one
        // EP0 round trip. `mt76x0/usb.c:265-272` does the same check and refuses `-ENODEV`.
        let asic = usb.rr(regs::MT_ASIC_VERSION)?;
        if asic >> 16 != 0x7610 {
            return Err(io_err(format!(
                "MT7610U: MT_ASIC_VERSION = {asic:#010x}, expected 0x7610xxxx — \
                 wrong silicon behind PID 0x7610, or the transport is not reading registers"
            )));
        }

        // The EEPROM shadow is a USB-bridge read (MT_VEND_READ_EEPROM) and needs no
        // firmware — MEASURED: 512/512 bytes, word 0 = 0x7610, MAC identical to the netdev's.
        let raw = read_eeprom_image(&usb)?;
        let eeprom = Mt76x0Eeprom::parse(&raw);

        // One domain per physical device, matching the scheme the Realtek and ath9k backends
        // use so a mixed-radio node's domains cannot collide.
        let dev = usb.handle().device();
        let tsf_domain =
            ClockDomainId((u32::from(dev.bus_number()) << 8) | u32::from(dev.address()));

        Ok(Self {
            usb,
            eeprom,
            channel: AtomicU8::new(0),
            bw: AtomicU8::new(0),
            cur_rate: Mutex::new(None),
            format: FrameFormat::default(),
            seq: AtomicU16::new(0),
            mcu_seq: AtomicU8::new(0),
            rf_lock: Mutex::new(()),
            rx: crate::rx_pump::RxPumpState::new(),
            tx_queues: std::sync::atomic::AtomicU8::new(
                std::env::var("NDN_TX_QUEUES")
                    .ok()
                    .and_then(|v| v.parse().ok())
                    .unwrap_or(1),
            ),
            tx_rr: std::sync::atomic::AtomicU32::new(0),
            tx_sender: Mutex::new(None),
            tx_bytes: AtomicU64::new(0),
            tx_count: AtomicU64::new(0),
            edca_saved: crate::mt76::knobs::EdcaSaved::default(),
            ampdu: std::sync::atomic::AtomicU16::new(0xff00),
            tsf_domain,
            rx_units: AtomicU64::new(0),
            rx_crc_errors: AtomicU64::new(0),
            rx_l2pad: AtomicU64::new(0),
            rx_undecodable: AtomicU64::new(0),
            rx_accepted: AtomicU64::new(0),
            rx_ok_window: AtomicU64::new(0),
            bringup_drain: Arc::new(std::sync::atomic::AtomicBool::new(true)),
            force_cold_fw: std::sync::atomic::AtomicBool::new(false),
        })
    }

    /// Set the wire frame format for [`FrameIo`] (defaults to the NDN ethertype).
    pub fn with_format(mut self, format: FrameFormat) -> Self {
        self.format = format;
        self
    }

    // ── Register access ─────────────────────────────────────────────────────

    /// Read a 32-bit MMIO register. ⚠ **151 µs** per call (MEASURED) — affordable in a
    /// channel switch or a sensing window, never on a per-frame path.
    pub fn rr(&self, addr: u32) -> Result<u32, FaceError> {
        self.usb.rr(addr)
    }

    /// Write a 32-bit MMIO register.
    pub fn wr(&self, addr: u32, val: u32) -> Result<(), FaceError> {
        self.usb.wr(addr, val)
    }

    /// Read-modify-write: clear `clear`, set `set`. Two round trips.
    pub fn rmw(&self, addr: u32, clear: u32, set: u32) -> Result<u32, FaceError> {
        Mt76Regs::rmw(self, addr, clear, set)
    }

    /// Poll `addr` until `(val & mask) == want`, at most `tries` milliseconds.
    /// Upstream's `mt76_poll`.
    fn poll(&self, addr: u32, mask: u32, want: u32, tries: u32) -> Result<u32, FaceError> {
        for _ in 0..tries.max(1) {
            let v = self.rr(addr)?;
            if v & mask == want {
                return Ok(v);
            }
            std::thread::sleep(Duration::from_millis(1));
        }
        Err(io_err(format!(
            "mt7610u: poll({addr:#06x} & {mask:#010x} == {want:#010x}) timed out after {tries} ms"
        )))
    }

    /// `mt76x02_wait_for_mac` (`mt76x02_mac.h:149-168`): `MAC_CSR0` (0x1000) reads neither
    /// 0 nor `~0` once the MAC block is out of reset. Both sentinels mean "the bus is
    /// answering but the block is not there", which is exactly the state a register write
    /// silently disappears into.
    fn wait_for_mac(&self) -> Result<(), FaceError> {
        const MAC_CSR0: u32 = 0x1000;
        for _ in 0..500 {
            match self.rr(MAC_CSR0) {
                Ok(0) | Ok(u32::MAX) | Err(_) => {}
                Ok(_) => return Ok(()),
            }
            std::thread::sleep(Duration::from_millis(5));
        }
        Err(io_err(
            "mt7610u: MAC never came out of reset (MAC_CSR0)".into(),
        ))
    }

    /// `mt76x02_wait_for_wpdma` (`mt76x02_dma.h:53-60`): both DMA-busy bits clear.
    fn wait_for_wpdma(&self, tries: u32) -> Result<(), FaceError> {
        self.poll(
            regs::MT_WPDMA_GLO_CFG,
            regs::MT_WPDMA_GLO_CFG_TX_DMA_BUSY | regs::MT_WPDMA_GLO_CFG_RX_DMA_BUSY,
            0,
            tries,
        )
        .map(|_| ())
    }

    /// `mt76x02_wait_for_txrx_idle` (`mt76x02.h:253-258`): the MAC has stopped both ways.
    fn wait_for_txrx_idle(&self) -> Result<(), FaceError> {
        self.poll(
            regs::MT_MAC_STATUS,
            regs::MT_MAC_STATUS_TX | regs::MT_MAC_STATUS_RX,
            0,
            100,
        )
        .map(|_| ())
    }

    /// `mt76x0_phy_wait_bbp_ready` (`mt76x0/phy.c:185-203`): `MT_BBP(CORE, 0)` reads a real
    /// version word rather than 0 or `~0`.
    fn wait_for_bbp(&self) -> Result<u32, FaceError> {
        let addr = regs::mt_bbp(regs::MT_BBP_CORE_BASE, 0);
        for _ in 0..20 {
            match self.rr(addr) {
                Ok(0) | Ok(u32::MAX) | Err(_) => {}
                Ok(v) => return Ok(v),
            }
            std::thread::sleep(Duration::from_millis(1));
        }
        Err(io_err("mt7610u: BBP is not ready (MT_BBP(CORE,0))".into()))
    }

    /// The chip's factory MAC address, from the parsed EEPROM.
    pub fn mac_address(&self) -> Result<[u8; 6], FaceError> {
        Ok(self.eeprom.mac_addr())
    }

    /// The parsed EEPROM calibration.
    pub fn eeprom(&self) -> &Mt76x0Eeprom {
        &self.eeprom
    }

    /// Read one RF bank register directly over `MT_RF_CSR_CFG`. Exposed because a tune that
    /// silently did not take is otherwise indistinguishable from a quiet channel — reading
    /// back RF(0,1)/RF(7,6) against the table we programmed is the cheap oracle.
    pub fn rf_read(&self, bank: u32, reg: u32) -> Result<u8, FaceError> {
        PhyBus::rf_rr(self, regs::mt_rf(bank, reg))
    }

    /// Write one RF bank register directly over `MT_RF_CSR_CFG`.
    pub fn rf_write(&self, bank: u32, reg: u32, val: u8) -> Result<(), FaceError> {
        PhyBus::rf_wr(self, regs::mt_rf(bank, reg), val)
    }

    // ── Power ───────────────────────────────────────────────────────────────

    /// `mt76x0_chip_onoff` (`mt76x0/init.c:44-69`) — the WLAN function gate.
    ///
    /// `reset` additionally pulses `WLAN_RESET | WLAN_RESET_RF` (only if WLAN was already
    /// enabled) and forces the GPIO output enables on while clearing the forced antenna
    /// select. Note what upstream deliberately does **not** do on the off path: it leaves
    /// `WLAN_CLK_EN` set, *"because that makes the device not respond properly on the probe
    /// path"* (`init.c:21-25`). Clearing the clock here would make the very next register
    /// read fail.
    fn chip_onoff(&self, enable: bool, reset: bool) -> Result<(), FaceError> {
        use regs::*;
        let mut val = self.rr(MT_WLAN_FUN_CTRL)?;

        if reset {
            val |= MT_WLAN_FUN_CTRL_GPIO_OUT_EN;
            val &= !MT_WLAN_FUN_CTRL_FRC_WL_ANT_SEL;
            if val & MT_WLAN_FUN_CTRL_WLAN_EN != 0 {
                val |= MT_WLAN_FUN_CTRL_WLAN_RESET | MT_WLAN_FUN_CTRL_WLAN_RESET_RF;
                self.wr(MT_WLAN_FUN_CTRL, val)?;
                std::thread::sleep(Duration::from_micros(20));
                val &= !(MT_WLAN_FUN_CTRL_WLAN_RESET | MT_WLAN_FUN_CTRL_WLAN_RESET_RF);
            }
        }

        self.wr(MT_WLAN_FUN_CTRL, val)?;
        std::thread::sleep(Duration::from_micros(20));

        // mt76x0_set_wlan_state (init.c:16-42).
        if enable {
            val |= MT_WLAN_FUN_CTRL_WLAN_EN | MT_WLAN_FUN_CTRL_WLAN_CLK_EN;
        } else {
            val &= !MT_WLAN_FUN_CTRL_WLAN_EN;
        }
        self.wr(MT_WLAN_FUN_CTRL, val)?;
        std::thread::sleep(Duration::from_micros(20));

        if enable {
            // Until the crystal is ready and the PLL is locked, MAC register writes do not
            // stick — and they do not fail either, which is the trap. Upstream logs and
            // continues (init.c:40-41); we surface it, because a bring-up that proceeds past
            // an unlocked PLL fails later somewhere far less legible.
            let mask = MT_CMB_CTRL_XTAL_RDY | MT_CMB_CTRL_PLL_LD;
            self.poll(MT_CMB_CTRL, mask, mask, 2000)
                .map_err(|_| io_err("mt7610u: XTAL/PLL never came ready (MT_CMB_CTRL)".into()))?;
        }
        Ok(())
    }

    /// True when the MCU firmware **status latch** says it is up: `mt76x0_firmware_running`
    /// (`mt76x0/mcu.h:41-44`) is exactly `MT_MCU_COM_REG0 == 1`.
    ///
    /// ☠ **This is no longer the warm/cold decision, and must not become one again** (M5).
    /// `MT_MCU_COM_REG0` (`0x0730`) is a **MAILBOX**, not a status flag: the firmware reuses it
    /// as a destination address and it goes stale. On the sibling MT7612U, deciding warm/cold
    /// from this latch was MEASURED wrong in BOTH directions — "cold" on a chip the kernel had
    /// just loaded (so the cold path downloaded firmware into a live MCU, collided with the FCE
    /// and wedged the part) and "warm" on a chip that answered nothing. Each mistake cost a
    /// physical replug. That part fixed it with a round trip; this one still read the latch,
    /// which is the live divergence M5 was told to close. See
    /// [`mcu_responsive`](Self::mcu_responsive), which is what the plan branches on.
    ///
    /// Kept as a diagnostic — it is the corroborating heuristic in the three-outcome decision,
    /// and it is what upstream itself tests — but it decides nothing on its own.
    pub fn firmware_running(&self) -> bool {
        matches!(self.rr(regs::MT_MCU_COM_REG0), Ok(1))
    }

    /// **Does the MCU answer *us*, as opposed to merely being loaded?** The warm/cold evidence.
    ///
    /// ☠ The rule this exists for: *warm/cold must use a round trip, not a status latch.*
    /// Deciding from `MT_MCU_COM_REG0` was wrong in both directions on the sibling MT7612U and
    /// cost a replug each time; that part's `mcu_responsive()` is the fix, and this is its
    /// MT7610U twin, written against the same reasoning:
    ///
    /// * it sends a command that requires the MCU to **compose a response** —
    ///   [`mcu::rd_rp`] is `CMD_RANDOM_READ`, which upstream itself always waits on
    ///   (`mt76x02_usb_mcu.c:198`) because there is nowhere else for the value to come from. A
    ///   fire-and-forget command would "succeed" against a dead MCU and tell us nothing;
    /// * it reads a register we already know (`MT_MCU_CLOCK_CTL`) so the *value* is irrelevant
    ///   and only the round trip is being tested;
    /// * **any** error, timeout or malformed answer reports NOT responsive. A false "responsive"
    ///   costs a bench trip (the cold path never runs and the chip stays half-initialised); a
    ///   false "unresponsive" costs one firmware download. Conservative in the cheap direction.
    ///
    /// ⚠ It is a real bus round trip (~1 ms, and up to the MCU response timeout when the MCU is
    /// dead), which is why it is taken **once**, inside the `firmware_ready` rung, and not
    /// polled.
    pub fn mcu_responsive(&self) -> bool {
        let mut pairs = [RegPair {
            reg: regs::MT_MCU_CLOCK_CTL,
            value: 0,
        }];
        mcu::rd_rp(self, 0, &mut pairs).is_ok()
    }

    // ── Bring-up ────────────────────────────────────────────────────────────

    /// **The full bring-up — [`PLAN_MT7610U`], `Role::TransmitAndReceive`.** A thin wrapper over
    /// [`bring_up_planned`](Self::bring_up_planned); the sequence lives in the plan, where every
    /// rung states why it is there and a reviewer sees the whole ladder at once.
    ///
    /// Power the WLAN block, download firmware, program the MAC and BBP tables, initialise the RF,
    /// program the factory MAC address, pin the contention posture, **put the MAC into promiscuous
    /// monitor RX, and tune** — the last two being the M5 change: `bring_up`, `setup_monitor_rx`
    /// and the tune are ONE plan now, so `MT_MAC_SYS_CTRL = ENABLE_TX | ENABLE_RX` can no longer
    /// be missing because a caller forgot a call. `ASSERTS_MT7610U` reads that gate back.
    ///
    /// ⚠ **`self: &Arc<Self>`, not `&self`** (M5). [`Step::run`] takes `&Arc<B>` because a rung
    /// may hand back a live guard. This part produces none, but the runner's signature is shared
    /// with the part that does (the 8733b `PowerTracker`).
    ///
    /// ⚠ **`channel` is now an argument.** The old `bring_up()` left the radio untuned and
    /// reported `channel: 0`; the factory then tuned separately and patched the report's channel
    /// field afterwards. The report now describes a tuned radio because the plan tuned it.
    ///
    /// ⚠ **`NDN_RADIO_FORCE_FW` is no longer read inside the ladder** (LAW 1). It is read once,
    /// here, at the wrapper boundary, and passed down as an argument.
    pub fn bring_up(self: &Arc<Self>, channel: u8) -> Result<BringUpReport, FaceError> {
        // ★ M8: ONE reader. `NDN_RADIO_FORCE_FW` is read by
        // `crate::open_radio::mt76_force_cold_from_env`, which `BringUpRequest::from_env` also
        // calls — so this wrapper and the factory can never disagree about it.
        let force_cold = crate::open_radio::mt76_force_cold_from_env();
        self.bring_up_planned(
            channel,
            Role::TransmitAndReceive,
            force_cold,
            None,
            ProofRequirement::BestAvailable,
        )
        .map(|(report, guards)| {
            debug_assert!(guards.is_empty(), "this plan produces no guards");
            report
        })
        .map_err(drop_partial_report)
    }

    /// **The one entry point.** Everything else is a wrapper over this.
    ///
    /// The role selects the plan ([`BringUp::plan`]); the deviation, if any, is resolved against
    /// that plan before the first register write and lands in the report's digest; the proof
    /// requirement is validated against the role and this part's (empty) instrument set, also
    /// before the first register write.
    ///
    /// `force_cold` is `NDN_RADIO_FORCE_FW`, read at the caller boundary rather than inside a
    /// rung. §1.1's `BringUpRequest` is where it belongs for good; until that exists it is an
    /// argument, stashed for the one rung that needs it and reported as the branch that rung took.
    // The `Err` is large BECAUSE it carries the partial report — the whole point of §3. Same
    // allow, same reason, as `run_plan`'s.
    #[allow(clippy::result_large_err)]
    pub fn bring_up_planned(
        self: &Arc<Self>,
        channel: u8,
        role: Role,
        force_cold: bool,
        deviation: Option<Deviation>,
        proof: ProofRequirement,
    ) -> Result<(BringUpReport, Guards), BringUpFailure> {
        self.force_cold_fw
            .store(force_cold, std::sync::atomic::Ordering::Relaxed);
        let mut run = PlanRun::new(
            "MT7610U",
            ndn_radio_hal::DeviceAddress::Usb(self.usb.usb_addr().to_string()),
            self.initial_state(channel, role),
        )
        .with_proof(proof);
        if let Some(d) = deviation {
            run = run.with_deviation(d);
        }
        // UFCS: the inherent `bring_up(channel)` above shadows the trait method of the same name.
        let out = <Self as BringUp>::bring_up(self, &run);
        // ★ Belt over the `rx_drain_stop` rung: a Required failure anywhere above it would return
        // before that rung runs, and the drain thread would then read bulk-IN for the life of the
        // process and compete with whatever ran next. `DrainGuard`'s `Drop` used to do this.
        self.bringup_drain
            .store(true, std::sync::atomic::Ordering::Relaxed);
        let (report, guards) = out?;
        Ok((
            report.with_capability(ndn_radio_hal::RadioProfile::capability(self.as_ref())),
            guards,
        ))
    }

    /// The regime a plan starts from. Not a claim about the radio: it is what the caller asked
    /// for, and the rungs fill in what they establish.
    fn initial_state(&self, channel: u8, role: Role) -> RadioState {
        RadioState {
            channel,
            bw: ndn_radio_hal::Bandwidth::Bw20,
            format: "RawNdn (see with_format)",
            role,
            // ⚠ The ladder sets no power: this part's power is per-channel AND per-rate, so it
            // cannot be set before a tune. `NoActuator` would be a lie (the knob works); this says
            // the bring-up did not touch it, which is the truth.
            power: AppliedPower::no_actuator(PowerRequest::NoActuator),
            rate: ndn_radio_hal::RateState::unreported(),
            warm: None,
            contention: None,
            pump: PumpPolicy::CallerOwns,
            facts: Vec::new(),
        }
    }

    /// `mt76x0_init_usb_dma` (`mt76x0/usb.c:46-71`).
    ///
    /// Enables both bulk directions and **clears `RX_BULK_AGG_EN`** — upstream's comment is
    /// *"disable AGGR_BULK_RX in order to receive one frame in each rx urb and avoid
    /// copies"*, and the oracle MEASURED the bit clear on the live kernel interface. That is
    /// the answer to "does mt76 USB aggregate RX?" and the reason [`parse_transfer`] sees
    /// one unit per transfer.
    ///
    /// The `RX_DROP_OR_PAD` set-then-clear at `usb.c:67-70` is a pulse whose purpose
    /// upstream does not state. Ported faithfully; meaning unknown.
    fn init_usb_dma(&self) -> Result<(), FaceError> {
        use regs::*;
        self.rmw(
            MT_USB_DMA_CFG,
            MT_USB_DMA_CFG_RX_BULK_AGG_EN,
            MT_USB_DMA_CFG_RX_BULK_EN | MT_USB_DMA_CFG_TX_BULK_EN,
        )?;
        let v = self.rr(MT_USB_DMA_CFG)?;
        self.wr(MT_USB_DMA_CFG, v | MT_USB_DMA_CFG_RX_DROP_OR_PAD)?;
        self.wr(MT_USB_DMA_CFG, v & !MT_USB_DMA_CFG_RX_DROP_OR_PAD)?;
        Ok(())
    }

    /// `mt76x0_init_hardware` (`mt76x0/init.c:171-212`) — MAC tables, BBP tables, RF init.
    ///
    /// Two upstream steps are deliberately absent and are called out in the module header:
    /// the 64-shared-key + 256-WCID wipe (`init.c:198-203`), and the EEPROM read, which this
    /// port does at [`open`](Self::open) because the shadow needs no firmware.
    fn init_hardware(&self) -> Result<(), FaceError> {
        self.wait_for_wpdma(1000)?;
        self.wait_for_mac()?;
        self.reset_csr_bbp()?;
        // Q_SELECT tells the firmware which transmit queue mapping to use. Upstream sends it
        // fire-and-forget: `mt76x02_mcu_function_select` waits for a response for every
        // function EXCEPT Q_SELECT (`mt76x02_mcu.c:92-95`).
        mcu::mcu_send(
            self,
            MCU_CMD_FUN_SET_OP,
            &encode_fun_set_op(MCU_FUNC_Q_SELECT, 1),
            false,
        )?;

        self.init_mac_registers()?;
        self.wait_for_txrx_idle()?;
        self.init_bbp()?;
        phy::init_rf(self)?;
        Ok(())
    }

    /// `mt76x0_reset_csr_bbp` (`mt76x0/init.c:72-81`): assert both resets, wait 200 ms, drop
    /// them. The 200 ms is upstream's; no shorter value has been tried on silicon.
    fn reset_csr_bbp(&self) -> Result<(), FaceError> {
        use regs::*;
        self.wr(
            MT_MAC_SYS_CTRL,
            MT_MAC_SYS_CTRL_RESET_CSR | MT_MAC_SYS_CTRL_RESET_BBP,
        )?;
        std::thread::sleep(Duration::from_millis(200));
        self.rmw(
            MT_MAC_SYS_CTRL,
            MT_MAC_SYS_CTRL_RESET_CSR | MT_MAC_SYS_CTRL_RESET_BBP,
            0,
        )?;
        Ok(())
    }

    /// `mt76x0_init_mac_registers` (`mt76x0/init.c:110-134`).
    ///
    /// The two tables go through the MCU register-pair path with the WLAN memory-map base,
    /// which is what upstream's `RANDOM_WRITE` macro expands to
    /// (`init.c:83-85`: `mt76_wr_rp(dev, MT_MCU_MEMMAP_WLAN, tab, n)`) — hence firmware
    /// before MAC init, not the other way round.
    fn init_mac_registers(&self) -> Result<(), FaceError> {
        use regs::*;
        self.wr_pairs(MT_MCU_MEMMAP_WLAN, initvals::COMMON_MAC_REG_TABLE)?;
        // "Enable PBF and MAC clock SYS_CTRL[11:10] = 0x3" (init.c:114).
        self.wr_pairs(MT_MCU_MEMMAP_WLAN, initvals::MT76X0_MAC_REG_TABLE)?;
        // "Release BBP and MAC reset MAC_SYS_CTRL[1:0] = 0x0" (init.c:117-118).
        self.rmw(MT_MAC_SYS_CTRL, 0x3, 0)?;
        // "Set 0x141C[15:12]=0xF" (init.c:120-121) — the ED-CCA mask. This is the top nibble
        // of the MEASURED kernel value 0x0000_f1e4.
        self.rmw(MT_EXT_CCA_CFG, 0, 0xf000)?;
        self.rmw(MT_FCE_L2_STUFF, MT_FCE_L2_STUFF_WR_MPDU_LEN_EN, 0)?;
        // "tx_ring 9 is for mgmt frame, tx_ring 8 is for in-band command frame" (init.c:125-133).
        self.rmw(MT_WMM_CTRL, 0x3ff, 0x201)?;
        Ok(())
    }

    /// `mt76x0_init_bbp` (`mt76x0/init.c:87-108`).
    ///
    /// The switch table is filtered to the entries whose `bw_band` contains **both**
    /// `RF_G_BAND` and `RF_BW_20` — upstream's `((RF_G_BAND | RF_BW_20) & item->bw_band) ==
    /// (RF_G_BAND | RF_BW_20)` (`init.c:101`), i.e. it seeds the baseband at 2.4 GHz/20 MHz
    /// regardless of where we are about to tune. `phy::set_channel` re-applies the correct
    /// band/width rows afterwards.
    ///
    /// Note the asymmetry, which is upstream's and is preserved: the init and DCOC tables go
    /// through the MCU register-pair path, the switch table through plain register writes
    /// (`init.c:102`).
    fn init_bbp(&self) -> Result<(), FaceError> {
        self.wait_for_bbp()?;
        self.wr_pairs(regs::MT_MCU_MEMMAP_WLAN, initvals::MT76X0_BBP_INIT_TAB)?;
        let want = initvals_phy::RF_G_BAND | initvals_phy::RF_BW_20;
        for item in initvals::BBP_SWITCH_TAB {
            if item.bw_band & want == want {
                self.wr(item.reg, item.val)?;
            }
        }
        self.wr_pairs(regs::MT_MCU_MEMMAP_WLAN, initvals::MT76X0_DCOC_TAB)?;
        Ok(())
    }

    /// Push a `(reg, value)` init table through the MCU `CMD_RANDOM_WRITE` register-pair
    /// protocol at `base`. Upstream's `RANDOM_WRITE` (`mt76x0/init.c:83-85`).
    fn wr_pairs(&self, base: u32, table: &[(u32, u32)]) -> Result<(), FaceError> {
        let pairs: Vec<RegPair> = table
            .iter()
            .map(|&(reg, value)| RegPair { reg, value })
            .collect();
        mcu::wr_rp(self, base, &pairs)
    }

    /// Program the station MAC address — `mt76x02_mac_setaddr` (`mt76x02_mac.c:740-743`).
    ///
    /// Only the `MT_MAC_ADDR_DW*` half of upstream's function: the `MT_MAC_BSSID_DW*` /
    /// multi-BSS / 16-slot beacon block after it (`:745-756`) configures beaconing, which
    /// this driver never does, and BSSID filtering, which a promiscuous monitor ignores.
    pub fn set_mac_address(&self, mac: [u8; 6]) -> Result<(), FaceError> {
        let dw0 = u32::from_le_bytes([mac[0], mac[1], mac[2], mac[3]]);
        let dw1 = u32::from(u16::from_le_bytes([mac[4], mac[5]]))
            | regs::field_prep(regs::MT_MAC_ADDR_DW1_U2ME_MASK, 0xff);
        self.wr(regs::MT_MAC_ADDR_DW0, dw0)?;
        self.wr(regs::MT_MAC_ADDR_DW1, dw1)?;
        Ok(())
    }

    // ── Monitor RX ──────────────────────────────────────────────────────────

    rung! {
        /// Put the MAC into promiscuous monitor receive and start the TSF.
        ///
        /// **`MT_RX_FILTR_CFG = 0`, not the MEASURED kernel-monitor `0x0000_1093`.** That value
        /// is what mac80211's monitor path leaves: the init table's `0x0001_7f97`
        /// (`mt76x0/initvals_init.h:20`) minus PROMISC and the control-frame group
        /// (`mt76x0/main.c:80-87`, `mt76x02_util.c:223-229`). It still **drops** CRC-error
        /// frames, RTS, duplicates, version errors and BARs. A named-data receiver wants the
        /// opposite: every PPDU the PHY delivers, with the CRC verdict carried per frame in
        /// `MT_RXINFO_CRCERR` so a bad-FCS frame is *counted* and then dropped in software
        /// rather than never seen. Dropping in hardware makes a marginal link and a quiet
        /// channel look identical — which is the exact ambiguity the frame-free occupancy
        /// counters exist to resolve.
        ///
        /// The bits are `mt76x02u_mac_start`'s otherwise (`mt76x02_usb_core.c:25-43`):
        /// `MT_MAC_SYS_CTRL = ENABLE_TX | ENABLE_RX` (= the MEASURED `0x0c`), with the USB DMA
        /// bulk enables re-asserted first because a firmware download leaves them elsewhere.
        fn setup_monitor_rx(&self) -> Result<(), FaceError> {
            use regs::*;
            self.init_usb_dma()?;
            self.wr(MT_RX_FILTR_CFG, 0)?;
            self.wr(
                MT_MAC_SYS_CTRL,
                MT_MAC_SYS_CTRL_ENABLE_TX | MT_MAC_SYS_CTRL_ENABLE_RX,
            )?;
            self.enable_tsf()?;
            // Arm the channel-time counters too. They are free (one register write), they are the
            // radio's only frame-free occupancy sense, and leaving them unarmed makes
            // `read_channel_activity` return a confident 0 permille — which reads as "quiet
            // channel" and is not. MEASURED before this call was added: `MT_CH_TIME_CFG` came out
            // of init at 0x1f, i.e. counting, but with `MT_CH_CCA_RC_EN` (bit 6) clear, so the
            // counters free-ran instead of being read-and-clear and `MT_CH_BUSY` read 0.
            crate::mt76::knobs::enable_channel_time_counters(self)?;

            // ★ Post-condition check, because the two ways this silicon fails silently are both
            // visible in one register each and neither raises an error on its own.
            //   * `MT_WLAN_FUN_CTRL` bit 0 (`WLAN_EN`) clear = the WLAN function is gated off. The
            //     MAC still answers every register read, the RF hears nothing, and the symptom is
            //     an interface that comes up perfectly and receives zero frames.
            //   * `MT_CMB_CTRL` without `XTAL_RDY|PLL_LD` = the synthesiser never locked.
            // MEASURED on this part: a run that reached monitor with `WLAN_FUN_CTRL = 0xff000012`
            // (bit 0 clear) logged 0 CRC errors, 0 PHY errors and 0 busy microseconds — the
            // receiver was not merely quiet, it was off.
            let wlan = self.rr(MT_WLAN_FUN_CTRL)?;
            if wlan & MT_WLAN_FUN_CTRL_WLAN_EN == 0 {
                tracing::warn!(
                    wlan_fun_ctrl = format!("{wlan:#010x}"),
                    "mt7610u: WLAN_EN is clear after monitor setup — re-asserting; the radio would                  otherwise receive nothing while looking healthy"
                );
                self.wr(
                    MT_WLAN_FUN_CTRL,
                    wlan | MT_WLAN_FUN_CTRL_WLAN_EN | MT_WLAN_FUN_CTRL_WLAN_CLK_EN,
                )?;
                std::thread::sleep(Duration::from_millis(1));
                let mask = MT_CMB_CTRL_XTAL_RDY | MT_CMB_CTRL_PLL_LD;
                self.poll(MT_CMB_CTRL, mask, mask, 2000)?;
            }
            Ok(())
        }
    }

    /// A one-line summary of the registers that decide whether this radio can hear anything.
    /// Printed by the bring-up gate; cheap enough (7 EP0 reads) to call after any state change.
    pub fn rx_health(&self) -> Result<String, FaceError> {
        use regs::*;
        Ok(format!(
            "WLAN_FUN_CTRL={:#010x} (WLAN_EN={}) CMB_CTRL={:#010x} (xtal+pll={})              MAC_SYS_CTRL={:#06x} RX_FILTR={:#010x} USB_DMA={:#010x} CH_TIME_CFG={:#06x}",
            self.rr(MT_WLAN_FUN_CTRL)?,
            self.rr(MT_WLAN_FUN_CTRL)? & MT_WLAN_FUN_CTRL_WLAN_EN != 0,
            self.rr(MT_CMB_CTRL)?,
            {
                let m = MT_CMB_CTRL_XTAL_RDY | MT_CMB_CTRL_PLL_LD;
                self.rr(MT_CMB_CTRL)? & m == m
            },
            self.rr(MT_MAC_SYS_CTRL)?,
            self.rr(MT_RX_FILTR_CFG)?,
            self.rr(MT_USB_DMA_CFG)?,
            self.rr(MT_CH_TIME_CFG)?,
        ))
    }

    /// Start the port TSF, and make it monotonic.
    ///
    /// ★ MEASURED: `MT_TSF_TIMER_DW0` reads a static 0 as found because
    /// `MT_BEACON_TIME_CFG` bit 16 (`TIMER_EN`) is clear — the kernel clears it in every
    /// non-beaconing vif (`mt76x02_usb_core.c:278-282`). Set it and the counter advances
    /// **1.000 µs per tick** (measured against host steps of ~11.2 ms over five windows).
    ///
    /// `SYNC_MODE [18:17]` is cleared at the same time and that is not cosmetic: it selects
    /// whether a received beacon slams the TSF to the transmitter's value. With it clear the
    /// counter free-runs, which is what lets [`RadioTime::time_sources`] honestly declare
    /// `monotonic: true` for a clock the 802.11 spec otherwise makes resynchronisable.
    pub fn enable_tsf(&self) -> Result<(), FaceError> {
        self.rmw(
            regs::MT_BEACON_TIME_CFG,
            regs::MT_BEACON_TIME_CFG_SYNC_MODE,
            regs::MT_BEACON_TIME_CFG_TIMER_EN,
        )?;
        Ok(())
    }

    /// Run a PHY calibration at the current channel — `full` selects `MCU_CAL_FULL` over the
    /// per-item set. Exposed because calibration is worth re-running after a long dwell or a
    /// temperature swing, not only at tune time.
    pub fn calibrate(&self, full: bool) -> Result<(), FaceError> {
        let ch = self.channel.load(Ordering::Relaxed);
        if ch == 0 {
            return Err(io_err("mt7610u: calibrate before set_channel".into()));
        }
        phy::calibrate(self, ch, full)
    }

    /// Take and clear the software RX counters — see [`RxStats`].
    pub fn rx_stats_reset(&self) -> RxStats {
        RxStats {
            units: self.rx_units.swap(0, Ordering::Relaxed),
            crc_errors: self.rx_crc_errors.swap(0, Ordering::Relaxed),
            l2pad: self.rx_l2pad.swap(0, Ordering::Relaxed),
            undecodable: self.rx_undecodable.swap(0, Ordering::Relaxed),
            accepted: self.rx_accepted.swap(0, Ordering::Relaxed),
        }
    }

    /// Keep `depth` bulk-IN transfers in flight so a busy channel is not dropped between
    /// userspace reads; [`FrameIo::recv_frame`] then drains the shared queue.
    pub fn spawn_rx_pump(self: &Arc<Self>, depth: usize) -> Vec<std::thread::JoinHandle<()>> {
        crate::rx_pump::spawn_rx_pump(self, depth)
    }

    /// Read one raw bulk-IN transfer (an RX unit: DMA header + RXWI + MPDU + FCE trailer).
    /// Returns 0 on timeout. For the first "is anything arriving at all" check.
    /// Which bulk-OUT endpoint the next transmit uses.
    ///
    /// Default: AC_BE only, exactly as before — one queue, so behaviour is unchanged unless
    /// asked for. `NDN_TX_QUEUES=<1..4>` round-robins across that many access-category pipes,
    /// which is the experiment [`EP_OUT_ACS`] documents.
    ///
    /// ⚠ Spreading a single logical stream across four ACs is **not** a free win in general:
    /// the four categories have different EDCA parameters, so frames can reorder, and on a
    /// contended channel the higher-priority queues would take airtime from other stations.
    /// It is a throughput probe first; making it a shipping default needs the reordering
    /// question answered by whoever consumes the stream.
    fn tx_endpoint(&self) -> u8 {
        let n = self.tx_queues.load(Ordering::Relaxed).clamp(1, 4) as usize;
        if n == 1 {
            return self.usb.ep_data_out();
        }
        let i = self.tx_rr.fetch_add(1, Ordering::Relaxed) as usize % n;
        self.usb.endpoints().out_at(EP_OUT_ACS[i])
    }

    /// ★ **The A-MPDU probe.** Program a WCID entry and start tagging transmits for
    /// aggregation, so the MAC can put several MPDUs into one PPDU.
    ///
    /// Why this is the only lever left, as arithmetic rather than opinion: the measured PPDU
    /// rate on this family is **~3000/s** with a fixed **~290 µs** per frame that is NOT
    /// airtime (the variable part tracks airtime exactly), NOT host USB dispatch (pipelined
    /// writers do not move it), NOT DCF backoff (removing it made throughput 5.7x *worse*), and
    /// NOT per-queue serialisation (1 vs 2 vs 4 AC endpoints measured 1625/1761/1685 f/s — all
    /// noise). At 3000 PPDU/s, 400 Mbit/s needs **16.7 kB per frame**, and the VHT MPDU limit is
    /// 11454 B. One MPDU per PPDU therefore cannot reach it *at any rate or width*. Several
    /// MPDUs must share one PPDU, which is A-MPDU, which needs a WCID.
    ///
    /// ⚠ **This is in tension with the named-radio doctrine and that is not hidden.** The
    /// doctrine forbids a host identity on air and this bearer transmits broadcast with
    /// `wcid = 0xff` (no station). Aggregation is defined over a *station relationship*: the
    /// hardware aggregates frames queued to a (WCID, TID). So the bulk path and the
    /// discovery/broadcast path may simply have to differ — a decision above this driver's
    /// pay grade, which is why this is an opt-in probe rather than a default.
    ///
    /// ⚠ `ba_window = 0` keeps `MT_TXWI_ACK_CTL_REQ` **clear**. A real block-ack session needs a
    /// responder, and on a bench with no peer, requesting ACKs would make the MAC retry every
    /// frame and collapse throughput.
    ///
    /// ☠ **MEASURED 2026-08-28: this does NOT engage aggregation, and the answer is structural.**
    /// ch36, 20 MHz, HT MCS7, 1400 B, WCID 1 programmed, `FLAGS_AMPDU` set, unicast `addr1`
    /// matching the WCID:
    ///
    /// | ba_window | offered |
    /// |---|---|
    /// | off (broadcast, wcid 0xff) | 2818 f/s |
    /// | 8 | 2244 f/s |
    /// | 32 | 2770 f/s |
    /// | 64 | MCU calibration failure |
    ///
    /// All within noise of the un-aggregated baseline. Setting the descriptor bit and a WCID is
    /// not enough: the hardware aggregates over a **block-ack agreement**, which needs a peer
    /// that ACKs and a BA session negotiated with it. A monitor-inject path with no association
    /// has none, so every MPDU is its own PPDU and pays its own medium access — which is exactly
    /// the ~290-370 µs floor measured across all three parts.
    ///
    /// ⇒ **This is why an AP/STA iperf reaches 400+ Mbit/s and this path cannot.** It is not a
    /// missing register write; it is a missing relationship. Closing the gap means a real link
    /// (station records, BA negotiation, ACK handling) — which is a direct conflict with the
    /// named-radio doctrine's "no host identity, broadcast only", and therefore a design
    /// decision rather than a driver task.
    ///
    /// Registers: `MT_WCID_ADDR(n) = 0x1800 + 8n` (the peer MAC),
    /// `MT_WCID_ATTR(n) = 0xa800 + 4n` (BSS index) — `mt76x02_mac_wcid_setup`,
    /// `mt76x02_mac.c:148-167`. Shared with the MT7612U, so a result here transfers.
    pub fn enable_ampdu(&self, wcid: u8, peer: [u8; 6], ba_window: u8) -> Result<(), FaceError> {
        // MT_WCID_ATTR: BSS_IDX 0, no pairwise key, no cipher.
        self.wr(0xa800 + (u32::from(wcid) << 2), 0)?;
        // MT_WCID_ADDR: 6 MAC bytes then 2 zero bytes, as two little-endian words.
        let lo = u32::from_le_bytes([peer[0], peer[1], peer[2], peer[3]]);
        let hi = u32::from(u16::from_le_bytes([peer[4], peer[5]]));
        self.wr(0x1800 + (u32::from(wcid) << 3), lo)?;
        self.wr(0x1800 + (u32::from(wcid) << 3) + 4, hi)?;
        self.ampdu.store(
            (u16::from(wcid) << 8) | u16::from(ba_window),
            Ordering::Relaxed,
        );
        Ok(())
    }

    /// Turn the A-MPDU probe back off (transmit as broadcast, `wcid = 0xff`, no aggregation).
    pub fn disable_ampdu(&self) {
        self.ampdu.store(0xff00, Ordering::Relaxed);
    }

    /// Set how many access-category TX pipes to round-robin over (1 = AC_BE only).
    pub fn set_tx_queues(&self, n: u8) {
        self.tx_queues.store(n.clamp(1, 4), Ordering::Relaxed);
    }

    /// Spawn `depth` dedicated TX-pump threads — the pipelined transmit path.
    ///
    /// ★ **Why this exists (MEASURED 2026-08-31).** `inject`'s synchronous `write_bulk` submits
    /// one USB bulk and blocks for its completion — `≈295 + 0.031·B µs`, a width-independent
    /// constant plus USB bus time. At VHT MCS9 / 80 MHz a 64 B frame carries ~1 µs of airtime and
    /// still took 305 µs/frame; the radio was idle ~99% of that. It capped the part near
    /// 3000 PPDU/s, and being width-independent it pinned all three widths to one period.
    ///
    /// ★ The paired proof (1400 B, 3 reps, VHT MCS7, µs/frame):
    ///
    /// ```text
    ///            Bw20        Bw40        Bw80      predicted (pumped)
    ///   pump=0   345/354/313 333/344/339 312/319/320   ~338 at every width
    ///   pump=8   294/313/322 194/194/210 140/142/144   279 / 187 / 143
    /// ```
    ///
    /// Synchronous is FLAT across width — the USB floor, masking the air entirely. Pipelined, the
    /// period tracks airtime. That width-dependence is also the proof the frames are real: no
    /// host-side or USB-side artifact can vary with channel width.
    ///
    /// ⚠⚠ **Those two rows were taken before EDCA was programmed at bring-up, so they share an
    /// arbitrary inherited posture.** The pump=0 vs pump=8 comparison is still sound — the posture
    /// was constant across every cell — but the absolute numbers are not reproducible and must not
    /// be quoted on their own. With the posture PINNED (3 reps each, Bw80, VHT MCS9, Mbit/s):
    ///
    /// ```text
    ///                        1400 B                 11400 B
    ///   shared, sync    33.4 / 31.2 / 28.9     125.2 / 130.9 / 127.3
    ///   shared, pump    43.3 / 43.0 / 30.1     127.6 / 169.8 / 123.1
    ///   owned,  pump    77.8 / 84.6 / 83.1     246.5 / 255.6 / 248.8
    /// ```
    ///
    /// Read honestly: the pump is worth ~+24% at 1400 B under `Shared` and is **within noise at
    /// 11400 B under `Shared`** — at that size, on a channel carrying other networks, the MEDIUM
    /// binds and not USB. Its full value appears where the USB term dominates: small payloads, and
    /// any aggressive posture. On this bench the posture is the larger lever of the two
    /// (`Owned` ≈ 2.1x `Shared` at both payloads). Peak measured: **~250 Mbit/s** at
    /// `Owned` + pump + 11400 B + Bw80 + VHT MCS9.
    ///
    /// Each thread locks the receiver only for a fast `recv`, then does the slow `write_bulk`
    /// OUTSIDE the lock, so up to `depth` transfers are in flight and the host controller
    /// pipelines them. Endpoint selection still goes through [`tx_endpoint`](Self::tx_endpoint),
    /// so `NDN_TX_QUEUES` round-robin composes with this. Call after `bring_up`.
    ///
    /// Frame order across threads is not preserved — fine for connectionless NDN broadcast,
    /// and the reason this is opt-in rather than the default.
    ///
    /// ⚠ The queue is **bounded** (`depth * 4`). An unbounded one makes `inject` a non-blocking
    /// enqueue with no backpressure: MEASURED on the sibling MT7921AU, a 3 s flood queued
    /// hundreds of thousands of frames that took minutes to drain, so the "throughput" recorded
    /// was the speed of a channel send while the radio was still transmitting. Measure with
    /// [`tx_count_written`](Self::tx_count_written), never by counting `inject` returns.
    pub fn spawn_tx_pump(
        self: &std::sync::Arc<Self>,
        depth: usize,
    ) -> Vec<std::thread::JoinHandle<()>> {
        let (tx, rx) = std::sync::mpsc::sync_channel::<Vec<u8>>(depth.max(1) * 4);
        *self.tx_sender.lock().unwrap() = Some(tx);
        let rx = std::sync::Arc::new(Mutex::new(rx));
        (0..depth.max(1))
            .map(|_| {
                let me = self.clone();
                let rx = rx.clone();
                std::thread::spawn(move || {
                    let handle = me.usb.handle();
                    loop {
                        // Block rather than poll: a sleep here would put a floor under
                        // per-frame latency and burn a core doing it.
                        // ⚠ Poison-tolerant: a bare `.unwrap()` here means ONE panicking pump
                        // thread poisons the shared receiver and silently kills every OTHER TX
                        // thread — the radio then transmits at a fraction of its rate with no
                        // error anywhere. The MT7921AU pump already did this; the other two copies
                        // of this loop did not, which is what three hand-copies of one loop costs.
                        let buf = match rx.lock().unwrap_or_else(|e| e.into_inner()).recv() {
                            Ok(b) => b,
                            Err(_) => break, // sender dropped
                        };
                        if let Ok(n) = handle.write_bulk(me.tx_endpoint(), &buf, BULK_TX_TIMEOUT) {
                            me.tx_bytes.fetch_add(n as u64, Ordering::Relaxed);
                            me.tx_count.fetch_add(1, Ordering::Relaxed);
                        }
                    }
                })
            })
            .collect()
    }

    /// Stop the TX pump: drops the sender so the pump threads see a closed channel and exit.
    /// `inject` falls back to the synchronous path.
    pub fn stop_tx_pump(&self) {
        *self.tx_sender.lock().unwrap() = None;
    }

    /// Bytes the TX pump has actually written to USB (0 if no pump has run).
    pub fn tx_bytes_written(&self) -> u64 {
        self.tx_bytes.load(Ordering::Relaxed)
    }

    /// Frames the TX pump has actually written to USB — the honest denominator for a
    /// throughput figure, as opposed to how many `inject` calls returned.
    pub fn tx_count_written(&self) -> u64 {
        self.tx_count.load(Ordering::Relaxed)
    }

    pub fn read_rx(&self, buf: &mut [u8]) -> Result<usize, FaceError> {
        match self
            .usb
            .handle()
            .read_bulk(self.usb.ep_in_data(), buf, BULK_RX_TIMEOUT)
        {
            Ok(n) => Ok(n),
            Err(rusb::Error::Timeout) => Ok(0),
            Err(e) => Err(usb_err(e)),
        }
    }

    // ── TX ──────────────────────────────────────────────────────────────────

    /// The TXWI rate word to transmit `frame` at.
    ///
    /// * A [`Reliability::MostRobust`](ndn_radio_hal::Reliability::MostRobust) intent goes
    ///   out **legacy OFDM 6 Mbps** whatever the control plane last set, because that intent
    ///   means "the worst receiver in earshot must decode this" and an HT PPDU excludes
    ///   every legacy-only receiver by construction. Same rule as the Realtek backends.
    /// * Otherwise the rate stored by [`FrameIo::set_rate`] / [`set_legacy_rate`], if any.
    /// * Otherwise legacy OFDM 6 Mbps — the workspace default for an unmeasured link.
    ///
    /// `NDN_RADIO_TX_RATE` overrides everything with a **raw TXWI rate word** (decimal, or
    /// `0x`-prefixed hex): `0x0000` CCK-1M, `0x2000` OFDM-6M, `0x4000` HT-MCS0,
    /// `0x4207` HT-MCS7 + SGI. The env is per-driver by design — a Realtek DESC code and an
    /// mt76 rate word are different encodings and must not be confused.
    fn resolved_rate(&self, frame: &InjectFrame) -> u16 {
        if let Some(v) = std::env::var("NDN_RADIO_TX_RATE").ok().and_then(|s| {
            let s = s.trim().to_ascii_lowercase();
            match s.strip_prefix("0x") {
                Some(hex) => u16::from_str_radix(hex, 16).ok(),
                None => s.parse::<u16>().ok(),
            }
        }) {
            // ☠ **Cross-encoding guard.** `NDN_RADIO_TX_RATE` is one env name meaning FIVE different
            // things: a Realtek DESC code on three backends, a connac2 rate word on the MT7921AU,
            // and an mt76x02 TXWI word here. The workspace's standing remedy for the one-way-link
            // failure is `NDN_RADIO_TX_RATE=4`, which IS legacy 6M as a Realtek DESC code — and
            // here decodes as `phy=CCK, index=4`, a rate that does not exist (CCK is indices 0-3)
            // and could not be used on 5 GHz if it did. It would have gone out silently.
            //
            // A raw TXWI word for any real rate is either a valid CCK index (0-3) or has a non-zero
            // PHY field (OFDM 0x2000, HT 0x4000, VHT 0x8000). Anything between is another driver's
            // number. Warn and fall through rather than air a rate nobody chose.
            const PHY_FIELD: u16 = 0xe000;
            if v & PHY_FIELD == 0 && (v & 0x3f) > 3 {
                eprintln!(
                    "mt7610u: NDN_RADIO_TX_RATE={v} is not an mt76x02 TXWI rate word — it decodes \
                     as CCK index {} (CCK has only 0-3, and none on 5 GHz). This is very likely a \
                     Realtek DESC code; on this part use 0x2000 for OFDM-6M. IGNORING it.",
                    v & 0x3f
                );
            } else {
                return v;
            }
        }
        if frame.tx.needs_basic_rate() {
            return LegacyRate::Ofdm6.rate_val();
        }
        self.cur_rate
            .lock()
            .unwrap_or_else(|e| e.into_inner())
            .unwrap_or_else(|| LegacyRate::Ofdm6.rate_val())
    }

    /// Transmit at a fixed legacy rate from here on — the worst-receiver lever as bearer
    /// state, the legacy sibling of [`FrameIo::set_rate`].
    pub fn set_legacy_rate(&self, rate: LegacyRate) {
        *self.cur_rate.lock().unwrap_or_else(|e| e.into_inner()) = Some(rate.rate_val());
    }

    /// Build the complete USB TX bulk for one bare 802.11 frame:
    /// `[info u32][TXWI 20 B][802.11 (+ L2 pad)][pad to 4][4 B zero]`.
    ///
    /// The layout is spelled out at `mt76x02_usb_core.c:50-61`:
    /// ```text
    ///   |   4B   | xfer len |      pad       |  4B  |
    ///   | TXINFO | pkt/cmd  | zero pad to 4B | zero |
    /// ```
    /// with `TXINFO.len = round_up(xfer_len, 4)`, `DPORT = WLAN_PORT (0)`, and the flags
    /// `mt76x02u_tx_prepare_skb` computes (`:99-103`): `QSEL = MT_QSEL_EDCA (2)`
    /// (`dma.h:147-152`), `MT_TXD_INFO_80211` (bit 19), and `MT_TXD_INFO_WIV` (bit 24, set
    /// whenever there is no hardware key — always, here).
    ///
    /// The **L2 pad** is `mt76_insert_hdr_pad` (`mt76x02_usb_core.c:77`): two zero bytes
    /// after the 802.11 header when the header length is not a multiple of 4, so the body
    /// lands 4-aligned. `len_ctl` excludes it (`mt76x02_txrx.c:152`: `len = skb->len -
    /// (hdrlen & 2)`). Our own frames have 24- or 36-byte headers, so in practice no pad is
    /// inserted — but the rule is implemented rather than assumed, because a 26-byte QoS
    /// header is one `build_dot11` change away.
    pub fn build_tx_bulk(&self, dot11: &[u8], rate: u16) -> Vec<u8> {
        let hdr_len = if dot11.len() >= 2 {
            dot11_hdr_len(dot11[0], dot11[1])
        } else {
            24
        };
        let pad = if hdr_len % 4 == 0 || hdr_len > dot11.len() {
            0
        } else {
            2
        };

        // struct mt76x02_txwi, mt76x02_mac.h:135-148.
        let a = self.ampdu.load(Ordering::Relaxed);
        let (wcid, ba_window) = ((a >> 8) as u8, (a & 0xff) as u8);
        let aggregate = wcid != 0xff;

        let mut txwi = [0u8; TXWI_LEN];
        // MT_TXWI_FLAGS_AMPDU = BIT(4) (mt76x02_mac.h:122). Only set for the aggregation probe;
        // a broadcast frame with no station cannot be aggregated and the bit would be a lie.
        let flags: u16 = if aggregate { 1 << 4 } else { 0 };
        txwi[0..2].copy_from_slice(&flags.to_le_bytes());
        txwi[2..4].copy_from_slice(&rate.to_le_bytes());
        // ack_ctl = 0: broadcast frames request no ACK, so MT_TXWI_ACK_CTL_REQ stays clear
        // (mt76x02_mac.c:409-410 sets it only when TX_CTL_NO_ACK is absent).
        // ack_ctl: BA_WINDOW is [7:2], ACK_CTL_REQ is BIT(0). Request no ACK even when
        // aggregating — see `enable_ampdu` for why asking for one on a peerless bench would
        // make the MAC retry every frame.
        txwi[4] = if aggregate { ba_window << 2 } else { 0 };
        // wcid 0xff = "no station" (mt76x02_mac.c:361-364). There is no peer table on a
        // connectionless bearer, and a data frame sent with a management wcid is dropped.
        txwi[5] = wcid;
        txwi[6..8].copy_from_slice(&(dot11.len() as u16).to_le_bytes()); // len_ctl, pad excluded
        // iv/eiv (8..16) zero: no hardware key. aid (16) zero.
        // txstream (17) zero: mt76x02_mac.c:397-401 only writes 0x13/0x93 when the chainmask
        // has more than one stream. This part is 1x1 — the mt7612 backend's TXWI template
        // carries 0x13 there for exactly that reason, and copying it here would describe a
        // second chain that does not exist.
        txwi[17] = 0;
        // ctl2 = FIELD_PREP(MT_TX_PWR_ADJ, 0) (mt76x02_mac.c:393-395): no per-frame power
        // trim. Power is a channel-level knob here (`phy::set_tx_power`), not a per-frame one.
        txwi[18] = 0;
        txwi[19] = 0; // pktid: we consume no TX status reports

        let xfer_len = TXWI_LEN + dot11.len() + pad;
        let mut buf = Vec::with_capacity(4 + xfer_len + 8);
        buf.extend_from_slice(&tx_info_word(xfer_len).to_le_bytes());
        buf.extend_from_slice(&txwi);
        if pad == 0 {
            buf.extend_from_slice(dot11);
        } else {
            buf.extend_from_slice(&dot11[..hdr_len]);
            buf.extend_from_slice(&[0u8; 2]);
            buf.extend_from_slice(&dot11[hdr_len..]);
        }
        while (buf.len() - 4) % 4 != 0 {
            buf.push(0);
        }
        buf.extend_from_slice(&[0u8; 4]); // the trailing zero word
        buf
    }

    /// Write a pre-built TX bulk to the **AC_BE** bulk-OUT pipe, blocking.
    ///
    /// AC_BE (`mt76.h:654`), not the inband-command pipe: the command pipe carries MCU
    /// messages and firmware chunks, and a data frame pushed into it is not transmitted.
    pub fn tx_raw(&self, bulk: &[u8]) -> Result<(), FaceError> {
        let n = self
            .usb
            .handle()
            .write_bulk(self.tx_endpoint(), bulk, BULK_TX_TIMEOUT)
            .map_err(usb_err)?;
        if n != bulk.len() {
            return Err(io_err(format!(
                "mt7610u TX: short write {n}/{}",
                bulk.len()
            )));
        }
        Ok(())
    }

    /// Transmit one bare 802.11 frame at `rate`. Synchronous; the async path is
    /// [`FrameIo::inject`].
    pub fn transmit(&self, dot11: &[u8], rate: u16) -> Result<(), FaceError> {
        self.tx_raw(&self.build_tx_bulk(dot11, rate))
    }

    /// The next 12-bit 802.11 sequence number. Reserved for a future A-MSDU/QoS path; the
    /// base `build_dot11` fills SeqCtrl itself.
    fn next_seq(&self) -> u16 {
        self.seq.fetch_add(1, Ordering::Relaxed) & 0x0fff
    }

    // ── The mt76 knob layer (isolated so a signature change is a one-line fix) ──

    /// Port TSF, `(dw1 << 32) | dw0` — the corrected word order, **not**
    /// `mt76x02_usb_core.c:155-157`'s. See [`crate::mt76::knobs`].
    fn tsf(&self) -> Result<u64, FaceError> {
        knobs::read_tsf(self)
    }

    /// Read-and-clear idle/busy microseconds for the window since the previous call.
    fn channel_time(&self) -> Result<knobs::ChannelTime, FaceError> {
        knobs::read_channel_time(self)
    }

    /// Read-and-clear `MT_RX_STAT_0`/`_1` error counters for the window since the previous call.
    fn rx_stat(&self) -> Result<knobs::RxStat, FaceError> {
        knobs::read_rx_stat(self)
    }
}

/// Read the whole 512-byte EEPROM shadow over `MT_VEND_READ_EEPROM`.
///
/// MEASURED: 512/512 bytes, 0 errors, word 0 = `0x7610`, bytes 4..9 = the netdev MAC. The
/// `MT_EFUSE_CTRL` block-read path (`mt76x02_eeprom.c`) is therefore not needed on this
/// part — the USB bridge already shadows the efuse.
fn read_eeprom_image(usb: &Mt76Usb) -> Result<Vec<u8>, FaceError> {
    const EEPROM_SIZE: usize = 512;
    let mut out = Vec::with_capacity(EEPROM_SIZE);
    for off in (0..EEPROM_SIZE).step_by(4) {
        let w = usb
            .read_eeprom(off as u16)
            .map_err(|e| io_err(format!("mt7610u: EEPROM read at {off:#05x} failed: {e}")))?;
        out.extend_from_slice(&w.to_le_bytes());
    }
    Ok(out)
}

/// The USB TX info word for a transfer whose TXWI + frame (+ L2 pad) is `xfer_len` bytes.
///
/// `MT_TXD_INFO_LEN = round_up(xfer_len, 4)` (`mt76x02_usb_core.c:56`), `DPORT = WLAN_PORT`
/// (0, so the field contributes nothing), plus the three flags
/// `mt76x02u_tx_prepare_skb` ORs in (`:99-103`): `MT_TXD_INFO_80211` (bit 19),
/// `MT_TXD_INFO_WIV` (bit 24 — set whenever there is no hardware key, which is always here),
/// and `MT_TXD_INFO_QSEL = MT_QSEL_EDCA` (2, `dma.h:147-152`) in bits [26:25].
/// Field masks: `mt76x02_dma.h:12-21`.
const fn tx_info_word(xfer_len: usize) -> u32 {
    ((xfer_len as u32).next_multiple_of(4)) | (1 << 19) | (1 << 24) | (2 << 25)
}

/// `struct { __le32 id; __le32 value; }` — the `CMD_FUN_SET_OP` payload
/// (`mt76x02_mcu.c:85-91`).
fn encode_fun_set_op(func: u32, value: u32) -> [u8; 8] {
    let mut b = [0u8; 8];
    b[0..4].copy_from_slice(&func.to_le_bytes());
    b[4..8].copy_from_slice(&value.to_le_bytes());
    b
}

// ── Trait seams ─────────────────────────────────────────────────────────────

impl Mt76Regs for Mt7610uBackend {
    fn rr(&self, addr: u32) -> Result<u32, FaceError> {
        self.usb.rr(addr)
    }
    fn wr(&self, addr: u32, val: u32) -> Result<(), FaceError> {
        self.usb.wr(addr, val)
    }
}

impl McuBus for Mt7610uBackend {
    /// One raw host->device vendor control write. The transport exposes typed wrappers
    /// (`dev_mode`, `wr_fce`, `power_on`) but the firmware download needs the general form —
    /// notably load-IVB, which is `MT_VEND_DEV_MODE` with a **64-byte body** rather than the
    /// empty-body variant the wrapper sends.
    fn vendor_write(
        &self,
        request: u8,
        value: u16,
        index: u16,
        data: &[u8],
    ) -> Result<(), FaceError> {
        self.usb
            .handle()
            .write_control(
                0x40,
                request,
                value,
                index,
                data,
                std::time::Duration::from_millis(500),
            )
            .map(|_| ())
            .map_err(usb_err)
    }

    fn rr(&self, addr: u32) -> Result<u32, FaceError> {
        self.usb.rr(addr)
    }

    fn wr(&self, addr: u32, val: u32) -> Result<(), FaceError> {
        self.usb.wr(addr, val)
    }

    /// Inband commands and firmware chunks go out on `MT_EP_OUT_INBAND_CMD`
    /// (`mt76x02_usb_mcu.c:95-96`, `:239-240`) — endpoint `0x04` on this dongle.
    fn bulk_out_cmd(&self, buf: &[u8]) -> Result<(), FaceError> {
        let n = self
            .usb
            .handle()
            .write_bulk(self.usb.ep_cmd_out(), buf, BULK_TX_TIMEOUT)
            .map_err(usb_err)?;
        if n != buf.len() {
            return Err(io_err(format!(
                "mt7610u MCU: short command write {n}/{}",
                buf.len()
            )));
        }
        Ok(())
    }

    /// Command responses arrive on `MT_EP_IN_CMD_RESP` (`0x85`), a pipe distinct from the
    /// data RX pipe — so the RX pump and the MCU never steal each other's transfers, which
    /// is the collision the mt7612 backend has to work around with a drain-pause.
    fn bulk_in_resp(&self, buf: &mut [u8]) -> Result<usize, FaceError> {
        match self
            .usb
            .handle()
            .read_bulk(self.usb.ep_in_resp(), buf, MCU_RESP_TIMEOUT)
        {
            Ok(n) => Ok(n),
            Err(rusb::Error::Timeout) => Ok(0),
            Err(e) => Err(usb_err(e)),
        }
    }

    /// 1..=15, never 0 — `__mt76x02u_mcu_send_msg` (`mt76x02_usb_mcu.c:83-86`) skips 0
    /// because seq 0 marks a fire-and-forget command that posts no response.
    fn next_seq(&self) -> u8 {
        let mut s = self.mcu_seq.fetch_add(1, Ordering::Relaxed).wrapping_add(1) & 0xf;
        if s == 0 {
            s = self.mcu_seq.fetch_add(1, Ordering::Relaxed).wrapping_add(1) & 0xf;
            if s == 0 {
                s = 1;
            }
        }
        s
    }
}

impl PhyBus for Mt7610uBackend {
    /// Write one RF bank register through **`MT_RF_CSR_CFG`**, the direct path.
    ///
    /// ★ Upstream routes USB through the MCU register-pair protocol instead
    /// (`mt76x0/phy.c:83-98`), branching on *bus* rather than on capability — so whether
    /// the direct path works over USB had never been settled. The oracle settled it:
    /// `MT_RF_CSR_CFG` reads returned exactly `mt76x0_rf_central_tab`'s values and the
    /// `rf_bw_switch_tab` entry matching the live channel. One EP0 round trip beats an MCU
    /// command, and it works before the MCU is up.
    ///
    /// Sequence and the 100 ms poll windows are `mt76x0_rf_csr_wr` (`phy.c:21-38`); the
    /// lock is upstream's `phy_mutex`, and it matters — two interleaved kicks return each
    /// other's data.
    fn rf_wr(&self, bank_reg: u32, val: u8) -> Result<(), FaceError> {
        let bank = regs::mt_rf_bank(bank_reg);
        let reg = regs::mt_rf_reg(bank_reg);
        if reg > 127 || bank > 8 {
            return Err(io_err(format!("mt7610u: RF({bank},{reg}) out of range")));
        }
        let _g = self.rf_lock.lock().unwrap_or_else(|e| e.into_inner());
        self.poll(regs::MT_RF_CSR_CFG, regs::MT_RF_CSR_CFG_KICK, 0, 100)?;
        self.wr(
            regs::MT_RF_CSR_CFG,
            regs::field_prep(regs::MT_RF_CSR_CFG_DATA, u32::from(val))
                | regs::field_prep(regs::MT_RF_CSR_CFG_REG_BANK, bank)
                | regs::field_prep(regs::MT_RF_CSR_CFG_REG_ID, reg)
                | regs::MT_RF_CSR_CFG_WR
                | regs::MT_RF_CSR_CFG_KICK,
        )
    }

    /// Read one RF bank register — `mt76x0_rf_csr_rr` (`mt76x0/phy.c:40-81`). The
    /// bank/reg echo check is upstream's (`:69-71`) and is load-bearing: the register keeps
    /// the *previous* transaction's data if the kick did not complete, so without it a
    /// stale value reads as a successful one.
    fn rf_rr(&self, bank_reg: u32) -> Result<u8, FaceError> {
        let bank = regs::mt_rf_bank(bank_reg);
        let reg = regs::mt_rf_reg(bank_reg);
        if reg > 127 || bank > 8 {
            return Err(io_err(format!("mt7610u: RF({bank},{reg}) out of range")));
        }
        let _g = self.rf_lock.lock().unwrap_or_else(|e| e.into_inner());
        self.poll(regs::MT_RF_CSR_CFG, regs::MT_RF_CSR_CFG_KICK, 0, 100)?;
        self.wr(
            regs::MT_RF_CSR_CFG,
            regs::field_prep(regs::MT_RF_CSR_CFG_REG_BANK, bank)
                | regs::field_prep(regs::MT_RF_CSR_CFG_REG_ID, reg)
                | regs::MT_RF_CSR_CFG_KICK,
        )?;
        self.poll(regs::MT_RF_CSR_CFG, regs::MT_RF_CSR_CFG_KICK, 0, 100)?;
        let v = self.rr(regs::MT_RF_CSR_CFG)?;
        if regs::field_get(regs::MT_RF_CSR_CFG_REG_ID, v) != reg
            || regs::field_get(regs::MT_RF_CSR_CFG_REG_BANK, v) != bank
        {
            return Err(io_err(format!(
                "mt7610u: RF({bank},{reg}) read echoed bank {} reg {} — stale transaction",
                regs::field_get(regs::MT_RF_CSR_CFG_REG_BANK, v),
                regs::field_get(regs::MT_RF_CSR_CFG_REG_ID, v),
            )));
        }
        Ok(regs::field_get(regs::MT_RF_CSR_CFG_DATA, v) as u8)
    }

    fn mcu_wr_rp(&self, base: u32, pairs: &[RegPair]) -> Result<(), FaceError> {
        mcu::wr_rp(self, base, pairs)
    }

    fn eeprom(&self) -> &Mt76x0Eeprom {
        &self.eeprom
    }

    /// ★ **`Csr`, not upstream's `Mcu`** — and this is a measurement, not a preference.
    ///
    /// Upstream picks the MCU register-pair path for every USB mt76x0 by branching on *bus
    /// type* (`mt76x0/phy.c:100-117`), never on whether the direct path works, so nothing in
    /// the tree records an answer either way. The oracle produced one: reading `RF(0,1)`,
    /// `RF(0,2)`, `RF(0,4)` and `RF(7,73)` through `MT_RF_CSR_CFG` over USB returned
    /// `0x01 / 0x11 / 0x30 / 0x34` — exactly `mt76x0_rf_central_tab` — and `RF(7,6)` returned
    /// `0x40`, which is the `rf_bw_switch_tab` entry for `RF_A_BAND | RF_BW_20`, matching the
    /// channel the interface was actually tuned to. Values that specific cannot come from a
    /// dead register.
    ///
    /// Why it is worth the divergence: one EP0 round trip (~151 µs) instead of an MCU command
    /// plus its response, and — more importantly — it works **before the MCU is running**, so
    /// RF init is not ordered behind firmware. If a future part disagrees, flip this to `Mcu`
    /// and `mcu_wr_rp` above carries the whole load; both paths are implemented.
    fn rf_path(&self) -> phy::RfPath {
        phy::RfPath::Csr
    }

    /// One MCU command. Needed by the calibration ladder and the USB bandwidth select, which
    /// have no register-level substitute — the trait's default deliberately fails loudly
    /// rather than skipping calibration, since an uncalibrated radio still transmits, badly.
    fn mcu_send(&self, cmd: u8, data: &[u8], wait_resp: bool) -> Result<(), FaceError> {
        mcu::mcu_send(self, cmd, data, wait_resp)
    }

    /// One little-endian EEPROM word, mirroring `mt76x02_eeprom_get`. The PHY reads raw words
    /// rather than parsed fields on purpose, so its math stays a line-for-line mirror of
    /// upstream and does not couple to this port's parse shape.
    fn eeprom_word(&self, addr: u16) -> u16 {
        self.eeprom.word(addr as usize)
    }
}

// ── RX pump ─────────────────────────────────────────────────────────────────

/// The mt76x0 side of the shared RX pipeline.
///
/// **One RX unit per transfer — and that is MEASURED, not assumed.** `mt76x0_init_usb_dma`
/// clears `MT_USB_DMA_CFG_RX_BULK_AGG_EN` with the comment *"disable AGGR_BULK_RX in order
/// to receive one frame in each rx urb and avoid copies"* (`mt76x0/usb.c:55-58`), and the
/// oracle read bit 21 clear on the live kernel interface. The loop below still walks the
/// DMA length field rather than treating the whole transfer as one unit, so a device that
/// ever did aggregate would be de-aggregated correctly instead of silently truncated after
/// the first frame — but it does not loop twice today, and this is stated as a fact rather
/// than left as the open question the mt7612 backend carries.
impl crate::rx_pump::Pumpable for Mt7610uBackend {
    fn pump_handle(&self) -> Arc<DeviceHandle<Context>> {
        self.usb.handle()
    }

    fn pump_bulk_in(&self) -> u8 {
        self.usb.ep_in_data()
    }

    fn pump_state(&self) -> &crate::rx_pump::RxPumpState {
        &self.rx
    }

    /// Split one bulk-IN transfer into the frames it carried:
    /// `[4 B DMA header][32 B RXWI][802.11 (+ L2 pad)][4 B FCE trailer]`.
    fn parse_transfer(&self, buf: &[u8]) -> Vec<CapturedFrame> {
        let mut out = Vec::new();
        let mut off = 0usize;
        let is_5ghz = self.channel.load(Ordering::Relaxed) > 14;
        let _bw_index = self.bw.load(Ordering::Relaxed);

        while off < buf.len() {
            let Some(u) = decode_rx_unit(&buf[off..]) else {
                // Not a decodable descriptor. If we are at the very start this is a short or
                // malformed transfer worth counting; past the first unit it is just the tail
                // of a padded transfer.
                if off == 0 {
                    self.rx_undecodable.fetch_add(1, Ordering::Relaxed);
                }
                break;
            };

            // ★ Every RX unit pulled off USB, counted BEFORE the CRC and format filters, so
            // `rx_raw_frames()` is directly comparable to a kernel monitor's `rx_packets`.
            // A backend that joins the pump and forgets this reports a structural 0, which
            // reads exactly like a dead receiver — see the warning on `crate::RX_RAW_FRAMES`.
            crate::RX_RAW_FRAMES.fetch_add(1, Ordering::Relaxed);
            self.rx_units.fetch_add(1, Ordering::Relaxed);
            if u.pad != 0 {
                self.rx_l2pad.fetch_add(1, Ordering::Relaxed);
            }

            let unit = &buf[off..];
            off += u.unit_len;

            if u.crc_err {
                // The hardware demodulated a PPDU and its FCS failed — a collision or a
                // marginal link. Counted (it is signal about the channel), never delivered.
                self.rx_crc_errors.fetch_add(1, Ordering::Relaxed);
                continue;
            }
            let Some(mpdu) = rx_mpdu(unit, &u) else {
                self.rx_undecodable.fetch_add(1, Ordering::Relaxed);
                continue;
            };

            let rate = decode_rate_word(u.rate);
            // RSSI: the raw per-chain byte plus the EEPROM's per-band LNA gain and offset
            // (`mt76x02_mac_get_rssi`, mt76x02_mac.c:760-768). Only chain 0 exists on a 1x1
            // part, so there is no second chain to report.
            // ⚠ Third parameter is the **RX chain index**, not a bandwidth index — `rssi_offset[]`
            // is indexed by chain and never by bandwidth (the contract is spelled out on
            // `Eeprom::rssi_dbm`). This passed `bw_index`, so at 40/80 MHz it read the offset of a
            // chain that does not exist on this 1x1 part (returning 0) and reported a wrong RSSI
            // in exactly the configurations we most want to measure. Chain 0 is the only one here.
            let rssi = Some(self.eeprom.rssi_dbm(u.rssi_raw, is_5ghz, 0));
            // `phy` stays None: see the module header. The RXWI's four `bbp_rxinfo` dwords
            // are the only candidate source of SNR/EVM/CFO here and nothing upstream reads
            // them, so there is no field to fill honestly yet.
            if let Some(f) = crate::frame::parse_dot11(self.format, &mpdu, rssi, rate.mcs, None) {
                self.rx_accepted.fetch_add(1, Ordering::Relaxed);
                self.rx_ok_window.fetch_add(1, Ordering::Relaxed);
                out.push(f);
            }
        }
        out
    }
}

/// Largest `RawNdn` payload this part will actually put on air.
///
/// ★ MEASURED ON AIR 2026-08-31 with a witness receiver, VHT MCS0 / Bw80 (rate chosen low enough
/// that the offered frame rate stays under the monitor's capture ceiling, so the ratio below is a
/// real delivery ratio and not the witness saturating):
///
/// ```text
///   payload   MPDU = payload+36   delivered to the witness
///    11410          11446                96.1 %
///    11418          11454                95.8 %      <- MAX_MPDU_PAYLOAD
///    11419          11455                 0.0 %
///    11430          11466                 0.0 %
///    11454          11490                 0.0 %
/// ```
///
/// The cliff is exactly the **802.11 maximum MPDU of 11454 B**, and it lands to the byte on
/// `payload + 36` — 24 B 802.11 header + 6 B LLC/SNAP + 2 B ethertype + the 4 B FCS the MAC
/// appends. Below it delivery is a flat 93-97% from 5650 B up, so there is no size-dependent
/// degradation approaching the limit; it is a hard edge, not a slope.
///
/// ⚠ **Why the guard below exists.** Over the limit the hardware discards the frame and reports
/// nothing: at 11419 B the host still printed `1206 frames ... (0 err), 27.46 Mbit/s` while the
/// witness saw **zero**. A TX counter counts USB writes the device accepted, not radiation
/// (see the same trap recorded for the whole part). Silently transmitting nothing is the worst
/// failure mode available, so it is refused loudly instead.
pub const MAX_MPDU_PAYLOAD: usize = 11_418;

// ── FrameIo ─────────────────────────────────────────────────────────────────

#[async_trait]
impl FrameIo for Mt7610uBackend {
    /// This radio's own capability, so a face built from the bare `dyn FrameIo` does not have to
    /// invent one. Delegates to this type's [`RadioProfile`] — the single source of truth.
    fn radio_capability(&self) -> Option<ndn_radio_hal::RadioCapability> {
        Some(<Self as ndn_radio_hal::RadioProfile>::capability(self))
    }
    async fn inject(&self, frame: InjectFrame) -> Result<(), FaceError> {
        // MEASURED cliff — see [`MAX_MPDU_PAYLOAD`]. One byte over and the frame never airs while
        // every host-side counter reports success, so this is refused rather than silently lost.
        if frame.payload.len() > MAX_MPDU_PAYLOAD {
            return Err(io_err(format!(
                "mt7610u: payload {} B exceeds MAX_MPDU_PAYLOAD {MAX_MPDU_PAYLOAD} (802.11 caps \
                 the MPDU at 11454 B and this frame carries 36 B of header+FCS). MEASURED: one \
                 byte over this and the radio transmits NOTHING while still reporting success. \
                 Fragment above this seam.",
                frame.payload.len()
            )));
        }
        let dot11 = crate::frame::build_dot11(self.format, &frame)?;
        let rate = self.resolved_rate(&frame);
        let buf = self.build_tx_bulk(&dot11, rate);
        // Fast path: hand the bulk to the pump (bounded queue = real backpressure) and let
        // `depth` transfers be in flight at once. Without it every frame pays a full
        // synchronous USB round trip and the radio idles between PPDUs.
        if let Some(s) = self.tx_sender.lock().unwrap().clone() {
            return s
                .send(buf)
                .map_err(|_| io_err("mt7610u: TX pump closed".into()));
        }
        let handle = self.usb.handle();
        let ep = self.tx_endpoint();
        tokio::task::spawn_blocking(move || {
            handle
                .write_bulk(ep, &buf, BULK_TX_TIMEOUT)
                .map_err(usb_err)
                .and_then(|n| {
                    (n == buf.len())
                        .then_some(())
                        .ok_or_else(|| io_err(format!("mt7610u TX: short write {n}/{}", buf.len())))
                })
        })
        .await
        .map_err(|e| io_err(format!("mt7610u TX: join {e}")))?
    }

    /// Rate as bearer state: the exact TXWI rate word every subsequent [`inject`] uses.
    ///
    /// Clamped to what a 1×1 part can build — see [`mt76_rate_val`]. A `MostRobust` frame
    /// still overrides this with legacy OFDM 6 Mbps, so control traffic stays decodable by
    /// a legacy-only neighbour no matter what the control plane set.
    fn set_rate(&self, mcs: McsDescriptor) -> Result<(), FaceError> {
        // The width is read at set_rate time from what the radio is tuned to; `set_channel`
        // re-stamps any stored rate word so a later retune cannot leave a stale width behind.
        let bw_code = self.bw.load(Ordering::Relaxed);
        *self.cur_rate.lock().unwrap_or_else(|e| e.into_inner()) =
            Some(mt76_rate_val(&mcs, bw_code));
        Ok(())
    }

    /// One plain MPDU per NDN packet.
    ///
    /// No A-MSDU bundling: host-built A-MSDU is firmware-gated on monitor injection on the
    /// sibling MT7612U (verified 0/200 on air there), and nothing has tested it here. Rather
    /// than inherit a neighbouring chip's unverified claim in either direction, this sends
    /// what is known to work and leaves the aggregation question to a measurement.
    async fn inject_batch(&self, frames: Vec<InjectFrame>) -> Result<(), FaceError> {
        for f in frames {
            self.inject(f).await?;
        }
        Ok(())
    }

    async fn recv_frame(&self) -> Result<CapturedFrame, FaceError> {
        // Pumped mode: background threads fill the shared queue; just drain it.
        if self.rx.is_pumped() {
            return Ok(self.rx.recv().await);
        }
        loop {
            if let Some(f) = self.rx.try_pop() {
                return Ok(f);
            }
            let handle = self.usb.handle();
            let ep = self.usb.ep_in_data();
            let got = tokio::task::spawn_blocking(move || {
                // 8 KB: RX aggregation is off, so one unit per transfer, but an A-MSDU MPDU
                // from a foreign network can reach ~7935 B and a short buffer would truncate
                // it into an undecodable descriptor rather than a dropped frame.
                let mut b = vec![0u8; 8192];
                match handle.read_bulk(ep, &mut b, BULK_RX_TIMEOUT) {
                    Ok(n) if n > 0 => {
                        b.truncate(n);
                        Ok(Some(b))
                    }
                    Ok(_) | Err(rusb::Error::Timeout) => Ok(None),
                    Err(e) => Err(usb_err(e)),
                }
            })
            .await
            .map_err(|e| io_err(format!("mt7610u recv_frame: join {e}")))??;
            if let Some(b) = got {
                self.rx
                    .push(crate::rx_pump::Pumpable::parse_transfer(self, &b));
            }
        }
    }
}

// ── RadioKnobs ──────────────────────────────────────────────────────────────

impl RadioKnobs for Mt7610uBackend {
    /// Tune to `channel` at 20, 40 or 80 MHz.
    ///
    /// ★ 20/40/80 are all MEASURED on air by a witness receiver (2026-08-31); [`declared_capability`]
    /// reports `max_bw: 2` to match. 40 and 80 MHz live in the RF/BBP switch tables and in
    /// `mt76x0_phy_set_channel`, but selecting either needs the control-channel offset
    /// (`chandef->center_freq1`, `mt76x0/phy.c:949-968`) to compute `ch_group_index`, and
    /// this seam carries only `(channel, bw)`. Guessing the offset would put the secondary
    /// channel in the wrong place — silently, on air.
    ///
    /// Rejects a channel with no PLL program too, rather than tuning something adjacent: a
    /// synthesiser that was never programmed looks exactly like a quiet channel.
    fn set_channel(&self, channel: u8, bw: Bandwidth) -> Result<(), FaceError> {
        // ★★ **The 20-MHz-only guard that used to sit here was wrong, and its stated reason was
        // false.** It read "40/80 MHz need the control-channel offset this seam does not carry" —
        // but [`phy::ht40_secondary_above`] and [`phy::VHT80_GROUPS`] derive exactly that offset
        // from the control channel alone, sixty lines from the guard, and
        // [`phy::set_channel_ext`] has implemented 20/40/80 end to end all along: the BBP width,
        // the band's UPPER_40M flag, the `EXT_CCA_CHAN` permutation, the width-dependent EEPROM
        // power delta and the `RF_BW_40`/`RF_BW_80` switch-table rows. It is unit-tested against
        // upstream's `channel += 6 - ch_group_index * 4` arithmetic.
        //
        // The cost of the guard was not just a missing width. Cognition reads `vht` as
        // `cap.max_bw() >= 2`, so declaring 20 MHz made a VHT-capable part transmit as 802.11n,
        // and `RadioCapability::rate_rank` never saw the 4x lever at all. On the sibling
        // MT7921AU, opening the same gate took MEASURED throughput from 112 to 174 Mbit/s.
        //
        // Narrowband 5/10 MHz genuinely is absent from this path — say so specifically rather
        // than refusing everything that is not 20.
        // ★★ **RE-GATED 2026-08-28 ON MEASUREMENT, after I un-gated it on a code-read.**
        //
        // An audit established that `phy::set_channel_ext` implements 20/40/80 end to end, that
        // `ht40_secondary_above` / `VHT80_GROUPS` derive the centre the old guard claimed was
        // missing, and that unit tests prove the arithmetic matches upstream. All true — and
        // none of it is evidence the sequence works on silicon. It does not:
        //
        //   ch149 Bw20 -> 2732 f/s      ch36 Bw20 -> 2818 f/s
        //   ch36  Bw40 -> MCU command 0x1f (CALIBRATION_OP) returns a 0-byte response
        //   ch36  Bw80 -> same failure
        //
        // and the failure leaves the MCU stuck for every later command until the kernel driver
        // re-initialises the chip. One Bw80 run did complete earlier and reported 2719 f/s —
        // indistinguishable from Bw20's 2732, i.e. it was transmitting at 20 MHz anyway, the
        // same silent width-clamp the MT7921AU's sniffer config was doing.
        //
        // The tests were real and tested the wrong thing: centre-channel arithmetic, not the
        // MCU sequence. Declaration follows the actuator, and the actuator is 20 MHz.
        // ★★★ **UN-GATED 2026-08-31 ON A WITNESS RECEIVER — the confirmation this refusal demanded.**
        //
        // Witness: o5p-0's RTL8812AU under the kernel `rtw88_8812au` driver, monitor mode on
        // channel 36, reading the radiotap of what the MT7610U (o5p-1) actually put in the air.
        // A transmitter cannot see its own PPDU width, so the width is read off the receiver.
        // Three arms, same channel, same 1400 B payload, our SA 02:4e:44:4e:00:01:
        //
        //   Bw20 -> 25948/25948 frames "MCS 7 20 MHz"           (65.0 Mb/s)
        //   Bw40 -> 19256/19256 frames "MCS 7 40 MHz"           (135.0 Mb/s)
        //   Bw80 -> 19386/19386 frames "MCS 7 BCC FEC 80 MHz"   (VHT)
        //
        // 100% in every arm, no mixing. The rate corroborates the width independently: HT MCS7
        // 1SS long-GI is 65.0 Mb/s at 20 MHz and 135.0 at 40, and the receiver derives that from
        // the PPDU it demodulated, not from what we asked for. The MCU calibration that used to
        // return a 0-byte response and wedge the chip did not fire once across the three arms.
        //
        // What changed since the 2026-08-28 re-gate: `rate_bw_field` now writes the TXWI rate
        // word's BW field. It was never written before — which is why the historical
        // "Bw80 -> 2719 f/s vs Bw20's 2732" was **20 MHz PPDUs on an 80 MHz channel**, the width
        // lever having never actually been pulled. A failed `MCU_CAL_FULL` also no longer latches
        // ALC off and the baseband override across processes (`phy::set_channel_ext`).
        //
        // ★ Width converts into goodput — but ONLY above the payload where airtime starts to
        // dominate the fixed per-PPDU cost. MEASURED the same day, VHT MCS7 1SS, ch36 (Mbit/s):
        //
        //           1400 B   3000 B   5650 B   7000 B
        //   Bw20     34.4     50.5     55.9     57.1
        //   Bw40     33.6     64.0     82.7     99.8   (+75% over Bw20)
        //   Bw80     33.1     67.9     84.7    105.2   (+84% over Bw20)
        //
        // At 1400 B the three widths are indistinguishable, and reading ONLY that row is how this
        // comment previously came to say "width buys no goodput" — wrong, and wrong in the
        // direction that would have retired a working lever. (Those figures predate both the TX
        // pump and the bring-up EDCA write; with BOTH in place, width at 1400 B pumped under
        // `Shared` measures 26.9 / 34.9 / 43.2 Mbit/s for 20/40/80 — still +61%.) The per-frame period is
        // roughly `fixed + airtime(payload, width)` with `fixed` ≈ 300-400 µs; at 1400 B the
        // 20 MHz airtime is ~172 µs, so the fixed term swamps the very thing being varied.
        // Measure a width knob at the LARGEST payload, never the smallest.
        //
        // Peak on this part: **139.0 Mbit/s** at 11000 B / Bw80 (was 73.8 on record). VHT MCS
        // 7/8/9 at 11000 B/Bw80 all land within noise (130.4 / 128.8 / 133.1), so above ~7 kB
        // the link is bound by the fixed per-PPDU cost, not by rate and not by airtime — which
        // is why `max_mcs` stays at 7 until a witness confirms MCS8/9 on air and they buy something.
        if !freq_plan::FREQUENCY_PLAN
            .iter()
            .any(|f| f.channel == channel)
        {
            return Err(io_err(format!(
                "mt7610u: channel {channel} has no PLL program in the mt76x0 frequency plan"
            )));
        }
        phy::set_channel(self, channel, bw)?;
        self.channel.store(channel, Ordering::Relaxed);
        // The EEPROM's 5 GHz LNA gain is per sub-band group and is selected from the channel it was
        // last told about. Without this it stays pinned at group 0 for the whole session, so every
        // RSSI on a high 5 GHz channel carries the low group's gain.
        self.eeprom.set_channel(channel);
        self.bw.store(bw.code(), Ordering::Relaxed);
        // ★ Re-stamp any stored rate word with the new width. Rate and width are set through two
        // independent seams (`FrameIo::set_rate` and `RadioKnobs::set_channel`) and either can move
        // last; without this a retune after set_rate would transmit at the OLD width — the same
        // stale-shadow class of bug as leaving the field at 0 in the first place.
        {
            let mut cur = self.cur_rate.lock().unwrap_or_else(|e| e.into_inner());
            if let Some(rate) = *cur {
                let phy = (rate >> 13) & 0x7;
                *cur = Some((rate & !(0x3 << 7)) | rate_bw_field(phy, bw.code()));
            }
        }
        // The idle/busy and RX_STAT counters accumulated on the *previous* channel. They are
        // read-and-clear, so one discarded read is all it takes to keep the first occupancy
        // window after a hop from being charged to the wrong channel. (Upstream reaches for
        // `mt76x02_mac_cc_reset` here, `mt76x0/main.c:21`.)
        let _ = self.channel_time();
        let _ = self.rx_stat();
        Ok(())
    }

    /// TX power on the chip's opaque index scale.
    ///
    /// Forwarded to `phy::set_tx_power`, which resolves it against the EEPROM's per-rate
    /// calibration for the current channel — power on this part is per-channel *and*
    /// per-rate (`mt76x0_get_tx_power_per_rate`), so it cannot be set before a tune.
    fn set_tx_power(&self, req: PowerRequest) -> Result<AppliedPower, FaceError> {
        let ch = self.channel.load(Ordering::Relaxed);
        if ch == 0 {
            return Err(io_err("mt7610u: set_tx_power before set_channel".into()));
        }
        // ⚠ There is no second (raw) axis on this part: `phy::set_tx_power` always resolves against
        // the EEPROM per-rate calibration, so `Raw` reaches the same writer. Recorded as
        // `DriverReference` rather than `FusedBase`, because what this knob writes is a per-rate
        // TXAGC target derived from the EEPROM — the fuse read is inside `phy`, and this driver has
        // never separated "the adapter's fused base" from "the vendor's reference".
        let idx = match &req {
            PowerRequest::Ceiling(_) => 63u32,
            PowerRequest::Index(i, _) => *i as u32,
            PowerRequest::Raw { idx, .. } => *idx as u32,
            PowerRequest::Dbm(d) => {
                let applied = phy::set_tx_power(self, ch, Some(*d), None)?;
                return Ok(AppliedPower::absolute_dbm(
                    req.clone(),
                    applied,
                    applied != *d,
                ));
            }
            PowerRequest::NoActuator => {
                return Err(ndn_radio_hal::bringup::power_unsupported(
                    "mt7610u: PowerRequest::NoActuator, but this part DOES actuate power.",
                ));
            }
        };
        let want = idx;
        let idx = idx.min(63);
        let applied = phy::set_tx_power(self, ch, None, Some(idx as u8))?;
        let mut p = AppliedPower::from_writes(
            req.clone(),
            PowerReference::DriverReference {
                source: "mt76x0 EEPROM per-rate target power (mt76x0_get_tx_power_per_rate), \
                         half-dB units — per-channel AND per-rate, so it cannot be set before a tune",
                slope_db_per_idx: None,
            },
            idx as u8,
            idx != want,
            vec![PowerWrite {
                reg: 0x1314,
                value: idx as u8,
                group: "MT_TX_PWR_CFG (per-rate)",
                path: 0,
            }],
        );
        // ⚠ `phy::set_tx_power` returns an EEPROM-derived half-dB figure, and `declared_capability`
        // still reports `tx_power_dbm: None` — the two are not in contradiction: nothing on THIS
        // silicon has been measured against a meter. So the value is recorded on the applied power
        // (where a reader can see the driver's own arithmetic) and NOT promoted into the capability
        // a planner budgets link margin from.
        p.dbm = Some(applied);
        Ok(p)
    }

    /// TX power on the absolute dBm scale, returning what was actually applied.
    ///
    /// ⚠ **Implemented, but [`declared_capability`] still reports `tx_power_dbm: None`**,
    /// and the two are not in contradiction. The EEPROM does carry a factory per-rate power
    /// calibration in half-dB units, so `phy::set_tx_power` has a real scale to resolve
    /// against — but nothing on *this* silicon has been measured against a meter, and
    /// `RadioCapability::tx_power_dbm` is the field a planner budgets link margin from. The
    /// same reasoning kept the RTL8733BU's range `None` even after its absolute anchor was
    /// measured, because the index→dB map was not reproducible across bring-ups.
    ///
    /// Closing it needs what closed the 8733b's anchor: a substitution measurement against a
    /// radio whose applied power is independently reported, on this part, on this bench.
    /// Until then a caller that asks gets an honest applied value and the planner is not
    /// told a range it would believe.
    fn set_tx_power_dbm(&self, dbm: i8) -> Result<i8, FaceError> {
        let ch = self.channel.load(Ordering::Relaxed);
        if ch == 0 {
            return Err(io_err(
                "mt7610u: set_tx_power_dbm before set_channel".into(),
            ));
        }
        phy::set_tx_power(self, ch, Some(dbm), None)
    }

    /// **Frame-free channel occupancy**, as busy per mille of the window since the last call.
    ///
    /// ★ MEASURED: `MT_CH_IDLE` (0x1130) and `MT_CH_BUSY` (0x1134) are **read-and-clear
    /// microsecond** counters — `(idle + busy) / elapsed` came out 1.00, 1.00, 1.01, 1.00
    /// over four consecutive 100 ms windows. That gives a true duty cycle, not a proxy: no
    /// frame is decoded, so an interferer that our PHY cannot demodulate still shows up.
    ///
    /// ⚠ **Do not difference two of these.** The trait's usual contract is a free-running
    /// counter sampled twice; this hardware clears on read, so the value returned is already
    /// the level for the elapsed window and a difference of two samples is meaningless. The
    /// permille normalisation is what makes that safe — a differenced permille is visibly
    /// nonsense, whereas a differenced raw microsecond count would look plausible.
    ///
    /// (`MT_ED_CCA_TIMER` 0x1140 is a second, energy-detect-only busy counter that ticks
    /// independently — MEASURED 762-3902 µs where `CH_BUSY` ran 2718-14112. It is a
    /// different question, and is left to `mt76::knobs` rather than folded in here.)
    /// Contention window as a posture — the actuator for the slot decision.
    ///
    /// Shared mt76x02 implementation ([`crate::mt76::knobs::set_contention`]); the clamp there is
    /// load-bearing, since a zero window on this register family is MEASURED to stop the MAC
    /// transmitting entirely rather than to speed it up.
    fn set_contention(
        &self,
        posture: ndn_radio_hal::ContentionPosture,
    ) -> Result<ndn_radio_hal::ContentionApplied, FaceError> {
        crate::mt76::knobs::set_contention(
            self,
            posture,
            crate::mt76::Family::Mt76x0,
            &self.edca_saved,
        )
    }

    fn read_channel_activity(&self) -> Result<Option<u16>, FaceError> {
        Ok(Some(crate::mt76::knobs::busy_permille(
            &self.channel_time()?,
        )))
    }

    /// `(ok, err)` PPDU counts for the window since the last call.
    ///
    /// `err` is the hardware's: `MT_RX_STAT_0`'s CRC-error and PHY-error halves summed —
    /// PPDUs the baseband began to demodulate and failed, which is exactly the
    /// collision / marginal-decode signature the trait is after. MEASURED read-and-clear.
    ///
    /// ⚠ `ok` is **not** a hardware counter, because this part has none: `MT_RX_STAT_0/1/2`
    /// count only errors (CRC, PHY, CCA, PLCP, duplicate, overflow). It is this driver's own
    /// count of RX units accepted off USB in the same window. Reporting a structural `0`
    /// instead would have been "honest" and useless — a permanent zero on the good half
    /// reads exactly like a dead receiver, which is the trap `crate::RX_RAW_FRAMES`
    /// documents having actually cost a debugging session. Both halves cover the same
    /// window because both are cleared here.
    fn read_ofdm_counters(&self) -> Result<Option<(u16, u16)>, FaceError> {
        let s = self.rx_stat()?;
        // Widened before the add so a saturating window cannot silently wrap, then clamped
        // into the trait's u16. (The `u32::from` also makes this indifferent to whether the
        // knob layer reports these as u16 or u32.)
        let err = u32::from(s.crc_err)
            .saturating_add(u32::from(s.phy_err))
            .min(u32::from(u16::MAX)) as u16;
        let ok = self
            .rx_ok_window
            .swap(0, Ordering::Relaxed)
            .min(u64::from(u16::MAX)) as u16;
        Ok(Some((ok, err)))
    }

    /// Listen-before-talk on/off, via `MT_TXOP_CTRL_CFG`'s `ED_CCA_EN` (bit 20).
    ///
    /// `on = true` **clears** the bit so the transmitter stops deferring to energy on the
    /// medium — the owned-spectrum case where a slot or a token is the collision avoidance
    /// and CSMA only adds jitter. `on = false` restores it.
    ///
    /// Note the direction against the hardware default: `mt76x0_mac_stop` clears this bit
    /// (`mt76x0/init.c:140`) and the MEASURED kernel monitor leaves `MT_TXOP_CTRL_CFG =
    /// 0x0000_583f`, i.e. **ED-CCA already off**. So this knob's `true` is the state the
    /// part is normally found in, and `false` is the one that changes behaviour.
    ///
    /// ⚠ Do not read this as "ignoring CCA raises throughput". MEASURED on the 8812au: on a
    /// saturated channel, defeating carrier sense trades collision loss for TX starvation
    /// and delivered frames fell 237→26/s. It is a knob for a channel you own, not a
    /// contention remedy.
    fn set_edcca_ignore(&self, on: bool) -> Result<(), FaceError> {
        if on {
            self.rmw(regs::MT_TXOP_CTRL_CFG, regs::MT_TXOP_ED_CCA_EN, 0)?;
        } else {
            self.rmw(regs::MT_TXOP_CTRL_CFG, 0, regs::MT_TXOP_ED_CCA_EN)?;
        }
        Ok(())
    }

    /// `BestEffort`. This part has a hardware TSF (MEASURED, 1 µs) but no ported path from
    /// that TSF to a *gated transmit* — nothing here arms a timer that keys the PA. Promising
    /// `PromptBounded` on the strength of the EDCCA knob alone would be a Cut-2 capability
    /// the scheduler reads and acts on, backed by no mechanism.
    fn tx_discipline(&self) -> TxDiscipline {
        TxDiscipline::BestEffort
    }
}

// ── RadioTime ───────────────────────────────────────────────────────────────

/// **The MT7610U's declared time surface** — one clock: the **port TSF** at
/// `MT_TSF_TIMER_DW0/DW1`. A free function so the declaration is assertable in a unit test without a
/// USB device.
///
/// * `tick_ns = 1_000` — MEASURED 1.000 µs/tick, five windows against the host clock, once
///   `TIMER_EN` is set ([`enable_tsf`](Mt7610uBackend::enable_tsf)).
/// * `precision_ns = 151_000` — **not** the 1 µs the tick would suggest, and not
///   `RadioTimeSource::port_tsf`'s hardcoded default. The counter is only reachable through an EP0
///   vendor request, MEASURED at **151 µs** per round trip (200/200), and a value you cannot read
///   more precisely than 151 µs is not a 1 µs value. Declaring the tick as the precision is the
///   shape of error that made every RX-stamp-derived duration on the 8733b 4× short.
/// * `monotonic = true` — earned, not assumed: [`enable_tsf`] clears `SYNC_MODE`, so no received
///   beacon can slam the counter. A beacon-resynced TSF would not qualify.
/// * `read_now = true`, and there is **no per-frame RX stamp**: the RXWI carries no documented
///   timestamp field (`mt76x02_mac.h:97-108`), so `CapturedFrame::stamp` is `None` and this radio
///   declares no [`ndn_time::RadioClockKind::FreeRunRxStamp`] — which is what makes
///   `FaceTimeProfile::can_common_view` correctly `false` for it. The four undecoded `bbp_rxinfo`
///   dwords are the only place such a stamp could hide, and `examples/mt7610_bringup.rs` stage 7
///   exists to look.
/// * `reference = Crystal` — witnessed by code in THIS driver's own bring-up path, see below.
///
/// ## The crystal citation, and what it is NOT
///
/// The chip is gated on a crystal starting, and this port depends on that gate:
/// [`Mt7610uBackend::chip_onoff`] and [`setup_monitor_rx`](Mt7610uBackend::setup_monitor_rx)'s
/// `WLAN_EN` re-assert both poll `MT_CMB_CTRL` for `XTAL_RDY | PLL_LD`, and this port turns a
/// timeout into an ERROR ("mt7610u: XTAL/PLL never came ready") where upstream logs and continues;
/// `mcu::chip_onoff` polls the same pair. A part whose MAC will not accept a register write until
/// its crystal-ready bit asserts is running on a crystal — [`ndn_radio_hal::ClockReferenceKind`]'s
/// `Crystal` doc names exactly this ("the part's clock tree is gated on a crystal-ready bit") as
/// evidence.
///
/// ⚠ Corrected 2026-08-31: this used to cite `MT_XO_CTRL5`/`MT_XO_CTRL6`'s C2 load-capacitance pair
/// and `MT_EE_XTAL_TRIM_1/2`. Those exist only as `pub const`s — nothing reads or writes them, and
/// [`clock_steering`](Mt7610uBackend::clock_steering) says so itself ("It is left unwired") — so the
/// citation pointed at a register map rather than at anything this driver does, and pointed away
/// from the one piece of evidence this backend actually has.
///
/// `measured: None` — the tick RATE is MEASURED at 1.000 µs over five windows, but that is a SCALE
/// check against the host clock (~4 significant figures, so it bounds the rate only at the ~100 ppm
/// level) and quoting it as a stability figure would be exactly the over-reading that type exists to
/// stop.
pub(crate) fn mt7610_time_sources(tsf_domain: ClockDomainId) -> Vec<RadioTimeSource> {
    vec![RadioTimeSource {
        precision_ns: regs::measured::EP0_ROUND_TRIP_US * 1_000,
        tick_ns: 1_000,
        monotonic: true,
        reference: ndn_radio_hal::ClockReference::crystal(),
        ..RadioTimeSource::port_tsf(tsf_domain)
    }]
}

impl RadioTime for Mt7610uBackend {
    fn time_sources(&self) -> Vec<RadioTimeSource> {
        mt7610_time_sources(self.tsf_domain)
    }

    fn read_clock(&self, domain: ClockDomainId) -> Result<Option<u64>, FaceError> {
        if domain != self.tsf_domain {
            return Ok(None);
        }
        self.tsf().map(Some)
    }

    /// `None` — this radio steers nothing.
    ///
    /// The mt76x0 does expose a crystal trim: `MT_XO_CTRL5`'s `C2_VAL [14:8]` and
    /// `MT_XO_CTRL6`'s `C2_CTRL [14:8]` (`mt76x02_regs.h:67-71`) are the load-capacitance
    /// pair, the same shape as the Realtek `crystal_cap` this codebase disciplines against.
    /// It is left unwired because [`ClockSteering`](ndn_radio_hal::ClockSteering) demands a
    /// **measured** `range_ppm` and `resolution_ppm` — a discipline loop believes both, and
    /// a datasheet-width range is exactly the fabricated number that field warns about. The
    /// range is **UNMEASURED**; closing it means the two-node common-view sweep that
    /// characterised the 8733b's curve, run on this part.
    fn clock_steering(&self) -> Option<ndn_radio_hal::ClockSteering> {
        None
    }
}

// ── RadioProfile ────────────────────────────────────────────────────────────

impl RadioProfile for Mt7610uBackend {
    fn capability(&self) -> RadioCapability {
        declared_capability()
    }
}

/// **What this radio is**, as a free function — a static fact about the silicon, not about
/// any open handle, so it is checkable with no dongle plugged in. A capability that can only
/// be asserted on hardware is one nothing verifies.
///
/// Every field is deliberately modest, because the planner and the worst-receiver rate cap
/// both believe what a radio says about itself:
///
/// * **`rate: Wifi { max_mcs: 7, max_nss: 1, max_bw: 2 }`.** One chain, HT MCS 0-7, up to 80 MHz.
///   `max_bw: 2` is MEASURED (2026-08-31): a witness receiver read 100% of our frames as
///   "MCS 7 20 MHz", "MCS 7 40 MHz" and "MCS 7 BCC FEC 80 MHz" across three arms, so the 80 MHz
///   VHT PPDU is confirmed emitted. `max_mcs` stays at **7** — VHT-1SS reaches MCS8/9 and
///   [`mt76_rate_val`] would encode them, but only MCS7 was put on air, and an unverified
///   MCS is exactly what the worst-receiver cap must not inherit.
/// * **`channels`.** 2.4 GHz 1-14 and the standard 20 MHz 5 GHz centres, all of which have a
///   PLL program in [`freq_plan::FREQUENCY_PLAN`]. The plan actually covers every integer
///   channel from 36-64 and 100-173 plus the 802.11j band; listing only the standard centres
///   under-declares deliberately — `set_channel` accepts more than this, and nothing is
///   declared that it would reject.
/// * **`tx_power_dbm: None`.** See [`RadioKnobs::set_tx_power_dbm`]: the knob is real, the
///   dBm scale is unmeasured on this part, and an invented range is worse than none.
/// * **`retune_us: Some(135_000)` — MEASURED**, and the number is bad news worth having.
///   `examples/mt7610_bringup.rs` timed `set_channel(149, Bw20)` at **113 / 121 / 135 ms**
///   across three cold runs; the figure here is the slowest, because a scheduler that
///   over-promises retune speed builds a hop schedule the radio cannot keep. That is ~2
///   orders of magnitude slower than the Realtek parts' 16-26 ms, and it is inherent rather
///   than lazy: the tune is ~150 RF register writes plus a calibration ladder, each an EP0
///   round trip at 151 µs. ⇒ `RadioCapability::can_hop` will now correctly refuse any hop
///   schedule with a dwell under ~135 ms on this radio, instead of answering "I cannot say".
/// * **`max_tx_power: 47`**, not 63. 63 is the width of the `MT_TX_ALC_CFG_0` index field
///   (`GENMASK(5,0)`) — the *scale's* top, not a reachable power. The ALC LIMIT fields stay at
///   their init value (23.5 dB) because [`phy::set_alc_limits`] has **no callers**, so requests
///   above the limit are clamped by the hardware and indices 48..63 all produce the same output.
///   Declaring 63 meant the first ~8 dB of every back-off did nothing at all — cognition
///   "reduced" power and the air did not change. 47 is the honest top of the actuated range;
///   raising it requires wiring `set_alc_limits`, not editing this number.
/// * **`csi: None`.** No host-visible channel state beyond per-frame RSSI/MCS.
pub fn declared_capability() -> RadioCapability {
    RadioCapability {
        // ── PHY-mode / hop fields, added to `RadioCapability` for the sub-GHz family ──
        // `PhyMode` is a LoRa/FSK/BLE-family table with no Wi-Fi modulation in it, so the honest
        // answer for this part is "I cannot say" — NOT a set of one. Per the field docs, an empty
        // `PhyModeSet` is exactly that statement, and a planner must not read it as a single mode.
        // `hop` is autonomous frequency hopping: a Wi-Fi radio has no sequencer of its own, and
        // host-commanded retunes are already priced by `retune_us`.
        phy_modes: ndn_radio_hal::PhyModeSet::empty(),
        phy_current: None,
        hop: None,
        kind: RadioKind::WifiMonitor,
        he_cap: false,
        bands: vec![Band::Band2_4GHz, Band::Band5GHz],
        rate: RateCapability::Wifi {
            max_mcs: 7,
            max_nss: 1,
            // ★ MEASURED 20/40/80 on air by a witness receiver (2026-08-31) — see
            // `set_channel`. This was `0` while the 40/80 MCU calibration failed on silicon;
            // it now completes, and cognition reads `vht` as `max_bw() >= 2`, so under-declaring
            // here would hide the width lever the same way it was hidden on the MT7921AU.
            max_bw: 2,
        },
        channels: vec![
            1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, // 2.4 GHz
            36, 40, 44, 48, 52, 56, 60, 64, // U-NII-1/2A
            100, 104, 108, 112, 116, 120, 124, 128, 132, 136, 140, 144, // U-NII-2C
            149, 153, 157, 161, 165, // U-NII-3
        ],
        max_tx_power: 47,
        // MEASURED-adjacent, not SDR-metered: the MT7610U's ALC index is the EEPROM's own factory
        // half-dB calibration (`phy.rs` target/limit path), which is a real calibrated scale rather
        // than a guess — but it has never been put on a spectrum analyser, so treat 0.5 as CODE-READ.
        min_tx_power: Some(0),
        db_per_power_idx: Some(0.5),
        power_actuated: true,
        tx_power_dbm: None,
        retune_us: Some(135_000),
        rx_only: false,
        duty_cycle_max: 1.0,
        // ★ MEASURED on air, not assumed: 11418 B delivers at 95.8% and 11419 B at 0.0%
        // (see [`MAX_MPDU_PAYLOAD`]). This was 1500, which pinned every HAL consumer to the one
        // corner where the fixed per-PPDU cost dominates: at Bw80, pumped, posture `Owned`,
        // 1400 B yields ~82 Mbit/s and 11400 B yields ~250.
        max_payload: MAX_MPDU_PAYLOAD,
        half_duplex: true,
        csi: CsiSupport::None,
    }
}

// ── Tests ───────────────────────────────────────────────────────────────────

// ─────────────────────────────────────────────────────────────────────────────
// M5 · §1.4 — THE PLAN.  One sequence, executed by the shared runner.
// ─────────────────────────────────────────────────────────────────────────────
//
// Specification: `docs/bringup-contract.md` §1.4/§1.5/§4/§5-M5.
//
// ★ **Every rung below is transcribed VERBATIM, IN ORDER, from the M2 `bring_up`, then
// `setup_monitor_rx`, then the tune** — the three calls `open_named_radio`'s MT7610U arm always
// made back to back. Not one register write moved, was added, or was reordered. A plan that
// "improves" a ladder is an unmeasured change to a radio nobody at this keyboard can test.
//
// **Why the three collapse into one plan** (§5-M5): `MT_MAC_SYS_CTRL = ENABLE_TX | ENABLE_RX` is
// written by `setup_monitor_rx` and by nothing else, so a caller that ran `bring_up` and stopped
// got a radio that answers every register read and receives nothing. Eight `mt7612_*` examples on
// the sibling part are exactly that shape (`bring_up` then `setup_monitor_rx` by hand), and the
// factory's own MT7921AU arm had the tune and the monitor call in the WRONG ORDER for every caller
// until 2026-09-01. One plan removes the ordering question from the caller, and `ASSERTS_MT7610U`
// reads the gate back afterwards.
//
// Three things that are NOT verbatim, each named where it happens:
//
//   1. ☠ **warm/cold now branches on a ROUND TRIP, not on a status latch** — `mcu_responsive()`,
//      not `firmware_running()`. That was M5's explicit instruction and it is the one live
//      divergence this part carried: the sibling MT7612U was MEASURED wrong in BOTH directions
//      from the same latch, each mistake costing a physical replug. See `R_FIRMWARE_READY`.
//   2. `NDN_RADIO_FORCE_FW` left the ladder (LAW 1): read once at the wrapper boundary, passed as
//      an argument, and reported as the branch the rung took (`cold-forced`).
//   3. the bring-up-duration bulk-IN drain is bracketed by two rungs (`rx_drain` /
//      `rx_drain_stop`) instead of a local `DrainGuard`, so its span is EXACTLY the rungs it
//      covered before — see `Mt7610uBackend::bringup_drain`.
//
// ⚠ **No on-air behaviour change.** This part declares no `TxInstrument` (see
// `TX_UNPROVABLE_MT7610U`), so `run_plan` takes no transmit probe and the plan puts nothing on the
// air the old ladder did not.

/// Shorthand for the step tables below.
type Mt7610 = Mt7610uBackend;

// ── the rungs ────────────────────────────────────────────────────────────────

fn s_rx_drain(b: &Arc<Mt7610>, _c: &mut Ctx<'_>) -> Result<StepOutcome, FaceError> {
    b.bringup_drain.store(false, Ordering::Relaxed);
    let stop = b.bringup_drain.clone();
    let h = b.usb.handle();
    let ep = b.usb.ep_in_data();
    std::thread::spawn(move || {
        let mut buf = vec![0u8; 8192];
        while !stop.load(Ordering::Relaxed) {
            let _ = h.read_bulk(ep, &mut buf, Duration::from_millis(50));
        }
    });
    Ok(StepOutcome::Done)
}

fn s_firmware_ready(b: &Arc<Mt7610>, c: &mut Ctx<'_>) -> Result<StepOutcome, FaceError> {
    let force = b.force_cold_fw.load(Ordering::Relaxed);
    let live = !force && b.mcu_responsive();
    // The corroborating latch, logged and never decisive — see `firmware_running`.
    let latch = b.firmware_running();
    if live != latch {
        c.warn(format!(
            "MT_MCU_COM_REG0 latch says firmware_running={latch} while the MCU round trip says \
             responsive={live}. The round trip decides (the latch is a MAILBOX and has been wrong \
             in both directions on the sibling MT7612U). Recorded because a disagreement is \
             evidence about the latch, and the latch is what upstream's own probe tests."
        ));
    }
    c.state().warm = Some(live);
    // LAW 6. `StepOutcome` admits ONE variant per rung and the operator-facing answer here is the
    // branch label, so the fact is written straight into the state the runner would have hoisted
    // it into. Warm/cold changes what everything after it means and no register read reveals it
    // to a later caller.
    let fact = Fact::Warm(live);
    if !c.state().facts.contains(&fact) {
        c.state().facts.push(fact);
    }
    if live {
        tracing::info!(
            target: "named_radio",
            chip = "MT7610U",
            "MCU answers a CMD_RANDOM_READ round trip — warm re-open, skipping the power cycle \
             and the firmware download"
        );
        return Ok(StepOutcome::Branch("warm"));
    }
    b.chip_onoff(false, false)?;
    b.wait_for_mac()?;
    b.chip_onoff(true, true)?;
    b.wait_for_mac()?;
    // The whole download — DMA-cfg preamble, FCE setup, ILM/IVB/DLM, load-IVB and the
    // MT_MCU_COM_REG0 readiness poll — belongs to `mcu::load_firmware`
    // (mt76x0/usb_mcu.c:85-162). Nothing here second-guesses it.
    mcu::load_firmware(b.as_ref(), MT7610U_FIRMWARE)?;
    Ok(StepOutcome::Branch(if force {
        "cold-forced"
    } else {
        "cold"
    }))
}

fn s_init_usb_dma(b: &Arc<Mt7610>, _c: &mut Ctx<'_>) -> Result<StepOutcome, FaceError> {
    b.init_usb_dma()?;
    Ok(StepOutcome::Done)
}

fn s_init_hardware(b: &Arc<Mt7610>, _c: &mut Ctx<'_>) -> Result<StepOutcome, FaceError> {
    b.init_hardware()?;
    Ok(StepOutcome::Done)
}

fn s_usb_timing(b: &Arc<Mt7610>, _c: &mut Ctx<'_>) -> Result<StepOutcome, FaceError> {
    b.rmw(regs::MT_US_CYC_CFG, regs::MT_US_CYC_CNT, 0x1e)?;
    b.wr(
        regs::MT_TXOP_CTRL_CFG,
        regs::field_prep(regs::MT_TXOP_TRUN_EN, 0x3f)
            | regs::field_prep(regs::MT_TXOP_EXT_CCA_DLY, 0x58),
    )?;
    Ok(StepOutcome::Done)
}

fn s_mac_address(b: &Arc<Mt7610>, _c: &mut Ctx<'_>) -> Result<StepOutcome, FaceError> {
    b.set_mac_address(b.eeprom.mac_addr())?;
    Ok(StepOutcome::Done)
}

fn s_pin_contention(b: &Arc<Mt7610>, c: &mut Ctx<'_>) -> Result<StepOutcome, FaceError> {
    // ★ The applied posture goes into the report. `state.contention == None` means "inherited /
    // unknown", which on USB is a real and hazardous state (§3's own table); recording what was
    // actually programmed is the only way a throughput number can be read back later and trusted.
    let applied = knobs::set_contention(
        b.as_ref(),
        ndn_radio_hal::ContentionPosture::Shared,
        Family::Mt76x0,
        &b.edca_saved,
    )?;
    c.state().contention = Some(applied);
    Ok(StepOutcome::Done)
}

fn s_rx_drain_stop(b: &Arc<Mt7610>, _c: &mut Ctx<'_>) -> Result<StepOutcome, FaceError> {
    b.bringup_drain.store(true, Ordering::Relaxed);
    Ok(StepOutcome::Done)
}

fn s_monitor_rx(b: &Arc<Mt7610>, _c: &mut Ctx<'_>) -> Result<StepOutcome, FaceError> {
    b.setup_monitor_rx()?;
    Ok(StepOutcome::Done)
}

fn s_tune_channel(b: &Arc<Mt7610>, c: &mut Ctx<'_>) -> Result<StepOutcome, FaceError> {
    let (ch, bw) = (c.state_ref().channel, c.state_ref().bw);
    ndn_radio_hal::RadioKnobs::set_channel(b.as_ref(), ch, bw)?;
    Ok(StepOutcome::Done)
}

// ── the rungs, as reviewable constants ───────────────────────────────────────

const R_RX_DRAIN: Step<Mt7610> = Step {
    id: StepId("rx_drain"),
    stage: Stage::Attach,
    class: StepClass::Required,
    why: "★ MEASURED: mt76 USB expects the host to keep RX transfers posted. If nobody reads, the \
          device's USB DMA backs up and the MCU stops CONSUMING inband commands — they are \
          accepted into the FIFO and never processed, so the NEXT command's bulk-out times out \
          rather than the offending one. Seen here as `command 0x0c seq 0 bulk-out (200 B): \
          Operation timed out` partway through `init_hardware`, on a device whose registers were \
          all answering normally. The command RESPONSE pipe (0x85) is a different pipe from the \
          data pipe (0x84) on this part, so this drain cannot steal an MCU reply.",
    must_follow: &[],
    must_precede: &[StepId("firmware_ready"), StepId("init_hardware")],
    run: s_rx_drain,
};

const R_FIRMWARE_READY: Step<Mt7610> = Step {
    id: StepId("firmware_ready"),
    stage: Stage::Firmware,
    class: StepClass::Required,
    why: "★ The warm/cold decision AND the cold sequence, in ONE rung — branching BETWEEN steps is \
          not expressible (§1.4), so the decision lives inside a named step and reports which way \
          it went as `StepOutcome::Branch`. Cold is upstream's order: `chip_onoff(false,false)` \
          (usb.c:257 — \"Disable the HW, otherwise MCU fail to initialize on hot reboot\"), \
          `wait_for_mac`, `chip_onoff(true,true)`, `wait_for_mac`, then the firmware download. \
          ☠ The decision is a ROUND TRIP (`mcu_responsive`), NOT `MT_MCU_COM_REG0`: that register \
          is a MAILBOX, and deciding from it was MEASURED wrong in BOTH directions on the sibling \
          MT7612U — 'cold' on a chip the kernel had just loaded (the download then collides with \
          the FCE and wedges the part) and 'warm' on a chip that answered nothing. Each mistake \
          cost a physical replug. **LAW 5**: the cold path polls XTAL_RDY|PLL_LD in `chip_onoff` \
          and MT_MCU_COM_REG0 for firmware readiness in `load_firmware`, and a timed-out boot that \
          continued would leave a radio whose every later readback is fiction.",
    must_follow: &[StepId("rx_drain")],
    must_precede: &[StepId("init_hardware")],
    run: s_firmware_ready,
};

const R_INIT_USB_DMA: Step<Mt7610> = Step {
    id: StepId("init_usb_dma"),
    stage: Stage::MacInit,
    class: StepClass::Required,
    why: "`mt76x0_init_usb_dma` (mt76x0/usb.c:46-71 via usb.c:164): enable both bulk directions \
          and CLEAR `RX_BULK_AGG_EN` — upstream's own comment is \"disable AGGR_BULK_RX in order \
          to receive one frame in each rx urb and avoid copies\", and the oracle MEASURED the bit \
          clear on the live kernel interface. One RX unit per bulk-IN transfer is what \
          `parse_transfer` is written against, so this rung is what makes the RX framing \
          assumption true.",
    must_follow: &[StepId("firmware_ready")],
    must_precede: &[],
    run: s_init_usb_dma,
};

const R_INIT_HARDWARE: Step<Mt7610> = Step {
    id: StepId("init_hardware"),
    stage: Stage::MacInit,
    class: StepClass::Required,
    why: "`mt76x0_init_hardware` (mt76x0/init.c:171-212 via usb.c:165) — the MAC tables, the BBP \
          tables and the RF init, in upstream's order. It must come AFTER the firmware: both init \
          tables go through the MCU register-pair path (upstream's `RANDOM_WRITE` macro is \
          `mt76_wr_rp(dev, MT_MCU_MEMMAP_WLAN, tab, n)`, init.c:83-85), so a cold run that \
          programmed the MAC first would be writing through an MCU that is not there. **LAW 5**: \
          it polls WPDMA idle, MAC idle, TX/RX idle and BBP ready, each of which returns Err on \
          timeout — a bring-up that walked past any of them would leave every later readback \
          fiction.",
    must_follow: &[StepId("firmware_ready"), StepId("init_usb_dma")],
    must_precede: &[],
    run: s_init_hardware,
};

const R_USB_TIMING: Step<Mt7610> = Step {
    id: StepId("usb_timing"),
    stage: Stage::MacInit,
    class: StepClass::Required,
    why: "`MT_US_CYC_CFG` + `MT_TXOP_CTRL_CFG` (usb.c:171-174), one pair, in upstream's order. \
          ★ The TXOP value written here is bit-identical to the MEASURED kernel-monitor \
          `MT_TXOP_CTRL_CFG = 0x0000_583f` (TRUN_EN 0x3f, EXT_CCA_DLY 0x58, ED-CCA off) — a \
          genuine cross-check of the transcription against the live kernel driver, and the reason \
          `set_edcca_ignore(true)` is close to a no-op on this part.",
    must_follow: &[StepId("init_hardware")],
    must_precede: &[],
    run: s_usb_timing,
};

const R_MAC_ADDRESS: Step<Mt7610> = Step {
    id: StepId("mac_address"),
    stage: Stage::MacInit,
    class: StepClass::Required,
    why: "`mt76x02_mac_setaddr` (mt76x02_mac.c:740-743) — the MAC_ADDR_DW0/DW1 half only; the \
          BSSID/multi-BSS/beacon block after it configures beaconing, which this driver never \
          does, and BSSID filtering, which a promiscuous monitor ignores. ⚠ The address is a \
          receive-side filter datum and diagnostic only: the named-data doctrine forbids a host \
          identity in the source field, and `build_dot11` stamps its own.",
    must_follow: &[StepId("init_hardware")],
    must_precede: &[],
    run: s_mac_address,
};

const R_PIN_CONTENTION: Step<Mt7610> = Step {
    id: StepId("pin_contention"),
    stage: Stage::Posture,
    class: StepClass::BestEffort(Degradation::new(
        "the contention posture is whatever the PREVIOUS process left on the chip — USB never \
         power-cycles it between processes, and there is no 'as found' to go back to",
        "RX capture and functional link work; NOT any absolute throughput, airtime or A/B figure, \
         which on this part MEASURED a 2.5x swing decided purely by run order",
    )),
    why: "★★ MEASURED 2026-08-31, Bw80/VHT MCS7/1400 B/pump=8, five consecutive processes: a run \
          that set no posture returned 2724 OR 6706 f/s — a 2.5x swing decided purely by what ran \
          before it (A 6321 no-posture, B 6382 owned, C 6706 no-posture INHERITING owned, D 2965 \
          shared, E 2724 no-posture INHERITING shared). This driver never wrote these registers at \
          all, so contention was simply latched across processes, and any unpinned A/B was \
          comparing history. ⚠ `Shared` is NOT this part's boot value — that constant comes from \
          the MT7612U's captured init, and the MEASURED as-found kernel state here is different \
          (AIFSN 0x1111 / CWMIN 0x2222 vs our 0x2222 / 0x4444). What this rung claims, and all it \
          claims, is that the posture is DETERMINISTIC instead of inherited. Best effort: a \
          contention write failing must not turn a working radio into no radio.",
    must_follow: &[StepId("init_hardware")],
    must_precede: &[],
    run: s_pin_contention,
};

const R_RX_DRAIN_STOP: Step<Mt7610> = Step {
    id: StepId("rx_drain_stop"),
    stage: Stage::RxEnable,
    class: StepClass::Required,
    why: "Ends the bring-up-duration drain, at exactly the point the old `DrainGuard` dropped — \
          the last statement of `bring_up`, BEFORE monitor RX is enabled. It has to stop here: \
          past `monitor_rx` the chip delivers real frames, and a drain thread reading the data \
          pipe would then compete with the RX pump for them and silently eat a fraction of every \
          capture. Making the drain a `StepOutcome::Guard` instead would have handed it to the \
          returned handle and done exactly that, for the life of the process.",
    must_follow: &[StepId("rx_drain")],
    must_precede: &[StepId("monitor_rx")],
    run: s_rx_drain_stop,
};

const R_MONITOR_RX: Step<Mt7610> = Step {
    id: StepId("monitor_rx"),
    stage: Stage::RxEnable,
    class: StepClass::Required,
    why: "★ THE RUNG §5-M5 EXISTS FOR. `mt76x02u_mac_start` (mt76x02_usb_core.c:25-43): re-assert \
          the USB DMA bulk enables (a firmware download leaves them elsewhere), open \
          `MT_RX_FILTR_CFG` to 0, and write `MT_MAC_SYS_CTRL = ENABLE_TX | ENABLE_RX` (the \
          MEASURED 0x0c). Nothing else in this ladder writes that register, so a caller that ran \
          the old `bring_up` and stopped had a radio that answered every register read and \
          received nothing. It also arms the TSF and the channel-time counters, which are free and \
          whose absence makes `read_channel_activity` return a confident 0 permille — reading as \
          'quiet channel' when the counters were simply never armed. `ASSERTS_MT7610U` reads the \
          MAC_SYS_CTRL gate back.",
    must_follow: &[StepId("init_hardware"), StepId("rx_drain_stop")],
    must_precede: &[],
    run: s_monitor_rx,
};

const R_TUNE_CHANNEL: Step<Mt7610> = Step {
    id: StepId("tune_channel"),
    stage: Stage::Tune,
    class: StepClass::Required,
    why: "★ Also part of §5-M5's collapse: the tune was the factory's third separate call, and the \
          old `bring_up` reported `channel: 0` because it genuinely left the radio untuned — the \
          factory then patched the report's channel field afterwards, which is a report describing \
          something the plan did not do. Fatal on purpose: an untuned monitor receives nothing, \
          and returning a working-looking handle that hears silence is the failure this repo keeps \
          paying for. It runs AFTER `monitor_rx`, which is the order the factory has always used \
          on this part; unlike the MT7612U's captured channel replay, `phy::set_channel_ext` here \
          writes neither `MT_RX_FILTR_CFG` nor `MT_MAC_SYS_CTRL`, so it cannot undo the rung above \
          it. (On the MT7612U it does, and that cost a MEASURED 9975-CRC-error silent receiver.)",
    must_follow: &[StepId("monitor_rx")],
    must_precede: &[],
    run: s_tune_channel,
};

const MT7610U_STEPS: &[Step<Mt7610>] = &[
    R_RX_DRAIN,
    R_FIRMWARE_READY,
    R_INIT_USB_DMA,
    R_INIT_HARDWARE,
    R_USB_TIMING,
    R_MAC_ADDRESS,
    R_PIN_CONTENTION,
    R_RX_DRAIN_STOP,
    R_MONITOR_RX,
    R_TUNE_CHANNEL,
];

const MT7610U_PLAN: Plan<Mt7610> = Plan {
    id: PlanId {
        part: "mt76x0",
        name: "monitor",
        // v1 was the M2 hand-filled `bring_up` report, which described only the first seven rungs
        // and reported an untuned radio. v2 is the executed plan and covers the monitor + tune the
        // factory always ran beside it, so the two digests must not be comparable.
        ver: 2,
    },
    role: Role::TransmitAndReceive,
    steps: MT7610U_STEPS,
    excluded: &[
        (
            Stage::PowerOn,
            "no PowerOn-stage rung. `chip_onoff` IS the power sequence on this part and it lives \
             inside `firmware_ready`, because the warm branch must skip it and the cold branch \
             must run it — and branching between rungs is not expressible (§1.4). The stage label \
             follows the rung, not the other way round (Appendix A.2).",
        ),
        (
            Stage::PhyInit,
            "no separate PhyInit rung. `phy::init_rf` is the last statement of \
             `mt76x0_init_hardware` (init.c:210) and is transcribed inside `init_hardware` where \
             upstream put it. Splitting it out would be a sequence change dressed as bookkeeping.",
        ),
        (
            Stage::Calibrate,
            "no Calibrate rung at bring-up, and that is upstream's shape, not an omission: on this \
             part the PHY calibration is per-channel and is issued by `phy::set_channel` itself \
             (`MCU_CAL_*` through CMD_CALIBRATION_OP). `Mt7610uBackend::calibrate` exists for a \
             re-cal after a long dwell or a temperature swing and REFUSES to run before a tune.",
        ),
        (
            Stage::Power,
            "no Power rung. TX power on this part is per-channel AND per-rate, computed from the \
             EEPROM by the tune, so there is no index to write at bring-up; `RadioState::power` \
             stays `no_actuator(NoActuator)`, which says 'the bring-up did not touch it' rather \
             than the false 'this part has no power knob' (it has one — `set_tx_power`).",
        ),
        (
            Stage::Verify,
            "no Verify rung. The one readback this ladder owes — `MT_MAC_SYS_CTRL` after \
             `monitor_rx` — is a part-wide `Assert` (`ASSERTS_MT7610U`) that the runner takes \
             after the last rung; and §4's transmit question is answered `Unprovable` with the \
             measurement quoted, because this part's only counter is read-and-clear and shared \
             with any bound kernel driver.",
        ),
    ],
};

// ★ A malformed plan is a compile error, not a runtime one.
const _: () = MT7610U_PLAN.check_or_panic();

/// The MT7610U plan — `bring_up` + `setup_monitor_rx` + the tune, as one reviewable sequence.
pub static PLAN_MT7610U: Plan<Mt7610> = MT7610U_PLAN;

/// §1.5 — read back every gate you write.
///
/// ★ `mac_tx_rx_enabled` is the assert §5-M5 named this part for: `MT_MAC_SYS_CTRL` is written by
/// `monitor_rx` and by nothing else in the ladder, and a chip without it answers every register
/// read while receiving and transmitting nothing. Now that the three calls are one plan the rung
/// cannot be *forgotten*; the assert covers the other half — that it was written and stuck.
///
/// ⚠ `Warn` on introduction, per §5/M-hazards. Promotion to `Fatal` is per part and needs a
/// measurement.
const ASSERTS_MT7610U: &[Assert<Mt7610>] = &[
    Assert {
        id: StepId("mac_tx_rx_enabled"),
        reg: regs::MT_MAC_SYS_CTRL,
        read: |b: &Mt7610| b.rr(regs::MT_MAC_SYS_CTRL),
        want: regs::MT_MAC_SYS_CTRL_ENABLE_TX | regs::MT_MAC_SYS_CTRL_ENABLE_RX,
        mask: regs::MT_MAC_SYS_CTRL_ENABLE_TX | regs::MT_MAC_SYS_CTRL_ENABLE_RX,
        why: "MT_MAC_SYS_CTRL ENABLE_TX|ENABLE_RX (the MEASURED 0x0c). Written once, by \
              `monitor_rx`. Without it the MAC engines are off: registers all read fine, the RF \
              hears nothing, and the symptom is an interface that comes up perfectly and receives \
              zero frames. ⚠ Also the bit `reset_csr_bbp` clears and `init_mac_registers` \
              re-clears, so an out-of-order sequence would leave it 0 with nothing complaining.",
        severity: Severity::Warn,
    },
    Assert {
        id: StepId("wlan_enabled"),
        reg: regs::MT_WLAN_FUN_CTRL,
        read: |b: &Mt7610| b.rr(regs::MT_WLAN_FUN_CTRL),
        want: regs::MT_WLAN_FUN_CTRL_WLAN_EN,
        mask: regs::MT_WLAN_FUN_CTRL_WLAN_EN,
        why: "MT_WLAN_FUN_CTRL bit 0 (WLAN_EN). MEASURED on this part: a run that reached monitor \
              with `WLAN_FUN_CTRL = 0xff000012` (bit 0 clear) logged 0 CRC errors, 0 PHY errors \
              and 0 busy microseconds — the receiver was not merely quiet, it was OFF. \
              `setup_monitor_rx` re-asserts the bit when it finds it clear; this reads back \
              whether that repair held, which the repair itself does not check.",
        severity: Severity::Warn,
    },
];

/// §4 — what this part can prove about its own transmitter: **nothing at bring-up, and it says
/// why.**
///
/// The instrument named in §4's table (`MT_TX_STAT_FIFO` 0x1718) is transcribed in
/// [`crate::mt76::regs`] with **zero readers**, and two properties make it wrong for a bring-up
/// probe rather than merely unimplemented — both recorded here so the next person does not wire it
/// up expecting an answer.
const TX_UNPROVABLE_MT7610U: &str = "no transmit counter is read at bring-up. The one candidate, MT_TX_STAT_FIFO 0x1718, is \
     READ-AND-CLEAR (so a bound kernel mt76x0u steals roughly half of every reading, and two \
     readers each see a fraction with neither looking wrong) and is transcribed with zero readers \
     — it has never been differenced across a known number of injects on this silicon, which is \
     what the 8733b's calibrated +50-across-50-injects instrument has and this one does not. \
     Question (A) is answerable here in principle and is NOT answered; prove (B) with a witness.";

impl BringUp for Mt7610uBackend {
    fn plan(role: Role) -> Option<&'static Plan<Self>> {
        match role {
            Role::TransmitAndReceive => Some(&PLAN_MT7610U),
            // ★ Named refusals, not silent downgrades. One plan is all this part has ever run.
            // `monitor_rx` writes ENABLE_TX and ENABLE_RX in the SAME register write, so an
            // RX-only variant would mean inventing a MAC_SYS_CTRL value this silicon has never
            // been brought up with — the unmeasured ladder this contract exists to remove. A
            // caller that only wants to listen asks for TransmitAndReceive and does not inject.
            Role::ReceiveOnly | Role::TransmitOnly => None,
        }
    }

    fn asserts() -> &'static [Assert<Self>] {
        ASSERTS_MT7610U
    }

    /// Empty — see [`TX_UNPROVABLE_MT7610U`].
    fn tx_instruments() -> &'static [ndn_radio_hal::TxInstrument] {
        &[]
    }

    fn tx_unprovable_reason() -> Option<&'static str> {
        Some(TX_UNPROVABLE_MT7610U)
    }
}

/// The cost of the pre-M8 wrapper signature: `FaceError` cannot carry a partial report.
///
/// So it is **emitted before it is dropped** — a failed bring-up that says how far it got is the
/// entire point of §3, and losing it silently here would put the old defect back one level up.
fn drop_partial_report(f: BringUpFailure) -> FaceError {
    f.report.emit();
    eprintln!(
        "mt7610u bring-up FAILED at `{}` — the partial report:\n{}",
        f.failed_at,
        f.report.render()
    );
    f.source
}

#[cfg(test)]
mod tests {
    /// ★ The bandwidth field must actually appear in the rate word, and must be clamped by PHY.
    /// Leaving it at 0 made the width knob a no-op: the channel widened and the PPDU did not.
    #[test]
    fn rate_word_carries_the_bandwidth_and_clamps_by_phy() {
        // VHT may use all three widths.
        for (code, want) in [(0u8, 0u16), (1, 1 << 7), (2, 2 << 7)] {
            let v = mt76_rate_val(&McsDescriptor::vht(7), code);
            assert_eq!(v & (0x3 << 7), want, "VHT bw code {code}");
        }
        // HT has no 80 MHz — asking for it must clamp to 40, not build an impossible PPDU.
        assert_eq!(mt76_rate_val(&McsDescriptor::ht(7), 2) & (0x3 << 7), 1 << 7);
        // And the field must not disturb the rest of the word.
        let base = mt76_rate_val(&McsDescriptor::vht(7), 0);
        let wide = mt76_rate_val(&McsDescriptor::vht(7), 2);
        assert_eq!(base & !(0x3 << 7), wide & !(0x3 << 7));
    }

    use super::*;

    /// A `RadioTime` over a fixed source list, so a DECLARATION can go through
    /// `FaceTimeProfile::derive` without a USB device.
    struct Declared(Vec<RadioTimeSource>);
    impl RadioTime for Declared {
        fn time_sources(&self) -> Vec<RadioTimeSource> {
            self.0.clone()
        }
    }

    /// ★ **The crystal declaration must rest on something this driver does.** The MT7610U's does:
    /// its bring-up polls `MT_CMB_CTRL` for `XTAL_RDY | PLL_LD` and errors out if the crystal never
    /// comes ready. The citation used to name `MT_XO_CTRL5/6` and `MT_EE_XTAL_TRIM_1/2` instead —
    /// constants nothing in this tree reads — which is a register map, not evidence.
    ///
    /// Nil consequence today, and that is pinned here too: a `PortTsf` fails common view on the
    /// LATCH axis whatever its reference is. The reference matters the moment stage 7 of
    /// `examples/mt7610_bringup.rs` finds a per-frame stamp in the undecoded `bbp_rxinfo` dwords.
    #[test]
    fn the_declared_reference_is_the_one_the_bring_up_path_witnesses() {
        use ndn_radio_hal::{ClockReferenceKind, FaceTimeProfile, TxDiscipline};

        let dom = ClockDomainId(0x7610);
        let v = mt7610_time_sources(dom);
        assert_eq!(v.len(), 1);
        assert_eq!(v[0].kind, ndn_time::RadioClockKind::PortTsf);
        assert_eq!(v[0].reference.kind, ClockReferenceKind::Crystal);
        assert_eq!(
            v[0].reference.measured, None,
            "a 1.000 us scale check is not a stability measurement"
        );
        assert_eq!(v[0].precision_ns, regs::measured::EP0_ROUND_TRIP_US * 1_000);

        let p = FaceTimeProfile::derive(&Declared(v), TxDiscipline::BestEffort);
        assert!(!p.hw_rx_stamp, "the RXWI carries no timestamp field");
        assert!(
            !p.can_common_view,
            "a crystal cannot rescue a radio with no per-frame stamp"
        );
    }

    /// The capability must describe *this* part and no other. Two failure modes are worth
    /// catching mechanically: a degenerate declaration (empty channels, zero rate ceiling —
    /// a radio nothing would ever select), and a declaration copy-pasted from the sibling
    /// mt7612 backend, which is 2×2/MCS9/VHT80 and would aim two-stream traffic at a
    /// one-chain radio. That is not hypothetical: the workspace's capability presets carry a
    /// standing warning about backends silently inheriting another part's numbers.
    #[test]
    fn declared_capability_is_non_degenerate_and_is_not_the_mt7612s() {
        let c = declared_capability();

        assert!(
            !c.channels.is_empty(),
            "a radio with no channels is unusable"
        );
        assert!(!c.bands.is_empty());
        assert!(!c.rx_only, "this part transmits");
        assert!(c.max_payload > 0);
        // ★ The declaration must BE the measured cliff, not a guess. 11418 B delivered at 95.8%
        // on air and 11419 B at 0.0%; `MPDU = payload + 36` puts that boundary exactly on the
        // 802.11 maximum MPDU of 11454 B. If someone "rounds" this declaration, the guard in
        // `inject` and the capability drift apart and oversized frames go back to vanishing
        // silently.
        assert_eq!(
            c.max_payload, MAX_MPDU_PAYLOAD,
            "declared max_payload must equal the guard in inject"
        );
        assert_eq!(
            MAX_MPDU_PAYLOAD + 24 + 6 + 2 + 4,
            11_454,
            "MAX_MPDU_PAYLOAD + (802.11 24 + LLC/SNAP 6 + ethertype 2 + FCS 4) is the 802.11 \
             maximum MPDU — this is the arithmetic the on-air cliff landed on to the byte"
        );
        assert!(c.rate_rank() > 0.0, "a zero rate rank is never selected");

        // 1x1, HT rate ceiling, 80 MHz — the whole point of the declaration.
        assert_eq!(c.max_nss(), 1, "MT7610U has ONE chain");
        assert_eq!(c.max_mcs(), 7, "single-stream HT stops at MCS7");
        assert_eq!(
            c.max_bw(),
            2,
            "MEASURED 2026-08-31: a witness receiver (RTL8812AU, kernel monitor, ch36) read 100% \
             of our frames as 20/40/80 MHz across three arms — the on-air width confirmation the \
             old 20 MHz declaration was waiting for"
        );
        assert!(!c.he_cap(), "802.11ac silicon, not ax");

        // Distinct from the 2x2 sibling on exactly the axes that would misroute traffic.
        let mt7612 = crate::Mt7612uBackend::declared_capability();
        assert_ne!(
            c, mt7612,
            "the two mt76 backends must not declare the same radio"
        );
        assert!(c.max_nss() < mt7612.max_nss());
        assert!(c.max_mcs() < mt7612.max_mcs());
        // ★ Width no longer separates them: the MT7612U reaches VHT80 through a captured RF
        // program (MEASURED 133 Mbit/s there) and this part now reaches it programmatically
        // (MEASURED on air 2026-08-31). CHAINS are what still separate them, and chains are what
        // would misroute traffic — so assert the axis that is real rather than one that lapsed.
        assert_eq!(c.max_bw(), mt7612.max_bw(), "both mt76 parts reach 80 MHz");

        // Nothing declared that set_channel would reject: every channel needs a PLL program,
        // and the declared width must be the one the knob accepts.
        for ch in &c.channels {
            assert!(
                freq_plan::FREQUENCY_PLAN.iter().any(|f| f.channel == *ch),
                "channel {ch} is declared but has no entry in the mt76x0 frequency plan"
            );
        }

        // MEASURED (113/121/135 ms over three cold runs); the declaration takes the slowest.
        // An unmeasured figure must stay absent rather than plausible, but this one is measured.
        assert_eq!(
            c.retune_us,
            Some(135_000),
            "retune_us must stay the MEASURED worst case, not an optimistic one"
        );
        assert!(c.tx_power_dbm.is_none(), "the dBm scale is unmeasured here");
        // With the retune MEASURED, `can_hop` gives a real answer — and the answer is no for
        // any dwell shorter than the tune itself. A 20 ms dwell on a radio that needs 135 ms to
        // change channel is not a schedule, it is a stall.
        assert_eq!(
            c.can_hop(20_000),
            Some(false),
            "a 20 ms dwell cannot accommodate a 135 ms retune"
        );
        assert_eq!(
            c.can_hop(2_000_000),
            Some(true),
            "a 2 s dwell comfortably accommodates it"
        );
    }

    /// The rate word must round-trip, and the 1×1 clamp must actually clamp — an HT index
    /// above 7 is a *two-stream* rate under this encoding (`nss = 1 + (idx >> 3)`), so
    /// letting one through would ask a one-chain radio for a PPDU it cannot build.
    #[test]
    fn rate_words_round_trip_and_clamp_to_one_stream() {
        for idx in 0u8..=7 {
            let d = McsDescriptor::ht(idx);
            let r = decode_rate_word(mt76_rate_val(&d, 0));
            assert_eq!(r.phy as u16, MT_PHY_TYPE_HT);
            assert_eq!(r.mcs, Some(idx));
            assert_eq!(r.nss, 1);
            assert_eq!(r.bw, 0);
            assert!(!r.short_gi);
        }

        // HT MCS 8-15 (2 streams) are clamped down to MCS7, not truncated into a different
        // rate: mcs 12 & 0x3f would otherwise encode a 2-stream MCS4.
        for idx in 8u8..=15 {
            let r = decode_rate_word(mt76_rate_val(&McsDescriptor::ht(idx), 0));
            assert_eq!(r.mcs, Some(7), "HT {idx} must clamp to the 1SS ceiling");
        }

        // VHT keeps its NSS field at one stream even when asked for two.
        let r = decode_rate_word(mt76_rate_val(&McsDescriptor::vht_2ss(7), 0));
        assert_eq!(r.phy as u16, MT_PHY_TYPE_VHT);
        assert_eq!(r.mcs, Some(7));
        assert_eq!(r.nss, 1, "one chain cannot carry two spatial streams");

        // STBC and LDPC are dropped on this part (see mt76_rate_val); short GI is not.
        let r = decode_rate_word(mt76_rate_val(
            &McsDescriptor::ht(5).with_stbc().with_ldpc(),
            0,
        ));
        assert!(!r.stbc && !r.ldpc);
        let sgi = McsDescriptor {
            short_gi: true,
            ..McsDescriptor::ht(3)
        };
        assert!(decode_rate_word(mt76_rate_val(&sgi, 0)).short_gi);
    }

    /// Legacy rate words, against `mt76x02_rates` (`mt76x02_util.c:10-30`). These are the
    /// rates the worst-receiver path depends on, so a transposed PHY nibble here would send
    /// an "everyone must hear this" frame as CCK on 5 GHz, where CCK does not exist.
    #[test]
    fn legacy_rate_words_match_upstream_hw_values() {
        assert_eq!(LegacyRate::Cck1.rate_val(), 0x0000);
        assert_eq!(LegacyRate::Cck2.rate_val(), 0x0001);
        assert_eq!(LegacyRate::Cck5_5.rate_val(), 0x0002);
        assert_eq!(LegacyRate::Cck11.rate_val(), 0x0003);
        assert_eq!(LegacyRate::Ofdm6.rate_val(), 0x2000);
        assert_eq!(LegacyRate::Ofdm54.rate_val(), 0x2007);
        // And they decode back as legacy — `mcs` must stay None, or a 6 Mbps frame shows up
        // in a rate histogram as "MCS0".
        for r in [LegacyRate::Cck11, LegacyRate::Ofdm6, LegacyRate::Ofdm24] {
            assert_eq!(decode_rate_word(r.rate_val()).mcs, None);
        }
    }

    /// `ieee80211_hdrlen`'s shape, on the headers this driver actually builds and parses.
    #[test]
    fn dot11_header_lengths() {
        assert_eq!(dot11_hdr_len(0x08, 0x00), 24, "plain data");
        assert_eq!(dot11_hdr_len(0x88, 0x00), 26, "QoS data");
        // The wide profile: 4-address QoS-Data + HT Control = 36, and 36 % 4 == 0, which is
        // why no L2 pad is inserted for it.
        assert_eq!(dot11_hdr_len(0x88, 0x83), 36, "wide profile");
        assert_eq!(dot11_hdr_len(0x80, 0x00), 24, "beacon");
        assert_eq!(dot11_hdr_len(0xb4, 0x00), 16, "RTS");
        assert_eq!(dot11_hdr_len(0xc4, 0x00), 10, "CTS");
    }

    /// Build one synthetic RX unit the way the hardware would — including the 4-byte
    /// alignment of `dma_len`, which `mt76u_get_rx_entry_len` (`usb.c:478`) requires and
    /// which the decoder rejects a unit for missing.
    fn rx_unit(dot11: &[u8], rate: u16, rssi: u8, crcerr: bool, l2pad: bool) -> Vec<u8> {
        let pad = usize::from(l2pad) * 2;
        let dma_len = (MT_RX_RXWI_LEN + dot11.len() + pad + MT_FCE_INFO_LEN).next_multiple_of(4);
        let body = MT_DMA_HDR_LEN + dma_len;
        let mut b = vec![0u8; body];
        b[0..2].copy_from_slice(&(dma_len as u16).to_le_bytes());
        let mut rxinfo = 0u32;
        if crcerr {
            rxinfo |= MT_RXINFO_CRCERR;
        }
        if l2pad {
            rxinfo |= MT_RXINFO_L2PAD;
        }
        b[RXWI_RXINFO..RXWI_RXINFO + 4].copy_from_slice(&rxinfo.to_le_bytes());
        b[RXWI_CTL..RXWI_CTL + 4].copy_from_slice(&((dot11.len() as u32) << 16).to_le_bytes());
        b[RXWI_RATE..RXWI_RATE + 2].copy_from_slice(&rate.to_le_bytes());
        b[RXWI_RSSI0] = rssi;
        if pad == 0 {
            b[RXD_LEN..RXD_LEN + dot11.len()].copy_from_slice(dot11);
        } else {
            let h = dot11_hdr_len(dot11[0], dot11[1]);
            b[RXD_LEN..RXD_LEN + h].copy_from_slice(&dot11[..h]);
            b[RXD_LEN + h + 2..RXD_LEN + dot11.len() + 2].copy_from_slice(&dot11[h..]);
        }
        b
    }

    /// The RX framing, checked against a synthetic transfer — no dongle needed.
    #[test]
    fn rx_descriptor_decode() {
        let mut dot11 = vec![0x08, 0x00, 0, 0];
        dot11.extend_from_slice(&[0xff; 6]); // addr1
        dot11.extend_from_slice(&[0x02; 6]); // addr2
        dot11.extend_from_slice(&[0xff; 6]); // addr3
        dot11.extend_from_slice(&[0, 0]); // seqctl
        dot11.extend_from_slice(&[0xaa, 0xaa, 0x03, 0, 0, 0, 0x86, 0x24]); // LLC/SNAP
        dot11.extend_from_slice(b"payload");

        let t = rx_unit(&dot11, LegacyRate::Ofdm6.rate_val(), 0x33, false, false);
        let u = decode_rx_unit(&t).expect("descriptor must decode");
        assert_eq!(u.unit_len, t.len());
        assert_eq!(u.mpdu_len, dot11.len());
        assert_eq!(u.hdr_len, 24);
        assert_eq!(u.pad, 0);
        assert_eq!(u.rssi_raw, 0x33);
        assert!(!u.crc_err);
        assert_eq!(decode_rate_word(u.rate).mcs, None);
        assert_eq!(rx_mpdu(&t, &u).as_deref(), Some(&dot11[..]));

        // The CRC verdict is carried, not hidden: MT_RX_FILTR_CFG = 0 means we see these,
        // and the parse path counts and then drops them.
        let t = rx_unit(&dot11, 0x4000, 0x20, true, false);
        assert!(decode_rx_unit(&t).expect("still decodes").crc_err);

        // A QoS header is 26 bytes, so the hardware pads; the pad must come back out and
        // leave the original MPDU byte-identical.
        let mut qos = dot11.clone();
        qos[0] = 0x88;
        qos.splice(24..24, [0u8, 0u8]); // QoS Control
        let t = rx_unit(&qos, 0x4000, 0x20, false, true);
        let u = decode_rx_unit(&t).expect("padded descriptor must decode");
        assert_eq!(u.pad, 2);
        assert_eq!(u.hdr_len, 26);
        assert_eq!(rx_mpdu(&t, &u).as_deref(), Some(&qos[..]));

        // Garbage must be refused rather than sliced.
        assert!(decode_rx_unit(&[0u8; 8]).is_none(), "too short");
        let mut bad = t.clone();
        bad[0] = 3; // dma_len not 4-aligned (usb.c:478)
        bad[1] = 0;
        assert!(decode_rx_unit(&bad).is_none());
    }

    /// The USB TX info word, against `mt76x02_usb_core.c:46-62` and
    /// `mt76x02u_tx_prepare_skb`'s flags (`:99-103`).
    #[test]
    fn tx_info_word_matches_upstream_layout() {
        // 24-byte header + 8 LLC/SNAP + 4 payload = 36 bytes; + 20 TXWI = 56, already
        // 4-aligned, so no rounding and no L2 pad.
        let info = tx_info_word(TXWI_LEN + 36);
        assert_eq!(info & 0xffff, 56, "TXINFO.len = round_up(TXWI + frame, 4)");
        assert_eq!(info & (1 << 19), 1 << 19, "MT_TXD_INFO_80211");
        assert_eq!(info & (1 << 24), 1 << 24, "MT_TXD_INFO_WIV");
        assert_eq!((info >> 25) & 0x3, 2, "MT_QSEL_EDCA");
        assert_eq!(info >> 27, 0, "DPORT = WLAN_PORT");
        // The mt7612 backend's captured kernel probe-request bulk carries the identical flag
        // nibbles (`0x050800f0`), so the two ports agree on the shared mt76x02 framing —
        // a cross-check of this transcription against a real captured frame.
        assert_eq!(info & 0xffff_0000, 0x0508_0000);

        // A frame length that is NOT 4-aligned must round the length field up, because the
        // device reads `TXINFO.len` bytes and the host pads to match (`:60`).
        assert_eq!(tx_info_word(TXWI_LEN + 37) & 0xffff, 60);
    }
}
