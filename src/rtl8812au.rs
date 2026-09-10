//! Userspace **RTL8812AU** backend — our own libusb driver for the classic
//! 11ac monitor-injection chip (the aircrack-ng lineage).
//!
//! This is a fresh port, not a reuse of [`LibUsbRtl88xxBackend`](crate::LibUsbRtl88xxBackend):
//! the 8812**AU** is the *original* 2014-era 11ac silicon (the vendor
//! `rtl8812au` / kernel `rtw88_88xxa` family), a different HAL generation from
//! the 8812**EU**/8822E (halmac-8822) that backend drives. They share only the
//! Realtek USB register-I/O protocol, reproduced here. The morrownr/aircrack
//! `rtl8812au` C source and the kernel `rtw88_88xxa` driver are **reference
//! only** — on this OPi's Linux 7.0 kernel neither compiles (timer-API
//! removal), which is exactly why a userspace driver is the path.
//!
//! ## Why device-targeted open matters
//!
//! The test rig also carries an RTL8812**EU** (`0bda:a81a`, `wlu1`) driven by a
//! kernel module. [`open`](Rtl8812auBackend::open) therefore matches **only**
//! the 8812AU product ids ([`RTL8812AU_PIDS`]) — never `a81a` — and claims that
//! one device, so bringing up the AU never detaches the EU sniffer.
//!
//! ## Status: validated on air — one of the two load-bearing Wi-Fi drivers
//!
//! Ported from the C reference and **confirmed on hardware** (the OPi 5 Pro rig,
//! several 8812AU and 8812AU-VS dongles): device-targeted USB open + the Realtek
//! register-I/O protocol + chip-version identification
//! ([`chip_info`](Rtl8812auBackend::chip_info)), the power-on sequence
//! ([`power_on`](Rtl8812auBackend::power_on)), firmware download
//! ([`download_firmware`](Rtl8812auBackend::download_firmware)), MAC/BB/RF init
//! ([`mac_config`](Rtl8812auBackend::mac_config) /
//! [`bb_config`](Rtl8812auBackend::bb_config) /
//! [`rf_config`](Rtl8812auBackend::rf_config)), monitor bring-up
//! ([`bring_up_monitor`](Rtl8812auBackend::bring_up_monitor)), and the [`FrameIo`]
//! inject/capture path. Both directions are proven on air: injected frames radiate
//! and are decoded by an independent receiver, and capture sustains **~500–640
//! frames/s** in monitor mode (it was ~200 f/s until the RX-DMA aggregation default
//! was corrected from `0x0101` to `0x2010` and `RXDMA_AGG_EN` was enabled). Several
//! identical dongles on one host are selected with
//! [`open_nth`](Rtl8812auBackend::open_nth) / `NDN_USB_INDEX`.
//!
//! Control-plane knobs wired through [`RadioKnobs`](ndn_radio_hal::RadioKnobs):
//! channel (**20 MHz only** — wider widths need their per-channel RF/BB program
//! captured), TXAGC index ([`set_tx_power`](Rtl8812auBackend::set_tx_power),
//! validated monotone on air), carrier-sense defeat
//! ([`set_cca_ignore`](Rtl8812auBackend::set_cca_ignore) — EDCCA *and* the OFDM
//! packet CCA, the latter being what still deferred TX on a busy channel), and
//! frame-free occupancy sensing
//! ([`read_phy_sense`](Rtl8812auBackend::read_phy_sense), `REG_RXERR_RPT`).
//!
//! Limits, each measured rather than assumed: 20 MHz only; the RX rate is **not** a
//! silicon ceiling (the in-kernel `rtw88_8812au` on the same chip at the same
//! geometry runs ~24% ahead — inherent userspace/usbfs overhead, not a driver
//! defect); and for the highest-fidelity receiver in a delivery measurement prefer
//! the 8822E ([`LibUsbRtl88xxBackend`](crate::LibUsbRtl88xxBackend), ~924 f/s).

use std::io;
use std::sync::Arc;
use std::time::Duration;

use async_trait::async_trait;
use bytes::Bytes;
use rusb::{Context, Device, DeviceHandle, Direction, TransferType};

use crate::realtek_rx;
use ndn_frame_io::{CapturedFrame, ClockDomainId, FrameFormat, FrameIo, InjectFrame, frame};
use ndn_radio_hal::bringup::{
    AppliedPower, Assert, BringUp, BringUpFailure, BringUpReport, Ctx, Degradation, Deviation,
    Fact, Guards, NO_CALIBRATION_MSG, Plan, PlanId, PlanRun, PowerReference, PowerRequest,
    PowerWrite, ProofRequirement, PumpPolicy, RadioState, Role, Severity, Stage, Step, StepClass,
    StepId, StepOutcome, TxInstrument, power_unsupported,
};
use ndn_radio_hal::{Band, RadioCapability, RadioProfile, RadioTime, RadioTimeSource};
use ndn_transport::FaceError;

// The RF-path enum is shared with the 8822E backend (an A/B selector).
use crate::RfPath;

/// Per-path convergence outcome of `Rtl8812auBackend::iq_calibrate`. A path
/// "converges" when two IQK measurements agree within ±4; otherwise an
/// identity-ish default is loaded and the flag is `false`.
#[derive(Debug, Clone, Copy, Default)]
pub struct IqkResult {
    /// Path A TX IQK converged; `tx_a_xy` holds the applied (X, Y) correction.
    pub tx_a: bool,
    /// Path A RX IQK converged; `rx_a_xy` holds the applied (X, Y) correction.
    pub rx_a: bool,
    /// Path B TX IQK converged.
    pub tx_b: bool,
    /// Path B RX IQK converged.
    pub rx_b: bool,
    /// Path A TX correction matrix (X, Y), 11-bit signed.
    pub tx_a_xy: (i32, i32),
    /// Path A RX correction matrix (X, Y), 11-bit signed.
    pub rx_a_xy: (i32, i32),
}

/// Realtek USB vendor request — identical to the 88xx path: `bRequest = 0x05`,
/// register address in `wValue`, `wIndex = 0`, little-endian data.
const VENDOR_REQ: u8 = 0x05;
const REQ_READ: u8 = 0xc0; // device-to-host | vendor | device
const REQ_WRITE: u8 = 0x40; // host-to-device | vendor | device
const CTRL_TIMEOUT: Duration = Duration::from_millis(500);

/// Realtek USB vendor id.
pub const REALTEK_VID: u16 = 0x0bda;

/// Known **RTL8812AU** USB product ids. Deliberately excludes `0xa81a` (the
/// 8812**EU**) so opening the AU never disturbs the EU monitor dongle on the
/// same host. `0x881a` is the RTL8812AU-VS on the test rig.
pub const RTL8812AU_PIDS: &[u16] = &[0x8812, 0x881a, 0x881b, 0x881c, 0x8813];

/// A product id that selects the RTL8812AU arm of [`open_radio`](crate::open_radio()).
///
/// The arm keys on membership of [`RTL8812AU_PIDS`] and `open_select` then scans the whole set, so
/// any member names the part; this is the one to write when a caller has no PID of its own.
pub const RTL8812AU_PID: u16 = 0x8812;

/// `REG_SYS_CFG` (`0x00F0`) — silicon configuration. For the 8812A the cut
/// version is bits `[15:12]`; the high bits carry vendor/RF-type straps. A read
/// of all-ones (`0xFFFFFFFF`) means the device is wedged / not responding.
pub const REG_SYS_CFG: u16 = 0x00F0;

/// `REG_SYS_CFG1` (`0x00FC`) — additional silicon config (vendor/RF straps).
pub const REG_SYS_CFG1: u16 = 0x00FC;

const REG_SYS_FUNC_EN: u16 = 0x0002;
const REG_RSV_CTRL: u16 = 0x001C;
/// RF enable control (`REG_RF_CTRL`); `0x07` = SDMRSTB|RSTB|EN (path A power-on).
const REG_RF_CTRL: u16 = 0x001F;
/// Path-B RF power-on control (`REG_OPT_CTRL_8812`; `+2` byte = `0x07`).
const REG_OPT_CTRL_8812: u16 = 0x0074;
/// `REG_SYS_FUNC_EN` baseband enables: BB reset, BB global reset-n, USB analog.
const FEN_BBRSTB: u8 = 1 << 0;
const FEN_BB_GLB_RSTN: u8 = 1 << 1;
const FEN_USBA: u8 = 1 << 2;
/// MCU firmware-download control register.
const REG_MCUFWDL: u16 = 0x0080;
/// `REG_MCUFWDL[7]` — the 8051 is running a RAM image (firmware already loaded).
/// A new image must not be written over it; see [`Rtl8812auBackend::download_firmware`].
const MCUFWDL_RAM_DL_SEL: u8 = 0x80;
/// Firmware RAM write window (the 4 KB page selected by `REG_MCUFWDL+2[2:0]`).
const FW_START_ADDRESS: u16 = 0x1000;
const MCUFWDL_RDY: u32 = 1 << 1;
const WINTINI_RDY: u32 = 1 << 6;
/// Firmware-download page size (4 KB) and the per-control-transfer write chunk.
const DLFW_PAGE_SIZE: usize = 4096;
const DLFW_BLOCK: usize = 196;

/// LLT (Link List Table) register — programs the TX packet-buffer page chain.
const REG_LLT_INIT: u16 = 0x01E0;
const LLT_NO_ACTIVE: u32 = 0x0;
const LLT_WRITE_ACCESS: u32 = 0x1;
const LLT_POLL_THRESHOLD: u32 = 20;
/// Last page of the 8812A TX packet buffer.
const LAST_TX_PKT_PAGE: u8 = 255;
/// TX-page boundary that separates the TX-queue page chain from the beacon /
/// loopback ring. `0xFF − BCNQ(7) − WoWLAN(0) − NDPA(0) − DBG(0) + 1` for our
/// non-WoWLAN / non-concurrent build (matches the vendor `TX_PAGE_BOUNDARY_8812`).
const TX_PAGE_BOUNDARY: u8 = 0xF9;

/// One entry of the halmac power sequence (`WLAN_PWR_CFG`), reduced to the fields
/// that matter for our single USB chip: a register op gated only by the command.
struct PwrCfg {
    offset: u16,
    cmd: u8,
    msk: u8,
    value: u8,
}
const PWR_WRITE: u8 = 1;
const PWR_POLLING: u8 = 2;

/// 8812A **card-enable** (`RTL8812_TRANS_CARDEMU_TO_ACT`) — bring the MAC power
/// domain up from card-emulation to active. Ported verbatim from the vendor
/// `Hal8812PwrSeq.h` (USB path; all entries apply to USB).
const CARDEMU_TO_ACT: &[PwrCfg] = &[
    // 0x04[10]=0: disable SW LPS.
    PwrCfg {
        offset: 0x0005,
        cmd: PWR_WRITE,
        msk: 0x04,
        value: 0x00,
    },
    // 0x04[17]=1: poll until power ready.
    PwrCfg {
        offset: 0x0006,
        cmd: PWR_POLLING,
        msk: 0x02,
        value: 0x02,
    },
    // 0x04[11]=0: disable WL suspend.
    PwrCfg {
        offset: 0x0005,
        cmd: PWR_WRITE,
        msk: 0x08,
        value: 0x00,
    },
    // 0x04[8]=1: APFM_ONMAC — turn on MAC via HW state machine.
    PwrCfg {
        offset: 0x0005,
        cmd: PWR_WRITE,
        msk: 0x01,
        value: 0x01,
    },
    // poll until 0x04[8]=0 (state machine done).
    PwrCfg {
        offset: 0x0005,
        cmd: PWR_POLLING,
        msk: 0x01,
        value: 0x00,
    },
    // 0x24[1]=0 / 0x28[3]=0: xosc buffer type.
    PwrCfg {
        offset: 0x0024,
        cmd: PWR_WRITE,
        msk: 0x02,
        value: 0x00,
    },
    PwrCfg {
        offset: 0x0028,
        cmd: PWR_WRITE,
        msk: 0x08,
        value: 0x00,
    },
];

/// 8812A **card-disable** (`RTL8812_TRANS_ACT_TO_CARDEMU`, USB entries) — bring
/// the MAC power domain down to card-emulation. Run before re-powering so a
/// repeated bring-up starts from a known state (`CARDEMU_TO_ACT` polls a
/// transition that never completes if the MAC is already active).
const ACT_TO_CARDEMU: &[PwrCfg] = &[
    // 0xc00/0xe00 = 4: turn off the BB 3-wire (path A/B).
    PwrCfg {
        offset: 0x0c00,
        cmd: PWR_WRITE,
        msk: 0xFF,
        value: 0x04,
    },
    PwrCfg {
        offset: 0x0e00,
        cmd: PWR_WRITE,
        msk: 0xFF,
        value: 0x04,
    },
    // 0x02[0] = 0: reset BB, close RF.
    PwrCfg {
        offset: 0x0002,
        cmd: PWR_WRITE,
        msk: 0x01,
        value: 0x00,
    },
    // 0x07 = 0x2A: SPS PWM mode.
    PwrCfg {
        offset: 0x0007,
        cmd: PWR_WRITE,
        msk: 0xFF,
        value: 0x2A,
    },
    // 0x08[1] = 0: ANA clock 500 kHz.
    PwrCfg {
        offset: 0x0008,
        cmd: PWR_WRITE,
        msk: 0x02,
        value: 0x00,
    },
    // 0x04[9] = 1: turn off MAC via HW state machine, then poll until it clears.
    PwrCfg {
        offset: 0x0005,
        cmd: PWR_WRITE,
        msk: 0x02,
        value: 0x02,
    },
    PwrCfg {
        offset: 0x0005,
        cmd: PWR_POLLING,
        msk: 0x02,
        value: 0x00,
    },
];

/// The vendored 8812A **NIC** firmware (`array_mp_8812a_fw_nic`, v52.14,
/// signature `0x9501`) — extracted from the morrownr/aircrack `rtl8812au` C
/// source. A 32-byte header precedes the body that gets written to the MCU.
static FW_NIC: &[u8] = include_bytes!("../fw/rtl8812au/rtl8812au_fw_nic.bin");
const FW_HDR_LEN: usize = 32;

/// The 8812A MAC register table (`array_mp_8812a_mac_reg`), extracted from the
/// vendor phydm `halhwimg8812a_mac.c` as little-endian `u32` `(addr, value)`
/// pairs interleaved with phydm condition markers. Applied by [`mac_config`].
static MAC_REG: &[u8] = include_bytes!("../fw/rtl8812au/rtl8812au_mac_reg.bin");

/// 8812A baseband (PHY) register table (`array_mp_8812a_phy_reg`) and AGC table
/// (`array_mp_8812a_agc_tab`), extracted from the vendor phydm
/// `halhwimg8812a_bb.c`. Same phydm conditional format as [`MAC_REG`], applied as
/// 32-bit writes with delay opcodes (addr `0xfe..0xf9`). See [`bb_config`](Self::bb_config).
static PHY_REG: &[u8] = include_bytes!("../fw/rtl8812au/rtl8812au_phy_reg.bin");
static AGC_TAB: &[u8] = include_bytes!("../fw/rtl8812au/rtl8812au_agc_tab.bin");

/// 8812A RF register tables for path A (`array_mp_8812a_radioa`) and path B
/// (`array_mp_8812a_radiob`), from phydm `halhwimg8812a_rf.c`. Same conditional
/// format; each entry programs an RF register via the BB LSSI (see [`rf_config`]).
static RADIO_A: &[u8] = include_bytes!("../fw/rtl8812au/rtl8812au_radioa.bin");
static RADIO_B: &[u8] = include_bytes!("../fw/rtl8812au/rtl8812au_radiob.bin");

/// LSSI write registers (BB) per RF path — RF writes ride these (`rA/rB_LSSIWrite_Jaguar`).
const RF_LSSI_WRITE_A: u16 = 0x0C90;
const RF_LSSI_WRITE_B: u16 = 0x0E90;

/// The phydm `driver1` discriminant for **this** chip (`check_positive`): cut B
/// (`1<<24`) · package "don't-care" (`15<<12`) · interface USB (`ODM_ITRF_USB
/// = 2`, in `<<8`). Board-type is 0 (generic dongle), so `driver2`/`driver4` are
/// 0 and only the interface/cut nibbles of a condition can match.
const PHYDM_DRIVER1: u32 = (1 << 24) | (15 << 12) | (0x2 << 8);
const COND_ELSE: u8 = 2;
const COND_ENDIF: u8 = 3;

// MAC-init register map (hal_com_reg.h).
const REG_CR: u16 = 0x0100;
const REG_PBP: u16 = 0x0104;
const REG_TRXDMA_CTRL: u16 = 0x010C;
const REG_TRXFF_BNDY: u16 = 0x0114;
const REG_RQPN: u16 = 0x0200;
const REG_TDECTRL: u16 = 0x0208;
const REG_RQPN_NPQ: u16 = 0x0214;
const REG_BCNQ_BDNY: u16 = 0x0424;
const REG_MGQ_BDNY: u16 = 0x0425;
const REG_WMAC_LBK_BF_HD: u16 = 0x045D;
const REG_RCR: u16 = 0x0608;
/// Station address register (`REG_MACID`, addr1 exact-match target for RCR's APM
/// bit; 6 bytes at 0x0610). Also the responder MAC for a hardware ACK.
const REG_MACID: u16 = 0x0610;
/// RCR bit 0 = **AAP** (Accept All Packets / promiscuous). Set in monitor mode;
/// cleared to make the chip filter RX by addr1 (name-group hardware wake-filter).
const RCR_AAP: u32 = 0x1;
const REG_RX_DRVINFO_SZ: u16 = 0x060F;
const REG_MAR: u16 = 0x0620;

/// EDCCA (energy-detect clear-channel-assessment) control — the carrier-sense
/// knob for the contention fix (#37). Three registers, matched to the
/// aircrack-ng/rtl8812au phydm adaptivity path:
///
/// * `REG_EDCCA_TH` (`0x08a4`, `rFPGA0_XB_LSSIReadBack` on Jaguar): the energy
///   thresholds live in the low word — **byte0 = L2H** (busy-enter), **byte1 =
///   H2L** (busy-exit, hysteresis). Both are signed. `0x7f/0x7f` = maxed out =
///   energy-detect effectively **off** (the promiscuous-injection default).
/// * `REG_TX_PTCL_CTRL` (`0x0520`) **BIT15** = *ignore EDCCA*: when set, the TX
///   engine transmits without deferring to the energy-detect CCA. phydm sets it
///   for `PhyDM_IGNORE_EDCCA`; clearing it makes TX **honor** the medium.
/// * `REG_RD_CTRL` (`0x0524`) **BIT11**: the adaptivity path sets it alongside
///   honoring EDCCA (RX-defer gate).
const REG_EDCCA_TH: u16 = 0x08a4;
const REG_TX_PTCL_CTRL: u16 = 0x0520;
const TX_PTCL_IGNORE_EDCCA: u16 = 1 << 15;
const REG_RD_CTRL: u16 = 0x0524;
const RD_CTRL_EDCCA_EN: u16 = 1 << 11;
/// Frame-free PHY sensing (#30) — read channel occupancy/energy from the chip's
/// own detectors WITHOUT the host decoding a single frame. The registers here are
/// the ones **empirically validated on this silicon** by a quiet-vs-busy scan
/// (`examples/sense_probe.rs scan`), *not* the phydm Jaguar OFDM FA block
/// (`0xF48`–`0xF54`), which reads a flat 0 on this part (the counters are unarmed
/// by our minimal bring-up — the scan proved they never move under traffic).
///
/// * `REG_IGI_A/B` (`0xC50`/`0xE50`, bits[6:0]): the AGC **Initial Gain Index** —
///   the gain the AGC picks to sit above the noise+interference floor, so it is a
///   direct instantaneous energy-floor proxy (a level, not a counter). Flat at
///   sparse ambient; rises under sustained energy/interference.
/// * `REG_RXERR_RPT` (`0x0664`, bits[15:0]): a MAC hardware counter that tracked
///   the decoded-frame rate ~1:1 on busy (0→63 for 65 frames; activity/s == the
///   decoded rate) and stayed 0 on a quiet channel — the validated occupancy
///   signal. Free-running; sample + diff.
///
/// (`0x0F38[31:16]` looked promising in the scan but failed re-validation — 0 on
/// busy, huge/erratic on quiet — so it is deliberately not shipped. And the phydm
/// OFDM FA block at `0xF48`–`0xF54` reads a flat 0 here; both are excluded because
/// the on-chip measurement refuted them.)
const REG_IGI_A: u16 = 0x0C50;
const REG_IGI_B: u16 = 0x0E50;
const REG_RXERR_RPT: u16 = 0x0664;

/// Written to both L2H and H2L to disable energy-detect deferral (`0x7f` = the
/// most-permissive signed threshold; nothing short of a decodable preamble
/// counts as busy).
const EDCCA_OFF: i8 = 0x7f;
/// Default busy-enter → busy-exit hysteresis (dB-ish, register units), matching
/// phydm's `TH_EDCCA_HL_diff` of 7.
const EDCCA_HL_DIFF: i8 = 7;

/// `REG_CR` block-enable bits set right after power-on: HCI TX/RX DMA, MAC TX/RX
/// DMA, protocol, scheduler, security, and the 32k cal timer (= `0x063F`).
const CR_DMA_ENABLE: u16 = 0x063F;
/// MAC TX / RX enable (`REG_CR[7:6]`), set last.
const MACTXEN: u8 = 1 << 6;
const MACRXEN: u8 = 1 << 7;
/// `REG_RQPN`: HPQ 0x10 · LPQ 0x10 · PUBQ 0xD8 · `LD_RQPN` (load) — the
/// reserved-page split for a 3-bulk-OUT-endpoint device (NPQ 0, written to
/// `REG_RQPN_NPQ`). PUBQ = TX_TOTAL(0xF8) − HPQ − LPQ.
const RQPN_3EP: u32 = 0x10 | (0x10 << 8) | (0xD8 << 16) | (1 << 31);
/// `REG_TRXDMA_CTRL` queue→priority map for 3 OUT endpoints: VO/MGT/HI→HIGH,
/// VI→NORMAL, BE/BK→LOW (`0xF5B0`; low 3 bits preserved).
const TRXDMA_MAP_3EP: u16 = 0xF5B0;
/// RX-FIFO boundary (`RX_DMA_BOUNDARY_8812 = MAX_RX_DMA(0x3E80) − reserved(0) − 1`).
const RX_DMA_BOUNDARY: u16 = 0x3E7F;
/// `REG_PBP` transfer page size: TX 512 B (`_PSTX(PBP_512)` = `3<<4`).
const PBP_TX_512: u8 = 0x30;
/// `REG_CR` network-type field (`MASK_NETTYPE` / `_NETTYPE(NT_LINK_AP)`).
const MASK_NETTYPE: u32 = 0x30000;
const NETTYPE_AP: u32 = 0x2 << 16;
/// **Monitor** receive config: accept all unicast/phys-match/multicast/broadcast,
/// data + management frames, no BSSID filter, append PHY status (for RSSI), force
/// ACK off-path. `AAP|APM|AM|AB|ADF|AMF|HTC_LOC_CTRL|APP_PHYST_RXFF|FORCEACK`.
const MONITOR_RCR: u32 = 0x1
    | 0x2
    | 0x4
    | 0x8
    | (1 << 8)
    | (1 << 9)
    | (1 << 11)
    | (1 << 13)
    | (1 << 14)
    | (1 << 28)
    | (1 << 26);
// bit8 ACRC32 + bit9 AICV: keep CRC/ICV-error frames too — a true monitor sees
// marginal/corrupt frames (e.g. an uncalibrated peer's TX), not just clean ones.

// ── Milestone 8: frame injection / capture (TX/RX descriptors) ──────────────
/// 8812A TX descriptor size (`TXDESC_SIZE`): the WiFi-info header prepended to
/// every injected frame, and the `OFFSET` value (the 802.11 frame starts here).
const TXDESC_SIZE: usize = 40;
/// 8812A RX status-descriptor size (`RXDESC_SIZE`).
const RXDESC_SIZE: usize = 24;
/// Hardware rate code (`DESC_RATE6M`) for the `TX_RATE` field — legacy 6 Mbps
/// OFDM, the rate real NAN devices emit beacons/SDFs at.
const DESC_RATE_6M: u32 = 0x04;
/// `DESC_RATEMCS0` — the TX rate code for 802.11n HT MCS0. The MCS index adds on
/// (0–7 = 1 stream, 8–15 = 2 streams).
const DESC_RATE_MCS0: u32 = 0x0c;
/// `DESC_RATEVHTSS1MCS0` — the TX rate code for 802.11ac VHT 1-stream MCS0.
const DESC_RATE_VHT1SS_MCS0: u32 = 0x2c;
/// `DESC_RATEVHTSS2MCS0` — the TX rate code for 802.11ac VHT 2-stream MCS0.
const DESC_RATE_VHT2SS_MCS0: u32 = 0x36;

/// Resolve a control-plane [`McsDescriptor`](crate::McsDescriptor) + the frame's
/// [`TxIntent`](ndn_radio_hal::TxIntent) into the 8812A `TX_RATE` DESC code —
/// the pure decision the [`inject`](Rtl8812auBackend::inject) rate path turns on,
/// factored out so it is unit-testable without a live USB device.
///
/// Mirrors the sibling 88xx backend's rate-code selection (the raw Realtek
/// `DESC_RATE*` table): 802.11n HT is `DESC_RATEMCS0` (`0x0c`) + index (0–7 =
/// 1 stream, 8–15 = 2 streams); 802.11ac VHT is `DESC_RATEVHTSS1MCS0` (`0x2c`) +
/// index for 1 stream or `DESC_RATEVHTSS2MCS0` (`0x36`) + index for `nss >= 2`.
/// Unlike the 8822E path this does **not** remap HT→VHT — that is an 8822E-only
/// silicon workaround (its HT TX is dead on air); the 8812A emits the requested
/// PHY verbatim.
///
/// A `MostRobust` frame (cooperative reports, discovery, control — anything the
/// worst receiver in range must decode) is forced to legacy 6 Mbps OFDM
/// (`DESC_RATE6M`, `0x04`) regardless of the stored rate, exactly as 802.11 sends
/// beacons/probes at a basic rate for universal reach.
fn desc_rate_for(mcs: &crate::McsDescriptor, intent: &ndn_radio_hal::TxIntent) -> u32 {
    if intent.needs_basic_rate() {
        return DESC_RATE_6M;
    }
    let index = mcs.index as u32;
    if mcs.vht {
        if mcs.nss >= 2 {
            DESC_RATE_VHT2SS_MCS0 + index
        } else {
            DESC_RATE_VHT1SS_MCS0 + index
        }
    } else {
        DESC_RATE_MCS0 + index
    }
}

/// A captured register program: `(addr, width-in-bytes, value)` writes applied in order.
type RegProgram = &'static [(u16, u8, u32)];

/// The RTL8812AU **5 GHz ch36** channel program — the BB/RFE/RF/TXAGC band-switch writes
/// captured from the kernel `rtw88_8812au` driver via usbmon (golden/rtw88-8812au-ch36-5g).
/// MAC/sys-control writes are filtered out (they reset the device out of kernel context).
/// `(addr, width_bytes, value)`, applied in order by [`Rtl8812auBackend::set_channel`].
static CH36_5G_PROGRAM: RegProgram = &[
    (0x0860, 4, 0x72d5c321),
    (0x08b0, 4, 0x00000618),
    (0x0c90, 4, 0x0180702a),
    (0x08ac, 4, 0x0ff0fa0a),
    (0x08ac, 4, 0x0ff0fa0a),
    (0x08c4, 4, 0x00000000),
    (0x08b0, 4, 0x00000618),
    (0x0c90, 4, 0x01807001),
    (0x08b0, 4, 0x00000618),
    (0x0e90, 4, 0x0180702a),
    (0x08ac, 4, 0x0ff0fa0a),
    (0x08ac, 4, 0x0ff0fa0a),
    (0x08c4, 4, 0x00000000),
    (0x08b0, 4, 0x00000618),
    (0x0e90, 4, 0x01807001),
    (0x0668, 2, 0x00001000),
    (0x08ac, 4, 0x0ff0fa08),
    (0x08c4, 4, 0x00000000),
    (0x0848, 4, 0x61d0ff8b),
    (0x08ac, 4, 0x0ff0fa08),
    (0x08ac, 4, 0x0ff0fa08),
    (0x08c4, 4, 0x00000000),
    (0x08b0, 4, 0x00000618),
    (0x0c90, 4, 0x01807c01),
    (0x08b0, 4, 0x00000618),
    (0x0e90, 4, 0x01807c01),
    (0x0c20, 4, 0x13131313),
    (0x0c24, 4, 0x20202020),
    (0x0c28, 4, 0x1c1e2020),
    (0x0c2c, 4, 0x20202020),
    (0x0c30, 4, 0x1c1e2020),
    (0x0c34, 4, 0x20202020),
    (0x0c38, 4, 0x1a1c1e20),
    (0x0c3c, 4, 0x20202020),
    (0x0c40, 4, 0x1c1e2020),
    (0x0c44, 4, 0x1e1e181a),
    (0x0c48, 4, 0x1e1e1e1e),
    (0x0c4c, 4, 0x16181a1c),
    (0x0c54, 4, 0x00040a12),
    (0x0e20, 4, 0x14141414),
    (0x0e24, 4, 0x1d1d1d1d),
    (0x0e28, 4, 0x191b1d1d),
    (0x0e2c, 4, 0x1f1f1f1f),
    (0x0e30, 4, 0x1b1d1f1f),
    (0x0e34, 4, 0x1f1f1f1f),
    (0x0e38, 4, 0x191b1d1f),
    (0x0e3c, 4, 0x1f1f1f1f),
    (0x0e40, 4, 0x1b1d1f1f),
    (0x0e44, 4, 0x1d1d1719),
    (0x0e48, 4, 0x1d1d1d1d),
    (0x0e4c, 4, 0x1517191b),
    (0x0e54, 4, 0x00030911),
    (0x0454, 1, 0x00000080),
    (0x0808, 4, 0x3e028233),
    (0x0834, 4, 0x0037a706),
    (0x0830, 4, 0x2eaaaeb8),
    (0x0830, 4, 0x2eaaaeb8),
    (0x082c, 4, 0x002083dd),
    (0x0cb0, 4, 0x54337717),
    (0x0eb0, 4, 0x54337717),
    (0x0cb4, 4, 0x01000077),
    (0x0eb4, 4, 0x01000077),
    (0x0900, 4, 0x00000401),
    (0x080c, 4, 0x12131103),
    (0x0a04, 4, 0x0fff000c),
    (0x0c1c, 4, 0x2d400003),
    (0x0e1c, 4, 0x2d400003),
    (0x0860, 4, 0x6929c321),
    (0x08b0, 4, 0x00000618),
    (0x0c90, 4, 0x01817d01),
    (0x08ac, 4, 0x0ff0fa08),
    (0x08ac, 4, 0x0ff0fa08),
    (0x08c4, 4, 0x00000000),
    (0x08b0, 4, 0x00000618),
    (0x0c90, 4, 0x01817d24),
    (0x08b0, 4, 0x00000618),
    (0x0e90, 4, 0x01817d01),
    (0x08ac, 4, 0x0ff0fa08),
    (0x08ac, 4, 0x0ff0fa08),
    (0x08c4, 4, 0x00000000),
    (0x08b0, 4, 0x00000618),
    (0x0e90, 4, 0x01817d24),
    (0x0668, 2, 0x00001000),
    (0x08ac, 4, 0x0ff0fa08),
    (0x08c4, 4, 0x00000000),
    (0x0848, 4, 0x61d0ff8b),
    (0x08ac, 4, 0x0ff0fa08),
    (0x08ac, 4, 0x0ff0fa08),
    (0x08c4, 4, 0x00000000),
    (0x08b0, 4, 0x00000618),
    (0x0c90, 4, 0x01817d24),
    (0x08b0, 4, 0x00000618),
    (0x0e90, 4, 0x01817d24),
    (0x0c24, 4, 0x27272727),
    (0x0c28, 4, 0x25272727),
    (0x0c2c, 4, 0x2c2c2c2c),
    (0x0c30, 4, 0x26282a2c),
    (0x0c34, 4, 0x28282828),
    (0x0c38, 4, 0x24262828),
    (0x0c3c, 4, 0x2c2c2c2c),
    (0x0c40, 4, 0x26282a2c),
    (0x0c44, 4, 0x28282424),
    (0x0c48, 4, 0x28282828),
    (0x0c4c, 4, 0x22222426),
    (0x0c54, 4, 0x000e141c),
    (0x0e24, 4, 0x23232323),
    (0x0e28, 4, 0x21232323),
    (0x0e2c, 4, 0x28282828),
    (0x0e30, 4, 0x22242628),
    (0x0e34, 4, 0x24242424),
    (0x0e38, 4, 0x20222424),
    (0x0e3c, 4, 0x28282828),
    (0x0e40, 4, 0x22242628),
    (0x0e44, 4, 0x24242020),
    (0x0e48, 4, 0x24242424),
    (0x0e4c, 4, 0x1e1e2022),
    (0x0e54, 4, 0x000a1018),
    (0x09a4, 4, 0x000a0080),
    (0x09a4, 4, 0x00080080),
    (0x0a2c, 4, 0x00900000),
    (0x0a2c, 4, 0x00908000),
    (0x0b58, 4, 0x00000001),
    (0x0b58, 4, 0x00000000),
    (0x0c50, 4, 0x00000022),
    (0x0e50, 4, 0x00000022),
    (0x08b0, 4, 0x00000642),
    (0x0c90, 4, 0x042378f0),
    (0x09a4, 4, 0x000a0080),
    (0x09a4, 4, 0x00080080),
    (0x0a2c, 4, 0x00900000),
    (0x0a2c, 4, 0x00908000),
    (0x0b58, 4, 0x00000001),
    (0x0b58, 4, 0x00000000),
    (0x0c50, 4, 0x00000024),
    (0x0e50, 4, 0x00000024),
    (0x08b0, 4, 0x00000642),
];
/// The RTL8812AU **5 GHz ch149** channel program (golden/rtw88-8812au-ch149-5g), same capture
/// method as ch36 — the per-channel fc_area + TXAGC + RF writes differ; band-switch writes match.
static CH149_5G_PROGRAM: RegProgram = &[
    (0x0860, 4, 0x72d5c321),
    (0x08b0, 4, 0x00000618),
    (0x0c90, 4, 0x0180702a),
    (0x08ac, 4, 0x0ff0fa0a),
    (0x08ac, 4, 0x0ff0fa0a),
    (0x08c4, 4, 0x00000000),
    (0x08b0, 4, 0x00000618),
    (0x0c90, 4, 0x01807001),
    (0x08b0, 4, 0x00000618),
    (0x0e90, 4, 0x0180702a),
    (0x08ac, 4, 0x0ff0fa0a),
    (0x08ac, 4, 0x0ff0fa0a),
    (0x08c4, 4, 0x00000000),
    (0x08b0, 4, 0x00000618),
    (0x0e90, 4, 0x01807001),
    (0x0668, 2, 0x00001000),
    (0x08ac, 4, 0x0ff0fa08),
    (0x08c4, 4, 0x00000000),
    (0x0848, 4, 0x61d0ff8b),
    (0x08ac, 4, 0x0ff0fa08),
    (0x08ac, 4, 0x0ff0fa08),
    (0x08c4, 4, 0x00000000),
    (0x08b0, 4, 0x00000618),
    (0x0c90, 4, 0x01807c01),
    (0x08b0, 4, 0x00000618),
    (0x0e90, 4, 0x01807c01),
    (0x0c20, 4, 0x13131313),
    (0x0c24, 4, 0x20202020),
    (0x0c28, 4, 0x1c1e2020),
    (0x0c2c, 4, 0x20202020),
    (0x0c30, 4, 0x1c1e2020),
    (0x0c34, 4, 0x20202020),
    (0x0c38, 4, 0x1a1c1e20),
    (0x0c3c, 4, 0x20202020),
    (0x0c40, 4, 0x1c1e2020),
    (0x0c44, 4, 0x1e1e181a),
    (0x0c48, 4, 0x1e1e1e1e),
    (0x0c4c, 4, 0x16181a1c),
    (0x0c54, 4, 0x00040a12),
    (0x0e20, 4, 0x14141414),
    (0x0e24, 4, 0x1d1d1d1d),
    (0x0e28, 4, 0x191b1d1d),
    (0x0e2c, 4, 0x1f1f1f1f),
    (0x0e30, 4, 0x1b1d1f1f),
    (0x0e34, 4, 0x1f1f1f1f),
    (0x0e38, 4, 0x191b1d1f),
    (0x0e3c, 4, 0x1f1f1f1f),
    (0x0e40, 4, 0x1b1d1f1f),
    (0x0e44, 4, 0x1d1d1719),
    (0x0e48, 4, 0x1d1d1d1d),
    (0x0e4c, 4, 0x1517191b),
    (0x0e54, 4, 0x00030911),
    (0x0454, 1, 0x00000080),
    (0x0808, 4, 0x3e028233),
    (0x0834, 4, 0x0037a706),
    (0x0830, 4, 0x2eaaaeb8),
    (0x0830, 4, 0x2eaaaeb8),
    (0x082c, 4, 0x002083dd),
    (0x0cb0, 4, 0x54337717),
    (0x0eb0, 4, 0x54337717),
    (0x0cb4, 4, 0x01000077),
    (0x0eb4, 4, 0x01000077),
    (0x0900, 4, 0x00000401),
    (0x080c, 4, 0x12131103),
    (0x0a04, 4, 0x0fff000c),
    (0x0c1c, 4, 0x2d400003),
    (0x0e1c, 4, 0x2d400003),
    (0x0860, 4, 0x6825c321),
    (0x08b0, 4, 0x00000618),
    (0x0c90, 4, 0x01857d01),
    (0x08ac, 4, 0x0ff0fa08),
    (0x08ac, 4, 0x0ff0fa08),
    (0x08c4, 4, 0x00000000),
    (0x08b0, 4, 0x00000618),
    (0x0c90, 4, 0x01817d95),
    (0x08b0, 4, 0x00000618),
    (0x0e90, 4, 0x01857d01),
    (0x08ac, 4, 0x0ff0fa08),
    (0x08ac, 4, 0x0ff0fa08),
    (0x08c4, 4, 0x00000000),
    (0x08b0, 4, 0x00000618),
    (0x0e90, 4, 0x01817d95),
    (0x0668, 2, 0x00001000),
    (0x08ac, 4, 0x0ff0fa08),
    (0x08c4, 4, 0x00000000),
    (0x0848, 4, 0x61d0ff8b),
    (0x08ac, 4, 0x0ff0fa08),
    (0x08ac, 4, 0x0ff0fa08),
    (0x08c4, 4, 0x00000000),
    (0x08b0, 4, 0x00000618),
    (0x0c90, 4, 0x01857d95),
    (0x08b0, 4, 0x00000618),
    (0x0e90, 4, 0x01857d95),
    (0x0c24, 4, 0x30303030),
    (0x0c28, 4, 0x2c2e3030),
    (0x0c2c, 4, 0x35353535),
    (0x0c30, 4, 0x2d2f3133),
    (0x0c34, 4, 0x31313131),
    (0x0c38, 4, 0x2b2d2f31),
    (0x0c3c, 4, 0x35353535),
    (0x0c40, 4, 0x2d2f3133),
    (0x0c44, 4, 0x31312b2b),
    (0x0c48, 4, 0x2f313131),
    (0x0c4c, 4, 0x29292b2d),
    (0x0c54, 4, 0x00151b23),
    (0x0e24, 4, 0x1e1e1e1e),
    (0x0e28, 4, 0x1a1c1e1e),
    (0x0e2c, 4, 0x23232323),
    (0x0e30, 4, 0x1b1d1f21),
    (0x0e34, 4, 0x1f1f1f1f),
    (0x0e38, 4, 0x191b1d1f),
    (0x0e3c, 4, 0x23232323),
    (0x0e40, 4, 0x1b1d1f21),
    (0x0e44, 4, 0x1f1f1919),
    (0x0e48, 4, 0x1d1f1f1f),
    (0x0e4c, 4, 0x1717191b),
    (0x0e54, 4, 0x00030911),
    (0x09a4, 4, 0x000a0080),
    (0x09a4, 4, 0x00080080),
    (0x0a2c, 4, 0x00900000),
    (0x0a2c, 4, 0x00908000),
    (0x0b58, 4, 0x00000001),
    (0x0b58, 4, 0x00000000),
    (0x0c50, 4, 0x00000022),
    (0x0e50, 4, 0x00000022),
    (0x08b0, 4, 0x00000642),
    (0x0c90, 4, 0x04238910),
    (0x09a4, 4, 0x000a0080),
    (0x09a4, 4, 0x00080080),
    (0x0a2c, 4, 0x00900000),
    (0x0a2c, 4, 0x00908000),
    (0x0b58, 4, 0x00000001),
    (0x0b58, 4, 0x00000000),
    (0x0c50, 4, 0x00000020),
    (0x0e50, 4, 0x00000020),
    (0x08b0, 4, 0x00000642),
    (0x0c1c, 4, 0x39000003),
    (0x0e1c, 4, 0x32e00003),
];

/// The RTL8812AU **5 GHz ch44** channel program (UNII-1) — golden trace
/// (golden/rtw88-8812au-ch44-5g), same capture/decode as ch36.
static CH44_5G_PROGRAM: RegProgram = &[
    (0x0860, 4, 0x72d5c321),
    (0x08b0, 4, 0x00000618),
    (0x0c90, 4, 0x0180702a),
    (0x08ac, 4, 0x0ff0fa0a),
    (0x08ac, 4, 0x0ff0fa0a),
    (0x08c4, 4, 0x00000000),
    (0x08b0, 4, 0x00000618),
    (0x0c90, 4, 0x01807001),
    (0x08b0, 4, 0x00000618),
    (0x0e90, 4, 0x0180702a),
    (0x08ac, 4, 0x0ff0fa0a),
    (0x08ac, 4, 0x0ff0fa0a),
    (0x08c4, 4, 0x00000000),
    (0x08b0, 4, 0x00000618),
    (0x0e90, 4, 0x01807001),
    (0x0668, 2, 0x00001000),
    (0x08ac, 4, 0x0ff0fa08),
    (0x08c4, 4, 0x00000000),
    (0x0848, 4, 0x61d0ff8b),
    (0x08ac, 4, 0x0ff0fa08),
    (0x08ac, 4, 0x0ff0fa08),
    (0x08c4, 4, 0x00000000),
    (0x08b0, 4, 0x00000618),
    (0x0c90, 4, 0x01807c01),
    (0x08b0, 4, 0x00000618),
    (0x0e90, 4, 0x01807c01),
    (0x0c20, 4, 0x13131313),
    (0x0c24, 4, 0x20202020),
    (0x0c28, 4, 0x1c1e2020),
    (0x0c2c, 4, 0x20202020),
    (0x0c30, 4, 0x1c1e2020),
    (0x0c34, 4, 0x20202020),
    (0x0c38, 4, 0x1a1c1e20),
    (0x0c3c, 4, 0x20202020),
    (0x0c40, 4, 0x1c1e2020),
    (0x0c44, 4, 0x1e1e181a),
    (0x0c48, 4, 0x1e1e1e1e),
    (0x0c4c, 4, 0x16181a1c),
    (0x0c54, 4, 0x00040a12),
    (0x0e20, 4, 0x14141414),
    (0x0e24, 4, 0x1d1d1d1d),
    (0x0e28, 4, 0x191b1d1d),
    (0x0e2c, 4, 0x1f1f1f1f),
    (0x0e30, 4, 0x1b1d1f1f),
    (0x0e34, 4, 0x1f1f1f1f),
    (0x0e38, 4, 0x191b1d1f),
    (0x0e3c, 4, 0x1f1f1f1f),
    (0x0e40, 4, 0x1b1d1f1f),
    (0x0e44, 4, 0x1d1d1719),
    (0x0e48, 4, 0x1d1d1d1d),
    (0x0e4c, 4, 0x1517191b),
    (0x0e54, 4, 0x00030911),
    (0x0454, 1, 0x00000080),
    (0x0808, 4, 0x3e028233),
    (0x0834, 4, 0x0037a706),
    (0x0830, 4, 0x2eaaaeb8),
    (0x0830, 4, 0x2eaaaeb8),
    (0x082c, 4, 0x002083dd),
    (0x0cb0, 4, 0x54337717),
    (0x0eb0, 4, 0x54337717),
    (0x0cb4, 4, 0x01000077),
    (0x0eb4, 4, 0x01000077),
    (0x0900, 4, 0x00000401),
    (0x080c, 4, 0x12131103),
    (0x0a04, 4, 0x0fff000c),
    (0x0c1c, 4, 0x2d400003),
    (0x0e1c, 4, 0x2d400003),
    (0x0860, 4, 0x6929c321),
    (0x08b0, 4, 0x00000618),
    (0x0c90, 4, 0x01817d01),
    (0x08ac, 4, 0x0ff0fa08),
    (0x08ac, 4, 0x0ff0fa08),
    (0x08c4, 4, 0x00000000),
    (0x08b0, 4, 0x00000618),
    (0x0c90, 4, 0x01817d2c),
    (0x08b0, 4, 0x00000618),
    (0x0e90, 4, 0x01817d01),
    (0x08ac, 4, 0x0ff0fa08),
    (0x08ac, 4, 0x0ff0fa08),
    (0x08c4, 4, 0x00000000),
    (0x08b0, 4, 0x00000618),
    (0x0e90, 4, 0x01817d2c),
    (0x0668, 2, 0x00001000),
    (0x08ac, 4, 0x0ff0fa08),
    (0x08c4, 4, 0x00000000),
    (0x0848, 4, 0x61d0ff8b),
    (0x08ac, 4, 0x0ff0fa08),
    (0x08ac, 4, 0x0ff0fa08),
    (0x08c4, 4, 0x00000000),
    (0x08b0, 4, 0x00000618),
    (0x0c90, 4, 0x01817d2c),
    (0x08b0, 4, 0x00000618),
    (0x0e90, 4, 0x01817d2c),
    (0x0c24, 4, 0x24242424),
    (0x0c28, 4, 0x22242424),
    (0x0c2c, 4, 0x29292929),
    (0x0c30, 4, 0x23252729),
    (0x0c34, 4, 0x25252525),
    (0x0c38, 4, 0x21232525),
    (0x0c3c, 4, 0x29292929),
    (0x0c40, 4, 0x23252729),
    (0x0c44, 4, 0x25252121),
    (0x0c48, 4, 0x25252525),
    (0x0c4c, 4, 0x1f1f2123),
    (0x0c54, 4, 0x000b1119),
    (0x0e24, 4, 0x21212121),
    (0x0e28, 4, 0x1f212121),
    (0x0e2c, 4, 0x26262626),
    (0x0e30, 4, 0x20222426),
    (0x0e34, 4, 0x22222222),
    (0x0e38, 4, 0x1e202222),
    (0x0e3c, 4, 0x26262626),
    (0x0e40, 4, 0x20222426),
    (0x0e44, 4, 0x22221e1e),
    (0x0e48, 4, 0x22222222),
    (0x0e4c, 4, 0x1c1c1e20),
    (0x0e54, 4, 0x00080e16),
    (0x09a4, 4, 0x000a0080),
    (0x09a4, 4, 0x00080080),
    (0x0a2c, 4, 0x00900000),
    (0x0a2c, 4, 0x00908000),
    (0x0b58, 4, 0x00000001),
    (0x0b58, 4, 0x00000000),
    (0x0c50, 4, 0x00000022),
    (0x0e50, 4, 0x00000022),
    (0x08b0, 4, 0x00000642),
    (0x0c90, 4, 0x04238910),
    (0x09a4, 4, 0x000a0080),
    (0x09a4, 4, 0x00080080),
    (0x0a2c, 4, 0x00900000),
    (0x0a2c, 4, 0x00908000),
    (0x0b58, 4, 0x00000001),
    (0x0b58, 4, 0x00000000),
    (0x0c50, 4, 0x00000020),
    (0x0e50, 4, 0x00000020),
    (0x08b0, 4, 0x00000642),
    (0x0c1c, 4, 0x32e00003),
    (0x0e1c, 4, 0x35e00003),
];

/// The RTL8812AU **5 GHz ch157** channel program (UNII-3) — golden trace
/// (golden/rtw88-8812au-ch157-5g), same capture/decode as ch36.
static CH157_5G_PROGRAM: RegProgram = &[
    (0x0860, 4, 0x72d5c321),
    (0x08b0, 4, 0x00000618),
    (0x0c90, 4, 0x01807c01),
    (0x08ac, 4, 0x0ff0fa08),
    (0x08ac, 4, 0x0ff0fa08),
    (0x08c4, 4, 0x00000000),
    (0x08b0, 4, 0x00000618),
    (0x0c90, 4, 0x01807c01),
    (0x08b0, 4, 0x00000618),
    (0x0e90, 4, 0x01807c01),
    (0x08ac, 4, 0x0ff0fa08),
    (0x08ac, 4, 0x0ff0fa08),
    (0x08c4, 4, 0x00000000),
    (0x08b0, 4, 0x00000618),
    (0x0e90, 4, 0x01807c01),
    (0x0668, 2, 0x00001000),
    (0x08ac, 4, 0x0ff0fa08),
    (0x08c4, 4, 0x00000000),
    (0x0848, 4, 0x61d0ff8b),
    (0x08ac, 4, 0x0ff0fa08),
    (0x08ac, 4, 0x0ff0fa08),
    (0x08c4, 4, 0x00000000),
    (0x08b0, 4, 0x00000618),
    (0x0c90, 4, 0x01807c01),
    (0x08b0, 4, 0x00000618),
    (0x0e90, 4, 0x01807c01),
    (0x0c20, 4, 0x13131313),
    (0x0c24, 4, 0x20202020),
    (0x0c28, 4, 0x1c1e2020),
    (0x0c2c, 4, 0x20202020),
    (0x0c30, 4, 0x1c1e2020),
    (0x0c34, 4, 0x20202020),
    (0x0c38, 4, 0x1a1c1e20),
    (0x0c3c, 4, 0x20202020),
    (0x0c40, 4, 0x1c1e2020),
    (0x0c44, 4, 0x1e1e181a),
    (0x0c48, 4, 0x1e1e1e1e),
    (0x0c4c, 4, 0x16181a1c),
    (0x0c54, 4, 0x00040a12),
    (0x0e20, 4, 0x14141414),
    (0x0e24, 4, 0x1d1d1d1d),
    (0x0e28, 4, 0x191b1d1d),
    (0x0e2c, 4, 0x1f1f1f1f),
    (0x0e30, 4, 0x1b1d1f1f),
    (0x0e34, 4, 0x1f1f1f1f),
    (0x0e38, 4, 0x191b1d1f),
    (0x0e3c, 4, 0x1f1f1f1f),
    (0x0e40, 4, 0x1b1d1f1f),
    (0x0e44, 4, 0x1d1d1719),
    (0x0e48, 4, 0x1d1d1d1d),
    (0x0e4c, 4, 0x1517191b),
    (0x0e54, 4, 0x00030911),
    (0x0454, 1, 0x00000080),
    (0x0808, 4, 0x3e028233),
    (0x0834, 4, 0x0037a706),
    (0x0830, 4, 0x2eaaaeb8),
    (0x0830, 4, 0x2eaaaeb8),
    (0x082c, 4, 0x002083dd),
    (0x0cb0, 4, 0x54337717),
    (0x0eb0, 4, 0x54337717),
    (0x0cb4, 4, 0x01000077),
    (0x0eb4, 4, 0x01000077),
    (0x0900, 4, 0x00000401),
    (0x080c, 4, 0x12131103),
    (0x0a04, 4, 0x0fff000c),
    (0x0c1c, 4, 0x2d400003),
    (0x0e1c, 4, 0x2d400003),
    (0x0860, 4, 0x6825c321),
    (0x08b0, 4, 0x00000618),
    (0x0c90, 4, 0x01857d01),
    (0x08ac, 4, 0x0ff0fa08),
    (0x08ac, 4, 0x0ff0fa08),
    (0x08c4, 4, 0x00000000),
    (0x08b0, 4, 0x00000618),
    (0x0c90, 4, 0x01817d9d),
    (0x08b0, 4, 0x00000618),
    (0x0e90, 4, 0x01857d01),
    (0x08ac, 4, 0x0ff0fa08),
    (0x08ac, 4, 0x0ff0fa08),
    (0x08c4, 4, 0x00000000),
    (0x08b0, 4, 0x00000618),
    (0x0e90, 4, 0x01817d9d),
    (0x0668, 2, 0x00001000),
    (0x08ac, 4, 0x0ff0fa08),
    (0x08c4, 4, 0x00000000),
    (0x0848, 4, 0x61d0ff8b),
    (0x08ac, 4, 0x0ff0fa08),
    (0x08ac, 4, 0x0ff0fa08),
    (0x08c4, 4, 0x00000000),
    (0x08b0, 4, 0x00000618),
    (0x0c90, 4, 0x01857d9d),
    (0x08b0, 4, 0x00000618),
    (0x0e90, 4, 0x01857d9d),
    (0x0c24, 4, 0x30303030),
    (0x0c28, 4, 0x2c2e3030),
    (0x0c2c, 4, 0x35353535),
    (0x0c30, 4, 0x2d2f3133),
    (0x0c34, 4, 0x31313131),
    (0x0c38, 4, 0x2b2d2f31),
    (0x0c3c, 4, 0x35353535),
    (0x0c40, 4, 0x2d2f3133),
    (0x0c44, 4, 0x31312b2b),
    (0x0c48, 4, 0x2f313131),
    (0x0c4c, 4, 0x29292b2d),
    (0x0c54, 4, 0x00151b23),
    (0x0e24, 4, 0x20202020),
    (0x0e28, 4, 0x1c1e2020),
    (0x0e2c, 4, 0x25252525),
    (0x0e30, 4, 0x1d1f2123),
    (0x0e34, 4, 0x21212121),
    (0x0e38, 4, 0x1b1d1f21),
    (0x0e3c, 4, 0x25252525),
    (0x0e40, 4, 0x1d1f2123),
    (0x0e44, 4, 0x21211b1b),
    (0x0e48, 4, 0x1f212121),
    (0x0e4c, 4, 0x19191b1d),
    (0x0e54, 4, 0x00050b13),
    (0x09a4, 4, 0x000a0080),
    (0x09a4, 4, 0x00080080),
    (0x0a2c, 4, 0x00900000),
    (0x0a2c, 4, 0x00908000),
    (0x0b58, 4, 0x00000001),
    (0x0b58, 4, 0x00000000),
    (0x0c50, 4, 0x0000001c),
    (0x0e50, 4, 0x0000001c),
    (0x08b0, 4, 0x00000642),
    (0x0c90, 4, 0x04238d10),
    (0x09a4, 4, 0x000a0080),
    (0x09a4, 4, 0x00080080),
    (0x0a2c, 4, 0x00900000),
    (0x0a2c, 4, 0x00908000),
    (0x0b58, 4, 0x00000001),
    (0x0b58, 4, 0x00000000),
    (0x08b0, 4, 0x00000642),
    (0x0c1c, 4, 0x39000003),
    (0x0e1c, 4, 0x32e00003),
];

/// The RTL8812AU **5 GHz ch40** channel program (UNII-1) — golden trace
/// (golden/rtw88-8812au-ch40-5g), same capture/decode as ch36.
static CH40_5G_PROGRAM: RegProgram = &[
    (0x0860, 4, 0x72d5c321),
    (0x08b0, 4, 0x00000618),
    (0x0c90, 4, 0x0180702a),
    (0x08ac, 4, 0x0ff0fa0a),
    (0x08ac, 4, 0x0ff0fa0a),
    (0x08c4, 4, 0x00000000),
    (0x08b0, 4, 0x00000618),
    (0x0c90, 4, 0x01807001),
    (0x08b0, 4, 0x00000618),
    (0x0e90, 4, 0x0180702a),
    (0x08ac, 4, 0x0ff0fa0a),
    (0x08ac, 4, 0x0ff0fa0a),
    (0x08c4, 4, 0x00000000),
    (0x08b0, 4, 0x00000618),
    (0x0e90, 4, 0x01807001),
    (0x0668, 2, 0x00001000),
    (0x08ac, 4, 0x0ff0fa08),
    (0x08c4, 4, 0x00000000),
    (0x0848, 4, 0x61d0ff8b),
    (0x08ac, 4, 0x0ff0fa08),
    (0x08ac, 4, 0x0ff0fa08),
    (0x08c4, 4, 0x00000000),
    (0x08b0, 4, 0x00000618),
    (0x0c90, 4, 0x01807c01),
    (0x08b0, 4, 0x00000618),
    (0x0e90, 4, 0x01807c01),
    (0x0c20, 4, 0x13131313),
    (0x0c24, 4, 0x20202020),
    (0x0c28, 4, 0x1c1e2020),
    (0x0c2c, 4, 0x20202020),
    (0x0c30, 4, 0x1c1e2020),
    (0x0c34, 4, 0x20202020),
    (0x0c38, 4, 0x1a1c1e20),
    (0x0c3c, 4, 0x20202020),
    (0x0c40, 4, 0x1c1e2020),
    (0x0c44, 4, 0x1e1e181a),
    (0x0c48, 4, 0x1e1e1e1e),
    (0x0c4c, 4, 0x16181a1c),
    (0x0c54, 4, 0x00040a12),
    (0x0e20, 4, 0x14141414),
    (0x0e24, 4, 0x1d1d1d1d),
    (0x0e28, 4, 0x191b1d1d),
    (0x0e2c, 4, 0x1f1f1f1f),
    (0x0e30, 4, 0x1b1d1f1f),
    (0x0e34, 4, 0x1f1f1f1f),
    (0x0e38, 4, 0x191b1d1f),
    (0x0e3c, 4, 0x1f1f1f1f),
    (0x0e40, 4, 0x1b1d1f1f),
    (0x0e44, 4, 0x1d1d1719),
    (0x0e48, 4, 0x1d1d1d1d),
    (0x0e4c, 4, 0x1517191b),
    (0x0e54, 4, 0x00030911),
    (0x0454, 1, 0x00000080),
    (0x0808, 4, 0x3e028233),
    (0x0834, 4, 0x0037a706),
    (0x0830, 4, 0x2eaaaeb8),
    (0x0830, 4, 0x2eaaaeb8),
    (0x082c, 4, 0x002083dd),
    (0x0cb0, 4, 0x54337717),
    (0x0eb0, 4, 0x54337717),
    (0x0cb4, 4, 0x01000077),
    (0x0eb4, 4, 0x01000077),
    (0x0900, 4, 0x00000401),
    (0x080c, 4, 0x12131103),
    (0x0a04, 4, 0x0fff000c),
    (0x0c1c, 4, 0x2d400003),
    (0x0e1c, 4, 0x2d400003),
    (0x0860, 4, 0x6929c321),
    (0x08b0, 4, 0x00000618),
    (0x0c90, 4, 0x01817d01),
    (0x08ac, 4, 0x0ff0fa08),
    (0x08ac, 4, 0x0ff0fa08),
    (0x08c4, 4, 0x00000000),
    (0x08b0, 4, 0x00000618),
    (0x0c90, 4, 0x01817d28),
    (0x08b0, 4, 0x00000618),
    (0x0e90, 4, 0x01817d01),
    (0x08ac, 4, 0x0ff0fa08),
    (0x08ac, 4, 0x0ff0fa08),
    (0x08c4, 4, 0x00000000),
    (0x08b0, 4, 0x00000618),
    (0x0e90, 4, 0x01817d28),
    (0x0668, 2, 0x00001000),
    (0x08ac, 4, 0x0ff0fa08),
    (0x08c4, 4, 0x00000000),
    (0x0848, 4, 0x61d0ff8b),
    (0x08ac, 4, 0x0ff0fa08),
    (0x08ac, 4, 0x0ff0fa08),
    (0x08c4, 4, 0x00000000),
    (0x08b0, 4, 0x00000618),
    (0x0c90, 4, 0x01817d28),
    (0x08b0, 4, 0x00000618),
    (0x0e90, 4, 0x01817d28),
    (0x0c24, 4, 0x27272727),
    (0x0c28, 4, 0x25272727),
    (0x0c2c, 4, 0x2c2c2c2c),
    (0x0c30, 4, 0x26282a2c),
    (0x0c34, 4, 0x28282828),
    (0x0c38, 4, 0x24262828),
    (0x0c3c, 4, 0x2c2c2c2c),
    (0x0c40, 4, 0x26282a2c),
    (0x0c44, 4, 0x28282424),
    (0x0c48, 4, 0x28282828),
    (0x0c4c, 4, 0x22222426),
    (0x0c54, 4, 0x000e141c),
    (0x0e24, 4, 0x23232323),
    (0x0e28, 4, 0x21232323),
    (0x0e2c, 4, 0x28282828),
    (0x0e30, 4, 0x22242628),
    (0x0e34, 4, 0x24242424),
    (0x0e38, 4, 0x20222424),
    (0x0e3c, 4, 0x28282828),
    (0x0e40, 4, 0x22242628),
    (0x0e44, 4, 0x24242020),
    (0x0e48, 4, 0x24242424),
    (0x0e4c, 4, 0x1e1e2022),
    (0x0e54, 4, 0x000a1018),
    (0x09a4, 4, 0x000a0080),
    (0x09a4, 4, 0x00080080),
    (0x0a2c, 4, 0x00900000),
    (0x0a2c, 4, 0x00908000),
    (0x0b58, 4, 0x00000001),
    (0x0b58, 4, 0x00000000),
    (0x0c50, 4, 0x00000022),
    (0x0e50, 4, 0x00000022),
    (0x08b0, 4, 0x00000642),
    (0x0c90, 4, 0x04238910),
    (0x09a4, 4, 0x000a0080),
    (0x09a4, 4, 0x00080080),
    (0x0a2c, 4, 0x00900000),
    (0x0a2c, 4, 0x00908000),
    (0x0b58, 4, 0x00000001),
    (0x0b58, 4, 0x00000000),
    (0x0c50, 4, 0x00000020),
    (0x0e50, 4, 0x00000020),
    (0x08b0, 4, 0x00000642),
    (0x0c1c, 4, 0x32e00003),
    (0x0e1c, 4, 0x35e00003),
];

/// The RTL8812AU **5 GHz ch48** channel program (UNII-1) — golden trace
/// (golden/rtw88-8812au-ch48-5g), same capture/decode as ch36.
static CH48_5G_PROGRAM: RegProgram = &[
    (0x09a4, 4, 0x000a0080),
    (0x09a4, 4, 0x00080080),
    (0x0a2c, 4, 0x00900000),
    (0x0a2c, 4, 0x00908000),
    (0x0b58, 4, 0x00000001),
    (0x0b58, 4, 0x00000000),
    (0x0c50, 4, 0x00000021),
    (0x0e50, 4, 0x00000021),
    (0x08b0, 4, 0x00000642),
    (0x0c90, 4, 0x04238d10),
    (0x0860, 4, 0x72d5c321),
    (0x08b0, 4, 0x00000618),
    (0x0c90, 4, 0x01807c01),
    (0x08ac, 4, 0x0ff0fa08),
    (0x08ac, 4, 0x0ff0fa08),
    (0x08c4, 4, 0x00000000),
    (0x08b0, 4, 0x00000618),
    (0x0c90, 4, 0x01807c01),
    (0x08b0, 4, 0x00000618),
    (0x0e90, 4, 0x01807c01),
    (0x08ac, 4, 0x0ff0fa08),
    (0x08ac, 4, 0x0ff0fa08),
    (0x08c4, 4, 0x00000000),
    (0x08b0, 4, 0x00000618),
    (0x0e90, 4, 0x01807c01),
    (0x0668, 2, 0x00001000),
    (0x08ac, 4, 0x0ff0fa08),
    (0x08c4, 4, 0x00000000),
    (0x0848, 4, 0x61d0ff8b),
    (0x08ac, 4, 0x0ff0fa08),
    (0x08ac, 4, 0x0ff0fa08),
    (0x08c4, 4, 0x00000000),
    (0x08b0, 4, 0x00000618),
    (0x0c90, 4, 0x01807c01),
    (0x08b0, 4, 0x00000618),
    (0x0e90, 4, 0x01807c01),
    (0x0c20, 4, 0x13131313),
    (0x0c24, 4, 0x20202020),
    (0x0c28, 4, 0x1c1e2020),
    (0x0c2c, 4, 0x20202020),
    (0x0c30, 4, 0x1c1e2020),
    (0x0c34, 4, 0x20202020),
    (0x0c38, 4, 0x1a1c1e20),
    (0x0c3c, 4, 0x20202020),
    (0x0c40, 4, 0x1c1e2020),
    (0x0c44, 4, 0x1e1e181a),
    (0x0c48, 4, 0x1e1e1e1e),
    (0x0c4c, 4, 0x16181a1c),
    (0x0c54, 4, 0x00040a12),
    (0x0e20, 4, 0x14141414),
    (0x0e24, 4, 0x1d1d1d1d),
    (0x0e28, 4, 0x191b1d1d),
    (0x0e2c, 4, 0x1f1f1f1f),
    (0x0e30, 4, 0x1b1d1f1f),
    (0x0e34, 4, 0x1f1f1f1f),
    (0x0e38, 4, 0x191b1d1f),
    (0x0e3c, 4, 0x1f1f1f1f),
    (0x0e40, 4, 0x1b1d1f1f),
    (0x0e44, 4, 0x1d1d1719),
    (0x0e48, 4, 0x1d1d1d1d),
    (0x0e4c, 4, 0x1517191b),
    (0x0e54, 4, 0x00030911),
    (0x0454, 1, 0x00000080),
    (0x0808, 4, 0x3e028233),
    (0x0834, 4, 0x0037a706),
    (0x0830, 4, 0x2eaaaeb8),
    (0x0830, 4, 0x2eaaaeb8),
    (0x082c, 4, 0x002083dd),
    (0x0cb0, 4, 0x54337717),
    (0x0eb0, 4, 0x54337717),
    (0x0cb4, 4, 0x01000077),
    (0x0eb4, 4, 0x01000077),
    (0x0900, 4, 0x00000401),
    (0x080c, 4, 0x12131103),
    (0x0a04, 4, 0x0fff000c),
    (0x0c1c, 4, 0x2d400003),
    (0x0e1c, 4, 0x2d400003),
    (0x0860, 4, 0x6929c321),
    (0x08b0, 4, 0x00000618),
    (0x0c90, 4, 0x01817d01),
    (0x08ac, 4, 0x0ff0fa08),
    (0x08ac, 4, 0x0ff0fa08),
    (0x08c4, 4, 0x00000000),
    (0x08b0, 4, 0x00000618),
    (0x0c90, 4, 0x01817d30),
    (0x08b0, 4, 0x00000618),
    (0x0e90, 4, 0x01817d01),
    (0x08ac, 4, 0x0ff0fa08),
    (0x08ac, 4, 0x0ff0fa08),
    (0x08c4, 4, 0x00000000),
    (0x08b0, 4, 0x00000618),
    (0x0e90, 4, 0x01817d30),
    (0x0668, 2, 0x00001000),
    (0x08ac, 4, 0x0ff0fa08),
    (0x08c4, 4, 0x00000000),
    (0x0848, 4, 0x61d0ff8b),
    (0x08ac, 4, 0x0ff0fa08),
    (0x08ac, 4, 0x0ff0fa08),
    (0x08c4, 4, 0x00000000),
    (0x08b0, 4, 0x00000618),
    (0x0c90, 4, 0x01817d30),
    (0x08b0, 4, 0x00000618),
    (0x0e90, 4, 0x01817d30),
    (0x0c24, 4, 0x24242424),
    (0x0c28, 4, 0x22242424),
    (0x0c2c, 4, 0x29292929),
    (0x0c30, 4, 0x23252729),
    (0x0c34, 4, 0x25252525),
    (0x0c38, 4, 0x21232525),
    (0x0c3c, 4, 0x29292929),
    (0x0c40, 4, 0x23252729),
    (0x0c44, 4, 0x25252121),
    (0x0c48, 4, 0x25252525),
    (0x0c4c, 4, 0x1f1f2123),
    (0x0c54, 4, 0x000b1119),
    (0x0e24, 4, 0x21212121),
    (0x0e28, 4, 0x1f212121),
    (0x0e2c, 4, 0x26262626),
    (0x0e30, 4, 0x20222426),
    (0x0e34, 4, 0x22222222),
    (0x0e38, 4, 0x1e202222),
    (0x0e3c, 4, 0x26262626),
    (0x0e40, 4, 0x20222426),
    (0x0e44, 4, 0x22221e1e),
    (0x0e48, 4, 0x22222222),
    (0x0e4c, 4, 0x1c1c1e20),
    (0x0e54, 4, 0x00080e16),
    (0x09a4, 4, 0x000a0080),
    (0x09a4, 4, 0x00080080),
    (0x0a2c, 4, 0x00900000),
    (0x0a2c, 4, 0x00908000),
    (0x0b58, 4, 0x00000001),
    (0x0b58, 4, 0x00000000),
    (0x08b0, 4, 0x00000642),
    (0x0c90, 4, 0x04238d10),
];

/// The RTL8812AU **5 GHz ch100** channel program (UNII-2C DFS) — golden trace
/// (golden/rtw88-8812au-ch100-5g), same capture/decode as ch36.
static CH100_5G_PROGRAM: RegProgram = &[
    (0x0860, 4, 0x72d5c321),
    (0x08b0, 4, 0x00000618),
    (0x0c90, 4, 0x01807c01),
    (0x08ac, 4, 0x0ff0fa08),
    (0x08ac, 4, 0x0ff0fa08),
    (0x08c4, 4, 0x00000000),
    (0x08b0, 4, 0x00000618),
    (0x0c90, 4, 0x01807c01),
    (0x08b0, 4, 0x00000618),
    (0x0e90, 4, 0x01807c01),
    (0x08ac, 4, 0x0ff0fa08),
    (0x08ac, 4, 0x0ff0fa08),
    (0x08c4, 4, 0x00000000),
    (0x08b0, 4, 0x00000618),
    (0x0e90, 4, 0x01807c01),
    (0x0668, 2, 0x00001000),
    (0x08ac, 4, 0x0ff0fa08),
    (0x08c4, 4, 0x00000000),
    (0x0848, 4, 0x61d0ff8b),
    (0x08ac, 4, 0x0ff0fa08),
    (0x08ac, 4, 0x0ff0fa08),
    (0x08c4, 4, 0x00000000),
    (0x08b0, 4, 0x00000618),
    (0x0c90, 4, 0x01807c01),
    (0x08b0, 4, 0x00000618),
    (0x0e90, 4, 0x01807c01),
    (0x0c20, 4, 0x13131313),
    (0x0c24, 4, 0x20202020),
    (0x0c28, 4, 0x1c1e2020),
    (0x0c2c, 4, 0x20202020),
    (0x0c30, 4, 0x1c1e2020),
    (0x0c34, 4, 0x20202020),
    (0x0c38, 4, 0x1a1c1e20),
    (0x0c3c, 4, 0x20202020),
    (0x0c40, 4, 0x1c1e2020),
    (0x0c44, 4, 0x1e1e181a),
    (0x0c48, 4, 0x1e1e1e1e),
    (0x0c4c, 4, 0x16181a1c),
    (0x0c54, 4, 0x00040a12),
    (0x0e20, 4, 0x14141414),
    (0x0e24, 4, 0x1d1d1d1d),
    (0x0e28, 4, 0x191b1d1d),
    (0x0e2c, 4, 0x1f1f1f1f),
    (0x0e30, 4, 0x1b1d1f1f),
    (0x0e34, 4, 0x1f1f1f1f),
    (0x0e38, 4, 0x191b1d1f),
    (0x0e3c, 4, 0x1f1f1f1f),
    (0x0e40, 4, 0x1b1d1f1f),
    (0x0e44, 4, 0x1d1d1719),
    (0x0e48, 4, 0x1d1d1d1d),
    (0x0e4c, 4, 0x1517191b),
    (0x0e54, 4, 0x00030911),
    (0x0454, 1, 0x00000080),
    (0x0808, 4, 0x3e028233),
    (0x0834, 4, 0x0037a706),
    (0x0830, 4, 0x2eaaaeb8),
    (0x0830, 4, 0x2eaaaeb8),
    (0x082c, 4, 0x002083dd),
    (0x0cb0, 4, 0x54337717),
    (0x0eb0, 4, 0x54337717),
    (0x0cb4, 4, 0x01000077),
    (0x0eb4, 4, 0x01000077),
    (0x0900, 4, 0x00000401),
    (0x080c, 4, 0x12131103),
    (0x0a04, 4, 0x0fff000c),
    (0x0c1c, 4, 0x2d400003),
    (0x0e1c, 4, 0x2d400003),
    (0x0860, 4, 0x68a5c321),
    (0x08b0, 4, 0x00000618),
    (0x0c90, 4, 0x01837d01),
    (0x08ac, 4, 0x0ff0fa08),
    (0x08ac, 4, 0x0ff0fa08),
    (0x08c4, 4, 0x00000000),
    (0x08b0, 4, 0x00000618),
    (0x0c90, 4, 0x01817d64),
    (0x08b0, 4, 0x00000618),
    (0x0e90, 4, 0x01837d01),
    (0x08ac, 4, 0x0ff0fa08),
    (0x08ac, 4, 0x0ff0fa08),
    (0x08c4, 4, 0x00000000),
    (0x08b0, 4, 0x00000618),
    (0x0e90, 4, 0x01817d64),
    (0x0668, 2, 0x00001000),
    (0x08ac, 4, 0x0ff0fa08),
    (0x08c4, 4, 0x00000000),
    (0x0848, 4, 0x61d0ff8b),
    (0x08ac, 4, 0x0ff0fa08),
    (0x08ac, 4, 0x0ff0fa08),
    (0x08c4, 4, 0x00000000),
    (0x08b0, 4, 0x00000618),
    (0x0c90, 4, 0x01837d64),
    (0x08b0, 4, 0x00000618),
    (0x0e90, 4, 0x01837d64),
    (0x0c24, 4, 0x27272727),
    (0x0c28, 4, 0x25272727),
    (0x0c2c, 4, 0x2c2c2c2c),
    (0x0c30, 4, 0x26282a2c),
    (0x0c34, 4, 0x28282828),
    (0x0c38, 4, 0x24262828),
    (0x0c3c, 4, 0x2c2c2c2c),
    (0x0c40, 4, 0x26282a2c),
    (0x0c44, 4, 0x28282424),
    (0x0c48, 4, 0x28282828),
    (0x0c4c, 4, 0x22222426),
    (0x0c54, 4, 0x000e141c),
    (0x0e24, 4, 0x21212121),
    (0x0e28, 4, 0x1f212121),
    (0x0e2c, 4, 0x26262626),
    (0x0e30, 4, 0x20222426),
    (0x0e34, 4, 0x22222222),
    (0x0e38, 4, 0x1e202222),
    (0x0e3c, 4, 0x26262626),
    (0x0e40, 4, 0x20222426),
    (0x0e44, 4, 0x22221e1e),
    (0x0e48, 4, 0x22222222),
    (0x0e4c, 4, 0x1c1c1e20),
    (0x0e54, 4, 0x00080e16),
    (0x09a4, 4, 0x000a0080),
    (0x09a4, 4, 0x00080080),
    (0x0a2c, 4, 0x00900000),
    (0x0a2c, 4, 0x00908000),
    (0x0b58, 4, 0x00000001),
    (0x0b58, 4, 0x00000000),
    (0x0c50, 4, 0x00000021),
    (0x0e50, 4, 0x00000021),
    (0x08b0, 4, 0x00000642),
    (0x0c90, 4, 0x04238d10),
    (0x09a4, 4, 0x000a0080),
    (0x09a4, 4, 0x00080080),
    (0x0a2c, 4, 0x00900000),
    (0x0a2c, 4, 0x00908000),
    (0x0b58, 4, 0x00000001),
    (0x0b58, 4, 0x00000000),
    (0x0c50, 4, 0x0000001f),
    (0x0e50, 4, 0x0000001f),
    (0x08b0, 4, 0x00000642),
    (0x0c1c, 4, 0x39000003),
    (0x0e1c, 4, 0x32e00003),
];

/// The RTL8812AU **5 GHz ch116** channel program (UNII-2C DFS) — golden trace
/// (golden/rtw88-8812au-ch116-5g), same capture/decode as ch36.
static CH116_5G_PROGRAM: RegProgram = &[
    (0x09a4, 4, 0x000a0080),
    (0x09a4, 4, 0x00080080),
    (0x0a2c, 4, 0x00900000),
    (0x0a2c, 4, 0x00908000),
    (0x0b58, 4, 0x00000001),
    (0x0b58, 4, 0x00000000),
    (0x0c50, 4, 0x00000020),
    (0x0e50, 4, 0x00000020),
    (0x08b0, 4, 0x00000642),
    (0x0c90, 4, 0x04238d10),
    (0x0860, 4, 0x72d5c321),
    (0x08b0, 4, 0x00000618),
    (0x0c90, 4, 0x01807c01),
    (0x08ac, 4, 0x0ff0fa08),
    (0x08ac, 4, 0x0ff0fa08),
    (0x08c4, 4, 0x00000000),
    (0x08b0, 4, 0x00000618),
    (0x0c90, 4, 0x01807c01),
    (0x08b0, 4, 0x00000618),
    (0x0e90, 4, 0x01807c01),
    (0x08ac, 4, 0x0ff0fa08),
    (0x08ac, 4, 0x0ff0fa08),
    (0x08c4, 4, 0x00000000),
    (0x08b0, 4, 0x00000618),
    (0x0e90, 4, 0x01807c01),
    (0x0668, 2, 0x00001000),
    (0x08ac, 4, 0x0ff0fa08),
    (0x08c4, 4, 0x00000000),
    (0x0848, 4, 0x61d0ff8b),
    (0x08ac, 4, 0x0ff0fa08),
    (0x08ac, 4, 0x0ff0fa08),
    (0x08c4, 4, 0x00000000),
    (0x08b0, 4, 0x00000618),
    (0x0c90, 4, 0x01807c01),
    (0x08b0, 4, 0x00000618),
    (0x0e90, 4, 0x01807c01),
    (0x0c20, 4, 0x13131313),
    (0x0c24, 4, 0x20202020),
    (0x0c28, 4, 0x1c1e2020),
    (0x0c2c, 4, 0x20202020),
    (0x0c30, 4, 0x1c1e2020),
    (0x0c34, 4, 0x20202020),
    (0x0c38, 4, 0x1a1c1e20),
    (0x0c3c, 4, 0x20202020),
    (0x0c40, 4, 0x1c1e2020),
    (0x0c44, 4, 0x1e1e181a),
    (0x0c48, 4, 0x1e1e1e1e),
    (0x0c4c, 4, 0x16181a1c),
    (0x0c54, 4, 0x00040a12),
    (0x0e20, 4, 0x14141414),
    (0x0e24, 4, 0x1d1d1d1d),
    (0x0e28, 4, 0x191b1d1d),
    (0x0e2c, 4, 0x1f1f1f1f),
    (0x0e30, 4, 0x1b1d1f1f),
    (0x0e34, 4, 0x1f1f1f1f),
    (0x0e38, 4, 0x191b1d1f),
    (0x0e3c, 4, 0x1f1f1f1f),
    (0x0e40, 4, 0x1b1d1f1f),
    (0x0e44, 4, 0x1d1d1719),
    (0x0e48, 4, 0x1d1d1d1d),
    (0x0e4c, 4, 0x1517191b),
    (0x0e54, 4, 0x00030911),
    (0x0454, 1, 0x00000080),
    (0x0808, 4, 0x3e028233),
    (0x0834, 4, 0x0037a706),
    (0x0830, 4, 0x2eaaaeb8),
    (0x0830, 4, 0x2eaaaeb8),
    (0x082c, 4, 0x002083dd),
    (0x0cb0, 4, 0x54337717),
    (0x0eb0, 4, 0x54337717),
    (0x0cb4, 4, 0x01000077),
    (0x0eb4, 4, 0x01000077),
    (0x0900, 4, 0x00000401),
    (0x080c, 4, 0x12131103),
    (0x0a04, 4, 0x0fff000c),
    (0x0c1c, 4, 0x2d400003),
    (0x0e1c, 4, 0x2d400003),
    (0x0860, 4, 0x68a5c321),
    (0x08b0, 4, 0x00000618),
    (0x0c90, 4, 0x01837d01),
    (0x08ac, 4, 0x0ff0fa08),
    (0x08ac, 4, 0x0ff0fa08),
    (0x08c4, 4, 0x00000000),
    (0x08b0, 4, 0x00000618),
    (0x0c90, 4, 0x01817d74),
    (0x08b0, 4, 0x00000618),
    (0x0e90, 4, 0x01837d01),
    (0x08ac, 4, 0x0ff0fa08),
    (0x08ac, 4, 0x0ff0fa08),
    (0x08c4, 4, 0x00000000),
    (0x08b0, 4, 0x00000618),
    (0x0e90, 4, 0x01817d74),
    (0x0668, 2, 0x00001000),
    (0x08ac, 4, 0x0ff0fa08),
    (0x08c4, 4, 0x00000000),
    (0x0848, 4, 0x61d0ff8b),
    (0x08ac, 4, 0x0ff0fa08),
    (0x08ac, 4, 0x0ff0fa08),
    (0x08c4, 4, 0x00000000),
    (0x08b0, 4, 0x00000618),
    (0x0c90, 4, 0x01837d74),
    (0x08b0, 4, 0x00000618),
    (0x0e90, 4, 0x01837d74),
    (0x0c24, 4, 0x2c2c2c2c),
    (0x0c28, 4, 0x282a2c2c),
    (0x0c2c, 4, 0x31313131),
    (0x0c30, 4, 0x292b2d2f),
    (0x0c34, 4, 0x2d2d2d2d),
    (0x0c38, 4, 0x27292b2d),
    (0x0c3c, 4, 0x31313131),
    (0x0c40, 4, 0x292b2d2f),
    (0x0c44, 4, 0x2d2d2727),
    (0x0c48, 4, 0x2b2d2d2d),
    (0x0c4c, 4, 0x25252729),
    (0x0c54, 4, 0x0011171f),
    (0x0e24, 4, 0x1e1e1e1e),
    (0x0e28, 4, 0x1a1c1e1e),
    (0x0e2c, 4, 0x23232323),
    (0x0e30, 4, 0x1b1d1f21),
    (0x0e34, 4, 0x1f1f1f1f),
    (0x0e38, 4, 0x191b1d1f),
    (0x0e3c, 4, 0x23232323),
    (0x0e40, 4, 0x1b1d1f21),
    (0x0e44, 4, 0x1f1f1919),
    (0x0e48, 4, 0x1d1f1f1f),
    (0x0e4c, 4, 0x1717191b),
    (0x0e54, 4, 0x00030911),
    (0x09a4, 4, 0x000a0080),
    (0x09a4, 4, 0x00080080),
    (0x0a2c, 4, 0x00900000),
    (0x0a2c, 4, 0x00908000),
    (0x0b58, 4, 0x00000001),
    (0x0b58, 4, 0x00000000),
    (0x08b0, 4, 0x00000642),
    (0x0c90, 4, 0x04238d10),
];

/// The RTL8812AU **5 GHz ch132** channel program (UNII-2C DFS) — golden trace
/// (golden/rtw88-8812au-ch132-5g), same capture/decode as ch36.
static CH132_5G_PROGRAM: RegProgram = &[
    (0x0860, 4, 0x72d5c321),
    (0x08b0, 4, 0x00000618),
    (0x0c90, 4, 0x01807c01),
    (0x08ac, 4, 0x0ff0fa08),
    (0x08ac, 4, 0x0ff0fa08),
    (0x08c4, 4, 0x00000000),
    (0x08b0, 4, 0x00000618),
    (0x0c90, 4, 0x01807c01),
    (0x08b0, 4, 0x00000618),
    (0x0e90, 4, 0x01807c01),
    (0x08ac, 4, 0x0ff0fa08),
    (0x08ac, 4, 0x0ff0fa08),
    (0x08c4, 4, 0x00000000),
    (0x08b0, 4, 0x00000618),
    (0x0e90, 4, 0x01807c01),
    (0x0668, 2, 0x00001000),
    (0x08ac, 4, 0x0ff0fa08),
    (0x08c4, 4, 0x00000000),
    (0x0848, 4, 0x61d0ff8b),
    (0x08ac, 4, 0x0ff0fa08),
    (0x08ac, 4, 0x0ff0fa08),
    (0x08c4, 4, 0x00000000),
    (0x08b0, 4, 0x00000618),
    (0x0c90, 4, 0x01807c01),
    (0x08b0, 4, 0x00000618),
    (0x0e90, 4, 0x01807c01),
    (0x0c20, 4, 0x13131313),
    (0x0c24, 4, 0x20202020),
    (0x0c28, 4, 0x1c1e2020),
    (0x0c2c, 4, 0x20202020),
    (0x0c30, 4, 0x1c1e2020),
    (0x0c34, 4, 0x20202020),
    (0x0c38, 4, 0x1a1c1e20),
    (0x0c3c, 4, 0x20202020),
    (0x0c40, 4, 0x1c1e2020),
    (0x0c44, 4, 0x1e1e181a),
    (0x0c48, 4, 0x1e1e1e1e),
    (0x0c4c, 4, 0x16181a1c),
    (0x0c54, 4, 0x00040a12),
    (0x0e20, 4, 0x14141414),
    (0x0e24, 4, 0x1d1d1d1d),
    (0x0e28, 4, 0x191b1d1d),
    (0x0e2c, 4, 0x1f1f1f1f),
    (0x0e30, 4, 0x1b1d1f1f),
    (0x0e34, 4, 0x1f1f1f1f),
    (0x0e38, 4, 0x191b1d1f),
    (0x0e3c, 4, 0x1f1f1f1f),
    (0x0e40, 4, 0x1b1d1f1f),
    (0x0e44, 4, 0x1d1d1719),
    (0x0e48, 4, 0x1d1d1d1d),
    (0x0e4c, 4, 0x1517191b),
    (0x0e54, 4, 0x00030911),
    (0x0454, 1, 0x00000080),
    (0x0808, 4, 0x3e028233),
    (0x0834, 4, 0x0037a706),
    (0x0830, 4, 0x2eaaaeb8),
    (0x0830, 4, 0x2eaaaeb8),
    (0x082c, 4, 0x002083dd),
    (0x0cb0, 4, 0x54337717),
    (0x0eb0, 4, 0x54337717),
    (0x0cb4, 4, 0x01000077),
    (0x0eb4, 4, 0x01000077),
    (0x0900, 4, 0x00000401),
    (0x080c, 4, 0x12131103),
    (0x0a04, 4, 0x0fff000c),
    (0x0c1c, 4, 0x2d400003),
    (0x0e1c, 4, 0x2d400003),
    (0x0860, 4, 0x6825c321),
    (0x08b0, 4, 0x00000618),
    (0x0c90, 4, 0x01837d01),
    (0x08ac, 4, 0x0ff0fa08),
    (0x08ac, 4, 0x0ff0fa08),
    (0x08c4, 4, 0x00000000),
    (0x08b0, 4, 0x00000618),
    (0x0c90, 4, 0x01817d84),
    (0x08b0, 4, 0x00000618),
    (0x0e90, 4, 0x01837d01),
    (0x08ac, 4, 0x0ff0fa08),
    (0x08ac, 4, 0x0ff0fa08),
    (0x08c4, 4, 0x00000000),
    (0x08b0, 4, 0x00000618),
    (0x0e90, 4, 0x01817d84),
    (0x0668, 2, 0x00001000),
    (0x08ac, 4, 0x0ff0fa08),
    (0x08c4, 4, 0x00000000),
    (0x0848, 4, 0x61d0ff8b),
    (0x08ac, 4, 0x0ff0fa08),
    (0x08ac, 4, 0x0ff0fa08),
    (0x08c4, 4, 0x00000000),
    (0x08b0, 4, 0x00000618),
    (0x0c90, 4, 0x01837d84),
    (0x08b0, 4, 0x00000618),
    (0x0e90, 4, 0x01837d84),
    (0x0c24, 4, 0x2c2c2c2c),
    (0x0c28, 4, 0x2a2c2c2c),
    (0x0c2c, 4, 0x31313131),
    (0x0c30, 4, 0x2b2d2f31),
    (0x0c34, 4, 0x2d2d2d2d),
    (0x0c38, 4, 0x292b2d2d),
    (0x0c3c, 4, 0x31313131),
    (0x0c40, 4, 0x2b2d2f31),
    (0x0c44, 4, 0x2d2d2929),
    (0x0c48, 4, 0x2d2d2d2d),
    (0x0c4c, 4, 0x2727292b),
    (0x0c54, 4, 0x00131921),
    (0x0e24, 4, 0x19191919),
    (0x0e28, 4, 0x17191919),
    (0x0e2c, 4, 0x1e1e1e1e),
    (0x0e30, 4, 0x181a1c1e),
    (0x0e34, 4, 0x1a1a1a1a),
    (0x0e38, 4, 0x16181a1a),
    (0x0e3c, 4, 0x1e1e1e1e),
    (0x0e40, 4, 0x181a1c1e),
    (0x0e44, 4, 0x1a1a1616),
    (0x0e48, 4, 0x1a1a1a1a),
    (0x0e4c, 4, 0x14141618),
    (0x0e54, 4, 0x0002060e),
    (0x09a4, 4, 0x000a0080),
    (0x09a4, 4, 0x00080080),
    (0x0a2c, 4, 0x00900000),
    (0x0a2c, 4, 0x00908000),
    (0x0b58, 4, 0x00000001),
    (0x0b58, 4, 0x00000000),
    (0x0c50, 4, 0x00000020),
    (0x0e50, 4, 0x00000020),
    (0x08b0, 4, 0x00000642),
    (0x0c90, 4, 0x04238d10),
    (0x09a4, 4, 0x000a0080),
    (0x09a4, 4, 0x00080080),
    (0x0a2c, 4, 0x00900000),
    (0x0a2c, 4, 0x00908000),
    (0x0b58, 4, 0x00000001),
    (0x0b58, 4, 0x00000000),
    (0x0c50, 4, 0x0000001e),
    (0x0e50, 4, 0x0000001e),
    (0x08b0, 4, 0x00000642),
    (0x0c1c, 4, 0x39000003),
    (0x0e1c, 4, 0x32e00003),
];

/// The RTL8812AU **5 GHz ch140** channel program (UNII-2C DFS) — golden trace
/// (golden/rtw88-8812au-ch140-5g), same capture/decode as ch36.
static CH140_5G_PROGRAM: RegProgram = &[
    (0x0860, 4, 0x72d5c321),
    (0x08b0, 4, 0x00000618),
    (0x0c90, 4, 0x01807c01),
    (0x08ac, 4, 0x0ff0fa08),
    (0x08ac, 4, 0x0ff0fa08),
    (0x08c4, 4, 0x00000000),
    (0x08b0, 4, 0x00000618),
    (0x0c90, 4, 0x01807c01),
    (0x08b0, 4, 0x00000618),
    (0x0e90, 4, 0x01807c01),
    (0x08ac, 4, 0x0ff0fa08),
    (0x08ac, 4, 0x0ff0fa08),
    (0x08c4, 4, 0x00000000),
    (0x08b0, 4, 0x00000618),
    (0x0e90, 4, 0x01807c01),
    (0x0668, 2, 0x00001000),
    (0x08ac, 4, 0x0ff0fa08),
    (0x08c4, 4, 0x00000000),
    (0x0848, 4, 0x61d0ff8b),
    (0x08ac, 4, 0x0ff0fa08),
    (0x08ac, 4, 0x0ff0fa08),
    (0x08c4, 4, 0x00000000),
    (0x08b0, 4, 0x00000618),
    (0x0c90, 4, 0x01807c01),
    (0x08b0, 4, 0x00000618),
    (0x0e90, 4, 0x01807c01),
    (0x0c20, 4, 0x13131313),
    (0x0c24, 4, 0x20202020),
    (0x0c28, 4, 0x1c1e2020),
    (0x0c2c, 4, 0x20202020),
    (0x0c30, 4, 0x1c1e2020),
    (0x0c34, 4, 0x20202020),
    (0x0c38, 4, 0x1a1c1e20),
    (0x0c3c, 4, 0x20202020),
    (0x0c40, 4, 0x1c1e2020),
    (0x0c44, 4, 0x1e1e181a),
    (0x0c48, 4, 0x1e1e1e1e),
    (0x0c4c, 4, 0x16181a1c),
    (0x0c54, 4, 0x00040a12),
    (0x0e20, 4, 0x14141414),
    (0x0e24, 4, 0x1d1d1d1d),
    (0x0e28, 4, 0x191b1d1d),
    (0x0e2c, 4, 0x1f1f1f1f),
    (0x0e30, 4, 0x1b1d1f1f),
    (0x0e34, 4, 0x1f1f1f1f),
    (0x0e38, 4, 0x191b1d1f),
    (0x0e3c, 4, 0x1f1f1f1f),
    (0x0e40, 4, 0x1b1d1f1f),
    (0x0e44, 4, 0x1d1d1719),
    (0x0e48, 4, 0x1d1d1d1d),
    (0x0e4c, 4, 0x1517191b),
    (0x0e54, 4, 0x00030911),
    (0x0454, 1, 0x00000080),
    (0x0808, 4, 0x3e028233),
    (0x0834, 4, 0x0037a706),
    (0x0830, 4, 0x2eaaaeb8),
    (0x0830, 4, 0x2eaaaeb8),
    (0x082c, 4, 0x002083dd),
    (0x0cb0, 4, 0x54337717),
    (0x0eb0, 4, 0x54337717),
    (0x0cb4, 4, 0x01000077),
    (0x0eb4, 4, 0x01000077),
    (0x0900, 4, 0x00000401),
    (0x080c, 4, 0x12131103),
    (0x0a04, 4, 0x0fff000c),
    (0x0c1c, 4, 0x2d400003),
    (0x0e1c, 4, 0x2d400003),
    (0x0860, 4, 0x6825c321),
    (0x08b0, 4, 0x00000618),
    (0x0c90, 4, 0x01837d01),
    (0x08ac, 4, 0x0ff0fa08),
    (0x08ac, 4, 0x0ff0fa08),
    (0x08c4, 4, 0x00000000),
    (0x08b0, 4, 0x00000618),
    (0x0c90, 4, 0x01817d8c),
    (0x08b0, 4, 0x00000618),
    (0x0e90, 4, 0x01837d01),
    (0x08ac, 4, 0x0ff0fa08),
    (0x08ac, 4, 0x0ff0fa08),
    (0x08c4, 4, 0x00000000),
    (0x08b0, 4, 0x00000618),
    (0x0e90, 4, 0x01817d8c),
    (0x0668, 2, 0x00001000),
    (0x08ac, 4, 0x0ff0fa08),
    (0x08c4, 4, 0x00000000),
    (0x0848, 4, 0x61d0ff8b),
    (0x08ac, 4, 0x0ff0fa08),
    (0x08ac, 4, 0x0ff0fa08),
    (0x08c4, 4, 0x00000000),
    (0x08b0, 4, 0x00000618),
    (0x0c90, 4, 0x01837d8c),
    (0x08b0, 4, 0x00000618),
    (0x0e90, 4, 0x01837d8c),
    (0x0c24, 4, 0x2b2b2b2b),
    (0x0c28, 4, 0x2b2b2b2b),
    (0x0c2c, 4, 0x30303030),
    (0x0c30, 4, 0x2c2e3030),
    (0x0c34, 4, 0x2c2c2c2c),
    (0x0c38, 4, 0x2a2c2c2c),
    (0x0c3c, 4, 0x30303030),
    (0x0c40, 4, 0x2c2e3030),
    (0x0c44, 4, 0x2c2c2a2a),
    (0x0c48, 4, 0x2c2c2c2c),
    (0x0c4c, 4, 0x28282a2c),
    (0x0c54, 4, 0x00141a22),
    (0x0e24, 4, 0x17171717),
    (0x0e28, 4, 0x17171717),
    (0x0e2c, 4, 0x1c1c1c1c),
    (0x0e30, 4, 0x181a1c1c),
    (0x0e34, 4, 0x18181818),
    (0x0e38, 4, 0x16181818),
    (0x0e3c, 4, 0x1c1c1c1c),
    (0x0e40, 4, 0x181a1c1c),
    (0x0e44, 4, 0x18181616),
    (0x0e48, 4, 0x18181818),
    (0x0e4c, 4, 0x14141618),
    (0x0e54, 4, 0x0002060e),
    (0x09a4, 4, 0x000a0080),
    (0x09a4, 4, 0x00080080),
    (0x0a2c, 4, 0x00900000),
    (0x0a2c, 4, 0x00908000),
    (0x0b58, 4, 0x00000001),
    (0x0b58, 4, 0x00000000),
    (0x08b0, 4, 0x00000642),
    (0x0c90, 4, 0x04238d10),
    (0x09a4, 4, 0x000a0080),
    (0x09a4, 4, 0x00080080),
    (0x0a2c, 4, 0x00900000),
    (0x0a2c, 4, 0x00908000),
    (0x0b58, 4, 0x00000001),
    (0x0b58, 4, 0x00000000),
    (0x0c50, 4, 0x0000001c),
    (0x0e50, 4, 0x0000001c),
    (0x08b0, 4, 0x00000642),
    (0x0c1c, 4, 0x39000003),
    (0x0e1c, 4, 0x32e00003),
];

/// The RTL8812AU **5 GHz ch153** channel program (UNII-3) — golden trace
/// (golden/rtw88-8812au-ch153-5g), same capture/decode as ch36.
static CH153_5G_PROGRAM: RegProgram = &[
    (0x09a4, 4, 0x000a0080),
    (0x09a4, 4, 0x00080080),
    (0x0a2c, 4, 0x00900000),
    (0x0a2c, 4, 0x00908000),
    (0x0b58, 4, 0x00000001),
    (0x0b58, 4, 0x00000000),
    (0x0c50, 4, 0x0000001e),
    (0x0e50, 4, 0x0000001e),
    (0x08b0, 4, 0x00000642),
    (0x0c90, 4, 0x04238d10),
    (0x0860, 4, 0x72d5c321),
    (0x08b0, 4, 0x00000618),
    (0x0c90, 4, 0x01807c01),
    (0x08ac, 4, 0x0ff0fa08),
    (0x08ac, 4, 0x0ff0fa08),
    (0x08c4, 4, 0x00000000),
    (0x08b0, 4, 0x00000618),
    (0x0c90, 4, 0x01807c01),
    (0x08b0, 4, 0x00000618),
    (0x0e90, 4, 0x01807c01),
    (0x08ac, 4, 0x0ff0fa08),
    (0x08ac, 4, 0x0ff0fa08),
    (0x08c4, 4, 0x00000000),
    (0x08b0, 4, 0x00000618),
    (0x0e90, 4, 0x01807c01),
    (0x0668, 2, 0x00001000),
    (0x08ac, 4, 0x0ff0fa08),
    (0x08c4, 4, 0x00000000),
    (0x0848, 4, 0x61d0ff8b),
    (0x08ac, 4, 0x0ff0fa08),
    (0x08ac, 4, 0x0ff0fa08),
    (0x08c4, 4, 0x00000000),
    (0x08b0, 4, 0x00000618),
    (0x0c90, 4, 0x01807c01),
    (0x08b0, 4, 0x00000618),
    (0x0e90, 4, 0x01807c01),
    (0x0c20, 4, 0x13131313),
    (0x0c24, 4, 0x20202020),
    (0x0c28, 4, 0x1c1e2020),
    (0x0c2c, 4, 0x20202020),
    (0x0c30, 4, 0x1c1e2020),
    (0x0c34, 4, 0x20202020),
    (0x0c38, 4, 0x1a1c1e20),
    (0x0c3c, 4, 0x20202020),
    (0x0c40, 4, 0x1c1e2020),
    (0x0c44, 4, 0x1e1e181a),
    (0x0c48, 4, 0x1e1e1e1e),
    (0x0c4c, 4, 0x16181a1c),
    (0x0c54, 4, 0x00040a12),
    (0x0e20, 4, 0x14141414),
    (0x0e24, 4, 0x1d1d1d1d),
    (0x0e28, 4, 0x191b1d1d),
    (0x0e2c, 4, 0x1f1f1f1f),
    (0x0e30, 4, 0x1b1d1f1f),
    (0x0e34, 4, 0x1f1f1f1f),
    (0x0e38, 4, 0x191b1d1f),
    (0x0e3c, 4, 0x1f1f1f1f),
    (0x0e40, 4, 0x1b1d1f1f),
    (0x0e44, 4, 0x1d1d1719),
    (0x0e48, 4, 0x1d1d1d1d),
    (0x0e4c, 4, 0x1517191b),
    (0x0e54, 4, 0x00030911),
    (0x0454, 1, 0x00000080),
    (0x0808, 4, 0x3e028233),
    (0x0834, 4, 0x0037a706),
    (0x0830, 4, 0x2eaaaeb8),
    (0x0830, 4, 0x2eaaaeb8),
    (0x082c, 4, 0x002083dd),
    (0x0cb0, 4, 0x54337717),
    (0x0eb0, 4, 0x54337717),
    (0x0cb4, 4, 0x01000077),
    (0x0eb4, 4, 0x01000077),
    (0x0900, 4, 0x00000401),
    (0x080c, 4, 0x12131103),
    (0x0a04, 4, 0x0fff000c),
    (0x0c1c, 4, 0x2d400003),
    (0x0e1c, 4, 0x2d400003),
    (0x0860, 4, 0x6825c321),
    (0x08b0, 4, 0x00000618),
    (0x0c90, 4, 0x01857d01),
    (0x08ac, 4, 0x0ff0fa08),
    (0x08ac, 4, 0x0ff0fa08),
    (0x08c4, 4, 0x00000000),
    (0x08b0, 4, 0x00000618),
    (0x0c90, 4, 0x01817d99),
    (0x08b0, 4, 0x00000618),
    (0x0e90, 4, 0x01857d01),
    (0x08ac, 4, 0x0ff0fa08),
    (0x08ac, 4, 0x0ff0fa08),
    (0x08c4, 4, 0x00000000),
    (0x08b0, 4, 0x00000618),
    (0x0e90, 4, 0x01817d99),
    (0x0668, 2, 0x00001000),
    (0x08ac, 4, 0x0ff0fa08),
    (0x08c4, 4, 0x00000000),
    (0x0848, 4, 0x61d0ff8b),
    (0x08ac, 4, 0x0ff0fa08),
    (0x08ac, 4, 0x0ff0fa08),
    (0x08c4, 4, 0x00000000),
    (0x08b0, 4, 0x00000618),
    (0x0c90, 4, 0x01857d99),
    (0x08b0, 4, 0x00000618),
    (0x0e90, 4, 0x01857d99),
    (0x0c24, 4, 0x30303030),
    (0x0c28, 4, 0x2c2e3030),
    (0x0c2c, 4, 0x35353535),
    (0x0c30, 4, 0x2d2f3133),
    (0x0c34, 4, 0x31313131),
    (0x0c38, 4, 0x2b2d2f31),
    (0x0c3c, 4, 0x35353535),
    (0x0c40, 4, 0x2d2f3133),
    (0x0c44, 4, 0x31312b2b),
    (0x0c48, 4, 0x2f313131),
    (0x0c4c, 4, 0x29292b2d),
    (0x0c54, 4, 0x00151b23),
    (0x0e24, 4, 0x1e1e1e1e),
    (0x0e28, 4, 0x1a1c1e1e),
    (0x0e2c, 4, 0x23232323),
    (0x0e30, 4, 0x1b1d1f21),
    (0x0e34, 4, 0x1f1f1f1f),
    (0x0e38, 4, 0x191b1d1f),
    (0x0e3c, 4, 0x23232323),
    (0x0e40, 4, 0x1b1d1f21),
    (0x0e44, 4, 0x1f1f1919),
    (0x0e48, 4, 0x1d1f1f1f),
    (0x0e4c, 4, 0x1717191b),
    (0x0e54, 4, 0x00030911),
    (0x09a4, 4, 0x000a0080),
    (0x09a4, 4, 0x00080080),
    (0x0a2c, 4, 0x00900000),
    (0x0a2c, 4, 0x00908000),
    (0x0b58, 4, 0x00000001),
    (0x0b58, 4, 0x00000000),
    (0x0c50, 4, 0x0000001c),
    (0x0e50, 4, 0x0000001c),
    (0x08b0, 4, 0x00000642),
    (0x0c90, 4, 0x04238d10),
];

/// The RTL8812AU **5 GHz ch161** channel program (UNII-3) — golden trace
/// (golden/rtw88-8812au-ch161-5g), same capture/decode as ch36.
static CH161_5G_PROGRAM: RegProgram = &[
    (0x0860, 4, 0x72d5c321),
    (0x08b0, 4, 0x00000618),
    (0x0c90, 4, 0x01807c01),
    (0x08ac, 4, 0x0ff0fa08),
    (0x08ac, 4, 0x0ff0fa08),
    (0x08c4, 4, 0x00000000),
    (0x08b0, 4, 0x00000618),
    (0x0c90, 4, 0x01807c01),
    (0x08b0, 4, 0x00000618),
    (0x0e90, 4, 0x01807c01),
    (0x08ac, 4, 0x0ff0fa08),
    (0x08ac, 4, 0x0ff0fa08),
    (0x08c4, 4, 0x00000000),
    (0x08b0, 4, 0x00000618),
    (0x0e90, 4, 0x01807c01),
    (0x0668, 2, 0x00001000),
    (0x08ac, 4, 0x0ff0fa08),
    (0x08c4, 4, 0x00000000),
    (0x0848, 4, 0x61d0ff8b),
    (0x08ac, 4, 0x0ff0fa08),
    (0x08ac, 4, 0x0ff0fa08),
    (0x08c4, 4, 0x00000000),
    (0x08b0, 4, 0x00000618),
    (0x0c90, 4, 0x01807c01),
    (0x08b0, 4, 0x00000618),
    (0x0e90, 4, 0x01807c01),
    (0x0c20, 4, 0x13131313),
    (0x0c24, 4, 0x20202020),
    (0x0c28, 4, 0x1c1e2020),
    (0x0c2c, 4, 0x20202020),
    (0x0c30, 4, 0x1c1e2020),
    (0x0c34, 4, 0x20202020),
    (0x0c38, 4, 0x1a1c1e20),
    (0x0c3c, 4, 0x20202020),
    (0x0c40, 4, 0x1c1e2020),
    (0x0c44, 4, 0x1e1e181a),
    (0x0c48, 4, 0x1e1e1e1e),
    (0x0c4c, 4, 0x16181a1c),
    (0x0c54, 4, 0x00040a12),
    (0x0e20, 4, 0x14141414),
    (0x0e24, 4, 0x1d1d1d1d),
    (0x0e28, 4, 0x191b1d1d),
    (0x0e2c, 4, 0x1f1f1f1f),
    (0x0e30, 4, 0x1b1d1f1f),
    (0x0e34, 4, 0x1f1f1f1f),
    (0x0e38, 4, 0x191b1d1f),
    (0x0e3c, 4, 0x1f1f1f1f),
    (0x0e40, 4, 0x1b1d1f1f),
    (0x0e44, 4, 0x1d1d1719),
    (0x0e48, 4, 0x1d1d1d1d),
    (0x0e4c, 4, 0x1517191b),
    (0x0e54, 4, 0x00030911),
    (0x0454, 1, 0x00000080),
    (0x0808, 4, 0x3e028233),
    (0x0834, 4, 0x0037a706),
    (0x0830, 4, 0x2eaaaeb8),
    (0x0830, 4, 0x2eaaaeb8),
    (0x082c, 4, 0x002083dd),
    (0x0cb0, 4, 0x54337717),
    (0x0eb0, 4, 0x54337717),
    (0x0cb4, 4, 0x01000077),
    (0x0eb4, 4, 0x01000077),
    (0x0900, 4, 0x00000401),
    (0x080c, 4, 0x12131103),
    (0x0a04, 4, 0x0fff000c),
    (0x0c1c, 4, 0x2d400003),
    (0x0e1c, 4, 0x2d400003),
    (0x0860, 4, 0x6825c321),
    (0x08b0, 4, 0x00000618),
    (0x0c90, 4, 0x01857d01),
    (0x08ac, 4, 0x0ff0fa08),
    (0x08ac, 4, 0x0ff0fa08),
    (0x08c4, 4, 0x00000000),
    (0x08b0, 4, 0x00000618),
    (0x0c90, 4, 0x01817da1),
    (0x08b0, 4, 0x00000618),
    (0x0e90, 4, 0x01857d01),
    (0x08ac, 4, 0x0ff0fa08),
    (0x08ac, 4, 0x0ff0fa08),
    (0x08c4, 4, 0x00000000),
    (0x08b0, 4, 0x00000618),
    (0x0e90, 4, 0x01817da1),
    (0x0668, 2, 0x00001000),
    (0x08ac, 4, 0x0ff0fa08),
    (0x08c4, 4, 0x00000000),
    (0x0848, 4, 0x61d0ff8b),
    (0x08ac, 4, 0x0ff0fa08),
    (0x08ac, 4, 0x0ff0fa08),
    (0x08c4, 4, 0x00000000),
    (0x08b0, 4, 0x00000618),
    (0x0c90, 4, 0x01857da1),
    (0x08b0, 4, 0x00000618),
    (0x0e90, 4, 0x01857da1),
    (0x0c24, 4, 0x30303030),
    (0x0c28, 4, 0x2c2e3030),
    (0x0c2c, 4, 0x35353535),
    (0x0c30, 4, 0x2d2f3133),
    (0x0c34, 4, 0x31313131),
    (0x0c38, 4, 0x2b2d2f31),
    (0x0c3c, 4, 0x35353535),
    (0x0c40, 4, 0x2d2f3133),
    (0x0c44, 4, 0x31312b2b),
    (0x0c48, 4, 0x2f313131),
    (0x0c4c, 4, 0x29292b2d),
    (0x0c54, 4, 0x00151b23),
    (0x0e24, 4, 0x20202020),
    (0x0e28, 4, 0x1c1e2020),
    (0x0e2c, 4, 0x25252525),
    (0x0e30, 4, 0x1d1f2123),
    (0x0e34, 4, 0x21212121),
    (0x0e38, 4, 0x1b1d1f21),
    (0x0e3c, 4, 0x25252525),
    (0x0e40, 4, 0x1d1f2123),
    (0x0e44, 4, 0x21211b1b),
    (0x0e48, 4, 0x1f212121),
    (0x0e4c, 4, 0x19191b1d),
    (0x0e54, 4, 0x00050b13),
    (0x09a4, 4, 0x000a0080),
    (0x09a4, 4, 0x00080080),
    (0x0a2c, 4, 0x00900000),
    (0x0a2c, 4, 0x00908000),
    (0x0b58, 4, 0x00000001),
    (0x0b58, 4, 0x00000000),
    (0x0c50, 4, 0x0000001e),
    (0x0e50, 4, 0x0000001e),
    (0x08b0, 4, 0x00000642),
    (0x0c90, 4, 0x04238d10),
    (0x09a4, 4, 0x000a0080),
    (0x09a4, 4, 0x00080080),
    (0x0a2c, 4, 0x00900000),
    (0x0a2c, 4, 0x00908000),
    (0x0b58, 4, 0x00000001),
    (0x0b58, 4, 0x00000000),
    (0x0c50, 4, 0x00000020),
    (0x0e50, 4, 0x00000020),
    (0x08b0, 4, 0x00000642),
    (0x0c1c, 4, 0x39000003),
    (0x0e1c, 4, 0x32e00003),
];

/// The RTL8812AU **5 GHz ch165** channel program (UNII-3) — golden trace
/// (golden/rtw88-8812au-ch165-5g), same capture/decode as ch36.
static CH165_5G_PROGRAM: RegProgram = &[
    (0x09a4, 4, 0x000a0080),
    (0x09a4, 4, 0x00080080),
    (0x0a2c, 4, 0x00900000),
    (0x0a2c, 4, 0x00908000),
    (0x0b58, 4, 0x00000001),
    (0x0b58, 4, 0x00000000),
    (0x0c50, 4, 0x00000022),
    (0x0e50, 4, 0x00000022),
    (0x08b0, 4, 0x00000642),
    (0x0c90, 4, 0x04238d10),
    (0x0860, 4, 0x72d5c321),
    (0x08b0, 4, 0x00000618),
    (0x0c90, 4, 0x01807c01),
    (0x08ac, 4, 0x0ff0fa08),
    (0x08ac, 4, 0x0ff0fa08),
    (0x08c4, 4, 0x00000000),
    (0x08b0, 4, 0x00000618),
    (0x0c90, 4, 0x01807c01),
    (0x08b0, 4, 0x00000618),
    (0x0e90, 4, 0x01807c01),
    (0x08ac, 4, 0x0ff0fa08),
    (0x08ac, 4, 0x0ff0fa08),
    (0x08c4, 4, 0x00000000),
    (0x08b0, 4, 0x00000618),
    (0x0e90, 4, 0x01807c01),
    (0x0668, 2, 0x00001000),
    (0x08ac, 4, 0x0ff0fa08),
    (0x08c4, 4, 0x00000000),
    (0x0848, 4, 0x61d0ff8b),
    (0x08ac, 4, 0x0ff0fa08),
    (0x08ac, 4, 0x0ff0fa08),
    (0x08c4, 4, 0x00000000),
    (0x08b0, 4, 0x00000618),
    (0x0c90, 4, 0x01807c01),
    (0x08b0, 4, 0x00000618),
    (0x0e90, 4, 0x01807c01),
    (0x0c20, 4, 0x13131313),
    (0x0c24, 4, 0x20202020),
    (0x0c28, 4, 0x1c1e2020),
    (0x0c2c, 4, 0x20202020),
    (0x0c30, 4, 0x1c1e2020),
    (0x0c34, 4, 0x20202020),
    (0x0c38, 4, 0x1a1c1e20),
    (0x0c3c, 4, 0x20202020),
    (0x0c40, 4, 0x1c1e2020),
    (0x0c44, 4, 0x1e1e181a),
    (0x0c48, 4, 0x1e1e1e1e),
    (0x0c4c, 4, 0x16181a1c),
    (0x0c54, 4, 0x00040a12),
    (0x0e20, 4, 0x14141414),
    (0x0e24, 4, 0x1d1d1d1d),
    (0x0e28, 4, 0x191b1d1d),
    (0x0e2c, 4, 0x1f1f1f1f),
    (0x0e30, 4, 0x1b1d1f1f),
    (0x0e34, 4, 0x1f1f1f1f),
    (0x0e38, 4, 0x191b1d1f),
    (0x0e3c, 4, 0x1f1f1f1f),
    (0x0e40, 4, 0x1b1d1f1f),
    (0x0e44, 4, 0x1d1d1719),
    (0x0e48, 4, 0x1d1d1d1d),
    (0x0e4c, 4, 0x1517191b),
    (0x0e54, 4, 0x00030911),
    (0x0454, 1, 0x00000080),
    (0x0808, 4, 0x3e028233),
    (0x0834, 4, 0x0037a706),
    (0x0830, 4, 0x2eaaaeb8),
    (0x0830, 4, 0x2eaaaeb8),
    (0x082c, 4, 0x002083dd),
    (0x0cb0, 4, 0x54337717),
    (0x0eb0, 4, 0x54337717),
    (0x0cb4, 4, 0x01000077),
    (0x0eb4, 4, 0x01000077),
    (0x0900, 4, 0x00000401),
    (0x080c, 4, 0x12131103),
    (0x0a04, 4, 0x0fff000c),
    (0x0c1c, 4, 0x2d400003),
    (0x0e1c, 4, 0x2d400003),
    (0x0860, 4, 0x6825c321),
    (0x08b0, 4, 0x00000618),
    (0x0c90, 4, 0x01857d01),
    (0x08ac, 4, 0x0ff0fa08),
    (0x08ac, 4, 0x0ff0fa08),
    (0x08c4, 4, 0x00000000),
    (0x08b0, 4, 0x00000618),
    (0x0c90, 4, 0x01817da5),
    (0x08b0, 4, 0x00000618),
    (0x0e90, 4, 0x01857d01),
    (0x08ac, 4, 0x0ff0fa08),
    (0x08ac, 4, 0x0ff0fa08),
    (0x08c4, 4, 0x00000000),
    (0x08b0, 4, 0x00000618),
    (0x0e90, 4, 0x01817da5),
    (0x0668, 2, 0x00001000),
    (0x08ac, 4, 0x0ff0fa08),
    (0x08c4, 4, 0x00000000),
    (0x0848, 4, 0x61d0ff8b),
    (0x08ac, 4, 0x0ff0fa08),
    (0x08ac, 4, 0x0ff0fa08),
    (0x08c4, 4, 0x00000000),
    (0x08b0, 4, 0x00000618),
    (0x0c90, 4, 0x01857da5),
    (0x08b0, 4, 0x00000618),
    (0x0e90, 4, 0x01857da5),
    (0x0c24, 4, 0x31313131),
    (0x0c28, 4, 0x2d2f3131),
    (0x0c2c, 4, 0x36363636),
    (0x0c30, 4, 0x2e303234),
    (0x0c34, 4, 0x32323232),
    (0x0c38, 4, 0x2c2e3032),
    (0x0c3c, 4, 0x36363636),
    (0x0c40, 4, 0x2e303234),
    (0x0c44, 4, 0x32322c2c),
    (0x0c48, 4, 0x30323232),
    (0x0c4c, 4, 0x2a2a2c2e),
    (0x0c54, 4, 0x00161c24),
    (0x0e24, 4, 0x21212121),
    (0x0e28, 4, 0x1d1f2121),
    (0x0e2c, 4, 0x26262626),
    (0x0e30, 4, 0x1e202224),
    (0x0e34, 4, 0x22222222),
    (0x0e38, 4, 0x1c1e2022),
    (0x0e3c, 4, 0x26262626),
    (0x0e40, 4, 0x1e202224),
    (0x0e44, 4, 0x22221c1c),
    (0x0e48, 4, 0x20222222),
    (0x0e4c, 4, 0x1a1a1c1e),
    (0x0e54, 4, 0x00060c14),
    (0x09a4, 4, 0x000a0080),
    (0x09a4, 4, 0x00080080),
    (0x0a2c, 4, 0x00900000),
    (0x0a2c, 4, 0x00908000),
    (0x0b58, 4, 0x00000001),
    (0x0b58, 4, 0x00000000),
    (0x0c50, 4, 0x00000020),
    (0x0e50, 4, 0x00000020),
    (0x08b0, 4, 0x00000642),
    (0x0c90, 4, 0x04238d10),
];

/// The RTL8812AU **5 GHz ch52** channel program (UNII-2A DFS) — golden trace
/// (golden/rtw88-8812au-ch52-5g), same capture/decode as ch36.
static CH52_5G_PROGRAM: RegProgram = &[
    (0x0860, 4, 0x72d5c321),
    (0x08b0, 4, 0x00000618),
    (0x0c90, 4, 0x0180702a),
    (0x08ac, 4, 0x0ff0fa0a),
    (0x08ac, 4, 0x0ff0fa0a),
    (0x08c4, 4, 0x00000000),
    (0x08b0, 4, 0x00000618),
    (0x0c90, 4, 0x01807001),
    (0x08b0, 4, 0x00000618),
    (0x0e90, 4, 0x0180702a),
    (0x08ac, 4, 0x0ff0fa0a),
    (0x08ac, 4, 0x0ff0fa0a),
    (0x08c4, 4, 0x00000000),
    (0x08b0, 4, 0x00000618),
    (0x0e90, 4, 0x01807001),
    (0x0668, 2, 0x00001000),
    (0x08ac, 4, 0x0ff0fa08),
    (0x08c4, 4, 0x00000000),
    (0x0848, 4, 0x61d0ff8b),
    (0x08ac, 4, 0x0ff0fa08),
    (0x08ac, 4, 0x0ff0fa08),
    (0x08c4, 4, 0x00000000),
    (0x08b0, 4, 0x00000618),
    (0x0c90, 4, 0x01807c01),
    (0x08b0, 4, 0x00000618),
    (0x0e90, 4, 0x01807c01),
    (0x0c20, 4, 0x13131313),
    (0x0c24, 4, 0x20202020),
    (0x0c28, 4, 0x1c1e2020),
    (0x0c2c, 4, 0x20202020),
    (0x0c30, 4, 0x1c1e2020),
    (0x0c34, 4, 0x20202020),
    (0x0c38, 4, 0x1a1c1e20),
    (0x0c3c, 4, 0x20202020),
    (0x0c40, 4, 0x1c1e2020),
    (0x0c44, 4, 0x1e1e181a),
    (0x0c48, 4, 0x1e1e1e1e),
    (0x0c4c, 4, 0x16181a1c),
    (0x0c54, 4, 0x00040a12),
    (0x0e20, 4, 0x14141414),
    (0x0e24, 4, 0x1d1d1d1d),
    (0x0e28, 4, 0x191b1d1d),
    (0x0e2c, 4, 0x1f1f1f1f),
    (0x0e30, 4, 0x1b1d1f1f),
    (0x0e34, 4, 0x1f1f1f1f),
    (0x0e38, 4, 0x191b1d1f),
    (0x0e3c, 4, 0x1f1f1f1f),
    (0x0e40, 4, 0x1b1d1f1f),
    (0x0e44, 4, 0x1d1d1719),
    (0x0e48, 4, 0x1d1d1d1d),
    (0x0e4c, 4, 0x1517191b),
    (0x0e54, 4, 0x00030911),
    (0x0454, 1, 0x00000080),
    (0x0808, 4, 0x3e028233),
    (0x0834, 4, 0x0037a706),
    (0x0830, 4, 0x2eaaaeb8),
    (0x0830, 4, 0x2eaaaeb8),
    (0x082c, 4, 0x002083dd),
    (0x0cb0, 4, 0x54337717),
    (0x0eb0, 4, 0x54337717),
    (0x0cb4, 4, 0x01000077),
    (0x0eb4, 4, 0x01000077),
    (0x0900, 4, 0x00000401),
    (0x080c, 4, 0x12131103),
    (0x0a04, 4, 0x0fff000c),
    (0x0c1c, 4, 0x2d400003),
    (0x0e1c, 4, 0x2d400003),
    (0x0860, 4, 0x68a7c321),
    (0x08b0, 4, 0x00000618),
    (0x0c90, 4, 0x01817d01),
    (0x08ac, 4, 0x0ff0fa08),
    (0x08ac, 4, 0x0ff0fa08),
    (0x08c4, 4, 0x00000000),
    (0x08b0, 4, 0x00000618),
    (0x0c90, 4, 0x01817d34),
    (0x08b0, 4, 0x00000618),
    (0x0e90, 4, 0x01817d01),
    (0x08ac, 4, 0x0ff0fa08),
    (0x08ac, 4, 0x0ff0fa08),
    (0x08c4, 4, 0x00000000),
    (0x08b0, 4, 0x00000618),
    (0x0e90, 4, 0x01817d34),
    (0x0668, 2, 0x00001000),
    (0x08ac, 4, 0x0ff0fa08),
    (0x08c4, 4, 0x00000000),
    (0x0848, 4, 0x61d0ff8b),
    (0x08ac, 4, 0x0ff0fa08),
    (0x08ac, 4, 0x0ff0fa08),
    (0x08c4, 4, 0x00000000),
    (0x08b0, 4, 0x00000618),
    (0x0c90, 4, 0x01817d34),
    (0x08b0, 4, 0x00000618),
    (0x0e90, 4, 0x01817d34),
    (0x0c24, 4, 0x24242424),
    (0x0c28, 4, 0x20222424),
    (0x0c2c, 4, 0x29292929),
    (0x0c30, 4, 0x21232527),
    (0x0c34, 4, 0x25252525),
    (0x0c38, 4, 0x1f212325),
    (0x0c3c, 4, 0x29292929),
    (0x0c40, 4, 0x21232527),
    (0x0c44, 4, 0x25251f1f),
    (0x0c48, 4, 0x23252525),
    (0x0c4c, 4, 0x1d1d1f21),
    (0x0c54, 4, 0x00090f17),
    (0x0e24, 4, 0x23232323),
    (0x0e28, 4, 0x1f212323),
    (0x0e2c, 4, 0x28282828),
    (0x0e30, 4, 0x20222426),
    (0x0e34, 4, 0x24242424),
    (0x0e38, 4, 0x1e202224),
    (0x0e3c, 4, 0x28282828),
    (0x0e40, 4, 0x20222426),
    (0x0e44, 4, 0x24241e1e),
    (0x0e48, 4, 0x22242424),
    (0x0e4c, 4, 0x1c1c1e20),
    (0x0e54, 4, 0x00080e16),
    (0x09a4, 4, 0x000a0080),
    (0x09a4, 4, 0x00080080),
    (0x0a2c, 4, 0x00900000),
    (0x0a2c, 4, 0x00908000),
    (0x0b58, 4, 0x00000001),
    (0x0b58, 4, 0x00000000),
    (0x0c50, 4, 0x00000022),
    (0x0e50, 4, 0x00000022),
    (0x08b0, 4, 0x00000642),
    (0x0c90, 4, 0x04238508),
    (0x09a4, 4, 0x000a0080),
    (0x09a4, 4, 0x00080080),
    (0x0a2c, 4, 0x00900000),
    (0x0a2c, 4, 0x00908000),
    (0x0b58, 4, 0x00000001),
    (0x0b58, 4, 0x00000000),
    (0x0c50, 4, 0x00000020),
    (0x0e50, 4, 0x00000020),
    (0x08b0, 4, 0x00000642),
    (0x0c1c, 4, 0x32e00003),
    (0x0e1c, 4, 0x35e00003),
];

/// The RTL8812AU **5 GHz ch56** channel program (UNII-2A DFS) — golden trace
/// (golden/rtw88-8812au-ch56-5g), same capture/decode as ch36.
static CH56_5G_PROGRAM: RegProgram = &[
    (0x0860, 4, 0x72d5c321),
    (0x08b0, 4, 0x00000618),
    (0x0c90, 4, 0x01807c01),
    (0x08ac, 4, 0x0ff0fa08),
    (0x08ac, 4, 0x0ff0fa08),
    (0x08c4, 4, 0x00000000),
    (0x08b0, 4, 0x00000618),
    (0x0c90, 4, 0x01807c01),
    (0x08b0, 4, 0x00000618),
    (0x0e90, 4, 0x01807c01),
    (0x08ac, 4, 0x0ff0fa08),
    (0x08ac, 4, 0x0ff0fa08),
    (0x08c4, 4, 0x00000000),
    (0x08b0, 4, 0x00000618),
    (0x0e90, 4, 0x01807c01),
    (0x0668, 2, 0x00001000),
    (0x08ac, 4, 0x0ff0fa08),
    (0x08c4, 4, 0x00000000),
    (0x0848, 4, 0x61d0ff8b),
    (0x08ac, 4, 0x0ff0fa08),
    (0x08ac, 4, 0x0ff0fa08),
    (0x08c4, 4, 0x00000000),
    (0x08b0, 4, 0x00000618),
    (0x0c90, 4, 0x01807c01),
    (0x08b0, 4, 0x00000618),
    (0x0e90, 4, 0x01807c01),
    (0x0c20, 4, 0x13131313),
    (0x0c24, 4, 0x20202020),
    (0x0c28, 4, 0x1c1e2020),
    (0x0c2c, 4, 0x20202020),
    (0x0c30, 4, 0x1c1e2020),
    (0x0c34, 4, 0x20202020),
    (0x0c38, 4, 0x1a1c1e20),
    (0x0c3c, 4, 0x20202020),
    (0x0c40, 4, 0x1c1e2020),
    (0x0c44, 4, 0x1e1e181a),
    (0x0c48, 4, 0x1e1e1e1e),
    (0x0c4c, 4, 0x16181a1c),
    (0x0c54, 4, 0x00040a12),
    (0x0e20, 4, 0x14141414),
    (0x0e24, 4, 0x1d1d1d1d),
    (0x0e28, 4, 0x191b1d1d),
    (0x0e2c, 4, 0x1f1f1f1f),
    (0x0e30, 4, 0x1b1d1f1f),
    (0x0e34, 4, 0x1f1f1f1f),
    (0x0e38, 4, 0x191b1d1f),
    (0x0e3c, 4, 0x1f1f1f1f),
    (0x0e40, 4, 0x1b1d1f1f),
    (0x0e44, 4, 0x1d1d1719),
    (0x0e48, 4, 0x1d1d1d1d),
    (0x0e4c, 4, 0x1517191b),
    (0x0e54, 4, 0x00030911),
    (0x0454, 1, 0x00000080),
    (0x0808, 4, 0x3e028233),
    (0x0834, 4, 0x0037a706),
    (0x0830, 4, 0x2eaaaeb8),
    (0x0830, 4, 0x2eaaaeb8),
    (0x082c, 4, 0x002083dd),
    (0x0cb0, 4, 0x54337717),
    (0x0eb0, 4, 0x54337717),
    (0x0cb4, 4, 0x01000077),
    (0x0eb4, 4, 0x01000077),
    (0x0900, 4, 0x00000401),
    (0x080c, 4, 0x12131103),
    (0x0a04, 4, 0x0fff000c),
    (0x0c1c, 4, 0x2d400003),
    (0x0e1c, 4, 0x2d400003),
    (0x0860, 4, 0x68a7c321),
    (0x08b0, 4, 0x00000618),
    (0x0c90, 4, 0x01817d01),
    (0x08ac, 4, 0x0ff0fa08),
    (0x08ac, 4, 0x0ff0fa08),
    (0x08c4, 4, 0x00000000),
    (0x08b0, 4, 0x00000618),
    (0x0c90, 4, 0x01817d38),
    (0x08b0, 4, 0x00000618),
    (0x0e90, 4, 0x01817d01),
    (0x08ac, 4, 0x0ff0fa08),
    (0x08ac, 4, 0x0ff0fa08),
    (0x08c4, 4, 0x00000000),
    (0x08b0, 4, 0x00000618),
    (0x0e90, 4, 0x01817d38),
    (0x0668, 2, 0x00001000),
    (0x08ac, 4, 0x0ff0fa08),
    (0x08c4, 4, 0x00000000),
    (0x0848, 4, 0x61d0ff8b),
    (0x08ac, 4, 0x0ff0fa08),
    (0x08ac, 4, 0x0ff0fa08),
    (0x08c4, 4, 0x00000000),
    (0x08b0, 4, 0x00000618),
    (0x0c90, 4, 0x01817d38),
    (0x08b0, 4, 0x00000618),
    (0x0e90, 4, 0x01817d38),
    (0x0c24, 4, 0x24242424),
    (0x0c28, 4, 0x20222424),
    (0x0c2c, 4, 0x29292929),
    (0x0c30, 4, 0x21232527),
    (0x0c34, 4, 0x25252525),
    (0x0c38, 4, 0x1f212325),
    (0x0c3c, 4, 0x29292929),
    (0x0c40, 4, 0x21232527),
    (0x0c44, 4, 0x25251f1f),
    (0x0c48, 4, 0x23252525),
    (0x0c4c, 4, 0x1d1d1f21),
    (0x0c54, 4, 0x00090f17),
    (0x0e24, 4, 0x23232323),
    (0x0e28, 4, 0x1f212323),
    (0x0e2c, 4, 0x28282828),
    (0x0e30, 4, 0x20222426),
    (0x0e34, 4, 0x24242424),
    (0x0e38, 4, 0x1e202224),
    (0x0e3c, 4, 0x28282828),
    (0x0e40, 4, 0x20222426),
    (0x0e44, 4, 0x24241e1e),
    (0x0e48, 4, 0x22242424),
    (0x0e4c, 4, 0x1c1c1e20),
    (0x0e54, 4, 0x00080e16),
    (0x09a4, 4, 0x000a0080),
    (0x09a4, 4, 0x00080080),
    (0x0a2c, 4, 0x00900000),
    (0x0a2c, 4, 0x00908000),
    (0x0b58, 4, 0x00000001),
    (0x0b58, 4, 0x00000000),
    (0x0c50, 4, 0x0000001c),
    (0x0e50, 4, 0x0000001c),
    (0x08b0, 4, 0x00000642),
    (0x0c90, 4, 0x04238908),
    (0x09a4, 4, 0x000a0080),
    (0x09a4, 4, 0x00080080),
    (0x0a2c, 4, 0x00900000),
    (0x0a2c, 4, 0x00908000),
    (0x0b58, 4, 0x00000001),
    (0x0b58, 4, 0x00000000),
    (0x08b0, 4, 0x00000642),
    (0x0c1c, 4, 0x32e00003),
    (0x0e1c, 4, 0x35e00003),
];

/// The RTL8812AU **5 GHz ch60** channel program (UNII-2A DFS) — golden trace
/// (golden/rtw88-8812au-ch60-5g), same capture/decode as ch36.
static CH60_5G_PROGRAM: RegProgram = &[
    (0x09a4, 4, 0x000a0080),
    (0x09a4, 4, 0x00080080),
    (0x0a2c, 4, 0x00900000),
    (0x0a2c, 4, 0x00908000),
    (0x0b58, 4, 0x00000001),
    (0x0b58, 4, 0x00000000),
    (0x08b0, 4, 0x00000642),
    (0x0c90, 4, 0x04238d08),
    (0x0860, 4, 0x72d5c321),
    (0x08b0, 4, 0x00000618),
    (0x0c90, 4, 0x01807c01),
    (0x08ac, 4, 0x0ff0fa08),
    (0x08ac, 4, 0x0ff0fa08),
    (0x08c4, 4, 0x00000000),
    (0x08b0, 4, 0x00000618),
    (0x0c90, 4, 0x01807c01),
    (0x08b0, 4, 0x00000618),
    (0x0e90, 4, 0x01807c01),
    (0x08ac, 4, 0x0ff0fa08),
    (0x08ac, 4, 0x0ff0fa08),
    (0x08c4, 4, 0x00000000),
    (0x08b0, 4, 0x00000618),
    (0x0e90, 4, 0x01807c01),
    (0x0668, 2, 0x00001000),
    (0x08ac, 4, 0x0ff0fa08),
    (0x08c4, 4, 0x00000000),
    (0x0848, 4, 0x61d0ff8b),
    (0x08ac, 4, 0x0ff0fa08),
    (0x08ac, 4, 0x0ff0fa08),
    (0x08c4, 4, 0x00000000),
    (0x08b0, 4, 0x00000618),
    (0x0c90, 4, 0x01807c01),
    (0x08b0, 4, 0x00000618),
    (0x0e90, 4, 0x01807c01),
    (0x0c20, 4, 0x13131313),
    (0x0c24, 4, 0x20202020),
    (0x0c28, 4, 0x1c1e2020),
    (0x0c2c, 4, 0x20202020),
    (0x0c30, 4, 0x1c1e2020),
    (0x0c34, 4, 0x20202020),
    (0x0c38, 4, 0x1a1c1e20),
    (0x0c3c, 4, 0x20202020),
    (0x0c40, 4, 0x1c1e2020),
    (0x0c44, 4, 0x1e1e181a),
    (0x0c48, 4, 0x1e1e1e1e),
    (0x0c4c, 4, 0x16181a1c),
    (0x0c54, 4, 0x00040a12),
    (0x0e20, 4, 0x14141414),
    (0x0e24, 4, 0x1d1d1d1d),
    (0x0e28, 4, 0x191b1d1d),
    (0x0e2c, 4, 0x1f1f1f1f),
    (0x0e30, 4, 0x1b1d1f1f),
    (0x0e34, 4, 0x1f1f1f1f),
    (0x0e38, 4, 0x191b1d1f),
    (0x0e3c, 4, 0x1f1f1f1f),
    (0x0e40, 4, 0x1b1d1f1f),
    (0x0e44, 4, 0x1d1d1719),
    (0x0e48, 4, 0x1d1d1d1d),
    (0x0e4c, 4, 0x1517191b),
    (0x0e54, 4, 0x00030911),
    (0x0454, 1, 0x00000080),
    (0x0808, 4, 0x3e028233),
    (0x0834, 4, 0x0037a706),
    (0x0830, 4, 0x2eaaaeb8),
    (0x0830, 4, 0x2eaaaeb8),
    (0x082c, 4, 0x002083dd),
    (0x0cb0, 4, 0x54337717),
    (0x0eb0, 4, 0x54337717),
    (0x0cb4, 4, 0x01000077),
    (0x0eb4, 4, 0x01000077),
    (0x0900, 4, 0x00000401),
    (0x080c, 4, 0x12131103),
    (0x0a04, 4, 0x0fff000c),
    (0x0c1c, 4, 0x2d400003),
    (0x0e1c, 4, 0x2d400003),
    (0x0860, 4, 0x68a7c321),
    (0x08b0, 4, 0x00000618),
    (0x0c90, 4, 0x01817d01),
    (0x08ac, 4, 0x0ff0fa08),
    (0x08ac, 4, 0x0ff0fa08),
    (0x08c4, 4, 0x00000000),
    (0x08b0, 4, 0x00000618),
    (0x0c90, 4, 0x01817d3c),
    (0x08b0, 4, 0x00000618),
    (0x0e90, 4, 0x01817d01),
    (0x08ac, 4, 0x0ff0fa08),
    (0x08ac, 4, 0x0ff0fa08),
    (0x08c4, 4, 0x00000000),
    (0x08b0, 4, 0x00000618),
    (0x0e90, 4, 0x01817d3c),
    (0x0668, 2, 0x00001000),
    (0x08ac, 4, 0x0ff0fa08),
    (0x08c4, 4, 0x00000000),
    (0x0848, 4, 0x61d0ff8b),
    (0x08ac, 4, 0x0ff0fa08),
    (0x08ac, 4, 0x0ff0fa08),
    (0x08c4, 4, 0x00000000),
    (0x08b0, 4, 0x00000618),
    (0x0c90, 4, 0x01817d3c),
    (0x08b0, 4, 0x00000618),
    (0x0e90, 4, 0x01817d3c),
    (0x0c24, 4, 0x22222222),
    (0x0c28, 4, 0x1e202222),
    (0x0c2c, 4, 0x27272727),
    (0x0c30, 4, 0x1f212325),
    (0x0c34, 4, 0x23232323),
    (0x0c38, 4, 0x1d1f2123),
    (0x0c3c, 4, 0x27272727),
    (0x0c40, 4, 0x1f212325),
    (0x0c44, 4, 0x23231d1d),
    (0x0c48, 4, 0x21232323),
    (0x0c4c, 4, 0x1b1b1d1f),
    (0x0c54, 4, 0x00070d15),
    (0x0e24, 4, 0x22222222),
    (0x0e28, 4, 0x1e202222),
    (0x0e2c, 4, 0x27272727),
    (0x0e30, 4, 0x1f212325),
    (0x0e34, 4, 0x23232323),
    (0x0e38, 4, 0x1d1f2123),
    (0x0e3c, 4, 0x27272727),
    (0x0e40, 4, 0x1f212325),
    (0x0e44, 4, 0x23231d1d),
    (0x0e48, 4, 0x21232323),
    (0x0e4c, 4, 0x1b1b1d1f),
    (0x0e54, 4, 0x00070d15),
    (0x09a4, 4, 0x000a0080),
    (0x09a4, 4, 0x00080080),
    (0x0a2c, 4, 0x00900000),
    (0x0a2c, 4, 0x00908000),
    (0x0b58, 4, 0x00000001),
    (0x0b58, 4, 0x00000000),
    (0x08b0, 4, 0x00000642),
    (0x0c90, 4, 0x04238d08),
];

/// The RTL8812AU **5 GHz ch64** channel program (UNII-2A DFS) — golden trace
/// (golden/rtw88-8812au-ch64-5g), same capture/decode as ch36.
static CH64_5G_PROGRAM: RegProgram = &[
    (0x0860, 4, 0x72d5c321),
    (0x08b0, 4, 0x00000618),
    (0x0c90, 4, 0x01807c01),
    (0x08ac, 4, 0x0ff0fa08),
    (0x08ac, 4, 0x0ff0fa08),
    (0x08c4, 4, 0x00000000),
    (0x08b0, 4, 0x00000618),
    (0x0c90, 4, 0x01807c01),
    (0x08b0, 4, 0x00000618),
    (0x0e90, 4, 0x01807c01),
    (0x08ac, 4, 0x0ff0fa08),
    (0x08ac, 4, 0x0ff0fa08),
    (0x08c4, 4, 0x00000000),
    (0x08b0, 4, 0x00000618),
    (0x0e90, 4, 0x01807c01),
    (0x0668, 2, 0x00001000),
    (0x08ac, 4, 0x0ff0fa08),
    (0x08c4, 4, 0x00000000),
    (0x0848, 4, 0x61d0ff8b),
    (0x08ac, 4, 0x0ff0fa08),
    (0x08ac, 4, 0x0ff0fa08),
    (0x08c4, 4, 0x00000000),
    (0x08b0, 4, 0x00000618),
    (0x0c90, 4, 0x01807c01),
    (0x08b0, 4, 0x00000618),
    (0x0e90, 4, 0x01807c01),
    (0x0c20, 4, 0x13131313),
    (0x0c24, 4, 0x20202020),
    (0x0c28, 4, 0x1c1e2020),
    (0x0c2c, 4, 0x20202020),
    (0x0c30, 4, 0x1c1e2020),
    (0x0c34, 4, 0x20202020),
    (0x0c38, 4, 0x1a1c1e20),
    (0x0c3c, 4, 0x20202020),
    (0x0c40, 4, 0x1c1e2020),
    (0x0c44, 4, 0x1e1e181a),
    (0x0c48, 4, 0x1e1e1e1e),
    (0x0c4c, 4, 0x16181a1c),
    (0x0c54, 4, 0x00040a12),
    (0x0e20, 4, 0x14141414),
    (0x0e24, 4, 0x1d1d1d1d),
    (0x0e28, 4, 0x191b1d1d),
    (0x0e2c, 4, 0x1f1f1f1f),
    (0x0e30, 4, 0x1b1d1f1f),
    (0x0e34, 4, 0x1f1f1f1f),
    (0x0e38, 4, 0x191b1d1f),
    (0x0e3c, 4, 0x1f1f1f1f),
    (0x0e40, 4, 0x1b1d1f1f),
    (0x0e44, 4, 0x1d1d1719),
    (0x0e48, 4, 0x1d1d1d1d),
    (0x0e4c, 4, 0x1517191b),
    (0x0e54, 4, 0x00030911),
    (0x0454, 1, 0x00000080),
    (0x0808, 4, 0x3e028233),
    (0x0834, 4, 0x0037a706),
    (0x0830, 4, 0x2eaaaeb8),
    (0x0830, 4, 0x2eaaaeb8),
    (0x082c, 4, 0x002083dd),
    (0x0cb0, 4, 0x54337717),
    (0x0eb0, 4, 0x54337717),
    (0x0cb4, 4, 0x01000077),
    (0x0eb4, 4, 0x01000077),
    (0x0900, 4, 0x00000401),
    (0x080c, 4, 0x12131103),
    (0x0a04, 4, 0x0fff000c),
    (0x0c1c, 4, 0x2d400003),
    (0x0e1c, 4, 0x2d400003),
    (0x0860, 4, 0x68a7c321),
    (0x08b0, 4, 0x00000618),
    (0x0c90, 4, 0x01817d01),
    (0x08ac, 4, 0x0ff0fa08),
    (0x08ac, 4, 0x0ff0fa08),
    (0x08c4, 4, 0x00000000),
    (0x08b0, 4, 0x00000618),
    (0x0c90, 4, 0x01817d40),
    (0x08b0, 4, 0x00000618),
    (0x0e90, 4, 0x01817d01),
    (0x08ac, 4, 0x0ff0fa08),
    (0x08ac, 4, 0x0ff0fa08),
    (0x08c4, 4, 0x00000000),
    (0x08b0, 4, 0x00000618),
    (0x0e90, 4, 0x01817d40),
    (0x0668, 2, 0x00001000),
    (0x08ac, 4, 0x0ff0fa08),
    (0x08c4, 4, 0x00000000),
    (0x0848, 4, 0x61d0ff8b),
    (0x08ac, 4, 0x0ff0fa08),
    (0x08ac, 4, 0x0ff0fa08),
    (0x08c4, 4, 0x00000000),
    (0x08b0, 4, 0x00000618),
    (0x0c90, 4, 0x01817d40),
    (0x08b0, 4, 0x00000618),
    (0x0e90, 4, 0x01817d40),
    (0x0c24, 4, 0x1e1e1e1e),
    (0x0c28, 4, 0x1e1e1e1e),
    (0x0c2c, 4, 0x23232323),
    (0x0c30, 4, 0x1f212323),
    (0x0c34, 4, 0x1f1f1f1f),
    (0x0c38, 4, 0x1d1f1f1f),
    (0x0c3c, 4, 0x23232323),
    (0x0c40, 4, 0x1f212323),
    (0x0c44, 4, 0x1f1f1d1d),
    (0x0c48, 4, 0x1f1f1f1f),
    (0x0c4c, 4, 0x1b1b1d1f),
    (0x0c54, 4, 0x00070d15),
    (0x0e24, 4, 0x1e1e1e1e),
    (0x0e28, 4, 0x1e1e1e1e),
    (0x0e2c, 4, 0x23232323),
    (0x0e30, 4, 0x1f212323),
    (0x0e34, 4, 0x1f1f1f1f),
    (0x0e38, 4, 0x1d1f1f1f),
    (0x0e3c, 4, 0x23232323),
    (0x0e40, 4, 0x1f212323),
    (0x0e44, 4, 0x1f1f1d1d),
    (0x0e48, 4, 0x1f1f1f1f),
    (0x0e4c, 4, 0x1b1b1d1f),
    (0x0e54, 4, 0x00070d15),
    (0x09a4, 4, 0x000a0080),
    (0x09a4, 4, 0x00080080),
    (0x0a2c, 4, 0x00900000),
    (0x0a2c, 4, 0x00908000),
    (0x0b58, 4, 0x00000001),
    (0x0b58, 4, 0x00000000),
    (0x08b0, 4, 0x00000642),
    (0x0c90, 4, 0x04238d08),
    (0x09a4, 4, 0x000a0080),
    (0x09a4, 4, 0x00080080),
    (0x0a2c, 4, 0x00900000),
    (0x0a2c, 4, 0x00908000),
    (0x0b58, 4, 0x00000001),
    (0x0b58, 4, 0x00000000),
    (0x08b0, 4, 0x00000642),
    (0x0c1c, 4, 0x32e00003),
    (0x0e1c, 4, 0x35e00003),
];

/// The RTL8812AU **5 GHz ch104** channel program (UNII-2C DFS) — golden trace
/// (golden/rtw88-8812au-ch104-5g), same capture/decode as ch36.
static CH104_5G_PROGRAM: RegProgram = &[
    (0x09a4, 4, 0x000a0080),
    (0x09a4, 4, 0x00080080),
    (0x0a2c, 4, 0x00900000),
    (0x0a2c, 4, 0x00908000),
    (0x0b58, 4, 0x00000001),
    (0x0b58, 4, 0x00000000),
    (0x08b0, 4, 0x00000642),
    (0x0c90, 4, 0x04238d08),
    (0x0860, 4, 0x72d5c321),
    (0x08b0, 4, 0x00000618),
    (0x0c90, 4, 0x01807c01),
    (0x08ac, 4, 0x0ff0fa08),
    (0x08ac, 4, 0x0ff0fa08),
    (0x08c4, 4, 0x00000000),
    (0x08b0, 4, 0x00000618),
    (0x0c90, 4, 0x01807c01),
    (0x08b0, 4, 0x00000618),
    (0x0e90, 4, 0x01807c01),
    (0x08ac, 4, 0x0ff0fa08),
    (0x08ac, 4, 0x0ff0fa08),
    (0x08c4, 4, 0x00000000),
    (0x08b0, 4, 0x00000618),
    (0x0e90, 4, 0x01807c01),
    (0x0668, 2, 0x00001000),
    (0x08ac, 4, 0x0ff0fa08),
    (0x08c4, 4, 0x00000000),
    (0x0848, 4, 0x61d0ff8b),
    (0x08ac, 4, 0x0ff0fa08),
    (0x08ac, 4, 0x0ff0fa08),
    (0x08c4, 4, 0x00000000),
    (0x08b0, 4, 0x00000618),
    (0x0c90, 4, 0x01807c01),
    (0x08b0, 4, 0x00000618),
    (0x0e90, 4, 0x01807c01),
    (0x0c20, 4, 0x13131313),
    (0x0c24, 4, 0x20202020),
    (0x0c28, 4, 0x1c1e2020),
    (0x0c2c, 4, 0x20202020),
    (0x0c30, 4, 0x1c1e2020),
    (0x0c34, 4, 0x20202020),
    (0x0c38, 4, 0x1a1c1e20),
    (0x0c3c, 4, 0x20202020),
    (0x0c40, 4, 0x1c1e2020),
    (0x0c44, 4, 0x1e1e181a),
    (0x0c48, 4, 0x1e1e1e1e),
    (0x0c4c, 4, 0x16181a1c),
    (0x0c54, 4, 0x00040a12),
    (0x0e20, 4, 0x14141414),
    (0x0e24, 4, 0x1d1d1d1d),
    (0x0e28, 4, 0x191b1d1d),
    (0x0e2c, 4, 0x1f1f1f1f),
    (0x0e30, 4, 0x1b1d1f1f),
    (0x0e34, 4, 0x1f1f1f1f),
    (0x0e38, 4, 0x191b1d1f),
    (0x0e3c, 4, 0x1f1f1f1f),
    (0x0e40, 4, 0x1b1d1f1f),
    (0x0e44, 4, 0x1d1d1719),
    (0x0e48, 4, 0x1d1d1d1d),
    (0x0e4c, 4, 0x1517191b),
    (0x0e54, 4, 0x00030911),
    (0x0454, 1, 0x00000080),
    (0x0808, 4, 0x3e028233),
    (0x0834, 4, 0x0037a706),
    (0x0830, 4, 0x2eaaaeb8),
    (0x0830, 4, 0x2eaaaeb8),
    (0x082c, 4, 0x002083dd),
    (0x0cb0, 4, 0x54337717),
    (0x0eb0, 4, 0x54337717),
    (0x0cb4, 4, 0x01000077),
    (0x0eb4, 4, 0x01000077),
    (0x0900, 4, 0x00000401),
    (0x080c, 4, 0x12131103),
    (0x0a04, 4, 0x0fff000c),
    (0x0c1c, 4, 0x2d400003),
    (0x0e1c, 4, 0x2d400003),
    (0x0860, 4, 0x68a5c321),
    (0x08b0, 4, 0x00000618),
    (0x0c90, 4, 0x01837d01),
    (0x08ac, 4, 0x0ff0fa08),
    (0x08ac, 4, 0x0ff0fa08),
    (0x08c4, 4, 0x00000000),
    (0x08b0, 4, 0x00000618),
    (0x0c90, 4, 0x01817d68),
    (0x08b0, 4, 0x00000618),
    (0x0e90, 4, 0x01837d01),
    (0x08ac, 4, 0x0ff0fa08),
    (0x08ac, 4, 0x0ff0fa08),
    (0x08c4, 4, 0x00000000),
    (0x08b0, 4, 0x00000618),
    (0x0e90, 4, 0x01817d68),
    (0x0668, 2, 0x00001000),
    (0x08ac, 4, 0x0ff0fa08),
    (0x08c4, 4, 0x00000000),
    (0x0848, 4, 0x61d0ff8b),
    (0x08ac, 4, 0x0ff0fa08),
    (0x08ac, 4, 0x0ff0fa08),
    (0x08c4, 4, 0x00000000),
    (0x08b0, 4, 0x00000618),
    (0x0c90, 4, 0x01837d68),
    (0x08b0, 4, 0x00000618),
    (0x0e90, 4, 0x01837d68),
    (0x0c24, 4, 0x27272727),
    (0x0c28, 4, 0x25272727),
    (0x0c2c, 4, 0x2c2c2c2c),
    (0x0c30, 4, 0x26282a2c),
    (0x0c34, 4, 0x28282828),
    (0x0c38, 4, 0x24262828),
    (0x0c3c, 4, 0x2c2c2c2c),
    (0x0c40, 4, 0x26282a2c),
    (0x0c44, 4, 0x28282424),
    (0x0c48, 4, 0x28282828),
    (0x0c4c, 4, 0x22222426),
    (0x0c54, 4, 0x000e141c),
    (0x0e24, 4, 0x21212121),
    (0x0e28, 4, 0x1f212121),
    (0x0e2c, 4, 0x26262626),
    (0x0e30, 4, 0x20222426),
    (0x0e34, 4, 0x22222222),
    (0x0e38, 4, 0x1e202222),
    (0x0e3c, 4, 0x26262626),
    (0x0e40, 4, 0x20222426),
    (0x0e44, 4, 0x22221e1e),
    (0x0e48, 4, 0x22222222),
    (0x0e4c, 4, 0x1c1c1e20),
    (0x0e54, 4, 0x00080e16),
    (0x09a4, 4, 0x000a0080),
    (0x09a4, 4, 0x00080080),
    (0x0a2c, 4, 0x00900000),
    (0x0a2c, 4, 0x00908000),
    (0x0b58, 4, 0x00000001),
    (0x0b58, 4, 0x00000000),
    (0x08b0, 4, 0x00000642),
    (0x0c90, 4, 0x04238d08),
];

/// The RTL8812AU **5 GHz ch108** channel program (UNII-2C DFS) — golden trace
/// (golden/rtw88-8812au-ch108-5g), same capture/decode as ch36.
static CH108_5G_PROGRAM: RegProgram = &[
    (0x0860, 4, 0x72d5c321),
    (0x08b0, 4, 0x00000618),
    (0x0c90, 4, 0x01807c01),
    (0x08ac, 4, 0x0ff0fa08),
    (0x08ac, 4, 0x0ff0fa08),
    (0x08c4, 4, 0x00000000),
    (0x08b0, 4, 0x00000618),
    (0x0c90, 4, 0x01807c01),
    (0x08b0, 4, 0x00000618),
    (0x0e90, 4, 0x01807c01),
    (0x08ac, 4, 0x0ff0fa08),
    (0x08ac, 4, 0x0ff0fa08),
    (0x08c4, 4, 0x00000000),
    (0x08b0, 4, 0x00000618),
    (0x0e90, 4, 0x01807c01),
    (0x0668, 2, 0x00001000),
    (0x08ac, 4, 0x0ff0fa08),
    (0x08c4, 4, 0x00000000),
    (0x0848, 4, 0x61d0ff8b),
    (0x08ac, 4, 0x0ff0fa08),
    (0x08ac, 4, 0x0ff0fa08),
    (0x08c4, 4, 0x00000000),
    (0x08b0, 4, 0x00000618),
    (0x0c90, 4, 0x01807c01),
    (0x08b0, 4, 0x00000618),
    (0x0e90, 4, 0x01807c01),
    (0x0c20, 4, 0x13131313),
    (0x0c24, 4, 0x20202020),
    (0x0c28, 4, 0x1c1e2020),
    (0x0c2c, 4, 0x20202020),
    (0x0c30, 4, 0x1c1e2020),
    (0x0c34, 4, 0x20202020),
    (0x0c38, 4, 0x1a1c1e20),
    (0x0c3c, 4, 0x20202020),
    (0x0c40, 4, 0x1c1e2020),
    (0x0c44, 4, 0x1e1e181a),
    (0x0c48, 4, 0x1e1e1e1e),
    (0x0c4c, 4, 0x16181a1c),
    (0x0c54, 4, 0x00040a12),
    (0x0e20, 4, 0x14141414),
    (0x0e24, 4, 0x1d1d1d1d),
    (0x0e28, 4, 0x191b1d1d),
    (0x0e2c, 4, 0x1f1f1f1f),
    (0x0e30, 4, 0x1b1d1f1f),
    (0x0e34, 4, 0x1f1f1f1f),
    (0x0e38, 4, 0x191b1d1f),
    (0x0e3c, 4, 0x1f1f1f1f),
    (0x0e40, 4, 0x1b1d1f1f),
    (0x0e44, 4, 0x1d1d1719),
    (0x0e48, 4, 0x1d1d1d1d),
    (0x0e4c, 4, 0x1517191b),
    (0x0e54, 4, 0x00030911),
    (0x0454, 1, 0x00000080),
    (0x0808, 4, 0x3e028233),
    (0x0834, 4, 0x0037a706),
    (0x0830, 4, 0x2eaaaeb8),
    (0x0830, 4, 0x2eaaaeb8),
    (0x082c, 4, 0x002083dd),
    (0x0cb0, 4, 0x54337717),
    (0x0eb0, 4, 0x54337717),
    (0x0cb4, 4, 0x01000077),
    (0x0eb4, 4, 0x01000077),
    (0x0900, 4, 0x00000401),
    (0x080c, 4, 0x12131103),
    (0x0a04, 4, 0x0fff000c),
    (0x0c1c, 4, 0x2d400003),
    (0x0e1c, 4, 0x2d400003),
    (0x0860, 4, 0x68a5c321),
    (0x08b0, 4, 0x00000618),
    (0x0c90, 4, 0x01837d01),
    (0x08ac, 4, 0x0ff0fa08),
    (0x08ac, 4, 0x0ff0fa08),
    (0x08c4, 4, 0x00000000),
    (0x08b0, 4, 0x00000618),
    (0x0c90, 4, 0x01817d6c),
    (0x08b0, 4, 0x00000618),
    (0x0e90, 4, 0x01837d01),
    (0x08ac, 4, 0x0ff0fa08),
    (0x08ac, 4, 0x0ff0fa08),
    (0x08c4, 4, 0x00000000),
    (0x08b0, 4, 0x00000618),
    (0x0e90, 4, 0x01817d6c),
    (0x0668, 2, 0x00001000),
    (0x08ac, 4, 0x0ff0fa08),
    (0x08c4, 4, 0x00000000),
    (0x0848, 4, 0x61d0ff8b),
    (0x08ac, 4, 0x0ff0fa08),
    (0x08ac, 4, 0x0ff0fa08),
    (0x08c4, 4, 0x00000000),
    (0x08b0, 4, 0x00000618),
    (0x0c90, 4, 0x01837d6c),
    (0x08b0, 4, 0x00000618),
    (0x0e90, 4, 0x01837d6c),
    (0x0c24, 4, 0x29292929),
    (0x0c28, 4, 0x25272929),
    (0x0c2c, 4, 0x2e2e2e2e),
    (0x0c30, 4, 0x26282a2c),
    (0x0c34, 4, 0x2a2a2a2a),
    (0x0c38, 4, 0x2426282a),
    (0x0c3c, 4, 0x2e2e2e2e),
    (0x0c40, 4, 0x26282a2c),
    (0x0c44, 4, 0x2a2a2424),
    (0x0c48, 4, 0x282a2a2a),
    (0x0c4c, 4, 0x22222426),
    (0x0c54, 4, 0x000e141c),
    (0x0e24, 4, 0x20202020),
    (0x0e28, 4, 0x1c1e2020),
    (0x0e2c, 4, 0x25252525),
    (0x0e30, 4, 0x1d1f2123),
    (0x0e34, 4, 0x21212121),
    (0x0e38, 4, 0x1b1d1f21),
    (0x0e3c, 4, 0x25252525),
    (0x0e40, 4, 0x1d1f2123),
    (0x0e44, 4, 0x21211b1b),
    (0x0e48, 4, 0x1f212121),
    (0x0e4c, 4, 0x19191b1d),
    (0x0e54, 4, 0x00050b13),
    (0x09a4, 4, 0x000a0080),
    (0x09a4, 4, 0x00080080),
    (0x0a2c, 4, 0x00900000),
    (0x0a2c, 4, 0x00908000),
    (0x0b58, 4, 0x00000001),
    (0x0b58, 4, 0x00000000),
    (0x08b0, 4, 0x00000642),
    (0x0c90, 4, 0x04238d08),
    (0x09a4, 4, 0x000a0080),
    (0x09a4, 4, 0x00080080),
    (0x0a2c, 4, 0x00900000),
    (0x0a2c, 4, 0x00908000),
    (0x0b58, 4, 0x00000001),
    (0x0b58, 4, 0x00000000),
    (0x08b0, 4, 0x00000642),
    (0x0c1c, 4, 0x39000003),
    (0x0e1c, 4, 0x32e00003),
];

/// The RTL8812AU **5 GHz ch112** channel program (UNII-2C DFS) — golden trace
/// (golden/rtw88-8812au-ch112-5g), same capture/decode as ch36.
static CH112_5G_PROGRAM: RegProgram = &[
    (0x09a4, 4, 0x000a0080),
    (0x09a4, 4, 0x00080080),
    (0x0a2c, 4, 0x00900000),
    (0x0a2c, 4, 0x00908000),
    (0x0b58, 4, 0x00000001),
    (0x0b58, 4, 0x00000000),
    (0x08b0, 4, 0x00000642),
    (0x0c90, 4, 0x04238d08),
    (0x0860, 4, 0x72d5c321),
    (0x08b0, 4, 0x00000618),
    (0x0c90, 4, 0x01807c01),
    (0x08ac, 4, 0x0ff0fa08),
    (0x08ac, 4, 0x0ff0fa08),
    (0x08c4, 4, 0x00000000),
    (0x08b0, 4, 0x00000618),
    (0x0c90, 4, 0x01807c01),
    (0x08b0, 4, 0x00000618),
    (0x0e90, 4, 0x01807c01),
    (0x08ac, 4, 0x0ff0fa08),
    (0x08ac, 4, 0x0ff0fa08),
    (0x08c4, 4, 0x00000000),
    (0x08b0, 4, 0x00000618),
    (0x0e90, 4, 0x01807c01),
    (0x0668, 2, 0x00001000),
    (0x08ac, 4, 0x0ff0fa08),
    (0x08c4, 4, 0x00000000),
    (0x0848, 4, 0x61d0ff8b),
    (0x08ac, 4, 0x0ff0fa08),
    (0x08ac, 4, 0x0ff0fa08),
    (0x08c4, 4, 0x00000000),
    (0x08b0, 4, 0x00000618),
    (0x0c90, 4, 0x01807c01),
    (0x08b0, 4, 0x00000618),
    (0x0e90, 4, 0x01807c01),
    (0x0c20, 4, 0x13131313),
    (0x0c24, 4, 0x20202020),
    (0x0c28, 4, 0x1c1e2020),
    (0x0c2c, 4, 0x20202020),
    (0x0c30, 4, 0x1c1e2020),
    (0x0c34, 4, 0x20202020),
    (0x0c38, 4, 0x1a1c1e20),
    (0x0c3c, 4, 0x20202020),
    (0x0c40, 4, 0x1c1e2020),
    (0x0c44, 4, 0x1e1e181a),
    (0x0c48, 4, 0x1e1e1e1e),
    (0x0c4c, 4, 0x16181a1c),
    (0x0c54, 4, 0x00040a12),
    (0x0e20, 4, 0x14141414),
    (0x0e24, 4, 0x1d1d1d1d),
    (0x0e28, 4, 0x191b1d1d),
    (0x0e2c, 4, 0x1f1f1f1f),
    (0x0e30, 4, 0x1b1d1f1f),
    (0x0e34, 4, 0x1f1f1f1f),
    (0x0e38, 4, 0x191b1d1f),
    (0x0e3c, 4, 0x1f1f1f1f),
    (0x0e40, 4, 0x1b1d1f1f),
    (0x0e44, 4, 0x1d1d1719),
    (0x0e48, 4, 0x1d1d1d1d),
    (0x0e4c, 4, 0x1517191b),
    (0x0e54, 4, 0x00030911),
    (0x0454, 1, 0x00000080),
    (0x0808, 4, 0x3e028233),
    (0x0834, 4, 0x0037a706),
    (0x0830, 4, 0x2eaaaeb8),
    (0x0830, 4, 0x2eaaaeb8),
    (0x082c, 4, 0x002083dd),
    (0x0cb0, 4, 0x54337717),
    (0x0eb0, 4, 0x54337717),
    (0x0cb4, 4, 0x01000077),
    (0x0eb4, 4, 0x01000077),
    (0x0900, 4, 0x00000401),
    (0x080c, 4, 0x12131103),
    (0x0a04, 4, 0x0fff000c),
    (0x0c1c, 4, 0x2d400003),
    (0x0e1c, 4, 0x2d400003),
    (0x0860, 4, 0x68a5c321),
    (0x08b0, 4, 0x00000618),
    (0x0c90, 4, 0x01837d01),
    (0x08ac, 4, 0x0ff0fa08),
    (0x08ac, 4, 0x0ff0fa08),
    (0x08c4, 4, 0x00000000),
    (0x08b0, 4, 0x00000618),
    (0x0c90, 4, 0x01817d70),
    (0x08b0, 4, 0x00000618),
    (0x0e90, 4, 0x01837d01),
    (0x08ac, 4, 0x0ff0fa08),
    (0x08ac, 4, 0x0ff0fa08),
    (0x08c4, 4, 0x00000000),
    (0x08b0, 4, 0x00000618),
    (0x0e90, 4, 0x01817d70),
    (0x0668, 2, 0x00001000),
    (0x08ac, 4, 0x0ff0fa08),
    (0x08c4, 4, 0x00000000),
    (0x0848, 4, 0x61d0ff8b),
    (0x08ac, 4, 0x0ff0fa08),
    (0x08ac, 4, 0x0ff0fa08),
    (0x08c4, 4, 0x00000000),
    (0x08b0, 4, 0x00000618),
    (0x0c90, 4, 0x01837d70),
    (0x08b0, 4, 0x00000618),
    (0x0e90, 4, 0x01837d70),
    (0x0c24, 4, 0x29292929),
    (0x0c28, 4, 0x25272929),
    (0x0c2c, 4, 0x2e2e2e2e),
    (0x0c30, 4, 0x26282a2c),
    (0x0c34, 4, 0x2a2a2a2a),
    (0x0c38, 4, 0x2426282a),
    (0x0c3c, 4, 0x2e2e2e2e),
    (0x0c40, 4, 0x26282a2c),
    (0x0c44, 4, 0x2a2a2424),
    (0x0c48, 4, 0x282a2a2a),
    (0x0c4c, 4, 0x22222426),
    (0x0c54, 4, 0x000e141c),
    (0x0e24, 4, 0x20202020),
    (0x0e28, 4, 0x1c1e2020),
    (0x0e2c, 4, 0x25252525),
    (0x0e30, 4, 0x1d1f2123),
    (0x0e34, 4, 0x21212121),
    (0x0e38, 4, 0x1b1d1f21),
    (0x0e3c, 4, 0x25252525),
    (0x0e40, 4, 0x1d1f2123),
    (0x0e44, 4, 0x21211b1b),
    (0x0e48, 4, 0x1f212121),
    (0x0e4c, 4, 0x19191b1d),
    (0x0e54, 4, 0x00050b13),
    (0x09a4, 4, 0x000a0080),
    (0x09a4, 4, 0x00080080),
    (0x0a2c, 4, 0x00900000),
    (0x0a2c, 4, 0x00908000),
    (0x0b58, 4, 0x00000001),
    (0x0b58, 4, 0x00000000),
    (0x08b0, 4, 0x00000642),
    (0x0c90, 4, 0x04238d08),
];

/// The RTL8812AU **5 GHz ch120** channel program (UNII-2C DFS) — golden trace
/// (golden/rtw88-8812au-ch120-5g), same capture/decode as ch36.
static CH120_5G_PROGRAM: RegProgram = &[
    (0x0860, 4, 0x72d5c321),
    (0x08b0, 4, 0x00000618),
    (0x0c90, 4, 0x01807c01),
    (0x08ac, 4, 0x0ff0fa08),
    (0x08ac, 4, 0x0ff0fa08),
    (0x08c4, 4, 0x00000000),
    (0x08b0, 4, 0x00000618),
    (0x0c90, 4, 0x01807c01),
    (0x08b0, 4, 0x00000618),
    (0x0e90, 4, 0x01807c01),
    (0x08ac, 4, 0x0ff0fa08),
    (0x08ac, 4, 0x0ff0fa08),
    (0x08c4, 4, 0x00000000),
    (0x08b0, 4, 0x00000618),
    (0x0e90, 4, 0x01807c01),
    (0x0668, 2, 0x00001000),
    (0x08ac, 4, 0x0ff0fa08),
    (0x08c4, 4, 0x00000000),
    (0x0848, 4, 0x61d0ff8b),
    (0x08ac, 4, 0x0ff0fa08),
    (0x08ac, 4, 0x0ff0fa08),
    (0x08c4, 4, 0x00000000),
    (0x08b0, 4, 0x00000618),
    (0x0c90, 4, 0x01807c01),
    (0x08b0, 4, 0x00000618),
    (0x0e90, 4, 0x01807c01),
    (0x0c20, 4, 0x13131313),
    (0x0c24, 4, 0x20202020),
    (0x0c28, 4, 0x1c1e2020),
    (0x0c2c, 4, 0x20202020),
    (0x0c30, 4, 0x1c1e2020),
    (0x0c34, 4, 0x20202020),
    (0x0c38, 4, 0x1a1c1e20),
    (0x0c3c, 4, 0x20202020),
    (0x0c40, 4, 0x1c1e2020),
    (0x0c44, 4, 0x1e1e181a),
    (0x0c48, 4, 0x1e1e1e1e),
    (0x0c4c, 4, 0x16181a1c),
    (0x0c54, 4, 0x00040a12),
    (0x0e20, 4, 0x14141414),
    (0x0e24, 4, 0x1d1d1d1d),
    (0x0e28, 4, 0x191b1d1d),
    (0x0e2c, 4, 0x1f1f1f1f),
    (0x0e30, 4, 0x1b1d1f1f),
    (0x0e34, 4, 0x1f1f1f1f),
    (0x0e38, 4, 0x191b1d1f),
    (0x0e3c, 4, 0x1f1f1f1f),
    (0x0e40, 4, 0x1b1d1f1f),
    (0x0e44, 4, 0x1d1d1719),
    (0x0e48, 4, 0x1d1d1d1d),
    (0x0e4c, 4, 0x1517191b),
    (0x0e54, 4, 0x00030911),
    (0x0454, 1, 0x00000080),
    (0x0808, 4, 0x3e028233),
    (0x0834, 4, 0x0037a706),
    (0x0830, 4, 0x2eaaaeb8),
    (0x0830, 4, 0x2eaaaeb8),
    (0x082c, 4, 0x002083dd),
    (0x0cb0, 4, 0x54337717),
    (0x0eb0, 4, 0x54337717),
    (0x0cb4, 4, 0x01000077),
    (0x0eb4, 4, 0x01000077),
    (0x0900, 4, 0x00000401),
    (0x080c, 4, 0x12131103),
    (0x0a04, 4, 0x0fff000c),
    (0x0c1c, 4, 0x2d400003),
    (0x0e1c, 4, 0x2d400003),
    (0x0860, 4, 0x6825c321),
    (0x08b0, 4, 0x00000618),
    (0x0c90, 4, 0x01837d01),
    (0x08ac, 4, 0x0ff0fa08),
    (0x08ac, 4, 0x0ff0fa08),
    (0x08c4, 4, 0x00000000),
    (0x08b0, 4, 0x00000618),
    (0x0c90, 4, 0x01817d78),
    (0x08b0, 4, 0x00000618),
    (0x0e90, 4, 0x01837d01),
    (0x08ac, 4, 0x0ff0fa08),
    (0x08ac, 4, 0x0ff0fa08),
    (0x08c4, 4, 0x00000000),
    (0x08b0, 4, 0x00000618),
    (0x0e90, 4, 0x01817d78),
    (0x0668, 2, 0x00001000),
    (0x08ac, 4, 0x0ff0fa08),
    (0x08c4, 4, 0x00000000),
    (0x0848, 4, 0x61d0ff8b),
    (0x08ac, 4, 0x0ff0fa08),
    (0x08ac, 4, 0x0ff0fa08),
    (0x08c4, 4, 0x00000000),
    (0x08b0, 4, 0x00000618),
    (0x0c90, 4, 0x01837d78),
    (0x08b0, 4, 0x00000618),
    (0x0e90, 4, 0x01837d78),
    (0x0c24, 4, 0x2c2c2c2c),
    (0x0c28, 4, 0x282a2c2c),
    (0x0c2c, 4, 0x31313131),
    (0x0c30, 4, 0x292b2d2f),
    (0x0c34, 4, 0x2d2d2d2d),
    (0x0c38, 4, 0x27292b2d),
    (0x0c3c, 4, 0x31313131),
    (0x0c40, 4, 0x292b2d2f),
    (0x0c44, 4, 0x2d2d2727),
    (0x0c48, 4, 0x2b2d2d2d),
    (0x0c4c, 4, 0x25252729),
    (0x0c54, 4, 0x0011171f),
    (0x0e24, 4, 0x1e1e1e1e),
    (0x0e28, 4, 0x1a1c1e1e),
    (0x0e2c, 4, 0x23232323),
    (0x0e30, 4, 0x1b1d1f21),
    (0x0e34, 4, 0x1f1f1f1f),
    (0x0e38, 4, 0x191b1d1f),
    (0x0e3c, 4, 0x23232323),
    (0x0e40, 4, 0x1b1d1f21),
    (0x0e44, 4, 0x1f1f1919),
    (0x0e48, 4, 0x1d1f1f1f),
    (0x0e4c, 4, 0x1717191b),
    (0x0e54, 4, 0x00030911),
    (0x09a4, 4, 0x000a0080),
    (0x09a4, 4, 0x00080080),
    (0x0a2c, 4, 0x00900000),
    (0x0a2c, 4, 0x00908000),
    (0x0b58, 4, 0x00000001),
    (0x0b58, 4, 0x00000000),
    (0x08b0, 4, 0x00000642),
    (0x0c90, 4, 0x04238d08),
    (0x09a4, 4, 0x000a0080),
    (0x09a4, 4, 0x00080080),
    (0x0a2c, 4, 0x00900000),
    (0x0a2c, 4, 0x00908000),
    (0x0b58, 4, 0x00000001),
    (0x0b58, 4, 0x00000000),
    (0x08b0, 4, 0x00000642),
    (0x0c1c, 4, 0x39000003),
    (0x0e1c, 4, 0x32e00003),
];

/// The RTL8812AU **5 GHz ch124** channel program (UNII-2C DFS) — golden trace
/// (golden/rtw88-8812au-ch124-5g), same capture/decode as ch36.
static CH124_5G_PROGRAM: RegProgram = &[
    (0x09a4, 4, 0x000a0080),
    (0x09a4, 4, 0x00080080),
    (0x0a2c, 4, 0x00900000),
    (0x0a2c, 4, 0x00908000),
    (0x0b58, 4, 0x00000001),
    (0x0b58, 4, 0x00000000),
    (0x08b0, 4, 0x00000642),
    (0x0c90, 4, 0x04238d08),
    (0x0860, 4, 0x72d5c321),
    (0x08b0, 4, 0x00000618),
    (0x0c90, 4, 0x01807c01),
    (0x08ac, 4, 0x0ff0fa08),
    (0x08ac, 4, 0x0ff0fa08),
    (0x08c4, 4, 0x00000000),
    (0x08b0, 4, 0x00000618),
    (0x0c90, 4, 0x01807c01),
    (0x08b0, 4, 0x00000618),
    (0x0e90, 4, 0x01807c01),
    (0x08ac, 4, 0x0ff0fa08),
    (0x08ac, 4, 0x0ff0fa08),
    (0x08c4, 4, 0x00000000),
    (0x08b0, 4, 0x00000618),
    (0x0e90, 4, 0x01807c01),
    (0x0668, 2, 0x00001000),
    (0x08ac, 4, 0x0ff0fa08),
    (0x08c4, 4, 0x00000000),
    (0x0848, 4, 0x61d0ff8b),
    (0x08ac, 4, 0x0ff0fa08),
    (0x08ac, 4, 0x0ff0fa08),
    (0x08c4, 4, 0x00000000),
    (0x08b0, 4, 0x00000618),
    (0x0c90, 4, 0x01807c01),
    (0x08b0, 4, 0x00000618),
    (0x0e90, 4, 0x01807c01),
    (0x0c20, 4, 0x13131313),
    (0x0c24, 4, 0x20202020),
    (0x0c28, 4, 0x1c1e2020),
    (0x0c2c, 4, 0x20202020),
    (0x0c30, 4, 0x1c1e2020),
    (0x0c34, 4, 0x20202020),
    (0x0c38, 4, 0x1a1c1e20),
    (0x0c3c, 4, 0x20202020),
    (0x0c40, 4, 0x1c1e2020),
    (0x0c44, 4, 0x1e1e181a),
    (0x0c48, 4, 0x1e1e1e1e),
    (0x0c4c, 4, 0x16181a1c),
    (0x0c54, 4, 0x00040a12),
    (0x0e20, 4, 0x14141414),
    (0x0e24, 4, 0x1d1d1d1d),
    (0x0e28, 4, 0x191b1d1d),
    (0x0e2c, 4, 0x1f1f1f1f),
    (0x0e30, 4, 0x1b1d1f1f),
    (0x0e34, 4, 0x1f1f1f1f),
    (0x0e38, 4, 0x191b1d1f),
    (0x0e3c, 4, 0x1f1f1f1f),
    (0x0e40, 4, 0x1b1d1f1f),
    (0x0e44, 4, 0x1d1d1719),
    (0x0e48, 4, 0x1d1d1d1d),
    (0x0e4c, 4, 0x1517191b),
    (0x0e54, 4, 0x00030911),
    (0x0454, 1, 0x00000080),
    (0x0808, 4, 0x3e028233),
    (0x0834, 4, 0x0037a706),
    (0x0830, 4, 0x2eaaaeb8),
    (0x0830, 4, 0x2eaaaeb8),
    (0x082c, 4, 0x002083dd),
    (0x0cb0, 4, 0x54337717),
    (0x0eb0, 4, 0x54337717),
    (0x0cb4, 4, 0x01000077),
    (0x0eb4, 4, 0x01000077),
    (0x0900, 4, 0x00000401),
    (0x080c, 4, 0x12131103),
    (0x0a04, 4, 0x0fff000c),
    (0x0c1c, 4, 0x2d400003),
    (0x0e1c, 4, 0x2d400003),
    (0x0860, 4, 0x6825c321),
    (0x08b0, 4, 0x00000618),
    (0x0c90, 4, 0x01837d01),
    (0x08ac, 4, 0x0ff0fa08),
    (0x08ac, 4, 0x0ff0fa08),
    (0x08c4, 4, 0x00000000),
    (0x08b0, 4, 0x00000618),
    (0x0c90, 4, 0x01817d7c),
    (0x08b0, 4, 0x00000618),
    (0x0e90, 4, 0x01837d01),
    (0x08ac, 4, 0x0ff0fa08),
    (0x08ac, 4, 0x0ff0fa08),
    (0x08c4, 4, 0x00000000),
    (0x08b0, 4, 0x00000618),
    (0x0e90, 4, 0x01817d7c),
    (0x0668, 2, 0x00001000),
    (0x08ac, 4, 0x0ff0fa08),
    (0x08c4, 4, 0x00000000),
    (0x0848, 4, 0x61d0ff8b),
    (0x08ac, 4, 0x0ff0fa08),
    (0x08ac, 4, 0x0ff0fa08),
    (0x08c4, 4, 0x00000000),
    (0x08b0, 4, 0x00000618),
    (0x0c90, 4, 0x01837d7c),
    (0x08b0, 4, 0x00000618),
    (0x0e90, 4, 0x01837d7c),
    (0x0c24, 4, 0x2c2c2c2c),
    (0x0c28, 4, 0x282a2c2c),
    (0x0c2c, 4, 0x31313131),
    (0x0c30, 4, 0x292b2d2f),
    (0x0c34, 4, 0x2d2d2d2d),
    (0x0c38, 4, 0x27292b2d),
    (0x0c3c, 4, 0x31313131),
    (0x0c40, 4, 0x292b2d2f),
    (0x0c44, 4, 0x2d2d2727),
    (0x0c48, 4, 0x2b2d2d2d),
    (0x0c4c, 4, 0x25252729),
    (0x0c54, 4, 0x0011171f),
    (0x0e24, 4, 0x1b1b1b1b),
    (0x0e28, 4, 0x17191b1b),
    (0x0e2c, 4, 0x20202020),
    (0x0e30, 4, 0x181a1c1e),
    (0x0e34, 4, 0x1c1c1c1c),
    (0x0e38, 4, 0x16181a1c),
    (0x0e3c, 4, 0x20202020),
    (0x0e40, 4, 0x181a1c1e),
    (0x0e44, 4, 0x1c1c1616),
    (0x0e48, 4, 0x1a1c1c1c),
    (0x0e4c, 4, 0x14141618),
    (0x0e54, 4, 0x0002060e),
    (0x09a4, 4, 0x000a0080),
    (0x09a4, 4, 0x00080080),
    (0x0a2c, 4, 0x00900000),
    (0x0a2c, 4, 0x00908000),
    (0x0b58, 4, 0x00000001),
    (0x0b58, 4, 0x00000000),
    (0x08b0, 4, 0x00000642),
    (0x0c90, 4, 0x04238d08),
];

/// The RTL8812AU **5 GHz ch128** channel program (UNII-2C DFS) — golden trace
/// (golden/rtw88-8812au-ch128-5g), same capture/decode as ch36.
static CH128_5G_PROGRAM: RegProgram = &[
    (0x09a4, 4, 0x000a0080),
    (0x09a4, 4, 0x00080080),
    (0x0a2c, 4, 0x00900000),
    (0x0a2c, 4, 0x00908000),
    (0x0b58, 4, 0x00000001),
    (0x0b58, 4, 0x00000000),
    (0x08b0, 4, 0x00000642),
    (0x0c90, 4, 0x04238d08),
    (0x0860, 4, 0x72d5c321),
    (0x08b0, 4, 0x00000618),
    (0x0c90, 4, 0x01807c01),
    (0x08ac, 4, 0x0ff0fa08),
    (0x08ac, 4, 0x0ff0fa08),
    (0x08c4, 4, 0x00000000),
    (0x08b0, 4, 0x00000618),
    (0x0c90, 4, 0x01807c01),
    (0x08b0, 4, 0x00000618),
    (0x0e90, 4, 0x01807c01),
    (0x08ac, 4, 0x0ff0fa08),
    (0x08ac, 4, 0x0ff0fa08),
    (0x08c4, 4, 0x00000000),
    (0x08b0, 4, 0x00000618),
    (0x0e90, 4, 0x01807c01),
    (0x0668, 2, 0x00001000),
    (0x08ac, 4, 0x0ff0fa08),
    (0x08c4, 4, 0x00000000),
    (0x0848, 4, 0x61d0ff8b),
    (0x08ac, 4, 0x0ff0fa08),
    (0x08ac, 4, 0x0ff0fa08),
    (0x08c4, 4, 0x00000000),
    (0x08b0, 4, 0x00000618),
    (0x0c90, 4, 0x01807c01),
    (0x08b0, 4, 0x00000618),
    (0x0e90, 4, 0x01807c01),
    (0x0c20, 4, 0x13131313),
    (0x0c24, 4, 0x20202020),
    (0x0c28, 4, 0x1c1e2020),
    (0x0c2c, 4, 0x20202020),
    (0x0c30, 4, 0x1c1e2020),
    (0x0c34, 4, 0x20202020),
    (0x0c38, 4, 0x1a1c1e20),
    (0x0c3c, 4, 0x20202020),
    (0x0c40, 4, 0x1c1e2020),
    (0x0c44, 4, 0x1e1e181a),
    (0x0c48, 4, 0x1e1e1e1e),
    (0x0c4c, 4, 0x16181a1c),
    (0x0c54, 4, 0x00040a12),
    (0x0e20, 4, 0x14141414),
    (0x0e24, 4, 0x1d1d1d1d),
    (0x0e28, 4, 0x191b1d1d),
    (0x0e2c, 4, 0x1f1f1f1f),
    (0x0e30, 4, 0x1b1d1f1f),
    (0x0e34, 4, 0x1f1f1f1f),
    (0x0e38, 4, 0x191b1d1f),
    (0x0e3c, 4, 0x1f1f1f1f),
    (0x0e40, 4, 0x1b1d1f1f),
    (0x0e44, 4, 0x1d1d1719),
    (0x0e48, 4, 0x1d1d1d1d),
    (0x0e4c, 4, 0x1517191b),
    (0x0e54, 4, 0x00030911),
    (0x0454, 1, 0x00000080),
    (0x0808, 4, 0x3e028233),
    (0x0834, 4, 0x0037a706),
    (0x0830, 4, 0x2eaaaeb8),
    (0x0830, 4, 0x2eaaaeb8),
    (0x082c, 4, 0x002083dd),
    (0x0cb0, 4, 0x54337717),
    (0x0eb0, 4, 0x54337717),
    (0x0cb4, 4, 0x01000077),
    (0x0eb4, 4, 0x01000077),
    (0x0900, 4, 0x00000401),
    (0x080c, 4, 0x12131103),
    (0x0a04, 4, 0x0fff000c),
    (0x0c1c, 4, 0x2d400003),
    (0x0e1c, 4, 0x2d400003),
    (0x0860, 4, 0x6825c321),
    (0x08b0, 4, 0x00000618),
    (0x0c90, 4, 0x01837d01),
    (0x08ac, 4, 0x0ff0fa08),
    (0x08ac, 4, 0x0ff0fa08),
    (0x08c4, 4, 0x00000000),
    (0x08b0, 4, 0x00000618),
    (0x0c90, 4, 0x01817d80),
    (0x08b0, 4, 0x00000618),
    (0x0e90, 4, 0x01837d01),
    (0x08ac, 4, 0x0ff0fa08),
    (0x08ac, 4, 0x0ff0fa08),
    (0x08c4, 4, 0x00000000),
    (0x08b0, 4, 0x00000618),
    (0x0e90, 4, 0x01817d80),
    (0x0668, 2, 0x00001000),
    (0x08ac, 4, 0x0ff0fa08),
    (0x08c4, 4, 0x00000000),
    (0x0848, 4, 0x61d0ff8b),
    (0x08ac, 4, 0x0ff0fa08),
    (0x08ac, 4, 0x0ff0fa08),
    (0x08c4, 4, 0x00000000),
    (0x08b0, 4, 0x00000618),
    (0x0c90, 4, 0x01837d80),
    (0x08b0, 4, 0x00000618),
    (0x0e90, 4, 0x01837d80),
    (0x0c24, 4, 0x2c2c2c2c),
    (0x0c28, 4, 0x282a2c2c),
    (0x0c2c, 4, 0x31313131),
    (0x0c30, 4, 0x292b2d2f),
    (0x0c34, 4, 0x2d2d2d2d),
    (0x0c38, 4, 0x27292b2d),
    (0x0c3c, 4, 0x31313131),
    (0x0c40, 4, 0x292b2d2f),
    (0x0c44, 4, 0x2d2d2727),
    (0x0c48, 4, 0x2b2d2d2d),
    (0x0c4c, 4, 0x25252729),
    (0x0c54, 4, 0x0011171f),
    (0x0e24, 4, 0x1b1b1b1b),
    (0x0e28, 4, 0x17191b1b),
    (0x0e2c, 4, 0x20202020),
    (0x0e30, 4, 0x181a1c1e),
    (0x0e34, 4, 0x1c1c1c1c),
    (0x0e38, 4, 0x16181a1c),
    (0x0e3c, 4, 0x20202020),
    (0x0e40, 4, 0x181a1c1e),
    (0x0e44, 4, 0x1c1c1616),
    (0x0e48, 4, 0x1a1c1c1c),
    (0x0e4c, 4, 0x14141618),
    (0x0e54, 4, 0x0002060e),
    (0x09a4, 4, 0x000a0080),
    (0x09a4, 4, 0x00080080),
    (0x0a2c, 4, 0x00900000),
    (0x0a2c, 4, 0x00908000),
    (0x0b58, 4, 0x00000001),
    (0x0b58, 4, 0x00000000),
    (0x08b0, 4, 0x00000642),
    (0x0c90, 4, 0x04238908),
];

/// The RTL8812AU **5 GHz ch136** channel program (UNII-2C DFS) — golden trace
/// (golden/rtw88-8812au-ch136-5g), same capture/decode as ch36.
static CH136_5G_PROGRAM: RegProgram = &[
    (0x0860, 4, 0x72d5c321),
    (0x08b0, 4, 0x00000618),
    (0x0c90, 4, 0x01807c01),
    (0x08ac, 4, 0x0ff0fa08),
    (0x08ac, 4, 0x0ff0fa08),
    (0x08c4, 4, 0x00000000),
    (0x08b0, 4, 0x00000618),
    (0x0c90, 4, 0x01807c01),
    (0x08b0, 4, 0x00000618),
    (0x0e90, 4, 0x01807c01),
    (0x08ac, 4, 0x0ff0fa08),
    (0x08ac, 4, 0x0ff0fa08),
    (0x08c4, 4, 0x00000000),
    (0x08b0, 4, 0x00000618),
    (0x0e90, 4, 0x01807c01),
    (0x0668, 2, 0x00001000),
    (0x08ac, 4, 0x0ff0fa08),
    (0x08c4, 4, 0x00000000),
    (0x0848, 4, 0x61d0ff8b),
    (0x08ac, 4, 0x0ff0fa08),
    (0x08ac, 4, 0x0ff0fa08),
    (0x08c4, 4, 0x00000000),
    (0x08b0, 4, 0x00000618),
    (0x0c90, 4, 0x01807c01),
    (0x08b0, 4, 0x00000618),
    (0x0e90, 4, 0x01807c01),
    (0x0c20, 4, 0x13131313),
    (0x0c24, 4, 0x20202020),
    (0x0c28, 4, 0x1c1e2020),
    (0x0c2c, 4, 0x20202020),
    (0x0c30, 4, 0x1c1e2020),
    (0x0c34, 4, 0x20202020),
    (0x0c38, 4, 0x1a1c1e20),
    (0x0c3c, 4, 0x20202020),
    (0x0c40, 4, 0x1c1e2020),
    (0x0c44, 4, 0x1e1e181a),
    (0x0c48, 4, 0x1e1e1e1e),
    (0x0c4c, 4, 0x16181a1c),
    (0x0c54, 4, 0x00040a12),
    (0x0e20, 4, 0x14141414),
    (0x0e24, 4, 0x1d1d1d1d),
    (0x0e28, 4, 0x191b1d1d),
    (0x0e2c, 4, 0x1f1f1f1f),
    (0x0e30, 4, 0x1b1d1f1f),
    (0x0e34, 4, 0x1f1f1f1f),
    (0x0e38, 4, 0x191b1d1f),
    (0x0e3c, 4, 0x1f1f1f1f),
    (0x0e40, 4, 0x1b1d1f1f),
    (0x0e44, 4, 0x1d1d1719),
    (0x0e48, 4, 0x1d1d1d1d),
    (0x0e4c, 4, 0x1517191b),
    (0x0e54, 4, 0x00030911),
    (0x0454, 1, 0x00000080),
    (0x0808, 4, 0x3e028233),
    (0x0834, 4, 0x0037a706),
    (0x0830, 4, 0x2eaaaeb8),
    (0x0830, 4, 0x2eaaaeb8),
    (0x082c, 4, 0x002083dd),
    (0x0cb0, 4, 0x54337717),
    (0x0eb0, 4, 0x54337717),
    (0x0cb4, 4, 0x01000077),
    (0x0eb4, 4, 0x01000077),
    (0x0900, 4, 0x00000401),
    (0x080c, 4, 0x12131103),
    (0x0a04, 4, 0x0fff000c),
    (0x0c1c, 4, 0x2d400003),
    (0x0e1c, 4, 0x2d400003),
    (0x0860, 4, 0x6825c321),
    (0x08b0, 4, 0x00000618),
    (0x0c90, 4, 0x01837d01),
    (0x08ac, 4, 0x0ff0fa08),
    (0x08ac, 4, 0x0ff0fa08),
    (0x08c4, 4, 0x00000000),
    (0x08b0, 4, 0x00000618),
    (0x0c90, 4, 0x01817d88),
    (0x08b0, 4, 0x00000618),
    (0x0e90, 4, 0x01837d01),
    (0x08ac, 4, 0x0ff0fa08),
    (0x08ac, 4, 0x0ff0fa08),
    (0x08c4, 4, 0x00000000),
    (0x08b0, 4, 0x00000618),
    (0x0e90, 4, 0x01817d88),
    (0x0668, 2, 0x00001000),
    (0x08ac, 4, 0x0ff0fa08),
    (0x08c4, 4, 0x00000000),
    (0x0848, 4, 0x61d0ff8b),
    (0x08ac, 4, 0x0ff0fa08),
    (0x08ac, 4, 0x0ff0fa08),
    (0x08c4, 4, 0x00000000),
    (0x08b0, 4, 0x00000618),
    (0x0c90, 4, 0x01837d88),
    (0x08b0, 4, 0x00000618),
    (0x0e90, 4, 0x01837d88),
    (0x0c24, 4, 0x2c2c2c2c),
    (0x0c28, 4, 0x2a2c2c2c),
    (0x0c2c, 4, 0x31313131),
    (0x0c30, 4, 0x2b2d2f31),
    (0x0c34, 4, 0x2d2d2d2d),
    (0x0c38, 4, 0x292b2d2d),
    (0x0c3c, 4, 0x31313131),
    (0x0c40, 4, 0x2b2d2f31),
    (0x0c44, 4, 0x2d2d2929),
    (0x0c48, 4, 0x2d2d2d2d),
    (0x0c4c, 4, 0x2727292b),
    (0x0c54, 4, 0x00131921),
    (0x0e24, 4, 0x19191919),
    (0x0e28, 4, 0x17191919),
    (0x0e2c, 4, 0x1e1e1e1e),
    (0x0e30, 4, 0x181a1c1e),
    (0x0e34, 4, 0x1a1a1a1a),
    (0x0e38, 4, 0x16181a1a),
    (0x0e3c, 4, 0x1e1e1e1e),
    (0x0e40, 4, 0x181a1c1e),
    (0x0e44, 4, 0x1a1a1616),
    (0x0e48, 4, 0x1a1a1a1a),
    (0x0e4c, 4, 0x14141618),
    (0x0e54, 4, 0x0002060e),
    (0x09a4, 4, 0x000a0080),
    (0x09a4, 4, 0x00080080),
    (0x0a2c, 4, 0x00900000),
    (0x0a2c, 4, 0x00908000),
    (0x0b58, 4, 0x00000001),
    (0x0b58, 4, 0x00000000),
    (0x08b0, 4, 0x00000642),
    (0x0c90, 4, 0x04238d08),
    (0x09a4, 4, 0x000a0080),
    (0x09a4, 4, 0x00080080),
    (0x0a2c, 4, 0x00900000),
    (0x0a2c, 4, 0x00908000),
    (0x0b58, 4, 0x00000001),
    (0x0b58, 4, 0x00000000),
    (0x08b0, 4, 0x00000642),
    (0x0c1c, 4, 0x39000003),
    (0x0e1c, 4, 0x32e00003),
];

/// The RTL8812AU **5 GHz ch144** channel program (UNII-2C DFS) — golden trace
/// (golden/rtw88-8812au-ch144-5g), same capture/decode as ch36.
static CH144_5G_PROGRAM: RegProgram = &[
    (0x09a4, 4, 0x000a0080),
    (0x09a4, 4, 0x00080080),
    (0x0a2c, 4, 0x00900000),
    (0x0a2c, 4, 0x00908000),
    (0x0b58, 4, 0x00000001),
    (0x0b58, 4, 0x00000000),
    (0x08b0, 4, 0x00000642),
    (0x0c90, 4, 0x04238d08),
    (0x0860, 4, 0x72d5c321),
    (0x08b0, 4, 0x00000618),
    (0x0c90, 4, 0x01807c01),
    (0x08ac, 4, 0x0ff0fa08),
    (0x08ac, 4, 0x0ff0fa08),
    (0x08c4, 4, 0x00000000),
    (0x08b0, 4, 0x00000618),
    (0x0c90, 4, 0x01807c01),
    (0x08b0, 4, 0x00000618),
    (0x0e90, 4, 0x01807c01),
    (0x08ac, 4, 0x0ff0fa08),
    (0x08ac, 4, 0x0ff0fa08),
    (0x08c4, 4, 0x00000000),
    (0x08b0, 4, 0x00000618),
    (0x0e90, 4, 0x01807c01),
    (0x0668, 2, 0x00001000),
    (0x08ac, 4, 0x0ff0fa08),
    (0x08c4, 4, 0x00000000),
    (0x0848, 4, 0x61d0ff8b),
    (0x08ac, 4, 0x0ff0fa08),
    (0x08ac, 4, 0x0ff0fa08),
    (0x08c4, 4, 0x00000000),
    (0x08b0, 4, 0x00000618),
    (0x0c90, 4, 0x01807c01),
    (0x08b0, 4, 0x00000618),
    (0x0e90, 4, 0x01807c01),
    (0x0c20, 4, 0x13131313),
    (0x0c24, 4, 0x20202020),
    (0x0c28, 4, 0x1c1e2020),
    (0x0c2c, 4, 0x20202020),
    (0x0c30, 4, 0x1c1e2020),
    (0x0c34, 4, 0x20202020),
    (0x0c38, 4, 0x1a1c1e20),
    (0x0c3c, 4, 0x20202020),
    (0x0c40, 4, 0x1c1e2020),
    (0x0c44, 4, 0x1e1e181a),
    (0x0c48, 4, 0x1e1e1e1e),
    (0x0c4c, 4, 0x16181a1c),
    (0x0c54, 4, 0x00040a12),
    (0x0e20, 4, 0x14141414),
    (0x0e24, 4, 0x1d1d1d1d),
    (0x0e28, 4, 0x191b1d1d),
    (0x0e2c, 4, 0x1f1f1f1f),
    (0x0e30, 4, 0x1b1d1f1f),
    (0x0e34, 4, 0x1f1f1f1f),
    (0x0e38, 4, 0x191b1d1f),
    (0x0e3c, 4, 0x1f1f1f1f),
    (0x0e40, 4, 0x1b1d1f1f),
    (0x0e44, 4, 0x1d1d1719),
    (0x0e48, 4, 0x1d1d1d1d),
    (0x0e4c, 4, 0x1517191b),
    (0x0e54, 4, 0x00030911),
    (0x0454, 1, 0x00000080),
    (0x0808, 4, 0x3e028233),
    (0x0834, 4, 0x0037a706),
    (0x0830, 4, 0x2eaaaeb8),
    (0x0830, 4, 0x2eaaaeb8),
    (0x082c, 4, 0x002083dd),
    (0x0cb0, 4, 0x54337717),
    (0x0eb0, 4, 0x54337717),
    (0x0cb4, 4, 0x01000077),
    (0x0eb4, 4, 0x01000077),
    (0x0900, 4, 0x00000401),
    (0x080c, 4, 0x12131103),
    (0x0a04, 4, 0x0fff000c),
    (0x0c1c, 4, 0x2d400003),
    (0x0e1c, 4, 0x2d400003),
    (0x0860, 4, 0x6825c321),
    (0x08b0, 4, 0x00000618),
    (0x0c90, 4, 0x01857d01),
    (0x08ac, 4, 0x0ff0fa08),
    (0x08ac, 4, 0x0ff0fa08),
    (0x08c4, 4, 0x00000000),
    (0x08b0, 4, 0x00000618),
    (0x0c90, 4, 0x01817d90),
    (0x08b0, 4, 0x00000618),
    (0x0e90, 4, 0x01857d01),
    (0x08ac, 4, 0x0ff0fa08),
    (0x08ac, 4, 0x0ff0fa08),
    (0x08c4, 4, 0x00000000),
    (0x08b0, 4, 0x00000618),
    (0x0e90, 4, 0x01817d90),
    (0x0668, 2, 0x00001000),
    (0x08ac, 4, 0x0ff0fa08),
    (0x08c4, 4, 0x00000000),
    (0x0848, 4, 0x61d0ff8b),
    (0x08ac, 4, 0x0ff0fa08),
    (0x08ac, 4, 0x0ff0fa08),
    (0x08c4, 4, 0x00000000),
    (0x08b0, 4, 0x00000618),
    (0x0c90, 4, 0x01857d90),
    (0x08b0, 4, 0x00000618),
    (0x0e90, 4, 0x01857d90),
    (0x0c24, 4, 0x2f2f2f2f),
    (0x0c28, 4, 0x2b2d2f2f),
    (0x0c2c, 4, 0x34343434),
    (0x0c30, 4, 0x2c2e3032),
    (0x0c34, 4, 0x32323232),
    (0x0c38, 4, 0x2a2c2e30),
    (0x0c3c, 4, 0x34343434),
    (0x0c40, 4, 0x2c2e3032),
    (0x0c44, 4, 0x32322a2a),
    (0x0c48, 4, 0x2e303232),
    (0x0c4c, 4, 0x28282a2c),
    (0x0c54, 4, 0x00141a22),
    (0x0e24, 4, 0x1b1b1b1b),
    (0x0e28, 4, 0x17191b1b),
    (0x0e2c, 4, 0x20202020),
    (0x0e30, 4, 0x181a1c1e),
    (0x0e34, 4, 0x1e1e1e1e),
    (0x0e38, 4, 0x16181a1c),
    (0x0e3c, 4, 0x20202020),
    (0x0e40, 4, 0x181a1c1e),
    (0x0e44, 4, 0x1e1e1616),
    (0x0e48, 4, 0x1a1c1e1e),
    (0x0e4c, 4, 0x14141618),
    (0x0e54, 4, 0x0002060e),
    (0x09a4, 4, 0x000a0080),
    (0x09a4, 4, 0x00080080),
    (0x0a2c, 4, 0x00900000),
    (0x0a2c, 4, 0x00908000),
    (0x0b58, 4, 0x00000001),
    (0x0b58, 4, 0x00000000),
    (0x08b0, 4, 0x00000642),
    (0x0c90, 4, 0x04238908),
];

/// Per-channel 5 GHz programs, keyed by channel — extend by capturing a usbmon golden trace on a
/// new 5 GHz channel (see golden/). Covers both sub-bands: UNII-1 (ch36/44) + UNII-3 (ch149/157).
static PROGS_5G: &[(u8, RegProgram)] = &[
    (36, CH36_5G_PROGRAM),
    (40, CH40_5G_PROGRAM),
    (44, CH44_5G_PROGRAM),
    (48, CH48_5G_PROGRAM),
    (52, CH52_5G_PROGRAM),
    (56, CH56_5G_PROGRAM),
    (60, CH60_5G_PROGRAM),
    (64, CH64_5G_PROGRAM),
    (100, CH100_5G_PROGRAM),
    (104, CH104_5G_PROGRAM),
    (108, CH108_5G_PROGRAM),
    (112, CH112_5G_PROGRAM),
    (116, CH116_5G_PROGRAM),
    (120, CH120_5G_PROGRAM),
    (124, CH124_5G_PROGRAM),
    (128, CH128_5G_PROGRAM),
    (132, CH132_5G_PROGRAM),
    (136, CH136_5G_PROGRAM),
    (140, CH140_5G_PROGRAM),
    (144, CH144_5G_PROGRAM),
    (149, CH149_5G_PROGRAM),
    (153, CH153_5G_PROGRAM),
    (157, CH157_5G_PROGRAM),
    (161, CH161_5G_PROGRAM),
    (165, CH165_5G_PROGRAM),
];

/// Management-queue select (`QSLT_MGNT`) + its rate-adaptation group
/// (`RATEID_IDX_G`, the OFDM/11g table).
const QSLT_MGNT: u32 = 0x12;

/// Best-effort **data** queue select. Data frames must not ride the management
/// queue: `QSLT_MGNT` maps to the HIGH queue, whose reserved pool is `HPQ` (16
/// pages in [`RQPN_3EP`]) and is sized for small management frames. See
/// [`Rtl8812auBackend::tx_buffer`].
const QSLT_BE: u32 = 0x0;

/// 802.11 frame type 2 = Data (frame-control byte 0, bits 3:2).
fn is_data_frame(fc0: u8) -> bool {
    (fc0 >> 2) & 0x03 == 2
}
const RATEID_IDX_G: u32 = 7;
/// Bulk-transfer timeouts (injection is fire-and-forget; capture polls).
const TX_TIMEOUT: Duration = Duration::from_millis(100);
const RX_TIMEOUT: Duration = Duration::from_millis(200);
/// Minimum plausible 802.11 management/data frame (24-byte header).
const DOT11_HDR_LEN: usize = 24;

/// Identified silicon details from [`Rtl8812auBackend::chip_info`].
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct ChipInfo {
    /// Raw `REG_SYS_CFG` (`0xF0`).
    pub sys_cfg: u32,
    /// Raw `REG_SYS_CFG1` (`0xFC`).
    pub sys_cfg1: u32,
    /// Cut/version, `REG_SYS_CFG[15:12]` (0 = A-cut, 1 = B-cut, …).
    pub cut: u8,
    /// True if this looks like a test chip (`REG_SYS_CFG` RTL-ID bit set).
    pub test_chip: bool,
}

impl ChipInfo {
    /// True when the registers read back as a live, sane device (not the wedged
    /// all-ones pattern).
    pub fn responsive(&self) -> bool {
        self.sys_cfg != 0xFFFF_FFFF && self.sys_cfg != 0
    }
}

/// One frame-free PHY-sensing sample (#30) — the baseband's own view of the
/// channel, read without decoding any frame. See [`Rtl8812auBackend::read_phy_sense`].
/// The counters are free-running 16-bit accumulators; take two samples and diff
/// ([`PhySense::delta`]) to get per-window rates. Field labels are phydm's,
/// validated on-chip.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct PhySense {
    /// AGC initial gain, path A (`0xC50[6:0]`) — energy/noise-floor proxy, higher
    /// = more energy present. Instantaneous (a level, not a counter).
    pub igi_a: u8,
    /// AGC initial gain, path B (`0xE50[6:0]`).
    pub igi_b: u8,
    /// `REG_RXERR_RPT[15:0]` (`0x0664`) — the validated occupancy counter; tracks
    /// channel-frame activity ~1:1, read without host-side decode. Free-running.
    pub rx_activity: u16,
}

impl PhySense {
    /// Counter delta from `earlier` to `self`, wrapped at 16 bits (the counter is
    /// a u16 accumulator and rolls over). IGI fields are copied from `self` (they
    /// are levels, not counts).
    pub fn delta(&self, earlier: &PhySense) -> PhySense {
        PhySense {
            igi_a: self.igi_a,
            igi_b: self.igi_b,
            rx_activity: self.rx_activity.wrapping_sub(earlier.rx_activity),
        }
    }
}

/// A failed vendor-request, named.
///
/// "rtl8812au usb: Operation timed out" says nothing about *where* the device
/// stopped answering, which is the only interesting part when it wedges. Report
/// the direction, the register, and how many control transfers had already
/// succeeded — the register localises the step in the bring-up, and the count
/// separates "died immediately" from "died after N ops".
fn ctrl_err(e: rusb::Error, dir: &str, addr: u16, len: usize, seq: u64) -> FaceError {
    FaceError::Io(io::Error::other(format!(
        "rtl8812au usb {dir}{}({addr:#06x}) failed after {seq} ok control transfers: {e}",
        len * 8
    )))
}

fn usb_err(e: rusb::Error) -> FaceError {
    FaceError::Io(io::Error::other(format!("rtl8812au usb: {e}")))
}

fn init_err(what: impl Into<String>) -> FaceError {
    FaceError::Io(io::Error::other(what.into()))
}

// ── EFUSE TX-power calibration ────────────────────────────────────────────────
// Port of OpenIPC devourer `src/jaguar1/EepromManager.cpp` (`ReadEFuseByte`,
// `Hal_EfuseReadEFuse8812A`, `LoadTxPowerInfo`, `GetTxPowerIndexBase`), itself a
// port of mainline Realtek `hal_com_phycfg.c` / `rtw_efuse.c`. Turns the flat
// uncalibrated TXAGC index into one referenced to THIS adapter's fused full-power
// point per channel/rate — the base a dB-accurate power knob needs.

const REG_EFUSE_CTRL: u16 = 0x0030; // E-Fuse control (addr in +1/+2, flag in +3, data via read32)
const REG_EFUSE_TEST: u16 = 0x0034;
const REG_EFUSE_BURN_GNT_8812: u16 = 0x00cf;
const EFUSE_ACCESS_ON_JAGUAR: u8 = 0x69;
const EFUSE_ACCESS_OFF_JAGUAR: u8 = 0x00;
const FEN_ELDR: u16 = 1 << 12; // REG_SYS_FUNC_EN eldr reset
const LOADER_CLK_EN: u16 = 1 << 5; // REG_SYS_CLKR
const ANA8M: u16 = 1 << 1; // REG_SYS_CLKR
const REG_SYS_CLKR: u16 = 0x0008;
const EFUSE_MAP_LEN_JAGUAR: usize = 512;
const EFUSE_REAL_CONTENT_LEN_JAGUAR: u16 = 512;
const EFUSE_MAX_SECTION_JAGUAR: usize = 64;
const EFUSE_MAX_WORD_UNIT: usize = 4;
const PG_TXPWR_SADDR: usize = 0x10; // EFUSE PG tx-power block start
const TXAGC_MAX: u8 = 63;

/// 5 GHz center channels the per-group base table is scattered across (mirrors
/// devourer `kCenterCh5gAll`).
const CENTER_CH_5G: [u8; 65] = [
    15, 16, 17, 18, 20, 24, 28, 32, 36, 38, 40, 42, 44, 46, 48, 52, 54, 56, 58, 60, 62, 64, 68, 72,
    76, 80, 84, 88, 92, 96, 100, 102, 104, 106, 108, 110, 112, 116, 118, 120, 122, 124, 126, 128,
    132, 134, 136, 138, 140, 142, 144, 149, 151, 153, 155, 157, 159, 161, 165, 167, 169, 171, 173,
    175, 177,
];

/// Rate-group classifier (port of Realtek `rtw_get_ch_group`). Returns
/// `Some((band, group, cck_group))` — band 0 = 2.4 GHz, 1 = 5 GHz — or `None` for
/// an invalid channel.
fn classify_channel(ch: u8) -> Option<(u8, u8, u8)> {
    if ch <= 14 {
        let gp = match ch {
            1..=2 => 0,
            3..=5 => 1,
            6..=8 => 2,
            9..=11 => 3,
            12..=14 => 4,
            _ => return None,
        };
        let cck_gp = if ch == 14 { 5 } else { gp };
        return Some((0, gp, cck_gp));
    }
    let gp = match ch {
        15..=42 => 0,
        44..=48 => 1,
        50..=58 => 2,
        60..=98 => 3,
        100..=106 => 4,
        108..=114 => 5,
        116..=122 => 6,
        124..=130 => 7,
        132..=138 => 8,
        140..=144 => 9,
        149..=155 => 10,
        157..=161 => 11,
        165..=171 => 12,
        173..=253 => 13,
        _ => return None,
    };
    Some((1, gp, 0))
}

/// Sign-extend the high nibble of a PG diff byte (`pg_msb_diff`).
fn pg_msb_diff(v: u8) -> i8 {
    let n = (v >> 4) & 0x0f;
    if n & 0x08 != 0 {
        (n | 0xf0) as i8
    } else {
        n as i8
    }
}
/// Sign-extend the low nibble of a PG diff byte (`pg_lsb_diff`).
fn pg_lsb_diff(v: u8) -> i8 {
    let n = v & 0x0f;
    if n & 0x08 != 0 {
        (n | 0xf0) as i8
    } else {
        n as i8
    }
}

/// MGN_* rate classifiers (from Realtek `phydm_types.h`).
fn is_cck(r: u8) -> bool {
    matches!(r, 0x02 | 0x04 | 0x0b | 0x16)
}
fn is_ofdm(r: u8) -> bool {
    matches!(r, 0x0c | 0x12 | 0x18 | 0x24 | 0x30 | 0x48 | 0x60 | 0x6c)
}

/// This adapter's EFUSE-programmed TX-power calibration (base index per channel +
/// signed per-Ntx bandwidth/modulation diffs), parsed by
/// [`Rtl8812auBackend::load_tx_power_info`]. Indices are 0–63 TXAGC steps (0.5 dB).
#[derive(Clone)]
struct TxPowerInfo {
    cck_base_2g: [[u8; 14]; 2],  // [path][ch_idx 0..13]
    bw40_base_2g: [[u8; 14]; 2], // [path][ch_idx]
    bw40_base_5g: [[u8; 65]; 2], // [path][5g ch_idx]
    ofdm_2g_diff: [[i8; 4]; 2],  // [path][ntx]
    cck_2g_diff: [[i8; 4]; 2],
    bw20_2g_diff: [[i8; 4]; 2],
    bw40_2g_diff: [[i8; 4]; 2],
    ofdm_5g_diff: [[i8; 4]; 2],
    bw20_5g_diff: [[i8; 4]; 2],
    bw40_5g_diff: [[i8; 4]; 2],
    bw80_5g_diff: [[i8; 4]; 2],
}

impl TxPowerInfo {
    fn zeroed() -> Self {
        TxPowerInfo {
            cck_base_2g: [[0; 14]; 2],
            bw40_base_2g: [[0; 14]; 2],
            bw40_base_5g: [[0; 65]; 2],
            ofdm_2g_diff: [[0; 4]; 2],
            cck_2g_diff: [[0; 4]; 2],
            bw20_2g_diff: [[0; 4]; 2],
            bw40_2g_diff: [[0; 4]; 2],
            ofdm_5g_diff: [[0; 4]; 2],
            bw20_5g_diff: [[0; 4]; 2],
            bw40_5g_diff: [[0; 4]; 2],
            bw80_5g_diff: [[0; 4]; 2],
        }
    }

    /// The calibrated TXAGC base index (0–63) for `rate` (MGN_*) on `channel` at
    /// `path` (0=A,1=B), `ntx_idx` (0-based extra-stream count), `bw` (0=20,1=40,
    /// 2=80). Port of `GetTxPowerIndexBase`; the by-rate overlay is a no-op in the
    /// USB reference build (`CONFIG_TXPWR_BY_RATE_EN=n`).
    fn index_base(&self, path: usize, rate: u8, ntx_idx: u8, bw: u8, channel: u8) -> u8 {
        let path = path.min(1);
        let Some((band, group, cck_group)) = classify_channel(channel) else {
            return 0;
        };
        let mut p: i32 = 0;
        if band == 0 {
            let ch_idx = (channel as usize).saturating_sub(1).min(13);
            let _ = group; // group indexes the per-group source, already scattered per-channel
            if is_cck(rate) {
                p = self.cck_base_2g[path][ch_idx] as i32;
                p += self.cck_2g_diff[path][0] as i32;
                for t in 1..=ntx_idx.min(3) as usize {
                    p += self.cck_2g_diff[path][t] as i32;
                }
                return p.clamp(0, TXAGC_MAX as i32) as u8;
            }
            let _ = cck_group;
            p = self.bw40_base_2g[path][ch_idx] as i32;
            if is_ofdm(rate) {
                p += self.ofdm_2g_diff[path][0] as i32;
                for t in 1..=ntx_idx.min(3) as usize {
                    p += self.ofdm_2g_diff[path][t] as i32;
                }
                return p.clamp(0, TXAGC_MAX as i32) as u8;
            }
            // MCS/VHT: cumulative BW20/BW40 diffs by stream count.
            let diff = if bw == 0 {
                &self.bw20_2g_diff[path]
            } else {
                &self.bw40_2g_diff[path]
            };
            p += mcs_diff_sum(rate, diff);
        } else {
            if rate < 0x0c {
                return 0;
            }
            let ch_idx = self.ch_idx_5g(channel);
            p = self.bw40_base_5g[path][ch_idx] as i32;
            if is_ofdm(rate) {
                p += self.ofdm_5g_diff[path][0] as i32;
                for t in 1..=ntx_idx.min(3) as usize {
                    p += self.ofdm_5g_diff[path][t] as i32;
                }
                return p.clamp(0, TXAGC_MAX as i32) as u8;
            }
            let diff = match bw {
                0 => &self.bw20_5g_diff[path],
                1 => &self.bw40_5g_diff[path],
                _ => &self.bw80_5g_diff[path],
            };
            p += mcs_diff_sum(rate, diff);
        }
        p.clamp(0, TXAGC_MAX as i32) as u8
    }

    /// Nearest 5 GHz center-channel index (exact hit wins).
    fn ch_idx_5g(&self, channel: u8) -> usize {
        let mut best = 0usize;
        let mut best_d = i32::MAX;
        for (i, &c) in CENTER_CH_5G.iter().enumerate() {
            let d = (c as i32 - channel as i32).abs();
            if d < best_d {
                best_d = d;
                best = i;
            }
            if d == 0 {
                break;
            }
        }
        best
    }
}

/// Cumulative MCS/VHT stream diffs (port of the `ge_1s..ge_4s` accumulation).
fn mcs_diff_sum(r: u8, diff: &[i8; 4]) -> i32 {
    let mcs0_7 = (0x80..=0x87).contains(&r);
    let mcs8_15 = (0x88..=0x8f).contains(&r);
    let mcs16_23 = (0x90..=0x97).contains(&r);
    let mcs24_31 = (0x98..=0x9f).contains(&r);
    let vht1 = (0xa0..=0xa9).contains(&r);
    let vht2 = (0xaa..=0xb3).contains(&r);
    let vht3 = (0xb4..=0xbd).contains(&r);
    let vht4 = (0xbe..=0xc7).contains(&r);
    let ge_1s = mcs0_7 || mcs8_15 || mcs16_23 || mcs24_31 || vht1 || vht2 || vht3 || vht4;
    let ge_2s = mcs8_15 || mcs16_23 || mcs24_31 || vht2 || vht3 || vht4;
    let ge_3s = mcs16_23 || mcs24_31 || vht3 || vht4;
    let ge_4s = mcs24_31 || vht4;
    let mut s = 0i32;
    if ge_1s {
        s += diff[0] as i32;
    }
    if ge_2s {
        s += diff[1] as i32;
    }
    if ge_3s {
        s += diff[2] as i32;
    }
    if ge_4s {
        s += diff[3] as i32;
    }
    s
}

/// A userspace RTL8812AU radio. Open with [`open`](Self::open); the handle keeps
/// interface 0 claimed for the backend's lifetime.
pub struct Rtl8812auBackend {
    /// As-found four-AC EDCA state, so a contention posture change can be undone.
    /// See [`crate::realtek_contention`] for why this is a knob and not a side effect.
    pub(crate) ndr_contention: crate::realtek_contention::RtlEdcaSaved,
    /// The deliberate RX detection floor as an IGI index (`floor_dBm = IGI - 110`). Re-asserted
    /// after every channel program, because the captured per-channel traces each leave a different
    /// value and ten of them leave none at all.
    rx_floor_igi: std::sync::atomic::AtomicU8,
    /// Round-robin index over the four H2C mailboxes, matching the kernel's observed rotation.
    h2c_box: std::sync::atomic::AtomicU32,
    /// CCX transmit reports seen (C2H id 0x03), and how many of those the MAC reported delivered.
    /// See [`Rtl8812auBackend::read_tx_counters`] for what these do and do NOT mean.
    tx_rpt_seen: std::sync::atomic::AtomicU32,
    tx_rpt_ok: std::sync::atomic::AtomicU32,
    tx_rpt_retries: std::sync::atomic::AtomicU32,
    /// As-found `REG_TX_RPT_CTRL`/`+1`/`TIME`, packed; `u32::MAX` = nothing saved.
    tx_rpt_saved: std::sync::atomic::AtomicU32,

    handle: Arc<DeviceHandle<Context>>,
    /// Bulk OUT endpoint (frame injection).
    bulk_out: u8,
    /// Bulk IN endpoint (frame capture).
    bulk_in: u8,
    /// The product id we matched (diagnostics).
    pid: u16,
    /// On-air frame format: `Raw80211` (NAN — the payload is a complete 802.11 frame) or a
    /// `RawNdn { ethertype }` (named-time — wrap/extract the NDN payload). Set via
    /// [`with_format`](Self::with_format); defaults to `Raw80211` for the NAN path.
    format: FrameFormat,
    /// Shared async-URB RX pipeline: the queue background pump threads fill and `recv_frame`
    /// drains (the 8812A stacks several 802.11 packets per USB transfer). See [`crate::rx_pump`].
    rx_pump: crate::rx_pump::RxPumpState,
    /// Vendor-request control transfers issued so far — reported when one fails,
    /// to separate "the device died immediately" from "it died after N ops".
    ctrl_ops: std::sync::atomic::AtomicU64,
    /// Clock domain of this device's free-run RX-stamp TSF (per-device, from the USB bus/address).
    tsf_domain: ClockDomainId,
    /// Latest **mesh** common-view observation (#75) — a locally-administered neighbour's HW-TSF-stamped
    /// timing beacon + our RXTSFL + its advertised belief. Lets an 8812au (e.g. an Alfa) be a leaf node
    /// in the network-time tree (receive-only). See `mesh_common_view`.
    mesh_cv: std::sync::Mutex<Option<ndn_radio_hal::MeshCv>>,
    /// Current channel (set by [`set_channel`](Self::set_channel)). The EFUSE-calibrated
    /// TX-power base is per-channel, so `set_tx_power` needs it. 0 = not yet tuned.
    cur_channel: std::sync::atomic::AtomicU8,
    /// Parsed EFUSE per-rate TX-power calibration ([`TxPowerInfo`]), `None` until
    /// [`load_tx_power_info`](Self::load_tx_power_info) reads the fuse. When present,
    /// `set_tx_power` folds the requested backoff onto this adapter's *fused* full-power
    /// base per channel/rate instead of writing a flat uncalibrated index.
    tx_power_info: std::sync::Mutex<Option<TxPowerInfo>>,
    /// Saved `0x838[3:0]` (the OFDM CCA-mode nibble) before [`set_cca_ignore`](Self::set_cca_ignore)
    /// forced it off, so the knob can restore the bring-up value. `0xffff` = nothing saved yet.
    cca_saved: std::sync::atomic::AtomicU16,
    /// Saved `REG_EDCA_BE_PARAM` (0x0508) before [`set_cca_ignore`](Self::set_cca_ignore) zeroed the
    /// contention window (aggressive-EDCA blast). `0xffff_ffff` = nothing saved yet.
    /// Current transmit rate as **state** — the exact MCS every `inject` uses once the
    /// control plane has decided one via [`FrameIo::set_rate`]; `None` ⇒ fall back to the
    /// legacy 6 Mbps default (or the `NDN_RADIO_TX_RATE` override). Mirrors the 88xx
    /// backend's "rate as bearer state" seam so the cognition plane's per-frame MCS
    /// decision is no longer silently discarded on this chip.
    cur_mcs: std::sync::Mutex<Option<crate::McsDescriptor>>,
}

impl Drop for Rtl8812auBackend {
    /// **Leave the device in a clean, re-openable state on a normal exit** — the fix for the
    /// "hears zero on the next open" state that a prior run's leftovers cause.
    ///
    /// Two things, both best-effort (a SIGKILL'd process skips Drop — the test harness handles that
    /// with a pre-run `pkill` + `timeout`, so a run can never outlive its window and hold the
    /// device): re-PAUSE RX DMA so the chip is not left streaming into a dead endpoint (set
    /// `RW_RELEASE_EN`, `REG_RXPKT_NUM[18]` — the inverse of `start_rx_dma`'s final step), and
    /// release the claimed interface so the kernel/next-open sees a free device. The pump threads
    /// hold a `Weak` and exit on their own once this backend's `Arc` is gone.
    fn drop(&mut self) {
        // Re-pause RX DMA (RW_RELEASE_EN = REG_RXPKT_NUM[18]); ignore errors on a dying device.
        if let Ok(v) = self.read32(0x0284) {
            let _ = self.write32(0x0284, v | (1 << 18));
        }
        let _ = self.handle.release_interface(0);
    }
}

impl Rtl8812auBackend {
    /// Find and claim the RTL8812AU (a product id in [`RTL8812AU_PIDS`]), taking
    /// it from any kernel driver. Never matches the 8812EU (`0xa81a`), so a
    /// co-resident EU monitor dongle is left untouched.
    pub fn open() -> Result<Self, FaceError> {
        Self::open_nth(0)
    }

    /// Open the **nth** (0-based) matching RTL8812AU-family adapter in USB-enumeration order.
    /// A host can carry several identical 8812au dongles (e.g. two `0bda:8812` on one OPi); `open()`
    /// always grabs the first, which is useless when that one is wedged and a fresh one sits behind it.
    /// Index is not stable across reboots/hotplug — [`open_select`](Self::open_select) also takes a
    /// stable `"<bus>-<port>"` USB address.
    pub fn open_nth(index: usize) -> Result<Self, FaceError> {
        Self::open_select(&crate::DeviceSelect::Index(index))
    }

    /// Open the RTL8812AU-family adapter chosen by `sel` (first / Nth / a `"<bus>-<port>"` address).
    /// Warns (or, with `NDN_GUARD_LIVE_LINK=1`, refuses) if the chosen device currently carries an UP
    /// kernel netdev, so a named-radio face never silently grabs the host's live Wi-Fi link.
    pub fn open_select(sel: &crate::DeviceSelect) -> Result<Self, FaceError> {
        let device =
            crate::usb_select::select_device(&RTL8812AU_PIDS, REALTEK_VID, sel, "RTL8812AU")?;
        let pid = device.device_descriptor().map_err(usb_err)?.product_id();
        Self::claim(device, pid)
    }

    fn claim(device: Device<Context>, pid: u16) -> Result<Self, FaceError> {
        // Per-device RX-stamp clock domain (bus<<8 | address) — read before opening the device.
        let tsf_domain =
            ClockDomainId((u32::from(device.bus_number()) << 8) | u32::from(device.address()));
        let handle = Arc::new(device.open().map_err(usb_err)?);
        // Detach only THIS device's kernel driver (Linux); leaves other adapters
        // (e.g. the 8812EU sniffer) bound to their drivers.
        let _ = handle.set_auto_detach_kernel_driver(true);
        // A USB port reset clears the device when the kernel driver's failed
        // probe / reset loop left its register interface unresponsive (vendor
        // reads time out otherwise). Best-effort: skip via `NDN_RADIO_NO_RESET=1`
        // (a stacked reset on a cold re-enumeration can re-wedge some devices).
        if std::env::var_os("NDN_RADIO_NO_RESET").is_none() {
            let _ = handle.reset();
        }
        handle.claim_interface(0).map_err(usb_err)?;

        let config = device.active_config_descriptor().map_err(usb_err)?;
        let (mut bulk_in, mut bulk_out) = (None, None);
        for iface in config.interfaces() {
            for desc in iface.descriptors() {
                for ep in desc.endpoint_descriptors() {
                    if ep.transfer_type() != TransferType::Bulk {
                        continue;
                    }
                    match ep.direction() {
                        // First bulk IN = RX (0x81); first bulk OUT = a TX queue.
                        Direction::In if bulk_in.is_none() => bulk_in = Some(ep.address()),
                        Direction::Out if bulk_out.is_none() => bulk_out = Some(ep.address()),
                        _ => {}
                    }
                }
            }
        }
        let no_ep = || init_err("RTL8812AU exposes no bulk IN/OUT endpoint");
        Ok(Self {
            ndr_contention: crate::realtek_contention::RtlEdcaSaved::new(),
            // 0x20 = -78 dBm, the value the majority of the captured programs settle on and the
            // vendor's `dm_dig_min`.
            rx_floor_igi: std::sync::atomic::AtomicU8::new(0x20),
            h2c_box: std::sync::atomic::AtomicU32::new(0),
            tx_rpt_seen: std::sync::atomic::AtomicU32::new(0),
            tx_rpt_ok: std::sync::atomic::AtomicU32::new(0),
            tx_rpt_retries: std::sync::atomic::AtomicU32::new(0),
            tx_rpt_saved: std::sync::atomic::AtomicU32::new(u32::MAX),
            handle,
            bulk_out: bulk_out.ok_or_else(no_ep)?,
            bulk_in: bulk_in.ok_or_else(no_ep)?,
            pid,
            // ★ **`RawNdn`, matching every other backend in this crate.**
            //
            // This defaulted to `Raw80211` ("the payload IS the frame, verbatim") for the NAN
            // management-frame path. That default had NO production benefit — the real open path
            // (`src/lib.rs`) always calls `.with_format(fmt)` explicitly — and it was reachable
            // only by a caller who forgot. MEASURED cost of forgetting, 2026-08-28: the flood
            // examples transmitted `vec![0x42; N]` as a malformed management frame with a
            // **unicast** `addr1 = 42:42:42:42:42:42`, so the MAC retried every frame to CWmax and
            // throughput collapsed from ~654 f/s to a dead-flat 10 f/s (the 100 ms `TX_TIMEOUT`).
            // A whole day of this radio's throughput and contention numbers was void.
            //
            // Every other backend here defaults to `RawNdn`; this one being different was an
            // accident of history, not a property of the silicon. NAN still gets `Raw80211` by
            // asking for it, which is what the production path already does.
            format: FrameFormat::default(),
            rx_pump: crate::rx_pump::RxPumpState::new(),
            ctrl_ops: std::sync::atomic::AtomicU64::new(0),
            tsf_domain,
            mesh_cv: std::sync::Mutex::new(None),
            cur_channel: std::sync::atomic::AtomicU8::new(0),
            tx_power_info: std::sync::Mutex::new(None),
            cca_saved: std::sync::atomic::AtomicU16::new(0xffff),
            cur_mcs: std::sync::Mutex::new(None),
        })
    }

    /// The matched USB product id.
    pub fn pid(&self) -> u16 {
        self.pid
    }

    /// The discovered bulk endpoint addresses `(IN, OUT)`.
    pub fn endpoints(&self) -> (u8, u8) {
        (self.bulk_in, self.bulk_out)
    }

    // ── Realtek register I/O (the `usbctrl_vendorreq` path) ──────────────────

    fn read_reg(&self, addr: u16, buf: &mut [u8]) -> Result<(), FaceError> {
        let seq = self
            .ctrl_ops
            .fetch_add(1, std::sync::atomic::Ordering::Relaxed);
        let n = self
            .handle
            .read_control(REQ_READ, VENDOR_REQ, addr, 0, buf, CTRL_TIMEOUT)
            .map_err(|e| ctrl_err(e, "read", addr, buf.len(), seq))?;
        if n != buf.len() {
            return Err(init_err(format!(
                "rtl8812au read_reg({addr:#06x}): short {n}/{}",
                buf.len()
            )));
        }
        Ok(())
    }

    fn write_reg(&self, addr: u16, data: &[u8]) -> Result<(), FaceError> {
        let seq = self
            .ctrl_ops
            .fetch_add(1, std::sync::atomic::Ordering::Relaxed);
        let n = self
            .handle
            .write_control(REQ_WRITE, VENDOR_REQ, addr, 0, data, CTRL_TIMEOUT)
            .map_err(|e| ctrl_err(e, "write", addr, data.len(), seq))?;
        if n != data.len() {
            return Err(init_err(format!(
                "rtl8812au write_reg({addr:#06x}): short {n}/{}",
                data.len()
            )));
        }
        Ok(())
    }

    /// Read an 8-bit register.
    pub fn read8(&self, addr: u16) -> Result<u8, FaceError> {
        let mut b = [0u8; 1];
        self.read_reg(addr, &mut b)?;
        Ok(b[0])
    }

    /// Read a 16-bit (little-endian) register.
    pub fn read16(&self, addr: u16) -> Result<u16, FaceError> {
        let mut b = [0u8; 2];
        self.read_reg(addr, &mut b)?;
        Ok(u16::from_le_bytes(b))
    }

    /// Read a 32-bit (little-endian) register.
    pub fn read32(&self, addr: u16) -> Result<u32, FaceError> {
        let mut b = [0u8; 4];
        self.read_reg(addr, &mut b)?;
        Ok(u32::from_le_bytes(b))
    }

    /// Write an 8-bit register.
    pub fn write8(&self, addr: u16, val: u8) -> Result<(), FaceError> {
        self.write_reg(addr, &[val])
    }

    /// Write a 16-bit (little-endian) register.
    pub fn write16(&self, addr: u16, val: u16) -> Result<(), FaceError> {
        self.write_reg(addr, &val.to_le_bytes())
    }

    /// Write a 32-bit (little-endian) register.
    pub fn write32(&self, addr: u16, val: u32) -> Result<(), FaceError> {
        self.write_reg(addr, &val.to_le_bytes())
    }

    /// Read and decode the chip-version registers — the first bring-up
    /// checkpoint (proves USB register I/O works and identifies the silicon).
    pub fn chip_info(&self) -> Result<ChipInfo, FaceError> {
        let sys_cfg = self.read32(REG_SYS_CFG)?;
        let sys_cfg1 = self.read32(REG_SYS_CFG1)?;
        Ok(ChipInfo {
            sys_cfg,
            sys_cfg1,
            cut: ((sys_cfg >> 12) & 0xF) as u8,
            // RTL-ID / test-chip bit (BIT(23)) in REG_SYS_CFG.
            test_chip: sys_cfg & (1 << 23) != 0,
        })
    }

    // ── Milestone 2: power-on + firmware download ────────────────────────────

    /// Walk a halmac power sequence: `PWR_WRITE` is a masked read-modify-write,
    /// `PWR_POLLING` waits for the masked value (each read is a USB control
    /// transfer, so ~1000 tries is roughly a 1 s ceiling).
    fn run_pwr_seq(&self, seq: &[PwrCfg]) -> Result<(), FaceError> {
        for c in seq {
            match c.cmd {
                PWR_WRITE => {
                    let v = (self.read8(c.offset)? & !c.msk) | (c.value & c.msk);
                    self.write8(c.offset, v)?;
                }
                PWR_POLLING => {
                    let mut ready = false;
                    for _ in 0..1000 {
                        if self.read8(c.offset)? & c.msk == (c.value & c.msk) {
                            ready = true;
                            break;
                        }
                    }
                    if !ready {
                        return Err(init_err(format!(
                            "rtl8812au pwrseq: polling timeout @ {:#06x} msk {:#04x} want {:#04x}",
                            c.offset, c.msk, c.value
                        )));
                    }
                }
                _ => {}
            }
        }
        Ok(())
    }

    rung! {
        /// Bring the MAC power domain up (card-emulation → active) by running the
        /// 8812A `CARDEMU_TO_ACT` sequence. Run once after [`open`](Self::open),
        /// before [`download_firmware`](Self::download_firmware).
        fn power_on(&self) -> Result<(), FaceError> {
            // Release REG_RSV_CTRL so MCU-IO-wrapper register writes take effect.
            self.write8(REG_RSV_CTRL, 0)?;
            self.run_pwr_seq(CARDEMU_TO_ACT)
        }
    }

    /// Bring the MAC power domain down to card-emulation (`ACT_TO_CARDEMU`).
    /// Call on a *brought-up* device before re-running `power_on`
    /// so a repeated bring-up starts from a known state (the `CARDEMU_TO_ACT`
    /// poll never completes if the MAC is already active). Not auto-invoked by
    /// `power_on` — its BB-register writes fault on a freshly enumerated device
    /// whose MAC is not yet powered.
    pub fn power_off(&self) -> Result<(), FaceError> {
        self.write8(REG_RSV_CTRL, 0)?;
        self.run_pwr_seq(ACT_TO_CARDEMU)
    }

    rung! {
        /// Download the vendored 8812A NIC firmware to the MCU and wait for it to
        /// boot (`WINTINI_RDY`). Requires [`power_on`](Self::power_on) first. Returns
        /// the firmware `(version, subversion)` from its header.
        ///
        /// Safe to re-run on a chip that is already up: it resets the 8051 first.
        fn download_firmware(&self) -> Result<(u16, u8), FaceError> {
            let version = u16::from_le_bytes([FW_NIC[4], FW_NIC[5]]);
            let subversion = FW_NIC[6];
            let body = &FW_NIC[FW_HDR_LEN..];

            // Reset the 8051 if it is already running a RAM image, before writing a new
            // one over it. Not a corner case: this is true on every bring-up after the
            // first, because a process that is killed — a `timeout`, a Ctrl-C — leaves
            // the chip live with its firmware running.
            //
            // Writing firmware over a running 8051 does not fail cleanly. The block
            // write to FW_START_ADDRESS times out and the device drops off the USB bus
            // entirely, recoverable only by a physical replug:
            //   usb write1568(0x1000) failed after 535 ok control transfers: timed out
            // and every subsequent open then fails on its first register read.
            //
            // The vendor driver guards it identically, and says why:
            // "If 8051 is running in RAM code, driver should inform Fw to reset by
            //  itself, or it will cause download Fw fail."
            if self.read8(REG_MCUFWDL)? & MCUFWDL_RAM_DL_SEL != 0 {
                self.write8(REG_MCUFWDL, 0)?;
                self.reset_8051()?;
            }

            self.fw_dl_enable(true)?;
            self.write_fw(body)?;
            self.fw_dl_enable(false)?;
            self.fw_free_to_go()?;
            Ok((version, subversion))
        }
    }

    /// `_FWDownloadEnable_8812`: gate the MCU firmware-download path and hold the
    /// 8051 in reset while writing (`enable`), or release it (`!enable`).
    fn fw_dl_enable(&self, enable: bool) -> Result<(), FaceError> {
        if enable {
            let t = self.read8(REG_MCUFWDL)?;
            self.write8(REG_MCUFWDL, t | 0x01)?; // MCUFWDL_EN
            let t = self.read8(REG_MCUFWDL + 2)?;
            self.write8(REG_MCUFWDL + 2, t & 0xf7)?; // hold 8051 reset
        } else {
            let t = self.read8(REG_MCUFWDL)?;
            self.write8(REG_MCUFWDL, t & 0xfe)?;
        }
        Ok(())
    }

    /// `_WriteFW_8812`: write the firmware body in 4 KB pages, selecting each
    /// page via `REG_MCUFWDL+2[2:0]` then block-writing it to the RAM window.
    fn write_fw(&self, body: &[u8]) -> Result<(), FaceError> {
        for (page, chunk) in body.chunks(DLFW_PAGE_SIZE).enumerate() {
            let v = (self.read8(REG_MCUFWDL + 2)? & 0xf8) | (page as u8 & 0x07);
            self.write8(REG_MCUFWDL + 2, v)?;
            self.block_write(chunk)?;
        }
        Ok(())
    }

    /// `_BlockWrite_8812` (USB): write a page's bytes to the firmware RAM window
    /// (`FW_START_ADDRESS`..) in ≤196-byte control transfers.
    fn block_write(&self, buf: &[u8]) -> Result<(), FaceError> {
        let mut off = 0;
        while off < buf.len() {
            let n = (buf.len() - off).min(DLFW_BLOCK);
            self.write_reg(FW_START_ADDRESS + off as u16, &buf[off..off + n])?;
            off += n;
        }
        Ok(())
    }

    /// `_FWFreeToGo8812`: mark download ready, reset the 8051, and poll until the
    /// firmware signals init done (`WINTINI_RDY`).
    fn fw_free_to_go(&self) -> Result<(), FaceError> {
        let mut v = self.read32(REG_MCUFWDL)?;
        v |= MCUFWDL_RDY;
        v &= !WINTINI_RDY;
        self.write32(REG_MCUFWDL, v)?;

        self.reset_8051()?;

        for _ in 0..1000 {
            if self.read32(REG_MCUFWDL)? & WINTINI_RDY != 0 {
                return Ok(());
            }
        }
        Err(init_err(
            "rtl8812au: firmware did not signal ready (WINTINI_RDY timeout)",
        ))
    }

    // ── Milestone 3 (first step): LLT packet-buffer page list ────────────────

    /// One LLT entry write: chain page `address` → page `data`, then poll the
    /// LLT engine back to idle (`_LLTWrite_8812A`).
    fn llt_write(&self, address: u8, data: u8) -> Result<(), FaceError> {
        let value = (data as u32) // _LLT_INIT_DATA: [7:0]
            | ((address as u32) << 8) // _LLT_INIT_ADDR: [15:8]
            | (LLT_WRITE_ACCESS << 30); // _LLT_OP: [31:30]
        self.write32(REG_LLT_INIT, value)?;
        for count in 0.. {
            if (self.read32(REG_LLT_INIT)? >> 30) & 0x3 == LLT_NO_ACTIVE {
                return Ok(());
            }
            if count > LLT_POLL_THRESHOLD {
                return Err(init_err(format!(
                    "rtl8812au: LLT write @page {address} timed out"
                )));
            }
        }
        unreachable!()
    }

    rung! {
        /// Initialize the LLT packet-buffer page chain (`InitLLTTable8812A`): pages
        /// `0..boundary-1` form a forward-linked TX-queue list ending in `0xFF`, and
        /// `boundary..255` form a ring (beacon / loopback buffer). Run after
        /// [`power_on`](Self::power_on); it's the first MAC-init step and each write
        /// polls, so success confirms the MAC's buffer engine is alive.
        fn init_llt(&self) -> Result<(), FaceError> {
            let boundary = TX_PAGE_BOUNDARY;
            // TX-queue list: each page points to the next.
            for i in 0..boundary - 1 {
                self.llt_write(i, i + 1)?;
            }
            // End of the TX-queue list.
            self.llt_write(boundary - 1, 0xFF)?;
            // Ring buffer over the remaining pages.
            for i in boundary..LAST_TX_PKT_PAGE {
                self.llt_write(i, i + 1)?;
            }
            // Last entry loops back to the ring start.
            self.llt_write(LAST_TX_PKT_PAGE, boundary)?;
            Ok(())
        }
    }

    // ── Milestone 3: MAC register table (phydm conditional config) ───────────

    /// phydm `check_positive` for the MAC array, specialized to this chip.
    /// `cond1` is an IF condition word; returns whether it matches our cut /
    /// interface. Board-type is 0 here, so the `cond2`/`cond4` RF-path checks
    /// (which compare against `driver2`/`driver4 == 0`) only ever apply when a
    /// condition's board nibble is set — none do in the 8812A MAC table.
    fn check_positive(cond1: u32) -> bool {
        let driver1 = PHYDM_DRIVER1;
        // Value-defined checks: package [15:12], cut [27:24].
        if cond1 & 0x0000_F000 != 0 && cond1 & 0x0000_F000 != driver1 & 0x0000_F000 {
            return false;
        }
        if cond1 & 0x0F00_0000 != 0 && cond1 & 0x0F00_0000 != driver1 & 0x0F00_0000 {
            return false;
        }
        // Bit-defined check (interface/board nibble).
        let c = cond1 & 0x00FF_0FFF;
        let d = driver1 & 0x00FF_0FFF;
        if c & d != c {
            return false;
        }
        // Board-type DONTCARE (nibble 0) ⇒ matched on interface alone.
        c & 0x0F == 0
    }

    /// Walk a phydm conditional config table (`(addr, value)` `u32` pairs with
    /// IF/ELSE/ENDIF markers in the high bits), applying each matching entry for
    /// our cut/interface via `apply`. Shared by the MAC, BB, and AGC tables —
    /// they use the identical phydm condition format.
    fn config_table(
        &self,
        table: &[u8],
        mut apply: impl FnMut(&Self, u32, u32) -> Result<(), FaceError>,
    ) -> Result<(), FaceError> {
        let words: Vec<u32> = table
            .chunks_exact(4)
            .map(|c| u32::from_le_bytes([c[0], c[1], c[2], c[3]]))
            .collect();

        let mut is_matched = true;
        let mut is_skipped = false;
        let mut pre = 0u32;
        let mut i = 0;
        while i + 1 < words.len() {
            let (v1, v2) = (words[i], words[i + 1]);
            if v1 & 0xC000_0000 != 0 {
                if v1 & 0x8000_0000 != 0 {
                    // Positive marker: IF / ELSE-IF (store), ELSE, or ENDIF.
                    let c_cond = ((v1 >> 28) & 0x3) as u8;
                    if c_cond == COND_ENDIF {
                        is_matched = true;
                        is_skipped = false;
                    } else if c_cond == COND_ELSE {
                        is_matched = !is_skipped;
                    } else {
                        pre = v1; // remember the IF condition for the negative entry
                    }
                } else if !is_skipped {
                    // Negative marker: evaluate the stored IF condition.
                    if Self::check_positive(pre) {
                        is_matched = true;
                        is_skipped = true;
                    } else {
                        is_matched = false;
                        is_skipped = false;
                    }
                } else {
                    is_matched = false;
                }
            } else if is_matched {
                apply(self, v1, v2)?;
            }
            i += 2;
        }
        Ok(())
    }

    rung! {
        /// Apply the MAC register table (`PHY_MACConfig8812`): byte writes, honoring
        /// the phydm condition blocks. Run after the firmware is ready.
        fn mac_config(&self) -> Result<(), FaceError> {
            self.config_table(MAC_REG, |s, addr, val| s.write8(addr as u16, val as u8))
        }
    }

    // ── Milestone 4: baseband (BB/PHY) + AGC init ────────────────────────────

    /// Apply one BB register entry: a delay opcode (addr `0xfe..0xf9`) or a full
    /// dword write to the BB register space (`odm_config_bb_phy_8812a`).
    fn bb_write(&self, addr: u32, data: u32) -> Result<(), FaceError> {
        match addr {
            0xfe => std::thread::sleep(Duration::from_millis(50)),
            0xfd => std::thread::sleep(Duration::from_millis(5)),
            0xfc => std::thread::sleep(Duration::from_millis(1)),
            0xfb => std::thread::sleep(Duration::from_micros(50)),
            0xfa => std::thread::sleep(Duration::from_micros(5)),
            0xf9 => std::thread::sleep(Duration::from_micros(1)),
            _ => self.write32(addr as u16, data)?,
        }
        Ok(())
    }

    rung! {
        /// Configure the baseband (`PHY_BBConfig8812`): power on BB + both RF paths,
        /// then apply the BB PHY-register table and the AGC table. Run after
        /// [`mac_init_queues`](Self::mac_init_queues). Reuses the phydm condition
        /// evaluator (the BB/AGC tables share the MAC table's conditional format).
        fn bb_config(&self) -> Result<(), FaceError> {
            // Power on baseband + RF analog.
            let fen = self.read8(REG_SYS_FUNC_EN)?;
            self.write8(REG_SYS_FUNC_EN, fen | FEN_USBA)?;
            self.write8(
                REG_SYS_FUNC_EN,
                fen | FEN_USBA | FEN_BB_GLB_RSTN | FEN_BBRSTB,
            )?;
            self.write8(REG_RF_CTRL, 0x07)?; // path-A RF power on
            self.write8(REG_OPT_CTRL_8812 + 2, 0x07)?; // path-B RF power on

            // BB PHY register table, then the AGC table.
            self.config_table(PHY_REG, |s, a, d| s.bb_write(a, d))?;
            self.config_table(AGC_TAB, |s, a, d| s.bb_write(a, d))?;
            Ok(())
        }
    }

    /// Read a baseband register (32-bit) — for verifying `bb_config`.
    pub fn bb_read(&self, addr: u16) -> Result<u32, FaceError> {
        self.read32(addr)
    }

    /// Masked baseband write (`phy_set_bb_reg`): read-modify-write the `mask`
    /// bits of `addr` with `data` (shifted to the mask's position).
    fn bb_set(&self, addr: u16, mask: u32, data: u32) -> Result<(), FaceError> {
        if mask == 0xFFFF_FFFF {
            self.write32(addr, data)
        } else {
            let shift = mask.trailing_zeros();
            let v = (self.read32(addr)? & !mask) | ((data << shift) & mask);
            self.write32(addr, v)
        }
    }

    /// Masked baseband read (`phy_query_bb_reg`).
    fn bb_query(&self, addr: u16, mask: u32) -> Result<u32, FaceError> {
        Ok((self.read32(addr)? & mask) >> mask.trailing_zeros())
    }

    // ── Milestone 5: RF (radio) register init ────────────────────────────────

    /// RF serial write (`phy_RFSerialWrite`): `DataAndAddr =
    /// (rf_addr<<20 | data&0xFFFFF) & 0x0FFFFFFF` → the path's LSSI write register.
    fn rf_write(&self, path: RfPath, rf_addr: u32, data: u32) -> Result<(), FaceError> {
        let lssi = match path {
            RfPath::A => RF_LSSI_WRITE_A,
            RfPath::B => RF_LSSI_WRITE_B,
        };
        self.write32(lssi, ((rf_addr << 20) | (data & 0x000F_FFFF)) & 0x0FFF_FFFF)
    }

    /// One RF table entry: a delay opcode (addr `0xfe`/`0xffe`) or an RF write.
    fn rf_write_entry(&self, path: RfPath, addr: u32, data: u32) -> Result<(), FaceError> {
        if addr == 0xfe || addr == 0xffe {
            std::thread::sleep(Duration::from_millis(50));
            return Ok(());
        }
        self.rf_write(path, addr, data)
    }

    /// Masked RF write (`phy_set_rf_reg`): read-modify-write the `mask` bits of
    /// RF register `rf_addr` on `path`.
    fn rf_set(&self, path: RfPath, rf_addr: u32, mask: u32, data: u32) -> Result<(), FaceError> {
        if mask == 0x000F_FFFF {
            self.rf_write(path, rf_addr, data)
        } else {
            let shift = mask.trailing_zeros();
            let v = (self.rf_read(path, rf_addr)? & !mask) | ((data << shift) & mask);
            self.rf_write(path, rf_addr, v)
        }
    }

    // ── Milestone 7: channel / bandwidth selection (2.4 GHz, 20 MHz) ──────────

    /// Tune to a 2.4 GHz `channel` (1–14) at 20 MHz — the 5 GHz→2.4 GHz band
    /// switch (`PHY_SwitchWirelessBand8812`), the channel write to RF `0x18` on
    /// both paths, and the 20 MHz bandwidth set. Run after the PHY is configured
    /// (`rf_config`). Verify by reading RF `0x18` back (byte 0
    /// = channel).  Assumes `rfe_type == 0` (the common generic-dongle RFE).
    pub fn set_channel(&self, channel: u8) -> Result<(), FaceError> {
        // Record the channel for the per-channel EFUSE-calibrated TX-power base.
        self.cur_channel
            .store(channel, std::sync::atomic::Ordering::Relaxed);
        // 5 GHz: replay the kernel rtw88 driver's exact per-channel program (usbmon golden traces
        // in golden/). The band switch (BB + RFE), the RF `0x18`/LSSI channel writes, and the 5 GHz
        // TXAGC tables are a single interdependent sequence — piecemeal deltas onto the 2.4 path did
        // not work, so the whole thing is applied. The band-switch writes are channel-constant; the
        // RF, fc_area, and TXAGC writes are per-channel, so each channel has its own traced program
        // ([`PROGS_5G`]). Untraced 5 GHz channels error (capture a trace to add one).
        if channel > 14 {
            return match PROGS_5G.iter().find(|(ch, _)| *ch == channel) {
                Some((_, prog)) => self.apply_reg_program(prog),
                None => Err(init_err(format!(
                    "rtl8812au: 5 GHz ch{channel} not traced (have {:?}); capture a golden trace",
                    PROGS_5G.iter().map(|(c, _)| *c).collect::<Vec<_>>()
                ))),
            };
        }

        // ── 2.4 GHz band switch (BB) — production two-way named-time on-air ──
        self.bb_set(0x0808, 0x3000_0000, 0x03)?; // OFDM + CCK enable (0x808)
        self.bb_set(0x0834, 0x3, 0x1)?; // BW indication
        self.bb_set(0x0830, 0x3_E000, 0x17)?; // PD_TH 0x830[17:13]
        self.bb_set(0x0830, 0xE, 0x04)?; // 0x830[3:1] = 100 (2T)
        self.bb_set(0x082C, 0x3, 0x0)?; // AGC table select 2.4 G
        // RFE (rfe_type 0): pinmux 0x77777777, inv 0.
        self.write32(0x0CB0, 0x7777_7777)?;
        self.write32(0x0EB0, 0x7777_7777)?;
        self.bb_set(0x0CB4, 0x3FF0_0000, 0x000)?;
        self.bb_set(0x0EB4, 0x3FF0_0000, 0x000)?;
        // ★ Arm the page-F sensing counters. The 5 GHz golden programs replay these three toggles
        // (0x09a4 bit17, 0x0a2c bit15, 0x0b58 bit0) and this branch did not, so on 2.4 GHz the
        // FA/CCA block read a flat zero — which is how it came to be recorded as "dead" hardware.
        // The mainline kernel reads live values from it on this dongle at ch6
        // (`golden/rtw88-2g-ch6.txt`: OFDM FA 10867 / 48980 / 50133 across three DIG ticks).
        let _ = self.reset_fa_counters();
        // CCK FA / scan workaround + CCK check (clear 0x454[7] for 2.4 G).
        self.bb_set(0x080C, 0xF0, 0x1)?;
        self.bb_set(0x0A04, 0x0F00_0000, 0x1)?;
        let cck = self.read8(0x0454)?;
        self.write8(0x0454, cck & !0x80)?;
        // fc_area (channel < 36).
        self.bb_set(0x0860, 0x1FFE_0000, 0x96A)?;
        // RF channel + 2.4 G band/mode + 20 MHz, both paths.
        for path in [RfPath::A, RfPath::B] {
            self.rf_set(path, 0x18, 0x7_0300, 0x000)?; // 2.4 G band/mode (bits 18,17,16,9,8 = 0)
            self.rf_set(path, 0x18, 0xC00, 0x3)?; // BW 20 MHz
            self.rf_set(path, 0x18, 0xFF, channel as u32)?; // channel byte
        }
        // 20 MHz bandwidth (BB + MAC).
        self.bb_set(0x08AC, 0x0030_03C3, 0x0030_0200)?; // rRFMOD 20 MHz
        self.bb_set(0x08AC, 0x300, 0x2)?; // spur workaround, ch ≤ 14
        let trx = self.read16(0x0668)?;
        self.write16(0x0668, trx & 0xFE7F)?; // WMAC TRXPTCL: 20 MHz
        // ★ The captured per-channel programs each leave a different IGI, and ten of the 25
        // write none at all — an 8 dB, order-dependent detection-floor swing. Make it
        // deterministic. See `reassert_rx_floor`.
        let _ = self.reassert_rx_floor();
        Ok(())
    }

    /// Apply a captured register program — `(addr, width-in-bytes, value)` writes, in order.
    fn apply_reg_program(&self, prog: &[(u16, u8, u32)]) -> Result<(), FaceError> {
        for &(addr, width, val) in prog {
            match width {
                1 => self.write8(addr, val as u8)?,
                2 => self.write16(addr, val as u16)?,
                _ => self.write32(addr, val)?,
            }
        }
        Ok(())
    }

    // ── Milestone 6: RF calibration (LCK; IQK) ───────────────────────────────

    /// LC (VCO/PLL) calibration (`_phy_lc_calibrate_8812a`): pause TX, enter LCK
    /// mode, trigger the LC cal on RF `0x18[15]`, poll until done, and restore.
    /// Locks the synthesizer for the current channel; run after
    /// [`set_channel`](Self::set_channel).
    /// ★ **Host-to-Card command transport** — the firmware interface this driver has never used.
    ///
    /// The parts this fleet calls "best supported" are the firmware-mediated ones: the MT7921AU
    /// (whose MCU *validates* requests, which is why a `cw_min` of 2 was safe there and cost the
    /// mt76x2 register path a replug) and the RTL8733BU (exact TX counters, the best transmit
    /// instrument here). The 8812AU has the same class of interface and it was never opened —
    /// which is why it has no TX feedback, no rate adaptation, and nothing firmware-mediated.
    ///
    /// **The transaction is not reverse-engineered from a foreign driver — it is captured on THIS
    /// dongle.** `golden/rtw88-5g-ch149.txt` and every per-channel capture show the mainline kernel
    /// doing exactly this, every 2.0 s:
    ///
    /// ```text
    /// Ci 01cc  1 byte    <- poll HMETFR: which boxes still hold an unconsumed command
    /// Co 01d0  4 bytes   <- write HMEBOX_0 with the command
    /// ...next time 01d4, then 01d8, then 01dc, wrapping to 01d0  (round-robin, 4 boxes)
    /// ```
    ///
    /// The firmware side corroborates: the image reads `0x1D0` through a helper, ORs status bits
    /// into `0x1C0`, and writes responses to `0x1C2`/`0x1C4`.
    ///
    /// ⚠ Four bytes only. Longer commands use the EXT boxes (`0x01F0+`), which no capture we hold
    /// exercises — do not assume their layout.
    pub fn h2c(&self, payload: [u8; 4]) -> Result<(), FaceError> {
        use std::sync::atomic::Ordering::Relaxed;
        // HMETFR bit N is set while box N still holds a command the firmware has not taken.
        // Wait for ours to clear rather than overwriting a pending command.
        let box_idx = (self.h2c_box.fetch_add(1, Relaxed) & 0x3) as u16;
        let bit = 1u8 << box_idx;
        let deadline = std::time::Instant::now() + Duration::from_millis(200);
        loop {
            let tfr = self.read8(0x01cc)?;
            if tfr & bit == 0 {
                break;
            }
            if std::time::Instant::now() >= deadline {
                return Err(init_err(format!(
                    "rtl8812au h2c: box {box_idx} still busy after 200 ms (HMETFR {tfr:#04x}) —                      firmware is not consuming commands"
                )));
            }
            std::thread::sleep(Duration::from_millis(1));
        }
        self.write32(0x01d0 + box_idx * 4, u32::from_le_bytes(payload))
    }

    /// ★ **Arm the MAC's transmit-report engine.**
    ///
    /// The per-frame descriptor bit (`W2 SPE_RPT`) makes the MAC *tag* a frame for reporting, and
    /// MEASURED it does move the CCX ring index — but it yielded only 1-4 records per ~2300 armed
    /// frames. The firmware cannot be the limiter: exactly one site in the whole image touches the
    /// ring at XDATA `0x8000` (the drain at code `0x7A6E`), so the MAC fills it directly. The
    /// remaining gate therefore had to be a MAC register, and this is it.
    ///
    /// **Address provenance — in-generation, which is why this is not a guess.**
    /// `REG_TX_RPT_CTRL = 0x04EC` / `REG_TX_RPT_TIME = 0x04F0` are defined in the mainline
    /// **Jaguar1** driver `rtlwifi/rtl8821ae/reg.h:225-226` — the PCIe sibling of this exact
    /// silicon generation. (rtl8821ae *defines* but never *writes* them, which is precisely why
    /// they never appear in our golden traces and why this looked unsourced.) The write sequence
    /// is taken from a driver that does use them, `rtlwifi/rtl8188ee/hw.c:851-855`:
    ///
    /// ```text
    /// REG_TX_RPT_CTRL     |= BIT(0) | BIT(1)
    /// REG_TX_RPT_CTRL + 1  = 2        // number of MACIDs to track
    /// REG_TX_RPT_TIME      = 0xcdf0   // report timer
    /// ```
    ///
    /// Restorable: [`disarm_tx_report`](Self::disarm_tx_report) puts back the exact bytes found.
    pub fn arm_tx_report(&self) -> Result<(), FaceError> {
        use std::sync::atomic::Ordering::Relaxed;
        let ctrl = self.read8(0x04ec)?;
        let macid = self.read8(0x04ed)?;
        let time = self.read16(0x04f0)?;
        // Pack the as-found state once so a later disarm is a restore and not a guess.
        let _ = self.tx_rpt_saved.compare_exchange(
            u32::MAX,
            u32::from(ctrl) | (u32::from(macid) << 8) | (u32::from(time) << 16),
            Relaxed,
            Relaxed,
        );
        self.write8(0x04ec, ctrl | 0x03)?;
        self.write8(0x04ed, 2)?;
        self.write16(0x04f0, 0xcdf0)?;
        Ok(())
    }

    /// Put `REG_TX_RPT_CTRL`/`TIME` back exactly as found. No-op if never armed.
    pub fn disarm_tx_report(&self) -> Result<(), FaceError> {
        use std::sync::atomic::Ordering::Relaxed;
        let v = self.tx_rpt_saved.swap(u32::MAX, Relaxed);
        if v == u32::MAX {
            return Ok(());
        }
        self.write8(0x04ec, (v & 0xff) as u8)?;
        self.write8(0x04ed, ((v >> 8) & 0xff) as u8)?;
        self.write16(0x04f0, ((v >> 16) & 0xffff) as u16)?;
        Ok(())
    }

    /// ★ **Transmit feedback** — `(reports seen, reports delivered)`, plus accumulated retries.
    ///
    /// ⚠ **This is NOT the same quantity the RTL8733BU reports through the same trait method, and
    /// comparing them across radios without reading this will produce a wrong answer.**
    ///
    /// * The **8733BU** pair are *baseband* counters — MAC→BB transmit requests and BB→RF keys,
    ///   one per attempt, MEASURED at exactly +50 per 50 injects. They answer **"did the MAC ever
    ///   ask?"** and they are exact.
    /// * **This** pair are *frame completions the firmware reported*: sampled 1-in-N (whatever
    ///   `NDN_AU_TXRPT` requested), and lossy when the 16-slot CCX ring or the firmware's C2H ring
    ///   overflows under load. They answer **"did the air eat it?"** — which the 8733BU pair
    ///   cannot — but they are a SAMPLE, not a census.
    ///
    /// So a delivery ratio from this is meaningful; an absolute frame count is not.
    /// `(0, 0)` until reports are armed: the arming is a per-frame TX-descriptor bit
    /// (`W2 bit 19 SPE_RPT`), not a register, and an RX pump must be running to collect them.
    pub fn tx_report_counters(&self) -> (u32, u32, u32) {
        use std::sync::atomic::Ordering::Relaxed;
        (
            self.tx_rpt_seen.load(Relaxed),
            self.tx_rpt_ok.load(Relaxed),
            self.tx_rpt_retries.load(Relaxed),
        )
    }

    /// Read the firmware's H2C status byte (`0x01C0`) and its two response bytes
    /// (`0x01C2`, `0x01C3`).
    ///
    /// The firmware ORs bits `0x01/0x02/0x10/0x20` into `0x01C0` at four distinct sites in the
    /// image — the only visible acknowledgement that a command was seen, since C2H reports are
    /// silent until enabled (MEASURED: 0 C2H buffers in 12 s of flooding).
    pub fn h2c_status(&self) -> Result<(u8, u8, u8), FaceError> {
        Ok((
            self.read8(0x01c0)?,
            self.read8(0x01c2)?,
            self.read8(0x01c3)?,
        ))
    }

    /// Per-queue MAC transmit gate — the hardware half of a slot MAC.
    ///
    /// Bit map (`hal_com_reg.h` `REG_TXPAUSE`): b0 VO, b1 VI, b2 BE, b3 BK, b4 MGQ, b5 HIQ,
    /// b6 BCNQ. [`TXPAUSE_DATA_QUEUES`](Self::TXPAUSE_DATA_QUEUES) = 0x3f gates every data and
    /// management queue while sparing the beacon queue.
    ///
    /// Address provenance: `0x0522` is already written by `lc_calibrate` in
    /// this file, and the firmware image itself touches it at six sites — so this is not an
    /// address imported from another chip generation.
    ///
    /// ⚠ MEASURED SEMANTICS on the sibling RTL8733BU: `REG_TXPAUSE` **HOLDS, it does not drop**.
    /// A held queue drains when released (~20 KB there), so this is only safe where the burst
    /// lands in a window you own. Throughput under a 50 % duty measured **13.9 %**, not 50 %.
    /// Budget from that measurement, not from the duty cycle.
    pub fn set_tx_pause(&self, mask: u8) -> Result<(), FaceError> {
        self.write8(0x0522, mask)?;
        // Read back: a gate that silently did not take is worse than no gate, because the
        // scheduler above believes the airtime lease is enforced.
        let rb = self.read8(0x0522)?;
        if rb != mask {
            return Err(init_err(format!(
                "rtl8812au: TXPAUSE readback {rb:#04x} != requested {mask:#04x}"
            )));
        }
        Ok(())
    }

    /// ★ **The fleet's first dB-denominated RX sensitivity knob.**
    ///
    /// On Jaguar1 the initial-gain index is an absolute axis: `floor_dBm = IGI - 110`
    /// (`IGI_2_DBM`, `phydm_adaptivity.h:36`) — the same -110 that anchors RSSI and EDCCA on this
    /// generation. The a81a's equivalent is explicitly *not* dB-denominated, so this is the only
    /// receive knob in the fleet a link budget can reason about.
    ///
    /// Address provenance: `rA_IGI_Jaguar 0xc50` / `rB_IGI_Jaguar 0xe50`, field mask `0x7f`
    /// (`ODM_BIT_IGI_11AC`). Decisive: `golden/rtw88-2g-ch6.txt` shows the mainline kernel walking
    /// `0c50` through 0x1e -> 0x20 -> 0x22 with `0e50` mirrored, on this dongle. Bounds 0x20..0x3e
    /// are the vendor's own `dm_dig_min`/`dm_dig_max`.
    ///
    /// ⚠ Two SEPARATE registers, one per path — not the a81a's packed `0x1d70[6:0]/[14:8]`.
    /// Returns the **achieved** floor read back from path A, never the requested one.
    pub fn set_rx_floor_dbm(&self, dbm: i8) -> Result<i8, FaceError> {
        const IGI_MIN: u32 = 0x20;
        const IGI_MAX: u32 = 0x3e;
        let igi = (i32::from(dbm) + 110).clamp(IGI_MIN as i32, IGI_MAX as i32) as u32;
        self.bb_set(0x0C50, 0x7f, igi)?;
        self.bb_set(0x0E50, 0x7f, igi)?;
        self.rx_floor_igi
            .store(igi as u8, std::sync::atomic::Ordering::Relaxed);
        Ok(((self.read32(0x0C50)? & 0x7f) as i16 - 110) as i8)
    }

    /// The detection floor currently programmed, in dBm, read from path A.
    pub fn read_rx_floor_dbm(&self) -> Result<i8, FaceError> {
        Ok(((self.read32(0x0C50)? & 0x7f) as i16 - 110) as i8)
    }

    /// ☠ **Re-assert the RX floor after a channel program.**
    ///
    /// MEASURED from our own captured programs: the per-channel 5 GHz traces each end with a
    /// different IGI — ch36 `0x24`, ch56/140/153/157 `0x1c`, ch100 `0x1f`, most others `0x20` —
    /// and **10 of the 25 channels never write it at all**, silently inheriting whatever the
    /// previously-tuned channel left. On the `IGI - 110` axis that is a **-82 to -74 dBm swing,
    /// 8 dB, plus order dependence**: every cross-channel RSSI or occupancy comparison this radio
    /// has ever produced carries that bias, including anything feeding channel-choice cognition.
    ///
    /// The captured traces are left intact (they are observed kernel behaviour); the floor is made
    /// deterministic by writing it again afterwards.
    pub fn reassert_rx_floor(&self) -> Result<(), FaceError> {
        let igi = u32::from(self.rx_floor_igi.load(std::sync::atomic::Ordering::Relaxed));
        self.bb_set(0x0C50, 0x7f, igi)?;
        self.bb_set(0x0E50, 0x7f, igi)?;
        Ok(())
    }

    /// Read the current per-queue transmit gate.
    pub fn read_tx_pause(&self) -> Result<u8, FaceError> {
        self.read8(0x0522)
    }

    /// Every data + management queue, beacon queue spared — the mask `iqk_configure_mac` already
    /// proves this silicon accepts.
    pub const TXPAUSE_DATA_QUEUES: u8 = 0x3f;
}

/// `REG_RXERR_RPT` counter selectors (`hal_com_reg.h:440-453`). The register exposes thirteen
/// different counters through one 4-bit selector; the driver had never written that selector.
pub mod rxerr_type {
    /// OFDM PPDUs seen — the frame-rate proxy the shipped occupancy sensor uses.
    pub const OFDM_PPDU: u8 = 0;
    /// OFDM false alarms.
    pub const OFDM_FALSE_ALARM: u8 = 1;
    /// CCK PPDUs seen.
    pub const CCK_PPDU: u8 = 4;
    /// HT PPDUs seen.
    pub const HT_PPDU: u8 = 8;
    /// ★ **RX FIFO full — frames the hardware dropped because the host could not drain it.**
    /// A direct hardware measure of the receive ceiling this fleet previously had to infer from
    /// delivery ratios (see the note that a "high drop ratio" was an observer RX limit, not
    /// on-air loss). Nothing else in the fleet reports this directly.
    pub const RX_FULL_DROP: u8 = 15;
}

/// Frame-free sensing counters — false alarms, CCA and per-modulation CRC, straight from the PHY.
///
/// ★ This block was recorded as **DEAD** in this project's notes. It is not: the mainline kernel
/// reads live, growing values from it on this dongle. `golden/rtw88-2g-ch6.txt` carries three DIG
/// ticks 2 s apart with `0x0f48` (OFDM FA) = **10867, 48980, 50133** and `0x0a5c` (CCK FA) =
/// 30, 197, 192 — MEASURED by decoding that trace.
///
/// The "dead" reading is fully explained by two things, both ours:
///   1. the arming sequence (`0x09a4` bit17, `0x0a2c` bit15, `0x0b58` bit0 — see
///      [`Rtl8812auBackend::reset_fa_counters`]) appears in our **5 GHz** golden programs and
///      **nowhere in the 2.4 GHz branch of `set_channel`** — and the probe that produced the
///      verdict defaulted to ch6;
///   2. it read CCA from `0xF4C`, which the vendor header names `rOFDM_FalseAlarm2_Jaguar` — the
///      *spoofing* counter. CCA lives at `0x0F08`.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct FaCounters {
    /// OFDM false alarms (`0x0F48`, `ODM_REG_OFDM_FA_11AC`).
    pub ofdm_fa: u16,
    /// CCK false alarms (`0x0A5C`, `ODM_REG_CCK_FA_11AC`).
    pub cck_fa: u16,
    /// CCK clear-channel assessments (`0x0F08[15:0]`).
    pub cck_cca: u16,
    /// OFDM clear-channel assessments (`0x0F08[31:16]`).
    pub ofdm_cca: u16,
    /// CRC32 OK, indexed CCK / OFDM / HT / VHT.
    pub crc_ok: [u16; 4],
    /// CRC32 error, same order.
    pub crc_err: [u16; 4],
}

impl Rtl8812auBackend {
    /// Read the page-F sensing block. Counters are free-running since the last
    /// [`reset_fa_counters`](Self::reset_fa_counters); difference two reads, or reset then read.
    pub fn read_fa_counters(&self) -> Result<FaCounters, FaceError> {
        // CRC32 pairs: OK in [13:0], ERR in [29:16]. Order matches `crc_ok`/`crc_err`.
        const CRC: [u16; 4] = [0x0f04, 0x0f14, 0x0f10, 0x0f0c]; // cck, ofdm, ht, vht
        let mut crc_ok = [0u16; 4];
        let mut crc_err = [0u16; 4];
        for (i, reg) in CRC.iter().enumerate() {
            let v = self.read32(*reg)?;
            crc_ok[i] = (v & 0x3fff) as u16;
            crc_err[i] = ((v >> 16) & 0x3fff) as u16;
        }
        let cca = self.read32(0x0f08)?;
        Ok(FaCounters {
            ofdm_fa: self.read16(0x0f48)?,
            cck_fa: self.read16(0x0a5c)?,
            cck_cca: (cca & 0xffff) as u16,
            ofdm_cca: ((cca >> 16) & 0xffff) as u16,
            crc_ok,
            crc_err,
        })
    }

    /// Arm/reset the sensing counters — the three read-modify-write toggles the kernel performs
    /// after every DIG tick (`golden/rtw88-2g-ch6.txt:555-590`), and which our own 5 GHz golden
    /// programs already replay.
    ///
    /// ⚠ Read-modify-write, never a blind dword: these registers carry unrelated PHY configuration
    /// in their other bits, and the kernel restores the original value after strobing.
    pub fn reset_fa_counters(&self) -> Result<(), FaceError> {
        for (reg, bit) in [(0x09a4u16, 1u32 << 17), (0x0a2c, 1 << 15), (0x0b58, 1 << 0)] {
            let orig = self.read32(reg)?;
            // 0x0a2c is strobed by CLEARING its bit; the other two by SETTING theirs.
            let strobe = if reg == 0x0a2c {
                orig & !bit
            } else {
                orig | bit
            };
            self.write32(reg, strobe)?;
            self.write32(reg, orig)?;
        }
        Ok(())
    }
}

impl Rtl8812auBackend {
    rung! {
        fn lc_calibrate(&self) -> Result<(), FaceError> {
            const RF_LCK: u32 = 0xB4;
            const RF_CHNLBW: u32 = 0x18;
            const REG_TXPAUSE: u16 = 0x0522;

            // If a continuous-tone TX is active (0x914[18:16]), don't pause; else
            // pause packet TX during the cal.
            let cont_tx = self.read32(0x0914)? & 0x7_0000 != 0;
            // ★ Save the as-found gate. This used to write 0x00 on exit unconditionally, which
            // silently RE-OPENS a hold that `RadioKnobs::set_tx_hold` had deliberately asserted — a
            // retune inside a held airtime slot would let the queues drain into somebody else's turn,
            // which is the exact bleed a slot schedule exists to prevent.
            let saved_pause = if cont_tx {
                None
            } else {
                Some(self.read8(REG_TXPAUSE)?)
            };
            if !cont_tx {
                self.write8(REG_TXPAUSE, 0xFF)?;
            }

            // Enter LCK mode.
            let lck = self.rf_read(RfPath::A, RF_LCK)?;
            self.rf_write(RfPath::A, RF_LCK, lck | (1 << 14))?;

            // Trigger LC cal (RF 0x18[15]) and poll until it self-clears.
            let lc_cal = self.rf_read(RfPath::A, RF_CHNLBW)?;
            self.rf_write(RfPath::A, RF_CHNLBW, lc_cal | 0x0_8000)?;
            std::thread::sleep(Duration::from_millis(150));
            for _ in 0..5 {
                if self.rf_read(RfPath::A, RF_CHNLBW)? & 0x8000 == 0 {
                    break;
                }
                std::thread::sleep(Duration::from_millis(10));
            }
            self.rf_write(RfPath::A, RF_CHNLBW, lc_cal)?; // restore RF 0x18

            // Leave LCK mode + un-pause TX.
            let lck = self.rf_read(RfPath::A, RF_LCK)?;
            self.rf_write(RfPath::A, RF_LCK, lck & !(1 << 14))?;
            if let Some(prev) = saved_pause {
                self.write8(REG_TXPAUSE, prev)?;
            }
            Ok(())
        }
    }

    /// Select BB page C (`c1=false`) or page C1 (`c1=true`) via `0x82c[31]`.
    /// The IQK reuses the same 0xc80–0xed4 addresses across two physical pages,
    /// so every page toggle in the vendor flow must be preserved exactly.
    fn page(&self, c1: bool) -> Result<(), FaceError> {
        self.bb_set(0x82c, 0x8000_0000, c1 as u32)
    }

    /// Read an 11-bit signed IQK result field (`[26:16]`) from `reg` (`0xd00`/
    /// `0xd40`) and sign-extend it — the vendor's `(val << 21) >> 21` idiom.
    fn iqk_result11(&self, reg: u16) -> Result<i32, FaceError> {
        let raw = self.bb_query(reg, 0x07ff_0000)? as i32;
        Ok((raw << 21) >> 21)
    }

    /// IQK MAC setup (`_iqk_configure_mac_8812a`): pause TX, RX antenna off,
    /// CCA off, CCK RX path off — quiesce the MAC so the loopback tone is clean.
    fn iqk_configure_mac(&self) -> Result<(), FaceError> {
        self.page(false)?;
        self.write8(0x522, 0x3f)?;
        self.bb_set(0x550, (1 << 11) | (1 << 3), 0x0)?;
        self.write8(0x808, 0x00)?; // RX ante off
        self.bb_set(0x838, 0xf, 0xc)?; // CCA off
        self.write8(0xa07, 0xf)?; // CCK RX path off
        Ok(())
    }

    /// Apply the measured TX IQ correction matrix (`_iqk_tx_fill_iqc_8812a`).
    /// `dpk_done == false` here, so the BIT(29) gates are always set.
    fn iqk_tx_fill_iqc(&self, path: RfPath, tx_x: i32, tx_y: i32) -> Result<(), FaceError> {
        let x = (tx_x & 0x7ff) as u32;
        let y = (tx_y & 0x7ff) as u32;
        self.page(true)?;
        match path {
            RfPath::A => {
                self.bb_set(0xc90, 1 << 7, 0x1)?;
                self.bb_set(0xcc4, 1 << 18, 0x1)?;
                self.bb_set(0xcc4, 1 << 29, 0x1)?;
                self.bb_set(0xcc8, 1 << 29, 0x1)?;
                self.bb_set(0xccc, 0x7ff, y)?;
                self.bb_set(0xcd4, 0x7ff, x)?;
            }
            RfPath::B => {
                self.bb_set(0xe90, 1 << 7, 0x1)?;
                self.bb_set(0xec4, 1 << 18, 0x1)?;
                self.bb_set(0xec4, 1 << 29, 0x1)?;
                self.bb_set(0xec8, 1 << 29, 0x1)?;
                self.bb_set(0xecc, 0x7ff, y)?;
                self.bb_set(0xed4, 0x7ff, x)?;
            }
        }
        Ok(())
    }

    /// Apply the measured RX IQ correction matrix (`_iqk_rx_fill_iqc_8812a`).
    /// `rx_x`/`rx_y` carry the sign-extended bit pattern (passed unsigned, as in
    /// the C); out-of-range corrections are clamped to the identity-ish default.
    fn iqk_rx_fill_iqc(&self, path: RfPath, rx_x: u32, rx_y: u32) -> Result<(), FaceError> {
        self.page(false)?;
        let reg: u16 = match path {
            RfPath::A => 0xc10,
            RfPath::B => 0xe10,
        };
        let xs = rx_x >> 1;
        let ys = rx_y >> 1;
        if xs >= 0x112 || (0x12..=0x3ee).contains(&ys) {
            self.bb_set(reg, 0x0000_03ff, 0x100)?;
            self.bb_set(reg, 0x03ff_0000, 0)?;
        } else {
            self.bb_set(reg, 0x0000_03ff, xs)?;
            self.bb_set(reg, 0x03ff_0000, ys)?;
        }
        Ok(())
    }

    /// Restore the AFE registers and re-arm the IQ-compensation datapath after
    /// IQK (`_iqk_restore_afe_8812a`, `dpk_done == false` branch).
    fn iqk_restore_afe(&self, afe_bk: &[u32; 12], afe: &[u16; 12]) -> Result<(), FaceError> {
        self.page(false)?;
        for (i, &r) in afe.iter().enumerate() {
            self.write32(r, afe_bk[i])?;
        }
        self.page(true)?;
        self.write32(0xc80, 0x0)?;
        self.write32(0xc84, 0x0)?;
        self.write32(0xc88, 0x0)?;
        self.write32(0xc8c, 0x3c00_0000)?;
        self.bb_set(0xc90, 1 << 7, 0x1)?;
        self.bb_set(0xcc4, 1 << 18, 0x1)?;
        self.bb_set(0xcc4, 1 << 29, 0x1)?;
        self.bb_set(0xcc8, 1 << 29, 0x1)?;
        self.write32(0xe80, 0x0)?;
        self.write32(0xe84, 0x0)?;
        self.write32(0xe88, 0x0)?;
        self.write32(0xe8c, 0x3c00_0000)?;
        self.bb_set(0xe90, 1 << 7, 0x1)?;
        self.bb_set(0xec4, 1 << 18, 0x1)?;
        self.bb_set(0xec4, 1 << 29, 0x1)?;
        self.bb_set(0xec8, 1 << 29, 0x1)?;
        Ok(())
    }

    /// The dual-path (A+B) TX and RX IQ calibration core (`_iqk_tx_8812a`),
    /// specialised to 2.4 GHz / 20 MHz / USB / rfe_type 0 / no ext-PA / no DPK /
    /// VDF off. Injects a 16-tone loopback, polls the IQK engine, averages two
    /// agreeing results per path, then fills the correction matrices.
    fn iqk_tx(&self) -> Result<IqkResult, FaceError> {
        use std::thread::sleep;
        let mut tx_temp = [[0i32; 4]; 10];
        let mut rx_temp = [[0i32; 4]; 10];
        let mut tx_iqc = [0i32; 4];
        let mut rx_iqc = [0i32; 4];
        let (mut tx0_fin, mut tx1_fin) = (false, false);
        let (mut rx0_fin, mut rx1_fin) = (false, false);

        // ── path A+B AFE all on ──
        self.page(false)?;
        self.write32(0xc60, 0x7777_7777)?;
        self.write32(0xc64, 0x7777_7777)?;
        self.write32(0xe60, 0x7777_7777)?;
        self.write32(0xe64, 0x7777_7777)?;
        self.write32(0xc68, 0x1979_1979)?;
        self.write32(0xe68, 0x1979_1979)?;
        self.bb_set(0xc00, 0xf, 0x4)?; // 3-wire off
        self.bb_set(0xe00, 0xf, 0x4)?;
        self.bb_set(0xc5c, (1 << 26) | (1 << 25) | (1 << 24), 0x7)?; // 160 MHz
        self.bb_set(0xe5c, (1 << 26) | (1 << 25) | (1 << 24), 0x7)?;

        // ── TX IQK RF setting, both paths ──
        self.page(false)?;
        for p in [RfPath::A, RfPath::B] {
            self.rf_write(p, 0xef, 0x80002)?;
            self.rf_write(p, 0x30, 0x20000)?;
            self.rf_write(p, 0x31, 0x3fffd)?;
            self.rf_write(p, 0x32, 0xfe83f)?;
            self.rf_write(p, 0x65, 0x931d5)?;
            self.rf_write(p, 0x8f, 0x8a001)?;
        }
        self.write32(0x90c, 0x0000_8000)?;
        self.bb_set(0xc94, 1 << 0, 0x1)?;
        self.bb_set(0xe94, 1 << 0, 0x1)?;
        self.write32(0x978, 0x2900_2000)?;
        self.write32(0x97c, 0xa900_2000)?;
        self.write32(0x984, 0x0046_2910)?;
        self.page(true)?;
        self.write32(0xc88, 0x8214_03f1)?; // ext_pa_5g = 0 (generic dongle, no ext 5G PA)
        self.write32(0xe88, 0x8214_03f1)?;
        // IQK tone LO band-select (0xc8c/0xe8c[30]) — the ONE band-dependent write in the
        // 8812A TX IQK (vendor `_iqk_tx_8812a`: 5G→0x68163e96, 2.4G→0x28163e96; 0xc88 varies
        // by ext-PA not band, 0xc80/0xc84/0xce8 are band-fixed). Running the 2.4G value on a
        // 5 GHz channel injects the IQK tone at the wrong LO, so the TX + loopback RX I/Q
        // correction is solved for the wrong band → RX EVM stays skewed and dense-QAM HT-MCS
        // won't demodulate while robust BPSK legacy OFDM still does (measured 2026-07-24:
        // 8812au on ch149 decoded 206 legacy frames but ~0 HT). Reads `cur_channel`, which
        // `set_channel` stamped before `iq_calibrate` runs in `bring_up_monitor`.
        let iqk_tone_lo = if self.cur_channel.load(std::sync::atomic::Ordering::Relaxed) > 14 {
            0x6816_3e96 // 5 GHz
        } else {
            0x2816_3e96 // 2.4 GHz
        };
        self.write32(0xc8c, iqk_tone_lo)?;
        self.write32(0xe8c, iqk_tone_lo)?;
        self.write32(0xc80, 0x1800_8c10)?; // TX tone idx = 16
        self.write32(0xc84, 0x3800_8c10)?;
        self.write32(0xce8, 0x0)?;
        self.write32(0xe80, 0x1800_8c10)?;
        self.write32(0xe84, 0x3800_8c10)?;
        self.write32(0xee8, 0x0)?;

        // ── TX IQK measurement loop (page stays C1) ──
        let (mut tx0_avg, mut tx1_avg) = (0usize, 0usize);
        let (mut cal0, mut cal1) = (0u8, 0u8);
        loop {
            // one shot
            self.write32(0xcb8, 0x0010_0000)?;
            self.write32(0xeb8, 0x0010_0000)?;
            self.write32(0x980, 0xfa00_0000)?;
            self.write32(0x980, 0xf800_0000)?;
            sleep(Duration::from_millis(10));
            self.write32(0xcb8, 0x0)?;
            self.write32(0xeb8, 0x0)?;

            let mut delay = 0;
            let (mut r0, mut r1) = (false, false);
            loop {
                if !tx0_fin {
                    r0 = self.bb_query(0xd00, 1 << 10)? != 0;
                }
                if !tx1_fin {
                    r1 = self.bb_query(0xd40, 1 << 10)? != 0;
                }
                if (r0 && r1) || delay > 20 {
                    break;
                }
                sleep(Duration::from_millis(1));
                delay += 1;
            }
            if delay < 20 {
                let tx0_fail = self.bb_query(0xd00, 1 << 12)? != 0;
                let tx1_fail = self.bb_query(0xd40, 1 << 12)? != 0;
                if !(tx0_fail || tx0_fin) {
                    self.write32(0xcb8, 0x0200_0000)?;
                    tx_temp[tx0_avg][0] = self.iqk_result11(0xd00)?;
                    self.write32(0xcb8, 0x0400_0000)?;
                    tx_temp[tx0_avg][1] = self.iqk_result11(0xd00)?;
                    tx0_avg += 1;
                } else {
                    cal0 += 1;
                    if cal0 == 10 {
                        break;
                    }
                }
                if !(tx1_fail || tx1_fin) {
                    self.write32(0xeb8, 0x0200_0000)?;
                    tx_temp[tx1_avg][2] = self.iqk_result11(0xd40)?;
                    self.write32(0xeb8, 0x0400_0000)?;
                    tx_temp[tx1_avg][3] = self.iqk_result11(0xd40)?;
                    tx1_avg += 1;
                } else {
                    cal1 += 1;
                    if cal1 == 10 {
                        break;
                    }
                }
            } else {
                cal0 += 1;
                cal1 += 1;
                if cal0 == 10 {
                    break;
                }
            }
            // accept a path once two measurements agree within ±4
            if tx0_avg >= 2 {
                for i in 0..tx0_avg {
                    for ii in (i + 1)..tx0_avg {
                        let dx = tx_temp[i][0] - tx_temp[ii][0];
                        let dy = tx_temp[i][1] - tx_temp[ii][1];
                        if dx > -4 && dx < 4 && dy > -4 && dy < 4 {
                            tx_iqc[0] = (tx_temp[i][0] + tx_temp[ii][0]) / 2;
                            tx_iqc[1] = (tx_temp[i][1] + tx_temp[ii][1]) / 2;
                            tx0_fin = true;
                        }
                    }
                }
            }
            if tx1_avg >= 2 {
                for i in 0..tx1_avg {
                    for ii in (i + 1)..tx1_avg {
                        let dx = tx_temp[i][2] - tx_temp[ii][2];
                        let dy = tx_temp[i][3] - tx_temp[ii][3];
                        if dx > -4 && dx < 4 && dy > -4 && dy < 4 {
                            tx_iqc[2] = (tx_temp[i][2] + tx_temp[ii][2]) / 2;
                            tx_iqc[3] = (tx_temp[i][3] + tx_temp[ii][3]) / 2;
                            tx1_fin = true;
                        }
                    }
                }
            }
            if tx0_fin && tx1_fin {
                break;
            }
            if (cal0 as usize + tx0_avg) >= 10 || (cal1 as usize + tx1_avg) >= 10 {
                break;
            }
        }

        // ── Load LOK (lo leakage cal result) into RF 0x58 ──
        self.page(false)?;
        let lok_a = (self.rf_read(RfPath::A, 0x08)? & 0xffc00) >> 10;
        self.rf_set(RfPath::A, 0x58, 0x7fe00, lok_a)?;
        let lok_b = (self.rf_read(RfPath::B, 0x08)? & 0xffc00) >> 10;
        self.rf_set(RfPath::B, 0x58, 0x7fe00, lok_b)?;
        self.page(true)?;

        // ── RX IQK setup ──
        self.page(false)?;
        if tx0_fin {
            self.rf_write(RfPath::A, 0xef, 0x80000)?;
            self.rf_write(RfPath::A, 0x30, 0x30000)?;
            self.rf_write(RfPath::A, 0x31, 0x3f7ff)?;
            self.rf_write(RfPath::A, 0x32, 0xfe7bf)?;
            self.rf_write(RfPath::A, 0x8f, 0x88001)?;
            self.rf_write(RfPath::A, 0x65, 0x931d1)?;
            self.rf_write(RfPath::A, 0xef, 0x00000)?;
        }
        if tx1_fin {
            self.rf_write(RfPath::B, 0xef, 0x80000)?;
            self.rf_write(RfPath::B, 0x30, 0x30000)?;
            self.rf_write(RfPath::B, 0x31, 0x3f7ff)?;
            self.rf_write(RfPath::B, 0x32, 0xfe7bf)?;
            self.rf_write(RfPath::B, 0x8f, 0x88001)?;
            self.rf_write(RfPath::B, 0x65, 0x931d1)?;
            self.rf_write(RfPath::B, 0xef, 0x00000)?;
        }
        self.bb_set(0x978, 1 << 31, 0x1)?;
        self.bb_set(0x97c, 1 << 31, 0x0)?;
        self.write32(0x90c, 0x0000_8000)?;
        self.write32(0x984, 0x0046_a890)?; // USB interface
        self.write32(0xcb0, 0x7777_7717)?; // rfe_type != 1
        self.write32(0xcb4, 0x0200_0077)?;
        self.write32(0xeb0, 0x7777_7717)?;
        self.write32(0xeb4, 0x0200_0077)?;
        self.page(true)?;
        // RX IQK tone setup. Two values here were mis-transcribed from vendor
        // `_iqk_rx_8812a` and made the RX IQK converge only marginally (one of the two
        // paths per run, alternating — measured on 5 GHz 2026-07-24):
        //   * 0xc88/0xe88 must be 0x0214_0119 — bit 31 is set in the *TX* IQK
        //     (0x8214_03f1) but CLEAR in the RX IQK; it had been left set (0x8214_0119).
        //   * path B uses RX tone byte 0x15 (0x..8c15), not path A's 0x10 — the path-B
        //     writes had been copied from path A (0x..8c10).
        // Both paths now lock, which is what the 5 GHz HT-MCS demod needs.
        if tx0_fin {
            self.write32(0xc80, 0x3800_8c10)?;
            self.write32(0xc84, 0x1800_8c10)?;
            self.write32(0xc88, 0x0214_0119)?;
        }
        if tx1_fin {
            self.write32(0xe80, 0x3800_8c15)?;
            self.write32(0xe84, 0x1800_8c15)?;
            self.write32(0xe88, 0x0214_0119)?;
        }

        // ── RX IQK measurement loop ──
        let (mut rx0_avg, mut rx1_avg) = (0usize, 0usize);
        cal0 = 0;
        cal1 = 0;
        loop {
            // re-apply the TX correction, then one-shot RX
            self.page(false)?;
            if tx0_fin {
                self.bb_set(0x978, 0x03ff_8000, (tx_iqc[0] & 0x7ff) as u32)?;
                self.bb_set(0x978, 0x0000_07ff, (tx_iqc[1] & 0x7ff) as u32)?;
                self.page(true)?;
                self.write32(0xc8c, 0x2816_0cc0)?; // rfe_type != 1
                self.write32(0xcb8, 0x0030_0000)?;
                self.write32(0xcb8, 0x0010_0000)?;
                sleep(Duration::from_millis(5));
                self.write32(0xc8c, 0x3c00_0000)?;
                self.write32(0xcb8, 0x0)?;
            }
            if tx1_fin {
                self.page(false)?;
                self.bb_set(0x978, 0x03ff_8000, (tx_iqc[2] & 0x7ff) as u32)?;
                self.bb_set(0x978, 0x0000_07ff, (tx_iqc[3] & 0x7ff) as u32)?;
                self.page(true)?;
                self.write32(0xe8c, 0x2816_0ca0)?; // rfe_type != 1
                self.write32(0xeb8, 0x0030_0000)?;
                self.write32(0xeb8, 0x0010_0000)?;
                sleep(Duration::from_millis(5));
                self.write32(0xe8c, 0x3c00_0000)?;
                self.write32(0xeb8, 0x0)?;
            }

            let mut delay = 0;
            let (mut r0, mut r1) = (false, false);
            loop {
                if !rx0_fin && tx0_fin {
                    r0 = self.bb_query(0xd00, 1 << 10)? != 0;
                }
                if !rx1_fin && tx1_fin {
                    r1 = self.bb_query(0xd40, 1 << 10)? != 0;
                }
                if (r0 && r1) || delay > 20 {
                    break;
                }
                sleep(Duration::from_millis(1));
                delay += 1;
            }
            if delay < 20 {
                let rx0_fail = self.bb_query(0xd00, 1 << 11)? != 0;
                let rx1_fail = self.bb_query(0xd40, 1 << 11)? != 0;
                if !(rx0_fail || rx0_fin) && tx0_fin {
                    self.write32(0xcb8, 0x0600_0000)?;
                    rx_temp[rx0_avg][0] = self.iqk_result11(0xd00)?;
                    self.write32(0xcb8, 0x0800_0000)?;
                    rx_temp[rx0_avg][1] = self.iqk_result11(0xd00)?;
                    rx0_avg += 1;
                } else {
                    cal0 += 1;
                    if cal0 == 10 {
                        break;
                    }
                }
                if !(rx1_fail || rx1_fin) && tx1_fin {
                    self.write32(0xeb8, 0x0600_0000)?;
                    rx_temp[rx1_avg][2] = self.iqk_result11(0xd40)?;
                    self.write32(0xeb8, 0x0800_0000)?;
                    rx_temp[rx1_avg][3] = self.iqk_result11(0xd40)?;
                    rx1_avg += 1;
                } else {
                    cal1 += 1;
                    if cal1 == 10 {
                        break;
                    }
                }
            } else {
                cal0 += 1;
                cal1 += 1;
                if cal0 == 10 {
                    break;
                }
            }
            if rx0_avg >= 2 {
                'a: for i in 0..rx0_avg {
                    for ii in (i + 1)..rx0_avg {
                        let dx = rx_temp[i][0] - rx_temp[ii][0];
                        let dy = rx_temp[i][1] - rx_temp[ii][1];
                        if dx > -4 && dx < 4 && dy > -4 && dy < 4 {
                            rx_iqc[0] = (rx_temp[i][0] + rx_temp[ii][0]) / 2;
                            rx_iqc[1] = (rx_temp[i][1] + rx_temp[ii][1]) / 2;
                            rx0_fin = true;
                            break 'a;
                        }
                    }
                }
            }
            if rx1_avg >= 2 {
                'b: for i in 0..rx1_avg {
                    for ii in (i + 1)..rx1_avg {
                        let dx = rx_temp[i][2] - rx_temp[ii][2];
                        let dy = rx_temp[i][3] - rx_temp[ii][3];
                        if dx > -4 && dx < 4 && dy > -4 && dy < 4 {
                            rx_iqc[2] = (rx_temp[i][2] + rx_temp[ii][2]) / 2;
                            rx_iqc[3] = (rx_temp[i][3] + rx_temp[ii][3]) / 2;
                            rx1_fin = true;
                            break 'b;
                        }
                    }
                }
            }
            if (rx0_fin || !tx0_fin) && (rx1_fin || !tx1_fin) {
                break;
            }
            if (cal0 as usize + rx0_avg) >= 10
                || (cal1 as usize + rx1_avg) >= 10
                || rx0_avg == 3
                || rx1_avg == 3
            {
                break;
            }
        }

        // ── fill the correction matrices (default = identity-ish 0x200/0x0) ──
        self.iqk_tx_fill_iqc(
            RfPath::A,
            if tx0_fin { tx_iqc[0] } else { 0x200 },
            if tx0_fin { tx_iqc[1] } else { 0x0 },
        )?;
        self.iqk_rx_fill_iqc(
            RfPath::A,
            if rx0_fin { rx_iqc[0] as u32 } else { 0x200 },
            if rx0_fin { rx_iqc[1] as u32 } else { 0x0 },
        )?;
        self.iqk_tx_fill_iqc(
            RfPath::B,
            if tx1_fin { tx_iqc[2] } else { 0x200 },
            if tx1_fin { tx_iqc[3] } else { 0x0 },
        )?;
        self.iqk_rx_fill_iqc(
            RfPath::B,
            if rx1_fin { rx_iqc[2] as u32 } else { 0x200 },
            if rx1_fin { rx_iqc[3] as u32 } else { 0x0 },
        )?;
        Ok(IqkResult {
            tx_a: tx0_fin,
            rx_a: rx0_fin,
            tx_b: tx1_fin,
            rx_b: rx1_fin,
            tx_a_xy: (tx_iqc[0], tx_iqc[1]),
            rx_a_xy: (rx_iqc[0], rx_iqc[1]),
        })
    }

    rung! {
        /// IQ calibration (`_phy_iq_calibrate_8812a`): back up the MAC/BB, AFE and RF
        /// registers it perturbs, run the dual-path TX/RX IQK, then restore. Run
        /// after [`set_channel`](Self::set_channel) (and typically before
        /// [`lc_calibrate`](Self::lc_calibrate)). Corrects TX/RX IQ imbalance —
        /// improves EVM and image rejection.
        fn iq_calibrate(&self) -> Result<IqkResult, FaceError> {
            const MACBB: [u16; 9] = [
                0x520, 0x550, 0x808, 0xa04, 0x90c, 0xc00, 0xe00, 0x838, 0x82c,
            ];
            const AFE: [u16; 12] = [
                0xc5c, 0xc60, 0xc64, 0xc68, 0xcb0, 0xcb4, 0xe5c, 0xe60, 0xe64, 0xe68, 0xeb0, 0xeb4,
            ];
            const RFREG: [u32; 3] = [0x65, 0x8f, 0x0];

            // back up MAC/BB (page C), the C1 one-shot regs, AFE (page C), RF A/B
            self.page(false)?;
            let mut macbb_bk = [0u32; 9];
            for (i, &r) in MACBB.iter().enumerate() {
                macbb_bk[i] = self.read32(r)?;
            }
            self.page(true)?;
            let reg_c1b8 = self.read32(0xcb8)?;
            let reg_e1b8 = self.read32(0xeb8)?;
            self.page(false)?;
            let mut afe_bk = [0u32; 12];
            for (i, &r) in AFE.iter().enumerate() {
                afe_bk[i] = self.read32(r)?;
            }
            let mut rfa_bk = [0u32; 3];
            let mut rfb_bk = [0u32; 3];
            for (i, &r) in RFREG.iter().enumerate() {
                rfa_bk[i] = self.rf_read(RfPath::A, r)?;
                rfb_bk[i] = self.rf_read(RfPath::B, r)?;
            }

            self.iqk_configure_mac()?;
            let result = self.iqk_tx()?;

            // restore RF (both paths), AFE, the C1 one-shot regs, and MAC/BB
            self.page(false)?;
            for (i, &r) in RFREG.iter().enumerate() {
                self.rf_write(RfPath::A, r, rfa_bk[i])?;
            }
            self.rf_write(RfPath::A, 0xef, 0x0)?;
            for (i, &r) in RFREG.iter().enumerate() {
                self.rf_write(RfPath::B, r, rfb_bk[i])?;
            }
            self.rf_write(RfPath::B, 0xef, 0x0)?;

            self.iqk_restore_afe(&afe_bk, &AFE)?;
            self.page(true)?;
            self.write32(0xcb8, reg_c1b8)?;
            self.write32(0xeb8, reg_e1b8)?;
            self.page(false)?;
            for (i, &r) in MACBB.iter().enumerate() {
                self.write32(r, macbb_bk[i])?;
            }
            Ok(result)
        }
    }

    rung! {
        /// Configure both RF paths (`PHY_RFConfig8812` → RF6052): apply the radio-A
        /// and radio-B register tables via the BB LSSI. Run after
        /// [`bb_config`](Self::bb_config) (which powers on the RF analog).
        fn rf_config(&self) -> Result<(), FaceError> {
            self.config_table(RADIO_A, |s, addr, data| {
                s.rf_write_entry(RfPath::A, addr, data)
            })?;
            self.config_table(RADIO_B, |s, addr, data| {
                s.rf_write_entry(RfPath::B, addr, data)
            })?;
            Ok(())
        }
    }

    /// Read an RF register via the BB serial interface (`phy_RFSerialRead`,
    /// jaguar): select PI/SI mode, latch the offset into the HSSI-read register,
    /// then read back the 20-bit value. Verifies the RF serial link round-trips.
    pub fn rf_read(&self, path: RfPath, offset: u32) -> Result<u32, FaceError> {
        // (PI-read reg, SI-read reg, PI-mode bit register).
        let (read_pi, read_si, mode_reg) = match path {
            RfPath::A => (0x0D04u16, 0x0D08u16, 0x0C00u16),
            RfPath::B => (0x0D44, 0x0D48, 0x0E00),
        };
        let offset = offset & 0xff;
        let pi_mode = self.bb_query(mode_reg, 0x4)? != 0;
        // Latch the read address into rHSSIRead_Jaguar (0x8B0), addr field 0xff.
        self.bb_set(0x08B0, 0xff, offset)?;
        let reg = if pi_mode { read_pi } else { read_si };
        // rRead_data_Jaguar = 20-bit data mask.
        self.bb_query(reg, 0x000F_FFFF)
    }

    rung! {
        /// Enable the MAC DMA / WMAC / scheduler / security blocks in `REG_CR`
        /// (`0x063F`). Run **right after** [`power_on`](Self::power_on), before LLT /
        /// firmware / MAC-table — the DMA engines must be on for those to take.
        fn mac_enable_dma(&self) -> Result<(), FaceError> {
            self.write16(REG_CR, 0)?;
            let cr = self.read16(REG_CR)?;
            self.write16(REG_CR, cr | CR_DMA_ENABLE)
        }
    }

    rung! {
        /// Finish MAC init after the MAC register table: reserved-page split,
        /// TX/RX FIFO boundaries, the 3-endpoint queue→priority map, transfer page
        /// size, driver-info size, network type, the **monitor** receive config, and
        /// finally `REG_CR |= MACTXEN | MACRXEN`. After this the MAC is transmitting
        /// and receiving. Read `REG_CR` back to confirm the enable bits.
        fn mac_init_queues(&self) -> Result<(), FaceError> {
            // Reserved pages (RQPN): NPQ first, then the HPQ/LPQ/PUBQ load word.
            self.write8(REG_RQPN_NPQ, 0x00)?;
            self.write32(REG_RQPN, RQPN_3EP)?;

            // TX packet-buffer boundary across the queue/beacon/loopback registers.
            let bndy = TX_PAGE_BOUNDARY;
            self.write8(REG_BCNQ_BDNY, bndy)?;
            self.write8(REG_MGQ_BDNY, bndy)?;
            self.write8(REG_WMAC_LBK_BF_HD, bndy)?;
            self.write8(REG_TRXFF_BNDY, bndy)?;
            self.write8(REG_TDECTRL + 1, bndy)?;

            // RX-FIFO boundary.
            self.write16(REG_TRXFF_BNDY + 2, RX_DMA_BOUNDARY)?;

            // Map the AC queues to the 3 OUT endpoints (preserve the low 3 bits).
            let pri = (self.read16(REG_TRXDMA_CTRL)? & 0x7) | TRXDMA_MAP_3EP;
            self.write16(REG_TRXDMA_CTRL, pri)?;

            // Transfer page size + RX driver-info size.
            self.write8(REG_PBP, PBP_TX_512)?;
            self.write8(REG_RX_DRVINFO_SZ, 4)?; // DRVINFO_SZ (unit 8 B)

            // Network type (MSR) = AP, preserving the rest of REG_CR.
            let cr = self.read32(REG_CR)?;
            self.write32(REG_CR, (cr & !MASK_NETTYPE) | NETTYPE_AP)?;

            // Monitor receive config + accept-all multicast.
            self.write32(REG_RCR, MONITOR_RCR)?;
            self.write32(REG_MAR, 0xFFFF_FFFF)?;
            self.write32(REG_MAR + 4, 0xFFFF_FFFF)?;

            // Enable MAC TX/RX.
            let cr = self.read8(REG_CR)?;
            self.write8(REG_CR, cr | MACTXEN | MACRXEN)?;
            Ok(())
        }
    }

    /// The current `REG_CR` value — read back after `mac_init_queues` to
    /// confirm the MAC TX/RX enable bits are set.
    pub fn read_cr(&self) -> Result<u16, FaceError> {
        self.read16(REG_CR)
    }

    /// `_8051Reset8812`: reset the MCU IO wrapper, pulse the 8051 enable
    /// (`REG_SYS_FUNC_EN+1[2]`), then re-enable the wrapper.
    fn reset_8051(&self) -> Result<(), FaceError> {
        // Reset MCU IO wrapper.
        let t = self.read8(REG_RSV_CTRL)?;
        self.write8(REG_RSV_CTRL, t & !0x02)?;
        let t = self.read8(REG_RSV_CTRL + 1)?;
        self.write8(REG_RSV_CTRL + 1, t & !0x08)?;
        // Pulse the 8051.
        let t = self.read8(REG_SYS_FUNC_EN + 1)?;
        self.write8(REG_SYS_FUNC_EN + 1, t & !0x04)?;
        // Re-enable MCU IO wrapper.
        let t = self.read8(REG_RSV_CTRL)?;
        self.write8(REG_RSV_CTRL, t & !0x02)?;
        let t = self.read8(REG_RSV_CTRL + 1)?;
        self.write8(REG_RSV_CTRL + 1, t | 0x08)?;
        let t = self.read8(REG_SYS_FUNC_EN + 1)?;
        self.write8(REG_SYS_FUNC_EN + 1, t | 0x04)?;
        Ok(())
    }

    // ── Milestone 8: frame injection / capture ───────────────────────────────

    /// Set a `len`-bit field at bit `shift` within the little-endian 32-bit word
    /// at byte `off` of a descriptor (the `SET_BITS_TO_LE_4BYTE` macro).
    fn set_desc_bits(desc: &mut [u8], off: usize, shift: u32, len: u32, val: u32) {
        let mask = if len >= 32 {
            u32::MAX
        } else {
            ((1u32 << len) - 1) << shift
        };
        let mut w = u32::from_le_bytes(desc[off..off + 4].try_into().unwrap());
        w = (w & !mask) | ((val << shift) & mask);
        desc[off..off + 4].copy_from_slice(&w.to_le_bytes());
    }

    /// Build the 40-byte 8812A TX descriptor for a `frame_len`-byte 802.11 frame
    /// at hardware rate `hw_rate` (`rtl8812a_fill_fake_txdesc` field set, with a
    /// forced legacy rate). The USB path drops frames whose descriptor checksum
    /// is wrong, so it is computed last over the first 32 bytes.
    fn build_txdesc(frame_len: usize, hw_rate: u32, qsel: u32) -> [u8; TXDESC_SIZE] {
        let mut d = [0u8; TXDESC_SIZE];
        Self::set_desc_bits(&mut d, 0, 0, 16, frame_len as u32); // PKT_SIZE
        Self::set_desc_bits(&mut d, 0, 16, 8, TXDESC_SIZE as u32); // OFFSET
        Self::set_desc_bits(&mut d, 0, 26, 1, 1); // LAST_SEG
        Self::set_desc_bits(&mut d, 0, 27, 1, 1); // FIRST_SEG
        Self::set_desc_bits(&mut d, 0, 31, 1, 1); // OWN
        Self::set_desc_bits(&mut d, 4, 8, 5, qsel); // QUEUE_SEL
        Self::set_desc_bits(&mut d, 4, 16, 5, RATEID_IDX_G); // RATE_ID
        Self::set_desc_bits(&mut d, 12, 8, 1, 1); // USE_RATE (forced rate)
        Self::set_desc_bits(&mut d, 16, 0, 7, hw_rate); // TX_RATE
        // Send each frame exactly once. Every peer this radio injects to is in
        // monitor mode and so never ACKs; left to itself the MAC reads that as
        // loss and retransmits a *unicast* frame up to its retry limit. A monitor
        // receiver does no duplicate filtering, so one datagram arrives many times
        // over while the frames queued behind it starve. Measured on air before
        // this: 20 receives of a single datagram, and none of the 19 after it.
        // Broadcast was never affected — nothing ACKs those either, so the MAC
        // already sent them once.
        Self::set_desc_bits(&mut d, 16, 17, 1, 1); // RETRY_LIMIT_ENABLE
        Self::set_desc_bits(&mut d, 16, 18, 6, 0); // DATA_RETRY_LIMIT = 0
        // HWSEQ_EN (HW sequence #) — unconditional, and the question of turning it off is CLOSED.
        //
        // ☠ **Refuted hypothesis, do not re-open without new evidence.** `tier0.rs` reserved SeqCtrl
        // for "LP reassembly" and the RX header walk in `ndn-frame-io` never reads it (it steps over
        // SeqCtrl via `hdr_len`), which made those 16 bits look like recyclable filter payload — the
        // route to a 206-bit filter. MEASURED 2026-09-03: with HWSEQ_EN cleared and a distinctive
        // sequence number `0xABC` injected, all **6,214** captured frames read `wlan.seq = 0`, while
        // *the same frames* carried our chosen Duration correctly (so the run was configured as
        // intended). The host's sequence number does not reach the air on this part. Independently,
        // the bring-up inventory found **four of our six Wi-Fi TX paths overwrite SeqCtrl anyway**,
        // two of them in silicon — so it can never be a fleet-wide field.
        //
        // The temporary `NDN_NO_HWSEQ` gate that tested this is deleted rather than left behind: a
        // knob that disables hardware sequence numbering, kept for a question already answered, is
        // a trap for whoever finds it next. See `ndn-phy-wifi/docs/p8-header-passthrough.md`.
        Self::set_desc_bits(&mut d, 32, 15, 1, 1);
        // #96 measurement gate: NAVUSEHDR (DWORD3 bit15) tells the MAC to take the
        // NAV/Duration from the MPDU header instead of computing its own. Off by
        // default (the MAC overwrites Duration — measured); set NDN_NAVUSEHDR=1 to
        // A/B whether the injected Duration then survives onto the air.
        if std::env::var_os("NDN_NAVUSEHDR").is_some() {
            Self::set_desc_bits(&mut d, 12, 15, 1, 1); // NAV_USE_HDR
        }
        // ★ **Per-frame CCX TX report** — the arm for this radio's only transmit instrument.
        //
        // The 8812AU has no `read_tx_counters()`: every "throughput" it has ever reported is an
        // *offered* rate, the host's `inject` call count, which a bulk-OUT queue will happily
        // absorb whether or not the MAC ever keyed the RF. The firmware image DOES build a
        // per-frame report — code 0x7A38 drains a 16-slot × 8-byte FIFO at XDATA 0x8000, indexed
        // by MAC `0x047E` (hardware write) / `0x047F` (firmware read), and emits it as C2H id
        // `0x03` — but nothing in this driver ever asked for one.
        //
        // The request is a *descriptor* bit, not an H2C command and not a register write. Three
        // independent sources agree on the two fields:
        //   * mainline rtw88 `tx.h:37`  `RTW_TX_DESC_W2_SPE_RPT   = BIT(19)` of word 2 (desc+8),
        //     set from `pkt_info->report` at `tx.c:57`;
        //   * mainline rtw88 `tx.h:53`  `RTW_TX_DESC_W6_SW_DEFINE = GENMASK(11,0)` of word 6
        //     (desc+24) — the tag the firmware echoes back, built at `tx.c:171` as
        //     `(sn << 2) & 0xfc` ([7:2] sequence, [1:0] reserved for firmware);
        //   * the 8812A-specific vendor header agrees exactly:
        //     `SET_TX_DESC_SPE_RPT_8812(d) = d+8 bit 19`,
        //     `SET_TX_DESC_SW_DEFINE_8812(d) = d+24 [11:0]`.
        // And `C2H_CCX_TX_RPT = 0x03` is rtw88 `fw.h:52`, chip-independent, matching the id the
        // blob's builder writes.
        //
        // ⚠ Off by default and UNMEASURED on this part: no capture we hold shows a report coming
        // back, and `rtl8xxxu core.c:4168` notes bit 0 of `REG_TX_REPORT_CTRL` (0x04EC) is
        // "required for both types" while setting it only on the 8188E — so this bit alone may or
        // may not be sufficient here. Set `NDN_AU_TXRPT=1`, run with `NDN_C2H_DBG=1`, and look for
        // `C2H8812AU id=0x03 len=8`. Both fields ride inside the descriptor checksum below, so
        // they must be written before it is computed — they are.
        // ⚠ SAMPLE, do not ask for every frame. The hardware CCX FIFO is 16 slots of 8 bytes at
        // XDATA 0x8000 and the firmware's C2H ring is 10 slots — at flood rates both overflow, and
        // the firmware's only signal for that is `0x01C1 |= 0x02` (code 0x7530), a sticky latch the
        // host must read. `NDN_AU_TXRPT=N` requests a report on 1 frame in N (N=1 for every frame).
        if let Some(n) = std::env::var("NDN_AU_TXRPT")
            .ok()
            .and_then(|v| v.trim().parse::<u32>().ok())
            .filter(|&n| n > 0)
        {
            static TXRPT_SN: std::sync::atomic::AtomicU32 = std::sync::atomic::AtomicU32::new(0);
            let seq = TXRPT_SN.fetch_add(1, std::sync::atomic::Ordering::Relaxed);
            if seq % n == 0 {
                // rtw88 tx.c:171 tag layout: [7:2] sequence, [1:0] reserved for firmware.
                Self::set_desc_bits(&mut d, 8, 19, 1, 1); // W2 SPE_RPT — report this frame
                Self::set_desc_bits(&mut d, 24, 0, 12, ((seq / n) << 2) & 0xfc); // W6 SW_DEFINE
            }
        }
        let mut csum: u16 = 0;
        for i in 0..16 {
            csum ^= u16::from_le_bytes([d[i * 2], d[i * 2 + 1]]);
        }
        Self::set_desc_bits(&mut d, 28, 0, 16, csum as u32); // TX_DESC_CHECKSUM
        d
    }

    /// Frame an 802.11 `frame` for bulk-OUT: descriptor ++ frame, padded off the
    /// USB 512-byte boundary (Realtek drops a bulk-out of exactly N×512 bytes).
    fn tx_buffer(frame: &[u8], hw_rate: u32) -> Vec<u8> {
        // Queue by what the frame *is*. Everything used to ride QSLT_MGNT, which
        // maps to the HIGH queue and its 16 reserved HPQ pages — sized for small
        // management frames. A data frame larger than that pool silently stops
        // going out: measured, NDN objects delivered 12/12 up to 1400 B and 0/12
        // from 2200 B, with 2200 B still a *single* fragment.
        let qsel = match frame.first() {
            Some(&fc0) if is_data_frame(fc0) => QSLT_BE,
            _ => QSLT_MGNT, // beacons / action frames (NAN) belong here
        };
        let desc = Self::build_txdesc(frame.len(), hw_rate, qsel);
        let mut buf = Vec::with_capacity(TXDESC_SIZE + frame.len() + 8);
        buf.extend_from_slice(&desc);
        buf.extend_from_slice(frame);
        if buf.len() % 512 == 0 {
            buf.extend_from_slice(&[0u8; 8]);
        }
        buf
    }

    /// Inject a complete 802.11 `frame` (management header onward) at `hw_rate`
    /// — synchronous bulk-OUT to the MGNT-queue endpoint. Fire-and-forget.
    pub fn send_frame(&self, frame: &[u8], hw_rate: u32) -> Result<(), FaceError> {
        let buf = Self::tx_buffer(frame, hw_rate);
        self.handle
            .write_bulk(self.bulk_out, &buf, TX_TIMEOUT)
            .map_err(usb_err)?;
        Ok(())
    }

    /// Arm **modulated continuous TX** (port of devourer jaguar1 `StartContinuousTx`,
    /// itself `hal_mpt_SetSingleToneTx`/`mpt_StartOfdmContTx`). After arming, injecting
    /// a frame makes the 8812A radiate a **continuous OFDM carrier** at the current
    /// TXAGC power — a steady 100%-duty signal for conducted power measurement, with
    /// none of the frame-injection duty gaps. Output power still tracks
    /// [`set_tx_power`](Self::set_tx_power) (the TXAGC path). End with
    /// [`stop_continuous_tx`](Self::stop_continuous_tx). Call after channel + power set.
    pub fn start_continuous_tx(&self) -> Result<(), FaceError> {
        self.bb_set(0x800, 0x0200_0000, 1)?; // rFPGA0_RFMOD[bOFDMEn] = 1 (OFDM block on)
        self.bb_set(0xa00, 0x3, 0)?; // rCCK0_System[bCCKBBMode] = 0 (CCK test mode off)
        self.bb_set(0xa00, 0x8, 1)?; // [bCCKScramble] = 1
        self.bb_set(0x914, 0x7_0000, 1)?; // 0x914[18:16] = OFDM_ContinuousTx
        self.write32(0x820, 0x0100_0500)?; // rFPGA0_XA_HSSIParameter1 (vendor cont-TX value)
        self.write32(0x828, 0x0100_0500)?; // rFPGA0_XB_HSSIParameter1
        Ok(())
    }

    /// Stop continuous TX and restore the BB (port of `StopContinuousTx`).
    pub fn stop_continuous_tx(&self) -> Result<(), FaceError> {
        self.bb_set(0x914, 0x7_0000, 0)?;
        std::thread::sleep(std::time::Duration::from_millis(10));
        self.bb_set(0x100, 0x100, 0)?; // rPMAC_Reset[bBBResetB] pulse
        self.bb_set(0x100, 0x100, 1)?;
        self.write32(0x820, 0x0100_0100)?;
        self.write32(0x828, 0x0100_0100)?;
        Ok(())
    }

    /// Read one physical EFUSE byte via `REG_EFUSE_CTRL` (port of devourer
    /// `RtlAdapter::ReadEFuseByte`): per-read `EFUSE_TEST` clear, write the address,
    /// clear the read flag, poll the ready bit, return the data byte.
    fn read_efuse_byte(&self, offset: u16) -> Result<u8, FaceError> {
        let _ = self.read16(REG_EFUSE_TEST)?;
        self.write16(REG_EFUSE_TEST, 0x0000)?;
        self.write8(REG_EFUSE_CTRL + 1, (offset & 0xff) as u8)?;
        let hi = self.read8(REG_EFUSE_CTRL + 2)?;
        self.write8(
            REG_EFUSE_CTRL + 2,
            (((offset >> 8) & 0x03) as u8) | (hi & 0xfc),
        )?;
        let b3 = self.read8(REG_EFUSE_CTRL + 3)?;
        self.write8(REG_EFUSE_CTRL + 3, b3 & 0x7f)?; // bit31=0 => request READ
        let mut v = self.read32(REG_EFUSE_CTRL)?;
        let mut retry = 0;
        while (v >> 24) & 0x80 == 0 && retry < 10000 {
            v = self.read32(REG_EFUSE_CTRL)?;
            retry += 1;
        }
        let v = self.read32(REG_EFUSE_CTRL)?;
        Ok((v & 0xff) as u8)
    }

    /// EFUSE read-access power switch (read side of `EfusePowerSwitch8812A`): grant
    /// access, ensure the ELDR reset + loader/8M clocks are on.
    fn efuse_power_switch(&self, on: bool) -> Result<(), FaceError> {
        if on {
            self.write8(REG_EFUSE_BURN_GNT_8812, EFUSE_ACCESS_ON_JAGUAR)?;
            let fe = self.read16(REG_SYS_FUNC_EN)?;
            if fe & FEN_ELDR == 0 {
                self.write16(REG_SYS_FUNC_EN, fe | FEN_ELDR)?;
            }
            let clk = self.read16(REG_SYS_CLKR)?;
            if clk & LOADER_CLK_EN == 0 || clk & ANA8M == 0 {
                self.write16(REG_SYS_CLKR, clk | LOADER_CLK_EN | ANA8M)?;
            }
        } else {
            self.write8(REG_EFUSE_BURN_GNT_8812, EFUSE_ACCESS_OFF_JAGUAR)?;
        }
        Ok(())
    }

    /// Read + decode the logical 512-byte EFUSE map (port of
    /// `Hal_EfuseReadEFuse8812A`): walk the physical PG headers (plain + extended),
    /// gather the enabled words into 64 sections × 4 word-units, flatten to bytes.
    fn read_efuse_map(&self) -> Result<[u8; EFUSE_MAP_LEN_JAGUAR], FaceError> {
        self.efuse_power_switch(true)?;
        let mut words = [[0xffffu16; EFUSE_MAX_WORD_UNIT]; EFUSE_MAX_SECTION_JAGUAR];
        let r = (|| -> Result<(), FaceError> {
            let mut addr: u16 = 0;
            let mut hdr = self.read_efuse_byte(addr)?;
            if hdr == 0xff {
                return Ok(()); // empty fuse
            }
            addr += 1;
            while hdr != 0xff && addr < EFUSE_REAL_CONTENT_LEN_JAGUAR {
                let offset: u8;
                let mut wren: u8;
                if hdr & 0x1f == 0x0f {
                    // extended header: second byte carries offset[hi]|wren.
                    let u1 = (hdr & 0xe0) >> 5;
                    hdr = self.read_efuse_byte(addr)?;
                    if hdr & 0x0f == 0x0f {
                        addr += 1;
                        hdr = self.read_efuse_byte(addr)?;
                        if hdr != 0xff && addr < EFUSE_REAL_CONTENT_LEN_JAGUAR {
                            addr += 1;
                        }
                        continue;
                    }
                    offset = ((hdr & 0xf0) >> 1) | u1;
                    wren = hdr & 0x0f;
                    addr += 1;
                } else {
                    offset = (hdr >> 4) & 0x0f;
                    wren = hdr & 0x0f;
                }
                for i in 0..EFUSE_MAX_WORD_UNIT {
                    if wren & 0x01 == 0 {
                        let lo = self.read_efuse_byte(addr)?;
                        addr += 1;
                        if (offset as usize) < EFUSE_MAX_SECTION_JAGUAR {
                            words[offset as usize][i] = lo as u16;
                        }
                        if addr >= EFUSE_REAL_CONTENT_LEN_JAGUAR {
                            break;
                        }
                        let hi = self.read_efuse_byte(addr)?;
                        addr += 1;
                        if (offset as usize) < EFUSE_MAX_SECTION_JAGUAR {
                            words[offset as usize][i] |= (hi as u16) << 8;
                        }
                        if addr >= EFUSE_REAL_CONTENT_LEN_JAGUAR {
                            break;
                        }
                    }
                    wren >>= 1;
                }
                hdr = self.read_efuse_byte(addr)?;
                if hdr != 0xff && addr < EFUSE_REAL_CONTENT_LEN_JAGUAR {
                    addr += 1;
                }
            }
            Ok(())
        })();
        self.efuse_power_switch(false)?;
        r?;
        let mut map = [0u8; EFUSE_MAP_LEN_JAGUAR];
        for i in 0..EFUSE_MAX_SECTION_JAGUAR {
            for j in 0..EFUSE_MAX_WORD_UNIT {
                map[i * 8 + j * 2] = (words[i][j] & 0xff) as u8;
                map[i * 8 + j * 2 + 1] = ((words[i][j] >> 8) & 0xff) as u8;
            }
        }
        Ok(map)
    }

    /// Read the EFUSE and parse this adapter's per-rate TX-power calibration into
    /// `Self::tx_power_info` (port of `LoadTxPowerInfo`). After this, `set_tx_power`
    /// references the chip's *fused* full-power point per channel/rate. Best-effort:
    /// on a USB error the info stays unset and `set_tx_power` uses the flat fallback.
    /// Run after the MAC is up (needs register access). Idempotent.
    pub fn load_tx_power_info(&self) -> Result<(), FaceError> {
        let map = self.read_efuse_map()?;
        // Base cell fallback: an unprogrammed cell (>63) uses the vendor generic
        // default, whose 2.4G/5G base bytes are all 0x2d (`kPgTxpwrDefGeneric`).
        let read_base = |o: usize| -> u8 {
            let v = map[o];
            if v <= TXAGC_MAX { v } else { 0x2d }
        };
        let mut info = TxPowerInfo::zeroed();
        let mut cck_g = [[0u8; 6]; 2];
        let mut bw40_g = [[0u8; 6]; 2];
        let mut bw40_5g_g = [[0u8; 14]; 2];
        let mut off = PG_TXPWR_SADDR;
        for path in 0..2 {
            // ── 2.4G (18 bytes): 6 CCK base, 5 BW40 base, then per-Ntx diffs ──
            for g in cck_g[path].iter_mut() {
                *g = read_base(off);
                off += 1;
            }
            for g in bw40_g[path].iter_mut().take(5) {
                *g = read_base(off);
                off += 1;
            }
            let v = map[off];
            off += 1;
            info.bw20_2g_diff[path][0] = pg_msb_diff(v);
            info.ofdm_2g_diff[path][0] = pg_lsb_diff(v);
            for t in 1..4 {
                let v = map[off];
                off += 1;
                info.bw40_2g_diff[path][t] = pg_msb_diff(v);
                info.bw20_2g_diff[path][t] = pg_lsb_diff(v);
                let v = map[off];
                off += 1;
                info.ofdm_2g_diff[path][t] = pg_msb_diff(v);
                info.cck_2g_diff[path][t] = pg_lsb_diff(v);
            }
            // ── 5G (24 bytes): 14 BW40 base, per-Ntx diffs, BW80 ──
            for g in bw40_5g_g[path].iter_mut() {
                *g = read_base(off);
                off += 1;
            }
            let v = map[off];
            off += 1;
            info.bw20_5g_diff[path][0] = pg_msb_diff(v);
            info.ofdm_5g_diff[path][0] = pg_lsb_diff(v);
            for t in 1..4 {
                let v = map[off];
                off += 1;
                info.bw40_5g_diff[path][t] = pg_msb_diff(v);
                info.bw20_5g_diff[path][t] = pg_lsb_diff(v);
            }
            let v = map[off];
            off += 1;
            info.ofdm_5g_diff[path][1] = pg_msb_diff(v);
            info.ofdm_5g_diff[path][2] = pg_lsb_diff(v);
            let v = map[off];
            off += 1;
            info.ofdm_5g_diff[path][3] = pg_lsb_diff(v);
            for t in 0..4 {
                let v = map[off];
                off += 1;
                info.bw80_5g_diff[path][t] = pg_msb_diff(v);
            }
        }
        // Scatter per-group bases to per-channel (Stage 2 of hal_load_txpwr_info).
        for path in 0..2 {
            for ch_idx in 0..14 {
                if let Some((0, group, cck_group)) = classify_channel((ch_idx + 1) as u8) {
                    info.cck_base_2g[path][ch_idx] = cck_g[path][cck_group as usize];
                    info.bw40_base_2g[path][ch_idx] = bw40_g[path][group as usize];
                }
            }
            for ch_idx in 0..65 {
                if let Some((1, group, _)) = classify_channel(CENTER_CH_5G[ch_idx]) {
                    info.bw40_base_5g[path][ch_idx] = bw40_5g_g[path][group as usize];
                }
            }
        }
        if std::env::var("NDN_TXPWR_DBG").is_ok() {
            let ch = self.cur_channel.load(std::sync::atomic::Ordering::Relaxed);
            eprintln!(
                "TXPWR8812 efuse[0x10..0x22]={:02x?} ch{ch}: A OFDM base={} CCK base={} | B OFDM base={}",
                &map[0x10..0x22],
                info.index_base(0, 0x0c, 0, 0, ch.max(1)),
                info.index_base(0, 0x02, 0, 0, ch.max(1)),
                info.index_base(1, 0x0c, 0, 0, ch.max(1)),
            );
        }
        *self.tx_power_info.lock().unwrap() = Some(info);
        Ok(())
    }

    /// **Set the TX power, and say what that meant.** ([`PowerRequest`] in, [`AppliedPower`] out.)
    ///
    /// ☠☠ **This is the function the 2026-09-03 bring-up contract exists for.** Its old form —
    /// `set_tx_power(idx: u8) -> Result<(), FaceError>` — meant **two different physical powers**,
    /// decided by whether [`load_tx_power_info`](Self::load_tx_power_info) had run three calls
    /// earlier:
    ///
    /// * calibration loaded → `index_base(path, rate, ch) + (idx − 63)` ≈ **27** on the ch6 adapter;
    /// * not loaded → **it fell through to `set_tx_power_raw(idx)`** — a flat **63** on ten
    ///   registers. ⚠ The SIZE of the step is channel-dependent and UNVERIFIED: the fused base is 27
    ///   on ch6 and 44 on ch149, and MEASURED 2026-09-04 at ch149 the difference is ~0 dB.
    ///
    /// Same call, same argument, same `Ok(())`. MEASURED at an AR9271 witness: raw 63 → 2301 frames
    /// at −85.6 dBm; raw 55 → **0**. `examples/nav_probe.rs` never loaded calibration (ran hot,
    /// "worked"); `bring_up_monitor` did (ran at the fused base, "did not work"); the node binary
    /// goes through the calibrated path — so the shipped node and every bench example **were not
    /// the same transmitter**, and nothing said so.
    ///
    /// **LAW 2 — the fallthrough is deleted.** Ask for the calibrated scale with no calibration
    /// resolved and this returns `Err` naming [`PowerRequest::Raw`]. It never silently becomes raw.
    ///
    /// **LAW 3 — no environment read.** `NDN_AU_TXAGC12` used to be read from *inside this
    /// function* and silently turned 10 register writes into 24; it is now
    /// `RateGroupPolicy` on the request and appears in [`AppliedPower::writes`].
    ///
    /// On the calibrated scale `idx = 63` **is** the fused regulatory base (`offset = idx − 63`),
    /// which is why `RadioCapability::max_tx_power` stays 63 and is not renumbered to ~27 — see
    /// the contract's §2.1 and Appendix A.1.
    ///
    /// Registers: the 8812A power-by-rate `rTxAGC_*_JAguar` at `0xC2x` (path A) / `0xE2x` (path B).
    /// The BB **TX swing** (`0xC1C[31:21]`) is a per-band constant the channel tables program, not
    /// this knob.
    pub fn set_tx_power(&self, req: PowerRequest) -> Result<AppliedPower, FaceError> {
        let ch = self.cur_channel.load(std::sync::atomic::Ordering::Relaxed);
        match &req {
            // This part has an actuator, and a caller claiming otherwise has the wrong part.
            PowerRequest::NoActuator => Err(power_unsupported(
                "rtl8812au: PowerRequest::NoActuator, but this part DOES actuate power \
                 (per-rate Jaguar1 TXAGC). Use PowerRequest::Ceiling or ::Index.",
            )),
            // ⚠ No dBm axis on this part: `RadioCapability::tx_power_dbm` is None, and the
            // register→power transfer is monotone but uncalibrated in absolute terms. Refusing by
            // name is the honest answer; an invented dBm would be believed by a link budget.
            PowerRequest::Dbm(d) => Err(power_unsupported(format!(
                "rtl8812au: PowerRequest::Dbm({d}) — this part has no absolute dBm axis \
                 (RadioCapability::tx_power_dbm is None; the TXAGC transfer is monotone over ~33 dB \
                 but has no measured absolute anchor). Use PowerRequest::Index on the calibrated \
                 scale.",
            ))),
            // ⚠ **Off the regulatory scale**, and only reachable with an `RfAuthority` — which no
            // library code can mint. This is the regime the sixteen hand-rolled examples were in
            // without knowing it; now it is a request, an authority, and a `PowerReference`.
            PowerRequest::Raw { idx, authority } => {
                let idx = (*idx).min(TXAGC_MAX);
                tracing::warn!(
                    target: "radio",
                    part = "rtl8812au",
                    idx,
                    granted_by = authority.granted_by(),
                    reason = authority.reason(),
                    "TX power on the RAW chip TXAGC axis — calibration bypassed, may exceed \
                     licensed EIRP"
                );
                let writes = self.write_txagc_flat(idx)?;
                Ok(AppliedPower::from_writes(
                    req.clone(),
                    PowerReference::ChipRaw,
                    idx,
                    false,
                    writes,
                ))
            }
            PowerRequest::Ceiling(policy) | PowerRequest::Index(_, policy) => {
                let want = match &req {
                    PowerRequest::Index(i, _) => *i,
                    _ => TXAGC_MAX,
                };
                let idx = want.min(TXAGC_MAX);
                let clamped = idx != want;
                let policy = *policy;
                let guard = self.tx_power_info.lock().unwrap();
                let Some(info) = guard.as_ref() else {
                    // ★★ **THE FIX.** This used to be `return self.set_tx_power_raw(idx);` at
                    // `rtl8812au.rs:6595` — the same call, the same argument, the same `Ok(())`,
                    // in a different power regime from, decided by a step three calls earlier the caller could not
                    // see. Deleting this one fallthrough is the whole repair; everything else in
                    // `bringup.rs` exists so it cannot come back in a different shape.
                    return Err(power_unsupported(NO_CALIBRATION_MSG));
                };
                if ch == 0 {
                    // Power on this part is per-channel: `index_base` needs a tuned channel.
                    // Refusing beats writing a base for channel 0, which does not exist.
                    return Err(power_unsupported(
                        "rtl8812au: TX power on the calibrated scale before set_channel — the \
                         fused base is per-channel and there is no channel to look it up for.",
                    ));
                }
                let offset = idx as i32 - TXAGC_MAX as i32; // 0 at full, negative = backoff
                // (path-A reg, path-B reg, representative MGN_* rate, group name) per group.
                //
                // ★ **All twelve Jaguar1 per-rate TXAGC groups.**
                //
                // Provenance, because an earlier attempt at this was reverted on a MISATTRIBUTION
                // and the reasoning must not be repeated:
                //
                //  * **Decisive**: THIS DRIVER ALREADY WRITES ALL 24 OF THESE REGISTERS on every
                //    5 GHz channel program — `PROGS_5G` contains 0x0c20..0x0c4c and
                //    0x0e20..0x0e4c, and this silicon has taken those writes since the port was
                //    written, MEASURED at 490-1258 f/s.
                //  * `fw/rtl8812au/rtl8812au_phy_reg.bin` boots all 24 at `0x12121212` — the flat
                //    four-byte TXAGC packing (index 18 x4), not arbitrary BB values.
                //  * `golden/rtw88-2g-ch6.txt` — the MAINLINE KERNEL driving THIS dongle — writes
                //    `0c34 = 1c1c1c1c` and `0c38 = 1c1a1816`: a per-rate ladder descending across
                //    rates, which is what TXAGC looks like and what BB config does not.
                //  * `Hal8812PhyReg.h:178` names 0xc34 `rTxAGC_A_MCS11_MCS8_JAguar`.
                //
                // ☠ **Where the earlier error came from**, recorded so nobody repeats it: the SAME
                // vendor header gives these addresses 11N names at lines 472-485
                // (`rOFDM0_RxDetector2` 0xc34, `rOFDM0_ECCAThreshold` 0xc4c). Reading the 11N name
                // and concluding "live baseband register on Jaguar1" is wrong — Jaguar re-uses the
                // block for per-rate TXAGC. The throughput collapse blamed on this change was in
                // fact the frame-format bug (a malformed UNICAST `addr1` driving the retry ladder,
                // see `examples/rtl_contention_ab.rs`), which was present before and after it.
                //
                // Rate codes: `ODM_MGN_MCS0 = 0x80`, `ODM_MGN_VHT1SS_MCS0 = 0xa0` — the same
                // classification `mcs_diff_sum` above uses, so base and register agree on what a
                // rate means.
                const GROUPS: [(u16, u16, u8, &str); 12] = [
                    (0xc20, 0xe20, 0x02, "CCK 11-1"),
                    (0xc24, 0xe24, 0x0c, "OFDM 18-6"),
                    (0xc28, 0xe28, 0x30, "OFDM 54-24"),
                    (0xc2c, 0xe2c, 0x80, "HT MCS0-3"),
                    (0xc30, 0xe30, 0x84, "HT MCS4-7"),
                    (0xc34, 0xe34, 0x88, "HT MCS8-11 (2SS)"),
                    (0xc38, 0xe38, 0x8c, "HT MCS12-15 (2SS)"),
                    (0xc3c, 0xe3c, 0xa0, "VHT 1SS MCS0-3"),
                    (0xc40, 0xe40, 0xa4, "VHT 1SS MCS4-7"),
                    (0xc44, 0xe44, 0xa8, "VHT 1SS MCS8-9 + 2SS MCS0-1"),
                    (0xc48, 0xe48, 0xac, "VHT 2SS MCS2-5"),
                    (0xc4c, 0xe4c, 0xb0, "VHT 2SS MCS6-9"),
                ];
                // ☠☠ **FIVE groups. MEASURED AGAINST A WITNESS RECEIVER — this is settled.**
                //
                //     12 groups: offered 711 f/s,  ON AIR   7 frames in 11 s =   1 f/s
                //      5 groups: offered 2805 f/s, ON AIR 2704 frames in 11 s = 246 f/s
                //
                // Writing the seven HT2SS/VHT groups **stops this radio transmitting.** A 250x
                // difference on air — which the offered rate could not see at all (711 vs 2805
                // reads as a modest difference), and which two rounds of careful source-reading got
                // wrong in both directions.
                //
                // ⚠ The addresses are NOT the issue (see the provenance above). What differs is the
                // VALUE: `PROGS_5G` replays captured ladders, while this writes
                // `index_base(..) + offset` — and for the 2SS/VHT rate codes that computation
                // evidently lands somewhere this silicon will not transmit from. Closing the gap
                // needs the value investigated, not the address list re-argued.
                //
                // ★ **This used to be an `NDN_AU_TXAGC12` environment read, performed from inside
                // this function.** A knob whose meaning depends on the environment is hidden state by
                // another name — the same disease as `load_tx_power_info`, one level down — and it
                // silently turned 10 register writes into 24. It is now
                // `RateGroupPolicy::AllTwelveUnderInvestigation` on the request, and the count of
                // writes in the returned `AppliedPower` shows which policy ran. Do not select it
                // without a witness receiver in the loop: the transmitter cannot detect this.
                let groups = &GROUPS[..policy.group_pairs()];
                let mut writes = Vec::with_capacity(groups.len() * 2);
                for &(reg_a, reg_b, rate, name) in groups {
                    for (path, reg) in [(0usize, reg_a), (1usize, reg_b)] {
                        let base = info.index_base(path, rate, 0, 0, ch) as i32;
                        // ☠ **No floor here, and that is MEASURED, not an oversight.**
                        //
                        // The RF-frontend plan recommended clamping the calibrated write to a
                        // non-zero minimum, reasoning that a fused base of ~27 plus an 18 dB
                        // back-off lands exactly on index 0. Tried on 2026-08-28 with a floor of
                        // 4 and A/B'd on air: **33-50 f/s with the floor, 1258 f/s without it** —
                        // a >25x collapse, on ch36/149/6 alike. Forcing a rate's index up to the
                        // floor where `index_base` legitimately returns something lower (it
                        // returns 0 for CCK on 5 GHz outright) is actively harmful on this part.
                        //
                        // If the index-0 landing turns out to matter, the fix belongs in the
                        // POLICY — `RadioCapability::min_tx_power`, which exists for exactly this
                        // and is honoured by `decide_power` — not in a blanket driver clamp that
                        // cannot tell a legitimately-low base from an over-deep back-off.
                        let v = (base + offset).clamp(0, TXAGC_MAX as i32) as u8;
                        self.write32(reg, u32::from_le_bytes([v, v, v, v]))?;
                        writes.push(PowerWrite {
                            reg: reg as u32,
                            value: v,
                            group: name,
                            path: path as u8,
                        });
                    }
                }
                // The representative fused base a reader compares against: path A, OFDM 18-6, this
                // channel. It is a FACT about the adapter, not a renumbering of the API scale.
                let base_index = info.index_base(0, 0x0c, 0, 0, ch);
                Ok(AppliedPower::from_writes(
                    req.clone(),
                    PowerReference::FusedBase {
                        base_index,
                        channel: ch,
                    },
                    idx,
                    clamped,
                    writes,
                ))
            }
        }
    }

    /// Write the **flat/raw** TXAGC index `idx` (0–63) to every per-rate register on both paths,
    /// bypassing the EFUSE regulatory calibration, and return what it wrote.
    ///
    /// ☠ **Renamed from `set_tx_power_raw`, and that rename is load-bearing.** Under the old name
    /// it was a *second power API*, and `set_tx_power` fell through to it — which is exactly the
    /// 2026-09-03 defect. It is now a register writer with a register writer's name, called only
    /// from [`set_tx_power`](Self::set_tx_power)'s [`PowerRequest::Raw`] arm, and
    /// `tests/power_has_one_meaning.rs` fails the build if any `fn set_tx_power*` reaches for a
    /// `set_tx_power_raw`/`set_tx_power_idx` sibling again.
    ///
    /// ☠☠ **`pub(crate)`, and that is compiler enforcement rather than discipline.** MEASURED
    /// 2026-09-03: while this was `pub`, `backend.write_txagc_flat(63)` from any example or face
    /// crate reached the raw chip axis in one line — no [`PowerRequest`], no
    /// [`RfAuthority`](ndn_radio_hal::RfAuthority), no `NDN_RF_UNRESTRICTED`, in a different power regime from the
    /// fused base — and `tests/power_has_one_meaning.rs` passed on it, because that guard only
    /// scanned `fn set_tx_power*` bodies. Narrowing the visibility is what actually closes it: the
    /// only route to this writer is now `set_tx_power`'s [`PowerRequest::Raw`] arm, which cannot be
    /// constructed without the operator's written authority.
    ///
    /// `set_tx_power` on the calibrated scale caps output at the fused *regulatory* base (≈ index
    /// 27 on this ch6 adapter); this reaches the chip's full range for rated/max output or bench
    /// characterization.
    ///
    /// On-air (#38, conducted SDR, randomized+replicated, 40 dB SNR): the register→power transfer
    /// is monotone over the full 0–63 span, ~33 dB range, with the step **accelerating 0.5→1.0
    /// dB/idx** up the range (the documented Realtek TXAGC non-linearity). ⚠️ Above the regulatory
    /// base this can exceed the licensed EIRP — an explicit operator/bench opt-in
    /// ([`ndn_radio_hal::RfAuthority`]), not for unattended regulatory operation.
    pub(crate) fn write_txagc_flat(&self, idx: u8) -> Result<Vec<PowerWrite>, FaceError> {
        let idx = idx.min(TXAGC_MAX);
        let w = u32::from_le_bytes([idx, idx, idx, idx]);
        let mut writes = Vec::with_capacity(10);
        for (path, regs) in [
            (0u8, [0xc20u16, 0xc24, 0xc28, 0xc2c, 0xc30]),
            (1u8, [0xe20, 0xe24, 0xe28, 0xe2c, 0xe30]),
        ] {
            for reg in regs {
                self.write32(reg, w)?;
                writes.push(PowerWrite {
                    reg: reg as u32,
                    value: idx,
                    group: "flat (uncalibrated)",
                    path,
                });
            }
        }
        Ok(writes)
    }

    /// Force **both** antenna paths to transmit (rTxPath_Jaguar `0x80C` low word =
    /// 0x3333 = A+B for every rate group). A legacy-OFDM frame is otherwise single-
    /// stream on path A; enabling 2T drives both PAs (more radiated power + the second
    /// connector carries signal). Call after channel setup.
    pub fn set_tx_2t(&self, on: bool) -> Result<(), FaceError> {
        let cur = self.read32(0x80c)?;
        let low = if on { 0x3333 } else { 0x1111 };
        self.write32(0x80c, (cur & 0xffff_0000) | low)
    }

    /// Make the name-group hash a **hardware receive filter**: restrict the chip's
    /// RX to frames whose `addr1` exactly matches `group` (RCR **APM**), plus
    /// broadcast (**AB**), by clearing **AAP** (accept-all / promiscuous) and
    /// writing `group` into `REG_MACID`. Non-matching frames are dropped by the
    /// chip and never traverse USB — the data-centric equivalent of a NIC's
    /// multicast address filter, so the host wakes only for its name-group. `None`
    /// restores promiscuous monitor RX (`MONITOR_RCR`).
    ///
    /// (`APM` exact-matches `addr1` against `REG_MACID`; for a group/multicast
    /// name-group MAC verify on-air that it still matches — if not, use a
    /// locally-administered unicast name-group, or `AM`+`REG_MAR` hash filtering.)
    pub fn set_rx_group_filter(&self, group: Option<[u8; 6]>) -> Result<(), FaceError> {
        match group {
            None => self.write32(REG_RCR, MONITOR_RCR),
            Some(mac) => {
                self.write_reg(REG_MACID, &mac)?;
                self.write32(REG_RCR, MONITOR_RCR & !RCR_AAP)
            }
        }
    }

    /// Program the EDCCA energy-detect thresholds (#37, contention fix). `l2h` is
    /// the busy-**enter** threshold, `h2l` the busy-**exit** (hysteresis); both are
    /// signed register units, lower = more sensitive (defer to weaker energy). This
    /// only sets the thresholds — whether the **TX** engine defers to them is a
    /// separate knob, [`set_edcca_honor`](Self::set_edcca_honor). Writing
    /// `0x7f/0x7f` maxes them out (energy detect effectively off). Preserves the
    /// upper half of `0x8a4` (the register is shared with the path-B LSSI readback).
    pub fn set_edcca_threshold(&self, l2h: i8, h2l: i8) -> Result<(), FaceError> {
        let v = (l2h as u8 as u32) | ((h2l as u8 as u32) << 8);
        self.bb_set(REG_EDCCA_TH, 0x0000_FFFF, v)
    }

    /// Make the MAC TX engine **honor** (`true`) or **ignore** (`false`) the
    /// EDCCA energy-detect carrier sense. Honoring clears `REG_TX_PTCL_CTRL[15]`
    /// (the *ignore-EDCCA* bit) and sets the `REG_RD_CTRL[11]` gate, so the TX
    /// backoff now waits for the medium to fall below the threshold before it
    /// keys up — the CSMA deferral that the contention A/B tests. Ignoring
    /// restores the blast-regardless behaviour.
    pub fn set_edcca_honor(&self, honor: bool) -> Result<(), FaceError> {
        let mut ctrl = self.read16(REG_TX_PTCL_CTRL)?;
        ctrl = if honor {
            ctrl & !TX_PTCL_IGNORE_EDCCA
        } else {
            ctrl | TX_PTCL_IGNORE_EDCCA
        };
        self.write16(REG_TX_PTCL_CTRL, ctrl)?;
        let mut rd = self.read16(REG_RD_CTRL)?;
        rd = if honor {
            rd | RD_CTRL_EDCCA_EN
        } else {
            rd & !RD_CTRL_EDCCA_EN
        };
        self.write16(REG_RD_CTRL, rd)
    }

    /// Turn on energy-detect carrier sense at busy-enter threshold `l2h` (with the
    /// default 7-unit exit hysteresis) **and** make TX defer to it. The one-call
    /// "enable contention avoidance" path; A/B against [`Self::disable_edcca`].
    pub fn enable_edcca(&self, l2h: i8) -> Result<(), FaceError> {
        self.set_edcca_threshold(l2h, l2h.saturating_sub(EDCCA_HL_DIFF))?;
        self.set_edcca_honor(true)
    }

    /// Restore the promiscuous-injection default: max out the thresholds and let
    /// the TX engine ignore energy detect (the behaviour before #37).
    pub fn disable_edcca(&self) -> Result<(), FaceError> {
        self.set_edcca_threshold(EDCCA_OFF, EDCCA_OFF)?;
        self.set_edcca_honor(false)
    }

    /// **Full carrier-sense off** — the knob that makes the TX engine transmit regardless of a busy
    /// medium. Disabling EDCCA ([`disable_edcca`](Self::disable_edcca)) alone is NOT enough: on a
    /// channel occupied by other transmitters the 8812A still defers via the OFDM **packet/preamble
    /// CCA** (a separate baseband gate from energy-detect). Measured: with only EDCCA off, an 8812au
    /// against two saturating neighbours sent ~600 frames in 20 s (deferred); the a81a/8822E — which
    /// does not gate TX on this CCA — blasted ~14000. This forces the OFDM CCA-mode nibble
    /// `0x838[3:0]=0xc` ("CCA off", the same value IQK uses to quiesce the BB) so the MAC always reads
    /// the medium idle, saving the bring-up value to restore on `ignore=false`. `ignore=true` also
    /// applies the EDCCA-ignore path; `false` restores both. RX detection is affected while off, which
    /// is fine for a TX-only blast node. This is the doctrine's "monitor mode without CSMA".
    ///
    /// ★ **This no longer touches the contention window.** It used to also write
    /// `REG_EDCA_BE_PARAM = 0x005e_0002` — both window exponents zero, AIFS 2 µs (below SIFS) —
    /// which meant a caller asking to ignore *carrier sense* silently also abolished its
    /// *backoff*, on a path cognition arms automatically whenever the channel is busy. A zero
    /// window is the configuration that MEASURED 119 → 21 Mbit/s on an MT7612U and then took that
    /// radio out until it was physically replugged. The capability it was reaching for is now
    /// [`ndn_radio_hal::RadioKnobs::set_contention`] with
    /// [`ndn_radio_hal::ContentionPosture::Owned`], which is aggressive at a *legal* window; see
    /// [`crate::realtek_contention`] for the full reasoning.
    pub fn set_cca_ignore(&self, ignore: bool) -> Result<(), FaceError> {
        use std::sync::atomic::Ordering;
        // `REG_EDCA_BE_PARAM` (0x0508): TXOP[31:16] | ECWmax[15:12] | ECWmin[11:8] | AIFS[7:0]. The
        // 8812au never programs it, so it runs the firmware default backoff and gets out-competed on a
        // busy channel (measured: 330 f/s vs an a81a's ~13000, because the a81a wins EDCA contention).
        // Zeroing the contention window (ECWmin=ECWmax=0) + minimal AIFS makes it transmit with no
        // random backoff — the aggressive "blast" config (TXOP kept from the a81a's 0x005e).
        if ignore {
            // Save the bring-up CCA nibble once, then force CCA off.
            let cur = (self.bb_query(0x838, 0xf)? & 0xf) as u16;
            let _ =
                self.cca_saved
                    .compare_exchange(0xffff, cur, Ordering::SeqCst, Ordering::SeqCst);
            self.bb_set(0x838, 0xf, 0xc)?; // OFDM CCA off — the MAC now reads the medium idle
            self.disable_edcca() // energy-detect path too (thresholds max + ignore-EDCCA bit)
        } else {
            let saved = self.cca_saved.swap(0xffff, Ordering::SeqCst);
            if saved != 0xffff {
                self.bb_set(0x838, 0xf, saved as u32)?;
            }
            self.set_edcca_honor(true)
        }
    }

    /// Read back `(l2h, h2l, honored)` for verification — the thresholds from the
    /// low word of `0x8a4`, and whether TX honors EDCCA (the *ignore* bit clear).
    pub fn edcca_state(&self) -> Result<(i8, i8, bool), FaceError> {
        let v = self.bb_query(REG_EDCCA_TH, 0x0000_FFFF)?;
        let l2h = (v & 0xff) as u8 as i8;
        let h2l = ((v >> 8) & 0xff) as u8 as i8;
        let honored = self.read16(REG_TX_PTCL_CTRL)? & TX_PTCL_IGNORE_EDCCA == 0;
        Ok((l2h, h2l, honored))
    }

    /// Frame-free PHY sensing (#30): one snapshot of the baseband's energy
    /// (IGI) and its free-running OFDM detector counters (cca / false-alarm) —
    /// the channel's occupancy and interference read straight from the silicon,
    /// with **no frame decoded and nothing transmitted**. Sample twice around a
    /// window and [`PhySense::delta`] them for per-second rates. This is the raw
    /// input the radio-cognition plane senses the medium with.
    pub fn read_phy_sense(&self) -> Result<PhySense, FaceError> {
        Ok(PhySense {
            igi_a: (self.read32(REG_IGI_A)? & 0x7f) as u8,
            igi_b: (self.read32(REG_IGI_B)? & 0x7f) as u8,
            // ★ The counter is **20 bits** (`RXERR_COUNTER_MASK 0xFFFFF`), not 16. Masking 16 made
            // it wrap invisibly at 65535 — and since the caller deltas two samples, a wrap reads as
            // a large NEGATIVE activity swing, i.e. a quiet channel. Also select the type
            // explicitly: nothing ever wrote the selector, so the shipped sensor has been reading
            // whichever type reset happened to leave. Type 0 = `RXERR_TYPE_OFDM_PPDU`, which is
            // what makes the MEASURED "tracks frame rate 1:1" behaviour true — now by construction
            // rather than by luck.
            rx_activity: (self.read_rxerr(rxerr_type::OFDM_PPDU)? & 0xffff) as u16,
        })
    }

    /// Read one of the 13 selectable `REG_RXERR_RPT` counters, full 20-bit width.
    ///
    /// Semantics from the chip-common vendor header (`hal_com_reg.h:439-476`):
    /// `_RXERR_RPT_SEL(type) = type << 28`, `RXERR_RPT_RST = BIT(27)`,
    /// `RXERR_COUNTER_MASK = 0xFFFFF`.
    ///
    /// Free-running after selection; sample twice and difference, or reset with
    /// [`reset_rxerr`](Self::reset_rxerr).
    pub fn read_rxerr(&self, ty: u8) -> Result<u32, FaceError> {
        let sel = (u32::from(ty) & 0xf) << 28;
        // Preserve the low bits; only the selector nibble is ours to move.
        let cur = self.read32(REG_RXERR_RPT)?;
        if (cur >> 28) != u32::from(ty) & 0xf {
            self.write32(REG_RXERR_RPT, sel)?;
        }
        Ok(self.read32(REG_RXERR_RPT)? & 0x000f_ffff)
    }

    /// Zero the currently selected `REG_RXERR_RPT` counter (`RXERR_RPT_RST`, BIT 27).
    pub fn reset_rxerr(&self, ty: u8) -> Result<(), FaceError> {
        let sel = (u32::from(ty) & 0xf) << 28;
        self.write32(REG_RXERR_RPT, sel | (1 << 27))?;
        self.write32(REG_RXERR_RPT, sel)?;
        Ok(())
    }

    rung! {
        /// Release (resume) RX DMA by clearing `RW_RELEASE_EN` (`REG_RXPKT_NUM[18]`).
        /// After init the 8812AU leaves RX DMA paused (`RW_RELEASE_EN` set,
        /// `RXDMA_IDLE`), so no captured frame reaches the bulk-IN endpoint until it
        /// is released. **Call this as the final bring-up step** — IQ calibration
        /// re-pauses RX DMA, so releasing earlier has no lasting effect.
        fn start_rx_dma(&self) -> Result<(), FaceError> {
            // **USB RX aggregation** — the throughput lever that reaches a81a parity. Ported from the
            // aircrack-ng rtl8812au vendor `usb_AggSettingRxUpdate_8812A` + mainline rtl8xxxu. The MISSING
            // piece was the enable bit: RXDMA_AGG_EN = BIT(2) of REG_TRXDMA_CTRL (0x010C). Without it the
            // chip ships ~1 frame per bulk-IN transfer → ~200 f/s; page-threshold tuning alone only reached
            // ~500. In USB-agg mode REG_RXDMA_AGG_PG_TH (0x0280) is `size | (timeout<<8)` where size is in
            // **512-byte units** (NOT the 128-B DMA-mode page). Default 0x1020 = size 0x20 (16 KB) · timeout
            // 0x10 → pack toward the 32 KB pump buffer, cutting per-transfer overhead. The parse
            // (`parse_rx_transfer`) already length-walks each subframe 8-byte aligned, matching the vendor.
            //  - NDN_RXDMA_AGG=<hex> overrides the 0x0280 size|timeout word.
            //  - NDN_RX_AGG_OFF=1 leaves aggregation DISABLED (prompt 1-frame flush) if a caller ever needs it.
            let agg = std::env::var("NDN_RXDMA_AGG")
                .ok()
                .and_then(|v| u16::from_str_radix(v.trim().trim_start_matches("0x"), 16).ok())
                .unwrap_or(0x1020);
            // mainline parity: USB agg is NOT gated by REG_USB_SPECIAL_OPTION — clear its bit3.
            let spec = self.read8(0xFE55)?;
            self.write8(0xFE55, spec & !(1 << 3))?;
            self.write16(0x0280, agg)?; // REG_RXDMA_AGG_PG_TH = size(512B units) | timeout<<8
            self.write8(0xFE5B, (agg >> 8) as u8)?; // REG_USB_DMA_AGG_TO = the same timeout (belt-and-suspenders)
            if std::env::var_os("NDN_RX_AGG_OFF").is_none() {
                let ctrl = self.read8(REG_TRXDMA_CTRL)?; // 0x010C low byte
                self.write8(REG_TRXDMA_CTRL, ctrl | 0x04)?; // RXDMA_AGG_EN = BIT(2), set LAST
            }
            let v = self.read32(0x0284)?; // REG_RXPKT_NUM — clear RW_RELEASE_EN to resume RX DMA
            self.write32(0x0284, v & !(1 << 18))?;
            if std::env::var_os("NDN_RX_AGG_DBG").is_some() {
                let ctrl = self.read16(REG_TRXDMA_CTRL).unwrap_or(0);
                let pgth = self.read16(0x0280).unwrap_or(0);
                let spec = self.read8(0xFE55).unwrap_or(0);
                eprintln!(
                    "RXAGG_DBG: TRXDMA_CTRL(0x10C)=0x{ctrl:04x} (RXDMA_AGG_EN bit2={}) AGG_PG_TH(0x280)=0x{pgth:04x} USB_SPEC(0xFE55)=0x{spec:02x}",
                    (ctrl >> 2) & 1
                );
            }
            Ok(())
        }
    }

    /// **The one entry point** for this part. M8 deleted `bring_up_monitor`, which was a wrapper
    /// over this; `open_radio`'s RTL8812AU arm calls it directly, and a bench instrument that
    /// wants the concrete backend calls it here.
    ///
    /// The role selects the plan ([`BringUp::plan`]); `power` is the [`PowerRequest`] the final
    /// rung actuates, carrying its [`RateGroupPolicy`](ndn_radio_hal::bringup::RateGroupPolicy) —
    /// so which of the twelve Jaguar1 TXAGC group pairs get written is a *caller's* decision that
    /// lands in the report, not a variable read from inside the writer; the deviation, if any, is
    /// resolved against the plan before the first register write and lands in the digest; the proof
    /// requirement is validated against the role and this part's (empty) instrument set, also
    /// before the first register write.
    // The `Err` is large BECAUSE it carries the partial report — the whole point of §3.
    #[allow(clippy::result_large_err)]
    pub fn bring_up_planned(
        self: &Arc<Self>,
        channel: u8,
        role: Role,
        power: PowerRequest,
        deviation: Option<Deviation>,
        proof: ProofRequirement,
    ) -> Result<(BringUpReport, Guards), BringUpFailure> {
        let mut run = PlanRun::new(
            "RTL8812AU",
            self.device_address(),
            self.initial_state(channel, role, power),
        )
        .with_proof(proof);
        if let Some(d) = deviation {
            run = run.with_deviation(d);
        }
        // UFCS: the inherent `bring_up_planned` is not the trait method, but keep the same spelling
        // the 88xx arm uses so the two read alike.
        let (report, guards) = <Self as BringUp>::bring_up(self, &run)?;
        // The runner is part-agnostic and cannot know this part's PHY capability; the driver does.
        Ok((
            report.with_capability(RadioProfile::capability(self.as_ref())),
            guards,
        ))
    }

    /// The regime a plan starts from. Not a claim about the radio: it is what the caller asked
    /// for, and the rungs fill in what they establish.
    ///
    /// ⚠ `power` starts at [`AppliedPower::no_actuator`] carrying the caller's **request**, not at
    /// the fused base. No power has been written when a plan begins; `load_tx_power_info` is the
    /// rung that establishes the reference and `set_tx_power` is the rung that writes, and claiming
    /// either before it ran is the exact shape of defect this contract exists to remove. The
    /// request rides here so the last rung can read it back out of [`Ctx`] instead of reaching for
    /// the environment.
    fn initial_state(&self, channel: u8, role: Role, power: PowerRequest) -> RadioState {
        RadioState {
            channel,
            bw: ndn_radio_hal::Bandwidth::Bw20,
            format: "RawNdn/Raw80211 (see with_format)",
            role,
            power: AppliedPower::no_actuator(power),
            // ⚠ The bring-up does not program a rate: `inject` carries the DESC rate per frame
            // (`NDN_RADIO_TX_RATE`, default legacy 6M). Saying "unreported" beats inventing one.
            rate: ndn_radio_hal::RateState::unreported(),
            warm: None,
            contention: None,
            pump: PumpPolicy::CallerOwns,
            facts: Vec::new(),
        }
    }

    /// Where this dongle is on the USB tree, for the bring-up report.
    fn device_address(&self) -> ndn_radio_hal::DeviceAddress {
        match crate::DeviceSelect::from_env() {
            crate::DeviceSelect::Addr(a) => ndn_radio_hal::DeviceAddress::Usb(a),
            crate::DeviceSelect::Index(i) => ndn_radio_hal::DeviceAddress::Usb(format!("#{i}")),
            // ⚠ "first match on the bus" is not an address. `Unknown` says so rather than
            // printing a number that would not identify the device again.
            crate::DeviceSelect::First => ndn_radio_hal::DeviceAddress::Unknown,
        }
    }

    /// One raw bulk-IN read (diagnostic): returns the byte count without parsing
    /// (0 on timeout). Use to confirm RX DMA is delivering to the IN endpoint.
    pub fn rx_raw(&self, buf: &mut [u8]) -> Result<usize, FaceError> {
        match self.handle.read_bulk(self.bulk_in, buf, RX_TIMEOUT) {
            Ok(n) => Ok(n),
            Err(rusb::Error::Timeout) => Ok(0),
            Err(e) => Err(usb_err(e)),
        }
    }

    /// Inject on an explicit bulk-OUT endpoint address (diagnostic: the 8812AU
    /// has three OUT endpoints `0x02/0x03/0x04` mapped to TX priority queues).
    pub fn send_frame_ep(&self, ep: u8, frame: &[u8], hw_rate: u32) -> Result<(), FaceError> {
        let buf = Self::tx_buffer(frame, hw_rate);
        self.handle
            .write_bulk(ep, &buf, TX_TIMEOUT)
            .map_err(usb_err)?;
        Ok(())
    }

    /// De-aggregate a bulk-IN buffer (`n` valid bytes) into whole 802.11 frames,
    /// queued on `rx_pending`. The 8812A stacks `RXDESC ++ drvinfo ++ shift ++
    /// frame` per packet, each 8-byte aligned (`recvbuf2recvframe`); CRC-error
    /// and firmware-report (C2H) packets are dropped.
    fn parse_rx_transfer(&self, buf: &[u8]) -> Vec<CapturedFrame> {
        let n = buf.len();
        let mut off = 0;
        let mut q: Vec<CapturedFrame> = Vec::new();
        while off + RXDESC_SIZE <= n {
            let d = &buf[off..];
            let w0 = u32::from_le_bytes([d[0], d[1], d[2], d[3]]);
            let pkt_len = (w0 & 0x3FFF) as usize;
            let crc_err = (w0 >> 14) & 0x1 != 0;
            let drvinfo = ((w0 >> 16) & 0xF) as usize;
            let shift = ((w0 >> 24) & 0x3) as usize;
            let w2 = u32::from_le_bytes([d[8], d[9], d[10], d[11]]);
            let rpt_sel = (w2 >> 28) & 0x1 != 0; // firmware C2H report, not 802.11
            if pkt_len == 0 {
                break;
            }
            let start = RXDESC_SIZE + drvinfo * 8 + shift;
            let end = start + pkt_len;
            if end > n - off {
                break;
            }
            // Raw pump throughput: every RX unit pulled off USB, counted BEFORE the CRC/name filter
            // below, so it's comparable to a kernel monitor iface's rx_packets (which counts bad-FCS).
            crate::RX_RAW_FRAMES.fetch_add(1, std::sync::atomic::Ordering::Relaxed);
            // Pre-CRC debug: CRC-failed frames are dropped below, so they never reach the
            // post-parse `RX8812AU` log — but their *rate* tells whether an HT frame arrived
            // and failed demod (IQK/EVM issue) vs never arrived (tuning/sensitivity). Logged
            // separately so the 5 GHz HT-RX diagnosis can tell the two apart.
            if crc_err && !rpt_sel && std::env::var("NDN_RX_META_DBG").is_ok() {
                let rate = (u32::from_le_bytes([d[12], d[13], d[14], d[15]]) & 0x7f) as u8;
                let rssi = (drvinfo >= 1 && rate >= 0x04).then(|| {
                    // Per-path Jaguar1 decode, max across chains — the vendor's own
                    // `rx_pwdb_all` is the max of the paths. Byte 0 is path A, byte 1 path B;
                    // this used to read byte 1 alone, with TRSW unmasked.
                    let a = realtek_rx::jaguar1_path_rssi_dbm(d[RXDESC_SIZE]);
                    let b = realtek_rx::jaguar1_path_rssi_dbm(d[RXDESC_SIZE + 1]);
                    a.max(b)
                });
                let w3 = u32::from_le_bytes([d[12], d[13], d[14], d[15]]);
                let w4 = u32::from_le_bytes([d[16], d[17], d[18], d[19]]);
                eprintln!(
                    "RX8812AU_CRCERR len={pkt_len} rate=0x{rate:02x} rssi={rssi:?} w3={w3:08x} w4={w4:08x}"
                );
            }
            // ★ **C2H: the firmware is talking and nothing was listening.**
            //
            // `rpt_sel` (dword2 bit 28) marks a buffer as a firmware Cmd-to-Host report rather
            // than an 802.11 frame. Every consumer below gates on `!rpt_sel`, so these were
            // parsed, recognised and silently dropped — and this driver has no H2C path either,
            // which is why it has no TX feedback, no rate adaptation and no firmware-mediated
            // anything, while the fleet's "best supported" radios are exactly the
            // firmware-mediated ones.
            //
            // Triple-anchored: the firmware image builds this report's own 24-byte RX descriptor
            // and sets byte 0x0B |= 0x10 (dword2 bit 28); the vendor macro is
            // `GET_RX_DESC_C2H(rxdesc) = LE_BITS_TO_4BYTE(rxdesc + 0x08, 28, 1)`; and this driver
            // already computes the same bit. The payload starts at exactly `RXDESC_SIZE` — the
            // firmware writes `drvinfo_sz = 0` and `shift = 0`.
            //
            // Surfaced under an env gate rather than wired to a consumer, because what this
            // firmware actually emits is UNMEASURED — one run with this on answers it.
            if rpt_sel {
                // ★ CCX transmit report. Layout MEASURED and cross-checked against the blob's
                // builder at code 0x7A38 (`id = 0x03`, `plen = 8`, a 16x8 B ring at XDATA 0x8000):
                //   [0] C2H id (0x03)   [1] seq   [2..10] the 8-byte CCX record
                // and within the record:
                //   [0] b7 RETRY_OVER, b6 LIFE_TIME_OVER  ⇒ both clear = delivered
                //   [2] [5:0] retry count
                //   [5] final data rate (DESC_RATE) — MEASURED 0x04 = legacy 6M, matching what
                //       `TxIntent::ROBUST` actually transmits. That agreement is what makes the
                //       rest of the record trustworthy.
                let pl = &d[RXDESC_SIZE..(RXDESC_SIZE + pkt_len).min(n - off)];
                if pl.len() >= 10 && pl[0] == 0x03 {
                    let rec = &pl[2..10];
                    self.tx_rpt_seen
                        .fetch_add(1, std::sync::atomic::Ordering::Relaxed);
                    if rec[0] & 0xc0 == 0 {
                        self.tx_rpt_ok
                            .fetch_add(1, std::sync::atomic::Ordering::Relaxed);
                    }
                    self.tx_rpt_retries.fetch_add(
                        u32::from(rec[2] & 0x3f),
                        std::sync::atomic::Ordering::Relaxed,
                    );
                }
            }
            if rpt_sel && std::env::var_os("NDN_C2H_DBG").is_some() {
                let payload = &d[RXDESC_SIZE..(RXDESC_SIZE + pkt_len).min(n - off)];
                let id = payload.first().copied().unwrap_or(0);
                eprintln!(
                    "C2H8812AU id=0x{id:02x} len={pkt_len} drvinfo={drvinfo} shift={shift} \
                     payload={:02x?}",
                    &payload[..payload.len().min(16)]
                );
            }
            if !crc_err && !rpt_sel && pkt_len >= DOT11_HDR_LEN {
                let body = &d[start..end];
                // Same Realtek RX-descriptor field interpretation as the sibling backends
                // (realtek_rx): RX HwRate (dword3) -> MCS; path-A power pwdb_a (phystatus byte 1)
                // -> RSSI; RXTSFL (dword5) -> free-run per-frame hardware stamp.
                let rate = (u32::from_le_bytes([d[12], d[13], d[14], d[15]]) & 0x7f) as u8;
                let mcs_index = realtek_rx::mcs_from_desc_rate(rate);
                let rssi_dbm = (drvinfo >= 1 && rate >= 0x04).then(|| {
                    // Per-path Jaguar1 decode, max across chains — the vendor's own
                    // `rx_pwdb_all` is the max of the paths. Byte 0 is path A, byte 1 path B;
                    // this used to read byte 1 alone, with TRSW unmasked.
                    let a = realtek_rx::jaguar1_path_rssi_dbm(d[RXDESC_SIZE]);
                    let b = realtek_rx::jaguar1_path_rssi_dbm(d[RXDESC_SIZE + 1]);
                    a.max(b)
                });
                let rxtsfl = u32::from_le_bytes([d[20], d[21], d[22], d[23]]);
                let stamp = Some(realtek_rx::rx_stamp(rxtsfl, self.tsf_domain));
                // #75 mesh common-view leaf: a locally-administered BEACON (FC 0x80) carries a
                // neighbour's HW TSF at body[24:32] and its network-time belief at body[32:49]. Pair with
                // our RXTSFL; lets this 8812au (e.g. an Alfa) compose network time as a receive-only node.
                if body.len() >= 32 && body[0] == 0x80 {
                    let mut bssid = [0u8; 6];
                    bssid.copy_from_slice(&body[16..22]);
                    if bssid[0] & 0x02 != 0 {
                        let btsf = u64::from_le_bytes(body[24..32].try_into().unwrap());
                        let belief = body
                            .get(32..32 + ndn_time::REF_BELIEF_BYTES)
                            .and_then(ndn_time::RefBelief::from_beacon_bytes);
                        if let Ok(mut cv) = self.mesh_cv.lock() {
                            let count = cv.map(|c: ndn_radio_hal::MeshCv| c.count).unwrap_or(0) + 1;
                            *cv = Some(ndn_radio_hal::MeshCv {
                                peer_tsf: btsf,
                                our_rxtsfl: rxtsfl as u64,
                                count,
                                bssid,
                                belief,
                            });
                        }
                    }
                }
                // Raw capture debug (pre-parse): shows every 802.11 frame the chip delivered,
                // independent of whether parse_dot11 accepts it — isolates RX-capture vs RX-parse.
                // First 2 header bytes (frame-control) + first payload byte after the 24-B hdr.
                if std::env::var("NDN_RX_META_DBG").is_ok() {
                    let fc = if body.len() >= 2 {
                        (body[0], body[1])
                    } else {
                        (0, 0)
                    };
                    let w3 = u32::from_le_bytes([d[12], d[13], d[14], d[15]]);
                    let w4 = u32::from_le_bytes([d[16], d[17], d[18], d[19]]);
                    eprintln!(
                        "RX8812AU len={pkt_len} rate=0x{rate:02x} rssi={rssi_dbm:?} fc={:02x}{:02x} w3={w3:08x} w4={w4:08x}",
                        fc.0, fc.1
                    );
                }
                match self.format {
                    // named-time: extract the NDN payload from the 802.11 data frame.
                    FrameFormat::RawNdn { .. } => {
                        if let Some(cap) =
                            frame::parse_dot11(self.format, body, rssi_dbm, mcs_index, stamp)
                        {
                            q.push(cap);
                        }
                    }
                    // NAN / Raw80211: capture the raw 802.11 body, source/group MACs preserved.
                    _ => {
                        let mut ta = [0u8; 6];
                        ta.copy_from_slice(&body[10..16]);
                        let mut group = [0u8; 6];
                        group.copy_from_slice(&body[4..10]);
                        let addr3 = body.get(16..22).map(|s| {
                            let mut a = [0u8; 6];
                            a.copy_from_slice(s);
                            a
                        });
                        q.push(CapturedFrame {
                            payload: Bytes::copy_from_slice(body),
                            addr: Some(ta),
                            group: Some(group),
                            addr3,
                            // NAN/Raw80211 capture: the whole 802.11 frame is the payload; the
                            // wide-profile extra fields are not surfaced here (RawNdn routes through
                            // parse_dot11 above, which does surface them).
                            extra: None,
                            htc: None,
                            rssi_dbm,
                            mcs_index,
                            stamp,
                            phy: None,
                        });
                    }
                }
            }
            off += (start + pkt_len + 7) & !7; // next subframe, 8-byte aligned
        }
        q
    }

    /// Set the on-air frame format — `RawNdn { ethertype }` for named-time (wrap/extract the NDN
    /// payload), leaving the NAN default `Raw80211` otherwise. Consumes/returns `self` (builder).
    pub fn with_format(mut self, format: FrameFormat) -> Self {
        self.format = format;
        self
    }

    /// Start `depth` background reader threads — USB RX pipelining via the shared [`crate::rx_pump`].
    /// Afterwards `recv_frame` / [`poll_frame`](Self::poll_frame) drain the shared queue.
    ///
    /// **Call this for any sustained receive.** Without it there is a bulk-IN read in flight only
    /// *during* a `recv_frame` call, so everything arriving while the caller parses, reassembles, or
    /// does anything else has nothing draining the dongle's RX FIFO and is lost. A single frame with
    /// idle time around it survives that; a back-to-back burst does not — which makes the loss scale
    /// with fragments-per-object and look like a flaky link. Measured: NDN objects at 4/8 fragments
    /// went 0/12 -> 9/12 and 0/12 -> 5/12 on this alone, and single-frame delivery stopped
    /// scattering (0-12/12 -> 12/12).
    ///
    /// Prefer **`depth = 1`**: one thread keeps a read permanently in flight, which is the whole
    /// point. Several threads issue independent blocking reads against the *same* endpoint and race
    /// to push, so frames reach the queue out of order.
    pub fn spawn_rx_pump(self: &Arc<Self>, depth: usize) -> Vec<std::thread::JoinHandle<()>> {
        crate::rx_pump::spawn_rx_pump(self, depth)
    }

    /// Synchronously poll for one captured 802.11 frame: drain the de-aggregation
    /// queue, else do one bulk-IN read (returns `None` on timeout / no frame).
    pub fn poll_frame(&self) -> Result<Option<CapturedFrame>, FaceError> {
        if let Some(f) = self.rx_pump.try_pop() {
            return Ok(Some(f));
        }
        let mut buf = vec![0u8; 16384];
        match self.handle.read_bulk(self.bulk_in, &mut buf, RX_TIMEOUT) {
            Ok(n) => self.rx_pump.push(self.parse_rx_transfer(&buf[..n])),
            Err(rusb::Error::Timeout) => return Ok(None),
            Err(e) => return Err(usb_err(e)),
        }
        Ok(self.rx_pump.try_pop())
    }
}

// ═════════════════════════════════════════════════════════════════════════════
// §5-M7 — the RTL8812AU bring-up, as a plan
//
// ☠ **TRANSCRIBED, NOT REDESIGNED.** Appendix A.3 of `docs/bringup-contract.md` is explicit that
// migrating the one 8812au sequence measured to put frames on air into a table someone *believes*
// is equivalent is exactly the reasoning-about-radios that has a ~0 % hit rate. So: this is the
// body of the pre-M7 `bring_up_monitor`, rung for rung, in its order, with **not one register
// write moved, added or reordered**. The acceptance is an on-air witness run the operator
// performs, not a green test — see the "M7 acceptance" section of the contract.
//
// ☠ **Three things are NOT here, and their absence is the measurement.** `init_llt` (added → no
// change on air), a `REG_TXPAUSE` clear (added → A/B'd inert: 2789 / 4508 / 4588 / 4406 frames,
// run-to-run noise, no direction — and `REG_TXPAUSE` MEASURED `0x00` after three bring-ups), and
// the bulk-OUT endpoint theory (all three endpoints → 0 frames). The 2026-09-03 "does not
// transmit" symptom was **the power regime**, fixed in M1 by deleting `set_tx_power`'s
// fallthrough. The TXPAUSE clear comes back as an `Assert` — read the gate, do not write it.
// ═════════════════════════════════════════════════════════════════════════════

// ── the rungs ─────────────────────────────────────────────────────────────────

fn s_power_on(b: &Arc<Rtl8812auBackend>, _c: &mut Ctx<'_>) -> Result<StepOutcome, FaceError> {
    b.power_on()?;
    Ok(StepOutcome::Done)
}

fn s_download_firmware(
    b: &Arc<Rtl8812auBackend>,
    _c: &mut Ctx<'_>,
) -> Result<StepOutcome, FaceError> {
    let (version, subversion) = b.download_firmware()?;
    tracing::debug!(target: "named_radio", version, subversion, "8812au firmware downloaded");
    // The name is `Fact::Firmware`'s only honest filling: `fw_free_to_go` polls `WINTINI_RDY` and
    // returns `Err` on timeout, so reaching this line IS the ready signal.
    Ok(StepOutcome::Established(Fact::Firmware {
        name: "rtl8812au_fw_nic.bin",
        ready: true,
    }))
}

fn s_mac_config(b: &Arc<Rtl8812auBackend>, _c: &mut Ctx<'_>) -> Result<StepOutcome, FaceError> {
    b.mac_config()?;
    Ok(StepOutcome::Done)
}

fn s_mac_enable_dma(b: &Arc<Rtl8812auBackend>, _c: &mut Ctx<'_>) -> Result<StepOutcome, FaceError> {
    b.mac_enable_dma()?;
    Ok(StepOutcome::Done)
}

fn s_mac_init_queues(
    b: &Arc<Rtl8812auBackend>,
    _c: &mut Ctx<'_>,
) -> Result<StepOutcome, FaceError> {
    b.mac_init_queues()?;
    Ok(StepOutcome::Done)
}

fn s_bb_config(b: &Arc<Rtl8812auBackend>, _c: &mut Ctx<'_>) -> Result<StepOutcome, FaceError> {
    b.bb_config()?;
    Ok(StepOutcome::Done)
}

fn s_rf_config(b: &Arc<Rtl8812auBackend>, _c: &mut Ctx<'_>) -> Result<StepOutcome, FaceError> {
    b.rf_config()?;
    Ok(StepOutcome::Done)
}

fn s_set_channel(b: &Arc<Rtl8812auBackend>, c: &mut Ctx<'_>) -> Result<StepOutcome, FaceError> {
    b.set_channel(c.state_ref().channel)?;
    Ok(StepOutcome::Done)
}

/// ★★ **The rung that decided what `set_tx_power` meant.**
///
/// Verbatim from the M2 ladder, including the `channel.max(1)` in the representative base and the
/// `BestEffort` class: on `Err` the old code pushed a warning and a `Skipped` record and carried
/// on, which is what `StepClass::BestEffort` does — except that now the loss is *named* by the
/// `Degradation` instead of spelled out in an ad-hoc string, and the failure can no longer end in a
/// silent regime step because `set_tx_power` refuses the calibrated scale outright (M1).
fn s_load_tx_power_info(
    b: &Arc<Rtl8812auBackend>,
    c: &mut Ctx<'_>,
) -> Result<StepOutcome, FaceError> {
    let channel = c.state_ref().channel;
    b.load_tx_power_info()?;
    let base_index = b
        .tx_power_info
        .lock()
        .unwrap()
        .as_ref()
        .map(|i| i.index_base(0, 0x0c, 0, 0, channel.max(1)))
        .unwrap_or(0);
    Ok(StepOutcome::Established(Fact::PowerReference(
        PowerReference::FusedBase {
            base_index,
            channel,
        },
    )))
}

fn s_disable_edcca(b: &Arc<Rtl8812auBackend>, _c: &mut Ctx<'_>) -> Result<StepOutcome, FaceError> {
    b.disable_edcca()?;
    Ok(StepOutcome::Done)
}

/// The IQK retry loop, transcribed: one attempt, then up to five more until **both** RX paths and
/// both TX paths converge. `Branch` reports which way it came out, which the old ladder only put on
/// the tracing tree.
fn s_iq_calibrate(b: &Arc<Rtl8812auBackend>, _c: &mut Ctx<'_>) -> Result<StepOutcome, FaceError> {
    let mut iqk = b.iq_calibrate();
    for _ in 0..5 {
        match &iqk {
            Ok(r) if r.tx_a && r.rx_a && r.tx_b && r.rx_b => break,
            _ => iqk = b.iq_calibrate(),
        }
    }
    match iqk {
        Ok(r) => {
            tracing::info!(
                target: "named_radio",
                tx_a = r.tx_a, rx_a = r.rx_a, tx_b = r.tx_b, rx_b = r.rx_b,
                ch = b.cur_channel.load(std::sync::atomic::Ordering::Relaxed),
                "8812au IQK done"
            );
            Ok(StepOutcome::Branch(
                if r.tx_a && r.rx_a && r.tx_b && r.rx_b {
                    "all four paths converged"
                } else {
                    "partial convergence after six attempts — the last attempt's corrections are \
                     the ones left applied"
                },
            ))
        }
        // BestEffort: the runner warns with the `Degradation` and continues, which is what the old
        // `tracing::warn!` arm did — except it now reaches the report as well as the log.
        Err(e) => {
            tracing::warn!(target: "named_radio", error = ?e, "8812au IQK failed");
            Err(e)
        }
    }
}

fn s_lc_calibrate(b: &Arc<Rtl8812auBackend>, _c: &mut Ctx<'_>) -> Result<StepOutcome, FaceError> {
    b.lc_calibrate()?;
    Ok(StepOutcome::Done)
}

fn s_start_rx_dma(b: &Arc<Rtl8812auBackend>, _c: &mut Ctx<'_>) -> Result<StepOutcome, FaceError> {
    b.start_rx_dma()?;
    Ok(StepOutcome::Done)
}

/// The power rung. It reads the request **out of [`Ctx`]** — LAW 1 and LAW 3 in one line.
fn s_set_tx_power(b: &Arc<Rtl8812auBackend>, c: &mut Ctx<'_>) -> Result<StepOutcome, FaceError> {
    let requested = c.state_ref().power.requested.clone();
    let applied = b.set_tx_power(requested)?;
    let reference = applied.reference;
    c.state().power = applied;
    // LAW 6 — the resolved reference is what a later reader compares two runs by, so it is a Fact
    // and the runner hoists it into `RadioState::facts`. On the M7 acceptance run this is the field
    // that must read `FusedBase` on one node and `ChipRaw` on the other.
    Ok(StepOutcome::Established(Fact::PowerReference(reference)))
}

// ── the rungs, as reviewable constants ───────────────────────────────────────

const R_POWER_ON: Step<Rtl8812auBackend> = Step {
    id: StepId("power_on"),
    stage: Stage::PowerOn,
    class: StepClass::Required,
    why: "the 8812A power-on sequence. Until it completes the MAC is card-disabled: register writes \
          are accepted and dropped, so every later readback is fiction. **LAW 5**: it polls the \
          power-sequence entries and returns Err on timeout.",
    must_follow: &[],
    must_precede: &[],
    run: s_power_on,
};

const R_DOWNLOAD_FIRMWARE: Step<Rtl8812auBackend> = Step {
    id: StepId("download_firmware"),
    stage: Stage::Firmware,
    class: StepClass::Required,
    why: "the 8051 runs `rtl8812au_fw_nic.bin`; the H2C path and the TX report block speak to it. \
          **LAW 5**: `fw_free_to_go` polls `WINTINI_RDY` (1000 tries) and errors on timeout — and a \
          firmware written over an ALREADY-RUNNING 8051 does not fail cleanly, it drops the device \
          off the USB bus until it is physically replugged, which is why the reset guard inside it \
          is not a corner case.",
    must_follow: &[StepId("power_on")],
    must_precede: &[],
    run: s_download_firmware,
};

const R_MAC_CONFIG: Step<Rtl8812auBackend> = Step {
    id: StepId("mac_config"),
    stage: Stage::MacInit,
    class: StepClass::Required,
    why: "the phydm MAC register table (`rtl8812au_mac_reg.bin`), walked through `config_table`. \
          ★ It is also what re-pins EDCA on every bring-up, so contention state does NOT leak \
          across processes on this part — MEASURED byte-identical as-found state over five \
          processes with `Owned` applied in the middle. ☠ The four EDCA words live as DATA in the \
          `include_bytes!` blob (0x0500/0x0504/0x0508/0x050c), so a source grep will never see \
          them; decode the table before concluding a register is unwritten.",
    must_follow: &[StepId("download_firmware")],
    must_precede: &[],
    run: s_mac_config,
};

const R_MAC_ENABLE_DMA: Step<Rtl8812auBackend> = Step {
    id: StepId("mac_enable_dma"),
    stage: Stage::MacInit,
    class: StepClass::Required,
    why: "enables the MAC DMA / WMAC / scheduler / security blocks in `REG_CR`. ★ It ZEROES \
          `REG_CR` before setting `DMA_ENABLE`, so it MUST run before `mac_init_queues` — which \
          sets `MACTXEN|MACRXEN` last. The reverse order silently disables RX, which is why the \
          ordering is a `must_precede` and not a sentence in a doc comment.",
    must_follow: &[StepId("mac_config")],
    must_precede: &[StepId("mac_init_queues")],
    run: s_mac_enable_dma,
};

const R_MAC_INIT_QUEUES: Step<Rtl8812auBackend> = Step {
    id: StepId("mac_init_queues"),
    stage: Stage::MacInit,
    class: StepClass::Required,
    why: "RQPN / page boundaries / the 3-endpoint queue map / the promiscuous monitor RCR, and \
          `REG_CR |= MACTXEN|MACRXEN` LAST. This is the rung that makes the part both a monitor and \
          a transmitter; the `mac_tx_rx_enabled` assert reads its final write back.",
    must_follow: &[StepId("mac_enable_dma")],
    must_precede: &[],
    run: s_mac_init_queues,
};

const R_BB_CONFIG: Step<Rtl8812auBackend> = Step {
    id: StepId("bb_config"),
    stage: Stage::PhyInit,
    class: StepClass::Required,
    why: "powers on the baseband + RF analog (`FEN_BB_GLB_RSTN`/`FEN_BBRSTB`, then path-A and \
          path-B RF power), then loads the BB PHY table and the AGC table. It must precede \
          `rf_config`, which addresses the radios through the LSSI this powers up. It is also where \
          the three asserted BB gates get their post-bring-up values: `0x0808 = 0x0e028233`, \
          `0x0838 = 0x06c89b44`, `0x0a04 = 0x01ff000c` (decoded from `rtl8812au_phy_reg.bin`).",
    must_follow: &[StepId("mac_init_queues")],
    must_precede: &[StepId("rf_config")],
    run: s_bb_config,
};

const R_RF_CONFIG: Step<Rtl8812auBackend> = Step {
    id: StepId("rf_config"),
    stage: Stage::PhyInit,
    class: StepClass::Required,
    why: "`PHY_RFConfig8812` → RF6052: the radio-A and radio-B register tables, applied through the \
          BB LSSI. `set_channel` writes RF `0x18` on both paths and therefore needs the radios \
          configured first.",
    must_follow: &[StepId("bb_config")],
    must_precede: &[StepId("set_channel")],
    run: s_rf_config,
};

const R_SET_CHANNEL: Step<Rtl8812auBackend> = Step {
    id: StepId("set_channel"),
    stage: Stage::Tune,
    class: StepClass::Required,
    why: "tunes the synth. It must precede BOTH cal rungs and the power rung: IQK is a per-channel \
          calibration (`iq_calibrate`'s own doc says run it after `set_channel`), and TX power on \
          this part is per-channel — `index_base(path, rate, ch)` needs a tuned channel, and \
          `set_tx_power` REFUSES outright when `cur_channel == 0` rather than writing a base for a \
          channel that does not exist. ⚠ On 5 GHz this replays a captured per-channel kernel \
          program (`PROGS_5G`), which also re-writes `0x0808` and `0x0a04`; see `cck_rx_restored`.",
    must_follow: &[StepId("rf_config")],
    must_precede: &[
        StepId("load_tx_power_info"),
        StepId("iq_calibrate"),
        StepId("set_tx_power"),
    ],
    run: s_set_channel,
};

const R_LOAD_TX_POWER_INFO: Step<Rtl8812auBackend> = Step {
    id: StepId("load_tx_power_info"),
    stage: Stage::Calibrate,
    class: StepClass::BestEffort(Degradation::new(
        "the EFUSE TX-power calibration is NOT loaded, so the calibrated power scale is \
         UNREACHABLE on this handle and `set_tx_power` will refuse `Ceiling`/`Index` by name",
        "nothing that puts energy on the air — the `set_tx_power` rung below is Required and will \
         fail the bring-up. The handle is still good for register-level inspection of everything \
         this ladder wrote before it",
    )),
    why: "★★ **The rung that decided what `set_tx_power` meant, and the founding case of LAW 6.** \
          MEASURED 2026-09-03: with calibration loaded, `set_tx_power(0x3f)` wrote the fused base \
          (≈ index 27 on the ch6 adapter); without it the SAME call fell through to a flat 63 on \
          ten registers, in a different power regime from, and still returned `Ok(())`. The fallthrough is deleted \
          (M1) and this rung now returns `Fact::PowerReference(FusedBase{..})`, which the runner \
          hoists into `RadioState::facts` — so the regime is in the report whether or not anyone \
          asks. Best-effort because that is what it was; the difference is that its failure is no \
          longer invisible in either direction.",
    must_follow: &[StepId("set_channel")],
    must_precede: &[StepId("set_tx_power")],
    run: s_load_tx_power_info,
};

const R_DISABLE_EDCCA: Step<Rtl8812auBackend> = Step {
    id: StepId("disable_edcca"),
    stage: Stage::Posture,
    class: StepClass::BestEffort(Degradation::new(
        "energy-detect carrier sense is left ARMED, so the TX engine may hold frames on a busy \
         channel — injection defers to a medium reading it did not choose to respect",
        "receiving, and for transmitting on a quiet channel; cognition can still re-arm or clear it \
         later through `set_edcca_ignore`",
    )),
    why: "monitor injection must not defer to energy-detect carrier sense: the reset default can \
          leave the TX engine holding frames when the channel reads busy, so this maxes the \
          thresholds and sets the ignore-EDCCA bit. ⚠ It does NOT touch `0x838[3:0]` (the OFDM \
          packet CCA) — that is `set_cca_ignore`, which the factory applies AFTER the plan under \
          `NDN_CCA_OFF`, which is why the `cca_restored` assert is meaningful here.",
    must_follow: &[StepId("mac_init_queues")],
    must_precede: &[],
    run: s_disable_edcca,
};

const R_IQ_CALIBRATE: Step<Rtl8812auBackend> = Step {
    id: StepId("iq_calibrate"),
    stage: Stage::Calibrate,
    class: StepClass::BestEffort(Degradation::new(
        "TX/RX IQ imbalance is uncorrected: worse EVM and image rejection, and 5 GHz HT-MCS demod \
         in particular is not fully calibrated",
        "transmitting and receiving — IQK is not the on-air gate on this part; every legacy-rate \
         number from the run stands, but per-rate/EVM/RSSI figures are uncalibrated",
    )),
    why: "`_phy_iq_calibrate_8812a`, run **up to six times**. Each RX path converges only \
          marginally — one of the two per attempt, ALTERNATING (measured on 5 GHz) — and the vendor \
          driver retries the whole IQK up to 3× for exactly this; this retries until BOTH RX paths \
          lock, because the last run's corrections are the ones left applied. ⚠ The loop count is \
          transcribed, not chosen: 1 + 5 is what the measured ladder ran. ☠ This is also the rung \
          the five §1.5 asserts exist for: `iqk_configure_mac` drops TXPAUSE, the RX antenna, CCA \
          and the CCK RX path, and restores them only in the tail block that `iqk_tx()?` skips on \
          error.",
    must_follow: &[StepId("set_channel")],
    must_precede: &[StepId("set_tx_power")],
    run: s_iq_calibrate,
};

const R_LC_CALIBRATE: Step<Rtl8812auBackend> = Step {
    id: StepId("lc_calibrate"),
    stage: Stage::Calibrate,
    class: StepClass::BestEffort(Degradation::new(
        "the RF VCO is left at its last tuned state — the LC tank is not re-centred for this \
         channel, so the synth may sit off its best operating point",
        "transmitting and receiving on the tuned channel; it is a trim, not a gate",
    )),
    why: "the LC tank calibration, run after IQK exactly as the ladder ran it. It pauses the TX \
          queues around the sweep (`REG_TXPAUSE = 0xff`) and restores the saved value at its tail — \
          the `0xff` half of what `txpause_released` reads back. ⚠ `8307161` changed this function \
          on 2026-08-31 and sixteen private ladders did not notice; that is the failure a named \
          rung with an id makes loud.",
    must_follow: &[StepId("iq_calibrate")],
    must_precede: &[StepId("set_tx_power")],
    run: s_lc_calibrate,
};

const R_START_RX_DMA: Step<Rtl8812auBackend> = Step {
    id: StepId("start_rx_dma"),
    stage: Stage::RxEnable,
    class: StepClass::Required,
    why: "releases RX DMA (`REG_RXPKT_NUM[18]` `RW_RELEASE_EN` cleared) and arms USB RX \
          aggregation. It must be LAST among the RX rungs because IQ calibration RE-PAUSES RX DMA — \
          releasing earlier has no lasting effect. Required, not best-effort: without it no \
          captured frame ever reaches the bulk-IN endpoint and the radio is silently deaf. \
          MEASURED: the aggregation enable bit is the difference between ~200 f/s and a81a parity. \
          ⚠ **KNOWN LAW 1 EXCEPTION, one call deep and written down rather than hidden**: \
          `start_rx_dma` itself still reads `NDN_RXDMA_AGG`, `NDN_RX_AGG_OFF` and `NDN_RX_AGG_DBG`. \
          Hoisting them to the request needs §1.1's `BringUpRequest`/`PartOpts`, which is not built \
          yet; M7 does not invent a third home for them, and `plan_shape_8812au.rs` pins the three \
          names so the set cannot grow silently.",
    must_follow: &[StepId("iq_calibrate")],
    must_precede: &[],
    run: s_start_rx_dma,
};

const R_SET_TX_POWER: Step<Rtl8812auBackend> = Step {
    id: StepId("set_tx_power"),
    stage: Stage::Power,
    class: StepClass::Required,
    why: "★ **TXAGC is set LAST, after everything that touches the TX gain chain.** IQK is a TX/RX \
          loopback calibration that drives the gain registers and runs up to six times above, so \
          setting power before it leaves the final TXAGC unguaranteed. ⚠ Not individually validated \
          on air — the ordering moved on reasoning, not measurement; what WAS measured \
          (2026-09-03) is that the ordering is NOT what made this bring-up appear silent \
          (`load_tx_power_info` is). Required because on the calibrated scale an unresolved \
          calibration is now a refusal, and a bring-up that returns a handle whose power regime is \
          unknown is the defect this contract removes. The request — including its \
          `RateGroupPolicy` — is read out of `Ctx`, never out of the environment.",
    must_follow: &[
        StepId("load_tx_power_info"),
        StepId("iq_calibrate"),
        StepId("lc_calibrate"),
    ],
    must_precede: &[],
    run: s_set_tx_power,
};

// ── the plan ─────────────────────────────────────────────────────────────────

/// The one sequence, in the order `bring_up_monitor` ran it.
const M7_STEPS: &[Step<Rtl8812auBackend>] = &[
    R_POWER_ON,
    R_DOWNLOAD_FIRMWARE,
    R_MAC_CONFIG,
    R_MAC_ENABLE_DMA,
    R_MAC_INIT_QUEUES,
    R_BB_CONFIG,
    R_RF_CONFIG,
    R_SET_CHANNEL,
    R_LOAD_TX_POWER_INFO,
    R_DISABLE_EDCCA,
    R_IQ_CALIBRATE,
    R_LC_CALIBRATE,
    R_START_RX_DMA,
    R_SET_TX_POWER,
];

const M7_PLAN: Plan<Rtl8812auBackend> = Plan {
    id: PlanId {
        part: "rtl8812au",
        name: "monitor",
        // ★ ver 2: ver 1 was the M2 hand-filled report. The digest changes, and that is correct —
        // an M2 number and an M7 number are not the same run description even though the register
        // sequence is byte-identical.
        ver: 2,
    },
    role: Role::TransmitAndReceive,
    steps: M7_STEPS,
    excluded: &[
        (
            Stage::Attach,
            "the device is selected and claimed BEFORE the plan runs — `open_select` takes a \
             `DeviceSelect` (first / Nth / `\"<bus>-<port>\"`) and `claim` does the USB port reset. \
             ⚠ That reset is load-bearing on this part and is deliberately NOT a rung: on a warm \
             chip without it `power_on`'s `CARDEMU_TO_ACT` poll cannot complete, so bring-up fails \
             LOUDLY at its first `?` rather than silently inheriting the previous process's \
             posture. Moving it into the plan would change when it happens.",
        ),
        (
            Stage::TxEnable,
            "★ **This part has no TX-enable rung, and that is MEASURED, not an omission.** Unlike \
             the 8733BU — where radiating needs a whole second block (`enable_tx`'s IQK → TXGAPK → \
             DPK plus a power-tracking thread) — the 8812AU transmits from this ladder as written: \
             `mac_init_queues` sets `MACTXEN|MACRXEN` and nothing further is required. The \
             reference number is 3074 frames at the AR9271 witness from a calibrated bring-up plus \
             a raw write. Adding a TX-enable rung here would be an unmeasured change to the one \
             radiating sequence this fleet has, which Appendix A.3 forbids by name.",
        ),
        (
            Stage::Verify,
            "no Verify-stage rung. The readbacks this ladder owes are the five part-wide `Assert`s \
             (`ASSERTS_8812AU`), which the runner takes after the last rung; and §4's transmit \
             question is answered `Unprovable` with the measurement quoted, because no Jaguar1 \
             MAC->BB counter is ported for this part. A rung that claimed to verify transmission \
             from the host alone would be spelling (B), which only a witness can answer.",
        ),
    ],
};

// ★ **A malformed plan is a compile error, not a runtime one.** In particular, moving
// `R_SET_TX_POWER` above the cal chain, or `R_MAC_INIT_QUEUES` above `R_MAC_ENABLE_DMA`, stops the
// crate building.
const _: () = M7_PLAN.check_or_panic();

/// **The RTL8812AU monitor plan** — the transcription of `bring_up_monitor` (contract §5-M7).
pub static PLAN_8812AU_MONITOR: Plan<Rtl8812auBackend> = M7_PLAN;

/// §1.5 — **read back every gate you write.** The five the contract names for this part.
///
/// ☠ **All five are the `iqk_configure_mac` quiesce list**, not just TXPAUSE. That function drops
/// five things — TX pause, `0x550`'s two bits, the RX antenna, the OFDM CCA mode, the CCK RX path —
/// and they are restored only in `iq_calibrate`'s tail block, which the `iqk_tx()?` error path
/// skips. The retry loop makes it worse: an attempt that fails after `iqk_configure_mac` leaves the
/// quiesced values in place, and the next attempt's backup then *saves the quiesced values* and
/// faithfully restores them. Nothing in the ladder notices; the radio simply goes quiet or deaf.
///
/// ⚠ **`Warn` for all five, per §5/M-hazards.** Promotion to `Fatal` is per part and needs a
/// measurement. `txpause_released` in particular has MEASURED `0x00 / 0x00 / 0x00` over three
/// bring-ups — structurally real, not firing today — and a readback nobody has watched fail is not
/// allowed to refuse the radio the forwarder runs on.
///
/// ⚠ **These are asserts, not steps.** The hand-rolled `write8(0x522, 0x00)` that lived in
/// `inject8812au`/`pwr_sweep8812au` was A/B'd INERT (2789 / 4508 / 4588 / 4406 frames — run-to-run
/// noise, no direction) and reverted. Read the gate; do not write it.
const ASSERTS_8812AU: &[Assert<Rtl8812auBackend>] = &[
    Assert {
        id: StepId("txpause_released"),
        reg: 0x0522,
        read: |b: &Rtl8812auBackend| b.read8(0x0522).map(u32::from),
        want: 0x00,
        mask: 0xff,
        why: "REG_TXPAUSE. `iqk_configure_mac` writes 0x3f and `lc_calibrate` writes 0xff; both \
              restore at a tail the error path skips. A held TXPAUSE means frames queue, `inject` \
              returns Ok, and NOTHING reaches the air with the host never told. 0x3f is the \
              aborted-IQK residue, 0xff the aborted-LCK one (`examples/rf_ab.rs`). The identical \
              one-line assert catches a LIVE defect on the sibling 88xx, where `txgapk_tx_pause` \
              writes 0xff and the resume sits past a swallowed error.",
        severity: Severity::Warn,
    },
    Assert {
        id: StepId("mac_tx_rx_enabled"),
        reg: 0x0100,
        read: |b: &Rtl8812auBackend| b.read_cr().map(u32::from),
        // MACTXEN (1<<6) | MACRXEN (1<<7).
        want: 0xc0,
        mask: 0xc0,
        why: "REG_CR's MACTXEN|MACRXEN, the last write `mac_init_queues` makes. `read_cr()` already \
              existed for exactly this readback and had ONE caller — an example. A monitor whose \
              MAC RX enable never took is deaf and reports nothing; a transmitter whose MAC TX \
              enable never took injects into a MAC that will not key.",
        severity: Severity::Warn,
    },
    Assert {
        id: StepId("rx_antenna_restored"),
        reg: 0x0808,
        read: |b: &Rtl8812auBackend| b.read8(0x0808).map(u32::from),
        // The RX-path byte `bb_config`'s `rtl8812au_phy_reg.bin` programs (0x0e028233) and every
        // one of the 25 captured 5 GHz programs writes (0x3e028233) — 0x33 in both, so this byte
        // is band-invariant.
        want: 0x33,
        mask: 0xff,
        why: "`iqk_configure_mac` writes `0x808 = 0x00` (RX antenna OFF) and the restore is a \
              dword write of the pre-IQK backup, in the tail block `iqk_tx()?` skips. Left at 0x00 \
              the part receives nothing at all — and the ladder would still return a handle. \
              0x33 is what the BB table and all 25 captured per-channel programs put in that byte.",
        severity: Severity::Warn,
    },
    Assert {
        id: StepId("cca_restored"),
        reg: 0x0838,
        read: |b: &Rtl8812auBackend| b.bb_read(0x0838),
        // `rtl8812au_phy_reg.bin`: 0x0838 = 0x06c89b44. No captured 5 GHz program writes 0x0838, so
        // the nibble is 0x4 in both bands after bring-up.
        want: 0x4,
        mask: 0xf,
        why: "the OFDM CCA-mode nibble `0x838[3:0]`. `iqk_configure_mac` forces it to 0xc (\"CCA \
              off\") and restores it from the pre-IQK dword backup. Left at 0xc the MAC always \
              reads the medium idle — which is a legitimate posture (`set_cca_ignore(true)` asks \
              for exactly it) but NOT one anybody asked for here, and it silently changes what \
              every later contention or occupancy measurement means. ⚠ The factory applies \
              `set_cca_ignore` under `NDN_CCA_OFF` *after* the plan returns, so this readback is \
              unambiguous at the point the runner takes it.",
        severity: Severity::Warn,
    },
    Assert {
        id: StepId("cck_rx_restored"),
        reg: 0x0a07,
        read: |b: &Rtl8812auBackend| b.read8(0x0a07).map(u32::from),
        // `rtl8812au_phy_reg.bin`: 0x0a04 = 0x01ff000c, i.e. byte 0x0a07 = 0x01; the 2.4 GHz branch
        // of `set_channel` then writes 0x1 into `0x0A04[27:24]`, leaving 0x01.
        want: 0x01,
        mask: 0xff,
        why: "the CCK RX path byte. `iqk_configure_mac` writes `0xa07 = 0x0f` (CCK RX off) and \
              restores it from the `0xa04` dword backup in the skipped tail block; left at 0x0f the \
              part cannot demodulate CCK at all. \
              ☠ **This assert has NO discriminating power on 5 GHz and WILL warn there — written \
              down rather than papered over.** `PROGS_5G` replays a captured kernel program that \
              sets `0x0a04 = 0x0fff000c`, i.e. `0xa07 = 0x0f` — byte-identical to the quiesce \
              value, so on a 5 GHz channel a restored register and a leaked one cannot be told \
              apart by any static `want`. The value chosen is the 2.4 GHz one, because ch6 is where \
              this part's power regime was measured and where the M7 acceptance runs. Closing the \
              5 GHz half needs a per-band `want`, which `Assert` does not carry — a type change \
              plus a measurement, not a guess.",
        severity: Severity::Warn,
    },
];

/// §4 — what this part can prove about its own transmitter: **nothing, and it says why.**
///
/// Quoted from the pre-M7 report, which is the fleet's own discipline and is kept verbatim. The
/// consequence is the whole shape of §5-M7: the acceptance for this migration is an on-air witness
/// run, because the host cannot answer even question (A) here.
const TX_UNPROVABLE_8812AU: &str = "no Jaguar1 MAC->BB counter ported; CCX/SPE_RPT is lossy (1-4 records per ~2300 armed) and \
     needs a running RX pump. Prove with a witness.";

impl BringUp for Rtl8812auBackend {
    fn plan(role: Role) -> Option<&'static Plan<Self>> {
        match role {
            Role::TransmitAndReceive => Some(&PLAN_8812AU_MONITOR),
            // ★ Named refusals, not silent downgrades. **One plan**, because one ladder is all this
            // part has ever run: `bring_up_monitor` is the only bring-up in the tree for it, and
            // the sixteen hand-rolled example ladders are copies of it or deviations from it, not
            // alternatives to it. An RX-only variant would mean deleting the cal chain and the
            // power rung, and a TX-only variant would mean deleting the monitor RCR out of the
            // middle of `mac_init_queues` — neither has been run on this silicon, and inventing one
            // here is exactly the unmeasured ladder this contract exists to remove. A caller that
            // only wants to listen asks for `TransmitAndReceive` and does not inject.
            Role::ReceiveOnly | Role::TransmitOnly => None,
        }
    }

    fn asserts() -> &'static [Assert<Self>] {
        ASSERTS_8812AU
    }

    /// Empty, and that is the measured answer — see `TX_UNPROVABLE_8812AU`.
    fn tx_instruments() -> &'static [TxInstrument] {
        &[]
    }

    fn tx_unprovable_reason() -> Option<&'static str> {
        Some(TX_UNPROVABLE_8812AU)
    }
}

/// The RX pump's per-transfer parse for the 8812AU — shares the async-URB pipeline with the other
/// Realtek USB backends via one [`crate::rx_pump`] implementation.
impl crate::rx_pump::Pumpable for Rtl8812auBackend {
    fn pump_handle(&self) -> Arc<DeviceHandle<Context>> {
        self.handle.clone()
    }
    fn pump_bulk_in(&self) -> u8 {
        self.bulk_in
    }
    fn pump_state(&self) -> &crate::rx_pump::RxPumpState {
        &self.rx_pump
    }
    fn parse_transfer(&self, buf: &[u8]) -> Vec<CapturedFrame> {
        self.parse_rx_transfer(buf)
    }
}

/// Async monitor radio: NAN injects management frames (`Raw80211` — the payload
/// is a complete 802.11 frame) and captures whatever is on the channel. Blocking
/// USB I/O runs on the blocking pool so the async reactor is never stalled.
#[async_trait]
impl FrameIo for Rtl8812auBackend {
    /// This radio's own capability, so a face built from the bare `dyn FrameIo` does not have to
    /// invent one. Delegates to this type's [`RadioProfile`] — the single source of truth.
    fn radio_capability(&self) -> Option<ndn_radio_hal::RadioCapability> {
        Some(<Self as ndn_radio_hal::RadioProfile>::capability(self))
    }
    /// ⚠ **This backend serves common-view observations while its own
    /// [`FaceTimeProfile::can_common_view`](ndn_radio_hal::FaceTimeProfile::can_common_view) is
    /// `false`.** Both statements are true, they answer different questions, and the gap between
    /// them is a real one — recorded here rather than papered over.
    ///
    /// * This method: "did I pair a neighbour's beacon TSF with my own RXTSFL?" The parse is real
    ///   and the latch is real (RXTSFL, per frame, in hardware).
    /// * The profile: "may anyone difference my counter against someone else's?" That needs the
    ///   latch **and** a reference that holds a rate, and nothing in this tree establishes what this
    ///   part's TSF runs on — see the ⚠ block on `impl RadioTime for Rtl8812auBackend` below.
    ///
    /// **Nothing gates the first on the second today.** The live consumer,
    /// `ndn-phy-wifi`'s `medium.rs`, polls this and feeds it straight into
    /// `FaceScheduler::ingest_common_view` / `ingest_mesh_beacon`; `FaceTimeProfile` is not
    /// consulted anywhere in that crate. So on an 8812au face the scheduler's `cv_hw` clock is
    /// disciplined from observations this radio's own profile says it cannot produce.
    ///
    /// The gate belongs at that consumer, not here — a backend reporting what it observed is
    /// correct, and suppressing the observation would also destroy the only signal that could ever
    /// characterise this counter. Two ways to close it, in preference order:
    ///
    /// 1. **Settle the reference.** One run of `examples/rxtick.rs` on this part (it prints the
    ///    declared reference) plus a rate regression against the host clock, or a two-node pairing
    ///    against a part whose reference IS established (the 8822E or the 8733BU). If it lands in
    ///    tens of ppm this becomes `ClockReference::crystal()` with a citation and the contradiction
    ///    evaporates.
    /// 2. **Gate the consumer.** `medium.rs` checks
    ///    `FaceTimeProfile::derive(radio_time, tx_discipline).can_common_view` for the same radio
    ///    before ingesting. ⚠ Doing (2) before (1) silently switches the #75 mesh-CV leaf path off
    ///    on this radio — which is the correct-but-costly outcome, and should be a deliberate choice
    ///    rather than a side effect.
    fn mesh_common_view(&self) -> Option<ndn_radio_hal::MeshCv> {
        self.mesh_cv.lock().ok().and_then(|g| *g)
    }

    /// Rate as bearer state: store the exact MCS every subsequent `inject` uses (until
    /// changed or a `MostRobust` frame overrides it to legacy). Before this the 8812au
    /// discarded the control plane's decision and every frame went out at legacy 6 Mbps.
    fn set_rate(&self, mcs: crate::McsDescriptor) -> Result<(), FaceError> {
        *self.cur_mcs.lock().unwrap() = Some(mcs);
        Ok(())
    }

    async fn inject(&self, frame: InjectFrame) -> Result<(), FaceError> {
        // Build the on-air 802.11 frame under the configured format (Raw80211 = the payload IS the
        // frame, verbatim, the NAN path; RawNdn = wrap the NDN payload in an 802.11 data frame).
        let handle = self.handle.clone();
        let ep = self.bulk_out;
        let dot11 = frame::build_dot11(self.format, &frame)?;
        // Rate is bearer state: once the cognition control plane has decided an MCS via
        // [`FrameIo::set_rate`], transmit at that DESC code ([`desc_rate_for`]); a `MostRobust`
        // frame still drops to legacy 6 Mbps inside the resolver so control/report frames stay
        // universally decodable. With no stored rate we keep the legacy 6 Mbps default (real NAN +
        // a legacy-only 8733b RX alike). `NDN_RADIO_TX_RATE=<dec>` is the manual override / ultimate
        // fallback (e.g. 12 = HT-MCS0, 44 = VHT-1SS-MCS0 — to test a peer's HT/VHT RX).
        let base = match *self.cur_mcs.lock().unwrap() {
            Some(mcs) => desc_rate_for(&mcs, &frame.tx),
            None => DESC_RATE_6M,
        };
        let rate = std::env::var("NDN_RADIO_TX_RATE")
            .ok()
            .and_then(|s| s.parse::<u32>().ok())
            .unwrap_or(base);
        let buf = Self::tx_buffer(&dot11, rate);
        tokio::task::spawn_blocking(move || {
            handle
                .write_bulk(ep, &buf, TX_TIMEOUT)
                .map_err(usb_err)
                .and_then(|n| {
                    // ★ A short write is a PARTIAL descriptor in the bulk-OUT stream — every later
                    // frame in that pipe is then misaligned garbage, and the old `.map(|_| ())`
                    // discarded the transferred count so it was invisible. The a81a backend already
                    // asserts this; this one silently reported success. An `inject` that returns Ok
                    // on a truncated transfer makes every throughput number downstream a fiction.
                    if n == buf.len() {
                        Ok(())
                    } else {
                        Err(init_err(format!(
                            "rtl8812au inject: short bulk write {n}/{} bytes",
                            buf.len()
                        )))
                    }
                })
        })
        .await
        .map_err(|e| init_err(format!("inject join: {e}")))?
    }

    async fn recv_frame(&self) -> Result<CapturedFrame, FaceError> {
        // Pumped mode: background reader threads fill the shared queue; just drain it.
        if self.rx_pump.is_pumped() {
            return Ok(self.rx_pump.recv().await);
        }
        loop {
            if let Some(f) = self.rx_pump.try_pop() {
                return Ok(f);
            }
            let handle = self.handle.clone();
            let ep = self.bulk_in;
            let (buf, n) = tokio::task::spawn_blocking(move || {
                let mut buf = vec![0u8; 16384];
                match handle.read_bulk(ep, &mut buf, RX_TIMEOUT) {
                    Ok(n) => Ok((buf, n)),
                    Err(rusb::Error::Timeout) => Ok((buf, 0)),
                    Err(e) => Err(usb_err(e)),
                }
            })
            .await
            .map_err(|e| init_err(format!("recv join: {e}")))??;
            if n > 0 {
                self.rx_pump.push(self.parse_rx_transfer(&buf[..n]));
            }
        }
    }
}

// `inject_at` is the derived HAL default (`set_rate` + `inject`), so the driver
// holds the transmit rate as state (`cur_mcs` + [`desc_rate_for`]) and no longer
// implements a per-frame exact-rate path. With no rate decided the injection
// default stays legacy 6 Mbps (universally decodable — real NAN + legacy-only RX).

/// Exposes the always-on free-run per-frame RX-stamp clock (RXTSFL) now latched onto every
/// received management frame. No read-now port TSF here, so `read_clock` stays the default `None`.
impl RadioTime for Rtl8812auBackend {
    /// ⚠ Reference: **UNKNOWN**, and that is a statement about this PORT, not about the silicon.
    ///
    /// Its two siblings here both witness a crystal from their own bring-up — the RTL8822E reads the
    /// factory crystal cap out of efuse `0x110` into `0x1040`, the RTL8733BU reads `EEPROM_XTAL_B9`
    /// and has a MEASURED trim curve. This port does neither: `grep -i 'xtal\|crystal'` over
    /// `rtl8812au.rs` finds nothing, it never touches a crystal-cap register, and nobody has
    /// regressed its TSF against a host or a peer clock. So nothing in this tree witnesses what this
    /// part's counter runs on.
    ///
    /// "It is an 802.11ac MAC, of course it has a crystal" is exactly the plausible inference this
    /// contract refuses: plausible is not measured, and the same reasoning declared a 1 us tick on
    /// the 8733b that MEASURED 4 us. The cost of the honest answer is that
    /// `FaceTimeProfile::can_common_view` is now false for this part; the cure is one run of
    /// `examples/rxtick.rs` (or a two-node pairing) and a one-line change here.
    ///
    /// ⚠ **This part therefore says two things about itself, and both are true.** It declares
    /// `can_common_view = false` here while [`FrameIo::mesh_common_view`] keeps emitting real
    /// observations — which are consumed, ungated, by `ndn-phy-wifi`'s `medium.rs`. The reason they
    /// are not the same claim, and what would close the gap, is written out on that method.
    fn time_sources(&self) -> Vec<RadioTimeSource> {
        vec![
            RadioTimeSource::free_run_rx_stamp(self.tsf_domain, 1_000)
                .with_reference(ndn_radio_hal::ClockReference::unknown()),
        ]
    }
}

impl RadioProfile for Rtl8812auBackend {
    fn capability(&self) -> RadioCapability {
        // RTL8812AU: 2-stream dual-band 11ac, used here for NAN management frames on the social
        // channels (2.4 ch6 + 5 GHz ch44/149). Reports the hardware profile (NAN-only is policy).
        RadioCapability {
            bands: vec![Band::Band2_4GHz, Band::Band5GHz],
            // ★ `max_bw: 0` (20 MHz), not the preset's 2 (80 MHz). `RadioKnobs::set_channel` on
            // this backend returns `Err` for every width except `Bw20` — the per-channel RF tables
            // for 40/80 are not ported — so declaring 80 was not optimism, it was a request the
            // policy would make every tick and the driver would refuse every tick. Before
            // `apply_knobs` degraded per knob, that refusal ABORTED the whole tick before reaching
            // the power branch, so this radio's power knob was unreachable for a reason that had
            // nothing to do with power. Declare what the driver can actually do.
            rate: ndn_radio_hal::RateCapability::Wifi {
                max_mcs: 9,
                max_nss: 2,
                max_bw: 0,
            },
            ..RadioCapability::wifi_monitor_5ghz(vec![6, 44, 149])
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::McsDescriptor;
    use ndn_radio_hal::TxIntent;

    /// ☠ **The shipped calibrated path writes FIVE groups, and that is a MEASURED result.**
    ///
    /// Covering the other seven (HT MCS8-15, all VHT) is a real gap — cognition sets `vht = true`
    /// here, so a commanded back-off moves CCK and legacy OFDM and leaves the data rate alone. But
    /// writing them with `index_base(..) + offset` **stops this radio transmitting**, measured
    /// against a witness receiver:
    ///
    /// | groups | offered | on air (11 s) |
    /// |---|---|---|
    /// | 12 | 711 f/s | **7 frames = 1 f/s** |
    /// | 5 | 2805 f/s | **2704 frames = 246 f/s** |
    ///
    /// ★ Note the offered rates: 711 vs 2805 looks like a difference of degree. On air it is 250x.
    /// The transmitter cannot see this, which is why two rounds of artefact reasoning got it wrong
    /// in both directions before anyone put a second radio on the channel.
    #[test]
    fn txagc_writes_five_groups_by_default() {
        let src = include_str!("rtl8812au.rs");
        assert!(
            src.contains("&groups[..5]"),
            "the default must be the five-group write — twelve MEASURED 1 f/s on air"
        );
        assert!(
            src.contains(concat!("NDN_AU_", "TXAGC12")),
            "the twelve-group path must stay reachable for investigation, behind a gate"
        );
    }

    /// ☠ A blanket driver-side index floor is MEASURED HARMFUL on this part: a floor of 4 took
    /// it from 1258 f/s to 33-50 f/s. `index_base` legitimately returns values below any floor you
    /// would pick (0 for CCK on 5 GHz), and forcing those up is worse than the problem the floor
    /// was for. Bottom-of-scale protection belongs in `RadioCapability::min_tx_power`, which the
    /// policy honours and which can distinguish a low base from an over-deep back-off.
    #[test]
    fn the_calibrated_path_has_no_blanket_floor() {
        let src = include_str!("rtl8812au.rs");
        // The DECLARATION, not a mention — this test's own doc names the constant.
        assert!(
            // Split so the needle does not match this line itself.
            !src.contains(concat!("const ", "TXAGC_CAL", "_FLOOR")),
            "a blanket TXAGC floor was reintroduced — it MEASURED 1258 -> 33 f/s on this part"
        );
        // And the calibrated write must clamp from 0, not from a floor.
        assert!(
            src.contains("(base + offset).clamp(0, TXAGC_MAX as i32)"),
            "the calibrated TXAGC write no longer clamps from 0"
        );
    }

    /// Read back the 7-bit `TX_RATE` field (DWORD4, bits\[6:0\]) of a built TX descriptor.
    fn desc_tx_rate(d: &[u8; TXDESC_SIZE]) -> u32 {
        u32::from_le_bytes(d[16..20].try_into().unwrap()) & 0x7f
    }

    /// The regression guard for the fixed bug: a control-plane-decided MCS must actually
    /// reach the descriptor's on-air rate field. Two distinct rates resolve to two distinct
    /// DESC codes, and the build path carries each one through — so `set_rate` is no longer a
    /// no-op that leaves every frame at legacy 6 Mbps.
    #[test]
    fn set_rate_changes_on_air_desc_rate() {
        let ht5 = desc_rate_for(&McsDescriptor::ht(5), &TxIntent::CONSERVATIVE);
        let vht7 = desc_rate_for(&McsDescriptor::vht(7), &TxIntent::CONSERVATIVE);
        assert_eq!(ht5, DESC_RATE_MCS0 + 5, "HT MCS5 -> DESC_RATEMCS0 + 5");
        assert_eq!(
            vht7,
            DESC_RATE_VHT1SS_MCS0 + 7,
            "VHT-1SS MCS7 -> DESC_RATEVHTSS1MCS0 + 7"
        );
        assert_ne!(ht5, vht7);

        // The same rate value flowing through the build path lands in TX_RATE, and the two
        // built descriptors differ in exactly that field.
        let d_ht = Rtl8812auBackend::build_txdesc(64, ht5, QSLT_MGNT);
        let d_vht = Rtl8812auBackend::build_txdesc(64, vht7, QSLT_MGNT);
        assert_eq!(desc_tx_rate(&d_ht), ht5);
        assert_eq!(desc_tx_rate(&d_vht), vht7);
        assert_ne!(
            desc_tx_rate(&d_ht),
            desc_tx_rate(&d_vht),
            "the decided rate must change the on-air DESC rate code"
        );
    }

    /// VHT `nss >= 2` selects the 2-stream base code, not the 1-stream one.
    #[test]
    fn vht_2ss_uses_the_two_stream_base() {
        let r = desc_rate_for(&McsDescriptor::vht_2ss(3), &TxIntent::CONSERVATIVE);
        assert_eq!(r, DESC_RATE_VHT2SS_MCS0 + 3);
    }

    /// A `MostRobust` frame drops to legacy 6 Mbps OFDM regardless of the stored rate, so
    /// control/report frames stay decodable by the worst receiver in range.
    #[test]
    fn most_robust_forces_legacy_regardless_of_stored_mcs() {
        let r = desc_rate_for(&McsDescriptor::vht_2ss(8), &TxIntent::ROBUST);
        assert_eq!(r, DESC_RATE_6M);
    }
}
