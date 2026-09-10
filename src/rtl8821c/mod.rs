//! Userspace libusb backend for the **RTL8821CU** (`0bda:c820` and siblings) —
//! a 1×1 2.4/5 GHz 802.11ac dongle — modelled on [`crate::LibUsbRtl88xxBackend`]
//! (the RTL8812EU/8822E backend) but ported from **rtw88** (`rtw88_8821c`), the
//! kernel driver actually bound to this chip.
//!
//! Why a separate module: the 8821C is a different Realtek HAL generation
//! (rtw88, not the 8822E vendor halmac), with a different power sequence, a DDMA
//! firmware-download path, a 48-byte rtw88 TX descriptor / 24-byte RX
//! descriptor, and — crucially — **firmware-offloaded IQK** (no host
//! IQK/LCK/DPK/DACK at all). That last point makes it far simpler to radiate
//! than the 8822E, whose unported `halrf` calibration was the hard part.
//!
//! Bring-up is staged and verified against the golden usbmon trace in
//! `golden/c820-usbmon-2026-06-17/`; see `docs/rtl8821cu-port-reference.md` for
//! the full register/flow reference (every `file:line` here cites lwfinger/rtw88).
//!
//! Status: **complete bring-up path, pending on-hardware validation.** All
//! stages are ported from the reference: USB/reg (`0x4e0` quirk), RF access,
//! power sequence, DDMA firmware download ([`fw`]), MAC init ([`mac`]), PHY
//! table apply with the phy_cond branch evaluator, per-channel BB/RF/RX-DFIR +
//! TX-power-index write + firmware IQK ([`phy`]), the 48-byte txdesc / 24-byte
//! rxdesc, and promiscuous monitor RCR. Not yet validated on-air; bring up
//! against the golden trace (`NDN_RADIO_LOG_WRITES=1` and diff). Known
//! simplifications, each marked TODO(hw): the rest of `rtw_mac_pre_system_cfg`,
//! efuse-derived phy_cond (cut/rfe/pkg) and BB-swing, and the regulatory
//! power-by-rate pipeline (a uniform TX index is written instead — the
//! `bb_pg`/`txpwr_lmt` tables are generated and ready to wire in).

// This is a staged bring-up scaffold: several register/base constants are
// present for stages not yet wired (firmware download, the WLAN_* MAC-init
// group, TX power) and cited in the reference doc. Allow them until used.
#![allow(dead_code)]

use std::io;
use std::sync::Arc;
use std::sync::atomic::{AtomicU8, AtomicU16, Ordering};
use std::time::Duration;

use async_trait::async_trait;
use rusb::{Context, Device, DeviceHandle, Direction, TransferType, UsbContext};

use ndn_transport::FaceError;

use crate::frame;
use crate::realtek_rx;
use crate::{CapturedFrame, FrameFormat, FrameIo, InjectFrame};
use ndn_frame_io::ClockDomainId;
use ndn_radio_hal::bringup::{
    AppliedPower, Assert, BringUp, BringUpFailure, BringUpReport, Ctx, Degradation, Guards, Plan,
    PlanId, PlanRun, PowerReference, PowerRequest, PowerWrite, ProofRequirement, PumpPolicy,
    RadioState, Role, Stage, Step, StepClass, StepId, StepOutcome,
};
use ndn_radio_hal::{Band, RadioCapability, RadioProfile};

mod coex;
mod efuse;
mod fw;
mod mac;
mod phy;
mod pwrseq;
mod tables;

use pwrseq::PwrCfg;

// ── USB vendor request (rtw_usb, usb.h:15) ──────────────────────────────────
const VENDOR_REQ: u8 = 0x05;
const REQ_READ: u8 = 0xc0; // RTW_USB_CMD_READ  (device→host | vendor | device)
const REQ_WRITE: u8 = 0x40; // RTW_USB_CMD_WRITE (host→device | vendor | device)
const CTRL_TIMEOUT: Duration = Duration::from_millis(500);

pub const REALTEK_VID: u16 = 0x0bda;
/// RTL8821CU / RTL8811CU USB product IDs (rtw88_8821cu + out-of-tree variants).
pub const RTL8821CU_PIDS: &[u16] = &[
    0xc820, 0xc82a, 0xc821, 0xc82b, 0xb82b, 0xb820, 0xc811, 0x8811, 0xc814,
];

// ── Registers (rtw88 reg.h) ─────────────────────────────────────────────────
const REG_SYS_FUNC_EN: u16 = 0x0002;
const REG_RSV_CTRL: u16 = 0x001c;
const REG_RF_CTRL: u16 = 0x001f;
const REG_SYS_CFG1: u16 = 0x00f0; // cut version in bits [15:12]
const REG_CR: u16 = 0x0100;
const REG_MCUFW_CTRL: u16 = 0x0080;
const REG_RCR: u16 = 0x0608;
const REG_RXFLTMAP0: u16 = 0x06a0;
const REG_RXFLTMAP1: u16 = 0x06a2;
const REG_RXFLTMAP2: u16 = 0x06a4;
const REG_MAR_LO: u16 = 0x0620;
const REG_MAR_HI: u16 = 0x0624;

// RF (radio) indirect access — 8821C (rtw8821c.c:2019-2020).
const RF_BASE_ADDR_A: u16 = 0x2800;
const RF_SIPI_ADDR_A: u16 = 0x0c90;
const RFREG_MASK: u32 = 0xfffff;
const RF_CFGCH: u8 = 0x18; // channel/band/bandwidth
const RF_XTALX2: u8 = 0xb8; // PLL reload (BIT19)
const RF_LUTDBG: u8 = 0xdf;

// RCR bits (reg.h:505).
const BIT_APP_FCS: u32 = 1 << 31;
const BIT_APP_MIC: u32 = 1 << 30;
const BIT_APP_ICV: u32 = 1 << 29;
const BIT_APP_PHYSTS: u32 = 1 << 28;
const BIT_PKTCTL_DLEN: u32 = 1 << 20;
const BIT_HTC_LOC_CTRL: u32 = 1 << 14;
const BIT_AICV: u32 = 1 << 9;
const BIT_ACRC32: u32 = 1 << 8;
const BIT_CBSSID_BCN: u32 = 1 << 7;
const BIT_CBSSID_DATA: u32 = 1 << 6;
const BIT_AB: u32 = 1 << 3;
const BIT_AM: u32 = 1 << 2;
const BIT_APM: u32 = 1 << 1;
const BIT_AAP: u32 = 1 << 0;

/// Promiscuous monitor RCR: defaults + accept-all-addr1 + accept CRC/ICV-error
/// frames, with BSSID filtering off (see reference §11).
const RCR_MONITOR: u32 = BIT_APP_FCS
    | BIT_APP_MIC
    | BIT_APP_ICV
    | BIT_APP_PHYSTS
    | BIT_PKTCTL_DLEN
    | BIT_HTC_LOC_CTRL
    | BIT_AICV
    | BIT_ACRC32
    | BIT_AB
    | BIT_AM
    | BIT_APM
    | BIT_AAP;

// ── Descriptor sizes / rate codes (tx.h, main.h:398) ────────────────────────
const TX_DESC_SIZE: usize = 48;

/// Locally-administered BSSID for our ad-hoc (IBSS) cell when `NDN_RADIO_IBSS`
/// is set — used as REG_BSSID and as 802.11 addr3 on injected frames.
const IBSS_BSSID: [u8; 6] = [0x02, 0x4e, 0x44, 0x4e, 0xbe, 0xef];
const RX_DESC_SIZE: usize = 24;
const DESC_RATE_OFDM6M: u8 = 0x04;
const DESC_RATE_MCS0: u8 = 0x0c; // HT MCS0
const DESC_RATE_VHT1SS_MCS0: u8 = 0x2c;

// phy_cond bitfield (main.h struct rtw_phy_cond, little-endian).
const INTF_USB_PHY: u8 = 1 << 1; // INTF_USB = BIT(1) = 2

fn usb_err(e: rusb::Error) -> FaceError {
    FaceError::Io(io::Error::other(format!("rtl8821cu usb: {e}")))
}
fn init_err(what: String) -> FaceError {
    FaceError::Io(io::Error::other(what))
}

/// The driver-side phy condition the table branch directives are matched
/// against (rtw88 `check_positive`, phy.c). For 8821C only `cut`, `pkg`, `intf`,
/// `rfe` participate, and `rfe` must match exactly.
#[derive(Clone, Copy, Default)]
struct PhyCond {
    cut: u8,
    pkg: u8,
    intf: u8,
    rfe: u8,
}

pub struct Rtl8821cuBackend {
    handle: Arc<DeviceHandle<Context>>,
    bulk_out: u8,
    /// All bulk-OUT endpoint addresses on the WLAN interface, in descriptor
    /// order. rtw88 maps TX queues (MGMT/HIGH/data) to distinct OUT pipes; the
    /// radiating scan-probe path uses the MGMT pipe, not necessarily the first.
    bulk_outs: Vec<u8>,
    bulk_in: u8,
    format: FrameFormat,
    seq: AtomicU16,
    /// Current transmit rate as state ([`FrameIo::set_rate`]); `None` ⇒ resolve the
    /// frame's intent. Retires the per-frame `inject_at` path.
    cur_mcs: std::sync::Mutex<Option<crate::McsDescriptor>>,
    /// Monotonic H2C packet sequence (firmware echoes it; must increment).
    h2c_seq: AtomicU16,
    /// Round-robin HMEBOX index (0-3) for the H2C mailbox path.
    h2c_box: AtomicU8,
    cur_channel: AtomicU8,
    /// The per-rate TXAGC index the last `set_channel` wrote, so the bring-up report can name the
    /// power this part is actually at rather than leaving it blank.
    tx_power_idx: AtomicU8,
    /// Channel bandwidth (`RTW_CHANNEL_WIDTH_*`: 0=20). Monitor is 20 MHz.
    cur_bw: AtomicU8,
    /// CCK TX-filter params snapshotted from 0xa24/0xa28/0xaac after the BB
    /// table load — replayed per 2.4 GHz channel (rtw8821c.c:196).
    ch_param: std::sync::Mutex<[u32; 3]>,
    /// Driver phy-condition for table selection, populated at bring-up.
    cond: std::sync::Mutex<PhyCond>,
    /// efuse `rfe_option & 0x1f` — the board's RFE profile (drives the 0xcb4
    /// front-end value, the agc-btg table, and antenna routing). 0xff = unread.
    rfe_option: AtomicU8,
    /// efuse-derived: antenna routes through the BTG front-end path.
    rfe_btg: std::sync::atomic::AtomicBool,
    rx_pending: std::sync::Mutex<std::collections::VecDeque<CapturedFrame>>,
    /// Set when a background RX pump is running (keeps several concurrent bulk-IN
    /// reads in flight — Realtek USB needs continuous outstanding IN requests for
    /// the RXDMA→USB engine to push frames; a single blocking read gets nothing).
    rx_pumped: std::sync::atomic::AtomicBool,
    rx_notify: tokio::sync::Notify,
    /// Count of every raw 802.11 RX unit seen on bulk-IN (before any NDN
    /// filtering) — the honest "is the receiver working at all" metric.
    rx_raw_count: std::sync::atomic::AtomicU64,
    /// Per-device TSF clock domain (`bus<<8 | address`) — the identity every RX hardware stamp is
    /// keyed on, so a receiver's [`CapturedFrame::stamp`] is comparable only within this device (#41).
    tsf_domain: ClockDomainId,
    /// True when the `ibss_opmode` rung ran — i.e. when [`PLAN_8821CU_IBSS`] was the plan.
    ///
    /// ★ It replaces a per-frame `std::env::var("NDN_RADIO_IBSS")` in `build_80211`. That read was
    /// the other half of a plan-level experiment leaking into the data path: the ladder's IBSS
    /// setup and the frame builder's addr3 had to agree, and the only thing making them agree was
    /// that both read the same environment variable on every single frame. Now the rung that puts
    /// the MAC into ad-hoc mode is the one thing that sets it.
    ibss_mode: std::sync::atomic::AtomicBool,
}

impl Rtl8821cuBackend {
    /// Find and open the first RTL8821CU dongle, claim the WLAN interface, and
    /// locate its bulk endpoints. (Bring-up is a separate call so the open path
    /// can be unit-exercised without a full radio init.)
    pub fn open() -> Result<Self, FaceError> {
        // Pass 1 — reset any matching dongle to a clean power-on state. This
        // re-enumerates the device (closest thing to a physical replug) and
        // recovers a chip wedged by a previous aborted bring-up. The handle is
        // discarded: after a reset the device comes back at a new address, so we
        // must re-scan and open it fresh (pass 2) rather than reuse this handle.
        {
            let context = Context::new().map_err(usb_err)?;
            for device in context.devices().map_err(usb_err)?.iter() {
                let desc = device.device_descriptor().map_err(usb_err)?;
                if desc.vendor_id() == REALTEK_VID
                    && RTL8821CU_PIDS.contains(&desc.product_id())
                    && let Ok(h) = device.open()
                {
                    let _ = h.reset();
                }
            }
        }
        std::thread::sleep(Duration::from_millis(1500)); // let it re-enumerate

        // Pass 2 — open + claim the (re-enumerated) device.
        let context = Context::new().map_err(usb_err)?;
        for device in context.devices().map_err(usb_err)?.iter() {
            let desc = device.device_descriptor().map_err(usb_err)?;
            if desc.vendor_id() == REALTEK_VID && RTL8821CU_PIDS.contains(&desc.product_id()) {
                return Self::claim(device);
            }
        }
        Err(FaceError::Io(io::Error::new(
            io::ErrorKind::NotFound,
            "no RTL8821CU dongle found (Realtek 0bda:c811/c820/...)",
        )))
    }

    fn claim(device: Device<Context>) -> Result<Self, FaceError> {
        // Per-device TSF clock domain (bus<<8 | address) — read before the device is opened (#41).
        let tsf_domain =
            ClockDomainId((u32::from(device.bus_number()) << 8) | u32::from(device.address()));
        let handle = Arc::new(device.open().map_err(usb_err)?);
        let _ = handle.set_auto_detach_kernel_driver(true);
        let config = device.active_config_descriptor().map_err(usb_err)?;

        // The WLAN function is the interface that exposes the bulk endpoints
        // (the 8821C combo also has BT/isoc interfaces). Claim that one.
        let (mut wlan_iface, mut bulk_in, mut bulk_out) = (None, None, None);
        let mut bulk_outs: Vec<u8> = Vec::new();
        for iface in config.interfaces() {
            for d in iface.descriptors() {
                let mut has_in = None;
                let mut has_out = None;
                let mut outs: Vec<u8> = Vec::new();
                for ep in d.endpoint_descriptors() {
                    if ep.transfer_type() != TransferType::Bulk {
                        continue;
                    }
                    match ep.direction() {
                        Direction::In if has_in.is_none() => has_in = Some(ep.address()),
                        Direction::Out => {
                            if has_out.is_none() {
                                has_out = Some(ep.address());
                            }
                            outs.push(ep.address());
                        }
                        _ => {}
                    }
                }
                if let (Some(i), Some(o)) = (has_in, has_out) {
                    wlan_iface = Some(iface.number());
                    bulk_in = Some(i);
                    bulk_out = Some(o);
                    bulk_outs = outs;
                }
            }
        }
        if std::env::var("NDN_RADIO_EP_DEBUG").is_ok() {
            eprintln!(
                "8821cu WLAN bulk OUT endpoints: {}  (IN {:#04x})",
                bulk_outs
                    .iter()
                    .map(|e| format!("{e:#04x}"))
                    .collect::<Vec<_>>()
                    .join(" "),
                bulk_in.unwrap_or(0)
            );
        }
        let iface = wlan_iface.ok_or_else(|| {
            FaceError::Io(io::Error::new(
                io::ErrorKind::NotFound,
                "RTL8821CU: no interface with bulk IN+OUT endpoints",
            ))
        })?;
        let _ = handle.set_auto_detach_kernel_driver(true);
        handle.claim_interface(iface).map_err(usb_err)?;

        Ok(Self {
            handle,
            bulk_out: bulk_out.unwrap(),
            bulk_outs,
            bulk_in: bulk_in.unwrap(),
            format: FrameFormat::default(),
            seq: AtomicU16::new(0),
            cur_mcs: std::sync::Mutex::new(None),
            h2c_seq: AtomicU16::new(0),
            h2c_box: AtomicU8::new(0),
            cur_channel: AtomicU8::new(0),
            tx_power_idx: AtomicU8::new(0x2d),
            cur_bw: AtomicU8::new(0),
            ch_param: std::sync::Mutex::new([0; 3]),
            cond: std::sync::Mutex::new(PhyCond::default()),
            rfe_option: AtomicU8::new(0xff),
            rfe_btg: std::sync::atomic::AtomicBool::new(false),
            rx_pending: std::sync::Mutex::new(std::collections::VecDeque::new()),
            rx_pumped: std::sync::atomic::AtomicBool::new(false),
            rx_notify: tokio::sync::Notify::new(),
            rx_raw_count: std::sync::atomic::AtomicU64::new(0),
            tsf_domain,
            ibss_mode: std::sync::atomic::AtomicBool::new(false),
        })
    }

    /// Start `depth` background threads each continuously reading the bulk-IN
    /// endpoint, parsing RX units into `rx_pending`. Keeps several concurrent
    /// reads in flight — required for the chip to DMA RX to USB. Call after
    /// bring-up, once the backend is in an `Arc`.
    pub fn spawn_rx_pump(self: &Arc<Self>, depth: usize) {
        self.rx_pumped.store(true, Ordering::Relaxed);
        for _ in 0..depth.max(1) {
            let weak = Arc::downgrade(self);
            std::thread::spawn(move || {
                let mut buf = vec![0u8; 32768];
                loop {
                    let Some(dev) = weak.upgrade() else { break };
                    let r = dev
                        .handle
                        .read_bulk(dev.bulk_in, &mut buf, Duration::from_millis(200));
                    if std::env::var("NDN_RADIO_RX_DEBUG").is_ok() {
                        match &r {
                            Ok(n) => eprintln!("read_bulk(ep {:#04x}) -> Ok({n})", dev.bulk_in),
                            Err(rusb::Error::Timeout) => {}
                            Err(e) => eprintln!("read_bulk(ep {:#04x}) -> Err({e})", dev.bulk_in),
                        }
                    }
                    match r {
                        Ok(n) if n > 0 => {
                            let mut off = 0;
                            {
                                let mut q = dev.rx_pending.lock().unwrap();
                                while let Some((decoded, advance)) = dev.parse_rx_at(&buf[..n], off)
                                {
                                    for f in decoded {
                                        q.push_back(f);
                                    }
                                    off += advance;
                                    if off + RX_DESC_SIZE > n {
                                        break;
                                    }
                                }
                            }
                            dev.rx_notify.notify_one();
                        }
                        _ => {} // timeout / error: re-submit
                    }
                }
            });
        }
    }

    /// Override the on-air frame format (e.g. ESP-NOW).
    pub fn with_format(mut self, format: FrameFormat) -> Self {
        self.format = format;
        self
    }

    // ── register access (usb.c:72-166) ──────────────────────────────────────

    fn read_reg(&self, addr: u16, buf: &mut [u8]) -> Result<(), FaceError> {
        let n = self
            .handle
            .read_control(REQ_READ, VENDOR_REQ, addr, 0, buf, CTRL_TIMEOUT)
            .map_err(usb_err)?;
        if n != buf.len() {
            return Err(init_err(format!("8821cu read_reg({addr:#06x}) short {n}")));
        }
        self.reg_sec(addr, buf);
        Ok(())
    }

    fn write_reg(&self, addr: u16, data: &[u8]) -> Result<(), FaceError> {
        log_write(addr, data);
        let n = self
            .handle
            .write_control(REQ_WRITE, VENDOR_REQ, addr, 0, data, CTRL_TIMEOUT)
            .map_err(usb_err)?;
        if n != data.len() {
            return Err(init_err(format!("8821cu write_reg({addr:#06x}) short {n}")));
        }
        self.reg_sec(addr, data);
        Ok(())
    }

    /// The 8821C `rtw_usb_reg_sec` quirk (usb.c:42-70): after any access to an
    /// "on" section register (`addr <= 0xff` or `0x1000..=0x10ff`), the kernel
    /// issues an extra 1-byte write to `0x4e0` with the same data. Replicated so
    /// our bring-up matches the golden trace byte-for-byte.
    fn reg_sec(&self, addr: u16, data: &[u8]) {
        let on_section = addr <= 0xff || (0x1000..=0x10ff).contains(&addr);
        if !on_section {
            return;
        }
        let byte = [*data.first().unwrap_or(&0)];
        let _ = self
            .handle
            .write_control(REQ_WRITE, VENDOR_REQ, 0x04e0, 0, &byte, CTRL_TIMEOUT);
    }

    /// Total raw 802.11 RX units seen on bulk-IN since open (before NDN
    /// filtering) — the "is the receiver working" metric.
    pub fn raw_rx_count(&self) -> u64 {
        self.rx_raw_count.load(Ordering::Relaxed)
    }

    /// Reset/kick the BB false-alarm + CRC counters (rtw8821c_false_alarm_statistics
    /// tail) so they accumulate from now — for the RX localization diagnostic.
    pub fn debug_reset_rx_counters(&self) -> Result<(), FaceError> {
        self.set32(0x09a4, 1 << 17)?; // REG_FAS BIT(17)
        self.clr32(0x09a4, 1 << 17)?;
        self.set32(0x0b58, 1 << 0)?; // REG_CNTRST BIT(0)
        self.clr32(0x0b58, 1 << 0)?;
        Ok(())
    }

    /// The board's RFE profile read from efuse: `(rfe_option_full, rfe_btg)`.
    /// `rfe_option_full == 0xff` means the efuse read failed (defaults in use).
    pub fn rfe_profile(&self) -> (u8, bool) {
        (
            self.rfe_option.load(Ordering::Relaxed),
            self.rfe_btg.load(Ordering::Relaxed),
        )
    }

    /// Read an 8-bit register (also exposed for diagnostics / examples).
    pub fn read8(&self, addr: u16) -> Result<u8, FaceError> {
        let mut b = [0u8; 1];
        self.read_reg(addr, &mut b)?;
        Ok(b[0])
    }
    /// Read a 16-bit register (also exposed for diagnostics / examples).
    pub fn read16(&self, addr: u16) -> Result<u16, FaceError> {
        let mut b = [0u8; 2];
        self.read_reg(addr, &mut b)?;
        Ok(u16::from_le_bytes(b))
    }
    /// Read a 32-bit register (also exposed for diagnostics / examples).
    pub fn read32(&self, addr: u16) -> Result<u32, FaceError> {
        let mut b = [0u8; 4];
        self.read_reg(addr, &mut b)?;
        Ok(u32::from_le_bytes(b))
    }
    fn write8(&self, addr: u16, v: u8) -> Result<(), FaceError> {
        self.write_reg(addr, &[v])
    }
    fn write16(&self, addr: u16, v: u16) -> Result<(), FaceError> {
        self.write_reg(addr, &v.to_le_bytes())
    }
    fn write32(&self, addr: u16, v: u32) -> Result<(), FaceError> {
        self.write_reg(addr, &v.to_le_bytes())
    }

    fn set8(&self, addr: u16, bits: u8) -> Result<(), FaceError> {
        let v = self.read8(addr)?;
        self.write8(addr, v | bits)
    }
    fn clr8(&self, addr: u16, bits: u8) -> Result<(), FaceError> {
        let v = self.read8(addr)?;
        self.write8(addr, v & !bits)
    }
    fn set32(&self, addr: u16, bits: u32) -> Result<(), FaceError> {
        let v = self.read32(addr)?;
        self.write32(addr, v | bits)
    }
    fn clr32(&self, addr: u16, bits: u32) -> Result<(), FaceError> {
        let v = self.read32(addr)?;
        self.write32(addr, v & !bits)
    }
    /// Read-modify-write a sub-field: clear `mask` bits, OR in `val` (already
    /// positioned within `mask`).
    fn write8_mask(&self, addr: u16, mask: u8, val: u8) -> Result<(), FaceError> {
        let v = self.read8(addr)?;
        self.write8(addr, (v & !mask) | (val & mask))
    }

    /// Poll `(read32(addr) & mask) == target`, up to `tries` × `delay`.
    fn poll32(
        &self,
        addr: u16,
        mask: u32,
        target: u32,
        tries: u32,
        delay: Duration,
    ) -> Result<(), FaceError> {
        for _ in 0..tries {
            if (self.read32(addr)? & mask) == target {
                return Ok(());
            }
            std::thread::sleep(delay);
        }
        Err(init_err(format!(
            "8821cu poll timeout @{addr:#06x} mask={mask:#x}"
        )))
    }

    /// Synchronous bulk-OUT write to the single WLAN endpoint.
    fn bulk_write(&self, buf: &[u8]) -> Result<(), FaceError> {
        let n = self
            .handle
            .write_bulk(self.bulk_out, buf, Duration::from_secs(1))
            .map_err(usb_err)?;
        if n != buf.len() {
            return Err(init_err(format!(
                "8821cu bulk_write short {n}/{}",
                buf.len()
            )));
        }
        Ok(())
    }

    // ── RF (radio) register access via BB indirect ports (phy.c:961-1071) ────

    /// Read RF register `addr` on path A: `read32_mask(0x2800 + addr*4, mask)`.
    pub fn read_rf(&self, addr: u8, mask: u32) -> Result<u32, FaceError> {
        let direct = RF_BASE_ADDR_A + ((addr as u16) << 2);
        let v = self.read32(direct)? & (mask & RFREG_MASK);
        Ok(v >> (mask & RFREG_MASK).trailing_zeros())
    }

    /// Write RF register `addr` on path A via SIPI: `write32(0xc90,
    /// (addr<<20)|data20)`. Partial masks read-modify-write first.
    fn write_rf(&self, addr: u8, mask: u32, data: u32) -> Result<(), FaceError> {
        let mask = mask & RFREG_MASK;
        let data = if mask != RFREG_MASK {
            let cur = self.read_rf(addr, RFREG_MASK)?;
            let shift = mask.trailing_zeros();
            (cur & !mask) | ((data << shift) & mask)
        } else {
            data & RFREG_MASK
        };
        let word = (((addr as u32) << 20) | data) & 0x0fff_ffff;
        self.write32(RF_SIPI_ADDR_A, word)?;
        std::thread::sleep(Duration::from_micros(13));
        Ok(())
    }

    // ── power sequence (mac.c:185-312) ──────────────────────────────────────

    /// Apply a power flow (a list of sub-sequences) for the USB interface and
    /// our cut version (rtw88 `rtw_pwr_seq_parser`).
    fn apply_pwr_flow(&self, flow: &[&[PwrCfg]], cut: u8) -> Result<(), FaceError> {
        let cut_mask = pwrseq::cut_version_to_mask(cut);
        for seq in flow {
            for c in *seq {
                if c.cmd == pwrseq::CMD_END {
                    break;
                }
                if (c.intf_mask & pwrseq::INTF_USB) == 0 || (c.cut_mask & cut_mask) == 0 {
                    continue;
                }
                // We only ever target the MAC base over USB; SDIO-base entries
                // are already filtered by the interface mask above.
                if c.base != pwrseq::BASE_MAC && c.base != pwrseq::BASE_USB {
                    continue;
                }
                match c.cmd {
                    pwrseq::CMD_WRITE => {
                        let v = self.read8(c.offset)?;
                        self.write8(c.offset, (v & !c.mask) | (c.value & c.mask))?;
                    }
                    pwrseq::CMD_POLLING => {
                        let mut ok = false;
                        for _ in 0..pwrseq::POLLING_CNT {
                            if (self.read8(c.offset)? & c.mask) == (c.value & c.mask) {
                                ok = true;
                                break;
                            }
                            std::thread::sleep(Duration::from_micros(50));
                        }
                        if !ok {
                            return Err(init_err(format!(
                                "8821cu pwrseq poll timeout @{:#06x}",
                                c.offset
                            )));
                        }
                    }
                    pwrseq::CMD_DELAY => {
                        let d = c.offset as u64;
                        if c.value == pwrseq::DELAY_US {
                            std::thread::sleep(Duration::from_micros(d));
                        } else {
                            std::thread::sleep(Duration::from_millis(d));
                        }
                    }
                    _ => {}
                }
            }
        }
        Ok(())
    }

    // ── PHY table application (phy.c:1199-1854) ──────────────────────────────

    /// Apply a flat phy_cond `{addr,data}` table, honouring the embedded
    /// IF/ELIF/ELSE/ENDIF branch directives and the per-op write width + delay
    /// sentinels. `do_cfg` is one of [`CfgKind`].
    fn apply_phy_table(&self, table: &[u32], kind: CfgKind) -> Result<(), FaceError> {
        let cond = *self.cond.lock().unwrap();
        let mut i = 0;
        let mut matched = true; // outside any branch → apply
        let mut skipped = false;
        while i + 1 < table.len() {
            let w0 = table[i];
            // A directive has pos or neg set in the top bits and consumes 2 words
            // (the condition) — or 4 for IF/ELIF (cond + the eval marker word).
            let pos = (w0 >> 31) & 1 != 0;
            let neg = (w0 >> 30) & 1 != 0;
            if pos {
                let branch = (w0 >> 28) & 0x3;
                match branch {
                    3 => {
                        // ENDIF
                        matched = true;
                        skipped = false;
                        i += 2;
                    }
                    2 => {
                        // ELSE
                        matched = !skipped;
                        i += 2;
                    }
                    _ => {
                        // IF / ELIF: stash condition, the next word (neg) evaluates it.
                        // (cond word, 0, eval-marker, 0) → 4 words.
                        let c = parse_phy_cond(w0);
                        let take = check_positive(c, cond);
                        if !skipped && take {
                            matched = true;
                            skipped = true;
                        } else {
                            matched = false;
                        }
                        i += 4;
                    }
                }
                continue;
            }
            if neg {
                // standalone eval marker without a leading pos — skip its 2 words
                i += 2;
                continue;
            }
            // Normal {addr, data} pair.
            let addr = w0;
            let data = table[i + 1];
            if matched {
                self.do_cfg(kind, addr, data)?;
            }
            i += 2;
        }
        Ok(())
    }

    fn do_cfg(&self, kind: CfgKind, addr: u32, data: u32) -> Result<(), FaceError> {
        match kind {
            CfgKind::Mac => self.write8(addr as u16, data as u8),
            CfgKind::Agc => self.write32(addr as u16, data),
            CfgKind::Bb => {
                // BB delay sentinels (phy.c:1823).
                match addr & 0xfff {
                    0xfe => std::thread::sleep(Duration::from_millis(50)),
                    0xfd => std::thread::sleep(Duration::from_millis(5)),
                    0xfc => std::thread::sleep(Duration::from_millis(1)),
                    0xfb => std::thread::sleep(Duration::from_micros(50)),
                    0xfa => std::thread::sleep(Duration::from_micros(5)),
                    0xf9 => std::thread::sleep(Duration::from_micros(1)),
                    _ => return self.write32(addr as u16, data),
                }
                Ok(())
            }
            CfgKind::Rf => {
                // RF delay sentinels (phy.c:1843).
                match addr & 0xfff {
                    0xffe => std::thread::sleep(Duration::from_millis(50)),
                    0xfe => std::thread::sleep(Duration::from_micros(100)),
                    _ => return self.write_rf(addr as u8, RFREG_MASK, data),
                }
                Ok(())
            }
        }
    }

    /// `rtw8821c_phy_set_param` (rtw8821c.c:159): re-enable the BB/RF domain that
    /// `pre_system_cfg` disabled, **before** loading the PHY tables. Without this
    /// the BB/RF stay powered off — RF registers read back garbage and the
    /// receiver never delivers frames. This was the bug that left RX dead.
    fn phy_set_param(&self) -> Result<(), FaceError> {
        // power on BB domain
        self.set8(0x0002, 1 << 6)?; // SYS_FUNC_EN |= FEN_PCIEA
        // toggle BB reset (set, clear, set)
        self.set8(0x0002, (1 << 0) | (1 << 1))?;
        self.clr8(0x0002, (1 << 0) | (1 << 1))?;
        self.set8(0x0002, (1 << 0) | (1 << 1))?;
        // enable RF
        self.write8(0x001f, (1 << 0) | (1 << 1) | (1 << 2))?; // RF_CTRL = EN|RSTB|SDM_RSTB
        std::thread::sleep(Duration::from_micros(10));
        self.write8(0x00ef, (1 << 0) | (1 << 1) | (1 << 2))?; // WLRF1+3 = EN|RSTB|SDM_RSTB
        std::thread::sleep(Duration::from_micros(10));
        self.clr32(0x0808, (1 << 28) | (1 << 29))?; // RXPSEL clr RX_PSEL_RST
        self.load_tables()
    }

    fn load_tables(&self) -> Result<(), FaceError> {
        self.apply_phy_table(tables::RTW8821C_MAC, CfgKind::Mac)?;
        self.apply_phy_table(tables::RTW8821C_BB, CfgKind::Bb)?;
        self.apply_phy_table(tables::RTW8821C_AGC, CfgKind::Agc)?;
        // BTG boards load the extra BTG AGC table (rfe_def->agc_btg_tbl).
        if self.rfe_btg.load(Ordering::Relaxed) {
            self.apply_phy_table(tables::RTW8821C_AGC_BTG_TYPE2, CfgKind::Agc)?;
        }
        // 8821C-specific REG_RFE_CTRL8 (0xcb4) between AGC and RF tables
        // (rtw_phy_load_tables, phy.c:1889), keyed on the board's rfe_option.
        let rfe = self.rfe_option.load(Ordering::Relaxed);
        let cb4 = if (0x28..=0x2f).contains(&rfe) {
            0x0000_0073
        } else if rfe == 4 {
            0x2000_0077
        } else {
            0x1000_0077
        };
        self.write32(0x0cb4, cb4)?;
        self.apply_phy_table(tables::RTW8821C_RF_A, CfgKind::Rf)?;
        // Snapshot the CCK TX-filter params the BB table left in place; they are
        // replayed per 2.4 GHz channel (rtw8821c.c:196).
        *self.ch_param.lock().unwrap() = [
            self.read32(0x0a24)?,
            self.read32(0x0a28)?,
            self.read32(0x0aac)?,
        ];
        Ok(())
    }

    // ── firmware download (DDMA path — see reference §2) ─────────────────────
    // TODO(hw): the DDMA reserved-page firmware download. Implemented as a
    // separate step because the WLAN function needs fw running only for the
    // (firmware-offloaded) IQK and for TX; monitor RX does not. Tracked against
    // the golden trace's BULK-OUT firmware chunks. See `fw.rs` (to add).

    // ── bring-up ─────────────────────────────────────────────────────────────

    /// **The full monitor-mode bring-up — [`PLAN_8821CU_MONITOR`], `Role::TransmitAndReceive`.**
    /// A thin wrapper over [`bring_up_planned`](Self::bring_up_planned); the sequence lives in the
    /// plan, where every rung states why it is there and a reviewer sees the whole ladder at once.
    /// Each rung is still diffable against the golden usbmon trace with `NDN_RADIO_LOG_WRITES=1`.
    ///
    /// ★ **The four TX-radiate theories are now four named plan variants**, not four `if
    /// std::env::var(..).is_ok()` blocks buried in the ladder (LAW 1). See
    /// [`Rtl8821cVariant`] — and read its doc before running one: all four are **untested
    /// hypotheses**, mutually exclusive, and none has ever been scored against a witness.
    ///
    /// ⚠ **`self: &Arc<Self>`, not `&self`** (M5) — [`Step::run`] takes `&Arc<B>`.
    pub fn bring_up(self: &Arc<Self>, channel: u8) -> Result<BringUpReport, FaceError> {
        self.bring_up_planned(
            channel,
            Rtl8821cVariant::from_env(),
            ProofRequirement::BestAvailable,
        )
        .map(|(report, guards)| {
            debug_assert!(guards.is_empty(), "these plans produce no guards");
            report
        })
        .map_err(drop_partial_report)
    }

    /// **The one entry point.** Everything else is a wrapper over this.
    ///
    /// `variant` selects which of the five plans runs. `Canonical` is the sequence this part has
    /// always run by default; the other four are the never-scored TX-radiate hypotheses, each of
    /// which differs from `Canonical` by exactly one rung and each of which reports its own
    /// [`PlanId`] and therefore its own `plan_digest`. A number taken under one of them can never
    /// be silently compared with a production number.
    // The `Err` is large BECAUSE it carries the partial report — §3's whole point.
    #[allow(clippy::result_large_err)]
    pub fn bring_up_planned(
        self: &Arc<Self>,
        channel: u8,
        variant: Rtl8821cVariant,
        proof: ProofRequirement,
    ) -> Result<(BringUpReport, Guards), BringUpFailure> {
        let run = PlanRun::new(
            "RTL8821CU",
            // This backend claims the first matching dongle and keeps no `DeviceSelect`, so it
            // has no stable bus:port string to report. `Unknown` is the honest answer.
            ndn_radio_hal::DeviceAddress::Unknown,
            self.initial_state(channel, Role::TransmitAndReceive),
        )
        .with_proof(proof);
        let (report, guards) = ndn_radio_hal::bringup::run_plan(self, variant.plan(), &run)?;
        Ok((
            report.with_capability(RadioProfile::capability(self.as_ref())),
            guards,
        ))
    }

    /// The regime a plan starts from — what the caller asked for, which the rungs fill in.
    ///
    /// ⚠ `power` starts at `no_actuator`, **not** at the TXAGC index the M2 hand-filled report
    /// asserted up front. No power has been written when a plan begins; `tune_channel` is the rung
    /// that writes the per-rate TXAGC block, and claiming it before it ran is the shape of defect
    /// this contract exists to remove.
    fn initial_state(&self, channel: u8, role: Role) -> RadioState {
        RadioState {
            channel,
            bw: ndn_radio_hal::Bandwidth::Bw20,
            format: "RawNdn (see with_format)",
            role,
            power: AppliedPower::no_actuator(PowerRequest::NoActuator),
            rate: ndn_radio_hal::RateState::unreported(),
            warm: None,
            contention: None,
            pump: PumpPolicy::CallerOwns,
            facts: Vec::new(),
        }
    }

    /// Send an 8-byte H2C command via the **HMEBOX mailbox** (rtw88
    /// `rtw_fw_send_h2c_command`): round-robin boxes 0-3, poll REG_HMETFR for the
    /// box to be free, write the ext word then the msg word. This is the path the
    /// kernel uses for media-status / RA — distinct from the qsel-19 packet path.
    fn send_h2c_mailbox(&self, w0: u32, w1: u32) -> Result<(), FaceError> {
        let box_n = self.h2c_box.fetch_add(1, Ordering::Relaxed) & 0x3;
        let (box_reg, ex_reg) = (0x01d0 + (box_n as u16) * 4, 0x01f0 + (box_n as u16) * 4);
        // REG_HMETFR (0x1cc): box busy when (state >> box) & 1.
        let mut free = false;
        for _ in 0..300 {
            if (self.read8(0x01cc)? >> box_n) & 1 == 0 {
                free = true;
                break;
            }
            std::thread::sleep(Duration::from_micros(100));
        }
        if !free {
            return Err(init_err(format!("8821cu H2C mailbox {box_n} busy")));
        }
        self.write32(ex_reg, w1)?;
        self.write32(box_reg, w0)?; // writing the msg word triggers the fw read
        Ok(())
    }

    /// `rtw_fw_media_status_report` (H2C_CMD_MEDIA_STATUS_RPT=0x01): tell the
    /// firmware a station at `macid` is connected/disconnected. On rtw88 the
    /// firmware gates PA-keying on this — without a "connected" station the chip
    /// dequeues TX frames but never radiates them. This is the suspected gate for
    /// host-injected TX (the kernel's monitor injection skips it and doesn't
    /// radiate either). w0 = cmd(0x01) | op_mode<<8 | macid<<16.
    fn media_status_report(&self, macid: u8, connect: bool) -> Result<(), FaceError> {
        let w0 = 0x01u32 | ((connect as u32) << 8) | ((macid as u32) << 16);
        self.send_h2c_mailbox(w0, 0)
    }

    /// Put the MAC into **ad-hoc (IBSS) operating mode** so the TX engine keys
    /// frames. Monitor mode leaves REG_CR network-type = NO_LINK, which gates all
    /// host TX (the chip dequeues + drains the FIFO but never keys the PA). The
    /// kernel's IBSS setup (golden_ibss.pcap) sets: REG_CR nettype=ADHOC, valid
    /// EDCA AC params, beacon control, own MAC + a BSSID. Replicated here without
    /// the reserved-page beacon download — the hypothesis being opmode+EDCA+BSSID
    /// alone ungates injected data TX.
    fn setup_ibss(&self) -> Result<(), FaceError> {
        let mac = ndn_frame_io::frame::DEFAULT_SRC; // 02:4e:44:4e:00:01
        // REG_CR (0x100): network type [17:16] = 01 (ADHOC).
        let cr = self.read32(0x0100)?;
        self.write32(0x0100, (cr & !0x0003_0000) | 0x0001_0000)?;
        // EDCA AC params (BE/BK/VI/VO) — valid TXOP/CW so the MAC can contend.
        for r in [0x0500u16, 0x0504, 0x0508, 0x050c] {
            self.write32(r, 0x0000_a432)?;
        }
        self.write8(0x0454, 0x05)?; // SIFS/ack timing (golden IBSS)
        self.write32(0x0420, 0x0070_1f80)?; // REG_FWHW_TXQ_CTRL: enable TX queues
        self.write8(0x0550, 0x18)?; // REG_BCN_CTRL: EN_BCN_FUNCTION | DIS_TSF_UDT
        // Own MAC + IBSS BSSID.
        self.write32(0x0610, u32::from_le_bytes([mac[0], mac[1], mac[2], mac[3]]))?;
        self.write16(0x0614, u16::from_le_bytes([mac[4], mac[5]]))?;
        let b = IBSS_BSSID;
        self.write32(0x0618, u32::from_le_bytes([b[0], b[1], b[2], b[3]]))?;
        self.write16(0x061c, u16::from_le_bytes([b[4], b[5]]))?;
        Ok(())
    }

    /// `rtw_fw_send_ra_info` (H2C_CMD_RA_INFO=0x40): install a rate-adaptation
    /// table entry for `macid` so the firmware knows what rates that station can
    /// use. Without it the firmware's RA has no entry for MACID 0 and silently
    /// drops injected frames (when USE_RATE=0). Fields per rtw88 fw.h:
    /// MACID=bits[15:8], RATE_ID=[20:16], INIT_RA_LVL=[22:21], SGI=BIT23,
    /// BW=[25:24], LDPC=BIT26, VHT=[29:28]; ext word = 32-bit rate bitmap
    /// (DESC_RATE index per bit: 0-3 CCK, 4-11 OFDM 6-54, 12-19 MCS0-7).
    fn ra_info(&self, macid: u8, rate_id: u8, ra_mask: u32) -> Result<(), FaceError> {
        let w0 = 0x40u32 | ((macid as u32) << 8) | ((rate_id as u32 & 0x1f) << 16) | (1u32 << 22); // INIT_RA_LVL = highest, so it tries real rates immediately
        self.send_h2c_mailbox(w0, ra_mask)
    }

    /// `rtw_mac_pre_system_cfg` (mac.c:62) for USB: RF front-end pin mux +
    /// disable BB/RF, run before the card-enable power flow.
    fn pre_system_cfg(&self) -> Result<(), FaceError> {
        self.write8(REG_RSV_CTRL, 0)?;
        // PIN mux: route PAPE/LNAON to WL/BT, clear the LED-pin selects, enable
        // the WL RFE 4/5 pins.
        self.set32(0x0064, (1 << 29) | (1 << 28))?; // PAD_CTRL1 |= PAPE/LNAON_WLBT_SEL
        self.clr32(0x004c, (1 << 25) | (1 << 26))?; // LED_CFG &= ~(PAPE/LNAON_SEL_EN)
        self.set32(0x0040, 1 << 2)?; // GPIO_MUXCFG |= WLRFE_4_5_EN
        // disable BB/RF (re-enabled by the tables)
        self.clr8(0x0002, (1 << 0) | (1 << 1))?; // SYS_FUNC_EN &= ~(BB_RSTB|BB_GLB_RST)
        self.clr8(0x001f, (1 << 0) | (1 << 1) | (1 << 2))?; // RF_CTRL &= ~(RF_EN|RSTB|SDM_RSTB)
        self.clr32(0x00ec, (1 << 24) | (1 << 25) | (1 << 26))?; // WLRF1 &= ~BBRF_EN
        Ok(())
    }

    /// Configure monitor receive. Uses the exact RCR the kernel's `iw set
    /// monitor` produced in the golden trace (0xf410408e) plus all-ones RX filter
    /// maps, rather than a hand-rolled promiscuous value — this is the config
    /// proven to capture on this driver.
    fn set_monitor_rx(&self) -> Result<(), FaceError> {
        // The golden trace's *final* monitor RCR (0xf410400f): AAP set (accept
        // any addr1) and CBSSID_BCN/DATA clear (no BSSID filtering) = truly
        // promiscuous. The intermediate 0xf410408e has CBSSID_BCN set + AAP clear
        // and drops nearly everything — that was the RX-killer.
        self.write32(REG_RCR, 0xf410_400f)?;
        self.write16(REG_RXFLTMAP0, 0xffff)?;
        self.write16(REG_RXFLTMAP1, 0xffff)?;
        self.write16(REG_RXFLTMAP2, 0xffff)?;
        self.write32(REG_MAR_LO, 0xffff_ffff)?;
        self.write32(REG_MAR_HI, 0xffff_ffff)?;
        Ok(())
    }

    /// Set the operating channel (20 MHz monitor): per-channel BB band/BW + BB
    /// swing, the RF synthesiser (`RF 0x18`), RX DFIR, and the per-rate TX-power
    /// index write (the radiate gate). The BB/RF/power methods live in `phy.rs`.
    pub fn set_channel(&self, channel: u8) -> Result<(), FaceError> {
        self.set_channel_bb(channel)?;
        self.set_channel_bb_swing(channel)?;
        self.set_channel_rf(channel)?;
        self.set_channel_rxdfir()?;
        self.cur_channel.store(channel, Ordering::Relaxed);
        // `NDN_RADIO_TXPWR=<0..63>` overrides the default per-rate index. Read HERE, in the
        // ladder, and not inside the writer: a power writer that reads the environment is the
        // hidden-state defect the bring-up contract removes (LAW 3). It becomes a
        // `BringUpRequest` field when this part gets a plan (M5).
        let idx = std::env::var("NDN_RADIO_TXPWR")
            .ok()
            .and_then(|s| s.parse::<u8>().ok())
            .unwrap_or(0x2d);
        self.tx_power_idx.store(idx.min(0x3f), Ordering::Relaxed);
        self.set_tx_power(idx)?;
        Ok(())
    }

    // ── TX/RX descriptors (tx.h / rx.h) ──────────────────────────────────────

    /// Build `[48-byte rtw88 TX descriptor][802.11 frame]` for `frame`, fixing
    /// the rate (USE_RATE + DISDATAFB + DATARATE) and routing to the MGMT queue.
    fn build_tx(
        &self,
        frame: &InjectFrame,
        mcs: crate::McsDescriptor,
    ) -> Result<Vec<u8>, FaceError> {
        let body = self.build_80211(frame)?;
        let mut buf = vec![0u8; TX_DESC_SIZE + body.len()];

        txdesc_set(&mut buf, 0, 0, 16, body.len() as u32); // W0 TXPKTSIZE
        txdesc_set(&mut buf, 0, 16, 8, TX_DESC_SIZE as u32); // W0 OFFSET
        let bmc = frame.dst[0] & 0x01;
        txdesc_set(&mut buf, 0, 24, 1, bmc as u32); // W0 BMC
        txdesc_set(&mut buf, 0, 26, 1, 1); // W0 LS (last segment)

        // Match the kernel's golden monitor-injection TX descriptor (decoded
        // from a usbmon capture of rtw88 injecting): MACID=0, QSEL=HIGH(17),
        // RATE_ID=6, SPE_RPT. Crucially QSEL=HIGH maps to the same bulk-OUT
        // endpoint we send to; QSEL=MGMT(18) routes to a *different* endpoint on
        // this 3-OUT-pipe dongle, so the frame was dequeued (FIFO drained) but
        // never keyed — the "TX keys but doesn't radiate" symptom.
        txdesc_set(&mut buf, 1, 0, 8, 0); // W1 MACID = 0
        // QSEL: golden scan probe-reqs (which radiate, unassociated) use MGMT(18),
        // not HIGH(17). MGMT is not firmware-gated (probe/auth must work pre-assoc).
        // NDN_RADIO_QSEL overrides for sweeping; default 18 = MGMT.
        let qsel: u32 = std::env::var("NDN_RADIO_QSEL")
            .ok()
            .and_then(|s| s.parse().ok())
            .unwrap_or(17);
        txdesc_set(&mut buf, 1, 8, 5, qsel); // W1 QSEL
        txdesc_set(&mut buf, 1, 16, 5, 6); // W1 RATE_ID = 6
        txdesc_set(&mut buf, 2, 19, 1, 1); // W2 SPE_RPT

        // `mcs` is the resolved rate — from the frame's intent (generic path) or
        // an exact rate (the `WifiRadio` path).
        //
        // ★ INTENT OVERRIDES THE STORED RATE (2026-09-01). `resolved_mcs` only consults the frame's
        // intent when `cur_mcs` is unset, so once the control plane named a rate, cooperative
        // reports and discovery — the traffic whose whole purpose is that the worst receiver
        // decodes it — went out at that throughput rate. Force the basic rate here, where the DESC
        // code is chosen, so it cannot be bypassed by a stored `cur_mcs`.
        let rate = if frame.tx.needs_basic_rate() {
            DESC_RATE_OFDM6M
        } else {
            rate_code(&mcs)
        };
        // The kernel injects with USE_RATE=0 (rate adaptation). Forcing a fixed
        // rate (USE_RATE+DISDATAFB) with no rate-table entry may be why TX didn't
        // key; default to the kernel style, `NDN_RADIO_FIXEDRATE=1` forces fixed.
        if std::env::var("NDN_RADIO_FIXEDRATE").is_ok() {
            txdesc_set(&mut buf, 3, 8, 1, 1); // W3 USE_RATE
            txdesc_set(&mut buf, 3, 10, 1, 1); // W3 DISDATAFB
        }
        txdesc_set(&mut buf, 4, 0, 7, rate as u32); // W4 DATARATE
        if mcs.short_gi {
            txdesc_set(&mut buf, 5, 4, 1, 1); // W5 DATA_SHORT (SGI)
        }
        if mcs.ldpc {
            txdesc_set(&mut buf, 5, 7, 1, 1); // W5 DATA_LDPC
        }
        let seq = self.seq.fetch_add(1, Ordering::Relaxed) & 0x0fff;
        txdesc_set(&mut buf, 8, 15, 1, 0); // W8 EN_HWSEQ off (we supply SEQ)
        txdesc_set(&mut buf, 9, 12, 12, seq as u32); // W9 SW_SEQ

        buf[TX_DESC_SIZE..].copy_from_slice(&body);
        txdesc_checksum(&mut buf);
        Ok(buf)
    }

    fn build_80211(&self, frame: &InjectFrame) -> Result<Vec<u8>, FaceError> {
        // TX-radiate investigation: rtw88/8821c firmware keys the PA for
        // *management* frames (probe/auth must work before association) but not
        // for host-injected *data* frames. NDN_RADIO_PROBE wraps the payload in a
        // probe-request with a vendor-specific IE so it rides the always-keyed
        // MGMT path. Mirrors the golden scan probe-req that we saw radiate.
        if std::env::var("NDN_RADIO_PROBE").is_ok() {
            return Ok(self.build_probe_req(frame));
        }
        // ☠ This arm used to hand-roll its own 3-address frame for `RawNdn`, reading neither
        // `frame.extra` nor `frame.htc` — so an 8821CU could not emit a filter frame no matter what
        // the face above it decided. One builder, `frame::build_dot11`, now owns the layout; this
        // backend only stamps what is genuinely its own (the hardware sequence number, and the IBSS
        // BSSID the firmware/peer matches on). SeqCtrl and addr3 are at fixed offsets 22..24 and
        // 16..22 in every 802.11 header shape, so both stamps are layout-independent.
        let mut out = frame::build_dot11(self.format, frame)?;
        if matches!(self.format, FrameFormat::RawNdn { .. }) && out.len() >= 24 {
            let seq = self.seq.fetch_add(1, Ordering::Relaxed) & 0x0fff;
            out[22..24].copy_from_slice(&(seq << 4).to_le_bytes());
            // ★ Was `std::env::var("NDN_RADIO_IBSS")`, on the per-frame path. M5: the flag is set
            // by the `ibss_opmode` rung, so the frame layout and the MAC's operating mode are
            // decided by ONE thing — the plan that ran — instead of by two independent reads of
            // the same environment variable that could disagree.
            if self.ibss_mode.load(Ordering::Relaxed) {
                out[16..22].copy_from_slice(&IBSS_BSSID); // addr3 = BSSID
            }
        }
        Ok(out)
    }

    /// Build an 802.11 probe-request (mgmt, subtype 4) carrying `frame.payload`
    /// in a vendor-specific IE. Used to test/exploit the firmware's always-keyed
    /// MGMT TX path. Layout mirrors the golden radiating scan probe: broadcast
    /// addrs, wildcard SSID, supported-rates, DS-param(channel), then the vendor
    /// IE (0xdd, OUI = our DEFAULT_SRC[0..3]).
    fn build_probe_req(&self, frame: &InjectFrame) -> Vec<u8> {
        let seq = self.seq.fetch_add(1, Ordering::Relaxed) & 0x0fff;
        let ch = self.cur_channel.load(Ordering::Relaxed);
        let mut out = Vec::with_capacity(40 + frame.payload.len());
        out.extend_from_slice(&[0x40, 0x00, 0x00, 0x00]); // FC=probe-req + duration
        out.extend_from_slice(&[0xff; 6]); // addr1 broadcast
        out.extend_from_slice(&frame.src); // addr2 = our MAC
        out.extend_from_slice(&[0xff; 6]); // addr3 broadcast (wildcard BSSID)
        out.extend_from_slice(&(seq << 4).to_le_bytes());
        // IEs:
        out.extend_from_slice(&[0x00, 0x00]); // SSID (wildcard, len 0)
        out.extend_from_slice(&[0x01, 0x08, 0x02, 0x04, 0x0b, 0x16, 0x0c, 0x12, 0x18, 0x24]); // rates
        out.extend_from_slice(&[0x03, 0x01, ch.max(1)]); // DS param: current channel
        // Vendor-specific IE carrying the NDN payload (chunked to <=255).
        let oui = [0x02u8, 0x4e, 0x44]; // DEFAULT_SRC[0..3]
        for chunk in frame.payload.chunks(252) {
            out.push(0xdd);
            out.push((chunk.len() + 3) as u8);
            out.extend_from_slice(&oui);
            out.extend_from_slice(chunk);
        }
        out
    }

    /// Parse one RX unit at `off`; returns the decoded frames (0 or 1) and the
    /// 8-byte-aligned stride to the next unit, or `None` if `off` is past the end.
    fn parse_rx_at(&self, buf: &[u8], off: usize) -> Option<(Vec<CapturedFrame>, usize)> {
        if off + RX_DESC_SIZE > buf.len() {
            return None;
        }
        let d = &buf[off..];
        let w0 = u32::from_le_bytes([d[0], d[1], d[2], d[3]]);
        let pkt_len = (w0 & 0x3fff) as usize;
        let crc_err = (w0 >> 14) & 1 != 0;
        let drv_info = ((w0 >> 16) & 0xf) as usize * 8;
        let shift = ((w0 >> 24) & 0x3) as usize;
        let physt = (w0 >> 26) & 1 != 0;
        let w2 = u32::from_le_bytes([d[8], d[9], d[10], d[11]]);
        let is_c2h = (w2 >> 28) & 1 != 0;
        let w3 = u32::from_le_bytes([d[12], d[13], d[14], d[15]]);
        let rate = (w3 & 0x7f) as u8;

        let hdr_off = RX_DESC_SIZE + drv_info + shift;
        let stride = (hdr_off + pkt_len).div_ceil(8) * 8;
        self.rx_raw_count.fetch_add(1, Ordering::Relaxed);

        // Raw RX diagnostic: every captured 802.11 unit, before NDN filtering.
        {
            use std::sync::OnceLock;
            static DBG: OnceLock<bool> = OnceLock::new();
            if *DBG.get_or_init(|| std::env::var("NDN_RADIO_RX_DEBUG").is_ok()) {
                eprintln!(
                    "RX unit @{off}: len={pkt_len} rate={rate:#04x} drv_info={drv_info} shift={shift} crc_err={crc_err} c2h={is_c2h} physt={physt}"
                );
            }
        }
        if pkt_len == 0 || off + hdr_off + pkt_len > buf.len() || is_c2h || crc_err {
            return Some((Vec::new(), stride.max(8)));
        }

        // RSSI from phy_status page 1 (OFDM/HT/VHT): PWDB_A(dword0[15:8]) - 110.
        let rssi = if physt && off + RX_DESC_SIZE + shift + 4 <= buf.len() {
            let ps = &buf[off + RX_DESC_SIZE + shift..];
            let page = ps[0] & 0xf;
            if page != 0 {
                let pwdb_a = ps[1]; // dword0 bits [15:8]
                Some((pwdb_a as i16 - 110).clamp(-120, 0) as i8)
            } else {
                None
            }
        } else {
            None
        };

        let body = &buf[off + hdr_off..off + hdr_off + pkt_len];
        // Strip the trailing FCS (4 bytes) that monitor RX appends.
        let body = if body.len() >= 4 {
            &body[..body.len() - 4]
        } else {
            body
        };
        // #41: per-frame hardware RX timestamp — RX descriptor dword5 (bytes 20-23) is the free-run
        // RX TSF-low latched at MAC-done (RXTSFL, µs). Same 88xx layout as the 8733b/8812au backends.
        let rxtsfl = u32::from_le_bytes([d[20], d[21], d[22], d[23]]);
        let stamp = Some(realtek_rx::rx_stamp(rxtsfl, self.tsf_domain));
        // DESC_RATE code -> MCS index (HT/VHT) or None for legacy; the shared Realtek
        // decode the sibling 88xx/8812au backends use. Passing the raw rate here would
        // hand every mcs_index consumer the Realtek rate enum, not an MCS index.
        let mcs_index = realtek_rx::mcs_from_desc_rate(rate);
        let decoded = frame::parse_dot11(self.format, body, rssi, mcs_index, stamp)
            .into_iter()
            .collect();
        Some((decoded, stride.max(8)))
    }
}

// ── free helpers ─────────────────────────────────────────────────────────────

/// rtw88 rate enum (`DESC_RATE*`) for an [`McsDescriptor`].
fn rate_code(mcs: &crate::McsDescriptor) -> u8 {
    if let Ok(s) = std::env::var("NDN_RADIO_TX_RATE")
        && let Ok(v) = s.parse::<u8>()
    {
        return v;
    }
    if mcs.vht {
        DESC_RATE_VHT1SS_MCS0 + mcs.index
    } else if mcs.index == 0 && !mcs.vht {
        // default conservative: 6 Mbps OFDM rather than CCK
        DESC_RATE_MCS0 + mcs.index
    } else {
        DESC_RATE_MCS0 + mcs.index
    }
    .max(DESC_RATE_OFDM6M)
}

/// Set a big-endian-dword-relative bitfield in the little-endian descriptor:
/// `word` is the dword index, `bit`/`len` the field within that dword.
fn txdesc_set(desc: &mut [u8], word: usize, bit: u32, len: u32, value: u32) {
    let off = word * 4;
    let mut v = u32::from_le_bytes([desc[off], desc[off + 1], desc[off + 2], desc[off + 3]]);
    let mask = if len >= 32 {
        u32::MAX
    } else {
        ((1u32 << len) - 1) << bit
    };
    v = (v & !mask) | ((value << bit) & mask);
    desc[off..off + 4].copy_from_slice(&v.to_le_bytes());
}

/// rtw88 TX-descriptor checksum (tx.h:124): zero W7[0:15], XOR the descriptor as
/// `__le16` words, store into W7[0:15].
fn txdesc_checksum(desc: &mut [u8]) {
    txdesc_set(desc, 7, 0, 16, 0);
    let mut sum: u16 = 0;
    let words = TX_DESC_SIZE / 2;
    for i in 0..words {
        sum ^= u16::from_le_bytes([desc[i * 2], desc[i * 2 + 1]]);
    }
    txdesc_set(desc, 7, 0, 16, sum as u32);
}

fn log_write(addr: u16, data: &[u8]) {
    use std::sync::OnceLock;
    static LOG: OnceLock<bool> = OnceLock::new();
    if *LOG.get_or_init(|| std::env::var("NDN_RADIO_LOG_WRITES").is_ok()) {
        let mut v = 0u32;
        for (i, b) in data.iter().enumerate().take(4) {
            v |= (*b as u32) << (i * 8);
        }
        eprintln!(
            "W{}\t0x{:04x}\t0x{:0w$x}",
            data.len(),
            addr,
            v,
            w = data.len() * 2
        );
    }
}

/// Which write width / sentinel handling a phy_cond table uses.
#[derive(Clone, Copy)]
enum CfgKind {
    Mac,
    Agc,
    Bb,
    Rf,
}

/// Decode a phy_cond directive word (main.h struct rtw_phy_cond, little-endian).
fn parse_phy_cond(w: u32) -> PhyCond {
    PhyCond {
        rfe: (w & 0xff) as u8,
        intf: ((w >> 8) & 0xf) as u8,
        pkg: ((w >> 12) & 0xf) as u8,
        cut: ((w >> 24) & 0xf) as u8,
    }
}

/// rtw88 `check_positive` for 8821C (phy.c): cut/pkg/intf match if the directive
/// specifies them (nonzero); rfe must match exactly.
fn check_positive(cond: PhyCond, drv: PhyCond) -> bool {
    if cond.cut != 0 && cond.cut != drv.cut {
        return false;
    }
    if cond.pkg != 0 && cond.pkg != drv.pkg {
        return false;
    }
    if cond.intf != 0 && cond.intf != drv.intf {
        return false;
    }
    cond.rfe == drv.rfe
}

impl Rtl8821cuBackend {
    /// Write one descriptor+frame to the radiating OUT pipe, erroring on a short
    /// write. Shared by the generic and exact-rate inject paths.
    async fn send(&self, buf: Vec<u8>) -> Result<(), FaceError> {
        let handle = self.handle.clone();
        // Endpoint selection for the TX-radiate investigation: NDN_RADIO_EP picks
        // an OUT pipe either by index (0,1,2…) or by raw address (e.g. 0x05).
        // Default: the MGMT pipe rtw88 uses for scan probes — the *last* OUT
        // endpoint on the 8821cu (high-priority/MGMT), which is the radiating one.
        let ep = match std::env::var("NDN_RADIO_EP").ok().and_then(|s| {
            s.strip_prefix("0x")
                .and_then(|h| u8::from_str_radix(h, 16).ok())
                .or_else(|| s.parse::<u8>().ok())
        }) {
            Some(v) if self.bulk_outs.contains(&v) => v,
            Some(idx) if (idx as usize) < self.bulk_outs.len() => self.bulk_outs[idx as usize],
            _ => self.bulk_out, // default: first OUT pipe (matches kernel injection)
        };
        tokio::task::spawn_blocking(move || {
            handle
                .write_bulk(ep, &buf, Duration::from_secs(1))
                .map_err(usb_err)
                .and_then(|n| {
                    (n == buf.len())
                        .then_some(())
                        .ok_or_else(|| init_err(format!("8821cu inject short {n}/{}", buf.len())))
                })
        })
        .await
        .map_err(|e| init_err(format!("8821cu inject join {e}")))?
    }
}

#[async_trait]
impl FrameIo for Rtl8821cuBackend {
    /// This radio's own capability, so a face built from the bare `dyn FrameIo` does not have to
    /// invent one. Delegates to this type's [`RadioProfile`] — the single source of truth.
    fn radio_capability(&self) -> Option<ndn_radio_hal::RadioCapability> {
        Some(<Self as ndn_radio_hal::RadioProfile>::capability(self))
    }
    async fn inject(&self, frame: InjectFrame) -> Result<(), FaceError> {
        let mcs = self.resolved_mcs(&frame);
        let buf = self.build_tx(&frame, mcs)?;
        self.send(buf).await
    }

    /// Rate as bearer state: store the exact MCS every subsequent `inject` uses.
    fn set_rate(&self, mcs: crate::McsDescriptor) -> Result<(), FaceError> {
        *self.cur_mcs.lock().unwrap() = Some(mcs);
        Ok(())
    }

    async fn recv_frame(&self) -> Result<CapturedFrame, FaceError> {
        // Pumped mode: background threads fill rx_pending; just drain it.
        if self.rx_pumped.load(Ordering::Relaxed) {
            loop {
                let notified = self.rx_notify.notified();
                if let Some(f) = self.rx_pending.lock().unwrap().pop_front() {
                    return Ok(f);
                }
                let _ = tokio::time::timeout(Duration::from_millis(200), notified).await;
            }
        }
        loop {
            if let Some(f) = self.rx_pending.lock().unwrap().pop_front() {
                return Ok(f);
            }
            let handle = self.handle.clone();
            let ep = self.bulk_in;
            let buf = tokio::task::spawn_blocking(move || {
                let mut buf = vec![0u8; 32768];
                match handle.read_bulk(ep, &mut buf, Duration::from_millis(200)) {
                    Ok(n) => {
                        buf.truncate(n);
                        Ok(Some(buf))
                    }
                    Err(rusb::Error::Timeout) => Ok(None),
                    Err(e) => Err(usb_err(e)),
                }
            })
            .await
            .map_err(|e| init_err(format!("8821cu recv join {e}")))??;

            if let Some(buf) = buf {
                {
                    use std::sync::OnceLock;
                    static DBG: OnceLock<bool> = OnceLock::new();
                    if *DBG.get_or_init(|| std::env::var("NDN_RADIO_RX_DEBUG").is_ok()) {
                        eprintln!("bulk-IN transfer: {} bytes", buf.len());
                    }
                }
                let mut off = 0;
                let mut q = self.rx_pending.lock().unwrap();
                while let Some((decoded, advance)) = self.parse_rx_at(&buf, off) {
                    for f in decoded {
                        q.push_back(f);
                    }
                    off += advance;
                    if off + RX_DESC_SIZE > buf.len() {
                        break;
                    }
                }
            }
        }
    }
}

// Marker only: `inject_at` is the derived HAL default (`set_rate` + `inject`).
/// **What this radio is** (#79/#83) — declared, so a planner stops believing the caller's guess.
///
/// The module header states it: RTL8821CU, **1×1 dual-band 802.11ac**. `wifi_monitor_5ghz_1ss`
/// already encodes exactly that shape (1 stream, VHT MCS0-8, up to 80 MHz); only the band list needs
/// widening, since this part does 2.4 GHz too.
///
/// Deliberately narrow: `RadioKnobs` and `RadioTime` stay unimplemented. This backend's bring-up is
/// staged and explicitly incomplete (the module header lists `TODO(hw)` gaps against the golden
/// trace), **and no 8821c is currently attached to either OPi** — `lsusb` on both shows none. A
/// knob that cannot be exercised on hardware is an assertion, and this codebase has enough of those;
/// capability is a static fact about the part, which is why it is safe to declare from the datasheet
/// while a control knob is not.
impl Rtl8821cuBackend {
    /// The part's capability as a free function — see [`Mt7612uBackend::declared_capability`] for
    /// why this is not behind `&self`. It matters more here: no 8821c is attached to either OPi, so
    /// a hardware-only assertion would be entirely unchecked.
    pub fn declared_capability() -> RadioCapability {
        RadioCapability {
            bands: vec![Band::Band2_4GHz, Band::Band5GHz],
            ..RadioCapability::wifi_monitor_5ghz_1ss(vec![
                1, 6, 11, 36, 40, 44, 48, 149, 153, 157, 161,
            ])
        }
    }
}

impl RadioProfile for Rtl8821cuBackend {
    fn capability(&self) -> RadioCapability {
        Self::declared_capability()
    }
}

impl Rtl8821cuBackend {
    /// The rate to transmit `frame` at: the control-plane-set MCS (state) if present,
    /// else the frame's intent resolved to this radio.
    fn resolved_mcs(&self, frame: &InjectFrame) -> crate::McsDescriptor {
        self.cur_mcs.lock().unwrap().unwrap_or_else(|| {
            crate::McsDescriptor::for_intent(&frame.tx, crate::MAX_RELIABLE_MCS, true, false)
        })
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// M5 · §1.4 — THE PLAN, and the four TX-radiate variants.
// ─────────────────────────────────────────────────────────────────────────────
//
// Specification: `docs/bringup-contract.md` §1.4/§1.5/§4/§5-M5.
//
// ★ **Every rung below is transcribed VERBATIM, IN ORDER, from `bring_up`.** Not one register
// write moved, was added, or was reordered.
//
// ★★ **The four never-scored TX-radiate theories become four named plan variants.** They were
// four `if std::env::var("NDN_RADIO_*").is_ok()` blocks inside the ladder — which is LAW 1's
// defect exactly: hidden state deciding what the bring-up meant, with two runs that look identical
// in their own output. Worse, they are MUTUALLY EXCLUSIVE explanations of the same symptom and
// nothing stopped an operator setting two at once and getting a sequence no one had ever reasoned
// about. As variants they are five distinct `PlanId`s and therefore five distinct `plan_digest`s,
// exactly one runs, and any number taken under one carries its own asterisk forever.
//
// ⚠ **None of them is promoted or deleted here, and that is deliberate.** §5-M5 says they are to
// be "scored once against a witness in one bench session, then promoted or deleted"; there is no
// hardware at this keyboard, and choosing between four unmeasured hypotheses by reading them is
// the reasoning-about-radios this contract exists to stop. Each variant's `why` says it is an
// untested hypothesis awaiting that session.
//
// One thing about the shape, said plainly: three of the four are ADDITIVE (they add a rung to the
// canonical sequence) and §1.4 forbids a `Deviation` from adding steps — a deviation may only
// subtract. That is why they are variants rather than deviations. The fourth, `NoTxen`, is
// genuinely subtractive and could have been a `Deviation::skip`; it is a variant anyway so that
// the four hypotheses are read, run and reported the same way as each other.

/// Shorthand for the step tables below.
type Rtl8821c = Rtl8821cuBackend;

/// **Which of the five 8821CU bring-up sequences to run.**
///
/// ☠ **This part has never been observed to radiate**, and the four non-canonical variants are the
/// four mutually exclusive explanations that were proposed for it. Every one of them is an
/// UNTESTED HYPOTHESIS: none has ever been scored against a witness receiver, and the ladder's own
/// comments already disagree with each other about which is "the prime" one. Running one and
/// seeing frames does not settle anything on its own — the acceptance is a witness at the far end
/// counting decoded frames, and one bench session can score all four.
///
/// The variants are mutually exclusive on purpose. As environment flags they were not: setting two
/// produced a sequence nobody had reasoned about, and the report could not tell you which run you
/// were looking at.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Default)]
pub enum Rtl8821cVariant {
    /// The sequence this part has always run by default. Includes the `txen_golden_block` rung
    /// (which was on unless `NDN_RADIO_NO_TXEN` was set) and none of the other three.
    #[default]
    Canonical,
    /// `NDN_RADIO_STA` — firmware station-emulation at MACID 0.
    FwStaEmulate,
    /// `NDN_RADIO_NO_TXEN` — canonical MINUS the golden finalize/function-enable block.
    NoTxen,
    /// `NDN_RADIO_STAREGS` — the station-identity registers the kernel writes before it transmits.
    StationRegs,
    /// `NDN_RADIO_IBSS` — ad-hoc operating mode instead of monitor's NO_LINK.
    Ibss,
}

impl Rtl8821cVariant {
    /// LAW 1 at the wrapper boundary: the four `NDN_RADIO_*` flags are read HERE, once, and never
    /// inside a rung. §1.1's `BringUpRequest::from_env` is where this belongs for good.
    ///
    /// ⚠ Two or more set at once is **refused loudly** rather than resolved by precedence. They
    /// are mutually exclusive hypotheses; silently running one of them would produce a number
    /// attributed to the wrong experiment, which is the whole failure this contract addresses. The
    /// `ndn_env` `Unrecognised` lesson applied to a conflict instead of a typo.
    pub fn from_env() -> Self {
        let set: Vec<(&str, Self)> = [
            ("NDN_RADIO_STA", Self::FwStaEmulate),
            ("NDN_RADIO_NO_TXEN", Self::NoTxen),
            ("NDN_RADIO_STAREGS", Self::StationRegs),
            ("NDN_RADIO_IBSS", Self::Ibss),
        ]
        .into_iter()
        .filter(|(k, _)| std::env::var_os(k).is_some())
        .collect();
        match set.as_slice() {
            [] => Self::Canonical,
            [(_, v)] => *v,
            many => {
                let names: Vec<&str> = many.iter().map(|(k, _)| *k).collect();
                let msg = format!(
                    "8821cu: {} are set together and they are MUTUALLY EXCLUSIVE TX-radiate \
                     hypotheses, not composable flags. Running CANONICALLY. Pick one: a number \
                     measured under a mixture of two untested hypotheses cannot be attributed to \
                     either.",
                    names.join(" + ")
                );
                tracing::warn!(target: "radio", part = "rtl8821cu", "{msg}");
                eprintln!("{msg}");
                Self::Canonical
            }
        }
    }

    /// The plan this variant runs.
    pub fn plan(self) -> &'static Plan<Rtl8821c> {
        match self {
            Self::Canonical => &PLAN_8821CU_MONITOR,
            Self::FwStaEmulate => &PLAN_8821CU_FW_STA,
            Self::NoTxen => &PLAN_8821CU_NO_TXEN,
            Self::StationRegs => &PLAN_8821CU_STATION_REGS,
            Self::Ibss => &PLAN_8821CU_IBSS,
        }
    }
}

// ── the rungs ────────────────────────────────────────────────────────────────

fn s_read_cut_version(b: &Arc<Rtl8821c>, _c: &mut Ctx<'_>) -> Result<StepOutcome, FaceError> {
    let cut = ((b.read32(REG_SYS_CFG1)? >> 12) & 0xf) as u8;
    *b.cond.lock().unwrap() = PhyCond {
        cut,
        pkg: 0,             // TODO(hw): efuse package type (rtw8821c.h pkg_type)
        intf: INTF_USB_PHY, // INTF_USB = 2
        rfe: 0,             // TODO(hw): efuse rfe_option_full >> 3
    };
    Ok(StepOutcome::Done)
}

fn s_pre_system_cfg(b: &Arc<Rtl8821c>, _c: &mut Ctx<'_>) -> Result<StepOutcome, FaceError> {
    b.pre_system_cfg()?;
    Ok(StepOutcome::Done)
}

fn s_power_cycle_if_on(b: &Arc<Rtl8821c>, c: &mut Ctx<'_>) -> Result<StepOutcome, FaceError> {
    if b.read8(REG_CR)? == 0xea {
        return Ok(StepOutcome::Skipped(
            "REG_CR reads the 0xea card-disabled signature — the chip is already down, so there \
             is nothing to power-cycle",
        ));
    }
    // Best-effort by transcription: a stale-state poll timeout here must not abort bring-up. The
    // `let _ =` is the ladder's own; what changed is that the loss is now stated.
    if let Err(e) = b.apply_pwr_flow(tables::CARD_DISABLE_FLOW_8821C, b.cond.lock().unwrap().cut) {
        c.warn(format!(
            "the card-disable flow did not complete ({e}); CARD_ENABLE is about to run on a chip \
             in an unknown power state, which is the case that wedges it"
        ));
    }
    b.write8(REG_RSV_CTRL, 0)?;
    Ok(StepOutcome::Branch("was-powered-on"))
}

fn s_card_enable(b: &Arc<Rtl8821c>, _c: &mut Ctx<'_>) -> Result<StepOutcome, FaceError> {
    let cut = b.cond.lock().unwrap().cut;
    b.apply_pwr_flow(tables::CARD_ENABLE_FLOW_8821C, cut)?;
    Ok(StepOutcome::Done)
}

fn s_read_chip_info(b: &Arc<Rtl8821c>, _c: &mut Ctx<'_>) -> Result<StepOutcome, FaceError> {
    let info = b.read_chip_info()?;
    tracing::info!(
        "8821cu efuse: rfe_option={:#04x} (full={:#04x}) pkg={} btg={}",
        info.rfe_option,
        info.rfe_option_full,
        info.pkg_type,
        info.rfe_btg
    );
    b.rfe_option.store(info.rfe_option_full, Ordering::Relaxed);
    b.rfe_btg.store(info.rfe_btg, Ordering::Relaxed);
    let mut c = b.cond.lock().unwrap();
    c.rfe = info.rfe_option_full >> 3;
    c.pkg = info.pkg_type;
    Ok(StepOutcome::Done)
}

fn s_download_firmware(b: &Arc<Rtl8821c>, _c: &mut Ctx<'_>) -> Result<StepOutcome, FaceError> {
    b.download_firmware()?;
    Ok(StepOutcome::Done)
}

fn s_mac_init(b: &Arc<Rtl8821c>, _c: &mut Ctx<'_>) -> Result<StepOutcome, FaceError> {
    b.mac_init()?;
    Ok(StepOutcome::Done)
}

fn s_phy_set_param(b: &Arc<Rtl8821c>, _c: &mut Ctx<'_>) -> Result<StepOutcome, FaceError> {
    b.phy_set_param()?;
    Ok(StepOutcome::Done)
}

fn s_send_fw_info(b: &Arc<Rtl8821c>, _c: &mut Ctx<'_>) -> Result<StepOutcome, FaceError> {
    b.send_fw_info()?;
    Ok(StepOutcome::Done)
}

fn s_fw_sta_emulate(b: &Arc<Rtl8821c>, c: &mut Ctx<'_>) -> Result<StepOutcome, FaceError> {
    if let Err(e) = b.media_status_report(0, true) {
        c.warn(format!("media-status(connect) failed: {e}"));
    }
    // Rate table for MACID 0 (OFDM 6-54 + MCS0-7). Required for the firmware RA to accept frames
    // from this station when USE_RATE=0.
    if let Err(e) = b.ra_info(0, 6, 0x000f_fff0) {
        c.warn(format!("ra_info failed: {e}"));
    }
    Ok(StepOutcome::Done)
}

fn s_coex_grant_wl(b: &Arc<Rtl8821c>, _c: &mut Ctx<'_>) -> Result<StepOutcome, FaceError> {
    b.coex_init_wl_only()?;
    Ok(StepOutcome::Done)
}

fn s_monitor_rx(b: &Arc<Rtl8821c>, _c: &mut Ctx<'_>) -> Result<StepOutcome, FaceError> {
    b.set_monitor_rx()?;
    Ok(StepOutcome::Done)
}

fn s_tune_channel(b: &Arc<Rtl8821c>, c: &mut Ctx<'_>) -> Result<StepOutcome, FaceError> {
    let ch = c.state_ref().channel;
    b.set_channel(ch)?;
    // LAW 6 — this rung wrote the per-rate TXAGC block, so the report says what is in force. It
    // is a driver reference, not a fused base: nothing here reads a per-adapter power fuse.
    let idx = b.tx_power_idx.load(Ordering::Relaxed);
    c.state().power = AppliedPower::from_writes(
        PowerRequest::index(idx),
        PowerReference::DriverReference {
            source: "8821c per-rate TXAGC block 0x1d00 (uniform index, no fuse read; \
                     NDN_RADIO_TXPWR overrides at set_channel)",
            slope_db_per_idx: None,
        },
        idx,
        false,
        vec![PowerWrite {
            reg: 0x1d00,
            value: idx,
            group: "per-rate TXAGC (uniform)",
            path: 0,
        }],
    );
    Ok(StepOutcome::Done)
}

fn s_bb_rx_path_enable(b: &Arc<Rtl8821c>, _c: &mut Ctx<'_>) -> Result<StepOutcome, FaceError> {
    b.set32(0x0808, 1 << 29)?;
    Ok(StepOutcome::Done)
}

fn s_iqk(b: &Arc<Rtl8821c>, _c: &mut Ctx<'_>) -> Result<StepOutcome, FaceError> {
    b.do_iqk()?;
    Ok(StepOutcome::Done)
}

fn s_txen_golden_block(b: &Arc<Rtl8821c>, _c: &mut Ctx<'_>) -> Result<StepOutcome, FaceError> {
    b.write32(0x042c, 0x4000_c000)?;
    b.write8(0x045f, 0x10)?;
    b.write32(0x06dc, 0x0484_0000)?;
    b.write32(0x1c94, 0xafff_afff)?;
    b.write8(0x0002, 0x1f)?; // SYS_FUNC_EN: enable all function blocks
    Ok(StepOutcome::Done)
}

fn s_station_identity_regs(b: &Arc<Rtl8821c>, _c: &mut Ctx<'_>) -> Result<StepOutcome, FaceError> {
    let mac = ndn_frame_io::frame::DEFAULT_SRC; // 02:4e:44:4e:00:01
    b.write32(0x0610, u32::from_le_bytes([mac[0], mac[1], mac[2], mac[3]]))?;
    b.write16(0x0614, u16::from_le_bytes([mac[4], mac[5]]))?;
    b.write8(0x0440, 0x5d)?; // TX control (scan golden)
    b.write8(0x0093, 0xd4)?;
    b.write8(0x0007, 0x20)?;
    b.write8(0x0550, 0x08)?;
    Ok(StepOutcome::Done)
}

fn s_ibss_opmode(b: &Arc<Rtl8821c>, _c: &mut Ctx<'_>) -> Result<StepOutcome, FaceError> {
    b.setup_ibss()?;
    // ★ The frame builder's addr3 and the MAC's operating mode are now decided by ONE thing: the
    // plan that ran. They used to be two independent reads of `NDN_RADIO_IBSS`, one of them on the
    // per-frame path.
    b.ibss_mode.store(true, Ordering::Relaxed);
    Ok(StepOutcome::Done)
}

fn s_hci_usb_cfg(b: &Arc<Rtl8821c>, _c: &mut Ctx<'_>) -> Result<StepOutcome, FaceError> {
    b.hci_usb_cfg()?;
    Ok(StepOutcome::Done)
}

// ── the rungs, as reviewable constants ───────────────────────────────────────

const R_READ_CUT_VERSION: Step<Rtl8821c> = Step {
    id: StepId("read_cut_version"),
    stage: Stage::Attach,
    class: StepClass::Required,
    why: "The cut version out of REG_SYS_CFG1[15:12] plus the efuse-derived phy condition \
          (rfe/pkg/intf), which is what selects the rows of every MAC/BB/AGC/RF table loaded \
          later — `apply_phy_table` matches each row against this. Loading another cut's tables \
          mis-tunes the receiver in a way that reads as a quiet channel. ⚠ `pkg` and `rfe` are \
          placeholders here and are corrected by `read_chip_info` once the chip is powered.",
    must_follow: &[],
    must_precede: &[StepId("phy_set_param")],
    run: s_read_cut_version,
};

const R_PRE_SYSTEM_CFG: Step<Rtl8821c> = Step {
    id: StepId("pre_system_cfg"),
    stage: Stage::PowerOn,
    class: StepClass::Required,
    why: "`rtw_mac_pre_system_cfg` — it configures the RF front-end pin mux (PAPE/LNAON routing) \
          and DISABLES BB/RF before the power flow. The front-end routing is what the receiver \
          needs; the BB/RF disable is what `phy_set_param` later undoes, and getting that pair out \
          of order left RX dead (see `phy_set_param`).",
    must_follow: &[],
    must_precede: &[StepId("card_enable")],
    run: s_pre_system_cfg,
};

const R_POWER_CYCLE_IF_ON: Step<Rtl8821c> = Step {
    id: StepId("power_cycle_if_on"),
    stage: Stage::PowerOn,
    class: StepClass::Required,
    why: "Re-run safety: if the chip is NOT showing the 0xea card-disabled signature at REG_CR it \
          is already powered on, and running CARD_ENABLE on an already-on chip WEDGES it — rtw88 \
          power-cycles in the same case. So it is taken down first. The branch is the plan's own \
          judgement from a register it reads, which is why it is one rung and not a caller's \
          choice.",
    must_follow: &[StepId("pre_system_cfg")],
    must_precede: &[StepId("card_enable")],
    run: s_power_cycle_if_on,
};

const R_CARD_ENABLE: Step<Rtl8821c> = Step {
    id: StepId("card_enable"),
    stage: Stage::PowerOn,
    class: StepClass::Required,
    why: "The halmac CARD_ENABLE power flow. Until it completes the MAC is card-disabled: register \
          writes are accepted and dropped, so every later readback is fiction. **LAW 5**: \
          `apply_pwr_flow` polls each pwrseq entry to its wanted value and returns Err on timeout.",
    must_follow: &[StepId("pre_system_cfg")],
    must_precede: &[StepId("download_firmware")],
    run: s_card_enable,
};

const R_READ_CHIP_INFO: Step<Rtl8821c> = Step {
    id: StepId("read_chip_info"),
    stage: Stage::Attach,
    class: StepClass::BestEffort(Degradation::new(
        "the board's real RFE profile is unknown, so the 0xcb4 front-end value, the BTG AGC table \
         and the antenna routing all fall back to another board's assumptions — the receiver is \
         mis-tuned in a way that presents as a quiet channel",
        "register-level and power-sequence work; NOT an RSSI, a range or a delivery number",
    )),
    why: "The RFE profile out of the efuse. Hardcoding another board's profile mis-tunes the \
          receiver, which is why this exists at all; best effort with a named loss, transcribed \
          from the ladder's `Err(e) => tracing::warn!(\"...using default RFE profile\")`. It must \
          run after `card_enable` (the efuse is not readable on a card-disabled chip) and before \
          `phy_set_param`, which is what consumes `rfe_option`/`rfe_btg`.",
    must_follow: &[StepId("card_enable")],
    must_precede: &[StepId("phy_set_param")],
    run: s_read_chip_info,
};

const R_DOWNLOAD_FIRMWARE: Step<Rtl8821c> = Step {
    id: StepId("download_firmware"),
    stage: Stage::Firmware,
    class: StepClass::Required,
    why: "The DDMA reserved-page firmware download. Required for the firmware-offloaded IQK and \
          for TX; monitor RX alone does not need it, but this plan is `TransmitAndReceive` and the \
          `send_fw_info` / `do_iqk` / station-emulation rungs all speak to the MAC CPU. **LAW 5**: \
          it polls the DDMA completion and the fw-ready state.",
    must_follow: &[StepId("card_enable")],
    must_precede: &[StepId("mac_init")],
    run: s_download_firmware,
};

const R_MAC_INIT: Step<Rtl8821c> = Step {
    id: StepId("mac_init"),
    stage: Stage::MacInit,
    class: StepClass::Required,
    why: "TRX FIFO/queue configuration + the chip MAC register group + the H2C ring. It takes the \
          MAC out of the download-mode page layout `download_firmware` left it in, so the data \
          path can run at all. ⚠ It also installs an RCR that `monitor_rx` later overrides — that \
          ordering is transcribed, not incidental.",
    must_follow: &[StepId("download_firmware")],
    must_precede: &[StepId("monitor_rx")],
    run: s_mac_init,
};

const R_PHY_SET_PARAM: Step<Rtl8821c> = Step {
    id: StepId("phy_set_param"),
    stage: Stage::PhyInit,
    class: StepClass::Required,
    why: "★ `rtw8821c_phy_set_param` — re-enable the BB/RF domain that `pre_system_cfg` disabled, \
          THEN load the MAC/BB/AGC/RF tables. **This was the bug that left RX dead**: without the \
          re-enable the BB/RF stay powered off, RF registers read back garbage and the receiver \
          never delivers a frame, while every other stage looks healthy. Everything above the PHY \
          — the tune, the IQK, the TX datapath — is addressed through what this writes.",
    must_follow: &[StepId("mac_init"), StepId("read_cut_version")],
    must_precede: &[StepId("tune_channel")],
    run: s_phy_set_param,
};

const R_SEND_FW_INFO: Step<Rtl8821c> = Step {
    id: StepId("send_fw_info"),
    stage: Stage::Firmware,
    class: StepClass::BestEffort(Degradation::new(
        "the firmware has no general_info/phydm_info, so it does not run the dynamic RX gain \
         (DIG) — the receiver sits at its table default gain instead of tracking the channel",
        "a strong, stable link; NOT a sensitivity, range or weak-signal delivery number",
    )),
    why: "The `rtw_power_on` tail H2C pair (general_info + phydm_info), which is what makes the \
          firmware run DIG. Best effort with a named loss, transcribed from the ladder's \
          `eprintln!(\"8821cu fw-info H2C failed\")`.",
    must_follow: &[StepId("phy_set_param")],
    must_precede: &[],
    run: s_send_fw_info,
};

/// ☠ VARIANT RUNG — [`PLAN_8821CU_FW_STA`] only.
const R_FW_STA_EMULATE: Step<Rtl8821c> = Step {
    id: StepId("fw_sta_emulate"),
    stage: Stage::TxEnable,
    class: StepClass::BestEffort(Degradation::new(
        "the station emulation this variant exists to test did not install, so the run is not the \
         experiment it claims to be",
        "nothing this variant was run for — take the result as a canonical run with noise, and \
         re-run",
    )),
    why: "☠ **UNTESTED HYPOTHESIS #1, awaiting one bench session against a witness.** Emulate a \
          connected station at MACID 0 (the MACID `build_tx` uses) so the firmware keys the PA for \
          injected frames: on rtw88 the firmware gates PA-keying on a media-status 'connected' \
          report, and without one the chip dequeues TX frames but never radiates them. The \
          supporting observation is that the kernel's own monitor injection ALSO skips this and \
          ALSO does not radiate. ⚠ It was switched off by default with the note 'had no measured \
          effect on radiation' — but that note is not a scored result: this part has never been \
          put in front of a witness receiver at all, so 'no measured effect' means 'nobody \
          measured'. Do not promote or delete it from a code read. Mutually exclusive with the \
          other three variants.",
    must_follow: &[StepId("send_fw_info")],
    must_precede: &[StepId("monitor_rx")],
    run: s_fw_sta_emulate,
};

const R_COEX_GRANT_WL: Step<Rtl8821c> = Step {
    id: StepId("coex_grant_wl"),
    stage: Stage::PhyInit,
    class: StepClass::BestEffort(Degradation::new(
        "the antenna stays under PTA (BT-coex) control, so the coex arbiter can gate the Wi-Fi \
         path — the prime RX unblock on this die is not applied",
        "a run whose purpose is the power/firmware sequence; NOT an RSSI or a delivery number, \
         since a PTA-gated antenna is indistinguishable from a dead one at the host",
    )),
    why: "Grant WL the antenna/RF (the coex WONLY path) + coex HW init. Even on a BT-less 8811CU \
          the die powers up with the antenna under PTA control, so ungating WL is the prime RX \
          unblock — the sibling RTL8822E measured the same class of fault as −87 → −22 dBm once \
          the BT-coex grant was forced. Best effort with a named loss, transcribed from the \
          ladder's `eprintln!(\"8821cu coex/grant-WL failed\")`.",
    must_follow: &[StepId("phy_set_param")],
    must_precede: &[StepId("tune_channel")],
    run: s_coex_grant_wl,
};

const R_MONITOR_RX: Step<Rtl8821c> = Step {
    id: StepId("monitor_rx"),
    stage: Stage::RxEnable,
    class: StepClass::Required,
    why: "Promiscuous monitor receive config — it deliberately OVERRIDES the RCR `mac_init` \
          installed, which is why it comes after it. This is what makes the part a monitor rather \
          than a station, and it is what the named-radio bearer's whole RX path depends on.",
    must_follow: &[StepId("mac_init")],
    must_precede: &[StepId("tune_channel")],
    run: s_monitor_rx,
};

const R_TUNE_CHANNEL: Step<Rtl8821c> = Step {
    id: StepId("tune_channel"),
    stage: Stage::Tune,
    class: StepClass::Required,
    why: "BB + RF + RX DFIR + the per-rate TX-power index, for `channel`. It must follow \
          `phy_set_param` (it writes through the tables that rung loads) and `coex_grant_wl` (a \
          PTA-gated antenna makes the tune unobservable), and it must precede the IQK, which is \
          channel-dependent. ★ It is also where power is actually written on this part — see the \
          `AppliedPower` this rung records, and note the part has no `RadioKnobs` impl at all, so \
          nothing can move it afterwards.",
    must_follow: &[StepId("phy_set_param"), StepId("monitor_rx")],
    must_precede: &[StepId("iqk")],
    run: s_tune_channel,
};

const R_BB_RX_PATH_ENABLE: Step<Rtl8821c> = Step {
    id: StepId("bb_rx_path_enable"),
    stage: Stage::RxEnable,
    class: StepClass::Required,
    why: "★ **THIS is what was killing monitor RX.** `RX_PSEL_RST` (REG_RXPSEL 0x0808 bit28|29) is \
          pulsed clear during `phy_set_param`, and the receiver only runs with bit 29 SET (RX path \
          selected). Without it the whole BB RX chain stays inactive: the FA and CRC counters read \
          0 and no frame reaches USB — a radio that looks perfectly healthy and is deaf. One \
          register write, and it is the difference between a receiver and a paperweight.",
    must_follow: &[StepId("tune_channel")],
    must_precede: &[],
    run: s_bb_rx_path_enable,
};

const R_IQK: Step<Rtl8821c> = Step {
    id: StepId("iqk"),
    stage: Stage::Calibrate,
    class: StepClass::BestEffort(Degradation::new(
        "the IQ imbalance is uncorrected for this channel, so carrier leakage and EVM are \
         uncharacterised — the transmitter's constellation is whatever the tables default to",
        "RX capture and link-level work at the robust low rates; NOT an EVM, a spectral-mask or a \
         high-order-MCS delivery number",
    )),
    why: "The firmware-offloaded IQK — the ONLY calibration on this part, and it needs firmware \
          running. Best effort with a named loss, transcribed from the ladder's `tracing::warn!(\
          \"8821cu IQK skipped\")`: a timeout must not abort an otherwise working RX bring-up.",
    must_follow: &[StepId("tune_channel")],
    must_precede: &[],
    run: s_iqk,
};

/// In the canonical plan and in three of the four variants; absent from [`PLAN_8821CU_NO_TXEN`].
const R_TXEN_GOLDEN_BLOCK: Step<Rtl8821c> = Step {
    id: StepId("txen_golden_block"),
    stage: Stage::TxEnable,
    class: StepClass::Required,
    why: "☠ **UNTESTED HYPOTHESIS #2 — and it is the one that is ON BY DEFAULT, which is why \
          removing it is the variant.** The golden 'finalize / function-enable' block the kernel \
          emits after the channel set and this port otherwise skips: `SYS_FUNC_EN = 0x1f` (all \
          blocks, against our partial set), the BB-TX-region `0x1c94 = 0xafffafff`, and \
          `0x042c`/`0x045f`/`0x06dc`. The theory is that TX keys but emits no RF without it. It \
          entered the ladder enabled and `NDN_RADIO_NO_TXEN` was its A/B, so the DEFAULT carries \
          an unmeasured hypothesis — which is exactly why [`PLAN_8821CU_NO_TXEN`] exists rather \
          than this being deleted on a code read. One bench session against a witness scores both \
          directions at once. ⚠ Required rather than best-effort because these are plain register \
          writes with no failure mode short of the bus being gone.",
    must_follow: &[StepId("tune_channel")],
    must_precede: &[StepId("hci_usb_cfg")],
    run: s_txen_golden_block,
};

/// ☠ VARIANT RUNG — [`PLAN_8821CU_STATION_REGS`] only.
const R_STATION_IDENTITY_REGS: Step<Rtl8821c> = Step {
    id: StepId("station_identity_regs"),
    stage: Stage::TxEnable,
    class: StepClass::Required,
    why: "☠ **UNTESTED HYPOTHESIS #3, awaiting one bench session against a witness.** The \
          station-identity registers the kernel writes before it transmits, decoded from the \
          `scan_tx` golden trace and absent from its monitor path: REG_MACID = our own MAC (the A2 \
          of frames we inject) plus a few TX-control bytes. The theory is that the TX engine keys \
          the PA off REG_MACID, and that with it zeroed (the monitor default) frames dequeue but \
          never radiate. The ladder's own comment called this 'the prime TX-radiate hypothesis' — \
          while the comment eight lines above called `fw_sta_emulate` the suspected gate and the \
          one below called IBSS 'the TX-radiate path'. Three primes is zero measurements. Mutually \
          exclusive with the other three variants.",
    must_follow: &[StepId("tune_channel")],
    must_precede: &[StepId("hci_usb_cfg")],
    run: s_station_identity_regs,
};

/// ☠ VARIANT RUNG — [`PLAN_8821CU_IBSS`] only.
const R_IBSS_OPMODE: Step<Rtl8821c> = Step {
    id: StepId("ibss_opmode"),
    stage: Stage::TxEnable,
    class: StepClass::BestEffort(Degradation::new(
        "the MAC stays in monitor's NO_LINK operating mode, so the ad-hoc opmode this variant \
         exists to test was not installed — but `build_80211` will still stamp the IBSS BSSID as \
         addr3, so the frames are IBSS-shaped and the MAC is not",
        "nothing this variant was run for — re-run rather than reading the result",
    )),
    why: "☠ **UNTESTED HYPOTHESIS #4, awaiting one bench session against a witness.** Put the MAC \
          into ad-hoc (IBSS) operating mode: REG_CR network-type = ADHOC, valid EDCA AC params, \
          beacon control, own MAC + a BSSID, replicated from the kernel's `golden_ibss.pcap` \
          WITHOUT the reserved-page beacon download. The theory is that monitor's NO_LINK opmode \
          gates all host TX (the chip dequeues and drains the FIFO but never keys the PA) and that \
          opmode + EDCA + BSSID alone ungates injected data TX. ★ This is the one variant that \
          also changes what goes ON AIR: it sets `ibss_mode`, so `build_80211` stamps \
          `IBSS_BSSID` as addr3. That coupling used to be a second, independent read of \
          `NDN_RADIO_IBSS` on the per-frame path. Mutually exclusive with the other three.",
    must_follow: &[StepId("tune_channel")],
    must_precede: &[StepId("hci_usb_cfg")],
    run: s_ibss_opmode,
};

const R_HCI_USB_CFG: Step<Rtl8821c> = Step {
    id: StepId("hci_usb_cfg"),
    stage: Stage::RxEnable,
    class: StepClass::Required,
    why: "★ USB RX burst + aggregation config, and it is LAST on purpose — `rtw_hci_start`'s own \
          ordering — so that the BB table load and the channel set cannot clobber REG_RXDMA_MODE. \
          This is what makes the chip DMA received frames to bulk-IN at all. Its position is the \
          reason the four variant rungs sit above it rather than at the end.",
    must_follow: &[StepId("bb_rx_path_enable")],
    must_precede: &[],
    run: s_hci_usb_cfg,
};

// ── the five step lists ──────────────────────────────────────────────────────
//
// ★ Each `const R_*` above is referenced by every list that contains it, so the shared prefix is
// LITERALLY shared rather than copy-pasted. Writing the common sixteen rungs out five times would
// have reintroduced "several sequences per part whose only difference nobody can see" inside the
// fix for it.

const STEPS_CANONICAL: &[Step<Rtl8821c>] = &[
    R_READ_CUT_VERSION,
    R_PRE_SYSTEM_CFG,
    R_POWER_CYCLE_IF_ON,
    R_CARD_ENABLE,
    R_READ_CHIP_INFO,
    R_DOWNLOAD_FIRMWARE,
    R_MAC_INIT,
    R_PHY_SET_PARAM,
    R_SEND_FW_INFO,
    R_COEX_GRANT_WL,
    R_MONITOR_RX,
    R_TUNE_CHANNEL,
    R_BB_RX_PATH_ENABLE,
    R_IQK,
    R_TXEN_GOLDEN_BLOCK,
    R_HCI_USB_CFG,
];

const STEPS_FW_STA: &[Step<Rtl8821c>] = &[
    R_READ_CUT_VERSION,
    R_PRE_SYSTEM_CFG,
    R_POWER_CYCLE_IF_ON,
    R_CARD_ENABLE,
    R_READ_CHIP_INFO,
    R_DOWNLOAD_FIRMWARE,
    R_MAC_INIT,
    R_PHY_SET_PARAM,
    R_SEND_FW_INFO,
    R_FW_STA_EMULATE, // + hypothesis #1, in the ladder's own position (5b.2)
    R_COEX_GRANT_WL,
    R_MONITOR_RX,
    R_TUNE_CHANNEL,
    R_BB_RX_PATH_ENABLE,
    R_IQK,
    R_TXEN_GOLDEN_BLOCK,
    R_HCI_USB_CFG,
];

const STEPS_NO_TXEN: &[Step<Rtl8821c>] = &[
    R_READ_CUT_VERSION,
    R_PRE_SYSTEM_CFG,
    R_POWER_CYCLE_IF_ON,
    R_CARD_ENABLE,
    R_READ_CHIP_INFO,
    R_DOWNLOAD_FIRMWARE,
    R_MAC_INIT,
    R_PHY_SET_PARAM,
    R_SEND_FW_INFO,
    R_COEX_GRANT_WL,
    R_MONITOR_RX,
    R_TUNE_CHANNEL,
    R_BB_RX_PATH_ENABLE,
    R_IQK,
    // − hypothesis #2: `txen_golden_block` is ABSENT here. This is the only variant that
    //   subtracts, and it is the A/B for the one hypothesis that ships enabled.
    R_HCI_USB_CFG,
];

const STEPS_STATION_REGS: &[Step<Rtl8821c>] = &[
    R_READ_CUT_VERSION,
    R_PRE_SYSTEM_CFG,
    R_POWER_CYCLE_IF_ON,
    R_CARD_ENABLE,
    R_READ_CHIP_INFO,
    R_DOWNLOAD_FIRMWARE,
    R_MAC_INIT,
    R_PHY_SET_PARAM,
    R_SEND_FW_INFO,
    R_COEX_GRANT_WL,
    R_MONITOR_RX,
    R_TUNE_CHANNEL,
    R_BB_RX_PATH_ENABLE,
    R_IQK,
    R_TXEN_GOLDEN_BLOCK,
    R_STATION_IDENTITY_REGS, // + hypothesis #3, in the ladder's own position (8c)
    R_HCI_USB_CFG,
];

const STEPS_IBSS: &[Step<Rtl8821c>] = &[
    R_READ_CUT_VERSION,
    R_PRE_SYSTEM_CFG,
    R_POWER_CYCLE_IF_ON,
    R_CARD_ENABLE,
    R_READ_CHIP_INFO,
    R_DOWNLOAD_FIRMWARE,
    R_MAC_INIT,
    R_PHY_SET_PARAM,
    R_SEND_FW_INFO,
    R_COEX_GRANT_WL,
    R_MONITOR_RX,
    R_TUNE_CHANNEL,
    R_BB_RX_PATH_ENABLE,
    R_IQK,
    R_TXEN_GOLDEN_BLOCK,
    R_IBSS_OPMODE, // + hypothesis #4, in the ladder's own position (8d)
    R_HCI_USB_CFG,
];

const EXCLUDED_8821C: &[(Stage, &str)] = &[
    (
        Stage::Power,
        "no Power-stage rung. This part writes its per-rate TXAGC block inside `tune_channel` \
         (power is per-channel here), and it has NO `RadioKnobs` impl at all — so there is no knob \
         a later caller could move and no separate rung to put one in. Textbook \
         decided-but-unactuated, recorded rather than papered over: the report names the applied \
         index and the fact that nothing can change it.",
    ),
    (
        Stage::Posture,
        "no Posture rung. The contention actuators are unported on this part (no `RadioKnobs`), so \
         EDCA is whatever the chip booted with — except under `PLAN_8821CU_IBSS`, whose \
         `setup_ibss` writes AC params as part of the ad-hoc opmode. ⚠ That means the IBSS variant \
         changes contention as well as opmode, which is a confound its bench session must account \
         for.",
    ),
    (
        Stage::Verify,
        "no Verify rung, and `asserts()` is empty in writing — see `ASSERTS_8821CU`.",
    ),
];

const fn plan_8821c(name: &'static str, steps: &'static [Step<Rtl8821c>]) -> Plan<Rtl8821c> {
    Plan {
        id: PlanId {
            part: "rtl8821c",
            name,
            // v1 was the M2 hand-filled `monitor` report, which described the ladder as one
            // undifferentiated sequence with the four hypotheses invisible inside it.
            ver: 2,
        },
        role: Role::TransmitAndReceive,
        steps,
        excluded: EXCLUDED_8821C,
    }
}

const P_MONITOR: Plan<Rtl8821c> = plan_8821c("monitor", STEPS_CANONICAL);
const P_FW_STA: Plan<Rtl8821c> = plan_8821c("monitor+fw_sta", STEPS_FW_STA);
const P_NO_TXEN: Plan<Rtl8821c> = plan_8821c("monitor-no_txen", STEPS_NO_TXEN);
const P_STATION_REGS: Plan<Rtl8821c> = plan_8821c("monitor+station_regs", STEPS_STATION_REGS);
const P_IBSS: Plan<Rtl8821c> = plan_8821c("monitor+ibss", STEPS_IBSS);

// ★ All five are checked at COMPILE TIME. A variant that reorders a rung, or names an ordering
// constraint whose target it does not contain, stops the crate building — which matters more here
// than anywhere else in this crate, because five sequences that differ by one rung each are
// exactly the shape that produced sixteen invisible 8812au ladders.
const _: () = P_MONITOR.check_or_panic();
const _: () = P_FW_STA.check_or_panic();
const _: () = P_NO_TXEN.check_or_panic();
const _: () = P_STATION_REGS.check_or_panic();
const _: () = P_IBSS.check_or_panic();

/// The canonical RTL8821CU monitor plan — the sequence this part has always run by default.
pub static PLAN_8821CU_MONITOR: Plan<Rtl8821c> = P_MONITOR;
/// ☠ TX-radiate hypothesis #1 — `NDN_RADIO_STA`. Untested; see [`R_FW_STA_EMULATE`].
pub static PLAN_8821CU_FW_STA: Plan<Rtl8821c> = P_FW_STA;
/// ☠ TX-radiate hypothesis #2, inverted — `NDN_RADIO_NO_TXEN`. Untested; see
/// [`R_TXEN_GOLDEN_BLOCK`], which ships ENABLED and is therefore the one hypothesis the default
/// carries.
pub static PLAN_8821CU_NO_TXEN: Plan<Rtl8821c> = P_NO_TXEN;
/// ☠ TX-radiate hypothesis #3 — `NDN_RADIO_STAREGS`. Untested; see [`R_STATION_IDENTITY_REGS`].
pub static PLAN_8821CU_STATION_REGS: Plan<Rtl8821c> = P_STATION_REGS;
/// ☠ TX-radiate hypothesis #4 — `NDN_RADIO_IBSS`. Untested, and the only one that also changes
/// what goes on air; see [`R_IBSS_OPMODE`].
pub static PLAN_8821CU_IBSS: Plan<Rtl8821c> = P_IBSS;

/// §1.5 — read back every gate you write. **Empty on this part, in writing.**
///
/// The gate worth reading back here is `REG_RXPSEL` bit 29 — the one register write that decides
/// whether the BB RX chain runs at all, and the one that was MEASURED to be the reason monitor RX
/// was dead. It is not asserted, and the reason is a rule rather than an oversight: this driver has
/// **no `read32` path that has been validated against a golden trace for that address**, the port
/// is explicitly incomplete (the firmware download carries its own `TODO(hw)`), and §1.5's own
/// discipline is that an assert is a claim about what a register MEANS. Adding one whose failure
/// nobody has ever watched would put a `Warn` line in front of every operator with nothing behind
/// it — the `MT_TOP_MISC` mistake in [`crate::connac2::regs`], repeated.
///
/// The right first assert on this part is `0x0808 bit 29` after `bb_rx_path_enable`, and it should
/// be added by whoever runs the bench session that scores the four variants — in the same sitting,
/// against the same witness. Recorded here as an open item so it is a decision and not a blank.
const ASSERTS_8821CU: &[Assert<Rtl8821c>] = &[];

/// §4 — what this part can prove about its own transmitter: **nothing, and the reason is the
/// textbook case of this codebase's characteristic defect.**
const TX_UNPROVABLE_8821CU: &str = "this part sets SPE_RPT on every transmitted frame and then DISCARDS the C2H report it asks \
     for — the instrument is armed on every frame and nothing reads it. Decided-but-unactuated, \
     textbook. It also has no `RadioKnobs` impl at all. Question (A) is one C2H handler away and \
     is NOT answered; and note that this part has never been observed to radiate at all, which is \
     what `Rtl8821cVariant`'s four untested hypotheses exist to settle — with a witness, which is \
     question (B) and cannot be answered from this host.";

impl BringUp for Rtl8821cuBackend {
    /// ⚠ Returns the CANONICAL plan. The four variants are selected by
    /// [`Rtl8821cVariant`] through [`bring_up_planned`](Rtl8821cuBackend::bring_up_planned),
    /// because `BringUp::plan` is keyed on [`Role`] and a variant is not a role — it is a
    /// hypothesis about the same role. Every variant reports its own `PlanId` and therefore its
    /// own digest, which is what keeps a hypothesis run from being compared with a production one.
    fn plan(role: Role) -> Option<&'static Plan<Self>> {
        match role {
            Role::TransmitAndReceive => Some(&PLAN_8821CU_MONITOR),
            // ★ Named refusals. One ladder is all this part has ever run, and it is an incomplete
            // port whose transmit path is unproven — inventing an RX-only or TX-only variant of a
            // sequence that has never been shown to work end to end would be two unmeasured
            // ladders instead of one.
            Role::ReceiveOnly | Role::TransmitOnly => None,
        }
    }

    fn asserts() -> &'static [Assert<Self>] {
        ASSERTS_8821CU
    }

    /// Empty — see [`TX_UNPROVABLE_8821CU`].
    fn tx_instruments() -> &'static [ndn_radio_hal::TxInstrument] {
        &[]
    }

    fn tx_unprovable_reason() -> Option<&'static str> {
        Some(TX_UNPROVABLE_8821CU)
    }
}

/// The cost of the pre-M8 wrapper signature: `FaceError` cannot carry a partial report, so it is
/// **emitted before it is dropped**.
fn drop_partial_report(f: BringUpFailure) -> FaceError {
    f.report.emit();
    eprintln!(
        "rtl8821cu bring-up FAILED at `{}` — the partial report:\n{}",
        f.failed_at,
        f.report.render()
    );
    f.source
}
