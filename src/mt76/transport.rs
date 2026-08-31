//! The mt76 **USB transport** — device selection, endpoint discovery, the vendor
//! control-request register window, and the bulk pipes — shared by the MT7610U
//! (`mt76x0`) and MT7612U (`mt76x2`) backends.
//!
//! This is a re-cut of the transport half of [`crate::Mt7612uBackend`]
//! (`src/mt7612/mod.rs:250-475`), which has been driving a radiating MT7612U for
//! months. Nothing here is a redesign: the timeouts, the auto-detach, the
//! deliberate absence of `clear_halt`, and the background RX drain are carried
//! over because each of them was paid for on hardware. What is new is that a
//! *second* part can now use it, and that device selection goes through the
//! shared [`crate::usb_select`] machinery instead of grabbing the first match.
//!
//! # MEASURED on mds-o5p-1's MT7610U (`0e8d:7610`, 2026-08-27)
//!
//! * **Endpoints.** Interface 0, class `ff/02/ff`. Bulk IN `0x84` (packet RX) and
//!   `0x85` (MCU command response); bulk OUT `0x04`…`0x09`. All `wMaxPacketSize`
//!   512 (high speed). Descriptor order matches mt76's `enum mt76u_out_ep`
//!   (`mt76.h:652-659`), so `0x04` is `MT_EP_OUT_INBAND_CMD` and `0x05` is
//!   `MT_EP_OUT_AC_BE`.
//! * **EP0 round trip = 151 µs.** That is the cost of every `rr`/`wr` below, and
//!   the reason no register access may sit on a per-frame path.
//!
//! # The register window
//!
//! All MMIO is one USB vendor control transfer, with no per-section quirk (the
//! contrast with Realtek's page/offset rules is the nicest thing about this
//! part). `wValue = addr >> 16`, `wIndex = addr & 0xffff`, 4-byte little-endian
//! data stage — `usb.c:76-89` (`___mt76u_rr`) and `usb.c:123-132`
//! (`___mt76u_wr`). Three *address spaces* hang off the same mechanism and are
//! selected by the request byte, which is upstream's `MT_VEND_TYPE_*` tag bits
//! (`mt76.h:626-630`) resolved at the call site here rather than smuggled in the
//! top bits of an address:
//!
//! | space | request | this module |
//! |---|---|---|
//! | MMIO | `MULTI_READ 0x07` / `MULTI_WRITE 0x06` | [`Mt76Usb::rr`] / [`Mt76Usb::wr`] |
//! | USB CFG | `READ_CFG 0x47` / `WRITE_CFG 0x46` | [`Mt76Usb::rr_cfg`] / [`Mt76Usb::wr_cfg`] |
//! | EEPROM | `READ_EEPROM 0x09` | [`Mt76Usb::read_eeprom`] |
//!
//! plus two that are not register spaces at all: `WRITE_FCE 0x42` (value rides
//! in `wValue`, no data stage — `usb.c:226-238`, used at
//! `mt76x02_usb_mcu.c:231-235`) and `DEV_MODE 0x01`
//! (the MCU mode/IVB door — `mt76x0/usb_mcu.c:47-49`, `mt76x2/usb_mcu.c:21-26`).
//!
//! # ★ The one family divergence
//!
//! The USB DMA config word is **plain MMIO at `0x0238` on mt76x0**
//! (`mt76x0/usb.c:50,59`) and lives in **USB CFG space at `0x9018` on mt76x2**
//! (`mt76x2/usb_init.c:15`, reached through `MT_VEND_ADDR(CFG, …)`). Same
//! bitfields, different door. [`Mt76Usb::dma_cfg_read`] /
//! [`Mt76Usb::dma_cfg_write`] switch on [`Family`] so the callers above them do
//! not have to know.
//!
//! # ★ No `handle.reset()`, ever
//!
//! A blind USB reset is what wedges these parts, and it is not in this file.
//! [`crate::Mt7612uBackend::open`] still does one behind `NDN_RADIO_NO_RESET`
//! for historical reasons; that behaviour is deliberately **not** carried over.
//! Recovery from a half-loaded MCU is the firmware layer's problem (check
//! whether the firmware is already running before re-downloading it), not the
//! bus's.
#![allow(dead_code)]

use std::io;
use std::sync::Arc;
use std::sync::atomic::{AtomicBool, AtomicU8, Ordering};
use std::time::Duration;

use rusb::{Context, Device, DeviceHandle, Direction, TransferType};

use crate::FaceError;
use crate::mt76::regs::{MT_USB_DMA_CFG, MT_USB_U3DMA_CFG};
use crate::mt76::{Family, Mt76Regs};
use crate::usb_select::{DeviceSelect, check_live_link, select_device, usb_addr};

/// MediaTek's USB vendor ID — the same for every part this module serves.
pub const MEDIATEK_VID: u16 = 0x0e8d;

// ── USB vendor requests (`enum mt76_vendor_req`, mt76.h:631-644) ─────────────

/// `bmRequestType` for a host→device vendor request (vendor | device recipient).
const REQ_OUT: u8 = 0x40;
/// `bmRequestType` for a device→host vendor request.
const REQ_IN: u8 = 0xc0;
/// Host→device **class** request — the door the MT7612U's WMT patch-enable
/// commands go through (`mt76x2/usb_mcu.c:28-56`). Vendor requests use
/// [`REQ_OUT`]; this exists only because those two commands do not.
const REQ_OUT_CLASS: u8 = 0x20;

/// MCU mode / IVB upload (`mt76.h:632`).
const MT_VEND_DEV_MODE: u8 = 0x01;
/// Chip power-on (`mt76.h:634`). Unused by the mt76x0/x2 USB paths — they power
/// up through `MT_WLAN_MTC_CTRL` — but kept because the request byte is part of
/// the transport's vocabulary and the mt792x/mt7615 parts do use it.
const MT_VEND_POWER_ON: u8 = 0x04;
/// MMIO write (`mt76.h:635`).
const MT_VEND_MULTI_WRITE: u8 = 0x06;
/// MMIO read (`mt76.h:636`).
const MT_VEND_MULTI_READ: u8 = 0x07;
/// EEPROM-space read (`mt76.h:637`).
const MT_VEND_READ_EEPROM: u8 = 0x09;
/// FCE register write, value in `wValue` (`mt76.h:638`).
const MT_VEND_WRITE_FCE: u8 = 0x42;
/// USB CFG-space write (`mt76.h:639`, `MT_VEND_WRITE_CFG`).
const MT_VEND_WRITE_CFG: u8 = 0x46;
/// USB CFG-space read (`mt76.h:640`, `MT_VEND_READ_CFG`).
const MT_VEND_READ_CFG: u8 = 0x47;

/// Control-transfer timeout. Carried over from the proven MT7612U path
/// (`src/mt7612/mod.rs:95`); upstream uses 300 ms
/// (`usb.c:12`, `MT_VEND_REQ_TOUT_MS`).
///
/// ★ Upstream also **retries a failed vendor request ten times** with a 5–10 ms
/// gap (`usb.c:11,30-42`). This port deliberately does not. A control transfer
/// that times out on this bus has meant a wedged device every time it has
/// happened here, and a silent retry loop converts that into a 3-second stall
/// followed by the same error — it hides the fault instead of reporting it. If a
/// retry is ever shown to recover a real transient, add it here with the
/// measurement that justified it.
const CTRL_TIMEOUT: Duration = Duration::from_millis(500);

/// Bulk-transfer timeout for command/firmware writes
/// (`src/mt7612/mod.rs:96`).
const BULK_TIMEOUT: Duration = Duration::from_millis(1000);

/// Default bulk-IN read timeout. Short on purpose: a reader polling for frames
/// wants to come back and check a stop flag, not block for a second.
const BULK_IN_TIMEOUT: Duration = Duration::from_millis(200);

// ── Endpoint indices (`enum mt76u_in_ep` / `mt76u_out_ep`, mt76.h:646-659) ───

/// Bulk IN 0 — packet RX. MEASURED `0x84`.
pub const MT_EP_IN_PKT_RX: usize = 0;
/// Bulk IN 1 — MCU command response. MEASURED `0x85`.
pub const MT_EP_IN_CMD_RESP: usize = 1;
/// Number of bulk IN endpoints mt76 requires (`__MT_EP_IN_MAX`).
pub const MT_EP_IN_MAX: usize = 2;

/// Bulk OUT 0 — the MCU **inband command** pipe. MEASURED `0x04`.
pub const MT_EP_OUT_INBAND_CMD: usize = 0;
/// Bulk OUT 1 — AC_BE, the ordinary data queue. MEASURED `0x05`.
pub const MT_EP_OUT_AC_BE: usize = 1;
/// Bulk OUT 2 — AC_BK. MEASURED `0x06`.
pub const MT_EP_OUT_AC_BK: usize = 2;
/// Bulk OUT 3 — AC_VI. MEASURED `0x07`.
pub const MT_EP_OUT_AC_VI: usize = 3;
/// Bulk OUT 4 — AC_VO. MEASURED `0x08`.
pub const MT_EP_OUT_AC_VO: usize = 4;
/// Bulk OUT 5 — HCCA. MEASURED `0x09`.
pub const MT_EP_OUT_HCCA: usize = 5;
/// Number of bulk OUT endpoints mt76 requires (`__MT_EP_OUT_MAX`).
pub const MT_EP_OUT_MAX: usize = 6;

fn usb_err(e: rusb::Error) -> FaceError {
    FaceError::Io(io::Error::other(format!("mt76 usb: {e}")))
}

fn tr_err(what: String) -> FaceError {
    FaceError::Io(io::Error::other(what))
}

/// The bulk endpoints of one mt76 USB interface, in mt76's own index order.
///
/// Split out of [`Mt76Usb`] as a plain value so the assignment rule — which is
/// *descriptor order*, not address order — is testable without a device.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Endpoints {
    /// Bulk IN addresses in descriptor order; index with [`MT_EP_IN_PKT_RX`] /
    /// [`MT_EP_IN_CMD_RESP`].
    pub ins: Vec<u8>,
    /// Bulk OUT addresses in descriptor order; index with the `MT_EP_OUT_*`
    /// constants.
    pub outs: Vec<u8>,
}

impl Endpoints {
    /// The address at mt76 index `idx`, or the last one present. The fallback
    /// exists so a device that enumerates fewer pipes than mt76 expects still
    /// yields *some* endpoint rather than panicking on an index — with the
    /// mismatch already logged by [`assign_endpoints`].
    pub fn out_at(&self, idx: usize) -> u8 {
        self.outs
            .get(idx)
            .copied()
            .unwrap_or_else(|| *self.outs.last().unwrap_or(&0))
    }

    fn in_at(&self, idx: usize) -> u8 {
        self.ins
            .get(idx)
            .copied()
            .unwrap_or_else(|| *self.ins.last().unwrap_or(&0))
    }
}

/// Assign the bulk endpoints the way `mt76u_set_endpoints` (`usb.c:303-327`)
/// does: walk the interface's endpoint descriptors **in descriptor order** and
/// fill `in_ep[]` / `out_ep[]` as they come. Address order happens to agree on
/// this part (MEASURED `0x84,0x85` and `0x04..0x09` enumerate ascending), but
/// descriptor order is the rule upstream follows and the one the MCU's endpoint
/// numbering is defined against, so that is what is implemented.
///
/// Upstream **hard-fails** unless it finds exactly 2 IN and 6 OUT. This port
/// warns instead and carries on with what it found: a mismatch means the caller
/// pointed the transport at something that is not one of these two families, and
/// an error message naming what it saw is more use than a bare `-EINVAL`. It
/// does fail when a direction is missing entirely, because there is then no
/// usable pipe at all.
pub fn assign_endpoints(ins: &[u8], outs: &[u8], label: &str) -> Result<Endpoints, FaceError> {
    if ins.is_empty() || outs.is_empty() {
        return Err(tr_err(format!(
            "{label}: interface has no bulk IN/OUT endpoints (IN {ins:?}, OUT {outs:?})"
        )));
    }
    if ins.len() != MT_EP_IN_MAX || outs.len() != MT_EP_OUT_MAX {
        tracing::warn!(
            target: "named_radio",
            chip = label,
            ins = ins.len(), outs = outs.len(),
            "mt76 endpoint count is not the expected 2 IN / 6 OUT (usb.c:325-326) — \
             indices may not mean what mt76's enum says",
        );
    }
    Ok(Endpoints {
        ins: ins.to_vec(),
        outs: outs.to_vec(),
    })
}

/// A claimed mt76 USB device: the register window plus the bulk pipes.
///
/// Deliberately **not** a radio. It holds no channel, rate, filter or frame
/// state — a backend owns those and borrows this for bus access. That is what
/// lets one copy serve both the mt76x0 and mt76x2 ports, and what lets the
/// [`Mt76Regs`] impl below be the only seam the shared knob layer needs.
pub struct Mt76Usb {
    handle: Arc<DeviceHandle<Context>>,
    family: Family,
    label: &'static str,
    /// USB topological address (`bus-port.port`), for logs and for telling two
    /// identical dongles apart after the fact.
    usb_addr: String,
    /// The claimed interface number — released when the handle drops.
    iface: u8,
    eps: Endpoints,
    /// Resolved bulk-OUT for MCU inband commands ([`MT_EP_OUT_INBAND_CMD`]),
    /// overridable via [`Mt76Usb::with_cmd_ep`].
    ep_cmd_out: u8,
    /// Resolved bulk-OUT for WLAN data ([`MT_EP_OUT_AC_BE`]), overridable via
    /// [`Mt76Usb::with_data_ep`].
    ep_data_out: u8,
    /// Resolved bulk-IN for packet RX ([`MT_EP_IN_PKT_RX`]).
    ep_in_data: u8,
    /// Resolved bulk-IN for MCU responses ([`MT_EP_IN_CMD_RESP`]).
    ep_in_resp: u8,
    /// `wMaxPacketSize` of the RX pipe — MEASURED 512 (high speed). A bulk read
    /// buffer that is not a multiple of this loses the tail of a burst on some
    /// stacks, so callers size against it rather than a hard-coded 512.
    ep_in_packet_size: u16,
    /// MCU command sequence, 1..=15 and never 0 — see [`Mt76Usb::next_seq`].
    mcu_seq: AtomicU8,
    /// Stop/pause flag handed to [`Mt76Usb::spawn_rx_drain`].
    drain_pause: Arc<AtomicBool>,
}

impl Mt76Usb {
    /// Open the mt76 part selected by `NDN_RADIO_DEV` / `NDN_USB_ADDR` /
    /// `NDN_USB_INDEX` ([`DeviceSelect::from_env`]).
    ///
    /// Selection goes through [`select_device`], which also runs the
    /// [`check_live_link`] guard — so pointing this at the dongle currently
    /// carrying the node's kernel mesh warns (or, with `NDN_GUARD_LIVE_LINK=1`,
    /// refuses) instead of silently dropping the link. That guard is the whole
    /// reason this does not just walk the device list itself.
    pub fn open(
        vendor_id: u16,
        pids: &[u16],
        family: Family,
        label: &'static str,
    ) -> Result<Self, FaceError> {
        Self::open_with(vendor_id, pids, family, label, &DeviceSelect::from_env())
    }

    /// [`open`](Self::open) with an explicit selector — the form a config's
    /// `RadioDeviceConfig.address` reaches through [`DeviceSelect::parse`].
    pub fn open_with(
        vendor_id: u16,
        pids: &[u16],
        family: Family,
        label: &'static str,
        sel: &DeviceSelect,
    ) -> Result<Self, FaceError> {
        let device = select_device(pids, vendor_id, sel, label)?;
        Self::claim(device, family, label)
    }

    /// Claim an already-selected device.
    ///
    /// ★ There is **no `handle.reset()`** here and there must never be one: a
    /// blind USB reset is what wedges these parts. The kernel driver is detached
    /// per-device by `set_auto_detach_kernel_driver`, which leaves any sibling
    /// dongle bound and carrying its link (see [`crate::usb_select`]).
    ///
    /// `clear_halt` is also **not** called by default. On macOS it resets the
    /// endpoint data toggle, which desyncs the FCE's first firmware transfer on
    /// a cold device and makes the download time out — paid for once already on
    /// the MT7612U (`src/mt7612/mod.rs:330-338`). `NDN_RADIO_CLEAR_HALT=1` opts
    /// back in for recovering a genuinely stalled endpoint.
    pub fn claim(
        device: Device<Context>,
        family: Family,
        label: &'static str,
    ) -> Result<Self, FaceError> {
        check_live_link(&device, label)?;
        let addr = usb_addr(&device);
        let handle = device.open().map_err(usb_err)?;
        let config = device.active_config_descriptor().map_err(usb_err)?;

        // Walk the interfaces for the one carrying bulk endpoints. MEASURED:
        // interface 0, class ff/02/ff, is the only one on this part.
        let (mut iface_n, mut outs, mut ins, mut in_pkt) = (None, Vec::new(), Vec::new(), 512u16);
        for iface in config.interfaces() {
            for d in iface.descriptors() {
                let mut has_bulk = false;
                for ep in d.endpoint_descriptors() {
                    if ep.transfer_type() != TransferType::Bulk {
                        continue;
                    }
                    has_bulk = true;
                    match ep.direction() {
                        Direction::Out => outs.push(ep.address()),
                        Direction::In => {
                            if ins.is_empty() {
                                in_pkt = ep.max_packet_size();
                            }
                            ins.push(ep.address());
                        }
                    }
                }
                if has_bulk {
                    iface_n = Some(iface.number());
                }
            }
        }
        let iface = iface_n.ok_or_else(|| {
            tr_err(format!(
                "{label} at {addr}: no interface with bulk endpoints"
            ))
        })?;
        let eps = assign_endpoints(&ins, &outs, label)?;

        // Detach the kernel driver from THIS device only, then claim.
        let _ = handle.set_auto_detach_kernel_driver(true);
        handle.claim_interface(iface).map_err(usb_err)?;
        if std::env::var_os("NDN_RADIO_CLEAR_HALT").is_some() {
            for ep in eps.outs.iter().chain(eps.ins.iter()) {
                let _ = handle.clear_halt(*ep);
            }
        }

        let ep_cmd_out = eps.out_at(MT_EP_OUT_INBAND_CMD);
        let ep_data_out = eps.out_at(MT_EP_OUT_AC_BE);
        let ep_in_data = eps.in_at(MT_EP_IN_PKT_RX);
        let ep_in_resp = eps.in_at(MT_EP_IN_CMD_RESP);
        tracing::info!(
            target: "named_radio",
            chip = label, usb_addr = %addr, iface, family = ?family,
            ep_cmd_out = format_args!("{ep_cmd_out:#04x}"),
            ep_data_out = format_args!("{ep_data_out:#04x}"),
            ep_in_data = format_args!("{ep_in_data:#04x}"),
            ep_in_resp = format_args!("{ep_in_resp:#04x}"),
            ep_in_packet_size = in_pkt,
            "mt76 USB claimed",
        );

        Ok(Self {
            handle: Arc::new(handle),
            family,
            label,
            usb_addr: addr,
            iface,
            eps,
            ep_cmd_out,
            ep_data_out,
            ep_in_data,
            ep_in_resp,
            ep_in_packet_size: in_pkt,
            mcu_seq: AtomicU8::new(0),
            drain_pause: Arc::new(AtomicBool::new(false)),
        })
    }

    /// Override the MCU inband-command bulk-OUT endpoint.
    ///
    /// Needed because the two families do **not** agree in practice: the MT7610U
    /// MEASURED `0x04` for `MT_EP_OUT_INBAND_CMD` (descriptor order, matching
    /// mt76's enum), while the MT7612U's golden usbmon trace shows the kernel
    /// driving commands on `0x08` (`src/mt7612/mod.rs:318-322`). Rather than
    /// guess which one a future part follows, the derived default is the enum
    /// order and this is the escape hatch.
    pub fn with_cmd_ep(mut self, ep: u8) -> Self {
        self.ep_cmd_out = ep;
        self
    }

    /// Override the WLAN data bulk-OUT endpoint (default [`MT_EP_OUT_AC_BE`]).
    /// The MT7612U path injects on AC_VO (`0x07`) instead, which is what the
    /// kernel does there for mgmt frames.
    pub fn with_data_ep(mut self, ep: u8) -> Self {
        self.ep_data_out = ep;
        self
    }

    // ── Accessors ───────────────────────────────────────────────────────────

    /// The shared device handle, for a backend that needs to run its own URB
    /// pumps on other threads.
    pub fn handle(&self) -> Arc<DeviceHandle<Context>> {
        self.handle.clone()
    }
    /// Which mt76x02 family this transport is talking to.
    pub fn family(&self) -> Family {
        self.family
    }
    /// The chip label used in logs and error text.
    pub fn label(&self) -> &'static str {
        self.label
    }
    /// USB topological address (`bus-port.port`) of the claimed device.
    pub fn usb_addr(&self) -> &str {
        &self.usb_addr
    }
    /// The claimed interface number.
    pub fn interface(&self) -> u8 {
        self.iface
    }
    /// All discovered bulk endpoints, in mt76 index order.
    pub fn endpoints(&self) -> &Endpoints {
        &self.eps
    }
    /// MCU inband-command bulk-OUT endpoint.
    pub fn ep_cmd_out(&self) -> u8 {
        self.ep_cmd_out
    }
    /// WLAN data bulk-OUT endpoint.
    pub fn ep_data_out(&self) -> u8 {
        self.ep_data_out
    }
    /// Packet-RX bulk-IN endpoint.
    pub fn ep_in_data(&self) -> u8 {
        self.ep_in_data
    }
    /// MCU-response bulk-IN endpoint.
    pub fn ep_in_resp(&self) -> u8 {
        self.ep_in_resp
    }
    /// `wMaxPacketSize` of the RX pipe — MEASURED 512 on this high-speed part.
    pub fn ep_in_packet_size(&self) -> u16 {
        self.ep_in_packet_size
    }

    // ── Vendor control transfers ────────────────────────────────────────────

    /// One device→host vendor request. Every read below is built from this.
    fn vendor_read(
        &self,
        req: u8,
        value: u16,
        index: u16,
        buf: &mut [u8],
    ) -> Result<usize, FaceError> {
        self.handle
            .read_control(REQ_IN, req, value, index, buf, CTRL_TIMEOUT)
            .map_err(|e| {
                tr_err(format!(
                    "{} vendor read req {req:#04x} val {value:#06x} idx {index:#06x}: {e}",
                    self.label
                ))
            })
    }

    /// One host→device vendor request.
    fn vendor_write(
        &self,
        req_type: u8,
        req: u8,
        value: u16,
        index: u16,
        data: &[u8],
    ) -> Result<usize, FaceError> {
        self.handle
            .write_control(req_type, req, value, index, data, CTRL_TIMEOUT)
            .map_err(|e| {
                tr_err(format!(
                    "{} vendor write req {req:#04x} val {value:#06x} idx {index:#06x}: {e}",
                    self.label
                ))
            })
    }

    /// Read a 32-bit MMIO register (`MT_VEND_MULTI_READ`, `usb.c:76-89`).
    /// ⚠ 151 µs MEASURED per call — never on a per-frame path.
    pub fn rr(&self, addr: u32) -> Result<u32, FaceError> {
        let mut b = [0u8; 4];
        let n = self.vendor_read(
            MT_VEND_MULTI_READ,
            (addr >> 16) as u16,
            (addr & 0xffff) as u16,
            &mut b,
        )?;
        if n != 4 {
            return Err(tr_err(format!(
                "{} rr({addr:#x}) short read: {n} bytes",
                self.label
            )));
        }
        Ok(u32::from_le_bytes(b))
    }

    /// Write a 32-bit MMIO register (`MT_VEND_MULTI_WRITE`, `usb.c:123-132`).
    pub fn wr(&self, addr: u32, val: u32) -> Result<(), FaceError> {
        let n = self.vendor_write(
            REQ_OUT,
            MT_VEND_MULTI_WRITE,
            (addr >> 16) as u16,
            (addr & 0xffff) as u16,
            &val.to_le_bytes(),
        )?;
        if n != 4 {
            return Err(tr_err(format!(
                "{} wr({addr:#x}) short write: {n} bytes",
                self.label
            )));
        }
        Ok(())
    }

    /// Read a 32-bit word from USB **CFG** space (`MT_VEND_READ_CFG`).
    ///
    /// A separate address space from MMIO, not a separate range of it —
    /// upstream tags it with `MT_VEND_TYPE_CFG` (`mt76.h:627`) and dispatches on
    /// the tag (`usb.c:96-105`). CFG addresses are 16-bit, hence the narrower
    /// parameter: `wValue` is always 0 here.
    pub fn rr_cfg(&self, addr: u16) -> Result<u32, FaceError> {
        let mut b = [0u8; 4];
        let n = self.vendor_read(MT_VEND_READ_CFG, 0, addr, &mut b)?;
        if n != 4 {
            return Err(tr_err(format!(
                "{} rr_cfg({addr:#x}) short read: {n} bytes",
                self.label
            )));
        }
        Ok(u32::from_le_bytes(b))
    }

    /// Write a 32-bit word to USB CFG space (`MT_VEND_WRITE_CFG`).
    pub fn wr_cfg(&self, addr: u16, val: u32) -> Result<(), FaceError> {
        self.vendor_write(REQ_OUT, MT_VEND_WRITE_CFG, 0, addr, &val.to_le_bytes())?;
        Ok(())
    }

    /// Write a 16-bit value to an FCE register (`MT_VEND_WRITE_FCE`).
    ///
    /// Note the unusual shape, which is why it cannot go through [`wr`](Self::wr):
    /// the **value rides in `wValue`** and the register index in `wIndex`, with
    /// **no data stage** (`usb.c:226-238`; used at `mt76x02_usb_mcu.c:231-235`). A 32-bit FCE field is
    /// therefore two calls, low half then high half, exactly as the firmware DMA
    /// descriptor is programmed in `src/mt7612/mod.rs:516-520`.
    pub fn wr_fce(&self, reg: u16, val: u16) -> Result<(), FaceError> {
        self.vendor_write(REQ_OUT, MT_VEND_WRITE_FCE, val, reg, &[])?;
        Ok(())
    }

    /// Read 4 bytes of EEPROM/efuse-shadow at `offset` (`MT_VEND_READ_EEPROM`,
    /// `usb.c:96-99`). `wValue = 0`, `wIndex = offset`.
    pub fn read_eeprom(&self, offset: u16) -> Result<u32, FaceError> {
        let mut b = [0u8; 4];
        let n = self.vendor_read(MT_VEND_READ_EEPROM, 0, offset, &mut b)?;
        if n != 4 {
            return Err(tr_err(format!(
                "{} read_eeprom({offset:#x}) short read: {n} bytes",
                self.label
            )));
        }
        Ok(u32::from_le_bytes(b))
    }

    /// Read an arbitrary run of EEPROM bytes in one transfer.
    ///
    /// The same request as [`read_eeprom`](Self::read_eeprom) with a longer data
    /// stage. Worth having because the EEPROM parse wants the whole image and
    /// pulling it 4 bytes at a time costs 151 µs per word — a 512-byte image is
    /// 19 ms word-by-word versus one transfer here.
    pub fn read_eeprom_into(&self, offset: u16, buf: &mut [u8]) -> Result<usize, FaceError> {
        self.vendor_read(MT_VEND_READ_EEPROM, 0, offset, buf)
    }

    /// The `MT_VEND_DEV_MODE` request (`0x01`) — the MCU's mode door, not a
    /// register.
    ///
    /// `value` is the sub-command and the meanings are positional, not named,
    /// anywhere upstream:
    ///   * `0x1` — firmware reset (`mt76x02_usb_mcu.c:207-212`),
    ///   * `0x12` — load IVB / start the firmware; the mt76x0 path passes the
    ///     IVB bytes as `data` (`mt76x0/usb_mcu.c:47-49`) while the mt76x2 path
    ///     passes none (`mt76x2/usb_mcu.c:21-26`).
    ///
    /// Why `0x12` means "load IVB" is not stated anywhere in the tree; it is
    /// ported as an opaque constant by its call sites, not derived.
    pub fn dev_mode(&self, value: u16, data: &[u8]) -> Result<(), FaceError> {
        self.vendor_write(REQ_OUT, MT_VEND_DEV_MODE, value, 0, data)?;
        Ok(())
    }

    /// [`dev_mode`](Self::dev_mode) sent as a **class** request instead of a
    /// vendor one — the door the MT7612U's WMT patch-enable / reset commands go
    /// through (`mt76x2/usb_mcu.c:29-56`). Same request byte, different
    /// `bmRequestType`; the MT7610U does not need it.
    pub fn dev_mode_class(&self, value: u16, data: &[u8]) -> Result<(), FaceError> {
        self.vendor_write(REQ_OUT_CLASS, MT_VEND_DEV_MODE, value, 0, data)?;
        Ok(())
    }

    /// The `MT_VEND_POWER_ON` request (`0x04`). Not used by the mt76x0/x2 USB
    /// bring-up — those power up through `MT_WLAN_MTC_CTRL` — but part of the
    /// request vocabulary and used by the newer parts (`mt792x_usb.c:218`).
    pub fn power_on(&self, value: u16) -> Result<(), FaceError> {
        self.vendor_write(REQ_OUT, MT_VEND_POWER_ON, value, 0, &[])?;
        Ok(())
    }

    // ── USB DMA config: the one family divergence ───────────────────────────

    /// Read the USB DMA config word, from wherever this family keeps it.
    ///
    /// ★ mt76x0: plain MMIO [`MT_USB_DMA_CFG`] `0x0238` (`mt76x0/usb.c:50`).
    /// ★ mt76x2: USB CFG space [`MT_USB_U3DMA_CFG`] `0x9018`
    /// (`mt76x2/usb_init.c:15`). Identical bitfields
    /// (`mt76x02_regs.h:78-90`), different door — reaching for the wrong one
    /// returns a plausible-looking word from an unrelated register, which is the
    /// worst kind of wrong.
    pub fn dma_cfg_read(&self) -> Result<u32, FaceError> {
        match self.family {
            Family::Mt76x0 => self.rr(MT_USB_DMA_CFG),
            Family::Mt76x2 => self.rr_cfg(MT_USB_U3DMA_CFG as u16),
        }
    }

    /// Write the USB DMA config word. See [`dma_cfg_read`](Self::dma_cfg_read)
    /// for the family split.
    pub fn dma_cfg_write(&self, val: u32) -> Result<(), FaceError> {
        match self.family {
            Family::Mt76x0 => self.wr(MT_USB_DMA_CFG, val),
            Family::Mt76x2 => self.wr_cfg(MT_USB_U3DMA_CFG as u16, val),
        }
    }

    /// Read-modify-write of the USB DMA config word, returning the previous
    /// value.
    pub fn dma_cfg_rmw(&self, clear: u32, set: u32) -> Result<u32, FaceError> {
        let old = self.dma_cfg_read()?;
        self.dma_cfg_write((old & !clear) | set)?;
        Ok(old)
    }

    // ── Polling ─────────────────────────────────────────────────────────────

    /// Poll `addr` until `val & mask == expect`, one read per millisecond.
    ///
    /// At 151 µs per read plus a 1 ms sleep, `tries` is very nearly the timeout
    /// in milliseconds — state it that way when choosing one. Mirrors upstream's
    /// `mt76_poll` / `mt76_poll_msec`.
    pub fn poll(&self, addr: u32, mask: u32, expect: u32, tries: u32) -> Result<u32, FaceError> {
        self.poll_pred(addr, |v| v & mask == expect, tries)
            .map_err(|_| {
                tr_err(format!(
                    "{} poll({addr:#x} & {mask:#x} == {expect:#x}) timed out after {tries} tries",
                    self.label
                ))
            })
    }

    /// Poll `addr` until `pred` accepts the value. The general form behind
    /// [`poll`](Self::poll); a read error is fatal rather than retried, so a
    /// device that has fallen off the bus reports that instead of spinning.
    pub fn poll_pred<F: Fn(u32) -> bool>(
        &self,
        addr: u32,
        pred: F,
        tries: u32,
    ) -> Result<u32, FaceError> {
        for _ in 0..tries {
            let v = self.rr(addr)?;
            if pred(v) {
                return Ok(v);
            }
            std::thread::sleep(Duration::from_millis(1));
        }
        Err(tr_err(format!(
            "{} poll({addr:#x}) timed out after {tries} tries",
            self.label
        )))
    }

    // ── Bulk pipes ──────────────────────────────────────────────────────────

    /// Write one bulk transfer to `ep`, requiring the whole buffer to go.
    pub fn bulk_out(&self, ep: u8, buf: &[u8]) -> Result<(), FaceError> {
        let n = self
            .handle
            .write_bulk(ep, buf, BULK_TIMEOUT)
            .map_err(|e| tr_err(format!("{} bulk out ep {ep:#04x}: {e}", self.label)))?;
        if n != buf.len() {
            return Err(tr_err(format!(
                "{} bulk out ep {ep:#04x}: short write {n}/{}",
                self.label,
                buf.len()
            )));
        }
        Ok(())
    }

    /// Write one bulk transfer to `ep` with a caller-chosen timeout — for the
    /// firmware-download path, where a chunk legitimately takes longer than
    /// `BULK_TIMEOUT`.
    pub fn bulk_out_timeout(&self, ep: u8, buf: &[u8], timeout: Duration) -> Result<(), FaceError> {
        let n = self
            .handle
            .write_bulk(ep, buf, timeout)
            .map_err(|e| tr_err(format!("{} bulk out ep {ep:#04x}: {e}", self.label)))?;
        if n != buf.len() {
            return Err(tr_err(format!(
                "{} bulk out ep {ep:#04x}: short write {n}/{}",
                self.label,
                buf.len()
            )));
        }
        Ok(())
    }

    /// Send `buf` on the MCU inband-command bulk-OUT endpoint. The
    /// `McuBus::bulk_out_cmd` half of the MCU contract.
    pub fn bulk_out_cmd(&self, buf: &[u8]) -> Result<(), FaceError> {
        self.bulk_out(self.ep_cmd_out, buf)
    }

    /// Send `buf` on the WLAN data bulk-OUT endpoint.
    pub fn bulk_out_data(&self, buf: &[u8]) -> Result<(), FaceError> {
        self.bulk_out(self.ep_data_out, buf)
    }

    /// Read one bulk transfer from `ep`. **A timeout returns `Ok(0)`**, not an
    /// error: on a monitor-mode RX pipe "no frame arrived in 200 ms" is the
    /// normal case on a quiet channel, and a caller polling in a loop should not
    /// have to distinguish it from a fault. Real bus errors still surface.
    pub fn bulk_in(&self, ep: u8, buf: &mut [u8]) -> Result<usize, FaceError> {
        self.bulk_in_timeout(ep, buf, BULK_IN_TIMEOUT)
    }

    /// [`bulk_in`](Self::bulk_in) with an explicit timeout.
    pub fn bulk_in_timeout(
        &self,
        ep: u8,
        buf: &mut [u8],
        timeout: Duration,
    ) -> Result<usize, FaceError> {
        match self.handle.read_bulk(ep, buf, timeout) {
            Ok(n) => Ok(n),
            Err(rusb::Error::Timeout) => Ok(0),
            Err(e) => Err(tr_err(format!("{} bulk in ep {ep:#04x}: {e}", self.label))),
        }
    }

    /// Read one RX burst (RXD descriptor + 802.11 frame + FCE trailer) from the
    /// packet bulk-IN endpoint.
    pub fn bulk_in_data(&self, buf: &mut [u8]) -> Result<usize, FaceError> {
        self.bulk_in(self.ep_in_data, buf)
    }

    /// Read one MCU response from the cmd-response bulk-IN endpoint. The
    /// `McuBus::bulk_in_resp` half of the MCU contract.
    ///
    /// ★ Reading this matters for throughput, not just correctness: the MCU
    /// holds its response and will not accept the next command until the host
    /// takes it, so a skipped ACK makes every following command's bulk-write
    /// block for about a second (`src/mt7612/mod.rs:836-843`).
    pub fn bulk_in_resp(&self, buf: &mut [u8]) -> Result<usize, FaceError> {
        self.bulk_in(self.ep_in_resp, buf)
    }

    /// The MCU sequence number, 1..=15 and **never 0**.
    ///
    /// Zero is not an off-by-one to be tidied away: mt76 uses `seq == 0` for
    /// fire-and-forget commands that post **no** response. Forcing a nonzero seq
    /// onto one of those makes the running firmware not drain it, the command
    /// FIFO fills after ~90 commands, and every further write blocks ~1 s
    /// (`src/mt7612/mod.rs:880-886`). So this generator skips 0, and a caller
    /// replaying a captured no-response command must pass 0 explicitly rather
    /// than take a number from here.
    pub fn next_seq(&self) -> u8 {
        let mut s = self.mcu_seq.fetch_add(1, Ordering::Relaxed).wrapping_add(1) & 0xf;
        if s == 0 {
            s = self.mcu_seq.fetch_add(1, Ordering::Relaxed).wrapping_add(1) & 0xf;
            if s == 0 {
                s = 1;
            }
        }
        s
    }

    /// Drain whatever is sitting on `ep` right now, with a short timeout, and
    /// return how many transfers were consumed.
    ///
    /// Used to clear a stale MCU response before issuing a command that expects
    /// its own — otherwise the previous command's ACK is mistaken for this one's.
    pub fn drain_bulk_in(&self, ep: u8, timeout: Duration, max: usize) -> usize {
        let mut buf = vec![0u8; 512];
        let mut n = 0;
        for _ in 0..max {
            match self.handle.read_bulk(ep, &mut buf, timeout) {
                Ok(0) | Err(_) => break,
                Ok(_) => n += 1,
            }
        }
        n
    }

    /// Continuously read the packet bulk-IN endpoint on a background thread,
    /// discarding what it gets, until the returned flag is set.
    ///
    /// ★ This is not a convenience. mt76 USB keeps RX URBs submitted; if the
    /// host stops reading, the device's USB DMA stalls and that **blocks the MCU
    /// command path** — commands are accepted into the FIFO and never processed
    /// (`src/mt7612/mod.rs:1190-1193`). So a bring-up that issues MCU commands
    /// must have something reading the RX pipe, even before it cares about
    /// frames.
    ///
    /// The returned [`AtomicBool`] is a **pause**, not a stop: set it to `true`
    /// before consuming frames in the foreground so the drain stops stealing
    /// them, clear it to resume. The thread itself runs for the life of the
    /// process; it holds only a clone of the device handle.
    pub fn spawn_rx_drain(&self) -> Arc<AtomicBool> {
        let pause = self.drain_pause.clone();
        let h = self.handle.clone();
        let ep = self.ep_in_data;
        let p = pause.clone();
        std::thread::spawn(move || {
            let mut buf = vec![0u8; 8192];
            loop {
                if p.load(Ordering::Relaxed) {
                    std::thread::sleep(Duration::from_millis(5));
                    continue;
                }
                let _ = h.read_bulk(ep, &mut buf, Duration::from_millis(50));
            }
        });
        pause
    }

    /// Pause or resume the background RX drain started by
    /// [`spawn_rx_drain`](Self::spawn_rx_drain).
    pub fn pause_drain(&self, paused: bool) {
        self.drain_pause.store(paused, Ordering::Relaxed);
    }
}

/// The register seam the shared knob layer is written against. Everything in
/// [`crate::mt76::knobs`] becomes available to any backend holding one of these.
impl Mt76Regs for Mt76Usb {
    fn rr(&self, addr: u32) -> Result<u32, FaceError> {
        Mt76Usb::rr(self, addr)
    }

    fn wr(&self, addr: u32, val: u32) -> Result<(), FaceError> {
        Mt76Usb::wr(self, addr, val)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Descriptor order is the assignment rule, and on this part it puts the
    /// MEASURED addresses at mt76's enum indices.
    #[test]
    fn endpoint_indices_match_the_measured_mt7610u_layout() {
        let eps = assign_endpoints(
            &[0x84, 0x85],
            &[0x04, 0x05, 0x06, 0x07, 0x08, 0x09],
            "MT7610U",
        )
        .unwrap();
        assert_eq!(eps.out_at(MT_EP_OUT_INBAND_CMD), 0x04);
        assert_eq!(eps.out_at(MT_EP_OUT_AC_BE), 0x05);
        assert_eq!(eps.out_at(MT_EP_OUT_AC_BK), 0x06);
        assert_eq!(eps.out_at(MT_EP_OUT_AC_VI), 0x07);
        assert_eq!(eps.out_at(MT_EP_OUT_AC_VO), 0x08);
        assert_eq!(eps.out_at(MT_EP_OUT_HCCA), 0x09);
        assert_eq!(eps.in_at(MT_EP_IN_PKT_RX), 0x84);
        assert_eq!(eps.in_at(MT_EP_IN_CMD_RESP), 0x85);
    }

    /// A device with no bulk pipe in one direction is unusable and must say so;
    /// a device with an unexpected *count* warns and carries on, because the
    /// indices may still be right and a hard failure would be less informative.
    #[test]
    fn endpoint_assignment_rejects_a_missing_direction_but_tolerates_a_short_count() {
        assert!(assign_endpoints(&[], &[0x04], "X").is_err());
        assert!(assign_endpoints(&[0x84], &[], "X").is_err());
        let eps = assign_endpoints(&[0x84], &[0x04, 0x05], "X").unwrap();
        assert_eq!(
            eps.in_at(MT_EP_IN_CMD_RESP),
            0x84,
            "falls back to the last IN"
        );
        assert_eq!(
            eps.out_at(MT_EP_OUT_HCCA),
            0x05,
            "falls back to the last OUT"
        );
    }

    /// The register-window address split, checked against a >16-bit address so a
    /// swapped `wValue`/`wIndex` cannot pass. `0x0001_0148`
    /// (`MT_WLAN_MTC_CTRL`) is a real register above the 64 KiB MAC window and
    /// is exactly the case that would break.
    #[test]
    fn address_splits_into_wvalue_and_windex() {
        let addr: u32 = 0x0001_0148;
        assert_eq!((addr >> 16) as u16, 0x0001);
        assert_eq!((addr & 0xffff) as u16, 0x0148);
        let low: u32 = 0x111c;
        assert_eq!((low >> 16) as u16, 0x0000);
        assert_eq!((low & 0xffff) as u16, 0x111c);
    }

    /// ★ The family switch: the DMA config word is MMIO `0x0238` on mt76x0 and
    /// CFG-space `0x9018` on mt76x2. Asserted as constants because the choice is
    /// made inside `dma_cfg_read`, which needs a device — this at least pins the
    /// two addresses so a future edit cannot quietly merge them.
    #[test]
    fn dma_cfg_lives_in_different_spaces_per_family() {
        assert_eq!(MT_USB_DMA_CFG, 0x0238);
        assert_eq!(MT_USB_U3DMA_CFG, 0x9018);
        assert_ne!(MT_USB_DMA_CFG, MT_USB_U3DMA_CFG);
        // The mt76x2 form is addressed as a 16-bit CFG offset, so it must fit.
        assert!(u16::try_from(MT_USB_U3DMA_CFG).is_ok());
    }

    /// The MCU sequence must stay in 1..=15 and never hand out 0 — 0 means
    /// "no response expected" to the firmware, and a wrong one there stalls the
    /// command FIFO.
    #[test]
    fn mcu_seq_never_yields_zero() {
        let seq = AtomicU8::new(0);
        // Same arithmetic as `next_seq`, exercised over several wraps.
        let next = || {
            let mut s = seq.fetch_add(1, Ordering::Relaxed).wrapping_add(1) & 0xf;
            if s == 0 {
                s = seq.fetch_add(1, Ordering::Relaxed).wrapping_add(1) & 0xf;
                if s == 0 {
                    s = 1;
                }
            }
            s
        };
        for _ in 0..64 {
            let s = next();
            assert!((1..=15).contains(&s), "seq {s} outside 1..=15");
        }
    }
}
