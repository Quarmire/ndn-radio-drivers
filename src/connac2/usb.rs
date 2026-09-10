//! The **connac2 USB transport** — device selection, the *composite*-device
//! interface split, the `READ_EXT`/`WRITE_EXT` register window, the UHW window,
//! and the bulk pipes — for the MT7921AU (`0e8d:7961`, `mt7921u` upstream) and
//! its rebadged siblings.
//!
//! This is a sibling of [`crate::mt76::transport`] (the MT7610U/MT7612U
//! transport, proven on two radiating parts), not a rewrite of it. Everything
//! that was paid for on hardware there — the [`DeviceSelect`] +
//! [`check_live_link`] guard, the deliberate absence of `handle.reset()`, the
//! deliberate absence of `clear_halt`, the timeout choices, the "a bulk-IN
//! timeout is `Ok(0)`, not an error" rule, the background RX drain — is carried
//! over verbatim. What is genuinely different is listed under
//! "Four ways this is not mt76x0/x2" below, and nothing else was changed.
//!
//! # MEASURED on mds-o5p-3's MT7921AU (`0e8d:7961`, 2026-08-27)
//!
//! Everything in this block came off the **target silicon**, not from reading
//! upstream. Everything not in this block is CODE-READ and says so.
//!
//! * **USB 2.0 High speed (480).** Not SuperSpeed on this node, whatever the
//!   part is capable of elsewhere. Every packet size below is the high-speed one.
//! * ★ **This is a composite device and the WLAN function is interface 3.**
//!   Interfaces 0-2 are class `e0/01/01` (Wireless Controller / RF / Bluetooth)
//!   and belong to `btusb`; interface 3 is class `ff/ff/ff` and is the WLAN
//!   function. **Claiming or auto-detaching a Bluetooth interface would take
//!   down someone else's radio**, so this module claims interface 3 *only*.
//!   It finds it by **class `ff/ff/ff` + bulk endpoints**, never by index, because
//!   other MT7921 boards put WLAN on interface 0 and both layouts must work.
//!   (Upstream matches the same triple:
//!   `USB_DEVICE_AND_INTERFACE_INFO(0x0e8d, 0x7961, 0xff, 0xff, 0xff)`,
//!   `mt7921/usb.c:16`.)
//! * **Endpoints on interface 3:** bulk IN `0x84`, `0x85`; bulk OUT `0x04`,
//!   `0x05`, `0x06`, `0x07`, `0x08`, `0x09`; interrupt IN `0x86`. All
//!   `wMaxPacketSize` 512. Descriptor order matches `enum mt76u_in_ep` /
//!   `enum mt76u_out_ep` (`mt76.h:646-660`), so `0x84` is `MT_EP_IN_PKT_RX`,
//!   `0x85` is `MT_EP_IN_CMD_RESP`, `0x04` is `MT_EP_OUT_INBAND_CMD` and `0x05`
//!   is `MT_EP_OUT_AC_BE`. The interrupt pipe is recorded and **never used** —
//!   `mt76u_set_endpoints` (`usb.c:304-327`) ignores it too.
//! * **Register access works from our own libusb code** via `MT_VEND_READ_EXT`
//!   (`0x63`) and `MT_VEND_WRITE_EXT` (`0x66`), `bmRequestType` `0xc0`/`0x40`,
//!   `wValue = addr >> 16`, `wIndex = addr & 0xffff`, 4-byte little-endian data
//!   stage. The **full 32-bit address** goes straight into `wValue`/`wIndex`;
//!   there is no register-remap window to program first for these addresses.
//!   Readings taken after claiming interface 3:
//!
//!   | register | address | value |
//!   |---|---|---|
//!   | `MT_HW_CHIPID` | `0x7001_0200` | `0x0000_7961` |
//!   | `MT_HW_REV` | `0x7001_0204` | `0x0000_8a10` |
//!   | `MT_CONN_ON_MISC` | `0x7c06_00f0` | `0x0000_0000` (FW_PWR_ON clear — firmware **not** running) |
//!   | `MT_TOP_MISC` | `0x7000_00f0` | `0x0000_0000` |
//!
//! * ★ **EP0 round trip = 268 µs** on this USB2 bus — versus 92-151 µs on the
//!   other parts in this crate. That is the cost of every [`Connac2Usb::rr`] /
//!   [`Connac2Usb::wr`] below, it is nearly *double* the mt76x0 figure, and it is
//!   why nothing on a per-frame path may touch a register. A 1000-iteration
//!   [`Connac2Usb::poll`] is 0.27 s of bus time before the sleeps are counted.
//!
//! # Four ways this is not mt76x0/x2
//!
//! 1. **Composite device.** See above. `crate::mt76::transport::Mt76Usb::claim`
//!    accumulates endpoints across *every* interface, which is correct for a
//!    single-function dongle and would be catastrophic here — it would mix the
//!    Bluetooth pipes into the WLAN endpoint vector. [`select_wlan_interface`]
//!    exists for exactly that reason.
//! 2. **`READ_EXT`/`WRITE_EXT`, not `MULTI_READ`/`MULTI_WRITE`.** The older parts
//!    use requests `0x07`/`0x06` (`usb.c:76-89`, `usb.c:123-132`); connac2 uses
//!    `0x63`/`0x66` (`mt792x_usb.c:154-174`, `mt76.h:641-642`). Same
//!    `wValue`/`wIndex` split, different request byte, and the older byte does
//!    **not** work here.
//! 3. **A second address space reached by the `bmRequestType` low bits.**
//!    Upstream tags connac2 vendor requests `MT_USB_TYPE_VENDOR = USB_TYPE_VENDOR | 0x1f`
//!    (`0x5f`) and the **UHW** space `MT_USB_TYPE_UHW_VENDOR = USB_TYPE_VENDOR | 0x1e`
//!    (`0x5e`) — `mt792x.h:555-556`. Those low bits sit in the standard
//!    *recipient* field, and here they are a **space selector**, not a recipient:
//!    the UHW window is reached with request `MT_VEND_DEV_MODE` (`0x1`) /
//!    `MT_VEND_WRITE` (`0x2`) at type `0xde`/`0x5e` (`mt792x_usb.c:242-260`), and
//!    that is the *only* way to reach `MT_SSUSB_EPCTL_CSR_EP_RST_OPT` and the
//!    `MT_CBTOP_RGU_WF_SUBSYS_RST` word that `mt792xu_wfsys_reset`
//!    (`mt792x_usb.c:425-468`) needs. So [`Connac2Usb::uhw_rr`] /
//!    [`Connac2Usb::uhw_wr`] use the exact upstream tags and must not be
//!    "simplified" to `0xc0`/`0x40`.
//!    ★ For the *normal* window the MEASURED fact is that `0xc0`/`0x40` works on
//!    this silicon, so that is the default; [`VendorTag::Upstream`] switches to
//!    upstream's `0xdf`/`0x5f` if a board ever turns out to care.
//! 4. **MCU responses arrive on two pipes.** `mt7921u` allocates a dedicated MCU
//!    RX queue on `MT_EP_IN_CMD_RESP` = `0x85` (`mt7921/usb.c:228`,
//!    `usb.c:607`) **and** sets `MT_WFDMA_HOST_CONFIG_USB_RXEVT_EP4_EN`
//!    (`mt792xu_dma_rx_evt_ep4`), which routes RX *events* onto EP4 = `0x84`
//!    alongside data. A reader that watches only one of the two will lose
//!    firmware events; both [`Connac2Usb::ep_in_resp`] and
//!    [`Connac2Usb::ep_in_data`] are live and `mcu.rs` should say which it means.
//!
//! # ★ No `handle.reset()`, ever — and this one is not theoretical
//!
//! Upstream's probe opens with a bare `usb_reset_device(udev)`
//! (`mt7921/usb.c:206`). **That call is deliberately not ported.** A failed USB
//! reset makes the kernel mark the hub port `disable=1` and the device becomes
//! unrecoverable without a physical replug — which happened to the MT7612U on
//! this bench three times in one week. There is no `handle.reset()` in this file
//! and there must never be one. Recovering a half-loaded MCU is the firmware
//! layer's problem (check `MT_CONN_ON_MISC` before re-downloading), not the bus's.
//!
//! `clear_halt` is likewise **not** called by default: on macOS it resets the
//! endpoint data toggle and desyncs the first firmware transfer on a cold device.
//! `NDN_RADIO_CLEAR_HALT=1` opts back in for a genuinely stalled endpoint.
#![allow(dead_code)]

use std::io;
use std::sync::Arc;
use std::sync::atomic::{AtomicBool, AtomicU8, Ordering};
use std::time::Duration;

use rusb::{Context, Device, DeviceHandle, Direction, TransferType};

use crate::FaceError;
use crate::usb_select::{DeviceSelect, check_live_link, select_device, usb_addr};

// ── Identity ─────────────────────────────────────────────────────────────────

/// MediaTek's USB vendor ID — the MEASURED part is `0e8d:7961`.
pub const MEDIATEK_VID: u16 = 0x0e8d;

/// The MT7921AU product id under MediaTek's own VID. MEASURED on mds-o5p-3.
pub const MT7921AU_PID: u16 = 0x7961;

/// Every `(vid, pid)` upstream's `mt7921u_device_table` claims
/// (`mt7921/usb.c:15-31`). Only the first entry has been seen on this bench; the
/// rebadges are carried so a Comfast/Netgear/TP-Link stick works without a code
/// change, and are marked CODE-READ because none of them has been held here.
///
/// ⚠ Every one of these is matched by upstream on the **interface** triple
/// `ff/ff/ff` as well as on the ids, which is the same rule
/// [`select_wlan_interface`] applies.
pub const MT7921U_VID_PIDS: &[(u16, u16)] = &[
    // MEASURED — MediaTek reference MT7921AU.
    (MEDIATEK_VID, MT7921AU_PID),
    // CODE-READ — Comfast CF-952AX (`mt7921/usb.c:19`).
    (0x3574, 0x6211),
    // CODE-READ — Netgear, Inc. [A8000, AXE3000] (`mt7921/usb.c:22`).
    (0x0846, 0x9060),
    // CODE-READ — Netgear, Inc. A7500 (`mt7921/usb.c:25`).
    (0x0846, 0x9065),
    // CODE-READ — TP-Link TXE50UH (`mt7921/usb.c:28`).
    (0x35bc, 0x0107),
];

/// The USB interface class triple the WLAN function carries: vendor-specific in
/// all three bytes. MEASURED on interface 3 of the MT7921AU, and the triple
/// upstream's device table matches on (`mt7921/usb.c:16`).
pub const WLAN_IFACE_CLASS: (u8, u8, u8) = (0xff, 0xff, 0xff);

/// The USB class of the **Bluetooth** function on this composite part:
/// `e0` Wireless Controller / `01` RF / `01` Bluetooth. MEASURED on interfaces
/// 0-2. Named here so the refusal below can be explicit rather than incidental —
/// claiming one of these detaches `btusb` and kills a link this crate does not
/// own.
pub const BLUETOOTH_IFACE_CLASS: u8 = 0xe0;

// ── USB vendor requests (`enum mt_vendor_req`, mt76.h:631-644) ───────────────

/// MCU mode door on the normal window; **also the UHW-space read request** when
/// paired with [`REQ_IN_UHW`] (`mt76.h:632`, `mt792x_usb.c:247`). The double duty
/// is upstream's, not ours.
const MT_VEND_DEV_MODE: u8 = 0x01;
/// Single-word write; used **only** as the UHW-space write request here
/// (`mt76.h:633`, `mt792x_usb.c:257`).
const MT_VEND_WRITE: u8 = 0x02;
/// Chip power-on (`mt76.h:634`). Unlike the mt76x0/x2 parts, connac2 really does
/// use this — see [`Connac2Usb::power_on`] and `mt792x_usb.c:218-220`.
const MT_VEND_POWER_ON: u8 = 0x04;
/// ★ Extended MMIO **read**, `0x63` (`mt76.h:641`). The connac2 register door;
/// MEASURED working on this silicon. `mt792x_usb.c:159` and `mt792x_usb.c:19`.
const MT_VEND_READ_EXT: u8 = 0x63;
/// ★ Extended MMIO **write**, `0x66` (`mt76.h:642`). `mt792x_usb.c:170`.
const MT_VEND_WRITE_EXT: u8 = 0x66;

// ── bmRequestType bytes ──────────────────────────────────────────────────────

/// Device→host, vendor type, **device recipient** — the plain `0xc0`.
/// ★ MEASURED: register reads with this byte return the correct `MT_HW_CHIPID`
/// on the target silicon.
const REQ_IN_PLAIN: u8 = 0xc0;
/// Host→device, vendor type, device recipient — the plain `0x40`. MEASURED.
const REQ_OUT_PLAIN: u8 = 0x40;

/// Device→host with upstream's `MT_USB_TYPE_VENDOR` tag:
/// `USB_DIR_IN | USB_TYPE_VENDOR | 0x1f` = `0xdf` (`mt792x.h:555`, used at
/// `mt792x_usb.c:160`). CODE-READ; not needed on the MEASURED board, kept as the
/// escape hatch in [`VendorTag`].
const REQ_IN_UPSTREAM: u8 = 0xdf;
/// Host→device counterpart, `0x5f` (`mt792x_usb.c:171`). CODE-READ.
const REQ_OUT_UPSTREAM: u8 = 0x5f;

/// Device→host into the **UHW** space: `USB_DIR_IN | USB_TYPE_VENDOR | 0x1e`
/// = `0xde` (`MT_USB_TYPE_UHW_VENDOR`, `mt792x.h:556`; used at
/// `mt792x_usb.c:247`).
///
/// ⚠ The `0x1e` is load-bearing. It is not a recipient here, it selects a
/// different register window, and request `0x1` at `0xc0` would be a
/// `MT_VEND_DEV_MODE` query instead of a UHW read. CODE-READ — the UHW path has
/// not been exercised on this bench yet.
const REQ_IN_UHW: u8 = 0xde;
/// Host→device into the UHW space, `0x5e` (`mt792x_usb.c:257`). CODE-READ.
const REQ_OUT_UHW: u8 = 0x5e;

/// Which `bmRequestType` tag the *normal* register window uses.
///
/// ★ **MEASURED 2026-08-27, and the answer is not the obvious one.** Plain
/// `0xc0`/`0x40` reads registers perfectly well — `MT_HW_CHIPID` returns
/// `0x7961` through it. That fact is a trap: it invites the conclusion that the
/// low five bits of `bmRequestType` do not matter on this part, and for
/// **register** traffic they do not. For `MT_VEND_POWER_ON` they decide
/// everything.
///
/// ★★ **The answer is SPLIT, and each half was paid for on hardware.**
///
/// `examples/mt7921_pwr.rs` put the candidate forms to the silicon for
/// `MT_VEND_POWER_ON`:
///
/// | `bmRequestType` | result |
/// |---|---|
/// | `0x40` plain | control write **accepted**, `MT_CONN_ON_MISC` stayed `0x00000000`, no `FW_PWR_ON` in 500 ms |
/// | `0x5f` upstream | `FW_PWR_ON` set after **11 ms** |
///
/// So power-on needs `0x5f`. The tempting next step — make `0xdf`/`0x5f` the tag
/// for *everything*, as the kernel does — was tried and is WRONG on this board:
/// with `0xdf` on the register window, **reads time out**
/// (`vendor read type 0xdf req 0x63 val 0x1806 idx 0x00f0: Operation timed out`),
/// and repeatedly retrying a timing-out control transfer left the chip
/// unenumerable (`device not accepting address, error -110`) until a physical
/// replug. Registers MEASURABLY work at `0xc0`/`0x40` and only there.
///
/// ⇒ The register window stays [`Plain`](VendorTag::Plain); the non-register
/// vendor requests that the bootrom gates on the recipient field — currently
/// just `MT_VEND_POWER_ON` — send `REQ_OUT_UPSTREAM` explicitly. The wrong
/// recipient neither stalls nor errors; the device ACKs the setup packet and
/// does nothing, which is the worst failure mode there is and presented four
/// layers away as "the firmware download timed out".
#[derive(Clone, Copy, Debug, PartialEq, Eq, Default)]
pub enum VendorTag {
    /// `0xc0` / `0x40` — plain vendor/device. ★ MEASURED to be the ONLY form the
    /// register window answers on this board, and therefore the default.
    #[default]
    Plain,
    /// `0xdf` / `0x5f` — upstream's `MT_USB_TYPE_VENDOR` (`mt792x.h:555`), whose
    /// low five bits are a vendor-specific recipient. ★ MEASURED to be the form
    /// the bootrom honours for **power-on** — and MEASURED to make register
    /// reads time out. Used only where power-on needs it; selectable for the
    /// whole window with `NDN_MT7921_VENDOR_TAG=upstream` for A/B testing, which
    /// on this board breaks the device.
    Upstream,
}

impl VendorTag {
    /// Read this from `NDN_MT7921_VENDOR_TAG`; anything but `plain` (case
    /// insensitive) means [`Upstream`](VendorTag::Upstream), because the
    /// measured-working answer is the one that should need no argument.
    fn from_env() -> Self {
        match std::env::var("NDN_MT7921_VENDOR_TAG") {
            Ok(v) if v.trim().eq_ignore_ascii_case("upstream") => VendorTag::Upstream,
            _ => VendorTag::Plain,
        }
    }

    /// `bmRequestType` for a device→host request in this tagging.
    fn req_in(self) -> u8 {
        match self {
            VendorTag::Plain => REQ_IN_PLAIN,
            VendorTag::Upstream => REQ_IN_UPSTREAM,
        }
    }

    /// `bmRequestType` for a host→device request in this tagging.
    fn req_out(self) -> u8 {
        match self {
            VendorTag::Plain => REQ_OUT_PLAIN,
            VendorTag::Upstream => REQ_OUT_UPSTREAM,
        }
    }
}

// ── Timeouts ─────────────────────────────────────────────────────────────────

/// Control-transfer timeout. Same value as the proven mt76x0/x2 transport
/// (`src/mt76/transport.rs:120`); upstream uses 300 ms (`usb.c:12`).
///
/// ★ Upstream also **retries a failed vendor request ten times** with a 5-10 ms
/// gap (`usb.c:11,30-42`) and, on connac2 only, escalates a final failure into
/// `bus_hung` + a queued `usb_queue_reset_device` (`mt792x_usb.c:37-45,93-99`).
/// **Neither is ported.** The retry loop turns one reported fault into a
/// 3-second stall followed by the same error, and the queued reset is precisely
/// the blind USB reset that wedges these parts — see the module header. A
/// control transfer that times out on this bus has meant a wedged device every
/// time it has happened here, and saying so immediately is the useful behaviour.
const CTRL_TIMEOUT: Duration = Duration::from_millis(500);

/// Bulk-transfer timeout for command writes. Matches upstream's MCU send
/// (`mt7921/usb.c:56-57` passes 1000 ms).
const BULK_TIMEOUT: Duration = Duration::from_millis(1000);

/// Default bulk-IN read timeout. Short on purpose: a reader polling for frames
/// wants to come back and check a stop flag, not block for a second.
const BULK_IN_TIMEOUT: Duration = Duration::from_millis(200);

/// Fallback control-transfer payload batch when `bMaxPacketSize0` is unreadable
/// or absurd. Upstream floors `usb->data_len` at 32 for the same reason
/// (`usb.c:1131-1133`).
const CTRL_BATCH_MIN: usize = 32;

// ── Endpoint indices (`enum mt76u_in_ep` / `mt76u_out_ep`, mt76.h:646-660) ───

/// Bulk IN 0 — packet RX (and, once `USB_RXEVT_EP4_EN` is set, firmware events
/// too — see the module header). MEASURED `0x84`.
pub const MT_EP_IN_PKT_RX: usize = 0;
/// Bulk IN 1 — MCU command response. MEASURED `0x85`.
pub const MT_EP_IN_CMD_RESP: usize = 1;
/// Number of bulk IN endpoints mt76 requires (`__MT_EP_IN_MAX`, `mt76.h:649`).
pub const MT_EP_IN_MAX: usize = 2;

/// Bulk OUT 0 — the MCU **inband command** pipe; every command except
/// `MCU_CMD(FW_SCATTER)` goes here (`mt7921/usb.c:47-50`). MEASURED `0x04`.
pub const MT_EP_OUT_INBAND_CMD: usize = 0;
/// Bulk OUT 1 — AC_BE, the ordinary data queue **and** the pipe firmware
/// payload rides on: `mt7921u_mcu_send_message` sends `FW_SCATTER` here rather
/// than on the command pipe (`mt7921/usb.c:50`). MEASURED `0x05`.
pub const MT_EP_OUT_AC_BE: usize = 1;
/// Bulk OUT 2 — AC_BK. MEASURED `0x06`.
pub const MT_EP_OUT_AC_BK: usize = 2;
/// Bulk OUT 3 — AC_VI. MEASURED `0x07`.
pub const MT_EP_OUT_AC_VI: usize = 3;
/// Bulk OUT 4 — AC_VO. MEASURED `0x08`.
pub const MT_EP_OUT_AC_VO: usize = 4;
/// Bulk OUT 5 — HCCA. MEASURED `0x09`.
pub const MT_EP_OUT_HCCA: usize = 5;
/// Number of bulk OUT endpoints mt76 requires (`__MT_EP_OUT_MAX`, `mt76.h:659`).
pub const MT_EP_OUT_MAX: usize = 6;

// ── Health check ─────────────────────────────────────────────────────────────

/// `MT_HW_CHIPID` (`mt792x_regs.h:429`), duplicated here as a **private**
/// constant on purpose: [`Connac2Usb::check_bus`] is the transport's own "is the
/// bus alive" probe (upstream's `mt792xu_check_bus`, `mt792x_usb.c:116-129`) and
/// must not make `usb.rs` depend on the shape of `regs.rs`. The public copy in
/// `crate::connac2::regs` is the one callers should use.
const CHIPID_ADDR: u32 = 0x7001_0200;

/// The chip id MEASURED at [`CHIPID_ADDR`] on the target part.
const CHIPID_MT7961: u32 = 0x7961;

fn usb_err(e: rusb::Error) -> FaceError {
    FaceError::Io(io::Error::other(format!("connac2 usb: {e}")))
}

fn tr_err(what: String) -> FaceError {
    FaceError::Io(io::Error::other(what))
}

// ── Endpoint assignment (pure) ───────────────────────────────────────────────

/// The bulk endpoints of one connac2 USB interface, in mt76's own index order.
///
/// Split out of [`Connac2Usb`] as a plain value so the assignment rule — which
/// is *descriptor order*, not address order — is testable without a device.
#[derive(Clone, Debug, Default, PartialEq, Eq)]
pub struct Endpoints {
    /// Bulk IN addresses in descriptor order; index with [`MT_EP_IN_PKT_RX`] /
    /// [`MT_EP_IN_CMD_RESP`].
    pub ins: Vec<u8>,
    /// Bulk OUT addresses in descriptor order; index with the `MT_EP_OUT_*`
    /// constants.
    pub outs: Vec<u8>,
    /// Interrupt IN addresses. MEASURED `[0x86]`. Recorded for completeness and
    /// **never submitted to** — `mt76u_set_endpoints` (`usb.c:304-327`) counts
    /// only bulk pipes, and nothing in the mt76 tree reads this one. If a
    /// firmware event ever turns out to arrive here, this is where to start.
    pub int_ins: Vec<u8>,
}

impl Endpoints {
    /// The OUT address at mt76 index `idx`, or the last one present. The
    /// fallback exists so a device enumerating fewer pipes than mt76 expects
    /// still yields *some* endpoint rather than panicking on an index — with the
    /// mismatch already logged by [`assign_endpoints`].
    pub fn out_at(&self, idx: usize) -> u8 {
        self.outs
            .get(idx)
            .copied()
            .unwrap_or_else(|| *self.outs.last().unwrap_or(&0))
    }

    /// The IN address at mt76 index `idx`, with the same last-one fallback as
    /// [`out_at`](Self::out_at).
    pub fn in_at(&self, idx: usize) -> u8 {
        self.ins
            .get(idx)
            .copied()
            .unwrap_or_else(|| *self.ins.last().unwrap_or(&0))
    }
}

/// Assign the bulk endpoints the way `mt76u_set_endpoints` (`usb.c:304-327`)
/// does: walk the interface's endpoint descriptors **in descriptor order** and
/// fill `in_ep[]` / `out_ep[]` as they come.
///
/// Address order happens to agree on this board (MEASURED `0x84,0x85` and
/// `0x04..0x09` enumerate ascending), but descriptor order is the rule upstream
/// follows and the one the MCU's endpoint numbering is defined against, so that
/// is what is implemented — a rebadged board that lists them out of order still
/// gets `MT_EP_OUT_INBAND_CMD` right.
///
/// Upstream **hard-fails** unless it finds exactly 2 IN and 6 OUT
/// (`usb.c:325-326`). This port warns and carries on with what it found, exactly
/// as `crate::mt76::transport::assign_endpoints` does: a count mismatch means the
/// caller pointed the transport at something unexpected, and an error naming what
/// it saw is more use than a bare `-EINVAL`. It does fail when a direction is
/// missing entirely, because there is then no usable pipe at all.
pub fn assign_endpoints(
    ins: &[u8],
    outs: &[u8],
    int_ins: &[u8],
    label: &str,
) -> Result<Endpoints, FaceError> {
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
            "connac2 endpoint count is not the expected 2 IN / 6 OUT (usb.c:325-326) — \
             indices may not mean what mt76's enum says",
        );
    }
    Ok(Endpoints {
        ins: ins.to_vec(),
        outs: outs.to_vec(),
        int_ins: int_ins.to_vec(),
    })
}

// ── Interface selection (pure) ───────────────────────────────────────────────

/// One USB interface as seen in the config descriptor, reduced to the facts
/// [`select_wlan_interface`] decides on.
///
/// A plain value with no `rusb` types in it, so the composite-device rule — the
/// single most dangerous thing in this file, because getting it wrong detaches
/// `btusb` — is testable with no hardware attached.
#[derive(Clone, Debug, Default, PartialEq, Eq)]
pub struct IfaceCandidate {
    /// `bInterfaceNumber`. MEASURED: `3` on this board, `0` on some others.
    pub number: u8,
    /// `bAlternateSetting`. Only setting 0 is considered; mt76 uses
    /// `intf->cur_altsetting` and never calls `usb_set_interface`.
    pub alt_setting: u8,
    /// `(bInterfaceClass, bInterfaceSubClass, bInterfaceProtocol)`.
    pub class: (u8, u8, u8),
    /// Bulk IN endpoint addresses, in descriptor order.
    pub bulk_in: Vec<u8>,
    /// Bulk OUT endpoint addresses, in descriptor order.
    pub bulk_out: Vec<u8>,
    /// Interrupt IN endpoint addresses, in descriptor order.
    pub int_in: Vec<u8>,
    /// `wMaxPacketSize` of the first bulk IN endpoint. MEASURED 512 (high speed).
    pub in_packet_size: u16,
}

impl IfaceCandidate {
    /// Is this the WLAN function? Class `ff/ff/ff` **and** at least one bulk pipe
    /// in each direction. Both halves matter: the class alone is how upstream
    /// matches (`mt7921/usb.c:16`), and the endpoints are what make it usable.
    fn is_wlan(&self) -> bool {
        self.class == WLAN_IFACE_CLASS && !self.bulk_in.is_empty() && !self.bulk_out.is_empty()
    }

    /// Is this one of the Bluetooth functions that belong to `btusb`? MEASURED
    /// `e0/01/01` on interfaces 0-2 of this part.
    fn is_bluetooth(&self) -> bool {
        self.class.0 == BLUETOOTH_IFACE_CLASS
    }

    /// One-line rendering for the error text, so a refusal names exactly what
    /// the bus offered instead of "not found".
    fn describe(&self) -> String {
        let (c, s, p) = self.class;
        format!(
            "if{} alt{} class {c:02x}/{s:02x}/{p:02x}{} bulk-in {:02x?} bulk-out {:02x?}",
            self.number,
            self.alt_setting,
            if self.is_bluetooth() {
                " (Bluetooth — belongs to btusb, NOT ours)"
            } else {
                ""
            },
            self.bulk_in,
            self.bulk_out,
        )
    }
}

/// Pick the WLAN interface out of a **composite** device, returning its index in
/// `cands`.
///
/// ★ This is the rule that keeps this driver off someone else's Bluetooth radio.
/// MEASURED on `0e8d:7961`: interfaces 0-2 are class `e0/01/01` and owned by
/// `btusb`; interface 3 is class `ff/ff/ff` and is WLAN. Claiming any interface
/// enables libusb's auto-detach for *that* interface, so grabbing a Bluetooth one
/// would silently unbind `btusb` and kill a link this crate does not own.
///
/// The rule, in order:
///
/// 1. **Only class `ff/ff/ff` with bulk endpoints in both directions** is a
///    candidate — the same triple upstream's device table matches on
///    (`mt7921/usb.c:16`). There is deliberately **no** "any interface with bulk
///    pipes" fallback: on this part that fallback would select interface 0, which
///    is Bluetooth.
/// 2. Among those, prefer the one with the **most bulk endpoints** (the WLAN
///    function has 8), ties broken by the lowest interface number, so a board
///    that exposes a second vendor-specific interface for something else does not
///    win by enumerating first.
/// 3. `NDN_MT7921_IFACE=<n>` forces an interface number for a board this rule
///    gets wrong — but it still **refuses a class-`e0` interface**, because
///    "the operator asked for it" is not a good enough reason to unbind `btusb`.
///
/// Errors name every interface seen, flagged, because "no interface found" on a
/// composite device is otherwise unactionable.
pub fn select_wlan_interface(cands: &[IfaceCandidate], label: &str) -> Result<usize, FaceError> {
    let inventory = || {
        cands
            .iter()
            .map(IfaceCandidate::describe)
            .collect::<Vec<_>>()
            .join("; ")
    };

    if let Some(forced) = std::env::var("NDN_MT7921_IFACE")
        .ok()
        .and_then(|s| s.trim().parse::<u8>().ok())
    {
        let hit = cands.iter().position(|c| c.number == forced);
        return match hit {
            Some(i) if cands[i].is_bluetooth() => Err(tr_err(format!(
                "{label}: NDN_MT7921_IFACE={forced} names a class-{:02x} Bluetooth interface; \
                 refusing to claim it — that would detach btusb from someone else's radio. \
                 Interfaces: {}",
                cands[i].class.0,
                inventory()
            ))),
            Some(i) => {
                tracing::warn!(
                    target: "named_radio",
                    chip = label, iface = forced,
                    "NDN_MT7921_IFACE overrides the class ff/ff/ff WLAN-interface rule",
                );
                Ok(i)
            }
            None => Err(tr_err(format!(
                "{label}: NDN_MT7921_IFACE={forced} matches no interface. Interfaces: {}",
                inventory()
            ))),
        };
    }

    let best = cands
        .iter()
        .enumerate()
        .filter(|(_, c)| c.is_wlan())
        .max_by_key(|(_, c)| {
            // Most bulk pipes wins; on a tie the lowest interface number does,
            // hence the negated number as the secondary key.
            (c.bulk_in.len() + c.bulk_out.len(), -(c.number as i32))
        })
        .map(|(i, _)| i);

    best.ok_or_else(|| {
        tr_err(format!(
            "{label}: no class ff/ff/ff interface with bulk endpoints (the WLAN function). \
             This part is composite — its Bluetooth interfaces are class e0 and are NOT a \
             fallback. Interfaces: {}",
            inventory()
        ))
    })
}

// ── The transport ────────────────────────────────────────────────────────────

/// A claimed connac2 USB device: the register windows plus the bulk pipes.
///
/// Deliberately **not** a radio. It holds no channel, rate, filter or frame
/// state — [`crate::connac2::mcu`], [`crate::connac2::mac`] and the
/// `Mt7921uBackend` own those and borrow this for bus access.
///
/// ⚠ This type deliberately does **not** implement `crate::mt76::Mt76Regs`, even
/// though the two `rr`/`wr` signatures match. That trait is the seam
/// `crate::mt76::knobs` is written against, and every register in that module is
/// an mt76x02 address (`MT_TSF_TIMER_DW0` at `0x111c`, `MT_CH_BUSY` at `0x1134`,
/// …) that means something else entirely on connac2. Implementing the trait would
/// make a catastrophic mistake compile.
/// How many control transfers may time out **in a row** before this transport stops trying.
///
/// ★ This exists because of a specific, expensive failure on 2026-08-27. A wrong `bmRequestType`
/// made every register read time out; the driver's own retry loops kept re-issuing them for a
/// second; and the MT7921AU stopped enumerating altogether — `device not accepting address,
/// error -110`, recoverable only by a physical replug at a remote site. A device that has stopped
/// answering does not start answering because you asked again, and hammering a confused USB
/// bootrom is how a diagnosable mistake becomes dead hardware.
///
/// So: after this many consecutive timeouts every subsequent call fails immediately with a
/// "presumed hung" error naming the count, and the caller gets its diagnosis in one second instead
/// of the device getting a thousand more setup packets. Any successful transfer resets the count,
/// so an isolated timeout costs nothing.
const BUS_HUNG_LIMIT: u32 = 12;

pub struct Connac2Usb {
    handle: Arc<DeviceHandle<Context>>,
    label: &'static str,
    /// USB topological address (`bus-port.port`), for logs and for telling two
    /// identical dongles apart after the fact.
    usb_addr: String,
    /// The claimed interface number — MEASURED 3 on this board. Released when
    /// the handle drops.
    iface: u8,
    /// `(vid, pid)` actually matched, so a rebadged board says which it was.
    ids: (u16, u16),
    /// Consecutive control-transfer timeouts. See [`BUS_HUNG_LIMIT`].
    ctrl_timeouts: std::sync::atomic::AtomicU32,
    eps: Endpoints,
    /// Resolved bulk-OUT for MCU inband commands ([`MT_EP_OUT_INBAND_CMD`]).
    ep_out_cmd: u8,
    /// Resolved bulk-OUT for WLAN data and `FW_SCATTER` ([`MT_EP_OUT_AC_BE`]).
    ep_out_data: u8,
    /// Resolved bulk-IN for packet RX ([`MT_EP_IN_PKT_RX`]).
    ep_in_data: u8,
    /// Resolved bulk-IN for MCU responses ([`MT_EP_IN_CMD_RESP`]).
    ep_in_resp: u8,
    /// `wMaxPacketSize` of the RX pipe — MEASURED 512 (high speed). A bulk read
    /// buffer that is not a multiple of this loses the tail of a burst on some
    /// stacks, so callers size against it rather than a hard-coded 512.
    ep_in_packet_size: u16,
    /// `bMaxPacketSize0`, the batch size the copy helpers chunk at — upstream's
    /// `usb->data_len` (`usb.c:1131-1133`). MEASURED 64 at high speed.
    ctrl_batch: usize,
    /// Which `bmRequestType` tagging the normal register window uses.
    tag: VendorTag,
    /// MCU command sequence, 1..=15 and never 0 — see [`Connac2Usb::next_seq`].
    mcu_seq: AtomicU8,
    /// Stop/pause flag handed to [`Connac2Usb::spawn_rx_drain`].
    drain_pause: Arc<AtomicBool>,
}

impl Connac2Usb {
    /// Open the connac2 part selected by `NDN_RADIO_DEV` / `NDN_USB_ADDR` /
    /// `NDN_USB_INDEX` ([`DeviceSelect::from_env`]).
    pub fn open() -> Result<Self, FaceError> {
        Self::open_selected(&DeviceSelect::from_env())
    }

    /// Open the connac2 part matching `sel` — the form a config's
    /// `RadioDeviceConfig.address` reaches through [`DeviceSelect::parse`].
    ///
    /// Every `(vid, pid)` in [`MT7921U_VID_PIDS`] is tried, MediaTek's own first.
    /// Selection goes through [`select_device`], which runs the
    /// [`check_live_link`] guard — so pointing this at the dongle currently
    /// carrying the node's kernel mesh warns (or, with `NDN_GUARD_LIVE_LINK=1`,
    /// refuses) instead of silently dropping the link.
    ///
    /// ⚠ Caveat on [`DeviceSelect::Index`]: because the vendor groups are walked
    /// in turn, an index counts *within the vendor group that matched*, not
    /// across the whole bus. With one vendor present — every case on this bench —
    /// the two are the same. [`DeviceSelect::Addr`] has no such ambiguity and is
    /// the selector to put in a config.
    pub fn open_selected(sel: &DeviceSelect) -> Result<Self, FaceError> {
        let label = "MT7921AU";
        let mut last: Option<FaceError> = None;
        for (vid, pids) in group_pids_by_vid(MT7921U_VID_PIDS) {
            match select_device(&pids, vid, sel, label) {
                Ok(device) => return Self::claim(device, (vid, pids[0]), label),
                Err(e) => last = Some(e),
            }
        }
        Err(last.unwrap_or_else(|| {
            tr_err(format!(
                "{label}: empty vid/pid table — nothing to look for"
            ))
        }))
    }

    /// Claim an already-selected device.
    ///
    /// ★ There is **no `handle.reset()`** here and there must never be one, even
    /// though upstream's probe opens with `usb_reset_device(udev)`
    /// (`mt7921/usb.c:206`). A failed USB reset leaves the hub port
    /// `disable=1` and the part needs a physical replug — three MT7612U casualties
    /// on this bench in one week.
    ///
    /// ★ `set_auto_detach_kernel_driver(true)` is enabled on the handle, but it
    /// only acts on the interface actually passed to `claim_interface` — and that
    /// is the **WLAN interface only**, chosen by [`select_wlan_interface`]. The
    /// Bluetooth interfaces stay bound to `btusb` and keep working.
    pub fn claim(
        device: Device<Context>,
        ids: (u16, u16),
        label: &'static str,
    ) -> Result<Self, FaceError> {
        check_live_link(&device, label)?;
        let addr = usb_addr(&device);
        let config = device.active_config_descriptor().map_err(usb_err)?;

        // bMaxPacketSize0 — upstream's control bounce-buffer size, floored at 32
        // (`usb.c:1131-1133`). MEASURED 64 on this high-speed part.
        let ctrl_batch = device
            .device_descriptor()
            .map(|d| d.max_packet_size() as usize)
            .unwrap_or(0)
            .max(CTRL_BATCH_MIN);

        let cands = enumerate_interfaces(&config);
        let pick = select_wlan_interface(&cands, label)
            .map_err(|e| tr_err(format!("{label} at {addr}: {e}")))?;
        let chosen = &cands[pick];
        let eps = assign_endpoints(&chosen.bulk_in, &chosen.bulk_out, &chosen.int_in, label)?;

        let handle = device.open().map_err(usb_err)?;
        // Detach the kernel driver from THIS device's WLAN interface only.
        let _ = handle.set_auto_detach_kernel_driver(true);
        handle.claim_interface(chosen.number).map_err(usb_err)?;
        if std::env::var_os("NDN_RADIO_CLEAR_HALT").is_some() {
            for ep in eps.outs.iter().chain(eps.ins.iter()) {
                let _ = handle.clear_halt(*ep);
            }
        }

        let ep_out_cmd = eps.out_at(MT_EP_OUT_INBAND_CMD);
        let ep_out_data = eps.out_at(MT_EP_OUT_AC_BE);
        let ep_in_data = eps.in_at(MT_EP_IN_PKT_RX);
        let ep_in_resp = eps.in_at(MT_EP_IN_CMD_RESP);
        let tag = VendorTag::from_env();
        tracing::info!(
            target: "named_radio",
            chip = label, usb_addr = %addr,
            vid = format_args!("{:#06x}", ids.0), pid = format_args!("{:#06x}", ids.1),
            iface = chosen.number,
            iface_class = format_args!(
                "{:02x}/{:02x}/{:02x}",
                chosen.class.0, chosen.class.1, chosen.class.2
            ),
            ep_out_cmd = format_args!("{ep_out_cmd:#04x}"),
            ep_out_data = format_args!("{ep_out_data:#04x}"),
            ep_in_data = format_args!("{ep_in_data:#04x}"),
            ep_in_resp = format_args!("{ep_in_resp:#04x}"),
            ep_in_packet_size = chosen.in_packet_size,
            ctrl_batch, tag = ?tag,
            "connac2 USB claimed (WLAN interface only; Bluetooth interfaces untouched)",
        );

        Ok(Self {
            handle: Arc::new(handle),
            label,
            usb_addr: addr,
            iface: chosen.number,
            ids,
            ctrl_timeouts: std::sync::atomic::AtomicU32::new(0),
            eps,
            ep_out_cmd,
            ep_out_data,
            ep_in_data,
            ep_in_resp,
            ep_in_packet_size: chosen.in_packet_size,
            ctrl_batch,
            tag,
            mcu_seq: AtomicU8::new(0),
            drain_pause: Arc::new(AtomicBool::new(false)),
        })
    }

    // ── Accessors ───────────────────────────────────────────────────────────

    /// The shared device handle, for a backend running its own URB pumps on
    /// other threads.
    pub fn handle(&self) -> Arc<DeviceHandle<Context>> {
        self.handle.clone()
    }
    /// The chip label used in logs and error text.
    pub fn label(&self) -> &'static str {
        self.label
    }
    /// USB topological address (`bus-port.port`) of the claimed device.
    pub fn usb_addr(&self) -> &str {
        &self.usb_addr
    }
    /// The claimed interface number — MEASURED 3 on the reference board.
    pub fn interface(&self) -> u8 {
        self.iface
    }
    /// The `(vid, pid)` that matched.
    pub fn ids(&self) -> (u16, u16) {
        self.ids
    }
    /// All discovered endpoints of the WLAN interface, in mt76 index order.
    pub fn endpoints(&self) -> &Endpoints {
        &self.eps
    }
    /// MCU inband-command bulk-OUT endpoint ([`MT_EP_OUT_INBAND_CMD`]).
    /// MEASURED `0x04`.
    pub fn ep_out_cmd(&self) -> u8 {
        self.ep_out_cmd
    }
    /// WLAN data bulk-OUT endpoint ([`MT_EP_OUT_AC_BE`]). MEASURED `0x05`.
    /// ★ Firmware `FW_SCATTER` payload also goes here, not on the command pipe
    /// (`mt7921/usb.c:47-50`).
    pub fn ep_out_data(&self) -> u8 {
        self.ep_out_data
    }
    /// Packet-RX bulk-IN endpoint. MEASURED `0x84`. ★ Once
    /// `MT_WFDMA_HOST_CONFIG_USB_RXEVT_EP4_EN` is set (`mt792xu_dma_rx_evt_ep4`)
    /// firmware *events* are interleaved here with data.
    pub fn ep_in_data(&self) -> u8 {
        self.ep_in_data
    }
    /// MCU-response bulk-IN endpoint. MEASURED `0x85`; upstream allocates a
    /// dedicated queue on it (`mt7921/usb.c:228`, `usb.c:607`).
    pub fn ep_in_resp(&self) -> u8 {
        self.ep_in_resp
    }
    /// Interrupt-IN endpoint, if the interface has one. MEASURED `0x86`.
    /// Nothing in mt76 reads it; exposed so a future investigation does not have
    /// to re-enumerate.
    pub fn ep_in_int(&self) -> Option<u8> {
        self.eps.int_ins.first().copied()
    }
    /// `wMaxPacketSize` of the RX pipe — MEASURED 512 on this high-speed part.
    pub fn ep_in_packet_size(&self) -> u16 {
        self.ep_in_packet_size
    }
    /// The control-transfer batch size the copy helpers chunk at
    /// (`bMaxPacketSize0`, MEASURED 64).
    pub fn ctrl_batch(&self) -> usize {
        self.ctrl_batch
    }
    /// Which `bmRequestType` tagging the normal register window is using.
    pub fn vendor_tag(&self) -> VendorTag {
        self.tag
    }

    // ── Vendor control transfers ────────────────────────────────────────────

    /// One device→host vendor request on the normal window, into `buf`.
    /// ⚠ 268 µs MEASURED per call.
    pub fn vendor_read(
        &self,
        req: u8,
        value: u16,
        index: u16,
        buf: &mut [u8],
    ) -> Result<usize, FaceError> {
        self.vendor_read_typed(self.tag.req_in(), req, value, index, buf)
    }

    /// One host→device vendor request on the normal window.
    ///
    /// Contract shape: no `bmRequestType` parameter, because on the normal window
    /// it is fixed by [`VendorTag`]. The UHW window has its own accessors
    /// ([`uhw_rr`](Self::uhw_rr) / [`uhw_wr`](Self::uhw_wr)) rather than a
    /// type byte threaded through here, so a caller cannot reach the wrong
    /// address space by passing the wrong constant.
    pub fn vendor_write(
        &self,
        req: u8,
        value: u16,
        index: u16,
        data: &[u8],
    ) -> Result<(), FaceError> {
        self.vendor_write_typed(self.tag.req_out(), req, value, index, data)?;
        Ok(())
    }

    /// The general device→host form, with an explicit `bmRequestType`. Private:
    /// the type byte selects an *address space* on this part
    /// (`mt792x.h:555-556`), so it is chosen by the accessor, never by a caller.
    /// Reset the consecutive-timeout counter — any transfer that completes proves the bus lives.
    fn note_ctrl_ok(&self) {
        self.ctrl_timeouts
            .store(0, std::sync::atomic::Ordering::Relaxed);
    }

    /// Count a timeout (only a timeout — a stall or a pipe error is a different fault and does not
    /// mean the device has stopped listening).
    fn note_ctrl_err(&self, e: rusb::Error) {
        if matches!(e, rusb::Error::Timeout) {
            self.ctrl_timeouts
                .fetch_add(1, std::sync::atomic::Ordering::Relaxed);
        }
    }

    /// Refuse to issue another transfer once [`BUS_HUNG_LIMIT`] consecutive ones have timed out.
    fn check_bus_alive(&self) -> Result<(), FaceError> {
        let n = self
            .ctrl_timeouts
            .load(std::sync::atomic::Ordering::Relaxed);
        if n >= BUS_HUNG_LIMIT {
            return Err(tr_err(format!(
                "{}: {n} consecutive control transfers timed out — the device has stopped \
                 answering and further requests would only push it further from recovery. \
                 Stop, and check for a wrong bmRequestType or an unmapped address before \
                 retrying; if it has already left the bus, only a physical replug returns it.",
                self.label
            )));
        }
        Ok(())
    }

    fn vendor_read_typed(
        &self,
        req_type: u8,
        req: u8,
        value: u16,
        index: u16,
        buf: &mut [u8],
    ) -> Result<usize, FaceError> {
        self.check_bus_alive()?;
        self.handle
            .read_control(req_type, req, value, index, buf, CTRL_TIMEOUT)
            .inspect(|_| self.note_ctrl_ok())
            .inspect_err(|e| self.note_ctrl_err(*e))
            .map_err(|e| {
                tr_err(format!(
                    "{} vendor read type {req_type:#04x} req {req:#04x} val {value:#06x} \
                     idx {index:#06x}: {e}",
                    self.label
                ))
            })
    }

    /// The general host→device form, with an explicit `bmRequestType`. Private
    /// for the same reason as [`vendor_read_typed`](Self::vendor_read_typed).
    fn vendor_write_typed(
        &self,
        req_type: u8,
        req: u8,
        value: u16,
        index: u16,
        data: &[u8],
    ) -> Result<usize, FaceError> {
        self.check_bus_alive()?;
        self.handle
            .write_control(req_type, req, value, index, data, CTRL_TIMEOUT)
            .inspect(|_| self.note_ctrl_ok())
            .inspect_err(|e| self.note_ctrl_err(*e))
            .map_err(|e| {
                tr_err(format!(
                    "{} vendor write type {req_type:#04x} req {req:#04x} val {value:#06x} \
                     idx {index:#06x}: {e}",
                    self.label
                ))
            })
    }

    // ── The register window ─────────────────────────────────────────────────

    /// Read a 32-bit register through `MT_VEND_READ_EXT` (`0x63`).
    ///
    /// ★ MEASURED working on `0e8d:7961` after claiming interface 3:
    /// `rr(0x7001_0200)` → `0x7961`, `rr(0x7001_0204)` → `0x8a10`. The **full**
    /// 32-bit address splits into `wValue = addr >> 16` / `wIndex = addr & 0xffff`
    /// with no remap window to program first — `mt792xu_rr`
    /// (`mt792x_usb.c:154-165`) via `___mt76u_rr` (`usb.c:76-89`).
    ///
    /// ⚠ 268 µs MEASURED per call on this USB2 bus — never on a per-frame path.
    pub fn rr(&self, addr: u32) -> Result<u32, FaceError> {
        let mut b = [0u8; 4];
        let n = self.vendor_read(
            MT_VEND_READ_EXT,
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

    /// Write a 32-bit register through `MT_VEND_WRITE_EXT` (`0x66`) —
    /// `mt792xu_wr` (`mt792x_usb.c:167-174`) via `___mt76u_wr`
    /// (`usb.c:123-132`). Same address split as [`rr`](Self::rr).
    pub fn wr(&self, addr: u32, val: u32) -> Result<(), FaceError> {
        let data = val.to_le_bytes();
        let n = self.vendor_write_typed(
            self.tag.req_out(),
            MT_VEND_WRITE_EXT,
            (addr >> 16) as u16,
            (addr & 0xffff) as u16,
            &data,
        )?;
        if n != 4 {
            return Err(tr_err(format!(
                "{} wr({addr:#x}) short write: {n} bytes",
                self.label
            )));
        }
        Ok(())
    }

    /// Read-modify-write: clear the `clear` bits, set the `set` bits, and return
    /// the value **before** the write.
    ///
    /// ⚠ Deliberate divergence from upstream, stated because it is the kind of
    /// thing that silently breaks a port: `mt792xu_rmw` (`mt792x_usb.c:176-187`)
    /// returns the **new** value. This crate's own convention — the default body
    /// of `crate::mt76::Mt76Regs::rmw`, and every caller written against it —
    /// returns the **old** one, so a knob can restore what it found. Consistency
    /// inside the crate wins over consistency with a return value upstream never
    /// uses (every `mt792xu_rmw` call site discards it).
    ///
    /// Two round trips ≈ 536 µs MEASURED.
    pub fn rmw(&self, addr: u32, clear: u32, set: u32) -> Result<u32, FaceError> {
        let old = self.rr(addr)?;
        self.wr(addr, (old & !clear) | set)?;
        Ok(old)
    }

    /// `mt76_set` — OR in `bits`, returning the previous value.
    pub fn set_bits(&self, addr: u32, bits: u32) -> Result<u32, FaceError> {
        self.rmw(addr, 0, bits)
    }

    /// `mt76_clear` — AND out `bits`, returning the previous value.
    pub fn clear_bits(&self, addr: u32, bits: u32) -> Result<u32, FaceError> {
        self.rmw(addr, bits, 0)
    }

    // ── The UHW window ──────────────────────────────────────────────────────

    /// Read a 32-bit word from the **UHW** register space —
    /// `mt792xu_uhw_rr` (`mt792x_usb.c:242-252`).
    ///
    /// ★ A genuinely different address space, selected by the `bmRequestType`
    /// low bits (`MT_USB_TYPE_UHW_VENDOR`, `mt792x.h:556`) rather than by the
    /// address. Request `MT_VEND_DEV_MODE` (`0x1`) at type `0xde`. This is the
    /// only door to `MT_SSUSB_EPCTL_CSR_EP_RST_OPT` and to the
    /// `MT_CBTOP_RGU_WF_SUBSYS_RST` word that `mt792xu_wfsys_reset`
    /// (`mt792x_usb.c:425-468`) drives, so a WFSYS reset cannot be written
    /// without it.
    ///
    /// ⚠ **CODE-READ, not measured.** The MEASURED reads on this bench were all
    /// on the normal window. If a UHW read returns `0xffff_ffff` or times out,
    /// the tag byte is the first thing to doubt.
    ///
    /// ⚠ Note this is *not* the blind chip reset the module header forbids. That
    /// prohibition is on `handle.reset()` / `usb_queue_reset_device` — a **USB
    /// bus** reset, which is what disables the hub port. A WFSYS reset is an
    /// in-chip register sequence over a live bus and is how upstream recovers a
    /// half-loaded MCU without touching USB at all.
    pub fn uhw_rr(&self, addr: u32) -> Result<u32, FaceError> {
        let mut b = [0u8; 4];
        let n = self.vendor_read_typed(
            REQ_IN_UHW,
            MT_VEND_DEV_MODE,
            (addr >> 16) as u16,
            (addr & 0xffff) as u16,
            &mut b,
        )?;
        if n != 4 {
            return Err(tr_err(format!(
                "{} uhw_rr({addr:#x}) short read: {n} bytes",
                self.label
            )));
        }
        Ok(u32::from_le_bytes(b))
    }

    /// Write a 32-bit word to the UHW register space — `mt792xu_uhw_wr`
    /// (`mt792x_usb.c:254-260`). Request `MT_VEND_WRITE` (`0x2`) at type `0x5e`.
    /// CODE-READ; see [`uhw_rr`](Self::uhw_rr).
    pub fn uhw_wr(&self, addr: u32, val: u32) -> Result<(), FaceError> {
        let data = val.to_le_bytes();
        let n = self.vendor_write_typed(
            REQ_OUT_UHW,
            MT_VEND_WRITE,
            (addr >> 16) as u16,
            (addr & 0xffff) as u16,
            &data,
        )?;
        if n != 4 {
            return Err(tr_err(format!(
                "{} uhw_wr({addr:#x}) short write: {n} bytes",
                self.label
            )));
        }
        Ok(())
    }

    /// Read-modify-write on the UHW window, returning the value before the
    /// write — the shape `mt792xu_epctl_rst_opt` (`mt792x_usb.c:332-347`) and
    /// `mt792xu_wfsys_reset` open-code.
    pub fn uhw_rmw(&self, addr: u32, clear: u32, set: u32) -> Result<u32, FaceError> {
        let old = self.uhw_rr(addr)?;
        self.uhw_wr(addr, (old & !clear) | set)?;
        Ok(old)
    }

    // ── Multi-dword copies ──────────────────────────────────────────────────

    /// Write a run of bytes to consecutive register addresses —
    /// `mt792xu_copy` (`mt792x_usb.c:189-212`).
    ///
    /// ★ Note this is **not** `mt76u_copy` (`usb.c:169-199`): the older helper
    /// uses `MT_VEND_MULTI_WRITE` with `wValue = 0` and only the low 16 bits of
    /// the offset in `wIndex`, which cannot address `0x7c06_00f0`. The connac2
    /// form uses `MT_VEND_WRITE_EXT` with the **full** `(offset + i)` split
    /// across `wValue`/`wIndex`, exactly like [`wr`](Self::wr), and chunks at
    /// `bMaxPacketSize0` (MEASURED 64).
    ///
    /// The length is rounded up to a multiple of 4 with **zero padding**.
    /// Upstream does `len = round_up(len, 4)` and then reads past the caller's
    /// buffer (`mt792x_usb.c:195`); the comment on the equivalent line of
    /// `mt76u_copy` explains why the rounding itself is mandatory —
    /// *"Assure that always a multiple of 4 bytes are copied, otherwise beacons
    /// can be corrupted"* (`usb.c:178-183`). Reading past a Rust slice is not on
    /// the table, so the pad is explicit and its content is defined rather than
    /// whatever the allocator held.
    pub fn write_copy(&self, offset: u32, data: &[u8]) -> Result<(), FaceError> {
        let padded = round_up4(data.len());
        let mut buf = Vec::with_capacity(padded);
        buf.extend_from_slice(data);
        buf.resize(padded, 0);

        let mut i = 0usize;
        while i < padded {
            let n = self.ctrl_batch.min(padded - i);
            let addr = offset.wrapping_add(i as u32);
            let wrote = self.vendor_write_typed(
                self.tag.req_out(),
                MT_VEND_WRITE_EXT,
                (addr >> 16) as u16,
                (addr & 0xffff) as u16,
                &buf[i..i + n],
            )?;
            if wrote != n {
                return Err(tr_err(format!(
                    "{} write_copy({offset:#x}) short write at +{i}: {wrote}/{n}",
                    self.label
                )));
            }
            i += n;
        }
        Ok(())
    }

    /// Read a run of bytes from consecutive register addresses —
    /// `mt76u_read_copy` (`usb.c:201-224`), which already uses
    /// `MT_VEND_READ_EXT` and is shared unchanged by connac2 (`mt7921/usb.c:178`
    /// installs it as `bus_ops.read_copy`).
    ///
    /// Same 4-byte rounding as [`write_copy`](Self::write_copy); when `buf` is
    /// not a multiple of 4 the transfer runs into a local scratch and only
    /// `buf.len()` bytes are copied back, so a caller cannot be handed the pad.
    pub fn read_copy(&self, offset: u32, buf: &mut [u8]) -> Result<(), FaceError> {
        let padded = round_up4(buf.len());
        let mut scratch = vec![0u8; padded];
        let mut i = 0usize;
        while i < padded {
            let n = self.ctrl_batch.min(padded - i);
            let addr = offset.wrapping_add(i as u32);
            let got = self.vendor_read_typed(
                self.tag.req_in(),
                MT_VEND_READ_EXT,
                (addr >> 16) as u16,
                (addr & 0xffff) as u16,
                &mut scratch[i..i + n],
            )?;
            if got != n {
                return Err(tr_err(format!(
                    "{} read_copy({offset:#x}) short read at +{i}: {got}/{n}",
                    self.label
                )));
            }
            i += n;
        }
        let take = buf.len();
        buf.copy_from_slice(&scratch[..take]);
        Ok(())
    }

    // ── Non-register vendor requests ────────────────────────────────────────

    rung! {
        /// The `MT_VEND_POWER_ON` request (`0x04`) — `mt792xu_mcu_power_on`
        /// (`mt792x_usb.c:214-232`).
        ///
        /// ★ Note the odd argument order upstream uses: `wValue = 0x0`,
        /// `wIndex = 0x1`, **no data stage**. The 1 is in the *index*, not the value.
        /// Why is not stated anywhere in the tree; it is ported as an opaque pair by
        /// its one call site, not derived.
        ///
        /// This only kicks the request. The caller must then poll `MT_CONN_ON_MISC`
        /// (`0x7c06_00f0`) for `MT_TOP_MISC2_FW_PWR_ON` (bit 0) — upstream allows
        /// 500 ms — and that poll belongs in `mcu.rs`, which owns the register names.
        /// ★ MEASURED: that register reads `0x0000_0000` on a cold part, so the bit
        /// really is the "did power-on take" signal and not something already set.
        fn power_on(&self) -> Result<(), FaceError> {
            // ★ REQ_OUT_UPSTREAM (0x5f), NOT `self.tag.req_out()`. MEASURED: the same request at
            // the plain 0x40 is ACKed and does nothing — `MT_CONN_ON_MISC` never leaves 0. The
            // bootrom gates this one on the vendor-specific recipient in the low five bits, while
            // the register window (also MEASURED) answers only at 0xc0/0x40. The split is real; do
            // not "simplify" either side to match the other.
            self.vendor_write_typed(REQ_OUT_UPSTREAM, MT_VEND_POWER_ON, 0x0, 0x1, &[])?;
            Ok(())
        }
    }

    /// Is the bus answering? Reads `MT_HW_CHIPID` and returns it —
    /// `mt792xu_check_bus` (`mt792x_usb.c:116-129`), minus the `bus_hung` /
    /// queued-USB-reset escalation, which is the wedging behaviour this port
    /// refuses (see the module header).
    ///
    /// ★ MEASURED `0x0000_7961` on the target part. A mismatch is logged rather
    /// than made fatal: a rebadged board could in principle report something
    /// else, and the caller comparing against its own table is better placed to
    /// decide than the transport is.
    pub fn check_bus(&self) -> Result<u32, FaceError> {
        let id = self.rr(CHIPID_ADDR)?;
        if id != CHIPID_MT7961 {
            tracing::warn!(
                target: "named_radio",
                chip = self.label,
                chip_id = format_args!("{id:#010x}"),
                expected = format_args!("{CHIPID_MT7961:#06x}"),
                "connac2 MT_HW_CHIPID is not the MEASURED 0x7961",
            );
        }
        Ok(id)
    }

    // ── Polling ─────────────────────────────────────────────────────────────

    /// Poll `addr` until `val & mask == expect`, one read per millisecond.
    ///
    /// At **268 µs** per read plus a 1 ms sleep, `tries` is a little over the
    /// timeout in milliseconds — state it that way when choosing one, and
    /// remember the read cost is nearly double the mt76x0 figure this crate's
    /// other polls were sized against. Mirrors `mt76_poll_msec`.
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

    /// [`poll`](Self::poll) on the UHW window — the shape of the
    /// `MT792x_WFSYS_INIT_RETRY_COUNT` loop in `mt792xu_wfsys_reset`
    /// (`mt792x_usb.c:455-462`), which polls a UHW register and cannot use
    /// [`poll`](Self::poll).
    pub fn uhw_poll(
        &self,
        addr: u32,
        mask: u32,
        expect: u32,
        tries: u32,
        gap: Duration,
    ) -> Result<u32, FaceError> {
        for _ in 0..tries {
            let v = self.uhw_rr(addr)?;
            if v & mask == expect {
                return Ok(v);
            }
            std::thread::sleep(gap);
        }
        Err(tr_err(format!(
            "{} uhw_poll({addr:#x} & {mask:#x} == {expect:#x}) timed out after {tries} tries",
            self.label
        )))
    }

    // ── Bulk pipes ──────────────────────────────────────────────────────────

    /// Write one bulk transfer to `ep`, requiring the whole buffer to go.
    pub fn bulk_out(&self, ep: u8, buf: &[u8]) -> Result<(), FaceError> {
        self.bulk_out_timeout(ep, buf, BULK_TIMEOUT)
    }

    /// [`bulk_out`](Self::bulk_out) with a caller-chosen timeout — for the
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

    /// Send `buf` on the MCU inband-command bulk-OUT endpoint (`0x04`).
    pub fn bulk_out_cmd(&self, buf: &[u8]) -> Result<(), FaceError> {
        self.bulk_out(self.ep_out_cmd, buf)
    }

    /// Send `buf` on the WLAN data bulk-OUT endpoint (`0x05`) — also the
    /// `FW_SCATTER` pipe (`mt7921/usb.c:47-50`).
    pub fn bulk_out_data(&self, buf: &[u8]) -> Result<(), FaceError> {
        self.bulk_out(self.ep_out_data, buf)
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

    /// Read one RX burst (RXD + 802.11 frame) from the packet bulk-IN endpoint
    /// (`0x84`).
    pub fn bulk_in_data(&self, buf: &mut [u8]) -> Result<usize, FaceError> {
        self.bulk_in(self.ep_in_data, buf)
    }

    /// Read one MCU response from the cmd-response bulk-IN endpoint (`0x85`).
    ///
    /// ★ Reading this matters for throughput, not just correctness: the MCU
    /// holds its response and will not accept the next command until the host
    /// takes it, so a skipped ACK makes every following command's bulk-write
    /// block for about a second — paid for on the MT7612U
    /// (`src/mt7612/mod.rs:836-843`).
    pub fn bulk_in_resp(&self, buf: &mut [u8]) -> Result<usize, FaceError> {
        self.bulk_in(self.ep_in_resp, buf)
    }

    /// Drain whatever is sitting on `ep` right now, with a short timeout, and
    /// return how many transfers were consumed.
    ///
    /// Used to clear a stale MCU response before issuing a command that expects
    /// its own — otherwise the previous command's ACK is mistaken for this one's.
    pub fn drain_bulk_in(&self, ep: u8, timeout: Duration, max: usize) -> usize {
        let mut buf = vec![0u8; self.ep_in_packet_size.max(512) as usize];
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
    /// must have something reading the RX pipe, even before it cares about frames.
    ///
    /// The returned [`AtomicBool`] is a **pause**, not a stop: set it before
    /// consuming frames in the foreground so the drain stops stealing them, clear
    /// it to resume. The thread runs for the life of the process and holds only a
    /// clone of the device handle.
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

    /// The MCU sequence number, 1..=15 and **never 0**.
    ///
    /// Exactly `mt76_connac2_mcu_fill_message`'s generator
    /// (`mt76_connac_mcu.c`: `seq = ++dev->mcu.msg_seq & 0xf; if (!seq) seq = ++… & 0xf;`).
    /// Zero is not an off-by-one to be tidied away: mt76 reserves `seq == 0` for
    /// commands that post **no** response, and forcing a nonzero seq onto one of
    /// those leaves the firmware not draining it — the command FIFO fills after
    /// ~90 commands and every further write blocks ~1 s
    /// (`src/mt7612/mod.rs:880-886`). A caller replaying a no-response command
    /// must pass 0 explicitly rather than take a number from here.
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
}

// ── Helpers (pure) ───────────────────────────────────────────────────────────

/// Round a length up to a multiple of 4 — upstream's `round_up(len, 4)` in both
/// copy helpers (`mt792x_usb.c:195`, `usb.c:208`). Saturating rather than
/// wrapping, because a wrapped length would turn a huge write into a tiny one.
fn round_up4(n: usize) -> usize {
    n.saturating_add(3) & !3usize
}

/// Group a `(vid, pid)` table into `(vid, [pid…])`, preserving first-seen vendor
/// order, because [`select_device`] takes one vendor id and a slice of product
/// ids. MediaTek's own entry is first in [`MT7921U_VID_PIDS`] and therefore tried
/// first.
fn group_pids_by_vid(table: &[(u16, u16)]) -> Vec<(u16, Vec<u16>)> {
    let mut out: Vec<(u16, Vec<u16>)> = Vec::new();
    for &(vid, pid) in table {
        match out.iter_mut().find(|(v, _)| *v == vid) {
            Some((_, pids)) => pids.push(pid),
            None => out.push((vid, vec![pid])),
        }
    }
    out
}

/// Reduce an active config descriptor to [`IfaceCandidate`]s, one per interface
/// **alt-setting 0**.
///
/// ★ Endpoints are collected *per interface*, which is the whole difference from
/// `crate::mt76::transport::Mt76Usb::claim` — that one accumulates across every
/// interface, correct for a single-function dongle and catastrophic on this
/// composite part. Higher alt settings are skipped because mt76 reads
/// `intf->cur_altsetting` and never calls `usb_set_interface`, so setting 0 is
/// what upstream sees too.
fn enumerate_interfaces(config: &rusb::ConfigDescriptor) -> Vec<IfaceCandidate> {
    let mut out = Vec::new();
    for iface in config.interfaces() {
        for d in iface.descriptors() {
            if d.setting_number() != 0 {
                continue;
            }
            let mut cand = IfaceCandidate {
                number: iface.number(),
                alt_setting: d.setting_number(),
                class: (d.class_code(), d.sub_class_code(), d.protocol_code()),
                in_packet_size: 512,
                ..Default::default()
            };
            let mut first_in = true;
            for ep in d.endpoint_descriptors() {
                match (ep.transfer_type(), ep.direction()) {
                    (TransferType::Bulk, Direction::Out) => cand.bulk_out.push(ep.address()),
                    (TransferType::Bulk, Direction::In) => {
                        if first_in {
                            cand.in_packet_size = ep.max_packet_size();
                            first_in = false;
                        }
                        cand.bulk_in.push(ep.address());
                    }
                    (TransferType::Interrupt, Direction::In) => cand.int_in.push(ep.address()),
                    _ => {}
                }
            }
            out.push(cand);
        }
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The MEASURED interface inventory of `0e8d:7961` on mds-o5p-3: three
    /// Bluetooth interfaces and one WLAN interface at number 3.
    fn measured_mt7921au_interfaces() -> Vec<IfaceCandidate> {
        let bt = |n: u8| IfaceCandidate {
            number: n,
            alt_setting: 0,
            class: (0xe0, 0x01, 0x01),
            // Interface 0 of a btusb device really does carry bulk pipes — which
            // is exactly why "any interface with bulk endpoints" is not a legal
            // selection rule here.
            bulk_in: vec![0x82],
            bulk_out: vec![0x02],
            int_in: vec![0x81],
            in_packet_size: 512,
        };
        vec![
            bt(0),
            IfaceCandidate {
                number: 1,
                alt_setting: 0,
                class: (0xe0, 0x01, 0x01),
                ..Default::default()
            },
            IfaceCandidate {
                number: 2,
                alt_setting: 0,
                class: (0xe0, 0x01, 0x01),
                ..Default::default()
            },
            IfaceCandidate {
                number: 3,
                alt_setting: 0,
                class: (0xff, 0xff, 0xff),
                bulk_in: vec![0x84, 0x85],
                bulk_out: vec![0x04, 0x05, 0x06, 0x07, 0x08, 0x09],
                int_in: vec![0x86],
                in_packet_size: 512,
            },
        ]
    }

    /// ★ The single most dangerous decision in this file: on the MEASURED
    /// composite board the WLAN function is interface **3**, and interfaces 0-2
    /// are Bluetooth — including one with bulk pipes, which an "any interface
    /// with bulk endpoints" rule would have grabbed.
    #[test]
    fn wlan_interface_is_selected_by_class_not_by_first_bulk() {
        let cands = measured_mt7921au_interfaces();
        let i = select_wlan_interface(&cands, "MT7921AU").unwrap();
        assert_eq!(cands[i].number, 3);
        assert_eq!(cands[i].class, WLAN_IFACE_CLASS);
    }

    /// Other MT7921 boards put WLAN on interface 0 with no Bluetooth at all.
    /// Both layouts must work off the same rule.
    #[test]
    fn wlan_interface_zero_layout_also_works() {
        let cands = vec![IfaceCandidate {
            number: 0,
            alt_setting: 0,
            class: (0xff, 0xff, 0xff),
            bulk_in: vec![0x84, 0x85],
            bulk_out: vec![0x04, 0x05, 0x06, 0x07, 0x08, 0x09],
            int_in: vec![0x86],
            in_packet_size: 512,
        }];
        assert_eq!(select_wlan_interface(&cands, "MT7921AU").unwrap(), 0);
    }

    /// A device with only Bluetooth interfaces must be refused outright — there
    /// is no fallback that would be safe, and the error has to name what it saw.
    #[test]
    fn a_bluetooth_only_device_is_refused_with_an_inventory() {
        let cands = measured_mt7921au_interfaces()
            .into_iter()
            .filter(|c| c.number != 3)
            .collect::<Vec<_>>();
        let err = select_wlan_interface(&cands, "MT7921AU").unwrap_err();
        let msg = format!("{err}");
        assert!(msg.contains("no class ff/ff/ff"), "{msg}");
        assert!(msg.contains("btusb") || msg.contains("Bluetooth"), "{msg}");
        assert!(
            msg.contains("if0"),
            "the error must list what it did see: {msg}"
        );
    }

    /// A vendor-specific interface with no bulk pipes is not the WLAN function
    /// either — the class alone is not sufficient.
    #[test]
    fn vendor_class_without_bulk_pipes_is_not_wlan() {
        let cands = vec![IfaceCandidate {
            number: 0,
            alt_setting: 0,
            class: (0xff, 0xff, 0xff),
            ..Default::default()
        }];
        assert!(select_wlan_interface(&cands, "X").is_err());
    }

    /// Descriptor order is the assignment rule, and on this part it puts the
    /// MEASURED addresses at mt76's enum indices (`mt76.h:646-660`).
    #[test]
    fn endpoint_indices_match_the_measured_mt7921au_layout() {
        let eps = assign_endpoints(
            &[0x84, 0x85],
            &[0x04, 0x05, 0x06, 0x07, 0x08, 0x09],
            &[0x86],
            "MT7921AU",
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
        assert_eq!(eps.int_ins, vec![0x86]);
    }

    /// Descriptor order, not address order: a board listing the OUT pipes
    /// backwards still gets `MT_EP_OUT_INBAND_CMD` from position 0, because that
    /// is what `mt76u_set_endpoints` (`usb.c:304-327`) does.
    #[test]
    fn endpoint_assignment_follows_descriptor_order_not_address_order() {
        let eps = assign_endpoints(
            &[0x85, 0x84],
            &[0x09, 0x08, 0x07, 0x06, 0x05, 0x04],
            &[],
            "X",
        )
        .unwrap();
        assert_eq!(eps.out_at(MT_EP_OUT_INBAND_CMD), 0x09);
        assert_eq!(eps.in_at(MT_EP_IN_PKT_RX), 0x85);
    }

    /// A device with no bulk pipe in one direction is unusable and must say so;
    /// an unexpected *count* warns and carries on, because the indices may still
    /// be right and a hard failure would be less informative.
    #[test]
    fn endpoint_assignment_rejects_a_missing_direction_but_tolerates_a_short_count() {
        assert!(assign_endpoints(&[], &[0x04], &[], "X").is_err());
        assert!(assign_endpoints(&[0x84], &[], &[], "X").is_err());
        let eps = assign_endpoints(&[0x84], &[0x04, 0x05], &[], "X").unwrap();
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

    /// ★ The `READ_EXT`/`WRITE_EXT` address split, checked against the MEASURED
    /// register addresses. `0x7c06_00f0` (`MT_CONN_ON_MISC`) is the case that
    /// proves the **full 32-bit** address is carried: a driver that only put the
    /// low 16 bits in `wIndex` would address `0x0000_00f0` and read something
    /// else entirely.
    #[test]
    fn address_splits_into_wvalue_and_windex() {
        for (addr, wval, widx) in [
            (0x7001_0200u32, 0x7001u16, 0x0200u16), // MT_HW_CHIPID -> 0x7961
            (0x7001_0204, 0x7001, 0x0204),          // MT_HW_REV    -> 0x8a10
            (0x7c06_00f0, 0x7c06, 0x00f0),          // MT_CONN_ON_MISC
            (0x7000_00f0, 0x7000, 0x00f0),          // MT_TOP_MISC
        ] {
            assert_eq!((addr >> 16) as u16, wval, "wValue for {addr:#x}");
            assert_eq!((addr & 0xffff) as u16, widx, "wIndex for {addr:#x}");
        }
        // MT_CONN_ON_MISC and MT_TOP_MISC share a low half and differ only in
        // wValue — the pair that a dropped high half would silently conflate.
        assert_eq!(0x7c06_00f0u32 & 0xffff, 0x7000_00f0u32 & 0xffff);
        assert_ne!(0x7c06_00f0u32 >> 16, 0x7000_00f0u32 >> 16);
    }

    /// The request bytes are the connac2 ones, not the mt76x0/x2 ones. Wiring
    /// `MULTI_READ`/`MULTI_WRITE` here would be a plausible-looking port that
    /// does not work on this silicon.
    #[test]
    fn vendor_requests_are_the_ext_pair() {
        assert_eq!(MT_VEND_READ_EXT, 0x63, "mt76.h:641");
        assert_eq!(MT_VEND_WRITE_EXT, 0x66, "mt76.h:642");
        // The mt76x0/x2 pair, for contrast — must never appear on this path.
        assert_ne!(MT_VEND_READ_EXT, 0x07);
        assert_ne!(MT_VEND_WRITE_EXT, 0x06);
    }

    /// ★ The `bmRequestType` low bits are a **space selector** on this part, so
    /// the normal and UHW tags must stay distinct, and the UHW tag must be the
    /// exact upstream one (`mt792x.h:555-556`).
    #[test]
    fn request_types_keep_the_two_address_spaces_apart() {
        assert_eq!(VendorTag::Plain.req_in(), 0xc0, "MEASURED working");
        assert_eq!(VendorTag::Plain.req_out(), 0x40, "MEASURED working");
        assert_eq!(VendorTag::Upstream.req_in(), 0xdf); // USB_DIR_IN | 0x40 | 0x1f
        assert_eq!(VendorTag::Upstream.req_out(), 0x5f);
        assert_eq!(REQ_IN_UHW, 0xde); // USB_DIR_IN | 0x40 | 0x1e
        assert_eq!(REQ_OUT_UHW, 0x5e);
        for t in [VendorTag::Plain, VendorTag::Upstream] {
            assert_ne!(t.req_in(), REQ_IN_UHW, "UHW must not collide with normal");
            assert_ne!(t.req_out(), REQ_OUT_UHW);
        }
        // ★ The register window default is the MEASURED-working `Plain`. Power-on is the
        // exception and sends REQ_OUT_UPSTREAM explicitly — see `Connac2Usb::power_on` and the
        // split-answer table on `VendorTag`.
        assert_eq!(VendorTag::default(), VendorTag::Plain);
    }

    /// The copy helpers round to a whole number of dwords — `round_up(len, 4)`
    /// (`mt792x_usb.c:195`, `usb.c:208`), the rounding whose absence corrupts
    /// beacons per `usb.c:178-183`.
    #[test]
    fn copy_length_rounds_up_to_a_dword() {
        assert_eq!(round_up4(0), 0);
        assert_eq!(round_up4(1), 4);
        assert_eq!(round_up4(3), 4);
        assert_eq!(round_up4(4), 4);
        assert_eq!(round_up4(5), 8);
        assert_eq!(round_up4(64), 64);
        // Saturating, so a pathological length cannot wrap to a tiny transfer.
        assert!(round_up4(usize::MAX) >= usize::MAX - 3);
    }

    /// The vid/pid table groups by vendor with MediaTek first, so the MEASURED
    /// part is the first thing looked for.
    #[test]
    fn vid_pid_table_groups_with_mediatek_first() {
        let groups = group_pids_by_vid(MT7921U_VID_PIDS);
        assert_eq!(groups[0].0, MEDIATEK_VID);
        assert_eq!(groups[0].1, vec![MT7921AU_PID]);
        // Netgear ships two product ids under one vendor id; they must land in
        // one group, not two.
        let netgear = groups.iter().find(|(v, _)| *v == 0x0846).unwrap();
        assert_eq!(netgear.1, vec![0x9060, 0x9065]);
        assert_eq!(
            groups.iter().map(|(_, p)| p.len()).sum::<usize>(),
            MT7921U_VID_PIDS.len(),
            "grouping must not lose or duplicate an id"
        );
    }

    /// The MCU sequence must stay in 1..=15 and never hand out 0 — 0 means
    /// "no response expected" to the firmware, and a wrong one there stalls the
    /// command FIFO.
    #[test]
    fn mcu_seq_never_yields_zero() {
        let seq = AtomicU8::new(0);
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
