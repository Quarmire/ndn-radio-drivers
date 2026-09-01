//! Userspace libusb backend for the **MT7612U** (`0e8d:7612`) — a 2×2 dual-band
//! 802.11ac dongle on the MediaTek `mt76x2u` driver. This is the **TX-capable**
//! radio for the NDN named-radio face: unlike the firmware-gated RTL8821c (which
//! is [RX-only][crate::Rtl8821cuBackend] — host injection keys the FIFO but
//! never radiates), MT76 offloads TX calibration to firmware and host-injected
//! frames radiate directly.
//!
//! Reference driver: **github.com/morrownr/mt76** (and the mainline mt76 tree).
//! Every register/flow is verified against the golden usbmon trace in
//! `golden/mt7612-usbmon-2026-06-17/golden_init.pcap`.
//!
//! ## USB register model (much simpler than Realtek's)
//! All MMIO is a USB vendor control transfer (no per-section quirk):
//! - **write32**: `bmRequestType=0x40`, `bRequest=0x06` (MULTI_WRITE),
//!   `wValue=addr>>16`, `wIndex=addr&0xffff`, 4-byte LE data.
//! - **read32**: `bmRequestType=0xc0`, `bRequest=0x07` (MULTI_READ), same split.
//! - CFG read/write `0x47`/`0x46`, FCE write `0x42` (value in `wValue`),
//!   EEPROM read `0x09`, dev-mode/power `0x01`.
//!
//! ## Status: complete and radiating — the rig's highest-throughput radio
//! The full path is implemented and validated on hardware: USB open + register
//! access, **firmware download** (ROM patch + ILM/DLM via the FCE DMA path),
//! EEPROM/cal, MAC init, monitor RX, and the [`FrameIo`] TX/RX impl. Host-injected
//! frames **radiate** (no firmware TX gate, unlike the RTL8821c). Two RF programs
//! are captured and replayed: 2.4 GHz **ch6 / 20 MHz**
//! ([`set_channel_ch6`](Mt7612uBackend::set_channel_ch6)) and 5 GHz **ch36 /
//! 80 MHz VHT80** ([`set_channel_5g80`](Mt7612uBackend::set_channel_5g80)), the
//! latter with both TX chains (2 spatial streams). One-call bring-up for the
//! high-throughput NDN path is [`start_ndn_vht80`](Mt7612uBackend::start_ndn_vht80).
//!
//! Two throughput levers, both measured, both unlike the Realtek parts:
//!   1. plain MPDUs radiate intact to **~5650 B**
//!      ([`MAX_MPDU_PAYLOAD`](Mt7612uBackend::MAX_MPDU_PAYLOAD)), so a large send
//!      MTU amortises the ~300 µs/MPDU fixed overhead (≈142 Mb/s at VHT80 2×2 SGI
//!      vs ≈37 Mb/s at a 1500 B MTU), and
//!   2. a background **TX pump** ([`spawn_tx_pump`](Mt7612uBackend::spawn_tx_pump))
//!      that pipelines per-frame injects while staying RX-compatible.
//!
//! Host-built **A-MSDU does not radiate** on monitor injection here (firmware-gated;
//! verified 0/200 on air), so `inject_batch` deliberately sends plain MPDUs rather
//! than bundling — the opposite of the RTL8812EU backend.
//!
//! Known gap: adding another channel means capturing its RF program the same way
//! (see `docs/RADIO_SUBSYSTEM.md`, "Adding a channel"), and the
//! [`RadioKnobs`](ndn_radio_hal::RadioKnobs) `set_channel` currently exposes only
//! the ch6/20 MHz replay — the VHT80 program is reachable through the inherent
//! method but not yet through the uniform knob. The power / TXOP / ED-CCA registers
//! are unported, so those knobs are still no-ops.
#![allow(dead_code)]

use std::io;
use std::sync::Arc;
use std::time::Duration;

use rusb::{Context, Device, DeviceHandle, Direction, TransferType, UsbContext};

use crate::{CapturedFrame, FrameFormat, FrameIo, InjectFrame, McsDescriptor};
use async_trait::async_trait;
use bytes::Bytes;
use ndn_radio_hal::{
    Band, ClockDomainId, RadioCapability, RadioProfile, RadioTime, RadioTimeSource, RateCapability,
};
use ndn_transport::FaceError;

/// Async USB TX ring (libusb URBs) — the TX-pipelining path. Linux-only.
#[cfg(target_os = "linux")]
mod tx_async;
#[cfg(target_os = "linux")]
pub use tx_async::TxRing;

/// mt76 USB RX descriptor length before the 802.11 frame: MT_RX_INFO_LEN (4) +
/// the RXWI (32) = 36. Verified against captured RX bursts (a beacon's FC +
/// broadcast addr1 land exactly at offset 36). A 4-byte FCE-info trailer follows
/// the frame. RXWI RSSI[0] sits at offset 18.
const MT76_RXD_LEN: usize = 36;

mod init_table;

// ── Identity ────────────────────────────────────────────────────────────────
pub const MEDIATEK_VID: u16 = 0x0e8d;
/// MT7612U in WiFi mode. (The CD-ROM "driver installer" PID, if present, would
/// need usb_modeswitch — handled the same way as the Realtek path.)
pub const MT7612U_PIDS: &[u16] = &[0x7612, 0x7632, 0x7662];

// ── USB vendor requests (mt76 usb.h `enum mt76_vendor_req`) ─────────────────
const REQ_OUT: u8 = 0x40; // host→device | vendor | device
const REQ_IN: u8 = 0xc0; // device→host | vendor | device
const MT_VEND_DEV_MODE: u8 = 0x01;
const MT_VEND_POWER_ON: u8 = 0x04;
const MT_VEND_MULTI_WRITE: u8 = 0x06;
const MT_VEND_MULTI_READ: u8 = 0x07;
const MT_VEND_READ_EEPROM: u8 = 0x09;
const MT_VEND_WRITE_FCE: u8 = 0x42;
const MT_VEND_WRITE_CFG: u8 = 0x46;
const MT_VEND_READ_CFG: u8 = 0x47;

const CTRL_TIMEOUT: Duration = Duration::from_millis(500);
const BULK_TIMEOUT: Duration = Duration::from_millis(1000);

// ── MCU / FCE registers (verified against golden_init.pcap) ─────────────────
// FCE config written just before the firmware download.
const MT_FCE_PSE_CTRL: u32 = 0x0800; // value 1
const MT_FCE_PDMA_GLOBAL_CONF: u32 = 0x09c4; // value 0x44 (golden)
const MT_FCE_SKIP_FS: u32 = 0x0a6c; // value 0x3 (golden)
const MT_FCE_PSE_CTRL_GO: u32 = 0x09a8; // value 1 after each chunk (golden)
const MT_TX_CPU_FROM_FCE_BASE_PTR: u32 = 0x0090; // -> 0x400230 region; cfg
const MT_USB_U3DMA_CFG: u16 = 0x9018; // CFG-space USB DMA config

// FCE DMA descriptor — written as 16-bit halves via WRITE_FCE (value in wValue).
const MT_FCE_DMA_ADDR: u16 = 0x0230; // +0x0232 = high half
const MT_FCE_DMA_LEN: u16 = 0x0234; // +0x0236 = high half

/// MCU↔host **mailbox** (`mt76x02_mcu.h:13`).
///
/// ⚠ It is a mailbox, not a status flag, and treating it as one is what wedged this dongle three
/// times. `start_mcu` writes `0x001140fb` here as the runtime handshake, so immediately after a
/// bring-up it reads back as "firmware running" — but the MCU uses the same register for its own
/// traffic, and after a channel set plus a few seconds of operation it no longer does. The next
/// `open()` then concluded "cold", re-downloaded the ROM patch into a live MCU, and timed out at
/// chunk 2 with the device dropping off the bus (`device not accepting address, error -62`).
/// Use [`Mt7612uBackend::rom_patch_applied`] for a persistent answer.
const MT_MCU_COM_REG0: u32 = 0x0730;
/// `MT_MCU_CLOCK_CTL` (`mt76x2/mcu.h:13`). Bit 0 is the **ROM-patch-applied latch** on rev ≥ E3 —
/// the flag upstream itself tests to print "ROM patch already applied" and skip the download
/// (`mt76x2/usb_mcu.c:72-83`). Unlike the mailbox it survives normal operation, clearing only on a
/// real chip power cycle.
const MT_MCU_CLOCK_CTL: u32 = 0x0708;

// MCU download target offsets (mt76x2u_mcu_load_*).
const MCU_ROM_PATCH_OFFSET: u32 = 0x9_0000;
const MCU_ILM_OFFSET: u32 = 0x8_0000;
// DLM base is 0x110000 on early silicon but 0x110800 on rev ≥ E3 (the kernel's
// MT_MCU_DLM_ADDR_E3). This adapter is ASIC rev 76120044 (E3): the golden trace
// loads DLM at 0x110800. Using 0x110000 lands the data segment 0x800 too low —
// the firmware boots and answers bootrom + a couple commands but then goes silent
// on the calibration commands (and COM_REG0 reads 0x1138f9 instead of 0x1140f9,
// off by exactly 0x800). 0x110800 is correct for this part.
const MCU_DLM_OFFSET: u32 = 0x11_0800;
const FW_CHUNK_MAX: usize = 0x3900; // max ILM/DLM payload per send (mt76x2u)
const PATCH_CHUNK_MAX: usize = 2048;

// MCU inband info header (FCE TX): little-endian `len | flags`. The golden
// firmware chunks use flag byte 0x50 in the top byte (info = 0x50<<24 | len).
const MCU_TXD_FLAG: u32 = 0x5000_0000;

// Vendored firmware (linux-firmware; see fw/mt7612/).
const ROM_PATCH: &[u8] = include_bytes!("../../fw/mt7612/mt7662_rom_patch.bin");
const RAM_FIRMWARE: &[u8] = include_bytes!("../../fw/mt7612/mt7662.bin");

/// Full golden init op-stream (see scripts/gen_mt7612_replay.py) replayed by
/// [`Mt7612uBackend::bring_up`].
const INIT_REPLAY: &[u8] = include_bytes!("init_replay.bin");

/// Monitor-mode + channel-6 (2.4GHz) RF/BB tune op-stream, captured from the
/// kernel `iw set monitor; set channel 6` (see scripts/gen_mt7612_chanset.py).
/// Replayed by [`Mt7612uBackend::set_channel`] AFTER `bring_up` to tune the RF
/// so ambient frames arrive (init alone leaves the RF untuned → 0 RX).
const CHANSET_REPLAY: &[u8] = include_bytes!("chanset_replay.bin");

/// Monitor-mode + **5GHz channel 36 @ 80MHz (VHT80, center 5210 MHz)** RF/BB tune,
/// captured from the kernel `iw dev wlan0 set channel 36 80MHz` (see
/// scripts/gen_mt7612_chanset_5g.py). Replayed by [`Mt7612uBackend::set_channel_5g80`].
/// The throughput path: a clean 5GHz channel (vs congested 2.4GHz ch6, which adds
/// ~200µs of CSMA per frame) at 80MHz bandwidth (4× the per-byte rate of HT20).
const CHANSET_REPLAY_5G80: &[u8] = include_bytes!("chanset_replay_5g80.bin");

/// One captured kernel probe-request TX bulk (ep 0x07): `[info u32][TXWI 20B]
/// [802.11 frame][tail]`. Replayed verbatim by `tx_raw` for the radiation test,
/// and the source of the TXWI template used by `transmit`.
const TX_PROBE: &[u8] = include_bytes!("tx_probe.bin");

/// TXWI template for a DATA frame, from a captured kernel data-frame TX
/// (`golden/.../tx_data.pcap`): flags=0, rate=0 (filled per-frame), ack_ctl=0
/// (broadcast → no ACK), **wcid=0xff** (broadcast/no-station), len_ctl=0 (filled),
/// iv/eiv=0, byte17=0x13 (the kernel's ctl2). The wcid is the key difference from
/// the mgmt template (0xfd) — the firmware drops data frames sent with a mgmt
/// wcid/endpoint.
const TXWI_DATA: [u8; 20] = [
    0, 0, 0, 0, 0, 0xff, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0x13, 0, 0,
];

/// `struct mt76x02_patch_header` — 30-byte header skipped before the patch body.
const PATCH_HEADER_LEN: usize = 30;
/// `struct mt76x02_fw_header` — 32-byte header before ILM/DLM.
const FW_HEADER_LEN: usize = 32;

fn usb_err(e: rusb::Error) -> FaceError {
    FaceError::Io(io::Error::other(format!("mt7612u usb: {e}")))
}
/// Encode an [`McsDescriptor`] into the mt76x02 TXWI rate word (the `__le16` at
/// TXWI offset 2). Field layout (mt76x02_mac.h `MT_RXWI_RATE_*`):
/// index[5:0] | LDPC[6] | BW[8:7] | SGI[9] | STBC[11:10] | PHY[15:13].
/// PHY type: OFDM=1, HT=2, VHT=4. HT carries NSS in the index (MCS8-15 = 2SS);
/// VHT splits index[3:0]=MCS, index[5:4]=NSS-1. BW left 0 (20 MHz) for now.
fn mt76_rate_val(m: &McsDescriptor) -> u16 {
    let (phy, idx): (u16, u16) = if m.vht {
        (
            4,
            (m.index as u16 & 0x0f) | (((m.nss.max(1) - 1) as u16 & 0x03) << 4),
        )
    } else {
        (2, m.index as u16 & 0x3f)
    };
    let mut v = idx | (phy << 13);
    if m.ldpc {
        v |= 1 << 6;
    }
    if m.short_gi {
        v |= 1 << 9;
    }
    if m.stbc {
        v |= 1 << 10; // STBC field [11:10] = 1
    }
    v
}

fn init_err(what: String) -> FaceError {
    FaceError::Io(io::Error::other(what))
}

pub struct Mt7612uBackend {
    handle: Arc<DeviceHandle<Context>>,
    /// Bulk-OUT endpoint for MCU/firmware inband commands (ep 0x08 on this dongle).
    ep_cmd: u8,
    /// Bulk-OUT endpoint for WLAN data TX.
    ep_data: u8,
    /// Bulk-IN endpoint for RX (data frames).
    ep_in: u8,
    /// All bulk-IN endpoints in descriptor order (last is the MCU cmd-response).
    ep_ins: Vec<u8>,
    /// MCU command sequence (1..=15, never 0).
    mcu_seq: std::sync::atomic::AtomicU8,
    /// Current transmit rate as state ([`FrameIo::set_rate`]); `None` ⇒ resolve the
    /// frame's intent. Retires the per-frame `inject_at` path.
    cur_mcs: std::sync::Mutex<Option<crate::McsDescriptor>>,
    /// When true the background RX-drain thread stops reading ep 0x84 (so a
    /// foreground `read_rx`/FrameIo consumer gets every frame instead of racing
    /// the drain). Toggled by [`pause_drain`](Self::pause_drain).
    drain_pause: std::sync::Arc<std::sync::atomic::AtomicBool>,
    /// Wire frame format for `FrameIo` (NDN ethertype by default).
    format: FrameFormat,
    /// 802.11 sequence-number counter (12-bit) for TX frames.
    seq: std::sync::atomic::AtomicU16,
    /// RX-pump queue: background reader threads de-aggregate bulk-IN bursts into
    /// `CapturedFrame`s here; `recv_frame` drains it. Full-rate capture without a
    /// The shared RX pipeline (#80) — queue + wake + pumped flag, the same
    /// [`RxPumpState`](crate::rx_pump::RxPumpState) the Realtek backends use. This was three
    /// hand-rolled fields; unifying them is what let the pump itself be shared.
    rx: crate::rx_pump::RxPumpState,
    /// TX pump: `inject` hands pre-built USB bulks to a dedicated thread that does
    /// `write_bulk` in a tight loop — no per-frame `spawn_blocking` task dispatch
    /// (that capped TX at ~2000 frames/s). Set by [`spawn_tx_pump`](Self::spawn_tx_pump).
    tx_sender: std::sync::Mutex<Option<std::sync::mpsc::SyncSender<Vec<u8>>>>,
    /// Bytes and frames the TX-pump thread has actually written (throughput
    /// measurement / queue-drain waits).
    tx_bytes: std::sync::atomic::AtomicU64,
    tx_count: std::sync::atomic::AtomicU64,
    /// Current TX channel bandwidth code for the TXWI rate word BW field [8:7]:
    /// 0=20MHz, 1=40MHz, 2=80MHz. Set by the `set_channel_*` methods to match the
    /// RF tune (an 80MHz rate word on a 20MHz-tuned BB would be malformed). Read by
    /// `build_data_bulk` so VHT80 frames carry the right bandwidth. Default 20MHz.
    tx_bw: std::sync::atomic::AtomicU8,
    /// This radio's port-TSF clock domain (`MT_TSF_TIMER_DW0/DW1`). Keyed on
    /// bus/address like the Realtek backends so two identical dongles on one host
    /// are never conflated into one clock.
    tsf_domain: ClockDomainId,
    /// As-found ED-CCA register pair, captured the first time
    /// [`RadioKnobs::set_edcca_ignore`] turns the knob on so `off` can restore the
    /// state the driver actually started in rather than a guessed default.
    edcca_saved: crate::mt76::knobs::EdccaSaved,
    /// As-found EDCA state, so a contention posture can be undone. See
    /// [`crate::mt76::knobs::set_contention`].
    edca_saved: crate::mt76::knobs::EdcaSaved,
    /// Wall-clock instant of the last [`read_channel_activity`] sample. The
    /// channel-time counters are read-and-clear, so a sample *is* its window —
    /// but `MT_ED_CCA_TIMER` has no idle counterpart and must be normalised
    /// against elapsed host time, which is what this remembers.
    ct_last: std::sync::Mutex<std::time::Instant>,
}

/// Legacy OFDM 6 Mbps as an mt76x02 TXWI rate word — the universally decodable basic rate.
pub const MT76_RATE_OFDM6M: u16 = 0x2000;

impl Mt7612uBackend {
    /// Find and open the first MT7612U, claiming its interface.
    ///
    /// ★ **This no longer resets the device, and that polarity change is a safety
    /// fix, not a preference.** The old default reset every matching dongle on
    /// every open, justified as "clean power-on state". It never bought that: a
    /// USB reset does not reset the on-chip MCU, so the thing that actually
    /// prevents a bad re-download is the `firmware_running()` warm guard below,
    /// not the reset. What the reset *did* buy was the failure mode itself —
    /// MEASURED on minidronesys-05, `usb 2-1.3-port4: cannot reset (err = -110)`
    /// five times over, and six seconds later the port was marked
    /// `disable=1 / state=not attached` by the USB core, leaving the device a
    /// zombie: present in sysfs, `ENODEV` at usbfs, and recoverable only by a
    /// physical replug. Every mt76 part in this lab has been wedged this way at
    /// least once, and the sibling MT7921AU is in that state right now.
    ///
    /// So: no reset by default. `NDN_RADIO_FORCE_RESET=1` opts back in for the
    /// rare case where a genuinely half-initialised device needs re-enumerating
    /// and you are physically next to it. (`NDN_RADIO_NO_RESET`, the old opt-out,
    /// is now the default and is accepted-and-ignored so existing scripts run
    /// unchanged.)
    pub fn open() -> Result<Self, FaceError> {
        if std::env::var("NDN_RADIO_FORCE_RESET").is_ok() {
            eprintln!(
                "mt7612u: NDN_RADIO_FORCE_RESET set — issuing a USB reset. This is the operation \
                 that has wedged mt76 parts on this fleet; a failed reset can leave the hub port \
                 disabled until a physical replug."
            );
            let ctx = Context::new().map_err(usb_err)?;
            for dev in ctx.devices().map_err(usb_err)?.iter() {
                if let Ok(d) = dev.device_descriptor()
                    && d.vendor_id() == MEDIATEK_VID
                    && MT7612U_PIDS.contains(&d.product_id())
                    && let Ok(h) = dev.open()
                {
                    let _ = h.reset();
                }
            }
            std::thread::sleep(Duration::from_millis(1200));
        }

        let ctx = Context::new().map_err(usb_err)?;
        for dev in ctx.devices().map_err(usb_err)?.iter() {
            if let Ok(d) = dev.device_descriptor()
                && d.vendor_id() == MEDIATEK_VID
                && MT7612U_PIDS.contains(&d.product_id())
            {
                return Self::claim(dev);
            }
        }
        Err(FaceError::Io(io::Error::new(
            io::ErrorKind::NotFound,
            "no MT7612U found (MediaTek 0e8d:7612)",
        )))
    }

    fn claim(device: Device<Context>) -> Result<Self, FaceError> {
        let handle = device.open().map_err(usb_err)?;
        let config = device.active_config_descriptor().map_err(usb_err)?;

        // Collect bulk endpoints on the (single) vendor-specific interface.
        let (mut iface_n, mut outs, mut ins) = (None, Vec::<u8>::new(), Vec::<u8>::new());
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
                        Direction::In => ins.push(ep.address()),
                    }
                }
                if has_bulk {
                    iface_n = Some(iface.number());
                }
            }
        }
        let ep_in = ins.first().copied();
        let iface =
            iface_n.ok_or_else(|| init_err("MT7612U: no interface with bulk endpoints".into()))?;
        if outs.is_empty() || ep_in.is_none() {
            return Err(init_err("MT7612U: missing bulk OUT/IN endpoints".into()));
        }
        // The mt76 inband-command endpoint is ep 0x08 on this dongle (verified in
        // the golden trace); data rides the lower OUT pipes. Fall back to the
        // highest/lowest OUT address if 0x08 isn't present.
        let ep_cmd = outs
            .iter()
            .copied()
            .find(|&e| e == 0x08)
            .unwrap_or_else(|| *outs.iter().max().unwrap());
        let ep_data = *outs.iter().min().unwrap();

        let _ = handle.set_auto_detach_kernel_driver(true);
        handle.claim_interface(iface).map_err(usb_err)?;
        // NOTE: do NOT clear_halt here by default — on macOS it resets the
        // endpoint data toggle, which desyncs the FCE's first firmware transfer
        // on a *cold* device (the download then times out). Only useful to
        // recover a genuinely stalled endpoint: `NDN_RADIO_CLEAR_HALT=1`.
        if std::env::var("NDN_RADIO_CLEAR_HALT").is_ok() {
            let _ = handle.clear_halt(ep_cmd);
            let _ = handle.clear_halt(ep_data);
            if let Some(i) = ep_in {
                let _ = handle.clear_halt(i);
            }
        }
        if std::env::var("NDN_RADIO_EP_DEBUG").is_ok() {
            eprintln!(
                "mt7612u eps: OUT {:?} cmd={ep_cmd:#04x} data={ep_data:#04x} IN {:?}",
                outs.iter().map(|e| format!("{e:#04x}")).collect::<Vec<_>>(),
                ins.iter().map(|e| format!("{e:#04x}")).collect::<Vec<_>>(),
            );
        }
        Ok(Self {
            handle: Arc::new(handle),
            ep_cmd,
            ep_data,
            ep_in: ep_in.unwrap(),
            ep_ins: ins,
            mcu_seq: std::sync::atomic::AtomicU8::new(0),
            cur_mcs: std::sync::Mutex::new(None),
            drain_pause: std::sync::Arc::new(std::sync::atomic::AtomicBool::new(false)),
            format: FrameFormat::default(),
            seq: std::sync::atomic::AtomicU16::new(0),
            rx: crate::rx_pump::RxPumpState::new(),
            tx_sender: std::sync::Mutex::new(None),
            tx_bytes: std::sync::atomic::AtomicU64::new(0),
            tx_count: std::sync::atomic::AtomicU64::new(0),
            tx_bw: std::sync::atomic::AtomicU8::new(0),
            tsf_domain: ClockDomainId(
                (u32::from(device.bus_number()) << 8) | u32::from(device.address()),
            ),
            edcca_saved: crate::mt76::knobs::EdccaSaved::default(),
            edca_saved: crate::mt76::knobs::EdcaSaved::default(),
            ct_last: std::sync::Mutex::new(std::time::Instant::now()),
        })
    }

    // ── Register access ─────────────────────────────────────────────────────
    /// Read a 32-bit MMIO register (`MT_VEND_MULTI_READ`).
    pub fn rr(&self, addr: u32) -> Result<u32, FaceError> {
        let mut b = [0u8; 4];
        let n = self
            .handle
            .read_control(
                REQ_IN,
                MT_VEND_MULTI_READ,
                (addr >> 16) as u16,
                (addr & 0xffff) as u16,
                &mut b,
                CTRL_TIMEOUT,
            )
            .map_err(usb_err)?;
        if n != 4 {
            return Err(init_err(format!("mt7612u rr({addr:#x}) short {n}")));
        }
        Ok(u32::from_le_bytes(b))
    }

    /// Write a 32-bit MMIO register (`MT_VEND_MULTI_WRITE`).
    pub fn wr(&self, addr: u32, val: u32) -> Result<(), FaceError> {
        let n = self
            .handle
            .write_control(
                REQ_OUT,
                MT_VEND_MULTI_WRITE,
                (addr >> 16) as u16,
                (addr & 0xffff) as u16,
                &val.to_le_bytes(),
                CTRL_TIMEOUT,
            )
            .map_err(usb_err)?;
        if n != 4 {
            return Err(init_err(format!("mt7612u wr({addr:#x}) short {n}")));
        }
        Ok(())
    }

    /// Write a 16-bit value to an FCE register (`MT_VEND_WRITE_FCE`): the value
    /// rides in `wValue`, the register index in `wIndex`, no data stage.
    fn wr_fce(&self, reg: u16, val: u16) -> Result<(), FaceError> {
        self.handle
            .write_control(REQ_OUT, MT_VEND_WRITE_FCE, val, reg, &[], CTRL_TIMEOUT)
            .map_err(usb_err)?;
        Ok(())
    }

    /// CFG-space read/write (`MT_VEND_READ_CFG` / `MT_VEND_WRITE_CFG`).
    fn rr_cfg(&self, addr: u16) -> Result<u32, FaceError> {
        let mut b = [0u8; 4];
        self.handle
            .read_control(REQ_IN, MT_VEND_READ_CFG, 0, addr, &mut b, CTRL_TIMEOUT)
            .map_err(usb_err)?;
        Ok(u32::from_le_bytes(b))
    }
    fn wr_cfg(&self, addr: u16, val: u32) -> Result<(), FaceError> {
        self.handle
            .write_control(
                REQ_OUT,
                MT_VEND_WRITE_CFG,
                0,
                addr,
                &val.to_le_bytes(),
                CTRL_TIMEOUT,
            )
            .map_err(usb_err)?;
        Ok(())
    }

    /// Read 4 bytes from the efuse-shadowed EEPROM at `offset`
    /// (`MT_VEND_READ_EEPROM`): `wValue=0`, `wIndex=offset`.
    pub fn read_efuse(&self, offset: u16) -> Result<u32, FaceError> {
        let mut b = [0u8; 4];
        self.handle
            .read_control(REQ_IN, MT_VEND_READ_EEPROM, 0, offset, &mut b, CTRL_TIMEOUT)
            .map_err(usb_err)?;
        Ok(u32::from_le_bytes(b))
    }

    /// Chip ID from EEPROM offset 0 (expect 0x7612 for the MT7612U).
    pub fn chip_id(&self) -> Result<u16, FaceError> {
        Ok((self.read_efuse(0x0000)? & 0xffff) as u16)
    }

    /// Factory MAC address (EEPROM offset 0x04).
    pub fn mac_address(&self) -> Result<[u8; 6], FaceError> {
        let lo = self.read_efuse(0x0004)?.to_le_bytes();
        let hi = self.read_efuse(0x0008)?.to_le_bytes();
        Ok([lo[0], lo[1], lo[2], lo[3], hi[0], hi[1]])
    }

    fn poll<F: Fn(u32) -> bool>(&self, addr: u32, pred: F, tries: u32) -> Result<u32, FaceError> {
        for _ in 0..tries {
            let v = self.rr(addr)?;
            if pred(v) {
                return Ok(v);
            }
            std::thread::sleep(Duration::from_millis(1));
        }
        Err(init_err(format!("mt7612u poll({addr:#x}) timed out")))
    }

    // ── Firmware download ───────────────────────────────────────────────────
    /// Send one firmware region (`data`) to MCU `offset` in `FW_CHUNK_MAX`-sized
    /// chunks. Each chunk: program the FCE DMA descriptor (target addr + len as
    /// 16-bit halves), then bulk-OUT `[4-byte info header][data][4-byte 0 pad]`
    /// to the inband-command endpoint, then kick the FCE.
    fn mcu_fw_send_data(&self, data: &[u8], offset: u32, max: usize) -> Result<(), FaceError> {
        let chunk = max - 8; // 8 bytes overhead (4 info + 4 trailer)
        let dbg = std::env::var("NDN_RADIO_EP_DEBUG").is_ok();
        let nchunks = data.len().div_ceil(chunk);
        if dbg {
            eprintln!(
                "    send_data off={offset:#x} len={} chunks={nchunks}",
                data.len()
            );
        }
        let mut pos = 0usize;
        let mut idx = 0usize;
        while pos < data.len() {
            if dbg && (idx == 0 || idx.is_multiple_of(32) || idx + 1 == nchunks) {
                eprintln!("    chunk {idx}/{nchunks}");
            }
            let cur = (data.len() - pos).min(chunk);
            let dst = offset + pos as u32;
            // FCE DMA descriptor (written as 16-bit halves via WRITE_FCE):
            //   MT_FCE_DMA_ADDR = dst,  MT_FCE_DMA_LEN = len << 16.
            self.wr_fce(MT_FCE_DMA_ADDR, (dst & 0xffff) as u16)?;
            self.wr_fce(MT_FCE_DMA_ADDR + 2, (dst >> 16) as u16)?;
            self.wr_fce(MT_FCE_DMA_LEN, 0)?; // low half of (len<<16)
            self.wr_fce(MT_FCE_DMA_LEN + 2, cur as u16)?; // high half = len

            // Inband packet: info header (flag | len) + data + 4-byte zero
            // trailer, padded to a 4-byte boundary.
            let mut buf = Vec::with_capacity(4 + cur + 8);
            buf.extend_from_slice(&(MCU_TXD_FLAG | cur as u32).to_le_bytes());
            buf.extend_from_slice(&data[pos..pos + cur]);
            buf.extend_from_slice(&[0u8; 4]); // trailer
            while buf.len() % 4 != 0 {
                buf.push(0);
            }
            self.handle
                .write_bulk(self.ep_cmd, &buf, BULK_TIMEOUT)
                .map_err(|e| {
                    init_err(format!("mt7612u fw chunk {idx} (dst {dst:#x}) bulk: {e}"))
                })?;
            // Inter-chunk handshake (from golden_init): wait for the FCE to drain
            // (MT_FCE_PSE_CTRL_GO reads 0), then write 1 to advance it to the next
            // chunk. The advance-write is essential — without it the next chunk's
            // bulk times out. A short settle is also essential on fast USB stacks
            // (Linux): reading 0x09a8 immediately after the bulk can catch the FCE
            // still-idle (DMA not started) and advance prematurely → next chunk
            // NAKs/times out. macOS's slower control path hid this. Wait for the
            // FCE to go busy first (best-effort), then drain.
            std::thread::sleep(Duration::from_millis(1));
            // DIAGNOSTIC (NDN_RADIO_EP_DEBUG): the handshake read of 0x09a8 is suspected to time
            // out on a SuperSpeed host (ODROID-C4), which the old `unwrap_or(false)` masked as
            // "not ready" → the FCE was advanced blind and the next chunk's bulk NAKed. Log the
            // read outcomes so the real behavior is measured, not guessed.
            let (mut busy_seen, mut busy_iters, mut busy_errs) = (false, 0u32, 0u32);
            for _ in 0..50 {
                match self.rr(MT_FCE_PSE_CTRL_GO) {
                    Ok(v) if v & 1 != 0 => {
                        busy_seen = true;
                        break;
                    }
                    Ok(_) => {}
                    Err(_) => busy_errs += 1,
                }
                busy_iters += 1;
                std::thread::sleep(Duration::from_millis(1));
            }
            let (mut drained, mut drain_iters, mut drain_errs) = (false, 0u32, 0u32);
            for _ in 0..200 {
                match self.rr(MT_FCE_PSE_CTRL_GO) {
                    Ok(v) if v & 1 == 0 => {
                        drained = true;
                        break;
                    }
                    Ok(_) => {}
                    Err(_) => drain_errs += 1,
                }
                drain_iters += 1;
                std::thread::sleep(Duration::from_millis(1));
            }
            if dbg && (idx <= 2 || !drained) {
                eprintln!(
                    "    chunk {idx} handshake: busy_seen={busy_seen} busy_iters={busy_iters} busy_errs={busy_errs} | drained={drained} drain_iters={drain_iters} drain_errs={drain_errs}"
                );
            }
            self.wr(MT_FCE_PSE_CTRL_GO, 1)?; // advance FCE to next chunk
            pos += cur;
            idx += 1;
        }
        Ok(())
    }

    /// FCE/USB-DMA setup before a firmware download stage — the exact sequence
    /// the kernel emits right before the first firmware chunk (golden_init):
    /// USB U3DMA bulk-enable, MCU dev-mode, FCE base-ptr + max-count + conf.
    fn fce_setup(&self) -> Result<(), FaceError> {
        let d = std::env::var("NDN_RADIO_EP_DEBUG").is_ok();
        if d {
            eprintln!("    fce: u3dma");
        }
        self.wr_cfg(MT_USB_U3DMA_CFG, 0x00c0_0020)?; // bulk TX/RX DMA enable
        // DEV_MODE (bReq 0x01) wValue=1: MCU run/dev mode.
        if d {
            eprintln!("    fce: devmode");
        }
        self.handle
            .write_control(REQ_OUT, MT_VEND_DEV_MODE, 0x0001, 0, &[], CTRL_TIMEOUT)
            .map_err(usb_err)?;
        // DEV_MODE wValue=1 switches the MCU into download mode; the device needs
        // ~9ms to settle before it accepts the next register write. The golden
        // trace shows an 8.67ms gap here vs 0.14ms between all other transfers —
        // firing the next write immediately times out and wedges the device.
        std::thread::sleep(Duration::from_millis(12));
        if d {
            eprintln!("    fce: pse_ctrl");
        }
        self.wr(MT_FCE_PSE_CTRL, 0x1)?; // 0x0800
        if d {
            eprintln!("    fce: base_ptr");
        }
        self.wr(0x09a0, 0x0040_0230)?; // MT_TX_CPU_FROM_FCE_BASE_PTR
        if d {
            eprintln!("    fce: max_count");
        }
        self.wr(0x09a4, 0x1)?; // MT_TX_CPU_FROM_FCE_MAX_COUNT
        if d {
            eprintln!("    fce: global_conf");
        }
        self.wr(MT_FCE_PDMA_GLOBAL_CONF, 0x44)?; // 0x09c4
        if d {
            eprintln!("    fce: skip_fs");
        }
        self.wr(MT_FCE_SKIP_FS, 0x3)?; // 0x0a6c
        if d {
            eprintln!("    fce: done");
        }
        Ok(())
    }

    /// Download the ROM patch (`mt76x2u_mcu_load_rom_patch`): skip the 30-byte
    /// patch header, stream the body to `MCU_ROM_PATCH_OFFSET`.
    /// Has the ROM patch already been applied to this chip? (`MT_MCU_CLOCK_CTL` bit 0.)
    ///
    /// This is the check upstream makes before every ROM-patch download and we did not, which is
    /// the whole of the difference between "reopening the radio is free" and "reopening the radio
    /// costs a physical replug". The latch is set by the patch activation and survives until the
    /// chip loses power, so it answers the question the [`MT_MCU_COM_REG0`] mailbox cannot.
    pub fn rom_patch_applied(&self) -> bool {
        matches!(self.rr(MT_MCU_CLOCK_CTL), Ok(v) if v & 1 != 0)
    }

    fn load_rom_patch(&self) -> Result<(), FaceError> {
        if ROM_PATCH.len() <= PATCH_HEADER_LEN {
            return Err(init_err("mt7612u rom patch too small".into()));
        }
        let d = std::env::var("NDN_RADIO_EP_DEBUG").is_ok();
        // ★ Skip a patch that is already in the chip — `mt76x2u_mcu_load_rom_patch` does exactly
        // this (`mt76x2/usb_mcu.c:80-83`, "ROM patch already applied"). Re-sending it over a live
        // MCU is not merely wasteful: MEASURED 2026-08-27, the second chunk's bulk write times
        // out and the device stops answering the host controller entirely.
        // `NDN_RADIO_FORCE_FW=1` overrides, for the case where the patch really must be reloaded.
        if self.rom_patch_applied() && std::env::var("NDN_RADIO_FORCE_FW").is_err() {
            eprintln!("mt7612u: ROM patch already applied (MT_MCU_CLOCK_CTL bit0) — skipping");
            return Ok(());
        }
        self.fce_setup()?;
        self.mcu_fw_send_data(
            &ROM_PATCH[PATCH_HEADER_LEN..],
            MCU_ROM_PATCH_OFFSET,
            PATCH_CHUNK_MAX,
        )?;
        if d {
            eprintln!("  rom: data sent, WMT enable ...");
        }
        // Activate the patch (mt76x2u_mcu_enable_patch + reset_wmt). WMT class
        // requests (bmRequestType=0x20, bRequest=0x01, wValue=0x12) carrying the
        // MediaTek WMT command bytes. WITHOUT THIS the patched firmware never runs
        // and the MCU never consumes ep-0x08 commands — the root cause of the
        // ~1.1s command-write timeouts. Bytes decoded from golden_init.
        const WMT_REQ: u8 = 0x20; // host->device | class | device
        let enable_patch = [
            0x6fu8, 0xfc, 0x08, 0x01, 0x20, 0x04, 0x00, 0x00, 0x00, 0x09, 0x00,
        ];
        let reset_wmt = [0x6fu8, 0xfc, 0x05, 0x01, 0x07, 0x01, 0x00, 0x04];
        self.handle
            .write_control(WMT_REQ, 0x01, 0x0012, 0x0000, &enable_patch, CTRL_TIMEOUT)
            .map_err(usb_err)?;
        std::thread::sleep(Duration::from_millis(20));
        self.handle
            .write_control(WMT_REQ, 0x01, 0x0012, 0x0000, &reset_wmt, CTRL_TIMEOUT)
            .map_err(usb_err)?;
        std::thread::sleep(Duration::from_millis(20));
        Ok(())
    }

    /// Download the main firmware (`mt76x2u_mcu_load_firmware`): parse the
    /// 32-byte header for `ilm_len`/`dlm_len`, stream ILM to `MCU_ILM_OFFSET` and
    /// DLM to `MCU_DLM_OFFSET`.
    fn load_ram_firmware(&self) -> Result<(), FaceError> {
        if RAM_FIRMWARE.len() <= FW_HEADER_LEN {
            return Err(init_err("mt7612u firmware too small".into()));
        }
        let ilm_len = u32::from_le_bytes(RAM_FIRMWARE[0..4].try_into().unwrap()) as usize;
        let dlm_len = u32::from_le_bytes(RAM_FIRMWARE[4..8].try_into().unwrap()) as usize;
        let ilm_start = FW_HEADER_LEN;
        let dlm_start = ilm_start + ilm_len;
        if dlm_start + dlm_len > RAM_FIRMWARE.len() {
            return Err(init_err(format!(
                "mt7612u fw header ilm={ilm_len} dlm={dlm_len} exceeds {} bytes",
                RAM_FIRMWARE.len()
            )));
        }
        self.fce_setup()?;
        // Experimental ILM patch: NDN_FW_ILM_PATCH="hexoff:hexval[;...]" rewrites a
        // u32 at the given ILM offset. For probing hardcoded code constants (e.g. the
        // 0x1800=6144 max-MPDU-buffer constant at ilm+0x4556) suspected to enforce
        // the ~5888B single-MPDU cap.
        let ilm_slice = &RAM_FIRMWARE[ilm_start..dlm_start];
        let ilm_patched: Option<Vec<u8>> = std::env::var("NDN_FW_ILM_PATCH").ok().map(|spec| {
            let mut v = ilm_slice.to_vec();
            for pair in spec.split(';').filter(|s| !s.is_empty()) {
                let mut it = pair.split(':');
                let off = usize::from_str_radix(it.next().unwrap().trim_start_matches("0x"), 16)
                    .unwrap_or(0);
                let val = u32::from_str_radix(it.next().unwrap().trim_start_matches("0x"), 16)
                    .unwrap_or(0);
                if off + 4 <= v.len() {
                    v[off..off + 4].copy_from_slice(&val.to_le_bytes());
                    eprintln!("  [fw ILM patched: 0x{off:05x} -> 0x{val:08x}]");
                }
            }
            v
        });
        self.mcu_fw_send_data(
            ilm_patched.as_deref().unwrap_or(ilm_slice),
            MCU_ILM_OFFSET,
            FW_CHUNK_MAX,
        )?;
        let dlm_slice = &RAM_FIRMWARE[dlm_start..dlm_start + dlm_len];
        // Experimental DLM patch: the firmware's per-bandwidth TX page-count table
        // (marker 0x3f1f1f10 then two max-page u32s = 22/23 pages = the ~5888B
        // single-MPDU cap) lives at these DLM offsets. NDN_FW_PGCNT=<n> rewrites all
        // eight fields so the firmware allows larger MPDUs (the cap is enforced in
        // firmware, not a host register — see the register hunt). Off by default.
        const DLM_PGCNT_OFFS: [usize; 8] = [
            0x33c8, 0x33cc, 0x33fc, 0x3400, 0x3430, 0x3434, 0x3464, 0x3468,
        ];
        let patched: Option<Vec<u8>> = std::env::var("NDN_FW_PGCNT")
            .ok()
            .and_then(|s| s.parse::<u32>().ok())
            .map(|pg| {
                let mut v = dlm_slice.to_vec();
                for off in DLM_PGCNT_OFFS {
                    if off + 4 <= v.len() {
                        v[off..off + 4].copy_from_slice(&pg.to_le_bytes());
                    }
                }
                eprintln!("  [fw DLM patched: 8 TX page-count fields -> {pg}]");
                v
            });
        self.mcu_fw_send_data(
            patched.as_deref().unwrap_or(dlm_slice),
            MCU_DLM_OFFSET,
            FW_CHUNK_MAX,
        )?;
        Ok(())
    }

    /// True if the MCU firmware is already running (COM_REG0 ready signature).
    /// A USB reset doesn't reset the on-chip MCU, so a re-run after a prior
    /// successful load finds firmware already up — re-downloading then would hang.
    pub fn firmware_running(&self) -> bool {
        matches!(self.rr(MT_MCU_COM_REG0), Ok(v) if v & 1 != 0 && (v >> 16) == 0x0011)
    }

    /// Is this chip already initialised enough that a cold bring-up would be destructive?
    ///
    /// [`firmware_running`](Self::firmware_running) alone is not a sound answer: it reads the
    /// MCU **mailbox**, which the firmware reuses, so it goes stale within seconds of a
    /// successful bring-up and reports "cold" on a chip that is very much warm. This adds the
    /// persistent evidence: the ROM-patch latch, plus the MAC actually being enabled.
    ///
    /// MEASURED on this dongle: after a successful bring-up plus one channel set plus 25 s of
    /// RX, `MT_MCU_COM_REG0` read `0x00090ff0` (a leftover firmware **destination address**),
    /// while `MT_MAC_SYS_CTRL` still read `0x0c` — the chip was fully up and the mailbox said
    /// nothing about it.
    pub fn already_initialised(&self) -> bool {
        if self.firmware_running() {
            return true;
        }
        let mac_enabled = matches!(self.rr(0x1004), Ok(v) if v & 0x0c == 0x0c);
        self.rom_patch_applied() && mac_enabled
    }

    /// Does the MCU answer **us**, as opposed to merely being loaded?
    ///
    /// [`already_initialised`](Self::already_initialised) answers "is firmware running", which is a
    /// weaker claim than it looks: the kernel driver's firmware satisfies it completely while
    /// leaving a chip our channel-set deltas cannot drive. This sends a real command and waits for
    /// a real answer, which is the only evidence that actually matters.
    ///
    /// Uses `MCU_CMD_RANDOM_READ` (0x0a) on a register we already know, because it is the cheapest
    /// command that requires the MCU to *compose a response*: a fire-and-forget command would
    /// "succeed" against a dead MCU and tell us nothing.
    ///
    /// ⚠ It cannot go through [`mcu_cmd`](Self::mcu_cmd), which deliberately treats a response
    /// timeout as success (correct there — with the ROM patch the MCU does not need every ACK
    /// drained for flow control). Liveness is exactly the question that distinction erases, so
    /// this does its own write/read pair and requires bytes back.
    ///
    /// Deliberately conservative — any error, timeout or empty read reports **not** responsive.
    /// A false "responsive" costs a bench trip; a false "unresponsive" costs one register replay.
    pub fn mcu_responsive(&self) -> bool {
        use std::sync::atomic::Ordering;
        let seq = (self.mcu_seq.fetch_add(1, Ordering::Relaxed).wrapping_add(1) & 0xf).max(1);
        // MCU_CMD_RANDOM_READ payload is {u32 reg, u32 val} pairs.
        let mut payload = [0u8; 8];
        payload[..4].copy_from_slice(&MT_MCU_CLOCK_CTL.to_le_bytes());
        let info = (payload.len() as u32 & 0xffff)
            | ((seq as u32) << 16)
            | ((0x0au32 & 0x7f) << 20)
            | (2u32 << 27)
            | (1u32 << 30);
        let mut buf = Vec::with_capacity(16);
        buf.extend_from_slice(&info.to_le_bytes());
        buf.extend_from_slice(&payload);
        buf.extend_from_slice(&[0u8; 4]);
        if self
            .handle
            .write_bulk(self.ep_cmd, &buf, Duration::from_millis(200))
            .is_err()
        {
            return false;
        }
        let ep_resp = *self.ep_ins.last().unwrap_or(&self.ep_in);
        let mut rx = [0u8; 512];
        matches!(
            self.handle
                .read_bulk(ep_resp, &mut rx, Duration::from_millis(300)),
            Ok(n) if n > 0
        )
    }

    /// Full firmware bring-up: ROM patch then RAM firmware (each preceded by the
    /// FCE/USB-DMA setup). Skipped if firmware is already running.
    pub fn load_firmware(&self) -> Result<(), FaceError> {
        let d = std::env::var("NDN_RADIO_EP_DEBUG").is_ok();
        if self.firmware_running() && std::env::var("NDN_RADIO_FORCE_FW").is_err() {
            if d {
                eprintln!("  load_firmware: already running, skip");
            }
            return Ok(());
        }
        if d {
            eprintln!("  load_firmware: rom patch ...");
        }
        self.load_rom_patch()?;
        if d {
            eprintln!("  load_firmware: ram firmware ...");
        }
        self.load_ram_firmware()?;
        if d {
            eprintln!("  load_firmware: done");
        }
        Ok(())
    }

    /// Start the MCU after the firmware download (`mt76x2u_mcu_load_ivb`): ack the
    /// last FCE completion, issue the IVB/run command (DEV_MODE wValue=0x12), then
    /// poll `MT_MCU_COM_REG0` for the firmware-ready bit. Returns the final
    /// COM_REG0 value and whether bit0 (ready) is set.
    pub fn start_mcu(&self) -> Result<(u32, bool), FaceError> {
        let d = std::env::var("NDN_RADIO_EP_DEBUG").is_ok();
        if d {
            eprintln!("  start_mcu: running={}", self.firmware_running());
        }
        // ⚠ `firmware_running()` reads the MCU **mailbox** and goes stale (see MT_MCU_COM_REG0),
        // so on a warm re-open it says "cold" and this used to re-send load-IVB to an MCU that was
        // already running. MEASURED cost of that mistake: the next `set_channel` took **18.6 s**
        // with 16 MCU command timeouts, against 1.3 s and zero on a chip whose MCU was left alone.
        if !self.already_initialised() {
            let _ = self.wr(MT_FCE_PSE_CTRL_GO, 0x14); // ack last FCE completion
            self.handle
                .write_control(REQ_OUT, MT_VEND_DEV_MODE, 0x0012, 0, &[], CTRL_TIMEOUT)
                .map_err(usb_err)?;
            // load-IVB (DEV_MODE wValue=0x12) hands control to the freshly loaded
            // firmware; the golden trace waits ~20ms before reading COM_REG0.
            std::thread::sleep(Duration::from_millis(20));
            if d {
                eprintln!("  start_mcu: ivb sent, polling COM_REG0 ...");
            }
            for _ in 0..200 {
                if self.rr(MT_MCU_COM_REG0)? & 1 != 0 {
                    break;
                }
                std::thread::sleep(Duration::from_millis(1));
            }
            if d {
                eprintln!(
                    "  start_mcu: COM_REG0={:#x}",
                    self.rr(MT_MCU_COM_REG0).unwrap_or(0)
                );
            }
        }
        // Firmware-ready handshake: write COM_REG0 back (golden writes 0x1140fb
        // after polling) to signal the MCU into runtime command mode. Without it
        // the MCU never consumes ep-0x08 commands (every write times out ~1.1s).
        self.wr(MT_MCU_COM_REG0, 0x0011_40fb)?;
        // Switch the USB DMA / FCE out of firmware-download mode into runtime
        // mode. REQUIRED before any MCU command or RX: without it, command writes
        // to ep 0x08 are never consumed (every one times out ~1.1s) and no RX
        // streams. From golden_init right after load-IVB.
        self.wr(MT_FCE_PSE_CTRL, 0x1)?; // 0x0800
        self.wr_cfg(MT_USB_U3DMA_CFG, 0x00c4_0020)?; // 0x9018 runtime (RX+TX bulk)
        let v = self.rr(MT_MCU_COM_REG0)?;
        Ok((v, v & 1 != 0))
    }

    /// Send an MCU command (`mt76x02u_mcu_send_msg`). Info word layout decoded
    /// from the golden trace: `LEN[15:0] | SEQ[19:16] | CMD[26:20] | PORT(2)[29:27]
    /// | TYPE_CMD(bit30)`. Frame = `[info LE][payload][4-byte trailer]`, padded to
    /// 4, to the inband-command endpoint. If `wait_resp`, read the matching ACK on
    /// the cmd-response IN endpoint (rxfce: seq must match, evt == CMD_DONE).
    pub fn mcu_cmd(&self, cmd: u8, payload: &[u8], wait_resp: bool) -> Result<(), FaceError> {
        use std::sync::atomic::Ordering;
        let mut seq = self.mcu_seq.fetch_add(1, Ordering::Relaxed).wrapping_add(1) & 0xf;
        if seq == 0 {
            seq = self.mcu_seq.fetch_add(1, Ordering::Relaxed).wrapping_add(1) & 0xf;
            if seq == 0 {
                seq = 1;
            }
        }
        let info = (payload.len() as u32 & 0xffff)
            | ((seq as u32) << 16)
            | ((cmd as u32 & 0x7f) << 20)
            | (2u32 << 27) // CPU_TX_PORT
            | (1u32 << 30); // MT_MCU_MSG_TYPE_CMD
        let mut buf = Vec::with_capacity(4 + payload.len() + 8);
        buf.extend_from_slice(&info.to_le_bytes());
        buf.extend_from_slice(payload);
        buf.extend_from_slice(&[0u8; 4]); // trailer
        while buf.len() % 4 != 0 {
            buf.push(0);
        }
        let dbg = std::env::var("NDN_RADIO_MCU_DEBUG").is_ok();
        let tw = std::time::Instant::now();
        let wres = self.handle.write_bulk(self.ep_cmd, &buf, BULK_TIMEOUT);
        let wms = tw.elapsed().as_millis();
        wres.map_err(|e| init_err(format!("mt7612u mcu_cmd 0x{cmd:02x} write({wms}ms): {e}")))?;
        if dbg && wms > 5 {
            eprintln!("  mcu 0x{cmd:02x} len{} write={wms}ms", payload.len());
        }

        if wait_resp {
            // Wait for the command ACK on the cmd-response IN endpoint
            // (MT_EP_IN_CMD_RESP = 0x85). This is REQUIRED for throughput: the MCU
            // holds its response and won't accept the next command until we read
            // it, so missing the ACK makes each subsequent command's bulk-write
            // block ~1s. read_bulk returns as soon as the response arrives, so
            // ACKing commands cost ~ms; we accept any response (seq-exact match is
            // unnecessary for serialized replay) with a bounded timeout.
            let _ = seq;
            let ep_resp = *self.ep_ins.last().unwrap_or(&self.ep_in);
            let mut rx = [0u8; 512];
            // Short drain: with the ROM patch enabled the MCU consumes commands
            // without needing us to read each ACK for flow control, so a brief
            // read just clears any response without stalling (returns immediately
            // when data is present). NDN_RADIO_MCU_RESP_MS overrides the timeout.
            let ms = std::env::var("NDN_RADIO_MCU_RESP_MS")
                .ok()
                .and_then(|s| s.parse().ok())
                .unwrap_or(200);
            let tr = std::time::Instant::now();
            let r = self
                .handle
                .read_bulk(ep_resp, &mut rx, Duration::from_millis(ms));
            if dbg {
                match &r {
                    Ok(n) => eprintln!(
                        "  mcu 0x{cmd:02x} resp {n}B in {}ms",
                        tr.elapsed().as_millis()
                    ),
                    Err(rusb::Error::Timeout) => eprintln!(
                        "  mcu 0x{cmd:02x} resp TIMEOUT {}ms",
                        tr.elapsed().as_millis()
                    ),
                    Err(_) => {}
                }
            }
            match r {
                Ok(_) | Err(rusb::Error::Timeout) => {}
                Err(e) => return Err(usb_err(e)),
            }
        }
        Ok(())
    }

    /// Replay an MCU command verbatim from its captured txd info word, preserving
    /// the exact seq. mt76 uses `seq==0` for fire-and-forget commands that post NO
    /// response; only `seq!=0` commands ACK on the cmd-response IN endpoint.
    /// Forcing a non-zero seq onto a seq-0 command makes the running firmware not
    /// drain it (the ep-0x08 FIFO fills after ~90 commands and every further write
    /// blocks ~1s), and waiting for an ACK that never comes wastes ~200ms each.
    /// So: send the info word as-is, and only read a response when seq is nonzero.
    pub fn mcu_cmd_raw(&self, info: u32, payload: &[u8]) -> Result<(), FaceError> {
        let mut buf = Vec::with_capacity(4 + payload.len() + 8);
        buf.extend_from_slice(&info.to_le_bytes());
        buf.extend_from_slice(payload);
        buf.extend_from_slice(&[0u8; 4]); // trailer
        while buf.len() % 4 != 0 {
            buf.push(0);
        }
        let dbg = std::env::var("NDN_RADIO_MCU_DEBUG").is_ok();
        let cmd = (info >> 20) & 0x7f;
        let seq = (info >> 16) & 0xf;
        let tw = std::time::Instant::now();
        let wres = self.handle.write_bulk(self.ep_cmd, &buf, BULK_TIMEOUT);
        let wms = tw.elapsed().as_millis();
        wres.map_err(|e| init_err(format!("mt7612u mcu_cmd 0x{cmd:02x} write({wms}ms): {e}")))?;
        if seq == 0 {
            if dbg && wms > 5 {
                eprintln!("  mcu 0x{cmd:02x} seq0 write={wms}ms (no-resp)");
            }
            return Ok(());
        }
        let ep_resp = *self.ep_ins.last().unwrap_or(&self.ep_in);
        let mut rx = [0u8; 512];
        let ms = std::env::var("NDN_RADIO_MCU_RESP_MS")
            .ok()
            .and_then(|s| s.parse().ok())
            .unwrap_or(200);
        let tr = std::time::Instant::now();
        let r = self
            .handle
            .read_bulk(ep_resp, &mut rx, Duration::from_millis(ms));
        if dbg {
            match &r {
                Ok(n) => eprintln!(
                    "  mcu 0x{cmd:02x} seq{seq} resp {n}B in {}ms",
                    tr.elapsed().as_millis()
                ),
                Err(rusb::Error::Timeout) => {
                    eprintln!(
                        "  mcu 0x{cmd:02x} seq{seq} resp TIMEOUT {}ms",
                        tr.elapsed().as_millis()
                    )
                }
                Err(_) => {}
            }
        }
        match r {
            Ok(_) | Err(rusb::Error::Timeout) => {}
            Err(e) => return Err(usb_err(e)),
        }
        Ok(())
    }

    /// Replay the MAC/BB MMIO init sequence captured from the kernel
    /// (`init_table::INIT_WRITES`) after firmware is up. Register-only — the
    /// RF/channel come from MCU commands (see [`bring_up`](Self::bring_up)).
    pub fn apply_init(&self) -> Result<(), FaceError> {
        for &(addr, val) in init_table::INIT_WRITES {
            self.wr(addr, val)?;
        }
        Ok(())
    }

    /// Full init: replay the golden op-stream (`init_replay.bin`) — MMIO writes,
    /// CFG writes, DEV_MODE, the firmware-load marker, and the 473 MCU commands
    /// (RF/BB programming + calibration) — in exact captured order. This is what
    /// tunes the RF for RX/TX. MCU commands are best-effort (an un-ACKed command
    /// is logged but doesn't abort the bring-up).
    pub fn bring_up(&self) -> Result<(), FaceError> {
        let dbg = std::env::var("NDN_RADIO_EP_DEBUG").is_ok();
        let force = std::env::var("NDN_RADIO_FORCE_FW").is_ok();

        let drain_pause = self.spawn_rx_drain();
        std::thread::sleep(Duration::from_millis(30));

        // WARM re-open (reliability): if the firmware is already running — from a
        // previous run of this driver or the kernel — DO NOT replay the cold
        // bring-up. Re-downloading firmware over a running MCU collides with the
        // FCE and times out a chunk, wedging the device (the recurring "warm run
        // wedges" failure). The MAC/BB init persists while firmware runs, so just
        // re-assert runtime mode via start_mcu(). `NDN_RADIO_FORCE_FW=1` overrides
        // to force the full cold replay. Poll a few times — the first register
        // read right after claim() can be racy.
        // ── Warm or cold? Decide it with a ROUND TRIP, not with a status bit. ──────────────
        //
        // ★ This decision has gone wrong in both directions on this part, and each mistake cost a
        // physical replug, so it is worth stating exactly what the evidence is:
        //
        // * The old test was `already_initialised()` — the MCU **mailbox** `MT_MCU_COM_REG0` plus
        //   the ROM-patch latch. Both are *heuristics about state*, not evidence the MCU will
        //   talk to us. MEASURED: the mailbox goes stale within seconds and reports "cold" on a
        //   chip whose firmware the kernel had just loaded; taking the cold path there downloads
        //   firmware into a live MCU, which collides with the FCE and hangs — the original
        //   "warm run wedges" failure. The ROM-patch latch fails the other way: it survives a USB
        //   reset and reports "warm" on a chip that answers nothing.
        // * `mcu_responsive()` sends a real `MCU_CMD_RANDOM_READ` and requires bytes back. It is
        //   strictly better evidence than either latch, so it **overrules both**, in both
        //   directions.
        //
        // The three outcomes, and why each is right:
        //   live                -> warm re-open. The MCU answers; re-initialising would be the
        //                          destructive act, not the safe one.
        //   !live && heuristic  -> **error out.** Firmware is loaded but not talking to us. Do NOT
        //                          try to fix it here: replaying ~5900 register writes and ~50 MCU
        //                          commands against a silent MCU leaves the FCE mid-transaction,
        //                          after which `mt76x2u`'s own probe fails `firmware upload
        //                          failed: -110` forever while `ASIC revision` still reads a
        //                          correct 0x76120044. The silicon is fine; the load path is stuck,
        //                          and only a power cycle clears it. Each attempt cost a bench trip.
        //   !live && !heuristic -> genuinely cold (no firmware yet). Full replay, as intended.
        if !force {
            let live = self.mcu_responsive();
            let heuristic = self.already_initialised();
            if dbg {
                eprintln!("  bring_up: mcu_responsive={live} already_initialised={heuristic}");
            }
            if live {
                eprintln!("mt7612u bring_up: MCU answers — warm re-open (skipping cold replay)");
                self.start_mcu()?;
                // ★ The warm path is where contention actually leaks: the chip kept the previous
                // process's EDCA precisely because it did not power-cycle. Pin here TOO.
                self.pin_edca();
                return Ok(());
            }
            if heuristic {
                return Err(init_err(
                    "mt7612u: firmware is loaded but the MCU does not answer us (a set_channel \
                     here would report all-op-errors and transmit nothing).\n  NOT attempting a \
                     register replay: that leaves the FCE mid-transaction and the part then \
                     refuses firmware upload (-110) until it is physically replugged.\n  Try \
                     `mt76_acquire.sh release <pid>` to let the kernel reload firmware, then \
                     `acquire`. If dmesg then shows `firmware upload failed: -110` alongside a \
                     good `ASIC revision`, it needs a physical replug."
                        .to_string(),
                ));
            }
        }

        // COLD path — FAITHFUL in-order replay. The post-load diff vs golden_init
        // showed our command sequence diverges: we skipped the pre-firmware
        // bootloader handshake (writes + ~51 MCU commands) the kernel runs BEFORE
        // the firmware download, so the firmware came up accepting-but-not-
        // processing commands. Replay everything in captured order; load firmware
        // at the marker. RX-drain runs throughout so command writes are accepted.

        let b: &[u8] = INIT_REPLAY;
        let mut i = 0usize;
        let (mut nw, mut nm, mut ne) = (0u32, 0u32, 0u32);
        let mut loaded_fw = false;
        while i < b.len() {
            if dbg && (nw + nm) % 500 == 0 && (nw + nm) > 0 {
                eprintln!(
                    "  ... {nw} writes + {nm} mcu, {ne} errs (op @ {i}/{})",
                    b.len()
                );
            }
            let tag = b[i];
            i += 1;
            macro_rules! exec {
                ($e:expr) => {
                    if let Err(e) = $e {
                        ne += 1;
                        if dbg && ne <= 30 {
                            eprintln!("  op err @{i} tag {tag:#04x}: {e}");
                        }
                    }
                };
            }
            match tag {
                0x06 => {
                    let addr = u32::from_le_bytes(b[i..i + 4].try_into().unwrap());
                    let val = u32::from_le_bytes(b[i + 4..i + 8].try_into().unwrap());
                    i += 8;
                    exec!(self.wr(addr, val));
                    nw += 1;
                }
                0x46 => {
                    let addr = u16::from_le_bytes(b[i..i + 2].try_into().unwrap());
                    let val = u32::from_le_bytes(b[i + 2..i + 6].try_into().unwrap());
                    i += 6;
                    exec!(self.wr_cfg(addr, val));
                }
                0x01 => {
                    let wv = u16::from_le_bytes(b[i..i + 2].try_into().unwrap());
                    i += 2;
                    // After the firmware marker, start_mcu() has already performed
                    // the runtime handoff (load-IVB + COM_REG0 + USB-DMA into runtime
                    // mode). The replay's remaining DEV_MODE writes are the captured
                    // download-mode / load-IVB switches; re-running them flips the
                    // device back out of runtime mode, so register writes still land
                    // but every MCU command write times out. Suppress them post-load.
                    if !loaded_fw {
                        exec!(
                            self.handle
                                .write_control(REQ_OUT, MT_VEND_DEV_MODE, wv, 0, &[], CTRL_TIMEOUT)
                                .map_err(usb_err)
                                .map(|_| ())
                        );
                    }
                }
                0x4d => {
                    let info = u32::from_le_bytes(b[i..i + 4].try_into().unwrap());
                    let len = u16::from_le_bytes(b[i + 4..i + 6].try_into().unwrap()) as usize;
                    i += 6;
                    let payload = &b[i..i + len];
                    i += len;
                    exec!(self.mcu_cmd_raw(info, payload));
                    nm += 1;
                }
                0xff => {
                    // Firmware download + load-IVB + runtime reconfig, in order.
                    // Pause the RX-drain: the FCE bulk download (ep 0x08) collides
                    // with concurrent ep-0x84 reads and the download times out.
                    // (Pre-fw commands still got drained; resume for post-fw cmds.)
                    use std::sync::atomic::Ordering;
                    drain_pause.store(true, Ordering::Relaxed);
                    std::thread::sleep(Duration::from_millis(60)); // let in-flight read finish
                    let r = (|| -> Result<(), FaceError> {
                        self.load_firmware()?;
                        self.start_mcu()?;
                        Ok(())
                    })();
                    drain_pause.store(false, Ordering::Relaxed);
                    r?;
                    loaded_fw = true;
                    if dbg {
                        eprintln!("  [firmware loaded @ op {i}]");
                    }
                }
                other => {
                    return Err(init_err(format!(
                        "mt7612u replay bad tag {other:#04x} @ {i}"
                    )));
                }
            }
        }
        let _ = loaded_fw;
        eprintln!("mt7612u bring_up: {nw} writes + {nm} mcu cmds, {ne} op errors");

        // ★★ Pin EDCA to a KNOWN posture. USB never power-cycles the chip between processes, so
        // contention was whatever the previous run left. MEASURED on the sibling MT7610U, five
        // consecutive processes: a run setting no posture returned 2724 or 6706 f/s — a **2.5x
        // swing decided purely by run order**, which silently turns any unpinned A/B into a
        // comparison of history. This file's own flood example names the same hazard.
        //
        // `restore_edca_defaults` already existed for this and was called only from an example.
        // It writes the boot window directly (0x2222/0x4444/0xaaaa + TXOP 0), so it CANNOT go below
        // the boot window — which matters here more than anywhere: on mt76x2 a window below boot is
        // the fault that cost two physical replugs, and `window_floor(Mt76x2)` exists because of it.
        // Non-fatal: a contention write failing must not turn a working radio into no radio.
        self.pin_edca();
        Ok(())
    }

    /// Put the MAC into promiscuous monitor RX: accept-all RX filter + enable the
    /// TX/RX MAC engines. `MT_RX_FILTR_CFG` (0x1400) = 0 accepts every frame;
    /// `MT_MAC_SYS_CTRL` (0x1004) bit2|bit3 = ENABLE_TX|ENABLE_RX.
    pub fn setup_monitor_rx(&self) -> Result<(), FaceError> {
        // USB RX bulk DMA enable. The replay excludes 0x9018 (an FCE reg), but the
        // kernel sets it to 0xc40020 post-firmware — the 0x40000 bit over the
        // firmware-load value (0xc00020) enables RX streaming. Without it the
        // device never delivers frames to bulk-IN.
        self.wr_cfg(MT_USB_U3DMA_CFG, 0x00c4_0020)?;
        // Promiscuous, minus the two "this frame is broken" drops — see the note in
        // `replay_chanset`, which re-asserts this after a tune overwrites it.
        self.wr(0x1400, crate::mt76::knobs::RX_FILTER_PROMISCUOUS_VALID)?;
        self.wr(0x1004, 0x0000_000c)?; // MT_MAC_SYS_CTRL: ENABLE_TX|ENABLE_RX
        // Arm the TSF and the channel-time counters here rather than leaving them to a caller
        // who would have to know they exist. Both are free (four register writes), both are off
        // after bring_up, and a radio that is receiving but silently has no clock and no
        // occupancy sense is the exact shape of gap this port set out to close. Failure is
        // logged, not fatal: monitor RX is the job, timekeeping is the bonus.
        if let Err(e) = self.arm_time_and_sense() {
            tracing::warn!("mt7612u: monitor RX is up but arming TSF/channel-time failed: {e}");
        }
        Ok(())
    }

    /// Tune the RF/BB to the monitor channel by replaying [`CHANSET_REPLAY`] (the
    /// kernel's `set monitor; set channel 6` op-stream: 194 RF/BB register writes
    /// and 32 calibration MCU commands). `bring_up` only does init (firmware + MAC/
    /// BB), which leaves the RF untuned — without this the receiver delivers no
    /// frames. Call after `bring_up`, before listening. Channel 6 (2.4GHz).
    pub fn set_channel_ch6(&self) -> Result<(), FaceError> {
        self.tx_bw.store(0, std::sync::atomic::Ordering::Relaxed); // 20MHz
        self.replay_chanset(CHANSET_REPLAY, "ch6/20MHz")
    }

    /// Tune the RF/BB to **5GHz channel 36 @ 80MHz (VHT80)** by replaying
    /// [`CHANSET_REPLAY_5G80`]. This is the throughput channel: 5GHz is far less
    /// congested than 2.4GHz ch6 (removing ~200µs of per-frame CSMA from the 337µs
    /// fixed TX overhead), and 80MHz bandwidth carries 4× the bits/symbol of HT20.
    /// Pair with a VHT TXWI ([`McsDescriptor::vht`]) and the 80MHz bandwidth bit
    /// (`build_data_bulk` honours `McsDescriptor::bw`). Call after `bring_up`.
    pub fn set_channel_5g80(&self) -> Result<(), FaceError> {
        self.tx_bw.store(2, std::sync::atomic::Ordering::Relaxed); // 80MHz
        self.replay_chanset(CHANSET_REPLAY_5G80, "ch36/80MHz")
    }

    /// Shared op-stream replayer for the captured channel-set blobs (see
    /// `gen_mt7612_chanset*.py`): 0x06 reg write, 0x46 cfg write, 0x01 DEV_MODE,
    /// 0x4D MCU command (verbatim info+payload). Errors are counted, not fatal —
    /// a few RF writes racing the MCU is normal and the tune still takes.
    fn replay_chanset(&self, b: &[u8], what: &str) -> Result<(), FaceError> {
        let mut i = 0usize;
        let (mut nw, mut nm, mut ne) = (0u32, 0u32, 0u32);
        while i < b.len() {
            let tag = b[i];
            i += 1;
            macro_rules! exec {
                ($e:expr) => {
                    if $e.is_err() {
                        ne += 1;
                    }
                };
            }
            match tag {
                0x06 => {
                    let addr = u32::from_le_bytes(b[i..i + 4].try_into().unwrap());
                    let val = u32::from_le_bytes(b[i + 4..i + 8].try_into().unwrap());
                    i += 8;
                    exec!(self.wr(addr, val));
                    nw += 1;
                }
                0x46 => {
                    let addr = u16::from_le_bytes(b[i..i + 2].try_into().unwrap());
                    let val = u32::from_le_bytes(b[i + 2..i + 6].try_into().unwrap());
                    i += 6;
                    exec!(self.wr_cfg(addr, val));
                }
                0x01 => {
                    let wv = u16::from_le_bytes(b[i..i + 2].try_into().unwrap());
                    i += 2;
                    exec!(
                        self.handle
                            .write_control(REQ_OUT, MT_VEND_DEV_MODE, wv, 0, &[], CTRL_TIMEOUT)
                            .map_err(usb_err)
                            .map(|_| ())
                    );
                }
                0x4d => {
                    let info = u32::from_le_bytes(b[i..i + 4].try_into().unwrap());
                    let len = u16::from_le_bytes(b[i + 4..i + 6].try_into().unwrap()) as usize;
                    i += 6;
                    let payload = &b[i..i + len];
                    i += len;
                    exec!(self.mcu_cmd_raw(info, payload));
                    nm += 1;
                }
                other => {
                    return Err(init_err(format!(
                        "mt7612u chanset bad tag {other:#04x} @ {i}"
                    )));
                }
            }
        }
        eprintln!("mt7612u set_channel {what}: {nw} writes + {nm} mcu cmds, {ne} op errors");
        // ★ RX BUG FIX (MEASURED 2026-08-27). The captured channel op-stream contains the
        // kernel's own `MT_RX_FILTR_CFG` write, so replaying it **silently overwrites whatever
        // `setup_monitor_rx` installed** — and since every caller tunes *after* bringing monitor
        // up, the replay always won. Measured on this dongle: after `setup_monitor_rx` wrote 0
        // (full promiscuity) and `set_channel_ch6` replayed, `MT_RX_FILTR_CFG` read back
        // `0x00001093` = drop CRC_ERR|PHY_ERR|VER_ERR|DUP|RTS. In the same run the PHY logged
        // **9975 CRC errors** and the host received essentially nothing — the hardware was
        // demodulating and the MAC was discarding the result before it reached USB, which
        // presents exactly like a dead antenna.
        //
        // Re-assert the monitor state after the replay. `RX_FILTER_PROMISCUOUS_VALID` rather
        // than 0: bad-FCS frames are evidence for a *sensor* but poison for a *decoder*, and
        // this path feeds `parse_dot11`. The error counts remain available, unfiltered, through
        // `read_rx_stat` — which is the honest place for them.
        self.wr(0x1400, crate::mt76::knobs::RX_FILTER_PROMISCUOUS_VALID)?;
        self.wr(0x1004, 0x0000_000c)?; // MT_MAC_SYS_CTRL: ENABLE_TX|ENABLE_RX
        Ok(())
    }

    /// Continuously drain the data bulk-IN endpoint (ep 0x84) in a background
    /// thread. mt76 USB keeps RX URBs submitted; if the host stops reading, the
    /// device's USB DMA stalls and can block the MCU command path (commands get
    /// accepted into the FIFO but never processed). Call before `bring_up`.
    pub fn spawn_rx_drain(&self) -> std::sync::Arc<std::sync::atomic::AtomicBool> {
        use std::sync::atomic::Ordering;
        let pause = self.drain_pause.clone();
        let h = self.handle.clone();
        let ep = self.ep_in;
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

    /// WLAN data-TX bulk-OUT endpoint. The kernel sends mgmt/probe frames on
    /// ep 0x07 (the AC_VO queue) on this dongle; we use the same.
    const TX_EP: u8 = 0x07;

    /// Write a pre-built USB TX bulk verbatim (`[info][TXWI][802.11][tail]`) to the
    /// WLAN data endpoint. Used to replay a captured frame for the radiation test.
    pub fn tx_raw(&self, bulk: &[u8]) -> Result<(), FaceError> {
        self.handle
            .write_bulk(Self::TX_EP, bulk, BULK_TIMEOUT)
            .map_err(usb_err)?;
        Ok(())
    }

    /// Replay the embedded captured probe-request bulk verbatim (known-good
    /// bytes) — the most reliable first radiation test.
    pub fn tx_raw_probe(&self) -> Result<(), FaceError> {
        self.tx_raw(TX_PROBE)
    }

    /// Transmit an 802.11 frame: wrap it in the mt76x02u USB TX framing
    /// (`[info u32][TXWI 20B][frame][4B tail]`, padded to 4) and write it to the
    /// WLAN data endpoint. The TXWI is templated from a captured kernel TX (basic
    /// rate, no-station wcid 0xfd) with the MPDU-length field set to this frame.
    /// info = round_up(TXWI+frame,4) | 80211(bit19) | WIV(bit24) | QSEL=2(bit26),
    /// matching the captured `0x050800f0`.
    pub fn transmit(&self, frame: &[u8]) -> Result<(), FaceError> {
        let buf = self.build_tx_bulk(frame, None);
        self.handle
            .write_bulk(Self::TX_EP, &buf, BULK_TIMEOUT)
            .map_err(usb_err)?;
        Ok(())
    }

    /// Transmit a bare 802.11 frame at a specific [`McsDescriptor`] rate (builds
    /// the TXWI rate field via [`mt76_rate_val`]). For TX rate/format diagnostics.
    pub fn transmit_mcs(&self, frame: &[u8], mcs: &McsDescriptor) -> Result<(), FaceError> {
        let buf = self.build_tx_bulk(frame, Some(mcs));
        self.handle
            .write_bulk(Self::TX_EP, &buf, BULK_TIMEOUT)
            .map_err(usb_err)?;
        Ok(())
    }

    /// Wrap a 20-byte TXWI + bare 802.11 frame in the mt76x02u USB TX framing:
    /// `[info u32][TXWI][frame][4B tail]`, padded to 4.
    /// info = round_up(TXWI+frame,4) | 80211(b19) | WIV(b24) | QSEL=2(b26).
    fn wrap_tx(&self, txwi: &[u8; 20], frame: &[u8]) -> Vec<u8> {
        let payload_len = txwi.len() + frame.len();
        let info = (((payload_len + 3) & !3) as u32) | (1 << 19) | (1 << 24) | (2 << 25);
        let mut buf = Vec::with_capacity(4 + payload_len + 8);
        buf.extend_from_slice(&info.to_le_bytes());
        buf.extend_from_slice(txwi);
        buf.extend_from_slice(frame);
        buf.extend_from_slice(&[0u8; 4]); // tail
        while buf.len() % 4 != 0 {
            buf.push(0);
        }
        buf
    }

    /// Build a TX bulk for a MANAGEMENT frame (TXWI templated from the captured
    /// probe: ack_ctl=2, wcid=0xfd). `mcs` overrides the rate field. Sent on the
    /// mgmt endpoint (ep 0x07). Used by the TX diagnostics.
    fn build_tx_bulk(&self, frame: &[u8], mcs: Option<&McsDescriptor>) -> Vec<u8> {
        let mut txwi = [0u8; 20];
        txwi.copy_from_slice(&TX_PROBE[4..24]);
        if let Some(m) = mcs {
            txwi[2..4].copy_from_slice(&mt76_rate_val(m).to_le_bytes());
        }
        txwi[6..8].copy_from_slice(&(frame.len() as u16).to_le_bytes()); // len_ctl
        self.wrap_tx(&txwi, frame)
    }

    /// Diagnostic: transmit a DATA frame on the data endpoint (0x04) with the data
    /// TXWI but an explicit raw TXWI rate word (e.g. 0x0000 CCK-1M, 0x2000 OFDM-6M,
    /// 0x4001 HT-MCS1). Isolates whether data-frame radiation depends on the rate.
    pub fn tx_data_at(&self, frame: &[u8], rate: u16) -> Result<(), FaceError> {
        let mut txwi = TXWI_DATA;
        txwi[2..4].copy_from_slice(&rate.to_le_bytes());
        txwi[6..8].copy_from_slice(&(frame.len() as u16).to_le_bytes());
        let buf = self.wrap_tx(&txwi, frame);
        self.handle
            .write_bulk(self.ep_data, &buf, BULK_TIMEOUT)
            .map_err(usb_err)?;
        Ok(())
    }

    /// USB-aggregation: pack several plain-data MPDUs into ONE bulk transfer
    /// (each `[info][TXWI][802.11][tail]` unit padded to 4). Amortizes the fixed
    /// ~0.4ms/transfer over many MPDUs — the RTL's `inject_amsdu_usbagg` lever,
    /// but with plain-data units (which radiate, unlike A-MSDU here). Returns the
    /// bytes written. The device's USB-DMA must chain units by each info.len.
    pub fn tx_data_agg(&self, frames: &[&[u8]], rate: u16) -> Result<usize, FaceError> {
        let mut buf = Vec::new();
        for f in frames {
            let mut txwi = TXWI_DATA;
            txwi[2..4].copy_from_slice(&rate.to_le_bytes());
            txwi[6..8].copy_from_slice(&(f.len() as u16).to_le_bytes());
            buf.extend_from_slice(&self.wrap_tx(&txwi, f));
        }
        self.handle
            .write_bulk(self.ep_data, &buf, BULK_TIMEOUT)
            .map_err(usb_err)
    }

    /// Enable the 2nd TX chain for VHT 2-stream / STBC. Writes the mt76x2 TX-chain
    /// mask `0x0820`: `0x31` = both chains active, `0x11` = single chain (the post-
    /// init default for monitor mode). A VHT-2SS rate word with only one chain
    /// enabled transmits as 1SS (or not at all), so call this before a 2SS sweep.
    pub fn set_tx_chains(&self, two: bool) -> Result<(), FaceError> {
        self.wr(0x0820, if two { 0x31 } else { 0x11 })
    }

    /// Zero the EDCA backoff on all four ACs — `MT_WMM_AIFSN`/`CWMIN`/`CWMAX` (0x0214/18/1c).
    ///
    /// ☠ **MEASURED HARMFUL. Do not call this expecting throughput.** It had never been run on
    /// hardware until 2026-08-28; the A/B, ch36 VHT80 2x2 MCS9 short-GI, 5650 B MPDUs, 8 TX-pump
    /// threads, on a quiet 5 GHz channel:
    ///
    /// | EDCA state | offered |
    /// |---|---|
    /// | as-initialised (AIFSN 2, CWmin 15, CWmax 1023) | 2636 f/s / **119 Mbit/s** |
    /// | this function (`MT_WMM_*`: AIFSN 1, CW 0) | 466 f/s / **21 Mbit/s** |
    /// | the other block (`MT_EDCA_CFG_AC(n)` 0x1300+4n, same idea) | 206 f/s / **9.3 Mbit/s** |
    ///
    /// **5.7x and 13x WORSE respectively.** Removing the backoff does not make this MAC
    /// transmit sooner; it makes it transmit far less. The mechanism is not established here —
    /// a CW exponent of 0 may be out of range for the arbiter, or self-collision across the four
    /// ACs may be the cost — but the direction is unambiguous and reproducible. It is the same
    /// shape as the 8812au's EDCCA result: a knob whose name promises aggression and whose
    /// effect is starvation.
    ///
    /// ★ The run also settles an open question: **both EDCA register blocks are live.** The
    /// tree carried a doubt about whether the MAC arbitrates from `MT_WMM_*` (what this writes)
    /// or from `MT_EDCA_CFG_AC(n)` (what `init_replay` programs as `0x000a4200` =
    /// TXOP 0, AIFSN 2, CWmin exp 4, CWmax exp 10). Writing *either* changes on-air behaviour
    /// enormously, so neither is dead and they are not alternatives to choose between.
    ///
    /// Kept, rather than deleted, because a knob measured to hurt is worth more than a knob
    /// nobody has tried — and because the reverse direction (raising CW to yield the medium) is
    /// the same register and may yet be useful for a politeness/coexistence experiment.
    /// Put the EDCA blocks back to the values `init_replay` programs.
    ///
    /// ☠ **This exists because [`set_edca_aggressive`](Self::set_edca_aggressive) has no undo and
    /// the warm re-open path does not provide one.** MEASURED 2026-08-28: after the aggressive
    /// A/B, every subsequent MCU command failed (`set_channel … 32 op errors`, where a healthy
    /// run reports 0) and the device stopped transmitting entirely. `bring_up` could not fix it
    /// — it correctly takes the warm path on a chip whose firmware is still running, so it never
    /// re-runs the init that owns these registers — and forcing the cold path re-downloads the
    /// ROM patch into a live MCU, which is the wedging hazard. The only clean recovery was to
    /// write the init values back.
    ///
    /// Values lifted from `init_replay.bin` itself (the last write to each address), not from a
    /// datasheet: `MT_WMM_AIFSN = 0x2222`, `MT_WMM_CWMIN = 0x4444`, `MT_WMM_CWMAX = 0xaaaa`,
    /// `MT_WMM_TXOP0/1 = 0`, and all four `MT_EDCA_CFG_AC(n) = 0x000a4200`.
    ///
    /// Any knob that can leave the MAC unable to transmit needs its restore written at the same
    /// time as the setter — the [`crate::mt76::knobs::EdccaSaved`] discipline, which this pair
    /// should adopt.
    /// Pin EDCA to the boot window, non-fatally — called on **BOTH** bring-up paths.
    ///
    /// ☠ MEASURED 2026-09-01, and this is why it is a helper rather than a line at the end of
    /// `bring_up`: the pin was written into the COLD path only, and the warm re-open returns early
    /// (`if live { start_mcu(); return Ok(()) }`). The warm path is **exactly** where contention
    /// leaks — a warm chip is by definition one that kept the previous process's state. So the fix
    /// was inert in the only case it existed for. Proven on the part: a prior run left
    /// `NDN_POSTURE=yielding` (AIFSN 0x3333 / CWMIN 0x6666 / AC0 0x000a6300) and a fresh `bring_up`
    /// read back *the same values*, unchanged.
    ///
    /// (`Owned` cannot be used to test this on mt76x2: `window_floor` clamps it to the boot window,
    /// so `Owned` == `Shared` == boot and the probe would show nothing. `Yielding` sits above the
    /// floor and is the only posture that moves these registers on this family.)
    fn pin_edca(&self) {
        if let Err(e) = self.restore_edca_defaults() {
            eprintln!(
                "mt7612u: EDCA not pinned ({e}) — contention posture is whatever the previous \
                 process left; pin NDN_POSTURE before trusting any throughput figure"
            );
        }
    }

    pub fn restore_edca_defaults(&self) -> Result<(), FaceError> {
        self.wr(0x0214, 0x0000_2222)?; // MT_WMM_AIFSN
        self.wr(0x0218, 0x0000_4444)?; // MT_WMM_CWMIN
        self.wr(0x021c, 0x0000_aaaa)?; // MT_WMM_CWMAX
        self.wr(0x0220, 0)?; // MT_WMM_TXOP0
        self.wr(0x0224, 0)?; // MT_WMM_TXOP1
        for ac in 0..4u32 {
            self.wr(0x1300 + (ac << 2), 0x000a_4200)?;
        }
        // ★ MAC timing too, and for the same reason the EDCA restore exists: `bring_up` takes the
        // warm path on a chip whose MCU is already running, so anything written into these
        // registers survives every later process start until a physical replug. These are the
        // values `init_replay.bin` leaves behind (slot 20, CC_DELAY 1; ACKTO 0x23 = 20 + SIFS 15).
        self.wr(0x1104, 0x0000_0114)?; // MT_BKOFF_SLOT_CFG
        crate::mt76::Mt76Regs::rmw(self, 0x1348, 0x0000_ffff, 0x0000_2390)?; // MT_TX_TIMEOUT_CFG
        Ok(())
    }

    /// Set the MAC slot time (and the ack timeout that must track it), returning what the
    /// register reads back as.
    ///
    /// ★ **This is the largest single contention knob on this part.** MEASURED from the init blob
    /// and confirmed live: the MT7612U boots with `MT_BKOFF_SLOT_CFG = 0x114`, a **20 µs** slot,
    /// where the MT7610U ships 9 and the MT7921AU programs 9. Backoff, AIFS and `CC_DELAY` are
    /// all counted in slots, so this one register scales every term of the DCF budget at once:
    /// at the boot EDCA (AIFSN 2, CWmin exponent 4) the budget is
    /// `SIFS 16 + 2×20 + 7.5×20 = 206 µs`, most of the ~247 µs fixed per-PPDU cost.
    ///
    /// ★ MEASURED 2026-08-28, VHT80 MCS9 2SS SGI, 5650 B, A/B/A/B: slot 20 → 9 took this part from
    /// **131.9 to 190.6 Mbit/s (+45 %)**, with the fixed cost falling 246.5 → 141.0 µs. The
    /// predicted saving is 105 µs and the measured one is 105.5 — and that agreement is itself a
    /// finding, because it excludes `CC_DELAY` from the per-frame budget (including it predicts
    /// 116 µs, 10 % high). The residual ~40 µs at slot 9 is USB and PSE, not contention.
    ///
    /// Unlike [`Self::set_edca_aggressive`], 9 µs is not an out-of-spec value — it is the ordinary
    /// 802.11a short slot every 5 GHz radio in the room is already using, and unlike the EDCA
    /// window on this part it has been written, read back and reversed repeatedly with no ill
    /// effect. On the mt76x2 this register is the *only* safe way to express
    /// [`ndn_radio_hal::ContentionPosture::Owned`]; see [`crate::mt76::knobs::window_floor`].
    pub fn set_slot_time(&self, slot_us: u8) -> Result<u8, FaceError> {
        crate::mt76::knobs::set_slot_time(self, slot_us, &self.edca_saved)?;
        crate::mt76::knobs::read_slot_time(self)
    }

    /// Read back the slot time the MAC is actually counting backoff in.
    pub fn slot_time(&self) -> Result<u8, FaceError> {
        crate::mt76::knobs::read_slot_time(self)
    }

    pub fn set_edca_aggressive(&self) -> Result<(), FaceError> {
        // ☠ **Gated, because this call cost a physical replug.** After the A/B below, every MCU
        // command on the part failed (`set_channel … 32 op errors`) and it stopped transmitting;
        // `restore_edca_defaults` did not bring it back, the kernel driver's own probe then
        // failed with `firmware upload failed: -110`, and a USB reset does not power-cycle the
        // on-chip MCU. Only unplugging it did.
        //
        // It stays in the tree because a measured-harmful knob is more useful than an untried
        // one, and because the *opposite* direction (raising CW to yield the medium for a
        // coexistence experiment) is the same register. But it must be asked for explicitly,
        // and the as-found state is saved first so the caller has an undo that the warm
        // re-open path cannot provide.
        if std::env::var_os("NDN_MT7612_EDCA_AGGRESSIVE").is_none() {
            return Err(init_err(
                "mt7612u: set_edca_aggressive is MEASURED HARMFUL (119 -> 21 Mbit/s) and has \
                 wedged this part's MCU beyond software recovery. Set \
                 NDN_MT7612_EDCA_AGGRESSIVE=1 if you mean it, and be next to the dongle."
                    .into(),
            ));
        }
        let saved = (self.rr(0x0214)?, self.rr(0x0218)?, self.rr(0x021c)?);
        eprintln!(
            "mt7612u: EDCA as-found {:#010x}/{:#010x}/{:#010x} — restore with \
             restore_edca_defaults() or these values",
            saved.0, saved.1, saved.2
        );
        self.wr(0x0214, 0x0000_1111)?; // AIFSN = 1 for all four ACs
        self.wr(0x0218, 0x0000_0000)?; // CWmin exponent 0 → CW=0 (no backoff)
        self.wr(0x021c, 0x0000_0000)?; // CWmax exponent 0
        Ok(())
    }

    /// Build a TX bulk for a DATA frame. Per the captured kernel data-frame TX,
    /// data frames use a different TXWI than mgmt — **wcid 0xff** (broadcast /
    /// no-station) and **ack_ctl 0** (broadcast → no ACK) — and go on the data AC
    /// endpoint (ep 0x04), NOT the mgmt ep 0x07. Sending a data frame with the
    /// mgmt TXWI on ep 0x07 is silently dropped by the firmware (it never
    /// radiates). `rate` sets the TXWI rate field from the frame's MCS.
    pub fn build_data_bulk(&self, frame: &[u8], mcs: &McsDescriptor) -> Vec<u8> {
        self.build_data_bulk_at(frame, mt76_rate_val(mcs))
    }

    /// The same, at an explicit raw TXWI rate word — what [`TxIntent::needs_basic_rate`] needs, so
    /// a `MostRobust` frame can be forced to legacy OFDM regardless of the stored `McsDescriptor`.
    pub fn build_data_bulk_at(&self, frame: &[u8], rate: u16) -> Vec<u8> {
        let mut txwi = TXWI_DATA;
        // Rate word + the channel bandwidth (BW[8:7]) the RF is tuned to. A VHT80
        // rate on a 20MHz BB (or vice-versa) is malformed, so the bandwidth comes
        // from `tx_bw` (set by `set_channel_5g80` = 2 = 80MHz), not the descriptor.
        //
        // ★ …but ONLY for HT/VHT. A legacy PPDU (CCK/OFDM, PHY field 0x0000/0x2000) has no wide
        // format to signal, and a non-zero BW on one is a malformed rate word — the same
        // clamp `mt76x0::rate_bw_field` applies. Without this the basic-rate override below would
        // emit "OFDM 6M at 80 MHz", which is not a thing.
        let bw = if rate & 0xe000 > MT76_RATE_OFDM6M {
            (self.tx_bw.load(std::sync::atomic::Ordering::Relaxed) as u16 & 0x3) << 7
        } else {
            0
        };
        txwi[2..4].copy_from_slice(&(rate | bw).to_le_bytes());
        txwi[6..8].copy_from_slice(&(frame.len() as u16).to_le_bytes()); // len_ctl
        self.wrap_tx(&txwi, frame)
    }

    /// Sync-send one DATA frame (build_data_bulk + write to ep_data). For the
    /// MPDU-size cap test: sweep frame sizes past the advertised ~3.8KB max MPDU
    /// and witness on a second MT7612 to see whether oversized MPDUs radiate intact
    /// or truncate at the 12-bit TXWI len_ctl (4095) / VHT max-MPDU (3895).
    pub fn tx_data_sync(&self, frame: &[u8], mcs: &McsDescriptor) -> Result<(), FaceError> {
        let buf = self.build_data_bulk(frame, mcs);
        self.handle
            .write_bulk(self.ep_data, &buf, BULK_TIMEOUT)
            .map_err(usb_err)?;
        Ok(())
    }

    /// Pause (or resume) the background RX-drain. Call `pause_drain(true)` before
    /// consuming frames with `read_rx` so the drain stops stealing them; the drain
    /// is only needed during init for MCU command flow-control.
    pub fn pause_drain(&self, paused: bool) {
        self.drain_pause
            .store(paused, std::sync::atomic::Ordering::Relaxed);
    }

    /// Read one raw bulk-IN transfer (an mt76 RX burst: RXD descriptor + 802.11).
    /// Returns the byte count (0 on timeout). For the first RX-alive check.
    pub fn read_rx(&self, buf: &mut [u8]) -> Result<usize, FaceError> {
        match self
            .handle
            .read_bulk(self.ep_in, buf, Duration::from_millis(200))
        {
            Ok(n) => Ok(n),
            Err(rusb::Error::Timeout) => Ok(0),
            Err(e) => Err(usb_err(e)),
        }
    }

    /// Firmware build versions parsed from the vendored headers (sanity check the
    /// blobs loaded correctly, no hardware needed).
    pub fn fw_versions(&self) -> (u16, u16) {
        let build = u16::from_le_bytes([RAM_FIRMWARE[8], RAM_FIRMWARE[9]]);
        let ver = u16::from_le_bytes([RAM_FIRMWARE[10], RAM_FIRMWARE[11]]);
        (build, ver)
    }

    /// Set the wire frame format for `FrameIo` (defaults to NDN ethertype).
    pub fn with_format(mut self, format: FrameFormat) -> Self {
        self.format = format;
        self
    }

    /// Decode one raw mt76 RX burst into a [`CapturedFrame`] if it is a frame in
    /// our [`FrameFormat`]. Strips the 36-byte RXD prefix + 4-byte FCE trailer,
    /// lifts RSSI from RXWI byte 18, and reuses the shared `frame::parse_dot11`.
    fn decode_rx(&self, burst: &[u8]) -> Option<CapturedFrame> {
        if burst.len() < MT76_RXD_LEN + 4 + 24 {
            return None;
        }
        let rssi = burst.get(18).map(|&b| b as i8);
        let dot11 = &burst[MT76_RXD_LEN..burst.len() - 4];
        crate::frame::parse_dot11(self.format, dot11, rssi, None, None)
    }

    /// Build an A-MSDU MPDU body: one QoS-data frame (FC subtype 8, A-MSDU-Present
    /// bit in the QoS control) carrying many `[DA|SA|len|LLC/SNAP|payload]`
    /// subframes (4-byte padded). Standard 802.11 A-MSDU — chip-independent, same
    /// as the RTL backend. This is the broadcast throughput lever (A-MPDU needs a
    /// Block-Ack that broadcast never gets; A-MSDU amortizes per-MPDU overhead).
    fn build_amsdu_body(
        &self,
        payloads: &[Bytes],
        dst: [u8; 6],
        src: [u8; 6],
    ) -> Result<Vec<u8>, FaceError> {
        use std::sync::atomic::Ordering;
        let ethertype = match self.format {
            FrameFormat::RawNdn { ethertype } => ethertype,
            other => {
                return Err(init_err(format!(
                    "mt7612u A-MSDU: format {other:?} unsupported"
                )));
            }
        };
        let seq = self.seq.fetch_add(1, Ordering::Relaxed) & 0x0fff;
        let mut out = Vec::new();
        out.extend_from_slice(&[0x88, 0x00]); // FC: Data, subtype QoS Data
        out.extend_from_slice(&[0x00, 0x00]); // Duration
        out.extend_from_slice(&dst); // addr1 (RA)
        out.extend_from_slice(&src); // addr2 (TA)
        out.extend_from_slice(&dst); // addr3 (BSSID)
        out.extend_from_slice(&(seq << 4).to_le_bytes()); // SeqCtrl
        out.extend_from_slice(&[0x80, 0x00]); // QoS Ctrl: A-MSDU Present (bit7), TID 0
        let last = payloads.len() - 1;
        for (i, p) in payloads.iter().enumerate() {
            let msdu_len = 8 + p.len(); // LLC/SNAP + payload
            out.extend_from_slice(&dst);
            out.extend_from_slice(&src);
            out.extend_from_slice(&(msdu_len as u16).to_be_bytes()); // Length (big-endian)
            out.extend_from_slice(&[0xaa, 0xaa, 0x03, 0x00, 0x00, 0x00]); // LLC/SNAP
            out.extend_from_slice(&ethertype.to_be_bytes());
            out.extend_from_slice(p);
            if i != last {
                let sub_len = 14 + msdu_len;
                out.extend(std::iter::repeat_n(0u8, (4 - (sub_len % 4)) % 4));
            }
        }
        Ok(out)
    }

    /// Transmit several payloads as one A-MSDU (QoS-data) frame on the data
    /// endpoint. The throughput path for `inject_batch`.
    pub async fn inject_amsdu(
        &self,
        payloads: &[Bytes],
        mcs: McsDescriptor,
        dst: [u8; 6],
        src: [u8; 6],
    ) -> Result<(), FaceError> {
        let body = self.build_amsdu_body(payloads, dst, src)?;
        let buf = self.build_data_bulk(&body, &mcs);
        self.send_bulk(buf).await
    }

    /// Spawn `depth` dedicated TX-pump threads. `inject`/`inject_amsdu` build a USB
    /// bulk + enqueue (no per-frame `spawn_blocking`); each thread locks only for a
    /// fast `try_recv`, then does the slow `write_bulk` OUTSIDE the lock — so up to
    /// `depth` bulk transfers are in flight at once and the host controller
    /// pipelines them, hiding the ~0.37ms per-transfer round-trip that bounds a
    /// single writer. Call after `bring_up`. (Frame order across threads is not
    /// preserved — fine for connectionless NDN broadcast.)
    pub fn spawn_tx_pump(
        self: &std::sync::Arc<Self>,
        depth: usize,
    ) -> Vec<std::thread::JoinHandle<()>> {
        use std::sync::atomic::Ordering;
        // ★ BOUNDED. An unbounded queue makes `inject` a non-blocking enqueue with no
        // backpressure at all: MEASURED on the sibling MT7921AU, a 3-second flood queued
        // hundreds of thousands of frames that then took minutes to drain, so every throughput
        // figure taken that way was the speed of a channel send and the radio was still
        // transmitting long after the test thought it had stopped. Depth x 4 keeps every writer
        // fed across a scheduling hiccup while making the caller wait on the radio, not on a
        // queue.
        let (tx, rx) = std::sync::mpsc::sync_channel::<Vec<u8>>(depth.max(1) * 4);
        *self.tx_sender.lock().unwrap() = Some(tx);
        let rx = std::sync::Arc::new(std::sync::Mutex::new(rx));
        (0..depth.max(1))
            .map(|_| {
                let me = self.clone();
                let rx = rx.clone();
                std::thread::spawn(move || {
                    loop {
                        // Block rather than poll: the 50 us sleep this replaces put a floor
                        // under per-frame latency and burned a core doing it.
                        // ⚠ Poison-tolerant: a bare `.unwrap()` here means ONE panicking pump
                        // thread poisons the shared receiver and silently kills every OTHER TX
                        // thread — the radio then transmits at a fraction of its rate with no
                        // error anywhere. The MT7921AU pump already did this; the other two copies
                        // of this loop did not, which is what three hand-copies of one loop costs.
                        let buf = match rx.lock().unwrap_or_else(|e| e.into_inner()).recv() {
                            Ok(b) => b,
                            Err(_) => break, // sender dropped
                        };
                        if let Ok(n) =
                            me.handle
                                .write_bulk(me.ep_data, &buf, Duration::from_secs(1))
                        {
                            me.tx_bytes.fetch_add(n as u64, Ordering::Relaxed);
                            me.tx_count.fetch_add(1, Ordering::Relaxed);
                        }
                    }
                })
            })
            .collect()
    }

    /// Bytes / frames the TX pump has written so far (throughput / drain measurement).
    pub fn tx_bytes_written(&self) -> u64 {
        self.tx_bytes.load(std::sync::atomic::Ordering::Relaxed)
    }
    pub fn tx_count_written(&self) -> u64 {
        self.tx_count.load(std::sync::atomic::Ordering::Relaxed)
    }

    /// Create an async USB TX ring on the data endpoint (`max_outstanding` URBs in
    /// flight). The pipelined-transfer path that breaks the ~0.7ms/transfer
    /// synchronous ceiling. Pause the RX drain before saturating TX — the ring's
    /// event thread owns all libusb completions on this context.
    #[cfg(target_os = "linux")]
    pub fn new_tx_ring(&self, max_outstanding: usize) -> std::sync::Arc<TxRing> {
        std::sync::Arc::new(TxRing::new(
            self.handle.clone(),
            self.ep_data,
            max_outstanding,
        ))
    }

    /// Send a pre-built USB TX bulk: fast path hands it to the TX pump thread
    /// (no per-frame `spawn_blocking`); otherwise a one-off `spawn_blocking`
    /// `write_bulk`. Used by `inject` and `inject_amsdu`.
    async fn send_bulk(&self, buf: Vec<u8>) -> Result<(), FaceError> {
        let sender = self.tx_sender.lock().unwrap().clone();
        if let Some(s) = sender {
            return s
                .send(buf)
                .map_err(|_| init_err("mt7612u: TX pump closed".into()));
        }
        let ep = self.ep_data;
        let handle = self.handle.clone();
        tokio::task::spawn_blocking(move || {
            handle
                .write_bulk(ep, &buf, Duration::from_secs(1))
                .map_err(usb_err)
                .and_then(|n| {
                    (n == buf.len()).then_some(()).ok_or_else(|| {
                        init_err(format!("mt7612u TX: short write {n}/{}", buf.len()))
                    })
                })
        })
        .await
        .map_err(|e| init_err(format!("mt7612u TX: join {e}")))?
    }

    /// Keep `depth` bulk-IN reads outstanding so the RX FIFO never stalls; `recv_frame` then drains
    /// the queue instead of doing its own blocking read. The mt76 USB continuous-URB analogue.
    ///
    /// Delegates to the shared [`spawn_rx_pump`](crate::rx_pump::spawn_rx_pump) (#80). The
    /// hand-rolled copy this replaces had two defects the shared pump does not:
    ///
    /// * **It leaked the backend.** Each thread captured a strong `Arc<Self>` and looped forever
    ///   with no exit, so the backend could never be dropped, the USB handle never released, and the
    ///   threads never joined — live on the NDN path, since [`bringup_ndn`](Self::bringup_ndn) calls
    ///   this. The shared pump holds a `Weak` and breaks when the backend goes away.
    /// * **A 16 KB read buffer.** The shared pump uses 32 KB precisely because a USB-aggregated
    ///   bulk-IN transfer can exceed 16 KB, and a short buffer truncates it.
    ///
    /// It also brings `NDN_RX_AGG_DBG`, which reports average bytes per transfer — the instrument
    /// for the still-open question below.
    pub fn spawn_rx_pump(
        self: &std::sync::Arc<Self>,
        depth: usize,
    ) -> Vec<std::thread::JoinHandle<()>> {
        self.pause_drain(true);
        crate::rx_pump::spawn_rx_pump(self, depth)
    }

    /// On-air-verified maximum single-MPDU 802.11 payload for this chip: plain DATA
    /// frames radiate intact to ~5700 B (6000 B+ are dropped by firmware — see the
    /// size-cap investigation; the margin keeps clear of the cliff). This is the
    /// recommended `MonitorWifiFace::with_mtu` value — bigger frames amortise the
    /// ~300 µs/MPDU fixed overhead (≈142 Mb/s at VHT80 2×2 SGI vs ≈37 at 1500 B).
    /// The A-MSDU/A-MPDU aggregation that would push past this is firmware-gated
    /// (broadcast) / unicast-only on the MT7612 — see docs/AMPDU_PORT_SCOPE.md.
    pub const MAX_MPDU_PAYLOAD: usize = 5650;

    /// One-call bring-up for the high-throughput NDN path: firmware → 5 GHz ch36
    /// VHT80 → both TX chains (2 spatial streams) → background TX pump (pipelined,
    /// RX-compatible — unlike the libusb async ring) → RX pump (continuous capture).
    /// Afterwards the [`FrameIo`] surface injects at full rate and `recv_frame`
    /// drains captured frames. Build the face with `with_mtu(Self::MAX_MPDU_PAYLOAD)`
    /// and inject at a VHT MCS9 2SS short-GI [`McsDescriptor`] for the ~142 Mb/s
    /// ceiling. (Needs a cold device; warm re-open wedges — physical replug.)
    pub fn start_high_throughput(self: &std::sync::Arc<Self>) -> Result<(), FaceError> {
        self.bring_up()?;
        // The 5 GHz blob is a ch6→ch36/80 delta; establish the 2.4 GHz baseline
        // first (a cold device has no prior channel state for the delta to build on).
        self.set_channel_ch6()?;
        self.set_channel_5g80()?;
        self.setup_monitor_rx()?;
        self.set_tx_chains(true)?; // 2 spatial streams (VHT 2SS)
        self.spawn_tx_pump(32); // pipelined TX
        self.spawn_rx_pump(2); // continuous bulk-IN capture
        Ok(())
    }
}

#[async_trait]
impl FrameIo for Mt7612uBackend {

    /// This radio's own capability, so a face built from the bare `dyn FrameIo` does not have to
    /// invent one. Delegates to this type's [`RadioProfile`] — the single source of truth.
    fn radio_capability(&self) -> Option<ndn_radio_hal::RadioCapability> {
        Some(<Self as ndn_radio_hal::RadioProfile>::capability(self))
    }
    async fn inject(&self, frame: InjectFrame) -> Result<(), FaceError> {
        // ★ **Refuse an over-long MPDU rather than letting it reset the radio.**
        //
        // MEASURED 2026-08-28: `MAX_MPDU_PAYLOAD` is a real hardware limit, not a cautious
        // guess. At 5650 B this part sustains 2898 f/s / 131 Mbit/s; at **7000 B it collapses to
        // 118 f/s**, and larger sizes take the device off the bus and back (the USB device
        // number changed under us, 021 -> 024) leaving it degraded. An oversized frame is
        // therefore not a slow frame — it is a self-inflicted radio reset, and a caller that
        // sets too large an MTU should learn that from an error rather than from a dead link.
        if frame.payload.len() > Self::MAX_MPDU_PAYLOAD {
            return Err(FaceError::Io(io::Error::other(format!(
                "mt7612u: payload {} B exceeds MAX_MPDU_PAYLOAD {} — MEASURED to reset this \
                 radio, not merely to be dropped. Fragment above this seam, or lower the MTU.",
                frame.payload.len(),
                Self::MAX_MPDU_PAYLOAD
            ))));
        }
        let dot11 = crate::frame::build_dot11(self.format, &frame)?;
        // NDN frames are 802.11 DATA frames → data TXWI (wcid 0xff, no-ACK) on the
        // data AC endpoint (0x04). Mgmt TXWI/ep 0x07 would be dropped (see
        // build_data_bulk / docs/RADIO_SUBSYSTEM.md).
        // Rate is bearer state: the control-plane-set MCS if present, else resolve the
        // frame's intent (the MT7612U is 11ac-capable).
        // ★ Intent overrides the stored rate (2026-09-01). `resolved_mcs` consults the frame only
        // when `cur_mcs` is unset, so once the control plane named a rate, the traffic whose whole
        // purpose is that the worst receiver decodes it went out at that throughput rate.
        let buf = if frame.tx.needs_basic_rate() {
            self.build_data_bulk_at(&dot11, MT76_RATE_OFDM6M)
        } else {
            self.build_data_bulk(&dot11, &self.resolved_mcs(&frame))
        };
        self.send_bulk(buf).await
    }

    /// Rate as bearer state: store the exact MCS every subsequent `inject` uses.
    fn set_rate(&self, mcs: crate::McsDescriptor) -> Result<(), FaceError> {
        *self.cur_mcs.lock().unwrap() = Some(mcs);
        Ok(())
    }

    /// Inject a batch. **Unlike the RTL8812EU, the MT7612 does NOT A-MSDU-bundle
    /// here** — host-built A-MSDU (QoS-data with A-MSDU-present) is firmware-gated
    /// on monitor injection and never radiates (verified 0/200 on air; see
    /// `inject_amsdu` / docs). So each NDN packet goes as its own plain DATA MPDU.
    /// The broadcast throughput levers on this chip are instead:
    ///   1. the background **TX pump** ([`spawn_tx_pump`](Self::spawn_tx_pump)),
    ///      which pipelines these per-frame injects (RX-compatible, unlike the
    ///      libusb async ring), and
    ///   2. a **large send MTU** — plain frames radiate intact to ~5700 B
    ///      ([`MAX_MPDU_PAYLOAD`](Self::MAX_MPDU_PAYLOAD)), so the link service
    ///      packs more NDN bytes per frame and amortises the ~300 µs/MPDU fixed
    ///      overhead → ~142 Mb/s at VHT80 2×2 SGI (vs ~37 Mb/s at a 1500 B MTU).
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
            let handle = self.handle.clone();
            let ep = self.ep_in;
            let burst = tokio::task::spawn_blocking(move || {
                let mut b = vec![0u8; 8192];
                match handle.read_bulk(ep, &mut b, Duration::from_millis(200)) {
                    Ok(n) => {
                        b.truncate(n);
                        Ok(Some(b))
                    }
                    Err(rusb::Error::Timeout) => Ok(None),
                    Err(e) => Err(usb_err(e)),
                }
            })
            .await
            .map_err(|e| init_err(format!("mt7612u recv_frame: join {e}")))??;
            let Some(burst) = burst else { continue };
            if let Some(cap) = self.decode_rx(&burst) {
                return Ok(cap);
            }
            // else: timeout, or not a frame in our format (beacon/other) — keep reading
        }
    }
}

/// **What this radio is** (#79/#83) — a 2×2 dual-band part, declared rather than assumed.
///
/// Until now the mt7612 implemented only `FrameIo`, so a caller had to hand-write a
/// `RadioCapability` for it and the planner believed whatever it was told. `RadioBearer::effective_cap`
/// prefers this over the caller's assertion, so what a control plane registers is now the chip's own
/// account of itself.
///
/// The numbers come from the driver's own tune streams: 2 spatial streams, 11ac VHT, and the 5 GHz
/// ch36 **VHT80** path (`max_bw = 2`) that `set_channel_ch36_vht80` drives — the throughput path this
/// backend exists for. 2.4 GHz ch6 is supported too and listed here; the driver's captured op-streams
/// are exactly these two channels, so the list is what it can actually tune, not the band's full span.
///
/// **`RadioTime` is deliberately NOT implemented.** This backend passes `None` for the per-frame
/// stamp — it reads no TSF — so an impl would declare zero clock sources and answer `None` to every
/// read. That would complete the trait matrix while delivering nothing, which is precisely the
/// decided-but-unactuated defect the matrix exists to expose. It needs a real MT76 TSF read first
/// (#74-class work).
impl Mt7612uBackend {
    /// The part's capability, as a free function: a static fact about the silicon, not about any
    /// open handle. Stated this way so it is checkable without the dongle plugged in — a capability
    /// only assertable on hardware is one nothing verifies.
    pub fn declared_capability() -> RadioCapability {
        RadioCapability {
            bands: vec![Band::Band2_4GHz, Band::Band5GHz],
            rate: RateCapability::Wifi {
                max_mcs: 9,
                max_nss: 2,
                max_bw: 2,
            },
            // ★ This part has NO power actuator: `set_tx_power` is not implemented, and the
            // registers are known (`MT_TX_PWR_CFG_0..9`, `MT_TX_ALC_CFG_0..4`) but the per-rate
            // packing needs an EEPROM target-power/delta parse to mean anything. Declaring the
            // absence is the point — until this field existed the radio accepted every back-off
            // cognition asked for and nothing upstream could tell.
            min_tx_power: None,
            db_per_power_idx: None,
            power_actuated: false,
            // ★ MEASURED, not inherited (2026-08-31 audit). The spread below supplies
            // `max_payload: 1500` — the preset's unmeasured default — while `inject` guards at
            // `Self::MAX_MPDU_PAYLOAD` = 5650, "a real hardware limit, not a cautious guess".
            // Declaring 1500 against a 5650 guard hides 3.8x of the payload lever, which on this
            // family is the dominant throughput knob (per-frame cost dominates; width and rate do
            // not move it). One number now, read from the guard itself.
            max_payload: Self::MAX_MPDU_PAYLOAD,
            ..RadioCapability::wifi_monitor_5ghz(vec![6, 36])
        }
    }
}

/// The mt76 side of the shared RX pump (#80).
///
/// **`parse_transfer` returns at most one frame, and that is a preserved limitation, not a design.**
/// The trait returns a `Vec` because a chip may pack several RX units into one bulk-IN transfer;
/// `decode_rx` instead treats the whole transfer as a single unit — it slices
/// `[MT76_RXD_LEN .. len-4]` and ignores the MT_RX_INFO length field in the first 4 bytes. If mt76
/// USB RX does aggregate, this drops every unit after the first *and* mis-parses the concatenation.
///
/// That is deliberately **not** changed here. This port is otherwise behaviour-identical, and the
/// aggregation question needs the wire answered on silicon, not guessed: run with `NDN_RX_AGG_DBG=1`
/// and compare average bytes/transfer against single-frame sizes. Reading the length field would be
/// the fix if it aggregates — but a "fix" written blind against a format nobody measured is how this
/// codebase accumulated the defects the surrounding work has been removing.
///
/// **Unvalidated on silicon.** No MT7612U is attached to either OPi; the MediaTek part on o5p-1 is
/// an MT7610U (`0e8d:7610`, bound to `mt76x0u`) — a different, 1×1 chip this backend's PID list does
/// not even match. The leak and buffer-size fixes above are provable by inspection; the RX path
/// itself has not been exercised.
impl crate::rx_pump::Pumpable for Mt7612uBackend {
    fn pump_handle(&self) -> Arc<DeviceHandle<Context>> {
        self.handle.clone()
    }

    fn pump_bulk_in(&self) -> u8 {
        self.ep_in
    }

    fn parse_transfer(&self, buf: &[u8]) -> Vec<CapturedFrame> {
        self.decode_rx(buf).into_iter().collect()
    }

    fn pump_state(&self) -> &crate::rx_pump::RxPumpState {
        &self.rx
    }
}

/// The shared mt76x02 register seam. Everything in [`crate::mt76::knobs`] is written against this
/// trait rather than against a backend, so the knobs MEASURED on the MT7610U (which shares
/// `mt76x02_regs.h` with this part) apply here without a second implementation — and, in the
/// period when this dongle was off the bus, could be developed against the part that was up.
impl crate::mt76::Mt76Regs for Mt7612uBackend {
    fn rr(&self, addr: u32) -> Result<u32, FaceError> {
        Mt7612uBackend::rr(self, addr)
    }
    fn wr(&self, addr: u32, val: u32) -> Result<(), FaceError> {
        Mt7612uBackend::wr(self, addr, val)
    }
}

impl Mt7612uBackend {
    /// Arm the two hardware senses this part has and the driver never switched on:
    /// the free-running TSF and the channel-time counters.
    ///
    /// Neither is on after `bring_up`, and neither was reachable before because the
    /// register addresses in the tree were wrong. MEASURED 2026-08-27 on this exact
    /// dongle: with `MT_BEACON_TIME_CFG` (0x1114) bit 16 set and `SYNC_MODE`
    /// cleared, `MT_TSF_TIMER_DW0` (0x111c) advanced +10415 / +10456 / +10545 /
    /// +10389 / +10430 against ~10.4 ms host steps — 1.000 µs per tick — where the
    /// as-found register read a constant 0. (The 2026-08-18 "the mt76 TSF does not
    /// tick" result read 0x1104, which is `MT_BKOFF_SLOT_CFG`; it reported the
    /// constant `0x114`, and this part still reads exactly that there.)
    ///
    /// Call after [`setup_monitor_rx`](Self::setup_monitor_rx). Idempotent.
    pub fn arm_time_and_sense(&self) -> Result<(), FaceError> {
        use crate::mt76::knobs;
        knobs::enable_tsf(self)?;
        knobs::enable_channel_time_counters(self)?;
        *self.ct_last.lock().unwrap() = std::time::Instant::now();
        Ok(())
    }

    /// One window of channel occupancy: decode-busy per-mille, energy-detect
    /// per-mille, and the raw counters. See [`crate::mt76::knobs::ChannelTime`].
    pub fn sample_channel_time(&self) -> Result<(crate::mt76::knobs::ChannelTime, u32), FaceError> {
        let ct = crate::mt76::knobs::read_channel_time(self)?;
        let mut last = self.ct_last.lock().unwrap();
        let window_us = last.elapsed().as_micros().min(u32::MAX as u128) as u32;
        *last = std::time::Instant::now();
        Ok((ct, window_us))
    }

    /// This device's port-TSF clock domain.
    pub fn tsf_domain(&self) -> ClockDomainId {
        self.tsf_domain
    }

    /// The saved-ED-CCA slot backing [`RadioKnobs::set_edcca_ignore`].
    pub(crate) fn edcca_slot(&self) -> &crate::mt76::knobs::EdccaSaved {
        &self.edcca_saved
    }

    /// The saved-EDCA slot backing [`RadioKnobs::set_contention`].
    pub(crate) fn edca_slot(&self) -> &crate::mt76::knobs::EdcaSaved {
        &self.edca_saved
    }
}

/// ★ **This impl exists because the register map in this driver was wrong, not because the silicon
/// changed.** The tree recorded, as MEASURED, that the mt76x2 TSF "is static even after enabling
/// `MT_BEACON_TIME_CFG` bit4" — citing 0x1104 / 0x1108 / 0x110c and 0x1100. In `mt76x02_regs.h`
/// those are `MT_BKOFF_SLOT_CFG`, an unnamed word, `MT_CH_TIME_CFG` and `MT_XIFS_TIME_CFG`: four
/// configuration registers and no counter among them. The TSF is 0x111c/0x1120, its enable is bit
/// **16** of 0x1114, and it runs at 1.000 MHz — re-measured on this dongle on 2026-08-27.
///
/// What that does and does not buy:
///
/// * **Does**: a real read-now [`RadioClockKind::PortTsf`], so this radio can date its own airtime
///   against hardware instead of against a host clock, and a scheduler has a shared reference the
///   MAC itself honours.
/// * **Does not**: common view. That needs a per-frame RX stamp, and `struct mt76x02_rxwi`
///   (`mt76x02_mac.h:97`) has no timestamp field — `mt76x02_mac_process_rx` never sets
///   `status->mactime`. Substituting a register read is not available either: an EP0 round trip
///   MEASURED **91.6 µs** on this SuperSpeed part (151 µs on the high-speed MT7610U), against a
///   200 µs common-view guard. So `can_common_view` stays false, and it stays false for a reason
///   that is now written down with the right register names.
///
/// `precision_ns` therefore describes the *stamp*, not the read: [`RadioTimeSource::port_tsf`]
/// derives it from the latch point. The cost of reading it is the 91.6 µs above, and any caller
/// putting this on a per-frame path is making a mistake this doc cannot prevent.
/// **The MT7612U's declared time surface.** A free function so the declaration is assertable in a
/// unit test without a USB device.
///
/// Reference: **UNKNOWN**, ⚠ corrected 2026-08-31 — this said `crystal()` and cited evidence that is
/// not in this backend.
///
/// The citation was: "the MAC does not accept register writes until `MT_CMB_CTRL`'s `XTAL_RDY`
/// (BIT 22) asserts, i.e. the whole chip is gated on a crystal starting". Nothing in `src/mt7612/`
/// reads that register. `MT_CMB_CTRL_XTAL_RDY` resolves to a bare `pub const` (`mt76/regs.rs`) plus
/// three polls that are all in the **mt76x0** path (`mt76x0/mod.rs`, `mt76x0/mcu.rs`) — a sibling
/// driver's bring-up behaviour, described as if it were this one's. This backend brings the chip up
/// by replaying a captured init table and never polls it.
///
/// Nor does the rate rescue it. The TSF was MEASURED at "1.000 us/tick" — +10415 / +10456 / +10545 /
/// +10389 / +10430 ticks against **~10.4 ms** host steps — but the reference leg of that comparison
/// is itself approximate, so the check bounds the rate at percent level. An RC oscillator is
/// percent-class (this tree's own: +2253 ppm, ~-3100 ppm), i.e. INSIDE that spread. So the scale
/// check cannot even exclude an RC, let alone establish a crystal.
///
/// Consequence today: none — a `PortTsf` is not a per-frame stamp, so `can_common_view` is false on
/// the LATCH axis whatever the reference is. It is declared anyway, because "false because there is
/// no per-frame stamp" and "false because nobody has established the oscillator" are different
/// facts, and this part is one dark-bytes discovery away (`examples/mt7610_bringup.rs` stage 7,
/// `rxwi.bbp_rxinfo[0..3]`) from the reference half being the only thing between it and a
/// common-view claim.
///
/// To EARN `Crystal` here: a rate regression against the host clock at real precision (as `mt7921`
/// did — 15_007_757 / 15_007_907 ticks over a known 15 s), or a documented read of a crystal
/// trim/ready bit in THIS driver's own path.
fn mt7612_time_sources(tsf_domain: ClockDomainId) -> Vec<RadioTimeSource> {
    vec![RadioTimeSource {
        reference: ndn_radio_hal::ClockReference::unknown(),
        ..RadioTimeSource::port_tsf(tsf_domain)
    }]
}

impl RadioTime for Mt7612uBackend {
    fn time_sources(&self) -> Vec<RadioTimeSource> {
        mt7612_time_sources(self.tsf_domain)
    }

    /// Read the 64-bit port TSF (µs). Wrap-safe across the DW0 carry.
    ///
    /// ⚠ Returns `Ok(None)` — not an error — when the timer is not armed, because a stopped
    /// counter reads a perfectly plausible zero and "the clock is off" must not be mistaken for
    /// "the epoch just started". Arm it with [`arm_time_and_sense`](Self::arm_time_and_sense).
    fn read_clock(&self, domain: ClockDomainId) -> Result<Option<u64>, FaceError> {
        if domain != self.tsf_domain {
            return Ok(None);
        }
        if !crate::mt76::knobs::tsf_running(self)? {
            return Ok(None);
        }
        crate::mt76::knobs::read_tsf(self).map(Some)
    }
}

impl RadioProfile for Mt7612uBackend {
    fn capability(&self) -> RadioCapability {
        Self::declared_capability()
    }
}

// Marker only: `inject_at` is the derived HAL default (`set_rate` + `inject`).

impl Mt7612uBackend {
    /// The rate to transmit `frame` at: the control-plane-set MCS (state) if present,
    /// else the frame's intent resolved to this 11ac radio.
    fn resolved_mcs(&self, frame: &InjectFrame) -> crate::McsDescriptor {
        self.cur_mcs.lock().unwrap().unwrap_or_else(|| {
            crate::McsDescriptor::for_intent(&frame.tx, crate::MAX_RELIABLE_MCS, true, false)
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use ndn_radio_hal::{ClockReferenceKind, FaceTimeProfile, TxDiscipline};

    /// A `RadioTime` over a fixed source list, so a DECLARATION can go through
    /// `FaceTimeProfile::derive` without a USB device.
    struct Declared(Vec<RadioTimeSource>);
    impl RadioTime for Declared {
        fn time_sources(&self) -> Vec<RadioTimeSource> {
            self.0.clone()
        }
    }

    /// ★ **This backend claims no reference, because it witnesses none.** It used to declare
    /// `Crystal` on the strength of `MT_CMB_CTRL`'s `XTAL_RDY` gate — which is real, and is in the
    /// **mt76x0** driver, not this one. `src/mt7612/` never reads that register (it replays a
    /// captured init table), and its TSF rate check is against "~10.4 ms" host steps, a percent-level
    /// bound that does not even exclude a percent-class RC.
    ///
    /// Nil consequence today, pinned here so it stays that way: a `PortTsf` fails common view on the
    /// LATCH axis regardless. The point is that if a per-frame stamp is ever found in the undecoded
    /// RXWI dwords, an unearned `Crystal` would convert straight into `can_common_view = true` with
    /// no new evidence.
    #[test]
    fn the_declared_reference_is_unknown_because_nothing_here_witnesses_one() {
        let dom = ClockDomainId(0x7612);
        let v = mt7612_time_sources(dom);
        assert_eq!(v.len(), 1);
        assert_eq!(v[0].kind, ndn_time::RadioClockKind::PortTsf);
        assert_eq!(v[0].domain, dom);
        assert_eq!(
            v[0].reference.kind,
            ClockReferenceKind::Unknown,
            "no code in this backend establishes the oscillator"
        );
        assert!(!v[0].reference.holds_rate());
        assert_eq!(v[0].reference.measured, None);

        let p = FaceTimeProfile::derive(&Declared(v), TxDiscipline::BestEffort);
        assert!(
            !p.hw_rx_stamp,
            "no per-frame stamp: mt76x02_rxwi has no timestamp"
        );
        assert!(!p.can_common_view);
        assert_eq!(
            p.clock_reference.map(|r| r.kind),
            Some(ClockReferenceKind::Unknown),
            "and the report can say WHICH half is missing"
        );
    }

    /// The sibling comparison, mechanically: the evidence belongs to the mt76x0 path, and the two
    /// declarations must not be copies of each other. (`mt7610_time_sources` keeps `Crystal` — it is
    /// the one that polls the crystal-ready bit.)
    #[test]
    fn the_two_mt76_siblings_do_not_share_a_reference_claim() {
        let a = mt7612_time_sources(ClockDomainId(1))[0].reference.kind;
        let b = crate::mt76x0::mt7610_time_sources(ClockDomainId(2))[0]
            .reference
            .kind;
        assert_eq!(a, ClockReferenceKind::Unknown);
        assert_eq!(b, ClockReferenceKind::Crystal);
    }
}
