//! Userspace **AR9271** (`ath9k_htc`) driver — USB transport, firmware download, HTC handshake, WMI.
//!
//! The AR9271 matters to this project for a reason no other Wi-Fi part we own does: **its firmware
//! is ours**. `named-filter-mac-redesign.md` §8.2 records that on commodity Wi-Fi we get Tier 0's
//! CPU win but not NDN-NIC's wakeup win, because the NIC has already copied the frame to the host
//! before any filter of ours can run. On this part the filter runs in `ath_tgt_rx_tasklet()` on the
//! dongle, and §8.5's "biggest loss" — no hardware-scheduled TX — is answered by the MAC's
//! `AR_TSF_*` / `AR_QUIET1/2` / `AR_D_GBL_IFS_*` registers. See
//! `ndn-radio-drivers/firmware/ath9k-htc-ndr/README.md`.
//!
//! # Status — read this before believing anything below
//!
//! **L1 (this module): transport only.** Device open, firmware download, HTC handshake, and WMI
//! request/response. It is written against the *authoritative* sources — the endpoint map was read
//! off the device with `lsusb -v`, the HTC/WMI wire formats from the open firmware's own
//! `wlan/include/{htc,htc_services,wmi}.h`, and the download constants from the mainline
//! `hif_usb.h` — but **it has not yet been run against hardware**. Treat every "should" here as
//! untested.
//!
//! **This module does NOT replace `ath9k_htc`.** It cannot yet bring the radio up, because in the
//! HTC split the *host* owns the PHY: `ath9k_hw` (≈500 KB in-kernel) performs the AR9271 reset,
//! initvals replay, and calibration by issuing thousands of `WMI_REG_WRITE`s. Porting that is the
//! RTL8812AU saga again and is deliberately not attempted here. Until it is done, the path to
//! on-air measurement is: let the kernel driver load *our* firmware and do the PHY init, and use
//! this module only for what the kernel driver cannot express.
//!
//! **The thing it is actually for** is the config/telemetry channel: reading and writing
//! `ndr_cfg` / `ndr_stats` in target RAM so the Tier-0 filter can be reconfigured and its drop
//! counters read without a firmware rebuild. One caveat, discovered by reading the firmware rather
//! than assuming: **`WMI_ACCESS_MEMORY_CMDID` is dispatched but its handler body is empty**
//! (`magpie.c:88`, and the same handler is wired in `if_ath.c`'s table). The wire format exists;
//! the target-side implementation does not. It has to be added to our firmware before
//! [`Ath9kHtcBackend::read_target_memory`] can work.

use std::time::Duration;

use rusb::{Context, DeviceHandle, UsbContext};

use ndn_transport::FaceError;

/// Qualcomm Atheros USB vendor ID.
pub const ATHEROS_VID: u16 = 0x0cf3;

/// AR9271 product IDs. `0x9271` is the reference part (the one on o5p-1); the others are
/// vendor-rebadged AR9271 dongles carried by `ath9k_htc`'s id table.
pub const AR9271_IDS: &[(u16, u16)] = &[
    (0x0cf3, 0x9271), // Atheros reference
    (0x0cf3, 0x1006),
    (0x0846, 0x9030), // Netgear WNA1100
    (0x07d1, 0x3a10), // D-Link DWA-126
    (0x13d3, 0x3327),
    (0x040d, 0x3801),
    (0x0cf3, 0xb003),
];

// ── USB endpoints ────────────────────────────────────────────────────────────
//
// Read off the device itself (`lsusb -v -d 0cf3:9271`), not assumed. Note the asymmetry that a
// "pipe number" naming scheme hides: the WLAN data pipes are **bulk** at 512 bytes, while the
// register/command pipes are **interrupt** at 64 bytes. Using bulk transfers on the reg pipes
// silently fails, and 64 bytes is a hard ceiling on an HTC control message.

/// EP1 OUT, bulk 512 — WLAN TX data.
pub const EP_WLAN_TX: u8 = 0x01;
/// EP2 IN, bulk 512 — WLAN RX data.
pub const EP_WLAN_RX: u8 = 0x82;
/// EP3 IN, **interrupt** 64 — WMI events / HTC control from the target.
pub const EP_REG_IN: u8 = 0x83;
/// EP4 OUT, **interrupt** 64 — WMI commands / HTC control to the target.
pub const EP_REG_OUT: u8 = 0x04;

/// Target-side HIF pipe ids (`magpie_fw_dev/target/inc/hif_usb.h`). They coincide with the USB
/// endpoint numbers: `HIF_USB_PIPE_INTERRUPT` = 3 = EP3 IN, `HIF_USB_PIPE_COMMAND` = 4 = EP4 OUT.
/// The host names them `USB_REG_IN_PIPE` / `USB_REG_OUT_PIPE` for the same pipes.
pub const PIPE_REG_IN: u8 = 3;
pub const PIPE_REG_OUT: u8 = 4;

/// Hard ceiling on a register-pipe message (`wMaxPacketSize` on EP3/EP4).
pub const REG_PIPE_MAX: usize = 64;

// ── Firmware download ────────────────────────────────────────────────────────
// Constants from mainline `drivers/net/wireless/ath/ath9k/hif_usb.h`.

const FIRMWARE_DOWNLOAD: u8 = 0x30;
const FIRMWARE_DOWNLOAD_COMP: u8 = 0x31;

/// Where the AR9271 firmware image is written in target address space.
pub const AR9271_FIRMWARE: u32 = 0x0050_1000;
/// The entry point handed to the target to start executing the freshly downloaded image.
pub const AR9271_FIRMWARE_TEXT: u32 = 0x0090_3000;

/// Download chunk size. The vendor driver uses 4 KB and the target's control handler is written
/// around it.
const FW_CHUNK: usize = 4096;

/// The firmware file this kernel actually requests — **not** `htc_9271.fw`. Confirmed with
/// `modinfo ath9k_htc`; mainline builds the versioned name and only falls back to the legacy one.
pub const FW_NAME: &str = "ath9k_htc/htc_9271-1.4.0.fw";

const USB_TIMEOUT: Duration = Duration::from_millis(1000);

// ── HTC ──────────────────────────────────────────────────────────────────────
// Wire formats from the open firmware's `wlan/include/htc.h`. All multi-byte HTC and WMI fields
// are **big-endian** on the wire.

/// `HTC_FRAME_HDR`: endpoint id, flags, big-endian payload length, 4 control bytes.
pub const HTC_HDR_LEN: usize = 8;

/// The reserved control endpoint every handshake message rides on.
const HTC_ENDPOINT_CTRL: u8 = 0;

/// Set in an HTC header's flags when a receive trailer is appended; its length is ControlBytes[0].
const HTC_FLAGS_RECV_TRAILER: u8 = 1 << 1;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u16)]
enum HtcMsg {
    Ready = 1,
    ConnectService = 2,
    ConnectServiceResponse = 3,
    SetupComplete = 4,
}

/// HTC service IDs (`htc_services.h`): `(group << 8) | index`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u16)]
pub enum HtcService {
    /// The reserved control service.
    CtrlRsvd = 0x0001,
    /// WMI command/response — the one this module needs.
    WmiControl = 0x0100,
    Beacon = 0x0101,
    Cab = 0x0102,
    Uapsd = 0x0103,
    Mgmt = 0x0104,
    DataVo = 0x0105,
    DataVi = 0x0106,
    DataBe = 0x0107,
    DataBk = 0x0108,
}

// ── WMI ──────────────────────────────────────────────────────────────────────

/// WMI command IDs, from the firmware's `wlan/include/wmi.h`. The enum is dense from `0x0001`, so
/// these are positional — do not reorder.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u16)]
pub enum WmiCmd {
    Echo = 0x0001,
    /// Read/write arbitrary target memory. ⚠ **The open firmware's handler is an empty `break`** —
    /// see the module docs. Our firmware must implement it before this is usable.
    AccessMemory = 0x0002,
    GetFwVersion = 0x0003,
    RegRead = 0x0014,
    RegWrite = 0x0015,
    /// Tear the WLAN application down cleanly. Must be the last command — the target's handler
    /// frees its softc.
    TgtDetach = 0x001a,
}

/// `WMI_CMD_HDR`: command id + sequence number, both big-endian.
const WMI_HDR_LEN: usize = 4;

/// Target→host event IDs. Events share the response pipe with command replies and are
/// distinguished by having ids at or above `0x1001`.
const WMI_EVENT_BASE: u16 = 0x1001;

// ── WMI_ACCESS_MEMORY (firmware/ath9k-htc-ndr/src/ndr_mem.h) ─────────────────

/// Words per exchange. Bounded by the 64-byte register pipe, **not** by the vendor's
/// `WMI_ACCESS_MEMORY_MAX_TUPLES` of 8 — which does not fit: 8 + 4 + 4 + 8*8 = 80 > 64.
///
/// **5, not 6.** A reply can carry a 4-byte HTC receive trailer (measured: flags=0x02,
/// control[0]=4), and 8 + 4 + 4 + 6*8 + 4 = 68 > 64. Six tuples fit only as long as no trailer is
/// attached, which is not something the host gets to decide.
pub const NDR_MEM_MAX_TUPLES: usize = 5;

const NDR_MEM_FLAG_WRITE: u16 = 0x0001;
const NDR_MEM_OK: u16 = 0;

fn ndr_mem_status(code: u16) -> &'static str {
    match code {
        0 => "ok",
        1 => "malformed request",
        2 => "too many tuples",
        3 => "unaligned address",
        4 => "address outside the firmware's RAM windows",
        _ => "unknown status",
    }
}

/// Tier-0 filter counters (`struct ndr_stats`). `dropped_filter` is the headline number: USB
/// transfers and host wakeups that did not happen — the quantity design §8.2 says is unreachable
/// on every other Wi-Fi part we own.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct NdrStats {
    pub seen: u32,
    pub passed: u32,
    pub dropped_filter: u32,
    pub dropped_foreign: u32,
    pub short_frame: u32,
}

/// Largest echo payload whose *reply* fits one register-pipe packet:
/// `64 - HTC(8) - WMI(4) - msgSize(1) = 51`.
///
/// ⚠ The firmware's own `WMI_ECHOCMD_MSG_MAX_LEN` says **53**, and its derivation comment reads
/// `64 - HTC_HDR_LENGTH + sizeof(WMI_CMD_HDR) - 1` — it *adds* the WMI header where it should
/// subtract it. 53 does not fit: `8 + 4 + 1 + 53 = 66 > 64`. Trust the arithmetic, not the header.
pub const MAX_ECHO_LEN: usize = REG_PIPE_MAX - HTC_HDR_LEN - WMI_HDR_LEN - 1;

/// A userspace AR9271 over libusb.
pub struct Ath9kHtcBackend {
    handle: DeviceHandle<Context>,
    /// WMI sequence number; the target echoes it so replies can be matched to commands.
    seq: u16,
    /// Endpoint the target assigned to `WMI_CONTROL_SVC` during the handshake.
    wmi_endpoint: u8,
    /// Credits the target offered in its READY message — the HTC flow-control budget.
    credits: u16,
    credit_size: u16,
}

fn usb_err<E: std::fmt::Display>(what: &str, e: E) -> FaceError {
    FaceError::Io(std::io::Error::other(format!("ath9k_htc: {what}: {e}")))
}

/// Hex-dump every HTC exchange when `NDR_ATH9K_DEBUG` is set. Guessing at a handshake is how you
/// lose an afternoon; the bytes are always cheaper.
fn dbg_dump(tag: &str, buf: &[u8]) {
    if std::env::var_os("NDR_ATH9K_DEBUG").is_some() {
        eprintln!("[ath9k] {tag} ({} B): {buf:02x?}", buf.len());
    }
}

fn err(msg: String) -> FaceError {
    FaceError::Io(std::io::Error::other(msg))
}

impl Ath9kHtcBackend {
    /// Claim the first AR9271 on the bus.
    ///
    /// Detaches the kernel driver for the duration of our claim. Note that this alone does **not**
    /// stop `ath9k_htc` re-grabbing the device on a re-enumeration — that lesson cost days on the
    /// RTL8812AU, where the fix was `echo 0 > /sys/bus/usb/drivers_autoprobe`. The AR9271 is far
    /// more forgiving because its firmware lives in RAM: a fight with the kernel driver ends in a
    /// failed probe, not a bricked part.
    pub fn open() -> Result<Self, FaceError> {
        let ctx = Context::new().map_err(|e| usb_err("libusb init", e))?;

        let mut handle = Self::find_and_open(&ctx)?;

        // ⛔ DO NOT port-reset this device. It was tried and it is destructive: `handle.reset()`
        // on an AR9271 that is already running firmware makes it re-enumerate, and on the o5p-1
        // rig it then **dropped off the USB bus entirely** and needed a physical replug. That is
        // the same pattern as the RTL8812AU, where repeated resets destabilised the part until it
        // vanished. `authorized` 0/1 does not help either — it re-binds the interface without
        // cutting VBUS, so a running MCU stays running.
        //
        // The practical consequence, stated plainly: this driver expects the target to be in a
        // state where a firmware download will take. That holds after a replug, and it held in
        // practice right after `ath9k_htc` released the device. If `htc_init()` times out waiting
        // for READY, the target is most likely already running an image and needs a replug — there
        // is no software path back to cold.

        // The device can be left UNCONFIGURED (`bConfigurationValue` empty in sysfs), which is what
        // `ath9k_htc` leaves behind when its own probe fails — e.g. when its firmware download is
        // attempted over a target we already loaded. libusb cannot claim an interface on an
        // unconfigured device and reports a bare `Invalid parameter`, which reads like a bug in the
        // claim rather than a state problem in the device.
        //
        // Setting the configuration is the non-destructive fix. Only do it when actually needed:
        // libusb performs a lightweight reset when the configuration is set on a device that
        // already has one, and resets are exactly what must be avoided here.
        let needs_config = match handle.active_configuration() {
            Ok(0) | Err(_) => true,
            Ok(_) => false,
        };
        if needs_config {
            handle
                .set_active_configuration(1)
                .map_err(|e| usb_err("set configuration 1 (device was unconfigured)", e))?;
        }

        handle
            .claim_interface(0)
            .map_err(|e| usb_err("claim interface 0", e))?;

        Ok(Self {
            handle,
            seq: 0,
            wmi_endpoint: 0,
            credits: 0,
            credit_size: 0,
        })
    }

    /// Find the first AR9271 and open it, retrying while a reset-induced re-enumeration settles.
    fn find_and_open(ctx: &Context) -> Result<DeviceHandle<Context>, FaceError> {
        for attempt in 0..20 {
            let devices = ctx.devices().map_err(|e| usb_err("enumerate", e))?;
            for dev in devices.iter() {
                let desc = match dev.device_descriptor() {
                    Ok(d) => d,
                    Err(_) => continue,
                };
                if !AR9271_IDS.contains(&(desc.vendor_id(), desc.product_id())) {
                    continue;
                }
                match dev.open() {
                    Ok(mut h) => {
                        // ⛔ Deliberately NOT `set_auto_detach_kernel_driver(true)`. That flag
                        // makes libusb **re-attach** the kernel driver when the interface is
                        // released, so the moment our process exits `ath9k_htc` probes, fails its
                        // firmware download over the target we just loaded ("Firmware ... download
                        // failed" in dmesg), and leaves the device UNCONFIGURED — costing a
                        // physical replug before anything can touch it again.
                        //
                        // Detach by hand and never re-attach: the device then stays configured and
                        // unbound, and successive userspace runs work without a replug.
                        //
                        // Consequence worth knowing: handing the device *back* to `ath9k_htc`
                        // still requires a replug, because its download cannot succeed while our
                        // firmware is running. Userspace iteration is free; switching back is not.
                        match h.kernel_driver_active(0) {
                            Ok(true) => {
                                h.detach_kernel_driver(0)
                                    .map_err(|e| usb_err("detach kernel driver", e))?;
                            }
                            _ => {}
                        }
                        return Ok(h);
                    }
                    Err(e) if attempt == 19 => return Err(usb_err("open", e)),
                    Err(_) => break,
                }
            }
            std::thread::sleep(Duration::from_millis(250));
        }
        Err(err("ath9k_htc: no AR9271 found".to_string()))
    }

    /// Download a firmware image to target RAM and start it.
    ///
    /// There is no flash on this part — the image is written to RAM at [`AR9271_FIRMWARE`] and the
    /// completion request hands the target [`AR9271_FIRMWARE_TEXT`] as its entry point. That is
    /// what makes the AR9271 unbrickable and worth iterating on: a bad image costs a replug.
    ///
    /// `wValue` carries `addr >> 8`, which is why the target address must be 256-byte aligned.
    pub fn download_firmware(&mut self, fw: &[u8]) -> Result<(), FaceError> {
        let mut addr = AR9271_FIRMWARE;

        for chunk in fw.chunks(FW_CHUNK) {
            self.handle
                .write_control(
                    rusb::request_type(
                        rusb::Direction::Out,
                        rusb::RequestType::Vendor,
                        rusb::Recipient::Device,
                    ),
                    FIRMWARE_DOWNLOAD,
                    (addr >> 8) as u16,
                    0,
                    chunk,
                    USB_TIMEOUT,
                )
                .map_err(|e| usb_err(&format!("fw chunk at {addr:#x}"), e))?;
            addr += chunk.len() as u32;
        }

        self.handle
            .write_control(
                rusb::request_type(
                    rusb::Direction::Out,
                    rusb::RequestType::Vendor,
                    rusb::Recipient::Device,
                ),
                FIRMWARE_DOWNLOAD_COMP,
                (AR9271_FIRMWARE_TEXT >> 8) as u16,
                0,
                &[],
                USB_TIMEOUT,
            )
            .map_err(|e| usb_err("fw download complete", e))?;

        Ok(())
    }

    // ── HTC ──────────────────────────────────────────────────────────────────

    /// Send one HTC frame on the register-out (interrupt) pipe.
    fn htc_send(&mut self, endpoint: u8, payload: &[u8]) -> Result<(), FaceError> {
        let total = HTC_HDR_LEN + payload.len();
        if total > REG_PIPE_MAX {
            return Err(err(format!(
                "ath9k_htc: HTC message {total} B exceeds the {REG_PIPE_MAX} B register pipe"
            )));
        }

        let mut buf = [0u8; REG_PIPE_MAX];
        buf[0] = endpoint;
        buf[1] = 0; // flags
        buf[2..4].copy_from_slice(&(payload.len() as u16).to_be_bytes());
        // buf[4..8] = control bytes, zero
        buf[HTC_HDR_LEN..total].copy_from_slice(payload);

        dbg_dump("tx", &buf[..total]);
        self.handle
            .write_interrupt(EP_REG_OUT, &buf[..total], USB_TIMEOUT)
            .map_err(|e| usb_err("htc send", e))?;
        Ok(())
    }

    /// Receive one HTC frame from the register-in (interrupt) pipe, returning `(endpoint, payload)`.
    fn htc_recv(&mut self, timeout: Duration) -> Result<(u8, Vec<u8>), FaceError> {
        let mut buf = [0u8; REG_PIPE_MAX];
        let n = self
            .handle
            .read_interrupt(EP_REG_IN, &mut buf, timeout)
            .map_err(|e| usb_err("htc recv", e))?;

        dbg_dump("rx", &buf[..n]);
        if n < HTC_HDR_LEN {
            return Err(err(format!(
                "ath9k_htc: runt HTC frame, {n} B < {HTC_HDR_LEN} B header"
            )));
        }

        let endpoint = buf[0];
        let flags = buf[1];
        let mut len = u16::from_be_bytes([buf[2], buf[3]]) as usize;
        let avail = n - HTC_HDR_LEN;
        if len > avail {
            return Err(err(format!(
                "ath9k_htc: HTC header claims {len} B but only {avail} B arrived"
            )));
        }

        // PayloadLen INCLUDES the receive trailer, whose length is in ControlBytes[0]. Observed on
        // the wire: a 5-word ACCESS_MEMORY reply came back with flags=0x02, control[0]=0x04 and
        // PayloadLen=52 = 4 (WMI) + 4 (our header) + 40 (tuples) + 4 (trailer). Failing to strip it
        // leaves credit-report bytes glued to the end of every payload — harmless for a reply whose
        // length is known up front, and silently corrupting for anything parsed to its end.
        if flags & HTC_FLAGS_RECV_TRAILER != 0 {
            let trailer = buf[4] as usize;
            if trailer > len {
                return Err(err(format!(
                    "ath9k_htc: trailer {trailer} B exceeds payload {len} B"
                )));
            }
            len -= trailer;
        }

        Ok((endpoint, buf[HTC_HDR_LEN..HTC_HDR_LEN + len].to_vec()))
    }

    /// Complete the HTC handshake: wait for READY, connect `WMI_CONTROL_SVC`, then SETUP_COMPLETE.
    ///
    /// Must follow [`download_firmware`](Self::download_firmware) — the target sends READY
    /// unprompted once the image is running, which doubles as proof the download took.
    pub fn htc_init(&mut self) -> Result<(), FaceError> {
        // 1. READY, unsolicited from the target.
        let (_ep, msg) = self.htc_recv(Duration::from_millis(3000))?;
        if msg.len() < 8 || u16::from_be_bytes([msg[0], msg[1]]) != HtcMsg::Ready as u16 {
            return Err(err(format!(
                "ath9k_htc: expected HTC READY, got {msg:02x?}"
            )));
        }
        self.credits = u16::from_be_bytes([msg[2], msg[3]]);
        self.credit_size = u16::from_be_bytes([msg[4], msg[5]]);

        // 2. CONNECT_SERVICE for WMI control.
        //
        //    HTC_CONNECT_SERVICE_MSG is **10** bytes, not 8:
        //    MessageID(2) ServiceID(2) ConnectionFlags(2) DownLinkPipeID(1) UpLinkPipeID(1)
        //    ServiceMetaLength(1) _Pad1(1)
        //    A short message leaves the target reading ServiceMetaLength off the end of the buffer.
        //
        //    ★ The pipe IDs are load-bearing and must not be left zero. The target replies on
        //    `Endpoints[ep].DownLinkPipeID` (`htc.c:402`) — whatever the host puts here. Send 0 and
        //    every WMI response is dispatched to pipe 0 and never arrives, which presents as a
        //    read timeout on a handshake that otherwise looks completely successful.
        let mut req = [0u8; 10];
        req[0..2].copy_from_slice(&(HtcMsg::ConnectService as u16).to_be_bytes());
        req[2..4].copy_from_slice(&(HtcService::WmiControl as u16).to_be_bytes());
        req[6] = PIPE_REG_IN; // DownLinkPipeID: target -> host
        req[7] = PIPE_REG_OUT; // UpLinkPipeID:   host -> target
        self.htc_send(HTC_ENDPOINT_CTRL, &req)?;

        let (_ep, resp) = self.htc_recv(Duration::from_millis(1000))?;
        if resp.len() < 5
            || u16::from_be_bytes([resp[0], resp[1]]) != HtcMsg::ConnectServiceResponse as u16
        {
            return Err(err(format!(
                "ath9k_htc: expected CONNECT_SERVICE_RESPONSE, got {resp:02x?}"
            )));
        }
        // MessageID(2) ServiceID(2) Status(1) EndpointID(1) ...
        let status = resp[4];
        if status != 0 {
            return Err(err(format!(
                "ath9k_htc: WMI service connect refused, status {status}"
            )));
        }
        self.wmi_endpoint = *resp.get(5).ok_or_else(|| {
            err("ath9k_htc: CONNECT_SERVICE_RESPONSE has no endpoint id".to_string())
        })?;

        dbg_dump("connect-resp", &resp);
        if std::env::var_os("NDR_ATH9K_DEBUG").is_some() {
            eprintln!("[ath9k] WMI endpoint assigned: {}", self.wmi_endpoint);
        }

        // 3. SETUP_COMPLETE.
        let done = (HtcMsg::SetupComplete as u16).to_be_bytes();
        self.htc_send(HTC_ENDPOINT_CTRL, &done)?;

        Ok(())
    }

    // ── WMI ──────────────────────────────────────────────────────────────────

    /// Read whatever the target volunteers on the register-in pipe until it goes quiet.
    ///
    /// This is the discriminator when a WMI command gets no reply. After SETUP_COMPLETE the target
    /// is expected to raise `WMI_TGT_RDY_EVENTID` (0x1001) of its own accord. If that arrives, HTC
    /// and WMI are both alive and any silence afterwards is our command framing; if nothing
    /// arrives, the WLAN application on the target never came up and the fault is earlier.
    pub fn drain_events(&mut self, ms: u64) -> Vec<(u8, Vec<u8>)> {
        let deadline = std::time::Instant::now() + Duration::from_millis(ms);
        let mut out = Vec::new();
        while std::time::Instant::now() < deadline {
            match self.htc_recv(Duration::from_millis(250)) {
                Ok(f) => out.push(f),
                Err(_) => break,
            }
        }
        out
    }

    /// Issue a WMI command and wait for its response.    /// Issue a WMI command and wait for its response.
    ///
    /// Strictly one command in flight, matched on the echoed sequence number. That discipline was
    /// learned the expensive way on the LoRa dongle: fire-and-forget hid a ~50% command loss for
    /// months because nothing ever checked for a reply.
    ///
    /// Unsolicited events (id ≥ `0x1001`) share this pipe and are skipped while waiting.
    pub fn wmi_cmd(&mut self, cmd: WmiCmd, payload: &[u8]) -> Result<Vec<u8>, FaceError> {
        self.seq = self.seq.wrapping_add(1);
        let seq = self.seq;

        let mut buf = Vec::with_capacity(WMI_HDR_LEN + payload.len());
        buf.extend_from_slice(&(cmd as u16).to_be_bytes());
        buf.extend_from_slice(&seq.to_be_bytes());
        buf.extend_from_slice(payload);

        let ep = self.wmi_endpoint;
        self.htc_send(ep, &buf)?;

        // Skip events until the matching reply arrives.
        for _ in 0..16 {
            let (_ep, resp) = self.htc_recv(USB_TIMEOUT)?;
            if resp.len() < WMI_HDR_LEN {
                continue;
            }
            let id = u16::from_be_bytes([resp[0], resp[1]]);
            let rseq = u16::from_be_bytes([resp[2], resp[3]]);

            if id >= WMI_EVENT_BASE {
                continue; // asynchronous event, not our reply
            }
            if rseq == seq {
                return Ok(resp[WMI_HDR_LEN..].to_vec());
            }
        }

        Err(err(format!(
            "ath9k_htc: no WMI reply for {cmd:?} seq {seq}"
        )))
    }

    /// Ask the target its firmware version — the correct liveness probe.
    ///
    /// Prefer this over [`echo`](Self::echo). `ath_get_tgt_version` replies via
    /// `sc->tgt_wmi_handle` like every other well-formed handler, whereas upstream's echo handler
    /// passed the dispatch table's `pContext` (the softc) where a WMI handle was expected — see the
    /// note on `handle_echo_command` in our `if_ath.c`. On stock firmware echo answers nothing and
    /// leaves the target unresponsive.
    pub fn fw_version(&mut self) -> Result<(u16, u16), FaceError> {
        let r = self.wmi_cmd(WmiCmd::GetFwVersion, &[])?;
        if r.len() < 4 {
            return Err(err(format!(
                "ath9k_htc: GET_FW_VERSION returned {} B, expected 4",
                r.len()
            )));
        }
        Ok((
            u16::from_be_bytes([r[0], r[1]]),
            u16::from_be_bytes([r[2], r[3]]),
        ))
    }

    /// Round-trip the target with `WMI_ECHO`    /// Round-trip the target with `WMI_ECHO` — the cheapest proof the whole stack is alive:
    /// firmware running, HTC connected, WMI dispatching, both pipes working.
    ///
    /// The firmware's echo handler caps the payload at `WMI_ECHOCMD_MSG_MAX_LEN` (53).
    pub fn echo(&mut self, data: &[u8]) -> Result<Vec<u8>, FaceError> {
        if data.len() > MAX_ECHO_LEN {
            return Err(err(format!(
                "ath9k_htc: echo payload {} B exceeds {MAX_ECHO_LEN} B (see MAX_ECHO_LEN)",
                data.len()
            )));
        }
        let mut payload = Vec::with_capacity(1 + data.len());
        payload.push(data.len() as u8); // WMI_ECHO_CMD.msgSize
        payload.extend_from_slice(data);
        self.wmi_cmd(WmiCmd::Echo, &payload)
    }

    /// HTC credits the target advertised, and their size. Populated by
    /// [`htc_init`](Self::htc_init); useful mainly as evidence the READY message parsed sanely.
    pub fn credits(&self) -> (u16, u16) {
        (self.credits, self.credit_size)
    }

    // ── Target memory (WMI_ACCESS_MEMORY) ────────────────────────────────────

    /// Read `n` 32-bit words from target memory.
    ///
    /// Deliberately word-oriented rather than byte-oriented. The target loads each word natively
    /// and serialises it big-endian; the host deserialises big-endian. The *value* therefore
    /// round-trips exactly regardless of the target's own byte order, whereas a byte-image API
    /// would silently depend on it.
    ///
    /// Addresses must be 4-byte aligned — Xtensa has no unaligned 32-bit load, so the firmware
    /// rejects unaligned requests rather than faulting.
    pub fn read_target_u32s(&mut self, addr: u32, n: usize) -> Result<Vec<u32>, FaceError> {
        self.access_memory(addr, &vec![0u32; n], false)
    }

    /// Write 32-bit words to target memory. Returns what the target echoed back.
    pub fn write_target_u32s(&mut self, addr: u32, vals: &[u32]) -> Result<Vec<u32>, FaceError> {
        self.access_memory(addr, vals, true)
    }

    /// One `WMI_ACCESS_MEMORY` exchange per chunk of at most [`NDR_MEM_MAX_TUPLES`] words.
    ///
    /// Wire format (defined by us — see `firmware/ath9k-htc-ndr/src/ndr_mem.h`; upstream declared
    /// the struct but its handler was `adf_os_assert(0)`):
    ///
    /// ```text
    ///   request:   u16 flags | u16 count | count * { u32 addr, u32 value }
    ///   response:  u16 status | u16 count | count * { u32 addr, u32 value }
    /// ```
    fn access_memory(&mut self, addr: u32, vals: &[u32], write: bool) -> Result<Vec<u32>, FaceError> {
        if addr % 4 != 0 {
            return Err(err(format!(
                "ath9k_htc: target address {addr:#010x} is not 4-byte aligned"
            )));
        }

        let mut out = Vec::with_capacity(vals.len());

        for (chunk_idx, chunk) in vals.chunks(NDR_MEM_MAX_TUPLES).enumerate() {
            let base = addr + (chunk_idx * NDR_MEM_MAX_TUPLES * 4) as u32;

            let mut payload = Vec::with_capacity(4 + chunk.len() * 8);
            payload.extend_from_slice(&(if write { NDR_MEM_FLAG_WRITE } else { 0u16 }).to_be_bytes());
            payload.extend_from_slice(&(chunk.len() as u16).to_be_bytes());
            for (i, v) in chunk.iter().enumerate() {
                payload.extend_from_slice(&(base + (i * 4) as u32).to_be_bytes());
                payload.extend_from_slice(&v.to_be_bytes());
            }

            let resp = self.wmi_cmd(WmiCmd::AccessMemory, &payload)?;
            if resp.len() < 4 {
                return Err(err(format!(
                    "ath9k_htc: ACCESS_MEMORY reply is {} B, need at least 4 — is the firmware \
                     the patched build? Stock asserts on this command.",
                    resp.len()
                )));
            }

            let status = u16::from_be_bytes([resp[0], resp[1]]);
            let count = u16::from_be_bytes([resp[2], resp[3]]) as usize;
            if status != NDR_MEM_OK {
                return Err(err(format!(
                    "ath9k_htc: ACCESS_MEMORY at {base:#010x} failed: {}",
                    ndr_mem_status(status)
                )));
            }
            if count != chunk.len() || resp.len() < 4 + count * 8 {
                return Err(err(format!(
                    "ath9k_htc: ACCESS_MEMORY returned {count} of {} words ({} B)",
                    chunk.len(),
                    resp.len()
                )));
            }

            for i in 0..count {
                let v = 4 + i * 8 + 4;
                out.push(u32::from_be_bytes([resp[v], resp[v + 1], resp[v + 2], resp[v + 3]]));
            }
        }

        Ok(out)
    }

    /// Tell the target to tear its WLAN application down, the way `ath9k_htc` does on stop.
    ///
    /// ★ Send this before dropping the device, or the next bring-up needs a **physical replug**.
    /// The target keeps running our firmware after we release the interface; `ath9k_htc` then
    /// auto-probes, fails its own firmware download over the live target ("Firmware ... download
    /// failed"), and leaves the device UNCONFIGURED — at which point neither the kernel nor we can
    /// claim it, and no software recovery works (re-authorize, set-configuration and writing
    /// `bConfigurationValue` were all tried and all fail).
    ///
    /// The target's handler frees its own softc, so no further command may be issued afterwards.
    /// Errors are worth logging but not worth failing on — we are on the way out either way.
    pub fn detach(&mut self) -> Result<(), FaceError> {
        self.wmi_cmd(WmiCmd::TgtDetach, &[])?;
        Ok(())
    }

    /// Read the Tier-0 filter's counters.    /// Read the Tier-0 filter's counters.
    ///
    /// `addr` is `ndr_stats`, which **moves between firmware builds** — take it from the image you
    /// actually loaded rather than hardcoding it:
    ///
    /// ```sh
    /// xtensa-elf-nm build/k2/fw.elf | grep ndr_stats
    /// ```
    pub fn read_ndr_stats(&mut self, addr: u32) -> Result<NdrStats, FaceError> {
        let w = self.read_target_u32s(addr, 5)?;
        Ok(NdrStats {
            seen: w[0],
            passed: w[1],
            dropped_filter: w[2],
            dropped_foreign: w[3],
            short_frame: w[4],
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The endpoint map is the thing most likely to be silently wrong, and it is read off the
    /// device rather than assumed — pin it so a future edit cannot quietly "tidy" it.
    #[test]
    fn endpoint_directions_match_the_device_descriptor() {
        assert_eq!(EP_WLAN_TX & 0x80, 0, "EP1 is OUT");
        assert_eq!(EP_WLAN_RX & 0x80, 0x80, "EP2 is IN");
        assert_eq!(EP_REG_IN & 0x80, 0x80, "EP3 is IN");
        assert_eq!(EP_REG_OUT & 0x80, 0, "EP4 is OUT");
    }

    #[test]
    fn service_ids_match_make_service_id() {
        // MAKE_SERVICE_ID(group, index) == (group << 8) | index
        assert_eq!(HtcService::CtrlRsvd as u16, (0 << 8) | 1);
        assert_eq!(HtcService::WmiControl as u16, (1 << 8) | 0);
        assert_eq!(HtcService::DataBe as u16, (1 << 8) | 7);
    }

    /// The WMI enum in `wmi.h` is dense from 0x0001, so these are positional. If someone inserts a
    /// command in the firmware header without updating this, register access silently targets the
    /// wrong handler.
    #[test]
    fn wmi_command_ids_are_positional() {
        assert_eq!(WmiCmd::Echo as u16, 0x0001);
        assert_eq!(WmiCmd::AccessMemory as u16, 0x0002);
        assert_eq!(WmiCmd::GetFwVersion as u16, 0x0003);
        assert_eq!(WmiCmd::RegRead as u16, 0x0014);
        assert_eq!(WmiCmd::RegWrite as u16, 0x0015);
    }

    /// The vendor's `WMI_ACCESS_MEMORY_MAX_TUPLES` is 8 and does not fit the 64-byte register
    /// pipe. Pin the derived bound so it cannot drift back.
    #[test]
    fn access_memory_chunk_fits_the_register_pipe() {
        // Budget the 4-byte receive trailer: a reply may carry one and the host cannot refuse it.
        let hdr = HTC_HDR_LEN + WMI_HDR_LEN + 4 + 4;
        assert!(hdr + NDR_MEM_MAX_TUPLES * 8 <= REG_PIPE_MAX);
        assert!(hdr + 6 * 8 > REG_PIPE_MAX, "6 tuples overflow once a trailer is attached");
        assert!(hdr + 8 * 8 > REG_PIPE_MAX, "the vendor's 8 tuples overflow the pipe");
    }

    /// A maximal echo must fit one register-pipe packet — and the firmware's own
    /// `WMI_ECHOCMD_MSG_MAX_LEN` (53) does **not**: 8 + 4 + 1 + 53 = 66 > 64. This pins the derived
    /// limit so nobody "corrects" it back to the header's value.
    #[test]
    fn max_echo_fits_the_register_pipe() {
        assert_eq!(MAX_ECHO_LEN, 51);
        assert!(HTC_HDR_LEN + WMI_HDR_LEN + 1 + MAX_ECHO_LEN <= REG_PIPE_MAX);
        assert!(HTC_HDR_LEN + WMI_HDR_LEN + 1 + 53 > REG_PIPE_MAX, "the firmware constant overflows");
    }
}
