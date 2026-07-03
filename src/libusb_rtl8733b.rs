//! Userspace **RTL8731BU / RTL8733BU** monitor-mode driver — a ground-up port.
//!
//! The 8733b is Realtek's **`halmac_87xx`** generation — distinct from the
//! `halmac_88xx` RTL8812EU/8822E in [`libusb_rtl88xx`](crate). Its power-on
//! sequence, firmware, TX descriptor, and PHY/RF tables are its own, so this is a
//! fresh port rather than an adaptation. What *is* shared is the Realtek USB HCI
//! register-I/O protocol (the `VENQT` vendor request), reused here verbatim.
//!
//! It is a **1×1** 11ac part (one spatial stream, one RF path A) — so single-
//! stream rate codes (HT MCS0–7 / VHT MCS0–8), no 2SS descriptor/TX-path setup,
//! and a simpler IQK/calibration than the 2×2 chips.
//!
//! ## Milestones
//! - **M1 (this file today): open + reg-I/O + chip-version identification.**
//! - M2+: power-on, firmware download, MAC/BB/RF init, 1×1 calibration,
//!   channel/TX-power, RX-DMA, TX inject, RX capture — implementing
//!   [`FrameIo`](ndn_frame_io::FrameIo) + [`WifiRadio`](ndn_frame_io::WifiRadio).

use std::sync::Arc;
use std::time::{Duration, Instant};

use ndn_transport::FaceError;
use rusb::{Context, Device, DeviceHandle, Direction, TransferType, UsbContext};

/// Realtek's USB vendor ID.
pub const REALTEK_VID: u16 = 0x0bda;

/// The 8731bu/8733bu USB product IDs: `0xf72b` (WiFi-only single-function) and
/// `0xb733` (USB multi-function). Both decode to the RTL8733B in the vendor
/// driver's id table.
pub const RTL8733B_PIDS: &[u16] = &[0xf72b, 0xb733];

// ── Realtek USB HCI register I/O (identical across their USB chips) ───────────
// A control transfer with bRequest 0x05; bmRequestType 0xC0 reads (device→host,
// vendor, device recipient), 0x40 writes. wValue = register address, wIndex = 0.
const VENQT_READ: u8 = 0xC0;
const VENQT_WRITE: u8 = 0x40;
const VENQT_REQ: u8 = 0x05;
const CTRL_TIMEOUT: Duration = Duration::from_millis(500);

// ── Chip-version registers (halmac_87xx) ─────────────────────────────────────
// From the vendor driver's chip-id decode: `chip_id = R8(REG_SYS_CFG2)` and
// `chip_ver = R8(REG_SYS_CFG1 + 1) >> 4`.
const REG_SYS_CFG1: u16 = 0x00F0;
const REG_SYS_CFG2: u16 = 0x00FC;

// ── M2: power-on (card enable) sequence ──────────────────────────────────────
// Transcribed from the vendor `card_en_flow_8733b` = CARDDIS_TO_CARDEMU +
// CARDEMU_TO_ACT, **pre-filtered to the USB interface** (SDIO-only steps
// dropped). Each step is a read-modify-write `(cur & ~msk) | (val & msk)` or a
// poll-until `(read8 & msk) == (val & msk)`, exactly as the halmac pwr_seq
// parser executes them. `0x0002` is SYS_FUNC_EN (BB/RF reset toggles); the
// `0x001F`/`0x0077` toggles to `0x87` bring the RF path up.
#[derive(Clone, Copy)]
enum PwrCmd {
    Write,
    Poll,
}

struct PwrStep {
    cmd: PwrCmd,
    offset: u16,
    msk: u8,
    val: u8,
}

const W: PwrCmd = PwrCmd::Write;
const P: PwrCmd = PwrCmd::Poll;

const POWER_ON_8733B: &[PwrStep] = &[
    // TRANS_CARDDIS_TO_CARDEMU (USB steps only)
    step(W, 0x0005, 0x08, 0x00), // clear APS_FSMCO BIT3
    step(W, 0x004A, 0x01, 0x00), // USB: clear BIT0
    // TRANS_CARDEMU_TO_ACT
    step(P, 0x0006, 0x02, 0x02), // wait power-ready
    step(W, 0x0006, 0x01, 0x01),
    step(W, 0x0005, 0x01, 0x01), // trigger power-on
    step(P, 0x0005, 0x01, 0x00), // wait power-on done
    step(W, 0x1002, 0x01, 0x01),
    // SYS_FUNC_EN 0x0002 — BB/RF reset toggles (ends enabled)
    step(W, 0x0002, 0x03, 0x00),
    step(W, 0x0002, 0x03, 0x03),
    step(W, 0x0002, 0x03, 0x00),
    step(W, 0x0002, 0x03, 0x03),
    step(W, 0x0002, 0x03, 0x00),
    step(W, 0x0002, 0x03, 0x03),
    // RF control 0x001F / 0x0077 — toggle to 0x87 (RF path up)
    step(W, 0x001F, 0xFF, 0x00),
    step(W, 0x0077, 0xFF, 0x00),
    step(W, 0x001F, 0xFF, 0x87),
    step(W, 0x0077, 0xFF, 0x87),
    step(W, 0x001F, 0xFF, 0x00),
    step(W, 0x0077, 0xFF, 0x00),
    step(W, 0x001F, 0xFF, 0x87),
    step(W, 0x0077, 0xFF, 0x87),
    step(W, 0x001F, 0xFF, 0x00),
    step(W, 0x0077, 0xFF, 0x00),
    step(W, 0x001F, 0xFF, 0x87),
    step(W, 0x0077, 0xFF, 0x87),
];

const fn step(cmd: PwrCmd, offset: u16, msk: u8, val: u8) -> PwrStep {
    PwrStep {
        cmd,
        offset,
        msk,
        val,
    }
}

/// The MAC control register — non-zero once the MAC is active.
const REG_CR: u16 = 0x0100;
/// SYS_FUNC_EN — after power-on its low two bits (BB/RF enable) read back set.
const REG_SYS_FUNC_EN: u16 = 0x0002;
/// Secure-boot control (halmac_fw_87xx): `(R8 & 0x03) == 0` selects the secure
/// firmware-download path; otherwise the non-secure path.
const REG_SECURE_CTRL: u16 = 0x0014;

// ── M3: firmware image + header ──────────────────────────────────────────────
/// The NIC firmware for the 8733b (halmac non-secure image, signature 0x8723),
/// embedded from the vendor `array_mp_8733b_fw_nic`. Downloaded (M3+) via the
/// bulk-OUT reserved-page path + DDMA into the wlan CPU's DMEM/IMEM.
pub const FW_NIC_8733B: &[u8] = include_bytes!("../fw/rtl8733b_fw_nic.bin");

// FW header field offsets (halmac_fw_info.h).
const FW_HDR_SIZE: u32 = 64;
const FW_HDR_CHKSUM_SIZE: u32 = 8;
const OFF_MEM_USAGE: usize = 24;
const OFF_DMEM_ADDR: usize = 32;
const OFF_DMEM_SIZE: usize = 36;
const OFF_IMEM_SIZE: usize = 48;
const OFF_EMEM_SIZE: usize = 52;
const OFF_EMEM_ADDR: usize = 56;
const OFF_IMEM_ADDR: usize = 60;

/// The parsed 8733b firmware header — the section addresses/sizes that drive the
/// DMA download.
#[derive(Debug, Clone, Copy)]
pub struct FwHeader {
    pub signature: u16,
    pub version: u16,
    pub subversion: u8,
    pub dmem_addr: u32,
    pub dmem_size: u32,
    pub imem_addr: u32,
    pub imem_size: u32,
    pub emem_addr: u32,
    pub emem_size: u32,
    /// Whether an EMEM section is present (`MEM_USAGE & BIT(4)`).
    pub has_emem: bool,
}

impl FwHeader {
    /// Parse the 64-byte halmac firmware header (little-endian fields), masking
    /// the top address bit the hardware ignores.
    pub fn parse(fw: &[u8]) -> Result<Self, FaceError> {
        if fw.len() < FW_HDR_SIZE as usize {
            return Err(io_err("firmware shorter than its 64-byte header".into()));
        }
        let u16a = |o: usize| u16::from_le_bytes([fw[o], fw[o + 1]]);
        let u32a = |o: usize| u32::from_le_bytes([fw[o], fw[o + 1], fw[o + 2], fw[o + 3]]);
        Ok(FwHeader {
            signature: u16a(0),
            version: u16a(4),
            subversion: fw[6],
            dmem_addr: u32a(OFF_DMEM_ADDR) & !(1 << 31),
            dmem_size: u32a(OFF_DMEM_SIZE),
            imem_addr: u32a(OFF_IMEM_ADDR) & !(1 << 31),
            imem_size: u32a(OFF_IMEM_SIZE),
            emem_addr: u32a(OFF_EMEM_ADDR) & !(1 << 31),
            emem_size: u32a(OFF_EMEM_SIZE),
            has_emem: fw[OFF_MEM_USAGE] & (1 << 4) != 0,
        })
    }

    /// Expected file length for the **non-secure** layout: header + each present
    /// section plus its 8-byte checksum. Matching the actual length confirms the
    /// header decode (and that this is the non-secure image).
    pub fn nonsecure_len(&self) -> u32 {
        let emem = if self.has_emem {
            self.emem_size + FW_HDR_CHKSUM_SIZE
        } else {
            0
        };
        FW_HDR_SIZE
            + (self.dmem_size + FW_HDR_CHKSUM_SIZE)
            + (self.imem_size + FW_HDR_CHKSUM_SIZE)
            + emem
    }
}

/// A minimal M1 handle to an 8731bu/8733bu: an open, claimed USB device with
/// register I/O. Grows into a full [`FrameIo`](ndn_frame_io::FrameIo) backend as
/// the bring-up milestones land.
pub struct Rtl8733buBackend {
    handle: Arc<DeviceHandle<Context>>,
    bulk_out: u8,
    bulk_in: u8,
}

/// What [`Rtl8733buBackend::chip_version`] reads back.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ChipVersion {
    /// `REG_SYS_CFG2` (0xFC) — the hardware chip-id byte.
    pub chip_id: u8,
    /// High nibble of `REG_SYS_CFG1 + 1` (0xF1) — the cut/version.
    pub chip_ver: u8,
    /// The full `REG_SYS_CFG1` dword (0xF0) — vendor / interface / trailing bits,
    /// surfaced raw so the identification can be refined against silicon.
    pub sys_cfg1: u32,
}

impl ChipVersion {
    /// Whether the reg reads look like a live 8733b (non-0x00/0xFF chip-id, and a
    /// plausible sys-cfg dword). A dead/unclaimed bus reads all-ones.
    pub fn looks_alive(&self) -> bool {
        self.sys_cfg1 != 0xFFFF_FFFF && self.sys_cfg1 != 0 && self.chip_id != 0xFF
    }
}

impl Rtl8733buBackend {
    /// Find and claim the first 8731bu/8733bu on the bus.
    pub fn open() -> Result<Self, FaceError> {
        let context = Context::new().map_err(usb_err)?;
        for device in context.devices().map_err(usb_err)?.iter() {
            let desc = device.device_descriptor().map_err(usb_err)?;
            if desc.vendor_id() == REALTEK_VID && RTL8733B_PIDS.contains(&desc.product_id()) {
                return Self::claim(device);
            }
        }
        Err(not_found(
            "no RTL8731BU/8733BU dongle found (Realtek 0bda:f72b / 0bda:b733)",
        ))
    }

    fn claim(device: Device<Context>) -> Result<Self, FaceError> {
        let handle = Arc::new(device.open().map_err(usb_err)?);
        // Detach any kernel driver (Linux); a no-op where none is bound (macOS).
        let _ = handle.set_auto_detach_kernel_driver(true);
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
                        Direction::In if bulk_in.is_none() => bulk_in = Some(ep.address()),
                        Direction::Out if bulk_out.is_none() => bulk_out = Some(ep.address()),
                        _ => {}
                    }
                }
            }
        }
        Ok(Self {
            handle,
            bulk_out: bulk_out.ok_or_else(|| not_found("RTL8733B exposes no bulk OUT endpoint"))?,
            bulk_in: bulk_in.ok_or_else(|| not_found("RTL8733B exposes no bulk IN endpoint"))?,
        })
    }

    /// Read one register byte over the `VENQT` control pipe.
    pub fn read8(&self, addr: u16) -> Result<u8, FaceError> {
        let mut buf = [0u8; 1];
        self.handle
            .read_control(VENQT_READ, VENQT_REQ, addr, 0, &mut buf, CTRL_TIMEOUT)
            .map_err(usb_err)?;
        Ok(buf[0])
    }

    /// Read a 32-bit register (little-endian on the wire).
    pub fn read32(&self, addr: u16) -> Result<u32, FaceError> {
        let mut buf = [0u8; 4];
        self.handle
            .read_control(VENQT_READ, VENQT_REQ, addr, 0, &mut buf, CTRL_TIMEOUT)
            .map_err(usb_err)?;
        Ok(u32::from_le_bytes(buf))
    }

    /// Write one register byte over the `VENQT` control pipe.
    pub fn write8(&self, addr: u16, val: u8) -> Result<(), FaceError> {
        self.handle
            .write_control(VENQT_WRITE, VENQT_REQ, addr, 0, &[val], CTRL_TIMEOUT)
            .map_err(usb_err)?;
        Ok(())
    }

    /// **M1**: read the hardware chip-id + cut from the SYS_CFG registers.
    pub fn chip_version(&self) -> Result<ChipVersion, FaceError> {
        let chip_id = self.read8(REG_SYS_CFG2)?;
        let chip_ver = self.read8(REG_SYS_CFG1 + 1)? >> 4;
        let sys_cfg1 = self.read32(REG_SYS_CFG1)?;
        Ok(ChipVersion {
            chip_id,
            chip_ver,
            sys_cfg1,
        })
    }

    /// **M2**: run the card-enable power-on sequence, bringing the MAC from
    /// card-disable/emulation to the active state (BB/RF out of reset, RF path
    /// up). Errors if any poll step times out (~200 ms budget each).
    pub fn power_on(&self) -> Result<(), FaceError> {
        for s in POWER_ON_8733B {
            match s.cmd {
                PwrCmd::Write => {
                    let cur = self.read8(s.offset)?;
                    self.write8(s.offset, (cur & !s.msk) | (s.val & s.msk))?;
                }
                PwrCmd::Poll => {
                    let deadline = Instant::now() + Duration::from_millis(200);
                    loop {
                        let got = self.read8(s.offset)? & s.msk;
                        if got == (s.val & s.msk) {
                            break;
                        }
                        if Instant::now() >= deadline {
                            return Err(io_err(format!(
                                "8733b power-on: poll timeout at 0x{:04x} (msk 0x{:02x}, want 0x{:02x}, got 0x{:02x})",
                                s.offset,
                                s.msk,
                                s.val & s.msk,
                                got
                            )));
                        }
                    }
                }
            }
        }
        Ok(())
    }

    /// Whether the MAC reads as powered-on. A card-disabled MAC returns the
    /// `0xEA` "not-ready" sentinel on `REG_CR` (and `0xFF` on a dead bus);
    /// power-on takes it to `0x00` (enabled, awaiting MAC init) and sets
    /// SYS_FUNC_EN's BB/RF-enable bits. So: BB/RF bits set **and** CR left the
    /// disabled sentinel. (The CR functional bits are set later, in MAC init.)
    pub fn is_powered(&self) -> Result<bool, FaceError> {
        let func_en = self.read8(REG_SYS_FUNC_EN)?;
        let cr = self.read8(REG_CR)?;
        Ok((func_en & 0x03) == 0x03 && cr != 0xEA && cr != 0xFF)
    }

    /// Read the post-power-on control registers (for M2 reporting / M3+).
    pub fn read_cr(&self) -> Result<u8, FaceError> {
        self.read8(REG_CR)
    }
    pub fn read_sys_func_en(&self) -> Result<u8, FaceError> {
        self.read8(REG_SYS_FUNC_EN)
    }

    /// **M3**: whether the chip selects the secure firmware-download path
    /// (`(REG_SECURE_CTRL & 0x03) == 0`). Our NIC image is the non-secure layout,
    /// so this is expected to read `false`.
    pub fn fw_secure(&self) -> Result<bool, FaceError> {
        Ok((self.read8(REG_SECURE_CTRL)? & 0x03) == 0x00)
    }

    /// The embedded NIC firmware and its parsed header (M3 groundwork).
    pub fn firmware() -> &'static [u8] {
        FW_NIC_8733B
    }
    pub fn fw_header() -> Result<FwHeader, FaceError> {
        FwHeader::parse(FW_NIC_8733B)
    }

    /// The discovered bulk endpoints (for the later TX/RX milestones).
    pub fn bulk_out(&self) -> u8 {
        self.bulk_out
    }
    pub fn bulk_in(&self) -> u8 {
        self.bulk_in
    }
}

fn usb_err(e: rusb::Error) -> FaceError {
    FaceError::Io(std::io::Error::other(format!("rtl8733b usb: {e}")))
}

fn not_found(msg: &str) -> FaceError {
    FaceError::Io(std::io::Error::new(
        std::io::ErrorKind::NotFound,
        msg.to_string(),
    ))
}

fn io_err(msg: String) -> FaceError {
    FaceError::Io(std::io::Error::other(msg))
}
