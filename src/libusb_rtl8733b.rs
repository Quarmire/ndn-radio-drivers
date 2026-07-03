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

// ── M4: TX descriptor + reserved-page (beacon) write ─────────────────────────
// The 87xx pushes firmware (and any reserved page) as a bulk-OUT packet: a
// 40-byte TX descriptor + payload, landed in the packet buffer's beacon page,
// with completion signalled by BCN_VALID. Register map (halmac_reg_8733b):
const TX_DESC_SIZE: usize = 40;
const REG_TXDMA_PQ_MAP: u16 = 0x010C; // +1 = queue→DMA priority map
const REG_RQPN_CTRL_HLPQ: u16 = 0x0200; // hi/low/pub queue page counts
const REG_DWBCN0_CTRL: u16 = 0x0208; // +1 = bcn head page, +2 bit0 = BCN_VALID (W1C)
const REG_FWHW_TXQ_CTRL: u16 = 0x0420; // +2 bit6 = bcn-queue enable
const REG_BCN_CTRL: u16 = 0x0550;
const REG_EXT_SYS_FUNC_EN: u16 = 0x1000;
const REG_EXT_SYS_CLK_CTRL: u16 = 0x1008;
const QSLT_MGNT: u32 = 0x12; // management queue-select for the download descriptor
const DMA_MAPPING_HIGH: u8 = 3; // HALMAC_DMA_MAPPING_HIGH — HIQ priority
const BIT_HCI_TXDMA_EN: u8 = 0x01; // REG_CR BIT(0)
const BIT_TXDMA_EN: u8 = 0x04; // REG_CR BIT(2)

/// Build the 40-byte TX descriptor for a reserved-page download packet: payload
/// length in TXPKTSIZE, header length in OFFSET, management queue-select. (Per
/// the vendor `fill_fake_txdesc`; rate fields are irrelevant — the packet is
/// stored in the beacon page, not transmitted.)
fn download_txdesc(payload_len: usize) -> [u8; TX_DESC_SIZE] {
    let mut d = [0u8; TX_DESC_SIZE];
    // dword0: [0:16] TXPKTSIZE, [16:24] OFFSET
    let dw0 = (payload_len as u32 & 0xFFFF) | ((TX_DESC_SIZE as u32 & 0xFF) << 16);
    d[0..4].copy_from_slice(&dw0.to_le_bytes());
    // dword1: [8:13] QSEL
    let dw1 = (QSLT_MGNT & 0x1F) << 8;
    d[4..8].copy_from_slice(&dw1.to_le_bytes());
    d
}

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
    /// All bulk-OUT endpoints, in descriptor order — Realtek USB chips expose one
    /// per TX queue priority; the reserved-page/HIQ path may need a specific one.
    bulk_outs: Vec<u8>,
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
        let mut bulk_in = None;
        let mut bulk_outs = Vec::new();
        for iface in config.interfaces() {
            for desc in iface.descriptors() {
                for ep in desc.endpoint_descriptors() {
                    if ep.transfer_type() != TransferType::Bulk {
                        continue;
                    }
                    match ep.direction() {
                        Direction::In if bulk_in.is_none() => bulk_in = Some(ep.address()),
                        Direction::Out => {
                            if !bulk_outs.contains(&ep.address()) {
                                bulk_outs.push(ep.address());
                            }
                        }
                        _ => {}
                    }
                }
            }
        }
        Ok(Self {
            handle,
            bulk_out: *bulk_outs
                .first()
                .ok_or_else(|| not_found("RTL8733B exposes no bulk OUT endpoint"))?,
            bulk_outs,
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

    /// Write a 32-bit register (little-endian on the wire).
    pub fn write32(&self, addr: u16, val: u32) -> Result<(), FaceError> {
        self.handle
            .write_control(VENQT_WRITE, VENQT_REQ, addr, 0, &val.to_le_bytes(), CTRL_TIMEOUT)
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

    /// **M4**: the minimal TX-DMA setup the firmware-download / reserved-page path
    /// needs before any `dl_rsvd_page` — enable platform clock + IRAM power, map
    /// HIQ to high priority, enable TXDMA, set the download queue page counts, and
    /// disable beacon functions. (The prologue of the vendor `download_firmware`.)
    pub fn fw_dl_setup(&self) -> Result<(), FaceError> {
        self.write8(REG_EXT_SYS_CLK_CTRL, self.read8(REG_EXT_SYS_CLK_CTRL)? | 0x02)?;
        self.write32(
            REG_EXT_SYS_FUNC_EN,
            (self.read32(REG_EXT_SYS_FUNC_EN)? | 0x0000_3000) & 0xFFFF_FF3F,
        )?;
        self.write8(REG_TXDMA_PQ_MAP + 1, DMA_MAPPING_HIGH << 6)?; // HIQ hi-priority
        self.write8(REG_CR, BIT_HCI_TXDMA_EN | BIT_TXDMA_EN)?; // TXDMA on
        // Download queue page config (hi=0xD0, pub=0x20, boundary=0x80).
        self.write8(REG_RQPN_CTRL_HLPQ, 0xD0)?;
        self.write8(REG_RQPN_CTRL_HLPQ + 1, 0x00)?;
        self.write8(REG_RQPN_CTRL_HLPQ + 2, 0x20)?;
        self.write8(REG_RQPN_CTRL_HLPQ + 3, 0x80)?;
        // Disable beacon-related functions during download.
        let bcn = self.read8(REG_BCN_CTRL)?;
        self.write8(REG_BCN_CTRL, (bcn & !(1 << 3)) | (1 << 4))?;
        Ok(())
    }

    /// **M4**: download one reserved page — a TX descriptor + `data` sent over
    /// bulk-OUT, landed in the beacon page `pg_addr`, confirmed by BCN_VALID.
    /// This is the transport the firmware sections ride on (M5). Returns `Ok(())`
    /// only when the hardware raised BCN_VALID (the packet reached the buffer).
    pub fn dl_rsvd_page(&self, pg_addr: u8, data: &[u8]) -> Result<(), FaceError> {
        // Set beacon head page and clear BCN_VALID (write-1-to-clear at +2 bit0).
        self.write8(REG_DWBCN0_CTRL + 1, pg_addr)?;
        self.write8(REG_DWBCN0_CTRL + 2, self.read8(REG_DWBCN0_CTRL + 2)? | 0x01)?;
        // Enable sw-beacon mode, mask the beacon queue (saved/restored).
        let cr1 = self.read8(REG_CR + 1)?;
        self.write8(REG_CR + 1, cr1 | 0x01)?;
        let txq2 = self.read8(REG_FWHW_TXQ_CTRL + 2)?;
        self.write8(REG_FWHW_TXQ_CTRL + 2, txq2 & !(1 << 6))?;

        // Build TXDESC + payload; USB pads so the length isn't an exact multiple
        // of 512 (the vendor +1 rule) to avoid the zero-length-packet ambiguity.
        let mut pkt = Vec::with_capacity(TX_DESC_SIZE + data.len() + 1);
        pkt.extend_from_slice(&download_txdesc(data.len()));
        pkt.extend_from_slice(data);
        if pkt.len() % 512 == 0 {
            pkt.push(0);
        }
        // The reserved-page/HIQ write goes to the highest-priority OUT endpoint
        // (the last one), not the default data endpoint.
        let ep = *self.bulk_outs.last().unwrap_or(&self.bulk_out);
        let wrote = self
            .handle
            .write_bulk(ep, &pkt, Duration::from_secs(1))
            .map_err(usb_err)?;
        if wrote != pkt.len() {
            return Err(io_err(format!(
                "dl_rsvd_page: short bulk write {wrote}/{}",
                pkt.len()
            )));
        }

        // Poll BCN_VALID (bit0 of REG_DWBCN0_CTRL+2) — the packet reached the page.
        let deadline = Instant::now() + Duration::from_millis(200);
        let mut valid = false;
        while Instant::now() < deadline {
            if self.read8(REG_DWBCN0_CTRL + 2)? & 0x01 != 0 {
                valid = true;
                break;
            }
        }

        // Restore: re-arm valid + the saved queue/CR bits.
        self.write8(REG_DWBCN0_CTRL + 2, self.read8(REG_DWBCN0_CTRL + 2)? | 0x01)?;
        self.write8(REG_FWHW_TXQ_CTRL + 2, txq2)?;
        self.write8(REG_CR + 1, cr1)?;

        if valid {
            Ok(())
        } else {
            Err(io_err("dl_rsvd_page: BCN_VALID poll timeout".into()))
        }
    }

    /// The discovered bulk endpoints (for the later TX/RX milestones).
    pub fn bulk_out(&self) -> u8 {
        self.bulk_out
    }
    /// All bulk-OUT endpoints (one per TX queue priority).
    pub fn bulk_outs(&self) -> &[u8] {
        &self.bulk_outs
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

#[cfg(test)]
mod golden {
    //! Golden-frame tests: pin the bytes the 8731bu/8733bu driver constructs and
    //! its decode of the *real* embedded firmware, so the M1–M4 wire format is
    //! verified without hardware and regressions are caught. These values are the
    //! ground truth to cross-check captured reference-driver USB traffic against
    //! (usbmon on a host with the real device — the vendor `rtl8733bu` driver).
    use super::*;

    /// The halmac firmware header decodes to exactly these values for the shipped
    /// `fw/rtl8733b_fw_nic.bin` — golden.
    #[test]
    fn fw_header_matches_embedded_image() {
        let h = FwHeader::parse(FW_NIC_8733B).expect("parse embedded fw header");
        assert_eq!(h.signature, 0x8723, "halmac signature");
        assert_eq!(h.version, 1);
        assert_eq!(h.subversion, 25);
        assert_eq!(h.dmem_addr, 0x1420_0000);
        assert_eq!(h.dmem_size, 16_008);
        assert_eq!(h.imem_addr, 0x1404_0000);
        assert_eq!(h.imem_size, 105_920);
        assert!(!h.has_emem, "the NIC image has no EMEM section");
        assert_eq!(h.emem_size, 0);
    }

    /// The strongest check: the header decode reproduces the real file length
    /// (header + each present section + its 8-byte checksum). If the offsets or
    /// masking were wrong this would not add up.
    #[test]
    fn nonsecure_len_reproduces_file_length() {
        let h = FwHeader::parse(FW_NIC_8733B).unwrap();
        assert_eq!(h.nonsecure_len() as usize, FW_NIC_8733B.len());
        assert_eq!(FW_NIC_8733B.len(), 122_008);
    }

    /// The reserved-page download descriptor for a 64-byte payload — golden bytes
    /// plus the field decode (TXPKTSIZE / OFFSET / QSEL).
    #[test]
    fn download_txdesc_golden() {
        let d = download_txdesc(64);
        let mut want = [0u8; TX_DESC_SIZE];
        want[..8].copy_from_slice(&[0x40, 0x00, 0x28, 0x00, 0x00, 0x12, 0x00, 0x00]);
        assert_eq!(d, want, "download_txdesc(64) golden bytes");
        assert_eq!(u16::from_le_bytes([d[0], d[1]]), 64, "TXPKTSIZE = payload len");
        assert_eq!(d[2], TX_DESC_SIZE as u8, "OFFSET = descriptor size");
        let qsel = (u32::from_le_bytes([d[4], d[5], d[6], d[7]]) >> 8) & 0x1F;
        assert_eq!(qsel, QSLT_MGNT & 0x1F, "QSEL = management queue");
    }
}
