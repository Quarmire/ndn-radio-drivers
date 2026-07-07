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

use std::sync::atomic::Ordering;
use std::sync::Arc;
use std::time::{Duration, Instant};

use async_trait::async_trait;
use ndn_frame_io::{frame, CapturedFrame, FrameFormat, FrameIo, InjectFrame};
use ndn_radio_hal::{Bandwidth, McsDescriptor, RadioKnobs, WifiRadio};
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

/// The 8733b MAC register table (`array_mp_8733b_mac_reg`, phydm halhwimg8733b_mac.c):
/// flat `[addr, value, …]` pairs terminated by `0xFFFF`. All byte writes, no
/// condition blocks. `0x002 = 0xC3` enables the BB block.
const MAC_REG_8733B: &[u32] = &[
    0x002, 0x0000_00C3, 0x55C, 0x0000_0050, 0x638, 0x0000_0050, 0x639, 0x0000_0019,
    0x640, 0x0000_0019, 0x63C, 0x0000_000D, 0x63D, 0x0000_000D, 0x63E, 0x0000_000D,
    0x63F, 0x0000_000D, 0x4CA, 0x0000_003F, 0x66C, 0x0000_0004, 0x520, 0x0000_006F,
    0x5A7, 0x0000_00FF, 0x6A2, 0x0000_00FF, 0x6A3, 0x0000_00FF, 0x4E5, 0x0000_00E0,
    0x4E6, 0x0000_0009, 0xFFFF, 0x0000_FFFF,
];

/// 8733b baseband tables (`array_mp_8733b_phy_reg` / `_agc_tab`) and the RF/radioA
/// table (`array_mp_8733b_radioa`), extracted from the vendor phydm C arrays to
/// little-endian `u32` `.bin`s.
const PHY_REG_8733B: &[u8] = include_bytes!("../fw/rtl8733b/rtl8733b_phy_reg.bin");
const AGC_TAB_8733B: &[u8] = include_bytes!("../fw/rtl8733b/rtl8733b_agc_tab.bin");
const RADIOA_8733B: &[u8] = include_bytes!("../fw/rtl8733b/rtl8733b_radioa.bin");
/// Calibration-init table (`array_mp_8733b_cal_init`, halrf_rfk_init_8733b.c): enables
/// the 0x1b register page and loads the NCTL (KIP) microcode — a prerequisite for IQK/DPK.
const CAL_INIT_8733B: &[u8] = include_bytes!("../fw/rtl8733b/rtl8733b_cal_init.bin");

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
const QSLT_BEACON: u32 = 0x10; // beacon queue-select — the vendor's download qsel
const DMA_MAPPING_HIGH: u8 = 3; // HALMAC_DMA_MAPPING_HIGH — HIQ priority
const BIT_HCI_TXDMA_EN: u8 = 0x01; // REG_CR BIT(0)
const BIT_TXDMA_EN: u8 = 0x04; // REG_CR BIT(2)

// M5 firmware download — the IDDMA (internal DMA) copy from the reserved-page
// packet buffer into the CPU's IMEM/DMEM, then the boot handshake. Register
// addresses + values verified against the reference driver's usbmon capture.
const REG_MCUFW_CTRL: u16 = 0x0080;
const REG_TXDMA_STATUS: u16 = 0x0210;
const REG_DDMA_CH0SA: u16 = 0x1200; // DDMA ch0 source address
const REG_DDMA_CH0DA: u16 = 0x1204; // DDMA ch0 destination address
const REG_DDMA_CH0CTRL: u16 = 0x1208; // DDMA ch0 control
// M6 tail — normal-mode TRX/queue init (init_trx_cfg_8733b).
const REG_RQPN_NPQ: u16 = 0x0214;
const REG_BCNQ_BDNY: u16 = 0x0424;
const REG_BCNQ2_BDNY: u16 = 0x0455;
const REG_AUTO_LLT: u16 = 0x0224;
const REG_TXDMA_OFFSET_CHK: u16 = 0x020C;
// M8 — RX control / filter maps (monitor mode).
const REG_RCR: u16 = 0x0608;
const REG_RXFLTMAP0: u16 = 0x06A0; // mgmt
const REG_RXFLTMAP1: u16 = 0x06A2; // ctrl
const REG_RXFLTMAP2: u16 = 0x06A4; // data
// 8733b data-path TX descriptor. The LIVE vendor driver uses a 40-byte descriptor
// (OFFSET=0x28) — confirmed by usbmon capture of its inject on the OPi — NOT the 48 the
// halmac struct suggests. A 48-byte descriptor misplaces the frame by 8 bytes (the DMA
// keys off the fixed 40) so the PHY transmits garbage: MAC accepts it, nothing radiates.
const DATA_TX_DESC_SIZE: usize = 40;
const IDDMA_SRC: u32 = 0x1878_0028; // OCPBASE_TXBUF + 40 (skip the txdesc); constant per capture
const DDMA_OWN: u32 = 1 << 31; // BIT_DDMACH0_OWN (start / busy)
const DDMA_CHKSUM_EN: u32 = 1 << 29; // BIT_DDMACH0_CHKSUM_EN
const DDMA_CHKSUM_STS: u32 = 1 << 27; // BIT_DDMACH0_CHKSUM_STS (1 = error)
const DDMA_RESET_CHKSUM: u32 = 1 << 25; // BIT_DDMACH0_RESET_CHKSUM_STS
const DDMA_CHKSUM_CONT: u32 = 1 << 24; // BIT_DDMACH0_CHKSUM_CONT
const DDMA_DLEN_MASK: u32 = 0x3FFFF; // 18-bit length
// Reserved-page chunk size. The vendor uses 4096, but that makes each bulk write
// TX_DESC_SIZE + 4096 = 4136 bytes, and the macOS USB stack truncates bulk transfers
// at 4096 — silently dropping the last 40 bytes of every chunk and corrupting the
// firmware in IMEM (the CPU then boots into garbage). 2048 keeps each transfer at
// 2088 bytes, well under the cap, and works identically on Linux.
const FW_CHUNK: usize = 2048;

/// Build the 40-byte TX descriptor for a reserved-page download packet, exactly as
/// the vendor `usb_write_data_not_xmitframe` → `rtl8733b_cal_txdesc_chksum`:
/// TXPKTSIZE + OFFSET(=40) in dword0, QSEL=BEACON in dword1, and a TX-descriptor
/// checksum at offset 0x1C = `~(XOR of the 40-byte descriptor's u16 words)`. The
/// checksum is mandatory: the hardware silently drops packets with a wrong one
/// (verified on air against the reference driver's usbmon capture).
fn download_txdesc(payload_len: usize) -> [u8; TX_DESC_SIZE] {
    let mut d = [0u8; TX_DESC_SIZE];
    let dw0 = (payload_len as u32 & 0xFFFF) | ((TX_DESC_SIZE as u32 & 0xFF) << 16);
    d[0..4].copy_from_slice(&dw0.to_le_bytes());
    let dw1 = (QSLT_BEACON & 0x1F) << 8;
    d[4..8].copy_from_slice(&dw1.to_le_bytes());
    // Checksum: field at 0x1C stays 0 while XORing the 20 little-endian u16 words.
    let mut ck: u16 = 0;
    for i in 0..TX_DESC_SIZE / 2 {
        ck ^= u16::from_le_bytes([d[2 * i], d[2 * i + 1]]);
    }
    d[0x1C..0x1E].copy_from_slice(&(!ck).to_le_bytes());
    d
}

/// Build the 48-byte data-path TX descriptor for a fixed-rate raw 802.11 inject:
/// TXPKTSIZE + OFFSET=48, QSEL=MGT, disable-agg, USE_RATE + DISDATAFB/DISRTSFB (fixed
/// rate), retry-limit-enable, SEC_TYPE=0 (raw), EN_HWSEQ=0 with a 12-bit SW sequence,
/// and the TX-desc checksum (`~XOR` of the first 32 bytes' u16 words) at 0x1C.
fn build_data_txdesc(
    frame_len: usize,
    rate: u8,
    seq: u16,
    bcast: bool,
    flags: u8,
) -> [u8; DATA_TX_DESC_SIZE] {
    let mut d = [0u8; DATA_TX_DESC_SIZE];
    let mut dw0 = (frame_len as u32 & 0xFFFF) | ((DATA_TX_DESC_SIZE as u32 & 0xFF) << 16);
    if bcast {
        dw0 |= 1 << 24; // BMC
    }
    d[0..4].copy_from_slice(&dw0.to_le_bytes());
    d[4..8].copy_from_slice(&((0x12u32 & 0x1F) << 8).to_le_bytes()); // dw1: QSEL=MGT
    d[8..12].copy_from_slice(&(1u32 << 16).to_le_bytes()); // dw2: BK (disable aggregation)
    d[12..16].copy_from_slice(&((1u32 << 8) | (1 << 9) | (1 << 10)).to_le_bytes()); // dw3: USE_RATE|DISRTSFB|DISDATAFB
    d[16..20].copy_from_slice(&((rate as u32 & 0x7F) | (1 << 17)).to_le_bytes()); // dw4: DATARATE|RTY_LMT_EN
    // dw5: DATA_SHORT/SGI(BIT4), DATA_BW[6:5], DATA_LDPC(BIT7), DATA_STBC[9:8].
    let sgi = (flags >> 1) & 1;
    let stbc = (flags >> 2) & 1;
    let ldpc = flags & 1;
    let bw = (flags >> 3) & 1; // 0=20, 1=40 MHz
    let dw5 = ((sgi as u32) << 4)
        | ((bw as u32) << 5)
        | ((ldpc as u32) << 7)
        | ((stbc as u32) << 8);
    d[20..24].copy_from_slice(&dw5.to_le_bytes());
    d[36..40].copy_from_slice(&(((seq as u32) & 0xFFF) << 12).to_le_bytes()); // dw9: SW_SEQ
    // Checksum over the FIRST 32 bytes (16 u16 words, dw0..dw7) with the field at 0x1C
    // left zero — NOT the whole descriptor (dw8/dw9 hold the seq and are excluded).
    let mut ck: u16 = 0;
    for i in 0..16 {
        ck ^= u16::from_le_bytes([d[2 * i], d[2 * i + 1]]);
    }
    d[0x1C..0x1E].copy_from_slice(&(!ck).to_le_bytes());
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
    /// On-air frame format for the [`FrameIo`] backend (default `RawNdn`).
    format: FrameFormat,
    /// Fixed TX HwRate for [`FrameIo::inject`] (default 6 Mbps OFDM `0x04`).
    tx_rate: std::sync::atomic::AtomicU8,
    /// Rolling 12-bit SW sequence number for injected frames.
    tx_seq: std::sync::atomic::AtomicU16,
    /// TX PHY flags for the descriptor: bit0 LDPC, bit1 short-GI, bit2 STBC,
    /// bit3 40 MHz (see [`set_tx_flags`](Rtl8733buBackend::set_tx_flags)).
    tx_flags: std::sync::atomic::AtomicU8,
    /// Parsed RX frames buffered between bulk-IN reads (one read yields many).
    rx_pending: std::sync::Mutex<std::collections::VecDeque<CapturedFrame>>,
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
            format: FrameFormat::default(),
            tx_rate: std::sync::atomic::AtomicU8::new(0x04), // 6 Mbps OFDM
            tx_seq: std::sync::atomic::AtomicU16::new(0),
            tx_flags: std::sync::atomic::AtomicU8::new(0),
            rx_pending: std::sync::Mutex::new(std::collections::VecDeque::new()),
        })
    }

    /// Select the on-air frame format for the [`FrameIo`] backend (default `RawNdn`).
    pub fn with_format(mut self, format: FrameFormat) -> Self {
        self.format = format;
        self
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

    /// Read a 16-bit register (little-endian on the wire).
    pub fn read16(&self, addr: u16) -> Result<u16, FaceError> {
        let mut buf = [0u8; 2];
        self.handle
            .read_control(VENQT_READ, VENQT_REQ, addr, 0, &mut buf, CTRL_TIMEOUT)
            .map_err(usb_err)?;
        Ok(u16::from_le_bytes(buf))
    }

    /// Write a 16-bit register (little-endian on the wire).
    pub fn write16(&self, addr: u16, val: u16) -> Result<(), FaceError> {
        self.handle
            .write_control(VENQT_WRITE, VENQT_REQ, addr, 0, &val.to_le_bytes(), CTRL_TIMEOUT)
            .map_err(usb_err)?;
        Ok(())
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
        // BIT(17) = DDMA_FUNC_EN gates the IDDMA registers (0x1200): if it is clear the
        // DMA engine is inert and the firmware never reaches IMEM. The Linux capture had
        // it set (EXT_FUNC=0x0003300f) by power-on default; a fresh macOS-side chip has
        // it clear (0x0001300f), which is exactly what blocked FW download on macOS —
        // set it explicitly so both platforms enable the DMA.
        self.write32(
            REG_EXT_SYS_FUNC_EN,
            (self.read32(REG_EXT_SYS_FUNC_EN)? | 0x0002_3000) & 0xFFFF_FF3F,
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
        // The beacon/reserved-page write goes to the HIGH-priority queue, which maps
        // to bulkout_id 0 = the FIRST OUT endpoint (verified against the reference
        // driver's usbmon capture — Bo:…:ep5, the lowest-numbered OUT ep).
        let ep = *self.bulk_outs.first().unwrap_or(&self.bulk_out);
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

    /// **M5**: download the firmware and boot the WLAN CPU. Enable download mode,
    /// push each memory section (DMEM then IMEM) to the reserved page in 4 KB
    /// chunks and IDDMA-copy it into the CPU's IMEM/DMEM, verify per-section
    /// checksums, then release the CPU and poll for firmware-ready. On success the
    /// on-chip WLAN CPU is running the NIC firmware. (Vendor `download_firmware` /
    /// `start_dlfw` / `dlfw_end_flow`, verified against a usbmon capture.)
    ///
    /// Verified on Linux: `MCUFW_CTRL=0xe079` (bit 15 booted) + a live `FW_DBG7` PC.
    ///
    /// Works on macOS too, after fixing two platform issues: (1) macOS caps bulk-OUT
    /// transfers at 4096 B, so a 4096-byte chunk (4136 with the txdesc) lost its tail —
    /// fixed by [`FW_CHUNK`] = 2048. (2) The IDDMA registers (0x1200) appeared inert on
    /// macOS because `DDMA_FUNC_EN` (BIT(17) of `REG_EXT_SYS_FUNC_EN`) was clear — the
    /// Linux/OPi chip had it set by power-on default, a fresh macOS-side chip did not —
    /// so `fw_dl_setup` now sets it explicitly. Verified on both: `MCUFW_CTRL=0xe079`.
    pub fn download_firmware(&self) -> Result<(), FaceError> {
        let hdr = Self::fw_header()?;
        let fw = FW_NIC_8733B;

        // wlan_cpu_en(0): hold the WLAN CPU off during the download.
        let v = self.read8(REG_SYS_FUNC_EN + 1)?;
        self.write8(REG_SYS_FUNC_EN + 1, v & !(1 << 2))?;

        // pltfm_reset: toggle BIT0 of REG_EXT_SYS_FUNC_EN+2 (0x1002), clear then set —
        // the platform reset the vendor does after fw_dl_setup and before download
        // mode. Without it the CPU never boots (bit 15 stays clear) even though the
        // download and checksums pass.
        let r = self.read8(REG_EXT_SYS_FUNC_EN + 2)?;
        self.write8(REG_EXT_SYS_FUNC_EN + 2, r & !1)?;
        let r = self.read8(REG_EXT_SYS_FUNC_EN + 2)?;
        self.write8(REG_EXT_SYS_FUNC_EN + 2, r | 1)?;

        // start_dlfw: enter FW-download mode. FWDL_EN (BIT0) + the MCU boot-select
        // bit 13 (0x2000): the reference driver had it set in REG_MCUFW_CTRL & 0x3800
        // before download; a fresh chip reads 0, so set it explicitly (without it the
        // CPU never sets the FW-booted bit 15 even though the download + checksums pass).
        let mcufw = self.read16(REG_MCUFW_CTRL)? & 0x3800;
        self.write16(REG_MCUFW_CTRL, mcufw | 0x2000 | 0x0001)?;

        // Section layout: header(64) | DMEM(size+8) | IMEM(size+8).
        let hdr_sz = FW_HDR_SIZE as usize;
        let dmem_len = (hdr.dmem_size + FW_HDR_CHKSUM_SIZE) as usize;
        let imem_len = (hdr.imem_size + FW_HDR_CHKSUM_SIZE) as usize;
        let dmem = &fw[hdr_sz..hdr_sz + dmem_len];
        let imem = &fw[hdr_sz + dmem_len..hdr_sz + dmem_len + imem_len];
        self.dlfw_section(dmem, hdr.dmem_addr)?;
        self.dlfw_section(imem, hdr.imem_addr)?;

        self.dlfw_end_flow()
    }

    /// Download one firmware memory section: stream it through the reserved page in
    /// `FW_CHUNK`-byte pieces, IDDMA-copying each into `dest_base`, then flag the
    /// section's download/checksum-OK bits in `REG_MCUFW_CTRL`.
    fn dlfw_section(&self, data: &[u8], dest_base: u32) -> Result<(), FaceError> {
        // Reset the DDMA checksum accumulator at the start of each section (vendor
        // dlfw_to_mem does this per section, not once).
        let ctrl = self.read32(REG_DDMA_CH0CTRL)?;
        self.write32(REG_DDMA_CH0CTRL, ctrl | DDMA_RESET_CHKSUM)?;
        let mut off = 0usize;
        let mut first = true;
        while off < data.len() {
            let n = FW_CHUNK.min(data.len() - off);
            self.dl_rsvd_page(0, &data[off..off + n])?;
            self.iddma_copy(dest_base + off as u32, n as u32, first)?;
            off += n;
            first = false;
        }
        // Checksum status (0 = OK) after the section.
        if self.read32(REG_DDMA_CH0CTRL)? & DDMA_CHKSUM_STS != 0 {
            return Err(io_err(format!("fw section @0x{dest_base:08x}: DDMA checksum error")));
        }
        // Flag DW_OK|CHKSUM_OK: DMEM (base >= 0x14200000) = 0x60, IMEM = 0x18.
        let bits = if dest_base >= 0x1420_0000 { 0x60 } else { 0x18 };
        let v = self.read8(REG_MCUFW_CTRL)?;
        self.write8(REG_MCUFW_CTRL, v | bits)?;
        Ok(())
    }

    /// One IDDMA copy: packet buffer ([`IDDMA_SRC`]) → `dest`, `len` bytes, waiting
    /// for the channel to be free before and after.
    fn iddma_copy(&self, dest: u32, len: u32, first: bool) -> Result<(), FaceError> {
        self.poll_ddma_idle()?;
        let mut ctrl = DDMA_OWN | DDMA_CHKSUM_EN | (len & DDMA_DLEN_MASK);
        if !first {
            ctrl |= DDMA_CHKSUM_CONT;
        }
        self.write32(REG_DDMA_CH0SA, IDDMA_SRC)?;
        self.write32(REG_DDMA_CH0DA, dest)?;
        self.write32(REG_DDMA_CH0CTRL, ctrl)?;
        self.poll_ddma_idle()
    }

    fn poll_ddma_idle(&self) -> Result<(), FaceError> {
        let deadline = Instant::now() + Duration::from_millis(200);
        while Instant::now() < deadline {
            if self.read32(REG_DDMA_CH0CTRL)? & DDMA_OWN == 0 {
                return Ok(());
            }
        }
        Err(io_err("IDDMA: OWN poll timeout".into()))
    }

    /// Release the WLAN CPU and wait for firmware-ready. Verifies both section
    /// checksums, sets FW_DW_RDY + clears FWDL_EN, enables the CPU, and polls
    /// `REG_MCUFW_CTRL & 0xC078 == 0xC078` (both ready bits + all download/checksum
    /// bits).
    fn dlfw_end_flow(&self) -> Result<(), FaceError> {
        // Restore the MAC registers the download prologue changed — the CPU needs a
        // clean TX-DMA state to boot (values per the reference driver's restore).
        self.write8(REG_TXDMA_PQ_MAP + 1, 0)?;
        self.write8(REG_CR, 0)?;
        self.write8(REG_RQPN_CTRL_HLPQ, 0)?;
        self.write8(REG_RQPN_CTRL_HLPQ + 1, 0)?;
        self.write8(REG_RQPN_CTRL_HLPQ + 2, 0)?;
        self.write8(REG_BCN_CTRL, 0x14)?;

        self.write32(REG_TXDMA_STATUS, 0x0004)?; // clear TXDMA
        let fw_ctrl = self.read16(REG_MCUFW_CTRL)?;
        if fw_ctrl & 0x50 != 0x50 {
            return Err(io_err(format!("fw checksums not OK (MCUFW_CTRL=0x{fw_ctrl:04x})")));
        }
        // Set FW_DW_RDY (BIT14), clear FWDL_EN (BIT0).
        self.write16(REG_MCUFW_CTRL, (fw_ctrl | 0x4000) & !0x0001)?;
        // wlan_cpu_en(1): set REG_SYS_FUNC_EN+1 BIT2.
        let v = self.read8(REG_SYS_FUNC_EN + 1)?;
        self.write8(REG_SYS_FUNC_EN + 1, v | (1 << 2))?;
        let dbg = std::env::var("M5DBG").is_ok();
        if dbg {
            eprintln!(
                "  [end_flow] pre-boot fw_ctrl=0x{fw_ctrl:04x}  MCUFW after boot-write=0x{:04x}  SYS_FUNC_EN+1=0x{:02x} (bit2={})",
                self.read16(REG_MCUFW_CTRL)?,
                self.read8(REG_SYS_FUNC_EN + 1)?,
                self.read8(REG_SYS_FUNC_EN + 1)? & 0x04 != 0,
            );
        }
        // Poll FW-ready: both ready bits (0xC000) + DW/checksum bits (0x78).
        let deadline = Instant::now() + Duration::from_millis(300);
        let mut ticks = 0u32;
        while Instant::now() < deadline {
            let m = self.read16(REG_MCUFW_CTRL)?;
            if dbg && ticks < 6 {
                eprintln!("  [poll {ticks}] MCUFW=0x{m:04x} FW_DBG7=0x{:08x}", self.read32(0x10AC)?);
            }
            ticks += 1;
            if m & 0xC078 == 0xC078 {
                return Ok(());
            }
        }
        Err(io_err(format!(
            "fw-ready poll timeout (MCUFW_CTRL=0x{:04x})",
            self.read16(REG_MCUFW_CTRL)?
        )))
    }

    // ── M6: MAC / BB / RF register-table initialization ──────────────────────

    /// Apply a phydm register table: flat `[addr, val, addr, val, …]` u32 words,
    /// terminated by `addr == 0xFFFF`, honoring the IF/ELSE/ENDIF condition markers
    /// (high bits `0xC000_0000` of the address word). For a standard 1x1 NIC the
    /// conditions we care about (cut/interface/RFE) all take the default branch, so
    /// [`cond_matches`] evaluates the stored IF and picks the matching side.
    fn config_table(
        &self,
        table: &[u32],
        mut apply: impl FnMut(&Self, u32, u32) -> Result<(), FaceError>,
    ) -> Result<(), FaceError> {
        let mut is_matched = true;
        let mut is_skipped = false;
        let mut pre = 0u32;
        let mut i = 0;
        while i + 1 < table.len() {
            let (v1, v2) = (table[i], table[i + 1]);
            if v1 == 0xFFFF {
                break; // table terminator
            }
            if v1 & 0xC000_0000 != 0 {
                if v1 & 0x8000_0000 != 0 {
                    let cond = ((v1 >> 28) & 0x3) as u8;
                    if cond == 0x2 {
                        // ENDIF
                        is_matched = true;
                        is_skipped = false;
                    } else if cond == 0x3 {
                        // ELSE
                        is_matched = !is_skipped;
                    } else {
                        pre = v1; // IF / ELSE-IF: remember for the negative entry
                    }
                } else if !is_skipped {
                    if self.cond_matches(pre, v2) {
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

    /// Evaluate a phydm IF condition against our driver profile (`check_positive`):
    /// value-check cut[27:24] / pkg[15:12] / interface[11:8] (a zero cond nibble is a
    /// wildcard), then an exact RFE low-byte match. Profile: a 1x1 USB NIC — cut D
    /// (chip_ver 3), USB interface (0x2), package wildcard, RFE type 0 (internal
    /// PA/LNA). Branches for other RFE/cut variants are skipped, so their ELSE/default
    /// entries apply instead.
    fn cond_matches(&self, cond1: u32, _cond2: u32) -> bool {
        const CUT: u32 = 3; // ODM_CUT_D
        const PKG: u32 = 15; // package_type 0 → 15
        const IFACE: u32 = 0x2; // ODM_ITRF_USB & 0x0F
        const RFE: u32 = 0; // internal PA/LNA
        if cond1 & 0x0F00_0000 != 0 && (cond1 >> 24) & 0xF != CUT {
            return false;
        }
        if cond1 & 0x0000_F000 != 0 && (cond1 >> 12) & 0xF != PKG {
            return false;
        }
        if cond1 & 0x0000_0F00 != 0 && (cond1 >> 8) & 0xF != IFACE {
            return false;
        }
        cond1 & 0xFF == RFE
    }

    /// Apply a phydm table stored as little-endian `u32` bytes (a `.bin` extracted
    /// from the vendor's `array_mp_8733b_*`).
    fn config_table_bin(
        &self,
        bin: &[u8],
        apply: impl FnMut(&Self, u32, u32) -> Result<(), FaceError>,
    ) -> Result<(), FaceError> {
        let words: Vec<u32> = bin
            .chunks_exact(4)
            .map(|c| u32::from_le_bytes([c[0], c[1], c[2], c[3]]))
            .collect();
        self.config_table(&words, apply)
    }

    /// One baseband-table write: a delay opcode (`0xfe..0xf9`, `0xffff`) or a 32-bit
    /// BB register write (`odm_config_bb_phy_8733b` → `odm_set_bb_reg`, full dword).
    fn bb_write(&self, addr: u32, data: u32) -> Result<(), FaceError> {
        match addr {
            0xfe | 0xffe => std::thread::sleep(Duration::from_millis(50)),
            0xfd => std::thread::sleep(Duration::from_millis(5)),
            0xfc => std::thread::sleep(Duration::from_millis(1)),
            0xfb => std::thread::sleep(Duration::from_micros(50)),
            0xfa => std::thread::sleep(Duration::from_micros(5)),
            0xf9 | 0xffff => std::thread::sleep(Duration::from_micros(1)),
            _ => self.write32(addr as u16, data)?,
        }
        Ok(())
    }

    /// **M6b**: baseband init — apply the BB PHY-register table then the AGC table
    /// (`array_mp_8733b_phy_reg` + `array_mp_8733b_agc_tab`). The BB block is already
    /// powered by [`mac_config`] (`0x002 = 0xC3`). Run after [`mac_config`].
    pub fn bb_config(&self) -> Result<(), FaceError> {
        self.config_table_bin(PHY_REG_8733B, |s, a, d| s.bb_write(a, d))?;
        self.config_table_bin(AGC_TAB_8733B, |s, a, d| s.bb_write(a, d))?;
        Ok(())
    }

    /// One RF (radioA) write. The 8733b maps each path-A RF register directly into
    /// the BB address space (`config_phydm_direct_write_rf_reg_8733b`): `RF[addr]`
    /// → BB `0x3C00 + (addr<<2)`, a 20-bit (`RFREG_MASK`) value — no 3-wire LSSI.
    /// `addr` may be a delay opcode.
    fn rf_write(&self, addr: u32, data: u32) -> Result<(), FaceError> {
        match addr {
            0xffe => std::thread::sleep(Duration::from_millis(50)),
            0xfe => std::thread::sleep(Duration::from_micros(100)),
            0xffff => std::thread::sleep(Duration::from_micros(1)),
            _ => {
                let direct = (0x3C00 + ((addr & 0xFF) << 2)) as u16;
                let v = (self.read32(direct)? & !0x000F_FFFF) | (data & 0x000F_FFFF);
                self.write32(direct, v)?;
                std::thread::sleep(Duration::from_micros(1));
            }
        }
        Ok(())
    }

    /// **M6c**: RF init — apply the radioA table (`array_mp_8733b_radioa`) through the
    /// direct RF-write window. Run after [`bb_config`].
    pub fn rf_config(&self) -> Result<(), FaceError> {
        self.config_table_bin(RADIOA_8733B, |s, a, d| s.rf_write(a, d))
    }

    /// Read a path-A RF register (for verifying [`rf_config`]) via the direct window.
    pub fn rf_read(&self, addr: u32) -> Result<u32, FaceError> {
        Ok(self.read32((0x3C00 + ((addr & 0xFF) << 2)) as u16)? & 0x000F_FFFF)
    }

    /// **M6 tail**: normal-mode TRX/queue init (`init_trx_cfg_8733b`). Switches the
    /// download-mode page/RQPN config over to the normal-operation layout so the data
    /// path can run: queue→DMA map, enable all TRX, normal RQPN + reserved-page
    /// boundary, then the hardware auto-LLT (poll `REG_AUTO_LLT` BIT16 until it
    /// clears). All values are the reference driver's, taken from its usbmon capture.
    /// Run after the BB/RF tables ([`rf_config`]).
    pub fn init_trx(&self) -> Result<(), FaceError> {
        self.write16(REG_TXDMA_PQ_MAP, 0xF5A0)?; // queue → DMA mapping (normal)
        self.write8(REG_CR, 0x00)?;
        self.write8(REG_CR, 0xFF)?; // MAC_TRX_ENABLE — enable all TX/RX engines
        // RQPN: high=8, low=8, pub=211; normal=8, extra=0; then trigger.
        self.write32(REG_RQPN_CTRL_HLPQ, 0x00D3_0808)?;
        self.write32(REG_RQPN_NPQ, 0x0000_0008)?;
        self.write8(REG_RQPN_CTRL_HLPQ + 3, 0x80)?;
        // Reserved-page boundary (236) across the beacon-queue boundary regs.
        self.write8(REG_DWBCN0_CTRL + 1, 0xEC)?;
        self.write8(REG_BCNQ_BDNY, 0xEC)?;
        self.write8(REG_BCNQ2_BDNY, 0xEC)?;
        // Block-descriptor count + TXDMA offset check.
        self.write8(REG_DWBCN0_CTRL, 0x30)?;
        self.write16(REG_TXDMA_OFFSET_CHK, 0x0200)?; // +1 BIT1
        // Hardware auto-LLT: set BIT16 (+ params), poll until it self-clears.
        self.write32(REG_AUTO_LLT, 0x0001_2020)?;
        let deadline = Instant::now() + Duration::from_millis(200);
        let mut done = false;
        while Instant::now() < deadline {
            if self.read32(REG_AUTO_LLT)? & (1 << 16) == 0 {
                done = true;
                break;
            }
        }
        if !done {
            return Err(io_err("init_trx: AUTO_LLT BIT16 poll timeout".into()));
        }
        self.write8(REG_CR + 3, 0x00)?; // transfer mode = normal
        Ok(())
    }

    // ── M8: TX/RX data path (monitor-mode inject/capture) ────────────────────

    /// **M8**: inject a raw 802.11 frame at a fixed rate. Builds the 48-byte data TX
    /// descriptor (QSEL=MGT, USE_RATE + DISDATAFB fixed-rate, no encryption, SW seq)
    /// and sends `[desc][frame]` to the HIGH bulk-OUT endpoint (0x05). `rate` is a
    /// HwRate code (0x00 = 1M CCK, 0x04 = 6M OFDM). Run after [`init_trx`].
    pub fn inject_raw(&self, frame: &[u8], rate: u8, seq: u16) -> Result<(), FaceError> {
        let bcast = frame.len() > 4 && frame[4] & 0x01 != 0; // 802.11 addr1[0] group bit
        let desc = build_data_txdesc(frame.len(), rate, seq, bcast, self.tx_flags.load(Ordering::Relaxed));
        let mut pkt = Vec::with_capacity(desc.len() + frame.len() + 1);
        pkt.extend_from_slice(&desc);
        pkt.extend_from_slice(frame);
        if pkt.len() % 512 == 0 {
            pkt.push(0); // avoid a bulk-max-multiple transfer (ZLP); TXPKTSIZE bounds the frame
        }
        let ep = *self.bulk_outs.first().unwrap_or(&self.bulk_out);
        let wrote = self
            .handle
            .write_bulk(ep, &pkt, Duration::from_millis(500))
            .map_err(usb_err)?;
        if wrote != pkt.len() {
            return Err(io_err(format!("inject: short bulk write {wrote}/{}", pkt.len())));
        }
        Ok(())
    }

    /// **M8**: put the receiver in monitor mode — accept every frame (incl. CRC/ICV
    /// errors), append PHY status + FCS, and open all mgmt/ctrl/data filter maps.
    pub fn set_monitor(&self) -> Result<(), FaceError> {
        self.write32(REG_RCR, 0x9000_030F)?; // AAP|APM|AM|AB|ACRC32|AICV|APP_PHYSTS|APP_FCS
        self.write16(REG_RXFLTMAP0, 0xFFFF)?; // mgmt
        self.write16(REG_RXFLTMAP1, 0xFFFF)?; // ctrl
        self.write16(REG_RXFLTMAP2, 0xFFFF)?; // data
        Ok(())
    }

    /// Full monitor-mode bring-up: power-on → firmware → MAC/BB/RF init → normal TRX →
    /// tune `channel` → promiscuous RX. After this the backend injects and captures
    /// (the [`FrameIo`] path). Calibration is optional and omitted here (TX radiates
    /// without it). Note the chip wedges after repeated re-inits in a process — open
    /// once per power-cycle.
    pub fn bring_up_monitor(&self, channel: u8) -> Result<(), FaceError> {
        self.power_on()?;
        self.fw_dl_setup()?;
        self.download_firmware()?;
        self.mac_config()?;
        self.bb_config()?;
        self.rf_config()?;
        self.init_trx()?;
        self.tune_channel(channel)?;
        self.set_monitor()?;
        self.enable_tx_path()?; // RF mode table → TX + BB CCK TX (normal-op TX enable)
        self.set_txagc_table(0x2d)?; // per-rate TX gain (0 by default → no output)
        let _ = self.configure_trsw(true); // external TRSW antenna routing (best-effort)
        Ok(())
    }

    /// **M8**: read one bulk-IN transfer and split out the 802.11 frames it packs.
    /// Each frame sits at `24 + drvinfo*8 + shift` after its 24-byte RX descriptor,
    /// and successive frames are 8-byte aligned; `DMA_AGG_NUM` counts them.
    pub fn capture(&self, timeout_ms: u64) -> Result<Vec<Vec<u8>>, FaceError> {
        let mut buf = vec![0u8; 16384];
        let n = match self.handle.read_bulk(self.bulk_in, &mut buf, Duration::from_millis(timeout_ms)) {
            Ok(n) => n,
            Err(rusb::Error::Timeout) => return Ok(Vec::new()),
            Err(e) => return Err(usb_err(e)),
        };
        let data = &buf[..n];
        let mut frames = Vec::new();
        let mut off = 0usize;
        while off + 24 <= data.len() {
            let dw0 = u32::from_le_bytes([data[off], data[off + 1], data[off + 2], data[off + 3]]);
            let pkt_len = (dw0 & 0x3FFF) as usize;
            if pkt_len == 0 {
                break;
            }
            let drvinfo = ((dw0 >> 16) & 0xF) as usize * 8;
            let shift = ((dw0 >> 24) & 0x3) as usize;
            let dw2 = u32::from_le_bytes([data[off + 8], data[off + 9], data[off + 10], data[off + 11]]);
            let is_c2h = dw2 & (1 << 28) != 0;
            let fstart = off + 24 + drvinfo + shift;
            if !is_c2h && fstart + pkt_len <= data.len() {
                frames.push(data[fstart..fstart + pkt_len].to_vec());
            }
            off += ((24 + drvinfo + shift + pkt_len) + 7) & !7; // _RND8 to next
        }
        Ok(frames)
    }

    /// **M6a**: apply the 8733b MAC register table (`array_mp_8733b_mac_reg`) — a
    /// short list of byte writes, including `0x002 = 0xC3` which brings up the BB
    /// block. Run after the firmware is booted.
    pub fn mac_config(&self) -> Result<(), FaceError> {
        self.config_table(MAC_REG_8733B, |s, addr, val| s.write8(addr as u16, val as u8))
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

// ── M7: IQK (IQ-imbalance) calibration — faithful port of halrf_iqk_8733b.c ──
// Part 1: masked BB/RF accessors, register backup/restore, AFE + preset setup.
// (LOK → TXK → RXK → fill_iqk_xy follow in subsequent parts.)

/// IQK register backup lists (halrf_iqk_8733b.c `_phy_iq_calibrate_8733b`).
const IQK_MAC_REGS: [u16; 1] = [0x0520];
const IQK_BB_REGS: [u16; 11] = [
    0x09f0, 0x09b4, 0x1c38, 0x1860, 0x1cd0, 0x0824, 0x2a24, 0x1d40, 0x1c20, 0x1880, 0x180c,
];
const IQK_RF_REGS: [u32; 5] = [0x05, 0xde, 0xdf, 0xef, 0x1f];
const RFREG_MASK: u32 = 0x000F_FFFF; // RF registers are 20-bit

/// DPK register backup lists (do_dpk_8733b).
const DPK_BB_REGS: [u16; 15] = [
    0x0522, 0x1884, 0x09f0, 0x2a24, 0x1830, 0x1d40, 0x1b38, 0x1b3c, 0x1bf8, 0x1e70, 0x1c38,
    0x1c68, 0x1864, 0x180c, 0x1880,
];
const DPK_RF_REGS: [u32; 9] = [0x0, 0x5, 0x83, 0x8c, 0x8f, 0x9e, 0xde, 0xdf, 0xef];
// DPK one-shot action tags (encode the shot_code in dpk_one_shot).
const DPK_PAS: u8 = 0;
const DO_DPK: u8 = 1;
const DPK_ON: u8 = 2;
const DPK_GAIN_LOSS: u8 = 3;

/// Backed-up register state, restored after calibration.
#[derive(Default)]
struct IqkBackup {
    mac: [u32; 1],
    bb: [u32; 11],
    rf: [[u32; 2]; 5], // [reg][path]
}

/// IQK measurement results — the per-path TX/RX correction coefficients.
#[derive(Default, Debug, Clone, Copy)]
pub struct IqkInfo {
    /// Band: 0 = 2.4 GHz (G-mode), 1 = 5 GHz (A-mode).
    pub band: u8,
    /// TX IQ coefficients `[path][x, y]`.
    pub txxy: [[u32; 2]; 2],
    /// RX IQ coefficients `[path][x, y][LNA small, LNA large]`.
    pub rxxy: [[[u32; 2]; 2]; 2],
    /// Whether the RX IQK succeeded (gates the RX-IQK correction enable).
    pub rxk_ok: bool,
}

impl Rtl8733buBackend {
    /// Masked baseband read/write (`odm_get_bb_reg` / `odm_set_bb_reg`).
    #[allow(dead_code)] // used by the LOK/TXK/RXK result reads (M7 part 2)
    fn bb_get(&self, addr: u16, mask: u32) -> Result<u32, FaceError> {
        Ok((self.read32(addr)? & mask) >> mask.trailing_zeros())
    }
    fn bb_set(&self, addr: u16, mask: u32, data: u32) -> Result<(), FaceError> {
        if mask == 0xFFFF_FFFF {
            self.write32(addr, data)
        } else {
            let sh = mask.trailing_zeros();
            let v = (self.read32(addr)? & !mask) | ((data << sh) & mask);
            self.write32(addr, v)
        }
    }

    /// The BB-space window for a path-A/B RF register (0x3C00 / 0x4C00 base).
    fn rf_win(path: u8, addr: u32) -> u16 {
        ((if path == 0 { 0x3C00u32 } else { 0x4C00 }) + ((addr & 0xFF) << 2)) as u16
    }
    /// Masked RF read/write (`odm_get_rf_reg` / `odm_set_rf_reg`, 20-bit).
    fn rf_get(&self, path: u8, addr: u32, mask: u32) -> Result<u32, FaceError> {
        let v = self.read32(Self::rf_win(path, addr))? & RFREG_MASK;
        Ok((v & mask) >> mask.trailing_zeros())
    }
    fn rf_set(&self, path: u8, addr: u32, mask: u32, data: u32) -> Result<(), FaceError> {
        let m = mask & RFREG_MASK;
        let w = Self::rf_win(path, addr);
        let cur = self.read32(w)? & RFREG_MASK;
        self.write32(w, (cur & !m) | ((data << m.trailing_zeros()) & m))?;
        std::thread::sleep(Duration::from_micros(1));
        Ok(())
    }

    /// Back up the MAC/BB/RF registers the IQK perturbs.
    fn iqk_backup(&self) -> Result<IqkBackup, FaceError> {
        let mut b = IqkBackup::default();
        for (i, &r) in IQK_MAC_REGS.iter().enumerate() {
            b.mac[i] = self.read32(r)?;
        }
        for (i, &r) in IQK_BB_REGS.iter().enumerate() {
            b.bb[i] = self.read32(r)?;
        }
        for (i, &r) in IQK_RF_REGS.iter().enumerate() {
            b.rf[i][0] = self.rf_get(0, r, RFREG_MASK)?;
            b.rf[i][1] = self.rf_get(1, r, RFREG_MASK)?;
        }
        Ok(b)
    }
    fn iqk_restore(&self, b: &IqkBackup) -> Result<(), FaceError> {
        for (i, &r) in IQK_MAC_REGS.iter().enumerate() {
            self.write32(r, b.mac[i])?;
        }
        for (i, &r) in IQK_BB_REGS.iter().enumerate() {
            self.write32(r, b.bb[i])?;
        }
        for (i, &r) in IQK_RF_REGS.iter().enumerate() {
            self.rf_set(0, r, RFREG_MASK, b.rf[i][0])?;
            self.rf_set(1, r, RFREG_MASK, b.rf[i][1])?;
        }
        Ok(())
    }

    /// AFE on/off for IQK mode (`_iqk_afe_setting_8733b`): ADDA on + clk gating +
    /// CCA block for `do_iqk`, else restore normal AFE.
    fn iqk_afe_setting(&self, do_iqk: bool) -> Result<(), FaceError> {
        if do_iqk {
            self.bb_set(0x1b08, 0xFFFF_FFFF, 0x80)?; // IQK/DPK KIP power on
            self.bb_set(0x1e24, 1 << 31, 0)?;
            self.bb_set(0x1e28, 0xF, 1)?;
            self.bb_set(0x0824, 0x000F_0000, 1)?;
            self.bb_set(0x1cd0, 0xF000_0000, 7)?; // IQK clk on
            self.bb_set(0x2a24, 1 << 13, 1)?; // block CCK CCA
            self.bb_set(0x1c68, 1 << 24, 1)?; // block OFDM CCA
            self.bb_set(0x1864, 1 << 31, 1)?; // trx gating clk force on
            self.bb_set(0x180c, 1 << 27, 1)?;
            self.bb_set(0x180c, 1 << 30, 1)?;
            self.bb_set(0x1e24, 1 << 17, 1)?;
            self.bb_set(0x1c38, 0xFFFF_FFFF, 0x0)?; // ADDA fifo force off
            self.bb_set(0x1830, 1 << 30, 0)?; // force ADDA
            self.bb_set(0x1860, 0xF000_0000, 0xf)?; // ADDA all on
            self.bb_set(0x1860, 0x0FFF_F000, 0x0041)?;
            self.bb_set(0x09f0, 0x0000_FFFF, 0xbbbb)?; // DAC clk 80M
            self.bb_set(0x1d40, 1 << 3, 1)?;
            self.bb_set(0x1d40, 0x7, 3)?;
            self.bb_set(0x09b4, 0x0000_0700, 3)?;
            self.bb_set(0x09b4, 0x0000_3800, 3)?;
            self.bb_set(0x09b4, 0x0001_C000, 3)?;
            self.bb_set(0x09b4, 0x000E_0000, 3)?;
            self.bb_set(0x1c20, 1 << 5, 1)?;
            self.bb_set(0x1c38, 0xFFFF_FFFF, 0xFFFF_FFFF)?; // release ADDA fifo
            // Re-assert KIP power ON *after* the IQK clock (0x1cd0) is up — the very
            // first afe write of 0x1b08=0x80 is lost because the KIP block has no clock
            // yet, so the NCTL engine never powers on without this.
            self.bb_set(0x1b08, 0xFFFF_FFFF, 0x80)?;
        } else {
            self.bb_set(0x1c38, 0xFFFF_FFFF, 0xffa1_005e)?;
            self.bb_set(0x1830, 1 << 30, 1)?;
            self.bb_set(0x1e24, 1 << 31, 1)?;
            self.bb_set(0x2a24, 1 << 13, 0)?;
            self.bb_set(0x1c68, 1 << 24, 0)?;
            self.bb_set(0x1864, 1 << 31, 0)?;
            self.bb_set(0x180c, 1 << 27, 0)?;
            self.bb_set(0x180c, 1 << 30, 0)?;
            self.bb_set(0x1880, 1 << 21, 0)?;
        }
        Ok(())
    }

    /// IQK preset / register restore (`_iqk_preset_8733b`).
    fn iqk_preset(&self, do_iqk: bool) -> Result<(), FaceError> {
        if do_iqk {
            self.rf_set(0, 0x05, 1 << 0, 0)?; // RF not controlled by BB
            self.rf_set(1, 0x05, 1 << 0, 0)?;
        } else {
            self.rf_set(0, 0x05, 1 << 0, 1)?;
            self.rf_set(1, 0x05, 1 << 0, 1)?;
            self.rf_set(0, 0xdf, 1 << 12, 0)?;
            self.rf_set(1, 0xdf, 1 << 12, 0)?;
            self.rf_set(0, 0xef, 1 << 2, 0)?;
            self.rf_set(1, 0xef, 1 << 2, 0)?;
            self.rf_set(0, 0xde, 0xFE000, 0)?;
            self.rf_set(1, 0xde, 0xFE000, 0)?;
            self.rf_set(0, 0xef, 1 << 2, 0)?;
            self.rf_set(1, 0xef, 1 << 2, 0)?;
            self.bb_set(0x1b08, 0xFFFF_FFFF, 0)?;
            self.bb_set(0x1b34, 0x0000_007C, 0)?;
            self.bb_set(0x1b38, 1 << 0, 0)?;
            self.bb_set(0x1bb8, 0xFFFF_FFFF, 0)?;
            self.bb_set(0x1bcc, 0x0000_003F, 0)?;
        }
        Ok(())
    }

    /// **RF-K init** (`odm_read_and_config_mp_8733b_cal_init`): apply the cal-init
    /// table — enable the 0x1b register page and load the NCTL/KIP microcode. Must run
    /// once (after M6 BB/RF init) before any IQK/DPK, or the NCTL engine has no routine
    /// to execute (the one-shot trigger is never consumed).
    pub fn rfk_init(&self) -> Result<(), FaceError> {
        let words: Vec<u32> = CAL_INIT_8733B
            .chunks_exact(4)
            .map(|c| u32::from_le_bytes([c[0], c[1], c[2], c[3]]))
            .collect();
        let mut i = 0;
        while i + 1 < words.len() {
            if let Err(e) = self.bb_write(words[i], words[i + 1]) {
                eprintln!(
                    "  [rfk] FAILED at pair {}/{} addr=0x{:04x} val=0x{:08x}: {e}",
                    i / 2,
                    words.len() / 2,
                    words[i],
                    words[i + 1]
                );
                return Err(e);
            }
            i += 2;
        }
        Ok(())
    }

    /// Tune the RF to a 2.4 GHz channel (`config_phydm_switch_channel_8733b`, G-band
    /// path): clear the band/channel bits of RF 0x18 and set the channel, write both
    /// paths with the cut-D settle loop (poll RF 0xc5 BIT15), restore RF 0x19, and
    /// select the 2.4 GHz RX AGC table. Prerequisite for calibration + TX/RX.
    /// Tune to `ch` on either band (`config_phydm_switch_channel_8733b`). `ch` ≤ 14 is
    /// 2.4 GHz; any higher value is treated as a 5 GHz channel number
    /// (`ch = (freq_MHz − 5000) / 5`) — the channel byte is written verbatim with no
    /// regulatory channel-list restriction, so arbitrary 5 GHz centres (the libc0607
    /// "unlock frequency" feature, ~5080–6030 MHz) are reachable. Combine with
    /// [`set_bandwidth`](Self::set_bandwidth) `Nb5`/`Nb10` for narrowband.
    pub fn tune_channel(&self, ch: u8) -> Result<(), FaceError> {
        let is_2g = ch <= 14;
        let mut rf18 = self.rf_get(0, 0x18, RFREG_MASK)?;
        let mut rf19 = self.rf_get(0, 0x19, RFREG_MASK)?;
        if is_2g {
            // 2.4 GHz: clear band bits (17,16,9,8) + channel byte, set channel.
            rf18 = (rf18 & !((1 << 17) | (1 << 16) | (1 << 9) | (1 << 8) | 0xFF)) | (ch as u32);
        } else {
            // 5 GHz: band markers BIT16|BIT8, channel in the low byte.
            rf18 = (rf18 & !((1 << 17) | (1 << 9) | 0xFF)) | (1 << 16) | (1 << 8) | (ch as u32);
            // 5G sub-band select: >144 → BIT19, 5400<f≤5720 (ch>80) → BIT18.
            rf19 &= !((1 << 19) | (1 << 18));
            if ch > 144 {
                rf19 |= 1 << 19;
            } else if ch > 80 {
                rf19 |= 1 << 18;
            }
        }
        for _ in 0..20 {
            self.rf_set(0, 0x18, RFREG_MASK, rf18)?;
            self.rf_set(1, 0x18, RFREG_MASK, rf18)?;
            std::thread::sleep(Duration::from_micros(250));
            if self.rf_get(0, 0xc5, 0x8000)? != 0 {
                break; // channel-setting ready
            }
        }
        self.rf_set(0, 0x19, RFREG_MASK, rf19)?;
        self.rf_set(1, 0x19, RFREG_MASK, rf19)?;
        self.bb_set(0x1ea8, 1 << 7, is_2g as u32)?; // RX idle AGC table: 1=2.4G, 0=5G

        // BB channel/bandwidth config (config_phydm_switch_channel_bw_8733b, 20 MHz
        // common part) — the digital front-end setup the TX/RX path needs, which the
        // RF writes alone don't do. Confirmed against the vendor's live procfs dump.
        self.bb_set(0x0818, 1 << 11, 0)?;
        self.bb_set(0x1940, 1 << 31, 0)?;
        self.bb_set(0x1ce8, 1 << 28, 0)?;
        self.bb_set(0x0db4, 1 << 0, 0)?;
        self.bb_set(0x0c10, 1 << 9, 0)?;
        self.bb_set(0x0c24, 0x0000_00FF, 0xFF)?;
        self.bb_set(0x0c24, 0x0000_FF00, 0x00)?;
        self.bb_set(0x0884, 0x0001_C000, 0x4)?;
        self.bb_set(0x1900, 0x0000_000F, 6)?; // BW mode 20
        self.bb_set(0x1908, 0x0000_00F0, 9)?;
        Ok(())
    }

    /// NCTL one-shot: if `0x2d9c[7:0]` is 0, trigger `0x1b00` BIT0 and poll
    /// `0x2d9c[7:0]` for `0x55` (measurement done), up to 10 × `step_ms`.
    fn nctl_one_shot(&self, step_ms: u64) -> Result<bool, FaceError> {
        if self.bb_get(0x2d9c, 0xFF)? == 0 {
            self.bb_set(0x1b00, 1 << 0, 1)?;
            for _ in 0..10 {
                std::thread::sleep(Duration::from_millis(step_ms));
                if self.bb_get(0x2d9c, 0xFF)? == 0x55 {
                    return Ok(true);
                }
            }
        }
        if std::env::var("IQKDBG").is_ok() {
            eprintln!("  [nctl] not converged (0x2d9c=0x{:02x})", self.bb_get(0x2d9c, 0xFF)?);
        }
        Ok(false)
    }

    /// TXK RF setup (`_iqk_txk_rf_setting_8733b`). `band`: 0=G, 1=A.
    fn iqk_txk_rf_setting(&self, path: u8, band: u8) -> Result<(), FaceError> {
        self.rf_set(0, 0xde, 0xFE000, 0x3f)?;
        self.rf_set(1, 0xde, 0xFE000, 0x3f)?;
        if path == 0 {
            self.rf_set(0, 0x60, 0x00007, 0x7)?;
            if band == 0 {
                self.rf_set(0, 0x51, 1 << 19, 0)?;
                self.rf_set(0, 0x51, 1 << 11, 0)?;
                self.rf_set(0, 0x52, 1 << 11, 0)?;
            }
        } else {
            self.rf_set(0, 0x51, 1 << 19, 0)?;
            self.rf_set(0, 0x51, 1 << 11, 0)?;
            self.rf_set(0, 0x52, 1 << 11, 0)?;
            self.rf_set(1, 0x51, 1 << 19, 0)?;
            self.rf_set(1, 0x51, 1 << 11, 0)?;
            self.rf_set(1, 0x52, 1 << 11, 0)?;
        }
        self.rf_set(0, 0x55, 1 << 0, 0)?;
        self.rf_set(1, 0x55, 1 << 0, 0)?;
        self.rf_set(0, 0xef, 1 << 2, 1)?;
        self.rf_set(1, 0xef, 1 << 2, 1)?;
        self.rf_set(0, 0xdf, 1 << 2, 0)?;
        self.rf_set(1, 0xdf, 1 << 2, 0)?;
        if band == 0 {
            self.rf_set(0, 0x33, 0x003FF, 0x000)?;
            self.rf_set(1, 0x33, 0x003FF, 0x000)?;
        } else {
            self.rf_set(0, 0x33, 0x003FF, 0x100)?;
        }
        self.rf_set(0, 0x00, 0xFFFF0, 0x403E)?;
        self.rf_set(1, 0x00, 0xFFFF0, 0x403E)?;
        self.rf_set(0, 0x56, 0x0FFFF, 0xe0e4)?;
        if path == 1 {
            self.rf_set(1, 0x56, 0x0FFFF, 0xe0e4)?;
        }
        let tx_pi = self.rf_get(path, 0x00, 0xFFFFF)?;
        self.bb_set(0x1b20, 0x000F_FFFF, tx_pi)?;
        self.bb_set(0x1b20, 0x0F00_0000, 0)?;
        self.bb_set(0x1bbc, 0x3000_0000, 0)?;
        self.bb_set(0x1b1c, 0x0001_C000, 0)?; // TX_P_Avg (vendor value; averaging didn't lift KFAIL)
        self.bb_set(0x1bb8, 1 << 20, 0)?;
        Ok(())
    }

    /// RXK RF setup (`_iqk_rxk_rf_setting_8733b`) — IQKPLL on.
    fn iqk_rxk_rf_setting(&self, path: u8, band: u8) -> Result<(), FaceError> {
        self.bb_set(0x1860, 1 << 30, 0)?; // DAC off
        if path == 0 {
            self.rf_set(0, 0x00, 0xF0000, 0x7)?;
            self.rf_set(1, 0x00, 0xF0000, 0x3)?;
        } else {
            self.rf_set(0, 0x00, 0xF0000, 0x3)?;
            self.rf_set(1, 0x00, 0xF0000, 0x7)?;
            self.rf_set(1, 0x88, 0x0000F, 0x3)?;
        }
        if band == 0 {
            self.rf_set(path, 0x20, 1 << 8, 1)?;
        } else {
            self.rf_set(path, 0x20, 1 << 7, 1)?;
        }
        let r1f = self.rf_get(path, 0x18, 0xFFFFF)?;
        self.rf_set(path, 0x1f, 0xFFFFF, r1f)?;
        self.rf_set(path, 0x1e, 0x0003F, 0x13)?;
        self.rf_set(path, 0x1e, 1 << 19, 0)?;
        self.rf_set(path, 0x1e, 1 << 19, 1)?;
        std::thread::sleep(Duration::from_millis(1));
        Ok(())
    }

    /// LOK (LO-leakage cal, coarse + fine) — the active NCTL path.
    fn iqk_lok_by_path(&self, path: u8) -> Result<(), FaceError> {
        self.rf_set(0, 0xf5, 1 << 17, 1)?; // clock gating on
        if path == 1 {
            self.rf_set(1, 0xf5, 1 << 17, 1)?;
        }
        // ---- coarse ----
        self.bb_set(0x1b10, 0xFF, 0)?; // reset 0x2d9c
        if path == 0 {
            self.bb_set(0x1b00, 0xFFFF_0000, 0x3c00)?;
            self.bb_set(0x1880, 1 << 21, 1)?;
            self.bb_set(0x1bcc, 0x3F, 0x9)?;
            self.bb_set(0x1b2c, 0xFFFF_FFFF, 0x0024_0024)?;
            self.bb_set(0x1b00, 0x0000_1FFF, 0x018)?;
        } else {
            self.bb_set(0x1b00, 0xFFFF_0000, 0x4c00)?;
            self.bb_set(0x1880, 1 << 21, 1)?;
            self.bb_set(0x1bcc, 0x3F, 0x9)?;
            self.bb_set(0x1b2c, 0x0000_0FFF, 0x024)?;
            self.bb_set(0x1b00, 0x0000_1FFF, 0x028)?;
        }
        self.nctl_one_shot(1)?;
        self.bb_set(0x1880, 1 << 21, 0)?;
        self.bb_set(0x1bd4, 0xFFFF_FFFF, 0x002c_0001)?;
        let r = self.read32(0x1bfc)?;
        let idac_ic = ((r >> 25) & 0x1F) + u32::from(r & (1 << 24) != 0);
        let idac_qc = ((r >> 5) & 0x1F) + u32::from(r & (1 << 4) != 0);
        self.rf_set(path, 0x08, 0xF8000, idac_ic)?;
        self.rf_set(path, 0x08, 0x003E0, idac_qc)?;
        // ---- fine ----
        self.bb_set(0x1b10, 0xFF, 0)?;
        self.bb_set(0x1880, 1 << 21, 1)?;
        self.bb_set(0x1bcc, 0x3F, 0x9)?;
        self.bb_set(0x1b2c, 0xFFFF_FFFF, 0x0024_0024)?;
        self.bb_set(0x1b00, 0x0000_1FFF, if path == 0 { 0x118 } else { 0x128 })?;
        self.nctl_one_shot(1)?;
        self.bb_set(0x1880, 1 << 21, 0)?;
        self.bb_set(0x1bd4, 0xFFFF_FFFF, 0x002c_0001)?;
        std::thread::sleep(Duration::from_millis(5));
        let r = self.read32(0x1bfc)?;
        let idac_if = ((r >> 26) & 0xF) + u32::from(r & (1 << 25) != 0);
        let idac_qf = ((r >> 6) & 0xF) + u32::from(r & (1 << 5) != 0);
        if std::env::var("IQKDBG").is_ok() {
            eprintln!("  [lok] idac ic={idac_ic:#x} qc={idac_qc:#x} if={idac_if:#x} qf={idac_qf:#x}");
        }
        self.rf_set(path, 0x09, 0xF0000, idac_if)?;
        self.rf_set(path, 0x09, 0x003C0, idac_qf)?;
        self.rf_set(path, 0x08, 0xF8000, idac_ic)?; // re-apply coarse
        self.rf_set(path, 0x08, 0x003E0, idac_qc)?;
        self.rf_set(0, 0xf5, 1 << 17, 0)?; // clock gating off
        if path == 1 {
            self.rf_set(1, 0xf5, 1 << 17, 0)?;
        }
        Ok(())
    }

    /// TX IQK — one-shot, store `txxy[path]` on success (`0x1b08` BIT26 == 0).
    fn iqk_txk_by_path(&self, info: &mut IqkInfo, path: u8) -> Result<(), FaceError> {
        let p = path as usize;
        self.bb_set(0x1b10, 0xFF, 0)?;
        self.bb_set(0x1bcc, 0x3F, 0x09)?;
        self.bb_set(0x1b2c, 0xFFFF_FFFF, 0x0024_0024)?;
        self.bb_set(0x1b00, 0x0000_1FFF, if path == 0 { 0x218 } else { 0x228 })?;
        let conv = self.nctl_one_shot(1)?;
        let kfail = self.bb_get(0x1b08, 1 << 26)?;
        if std::env::var("IQKDBG").is_ok() {
            eprintln!(
                "  [txk] conv={conv} KFAIL={kfail} 0x1b38=0x{:08x} is_tssi(0x1e7c[30])={} rf0x56=0x{:05x}",
                self.read32(0x1b38)?,
                self.bb_get(0x1e7c, 1 << 30)?,
                self.rf_get(0, 0x56, 0xFFFFF)?
            );
        }
        if conv && kfail == 0 {
            info.txxy[p][0] = self.bb_get(0x1b38, 0x7FF0_0000)?;
            info.txxy[p][1] = self.bb_get(0x1b38, 0x0007_FF00)?;
        }
        Ok(())
    }

    /// RX IQK — LNA-small then LNA-large one-shots, store `rxxy[path]`.
    fn iqk_rxk_by_path(&self, info: &mut IqkInfo, path: u8) -> Result<(), FaceError> {
        let (p, band) = (path as usize, info.band);
        let reg1b00 = if path == 0 { 0x418 } else { 0x428 };
        for lna in 0..2usize {
            // -- RF LNA gain (small=0 / large=1) --
            if band == 0 {
                let (rf00, rf83) = if lna == 0 { (0x1cc, 0x79) } else { (0x342, 0x7e) };
                self.rf_set(0, 0x00, 0x03FF0, rf00)?;
                self.rf_set(1, 0x00, 0x03FF0, rf00)?;
                self.rf_set(path, 0x83, 0x00300, 0x2)?;
                self.rf_set(path, 0x83, 0x1FC00, rf83)?;
            } else {
                self.rf_set(0, 0x00, 0x03FF0, if lna == 0 { 0x1c8 } else { 0x348 })?;
                self.rf_set(path, 0x8c, 0x00180, 0x1)?;
                self.rf_set(path, 0x8c, 0x0007F, 0x07)?;
            }
            if lna == 0 {
                let rx_pi = self.rf_get(path, 0x00, 0xFFFFF)?;
                self.bb_set(0x1b24, 0x000F_FFFF, rx_pi)?;
            }
            self.bb_set(0x1b10, 0xFF, 0)?;
            self.bb_set(0x1b34, 0x0000_007C, if lna == 0 { 0x07 } else { 0x0D })?;
            self.bb_set(0x1bb8, 1 << 20, 1)?;
            self.bb_set(0x1bcc, 0x3F, 0x3f)?;
            self.bb_set(0x1b2c, 0x0FFF_0000, 0x044)?;
            self.bb_set(0x1b00, 0x0000_1FFF, reg1b00)?;
            if self.nctl_one_shot(2)? && self.bb_get(0x1b08, 1 << 26)? == 0 {
                let r = self.read32(0x1b3c)?;
                info.rxxy[p][0][lna] = (r >> 20) & 0x7FF;
                info.rxxy[p][1][lna] = (r >> 8) & 0x7FF;
                info.rxk_ok = true;
            }
        }
        // disable RXIQKPLL
        self.rf_set(path, 0x20, if band == 0 { 1 << 8 } else { 1 << 7 }, 0)?;
        self.rf_set(path, 0x1e, 1 << 19, 0)?;
        self.bb_set(0x1860, 1 << 30, 1)?; // DAC on
        Ok(())
    }

    /// Apply the measured coefficients to the correction registers
    /// (`_iqk_fill_iqk_xy_8733b`).
    fn iqk_fill_iqk_xy(&self, info: &IqkInfo, path: u8) -> Result<(), FaceError> {
        let p = path as usize;
        self.bb_set(0x1b38, 0x3, if path == 0 { 0x1 } else { 0x3 })?;
        self.bb_set(0x1b38, 0x7FF0_0000, info.txxy[p][0])?;
        self.bb_set(0x1b38, 0x0007_FF00, info.txxy[p][1])?;
        self.bb_set(0x1b34, 0x0000_007C, 0x07)?; // LNA small
        self.bb_set(0x1b3c, 0x7FF0_0000, info.rxxy[p][0][0])?;
        self.bb_set(0x1b3c, 0x0007_FF00, info.rxxy[p][1][0])?;
        self.bb_set(0x1b34, 0x0000_007C, 0x0D)?; // LNA large
        self.bb_set(0x1b3c, 0x7FF0_0000, info.rxxy[p][0][1])?;
        self.bb_set(0x1b3c, 0x0007_FF00, info.rxxy[p][1][1])?;
        self.bb_set(0x1b38, 0x3, 0x0)?;
        Ok(())
    }

    /// **M7**: full IQK calibration. Backup → AFE-on → preset → per-path
    /// (LOK → TXK → RXK) → apply coefficients → preset-restore → AFE-off →
    /// restore. 1x1 path A, 2.4 GHz. Returns the measured [`IqkInfo`].
    pub fn phy_iq_calibrate(&self) -> Result<IqkInfo, FaceError> {
        let mut info = IqkInfo {
            band: 0,
            ..Default::default()
        };
        // Identity defaults (_iq_calibrate_8733b_init): x=0x200, y=0 — applied if a K
        // step fails so the correction is a no-op rather than a zeroing.
        for p in 0..2 {
            info.txxy[p] = [0x200, 0x000];
            info.rxxy[p][0] = [0x200, 0x200]; // x (LNA small, large)
            info.rxxy[p][1] = [0x000, 0x000]; // y (LNA small, large)
        }
        let path = 0u8;
        let backup = self.iqk_backup()?;
        self.iqk_afe_setting(true)?;
        // _iqk_start_iqk: backup RF mode → preset(true) → LOK/TXK/RXK → preset(false).
        let pa_mode = self.rf_get(0, 0x0, 0xF0000)?;
        self.iqk_preset(true)?;
        self.iqk_txk_rf_setting(path, info.band)?;
        self.iqk_lok_by_path(path)?;
        self.iqk_txk_by_path(&mut info, path)?;
        self.iqk_rxk_rf_setting(path, info.band)?;
        self.iqk_rxk_by_path(&mut info, path)?;
        self.iqk_preset(false)?;
        self.rf_set(0, 0x0, 0xF0000, pa_mode)?;
        self.iqk_afe_setting(false)?;
        self.iqk_fill_iqk_xy(&info, path)?;
        self.iqk_restore(&backup)?;
        // Enable the RX-IQK correction if RXK succeeded (`_iqk_restore_mac_bb`).
        self.bb_set(0x180c, 1 << 31, u32::from(!info.rxk_ok))?;
        Ok(info)
    }

    // ── M7 DPK (digital pre-distortion) — part 1: infrastructure + one-shot ──

    fn dpk_backup(&self) -> Result<(Vec<u32>, Vec<u32>), FaceError> {
        let mut bb = Vec::new();
        for &r in &DPK_BB_REGS {
            bb.push(self.read32(r)?);
        }
        let mut rf = Vec::new();
        for &r in &DPK_RF_REGS {
            rf.push(self.rf_get(0, r, RFREG_MASK)?);
        }
        Ok((bb, rf))
    }
    fn dpk_restore(&self, bb: &[u32], rf: &[u32]) -> Result<(), FaceError> {
        for (i, &r) in DPK_BB_REGS.iter().enumerate() {
            self.write32(r, bb[i])?;
        }
        for (i, &r) in DPK_RF_REGS.iter().enumerate() {
            self.rf_set(0, r, RFREG_MASK, rf[i])?;
        }
        Ok(())
    }

    /// DPK NCTL one-shot (`_dpk_one_shot_8733b`): write the `shot_code` (path + action
    /// bits) to `0x1b00`, trigger with `+1`, then wait for the timing-sync report.
    /// Returns the fail report (0 = ok); `DPK_ON` has no report.
    fn dpk_one_shot(&self, path: u8, action: u8) -> Result<u8, FaceError> {
        let mut shot: u16 = if path == 0 { 0x0018 } else { 0x002a };
        match action {
            DPK_GAIN_LOSS => {
                self.write8(0x1bef, if path == 0 { 0xa2 } else { 0xaa })?;
                shot |= 0x1100;
            }
            DPK_PAS => {
                self.write8(0x1bef, 0x2a)?;
                shot |= 0x1100;
            }
            DO_DPK => shot |= 0x1300,
            DPK_ON => shot |= 0x1400,
            _ => {}
        }
        self.write16(0x1b00, shot)?;
        self.write16(0x1b00, shot + 1)?; // one-shot trigger
        std::thread::sleep(Duration::from_micros(100));
        if action == DPK_ON {
            Ok(0)
        } else {
            self.dpk_timing_sync()
        }
    }

    /// DPK timing-sync (`_dpk_timing_sync_report_8733b`): poll `0x2d9c==0x55` (done),
    /// then wait for KFAIL (`0x1b08` BIT26) to clear; reset `0x1b10`. Returns fail.
    fn dpk_timing_sync(&self) -> Result<u8, FaceError> {
        let mut count = 0;
        while self.bb_get(0x2d9c, 0xFF)? != 0x55 && count < 1000 {
            std::thread::sleep(Duration::from_micros(20));
            count += 1;
        }
        let mut fail = (self.bb_get(0x1b08, 1 << 26)? & 1) as u8;
        if count < 1000 {
            let mut c2 = 0;
            while fail != 0 && c2 < 100 {
                std::thread::sleep(Duration::from_micros(10));
                fail = (self.bb_get(0x1b08, 1 << 26)? & 1) as u8;
                c2 += 1;
            }
        }
        self.write8(0x1b10, 0)?;
        Ok(fail)
    }

    /// DPK RF setup for 2.4 GHz path A (`_dpk_rf_setting_8733b`, G-band), using the
    /// current RF TXAGC. Returns the applied TXAGC.
    fn dpk_rf_setting(&self, path: u8) -> Result<u8, FaceError> {
        // NOTE: the proper TX AGC comes from _dpk_get_tssi_mode_txagc via a live HW-TX
        // TSSI measurement (needs the TX data path, M8). Here we use the current RF 0x1;
        // the hardware keeps it at a low managed value without HW-TX, so the AGC search
        // starves — this is the TX-path dependency on M8.
        let txagc = self.rf_get(path, 0x1, RFREG_MASK)? & 0x1f;
        self.rf_set(0, 0x5, 1 << 0, 0)?;
        self.rf_set(1, 0x5, 1 << 0, 0)?;
        self.rf_set(0, 0x00, RFREG_MASK, 0x50000)?;
        self.rf_set(1, 0x00, RFREG_MASK, 0x50000)?;
        // 2G path A gain chain.
        self.rf_set(path, 0x1, 0xff, (txagc + 5) & 0xff)?;
        self.rf_set(0, 0x83, 0x00007, 0x2)?;
        self.rf_set(0, 0xdf, 1 << 12, 0x1)?;
        self.rf_set(0, 0x9e, 1 << 8, 0x1)?;
        self.rf_set(path, 0x83, 0x000f0, 0x3)?;
        self.rf_set(0, 0x8f, 1 << 1, 0x0)?;
        self.rf_set(0, 0x8f, 0x0e000, 0x3)?;
        Ok((self.rf_get(path, 0x1, 0x1f)?) as u8)
    }

    /// Read a DPK debug report (`_dpk_dbg_report_read_8733b`): select `index` into
    /// `0x1bd6`, read the result from `0x1bfc`.
    fn dpk_dbg_report(&self, index: u8) -> Result<u32, FaceError> {
        self.write8(0x1bd6, index)?;
        self.read32(0x1bfc)
    }

    /// Gain-loss / back-off read (`_dpk_gainloss_result_8733b`).
    fn dpk_gainloss_result(&self, item: u8) -> Result<u32, FaceError> {
        match item {
            0 => {
                // GL_BACK_VALUE
                self.bb_set(0x1bcc, 1 << 26, 1)?;
                self.bb_set(0x1b90, 0xFFFF_FFFF, 0x0105_e038)?;
                self.dpk_dbg_report(0x06)
            }
            1 => {
                // LOSS_CHK
                self.bb_set(0x1bcc, 1 << 26, 0)?;
                self.bb_set(0x1b90, 0xFFFF_FFFF, 0x0105_e038)?;
                self.dpk_dbg_report(0x06)
            }
            _ => {
                // GAIN_CHK
                self.bb_set(0x1bcc, 1 << 26, 0)?;
                self.bb_set(0x1b90, 0xFFFF_FFFF, 0x0105_e03f)?;
                self.dpk_dbg_report(0x09)
            }
        }
    }

    /// One AGC-tune step (`_dpk_agc_tune_8733b`): read loss + back-off, adjust the RF
    /// TXAGC toward back-off 0xA. Returns the next auto-AGC state (1/2/3/4).
    fn dpk_agc_tune(&self, path: u8, ori_agc: u8) -> Result<u8, FaceError> {
        let loss = self.dpk_gainloss_result(1)?;
        if loss > 0x3FF_0000 {
            if std::env::var("IQKDBG").is_ok() {
                eprintln!("  [agc] loss=0x{loss:08x} OVERFLOW → fail");
            }
            return Ok(4); // gain-loss overflow
        }
        let backoff = self.dpk_gainloss_result(0)? as u8;
        if std::env::var("IQKDBG").is_ok() {
            eprintln!("  [agc] ori_agc=0x{ori_agc:x} loss=0x{loss:08x} backoff=0x{backoff:x}");
        }
        if backoff < 0x5 {
            Ok(1)
        } else if backoff == 0xA {
            Ok(2)
        } else if backoff > 0x4 && backoff < 0xA {
            let new_agc = ori_agc.wrapping_sub(0xA - backoff);
            self.rf_set(path, 0x1, 0x0001F, new_agc as u32)?;
            std::thread::sleep(Duration::from_micros(10));
            Ok(3)
        } else {
            Ok(4)
        }
    }

    /// Auto-AGC loop (`_dpk_gainloss_auto_agc_8733b`): drive GAIN_LOSS/PAS one-shots
    /// and step the RF TXAGC / PGA until the back-off lands in range. Returns
    /// `agc_done` (1 = converged).
    fn dpk_auto_agc(&self, path: u8, _ori_agc: u8) -> Result<u8, FaceError> {
        let mut tmp_txagc = 0u8;
        let mut auto_pga = 0u8;
        let (mut i, mut agc_cnt, mut agc_done) = (0u8, 0u8, 0u8);
        loop {
            let mut goout = false;
            match i {
                0 => {
                    tmp_txagc = (self.rf_get(path, 0x1, 0x0001f)?) as u8;
                    self.dpk_one_shot(path, DPK_GAIN_LOSS)?;
                    auto_pga = ((self.dpk_dbg_report(0x02)? >> 16) & 0x7) as u8;
                    self.rf_set(0, 0x8f, 0x0e000, auto_pga as u32)?;
                    std::thread::sleep(Duration::from_micros(10));
                    self.dpk_one_shot(path, DPK_PAS)?;
                    i = self.dpk_agc_tune(path, tmp_txagc)?;
                    agc_cnt += 1;
                }
                1 => {
                    if tmp_txagc < 0x5 {
                        goout = true;
                    } else {
                        tmp_txagc -= 2;
                        self.rf_set(path, 0x1, 0x0001f, tmp_txagc as u32)?;
                        i = 0;
                        agc_cnt += 1;
                    }
                }
                2 => {
                    if tmp_txagc == 0x1f {
                        goout = true;
                    } else {
                        tmp_txagc += if tmp_txagc > 0x1c {
                            1
                        } else if tmp_txagc < 0x15 {
                            3
                        } else {
                            2
                        };
                        self.rf_set(path, 0x1, 0x0001f, tmp_txagc as u32)?;
                        i = 0;
                        agc_cnt += 1;
                    }
                }
                3 => {
                    agc_done = 1;
                    auto_pga += ((0xA - self.dpk_gainloss_result(0)? as u8) / 3).min(0x6);
                    if auto_pga > 0x6 {
                        auto_pga = 0x6;
                    }
                    self.rf_set(0, 0x8f, 0x0e000, auto_pga as u32)?;
                    goout = true;
                }
                4 => {
                    if auto_pga > 0 {
                        self.rf_set(0, 0x8f, 0x0e000, 0)?;
                        i = 0;
                        agc_cnt += 2;
                    } else {
                        goout = true;
                    }
                }
                _ => goout = true,
            }
            if goout || agc_cnt >= 6 {
                break;
            }
        }
        Ok(agc_done)
    }

    /// DPK gain-loss setup + auto-AGC (`_dpk_gainloss_8733b`): RF setup, TPG BW select,
    /// RXIQC default, then run the auto-AGC to condition the TX level.
    fn dpk_gainloss(&self, path: u8) -> Result<u8, FaceError> {
        let ori_txagc = self.dpk_rf_setting(path)?;
        self.write8(0x1b00, 0x08)?;
        self.write8(0x1bd8, 0x00)?;
        // TPG BW select (test-pattern generator) — the DPK measurement signal.
        let dpk_bw = (self.rf_get(path, 0x18, RFREG_MASK)? >> 10) & 1;
        let tpg = if dpk_bw == 1 { 0xd200_0065 } else { 0xd200_0068 };
        self.bb_set(0x1bf8, 0xFFFF_FFFF, tpg)?;
        self.bb_set(0x1b3c, 0xFFFF_FF00, 0x200000)?; // RXIQC default
        self.bb_set(0x1b88, 0xFFFF_FFFF, 0x00b4_8000)?;
        self.dpk_auto_agc(path, ori_txagc)
    }

    /// DPK per-path K (`_dpk_one_path_8733b`, non-TSSI): set PWSF, run PAS then DO_DPK,
    /// verify via the gain-loss check. Returns the DPK fail report (0 = ok).
    fn dpk_one_path(&self, path: u8) -> Result<u8, FaceError> {
        let tx_agc = (self.rf_get(path, 0x1, RFREG_MASK)? & 0x1f) as i32;
        self.bb_set(0x1bb8, 1 << 4, 0)?; // disable TSSI mode
        let pwsf = ((((0x19 - tx_agc) << 3) + 0x50) & 0x1ff) as u32;
        self.bb_set(0x1bd8, 0x001f_f000, pwsf)?; // path A
        self.bb_set(0x1bec, 0x00e0_0000, 0x6)?; // LUT point 6
        let mut result = 0u8;
        if self.dpk_one_shot(path, DPK_PAS)? == 0 {
            result = self.dpk_one_shot(path, DO_DPK)?;
        }
        if self.dpk_gainloss_result(1)? != 0x400_0000 {
            result = 1;
        }
        Ok(result)
    }

    /// **M7 DPK**: full per-path calibration — backup → shared BB setup → gain-loss +
    /// auto-AGC (conditions the TX level) → per-path DO_DPK → restore. 1x1 path A.
    /// Returns `(agc_done, dpk_fail)` (agc_done 1 = AGC converged; dpk_fail 0 = ok).
    pub fn phy_dpk(&self) -> Result<(u8, u8), FaceError> {
        let path = 0u8;
        let (bb, rf) = self.dpk_backup()?;
        self.iqk_afe_setting(true)?; // ≈ _dpk_mac_bb_setting_8733b
        self.bb_set(0x1884, 1 << 20, path as u32)?; // one-path K
        let agc_done = self.dpk_gainloss(path)?;
        let dpk_fail = self.dpk_one_path(path)?;
        self.iqk_afe_setting(false)?;
        self.dpk_restore(&bb, &rf)?;
        Ok((agc_done, dpk_fail))
    }

    /// NCTL diagnostic: set up a LOK-style one-shot and watch the mirror, the
    /// trigger self-clear, `0x2d9c`, and the WLAN-CPU PC (`FW_DBG7`) — to tell whether
    /// the read path, the trigger, or the KIP-kernel execution is the problem.
    pub fn nctl_debug(&self) -> Result<(), FaceError> {
        let backup = self.iqk_backup()?;
        self.iqk_afe_setting(true)?;
        self.iqk_preset(true)?;
        // (a) mirror check: 0x1b10[7:0] should route to 0x2d9c[7:0].
        self.write32(0x1b10, (self.read32(0x1b10)? & !0xFF) | 0x55)?;
        eprintln!(
            "[nctl] mirror: 0x1b10[7:0]=0x55 → 0x2d9c[7:0]=0x{:02x}",
            self.read32(0x2d9c)? & 0xFF
        );
        // (b) LOK-style one-shot setup (path A).
        self.iqk_txk_rf_setting(0, 0)?;
        self.rf_set(0, 0xf5, 1 << 17, 1)?;
        self.bb_set(0x1b10, 0xFF, 0)?;
        self.bb_set(0x1b00, 0xFFFF_0000, 0x3c00)?;
        self.bb_set(0x1880, 1 << 21, 1)?;
        self.bb_set(0x1bcc, 0x3F, 0x9)?;
        self.bb_set(0x1b2c, 0xFFFF_FFFF, 0x0024_0024)?;
        self.bb_set(0x1b00, 0x0000_1FFF, 0x018)?;
        eprintln!(
            "[nctl] pre-trigger 0x1b00=0x{:08x} FW_DBG7=0x{:08x} 0x2d9c=0x{:02x}",
            self.read32(0x1b00)?,
            self.read32(0x10AC)?,
            self.read32(0x2d9c)? & 0xFF
        );
        self.bb_set(0x1b00, 1 << 0, 1)?; // one-shot trigger
        for i in 0..8 {
            std::thread::sleep(Duration::from_millis(1));
            eprintln!(
                "[nctl] t={i}ms 0x2d9c=0x{:02x} 0x1b00[0]={} FW_DBG7=0x{:08x}",
                self.read32(0x2d9c)? & 0xFF,
                self.read32(0x1b00)? & 1,
                self.read32(0x10AC)?
            );
        }
        self.iqk_preset(false)?;
        self.iqk_afe_setting(false)?;
        self.iqk_restore(&backup)?;
        Ok(())
    }

    /// **M7 (part 1) self-test**: exercise the IQK setup/teardown scaffold —
    /// backup → AFE-on → preset-on → preset-off → AFE-off → restore — and confirm
    /// the backed-up registers round-trip. The LOK/TXK/RXK measurement core lands
    /// in later parts; this validates the accessors + backup/restore on hardware.
    pub fn iqk_setup_selftest(&self) -> Result<(u32, u32), FaceError> {
        let probe = self.read32(IQK_BB_REGS[0])?;
        let backup = self.iqk_backup()?;
        self.iqk_afe_setting(true)?;
        self.iqk_preset(true)?;
        self.iqk_preset(false)?;
        self.iqk_afe_setting(false)?;
        self.iqk_restore(&backup)?;
        Ok((probe, self.read32(IQK_BB_REGS[0])?))
    }
}

// ── M8 async FrameIo backend + the full radio-knob surface ───────────────────

impl Rtl8733buBackend {
    /// Set the fixed TX HwRate for injection (0x00=1M CCK … 0x03=11M, 0x04=6M OFDM …
    /// 0x0b=54M, 0x0c..0x13 = HT MCS0-7). Default 6 Mbps OFDM.
    pub fn set_tx_rate(&self, hw_rate: u8) {
        self.tx_rate.store(hw_rate, Ordering::Relaxed);
    }

    /// Set the TX PHY flags applied to every injected frame's descriptor: `ldpc`
    /// (LDPC coding), `stbc` (space-time block coding), `sgi` (short guard interval),
    /// and `bw40` (mark the frame 40 MHz — pair with [`set_bandwidth`] `Bw40`). These
    /// only affect HT (MCS) rates; a legacy CCK/OFDM frame ignores them.
    pub fn set_tx_flags(&self, ldpc: bool, stbc: bool, sgi: bool, bw40: bool) {
        let f = (ldpc as u8) | ((sgi as u8) << 1) | ((stbc as u8) << 2) | ((bw40 as u8) << 3);
        self.tx_flags.store(f, Ordering::Relaxed);
    }

    /// Set bandwidth / narrowband. 5 & 10 MHz narrowband (the vendor `narrowband`
    /// knob) are `0x9b0[7:6] = 1/2` on top of a 20 MHz channel; 20 vs 40 MHz is the
    /// BB path-width mode (`0x1900[3:0]` = 6/7 + the 40 MHz enables).
    pub fn set_bandwidth(&self, bw: Bandwidth) -> Result<(), FaceError> {
        let nb = match bw {
            Bandwidth::Nb5 => 1,
            Bandwidth::Nb10 => 2,
            _ => 0,
        };
        self.bb_set(0x09b0, 0xC0, nb)?;
        match bw {
            Bandwidth::Bw40 | Bandwidth::Bw80 => {
                self.bb_set(0x1900, 0xF, 7)?; // BW mode 40
                self.bb_set(0x0c10, 1 << 9, 1)?;
                self.bb_set(0x0db4, 1 << 0, 1)?;
                self.bb_set(0x0818, 1 << 11, 1)?;
                self.bb_set(0x1940, 1 << 31, 1)?;
            }
            _ => {
                self.bb_set(0x1900, 0xF, 6)?; // BW mode 20 (narrowband rides on top)
            }
        }
        Ok(())
    }

    /// Set the reference TX-power index (0..0x7f) for path A OFDM + CCK
    /// (`config_phydm_write_txagc_ref`, reg 0x4308). Effective because this userspace
    /// bring-up runs with tx-power-by-rate / power-limit disabled.
    pub fn set_tx_power_idx(&self, idx: u8) -> Result<(), FaceError> {
        let p = (idx & 0x7f) as u32;
        self.bb_set(0x4308, 0x0000_007f, p)?; // OFDM path A
        self.bb_set(0x4308, 0x0000_7f00, p)?; // CCK path A
        Ok(())
    }

    /// Write the **per-rate TX AGC table** (`phydm_write_txagc_1byte`, base `0x3a00`)
    /// with a uniform gain index for CCK (`0x00–0x03`), OFDM (`0x04–0x0b`), and HT
    /// MCS0–7 (`0x0c–0x13`). This is the gain the PHY applies to the RF during
    /// transmit — it starts at 0 without a tx-power apply, so nothing radiates until
    /// it is set. `idx` is a raw AGC index (≈ 0.5 dB/step); try `0x2d`–`0x3f`.
    pub fn set_txagc_table(&self, idx: u8) -> Result<(), FaceError> {
        for rate in 0u16..=0x13 {
            let reg = 0x3a00 + (rate & 0xfc);
            let mask = 0xFFu32 << ((rate & 0x3) * 8);
            self.bb_set(reg, mask, idx as u32)?;
        }
        Ok(())
    }

    /// BB reset (`phydm_bb_reset_8733b`): toggle `SYS_FUNC_EN[0]` (FEN_BBRSTB, bit 16
    /// of the dword at MAC `0x0`) 1→0→1 so BB config changes latch.
    fn bb_reset(&self) -> Result<(), FaceError> {
        for v in [1u32, 0, 1] {
            let cur = self.read32(0x0)?;
            self.write32(0x0, (cur & !(1 << 16)) | (v << 16))?;
        }
        Ok(())
    }

    /// IGI toggle (`phydm_igi_toggle_8733b`): nudge `0x1d70[6:0]` down 2 then back, to
    /// force the RF to (re)enter its mode after the mode-table change.
    fn igi_toggle(&self) -> Result<(), FaceError> {
        let igi = self.bb_get(0x1d70, 0x7f)?;
        if igi > 2 {
            self.bb_set(0x1d70, 0x7f, igi - 2)?;
        }
        self.bb_set(0x1d70, 0x7f, igi)?;
        Ok(())
    }

    /// **Enable the RF TX path for normal operation** (`config_phydm_trx_mode_8733b` +
    /// `phydm_dis_cck_trx_8733b(SET)`, path A 1×1). Programs the RF mode table
    /// (`0x1800`, nibbles `0=shutdown/1=standby/2=TX/3=RX`) to a TX-capable state,
    /// selects path A (`0x1884`), enables BB CCK TX (`0x2a00[1]=0`) + CCK CCA
    /// (`0x2a24[13]=0`), and resets the BB. Without this the RF never enters TX mode on
    /// a MAC transmit, so injected frames don't radiate.
    pub fn enable_tx_path(&self) -> Result<(), FaceError> {
        self.bb_set(0x1800, 0x000F_FFFF, 0x33311)?; // RF mode table (pre)
        self.bb_set(0x1884, 1 << 21, 0)?; // sw-control s0/s1
        self.bb_set(0x1884, 1 << 20, 0)?; // tx = rx = path A
        self.bb_set(0x1800, 0x000F_FFFF, 0x33312)?; // RF mode table (TX-capable)
        self.bb_reset()?;
        self.igi_toggle()?;
        self.bb_set(0x2a24, 1 << 13, 0)?; // enable CCK CCA
        self.bb_set(0x2a00, 1 << 1, 0)?; // enable BB CCK TX
        self.bb_reset()?;
        Ok(())
    }

    /// Route the TX/RX antenna-switch (TRSW) control GPIOs
    /// (`phydm_init_hw_info_by_rfe_type_8733b`). `ext = true` is the external-TRSW
    /// board config (rfe 1/3/4/5, "pin usecase E9") most USB dongles use; without it
    /// the antenna stays on the RX path and TX never keys the air. `false` is the
    /// internal/default (rfe 0) routing. GPIO_MUXCFG/LED_CFG/PAD_CTRL are MAC regs.
    pub fn configure_trsw(&self, ext: bool) -> Result<(), FaceError> {
        if ext {
            let v40 = (self.read32(0x40)? & !0x0f00_0000) | (0x5 << 24);
            self.write32(0x40, v40)?;
            let v4c = (self.read32(0x4c)? & !0x0780_0000) | (0x2 << 23);
            self.write32(0x4c, v4c)?;
            let v64 = (self.read32(0x64)? & !0x3000_0000) | (0x3 << 28);
            self.write32(0x64, v64)?;
        } else {
            self.write32(0x4c, self.read32(0x4c)? & !(1 << 24))?;
            self.write32(0x64, self.read32(0x64)? & !0x3000_0000)?;
        }
        Ok(())
    }

    /// Split a bulk-IN transfer into 802.11 frames and queue the parsed
    /// [`CapturedFrame`]s (used by [`FrameIo::recv_frame`]).
    fn parse_rx_into_pending(&self, data: &[u8]) {
        let mut q = self.rx_pending.lock().unwrap();
        let mut off = 0usize;
        while off + 24 <= data.len() {
            let dw0 = u32::from_le_bytes([data[off], data[off + 1], data[off + 2], data[off + 3]]);
            let pkt_len = (dw0 & 0x3FFF) as usize;
            if pkt_len == 0 {
                break;
            }
            let drvinfo = ((dw0 >> 16) & 0xF) as usize * 8;
            let shift = ((dw0 >> 24) & 0x3) as usize;
            let dw2 =
                u32::from_le_bytes([data[off + 8], data[off + 9], data[off + 10], data[off + 11]]);
            let is_c2h = dw2 & (1 << 28) != 0;
            let fstart = off + 24 + drvinfo + shift;
            if !is_c2h && fstart + pkt_len <= data.len() {
                if let Some(cap) =
                    frame::parse_dot11(self.format, &data[fstart..fstart + pkt_len], None, None, None)
                {
                    q.push_back(cap);
                }
            }
            off += (24 + drvinfo + shift + pkt_len + 7) & !7;
        }
    }
}

/// The 8733b as an async monitor radio: inject NDN-framed 802.11 at the current fixed
/// rate and capture every frame on the channel. Blocking USB I/O runs on the blocking
/// pool so the async reactor is never stalled.
#[async_trait]
impl FrameIo for Rtl8733buBackend {
    async fn inject(&self, frame_in: InjectFrame) -> Result<(), FaceError> {
        let dot11 = frame::build_dot11(self.format, &frame_in)?;
        let rate = self.tx_rate.load(Ordering::Relaxed);
        let seq = self.tx_seq.fetch_add(1, Ordering::Relaxed) & 0xFFF;
        let bcast = dot11.len() > 4 && dot11[4] & 0x01 != 0;
        let desc = build_data_txdesc(dot11.len(), rate, seq, bcast, self.tx_flags.load(Ordering::Relaxed));
        let mut buf = Vec::with_capacity(desc.len() + dot11.len() + 1);
        buf.extend_from_slice(&desc);
        buf.extend_from_slice(&dot11);
        if buf.len() % 512 == 0 {
            buf.push(0);
        }
        let handle = self.handle.clone();
        let ep = self.bulk_out;
        tokio::task::spawn_blocking(move || {
            handle
                .write_bulk(ep, &buf, Duration::from_millis(500))
                .map(|_| ())
                .map_err(usb_err)
        })
        .await
        .map_err(|e| io_err(format!("inject join: {e}")))?
    }

    async fn recv_frame(&self) -> Result<CapturedFrame, FaceError> {
        loop {
            if let Some(f) = self.rx_pending.lock().unwrap().pop_front() {
                return Ok(f);
            }
            let handle = self.handle.clone();
            let ep = self.bulk_in;
            let (buf, n) = tokio::task::spawn_blocking(move || {
                let mut buf = vec![0u8; 16384];
                match handle.read_bulk(ep, &mut buf, Duration::from_millis(200)) {
                    Ok(n) => Ok((buf, n)),
                    Err(rusb::Error::Timeout) => Ok((buf, 0)),
                    Err(e) => Err(usb_err(e)),
                }
            })
            .await
            .map_err(|e| io_err(format!("recv join: {e}")))??;
            if n > 0 {
                self.parse_rx_into_pending(&buf[..n]);
            }
        }
    }
}

#[async_trait]
impl WifiRadio for Rtl8733buBackend {
    async fn inject_at(&self, frame_in: InjectFrame, _mcs: McsDescriptor) -> Result<(), FaceError> {
        // The current fixed rate (set_tx_rate) applies; per-frame rate control is via
        // set_tx_rate before inject.
        self.inject(frame_in).await
    }
}

impl RadioKnobs for Rtl8733buBackend {
    fn set_channel(&self, channel: u8, bw: Bandwidth) -> Result<(), FaceError> {
        self.tune_channel(channel)?;
        self.set_bandwidth(bw)
    }
    fn set_tx_power(&self, idx: u32) -> Result<(), FaceError> {
        self.set_tx_power_idx(idx.min(0x7f) as u8)
    }
}

#[cfg(test)]
mod golden {
    //! Golden-frame tests: pin the bytes the 8731bu/8733bu driver constructs and
    //! its decode of the *real* embedded firmware, so the M1–M4 wire format is
    //! verified without hardware and regressions are caught. These values are the
    //! ground truth to cross-check captured reference-driver USB traffic against
    //! (usbmon on a host with the real device — the vendor `rtl8733bu` driver).
    //!
    //! Cross-checked on real silicon (an RTL8731BU, 0bda:f72b, unbound, on the OPi
    //! via libusb): the chip reads back `chip_id=0x16` (matching the golden decode),
    //! `chip_ver=3`, `sys_cfg1=0x0069333d` — so the VENQT reg-read encoding and
    //! chip-id decode (M1) are correct against the hardware, not just self-consistent.
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
    /// (TXPKTSIZE / OFFSET=40 / QSEL=BEACON) + the TX-desc checksum at 0x1C. The
    /// checksum = ~(XOR of the u16 words): word0 0x0040 ^ word2 0x0028 (OFFSET) ^
    /// word2 0x1000 (QSEL) = 0x1068 → ~ = 0xEF97 (LE bytes 97 EF).
    #[test]
    fn download_txdesc_golden() {
        let d = download_txdesc(64);
        assert_eq!(&d[..8], &[0x40, 0x00, 0x28, 0x00, 0x00, 0x10, 0x00, 0x00], "dw0/dw1");
        assert_eq!(&d[0x1C..0x1E], &[0x97, 0xEF], "TX-desc checksum (~XOR)");
        assert_eq!(u16::from_le_bytes([d[0], d[1]]), 64, "TXPKTSIZE = payload len");
        assert_eq!(d[2], TX_DESC_SIZE as u8, "OFFSET = descriptor size (40)");
        let qsel = (u32::from_le_bytes([d[4], d[5], d[6], d[7]]) >> 8) & 0x1F;
        assert_eq!(qsel, QSLT_BEACON & 0x1F, "QSEL = beacon queue");
    }
}
