//! Minimal blocking SX1262 (Semtech) LoRa driver for the Waveshare USB-LoRa dongle.
//!
//! Board specifics (from the Archie3d reference + the SX126x datasheet):
//!   - regulator: **LDO** (not DC-DC);
//!   - DIO2 is NOT the RF switch — the MCU drives PB4 (RF_SW): **HIGH = RX, LOW = TX**;
//!   - TCXO on DIO3 at **1.7 V** (so an XOSC calibration is required after enabling it);
//!   - PA is an SX1262 (device_sel = 0).
//!
//! Generic over embedded-hal 0.2 SPI (blocking Transfer + Write) and the six control GPIOs, so the
//! same driver compiles against the stm32f1xx-hal on the GD32. NSS is driven by hand (SX126x needs
//! NSS held across the BUSY handshake). Reads follow the Semtech convention: the byte after the
//! opcode carries the chip status, then the requested data.

#![allow(dead_code)]

use embedded_hal::blocking::spi::{Transfer as SpiTransfer, Write as SpiWrite};
use embedded_hal::digital::v2::{InputPin, OutputPin};

use crate::capture::{self, StampVerdict};
use crate::rxstamp;

// --- Opcodes ---
const OP_SET_STANDBY: u8 = 0x80;
const OP_SET_PACKET_TYPE: u8 = 0x8A;
const OP_SET_RF_FREQUENCY: u8 = 0x86;
const OP_SET_PA_CONFIG: u8 = 0x95;
const OP_SET_TX_PARAMS: u8 = 0x8E;
const OP_SET_BUFFER_BASE: u8 = 0x8F;
const OP_SET_MOD_PARAMS: u8 = 0x8B;
const OP_SET_PKT_PARAMS: u8 = 0x8C;
const OP_SET_DIO_IRQ: u8 = 0x08;
const OP_CLR_IRQ: u8 = 0x02;
const OP_GET_IRQ: u8 = 0x12;
const OP_SET_TX: u8 = 0x83;
const OP_SET_RX: u8 = 0x82;
const OP_WRITE_BUFFER: u8 = 0x0E;
const OP_READ_BUFFER: u8 = 0x1E;
const OP_WRITE_REGISTER: u8 = 0x0D;
const OP_READ_REGISTER: u8 = 0x1D;
const OP_GET_RX_BUF_STATUS: u8 = 0x13;
const OP_GET_PKT_STATUS: u8 = 0x14;
const OP_GET_STATUS: u8 = 0xC0;
const OP_GET_DEVICE_ERRORS: u8 = 0x17;
const OP_CLR_DEVICE_ERRORS: u8 = 0x07;
const OP_CALIBRATE: u8 = 0x89;
const OP_CALIBRATE_IMAGE: u8 = 0x98;
const OP_SET_DIO3_TCXO: u8 = 0x97;
const OP_SET_DIO2_RFSW: u8 = 0x9D;
const OP_SET_REGULATOR: u8 = 0x96;
const OP_SET_CAD_PARAMS: u8 = 0x88; // #52: Channel Activity Detection config
const OP_SET_CAD: u8 = 0xC5; //        #52: run one CAD
const OP_GET_RSSI_INST: u8 = 0x15; //  #52: instantaneous channel RSSI (must be in RX)

// The chip's own packet counters (DS §13.5.5). Without these a CRC failure vanishes inside
// `poll_rx` with nothing counting it, so RX loss is invisible to the host (bug P5).
const OP_GET_STATS: u8 = 0x10; //      GetStats   -> [nbPktReceived(2), nbPktCrcError(2), nbPktHeaderErr(2)]
const OP_RESET_STATS: u8 = 0x00; //    ResetStats -> zero all three (takes six 0x00 param bytes)

// --- Registers ---
const REG_LORA_SYNC_MSB: u16 = 0x0740;
// GFSK-mode registers. `SetPacketType` repurposes the register file, so these are only meaningful
// while the chip is in [`SX126X_PKT_GFSK`] — which is why every one of them is (re)written by
// `apply_phy` on entry to the mode rather than once at init.
const REG_GFSK_WHITENING_MSB: u16 = 0x06B8; // [0] = seed bit 8; the rest of the byte is reserved
const REG_GFSK_CRC_INIT_MSB: u16 = 0x06BC;
const REG_GFSK_CRC_POLY_MSB: u16 = 0x06BE;
const REG_GFSK_SYNC0: u16 = 0x06C0; //        8 sync bytes, MSB-first from here
const REG_RANDOM_GEN: u16 = 0x0819; // #52: SX1262 hardware random-number registers (0x0819..0x081C)
/// RX gain (DS §9.6). The power-on default is power-saving; on a bearer whose entire purpose is
/// reach that is the wrong default, so this firmware writes BOOSTED at init and on every RX arm
/// (the register is not retained across a warm start, so re-writing it is the safe pattern).
const REG_RX_GAIN: u16 = 0x08AC;
/// Boosted LNA gain: ~+3 dB sensitivity for ~+2 mA in RX. This firmware's default.
pub const RX_GAIN_BOOSTED: u8 = 0x94;
/// Power-saving LNA gain — the SX126x power-on default, selectable by the host.
pub const RX_GAIN_POWER_SAVING: u8 = 0x96;

// --- IRQ bits ---
pub const IRQ_TX_DONE: u16 = 0x0001;
pub const IRQ_RX_DONE: u16 = 0x0002;
pub const IRQ_CRC_ERR: u16 = 0x0040;
pub const IRQ_CAD_DONE: u16 = 0x0080; //     #52: CAD finished
pub const IRQ_CAD_DETECTED: u16 = 0x0100; // #52: CAD saw channel activity (busy)
pub const IRQ_TIMEOUT: u16 = 0x0200;

/// ★ **What is allowed to drive the DIO1 pin: `RxDone`, and nothing else.**
///
/// PB0 is wired to DIO1 and is the TIM3_CH3 capture input ([`crate::rxstamp`]), so this mask decides
/// what a captured edge can *mean*. It used to be `TX_DONE | RX_DONE | TIMEOUT`, which made an edge
/// **not self-identifying** and left the attribution resting on an argument about which of those
/// bits could latch while RX was armed. The argument had a hole: `start_rx` clears the latch ~40 µs
/// before it issues `SetRx`, so a `TxDone` arriving in that gap (reachable when `wait_txdone`'s
/// 2000 × 1 ms ceiling expires — SF12/BW125 at 247 B is ~8.9 s of airtime, well past it) leaves
/// `TX_DONE` latched with the receiver armed. DIO1 then stays HIGH, the next frame's `RxDone` raises
/// **no rising edge at all**, and `take()` hands back the stale TX edge with `edges == 1` and no
/// overcapture — indistinguishable, to the timer, from a clean reception.
///
/// Narrowing the mask deletes that class rather than arguing about it. Nothing is lost: no code
/// reads the DIO1 pin as a GPIO (`wait_txdone`, `poll_rx` and `do_cad` all poll `GetIrqStatus` over
/// SPI), and the full IRQ **status** word is unaffected — `set_dio_irq`'s first argument stays
/// `0xFFFF`, so `TX_DONE`, `TIMEOUT`, `CRC_ERR` and the CAD bits all still latch and are all still
/// read. Only the pin gets quieter.
pub const DIO1_MASK: u16 = IRQ_RX_DONE;

// --- `SetPacketType` (0x8A) argument values, in the SX126x's OWN numbering (DS Table 13-38) ---
//
// ⚠ These are NOT the 7E-A5 v3 wire values. The wire uses the LR20xx `SetPacketType` numbering
// (0x0 LoRa, 0x2 FSK, ...), which disagrees with the SX126x's on BOTH modes this part implements —
// LoRa is 0x0 on the wire and 0x01 here, FSK is 0x2 on the wire and 0x00 here. The translation
// happens once, at the protocol boundary in `main.rs` (`wire_to_chip_phy`), and the mapping is
// pinned by a compile-time table there so the two numberings cannot be confused silently.
/// (G)FSK packet type — the SX1262's second modem (DS §6.1). Bring-up in [`Sx1262::apply_phy`].
pub const SX126X_PKT_GFSK: u8 = 0x00;
/// LoRa packet type — this firmware's default and the mode every fleet peer speaks.
pub const SX126X_PKT_LORA: u8 = 0x01;
// Not implemented here, and therefore never advertised: LR-FHSS (0x03 on the SX1262) is
// TRANSMIT-ONLY and builds its own hop sequence inside the packet, so it can neither receive nor
// take a host hop table. Claiming it would be exactly the "plausible invention" the capability
// rules forbid.

// --- LoRa modulation codes ---
pub const BW_125: u8 = 0x04;
pub const BW_250: u8 = 0x05;
pub const BW_500: u8 = 0x06;
pub const CR_4_5: u8 = 0x01;
pub const CR_4_6: u8 = 0x02;
pub const CR_4_7: u8 = 0x03;
pub const CR_4_8: u8 = 0x04;

// --- (G)FSK profile: ONE fixed operating point, and every number here is a source constant -------
//
// The SX1262's GFSK modem has its own modulation/packet/CRC/whitening configuration, none of which
// is shared with LoRa — which is precisely why `SetPacketType` is a PHY *switch* and not a flag.
// This firmware brings up a single well-known profile rather than inventing a knob surface: 50 kbps
// / ±25 kHz deviation / 117.3 kHz RX bandwidth / BT 0.5 Gaussian shaping, the LoRaWAN FSK profile
// (RP002 "FSK modulation, 50 kbps, BT 0.5, Fdev 25 kHz"). A bitrate/deviation knob would need a
// fleet-wide opcode assignment; `CMD_SET_MOD` carries `[sf, bw, cr]`, which GFSK has none of, so
// this node REFUSES it in GFSK rather than reinterpreting three bytes into a different meaning.
/// Crystal reference the SX126x's frequency and bitrate registers are scaled against (DS §13.4.1).
const XTAL_HZ: u64 = 32_000_000;
/// GFSK bitrate. Also the divisor in [`gfsk_airtime_ms`].
pub const GFSK_BITRATE_BPS: u32 = 50_000;
/// GFSK frequency deviation (single-sided).
pub const GFSK_FDEV_HZ: u32 = 25_000;
/// `SetModulationParams` bitrate register: `br = 32 * F_XTAL / bitrate` (DS §13.4.5.1).
/// 32 x 32 MHz / 50 kbps = 20 480 = 0x005000.
const GFSK_BR_REG: u32 = ((32 * XTAL_HZ) / GFSK_BITRATE_BPS as u64) as u32;
const _: () = assert!(GFSK_BR_REG == 0x00_5000);
/// `SetModulationParams` deviation register: `fdev = Fdev * 2^25 / F_XTAL` — the same scaling as
/// `SetRfFrequency`. 25 kHz -> 26 214.
const GFSK_FDEV_REG: u32 = (((GFSK_FDEV_HZ as u64) << 25) / XTAL_HZ) as u32;
const _: () = assert!(GFSK_FDEV_REG == 26_214);
/// Gaussian filter BT 0.5 (DS `SetModulationParams` pulse-shape table).
const GFSK_PULSE_SHAPE: u8 = 0x09;
/// RX bandwidth code 0x0B = **117.3 kHz** double-sided (DS Table 13-38 GFSK RX-bandwidth table).
const GFSK_RX_BW: u8 = 0x0B;
/// The bandwidth that code means, kept beside it so the Carson-rule check below is real arithmetic
/// and not a comment.
pub const GFSK_RX_BW_HZ: u32 = 117_300;
// Carson: the occupied bandwidth is 2*(Fdev + BR/2). The receiver must be at least that wide, or
// the profile is mis-specified in a way no runtime check would ever catch.
const _: () = assert!(2 * (GFSK_FDEV_HZ + GFSK_BITRATE_BPS / 2) <= GFSK_RX_BW_HZ);
/// Preamble-detector length code 0x05 = 16 bits (DS GFSK `SetPacketParams`).
const GFSK_PREAMBLE_DET: u8 = 0x05;
/// Sync-word length **in bits**. 24 bits = the 3-byte sync word below.
pub const GFSK_SYNC_BITS: u8 = 24;
/// Default GFSK sync word: `C1 94 C1`, the SX127x/LoRaWAN FSK convention. Byte 0 is the one an
/// SX127x peer's `RegSyncValue1` holds, which is what `CMD_SET_SYNC`'s single host byte replaces.
const GFSK_SYNC_DEFAULT: [u8; 3] = [0xC1, 0x94, 0xC1];
/// GFSK CRC type 0x06 = 2 bytes, inverted (DS GFSK `SetPacketParams` CRC table) — the LoRaWAN FSK
/// convention, paired with the poly/seed below. (In GFSK the CRC codes are NOT the LoRa on/off
/// 0x00/0x01: 0x01 means CRC *off* there, which is an easy and silent way to disable it.)
const GFSK_CRC_2_BYTE_INV: u8 = 0x06;
const GFSK_CRC_INIT: u16 = 0x1D0F;
const GFSK_CRC_POLY: u16 = 0x1021;
/// Whitening LFSR seed. Whitening is ON: a GFSK payload of repeated bytes otherwise puts a tone in
/// the middle of the channel and the DC/AGC loops fight it.
const GFSK_WHITENING_SEED: u16 = 0x01FF;
/// Largest GFSK payload the variable-length header can describe (its length field is one byte).
/// Reported nowhere as `max_payload`, because the 7E-A5 serial framing binds first at 247 — but
/// kept here so the comparison is visible rather than assumed.
pub const GFSK_PDU_MAX: u16 = 255;

// TCXO control voltage codes.
const TCXO_1_7V: u8 = 0x01;
/// DIO3 TCXO startup timeout, in the SX126x's 15.625 us units (DS SetDIO3AsTcxoCtrl).
/// **5000 x 15.625 us = 78 125 us = 78.125 ms**, and that number is the single largest cost in this
/// firmware's command latency: every transition from STDBY_RC into a mode that needs the crystal
/// (RX, TX, CalibrateImage) stalls for it. See [`TCXO_STARTUP_US`] and `set_frequency`.
const TCXO_TIMEOUT_UNITS: u32 = 5000;
/// The same figure in microseconds. Kept as a constant because the scheduled-TX path is designed
/// around AVOIDING it (`stage_tx` uses STDBY_XOSC, which never stops the crystal) and the retune
/// path around not paying it twice (`set_frequency` skips a redundant image calibration).
pub const TCXO_STARTUP_US: u32 = TCXO_TIMEOUT_UNITS * 15_625 / 1000; // 78_125

/// `SetTxParams` ramp-time code this firmware uses, and its value in microseconds (DS SetTxParams
/// ramp table: 0x04 = SET_RAMP_200U). It is a component of `sched_gran_ns` in main.rs, so the code
/// and the number it implies live together.
pub const TX_RAMP_CODE: u8 = 0x04;
pub const TX_RAMP_US: u32 = 200;

// --- Capability constants (the source of truth EVT_CAP reports; see main.rs `send_cap`) ---
/// Carrier range THIS firmware constrains itself to. Not the SX1262's silicon range (150-960 MHz):
/// `init`/`set_frequency` hard-code the 902-928 MHz image calibration (`calibrate_image(0xE1,0xE9)`,
/// DS §13.1.12 table), so outside this band the image rejection is simply wrong. Reporting the
/// silicon range would be a lie the host would act on.
pub const FREQ_MIN_HZ: u32 = 902_000_000;
pub const FREQ_MAX_HZ: u32 = 928_000_000;
/// SetTxParams power range for an SX1262 PA (DS §13.4.4). `set_power` clamps to this, so it is the
/// range the host can actually obtain — the value it reports back in EVT_INFO.
pub const PWR_MIN_DBM: i8 = -9;
pub const PWR_MAX_DBM: i8 = 22;
/// `CalibrateImage` band code for 902-928 MHz (DS SetCalibrateImage frequency-band table), the ONLY
/// band this firmware operates: `CMD_SET_FREQ` refuses everything outside [`FREQ_MIN_HZ`]..=
/// [`FREQ_MAX_HZ`]. Because the argument is a BAND and not a point, one calibration covers every
/// frequency the host can legally ask for — which is what lets `set_frequency` skip it. See the
/// `Sx1262::cal_band` memo.
pub const CAL_BAND_902_928: (u8, u8) = (0xE1, 0xE9);

/// Spreading factors this firmware operates and `CMD_SF_SCAN` sweeps. The chip also supports SF5/SF6,
/// but those need a different sync-word handling and do not interop with the SX127x peers in this
/// fleet, so they are outside the advertised range.
pub const SF_MIN: u8 = 7;
pub const SF_MAX: u8 = 12;

// Assumes an 8 MHz core clock (HSI default) for the busy-wait delays.
const CYCLES_PER_US: u32 = 8;

pub struct Diagnostics {
    pub status: u8,         // GetStatus byte (chip mode + command status)
    pub sync_readback: u16, // LoRa sync-word registers read back (should equal what we set)
    pub device_errors: u16, // GetDeviceErrors op-error bitfield (0 = clean)
    pub busy_ok: bool,      // BUSY settled low within the timeout after reset
    /// GetStatus taken immediately after the boot `SetPacketType`, i.e. the chip's verdict on the
    /// initial PHY bring-up — the same byte `EVT_PHY_ERR` would carry, judged by
    /// [`cmd_status_ok`]. Reported in the boot EVT_LOG so a node that came up in a PHY the silicon
    /// declined says so at boot instead of at the first failed transmission.
    pub phy_status: u8,
}

pub struct RxPacket {
    pub len: u8,
    pub rssi_dbm: i16,
    pub snr_db: i16,
    /// **The hardware RX timestamp, and the verdict on whether it may be attributed to THIS frame.**
    ///
    /// Read inside [`Sx1262::poll_rx`] rather than by the caller, and that is a correctness
    /// requirement rather than a convenience: the read has to happen between `GetIrqStatus` and
    /// `ClearIrqStatus`, in the one window where DIO1 is provably still high and no second rising
    /// edge can exist. After the clear, `poll_rx` spends ~2 ms reading the buffer out; a frame
    /// arriving in that stretch would take its own capture and OVERWRITE the register, so a stamp
    /// read at the call site could carry frame 2's instant on frame 1's payload — silently. See
    /// [`crate::rxstamp`].
    pub stamp: StampVerdict,
}

/// The SX126x's own RX packet counters (`GetStats`, DS §13.5.5). These are the ONLY place a CRC
/// failure is visible: `poll_rx` drops such a frame and returns `None`, so without reading these the
/// host cannot tell a quiet channel from a channel it is failing to decode.
pub struct ChipStats {
    pub pkt_received: u16,
    pub pkt_crc_error: u16,
    pub pkt_header_error: u16,
}

pub struct Sx1262<SPI, NSS, RST, BSY, DIO1, RFSW> {
    spi: SPI,
    nss: NSS,
    rst: RST,
    busy: BSY,
    dio1: DIO1,
    rfsw: RFSW,
    /// Preamble length. **The unit is per-PHY**: LoRa symbols, or GFSK *bytes* (the register wants
    /// bits, so the GFSK path multiplies by 8). Runtime-tunable (#52); longer = more reliable CAD /
    /// preamble detection by peers.
    preamble: u16,
    /// The `SetPacketType` value currently loaded, in the CHIP's numbering ([`SX126X_PKT_LORA`] /
    /// [`SX126X_PKT_GFSK`]). Every packet-shaped operation below branches on it, because the chip's
    /// two modems share opcodes but not parameter layouts: `SetModulationParams` is 4 bytes in LoRa
    /// and 8 in GFSK, `SetPacketParams` 6 and 9, `GetPacketStatus` decodes differently, and CAD does
    /// not exist in GFSK at all. Getting this wrong is silent — the chip accepts the bytes.
    pkt_type: u8,
    /// LoRa sync word in the SX127x single-byte convention, remembered so a PHY switch can restore
    /// it (the sync registers move from 0x0740 to 0x06C0 between the two modems).
    lora_sync: u8,
    /// GFSK sync word (24 bits). Byte 0 is what `CMD_SET_SYNC` writes.
    gfsk_sync: [u8; 3],
    /// LNA gain written to [`REG_RX_GAIN`] on every RX arm. Defaults to [`RX_GAIN_BOOSTED`].
    rx_gain: u8,
    /// **The chip's own completed-reception count at the instant the current capture window
    /// opened** — the baseline half of the coalescing detector. See [`Sx1262::completed_packets`]
    /// and [`crate::capture::attribute`]. Snapshotted inside `clear_irq`, so it is refreshed by the
    /// same call that arms the timer window and cannot drift from it.
    pkt_base: u16,
    /// Set when a `ResetStats` moved [`Self::pkt_base`] out from under an OPEN capture window, so
    /// that window's edge count and packet count no longer share an origin. Consumed by the next
    /// `poll_rx`, which refuses to attribute that one frame. See the ☠ note there.
    window_dirty: bool,
    /// **Image-calibration memo** — the (f1, f2) band pair currently loaded in the chip, or (0, 0)
    /// if none. `CalibrateImage` takes a BAND, so re-running it for a hop that stays inside the
    /// band it already calibrated is pure cost; this remembers what is loaded so `set_frequency`
    /// can skip it. Calibration results live in chip registers that survive standby, so the memo is
    /// only invalidated by a hard reset — which is exactly when `init` reloads it.
    cal_band: (u8, u8),
}

impl<SPI, NSS, RST, BSY, DIO1, RFSW, E> Sx1262<SPI, NSS, RST, BSY, DIO1, RFSW>
where
    SPI: SpiTransfer<u8, Error = E> + SpiWrite<u8, Error = E>,
    NSS: OutputPin,
    RST: OutputPin,
    RFSW: OutputPin,
    BSY: InputPin,
    DIO1: InputPin,
{
    pub fn new(spi: SPI, nss: NSS, rst: RST, busy: BSY, dio1: DIO1, rfsw: RFSW) -> Self {
        let mut s = Self {
            spi,
            nss,
            rst,
            busy,
            dio1,
            rfsw,
            preamble: 8,
            pkt_type: SX126X_PKT_LORA,
            lora_sync: 0x12,
            gfsk_sync: GFSK_SYNC_DEFAULT,
            rx_gain: RX_GAIN_BOOSTED,
            pkt_base: 0, // the chip's counters are 0 out of reset; `init`'s clear re-reads anyway
            window_dirty: false,
            cal_band: (0, 0), // nothing calibrated until `init` runs
        };
        let _ = s.nss.set_high();
        s
    }

    fn delay_us(us: u32) {
        cortex_m::asm::delay(us.saturating_mul(CYCLES_PER_US));
    }
    fn delay_ms(ms: u32) {
        Self::delay_us(ms.saturating_mul(1000));
    }

    /// Poll BUSY until it drops low, up to ~200 ms. Returns false on timeout.
    fn wait_busy(&mut self) -> bool {
        for _ in 0..20_000 {
            if matches!(self.busy.is_low(), Ok(true)) {
                return true;
            }
            Self::delay_us(10);
        }
        false
    }

    // --- SPI primitives (each frames one NSS-low..high transaction) ---

    fn cmd(&mut self, opcode: u8, params: &[u8]) {
        self.wait_busy();
        let _ = self.nss.set_low();
        let _ = self.spi.write(&[opcode]);
        if !params.is_empty() {
            let _ = self.spi.write(params);
        }
        let _ = self.nss.set_high();
    }

    /// Get-style read: send opcode, then clock `out.len()+1` bytes; first is status, rest is data.
    fn read_cmd(&mut self, opcode: u8, out: &mut [u8]) -> u8 {
        self.wait_busy();
        let mut buf = [0u8; 8];
        buf[0] = opcode;
        let n = 2 + out.len();
        let _ = self.nss.set_low();
        let _ = self.spi.transfer(&mut buf[..n]);
        let _ = self.nss.set_high();
        for (i, b) in out.iter_mut().enumerate() {
            *b = buf[2 + i];
        }
        buf[1]
    }

    fn write_regs(&mut self, addr: u16, data: &[u8]) {
        self.wait_busy();
        let _ = self.nss.set_low();
        let _ = self
            .spi
            .write(&[OP_WRITE_REGISTER, (addr >> 8) as u8, addr as u8]);
        let _ = self.spi.write(data);
        let _ = self.nss.set_high();
    }

    fn read_regs(&mut self, addr: u16, out: &mut [u8]) {
        self.wait_busy();
        let mut hdr = [OP_READ_REGISTER, (addr >> 8) as u8, addr as u8, 0x00];
        let _ = self.nss.set_low();
        let _ = self.spi.transfer(&mut hdr); // hdr[3] <- status
        for b in out.iter_mut() {
            *b = 0;
        }
        let _ = self.spi.transfer(out); // <- register data
        let _ = self.nss.set_high();
    }

    fn write_buffer(&mut self, offset: u8, data: &[u8]) {
        self.wait_busy();
        let _ = self.nss.set_low();
        let _ = self.spi.write(&[OP_WRITE_BUFFER, offset]);
        let _ = self.spi.write(data);
        let _ = self.nss.set_high();
    }

    fn read_buffer(&mut self, offset: u8, out: &mut [u8]) {
        self.wait_busy();
        let mut hdr = [OP_READ_BUFFER, offset, 0x00];
        let _ = self.nss.set_low();
        let _ = self.spi.transfer(&mut hdr); // hdr[2] <- status
        for b in out.iter_mut() {
            *b = 0;
        }
        let _ = self.spi.transfer(out); // <- payload
        let _ = self.nss.set_high();
    }

    // --- Antenna (RF) switch: PB4 HIGH = RX, LOW = TX ---
    fn rf_rx(&mut self) {
        let _ = self.rfsw.set_high();
    }
    fn rf_tx(&mut self) {
        let _ = self.rfsw.set_low();
    }

    // --- Command wrappers ---
    fn set_standby(&mut self, cfg: u8) {
        self.cmd(OP_SET_STANDBY, &[cfg]);
    }
    fn set_regulator(&mut self, mode: u8) {
        self.cmd(OP_SET_REGULATOR, &[mode]);
    }
    fn set_dio2_rfsw(&mut self, enable: bool) {
        self.cmd(OP_SET_DIO2_RFSW, &[enable as u8]);
    }
    fn set_dio3_tcxo(&mut self, voltage: u8, timeout: u32) {
        self.cmd(
            OP_SET_DIO3_TCXO,
            &[
                voltage,
                (timeout >> 16) as u8,
                (timeout >> 8) as u8,
                timeout as u8,
            ],
        );
    }
    fn calibrate(&mut self, mask: u8) {
        self.cmd(OP_CALIBRATE, &[mask]);
    }
    fn calibrate_image(&mut self, f1: u8, f2: u8) {
        self.cmd(OP_CALIBRATE_IMAGE, &[f1, f2]);
    }
    /// `SetPacketType` — **the PHY switch**, in the chip's own numbering. Per DS §13.4.2 it must be
    /// the FIRST command of a configuration sequence, because it re-maps the register file and the
    /// parameter layout of `SetModulationParams`/`SetPacketParams`; everything the new mode needs is
    /// therefore (re)written after it by [`Self::apply_phy`].
    fn set_packet_type(&mut self, chip_pkt: u8) {
        self.cmd(OP_SET_PACKET_TYPE, &[chip_pkt]);
    }

    /// GFSK `SetModulationParams` — 8 parameters, against LoRa's 4 (DS §13.4.5).
    fn set_gfsk_mod_params(&mut self) {
        let br = GFSK_BR_REG;
        let fd = GFSK_FDEV_REG;
        self.cmd(
            OP_SET_MOD_PARAMS,
            &[
                (br >> 16) as u8,
                (br >> 8) as u8,
                br as u8,
                GFSK_PULSE_SHAPE,
                GFSK_RX_BW,
                (fd >> 16) as u8,
                (fd >> 8) as u8,
                fd as u8,
            ],
        );
    }

    /// GFSK `SetPacketParams` — 9 parameters, against LoRa's 6 (DS §13.4.6). Variable-length
    /// packets, so the one-byte length field goes on air and the receiver does not need to be told
    /// the size in advance (that byte is charged for in [`gfsk_airtime_ms`]).
    ///
    /// `preamble` is in BITS here; the firmware's host-facing preamble knob is in bytes for this
    /// PHY, so it is multiplied by 8 and saturated at the register's 16-bit ceiling.
    fn set_gfsk_pkt_params(&mut self, len: u8) {
        let bits = (self.preamble as u32).saturating_mul(8).min(0xFFFF) as u16;
        self.cmd(
            OP_SET_PKT_PARAMS,
            &[
                (bits >> 8) as u8,
                bits as u8,
                GFSK_PREAMBLE_DET,
                GFSK_SYNC_BITS,
                0x00, // address filtering off — this is a broadcast bearer with no host identity
                0x01, // variable length (explicit 1-byte length on air)
                len,
                GFSK_CRC_2_BYTE_INV,
                0x01, // whitening on
            ],
        );
    }

    /// Write the 24-bit GFSK sync word (registers 0x06C0.., MSB first).
    fn set_gfsk_sync_regs(&mut self, sync: [u8; 3]) {
        self.write_regs(REG_GFSK_SYNC0, &sync);
    }

    /// GFSK CRC seed + polynomial. These have sane reset defaults, but they are written explicitly
    /// so the on-air CRC is a property of this file rather than of whatever last touched the chip.
    fn set_gfsk_crc(&mut self) {
        self.write_regs(REG_GFSK_CRC_INIT_MSB, &GFSK_CRC_INIT.to_be_bytes());
        self.write_regs(REG_GFSK_CRC_POLY_MSB, &GFSK_CRC_POLY.to_be_bytes());
    }

    /// GFSK whitening seed. Read-modify-write on the MSB register: only bit 0 of it is the seed's
    /// bit 8 and the rest is reserved, so writing the whole byte would clobber reserved state.
    fn set_gfsk_whitening(&mut self) {
        let mut msb = [0u8; 1];
        self.read_regs(REG_GFSK_WHITENING_MSB, &mut msb);
        let hi = (msb[0] & 0xFE) | ((GFSK_WHITENING_SEED >> 8) as u8 & 0x01);
        self.write_regs(REG_GFSK_WHITENING_MSB, &[hi, GFSK_WHITENING_SEED as u8]);
    }

    /// Packet parameters for a TRANSMIT of `len` bytes, in whichever PHY is loaded.
    fn set_tx_pkt_params(&mut self, len: u8) {
        if self.pkt_type == SX126X_PKT_LORA {
            let pre = self.preamble;
            self.set_pkt_params(pre, 0x00, len, 0x01, 0x00);
        } else {
            self.set_gfsk_pkt_params(len);
        }
    }

    /// Packet parameters for RECEIVE (max length), in whichever PHY is loaded.
    fn set_rx_pkt_params(&mut self) {
        if self.pkt_type == SX126X_PKT_LORA {
            let pre = self.preamble;
            self.set_pkt_params(pre, 0x00, 0xFF, 0x01, 0x00);
        } else {
            self.set_gfsk_pkt_params(0xFF);
        }
    }
    fn set_rf_freq(&mut self, hz: u32) {
        // freq_reg = hz * 2^25 / 32e6
        let reg = (((hz as u64) << 25) / 32_000_000) as u32;
        self.cmd(OP_SET_RF_FREQUENCY, &reg.to_be_bytes());
    }
    fn set_pa_config(&mut self, duty: u8, hp_max: u8) {
        self.cmd(OP_SET_PA_CONFIG, &[duty, hp_max, 0x00, 0x01]); // device_sel=0 (SX1262), paLut=1
    }
    fn set_tx_params(&mut self, power_dbm: i8, ramp: u8) {
        self.cmd(OP_SET_TX_PARAMS, &[power_dbm as u8, ramp]);
    }
    fn set_buffer_base(&mut self, tx: u8, rx: u8) {
        self.cmd(OP_SET_BUFFER_BASE, &[tx, rx]);
    }
    fn set_mod_params(&mut self, sf: u8, bw: u8, cr: u8, ldro: u8) {
        self.cmd(OP_SET_MOD_PARAMS, &[sf, bw, cr, ldro]);
    }
    fn set_pkt_params(&mut self, preamble: u16, header: u8, len: u8, crc: u8, iq: u8) {
        self.cmd(
            OP_SET_PKT_PARAMS,
            &[(preamble >> 8) as u8, preamble as u8, header, len, crc, iq],
        );
    }
    fn set_sync_word(&mut self, sx127x_sync: u8) {
        // Map an SX127x single-byte sync word to the SX126x two-register form.
        let msb = (sx127x_sync & 0xF0) | 0x04;
        let lsb = ((sx127x_sync & 0x0F) << 4) | 0x04;
        self.write_regs(REG_LORA_SYNC_MSB, &[msb, lsb]);
    }
    fn set_dio_irq(&mut self, irq: u16, dio1: u16) {
        self.cmd(
            OP_SET_DIO_IRQ,
            &[
                (irq >> 8) as u8,
                irq as u8,
                (dio1 >> 8) as u8,
                dio1 as u8,
                0,
                0,
                0,
                0,
            ],
        );
    }
    /// Clear the chip's IRQ latch — **and, with it, open a fresh hardware-capture window.**
    ///
    /// The three belong together. DIO1 is the OR of the latched masked bits and stays high until
    /// this command drops it, so "edges since the last ClearIrq", "IRQ bits since the last
    /// ClearIrq" and "packets the chip completed since the last ClearIrq" are all the same window —
    /// and the timer's capture register only means anything relative to it. Doing all three here
    /// rather than at the call sites is what makes the pairing structural: `transmit`, `stage_tx`,
    /// `wait_txdone`, `start_rx`, `poll_rx` and the three CAD clears all get it without anyone
    /// having to remember.
    ///
    /// The arm runs *after* the SPI transaction, deliberately. If DIO1 has not physically fallen yet
    /// the line is simply still high, which produces no new rising edge, so nothing is captured
    /// until the genuine next event.
    fn clear_irq(&mut self, mask: u16) {
        // ★ Read BEFORE the clear, never after. A frame landing between the two is then absent from
        // the baseline and shows up in the NEXT window's delta, which costs a good stamp; reading
        // after the clear would put that frame IN the baseline while its edge sat in the new
        // window, which admits a wrong one. See [`crate::capture::attribute`].
        let base = self.completed_packets();
        self.clear_irq_from(mask, base);
    }

    /// [`Self::clear_irq`] with the baseline already in hand — for `poll_rx`, which has just read
    /// the count to compute this window's delta and must not pay for a second `GetStats`.
    fn clear_irq_from(&mut self, mask: u16, base: u16) {
        self.cmd(OP_CLR_IRQ, &[(mask >> 8) as u8, mask as u8]);
        self.pkt_base = base;
        rxstamp::arm();
    }

    /// **The chip's own count of receptions that reached completion**, as one wrapping `u16`.
    ///
    /// ⚠ **Why it is a SUM and not `nbPktReceived` alone.** The SX126x datasheet (§13.5.5) does not
    /// say whether `nbPktReceived` counts every reception or only the CRC-good ones, and nothing on
    /// this bench has established it — the same disclosure the host makes about `phy_counters`. So
    /// the detector is built to be correct under *either* reading: a CRC-failed packet raises a real
    /// `RxDone` and a real DIO1 edge, and it increments `nbPktReceived`, or `nbPktCrcError`, or
    /// both. Summing the two can therefore over-count a bad packet (a +2 that degrades a stamp) but
    /// can never miss one (a +0 that admits a wrong one), which is the direction this has to err in.
    ///
    /// `nbPktHeaderErr` is deliberately **excluded**: a header error aborts before `RxDone`, so it
    /// raises no edge, does not move `payloadLengthRx`, and cannot mis-attribute anything. Counting
    /// it would only discard good stamps on a noisy channel.
    ///
    /// ☠ **Also not measured: whether these counters wrap or saturate at `0xFFFF`.** The host's
    /// `NdnStats` documents them as free-running and wrapping, which is what the arithmetic here
    /// assumes (`wrapping_sub`). If they in fact *saturate*, this detector fails CLOSED — the delta
    /// pins at 0 after the 65 536th reception and every frame degrades to the software stamp, which
    /// is visible immediately in `hw_stamped`/`hw_stamp_ambig` and in the per-frame notes, and a
    /// `CMD_RESET_STATS` (which zeroes the chip's counters and this baseline together) recovers it.
    /// That is the acceptable direction for an unknown; the other one would admit wrong stamps.
    ///
    /// One 6-byte status read, ~64 µs at SCK = 1 MHz, and safe with RX armed (see
    /// [`Self::get_stats`]).
    fn completed_packets(&mut self) -> u16 {
        let s = self.get_stats();
        s.pkt_received.wrapping_add(s.pkt_crc_error)
    }
    fn get_irq(&mut self) -> u16 {
        let mut b = [0u8; 2];
        self.read_cmd(OP_GET_IRQ, &mut b);
        ((b[0] as u16) << 8) | b[1] as u16
    }
    fn set_tx(&mut self, timeout: u32) {
        self.cmd(
            OP_SET_TX,
            &[(timeout >> 16) as u8, (timeout >> 8) as u8, timeout as u8],
        );
    }
    fn set_rx(&mut self, timeout: u32) {
        self.cmd(
            OP_SET_RX,
            &[(timeout >> 16) as u8, (timeout >> 8) as u8, timeout as u8],
        );
    }

    pub fn get_status(&mut self) -> u8 {
        self.read_cmd(OP_GET_STATUS, &mut [])
    }
    pub fn get_device_errors(&mut self) -> u16 {
        let mut b = [0u8; 2];
        self.read_cmd(OP_GET_DEVICE_ERRORS, &mut b);
        ((b[0] as u16) << 8) | b[1] as u16
    }
    fn clear_device_errors(&mut self) {
        self.cmd(OP_CLR_DEVICE_ERRORS, &[0, 0]);
    }

    /// Hard reset via the RESET pin, then wait for BUSY to settle.
    pub fn reset(&mut self) -> bool {
        let _ = self.rst.set_low();
        Self::delay_ms(2);
        let _ = self.rst.set_high();
        Self::delay_ms(5);
        self.wait_busy()
    }

    /// Full init for LoRa at `freq_hz`, given SF / BW / CR. US 902-928 image cal, sync 0x12
    /// (SX127x-private, matching the Heltec node), preamble 8, explicit header, CRC on, +22 dBm.
    pub fn init(&mut self, freq_hz: u32, sf: u8, bw: u8, cr: u8) -> Diagnostics {
        let busy_ok = self.reset();
        self.set_standby(0x00); // STDBY_RC
        self.set_regulator(0x00); // LDO
        self.set_dio2_rfsw(false); // MCU drives the RF switch
        self.set_dio3_tcxo(TCXO_1_7V, TCXO_TIMEOUT_UNITS); // 78.125 ms; see TCXO_STARTUP_US
        self.calibrate(0x7F); // recalibrate all blocks with the TCXO running
        Self::delay_ms(5);
        self.wait_busy();
        self.clear_device_errors();

        // The ONE image calibration this firmware needs: it covers 902-928 MHz, and `CMD_SET_FREQ`
        // refuses anything outside that, so every legal retune afterwards reuses it (see the
        // `cal_band` memo and `set_frequency`). `CalibrateImage` takes a BAND and is independent of
        // the packet type, so it is not repeated per PHY switch.
        self.calibrate_image(CAL_BAND_902_928.0, CAL_BAND_902_928.1);
        self.cal_band = CAL_BAND_902_928;
        // Everything that is per-PHY — packet type, modulation, sync, packet params, RX gain, IRQ
        // routing — goes through the SAME path a runtime `CMD_SET_PHY` takes, so a node that has
        // switched PHYs and back is in exactly the state a freshly-booted one is. There is no
        // second, init-only configuration sequence to drift.
        //
        // No ResetStats either: `init` begins with a hard RESET-pin reset, which already zeroes the
        // chip's packet counters. Issuing the extra command would only add a way for init to fail.
        let phy_status = self.apply_phy(SX126X_PKT_LORA, freq_hz, sf, bw, cr, 22);

        let mut sync = [0u8; 2];
        self.read_regs(REG_LORA_SYNC_MSB, &mut sync);
        Diagnostics {
            status: self.get_status(),
            sync_readback: ((sync[0] as u16) << 8) | sync[1] as u16,
            device_errors: self.get_device_errors(),
            busy_ok,
            phy_status,
        }
    }

    /// **Enter a PHY**, in the chip's numbering, and reconfigure everything that mode needs.
    /// Returns the chip's own `GetStatus` byte taken immediately after `SetPacketType` — the literal
    /// byte `EVT_PHY_ERR` carries, so a refusal is reported as the chip stated it rather than as a
    /// firmware opinion about it.
    ///
    /// This is the only place the packet type changes, and it rewrites the whole per-PHY
    /// configuration, because `SetPacketType` re-maps the register file: the modulation and packet
    /// parameter layouts differ, the sync word lives at a different address (0x0740 vs 0x06C0), and
    /// GFSK additionally needs CRC seed/polynomial and a whitening seed that LoRa has no concept of.
    /// Anything left over from the previous mode would be accepted by the chip and wrong on air.
    ///
    /// Leaves the chip in STDBY_RC; the caller re-arms RX.
    pub fn apply_phy(&mut self, chip_pkt: u8, freq_hz: u32, sf: u8, bw: u8, cr: u8, pwr_dbm: i8) -> u8 {
        self.set_standby(0x00);
        self.set_packet_type(chip_pkt);
        // Read the verdict HERE, while `cmdStatus` still refers to `SetPacketType` — after the
        // reconfiguration below it would describe whichever command ran last.
        let status = self.get_status();
        self.pkt_type = chip_pkt;
        self.set_rf_freq(freq_hz);
        self.set_power(pwr_dbm); // PA config + SetTxParams; identical in both modems
        self.set_buffer_base(0, 0);
        if chip_pkt == SX126X_PKT_LORA {
            self.set_modulation(sf, bw, cr);
            let sy = self.lora_sync;
            self.set_sync_word(sy);
        } else {
            self.set_gfsk_mod_params();
            let sy = self.gfsk_sync;
            self.set_gfsk_sync_regs(sy);
            self.set_gfsk_crc();
            self.set_gfsk_whitening();
        }
        self.set_rx_pkt_params();
        // Boosted LNA (P6): the register is not retained across a warm start, and a packet-type
        // change is exactly such a transition, so it is re-written here as well as on every RX arm.
        let g = self.rx_gain;
        self.write_regs(REG_RX_GAIN, &[g]);
        self.set_dio_irq(0xFFFF, DIO1_MASK);
        self.clear_irq(0xFFFF);
        status
    }

    /// The `SetPacketType` value currently loaded, in the CHIP's numbering.
    pub fn packet_type(&self) -> u8 {
        self.pkt_type
    }

    /// **Is Channel Activity Detection available in the current PHY?** `SetCad` (0xC5) is a LoRa
    /// modem function: it correlates against a LoRa preamble, and there is nothing to correlate in
    /// GFSK. Issuing it in GFSK is not harmless — the chip answers with a command error and the
    /// caller would read the resulting "not busy" as a clear channel. Every sensing path asks this
    /// first and falls back to the energy detector, which works in both modes.
    pub fn supports_cad(&self) -> bool {
        self.pkt_type == SX126X_PKT_LORA
    }

    /// Transmit one frame in the current PHY, blocking until TxDone or a bounded timeout. Returns
    /// true on TxDone.
    pub fn transmit(&mut self, payload: &[u8]) -> bool {
        self.rf_tx();
        self.set_tx_pkt_params(payload.len() as u8);
        self.write_buffer(0, payload);
        self.clear_irq(0xFFFF);
        self.set_tx(0); // no timeout: transmit until done
        self.wait_txdone()
    }

    // --- Scheduled TX (CMD_TX_AT). The SX1262 has no delayed key-up engine, so the MCU's timer is
    //     the transmit queue: `stage_tx` does every slow step AHEAD of the deadline, `tx_issue` is
    //     the single SPI transaction left to perform at it, and `wait_txdone` collects the result.
    //     Splitting `transmit` into those three is the whole mechanism. ---

    /// **Stage a frame for a key-up that has not happened yet.** Everything expensive — leaving RX,
    /// programming the packet length, pushing up to 251 payload bytes over a 1 MHz SPI, clearing the
    /// IRQ latch — happens here, before the deadline. Returns true once BUSY has settled low, i.e.
    /// the chip is idle and the only step remaining is [`Self::tx_issue`].
    ///
    /// ⚠ **STDBY_XOSC (0x01), not STDBY_RC (0x00).** STDBY_RC powers the crystal down, so the
    /// `SetTx` at the deadline would have to restart the TCXO and wait the DIO3 startup timeout —
    /// [`TCXO_STARTUP_US`] = 78.125 ms — which is the exact quantum measured in the knob latencies
    /// (see `set_frequency`). Entering STDBY_XOSC straight out of RX, where the crystal is already
    /// running, is what keeps the key-up in the hundreds of microseconds instead of tens of
    /// milliseconds. The caller does not have to trust that: `tx_issue` is timed against BUSY, and
    /// main.rs reports the measured key-up in EVT_TXDONE and folds it into EVT_CAP's
    /// `sched_gran_ns`, so if this assumption is ever wrong the node says so instead of lying.
    pub fn stage_tx(&mut self, payload: &[u8]) -> bool {
        self.set_standby(0x01); // STDBY_XOSC — keep the TCXO alive
        self.rf_tx();
        self.set_tx_pkt_params(payload.len() as u8);
        self.write_buffer(0, payload);
        self.clear_irq(0xFFFF);
        self.wait_busy()
    }

    /// **The key-up.** One NSS window, four bytes, nothing else: `SetTx(timeout = 0)`. Deliberately
    /// does NOT call `wait_busy` first — [`Self::stage_tx`] already waited BUSY low, and a poll loop
    /// between the timer event and NSS going low would add jitter to the one instant that matters.
    /// It also writes the opcode and its three parameter bytes as a single SPI transfer rather than
    /// the two `cmd()` issues, saving one call boundary inside the window.
    pub fn tx_issue(&mut self) {
        let _ = self.nss.set_low();
        let _ = self.spi.write(&[OP_SET_TX, 0x00, 0x00, 0x00]);
        let _ = self.nss.set_high();
    }

    /// Raw BUSY-pin state. After [`Self::tx_issue`] this is the chip's own key-up indicator: it goes
    /// high while `SetTx` is processed and drops once the transmitter is running, so the interval is
    /// a direct measurement of SPI + command processing + (if the crystal was stopped) TCXO restart.
    pub fn busy_high(&self) -> bool {
        matches!(self.busy.is_high(), Ok(true))
    }

    /// Block until TxDone (or the bounded timeout), clear the IRQ latch, report whether it fired.
    /// The 2000 x 1 ms ceiling covers the longest LoRa frame this firmware can emit (SF12/BW125).
    pub fn wait_txdone(&mut self) -> bool {
        let mut ok = false;
        for _ in 0..2000 {
            let irq = self.get_irq();
            if irq & IRQ_TX_DONE != 0 {
                ok = true;
                break;
            }
            if irq & IRQ_TIMEOUT != 0 {
                break;
            }
            Self::delay_ms(1);
        }
        self.clear_irq(0xFFFF);
        ok
    }

    // --- Runtime knobs (host-driven). Each expects the chip in standby; caller re-arms RX. ---

    /// Enter STDBY_RC so parameters can be changed safely.
    pub fn standby(&mut self) {
        self.set_standby(0x00);
    }

    /// Retune the carrier (Hz). Returns **true if an image calibration was needed** (and run).
    ///
    /// **Why this is not just `calibrate_image(); set_rf_freq()` any more (measured, 2026-08-28).**
    /// `CMD_SET_FREQ` cost 160.87 ms on o5p-0 and 160.15 ms on mds-05 while `CMD_SET_PWR` cost
    /// 82.68 ms and `CMD_SET_MOD` 82.78 ms, with a `CMD_GET_INFO` floor of 4.76 ms. Subtract:
    ///
    /// ```text
    ///   SET_PWR  - GET_INFO floor = 82.68 - 4.76 = 77.92 ms   <- one TCXO startup
    ///   SET_MOD  - GET_INFO floor = 82.78 - 4.76 = 78.02 ms   <- one TCXO startup
    ///   SET_FREQ - SET_PWR        = 160.87 - 82.68 = 78.19 ms <- a SECOND one
    ///   TCXO_STARTUP_US                                = 78.125 ms  (5000 x 15.625 us)
    /// ```
    ///
    /// Three independent subtractions land on the same 78.125 ms quantum to within 0.3 %. So the
    /// "~80 ms of extra retune cost" is **not** the image calibration's own compute time (the
    /// residual left for that is ~60 us); it is a second **TCXO startup**, forced because
    /// `CalibrateImage` needs the crystal and `standby()` had just stopped it by entering STDBY_RC.
    ///
    /// `CalibrateImage` takes a frequency **band**, not a point, and this firmware refuses every
    /// frequency outside 902-928 MHz — so the calibration `init` already ran covers every retune the
    /// host can legally request, and re-running it buys nothing while costing a full TCXO restart.
    /// The [`Self::cal_band`] memo makes that skip conditional rather than assumed: if the operating
    /// band ever widens, a target outside the loaded band still recalibrates.
    ///
    /// Expected cost after this change: a retune becomes the same shape as `SET_PWR`/`SET_MOD` —
    /// standby, one register write, re-arm RX — i.e. **~83 ms**, one remaining TCXO startup plus the
    /// serial floor. That is a PREDICTION from the arithmetic above and must be re-measured before
    /// the host's `retune_us` is changed away from 161 ms.
    pub fn set_frequency(&mut self, hz: u32) -> bool {
        let in_band = (FREQ_MIN_HZ..=FREQ_MAX_HZ).contains(&hz);
        // Outside the band there is no calibration this firmware knows to be correct, so it loads
        // none and says so; `CMD_SET_FREQ` rejects such a request before ever reaching here.
        let recalibrated = in_band && self.cal_band != CAL_BAND_902_928;
        if recalibrated {
            self.calibrate_image(CAL_BAND_902_928.0, CAL_BAND_902_928.1);
            self.cal_band = CAL_BAND_902_928;
        }
        self.set_rf_freq(hz);
        recalibrated
    }

    /// The image-calibration band currently loaded, or `(0, 0)` if none.
    pub fn cal_band(&self) -> (u8, u8) {
        self.cal_band
    }

    /// Set LoRa modulation: spreading factor (5-12), bandwidth code, coding-rate code.
    pub fn set_modulation(&mut self, sf: u8, bw: u8, cr: u8) {
        let ldro = if sf >= 11 && bw == BW_125 { 1 } else { 0 };
        self.set_mod_params(sf, bw, cr, ldro);
    }

    /// Set TX power in dBm (SX1262: up to +22), also re-optimising the PA config for the power band
    /// (Semtech DS §13.1.14) so lower powers are efficient instead of using the fixed +22 dBm PA setup.
    /// Returns the dBm actually applied — clamped to [`PWR_MIN_DBM`]..=[`PWR_MAX_DBM`], the SX1262
    /// SetTxParams range (DS §13.4.4). Before this clamp an out-of-range request was passed straight
    /// through to the chip as a raw register byte, so EVT_INFO reported a power the PA never produced
    /// and EVT_CAP's advertised range could not have been true.
    pub fn set_power(&mut self, dbm: i8) -> i8 {
        let dbm = dbm.clamp(PWR_MIN_DBM, PWR_MAX_DBM);
        // (paDutyCycle, hpMax) tiers; txParams power then fine-tunes within the band.
        let (duty, hp) = match dbm {
            d if d >= 20 => (0x04, 0x07), // +22 dBm optimal
            d if d >= 15 => (0x03, 0x05), // +20 dBm optimal
            d if d >= 10 => (0x02, 0x03), // +17 dBm optimal
            _ => (0x02, 0x02),            // +14 dBm optimal
        };
        self.set_pa_config(duty, hp);
        self.set_tx_params(dbm, TX_RAMP_CODE);
        dbm
    }

    /// Energy-detect the channel: arm RX, sample instantaneous RSSI, return true if it exceeds
    /// `thresh_dbm` (busy). Catches non-LoRa interference that CAD (preamble detect) is blind to.
    /// Leaves the chip in standby.
    pub fn rssi_busy(&mut self, thresh_dbm: i16) -> bool {
        self.rf_rx();
        self.set_rx(0xFFFFFF);
        Self::delay_us(300); // let AGC settle
        let r = self.rssi_inst();
        self.set_standby(0x00);
        r > thresh_dbm
    }

    /// Set the network sync word from the host's single byte, in whichever PHY is loaded.
    ///
    /// * **LoRa** — the SX127x single-byte convention (0x12 private / 0x34 public), expanded into
    ///   the SX126x's two-register form.
    /// * **GFSK** — the byte replaces sync byte 0, the one an SX127x FSK peer holds in
    ///   `RegSyncValue1`; bytes 1..3 keep the `C1 94 C1` convention's tail. One host byte maps to
    ///   exactly one on-air byte, so nothing about the mapping has to be guessed at either end.
    pub fn set_sync(&mut self, sx127x_sync: u8) {
        if self.pkt_type == SX126X_PKT_LORA {
            self.lora_sync = sx127x_sync;
            self.set_sync_word(sx127x_sync);
        } else {
            self.gfsk_sync[0] = sx127x_sync;
            let sy = self.gfsk_sync;
            self.set_gfsk_sync_regs(sy);
        }
    }

    /// Read back the first two sync-word register bytes of the current PHY — a genuine read-back in
    /// both modes, from the address that mode uses (LoRa 0x0740, GFSK 0x06C0). LoRa 0x12 reads back
    /// as 0x1424; GFSK's default reads back as 0xC194.
    pub fn read_sync(&mut self) -> u16 {
        let addr = if self.pkt_type == SX126X_PKT_LORA {
            REG_LORA_SYNC_MSB
        } else {
            REG_GFSK_SYNC0
        };
        let mut b = [0u8; 2];
        self.read_regs(addr, &mut b);
        ((b[0] as u16) << 8) | b[1] as u16
    }

    /// Arm continuous RX. Call once, then poll with `poll_rx`.
    pub fn start_rx(&mut self) {
        self.rf_rx();
        // REG_RX_GAIN is not retained across a warm start, so re-write it on every arm — one 4-byte
        // SPI write, negligible against the SetPacketParams/ClearIrq/SetRx that follow.
        let g = self.rx_gain;
        self.write_regs(REG_RX_GAIN, &[g]);
        self.set_rx_pkt_params();
        self.clear_irq(0xFFFF);
        self.set_rx(0xFFFFFF); // continuous
    }

    /// Non-blocking: if a frame arrived, copy it into `out` and return its metadata.
    ///
    /// A CRC error is dropped (returns None) after clearing the IRQ so RX stays armed. That drop is
    /// NOT silent any more: the chip's own `nbPktCrcError` counts it, readable via [`get_stats`] and
    /// reported in EVT_STATS (P5).
    ///
    /// `RxPacket::len` is the TRUE on-air length, which may exceed `out.len()`; the caller must
    /// compare the two to notice a truncation rather than assume `len` bytes landed in `out`.
    pub fn poll_rx(&mut self, out: &mut [u8]) -> Option<RxPacket> {
        let irq = self.get_irq();
        if irq & IRQ_RX_DONE == 0 {
            // No clear, so the capture window stays open. That is correct **given the narrow
            // [`DIO1_MASK`]**: the only edge that can be pending is an `RxDone` whose status bit
            // landed after `get_irq` sampled it, and the next poll reports that frame against that
            // edge — which is the right pairing.
            //
            // ⚠ It was NOT correct with the old three-source mask, and the guard below is written
            // against `DIO1_MASK` rather than against today's value of it so that widening the mask
            // cannot quietly restore the bug. A non-RX source latching here holds DIO1 high with the
            // receiver armed: the next frame's `RxDone` raises no edge, and `take()` returns the
            // stale edge with `edges == 1` and no overcapture, which `classify` cannot see through.
            // Clearing those bits (and, via `clear_irq`, re-arming) is the one-line fix; `RX_DONE`
            // is deliberately not in the mask, so a frame landing in this window is never dropped.
            let stale = irq & DIO1_MASK & !IRQ_RX_DONE;
            if stale != 0 {
                self.clear_irq(stale);
            }
            return None;
        }
        // ★ **The stamp is read HERE**, between the status read and the clear. `irq` is the REASON,
        // the capture is the INSTANT, and they describe the same event only because both are read in
        // this one IRQ-clear cycle.
        let stamp = rxstamp::take();
        let crc_err = irq & IRQ_CRC_ERR != 0;

        // ★ **The packet's identity is pinned in the SAME cycle as the stamp.** These two reads used
        // to sit *after* the clear — after DIO1 had fallen and a fresh window was open — so a frame
        // arriving in the ~50 µs between the arm and `GetRxBufferStatus` moved
        // `rxStartBufferPointer`, `payloadLengthRx` and the packet status, and this frame's stamp
        // went out with the NEXT frame's length, RSSI and SNR (which the next poll then reported a
        // second time). The stamp supplies the instant, the IRQ word supplies the reason, and these
        // supply the identity: all three are only the same event if they are read in one cycle.
        let mut st = [0u8; 2];
        self.read_cmd(OP_GET_RX_BUF_STATUS, &mut st);
        let len = st[0];
        let ptr = st[1];
        let mut ps = [0u8; 3];
        self.read_cmd(OP_GET_PKT_STATUS, &mut ps);

        // ★ **The chip's half of the attribution rule** — and the fix for the failure this whole
        // path exists to prevent. DIO1 is level-latched, so a frame arriving while it is already
        // high raises NO edge while the buffer and packet status above advance to describe it: the
        // timer then sees one clean edge belonging to an EARLIER frame and cannot tell. The chip's
        // own completed-packet count is the only per-frame detector this part has. Read last, so it
        // covers everything read above; exactly one completion is the only attributable window.
        let completed = self.completed_packets();
        // ☠ A `CMD_RESET_STATS` that landed *inside* this window zeroed the chip's counters and
        // `pkt_base` while leaving the timer's edge count and the chip's `RxDone` latch untouched.
        // The two halves of the rule then count from different origins, and the delta can read a
        // benign `1` for a frame whose edge belongs to its predecessor — the coalescing failure
        // again, through a smaller hole. A window in that state is not attributable, so it is not
        // attributed: exactly one frame pays, and `clear_irq_from` below re-bases the next.
        let delta = if core::mem::replace(&mut self.window_dirty, false) {
            0
        } else {
            completed.wrapping_sub(self.pkt_base)
        };
        let stamp = capture::attribute(stamp, delta);

        // The window closes here, and the count just read becomes the next window's baseline.
        self.clear_irq_from(0xFFFF, completed);
        if crc_err {
            // A CRC failure raises a real `RxDone` and a real edge. Both have now been consumed:
            // the IRQ status by the clear above, and the capture by the `arm` inside it, which
            // discards whatever is in the register and opens the next window. Neither is left to be
            // mis-attributed to the next frame. The chip's own `nbPktCrcError` counts the drop.
            return None;
        }
        // The payload readback stays AFTER the clear, deliberately: it is the ~2 ms transaction, and
        // holding DIO1 high across it would widen the coalescing window for no gain. Its hazard is
        // payload corruption by a later frame's write into the same buffer address, which is
        // pre-existing, orthogonal to attribution, and not what this reordering is about.
        let n = core::cmp::min(len as usize, out.len());
        self.read_buffer(ptr, &mut out[..n]);
        // `GetPacketStatus` returns three bytes in both modes and means something different by them
        // (DS §13.5.3):
        //   LoRa: [rssiPkt, snrPkt, signalRssiPkt]  -> rssi = -rssiPkt/2 dBm, snr = (i8)snr/4 dB
        //   GFSK: [rxStatus, rssiSync, rssiAvg]     -> rssi = -rssiSync/2 dBm, and there is NO SNR
        // Decoding a GFSK reply with the LoRa formula would report the rxStatus BITFIELD as an RSSI.
        let (rssi_dbm, snr_db) = if self.pkt_type == SX126X_PKT_LORA {
            (-(ps[0] as i16) / 2, (ps[1] as i8) as i16 / 4)
        } else {
            // The GFSK modem measures no signal-to-noise ratio. The field is 0 — an explicit "not
            // measured", never a number derived from something else and presented as an SNR.
            (-(ps[1] as i16) / 2, 0)
        };
        Some(RxPacket {
            len,
            rssi_dbm,
            snr_db,
            stamp,
        })
    }

    // --- #52: carrier sense (CAD), instantaneous RSSI, hardware RNG, preamble ---

    /// Runtime preamble length in symbols (used by `transmit`/`start_rx`).
    pub fn set_preamble(&mut self, preamble: u16) {
        self.preamble = preamble.max(1);
    }

    /// Configure Channel Activity Detection. `sym` = cadSymbolNum code (0..4 → 1/2/4/8/16 symbols);
    /// `det_peak`/`det_min` = detector sensitivity (SF-dependent, tune on air). Exit-mode STDBY.
    pub fn set_cad_params(&mut self, sym: u8, det_peak: u8, det_min: u8) {
        self.cmd(
            OP_SET_CAD_PARAMS,
            &[sym, det_peak, det_min, 0x00, 0x00, 0x00, 0x00],
        );
    }

    /// Run one CAD at the current modulation and block until it completes. Returns true if the channel
    /// is BUSY (a LoRa preamble/energy was detected). Call `set_cad_params` first; leaves the chip in
    /// STDBY (exit-mode 0), so the caller then transmits or re-arms RX.
    pub fn do_cad(&mut self) -> bool {
        self.rf_rx(); // CAD listens
        self.clear_irq(0xFFFF);
        self.cmd(OP_SET_CAD, &[]);
        for _ in 0..2000 {
            let irq = self.get_irq();
            if irq & IRQ_CAD_DONE != 0 {
                let busy = irq & IRQ_CAD_DETECTED != 0;
                self.clear_irq(0xFFFF);
                return busy;
            }
            Self::delay_us(100);
        }
        self.clear_irq(0xFFFF);
        false // timed out → treat as clear
    }

    /// Instantaneous channel RSSI in dBm. Only meaningful with RX armed.
    pub fn rssi_inst(&mut self) -> i16 {
        let mut b = [0u8; 1];
        self.read_cmd(OP_GET_RSSI_INST, &mut b);
        -(b[0] as i16) / 2
    }

    /// Select the LNA gain used from the next RX arm on: [`RX_GAIN_BOOSTED`] (this firmware's
    /// default, ~+3 dB sensitivity) or [`RX_GAIN_POWER_SAVING`] (the chip's power-on default).
    /// Applied immediately as well as on every subsequent `start_rx`.
    pub fn set_rx_gain(&mut self, gain: u8) {
        self.rx_gain = if gain == RX_GAIN_POWER_SAVING {
            RX_GAIN_POWER_SAVING
        } else {
            RX_GAIN_BOOSTED
        };
        let g = self.rx_gain;
        self.write_regs(REG_RX_GAIN, &[g]);
    }

    /// The LNA gain currently selected.
    pub fn rx_gain(&self) -> u8 {
        self.rx_gain
    }

    /// Read the chip's own RX packet counters (P5). Safe to call with RX armed — it is a status
    /// command and does not disturb the receiver.
    pub fn get_stats(&mut self) -> ChipStats {
        let mut b = [0u8; 6];
        self.read_cmd(OP_GET_STATS, &mut b);
        ChipStats {
            pkt_received: ((b[0] as u16) << 8) | b[1] as u16,
            pkt_crc_error: ((b[2] as u16) << 8) | b[3] as u16,
            pkt_header_error: ((b[4] as u16) << 8) | b[5] as u16,
        }
    }

    /// Zero the chip's packet counters (`ResetStats`), so the host can re-baseline without a reflash.
    ///
    /// The coalescing baseline moves with them, because it is a *difference* against these
    /// counters. But moving it is not enough on its own: this is the one writer of `pkt_base` that
    /// does NOT also arm the timer window, so a window already open when the reset lands is left
    /// with its two halves measured from different origins. [`Self::window_dirty`] marks that one
    /// window unattributable rather than letting its delta read a benign `1`.
    pub fn reset_stats(&mut self) {
        self.cmd(OP_RESET_STATS, &[0, 0, 0, 0, 0, 0]);
        self.pkt_base = 0;
        self.window_dirty = true;
    }

    /// One 32-bit sample from the SX1262 hardware RNG (LNA noise, read with IRQ masked while in RX).
    /// One-shot at init to seed the MCU's backoff PRNG; leaves the chip in STDBY.
    pub fn hw_random(&mut self) -> u32 {
        self.set_dio_irq(0x0000, 0x0000);
        self.rf_rx();
        self.set_rx(0xFFFFFF);
        Self::delay_ms(3);
        let mut b = [0u8; 4];
        self.read_regs(REG_RANDOM_GEN, &mut b);
        self.set_standby(0x00);
        self.set_dio_irq(0xFFFF, DIO1_MASK);
        // The RNG ran with the receiver armed and DIO1 masked, so an `RxDone` may have latched
        // unseen; re-enabling the mask on the line above would then drive DIO1 high immediately and
        // manufacture a rising edge that belongs to nothing. Clearing here consumes it — and, via
        // `clear_irq`, opens a clean capture window at the same instant.
        self.clear_irq(0xFFFF);
        u32::from_be_bytes(b)
    }
}

/// **Did the chip refuse the last command?** `GetStatus` (DS §13.5.1) packs the chip mode in bits
/// [6:4] and the command status in bits [3:1]; 0x3/0x4/0x5 are "command timeout", "command
/// processing error" and "failure to execute command". Everything else — including 0x2 (data
/// available) and 0x6 (command TX done) — is a normal outcome.
///
/// This is the oracle behind `EVT_PHY_ERR`: the node advertises the PHYs it brings up, and if the
/// silicon ever declines one at runtime the host is told so, with the chip's own byte attached,
/// instead of being left to infer it from frames that never arrive.
pub const fn cmd_status_ok(status: u8) -> bool {
    !matches!((status >> 1) & 0x07, 0x03 | 0x04 | 0x05)
}

/// GFSK time-on-air in whole milliseconds (rounded up), for the one profile this firmware brings up.
///
/// GFSK has no symbol/spreading arithmetic — it is a fixed-rate bit pipe, so the airtime is just the
/// framing's bit count over the bitrate. Every term is a field this driver actually programs in
/// [`Sx1262::set_gfsk_pkt_params`]:
///
/// ```text
///   preamble   preamble_bytes x 8   (the host knob is bytes in this PHY)
///   sync       GFSK_SYNC_BITS = 24
///   length     8                    variable-length packets put the length byte on air
///   payload    payload_len x 8
///   CRC        16                   GFSK_CRC_2_BYTE_INV
/// ```
///
/// At 50 kbps a 32-byte frame is 368 bits = 7.4 ms, against ~60 ms for the same frame at
/// SF7/BW125 — which is the whole reason the PHY is worth having as a knob.
pub const fn gfsk_airtime_ms(payload_len: u8, preamble_bytes: u16) -> u32 {
    // Saturated at the GFSK preamble register's 16-bit ceiling, matching what
    // `set_gfsk_pkt_params` actually programs — written as an `if` rather than `.min()` because
    // `Ord::min` is not const-callable, and this function is asserted at build time in `main.rs`.
    let raw = (preamble_bytes as u64).saturating_mul(8);
    let preamble_bits = if raw > 0xFFFF { 0xFFFF } else { raw };
    let bits = preamble_bits + GFSK_SYNC_BITS as u64 + 8 + (payload_len as u64) * 8 + 16;
    let us = bits * 1_000_000 / GFSK_BITRATE_BPS as u64;
    (us / 1000 + 1) as u32
}

/// LoRa time-on-air in whole milliseconds (rounded up). Standard Semtech formula, explicit header +
/// CRC on. Used to tell the host the real airtime so a fixed command timeout does not blow at high SF.
pub const fn airtime_ms(sf: u8, bw_code: u8, cr: u8, payload_len: u8, preamble: u16) -> u32 {
    let bw_hz: u64 = match bw_code {
        BW_250 => 250_000,
        BW_500 => 500_000,
        _ => 125_000,
    };
    let sf_i = sf as i64;
    let de: i64 = if sf >= 11 && bw_code == BW_125 { 1 } else { 0 };
    let cr_i = cr as i64; // 1..4
    let pl = payload_len as i64;
    // payloadSymbNb = 8 + max(ceil((8*PL - 4*SF + 28 + 16)/(4*(SF-2*DE))) * (CR+4), 0)
    let num = 8 * pl - 4 * sf_i + 28 + 16;
    let den = 4 * (sf_i - 2 * de);
    let mut steps = if num <= 0 || den <= 0 {
        0
    } else {
        (num + den - 1) / den
    };
    if steps < 0 {
        steps = 0;
    }
    let payload_sym = 8 + steps * (cr_i + 4);
    // Tsym (µs) = 2^SF * 1e6 / BW; preamble time = (preamble + 4.25) * Tsym = Tsym*(4*preamble+17)/4.
    let tsym_us: u64 = ((1u64 << sf) * 1_000_000) / bw_hz;
    let preamble_us = tsym_us * (4 * preamble as u64 + 17) / 4;
    let payload_us = tsym_us * payload_sym as u64;
    ((preamble_us + payload_us) / 1000 + 1) as u32
}
