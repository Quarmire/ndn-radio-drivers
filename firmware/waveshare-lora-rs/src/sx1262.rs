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

// --- Registers ---
const REG_LORA_SYNC_MSB: u16 = 0x0740;
const REG_RANDOM_GEN: u16 = 0x0819; // #52: SX1262 hardware random-number registers (0x0819..0x081C)

// --- IRQ bits ---
pub const IRQ_TX_DONE: u16 = 0x0001;
pub const IRQ_RX_DONE: u16 = 0x0002;
pub const IRQ_CRC_ERR: u16 = 0x0040;
pub const IRQ_CAD_DONE: u16 = 0x0080; //     #52: CAD finished
pub const IRQ_CAD_DETECTED: u16 = 0x0100; // #52: CAD saw channel activity (busy)
pub const IRQ_TIMEOUT: u16 = 0x0200;

// --- LoRa modulation codes ---
pub const BW_125: u8 = 0x04;
pub const BW_250: u8 = 0x05;
pub const BW_500: u8 = 0x06;
pub const CR_4_5: u8 = 0x01;
pub const CR_4_6: u8 = 0x02;
pub const CR_4_7: u8 = 0x03;
pub const CR_4_8: u8 = 0x04;

// TCXO control voltage codes.
const TCXO_1_7V: u8 = 0x01;

// Assumes an 8 MHz core clock (HSI default) for the busy-wait delays.
const CYCLES_PER_US: u32 = 8;

pub struct Diagnostics {
    pub status: u8,        // GetStatus byte (chip mode + command status)
    pub sync_readback: u16, // LoRa sync-word registers read back (should equal what we set)
    pub device_errors: u16, // GetDeviceErrors op-error bitfield (0 = clean)
    pub busy_ok: bool,      // BUSY settled low within the timeout after reset
}

pub struct RxPacket {
    pub len: u8,
    pub rssi_dbm: i16,
    pub snr_db: i16,
}

pub struct Sx1262<SPI, NSS, RST, BSY, DIO1, RFSW> {
    spi: SPI,
    nss: NSS,
    rst: RST,
    busy: BSY,
    dio1: DIO1,
    rfsw: RFSW,
    /// LoRa preamble length in symbols (runtime-tunable, #52). Longer = more reliable CAD by peers.
    preamble: u16,
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
        let mut s = Self { spi, nss, rst, busy, dio1, rfsw, preamble: 8 };
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
        let _ = self.spi.write(&[OP_WRITE_REGISTER, (addr >> 8) as u8, addr as u8]);
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
            &[voltage, (timeout >> 16) as u8, (timeout >> 8) as u8, timeout as u8],
        );
    }
    fn calibrate(&mut self, mask: u8) {
        self.cmd(OP_CALIBRATE, &[mask]);
    }
    fn calibrate_image(&mut self, f1: u8, f2: u8) {
        self.cmd(OP_CALIBRATE_IMAGE, &[f1, f2]);
    }
    fn set_packet_type_lora(&mut self) {
        self.cmd(OP_SET_PACKET_TYPE, &[0x01]);
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
                (irq >> 8) as u8, irq as u8,
                (dio1 >> 8) as u8, dio1 as u8,
                0, 0, 0, 0,
            ],
        );
    }
    fn clear_irq(&mut self, mask: u16) {
        self.cmd(OP_CLR_IRQ, &[(mask >> 8) as u8, mask as u8]);
    }
    fn get_irq(&mut self) -> u16 {
        let mut b = [0u8; 2];
        self.read_cmd(OP_GET_IRQ, &mut b);
        ((b[0] as u16) << 8) | b[1] as u16
    }
    fn set_tx(&mut self, timeout: u32) {
        self.cmd(OP_SET_TX, &[(timeout >> 16) as u8, (timeout >> 8) as u8, timeout as u8]);
    }
    fn set_rx(&mut self, timeout: u32) {
        self.cmd(OP_SET_RX, &[(timeout >> 16) as u8, (timeout >> 8) as u8, timeout as u8]);
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
        self.set_dio3_tcxo(TCXO_1_7V, 5000); // 5000 * 15.625 us = ~78 ms TCXO startup
        self.calibrate(0x7F); // recalibrate all blocks with the TCXO running
        Self::delay_ms(5);
        self.wait_busy();
        self.clear_device_errors();

        self.set_packet_type_lora();
        self.calibrate_image(0xE1, 0xE9); // 902-928 MHz band
        self.set_rf_freq(freq_hz);
        self.set_pa_config(0x04, 0x07); // +22 dBm SX1262 PA config
        self.set_tx_params(22, 0x04); // 22 dBm, 200 us ramp
        self.set_buffer_base(0, 0);

        // For SF7/BW125 the symbol time is 1.024 ms (< 16 ms) so low-data-rate optimize stays off.
        let ldro = if sf >= 11 && bw == BW_125 { 1 } else { 0 };
        self.set_mod_params(sf, bw, cr, ldro);
        self.set_sync_word(0x12);
        self.set_pkt_params(8, 0x00, 0xFF, 0x01, 0x00);
        self.set_dio_irq(0xFFFF, IRQ_TX_DONE | IRQ_RX_DONE | IRQ_TIMEOUT);
        self.clear_irq(0xFFFF);

        let mut sync = [0u8; 2];
        self.read_regs(REG_LORA_SYNC_MSB, &mut sync);
        Diagnostics {
            status: self.get_status(),
            sync_readback: ((sync[0] as u16) << 8) | sync[1] as u16,
            device_errors: self.get_device_errors(),
            busy_ok,
        }
    }

    /// Transmit one LoRa frame, blocking until TxDone or a bounded timeout. Returns true on TxDone.
    pub fn transmit(&mut self, payload: &[u8]) -> bool {
        self.rf_tx();
        let pre = self.preamble;
        self.set_pkt_params(pre, 0x00, payload.len() as u8, 0x01, 0x00);
        self.write_buffer(0, payload);
        self.clear_irq(0xFFFF);
        self.set_tx(0); // no timeout: transmit until done
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

    /// Retune the carrier (Hz). Re-runs image calibration for the 902-928 US band.
    pub fn set_frequency(&mut self, hz: u32) {
        self.calibrate_image(0xE1, 0xE9);
        self.set_rf_freq(hz);
    }

    /// Set LoRa modulation: spreading factor (5-12), bandwidth code, coding-rate code.
    pub fn set_modulation(&mut self, sf: u8, bw: u8, cr: u8) {
        let ldro = if sf >= 11 && bw == BW_125 { 1 } else { 0 };
        self.set_mod_params(sf, bw, cr, ldro);
    }

    /// Set TX power in dBm (SX1262: up to +22), also re-optimising the PA config for the power band
    /// (Semtech DS §13.1.14) so lower powers are efficient instead of using the fixed +22 dBm PA setup.
    pub fn set_power(&mut self, dbm: i8) {
        // (paDutyCycle, hpMax) tiers; txParams power then fine-tunes within the band.
        let (duty, hp) = match dbm {
            d if d >= 20 => (0x04, 0x07), // +22 dBm optimal
            d if d >= 15 => (0x03, 0x05), // +20 dBm optimal
            d if d >= 10 => (0x02, 0x03), // +17 dBm optimal
            _ => (0x02, 0x02),            // +14 dBm optimal
        };
        self.set_pa_config(duty, hp);
        self.set_tx_params(dbm, 0x04);
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

    /// Set the LoRa sync word using the SX127x single-byte convention (0x12 private / 0x34 public).
    pub fn set_sync(&mut self, sx127x_sync: u8) {
        self.set_sync_word(sx127x_sync);
    }

    /// Read the current LoRa sync-word registers (0x1424 == SX127x 0x12).
    pub fn read_sync(&mut self) -> u16 {
        let mut b = [0u8; 2];
        self.read_regs(REG_LORA_SYNC_MSB, &mut b);
        ((b[0] as u16) << 8) | b[1] as u16
    }

    /// Arm continuous RX. Call once, then poll with `poll_rx`.
    pub fn start_rx(&mut self) {
        self.rf_rx();
        let pre = self.preamble;
        self.set_pkt_params(pre, 0x00, 0xFF, 0x01, 0x00);
        self.clear_irq(0xFFFF);
        self.set_rx(0xFFFFFF); // continuous
    }

    /// Non-blocking: if a frame arrived, copy it into `out` and return its metadata.
    /// A CRC error is dropped (returns None) after clearing the IRQ so RX stays armed.
    pub fn poll_rx(&mut self, out: &mut [u8]) -> Option<RxPacket> {
        let irq = self.get_irq();
        if irq & IRQ_RX_DONE == 0 {
            return None;
        }
        let crc_err = irq & IRQ_CRC_ERR != 0;
        self.clear_irq(0xFFFF);
        if crc_err {
            return None;
        }
        let mut st = [0u8; 2];
        self.read_cmd(OP_GET_RX_BUF_STATUS, &mut st);
        let len = st[0];
        let ptr = st[1];
        let n = core::cmp::min(len as usize, out.len());
        self.read_buffer(ptr, &mut out[..n]);

        let mut ps = [0u8; 3];
        self.read_cmd(OP_GET_PKT_STATUS, &mut ps);
        // LoRa: rssiPkt = -rssi/2 dBm, snrPkt = (i8)snr / 4 dB.
        let rssi_dbm = -(ps[0] as i16) / 2;
        let snr_db = (ps[1] as i8) as i16 / 4;
        Some(RxPacket { len, rssi_dbm, snr_db })
    }

    // --- #52: carrier sense (CAD), instantaneous RSSI, hardware RNG, preamble ---

    /// Runtime preamble length in symbols (used by `transmit`/`start_rx`).
    pub fn set_preamble(&mut self, preamble: u16) {
        self.preamble = preamble.max(1);
    }

    /// Configure Channel Activity Detection. `sym` = cadSymbolNum code (0..4 → 1/2/4/8/16 symbols);
    /// `det_peak`/`det_min` = detector sensitivity (SF-dependent, tune on air). Exit-mode STDBY.
    pub fn set_cad_params(&mut self, sym: u8, det_peak: u8, det_min: u8) {
        self.cmd(OP_SET_CAD_PARAMS, &[sym, det_peak, det_min, 0x00, 0x00, 0x00, 0x00]);
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
        self.set_dio_irq(0xFFFF, IRQ_TX_DONE | IRQ_RX_DONE | IRQ_TIMEOUT);
        u32::from_be_bytes(b)
    }
}

/// LoRa time-on-air in whole milliseconds (rounded up). Standard Semtech formula, explicit header +
/// CRC on. Used to tell the host the real airtime so a fixed command timeout does not blow at high SF.
pub fn airtime_ms(sf: u8, bw_code: u8, cr: u8, payload_len: u8, preamble: u16) -> u32 {
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
    let mut steps = if num <= 0 || den <= 0 { 0 } else { (num + den - 1) / den };
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
