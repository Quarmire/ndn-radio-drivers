//! Raw SX1276 register access, sharing the one SPI bus with `lora-phy`.
//!
//! # Why this exists
//!
//! `lora-phy` takes ownership of the `SpiDevice` and keeps `read_register`/`write_register` private,
//! so a firmware built only on its API can never read a register it does not model. That is what
//! forced the old firmware to *fabricate* half of `EVT_INFO` (bug C4) and to answer `CMD_GET_RSSI`
//! with a hardcoded `0` — the SX1276 has all of those registers, the crate just does not expose
//! them.
//!
//! [`SharedSpi`] is a two-handle `SpiDevice`: one handle is moved into `lora_phy::sx127x::Sx127x`,
//! the other stays with the firmware for raw register reads. Both point at the same
//! `RefCell<Bus>`, which owns the SPI peripheral *and* the NSS line, so a transaction from either
//! handle is a complete, correctly-framed SPI transaction.
//!
//! ## Why a `RefCell` and not a mutex
//!
//! The firmware is a **single embassy task** for everything that touches SPI. The one `select` in
//! the main loop races the host-command channel and the DIO0 level wait — neither touches SPI — so
//! two SPI transactions can never be in flight at once and the `RefCell` can never be doubly
//! borrowed. Stated as an invariant so a future edit knows what it must preserve:
//!
//! > **INVARIANT.** Every `SharedSpi::transaction` (whether ours or `lora-phy`'s) is awaited from
//! > the main task, and no future holding one is ever raced against another that also touches the
//! > radio.
//!
//! (The UART reader task never touches SPI; it only owns `UART0`'s RX half.)

use core::cell::RefCell;

use embedded_hal_async::spi::{ErrorType, Operation, SpiBus, SpiDevice};
use esp_hal::gpio::Output;
use esp_hal::spi::master::Spi;
use esp_hal::Async;

/// The SPI bus plus the NSS line the SX1276 hangs off. Owned by one `RefCell`, shared by handle.
pub struct Bus {
    pub spi: Spi<'static, Async>,
    pub cs: Output<'static>,
}

/// A cheap, copyable `SpiDevice` handle onto the shared [`Bus`].
#[derive(Clone, Copy)]
pub struct SharedSpi(pub &'static RefCell<Bus>);

impl ErrorType for SharedSpi {
    type Error = esp_hal::spi::Error;
}

impl SpiDevice<u8> for SharedSpi {
    async fn transaction(&mut self, ops: &mut [Operation<'_, u8>]) -> Result<(), Self::Error> {
        let mut guard = self.0.borrow_mut();
        let Bus { spi, cs } = &mut *guard;
        cs.set_low();
        let mut res = Ok(());
        for op in ops.iter_mut() {
            res = match op {
                Operation::Read(buf) => SpiBus::read(spi, buf).await,
                Operation::Write(buf) => SpiBus::write(spi, buf).await,
                Operation::Transfer(r, w) => SpiBus::transfer(spi, r, w).await,
                Operation::TransferInPlace(buf) => SpiBus::transfer_in_place(spi, buf).await,
                // `DelayNs` inside a transaction keeps NSS asserted, which is what the trait wants.
                Operation::DelayNs(ns) => {
                    embassy_time::Timer::after(embassy_time::Duration::from_nanos(*ns as u64)).await;
                    Ok(())
                }
            };
            if res.is_err() {
                break;
            }
        }
        let flushed = SpiBus::flush(spi).await;
        cs.set_high();
        res.and(flushed)
    }
}

// --- SX1276 registers this firmware reads/writes directly (datasheet rev.7 §6.4 register map) ---
//
// Only registers lora-phy leaves unreachable are listed; everything else goes through `RadioKind`.
/// Operating mode + LongRangeMode bit. The closest real analogue the SX1276 has to the SX1262's
/// `GetStatus()` byte, which is what `EVT_INFO[0]` carries on the Waveshare node.
pub const REG_OP_MODE: u8 = 0x01;
/// Modem status: bit0 signal-detected, bit1 signal-synchronised, bit2 RX-ongoing, bit3
/// header-info-valid, bit4 modem-clear.
pub const REG_MODEM_STAT: u8 = 0x18;
/// Current wideband RSSI, valid **only while the modem is in an RX mode**.
pub const REG_RSSI_VALUE: u8 = 0x1B;
/// LoRa sync word. 0x12 = private, 0x34 = public (LoRaWAN).
pub const REG_SYNC_WORD: u8 = 0x39;

// -------------------------------------------------------------------------------------------------
// The receive-gain register (CMD_SET_RX_GAIN, 0x1C)
// -------------------------------------------------------------------------------------------------

/// **`RegLna` (0x0C)** — datasheet rev.7 §6.4, Table 43 "Registers Summary":
///
/// ```text
///  7:5  LnaGain     001 = G1 MAXIMUM gain (power-on default) ... 110 = G6 minimum gain.
///                   000 and 111 are RESERVED and must never be written.
///  4:3  LnaBoostLf  low-frequency-port LNA current, 00 = default. Structurally irrelevant here:
///                   this is the 915 MHz Heltec V2, so the part runs entirely on the HF port.
///  2    reserved
///  1:0  LnaBoostHf  00 = default LNA current, 11 = boost on (150% LNA current, ~+3 dB sensitivity)
/// ```
///
/// `lora-phy` writes this register itself on **every** `do_rx` and `do_cad`, from its
/// `Sx127xConfig::rx_boost` flag, which is fixed at construction. That is why the host-set gain has
/// to be re-applied after each arm rather than written once — see `Radio::arm_rx` in `main.rs`.
pub const REG_LNA: u8 = 0x0C;

/// `RegLna` with LnaGain = G1 (maximum gain) and LnaBoostHf = 00. Byte-identical to `lora-phy`'s
/// `LnaGain::G1.value()`, so "power-saving" here means exactly what `rx_boost: false` means there.
pub const LNA_MAX_GAIN: u8 = 0x20;
/// LnaBoostHf = 11. `lora-phy`'s `LnaGain::boosted_value()` is `value() | 0x03`, the same two bits.
pub const LNA_BOOST_HF: u8 = 0x03;

/// The two `RegLna` bytes `CMD_SET_RX_GAIN` selects between, matching the Waveshare node's boolean
/// payload (`0` = the chip's power-saving default, anything else = boosted).
///
/// The SX1276's 6-step `LnaGain` field (G1..G6) is deliberately **not** exposed: the fleet opcode's
/// payload is one boolean byte and inventing a second byte on this node alone would make 0x1C mean
/// different things on different nodes — the exact divergence `EVT_CAP`'s bitmap exists to prevent.
pub const fn lna_reg(boosted: bool) -> u8 {
    if boosted {
        LNA_MAX_GAIN | LNA_BOOST_HF
    } else {
        LNA_MAX_GAIN
    }
}

// -------------------------------------------------------------------------------------------------
// The CAD/preamble detector registers (CMD_SET_CAD_CFG, 0x0A)
// -------------------------------------------------------------------------------------------------

/// **`RegDetectOptimize` (0x31)** — datasheet rev.7 Table 45.
///
/// ```text
///  7    AutomaticIFOn  reserved; the errata requires 0
///  6:3  reserved       must be preserved read-modify-write (this is what DET_OPT_KEEP masks)
///  2:0  DetectionOptimize  0x03 = SF7..SF12, 0x05 = SF6.  Those two are the ONLY legal codes.
/// ```
pub const REG_DETECT_OPTIMIZE: u8 = 0x31;

/// **`RegDetectionThreshold` (0x37)** — datasheet rev.7 Table 45: an 8-bit LoRa detection threshold,
/// specified as 0x0A for SF7..SF12 and 0x0C for SF6. This is the SX1276's one detector-sensitivity
/// knob and the direct analogue of the SX1262 `SetCadParams` field `cadDetPeak` (both are the
/// correlation threshold the detector compares a peak against).
pub const REG_DETECTION_THRESHOLD: u8 = 0x37;

/// `DetectionOptimize` for SF7..SF12 — the only value legal for this firmware's SF span.
pub const DET_OPT_SF7_12: u8 = 0x03;
/// `DetectionOptimize` for SF6. Unreachable through `CMD_SET_MOD` here (SF6 needs an implicit
/// header and is excluded), but it is the other datasheet-legal code, so the clamp admits it.
pub const DET_OPT_SF6: u8 = 0x05;
/// `DetectionOptimize` occupies bits 2:0.
pub const DET_OPT_MASK: u8 = 0x07;
/// Bits 6:3 of `RegDetectOptimize` are reserved and must survive a write. This is byte-for-byte the
/// mask `lora-phy` uses (`sx127x/mod.rs`: `(reg_val & 0b0111_1000) | opt`), so a value written here
/// leaves the register in exactly the shape the driver would have left it in.
pub const DET_OPT_KEEP: u8 = 0b0111_1000;

/// `DetectionThreshold` for SF7..SF12 (datasheet) — also the register's power-on default.
pub const DET_THRESH_SF7_12: u8 = 0x0A;
/// `DetectionThreshold` for SF6 (datasheet).
pub const DET_THRESH_SF6: u8 = 0x0C;

/// Clamp a host-supplied detector threshold into the datasheet's legal span.
///
/// The datasheet names exactly two values, 0x0A and 0x0C; the closed interval they span is what is
/// accepted, and anything outside is pulled to the nearer end. The clamp is NOT cosmetic: the
/// SX1262 `cadDetPeak` scale a fleet host works in runs around 0x18-0x25, which on this chip is a
/// threshold so high the demodulator stops detecting preambles at all. A clamped write is reported
/// in an `EVT_LOG` by the caller, so it is never silent.
pub const fn clamp_det_thresh(v: u8) -> u8 {
    if v < DET_THRESH_SF7_12 {
        DET_THRESH_SF7_12
    } else if v > DET_THRESH_SF6 {
        DET_THRESH_SF6
    } else {
        v
    }
}

/// Clamp a host-supplied value onto the two legal `DetectionOptimize` codes.
///
/// ⚠ **This field is NOT the SX1262's `cadDetMin`.** `cadDetMin` is a minimum-symbol-recognition
/// count; the SX1276 has no such counter and no register that holds one. `DetectionOptimize` is the
/// other half of the datasheet's detector pair — an SF-class selector. It is driven from the
/// `det_min` byte because it is the only remaining CAD-detector register on this chip, and a host
/// that literally means "minimum symbols" gets nothing it asked for. That is stated here and in the
/// `CMD_SET_CAD_CFG` handler rather than papered over with a scaling formula that would be invented.
///
/// The fleet default `cad_min = 0x0A` lands on `DET_OPT_SF7_12`, which is the datasheet value for
/// this firmware's SF span — i.e. a no-op, which is the right outcome for a value that means
/// nothing here.
pub const fn clamp_det_opt(v: u8) -> u8 {
    if v == DET_OPT_SF6 {
        DET_OPT_SF6
    } else {
        DET_OPT_SF7_12
    }
}

/// **The table test that pins the detector and LNA encodings**, checked by the compiler on every
/// build. `#[cfg(test)]` cannot do this job — the crate is `no_std`/`no_main` for
/// `xtensa-esp32-none-elf`, so `cargo test` never builds it — but a `const` block is evaluated
/// during the real firmware build. Same technique as the bandwidth table in `main.rs`.
const _: () = {
    // RegLna: the boolean payload maps onto lora-phy's own two bytes, and nothing else moves.
    assert!(lna_reg(false) == 0x20); // == LnaGain::G1.value()
    assert!(lna_reg(true) == 0x23); // == LnaGain::G1.boosted_value()
    assert!(lna_reg(false) & 0b1110_0000 == 0x20); // gain field stays G1 = maximum gain
    assert!(lna_reg(true) & 0b1110_0000 == 0x20);
    assert!(lna_reg(false) & 0b0000_0011 == 0x00); // LnaBoostHf off
    assert!(lna_reg(true) & 0b0000_0011 == 0x03); // LnaBoostHf on (150% LNA current)
    // Neither reserved LnaGain encoding (000, 111) can ever be produced.
    assert!(lna_reg(false) & 0b1110_0000 != 0x00);
    assert!(lna_reg(false) & 0b1110_0000 != 0xE0);

    // RegDetectionThreshold: the datasheet's two values pass through; everything else is pulled in.
    assert!(clamp_det_thresh(DET_THRESH_SF7_12) == 0x0A);
    assert!(clamp_det_thresh(DET_THRESH_SF6) == 0x0C);
    assert!(clamp_det_thresh(0x0B) == 0x0B); // inside the span the two named values bound
    assert!(clamp_det_thresh(0x00) == 0x0A);
    assert!(clamp_det_thresh(0x09) == 0x0A);
    assert!(clamp_det_thresh(0x0D) == 0x0C);
    assert!(clamp_det_thresh(0x18) == 0x0C); // the Waveshare's cad_peak default, out of scale here
    assert!(clamp_det_thresh(0xFF) == 0x0C);

    // RegDetectOptimize: only the two datasheet codes exist, and the reserved bits are never in the
    // value the clamp returns (they come from the read-modify-write in `main.rs`).
    assert!(clamp_det_opt(DET_OPT_SF7_12) == 0x03);
    assert!(clamp_det_opt(DET_OPT_SF6) == 0x05);
    assert!(clamp_det_opt(0x0A) == 0x03); // the Waveshare's cad_min default -> the SF7-12 value
    assert!(clamp_det_opt(0x00) == 0x03);
    assert!(clamp_det_opt(0xFF) == 0x03);
    assert!(clamp_det_opt(0x0D) == 0x03); // masks to 5 but is not 5: strict equality, not a mask
    assert!(clamp_det_opt(0xFF) & !DET_OPT_MASK == 0); // never sets a reserved bit itself
    assert!(DET_OPT_KEEP & DET_OPT_MASK == 0); // the keep-mask and the field cannot overlap
    assert!(DET_OPT_KEEP & 0x80 == 0); // AutomaticIFOn is cleared, per the errata
};

// -------------------------------------------------------------------------------------------------
// C1 — intra-packet frequency hopping (`CMD_SET_HOP`, 0x1E)
// -------------------------------------------------------------------------------------------------
//
// ## Why every one of these registers is here rather than in `lora-phy`
//
// `lora-phy` 3.0.1 knows the hop interrupt EXISTS — `IrqMask::FhssChangedChannel = 0x02` and the
// DIO comment block in `sx127x/radio_kind_params.rs:63-87` both name it — and exposes **no hopping
// API at all**: no `RegHopPeriod`, no hop list, no way to reach `RegFrf` outside `set_channel`, and
// `set_irq_params` MASKS `FhssChangedChannel` in every single radio mode. So the hop path is
// unreachable through the crate by construction, and this is exactly the seam the raw
// [`SharedSpi`] handle exists for.
//
// ## ⚠ A bug in the crate's own DIO table — do not use `DioMapping1Dio1`
//
// `RegDioMapping1` (0x40) is, per the SX1276 datasheet rev.7 Table 18:
//
// ```text
//   7:6  Dio0Mapping    00 RxDone       01 TxDone             10 CadDone
//   5:4  Dio1Mapping    00 RxTimeout    01 FhssChangeChannel  10 CadDetected
//   3:2  Dio2Mapping    00/01/10 all three encodings = FhssChangeChannel
//   1:0  Dio3Mapping    00 CadDone      01 ValidHeader        10 PayloadCrcError
// ```
//
// `lora-phy`'s `DioMapping1Dio0` (bits 7:6) and `DioMapping1Dio3` (bits 1:0) are correct, and its
// prose comment lists the four pins correctly — but its `DioMapping1Dio1` enum encodes the values
// at `0b01 << 2` with mask `0xf3`, i.e. in **bits 3:2, the DIO2 field**. It is `#[allow(dead_code)]`
// and the crate never uses it, so the error has never bitten anyone; using it here would have
// mapped DIO2 while wiring DIO1. [`DIO1_FHSS_CHANGE_CHANNEL`] below is at the datasheet's bits 5:4.

/// **`RegIrqFlagsMask` (0x11)** — a 1 masks the corresponding interrupt, i.e. stops it reaching a
/// DIO pin. `lora-phy` rewrites this register wholesale from the radio mode on every
/// `set_irq_params`, and `FhssChangedChannel` is masked in *all* of its arms, so the bit has to be
/// cleared again after every arm (see `Radio::apply_hop` in `main.rs`).
pub const REG_IRQ_FLAGS_MASK: u8 = 0x11;
/// **`RegIrqFlags` (0x12)** — write-1-to-clear. Writing a 0 to a bit leaves it alone, which is the
/// property the hop service depends on: it clears ONLY [`IRQ_FHSS_CHANGE_CHANNEL`] and a latched
/// RxDone/TxDone in the same register survives to be consumed by `lora-phy`'s `process_irq_event`.
pub const REG_IRQ_FLAGS: u8 = 0x12;
/// `FhssChangeChannel` in both `RegIrqFlags` and `RegIrqFlagsMask`. Byte-identical to
/// `lora_phy::sx127x::radio_kind_params::IrqMask::FhssChangedChannel`.
pub const IRQ_FHSS_CHANGE_CHANNEL: u8 = 0x02;

/// **`RegHopChannel` (0x1C)** — read-only hop status:
///
/// ```text
///  7    PllTimeout        1 = the PLL did not lock in time on the last hop (the hop was missed)
///  6    CrcOnPayload      the CRC bit of the last received header
///  5:0  FhssPresentChannel  the hop counter the modem is on, 0..63
/// ```
pub const REG_HOP_CHANNEL: u8 = 0x1C;
/// `RegHopChannel` bit 7, PllTimeout.
pub const HOP_CHANNEL_PLL_TIMEOUT: u8 = 0x80;
/// `RegHopChannel` bits 5:0, FhssPresentChannel.
pub const HOP_CHANNEL_MASK: u8 = 0x3F;

/// **`RegHopPeriod` (0x24)** — symbol periods between hops. **0 disables hopping**, which is both
/// the power-on default and how `CMD_SET_HOP` turns the feature off. `lora-phy` has no entry for
/// this register at all, so nothing in the crate can disturb a value written here.
pub const REG_HOP_PERIOD: u8 = 0x24;

/// **`RegDioMapping1` (0x40)** — see the table above.
pub const REG_DIO_MAPPING1: u8 = 0x40;
/// Everything in `RegDioMapping1` EXCEPT the DIO1 field (bits 5:4), for the read-modify-write that
/// maps DIO1 without disturbing the DIO0 / DIO2 / DIO3 fields `lora-phy` owns.
pub const DIO1_KEEP: u8 = 0b1100_1111;
/// DIO1 = FhssChangeChannel (bits 5:4 = 01).
pub const DIO1_FHSS_CHANGE_CHANNEL: u8 = 0b01 << 4;
/// DIO1 = RxTimeout (bits 5:4 = 00) — the power-on default, and what mapping DIO1 to the hop
/// interrupt DISPLACES. Nothing in this firmware uses it: `RxTimeout` is raised only in
/// `RxMode::Single`, and every receive here is `RxMode::Continuous`.
pub const DIO1_RX_TIMEOUT: u8 = 0b00 << 4;

/// The three carrier registers, written in this order (MSB first) exactly as `lora-phy`'s
/// `set_channel` does. Three separate single-byte-address transactions, NOT one auto-incrementing
/// burst: register burst access is not something this firmware has verified on this part, and
/// getting it wrong would write all three bytes into `RegFrfMsb` — a hop to a wildly wrong carrier.
/// The cost of being sure is 16 µs of extra SPI byte time per hop.
pub const REG_FRF_MSB: u8 = 0x06;
pub const REG_FRF_MID: u8 = 0x07;
pub const REG_FRF_LSB: u8 = 0x08;

/// SX1276 crystal, from `lora_phy`'s own comment on `FREQUENCY_SYNTHESIZER_STEP`:
/// "FXOSC (32 MHz) * 1000000 (Hz/MHz) / 524288 (2^19)".
pub const FXOSC_HZ: u64 = 32_000_000;

/// The `RegFrf` word for a carrier in Hz: `frf = f_rf * 2^19 / FXOSC`.
///
/// `lora-phy` computes the same quantity as `(hz as f64 / 61.03515625) as u32`. This is the exact
/// integer form, evaluated at `CMD_SET_HOP` time so that **no arithmetic at all** — least of all a
/// soft-float divide on a chip with no double-precision FPU — runs inside the hop interrupt
/// service. The two forms agree to at worst one LSB (61.035 Hz, 6.7e-8 of a 915 MHz carrier), far
/// inside any LoRa demodulator's frequency tolerance.
pub const fn frf_of_hz(hz: u32) -> u32 {
    (((hz as u64) << 19) / FXOSC_HZ) as u32
}

/// **The FHSS table test**, checked by the compiler on every build (same technique as the detector
/// table above: `#[cfg(test)]` never runs on this `no_std`/`no_main` xtensa target).
const _: () = {
    // The DIO1 field and the bits lora-phy owns cannot overlap, in either direction.
    assert!(DIO1_KEEP & 0b0011_0000 == 0); // the keep-mask never covers DIO1's own field
    assert!(DIO1_FHSS_CHANGE_CHANNEL == 0x10);
    assert!(DIO1_FHSS_CHANGE_CHANNEL & DIO1_KEEP == 0); // the value only touches bits 5:4
    assert!(DIO1_RX_TIMEOUT == 0x00);
    // lora-phy's DIO0 mask is 0x3f and its DIO3 mask is 0xfc; the bits they preserve must include
    // ours, or an arm would silently un-map the hop interrupt.
    assert!(0x3f & 0b0011_0000 == 0b0011_0000); // Transmit / CAD arms keep bits 5:0
    assert!(0x3f & 0xfc & 0b0011_0000 == 0b0011_0000); // Receive arm keeps bits 5:2
    // The hop-status field decomposition covers the whole byte with no overlap.
    assert!(HOP_CHANNEL_PLL_TIMEOUT & HOP_CHANNEL_MASK == 0);
    assert!(IRQ_FHSS_CHANGE_CHANNEL == 0x02); // == IrqMask::FhssChangedChannel

    // frf_of_hz against the three carriers this board is matched for. 915 MHz -> 0xE4C000 is the
    // canonical SX127x value, and the band edges bracket every frequency CMD_SET_HOP will accept.
    assert!(frf_of_hz(915_000_000) == 0x00E4_C000);
    assert!(frf_of_hz(902_000_000) == 0x00E1_8000);
    assert!(frf_of_hz(928_000_000) == 0x00E8_0000);
    // The identity the formula reduces to, 2^19 / 32e6 == 2^14 / 1e6, on a non-round frequency.
    assert!(frf_of_hz(903_500_000) == (903_500_000u64 * 16_384 / 1_000_000) as u32);
    // A hop word is 24 bits wide: three registers, and nothing above bit 23 may be dropped.
    assert!(frf_of_hz(928_000_000) <= 0x00FF_FFFF);
};

/// SX1276 RSSI offsets, split at the LF/HF port boundary. Same source constants `lora-phy` uses for
/// its per-packet RSSI (`sx127x/mod.rs`: `SX1276_RSSI_OFFSET_LF/HF`, `SX1276_RF_MID_BAND_THRESH`),
/// so an instantaneous RSSI and a per-packet RSSI from this node are on the same scale.
const RSSI_OFFSET_LF_DBM: i16 = -164;
const RSSI_OFFSET_HF_DBM: i16 = -157;
const RF_MID_BAND_THRESH_HZ: u32 = 525_000_000;

impl SharedSpi {
    /// Read one register. SX1276 SPI: address byte with bit7 = 0 selects a read.
    pub async fn read_reg(&mut self, addr: u8) -> Result<u8, esp_hal::spi::Error> {
        let mut buf = [addr & 0x7F, 0x00];
        self.transaction(&mut [Operation::TransferInPlace(&mut buf)])
            .await?;
        Ok(buf[1])
    }

    /// Write one register. Address byte with bit7 = 1 selects a write.
    pub async fn write_reg(&mut self, addr: u8, val: u8) -> Result<(), esp_hal::spi::Error> {
        self.transaction(&mut [Operation::Write(&[addr | 0x80, val])])
            .await
    }

    /// Point the CAD/preamble detector at a host-chosen `(DetectionOptimize, DetectionThreshold)`
    /// pair and return the **full** `RegDetectOptimize` byte that was written.
    ///
    /// The read-modify-write happens once, here, so that re-applying the pair after every
    /// `set_modulation_params` (which `lora-phy` uses to overwrite both registers from the SF) costs
    /// two plain writes and no read. That matters: `arm_rx` is on the 5.6 ms retune path.
    pub async fn set_detector(
        &mut self,
        opt: u8,
        thresh: u8,
    ) -> Result<u8, esp_hal::spi::Error> {
        let cur = self.read_reg(REG_DETECT_OPTIMIZE).await?;
        let full = (cur & DET_OPT_KEEP) | (opt & DET_OPT_MASK);
        self.write_reg(REG_DETECT_OPTIMIZE, full).await?;
        self.write_reg(REG_DETECTION_THRESHOLD, thresh).await?;
        Ok(full)
    }

    /// Re-write a previously computed detector pair. No read: `full_opt` already carries the
    /// reserved bits 6:3 that [`SharedSpi::set_detector`] preserved.
    pub async fn reapply_detector(
        &mut self,
        full_opt: u8,
        thresh: u8,
    ) -> Result<(), esp_hal::spi::Error> {
        self.write_reg(REG_DETECT_OPTIMIZE, full_opt).await?;
        self.write_reg(REG_DETECTION_THRESHOLD, thresh).await
    }

    /// Instantaneous channel RSSI in **dBm**.
    ///
    /// `RegRssiValue` is a raw wideband reading that is only meaningful while the modem is actually
    /// receiving — the caller must have the chip in RX (this firmware re-arms RX before every sense,
    /// see `sense_rssi_dbm` in `main.rs`). Returns dBm, never a register unit, so it can be compared
    /// against a host-set threshold in dBm.
    pub async fn rssi_inst_dbm(&mut self, freq_hz: u32) -> Result<i16, esp_hal::spi::Error> {
        let raw = self.read_reg(REG_RSSI_VALUE).await?;
        let offset = if freq_hz > RF_MID_BAND_THRESH_HZ {
            RSSI_OFFSET_HF_DBM
        } else {
            RSSI_OFFSET_LF_DBM
        };
        Ok(offset + raw as i16)
    }

    /// **Map DIO1 to the hop interrupt** (C1) and return the full `RegDioMapping1` byte written.
    ///
    /// Read-modify-write, because bits 7:6 (DIO0) and 1:0 (DIO3) belong to `lora-phy` and are
    /// rewritten by it on every `set_irq_params`. Only bits 5:4 are ours. Called ONCE, from the
    /// `CMD_SET_HOP` handler: `lora-phy`'s three DIO writes all mask bits 5:4 through (`& 0x3f`
    /// in the Transmit/CAD arms, `& 0x3f & 0xfc` in the Receive arm), so the mapping survives
    /// every arm this firmware performs and never has to be re-applied.
    pub async fn map_dio1(&mut self, field: u8) -> Result<u8, esp_hal::spi::Error> {
        let cur = self.read_reg(REG_DIO_MAPPING1).await?;
        let full = (cur & DIO1_KEEP) | (field & !DIO1_KEEP);
        self.write_reg(REG_DIO_MAPPING1, full).await?;
        Ok(full)
    }

    /// **Let the hop interrupt reach DIO1**, and drop any stale latched hop flag.
    ///
    /// Two things, because `lora-phy`'s `set_irq_params` undoes the first on every arm: it writes
    /// `RegIrqFlagsMask` wholesale from the radio mode and masks `FhssChangedChannel` in all four
    /// of its arms. The read-modify-write is what keeps the RxDone/TxDone/CRC/HeaderValid mask bits
    /// the driver just chose. The trailing `RegIrqFlags <- 0x02` is write-1-to-clear on that one
    /// bit only, so a latched RxDone or TxDone in the same register is untouched and still reaches
    /// `process_irq_event`.
    ///
    /// Three SPI transactions, 6 bytes, ~24 µs of byte time at 2 MHz — and issued ONLY when the
    /// host has actually enabled hopping (see `Radio::apply_hop`).
    pub async fn unmask_hop_irq(&mut self) -> Result<(), esp_hal::spi::Error> {
        // Clear BEFORE unmasking, never after: a flag left latched from the previous packet would
        // otherwise drive DIO1 high the instant the mask lifts, and the main loop would service a
        // hop that never happened. In this order the only thing that can raise DIO1 after the
        // second write is a real hop.
        self.write_reg(REG_IRQ_FLAGS, IRQ_FHSS_CHANGE_CHANNEL).await?;
        let cur = self.read_reg(REG_IRQ_FLAGS_MASK).await?;
        self.write_reg(REG_IRQ_FLAGS_MASK, cur & !IRQ_FHSS_CHANGE_CHANNEL)
            .await
    }

    /// Re-mask the hop interrupt and clear its flag — the disable half of [`Self::unmask_hop_irq`],
    /// so that turning hopping off cannot leave DIO1 latched high and spin the main loop.
    pub async fn mask_hop_irq(&mut self) -> Result<(), esp_hal::spi::Error> {
        let cur = self.read_reg(REG_IRQ_FLAGS_MASK).await?;
        self.write_reg(REG_IRQ_FLAGS_MASK, cur | IRQ_FHSS_CHANGE_CHANNEL)
            .await?;
        self.write_reg(REG_IRQ_FLAGS, IRQ_FHSS_CHANGE_CHANNEL).await
    }

    /// **Retune the carrier from a precomputed `RegFrf` word** — the one write the hop interrupt
    /// service exists to perform. Same three registers, same order (MSB, MID, LSB) as `lora-phy`'s
    /// `set_channel`; the difference is that the word was computed by [`frf_of_hz`] when the host
    /// set the hop list, so this path does no arithmetic whatsoever.
    pub async fn write_frf(&mut self, frf: u32) -> Result<(), esp_hal::spi::Error> {
        self.write_reg(REG_FRF_MSB, (frf >> 16) as u8).await?;
        self.write_reg(REG_FRF_MID, (frf >> 8) as u8).await?;
        self.write_reg(REG_FRF_LSB, frf as u8).await
    }
}
