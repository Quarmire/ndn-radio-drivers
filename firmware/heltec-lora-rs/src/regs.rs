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
}
