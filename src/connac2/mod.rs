//! Shared **connac2** layer — the register map, USB transport, MCU protocol and
//! MAC descriptors of MediaTek's MT7921/MT792x generation.
//!
//! Separate from [`crate::mt76`] because connac2 is a different architecture, not
//! a newer revision of one. The differences that matter here:
//!
//! * Register access is `MT_VEND_READ_EXT` (0x63) / `MT_VEND_WRITE_EXT` (0x66),
//!   not the `MULTI_READ`/`MULTI_WRITE` 0x07/0x06 the mt76x0/mt76x2 parts use.
//! * The device is **composite**: interfaces 0-2 are a Bluetooth radio owned by
//!   `btusb`, and only the class `ff/ff/ff` interface is ours.
//! * Firmware is a **patch + RAM-code pair** driven over MCU commands, with the
//!   RAM image's region table in a trailer at the *end* of the file — the
//!   opposite of the mt76x0/mt76x2 header-at-the-front layout.
//! * ★ The RX descriptor is **variable-length**, and its group 2 carries a
//!   per-frame hardware timestamp (`mt7921/mac.c:307-309`). No mt76x0 or mt76x2
//!   part has one, which is the whole reason this generation is worth porting:
//!   it is the first MediaTek radio in this crate that can source a
//!   [`ndn_radio_hal::RadioClockKind::FreeRunRxStamp`], and therefore common view.
//!
//! MEASURED on mds-o5p-3's MT7921AU, 2026-08-27, before a line of this was
//! written: `MT_HW_CHIPID` (0x70010200) = `0x7961`, `MT_HW_REV` (0x70010204) =
//! `0x8a10` — which equals the `hw_sw_ver` in the patch blob's header, so blob
//! and silicon are a matched pair. EP0 round trip 268 µs on that USB 2.0 bus.

pub mod mac;
pub mod mcu;
pub mod regs;
pub mod usb;
