//! The **MT76x0U in-band MCU transport** — how the host talks to the on-chip
//! MCU over the bulk pipes, and how the firmware gets there in the first place.
//!
//! Three things live here, in dependency order:
//!   1. [`mcu_send`] — one in-band command: a 4-byte info word, the payload, a
//!      4-byte trailer, out on the inband-command bulk OUT endpoint, and
//!      (optionally) a sequence-matched response off the cmd-resp bulk IN.
//!   2. [`wr_rp`] / [`rd_rp`] — the **register-pair** protocol built on top of
//!      it. This is not a nicety: once firmware is running, every MAC/BBP init
//!      table and **every RF-bank access** on the USB part goes through it
//!      (`mt76x0/phy.c:113,131` — `mt76x0_rf_wr`/`_rr` are literally a one-pair
//!      `wr_rp`/`rd_rp` against [`regs::MT_MCU_MEMMAP_RF`]). No firmware, no RF.
//!   3. [`load_firmware`] — the mt76x0u download path (`mt76x0/usb_mcu.c:85`).
//!
//! Ported from the mainline mt76 tree: `mt76x0/usb_mcu.c`, `mt76x0/mcu.h`,
//! `mt76x02_usb_mcu.c`, `mt76x02_mcu.c`, `mt76x02_mcu.h`, `mt76x02_dma.h`,
//! `mt76x02_usb_core.c`, `usb.c`. Every non-obvious constant carries its
//! upstream `file:line`.
//!
//! # MEASURED vs CODE-READ
//!
//! **MEASURED** on mds-o5p-1's MT7610U (`0e8d:7610`, 2026-08-27, via
//! `examples/mt76_oracle.rs`) — these constrain the code below:
//!   * The endpoints this module names are real and are where mt76's enum order
//!     says they are: bulk OUT `0x04` = `MT_EP_OUT_INBAND_CMD` (then AC_BE/BK/
//!     VI/VO/HCCA on `0x05..0x09`), bulk IN `0x84` = packet RX, `0x85` =
//!     `MT_EP_IN_CMD_RESP`. All 512 B, high speed.
//!   * **EP0 vendor-request round trip = 151 µs.** That is the unit cost of
//!     every [`McuBus::rr`] / [`McuBus::wr`] here. It is why the per-chunk
//!     firmware handshake ([`fw_send_chunk`]) is two control writes plus one
//!     read-modify-write and not a poll loop, and why [`wr_rp`] batching 24
//!     pairs into one bulk transfer is worth ~3.6 ms per full table versus
//!     writing them one register at a time.
//!
//! **MEASURED** from the shipped blob (`fw/mt76x0/mt7610u.bin`, 80288 B):
//!   `ilm_len = 68780 (0x10cac)`, `dlm_len = 11476 (0x2cd4)`,
//!   `build_ver = 0x7640`, `fw_ver = 0x0100`, `build_time = "201308221655"`,
//!   and `32 + ilm_len + dlm_len == 80288` exactly — so the header layout in
//!   [`FwHeader`] is confirmed against the real image, not just against the C
//!   struct. Both lengths are 4-aligned, which is why upstream's ragged-tail
//!   padding bug (noted on [`build_fw_frame`]) never fires in practice.
//!
//! **CODE-READ** (ported from upstream, not yet exercised on this silicon):
//! the whole download sequence, the FCE descriptor programming, the IVB hand-off
//! and the readiness poll. Nothing in [`load_firmware`] has been run against the
//! target yet — it is a faithful transcription, and the places where upstream's
//! reasoning is opaque are marked as such rather than rationalised.
//!
//! # There is no ROM patch on this part
//!
//! `mt76x2u_mcu_fw_init` (`mt76x2/usb_mcu.c:235-244`) loads a ROM patch and then
//! the firmware, and activates the patch with two **class**-typed WMT requests.
//! `mt76x0u_mcu_init` (`mt76x0/usb_mcu.c:164-175`) calls `load_firmware` and
//! nothing else. A port that copies the sibling MT7612U backend's `load_rom_patch`
//! onto this part is sending the 7612's `mt7662_rom_patch.bin` to a 7610.
//!
//! # Load offsets: mt76x0 is not mt76x2, and there is no E3 variant here
//!
//! | | ILM dst | DLM dst | E3 adjust |
//! |---|---|---|---|
//! | mt76x2u (`mt76x2/usb_mcu.c:17-18,207-208`) | `0x80000` | `0x110000` | `+0x800` at rev ≥ E3 |
//! | mt76x0u (`mt76x0/usb_mcu.c:32-43`, `mt76x0/mcu.h:14-15`) | **`0x40`** | **`0x80000`** | **none** |
//!
//! So: the contract's `MT76U_MCU_ILM_OFFSET` / `MT76U_MCU_DLM_OFFSET` resolve,
//! **for this part**, to [`regs::MT_MCU_IVB_SIZE`] (`0x40`) and
//! [`regs::MT_MCU_DLM_OFFSET`] (`0x80000`) — the mt76x0 ILM window starts at 0
//! with the 64-byte interrupt-vector block at its base, and that block is *not*
//! DMA'd with the rest: it is split off and handed over separately as the body
//! of vendor request `MT_VEND_DEV_MODE` `wValue=0x12`, which is what starts the
//! CPU. **mt76x0 does not use the E3 DLM variant at all**; `mt76xx_rev()` is
//! never consulted on this path.
//!
//! # The `unwrap_or(false)` trap, and why it is not repeated here
//!
//! The sibling [`crate::mt7612`] backend polls the FCE doorbell with
//! `.unwrap_or(false)`, which makes a *failed control read* indistinguishable
//! from "FCE not ready" — the loop then advances the engine blind and the next
//! chunk's bulk write NAKs. Upstream never polls that register at all: it does a
//! read-increment-write of [`regs::MT_TX_CPU_FROM_FCE_CPU_DESC_IDX`]
//! (`mt76x02_usb_mcu.c:246-248`). This module does exactly that, with `?` on the
//! read, so a dead control pipe surfaces as an error at the chunk that hit it
//! instead of as a mysterious stall 200 chunks later.
#![allow(dead_code)]

use std::time::Duration;

use crate::FaceError;
use crate::mt76::regs;

// ─────────────────────────────────────────────────────────────────────────────
// Errors
// ─────────────────────────────────────────────────────────────────────────────

/// All errors from this module carry the `mt7610u mcu:` prefix so a log line
/// says which of the rig's five USB radios produced it.
fn mcu_err(msg: impl AsRef<str>) -> FaceError {
    FaceError::Io(std::io::Error::other(format!(
        "mt7610u mcu: {}",
        msg.as_ref()
    )))
}

/// Debug tracing for the download path, gated on the same environment variable
/// the MT7612U backend uses so one `NDN_RADIO_EP_DEBUG=1` lights up both.
fn ep_debug() -> bool {
    std::env::var("NDN_RADIO_EP_DEBUG").is_ok()
}

/// Debug tracing for individual MCU commands (`NDN_RADIO_MCU_DEBUG=1`), again
/// matching the MT7612U backend.
fn mcu_debug() -> bool {
    std::env::var("NDN_RADIO_MCU_DEBUG").is_ok()
}

// ─────────────────────────────────────────────────────────────────────────────
// USB vendor requests used by this module
// ─────────────────────────────────────────────────────────────────────────────

/// `MT_VEND_DEV_MODE` (mt76.h:632). Three distinct jobs on this path, told
/// apart purely by `wValue`: `0x1` = firmware reset
/// (`mt76x02_usb_mcu.c:207-212`), `0x12` = load-IVB / start the CPU
/// (`mt76x0/usb_mcu.c:47-49`).
pub const MT_VEND_DEV_MODE: u8 = 0x01;

/// `MT_VEND_WRITE_FCE` (mt76.h:638) — the FCE-region write. The value rides in
/// `wValue` (16 bits at a time) and there is **no data stage**, which is why
/// [`single_wr`] issues two transfers for one 32-bit descriptor field.
pub const MT_VEND_WRITE_FCE: u8 = 0x42;

/// `wValue` for the firmware-reset flavour of [`MT_VEND_DEV_MODE`].
/// `mt76x02_usb_mcu.c:210`
pub const DEV_MODE_FW_RESET: u16 = 0x0001;

/// `wValue` for the load-IVB flavour of [`MT_VEND_DEV_MODE`]. On mt76x0 this
/// request **carries the 64-byte IVB as its data stage** (`mt76x0/usb_mcu.c:47-49`);
/// on mt76x2 the same `wValue` is sent with a NULL body
/// (`mt76x2/usb_mcu.c:23-25`). Sending an empty body here does not start the CPU.
pub const DEV_MODE_LOAD_IVB: u16 = 0x0012;

// ─────────────────────────────────────────────────────────────────────────────
// The bus seam
// ─────────────────────────────────────────────────────────────────────────────

/// What the MCU code needs from a backend, and nothing more.
///
/// Implemented by the `Mt7610uBackend`; everything in this module is written
/// against `&dyn McuBus` so one copy serves the backend, the tests and any
/// future bring-up example without monomorphising.
///
/// **Timeouts are the implementor's business** because they differ per method
/// and upstream picks them deliberately. Use these:
///   * [`bulk_out_cmd`](Self::bulk_out_cmd) — **500 ms** for commands
///     (`mt76x02_usb_mcu.c:95`), **1000 ms** for firmware chunks
///     (`:239`). One timeout of 1000 ms satisfies both.
///   * [`bulk_in_resp`](Self::bulk_in_resp) — **300 ms** (`:45`).
///   * [`vendor_write`](Self::vendor_write) / [`rr`](Self::rr) /
///     [`wr`](Self::wr) — **300 ms** (`usb.c:12`, `MT_VEND_REQ_TOUT_MS`),
///     with up to 10 retries (`usb.c:11`, `MT_VEND_REQ_MAX_RETRY`).
///
/// ⚠ **Never** implement any of these with a USB device reset. A blind
/// `handle.reset()` is what wedges these parts.
pub trait McuBus {
    /// Read a 32-bit MMIO register (`MT_VEND_MULTI_READ`, `usb.c:104`).
    ///
    /// ★ MEASURED 151 µs per round trip. Budget accordingly.
    fn rr(&self, addr: u32) -> Result<u32, FaceError>;

    /// Write a 32-bit MMIO register (`MT_VEND_MULTI_WRITE`, `usb.c:144`).
    fn wr(&self, addr: u32, val: u32) -> Result<(), FaceError>;

    /// Send `buf` on the inband-command bulk OUT endpoint
    /// (`MT_EP_OUT_INBAND_CMD`, mt76.h:653 — MEASURED as `0x04` on this part).
    fn bulk_out_cmd(&self, buf: &[u8]) -> Result<(), FaceError>;

    /// Read one response from the cmd-resp bulk IN endpoint
    /// (`MT_EP_IN_CMD_RESP`, mt76.h:648 — MEASURED as `0x85`). Returns the
    /// number of bytes actually transferred. A USB timeout must surface as an
    /// `Err`; [`mcu_send`] retries a bounded number of times before giving up.
    fn bulk_in_resp(&self, buf: &mut [u8]) -> Result<usize, FaceError>;

    /// Next MCU sequence number, in `1..=15` — **never 0**.
    ///
    /// Upstream increments a `u8` and masks to 4 bits, re-incrementing if the
    /// result is zero (`mt76x02_usb_mcu.c:83-86`), because seq 0 is reserved for
    /// fire-and-forget commands that post no response. Only called when a
    /// response is actually wanted.
    fn next_seq(&self) -> u8;

    /// One host→device **vendor control write** —
    /// `mt76u_vendor_request(dev, request, USB_DIR_OUT | USB_TYPE_VENDOR,
    /// value, index, data, data.len())` (`usb.c:60-72`), i.e.
    /// `bmRequestType = 0x40`.
    ///
    /// ⚠ **This method is an addition to the agreed `McuBus` contract.** It is
    /// unavoidable: three steps of the download reach the device through a
    /// vendor request that is *not* a register write, and none of them can be
    /// expressed with [`rr`](Self::rr) / [`wr`](Self::wr).
    ///   * [`MT_VEND_DEV_MODE`] `wValue = 0x1` — firmware reset, empty body
    ///     (`mt76x02_usb_mcu.c:209-211`).
    ///   * [`MT_VEND_DEV_MODE`] `wValue = 0x12` — load-IVB, **64-byte body**
    ///     (`mt76x0/usb_mcu.c:47-49`).
    ///   * [`MT_VEND_WRITE_FCE`] — the FCE DMA descriptor, value in `wValue`,
    ///     **no body** (`usb.c:219-233` via [`single_wr`]). A plain
    ///     `MT_VEND_MULTI_WRITE` to 0x0230/0x0234 is a different transaction and
    ///     does not program the descriptor.
    fn vendor_write(
        &self,
        request: u8,
        value: u16,
        index: u16,
        data: &[u8],
    ) -> Result<(), FaceError>;
}

/// A 32-bit write through a vendor request that has **no data stage**, split
/// into two 16-bit `wValue` writes at `offset` and `offset + 2`.
///
/// This is `mt76u_single_wr` (`usb.c:219-233`), and it is the only way to reach
/// the FCE DMA descriptor: [`regs::MT_FCE_DMA_ADDR`] / [`regs::MT_FCE_DMA_LEN`]
/// are **not** reachable with an ordinary register write, they need
/// [`MT_VEND_WRITE_FCE`].
fn single_wr(bus: &dyn McuBus, req: u8, offset: u16, val: u32) -> Result<(), FaceError> {
    bus.vendor_write(req, (val & 0xffff) as u16, offset, &[])?;
    bus.vendor_write(req, (val >> 16) as u16, offset + 2, &[])?;
    Ok(())
}

/// `mt76_set` — read, OR in `bits`, write back.
fn reg_set(bus: &dyn McuBus, addr: u32, bits: u32) -> Result<(), FaceError> {
    let v = bus.rr(addr)?;
    bus.wr(addr, v | bits)
}

/// `____mt76_poll_msec` with the default 10 ms tick (mt76.h:1237, `util.c:26-41`):
/// loop until `(rr(addr) & mask) == val`, sleeping 10 ms between reads, for at
/// most `timeout_ms`. Returns whether the condition was met; a **read failure is
/// an error**, not a "no".
fn poll_msec(
    bus: &dyn McuBus,
    addr: u32,
    mask: u32,
    val: u32,
    timeout_ms: u64,
) -> Result<bool, FaceError> {
    const TICK_MS: u64 = 10;
    let mut left = timeout_ms / TICK_MS;
    loop {
        if bus.rr(addr)? & mask == val {
            return Ok(true);
        }
        if left == 0 {
            return Ok(false);
        }
        left -= 1;
        std::thread::sleep(Duration::from_millis(TICK_MS));
    }
}

/// `__mt76_poll` (`util.c:9-24`): same shape, but upstream's tick is a 10 µs
/// `udelay` and its `timeout` argument is in µs, so the real iteration count is
/// `timeout_us / 10`. On USB the 10 µs delay is noise next to the MEASURED
/// 151 µs register read, so only the iteration count is reproduced.
fn poll_usec(
    bus: &dyn McuBus,
    addr: u32,
    mask: u32,
    val: u32,
    timeout_us: u64,
) -> Result<bool, FaceError> {
    let mut left = timeout_us / 10;
    loop {
        if bus.rr(addr)? & mask == val {
            return Ok(true);
        }
        if left == 0 {
            return Ok(false);
        }
        left -= 1;
        std::thread::sleep(Duration::from_micros(10));
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// The in-band command info word
// ─────────────────────────────────────────────────────────────────────────────

// mt76x02_dma.h:32-38 — "MCU request message header". The same 32-bit word is
// also described as the TX descriptor info word (mt76x02_dma.h:12-21) because
// mt76x02u_skb_dma_info() fills LEN and DPORT into the flags the MCU layer
// already built; MT_TXD_INFO_LEN == MT_MCU_MSG_LEN and MT_TXD_INFO_DPORT ==
// MT_MCU_MSG_PORT, bit for bit.

/// `MT_MCU_MSG_LEN` — GENMASK(15, 0). mt76x02_dma.h:33
pub const MT_MCU_MSG_LEN: u32 = 0x0000_ffff;
/// `MT_MCU_MSG_CMD_SEQ` — GENMASK(19, 16). mt76x02_dma.h:34
pub const MT_MCU_MSG_CMD_SEQ: u32 = 0x000f_0000;
/// `MT_MCU_MSG_CMD_TYPE` — GENMASK(26, 20). Seven bits, so a command id above
/// 127 cannot be expressed. mt76x02_dma.h:35
pub const MT_MCU_MSG_CMD_TYPE: u32 = 0x07f0_0000;
/// `MT_MCU_MSG_PORT` — GENMASK(29, 27). mt76x02_dma.h:36
pub const MT_MCU_MSG_PORT: u32 = 0x3800_0000;
/// `MT_MCU_MSG_TYPE` — GENMASK(31, 30). mt76x02_dma.h:37
pub const MT_MCU_MSG_TYPE: u32 = 0xc000_0000;
/// `MT_MCU_MSG_TYPE_CMD` — BIT(30), i.e. `MT_MCU_MSG_TYPE == 1`.
/// mt76x02_dma.h:38
pub const MT_MCU_MSG_TYPE_CMD: u32 = 0x4000_0000;

/// `CPU_TX_PORT` — index 2 of `enum dma_msg_port` (mt76x02_dma.h:43-51:
/// `WLAN_PORT, CPU_RX_PORT, CPU_TX_PORT, …`). Both the command path
/// (`mt76x02_usb_mcu.c:91`) and the firmware-data path (`:223`) use it.
pub const CPU_TX_PORT: u32 = 2;

/// `MT_CMD_HDR_LEN` — the 4-byte info word. mt76x02_usb_mcu.c:13
pub const MT_CMD_HDR_LEN: usize = 4;

/// `MCU_RESP_URB_SIZE` — the cmd-response read buffer. mt76.h:675
pub const MCU_RESP_URB_SIZE: usize = 1024;

/// Round `n` up to the next multiple of 4 (upstream `round_up(x, 4)`).
/// Written as a mask rather than `div_ceil` so it is usable from a `const fn`
/// on the crate's pinned toolchain.
const fn round_up4(n: usize) -> usize {
    (n + 3) & !3
}

/// Build the info word for an in-band **command**.
///
/// Bit layout, exactly (mt76x02_dma.h:33-38, assembled across
/// `mt76x02_usb_mcu.c:88-91` and `mt76x02_usb_core.c:57-58`):
///
/// ```text
///  31 30 | 29 28 27 | 26 ......... 20 | 19 18 17 16 | 15 ................ 0
///  TYPE  |   PORT   |    CMD_TYPE     |   CMD_SEQ   |         LEN
///   0b01 |  0b010   |   cmd (7 bit)   |  seq (4 b)  |  round_up(payload, 4)
/// ```
///
/// * `TYPE = 1` (`MT_MCU_MSG_TYPE_CMD`, BIT(30)) — this is a command, not data.
/// * `PORT = CPU_TX_PORT = 2` — destination port inside the chip.
/// * `CMD_TYPE` — the [`McuCmd`] id. Seven bits.
/// * `CMD_SEQ` — `0` when no response is wanted; otherwise `1..=15`, echoed back
///   in the response's FCE word so the reply can be matched.
/// * `LEN` — **the 4-byte-rounded payload length**, not the raw length. Upstream
///   gets this from `FIELD_PREP(MT_TXD_INFO_LEN, round_up(skb->len, 4))` at
///   `mt76x02_usb_core.c:57`, where `skb->len` is the payload (the info word has
///   not been pushed yet). Note this differs from the firmware-data path — see
///   [`fw_info_word`].
pub const fn mcu_info_word(cmd: u8, seq: u8, payload_len: usize) -> u32 {
    let len = round_up4(payload_len) as u32 & MT_MCU_MSG_LEN;
    MT_MCU_MSG_TYPE_CMD
        | ((CPU_TX_PORT << 27) & MT_MCU_MSG_PORT)
        | (((cmd as u32) << 20) & MT_MCU_MSG_CMD_TYPE)
        | (((seq as u32) << 16) & MT_MCU_MSG_CMD_SEQ)
        | len
}

/// Build the info word for a **firmware-data** transfer.
///
/// Same word, three differences from [`mcu_info_word`]
/// (`mt76x02_usb_mcu.c:223-225`):
///   * no `CMD_SEQ` (firmware chunks are never acknowledged this way),
///   * no `CMD_TYPE` (the FCE routes by descriptor, not by command id),
///   * `LEN` is the **raw** chunk length. Upstream computes `info` at `:223`
///     and only rounds `len` afterwards at `:233`, so the rounding applies to
///     the FCE descriptor and the USB transfer size but *not* to this field.
pub const fn fw_info_word(payload_len: usize) -> u32 {
    MT_MCU_MSG_TYPE_CMD
        | ((CPU_TX_PORT << 27) & MT_MCU_MSG_PORT)
        | (payload_len as u32 & MT_MCU_MSG_LEN)
}

/// Assemble a complete in-band command frame.
///
/// Buffer layout, from the comment at `mt76x02_usb_core.c:48-53`:
///
/// ```text
/// |   4B   | xfer len |      pad       |  4B  |
/// | TXINFO | pkt/cmd  | zero pad to 4B | zero |
/// ```
///
/// Total = `4 + round_up(payload, 4) + 4`. The trailing zero word is the FCE's
/// end-of-packet marker (`mt76_skb_adjust_pad` at `:63` appends
/// `round_up(len,4) + 4 - len` zero bytes, which is the pad *and* the trailer).
pub fn build_cmd_frame(cmd: u8, seq: u8, payload: &[u8]) -> Vec<u8> {
    let padded = round_up4(payload.len());
    let mut buf = Vec::with_capacity(MT_CMD_HDR_LEN + padded + 4);
    buf.extend_from_slice(&mcu_info_word(cmd, seq, payload.len()).to_le_bytes());
    buf.extend_from_slice(payload);
    buf.resize(MT_CMD_HDR_LEN + padded + 4, 0);
    buf
}

/// Assemble one firmware-data frame: `[info][chunk][pad to 4][4 zero bytes]`,
/// total `4 + round_up(len, 4) + 4` — which is exactly the `data_len` upstream
/// hands to the bulk write (`mt76x02_usb_mcu.c:237`).
///
/// ⚠ **Deliberate divergence, in the safe direction.** Upstream zeroes only
/// `4` bytes at `data + 4 + len` (`:229`) but transmits `4 + round_up(len,4) + 4`
/// bytes, so when `len % 4 != 0` it ships up to 3 bytes of uninitialised kmalloc
/// memory in the trailer. This builds the whole tail from zeros. The bug never
/// fires on the shipped image anyway — MEASURED, `ilm_len` and `dlm_len` are
/// both 4-aligned and every chunk but the last is `max_len`, itself 4-aligned.
pub fn build_fw_frame(payload: &[u8]) -> Vec<u8> {
    let padded = round_up4(payload.len());
    let mut buf = Vec::with_capacity(MT_CMD_HDR_LEN + padded + 4);
    buf.extend_from_slice(&fw_info_word(payload.len()).to_le_bytes());
    buf.extend_from_slice(payload);
    buf.resize(MT_CMD_HDR_LEN + padded + 4, 0);
    buf
}

// ─────────────────────────────────────────────────────────────────────────────
// The response FCE word
// ─────────────────────────────────────────────────────────────────────────────

/// `MT_RX_FCE_INFO_CMD_SEQ` — GENMASK(19, 16). mt76x02_dma.h:25
pub const MT_RX_FCE_INFO_CMD_SEQ: u32 = 0x000f_0000;
/// `MT_RX_FCE_INFO_EVT_TYPE` — GENMASK(23, 20). mt76x02_dma.h:26
pub const MT_RX_FCE_INFO_EVT_TYPE: u32 = 0x00f0_0000;

/// `EVT_CMD_DONE` — the first member of `enum mt76_mcu_evt_type`, so `0`.
/// dma.h:154-155
pub const EVT_CMD_DONE: u32 = 0;
/// `EVT_CMD_ERROR` — dma.h:156.
pub const EVT_CMD_ERROR: u32 = 1;
/// `EVT_CMD_RETRY` — dma.h:157.
pub const EVT_CMD_RETRY: u32 = 2;

/// Sequence number echoed in a response's leading FCE word.
/// `mt76x02_usb_mcu.c:56`
pub const fn rxfce_seq(rxfce: u32) -> u8 {
    ((rxfce & MT_RX_FCE_INFO_CMD_SEQ) >> 16) as u8
}

/// Event type in a response's leading FCE word. `mt76x02_usb_mcu.c:57`
pub const fn rxfce_evt(rxfce: u32) -> u32 {
    (rxfce & MT_RX_FCE_INFO_EVT_TYPE) >> 20
}

// ─────────────────────────────────────────────────────────────────────────────
// Command ids and enums
// ─────────────────────────────────────────────────────────────────────────────

/// `enum mcu_cmd` — mt76x02_mcu.h:30-52. Only the members mt76x0 actually
/// issues are commented; the rest are transcribed for completeness so nobody
/// has to guess an id later.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[repr(u8)]
pub enum McuCmd {
    /// Carries an [`McuFunction`] selection. `mt76x0/init.c:183` (`Q_SELECT`)
    /// and `mt76x0/phy.c:500` (`BW_SETTING`).
    FunSetOp = 1,
    LoadCr = 2,
    InitGainOp = 3,
    DyncVgaOp = 6,
    TdlsChSw = 7,
    BurstWrite = 8,
    ReadModifyWrite = 9,
    /// Read up to 24 registers in one command. See [`rd_rp`].
    RandomRead = 10,
    BurstRead = 11,
    /// Write up to 24 registers in one command. See [`wr_rp`]. This is the
    /// workhorse: MAC/BBP init tables *and* all RF-bank writes ride on it.
    RandomWrite = 12,
    LedModeOp = 16,
    /// Carries an [`McuPowerMode`]. `mt76x02_mcu.c:112`
    PowerSavingOp = 20,
    WowConfig = 21,
    WowQuery = 22,
    WowFeature = 24,
    CarrierDetectOp = 28,
    RadorDetectOp = 29,
    SwitchChannelOp = 30,
    /// Carries an [`McuCalibrate`]. `mt76x0/phy.c:871,872,903,904,909,1041`
    CalibrationOp = 31,
    BeaconOp = 32,
    AntennaOp = 33,
}

/// `enum mcu_function` — mt76x02_mcu.h:62-69.
///
/// ⚠ `BW_SETTING` and `USB2_SW_DISCONNECT` are **both 2** upstream; they are
/// distinguished only by context. Transcribed as-is.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[repr(u32)]
pub enum McuFunction {
    /// Queue select. The one function upstream sends **without** waiting for a
    /// response (`mt76x02_mcu.c:94-95`).
    QSelect = 1,
    /// Bandwidth setting (`mt76x0/phy.c:500`). Same numeric id as
    /// `USB2_SW_DISCONNECT`.
    BwSetting = 2,
    Usb3SwDisconnect = 3,
    LogFwDebugMsg = 4,
    GetFwVersion = 5,
}

/// `enum mcu_power_mode` — mt76x02_mcu.h:54-60.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[repr(u32)]
pub enum McuPowerMode {
    RadioOff = 0x30,
    RadioOn = 0x31,
    RadioOffAutoWakeup = 0x32,
    RadioOffAdvance = 0x33,
    RadioOnAdvance = 0x34,
}

/// `enum mcu_calibrate` — **mt76x0/mcu.h:22-37**, i.e. the mt76x0-specific
/// list. (mt76x2 has its own with different numbering; do not cross them.)
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[repr(u32)]
pub enum McuCalibrate {
    R = 1,
    RxDcoc = 2,
    Lc = 3,
    Loft = 4,
    TxIq = 5,
    Bw = 6,
    Dpd = 7,
    RxIq = 8,
    TxDcoc = 9,
    RxGroupDelay = 10,
    TxGroupDelay = 11,
    Vco = 12,
    NoSignal = 0xfe,
    Full = 0xff,
}

// ─────────────────────────────────────────────────────────────────────────────
// mcu_send
// ─────────────────────────────────────────────────────────────────────────────

/// How many times to re-read the cmd-resp endpoint before giving up on a
/// response. `mt76x02_usb_mcu.c:44` (`for (i = 0; i < 5; i++)`).
const MCU_RESP_RETRIES: usize = 5;

/// Send one in-band MCU command.
///
/// The wire frame is [`build_cmd_frame`]; see [`mcu_info_word`] for the exact
/// bit layout of the leading info word.
///
/// `wait_resp` decides two things at once, exactly as upstream does
/// (`mt76x02_usb_mcu.c:82-86`): whether a **non-zero sequence number** goes into
/// the info word, and whether the cmd-resp endpoint is read afterwards. They are
/// not separable — a command sent with `seq == 0` posts no response, so waiting
/// for one just burns the timeout. (The MT7612U backend learned this the hard
/// way: forcing a non-zero seq onto a fire-and-forget command left the command
/// FIFO undrained and every subsequent write blocked ~1 s.)
pub fn mcu_send(bus: &dyn McuBus, cmd: u8, data: &[u8], wait_resp: bool) -> Result<(), FaceError> {
    send_msg(bus, cmd, data, wait_resp).map(|_| ())
}

/// [`mcu_send`], but handing back the response bytes when one was awaited. Only
/// [`rd_rp`] needs them; upstream reaches the same data through a
/// `usb->mcu.rp` side channel set up around the call (`mt76x02_usb_mcu.c:194-200`),
/// which is not expressible without shared mutable state.
fn send_msg(
    bus: &dyn McuBus,
    cmd: u8,
    data: &[u8],
    wait_resp: bool,
) -> Result<Option<Vec<u8>>, FaceError> {
    // ⚠ Not an upstream check. Upstream enforces MT_INBAND_PACKET_MAX_LEN only
    // on the reg-pair paths (`mt76x02_usb_mcu.c:136,170`) and trusts every other
    // caller; the constant's name (mt76x02_mcu.h:18) says it is the in-band
    // channel's limit, and mt76x0 never sends a payload over 8 B outside those
    // paths, so rejecting an oversize command here can only catch a bug. It runs
    // before `next_seq()` so a rejection does not burn a sequence number.
    if data.len() > regs::MT_INBAND_PACKET_MAX_LEN {
        return Err(mcu_err(format!(
            "command 0x{cmd:02x} payload {} B exceeds MT_INBAND_PACKET_MAX_LEN ({})",
            data.len(),
            regs::MT_INBAND_PACKET_MAX_LEN
        )));
    }
    // seq 0 == "no response expected" (mt76x02_usb_mcu.c:82-86).
    let seq = if wait_resp { bus.next_seq() & 0xf } else { 0 };
    if wait_resp && seq == 0 {
        return Err(mcu_err("McuBus::next_seq returned 0; must be 1..=15"));
    }

    let frame = build_cmd_frame(cmd, seq, data);
    bus.bulk_out_cmd(&frame).map_err(|e| {
        mcu_err(format!(
            "command 0x{cmd:02x} seq {seq} bulk-out ({} B): {e}",
            frame.len()
        ))
    })?;

    if !wait_resp {
        return Ok(None);
    }
    Ok(Some(wait_resp_seq(bus, cmd, seq)?))
}

/// `mt76x02u_mcu_wait_resp` (`mt76x02_usb_mcu.c:37-67`): read the cmd-resp
/// endpoint until a response arrives whose FCE word carries our sequence number
/// and `EVT_CMD_DONE`. Returns the response bytes.
///
/// Two deliberate differences from upstream, both stated here rather than left
/// to be discovered:
///   1. Upstream retries only on `-ETIMEDOUT` and bails on any other error. The
///      [`McuBus::bulk_in_resp`] contract cannot distinguish the two, so **any**
///      error is retried up to [`MCU_RESP_RETRIES`] times and the last one is
///      reported. Worst case on a genuinely dead pipe: five quick failures.
///   2. Upstream parses the register-pair payload *before* checking the sequence
///      number (`:52-53`), so a stale or mismatched response would still be
///      written into the caller's array. Here the bytes are only returned once
///      the sequence matched, and [`rd_rp`] parses them after that.
fn wait_resp_seq(bus: &dyn McuBus, cmd: u8, seq: u8) -> Result<Vec<u8>, FaceError> {
    let mut buf = vec![0u8; MCU_RESP_URB_SIZE];
    let mut last: Option<FaceError> = None;

    for _ in 0..MCU_RESP_RETRIES {
        let n = match bus.bulk_in_resp(&mut buf) {
            Ok(n) => n,
            Err(e) => {
                last = Some(e);
                continue;
            }
        };
        if n < 4 {
            last = Some(mcu_err(format!(
                "command 0x{cmd:02x} seq {seq}: response {n} B, need at least 4"
            )));
            continue;
        }
        let rxfce = u32::from_le_bytes([buf[0], buf[1], buf[2], buf[3]]);
        let (rseq, revt) = (rxfce_seq(rxfce), rxfce_evt(rxfce));
        if rseq == seq && revt == EVT_CMD_DONE {
            if mcu_debug() {
                eprintln!("  mcu 0x{cmd:02x} seq {seq} resp {n} B ok");
            }
            buf.truncate(n);
            return Ok(buf);
        }
        // Upstream logs and retries the same way (`:60-62`).
        if mcu_debug() {
            eprintln!(
                "  mcu 0x{cmd:02x}: resp evt {revt:#x} seq {rseq} (wanted seq {seq}, evt CMD_DONE)"
            );
        }
        last = Some(mcu_err(format!(
            "command 0x{cmd:02x}: response evt {revt:#x} seq {rseq}, wanted evt CMD_DONE seq {seq}"
        )));
    }
    Err(last.unwrap_or_else(|| mcu_err(format!("command 0x{cmd:02x} seq {seq}: no response"))))
}

// ─────────────────────────────────────────────────────────────────────────────
// Register pairs
// ─────────────────────────────────────────────────────────────────────────────

/// One `{ base + reg, value }` entry of the MCU register-pair protocol —
/// upstream's `struct mt76_reg_pair` (mt76.h:80).
///
/// `reg` is an **offset from `base`**, not an absolute address; the firmware
/// adds the base. The two bases in play are [`regs::MT_MCU_MEMMAP_WLAN`]
/// (`0x410000`, MAC and BBP — upstream notes at `mt76x0/mcu.h:17-19` that the
/// BBP deliberately shares the MAC space) and [`regs::MT_MCU_MEMMAP_RF`]
/// (`0x80000000`, the RF banks).
#[derive(Clone, Copy, Debug, PartialEq, Eq, Default)]
pub struct RegPair {
    /// Offset from the command's `base`.
    pub reg: u32,
    /// Value to write, or the slot a read fills in.
    pub value: u32,
}

impl RegPair {
    /// A pair, spelled for table literals.
    pub const fn new(reg: u32, value: u32) -> Self {
        Self { reg, value }
    }
}

/// Most register pairs one command can carry:
/// `MT_INBAND_PACKET_MAX_LEN / 8 = 192 / 8 = 24`
/// (`mt76x02_usb_mcu.c:136` and `:170`, both spelled `max_vals_per_cmd`).
pub const MT_MCU_REGPAIRS_MAX: usize = regs::MT_INBAND_PACKET_MAX_LEN / 8;

/// Encode the payload of a `RANDOM_READ` / `RANDOM_WRITE` command: repeated
/// `{ le32 base + reg; le32 value }` (`mt76x02_usb_mcu.c:151-154`, `:187-190`).
///
/// The read path encodes the caller's `value` field too, unchanged — upstream
/// does the same at `:189` and gives no reason, so it is ported rather than
/// zeroed. In practice callers leave it 0 (`mt76x0/phy.c:128-130`).
pub fn encode_reg_pairs(base: u32, pairs: &[RegPair]) -> Vec<u8> {
    let mut buf = Vec::with_capacity(pairs.len() * 8);
    for p in pairs {
        buf.extend_from_slice(&base.wrapping_add(p.reg).to_le_bytes());
        buf.extend_from_slice(&p.value.to_le_bytes());
    }
    buf
}

/// Decode a `RANDOM_READ` response into `out`, filling each pair's `value`.
///
/// Response layout (`mt76x02_usb_mcu.c:53` passes `data + 4` and `len - 8`, and
/// `:29-33` walks it in 8-byte strides):
///
/// ```text
/// | 4B rxfce | le32 base+reg | le32 value | … n times … | 4B trailer |
/// ```
///
/// so parsing starts at `resp[4]`, `reg = le32(p) - base`, `val = le32(p + 4)`.
///
/// ⚠ **Deliberate divergence.** Upstream's two consistency checks are
/// `WARN_ON_ONCE` (`:26`, `:32`): it complains and then *writes the value
/// anyway*. A register mismatch means the response is misaligned with the
/// request, so every value in it is suspect — this returns an error instead of
/// handing back plausible-looking garbage.
pub fn decode_reg_pairs(base: u32, resp: &[u8], out: &mut [RegPair]) -> Result<(), FaceError> {
    let n = out.len();
    if resp.len() < 8 {
        return Err(mcu_err(format!(
            "rd_rp response {} B, need at least 8",
            resp.len()
        )));
    }
    // Upstream's predicate, restated: (len - 8) / 8 == rp_len (`:26`, `:53`).
    if (resp.len() - 8) / 8 != n {
        return Err(mcu_err(format!(
            "rd_rp response {} B carries {} pairs, expected {n}",
            resp.len(),
            (resp.len() - 8) / 8
        )));
    }
    for (i, slot) in out.iter_mut().enumerate() {
        let p = 4 + 8 * i;
        let reg_abs = u32::from_le_bytes([resp[p], resp[p + 1], resp[p + 2], resp[p + 3]]);
        let val = u32::from_le_bytes([resp[p + 4], resp[p + 5], resp[p + 6], resp[p + 7]]);
        let reg = reg_abs.wrapping_sub(base);
        if reg != slot.reg {
            return Err(mcu_err(format!(
                "rd_rp pair {i}: response reg {reg:#x} (abs {reg_abs:#x}) != requested {:#x}",
                slot.reg
            )));
        }
        slot.value = val;
    }
    Ok(())
}

/// Write register pairs through the MCU (`CMD_RANDOM_WRITE = 12`).
///
/// `mt76x02u_mcu_wr_rp` (`mt76x02_usb_mcu.c:132-163`) chunks at
/// [`MT_MCU_REGPAIRS_MAX`] and — the detail that is easy to get wrong —
/// **waits for a response only on the final chunk**: its `wait_resp` argument is
/// `cnt == n` (`:157`), which is true exactly when the current chunk drains the
/// remainder. Intermediate chunks are fire-and-forget with seq 0.
///
/// An empty slice is a no-op (`:141-142`).
///
/// ⚠ This path requires **running firmware**. Upstream routes `mt76_wr_rp`
/// through the MCU only when `MT76_STATE_MCU_RUNNING` is set and otherwise falls
/// back to plain per-register control writes (`usb.c:257-265`); with the MCU
/// down, these commands are simply never consumed. Use [`wr_rp_direct`] before
/// firmware is up.
pub fn wr_rp(bus: &dyn McuBus, base: u32, pairs: &[RegPair]) -> Result<(), FaceError> {
    if pairs.is_empty() {
        return Ok(());
    }
    let mut rest = pairs;
    while !rest.is_empty() {
        let cnt = rest.len().min(MT_MCU_REGPAIRS_MAX);
        let final_chunk = cnt == rest.len();
        let payload = encode_reg_pairs(base, &rest[..cnt]);
        mcu_send(bus, McuCmd::RandomWrite as u8, &payload, final_chunk)?;
        rest = &rest[cnt..];
    }
    Ok(())
}

/// Read register pairs through the MCU (`CMD_RANDOM_READ = 10`).
///
/// `mt76x02u_mcu_rd_rp` (`mt76x02_usb_mcu.c:165-205`) does **not** chunk: it
/// computes `cnt = min(24, n)` and then `if (cnt != n) return -EINVAL` (`:178-180`),
/// so more than [`MT_MCU_REGPAIRS_MAX`] pairs is a caller error, not a loop.
/// Split the request yourself if you need more. Always waits for the response
/// (`:198`) — there is nowhere else for the values to come from.
///
/// See the firmware caveat on [`wr_rp`]; [`rd_rp_direct`] is the pre-firmware
/// equivalent.
pub fn rd_rp(bus: &dyn McuBus, base: u32, out: &mut [RegPair]) -> Result<(), FaceError> {
    if out.is_empty() {
        return Ok(());
    }
    if out.len() > MT_MCU_REGPAIRS_MAX {
        return Err(mcu_err(format!(
            "rd_rp: {} pairs exceeds the {MT_MCU_REGPAIRS_MAX}-pair command limit \
             (upstream returns -EINVAL, mt76x02_usb_mcu.c:178-180)",
            out.len()
        )));
    }
    let payload = encode_reg_pairs(base, out);
    let resp = send_msg(bus, McuCmd::RandomRead as u8, &payload, true)?
        .ok_or_else(|| mcu_err("rd_rp: no response body"))?;
    decode_reg_pairs(base, &resp, out)
}

/// `mt76u_req_wr_rp` (`usb.c:240-256`) — the **pre-firmware** fallback: write
/// each pair as an ordinary register write to the absolute address
/// `base + reg`. One 151 µs control transfer per pair, so a 100-entry init table
/// costs ~15 ms here versus ~1 ms through [`wr_rp`]. Correct before the MCU is
/// up; wasteful after.
pub fn wr_rp_direct(bus: &dyn McuBus, base: u32, pairs: &[RegPair]) -> Result<(), FaceError> {
    for p in pairs {
        bus.wr(base.wrapping_add(p.reg), p.value)?;
    }
    Ok(())
}

/// `mt76u_req_rd_rp` (`usb.c:267-282`) — the pre-firmware read fallback. See
/// [`wr_rp_direct`].
pub fn rd_rp_direct(bus: &dyn McuBus, base: u32, out: &mut [RegPair]) -> Result<(), FaceError> {
    for p in out.iter_mut() {
        p.value = bus.rr(base.wrapping_add(p.reg))?;
    }
    Ok(())
}

// ─────────────────────────────────────────────────────────────────────────────
// The handful of typed commands mt76x0 actually issues
// ─────────────────────────────────────────────────────────────────────────────

/// Payload shared by `CMD_FUN_SET_OP`, `CMD_POWER_SAVING_OP` and
/// `CMD_CALIBRATION_OP`: `{ __le32 id; __le32 value; }`
/// (`mt76x02_mcu.c:85-91`, `:104-110`, `:119-125`).
fn id_value_payload(id: u32, value: u32) -> [u8; 8] {
    let mut b = [0u8; 8];
    b[0..4].copy_from_slice(&id.to_le_bytes());
    b[4..8].copy_from_slice(&value.to_le_bytes());
    b
}

/// `mt76x02_mcu_function_select` (`mt76x02_mcu.c:82-100`).
///
/// Waits for a response for every function **except** `Q_SELECT` (`:94-95`);
/// upstream gives no reason and neither can we, so it is ported as written.
/// Called by mt76x0 twice: `Q_SELECT` at init (`mt76x0/init.c:183`) and
/// `BW_SETTING` on every bandwidth change (`mt76x0/phy.c:500`).
pub fn function_select(bus: &dyn McuBus, func: McuFunction, val: u32) -> Result<(), FaceError> {
    let wait = func != McuFunction::QSelect;
    let msg = id_value_payload(func as u32, val);
    mcu_send(bus, McuCmd::FunSetOp as u8, &msg, wait)
}

/// `mt76x02_mcu_set_radio_state` (`mt76x02_mcu.c:102-115`). Fire-and-forget:
/// `wait_resp` is false (`:113`).
pub fn set_radio_state(bus: &dyn McuBus, on: bool) -> Result<(), FaceError> {
    let mode = if on {
        McuPowerMode::RadioOn
    } else {
        McuPowerMode::RadioOff
    };
    let msg = id_value_payload(mode as u32, 0);
    mcu_send(bus, McuCmd::PowerSavingOp as u8, &msg, false)
}

/// `mt76x02_mcu_calibrate` (`mt76x02_mcu.c:117-144`) — run one firmware
/// calibration and wait for it to acknowledge.
///
/// The `MT_MCU_COM_REG0` BIT(31) handshake around the command (`:129-141`) is
/// gated on `mt76_is_mmio(...) && is_mt76x2(...)`, i.e. the **PCIe MT7612E**
/// only. It is correctly absent on this USB mt76x0 path, and adding it would
/// hang waiting for a bit the firmware never sets.
///
/// (Distinct from `phy::calibrate`, which orchestrates *which* calibrations run
/// for a given channel; this sends one.)
pub fn calibrate(bus: &dyn McuBus, kind: McuCalibrate, param: u32) -> Result<(), FaceError> {
    let msg = id_value_payload(kind as u32, param);
    mcu_send(bus, McuCmd::CalibrationOp as u8, &msg, true)
}

// ─────────────────────────────────────────────────────────────────────────────
// Firmware image
// ─────────────────────────────────────────────────────────────────────────────

/// The MT7610U RAM firmware, linked into the binary.
///
/// Upstream tries `MT7610E_FIRMWARE` first and falls back to `MT7610U_FIRMWARE`
/// (`mt76x0/usb_mcu.c:67-83`); the E-part image is a PCIe convenience and there
/// is no reason to carry two blobs, so only the U image is embedded.
///
/// MEASURED from the shipped file: 80288 B, `ilm_len = 0x10cac`,
/// `dlm_len = 0x2cd4`, `fw_ver = 0x0100` ("0.1.00"), `build_ver = 0x7640`,
/// `build_time = "201308221655"`.
pub const MT7610U_FIRMWARE: &[u8] = include_bytes!("../../fw/mt76x0/mt7610u.bin");

/// `sizeof(struct mt76x02_fw_header)` — mt76x02_mcu.h:71-78:
/// `__le32 ilm_len; __le32 dlm_len; __le16 build_ver; __le16 fw_ver; u8 pad[4];
/// char build_time[16];` = 4+4+2+2+4+16 = **32**.
pub const FW_HEADER_LEN: usize = 32;

/// Parsed `struct mt76x02_fw_header` (mt76x02_mcu.h:71-78).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct FwHeader {
    /// Instruction-memory length **including** the 64-byte IVB that is split off
    /// and delivered separately.
    pub ilm_len: u32,
    /// Data-memory length.
    pub dlm_len: u32,
    /// Build number, printed as hex by upstream.
    pub build_ver: u16,
    /// Version nibbles: `major = >>12 & 0xf`, `minor = >>8 & 0xf`,
    /// `patch = & 0xf` (`mt76x0/usb_mcu.c:117-121`).
    pub fw_ver: u16,
    /// ASCII build stamp; trailing `_` padding in the shipped image.
    pub build_time: [u8; 16],
}

impl FwHeader {
    /// Parse and validate the header against the whole image, applying every
    /// check `mt76x0u_load_firmware` makes (`mt76x0/usb_mcu.c:102-115`):
    /// big enough for a header, `ilm_len > MT_MCU_IVB_SIZE`, and
    /// `size == sizeof(hdr) + ilm_len + dlm_len` **exactly**.
    pub fn parse(fw: &[u8]) -> Result<Self, FaceError> {
        if fw.len() < FW_HEADER_LEN {
            return Err(mcu_err(format!(
                "firmware image {} B is smaller than the {FW_HEADER_LEN} B header",
                fw.len()
            )));
        }
        let ilm_len = u32::from_le_bytes([fw[0], fw[1], fw[2], fw[3]]);
        let dlm_len = u32::from_le_bytes([fw[4], fw[5], fw[6], fw[7]]);
        let build_ver = u16::from_le_bytes([fw[8], fw[9]]);
        let fw_ver = u16::from_le_bytes([fw[10], fw[11]]);
        // fw[12..16] is `u8 pad[4]` — not interpreted upstream, not here.
        let mut build_time = [0u8; 16];
        build_time.copy_from_slice(&fw[16..32]);

        // mt76x0/usb_mcu.c:107-108 — the IVB is carved out of the ILM region, so
        // an ILM no larger than the IVB leaves nothing to DMA.
        if ilm_len <= regs::MT_MCU_IVB_SIZE {
            return Err(mcu_err(format!(
                "firmware ilm_len {ilm_len} <= MT_MCU_IVB_SIZE {}",
                regs::MT_MCU_IVB_SIZE
            )));
        }
        // mt76x0/usb_mcu.c:110-115 — exact-size check, not a lower bound.
        let want = FW_HEADER_LEN as u64 + ilm_len as u64 + dlm_len as u64;
        if fw.len() as u64 != want {
            return Err(mcu_err(format!(
                "firmware image {} B != header + ilm {ilm_len} + dlm {dlm_len} = {want}",
                fw.len()
            )));
        }
        Ok(Self {
            ilm_len,
            dlm_len,
            build_ver,
            fw_ver,
            build_time,
        })
    }

    /// `"%d.%d.%02d-b%x"` — the string upstream puts in `wiphy->fw_version`
    /// (`mt76x02_mcu.c:163-169`). For the shipped image: `"0.1.00-b7640"`.
    pub fn version_string(&self) -> String {
        format!(
            "{}.{}.{:02}-b{:x}",
            (self.fw_ver >> 12) & 0xf,
            (self.fw_ver >> 8) & 0xf,
            self.fw_ver & 0xf,
            self.build_ver
        )
    }

    /// The build stamp with the `_` padding trimmed.
    pub fn build_time_str(&self) -> String {
        String::from_utf8_lossy(&self.build_time)
            .trim_end_matches(['_', '\0', ' '])
            .to_string()
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Firmware download
// ─────────────────────────────────────────────────────────────────────────────

/// `MCU_FW_URB_MAX_PAYLOAD` — mt76x0/usb_mcu.c:13. Note this is **0x38f8**,
/// eight bytes below the mt76x2 value of 0x3900 (`mt76x2/usb_mcu.c:14`); the
/// difference is not explained upstream, and it matters only in that it sets
/// [`FW_CHUNK_MAX`].
pub const MCU_FW_URB_MAX_PAYLOAD: usize = 0x38f8;

/// `MCU_FW_URB_SIZE` — mt76x0/usb_mcu.c:14, defined as
/// `MCU_FW_URB_MAX_PAYLOAD + 12` and then never referenced in that file.
/// Recorded for completeness; unused here for the same reason.
pub const MCU_FW_URB_SIZE: usize = MCU_FW_URB_MAX_PAYLOAD + 12;

/// Largest firmware payload in one chunk: `max_payload - 8`
/// (`mt76x02_usb_mcu.c:256`). The 8 is the frame's own overhead — 4-byte info
/// word plus 4-byte trailer — so a full chunk is exactly
/// [`MCU_FW_URB_MAX_PAYLOAD`] bytes on the wire, which is what upstream's
/// `kmalloc(max_payload)` is sized for.
pub const FW_CHUNK_MAX: usize = MCU_FW_URB_MAX_PAYLOAD - 8;

/// Upstream sleeps 5-10 ms between firmware chunks (`mt76x02_usb_mcu.c:272`).
/// No reason is given; with ~5 chunks for ILM and 1 for DLM it costs under
/// 50 ms, so it is ported as-is rather than tuned.
const FW_CHUNK_SETTLE: Duration = Duration::from_millis(5);

/// `mt76x02u_mcu_fw_reset` (`mt76x02_usb_mcu.c:207-212`): vendor
/// `MT_VEND_DEV_MODE` with `wValue = 1`, no data.
pub fn mcu_fw_reset(bus: &dyn McuBus) -> Result<(), FaceError> {
    bus.vendor_write(MT_VEND_DEV_MODE, DEV_MODE_FW_RESET, 0, &[])
}

/// Send one firmware chunk — `__mt76x02u_mcu_fw_send_data`
/// (`mt76x02_usb_mcu.c:215-251`), in upstream's order, which matters:
///
/// 1. Program the FCE DMA **destination address** at [`regs::MT_FCE_DMA_ADDR`]
///    via [`MT_VEND_WRITE_FCE`] (`:231-232`).
/// 2. Round the length up to 4 and program **`len << 16`** at
///    [`regs::MT_FCE_DMA_LEN`] (`:233-235`) — the length lives in the *upper*
///    half word, so the two `wValue` halves are `0` then `len`.
/// 3. Bulk-OUT `[info][chunk][pad][trailer]` on the inband-command endpoint
///    (`:239-240`). The info word's LEN field carries the **unrounded** length —
///    see [`fw_info_word`].
/// 4. Ring the doorbell: read [`regs::MT_TX_CPU_FROM_FCE_CPU_DESC_IDX`],
///    increment, write back (`:246-248`).
///
/// ★ Step 4 is where the sibling MT7612U backend goes wrong. It replaces the
/// read-increment-write with a poll of the same register guarded by
/// `.unwrap_or(false)`, so a control-read failure reads as "not busy" and the
/// engine is advanced blind. Here the read is `?`-propagated: a broken control
/// pipe fails the chunk that hit it.
fn fw_send_chunk(bus: &dyn McuBus, chunk: &[u8], dst: u32) -> Result<(), FaceError> {
    // (1) FCE DMA destination.
    single_wr(bus, MT_VEND_WRITE_FCE, regs::MT_FCE_DMA_ADDR as u16, dst)?;
    // (2) FCE DMA length, 4-aligned, in the high half word.
    let rounded = round_up4(chunk.len()) as u32;
    single_wr(
        bus,
        MT_VEND_WRITE_FCE,
        regs::MT_FCE_DMA_LEN as u16,
        rounded << 16,
    )?;
    // (3) The frame itself.
    let frame = build_fw_frame(chunk);
    bus.bulk_out_cmd(&frame).map_err(|e| {
        mcu_err(format!(
            "firmware chunk (dst {dst:#x}, {} B payload, {} B frame): {e}",
            chunk.len(),
            frame.len()
        ))
    })?;
    // (4) Doorbell — read/increment/write, error propagated.
    let idx = bus.rr(regs::MT_TX_CPU_FROM_FCE_CPU_DESC_IDX).map_err(|e| {
        mcu_err(format!(
            "firmware chunk (dst {dst:#x}): FCE desc-idx read failed: {e}"
        ))
    })?;
    bus.wr(regs::MT_TX_CPU_FROM_FCE_CPU_DESC_IDX, idx.wrapping_add(1))?;
    Ok(())
}

/// `mt76x02u_mcu_fw_send_data` (`mt76x02_usb_mcu.c:253-277`): stream `data` to
/// MCU address `offset` in [`FW_CHUNK_MAX`]-sized pieces.
pub fn fw_send_data(bus: &dyn McuBus, data: &[u8], offset: u32) -> Result<(), FaceError> {
    let dbg = ep_debug();
    let nchunks = data.len().div_ceil(FW_CHUNK_MAX).max(1);
    if dbg {
        eprintln!(
            "    fw_send_data dst={offset:#x} len={} chunks={nchunks}",
            data.len()
        );
    }
    let mut pos = 0usize;
    let mut idx = 0usize;
    while pos < data.len() {
        let cur = (data.len() - pos).min(FW_CHUNK_MAX);
        fw_send_chunk(bus, &data[pos..pos + cur], offset + pos as u32)?;
        if dbg {
            eprintln!(
                "    chunk {idx}/{nchunks} ({cur} B @ {:#x})",
                offset + pos as u32
            );
        }
        pos += cur;
        idx += 1;
        std::thread::sleep(FW_CHUNK_SETTLE);
    }
    Ok(())
}

/// `mt76x0_firmware_running` (`mt76x0/mcu.h:41-44`) — literally
/// `mt76_rr(dev, MT_MCU_COM_REG0) == 1`, an **exact equality**, not a bit test.
///
/// Two notes worth carrying:
///   * The MT7612U backend's equivalent tests `v & 1 != 0 && v >> 16 == 0x0011`.
///     That is the mt76x2 runtime signature and does not apply here.
///   * Upstream's `mt76_rr` returns `~0` when the control transfer fails, so a
///     dead pipe silently reads as "not running" and the driver re-downloads.
///     Here the read error is propagated instead.
pub fn firmware_running(bus: &dyn McuBus) -> Result<bool, FaceError> {
    Ok(bus.rr(regs::MT_MCU_COM_REG0)? == 1)
}

/// `mt76x02_wait_for_mac` (`mt76x02_mac.h:149-168`): poll `MAC_CSR0` until it
/// reads as something other than `0` or `~0` — i.e. until the MAC block answers
/// at all. Up to 500 tries, 5-10 ms apart.
///
/// The bring-up order upstream uses is worth stating because it is not obvious
/// from any one file: at probe, `chip_onoff(false, false)` →
/// `wait_for_mac` → read `MT_ASIC_VERSION` (`mt76x0/usb.c:258-266`); then, per
/// init, `chip_onoff(true, reset)` → `wait_for_mac` → [`load_firmware`]
/// (`mt76x0/usb.c:151-165`).
pub fn wait_for_mac(bus: &dyn McuBus) -> Result<(), FaceError> {
    for _ in 0..500 {
        let v = bus.rr(regs::MT_MAC_CSR0)?;
        if v != 0 && v != u32::MAX {
            return Ok(());
        }
        std::thread::sleep(Duration::from_millis(5));
    }
    Err(mcu_err("MAC did not come up (MAC_CSR0 stayed 0 / ~0)"))
}

/// `mt76x0_chip_onoff` (`mt76x0/init.c:44-70`) plus the `mt76x0_set_wlan_state`
/// tail it always calls (`:16-42`).
///
/// With `reset`, the WLAN digital and RF resets are pulsed (only if the block
/// was already enabled), `GPIO_OUT_EN` is forced on and `FRC_WL_ANT_SEL` off.
/// Then the enable/clock bits are set or the enable bit cleared, and — when
/// enabling — [`regs::MT_CMB_CTRL`] is polled for crystal-ready **and** PLL-lock
/// before anything else touches the MAC.
///
/// Faithful details worth not "fixing":
///   * `WLAN_CLK_EN` is deliberately never cleared on the way down; upstream's
///     comment (`:20-24`) says clearing it makes the device stop answering on
///     the probe path.
///   * A failed PLL/XTAL poll is `dev_err` **and continue** upstream (`:40-41`),
///     not a failure. That is preserved — the warning goes to stderr and the
///     caller decides — because turning it into an error changes bring-up
///     behaviour on silicon nobody here has measured yet.
pub fn chip_onoff(bus: &dyn McuBus, enable: bool, reset: bool) -> Result<(), FaceError> {
    let mut val = bus.rr(regs::MT_WLAN_FUN_CTRL)?;

    if reset {
        val |= regs::MT_WLAN_FUN_CTRL_GPIO_OUT_EN;
        val &= !regs::MT_WLAN_FUN_CTRL_FRC_WL_ANT_SEL;

        if val & regs::MT_WLAN_FUN_CTRL_WLAN_EN != 0 {
            val |= regs::MT_WLAN_FUN_CTRL_WLAN_RESET | regs::MT_WLAN_FUN_CTRL_WLAN_RESET_RF;
            bus.wr(regs::MT_WLAN_FUN_CTRL, val)?;
            std::thread::sleep(Duration::from_micros(20));

            val &= !(regs::MT_WLAN_FUN_CTRL_WLAN_RESET | regs::MT_WLAN_FUN_CTRL_WLAN_RESET_RF);
        }
    }

    bus.wr(regs::MT_WLAN_FUN_CTRL, val)?;
    std::thread::sleep(Duration::from_micros(20));

    // ── mt76x0_set_wlan_state (mt76x0/init.c:16-42) ──
    if enable {
        val |= regs::MT_WLAN_FUN_CTRL_WLAN_EN | regs::MT_WLAN_FUN_CTRL_WLAN_CLK_EN;
    } else {
        val &= !regs::MT_WLAN_FUN_CTRL_WLAN_EN;
    }
    bus.wr(regs::MT_WLAN_FUN_CTRL, val)?;
    std::thread::sleep(Duration::from_micros(20));

    if enable {
        let mask = regs::MT_CMB_CTRL_XTAL_RDY | regs::MT_CMB_CTRL_PLL_LD;
        if !poll_usec(bus, regs::MT_CMB_CTRL, mask, mask, 2000)? {
            // Upstream: dev_err + continue (mt76x0/init.c:40-41).
            eprintln!("mt7610u mcu: PLL and XTAL check failed (MT_CMB_CTRL)");
        }
    }
    Ok(())
}

/// `mt76x0u_upload_firmware` (`mt76x0/usb_mcu.c:16-65`): ILM, DLM, IVB, poll.
fn upload_firmware(bus: &dyn McuBus, hdr: &FwHeader, payload: &[u8]) -> Result<(), FaceError> {
    let dbg = ep_debug();
    let ivb_size = regs::MT_MCU_IVB_SIZE as usize;
    let ilm_total = hdr.ilm_len as usize;
    let dlm_len = hdr.dlm_len as usize;

    // The first MT_MCU_IVB_SIZE bytes of the ILM region are the interrupt-vector
    // block. They are copied out here and shipped last, by a different mechanism
    // (`:25-27` then `:47-49`).
    let ivb = &payload[..ivb_size];

    // ILM, minus the IVB, loaded at MT_MCU_IVB_SIZE (`:29-34`). ★ This offset is
    // 0x40 on mt76x0 — the ILM window starts at 0 — not the 0x80000 mt76x2 uses.
    let ilm_len = ilm_total - ivb_size;
    if dbg {
        eprintln!("  fw: ILM {ilm_len} B + IVB {ivb_size} B");
    }
    fw_send_data(
        bus,
        &payload[ivb_size..ivb_size + ilm_len],
        regs::MT_MCU_IVB_SIZE,
    )?;

    // DLM at MT_MCU_DLM_OFFSET (`:38-43`). No E3 adjustment on this part — see
    // the module header.
    if dbg {
        eprintln!("  fw: DLM {dlm_len} B @ {:#x}", regs::MT_MCU_DLM_OFFSET);
    }
    fw_send_data(
        bus,
        &payload[ilm_total..ilm_total + dlm_len],
        regs::MT_MCU_DLM_OFFSET,
    )?;

    // Hand over the IVB. This is what starts the CPU: vendor MT_VEND_DEV_MODE,
    // wValue 0x12, with the 64 IVB bytes as the data stage (`:47-49`).
    if dbg {
        eprintln!("  fw: load IVB (DEV_MODE wValue 0x12, {ivb_size} B body)");
    }
    bus.vendor_write(MT_VEND_DEV_MODE, DEV_MODE_LOAD_IVB, 0, ivb)?;

    // mt76_poll_msec(dev, MT_MCU_COM_REG0, 1, 1, 1000) — `:53-57`.
    if !poll_msec(bus, regs::MT_MCU_COM_REG0, 1, 1, 1000)? {
        let last = bus.rr(regs::MT_MCU_COM_REG0)?;
        return Err(mcu_err(format!(
            "firmware failed to start: MT_MCU_COM_REG0 = {last:#x} after 1000 ms (want 1)"
        )));
    }
    if dbg {
        eprintln!("  fw: running");
    }
    Ok(())
}

/// Download and start the MT7610U firmware — `mt76x0u_load_firmware`
/// (`mt76x0/usb_mcu.c:85-162`).
///
/// Pass [`MT7610U_FIRMWARE`] unless you are testing an alternative image. The
/// caller is responsible for having powered the chip ([`chip_onoff`]) and waited
/// for the MAC ([`wait_for_mac`]) first; upstream does both in
/// `mt76x0u_init_hardware` (`mt76x0/usb.c:151-165`) rather than in here, and
/// that split is kept so a caller can re-run the load without re-cycling power.
///
/// The sequence, in order, with the pieces that are opaque upstream flagged:
///
/// | step | what | upstream |
/// |---|---|---|
/// | 1 | `MT_USB_DMA_CFG = RX_BULK_EN \| TX_BULK_EN` | `:92-93` |
/// | 2 | bail out early if firmware already runs | `:95-96` |
/// | 3 | parse + validate the header | `:102-121` |
/// | 4 | `wr(0x1004, 0x2c)` | `:125` — ⚠ see below |
/// | 5 | OR in `RX_BULK_AGG_TOUT = 0x20` | `:127-129` |
/// | 6 | vendor firmware reset, then ~5 ms | `:130-131` |
/// | 7 | FCE: `PSE_CTRL=1`, base-ptr `0x400230`, max-count `1`, `PDMA_GLOBAL_CONF=0x44`, `SKIP_FS=3` | `:133-142` |
/// | 8 | pulse `UDMA_TX_WL_DROP` on then off | `:144-148` |
/// | 9 | ILM → DLM → IVB → poll `MT_MCU_COM_REG0` | `:150` → [`upload_firmware`] |
/// | 10 | `PSE_CTRL = 1` again, **even if step 9 failed** | `:154` |
///
/// ⚠ **Step 4 is unexplained.** Upstream writes the literal `0x2c` to the
/// literal address `0x1004` — that is [`regs::MT_MAC_SYS_CTRL`], and `0x2c` is
/// `ENABLE_TX | ENABLE_RX | BIT(5)`, where BIT(5) has no name in
/// `mt76x02_regs.h`. Enabling the MAC in the middle of a firmware download is
/// not something we can derive a reason for. Ported faithfully; do not
/// "correct" it to `MT_MAC_SYS_CTRL_ENABLE_TX | _ENABLE_RX`.
///
/// ⚠ **Step 8 is also unexplained.** Setting `UDMA_TX_WL_DROP` and immediately
/// clearing it presumably flushes a queue. Ported faithfully.
pub fn load_firmware(bus: &dyn McuBus, fw: &[u8]) -> Result<(), FaceError> {
    let dbg = ep_debug();

    // (1) Minimal DMA config so the bulk pipes work at all (`:92-93`).
    bus.wr(
        regs::MT_USB_DMA_CFG,
        regs::MT_USB_DMA_CFG_RX_BULK_EN | regs::MT_USB_DMA_CFG_TX_BULK_EN,
    )?;

    // (2) Already up? A USB re-enumeration does not reset the on-chip MCU, so
    // this is the common case on a second run (`:95-96`).
    if firmware_running(bus)? {
        if dbg {
            eprintln!("  load_firmware: MCU already running, skipping download");
        }
        return Ok(());
    }

    // (3) Header + image validation (`:102-115`).
    let hdr = FwHeader::parse(fw)?;
    let payload = &fw[FW_HEADER_LEN..];
    if dbg {
        eprintln!(
            "  load_firmware: version {} build-time {} (ilm {} dlm {})",
            hdr.version_string(),
            hdr.build_time_str(),
            hdr.ilm_len,
            hdr.dlm_len
        );
    }

    // (4) Unexplained; see the doc table above (`:125`).
    bus.wr(regs::MT_MAC_SYS_CTRL, 0x2c)?;

    // (5) Add the RX bulk aggregation timeout (`:127-129`).
    reg_set(
        bus,
        regs::MT_USB_DMA_CFG,
        regs::MT_USB_DMA_CFG_RX_BULK_EN
            | regs::MT_USB_DMA_CFG_TX_BULK_EN
            | regs::field_prep(regs::MT_USB_DMA_CFG_RX_BULK_AGG_TOUT, 0x20),
    )?;

    // (6) Vendor firmware reset + settle (`:130-131`, usleep_range(5000, 6000)).
    mcu_fw_reset(bus)?;
    std::thread::sleep(Duration::from_millis(5));

    // (7) FCE setup — the engine that will DMA the chunks into MCU memory.
    bus.wr(regs::MT_FCE_PSE_CTRL, 1)?; // `:133`
    bus.wr(regs::MT_TX_CPU_FROM_FCE_BASE_PTR, 0x0040_0230)?; // `:135-136` tx_fs_base_ptr
    bus.wr(regs::MT_TX_CPU_FROM_FCE_MAX_COUNT, 1)?; // `:137-138` tx_fs_max_cnt
    bus.wr(regs::MT_FCE_PDMA_GLOBAL_CONF, 0x44)?; // `:139-140` pdma enable
    bus.wr(regs::MT_FCE_SKIP_FS, 3)?; // `:141-142` skip_fs_en

    // (8) Pulse UDMA_TX_WL_DROP (`:144-148`). Unexplained upstream.
    let val = bus.rr(regs::MT_USB_DMA_CFG)?;
    bus.wr(
        regs::MT_USB_DMA_CFG,
        val | regs::MT_USB_DMA_CFG_UDMA_TX_WL_DROP,
    )?;
    bus.wr(
        regs::MT_USB_DMA_CFG,
        val & !regs::MT_USB_DMA_CFG_UDMA_TX_WL_DROP,
    )?;

    // (9) The download proper.
    let uploaded = upload_firmware(bus, &hdr, payload);

    // (10) Upstream re-arms the FCE unconditionally, on the failure path too
    // (`:150-156` — the write sits between the `ret =` and the `return ret`).
    // Both results are reported; neither is swallowed.
    let rearmed = bus.wr(regs::MT_FCE_PSE_CTRL, 1);
    uploaded?;
    rearmed?;
    Ok(())
}

/// `mt76x0u_mcu_init` (`mt76x0/usb_mcu.c:164-175`) — load the firmware and, on
/// success, the caller may consider `MT76_STATE_MCU_RUNNING` set, which is the
/// flag that switches [`wr_rp`] / [`rd_rp`] from the direct fallback to the
/// in-band path (`usb.c:257-292`).
pub fn mcu_init(bus: &dyn McuBus) -> Result<(), FaceError> {
    load_firmware(bus, MT7610U_FIRMWARE)
}

// ─────────────────────────────────────────────────────────────────────────────
// Tests — pure encoders/decoders only, no device
// ─────────────────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn info_word_bit_layout() {
        // CMD_RANDOM_WRITE (12), seq 3, 16-byte payload (two reg pairs).
        //   TYPE_CMD  bit30            = 0x4000_0000
        //   PORT 2    << 27            = 0x1000_0000
        //   CMD  12   << 20            = 0x00c0_0000
        //   SEQ  3    << 16            = 0x0003_0000
        //   LEN  16                    = 0x0000_0010
        assert_eq!(mcu_info_word(12, 3, 16), 0x50c3_0010);
    }

    #[test]
    fn info_word_rounds_len_up_to_four() {
        // mt76x02_usb_core.c:57 — FIELD_PREP(MT_TXD_INFO_LEN, round_up(len, 4)).
        assert_eq!(mcu_info_word(0, 0, 5) & MT_MCU_MSG_LEN, 8);
        assert_eq!(mcu_info_word(0, 0, 4) & MT_MCU_MSG_LEN, 4);
        assert_eq!(mcu_info_word(0, 0, 0) & MT_MCU_MSG_LEN, 0);
    }

    #[test]
    fn info_word_fields_are_disjoint() {
        // 0xfffc, not 0xffff: LEN is the *rounded* length, and rounding 0xffff
        // to 4 gives 0x10000, which no longer fits GENMASK(15, 0). Real payloads
        // are capped at MT_INBAND_PACKET_MAX_LEN (192 B), so this is a property
        // of the field, not a limit anyone can hit.
        let w = mcu_info_word(0x7f, 0xf, 0xfffc);
        assert_eq!(w & MT_MCU_MSG_LEN, 0xfffc);
        assert_eq!((w & MT_MCU_MSG_CMD_SEQ) >> 16, 0xf);
        assert_eq!((w & MT_MCU_MSG_CMD_TYPE) >> 20, 0x7f);
        assert_eq!((w & MT_MCU_MSG_PORT) >> 27, CPU_TX_PORT);
        assert_eq!((w & MT_MCU_MSG_TYPE) >> 30, 1);
        // The five masks tile the word exactly.
        assert_eq!(
            MT_MCU_MSG_LEN
                | MT_MCU_MSG_CMD_SEQ
                | MT_MCU_MSG_CMD_TYPE
                | MT_MCU_MSG_PORT
                | MT_MCU_MSG_TYPE,
            u32::MAX
        );
    }

    #[test]
    fn fw_info_word_carries_raw_len_and_no_seq_or_cmd() {
        // mt76x02_usb_mcu.c:223-225 computes info BEFORE the roundup at :233.
        let w = fw_info_word(14576); // FW_CHUNK_MAX
        assert_eq!(w, 0x5000_38f0);
        assert_eq!(w & MT_MCU_MSG_CMD_SEQ, 0);
        assert_eq!(w & MT_MCU_MSG_CMD_TYPE, 0);
        // Deliberately NOT rounded, unlike mcu_info_word.
        assert_eq!(fw_info_word(5) & MT_MCU_MSG_LEN, 5);
    }

    #[test]
    fn cmd_frame_layout_is_info_payload_pad_trailer() {
        let f = build_cmd_frame(12, 1, &[0xaa, 0xbb, 0xcc, 0xdd]);
        assert_eq!(f.len(), 4 + 4 + 4);
        assert_eq!(
            u32::from_le_bytes([f[0], f[1], f[2], f[3]]),
            mcu_info_word(12, 1, 4)
        );
        assert_eq!(&f[4..8], &[0xaa, 0xbb, 0xcc, 0xdd]);
        assert_eq!(&f[8..12], &[0, 0, 0, 0]); // trailer
    }

    #[test]
    fn cmd_frame_pads_ragged_payload_with_zeros() {
        // 5 bytes -> 3 pad + 4 trailer, everything after the payload zero.
        let f = build_cmd_frame(1, 2, &[1, 2, 3, 4, 5]);
        assert_eq!(f.len(), 4 + 8 + 4);
        assert_eq!(&f[4..9], &[1, 2, 3, 4, 5]);
        assert!(f[9..].iter().all(|&b| b == 0));
    }

    #[test]
    fn fw_frame_is_fully_zeroed_past_the_payload() {
        // The divergence documented on build_fw_frame: upstream would leave up
        // to 3 bytes uninitialised here.
        let f = build_fw_frame(&[0xde, 0xad, 0xbe]);
        assert_eq!(f.len(), 4 + 4 + 4);
        assert_eq!(&f[4..7], &[0xde, 0xad, 0xbe]);
        assert!(f[7..].iter().all(|&b| b == 0));
    }

    #[test]
    fn full_fw_chunk_is_exactly_the_urb_payload_size() {
        // mt76x02_usb_mcu.c:256 (max_len = max_payload - 8) exists so that a
        // full chunk's frame fills kmalloc(max_payload) exactly.
        assert_eq!(FW_CHUNK_MAX, 0x38f8 - 8);
        assert_eq!(build_fw_frame(&vec![0u8; FW_CHUNK_MAX]).len(), 0x38f8);
    }

    #[test]
    fn reg_pairs_max_is_twentyfour() {
        // MT_INBAND_PACKET_MAX_LEN / 8 — mt76x02_usb_mcu.c:136, mt76x02_mcu.h:18.
        assert_eq!(regs::MT_INBAND_PACKET_MAX_LEN, 192);
        assert_eq!(MT_MCU_REGPAIRS_MAX, 24);
        assert_eq!(MT_MCU_REGPAIRS_MAX * 8, regs::MT_INBAND_PACKET_MAX_LEN);
    }

    #[test]
    fn encode_reg_pairs_adds_the_base_little_endian() {
        let pairs = [
            RegPair::new(0x1004, 0x0000_002c),
            RegPair::new(0x1008, 0xdead_beef),
        ];
        let buf = encode_reg_pairs(regs::MT_MCU_MEMMAP_WLAN, &pairs);
        assert_eq!(buf.len(), 16);
        assert_eq!(&buf[0..4], &0x0041_1004u32.to_le_bytes());
        assert_eq!(&buf[4..8], &0x0000_002cu32.to_le_bytes());
        assert_eq!(&buf[8..12], &0x0041_1008u32.to_le_bytes());
        assert_eq!(&buf[12..16], &0xdead_beefu32.to_le_bytes());
    }

    #[test]
    fn encode_reg_pairs_rf_base_wraps_as_upstream_does() {
        // MT_MCU_MEMMAP_RF is 0x80000000; base + reg must not be treated as an
        // overflow. mt76x0/phy.c:105-113 sends RF offsets against it.
        let buf = encode_reg_pairs(regs::MT_MCU_MEMMAP_RF, &[RegPair::new(0x0004_0002, 0x11)]);
        assert_eq!(&buf[0..4], &0x8004_0002u32.to_le_bytes());
        assert_eq!(&buf[4..8], &0x0000_0011u32.to_le_bytes());
    }

    /// Build a synthetic RANDOM_READ response: 4-byte FCE word, n pairs, 4-byte
    /// trailer (mt76x02_usb_mcu.c:29-33 walked from `data + 4`).
    fn fake_rd_resp(base: u32, pairs: &[(u32, u32)]) -> Vec<u8> {
        let mut v = Vec::new();
        v.extend_from_slice(&0u32.to_le_bytes()); // rxfce
        for (reg, val) in pairs {
            v.extend_from_slice(&base.wrapping_add(*reg).to_le_bytes());
            v.extend_from_slice(&val.to_le_bytes());
        }
        v.extend_from_slice(&0u32.to_le_bytes()); // trailer
        v
    }

    #[test]
    fn decode_reg_pairs_fills_values() {
        let base = regs::MT_MCU_MEMMAP_WLAN;
        let resp = fake_rd_resp(base, &[(0x1004, 0x1234_5678), (0x1008, 0x0000_000c)]);
        let mut out = [RegPair::new(0x1004, 0), RegPair::new(0x1008, 0)];
        decode_reg_pairs(base, &resp, &mut out).expect("decode");
        assert_eq!(out[0].value, 0x1234_5678);
        assert_eq!(out[1].value, 0x0000_000c);
    }

    #[test]
    fn decode_reg_pairs_round_trips_the_encoder() {
        let base = regs::MT_MCU_MEMMAP_RF;
        let want = [RegPair::new(0x30, 0x5a), RegPair::new(0x31, 0xa5)];
        // The response reuses the request's address encoding, so the encoder's
        // output is the middle of a well-formed response.
        let mut resp = vec![0u8; 4];
        resp.extend_from_slice(&encode_reg_pairs(base, &want));
        resp.extend_from_slice(&[0u8; 4]);
        let mut out = [RegPair::new(0x30, 0), RegPair::new(0x31, 0)];
        decode_reg_pairs(base, &resp, &mut out).expect("decode");
        assert_eq!(out, want);
    }

    #[test]
    fn decode_reg_pairs_rejects_a_misaligned_response() {
        // The case upstream only WARN_ON_ONCEs about (mt76x02_usb_mcu.c:32).
        let base = regs::MT_MCU_MEMMAP_WLAN;
        let resp = fake_rd_resp(base, &[(0x9999, 0xdead_beef)]);
        let mut out = [RegPair::new(0x1004, 0)];
        assert!(decode_reg_pairs(base, &resp, &mut out).is_err());
        assert_eq!(out[0].value, 0, "no value written on a mismatch");
    }

    #[test]
    fn decode_reg_pairs_rejects_a_short_response() {
        let base = regs::MT_MCU_MEMMAP_WLAN;
        let resp = fake_rd_resp(base, &[(0x1004, 1)]);
        let mut out = [RegPair::new(0x1004, 0), RegPair::new(0x1008, 0)];
        assert!(decode_reg_pairs(base, &resp, &mut out).is_err());
        assert!(decode_reg_pairs(base, &[0u8; 4], &mut out[..1]).is_err());
    }

    #[test]
    fn rxfce_accessors_match_the_genmasks() {
        // seq 5 at bits 19:16, evt EVT_CMD_DONE(0) at 23:20.
        assert_eq!(rxfce_seq(0x0005_0000), 5);
        assert_eq!(rxfce_evt(0x0005_0000), EVT_CMD_DONE);
        // evt EVT_CMD_ERROR(1), seq 9.
        assert_eq!(rxfce_seq(0x0019_0000), 9);
        assert_eq!(rxfce_evt(0x0019_0000), EVT_CMD_ERROR);
    }

    #[test]
    fn firmware_blob_header_matches_the_measured_image() {
        // ★ MEASURED from fw/mt76x0/mt7610u.bin: 80288 B total.
        let hdr = FwHeader::parse(MT7610U_FIRMWARE).expect("shipped firmware parses");
        assert_eq!(hdr.ilm_len, 0x0001_0cac);
        assert_eq!(hdr.dlm_len, 0x0000_2cd4);
        assert_eq!(hdr.build_ver, 0x7640);
        assert_eq!(hdr.fw_ver, 0x0100);
        assert_eq!(hdr.version_string(), "0.1.00-b7640");
        assert_eq!(hdr.build_time_str(), "201308221655");
        assert_eq!(
            MT7610U_FIRMWARE.len(),
            FW_HEADER_LEN + hdr.ilm_len as usize + hdr.dlm_len as usize
        );
        // Both regions are 4-aligned, so build_fw_frame never has to pad.
        assert_eq!(hdr.ilm_len % 4, 0);
        assert_eq!(hdr.dlm_len % 4, 0);
        // And the ILM is bigger than the IVB carved out of it.
        assert!(hdr.ilm_len > regs::MT_MCU_IVB_SIZE);
    }

    #[test]
    fn firmware_header_validation_rejects_bad_images() {
        assert!(
            FwHeader::parse(&[0u8; 8]).is_err(),
            "too short for a header"
        );

        // ilm_len <= MT_MCU_IVB_SIZE (mt76x0/usb_mcu.c:107-108).
        let mut img = vec![0u8; FW_HEADER_LEN + 0x40];
        img[0..4].copy_from_slice(&0x40u32.to_le_bytes());
        assert!(FwHeader::parse(&img).is_err(), "ilm_len == IVB size");

        // Size mismatch (mt76x0/usb_mcu.c:110-115).
        let mut img = vec![0u8; FW_HEADER_LEN + 100];
        img[0..4].copy_from_slice(&80u32.to_le_bytes());
        img[4..8].copy_from_slice(&40u32.to_le_bytes()); // 80 + 40 != 100
        assert!(FwHeader::parse(&img).is_err(), "declared lengths mismatch");
    }

    #[test]
    fn load_offsets_are_the_mt76x0_ones_not_the_mt76x2_ones() {
        // ILM lands at MT_MCU_IVB_SIZE (0x40), DLM at 0x80000. The mt76x2 pair
        // is 0x80000 / 0x110000 and must not leak into this port.
        assert_eq!(regs::MT_MCU_IVB_SIZE, 0x40);
        assert_eq!(regs::MT_MCU_DLM_OFFSET, 0x0008_0000);
    }
}
