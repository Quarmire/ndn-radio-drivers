//! The **7E-A5 host protocol, v2** — the same wire contract the Waveshare (SX1262) and Heltec
//! (SX1276) nodes speak, so one host driver talks to every sub-GHz/2.4 GHz node in the rig.
//!
//! ```text
//!   0x7E 0xA5 <type> <len> <payload…> <crc>       crc = XOR of type, len and every payload byte
//! ```
//!
//! Deliberately identical to `waveshare-lora-rs`. A second, subtly different framing would not fail
//! loudly — the host would simply mis-parse one node — so the constants below are copied verbatim
//! and **any change must be made in both places**.
//!
//! ## What v2 adds, and why
//!
//! v1 let a host talk to a node it already knew everything about. That does not survive a fleet of
//! five nodes on three different radio parts: the host had to hard-code which knobs each node has,
//! which units they are in, and how good its timestamps are — and every one of those assumptions
//! was wrong for at least one node. v2 adds [`CMD_GET_CAP`]/[`EVT_CAP`], where a node states its
//! own capabilities in fields with **fixed units**, plus the three primitives the MAC work needs
//! from every node ([`CMD_READ_CLOCK`], [`CMD_SENSE`], [`CMD_TX_AT`]).
//!
//! **Every field of [`EVT_CAP`] must be true for this node.** Where nothing is known the field is 0
//! and the reason is in a comment — a fabricated number is worse than a zero, because the host will
//! believe it and calibrate against it.
//!
//! ## Never silence
//!
//! Any opcode this node does not implement is answered [`EVT_UNSUPPORTED`] `[cmd, reason]`. A host
//! that assumes a knob gets an error instead of a four-attempt retry loop that ends in a timeout,
//! which is the difference between a diagnosable bug and a mystery. LoRa-only commands (spreading
//! factor, LoRa CAD tuning, SF scan) are rejected here for exactly that reason.

/// Frame start, byte 0.
pub const SYNC0: u8 = 0x7E;
/// Frame start, byte 1.
pub const SYNC1: u8 = 0xA5;

/// Protocol version reported in [`EVT_CAP`].
pub const PROTO_VER: u8 = 2;

// ── Host → node ────────────────────────────────────────────────────────────────────────────────
/// payload = frame bytes to transmit; replies [`EVT_TXDONE`] `[ok, 0]`.
///
/// `ok` means **the chip reported TxDone**, not "the SPI writes succeeded" — the difference matters,
/// because the bug that motivated most of this file made every transmit after a retune fail while
/// still reporting success. A payload longer than [`Capabilities::max_payload`] is refused with
/// `ok = 0` and never truncated: a silently shortened NDN packet is worse than a refused transmit.
pub const CMD_TX: u8 = 0x01;
/// payload = u32 BE Hz; replies [`EVT_INFO`], or [`EVT_UNSUPPORTED`] if out of the built band.
pub const CMD_SET_FREQ: u8 = 0x02;
/// payload = `[sf, bw_code, cr_code]` on a LoRa node.
///
/// **On this node (`radio_kind` = 2, FLRC) the triple is `[rate_code, _, cr_code]`**, because FLRC
/// has no spreading factor and its bandwidth is not an independent knob — it is implied by the
/// bitrate rung. The byte positions are kept so one host code path drives every node; what each
/// slot *means* is what `radio_kind` tells the host.
///
/// * `rate_code` 0..7 → `FlrcBitrate::{Br2600, Br2080, Br1300, Br1040, Br0650, Br0520, Br0325,
///   Br0260}` — the chip's own encoding (`vendor/lr2021/src/cmd/cmd_flrc.rs`), not a re-mapping.
/// * `bw_code` ignored; echoed back as 0 in [`EVT_INFO`].
/// * `cr_code` 0..3 → `FlrcCr::{Cr12, Cr34, None, Cr23}` — again the chip's own encoding, so
///   `1` is 3/4 and `2` is FEC OFF. Counter-intuitive, and it is what the silicon uses.
///
/// Replies [`EVT_INFO`]; a bad code replies [`EVT_UNSUPPORTED`] `[cmd, REASON_OUT_OF_RANGE]`.
pub const CMD_SET_MOD: u8 = 0x03;
/// payload = `[i8 dBm]` — **real dBm**, converted to the chip's half-dB register unit by the node
/// and clamped to the PA's range. Replies [`EVT_INFO`] carrying the value actually applied.
pub const CMD_SET_PWR: u8 = 0x04;
/// payload = []; replies [`EVT_INFO`].
pub const CMD_GET_INFO: u8 = 0x06;
/// payload = []; replies [`EVT_CAD`] `[busy]`. On this node the sense is an **energy-detect CCA**
/// (the LR2021 has no LoRa CAD in FLRC mode), thresholded at the `CMD_SET_SENSE_CFG` level.
pub const CMD_CAD: u8 = 0x08;
/// payload = []; replies [`EVT_RSSI`] `[rssi i16 BE dBm]`.
pub const CMD_GET_RSSI: u8 = 0x09;
/// payload = `[cw_ms u16 BE, max_backoff, max_attempts]`; replies [`EVT_INFO`].
pub const CMD_SET_LBT_CFG: u8 = 0x0B;
/// payload = frame bytes; atomic sense → randomised backoff → key-up. Replies
/// [`EVT_TXDONE`] `[sent, attempts]`.
pub const CMD_TX_LBT: u8 = 0x0E;
/// payload = [u64 BE hash]* — empty clears (pass-all); replies [`EVT_INFO`].
pub const CMD_SET_NAME_FILTER: u8 = 0x0F;
/// payload = [u64 BE hash]* — the relay set; empty clears. Replies [`EVT_INFO`].
pub const CMD_SET_RELAY: u8 = 0x10;
/// payload = `[cs_serve, dedup, hop_on, hop_base_ch, hop_span]`; replies [`EVT_INFO`].
/// `hop_on != 0` is rejected here — see the bridge, this bearer has no channel-index convention.
pub const CMD_DATAPLANE: u8 = 0x11;
/// payload = `[rssi_thresh i16 BE dBm, cad_repeat]`; replies [`EVT_INFO`].
pub const CMD_SET_SENSE_CFG: u8 = 0x12;
/// payload = []; replies [`EVT_STATS`].
pub const CMD_GET_STATS: u8 = 0x13;
/// payload = [] — clear every counter; replies [`EVT_INFO`].
pub const CMD_RESET_STATS: u8 = 0x14;
/// **v2.** payload = []; replies [`EVT_CLOCK`] `[ticks u64 BE]` on the same counter [`EVT_RX`]
/// stamps with, so a host can relate an RX stamp to "now" on one timebase.
pub const CMD_READ_CLOCK: u8 = 0x17;
/// **v2.** payload = `[delay_us u32 BE][frame]` — hardware-scheduled transmit; replies
/// [`EVT_TXDONE`]. Answered [`EVT_UNSUPPORTED`] on this board: see the bridge for the exact
/// peripheral conflict, and note [`EVT_CAP`]`.sched_gran_ns` is 0 to match.
pub const CMD_TX_AT: u8 = 0x18;
/// **v2.** payload = []; replies [`EVT_CAP`].
pub const CMD_GET_CAP: u8 = 0x1A;
/// **v2.** payload = []; replies [`EVT_SENSE`].
pub const CMD_SENSE: u8 = 0x1B;

// ── Node → host ────────────────────────────────────────────────────────────────────────────────
/// payload = `[rssi i16 BE dBm, snr i16 BE dB, ts u32 BE, frame bytes]`.
///
/// **`ts` is TICKS, not microseconds** — at [`EVT_CAP`]`.stamp_hz` (16 MHz here, so 62.5 ns per
/// tick), latched by DPPI at the DIO edge. The field was documented as `ts_us` while carrying
/// ticks, which is a 16× error waiting to happen in whichever host reads it first; the wire layout
/// is unchanged, only the documentation was wrong. The Waveshare node fills the same field with a
/// millisecond software counter and reports `stamp_hz = 1000`, which is exactly why the units have
/// to travel in [`EVT_CAP`] rather than being assumed.
pub const EVT_RX: u8 = 0x81;
/// payload = `[ok, attempts]`. `attempts` is 0 for a plain [`CMD_TX`].
pub const EVT_TXDONE: u8 = 0x82;
/// payload, **19 bytes**, matching the fleet:
/// `[status, sync(2), errors(2), freq(4), sf, bw, cr, pwr, lost(2), cad_busy(2), defer(2)]`.
///
/// Per-field meaning on this node — see [`CMD_SET_MOD`] for why the `sf`/`bw`/`cr` slots carry a
/// bitrate rung rather than a spreading factor:
///
/// | field | here |
/// |---|---|
/// | `status` | `chip_mode | (cmd_status << 4)` from the LR2021 status word |
/// | `sync` | low 16 bits of the 32-bit FLRC syncword |
/// | `errors` | the chip's sticky error flags, packed — see [`err_bits`] |
/// | `freq` | current carrier, Hz |
/// | `sf` | FLRC bitrate code 0..7 (**not** a spreading factor; there is none) |
/// | `bw` | 0 — FLRC bandwidth is implied by the bitrate rung, not an independent knob |
/// | `cr` | `FlrcCr` code 0..3 |
/// | `pwr` | TX power **actually applied**, in dBm |
/// | `lost` | 0 — the buffered UARTE ring exposes no overrun count on this MCU |
/// | `cad_busy` | senses that came back busy |
/// | `defer` | transmissions abandoned after `max_attempts` |
pub const EVT_INFO: u8 = 0x83;
/// payload = ascii.
pub const EVT_LOG: u8 = 0x84;
/// payload = `[busy(0/1)]`.
pub const EVT_CAD: u8 = 0x85;
/// payload = `[rssi i16 BE dBm]`.
pub const EVT_RSSI: u8 = 0x86;
/// payload = `[sf | 0]`. **Fleet number, unimplemented here** — FLRC has no spreading factor.
/// Listed so nobody re-uses 0x87 for something else, which is exactly what happened once: this
/// firmware defined `EVT_STATS = 0x87` while the host mapped 0x89 to it and 0x87 to SF-detected, so
/// a stats read on this node decoded as an SF scan on the host.
pub const EVT_SF_DETECTED: u8 = 0x87;
/// payload = `[airtime_ms u16 BE]`. Fleet number; not emitted here.
pub const EVT_TX_STARTED: u8 = 0x88;
/// payload, **24 bytes**:
/// `[rx(4), filtered(4), deduped(4), served(4), relayed(4), cad_busy(2), defer(2)]`, all BE.
///
/// `rx` counts frames the on-device data plane **classified** — a frame that is not in the app wire
/// shape (`KIND|SRC|SF|NAME|…`) is delivered without being counted, exactly as on the Waveshare
/// node. One meaning per field across the fleet matters more than a second, node-local definition.
pub const EVT_STATS: u8 = 0x89;
/// **v2.** payload = `[ticks u64 BE]`, units = [`EVT_CAP`]`.stamp_hz`.
pub const EVT_CLOCK: u8 = 0x8A;
/// **v2.** payload = 29 bytes, see [`Capabilities`].
pub const EVT_CAP: u8 = 0x8B;
/// **v2.** payload = `[activity u16 BE, rssi i16 BE dBm]`.
///
/// `activity` is a **free-running, wrapping** count of channel-busy observations. The host
/// differences two reads over a window; it is never an absolute occupancy. Wrapping is deliberate:
/// a saturating counter silently stops measuring, and a host differencing a stuck counter reads a
/// busy channel as idle.
pub const EVT_SENSE: u8 = 0x8C;
/// payload = `[cmd, reason]` — see [`REASON_NOT_IMPLEMENTED`] and friends.
pub const EVT_UNSUPPORTED: u8 = 0x8F;

// `EVT_UNSUPPORTED` reason codes. **Fleet-wide space** — these are the same four values the
// Waveshare and Heltec nodes use (`UNSUP_*` in their `main.rs`). They were 1/2/3 =
// not-implemented/param/hardware here, which put `NO_HARDWARE` and `BAD_LENGTH` in each other's
// slots relative to the other two nodes: a host decoding the byte would have read this node's
// `CMD_TX_AT` refusal as "you sent a short payload". Nothing on the wire distinguishes the two
// conventions, so the majority spelling is the fleet's.
/// This firmware does not know the opcode at all.
pub const REASON_UNKNOWN_OPCODE: u8 = 1;
/// The opcode is understood and **this board's wiring cannot do it**. Distinct from
/// [`REASON_UNKNOWN_OPCODE`] because it is not a firmware gap: no amount of firmware fixes it.
pub const REASON_NO_HARDWARE: u8 = 2;
/// The opcode is understood but the payload does not satisfy its argument requirements.
pub const REASON_BAD_LENGTH: u8 = 3;
/// An argument outside the range [`EVT_CAP`] advertises — out of band, an unknown rate code.
pub const REASON_OUT_OF_RANGE: u8 = 4;

/// Largest payload accepted in either direction **on the serial link**. Not the on-air cap — that
/// is [`Capabilities::max_payload`], which is far smaller here.
pub const MAX_PAYLOAD: usize = 255;

/// Pack the LR2021's sticky error flags into the 16-bit `errors` field of [`EVT_INFO`].
///
/// Bit order is fixed here so a host decoding it does not have to know the driver's struct layout:
/// 0 hf_xosc, 1 lf_xosc, 2 pll_lock, 3 lf_rc_calib, 4 hf_rc_calib, 5 pll_calib, 6 aaf_calib,
/// 7 img_calib, 8 chip_busy, 9 rxfreq_no_fe_cal, 10 meas_unit_adc_calib, 11 pa_offset_calib,
/// 12 ppf_calib, 13 src_calib.
pub const fn err_bits(flags: [bool; 14]) -> u16 {
    let mut v = 0u16;
    let mut i = 0;
    while i < 14 {
        if flags[i] {
            v |= 1 << i;
        }
        i += 1;
    }
    v
}

// ── EVT_CAP ────────────────────────────────────────────────────────────────────────────────────

/// Wire length of [`EVT_CAP`].
pub const CAP_LEN: usize = 29;

/// `radio_kind` values. The host keys unit and semantic decisions on this, so it is the one field
/// that must never be approximated.
pub mod radio_kind {
    pub const SX1262: u8 = 0;
    pub const SX1276: u8 = 1;
    pub const LR2021_FLRC: u8 = 2;
    pub const LR2021_LORA: u8 = 3;
}

/// `stamp_kind` values — how good the [`EVT_RX`] `ts` field actually is.
pub mod stamp_kind {
    /// No per-frame stamp at all; `stamp_hz` is 0.
    pub const NONE: u8 = 0;
    /// Latched when the serial line delivered the frame to the host.
    pub const HOST_RECV: u8 = 1;
    /// A counter incremented by firmware — carries task/interrupt latency.
    pub const SOFTWARE_COUNTER: u8 = 2;
    /// Latched in silicon at the radio's own event edge, free-running counter.
    pub const HARDWARE_FREE_RUNNING: u8 = 3;
}

/// One bit per implemented opcode, `bit N == opcode N`. Every opcode is < 32 by construction.
///
/// Derived from the constants above rather than written out, so the bitmap cannot drift from the
/// dispatch table the way a hand-maintained list does. **Anything added here must be answered by
/// the bridge**, and anything the bridge answers `EVT_UNSUPPORTED` must be absent.
pub const CMD_BITMAP: u32 = (1 << CMD_TX)
    | (1 << CMD_SET_FREQ)
    | (1 << CMD_SET_MOD)
    | (1 << CMD_SET_PWR)
    | (1 << CMD_GET_INFO)
    | (1 << CMD_CAD)
    | (1 << CMD_GET_RSSI)
    | (1 << CMD_SET_LBT_CFG)
    | (1 << CMD_TX_LBT)
    | (1 << CMD_SET_NAME_FILTER)
    | (1 << CMD_SET_RELAY)
    | (1 << CMD_DATAPLANE)
    | (1 << CMD_SET_SENSE_CFG)
    | (1 << CMD_GET_STATS)
    | (1 << CMD_RESET_STATS)
    | (1 << CMD_READ_CLOCK)
    | (1 << CMD_GET_CAP)
    | (1 << CMD_SENSE);
// Deliberately ABSENT, and each absence is answered EVT_UNSUPPORTED rather than ignored:
//   0x05 CMD_SET_SYNC        — a one-byte LoRa sync word does not map onto a 32-bit FLRC syncword
//   0x07 CMD_SET_BEACON      — no beacon task on this node
//   0x0A CMD_SET_CAD_CFG     — LoRa CAD detector peak/min; this part energy-detects instead
//   0x0C CMD_SET_PREAMBLE    — FLRC preamble is 4..32 *bits* in 4-bit steps, not LoRa symbols
//   0x0D CMD_SF_SCAN         — no spreading factor
//   0x15 CMD_SET_DEBUG       — no EVT_LOG diagnostics wired here yet
//   0x16 CMD_ENTER_BOOTLOADER— the XIAO reflashes over its own CMSIS-DAP probe, not a ROM loader
//   0x18 CMD_TX_AT           — HARDWARE conflict, see the bridge; sched_gran_ns is 0 to match

/// A node's self-description — the one place a node states what it is, in fixed units.
///
/// Every multi-byte field is **big-endian** on the wire.
///
/// ```text
///   [0]      proto_ver
///   [1]      radio_kind
///   [2..6]   freq_min_hz  u32
///   [6..10]  freq_max_hz  u32
///   [10]     pwr_min_dbm  i8     ** real dBm, never a chip register unit **
///   [11]     pwr_max_dbm  i8
///   [12..16] stamp_hz     u32    ticks/second of the EVT_RX ts field; 0 = no per-frame stamp
///   [16]     stamp_kind
///   [17..19] max_payload  u16    the REAL end-to-end cap
///   [19..23] cmd_bitmap   u32
///   [23]     sf_min              0 when the node has no spreading factor
///   [24]     sf_max
///   [25..29] sched_gran_ns u32   0 = no hardware-scheduled TX
/// ```
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub struct Capabilities {
    pub proto_ver: u8,
    pub radio_kind: u8,
    pub freq_min_hz: u32,
    pub freq_max_hz: u32,
    pub pwr_min_dbm: i8,
    pub pwr_max_dbm: i8,
    pub stamp_hz: u32,
    pub stamp_kind: u8,
    /// The **real** end-to-end cap: `min(TX accept, RX buffer, on-air PDU)`. A node that accepts
    /// 240 bytes on TX and truncates RX at 64 reports 64 — the smaller side, always.
    pub max_payload: u16,
    pub cmd_bitmap: u32,
    pub sf_min: u8,
    pub sf_max: u8,
    pub sched_gran_ns: u32,
}

impl Capabilities {
    /// Serialise to the 29 wire bytes.
    pub fn to_bytes(&self) -> [u8; CAP_LEN] {
        let mut b = [0u8; CAP_LEN];
        b[0] = self.proto_ver;
        b[1] = self.radio_kind;
        b[2..6].copy_from_slice(&self.freq_min_hz.to_be_bytes());
        b[6..10].copy_from_slice(&self.freq_max_hz.to_be_bytes());
        b[10] = self.pwr_min_dbm as u8;
        b[11] = self.pwr_max_dbm as u8;
        b[12..16].copy_from_slice(&self.stamp_hz.to_be_bytes());
        b[16] = self.stamp_kind;
        b[17..19].copy_from_slice(&self.max_payload.to_be_bytes());
        b[19..23].copy_from_slice(&self.cmd_bitmap.to_be_bytes());
        b[23] = self.sf_min;
        b[24] = self.sf_max;
        b[25..29].copy_from_slice(&self.sched_gran_ns.to_be_bytes());
        b
    }

    /// Parse the 29 wire bytes back. Exists so the encoder can be round-trip tested on the host;
    /// nothing on the device decodes an `EVT_CAP`.
    pub fn from_bytes(b: &[u8]) -> Option<Self> {
        if b.len() < CAP_LEN {
            return None;
        }
        let u32be = |o: usize| u32::from_be_bytes([b[o], b[o + 1], b[o + 2], b[o + 3]]);
        Some(Self {
            proto_ver: b[0],
            radio_kind: b[1],
            freq_min_hz: u32be(2),
            freq_max_hz: u32be(6),
            pwr_min_dbm: b[10] as i8,
            pwr_max_dbm: b[11] as i8,
            stamp_hz: u32be(12),
            stamp_kind: b[16],
            max_payload: u16::from_be_bytes([b[17], b[18]]),
            cmd_bitmap: u32be(19),
            sf_min: b[23],
            sf_max: b[24],
            sched_gran_ns: u32be(25),
        })
    }
}

/// Incremental frame parser: feed bytes, get whole frames.
///
/// A state machine rather than a buffer-and-scan so a partial frame across UART reads cannot be
/// lost, and so a stray `0x7E` inside a payload cannot resynchronise the parser mid-frame.
pub struct Parser {
    state: u8,
    typ: u8,
    len: u8,
    idx: usize,
    crc: u8,
    buf: [u8; MAX_PAYLOAD],
}

impl Default for Parser {
    fn default() -> Self {
        Self::new()
    }
}

impl Parser {
    pub const fn new() -> Self {
        Self {
            state: 0,
            typ: 0,
            len: 0,
            idx: 0,
            crc: 0,
            buf: [0; MAX_PAYLOAD],
        }
    }

    /// Feed one byte. Returns `Some((type, payload))` on a complete, CRC-valid frame.
    pub fn push(&mut self, b: u8) -> Option<(u8, &[u8])> {
        match self.state {
            0 => {
                if b == SYNC0 {
                    self.state = 1;
                }
            }
            1 => {
                // A second 0x7E keeps us waiting for 0xA5 rather than resetting: back-to-back frame
                // starts are otherwise mis-parsed.
                self.state = if b == SYNC1 {
                    2
                } else if b == SYNC0 {
                    1
                } else {
                    0
                };
            }
            2 => {
                self.typ = b;
                self.crc = b;
                self.state = 3;
            }
            3 => {
                self.len = b;
                self.crc ^= b;
                self.idx = 0;
                self.state = if b == 0 { 5 } else { 4 };
            }
            4 => {
                if self.idx < self.buf.len() {
                    self.buf[self.idx] = b;
                }
                self.crc ^= b;
                self.idx += 1;
                if self.idx as u8 >= self.len {
                    self.state = 5;
                }
            }
            _ => {
                self.state = 0;
                if b == self.crc {
                    let n = (self.len as usize).min(self.buf.len());
                    return Some((self.typ, &self.buf[..n]));
                }
                // CRC mismatch: drop silently and resynchronise. A corrupt frame must never reach
                // the radio as a transmit request.
            }
        }
        None
    }
}

/// Emit one frame through a byte sink.
pub fn write_frame<F: FnMut(u8)>(mut out: F, typ: u8, payload: &[u8]) {
    out(SYNC0);
    out(SYNC1);
    out(typ);
    out(payload.len() as u8);
    let mut crc = typ ^ (payload.len() as u8);
    for &b in payload {
        out(b);
        crc ^= b;
    }
    out(crc);
}

/// Encode a frame into `dst`, returning the byte count (0 if it would not fit).
pub fn encode(dst: &mut [u8], typ: u8, payload: &[u8]) -> usize {
    let need = 5 + payload.len();
    if dst.len() < need {
        return 0;
    }
    let mut i = 0;
    write_frame(
        |b| {
            dst[i] = b;
            i += 1;
        },
        typ,
        payload,
    );
    i
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Encode → parse must be the identity for every frame the node emits, INCLUDING the ones whose
    /// payload contains the frame-start bytes. This is not hypothetical: an `EVT_RX` carrying a
    /// whitened FLRC payload is uniformly distributed, so `7E A5` appears inside payloads routinely,
    /// and a parser that resynchronised on it would corrupt roughly one frame in 2^16.
    fn round_trip(typ: u8, payload: &[u8]) {
        let mut buf = [0u8; 5 + MAX_PAYLOAD];
        let n = encode(&mut buf, typ, payload);
        assert_eq!(n, 5 + payload.len());
        let mut p = Parser::new();
        let mut got = None;
        for (i, &b) in buf[..n].iter().enumerate() {
            if let Some((t, pl)) = p.push(b) {
                assert_eq!(i, n - 1, "frame completed early");
                got = Some((t, pl.to_vec()));
            }
        }
        let (t, pl) = got.expect("no frame parsed");
        assert_eq!(t, typ);
        assert_eq!(pl, payload);
    }

    #[test]
    fn parser_round_trips_every_event_shape() {
        round_trip(EVT_TXDONE, &[1, 0]);
        round_trip(CMD_GET_INFO, &[]);
        round_trip(EVT_INFO, &[0u8; 19]);
        round_trip(EVT_STATS, &[0xAAu8; 24]);
        round_trip(EVT_CAP, &[0x5Au8; CAP_LEN]);
        round_trip(EVT_CLOCK, &[0, 0, 0, 0, 0xDE, 0xAD, 0xBE, 0xEF]);
        round_trip(EVT_SENSE, &[0xFF, 0xFE, 0xFF, 0xA2]);
        // Payload containing the frame-start pair and a lone SYNC0.
        round_trip(EVT_RX, &[SYNC0, SYNC1, 0x00, SYNC0, 0x7E, 0x7E, 0xA5]);
        // Longest legal payload.
        round_trip(EVT_LOG, &[0x41u8; MAX_PAYLOAD]);
    }

    #[test]
    fn parser_rejects_a_corrupt_crc() {
        let mut buf = [0u8; 32];
        let n = encode(&mut buf, EVT_TXDONE, &[1, 0]);
        buf[n - 1] ^= 0xFF; // clobber the CRC
        let mut p = Parser::new();
        for &b in &buf[..n] {
            assert!(
                p.push(b).is_none(),
                "a CRC-corrupt frame must never be delivered"
            );
        }
    }

    #[test]
    fn parser_skips_leading_garbage() {
        let mut buf = [0u8; 32];
        let n = encode(&mut buf, CMD_TX, b"hi");
        let mut p = Parser::new();
        // Line noise with a lone SYNC0 in it — the state machine must fall back to hunting rather
        // than sit waiting for a payload that is not coming.
        for &b in b"\x00\xff\x7e\x00\xff\x7e" {
            assert!(p.push(b).is_none());
        }
        let mut got = false;
        for &b in &buf[..n] {
            if let Some((t, pl)) = p.push(b) {
                assert_eq!(t, CMD_TX);
                assert_eq!(pl, b"hi");
                got = true;
            }
        }
        assert!(got);
    }

    /// **Resynchronisation is bounded, not immediate**, and the bound is worth knowing.
    ///
    /// A `7E A5` that appears in line noise (or as the tail of a truncated frame) puts the parser
    /// into a real frame's state machine: the next two bytes become a type and a *length*, and the
    /// parser then consumes up to 255 payload bytes plus a CRC before it can look for a frame start
    /// again. So a frame arriving immediately behind a bogus start is **eaten**, and this test
    /// pins how much stream it takes to get back in step: at most `2 + 255 + 1` bytes.
    ///
    /// Two consequences a reader should have in front of them rather than rediscover:
    /// * the resync can emit at most one *spurious* frame (a length and a CRC that happen to agree,
    ///   as an all-zero run does) — which is why an unrecognised opcode must be answered
    ///   `EVT_UNSUPPORTED` and never acted on;
    /// * this is inherent to length-prefixed framing with no inter-frame timeout, and is the reason
    ///   the CRC is checked before a frame is ever handed to the radio.
    #[test]
    fn parser_resynchronises_within_a_bounded_window() {
        let mut p = Parser::new();
        for &b in b"\x7e\x7e\xa5" {
            let _ = p.push(b); // a bogus frame start: the parser is now mid-"frame"
        }
        // Enough stream to flush the worst-case bogus length. Spurious frames here are allowed;
        // being permanently desynchronised is not.
        for _ in 0..(2 + 255 + 1) {
            let _ = p.push(0x00);
        }
        let mut buf = [0u8; 32];
        let n = encode(&mut buf, CMD_TX, b"hi");
        let mut got = false;
        for &b in &buf[..n] {
            if let Some((t, pl)) = p.push(b) {
                assert_eq!(t, CMD_TX);
                assert_eq!(pl, b"hi");
                got = true;
            }
        }
        assert!(got, "parser never resynchronised");
    }

    /// The exact 29 bytes the LF (915 MHz) build of `m6_bridge` emits, pinned so a change to any
    /// contributing constant — band, PA range, `TICKS_PER_US`, `FRAME_LEN`, the opcode set — shows
    /// up here rather than as a host that quietly mis-sizes packets or mis-scales a timestamp.
    ///
    /// ```text
    ///   02 02 35 C3 6D 80 37 50 28 00 F7 16 00 F4 24 00 03 00 2F 0C 9F CB 5E 00 00 00 00 00 00
    ///   ^  ^  \__ 902 MHz __/ \__ 928 MHz __/ ^  ^  \_ 16 MHz __/ ^  \_47_/ \_ bitmap _/ ^ ^ \_ 0 _/
    ///   |  radio_kind = LR2021-FLRC        -9 +22 dBm         stamp_kind=3    sf_min/max  sched_gran
    ///   proto_ver = 2
    /// ```
    #[test]
    fn capabilities_wire_bytes_for_this_node() {
        let cap = Capabilities {
            proto_ver: PROTO_VER,
            radio_kind: radio_kind::LR2021_FLRC,
            freq_min_hz: 902_000_000,
            freq_max_hz: 928_000_000,
            pwr_min_dbm: -9,
            pwr_max_dbm: 22,
            stamp_hz: 16_000_000,
            stamp_kind: stamp_kind::HARDWARE_FREE_RUNNING,
            max_payload: 47,
            cmd_bitmap: CMD_BITMAP,
            sf_min: 0,
            sf_max: 0,
            sched_gran_ns: 0,
        };
        assert_eq!(CMD_BITMAP, 0x0C9F_CB5E);
        assert_eq!(
            cap.to_bytes(),
            [
                0x02, 0x02, 0x35, 0xC3, 0x6D, 0x80, 0x37, 0x50, 0x28, 0x00, 0xF7, 0x16, 0x00, 0xF4,
                0x24, 0x00, 0x03, 0x00, 0x2F, 0x0C, 0x9F, 0xCB, 0x5E, 0x00, 0x00, 0x00, 0x00, 0x00,
                0x00,
            ]
        );
        // …and the whole 7E-A5 frame the host sees.
        let mut buf = [0u8; 64];
        let n = encode(&mut buf, EVT_CAP, &cap.to_bytes());
        assert_eq!(&buf[..5], &[0x7E, 0xA5, 0x8B, 0x1D, 0x02]);
        assert_eq!(buf[n - 1], 0xD9);
    }

    #[test]
    fn capabilities_round_trip() {
        let cap = Capabilities {
            proto_ver: PROTO_VER,
            radio_kind: radio_kind::LR2021_FLRC,
            freq_min_hz: 902_000_000,
            freq_max_hz: 928_000_000,
            pwr_min_dbm: -9,
            pwr_max_dbm: 22,
            stamp_hz: 16_000_000,
            stamp_kind: stamp_kind::HARDWARE_FREE_RUNNING,
            max_payload: 47,
            cmd_bitmap: CMD_BITMAP,
            sf_min: 0,
            sf_max: 0,
            sched_gran_ns: 0,
        };
        let b = cap.to_bytes();
        assert_eq!(b.len(), CAP_LEN);
        assert_eq!(Capabilities::from_bytes(&b), Some(cap));
        // Field offsets are the wire contract; pin the ones a host indexes directly.
        assert_eq!(b[0], 2);
        assert_eq!(b[1], 2);
        assert_eq!(b[10] as i8, -9);
        assert_eq!(u32::from_be_bytes([b[12], b[13], b[14], b[15]]), 16_000_000);
        assert_eq!(b[16], 3);
        assert_eq!(u16::from_be_bytes([b[17], b[18]]), 47);
        assert_eq!(u32::from_be_bytes([b[25], b[26], b[27], b[28]]), 0);
    }

    /// The bitmap and the `EVT_UNSUPPORTED` list must partition the opcode space: a bit set for a
    /// command the bridge rejects is a lie the host will act on.
    #[test]
    fn cmd_bitmap_matches_the_documented_set() {
        for op in [
            CMD_TX,
            CMD_SET_FREQ,
            CMD_SET_MOD,
            CMD_SET_PWR,
            CMD_GET_INFO,
            CMD_CAD,
            CMD_GET_RSSI,
            CMD_SET_LBT_CFG,
            CMD_TX_LBT,
            CMD_SET_NAME_FILTER,
            CMD_SET_RELAY,
            CMD_DATAPLANE,
            CMD_SET_SENSE_CFG,
            CMD_GET_STATS,
            CMD_RESET_STATS,
            CMD_READ_CLOCK,
            CMD_GET_CAP,
            CMD_SENSE,
        ] {
            assert!(
                CMD_BITMAP & (1 << op) != 0,
                "opcode {op:#04x} implemented but not in bitmap"
            );
        }
        // CMD_TX_AT is a HARDWARE refusal on this board (one DIO pin, see the bridge), so the bit
        // must stay clear and sched_gran_ns must stay 0.
        assert_eq!(CMD_BITMAP & (1 << CMD_TX_AT), 0);
        for op in [0x05u8, 0x07, 0x0A, 0x0C, 0x0D, 0x15, 0x16] {
            assert_eq!(
                CMD_BITMAP & (1 << op),
                0,
                "opcode {op:#04x} is rejected but claimed"
            );
        }
    }

    #[test]
    fn error_bit_packing_is_positional() {
        let mut f = [false; 14];
        f[9] = true; // rxfreq_no_fe_cal
        assert_eq!(err_bits(f), 1 << 9);
        assert_eq!(err_bits([true; 14]), 0x3FFF);
        assert_eq!(err_bits([false; 14]), 0);
    }

    /// Event numbering is shared across four firmwares; a collision here is a silent mis-decode on
    /// the host, which is exactly how `EVT_STATS = 0x87` survived.
    #[test]
    fn fleet_event_numbering() {
        assert_eq!(
            [
                EVT_RX,
                EVT_TXDONE,
                EVT_INFO,
                EVT_LOG,
                EVT_CAD,
                EVT_RSSI,
                EVT_SF_DETECTED,
                EVT_TX_STARTED,
                EVT_STATS,
                EVT_CLOCK,
                EVT_CAP,
                EVT_SENSE,
                EVT_UNSUPPORTED
            ],
            [
                0x81, 0x82, 0x83, 0x84, 0x85, 0x86, 0x87, 0x88, 0x89, 0x8A, 0x8B, 0x8C, 0x8F
            ]
        );
    }
}
