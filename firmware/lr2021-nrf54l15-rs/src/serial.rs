//! The **7E-A5 host protocol** — the same wire contract the Waveshare (SX1262) and Heltec (SX1276)
//! nodes speak, so one host driver talks to every sub-GHz/2.4 GHz node in the rig.
//!
//! ```text
//!   0x7E 0xA5 <type> <len> <payload…> <crc>       crc = XOR of type, len and every payload byte
//! ```
//!
//! Deliberately identical to `waveshare-lora-rs`. A second, subtly different framing would not fail
//! loudly — the host would simply mis-parse one node — so the constants below are copied verbatim
//! and any change must be made in both places. Only the subset meaningful for an FLRC link is
//! implemented here; the LoRa-only commands (spreading factor, CAD config, SF scan) are rejected
//! rather than silently ignored, so a host that assumes them gets an error instead of silence.

/// Frame start, byte 0.
pub const SYNC0: u8 = 0x7E;
/// Frame start, byte 1.
pub const SYNC1: u8 = 0xA5;

// ── Host → node ────────────────────────────────────────────────────────────────────────────────
/// payload = frame bytes to transmit.
pub const CMD_TX: u8 = 0x01;
/// payload = u32 BE Hz.
pub const CMD_SET_FREQ: u8 = 0x02;
/// payload = [i8 dBm].
pub const CMD_SET_PWR: u8 = 0x04;
/// payload = []; replies [`EVT_INFO`].
pub const CMD_GET_INFO: u8 = 0x06;
/// payload = []; replies [`EVT_RSSI`].
pub const CMD_GET_RSSI: u8 = 0x09;
/// payload = [u64 BE hash]* — empty clears (pass-all).
pub const CMD_SET_NAME_FILTER: u8 = 0x0F;
/// payload = [u64 BE hash]* — the relay set; empty clears.
pub const CMD_SET_RELAY: u8 = 0x10;
/// payload = [cs_serve, dedup, hop_on, hop_base_ch, hop_span].
pub const CMD_DATAPLANE: u8 = 0x11;
/// payload = []; replies [`EVT_STATS`].
pub const CMD_GET_STATS: u8 = 0x13;
/// payload = [] — clear all counters.
pub const CMD_RESET_STATS: u8 = 0x14;

// ── Node → host ────────────────────────────────────────────────────────────────────────────────
/// payload = [rssi i16 BE, snr i16 BE, ts_us u32 BE, frame bytes].
///
/// **`ts_us` is the hardware capture** (M4): latched by DPPI at the DIO edge, 62.5 ns resolution,
/// not a host-side or task-side stamp. The Waveshare node fills this field with a millisecond
/// software counter; the same field carries a far better number here, and the host should treat its
/// precision as a per-node property rather than assume the worst case everywhere.
pub const EVT_RX: u8 = 0x81;
/// payload = [ok, attempts].
pub const EVT_TXDONE: u8 = 0x82;
/// payload = [status, fw_major, fw_minor, freq(4 BE), pwr, bitrate_code].
pub const EVT_INFO: u8 = 0x83;
/// payload = ascii.
pub const EVT_LOG: u8 = 0x84;
/// payload = [rssi i16 BE].
pub const EVT_RSSI: u8 = 0x86;
/// payload = counters, see the bridge.
pub const EVT_STATS: u8 = 0x87;
/// payload = [cmd, reason] — a command this node does not implement (e.g. a LoRa-only knob).
pub const EVT_UNSUPPORTED: u8 = 0x8F;

/// Largest payload accepted in either direction.
pub const MAX_PAYLOAD: usize = 255;

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
        Self { state: 0, typ: 0, len: 0, idx: 0, crc: 0, buf: [0; MAX_PAYLOAD] }
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
                self.state = if b == SYNC1 { 2 } else if b == SYNC0 { 1 } else { 0 };
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
