//! Waveshare USB-TO-LoRa (SX1262) serial-bridged [`FrameIo`] backend.
//!
//! The dongle is a GD32F103 MCU + Semtech SX1262 behind a CH343 USB-UART, presenting
//! `/dev/ttyACM*` at 115200 8N1. In its default **transparent mode** (`AT+MODE=1`) it is a plain
//! byte pipe: bytes written to the UART are sent as a LoRa frame, and a received LoRa frame's
//! payload comes back out the UART. There is no board-side packet protocol (unlike the BW16
//! bridge, which speaks a typed frame), so this backend supplies its own length-delimited framing
//! to recover NDN packet boundaries from the byte stream, and configures the radio parameters
//! (spreading factor, bandwidth, channel, …) once at open via the module's `+++`/`AT` interface.
//!
//! This makes a long-range, low-rate, duty-cycled sub-GHz radio just another `FrameIo` — the same
//! seam the USB Wi-Fi drivers and the BW16 board sit behind. Its named-time surface is the honest
//! floor: no hardware timestamp, only a `HostRecv` stamp latched when the serial line delivers the
//! frame (see [`RadioTime`]).

use std::io::{Read, Write};
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

use async_trait::async_trait;
use ndn_frame_io::{
    CapturedFrame, ClockDomainId, FrameIo, InjectFrame, LatchPoint, LinkStamp, RadioCapability,
    RadioProfile, RadioTime, RadioTimeSource,
};
use ndn_transport::FaceError;
use tokio::sync::{Mutex as AsyncMutex, mpsc};

/// Baud the CH343↔GD32 link (and thus the host) runs at — the module's factory default.
pub const LORA_BAUD: u32 = 115_200;

/// Host clock domain for LoRa `HostRecv` stamps ("LORA"); the module exposes no hardware counter.
const HOST_CLOCK_DOMAIN: ClockDomainId = ClockDomainId(0x4C4F_5241);

/// Framing sync word ("ND") prefixing every host↔host NDN packet on the transparent byte stream.
const SYNC0: u8 = 0x4E;
const SYNC1: u8 = 0x44;

/// Largest NDN payload carried in one LoRa frame. The SX1262 LoRa PHY caps a frame near 255 bytes;
/// our 5-byte framing overhead (sync+len+cksum) leaves this so one framed packet is exactly one
/// air frame — keeping loss atomic (a dropped frame loses one packet, never desyncs a larger one).
pub const MAX_LORA_PAYLOAD: usize = 240;

/// A `HostRecv` [`LinkStamp`]: nanoseconds since process start (monotonic), latched when the serial
/// line delivered the frame — the coarsest but honest time a serial LoRa bridge can offer.
fn host_stamp() -> LinkStamp {
    static START: std::sync::OnceLock<Instant> = std::sync::OnceLock::new();
    let start = START.get_or_init(Instant::now);
    LinkStamp::new(
        start.elapsed().as_nanos() as u64,
        HOST_CLOCK_DOMAIN,
        LatchPoint::HostRecv.precision_floor_ns(),
        LatchPoint::HostRecv,
    )
}

/// Radio parameters programmed over the `AT` interface at open. Two modules must agree on all of
/// these to hear each other; [`Default`] is the dongle's factory pairing (SF7, 125 kHz, 4/5,
/// channel 18, network 0, 22 dBm), which two units share out of the box.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct LoraParams {
    /// Spreading factor 7–12 (`AT+SF`). Higher = longer range, exponentially slower.
    pub sf: u8,
    /// Bandwidth code (`AT+BW`): 0 = 125 kHz, 1 = 250 kHz, 2 = 500 kHz.
    pub bw: u8,
    /// Coding-rate code (`AT+CR`): 1 = 4/5 … 4 = 4/8.
    pub cr: u8,
    /// TX channel index (`AT+TXCH`).
    pub tx_ch: u8,
    /// RX channel index (`AT+RXCH`).
    pub rx_ch: u8,
    /// Network id (`AT+NETID`) — only same-netid modules pair.
    pub netid: u16,
    /// Module address (`AT+ADDR`).
    pub addr: u16,
    /// TX power, dBm (`AT+PWR`), 10–22.
    pub pwr: u8,
}

impl Default for LoraParams {
    fn default() -> Self {
        Self {
            sf: 7,
            bw: 0,
            cr: 1,
            tx_ch: 18,
            rx_ch: 18,
            netid: 0,
            addr: 0,
            pwr: 22,
        }
    }
}

/// A Waveshare USB-TO-LoRa dongle reached over its `/dev/ttyACM*` serial port.
pub struct LoraSerialBackend {
    tx: Arc<Mutex<Box<dyn serialport::SerialPort>>>,
    rx: AsyncMutex<mpsc::UnboundedReceiver<CapturedFrame>>,
    params: LoraParams,
}

impl LoraSerialBackend {
    /// Open the dongle at `path` with the factory-default (paired) parameters.
    pub fn open(path: &str) -> Result<Self, FaceError> {
        Self::open_with(path, LoraParams::default())
    }

    /// Open the dongle at `path`, program `params` over the `AT` interface, leave it in transparent
    /// data mode, and spawn the reader that deframes received NDN packets off the byte stream.
    pub fn open_with(path: &str, params: LoraParams) -> Result<Self, FaceError> {
        let mut port = serialport::new(path, LORA_BAUD)
            .timeout(Duration::from_millis(200))
            .open()
            .map_err(|e| io_err(format!("lora open {path}: {e}")))?;
        configure(&mut port, &params)?;
        let reader = port
            .try_clone()
            .map_err(|e| io_err(format!("lora clone: {e}")))?;
        let (txch, rxch) = mpsc::unbounded_channel();
        std::thread::spawn(move || reader_loop(reader, txch));
        Ok(Self {
            tx: Arc::new(Mutex::new(port)),
            rx: AsyncMutex::new(rxch),
            params,
        })
    }

    /// The radio parameters this backend programmed at open.
    pub fn params(&self) -> LoraParams {
        self.params
    }

    /// Frame and write one NDN payload as a single LoRa air frame.
    fn write_framed(&self, payload: &[u8]) -> Result<(), FaceError> {
        if payload.len() > MAX_LORA_PAYLOAD {
            return Err(io_err(format!(
                "lora payload {} > {MAX_LORA_PAYLOAD} (one frame)",
                payload.len()
            )));
        }
        let len = payload.len() as u16;
        let cksum = checksum(payload);
        let hdr = [SYNC0, SYNC1, (len & 0xff) as u8, (len >> 8) as u8];
        let mut port = self.tx.lock().unwrap();
        port.write_all(&hdr)
            .and_then(|_| port.write_all(payload))
            .and_then(|_| port.write_all(&[cksum]))
            .map_err(|e| io_err(format!("lora write: {e}")))
    }
}

/// Sum-of-bytes checksum (mod 256) over a framed payload — lets the reader drop a frame corrupted
/// by a partial on-air loss and resync on the next sync word rather than emit garbage.
fn checksum(payload: &[u8]) -> u8 {
    payload.iter().fold(0u8, |a, &b| a.wrapping_add(b))
}

/// Enter AT command mode, program the radio parameters, and return to transparent data mode.
///
/// Robust to the module's persisted mode (open does not reset the MCU): `+++\r\n` enters AT mode
/// from data mode, and merely errors (harmlessly) if already there, so after it we are always in
/// AT mode; `AT+EXIT` then drops back to transparent data mode for the payload stream.
fn configure(port: &mut Box<dyn serialport::SerialPort>, p: &LoraParams) -> Result<(), FaceError> {
    // Opening the port pulses DTR, which resets the GD32 MCU; the module ignores `+++` within 3 s
    // of power-on (datasheet), so settle past that window before entering command mode.
    let _ = port.clear(serialport::ClearBuffer::Input);
    std::thread::sleep(Duration::from_millis(3500));
    let _ = port.clear(serialport::ClearBuffer::Input);
    at(port, "+++");
    at(port, "AT+MODE=1");
    at(port, &format!("AT+SF={}", p.sf));
    at(port, &format!("AT+BW={}", p.bw));
    at(port, &format!("AT+CR={}", p.cr));
    at(port, &format!("AT+TXCH={}", p.tx_ch));
    at(port, &format!("AT+RXCH={}", p.rx_ch));
    at(port, &format!("AT+NETID={}", p.netid));
    at(port, &format!("AT+ADDR={}", p.addr));
    at(port, &format!("AT+PWR={}", p.pwr));
    at(port, "AT+EXIT");
    let _ = port.clear(serialport::ClearBuffer::Input);
    Ok(())
}

/// Send one `AT` command (CRLF-terminated) and drain its response until `OK`/`ERROR` or a short
/// deadline. Best-effort: a non-`OK` reply (e.g. `+++` returning `ERROR` when already in AT mode)
/// is not fatal — the sequence in [`configure`] converges regardless.
fn at(port: &mut Box<dyn serialport::SerialPort>, cmd: &str) {
    let _ = port.write_all(cmd.as_bytes());
    let _ = port.write_all(b"\r\n");
    let _ = port.flush();
    let deadline = Instant::now() + Duration::from_millis(800);
    let mut resp = Vec::new();
    let mut tmp = [0u8; 256];
    while Instant::now() < deadline {
        match port.read(&mut tmp) {
            Ok(n) if n > 0 => {
                resp.extend_from_slice(&tmp[..n]);
                if resp.windows(2).any(|w| w == b"OK" || w == b"OR") {
                    break; // "OK" or the tail of "ERROR"
                }
            }
            Ok(_) => {}
            Err(ref e) if e.kind() == std::io::ErrorKind::TimedOut => {}
            Err(_) => break,
        }
    }
    if std::env::var_os("LORA_AT_DEBUG").is_some() {
        let printable: String = resp
            .iter()
            .map(|&b| if (0x20..0x7f).contains(&b) { b as char } else { '.' })
            .collect();
        eprintln!("AT[{cmd}] <- {} bytes [{printable}]", resp.len());
    }
}

/// Background reader: accumulate serial bytes, deframe `[SYNC0 SYNC1 len16 payload cksum]` packets,
/// verify the checksum, and hand each recovered NDN payload up as a `CapturedFrame` stamped
/// `HostRecv` at delivery. A checksum miss (partial air loss) is dropped and resynced.
fn reader_loop(mut port: Box<dyn serialport::SerialPort>, tx: mpsc::UnboundedSender<CapturedFrame>) {
    let mut acc: Vec<u8> = Vec::new();
    let mut tmp = [0u8; 512];
    loop {
        match port.read(&mut tmp) {
            Ok(n) if n > 0 => {
                acc.extend_from_slice(&tmp[..n]);
                loop {
                    match deframe(&acc) {
                        Deframe::Frame { payload, consumed } => {
                            let cap = CapturedFrame {
                                payload: payload.into(),
                                addr: None,
                                group: None,
                                rssi_dbm: None,
                                mcs_index: None,
                                stamp: Some(host_stamp()),
                            };
                            acc.drain(..consumed);
                            if tx.send(cap).is_err() {
                                return; // backend dropped
                            }
                        }
                        Deframe::Drop { consumed } => {
                            acc.drain(..consumed);
                        }
                        Deframe::Need => break,
                    }
                }
                if acc.len() > 8192 {
                    acc.clear(); // bound the buffer mid-desync
                }
            }
            Ok(_) => {}
            Err(ref e) if e.kind() == std::io::ErrorKind::TimedOut => {}
            Err(_) => return,
        }
    }
}

/// Outcome of trying to deframe the front of the accumulator.
enum Deframe {
    /// A whole, checksum-valid packet; `consumed` bytes cover the sync word through the checksum.
    Frame { payload: Vec<u8>, consumed: usize },
    /// Bytes to discard (leading garbage before a sync word, or a checksum-failed frame).
    Drop { consumed: usize },
    /// Not enough bytes yet — read more.
    Need,
}

/// Parse one framed packet from the front of `buf`.
fn deframe(buf: &[u8]) -> Deframe {
    let Some(start) = buf.windows(2).position(|w| w == [SYNC0, SYNC1]) else {
        // No sync word; keep only a possible trailing lone SYNC0 for the next read.
        let keep = if buf.last() == Some(&SYNC0) { 1 } else { 0 };
        return if buf.len() > keep {
            Deframe::Drop {
                consumed: buf.len() - keep,
            }
        } else {
            Deframe::Need
        };
    };
    if start > 0 {
        return Deframe::Drop { consumed: start }; // discard leading garbage
    }
    if buf.len() < 4 {
        return Deframe::Need;
    }
    let len = (buf[2] as usize) | ((buf[3] as usize) << 8);
    let total = 4 + len + 1; // sync(2) + len(2) + payload + cksum(1)
    if len > MAX_LORA_PAYLOAD {
        return Deframe::Drop { consumed: 2 }; // implausible length — skip this sync word
    }
    if buf.len() < total {
        return Deframe::Need;
    }
    let payload = &buf[4..4 + len];
    if checksum(payload) != buf[4 + len] {
        return Deframe::Drop { consumed: 2 }; // corrupt — skip past this sync word and resync
    }
    Deframe::Frame {
        payload: payload.to_vec(),
        consumed: total,
    }
}

#[async_trait]
impl FrameIo for LoraSerialBackend {
    async fn inject(&self, frame_in: InjectFrame) -> Result<(), FaceError> {
        // The payload is the NDN packet itself — LoRa carries no link addressing here, so `dst`/
        // `src`/`tx` are advisory only. Transparent mode sends it as one air frame.
        self.write_framed(&frame_in.payload)
    }

    async fn recv_frame(&self) -> Result<CapturedFrame, FaceError> {
        let mut rx = self.rx.lock().await;
        rx.recv().await.ok_or(FaceError::Closed)
    }
}

/// Reference [`RadioTime`]: the LoRa bridge reports no hardware timestamp, so its only honest link
/// clock is the host monotonic clock read when the serial line delivered the frame. Readable on
/// demand, so `read_clock` returns it. (A [`ndn_radio_hal::FaceTimeProfile`] derived from this
/// reports `HostRecv` precision and `can_common_view = false`.)
impl RadioTime for LoraSerialBackend {
    fn time_sources(&self) -> Vec<RadioTimeSource> {
        vec![RadioTimeSource::host_recv(HOST_CLOCK_DOMAIN)]
    }

    fn read_clock(&self, domain: ClockDomainId) -> Result<Option<u64>, FaceError> {
        Ok((domain == HOST_CLOCK_DOMAIN).then(|| host_stamp().raw))
    }
}

impl RadioProfile for LoraSerialBackend {
    fn capability(&self) -> RadioCapability {
        // Sub-GHz LoRa: the channel the module is tuned to, duty-cycled, half-duplex, ~256 B frames.
        RadioCapability::lora(vec![self.params.tx_ch])
    }
}

fn io_err(msg: String) -> FaceError {
    FaceError::Io(std::io::Error::other(msg))
}

#[cfg(test)]
mod tests {
    use super::*;

    fn frame_bytes(payload: &[u8]) -> Vec<u8> {
        let len = payload.len() as u16;
        let mut v = vec![SYNC0, SYNC1, (len & 0xff) as u8, (len >> 8) as u8];
        v.extend_from_slice(payload);
        v.push(checksum(payload));
        v
    }

    #[test]
    fn deframes_a_clean_packet() {
        let buf = frame_bytes(b"hello-ndn");
        match deframe(&buf) {
            Deframe::Frame { payload, consumed } => {
                assert_eq!(payload, b"hello-ndn");
                assert_eq!(consumed, buf.len());
            }
            _ => panic!("expected a frame"),
        }
    }

    #[test]
    fn skips_leading_garbage_then_frames() {
        let mut buf = vec![0x00, 0x11, 0x22];
        buf.extend_from_slice(&frame_bytes(b"abc"));
        // First: drop the 3 garbage bytes.
        match deframe(&buf) {
            Deframe::Drop { consumed } => buf.drain(..consumed),
            _ => panic!("expected drop of garbage"),
        };
        match deframe(&buf) {
            Deframe::Frame { payload, .. } => assert_eq!(payload, b"abc"),
            _ => panic!("expected frame after garbage"),
        }
    }

    #[test]
    fn partial_frame_needs_more() {
        let full = frame_bytes(b"waiting");
        assert!(matches!(deframe(&full[..6]), Deframe::Need));
    }

    #[test]
    fn corrupt_checksum_is_dropped_and_resyncs() {
        let mut buf = frame_bytes(b"payload");
        let last = buf.len() - 1;
        buf[last] ^= 0xff; // wreck the checksum
        // Append a good frame after the corrupt one.
        let good = frame_bytes(b"ok");
        buf.extend_from_slice(&good);
        // Corrupt frame → drop past its sync word (2 bytes).
        match deframe(&buf) {
            Deframe::Drop { consumed } => {
                assert_eq!(consumed, 2);
                buf.drain(..consumed);
            }
            _ => panic!("expected drop of corrupt frame"),
        }
        // Resync: find the good frame.
        loop {
            match deframe(&buf) {
                Deframe::Frame { payload, .. } => {
                    assert_eq!(payload, b"ok");
                    break;
                }
                Deframe::Drop { consumed } => {
                    buf.drain(..consumed);
                }
                Deframe::Need => panic!("lost the good frame"),
            }
        }
    }

    #[test]
    fn checksum_round_trips() {
        assert_eq!(checksum(&[1, 2, 3]), 6);
        assert_eq!(checksum(&[0xff, 0x01]), 0x00);
    }
}
