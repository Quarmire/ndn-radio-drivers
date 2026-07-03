//! BW16 (RTL8720DN) serial-bridged monitor-mode backend.
//!
//! A BW16 running the `firmware/bw16-ndn-bridge` sketch is a dual-band 802.11
//! injector/capturer driven over USB-serial. The host builds the *same* 802.11
//! frame (`ndn_frame_io::frame::build_dot11`) the USB drivers build, ships the
//! bytes to the board to inject raw, and parses captured frames back — so a
//! `MonitorWifiFace` over a BW16 is just another [`FrameIo`] backend, and the
//! NAN engine / cognition above it are none the wiser. The proof that the HAL
//! seam accommodates a radically different radio (an ARM Cortex-M board on a
//! serial tether, not a USB host driver).

use std::io::{Read, Write};
use std::sync::{Arc, Mutex};
use std::time::Duration;

use async_trait::async_trait;
use ndn_frame_io::{CapturedFrame, FrameFormat, FrameIo, InjectFrame, McsDescriptor, WifiRadio, frame};
use ndn_radio_hal::{Bandwidth, RadioKnobs};
use ndn_transport::FaceError;
use tokio::sync::{Mutex as AsyncMutex, mpsc};

const SYNC0: u8 = 0x4E;
const SYNC1: u8 = 0x44;
const T_INJECT: u8 = 0x01;
const T_CHANNEL: u8 = 0x02;
const T_TXPOWER: u8 = 0x03; // wext "txpower patha=N" power index
const T_RATE: u8 = 0x04; // wifi_set_tx_data_rate code
const T_BW40: u8 = 0x05; // wext_set_bw40_enable
const T_RX: u8 = 0x81;

/// BW16 fixed TX-rate codes for [`Bw16SerialBackend::set_tx_rate`]
/// (`wifi_set_tx_data_rate`). CCK 0x00–0x03, OFDM 0x04–0x0b, HT MCS0–7 0x0c–0x13,
/// `0xFF` = auto rate adaptation.
pub mod rate {
    pub const CCK_1M: u8 = 0x00;
    pub const CCK_11M: u8 = 0x03;
    pub const OFDM_6M: u8 = 0x04;
    pub const OFDM_54M: u8 = 0x0b;
    pub const HT_MCS0: u8 = 0x0c;
    pub const HT_MCS7: u8 = 0x13;
    pub const AUTO: u8 = 0xFF;
}

/// Baud the firmware opens `Serial` at — the RTL8720 LOG UART's native rate,
/// shared with WiFi-driver debug (the deframer picks our SYNC'd frames out).
pub const BW16_BAUD: u32 = 115_200;

/// A BW16 reached over its USB-serial port.
pub struct Bw16SerialBackend {
    tx: Arc<Mutex<Box<dyn serialport::SerialPort>>>,
    format: FrameFormat,
    rx: AsyncMutex<mpsc::UnboundedReceiver<CapturedFrame>>,
}

impl Bw16SerialBackend {
    /// Open the BW16 at `path` (e.g. `/dev/tty.usbserial-XXXX`) and spawn the RX
    /// reader that deframes captured 802.11 frames off the serial link.
    pub fn open(path: &str) -> Result<Self, FaceError> {
        let mut port = serialport::new(path, BW16_BAUD)
            .timeout(Duration::from_millis(50))
            .open()
            .map_err(|e| io_err(format!("bw16 open {path}: {e}")))?;
        // Reset the board into our firmware via DTR→CEN so a freshly-flashed board
        // comes up without a manual reset: settle low, pulse the reset, then let it
        // boot (~900 ms to `ready`).
        let _ = port.write_request_to_send(false);
        let _ = port.write_data_terminal_ready(false);
        std::thread::sleep(Duration::from_millis(150));
        let _ = port.write_data_terminal_ready(true);
        std::thread::sleep(Duration::from_millis(120));
        let _ = port.write_data_terminal_ready(false);
        std::thread::sleep(Duration::from_millis(1000));
        let _ = port.clear(serialport::ClearBuffer::Input);
        let reader = port
            .try_clone()
            .map_err(|e| io_err(format!("bw16 clone: {e}")))?;
        let (txch, rxch) = mpsc::unbounded_channel();
        let format = FrameFormat::default();
        std::thread::spawn(move || reader_loop(reader, format, txch));
        Ok(Self {
            tx: Arc::new(Mutex::new(port)),
            format,
            rx: AsyncMutex::new(rxch),
        })
    }

    /// Select the on-air frame format built on the host (default `RawNdn`).
    pub fn with_format(mut self, format: FrameFormat) -> Self {
        self.format = format;
        self
    }

    fn send_framed(&self, ty: u8, payload: &[u8]) -> Result<(), FaceError> {
        let len = payload.len() as u16;
        let hdr = [SYNC0, SYNC1, ty, (len & 0xff) as u8, (len >> 8) as u8];
        let mut port = self.tx.lock().unwrap();
        port.write_all(&hdr)
            .and_then(|_| port.write_all(payload))
            .map_err(|e| io_err(format!("bw16 write: {e}")))
    }

    /// Retune the board's radio (2.4 or 5 GHz channel).
    pub fn set_channel(&self, channel: u8) -> Result<(), FaceError> {
        self.send_framed(T_CHANNEL, &[channel])
    }

    /// Pin the on-air TX rate (a [`rate`] code; `wifi_set_tx_data_rate`). Whether
    /// it affects the raw-inject path is empirical — verify by capturing the MCS.
    pub fn set_tx_rate(&self, code: u8) -> Result<(), FaceError> {
        self.send_framed(T_RATE, &[code])
    }

    /// Enable/disable 40 MHz channel bandwidth (`wext_set_bw40_enable`).
    pub fn set_bw40(&self, enable: bool) -> Result<(), FaceError> {
        self.send_framed(T_BW40, &[enable as u8])
    }

    /// Set the TX power index (wext `txpower patha=<idx>`). Only reachable on the
    /// Rust firmware, which reimplements the SDK's `#if 0`-disabled wifi_set_txpower.
    pub fn set_txpower(&self, idx: u8) -> Result<(), FaceError> {
        self.send_framed(T_TXPOWER, &[idx])
    }
}

/// Background reader: accumulate serial bytes, deframe RX packets, parse each as
/// an 802.11 frame in `format`, and hand the `CapturedFrame`s to `recv_frame`.
fn reader_loop(
    mut port: Box<dyn serialport::SerialPort>,
    format: FrameFormat,
    tx: mpsc::UnboundedSender<CapturedFrame>,
) {
    let mut acc: Vec<u8> = Vec::new();
    let mut tmp = [0u8; 2048];
    loop {
        match port.read(&mut tmp) {
            Ok(n) if n > 0 => {
                acc.extend_from_slice(&tmp[..n]);
                while let Some((ty, payload, consumed)) = deframe(&acc) {
                    if ty == T_RX && !payload.is_empty() {
                        let rssi = payload[0] as i8;
                        if let Some(cap) = frame::parse_dot11(format, &payload[1..], Some(rssi), None) {
                            if tx.send(cap).is_err() {
                                return; // backend dropped
                            }
                        }
                    }
                    acc.drain(..consumed);
                }
                // Bound the buffer if we're mid-desync.
                if acc.len() > 8192 {
                    acc.clear();
                }
            }
            Ok(_) => {}
            Err(ref e) if e.kind() == std::io::ErrorKind::TimedOut => {}
            Err(_) => return,
        }
    }
}

/// Parse one `[SYNC0 SYNC1 type len_le16 payload]` frame from the front of `buf`;
/// returns `(type, payload, bytes_consumed_up_to_and_including_it)`.
fn deframe(buf: &[u8]) -> Option<(u8, Vec<u8>, usize)> {
    let start = buf.windows(2).position(|w| w == [SYNC0, SYNC1])?;
    let rest = &buf[start..];
    if rest.len() < 5 {
        return None;
    }
    let ty = rest[2];
    let len = (rest[3] as usize) | ((rest[4] as usize) << 8);
    if rest.len() < 5 + len {
        return None;
    }
    Some((ty, rest[5..5 + len].to_vec(), start + 5 + len))
}

#[async_trait]
impl FrameIo for Bw16SerialBackend {
    async fn inject(&self, frame_in: InjectFrame) -> Result<(), FaceError> {
        // Build the 802.11 frame on the host — identical to the USB backends —
        // then hand the raw bytes to the board to inject (it adds FCS + seq).
        let dot11 = frame::build_dot11(self.format, &frame_in)?;
        self.send_framed(T_INJECT, &dot11)
    }

    async fn recv_frame(&self) -> Result<CapturedFrame, FaceError> {
        let mut rx = self.rx.lock().await;
        rx.recv().await.ok_or(FaceError::Closed)
    }
}

#[async_trait]
impl WifiRadio for Bw16SerialBackend {
    async fn inject_at(&self, frame_in: InjectFrame, _mcs: McsDescriptor) -> Result<(), FaceError> {
        // The Ameba management-TX path picks the rate; the exact MCS doesn't apply.
        self.inject(frame_in).await
    }
}

impl RadioKnobs for Bw16SerialBackend {
    fn set_channel(&self, channel: u8, bw: Bandwidth) -> Result<(), FaceError> {
        Bw16SerialBackend::set_channel(self, channel)?;
        // Map the HAL bandwidth to the board's 40 MHz enable (the widest this SDK
        // exposes): Bw20 → off, wider → on.
        self.set_bw40(!matches!(bw, Bandwidth::Bw20))
    }
    fn set_tx_power(&self, idx: u32) -> Result<(), FaceError> {
        // Reachable on the Rust firmware (reimplemented txpower). Index is clamped
        // into a byte for the wext command.
        self.set_txpower(idx.min(u8::MAX as u32) as u8)
    }
}

fn io_err(msg: String) -> FaceError {
    FaceError::Io(std::io::Error::other(msg))
}
