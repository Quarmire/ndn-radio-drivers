//! Waveshare USB-TO-LoRa (SX1262) serial-bridged [`FrameIo`] backend.
//!
//! The dongle is a GD32F103 MCU + Semtech SX1262 behind a CH343 USB-UART, presenting
//! `/dev/ttyACM*` (Linux) / `/dev/cu.usbmodem*` (macOS) at 115200 8N1. It runs our **open Rust
//! firmware** (`firmware/waveshare-lora-rs`), which replaced the closed factory firmware: the air
//! side is now plain **standard LoRa** (no proprietary header — interoperates with any SX127x/
//! SX126x peer), and the host link is a small **binary protocol** — `7E A5 | type | len | payload |
//! xor-crc` — carrying commands (transmit a frame; set frequency / SF-BW-CR / power / sync word;
//! query info) and events (received frame **with RSSI + SNR**; TX-done; info; ascii log).
//!
//! Because the board now does its own LoRa framing, this backend no longer needs the transparent-
//! mode workarounds the closed firmware forced (COBS to survive a `0x00`-truncating byte pipe, an
//! `AT`/`+++` command mode, a DTR-reset settle): it just frames commands and parses events. Knobs
//! are binary commands the firmware applies live, so there is no command-mode/data-mode switch and
//! the reader never has to park. RX frames arrive with a real `rssi_dbm` (the closed path had none).
//!
//! This keeps a long-range, low-rate, duty-cycled sub-GHz radio as just another `FrameIo` — the
//! same seam the USB Wi-Fi drivers and the BW16 board sit behind. Its named-time surface is the
//! honest floor: no hardware timestamp, only a `HostRecv` stamp latched when the serial line
//! delivers the frame (see [`RadioTime`]).

use std::os::unix::io::RawFd;
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

use async_trait::async_trait;
use ndn_frame_io::{
    CapturedFrame, ClockDomainId, FrameIo, InjectFrame, LatchPoint, LinkStamp, RadioCapability,
    RadioProfile, RadioTime, RadioTimeSource,
};
use ndn_radio_hal::{Bandwidth, RadioKnobs, TxDiscipline};
use ndn_transport::FaceError;
use tokio::sync::{Mutex as AsyncMutex, mpsc};

/// Baud the CH343↔GD32 link (and thus the host) runs at.
pub const LORA_BAUD: u32 = 115_200;

/// Host clock domain for LoRa `HostRecv` stamps ("LORA"); the module exposes no hardware counter.
const HOST_CLOCK_DOMAIN: ClockDomainId = ClockDomainId(0x4C4F_5241);

// --- Host ⇄ firmware binary protocol (mirrors firmware/waveshare-lora-rs) ---
const SYNC0: u8 = 0x7E;
const SYNC1: u8 = 0xA5;
// Host -> firmware commands.
const CMD_TX: u8 = 0x01; //       payload = LoRa frame bytes
const CMD_SET_FREQ: u8 = 0x02; //  payload = u32 BE Hz
const CMD_SET_MOD: u8 = 0x03; //   payload = [sf, bw_code, cr_code]
const CMD_SET_PWR: u8 = 0x04; //   payload = [i8 dBm]
const CMD_SET_SYNC: u8 = 0x05; //  payload = [sx127x sync byte]
#[allow(dead_code)]
const CMD_GET_INFO: u8 = 0x06; //  payload = []
const CMD_SET_BEACON: u8 = 0x07; // payload = [enabled(0/1)] (opt [enabled, period_mult])
// Firmware -> host events.
const EVT_RX: u8 = 0x81; //     payload = [rssi i16 BE, snr i16 BE, ts_ms u32 BE, LoRa bytes] (#52)
const EVT_TXDONE: u8 = 0x82; // payload = [ok, attempts]  (#52: attempts=0 for a plain CMD_TX)
const EVT_INFO: u8 = 0x83; //   payload = [status, sync(2), errors(2), freq(4), sf, bw, cr, pwr, lost(2), cad_busy(2), defer(2)]
const EVT_LOG: u8 = 0x84; //    payload = ascii
// #52 events (only parsed when the LBT/CAD host paths are wired; harmless to receive otherwise).
const EVT_CAD: u8 = 0x85; //    payload = [busy(0/1)]
const EVT_RSSI: u8 = 0x86; //   payload = [rssi i16 BE]
const EVT_SF_DETECTED: u8 = 0x87; // payload = [sf | 0]
const EVT_TX_STARTED: u8 = 0x88; //  payload = [airtime_ms u16 BE]

/// Largest NDN payload carried in one LoRa frame. The SX1262 LoRa PHY caps a frame near 255 bytes;
/// keeping a margin means one NDN packet is exactly one air frame — loss stays atomic (a dropped
/// frame loses one packet, never desyncs a larger one).
pub const MAX_LORA_PAYLOAD: usize = 240;

/// The Waveshare/firmware channel convention: channel index → carrier = `(850 + ch)` MHz
/// (ch 18 = 868 EU, ch 65 = 915 US). Kept so cognition can keep thinking in channels.
fn channel_to_hz(ch: u8) -> u32 {
    (850 + ch as u32) * 1_000_000
}

/// Host bandwidth code (0/1/2 = 125/250/500 kHz) → SX1262 modulation bandwidth code.
fn bw_to_fw(bw: u8) -> u8 {
    match bw {
        0 => 0x04, // 125 kHz
        1 => 0x05, // 250 kHz
        _ => 0x06, // 500 kHz
    }
}

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

/// Radio parameters programmed over the binary protocol at open. Two dongles must agree on
/// frequency, SF, BW, CR and sync word to hear each other; [`Default`] is 915 MHz (US) / SF7 /
/// 125 kHz / 4-5 / private sync — matching the firmware defaults and the Heltec interop node.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct LoraParams {
    /// Spreading factor 7–12. Higher = longer range, exponentially slower.
    pub sf: u8,
    /// Bandwidth code: 0 = 125 kHz, 1 = 250 kHz, 2 = 500 kHz.
    pub bw: u8,
    /// Coding-rate code: 1 = 4/5 … 4 = 4/8.
    pub cr: u8,
    /// TX channel index; carrier = `(850 + tx_ch)` MHz.
    pub tx_ch: u8,
    /// RX channel index; LoRa is half-duplex so this tracks `tx_ch`.
    pub rx_ch: u8,
    /// TX power, dBm, 10–22.
    pub pwr: u8,
    /// LoRa sync word in the SX127x single-byte convention: 0x12 private / 0x34 public.
    pub sync: u8,
    /// Emit the firmware's on-air heartbeat beacon. Off by default when a host drives the dongle
    /// (its own traffic is liveness enough); set true to keep the node discoverable on-air.
    pub beacon: bool,
}

impl Default for LoraParams {
    fn default() -> Self {
        Self {
            sf: 7,
            bw: 0,
            cr: 1,
            tx_ch: 65, // 915 MHz (US ISM)
            rx_ch: 65,
            pwr: 22,
            sync: 0x12,
            beacon: false, // host-driven: silence the firmware beacon at open
        }
    }
}

/// A raw serial tty opened with libc termios — a `cfmakeraw` / `CLOCAL` / `8N1` port, exactly what
/// `stty raw clocal` gives. The `serialport` crate does not exchange bytes with the CH340 bridge
/// on aarch64-musl (the OPi target), so we own the termios setup ourselves; this also drops the
/// `serialport` dependency for the LoRa backend and cross-compiles cleanly.
struct SerialFd {
    fd: RawFd,
}

impl SerialFd {
    fn open(path: &str, baud: u32) -> std::io::Result<Self> {
        let cpath = std::ffi::CString::new(path)
            .map_err(|_| std::io::Error::other("path has a NUL byte"))?;
        // O_NONBLOCK during open avoids blocking on carrier-detect; cleared right after so reads
        // then block under VMIN/VTIME control.
        let fd = unsafe {
            libc::open(
                cpath.as_ptr(),
                libc::O_RDWR | libc::O_NOCTTY | libc::O_NONBLOCK,
            )
        };
        if fd < 0 {
            return Err(std::io::Error::last_os_error());
        }
        unsafe {
            let fl = libc::fcntl(fd, libc::F_GETFL);
            libc::fcntl(fd, libc::F_SETFL, fl & !libc::O_NONBLOCK);
        }
        let me = SerialFd { fd };
        me.set_termios(baud)?;
        Ok(me)
    }

    fn set_termios(&self, baud: u32) -> std::io::Result<()> {
        let speed: libc::speed_t = match baud {
            9600 => libc::B9600,
            19200 => libc::B19200,
            38400 => libc::B38400,
            57600 => libc::B57600,
            _ => libc::B115200,
        };
        unsafe {
            let mut t: libc::termios = std::mem::zeroed();
            if libc::tcgetattr(self.fd, &mut t) != 0 {
                return Err(std::io::Error::last_os_error());
            }
            libc::cfmakeraw(&mut t); // 8N1, no echo, no canon, no flow xlate
            libc::cfsetispeed(&mut t, speed);
            libc::cfsetospeed(&mut t, speed);
            t.c_cflag |= libc::CLOCAL | libc::CREAD; // ignore modem lines, enable receiver
            t.c_cflag &= !libc::CRTSCTS; // no hardware flow control
            t.c_cc[libc::VMIN] = 0; // read returns after VTIME even with no data…
            t.c_cc[libc::VTIME] = 2; // …a 0.2 s inter-read timeout
            if libc::tcsetattr(self.fd, libc::TCSANOW, &t) != 0 {
                return Err(std::io::Error::last_os_error());
            }
            libc::tcflush(self.fd, libc::TCIOFLUSH);
        }
        Ok(())
    }

    fn flush_input(&self) {
        unsafe {
            libc::tcflush(self.fd, libc::TCIFLUSH);
        }
    }

    fn read(&self, buf: &mut [u8]) -> std::io::Result<usize> {
        loop {
            let n = unsafe { libc::read(self.fd, buf.as_mut_ptr() as *mut libc::c_void, buf.len()) };
            if n < 0 {
                let e = std::io::Error::last_os_error();
                if e.kind() == std::io::ErrorKind::Interrupted {
                    continue;
                }
                return Err(e);
            }
            return Ok(n as usize);
        }
    }

    fn write_all(&self, mut buf: &[u8]) -> std::io::Result<()> {
        while !buf.is_empty() {
            let n = unsafe { libc::write(self.fd, buf.as_ptr() as *const libc::c_void, buf.len()) };
            if n < 0 {
                let e = std::io::Error::last_os_error();
                if e.kind() == std::io::ErrorKind::Interrupted {
                    continue;
                }
                return Err(e);
            }
            buf = &buf[n as usize..];
        }
        Ok(())
    }

    fn flush(&self) -> std::io::Result<()> {
        unsafe {
            libc::tcdrain(self.fd);
        }
        Ok(())
    }

    fn try_clone(&self) -> std::io::Result<SerialFd> {
        let fd = unsafe { libc::dup(self.fd) };
        if fd < 0 {
            Err(std::io::Error::last_os_error())
        } else {
            Ok(SerialFd { fd })
        }
    }
}

impl Drop for SerialFd {
    fn drop(&mut self) {
        unsafe {
            libc::close(self.fd);
        }
    }
}

/// How long to wait for a transmission to complete. The firmware's transmit polls the chip for at
/// most ~2 s (SF12 at 125 kHz is ~1.5 s on air for a full frame); the rest is serial slack.
const TXDONE_TIMEOUT: Duration = Duration::from_millis(3_000);
/// How long to wait for a knob to be acknowledged. Each `SET_*` is standby → apply → re-arm RX →
/// reply, all well under this.
const INFO_TIMEOUT: Duration = Duration::from_millis(1_000);
/// How many times to re-send an idempotent knob whose reply never came. See `exec_idempotent`.
const KNOB_ATTEMPTS: usize = 4;

/// The command side of the link: the port, plus the firmware's replies to whatever we last sent.
///
/// **This protocol is strictly request/response, one command in flight.** Every handler in the
/// firmware runs to completion before the main loop drains the USART again — and the STM32F1's
/// USART has no FIFO, so a second command sent while the first is still being serviced is not
/// queued, it is destroyed by an overrun that neither side reports. A blocking transmit is the
/// worst case (up to ~2 s on air), but even `SET_MOD` (standby → apply → re-arm → reply) is long
/// enough to swallow the command behind it. So: hold this lock, send, and wait for the reply.
struct CmdPort {
    port: SerialFd,
    /// `(event type, payload)` for every non-RX event the reader parses.
    resp: std::sync::mpsc::Receiver<(u8, Vec<u8>)>,
}

/// A Waveshare USB-TO-LoRa dongle (open firmware) reached over its serial port.
pub struct LoraSerialBackend {
    cmd: Arc<Mutex<CmdPort>>,
    rx: AsyncMutex<mpsc::UnboundedReceiver<CapturedFrame>>,
    /// Behind a mutex because [`RadioKnobs`] retunes the module at runtime; `capability()` and
    /// `params()` reflect the live values.
    params: Arc<Mutex<LoraParams>>,
}

impl LoraSerialBackend {
    /// Open the dongle at `path` with the default (915 MHz / SF7) parameters.
    pub fn open(path: &str) -> Result<Self, FaceError> {
        Self::open_with(path, LoraParams::default())
    }

    /// Open the dongle at `path`, program `params` over the binary protocol, and spawn the reader
    /// that parses events and hands received NDN frames (with RSSI/SNR) up as [`CapturedFrame`]s.
    pub fn open_with(path: &str, params: LoraParams) -> Result<Self, FaceError> {
        let port =
            SerialFd::open(path, LORA_BAUD).map_err(|e| io_err(format!("lora open {path}: {e}")))?;
        configure(&port, &params)?;
        let reader = port
            .try_clone()
            .map_err(|e| io_err(format!("lora clone: {e}")))?;
        let (txch, rxch) = mpsc::unbounded_channel();
        let (respch, resprx) = std::sync::mpsc::channel();
        std::thread::spawn(move || reader_loop(reader, txch, respch));
        Ok(Self {
            cmd: Arc::new(Mutex::new(CmdPort { port, resp: resprx })),
            rx: AsyncMutex::new(rxch),
            params: Arc::new(Mutex::new(params)),
        })
    }

    /// The radio parameters currently programmed (reflects runtime [`RadioKnobs`] changes).
    pub fn params(&self) -> LoraParams {
        *self.params.lock().unwrap()
    }

    /// Toggle the firmware's on-air heartbeat beacon at runtime (off by default under host control).
    pub fn set_beacon(&self, on: bool) -> Result<(), FaceError> {
        self.exec_idempotent(CMD_SET_BEACON, &[on as u8], EVT_INFO)?;
        self.params.lock().unwrap().beacon = on;
        Ok(())
    }

    /// Send an **idempotent** command, retrying if the firmware never answers.
    ///
    /// The firmware polls its FIFO-less USART from the main loop, so a byte that lands while it is
    /// busy (an SPI `poll_rx`, a previous handler, a transmission) is simply gone — commands are
    /// lost at a measurable rate. Re-sending a `SET_*` is harmless because it re-asserts a state
    /// rather than causing an event, so retry it. `CMD_TX` gets no retry: a lost *reply* is
    /// indistinguishable from a lost *command*, and re-sending would risk a duplicate frame on air.
    /// The real repair is interrupt/DMA-driven RX in the firmware; this only buys reliability
    /// against a defect the host cannot fix.
    fn exec_idempotent(&self, typ: u8, payload: &[u8], expect: u8) -> Result<Vec<u8>, FaceError> {
        let mut last = None;
        for _ in 0..KNOB_ATTEMPTS {
            match self.exec(typ, payload, expect, INFO_TIMEOUT) {
                Ok(v) => return Ok(v),
                Err(e) => last = Some(e),
            }
        }
        Err(last.unwrap_or_else(|| io_err(format!("lora cmd {typ:#04x}: no attempt made"))))
    }

    /// Send one command and wait for the firmware's reply, with no other command in flight.
    ///
    /// Blocking by construction — see [`CmdPort`]. The wait is bounded by the device's own physics
    /// (a frame's airtime), and the reader runs on its own thread, so reception keeps up meanwhile.
    fn exec(
        &self,
        typ: u8,
        payload: &[u8],
        expect: u8,
        timeout: Duration,
    ) -> Result<Vec<u8>, FaceError> {
        let cmd = self.cmd.lock().unwrap();
        // Drop replies to earlier commands (e.g. one that timed out) so we read *this* one's.
        while cmd.resp.try_recv().is_ok() {}
        send_cmd(&cmd.port, typ, payload)
            .map_err(|e| io_err(format!("lora cmd {typ:#04x}: {e}")))?;
        let deadline = std::time::Instant::now() + timeout;
        loop {
            let left = deadline.saturating_duration_since(std::time::Instant::now());
            if left.is_zero() {
                return Err(io_err(format!("lora cmd {typ:#04x}: no reply in {timeout:?}")));
            }
            match cmd.resp.recv_timeout(left) {
                Ok((t, p)) if t == expect => return Ok(p),
                Ok(_) => continue, // some other event slipped in; keep waiting for ours
                Err(std::sync::mpsc::RecvTimeoutError::Timeout) => {
                    return Err(io_err(format!("lora cmd {typ:#04x}: no reply in {timeout:?}")));
                }
                Err(std::sync::mpsc::RecvTimeoutError::Disconnected) => {
                    return Err(FaceError::Closed);
                }
            }
        }
    }

    /// Push the current SF/BW/CR triple to the firmware (any of them changing needs the full set).
    fn send_mod(&self) -> Result<(), FaceError> {
        let (sf, bw, cr) = {
            let p = self.params.lock().unwrap();
            (p.sf, bw_to_fw(p.bw), p.cr)
        };
        self.exec_idempotent(CMD_SET_MOD, &[sf, bw, cr], EVT_INFO)?;
        Ok(())
    }
}

/// Frame one command/event as `7E A5 | type | len | payload | xor-crc` and write it in one call.
fn send_cmd(port: &SerialFd, typ: u8, payload: &[u8]) -> std::io::Result<()> {
    let mut f = Vec::with_capacity(5 + payload.len());
    f.push(SYNC0);
    f.push(SYNC1);
    f.push(typ);
    f.push(payload.len() as u8);
    let mut crc = typ ^ (payload.len() as u8);
    for &b in payload {
        f.push(b);
        crc ^= b;
    }
    f.push(crc);
    port.write_all(&f)?;
    port.flush()
}

/// Program frequency, modulation, power and sync word at open. The firmware applies each live; the
/// port open does not reset the MCU (DTR is not wired to nRST), so a short settle + flush suffices.
fn configure(port: &SerialFd, p: &LoraParams) -> Result<(), FaceError> {
    port.flush_input();
    std::thread::sleep(Duration::from_millis(200));
    let mkerr = |e: std::io::Error| io_err(format!("lora configure: {e}"));
    // One command at a time, with room for the firmware to service it. The reader thread does not
    // exist yet, so there is no reply to wait on here — but the same hazard applies as at runtime
    // (see `CmdPort`): each handler runs to completion before the FIFO-less USART is drained again,
    // and `SET_FREQ` runs a full image calibration. Blasting these back-to-back overruns the port
    // and silently corrupts whatever follows, which is how the radio ends up half-configured.
    let settle = || std::thread::sleep(Duration::from_millis(150));
    // Beacon state first: a host silencing it must not be beaten by a stray beacon that would fire
    // during the slower radio reconfiguration below.
    send_cmd(port, CMD_SET_BEACON, &[p.beacon as u8]).map_err(mkerr)?;
    settle();
    send_cmd(port, CMD_SET_FREQ, &channel_to_hz(p.tx_ch).to_be_bytes()).map_err(mkerr)?;
    settle();
    send_cmd(port, CMD_SET_MOD, &[p.sf, bw_to_fw(p.bw), p.cr]).map_err(mkerr)?;
    settle();
    send_cmd(port, CMD_SET_PWR, &[p.pwr]).map_err(mkerr)?;
    settle();
    send_cmd(port, CMD_SET_SYNC, &[p.sync]).map_err(mkerr)?;
    settle();
    port.flush_input();
    Ok(())
}

/// Background reader: accumulate serial bytes, parse protocol frames, and hand each received LoRa
/// frame up as a `CapturedFrame` stamped `HostRecv` with its RSSI. Non-RX events are logged under
/// `LORA_DEBUG` and otherwise ignored; a crc miss resyncs on the next sync word.
fn reader_loop(
    port: SerialFd,
    tx: mpsc::UnboundedSender<CapturedFrame>,
    resp: std::sync::mpsc::Sender<(u8, Vec<u8>)>,
) {
    let debug = std::env::var_os("LORA_DEBUG").is_some();
    let mut acc: Vec<u8> = Vec::new();
    let mut tmp = [0u8; 512];
    loop {
        match port.read(&mut tmp) {
            Ok(n) if n > 0 => {
                acc.extend_from_slice(&tmp[..n]);
                loop {
                    match next_event(&acc) {
                        EvParse::Event { typ, payload, consumed } => {
                            handle_event(typ, &payload, &tx, &resp, debug);
                            acc.drain(..consumed);
                            if tx.is_closed() {
                                return;
                            }
                        }
                        EvParse::Drop { consumed } => {
                            acc.drain(..consumed);
                        }
                        EvParse::Need => break,
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

/// Dispatch one decoded event: an `EVT_RX` becomes a `CapturedFrame`; others are optional debug.
fn handle_event(
    typ: u8,
    payload: &[u8],
    tx: &mpsc::UnboundedSender<CapturedFrame>,
    resp: &std::sync::mpsc::Sender<(u8, Vec<u8>)>,
    debug: bool,
) {
    // Everything that is not a received frame is a reply to a command we sent; route it to the
    // caller blocked in `exec` so the next command only goes out once this one is serviced.
    if typ != EVT_RX {
        if debug {
            match typ {
                EVT_TXDONE => {
                    eprintln!("lora TXDONE ok={}", payload.first().copied().unwrap_or(0))
                }
                EVT_INFO => eprintln!("lora INFO {}", hex(payload)),
                EVT_LOG => eprintln!("lora LOG: {}", String::from_utf8_lossy(payload)),
                other => eprintln!("lora EVT {other:#04x} {}", hex(payload)),
            }
        }
        // A LOG line is unsolicited chatter, not a reply — never let it satisfy a wait.
        if typ != EVT_LOG {
            let _ = resp.send((typ, payload.to_vec()));
        }
        return;
    }
    match typ {
        // #52: EVT_RX gained a 4-byte device (MCU-ms) arrival timestamp between SNR and the payload:
        // [rssi(2), snr(2), ts_ms(4 BE), bytes]. Skip it (the device clock is surfaced separately when
        // needed for common-view timing); the payload now starts at offset 8.
        EVT_RX if payload.len() >= 8 => {
            let rssi = i16::from_be_bytes([payload[0], payload[1]]);
            let snr = i16::from_be_bytes([payload[2], payload[3]]);
            let ts_ms = u32::from_be_bytes([payload[4], payload[5], payload[6], payload[7]]);
            let ndn = &payload[8..];
            if debug {
                eprintln!("lora RX [{rssi} dBm, SNR {snr}, ts {ts_ms}ms] {} bytes", ndn.len());
            }
            let cap = CapturedFrame {
                payload: ndn.to_vec().into(),
                addr: None,
                group: None,
                rssi_dbm: Some(rssi.clamp(i8::MIN as i16, i8::MAX as i16) as i8),
                mcs_index: None,
                stamp: Some(host_stamp()),
            };
            let _ = tx.send(cap);
        }
        _ => {}
    }
}

fn hex(b: &[u8]) -> String {
    b.iter().map(|x| format!("{x:02x}")).collect()
}

/// Outcome of trying to parse one event from the front of the accumulator.
enum EvParse {
    Event { typ: u8, payload: Vec<u8>, consumed: usize },
    Drop { consumed: usize },
    Need,
}

/// Parse one `7E A5 | type | len | payload | crc` frame from the front of `buf`.
fn next_event(buf: &[u8]) -> EvParse {
    let Some(start) = buf.windows(2).position(|w| w == [SYNC0, SYNC1]) else {
        // No sync word; keep only a possible trailing lone SYNC0 for the next read.
        let keep = if buf.last() == Some(&SYNC0) { 1 } else { 0 };
        return if buf.len() > keep {
            EvParse::Drop { consumed: buf.len() - keep }
        } else {
            EvParse::Need
        };
    };
    if start > 0 {
        return EvParse::Drop { consumed: start }; // discard leading garbage
    }
    if buf.len() < 4 {
        return EvParse::Need; // sync(2) + type + len
    }
    let typ = buf[2];
    let len = buf[3] as usize;
    let total = 4 + len + 1; // + crc
    if buf.len() < total {
        return EvParse::Need;
    }
    let payload = &buf[4..4 + len];
    let mut crc = typ ^ (len as u8);
    for &b in payload {
        crc ^= b;
    }
    if crc != buf[4 + len] {
        return EvParse::Drop { consumed: 2 }; // crc miss — skip past this sync word and resync
    }
    EvParse::Event { typ, payload: payload.to_vec(), consumed: total }
}

#[async_trait]
impl FrameIo for LoraSerialBackend {
    async fn inject(&self, frame_in: InjectFrame) -> Result<(), FaceError> {
        // The payload is the NDN packet itself; the firmware LoRa-frames it. `dst`/`src`/`tx` carry
        // no link addressing on standard LoRa here, so they are advisory only.
        if frame_in.payload.len() > MAX_LORA_PAYLOAD {
            return Err(io_err(format!(
                "lora payload {} > {MAX_LORA_PAYLOAD} (one frame)",
                frame_in.payload.len()
            )));
        }
        let ok = self.exec(CMD_TX, &frame_in.payload, EVT_TXDONE, TXDONE_TIMEOUT)?;
        match ok.first() {
            Some(1) => Ok(()),
            _ => Err(io_err("lora TX reported failure".into())),
        }
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
        RadioCapability::lora(vec![self.params.lock().unwrap().tx_ch])
    }
}

/// Control plane: LoRa's dials, actuated at runtime as binary commands the firmware applies live
/// (no command-mode switch, no reader parking). `set_spreading_factor` is the reach/rate knob
/// cognition drives (down for close/bulk, up for far/urgent); the rest tune channel, coding rate,
/// bandwidth, and power.
impl RadioKnobs for LoraSerialBackend {
    fn set_channel(&self, channel: u8, _bw: Bandwidth) -> Result<(), FaceError> {
        // LoRa is half-duplex on a single carrier: point TX and RX at it together.
        self.exec_idempotent(CMD_SET_FREQ, &channel_to_hz(channel).to_be_bytes(), EVT_INFO)?;
        let mut p = self.params.lock().unwrap();
        p.tx_ch = channel;
        p.rx_ch = channel;
        Ok(())
    }

    fn set_tx_power(&self, idx: u32) -> Result<(), FaceError> {
        let dbm = idx.clamp(10, 22) as u8;
        self.exec_idempotent(CMD_SET_PWR, &[dbm], EVT_INFO)?;
        self.params.lock().unwrap().pwr = dbm;
        Ok(())
    }

    fn set_edcca_ignore(&self, _on: bool) -> Result<(), FaceError> {
        // The open firmware transmits without listen-before-talk (standard LoRa TX), so EDCCA is
        // effectively always ignored; nothing to actuate.
        Ok(())
    }

    fn set_spreading_factor(&self, sf: u8) -> Result<(), FaceError> {
        let sf = sf.clamp(7, 12);
        self.params.lock().unwrap().sf = sf;
        self.send_mod()
    }

    fn set_coding_rate(&self, cr: u8) -> Result<(), FaceError> {
        let cr = cr.clamp(1, 4);
        self.params.lock().unwrap().cr = cr;
        self.send_mod()
    }

    fn set_bandwidth_khz(&self, khz: u32) -> Result<(), FaceError> {
        let bw = match khz {
            0..=180 => 0,   // 125 kHz
            181..=360 => 1, // 250 kHz
            _ => 2,         // 500 kHz
        };
        self.params.lock().unwrap().bw = bw;
        self.send_mod()
    }

    fn tx_discipline(&self) -> TxDiscipline {
        // The serial bridge + duty-cycle limit make the on-air instant loose; honest floor.
        TxDiscipline::BestEffort
    }
}

fn io_err(msg: String) -> FaceError {
    FaceError::Io(std::io::Error::other(msg))
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Build a wire frame the way the firmware would emit an event.
    fn wire(typ: u8, payload: &[u8]) -> Vec<u8> {
        let mut v = vec![SYNC0, SYNC1, typ, payload.len() as u8];
        v.extend_from_slice(payload);
        let mut crc = typ ^ (payload.len() as u8);
        for &b in payload {
            crc ^= b;
        }
        v.push(crc);
        v
    }

    #[test]
    fn parses_a_clean_rx_event() {
        // rssi = -31, snr = 9, payload "hi"
        let mut p = Vec::new();
        p.extend_from_slice(&(-31i16).to_be_bytes());
        p.extend_from_slice(&9i16.to_be_bytes());
        p.extend_from_slice(b"hi");
        let buf = wire(EVT_RX, &p);
        match next_event(&buf) {
            EvParse::Event { typ, payload, consumed } => {
                assert_eq!(typ, EVT_RX);
                assert_eq!(consumed, buf.len());
                assert_eq!(i16::from_be_bytes([payload[0], payload[1]]), -31);
                assert_eq!(&payload[4..], b"hi");
            }
            _ => panic!("expected an event"),
        }
    }

    #[test]
    fn skips_leading_garbage_then_parses() {
        let mut buf = vec![0x00, 0x11, 0x22];
        buf.extend_from_slice(&wire(EVT_TXDONE, &[1]));
        match next_event(&buf) {
            EvParse::Drop { consumed } => buf.drain(..consumed),
            _ => panic!("expected drop of garbage"),
        };
        match next_event(&buf) {
            EvParse::Event { typ, payload, .. } => {
                assert_eq!(typ, EVT_TXDONE);
                assert_eq!(payload, vec![1]);
            }
            _ => panic!("expected event after garbage"),
        }
    }

    #[test]
    fn partial_frame_needs_more() {
        let full = wire(EVT_RX, &[0, 0, 0, 0, b'x']);
        assert!(matches!(next_event(&full[..6]), EvParse::Need));
    }

    #[test]
    fn crc_miss_is_dropped_and_resyncs() {
        let mut buf = wire(EVT_RX, &[0, 0, 0, 0, b'a']);
        let last = buf.len() - 1;
        buf[last] ^= 0xff; // wreck the crc
        let good = wire(EVT_TXDONE, &[1]);
        buf.extend_from_slice(&good);
        match next_event(&buf) {
            EvParse::Drop { consumed } => {
                assert_eq!(consumed, 2);
                buf.drain(..consumed);
            }
            _ => panic!("expected drop of corrupt frame"),
        }
        loop {
            match next_event(&buf) {
                EvParse::Event { typ, .. } => {
                    assert_eq!(typ, EVT_TXDONE);
                    break;
                }
                EvParse::Drop { consumed } => {
                    buf.drain(..consumed);
                }
                EvParse::Need => panic!("lost the good frame"),
            }
        }
    }

    #[test]
    fn command_frame_round_trips_through_parser() {
        // A command we send should parse back with the same type/payload (same framing both ways).
        let payload = 915_000_000u32.to_be_bytes();
        let buf = wire(CMD_SET_FREQ, &payload);
        match next_event(&buf) {
            EvParse::Event { typ, payload: got, .. } => {
                assert_eq!(typ, CMD_SET_FREQ);
                assert_eq!(u32::from_be_bytes([got[0], got[1], got[2], got[3]]), 915_000_000);
            }
            _ => panic!("expected event"),
        }
    }

    #[test]
    fn channel_maps_to_us_and_eu() {
        assert_eq!(channel_to_hz(65), 915_000_000);
        assert_eq!(channel_to_hz(18), 868_000_000);
    }
}
