//! Serial-bridged 802.11 monitor-mode backend (the `[4E 44 …]` "ND" wire protocol).
//!
//! A microcontroller running our bridge firmware is a raw 802.11 injector/capturer driven over a serial
//! link. Two chipsets speak this exact protocol behind [`SerialRadioBackend`]: the **BW16 (RTL8720DN)**
//! (`firmware/bw16-ndn-bridge`) and the **ESP32-C5** (`firmware/esp32c5-ndn[-rs]`, via its native
//! USB-Serial-JTAG) — hence the chipset-neutral name. The host builds the *same* 802.11 frame
//! (`ndn_frame_io::frame::build_dot11`) the USB drivers build, ships the bytes to the board to inject raw,
//! and parses captured frames back — so a `MonitorWifiFace` over either board is just another [`FrameIo`]
//! backend, and the NAN engine / cognition above it are none the wiser. The proof that the HAL seam
//! accommodates a radically different radio (an MCU on a serial tether, not a USB host driver).
//!
//! [`Esp32SerialBackend`] is a thin newtype over this that supplies C5-specific surface (dual-band
//! capability, hardware scheduled TX, the free-run RX clock, the `wifi_phy_rate_t` rate mapping).

use std::io::{Read, Write};
use std::sync::{Arc, Mutex};
use std::time::Duration;

use async_trait::async_trait;
use bytes::Bytes;
use ndn_frame_io::{
    CapturedFrame, ClockDomainId, FrameFormat, FrameIo, InjectFrame, LatchPoint, LinkStamp,
    McsDescriptor, PhyMetrics, RadioCapability, RadioProfile, RadioTime, RadioTimeSource, frame,
};

/// Host monotonic clock domain — shared by every host-stamped frame in this process (the serial
/// board has no hardware TSF, so its stamps are host-side). Distinct from per-device TSF domains.
const HOST_CLOCK_DOMAIN: ClockDomainId = ClockDomainId(0x484F_5354); // "HOST"

/// A HostRecv [`LinkStamp`]: nanoseconds since process start (monotonic), latched when the serial
/// line delivered the frame — the coarsest but honest time a serial board can offer.
fn host_stamp() -> LinkStamp {
    static START: std::sync::OnceLock<std::time::Instant> = std::sync::OnceLock::new();
    let start = START.get_or_init(std::time::Instant::now);
    LinkStamp::new(
        start.elapsed().as_nanos() as u64,
        HOST_CLOCK_DOMAIN,
        LatchPoint::HostRecv.precision_floor_ns(),
        LatchPoint::HostRecv,
    )
}
use ndn_radio_hal::{Bandwidth, OpenRadio, RadioKnobs, TxDiscipline};

/// A device stamp from the ESP32-C5's free-running per-frame RX clock (`rx_ctrl.timestamp`): a µs
/// counter, so `raw` is the µs value and the tick is 1000 ns (see [`RadioTimeSource::free_run_rx_stamp`]).
/// Unlike [`host_stamp`] this is latched on the device at RX (no serial jitter) and is the same domain as
/// the clock the C5 schedules TX on — the basis for common-view and frame-age.
fn dev_rx_stamp(ts_us: u32, domain: ClockDomainId) -> LinkStamp {
    LinkStamp::new(
        ts_us as u64,
        domain,
        LatchPoint::MacDone.precision_floor_ns(),
        LatchPoint::MacDone,
    )
}

/// A per-device clock domain for a C5 on `path` (each device is its own physical counter). FNV-1a over
/// the port path, tagged into the "C5" space so it never collides with the host-recv domain.
fn c5_clock_domain(path: &str) -> ClockDomainId {
    let mut h: u32 = 0x811c_9dc5;
    for b in path.bytes() {
        h ^= b as u32;
        h = h.wrapping_mul(0x0100_0193);
    }
    ClockDomainId((h & 0x00ff_ffff) | 0x4335_0000) // "C5" tag in the top bytes
}
use ndn_transport::FaceError;
use tokio::sync::{Mutex as AsyncMutex, mpsc};

const SYNC0: u8 = 0x4E;
const SYNC1: u8 = 0x44;
const T_INJECT: u8 = 0x01;
const T_CHANNEL: u8 = 0x02;
const T_TXPOWER: u8 = 0x03; // wext "txpower patha=N" power index
const T_RATE: u8 = 0x04; // wifi_set_tx_data_rate code
const T_BW40: u8 = 0x05; // wext_set_bw40_enable
const T_INJECT_ATTR: u8 = 0x06; // poke pkt_attrib bytes, then inject
const T_NAMEFILTER: u8 = 0x07; // load on-device Tier-0 masks: [enabled][n_masks][mask 16B]*
const T_INJECT_AT: u8 = 0x09; // scheduled TX: [delay_us_le32][802.11 frame] (ESP32-C5 firmware only)
const T_INJECT_ABS: u8 = 0x0A; // scheduled TX at an ABSOLUTE esp_timer µs: [target_us_le64][frame]
const T_READCLOCK: u8 = 0x0B; // request the device's schedule clock; reply T_CLOCK [esp_timer_us_le64]
const T_CLOCK: u8 = 0x85; // reply to T_READCLOCK: [esp_timer_us_le64]
const T_TXTIME: u8 = 0x83; // scheduled-TX confirmation: [target_le64][actual_le64][tsf_le64]
const T_RX: u8 = 0x81;
const T_OCC: u8 = 0x86; // [activity_count_le32] — periodic free-running channel-activity counter
const T_RX_TS: u8 = 0x82; // [rssi_i8][noise_i8][rate_code][phy_flags][rx_ts_us_le32][frame] — RX + hardware
// BLE bearer (the UNIFIED esp32c5-ndn firmware serves Wi-Fi + BLE from one image over this one port, so the
// BLE bearer gets its own message types — this backend then demuxes both from the single reader, making it
// the SHARED MUX one host connection uses for both FrameIo (Wi-Fi) and the BLE AdvBackend).
const T_BLE_ADV: u8 = 0x30; // host->device: advertise this payload (BLE 5 ext-adv)
const T_COEX: u8 = 0x31; // host->device: [scan_window_le16][scan_itvl_le16] — the BLE<->Wi-Fi radio-time split
const T_BLE_RX: u8 = 0x88; // device->host: [rssi_i8][addr6][payload] — a scanned advertisement
// µs stamp + the ESP's per-frame PHY metadata (radiotap-equiv): RX rate/MCS + SNR

/// BW16 fixed TX-rate codes for [`SerialRadioBackend::set_tx_rate`]
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
pub const SERIAL_RADIO_BAUD: u32 = 115_200;

/// A BW16 reached over its USB-serial port.
pub struct SerialRadioBackend {
    tx: Arc<Mutex<Box<dyn serialport::SerialPort>>>,
    format: FrameFormat,
    rx: AsyncMutex<mpsc::UnboundedReceiver<CapturedFrame>>,
    /// Device schedule-clock replies (`T_CLOCK` → esp_timer µs), routed here by the reader for
    /// [`read_schedule_clock`](Self::read_schedule_clock). Empty on the BW16 (never replies).
    clock_rx: AsyncMutex<mpsc::UnboundedReceiver<u64>>,
    /// Scheduled-TX confirmations (`T_TXTIME` → (target, actual) esp_timer µs) — the actual on-air
    /// instant of an [`inject_at_abs`](Self::inject_at_abs), for verifying slot placement.
    txtime_rx: AsyncMutex<mpsc::UnboundedReceiver<(u64, u64)>>,
    /// Latest free-running channel-activity counter (`T_OCC`, ~5×/s from the C5), for
    /// [`RadioKnobs::read_channel_activity`]. `u32::MAX` = no report yet (e.g. the BW16, which never emits).
    activity: Arc<std::sync::atomic::AtomicU32>,
    /// Scanned BLE advertisements (`T_BLE_RX` → (rssi, addr6, payload)) from the unified firmware's BLE
    /// bearer, routed here by the one reader so the SAME port connection also drives a BLE `AdvBackend`
    /// (see [`ble_next_scanned`](Self::ble_next_scanned)). Empty on the BW16 (Wi-Fi only, no BLE).
    ble_rx: AsyncMutex<mpsc::UnboundedReceiver<(i8, [u8; 6], Bytes)>>,
    /// Running count of scanned BLE advertisements — the BLE **demand** signal, incremented by the reader
    /// independently of the `ble_rx` channel so [`spawn_demand_coex`](Self::spawn_demand_coex) can measure
    /// BLE traffic without stealing frames from the face (mirrors `wifi_frames` = the Wi-Fi demand signal).
    ble_activity: Arc<std::sync::atomic::AtomicU32>,
    /// Running count of Wi-Fi frames FORWARDED to the face (`T_RX`/`T_RX_TS` that passed the named-radio
    /// filter) — the Wi-Fi **demand** signal, the symmetric counterpart of `ble_activity`. Distinct from
    /// `activity` (`T_OCC`), which is raw channel energy (ambient included) → the interference/channel lever,
    /// not this bearer's named-traffic demand. The coex split balances the two NAMED demands, not occupancy.
    wifi_frames: Arc<std::sync::atomic::AtomicU32>,
}

impl SerialRadioBackend {
    /// Open the BW16 (RTL8720DN) at `path` (e.g. `/dev/tty.usbserial-XXXX`) and spawn the RX
    /// reader that deframes captured 802.11 frames off the serial link. Pulses DTR→CEN to reset
    /// the board into our firmware so a freshly-flashed board comes up without a manual reset.
    pub fn open(path: &str) -> Result<Self, FaceError> {
        Self::open_inner(path, true, None)
    }

    /// Open a native-USB-Serial-JTAG ESP32 (e.g. the ESP32-C5) running the same serial-bridge
    /// firmware, WITHOUT any reset pulse. On these parts RTS maps to EN (chip reset) and DTR to
    /// GPIO9 (the boot strap) — so the BW16's DTR→CEN pulse would instead toggle the boot strap and
    /// asserting RTS would hold the chip in reset. We de-assert both and never touch them again; the
    /// chip free-runs the app it booted on power-up. (This was THE bug: `serialport` asserting RTS on
    /// open held the C5's EN low, so every inject went to a halted chip and nothing reached air.)
    pub fn open_no_reset(path: &str) -> Result<Self, FaceError> {
        Self::open_inner(path, false, None)
    }

    /// Like [`open_no_reset`](Self::open_no_reset) but the reader stamps `T_RX_TS` frames in the given
    /// device clock `domain` (the ESP32-C5's hardware per-frame RX timestamp) rather than host-recv time.
    pub fn open_no_reset_clocked(path: &str, domain: ClockDomainId) -> Result<Self, FaceError> {
        Self::open_inner(path, false, Some(domain))
    }

    fn open_inner(
        path: &str,
        reset_pulse: bool,
        dev_clock: Option<ClockDomainId>,
    ) -> Result<Self, FaceError> {
        let mut port = serialport::new(path, SERIAL_RADIO_BAUD)
            .timeout(Duration::from_millis(50))
            .open()
            .map_err(|e| io_err(format!("bw16 open {path}: {e}")))?;
        // Always leave RTS/DTR de-asserted. On the BW16 this is idle; on a USB-Serial-JTAG ESP32
        // this is critical — asserted RTS = EN low = chip held in reset.
        let _ = port.write_request_to_send(false);
        let _ = port.write_data_terminal_ready(false);
        if reset_pulse {
            // BW16: DTR→CEN pulse to boot our firmware (~900 ms to `ready`).
            std::thread::sleep(Duration::from_millis(150));
            let _ = port.write_data_terminal_ready(true);
            std::thread::sleep(Duration::from_millis(120));
            let _ = port.write_data_terminal_ready(false);
            std::thread::sleep(Duration::from_millis(1000));
        }
        let _ = port.clear(serialport::ClearBuffer::Input);
        let reader = port
            .try_clone()
            .map_err(|e| io_err(format!("bw16 clone: {e}")))?;
        let (txch, rxch) = mpsc::unbounded_channel();
        let (clkch, clk_rxch) = mpsc::unbounded_channel();
        let (ttch, tt_rxch) = mpsc::unbounded_channel();
        let (blech, ble_rxch) = mpsc::unbounded_channel();
        let format = FrameFormat::default();
        let activity = Arc::new(std::sync::atomic::AtomicU32::new(u32::MAX));
        let ble_activity = Arc::new(std::sync::atomic::AtomicU32::new(0));
        let wifi_frames = Arc::new(std::sync::atomic::AtomicU32::new(0));
        let act_reader = activity.clone();
        let ble_act_reader = ble_activity.clone();
        let wifi_fr_reader = wifi_frames.clone();
        std::thread::spawn(move || {
            reader_loop(
                reader,
                format,
                txch,
                clkch,
                ttch,
                blech,
                act_reader,
                ble_act_reader,
                wifi_fr_reader,
                dev_clock,
            )
        });
        Ok(Self {
            tx: Arc::new(Mutex::new(port)),
            format,
            rx: AsyncMutex::new(rxch),
            clock_rx: AsyncMutex::new(clk_rxch),
            txtime_rx: AsyncMutex::new(tt_rxch),
            activity,
            ble_rx: AsyncMutex::new(ble_rxch),
            ble_activity,
            wifi_frames,
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

    /// Pin an explicit `phymode` (1=11B 2=11G 3=11A 4=HT20 6=HE20) + HE reach flags (bit0=DCM, bit1=ER-SU)
    /// alongside the rate `code`. The C5 firmware feeds these to `ic_set_80211_tx_rate_config`. Used for the
    /// HE reach levers (phymode HE20 + DCM/ER-SU); the plain [`set_tx_rate`](Self::set_tx_rate) stays HT.
    pub fn set_tx_rate_ex(&self, code: u8, phymode: u8, he_flags: u8) -> Result<(), FaceError> {
        self.send_framed(T_RATE, &[code, phymode, he_flags])
    }

    /// Enable/disable 40 MHz channel bandwidth (`wext_set_bw40_enable`).
    pub fn set_bw40(&self, enable: bool) -> Result<(), FaceError> {
        self.send_framed(T_BW40, &[enable as u8])
    }

    // --- BLE bearer (the unified esp32c5-ndn firmware serves Wi-Fi + BLE from this one port) ---
    // These make the ONE `SerialRadioBackend`/serial connection the SHARED MUX for both bearers: it is a
    // Wi-Fi `FrameIo` (inject/recv_frame above) *and* the source of a BLE `AdvBackend` — the reader demuxes
    // T_RX_TS (Wi-Fi) and T_BLE_RX (BLE) off the same stream, so a node opens the port once and gets both.

    /// Broadcast `payload` as a BLE 5 extended advertisement (the unified C5 firmware wraps it in the ND
    /// manufacturer AD and burst-advertises it, fire-and-forget). The BLE analog of [`inject`](FrameIo::inject).
    pub fn ble_broadcast(&self, payload: &[u8]) -> Result<(), FaceError> {
        self.send_framed(T_BLE_ADV, payload)
    }

    /// Await the next scanned BLE advertisement — `(rssi_dbm, addr6, payload)`. The BLE analog of
    /// [`recv_frame`](FrameIo::recv_frame); returns `Err(Closed)` if the reader thread has exited.
    pub async fn ble_next_scanned(&self) -> Result<(i8, [u8; 6], Bytes), FaceError> {
        let mut rx = self.ble_rx.lock().await;
        rx.recv().await.ok_or(FaceError::Closed)
    }

    /// Set this radio's **BLE share** of airtime: `fraction` (0.0–1.0) of the scan interval spent scanning
    /// for BLE, the rest left to the concurrent promiscuous Wi-Fi RX (both bearers share one radio via coex).
    /// The NDR way to split the two — **not** a firmware constant but a lever cognition drives from measured
    /// per-bearer demand (see [`auto_coex_share`](Self::auto_coex_share)). `itvl` is the scan interval in
    /// 0.625 ms units (256 ≈ 160 ms); window = `fraction·itvl`, clamped to [4, itvl].
    pub fn set_ble_share(&self, fraction: f32, itvl: u16) -> Result<(), FaceError> {
        let window = ((fraction.clamp(0.0, 1.0) * itvl as f32) as u16).clamp(4, itvl.max(4));
        let mut p = window.to_le_bytes().to_vec();
        p.extend_from_slice(&itvl.to_le_bytes());
        self.send_framed(T_COEX, &p)
    }

    /// Running count of scanned BLE advertisements since open — the BLE demand signal (take deltas).
    pub fn ble_scan_count(&self) -> u32 {
        self.ble_activity.load(std::sync::atomic::Ordering::Relaxed)
    }

    /// Running count of Wi-Fi frames forwarded to the face (named-radio traffic) — the Wi-Fi demand signal,
    /// the symmetric counterpart of [`ble_scan_count`](Self::ble_scan_count) (take deltas). This is *named*
    /// traffic, not raw channel energy — see [`wifi_activity_count`](Self::wifi_activity_count) for the latter.
    pub fn wifi_frame_count(&self) -> u32 {
        self.wifi_frames.load(std::sync::atomic::Ordering::Relaxed)
    }

    /// Latest Wi-Fi channel-**occupancy** counter (`T_OCC`), or `0` if the device hasn't reported yet — raw
    /// energy on the channel (ambient included). This drives the interference/channel lever, NOT the coex
    /// split (that balances the two bearers' *named* demand — see [`wifi_frame_count`](Self::wifi_frame_count)).
    pub fn wifi_activity_count(&self) -> u32 {
        match self.activity.load(std::sync::atomic::Ordering::Relaxed) {
            u32::MAX => 0,
            v => v,
        }
    }

    /// Spawn the **demand-driven coex** loop: it drives [`set_ble_share`](Self::set_ble_share) from measured
    /// per-bearer demand instead of a constant — the NDR airtime split as a CLOSED LOOP. Every `period` it
    /// samples both bearers' *named*-traffic counters (Wi-Fi frames forwarded, BLE ads scanned), takes their
    /// deltas, and sets BLE's share of the scan interval to BLE's share of total named traffic, `[floor,ceil]`
    /// so neither bearer is fully starved. The `floor` also keeps enough scan airtime to notice a BLE burst
    /// (else a low share is self-reinforcing — the one caveat; a real system would periodically probe wider).
    /// Returns the task handle — abort it to stop. `itvl` = scan interval in 0.625 ms units.
    pub fn spawn_demand_coex(
        self: Arc<Self>,
        period: Duration,
        floor: f32,
        ceil: f32,
        itvl: u16,
    ) -> tokio::task::JoinHandle<()> {
        tokio::spawn(async move {
            let mut prev_w = self.wifi_frame_count();
            let mut prev_b = self.ble_scan_count();
            loop {
                tokio::time::sleep(period).await;
                let w = self.wifi_frame_count();
                let b = self.ble_scan_count();
                let dw = w.wrapping_sub(prev_w) as f32;
                let db = b.wrapping_sub(prev_b) as f32;
                prev_w = w;
                prev_b = b;
                let total = dw + db;
                // Idle on both bearers → hold the midpoint so each stays reachable; else split by demand.
                let share = if total < 1.0 {
                    (floor + ceil) * 0.5
                } else {
                    (db / total).clamp(floor, ceil)
                };
                let _ = self.set_ble_share(share, itvl);
            }
        })
    }

    /// Set the TX power index (wext `txpower patha=<idx>`). Only reachable on the
    /// Rust firmware, which reimplements the SDK's `#if 0`-disabled wifi_set_txpower.
    pub fn set_txpower(&self, idx: u8) -> Result<(), FaceError> {
        self.send_framed(T_TXPOWER, &[idx])
    }

    /// Load the on-device **Tier-0 name filter** (ESP32-C5 firmware only): up to 8 16-byte prefix-set
    /// masks (cognition derives them via the shared `tier0` code). When `enabled` and at least one mask
    /// is present, a received 0x8624 frame whose in-address prefix-set matches no mask is dropped ON THE
    /// DEVICE — it never crosses the serial link, the §8.2 pre-USB drop. `enabled=false` (or no masks)
    /// forwards everything (stock behaviour). Masks beyond the 8th are ignored (the firmware cap).
    pub fn configure_name_filter(
        &self,
        enabled: bool,
        masks: &[[u8; 16]],
    ) -> Result<(), FaceError> {
        let n = masks.len().min(8);
        let mut payload = Vec::with_capacity(2 + n * 16);
        payload.push(enabled as u8);
        payload.push(n as u8);
        for m in &masks[..n] {
            payload.extend_from_slice(m);
        }
        self.send_framed(T_NAMEFILTER, &payload)
    }

    /// **Scheduled TX** (ESP32-C5 firmware only): place a frame on air at `delay_us` from now, timed by
    /// the device's always-running monotonic clock to within a few hundred µs — the airtime-lease
    /// primitive (`TxDiscipline::ScheduledAt`). Ordinary [`inject`](Self::inject) leaves at request time
    /// with host+OS+serial jitter (ms); this places TX at a precise instant. Delay is capped in firmware
    /// (~20 ms; this is a busy-wait primitive, not yet a periodic slot lease). Measured error ≤ ~190 µs.
    pub fn inject_at(&self, frame_in: InjectFrame, delay_us: u32) -> Result<(), FaceError> {
        let dot11 = frame::build_dot11(self.format, &frame_in)?;
        let mut payload = Vec::with_capacity(4 + dot11.len());
        payload.extend_from_slice(&delay_us.to_le_bytes());
        payload.extend_from_slice(&dot11);
        self.send_framed(T_INJECT_AT, &payload)
    }

    /// **Scheduled TX at an ABSOLUTE device-clock instant** (ESP32-C5, `T_INJECT_ABS`): place the frame
    /// on air when the device's esp_timer reaches `target_us` — the slot-lease primitive. `target_us` is
    /// in the device's schedule clock (the same µs domain the C5 reports for scheduling); the firmware
    /// drops a stale/past or too-far-future target rather than firing late. This backs the HAL's
    /// [`FrameIo::inject_at_clock`] so a scheduler can place a frame in its slot with no host jitter.
    pub fn inject_at_abs(&self, frame_in: InjectFrame, target_us: u64) -> Result<(), FaceError> {
        let dot11 = frame::build_dot11(self.format, &frame_in)?;
        let mut payload = Vec::with_capacity(8 + dot11.len());
        payload.extend_from_slice(&target_us.to_le_bytes());
        payload.extend_from_slice(&dot11);
        self.send_framed(T_INJECT_ABS, &payload)
    }

    /// **Read the device's schedule clock** (ESP32-C5 esp_timer µs) — the same domain [`inject_at_abs`]
    /// targets, so a caller reads it, computes a slot instant, and schedules against it. Sends
    /// `T_READCLOCK` and awaits the `T_CLOCK` reply. `None` on the BW16 (it has no such clock) or timeout.
    pub async fn read_schedule_clock(&self) -> Option<u64> {
        let mut rx = self.clock_rx.lock().await;
        while rx.try_recv().is_ok() {} // drop any stale reply before requesting a fresh one
        self.send_framed(T_READCLOCK, &[]).ok()?;
        tokio::time::timeout(Duration::from_millis(300), rx.recv())
            .await
            .ok()
            .flatten()
    }

    /// Await the next scheduled-TX confirmation `(target, actual)` esp_timer µs — the actual on-air
    /// instant of an [`inject_at_abs`](Self::inject_at_abs), for verifying slot placement. `None` on timeout.
    pub async fn recv_tx_confirm(&self) -> Option<(u64, u64)> {
        let mut rx = self.txtime_rx.lock().await;
        tokio::time::timeout(Duration::from_millis(500), rx.recv())
            .await
            .ok()
            .flatten()
    }

    /// Inject a complete 802.11 frame after poking `(offset, value)` bytes into the
    /// driver's `pkt_attrib` (Rust firmware, `T_INJECT_ATTR`). The RE harness for
    /// the TX-descriptor rate field: sweep the offset, set MGN_MCSx, watch the MCS.
    pub fn inject_attr(&self, frame: &[u8], pairs: &[(u8, u8)]) -> Result<(), FaceError> {
        let mut payload = Vec::with_capacity(1 + 2 * pairs.len() + frame.len());
        payload.push(pairs.len() as u8);
        for (o, v) in pairs {
            payload.push(*o);
            payload.push(*v);
        }
        payload.extend_from_slice(frame);
        self.send_framed(T_INJECT_ATTR, &payload)
    }
}

/// Background reader: accumulate serial bytes, deframe RX packets, parse each as
/// an 802.11 frame in `format`, and hand the `CapturedFrame`s to `recv_frame`.
fn reader_loop(
    mut port: Box<dyn serialport::SerialPort>,
    format: FrameFormat,
    tx: mpsc::UnboundedSender<CapturedFrame>,
    clk: mpsc::UnboundedSender<u64>,
    txtime: mpsc::UnboundedSender<(u64, u64)>,
    ble: mpsc::UnboundedSender<(i8, [u8; 6], Bytes)>,
    activity: Arc<std::sync::atomic::AtomicU32>,
    ble_activity: Arc<std::sync::atomic::AtomicU32>,
    wifi_frames: Arc<std::sync::atomic::AtomicU32>,
    dev_clock: Option<ClockDomainId>,
) {
    let mut acc: Vec<u8> = Vec::new();
    let mut tmp = [0u8; 2048];
    loop {
        match port.read(&mut tmp) {
            Ok(n) if n > 0 => {
                acc.extend_from_slice(&tmp[..n]);
                while let Some((ty, payload, consumed)) = deframe(&acc) {
                    // T_CLOCK [esp_timer_us_le64] — the device's schedule clock reply; route to read_schedule_clock.
                    if ty == T_CLOCK && payload.len() >= 8 {
                        let t = u64::from_le_bytes(payload[0..8].try_into().unwrap());
                        let _ = clk.send(t);
                        acc.drain(..consumed);
                        continue;
                    }
                    if ty == T_TXTIME && payload.len() >= 16 {
                        let target = u64::from_le_bytes(payload[0..8].try_into().unwrap());
                        let actual = u64::from_le_bytes(payload[8..16].try_into().unwrap());
                        let _ = txtime.send((target, actual));
                        acc.drain(..consumed);
                        continue;
                    }
                    if ty == T_OCC && payload.len() >= 4 {
                        let c =
                            u32::from_le_bytes([payload[0], payload[1], payload[2], payload[3]]);
                        activity.store(c, std::sync::atomic::Ordering::Relaxed);
                        acc.drain(..consumed);
                        continue;
                    }
                    // T_BLE_RX [rssi_i8][addr6][payload] — a scanned BLE advertisement from the unified
                    // firmware's BLE bearer; route to ble_next_scanned (drives the BLE AdvBackend on this
                    // same port). The dedup lives on-device (scan filter_duplicates), so pass it straight up.
                    if ty == T_BLE_RX && payload.len() >= 7 {
                        let rssi = payload[0] as i8;
                        let mut addr = [0u8; 6];
                        addr.copy_from_slice(&payload[1..7]);
                        ble_activity.fetch_add(1, std::sync::atomic::Ordering::Relaxed);
                        let _ = ble.send((rssi, addr, Bytes::copy_from_slice(&payload[7..])));
                        acc.drain(..consumed);
                        continue;
                    }
                    // T_RX [rssi][frame] (BW16, no hardware timestamp → HostRecv stamp) and
                    // T_RX_TS [rssi][noise][rate_code][phy_flags][rx_ts_us_le32][frame] (ESP32-C5: hardware
                    // RX stamp + the ESP's per-frame PHY metadata — its "radiotap": RX rate/MCS + SNR).
                    let parsed = if ty == T_RX && !payload.is_empty() {
                        let rssi = payload[0] as i8;
                        frame::parse_dot11(
                            format,
                            &payload[1..],
                            Some(rssi),
                            None,
                            Some(host_stamp()),
                        )
                    } else if ty == T_RX_TS && payload.len() >= 8 {
                        let rssi = payload[0] as i8;
                        let noise = payload[1] as i8;
                        let rate_code = payload[2]; // MCS (bb_format ≥ HT) or the L-SIG rate (legacy)
                        let bb_format = payload[3] & 0x0f; // RX_BB_FORMAT_*: 0=11B 1=11G/A 2=HT 3=VHT 4+=HE
                        let ts_us =
                            u32::from_le_bytes([payload[4], payload[5], payload[6], payload[7]]);
                        // MCS index is meaningful only for HT/VHT/HE; a legacy (11b/g/a) rate is not an MCS.
                        let mcs = (bb_format >= 2).then_some(rate_code);
                        // A device stamp only if this backend was opened with a device clock domain (the
                        // C5); else fall back to HostRecv so an un-clocked open still yields frames.
                        let stamp = dev_clock
                            .map(|d| dev_rx_stamp(ts_us, d))
                            .unwrap_or_else(host_stamp);
                        frame::parse_dot11(format, &payload[8..], Some(rssi), mcs, Some(stamp)).map(
                            |mut c| {
                                // rssi says how loud; SNR (rssi − noise floor) says how clean — the decode predictor.
                                c.phy = Some(PhyMetrics {
                                    snr_db: Some(rssi.saturating_sub(noise)),
                                    evm_db: None,
                                    cfo_hz: None,
                                });
                                c
                            },
                        )
                    } else {
                        None
                    };
                    if let Some(cap) = parsed {
                        wifi_frames.fetch_add(1, std::sync::atomic::Ordering::Relaxed); // Wi-Fi demand signal
                        if tx.send(cap).is_err() {
                            return; // backend dropped
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
impl FrameIo for SerialRadioBackend {
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

// Marker only: the Ameba board's management-TX path picks its own rate, so `set_rate`
// (the FrameIo default no-op) and the derived `inject_at` both just inject.

impl RadioKnobs for SerialRadioBackend {
    fn set_channel(&self, channel: u8, bw: Bandwidth) -> Result<(), FaceError> {
        SerialRadioBackend::set_channel(self, channel)?;
        // Map the HAL bandwidth to the board's 40 MHz enable (the widest this SDK
        // exposes): Bw20 → off, wider → on.
        self.set_bw40(!matches!(bw, Bandwidth::Bw20))
    }
    fn set_tx_power(&self, idx: u32) -> Result<(), FaceError> {
        // Reachable on the Rust firmware (reimplemented txpower). Index is clamped
        // into a byte for the wext command.
        self.set_txpower(idx.min(u8::MAX as u32) as u8)
    }
    fn configure_name_filter(
        &self,
        enabled: bool,
        _key: &[u8; 16],
        masks: &[[u8; 16]],
    ) -> Result<(), FaceError> {
        // The C5/BW16 firmware compares masks against the frame's pre-encoded address octets (the
        // transmitter baked the prefix-set in), so no on-device re-hash → the `key` is unused here.
        SerialRadioBackend::configure_name_filter(self, enabled, masks)
    }
    fn read_channel_activity(&self) -> Result<Option<u16>, FaceError> {
        // The C5 emits its free-running frame counter as T_OCC ~5×/s; the reader caches the latest.
        // u32::MAX = no report yet (the BW16 firmware never emits) → honestly None.
        let v = self.activity.load(std::sync::atomic::Ordering::Relaxed);
        Ok((v != u32::MAX).then_some(v as u16))
    }
}

/// Reference [`RadioTime`] for the `HostRecv` clock kind: the serial board reports no hardware
/// timestamp, so its only honest link clock is the host monotonic clock read when the serial
/// line delivered the frame. It is readable on demand, so `read_clock` returns it.
impl RadioTime for SerialRadioBackend {
    fn time_sources(&self) -> Vec<RadioTimeSource> {
        vec![RadioTimeSource::host_recv(HOST_CLOCK_DOMAIN)]
    }

    fn read_clock(&self, domain: ClockDomainId) -> Result<Option<u64>, FaceError> {
        Ok((domain == HOST_CLOCK_DOMAIN).then(|| host_stamp().raw))
    }
}

impl RadioProfile for SerialRadioBackend {
    fn capability(&self) -> RadioCapability {
        // RTL8720DN (BW16): single-chain 2.4 GHz 11n over the serial bridge.
        RadioCapability::wifi_monitor_2ghz_1ss(vec![1, 6, 11])
    }
}

/// **ESP32-C5 serial-bridge backend** — the C5 speaks the *same* BW16 wire protocol over its native
/// USB-Serial-JTAG, so transport, framing, and knobs are the RTL8720DN [`SerialRadioBackend`] verbatim.
/// It differs in exactly one thing that matters to the planner: it is **dual-band** (2.4 + 5 GHz, both
/// validated on air), whereas the BW16 profile is 2.4-only — so cognition driving a C5 through the BW16
/// identity would never pick a 5 GHz channel. This newtype supplies the dual-band [`RadioCapability`]
/// and delegates everything else. A distinct type (not a config flag) so C5-specific behaviour — 5 GHz
/// knobs, a hardware-TSF clock — has a home as it diverges from the BW16.
pub struct Esp32SerialBackend {
    // Arc so the SAME underlying port/reader (the shared mux) can also back a BLE `AdvBackend` — one host
    // connection, both bearers. See [`shared_mux`](Esp32SerialBackend::shared_mux).
    inner: Arc<SerialRadioBackend>,
    capability: RadioCapability,
    clock_domain: ClockDomainId,
}

impl Esp32SerialBackend {
    /// Open an ESP32-C5 running the `firmware/esp32c5-ndn` (or `-rs`) serial bridge on its native
    /// USB-Serial-JTAG. Uses [`SerialRadioBackend::open_no_reset_clocked`] — RTS/DTR map to EN/GPIO9 on
    /// the C5, so they are never toggled (asserting RTS holds the chip in reset). Dual-band capability
    /// spans 2.4 GHz (1/6/11) and 5 GHz (36/40/44/48). Frames are stamped in the C5's hardware RX clock.
    pub fn open_c5(path: &str) -> Result<Self, FaceError> {
        let clock_domain = c5_clock_domain(path);
        Ok(Self {
            inner: Arc::new(SerialRadioBackend::open_no_reset_clocked(
                path,
                clock_domain,
            )?),
            // .with_he(): the C5 is Wi-Fi 6 — it transmits real HE (verified on air, RX cur_bb_format=HE_SU),
            // so it advertises the HE reach levers (ER-SU + DCM) that for_intent(MostRobust) and set_rate use.
            capability: RadioCapability::wifi_monitor_dual_1ss(vec![1, 6, 11, 36, 40, 44, 48])
                .with_he(),
            clock_domain,
        })
    }

    /// Open the C5 and bundle it as an [`OpenRadio`] — io + knobs + time + profile all backed by the
    /// same instance. This is the capability-carrying path for `MonitorWifiFace::from_open`: the
    /// dual-band [`RadioProfile`] survives into the engine (the scheduler gets the channel knob, the
    /// planner the real bands), whereas `MonitorWifiFace::new(io)` would invent a placeholder cap.
    pub fn open_c5_radio(path: &str) -> Result<OpenRadio, FaceError> {
        let dev = Arc::new(Self::open_c5(path)?);
        // Explicit trait-object bindings: Arc<Self> → Arc<dyn Trait> unsize coercion (one instance,
        // four views). `as` can't spell this — the coercion is implicit, via the annotated `let`.
        let io: Arc<dyn FrameIo> = dev.clone();
        let knobs: Arc<dyn RadioKnobs> = dev.clone();
        let time: Arc<dyn RadioTime> = dev.clone();
        let profile: Arc<dyn RadioProfile> = dev;
        Ok(OpenRadio {
            io,
            knobs: Some(knobs),
            time: Some(time),
            profile: Some(profile),
        })
    }

    /// The shared-mux handle: the `Arc<SerialRadioBackend>` behind this Wi-Fi view, whose BLE methods
    /// (`ble_broadcast`/`ble_next_scanned`/`set_ble_share`/`spawn_demand_coex`) drive the **BLE bearer of
    /// the same port/reader**. Pass this to `ndn-face-ble-adv`'s shared-mux `AdvBackend` so ONE host
    /// connection carries both bearers of the unified C5 firmware (the reader demuxes Wi-Fi and BLE).
    pub fn shared_mux(&self) -> Arc<SerialRadioBackend> {
        self.inner.clone()
    }

    /// Load the on-device Tier-0 name filter — see [`SerialRadioBackend::configure_name_filter`]. On the
    /// C5 this is a real pre-serial drop (the firmware is ours), unlike a commodity monitor NIC.
    pub fn configure_name_filter(
        &self,
        enabled: bool,
        masks: &[[u8; 16]],
    ) -> Result<(), FaceError> {
        self.inner.configure_name_filter(enabled, masks)
    }

    /// Scheduled TX (the airtime-lease primitive) — see [`SerialRadioBackend::inject_at`].
    pub fn inject_at(&self, frame_in: InjectFrame, delay_us: u32) -> Result<(), FaceError> {
        self.inner.inject_at(frame_in, delay_us)
    }

    /// Read the C5's schedule clock — see [`SerialRadioBackend::read_schedule_clock`].
    pub async fn read_schedule_clock(&self) -> Option<u64> {
        self.inner.read_schedule_clock().await
    }

    /// Next scheduled-TX confirmation — see [`SerialRadioBackend::recv_tx_confirm`].
    pub async fn recv_tx_confirm(&self) -> Option<(u64, u64)> {
        self.inner.recv_tx_confirm().await
    }
}

#[async_trait]
impl FrameIo for Esp32SerialBackend {
    async fn inject(&self, frame: InjectFrame) -> Result<(), FaceError> {
        self.inner.inject(frame).await
    }
    async fn recv_frame(&self) -> Result<CapturedFrame, FaceError> {
        self.inner.recv_frame().await
    }
    /// Hardware scheduled placement: the C5 fires T_INJECT_ABS when its esp_timer reaches `target_tick`,
    /// so a scheduler places the frame in its slot without host sleep+inject jitter. `target_tick` is a
    /// value in the C5's schedule clock (esp_timer µs). See [`SerialRadioBackend::inject_at_abs`].
    async fn inject_at_clock(
        &self,
        frame: InjectFrame,
        target_tick: u64,
        _domain: ClockDomainId,
    ) -> Result<(), FaceError> {
        self.inner.inject_at_abs(frame, target_tick)
    }
    /// Relative hardware scheduling (T_INJECT_AT): the C5 fires the frame `delay_us` after it receives the
    /// command, on its own esp_timer — so the scheduler's slot_wait drives it with no clock reconcile.
    async fn inject_after(&self, frame: InjectFrame, delay_us: u64) -> Result<(), FaceError> {
        if delay_us == 0 {
            self.inner.inject(frame).await
        } else {
            self.inner
                .inject_at(frame, delay_us.min(u32::MAX as u64) as u32)
        }
    }
    /// Actuate cognition's rate lever. The C5 firmware decodes T_RATE as a `wifi_phy_rate_t` and calls the
    /// blob-internal `ic_set_80211_tx_rate_config` directly (the public wrapper ESP_FAILs in the C5's boot
    /// HE20 mode) — MEASURED on air up to MCS7/65 Mbps. The C5 is single-stream HT, so map the descriptor's
    /// MCS index (0–7) to the HT long-GI code `MCS0_LGI(0x10) + index`; the firmware derives HT20 phymode
    /// from it. (SGI/VHT/2SS aren't exposed by this bearer; index is clamped to the 1-stream HT range. This
    /// overrides the shared BW16 `set_rate`, whose T_RATE byte is an RTL8720DN rate code, not a phy_rate_t.)
    fn set_rate(&self, mcs: McsDescriptor) -> Result<(), FaceError> {
        if mcs.he {
            // 802.11ax reach path: phymode HE20 (6) + the DCM / ER-SU flags. Verified on air (RX HE_SU / HE_ERSU).
            // ★ HE ER-SU is only valid at MCS 0–2 (802.11ax) — the PHY silently drops the frame otherwise
            // (measured: MCS4+ER-SU → nothing on air), so clamp the index when ER-SU is requested.
            let idx = if mcs.er_su {
                mcs.index.min(2)
            } else {
                mcs.index.min(7)
            };
            let flags = (mcs.dcm as u8) | ((mcs.er_su as u8) << 1);
            self.inner.set_tx_rate_ex(0x10 + idx, 6, flags)
        } else {
            self.inner.set_tx_rate(0x10 + mcs.index.min(7)) // HT (phymode auto-derived by the firmware)
        }
    }
}

impl RadioKnobs for Esp32SerialBackend {
    fn set_channel(&self, channel: u8, bw: Bandwidth) -> Result<(), FaceError> {
        // FQ call: SerialRadioBackend has an inherent 1-arg `set_channel` that would shadow this.
        RadioKnobs::set_channel(self.inner.as_ref(), channel, bw)
    }
    fn set_tx_power(&self, idx: u32) -> Result<(), FaceError> {
        RadioKnobs::set_tx_power(self.inner.as_ref(), idx)
    }
    fn tx_discipline(&self) -> TxDiscipline {
        // The C5 firmware places T_INJECT_AT frames at a scheduled instant via its monotonic timer.
        // Measured error ≤ ~190 µs (dominated by the esp_wifi_80211_tx submission latency), so declare
        // a conservative 200 µs granularity — the scheduler learns the C5 can name an airtime slot.
        TxDiscipline::ScheduledAt {
            granularity_ns: 200_000,
        }
    }
    fn configure_name_filter(
        &self,
        enabled: bool,
        key: &[u8; 16],
        masks: &[[u8; 16]],
    ) -> Result<(), FaceError> {
        RadioKnobs::configure_name_filter(self.inner.as_ref(), enabled, key, masks)
    }
    fn read_channel_activity(&self) -> Result<Option<u16>, FaceError> {
        RadioKnobs::read_channel_activity(self.inner.as_ref())
    }
}

impl RadioTime for Esp32SerialBackend {
    fn time_sources(&self) -> Vec<RadioTimeSource> {
        // The C5's real link clock is its free-running per-frame RX stamp (rx_ctrl.timestamp, µs ticks),
        // latched on the device — a genuine hardware stamp, unlike the BW16's host-recv fallback. (The
        // 802.11 port TSF reads 0 while unassociated, so it is deliberately NOT advertised.)
        vec![RadioTimeSource::free_run_rx_stamp(self.clock_domain, 1_000)]
    }
    fn read_clock(&self, _domain: ClockDomainId) -> Result<Option<u64>, FaceError> {
        // Latch-only: the free-run RX stamp has no read-now over the serial link. (A T_READCLOCK
        // round-trip could expose esp_timer, but with serial jitter it would be worse than the per-frame
        // latch it shares a domain with — so leave it None rather than advertise a jittery read-now.)
        Ok(None)
    }
}

impl RadioProfile for Esp32SerialBackend {
    fn capability(&self) -> RadioCapability {
        self.capability.clone()
    }
}

fn io_err(msg: String) -> FaceError {
    FaceError::Io(std::io::Error::other(msg))
}
