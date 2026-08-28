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
use ndn_radio_hal::DbmRange;
use ndn_transport::FaceError;
use tokio::sync::{Mutex as AsyncMutex, mpsc};

const SYNC0: u8 = 0x4E;
const SYNC1: u8 = 0x44;
const T_INJECT: u8 = 0x01;
const T_CHANNEL: u8 = 0x02;
const T_TXPOWER: u8 = 0x03; // RTL8720DN: TXAGC index for every rate (0xFF = restore driver defaults)
const T_RATE: u8 = 0x04; // RTL8720DN: MGN rate code; ESP32-C5: wifi_phy_rate_t
const T_BW40: u8 = 0x05; // wext_set_bw40_enable
const T_INJECT_ATTR: u8 = 0x06; // poke pkt_attrib bytes, then inject
const T_NAMEFILTER: u8 = 0x07; // load on-device Tier-0 masks: [enabled][n_masks][mask 16B]*
const T_INJECT_AT: u8 = 0x09; // scheduled TX: [delay_us_le32][802.11 frame] (ESP32-C5 firmware only)
const T_INJECT_ABS: u8 = 0x0A; // scheduled TX at an ABSOLUTE esp_timer µs: [target_us_le64][frame]
const T_READCLOCK: u8 = 0x0B; // request the device's schedule clock; reply T_CLOCK [esp_timer_us_le64]
const T_CLOCK: u8 = 0x85; // reply to T_READCLOCK: [esp_timer_us_le64]
const T_POWER_PCT: u8 = 0x0D; // RTL8720DN coarse power: [idx] 0=100% 1=-1.5dB 2=-3dB 3=-6dB 4=-9dB
const T_READPOWER: u8 = 0x0E; // request the live TXAGC indices; reply T_POWERIDX
const T_POWERIDX: u8 = 0x89;
const T_READSTATS: u8 = 0x10; // request the ESP32-C5's hardware receive counters
const T_HWSTATS: u8 = 0x8B; // [rx_fcs_err][rx_abort][brx_err_agc][nrx_err_agcexit][nrx_err][rx_mpdu]
// [rx_fifo_ovf][rx_cfo_hz] — all u16 LE; the receive-side loss a frame count cannot show // reply to T_READPOWER: [status_i8][20 TXAGC bytes]
const T_LOG: u8 = 0x84; // device status text (the RTL8720DN's boot markers)
const T_BLE_TXPOWER: u8 = 0x35; // BLE advertising TX power. Units are per-radio: the ESP32-C5 takes an
// esp_power_level_t 0..15 (-24..+20 dBm, 3 dB/step, MEASURED 36.4 dB span); the RTL8720DN takes a
// controller gain index (~0.5 dB/step, MEASURED 22.2 dB span).
const T_BLE_PACE: u8 = 0x32; // RTL8720DN: [hold_ms_le16]([adv_int_min_ms][adv_int_max_ms]) — the BLE
// bearer's packet-rate lever (how long each payload is held on air before the next replaces it)
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

/// Realtek **MGN** rate codes — the units of `padapter[0x855]`, which is what the
/// RTL8720DN's management-TX descriptor actually reads: `rtl8721d_update_txdesc`
/// fills the descriptor's DATA_RATE from `MRateToHwRate(padapter[0x855])` and sets
/// the use-fixed-rate bit, so hardware rate adaptation cannot override it.
///
/// These are NOT the `wifi_set_tx_data_rate` codes. That API writes the **data**
/// path's fixed rate (`padapter[0x230c]`), which the management path — the one our
/// raw inject uses — ignores unless a driver global we never set is nonzero. Driving
/// it is why the rate knob previously actuated nothing on air, and why the earlier
/// conclusion ("the mgmt path is legacy-rate-locked; a data-path rewrite is needed")
/// was wrong: the mgmt path has a rate lever, just a different one.
///
/// For legacy rates an MGN code is the rate in 500 kb/s units (0x0C = 6 Mb/s,
/// 0x6C = 54 Mb/s) — the same convention radiotap uses.
///
/// MEASURED on air, all 17 values, witnessed by an ESP32-C5's per-frame PHY
/// metadata: every request produced exactly that rate and PHY format.
pub mod rate {
    pub const CCK_1M: u8 = 0x02;
    pub const CCK_2M: u8 = 0x04;
    pub const CCK_5_5M: u8 = 0x0B;
    pub const CCK_11M: u8 = 0x16;
    pub const OFDM_6M: u8 = 0x0C;
    pub const OFDM_24M: u8 = 0x30;
    pub const OFDM_54M: u8 = 0x6C;
    pub const HT_MCS0: u8 = 0x80;
    pub const HT_MCS7: u8 = 0x87;
    /// Hand the rate choice back to the driver.
    pub const AUTO: u8 = 0x00;

    /// The MGN code for a 1-stream HT MCS index. The RTL8720DN is 1x1 HT20, so 0–7.
    pub fn ht_mcs(index: u8) -> u8 {
        HT_MCS0 + index.min(7)
    }
}

/// Baud the firmware opens `Serial` at — the RTL8720 LOG UART's native rate,
/// shared with WiFi-driver debug (the deframer picks our SYNC'd frames out).
pub const SERIAL_RADIO_BAUD: u32 = 115_200;

/// A BW16 reached over its USB-serial port.
pub struct SerialRadioBackend {
    tx: Arc<Mutex<Box<dyn serialport::SerialPort>>>,
    format: FrameFormat,
    rx: AsyncMutex<mpsc::UnboundedReceiver<CapturedFrame>>,
    /// Device schedule-clock replies (`T_CLOCK` → device µs), routed here by the reader for
    /// [`read_schedule_clock`](Self::read_schedule_clock). Both firmwares reply.
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
    /// TX-power readback replies (`T_POWERIDX` → (status, the 20 live TXAGC indices)). The power
    /// knob's own instrument: it reads the registers back out of the hardware, so a power change can
    /// be proven to have landed before anything is claimed about the air. RTL8720DN only.
    /// Device request/reply traffic, tagged by type: `T_POWERIDX` (TX-power readback, shape per radio)
    /// and `T_HWSTATS` (the C5's hardware receive counters). One channel because both are rare,
    /// synchronous request→reply exchanges; the type tag keeps them from being mistaken for each other.
    reply_rx: AsyncMutex<mpsc::UnboundedReceiver<(u8, Vec<u8>)>>,
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

    /// Like [`open`](Self::open) (with the RTL8720DN's DTR→CEN reset pulse) but stamping frames in the
    /// given device clock `domain` — the RTL8720DN firmware's per-frame µs stamp.
    pub fn open_clocked(path: &str, domain: ClockDomainId) -> Result<Self, FaceError> {
        Self::open_inner(path, true, Some(domain))
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
        let (pwrch, pwr_rxch) = mpsc::unbounded_channel();
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
                pwrch,
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
            reply_rx: AsyncMutex::new(pwr_rxch),
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

    /// Pin the on-air TX rate. On the RTL8720DN this is a [`rate`] MGN code, re-asserted by the
    /// firmware before every inject (a band or channel change resets the driver's copy, so a
    /// set-once knob would silently revert the first time cognition retuned). On the ESP32-C5 it is
    /// a `wifi_phy_rate_t`. Both were MEASURED on air against a witness's per-frame PHY metadata.
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

    /// **RTL8720DN only.** How long each BLE payload is held on air, and the advertising interval (ms).
    ///
    /// Advertising is a repeating broadcast, not a send: new data replaces whatever is on air. The hold
    /// time is therefore the bearer's packet-rate lever, and its right value is a property of the *link*
    /// — a receiver duty-cycling its scan needs a longer hold to catch the same payload than one
    /// scanning continuously. MEASURED against a wide-open ESP32-C5 peer: 120 ms → 6.0 pkt/s at 30/30,
    /// 40 ms → 16.5 pkt/s still at 30/30, 20 ms → 24.2 pkt/s but only 26/30. Default 60 ms.
    ///
    /// `interval_ms` is the advertising interval; GAP's floor is 20 ms. Pass `None` to leave it alone.
    pub fn set_ble_pace(
        &self,
        hold_ms: u16,
        interval_ms: Option<(u16, u16)>,
    ) -> Result<(), FaceError> {
        let mut p = hold_ms.to_le_bytes().to_vec();
        if let Some((lo, hi)) = interval_ms {
            p.extend_from_slice(&lo.to_le_bytes());
            p.extend_from_slice(&hi.to_le_bytes());
        }
        self.send_framed(T_BLE_PACE, &p)
    }

    /// BLE **advertising** TX power — a reach lever for the broadcast bearer, independent of the
    /// Wi-Fi power knob (different register path, one shared PA).
    ///
    /// The unit is the radio's own and they differ, so the caller picks by radio:
    /// * ESP32-C5 — `esp_power_level_t` 0..15, i.e. −24 dBm to +20 dBm in 3 dB steps
    ///   (MEASURED: 36.4 dB span, R² = 0.977; at level 0 the link starts dropping frames).
    /// * RTL8720DN — a controller gain-table index, ~0.5 dB/step
    ///   (MEASURED: 22.2 dB span, R² = 0.993; the vendor table anchors 0x06 ≈ −10 dBm, 0x1A ≈ 0 dBm).
    pub fn set_ble_tx_power(&self, level: u8) -> Result<(), FaceError> {
        self.send_framed(T_BLE_TXPOWER, &[level])
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

    /// Set the TX power index.
    ///
    /// On the RTL8720DN this is the **TXAGC index** written to every rate's power register via
    /// phydm (`config_phydm_write_txagc_8721d`). MEASURED on air: **0.274 dB per step** (the
    /// driver's own arithmetic says 0.25), R² = 0.982 over a 31 dB span. `0xFF` restores the
    /// driver's computed per-rate power.
    ///
    /// This replaced the `txpower patha=<idx>` wext command the firmware used to send, which was a
    /// genuine no-op: that sub-command is registered under the private-ioctl **GET** family and its
    /// handler only parses and echoes. Sweeping it moved a witness's RSSI by 0.6 dB — i.e. noise.
    pub fn set_txpower(&self, idx: u8) -> Result<(), FaceError> {
        self.send_framed(T_TXPOWER, &[idx])
    }

    /// Restore the RTL8720DN's driver-computed per-rate TX power, undoing [`set_txpower`](Self::set_txpower).
    pub fn reset_txpower(&self, channel: u8) -> Result<(), FaceError> {
        self.send_framed(T_TXPOWER, &[0xFF, channel])
    }

    /// Coarse RTL8720DN TX power: 0=100%, 1=−1.5 dB, 2=−3 dB, 3=−6 dB, 4=−9 dB.
    ///
    /// Worth having beside the fine knob because it is applied through the driver's own percentage
    /// path, so the DM power-tracking watchdog will not silently reprogram it away — whereas direct
    /// TXAGC writes are stomped unless tracking is disabled (which [`set_txpower`] does).
    pub fn set_tx_power_pct(&self, idx: u8) -> Result<(), FaceError> {
        self.send_framed(T_POWER_PCT, &[idx.min(4)])
    }

    /// Read the RTL8720DN's 20 live TXAGC indices straight back out of the hardware:
    /// `[0..4]` CCK 1/2/5.5/11, `[4..12]` OFDM 6…54, `[12..20]` HT MCS0…7.
    ///
    /// The power knob's instrument. A TX-power claim that rests only on a witness's RSSI cannot
    /// separate "the register moved" from "the air changed for another reason"; this can.
    pub async fn read_txpower(&self) -> Option<(i8, [u8; 20])> {
        let p = self.request_reply(T_READPOWER, &[], T_POWERIDX).await?;
        if p.len() < 21 {
            return None;
        }
        let mut idx = [0u8; 20];
        idx.copy_from_slice(&p[1..21]);
        Some((p[0] as i8, idx))
    }

    /// Send a request and await the device reply of type `want` (raw payload).
    async fn request_reply(&self, ty: u8, payload: &[u8], want: u8) -> Option<Vec<u8>> {
        let mut rx = self.reply_rx.lock().await;
        while rx.try_recv().is_ok() {} // drop stale replies so we read OUR answer
        self.send_framed(ty, payload).ok()?;
        let deadline = tokio::time::Instant::now() + Duration::from_millis(600);
        loop {
            let left = deadline.saturating_duration_since(tokio::time::Instant::now());
            if left.is_zero() {
                return None;
            }
            match tokio::time::timeout(left, rx.recv()).await {
                Ok(Some((t, p))) if t == want => return Some(p),
                Ok(Some(_)) => continue, // a reply of the other kind; keep waiting for ours
                _ => return None,
            }
        }
    }

    /// **ESP32-C5.** The MAC/PHY receive counters the promiscuous callback cannot see: PPDUs whose
    /// preamble the PHY locked onto and whose payload then failed, plus AGC-level receive failures.
    /// Returns `(rx_mpdu, rx_fcs_err, brx_err_agc, rx_cfo_hz)`. Free-running and wrapping — difference
    /// them over an interval, never read absolutes.
    pub async fn read_hw_rx_stats(&self) -> Option<(u16, u16, u16, i16)> {
        let p = self.request_reply(T_READSTATS, &[], T_HWSTATS).await?;
        if p.len() < 16 {
            return None;
        }
        let g = |k: usize| u16::from_le_bytes([p[2 * k], p[2 * k + 1]]);
        Some((g(5), g(0), g(2), i16::from_le_bytes([p[14], p[15]])))
    }

    /// **ESP32-C5.** Set TX power on the absolute dBm scale and return what the radio actually
    /// applied. The wire unit is `esp_wifi_set_max_tx_power`'s: 0.25 dBm, valid `[8,84]` = 2..20 dBm,
    /// which the IDF then quantises to 11 steps — so the applied value is read back rather than
    /// assumed. The firmware clamps into range; passing a value below 8 used to be *rejected*, leaving
    /// the radio at full power (measured: a request of 1 dBm produced the same RSSI as 20 dBm).
    pub async fn set_max_tx_power_dbm(&self, dbm: i8) -> Option<i8> {
        let q = (dbm as i32 * 4).clamp(8, 84) as u8;
        let p = self.request_reply(T_TXPOWER, &[q], T_POWERIDX).await?;
        (p.len() >= 2).then(|| (p[1] as i8) / 4)
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
    reply: mpsc::UnboundedSender<(u8, Vec<u8>)>,
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
                    // T_LOG — device status text (boot markers, short-command warnings). Consume it
                    // explicitly: it shares no code with T_RX_TS now, but leaving it to fall through
                    // would let a status line be mistaken for a frame the moment its length reached 8.
                    if ty == T_LOG {
                        tracing::debug!(
                            target: "serial_radio",
                            msg = %String::from_utf8_lossy(&payload),
                            "device log"
                        );
                        acc.drain(..consumed);
                        continue;
                    }
                    // T_POWERIDX — a TX-power readback. The shape is per-radio: the RTL8720DN
                    // answers `[status][20 TXAGC bytes]`, the ESP32-C5 `[requested_q][applied_q]`
                    // in 0.25 dBm units. Forward the payload raw; each backend interprets its own.
                    if (ty == T_POWERIDX || ty == T_HWSTATS) && !payload.is_empty() {
                        let _ = reply.send((ty, payload.clone()));
                        acc.drain(..consumed);
                        continue;
                    }
                    // T_CLOCK [device_us_le64] — the device's schedule clock reply; route to read_schedule_clock.
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

// `set_rate` is deliberately NOT implemented on the shared transport: the two firmwares take
// different rate units on the same T_RATE byte (RTL8720DN = Realtek MGN, ESP32-C5 = wifi_phy_rate_t),
// so a shared mapping would silently transmit at the wrong rate on one of them. Each newtype
// (`Bw16SerialBackend`, `Esp32SerialBackend`) supplies its own.

impl RadioKnobs for SerialRadioBackend {
    fn set_channel(&self, channel: u8, bw: Bandwidth) -> Result<(), FaceError> {
        SerialRadioBackend::set_channel(self, channel)?;
        // Map the HAL bandwidth to the board's 40 MHz enable (the widest this SDK
        // exposes): Bw20 → off, wider → on.
        self.set_bw40(!matches!(bw, Bandwidth::Bw20))
    }
    fn set_tx_power(&self, idx: u32) -> Result<(), FaceError> {
        // RTL8720DN: the phydm TXAGC index, ~0.25 dB/step (measured 0.274). Clamp to 0..=126 so a
        // large cognition index cannot land on 0xFF, which the firmware reads as "restore defaults"
        // — i.e. asking for maximum power would instead give up control of it.
        self.set_txpower(idx.min(126) as u8)
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
        // Both firmwares emit their free-running frame counter as T_OCC ~5×/s; the reader caches the
        // latest. u32::MAX = no report yet (an older firmware, or one still booting) → honestly None.
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
        // The conservative profile for "some serial radio": single-chain 2.4 GHz 11n. Both real
        // devices are dual-band and override this — open through [`Bw16SerialBackend`] or
        // [`Esp32SerialBackend`] to get the measured profile, or the planner will never pick 5 GHz.
        RadioCapability::wifi_monitor_2ghz_1ss(vec![1, 6, 11])
    }
}

/// A stable per-device clock domain for an RTL8720DN, derived from its port path (as
/// [`c5_clock_domain`] does for the C5) so two boards on one host never share a timeline.
pub fn bw16_clock_domain(path: &str) -> ClockDomainId {
    let mut h: u32 = 0x811c_9dc5;
    for b in path.bytes() {
        h ^= b as u32;
        h = h.wrapping_mul(0x0100_0193);
    }
    ClockDomainId((h & 0x00ff_ffff) | 0x4257_0000) // "BW" tag in the top bytes
}

/// **BW16 / RTL8720DN serial-bridge backend** — the RTL8720DN's own identity, the sibling of
/// [`Esp32SerialBackend`].
///
/// [`SerialRadioBackend`] is the shared transport (framing, reader, demux). This newtype carries what
/// is specific to the RTL8720DN and would be wrong on the C5:
///
/// * **Dual-band.** The RTL8720DN is a 2.4 + 5 GHz part, and both bands were verified on air in both
///   directions (ch36 and ch149, TX and RX). The shared profile is 2.4-only, so cognition driving a
///   BW16 through it would never choose a 5 GHz channel.
/// * **Rate units.** `T_RATE` carries a Realtek **MGN** code here, not the C5's `wifi_phy_rate_t`.
/// * **A device clock.** The firmware stamps every frame from a monotonic µs timeline and answers
///   `T_READCLOCK`, so frames carry a device stamp rather than a host-receive time.
/// * **Scheduled TX.** `T_INJECT_AT`/`T_INJECT_ABS` place a frame at a named instant.
pub struct Bw16SerialBackend {
    inner: Arc<SerialRadioBackend>,
    capability: RadioCapability,
    clock_domain: ClockDomainId,
}

impl Bw16SerialBackend {
    /// Open a BW16 running the `firmware/bw16-rs` bridge. Pulses DTR→CEN so a freshly-flashed board
    /// boots our firmware without a manual reset, and stamps frames in this device's clock domain.
    pub fn open(path: &str) -> Result<Self, FaceError> {
        let clock_domain = bw16_clock_domain(path);
        Ok(Self {
            inner: Arc::new(SerialRadioBackend::open_clocked(path, clock_domain)?),
            // MEASURED on air, both directions: 2.4 GHz ch6 and 5 GHz ch36/ch149. The RTL8720DN is
            // 1x1 HT20 — it tops out at HT MCS7 (65 Mb/s), with no VHT and no HE, so no `.with_he()`:
            // claiming the HE reach levers here would make cognition escalate to a modulation this
            // PHY cannot emit.
            capability: RadioCapability::wifi_monitor_dual_1ss(vec![
                1, 6, 11, 36, 40, 44, 48, 149, 153, 157, 161,
            ]),
            clock_domain,
        })
    }

    /// Open the BW16 and bundle it as an [`OpenRadio`] — io + knobs + time + profile backed by one
    /// instance, so the measured dual-band profile survives into the engine.
    pub fn open_radio(path: &str) -> Result<OpenRadio, FaceError> {
        let dev = Arc::new(Self::open(path)?);
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

    /// The shared transport behind this view (for the wire-level helpers: `read_txpower`,
    /// `set_tx_power_pct`, `inject_at_abs`, `read_schedule_clock`, `recv_tx_confirm`, …).
    pub fn inner(&self) -> &Arc<SerialRadioBackend> {
        &self.inner
    }

    // Inherent delegates for the wire-level knobs. Spelled out rather than reached through a
    // `Deref` to the transport: `RadioKnobs` is also implemented here, and its `set_channel` takes
    // a bandwidth, so a deref-based single-argument `set_channel` would resolve to the trait method
    // and fail on arity. Inherent methods win resolution, which is exactly what a caller holding a
    // `Bw16SerialBackend` means.

    /// Retune the radio (2.4 or 5 GHz channel), leaving the bandwidth alone.
    pub fn set_channel(&self, channel: u8) -> Result<(), FaceError> {
        self.inner.set_channel(channel)
    }
    /// Pin the management-TX rate to a [`rate`] MGN code (`rate::AUTO` gives it back to the driver).
    pub fn set_tx_rate(&self, code: u8) -> Result<(), FaceError> {
        self.inner.set_tx_rate(code)
    }
    /// Set the TXAGC index for every rate (~0.25 dB/step); `0xFF` restores driver defaults.
    pub fn set_txpower(&self, idx: u8) -> Result<(), FaceError> {
        self.inner.set_txpower(idx)
    }
    /// Read the 20 live TXAGC indices back out of the hardware.
    pub async fn read_txpower(&self) -> Option<(i8, [u8; 20])> {
        self.inner.read_txpower().await
    }
    /// Enable/disable 40 MHz channel bandwidth.
    pub fn set_bw40(&self, enable: bool) -> Result<(), FaceError> {
        self.inner.set_bw40(enable)
    }
    /// Poke `pkt_attrib` bytes before injecting — the driver-RE probe harness.
    pub fn inject_attr(&self, frame: &[u8], pairs: &[(u8, u8)]) -> Result<(), FaceError> {
        self.inner.inject_attr(frame, pairs)
    }

    /// Open with an explicit on-air frame format (default `RawNdn`). Taken at open rather than as a
    /// `with_format` builder because the transport is already behind an `Arc` by then — and the
    /// format must be fixed before the reader thread starts parsing with it.
    pub fn open_with_format(path: &str, format: FrameFormat) -> Result<Self, FaceError> {
        let clock_domain = bw16_clock_domain(path);
        let inner = SerialRadioBackend::open_clocked(path, clock_domain)?.with_format(format);
        Ok(Self {
            inner: Arc::new(inner),
            capability: RadioCapability::wifi_monitor_dual_1ss(vec![
                1, 6, 11, 36, 40, 44, 48, 149, 153, 157, 161,
            ]),
            clock_domain,
        })
    }
}

#[async_trait]
impl FrameIo for Bw16SerialBackend {
    async fn inject(&self, frame_in: InjectFrame) -> Result<(), FaceError> {
        self.inner.inject(frame_in).await
    }

    async fn recv_frame(&self) -> Result<CapturedFrame, FaceError> {
        self.inner.recv_frame().await
    }

    /// Actuate cognition's rate lever.
    ///
    /// The RTL8720DN is 1x1 HT20, so an `McsDescriptor` index maps to the HT MGN code
    /// `MGN_MCS0 (0x80) + index`, clamped to MCS7. MEASURED on air across every rate the chip has.
    /// The `he`/`dcm`/`er_su` reach flags are ignored on purpose — this is an 802.11n PHY and
    /// silently substituting an HT rate for a requested HE one is the honest behaviour, since the
    /// capability never advertises `he_cap` for cognition to escalate into.
    fn set_rate(&self, mcs: McsDescriptor) -> Result<(), FaceError> {
        self.inner.set_tx_rate(rate::ht_mcs(mcs.index))
    }

    /// Real scheduled placement: this backend implements `inject_after`, so the caller may
    /// skip its software gate and hand us the delay. Moves with that implementation.
    fn schedules_tx(&self) -> bool {
        true
    }

    async fn inject_after(&self, frame_in: InjectFrame, delay_us: u64) -> Result<(), FaceError> {
        // T_INJECT_AT carries a u32 µs delay, and the firmware refuses anything past its
        // busy-wait cap anyway — saturate rather than wrap, so an over-long delay is rejected
        // by the device instead of silently becoming a near-immediate transmission.
        self.inner
            .inject_at(frame_in, delay_us.min(u32::MAX as u64) as u32)
    }
}

impl RadioKnobs for Bw16SerialBackend {
    fn set_channel(&self, channel: u8, bw: Bandwidth) -> Result<(), FaceError> {
        RadioKnobs::set_channel(self.inner.as_ref(), channel, bw)
    }
    fn set_tx_power(&self, idx: u32) -> Result<(), FaceError> {
        RadioKnobs::set_tx_power(self.inner.as_ref(), idx)
    }
    fn tx_discipline(&self) -> TxDiscipline {
        // The firmware spins on its µs timeline to the target instant, then injects. MEASURED
        // submit error: a constant 10 µs across 15 frames (the read-back overhead), so declare a
        // conservative 20 µs granularity. Tighter than the C5's 200 µs — the busy-wait is exact;
        // what it cannot control is CSMA backoff between submission and the air.
        TxDiscipline::ScheduledAt {
            granularity_ns: 20_000,
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

impl RadioTime for Bw16SerialBackend {
    fn time_sources(&self) -> Vec<RadioTimeSource> {
        // Stamped in the promiscuous callback from the SDK's µs ticker — earlier than any host
        // timestamp, but taken in the driver's RX path rather than latched by the MAC, so it carries
        // the driver's delivery jitter. Declared at 1 µs resolution, which is the clock's; the
        // *accuracy* is bounded by that jitter and is not claimed here. (The MAC's own RXTSFL does
        // sit in the RX descriptor, but the blob discards it before our callback ever runs.)
        vec![RadioTimeSource::free_run_rx_stamp(self.clock_domain, 1_000)]
    }
    fn read_clock(&self, _domain: ClockDomainId) -> Result<Option<u64>, FaceError> {
        // Latch-only, as on the C5: `T_READCLOCK` exists and answers, but a serial round-trip is a
        // worse estimate of "now" than the per-frame latch it shares a domain with.
        Ok(None)
    }
}

impl RadioProfile for Bw16SerialBackend {
    fn capability(&self) -> RadioCapability {
        self.capability.clone()
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
                .with_he()
                // The C5's power really is an absolute scale (esp_wifi_set_max_tx_power, 2..20 dBm),
                // so advertise it: that is what lets cognition reason in dBm across bearers instead
                // of in per-chip indices.
                .with_tx_power_dbm(DbmRange::new(2, 20)),
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
    /// Real scheduled placement: this backend implements `inject_after`, so the caller may
    /// skip its software gate and hand us the delay. Moves with that implementation.
    fn schedules_tx(&self) -> bool {
        true
    }

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
    /// The C5's power unit is **0.25 dBm** (`esp_wifi_set_max_tx_power`), valid `[8,84]` = 2..20 dBm —
    /// NOT the RTL8720DN's TXAGC index, so this must not inherit the shared implementation. `idx` is
    /// taken as that quarter-dBm value and clamped into range.
    ///
    /// Prefer [`set_tx_power_dbm`](RadioKnobs::set_tx_power_dbm): this radio's power really is an
    /// absolute scale, and the index form throws that away.
    fn set_tx_power(&self, idx: u32) -> Result<(), FaceError> {
        self.inner.set_txpower(idx.clamp(8, 84) as u8)
    }

    /// The portable absolute-power knob, returning the power the radio actually applied — which the
    /// firmware reads back with `esp_wifi_get_max_tx_power`, because the IDF quantises a request to 11
    /// discrete steps and the applied value routinely differs from the requested one (21 dBm applies as
    /// 20). MEASURED across the full range: monotonic, 22.6 dB span, R² = 0.974.
    fn set_tx_power_dbm(&self, dbm: i8) -> Result<i8, FaceError> {
        // The readback is a wire round-trip and this trait method is sync. Inside a runtime, use
        // block_in_place so blocking on it cannot panic with "cannot start a runtime from within a
        // runtime"; outside one, set without the readback and report the clamped request.
        let inner = self.inner.clone();
        match tokio::runtime::Handle::try_current() {
            Ok(h) => tokio::task::block_in_place(|| h.block_on(inner.set_max_tx_power_dbm(dbm)))
                .ok_or_else(|| io_err("no TX-power readback from the device".into())),
            Err(_) => {
                let q = (dbm as i32 * 4).clamp(8, 84) as u8;
                self.inner.set_txpower(q)?;
                Ok((q / 4) as i8)
            }
        }
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

    /// `(ok, err)` PPDU counters from the C5's hardware MAC/PHY registers.
    ///
    /// `ok` is `rx_mpdu`; `err` is `rx_fcs_err + brx_err_agc` — PPDUs the PHY began to demodulate and
    /// failed. That is precisely the half of channel activity the promiscuous callback cannot see: it
    /// only ever reports clean receptions, so an occupancy built from it structurally under-reports a
    /// busy or contended channel. MEASURED on an idle bench: 760 FCS errors and 125 AGC failures
    /// against 14227 good MPDUs, none of which reached the callback.
    fn read_ofdm_counters(&self) -> Result<Option<(u16, u16)>, FaceError> {
        let inner = self.inner.clone();
        let got = match tokio::runtime::Handle::try_current() {
            Ok(h) => tokio::task::block_in_place(|| h.block_on(inner.read_hw_rx_stats())),
            Err(_) => return Ok(None), // no runtime to do the round-trip on
        };
        Ok(got.map(|(mpdu, fcs_err, agc_err, _cfo)| (mpdu, fcs_err.saturating_add(agc_err))))
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
