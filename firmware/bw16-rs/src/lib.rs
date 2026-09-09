//! BW16 (RTL8720DN) named-radio firmware — **in Rust**.
//!
//! The first Rust-on-Ameba firmware: a `no_std` staticlib holding all the bridge
//! logic (serial framing, command dispatch, the promiscuous NDN capture,
//! the radio knobs, the schedule clock), linked into the AmebaD image beside the
//! closed Realtek WiFi blobs. A small C++ shim (`bw16-rs-sketch.ino`) is the
//! *only* C: it wraps the Arduino/SDK C calls behind the `c_*` FFI below and
//! forwards `setup()`/`loop()` and the promiscuous-RX callback to us.
//!
//! ## Wire protocol — the SAME one the ESP32-C5 firmware speaks
//!
//! `[0x4E 0x44] [type:u8] [len:u16 LE] [payload]`
//!
//! | dir | type | meaning |
//! |-----|------|---------|
//! | h→d | 0x01 `T_INJECT` | complete 802.11 frame (no FCS) |
//! | h→d | 0x02 `T_CHANNEL` | `[ch]` |
//! | h→d | 0x03 `T_TXPOWER` | `[idx]` |
//! | h→d | 0x04 `T_RATE` | `[rate]` |
//! | h→d | 0x05 `T_BW40` | `[0/1]` |
//! | h→d | 0x06 `T_INJECT_ATTR` | `[n][off,val]*n[frame]` — pkt_attrib RE probe |
//! | h→d | 0x09 `T_INJECT_AT` | `[delay_us_le32][frame]` — scheduled TX |
//! | h→d | 0x0A `T_INJECT_ABS` | `[target_us_le64][frame]` — slot-lease TX |
//! | h→d | 0x0B `T_READCLOCK` | — reply `T_CLOCK` |
//! | d→h | 0x82 `T_RX_TS` | `[rssi][noise][rate_code][phy_flags][rx_ts_us_le32][frame]` |
//! | d→h | 0x83 `T_TXTIME` | `[target_le64][actual_le64][tsf_le64]` |
//! | d→h | 0x84 `T_LOG` | text |
//! | d→h | 0x85 `T_CLOCK` | `[us_le64]` |
//! | d→h | 0x86 `T_OCC` | `[activity_count_le32]` |
//!
//! `T_LOG` moved from 0x82 to 0x84 so 0x82 means the same thing on both radios:
//! one protocol, two different silicon families. (The old BW16-only `T_RX` 0x81
//! is gone; the host parses `T_RX_TS` from either device.)
//!
//! ## Why everything is queued
//!
//! The promiscuous callback runs in the **WiFi task** and `rust_loop` in the
//! Arduino task. The old firmware wrote RX frames to `Serial` straight from the
//! callback, which was safe only because nothing else ever transmitted. Now that
//! the loop also emits `T_OCC`/`T_CLOCK`/`T_TXTIME`, two writers would interleave
//! *mid-message* and corrupt the framing. So every outbound message goes through
//! one byte ring: producers append whole messages under a short critical section,
//! and `rust_loop` is the only writer to `Serial`.

#![no_std]
#![allow(static_mut_refs)]

use core::panic::PanicInfo;
use core::sync::atomic::{AtomicU32, AtomicUsize, Ordering};

// --- the C shim's wrappers (Arduino + Ameba SDK, the closed WiFi blob) ---
extern "C" {
    fn c_serial_begin(baud: u32);
    fn c_serial_write(buf: *const u8, len: u32);
    fn c_serial_read() -> i32; // -1 when no byte is ready
    fn c_serial_available() -> i32;
    fn c_delay(ms: u32);
    fn c_millis() -> u32;
    fn c_micros() -> u32; // us_ticker_read(): monotonic µs, 1 µs granularity
    fn c_enter_critical();
    fn c_exit_critical();
    fn c_wifi_on_sta();
    fn c_wifi_set_channel(ch: i32);
    fn c_wifi_tx_raw_frame(buf: *const u8, len: u32);
    fn c_wifi_tx_raw_frame_attr(frame: *const u8, len: u32, pairs: *const u8, n_pairs: u32);
    fn c_wifi_set_promisc_enable();
    fn c_wifi_set_tx_data_rate(code: u8);
    fn c_wext_set_bw40(enable: u8);
    fn c_wifi_set_txpower(idx: i32);
    fn c_set_mgnt_rate(mgn: u8);
    fn c_set_txagc(idx: u32) -> i32;
    fn c_reset_txagc(channel: u8) -> i32;
    fn c_set_tx_power_pct(idx: u32) -> i32;
    fn c_get_txagc(out20: *mut u8) -> i32;
    fn c_ble_init();
    fn c_ble_ready() -> i32;
    fn c_ble_adv_start(payload: *const u8, len: u32);
    fn c_ble_adv_stop();
    fn c_ble_coex(win_units: u16, itvl_units: u16);
    fn c_ble_adv_interval(min_ms: u16, max_ms: u16);
    fn c_ble_features(out8: *mut u8);
    fn c_ble_scanning() -> i32;
    fn c_ble_scan_restart();
    fn c_ble_set_tx_power(gain: u8) -> i32;
    fn c_ble_scan_filter(enable: i32) -> i32;
    fn c_ble_hci_raw(opcode: u16, params: *mut u8, len: u16) -> i32;
}

const SYNC0: u8 = 0x4E;
const SYNC1: u8 = 0x44;

const T_INJECT: u8 = 0x01;
const T_CHANNEL: u8 = 0x02;
const T_TXPOWER: u8 = 0x03;
const T_RATE: u8 = 0x04;
const T_BW40: u8 = 0x05;
const T_INJECT_ATTR: u8 = 0x06;
const T_INJECT_AT: u8 = 0x09;
const T_INJECT_ABS: u8 = 0x0A;
const T_READCLOCK: u8 = 0x0B;

const T_RX_TS: u8 = 0x82;
const T_TXTIME: u8 = 0x83;
const T_LOG: u8 = 0x84;
const T_CLOCK: u8 = 0x85;
const T_OCC: u8 = 0x86;
/// `[userdata bytes]` — a raw dump of the SDK's per-frame `ieee80211_frame_info_t`,
/// so the struct layout is *measured* against known fields (the sender's MAC and
/// sequence number appear in it) instead of trusted from a header whose config
/// flags may differ from the blob's. Armed by `T_RXINFO_ARM`.
const T_RXINFO: u8 = 0x87;
const T_RXINFO_ARM: u8 = 0x0C;
/// `[idx]` — coarse, stomp-proof power: 0=100%, 1=−1.5 dB, 2=−3 dB, 3=−6 dB, 4=−9 dB.
const T_POWER_PCT: u8 = 0x0D;
/// no payload — reply `T_POWERIDX` with the 20 live TXAGC bytes read back from
/// the hardware. The knob's own instrument: it proves a write landed in the
/// register before anything is claimed about the air.
const T_READPOWER: u8 = 0x0E;
/// `[status_i8][20 TXAGC bytes]` — CCK 1/2/5.5/11, OFDM 6..54, HT MCS0..7.
const T_POWERIDX: u8 = 0x89;
/// `[payload]` — broadcast as a BLE advertisement (manufacturer AD, company 0x4E44).
const T_BLE_ADV: u8 = 0x30;
/// `[scan_window_le16][scan_itvl_le16]` (0.625 ms units) — the BLE↔Wi-Fi radio-time split.
const T_COEX: u8 = 0x31;
/// `[rssi_i8][addr6][payload]` — a scanned advertisement carrying our magic.
const T_BLE_RX: u8 = 0x88;

/// Legacy advertising caps the payload at 31 − 4 = 27 bytes on this part (BLE 5
/// extended advertising is compiled out of the shipped BT stack).
const BLE_MAXPAY: usize = 27;
/// `[burst_ms_le16]` or `[burst_ms_le16][int_min_ms][int_max_ms]` — how long each
/// payload is held on air, and the advertising interval. Tunable at runtime because
/// the right burst is a measured property of the link, not a constant: too short and
/// a payload never radiates, too long and the bearer's packet rate collapses.
const T_BLE_PACE: u8 = 0x32;
/// no payload — reply `T_BLE_CAPS` with the controller's 8-byte LE feature mask.
const T_BLE_READCAPS: u8 = 0x33;
/// `[opcode_le16][params]` — send a raw HCI command. The RE harness for features the
/// host stack was built without; the controller may still implement them.
const T_BLE_HCI: u8 = 0x34;
/// `[8-byte LE feature mask]` — reply to `T_BLE_READCAPS`.
const T_BLE_CAPS: u8 = 0x8A;
/// `[gain]` — BLE advertising TX power, ~0.5 dB/step (0x06 ≈ −10 dBm, 0x1A ≈ 0 dBm,
/// 0x23 ≈ +4.5 dBm). Separate from the Wi-Fi TXAGC knob.
const T_BLE_TXPOWER: u8 = 0x35;
/// `[enabled]` — drop adverts without our company magic before they are queued,
/// the BLE analogue of a pre-link drop. Off by default so the ambient
/// advert census (the BLE occupancy signal) still works.
const T_BLE_FILTER: u8 = 0x36;

/// Default hold time per payload.
///
/// MEASURED against an ESP32-C5 peer with a wide-open scan, 30 payloads per point:
///
/// | hold | delivered | rate |
/// |------|-----------|------|
/// | 120 ms | 30/30 | 6.0 pkt/s |
/// | 60 ms | 30/30 | 11.7 pkt/s |
/// | 40 ms | 30/30 | 16.5 pkt/s |
/// | 30 ms | 29/30 | 21.1 pkt/s |
/// | 20 ms | 26/30 | 24.2 pkt/s |
///
/// 40 ms is the lossless knee, but it is the knee *for a receiver that is scanning
/// continuously*; a receiver duty-cycling its scan to give Wi-Fi airtime needs a
/// longer hold to catch the same payload. 60 ms is the default because it doubles
/// the old throughput while keeping margin for a receiver that is not wide open —
/// and the host can tune it per link with `T_BLE_PACE`.
const BLE_BURST_MS_DEFAULT: u32 = 60;
static BLE_BURST_MS: AtomicU32 = AtomicU32::new(BLE_BURST_MS_DEFAULT);

const MAX_FRAME: usize = 1600;
/// Cap the scheduled-TX busy-wait to about one slot period. A hardware timer
/// would avoid the spin entirely; until the RTL8720DN's is mapped, the spin is
/// what makes the instant precise, so it must stay bounded.
const SCHED_MAX_DELAY_US: u64 = 100_000;

// ---------------------------------------------------------------------------
// Outbound ring — one writer to Serial, many producers.
// ---------------------------------------------------------------------------

const RING: usize = 8192;
static mut RB: [u8; RING] = [0; RING];
static HEAD: AtomicUsize = AtomicUsize::new(0);
static TAIL: AtomicUsize = AtomicUsize::new(0);
/// Messages dropped because the ring was full — a real signal (the 115200 LOG
/// UART is this radio's true bottleneck), reported rather than hidden.
static DROPPED: AtomicU32 = AtomicU32::new(0);

/// Append one complete `[SYNC SYNC ty len payload…]` message, or drop it whole.
///
/// Whole-message-or-nothing is the invariant that keeps the host's deframer in
/// sync: a partial write under back-pressure would desynchronise the stream far
/// more expensively than a dropped frame.
fn ring_push(ty: u8, parts: &[&[u8]]) {
    let body: usize = parts.iter().map(|p| p.len()).sum();
    let total = 5 + body;
    unsafe {
        c_enter_critical();
        let head = HEAD.load(Ordering::Relaxed);
        let tail = TAIL.load(Ordering::Acquire);
        let used = head.wrapping_sub(tail);
        if used + total > RING - 1 {
            c_exit_critical();
            DROPPED.fetch_add(1, Ordering::Relaxed);
            return;
        }
        let mut w = head;
        let hdr = [SYNC0, SYNC1, ty, (body & 0xff) as u8, ((body >> 8) & 0xff) as u8];
        for &b in hdr.iter() {
            RB[w % RING] = b;
            w = w.wrapping_add(1);
        }
        for p in parts {
            for &b in p.iter() {
                RB[w % RING] = b;
                w = w.wrapping_add(1);
            }
        }
        HEAD.store(w, Ordering::Release);
        c_exit_critical();
    }
}

/// Drain at most `budget` bytes to the UART. Bounded so a burst of RX cannot
/// starve command handling (and so scheduled TX still lands on time).
fn ring_drain(budget: usize) {
    let head = HEAD.load(Ordering::Acquire);
    let mut tail = TAIL.load(Ordering::Relaxed);
    let mut sent = 0;
    while tail != head && sent < budget {
        let start = tail % RING;
        let contiguous = core::cmp::min(RING - start, head.wrapping_sub(tail));
        let n = core::cmp::min(contiguous, budget - sent);
        unsafe { c_serial_write(RB.as_ptr().add(start), n as u32) };
        tail = tail.wrapping_add(n);
        sent += n;
    }
    TAIL.store(tail, Ordering::Release);
}

fn logmsg(s: &[u8]) {
    ring_push(T_LOG, &[s]);
}

// ---------------------------------------------------------------------------
// The schedule clock: a 64-bit µs timeline from Arduino's 32-bit `micros()`.
// ---------------------------------------------------------------------------

static CLK_LAST: AtomicU32 = AtomicU32::new(0);
static CLK_EPOCH: AtomicU32 = AtomicU32::new(0);

/// Monotonic microseconds, wrap-extended to 64 bits.
///
/// `micros()` wraps every ~71.6 minutes; a slot lease scheduled across a wrap
/// would target an instant in the past and fire immediately. Extending here — in
/// a critical section, so the two calling contexts cannot both observe the wrap —
/// makes the timeline monotonic for as long as the board is powered.
fn now_us() -> u64 {
    unsafe {
        c_enter_critical();
        let raw = c_micros();
        let last = CLK_LAST.load(Ordering::Relaxed);
        // Distinguish a genuine 32-bit wrap from this platform's ~1 ms backward
        // jitter: only a decrease of more than half the range is a wrap. Treating
        // every decrease as one (the obvious version) adds 2^32 µs per jitter step
        // — which made the clock jump forward on almost every call.
        let now = if raw < last {
            if last - raw > 0x8000_0000 {
                CLK_EPOCH.fetch_add(1, Ordering::Relaxed);
                raw
            } else {
                last // jitter: hold the floor rather than go backwards
            }
        } else {
            raw
        };
        CLK_LAST.store(now, Ordering::Relaxed);
        let epoch = CLK_EPOCH.load(Ordering::Relaxed) as u64;
        c_exit_critical();
        (epoch << 32) | now as u64
    }
}

// ---------------------------------------------------------------------------
// Occupancy
// ---------------------------------------------------------------------------

/// Every frame the radio decodes, of any type — the frame-free occupancy proxy
/// the host reads through `RadioKnobs::read_channel_activity`.
static ACTIVITY: AtomicU32 = AtomicU32::new(0);

// ---------------------------------------------------------------------------
// BLE bearer: a PACED advertiser.
// ---------------------------------------------------------------------------
//
// Advertising is a repeating broadcast, not a send: setting new data replaces
// what is on air. Handing each `T_BLE_ADV` straight to the controller therefore
// loses every payload but the last whenever they arrive back-to-back — which is
// exactly what a fragmented NDN packet does, so reassembly would never complete.
// (The C5 firmware hit this and fixed it the same way.) So each payload is
// queued and given its own burst before the next replaces it.

const ADVQ_LEN: usize = 8;
struct AdvQueue {
    buf: [[u8; BLE_MAXPAY]; ADVQ_LEN],
    len: [u8; ADVQ_LEN],
    head: usize,
    tail: usize,
}
static mut ADVQ: AdvQueue = AdvQueue {
    buf: [[0; BLE_MAXPAY]; ADVQ_LEN],
    len: [0; ADVQ_LEN],
    head: 0,
    tail: 0,
};
/// Set while an advertisement is mid-burst; cleared when its window expires.
static mut ADV_UNTIL: u64 = 0;
static mut ADV_ACTIVE: bool = false;

fn advq_push(payload: &[u8]) {
    if payload.is_empty() || payload.len() > BLE_MAXPAY {
        return;
    }
    unsafe {
        c_enter_critical();
        let next = (ADVQ.head + 1) % ADVQ_LEN;
        if next != ADVQ.tail {
            ADVQ.buf[ADVQ.head][..payload.len()].copy_from_slice(payload);
            ADVQ.len[ADVQ.head] = payload.len() as u8;
            ADVQ.head = next;
        }
        c_exit_critical();
    }
}

/// Advance the advertiser: retire an expired burst, then start the next payload.
fn advq_service(now: u64) {
    unsafe {
        if ADV_ACTIVE {
            if now < ADV_UNTIL {
                return;
            }
            c_ble_adv_stop();
            ADV_ACTIVE = false;
        }
        if ADVQ.tail == ADVQ.head {
            return;
        }
        let i = ADVQ.tail;
        let n = ADVQ.len[i] as usize;
        ADVQ.tail = (i + 1) % ADVQ_LEN;
        c_ble_adv_start(ADVQ.buf[i].as_ptr(), n as u32);
        ADV_ACTIVE = true;
        ADV_UNTIL = now + (BLE_BURST_MS.load(Ordering::Relaxed) as u64) * 1000;
    }
}

/// Every advertisement the scanner reports, before the magic filter — the BLE
/// analogue of the Wi-Fi activity counter, and the honest answer to "is the
/// scanner running at all?".
static BLE_SEEN: AtomicU32 = AtomicU32::new(0);

/// Whether the BLE stack came up — gates the scanner watchdog.
static mut BLE_UP: bool = false;

#[no_mangle]
pub extern "C" fn rust_ble_seen() {
    BLE_SEEN.fetch_add(1, Ordering::Relaxed);
}

/// A scanned advertisement carrying our company magic (called from the BT task).
#[no_mangle]
pub extern "C" fn rust_ble_rx(rssi: i8, addr: *const u8, payload: *const u8, len: u32) {
    let len = len as usize;
    if addr.is_null() || payload.is_null() || len == 0 || len > BLE_MAXPAY {
        return;
    }
    let a = unsafe { core::slice::from_raw_parts(addr, 6) };
    let p = unsafe { core::slice::from_raw_parts(payload, len) };
    ring_push(T_BLE_RX, &[&[rssi as u8], a, p]);
}

/// Remaining `ieee80211_frame_info_t` dumps to emit (a layout probe, off by default).
static RXINFO_ARM: AtomicU32 = AtomicU32::new(0);

/// Emit the raw per-frame info struct while armed.
#[no_mangle]
pub extern "C" fn rust_rxinfo_dump(ud: *const u8, n: u32) {
    if RXINFO_ARM.load(Ordering::Relaxed) == 0 || ud.is_null() {
        return;
    }
    RXINFO_ARM.fetch_sub(1, Ordering::Relaxed);
    let s = unsafe { core::slice::from_raw_parts(ud, n as usize) };
    ring_push(T_RXINFO, &[s]);
}

// ---------------------------------------------------------------------------
// Entry points
// ---------------------------------------------------------------------------

/// Called by the C shim from `setup()`.
#[no_mangle]
pub extern "C" fn rust_setup() {
    unsafe {
        c_serial_begin(115_200);
        c_delay(200);
    }
    logmsg(b"boot-rs");
    ring_drain(256);
    unsafe { c_wifi_on_sta() };
    logmsg(b"wifi_on");
    unsafe { c_wifi_set_channel(6) };
    logmsg(b"ch6");
    unsafe { c_wifi_set_promisc_enable() };
    // Flush what we have BEFORE bringing BLE up. BT bring-up blocks until its
    // stack reports ready, so if it ever fails to, the Wi-Fi markers must already
    // be on the wire — otherwise a BT problem is indistinguishable from a board
    // that never booted at all.
    logmsg(b"wifi-ready");
    ring_drain(1024);
    // BLE last: its bring-up blocks until Wi-Fi is up and then arms Wi-Fi/BT
    // coexistence, so it cannot run before wifi_on.
    unsafe { c_ble_init() };
    if unsafe { c_ble_ready() } != 0 {
        unsafe { BLE_UP = true };
        logmsg(b"ble-ready");
    }
    logmsg(b"ready-rs");
    ring_drain(1024);
}

/// Promiscuous RX, in the WiFi task: stamp, count, filter, queue.
///
/// The stamp is taken **first**, before any work, so it dates the frame's arrival
/// rather than our processing of it.
#[no_mangle]
pub extern "C" fn rust_promisc_cb(buf: *const u8, len: u32, rssi: i8, mrate: u8) {
    let ts = now_us() as u32;
    ACTIVITY.fetch_add(1, Ordering::Relaxed); // count ALL activity, before any filter
    let len = len as usize;
    if len < 32 || len > MAX_FRAME || buf.is_null() {
        return;
    }
    let frame = unsafe { core::slice::from_raw_parts(buf, len) };
    if frame[0] & 0x0C != 0x08 {
        return; // must be a DATA frame
    }
    // LLC/SNAP aa aa 03 at [24..27] and the NDN ethertype 0x8624 at [30..32].
    if frame[24] != 0xaa || frame[25] != 0xaa || frame[26] != 0x03 {
        return;
    }
    if frame[30] != 0x86 || frame[31] != 0x24 {
        return;
    }
    // Relevance is decided by PARSING the NDN name upstream (in-frame filter retired,
    // see firmware/NDR_MAC_SPEC.md): every captured 0x8624 frame is delivered over the
    // serial link (parse-everywhere floor), so nothing is dropped here (FN = 0).
    // [rssi][noise][rate_code][phy_flags][rx_ts_us_le32] — the C5's per-frame PHY
    // metadata layout. The RSSI/rate fields are zero until the RTL8720DN's
    // phy-status path is mapped; the stamp is real now.
    // Decode the per-frame MGN rate code into the C5's (rate_code, bb_format) pair
    // so ONE host parser serves both radios. MGN ≥ 0x80 is HT (MCS = code − 0x80);
    // the CCK codes are 11b; everything else is OFDM. For legacy the MGN code is
    // already the rate in 500 kb/s units — the radiotap convention — so it passes
    // through unchanged and stays meaningful across chip families.
    let (rate_code, bb_format) = if mrate >= 0x80 {
        (mrate - 0x80, 2u8) // HT: rate_code = MCS index
    } else if matches!(mrate, 0x02 | 0x04 | 0x0B | 0x16) {
        (mrate, 0u8) // 11b CCK
    } else {
        (mrate, 1u8) // 11g/a OFDM
    };
    let meta = [
        rssi as u8, // per-frame RSSI (dBm), from the SDK's ieee80211_frame_info_t
        0u8,        // noise floor — not per-frame on this chip
        rate_code,
        bb_format,
        (ts & 0xff) as u8,
        ((ts >> 8) & 0xff) as u8,
        ((ts >> 16) & 0xff) as u8,
        ((ts >> 24) & 0xff) as u8,
    ];
    ring_push(T_RX_TS, &[&meta, frame]);
}

/// Inject at a target instant on our own µs timeline.
///
/// The wait is a bounded spin: precise, and acceptable only because it is capped.
fn inject_at(target: u64, frame: &[u8]) {
    let now = now_us();
    if target > now {
        let delay = target - now;
        if delay > SCHED_MAX_DELAY_US {
            return; // refuse rather than block the loop for an unbounded time
        }
        while now_us() < target {}
    }
    let actual = now_us();
    unsafe { c_wifi_tx_raw_frame(frame.as_ptr(), frame.len() as u32) };
    let mut rep = [0u8; 24];
    rep[0..8].copy_from_slice(&target.to_le_bytes());
    rep[8..16].copy_from_slice(&actual.to_le_bytes());
    // [16..24] = the 802.11 TSF, reported as 0: this radio schedules on the SoC
    // µs timer, not the MAC TSF (which does not run unassociated).
    ring_push(T_TXTIME, &[&rep]);
}

/// Read at most one framed command from the host and act on it, then service the
/// periodic emissions. Called repeatedly by the shim's `loop()`.
#[no_mangle]
pub extern "C" fn rust_loop() {
    static mut RXBUF: [u8; MAX_FRAME] = [0; MAX_FRAME];
    static mut LAST_OCC: u64 = 0;
    static mut LAST_SCAN_CHECK: u64 = 0;

    unsafe {
        // Periodic occupancy, ~5×/s — same cadence as the C5 so one host-side
        // rate calculation serves both radios.
        let now = now_us();
        if now.wrapping_sub(LAST_OCC) >= 200_000 {
            LAST_OCC = now;
            // [wifi_activity_le32] plus, as a 4-byte extension the C5 does not send,
            // [ble_adv_seen_le32]. The host reads only the first word, so the
            // extension is backward compatible.
            let a = ACTIVITY.load(Ordering::Relaxed);
            let bl = BLE_SEEN.load(Ordering::Relaxed);
            ring_push(T_OCC, &[&a.to_le_bytes(), &bl.to_le_bytes()]);
        }
        advq_service(now);
        // Scanner watchdog. A stalled scan is invisible from the host — the device keeps
        // answering commands while receiving nothing — so check it rather than assume it.
        // Costly to call often (it reaches into the BT stack), so once a second.
        if BLE_UP && now.wrapping_sub(LAST_SCAN_CHECK) >= 1_000_000 {
            LAST_SCAN_CHECK = now;
            if c_ble_scanning() == 0 {
                c_ble_scan_restart();
                logmsg(b"ble scan restarted");
            }
        }
        ring_drain(1024);

        if c_serial_available() < 1 {
            return;
        }
        if c_serial_read() != SYNC0 as i32 {
            return;
        }
        while c_serial_available() < 1 {}
        if c_serial_read() != SYNC1 as i32 {
            return;
        }
        while c_serial_available() < 3 {}
        let ty = c_serial_read() as u8;
        let mut len = c_serial_read() as u16;
        len |= (c_serial_read() as u16) << 8;
        let len = len as usize;
        if len > MAX_FRAME {
            return;
        }
        let mut got = 0usize;
        let t0 = c_millis();
        while got < len && c_millis().wrapping_sub(t0) < 200 {
            if c_serial_available() > 0 {
                RXBUF[got] = c_serial_read() as u8;
                got += 1;
            }
        }
        if got != len {
            logmsg(b"short cmd");
            return;
        }
        let cmd = &RXBUF[..len];
        match ty {
            T_INJECT => c_wifi_tx_raw_frame(cmd.as_ptr(), len as u32),
            T_INJECT_ATTR if len >= 1 => {
                let n = cmd[0] as usize;
                let foff = 1 + 2 * n;
                if foff <= len {
                    let pairs = &cmd[1..foff];
                    let frame = &cmd[foff..];
                    c_wifi_tx_raw_frame_attr(
                        frame.as_ptr(),
                        frame.len() as u32,
                        pairs.as_ptr(),
                        n as u32,
                    );
                }
            }
            // [delay_us_le32][frame] — schedule relative to the device's own clock.
            // A delay (not an absolute instant) is what makes this work without any
            // host↔device clock conversion: it is applied on the device timeline.
            T_INJECT_AT if len >= 4 => {
                let d = u32::from_le_bytes([cmd[0], cmd[1], cmd[2], cmd[3]]) as u64;
                inject_at(now_us() + d, &cmd[4..]);
            }
            // [target_us_le64][frame] — an ABSOLUTE instant on the shared timeline
            // (the slot lease). Guarded to [now, now+cap] so a stale target from a
            // queued reply cannot fire immediately or hang the loop.
            T_INJECT_ABS if len >= 8 => {
                let t = u64::from_le_bytes([
                    cmd[0], cmd[1], cmd[2], cmd[3], cmd[4], cmd[5], cmd[6], cmd[7],
                ]);
                let now = now_us();
                if t >= now && t - now <= SCHED_MAX_DELAY_US {
                    inject_at(t, &cmd[8..]);
                }
            }
            // Coarse power (5 levels) — no pointer walk, cannot be stomped by the
            // DM watchdog. Worth having beside the fine knob as the safe fallback.
            T_POWER_PCT if len >= 1 => {
                c_set_tx_power_pct(cmd[0] as u32);
            }
            // [payload] — queue one BLE advertisement (paced; see advq_service).
            T_BLE_ADV if len >= 1 => advq_push(cmd),
            // [scan_window_le16][scan_itvl_le16] in 0.625 ms units — the radio-time
            // split between the two bearers, which cognition drives by measured
            // demand rather than pinning to a constant.
            T_BLE_PACE if len >= 2 => {
                let ms = u16::from_le_bytes([cmd[0], cmd[1]]) as u32;
                if ms > 0 {
                    BLE_BURST_MS.store(ms, Ordering::Relaxed);
                }
                if len >= 6 {
                    let lo = u16::from_le_bytes([cmd[2], cmd[3]]);
                    let hi = u16::from_le_bytes([cmd[4], cmd[5]]);
                    c_ble_adv_interval(lo, hi);
                }
            }
            T_BLE_TXPOWER if len >= 1 => {
                c_ble_set_tx_power(cmd[0]);
            }
            T_BLE_FILTER if len >= 1 => {
                c_ble_scan_filter((cmd[0] != 0) as i32);
            }
            T_BLE_READCAPS => {
                let mut f = [0u8; 8];
                c_ble_features(f.as_mut_ptr());
                ring_push(T_BLE_CAPS, &[&f]);
            }
            T_BLE_HCI if len >= 2 => {
                let op = u16::from_le_bytes([cmd[0], cmd[1]]);
                let n = (len - 2) as u16;
                let st = c_ble_hci_raw(op, RXBUF.as_mut_ptr().add(2), n);
                logmsg(if st == 0 { b"hci ok" } else { b"hci err" });
            }
            T_COEX if len >= 4 => {
                let win = u16::from_le_bytes([cmd[0], cmd[1]]);
                let itvl = u16::from_le_bytes([cmd[2], cmd[3]]);
                c_ble_coex(win, itvl);
            }
            T_READPOWER => {
                let mut idx = [0u8; 20];
                let st = c_get_txagc(idx.as_mut_ptr());
                ring_push(T_POWERIDX, &[&[st as u8], &idx]);
            }
            T_RXINFO_ARM if len >= 1 => {
                RXINFO_ARM.store(cmd[0] as u32, Ordering::Relaxed);
            }
            T_READCLOCK => {
                let t = now_us();
                ring_push(T_CLOCK, &[&t.to_le_bytes()]);
            }
            T_CHANNEL if len >= 1 => c_wifi_set_channel(cmd[0] as i32),
            // [idx] sets every rate's TXAGC index (1 step = 0.25 dB); [0xFF] restores
            // the driver's computed per-rate power. The old `txpower patha=` iwpriv
            // this used to call is a parse-and-echo stub registered under the GET
            // ioctl family — it never wrote a register, which is why sweeping it
            // moved the witness's RSSI by 0.6 dB (i.e. noise).
            T_TXPOWER if len >= 1 => {
                if cmd[0] == 0xFF {
                    c_reset_txagc(if len >= 2 { cmd[1] } else { 6 });
                } else {
                    c_set_txagc(cmd[0] as u32);
                }
            }
            // [mgn_rate] — a Realtek MGN code (0x02=1M, 0x0C=6M, 0x30=24M, 0x6C=54M,
            // 0x80..0x87=HT MCS0..7); 0 restores the driver default. This targets the
            // MANAGEMENT rate (padapter[0x855]) because that is the path our raw
            // inject uses; wifi_set_tx_data_rate writes the DATA-path rate, which is
            // why the old T_RATE actuated nothing on air.
            T_RATE if len >= 1 => c_set_mgnt_rate(cmd[0]),
            T_BW40 if len >= 1 => c_wext_set_bw40((cmd[0] != 0) as u8),
            _ => {}
        }
    }
}

#[panic_handler]
fn panic(_: &PanicInfo) -> ! {
    loop {}
}
