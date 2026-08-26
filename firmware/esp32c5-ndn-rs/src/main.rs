// ESP32-C5 named-radio serial bridge — Rust (esp-idf-svc / std) transcription of
// ../esp32c5-ndn/main/ndn_radio.c. Same FoA primitive (raw 802.11 0x8624 inject + promiscuous capture)
// and the same BW16 wire protocol [0x4E 0x44 ty len_le16 payload] over the native USB-Serial-JTAG:
//   host->device  T_INJECT (a full 802.11 frame -> esp_wifi_80211_tx), T_CHANNEL/T_TXPOWER/T_BW40/T_RATE
//   device->host  T_RX = [rssi_i8][802.11 frame] for each 0x8624 frame heard
// WiFi init boilerplate comes from esp-idf-svc (so we skip the WIFI_INIT_CONFIG_DEFAULT C macro); the
// raw-TX/RX and usb_serial_jtag calls are `sys` FFI. Same host driver: Bw16SerialBackend::open_no_reset.
use std::collections::VecDeque;
use std::sync::Mutex;

use esp_idf_sys as sys;

// WIFI_INIT_CONFIG_DEFAULT() is a C macro, but the ESP32-C5's wifi_init_config_t has NO bitfields (every
// flag is a plain c_int), so we can reproduce it as a Rust struct literal from the esp-idf-sys constants —
// no C shim, no esp-idf component.
fn wifi_init_config_default() -> sys::wifi_init_config_t {
    sys::wifi_init_config_t {
        osi_funcs: &raw mut sys::g_wifi_osi_funcs,
        wpa_crypto_funcs: unsafe { sys::g_wifi_default_wpa_crypto_funcs },
        static_rx_buf_num: sys::CONFIG_ESP_WIFI_STATIC_RX_BUFFER_NUM as i32,
        dynamic_rx_buf_num: sys::CONFIG_ESP_WIFI_DYNAMIC_RX_BUFFER_NUM as i32,
        tx_buf_type: sys::CONFIG_ESP_WIFI_TX_BUFFER_TYPE as i32,
        static_tx_buf_num: sys::WIFI_STATIC_TX_BUFFER_NUM as i32,
        dynamic_tx_buf_num: sys::WIFI_DYNAMIC_TX_BUFFER_NUM as i32,
        rx_mgmt_buf_type: sys::CONFIG_ESP_WIFI_DYNAMIC_RX_MGMT_BUF as i32,
        rx_mgmt_buf_num: sys::WIFI_RX_MGMT_BUF_NUM_DEF as i32,
        cache_tx_buf_num: sys::WIFI_CACHE_TX_BUFFER_NUM as i32,
        csi_enable: sys::WIFI_CSI_ENABLED as i32,
        ampdu_rx_enable: sys::WIFI_AMPDU_RX_ENABLED as i32,
        ampdu_tx_enable: sys::WIFI_AMPDU_TX_ENABLED as i32,
        amsdu_tx_enable: sys::WIFI_AMSDU_TX_ENABLED as i32,
        nvs_enable: sys::WIFI_NVS_ENABLED as i32,
        nano_enable: sys::WIFI_NANO_FORMAT_ENABLED as i32,
        rx_ba_win: sys::WIFI_DEFAULT_RX_BA_WIN as i32,
        wifi_task_core_id: 0,
        beacon_max_len: sys::WIFI_SOFTAP_BEACON_MAX_LEN as i32,
        mgmt_sbuf_num: sys::WIFI_MGMT_SBUF_NUM as i32,
        feature_caps: sys::WIFI_FEATURE_CAPS as u64,
        sta_disconnected_pm: true,
        espnow_max_encrypt_num: sys::CONFIG_ESP_WIFI_ESPNOW_MAX_ENCRYPT_NUM as i32,
        tx_hetb_queue_num: sys::WIFI_TX_HETB_QUEUE_NUM as i32,
        dump_hesigb_enable: false,
        magic: sys::WIFI_INIT_CONFIG_MAGIC as i32,
    }
}

// Blob-internal rate setter (libpp.a). The public esp_wifi_config_80211_tx_rate is just a validator in
// front of this: it refuses (ESP_FAIL) unless the interface phymode byte is ≤ 4 (LR/11B/11G/11A/HT20),
// and the dual-band C5 boots in HE20 (phymode 6) → the wrapper never reaches the setter. We call the
// setter directly, bypassing that gate. Signature confirmed by disassembly: (ifx, wifi_phy_rate_t).
extern "C" {
    fn ic_set_80211_tx_rate(ifx: u32, rate: u32) -> i32;
    // The struct setter behind esp_wifi_config_80211_tx. In the pp descriptor-build, the tx_rate_config's
    // rate OVERRIDES the scalar rate at descriptor offset 12, and its phymode picks the HE vs legacy TX
    // path — so the config, with an explicit legacy phymode, is the actual lever for raw injection.
    fn ic_set_80211_tx_rate_config(ifx: u32, cfg: *const sys::wifi_tx_rate_config_t) -> i32;
}

/// Last channel set via T_CHANNEL (for deriving the OFDM phymode: 2.4 GHz→11G, 5 GHz→11A).
static CUR_CHAN: std::sync::atomic::AtomicU8 = std::sync::atomic::AtomicU8::new(1);

/// The phymode a `wifi_phy_rate_t` implies. CCK codes (<0x08) → 11B; OFDM (0x08..0x10) → 11G on 2.4 GHz,
/// 11A on 5 GHz; MCS (≥0x10) → HT20. This is what the pp descriptor-build needs to pick the legacy/HT TX
/// path instead of the C5's default HE20 (which forces the basic rate for raw injection).
fn phymode_for_rate(rate: u8, chan: u8) -> sys::wifi_phy_mode_t {
    if rate >= 0x10 {
        sys::wifi_phy_mode_t_WIFI_PHY_MODE_HT20
    } else if rate >= 0x08 {
        if chan <= 14 {
            sys::wifi_phy_mode_t_WIFI_PHY_MODE_11G
        } else {
            sys::wifi_phy_mode_t_WIFI_PHY_MODE_11A
        }
    } else {
        sys::wifi_phy_mode_t_WIFI_PHY_MODE_11B
    }
}

/// Fix the raw-injection TX rate by calling the blob's internal config setter directly (the public
/// esp_wifi_config_80211_tx wrapper ESP_FAILs because the C5 boots in HE20). `phymode==0` derives it.
unsafe fn set_fix_rate(rate: u8, phymode: u8) {
    let pm = if phymode == 0 {
        phymode_for_rate(rate, CUR_CHAN.load(std::sync::atomic::Ordering::Relaxed))
    } else {
        phymode as sys::wifi_phy_mode_t
    };
    ic_set_80211_tx_rate(sys::wifi_interface_t_WIFI_IF_STA as u32, rate as u32);
    let cfg = sys::wifi_tx_rate_config_t { phymode: pm, rate: rate as sys::wifi_phy_rate_t, ersu: false, dcm: false };
    ic_set_80211_tx_rate_config(sys::wifi_interface_t_WIFI_IF_STA as u32, &cfg);
}

const NDN_ETHERTYPE: u16 = 0x8624;
const SYNC0: u8 = 0x4E;
const SYNC1: u8 = 0x44;
const T_INJECT: u8 = 0x01;
const T_CHANNEL: u8 = 0x02;
const T_TXPOWER: u8 = 0x03;
const T_RATE: u8 = 0x04;
const T_BW40: u8 = 0x05;
const T_INJECT_ATTR: u8 = 0x06;
const T_NAMEFILTER: u8 = 0x07; // [enabled][n_masks][mask 16B]* — on-device Tier-0 prefix-set drop
const T_INJECT_AT: u8 = 0x09; // [delay_us_le32][802.11 frame] — scheduled TX, delay from now
const T_INJECT_ABS: u8 = 0x0A; // [target_us_le64][802.11 frame] — scheduled TX at an ABSOLUTE esp_timer µs (slot lease)
const T_READCLOCK: u8 = 0x0B; // no payload — reply T_CLOCK with the current esp_timer (schedule clock)
const T_RX_TS: u8 = 0x82; // [rssi_i8][noise_i8][rate_code u8][phy_flags u8][rx_ts_us_le32][802.11 frame]
                          // — RX + hardware µs stamp + the ESP's per-frame PHY metadata (radiotap-equiv)
const T_TXTIME: u8 = 0x83; // [target_le64][actual_le64][tsf_le64] — scheduling error report
const T_CLOCK: u8 = 0x85; // [esp_timer_us_le64] — reply to T_READCLOCK
const T_OCC: u8 = 0x86; // [activity_count_le32] — periodic free-running channel-activity counter (occupancy)
const MAXFRAME: usize = 512;
const MAX_MASKS: usize = 8;
const SCHED_MAX_DELAY_US: i64 = 100_000; // cap the busy-wait to ~1 slot period (a hw timer would avoid the spin)

// Reuse the LR2021 firmware's Tier-0 filter VERBATIM (no_std, dependency-free, golden-vector-pinned —
// the very file the host tier0.rs was ported from). Both C5 firmwares share EXISTING pinned copies: the
// C build compiles the AR9271's ndr_tier0.c, this one includes the LR2021's tier0.rs — no new copy to
// drift. Only PrefixFilter::may_match is used on the RX side (the host computes masks), so the hashing
// functions are unused here (hence allow(dead_code)).
#[allow(dead_code)]
#[path = "../../lr2021-nrf54l15-rs/src/tier0.rs"]
mod tier0;

// Tier-0 name-filter state. Host-computed masks; a frame whose in-address prefix-set matches none is
// dropped in rx_cb before it crosses the USB-Serial-JTAG (the pre-USB drop, real because this fw is ours).
struct NameFilter {
    enabled: bool,
    n: usize,
    masks: [[u8; 16]; MAX_MASKS],
}
static NF: Mutex<NameFilter> = Mutex::new(NameFilter { enabled: false, n: 0, masks: [[0u8; 16]; MAX_MASKS] });


// RX ring shared between the promiscuous callback (WiFi-task context) and the serial-TX loop (main).
// (rssi, noise_floor, rate_code, phy_flags, rx_ts_us, frame) — the ESP's per-frame PHY metadata (its
// "radiotap"): rate_code is the legacy rate (sig_mode 0) or the MCS (HT/VHT); phy_flags packs
// sig_mode(bits0-1) | sgi(bit2) | cwb40(bit3). ts is the hardware per-frame RX stamp (µs, esp_timer domain).
static RXQ: Mutex<VecDeque<(i8, i8, u8, u8, u32, Vec<u8>)>> = Mutex::new(VecDeque::new());

// Free-running channel-activity counter: every promiscuous frame the radio decodes, of ANY type — the
// frame-free occupancy proxy the host reads via T_OCC (RadioKnobs::read_channel_activity). Two reads
// differenced over a window give a frames/s rate → channel-busy%.
static ACTIVITY: std::sync::atomic::AtomicU32 = std::sync::atomic::AtomicU32::new(0);

// Promiscuous RX: queue each 0x8624 data frame (+ RSSI) for the serial-TX loop.
unsafe extern "C" fn rx_cb(buf: *mut core::ffi::c_void, ty: sys::wifi_promiscuous_pkt_type_t) {
    ACTIVITY.fetch_add(1, std::sync::atomic::Ordering::Relaxed); // count ALL activity, before any filter
    if ty != sys::wifi_promiscuous_pkt_type_t_WIFI_PKT_DATA {
        return;
    }
    let p = buf as *const sys::wifi_promiscuous_pkt_t;
    let len = (*p).rx_ctrl.sig_len() as usize;
    if len < 32 || len > MAXFRAME {
        return;
    }
    let f = (*p).payload.as_ptr();
    let b = core::slice::from_raw_parts(f, len);
    // LLC/SNAP aa aa 03 at [24..27] and ethertype 0x8624 at [30..32].
    if !(b[24] == 0xaa && b[25] == 0xaa && b[26] == 0x03) {
        return;
    }
    if (((b[30] as u16) << 8) | b[31] as u16) != NDN_ETHERTYPE {
        return;
    }
    // Tier-0: drop off-prefix frames HERE, before they cross the serial link.
    if let Ok(nf) = NF.lock() {
        if nf.enabled && nf.n > 0 {
            let mut fb = [0u8; 16];
            fb.copy_from_slice(&b[4..20]); // addr1‖addr2‖addr3[0..4]
            let frame = tier0::PrefixFilter(fb);
            if !(0..nf.n).any(|i| frame.may_match(&tier0::PrefixFilter(nf.masks[i]))) {
                return;
            }
        }
    }
    // The C5's HE-MAC rxctrl (MAC v3) is its "radiotap": rssi, noise_floor, the L-SIG `rate`, the PHY
    // format `cur_bb_format` (0=11B 1=11G/A 2=HT 3=VHT 4+=HE), and `he_siga1` (= HT-SIG / VHT-SIG / HE-SIGA
    // — the MCS source). For an HT frame the HT-SIG MCS is the low 7 bits of he_siga1; legacy uses `rate`.
    let rssi = (*p).rx_ctrl.rssi() as i8;
    let noise = (*p).rx_ctrl.noise_floor() as i8; // → host computes SNR = rssi - noise_floor
    let fmt = (*p).rx_ctrl.cur_bb_format() as u8; // RX_BB_FORMAT_*
    let rate_code = if fmt >= 2 {
        ((*p).rx_ctrl.he_siga1 & 0x7f) as u8 // HT-SIG/VHT-SIG/HE-SIGA MCS (HT: low 7 bits)
    } else {
        (*p).rx_ctrl.rate() as u8 // legacy: L-SIG rate code
    };
    let flags = fmt & 0x0f; // carry the PHY format so the host knows MCS-vs-legacy
    let ts_us = (*p).rx_ctrl.timestamp(); // hardware per-frame RX stamp (µs, same domain as esp_timer)
    if let Ok(mut q) = RXQ.lock() {
        if q.len() < 32 {
            q.push_back((rssi, noise, rate_code, flags, ts_us, b.to_vec()));
        }
    }
}

fn send_framed(ty: u8, payload: &[u8]) {
    let len = payload.len() as u16;
    let hdr = [SYNC0, SYNC1, ty, (len & 0xff) as u8, (len >> 8) as u8];
    unsafe {
        sys::usb_serial_jtag_write_bytes(hdr.as_ptr() as *const _, hdr.len(), u32::MAX);
        if len > 0 {
            sys::usb_serial_jtag_write_bytes(payload.as_ptr() as *const _, payload.len(), u32::MAX);
        }
    }
}

// Read the host protocol -> dispatch inject / channel / power / bw40 / rate.
fn serial_rx_loop() -> ! {
    let mut acc = vec![0u8; 2048];
    let mut n = 0usize;
    loop {
        let space = (acc.len() - n) as u32;
        // Block until the host sends (usb_serial_jtag_read_bytes returns as soon as any byte is available).
        let got = unsafe {
            sys::usb_serial_jtag_read_bytes(acc.as_mut_ptr().add(n) as *mut _, space, u32::MAX)
        };
        if got > 0 {
            n += got as usize;
        }
        let mut i = 0usize;
        while n - i >= 5 {
            if acc[i] != SYNC0 || acc[i + 1] != SYNC1 {
                i += 1;
                continue;
            }
            let ty = acc[i + 2];
            let len = (acc[i + 3] as usize) | ((acc[i + 4] as usize) << 8);
            if n - i < 5 + len {
                break;
            }
            let pl = &acc[i + 5..i + 5 + len];
            unsafe {
                match ty {
                    T_INJECT => {
                        sys::esp_wifi_80211_tx(
                            sys::wifi_interface_t_WIFI_IF_STA,
                            pl.as_ptr() as *const _,
                            len as i32,
                            true,
                        );
                    }
                    T_CHANNEL if len >= 1 => {
                        sys::esp_wifi_set_channel(pl[0], sys::wifi_second_chan_t_WIFI_SECOND_CHAN_NONE);
                        CUR_CHAN.store(pl[0], std::sync::atomic::Ordering::Relaxed);
                    }
                    T_TXPOWER if len >= 1 => {
                        sys::esp_wifi_set_max_tx_power(pl[0] as i8);
                    }
                    T_BW40 if len >= 1 => {
                        let bw = if pl[0] != 0 {
                            sys::wifi_bandwidth_t_WIFI_BW_HT40
                        } else {
                            sys::wifi_bandwidth_t_WIFI_BW_HT20
                        };
                        sys::esp_wifi_set_bandwidth(sys::wifi_interface_t_WIFI_IF_STA, bw);
                    }
                    T_RATE if len >= 2 => {
                        // [rate][phymode] — explicit phymode override (1=11B 2=11G 3=11A 4=HT20).
                        set_fix_rate(pl[0], pl[1]);
                    }
                    T_RATE if len >= 1 => {
                        // [rate] — phymode auto-derived from the rate code + current band. rate is a
                        // wifi_phy_rate_t: 0x00=1M, 0x09=24M, 0x0C=54M, 0x10=MCS0, 0x17=MCS7. On air ✔.
                        set_fix_rate(pl[0], 0);
                    }
                    T_INJECT_ATTR if len >= 1 => {
                        let np = pl[0] as usize;
                        let off = 1 + 2 * np;
                        if len > off {
                            sys::esp_wifi_80211_tx(
                                sys::wifi_interface_t_WIFI_IF_STA,
                                pl.as_ptr().add(off) as *const _,
                                (len - off) as i32,
                                true,
                            );
                        }
                    }
                    T_INJECT_AT if len >= 4 => {
                        // [delay_us_le32][frame] — place TX at a precise instant on esp_timer (the 802.11
                        // TSF reads 0 while unassociated). Report [target][actual][tsf] for the error.
                        let mut delay = (pl[0] as i64) | ((pl[1] as i64) << 8) | ((pl[2] as i64) << 16) | ((pl[3] as i64) << 24);
                        if delay > SCHED_MAX_DELAY_US { delay = SCHED_MAX_DELAY_US; }
                        let target = sys::esp_timer_get_time() + delay;
                        while sys::esp_timer_get_time() < target {} // spin to the scheduled instant
                        sys::esp_wifi_80211_tx(sys::wifi_interface_t_WIFI_IF_STA, pl.as_ptr().add(4) as *const _, (len - 4) as i32, true);
                        let actual = sys::esp_timer_get_time();
                        let tsf = sys::esp_wifi_get_tsf_time(sys::wifi_interface_t_WIFI_IF_STA);
                        let mut rep = [0u8; 24];
                        rep[0..8].copy_from_slice(&target.to_le_bytes());
                        rep[8..16].copy_from_slice(&actual.to_le_bytes());
                        rep[16..24].copy_from_slice(&tsf.to_le_bytes());
                        send_framed(T_TXTIME, &rep);
                    }
                    T_INJECT_ABS if len >= 8 => {
                        // [target_us_le64][frame] — place TX at an ABSOLUTE esp_timer µs, so the frame lands
                        // at a common-clock instant regardless of when the command arrived (the slot lease).
                        let target = i64::from_le_bytes([pl[0], pl[1], pl[2], pl[3], pl[4], pl[5], pl[6], pl[7]]);
                        let now = sys::esp_timer_get_time();
                        // Guard: only honour a target within [now, now + cap] — a stale/past or far-future
                        // target is dropped rather than blocking the dispatch or firing late.
                        if target > now && target - now <= SCHED_MAX_DELAY_US {
                            while sys::esp_timer_get_time() < target {}
                            sys::esp_wifi_80211_tx(sys::wifi_interface_t_WIFI_IF_STA, pl.as_ptr().add(8) as *const _, (len - 8) as i32, true);
                            let actual = sys::esp_timer_get_time();
                            let mut rep = [0u8; 24];
                            rep[0..8].copy_from_slice(&target.to_le_bytes());
                            rep[8..16].copy_from_slice(&actual.to_le_bytes());
                            rep[16..24].copy_from_slice(&sys::esp_wifi_get_tsf_time(sys::wifi_interface_t_WIFI_IF_STA).to_le_bytes());
                            send_framed(T_TXTIME, &rep);
                        }
                    }
                    T_READCLOCK => {
                        // Reply with the current esp_timer (the clock T_INJECT_ABS schedules against), so
                        // the host can place absolute slot targets and align two devices' clocks.
                        send_framed(T_CLOCK, &sys::esp_timer_get_time().to_le_bytes());
                    }
                    T_NAMEFILTER if len >= 2 => {
                        // [enabled][n_masks][mask 16B]* — load host-computed Tier-0 masks.
                        if let Ok(mut nf) = NF.lock() {
                            let nm = (pl[1] as usize).min(MAX_MASKS);
                            if len >= 2 + nm * 16 {
                                for m in 0..nm {
                                    nf.masks[m].copy_from_slice(&pl[2 + m * 16..2 + m * 16 + 16]);
                                }
                                nf.n = nm;
                                nf.enabled = pl[0] != 0;
                            }
                        }
                    }
                    _ => {}
                }
            }
            i += 5 + len;
        }
        if i > 0 {
            acc.copy_within(i..n, 0);
            n -= i;
        }
        if n > acc.len() - 64 {
            n = 0; // desync guard
        }
    }
}

fn main() {
    sys::link_patches();

    unsafe {
        // nvs + event loop + WiFi init (RAM storage, STA, started but unassociated) — the app_main() of
        // the C build, now in Rust. Config from wifi_init_config_default() (the WIFI_INIT_CONFIG_DEFAULT macro).
        sys::nvs_flash_init();
        sys::esp_event_loop_create_default();
        let cfg = wifi_init_config_default();
        sys::esp_wifi_init(&cfg);
        sys::esp_wifi_set_storage(sys::wifi_storage_t_WIFI_STORAGE_RAM);
        sys::esp_wifi_set_mode(sys::wifi_mode_t_WIFI_MODE_STA);
        // TX RATE — actuated. The public rate APIs (esp_wifi_config_80211_tx_rate / _tx) all ESP_FAIL on the
        // dual-band C5 because it boots in HE20 (phymode 6) and their wrapper refuses to set a rate in HE
        // mode. But those wrappers are only validators in front of the blob-internal ic_set_80211_tx_rate_
        // config, which we call directly (see set_fix_rate) with an explicit legacy/HT phymode. MEASURED on
        // air vs an mt76: 11G/24M→24, 11G/54M→54, HT20/MCS7→65 (MCS7), HT20/MCS4→39 (MCS4). Driven per-frame
        // via T_RATE; the host maps cognition's rate lever onto it. Default below = 6 Mbps OFDM (robust).
        sys::esp_wifi_start();
        sys::esp_wifi_set_channel(1, sys::wifi_second_chan_t_WIFI_SECOND_CHAN_NONE);
        set_fix_rate(sys::wifi_phy_rate_t_WIFI_PHY_RATE_6M as u8, 0); // OFDM 6M — beats the 1 Mbps basic default
        sys::esp_wifi_set_promiscuous_rx_cb(Some(rx_cb));
        sys::esp_wifi_set_promiscuous(true);

        // Own the USB-Serial-JTAG for the binary protocol.
        let mut ucfg = sys::usb_serial_jtag_driver_config_t {
            tx_buffer_size: 2048,
            rx_buffer_size: 2048,
        };
        sys::usb_serial_jtag_driver_install(&mut ucfg);
    }

    // Serial-RX dispatch on its own thread; drain RX -> T_RX on main.
    std::thread::Builder::new()
        .stack_size(8192)
        .spawn(serial_rx_loop)
        .expect("spawn serial_rx_loop");

    let mut out = Vec::with_capacity(5 + MAXFRAME);
    let mut last_occ = unsafe { sys::esp_timer_get_time() };
    loop {
        let item = { RXQ.lock().ok().and_then(|mut q| q.pop_front()) };
        match item {
            Some((rssi, noise, rate_code, flags, ts_us, frame)) => {
                out.clear();
                out.push(rssi as u8);
                out.push(noise as u8); // noise floor (dBm) → SNR = rssi - noise
                out.push(rate_code); // legacy rate or MCS (per flags sig_mode)
                out.push(flags); // sig_mode(0-1) | sgi(2) | cwb40(3)
                out.extend_from_slice(&ts_us.to_le_bytes()); // hardware per-frame RX stamp (µs)
                out.extend_from_slice(&frame);
                send_framed(T_RX_TS, &out);
            }
            None => std::thread::sleep(std::time::Duration::from_millis(2)),
        }
        // Emit the free-running activity counter ~5×/s so the host's read_channel_activity has a fresh value.
        let now = unsafe { sys::esp_timer_get_time() };
        if now - last_occ >= 200_000 {
            last_occ = now;
            let c = ACTIVITY.load(std::sync::atomic::Ordering::Relaxed);
            send_framed(T_OCC, &c.to_le_bytes());
        }
    }
}
