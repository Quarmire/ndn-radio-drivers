// ESP32-C5 named-data radio node — host-driven serial-bridge FrameIo (BW16 wire protocol).
//
// The C5 does the FoA primitive (raw 802.11 inject + promiscuous capture of ethertype 0x8624) and is
// driven over its native USB-Serial-JTAG by the host's `Bw16SerialBackend` UNCHANGED — same framing
// `[0x4E 0x44 type len_le16 payload]`: host→device T_INJECT (a full 802.11 frame to esp_wifi_80211_tx),
// T_CHANNEL/T_RATE/T_TXPOWER/T_BW40 (1-byte params); T_NAMEFILTER loads the on-device Tier-0 masks;
// device→host T_RX = [rssi_i8][802.11 frame] for each 0x8624 frame that PASSES the filter. Console
// logging is OFF (LOG level NONE) so the binary stream is clean, while the console stays bound to the
// USB-Serial-JTAG so esptool auto-reset keeps working (see sdkconfig.defaults).
//
// Tier-0 name filter (§8.2): a frame whose in-address prefix-set Bloom filter matches no registered mask
// is dropped HERE, before it crosses the USB-Serial-JTAG — the C5 is the second Wi-Fi part (after the
// AR9271) whose firmware is ours, so the paper's pre-USB drop is actually reachable. The filter math is
// the AR9271's ndr_tier0.c reused verbatim (golden-vector-pinned; no fourth copy). MEASURED on air: with
// a /ndn/alarm mask, matching frames 306→admitted, non-matching 0 (filter on) vs 295 (filter off).
//
// IMPORTANT (host side): the C5's native USB-Serial-JTAG maps RTS→EN and DTR→GPIO9. A host that asserts
// RTS on open holds the chip in reset (silent, no TX); one that pulses DTR can latch the download strap.
// Drive it with `Bw16SerialBackend::open_no_reset` — never toggle RTS/DTR; the chip free-runs the app.
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_wifi.h"
#include "esp_private/wifi.h"  // esp_wifi_internal_set_fix_rate — pin the injected-frame PHY rate
#include "esp_event.h"
#include "nvs_flash.h"
#include "driver/usb_serial_jtag.h"
#include "esp_timer.h" // esp_timer_get_time — the always-running monotonic µs clock we schedule against
#include "ndr_tier0.h" // the AR9271 firmware's Tier-0 filter, reused verbatim (golden-vector-pinned)
// ── BLE bearer (NimBLE) — one firmware, all bearers (the named radio is bearer-agnostic) ──
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/ble_hs.h"
#include "host/util/util.h"
#include "host/ble_gap.h"
#include "os/os_mbuf.h"

#define NDN_ETHERTYPE 0x8624
#define SYNC0 0x4E
#define SYNC1 0x44
#define T_INJECT     0x01
#define T_CHANNEL    0x02
#define T_TXPOWER    0x03
#define T_RATE       0x04
#define T_BW40       0x05
#define T_INJECT_ATTR 0x06
#define T_NAMEFILTER 0x07 // [enabled][n_masks][mask 16B]* — on-device Tier-0 prefix-set drop (§8.2)
#define T_INJECT_AT  0x09 // [delay_us_le32][802.11 frame] — scheduled TX, delay from now
#define T_INJECT_ABS 0x0A // [target_us_le64][802.11 frame] — scheduled TX at an ABSOLUTE esp_timer µs (slot lease)
#define T_READCLOCK  0x0B // no payload — reply T_CLOCK with the current esp_timer (schedule clock)
#define T_CLOCK      0x85 // [esp_timer_us_le64] — reply to T_READCLOCK
#define T_OCC        0x86 // [activity_count_le32] — periodic free-running channel-activity counter (occupancy)
#define T_RX         0x81 // [rssi_i8][802.11 frame] — used by the BW16; the C5 sends T_RX_TS instead
#define T_RX_TS      0x82 // [rssi_i8][noise_i8][rate_code][phy_flags][rx_ts_us_le32][802.11 frame]
                          // — RX + hardware µs stamp + per-frame PHY metadata (radiotap-equiv)
#define T_TXTIME     0x83 // [target_le64][actual_le64][tsf_le64] — scheduling error report for T_INJECT_AT
// BLE bearer messages (same wire protocol, distinct types so one firmware serves Wi-Fi + BLE at once):
#define T_BLE_ADV    0x30 // host→device: [payload] — advertise as a BLE 5 extended advertisement
#define T_COEX       0x31 // host→device: [scan_window_le16][scan_itvl_le16] — the BLE↔Wi-Fi radio-time split
#define T_BLE_RX     0x88 // device→host: [rssi_i8][addr6][payload] — a scanned advertisement carrying our magic
#define MAXFRAME 512
#define BLE_MAXPAY 240
#define BLE_ADV_MAGIC_LO 0x44 // manufacturer company id 0x4E44 ("ND"), LE on air — the BLE analog of 0x8624
#define BLE_ADV_MAGIC_HI 0x4E

// Blob-internal rate setters (libpp.a). The public esp_wifi_config_80211_tx[_rate] wrappers ESP_FAIL on
// the dual-band C5 because it boots in HE20 (phymode 6) and they refuse to set a rate in HE mode; these
// underlying setters, called with an explicit legacy/HT phymode, actuate the raw-injection rate. The
// tx_rate_config's rate+phymode reach the descriptor (offset 12 + the HE-vs-legacy branch) in the pp
// TX-build path. MEASURED on air: 11G/24M->24, 11G/54M->54, HT20/MCS7->65, HT20/MCS4->39.
extern int ic_set_80211_tx_rate(uint32_t ifx, uint32_t rate);
extern int ic_set_80211_tx_rate_config(uint32_t ifx, const wifi_tx_rate_config_t *cfg);

static uint8_t s_cur_chan = 1; // last T_CHANNEL, for deriving the OFDM phymode (2.4G->11G, 5G->11A)

static wifi_phy_mode_t phymode_for_rate(uint8_t rate, uint8_t chan) {
    if (rate >= 0x10) return WIFI_PHY_MODE_HT20;                 // MCS
    if (rate >= 0x08) return chan <= 14 ? WIFI_PHY_MODE_11G : WIFI_PHY_MODE_11A; // OFDM
    return WIFI_PHY_MODE_11B;                                    // CCK
}

// Fix the raw-injection TX rate. phymode==0 => auto-derive from the rate code + current band. dcm/ersu are
// the 802.11ax reach levers (HE Dual-Carrier Modulation, HE Extended-Range SU); only apply for phymode HE20.
static void set_fix_rate(uint8_t rate, uint8_t phymode, bool dcm, bool ersu) {
    wifi_phy_mode_t pm = phymode ? (wifi_phy_mode_t)phymode : phymode_for_rate(rate, s_cur_chan);
    ic_set_80211_tx_rate(WIFI_IF_STA, rate);
    wifi_tx_rate_config_t cfg = { .phymode = pm, .rate = (wifi_phy_rate_t)rate, .ersu = ersu, .dcm = dcm };
    ic_set_80211_tx_rate_config(WIFI_IF_STA, &cfg);
}
#define MAX_MASKS 8
#define SCHED_MAX_DELAY_US 100000 // cap the busy-wait to ~1 slot period (a hw timer would avoid the spin)

typedef struct { int8_t rssi; int8_t noise; uint8_t rate_code; uint8_t flags; uint16_t len; uint32_t rxts; uint8_t buf[MAXFRAME]; } rxpkt_t;
static QueueHandle_t rxq;

// Tier-0 name-filter state. Host-computed masks (cognition derives them via the shared tier0.rs); the
// C5 only tests them — a frame whose in-address prefix-set filter matches no registered mask is dropped
// HERE, before it ever crosses the USB-Serial-JTAG. The win only exists because this firmware is ours
// (like the AR9271; see ndr_tier0.h). Written by serial_rx_loop, read in rx_cb (WiFi-task ctx): a torn
// read during a rare reconfig at worst mis-filters one best-effort frame, so no lock.
static volatile uint8_t nf_enabled = 0;
static volatile uint8_t nf_n_masks = 0;
static ndr_filter_t nf_masks[MAX_MASKS];

// True if the frame passes the filter (filter off, or its prefix-set matches a registered mask).
static int name_admits(const uint8_t *f) {
    if (!nf_enabled || nf_n_masks == 0) return 1; // off / no masks → forward all (backward compatible)
    ndr_filter_t got;
    ndr_filter_from_hdr(&got, f); // lifts addr1‖addr2‖addr3[0..4] from offset 4
    for (uint8_t i = 0; i < nf_n_masks; i++) {
        if (ndr_may_match(&got, &nf_masks[i])) return 1; // any registered prefix could hold this name
    }
    return 0; // definitely under none of our prefixes — drop, never cross USB
}

// Free-running channel-activity counter (every promiscuous frame of any type) — the occupancy proxy the
// host reads via T_OCC (read_channel_activity). Written in WiFi-task ctx, read in serial_tx_task.
static volatile uint32_t s_activity = 0;

// Promiscuous RX: queue each 0x8624 frame (+ RSSI) for the serial-TX task.
static void rx_cb(void *buf, wifi_promiscuous_pkt_type_t type) {
    s_activity++; // count ALL activity, before any filter
    if (type != WIFI_PKT_DATA) return;
    const wifi_promiscuous_pkt_t *p = (wifi_promiscuous_pkt_t *)buf;
    const uint8_t *f = p->payload;
    int len = p->rx_ctrl.sig_len;
    if (len < 32 || len > MAXFRAME) return;
    if (!(f[24] == 0xaa && f[25] == 0xaa && f[26] == 0x03)) return;
    if ((((uint16_t)f[30] << 8) | f[31]) != NDN_ETHERTYPE) return;
    if (!name_admits(f)) return; // Tier-0: drop off-prefix frames on the dongle, pre-USB
    rxpkt_t pk;
    pk.rssi = p->rx_ctrl.rssi;
    pk.noise = p->rx_ctrl.noise_floor;              // → host SNR = rssi - noise
    uint8_t fmt = p->rx_ctrl.cur_bb_format;         // 0=11B 1=11G/A 2=HT 3=VHT 4+=HE
    // HT-SIG/VHT-SIG/HE-SIGA MCS (HT: low 7 bits of he_siga1) for a coded frame; else the L-SIG rate.
    pk.rate_code = fmt >= 2 ? (uint8_t)(p->rx_ctrl.he_siga1 & 0x7f) : (uint8_t)p->rx_ctrl.rate;
    pk.flags = fmt & 0x0f;                          // carry the PHY format (MCS-vs-legacy) to the host
    pk.len = len;
    pk.rxts = p->rx_ctrl.timestamp;   // hardware per-frame RX stamp (µs, same domain as esp_timer)
    memcpy(pk.buf, f, len);
    BaseType_t hp = pdFALSE;
    xQueueSendFromISR(rxq, &pk, &hp); // drops if full — fine, promiscuous is best-effort
    if (hp) portYIELD_FROM_ISR();
}

static void send_framed(uint8_t ty, const uint8_t *payload, uint16_t len) {
    uint8_t hdr[5] = { SYNC0, SYNC1, ty, (uint8_t)(len & 0xff), (uint8_t)(len >> 8) };
    usb_serial_jtag_write_bytes(hdr, 5, portMAX_DELAY);
    if (len) usb_serial_jtag_write_bytes(payload, len, portMAX_DELAY);
}

// Drain the RX queue → serial as T_RX_TS [rssi][rx_ts_us_le32][frame]: every C5 frame carries the
// hardware per-frame RX timestamp, so the host can stamp it in the device's clock domain (common-view /
// frame-age) instead of the coarse host-recv time. The 802.11 TSF is 0 unassociated, so this µs stamp
// (rx_ctrl.timestamp, same domain as the esp_timer we schedule TX on) is the C5's real link clock.
// ── BLE bearer (NimBLE ext-adv + scan) — the second bearer of the one unified firmware ──
static uint8_t s_ble_addr_type;
static uint8_t s_ble_addr[6];
static QueueHandle_t bleq;          // scanned reports -> serial-TX task
static volatile bool s_ble_ready;
typedef struct { int8_t rssi; uint8_t addr[6]; uint8_t len; uint8_t buf[BLE_MAXPAY]; } blerep_t;

// Scan an adv payload for our manufacturer AD (0xFF, company 0x4E44); return the inner named payload len.
static int ble_find_named(const uint8_t *d, int n, const uint8_t **out) {
    int i = 0;
    while (i + 2 <= n) {
        int adlen = d[i];
        if (adlen < 1 || i + 1 + adlen > n) break;
        if (d[i + 1] == 0xFF && adlen >= 3 && d[i + 2] == BLE_ADV_MAGIC_LO && d[i + 3] == BLE_ADV_MAGIC_HI) {
            *out = &d[i + 4];
            return adlen - 3;
        }
        i += 1 + adlen;
    }
    return 0;
}

static int ble_gap_event(struct ble_gap_event *ev, void *arg) {
    if (ev->type == BLE_GAP_EVENT_EXT_DISC) {
        const struct ble_gap_ext_disc_desc *dsc = &ev->ext_disc;
        if (memcmp(dsc->addr.val, s_ble_addr, 6) == 0) return 0; // ignore our own reflected ads
        const uint8_t *pay = NULL;
        int plen = ble_find_named(dsc->data, dsc->length_data, &pay);
        if (plen > 0 && plen <= BLE_MAXPAY) {
            blerep_t r;
            r.rssi = dsc->rssi;
            memcpy(r.addr, dsc->addr.val, 6);
            r.len = (uint8_t)plen;
            memcpy(r.buf, pay, plen);
            BaseType_t hp = pdFALSE;
            xQueueSendFromISR(bleq, &r, &hp);
            if (hp) portYIELD_FROM_ISR();
        }
    }
    return 0;
}

// T_BLE_ADV: wrap the host payload in our manufacturer AD and burst-advertise it (fire-and-forget).
static void ble_advertise(const uint8_t *payload, int len) {
    if (!s_ble_ready || len <= 0 || len > BLE_MAXPAY) return;
    struct os_mbuf *m = os_msys_get_pkthdr(len + 4, 0);
    if (!m) return;
    uint8_t hdr[4] = { (uint8_t)(len + 3), 0xFF, BLE_ADV_MAGIC_LO, BLE_ADV_MAGIC_HI };
    os_mbuf_append(m, hdr, 4);
    os_mbuf_append(m, payload, len);
    ble_gap_ext_adv_stop(0);
    if (ble_gap_ext_adv_set_data(0, m) == 0) ble_gap_ext_adv_start(0, 0, 3);
    else os_mbuf_free_chain(m);
}

// BLE↔Wi-Fi radio-time split, as a scan window/interval. This is NOT a fixed MAC parameter — it is the
// airtime allocation between the two bearers of the one radio, which the NDR MAC allocates by measured
// demand (host/cognition drives it via T_COEX). The boot values are only a **fallback** so the radio comes
// up balanced before cognition speaks; a fully-open scan (window==itvl) starves the promiscuous Wi-Fi RX,
// so the fallback duty-cycles (~12%). Both bearers report activity (T_OCC = Wi-Fi frames; BLE hit rate is
// observable at the host) so the split can track which bearer actually has named traffic.
static uint16_t s_ble_scan_win = 0x20;   // fallback: 20ms window …
static uint16_t s_ble_scan_itvl = 0x100; // … / 160ms interval

static void ble_start_scan(void) {
    ble_gap_disc_cancel(); // no-op if not scanning
    struct ble_gap_ext_disc_params up;
    memset(&up, 0, sizeof(up));
    up.itvl = s_ble_scan_itvl;
    up.window = s_ble_scan_win;
    up.passive = 1;
    ble_gap_ext_disc(s_ble_addr_type, 0, 0, 0, 0, 0, &up, NULL, ble_gap_event, NULL);
}

static void ble_on_sync(void) {
    ble_hs_util_ensure_addr(0);
    ble_hs_id_infer_auto(0, &s_ble_addr_type);
    ble_hs_id_copy_addr(s_ble_addr_type, s_ble_addr, NULL);
    struct ble_gap_ext_adv_params p;
    memset(&p, 0, sizeof(p));
    p.own_addr_type = s_ble_addr_type; // non-connectable, non-scannable, extended PDU
    p.primary_phy = BLE_HCI_LE_PHY_1M;
    p.secondary_phy = BLE_HCI_LE_PHY_2M;
    p.itvl_min = 0x30;
    p.itvl_max = 0x30;
    p.tx_power = 127;
    int8_t sel;
    ble_gap_ext_adv_configure(0, &p, &sel, ble_gap_event, NULL);
    ble_start_scan();
    s_ble_ready = true;
}

static void ble_host_task(void *param) {
    nimble_port_run();
    nimble_port_freertos_deinit();
}

static void serial_tx_task(void *arg) {
    static uint8_t out[8 + MAXFRAME];
    static uint8_t bout[7 + BLE_MAXPAY];
    rxpkt_t pk;
    blerep_t br;
    int64_t last_occ = esp_timer_get_time();
    for (;;) {
        // 100 ms timeout so the loop wakes to emit T_OCC / drain BLE even when no Wi-Fi frames are queued.
        if (xQueueReceive(rxq, &pk, pdMS_TO_TICKS(100)) == pdTRUE) {
            out[0] = (uint8_t)pk.rssi;
            out[1] = (uint8_t)pk.noise;      // noise floor (dBm)
            out[2] = pk.rate_code;           // legacy rate or MCS (per flags sig_mode)
            out[3] = pk.flags;               // sig_mode(0-1) | sgi(2) | cwb40(3)
            for (int k = 0; k < 4; k++) out[4 + k] = (uint8_t)(pk.rxts >> (8 * k));
            memcpy(out + 8, pk.buf, pk.len);
            send_framed(T_RX_TS, out, pk.len + 8);
        }
        // Drain scanned BLE advertisements -> T_BLE_RX [rssi][addr6][payload].
        while (bleq && xQueueReceive(bleq, &br, 0) == pdTRUE) {
            bout[0] = (uint8_t)br.rssi;
            memcpy(bout + 1, br.addr, 6);
            memcpy(bout + 7, br.buf, br.len);
            send_framed(T_BLE_RX, bout, 7 + br.len);
        }
        int64_t now = esp_timer_get_time();
        if (now - last_occ >= 200000) { // ~5×/s: emit the free-running activity counter
            last_occ = now;
            uint32_t a = s_activity;
            uint8_t c[4];
            for (int k = 0; k < 4; k++) c[k] = (uint8_t)(a >> (8 * k));
            send_framed(T_OCC, c, 4);
        }
    }
}

// Read the host protocol → dispatch inject / channel / rate / power / bw40.
static void serial_rx_loop(void) {
    static uint8_t acc[2048];
    int n = 0;
    for (;;) {
        int got = usb_serial_jtag_read_bytes(acc + n, sizeof(acc) - n, pdMS_TO_TICKS(50));
        if (got > 0) n += got;
        // Deframe from the front.
        int i = 0;
        while (n - i >= 5) {
            if (acc[i] != SYNC0 || acc[i + 1] != SYNC1) { i++; continue; }
            uint8_t ty = acc[i + 2];
            uint16_t len = acc[i + 3] | (acc[i + 4] << 8);
            if (n - i < 5 + len) break; // need more bytes
            uint8_t *pl = acc + i + 5;
            switch (ty) {
                case T_INJECT: esp_wifi_80211_tx(WIFI_IF_STA, pl, len, true); break;
                case T_BLE_ADV: ble_advertise(pl, len); break; // the BLE bearer, same firmware
                case T_COEX: if (len >= 4 && s_ble_ready) { // cognition sets the BLE↔Wi-Fi radio-time split
                    s_ble_scan_win = pl[0] | (pl[1] << 8);
                    s_ble_scan_itvl = pl[2] | (pl[3] << 8);
                    if (s_ble_scan_itvl < s_ble_scan_win) s_ble_scan_itvl = s_ble_scan_win; // window ≤ interval
                    ble_start_scan();
                } break;
                case T_CHANNEL: if (len >= 1) { esp_wifi_set_channel(pl[0], WIFI_SECOND_CHAN_NONE); s_cur_chan = pl[0]; } break;
                case T_TXPOWER: if (len >= 1) esp_wifi_set_max_tx_power((int8_t)pl[0]); break;
                case T_BW40: if (len >= 1) esp_wifi_set_bandwidth(WIFI_IF_STA, pl[0] ? WIFI_BW_HT40 : WIFI_BW_HT20); break;
                case T_RATE:
                    // [rate] (phymode auto-derived from rate code + band) or [rate][phymode] override.
                    // rate = wifi_phy_rate_t (1M_L=0x00, 24M=0x09, 54M=0x0C, MCS0_LGI=0x10, MCS7_LGI=0x17).
                    // [rate] | [rate][phymode] | [rate][phymode][he_flags: bit0=DCM bit1=ER-SU]
                    if (len >= 3)      set_fix_rate(pl[0], pl[1], pl[2] & 1, pl[2] & 2);
                    else if (len >= 2) set_fix_rate(pl[0], pl[1], false, false);
                    else if (len >= 1) set_fix_rate(pl[0], 0, false, false);
                    break;
                case T_INJECT_ATTR: { // [npairs][o,v]*n[frame] — skip the attr pokes (no pkt_attrib on esp), inject the frame
                    if (len >= 1) { int np = pl[0]; int off = 1 + 2 * np; if (len > off) esp_wifi_80211_tx(WIFI_IF_STA, pl + off, len - off, true); }
                    break;
                }
                case T_INJECT_AT: if (len >= 4) { // [delay_us_le32][frame] — place TX at a precise instant
                    uint32_t delay = pl[0] | ((uint32_t)pl[1] << 8) | ((uint32_t)pl[2] << 16) | ((uint32_t)pl[3] << 24);
                    if (delay > SCHED_MAX_DELAY_US) delay = SCHED_MAX_DELAY_US;
                    // Schedule against esp_timer (always-running µs); the 802.11 TSF reads 0 when the STA
                    // is unassociated, so it can't be the schedule clock here — but report it too so the
                    // host can see whether it's live (a future common-view lease would sync on it).
                    int64_t target = esp_timer_get_time() + delay;
                    while (esp_timer_get_time() < target) { /* spin to the scheduled instant */ }
                    esp_wifi_80211_tx(WIFI_IF_STA, pl + 4, len - 4, true);
                    int64_t actual = esp_timer_get_time();
                    int64_t tsf = esp_wifi_get_tsf_time(WIFI_IF_STA);
                    uint8_t rep[24];
                    for (int k = 0; k < 8; k++) {
                        rep[k] = (uint8_t)(target >> (8 * k));
                        rep[8 + k] = (uint8_t)(actual >> (8 * k));
                        rep[16 + k] = (uint8_t)(tsf >> (8 * k));
                    }
                    send_framed(T_TXTIME, rep, 24); // [target][actual][tsf]: error = actual-target
                    break; }
                case T_INJECT_ABS: if (len >= 8) { // [target_us_le64][frame] — TX at an ABSOLUTE esp_timer µs (slot lease)
                    int64_t target = 0;
                    for (int k = 0; k < 8; k++) target |= ((int64_t)pl[k]) << (8 * k);
                    int64_t now = esp_timer_get_time();
                    // Honour only a target within [now, now+cap] — a stale/past or far-future one is dropped
                    // rather than blocking the dispatch or firing late (the slot has already passed).
                    if (target > now && target - now <= SCHED_MAX_DELAY_US) {
                        while (esp_timer_get_time() < target) { /* spin to the scheduled instant */ }
                        esp_wifi_80211_tx(WIFI_IF_STA, pl + 8, len - 8, true);
                        int64_t actual = esp_timer_get_time();
                        int64_t tsf = esp_wifi_get_tsf_time(WIFI_IF_STA);
                        uint8_t rep[24];
                        for (int k = 0; k < 8; k++) {
                            rep[k] = (uint8_t)(target >> (8 * k));
                            rep[8 + k] = (uint8_t)(actual >> (8 * k));
                            rep[16 + k] = (uint8_t)(tsf >> (8 * k));
                        }
                        send_framed(T_TXTIME, rep, 24);
                    }
                    break; }
                case T_READCLOCK: { // reply with the current esp_timer (the T_INJECT_ABS schedule clock)
                    int64_t t = esp_timer_get_time();
                    uint8_t c[8]; for (int k = 0; k < 8; k++) c[k] = (uint8_t)(t >> (8 * k));
                    send_framed(T_CLOCK, c, 8);
                    break; }
                case T_NAMEFILTER: if (len >= 2) { // [enabled][n_masks][mask 16B]* — load host-computed Tier-0 masks
                    uint8_t nm = pl[1]; if (nm > MAX_MASKS) nm = MAX_MASKS;
                    if (len >= (uint16_t)(2 + nm * 16)) {
                        for (uint8_t m = 0; m < nm; m++) memcpy(nf_masks[m].b, pl + 2 + m * 16, 16);
                        nf_n_masks = nm;          // publish masks before enabling (rx_cb reads enabled last)
                        nf_enabled = pl[0] ? 1 : 0;
                    }
                    break; }
                default: break;
            }
            i += 5 + len;
        }
        if (i > 0) { memmove(acc, acc + i, n - i); n -= i; }
        if (n > (int)sizeof(acc) - 64) n = 0; // desync guard
    }
}

void app_main(void) {
    usb_serial_jtag_driver_config_t ucfg = USB_SERIAL_JTAG_DRIVER_CONFIG_DEFAULT();
    ucfg.rx_buffer_size = 2048;
    ucfg.tx_buffer_size = 2048;
    usb_serial_jtag_driver_install(&ucfg);

    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_start());
    // C5 is dual-band: enable 2.4 GHz + 5 GHz so a T_CHANNEL for a 5 GHz channel (36 = 5180 MHz, ..)
    // switches band automatically via esp_wifi_set_channel. Best-effort — older blobs may lack it.
    esp_wifi_set_band_mode(WIFI_BAND_MODE_AUTO);
    ESP_ERROR_CHECK(esp_wifi_set_channel(1, WIFI_SECOND_CHAN_NONE));
    set_fix_rate(WIFI_PHY_RATE_6M, 0, false, false); // OFDM 6M default — beats the 1 Mbps basic rate
    ESP_ERROR_CHECK(esp_wifi_set_promiscuous_rx_cb(rx_cb));
    ESP_ERROR_CHECK(esp_wifi_set_promiscuous(true));

    rxq = xQueueCreate(32, sizeof(rxpkt_t));
    bleq = xQueueCreate(24, sizeof(blerep_t));

    // BLE bearer (NimBLE) alongside Wi-Fi — one firmware, all bearers. Software coex (CONFIG_ESP_COEX_SW_
    // COEXIST_ENABLE, auto-on with BT+Wi-Fi) time-shares the radio between promiscuous Wi-Fi RX and BLE scan.
    ESP_ERROR_CHECK(nimble_port_init());
    ble_hs_cfg.sync_cb = ble_on_sync;
    nimble_port_freertos_init(ble_host_task);

    xTaskCreate(serial_tx_task, "ser_tx", 4096, NULL, 5, NULL);
    serial_rx_loop(); // never returns
}
