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
#define MAXFRAME 512

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

// Fix the raw-injection TX rate. phymode==0 => auto-derive from the rate code + current band.
static void set_fix_rate(uint8_t rate, uint8_t phymode) {
    wifi_phy_mode_t pm = phymode ? (wifi_phy_mode_t)phymode : phymode_for_rate(rate, s_cur_chan);
    ic_set_80211_tx_rate(WIFI_IF_STA, rate);
    wifi_tx_rate_config_t cfg = { .phymode = pm, .rate = (wifi_phy_rate_t)rate, .ersu = false, .dcm = false };
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
static void serial_tx_task(void *arg) {
    static uint8_t out[8 + MAXFRAME];
    rxpkt_t pk;
    int64_t last_occ = esp_timer_get_time();
    for (;;) {
        // 100 ms timeout so the loop wakes to emit T_OCC even when no frames are queued.
        if (xQueueReceive(rxq, &pk, pdMS_TO_TICKS(100)) == pdTRUE) {
            out[0] = (uint8_t)pk.rssi;
            out[1] = (uint8_t)pk.noise;      // noise floor (dBm)
            out[2] = pk.rate_code;           // legacy rate or MCS (per flags sig_mode)
            out[3] = pk.flags;               // sig_mode(0-1) | sgi(2) | cwb40(3)
            for (int k = 0; k < 4; k++) out[4 + k] = (uint8_t)(pk.rxts >> (8 * k));
            memcpy(out + 8, pk.buf, pk.len);
            send_framed(T_RX_TS, out, pk.len + 8);
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
                case T_CHANNEL: if (len >= 1) { esp_wifi_set_channel(pl[0], WIFI_SECOND_CHAN_NONE); s_cur_chan = pl[0]; } break;
                case T_TXPOWER: if (len >= 1) esp_wifi_set_max_tx_power((int8_t)pl[0]); break;
                case T_BW40: if (len >= 1) esp_wifi_set_bandwidth(WIFI_IF_STA, pl[0] ? WIFI_BW_HT40 : WIFI_BW_HT20); break;
                case T_RATE:
                    // [rate] (phymode auto-derived from rate code + band) or [rate][phymode] override.
                    // rate = wifi_phy_rate_t (1M_L=0x00, 24M=0x09, 54M=0x0C, MCS0_LGI=0x10, MCS7_LGI=0x17).
                    if (len >= 2)      set_fix_rate(pl[0], pl[1]);
                    else if (len >= 1) set_fix_rate(pl[0], 0);
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
    set_fix_rate(WIFI_PHY_RATE_6M, 0); // OFDM 6M default — beats the 1 Mbps basic rate for raw injection
    ESP_ERROR_CHECK(esp_wifi_set_promiscuous_rx_cb(rx_cb));
    ESP_ERROR_CHECK(esp_wifi_set_promiscuous(true));

    rxq = xQueueCreate(32, sizeof(rxpkt_t));
    xTaskCreate(serial_tx_task, "ser_tx", 4096, NULL, 5, NULL);
    serial_rx_loop(); // never returns
}
