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
#define T_INJECT_AT  0x09 // [delay_us_le32][802.11 frame] — scheduled TX at TSF now+delay (airtime lease)
#define T_RX         0x81 // [rssi_i8][802.11 frame] — used by the BW16; the C5 sends T_RX_TS instead
#define T_RX_TS      0x82 // [rssi_i8][rx_ts_us_le32][802.11 frame] — RX + hardware per-frame µs stamp
#define T_TXTIME     0x83 // [target_le64][actual_le64][tsf_le64] — scheduling error report for T_INJECT_AT
#define MAXFRAME 512
#define MAX_MASKS 8
#define SCHED_MAX_DELAY_US 20000 // cap the busy-wait (this primitive spins; a slot lease would use a timer)

typedef struct { int8_t rssi; uint16_t len; uint32_t rxts; uint8_t buf[MAXFRAME]; } rxpkt_t;
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

// Promiscuous RX: queue each 0x8624 frame (+ RSSI) for the serial-TX task.
static void rx_cb(void *buf, wifi_promiscuous_pkt_type_t type) {
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
    static uint8_t out[5 + MAXFRAME];
    rxpkt_t pk;
    for (;;) {
        if (xQueueReceive(rxq, &pk, portMAX_DELAY) == pdTRUE) {
            out[0] = (uint8_t)pk.rssi;
            for (int k = 0; k < 4; k++) out[1 + k] = (uint8_t)(pk.rxts >> (8 * k));
            memcpy(out + 5, pk.buf, pk.len);
            send_framed(T_RX_TS, out, pk.len + 5);
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
                case T_CHANNEL: if (len >= 1) esp_wifi_set_channel(pl[0], WIFI_SECOND_CHAN_NONE); break;
                case T_TXPOWER: if (len >= 1) esp_wifi_set_max_tx_power((int8_t)pl[0]); break;
                case T_BW40: if (len >= 1) esp_wifi_set_bandwidth(WIFI_IF_STA, pl[0] ? WIFI_BW_HT40 : WIFI_BW_HT20); break;
                case T_RATE: if (len >= 1) { // payload byte = wifi_phy_rate_t (1M_L=0x00, 6M=0x0B, 54M=0x0C, MCS0_LGI=0x10..)
                    // MEASURED INERT for injection: esp_wifi_80211_tx always goes out at the 1 Mbps basic rate
                    // regardless of these calls (mt76 radiotap confirms 1.0 Mb/s for every rate 1M/6M/MCS0). The
                    // raw-inject path picks its own rate on this SoC (same as the BW16). Kept for any non-inject
                    // TX and in case a future IDF honours it; the injected-frame rate is NOT a working knob here.
                    esp_wifi_config_80211_tx_rate(WIFI_IF_STA, (wifi_phy_rate_t)pl[0]);
                    esp_wifi_internal_set_fix_rate(WIFI_IF_STA, true, (wifi_phy_rate_t)pl[0]);
                    break; }
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
    ESP_ERROR_CHECK(esp_wifi_set_promiscuous_rx_cb(rx_cb));
    ESP_ERROR_CHECK(esp_wifi_set_promiscuous(true));

    rxq = xQueueCreate(32, sizeof(rxpkt_t));
    xTaskCreate(serial_tx_task, "ser_tx", 4096, NULL, 5, NULL);
    serial_rx_loop(); // never returns
}
