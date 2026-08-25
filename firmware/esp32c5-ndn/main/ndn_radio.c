// ESP32-C5 named-data radio node — the "FoA" (frame-over-air) primitive for the ndn-radio fleet.
//
// The C5 is a dual-band Wi-Fi 6 part; here it does exactly what the AR9271/Realtek FrameIo backends do:
// inject and capture raw 802.11 data frames carrying the canonical NDN LLC/SNAP ethertype 0x8624, so a
// frame it TXes de-frames identically on any other radio (`parse_dot11(RawNdn{0x8624})`), and it counts
// the 0x8624 frames it hears. No association, no netstack — promiscuous RX + esp_wifi_80211_tx.
//
// Rust-transcription note (per the plan): raw injection lives only in the ESP-IDF C API today (bare-metal
// esp-radio has Sniffer RX but no raw TX); esp-idf-svc binds esp_wifi_80211_tx via FFI, so this maps 1:1
// to a std-Rust node later. Keep the wire layout + the state here as the reference.
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_mac.h"
#include "nvs_flash.h"

static const char *TAG = "c5-ndn";

// The canonical named-data-over-802.11 EtherType (matches ndn-radio-drivers NDN_ETHERTYPE).
#define NDN_ETHERTYPE 0x8624
#define CHANNEL 1

static volatile uint32_t rx_ndn = 0, rx_total = 0, tx_count = 0;
static int8_t last_rssi = 0;

// Promiscuous RX: count frames whose LLC/SNAP ethertype is 0x8624 (our NDN frames).
static void rx_cb(void *buf, wifi_promiscuous_pkt_type_t type) {
    if (type != WIFI_PKT_DATA) return;
    const wifi_promiscuous_pkt_t *p = (wifi_promiscuous_pkt_t *)buf;
    const uint8_t *f = p->payload;
    int len = p->rx_ctrl.sig_len;
    rx_total++;
    // 802.11 data hdr = 24 B; LLC/SNAP = 8 B (aa aa 03 00 00 00 <et_hi> <et_lo>). ethertype @ 30..31.
    if (len >= 32 && f[24] == 0xaa && f[25] == 0xaa && f[26] == 0x03) {
        uint16_t et = ((uint16_t)f[30] << 8) | f[31];
        if (et == NDN_ETHERTYPE) {
            rx_ndn++;
            last_rssi = p->rx_ctrl.rssi;
        }
    }
}

// Build one broadcast 802.11 data frame carrying an NDN payload under LLC/SNAP 0x8624.
static int build_frame(uint8_t *out, const uint8_t *src_mac, const uint8_t *payload, int plen) {
    int i = 0;
    out[i++] = 0x08; out[i++] = 0x00;                 // FC: type=data subtype=0, no flags
    out[i++] = 0x00; out[i++] = 0x00;                 // duration
    for (int k = 0; k < 6; k++) out[i++] = 0xff;      // addr1 = DA = broadcast
    memcpy(&out[i], src_mac, 6); i += 6;              // addr2 = SA = our MAC
    for (int k = 0; k < 6; k++) out[i++] = 0xff;      // addr3 = BSSID = broadcast
    out[i++] = 0x00; out[i++] = 0x00;                 // seq (HW fills when en_sys_seq=true)
    // LLC/SNAP → EtherType 0x8624
    out[i++] = 0xaa; out[i++] = 0xaa; out[i++] = 0x03;
    out[i++] = 0x00; out[i++] = 0x00; out[i++] = 0x00;
    out[i++] = (NDN_ETHERTYPE >> 8) & 0xff; out[i++] = NDN_ETHERTYPE & 0xff;
    memcpy(&out[i], payload, plen); i += plen;
    return i;
}

void app_main(void) {
    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_start());
    // Pin the channel and go promiscuous (no association) — a pure named-radio, like monitor mode.
    ESP_ERROR_CHECK(esp_wifi_set_channel(CHANNEL, WIFI_SECOND_CHAN_NONE));
    ESP_ERROR_CHECK(esp_wifi_set_promiscuous_rx_cb(rx_cb));
    ESP_ERROR_CHECK(esp_wifi_set_promiscuous(true));

    uint8_t mac[6];
    esp_wifi_get_mac(WIFI_IF_STA, mac);
    ESP_LOGI(TAG, "C5 named-radio up: ch%d, SA %02x:%02x:%02x:%02x:%02x:%02x, ethertype 0x%04x",
             CHANNEL, mac[0], mac[1], mac[2], mac[3], mac[4], mac[5], NDN_ETHERTYPE);

    uint8_t frame[64];
    const uint8_t payload[] = { 0x05, 0x08, 'c', '5', '-', 'n', 'd', 'n' }; // tiny NDN-ish TLV
    int flen = build_frame(frame, mac, payload, sizeof(payload));

    int secs = 0;
    while (1) {
        // Inject the NDN frame ~10x/s.
        for (int n = 0; n < 10; n++) {
            if (esp_wifi_80211_tx(WIFI_IF_STA, frame, flen, true) == ESP_OK) tx_count++;
            vTaskDelay(pdMS_TO_TICKS(100));
        }
        secs++;
        ESP_LOGI(TAG, "t=%ds  TX=%lu  RX(0x8624)=%lu / total=%lu  lastRSSI=%d",
                 secs, tx_count, rx_ndn, rx_total, last_rssi);
    }
}
