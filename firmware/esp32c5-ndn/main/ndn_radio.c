// ESP32-C5 named-data radio node — host-driven serial-bridge FrameIo (BW16 wire protocol).
//
// The C5 does the FoA primitive (raw 802.11 inject + promiscuous capture of ethertype 0x8624) and is
// driven over its native USB-Serial-JTAG by the host's `Bw16SerialBackend` UNCHANGED — same framing
// `[0x4E 0x44 type len_le16 payload]`: host→device T_INJECT (a full 802.11 frame to esp_wifi_80211_tx),
// T_CHANNEL/T_RATE/T_TXPOWER/T_BW40 (1-byte params); device→host T_RX = [rssi_i8][802.11 frame] for each
// 0x8624 frame heard. Console logging is OFF (LOG level NONE) so the binary stream is clean, while the
// console stays bound to the USB-Serial-JTAG so esptool auto-reset keeps working (see sdkconfig.defaults).
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

#define NDN_ETHERTYPE 0x8624
#define SYNC0 0x4E
#define SYNC1 0x44
#define T_INJECT     0x01
#define T_CHANNEL    0x02
#define T_TXPOWER    0x03
#define T_RATE       0x04
#define T_BW40       0x05
#define T_INJECT_ATTR 0x06
#define T_RX         0x81
#define MAXFRAME 512

typedef struct { int8_t rssi; uint16_t len; uint8_t buf[MAXFRAME]; } rxpkt_t;
static QueueHandle_t rxq;

// Promiscuous RX: queue each 0x8624 frame (+ RSSI) for the serial-TX task.
static void rx_cb(void *buf, wifi_promiscuous_pkt_type_t type) {
    if (type != WIFI_PKT_DATA) return;
    const wifi_promiscuous_pkt_t *p = (wifi_promiscuous_pkt_t *)buf;
    const uint8_t *f = p->payload;
    int len = p->rx_ctrl.sig_len;
    if (len < 32 || len > MAXFRAME) return;
    if (!(f[24] == 0xaa && f[25] == 0xaa && f[26] == 0x03)) return;
    if ((((uint16_t)f[30] << 8) | f[31]) != NDN_ETHERTYPE) return;
    rxpkt_t pk;
    pk.rssi = p->rx_ctrl.rssi;
    pk.len = len;
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

// Drain the RX queue → serial as T_RX [rssi][frame].
static void serial_tx_task(void *arg) {
    static uint8_t out[1 + MAXFRAME];
    rxpkt_t pk;
    for (;;) {
        if (xQueueReceive(rxq, &pk, portMAX_DELAY) == pdTRUE) {
            out[0] = (uint8_t)pk.rssi;
            memcpy(out + 1, pk.buf, pk.len);
            send_framed(T_RX, out, pk.len + 1);
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
