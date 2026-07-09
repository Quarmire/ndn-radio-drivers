/*
 * esp32-cyd-radio — ESP32 "Cheap Yellow Display" (ESP32-2432S028R, ILI9341 320x240)
 * as a serial-bridged named-radio FrameIo node with a live on-screen dashboard.
 *
 * Same wire protocol and 802.11 inject/capture behaviour as esp32-ndn-bridge (so
 * ndn-radio-drivers::Bw16SerialBackend drives it too), but the CYD's TFT shows a
 * radio HUD: channel, TX/RX frame counts, last RSSI, and a scrolling list of the
 * most-recently-heard NDN frames (source MAC + RSSI). Put this and the Heltec node
 * on the same channel and each screen shows the other's traffic — a 2-node ESP32
 * mesh you can watch.
 *
 * Build: arduino-cli, esp32:esp32 core + LovyanGFX. Flash to the CYD.
 *
 * Wire protocol: see esp32-ndn-bridge.ino (identical).
 */

#define LGFX_USE_V1
#include <LovyanGFX.hpp>
#include "WiFi.h"
#include "esp_wifi.h"

// --- LovyanGFX config for the CYD (ESP32-2432S028R): ILI9341 on SPI2, backlight GPIO21 ---
class LGFX : public lgfx::LGFX_Device {
  lgfx::Panel_ILI9341 _panel;
  lgfx::Bus_SPI _bus;
  lgfx::Light_PWM _light;
public:
  LGFX() {
    { auto c = _bus.config();
      c.spi_host = SPI2_HOST; c.spi_mode = 0;
      c.freq_write = 40000000; c.freq_read = 16000000;
      c.pin_sclk = 14; c.pin_mosi = 13; c.pin_miso = 12; c.pin_dc = 2;
      _bus.config(c); _panel.setBus(&_bus); }
    { auto c = _panel.config();
      c.pin_cs = 15; c.pin_rst = -1; c.pin_busy = -1;
      c.panel_width = 240; c.panel_height = 320;
      c.readable = false; c.invert = false; c.rgb_order = false; c.bus_shared = false;
      _panel.config(c); }
    { auto c = _light.config();
      c.pin_bl = 21; c.invert = false; c.freq = 44100; c.pwm_channel = 7;
      _light.config(c); _panel.setLight(&_light); }
    setPanel(&_panel);
  }
};
static LGFX tft;

static const uint8_t SYNC0 = 0x4E, SYNC1 = 0x44;
enum { T_INJECT = 0x01, T_CHANNEL = 0x02, T_TXPOWER = 0x03, T_RX = 0x81, T_LOG = 0x82 };
static const size_t MAX_FRAME = 1600;

static volatile uint32_t tx_count = 0, rx_count = 0;
static volatile int8_t last_rssi = 0;
static volatile uint8_t cur_channel = 6;
static const char *phase = "boot";

// Ring buffer of recently-heard frames (src MAC + RSSI), rendered as a scrolling list.
struct RxLog { uint8_t src[6]; int8_t rssi; };
static const int LOG_N = 8;
static RxLog rxlog[LOG_N];      // written in the WiFi task, read in loop(); a torn read is cosmetic
static int rxlog_head = 0;
static volatile bool dirty = true;

static void send_framed(uint8_t type, const uint8_t *payload, uint16_t len) {
  uint8_t hdr[5] = {SYNC0, SYNC1, type, (uint8_t)(len & 0xff), (uint8_t)(len >> 8)};
  Serial.write(hdr, 5);
  if (len) Serial.write(payload, len);
}
static void logmsg(const char *s) { send_framed(T_LOG, (const uint8_t *)s, strlen(s)); }

static void tft_render() {
  char l[40];
  tft.startWrite();
  tft.fillScreen(TFT_BLACK);
  tft.setTextColor(TFT_YELLOW, TFT_BLACK);
  tft.setTextSize(2);
  tft.setCursor(6, 6);
  tft.print("NDN Radio Node");
  tft.setTextColor(TFT_CYAN, TFT_BLACK);
  tft.setTextSize(2);
  snprintf(l, sizeof l, "ch %-3d  %s", cur_channel, phase);
  tft.setCursor(6, 34); tft.print(l);
  tft.setTextColor(TFT_GREEN, TFT_BLACK);
  snprintf(l, sizeof l, "TX %-6lu RX %-6lu", (unsigned long)tx_count, (unsigned long)rx_count);
  tft.setCursor(6, 60); tft.print(l);
  tft.setTextColor(TFT_ORANGE, TFT_BLACK);
  snprintf(l, sizeof l, "last RSSI %d dBm", (int)last_rssi);
  tft.setCursor(6, 86); tft.print(l);
  tft.drawFastHLine(0, 110, 320, TFT_DARKGREY);
  tft.setTextSize(1);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.setCursor(6, 116); tft.print("recent frames (src / rssi)");
  for (int i = 0; i < LOG_N; i++) {
    int idx = (rxlog_head - 1 - i + LOG_N * 2) % LOG_N;
    RxLog e = rxlog[idx];
    if (e.src[0] == 0 && e.src[1] == 0 && e.src[5] == 0 && e.rssi == 0) continue;
    snprintf(l, sizeof l, "%02x:%02x:%02x:%02x:%02x:%02x  %d dBm",
             e.src[0], e.src[1], e.src[2], e.src[3], e.src[4], e.src[5], (int)e.rssi);
    tft.setCursor(10, 132 + i * 12); tft.print(l);
  }
  tft.endWrite();
}

static void promisc_cb(void *buf, wifi_promiscuous_pkt_type_t type) {
  if (type != WIFI_PKT_DATA) return;
  const wifi_promiscuous_pkt_t *p = (const wifi_promiscuous_pkt_t *)buf;
  const uint8_t *frame = p->payload;
  int flen = (int)p->rx_ctrl.sig_len - 4;
  if (flen < 32 || flen > (int)MAX_FRAME - 1) return;
  if ((frame[0] & 0x0C) != 0x08) return;
  if (frame[30] != 0x86 || frame[31] != 0x24) return;
  last_rssi = p->rx_ctrl.rssi;
  rx_count++;
  RxLog e;
  memcpy(e.src, frame + 10, 6); // addr2 = source
  e.rssi = p->rx_ctrl.rssi;
  rxlog[rxlog_head] = e;
  rxlog_head = (rxlog_head + 1) % LOG_N;
  dirty = true;
  static uint8_t out[MAX_FRAME];
  out[0] = (uint8_t)p->rx_ctrl.rssi;
  memcpy(out + 1, frame, flen);
  send_framed(T_RX, out, (uint16_t)(flen + 1));
}

void setup() {
  Serial.begin(115200);
  delay(200);
  tft.init();
  tft.setRotation(1); // landscape 320x240
  tft.fillScreen(TFT_BLACK);
  tft_render();
  logmsg("boot");
  WiFi.mode(WIFI_MODE_STA);
  esp_wifi_start();
  phase = "wifi_on"; logmsg("wifi_on");
  esp_wifi_set_channel(cur_channel, WIFI_SECOND_CHAN_NONE);
  logmsg("ch6");
  wifi_promiscuous_filter_t filt = {.filter_mask = WIFI_PROMIS_FILTER_MASK_DATA};
  esp_wifi_set_promiscuous_filter(&filt);
  esp_wifi_set_promiscuous_rx_cb(&promisc_cb);
  esp_wifi_set_promiscuous(true);
  phase = "ready"; logmsg("ready");
  dirty = true;
}

static uint8_t rxbuf[MAX_FRAME];
void loop() {
  static unsigned long last_render = 0;
  if (dirty && millis() - last_render > 150) {
    last_render = millis(); dirty = false;
    tft_render();
  }
  if (Serial.read() != SYNC0) return;
  while (Serial.available() < 1) {}
  if (Serial.read() != SYNC1) return;
  while (Serial.available() < 3) {}
  uint8_t type = Serial.read();
  uint16_t len = Serial.read();
  len |= ((uint16_t)Serial.read()) << 8;
  if (len > MAX_FRAME) return;
  size_t got = 0;
  unsigned long t0 = millis();
  while (got < len && millis() - t0 < 200) {
    if (Serial.available()) rxbuf[got++] = Serial.read();
  }
  if (got != len) { logmsg("short cmd"); return; }

  switch (type) {
    case T_INJECT:
      esp_wifi_80211_tx(WIFI_IF_STA, rxbuf, len, true);
      tx_count++; dirty = true;
      break;
    case T_CHANNEL:
      if (len >= 1) { cur_channel = rxbuf[0]; esp_wifi_set_channel(cur_channel, WIFI_SECOND_CHAN_NONE); dirty = true; }
      break;
    case T_TXPOWER:
      if (len >= 1) esp_wifi_set_max_tx_power((int8_t)rxbuf[0]);
      break;
    default:
      break;
  }
}
