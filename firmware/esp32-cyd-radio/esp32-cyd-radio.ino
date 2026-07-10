/*
 * esp32-cyd-radio — ESP32 "Cheap Yellow Display" (ESP32-2432S028R, ILI9341 320x240)
 * as a serial-bridged named-radio FrameIo node with a live on-screen radio HUD,
 * including a Channel State Information (CSI) subcarrier plot.
 *
 * Same wire protocol and 802.11 inject/capture behaviour as esp32-ndn-bridge (so
 * ndn-radio-drivers::Bw16SerialBackend drives it too). Additionally it enables the
 * ESP32's native CSI: every received frame yields per-subcarrier amplitude/phase,
 * which the TFT plots live. Point the Heltec node (self-beaconing on the same
 * channel) at it and the CSI plot updates continuously — waving a hand near the
 * antennas visibly perturbs it (CSI = motion/presence sensing, not just a link).
 *
 * Build: arduino-cli, esp32:esp32 core + LovyanGFX. Flash at UploadSpeed=115200
 * (the CYD's CH340 drops the default 921600 mid-write).
 */

#define LGFX_USE_V1
#include <LovyanGFX.hpp>
#include <math.h>
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
enum { T_INJECT = 0x01, T_CHANNEL = 0x02, T_TXPOWER = 0x03, T_RX = 0x81, T_LOG = 0x82, T_CSI = 0x83 };
static const size_t MAX_FRAME = 1600;

static volatile uint32_t tx_count = 0, rx_count = 0;
static volatile int8_t last_rssi = 0, last_noise = 0;
static volatile uint8_t cur_channel = 6;
static const char *phase = "boot";

// CSI: per-subcarrier amplitude of the most recent frame (int8 I/Q -> magnitude).
static const int CSI_MAX = 128;
static uint8_t csi_amp[CSI_MAX];
static volatile int csi_n = 0;
static volatile uint32_t csi_count = 0;
static volatile bool dirty = true;

static void send_framed(uint8_t type, const uint8_t *payload, uint16_t len) {
  uint8_t hdr[5] = {SYNC0, SYNC1, type, (uint8_t)(len & 0xff), (uint8_t)(len >> 8)};
  Serial.write(hdr, 5);
  if (len) Serial.write(payload, len);
}
static void logmsg(const char *s) { send_framed(T_LOG, (const uint8_t *)s, strlen(s)); }

static void hud_render() {
  char l[48];
  // Header block — text with black background so fields overwrite cleanly (no full clear/flicker).
  tft.setTextColor(TFT_YELLOW, TFT_BLACK); tft.setTextSize(2);
  tft.setCursor(6, 6); tft.print("NDN Radio + CSI ");
  tft.setTextColor(TFT_CYAN, TFT_BLACK);
  snprintf(l, sizeof l, "ch %-3d  %-8s", cur_channel, phase);
  tft.setCursor(6, 32); tft.print(l);
  tft.setTextColor(TFT_GREEN, TFT_BLACK);
  snprintf(l, sizeof l, "TX %-7lu RX %-7lu", (unsigned long)tx_count, (unsigned long)rx_count);
  tft.setCursor(6, 58); tft.print(l);
  tft.setTextColor(TFT_ORANGE, TFT_BLACK);
  snprintf(l, sizeof l, "RSSI %-4d noise %-4d   ", (int)last_rssi, (int)last_noise);
  tft.setCursor(6, 84); tft.print(l);
  tft.setTextColor(TFT_WHITE, TFT_BLACK); tft.setTextSize(1);
  int n = csi_n; if (n > CSI_MAX) n = CSI_MAX;
  snprintf(l, sizeof l, "CSI %-3d subcarriers  frames %-8lu", n, (unsigned long)csi_count);
  tft.setCursor(6, 108); tft.print(l);
  // CSI subcarrier bar plot, drawn directly (no sprite): clear the plot band, draw bars.
  const int y0 = 232, H = 108;
  tft.fillRect(0, 120, 320, 120, TFT_BLACK);
  tft.drawFastHLine(0, 120, 320, TFT_DARKGREY);
  if (n > 0) {
    for (int i = 0; i < n; i++) {
      int x = (i * 319) / (n - 1 > 0 ? n - 1 : 1);
      int h = (csi_amp[i] * H) / 127;
      if (h > 0) tft.drawFastVLine(x, y0 - h, h, TFT_MAGENTA);
    }
  }
}

// Promiscuous RX: forward NDN DATA frames to the host + count them.
static void promisc_cb(void *buf, wifi_promiscuous_pkt_type_t type) {
  if (type != WIFI_PKT_DATA) return;
  const wifi_promiscuous_pkt_t *p = (const wifi_promiscuous_pkt_t *)buf;
  const uint8_t *frame = p->payload;
  int flen = (int)p->rx_ctrl.sig_len - 4;
  if (flen < 32 || flen > (int)MAX_FRAME - 1) return;
  if ((frame[0] & 0x0C) != 0x08) return;
  if (frame[30] != 0x86 || frame[31] != 0x24) return;
  rx_count++;
  static uint8_t out[MAX_FRAME];
  out[0] = (uint8_t)p->rx_ctrl.rssi;
  memcpy(out + 1, frame, flen);
  send_framed(T_RX, out, (uint16_t)(flen + 1));
}

// CSI: per-frame channel state — magnitude of each subcarrier's I/Q pair.
static void csi_cb(void *ctx, wifi_csi_info_t *info) {
  if (!info || !info->buf) return;
  int n = info->len / 2;
  if (n > CSI_MAX) n = CSI_MAX;
  for (int i = 0; i < n; i++) {
    int im = info->buf[2 * i];
    int re = info->buf[2 * i + 1];
    int a = (int)sqrtf((float)(im * im + re * re));
    csi_amp[i] = a > 127 ? 127 : (uint8_t)a;
  }
  csi_n = n;
  csi_count++;
  last_rssi = info->rx_ctrl.rssi;
  last_noise = info->rx_ctrl.noise_floor;
  dirty = true;
  // Forward a throttled CSI summary to the host: [rssi:i8][n:u8][amp:n] — real subcarrier
  // magnitudes, so CSI-based sensing can run host-side too (a named-radio capability, not just a
  // local plot). ~4/s keeps the 115200 link clear.
  static unsigned long last_csi_tx = 0;
  if (millis() - last_csi_tx > 250) {
    last_csi_tx = millis();
    static uint8_t buf[2 + CSI_MAX];
    buf[0] = (uint8_t)info->rx_ctrl.rssi;
    buf[1] = (uint8_t)n;
    memcpy(buf + 2, csi_amp, n);
    send_framed(T_CSI, buf, (uint16_t)(2 + n));
  }
}

void setup() {
  Serial.begin(115200);
  delay(200);
  tft.init();
  tft.setRotation(1);
  tft.fillScreen(TFT_BLACK);
  logmsg("boot");
  WiFi.mode(WIFI_MODE_STA);
  esp_wifi_start();
  phase = "wifi_on"; logmsg("wifi_on");
  esp_wifi_set_channel(cur_channel, WIFI_SECOND_CHAN_NONE);
  wifi_promiscuous_filter_t filt = {.filter_mask = WIFI_PROMIS_FILTER_MASK_DATA};
  esp_wifi_set_promiscuous_filter(&filt);
  esp_wifi_set_promiscuous_rx_cb(&promisc_cb);
  esp_wifi_set_promiscuous(true);
  // Enable native CSI: legacy + HT long training fields, channel-filtered, merged.
  wifi_csi_config_t csi_cfg = {};
  csi_cfg.lltf_en = true;
  csi_cfg.htltf_en = true;
  csi_cfg.stbc_htltf2_en = true;
  csi_cfg.ltf_merge_en = true;
  csi_cfg.channel_filter_en = true;
  csi_cfg.manu_scale = false;
  csi_cfg.shift = 0;
  char m[48];
  esp_err_t e1 = esp_wifi_set_csi_config(&csi_cfg);
  esp_err_t e2 = esp_wifi_set_csi_rx_cb(&csi_cb, NULL);
  esp_err_t e3 = esp_wifi_set_csi(true);
  snprintf(m, sizeof m, "csi cfg=%d cb=%d en=%d", (int)e1, (int)e2, (int)e3);
  logmsg(m);
  phase = "ready"; logmsg("ready");
  dirty = true;
}

static uint8_t rxbuf[MAX_FRAME];
void loop() {
  static unsigned long last_render = 0;
  if (millis() - last_render > 150) { // always refresh, independent of RX activity
    last_render = millis(); dirty = false;
    hud_render();
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
