/*
 * esp32-ndn-bridge — turn an ESP32 (WROOM/Heltec/CYD) into a serial-bridged
 * named-radio FrameIo backend for the ndn-rs stack.
 *
 * Protocol-compatible with the BW16 bridge: the host builds complete 802.11
 * frames (ndn_frame_io::frame::build_dot11) and drives this board with the SAME
 * length-framed serial protocol, so ndn-radio-drivers::Bw16SerialBackend runs it
 * unchanged. This firmware is purely a transport: raw-inject what it's told with
 * esp_wifi_80211_tx, forward what promiscuous mode hears — with the ESP32's real
 * per-frame RSSI (rx_ctrl.rssi), which the BW16 SDK could not surface.
 *
 * On a Heltec WiFi LoRa 32 V2 (HAS_OLED) the on-board SSD1306 shows live node
 * status — channel, TX/RX frame counts, last RSSI — so the injection node is
 * observable at a glance without a host attached.
 *
 * Build: Arduino with the esp32:esp32 core + U8g2 lib. Flash to any ESP32 dev
 * board; leave HAS_OLED on for the Heltec, comment it out for a headless board.
 *
 * Wire protocol (both directions), little-endian:
 *   [0x4E 0x44] [type:u8] [len:u16] [payload:len]
 *   host->board: 0x01 INJECT   payload = complete 802.11 frame (no FCS; HW adds it)
 *                0x02 CHANNEL   payload = [channel:u8]
 *                0x03 TXPOWER   payload = [quarter-dBm:u8]  (esp_wifi_set_max_tx_power)
 *   board->host: 0x81 RX        payload = [rssi:i8] + captured 802.11 frame (FCS stripped)
 *                0x82 LOG       payload = ASCII status
 */

#include "WiFi.h"
#include "esp_wifi.h"

// --- Heltec WiFi LoRa 32 V2 on-board SSD1306 OLED (comment out for a headless board) ---
#define HAS_OLED 1
#ifdef HAS_OLED
#include <U8g2lib.h>
#define OLED_SDA 4
#define OLED_SCL 15
#define OLED_RST 16
#define VEXT 21
U8G2_SSD1306_128X64_NONAME_F_SW_I2C oled(U8G2_R0, OLED_SCL, OLED_SDA, OLED_RST);
#endif

static const uint8_t SYNC0 = 0x4E, SYNC1 = 0x44;
enum {
  T_INJECT = 0x01,
  T_CHANNEL = 0x02,
  T_TXPOWER = 0x03,
  T_RX = 0x81,
  T_LOG = 0x82,
};
static const size_t MAX_FRAME = 1600;

// Live status (promisc_cb runs in the WiFi task; loop() renders — plain volatiles suffice).
static volatile uint32_t tx_count = 0, rx_count = 0;
static volatile int8_t last_rssi = 0;
static volatile uint8_t cur_channel = 6;
static const char *phase = "boot";

static void send_framed(uint8_t type, const uint8_t *payload, uint16_t len) {
  uint8_t hdr[5] = {SYNC0, SYNC1, type, (uint8_t)(len & 0xff), (uint8_t)(len >> 8)};
  Serial.write(hdr, 5);
  if (len) Serial.write(payload, len);
}

static void logmsg(const char *s) { send_framed(T_LOG, (const uint8_t *)s, strlen(s)); }

#ifdef HAS_OLED
static void oled_render() {
  char l[24];
  oled.clearBuffer();
  oled.setFont(u8g2_font_7x13B_tf);
  oled.drawStr(0, 12, "NDN radio node");
  oled.setFont(u8g2_font_6x10_tf);
  snprintf(l, sizeof l, "ch %-3d  %s", cur_channel, phase);
  oled.drawStr(0, 26, l);
  snprintf(l, sizeof l, "TX: %lu", (unsigned long)tx_count);
  oled.drawStr(0, 40, l);
  snprintf(l, sizeof l, "RX: %lu", (unsigned long)rx_count);
  oled.drawStr(0, 52, l);
  snprintf(l, sizeof l, "RSSI: %d dBm", (int)last_rssi);
  oled.drawStr(0, 64, l);
  oled.sendBuffer();
}
#endif

/* Promiscuous RX: forward NDN 802.11 DATA frames + real RSSI to the host. */
static void promisc_cb(void *buf, wifi_promiscuous_pkt_type_t type) {
  if (type != WIFI_PKT_DATA) return;
  const wifi_promiscuous_pkt_t *p = (const wifi_promiscuous_pkt_t *)buf;
  const uint8_t *frame = p->payload;
  // sig_len includes the 4-byte FCS the host neither builds nor expects — strip it.
  int flen = (int)p->rx_ctrl.sig_len - 4;
  if (flen < 32 || flen > (int)MAX_FRAME - 1) return;
  // Same NDN-radio prefilter as the BW16 bridge: a non-QoS DATA frame carrying our
  // ethertype 0x8624 after the 24-byte 802.11 header + 6-byte LLC/SNAP.
  if ((frame[0] & 0x0C) != 0x08) return;              // DATA frame
  if (frame[30] != 0x86 || frame[31] != 0x24) return; // NDN ethertype
  last_rssi = p->rx_ctrl.rssi;
  rx_count++;
  static uint8_t out[MAX_FRAME];
  out[0] = (uint8_t)p->rx_ctrl.rssi; // real per-frame RSSI, dBm
  memcpy(out + 1, frame, flen);
  send_framed(T_RX, out, (uint16_t)(flen + 1));
}

void setup() {
  Serial.begin(115200);
  delay(200);
#ifdef HAS_OLED
  pinMode(VEXT, OUTPUT);
  digitalWrite(VEXT, LOW); // enable OLED power rail (Heltec Vext, active-low)
  pinMode(OLED_RST, OUTPUT);
  digitalWrite(OLED_RST, LOW);
  delay(20);
  digitalWrite(OLED_RST, HIGH);
  oled.begin();
  oled_render();
#endif
  logmsg("boot");
  WiFi.mode(WIFI_MODE_STA);
  esp_wifi_start();
  phase = "wifi_on";
  logmsg("wifi_on");
  esp_wifi_set_channel(cur_channel, WIFI_SECOND_CHAN_NONE);
  logmsg("ch6");
  wifi_promiscuous_filter_t filt = {.filter_mask = WIFI_PROMIS_FILTER_MASK_DATA};
  esp_wifi_set_promiscuous_filter(&filt);
  esp_wifi_set_promiscuous_rx_cb(&promisc_cb);
  esp_wifi_set_promiscuous(true);
  phase = "ready";
  logmsg("ready");
#ifdef HAS_OLED
  oled_render();
#endif
}

/* Read one framed command from the host; inject or retune. Refresh the OLED on a timer. */
static uint8_t rxbuf[MAX_FRAME];
void loop() {
#ifdef HAS_OLED
  static unsigned long last_render = 0;
  if (millis() - last_render > 250) {
    last_render = millis();
    oled_render();
  }
#endif
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
      // en_sys_seq = true: the MAC fills the sequence number; broadcast NDN frames
      // don't depend on it. Raw payload is otherwise sent verbatim; HW adds FCS.
      esp_wifi_80211_tx(WIFI_IF_STA, rxbuf, len, true);
      tx_count++;
      break;
    case T_CHANNEL:
      if (len >= 1) {
        cur_channel = rxbuf[0];
        esp_wifi_set_channel(cur_channel, WIFI_SECOND_CHAN_NONE);
      }
      break;
    case T_TXPOWER:
      if (len >= 1) esp_wifi_set_max_tx_power((int8_t)rxbuf[0]); // quarter-dBm units
      break;
    default:
      break;
  }
}
