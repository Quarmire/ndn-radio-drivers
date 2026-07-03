/*
 * bw16-ndn-bridge — turn a BW16 (RTL8720DN) into a serial-bridged named-radio
 * FrameIo backend for the ndn-rs stack.
 *
 * The host (ndn-radio-drivers::Bw16SerialBackend) hands us complete 802.11 frames
 * to inject and we hand it captured frames back, over a tiny length-framed serial
 * protocol. All NDN framing (radiotap-free 802.11 + LLC/SNAP) is built on the host
 * by `ndn_frame_io::frame::build_dot11`, so this firmware is purely a transport:
 * raw-inject what it's told, forward what it hears.
 *
 * Build: Arduino IDE with the "Realtek Ameba" (AmebaD) board package, board
 * "AI-Thinker BW16 (RTL8720DN)". Add tesa-klebeband/RTL8720dn-WiFi-Packet-Injection
 * as `packet-injection.{h,cpp}` beside this sketch (its GPLv3 lib provides
 * wifi_tx_raw_frame — we only call it).
 *
 * Wire protocol (both directions), little-endian:
 *   [0x4E 0x44] [type:u8] [len:u16] [payload:len]
 *   host->board: 0x01 INJECT  payload = complete 802.11 frame (no FCS; auto-added)
 *                0x02 CHANNEL  payload = [channel:u8]
 *   board->host: 0x81 RX       payload = [rssi:i8] + captured 802.11 frame
 *                0x82 LOG      payload = ASCII status
 */

#include "packet-injection.h" // GPLv3 submodule: wifi_tx_raw_frame(frame, len)
#include "WiFi.h"
extern "C" {
#include "wifi_conf.h" // wifi_on, wifi_set_channel, wifi_set_promisc
}

static const uint8_t SYNC0 = 0x4E, SYNC1 = 0x44;
enum { T_INJECT = 0x01, T_CHANNEL = 0x02, T_RX = 0x81, T_LOG = 0x82 };
static const size_t MAX_FRAME = 1600;

static void send_framed(uint8_t type, const uint8_t *payload, uint16_t len) {
  uint8_t hdr[5] = {SYNC0, SYNC1, type, (uint8_t)(len & 0xff), (uint8_t)(len >> 8)};
  Serial.write(hdr, 5);
  if (len) Serial.write(payload, len);
}

static void logmsg(const char *s) { send_framed(T_LOG, (const uint8_t *)s, strlen(s)); }

/* Promiscuous RX: forward every captured 802.11 frame + its RSSI to the host. */
static void promisc_cb(unsigned char *buf, unsigned int len, void *userdata) {
  // AmebaD hands the raw 802.11 frame in `buf`. Per-frame RSSI format varies by
  // SDK version (no stable rx-info struct here) — forward 0 for now; refine once
  // RX is confirmed on air.
  (void)userdata;
  int8_t rssi = 0;
  if (len == 0 || len > MAX_FRAME - 1) return;
  static uint8_t out[MAX_FRAME];
  out[0] = (uint8_t)rssi;
  memcpy(out + 1, buf, len);
  send_framed(T_RX, out, (uint16_t)(len + 1));
}

void setup() {
  // The RTL8720 LOG UART (where the USB-TTL sits) is native 115200 and shared
  // with the WiFi driver's debug; match it and let the host deframer pick our
  // SYNC'd frames out of the noise. Emit per-step markers so a hang in WiFi init
  // is visible. RX (promisc) is enabled last — TX/inject is the priority.
  Serial.begin(115200);
  delay(200);
  logmsg("boot");
  wifi_on(RTW_MODE_STA);
  logmsg("wifi_on");
  wifi_set_channel(6);
  logmsg("ch6");
  wifi_set_promisc(RTW_PROMISC_ENABLE_2, promisc_cb, 1);
  logmsg("ready");
}

/* Read one framed command from the host; inject or retune. */
static uint8_t rxbuf[MAX_FRAME];
void loop() {
  // sync
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
      wifi_tx_raw_frame(rxbuf, len); // FCS + seq added by hardware
      break;
    case T_CHANNEL:
      if (len >= 1) wifi_set_channel(rxbuf[0]);
      break;
    default:
      break;
  }
}
