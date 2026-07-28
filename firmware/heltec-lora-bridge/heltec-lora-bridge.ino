/*
 * heltec-lora-bridge — Heltec WiFi LoRa 32 V2 (ESP32 + SX1276) as a host-driven LoRa modem that
 * speaks the SAME serial protocol as the Waveshare `waveshare-lora-rs` firmware. So the OPi host
 * drives it with the identical `LoraSerialBackend` (no host changes) — this is what makes the Heltec
 * a third NDN node (task #54, N=3 LBT test) alongside the two Waveshare SX1262 dongles.
 *
 * Wire framing (identical to waveshare): `7E A5 | type | len | payload[len] | xor-crc`, where
 * xor-crc = type ^ len ^ payload bytes. Air side is plain LoRa (no proprietary header), so SX1276
 * interoperates with the SX1262 dongles when params match.
 *
 * Build on the Mac:  arduino-cli compile --fqbn esp32:esp32:esp32 firmware/heltec-lora-bridge
 * Flash in place on o5p-1:  nix-shell -p esptool --run "esptool.py --chip esp32 -p /dev/ttyUSB0 \
 *     write_flash 0x10000 heltec-lora-bridge.ino.bin" (plus bootloader/partitions at 0x1000/0x8000)
 *
 * Pins are the Heltec V2 SX1276 map, reused from heltec-lora-node.ino.
 */

#include <RadioLib.h>
#include <U8g2lib.h>

// --- Heltec V2 SX1276 pinout (SPI) + OLED (I2C) ---
#define LORA_SCK 5
#define LORA_MISO 19
#define LORA_MOSI 27
#define LORA_CS 18
#define LORA_RST 14
#define LORA_DIO0 26
#define LORA_DIO1 35
#define OLED_SDA 4
#define OLED_SCL 15
#define OLED_RST 16
#define VEXT 21

// --- serial framing (must match src/lora_serial.rs) ---
static const uint8_t SYNC0 = 0x7E, SYNC1 = 0xA5;
// host -> firmware
static const uint8_t CMD_TX = 0x01, CMD_SET_FREQ = 0x02, CMD_SET_MOD = 0x03, CMD_SET_PWR = 0x04,
                     CMD_SET_SYNC = 0x05, CMD_GET_INFO = 0x06, CMD_SET_BEACON = 0x07, CMD_CAD = 0x08,
                     CMD_GET_RSSI = 0x09, CMD_SET_CAD_CFG = 0x0A, CMD_SET_LBT_CFG = 0x0B,
                     CMD_SET_PREAMBLE = 0x0C, CMD_SF_SCAN = 0x0D, CMD_TX_LBT = 0x0E;
// data-plane / stats opcodes we ACK but don't implement on the Heltec (the host cognition owns NDN
// logic; the Heltec is a plain modem here): 0x0F..0x16.
// firmware -> host
static const uint8_t EVT_RX = 0x81, EVT_TXDONE = 0x82, EVT_INFO = 0x83, EVT_LOG = 0x84,
                     EVT_CAD = 0x85, EVT_RSSI = 0x86, EVT_SF_DETECTED = 0x87, EVT_TX_STARTED = 0x88;

// --- default air parameters (host overrides at runtime) ---
static float freq_mhz = 915.0;   // Waveshare channel 65 = 850 + 65
static float bw_khz = 125.0;
static uint8_t sf = 9;
static uint8_t cr_denom = 5;     // 4/5
static uint8_t sync_word = 0x12;
static int8_t pwr_dbm = 17;
static uint16_t preamble = 8;

// LBT tunables (CMD_SET_LBT_CFG)
static uint16_t lbt_cw_ms = 20;
static uint8_t lbt_max_backoff = 5;
static uint8_t lbt_max_attempts = 8;
// CSMA observability counters (reported in EVT_INFO tail, like the Waveshare)
static uint16_t cad_busy = 0, defer_ct = 0;

SX1276 radio = new Module(LORA_CS, LORA_DIO0, LORA_RST, LORA_DIO1);
U8G2_SSD1306_128X64_NONAME_F_SW_I2C oled(U8G2_R0, OLED_SCL, OLED_SDA, OLED_RST);

volatile bool rx_flag = false;
void IRAM_ATTR on_dio0() { rx_flag = true; }

uint32_t tx_count = 0, rx_count = 0;
float last_rssi = 0, last_snr = 0;
char last_rx[24] = "";

// ---- framed output ----
static void send_frame(uint8_t type, const uint8_t *p, uint8_t len) {
  uint8_t hdr[4] = {SYNC0, SYNC1, type, len};
  Serial.write(hdr, 4);
  uint8_t crc = type ^ len;
  for (uint8_t i = 0; i < len; i++) crc ^= p[i];
  if (len) Serial.write(p, len);
  Serial.write(&crc, 1);
  Serial.flush();
}

static void send_info() {
  // [status, sync(2), errors(2), freq(4 BE Hz), sf, bw_code, cr, pwr, lost(2), cad_busy(2), defer(2)]
  uint32_t hz = (uint32_t)(freq_mhz * 1e6);
  uint8_t bw_code = (bw_khz > 360) ? 2 : (bw_khz > 180 ? 1 : 0);
  uint8_t p[19] = {0};
  p[0] = 0;                       // status OK
  p[1] = 0; p[2] = sync_word;     // sync (2)
  p[3] = 0; p[4] = 0;             // errors (2)
  p[5] = (hz >> 24); p[6] = (hz >> 16); p[7] = (hz >> 8); p[8] = hz; // freq BE
  p[9] = sf; p[10] = bw_code; p[11] = cr_denom - 4; p[12] = (uint8_t)pwr_dbm;
  p[13] = 0; p[14] = 0;           // lost (2)
  p[15] = (cad_busy >> 8); p[16] = cad_busy;
  p[17] = (defer_ct >> 8); p[18] = defer_ct;
  send_frame(EVT_INFO, p, sizeof p);
}

static void send_txdone(uint8_t ok, uint8_t attempts) {
  uint8_t p[2] = {ok, attempts};
  send_frame(EVT_TXDONE, p, 2);
}

static void apply_mod() {
  radio.setFrequency(freq_mhz);
  radio.setBandwidth(bw_khz);
  radio.setSpreadingFactor(sf);
  radio.setCodingRate(cr_denom);
  radio.setSyncWord(sync_word);
  radio.setOutputPower(pwr_dbm);
  radio.setPreambleLength(preamble);
  radio.startReceive();
}

// One LoRa transmit, then back to RX. Returns RadioLib status.
static int do_tx(const uint8_t *data, uint8_t len) {
  int st = radio.transmit((uint8_t *)data, len);
  radio.startReceive();
  if (st == RADIOLIB_ERR_NONE) tx_count++;
  return st;
}

// Listen-before-talk: CAD (scanChannel) with bounded backoff, then transmit. Mirrors the Waveshare
// lbt_tx(): channel free -> key up; busy -> random backoff up to max_attempts; give up -> defer.
static void tx_lbt(const uint8_t *data, uint8_t len) {
  for (uint8_t attempt = 0; attempt < lbt_max_attempts; attempt++) {
    int cad = radio.scanChannel();
    if (cad == RADIOLIB_CHANNEL_FREE) {
      uint8_t st = do_tx(data, len);
      send_txdone(st == RADIOLIB_ERR_NONE ? 1 : 0, attempt + 1);
      return;
    }
    cad_busy++;
    // exponential-ish backoff, one CAD-slot base, jittered by the ESP32 RNG
    uint32_t slot = (uint32_t)lbt_cw_ms << (attempt < lbt_max_backoff ? attempt : lbt_max_backoff);
    delay((esp_random() % (slot + 1)) + 1);
  }
  defer_ct++;
  radio.startReceive();
  send_txdone(0, lbt_max_attempts); // deferred: channel stayed busy
}

// ---- framed input parser ----
static uint8_t rxbuf[300];
static void handle_cmd(uint8_t type, const uint8_t *p, uint8_t len) {
  switch (type) {
    case CMD_TX: {
      uint8_t st = do_tx(p, len);
      send_txdone(st == RADIOLIB_ERR_NONE ? 1 : 0, 0);
      break;
    }
    case CMD_TX_LBT:
      tx_lbt(p, len);
      break;
    case CMD_SET_FREQ:
      if (len >= 4) {
        uint32_t hz = ((uint32_t)p[0] << 24) | ((uint32_t)p[1] << 16) | ((uint32_t)p[2] << 8) | p[3];
        freq_mhz = hz / 1e6;
        apply_mod();
      }
      send_info();
      break;
    case CMD_SET_MOD:
      if (len >= 3) {
        sf = p[0];
        bw_khz = (p[1] == 2) ? 500.0 : (p[1] == 1 ? 250.0 : 125.0);
        cr_denom = 4 + (p[2] ? p[2] : 1); // cr_code 1..4 -> 4/5..4/8
        apply_mod();
      }
      send_info();
      break;
    case CMD_SET_PWR:
      if (len >= 1) { pwr_dbm = (int8_t)p[0]; radio.setOutputPower(pwr_dbm); }
      send_info();
      break;
    case CMD_SET_SYNC:
      if (len >= 1) { sync_word = p[0]; radio.setSyncWord(sync_word); radio.startReceive(); }
      send_info();
      break;
    case CMD_SET_PREAMBLE:
      if (len >= 2) { preamble = ((uint16_t)p[0] << 8) | p[1]; radio.setPreambleLength(preamble); radio.startReceive(); }
      send_info();
      break;
    case CMD_SET_LBT_CFG:
      if (len >= 4) { lbt_cw_ms = ((uint16_t)p[0] << 8) | p[1]; lbt_max_backoff = p[2]; lbt_max_attempts = p[3]; }
      send_info();
      break;
    case CMD_CAD: {
      uint8_t busy = (radio.scanChannel() != RADIOLIB_CHANNEL_FREE) ? 1 : 0;
      radio.startReceive();
      send_frame(EVT_CAD, &busy, 1);
      break;
    }
    case CMD_GET_RSSI: {
      int16_t r = (int16_t)radio.getRSSI(false); // current channel RSSI
      uint8_t q[2] = {(uint8_t)(r >> 8), (uint8_t)r};
      send_frame(EVT_RSSI, q, 2);
      break;
    }
    case CMD_GET_INFO:
    default:
      // Every other SET (beacon/cad-cfg/name-filter/relay/dataplane/sense/stats/debug/bootloader) is
      // ACKed with EVT_INFO so the host's idempotent SET path is satisfied. The Heltec is a plain
      // modem; NDN data-plane logic lives in the host cognition for this node.
      send_info();
      break;
  }
}

// Feed one byte through the frame state machine.
static void parse_byte(uint8_t b) {
  static enum { S0, S1, TYPE, LEN, PAY, CRC } st = S0;
  static uint8_t type, len, idx, crc;
  switch (st) {
    case S0: st = (b == SYNC0) ? S1 : S0; break;
    case S1: st = (b == SYNC1) ? TYPE : (b == SYNC0 ? S1 : S0); break;
    case TYPE: type = b; crc = b; st = LEN; break;
    case LEN: len = b; crc ^= b; idx = 0; st = (len == 0) ? CRC : PAY; break;
    case PAY:
      rxbuf[idx++] = b; crc ^= b;
      if (idx >= len) st = CRC;
      break;
    case CRC:
      if (b == crc) handle_cmd(type, rxbuf, len);
      st = S0;
      break;
  }
}

void oled_render() {
  char l[26];
  oled.clearBuffer();
  oled.setFont(u8g2_font_7x13B_tf);
  oled.drawStr(0, 12, "LoRa bridge C");
  oled.setFont(u8g2_font_6x10_tf);
  snprintf(l, sizeof l, "%.1fM SF%d BW%.0f", (double)freq_mhz, sf, (double)bw_khz);
  oled.drawStr(0, 26, l);
  snprintf(l, sizeof l, "TX:%lu RX:%lu", (unsigned long)tx_count, (unsigned long)rx_count);
  oled.drawStr(0, 39, l);
  snprintf(l, sizeof l, "busy:%u def:%u", cad_busy, defer_ct);
  oled.drawStr(0, 51, l);
  snprintf(l, sizeof l, "rx:%s", last_rx);
  oled.drawStr(0, 63, l);
  oled.sendBuffer();
}

void setup() {
  Serial.begin(115200);
  pinMode(VEXT, OUTPUT); digitalWrite(VEXT, LOW);
  pinMode(OLED_RST, OUTPUT); digitalWrite(OLED_RST, LOW); delay(20); digitalWrite(OLED_RST, HIGH);
  oled.begin();
  SPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_CS);
  int rc = radio.begin(freq_mhz, bw_khz, sf, cr_denom, sync_word, pwr_dbm, preamble);
  radio.setCRC(true); // Waveshare frames carry a CRC; require it so the air formats match
  radio.setDio0Action(on_dio0, RISING);
  radio.startReceive();
  send_info(); // announce ready
  (void)rc;
  oled_render();
}

void loop() {
  // Drain host serial.
  while (Serial.available()) parse_byte((uint8_t)Serial.read());

  // RX complete?
  if (rx_flag) {
    rx_flag = false;
    size_t n = radio.getPacketLength();
    if (n > 0 && n <= sizeof(rxbuf)) {
      int st = radio.readData(rxbuf, n);
      if (st == RADIOLIB_ERR_NONE) {
        rx_count++;
        last_rssi = radio.getRSSI();
        last_snr = radio.getSNR();
        int16_t r = (int16_t)last_rssi, s = (int16_t)(last_snr * 1); // dBm, dB
        uint32_t ts = millis();
        // EVT_RX = [rssi i16 BE, snr i16 BE, ts_ms u32 BE, LoRa bytes]
        static uint8_t ev[308];
        ev[0] = r >> 8; ev[1] = r; ev[2] = s >> 8; ev[3] = s;
        ev[4] = ts >> 24; ev[5] = ts >> 16; ev[6] = ts >> 8; ev[7] = ts;
        memcpy(ev + 8, rxbuf, n);
        send_frame(EVT_RX, ev, (uint8_t)(8 + n));
        size_t k = (n < sizeof(last_rx) - 1) ? n : sizeof(last_rx) - 1;
        memcpy(last_rx, rxbuf, k); last_rx[k] = 0;
      }
    }
    radio.startReceive();
    oled_render();
  }

  static unsigned long last_oled = 0;
  if (millis() - last_oled > 1000) { last_oled = millis(); oled_render(); }
}
