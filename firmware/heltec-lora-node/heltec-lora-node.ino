/*
 * heltec-lora-node — Heltec WiFi LoRa 32 V2 as a **standalone** on-device LoRa node.
 *
 * Unlike the serial-bridged backends (bw16/esp32-ndn-bridge/waveshare), nothing on a host drives
 * this: the ESP32 runs the SX1276 (SX127x) itself via RadioLib — beacons a named frame, receives
 * peers, and shows live state on the on-board OLED. Its purpose in the named-radio effort is (a) a
 * self-contained sub-GHz node, and (b) an interop check: with the air params matched (frequency,
 * SF7 / BW125 / CR4-5 / sync word) it should hear the Waveshare SX1262 dongles, proving SX127x↔
 * SX126x LoRa interoperability.
 *
 * Build: arduino-cli, esp32:esp32 core + RadioLib + U8g2. Flash to the Heltec V2.
 *
 * Air params are #defines below — tweak FREQ_MHZ to match the peer (the Waveshare's channel index
 * maps to a fixed MHz its firmware doesn't expose; 868 for the EU HF band is the first guess).
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

// --- Air parameters: match the peer to interoperate ---
#define FREQ_MHZ 915.0    // US ISM band (Waveshare channel 65 = 850+65)
#define BW_KHZ 125.0      // Waveshare BW=0
#define SF 7              // Waveshare SF=7
#define CR_DENOM 5        // Waveshare CR=1 → coding rate 4/5
#define SYNC_WORD 0x12    // LoRa private sync word (RadioLib default)
#define TX_POWER_DBM 17
#define PREAMBLE_LEN 8   // match the Waveshare (measured 8-symbol preamble on air)
#define TX_CRC true // match the Waveshare (its frames carry a CRC), else its RX drops ours
#define BEACON_MS 3000

SX1276 radio = new Module(LORA_CS, LORA_DIO0, LORA_RST, LORA_DIO1);
U8G2_SSD1306_128X64_NONAME_F_SW_I2C oled(U8G2_R0, OLED_SCL, OLED_SDA, OLED_RST);

volatile bool rx_flag = false;
void IRAM_ATTR on_dio0() { rx_flag = true; }

uint32_t tx_count = 0, rx_count = 0;
float last_rssi = 0, last_snr = 0;
char last_rx[24] = "";
const char *phase = "boot";

void oled_render() {
  char l[26];
  oled.clearBuffer();
  oled.setFont(u8g2_font_7x13B_tf);
  oled.drawStr(0, 12, "LoRa node (SX1276)");
  oled.setFont(u8g2_font_6x10_tf);
  snprintf(l, sizeof l, "%.1fMHz SF%d %s", (double)FREQ_MHZ, SF, phase);
  oled.drawStr(0, 26, l);
  snprintf(l, sizeof l, "TX:%lu  RX:%lu", (unsigned long)tx_count, (unsigned long)rx_count);
  oled.drawStr(0, 39, l);
  snprintf(l, sizeof l, "RSSI %.0f SNR %.0f", (double)last_rssi, (double)last_snr);
  oled.drawStr(0, 51, l);
  snprintf(l, sizeof l, "rx:%s", last_rx);
  oled.drawStr(0, 63, l);
  oled.sendBuffer();
}

void setup() {
  Serial.begin(115200);
  delay(200);
  // OLED power + reset (Heltec V2).
  pinMode(VEXT, OUTPUT);
  digitalWrite(VEXT, LOW);
  pinMode(OLED_RST, OUTPUT);
  digitalWrite(OLED_RST, LOW);
  delay(20);
  digitalWrite(OLED_RST, HIGH);
  oled.begin();
  oled_render();

  SPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_CS);
  Serial.print("radio.begin… ");
  int st = radio.begin(FREQ_MHZ, BW_KHZ, SF, CR_DENOM, SYNC_WORD, TX_POWER_DBM, PREAMBLE_LEN);
  Serial.println(st);
  if (st != RADIOLIB_ERR_NONE) {
    phase = "RADIO FAIL";
    oled_render();
    while (true) delay(1000);
  }
  int cst = radio.setCRC(TX_CRC); // payload CRC on TX (and require it on RX)
  Serial.printf("setCRC(%d) -> %d\n", (int)TX_CRC, cst);
  radio.setDio0Action(on_dio0, RISING);
  radio.startReceive();
  phase = "listening";
  oled_render();
}

void loop() {
  // RX: a completed reception raised DIO0.
  if (rx_flag) {
    rx_flag = false;
    String data;
    int st = radio.readData(data);
    if (st == RADIOLIB_ERR_NONE) {
      rx_count++;
      last_rssi = radio.getRSSI();
      last_snr = radio.getSNR();
      strncpy(last_rx, data.c_str(), sizeof(last_rx) - 1);
      last_rx[sizeof(last_rx) - 1] = 0;
      Serial.printf("RX [%.0f dBm, SNR %.1f] %s\n", last_rssi, last_snr, data.c_str());
      oled_render();
    }
    radio.startReceive();
  }
  // TX: beacon a named frame every BEACON_MS (transmit() blocks, then resume RX).
  static unsigned long last_tx = 0;
  if (millis() - last_tx > BEACON_MS) {
    last_tx = millis();
    char msg[24];
    snprintf(msg, sizeof msg, "HELTEC-LORA seq=%lu", (unsigned long)tx_count);
    int st = radio.transmit((uint8_t *)msg, strlen(msg));
    if (st == RADIOLIB_ERR_NONE) {
      tx_count++;
      Serial.printf("TX %s\n", msg);
    } else {
      Serial.printf("TX err %d\n", st);
    }
    radio.startReceive();
    oled_render();
  }
}
