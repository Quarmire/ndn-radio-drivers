/*
 * bw16-rs-sketch — the thin C++ shim for the Rust BW16 firmware.
 *
 * ALL firmware logic lives in Rust (libbw16_rs.a, from ../bw16-rs). This file
 * exists only because the Ameba WiFi stack is a closed C blob and Arduino owns
 * setup()/loop(): it wraps the Arduino + SDK C calls behind the `c_*` FFI the
 * Rust core imports, forwards setup()/loop() to Rust, and trampolines the
 * promiscuous-RX callback into Rust. No logic, no state — just glue.
 *
 * Build: compile the Rust staticlib for thumbv8m.main-none-eabihf, then link it
 * into the Ameba image via the Arduino build (see build-bw16-rs.sh).
 */

#include "packet-injection.h" // GPLv3: wifi_tx_raw_frame(void*, size_t)
#include "WiFi.h"
extern "C" {
#include "wifi_conf.h"
int wext_set_bw40_enable(unsigned char enable);
int wifi_set_tx_data_rate(unsigned char data_rate);
}

// --- Rust entry points (in libbw16_rs.a) ---
extern "C" void rust_setup(void);
extern "C" void rust_loop(void);
extern "C" void rust_promisc_cb(const unsigned char *buf, unsigned int len);

// --- C wrappers the Rust core imports ---
extern "C" void c_serial_begin(unsigned int baud) { Serial.begin(baud); }
extern "C" void c_serial_write(const unsigned char *buf, unsigned int len) { Serial.write(buf, len); }
extern "C" int c_serial_read(void) { return Serial.read(); }
extern "C" int c_serial_available(void) { return Serial.available(); }
extern "C" void c_delay(unsigned int ms) { delay(ms); }
extern "C" unsigned int c_millis(void) { return (unsigned int)millis(); }
extern "C" void c_wifi_on_sta(void) { wifi_on(RTW_MODE_STA); }
extern "C" void c_wifi_set_channel(int ch) { wifi_set_channel(ch); }
extern "C" void c_wifi_tx_raw_frame(const unsigned char *buf, unsigned int len) {
  wifi_tx_raw_frame((void *)buf, (size_t)len);
}
extern "C" void c_wifi_set_tx_data_rate(unsigned char code) { wifi_set_tx_data_rate(code); }
extern "C" void c_wext_set_bw40(unsigned char en) { wext_set_bw40_enable(en); }

// SDK promisc callback -> Rust
static void promisc_trampoline(unsigned char *buf, unsigned int len, void *ud) {
  (void)ud;
  rust_promisc_cb(buf, len);
}
extern "C" void c_wifi_set_promisc_enable(void) {
  wifi_set_promisc(RTW_PROMISC_ENABLE_2, promisc_trampoline, 1);
}

void setup() { rust_setup(); }
void loop() { rust_loop(); }
