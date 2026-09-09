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

#include <stdio.h> // snprintf
#include "packet-injection.h" // GPLv3: wifi_tx_raw_frame(void*, size_t)
#include "WiFi.h"
#include "BLEDevice.h"
extern "C" {
#include "wifi_conf.h"
// The BLE GAP layer is plain C. BLEDevice.h pulls some of it in, but not the
// advertising-parameter API we drive directly for the per-payload address rotation.
#include "gap.h"
#include "gap_adv.h"
#include "gap_le.h"
#include "gap_le_types.h"
#include "gap_scan.h"
#include "gap_config.h"
// vendor_cmd_bt.h lives under .../board/amebad/src/vendor_cmd/ and is not on the
// sketch include path, so declare what we use. It is a normal linkable symbol in
// lib_arduino.a (vendor_cmd.o), guarded in its own header by a flag that is set.
T_GAP_CAUSE le_adv_set_tx_power(uint8_t option, uint8_t tx_gain);
int wext_set_bw40_enable(unsigned char enable);
int wifi_set_tx_data_rate(unsigned char data_rate);
// wifi_set_txpower() is behind `#if 0 //Not ready` in wifi_conf.c, so it's not
// linkable — but its one-liner (wext_private_command "txpower patha=N") is, since
// wext_private_command (wifi_util.c) is live. We reimplement it below.
int wext_private_command(const char *ifname, char *cmd, int show_msg);

// --- Blob-internal symbols (all global T/COMMON in lib_wlan.a, which sits inside
// --- the Arduino link's --start-group, so a plain extern "C" resolves them).
// The public wrappers for these are either stubbed out (`wifi_set_txpower` is
// behind `#if 0 //Not ready`, and its iwpriv handler is a parse-and-echo stub) or
// aimed at the wrong TX path (`wifi_set_tx_data_rate` writes the DATA-path fixed
// rate, which the MGMT path our injector uses ignores). The internals are not.
//
// Rate: rtl8721d_update_txdesc fills TX-descriptor DATA_RATE from
// MRateToHwRate(padapter[0x855]) and sets the use-fixed-rate bit for every mgmt
// frame — so padapter[0x855], written by update_mgnt_tx_rate, IS the inject rate.
void update_mgnt_tx_rate(void *padapter, unsigned char mgn_rate);
// Power: the TXAGC byte per rate, path A. 1 index step = 0.25 dB.
unsigned char config_phydm_write_txagc_8721d(void *dm, unsigned int power_index,
                                             unsigned char path, unsigned char hw_rate);
void rtl8721d_set_tx_power_level(void *padapter, unsigned char channel);
void halrf_set_pwr_track(void *dm, unsigned char enable);
int rltk_set_tx_power_percentage(unsigned long idx);
// Readback — compiled, unlike wifi_get_txpower(). 20 live TXAGC bytes:
// [0..3]=CCK 1/2/5.5/11, [4..11]=OFDM 6..54, [12..19]=HT MCS0..7.
int wext_get_tx_power(const char *ifname, unsigned char *poweridx);
// The controller's LE feature mask, cached by the BT stack at init from
// `LE Read Local Supported Features`. 8 bytes; `le_check_supported_features(byte,
// mask)` is just `gap_local_features[byte] & mask`. Reading it answers "what BLE 5
// features does this SILICON have" without sending a single HCI command — the
// difference between a controller limitation and a host-stack build choice.
// Bit map (Core spec): byte1 bit0 = LE 2M PHY, bit3 = LE Coded PHY,
// bit4 = LE Extended Advertising, bit5 = LE Periodic Advertising.
extern unsigned char gap_local_features[8];
// Generic HCI command sender: (opcode, params, len). Its opcode switch only selects
// which of three internal queues the message uses — the default branch still sends —
// so it forwards ANY opcode to the controller verbatim.
int hci_send_cmd_msg(unsigned short opcode, unsigned char *params, unsigned short len);
// The blessed raw-HCI path: posts a message to the BT task rather than touching the
// HCI queues from ours. The opcode and parameters reach the controller verbatim —
// there is no whitelist, no OGF check and no length check anywhere on the TX path.
T_GAP_CAUSE gap_vendor_cmd_req(unsigned short op, unsigned char len, unsigned char *p_param);
// mbed HAL microsecond ticker (lib_arduino.a). Preferred over Arduino micros():
// micros() on this core is (tick*1000 - SysTick_current/200) and steps BACKWARDS
// by up to ~1 ms at a tick boundary; us_ticker_read carries its own monotonicity
// correction. The Rust side clamps either way.
uint32_t us_ticker_read(void);
void us_ticker_init(void);
}

// Walk to the driver's private structures. `rltk_wlan_info` is declared (in
// packet-injection.h) as the first word of the SDK's netdev table — i.e. the
// `dev` pointer — so dev+0x10 is `priv`, and *priv is the padapter. This is the
// same walk the GPLv3 injector already does for alloc_mgtxmitframe.
static void *bw16_padapter(void) {
  if (!rltk_wlan_info) return 0;
  uint32_t *priv = *(uint32_t **)(rltk_wlan_info + 0x10);
  if (!priv) return 0;
  return (void *)*priv;
}
// phydm's `dm` struct: padapter -> pHalData (+0x20C8) -> dm (+0x23C8).
static void *bw16_dm(void) {
  unsigned char *pa = (unsigned char *)bw16_padapter();
  if (!pa) return 0;
  unsigned char *hal = *(unsigned char **)(pa + 0x20C8);
  if (!hal) return 0;
  return hal + 0x23C8;
}

// --- Rust entry points (in libbw16_rs.a) ---
extern "C" void rust_setup(void);
extern "C" void rust_loop(void);
extern "C" void rust_promisc_cb(const unsigned char *buf, unsigned int len, signed char rssi,
                                unsigned char mrate);
extern "C" void rust_rxinfo_dump(const unsigned char *ud, unsigned int n);
extern "C" void rust_ble_rx(signed char rssi, const unsigned char *addr,
                            const unsigned char *payload, unsigned int len);
extern "C" void rust_ble_seen(void);

// --- C wrappers the Rust core imports ---
extern "C" void c_serial_begin(unsigned int baud) { Serial.begin(baud); }
extern "C" void c_serial_write(const unsigned char *buf, unsigned int len) { Serial.write(buf, len); }
extern "C" int c_serial_read(void) { return Serial.read(); }
extern "C" int c_serial_available(void) { return Serial.available(); }
extern "C" void c_delay(unsigned int ms) { delay(ms); }
extern "C" unsigned int c_millis(void) { return (unsigned int)millis(); }
extern "C" unsigned int c_micros(void) { return (unsigned int)us_ticker_read(); }

// Producer-side mutual exclusion for the Rust core's outbound ring. The promisc
// callback runs in the WiFi task and rust_loop in the Arduino task; masking
// interrupts stops preemption between them (and is what Arduino's own
// noInterrupts() does on this core). Not nested by any caller, so one saved mask
// suffices — inside the section nothing else can be running to overwrite it.
static uint32_t s_irq_mask;
extern "C" void c_enter_critical(void) { s_irq_mask = ulSetInterruptMaskFromISR(); }
extern "C" void c_exit_critical(void) { vClearInterruptMaskFromISR(s_irq_mask); }
extern "C" void c_wifi_on_sta(void) { wifi_on(RTW_MODE_STA); us_ticker_init(); }
extern "C" void c_wifi_set_channel(int ch) { wifi_set_channel(ch); }
// The requested management-TX rate as a Realtek MGN code; 0 = leave the driver's
// default alone. Re-asserted before EVERY inject on purpose: update_tx_basic_rate
// resets padapter[0x855] to MGN_1M/MGN_6M on any band or channel change, so a
// set-once knob would silently revert the first time cognition retunes.
static unsigned char g_mgn_rate = 0;
extern "C" void c_set_mgnt_rate(unsigned char mgn) { g_mgn_rate = mgn; }

extern "C" void c_wifi_tx_raw_frame(const unsigned char *buf, unsigned int len) {
  if (g_mgn_rate) {
    void *pa = bw16_padapter();
    if (pa) update_mgnt_tx_rate(pa, g_mgn_rate);
  }
  wifi_tx_raw_frame((void *)buf, (size_t)len);
}

// Set every rate's TXAGC index (0..127; 1 step = 0.25 dB). Returns the number of
// rate slots written, or a negative status. The pointer walk is checked first:
// odm_set_bb_reg and friends all begin `ldr r0,[r0,#0]` and pass the result to a
// padapter-taking function, so *dm must equal padapter. If it does not, the
// offsets are wrong for this SDK build and we must not write anything.
extern "C" int c_set_txagc(unsigned int idx) {
  void *dm = bw16_dm(), *pa = bw16_padapter();
  if (!dm || !pa) return -1;
  if (*(void **)dm != pa) return -2;
  halrf_set_pwr_track(dm, 0); // else the DM watchdog reprograms TXAGC from its tables
  int n = 0;
  for (unsigned char r = 0; r <= 19; r++) n += config_phydm_write_txagc_8721d(dm, idx, 0, r);
  return n;
}
// Restore the driver's computed per-rate power (undoes c_set_txagc).
extern "C" int c_reset_txagc(unsigned char channel) {
  void *pa = bw16_padapter();
  if (!pa) return -1;
  rtl8721d_set_tx_power_level(pa, channel);
  return 0;
}
// Coarse but stomp-proof: 0=100%, 1=-1.5dB, 2=-3dB, 3=-6dB, 4=-9dB.
extern "C" int c_set_tx_power_pct(unsigned long idx) { return rltk_set_tx_power_percentage(idx); }
// The instrument: read the 20 live TXAGC bytes back out of the hardware.
extern "C" int c_get_txagc(unsigned char *out20) { return wext_get_tx_power("wlan0", out20); }

// Rate-controllable raw inject: same management-TX primitives as wifi_tx_raw_frame
// (alloc_mgtxmitframe / update_mgntframe_attrib / dump_mgntframe — SDK symbols;
// the adapter/xmit-frame offsets are the tesa-klebeband GPLv3 lib's discovered
// ABI facts), but after update_mgntframe_attrib fills the default (legacy) attribs
// we poke `n` [offset,value] byte pairs into the pkt_attrib (at xmit_frame+8)
// before dump. That lets the host sweep for the rate field (and set it to an
// MGN_MCS code) — escaping the fixed-rate mgmt path. `pairs` = [off0,val0,...].
extern "C" void c_wifi_tx_raw_frame_attr(const unsigned char *frame, unsigned int len,
                                         const unsigned char *pairs, unsigned int n_pairs) {
  unsigned char *ptr = (unsigned char *)**(uint32_t **)(rltk_wlan_info + 0x10);
  unsigned char *xf = (unsigned char *)alloc_mgtxmitframe(ptr + 0xa80);
  if (!xf) return;
  unsigned char *pattrib = xf + 8;
  update_mgntframe_attrib(ptr, pattrib);
  for (unsigned int i = 0; i < n_pairs; i++) {
    pattrib[pairs[2 * i]] = pairs[2 * i + 1];
  }
  memset((void *)*(uint32_t *)(xf + 0x80), 0, 0x68);
  unsigned char *fd = (unsigned char *)*(uint32_t *)(xf + 0x80) + 0x28;
  memcpy(fd, frame, len);
  *(uint32_t *)(xf + 0x14) = len;
  *(uint32_t *)(xf + 0x18) = len;
  dump_mgntframe(ptr, xf);
}
extern "C" void c_wifi_set_tx_data_rate(unsigned char code) { wifi_set_tx_data_rate(code); }
extern "C" void c_wext_set_bw40(unsigned char en) { wext_set_bw40_enable(en); }
extern "C" void c_wifi_set_txpower(int idx) {
  char buf[24];
  snprintf(buf, sizeof(buf), "txpower patha=%d", idx);
  wext_private_command("wlan0", buf, 0);
}

// ---------------------------------------------------------------------------
// BLE bearer — the same named-data-over-advertising contract as the ESP32-C5's,
// so ONE host backend drives either radio. Named data rides a manufacturer AD
// with company id 0x4E44 ("ND"), the BLE analogue of the 0x8624 ethertype: the
// device forwards only ads carrying that magic, so the 115200 UART is not
// swamped by every beacon in the room.
//
// This part is LEGACY advertising only — BLE 5 extended advertising is compiled
// out of the shipped BT stack, so the payload ceiling is 31 - 4 = 27 bytes,
// against the C5's ~245. Named data still fits; large Data packets need more
// fragments here than on the C5.
#define BLE_MAGIC_LO 0x44
#define BLE_MAGIC_HI 0x4E
#define BLE_MAXPAY 27

static bool s_ble_ready = false;
static unsigned char s_own_addr[6] = {0};

// Rotate the advertiser's BD address for every payload.
//
// Two reasons, and they agree. (1) DOCTRINE: a named-data radio carries no host
// identity — the source field is an ephemeral nonce, not a station address — so a
// stable BD address on this bearer would be exactly the persistent identifier the
// design forbids. (2) MEASURED: BLE controllers duplicate-filter by ADVERTISER
// ADDRESS, not by payload. With a fixed address a receiver hears the first advert
// and suppresses the rest until its scan period resets — the ESP32-C5 peer heard
// 2 of 20 rapid adverts, but 5 of 6 when they were spaced 3 s apart. A fresh
// non-resolvable address per payload makes each advert a new device to the
// receiver's filter, so back-to-back fragments all get through.
static void ble_rotate_addr(void) {
  unsigned char rnd[6];
  if (le_gen_rand_addr(GAP_RAND_ADDR_NON_RESOLVABLE, rnd) != GAP_CAUSE_SUCCESS) return;
  if (le_set_rand_addr(rnd) != GAP_CAUSE_SUCCESS) return;
  memcpy(s_own_addr, rnd, 6); // keep our own-reflection filter pointed at the live address
}

// Forward only ads carrying our company magic, and never our own reflections.
static void ble_scan_cb(T_LE_CB_DATA *p) {
  if (!p || !p->p_le_scan_info) return;
  rust_ble_seen(); // count EVERY advertisement, before any filter — the BLE analogue
                   // of T_OCC, and the diagnostic that separates "the scanner is
                   // dead" from "nothing is advertising our magic".
  T_LE_SCAN_INFO *si = p->p_le_scan_info;
  if (memcmp(si->bd_addr, s_own_addr, 6) == 0) return;
  unsigned char pos = 0;
  while (pos < si->data_len) {
    unsigned char adlen = si->data[pos];
    if (adlen < 1 || (unsigned int)(pos + 1 + adlen) > si->data_len) break;
    unsigned char type = si->data[pos + 1];
    if (type == 0xFF && adlen >= 4 && si->data[pos + 2] == BLE_MAGIC_LO &&
        si->data[pos + 3] == BLE_MAGIC_HI) {
      rust_ble_rx((signed char)si->rssi, si->bd_addr, &si->data[pos + 4], adlen - 3);
      return;
    }
    pos += 1 + adlen;
  }
}

extern "C" void c_ble_init(void) {
  // REQUIRED before any other BLE call: this is what actually starts the BT
  // stack (bt_trace_init + ftl_init + bte_init). Without it beginCentral() spins
  // forever waiting for GAP_INIT_STATE_STACK_READY, which looks exactly like a
  // hung radio. It also blocks until Wi-Fi is up, which is why BLE is brought up
  // after wifi_on rather than beside it.
  // Size the advertising-report pool BEFORE the stack starts. The default is 16
  // buffers and the Arduino library never touches it; when the pool empties the
  // controller's reports are dropped on the floor, which on a busy channel looks
  // like poor RF rather than a full queue.
  gap_config_bt_report_buf_num(40);
  BLE.init();
  BLE.setScanCallback(ble_scan_cb);
  // Passive scan: we only want broadcast payloads, and an active scan would emit
  // scan requests — airtime spent to solicit data we do not use.
  BLE.configScan()->setScanMode(GAP_SCAN_MODE_PASSIVE);
  BLE.configScan()->setScanInterval(160); // fallback duty ~12%, same as the C5's
  BLE.configScan()->setScanWindow(20);    // (interval first: the setter validates against it)
  // Duplicate filtering ON, which is only correct because every sender in this fleet now rotates its
  // advertiser address per payload. Controllers dedup by ADDRESS: with rotation, dedup collapses the
  // multi-event burst of ONE payload (halving what crosses the 115200 UART) while never suppressing a
  // distinct payload — measured 44 reports for 20 payloads with it off, and all 20 still delivered
  // with it on. Against a peer that does NOT rotate, this would suppress real payloads; that is the
  // same trap the ESP32-C5's dedup set for us before the BW16 rotated.
  BLE.configScan()->setScanDuplicateFilter(true);
  BLE.configAdvert()->setAdvType(GAP_ADTYPE_ADV_NONCONN_IND); // connectionless broadcast
  // Advertise from a RANDOM address so the per-payload rotation below takes effect;
  // with the default public type the controller would ignore the random address.
  {
    unsigned char local_type = GAP_LOCAL_ADDR_LE_RANDOM;
    le_adv_set_param(GAP_PARAM_ADV_LOCAL_ADDR_TYPE, sizeof(local_type), &local_type);
  }
  BLE.configAdvert()->setMinInterval(20);
  BLE.configAdvert()->setMaxInterval(30);
  // beginCentral brings the stack up with the SCAN parameters registered; the
  // advertising parameters are registered afterwards by hand. The two roles are
  // separate GAP state machines — the Arduino wrapper only refuses a second
  // begin*(), not advertising from a central-initialised stack. bt_coex_init()
  // and wifi_btcoex_set_bt_on() run inside beginCentral, so Wi-Fi/BT coexistence
  // is armed by the same call.
  BLE.beginCentral(0); // 0 connections: this bearer is connectionless broadcast only
  BLE.configAdvert()->updateAdvertParams();
  BLE.getLocalAddr(s_own_addr);
  BLE.configScan()->startScan();
  s_ble_ready = true;
}

extern "C" int c_ble_ready(void) { return s_ble_ready ? 1 : 0; }

// Copy out the controller's cached LE feature mask.
extern "C" void c_ble_features(unsigned char *out8) { memcpy(out8, gap_local_features, 8); }

// Raw HCI escape hatch — the BLE analogue of the Wi-Fi side's pkt_attrib probe.
extern "C" int c_ble_hci_raw(unsigned short opcode, unsigned char *params, unsigned short len) {
  if (!s_ble_ready) return -1;
  if (len > 255) return -2;
  // Via gap_vendor_cmd_req, not hci_send_cmd_msg directly: the former hands the
  // command to the BT task, so it serialises with the stack's own HCI traffic
  // instead of racing it from the Arduino task.
  return (int)gap_vendor_cmd_req(opcode, (unsigned char)len, params);
}

extern "C" void c_ble_adv_start(const unsigned char *payload, unsigned int len) {
  if (!s_ble_ready || len == 0 || len > BLE_MAXPAY) return;
  unsigned char ad[31];
  ad[0] = (unsigned char)(len + 3); // AD length: type + 2-byte company + payload
  ad[1] = 0xFF;                     // manufacturer specific data
  ad[2] = BLE_MAGIC_LO;
  ad[3] = BLE_MAGIC_HI;
  memcpy(ad + 4, payload, len);
  // Order matters, and setAdvData alone is NOT enough: it only stores into the
  // BLEAdvert member. The bytes reach the controller only via updateAdvertParams(),
  // which is what pushes GAP_PARAM_ADV_DATA down. Without it the radio advertises
  // an empty AD — the scanner sees a device and no payload, which looks exactly
  // like "BLE TX is broken". Stop first: GAP rejects an adv-data update while the
  // advertiser is running.
  BLE.configAdvert()->stopAdv();
  ble_rotate_addr(); // while stopped: the controller rejects an address change mid-advertise
  BLE.configAdvert()->setAdvData(ad, (unsigned char)(len + 4));
  BLE.configAdvert()->updateAdvertParams();
  BLE.configAdvert()->startAdv();
}

// Advertising interval (ms). GAP's own floor is 20 ms (0x0020, 0.625 ms/step) for
// undirected advertising, so this is the fastest legal repeat for the bearer.
extern "C" void c_ble_adv_interval(unsigned short min_ms, unsigned short max_ms) {
  if (!s_ble_ready) return;
  BLE.configAdvert()->setMinInterval(min_ms);
  BLE.configAdvert()->setMaxInterval(max_ms);
}

extern "C" void c_ble_adv_stop(void) {
  if (s_ble_ready) BLE.configAdvert()->stopAdv();
}

// The BLE<->Wi-Fi radio-time split, as a scan window/interval. Wire units are
// 0.625 ms (the HCI unit the host and the C5 both speak); the Arduino setters
// take milliseconds and convert internally, so undo that here rather than
// letting the same number mean two different durations on the two radios.
extern "C" void c_ble_coex(unsigned short win_units, unsigned short itvl_units) {
  if (!s_ble_ready) return;
  // Set the GAP parameters DIRECTLY in wire units (0.625 ms). The Arduino setters
  // take milliseconds and convert both ways with truncating integer division, so a
  // requested duty came back up to ~20% short at small windows — the cognition lever
  // would then be asking for one airtime split and getting another.
  if (itvl_units < 4) itvl_units = 4;
  if (win_units < 4) win_units = 4;
  if (win_units > itvl_units) win_units = itvl_units;
  BLE.configScan()->stopScan();
  le_scan_set_param(GAP_PARAM_SCAN_INTERVAL, sizeof(itvl_units), &itvl_units);
  le_scan_set_param(GAP_PARAM_SCAN_WINDOW, sizeof(win_units), &win_units);
  // le_scan_stop() is ASYNCHRONOUS — the GAP state reaches idle via a callback. Restarting
  // immediately is refused and leaves the scanner OFF, silently and permanently: the bearer keeps
  // accepting commands while receiving nothing. MEASURED before this delay existed: 11/20 adverts
  // heard before a coex change, 0/20 after, with the device's own advert counter frozen.
  // The wrapper's own startScan(duration) leaves the same 100 ms gap after stopping.
  for (int attempt = 0; attempt < 3; attempt++) {
    delay(150);
    BLE.configScan()->startScan();
    if (BLE.configScan()->scanInProgress()) return;
  }
}

// BLE advertising TX power. `tx_gain` indexes a controller gain table (NOT dBm);
// the vendor header's anchors are 0x06 = -10 dBm, 0x1A = 0 dBm, 0x23 = +4.5 dBm,
// i.e. ~0.5 dB per step — twice the Wi-Fi TXAGC step. Advertising-specific, and a
// separate register path from the Wi-Fi TXAGC, though both drive the one PA.
extern "C" int c_ble_set_tx_power(unsigned char gain) {
  if (!s_ble_ready) return -1;
  return (int)le_adv_set_tx_power(0, gain);
}

// Controller-side advert pre-filter: drop every advert that does not carry our
// company magic before it is queued to the GAP task. The BLE analogue of the
// Tier-0 pre-link drop on the Wi-Fi side — except this one runs even earlier, on
// the report path itself. `offset` is measured into the raw AD payload.
extern "C" int c_ble_scan_filter(int enable) {
  if (!s_ble_ready) return -1;
  unsigned char magic[3] = {0xFF, BLE_MAGIC_LO, BLE_MAGIC_HI};
  return (int)le_scan_info_filter(enable ? true : false, 1, 3, magic);
}

extern "C" int c_ble_scanning(void) {
  return (s_ble_ready && BLE.configScan()->scanInProgress()) ? 1 : 0;
}

extern "C" void c_ble_scan_restart(void) {
  if (!s_ble_ready) return;
  BLE.configScan()->stopScan();
  delay(150);
  BLE.configScan()->startScan();
}

// SDK promisc callback -> Rust
// The SDK hands the callback an `ieee80211_frame_info_t *` as `userdata` — the
// SDK's own promisc_callback_all reads `->rssi` from it. The per-frame RSSI was
// never missing from this SDK; the old trampoline simply threw the pointer away.
static void promisc_trampoline(unsigned char *buf, unsigned int len, void *ud) {
  signed char rssi = 0;
  unsigned char mrate = 0;
  if (ud) {
    rssi = ((ieee80211_frame_info_t *)ud)->rssi;
    // The blob writes ONE byte more than the shipped header declares: offset 32 is
    // HwRateToMRate(pattrib->data_rate) — the per-frame PHY rate as an MGN code.
    // Read it by explicit offset; there is no header field for it (the `type`
    // field that would occupy it is behind CONFIG_UNSUPPORT_PLCPHDR_RPT, which
    // this build does not define). It lies inside the caller's live stack frame,
    // so it is initialised storage, not an overread.
    mrate = ((const unsigned char *)ud)[32];
    rust_rxinfo_dump((const unsigned char *)ud, 48); // layout probe (see T_RXINFO)
  }
  rust_promisc_cb(buf, len, rssi, mrate);
}
extern "C" void c_wifi_set_promisc_enable(void) {
  wifi_set_promisc(RTW_PROMISC_ENABLE_2, promisc_trampoline, 1);
}

void setup() { rust_setup(); }
void loop() { rust_loop(); }
