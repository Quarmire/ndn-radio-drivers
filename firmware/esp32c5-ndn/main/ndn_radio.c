// ESP32-C5 named-data radio node — host-driven serial-bridge FrameIo (BW16 wire protocol).
//
// The C5 does the FoA primitive (raw 802.11 inject + promiscuous capture of ethertype 0x8624) and is
// driven over its native USB-Serial-JTAG by the host's `Bw16SerialBackend` UNCHANGED — same framing
// `[0x4E 0x44 type len_le16 payload]`: host→device T_INJECT (a full 802.11 frame to esp_wifi_80211_tx),
// T_CHANNEL/T_RATE/T_TXPOWER/T_BW40 (1-byte params);
// device→host T_RX = [rssi_i8][802.11 frame] for each 0x8624 frame received. Console
// logging is OFF (LOG level NONE) so the binary stream is clean, while the console stays bound to the
// USB-Serial-JTAG so esptool auto-reset keeps working (see sdkconfig.defaults).
//
// Relevance is decided by PARSING the NDN name (host-side, or off-host where a radio keeps up), not by
// an in-frame filter (retired — see firmware/NDR_MAC_SPEC.md). Every received 0x8624 frame crosses to
// the host; nothing is dropped pre-USB (parse-everywhere floor, so FN=0 by construction).
//
// IMPORTANT (host side): the C5's native USB-Serial-JTAG maps RTS→EN and DTR→GPIO9. A host that asserts
// RTS on open holds the chip in reset (silent, no TX); one that pulses DTR can latch the download strap.
// Drive it with `Bw16SerialBackend::open_no_reset` — never toggle RTS/DTR; the chip free-runs the app.
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_wifi.h"
#include "esp_bt.h"
#include "esp_private/esp_wifi_he_private.h"

// --- Blob-internal PHY state (libphy.a). `phy_param` is a global, exported 1080-byte data object; the
// public esp_wifi_set_max_tx_power only writes ONE byte of it ([0x04], a per-rate min() CEILING) and its
// real actuation quantum is 1 dBm. phy_param[0x12B] is a global TX-power OFFSET in 0.25 dB steps applied
// to the whole 32-entry gain ladder, and nothing in any linked library ever writes it — so a value we
// write is sticky across channel changes. This is the C5's analogue of the RTL8720DN's per-rate TXAGC.
extern uint8_t phy_param[];
extern void phy_wifi_set_tx_gain_new(uint16_t freq_mhz, int mode);
#define PHY_PARAM_MAX_TPW   0x04  // i8, 0.25 dBm — the ceiling esp_wifi_set_max_tx_power writes
#define PHY_PARAM_FREEZE    0x05  // non-zero => the blob skips recomputing the gain table
#define PHY_PARAM_TRIM_QDB  0x12B // i8, 0.25 dB — the fine offset
#define PHY_PARAM_FREQ_MHZ  0x120 // u16, current centre frequency
#include "esp_private/wifi.h"  // esp_wifi_internal_set_fix_rate — pin the injected-frame PHY rate
#include "esp_event.h"
#include "nvs_flash.h"
#include "driver/usb_serial_jtag.h"
#include "esp_timer.h" // esp_timer_get_time — the always-running monotonic µs clock we schedule against
#include "ndr_parse.h" // off-host NDN name parse (NDR_MAC_SPEC §6) — ndr_name_admits()
// ── BLE bearer (NimBLE) — one firmware, all bearers (the named radio is bearer-agnostic) ──
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/ble_hs.h"
#include "host/util/util.h"
#include "host/ble_gap.h"
#include "os/os_mbuf.h"

#define NDN_ETHERTYPE 0x8624
#define SYNC0 0x4E
#define SYNC1 0x44
#define T_INJECT     0x01
#define T_CHANNEL    0x02
#define T_TXPOWER    0x03
#define T_RATE       0x04
#define T_BW40       0x05
#define T_INJECT_ATTR 0x06
#define T_PREFIXES   0x07 // [n][u64_le prefix-hash]*n — off-host parse relevance set (FNV-1a-64 of each "/prefix"); empty = forward all (§6)
#define T_INJECT_AT  0x09 // [delay_us_le32][802.11 frame] — scheduled TX, delay from now
#define T_INJECT_ABS 0x0A // [target_us_le64][802.11 frame] — scheduled TX at an ABSOLUTE esp_timer µs (slot lease)
#define T_READCLOCK  0x0B // no payload — reply T_CLOCK with the current esp_timer (schedule clock)
#define T_CLOCK      0x85 // [esp_timer_us_le64] — reply to T_READCLOCK
#define T_LOG        0x84 // device status text — same type byte the RTL8720DN firmware uses
#define T_POWERIDX   0x89 // [requested_q][applied_q] — 0.25 dBm units; the applied value after
                          // the IDF's 11-step quantisation, so the host reports truth not intent
#define T_OCC        0x86 // [activity_count_le32] — periodic free-running channel-activity counter (occupancy)
#define T_RX         0x81 // [rssi_i8][802.11 frame] — used by the BW16; the C5 sends T_RX_TS instead
#define T_RX_TS      0x82 // [rssi_i8][noise_i8][rate_code][phy_flags][rx_ts_us_le32][802.11 frame]
                          // — RX + hardware µs stamp + per-frame PHY metadata (radiotap-equiv)
#define T_TXTIME     0x83 // [target_le64][actual_le64][tsf_le64] — scheduling error report for T_INJECT_AT
// BLE bearer messages (same wire protocol, distinct types so one firmware serves Wi-Fi + BLE at once):
#define T_BLE_ADV    0x30 // host→device: [payload] — advertise as a BLE 5 extended advertisement
#define T_COEX       0x31
#define T_TXTRIM     0x0F // [qdb_i8] — fine TX-power trim, 0.25 dB steps, on top of T_TXPOWER's ceiling
#define T_READSTATS  0x10 // no payload — reply T_HWSTATS
#define T_HWSTATS    0x8B
#define T_CSIRAW_ARM 0x13 // [n] — dump the raw per-subcarrier magnitudes of the next n CSI frames
#define T_CSIRAW     0x8E // [count][mag_q3 per subcarrier] — the reduction's own raw material
#define T_RXDUMP_ARM 0x12 // [n] — dump the first 64 bytes of the next n promiscuous frames
#define T_RXDUMP     0x8D // [sig_len_le16][first 64 bytes of the buffer handed to the sniffer callback]
#define T_CSI_CFG    0x11 // [enabled][every_nth] — channel-state sensing on/off + subsampling
#define T_CSI        0x8C // [n_le16][mean_rssi][mean_noise][subcarriers][spread_q3][16 bins][eph][flags]
                          // — the channel profile INTEGRATED over n_frames (a single frame's estimate
                          // is noise-dominated; see csi_cb) // hardware MAC/PHY receive counters: the frame-free half of channel activity
#define T_BLE_PHY    0x37 // [phy] advertising PHY: 1=LE 1M, 2=LE 2M, 3=LE Coded (S=8, long range)
#define T_BLE_TXPOWER 0x35 // [level] — BLE advertising TX power, esp_power_level_t 0..15
                           // (-24 dBm .. +20 dBm in 3 dB steps). Same type byte as the RTL8720DN's
                           // BLE power knob, so one host call drives either radio. // host→device: [scan_window_le16][scan_itvl_le16] — the BLE↔Wi-Fi radio-time split
#define T_BLE_RX     0x88 // device→host: [rssi_i8][addr6][payload] — a scanned advertisement carrying our magic
#define MAXFRAME 512
#define BLE_MAXPAY 240
#define BLE_ADV_MAGIC_LO 0x44 // manufacturer company id 0x4E44 ("ND"), LE on air — the BLE analog of 0x8624
#define BLE_ADV_MAGIC_HI 0x4E

// Blob-internal rate setters (libpp.a). The public esp_wifi_config_80211_tx[_rate] wrappers ESP_FAIL on
// the dual-band C5 because it boots in HE20 (phymode 6) and they refuse to set a rate in HE mode; these
// underlying setters, called with an explicit legacy/HT phymode, actuate the raw-injection rate. The
// tx_rate_config's rate+phymode reach the descriptor (offset 12 + the HE-vs-legacy branch) in the pp
// TX-build path. MEASURED on air: 11G/24M->24, 11G/54M->54, HT20/MCS7->65, HT20/MCS4->39.
extern int ic_set_80211_tx_rate(uint32_t ifx, uint32_t rate);
extern int ic_set_80211_tx_rate_config(uint32_t ifx, const wifi_tx_rate_config_t *cfg);

static uint8_t s_cur_chan = 1; // last T_CHANNEL, for deriving the OFDM phymode (2.4G->11G, 5G->11A)

static wifi_phy_mode_t phymode_for_rate(uint8_t rate, uint8_t chan) {
    if (rate >= 0x10) return WIFI_PHY_MODE_HT20;                 // MCS
    if (rate >= 0x08) return chan <= 14 ? WIFI_PHY_MODE_11G : WIFI_PHY_MODE_11A; // OFDM
    return WIFI_PHY_MODE_11B;                                    // CCK
}

// Fix the raw-injection TX rate. phymode==0 => auto-derive from the rate code + current band. dcm/ersu are
// the 802.11ax reach levers (HE Dual-Carrier Modulation, HE Extended-Range SU); only apply for phymode HE20.
static void set_fix_rate(uint8_t rate, uint8_t phymode, bool dcm, bool ersu) {
    wifi_phy_mode_t pm = phymode ? (wifi_phy_mode_t)phymode : phymode_for_rate(rate, s_cur_chan);
    ic_set_80211_tx_rate(WIFI_IF_STA, rate);
    wifi_tx_rate_config_t cfg = { .phymode = pm, .rate = (wifi_phy_rate_t)rate, .ersu = ersu, .dcm = dcm };
    ic_set_80211_tx_rate_config(WIFI_IF_STA, &cfg);
}
#define SCHED_MAX_DELAY_US 100000 // cap the busy-wait to ~1 slot period (a hw timer would avoid the spin)

typedef struct { int8_t rssi; int8_t noise; uint8_t rate_code; uint8_t flags; uint16_t len; uint32_t rxts; uint8_t buf[MAXFRAME]; } rxpkt_t;
static QueueHandle_t rxq;

// Off-host parse relevance (NDR_MAC_SPEC §6). Host-pushed FNV-1a-64 hashes of the node's registered
// /-joined prefixes; rx_cb parses each frame's NDN name and drops, before it crosses the USB-Serial-JTAG,
// any NAMED frame under none of them. Empty set = parse-everywhere floor (forward all). Written by the
// serial command task, read in rx_cb (ISR): a torn read at worst mis-gates one best-effort frame, so no lock.
#define MAX_PREFIXES 24
static uint64_t s_prefixes[MAX_PREFIXES];
static volatile uint8_t s_nprefixes = 0;

// Free-running channel-activity counter (every promiscuous frame of any type) — the occupancy proxy the
// host reads via T_OCC (read_channel_activity). Written in WiFi-task ctx, read in serial_tx_task.
static volatile uint32_t s_activity = 0;
// Every advertisement the scanner reports, before the company-magic filter — the BLE half of the
// demand signal the coex split is driven from, and the honest answer to "is the scanner alive?".
static volatile uint32_t s_ble_seen = 0;
/// Remaining raw promiscuous-buffer dumps to emit — the layout probe. Off by default.
/// Staged here rather than sent from rx_cb: that callback runs in interrupt context (it uses
/// xQueueSendFromISR), so it must not touch the blocking USB-Serial-JTAG writer.
static volatile uint32_t s_rxdump = 0;
static volatile uint32_t s_csiraw = 0;
static volatile bool s_csi_ready = false;
static uint8_t s_csi_buf[32];
static volatile uint16_t s_csi_len = 0;
static volatile bool s_csiraw_ready = false;
static uint8_t s_csiraw_buf[256 + 1];
static volatile uint16_t s_csiraw_len = 0;
static volatile bool s_rxdump_ready = false;
static uint8_t s_rxdump_buf[66];
static volatile uint16_t s_rxdump_len = 0;

// Promiscuous RX: queue each 0x8624 frame (+ RSSI) for the serial-TX task.
static void rx_cb(void *buf, wifi_promiscuous_pkt_type_t type) {
    s_activity++; // count ALL activity, before any filter
    const wifi_promiscuous_pkt_t *p = (wifi_promiscuous_pkt_t *)buf;
    const uint8_t *f = p->payload;
    int len = p->rx_ctrl.sig_len;
    // Probe BEFORE the packet-type check: the type a frame is delivered under is itself something to
    // measure, not assume (it changes when CSI is enabled).
    if (s_rxdump && !s_rxdump_ready && len >= 8) {
        s_rxdump--;
        s_rxdump_buf[0] = (uint8_t)type;
        s_rxdump_buf[1] = (uint8_t)(len & 0xff);
        uint16_t n = len < 63 ? (uint16_t)len : 63;
        memcpy(s_rxdump_buf + 2, f, n);
        s_rxdump_len = 2 + n;
        s_rxdump_ready = true;
    }
    // Do NOT gate on wifi_promiscuous_pkt_type_t. MEASURED: enabling CSI makes the sniffer deliver
    // the very same frame — byte-identical, LLC/SNAP still at 24, ethertype still at 30 — as
    // WIFI_PKT_MGMT instead of WIFI_PKT_DATA, so this gate silently dropped every named frame the
    // moment channel sensing was switched on (30/30 forwarded before, 0/30 after, recovering on
    // disable). The 802.11 Frame Control field is the authoritative answer and does not move.
    (void)type;
    if ((f[0] & 0x0C) != 0x08) return; // type=data in the frame itself
    // Layout probe: the raw bytes the sniffer handed us, BEFORE any offset assumption. Armed on
    // demand, because "where does the 802.11 header start" turned out to be a question that has a
    // different answer with CSI enabled than without.
    if (len < 32 || len > MAXFRAME) return;
    if (!(f[24] == 0xaa && f[25] == 0xaa && f[26] == 0x03)) return;
    if ((((uint16_t)f[30] << 8) | f[31]) != NDN_ETHERTYPE) return;
    // Off-host parse relevance (§6): parse the carried NDN name and drop a NAMED frame under none of
    // the host-registered prefixes, before it crosses the link. No wire cost (the name is already in
    // the frame). Fail-open on an unparseable/nameless frame (kind==NONE) so control frames survive
    // (H1: never drop a frame that was for you). s_nprefixes==0 => parse-everywhere floor.
    if (s_nprefixes > 0) {
        uint8_t kind = 0;
        if (!ndr_name_admits(f + 32, (uint32_t)(len - 32), s_prefixes, s_nprefixes, &kind)
            && kind != NDR_KIND_NONE) return;
    }
    rxpkt_t pk;
    pk.rssi = p->rx_ctrl.rssi;
    pk.noise = p->rx_ctrl.noise_floor;              // → host SNR = rssi - noise
    uint8_t fmt = p->rx_ctrl.cur_bb_format;         // 0=11B 1=11G/A 2=HT 3=VHT 4+=HE
    // The MCS sits in DIFFERENT bits per PHY format, and treating them alike was a real instrument
    // fault: for an HE SU PPDU the MCS is he_siga1[6:3], while [2:0] are format/beam-change/UL-DL — so
    // the old `he_siga1 & 0x7f` reported an HE MCS multiplied by 8 plus flag bits. Worst-receiver rate
    // adaptation consumes this value, so it was driving the rate ladder from a corrupted observation.
    if (fmt >= 4) {
        pk.rate_code = (uint8_t)((p->rx_ctrl.he_siga1 >> 3) & 0x0f); // HE-SIGA1: MCS in [6:3]
    } else if (fmt >= 2) {
        pk.rate_code = (uint8_t)(p->rx_ctrl.he_siga1 & 0x7f);       // HT-SIG / VHT-SIG
    } else {
        pk.rate_code = (uint8_t)p->rx_ctrl.rate;                    // legacy L-SIG
    }
    // flags: [3:0] PHY format, bit4 = the transmitted bandwidth. For HT that is HT-SIG's CBW bit
    // (he_siga1 bit 7); for HE it is he_siga1[20:19]. This is the free, no-SDR oracle for "did that
    // frame actually go out at 40 MHz" — the question T_BW40 has never been able to answer.
    uint8_t bw40 = 0;
    if (fmt == 2) bw40 = (p->rx_ctrl.he_siga1 >> 7) & 1;
    else if (fmt >= 4) bw40 = ((p->rx_ctrl.he_siga1 >> 19) & 3) ? 1 : 0;
    pk.flags = (fmt & 0x0f) | (bw40 << 4);
    pk.len = len;
    pk.rxts = p->rx_ctrl.timestamp;   // hardware per-frame RX stamp (µs, same domain as esp_timer)
    memcpy(pk.buf, f, len);
    BaseType_t hp = pdFALSE;
    xQueueSendFromISR(rxq, &pk, &hp); // drops if full — fine, promiscuous is best-effort
    if (hp) portYIELD_FROM_ISR();
}

// ONE driver call per message. Writing the header and payload separately cost two mutex takes, two
// ringbuffer sends and two interrupt-enable round trips through the USB-Serial-JTAG driver for every
// frame forwarded — pure overhead on the link that is already this bearer's tightest bottleneck.
static void send_framed(uint8_t ty, const uint8_t *payload, uint16_t len) {
    static uint8_t buf[8 + MAXFRAME + 8];
    if (len > sizeof(buf) - 5) return;
    buf[0] = SYNC0; buf[1] = SYNC1; buf[2] = ty;
    buf[3] = (uint8_t)(len & 0xff); buf[4] = (uint8_t)(len >> 8);
    if (len) memcpy(buf + 5, payload, len);
    usb_serial_jtag_write_bytes(buf, 5 + len, portMAX_DELAY);
}

// ── Channel State Information: the per-frame channel response, reduced on-device ──
//
// CSI turns each received frame from two scalars (rssi, noise) into a ~245-point complex channel
// response. Raw, that is 490 bytes per frame at HE20 — about 4x a small named-data frame, and at the
// measured ~4.6 Mbit/s host link it would consume the entire link. So it is reduced HERE, in the same
// place and for the same reason as the Tier-0 name filter: what crosses the link should be the answer,
// not the raw material.
//
// The summary is chosen for what cognition actually asks of a channel:
//   * mean magnitude          — the channel gain, independent of the packet's own rssi scaling;
//   * magnitude spread        — frequency selectivity, i.e. multipath / coherence bandwidth, which is
//                               the physical quantity behind whether coding or a lower rate helps;
//   * notched subcarrier count — a narrowband dip, which separates a co-channel interferer from a
//                               flat fade. A single RSSI cannot tell those apart at all.
static volatile bool s_csi_on = false;
static volatile uint8_t s_csi_every = 1;   // subsample: report 1 frame in N
static volatile uint32_t s_csi_seen = 0;

// 8*log2(v), saturating — a coarse log scale so a whole subcarrier band's power fits one byte with
// ~0.38 dB resolution (one log2 unit is 3.01 dB, and we keep 3 fractional bits).
static inline uint8_t log2_q3(uint32_t v) {
    if (v == 0) return 0;
    int msb = 31 - __builtin_clz(v);
    uint32_t frac = (msb >= 3) ? ((v >> (msb - 3)) & 0x7u) : ((v << (3 - msb)) & 0x7u);
    uint32_t r = (uint32_t)msb * 8u + frac;
    return r > 255u ? 255u : (uint8_t)r;
}

// Per-subcarrier accumulator. MEASURED, and it is the whole reason this integrates rather than
// reporting per frame: on a static bench link a SINGLE frame's channel estimate spans 28.4 dB across
// subcarriers, while the same link averaged over ~100 frames spans 6.5 dB and reproduces to 0.85 dB
// RMS between runs. Two static radios cannot have a channel that reshapes between consecutive frames,
// so the per-frame spread is estimation noise — the int8 I/Q of a single L-LTF symbol — and only the
// average is the channel. A per-frame "notched subcarrier" count therefore counts noise, which is
// exactly the mistake the first version of this made.
static uint32_t s_csi_acc[256];
static uint16_t s_csi_n = 0;
static int32_t s_csi_rssi_acc = 0;
static int32_t s_csi_noise_acc = 0;
static uint8_t s_csi_sc = 0;
// Which SENDER this profile is about, as the 8-bit **ephemeral ID** at `addr3[4]` — 802.11 header
// offset 20.
//
// NOT the source address. This MAC has no host addressing: under the Blurred Name wire format the
// address octets `addr1 ‖ addr2 ‖ addr3[0..4]` carry the prefix-set Bloom filter, so "the source MAC"
// is *name* bits, and grouping by it would group frames by which prefixes they carry rather than by who
// sent them. The 128:8 partition leaves exactly one field that identifies a sender — the ephemeral ID —
// and it is deliberately narrow, rotating and soft-state.
//
// An 8-bit ID aliases (birthday bound ~19 neighbours), and that is acceptable here for the same reason
// the ID module gives for RSSI: a residual alias blends two channels into one profile, degrading an
// estimate — it never costs a delivery. Cooperative deconfliction (PFS/DAR) shrinks the window anyway.
#define NDR_EPHID_OFF   20  // addr3[4]
#define NDR_FLAGS_OFF   21  // addr3[5]
static uint8_t s_csi_eph = 0;
static uint8_t s_csi_flags = 0;
static uint8_t s_csi_filter_eph = 0;
static bool s_csi_filter_on = false;
// When the open window last took a frame. A window is pinned to one sender, so without a deadline a
// sender that goes silent mid-window strands the accumulator and NO further profile is ever emitted —
// the sensor dies quietly, which is the failure mode this codebase keeps being bitten by.
static int64_t s_csi_win_us = 0;
#define CSI_WINDOW_TIMEOUT_US 3000000

static void csi_cb(void *ctx, wifi_csi_info_t *info) {
    if (!info || !info->buf || info->len < 8) return;
    // The buffer points INTO the live hardware RX descriptor and is gone when this returns, so the
    // accumulation must happen synchronously here — never queue the pointer.
    const int8_t *b = (const int8_t *)info->buf;
    int n = info->len / 2;                      // (imag, real) int8 pairs
    if (n > 256) n = 256;
    if (n < 8) return;
    if (!info->hdr) return;                     // no header: cannot attribute the measurement
    // Integrate only OUR frames. `addr3[4]` is the ephemeral ID on an NDR frame, but on ordinary
    // ambient traffic it is just a byte of somebody's MAC address — so without this check the room's
    // Wi-Fi manufactures spurious "senders" and pollutes the profiles. Same LLC/SNAP + ethertype test
    // the promiscuous path uses.
    const uint8_t *h = info->hdr;
    if (!(h[24] == 0xaa && h[25] == 0xaa && h[26] == 0x03)) return;
    if ((((uint16_t)h[30] << 8) | h[31]) != NDN_ETHERTYPE) return;
    uint8_t eph = info->hdr[NDR_EPHID_OFF];
    if (s_csi_filter_on && eph != s_csi_filter_eph) return;
    // Abandon a stale or format-changed window BEFORE the sender gate, so neither can strand it.
    int64_t now_us = esp_timer_get_time();
    bool stale = s_csi_n && (now_us - s_csi_win_us > CSI_WINDOW_TIMEOUT_US);
    bool reshaped = s_csi_n && s_csi_sc && n != (int)s_csi_sc;
    if (stale || reshaped) {
        s_csi_n = 0;
        s_csi_eph = 0;
        for (int i = 0; i < 256; i++) s_csi_acc[i] = 0;
        s_csi_rssi_acc = s_csi_noise_acc = 0;
    }
    if (s_csi_n == 0) {
        s_csi_eph = eph;                        // the first frame of a window fixes whose channel this is
        s_csi_flags = info->hdr[NDR_FLAGS_OFF];
        s_csi_win_us = now_us;
    } else if (eph != s_csi_eph) {
        return;                                 // a different sender: not this link's channel
    }
    s_csi_win_us = now_us;
    s_csi_sc = (uint8_t)n;
    for (int i = 0; i < n; i++) {
        int im = b[2 * i], re = b[2 * i + 1];
        s_csi_acc[i] += (uint32_t)(im * im + re * re);   // |H|^2 avoids a sqrt in the RX path
    }
    s_csi_rssi_acc += info->rx_ctrl.rssi;
    s_csi_noise_acc += info->rx_ctrl.noise_floor;
    s_csi_n++;
    if (s_csi_n < s_csi_every || s_csi_ready) return;   // integrate until the window is full

    // Emit the AVERAGED profile: 16 bins of log power, which is meaningful because it is an average.
    uint8_t *o = s_csi_buf;
    o[0] = (uint8_t)(s_csi_n & 0xff); o[1] = (uint8_t)(s_csi_n >> 8);
    o[2] = (uint8_t)(int8_t)(s_csi_rssi_acc / s_csi_n);
    o[3] = (uint8_t)(int8_t)(s_csi_noise_acc / s_csi_n);
    o[4] = (uint8_t)n;
    uint8_t mn = 255, mx = 0;
    for (int k = 0; k < 16; k++) {
        int lo = (n * k) / 16, hi = (n * (k + 1)) / 16;
        uint32_t acc = 0; int cnt = 0;
        for (int i = lo; i < hi; i++) {
            if (s_csi_acc[i] == 0) continue;    // skip structural nulls (the band edge reads zero)
            acc += s_csi_acc[i] / s_csi_n; cnt++;
        }
        uint8_t v = cnt ? log2_q3(acc / (uint32_t)cnt) : 0;
        o[6 + k] = v;
        if (v) { if (v < mn) mn = v; if (v > mx) mx = v; }
    }
    o[5] = (mx >= mn) ? (uint8_t)(mx - mn) : 0;  // spread of the AVERAGED profile, in 0.376 dB units
    o[22] = s_csi_eph;                           // whose channel this profile describes
    o[23] = s_csi_flags;
    s_csi_len = 24;
    s_csi_ready = true;
    s_csi_n = 0;
    s_csi_eph = 0;
    for (int i = 0; i < 256; i++) s_csi_acc[i] = 0;
    s_csi_rssi_acc = s_csi_noise_acc = 0;
}

// Order matters and every step is checked: the per-format acquisition bits only reach the hardware
// through esp_wifi_set_csi_config, and esp_wifi_set_csi returns ESP_FAIL silently if CSI was compiled
// out — the same unchecked-return defect class as the TX-power bug.
static int csi_enable(bool on, uint8_t every) {
    s_csi_every = every ? every : 32;  // frames to integrate before reporting a profile
    if (!on) { s_csi_on = false; return esp_wifi_set_csi(false); }
    esp_err_t e = esp_wifi_set_csi_rx_cb(csi_cb, NULL);
    if (e != ESP_OK) return e;
    // Every acquisition format is opt-in. A zeroed config enables CSI and acquires NOTHING — the
    // callback simply never fires, which looks exactly like a broken sensor. Turn on the formats this
    // bearer actually receives: legacy OFDM, HT20 and HE20 SU. (11b is skipped: we never transmit it,
    // and the sniffer path does not deliver CSI for 11b PPDUs anyway.)
    wifi_csi_config_t cfg = { 0 };
    cfg.enable = 1;
    cfg.acquire_csi_legacy = 1;
    cfg.acquire_csi_force_lltf = 1;
    cfg.acquire_csi_ht20 = 1;
    cfg.acquire_csi_su = 1;
    cfg.acquire_csi_dcm = 1;
    cfg.acquire_csi_beamformed = 1;
    cfg.val_scale_cfg = 0;
    e = esp_wifi_set_csi_config(&cfg);
    if (e != ESP_OK) return e;
    e = esp_wifi_set_csi(true);
    if (e == ESP_OK) s_csi_on = true;
    return e;
}

// Drain the RX queue → serial as T_RX_TS [rssi][rx_ts_us_le32][frame]: every C5 frame carries the
// hardware per-frame RX timestamp, so the host can stamp it in the device's clock domain (common-view /
// frame-age) instead of the coarse host-recv time. The 802.11 TSF is 0 unassociated, so this µs stamp
// (rx_ctrl.timestamp, same domain as the esp_timer we schedule TX on) is the C5's real link clock.
// ── BLE bearer (NimBLE ext-adv + scan) — the second bearer of the one unified firmware ──
static uint8_t s_ble_addr_type;
static uint8_t s_ble_addr[6];
static QueueHandle_t bleq;          // scanned reports -> serial-TX task
static QueueHandle_t advq;          // pending advertisements -> paced adv task
static volatile bool s_ble_ready;
typedef struct { int8_t rssi; uint8_t addr[6]; uint8_t len; uint8_t buf[BLE_MAXPAY]; } blerep_t;
typedef struct { uint8_t len; uint8_t buf[BLE_MAXPAY]; } advreq_t;

// Scan an adv payload for our manufacturer AD (0xFF, company 0x4E44); return the inner named payload len.
static int ble_find_named(const uint8_t *d, int n, const uint8_t **out) {
    int i = 0;
    while (i + 2 <= n) {
        int adlen = d[i];
        if (adlen < 1 || i + 1 + adlen > n) break;
        if (d[i + 1] == 0xFF && adlen >= 3 && d[i + 2] == BLE_ADV_MAGIC_LO && d[i + 3] == BLE_ADV_MAGIC_HI) {
            *out = &d[i + 4];
            return adlen - 3;
        }
        i += 1 + adlen;
    }
    return 0;
}

static int ble_gap_event(struct ble_gap_event *ev, void *arg) {
    if (ev->type == BLE_GAP_EVENT_EXT_DISC) {
        s_ble_seen++; // count ALL adverts, before the magic filter
        const struct ble_gap_ext_disc_desc *dsc = &ev->ext_disc;
        if (memcmp(dsc->addr.val, s_ble_addr, 6) == 0) return 0; // ignore our own reflected ads
        const uint8_t *pay = NULL;
        int plen = ble_find_named(dsc->data, dsc->length_data, &pay);
        if (plen > 0 && plen <= BLE_MAXPAY) {
            blerep_t r;
            r.rssi = dsc->rssi;
            memcpy(r.addr, dsc->addr.val, 6);
            r.len = (uint8_t)plen;
            memcpy(r.buf, pay, plen);
            BaseType_t hp = pdFALSE;
            xQueueSendFromISR(bleq, &r, &hp);
            if (hp) portYIELD_FROM_ISR();
        }
    }
    return 0;
}

// Legacy advertising carries 31 AD bytes, of which our manufacturer envelope takes 4.
#define BLE_LEGACY_MAXPAY 27

// Which PDU the advertising instance is currently configured for; -1 = not yet configured.
static int s_adv_legacy = -1;

// The advertising PHY — a REACH lever, and the one BLE knob that trades range against reachability
// rather than against airtime.
//
// LE Coded (S=8) buys roughly 2-4x range for the same power with no receiver feedback, which is the
// property a feedback-free broadcast bearer values most. But it is a genuine trade, not a free upgrade:
// coded advertising requires EXTENDED PDUs, so a legacy-only receiver (the RTL8720DN peer, whose
// controller has no extended advertising at all) cannot hear a coded advert — at any range. That is why
// this is exposed as a cognition-selectable PHY rather than switched on: the choice depends on who the
// neighbours are, which is exactly the worst-receiver question the Wi-Fi side already answers.
static uint8_t s_adv_phy = BLE_HCI_LE_PHY_1M;

// Reconfigure the advertising instance for legacy or extended PDUs.
//
// WHY THIS EXISTS: an extended advertisement is invisible to a legacy-only scanner — it lives on the
// secondary channels behind an AUX pointer, which such a receiver cannot follow. MEASURED: an
// RTL8720DN (BW16) peer heard 0 of 20 of this radio's extended adverts, and 20 of 20 legacy ones.
// And the BW16's controller genuinely lacks LE Extended Advertising (its LE feature mask reads
// 3d 01 .., bit 12 clear) — so this is not something the peer can be upgraded into.
//
// This is the BLE form of the rule the Wi-Fi side already follows: transmit at the most widely
// decodable encoding that can still carry the payload, because a broadcast with no receiver feedback
// cannot discover that it was unheard. Extended PDUs are used only when the payload genuinely needs
// them.
// Give each advertised payload a fresh non-resolvable random address.
//
// Two reasons, and they agree. (1) DOCTRINE: a named-data radio carries no host identity — the source
// is an ephemeral nonce — so one BD address for the device's lifetime is exactly the persistent
// identifier the design forbids. (2) MEASURED on the sibling radio: BLE controllers duplicate-filter by
// ADVERTISER ADDRESS, so a fixed address makes a receiver hear the first advert and suppress the rest
// until its scan period resets — 2 of 20 rapid payloads delivered, versus 20 of 20 once the sender
// rotated per payload. Rotating per payload keeps dedup doing the one thing it should: collapsing the
// 3-event burst of a SINGLE payload, while never suppressing a distinct one.
static void ble_rotate_addr(void) {
    ble_addr_t a;
    if (ble_hs_id_gen_rnd(1 /* nrpa: non-resolvable */, &a) != 0) return;
    // Per-instance: an extended advertising set carries its own address, so setting the global
    // identity would not change what this set puts on air.
    if (ble_gap_ext_adv_set_addr(0, &a) == 0) memcpy(s_ble_addr, a.val, 6);
}

static int ble_set_pdu_mode(bool legacy) {
    if (s_adv_legacy == (int)legacy) return 0;
    struct ble_gap_ext_adv_params p;
    memset(&p, 0, sizeof(p));
    p.own_addr_type = BLE_OWN_ADDR_RANDOM; // required for the per-payload rotation below to take
    p.legacy_pdu = legacy;   // non-connectable + non-scannable + legacy => ADV_NONCONN_IND
    if (legacy) {
        // Legacy PDUs are 1M by specification; the PHY fields are ignored (NimBLE forces them).
        p.primary_phy = BLE_HCI_LE_PHY_1M;
        p.secondary_phy = BLE_HCI_LE_PHY_1M;
    } else {
        // The primary PHY carries the advertising indication itself, so it is what determines whether a
        // scanner on the coded PHY can find us at all; the secondary carries the payload.
        p.primary_phy = (s_adv_phy == BLE_HCI_LE_PHY_CODED) ? BLE_HCI_LE_PHY_CODED : BLE_HCI_LE_PHY_1M;
        p.secondary_phy = s_adv_phy;
    }
    p.itvl_min = 0x30;
    p.itvl_max = 0x30;
    p.tx_power = 127;
    int8_t sel;
    int rc = ble_gap_ext_adv_configure(0, &p, &sel, ble_gap_event, NULL);
    if (rc != 0) {
        // Reconfiguring an instance NimBLE already knows about can be refused; drop it and retry.
        ble_gap_ext_adv_remove(0);
        rc = ble_gap_ext_adv_configure(0, &p, &sel, ble_gap_event, NULL);
    }
    if (rc == 0) s_adv_legacy = legacy;
    { char b[40]; int n = snprintf(b, sizeof b, "pdu legacy=%d rc=%d", (int)legacy, rc);
      send_framed(T_LOG, (const uint8_t *)b, n); }
    return rc;
}

// T_BLE_ADV: wrap the host payload in our manufacturer AD and burst-advertise it (fire-and-forget).
static void ble_advertise(const uint8_t *payload, int len) {
    if (!s_ble_ready || len <= 0 || len > BLE_MAXPAY) return;
    struct os_mbuf *m = os_msys_get_pkthdr(len + 4, 0);
    if (!m) return;
    uint8_t hdr[4] = { (uint8_t)(len + 3), 0xFF, BLE_ADV_MAGIC_LO, BLE_ADV_MAGIC_HI };
    os_mbuf_append(m, hdr, 4);
    os_mbuf_append(m, payload, len);
    ble_gap_ext_adv_stop(0);
    // Reach every receiver when the payload allows it; spend the extended PDU only when it must.
    // Legacy PDUs are 1M-only, so asking for 2M/Coded forces the extended form regardless of size.
    ble_set_pdu_mode(s_adv_phy == BLE_HCI_LE_PHY_1M && len <= BLE_LEGACY_MAXPAY);
    ble_rotate_addr(); // while stopped — the address of a running advertising set cannot be changed
    int rcd = ble_gap_ext_adv_set_data(0, m);
    int rcs = -1;
    if (rcd == 0) rcs = ble_gap_ext_adv_start(0, 0, 3);
    else os_mbuf_free_chain(m);
    if (rcd != 0 || rcs != 0) {
        char b[48]; int n = snprintf(b, sizeof b, "adv set=%d start=%d len=%d", rcd, rcs, len);
        send_framed(T_LOG, (const uint8_t *)b, n);
    }
}

// BLE↔Wi-Fi radio-time split, as a scan window/interval. This is NOT a fixed MAC parameter — it is the
// airtime allocation between the two bearers of the one radio, which the NDR MAC allocates by measured
// demand (host/cognition drives it via T_COEX). The boot values are only a **fallback** so the radio comes
// up balanced before cognition speaks; a fully-open scan (window==itvl) starves the promiscuous Wi-Fi RX,
// so the fallback duty-cycles (~12%). Both bearers report activity (T_OCC = Wi-Fi frames; BLE hit rate is
// observable at the host) so the split can track which bearer actually has named traffic.
static uint16_t s_ble_scan_win = 0x20;   // fallback: 20ms window …
static uint16_t s_ble_scan_itvl = 0x100; // … / 160ms interval

static void ble_start_scan(void) {
    ble_gap_disc_cancel(); // no-op if not scanning
    struct ble_gap_ext_disc_params up;
    memset(&up, 0, sizeof(up));
    up.itvl = s_ble_scan_itvl;
    up.window = s_ble_scan_win;
    up.passive = 1;
    // filter_duplicates=1: the 3-event adv burst re-sends each fragment, and the NDNts reassembler would
    // append the duplicate continuations and corrupt the packet — dedup them at the controller. A scan
    // PERIOD (2 * 1.28s) resets the dedup list so a re-broadcast can still recover a lost fragment.
    // Scan the CODED primary PHY as well as the uncoded one. Passing coded_params = NULL (as this did)
    // means a coded advert is never even looked for — the bearer would be able to transmit long-range
    // and structurally unable to receive it.
    ble_gap_ext_disc(s_ble_addr_type, 0, 2 /*period*/, 1 /*filter_dup*/, 0, 0, &up, &up,
                     ble_gap_event, NULL);
}

static void ble_on_sync(void) {
    ble_hs_util_ensure_addr(0);
    ble_hs_id_infer_auto(0, &s_ble_addr_type);
    ble_hs_id_copy_addr(s_ble_addr_type, s_ble_addr, NULL);
    ble_set_pdu_mode(true); // start legacy: reachable by every scanner, incl. legacy-only peers
    ble_start_scan();
    s_ble_ready = true;
}

static void ble_host_task(void *param) {
    nimble_port_run();
    nimble_port_freertos_deinit();
}

// Paced advertiser: each queued payload (e.g. one LP fragment) gets its full burst on air before the next
// replaces it. Without this, back-to-back T_BLE_ADV (a multi-fragment packet) overwrite the adv data before
// it radiates and only the last fragment is transmitted — reassembly then never completes.
static void ble_adv_task(void *arg) {
    advreq_t req;
    for (;;) {
        if (xQueueReceive(advq, &req, portMAX_DELAY) == pdTRUE) {
            ble_advertise(req.buf, req.len);
            vTaskDelay(pdMS_TO_TICKS(110)); // 3 adv events @ 30ms ≈ 90ms — let the burst radiate
        }
    }
}

static void serial_tx_task(void *arg) {
    static uint8_t out[8 + MAXFRAME];
    static uint8_t bout[7 + BLE_MAXPAY];
    rxpkt_t pk;
    blerep_t br;
    int64_t last_occ = esp_timer_get_time();
    for (;;) {
        // 100 ms timeout so the loop wakes to emit T_OCC / drain BLE even when no Wi-Fi frames are queued.
        if (xQueueReceive(rxq, &pk, pdMS_TO_TICKS(100)) == pdTRUE) {
            out[0] = (uint8_t)pk.rssi;
            out[1] = (uint8_t)pk.noise;      // noise floor (dBm)
            out[2] = pk.rate_code;           // legacy rate or MCS (per flags sig_mode)
            out[3] = pk.flags;               // sig_mode(0-1) | sgi(2) | cwb40(3)
            for (int k = 0; k < 4; k++) out[4 + k] = (uint8_t)(pk.rxts >> (8 * k));
            memcpy(out + 8, pk.buf, pk.len);
            send_framed(T_RX_TS, out, pk.len + 8);
        }
        // Drain scanned BLE advertisements -> T_BLE_RX [rssi][addr6][payload].
        while (bleq && xQueueReceive(bleq, &br, 0) == pdTRUE) {
            bout[0] = (uint8_t)br.rssi;
            memcpy(bout + 1, br.addr, 6);
            memcpy(bout + 7, br.buf, br.len);
            send_framed(T_BLE_RX, bout, 7 + br.len);
        }
        if (s_csi_ready) {
            send_framed(T_CSI, s_csi_buf, s_csi_len);
            s_csi_ready = false;
        }
        if (s_csiraw_ready) {
            send_framed(T_CSIRAW, s_csiraw_buf, s_csiraw_len);
            s_csiraw_ready = false;
        }
        if (s_rxdump_ready) {
            send_framed(T_RXDUMP, s_rxdump_buf, s_rxdump_len);
            s_rxdump_ready = false;
        }
        int64_t now = esp_timer_get_time();
        if (now - last_occ >= 200000) { // ~5×/s: emit the free-running activity counter
            last_occ = now;
            // [wifi_activity_le32][ble_adv_seen_le32] — both bearers' raw activity, so the coex
            // split can be driven from measured demand. The host reads only the first word unless it
            // wants the second, so the extension stays backward compatible.
            uint32_t a = s_activity, bl = s_ble_seen;
            uint8_t c[8];
            for (int k = 0; k < 4; k++) c[k] = (uint8_t)(a >> (8 * k));
            for (int k = 0; k < 4; k++) c[4 + k] = (uint8_t)(bl >> (8 * k));
            send_framed(T_OCC, c, 8);
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
                case T_BLE_ADV: if (advq && len > 0 && len <= BLE_MAXPAY) { // enqueue -> paced adv task
                    advreq_t rq; rq.len = (uint8_t)len; memcpy(rq.buf, pl, len);
                    xQueueSend(advq, &rq, 0); // drop if full (broadcast is best-effort)
                } break;
                case T_COEX: if (len >= 4 && s_ble_ready) { // cognition sets the BLE↔Wi-Fi radio-time split
                    s_ble_scan_win = pl[0] | (pl[1] << 8);
                    s_ble_scan_itvl = pl[2] | (pl[3] << 8);
                    if (s_ble_scan_itvl < s_ble_scan_win) s_ble_scan_itvl = s_ble_scan_win; // window ≤ interval
                    ble_start_scan();
                } break;
                case T_CHANNEL: if (len >= 1) { esp_wifi_set_channel(pl[0], WIFI_SECOND_CHAN_NONE); s_cur_chan = pl[0]; } break;
                case T_BLE_PHY:
                    // 1 = LE 1M (universal), 2 = LE 2M, 3 = LE Coded S=8 (long range).
                    // Takes effect on the next advertisement, which reconfigures the instance.
                    if (len >= 1 && pl[0] >= 1 && pl[0] <= 3) {
                        s_adv_phy = pl[0];
                        s_adv_legacy = -1; // force a reconfigure so the new PHY is pushed down
                        char b[40];
                        int n = snprintf(b, sizeof b, "adv phy=%u", (unsigned)s_adv_phy);
                        send_framed(T_LOG, (const uint8_t *)b, n);
                    }
                    break;
                case T_BLE_TXPOWER:
                    // esp_power_level_t: 0 = -24 dBm .. 15 = +20 dBm, 3 dB per step. Applies to the
                    // advertising path specifically (ESP_BLE_PWR_TYPE_ADV), which is the only BLE TX
                    // this bearer does. Default is P3 (+3 dBm), so there is headroom in both
                    // directions — this is a genuine reach/airtime lever, not just an attenuator.
                    if (len >= 1) {
                        esp_power_level_t lvl = (esp_power_level_t)(pl[0] > 15 ? 15 : pl[0]);
                        esp_ble_tx_power_set(ESP_BLE_PWR_TYPE_ADV, lvl);
                    }
                    break;
                case T_TXTRIM:
                    // Signed 0.25 dB offset applied to the entire TX gain ladder. Negative backs off;
                    // 0 restores the calibrated default. Recompute is needed for the write to reach the
                    // hardware, and it briefly drops TX and RX (phy_force_txrx_off around the update) —
                    // so this is a per-power-change knob, never a per-frame one.
                    if (len >= 1 && phy_param[PHY_PARAM_FREEZE] == 0) {
                        phy_param[PHY_PARAM_TRIM_QDB] = pl[0];
                        uint16_t f = (uint16_t)(phy_param[PHY_PARAM_FREQ_MHZ] |
                                                (phy_param[PHY_PARAM_FREQ_MHZ + 1] << 8));
                        if (f) phy_wifi_set_tx_gain_new(f, 0);
                        uint8_t r[4] = { pl[0], phy_param[PHY_PARAM_TRIM_QDB],
                                         (uint8_t)(f & 0xff), (uint8_t)(f >> 8) };
                        send_framed(T_POWERIDX, r, 4);
                    }
                    break;
                case T_CSIRAW_ARM:
                    if (len >= 1) s_csiraw = pl[0];
                    break;
                case T_RXDUMP_ARM:
                    if (len >= 1) s_rxdump = pl[0];
                    break;
                case T_CSI_CFG:
                    if (len >= 1) {
                        // [on][integrate]([ephemeral_id]) — the optional ID pins the profile to one sender.
                        s_csi_filter_on = (len >= 3);
                        if (s_csi_filter_on) s_csi_filter_eph = pl[2];
                        int rc = csi_enable(pl[0] != 0, len >= 2 ? pl[1] : 1);
                        char b[40];
                        int n = snprintf(b, sizeof b, "csi on=%d rc=%d", (int)(pl[0] != 0), rc);
                        send_framed(T_LOG, (const uint8_t *)b, n);
                    }
                    break;
                case T_READSTATS: {
                    // The counters the promiscuous callback cannot see: frames whose preamble the PHY
                    // locked but whose payload failed, and energy that never became a frame at all. Our
                    // occupancy counter counts only clean receptions, so it structurally under-reports a
                    // busy channel — this is the missing half, and it needs no airtime to collect.
                    esp_test_hw_rx_statistics_t st;
                    if (esp_test_get_hw_rx_statistics(&st) == ESP_OK) {
                        uint8_t r[16];
                        uint16_t v[7] = { st.rx_fcs_err, st.rx_abort, st.brx_err_agc, st.nrx_err_agcexit,
                                          st.nrx_err, st.rx_mpdu, st.rx_fifo_ovfcnt };
                        for (int k = 0; k < 7; k++) { r[2*k] = v[k] & 0xff; r[2*k+1] = v[k] >> 8; }
                        r[14] = (uint8_t)(st.rx_cfo_hz & 0xff);   // carrier frequency offset, Hz
                        r[15] = (uint8_t)((st.rx_cfo_hz >> 8) & 0xff);
                        send_framed(T_HWSTATS, r, 16);
                    }
                    break;
                }
                case T_TXPOWER:
                    // Argument is esp_wifi_set_max_tx_power's unit: 0.25 dBm, valid [8,84]
                    // (= 2..20 dBm), quantised by the IDF to 11 steps.
                    //
                    // CLAMP, do not pass through. Out-of-range is rejected with
                    // ESP_ERR_INVALID_ARG and leaves the radio at whatever it was — so an
                    // unclamped low request silently yields FULL power, the exact opposite of
                    // what was asked. MEASURED before this clamp: arg 4 gave the same RSSI at a
                    // witness as arg 80 (max). A power lever that inverts at the bottom of its
                    // range is worse for cognition than no lever at all.
                    if (len >= 1) {
                        int8_t q = (int8_t)pl[0];
                        if (q < 8) q = 8;
                        if (q > 84) q = 84;
                        esp_wifi_set_max_tx_power(q);
                        // Report what the radio ACTUALLY applied (the IDF quantises), so the host
                        // can return the applied power rather than assume the request took.
                        int8_t applied = 0;
                        if (esp_wifi_get_max_tx_power(&applied) == ESP_OK) {
                            uint8_t r[2] = { (uint8_t)q, (uint8_t)applied };
                            send_framed(T_POWERIDX, r, 2);
                        }
                    }
                    break;
                case T_BW40: if (len >= 1) esp_wifi_set_bandwidth(WIFI_IF_STA, pl[0] ? WIFI_BW_HT40 : WIFI_BW_HT20); break;
                case T_RATE:
                    // [rate] (phymode auto-derived from rate code + band) or [rate][phymode] override.
                    // rate = wifi_phy_rate_t (1M_L=0x00, 24M=0x09, 54M=0x0C, MCS0_LGI=0x10, MCS7_LGI=0x17).
                    // [rate] | [rate][phymode] | [rate][phymode][he_flags: bit0=DCM bit1=ER-SU]
                    if (len >= 3)      set_fix_rate(pl[0], pl[1], pl[2] & 1, pl[2] & 2);
                    else if (len >= 2) set_fix_rate(pl[0], pl[1], false, false);
                    else if (len >= 1) set_fix_rate(pl[0], 0, false, false);
                    break;
                case T_INJECT_ATTR: { // [npairs][o,v]*n[frame] — skip the attr pokes (no pkt_attrib on esp), inject the frame
                    if (len >= 1) { int np = pl[0]; int off = 1 + 2 * np; if (len > off) esp_wifi_80211_tx(WIFI_IF_STA, pl + off, len - off, true); }
                    break;
                }
                case T_INJECT_AT: if (len >= 4) { // [delay_us_le32][frame] — place TX at a precise instant
                    uint32_t delay = pl[0] | ((uint32_t)pl[1] << 8) | ((uint32_t)pl[2] << 16) | ((uint32_t)pl[3] << 24);
                    if (delay > SCHED_MAX_DELAY_US) delay = SCHED_MAX_DELAY_US;
                    // Schedule against esp_timer (always-running µs); the 802.11 TSF reads 0 when the STA
                    // is unassociated, so it can't be the schedule clock here — but report it too so the
                    // host can see whether it's live (a future common-view lease would sync on it).
                    int64_t target = esp_timer_get_time() + delay;
                    while (esp_timer_get_time() < target) { /* spin to the scheduled instant */ }
                    esp_wifi_80211_tx(WIFI_IF_STA, pl + 4, len - 4, true);
                    int64_t actual = esp_timer_get_time();
                    int64_t tsf = esp_wifi_get_tsf_time(WIFI_IF_STA);
                    uint8_t rep[24];
                    for (int k = 0; k < 8; k++) {
                        rep[k] = (uint8_t)(target >> (8 * k));
                        rep[8 + k] = (uint8_t)(actual >> (8 * k));
                        rep[16 + k] = (uint8_t)(tsf >> (8 * k));
                    }
                    send_framed(T_TXTIME, rep, 24); // [target][actual][tsf]: error = actual-target
                    break; }
                case T_INJECT_ABS: if (len >= 8) { // [target_us_le64][frame] — TX at an ABSOLUTE esp_timer µs (slot lease)
                    int64_t target = 0;
                    for (int k = 0; k < 8; k++) target |= ((int64_t)pl[k]) << (8 * k);
                    int64_t now = esp_timer_get_time();
                    // Honour only a target within [now, now+cap] — a stale/past or far-future one is dropped
                    // rather than blocking the dispatch or firing late (the slot has already passed).
                    if (target > now && target - now <= SCHED_MAX_DELAY_US) {
                        while (esp_timer_get_time() < target) { /* spin to the scheduled instant */ }
                        esp_wifi_80211_tx(WIFI_IF_STA, pl + 8, len - 8, true);
                        int64_t actual = esp_timer_get_time();
                        int64_t tsf = esp_wifi_get_tsf_time(WIFI_IF_STA);
                        uint8_t rep[24];
                        for (int k = 0; k < 8; k++) {
                            rep[k] = (uint8_t)(target >> (8 * k));
                            rep[8 + k] = (uint8_t)(actual >> (8 * k));
                            rep[16 + k] = (uint8_t)(tsf >> (8 * k));
                        }
                        send_framed(T_TXTIME, rep, 24);
                    }
                    break; }
                case T_READCLOCK: { // reply with the current esp_timer (the T_INJECT_ABS schedule clock)
                    int64_t t = esp_timer_get_time();
                    uint8_t c[8]; for (int k = 0; k < 8; k++) c[k] = (uint8_t)(t >> (8 * k));
                    send_framed(T_CLOCK, c, 8);
                    break; }
                case T_PREFIXES: if (len >= 1) { // [n][u64_le prefix-hash]*n — §6 relevance set
                    uint8_t n = pl[0]; if (n > MAX_PREFIXES) n = MAX_PREFIXES;
                    if (len >= (uint16_t)(1 + n * 8)) {
                        for (uint8_t k = 0; k < n; k++) {
                            uint64_t hh = 0;
                            for (int b = 0; b < 8; b++) hh |= (uint64_t)pl[1 + k * 8 + b] << (8 * b);
                            s_prefixes[k] = hh;
                        }
                        s_nprefixes = n; // publish count last
                    }
                    break; }
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

#ifdef NDR_PARSE_BENCH
    { void ndr_parse_bench(void); ndr_parse_bench(); }
#endif

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
    set_fix_rate(WIFI_PHY_RATE_6M, 0, false, false); // OFDM 6M default — beats the 1 Mbps basic rate
    ESP_ERROR_CHECK(esp_wifi_set_promiscuous_rx_cb(rx_cb));
    ESP_ERROR_CHECK(esp_wifi_set_promiscuous(true));

    rxq = xQueueCreate(32, sizeof(rxpkt_t));
    bleq = xQueueCreate(24, sizeof(blerep_t));
    advq = xQueueCreate(16, sizeof(advreq_t)); // holds a few multi-fragment packets in flight

    // BLE bearer (NimBLE) alongside Wi-Fi — one firmware, all bearers. Software coex (CONFIG_ESP_COEX_SW_
    // COEXIST_ENABLE, auto-on with BT+Wi-Fi) time-shares the radio between promiscuous Wi-Fi RX and BLE scan.
    ESP_ERROR_CHECK(nimble_port_init());
    ble_hs_cfg.sync_cb = ble_on_sync;
    nimble_port_freertos_init(ble_host_task);

    xTaskCreate(serial_tx_task, "ser_tx", 4096, NULL, 5, NULL);
    xTaskCreate(ble_adv_task, "ble_adv", 4096, NULL, 5, NULL);
    serial_rx_loop(); // never returns
}
