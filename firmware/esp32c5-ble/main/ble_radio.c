// ESP32-C5 named-data radio — BLE bearer.
//
// A BLE analog of the Wi-Fi named radio: instead of raw 802.11 injection it uses **BLE 5 extended
// advertising** to broadcast named data and **extended scanning** to receive it — connectionless, no
// pairing, no GATT (the named-radio model). It speaks the SAME `[4E 44 type len_le16 payload]` ("ND")
// serial wire protocol as the Wi-Fi firmware, so the host maps the AdvBackend trait onto it directly:
//   host->device  T_INJECT (0x01) [payload]                 -> wrap + extended-advertise (fire-and-forget)
//   device->host  T_RX     (0x81) [rssi_i8][addr6][payload] -> a scanned advertisement carrying our magic
//
// Named data rides in a **manufacturer-specific AD** with company id 0x4E44 ("ND"), mirroring how the
// Wi-Fi firmware filters ethertype 0x8624 — the firmware forwards only ads carrying that magic, so the
// 115200 serial pipe is not swamped by every BLE beacon in the room.

#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "nvs_flash.h"
#include "driver/usb_serial_jtag.h"
#include "esp_bt.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/ble_hs.h"
#include "host/util/util.h"
#include "host/ble_gap.h"
#include "os/os_mbuf.h"

#define SYNC0 0x4E
#define SYNC1 0x44
#define T_INJECT 0x01 // [payload] -> advertise
#define T_RX     0x81 // [rssi][addr6][payload] -> scanned
#define ADV_INSTANCE 0
#define ADV_MAGIC_LO 0x44 // company id 0x4E44 ("ND"), little-endian on air
#define ADV_MAGIC_HI 0x4E
#define MAXPAY 240   // inner named payload cap (ext-adv 254 minus the 4-byte manufacturer-AD header + margin)

static uint8_t s_own_addr_type;
static uint8_t s_own_addr[6];
static QueueHandle_t s_rxq;         // scanned reports -> serial-TX task
static volatile bool s_configured;  // adv instance configured (post-sync)

typedef struct { int8_t rssi; uint8_t addr[6]; uint8_t len; uint8_t buf[MAXPAY]; } rxrep_t;

static void send_framed(uint8_t ty, const uint8_t *p, uint16_t len) {
    uint8_t hdr[5] = { SYNC0, SYNC1, ty, (uint8_t)(len & 0xff), (uint8_t)(len >> 8) };
    usb_serial_jtag_write_bytes(hdr, 5, portMAX_DELAY);
    if (len) usb_serial_jtag_write_bytes(p, len, portMAX_DELAY);
}

// Scan an ext-adv payload for our manufacturer AD (type 0xFF, company 0x4E44); return the inner named
// payload pointer/len, or 0 if not present. AD list = repeating [len][type][data(len-1)].
static int find_named(const uint8_t *d, int n, const uint8_t **out) {
    int i = 0;
    while (i + 2 <= n) {
        int adlen = d[i];
        if (adlen < 1 || i + 1 + adlen > n) break;
        uint8_t adtype = d[i + 1];
        if (adtype == 0xFF && adlen >= 3 && d[i + 2] == ADV_MAGIC_LO && d[i + 3] == ADV_MAGIC_HI) {
            *out = &d[i + 4];
            return adlen - 3; // minus type(1) + company(2)
        }
        i += 1 + adlen;
    }
    return 0;
}

// The single GAP callback: extended-discovery reports (scan) + adv-complete events.
static int gap_event(struct ble_gap_event *ev, void *arg) {
    if (ev->type == BLE_GAP_EVENT_EXT_DISC) {
        const struct ble_gap_ext_disc_desc *dsc = &ev->ext_disc;
        // Ignore our own advertisements reflected back (half-duplex: a node never hears itself).
        if (memcmp(dsc->addr.val, s_own_addr, 6) == 0) return 0;
        const uint8_t *pay = NULL;
        int plen = find_named(dsc->data, dsc->length_data, &pay);
        if (plen > 0 && plen <= MAXPAY) {
            rxrep_t r;
            r.rssi = dsc->rssi;
            memcpy(r.addr, dsc->addr.val, 6);
            r.len = (uint8_t)plen;
            memcpy(r.buf, pay, plen);
            BaseType_t hp = pdFALSE;
            xQueueSendFromISR(s_rxq, &r, &hp); // best-effort; drops if full
            if (hp) portYIELD_FROM_ISR();
        }
    }
    return 0;
}

static void adv_configure(void) {
    struct ble_gap_ext_adv_params p;
    memset(&p, 0, sizeof(p));
    p.connectable = 0;
    p.scannable = 0;
    p.legacy_pdu = 0;          // BLE 5 extended PDU (the >31-byte payload path)
    p.own_addr_type = s_own_addr_type;
    p.primary_phy = BLE_HCI_LE_PHY_1M;
    p.secondary_phy = BLE_HCI_LE_PHY_2M;
    p.itvl_min = 0x30;         // 48 * 0.625ms = 30ms
    p.itvl_max = 0x30;
    p.sid = 0;
    p.tx_power = 127;          // no preference
    int8_t selected;
    ble_gap_ext_adv_configure(ADV_INSTANCE, &p, &selected, gap_event, NULL);
    s_configured = true;
}

static void scan_start(void) {
    struct ble_gap_ext_disc_params up;
    memset(&up, 0, sizeof(up));
    up.itvl = 0x50;
    up.window = 0x50;          // fully open (scan window == interval) — continuous listen
    up.passive = 1;            // no scan requests (connectionless)
    ble_gap_ext_disc(s_own_addr_type, 0 /*forever*/, 0, 0 /*no dup filter*/, 0, 0, &up, NULL, gap_event, NULL);
}

// T_INJECT: wrap the host payload in our manufacturer AD and burst-advertise it (fire-and-forget).
static void advertise(const uint8_t *payload, int len) {
    if (!s_configured || len <= 0 || len > MAXPAY) return;
    struct os_mbuf *m = os_msys_get_pkthdr(len + 4, 0);
    if (!m) return;
    uint8_t hdr[4] = { (uint8_t)(len + 3), 0xFF, ADV_MAGIC_LO, ADV_MAGIC_HI }; // AD: len,type,company
    os_mbuf_append(m, hdr, 4);
    os_mbuf_append(m, payload, len);
    ble_gap_ext_adv_stop(ADV_INSTANCE);
    if (ble_gap_ext_adv_set_data(ADV_INSTANCE, m) == 0) {
        ble_gap_ext_adv_start(ADV_INSTANCE, 0, 3); // 3 adv events, then stop — a broadcast burst
    } else {
        os_mbuf_free_chain(m);
    }
}

static void on_sync(void) {
    ble_hs_util_ensure_addr(0);
    ble_hs_id_infer_auto(0, &s_own_addr_type);
    ble_hs_id_copy_addr(s_own_addr_type, s_own_addr, NULL);
    adv_configure();
    scan_start();
}

static void host_task(void *param) {
    nimble_port_run();
    nimble_port_freertos_deinit();
}

// Drain scanned reports -> serial as T_RX [rssi][addr6][payload].
static void serial_tx_task(void *arg) {
    static uint8_t out[7 + MAXPAY];
    rxrep_t r;
    for (;;) {
        if (xQueueReceive(s_rxq, &r, portMAX_DELAY) == pdTRUE) {
            out[0] = (uint8_t)r.rssi;
            memcpy(out + 1, r.addr, 6);
            memcpy(out + 7, r.buf, r.len);
            send_framed(T_RX, out, 7 + r.len);
        }
    }
}

// Read the host protocol -> dispatch T_INJECT (advertise). Other types (channel/rate/power) are no-ops
// for BLE (fixed adv channels 37/38/39, controller-managed rate/power).
static void serial_rx_loop(void) {
    static uint8_t acc[512];
    int n = 0;
    for (;;) {
        int r = usb_serial_jtag_read_bytes(acc + n, sizeof(acc) - n, pdMS_TO_TICKS(20));
        if (r > 0) n += r;
        int i = 0;
        while (n - i >= 5) {
            if (acc[i] != SYNC0 || acc[i + 1] != SYNC1) { i++; continue; }
            uint8_t ty = acc[i + 2];
            uint16_t len = acc[i + 3] | (acc[i + 4] << 8);
            if (n - i < 5 + len) break;
            uint8_t *pl = acc + i + 5;
            if (ty == T_INJECT) advertise(pl, len);
            i += 5 + len;
        }
        if (i > 0) { memmove(acc, acc + i, n - i); n -= i; }
        if (n == (int)sizeof(acc)) n = 0; // desync guard
    }
}

void app_main(void) {
    nvs_flash_init();
    s_rxq = xQueueCreate(24, sizeof(rxrep_t));

    usb_serial_jtag_driver_config_t ucfg = {
        .tx_buffer_size = 2048,
        .rx_buffer_size = 2048,
    };
    usb_serial_jtag_driver_install(&ucfg);

    nimble_port_init();
    ble_hs_cfg.sync_cb = on_sync;
    nimble_port_freertos_init(host_task);

    xTaskCreate(serial_tx_task, "ble_tx", 4096, NULL, 5, NULL);
    serial_rx_loop(); // never returns
}
