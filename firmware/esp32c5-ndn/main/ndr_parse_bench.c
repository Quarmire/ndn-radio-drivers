/* ndr_parse_bench.c — ESP32-C5 in-firmware timing of the fused name parser.
 * Gated by NDR_PARSE_BENCH; app_main() calls ndr_parse_bench() right after the USB-serial driver is up
 * and never returns, so this is a clean measurement build (no Wi-Fi/BLE init).
 * Builds the corpus at boot (not timed), then times ndr_name_admits() with esp_cpu_get_cycle_count()
 * (240 MHz => 240 cyc/us) and prints ASCII over USB-Serial-JTAG, repeating every 3 s so a serial read
 * always catches a full block. Integer formatting only (no %f).
 */
#include "ndr_parse.h"
#include "esp_cpu.h"
#include "driver/usb_serial_jtag.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <stdint.h>
#include <stdio.h>
#include <string.h>

/* Minimal TLV encoder — build NDNLPv2 LpPacket{Fragment{Interest{Name}}} at boot (untimed). */
static void put_var(uint8_t *b, uint32_t *n, uint64_t v) {
    if (v < 253) { b[(*n)++] = (uint8_t)v; }
    else if (v < 0x10000ULL) { b[(*n)++] = 253; b[(*n)++] = (uint8_t)(v >> 8); b[(*n)++] = (uint8_t)v; }
    else { b[(*n)++] = 254; for (int i = 3; i >= 0; i--) b[(*n)++] = (uint8_t)(v >> (8 * i)); }
}
static uint32_t enc_frame(const char *slash, uint8_t *out) {
    uint8_t nv[256]; uint32_t nvl = 0; const char *p = slash; if (*p == '/') p++;
    while (*p) { const char *s = p; while (*p && *p != '/') p++; uint32_t cl = (uint32_t)(p - s);
        put_var(nv, &nvl, 0x08); put_var(nv, &nvl, cl); memcpy(nv + nvl, s, cl); nvl += cl; if (*p == '/') p++; }
    uint8_t name[300]; uint32_t nl = 0; put_var(name, &nl, 0x07); put_var(name, &nl, nvl); memcpy(name + nl, nv, nvl); nl += nvl;
    uint8_t pkt[360]; uint32_t pl = 0; put_var(pkt, &pl, 0x05); put_var(pkt, &pl, nl); memcpy(pkt + pl, name, nl); pl += nl;
    uint8_t frag[400]; uint32_t fl = 0; put_var(frag, &fl, 0x50); put_var(frag, &fl, pl); memcpy(frag + fl, pkt, pl); fl += pl;
    uint32_t o = 0; put_var(out, &o, 0x64); put_var(out, &o, fl); memcpy(out + o, frag, fl); o += fl; return o;
}
static void emit(const char *s) { usb_serial_jtag_write_bytes((const uint8_t *)s, strlen(s), portMAX_DELAY); }

void ndr_parse_bench(void) {
    static const char *names[] = {
        "/a", "/ndn/test/v1", "/ndn/iot/sensor/temp/2026/reading", "/x/y/z/w/v/u/t/s/r/q/p/o"
    };
    uint64_t miss[16]; for (int i = 0; i < 16; i++) miss[i] = 0xdead0000ULL + i; /* worst case: never admits */
    char line[176];
    vTaskDelay(pdMS_TO_TICKS(400)); /* let boot-log settle */
    for (;;) {
        emit("\r\n=== NDR PARSE BENCH  (ESP32-C5, 240 MHz, fused walk+roll-FNV+LPM, 16-entry non-match) ===\r\n");
        for (int ni = 0; ni < 4; ni++) {
            uint8_t frame[512]; uint32_t flen = enc_frame(names[ni], frame);
            uint8_t kind; volatile int sink = 0;
            const int N = 20000;
            uint32_t c0 = esp_cpu_get_cycle_count();
            for (int i = 0; i < N; i++) sink += (int)frame[0];              /* empty-loop baseline */
            uint32_t base = esp_cpu_get_cycle_count() - c0;
            c0 = esp_cpu_get_cycle_count();
            for (int i = 0; i < N; i++) sink += ndr_name_admits(frame, flen, miss, 16, &kind);
            uint32_t tot = esp_cpu_get_cycle_count() - c0;
            uint32_t net = (tot > base) ? (tot - base) : tot;
            uint32_t cyc_x1000 = (uint32_t)(((uint64_t)net * 1000) / N);      /* cyc/decision x1000 */
            uint32_t us_x1000  = (uint32_t)(((uint64_t)net * 1000) / ((uint64_t)N * 240)); /* us x1000 (240 cyc/us) */
            snprintf(line, sizeof line, "  %-38s %3uB  %u.%03u cyc/dec  %u.%03u us  (sink=%d)\r\n",
                     names[ni], (unsigned)flen,
                     (unsigned)(cyc_x1000 / 1000), (unsigned)(cyc_x1000 % 1000),
                     (unsigned)(us_x1000 / 1000), (unsigned)(us_x1000 % 1000), sink);
            emit(line);
        }
        emit("=== done (repeats every 3 s) ===\r\n");
        vTaskDelay(pdMS_TO_TICKS(3000));
    }
}
