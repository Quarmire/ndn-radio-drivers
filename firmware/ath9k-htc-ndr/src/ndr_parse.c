/* ndr_parse.c — see ndr_parse.h. Division-free, single-pass, bounded, no heap. */
#include "ndr_parse.h"

/* Hot RX path: optimize this TU at -O2 even inside a debug (-Og) firmware build, so the measured
 * parse cost reflects a shipping build. GCC (riscv32-esp-elf / xtensa) honours this pragma. */
#pragma GCC optimize ("O2")

/* NDN-TLV / NDNLPv2 type codes we walk. */
#define T_BODY_PREFIX  0xF5  /* LoRa self-signaling GCS body TLV (fail-open strip) */
#define T_LP_PACKET    0x64
#define T_LP_FRAGMENT  0x50
#define T_LP_FRAGINDEX 0x52
#define T_INTEREST     0x05
#define T_DATA         0x06
#define T_NAME         0x07

#define FNV_OFFSET 0xcbf29ce484222325ULL
#define FNV_PRIME  0x00000100000001b3ULL

/* Read one VAR-NUMBER at b[*pos], big-endian. <253 = the byte; 253/254/255 = u16/u32/u64 BE.
 * Bounded: returns 0 (and leaves *pos) if it would run off the end. Accepts non-minimal (RX fast path). */
static int rd_varnum(const uint8_t *b, uint32_t len, uint32_t *pos, uint64_t *out)
{
    if (*pos >= len) return 0;
    uint8_t first = b[*pos];
    uint32_t adv, need;
    if (first < 253) { *out = first; *pos += 1; return 1; }
    else if (first == 253) { need = 2; adv = 3; }
    else if (first == 254) { need = 4; adv = 5; }
    else                   { need = 8; adv = 9; }
    if ((uint64_t)*pos + adv > len) return 0;
    uint64_t v = 0;
    for (uint32_t i = 0; i < need; i++) v = (v << 8) | b[*pos + 1 + i];
    *pos += adv; *out = v; return 1;
}

int ndr_walk_to_name(const uint8_t *frame, uint32_t len,
                     const uint8_t **name_val, uint32_t *name_len, uint8_t *kind)
{
    *kind = NDR_KIND_NONE;

    /* Fail-open strip of the optional body-prefix GCS TLV (0xF5 len ...). */
    const uint8_t *b = frame; uint32_t blen = len;
    if (blen >= 2 && b[0] == T_BODY_PREFIX) {
        uint32_t off = 2u + (uint32_t)b[1];
        if (off <= blen) { b += off; blen -= off; }
    }
    if (blen == 0) return 0;

    /* LpPacket? find the Fragment; bail on a non-zero FragIndex (continuation, no Name). */
    const uint8_t *pkt; uint32_t pktlen;
    if (b[0] == T_LP_PACKET) {
        uint32_t pos = 0; uint64_t t, l;
        if (!rd_varnum(b, blen, &pos, &t)) return 0;   /* 0x64 */
        if (!rd_varnum(b, blen, &pos, &l)) return 0;   /* outer length */
        if ((uint64_t)pos + l > blen) return 0;
        const uint8_t *inner = b + pos; uint32_t ilen = (uint32_t)l;
        const uint8_t *frag = 0; uint32_t fraglen = 0;
        uint32_t p = 0;
        while (p < ilen) {
            uint64_t st, sl;
            if (!rd_varnum(inner, ilen, &p, &st)) return 0;
            if (!rd_varnum(inner, ilen, &p, &sl)) return 0;
            if ((uint64_t)p + sl > ilen) return 0;
            if (st == T_LP_FRAGINDEX) {
                for (uint32_t i = 0; i < sl; i++) if (inner[p + i]) return 0; /* non-zero => continuation */
            } else if (st == T_LP_FRAGMENT) {
                frag = inner + p; fraglen = (uint32_t)sl;
            }
            p += (uint32_t)sl;
        }
        if (!frag) return 0;
        pkt = frag; pktlen = fraglen;
    } else {
        pkt = b; pktlen = blen;
    }

    /* Interest (0x05) / Data (0x06); then the first Name (0x07) sub-TLV. */
    if (pktlen == 0) return 0;
    uint8_t k;
    if (pkt[0] == T_INTEREST) k = NDR_KIND_INTEREST;
    else if (pkt[0] == T_DATA) k = NDR_KIND_DATA;
    else return 0;

    uint32_t pos = 0; uint64_t t, l;
    if (!rd_varnum(pkt, pktlen, &pos, &t)) return 0;   /* packet type */
    if (!rd_varnum(pkt, pktlen, &pos, &l)) return 0;   /* packet length */
    if ((uint64_t)pos + l > pktlen) return 0;
    const uint8_t *body = pkt + pos; uint32_t bodylen = (uint32_t)l;

    uint32_t q = 0;
    while (q < bodylen) {
        uint64_t tt, ll;
        if (!rd_varnum(body, bodylen, &q, &tt)) return 0;
        if (!rd_varnum(body, bodylen, &q, &ll)) return 0;
        if ((uint64_t)q + ll > bodylen) return 0;
        if (tt == T_NAME) { *name_val = body + q; *name_len = (uint32_t)ll; *kind = k; return 1; }
        q += (uint32_t)ll;
    }
    return 0;
}

int ndr_name_admits(const uint8_t *frame, uint32_t len,
                    const uint64_t *set, uint32_t set_len, uint8_t *kind)
{
    const uint8_t *nv; uint32_t nl;
    if (!ndr_walk_to_name(frame, len, &nv, &nl, kind)) return 0;

    /* One forward pass over the Name components, rolling FNV over the virtual /-joined form,
     * testing longest-prefix membership at each component boundary (== `any_prefix_in` fused in). */
    uint64_t hash = FNV_OFFSET;
    uint32_t p = 0, depth = 0; int first = 1;
    while (p < nl) {
        uint64_t ct, cl;
        if (!rd_varnum(nv, nl, &p, &ct)) return 0;
        if (!rd_varnum(nv, nl, &p, &cl)) return 0;
        if ((uint64_t)p + cl > nl) return 0;
        if (++depth > NDR_MAX_DEPTH) return 0;               /* reject-early */
        if (!first) {                                        /* boundary: hash == prefix so far */
            for (uint32_t s = 0; s < set_len; s++) if (set[s] == hash) return 1;
        }
        first = 0;
        hash ^= (uint8_t)'/'; hash *= FNV_PRIME;
        for (uint32_t i = 0; i < cl; i++) { hash ^= nv[p + i]; hash *= FNV_PRIME; }
        p += (uint32_t)cl;
    }
    if (depth == 0) { hash ^= (uint8_t)'/'; hash *= FNV_PRIME; } /* root name "/" */
    for (uint32_t s = 0; s < set_len; s++) if (set[s] == hash) return 1; /* full name */
    return 0;
}

int ndr_parse_name(const uint8_t *frame, uint32_t len,
                   uint8_t *out, uint32_t out_cap,
                   uint8_t *kind, uint32_t *name_out_len,
                   uint64_t *full_hash, uint8_t *depth)
{
    const uint8_t *nv; uint32_t nl;
    if (!ndr_walk_to_name(frame, len, &nv, &nl, kind)) return 0;

    uint32_t n = 0, p = 0, d = 0;
    while (p < nl) {
        uint64_t ct, cl;
        if (!rd_varnum(nv, nl, &p, &ct)) return 0;
        if (!rd_varnum(nv, nl, &p, &cl)) return 0;
        if ((uint64_t)p + cl > nl) return 0;
        if (++d > NDR_MAX_DEPTH) return 0;
        if ((uint64_t)n + 1 + cl > out_cap) return 0;
        out[n++] = '/';
        for (uint32_t i = 0; i < cl; i++) out[n++] = nv[p + i];
        p += (uint32_t)cl;
    }
    if (n == 0) { if (out_cap < 1) return 0; out[0] = '/'; n = 1; } /* root */
    *name_out_len = n; *depth = (uint8_t)d;
    *full_hash = ndr_fnv1a64(out, n);
    return 1;
}

uint64_t ndr_fnv1a64(const uint8_t *p, uint32_t n)
{
    uint64_t h = FNV_OFFSET;
    for (uint32_t i = 0; i < n; i++) { h ^= p[i]; h *= FNV_PRIME; }
    return h;
}
