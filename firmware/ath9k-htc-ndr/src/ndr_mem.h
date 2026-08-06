/*
 * WMI_ACCESS_MEMORY_CMDID — target memory read/write, so the host can configure the Tier-0 filter
 * and read its counters without rebuilding and reloading the firmware.
 *
 * ## This command did not previously work AT ALL
 *
 * `wmi.h` declares `WMI_ACCESS_MEMORY_CMD` and `if_ath.c` registers a dispatch entry for it, which
 * makes it look implemented. It is not:
 *
 *   - in the **WLAN build** (`if_ath.c:1739`) the handler body is `adf_os_assert(0)` — it does not
 *     merely ignore the command, it **asserts**;
 *   - in the non-WLAN build (`magpie.c:88`) the body is an empty `break`.
 *
 * So a host must never send this command to stock firmware. Everything below is new.
 *
 * ## Wire format
 *
 * The vendor header declares the shape but nothing ever implemented the semantics, so we define
 * them. The layout matches `WMI_ACCESS_MEMORY_CMD`'s *size* exactly — a 4-byte header then 8-byte
 * tuples — while giving the two 16-bit header halves a meaning the original never assigned.
 * Everything is big-endian, like the rest of WMI.
 *
 *   request:   u16 flags | u16 count | count * { u32 addr, u32 value }
 *   response:  u16 status | u16 count | count * { u32 addr, u32 value }
 *
 * `flags & NDR_MEM_FLAG_WRITE` selects write; otherwise read. On a read the response `value` is
 * what was at `addr`; on a write it echoes what was stored. On any error `status` is non-zero and
 * `count` is 0, so a failed request costs 4 bytes and never returns half-valid data.
 *
 * ## Two limits that are not the vendor's
 *
 * `WMI_ACCESS_MEMORY_MAX_TUPLES` is 8, and **8 tuples do not fit the pipe**: the register endpoints
 * are 64-byte interrupt endpoints, and 8 (HTC) + 4 (WMI) + 4 (header) + 8*8 = 80 > 64. The real
 * ceiling is 6. This is the same class of mistake as the firmware's echo constant — a header
 * declaring a size its own transport cannot carry.
 *
 * Addresses must be 4-byte aligned. Xtensa has no unaligned 32-bit load, so an unaligned address
 * would fault the target rather than return an error.
 */

#ifndef _NDR_MEM_H_
#define _NDR_MEM_H_

#include "ndr_tier0.h"

/* Read/write. */
#define NDR_MEM_FLAG_WRITE   0x0001

/* Bounded by the 64-byte register pipe, NOT by the vendor's 8. See above. */
#define NDR_MEM_MAX_TUPLES   6

/* Largest response this can emit. */
#define NDR_MEM_RSP_MAX      (4 + NDR_MEM_MAX_TUPLES * 8)

/* status codes */
#define NDR_MEM_OK           0
#define NDR_MEM_ERR_MALFORMED 1
#define NDR_MEM_ERR_TOO_MANY  2
#define NDR_MEM_ERR_UNALIGNED 3
#define NDR_MEM_ERR_RANGE     4

/*
 * Address windows the AR9271 (k2) firmware actually occupies, from `ram-k2.ld`:
 *
 *   lit_seg  org 0x004E5200 len 0x1DE00   (literals / read-only)
 *   dram_seg org 0x0050CB40 len 0x1800    (data + bss — where ndr_cfg and ndr_stats live)
 *   iram_seg org 0x00903000 len 0x9B40    (text)
 *
 * The bounds below are a deliberately generous union of those, widened to the region boundaries
 * documented in the same file. This is a **sanity** check, not a security boundary — the host is
 * trusted. Its job is to turn a typo into a status code instead of a target fault, because a fault
 * costs a replug and a confusing debugging session.
 */
#define NDR_MEM_RAM_LO   0x004E5200u
#define NDR_MEM_RAM_HI   0x00515000u
#define NDR_MEM_IRAM_LO  0x00903000u
#define NDR_MEM_IRAM_HI  0x0092C000u

/*
 * Service one request. Returns the number of bytes written to `rsp` (always at least the 4-byte
 * header). Never returns more than `rspcap`.
 */
a_int32_t ndr_mem_access(const a_uint8_t *req, a_int32_t reqlen,
			 a_uint8_t *rsp, a_int32_t rspcap);

#endif /* _NDR_MEM_H_ */
