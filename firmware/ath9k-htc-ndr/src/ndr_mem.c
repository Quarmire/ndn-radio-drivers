/* WMI_ACCESS_MEMORY_CMDID handler — see ndr_mem.h for the wire format and why it is ours. */

#include "ndr_mem.h"

/*
 * All payload access is byte-wise. The WMI payload sits at an offset the transport chooses, so a
 * `u32 *` cast into it is not guaranteed to be aligned — and on Xtensa an unaligned 32-bit load
 * does not merely go slow, it faults.
 */
static a_uint32_t rd_be32(const a_uint8_t *p)
{
	return ((a_uint32_t)p[0] << 24) | ((a_uint32_t)p[1] << 16) |
	       ((a_uint32_t)p[2] << 8) | (a_uint32_t)p[3];
}

static void wr_be32(a_uint8_t *p, a_uint32_t v)
{
	p[0] = (a_uint8_t)(v >> 24);
	p[1] = (a_uint8_t)(v >> 16);
	p[2] = (a_uint8_t)(v >> 8);
	p[3] = (a_uint8_t)v;
}

static a_uint16_t rd_be16(const a_uint8_t *p)
{
	return (a_uint16_t)(((a_uint16_t)p[0] << 8) | (a_uint16_t)p[1]);
}

static void wr_be16(a_uint8_t *p, a_uint16_t v)
{
	p[0] = (a_uint8_t)(v >> 8);
	p[1] = (a_uint8_t)v;
}

static a_int32_t addr_ok(a_uint32_t a)
{
	if (a >= NDR_MEM_RAM_LO && a < NDR_MEM_RAM_HI)
		return 1;
	if (a >= NDR_MEM_IRAM_LO && a < NDR_MEM_IRAM_HI)
		return 1;
	return 0;
}

a_int32_t ndr_mem_access(const a_uint8_t *req, a_int32_t reqlen,
			 a_uint8_t *rsp, a_int32_t rspcap)
{
	a_uint16_t flags = 0;
	a_uint16_t count = 0;
	a_uint16_t status = NDR_MEM_OK;
	a_uint32_t i;

	if (rspcap < 4)
		return 0;

	if (reqlen < 4) {
		status = NDR_MEM_ERR_MALFORMED;
		goto out;
	}

	flags = rd_be16(req);
	count = rd_be16(req + 2);

	if (count > NDR_MEM_MAX_TUPLES) {
		status = NDR_MEM_ERR_TOO_MANY;
		count = 0;
		goto out;
	}
	if (reqlen < (a_int32_t)(4 + count * 8) ||
	    rspcap < (a_int32_t)(4 + count * 8)) {
		status = NDR_MEM_ERR_MALFORMED;
		count = 0;
		goto out;
	}

	/*
	 * Validate every address BEFORE touching any of them. A request that is going to be
	 * rejected must not have already written half its tuples — a partially applied
	 * configuration is far worse to debug than a cleanly refused one.
	 */
	for (i = 0; i < count; i++) {
		a_uint32_t addr = rd_be32(req + 4 + i * 8);

		if (addr & 3u) {
			status = NDR_MEM_ERR_UNALIGNED;
			count = 0;
			goto out;
		}
		if (!addr_ok(addr)) {
			status = NDR_MEM_ERR_RANGE;
			count = 0;
			goto out;
		}
	}

	for (i = 0; i < count; i++) {
		const a_uint8_t *t = req + 4 + i * 8;
		a_uint8_t *o = rsp + 4 + i * 8;
		a_uint32_t addr = rd_be32(t);
		a_uint32_t val = rd_be32(t + 4);

		wr_be32(o, addr);

		if (flags & NDR_MEM_FLAG_WRITE) {
			*(volatile a_uint32_t *)addr = val;
			wr_be32(o + 4, val);
		} else {
			wr_be32(o + 4, *(volatile a_uint32_t *)addr);
		}
	}

out:
	wr_be16(rsp, status);
	wr_be16(rsp + 2, count);
	return 4 + (a_int32_t)count * 8;
}
