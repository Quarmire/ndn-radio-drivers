/* Runtime configuration over the transmit path -- see ndr_ctl.h. */

#include "ndr_ctl.h"
#include "ndr_mac.h"

a_uint32_t ndr_ctl_lease_override;
a_uint32_t ndr_ctl_lease_slot;
a_uint32_t ndr_ctl_lease_slots = 4;
a_uint32_t ndr_ctl_lease_slot_tu = 8;
a_uint32_t ndr_ctl_quiet_off;
a_uint32_t ndr_ctl_applied;

static a_uint32_t rd_be32(const a_uint8_t *p)
{
	return ((a_uint32_t)p[0] << 24) | ((a_uint32_t)p[1] << 16) |
	       ((a_uint32_t)p[2] << 8) | (a_uint32_t)p[3];
}

static a_int32_t is_ctl_addr(const a_uint8_t *a)
{
	return a[0] == NDR_CTL_A0 && a[1] == NDR_CTL_A1 && a[2] == NDR_CTL_A2 &&
	       a[3] == NDR_CTL_A3 && a[4] == NDR_CTL_A4 && a[5] == NDR_CTL_A5;
}

a_int32_t ndr_ctl_intercept(const a_uint8_t *data, a_uint32_t len)
{
	const a_uint8_t *b;
	a_uint32_t oplen;
	a_uint8_t op;

	if (len < NDR_CTL_BODY + 6)
		return 0;
	if (!is_ctl_addr(data + 4))
		return 0;

	b = data + NDR_CTL_BODY;
	if (rd_be32(b) != NDR_CTL_MAGIC)
		return 0;

	op = b[4];
	oplen = b[5];
	if (len < NDR_CTL_BODY + 6 + oplen)
		return 1; /* addressed to us but malformed: consume it, never transmit it */

	b += 6;

	switch (op) {
	/* Ops 0x01..0x06 (the retired in-frame name filter) are gone; only the lease/quiet
	 * control survives here — see firmware/NDR_MAC_SPEC.md. */
	case NDR_OP_LEASE:
		if (oplen >= 3) {
			ndr_ctl_lease_slots = 1u << b[0]; /* log2, so the mask stays a mask */
			ndr_ctl_lease_slot_tu = b[1];
			ndr_ctl_lease_slot = b[2];
			ndr_ctl_lease_override = 1;
			ndr_ctl_quiet_off = 0;
			ndr_quiet_disarm(); /* force ndr_quiet_rearm() to re-apply with the new shape */
		}
		break;

	case NDR_OP_QUIET_OFF:
		ndr_ctl_quiet_off = 1;
		ndr_quiet_disarm();
		break;

	default:
		break;
	}

	ndr_ctl_applied++;
	return 1;
}
