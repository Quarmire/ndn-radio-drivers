/*
 * Runtime configuration over the transmit path.
 *
 * ## Why this exists
 *
 * Every experiment so far cost a firmware rebuild, because the two ways to reach `ndr_cfg` are
 * mutually exclusive: `ath9k_htc` must own the device to bring the PHY up, and our userspace WMI
 * driver must own the device to reach target memory. So the radio can be *configured* or it can be
 * *receiving*, never both.
 *
 * The way out is that we own the firmware's transmit path too. `tgt_HTCRecvMessageHandler()` sees
 * every frame the host sends. A frame addressed to NDR_CTL_ADDR is not transmitted — it is consumed
 * as configuration. The host injects it with the ordinary monitor-mode path that already works, so
 * this needs no kernel patch, no WMI, and no change of device ownership.
 *
 * It is also the doctrine-clean shape: configuration arrives as a frame with a reserved address,
 * not as a side-band control channel bolted onto the driver.
 *
 * ## Wire format
 *
 *   802.11: fc(2) dur(2) addr1(6)=NDR_CTL_ADDR addr2(6) addr3(6) seq(2)
 *   body @24: u32 magic 'NDRC' | u8 op | u8 len | payload[len]
 *
 * Everything is big-endian, matching the rest of this firmware's host protocol.
 *
 * ⚠ The control address must keep the locally-administered group bits (§2 of the addressing
 * doctrine), like every other address we emit — even though these frames never reach the air.
 */

#ifndef _NDR_CTL_H_
#define _NDR_CTL_H_

#include "ndr_tier0.h"

/* Reserved control address: locally-administered group, "NDRCT". */
#define NDR_CTL_A0 0x03
#define NDR_CTL_A1 0x4e
#define NDR_CTL_A2 0x44
#define NDR_CTL_A3 0x52
#define NDR_CTL_A4 0x43
#define NDR_CTL_A5 0x54

#define NDR_CTL_MAGIC 0x4e445243u /* 'NDRC' */

/* Body offset of the control header: past the 24-byte 802.11 MAC header. */
#define NDR_CTL_BODY 24

enum {
	NDR_OP_NOP        = 0x00,
	NDR_OP_ENABLE     = 0x01, /* u8 enabled */
	NDR_OP_DROP_FOREIGN = 0x02, /* u8 */
	NDR_OP_KEY        = 0x03, /* u64 name-hash key */
	NDR_OP_NMASKS     = 0x04, /* u8 count */
	NDR_OP_MASK       = 0x05, /* u8 index, then 12 mask bytes */
	NDR_OP_CLEAR_STATS = 0x06,
	NDR_OP_LEASE      = 0x07, /* u8 slots_log2, u8 slot_tu, u8 slot -- re-arms the schedule */
	NDR_OP_QUIET_OFF  = 0x08, /* disarm the quiet schedule entirely */
};

/*
 * Returns non-zero if this frame was a control frame and has been consumed (the caller must free it
 * and must NOT transmit it).
 */
a_int32_t ndr_ctl_intercept(const a_uint8_t *data, a_uint32_t len);

/* Set by NDR_OP_LEASE / NDR_OP_QUIET_OFF; read by ndr_quiet_rearm(). */
extern a_uint32_t ndr_ctl_lease_override;
extern a_uint32_t ndr_ctl_lease_slot;
extern a_uint32_t ndr_ctl_lease_slots;
extern a_uint32_t ndr_ctl_lease_slot_tu;
extern a_uint32_t ndr_ctl_quiet_off;
extern a_uint32_t ndr_ctl_applied;   /* count of control frames applied -- observability */

#endif /* _NDR_CTL_H_ */
