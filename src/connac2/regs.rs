//! The **connac2 / MT7921 (MT7961) register map**, resolved to literal `u32` values.
//!
//! This serves the **MT7921AU** (`0e8d:7961`) userspace port in `crate::mt7921` — MediaTek's
//! 2×2 **802.11ax** USB part, and the first radio in this crate with (a) an HE actuator and
//! (b) a **per-frame hardware RX timestamp**. It is a sibling of, and shares nothing with,
//! [`crate::mt76::regs`]: connac2 is a different MAC generation from mt76x02, with a different
//! address space, different block layout, and a firmware-mediated RX filter. Do not cross-read
//! the two maps.
//!
//! Transcribed from the mainline mt76 tree, `drivers/net/wireless/mediatek/mt76`:
//!   * `mt792x_regs.h` — almost all of it (the shared connac2 blocks + the USB UDMA block),
//!   * `mt7921/regs.h` — the MDP block, `MT_WFSYS_SW_RST_B`, the PCIe remap window,
//!   * `mt7615/regs.h` — cross-checked for the UDMA/WLCFG bitfields, which are identical
//!     **but sit at a different base** (see the ⚠ below),
//!   * `mt792x_usb.c` / `mt7921/usb.c` — which access path each register actually takes,
//!   * `mt7921/pci.c` — the fixed remap table, transcribed here only to *disqualify* the
//!     addresses it produces.
//!
//! Every constant carries its upstream `file:line`. Upstream writes bitfields as `BIT(n)` /
//! `GENMASK(hi, lo)`; here each is the **literal mask**, with the bit range restated in the
//! comment so a reader can check the arithmetic without re-deriving it. Every derived address
//! in this file was computed and cross-checked against upstream's macro before being written
//! down, and the `tests` module re-checks the strides.
//!
//! # MEASURED (mds-o5p-3's MT7921AU, 2026-08-27) vs CODE-READ
//!
//! Reasoning about these radios has a ~0% hit rate on this rig and measurement ~100%, so the
//! distinction is kept in the code. **MEASURED on the target silicon**, over our own libusb
//! code after claiming interface 3, and recorded as constants in [`measured`]:
//!
//! | address | symbol | value |
//! |---|---|---|
//! | `0x7001_0200` | [`MT_HW_CHIPID`] | `0x0000_7961` |
//! | `0x7001_0204` | [`MT_HW_REV`] | `0x0000_8a10` |
//! | `0x7c06_00f0` | [`MT_CONN_ON_MISC`] | `0x0000_0000` (firmware **not** running) |
//! | `0x7000_00f0` | ⚠ **not** [`MT_TOP_MISC`] — see below | `0x0000_0000` |
//!
//! EP0 round trip on this USB **2.0** bus: **268 µs** ([`measured::EP0_ROUND_TRIP_US`]) — nearly
//! double the mt76x0's 151 µs. Nothing on a per-frame path may read a register here; that
//! budget is why the headline RX timestamp has to come out of the RXD (`mt7921/mac.c:307-309`)
//! and not out of [`mt_lpon_uttr0`].
//!
//! **CODE-READ, unvalidated on silicon:** every other address and every bitfield in this file.
//!
//! # ★ Flag 1 — over USB there is **no remap window**. Addresses are raw and verbatim.
//!
//! `mt7921/usb.c:174-181` installs `bus_ops = { .rr = mt792xu_rr, .wr = mt792xu_wr, … }`
//! **directly**, with no address-translating wrapper. `mt792xu_rr` (`mt792x_usb.c:154-164`)
//! calls `___mt76u_rr` (`usb.c:76-90`), which splits the address as `wValue = addr >> 16`,
//! `wIndex = (u16)addr` and issues `MT_VEND_READ_EXT` (`0x63`). The full 32-bit physical
//! address therefore goes straight onto the wire. MEASURED: `0x7001_0200` returned `0x7961`
//! with no window setup of any kind.
//!
//! The remap is the **PCIe** path's problem: `mt7921/pci.c:70-146` wraps every access in
//! `__mt7921_reg_addr`, which walks a 43-row `fixed_map` and falls back to the L1 window at
//! [`pcie_only::MT_HIF_REMAP_L1`]. None of that machinery is reachable — or needed — here.
//!
//! # ★ Flag 2 — five upstream symbol families are written **already remapped** and are
//! therefore INVALID over USB.
//!
//! This is the trap the flag above creates: a handful of `mt792x_regs.h` / `mt7921/regs.h`
//! symbols are *not* physical addresses at all — they are the PCIe BAR offsets the fixed map
//! produces, checked in as if they were register addresses. Writing one of them over USB
//! writes some unrelated place. They are quarantined in [`pcie_only`], each with the physical
//! address obtained by inverting the corresponding `fixed_map` row:
//!
//! | upstream family | example | PCIe form | physical (USB) form | `fixed_map` row |
//! |---|---|---|---|---|
//! | `MT_WFDMA0(x)` | `MT_WFDMA0_GLO_CFG` | `0x000d_4208` | `0x7c02_4208` | `{0x7c020000, 0xd0000, 0x10000}` |
//! | `MT_WFDMA_EXT_CSR(x)` | `MT_WFDMA_EXT_CSR_HIF_MISC` | `0x000d_7044` | `0x7c02_7044` | same row |
//! | `MT_MCU_WFDMA1(x)` | `MT_MCU_INT_EVENT` | `0x0000_3108` | `0x5500_0108` | `{0x55000000, 0x03000, 0x01000}` |
//! | `MT_INFRA(x)` | `MT_HIF_REMAP_L1` | `0x000f_e24c` | `0x7c00_e24c` | `{0x7c000000, 0xf0000, 0x10000}` |
//! | `MT_PCIE_MAC(x)` | `MT_PCIE_MAC_PM` | `0x0001_0194` | `0x7403_0194` | `{0x74030000, 0x10000, 0x10000}` |
//!
//! Upstream itself sidesteps the first row by defining a **second**, physical alias for the
//! same block — [`MT_UWFDMA0_GLO_CFG`] (`mt792x_regs.h:491-495`) — and using that on USB
//! (`mt792x_usb.c:279-291`). Use the `MT_UWFDMA0_*` names. The physical forms of the other
//! four rows are **derived by us**, not stated upstream, and are untested; they are marked as
//! such in [`pcie_only`].
//!
//! ⚠ There is no remapping to *do* for anything in the main body of this file. Every address
//! outside [`pcie_only`] is already physical.
//!
//! # ★ Flag 3 — four registers are **not** reachable through `READ_EXT`/`WRITE_EXT`.
//!
//! They take the *UHW* vendor path instead: read = `MT_VEND_DEV_MODE` (`0x01`), write =
//! `MT_VEND_WRITE` (`0x02`), with `bmRequestType` built from `MT_USB_TYPE_UHW_VENDOR`
//! (`mt792x.h:556`) rather than `MT_USB_TYPE_VENDOR` (`mt792x.h:555`). See
//! [`access::MT_VEND_DEV_MODE`] and friends. The four are:
//!
//!   * [`MT_SSUSB_EPCTL_CSR_EP_RST_OPT`] (`mt792x_usb.c:332-347`),
//!   * [`MT_CBTOP_RGU_WF_SUBSYS_RST`] (`mt792x_usb.c:437-450`),
//!   * [`MT_UDMA_CONN_INFRA_STATUS`] and [`MT_UDMA_CONN_INFRA_STATUS_SEL`]
//!     (`mt792x_usb.c:452-453`).
//!
//! Each is tagged `ACCESS: UHW` on its own doc comment so the fact travels with the constant.
//! **Upstream's reason is undetermined.** It is not an address-range rule:
//! [`MT_UDMA_WLCFG_0`] sits in the very same `0x7400_0000` block and takes the ordinary
//! `READ_EXT` path (`mt792x_usb.c:354-360`). The plausible story — that UHW reaches a block
//! which stays alive while WFSYS is held in reset, which is exactly when these four are used —
//! is a guess, and is recorded as a guess.
//!
//! # ★ Flag 4 — the brief's "`MT_TOP_MISC` `0x7000_00f0`" is **not** upstream's `MT_TOP_MISC`.
//!
//! [`MT_TOP_MISC`] is `MT_TOP(0xf0)` = **`0x1806_00f0`** (`mt792x_regs.h:404,411`). The address
//! actually read on the bench was `0x7000_00f0`, which lies in the CB-TOP region
//! (`MT_CBTOP1_PHY_START 0x70000000`, `mt7915/regs.h:814`) alongside [`MT_HW_CHIPID`] and
//! [`MT_CBTOP_RGU_WF_SUBSYS_RST`]. **No upstream symbol names `0x7000_00f0`** — grepped, zero
//! hits. Its `0x0000_0000` reading is therefore a reading of an unnamed CB-TOP word and says
//! nothing about firmware state; the real [`MT_TOP_MISC`] has never been read here. Recorded as
//! [`measured::MEASURED_CBTOP_00F0`], and the test
//! `top_misc_is_not_the_measured_cbtop_word` exists so the confusion cannot silently
//! re-enter. The firmware-state fact we *do* have is [`MT_CONN_ON_MISC`] = 0, i.e.
//! [`MT_TOP_MISC2_FW_PWR_ON`] clear.
//!
//! # ★ Flag 5 — [`mt_wf_rfcr`] is not written directly on this part.
//!
//! Every other radio in this crate sets its RX filter by writing a register. MT7921 does not:
//! `mt7921_configure_filter` (`mt7921/main.c:666-693`) calls `mt7921_mcu_set_rxfilter`, and the
//! `MT_WF_RFCR_DROP_*` bits below are passed to the **firmware** as command arguments
//! (`mt7921/mcu.c:1106-1122` does exactly that with [`MT_WF_RFCR_DROP_OTHER_BEACON`]). The bit
//! *values* are still correct and still needed — as arguments. Whether a direct host write to
//! `0x820e_5000` also takes effect while the firmware owns the MAC is **undetermined**; the
//! addresses are transcribed so that question can be settled by measurement rather than by
//! having to re-derive them.
//!
//! # ⚠ `MT_UMAC_BASE` differs between mt7615 and mt792x
//!
//! `mt7615/regs.h:589` puts the UDMA/WLCFG block at `0x7c00_0000`; `mt792x_regs.h:462` puts the
//! byte-identical block at **`0x7400_0000`**. The bitfields were cross-read from mt7615 (which
//! documents them identically) but every address here uses the mt792x base. Copying an mt7615
//! *address* into this port would land in CONN_INFRA.
//!
//! # ⚠ Never reset
//!
//! `mt7921u_probe` opens with `usb_reset_device(udev)` (`mt7921/usb.c:206`) and
//! `mt792xu_reset_work` queues more (`mt792x_usb.c:25-35`). **Do not port either.** A failed
//! USB reset makes the kernel mark the hub port `disable=1` and the part needs a physical
//! replug; that happened three times to the MT7612U on this bench this week. The same caution
//! covers [`MT_CBTOP_RGU_WF_SUBSYS_RST`] and [`pcie_only::MT_WFSYS_SW_RST_B`]: they are
//! transcribed, they are not endorsements.
#![allow(dead_code)]

// ── Field helpers ────────────────────────────────────────────────────────────
// Upstream's FIELD_PREP/FIELD_GET. Deliberately duplicated from
// `crate::mt76::regs` rather than imported: connac2 and mt76x02 are different
// silicon generations and this module must not grow a dependency on that one
// just to shift bits. `mask` must be non-zero — a zero mask makes
// `trailing_zeros()` return 32 and the shift overflows. No caller passes one.

/// Place `value` into the bit range described by `mask` (upstream `FIELD_PREP`).
pub const fn field_prep(mask: u32, value: u32) -> u32 {
    (value << mask.trailing_zeros()) & mask
}

/// Extract the bit range described by `mask` from `value` (upstream `FIELD_GET`).
pub const fn field_get(mask: u32, value: u32) -> u32 {
    (value & mask) >> mask.trailing_zeros()
}

// ── Identity ─────────────────────────────────────────────────────────────────
// The first three registers any bring-up touches, and the only three read so
// far on the target silicon. All three are physical CB-TOP addresses that need
// no window on either bus.

/// Chip id. **MEASURED `0x0000_7961`** on mds-o5p-3, which is how this port knows
/// it is talking to an MT7961 (the die inside every "MT7921AU") and not an
/// MT7922/MT7925. Read *before* claiming anything else — it is the cheapest
/// proof that the vendor-request path works at all, which is exactly what
/// `mt792xu_check_bus` uses it for (`mt792x_usb.c:116-129`).
pub const MT_HW_CHIPID: u32 = 0x7001_0200; // mt792x_regs.h:429

/// Chip revision. **MEASURED `0x0000_8a10`**, and the low half is what upstream
/// folds into `mdev->rev` (`mt7921/usb.c:214-215`: `chipid << 16 | (rev & 0xff)`).
/// ★ `0x8a10` is also the top half of the vendored patch blob's `hw_sw_ver`
/// field — see [`measured::PATCH_HDR_HW_SW_VER`]; that agreement is the evidence
/// the firmware in `fw/mt7961/` is the right firmware for this die.
pub const MT_HW_REV: u32 = 0x7001_0204; // mt792x_regs.h:430

/// Strap/bound register. `mt7921/pci.c:411-412` uses **bit 7** to separate an
/// MT7920 from an MT7961 when both report chip id `0x7961`; the two have
/// different capability sets, so a port that trusts the chip id alone can
/// declare capabilities the silicon does not have.
pub const MT_HW_BOUND: u32 = 0x7001_0020; // mt792x_regs.h:428
/// The MT7920-vs-MT7961 discriminator inside [`MT_HW_BOUND`]. Set ⇒ MT7920.
pub const MT_HW_BOUND_IS_MT7920: u32 = 0x0000_0080; // BIT(7) — mt7921/pci.c:411

// ── Firmware state and host/firmware ownership ───────────────────────────────
// Two near-identical "misc" registers in two different power domains. Getting
// them confused is Flag 4 above.

/// Base of the WF_TOP_MISC_ON block. Everything under it survives WFSYS reset,
/// which is why the firmware-state field lives here.
pub const MT_TOP_BASE: u32 = 0x1806_0000; // mt792x_regs.h:404

/// Host↔firmware ownership handshake for band 0. On PCIe the driver sets/clears
/// ownership here (via the L1 window, `mt7921/pci_mcu.c:9`); over USB the
/// equivalent traffic goes to [`MT_CONN_ON_LPCTL`] instead
/// (`mt792x_core.c:913,960`). Kept because a USB port that wants to know who
/// owns the MAC can still *read* it.
pub const MT_TOP_LPCR_HOST_BAND0: u32 = 0x1806_0010; // mt792x_regs.h:407
/// Firmware owns the MAC. Set by the firmware, polled by the host.
pub const MT_TOP_LPCR_HOST_FW_OWN: u32 = 0x0000_0001; // BIT(0) — mt792x_regs.h:408
/// Driver owns the MAC. The state a bring-up needs before it may write MAC
/// registers at all.
pub const MT_TOP_LPCR_HOST_DRV_OWN: u32 = 0x0000_0002; // BIT(1) — mt792x_regs.h:409

/// ⚠ The **real** `MT_TOP_MISC`, at `0x1806_00f0` — *not* the `0x7000_00f0` in the
/// bench notes (Flag 4). Carries the firmware state machine in its low three
/// bits. **Never read on this silicon.**
pub const MT_TOP_MISC: u32 = 0x1806_00f0; // mt792x_regs.h:411
/// Firmware state, bits `[2:0]` of [`MT_TOP_MISC`]. ⚠ Upstream then polls this
/// *mask* against the *other* register — `mt792x_core.c:986-987` does
/// `poll(MT_CONN_ON_MISC, MT_TOP_MISC_FW_STATE, MT_TOP_MISC2_FW_PWR_ON)`, i.e.
/// `(CONN_ON_MISC & 0x7) == 1`. That name mixing is upstream's, transcribed
/// rather than tidied so the port matches the code that is known to work.
pub const MT_TOP_MISC_FW_STATE: u32 = 0x0000_0007; // GENMASK(2,0) — mt792x_regs.h:412

/// Host CSR ownership control in the CONN_INFRA domain. This is the register the
/// **USB** driver actually drives for driver-own / firmware-own
/// (`mt792x_core.c:908-978`).
pub const MT_CONN_ON_LPCTL: u32 = 0x7c06_0010; // mt792x_regs.h:500
/// Write to hand the MAC back to firmware (`mt792x_core.c:960`).
pub const PCIE_LPCR_HOST_SET_OWN: u32 = 0x0000_0001; // BIT(0) — mt792x_regs.h:501
/// Write to take the MAC for the driver (`mt792x_core.c:913`).
pub const PCIE_LPCR_HOST_CLR_OWN: u32 = 0x0000_0002; // BIT(1) — mt792x_regs.h:502
/// Ownership-sync status: poll to **0** after CLR_OWN, to **4** after SET_OWN
/// (`mt792x_core.c:918-920, 961-963`). Despite the `PCIE_` prefix these three
/// are the USB path's too — the name is upstream's history, not a bus rule.
pub const PCIE_LPCR_HOST_OWN_SYNC: u32 = 0x0000_0004; // BIT(2) — mt792x_regs.h:503

/// ★ The firmware-liveness register, and the one register whose value we have
/// **MEASURED**: `0x0000_0000` on mds-o5p-3, i.e. [`MT_TOP_MISC2_FW_PWR_ON`]
/// clear ⇒ no firmware running, the part is exactly where a cold port wants it.
/// `mt792xu_mcu_power_on` polls this for `FW_PWR_ON` after the
/// [`access::MT_VEND_POWER_ON`] request (`mt792x_usb.c:214-231`), and
/// `mt7921u_probe` polls it for [`MT_TOP_MISC2_FW_N9_RDY`] to decide whether a
/// WFSYS reset is needed first (`mt7921/usb.c:218-222`).
pub const MT_CONN_ON_MISC: u32 = 0x7c06_00f0; // mt792x_regs.h:505
/// Power is on. The post-`POWER_ON`-request gate.
pub const MT_TOP_MISC2_FW_PWR_ON: u32 = 0x0000_0001; // BIT(0) — mt792x_regs.h:506
/// The N9 (WiFi) MCU core is running.
pub const MT_TOP_MISC2_FW_N9_ON: u32 = 0x0000_0002; // BIT(1) — mt792x_regs.h:507
/// Both bits: firmware fully up. The 1500 ms gate at the end of the firmware
/// download (`mt792x_core.c:1021-1022`) and the "is stale firmware already
/// resident?" probe test (`mt7921/usb.c:218`).
pub const MT_TOP_MISC2_FW_N9_RDY: u32 = 0x0000_0003; // GENMASK(1,0) — mt792x_regs.h:508

/// ROM-patch download state. Cleared by the SDIO path before pushing the patch
/// (`mt7921/sdio_mac.c:72`); the USB path never touches it. Transcribed because
/// a stuck patch state is a plausible suspect if a download hangs, and finding
/// the address again later is the expensive part.
pub const MT_CONN_STATUS: u32 = 0x7c05_3c10; // mt792x_regs.h:497
/// The patch-download-in-progress flag inside [`MT_CONN_STATUS`].
pub const MT_WIFI_PATCH_DL_STATE: u32 = 0x0000_0001; // BIT(0) — mt792x_regs.h:498

// ── WFSYS reset ──────────────────────────────────────────────────────────────
// ⚠ Transcribed, not endorsed. See "Never reset" in the module header. The USB
// and PCIe paths reset *different registers by different means*, which is the
// non-obvious part worth writing down.

/// ★ **ACCESS: UHW** — the WLAN-subsystem reset the **USB** driver uses.
/// `mt792xu_wfsys_reset` (`mt792x_usb.c:425-467`) sets then clears
/// [`MT_CBTOP_RGU_WF_SUBSYS_RST_WF_WHOLE_PATH`] here through
/// [`access::MT_VEND_DEV_MODE`]/[`access::MT_VEND_WRITE`], **not** through
/// `READ_EXT`/`WRITE_EXT`. Note this is a *different register* from the PCIe
/// path's [`pcie_only::MT_WFSYS_SW_RST_B`] — porting the PCIe sequence onto USB
/// would reset nothing.
pub const MT_CBTOP_RGU_WF_SUBSYS_RST: u32 = 0x7000_2600; // mt792x_regs.h:420-421
/// Whole-path WLAN subsystem reset bit.
pub const MT_CBTOP_RGU_WF_SUBSYS_RST_WF_WHOLE_PATH: u32 = 0x0000_0001; // BIT(0) — mt792x_regs.h:422

/// ★ **ACCESS: UHW** — polled after the reset above until
/// [`MT_UDMA_CONN_WFSYS_INIT_DONE`] is set (`mt792x_usb.c:455-461`, 2 tries ×
/// 100 ms). ⚠ It sits at `MT_UMAC(0xa20)`, the same block as
/// [`MT_UDMA_WLCFG_0`], yet takes the UHW path where WLCFG_0 does not — Flag 3.
pub const MT_UDMA_CONN_INFRA_STATUS: u32 = 0x7400_0a20; // mt792x_regs.h:484
/// WFSYS init-done, the reset-completed gate.
pub const MT_UDMA_CONN_WFSYS_INIT_DONE: u32 = 0x0040_0000; // BIT(22) — mt792x_regs.h:485
/// ★ **ACCESS: UHW** — status-source selector, written `0` after an MT7921 WFSYS
/// reset so [`MT_UDMA_CONN_INFRA_STATUS`] reports the WFSYS view
/// (`mt792x_usb.c:452-453`; `mt7921_wfsys_desc.need_status_sel = true`).
pub const MT_UDMA_CONN_INFRA_STATUS_SEL: u32 = 0x7400_0a24; // mt792x_regs.h:486

/// Number of WFSYS-init poll attempts upstream allows before giving up
/// (`mt792x.h:43`), each separated by 100 ms.
pub const MT792X_WFSYS_INIT_RETRY_COUNT: u32 = 2; // mt792x.h:43

// ── UMAC / UDMA — the USB DMA engine ─────────────────────────────────────────
// ⚠ Base is 0x7400_0000 on mt792x, NOT the 0x7c00_0000 of mt7615/regs.h:589.

/// Base of the UMAC block that owns the USB DMA path.
pub const MT_UMAC_BASE: u32 = 0x7400_0000; // mt792x_regs.h:462

/// TX queue select. Its one interesting bit is [`MT_FW_DL_EN`], which is what
/// makes the bulk-OUT `AC_BE` pipe carry firmware scatter payloads instead of
/// 802.11 frames.
pub const MT_UDMA_TX_QSEL: u32 = 0x7400_0008; // mt792x_regs.h:463
/// ★ Firmware-download mode. Set **before** the download and cleared **after**
/// (`mt7921/usb.c:76,82`), bracketing `mt7921_run_firmware`. Leaving it set
/// means data frames get eaten by the MCU; leaving it clear during download
/// means the firmware never arrives.
pub const MT_FW_DL_EN: u32 = 0x0000_0008; // BIT(3) — mt792x_regs.h:464

/// UDMA WLAN config word 1 — RX aggregation packet limit and the TX timeout.
pub const MT_UDMA_WLCFG_1: u32 = 0x7400_000c; // mt792x_regs.h:466
/// RX aggregation packet limit, bits `[7:0]`. **Cleared** by `mt792xu_dma_init`
/// (`mt792x_usb.c:410`), which together with the WLCFG_0 clears below means
/// **one RX unit per bulk-IN transfer** on this part — the same de-aggregated
/// arrangement the mt76x0 port relies on, reached by a different register.
pub const MT_WL_RX_AGG_PKT_LMT: u32 = 0x0000_00ff; // GENMASK(7,0) — mt792x_regs.h:467
/// TX timeout limit, bits `[27:8]`. Programmed with
/// [`MT792X_USB_TX_TIMEOUT_LIMIT`] (`mt792x_usb.c:404-406`).
pub const MT_WL_TX_TMOUT_LMT: u32 = 0x0fff_ff00; // GENMASK(27,8) — mt792x_regs.h:468

/// UDMA WLAN config word 0 — the master TX/RX enable for the USB DMA engine,
/// plus the busy bits a shutdown must wait on.
pub const MT_UDMA_WLCFG_0: u32 = 0x7400_0018; // mt792x_regs.h:470
/// RX aggregation timeout, bits `[7:0]`. Cleared by `mt792x_usb.c:408-409`.
pub const MT_WL_RX_AGG_TO: u32 = 0x0000_00ff; // GENMASK(7,0) — mt792x_regs.h:471
/// RX aggregation limit, bits `[15:8]`. Cleared by `mt792x_usb.c:408-409`.
pub const MT_WL_RX_AGG_LMT: u32 = 0x0000_ff00; // GENMASK(15,8) — mt792x_regs.h:472
/// Enable the TX-timeout function that [`MT_WL_TX_TMOUT_LMT`] parameterises
/// (`mt792x_usb.c:407`).
pub const MT_WL_TX_TMOUT_FUNC_EN: u32 = 0x0001_0000; // BIT(16) — mt792x_regs.h:473
/// TX data-path-header check enable. Never written by upstream on this part;
/// meaning undetermined, transcribed for completeness.
pub const MT_WL_TX_DPH_CHK_EN: u32 = 0x0002_0000; // BIT(17) — mt792x_regs.h:474
/// Pad RX max-packet-size to zero-length-packet boundaries. Set during
/// `mt792xu_dma_init` (`mt792x_usb.c:401-403`); upstream gives no reason, and the
/// obvious one — avoiding the USB short-packet ambiguity at a 512 B multiple — is
/// our inference, not upstream's statement.
pub const MT_WL_RX_MPSZ_PAD0: u32 = 0x0004_0000; // BIT(18) — mt792x_regs.h:475
/// RX flush. Set to drain the engine before a WFSYS reset
/// (`mt792x_usb.c:354`), cleared before enabling RX (`mt792x_usb.c:399`).
pub const MT_WL_RX_FLUSH: u32 = 0x0008_0000; // BIT(19) — mt792x_regs.h:476
/// ★ 1 µs tick enable. Set in `mt792xu_dma_init` (`mt792x_usb.c:401-403`). The
/// name says the UDMA timebase is microseconds; whether this is the same
/// timebase as the RXD group-2 timestamp is **undetermined** and matters for
/// `crate::mt7921`'s clock claim — settle it by measurement, not by reading.
pub const MT_TICK_1US_EN: u32 = 0x0010_0000; // BIT(20) — mt792x_regs.h:477
/// RX aggregation enable. ⚠ mt7663u sets it (`mt7615/usb_sdio.c:268`); the
/// mt792x USB path **does not** (`mt792x_usb.c:401-403` omits it), which is the
/// register-level reason RX arrives one unit per transfer.
pub const MT_WL_RX_AGG_EN: u32 = 0x0020_0000; // BIT(21) — mt792x_regs.h:478
/// RX enable.
pub const MT_WL_RX_EN: u32 = 0x0040_0000; // BIT(22) — mt792x_regs.h:479
/// TX enable.
pub const MT_WL_TX_EN: u32 = 0x0080_0000; // BIT(23) — mt792x_regs.h:480
/// RX engine busy. Polled to 0 (with [`MT_WL_TX_BUSY`]) before a reset
/// (`mt792x_usb.c:356-357`).
pub const MT_WL_RX_BUSY: u32 = 0x4000_0000; // BIT(30) — mt792x_regs.h:481
/// TX engine busy — see [`MT_WL_RX_BUSY`].
pub const MT_WL_TX_BUSY: u32 = 0x8000_0000; // BIT(31) — mt792x_regs.h:482

/// The value upstream programs into [`MT_WL_TX_TMOUT_LMT`]. Units are
/// undetermined upstream; if [`MT_TICK_1US_EN`] governs, 50 ms.
pub const MT792X_USB_TX_TIMEOUT_LIMIT: u32 = 50_000; // mt792x_usb.c:14
/// Milliseconds `mt792xu_wait_udma_idle` waits for the busy bits to clear.
pub const MT792X_USB_UDMA_IDLE_TIMEOUT_MS: u32 = 1_000; // mt792x_usb.c:15

/// ★ **ACCESS: UHW** — SuperSpeed-USB endpoint reset options. `mt792xu_epctl_rst_opt`
/// (`mt792x_usb.c:332-347`) clears the reset-opt bits for the bulk-OUT and
/// bulk-IN/interrupt-IN endpoints before DMA init and before a WFSYS reset, so
/// an endpoint stall does not tear the pipes down under us. Upstream's own
/// comment maps the bits: `[9:4]` = OUT bulk EP 4-9, `[21:20]` = IN bulk EP 4-5,
/// `[22]` = IN interrupt EP 6 — which is exactly the endpoint set MEASURED on
/// interface 3 of this part.
pub const MT_SSUSB_EPCTL_CSR_EP_RST_OPT: u32 = 0x7401_1890; // mt792x_regs.h:488-489
/// Reset-opt bits for bulk-OUT endpoints `0x04`..`0x09`, bits `[9:4]`.
pub const MT_SSUSB_EPCTL_RST_OPT_OUT_EP: u32 = 0x0000_03f0; // GENMASK(9,4) — mt792x_usb.c:343
/// Reset-opt bits for bulk-IN `0x84`/`0x85` and interrupt-IN `0x86`, bits `[22:20]`.
pub const MT_SSUSB_EPCTL_RST_OPT_IN_EP: u32 = 0x0070_0000; // GENMASK(22,20) — mt792x_usb.c:343

// ── UWFDMA0 — the physical (USB) alias of the WFDMA0 block ───────────────────
// This is the block whose PCIe-remapped twin is quarantined in `pcie_only`. The
// *bitfields* are shared; only the base differs, so the bit constants below
// carry upstream's `MT_WFDMA0_GLO_CFG_*` names and apply to both addresses.

/// Base of WFDMA0 as addressed over USB (`= 0x7c02_4000`).
pub const MT_UWFDMA0_BASE: u32 = 0x7c02_4000; // mt792x_regs.h:491

/// Global DMA config. `mt792xu_wfdma_init` (`mt792x_usb.c:279-291`) clears
/// [`MT_WFDMA0_GLO_CFG_OMIT_RX_INFO`] and sets OMIT_TX_INFO,
/// OMIT_RX_INFO_PFET2, FW_DWLD_BYPASS_DMASHDL and both DMA enables.
pub const MT_UWFDMA0_GLO_CFG: u32 = 0x7c02_4208; // mt792x_regs.h:492
/// Global DMA config extension 0. Not written on the USB path; transcribed
/// because the PCIe path programs prefetch/arbitration here and a USB port
/// investigating throughput will want the address.
pub const MT_UWFDMA0_GLO_CFG_EXT0: u32 = 0x7c02_42b0; // mt792x_regs.h:493
/// Global DMA config extension 1 — as [`MT_UWFDMA0_GLO_CFG_EXT0`].
pub const MT_UWFDMA0_GLO_CFG_EXT1: u32 = 0x7c02_42b4; // mt792x_regs.h:494

/// TX-ring prefetch control for ring `n` (`n` ∈ {0..4, 16, 17} upstream).
/// `mt792xu_dma_prefetch` (`mt792x_usb.c:262-277`) programs seven of them with
/// a 4-descriptor count and a per-ring base pointer.
pub const fn mt_uwfdma0_tx_ring_ext_ctrl(n: u32) -> u32 {
    0x7c02_4600 + (n << 2) // mt792x_regs.h:495
}
/// Descriptor count, bits `[7:0]` of a ring's ext-ctrl word.
pub const MT_WPDMA0_MAX_CNT_MASK: u32 = 0x0000_00ff; // GENMASK(7,0) — mt792x_regs.h:370
/// Ring base pointer, bits `[31:16]` of a ring's ext-ctrl word.
pub const MT_WPDMA0_BASE_PTR_MASK: u32 = 0xffff_0000; // GENMASK(31,16) — mt792x_regs.h:371

/// TX DMA enable. Set by `mt792xu_wfdma_init`.
pub const MT_WFDMA0_GLO_CFG_TX_DMA_EN: u32 = 0x0000_0001; // BIT(0) — mt792x_regs.h:293
/// TX DMA busy (read-only status).
pub const MT_WFDMA0_GLO_CFG_TX_DMA_BUSY: u32 = 0x0000_0002; // BIT(1) — mt792x_regs.h:294
/// RX DMA enable. Bounced off/on around the EP4 event-routing change
/// (`mt792x_usb.c:318-330`).
pub const MT_WFDMA0_GLO_CFG_RX_DMA_EN: u32 = 0x0000_0004; // BIT(2) — mt792x_regs.h:295
/// RX DMA busy. Polled to 0 before touching the EP4 routing.
pub const MT_WFDMA0_GLO_CFG_RX_DMA_BUSY: u32 = 0x0000_0008; // BIT(3) — mt792x_regs.h:296
/// DMA burst size selector, bits `[5:4]`. Untouched on USB.
pub const MT_WFDMA0_GLO_CFG_DMA_SIZE: u32 = 0x0000_0030; // GENMASK(5,4) — mt792x_regs.h:297
/// Write-back on TX descriptor done. PCIe-ring semantics; unused on USB.
pub const MT_WFDMA0_GLO_CFG_TX_WB_DDONE: u32 = 0x0000_0040; // BIT(6) — mt792x_regs.h:298
/// ★ Let firmware-download traffic bypass the DMA scheduler. Set by
/// `mt792xu_wfdma_init` — without it the scheduler quotas below throttle the
/// download.
pub const MT_WFDMA0_GLO_CFG_FW_DWLD_BYPASS_DMASHDL: u32 = 0x0000_0200; // BIT(9) — mt792x_regs.h:299
/// Disable the FIFO fullness check. Untouched on USB.
pub const MT_WFDMA0_GLO_CFG_FIFO_DIS_CHECK: u32 = 0x0000_0800; // BIT(11) — mt792x_regs.h:300
/// FIFO little-endian. Untouched on USB.
pub const MT_WFDMA0_GLO_CFG_FIFO_LITTLE_ENDIAN: u32 = 0x0000_1000; // BIT(12) — mt792x_regs.h:301
/// Write-back on RX descriptor done. PCIe-ring semantics; unused on USB.
pub const MT_WFDMA0_GLO_CFG_RX_WB_DDONE: u32 = 0x0000_2000; // BIT(13) — mt792x_regs.h:302
/// Chain the CSR display base pointer. Untouched on USB.
pub const MT_WFDMA0_GLO_CFG_CSR_DISP_BASE_PTR_CHAIN_EN: u32 = 0x0000_8000; // BIT(15) — mt792x_regs.h:303
/// Loopback RX queue select. Untouched on USB.
pub const MT_WFDMA0_GLO_CFG_CSR_LBK_RX_Q_SEL_EN: u32 = 0x0010_0000; // BIT(20) — mt792x_regs.h:304
/// Omit the RX info word on prefetch path 2. **Set** on USB.
pub const MT_WFDMA0_GLO_CFG_OMIT_RX_INFO_PFET2: u32 = 0x0020_0000; // BIT(21) — mt792x_regs.h:305
/// 36-bit address extension. PCIe only.
pub const MT_WFDMA0_GLO_CFG_ADDR_EXT_EN: u32 = 0x0400_0000; // BIT(26) — mt792x_regs.h:306
/// ★ Omit the RX info word. **Cleared** on USB (`mt792x_usb.c:285`) — i.e. the
/// bulk-IN stream *does* carry the DMA info header, which is what the RX parser
/// in `mac.rs` has to skip before the RXD.
pub const MT_WFDMA0_GLO_CFG_OMIT_RX_INFO: u32 = 0x0800_0000; // BIT(27) — mt792x_regs.h:307
/// ★ Omit the TX info word. **Set** on USB (`mt792x_usb.c:286-291`) — the TX
/// path therefore does *not* prepend a DMA info word ahead of the TXD.
pub const MT_WFDMA0_GLO_CFG_OMIT_TX_INFO: u32 = 0x1000_0000; // BIT(28) — mt792x_regs.h:308
/// Disable DMA clock gating. Untouched on USB.
pub const MT_WFDMA0_GLO_CFG_CLK_GAT_DIS: u32 = 0x4000_0000; // BIT(30) — mt792x_regs.h:309

/// Routes MCU events to bulk-IN endpoint 4 (`0x84`) instead of the response
/// pipe. `mt792xu_dma_rx_evt_ep4` (`mt792x_usb.c:318-330`) sets
/// [`MT_WFDMA_HOST_CONFIG_USB_RXEVT_EP4_EN`] here between an RX-DMA off/on
/// bounce. ⚠ This is the *physical* form (`0x7c02_7030`) of an address whose
/// block is quarantined in [`pcie_only`] — upstream states it directly, so it
/// is not a derivation.
pub const MT_WFDMA_HOST_CONFIG: u32 = 0x7c02_7030; // mt792x_regs.h:459
/// Enable MCU-event delivery on USB bulk-IN EP4.
pub const MT_WFDMA_HOST_CONFIG_USB_RXEVT_EP4_EN: u32 = 0x0000_0040; // BIT(6) — mt792x_regs.h:460

// ── DMASHDL — the DMA scheduler ──────────────────────────────────────────────
// Programmed wholesale by `mt792xu_wfdma_init` (`mt792x_usb.c:293-315`).
// Group quotas decide how much buffer each traffic class may hold; getting them
// wrong shows up as TX stalling rather than as an error.

/// Base of the DMA scheduler block.
pub const MT_DMA_SHDL_BASE: u32 = 0x7c02_6000; // mt792x_regs.h:437
/// Software control. Its [`MT_DMASHDL_DMASHDL_BYPASS`] bit disables the
/// scheduler outright; the USB path does not use it (it uses the per-download
/// bypass in [`MT_WFDMA0_GLO_CFG_FW_DWLD_BYPASS_DMASHDL`] instead).
pub const MT_DMASHDL_SW_CONTROL: u32 = 0x7c02_6004; // mt792x_regs.h:438
/// Bypass the DMA scheduler entirely.
pub const MT_DMASHDL_DMASHDL_BYPASS: u32 = 0x1000_0000; // BIT(28) — mt792x_regs.h:439
/// Optional-features word. Written `0x7004_801c` by mt7663u
/// (`mt7615/usb_sdio.c:257`); **not** written by mt792x. Meaning undetermined.
pub const MT_DMASHDL_OPTIONAL: u32 = 0x7c02_6008; // mt792x_regs.h:440
/// Page config. `mt792xu_wfdma_init` clears [`MT_DMASHDL_GROUP_SEQ_ORDER`] here.
pub const MT_DMASHDL_PAGE: u32 = 0x7c02_600c; // mt792x_regs.h:441
/// Force strict group sequence ordering. Cleared on USB; upstream gives no
/// reason.
pub const MT_DMASHDL_GROUP_SEQ_ORDER: u32 = 0x0001_0000; // BIT(16) — mt792x_regs.h:442
/// Page-refill config. Programmed `0xffe0_0000` into [`MT_DMASHDL_REFILL_MASK`].
pub const MT_DMASHDL_REFILL: u32 = 0x7c02_6010; // mt792x_regs.h:443
/// Refill field, bits `[31:16]`.
pub const MT_DMASHDL_REFILL_MASK: u32 = 0xffff_0000; // GENMASK(31,16) — mt792x_regs.h:444
/// Max packet size for the PLE/PSE pools. Programmed PLE=1, PSE=0 on USB.
pub const MT_DMASHDL_PKT_MAX_SIZE: u32 = 0x7c02_601c; // mt792x_regs.h:445
/// PLE max packet size, bits `[11:0]`.
pub const MT_DMASHDL_PKT_MAX_SIZE_PLE: u32 = 0x0000_0fff; // GENMASK(11,0) — mt792x_regs.h:446
/// PSE max packet size, bits `[27:16]`.
pub const MT_DMASHDL_PKT_MAX_SIZE_PSE: u32 = 0x0fff_0000; // GENMASK(27,16) — mt792x_regs.h:447

/// Per-group page quota, `n` ∈ 0..16. USB programs groups 0..4 as
/// `min = 3, max = 0xfff` and groups 5..15 as `min = 0, max = 0`
/// (`mt792x_usb.c:299-306`) — i.e. only the first five groups get any buffer.
pub const fn mt_dmashdl_group_quota(n: u32) -> u32 {
    0x7c02_6020 + (n << 2) // mt792x_regs.h:449
}
/// Minimum guaranteed pages, bits `[11:0]`.
pub const MT_DMASHDL_GROUP_QUOTA_MIN: u32 = 0x0000_0fff; // GENMASK(11,0) — mt792x_regs.h:450
/// Maximum pages, bits `[27:16]`.
pub const MT_DMASHDL_GROUP_QUOTA_MAX: u32 = 0x0fff_0000; // GENMASK(27,16) — mt792x_regs.h:451

/// Queue→group map, `n` ∈ 0..4 (eight 4-bit entries per word). USB writes
/// `0x3201_3201`, `0x3201_3201`, `0x5555_5444`, `0x5555_5444`
/// (`mt792x_usb.c:307-310`).
pub const fn mt_dmashdl_q_map(n: u32) -> u32 {
    0x7c02_6060 + (n << 2) // mt792x_regs.h:453
}
/// One queue's group id, 4 bits wide.
pub const MT_DMASHDL_Q_MAP_MASK: u32 = 0x0000_000f; // GENMASK(3,0) — mt792x_regs.h:454
/// Bit position of entry `n` within a [`mt_dmashdl_q_map`] word.
pub const fn mt_dmashdl_q_map_shift(n: u32) -> u32 {
    4 * (n % 8) // mt792x_regs.h:455
}

/// Group scheduling priority order, `n` ∈ {0, 1}. USB writes `0x7654_0132` and
/// `0xFEDC_BA98` (`mt792x_usb.c:312-313`).
pub const fn mt_dmashdl_sched_set(n: u32) -> u32 {
    0x7c02_6070 + (n << 2) // mt792x_regs.h:457
}

// ── MCU-adjacent registers (command/event IDs live in `mcu.rs`, not here) ─────

/// Firmware operating-mode selector. Written [`MT_SWDEF_NORMAL_MODE`] **before**
/// the firmware download (`mt7921/init.c:91`, `mt7921/usb.c:122`) — upstream's
/// comment: *"force firmware operation mode into normal state, which should be
/// set before firmware download stage."*
pub const MT_SWDEF_MODE: u32 = 0x0041_f23c; // mt792x_regs.h:397-399
/// Normal operating mode — the only one this port wants.
pub const MT_SWDEF_NORMAL_MODE: u32 = 0; // mt792x_regs.h:400
/// I-Capture (raw-IQ capture) mode. Not used here; recorded because it is the
/// only visible hook toward raw PHY samples on this part.
pub const MT_SWDEF_ICAP_MODE: u32 = 1; // mt792x_regs.h:401
/// Spectrum-scan mode — as [`MT_SWDEF_ICAP_MODE`].
pub const MT_SWDEF_SPECTRUM_MODE: u32 = 2; // mt792x_regs.h:402

/// USB MCU event/handshake scratch register. `mt7921u_resume` polls it for
/// [`MT_WF_SW_SER_TRIGGER_SUSPEND`] / [`MT_WF_SW_SER_DONE_SUSPEND`] to decide
/// whether the DMA engine must be re-initialised after a bus suspend
/// (`mt7921/usb.c:296-309`).
pub const MT_WF_SW_DEF_CR_USB_MCU_EVENT: u32 = 0x0040_1a28; // mt792x_regs.h:510-511
/// A system-error recovery was triggered across the suspend.
pub const MT_WF_SW_SER_TRIGGER_SUSPEND: u32 = 0x0000_0040; // BIT(6) — mt792x_regs.h:512
/// That recovery completed; the host clears the whole word on seeing it.
pub const MT_WF_SW_SER_DONE_SUSPEND: u32 = 0x0000_0080; // BIT(7) — mt792x_regs.h:513

/// A firmware-visible scratch word the driver uses as a "DMA needs re-init"
/// flag across suspend/resume. Set at the end of `mt792xu_wfdma_init`
/// (`mt792x_usb.c:315`) and tested by `mt792x_dma_need_reinit`.
pub const MT_WFDMA_DUMMY_CR: u32 = 0x5400_0120; // mt792x_regs.h:414-417
/// The re-init flag inside [`MT_WFDMA_DUMMY_CR`].
pub const MT_WFDMA_NEED_REINIT: u32 = 0x0000_0002; // BIT(1) — mt792x_regs.h:418

// ═════════════════════════════════════════════════════════════════════════════
// MAC blocks. Every address below is a **physical** `0x82xx_xxxx` LMAC address
// and goes onto the wire verbatim over USB (Flag 1). MT7921 is single-band, so
// in practice `band` is always 0; the band-1 bases are transcribed because
// `mt7921_mac_init` loops `for (i = 0; i < 2; i++) mt792x_mac_init_band(dev, i)`
// (`mt7921/init.c:77-78`) and a faithful port needs both.
// ═════════════════════════════════════════════════════════════════════════════

// ── RMAC — receive MAC, i.e. the RX filter and the airtime counters ──────────

/// Base of the RMAC block for `band` (`mt792x_regs.h:220`).
pub const fn mt_wf_rmac_base(band: u32) -> u32 {
    if band != 0 { 0x820f_5000 } else { 0x820e_5000 }
}
/// An offset within the RMAC block for `band`.
pub const fn mt_wf_rmac(band: u32, ofs: u32) -> u32 {
    mt_wf_rmac_base(band) + ofs // mt792x_regs.h:221
}

/// ★ The receive-filter control register. **See Flag 5**: on MT7921 the driver
/// does not write this — `mt7921_configure_filter` (`mt7921/main.c:666-693`)
/// hands an abstract flag word to the firmware via `mt7921_mcu_set_rxfilter`,
/// and `mt7921/mcu.c:1106-1122` passes the `MT_WF_RFCR_DROP_*` bits below as
/// *arguments* to that command. The address is transcribed so a direct write can
/// be tried and measured; whether it takes effect while firmware owns the MAC is
/// undetermined.
pub const fn mt_wf_rfcr(band: u32) -> u32 {
    mt_wf_rmac(band, 0x000) // mt792x_regs.h:223
}
/// Drop STBC multicast. A monitor-mode port wants every drop bit **clear**.
pub const MT_WF_RFCR_DROP_STBC_MULTI: u32 = 0x0000_0001; // BIT(0) — mt792x_regs.h:224
/// Drop frames that failed FCS. Clear it to see the errored frames a
/// contention estimate needs.
pub const MT_WF_RFCR_DROP_FCSFAIL: u32 = 0x0000_0002; // BIT(1) — mt792x_regs.h:225
/// Drop frames with an unexpected protocol version.
pub const MT_WF_RFCR_DROP_VERSION: u32 = 0x0000_0008; // BIT(3) — mt792x_regs.h:226
/// Drop probe requests.
pub const MT_WF_RFCR_DROP_PROBEREQ: u32 = 0x0000_0010; // BIT(4) — mt792x_regs.h:227
/// Drop multicast. ⚠ Named-radio traffic is broadcast — never set this.
pub const MT_WF_RFCR_DROP_MCAST: u32 = 0x0000_0020; // BIT(5) — mt792x_regs.h:228
/// Drop broadcast. ⚠ Same warning as [`MT_WF_RFCR_DROP_MCAST`], more so.
pub const MT_WF_RFCR_DROP_BCAST: u32 = 0x0000_0040; // BIT(6) — mt792x_regs.h:229
/// Drop multicast not matching the multicast filter table.
pub const MT_WF_RFCR_DROP_MCAST_FILTERED: u32 = 0x0000_0080; // BIT(7) — mt792x_regs.h:230
/// Drop when addr3 matches our own MAC.
pub const MT_WF_RFCR_DROP_A3_MAC: u32 = 0x0000_0100; // BIT(8) — mt792x_regs.h:231
/// Drop when addr3 matches the BSSID.
pub const MT_WF_RFCR_DROP_A3_BSSID: u32 = 0x0000_0200; // BIT(9) — mt792x_regs.h:232
/// Drop when addr2 matches the BSSID.
pub const MT_WF_RFCR_DROP_A2_BSSID: u32 = 0x0000_0400; // BIT(10) — mt792x_regs.h:233
/// Drop beacons from other BSSes. ★ The one drop bit upstream toggles at
/// runtime, through the MCU, for beacon filtering (`mt7921/mcu.c:1107,1121`).
pub const MT_WF_RFCR_DROP_OTHER_BEACON: u32 = 0x0000_0800; // BIT(11) — mt792x_regs.h:234
/// Drop hardware frame-report frames.
pub const MT_WF_RFCR_DROP_FRAME_REPORT: u32 = 0x0000_1000; // BIT(12) — mt792x_regs.h:235
/// Drop reserved control subtypes.
pub const MT_WF_RFCR_DROP_CTL_RSV: u32 = 0x0000_2000; // BIT(13) — mt792x_regs.h:236
/// Drop CTS. ⚠ Clear if you want to *observe* the NAV others assert.
pub const MT_WF_RFCR_DROP_CTS: u32 = 0x0000_4000; // BIT(14) — mt792x_regs.h:237
/// Drop RTS — see [`MT_WF_RFCR_DROP_CTS`].
pub const MT_WF_RFCR_DROP_RTS: u32 = 0x0000_8000; // BIT(15) — mt792x_regs.h:238
/// Drop retransmitted duplicates. ⚠ Clear it: our own dedup wants to see them.
pub const MT_WF_RFCR_DROP_DUPLICATE: u32 = 0x0001_0000; // BIT(16) — mt792x_regs.h:239
/// Drop frames from other BSSes. ⚠ Never set on a promiscuous port.
pub const MT_WF_RFCR_DROP_OTHER_BSS: u32 = 0x0002_0000; // BIT(17) — mt792x_regs.h:240
/// Drop unicast addressed to someone else. ⚠ Never set on a promiscuous port.
pub const MT_WF_RFCR_DROP_OTHER_UC: u32 = 0x0004_0000; // BIT(18) — mt792x_regs.h:241
/// Drop frames whose TIM does not concern us.
pub const MT_WF_RFCR_DROP_OTHER_TIM: u32 = 0x0008_0000; // BIT(19) — mt792x_regs.h:242
/// Drop NDPA (sounding announcement) frames.
pub const MT_WF_RFCR_DROP_NDPA: u32 = 0x0010_0000; // BIT(20) — mt792x_regs.h:243
/// Drop control frames not addressed to us.
pub const MT_WF_RFCR_DROP_UNWANTED_CTL: u32 = 0x0020_0000; // BIT(21) — mt792x_regs.h:244

/// Second receive-filter word — the response-frame drops that [`mt_wf_rfcr`]
/// does not cover.
pub const fn mt_wf_rfcr1(band: u32) -> u32 {
    mt_wf_rmac(band, 0x004) // mt792x_regs.h:246
}
/// Drop ACK frames.
pub const MT_WF_RFCR1_DROP_ACK: u32 = 0x0000_0010; // BIT(4) — mt792x_regs.h:247
/// Drop beamforming-report-poll frames.
pub const MT_WF_RFCR1_DROP_BF_POLL: u32 = 0x0000_0020; // BIT(5) — mt792x_regs.h:248
/// Drop BlockAck frames.
pub const MT_WF_RFCR1_DROP_BA: u32 = 0x0000_0040; // BIT(6) — mt792x_regs.h:249
/// Drop CF-End frames.
pub const MT_WF_RFCR1_DROP_CFEND: u32 = 0x0000_0080; // BIT(7) — mt792x_regs.h:250
/// Drop CF-Ack frames.
pub const MT_WF_RFCR1_DROP_CFACK: u32 = 0x0000_0100; // BIT(8) — mt792x_regs.h:251

/// ★ Airtime-measurement control. `mt792x_mac_init_band` sets
/// [`MT_WF_RMAC_MIB_RXTIME_EN`] here (`mt792x_mac.c:295`) and every survey pass
/// sets [`MT_WF_RMAC_MIB_RXTIME_CLR`] to zero the OBSS accumulator
/// (`mt792x_mac.c:262`). Without the enable, the OBSS counter never moves — the
/// same "decided but unactuated" failure mode this codebase keeps finding.
pub const fn mt_wf_rmac_mib_time0(band: u32) -> u32 {
    mt_wf_rmac(band, 0x03c4) // mt792x_regs.h:253
}
/// Clear-on-write for the RX-time accumulators.
pub const MT_WF_RMAC_MIB_RXTIME_CLR: u32 = 0x8000_0000; // BIT(31) — mt792x_regs.h:254
/// Enable RX-time accumulation.
pub const MT_WF_RMAC_MIB_RXTIME_EN: u32 = 0x4000_0000; // BIT(30) — mt792x_regs.h:255

/// Airtime register 0. Carries the same CLR/EN bits as
/// [`mt_wf_rmac_mib_time0`] and is set/cleared alongside it
/// (`mt792x_mac.c:296, 211`). Upstream never reads a *value* from it; what it
/// counts is undetermined.
pub const fn mt_wf_rmac_mib_airtime0(band: u32) -> u32 {
    mt_wf_rmac(band, 0x0380) // mt792x_regs.h:259
}
/// ★ **OBSS airtime** — microseconds the channel was occupied by *other* BSSes
/// since the last clear. Read by `mt792x_phy_update_channel`
/// (`mt792x_mac.c:236-237`) and folded into the survey's `cc_rx`. This is the
/// frame-free occupancy sensor on this part: an interference estimate that costs
/// one EP0 read and needs no decode, exactly like `REG_RXERR_RPT` on the 8812au.
pub const fn mt_wf_rmac_mib_airtime14(band: u32) -> u32 {
    mt_wf_rmac(band, 0x03b8) // mt792x_regs.h:257
}
/// OBSS time, bits `[23:0]`. **Microseconds** — inferred, not stated: upstream
/// adds it into `cc_rx`, and `mac80211.c:1012` accumulates `cc_active` with
/// `ktime_to_us`, then `mac80211.c:1174` divides `cc_active` by 1000 for the
/// survey's milliseconds. The unit follows from that arithmetic. 24 bits wrap
/// after ~16.8 s of occupancy.
pub const MT_MIB_OBSSTIME_MASK: u32 = 0x00ff_ffff; // GENMASK(23,0) — mt792x_regs.h:258

// ── WF_DMA — the per-band RX descriptor config ──────────────────────────────

/// Base of the per-band WF_DMA block (`mt792x_regs.h:56`).
pub const fn mt_wf_dma_base(band: u32) -> u32 {
    if band != 0 { 0x820f_7000 } else { 0x820e_7000 }
}
/// An offset within the WF_DMA block for `band`.
pub const fn mt_wf_dma(band: u32, ofs: u32) -> u32 {
    mt_wf_dma_base(band) + ofs // mt792x_regs.h:57
}

/// ★ RX descriptor control. Two things matter here, both set by
/// `mt792x_mac_init_band` (`mt792x_mac.c:302-304`):
/// [`MT_DMA_DCR0_MAX_RX_LEN`] ← 1536, and [`MT_DMA_DCR0_RXD_G5_EN`] **cleared**.
pub const fn mt_dma_dcr0(band: u32) -> u32 {
    mt_wf_dma(band, 0x000) // mt792x_regs.h:59
}
/// Maximum RX length, bits `[15:3]`. Programmed 1536.
pub const MT_DMA_DCR0_MAX_RX_LEN: u32 = 0x0000_fff8; // GENMASK(15,3) — mt792x_regs.h:60
/// ★ RXD **group 5** enable — the per-frame *rate* report. Upstream **clears**
/// it, with the comment *"disable rx rate report by default due to hw issues"*
/// (`mt792x_mac.c:303-304`). Consequence for this port: the RX rate is not
/// available per frame unless we set this and measure whether the "hw issues"
/// bite. ⚠ **Group 5 is not group 2.** The headline RX timestamp lives in
/// **group 2** (`mt7921/mac.c:307-309`, gated by `MT_RXD1_NORMAL_GROUP_2`,
/// `mt76_connac2_mac.h:196`) and is entirely unaffected by this bit. Whether
/// group 2 is present by default, or needs enabling elsewhere, is
/// **undetermined** — there is no `RXD_G2_EN` anywhere upstream, which suggests
/// it is unconditional, but that is inference.
pub const MT_DMA_DCR0_RXD_G5_EN: u32 = 0x0080_0000; // BIT(23) — mt792x_regs.h:61

// ── LPON — the TSF. ★ The clock this port exists to expose. ─────────────────
//
// The per-frame RX timestamp in RXD group 2 is only the **low 32 bits** and is
// flagged RX_FLAG_MACTIME_START, i.e. it is the LPON TSF latched at the start of
// the PPDU (`mt7921/mac.c:307-309`). To turn that into a 64-bit common-view
// stamp the port must read the full counter here and stitch the high half — at
// 1 µs/tick the low half wraps every ~71.6 minutes, so the stitch is cheap but
// mandatory. ⚠ At 268 µs per EP0 round trip, reading UTTR0/UTTR1 costs ~536 µs;
// that read belongs in a slow loop, never on the RX path.

/// Base of the per-band LPON block (`mt792x_regs.h:72`).
pub const fn mt_wf_lpon_base(band: u32) -> u32 {
    if band != 0 { 0x820f_b000 } else { 0x820e_b000 }
}

/// ★ TSF **low** 32 bits. Read first, after arming [`mt_lpon_tcr`] with
/// [`MT_LPON_TCR_SW_READ`] (`mt792x_core.c:258-260`). This is the word the RXD
/// group-2 timestamp is a snapshot of.
pub const fn mt_lpon_uttr0(band: u32) -> u32 {
    mt_wf_lpon_base(band) + 0x080 // mt792x_regs.h:75
}
/// ★ TSF **high** 32 bits — `tsf.t32[1]` in `mt792x_core.c:260`. Note the word
/// order is the plain one here (`t32[0]` = low, `t32[1]` = high), unlike the
/// mt76x02 family where `mt76x02_usb_core.c:155-157` gets it backwards; no
/// correction is needed on this part.
pub const fn mt_lpon_uttr1(band: u32) -> u32 {
    mt_wf_lpon_base(band) + 0x084 // mt792x_regs.h:76
}

/// TSF control for hardware-BSSID slot `n` (`n` ≤ 3, `HW_BSSID_MAX`,
/// `mt76_connac.h:73-77`). Arm it before reading or after writing UTTR0/1.
/// ⚠ Stride differs across families: mt792x uses `0x0a8 + n*4`
/// (`mt792x_regs.h:78`), mt7915 uses `0x0a8 + ((n*4) << 1)`
/// (`mt7915/regs.h:292-293`). This port follows mt792x.
pub const fn mt_lpon_tcr(band: u32, n: u32) -> u32 {
    mt_wf_lpon_base(band) + 0x0a8 + n * 4 // mt792x_regs.h:78
}
/// Software TSF mode field, bits `[1:0]`.
pub const MT_LPON_TCR_SW_MODE: u32 = 0x0000_0003; // GENMASK(1,0) — mt792x_regs.h:79
/// Latch the TSF into UTTR0/UTTR1 for a software read. ⚠ Same value as
/// [`MT_LPON_TCR_SW_MODE`]: `mt792x_core.c:258` does `mt76_set(…, SW_MODE)`,
/// which ORs `0b11`. mt792x omits the explicit `SW_READ` name that
/// `mt7915/regs.h:299` defines with the identical value; it is restored here so
/// the intent at the call site is legible.
pub const MT_LPON_TCR_SW_READ: u32 = 0x0000_0003; // GENMASK(1,0) — mt7915/regs.h:299
/// Commit UTTR0/UTTR1 into the running TSF (`mt792x_core.c:286`).
pub const MT_LPON_TCR_SW_WRITE: u32 = 0x0000_0001; // BIT(0) — mt792x_regs.h:80
/// ★ Add UTTR0/UTTR1 to the running TSF instead of replacing it — a **relative**
/// trim, which is the actuator a disciplined common-view clock wants (the
/// 8733b work found trim, not the sensor, to be the limit). Absent from
/// `mt792x_regs.h`; taken from `mt7915/regs.h:298`, where the surrounding three
/// bits are byte-identical. **Unverified on this silicon** — the inference that
/// the fourth bit matches too is exactly the kind of guess that has a ~0% hit
/// rate here, so measure before trusting it.
pub const MT_LPON_TCR_SW_ADJUST: u32 = 0x0000_0002; // BIT(1) — mt7915/regs.h:298

// ── MIB — the statistics block, including the airtime counters ──────────────

/// Base of the per-band MIB block (`mt792x_regs.h:97`).
pub const fn mt_wf_mib_base(band: u32) -> u32 {
    if band != 0 { 0x820f_d000 } else { 0x820e_d000 }
}
/// An offset within the MIB block for `band`.
pub const fn mt_wf_mib(band: u32, ofs: u32) -> u32 {
    mt_wf_mib_base(band) + ofs // mt792x_regs.h:98
}

/// ★ MIB control. `mt792x_mac_init_band` sets [`MT_MIB_TXDUR_EN`] and
/// [`MT_MIB_RXDUR_EN`] here (`mt792x_mac.c:298-300`) — *"enable MIB tx-rx time
/// reporting"*. Without them [`mt_mib_sdr36`]/[`mt_mib_sdr37`] read zero.
pub const fn mt_mib_scr1(band: u32) -> u32 {
    mt_wf_mib(band, 0x004) // mt792x_regs.h:100
}
/// Enable TX-duration accumulation.
pub const MT_MIB_TXDUR_EN: u32 = 0x0000_0100; // BIT(8) — mt792x_regs.h:101
/// Enable RX-duration accumulation.
pub const MT_MIB_RXDUR_EN: u32 = 0x0000_0200; // BIT(9) — mt792x_regs.h:102

/// FCS-error counter (`mt792x_mac.c:84-85`). A cheap PER proxy that needs no
/// per-frame work.
pub const fn mt_mib_sdr3(band: u32) -> u32 {
    mt_wf_mib(band, 0x698) // mt792x_regs.h:104
}
/// FCS-error count, bits `[31:16]`.
pub const MT_MIB_SDR3_FCS_ERR_MASK: u32 = 0xffff_0000; // GENMASK(31,16) — mt792x_regs.h:105

/// RX MPDU count (`mt792x_mac.c:113`).
pub const fn mt_mib_sdr5(band: u32) -> u32 {
    mt_wf_mib(band, 0x780) // mt792x_regs.h:107
}

/// ★ **Channel-busy time.** Read by `mt792x_phy_update_channel`
/// (`mt792x_mac.c:230-231`) into the survey's `cc_busy`, and read-to-reset by
/// `mt792x_mac_reset_counters` (`mt792x_mac.c:206`). Together with
/// [`mt_mib_sdr36`], [`mt_mib_sdr37`] and [`mt_wf_rmac_mib_airtime14`] this is
/// the full occupancy picture: busy = tx + rx + obss + everything else the CCA
/// saw.
pub const fn mt_mib_sdr9(band: u32) -> u32 {
    mt_wf_mib(band, 0x02c) // mt792x_regs.h:109
}
/// Busy time, bits `[23:0]`. Microseconds — same derivation as
/// [`MT_MIB_OBSSTIME_MASK`]. Wraps after ~16.8 s.
pub const MT_MIB_SDR9_BUSY_MASK: u32 = 0x00ff_ffff; // GENMASK(23,0) — mt792x_regs.h:110

/// TX A-MPDU count (`mt792x_mac.c:95`).
pub const fn mt_mib_sdr12(band: u32) -> u32 {
    mt_wf_mib(band, 0x558) // mt792x_regs.h:112
}
/// TX MPDU attempts (`mt792x_mac.c:96`). With [`mt_mib_sdr15`] this gives a
/// hardware-measured delivery ratio without touching the frame path.
pub const fn mt_mib_sdr14(band: u32) -> u32 {
    mt_wf_mib(band, 0x564) // mt792x_regs.h:113
}
/// TX MPDU successes (`mt792x_mac.c:97`) — the denominator's partner.
pub const fn mt_mib_sdr15(band: u32) -> u32 {
    mt_wf_mib(band, 0x568) // mt792x_regs.h:114
}

/// A second busy-time register. ⚠ **Never read by upstream** on this part — the
/// survey uses [`mt_mib_sdr9`]. Transcribed because the two disagreeing would be
/// informative; what it actually counts is undetermined.
pub const fn mt_mib_sdr16(band: u32) -> u32 {
    mt_wf_mib(band, 0x048) // mt792x_regs.h:116
}
/// Busy time, bits `[23:0]`, of [`mt_mib_sdr16`].
pub const MT_MIB_SDR16_BUSY_MASK: u32 = 0x00ff_ffff; // GENMASK(23,0) — mt792x_regs.h:117

/// RX A-MPDU count (`mt792x_mac.c:114`).
pub const fn mt_mib_sdr22(band: u32) -> u32 {
    mt_wf_mib(band, 0x770) // mt792x_regs.h:119
}
/// RX A-MPDU bytes (`mt792x_mac.c:115`).
pub const fn mt_mib_sdr23(band: u32) -> u32 {
    mt_wf_mib(band, 0x774) // mt792x_regs.h:120
}
/// RX BlockAck count (`mt792x_mac.c:116`).
pub const fn mt_mib_sdr31(band: u32) -> u32 {
    mt_wf_mib(band, 0x55c) // mt792x_regs.h:121
}

/// Implicit/explicit beamforming TX counts, packed
/// (`mt792x_mac.c:99-101`).
pub const fn mt_mib_sdr32(band: u32) -> u32 {
    mt_wf_mib(band, 0x7a8) // mt792x_regs.h:123
}
/// Implicit-BF count, bits `[31:16]`. ⚠ Upstream names these two `MT_MIB_SDR9_*`
/// although they belong to SDR32; the misnomer is upstream's and is kept so a
/// grep against the kernel tree still lands.
pub const MT_MIB_SDR9_IBF_CNT_MASK: u32 = 0xffff_0000; // GENMASK(31,16) — mt792x_regs.h:124
/// Explicit-BF count, bits `[15:0]` of [`mt_mib_sdr32`].
pub const MT_MIB_SDR9_EBF_CNT_MASK: u32 = 0x0000_ffff; // GENMASK(15,0) — mt792x_regs.h:125

/// MU beamforming TX count. Declared upstream, read by nothing.
pub const fn mt_mib_sdr34(band: u32) -> u32 {
    mt_wf_mib(band, 0x090) // mt792x_regs.h:127
}
/// MU-BF TX count, bits `[15:0]`.
pub const MT_MIB_MU_BF_TX_CNT: u32 = 0x0000_ffff; // GENMASK(15,0) — mt792x_regs.h:128

/// ★ **TX airtime.** Read into the survey's `cc_tx` (`mt792x_mac.c:232-233`).
/// The measured half of an airtime-lease accounting loop: what the lease
/// *actually* consumed, as opposed to what it was granted.
pub const fn mt_mib_sdr36(band: u32) -> u32 {
    mt_wf_mib(band, 0x054) // mt792x_regs.h:130
}
/// TX time, bits `[23:0]`, microseconds.
pub const MT_MIB_SDR36_TXTIME_MASK: u32 = 0x00ff_ffff; // GENMASK(23,0) — mt792x_regs.h:131
/// ★ **RX airtime** — `cc_rx`/`cc_bss_rx` (`mt792x_mac.c:234-235`).
pub const fn mt_mib_sdr37(band: u32) -> u32 {
    mt_wf_mib(band, 0x058) // mt792x_regs.h:132
}
/// RX time, bits `[23:0]`, microseconds.
pub const MT_MIB_SDR37_RXTIME_MASK: u32 = 0x00ff_ffff; // GENMASK(23,0) — mt792x_regs.h:133

/// Debug register 8. Declared upstream, read by nothing in the mt792x tree;
/// contents undetermined.
pub const fn mt_mib_dr8(band: u32) -> u32 {
    mt_wf_mib(band, 0x0c0) // mt792x_regs.h:135
}
/// Debug register 9 — as [`mt_mib_dr8`].
pub const fn mt_mib_dr9(band: u32) -> u32 {
    mt_wf_mib(band, 0x0c4) // mt792x_regs.h:136
}
/// Debug register 11 — as [`mt_mib_dr8`].
pub const fn mt_mib_dr11(band: u32) -> u32 {
    mt_wf_mib(band, 0x0cc) // mt792x_regs.h:137
}

/// Per-multi-BSSID retry counters, entry `n` (16-byte stride).
pub const fn mt_mib_mb_sdr0(band: u32, n: u32) -> u32 {
    mt_wf_mib(band, 0x100 + (n << 4)) // mt792x_regs.h:139
}
/// RTS retry count, bits `[31:16]` of [`mt_mib_mb_sdr0`].
pub const MT_MIB_RTS_RETRIES_COUNT_MASK: u32 = 0xffff_0000; // GENMASK(31,16) — mt792x_regs.h:140
/// Per-multi-BSSID frame-retry counters, entry `n`.
pub const fn mt_mib_mb_sdr2(band: u32, n: u32) -> u32 {
    mt_wf_mib(band, 0x108 + (n << 4)) // mt792x_regs.h:151
}
/// Frame retry count, bits `[15:0]` of [`mt_mib_mb_sdr2`].
pub const MT_MIB_FRAME_RETRIES_COUNT_MASK: u32 = 0x0000_ffff; // GENMASK(15,0) — mt792x_regs.h:152

/// RTS count (`mt792x_mac.c:90-91`).
pub const fn mt_mib_mb_bsdr0(band: u32) -> u32 {
    mt_wf_mib(band, 0x688) // mt792x_regs.h:142
}
/// RTS count, bits `[15:0]`.
pub const MT_MIB_RTS_COUNT_MASK: u32 = 0x0000_ffff; // GENMASK(15,0) — mt792x_regs.h:143
/// RTS failure count (`mt792x_mac.c:92-93`). With [`mt_mib_mb_bsdr0`] this is a
/// direct hardware read of contention loss — the quantity the EDCCA and
/// channel-choice work had to infer indirectly on the Realtek parts.
pub const fn mt_mib_mb_bsdr1(band: u32) -> u32 {
    mt_wf_mib(band, 0x690) // mt792x_regs.h:144
}
/// RTS failure count, bits `[15:0]`.
pub const MT_MIB_RTS_FAIL_COUNT_MASK: u32 = 0x0000_ffff; // GENMASK(15,0) — mt792x_regs.h:145
/// BlockAck failure count (`mt792x_mac.c:88-89`).
pub const fn mt_mib_mb_bsdr2(band: u32) -> u32 {
    mt_wf_mib(band, 0x518) // mt792x_regs.h:146
}
/// BA failure count, bits `[15:0]`.
pub const MT_MIB_BA_FAIL_COUNT_MASK: u32 = 0x0000_ffff; // GENMASK(15,0) — mt792x_regs.h:147
/// ACK failure count (`mt792x_mac.c:86-87`).
pub const fn mt_mib_mb_bsdr3(band: u32) -> u32 {
    mt_wf_mib(band, 0x520) // mt792x_regs.h:148
}
/// ACK failure count, bits `[15:0]`.
pub const MT_MIB_ACK_FAIL_COUNT_MASK: u32 = 0x0000_ffff; // GENMASK(15,0) — mt792x_regs.h:149

/// TX aggregation-length histogram, low bank, word `n` (`n` < 4; two 16-bit
/// buckets per word). Read and reset by `mt792x_mac_reset_counters`
/// (`mt792x_mac.c:197-199`).
pub const fn mt_tx_agg_cnt(band: u32, n: u32) -> u32 {
    mt_wf_mib(band, 0x7dc + (n << 2)) // mt792x_regs.h:154
}
/// TX aggregation-length histogram, high bank — see [`mt_tx_agg_cnt`].
pub const fn mt_tx_agg_cnt2(band: u32, n: u32) -> u32 {
    mt_wf_mib(band, 0x7ec + (n << 2)) // mt792x_regs.h:155
}
/// Aggregation-range configuration, word `n` (four 8-bit ranges per word).
pub const fn mt_mib_arng(band: u32, n: u32) -> u32 {
    mt_wf_mib(band, 0x0b0 + (n << 2)) // mt792x_regs.h:156
}
/// Extract range `n` from an [`mt_mib_arng`] word (upstream `MT_MIB_ARNCR_RANGE`).
pub const fn mt_mib_arncr_range(val: u32, n: u32) -> u32 {
    (val >> (n << 3)) & 0x0000_00ff // mt792x_regs.h:157
}

// ── TMAC — transmit MAC: the IFS/slot timing knobs ──────────────────────────

/// Base of the per-band TMAC block (`mt792x_regs.h:31`).
pub const fn mt_wf_tmac_base(band: u32) -> u32 {
    if band != 0 { 0x820f_4000 } else { 0x820e_4000 }
}
/// An offset within the TMAC block for `band`.
pub const fn mt_wf_tmac(band: u32, ofs: u32) -> u32 {
    mt_wf_tmac_base(band) + ofs // mt792x_regs.h:32
}

/// TX control 0. Its one named bit stops TX at TBTT.
pub const fn mt_tmac_tcr0(band: u32) -> u32 {
    mt_wf_tmac(band, 0) // mt792x_regs.h:34
}
/// Stop transmission at the target beacon transmit time.
pub const MT_TMAC_TCR0_TBTT_STOP_CTRL: u32 = 0x0200_0000; // BIT(25) — mt792x_regs.h:35

/// CCK detection-timeout register. `mt792x_mac_set_timeing` writes
/// `PLCP=231, CCA=48` plus `3 × coverage_class` (`mt792x_mac.c:58`). ★ That
/// `3 × coverage_class` term is the propagation-delay dial — the same knob
/// `crate::coverage` reaches on other parts.
pub const fn mt_tmac_cdtr(band: u32) -> u32 {
    mt_wf_tmac(band, 0x090) // mt792x_regs.h:37
}
/// OFDM detection-timeout register: `PLCP=60, CCA=28` + the coverage term
/// (`mt792x_mac.c:59`).
pub const fn mt_tmac_odtr(band: u32) -> u32 {
    mt_wf_tmac(band, 0x094) // mt792x_regs.h:38
}
/// PLCP timeout, bits `[15:0]`, of the CDTR/ODTR words.
pub const MT_TIMEOUT_VAL_PLCP: u32 = 0x0000_ffff; // GENMASK(15,0) — mt792x_regs.h:39
/// CCA timeout, bits `[31:16]`, of the CDTR/ODTR words.
pub const MT_TIMEOUT_VAL_CCA: u32 = 0xffff_0000; // GENMASK(31,16) — mt792x_regs.h:40

/// ★ Interframe-space control. `mt792x_mac_set_timeing` (`mt792x_mac.c:60-64`)
/// programs EIFS=360, RIFS=2, SIFS=10 (2.4 GHz) or 16 (5 GHz), and SLOT from
/// `phy->slottime`. This is where a named-airtime-lease MAC gets its slot and
/// SIFS; write it only with TX+RX disabled via [`mt_arb_scr`], which is what
/// upstream brackets it with (`mt792x_mac.c:50-51, 72-73`).
pub const fn mt_tmac_icr0(band: u32) -> u32 {
    mt_wf_tmac(band, 0x0a4) // mt792x_regs.h:42
}
/// EIFS, bits `[8:0]`, microseconds.
pub const MT_IFS_EIFS: u32 = 0x0000_01ff; // GENMASK(8,0) — mt792x_regs.h:43
/// RIFS, bits `[14:10]`.
pub const MT_IFS_RIFS: u32 = 0x0000_7c00; // GENMASK(14,10) — mt792x_regs.h:44
/// SIFS, bits `[22:16]`, microseconds.
pub const MT_IFS_SIFS: u32 = 0x007f_0000; // GENMASK(22,16) — mt792x_regs.h:45
/// Slot time, bits `[30:24]`, microseconds.
pub const MT_IFS_SLOT: u32 = 0x7f00_0000; // GENMASK(30,24) — mt792x_regs.h:46

/// TX control for the A-MSDU deadline limiter. `mt792x_mac_init_band` sets
/// `REFTIME=0x3f` plus both enables (`mt792x_mac.c:289-293`).
pub const fn mt_tmac_ctcr0(band: u32) -> u32 {
    mt_wf_tmac(band, 0x0f4) // mt792x_regs.h:48
}
/// Deadline-limiter reference time, bits `[5:0]`.
pub const MT_TMAC_CTCR0_INS_DDLMT_REFTIME: u32 = 0x0000_003f; // GENMASK(5,0) — mt792x_regs.h:49
/// Enable the deadline limiter.
pub const MT_TMAC_CTCR0_INS_DDLMT_EN: u32 = 0x0002_0000; // BIT(17) — mt792x_regs.h:50
/// Enable the deadline limiter for VHT single-MPDU frames.
pub const MT_TMAC_CTCR0_INS_DDLMT_VHT_SMPDU_EN: u32 = 0x0004_0000; // BIT(18) — mt792x_regs.h:51

/// TX rate-control register 0. Declared upstream, written by nothing in the
/// mt792x tree; contents undetermined.
pub const fn mt_tmac_trcr0(band: u32) -> u32 {
    mt_wf_tmac(band, 0x09c) // mt792x_regs.h:53
}
/// TX frame-control register 0 — as [`mt_tmac_trcr0`].
pub const fn mt_tmac_tfcr0(band: u32) -> u32 {
    mt_wf_tmac(band, 0x1e0) // mt792x_regs.h:54
}

// ── AGG / ARB — aggregation and the TX/RX arbiter ───────────────────────────

/// Base of the per-band AGG block (`mt792x_regs.h:179`).
pub const fn mt_wf_agg_base(band: u32) -> u32 {
    if band != 0 { 0x820f_2000 } else { 0x820e_2000 }
}
/// An offset within the AGG block for `band`.
pub const fn mt_wf_agg(band: u32, ofs: u32) -> u32 {
    mt_wf_agg_base(band) + ofs // mt792x_regs.h:180
}

/// Aggregation window-size control, word `n`.
pub const fn mt_agg_awscr0(band: u32, n: u32) -> u32 {
    mt_wf_agg(band, 0x05c + n * 4) // mt792x_regs.h:182
}
/// Protection control, word `n` — which PHY modes get RTS/CTS or CTS-to-self
/// protection.
pub const fn mt_agg_pcr0(band: u32, n: u32) -> u32 {
    mt_wf_agg(band, 0x06c + n * 4) // mt792x_regs.h:183
}
/// Protect mixed-mode HT.
pub const MT_AGG_PCR0_MM_PROT: u32 = 0x0000_0001; // BIT(0) — mt792x_regs.h:184
/// Protect greenfield HT.
pub const MT_AGG_PCR0_GF_PROT: u32 = 0x0000_0002; // BIT(1) — mt792x_regs.h:185
/// Protect 20 MHz.
pub const MT_AGG_PCR0_BW20_PROT: u32 = 0x0000_0004; // BIT(2) — mt792x_regs.h:186
/// Protect 40 MHz.
pub const MT_AGG_PCR0_BW40_PROT: u32 = 0x0000_0010; // BIT(4) — mt792x_regs.h:187
/// Protect 80 MHz.
pub const MT_AGG_PCR0_BW80_PROT: u32 = 0x0000_0040; // BIT(6) — mt792x_regs.h:188
/// ERP protection selector, bits `[12:8]`.
pub const MT_AGG_PCR0_ERP_PROT: u32 = 0x0000_1f00; // GENMASK(12,8) — mt792x_regs.h:189
/// Protect VHT.
pub const MT_AGG_PCR0_VHT_PROT: u32 = 0x0000_2000; // BIT(13) — mt792x_regs.h:190
/// Disable the packet-traffic-arbitration window (BT coexistence).
pub const MT_AGG_PCR0_PTA_WIN_DIS: u32 = 0x0000_8000; // BIT(15) — mt792x_regs.h:191
/// RTS trigger threshold in MPDU count, bits `[31:23]`, of AGG PCR word 1.
pub const MT_AGG_PCR1_RTS0_NUM_THRES: u32 = 0xff80_0000; // GENMASK(31,23) — mt792x_regs.h:193
/// RTS trigger threshold in bytes, bits `[19:0]`, of AGG PCR word 1.
pub const MT_AGG_PCR1_RTS0_LEN_THRES: u32 = 0x000f_ffff; // GENMASK(19,0) — mt792x_regs.h:194

/// Aggregation control 0 — the CF-End and BAR transmit rates.
/// `mt792x_mac_set_timeing` picks OFDM-24M or 11b-11M here depending on slot
/// time and band (`mt792x_mac.c:66-71`).
pub const fn mt_agg_acr0(band: u32) -> u32 {
    mt_wf_agg(band, 0x084) // mt792x_regs.h:196
}
/// CF-End rate, bits `[13:0]`.
pub const MT_AGG_ACR_CFEND_RATE: u32 = 0x0000_3fff; // GENMASK(13,0) — mt792x_regs.h:197
/// BAR rate, bits `[29:16]`.
pub const MT_AGG_ACR_BAR_RATE: u32 = 0x3fff_0000; // GENMASK(29,16) — mt792x_regs.h:198
/// Default CF-End rate value: OFDM 24 Mbit/s (`mt792x.h:22`).
pub const MT792X_CFEND_RATE_DEFAULT: u32 = 0x49; // mt792x.h:22
/// 11b long-preamble 11 Mbit/s CF-End rate, used on a 2.4 GHz long-slot BSS
/// (`mt792x.h:23`).
pub const MT792X_CFEND_RATE_11B: u32 = 0x03; // mt792x.h:23

/// Multi-rate/retry control — the RTS and BAR retry limits.
pub const fn mt_agg_mrcr(band: u32) -> u32 {
    mt_wf_agg(band, 0x098) // mt792x_regs.h:200
}
/// BAR retry count limit, bits `[15:12]`.
pub const MT_AGG_MRCR_BAR_CNT_LIMIT: u32 = 0x0000_f000; // GENMASK(15,12) — mt792x_regs.h:201
/// Randomise the last RTS/CTS.
pub const MT_AGG_MRCR_LAST_RTS_CTS_RN: u32 = 0x0000_0040; // BIT(6) — mt792x_regs.h:202
/// RTS failure limit, bits `[11:7]`.
pub const MT_AGG_MRCR_RTS_FAIL_LIMIT: u32 = 0x0000_0f80; // GENMASK(11,7) — mt792x_regs.h:203
/// RTS failure limit for TXCMD-sourced frames, bits `[28:24]`.
pub const MT_AGG_MRCR_TXCMD_RTS_FAIL_LIMIT: u32 = 0x1f00_0000; // GENMASK(28,24) — mt792x_regs.h:204

/// Aggregation timing control 1. Declared upstream, written by nothing in the
/// mt792x tree; contents undetermined.
pub const fn mt_agg_atcr1(band: u32) -> u32 {
    mt_wf_agg(band, 0x0f0) // mt792x_regs.h:206
}
/// Aggregation timing control 3 — as [`mt_agg_atcr1`].
pub const fn mt_agg_atcr3(band: u32) -> u32 {
    mt_wf_agg(band, 0x0f4) // mt792x_regs.h:207
}

/// Base of the per-band ARB block (`mt792x_regs.h:210`).
pub const fn mt_wf_arb_base(band: u32) -> u32 {
    if band != 0 { 0x820f_3000 } else { 0x820e_3000 }
}
/// An offset within the ARB block for `band`.
pub const fn mt_wf_arb(band: u32, ofs: u32) -> u32 {
    mt_wf_arb_base(band) + ofs // mt792x_regs.h:211
}

/// ★ Arbiter control — the TX and RX kill switches. Upstream brackets every
/// IFS/slot change with `set(TX_DISABLE|RX_DISABLE)`, `udelay(1)`, the writes,
/// then `clear(...)` (`mt792x_mac.c:50-51, 72-73`). It is also the closest thing
/// this part has to the Realtek `TXPAUSE` airtime actuator, though nothing here
/// has measured it as one.
pub const fn mt_arb_scr(band: u32) -> u32 {
    mt_wf_arb(band, 0x080) // mt792x_regs.h:213
}
/// Disable transmission.
pub const MT_ARB_SCR_TX_DISABLE: u32 = 0x0000_0100; // BIT(8) — mt792x_regs.h:214
/// Disable reception.
pub const MT_ARB_SCR_RX_DISABLE: u32 = 0x0000_0200; // BIT(9) — mt792x_regs.h:215

/// Arbiter DRNG register `n`. Declared upstream, written by nothing in the
/// mt792x tree; contents undetermined.
pub const fn mt_arb_drngr0(band: u32, n: u32) -> u32 {
    mt_wf_arb(band, 0x194 + n * 4) // mt792x_regs.h:217
}

// ── WTBLOFF / ETBF — RSSI reporting mode and beamforming counters ───────────

/// Base of the per-band WTBLOFF block (`mt792x_regs.h:64`).
pub const fn mt_wtbloff_top_base(band: u32) -> u32 {
    if band != 0 { 0x820f_9000 } else { 0x820e_9000 }
}

/// ★ RSSI/RCPI reporting mode. `mt792x_mac_init_band` programs `RCPI_MODE=0`
/// and `RCPI_PARAM=3` (`mt792x_mac.c:306-310`) with the comment *"filter out
/// non-resp frames and get instantaneous signal reporting"*. Instantaneous
/// rather than averaged RSSI is what a per-frame RSSI keyed on an ephemeral
/// source nonce needs — an averaged report would blend unrelated senders.
pub const fn mt_wtbloff_top_rscr(band: u32) -> u32 {
    mt_wtbloff_top_base(band) + 0x008 // mt792x_regs.h:67
}
/// RCPI mode, bits `[31:30]`. Programmed 0.
pub const MT_WTBLOFF_TOP_RSCR_RCPI_MODE: u32 = 0xc000_0000; // GENMASK(31,30) — mt792x_regs.h:68
/// RCPI parameter, bits `[25:24]`. Programmed 3.
pub const MT_WTBLOFF_TOP_RSCR_RCPI_PARAM: u32 = 0x0300_0000; // GENMASK(25,24) — mt792x_regs.h:69

/// Base of the per-band ETBF (beamforming) block (`mt792x_regs.h:83`).
pub const fn mt_wf_etbf_base(band: u32) -> u32 {
    if band != 0 { 0x820f_a000 } else { 0x820e_a000 }
}
/// Beamformed-TX counters (`mt792x_mac.c:103-105`).
pub const fn mt_etbf_tx_app_cnt(band: u32) -> u32 {
    mt_wf_etbf_base(band) + 0x150 // mt792x_regs.h:86
}
/// Implicit-BF TX count, bits `[31:16]`.
pub const MT_ETBF_TX_IBF_CNT: u32 = 0xffff_0000; // GENMASK(31,16) — mt792x_regs.h:87
/// Explicit-BF TX count, bits `[15:0]`.
pub const MT_ETBF_TX_EBF_CNT: u32 = 0x0000_ffff; // GENMASK(15,0) — mt792x_regs.h:88

/// ★ Beamforming-feedback RX counters, split **by PHY generation**
/// (`mt792x_mac.c:107-111`). The `HE` field is one of the few places the
/// hardware itself distinguishes 11ax traffic, which makes it a cheap check that
/// an HE actuation actually produced HE frames on air.
pub const fn mt_etbf_rx_fb_cnt(band: u32) -> u32 {
    mt_wf_etbf_base(band) + 0x158 // mt792x_regs.h:90
}
/// All feedback frames, bits `[31:24]`.
pub const MT_ETBF_RX_FB_ALL: u32 = 0xff00_0000; // GENMASK(31,24) — mt792x_regs.h:91
/// HE (802.11ax) feedback frames, bits `[23:16]`.
pub const MT_ETBF_RX_FB_HE: u32 = 0x00ff_0000; // GENMASK(23,16) — mt792x_regs.h:92
/// VHT feedback frames, bits `[15:8]`.
pub const MT_ETBF_RX_FB_VHT: u32 = 0x0000_ff00; // GENMASK(15,8) — mt792x_regs.h:93
/// HT feedback frames, bits `[7:0]`.
pub const MT_ETBF_RX_FB_HT: u32 = 0x0000_00ff; // GENMASK(7,0) — mt792x_regs.h:94

// ── WTBL — the station table ────────────────────────────────────────────────

/// Base of the WTBL control block (not per-band).
pub const MT_WTBLON_TOP_BASE: u32 = 0x820d_4000; // mt792x_regs.h:159
/// WTBL data-unit control. Its group field selects which DW group a WTBL
/// read/write touches.
pub const MT_WTBLON_TOP_WDUCR: u32 = 0x820d_4200; // mt7921/regs.h:74
/// DW-group selector, bits `[2:0]`.
pub const MT_WTBLON_TOP_WDUCR_GROUP: u32 = 0x0000_0007; // GENMASK(2,0) — mt7921/regs.h:75

/// WTBL update command. `mt7921_mac_init` walks all
/// [`MT792X_WTBL_SIZE`] entries clearing their admission counters
/// (`mt7921/init.c:74-76` via `mt7921_mac_wtbl_update`, `mt7921/mac.c:19-26`).
pub const MT_WTBL_UPDATE: u32 = 0x820d_4230; // mt7921/regs.h:77
/// Target WLAN index, bits `[9:0]`.
pub const MT_WTBL_UPDATE_WLAN_IDX: u32 = 0x0000_03ff; // GENMASK(9,0) — mt7921/regs.h:78
/// Clear the entry's admission counter.
pub const MT_WTBL_UPDATE_ADM_COUNT_CLEAR: u32 = 0x0000_1000; // BIT(12) — mt7921/regs.h:79
/// Update in progress — poll to 0 (`mt7921/mac.c:24-25`).
pub const MT_WTBL_UPDATE_BUSY: u32 = 0x8000_0000; // BIT(31) — mt792x_regs.h:162

/// Number of WTBL entries on this part — **20**, not 256
/// (`mt792x.h:18`). That is small enough that the full clear costs 20 EP0 round
/// trips (~5.4 ms here), unlike the mt76x0's ~1300-write table wipe which this
/// crate skips.
pub const MT792X_WTBL_SIZE: u32 = 20; // mt792x.h:18
/// The reserved (no-station) WTBL index, `MT792x_WTBL_SIZE - 1` (`mt792x.h:19`).
pub const MT792X_WTBL_RESERVED: u32 = 19; // mt792x.h:19

/// WTBL indirect-access control.
pub const MT_WTBL_ITCR: u32 = 0x820d_43b0; // mt792x_regs.h:164
/// Indirect access is a write (rather than a read).
pub const MT_WTBL_ITCR_WR: u32 = 0x0001_0000; // BIT(16) — mt792x_regs.h:165
/// Execute the indirect access.
pub const MT_WTBL_ITCR_EXEC: u32 = 0x8000_0000; // BIT(31) — mt792x_regs.h:166
/// Indirect-access data word 0.
pub const MT_WTBL_ITDR0: u32 = 0x820d_43b8; // mt792x_regs.h:167
/// Indirect-access data word 1.
pub const MT_WTBL_ITDR1: u32 = 0x820d_43bc; // mt792x_regs.h:168
/// Spatial-extension-index select bit within a WTBL entry.
pub const MT_WTBL_SPE_IDX_SEL: u32 = 0x0000_0040; // BIT(6) — mt792x_regs.h:169

/// Base of the directly-mapped WTBL memory.
pub const MT_WTBL_BASE: u32 = 0x820d_8000; // mt792x_regs.h:171
/// LMAC entry id within a [`mt_wtbl_lmac_offs`] address, bits `[14:8]`.
pub const MT_WTBL_LMAC_ID: u32 = 0x0000_7f00; // GENMASK(14,8) — mt792x_regs.h:172
/// DW index within a [`mt_wtbl_lmac_offs`] address, bits `[7:2]`.
pub const MT_WTBL_LMAC_DW: u32 = 0x0000_00fc; // GENMASK(7,2) — mt792x_regs.h:173
/// Address of DW `dw` of WTBL entry `id` (upstream `MT_WTBL_LMAC_OFFS`).
pub const fn mt_wtbl_lmac_offs(id: u32, dw: u32) -> u32 {
    MT_WTBL_BASE | field_prep(MT_WTBL_LMAC_ID, id) | field_prep(MT_WTBL_LMAC_DW, dw) // mt792x_regs.h:174-176
}

// ── MDP — the RX header/A-MSDU de-aggregation path ──────────────────────────

/// Base of the MDP block (not per-band).
pub const MT_MDP_BASE: u32 = 0x820c_d000; // mt7921/regs.h:9

/// ★ MDP data control 0. `mt7921_mac_init` sets **both** named bits
/// (`mt7921/init.c:70,72`): hardware A-MSDU de-aggregation and hardware RX
/// header translation. ⚠ The second one rewrites 802.11 headers into
/// Ethernet-style headers before the host sees them, which would destroy the
/// raw frame this crate's `FrameIo` contract hands upward.
/// **Do not set [`MT_MDP_DCR0_RX_HDR_TRANS_EN`] in this port** — the faithful
/// port of upstream's line is the wrong behaviour here, and this is one of the
/// few places that is true.
pub const MT_MDP_DCR0: u32 = 0x820c_d000; // mt7921/regs.h:12
/// Enable hardware de-aggregation of A-MSDUs.
pub const MT_MDP_DCR0_DAMSDU_EN: u32 = 0x0000_8000; // BIT(15) — mt7921/regs.h:13
/// ⚠ Enable RX header translation to Ethernet form — see [`MT_MDP_DCR0`].
pub const MT_MDP_DCR0_RX_HDR_TRANS_EN: u32 = 0x0008_0000; // BIT(19) — mt7921/regs.h:14

/// MDP data control 1 — the host-visible RX length cap, programmed 1536
/// (`mt7921/init.c:68`).
pub const MT_MDP_DCR1: u32 = 0x820c_d004; // mt7921/regs.h:16
/// Maximum RX length, bits `[15:3]`.
pub const MT_MDP_DCR1_MAX_RX_LEN: u32 = 0x0000_fff8; // GENMASK(15,3) — mt7921/regs.h:17

/// ★ Per-band RX-classification config 0 — **where management and control
/// frames go**. Each 2-bit field selects [`MT_MDP_TO_HIF`] (the host) or
/// [`MT_MDP_TO_WM`] (the MCU). A monitor-style port wants HIF for all three;
/// upstream never writes this register on mt7921, so the reset default governs
/// and is **undetermined**.
pub const fn mt_mdp_bnrcfr0(band: u32) -> u32 {
    MT_MDP_BASE + 0x070 + (band << 8) // mt7921/regs.h:19
}
/// Management-frame destination, bits `[5:4]`.
pub const MT_MDP_RCFR0_MCU_RX_MGMT: u32 = 0x0000_0030; // GENMASK(5,4) — mt7921/regs.h:20
/// Non-BAR control-frame destination, bits `[7:6]`.
pub const MT_MDP_RCFR0_MCU_RX_CTL_NON_BAR: u32 = 0x0000_00c0; // GENMASK(7,6) — mt7921/regs.h:21
/// BAR-frame destination, bits `[9:8]`.
pub const MT_MDP_RCFR0_MCU_RX_CTL_BAR: u32 = 0x0000_0300; // GENMASK(9,8) — mt7921/regs.h:22

/// Per-band RX-classification config 1 — bypass and drop routing.
pub const fn mt_mdp_bnrcfr1(band: u32) -> u32 {
    MT_MDP_BASE + 0x074 + (band << 8) // mt7921/regs.h:24
}
/// Destination for frames that bypass classification, bits `[23:22]`.
pub const MT_MDP_RCFR1_MCU_RX_BYPASS: u32 = 0x00c0_0000; // GENMASK(23,22) — mt7921/regs.h:25
/// Destination for dropped unicast, bits `[28:27]`.
pub const MT_MDP_RCFR1_RX_DROPPED_UCAST: u32 = 0x1800_0000; // GENMASK(28,27) — mt7921/regs.h:26
/// Destination for dropped multicast, bits `[30:29]`.
pub const MT_MDP_RCFR1_RX_DROPPED_MCAST: u32 = 0x6000_0000; // GENMASK(30,29) — mt7921/regs.h:27
/// Route to the host interface. The value this port wants everywhere.
pub const MT_MDP_TO_HIF: u32 = 0; // mt7921/regs.h:28
/// Route to the WiFi MCU.
pub const MT_MDP_TO_WM: u32 = 1; // mt7921/regs.h:29

// ── PLE / PSE — the packet buffer pools ─────────────────────────────────────

/// Base of the packet-link-engine block.
pub const MT_PLE_BASE: u32 = 0x820c_0000; // mt792x_regs.h:17
/// Flow-control queue-0 control. Read by the debugfs dump only; a stuck queue
/// shows up here, which is why the four are transcribed.
pub const MT_PLE_FL_Q0_CTRL: u32 = 0x820c_03e0; // mt792x_regs.h:20
/// Flow-control queue-1 control — see [`MT_PLE_FL_Q0_CTRL`].
pub const MT_PLE_FL_Q1_CTRL: u32 = 0x820c_03e4; // mt792x_regs.h:21
/// Flow-control queue-2 control — see [`MT_PLE_FL_Q0_CTRL`].
pub const MT_PLE_FL_Q2_CTRL: u32 = 0x820c_03e8; // mt792x_regs.h:22
/// Flow-control queue-3 control — see [`MT_PLE_FL_Q0_CTRL`].
pub const MT_PLE_FL_Q3_CTRL: u32 = 0x820c_03ec; // mt792x_regs.h:23
/// Per-AC queue-empty status, AC `n` (0x40 stride).
pub const fn mt_ple_ac_qempty(n: u32) -> u32 {
    MT_PLE_BASE + 0x500 + 0x40 * n // mt792x_regs.h:25
}
/// A-MSDU packing histogram bucket `n` (`mt792x_mac.c:118-122`).
pub const fn mt_ple_amsdu_pack_msdu_cnt(n: u32) -> u32 {
    MT_PLE_BASE + 0x10e0 + (n << 2) // mt792x_regs.h:26
}
/// Base of the packet-store-engine block. No offsets are defined upstream for
/// mt792x; recorded so the block can be located if one is ever needed.
pub const MT_PSE_BASE: u32 = 0x820c_8000; // mt792x_regs.h:28

// ═════════════════════════════════════════════════════════════════════════════

/// ⚠ **Quarantine: addresses that are already PCIe-remapped and are therefore
/// WRONG over USB.** See Flag 2 in the module header.
///
/// Every constant here is transcribed exactly as `mt792x_regs.h` /
/// `mt7921/regs.h` writes it — which is to say, as the BAR offset
/// `__mt7921_reg_addr` (`mt7921/pci.c:70-146`) produces, not as a physical
/// address. **Nothing in this module may be passed to `Connac2Usb::rr`/`wr`.**
/// They live here so that (a) a reader recognises them when they appear in
/// upstream code, and (b) the physical equivalents are recorded once, in one
/// place, instead of being re-derived under pressure.
///
/// The `*_PHYS` companions are **our derivation**, obtained by inverting the
/// matching `fixed_map` row, and are **untested on silicon**. The one exception
/// is the WFDMA0 block, for which upstream supplies its own physical alias —
/// use `super::MT_UWFDMA0_GLO_CFG` and friends, not a `_PHYS` constant.
pub mod pcie_only {
    /// PCIe-mapped base of WFDMA0. Physical equivalent: `0x7c02_4000`, and
    /// upstream names it [`super::MT_UWFDMA0_BASE`] — use that.
    pub const MT_WFDMA0_BASE: u32 = 0x000d_4000; // mt792x_regs.h:262
    /// WFDMA0 reset control (PCIe form).
    pub const MT_WFDMA0_RST: u32 = 0x000d_4100; // mt792x_regs.h:265
    /// WFDMA logic reset.
    pub const MT_WFDMA0_RST_LOGIC_RST: u32 = 0x0000_0010; // BIT(4) — mt792x_regs.h:266
    /// DMA-scheduler-wide reset.
    pub const MT_WFDMA0_RST_DMASHDL_ALL_RST: u32 = 0x0000_0020; // BIT(5) — mt792x_regs.h:267
    /// FIFO busy-status enables (PCIe form).
    pub const MT_WFDMA0_BUSY_ENA: u32 = 0x000d_413c; // mt792x_regs.h:269
    /// TX FIFO 0 busy enable.
    pub const MT_WFDMA0_BUSY_ENA_TX_FIFO0: u32 = 0x0000_0001; // BIT(0) — mt792x_regs.h:270
    /// TX FIFO 1 busy enable.
    pub const MT_WFDMA0_BUSY_ENA_TX_FIFO1: u32 = 0x0000_0002; // BIT(1) — mt792x_regs.h:271
    /// RX FIFO busy enable.
    pub const MT_WFDMA0_BUSY_ENA_RX_FIFO: u32 = 0x0000_0004; // BIT(2) — mt792x_regs.h:272

    /// ⚠ The host→MCU doorbell (PCIe form). There is **no USB analogue**: over
    /// USB, MCU commands go out as framed messages on the bulk-OUT command pipe
    /// (`mt7921/usb.c:47-57`), not through a register. Recorded so nobody ports
    /// the PCIe mailbox by mistake.
    pub const MT_MCU_CMD: u32 = 0x000d_41f0; // mt792x_regs.h:274
    /// Wake the RX path (PCIe).
    pub const MT_MCU_CMD_WAKE_RX_PCIE: u32 = 0x0000_0001; // BIT(0) — mt792x_regs.h:275
    /// Stop DMA and reload firmware.
    pub const MT_MCU_CMD_STOP_DMA_FW_RELOAD: u32 = 0x0000_0002; // BIT(1) — mt792x_regs.h:276
    /// Stop DMA.
    pub const MT_MCU_CMD_STOP_DMA: u32 = 0x0000_0004; // BIT(2) — mt792x_regs.h:277
    /// Reset done.
    pub const MT_MCU_CMD_RESET_DONE: u32 = 0x0000_0008; // BIT(3) — mt792x_regs.h:278
    /// Recovery done.
    pub const MT_MCU_CMD_RECOVERY_DONE: u32 = 0x0000_0010; // BIT(4) — mt792x_regs.h:279
    /// Normal state.
    pub const MT_MCU_CMD_NORMAL_STATE: u32 = 0x0000_0020; // BIT(5) — mt792x_regs.h:280
    /// All error/recovery bits, `[5:1]`.
    pub const MT_MCU_CMD_ERROR_MASK: u32 = 0x0000_003e; // GENMASK(5,1) — mt792x_regs.h:281
    /// MCU→host software-interrupt enable (PCIe form).
    pub const MT_MCU2HOST_SW_INT_ENA: u32 = 0x000d_41f4; // mt792x_regs.h:283
    /// Host interrupt status (PCIe form).
    pub const MT_WFDMA0_HOST_INT_STA: u32 = 0x000d_4200; // mt792x_regs.h:285
    /// Host interrupt enable (PCIe form).
    pub const MT_WFDMA0_HOST_INT_ENA: u32 = 0x000d_4204; // mt7921/regs.h:31
    /// Global DMA config (PCIe form). Physical: `0x7c02_4208` =
    /// [`super::MT_UWFDMA0_GLO_CFG`]. The **bitfields are shared** — the
    /// `MT_WFDMA0_GLO_CFG_*` constants in the parent module apply to both.
    pub const MT_WFDMA0_GLO_CFG: u32 = 0x000d_4208; // mt792x_regs.h:292

    /// PCIe-mapped WFDMA external CSR base. Physical: `0x7c02_7000` — of which
    /// upstream states one member directly, [`super::MT_WFDMA_HOST_CONFIG`]
    /// (`0x7c02_7030`).
    pub const MT_WFDMA_EXT_CSR_BASE: u32 = 0x000d_7000; // mt792x_regs.h:386
    /// HIF misc/busy register (PCIe form).
    pub const MT_WFDMA_EXT_CSR_HIF_MISC: u32 = 0x000d_7044; // mt792x_regs.h:388
    /// Derived physical address of [`MT_WFDMA_EXT_CSR_HIF_MISC`]. **Untested.**
    pub const MT_WFDMA_EXT_CSR_HIF_MISC_PHYS: u32 = 0x7c02_7044;
    /// HIF busy flag.
    pub const MT_WFDMA_EXT_CSR_HIF_MISC_BUSY: u32 = 0x0000_0001; // BIT(0) — mt792x_regs.h:389

    /// PCIe-mapped MCU WFDMA1 base. Physical: `0x5500_0000` (`fixed_map`
    /// `{0x55000000, 0x03000, 0x01000}`).
    pub const MT_MCU_WFDMA1_BASE: u32 = 0x0000_3000; // mt792x_regs.h:8
    /// MCU interrupt-event register (PCIe form).
    pub const MT_MCU_INT_EVENT: u32 = 0x0000_3108; // mt792x_regs.h:11
    /// Derived physical address of [`MT_MCU_INT_EVENT`]. **Untested.**
    pub const MT_MCU_INT_EVENT_PHYS: u32 = 0x5500_0108;
    /// DMA stopped.
    pub const MT_MCU_INT_EVENT_DMA_STOPPED: u32 = 0x0000_0001; // BIT(0) — mt792x_regs.h:12
    /// DMA initialised.
    pub const MT_MCU_INT_EVENT_DMA_INIT: u32 = 0x0000_0002; // BIT(1) — mt792x_regs.h:13
    /// System-error recovery triggered.
    pub const MT_MCU_INT_EVENT_SER_TRIGGER: u32 = 0x0000_0004; // BIT(2) — mt792x_regs.h:14
    /// Reset done.
    pub const MT_MCU_INT_EVENT_RESET_DONE: u32 = 0x0000_0008; // BIT(3) — mt792x_regs.h:15

    /// PCIe-mapped INFRA-CFG base. Physical: `0x7c00_e000`.
    pub const MT_INFRA_CFG_BASE: u32 = 0x000f_e000; // mt7921/regs.h:63
    /// ★ The L1 remap window register itself — the PCIe fallback for addresses
    /// no `fixed_map` row covers (`mt7921/mt7921.h:227-236`). **Meaningless over
    /// USB**, where addresses need no window at all (Flag 1).
    pub const MT_HIF_REMAP_L1: u32 = 0x000f_e24c; // mt7921/regs.h:66
    /// Derived physical address of [`MT_HIF_REMAP_L1`]. **Untested**, and there
    /// is no reason to use it.
    pub const MT_HIF_REMAP_L1_PHYS: u32 = 0x7c00_e24c;
    /// Window-select field written into the remap register, bits `[15:0]`.
    pub const MT_HIF_REMAP_L1_MASK: u32 = 0x0000_ffff; // GENMASK(15,0) — mt7921/regs.h:67
    /// Offset half of a remapped address, bits `[15:0]`.
    pub const MT_HIF_REMAP_L1_OFFSET: u32 = 0x0000_ffff; // GENMASK(15,0) — mt7921/regs.h:68
    /// Base half of a remapped address, bits `[31:16]`.
    pub const MT_HIF_REMAP_L1_BASE: u32 = 0xffff_0000; // GENMASK(31,16) — mt7921/regs.h:69
    /// BAR offset the L1 window appears at.
    pub const MT_HIF_REMAP_BASE_L1: u32 = 0x0004_0000; // mt7921/regs.h:70

    /// PCIe-mapped PCIe-MAC base. Physical: `0x7403_0000`. Nothing under it has
    /// any meaning on a USB part; transcribed only to be recognisable.
    pub const MT_PCIE_MAC_BASE: u32 = 0x0001_0000; // mt7921/regs.h:81
    /// PCIe MAC interrupt enable (PCIe form).
    pub const MT_PCIE_MAC_INT_ENABLE: u32 = 0x0001_0188; // mt7921/regs.h:83
    /// PCIe MAC power management (PCIe form).
    pub const MT_PCIE_MAC_PM: u32 = 0x0001_0194; // mt7921/regs.h:84
    /// Disable PCIe L0s.
    pub const MT_PCIE_MAC_PM_L0S_DIS: u32 = 0x0000_0100; // BIT(8) — mt792x_regs.h:435

    /// ⚠ The WFSYS reset register the **PCIe/SDIO** path uses
    /// (`mt792x_dma.c:603-616`, connac2 branch). This *is* a physical address —
    /// it is quarantined not because of remapping but because the USB path
    /// resets a **different register by a different method**:
    /// [`super::MT_CBTOP_RGU_WF_SUBSYS_RST`] over the UHW vendor path. Porting
    /// this sequence to USB resets nothing. See "Never reset" in the module
    /// header before using either.
    pub const MT_WFSYS_SW_RST_B: u32 = 0x1800_0140; // mt7921/regs.h:72
    /// The reset bit inside [`MT_WFSYS_SW_RST_B`] (active low: cleared to reset).
    pub const WFSYS_SW_RST_B: u32 = 0x0000_0001; // BIT(0) — mt792x_regs.h:515
    /// WFSYS init-done, polled after releasing the reset.
    pub const WFSYS_SW_INIT_DONE: u32 = 0x0000_0010; // BIT(4) — mt792x_regs.h:516
}

/// The USB **access paths** a register can take, named so that each register's
/// `ACCESS:` tag above means something concrete.
///
/// ⚠ *Contract note:* `usb.rs` owns the transport and will define whatever it
/// needs; these constants are here because **which request reaches which
/// address** is a property of the register map (Flag 3), not of the transport.
/// Nothing here is required by the `Connac2Usb` contract.
pub mod access {
    /// Ordinary register read — `bRequest` for the extended-read path used by
    /// `mt792xu_rr` for essentially every address in this file.
    /// **MEASURED working** on mds-o5p-3's MT7921AU.
    pub const MT_VEND_READ_EXT: u8 = 0x63; // mt76.h:641
    /// Ordinary register write — the companion of [`MT_VEND_READ_EXT`], and the
    /// request `mt792xu_copy` uses for bulk register-space writes too
    /// (`mt792x_usb.c:189-212`).
    pub const MT_VEND_WRITE_EXT: u8 = 0x66; // mt76.h:642
    /// UHW-path **read** (`mt792xu_uhw_rr`, `mt792x_usb.c:242-252`). Reaches the
    /// four registers listed in Flag 3 and nothing else in this port.
    pub const MT_VEND_DEV_MODE: u8 = 0x01; // mt76.h:632
    /// UHW-path **write** (`mt792xu_uhw_wr`, `mt792x_usb.c:254-260`).
    pub const MT_VEND_WRITE: u8 = 0x02; // mt76.h:633
    /// Power-on request. Sent with `wValue = 0, wIndex = 1` and no data, then
    /// [`super::MT_CONN_ON_MISC`] is polled for
    /// [`super::MT_TOP_MISC2_FW_PWR_ON`] (`mt792x_usb.c:214-231`). This is the
    /// first thing a cold bring-up sends.
    pub const MT_VEND_POWER_ON: u8 = 0x04; // mt76.h:634

    /// `bmRequestType` recipient nibble upstream uses for the ordinary path:
    /// `USB_TYPE_VENDOR | 0x1f` = `0x5f`, i.e. `0xdf` with `USB_DIR_IN`.
    pub const MT_USB_TYPE_VENDOR: u8 = 0x5f; // mt792x.h:555
    /// `bmRequestType` for the UHW path: `USB_TYPE_VENDOR | 0x1e` = `0x5e`,
    /// i.e. `0xde` with `USB_DIR_IN`.
    pub const MT_USB_TYPE_UHW_VENDOR: u8 = 0x5e; // mt792x.h:556

    /// ★ **MEASURED**: `0xc0` (IN) / `0x40` (OUT) — the plain
    /// `vendor | device` recipient — works on this part for
    /// [`MT_VEND_READ_EXT`]/[`MT_VEND_WRITE_EXT`], where upstream sends
    /// `0xdf`/`0x5f`. Both apparently reach the same handler. The measured pair
    /// is what this port uses; recorded because a future failure on some other
    /// request would make the difference suddenly interesting.
    pub const MEASURED_BMREQUESTTYPE_IN: u8 = 0xc0;
    /// Companion of [`MEASURED_BMREQUESTTYPE_IN`] for OUT transfers.
    pub const MEASURED_BMREQUESTTYPE_OUT: u8 = 0x40;
}

/// Everything **MEASURED** on the target silicon — mds-o5p-3's MT7921AU
/// (`0e8d:7961`), 2026-08-27. Facts, not transcriptions. Any change to this file
/// must keep agreeing with them, which is what this module's `tests` enforce.
pub mod measured {
    /// [`super::MT_HW_CHIPID`] as read over our own libusb code after claiming
    /// interface 3. Identifies the die as MT7961.
    pub const MEASURED_HW_CHIPID: u32 = 0x0000_7961;
    /// [`super::MT_HW_REV`] as read the same way.
    pub const MEASURED_HW_REV: u32 = 0x0000_8a10;
    /// [`super::MT_CONN_ON_MISC`] as read on a cold part: **zero**, so
    /// [`super::MT_TOP_MISC2_FW_PWR_ON`] is clear and no firmware is running.
    pub const MEASURED_CONN_ON_MISC: u32 = 0x0000_0000;
    /// ⚠ The value read at **`0x7000_00f0`**, an unnamed CB-TOP word. The bench
    /// notes label this "MT_TOP_MISC"; it is **not** — see Flag 4.
    /// [`super::MT_TOP_MISC`] is `0x1806_00f0` and has never been read here.
    pub const MEASURED_CBTOP_00F0: u32 = 0x0000_0000;
    /// The address [`MEASURED_CBTOP_00F0`] was read from, recorded so the two
    /// travel together and the mislabel cannot recur.
    pub const MEASURED_CBTOP_00F0_ADDR: u32 = 0x7000_00f0;

    /// One EP0 vendor-request round trip on this **USB 2.0** (high-speed) bus.
    /// Nearly double the mt76x0's 151 µs and the 8733b's ~92 µs. A register
    /// read is affordable in a channel switch or a 100 ms sensing window and is
    /// **never** affordable on a per-frame path — which is why the RX timestamp
    /// must come from the RXD and not from [`super::mt_lpon_uttr0`].
    pub const EP0_ROUND_TRIP_US: u32 = 268;

    /// USB vendor id.
    pub const USB_VID: u16 = 0x0e8d;
    /// USB product id.
    pub const USB_PID: u16 = 0x7961;
    /// ★ The WLAN function is **interface 3**, class `ff/ff/ff`. Interfaces 0-2
    /// are class `e0/01/01` Bluetooth and belong to `btusb` — **never touch
    /// them**. Upstream's device table matches on exactly this interface class
    /// triple (`mt7921/usb.c:16`).
    pub const WLAN_INTERFACE: u8 = 3;
    /// Bulk IN, packet RX (`MT_EP_IN_PKT_RX`, `mt76.h:646-650`).
    pub const EP_IN_PKT_RX: u8 = 0x84;
    /// Bulk IN, MCU command response (`MT_EP_IN_CMD_RESP`).
    pub const EP_IN_CMD_RESP: u8 = 0x85;
    /// Bulk OUT, in-band MCU command (`MT_EP_OUT_INBAND_CMD`, first of the
    /// `enum mt76u_out_ep` order at `mt76.h:652-660`). MCU commands other than
    /// `FW_SCATTER` go here (`mt7921/usb.c:47-50`).
    pub const EP_OUT_INBAND_CMD: u8 = 0x04;
    /// Bulk OUT, AC_BE — ordinary data **and** firmware-scatter payloads
    /// (`mt7921/usb.c:50`).
    pub const EP_OUT_AC_BE: u8 = 0x05;
    /// Bulk OUT, AC_BK.
    pub const EP_OUT_AC_BK: u8 = 0x06;
    /// Bulk OUT, AC_VI.
    pub const EP_OUT_AC_VI: u8 = 0x07;
    /// Bulk OUT, AC_VO.
    pub const EP_OUT_AC_VO: u8 = 0x08;
    /// Bulk OUT, HCCA.
    pub const EP_OUT_HCCA: u8 = 0x09;
    /// Interrupt IN. Present on interface 3; unused by upstream and by this
    /// port.
    pub const EP_IN_INTERRUPT: u8 = 0x86;
    /// Endpoint max packet size at high speed, for every endpoint above.
    pub const EP_MAX_PACKET_SIZE: u16 = 512;

    /// Size of the vendored ROM patch, `fw/mt7961/WIFI_MT7961_patch_mcu_1_2_hdr.bin`.
    pub const PATCH_LEN: usize = 92_192;
    /// Size of the vendored RAM firmware, `fw/mt7961/WIFI_RAM_CODE_MT7961_1.bin`.
    pub const RAM_LEN: usize = 791_588;
    /// ★ The patch header's `hw_sw_ver` (big-endian, offset 20 of
    /// `struct mt76_connac2_patch_hdr`, `mt76_connac_mcu.h:145-148`) in the
    /// vendored blob: bytes `8a 10 8a 10`. Its **top half is `0x8a10`, equal to
    /// [`MEASURED_HW_REV`]** — the evidence that this firmware belongs to this
    /// die. ⚠ Upstream only *prints* this field (`mt76_connac_mcu.c:3298`); it
    /// validates nothing. The check is ours, and it is worth keeping.
    pub const PATCH_HDR_HW_SW_VER: u32 = 0x8a10_8a10;
    /// Byte offset of `hw_sw_ver` within the patch header: `build_date[16]` +
    /// `platform[4]` (`mt76_connac_mcu.h:146-148`).
    pub const PATCH_HDR_HW_SW_VER_OFFSET: usize = 20;
    /// The patch header's `platform` field in the vendored blob.
    pub const PATCH_HDR_PLATFORM: &[u8; 4] = b"ALPS";
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The address arithmetic upstream expresses as macros, re-derived here so a
    /// typo in a base or a stride cannot pass silently. Values are the ones
    /// computed independently from `mt792x_regs.h` before this file was written.
    #[test]
    fn band_block_addresses_match_upstream_macros() {
        // RMAC: the RX filter and the OBSS airtime sensor.
        assert_eq!(mt_wf_rfcr(0), 0x820e_5000);
        assert_eq!(mt_wf_rfcr1(0), 0x820e_5004);
        assert_eq!(mt_wf_rmac_mib_time0(0), 0x820e_53c4);
        assert_eq!(mt_wf_rmac_mib_airtime0(0), 0x820e_5380);
        assert_eq!(mt_wf_rmac_mib_airtime14(0), 0x820e_53b8);
        // ★ LPON: the TSF path the headline RX timestamp is stitched against.
        assert_eq!(mt_lpon_uttr0(0), 0x820e_b080);
        assert_eq!(mt_lpon_uttr1(0), 0x820e_b084);
        assert_eq!(mt_lpon_tcr(0, 0), 0x820e_b0a8);
        assert_eq!(mt_lpon_tcr(0, 3), 0x820e_b0b4); // HW_BSSID_MAX, n*4 stride
        // MIB: the airtime and contention counters.
        assert_eq!(mt_mib_scr1(0), 0x820e_d004);
        assert_eq!(mt_mib_sdr9(0), 0x820e_d02c);
        assert_eq!(mt_mib_sdr36(0), 0x820e_d054);
        assert_eq!(mt_mib_sdr37(0), 0x820e_d058);
        assert_eq!(mt_mib_sdr3(0), 0x820e_d698);
        assert_eq!(mt_mib_mb_bsdr0(0), 0x820e_d688);
        assert_eq!(mt_mib_mb_bsdr1(0), 0x820e_d690);
        // Non-uniform strides, the usual source of transcription errors.
        assert_eq!(mt_mib_mb_sdr0(0, 1), 0x820e_d110); // 16-byte stride
        assert_eq!(mt_mib_mb_sdr2(0, 1), 0x820e_d118);
        assert_eq!(mt_tx_agg_cnt(0, 3), 0x820e_d7e8);
        assert_eq!(mt_tx_agg_cnt2(0, 3), 0x820e_d7f8);
        // TMAC / AGG / ARB / WTBLOFF / ETBF.
        assert_eq!(mt_tmac_cdtr(0), 0x820e_4090);
        assert_eq!(mt_tmac_icr0(0), 0x820e_40a4);
        assert_eq!(mt_tmac_ctcr0(0), 0x820e_40f4);
        assert_eq!(mt_agg_acr0(0), 0x820e_2084);
        assert_eq!(mt_agg_pcr0(0, 1), 0x820e_2070);
        assert_eq!(mt_arb_scr(0), 0x820e_3080);
        assert_eq!(mt_wtbloff_top_rscr(0), 0x820e_9008);
        assert_eq!(mt_etbf_rx_fb_cnt(0), 0x820e_a158);
        assert_eq!(mt_dma_dcr0(0), 0x820e_7000);
        // Band 1 exists in the address map even though MT7921 is single-band;
        // `mt7921_mac_init` walks both (`mt7921/init.c:77-78`).
        assert_eq!(mt_wf_rfcr(1), 0x820f_5000);
        assert_eq!(mt_mib_sdr9(1), 0x820f_d02c);
        assert_eq!(mt_lpon_uttr0(1), 0x820f_b080);
    }

    /// The non-band-indexed strides: WTBL, MDP, PLE and the DMA scheduler.
    #[test]
    fn global_block_addresses_match_upstream_macros() {
        assert_eq!(MT_WTBLON_TOP_WDUCR, MT_WTBLON_TOP_BASE + 0x200);
        assert_eq!(MT_WTBL_UPDATE, MT_WTBLON_TOP_BASE + 0x230);
        assert_eq!(mt_wtbl_lmac_offs(0, 0), MT_WTBL_BASE);
        assert_eq!(mt_wtbl_lmac_offs(1, 0), 0x820d_8100);
        assert_eq!(mt_wtbl_lmac_offs(0, 1), 0x820d_8004);
        assert_eq!(mt_mdp_bnrcfr0(0), 0x820c_d070);
        assert_eq!(mt_mdp_bnrcfr1(1), 0x820c_d174); // band shifts by 0x100
        assert_eq!(mt_ple_ac_qempty(1), 0x820c_0540); // 0x40 stride
        assert_eq!(mt_ple_amsdu_pack_msdu_cnt(3), 0x820c_10ec);
        assert_eq!(mt_dmashdl_group_quota(4), 0x7c02_6030);
        assert_eq!(mt_dmashdl_q_map(3), 0x7c02_606c);
        assert_eq!(mt_dmashdl_q_map_shift(9), 4); // wraps at 8 entries/word
        assert_eq!(mt_dmashdl_sched_set(1), 0x7c02_6074);
        assert_eq!(mt_uwfdma0_tx_ring_ext_ctrl(17), 0x7c02_4644);
    }

    /// `field_prep`/`field_get` round-trip on the fields this port actually
    /// moves.
    #[test]
    fn field_helpers_round_trip() {
        assert_eq!(field_prep(MT_DMA_DCR0_MAX_RX_LEN, 1536), 0x0000_3000);
        assert_eq!(field_get(MT_DMA_DCR0_MAX_RX_LEN, 0x0000_3000), 1536);
        assert_eq!(field_prep(MT_IFS_SLOT, 9), 0x0900_0000);
        assert_eq!(field_get(MT_IFS_SLOT, 0x0900_0000), 9);
        assert_eq!(field_prep(MT_IFS_SIFS, 16), 0x0010_0000);
        assert_eq!(
            field_prep(MT_WL_TX_TMOUT_LMT, MT792X_USB_TX_TIMEOUT_LIMIT),
            0x00c3_5000
        );
        // The RCPI programming from `mt792x_mac.c:306-310`: MODE=0, PARAM=3.
        assert_eq!(
            field_prep(MT_WTBLOFF_TOP_RSCR_RCPI_MODE, 0)
                | field_prep(MT_WTBLOFF_TOP_RSCR_RCPI_PARAM, 3),
            0x0300_0000
        );
    }

    /// ★ The transcribed masks, checked against the values MEASURED on the
    /// target silicon. This is the only test here that touches hardware truth
    /// rather than upstream arithmetic.
    #[test]
    fn transcribed_masks_agree_with_measured_values() {
        // MT_HW_CHIPID: the whole word is the part number on this part, and it
        // is what upstream shifts into `mdev->rev` (`mt7921/usb.c:214`).
        assert_eq!(measured::MEASURED_HW_CHIPID, 0x7961);
        assert_eq!(measured::MEASURED_HW_CHIPID as u16, measured::USB_PID);
        // ★ MT_HW_REV's low byte is what upstream keeps (`rev & 0xff`), but the
        // low *half* is what must match the firmware header.
        assert_eq!(measured::MEASURED_HW_REV & 0xffff, 0x8a10);
        // ★ The firmware/silicon agreement: the patch header's hw_sw_ver top
        // half equals the measured revision. If this ever fails, the blob in
        // fw/mt7961/ is for a different die.
        assert_eq!(
            measured::PATCH_HDR_HW_SW_VER >> 16,
            measured::MEASURED_HW_REV & 0xffff
        );
        // The measured CONN_ON_MISC decodes as "no firmware running", which is
        // what a cold bring-up requires before it downloads any.
        assert_eq!(measured::MEASURED_CONN_ON_MISC & MT_TOP_MISC2_FW_PWR_ON, 0);
        assert_eq!(measured::MEASURED_CONN_ON_MISC & MT_TOP_MISC2_FW_N9_RDY, 0);
        // The MT7920 discriminator would have to be read from MT_HW_BOUND, not
        // from the chip id; assert the two are distinct registers so a future
        // shortcut cannot conflate them (`mt7921/pci.c:411`).
        assert_ne!(MT_HW_BOUND, MT_HW_CHIPID);
    }

    /// ★ Flag 4, made mechanical: the address the bench notes called
    /// "MT_TOP_MISC" is **not** [`MT_TOP_MISC`], and the reading taken there says
    /// nothing about firmware state.
    #[test]
    fn top_misc_is_not_the_measured_cbtop_word() {
        assert_ne!(MT_TOP_MISC, measured::MEASURED_CBTOP_00F0_ADDR);
        assert_eq!(MT_TOP_MISC, MT_TOP_BASE + 0xf0);
        assert_eq!(measured::MEASURED_CBTOP_00F0_ADDR, 0x7000_00f0);
        // The two live in different power domains — different top 16 bits, i.e.
        // different `wValue` on the wire.
        assert_ne!(MT_TOP_MISC >> 16, measured::MEASURED_CBTOP_00F0_ADDR >> 16);
        // The real firmware-state evidence we do have comes from a third
        // register entirely.
        assert_eq!(MT_CONN_ON_MISC, 0x7c06_00f0);
    }

    /// ★ Flag 2, made mechanical: nothing in [`pcie_only`] may be mistaken for a
    /// USB-usable address. Every quarantined address is below `0x0010_0000`,
    /// which is exactly the range `__mt7921_reg_addr` passes through untouched
    /// (`mt7921/pci.c:120-121`) — i.e. the range that only ever means "already
    /// remapped". Every address in the main body is above it.
    #[test]
    fn quarantined_addresses_are_distinguishable_from_physical_ones() {
        const PCIE_FORMS: [u32; 10] = [
            pcie_only::MT_WFDMA0_BASE,
            pcie_only::MT_WFDMA0_RST,
            pcie_only::MT_WFDMA0_BUSY_ENA,
            pcie_only::MT_MCU_CMD,
            pcie_only::MT_WFDMA0_GLO_CFG,
            pcie_only::MT_WFDMA_EXT_CSR_HIF_MISC,
            pcie_only::MT_MCU_INT_EVENT,
            pcie_only::MT_HIF_REMAP_L1,
            pcie_only::MT_PCIE_MAC_PM,
            pcie_only::MT_PCIE_MAC_INT_ENABLE,
        ];
        for addr in PCIE_FORMS {
            assert!(
                addr < 0x0010_0000,
                "{addr:#010x} should be a PCIe BAR offset"
            );
        }
        // The physical twins upstream states or we derived, at the same offsets.
        assert_eq!(
            MT_UWFDMA0_GLO_CFG - MT_UWFDMA0_BASE,
            pcie_only::MT_WFDMA0_GLO_CFG - pcie_only::MT_WFDMA0_BASE
        );
        assert_eq!(
            pcie_only::MT_WFDMA_EXT_CSR_HIF_MISC_PHYS - 0x7c02_7000,
            pcie_only::MT_WFDMA_EXT_CSR_HIF_MISC - pcie_only::MT_WFDMA_EXT_CSR_BASE
        );
        assert_eq!(
            pcie_only::MT_MCU_INT_EVENT_PHYS - 0x5500_0000,
            pcie_only::MT_MCU_INT_EVENT - pcie_only::MT_MCU_WFDMA1_BASE
        );
        assert_eq!(
            pcie_only::MT_HIF_REMAP_L1_PHYS - 0x7c00_e000,
            pcie_only::MT_HIF_REMAP_L1 - pcie_only::MT_INFRA_CFG_BASE
        );
        // ⚠ MT_WFSYS_SW_RST_B is quarantined for a different reason (wrong
        // *method*, not wrong address), so it is legitimately a high address.
        assert!(pcie_only::MT_WFSYS_SW_RST_B > 0x0010_0000);
        assert_ne!(pcie_only::MT_WFSYS_SW_RST_B, MT_CBTOP_RGU_WF_SUBSYS_RST);

        // Everything a USB bring-up touches is a real physical address.
        const USB_ADDRS: [u32; 9] = [
            MT_HW_CHIPID,
            MT_CONN_ON_MISC,
            MT_UDMA_TX_QSEL,
            MT_UDMA_WLCFG_0,
            MT_UWFDMA0_GLO_CFG,
            MT_WFDMA_HOST_CONFIG,
            MT_SWDEF_MODE,
            MT_WF_SW_DEF_CR_USB_MCU_EVENT,
            MT_WFDMA_DUMMY_CR,
        ];
        for addr in USB_ADDRS {
            assert!(
                addr >= 0x0010_0000,
                "{addr:#010x} looks like a PCIe BAR offset"
            );
        }
    }

    /// The `wValue`/`wIndex` split `___mt76u_rr` performs (`usb.c:76-90`, the split at 82-83),
    /// checked on the addresses actually measured — this is the whole of Flag 1
    /// reduced to arithmetic.
    #[test]
    fn addresses_split_cleanly_into_wvalue_and_windex() {
        for addr in [MT_HW_CHIPID, MT_HW_REV, MT_CONN_ON_MISC, mt_lpon_uttr0(0)] {
            let value = (addr >> 16) as u16;
            let index = addr as u16;
            assert_eq!(((value as u32) << 16) | index as u32, addr);
        }
        // The two MEASURED reads, spelled out.
        assert_eq!((MT_HW_CHIPID >> 16) as u16, 0x7001);
        assert_eq!(MT_HW_CHIPID as u16, 0x0200);
        assert_eq!((MT_HW_REV >> 16) as u16, 0x7001);
        assert_eq!(MT_HW_REV as u16, 0x0204);
    }

    /// The bring-up bit sets upstream writes as composites, so a wrong bit shows
    /// up as a wrong word rather than as a silently different behaviour.
    #[test]
    fn composed_bring_up_words() {
        // `mt792xu_dma_init` (`mt792x_usb.c:401-403`).
        assert_eq!(
            MT_WL_RX_EN | MT_WL_TX_EN | MT_WL_RX_MPSZ_PAD0 | MT_TICK_1US_EN,
            0x00d4_0000
        );
        // `mt792xu_wfdma_init` (`mt792x_usb.c:286-291`).
        assert_eq!(
            MT_WFDMA0_GLO_CFG_OMIT_TX_INFO
                | MT_WFDMA0_GLO_CFG_OMIT_RX_INFO_PFET2
                | MT_WFDMA0_GLO_CFG_FW_DWLD_BYPASS_DMASHDL
                | MT_WFDMA0_GLO_CFG_TX_DMA_EN
                | MT_WFDMA0_GLO_CFG_RX_DMA_EN,
            0x1020_0205
        );
        // `mt792xu_epctl_rst_opt` (`mt792x_usb.c:343`).
        assert_eq!(
            MT_SSUSB_EPCTL_RST_OPT_OUT_EP | MT_SSUSB_EPCTL_RST_OPT_IN_EP,
            0x0070_03f0
        );
        // `mt792xu_wait_udma_idle` (`mt792x_usb.c:351`).
        assert_eq!(MT_WL_RX_BUSY | MT_WL_TX_BUSY, 0xc000_0000);
        // `mt792x_mac_set_timeing`'s CCK and OFDM timeout words
        // (`mt792x_mac.c:40-43, 58-59`), coverage_class = 0.
        assert_eq!(
            field_prep(MT_TIMEOUT_VAL_PLCP, 231) | field_prep(MT_TIMEOUT_VAL_CCA, 48),
            0x0030_00e7
        );
        assert_eq!(
            field_prep(MT_TIMEOUT_VAL_PLCP, 60) | field_prep(MT_TIMEOUT_VAL_CCA, 28),
            0x001c_003c
        );
    }

    /// The three airtime counters share a 24-bit width, so they share a wrap
    /// period — ~16.8 s at 1 µs/tick. A sensing window longer than that reads a
    /// wrapped value as a small one, which is the silent-failure shape this
    /// codebase keeps meeting.
    #[test]
    fn airtime_counters_share_a_24_bit_wrap() {
        assert_eq!(MT_MIB_SDR9_BUSY_MASK, MT_MIB_SDR36_TXTIME_MASK);
        assert_eq!(MT_MIB_SDR36_TXTIME_MASK, MT_MIB_SDR37_RXTIME_MASK);
        assert_eq!(MT_MIB_SDR37_RXTIME_MASK, MT_MIB_OBSSTIME_MASK);
        assert_eq!(MT_MIB_OBSSTIME_MASK, 0x00ff_ffff);
        // 2^24 µs ≈ 16.777 s.
        assert_eq!((MT_MIB_OBSSTIME_MASK as u64 + 1) / 1_000, 16_777);
    }
}
