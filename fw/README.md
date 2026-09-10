# Realtek RTL8822E firmware

`rtl8822e_fw_nic.bin` is Realtek's WLAN-CPU firmware for the RTL8822E family
(which includes the RTL8812EU dongle this crate's `libusb-backend` drives),
extracted byte-for-byte from the `array_mp_8822e_fw_nic` C array in the vendor
reference driver `svpcom/rtl8812eu` (`hal/rtl8822e/hal8822e_fw.c`, also shipped
as `OpenHD/rtl88x2eu`). Header: signature 0x2288, version 1.27, built
2024-09-04; sections DMEM 14,400 B + IMEM 56,568 B + EMEM 128,808 B (each with
an 8-byte checksum tail) behind a 64-byte header — 199,864 bytes total.

This is proprietary Realtek firmware (data executed by the on-chip CPU, not
host code), redistributed in binary form the same way the kernel driver and
linux-firmware do. The matching golden runtime state from this exact version is
in `../golden/opi0-2026-06-12/` ("FW VER -1.27").

## phydm BB/RF tables

`rtl8822e_phy_reg.bin`, `rtl8822e_agc_tab.bin`, `rtl8822e_radioa.bin`,
`rtl8822e_radiob.bin` are the baseband and RF register tables, extracted
verbatim (LE u32 words) from the phydm `array_mp_8822e_*` arrays in
`hal/phydm/rtl8822e/halhwimg8822e_bb.c` and
`hal/phydm/halrf/rtl8822e/halhwimg8822e_rf.c`. Each is a condition-encoded
stream: a leading headline of `{cut, rfe_type}` variant descriptors followed
by an IF/ELSE/END/CHK body of `(addr, data)` pairs (see `load_table` /
`HeadlineSel` in `libusb_rtl88xx.rs`). The driver loads these to bring up the
PHY; our port replays the same bytes. Verified: after loading them and
switching to channel 161, 509/512 BB registers and the RF channel/bandwidth
registers match the golden kernel state.

## RF calibration setup table

`rtl8822e_cal_init.bin` is the `array_mp_8822e_cal_init` table from
`hal/phydm/halrf/rtl8822e/halrf_rfk_init_8822e.h` — straight `(addr, data)` BB
register pairs (not condition-encoded) that arm the calibration blocks before
the iterative RF calibrations run. Loaded by `rf_cal_init()` as part of the
kernel's `_init_rf_reg` flow, after the BB/AGC tables and before the RadioA/B
tables. Deterministic; the calibration loops themselves (DACK and the heavier
unported IQK/LCK/DPK/TSSI) are code, not tables.

The reference driver also ships variants we don't currently embed: `fw_10M`
(5/10 MHz narrowband PHY), `fw_wowlan` (wake-on-WLAN pattern matching), and
`fw_ap`/`fw_spic` — see the named-radio knobs note for why the first two are
interesting.

## MediaTek MT7610U firmware (`mt76x0/mt7610u.bin`)

MediaTek's MCU firmware for the MT7610U (mt76x0 family), copied byte-for-byte
from `linux-firmware`'s `mediatek/mt7610u.bin` as shipped on the lab's NixOS
nodes (`/run/current-system/firmware/mediatek/mt7610u.bin.zst`, decompressed).

    sha256  5a4268e9021bb587426ba624b425f1e660bfc82cd63b36ad3ce6fb9ce6751760
    size    80,288 B

Header (`struct mt76x02_fw_header`, 32 B) read from the blob itself:
`ilm_len = 0x00010cac` (68,780) + `dlm_len = 0x00002cd4` (11,476), and
`32 + 68780 + 11476 = 80288` — the file's own length, so the split is confirmed
by arithmetic, not assumed. `build_ver = 0x7640`, `fw_ver = 0x0100`,
`build_time = "201308221655____"`.

Unlike the MT7612U, this part has **no ROM patch** — `mt76x0u_load_firmware`
loads ILM/DLM only. Proprietary vendor firmware (data executed by the on-chip
MCU, not host code), redistributed in binary form exactly as linux-firmware and
the in-tree `mt76x0u` driver do.

## MediaTek MT7961 firmware (`mt7961/`) — the MT7921AU

MediaTek's patch + RAM firmware for the MT7921AU (connac2), copied byte-for-byte from
`linux-firmware` as shipped on the lab's NixOS nodes
(`/run/current-system/firmware/mediatek/WIFI_*_MT7961_*.bin.zst`, decompressed).

    WIFI_MT7961_patch_mcu_1_2_hdr.bin   92,192 B  sha256 1cf118a88b131202cceeb480441df91e…
    WIFI_RAM_CODE_MT7961_1.bin         791,588 B  sha256 b42237d20b1375a5160d9f220ea34723…

Header facts read from the blobs themselves, not assumed:

* The patch begins with a 16-byte build date `"20250625153620a\n"`, then the platform tag `"ALPS"`,
  then `hw_sw_ver = 0x8a10` — which **equals the `MT_HW_REV` (0x70010204) read from the silicon**,
  so the blob and this chip are a matched pair and the driver checks it. The patch magic
  `0x11223344` sits at offset 32.
* The RAM image carries its `mt76_connac2_fw_trailer` at the **end** (`…"____" "0100"
  "00202506251537" 03`), with the region count in it — the regions are walked backwards from there,
  which is the opposite of the mt76x0/mt76x2 header-at-the-front layout.

Proprietary vendor firmware (data executed by the on-chip MCU, not host code), redistributed in
binary form exactly as linux-firmware and the in-tree `mt7921u` driver do.
