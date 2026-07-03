# BW16 (RTL8720DN) radio-knob findings

What the Rust firmware can and cannot control on the raw-inject path, established
empirically (inject from the BW16, capture MCS/RSSI on an RTL8812EU witness).

| Knob | API | Result |
|------|-----|--------|
| Channel + band | `wifi_set_channel` | ✅ works (2.4 + 5 GHz) |
| TX data rate / MCS | `wifi_set_tx_data_rate` | ❌ no effect on inject (mgmt path is fixed-rate) |
| Bandwidth 40 MHz | `wext_set_bw40_enable` | ❌ no effect on inject |
| TX power | `wifi_set_txpower` | ❌ reachable (reimplemented past `#if 0`) but a **driver stub** — RSSI flat 0..63 |

## Flagged for future reverse-engineering

**A rate/BW/power-controllable inject is not reachable through the management-TX
path.** `wifi_tx_raw_frame` uses `alloc_mgtxmitframe → update_mgntframe_attrib →
dump_mgntframe`; 802.11 management frames are legacy-rate by design and the
Realtek mgmt TX descriptor forces a legacy rate. Probing the opaque `pkt_attrib`
(at `xmit_frame+8`) byte-by-byte with `MGN_MCS7` (see `examples/bw16_probe_rate`,
`T_INJECT_ATTR`, `Bw16SerialBackend::inject_attr`) produced **no** HT frame across
offsets 0..56.

Two deeper paths remain, each a real multi-session effort:

1. **Rate / bandwidth** — inject via the **data-path xmit** (`xmitframe_enqueue` /
   the data TX descriptor) instead of the mgmt queue, where the rate/`ht_en`/`raid`
   fields are honored. The `inject_attr` probe harness is reusable to map those
   descriptor fields.
2. **TX power** — the PHY power path (`TxPowerByRate` table / RF power-index
   registers), below the stubbed `txpower` iwpriv command.

The Rust firmware (owning its own link) is the right place to do both — it can
reach SDK symbols the prebuilt Arduino link drops.
