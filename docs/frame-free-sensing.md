# Frame-free sensing on the RTL8812AU — occupancy from a register, not a demod (#30)

**TL;DR.** A named-data node can read channel occupancy straight from a MAC
hardware counter without the host decoding a single frame. On the 8812au that
counter is **`REG_RXERR_RPT` (`0x0664`, bits[15:0])**: over a window its delta
tracks the decoded-frame rate ~1:1 on a busy channel and reads 0 on a quiet one —
validated on-chip, quiet-vs-busy. This is the raw input the radio-cognition plane
can sense the medium with, for the cost of one register read per window instead of
demodulating every packet.

## What was built

`rtl8812au.rs`:
- `PhySense { igi_a, igi_b, rx_activity }` + `read_phy_sense()` — one snapshot of
  the AGC initial-gain (noise-floor proxy, `0xC50`/`0xE50`) and the `0x0664`
  activity counter. `PhySense::delta()` diffs two snapshots (16-bit wrap-aware)
  for a per-window rate. No TX, no frame decode.
- `examples/sense_probe.rs` — labelled mode (per-window activity/s vs decoded/s
  as ground truth) and a `scan` mode (diff a range of PHY status registers across
  a busy window and print what changed — the tool that *found* the counter).

## How the register was found (measurement, not headers)

The phydm Jaguar recipe points at an OFDM FA/CCA block at `0xF48`–`0xF54` (low
word of `0xF4C` = `cnt_ofdm_cca`). Coded that first — and on-chip it read a **flat
0 under real traffic** (the counters are unarmed by our minimal bring-up). The
upstream headers were also un-greppable through the fetch path. So per the
hardware-truth method (suspect the instrument, measure don't reason), `sense_probe
scan` diffed the PHY status pages (`0xF00`–`0xFFC`, `0xA00`–`0xAFC`, `0x0660`–`0x06A0`)
across a busy window and printed every register that moved. Then the **same scan
on a quiet channel** separated occupancy-driven registers from free-running
timers:

| register | busy Δ | quiet Δ | verdict |
|---|---|---|---|
| **`0x0664`** | **63 (≈65 frames)** | **0** | **occupancy counter — shipped** |
| `0x0FD0` | 99 | 99 | free-running timer — rejected |
| `0x0F38[31:16]` | 2365 | (later 0, erratic) | failed re-validation — rejected |
| `0xF48`–`0xF54` | 0 | 0 | phydm FA block, unarmed here — rejected |

Only `0x0664` survived. `0x0F38` looked promising in the first scan but on
re-validation read 0 on busy and huge/erratic on quiet, so it was dropped — a
reminder that one scan is a hypothesis, not a result.

## Validation (labelled, quiet vs busy, 2 s windows)

```
QUIET ch1:  activity/s = 0, 0, 0        decoded/s = 0, 0, 0
BUSY  ch6:  activity/s = 26,15,13,23,12  decoded/s = 26,16,13,22,12
```

`activity/s` tracks `decoded/s` to within a frame or two, and the host never ran
a demod to produce it — it read one register at each window boundary. IGI sat at
32 throughout: at this sparse ambient the *noise floor* doesn't move (IGI would
rise under sustained energy/interference, not a few frames/s), so IGI is a real
noise-floor sensor but not a per-frame occupancy signal.

## Why this matters for named-radio

- It answers the "does a named node have to process everything?" tax directly:
  **no** — occupancy comes from a counter, complementing the name-group hardware
  RX filter (#43, which already drops non-subscribed frames in silicon). The node
  senses how busy the air is without waking the host per frame.
- It's the clean input layer for the cognitive plane (`ndn-radio-cognition`):
  feed `rx_activity`/s (and IGI) as the medium-load signal the rate/redundancy
  policy already wants. That wiring is the natural follow-on.
- Honest scope: `0x0664` is a *frame-activity* counter (the MAC counts frames it
  received), not a pure energy/CCA busy-time meter. It captures occupancy from
  decodable traffic well; sensing non-decodable interference energy would need the
  CCA/FA counters armed (the `0xF48` block, or `REG_RXERR_RPT`'s selector field
  reprogrammed) — an open follow-on if interference-vs-traffic discrimination is
  wanted.
