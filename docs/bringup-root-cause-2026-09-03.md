# Why the shared bring-up "did not transmit" on the RTL8812AU

**It always transmitted.** It transmitted tens of dB too quietly to close the link to the witness,
and every layer above reported success. Measured 2026-09-03; mds-o5p-2 TX, AR9271 on mds-o5p-1 as an
independent-vendor witness in monitor on the same channel, tshark filtered on the frame's own address.

## The defect

`set_tx_power(idx)` means **two different physical powers** depending on whether
`load_tx_power_info()` ran earlier in the bring-up:

| calibration loaded? | what `set_tx_power(0x3f)` writes | on this ch6 adapter |
|---|---|---|
| **no** — falls through to `set_tx_power_raw` | raw TXAGC index `0x3f` | **index 63** (chip maximum) |
| **yes** | `index_base(fused) + (idx − 63)`, i.e. a **back-off from the fused regulatory base** | **≈ index 27** |

The register→power transfer on this part is monotone over ~33 dB across the 0–63 span (#38,
conducted SDR), so those two calls differ by roughly **18–33 dB** — the difference between closing
the link to the witness and not.

`examples/nav_probe.rs` never calls `load_tx_power_info`, so its `set_tx_power(0x3f)` took the raw
path and ran hot. `bring_up_monitor` loads calibration, so the identical call ran at the regulatory
base. **Same API, same argument, same `Ok(())`, ~20+ dB apart, decided by a step three calls
earlier that the caller cannot see.**

## The measurement trail, including the three wrong answers

The symptom was "0 frames on air". Three plausible, code-founded hypotheses were tested and
**refuted** before the real one — recorded because each was convincing and each was wrong:

| # | hypothesis | test | result |
|---|---|---|---|
| 1 | `iqk_configure_mac` leaves `REG_TXPAUSE = 0x3f` and nothing restores it (true! and `nav_probe` has a hand-rolled clear) | clear it in the shared path | **still 0** — refuted |
| 2 | the driver binds `bulk_out` to the FIRST enumerated OUT endpoint (0x02); the example pins 0x04 "the one that radiated" | sweep 0x02 / 0x03 / 0x04 | **0 / 0 / 0** — refuted |
| 3 | `init_llt` (TX page chain) has *one* reference in the crate — its own definition; only the example calls it | add it to the shared bring-up | **still 0** — refuted |

What actually located it was a **discriminator, not a guess**: hold everything constant and swap one
thing at a time.

* Swap only the *transmit call* (`send_frame_ep` → production `FrameIo::inject`) on the known-good
  bring-up: **6277 → 3548 frames on air.** Transmit path, endpoint and frame shape cleared.
* Swap only the *bring-up* (hand-rolled → `bring_up_monitor`), same open and same transmit:
  **3630 → 0.** The defect is in the bring-up.
* Bisect the three steps the shared bring-up does that the example does not:

  | skipped | frames on air |
  |---|---|
  | `pwrinfo,edcca,iqkloop` | 3064 |
  | `edcca` | **0** |
  | **`pwrinfo`** | **3114** |
  | `iqkloop` | **0** |

  Two arms pass, and they are exactly the two that skip `load_tx_power_info`.
* Confirm the mechanism directly — keep the calibrated bring-up, re-write raw TXAGC afterwards:
  **0 → 3074.**

Once the link closed, the two speculative fixes from hypotheses 1 and 3 were **A/B'd and reverted**:
with raw power applied, skipping `llt`, `txpause`, or both transmits identically
(2789 / 4508 / 4588 / 4406 frames — run-to-run noise, no direction). Neither is load-bearing for
transmit on this part, so neither is kept. Adding steps "just in case" is the disease, not the cure.

## A latent hazard, measured and correctly NOT fixed

`8307161` (2026-08-31, "witness receiver, firmware H2C, and four measurement fixes") changed
`lc_calibrate`'s teardown from **unconditionally un-pausing**

```rust
self.write8(REG_TXPAUSE, 0x00)?;      // before
self.write8(REG_TXPAUSE, prev)?;      // after
```

IQK runs before it and leaves `REG_TXPAUSE = 0x3f`, so a *faithful* restore can now preserve a paused
MAC where the old code accidentally cleared it. Worse, the IQK retry loop re-enters `iq_calibrate`,
which re-reads the register backup — so if attempt *n* failed, attempt *n+1* saves `0x3f` and
faithfully restores it. Whether a bring-up ends paused therefore depends on whether the **last** IQK
attempt started from a clean state, which is not deterministic.

**Measured, three independent bring-ups: `REG_TXPAUSE = 0x00, 0x00, 0x00`.** The hazard is real in
structure and **not firing today**, which is why hypothesis 1 was correctly refuted and why no
TXPAUSE clear was kept. This is exactly the class of thing the contract should *assert* at the end of
bring-up rather than leave to the ordering of a retry loop — an invariant to check, not a step to add.

⚠ Consequence for the record, from the inventory: examples that end their ladder at `lc_calibrate`
and never clear TXPAUSE relied on the pre-`8307161` accidental un-pause. Any on-air number in them
dated after 2026-08-31 should be re-checked before it is quoted.

## What this says about the design, not the bug

The bug is a symptom. Four properties of the current arrangement made it possible, and a canonical
bring-up has to remove all four:

1. **An API whose meaning depends on hidden earlier state.** `set_tx_power` is a raw index or a
   regulatory back-off depending on whether a different method ran. One name must mean one thing.
2. **Several bring-up sequences per part, with no owner.** The knowledge that one of them ran hot
   lived in an example, uncommented. `nav_probe` also carried `write8(0x522, 0x00)` — a private
   workaround for a shared-path defect — and the comment "endpoint 0x04 is the one that radiated in
   the inject8812au sweep", i.e. a measured result stored in an example instead of the driver.
3. **A bring-up that returns `Ok(())` and nothing else.** It should report the regime it left the
   radio in — calibration loaded or not, the TXAGC index actually written, the channel, the rate —
   so this failure is one assertion away instead of a day of bisection.
4. **No transmit evidence.** `inject` returning `Ok` proves a bulk-OUT write completed. Nothing in
   the stack distinguishes "radiated" from "queued into a MAC that will never key the RF" — and, as
   this case shows, not even from "radiated 20 dB below what the link needs". The transmitter cannot
   detect this alone; the design has to make the regime legible instead.

## Not measured

* Whether index ≈27 is the *correct* regulatory answer. It probably is — the raw path may exceed
  licensed EIRP, which the driver already warns about. **This document does not conclude that
  calibration is wrong**, only that the same call must not silently mean both things.
* Whether the other Realtek/mt76/ath9k bring-ups have the same split. Unswept here.
* `set_tx_power`'s position (moved after IQK) was changed on reasoning and is **not** individually
  validated on air.

## ☠ RETRACTED — "the bench link is marginal"

**This section previously reported a power sweep (raw idx 63 → 2301 frames at −85.6 dBm; idx 55 →
0) and concluded the bench link closes only at chip maximum. That conclusion is WRONG and is
withdrawn.**

Everything on this bench is a few feet apart. The sweep was taken through a **kernel** witness
(`ath9k_htc` + `tshark` radiotap), and its −85.6 dBm is ~20 dB below this bench's own documented
figure of **−60 to −67 dBm, symmetric, adequate — 30 dB over 6M sensitivity**
(`wifi-loss-is-contention`, root-caused 2026-07-17). A number 20 dB below the established value for
the same bench is an instrument reading, not a link.

**Measured 2026-09-04 with our own libusb stack on both ends and matched frame formats** — RTL8812AU
TX (mds-o5p-2) → RTL8822E/a81a RX (mds-o5p-0), ch149, **at the CALIBRATED regulatory base**
(`ref=FusedBase{base:44, ch:149}`, `req=Ceiling`, no `NDN_RF_UNRESTRICTED`):

```
── VERDICT: 1000 frames, byte-exact 1000 | addr4 lost 0 altered 0 | QoS lost 0 altered 0 |
   HTC lost 0 altered 0 | base altered 0 → this chip CARRIES the 190-bit filter
```

**At the calibrated base, at bench range, on our own stack, the link is perfect.** So:

* the "0 frames at `Ceiling`" results that produced the marginal-link story were **instrument
  failures**, not link failures (see below);
* §7.5's "the bench link is still marginal" is withdrawn;
* the M7 acceptance's criteria 1–2 are **not blocked by the link**.

### The three instrument failures, so they are not repeated

| witness | stack | ambient traffic seen | our frames seen | verdict |
|---|---|---|---|---|
| a81a via `au_witness` | ours | 1277–1385 (49–53 f/s) | **0** | ☠ **format mismatch**: `au_witness` opens `FrameFormat::Raw80211` while `tx_flood_8812au` transmits `RawNdn{0x8624}` |
| AR9271 via `au_witness` | ours | **0** | 0 | ☠ our AR9271 libusb RX delivers **nothing at all** — not a link result |
| AR9271 via `tshark` | **kernel** | n/a | thousands | the only path that produced numbers, and the one doctrine says to avoid |
| a81a via `wide_profile_onair` | ours | n/a | **1000/1000 byte-exact** | matched formats, both ends ours — the correct instrument |

The lesson is the project's own: **suspect the instrument before the world.** Three witnesses
disagreed; the two that said "no link" were broken, and the conclusion was drawn from the broken
ones because they were the ones that produced a number.

## The consequence nobody had noticed: the node and the bench are not the same radio

`ndn-radio/crates/faces/ndn-radio/examples/wireless_node.rs:209` — the node binary — calls
`open_named_radio(pid, ch)`. On an RTL8812AU that runs `bring_up_monitor`, which loads calibration,
so the node transmits at the **fused regulatory base (≈ TXAGC idx 27)**.

The 16 examples that hand-roll the ladder never call `load_tx_power_info`, so their identical
`set_tx_power(0x3f)` writes **raw index 63**.

**Every on-air characterisation done with those examples — range, delivery ratio, throughput,
rate-adaptation behaviour, contention results — was measured on a transmitter running up to ~20 dB
hotter than the node that ships.** Nothing in the code, the logs or the capability declaration says
so. This is not a second bug; it is the same ambiguity, seen from the other end, and it is the
strongest argument that the contract must make the power regime a reported fact rather than a
consequence of which ladder you happened to call.

Scope: this split is **8812au-specific**. `libusb_rtl88xx` (the a81a/8822E) exposes a single
`set_tx_power(idx)` with no calibration-conditional branch, and the 8733b returns its
`read_tx_power_info` to the caller instead of stashing it as hidden state — a sibling driver already
avoids the defect, which is evidence the contract is meetable rather than aspirational.

## Acceptance — M0/M1/M2 on hardware, 2026-09-03

The fix is not "it compiles". On the 8812au (mds-o5p-2), AR9271 witness (mds-o5p-1), ch6:

**Without written authority, the raw axis is refused** — and the refusal names the physical
consequence, not just the rule:

```
Error: Unsupported: "PowerRequest::Raw leaves the regulatory scale (raw chip TXAGC, ~18-33 dB hotter
than the fused base on the RTL8812AU, may exceed licensed EIRP). It needs an explicit operator
decision: set NDN_RF_UNRESTRICTED="<operator>:<reason>" — the reason is printed verbatim in every
report of the run."
```

**With it, the run transmits (3027 frames at the witness) and says what it did:**

```
       writes 10 regs  idx span 63..63   [FiveMeasured]  ⚠ AUTHORITY pmle:"M1 acceptance …"
```

**And the calibrated bring-up now reports its regime before a frame is sent:**

```
radio RTL8812AU  plan rtl8812au/monitor@v1  digest 0x2cb8ddb1b28afbba  CANONICAL   1.23 s
  ch 6 / Bw20   role TransmitAndReceive   pump CallerOwns
  power  ref=FusedBase{base:27, ch:6}  req=Ceiling(FiveMeasured)  clamped=no  dbm=None  actuated
         writes 10 regs  idx span 21..27   [FiveMeasured]
  steps  power_on · download_firmware · mac_config · … · load_tx_power_info · … · set_tx_power
  tx     UNPROVABLE — no Jaguar1 MAC->BB counter ported; CCX/SPE_RPT is lossy (1-4 records per
         ~2300 armed) and needs a running RX pump. Prove with a witness.
```

**`idx span 21..27` versus `63..63`.** The ~20 dB fork that cost a day of bisection is now the second
line of output, on both paths, with the reference named. Per §7.1 this is *visibility, not
detection* — it would not have raised an alarm by itself, because nothing here knows what the link
needs. It would have made the diagnosis take minutes. That is the claim, and it is the whole claim.

Note the last line: the contract **refuses to claim transmission it cannot prove** on this part, and
says what would prove it. That is §4 working as specified.

