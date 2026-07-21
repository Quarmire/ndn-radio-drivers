# EDCCA on the RTL8812AU — measured, and why it can't rescue a contended channel (#37)

**TL;DR.** EDCCA (energy-detect carrier sense) is now wired on the 8812au and
register-verified, and it demonstrably reaches the injection TX path. But on a
saturated channel it does **not** cut our losses: any threshold sensitive enough
to detect the ambient contention detects it ~continuously, so TX defers ~always
(starvation); any threshold permissive enough to keep TX flowing doesn't sense
the contention at all (no benefit). Carrier-sense trades collision-loss for
airtime-starvation — and you can't sense your way onto a channel that's already
full. This sharpens #36: the fix for contention is a **quiet channel** (ch14) or
**coding through the loss** (FEC, #34), not CSMA.

## What was built

`rtl8812au.rs`, matched to the aircrack-ng/rtl8812au phydm adaptivity path:

- `set_edcca_threshold(l2h, h2l)` — writes the signed busy-enter / busy-exit
  thresholds to the low word of `0x8a4` (`rFPGA0_XB_LSSIReadBack` on Jaguar;
  byte0 = L2H, byte1 = H2L), preserving the upper half.
- `set_edcca_honor(bool)` — clears/sets `REG_TX_PTCL_CTRL` (`0x0520`) **BIT15**
  (the *ignore-EDCCA* bit) and the `REG_RD_CTRL` (`0x0524`) **BIT11** gate, so the
  MAC TX backoff waits for the medium to fall below the threshold before keying up.
- `enable_edcca(l2h)` / `disable_edcca()` — one-call on (with 7-unit hysteresis)
  and restore-the-blast-default.
- `edcca_state()` — reads back `(l2h, h2l, honored)` to verify the actuation.

`0x7f/0x7f` + ignore = the promiscuous-injection default (energy detect off).

The probe: `examples/edcca_probe.rs` — a two-radio A/B (o5p-0 TX, o5p-1 RX) that
sweeps L2H, embeds a phase byte per frame so the receiver buckets each arm from
one promiscuous capture, and counts TX-side deferral stalls (a `send_frame`
timeout = the TX FIFO couldn't drain because EDCCA held the medium busy).

## The measurement (ch6, ~99 ambient frames/s, 3 ft bench, 400 frames/phase @ 300/s)

| L2H (register units) | sent-ok | deferral stalls | received | delivery ratio | delivered/s |
|---|---|---|---|---|---|
| off (0x7f) | 400 | 0 | 315 | 78.8% | **237** |
| −12 | 400 | 0 | 272 | 68.0% | 205 |
| −14 | 400 | 0 | 295 | 73.8% | 222 |
| −16 | 400 | 0 | 322 | 80.5% | 242 |
| **−18** | **145** | **45** | **128** | **88.3%** | **26** |
| −20 | (first frame → USB bulk-out timeout: TX totally stalled) | | | | |

Instrument verified each run via `edcca_state()` readback (e.g. phase −18 read
back `L2H=-18 H2L=-25 honored=true`).

## Reading it

- **The gate is real and reaches the injection path.** At −18 the on-air rate
  collapsed 300 → 29/s with 45 timeouts; at −20 the very first frame timed out
  (TX FIFO never drained). That is the MAC deferring to energy it can't decode —
  exactly EDCCA. It is not a no-op.
- **But it's a cliff, not a knob.** −12…−16 produce **zero** stalls and a
  delivery profile indistinguishable from "off" (68–80% is just run-to-run
  contention variance). The threshold is simply above the ambient energy, so
  nothing defers and nothing improves.
- **−18 "improves the ratio" only by starving the sender.** 88% of the 145 that
  escaped got through — but absolute delivered throughput cratered from 237/s to
  **26/s** (9× worse). You didn't cut loss; you stopped transmitting.

The reason is structural: ch6 carries near-continuous OFDM energy, so any
threshold that senses real WiFi senses it ~90% of the time. CSMA's answer to a
busy medium is *wait*, and waiting is not delivery. There is no intermediate
setpoint that both keeps TX flowing and dodges the collisions, because the
collisions and the "keep flowing" airtime are the same airtime.

## Consequence for named-radio

- EDCCA is the correct, polite behaviour and it's now available as a knob
  (`RadioKnobs`-wireable), but it is **not** the contention remedy. Confirms #36.
- The remedies that actually moved delivery: **change channel** (ch14 quiet →
  40/40 in #36) and **code through the residual loss** (link-FEC #34: R=4 → 82/120
  on the quiet channel). EDCCA composes with those (defer politely, then FEC the
  collisions you still take), but it cannot substitute for them.
- Open follow-on: on a *moderately* loaded channel (not saturated) EDCCA should
  have a genuine operating point — the cliff becomes a knob when the medium is
  idle enough of the time. Worth a sweep on a mid-load channel to find it.
