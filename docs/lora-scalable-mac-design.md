# A scalable, receiver-agnostic MAC for the LoRa named-radio face (#52)

**TL;DR.** The half-duplex fix in `examples/lora_cognition.rs` closes NDN round
trips on two OPis (PER 1.00 → 0.04), but two of its mechanisms are **pairwise** and
do not scale: a name-ordered TX offset (`name < peer`, a 2-party TDMA contract that
keys on host identity — against the named-data doctrine) and a fixed `RECEIVE_DELAY`
that assumes exactly one requester. This doc designs the receiver-agnostic
replacement: **CAD-based CSMA (listen-before-talk)** for channel access and
**randomized-reply-with-overhearing-suppression** for Data, keeping the parts that
already generalize (soft-state PIT aggregation, round-robin pacing). Carrier-sense
must run **atomically in the firmware** (an LBT mode on `CMD_TX`, not a host-polled
CAD — the serial round-trip breaks sense-then-transmit); since reflashing over ST-Link
is the expensive step, that one flash also batches sensing (`CMD_CAD`/`GET_RSSI`),
runtime-tunable CAD/LBT/preamble config, a hardware RX timestamp, a CAD-based SF-scan
(ASFS) primitive, and observability counters — the four tiers below. The suppression
half is host-only. No code or reflash yet — this is for review.

Related: `edcca-contention-findings.md` (#37, the 802.11 sibling of CSMA here),
`frame-free-sensing.md` (#30), the LoRa ADR literature note, and the named-radio
doctrine (`mac-addressing-doctrine`, `named-data-radio-no-host-identity`).

---

## Where we are — the N=2 baseline (measured)

`lora_cognition.rs`, two OPis, 89 dB of attenuation (RSSI −90/−93 dBm), SF10:

- **PER 1.00 → 0.04**, symmetric, all three traffic classes served fairly.
- Fixed by (1) **one Interest per tick** (round-robin — a burst of three kept the
  radio deaf across ~3 frame-times, so the peer's Data reply to the first Interest
  collided with our own second Interest) and (2) a **400 ms `RECEIVE_DELAY`** before
  each Data reply, so the requester has finished turning its radio from TX to RX.

Both are real fixes. But #2's delay is timed to *the one requester's* turnaround, and
the TX schedule that keeps the two nodes from talking over each other is a half-tick
offset assigned by `name < peer`.

## Why it doesn't scale

1. **Name-ordered TX offset is a 2-party contract.** `if node.name < node.peer { 0 }
   else { TICK/2 }` defines a schedule only for a *pair*, and it keys the schedule on
   host identity — exactly the design `named-data-radio-no-host-identity` says not to
   build. For 3+ nodes it does not even define non-overlapping slots.
2. **Fixed `RECEIVE_DELAY` assumes one requester.** On a broadcast medium an Interest
   is heard by many, multiple nodes may hold the Data, and one broadcast Data should
   satisfy *all* pending Interests. A single fixed delay to "the requester" is the
   wrong model: it neither de-duplicates responders nor serves multiple askers with
   different turnaround phases.

What *does* generalize and stays: **round-robin self-pacing** (a node not hogging the
medium) and the **soft-state PIT aggregation** (many pending Interests for a name
collapse to one in-record; one Data retires them all).

## Design principles

From the doctrine, made concrete for this MAC:

- **Receiver-agnostic.** Access and reply decisions use only what any node can
  observe on a broadcast channel (is it busy? did I overhear this name answered?),
  never who asked or how many.
- **Soft-state, no host identity.** No peer tables, no pairwise schedules. The source
  field stays an ephemeral per-frame nonce; RSSI keys on it, not on a node id.
- **Name is the rendezvous key.** Where two ends must agree (the modulation-
  rendezvous parameters below), they agree by *deriving the same value from the
  name*, not by negotiating.

## The scalable MAC

### 1. Channel access — CAD-based CSMA / listen-before-talk (IN FIRMWARE)

Replace the name-ordered offset with contention-based access that any number of nodes
share. **The listen-before-talk loop lives in the firmware, not the host** — see the
atomicity note below. The host sends `CMD_TX` with an LBT flag; the firmware runs:

```
CMD_TX{lbt}:                                    # atomic on the MCU, no serial in the loop
    for attempt in 0 .. MAX_ATTEMPTS:
        if CAD() == CLEAR:                       # SX1262 Channel Activity Detection
            key_up(); transmit(); reply SENT; return
        backoff = min(attempt, MAX_BACKOFF)
        wait(hw_rng() mod (CW << backoff))       # binary exponential backoff, HW-RNG jitter
    reply DEFERRED(attempts)                      # channel stayed busy; host may retry/age
```

- **Atomicity — why it must be in firmware.** If the host called a `CMD_CAD`, got the
  result, then sent `CMD_TX`, the serial round-trip (~5–20 ms at 115200) sits *between*
  "sensed clear" and "key up" — inside a LoRa preamble window. Two nodes could both
  sense clear, both eat their serial latency, and both transmit → collision. The
  `sense → backoff → key-up` loop therefore has to run on the MCU with no serial gap.
- **CAD, not decode.** SX1262 CAD detects a LoRa preamble/energy on-channel in a few
  symbols *without* decoding — it catches an in-progress transmission we could never
  demodulate (e.g. a frame at a different SF), which pure RX-based sensing misses
  until too late. This is the LoRa-native carrier sense; the analogue of EDCCA (#37).
- **Backoff is per-attempt and random** (seeded from the SX1262 hardware RNG — the MCU
  has none), so N nodes de-synchronize without any identity or schedule. At N=2 this
  reduces to "usually clear, transmit immediately"; it does not need the offset.
- **Caveat from #37:** carrier sense cannot rescue a *saturated* channel (it trades
  collision-loss for TX starvation). LoRa's duty-cycle discipline keeps our channel
  far from saturation, so CSMA operates in the regime where it helps.
- A standalone `CMD_CAD` / `CMD_GET_RSSI` still exists — for the cognition plane's
  channel-occupancy *sensing* (the LoRa cousin of frame-free sensing #30), which is a
  separate concern from the access loop and tolerates the serial latency.

### 2. Data reply — randomized delay + overhearing suppression

Replace the fixed `RECEIVE_DELAY` with the classic NDN-over-broadcast strategy
("listen first, broadcast later"):

```
on Interest(name) for data we hold:
    schedule a reply at now + RX_GUARD + random(0 .. REPLY_JITTER)
on overhearing Data(name):        # someone else already answered
    cancel any scheduled reply for name
when a scheduled reply fires (not cancelled):
    CSMA-transmit the Data          # access rule (1) still applies
```

- **`RX_GUARD`** (a small floor, ~one turnaround) preserves the turnaround fix that
  the fixed delay gave us — but it is now a floor, not the whole timing.
- **`REPLY_JITTER`** de-synchronizes multiple responders; the first to fire wins and
  the rest hear it and **suppress** — so K cache-holders of a name emit ~one Data,
  not K. This is the piece that makes caching/multicast scale.
- One broadcast Data retires **every** pending Interest for that name (soft-state
  PIT), so multiple askers are served by one frame regardless of their phases.

At N=2 there is one holder per name, so suppression is a no-op and the jittered guard
just replaces the fixed delay — **no regression**, and the mechanism is already the
N-node one.

### 3. Rendezvous parameters (SF / bandwidth / frequency)

SF, bandwidth, and carrier are **modulation-rendezvous** parameters: both ends must
match or they cannot decode (SF/BW are not in the LoRa header; frequency obviously
must match). Coding rate (explicit-header auto-detect), TX power, and FEC are
independently settable. The scalable rendezvous is **name-keyed + windowed**, not
pairwise:

- **Frequency:** already name-keyed in the example (`channel = f(H(name-or-pair))`);
  the N-node generalization is per-name FHSS with a shared hop clock (#40 `Key =
  H(data-name)` from the devourer HopSchedule) anchored to common-view time (#41).
- **SF/BW:** derive from *shared* observables (the measured link, the name), never a
  local-only signal — the bug this work already caught: bandwidth keyed off the
  synthetic RSSI proxy split one node to BW250 while the other held BW125. The robust
  N-node answer is **receiver-side SF auto-detect (ASFS-style CAD sweep)** so a node
  follows whatever SF a transmitter actually used (see the LoRa ADR literature note),
  which also reuses the CAD machinery from (1).

## Firmware changes (waveshare-lora-rs) — batch ALL of this into ONE reflash

Reflashing over ST-Link is the expensive, finicky step (the flash window in
`hardware-tools-runbook`), so everything worth having goes in one flash. Current
opcodes: `CMD_TX/SET_FREQ/SET_MOD/SET_PWR/SET_SYNC/GET_INFO/SET_BEACON` (+ `EVT_RX/
TXDONE/INFO/LOG`). The batched additions, by tier:

### Tier 1 — makes the MAC correct

- **LBT mode on `CMD_TX`** — a flag byte; the firmware runs the atomic `CAD → HW-RNG
  backoff → key-up` loop from §1 and replies `SENT` or `DEFERRED(attempts)`. This, not
  a host-polled CAD, is the carrier-sense primitive (atomicity note in §1).
- **SX1262 hardware RNG** (`GetRandomNumber`) as the backoff jitter source — the MCU
  has no RNG, and identical backoff on every node defeats de-synchronization.
- **`CMD_CAD` (0x08)** → `SetCad` + `GetIrqStatus` (`CadDetected`/`CadDone`) → **`EVT_CAD`
  (0x85) `[busy]`**; **`CMD_GET_RSSI` (0x09)** → `GetRssiInst` → instantaneous channel
  RSSI. These are for cognition-plane *sensing* (separate from access; serial latency
  is fine here).

### Tier 2 — so we never reflash *just to tune* (the #37 lesson: on-air calibration)

- **`CMD_SET_CAD_CFG`** — det-peak, det-min, symbol count (CAD sensitivity vs speed).
- **`CMD_SET_LBT_CFG`** — contention window `CW`, `MAX_BACKOFF`, `MAX_ATTEMPTS`.
- **`CMD_SET_PREAMBLE`** — preamble length (currently hardcoded 8; CAD detectability by
  *others* scales with it). All runtime, so tuning is a serial command, not a reflash.

### Tier 3 — unlock the bigger wins while we're in there

- **Hardware RX timestamp** — extend `EVT_RX` with an MCU-tick stamp of frame arrival.
  Needed for common-view timing (#41), precise reply/window scheduling, and to *measure*
  the TX→RX turnaround we currently guess at with the 400 ms `RX_GUARD`.
- **CAD-based SF-scan primitive** — **`CMD_SF_SCAN`** / an RX mode that sweeps SF7→SF12
  via CAD and reports the detected SF (**`EVT_SF_DETECTED`**). This is the firmware half
  of receiver-side SF auto-detect (ASFS), the real fix for the SF-rendezvous split (see
  the LoRa ADR literature note). Adding the primitive now means the SF fix later needs
  no reflash.

### Tier 4 — observability + a clean fix for a bug we already hit

- **CAD-busy / defer / occupancy counters in `EVT_INFO`** — you cannot tune CSMA blind
  (same as the EDCCA sweep in #37); occupancy also feeds cognition sensing (#30 cousin).
- **`EVT_TX_STARTED` + reported airtime** — fixes the SF12 `TX FAILED: no reply in 3s`
  cleanly: the host learns the real airtime and sets its timeout, instead of a fixed 3 s
  that a long high-SF frame blows.

**Integration constraints:** CAD/SF-scan must interleave with the existing RX listen
without dropping frames (the SX1262 returns to RX after CAD via the standard sequence);
the LBT loop must not starve USART draining (keep the task-#17 ISR ring pumping). Reflash
both dongles (already RDP-unlocked → no unlock step; readback-verify per the runbook).

## Host changes

- **`LoraSerialBackend`:** set the LBT flag on `inject`'s `CMD_TX` and parse `SENT`/
  `DEFERRED`; a `TxDiscipline` that advertises real carrier-sense so `set_edcca_ignore`
  stops being a no-op on this bearer. Plus `cad()` / `channel_rssi()` (sensing),
  `EVT_RX` timestamp surfaced on `CapturedFrame`, and the `CMD_SET_*_CFG` setters.
- **`lora_cognition.rs`:** (a) `inject` uses LBT-`CMD_TX` (access is now the firmware's
  job — the host no longer schedules around it); (b) replace the immediate/fixed-delay
  Data reply with the scheduled-reply queue + overhearing cancel (2); (c) drop the
  `name < peer` offset; (d) keep round-robin pacing and PIT aggregation.

## Increments

1. **Host suppression only** (no firmware): scheduled-reply queue + overhearing cancel,
   keep the offset for access. Additive, testable now, no regression. Proves the
   Data-dedup half and the async reply restructure.
2. **The one batched reflash** (all four tiers) + backend wiring: validate LBT-`CMD_TX`
   defers under a concurrent transmitter and sends when clear; CAD/RSSI sensing reads
   right; RX timestamps land.
3. **CSMA access**: switch `inject` to LBT-`CMD_TX`, drop the offset; validate N=2
   non-regression (≥96%), then N=3 with the Heltec.
4. **Receiver-side SF auto-detect** on the `CMD_SF_SCAN` primitive — dissolves the
   SF-rendezvous split (no further reflash, thanks to Tier 3).

## Validation

- **N=2 non-regression:** the two OPis must hold ≥ the current ~96% delivery with the
  offset removed and CSMA in — the bar this refactor must clear.
- **N=3:** add the Heltec (SX1276) as a third node. Two consumers asking the same
  producer's name must be served by **one** Data (suppression working), and three
  nodes must share the channel without the pairwise offset (CSMA working).
- **Instrument:** count CAD-defers, suppressed replies, and per-name responder
  multiplicity — silent success looks the same as "suppression never triggered."

## Risks / open questions

- **CAD false-negatives/positives** cost collisions or needless defers; the det-peak/
  det-min config needs on-air tuning (like the EDCCA thresholds in #37).
- **CAD latency vs duty:** each CAD is a few symbols of airtime-adjacent listening;
  at high SF it is not free. Budget it against the tick.
- **Reply-storm bound:** even with suppression, a popular name across many nodes needs
  the jitter window sized so the first reply is heard before the rest fire — window
  vs latency tradeoff.
- **SF-rendezvous under N nodes:** without ASFS, independent SF dialers can still
  split; CAD-CSMA does not fix that (it is access, not modulation agreement). ASFS or
  a fixed rendezvous control SF remains the real answer.

## Connections

#37 (EDCCA/carrier-sense on Wi-Fi — the sibling), #40 (name-keyed FHSS), #41
(common-view timing for hop/window anchoring), #30 (frame-free sensing — CAD is its
LoRa cousin), the LoRa ADR literature note (ASFS, ADR stability), and the named-radio
doctrine (receiver-agnostic, soft-state, no host identity).
