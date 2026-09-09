# NDR MAC — Concrete Specification (v0.2, working draft)

Named-Data Radio MAC. Supersedes the in-frame-filter direction (Blur + fingerprint are **dropped**;
see NAME_PARSE_FRAMEWORK.md for the measurements that justified it). This is the coherent whole from the
design dialogue: parse-for-relevance, a capability-spectrum interop model over a universal floor, a merged
rendezvous, scheduled sleep, and data-centric security with a provisioned trust anchor.

v0.2 changelog: closed the open ends — rendezvous is keyless (runs on the clear prefix); clock-absence =
floor; sleep phase = per-prefix `F.phase` default; worst-of merge bounded to ≤2 TX; `T_k` = one-way keyed
hash; off-host margin `H`=2; bootstrap = provisioned anchor (§9.1). Remaining TBD: TLV numbers +
rendezvous quantization constants (bookkeeping / on-air calibration).

---

## 0. Scope

How a node, per received frame, decides relevance / serve / forward efficiently across a **spectrum of
hardware** (LoRa MCU · 40 MHz AR9271 Xtensa · 240 MHz ESP32-C5 RISC-V · commodity monitor-mode host), over
a broadcast, **L2-identity-free**, mobile medium, presenting **one wireless face** to NFD. Confidentiality
and authorization are **data-centric** (NAC), rooted in a **provisioned trust anchor**; no manual/link/
channel keys.

---

## 1. Principles

### Hard invariants
- **H1 FN = 0.** Never drop a frame that was for you (capacity limits are not filter FNs).
- **H2 Universal interpretability.** Strip every optional field ⇒ a valid, parseable NDN frame remains;
  relevance is always answerable by parsing. No frame's meaning depends on an optional profile.
- **H3 Data-centric security only.** No manual/link/channel keys. Confidentiality/authorization is
  NAC-derived, auto-actuated, namespace-scoped, revocable by publishing, rooted in a provisioned anchor
  (§9). **L2 addressing stays identity-free** (ephemeral source, no peer tables); trust identity is an
  L3 construct — orthogonal layers.
- **H4 No abuse lever on sleep.** No frame/energy-triggered wake exists for a sleeping node (it wakes on
  its own clock). Corollary: open participation and drain-proof sleep are mutually exclusive.

### Actuation model
Precedence, **most-protective-wins** unless an explicit override says otherwise:
**trust schema (default, automatic) < policy (administrative) < manual (override).**
Applies to the name-opacity boundary, namespace access, reversibility, and any per-namespace knob.

### Design stance
Universal **floor** every node speaks + **capabilities discovered through interaction** + optimizations
**unlocked pairwise, degrading to the floor** + **multi-PHY nodes bridge** the bearer spectrum under one
face. No capability assumed; none mandated.

---

## 2. Architecture overview

```
  L3 (NDN):  names · signed Data · NAC (content keys, name-token keys, trust schema, provisioned anchor)
             ─────────────────────────────────────────────────────────────────────────
  NDR MAC:   relevance = PARSE (off-host if capable, else host)          §6
             name encoding = 3-zone opacity, per-namespace boundary       §4
             capability descriptor on Interest reverse-path               §5
             rendezvous  F(clear-prefix, epoch) → (channel, phase)        §7
             scheduled sleep + off-host serve                             §8
             physical cognition: worst-receiver rate · lease/slot · occupancy (existing)
             ─────────────────────────────────────────────────────────────────────────
  Bearer:    Wi-Fi (802.11) · LoRa · BLE · …  — one wireless face, multi-bearer nodes bridge
```

The MAC does not re-implement crypto: content encryption + name tokenization are NAC at L3; the MAC
carries and respects them (opaque names on the wire; serve/cache encrypted Data liberally).

---

## 3. The floor contract

Minimum **every** node speaks (commodity monitor-mode host, one channel, no off-host, no agility, no clock):
- **F1** a **common/default channel** per bearer (well-known).
- **F2** a **parseable NDNLPv2 frame** carrying Interest/Data, name per §4 (opaque components are ordinary
  `GenericNameComponent`s byte-wise — the fabric never needs the boundary). No filter, no fingerprint, no
  mandatory profile (H2).
- **F3** **host-side parse** is always sufficient to decide relevance/serve/forward.
- **F4** **always-listen** is always sufficient to participate (sleep is opt-in overlay).

Nothing at the floor needs µs timing, agility, off-host compute, or keys. First contact and the
least-capable live here; §5–§8 are overlay negotiated up from F1–F4.

---

## 4. Frame & name model

### 4.1 Frame
NDR payload = **NDNLPv2 LpPacket** (optional link fields, §5) wrapping **Interest (0x05)** / **Data (0x06)**,
under the bearer's framing (Wi-Fi: 802.11 data + LLC/SNAP ethertype `0x8624`, broadcast/ephemeral
addressing; LoRa/BLE native). Single-fragment fast path; NDNLPv2 fragmentation for larger.

### 4.2 Name: three-zone opacity
```
  [ clear routable prefix ) [ opaque semantic middle ) [ clear structural tail )
    [0, B)  routing/FIB        [B, M)  NAC-tokenized      [M, N)  segment/version/freshness
```
- **Clear prefix [0,B):** routable; relays LPM on it; always exposed (floor of the privacy dial).
- **Opaque middle [B,M):** each value = **`T_k(component)` — a deterministic, one-way keyed hash** (NAC
  name-token key). Wire type stays `GenericNameComponent` (fabric-agnostic). Both endpoints compute
  *forward*: consumer knows the real name → tokenizes → Interest; producer indexes content by the same
  tokens → matches → serves. Nothing recovers the name from the wire (stronger; no recovery path).
  Authorized nodes compute identical tokens ⇒ cache/PIT exact-match works within the authorized set;
  eavesdroppers see opaque bytes (meaning hidden) and can correlate access patterns (accepted leak).
  Token collisions ⇒ at most a wasted fetch, caught at the consumer's signature verify (H1 intact).
  *Reversible* component-encryption is an optional per-namespace mode (trust→policy→manual) only where an
  intermediary must recover names (rare).
- **Clear tail [M,N):** structural components (segment/version) the forwarder/segmenter needs, left clear.

`B,M` are per-namespace, **dynamic**, from a named signed **policy object** (data-centric), actuated
trust→policy→manual. **No boundary metadata on the wire** — endpoints derive `(B,M)` from the namespace
policy they hold; the fabric routes on whatever clear prefix hits its FIB. A node **without** the policy is
**route-only** (LPM the clear prefix; cannot — correctly — interpret the opaque middle).

### 4.3 The privacy ↔ routing dial
Shorter clear prefix ⇒ more protection, **coarser routing** (Interest routable only on a more-aggregated
prefix ⇒ reaches more of the namespace ⇒ more flooding). Longer ⇒ precise routing, more name exposed. Each
namespace sets its own point. Permanent limits: routable prefix always leaks; the boundary coarsely signals
"sensitive"; deterministic tokens leak access-correlation. Hiding the *namespace itself* is out of scope.

---

## 5. Capability descriptor & the per-link handshake

Dropping the in-frame filter shrank negotiation to **physical** concerns. Only **sender-relevant** receiver
capabilities are advertised; **node-local** ones (off-host parse, off-host cache/serve) are invisible — a
sender emits the same frame regardless.

### 5.1 Descriptor (NDNLPv2 link field `NdrCapability`, TLV# TBD, experimental range)
Carried on **Interests**, **hop-local** (each forwarder overwrites with its *own* capability — Data returns
to the immediate next hop). Sub-fields:

| field | meaning | drives |
|---|---|---|
| `MaxRate` | highest PHY rate reliably received | worst-receiver rate |
| `Hop` (flag) | channel-agile: can follow `F(prefix,epoch).channel` | hop overlay vs common channel |
| `Sleep` (opt) | duty-cycles; empty ⇒ per-prefix `F.phase`; params ⇒ consolidated node window | TX in the receiver's window; relay-hold |
| `Phys` | bearer bitset | bearer/relay selection |

Absent ⇒ floor (base rate, always-listen, no hop, this-bearer). **A clock is prerequisite for `Hop`/`Sleep`;
its absence simply shows as neither flag present — no separate bit needed.**

### 5.2 Reverse-path storage
Recorded in the **PIT in-record** (reverse path already tracks the incoming link). On Data return, consulted
to pick rate/channel/timing for the TX to that neighbor.

### 5.3 Offer / accept / degrade (observe, don't handshake)
- Capability is **read off Interests**, not negotiated. No beacons.
- Engage an optimization **iff both ends support it**; else **degrade to the floor**.
- **First contact / stale / absent ⇒ floor.** Soft-state; expires with the PIT entry; re-stamped by every
  Interest ⇒ mobility-safe.
- **Interest-forward leg** (rate/channel for an Interest a relay forwards upstream) uses the upstream's
  last-seen descriptor (bidirectional soft-state); absent ⇒ floor. The cheap leg; best-effort.
- **Worst-of merge for a Data satisfying multiple in-records:** `rate = min`; **phase is shared** (aligned
  by `F` — same name ⇒ same prefix+epoch ⇒ same window); **channel** = `{F.channel}` if all requesters hop,
  `{common}` if none, **both** if mixed. The forwarder (holding all in-records) replicates onto each distinct
  channel at the one shared phase ⇒ **≤ 2 transmissions**.

---

## 6. Relevance decision (node-local, no wire impact)
```
  if (our-firmware) and (T_parse(worst_name) ≤ T_gap(rate) / H):     # H = 2 default, tunable
      parse in radio → FIB/PIT/CS → forward / serve-from-cache / drop;  wake host only if it must consume/produce
  else:
      deliver to host → host parses and decides                       # commodity, or constrained-at-high-rate
```
Measured anchors: C5 2.2–10 µs (off-host any Wi-Fi rate); AR9271 @40 MHz 23–98 µs (off-host ≤6 M,
**host-fallback at MCS4+**); LoRa µs ≪ ms (off-host trivially); commodity host-only. Fallback costs the
optimization, never correctness (H1; over-capacity frames are missed by capacity, not false-negatived).

---

## 7. Rendezvous  `F(clear-prefix, epoch) → (channel, phase)`

One function, two projections, on the **common-view clock** (measured µs TSF: AR9271, mt76, MT7921AU, LoRa).
**Keyless and public — it runs on the *clear* prefix**, so relays compute it to forward (and eavesdroppers
can too — it reveals nothing the clear prefix didn't). **Coordination/spectral-reuse only, not privacy.**

- **channel** = `H(clear-prefix ∥ epoch)` mapped to the bearer's channel set ⇒ spectral reuse / contention relief.
- **phase** = mapped to the epoch's slots ⇒ the listen window for sleep.
- Deterministic ⇒ sender and receiver agree; **same name ⇒ same `(channel,phase)` ⇒ one TX reaches all
  co-requesters per channel.**

### Capability degeneration (this *is* the interop mechanism)
| receiver | channel | listens |
|---|---|---|
| clock + agile, awake | `F.channel` | always |
| clock + agile, sleeping | `F.channel` | at `F.phase` |
| clock, not agile, awake | common channel | always |
| clock, not agile, sleeping | common channel | at `F.phase` (time-only) |
| **no clock** (commodity) | **common channel** | **always (floor)** |

Sleep phase default = **per-prefix `F.phase`** (zero-config, auto-aligned; best for few-prefix nodes); a
many-prefix node advertises a **consolidated node window** via `Sleep` params. Constants (channel map, epoch
length, slots, guard) **TBD**, bounded by measured µs precision; `F` is one cheap hash/prefix (cached) —
affordable on the 40 MHz AR9271.

---

## 8. Sleep (optional, respected)
- **Self-clocked** (H4): radio *off* between `F.phase` windows ⇒ deaf ⇒ **no wake lever to abuse**.
- **Off-host serve:** in a window, an our-firmware node answers matching Interests from its **off-host
  mini-cache** (LoRa prototype CS_N=4) **without waking the host**; serve liberally (encryption enforces
  access; consumer verifies signature — poisoned cache caught at the consumer). Commodity: wakes host, no fallback.
- **Reaching a sleeper:** sender learns `Sleep` from the reverse-path, TXes in the window; a relay **holds
  the PIT entry** until the window (latency ≤ one window period — inherent, absorbed by PIT + re-expression).
- **Abuse bound:** flood a *known* window (waste that window's RX), never the sleep budget;
  **occupancy-driven window hopping** relocates a flooded window; node can cut it short. Keyless.

---

## 9. Security model (data-centric)
- **L2 open/cooperative; trust at L3** (signed Data, consumer-verified). No link/channel keys.
- **Name semantic-opacity** (§4.2): one-way keyed tokens, per-namespace boundary. Hides *meaning*, not
  location (deliberate). Fabric routes on the clear prefix.
- **Content confidentiality:** NAC — content key encrypted for authorized consumers, distributed as named
  Data, granted/revoked by publishing. MAC carries encrypted Data; serves/caches liberally.
- **One access grant** carries both the name-token key and the content key ("authorized for `/ns`" =
  form/read its names AND decrypt its content), revoked together (key rotation; revoked nodes keep only
  old-content keys, get no new ones).
- **Wake-drain:** eliminated by self-clocked scheduling (§8), keyless.
- **Serve/forward gate:** data-centric (have-it? / carry this prefix?), not an auth gate — encryption
  enforces. Abuse bounded by **airtime lease/slot budget + PIT aggregation + rate-limit on unmatched
  Interests**. Attackers can flood a *public* prefix but cannot forge *specific* opaque names.

### 9.1 Trust bootstrap = provisioned anchor
- **Provisioning (once):** node ← {**trust anchor** (root pubkey), **its identity key certified under the
  anchor**}. A trust root, not a passkey; the only manual step.
- **Runtime, private `/ns`:** node **fetches** named signed Data — the `/ns` trust schema and its NAC access
  grant (the `/ns` name-token + content keys, encrypted to the node's identity key, signed by the `/ns`
  authority) — and verifies both against the anchor. This *is* the "public control plane": ordinary Data
  fetching rooted in the anchor. No beacons, no handshake.
- **Cooperative baseline is anchor-free:** public content = clear names, no grants, no anchor.
- **Same-anchor fleet** ⇒ private interop (homogeneous+known optimal). **Different-anchor** ⇒ public-only
  unless an explicit **cross-anchor bridge** (published cross-signature) federates them.
- **Honest limits:** routable-prefix leak; boundary signals sensitivity; deterministic-token access
  correlation; commodity gets no location-privacy and is more jammable (single common channel).

---

## 10. Capability × mechanism (gradient, filled)
| node class | relevance | rendezvous | sleep | privacy |
|---|---|---|---|---|
| commodity monitor host, 1 ch, no clock | host parse | common channel (floor) | host-crude / none | opacity encoding only |
| our-fw constrained (AR9271) | off-host ≤6 M, host @MCS4+ | common ch unless wideband+clock | scheduled + off-host serve | opacity |
| our-fw capable (C5) | off-host always | `F.channel` (agile+clock) | scheduled + off-host serve | opacity + spectral reuse |
| LoRa MCU | off-host (µs≪ms) | dedicated/low-rate or common | scheduled (native duty-cycle) | opacity |
| multi-PHY node | per bearer | bridges bearers under one face | per bearer | opacity |

---

## 11. Open items
- **TLV assignments** for `NdrCapability` + sub-fields, and the opaque-component convention (experimental
  range; bookkeeping, not a design unknown).
- **Rendezvous constants** (§7): channel map, epoch length, slot count, guard — default them, on-air
  calibrate later (as the lease guard was), bounded by measured µs clock.
- **Documentation sweep:** the report/story artifacts and NAME_PARSE_FRAMEWORK still argue the in-frame
  filter; they must be reframed around parse-everywhere + host-fallback + this spec.
- (Bootstrap, off-host `H`, worst-of merge, sleep phase, token function — CLOSED in v0.2.)
```
