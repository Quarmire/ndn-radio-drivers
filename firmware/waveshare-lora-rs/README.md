# waveshare-lora-rs

Open **Rust** firmware for the Waveshare USB-TO-LoRa dongle (**GD32F103C8** + **SX1262** + CH343
USB-UART), replacing the closed factory firmware. The stock firmware wraps every LoRa frame in a
proprietary NETID/ADDR header, so it can only talk to its own kind; this firmware speaks **plain
standard LoRa**, so the dongle interoperates with any SX127x/SX126x peer (verified bidirectionally
against a Heltec WiFi-LoRa-32 SX1276 node) and is driven by the host as a clean serial modem. Since
7E-A5 v3 the modulation itself is a runtime knob: the same chip also brings up **(G)FSK**, selected
with `CMD_SET_PHY` — see [PHY selection](#phy-selection-cmd_set_phy).

## Hardware map

| Function | Pin | Notes |
|----------|-----|-------|
| Host UART | USART1 PA9 (TX) / PA10 (RX) | 115200 8N1 → CH343 → `/dev/ttyACM*` (Linux) / `/dev/cu.usbmodem*` (macOS) |
| SX1262 SPI | SPI2 — NSS PB12, SCK PB13, MISO PB14, MOSI PB15 | NSS driven by hand (BUSY handshake) |
| SX1262 RESET | PA4 | |
| SX1262 BUSY | PB1 | |
| SX1262 DIO1 | PB0 | RxDone/TxDone (polled) |
| RF switch | PB4 | **HIGH = RX, LOW = TX**; PB4 is JNTRST → JTAG disabled (SWD kept) |
| TCXO | DIO3, 1.7 V | non-`-B` variant; recalibrated after enable |
| LEDs | PA6 (RXD) / PA7 (TXD) | |

Regulator is **LDO** (not DC-DC); PA is an SX1262 (+22 dBm: paDutyCycle 0x04, hpMax 0x07).

## Host ⇄ firmware serial protocol (7E-A5 **v3**)

Binary framing: `7E A5 | type | len | payload[len] | xor-crc` (crc = XOR of type, len, payload).
The parser resyncs on `7E A5` and validates the crc, so a dropped byte costs at most one frame.
`len` is one byte, which is what bounds every payload below at 255 B.

**Every command is answered.** A recognised command replies with its event; anything else replies
`EVT_UNSUPPORTED (0x8F) [cmd, reason]`. Silence is never a valid answer — it costs the host its full
retry budget and then a failure for what is a one-frame reply.

### Host → firmware

| Type | Name | Payload | Reply |
|------|------|---------|-------|
| 0x01 | TX | LoRa frame bytes | TXDONE `[ok, 0]` |
| 0x02 | SET_FREQ | u32 BE Hz | INFO — **refused** (`UNSUPPORTED[.., 0x04]`) outside 902-928 MHz, the band this firmware image-calibrates for. ~83 ms inside the calibrated band; see [Command latency](#command-latency-and-the-78-ms-quantum) |
| 0x03 | SET_MOD | `[sf, bw_code, cr_code]` (bw 0x04=125k/0x05=250k/0x06=500k; cr 0x01..0x04 = 4/5..4/8) | INFO — sf **refused** outside 7..12. **LoRa only**: `UNSUPPORTED[0x03, 0x02]` in GFSK, which has no SF/BW/CR |
| 0x04 | SET_PWR | `[i8 dBm]` | INFO — clamped to −9..+22 dBm; INFO reports what was applied |
| 0x05 | SET_SYNC | `[sx127x sync byte]` — LoRa: 0x12 private / 0x34 public. GFSK: the byte replaces sync byte 0 (`RegSyncValue1`'s peer), bytes 1..3 keep `94 C1` | INFO |
| 0x06 | GET_INFO | *(empty)* | INFO |
| 0x07 | SET_BEACON | `[enabled]` or `[enabled, period_mult]` | INFO |
| 0x08 | CAD | *(empty)* | CAD `[busy]` — **LoRa only** (`UNSUPPORTED[0x08, 0x02]` in GFSK) |
| 0x09 | GET_RSSI | *(empty)* | RSSI `[rssi i16 BE]` |
| 0x0A | SET_CAD_CFG | `[sym, det_peak, det_min]` | INFO — **LoRa only** (`UNSUPPORTED[0x0A, 0x02]` in GFSK) |
| 0x0B | SET_LBT_CFG | `[cw_ms(2 BE), max_backoff, max_attempts]` | INFO |
| 0x0C | SET_PREAMBLE | `[preamble(2 BE)]` — **the unit is per-PHY**: LoRa *symbols*, GFSK *bytes* (×8 = the register's bits) | INFO |
| 0x0D | SF_SCAN | *(empty)* — sweep SF7..12 by CAD | SF_DETECTED `[sf | 0]` — **LoRa only** (`UNSUPPORTED[0x0D, 0x02]` in GFSK: no SF, no CAD) |
| 0x0E | TX_LBT | LoRa frame bytes; atomic CAD + backoff + key-up | TX_STARTED then TXDONE `[sent, attempts]` |
| 0x0F | SET_NAME_FILTER | `[u64 BE name-hash]*` (empty clears → pass-all) | INFO |
| 0x10 | SET_RELAY | `[u64 BE name-hash]*` | INFO |
| 0x11 | DATAPLANE | `[cs_serve, dedup, hop_on, hop_base_ch, hop_span]` | INFO |
| 0x12 | SET_SENSE_CFG | `[rssi_thresh i16 BE, cad_repeat]` | INFO |
| 0x13 | GET_STATS | *(empty)* | STATS (32 B) |
| 0x14 | RESET_STATS | *(empty)* | INFO |
| 0x15 | SET_DEBUG | `[on]` — EVT_LOG data-plane traces | INFO |
| 0x16 | ENTER_BOOTLOADER | `[0xB0, 0x07]` guard → GD32 ROM UART bootloader | *(none — the chip resets)* |
| 0x17 | READ_CLOCK | *(empty)* | CLOCK `[ticks u64 BE]`, µs |
| 0x18 | TX_AT | `[delay_us u32 BE][frame]`, frame ≤ 251 B | TXDONE **when the frame actually airs**, with the 8-byte scheduling tail. Its delay is counted from firmware *decode*, so the host transport lands in the placement — see [Scheduled TX](#scheduled-tx-cmd_tx_at-0x18-and-cmd_tx_at_abs-0x1f) |
| 0x1A | GET_CAP | *(empty)* | CAP (34 B) — **describes the PHY in effect** |
| 0x1B | SENSE | *(empty)* | SENSE `[activity u16 BE, rssi i16 BE]` |
| 0x1C | SET_RX_GAIN | **exactly one byte**: `0` = power-saving (the chip's power-on default), `1` = boosted (this firmware's default, ~+3 dB sensitivity for ~+2 mA in RX). Any other value is **refused** `UNSUPPORTED[0x1C, 0x04]` | INFO — *Waveshare-local, advertised via CAP's cmd_bitmap bit 28* |
| 0x1D | SET_PHY | `[packet_type]` in the **LR20xx wire numbering** — `0x0` LoRa, `0x2` FSK. Anything outside `phy_bitmap` is **refused** `UNSUPPORTED[0x1D, 0x04]` | **CAP (34 B) — the whole new record**, see [PHY selection](#phy-selection-cmd_set_phy). `PHY_ERR [phy, chip_status]` if the chip declines |
| 0x1E | SET_HOP | `[hop_ctrl][hop_period u16 BE][n][freq_hz u32 BE]*n` | **`UNSUPPORTED[0x1E, 0x02]` — always.** The SX126x has no intra-packet FHSS engine; see [No hopping](#no-hopping-cmd_set_hop-0x1e) |
| 0x1F | TX_AT_ABS | `[target_ticks u64 BE][frame]`, frame ≤ 247 B | TXDONE **when the frame actually airs**, with the same 8-byte scheduling tail. **Prefer this over 0x18** — see [Scheduled TX](#scheduled-tx-cmd_tx_at-0x18-and-cmd_tx_at_abs-0x1f) |

`UNSUPPORTED` reasons: `0x01` unknown opcode · `0x02` no such hardware/engine · `0x03` payload does
not satisfy the opcode's arguments · `0x04` argument outside the range CAP advertises.

### Firmware → host

| Type | Name | Payload |
|------|------|---------|
| 0x81 | RX | `[rssi i16 BE, snr i16 BE, ts_us u32 BE, frame bytes]` — **`ts_us` is MICROseconds** (see CAP `stamp_hz`); up to 247 frame bytes. **`snr` is 0 in GFSK**: that modem measures no signal-to-noise ratio, and 0 is the explicit "not measured" rather than a number derived from something else |
| 0x82 | TXDONE | `[ok, attempts]` (attempts = 0 for a plain TX). A **CMD_TX_AT** / **CMD_TX_AT_ABS** reply appends 8 more bytes: `[late_us u32 BE, keyup_us u32 BE]` — see [Scheduled TX](#scheduled-tx-cmd_tx_at-0x18-and-cmd_tx_at_abs-0x1f). Read byte 0 (and 1) and ignore the rest if you do not want it |
| 0x83 | INFO | `[status, sync(2), errors(2), freq(4), sf, bw, cr, pwr, lost(2), cad_busy(2), defer(2)]` — **19 B, frozen**: the host reads cad_busy/defer as the last 4 bytes, so nothing may be appended |
| 0x84 | LOG | ascii |
| 0x85 | CAD | `[busy(0/1)]` |
| 0x86 | RSSI | `[rssi i16 BE]` |
| 0x87 | SF_DETECTED | `[sf | 0 = none]` |
| 0x88 | TX_STARTED | `[airtime_ms u16 BE]` — emitted just before key-up |
| 0x89 | STATS | 32 B, see [STATS](#stats-0x89--32-bytes) |
| 0x8A | CLOCK | `[ticks u64 BE]` — the same counter EVT_RX stamps with, at full width |
| 0x8B | CAP | **34 B**, see below. Also arrives **unsolicited** when the measured `sched_gran_ns` moves — see [Re-published capabilities](#re-published-capabilities) |
| 0x8C | SENSE | `[activity u16 BE, rssi i16 BE]` |
| 0x8D | PHY_ERR | `[requested_phy, chip_status]` — a PHY this node **advertises** that the chip refused at runtime, carrying the SX126x's literal `GetStatus` byte (chip mode in bits [6:4], command status in [3:1]) |
| 0x8F | UNSUPPORTED | `[cmd, reason]` |

### STATS (0x89) — 32 bytes

`CMD_GET_STATS` (0x13) always replies with **exactly 32 payload bytes**. Every field is **unsigned
and big-endian**; the offsets below are half-open byte ranges into the payload (the byte after the
framing's `len`, i.e. payload byte 0 is wire byte 4). They are transcribed from the `stats` module in
`src/main.rs`, which the emitter indexes with and whose contiguity is asserted at compile time — the
table and the wire cannot drift.

Bytes `[0..24]` are byte-identical to the v1 layout, and the existing host parser length-checks
`< 24` rather than `!= 24`, so a v1 host reads exactly the 24 bytes it always did and ignores the
tail.

| Offset | Width | Field | Meaning | Cleared by `RESET_STATS` (0x14) |
|--------|-------|-------|---------|------------------------------|
| `[0..4]`   | u32 BE | `rx`           | frames the on-device data plane classified | yes |
| `[4..8]`   | u32 BE | `filtered`     | dropped: no installed prefix covers the name | yes |
| `[8..12]`  | u32 BE | `deduped`      | dropped: duplicate Data object | yes |
| `[12..16]` | u32 BE | `served`       | answered from the on-device Content Store | yes |
| `[16..20]` | u32 BE | `relayed`      | re-broadcast by the relay set | yes |
| `[20..22]` | u16 BE | `cad_busy`     | channel sensed busy — the **resettable view**; `RESET_STATS` moves a baseline forward while the free-running counter EVT_SENSE reports keeps advancing, so the two can never disagree about direction | yes (re-baselined) |
| `[22..24]` | u16 BE | `defer`        | transmissions abandoned after the LBT backoff budget | yes |
| *— end of the v1 payload; a v1 host stops here —* | | | | |
| `[24..26]` | u16 BE | `chip_rx`      | SX126x `GetStats` `nbPktReceived` | yes (`ResetStats` is issued to the chip) |
| `[26..28]` | u16 BE | `chip_crc_err` | SX126x `GetStats` `nbPktCrcError` — **the only place a failed decode is visible**: `poll_rx` drops a CRC failure and returns nothing, so without this a quiet channel and a channel we are failing to decode look identical | yes (″) |
| `[28..30]` | u16 BE | `chip_hdr_err` | SX126x `GetStats` third counter — `nbPktHeaderErr` in LoRa, **`nbPktLengthError` in GFSK** (same register, different meaning per PHY) | yes (″) |
| `[30..32]` | u16 BE | `rx_trunc`     | frames whose on-air length exceeded `max_payload` (247) and were therefore truncated. **Should stay 0**; non-zero means a peer is transmitting past our advertised cap | yes |

Notes for a reader:

* the three `chip_*` fields are the SX126x's own 16-bit counters, so they are narrower than the
  firmware counters above them and are also zeroed by any hard reset of the chip;
* `cad_busy` here is *not* the same number as `EVT_SENSE.activity`, which is the free-running
  counter. Difference `EVT_SENSE.activity` over a window for occupancy; read this one for
  "since the last reset".

**CAP (0x8B), 34 bytes, every multi-byte field big-endian.** The one place this node describes
itself; every field is a source constant or a verified property, and where nothing is known the
field is `0`.

★ **CAP describes the PHY in effect, not the node.** `max_payload`, `sf_min`/`sf_max` and
`cmd_bitmap` all move with `phy_current` — the same SX1262 in GFSK has no spreading factor and no
CAD. That is why `SET_PHY` answers with the *whole* record and a host **replaces its profile
wholesale**; nothing here is safe to patch field by field.

Freshly booted (LoRa, before the first scheduled transmission refines `sched_gran_ns`):

```
7E A5 8B 22  03 00 35 C3 6D 80 37 50 28 00 F7 16 00 0F 42 40 02 00 F7
             BD FF FF FE 07 0C 00 06 1A 80 00 00 00 05 00  36
```

and after `SET_PHY [0x02]` (GFSK) — note `cmd_bitmap`, `sf_min`, `sf_max` and `phy_current` all move:

```
7E A5 8B 22  03 00 35 C3 6D 80 37 50 28 00 F7 16 00 0F 42 40 02 00 F7
             BD FF DA F6 00 00 00 06 1A 80 00 00 00 05 02  12
```

| Bytes | Field | LoRa | GFSK | Why |
|-------|-------|------|------|-----|
| 0 | proto_ver | 3 | 3 | v3 |
| 1 | radio_kind | 0 | 0 | **the PART** — 0 = SX1262, 1 = SX1276, 2 = LR2021. v2 read this as a part-and-mode pair; the mode is now `phy_current` |
| 2..6 | freq_min_hz | 902 000 000 | ″ | the band `set_frequency` image-calibrates for, **not** the SX1262's 150-960 MHz silicon range. `CalibrateImage` takes a *band* and is packet-type independent, so the two PHYs genuinely share it |
| 6..10 | freq_max_hz | 928 000 000 | ″ | ″ |
| 10 | pwr_min_dbm | −9 (0xF7) | ″ | real dBm — the SetTxParams range `set_power` clamps to; the PA is not per-PHY |
| 11 | pwr_max_dbm | +22 (0x16) | ″ | ″ |
| 12..16 | stamp_hz | 1 000 000 | ″ | **microseconds** — `micros()` adds SysTick sub-ms cycles (RELOAD 7999 @ 8 MHz) to `ms×1000` |
| 16 | stamp_kind | 2 | ″ | software counter — the MCU reads its own clock on noticing RxDone in the poll loop; there is no hardware capture |
| 17..19 | max_payload | 247 | 247 | **the same number for the same reason in both**: the binding limit is the *serial framing* (EVT_RX = 8 header bytes + frame ≤ 255), not the radio. Both radio caps are larger (LoRa PDU 255, GFSK PDU 255) and therefore not binding |
| 19..23 | cmd_bitmap | 0xBDFF_FFFE | 0xBDFF_DAF6 | 29 opcodes in LoRa; GFSK clears the four LoRa-modem ones — `SET_MOD` (0x03), `CAD` (0x08), `SET_CAD_CFG` (0x0A), `SF_SCAN` (0x0D). Clear in both: bit 0 (no opcode 0), bit 25 (0x19 unassigned), bit 30 (0x1E `SET_HOP`, understood and refused). The build asserts the LoRa bitmap equals the OR of every opcode `handle_cmd` dispatches, so the bitmap **is** the dispatcher rather than a claim about it |
| 23 | sf_min | 7 | **0** | SF5/6 are chip-supported but do not interop with the fleet's SX127x peers. 0/0 in GFSK is "this PHY has no such knob", not "unknown" |
| 24 | sf_max | 12 | **0** | ″ |
| 25..29 | sched_gran_ns | 400 000 | ″ | the **absolute** path's granularity (`TX_AT_ABS`, 0x1F). Derived term by term under [Scheduled TX](#scheduled-tx-cmd_tx_at-0x18-and-cmd_tx_at_abs-0x1f) and **raised to the measured key-up** once this node has released a scheduled frame — it can get more conservative from evidence, never more optimistic |
| 29..33 | **phy_bitmap** | 0x0000_0005 | ″ | *v3 tail.* Bit N ⇔ wire `SetPacketType` value N is usable: bit 0 LoRa + bit 2 FSK |
| 33 | **phy_current** | 0x00 | 0x02 | *v3 tail.* The wire `SetPacketType` value in effect now |

**Back-compat, both directions.** Bytes `[0..29]` keep their v2 positions and widths, and for this
node `radio_kind` keeps its v2 *value* too (0 = SX1262 was already the part) — so a v2 host reads a
v3 node correctly and ignores the 5-byte tail. A v3 host reading a v2 node (29 bytes, `proto_ver` 2)
should synthesise a single-entry `phy_bitmap` from the v2 `radio_kind` and set `phy_current` to it.

## PHY selection (`CMD_SET_PHY`)

**Modulation is a knob, not an identity.** `SetPacketType` is a runtime command on every part in this
fleet: this SX1262 does LoRa **and** (G)FSK, the SX1276 adds OOK, the LR2021 a dozen more. A
node that happens to boot into one of them is not a different radio from the same node in another,
which is why `radio_kind` now names the *part* and the mode travels in `phy_current`.

### The two numberings, and why they are mapped exactly once

The wire uses the **LR20xx** `SetPacketType` values. The SX126x has its own, and **they disagree on
both modes this part has**:

| PHY | wire (`phy_bitmap` bit, `phy_current`) | SX126x `SetPacketType` (0x8A) |
|-----|---------------------------------------|-------------------------------|
| LoRa | **0x0** | **0x01** |
| (G)FSK | **0x2** | **0x00** |

Note what a missed translation would do: passing the wire value straight through selects GFSK when
the host asked for LoRa, *and the chip accepts it*. The failure is completely silent and surfaces
only as an air link that never forms. So the mapping lives in one function at the protocol boundary
(`wire_to_chip_phy` in `src/main.rs`) and is pinned by a compile-time table that asserts both rows,
the round trip, the fact that the numberings differ at all, and that **every other value in the
LR20xx table maps to nothing**.

`phy_bitmap` = **0x0000_0005** — LoRa and FSK, the two this firmware brings up and receives on.

Not advertised, and why: **LR-FHSS** (0x03 in the SX1262's numbering) is real silicon on this part
but **transmit-only**, and it builds its own hop sequence inside the packet rather than taking one —
a mode that cannot receive is not a PHY a bidirectional bearer can offer. Everything else in the
LR20xx table (BLE, RTToF, FLRC, BPSK, WM-BUS, Wi-SUN, OOK, Raw, Z-Wave, O-QPSK) is simply not in
this chip.

### What moves when the PHY moves

| | LoRa | GFSK |
|--|------|------|
| modulation knob | `SET_MOD [sf, bw, cr]`, SF 7..12 | **none** — `SET_MOD` is refused `NO_HARDWARE` |
| carrier sense | `SetCad` (preamble correlation), plus optional RSSI energy detect | **energy detect only** — `SetCad` is a LoRa-modem function |
| airtime model | symbol arithmetic over SF/BW/CR (`airtime_ms`) | bit count over a fixed bitrate (`gfsk_airtime_ms`) |
| 32-byte frame | ≈ 72 ms at SF7/BW125/CR4-5 | ≈ 8 ms |
| RX metadata | RSSI **and** SNR | RSSI only; `EVT_RX.snr` = 0 |
| sync word | 1 byte, SX127x convention, reg 0x0740 | 24 bits `C1 94 C1`, reg 0x06C0; `SET_SYNC` replaces byte 0 |
| preamble unit | symbols | bytes (×8 into the 16-bit bit-count register) |

**The GFSK profile is one fixed operating point**: 50 kbps, ±25 kHz deviation, 117.3 kHz RX
bandwidth, BT 0.5 shaping, 24-bit sync, whitening on, 2-byte inverted CRC (poly 0x1021, seed
0x1D0F) — the LoRaWAN FSK point. There is deliberately **no** GFSK modulation knob: `SET_MOD` carries
`[sf, bw, cr]`, GFSK has none of the three, and reinterpreting three bytes of a fleet-wide opcode
into a node-local meaning is how a shared contract stops being shared. A GFSK bitrate/deviation knob
needs its own opcode assignment.

**Carrier sense in GFSK is real but different.** With no CAD, the energy detector carries the whole
sense — so the threshold stops being optional there. If the host has not set one
(`SET_SENSE_CFG`, 0x12), the firmware uses **−95 dBm** rather than leaving the sense structurally
incapable of ever returning busy; a host-set threshold always wins. `CMD_SENSE`, `TX_LBT` and the
`activity` counter therefore keep working and keep meaning what they say.

⚠ **GFSK is implemented and advertised; it has not yet been on air.** The bring-up is a complete
configuration sequence from the SX126x datasheet — packet type, modulation, packet params, sync,
CRC seed/polynomial, whitening — and every consequence of the switch is wired through (airtime
model, packet-status decode, sensing path, `sf 0/0`, the per-PHY `cmd_bitmap`), but this firmware has
not been flashed since the change, so no GFSK frame has been transmitted or received. What makes the
advertisement safe rather than a claim is that the node **checks and reports**: the chip's `GetStatus`
verdict on `SetPacketType` is read at the switch and at boot (the boot `EVT_LOG` carries
`physt=`/`ok=`), and a refusal comes back as `EVT_PHY_ERR` with that literal byte instead of a link
that silently never forms. First flash: switch to `0x02`, confirm `phy_current` = 2 in the CAP reply,
and run a frame each way.

**A queued scheduled frame is cancelled by a PHY switch**, and told so: it was accepted under the old
medium, its airtime was computed there, and the host holds a re-based deadline from it — so it is
closed with `TXDONE [0, 0]` (`ok = 0` is exactly true: it did not air) before the switch proceeds.

**If the chip declines**, the node says so with `EVT_PHY_ERR [requested_phy, chip_status]`, carrying
the SX126x's literal `GetStatus` byte read immediately after `SetPacketType` (command status 0x3/0x4/
0x5 = timeout / processing error / execution failure), and rolls the radio back to the PHY it was in.
An advertised PHY that the silicon refuses at runtime is exactly the case this event exists for.

## No hopping (`CMD_SET_HOP`, 0x1E)

Always `UNSUPPORTED[0x1E, 0x02]` (`NO_HARDWARE`), from its own dispatcher arm rather than the
catch-all, so the exclusion is legible rather than accidental.

The SX127x has `RegHopPeriod` and an `FhssChangeChannel` interrupt that steps a host-supplied table
**mid-frame**. The SX126x has no such engine: its only hopping is inside the LR-FHSS packet type,
which is transmit-only and builds its own sequence. This node is therefore *structurally* outside the
intra-packet hopping pair, and bit 30 of `cmd_bitmap` stays clear — the bitmap means "implemented and
will act", and this will not.

(`DATAPLANE`'s `hop_*` fields, 0x11, are a different mechanism: a name-keyed choice of which channel
to sit on *between* frames. That needs no hardware sequencer and is unaffected.)

Defaults: **PHY = LoRa** (wire 0x0) / 915 MHz (US) / SF7 / BW125 / CR4-5 / sync 0x12 (→ SX126x reg
0x1424) / preamble 8 / explicit header / CRC on / **boosted LNA gain** (reg 0x08AC = 0x94; the chip's
power-on default is the power-saving 0x96, which is the wrong default on a bearer that exists for
reach) — matched to the Heltec node. Nothing about the PHY switch is persistent: a reset comes back
in LoRa.

## Scheduled TX (`CMD_TX_AT` 0x18 and `CMD_TX_AT_ABS` 0x1F)

Two entry points into the same release mechanism, differing only in how the deadline is named:

| | `CMD_TX_AT` (0x18) | `CMD_TX_AT_ABS` (0x1F) |
|--|-------------------|------------------------|
| payload | `[delay_us u32 BE][frame]` | `[target_ticks u64 BE][frame]` |
| frame ≤ | 251 B (255 − 4) | **247 B** (255 − 8) |
| deadline | `delay_us` after the firmware **decodes** the command | the named `micros64()` instant |
| bound | delay ≤ 60 s, else `UNSUPPORTED[.., 0x04]` | \|target − now\| ≤ 60 s **in either direction**, else `UNSUPPORTED[.., 0x04]` |
| placement | + host transport (see below) | node-internal only |

Both are on the **`micros64()` timebase** — the same counter `EVT_RX.ts_us` stamps with and
`CMD_READ_CLOCK` (0x17) returns at full 64-bit width — so a host converts freely between "when it
arrived" and "when to send" without reconciling two clocks. Both reply with a single `EVT_TXDONE`
emitted **when the frame actually goes**, never at arm time.

* `delay_us = 0`, or a target slightly in the past, means *now*: it transmits immediately and the
  lateness is measured and reported in `late_us` rather than hidden.
* A target more than 60 s in the **past** is refused, not fired. The only way to be a minute behind a
  monotonic counter the host just read is to be converting from a different timebase, and firing
  anyway would hide that behind a frame that looks like it worked.
* No LBT: a backoff would move the transmission off the instant that was asked for, and a scheduled
  TX exists precisely because the caller has already decided when the air is theirs.
* The queue is **one deep**, shared by both opcodes. A second arm while one is pending is refused
  with `TXDONE [0, 0]` (`ok = 0` is exactly true — that frame did not air) rather than displacing a
  frame the host is already waiting on. A `SET_PHY` cancels it the same way.

**Why an SX1262 can do this at all.** The chip has no delayed key-up engine; `SetTx` starts the
transmitter now. But nothing reaches the air here except through the MCU, so the **MCU is the
transmit queue**, and a GD32 TIM2 compare that releases `SetTx` at a deadline is a real scheduled
transmission. v2 answered `UNSUPPORTED[NO_HARDWARE]`, which was honest about the radio and wrong
about the node.

**How it stays out of the host link's way.** USART1 has no FIFO; the ISR rescues each byte into a
512-byte ring, which holds ~44 ms of continuous 115200 traffic. So the scheduler is a state machine
the main loop *services*, never a delay it *waits out*:

| State | Radio | Main loop |
|-------|-------|-----------|
| `Armed` | in RX, receiving and delivering normally | drains the host link as usual; nothing is blocked |
| → stage at `deadline − lead` | leaves RX for **STDBY_XOSC** (not STDBY_RC — see below), payload + packet params written | one ~2.7 ms SPI burst |
| `Staged` | idle in STDBY_XOSC, payload loaded | drains until the deadline is within 4 ms, then hands that last stretch to TIM2 |
| fire | `SetTx` — one 4-byte SPI transaction | the only non-draining stretch, ≤ 4 ms of the ring's 44 ms (asserted at compile time) |

The staging lead starts at 8 ms and is **raised in flight** if a staging pass ever measures longer,
so an assumption that turns out wrong costs one late frame, not a broken feature.

### `sched_gran_ns` = 400 000 — and which opcode it describes

The contract: *`SetTx` is issued at the requested instant, and the first symbol leaves within
`sched_gran_ns` of it.* The fixed part is deliberately **not** compensated away — a compensation
derived from an unmeasured latency is a bias dressed as precision.

| Term | µs | Where it comes from |
|------|----|---------------------|
| TIM2 tick | 1 | timer runs at 1 MHz (PSC = `pclk1_tim`/1 MHz − 1 = 7 at the 8 MHz HSI clock; the prescaler actually programmed is printed in the boot `EVT_LOG` as `tim2psc=`) |
| deadline quantization | 1 | `micros64()` reports whole microseconds (SysTick CVR ÷ 8) |
| UIF poll loop | 2 | read TIM2.SR + test + branch ≈ 12 cycles at 8 MHz = 1.5 µs, rounded up |
| `SetTx` SPI transaction | 50 | 4 bytes at SCK = 1 MHz = 8 µs/byte = 32 µs, plus the two NSS edges and the call boundary, rounded up |
| `SetTx` → transmitter | 100 | BUSY-high while the chip processes the command out of STDBY_XOSC. Not stated crisply in the datasheet for a TCXO part, so rounded **up** generously — and it is the one term measured at runtime |
| PA ramp | 200 | `SetTxParams` ramp code 0x04 = SET_RAMP_200U |
| **total** | **354** | rounded **up** to **400 µs = 400 000 ns** |

Every term in that table is **internal** — a timer tick, an SPI transaction, a PA ramp. Nothing in it
involves the host. So it is the whole granularity of `TX_AT_ABS` (0x1F), whose deadline is an instant
the host names outright, and it is what CAP publishes.

`TX_AT` (0x18) releases just as precisely but against a different reference: its delay starts when
the **firmware decodes** the arm, so the host→device transport sits between "when the host meant" and
"when the node starts counting". The node cannot measure that term — it never sees the host's clock —
so it is bounded from the closest thing this node *has* measured, its own command round trips:
`GET_INFO` here has a 4 758 µs mean floor with a **sub-millisecond spread** (2026-08-28, n = 8..10),
and it is the spread that lands in the placement (a constant offset a host can calibrate away, a
varying one it cannot).

```
TX_AT_ABS (0x1F)   400 000 ns                                  <- sched_gran_ns
TX_AT     (0x18)   400 000 ns + up to ~1 000 000 ns of host transport
```

The same effect was **measured end-to-end on the LR2021**, which declares a 50 µs granularity and has
a 550 µs `GET_INFO` round-trip spread: an absolute-boundary slot train fired 45/45 with a mean gap
2 399 818 ticks against 2 400 000 nominal — 11 µs of drift over 44 slots, so the *accuracy* is
excellent — while the jitter came out at **sd 553 µs / p2p 1875 µs**, which is the round-trip number,
not the release mechanism. Host-armed relative scheduling was there *worse* than that node's software
path (sd 553 vs 155 µs) purely because it pays an extra round trip. **Prefer 0x1F**; 0x18 remains for
hosts that have not moved.

**And then it is measured.** Every scheduled transmission times BUSY across `SetTx` on the same
microsecond counter and reports `keyup_us` in the `EVT_TXDONE` tail, alongside `late_us` (how far
past the requested instant `SetTx` was actually issued). If the measured key-up ever exceeds the
derived figure, `EVT_CAP.sched_gran_ns` rises to it. A capability can get more conservative from
evidence; it never gets more optimistic from it.

### Re-published capabilities

That refinement used to be **unreachable**: the host reads CAP once at open and never asks again, so
a node that measured a 2 ms key-up went on being planned against the derived 400 µs forever.

So the node now emits an **unsolicited `EVT_CAP`** when the live `sched_gran_ns` differs from the
last published figure by more than **25 %**. It is an ordinary event frame — a host that keeps
listening simply learns the better number and replaces its profile.

The trigger is *self-limiting*, not merely rate-limited: `sched_gran_ns` is the maximum of a constant
and a running worst case, so it never decreases, and each re-publish must clear 1.25× the last one.
From 400 µs to the highest value a stuck-BUSY measurement can reach (~250 ms) is a factor of 626,
i.e. at most **⌈log₁.₂₅ 626⌉ = 29 events in the lifetime of a node**, however many frames it
schedules. A 5-second floor between emissions is belt-and-braces on top of that, so a burst of
scheduled frames can never carry a burst of CAPs. `GET_CAP` also updates the baseline, so the
comparison is always against what the host actually holds.

⚠ **Not yet confirmed on air.** The mechanism is implemented and the arithmetic is sourced, but this
firmware has not been flashed since the change, so `keyup_us`/`late_us` have no measured values yet
on *this* node. The first scheduled frame after a flash reports both, and that is the number to trust
over the 400 µs derivation. The relative-vs-absolute gap, by contrast, **is** measured — on the
LR2021, above.

## Command latency, and the 78 ms quantum

Measured 2026-08-28, host command → `EVT_INFO`, n = 8..10 each, alternating values so nothing is a
no-op:

| Command | o5p-0 | mds-05 |
|---------|-------|--------|
| SET_FREQ (retune) | 160 866 µs | 160 150 µs |
| SET_MOD | 82 778 µs | |
| SET_PWR | 82 679 µs | |
| GET_INFO (the serial floor) | 4 758 µs | |

Subtracting these against each other lands three separate times on the same number:

```
SET_PWR  − GET_INFO floor  =  82.68 − 4.76  = 77.92 ms
SET_MOD  − GET_INFO floor  =  82.78 − 4.76  = 78.02 ms
SET_FREQ − SET_PWR         = 160.87 − 82.68 = 78.19 ms
DIO3 TCXO startup timeout  =  5000 × 15.625 µs = 78.125 ms
```

All three within 0.3 %. So the cost is **not** SPI, not the radio, and — for the retune — **not the
image calibration's compute time**: it is the TCXO startup that `SetStandby(STDBY_RC)` makes
necessary every time the firmware re-enters a mode that needs the crystal. A retune paid it twice,
because `CalibrateImage` needs the crystal too and `standby()` had just stopped it.

**What changed.** `CalibrateImage` takes a frequency **band**, not a point, and `CMD_SET_FREQ`
refuses everything outside 902-928 MHz — so the calibration `init` already ran covers every retune
the host can legally request. `Sx1262` now remembers the band it has loaded and skips the
recalibration while the target stays inside it (a target outside the loaded band still recalibrates,
so widening the band later stays correct).

Expected result: a retune becomes the same shape of operation as `SET_PWR`/`SET_MOD` — standby, one
register write, re-arm RX — i.e. **~83 ms**, one remaining TCXO startup plus the serial floor.
That is a *prediction* from the arithmetic above and **must be re-measured before the host's
`retune_us` is moved off 161 ms**.

**The 78 ms that is left, and how to kill it.** Every `SET_*` still pays one TCXO startup because
`standby()` uses STDBY_RC (0x00), which powers the crystal down, and the following `start_rx()` has
to bring it back. Using **STDBY_XOSC (0x01)** for parameter changes — entered straight out of RX,
where the crystal is already running — should remove it and take every knob from ~83 ms to ~5 ms.
That change is *not* made here: it is one line, it cannot be verified without hardware, and it would
alter the one path that is known to work. It is also self-validating when someone does flash it — the
chip raises `XOSC_START_ERR` in `GetDeviceErrors`, which `EVT_INFO` already carries in bytes 3..5, so
a bad shortcut shows up as a non-zero `errors` field rather than as silent mistuning. The scheduled-TX
path already uses STDBY_XOSC and measures the result, so it will be the first evidence either way.

## On-device NDN data plane

The dongle classifies each received frame **by name** before deciding whether to wake the host:
prefix filter, duplicate suppression, Content-Store serve, and relay (`src/ndn.rs`, shared by
`#[path]` with the LR2021 firmware). Everything defaults inert, so a freshly-flashed dongle behaves
exactly like the plain modem until the host installs routes.

It parses the **real wire** — the optional body-prefix GCS TLV (`0xF5`), the NDNLPv2 `LpPacket`
(`0x64` → `0x50` Fragment, skipping continuation fragments by `0x52` FragIndex), and the
Interest/Data (`0x05`/`0x06`) `0x07` Name inside — and renders the name to the `/`-joined form
(`/ndn/lora-cog/A/alarm/6`) that the host's `ndn_name_to_slash` produces, so `fnv1a64` over it lands
in the same #44 keyspace `set_name_filter` installs. **Host-visible consequence:** prefixes must now
be installed in `/`-leading form (`/ndn/lora-cog/A`), because that is what the wire renders to.

Until 2026-08-28 it recognised only the ASCII demo wire `KIND|SRC|SF|NAME|…`, so every offload path
was inert on the real face. That wire still works, behind an explicit second branch, for the older
examples.

## Build & flash

```sh
cargo build --release
# NOTE: CARGO_TARGET_DIR is redirected workspace-wide, so ask cargo where the artifact went rather
# than assuming ./target — `cargo metadata --no-deps --format-version 1 | jq -r .target_directory`
# currently resolves to ../../../target-shared.
OUT=$(cargo metadata --no-deps --format-version 1 | jq -r .target_directory)
arm-none-eabi-objcopy -O binary "$OUT/thumbv7m-none-eabi/release/waveshare-lora-rs" firmware.bin
```

Two flash paths: **ST-Link/openocd** (below — needed once per dongle) and, on firmware that already
carries `CMD_ENTER_BOOTLOADER` (0x16), **`./reflash-over-usb.sh <tty> firmware.bin <lora_dfu>`** over
the same CH343 link, with no ST-Link and no replug.

Flash over **ST-Link + openocd** (the chip ships RDP read-protected; the first `stm32f1x unlock 0`
mass-erases the stock firmware). No nRST is broken out, so openocd only attaches in the brief window
right after the ST-Link USB re-enumerates — build first, replug, then flash immediately:

```sh
openocd -f interface/stlink.cfg -f target/stm32f1x.cfg \
  -c "init; reset halt; stm32f1x unlock 0; reset halt; \
      flash write_image erase firmware.bin 0x08000000; verify_image firmware.bin 0x08000000; \
      reset run; exit"
```

JTAG is disabled in software but **SWD is preserved**, so the dongle stays reflashable.
