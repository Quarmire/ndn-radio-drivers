# waveshare-lora-rs

Open **Rust** firmware for the Waveshare USB-TO-LoRa dongle (**GD32F103C8** + **SX1262** + CH343
USB-UART), replacing the closed factory firmware. The stock firmware wraps every LoRa frame in a
proprietary NETID/ADDR header, so it can only talk to its own kind; this firmware speaks **plain
standard LoRa**, so the dongle interoperates with any SX127x/SX126x peer (verified bidirectionally
against a Heltec WiFi-LoRa-32 SX1276 node) and is driven by the host as a clean serial modem.

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

## Host ⇄ firmware serial protocol (7E-A5 **v2**)

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
| 0x02 | SET_FREQ | u32 BE Hz | INFO — **refused** (`UNSUPPORTED[.., 0x04]`) outside 902-928 MHz, the band this firmware image-calibrates for |
| 0x03 | SET_MOD | `[sf, bw_code, cr_code]` (bw 0x04=125k/0x05=250k/0x06=500k; cr 0x01..0x04 = 4/5..4/8) | INFO — sf **refused** outside 7..12 |
| 0x04 | SET_PWR | `[i8 dBm]` | INFO — clamped to −9..+22 dBm; INFO reports what was applied |
| 0x05 | SET_SYNC | `[sx127x sync byte]` (0x12 private / 0x34 public) | INFO |
| 0x06 | GET_INFO | *(empty)* | INFO |
| 0x07 | SET_BEACON | `[enabled]` or `[enabled, period_mult]` | INFO |
| 0x08 | CAD | *(empty)* | CAD `[busy]` |
| 0x09 | GET_RSSI | *(empty)* | RSSI `[rssi i16 BE]` |
| 0x0A | SET_CAD_CFG | `[sym, det_peak, det_min]` | INFO |
| 0x0B | SET_LBT_CFG | `[cw_ms(2 BE), max_backoff, max_attempts]` | INFO |
| 0x0C | SET_PREAMBLE | `[preamble(2 BE)]` | INFO |
| 0x0D | SF_SCAN | *(empty)* — sweep SF7..12 by CAD | SF_DETECTED `[sf | 0]` |
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
| 0x18 | TX_AT | `[delay_us u32 BE][frame]` | **UNSUPPORTED `[0x18, 0x02]`** — no scheduled-TX engine here |
| 0x1A | GET_CAP | *(empty)* | CAP (29 B) |
| 0x1B | SENSE | *(empty)* | SENSE `[activity u16 BE, rssi i16 BE]` |
| 0x1C | SET_RX_GAIN | `[0 = power-saving | 1 = boosted]` | INFO — *Waveshare-local, advertised via CAP's cmd_bitmap* |

`UNSUPPORTED` reasons: `0x01` unknown opcode · `0x02` no such hardware/engine · `0x03` payload does
not satisfy the opcode's arguments · `0x04` argument outside the range CAP advertises.

### Firmware → host

| Type | Name | Payload |
|------|------|---------|
| 0x81 | RX | `[rssi i16 BE, snr i16 BE, ts_us u32 BE, LoRa bytes]` — **`ts_us` is MICROseconds** (see CAP `stamp_hz`); up to 247 frame bytes |
| 0x82 | TXDONE | `[ok, attempts]` (attempts = 0 for a plain TX) |
| 0x83 | INFO | `[status, sync(2), errors(2), freq(4), sf, bw, cr, pwr, lost(2), cad_busy(2), defer(2)]` — **19 B, frozen**: the host reads cad_busy/defer as the last 4 bytes, so nothing may be appended |
| 0x84 | LOG | ascii |
| 0x85 | CAD | `[busy(0/1)]` |
| 0x86 | RSSI | `[rssi i16 BE]` |
| 0x87 | SF_DETECTED | `[sf | 0 = none]` |
| 0x88 | TX_STARTED | `[airtime_ms u16 BE]` — emitted just before key-up |
| 0x89 | STATS | 32 B, see below |
| 0x8A | CLOCK | `[ticks u64 BE]` — the same counter EVT_RX stamps with, at full width |
| 0x8B | CAP | 29 B, see below |
| 0x8C | SENSE | `[activity u16 BE, rssi i16 BE]` |
| 0x8F | UNSUPPORTED | `[cmd, reason]` |

**STATS (0x89), 32 bytes.** Bytes 0..24 are byte-identical to the v1 layout and the existing host
parser length-checks `< 24`, so a v1 host reads it unchanged and ignores the tail:

```
[0..4]   rx            frames the data plane classified
[4..8]   filtered      dropped: no installed prefix covers the name
[8..12]  deduped       dropped: duplicate Data object
[12..16] served        answered from the on-device Content Store
[16..20] relayed       re-broadcast by the relay set
[20..22] cad_busy      channel sensed busy before a key-up  (resettable view)
[22..24] defer         transmissions abandoned after backoff (resettable)
--- v2 tail ---
[24..26] chip_rx       SX126x GetStats nbPktReceived
[26..28] chip_crc_err  SX126x GetStats nbPktCrcError   <- the ONLY place a failed decode is visible
[28..30] chip_hdr_err  SX126x GetStats nbPktHeaderErr
[30..32] rx_trunc      frames longer than max_payload (should be 0)
```

**CAP (0x8B), 29 bytes, every multi-byte field big-endian.** The one place this node describes
itself; every field is a source constant or a verified property, and where nothing is known the
field is `0`. For this firmware the reply is fixed:

```
7E A5 8B 1D  02 00 35 C3 6D 80 37 50 28 00 F7 16 00 0F 42 40 02 00 F7
             1C FF FF FE 07 0C 00 00 00 00  30
```

| Bytes | Field | Value | Why |
|-------|-------|-------|-----|
| 0 | proto_ver | 2 | |
| 1 | radio_kind | 0 | SX1262 |
| 2..6 | freq_min_hz | 902 000 000 | the band `set_frequency` image-calibrates for, **not** the SX1262's 150-960 MHz silicon range |
| 6..10 | freq_max_hz | 928 000 000 | ″ |
| 10 | pwr_min_dbm | −9 (0xF7) | real dBm — the SetTxParams range `set_power` clamps to |
| 11 | pwr_max_dbm | +22 (0x16) | ″ |
| 12..16 | stamp_hz | 1 000 000 | **microseconds** — `micros()` adds SysTick sub-ms cycles (RELOAD 7999 @ 8 MHz) to `ms×1000` |
| 16 | stamp_kind | 2 | software counter — the MCU reads its own clock on noticing RxDone in the poll loop; there is no hardware capture |
| 17..19 | max_payload | 247 | the binding limit is the **serial framing**: EVT_RX = 8 header bytes + frame ≤ 255. TX accepts 255 and the LoRa PDU is 255, so 247 is the smaller and therefore the reported number |
| 19..23 | cmd_bitmap | 0x1CFF_FFFE | bits 1..23 (0x01..0x17) + 26 (0x1A) + 27 (0x1B) + 28 (0x1C). Clear: bit 24 = 0x18 TX_AT, bit 25 = 0x19 unassigned |
| 23 | sf_min | 7 | SF5/6 are chip-supported but do not interop with the fleet's SX127x peers |
| 24 | sf_max | 12 | |
| 25..29 | sched_gran_ns | 0 | **this firmware exposes no scheduled-TX engine** — 0 is the truth, not a placeholder |

Defaults: 915 MHz (US) / SF7 / BW125 / CR4-5 / sync 0x12 (→ SX126x reg 0x1424) / preamble 8 /
explicit header / CRC on / **boosted LNA gain** (reg 0x08AC = 0x94; the chip's power-on default is
the power-saving 0x96, which is the wrong default on a bearer that exists for reach) — matched to
the Heltec node.

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
