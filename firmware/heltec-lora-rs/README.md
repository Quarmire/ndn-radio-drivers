# heltec-lora-rs

Open **Rust** firmware for the **Heltec WiFi LoRa 32 V2** (ESP32 Xtensa LX6 + **SX1276**), the third
named-radio node (node C, task #54). It speaks the **7E-A5 v3** serial protocol at feature parity
with the Waveshare SX1262 node (`../waveshare-lora-rs`), so one host driver
(`ndn-radio-drivers/src/lora_serial.rs`) treats the two as peers. The air side is plain **standard
LoRa** — no proprietary header — so it interoperates with any SX127x/SX126x peer.

Async firmware on `embassy` / `esp-rtos`, because `lora-phy` is `embedded-hal-async`.

## Build and flash — the exact commands

The `esp` rustup toolchain (from `espup`) supplies the `xtensa-esp32-none-elf` target, and the GCC
linker it needs is **not on `PATH` by default**. Source the espup environment first or the build
fails with `linker 'xtensa-esp32-elf-gcc' not found`:

```sh
. ~/export-esp.sh                                   # LIBCLANG_PATH + xtensa-esp-elf/bin on PATH
cd ndn-radio-drivers/firmware/heltec-lora-rs
cargo build --release                               # compile only
cargo run   --release                               # flash + monitor over the CP2102 (espflash)
```

`.cargo/config.toml` pins the target, the `linkall.x` link arg and `build-std = ["alloc", "core"]`
(required for this tier-3 target); `rust-toolchain.toml` pins `channel = "esp"`. Nothing else is
needed — no `idf.py`, no ESP-IDF checkout.

Flashing needs no firmware cooperation: the CP2102 drives `EN`/`GPIO0` from DTR/RTS, so `espflash`
puts the chip in its ROM downloader in hardware. (That is why `CMD_ENTER_BOOTLOADER` (0x16), which
the GD32 node implements, is answered `EVT_UNSUPPORTED` here — there is nothing for it to do.)

### Build attribution (bug C7)

`heltec-lora-rs.bin` in this directory is **untracked, gitignored, and predates this firmware**. Do
not flash it and do not attribute any measurement to it. It is not deleted only because it is not
this pass's file to delete; regenerate from source with the commands above.

Residual gap, outside this directory to fix: the repo root `.gitignore` ignores `Cargo.lock`
globally (line 2), for this node and the Waveshare node alike, so the exact dependency patch
versions of a build are not recorded anywhere. `Cargo.toml` pins majors/minors, and the build id
below identifies the *source*, but two machines can still resolve different patch releases.

To make results attributable, `build.rs` stamps the git identity of the source into the image and the
firmware reports it:

* as an `EVT_LOG` (0x84) at boot, and
* as an `EVT_LOG` immediately before every `EVT_INFO` reply to `CMD_GET_INFO` (0x06),

in the form `heltec-lora-rs build=<short-sha>[+dirty] proto=3 stamp_hz=1000000`. A `+dirty` suffix
means the working tree differed from the commit, so the sha alone does not identify what was built.

## Hardware map (Heltec WiFi LoRa 32 V2)

| Function | Pin | Notes |
|----------|-----|-------|
| Host UART | UART0 — TX GPIO1 / RX GPIO3 | 115200 8N1 → CP2102 → `/dev/ttyUSB*` (Linux) / `/dev/cu.usbserial-*` (macOS) |
| SX1276 SPI | SPI2 — SCK GPIO5, MISO GPIO19, MOSI GPIO27 | 2 MHz, mode 0 |
| SX1276 NSS | GPIO18 | driven by the firmware (see `src/regs.rs`) |
| SX1276 RESET | GPIO14 | |
| SX1276 DIO0 | GPIO26 | the only interrupt line wired to `lora-phy`; RxDone / TxDone / CadDone, remapped per mode |
| SX1276 DIO1 | GPIO35 | **C1** — owned by this firmware, not by `lora-phy`; mapped to `FhssChangeChannel` while hopping is on. Pin number from this repo's own attested pinout (`firmware/heltec-lora-node/heltec-lora-node.ino:27`), not a datasheet guess. GPIO34–39 are input-only with no internal pulls, which is right for a push-pull IRQ line |
| PA | PA_BOOST | the antenna is on PA_BOOST, not RFO → real TX range **+2 … +20 dBm** |

UART0 is owned **raw** for the binary protocol; nothing prints text on it in normal operation, or the
framing desyncs. `esp-backtrace` still prints on panic, when the link is lost anyway.

## Host ⇄ firmware protocol (7E-A5 v3)

Framing `7E A5 | type | len | payload[len] | xor-crc`, crc = XOR of type, len and payload. The parser
resyncs on `7E A5` and validates the crc, so a dropped byte costs at most one frame.

Every opcode this node does not implement is answered **`EVT_UNSUPPORTED` (0x8F) `[cmd, reason]`** —
never silence, never a fake success. Reasons: `0x01` unknown opcode, `0x02` no hardware, `0x03` bad
length, `0x04` out of range.

### Host → firmware

| Type | Name | Payload | On this node |
|------|------|---------|--------------|
| 0x01 | TX | LoRa frame bytes | yes |
| 0x02 | SET_FREQ | u32 BE Hz | yes |
| 0x03 | SET_MOD | `[sf, bw_code, cr_code]` | yes — see **Bandwidth codes** |
| 0x04 | SET_PWR | `[i8 dBm]` | yes, clamped to +2…+20 |
| 0x05 | SET_SYNC | `[sync byte]` | yes — written straight to `RegSyncWord` (0x39) |
| 0x06 | GET_INFO | *(empty)* | yes → `EVT_LOG` (build id) + `EVT_INFO` |
| 0x07 | SET_BEACON | `[enabled]` or `[enabled, period_mult]` | yes — real 10 s × mult period |
| 0x08 | CAD | *(empty)* | yes → `EVT_CAD` |
| 0x09 | GET_RSSI | *(empty)* | yes → `EVT_RSSI`, real dBm |
| 0x0A | SET_CAD_CFG | `[sym, det_peak, det_min]` | **yes (P2)** → `EVT_INFO`. `det_peak` → `RegDetectionThreshold` (0x37), clamped to the datasheet's [0x0A, 0x0C]; `det_min` → `RegDetectOptimize` (0x31) bits 2:0, clamped to {0x03, 0x05} — that field is an SF-class detector selector on this chip, **not** the SX1262's minimum-symbol count. `sym` is **ignored**: the SX1276 has no CAD symbol-count register (the firmware-side equivalent, `cad_repeat`, is 0x12). A clamp that moved any byte also emits an `EVT_LOG` naming what was written. The pair is re-applied after every `set_modulation_params`, without which `lora-phy` would overwrite it from the SF on the next arm |
| 0x0B | SET_LBT_CFG | `[cw_ms u16 BE, max_backoff, max_attempts]` | yes |
| 0x0C | SET_PREAMBLE | `[preamble u16 BE]` | yes |
| 0x0D | SF_SCAN | *(empty)* | yes → `EVT_SF_DETECTED` |
| 0x0E | TX_LBT | LoRa frame bytes | yes → `EVT_TX_STARTED` then `EVT_TXDONE` |
| 0x0F | SET_NAME_FILTER | `[u64 BE hash]*` | yes |
| 0x10 | SET_RELAY | `[u64 BE hash]*` | yes |
| 0x11 | DATAPLANE | `[cs_serve, dedup, hop_on, hop_base_ch, hop_span]` | yes |
| 0x12 | SET_SENSE_CFG | `[rssi_thresh i16 BE, cad_repeat]` | yes |
| 0x13 | GET_STATS | *(empty)* | yes → `EVT_STATS` (24 B, all real) |
| 0x14 | RESET_STATS | *(empty)* | yes |
| 0x15 | SET_DEBUG | `[on]` | yes; turning it on also emits a live chip-state `EVT_LOG` |
| 0x16 | ENTER_BOOTLOADER | `[0xB0,0x07]` | **`EVT_UNSUPPORTED`/NO_HARDWARE** — GD32-only; the ESP32 ROM downloader is entered by the CP2102 in hardware |
| 0x17 | READ_CLOCK | *(empty)* | yes → `EVT_CLOCK` |
| 0x18 | TX_AT | `[delay_us u32 BE][frame]` | **yes (P1)** → `EVT_TX_STARTED` at the staging point, then `EVT_TXDONE`. There is still no TSF comparator and no delayed key-up on the SX1276 — the MCU is the queue. The frame is programmed into the chip 1.5–2.7 ms *ahead* of its deadline so that only the 2-byte `RegOpMode ← TX` write is left to fire: `sched_gran_ns` = 99 µs, derived below. One deep (a second request while one is armed answers `EVT_TXDONE [0,0]`); delays over 60 s are `EVT_UNSUPPORTED`/OUT_OF_RANGE, because the host's own reply timeout caps there |
| 0x1A | GET_CAP | *(empty)* | yes → `EVT_CAP` |
| 0x1B | SENSE | *(empty)* | yes → `EVT_SENSE` |
| 0x1C | SET_RX_GAIN | `[0 = power-saving \| 1 = boosted]` | **yes (P3)** → `EVT_INFO`. `RegLna` (0x0C): LnaGain stays G1 (maximum gain) and LnaBoostHf switches 00 ↔ 11 (150% LNA current, ~+3 dB). Payload shape is the Waveshare node's byte for byte — the SX1276's six-step LnaGain field is deliberately **not** exposed, because a second byte on one node would make 0x1C mean two different things in one fleet. Re-applied after every `do_rx`/`do_cad`, which each rewrite `RegLna` from `lora-phy`'s fixed `rx_boost` |
| 0x1D | SET_PHY | `[packet_type u8]` | **yes (C2)** → the **whole new `EVT_CAP`**, because every capability field is per-PHY. `phy_bitmap` on this node is `0x0000_0001`: LoRa only. A `packet_type` outside the bitmap → `EVT_UNSUPPORTED`/OUT_OF_RANGE (the host asked for something never claimed); one **inside** it that the chip does not corroborate → `EVT_PHY_ERR` (0x8D) `[phy, RegOpMode]`. Selecting the PHY already running is a *verify* (read `RegOpMode`, check bit 7 `LongRangeMode`), not a re-init — `LongRangeMode` is writable only in SLEEP, so a real transition costs a sleep + full re-init |
| 0x1E | SET_HOP | `[hop_ctrl][hop_period u16 BE][n][freq_hz u32 BE]*n`, n ≤ 40 | **yes (C1)** → `EVT_INFO`. Real SX1276 **intra-packet** frequency hopping — see the section below |
| 0x1F | TX_AT_ABS | `[target_ticks u64 BE][frame]` | **yes (C3)** → `EVT_TX_STARTED` at the staging point, then `EVT_TXDONE`. Same one-deep slot and same stage/fire machine as 0x18, with the deadline read off the wire instead of derived from `Instant::now()` at command-processing time. Target on the counter `CMD_READ_CLOCK` returns, at full 64-bit width. **Target in the past → fires now**, unclamped, so `report_sched_error` states the real lateness on the wire; further than 60 s ahead → `EVT_UNSUPPORTED`/OUT_OF_RANGE (the same bound as 0x18, so the answer to "how far ahead can I schedule?" does not depend on which opcode you reach for) |
| **0x20** | **GET_HOPTRACE** | *(empty)* | **yes (H2)** → `EVT_HOPTRACE` (0x8E). This node's own hop timeline: up to 32 free-running `(idx, t_ticks)` entries, most recent last, on the counter `CMD_READ_CLOCK` returns. Reading does **not** clear it. See the section below. ⚠ 0x20 is opcode **32** and `EVT_CAP.cmd_bitmap` is a u32 that is full at 0x1F, so this opcode is **not** advertised there — discover it by sending it (`EVT_HOPTRACE` vs `EVT_UNSUPPORTED`), or read `bitmap_ext` from the `CMD_SET_DEBUG` dump |

### Firmware → host

`EVT_RX` 0x81 `[rssi i16 BE, snr i16 BE, ts u32 BE, LoRa bytes]` (`rssi`/`snr` from the SX1276's
per-packet `RegPktRssiValue`/`RegPktSnrValue`; `ts` in `EVT_CAP.stamp_hz` units) ·
`EVT_TXDONE` 0x82 `[ok, attempts]` · `EVT_INFO` 0x83 (19 B, fixed) · `EVT_LOG` 0x84 ·
`EVT_CAD` 0x85 · `EVT_RSSI` 0x86 · `EVT_SF_DETECTED` 0x87 · `EVT_TX_STARTED` 0x88 ·
`EVT_STATS` 0x89 (24 B) · `EVT_CLOCK` 0x8A · `EVT_CAP` 0x8B (**34 B in v3**) · `EVT_SENSE` 0x8C ·
`EVT_PHY_ERR` 0x8D `[requested_phy, chip_status]` ·
**`EVT_HOPTRACE` 0x8E** `[stamp_hz u32 BE][n u8][idx u8, t_ticks u32 BE]*n` (H2 — ≤ 165 B) ·
`EVT_UNSUPPORTED` 0x8F.

### Bandwidth codes — the canonical space, pinned

**`0 = 125 kHz, 1 = 250 kHz, 2 = 500 kHz`** is the canonical 7E-A5 bandwidth code. It is what the
host already computes in (`set_bandwidth_khz`) and what `EVT_INFO.bw` reports back.

This firmware **also** decodes the legacy SX1262 modulation codes `0x04/0x05/0x06`, which the shipped
host emitted via `bw_to_fw`. The two sets are disjoint, so accepting both costs nothing and means a
host that has not been updated still gets the width it asked for. `EVT_INFO` always answers in the
canonical space, so the host can see which width was actually applied. The mapping is pinned by a
`const` assertion table in `src/main.rs` that the compiler checks on **every** build (a `#[cfg(test)]`
module could not: this crate is `no_std`/`no_main` for a bare-metal target, so `cargo test` cannot
build it and such a test would never run).

## What this node reports about itself — `EVT_CAP` (0x8B, **34 B**, v3)

Every field is a source constant or a verified property of this build. Where nothing is knowable the
field is 0 and says so, because a fabricated number is worse than 0 — the host believes it.

★ **`EVT_CAP` describes the CURRENT PHY, not the board.** `max_payload`, `sf_min`/`sf_max`, the band
and `sched_gran_ns` are all properties of the modulation in effect — an LR2021 in FLRC carries 47
bytes and has no spreading factor; the same chip in LoRa carries far more and has SF7–SF12. That is
why `CMD_SET_PHY` replies with a **whole new `EVT_CAP`** and the host replaces its profile wholesale
rather than patching fields. On this node there is one PHY, so today the values are constant.

⚠ **`radio_kind` names the PART in v3**, not the mode: `0 = SX1262, 1 = SX1276, 2 = LR2021`. v2's
"3 = LR2021-LoRa" is retired — encoding a one-time `SetPacketType` call as an identity was the design
error v3 exists to undo. For this node the value is unchanged (1 under v2, 1 under v3), so a v2 host
reading a v3 Heltec still gets a sane kind.

| Bytes | Field | Value | Where it comes from |
|-------|-------|-------|---------------------|
| 0 | proto_ver | 3 | |
| 1 | radio_kind | 1 | SX1276 — the **part** |
| 2..6 | freq_min_hz | 902 000 000 | the band this **board** is matched for. NOT the SX1276's 137–1020 MHz silicon range: this is the 915 MHz Heltec variant, whose PA matching, SAW filter and antenna are tuned for US 902–928, and advertising the silicon range would invite a tune where the board radiates almost nothing |
| 6..10 | freq_max_hz | 928 000 000 | as above |
| 10 | pwr_min_dbm | +2 | real dBm — the PA_BOOST interval `lora_phy::sx127x::sx1276::set_tx_power` clamps to. Never a register unit |
| 11 | pwr_max_dbm | +20 | as above (above +17 dBm the driver enables PaDac 20 dBm and raises OCP to 240 mA) |
| 12..16 | stamp_hz | 1 000 000 | `embassy_time::TICK_HZ`, a compile-time constant of the driver actually linked (esp-rtos selects `tick-hz-1_000_000`) — verified against this build, not assumed |
| 16 | stamp_kind | 2 | **software counter**: the MCU reads its own monotonic clock when the DIO0 interrupt wakes it. The SX1276 has no RX-time capture register, so a hardware stamp is impossible on this radio |
| 17..19 | max_payload | 247 | the REAL end-to-end cap, set by the serial framing, not the radio: an event's `len` is one byte, so an `EVT_RX` payload is ≤ 255 B of which 8 are the rssi/snr/ts header. The FIFO (256 B), the LoRa PDU (255 B) and `CMD_TX`'s accept (255 B) are all larger, so 247 binds. A frame larger than this is **counted and dropped**, never truncated into something that looks complete |
| 19..23 | cmd_bitmap | 0xFDBF_FFFE | bit N set ⇔ opcode N is implemented. It is also the dispatcher's own "known opcode?" oracle, so the bitmap and the command handler cannot drift apart. ☠ **This word is full at opcode 0x1F.** `CMD_GET_HOPTRACE` is 0x20 = bit 32 and cannot be represented in a u32, and the 34-byte v3 layout was **not** widened to make room: that would be a fleet-wide wire change desynchronising this node from the LR2021 — the very node the hop trace exists to be compared with. Opcodes 32..63 live in `CMD_BITMAP_EXT`, emitted in the `CMD_SET_DEBUG` dump as `bitmap_ext=`; the primary discovery path is the one the 7E-A5 rule set already guarantees — send the opcode and read the answer |
| 23 | sf_min | 7 | SF6 needs an implicit header and would break fleet interop |
| 24 | sf_max | 12 | |
| 25..29 | sched_gran_ns | 99 000 | P1. A **sum of source constants**, not an estimate: 1 000 ns of `embassy-time` tick quantisation (`TICK_HZ` = 1 MHz, and `Timer::at` fires on the first tick at or after the target) + 8 000 ns for the 2-byte key-up write at the 2 MHz SPI rate `main` configures + 40 000 ns of PA ramp (`lora-phy` programs `RegPaRamp` = `RampTime::Ramp40Us` for a TX prep) + 50 000 ns of budget for the executor wake and NSS framing, which are **not measured on this board** and are therefore given at least as much room as everything that is. One-sided: the node never keys up early. The firmware measures its own error against the same counter `EVT_RX` stamps with and emits an `EVT_LOG` (`sched err=…us`) on every miss, whether or not debug is on — so the number is falsifiable from the wire. ★ v3: this is the **firmware-side** granularity and it is the same on `CMD_TX_AT` and `CMD_TX_AT_ABS` — both leave `fire_tx` the same single write. What the absolute opcode removes is the host→node serial transit, which the relative one adds to the *host's* placement and which no field here can express (see **Scheduled TX** below) |
| 29..33 | phy_bitmap | 0x0000_0001 | **C2.** Bit N set ⇔ `CMD_SET_PHY N` is usable, in the LR2021's `SetPacketType` numbering, which v3 makes the fleet-wide PHY namespace. One bit, because this pass brought up one PHY |
| 33 | phy_current | 0 | LoRa |

Exact payload:

```
03 01 35 C3 6D 80 37 50 28 00 02 14 00 0F 42 40 02 00 F7 FD BF FF FE 07 0C 00 01 82 B8 00 00 00 01 00
```

on the wire (with framing and crc):

```
7E A5 8B 22 03 01 35 C3 6D 80 37 50 28 00 02 14 00 0F 42 40 02 00 F7 FD BF FF FE 07 0C 00 01 82 B8 00 00 00 01 00 63
```

## `EVT_INFO` (0x83) — 19 bytes, all real

`[status, sync(2), errors(2), freq(4), sf, bw, cr, pwr, lost(2), cad_busy(2), defer(2)]`. Fixed
width: the host reads `cad_busy`/`defer` as the last four bytes, so nothing may ever be appended —
new counters go in `EVT_STATS`.

* **`status`** — the SX1276's `RegOpMode` (0x01), read live. This chip has no analogue of the
  SX1262's `GetStatus()` byte; `RegOpMode` is the closest real thing (bit 7 LongRangeMode, bits 2:0
  mode: 0 SLEEP, 1 STDBY, 3 TX, 5 RXCONTINUOUS, 7 CAD).
* **`sync`** — `RegSyncWord` (0x39) read back, in the **low** byte. The SX127x sync word is 8 bits
  where the SX1262's is 16, so the high byte is structurally 0. That is the chip, not a placeholder.
* **`errors`** — this firmware's own count of radio-layer failures. **It is not the chip's device
  errors.** The SX1276 has no equivalent of the SX1262's `GetDeviceErrors` (0x17), so there is no
  such register to read on this radio; a firmware error count is the honest substitute.
* **`lost`** — host-link errors seen by the UART reader task, one per `RxError` (a FIFO overflow is
  one error, not one byte). Should stay 0; a climbing count is the link telling you it is losing
  commands.
* **`cad_busy` / `defer`** — the real CSMA counters. `cad_busy` here is a *resettable view*; the
  underlying counter behind `EVT_SENSE.activity` free-runs and is never cleared.

## On-device NDN data plane

`src/ndn.rs` is **not a copy**: it is `../waveshare-lora-rs/src/ndn.rs`, included by `#[path]`
exactly as `lr2021-nrf54l15-rs`'s `m6_bridge` includes it, so name-hash / filter / dedup / Content
Store semantics cannot drift between the nodes that must interoperate. It builds on `ndn-embedded`
(`default-features = false, features = ["cs"]` — the allocation-free path). Every feature defaults
**inert**: empty filter passes everything, CS-serve off, dedup off, hopping off, so a freshly-flashed
board behaves exactly like the plain modem until the host opts a feature in.

## ★ Intra-packet frequency hopping — `CMD_SET_HOP` (0x1E)

```
CMD_SET_HOP  [hop_ctrl u8][hop_period u16 BE][n u8][freq_hz u32 BE]*n   ->  EVT_INFO     (n <= 40)
```

**This is the substrate `DataPlane::hop_channel` was written for and never had.** Host-driven
retuning cannot hop inside a packet on any node: this is the fastest-retuning radio in the fleet at
5 597 µs, and even so a *serial round trip* to command a retune has a measured mean of 10 733 µs —
two orders of magnitude longer than a LoRa symbol. Hopping inside a packet has to be done by the
modem, with the MCU only feeding it the next carrier. That is exactly what the SX1276's FHSS mode
is, and `lora-phy` 3.0.1 reaches none of it.

### What `lora-phy` gives you, and what it does not

It **knows the interrupt exists** — `IrqMask::FhssChangedChannel = 0x02`, and the DIO comment block
at `sx127x/radio_kind_params.rs:63-87` lists it against DIO1 and DIO2 — and it exposes **no hopping
API whatsoever**: no `RegHopPeriod`, no hop list, no way to reach `RegFrf` outside `set_channel`, and
`set_irq_params` **masks** `FhssChangedChannel` in every one of its four radio-mode arms. So the
whole feature is unreachable through the crate by construction. `src/regs.rs`'s raw `SharedSpi`
handle is what it is for.

| Register | | Used for |
|---|---|---|
| `RegHopPeriod` | 0x24 | dwell in symbols; **0 = hopping off**. `lora-phy` has no entry for this register at all, so nothing it does can disturb the value |
| `RegFrf` | 0x06 / 0x07 / 0x08 | the carrier, rewritten on every hop from a **precomputed** synthesiser word |
| `RegHopChannel` | 0x1C | bits 5:0 the modem's own hop counter, bit 7 `PllTimeout` (the synthesiser did not lock in time — a real, chip-sourced miss count, surfaced in the debug dump) |
| `RegIrqFlagsMask` | 0x11 | bit 1 cleared after every arm, to undo `lora-phy`'s masking |
| `RegIrqFlags` | 0x12 | bit 1 cleared per hop, **write-1-to-clear on that bit only** so a latched RxDone/TxDone in the same register survives for `process_irq_event` |
| `RegDioMapping1` | 0x40 | bits 5:4 ← 01, DIO1 = `FhssChangeChannel` |

### The DIO mapping chosen, and what it displaces

**DIO1 (GPIO35), bits 5:4 of `RegDioMapping1` set to `01`. It displaces `RxTimeout`** — DIO1's
power-on mapping, raised only in `RxMode::Single`, which this firmware never uses (every receive is
`RxMode::Continuous`) and which was not wired to a GPIO at all before this pass. The other DIO1
option, `CadDetected`, is likewise unused: CAD here waits on `CadDone` on DIO0 and reads the
activity bit out of `RegIrqFlags`. **Nothing that was working stops working.**

DIO2 would have been *cheaper* — on the SX1276 all three of its encodings mean `FhssChangeChannel`,
so it needs no mapping write ever — and it was rejected anyway, because **no GPIO number for DIO2 on
this board is attested anywhere in this project**. DIO1 = GPIO35 is: it is in the working RadioLib
firmware `firmware/heltec-lora-node/heltec-lora-node.ino:27` and was passed to `new Module(CS, DIO0,
RST, DIO1)`. A plausible pin number would have been an invention, and an invented pin is a hop
interrupt that never arrives.

The mapping is written **once**, at enable. `lora-phy`'s three `RegDioMapping1` writes all preserve
bits 5:4 (`& 0x3f` in its Transmit and CAD arms, `& 0x3f & 0xfc` in its Receive arm) — pinned by a
compile-time assertion in `src/regs.rs` — so nothing it does can un-map DIO1.

⚠ **A bug in `lora-phy` you must not step into.** Its `DioMapping1Dio1` enum encodes the values at
`0b01 << 2` with mask `0xf3`, i.e. in **bits 3:2 — the DIO2 field**, not DIO1's bits 5:4. It is
`#[allow(dead_code)]` and the crate never uses it, so the error has never bitten anyone; using it
here would have mapped DIO2 while wiring DIO1. `regs::DIO1_FHSS_CHANGE_CHANNEL` is at the
datasheet's bit positions.

### The service path — and why it cannot cancel an SPI future mid-flow

Four SPI transactions, 8 bytes, **~32 µs of byte time at 2 MHz**, and **no arithmetic**: the
`RegFrf` word was computed by `regs::frf_of_hz` when the host set the list, so no soft-float divide
(this chip has no double-precision FPU) runs in the interrupt path. The order is the Semtech
reference driver's — clear the flag, ask the chip which hop it is on, program that hop's carrier —
and `(RegHopChannel & 0x3F) % n` is used directly as the list index, which is the same reference
driver's own pattern (`SX1276OnDio1Irq` → the application's `SetChannel(HoppingFrequencies[idx])`).
Using the chip's counter rather than one of our own is what keeps the sequence self-synchronising:
a software counter would drift the moment an interrupt was serviced late or a packet restarted.

`lora-phy` warns that cancelling `process_irq_event` mid-flow can lock the radio up, and the C3
restructuring made the only cancellable await one that touches neither SPI nor driver state. **The
hop path does not weaken that, and the reason is structural rather than careful:**

* `Radio::service_hop` — the only SPI-touching future the hop path adds — is **never a `select`
  branch.** It is awaited as a bare statement in both places a hop can arrive: the main loop's
  `Wake::Hop` arm, and the `FireWake::Hop` arm inside `Radio::fire_tx`. In each case the `select`
  has already resolved and been bound to a plain local, so every borrow it held is released and
  there is nothing left that could drop the future partway.
* What the `select`s race is **only** GPIO level waits and timers. The hop branch is
  `Input::wait_for_high()` on DIO1 — byte for byte the same construction as the existing DIO0 wait,
  so the C3 argument transfers unchanged: no SPI, no chip state, no driver state, and because
  esp-hal implements it as a **level** event (`Event::HighLevel`), re-arming while DIO1 is still
  high fires immediately, so a cancelled wait cannot lose a hop either.
* The flag is cleared **first**, and with `RegIrqFlags ← 0x02` rather than `← 0xFF`: writing a 0 to
  a bit of that register leaves it alone, so a TxDone that landed while we were servicing a hop
  survives to be consumed. Clearing first is also what drops DIO1 *before* the service returns, so
  the level wait cannot re-fire on the hop just handled.
* The hop branch exists **only while hopping is enabled, and never while a frame is staged.** Staged
  means the chip is idle in standby with no packet in flight, so no hop can occur — and `stage_tx`'s
  `apply_hop` cleared any latched flag on the way in, so DIO1 is low and cannot hold up a branch
  that is no longer there.
* A hop that cannot be serviced (SPI error) would leave DIO1 latched high and spin the loop, so it
  **turns hopping off** — `RegHopPeriod ← 0`, branch removed, `EVT_LOG` emitted. That is the one
  recovery that cannot spin whatever the bus is doing.

### ⚠ One hop can be swallowed — and the sequence still does not desync

`lora-phy`'s `process_irq_event` is called with `clear_interrupts = true` on both the RX and TX
paths and clears `RegIrqFlags` with `0xFF`, hop flag included. So when RxDone/TxDone and a
`FhssChangeChannel` land close enough together that the DIO0 branch wins the `select`, that one
hop's `RegFrf` write is skipped and the modem spends the dwell on the previous carrier. **This is
precisely why the list index is read from `RegHopChannel` rather than counted in software:** the
chip kept counting, so the next serviced hop lands on the right entry and the sequence
re-synchronises by itself. The cost is one dwell, not a broken link. Left as is rather than
engineered around, because the alternative is taking IRQ-flag clearing away from the driver on the
two paths this firmware most depends on.

### The invariant, and the refusals

**Hop channel 0 IS the base frequency.** Enabling sets `st.p.freq_hz` from `list[0]`, and
`CMD_SET_FREQ` maintains the same invariant from the other side by moving `hop[0]` with the base —
one shift and one divide, no SPI and no UART, skipped entirely while hopping is off. So
`EVT_INFO.freq`, the modulation parameters and the channel a packet actually starts on are one value
and cannot drift.

Every one of these is `EVT_UNSUPPORTED [0x1E, …]`, never a fudge:

| Condition | Reason | Why not just cope |
|---|---|---|
| `hop_ctrl` outside {0, 1} | OUT_OF_RANGE | an unknown mode is not "on" — the same discipline `CMD_SET_RX_GAIN` applies to its boolean |
| `hop_period` > 255 | OUT_OF_RANGE | **`RegHopPeriod` is 8 bits** and the wire field is a u16. Truncating would hop up to 256× faster than asked and desynchronise the pair silently |
| `hop_period` = 0 with hopping on | OUT_OF_RANGE | 0 is how the register means "off"; asking for both at once is a contradiction, not a default |
| `n` > 40, or `n` = 0 with hopping on | OUT_OF_RANGE | the v3 contract's cap; an empty list is not a hop sequence |
| payload shorter than `4 + 4n` | BAD_LENGTH | |
| any channel outside 902–928 MHz | OUT_OF_RANGE | the SX1276 silicon reaches 137–1020 MHz; this **board's** PA match, SAW filter and antenna do not. A hop out there is a hop into a channel the node barely radiates on |

The whole list is validated before any of it is committed: a half-applied hop list is a sequence the
two ends of a link no longer agree on.

### ⚠ What enabling it costs, stated up front

In FHSS **the receiver hops too** — it must, to follow the transmitter — so a hopping node is deaf
to a non-hopping peer, and the interrupt fires every `hop_period` symbols continuously, not only
during a transmission. At SF9/125 kHz (T_sym 4.096 ms) with `hop_period = 16` that is ~15 interrupts
a second at ~32 µs of SPI each: nothing. At `hop_period = 1` with SF7/500 kHz (T_sym 256 µs) it is
~3 900 a second, ~16% of the SPI budget, on a node that has other work. The firmware does not forbid
it; it is the host's dial, and this is the bill.

## ★ Hop-event timestamping — `CMD_GET_HOPTRACE` (0x20) / `EVT_HOPTRACE` (0x8E)

```
CMD_GET_HOPTRACE  []                                                   ->  EVT_HOPTRACE
EVT_HOPTRACE      [stamp_hz u32 BE][n u8][ idx u8, t_ticks u32 BE ]*n       (n <= 32, <= 165 B)
```

### The question this instrument exists to answer

An LR2021 and this SX1276 both do LoRa **intra-packet frequency hopping**, each interoperates with
its own kind, and they cannot hop with each other. Measured on hardware:

| TX (hopping) | RX | RX hop | result |
|---|---|---|---|
| Heltec n=1 | Waveshare (no hop support) | off | **4/4** |
| LR2021 n=1 | Waveshare | off | **4/4** |
| Heltec n=1 | LR2021, hop **off** | off | **4/4** |
| Heltec n=1 | LR2021, hop **on** | on | **0/4** |
| LR2021 n=1 | Heltec, hop **on** | on | **1/20** |
| LR2021 n=1 | Heltec, hop **off** | off | **20/20** |
| LR2021 n=4 | LR2021, hop on | on | 4/4 |

**`n = 1` is a one-entry hop list: the hop machinery runs but the carrier can never move.** So it is
not the frequency sequence or the phase (n=1 still fails); not the frame format (a plain receiver
decodes either part's hop-mode TX perfectly); and not structural (1/20 is not 0/20 — a format
mismatch would be absolute). What is left is that **the two disagree about WHEN a hop boundary
falls**, which is what SX127x/LR2021 §9.8 says in as many words. A period sweep is already ruled
out: with this node's RX period fixed at 8 symbols, sweeping the LR2021's TX period over 2/4/8/16
gave 0–1 of 10 at **every** setting, so no parameter search over the exposed knobs will find it.

★ **Each node timestamps its OWN hop events.** No cross-vendor reception is required to compare the
two timelines — which is the only reason this is measurable at all, because the link that would
carry the comparison is the very thing that is broken.

At **SF7 / BW 125 kHz** one symbol is 2⁷/125000 = **1.024 ms**, so a nominal 8-symbol hop period is
**8.192 ms** and both parts should show that interval. Whatever differs — the interval itself, the
instant of the first hop relative to the start of a frame, or whether hops continue between frames —
is the answer.

### The encoding

| Field | Bytes | Meaning |
|---|---|---|
| `stamp_hz` | 4, BE | **this node's own tick rate**: 1 000 000 here, `embassy_time::TICK_HZ`, identical to `EVT_CAP.stamp_hz` and to the units of `EVT_RX.ts` and `EVT_CLOCK`. On the wire, and **not** converted to microseconds in firmware — the host divides, so a 16 MHz node does not have to throw away resolution to match this 1 MHz one |
| `n` | 1 | entries returned, 0…32, **most recent last** |
| `idx` | 1 | bits **5:0** = `RegHopChannel`'s `FhssPresentChannel`, **verbatim** (not `% n`); bit **7** = the hop was taken while **transmitting** (H4); bit 6 spare |
| `t_ticks` | 4, BE | `Instant::now()` sampled at the top of `service_hop`, low 32 bits, in `stamp_hz` units — the same counter and the same truncation as `EVT_RX.ts`, so the two wrap together (~71 min at 1 MHz) |

`idx` carries the chip's **raw** 6-bit counter rather than the list index the retune used, because
the experiment above runs on a **one-entry** list: `field % 1` is 0 forever and would hide the
modem's hop counter entirely, while the raw field still counts 0, 1, 2, … and wraps at 64.

☠ **`idx` is the one field that is NOT the same quantity on the LR2021**, though the byte, the mask
and the flag bit are identical. Here it is the *chip's* counter, read out of `RegHopChannel`; there
it is a *firmware* count of recorded hop interrupts, because the LR2021 exposes no hop-index
register at all (`SetLoraHopping` is commented out of its command spec and there is no
`GetLoraHopStatus`). Only this node's `idx` can reveal a hop that was swallowed (it jumps by more
than 1) or a per-packet reset of the modem's counter. ★ **Compare `t_ticks` across the two nodes;
compare `idx` only within one.**

The ring is **free-running and wraps**, and **reading does not clear it** — the `EVT_SENSE.activity`
contract. Neither does `CMD_SET_HOP 0`: reading the trace *after* a run is the use case, and
clearing on disable would delete the measurement at the moment it was taken. `n = 0` is a real
answer meaning *this node has serviced no hop since boot*, since only `service_hop` ever writes the
ring. At `hop_period = 8` / SF7 / BW125 the ring turns over in 262 ms, so a stale entry from an
earlier run cannot survive into a new one; if one did, its timestamp would say so.

★ **The LR2021 node implements the same rule** (`hoptrace::HopTrace::arm`): arming a plan clears the
ring, disarming does not. That agreement is load-bearing rather than tidy. If one node cleared on
`CMD_SET_HOP 0` and the other did not, a harness that stopped the hopping before pulling the
timeline would get a full trace from one part and `n = 0` from the other — and at the host, `n = 0`
from a part that hops is indistinguishable from **"this part does not signal its hops at all"**,
which is a conclusion the measurement protocol explicitly draws.

This node **never** answers `EVT_UNSUPPORTED [0x20, NO_HARDWARE]`: it can stamp its hops, on the
same counter it stamps everything else with. That reply is reserved for a part that genuinely
cannot — the rule being *never a fabricated timeline*, which is also why `n = 0` is returned plainly
rather than padded into something that looks like data.

### ⚠ Where the stamp is taken, and what is still between it and the RF boundary

The stamp is `Instant::now()` as the **first statement of `Radio::service_hop`**, before that
function's four SPI transactions — those are ~32 µs of byte time at 2 MHz and would otherwise sit
inside the measurement. Nothing may be inserted above that line.

The SX1276 offers nothing closer: it has **no RX-time capture register and no hop-time capture
register**, which is the same reason `EVT_CAP.stamp_kind` is 2 (software counter). So the MCU-side
stamp is the best this part allows, and what matters is stating what remains:

| Term | Value | Measured? |
|---|---|---|
| RF hop boundary → DIO1 asserts | unknown | **no** — no datasheet figure, no register to read it from |
| DIO1 high → GPIO interrupt → esp-hal waker → embassy executor → task resumes | unknown | **no** — never timed on this board |
| task resumes → `Instant::now()` | ≤ 1 tick = 1 µs | derived (`STAMP_HZ` = 1 MHz, and the counter has 1 µs resolution) |
| main task busy elsewhere when DIO1 asserts | 0 … milliseconds | **no** |

The last row is the one that can dominate: the hop branch is a `select` arm of a single-threaded
loop, so a hop landing while that task is inside a bare SPI or UART statement waits for it. The
known worst cases in this firmware are `stage_tx`'s ~30-transaction register burst (≈ 1.5–2.7 ms,
budgeted at `STAGE_FIXED_US`) and `write_all` draining a full 128-byte UART TX FIFO at 115200
(≈ 11 ms). Neither has been timed against a hop. ⚠ The 11 ms case **exceeds one 8.192 ms hop
period**, so it is not only jitter: it can put two hops on the wrong side of one stamp. Do not poll
`CMD_GET_HOPTRACE` (a 170-byte reply, ≈ 14 ms of UART) during the window being measured.

### ☠ A fifth term that is a LOSS, not a latency

**A hop that coincides with RxDone, TxDone or HeaderValid produces no entry at all.** `lora-phy`'s
`process_irq_event` clears `RegIrqFlags` with `0xFF`, hop flag included (see "One hop can be
swallowed" above), so when the DIO0 branch of the `select` wins that round nothing reaches
`service_hop` to be stamped. It is pre-existing behaviour of the C1 hop path, not something the
trace added — but it lands on the trace, and a reader differencing across the gap would read
**2 × the hop period and believe it**.

★ This is the case that pays for carrying the chip's raw counter in `idx`: the modem kept counting,
so the entry after a swallowed hop arrives **2 higher, not 1**. The host's rule is therefore not
"difference consecutive stamps" but *"difference consecutive stamps and divide by the `idx` step"*,
and any interval whose `idx` step is not 1 is a gap rather than a measurement.

⚠ The loss is **correlated with frames**, not random — it happens exactly at a packet-boundary
interrupt, which is the region of the timeline the hop question is about — and its rate has never
been counted. The LR2021 fails differently in the same situation: it records the entry but may stamp
it with the frame's instant instead of the hop's, and counts how often (`HopTrace::coalesced`). So
**prefer to take each timeline on a node that is hopping but not carrying traffic.**

**No guessed offset is folded into a stamp**, and no interpolation to a symbol boundary is
attempted. A trace whose own offset is unknown cannot answer a timing question, so the offset is
declared unknown rather than invented.

★ **The trace makes its own offset falsifiable, which is why it is still worth taking.** Every
*constant* term above cancels in the **difference** of two stamps, so consecutive entries measure
the hop **interval** with only the variable part left in — and the spread of those intervals over a
quiet run is a direct upper bound on the jitter of the whole wake path. Read the interval first;
trust an absolute offset only once it has been measured.

A tighter stamp is possible in principle — a custom GPIO ISR sampling the counter before the
executor runs — and is deliberately not done here: esp-hal's `Input::wait_for_high` owns that pin's
interrupt to drive the async waker, so installing a handler on it would replace the mechanism the
cancel-safe hop path is built on. If the interval jitter turns out to matter, that is the next step
and a pass of its own.

### H4 — a hop taken while transmitting vs one taken while listening

In FHSS the receiver hops too, so the interrupt fires every `hop_period` symbols **forever**, not
only during a transmission: a trace read after a frame otherwise mixes the frame's own hops with the
idle-RX hops that surround it. That is exactly the confusion that made the raw `hops` counter
misleading, so one bit resolves it.

`HOPTRACE_TX_FLAG` (bit 7 of `idx`) is set **iff** the event was serviced from `Radio::fire_tx` —
the chip in TX with a frame in flight. Clear means *not transmitting*: the main loop serviced it,
which is idle RX or a reception in progress. The call site is the authority and no extra register is
read for it, and the two sites are exhaustive because `fire_tx` does not return until TxDone, a
timeout or an error, so the main loop cannot run during a transmission.

This is also what makes *"the instant of the first hop relative to the start of a frame"* readable
off the trace: the first TX-flagged entry after a run of unflagged ones **is** the frame's first
intra-packet hop.

### Cancel safety (H3) is untouched

`service_hop` gains exactly two things — `Instant::now()`, a counter read on the MCU, and
`HopTrace::push`, arithmetic on a fixed RAM array. **Neither is an `await`**, so the set of points
at which that future can suspend is byte-for-byte what it was, and it remains awaited as a **bare
statement** at both call sites (`Wake::Hop` in the main loop, `FireWake::Hop` inside `fire_tx`),
never as a `select` branch. The `select`s still race only GPIO level waits and timers. Nothing about
`CMD_GET_HOPTRACE` touches the radio at all: it reads a RAM array in the command path and emits one
frame, so it adds no SPI transaction anywhere and cannot interleave with a hop it is reporting (the
firmware is single-threaded and `send_hoptrace` snapshots the ring into a stack buffer before the
first UART byte goes out).

### Cost

Per hop: one `Instant::now()` and ~10 words of RAM stores — **no SPI transaction is added**, because
the `idx` comes from the `RegHopChannel` read `service_hop` already performs. Static cost: 32 × 5
bytes of ring plus two indices, ≈ 176 B of RAM, allocated inside `Hop` whether or not hopping is on.
`CMD_GET_HOPTRACE` costs one 165-byte UART frame (≈ 14 ms at 115200 — worth knowing, because that
write occupies the same task the hop branch lives in, so **do not poll the trace during the window
you are measuring**).

## PHY selection — `CMD_SET_PHY` (0x1D)

**Modulation is a knob cognition actuates, like MCS or spreading factor — not a property of the
node.** The SX1276 is a three-PHY radio in register form: `RegOpMode` bit 7 `LongRangeMode` selects
LoRa vs FSK/OOK, and `ModulationType` (bits 6:5, in FSK/OOK mode) picks FSK from OOK.

**`phy_bitmap` on this node is `0x0000_0001` — LoRa, one bit — and that is the honest number.** A
correct one-entry bitmap beats a fictional three-entry one: advertising FSK would promise a host a
PHY that has no packet engine here at all. `lora-phy` 3.0.1 is a LoRa-only driver, so FSK/OOK is not
a knob that is merely unexposed — it is a **second packet engine** with its own register page
(`RegBitrate` 0x02/0x03, `RegFdev` 0x04/0x05, `RegSyncConfig` 0x27, `RegPacketConfig1/2` 0x30/0x31,
a FIFO-threshold-driven TX/RX loop instead of a one-shot FIFO, and a different DIO map) that would
have to be written from nothing. The **selection machinery is implemented**; the second PHY is a
pass of its own, and what it would take is written down beside the constants in `src/main.rs`.

Switching between LoRa and FSK/OOK additionally requires the chip to be in **SLEEP** (`LongRangeMode`
is only writable there), so a real transition is `set_sleep → rewrite RegOpMode → re-init the new
engine` — which is why `CMD_SET_PHY` returns a **whole new `EVT_CAP`** rather than a patch. Selecting
the PHY already in effect is therefore a *verify*, not a re-init: read `RegOpMode`, check bit 7.

Two answers, and the split is the contract's:

* the requested PHY is **not** in `phy_bitmap` → `EVT_UNSUPPORTED [0x1D, OUT_OF_RANGE]`. The host
  asked for something this node never claimed; the fleet already has vocabulary for that.
* it **is** in the bitmap but the chip does not corroborate → `EVT_PHY_ERR (0x8D) [phy, RegOpMode]`,
  carrying the chip's literal status byte. That path is live here rather than decorative: a chip that
  has lost LoRa mode, or an SPI bus that has stopped answering, is reported instead of being handed a
  cheerful `EVT_CAP`.

## Scheduled TX — the relative opcode's ceiling, and the absolute one

`sched_gran_ns` = 99 µs is a **firmware-side** figure — the distance between the deadline the
firmware holds and the instant the carrier comes up — and it is identical on `CMD_TX_AT` (0x18) and
`CMD_TX_AT_ABS` (0x1F): both leave `Radio::fire_tx` one 2-byte `RegOpMode ← TX` write, and
`report_sched_error` measures both against the same counter. What differs is **where the deadline
comes from**.

| | 0x18 `TX_AT [delay_us]` | 0x1F `TX_AT_ABS [target_ticks]` |
|---|---|---|
| deadline | `Instant::now() + delay`, evaluated when the **firmware** dequeues the command | a point on **this node's** clock, the one `CMD_READ_CLOCK` returns and `EVT_RX.ts` stamps with |
| host→node serial latency | lands **directly in the host's placement**, and the firmware cannot see it — so it reports a small error for a frame that went out in the wrong place | decides only whether the arm **arrives** in time; it cannot move the frame |

This is measured, not argued. On the LR2021 node an absolute-boundary slot train fired **45/45**
with a mean inter-frame gap of 2 399 818 ticks against a 2 400 000 nominal — **within 11 µs over 44
slots**, so placement *accuracy* is excellent — but per-slot **sd 553 µs / p2p 1875 µs** against a
declared 50 µs, matching that node's 550 µs `CMD_GET_INFO` p2p round trip exactly. As exercised,
host-armed relative scheduling was *worse* than the software path (sd 553 vs 155 µs) because it pays
an extra round trip.

On **this** node the same term is a `CMD_GET_INFO` round-trip floor with a **MEASURED mean of
10 733 µs** — over 100× the 99 µs firmware granularity. That figure is reported in the
`CMD_SET_DEBUG` dump (`rtt_mean=10733us`) and deliberately **not** in `EVT_CAP`, whose v3 layout is
fixed and whose `sched_gran_ns` means the same thing on every node.

⚠ **Honesty about that number:** only the **mean** was measured on this node, so it is used as a
bound on the *typical* one-way transit (conservative, since the round trip contains it twice plus the
firmware's own reply work). **No p99, sd or peak-to-peak is claimed here** — bounding the spread needs
the same slot-train harness that produced the LR2021's sd 553 µs, and it has not been run on the
Heltec.

## The 2026-08-28 H pass (H1–H5) — hop-event timestamping

| | Before | After |
|---|---|---|
| **H1** | a hop advanced `Hop::hops` and nothing else; **when** it happened was unrecoverable | `Instant::now()` as the **first statement of `service_hop`**, ahead of that function's ~32 µs of SPI byte time, on the same counter `EVT_RX.ts` and `CMD_READ_CLOCK` use. What is still between it and the RF boundary is written down term by term and marked measured / not measured — see the section above; **nothing unmeasured is folded into the stamp** |
| **H2** | — | **`CMD_GET_HOPTRACE` 0x20 → `EVT_HOPTRACE` 0x8E.** 32-entry free-running ring, most recent last, reading does not clear it, ticks on the wire with `stamp_hz` beside them so the host — not the firmware — does the unit conversion |
| **H3** | `service_hop` awaited as a bare statement at both call sites, never a `select` branch | **unchanged, structurally.** The two additions (`Instant::now()`, `HopTrace::push`) contain no `await`, so the future's suspension points are byte-for-byte what they were |
| **H4** | `hops` mixed a frame's own intra-packet hops with the idle-RX hops around it, because in FHSS the receiver hops too | bit 7 of `idx` = **taken while transmitting**, set from the call site (`fire_tx` vs the main loop), no extra register read. The first TX-flagged entry after a run of unflagged ones is a frame's first intra-packet hop |
| **H5** | `CMD_BITMAP` was the whole self-description | the u32 is **full at 0x1F** and 0x20 is bit 32, so `CMD_BITMAP_EXT` carries opcodes 32..63 and `EVT_CAP`'s frozen 34-byte layout is **not** widened (that would desynchronise this node from the LR2021 it exists to be compared with). The dispatcher's catch-all oracle now spans both words; a sixth compile-time table pins the wire arithmetic, the 165-byte payload bound and the flag/index bit split |

## The 2026-08-28 v3 pass (tasks C1–C5)

⚠ Two label sets share the letter C in this directory: the **bug** labels C1–C7 in the section below
(2026-08-28 correctness pass) and the **task** labels C1–C5 here. Where the source says "C3
discipline" it means the *bug* C3 — the cancel-safe restructuring.

| | Was | Now |
|-|-----|-----|
| **★ C1** | no hopping of any kind; `DataPlane::hop_channel` had no substrate under it, and `lora-phy` exposes no hopping API | **`CMD_SET_HOP` 0x1E — real SX1276 intra-packet FHSS**, reaching past `lora-phy` to `RegHopPeriod` / `RegFrf` / `RegHopChannel`, with **DIO1 (GPIO35)** mapped to `FhssChangeChannel` and a ~32 µs, arithmetic-free service path that stays inside the C3 cancel-safety discipline. Full section above |
| **C2** | modulation was an identity: `EVT_CAP.radio_kind` encoded "LR2021-FLRC" and "LR2021-LoRa" as different *kinds* | **`CMD_SET_PHY` 0x1D + `phy_bitmap`.** `radio_kind` now names the PART; `EVT_CAP` describes the CURRENT PHY and is replaced wholesale on a switch. This node advertises **LoRa only** — one honest bit — with FSK/OOK's real cost written down beside it, and gains `EVT_PHY_ERR` 0x8D |
| **C3** | `CMD_TX_AT`'s delay is counted from when the **firmware** processes the arm, so the host's serial latency lands in the placement — MEASURED sd 553 µs / p2p 1875 µs against a declared 50 µs on the LR2021 | **`CMD_TX_AT_ABS` 0x1F**, a target on this node's own clock at full 64-bit width. Past → fires now, unclamped, so the real lateness is reported; > 60 s ahead → OUT_OF_RANGE |
| **C4** | — | **the 5 597 µs retune is unchanged.** Everything C1–C3 adds is gated on hopping being enabled: with it off, **zero** extra SPI transactions and **zero** extra UART bytes reach `CMD_SET_FREQ`, and the only addition is one predicated branch. With hopping **on**, the cost is three SPI transactions (6 bytes ≈ 24 µs of byte time at 2 MHz, ~30 µs allowing NSS framing at the firmware's own 5 µs/byte slack rate) — and they are not in the `CMD_SET_FREQ` handler at all but in the `arm_rx` that follows, which was already programming the carrier. ≈ 0.5% of 5 597 µs, and derived, not measured |
| **C5** | `CMD_BITMAP` = 0x1DBF_FFFE; three compile-time tables | `CMD_BITMAP` = **0xFDBF_FFFE** (0x1D/0x1E/0x1F set), and (with the H pass below) **six** compile-time tables: bandwidth, the command bitmap + scheduling arithmetic, the PHY surface, the hop table, the hop-trace table (`main.rs`), and the detector/LNA + FHSS register encodings (`regs.rs`). `#[cfg(test)]` still cannot run on this `no_std`/`no_main` xtensa target |

## The 2026-08-28 correctness pass (bugs C1–C7)

| | Was | Now |
|-|-----|-----|
| **C1** | Host sent SX1262 codes 0x04/0x05/0x06; firmware decoded 1/2 ⇒ **every** bandwidth fell through to 125 kHz, so `set_bandwidth_khz` was a silent no-op and 250/500 kHz were unreachable | one canonical code space, both spaces decoded, pinned by a compile-time table |
| **C2** | `lora-phy`'s `cad()` returns `InvalidRadioMode` unless `prepare_for_cad` has run — and only `prepare_for_cad` remaps DIO0 from RxDone to **CadDone**. The loop armed `RxMode::Continuous` every iteration, so CAD failed instantly, `CMD_CAD` always answered "clear" and `CMD_TX_LBT` took its "error ⇒ treat as clear" branch on the first attempt. The Heltec's half of the N=3 LBT experiment measured nothing | full CAD sequence, with an SF/BW-scaled watchdog instead of the old fixed 60 ms (shorter than one SF12 symbol pair) |
| **C3** | `select(read_frame(..), lora.rx(..))` cancelled the RX future mid-flow, against `lora-phy`'s explicit warning | driven through the `RadioKind` trait so the loop splits at the one cancel-safe seam — see `struct Radio`'s doc comment |
| **C4** | `EVT_INFO` was a zeroed array with sync hardcoded 0x12; `CMD_GET_STATS` replied `[0u8; 24]` "so the host does not time out" | both real; the two fields the SX1276 cannot supply are named above |
| **C5** | `if let Ok(n) = rx.read_async(..)` discarded every `Err`, `FifoOverflowed` included — a dropped byte desynced a command silently | a dedicated UART reader task counts every error into `EVT_INFO.lost`, and (being independent of the radio task) keeps taking host commands off the wire during a multi-second SF12 transmission |
| **C6** | `8 + attempt*6` ms, no PRNG ⇒ two Heltecs backed off identically and LBT could not separate them | xorshift32 seeded from the ESP32 RNG **and** the chip-unique eFuse MAC (the MAC is what guarantees inter-node difference; the ESP32 RNG is only a certified TRNG with the RF subsystem running, which this firmware does not start) |
| **C7** | the flashed `.bin` was untracked and older than the last source commit | `build.rs` stamps the git identity; the firmware reports it; build/flash commands documented above |

One further bug found while fixing those: `UartTx::write_async` returns how many bytes fitted in the
128-byte TX FIFO and the old `send_frame` ignored it, so **any event longer than the free FIFO space
was silently truncated on the wire** — a 247-byte `EVT_RX` would lose its tail. `write_all` now loops.

## The 2026-08-28 parity pass (P1–P6)

The v2 survey left four opcodes unimplemented on this node. Three of them were possible after all;
one really is not, and now says so from a single explicit arm rather than a catch-all.

| | Was | Now |
|-|-----|-----|
| **P1** | `CMD_TX_AT` → `EVT_UNSUPPORTED`/NO_HARDWARE, `sched_gran_ns = 0`, on the reasoning that "an embassy timer fires the SPI write whenever the executor gets round to it" | **implemented.** That reasoning was right about running the *whole* transmit from the deadline — ~30 register transactions plus a payload FIFO burst, over 2 ms and payload-length dependent — and wrong about what has to run there. `Radio::stage_tx` programs the chip 1.5–2.7 ms ahead; `Radio::fire_tx` is left one 2-byte `RegOpMode ← TX` write. `sched_gran_ns` = **99 µs**, summed from source constants (see the `EVT_CAP` table) |
| **P2** | `CMD_SET_CAD_CFG` → `EVT_UNSUPPORTED`/NO_HARDWARE, "the SX1276 has no `SetCadParams` equivalent" | **implemented.** True of the *command set*, false of the *hardware*: `RegDetectOptimize` (0x31) and `RegDetectionThreshold` (0x37) are the detector, and `lora-phy` merely leaves them unreachable. Two of the three fleet fields map; `sym` has no counterpart and is ignored **in writing** rather than given an invented mapping |
| **P3** | `CMD_SET_RX_GAIN` not implemented (bit clear) | **implemented** via `RegLna` (0x0C), with the Waveshare node's boolean payload. Re-applied after every arm, because `lora-phy` rewrites that register from its fixed `rx_boost` flag on each `do_rx` and `do_cad` — a write-once version would have been a decoration |
| **P4** | `CMD_ENTER_BOOTLOADER` → `EVT_UNSUPPORTED` | unchanged, and deliberately so: the CP2102 drives `EN`/`GPIO0` from DTR/RTS, so the ESP32 ROM downloader comes up in hardware and there is nothing for firmware to do. It is an explicit match arm carrying that reason, not a fall-through |
| **P5** | — | the 5.6 ms retune is documented and guarded at the `CMD_SET_FREQ` handler. Nothing in P1–P3 lengthens that path: `apply_detector` and `apply_lna` issue **zero** SPI transactions until the host has actually set those knobs |
| **P6** | one compile-time table (bandwidth) | three: bandwidth, the command bitmap + scheduling arithmetic (`main.rs`), and the detector/LNA encodings (`regs.rs`). `#[cfg(test)]` still cannot run on this `no_std`/`no_main` xtensa target, so `const _: () = { assert!(…) }` remains the mechanism. (The v3 pass took it to five — see task C5 above) |

### Cancel safety — how P1 stays inside the C3 discipline

`lora-phy` warns that cancelling `process_irq_event` mid-flow can lock the radio up, and C3
restructured the main loop so that the only cancellable await is one that touches neither SPI nor
driver state. P1 does not weaken that:

* `stage_tx` and `fire_tx` are the only SPI-touching futures P1 adds, and both are awaited as **bare
  statements** inside the timer arm, *after* the `select` has already resolved. Neither is ever a
  `select` branch.
* What the deadline timer races is `CMDQ.receive()` (a channel) and — only while nothing is staged —
  `Radio::wait_irq` (a bare GPIO level wait). While a frame **is** staged the loop drops to a
  two-way `select` that omits the DIO0 branch entirely, so `process_irq_event` can never be entered
  with the mode mirror reading `Transmit` while the chip idles in standby. The branch is deleted,
  not guarded — there is nothing left to get a guard wrong about.
* A host command arriving on a staged frame demotes it to `Pending` via `abandon_staged_tx`, which
  performs **no SPI at all** (the chip really is in standby, so only the mirror needs correcting).
* `fire_tx`'s TxDone wait races the timeout against `await_irq` **only**, never against the SPI
  phase — the same shape `transmit` already used, which `transmit` now reaches by calling it.

## Measured on air (2026-08-28)

Knob round-trip latency, host command to `EVT_INFO`, n = 8–10 each, alternating values so nothing
was a no-op:

| | SET_FREQ (retune) | SET_MOD | SET_PWR | GET_INFO floor |
|---|---|---|---|---|
| **Heltec SX1276 (this node)** | **5 597 µs** | 5 394 | 4 755 | 10 733 |
| LR2021 FLRC | 52 798 µs | 55 015 | 51 394 | 16 714 |
| Waveshare SX1262 | 160 866 µs / 160 150 µs on two independent dongles | 82 778 | 82 679 | 4 758 |

★ **This node retunes 29× faster than the Waveshare.** That makes it the only node in the fleet that
can plausibly do name-keyed frequency hopping at a useful dwell — which is why `CMD_SET_FREQ` carries
a warning comment, and why P1–P3 were written to add nothing to that path.

## Not yet measured

Compile-verified and reasoned from the datasheet, the `lora-phy` source and the Waveshare reference
node, but **not** confirmed on air on this board:

* **★ Everything about C1's hopping.** The registers are written, the interrupt is mapped and
  unmasked, and the service path is bounded — but **no hop has been observed on air from this
  firmware.** Three things to check first, in order: (1) does `hops` in the `CMD_SET_DEBUG` dump
  advance at all once `CMD_SET_HOP 1` is sent (i.e. does DIO1 actually assert)? (2) is `pll_to`
  zero — if not, `hop_period` is too short for the SF/BW in use; (3) do two nodes on the same list
  and the same `hop_period` still exchange frames, which is the only test of the index semantics.
  ⚠ `(RegHopChannel & 0x3F)` is used as the list index because that is the **Semtech reference
  driver's own pattern**, not because it has been measured here. If the pair walks the list out of
  step, that mapping is the first suspect.
* **★ `EVT_HOPTRACE`'s offset from the true RF hop instant.** The stamp is taken as early as this
  part allows (first statement of `service_hop`, before any SPI), but **two terms of the path are
  unmeasured and are declared unknown rather than budgeted**: the SX1276's RF-boundary → DIO1 delay,
  and the DIO1 edge → GPIO interrupt → esp-hal waker → embassy executor → task-resume path. A third,
  the main task being busy in a bare SPI/UART statement when DIO1 asserts, is variable and can reach
  milliseconds. Nothing is folded into the timestamp to cover any of them. The constant terms cancel
  in a difference, so the **interval** between consecutive entries is the number to read first, and
  the spread of those intervals over a quiet run is itself the bound on the variable term — that is
  the first measurement to take with this instrument, before any absolute claim. Expected at
  SF7/BW125 with `hop_period = 8`: **8 192 ticks** between entries.
* **`SERIAL_RTT_MEAN_US` as a spread bound.** The 10 733 µs is a measured **mean**; the spread was
  not measured on this node, and none is claimed. Run the slot-train harness against `CMD_TX_AT_ABS`
  and `CMD_TX_AT` back to back — that comparison is the whole point of C3 and it is exactly the
  measurement this pass could not take.
* **C4's hopping-enabled retune cost (~30 µs).** Derived from the SPI byte count at 2 MHz, not
  timed. The retune itself (5 597 µs, hopping off) *is* measured.
* **`EVT_PHY_ERR`'s failure path.** The refusal branch is reachable in principle (a chip not in LoRa
  mode) but has never fired; only the success path — `CMD_SET_PHY 0` → `EVT_CAP` — is exercisable
  without breaking something on purpose.

* **`sched_gran_ns` = 99 µs.** The 50 µs of it that covers the executor wake and NSS framing is a
  budget, not a measurement. The firmware already computes its own error against the same counter
  and emits `EVT_LOG` `sched err=<signed µs> gran=99us late=<n>` on every miss — so the first
  bring-up test is simply: schedule a series of frames and see whether any `sched err` line appears.
  Replace `SCHED_SLACK_NS` with a measured p99 once it has.
* **P2's effect on CAD sensitivity.** The registers are written and re-applied (verifiable from the
  `CMD_SET_DEBUG` dump line `knobs det=0x../0x..`), but no detection-rate curve has been taken
  against a real transmitter, so no claim is made about what the knob buys.
* **P3's effect on receive sensitivity.** `RegLna` is written; the ~+3 dB the datasheet attributes to
  LnaBoostHf has not been measured here.
* Anything about SF6, the LF port, or 250/500 kHz CAD behaviour.
