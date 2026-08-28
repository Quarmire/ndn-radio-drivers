# heltec-lora-rs

Open **Rust** firmware for the **Heltec WiFi LoRa 32 V2** (ESP32 Xtensa LX6 + **SX1276**), the third
named-radio node (node C, task #54). It speaks the **7E-A5 v2** serial protocol at feature parity
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

in the form `heltec-lora-rs build=<short-sha>[+dirty] proto=2 stamp_hz=1000000`. A `+dirty` suffix
means the working tree differed from the commit, so the sha alone does not identify what was built.

## Hardware map (Heltec WiFi LoRa 32 V2)

| Function | Pin | Notes |
|----------|-----|-------|
| Host UART | UART0 — TX GPIO1 / RX GPIO3 | 115200 8N1 → CP2102 → `/dev/ttyUSB*` (Linux) / `/dev/cu.usbserial-*` (macOS) |
| SX1276 SPI | SPI2 — SCK GPIO5, MISO GPIO19, MOSI GPIO27 | 2 MHz, mode 0 |
| SX1276 NSS | GPIO18 | driven by the firmware (see `src/regs.rs`) |
| SX1276 RESET | GPIO14 | |
| SX1276 DIO0 | GPIO26 | the ONLY interrupt line wired to `lora-phy`; RxDone / TxDone / CadDone, remapped per mode |
| PA | PA_BOOST | the antenna is on PA_BOOST, not RFO → real TX range **+2 … +20 dBm** |

UART0 is owned **raw** for the binary protocol; nothing prints text on it in normal operation, or the
framing desyncs. `esp-backtrace` still prints on panic, when the link is lost anyway.

## Host ⇄ firmware protocol (7E-A5 v2)

Framing `7E A5 | type | len | payload[len] | xor-crc`, crc = XOR of type, len and payload. The parser
resyncs on `7E A5` and validates the crc, so a dropped byte costs at most one frame.

Every opcode this node does not implement is answered **`EVT_UNSUPPORTED` (0x8F) `[cmd, reason]`** —
never silence, never a fake success. Reasons: `0x01` unknown opcode, `0x02` no hardware, `0x03` bad
length.

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
| 0x0A | SET_CAD_CFG | `[sym, det_peak, det_min]` | **`EVT_UNSUPPORTED`/NO_HARDWARE** — the SX1276 has no equivalent of the SX1262's `SetCadParams` (0x88); its CAD window and detector are fixed by the modem's SF-derived settings. The firmware half of the same knob (`cad_repeat`) IS supported, via 0x12 |
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
| 0x18 | TX_AT | `[delay_us u32 BE][frame]` | **`EVT_UNSUPPORTED`/NO_HARDWARE** — no scheduled-TX engine: the SX1276 has no TSF comparator and no delayed key-up, and an embassy timer firing the SPI write is exactly the jitter TX_AT exists to remove |
| 0x1A | GET_CAP | *(empty)* | yes → `EVT_CAP` |
| 0x1B | SENSE | *(empty)* | yes → `EVT_SENSE` |

### Firmware → host

`EVT_RX` 0x81 `[rssi i16 BE, snr i16 BE, ts u32 BE, LoRa bytes]` (`rssi`/`snr` from the SX1276's
per-packet `RegPktRssiValue`/`RegPktSnrValue`; `ts` in `EVT_CAP.stamp_hz` units) ·
`EVT_TXDONE` 0x82 `[ok, attempts]` · `EVT_INFO` 0x83 (19 B, fixed) · `EVT_LOG` 0x84 ·
`EVT_CAD` 0x85 · `EVT_RSSI` 0x86 · `EVT_SF_DETECTED` 0x87 · `EVT_TX_STARTED` 0x88 ·
`EVT_STATS` 0x89 (24 B) · `EVT_CLOCK` 0x8A · `EVT_CAP` 0x8B (29 B) · `EVT_SENSE` 0x8C ·
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

## What this node reports about itself — `EVT_CAP` (0x8B, 29 B)

Every field is a source constant or a verified property of this build. Where nothing is knowable the
field is 0 and says so, because a fabricated number is worse than 0 — the host believes it.

| Bytes | Field | Value | Where it comes from |
|-------|-------|-------|---------------------|
| 0 | proto_ver | 2 | |
| 1 | radio_kind | 1 | SX1276 |
| 2..6 | freq_min_hz | 902 000 000 | the band this **board** is matched for. NOT the SX1276's 137–1020 MHz silicon range: this is the 915 MHz Heltec variant, whose PA matching, SAW filter and antenna are tuned for US 902–928, and advertising the silicon range would invite a tune where the board radiates almost nothing |
| 6..10 | freq_max_hz | 928 000 000 | as above |
| 10 | pwr_min_dbm | +2 | real dBm — the PA_BOOST interval `lora_phy::sx127x::sx1276::set_tx_power` clamps to. Never a register unit |
| 11 | pwr_max_dbm | +20 | as above (above +17 dBm the driver enables PaDac 20 dBm and raises OCP to 240 mA) |
| 12..16 | stamp_hz | 1 000 000 | `embassy_time::TICK_HZ`, a compile-time constant of the driver actually linked (esp-rtos selects `tick-hz-1_000_000`) — verified against this build, not assumed |
| 16 | stamp_kind | 2 | **software counter**: the MCU reads its own monotonic clock when the DIO0 interrupt wakes it. The SX1276 has no RX-time capture register, so a hardware stamp is impossible on this radio |
| 17..19 | max_payload | 247 | the REAL end-to-end cap, set by the serial framing, not the radio: an event's `len` is one byte, so an `EVT_RX` payload is ≤ 255 B of which 8 are the rssi/snr/ts header. The FIFO (256 B), the LoRa PDU (255 B) and `CMD_TX`'s accept (255 B) are all larger, so 247 binds. A frame larger than this is **counted and dropped**, never truncated into something that looks complete |
| 19..23 | cmd_bitmap | 0x0CBF_FBFE | bit N set ⇔ opcode N is implemented. It is also the dispatcher's own "known opcode?" oracle, so the bitmap and the command handler cannot drift apart |
| 23 | sf_min | 7 | SF6 needs an implicit header and would break fleet interop |
| 24 | sf_max | 12 | |
| 25..29 | sched_gran_ns | 0 | no scheduled-TX engine exists on this radio (see `CMD_TX_AT`); 0 is the truth, not a placeholder |

Exact payload:

```
02 01 35 C3 6D 80 37 50 28 00 02 14 00 0F 42 40 02 00 F7 0C BF FB FE 07 0C 00 00 00 00
```

on the wire (with framing and crc):

```
7E A5 8B 1D 02 01 35 C3 6D 80 37 50 28 00 02 14 00 0F 42 40 02 00 F7 0C BF FB FE 07 0C 00 00 00 00 92
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

## The 2026-08-28 correctness pass (C1–C7)

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

## Not yet exercised on air

The node was unreachable when this firmware was written (`minidronesys-05-o5p-2`'s sshd was failing
on every connection, so the board could not be flashed). **Everything above is compile-verified and
reasoned from the datasheet, the `lora-phy` source and the Waveshare reference node; none of it has
been confirmed on air on this board.** First bring-up should check, in order: `CMD_GET_CAP` returns
the exact bytes listed above; `CMD_GET_INFO`'s `EVT_LOG` build id matches the flashed tree;
`CMD_SET_MOD` with bw 1 and 2 changes `EVT_INFO.bw` *and* the measured airtime; `CMD_CAD` reports
busy while a Waveshare transmits and clear when it does not.
