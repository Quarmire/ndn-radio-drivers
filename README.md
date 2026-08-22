# ndn-radio-drivers

Userspace USB Wi-Fi monitor-mode and LoRa driver backends over the `ndn-radio-hal`
contract, plus the device firmware they talk to and — since P2/D5 — the radio
foundation crates themselves (`crates/`). Split out of `ndn-face-monitor-wifi`
(ndn-ext) so drivers have a dedicated home: each backend implements
`FrameIo` + `WifiRadio` (and `RadioKnobs` / `RadioTime` / `RadioProfile` where the
silicon allows) against the HAL and does **no NDN forwarding**.

Dependency direction across the workspace is a DAG: `ndn-rs <- ndn-radio-drivers <- ndn-ext`.
The repo expects the `ndn-rs` checkout as a sibling (`../ndn-rs`) for its
`ndn-time` / `ndn-transport` path deps.

## Layout

| Path | What it is |
|---|---|
| `src/` | The host-side driver backends, one module per chip (table below), plus the shared plumbing: `usb_select` (picking one dongle among identical ones, with a live-kernel-link guard), `rx_pump` (shared async-URB bulk-IN pipelining), `realtek_rx` (shared RX-descriptor decode), `coverage` (the backend trait-coverage table, #79 — every radio shows a full row or a written exclusion). |
| `crates/` | The radio foundation crates, moved here from ndn-rs `crates/core` in P2/D5 (history there up to `df454d63`): **ndn-radio-hal** (the data-plane radio contract: `TxIntent`, MCS descriptors/policy, the `FrameIo`/`WifiRadio` traits — pure types, no I/O), **ndn-frame-io** (backend-agnostic link-layer frame I/O: on-air framing per `FrameFormat`, AF_PACKET + loopback backends), **ndn-env** (the classified `NDN_*` environment surface, #81). |
| `firmware/` | Per-board device firmware, each an independent embedded sub-workspace with its own toolchain (table below). `build-bw16-rs.sh` builds and links the BW16 image. |
| `fw/` | Vendor firmware blobs and extracted BB/RF register tables (RTL8822E, RTL8821C, MT7612, RTL8733B, RTL8812AU). Byte-exact provenance for every blob: [`fw/README.md`](fw/README.md). |
| `examples/` | 40+ hardware probe / bring-up binaries. They need a radio on the USB bus to be useful and are deliberately **not** built in CI. BW16 and LoRa examples are feature-gated (`--features bw16` / `--features lora`). |
| `docs/` | Measured findings that outlived their campaign: EDCCA contention (#37), frame-free sensing (#30), the scalable LoRa MAC design (#52). |
| `golden/` | Captured golden state from the kernel drivers: rtw88 usbmon register traces per channel (the 8812au bring-up oracle) and `tier0/vectors.txt` (the shared Tier-0 filter vectors). |
| `tests/` | `tier0_golden.rs` — the three-way Tier-0 wire gate: the LR2021 firmware copy (mounted via `#[path]`, so the bytes that ship are the bytes tested) checked against the golden vectors ndn-ext's `tier0.rs` generates; the ath9k C copy is checked by `firmware/ath9k-htc-ndr/tools`. |
| `scripts/` | `supervise_tx.sh` — process-level TX supervisor for the RTL8731BU (its per-boot analog TX state is locked at process start, so reliable TX = relaunch until delivery is confirmed). |
| `tools/` | Gone — the one-shot RTL8733B RE tools (hwprobe, usbmon parser) were absorbed into `src/libusb_rtl8733b.rs` and archived; see [`ARCHIVE.md`](ARCHIVE.md). |
| `ARCHIVE.md` | Tombstones for everything removed from the working tree (one-shot RE probes, superseded firmware); git history is the archive. |

## Backends by chip (`src/`)

| Chip | Module | Notes |
|---|---|---|
| RTL8812EU / RTL8822E | `libusb_rtl88xx` | The original libusb port: firmware download, phydm BB/RF table load, calibration, monitor TX/RX. |
| RTL8821CU | `rtl8821c` | Full bring-up split by block (mac/phy/efuse/fw/pwrseq/coex); the one backend that natively reads `NDN_RADIO_TXPWR`. |
| MT7612U | `mt7612` | MediaTek path: init-table replay + async TX. |
| RTL8812AU | `rtl8812au` | Carries the sensing seams: EDCCA knobs, frame-free occupancy (`PhySense`), IQK. |
| RTL8731BU / RTL8733BU | `libusb_rtl8733b` | Ground-up halmac_87xx (1x1 11ac) port, at M1: open + register I/O + chip version. |
| AR9271 | `ath9k_htc` | The one Wi-Fi part whose *firmware* is ours (see `firmware/ath9k-htc-ndr`). Host side at L1: USB transport + firmware download + HTC handshake + WMI. Does not yet replace kernel `ath9k_htc`. |
| BW16 (RTL8720DN) | `bw16_serial` (feature `bw16`) | Dual-band 802.11 injector/capturer bridged over USB-serial to the BW16 firmware; same HAL contract as the USB drivers. |
| Waveshare USB-TO-LoRa (SX1262) | `lora_serial` (feature `lora`) | Sub-GHz serial modem: transparent-mode byte pipe, host-supplied framing, AT-programmed radio params. |

## Firmware by board (`firmware/`)

The per-board READMEs are **milestone lab logs** — measured results with dates,
kept as records — not polished manuals. One-line status of each:

| Board | Dir | Status |
|---|---|---|
| AR9271 dongle (Xtensa) | `ath9k-htc-ndr` | Patches to `qca/open-ath9k-htc-firmware` putting the named-data-radio MAC on the dongle. M0 (behaviour-identical patched build), M2 (WMI memory access), M3 (hardware-scheduled TX: 97.7% of transmit opportunity removed by the MAC, no host in the loop) done; M1 (on-chip Tier-0 filter) and M4 (NAV) open. |
| Seeed XIAO nRF54L15 + LR2021 | `lr2021-nrf54l15-rs` | The custom-hardware named-radio MAC testbed (#102) — the only rig hardware that tests the MAC design rather than its commodity-Wi-Fi degradation. M3–M7a done: on-air FLRC link, 62.5 ns hardware RX timestamp, hardware-scheduled TX (58.9 µs guard band), fleet 7E-A5 protocol, Tier-0 filter FP curve measured (k=4). |
| Waveshare USB-TO-LoRa (GD32F103C8 + SX1262) | `waveshare-lora-rs` | Open Rust firmware replacing the closed factory firmware; speaks plain standard LoRa, interop verified bidirectionally against an SX1276 peer. |
| Heltec WiFi LoRa 32 V2 (ESP32 + SX1276) | `heltec-lora-rs` | Node-C modem (#54): host-driven over the same `7E A5` serial protocol as `waveshare-lora-rs`, so `LoraSerialBackend` drives it unchanged. |
| BW16 (RTL8720DN) | `bw16-rs` + `bw16-rs-sketch` | Rust `no_std` firmware core for the named-radio serial bridge, linked into the Ameba image by `build-bw16-rs.sh` via the C++ FFI shim in `bw16-rs-sketch` (which is load-bearing, not a duplicate). Knob reality (what inject can and cannot control) in `bw16-rs/FINDINGS.md`. |
| BW16 (RTL8720DN) | `bw16-ndn-bridge` | The self-contained Arduino sketch version of the same serial bridge (host counterpart: `Bw16SerialBackend`). |

## Building

```sh
cargo check                      # host crate + crates/ (needs libusb-1.0 + pkg-config, and ../ndn-rs)
cargo test --lib --test tier0_golden   # what CI runs; plain `cargo test` also builds every example
cargo check --features bw16,lora # include the serial-bridged backends
```

- **Examples** are hardware probes: they need the matching radio on the USB bus
  (and often root / detached kernel drivers). Build them explicitly; several
  require `--features bw16` or `--features lora`.
- **Firmware** builds are independent sub-workspaces with their own cross
  toolchains (thumbv8m / Xtensa / nRF54) — build inside each directory per its
  README; the BW16 image via `firmware/build-bw16-rs.sh`.
- **fw/ blobs** are extracted vendor firmware and register tables; do not edit —
  [`fw/README.md`](fw/README.md) records where every byte came from.

## Where the doctrine lives

This repo holds drivers, firmware and measured findings. The radio *design*
docs — why a named-data MAC, the filter/MAC redesign the firmware milestones
execute, the campaign chapters — live in ndn-ext next to the face that consumes
these drivers: `ndn-ext/crates/faces/ndn-face-monitor-wifi/docs/`. Start with
[`named-radio-primer.md`](../ndn-ext/crates/faces/ndn-face-monitor-wifi/docs/named-radio-primer.md);
`named-filter-mac-redesign.md` is the design document `ath9k-htc-ndr` and
`lr2021-nrf54l15-rs` are executing.

## CI

`.github/workflows/ci.yml`, with sibling checkouts (ndn-rs, ndn-ext) **pinned to
known-good SHAs** so a sibling push cannot redden this repo's gate:

- **clippy (lib + bins)** — plain `clippy`, not yet `-D warnings` (a known
  backlog of pre-existing lints is being burned down first).
- **host tests** — `cargo test --lib --test tier0_golden`: unit tests plus the
  three-way Tier-0 wire gate.
- Examples and `firmware/` are deliberately not built in CI (hardware probes and
  cross-toolchain sub-workspaces respectively).

## License

The root driver package is **MPL-2.0**. The foundation crates under `crates/`
came from ndn-rs and keep their original **MIT OR Apache-2.0** licensing
(`[workspace.package]` in the root manifest). Exceptions to note:
`firmware/bw16-rs-sketch` carries GPLv3 `packet-injection.{h,cpp}`;
`firmware/ath9k-htc-ndr` patches the QCA firmware tree under its upstream
license; `fw/` redistributes proprietary vendor firmware blobs in binary form
the same way linux-firmware does (provenance in [`fw/README.md`](fw/README.md)).
