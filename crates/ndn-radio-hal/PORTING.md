# Porting the named-radio stack to another NDN forwarder

The radio work (HAL + frame I/O + MAC + PHY faces) is designed to plug into **any** NDN
forwarding stack, not just the Rust `ndn-rs` one. This document records the interface
contract so the boundary stays clean.

## The interface (what a host NDN stack must provide)

The radio stack couples to a host NDN stack through **four narrow crates only**:

| Crate | Role at the boundary |
|---|---|
| `ndn-transport` | the **face trait** — how the forwarder sends/receives on a face |
| `ndn-packet`    | NDN packet types (Name/Interest/Data) — for **parsing the name** (the relevance floor, see NDR_MAC_SPEC.md §6) |
| `ndn-tlv`       | the TLV codec used to parse/emit names and MAC sublayer TLVs |
| `ndn-time`      | a monotonic time source |

To port to another stack, provide equivalents of these (a face/transport trait, a
TLV/name parser, a clock). Nothing in the radio stack reaches into the forwarder engine,
app layer, security, storage, strategy, sync, or management crates.

## Verified boundary (audit 2026)

These crates depend **only** on the interface above (+ peer radio crates), no forwarder internals:

- `ndn-radio-hal`  — `ndn-transport`, `ndn-time`
- `ndn-frame-io`   — `ndn-radio-hal`, `ndn-transport`, `ndn-time`
- `ndn-radio-drivers` (lib) — `ndn-radio-hal`, `ndn-frame-io`, `ndn-time`, `ndn-transport`
- `ndn-radio` (MAC / WirelessFace) — `ndn-transport`, `ndn-packet`, `ndn-tlv`, `ndn-time`, `ndn-radio-hal`, `ndn-frame-io`
- `ndn-phy-wifi`, `ndn-phy-lora`, `ndn-phy-ble` — narrow interface + peer radio crates

Forwarder-engine coupling in the MAC crate (`ndn-engine`, `ndn-app`, `ndn-security`) is
confined to **dev-dependencies** (the engine-roundtrip test + the NAC example); the library
does not use them.

## Known coupling to keep an eye on

- **`ndn-coding`** is a portable codec core. Its forwarder integrations are feature-gated +
  `optional` (`endpoint`->`ndn-app`, `mgmt`->`ndn-mgmt`, `f2-recode-face`->`ndn-engine`/`ndn-security`),
  and as of 2026 the codec is decoupled by default:
  1. **`default = []`** — the lean FEC/codec core is the default; the native forwarder
     integrations are opt-in (`endpoint`, `mgmt`). In-workspace consumers that need them
     (ndn-ext/ndn-mobile/ndn-fwd) request them explicitly.
  2. **`ndn-store` removed** — `policy.rs`'s prefix table is now a local dependency-free
     `NameTrie` (`src/prefix_table.rs`), so the codec core carries no forwarder-stack edge at all.

## Rule going forward

A new dependency from any radio crate onto a forwarder-internal `ndn-*` crate (engine/app/
security/store/mgmt/sync/strategy/...) in **non-dev** dependencies is a decoupling regression.
Keep it in `[dev-dependencies]` or behind an `optional` feature.
