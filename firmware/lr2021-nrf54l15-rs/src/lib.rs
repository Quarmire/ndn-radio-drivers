//! Shared support for the LR2021 + nRF54L15 MAC testbed binaries.
//!
//! A lib target exists purely so `src/main.rs` and the `src/bin/*.rs` milestone binaries share one
//! pin map, one radio bring-up and one link configuration. Two nodes that disagree about the
//! frequency, syncword or packet format do not fail loudly — they simply never hear each other,
//! which is the same symptom as a broken radio. Keeping the configuration in exactly one place is
//! what makes "no packets received" mean something.
//!
//! See `src/main.rs` for the milestone table and the crate README for status.

// `no_std` for the device; plain `std` under `cargo test`, so the pure-logic modules below have a
// runnable test harness. See the module split under it.
#![cfg_attr(not(test), no_std)]

// ── Pure logic: no MCU, no radio, no allocator. Compiles for ANY target ──────────────────────────
//
// These are the modules whose correctness is a *wire contract* with other implementations — the
// Tier-0 filter must be byte-identical to the host and the ath9k-htc C port, and the 7E-A5 parser
// must be byte-identical to three other firmwares — so they are also the modules that most need
// tests, and they are the ones that could not have any: the crate's dependencies only build for
// `thumbv8m.main-none-eabihf`, so `cargo test` could not even start.
//
// The fix is in `Cargo.toml`: every embedded dependency now lives under
// `[target.'cfg(target_os = "none")'.dependencies]`, and the modules that need them are gated to
// match. On the device nothing changes — the default target *is* `target_os = "none"`. On the host,
//
//     cargo test --target <host-triple> --lib
//
// builds just these four modules and runs their tests.
pub mod board;
pub mod gcs;
pub mod serial;
pub mod tier0;

// ── The embedded half: embassy-nrf + the LR2021 driver, device target only ───────────────────────
#[cfg(target_os = "none")]
pub mod flrc_link;
#[cfg(target_os = "none")]
pub mod hw;
#[cfg(target_os = "none")]
pub mod timing;

