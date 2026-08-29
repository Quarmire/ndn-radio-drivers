//! Host-testable half of the Waveshare USB-LoRa firmware.
//!
//! The firmware itself is `src/main.rs`, a `no_main` binary. This lib target exists for one reason:
//! **the RX-timestamp arithmetic and the `stamp_kind` byte are contracts, and contracts need tests
//! that can run.** `EVT_CAP[16]` is what makes the host publish `LatchPoint::RadioCapture` and set
//! `can_common_view`; the wrap extension is what keeps a 16-bit counter from silently aliasing a
//! timestamp by 65.536 ms; the attribution rule is what stops a plausible timestamp being reported
//! against the wrong frame. None of those needs a GD32 to check, and none of them could be checked
//! at all while every dependency in the crate only built for `thumbv7m-none-eabi`.
//!
//! The fix is the same one `lr2021-nrf54l15-rs` uses: the embedded dependencies live under
//! `[target.'cfg(target_os = "none")'.dependencies]` in `Cargo.toml`, so on the device nothing
//! changes (the pinned target *is* `target_os = "none"`), and on the host
//!
//! ```text
//! cargo test --target <host-triple> --lib
//! ```
//!
//! builds just [`capture`] and runs its tests. `--lib` is load-bearing: the binary is `no_main` and
//! cannot build for a hosted target.
//!
//! Everything that touches a register stays in the binary — `rxstamp` (TIM3 input capture),
//! `sx1262`, `ndn` — because there is nothing about them a host test could establish.

// `no_std` on the device; plain `std` under `cargo test` so the tests have a harness.
#![cfg_attr(not(test), no_std)]

pub mod capture;
