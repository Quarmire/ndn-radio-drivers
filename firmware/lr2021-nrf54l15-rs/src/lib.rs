//! Shared support for the LR2021 + nRF54L15 MAC testbed binaries.
//!
//! A lib target exists purely so `src/main.rs` and the `src/bin/*.rs` milestone binaries share one
//! pin map, one radio bring-up and one link configuration. Two nodes that disagree about the
//! frequency, syncword or packet format do not fail loudly — they simply never hear each other,
//! which is the same symptom as a broken radio. Keeping the configuration in exactly one place is
//! what makes "no packets received" mean something.
//!
//! See `src/main.rs` for the milestone table and the crate README for status.

#![no_std]

pub mod board;
pub mod flrc_link;
pub mod hw;
pub mod serial;
pub mod timing;

