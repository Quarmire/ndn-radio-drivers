//! Stamps a **build identifier** into the firmware (bug C7).
//!
//! Before this, `heltec-lora-rs.bin` was an untracked blob older than the last source commit, so no
//! on-air result from this node was attributable to a known build: you could not tell whether the
//! chip was running the code in the tree or something from weeks earlier. The firmware now reports
//! `BUILD_ID` in an `EVT_LOG` at boot and again before every `EVT_INFO` reply to `CMD_GET_INFO`, so
//! the host can log exactly which build produced a measurement.
//!
//! Shape: `<git-short-sha>[+dirty]` — or `nogit` when the tree is not a checkout / git is absent.
//! `rerun-if-changed` on `.git/HEAD` and the index keeps it fresh without forcing a rebuild on every
//! `cargo build`.

use std::process::Command;

fn main() {
    let sha = Command::new("git")
        .args(["rev-parse", "--short=9", "HEAD"])
        .output()
        .ok()
        .filter(|o| o.status.success())
        .map(|o| String::from_utf8_lossy(&o.stdout).trim().to_string())
        .filter(|s| !s.is_empty())
        .unwrap_or_else(|| "nogit".to_string());

    // `--porcelain` prints one line per modified path; any output means the working tree differs
    // from the commit, so the sha alone would NOT identify what was built.
    let dirty = Command::new("git")
        .args(["status", "--porcelain", "--untracked-files=no"])
        .output()
        .ok()
        .filter(|o| o.status.success())
        .map(|o| !o.stdout.is_empty())
        .unwrap_or(false);

    println!(
        "cargo:rustc-env=HELTEC_BUILD_ID={}{}",
        sha,
        if dirty { "+dirty" } else { "" }
    );
    println!("cargo:rerun-if-changed=../../.git/HEAD");
    println!("cargo:rerun-if-changed=../../.git/index");
    println!("cargo:rerun-if-changed=build.rs");
}
