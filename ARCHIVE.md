# ARCHIVE

Tombstones for items removed from the working tree. Git history is the archive:
recover any item with `git show <sha>:<path>` (or `git checkout <sha> -- <path>`),
where `<sha>` is the last commit that contained it.

| Path | ~LOC | Last SHA | Reason | Date |
|---|---|---|---|---|
| `examples/opi_abtest.rs` | 58 | a394c7f | One-shot Orange-Pi RTL8733B/8812AU RE probe (July campaign); findings absorbed into src/libusb_rtl8733b.rs | 2026-08-21 |
| `examples/opi_cca.rs` | 34 | e330b9c | One-shot Orange-Pi RE probe (July campaign) | 2026-08-21 |
| `examples/opi_datapath.rs` | 56 | f271f56 | One-shot Orange-Pi RE probe (July campaign) | 2026-08-21 |
| `examples/opi_edcca.rs` | 29 | 4f768cf | One-shot Orange-Pi RE probe (July campaign) | 2026-08-21 |
| `examples/opi_efuse.rs` | 58 | d35e0a6 | One-shot Orange-Pi RE probe (July campaign) | 2026-08-21 |
| `examples/opi_fullcal.rs` | 65 | 19758a2 | One-shot Orange-Pi RE probe (July campaign) | 2026-08-21 |
| `examples/opi_fwtx.rs` | 50 | 08292c9 | One-shot Orange-Pi RE probe (July campaign) | 2026-08-21 |
| `examples/opi_gnt.rs` | 35 | 08292c9 | One-shot Orange-Pi RE probe (July campaign) | 2026-08-21 |
| `examples/opi_h2c3.rs` | 55 | 5e802c6 | One-shot Orange-Pi RE probe (July campaign) | 2026-08-21 |
| `examples/opi_inherit.rs` | 35 | 08292c9 | One-shot Orange-Pi RE probe (July campaign) | 2026-08-21 |
| `examples/opi_inject.rs` | 82 | dce6d50 | One-shot Orange-Pi RE probe (July campaign) | 2026-08-21 |
| `examples/opi_msr.rs` | 34 | 1ef5954 | One-shot Orange-Pi RE probe (July campaign) | 2026-08-21 |
| `examples/opi_pwrbump.rs` | 43 | c432e23 | One-shot Orange-Pi RE probe (July campaign) | 2026-08-21 |
| `examples/opi_regdump.rs` | 27 | 10eaa87 | One-shot Orange-Pi RE probe (July campaign) | 2026-08-21 |
| `examples/opi_replay.rs` | 46 | 0c2856a | One-shot Orange-Pi RE probe (July campaign) | 2026-08-21 |
| `examples/opi_rfdump.rs` | 11 | 88bb6c6 | One-shot Orange-Pi RE probe (July campaign) | 2026-08-21 |
| `examples/opi_rfhealth.rs` | 47 | 9b2cad1 | One-shot Orange-Pi RE probe (July campaign) | 2026-08-21 |
| `examples/opi_rfmatch.rs` | 40 | 88bb6c6 | One-shot Orange-Pi RE probe (July campaign) | 2026-08-21 |
| `examples/opi_rsvd.rs` | 48 | cf0a266 | One-shot Orange-Pi RE probe (July campaign) | 2026-08-21 |
| `examples/opi_rxmeta.rs` | 26 | 6dc9036 | One-shot Orange-Pi RE probe (July campaign) | 2026-08-21 |
| `examples/opi_rxtx.rs` | 44 | 4c9d5ca | One-shot Orange-Pi RE probe (July campaign) | 2026-08-21 |
| `examples/opi_selftest.rs` | 46 | df71cad | One-shot Orange-Pi RE probe (July campaign) | 2026-08-21 |
| `examples/opi_sweep.rs` | 35 | cfa145e | One-shot Orange-Pi RE probe (July campaign) | 2026-08-21 |
| `examples/opi_swing.rs` | 34 | 0d001b0 | One-shot Orange-Pi RE probe (July campaign) | 2026-08-21 |
| `examples/opi_thermal.rs` | 30 | a9084cf | One-shot Orange-Pi RE probe (July campaign) | 2026-08-21 |
| `examples/opi_timesrc.rs` | 20 | 2155a5a | One-shot Orange-Pi RE probe (July campaign) | 2026-08-21 |
| `examples/opi_track.rs` | 30 | 0d001b0 | One-shot Orange-Pi RE probe (July campaign) | 2026-08-21 |
| `examples/opi_tsf.rs` | 18 | a6a6613 | One-shot Orange-Pi RE probe (July campaign) | 2026-08-21 |
| `examples/opi_tssi_ab.rs` | 38 | 8e149ff | One-shot Orange-Pi RE probe (July campaign) | 2026-08-21 |
| `examples/opi_tssi.rs` | 32 | debbd63 | One-shot Orange-Pi RE probe (July campaign) | 2026-08-21 |
| `examples/opi_tx.rs` | 30 | 75fab8e | One-shot Orange-Pi RE probe (July campaign); was the usage example in scripts/supervise_tx.sh's comment (TXBIN is user-supplied there) | 2026-08-21 |
| `examples/opi_txretry.rs` | 38 | df71cad | One-shot Orange-Pi RE probe (July campaign) | 2026-08-21 |
| `examples/opi_txrx.rs` | 34 | 6dadd42 | One-shot Orange-Pi RE probe (July campaign) | 2026-08-21 |
| `examples/opi_usbreset.rs` | 42 | 8b532f8 | One-shot Orange-Pi RE probe (July campaign) | 2026-08-21 |
| `tools/rtl8733b-hwprobe/` | 605 (src) + committed target/ | 32a5cdc | One-shot M4.5 RE bench; verified sequences ported into src/libusb_rtl8733b.rs; carried ~2395 tracked files (mostly target/) | 2026-08-21 |
| `tools/rtl8733b-usbmon/` | 310 (src) + committed target/ | 12f6abc | One-shot usbmon capture parser for the RTL8733BU TX-enable hunt; findings absorbed into src/libusb_rtl8733b.rs; carried ~345 tracked files (mostly target/) | 2026-08-21 |
| `firmware/esp32-cyd-radio/` | 218 | f883bfe | One-shot ESP32-CYD CSI-capture experiment; untouched since 2026-07-09, no consumers | 2026-08-21 |
| `firmware/esp32-ndn-bridge/` | 204 | f883bfe | One-shot ESP32 serial-bridge experiment; untouched since 2026-07-09, no consumers | 2026-08-21 |
| `firmware/heltec-lora-node/` | 135 | 7768823 | Standalone SX1276 interop-check sketch (2026-07-09); interop proven, superseded by firmware/heltec-lora-rs | 2026-08-21 |
| `firmware/heltec-lora-bridge/` | 295 + committed Arduino build/ | bb78a21 | Arduino stopgap for the node-C 7E-A5 modem role during the 2026-07-28 field session; firmware/heltec-lora-rs (whose CAD-hang fix postdates it) is the current node-C firmware; build/ carried a committed 9.6MB elf + 4MB merged.bin | 2026-08-21 |
| `firmware/build-out/` | 0 (empty, never tracked) | — | Empty untracked arduino-cli --output-dir; build-bw16-rs.sh recreates it on demand | 2026-08-21 |
| `.DS_Store`, `firmware/.DS_Store` | — | f1af97a / 32a5cdc | macOS Finder droppings untracked (`git rm --cached`); now gitignored | 2026-08-21 |

Kept deliberately (checked, not archived):

- `firmware/bw16-rs-sketch/` — NOT a duplicate of `bw16-rs`: it is the live ~40-line C++
  FFI shim (plus GPLv3 `packet-injection.{h,cpp}`) that `firmware/build-bw16-rs.sh` compiles
  and links `libbw16_rs.a` into. Removing it breaks the BW16 firmware build.
