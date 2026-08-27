#!/bin/bash
# Build the Rust BW16 firmware: compile the no_std staticlib for thumbv8m, then
# link it into the Ameba image via the Arduino build. The Rust .a is injected
# through {compiler.libraries.ldflags}, which sits inside the linker's
# --start-group {object_files} ... --end-group, so normal archive resolution
# pulls exactly the referenced Rust objects.
set -e
HERE="$(cd "$(dirname "$0")" && pwd)"
CRATE="$HERE/bw16-rs"
SKETCH="$HERE/bw16-rs-sketch"
FQBN="realtek:AmebaD:Ai-Thinker_BW16"

echo "== cargo build (thumbv8m.main-none-eabihf) =="
(cd "$CRATE" && cargo build --release)
# Ask cargo where it actually put the archive: a shared CARGO_TARGET_DIR (this
# workspace sets one) redirects it out of the crate directory.
TARGET_DIR="$(cd "$CRATE" && cargo metadata --format-version 1 --no-deps | python3 -c 'import json,sys; print(json.load(sys.stdin)["target_directory"])')"
LIBDIR="$TARGET_DIR/thumbv8m.main-none-eabihf/release"
[ -f "$LIBDIR/libbw16_rs.a" ] || { echo "libbw16_rs.a not found in $LIBDIR" >&2; exit 1; }

echo "== arduino-cli compile + link libbw16_rs.a =="
arduino-cli compile --fqbn "$FQBN" \
  --build-property "compiler.libraries.ldflags=-L$LIBDIR -lbw16_rs" \
  --output-dir "$HERE/build-out" \
  "$SKETCH"
