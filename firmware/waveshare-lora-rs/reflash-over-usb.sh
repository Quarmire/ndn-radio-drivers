#!/usr/bin/env bash
# Reflash a Waveshare LoRa dongle over its USB/CH343 link — NO ST-Link.
#
# Requires firmware already carrying CMD_ENTER_BOOTLOADER (the self-DFU bootstrap flash onward), the
# `lora_dfu` example binary, and `stm32flash` (on NixOS: this script wraps it in nix-shell).
#
# Usage:  ./reflash-over-usb.sh <tty> <firmware.bin> <lora_dfu-binary>
#   e.g.  ./reflash-over-usb.sh /dev/ttyACM0 firmware.bin ./lora_dfu
set -euo pipefail
TTY="${1:?tty, e.g. /dev/ttyACM0}"
BIN="${2:?firmware.bin path}"
DFU="${3:?lora_dfu binary path}"

echo ">> kicking $TTY into the ROM bootloader (CMD_ENTER_BOOTLOADER)"
"$DFU" "$TTY" || true          # fire-and-forget; the dongle branches away with no reply
sleep 1                        # let the ROM bootloader settle its autobaud

run_stm32flash() {
  # -b 115200 matches the CH343 line; -w write + -v verify; -g runs the new app (nRST isn't wired).
  stm32flash -b 115200 -w "$BIN" -v -g 0x08000000 "$TTY"
}

echo ">> flashing $BIN via stm32flash"
if command -v stm32flash >/dev/null 2>&1; then
  run_stm32flash
else
  nix-shell -p stm32flash --run "stm32flash -b 115200 -w '$BIN' -v -g 0x08000000 '$TTY'"
fi
echo ">> done — $TTY is running the new firmware"
