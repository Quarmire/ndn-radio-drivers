#!/usr/bin/env bash
# Hand an mt76 USB radio to our userspace driver — and give it back afterwards.
#
# WHY THIS EXISTS (measured 2026-08-27, at the cost of two MT7612U replugs):
# claiming an mt76 part with libusb's auto-detach-kernel-driver is NOT a safe way
# to acquire it. The detach runs the kernel driver's *disconnect* path, which for
# mt76x2u stops the MCU and powers the chip down. Our bring_up then correctly
# observes "firmware not running", re-downloads it into a chip mid-teardown, and
# on a SuperSpeed bus that is what wedges the dongle:
#
#     usb 2-1.2: device not accepting address 11, error -62
#     usb 2-1-port2: unable to enumerate USB device
#
# and only a physical replug brings it back. Worse, between runs the kernel
# re-binds and the next claim fails with "Resource busy", so the failure mode
# alternates confusingly between "busy" and "timed out".
#
# The fix is the same one this repo already uses for the RTL8812AU: take the
# device away from the kernel BEFORE the driver runs, and stop the kernel
# re-grabbing it, then restore both afterwards. Nothing here resets the device —
# a blind USB reset is the other half of the wedge (see Mt7612uBackend::open).
#
#   ./mt76_acquire.sh acquire 7610      # or 7612 / 7961
#   ./mt76_acquire.sh release 7610
#   ./mt76_acquire.sh status
set -uo pipefail

PID="${2:-}"
drv_for_pid() {
  case "$1" in
    7610) echo mt76x0u ;;
    7612|7632|7662) echo mt76x2u ;;
    7961) echo mt7921u ;;
    *) echo "" ;;
  esac
}

# Every USB device node whose idProduct matches, e.g. "2-1.2".
devs_for_pid() {
  local pid="$1" d
  for d in /sys/bus/usb/devices/*/; do
    [ -f "$d/idProduct" ] || continue
    [ "$(cat "$d/idVendor")" = "0e8d" ] || continue
    [ "$(cat "$d/idProduct")" = "$pid" ] || continue
    basename "$d"
  done
}

status() {
  echo "drivers_autoprobe = $(cat /sys/bus/usb/drivers_autoprobe)"
  for d in /sys/bus/usb/devices/*/; do
    [ -f "$d/idVendor" ] || continue
    [ "$(cat "$d/idVendor")" = "0e8d" ] || continue
    local n; n=$(basename "$d")
    echo "$n  ${1:-}$(cat "$d/idProduct")  speed=$(cat "$d/speed")"
    for i in "$d"*:*; do
      [ -d "$i" ] || continue
      printf "    %-14s class=%s drv=%s\n" "$(basename "$i")" \
        "$(cat "$i/bInterfaceClass")" \
        "$( [ -e "$i/driver" ] && basename "$(readlink -f "$i/driver")" || echo none)"
    done
  done
}

case "${1:-status}" in
  acquire)
    [ -n "$PID" ] || { echo "usage: $0 acquire <pid hex, e.g. 7610>"; exit 2; }
    drv=$(drv_for_pid "$PID")
    # Stop the kernel re-binding the moment we let go, or on any re-enumeration.
    # Global by necessity (there is no per-device knob); restored by `release`.
    echo 0 | sudo tee /sys/bus/usb/drivers_autoprobe >/dev/null
    for dev in $(devs_for_pid "$PID"); do
      for i in /sys/bus/usb/devices/"$dev"/"$dev":*; do
        [ -e "$i/driver" ] || continue
        cur=$(basename "$(readlink -f "$i/driver")")
        # Only unbind the WLAN driver; leave btusb alone on the composite
        # MT7921AU, whose interfaces 0-2 are a Bluetooth radio someone else owns.
        [ "$cur" = "$drv" ] || continue
        echo "unbinding $cur from $(basename "$i")"
        echo -n "$(basename "$i")" | sudo tee "/sys/bus/usb/drivers/$cur/unbind" >/dev/null
      done
    done
    sleep 1
    status
    ;;
  release)
    [ -n "$PID" ] || { echo "usage: $0 release <pid hex>"; exit 2; }
    drv=$(drv_for_pid "$PID")
    echo 1 | sudo tee /sys/bus/usb/drivers_autoprobe >/dev/null
    for dev in $(devs_for_pid "$PID"); do
      for i in /sys/bus/usb/devices/"$dev"/"$dev":*; do
        [ -d "$i" ] || continue
        [ -e "$i/driver" ] && continue
        # Re-bind only the interface class the WLAN driver owns (0xff).
        [ "$(cat "$i/bInterfaceClass")" = "ff" ] || continue
        echo "rebinding $drv to $(basename "$i")"
        echo -n "$(basename "$i")" | sudo tee "/sys/bus/usb/drivers/$drv/bind" >/dev/null 2>&1 || true
      done
    done
    sleep 1
    status
    ;;
  park)
    # ★ RUN THIS BEFORE PHYSICALLY REPLUGGING AN mt76 DONGLE.
    #
    # `release` restores drivers_autoprobe=1. If the dongle is then unplugged and
    # replugged, the KERNEL probes it first and loads its own firmware. Our driver's
    # warm-path guard sees "firmware running", skips its own register init, and every
    # MCU command then fails (32 of 32 op errors, 0 frames transmitted) because our
    # channel-set blobs are deltas against OUR init state, not the kernel's. That state
    # is NOT recoverable in software -- the register replay hangs and a forced firmware
    # download times out on chunk 0 -- so it costs another physical replug.
    #
    # MEASURED 2026-08-28: replug with autoprobe=0 -> cold path -> 2946 f/s.
    #                      replug with autoprobe=1 -> kernel probes -> 0 f/s, bricked.
    echo 0 | sudo tee /sys/bus/usb/drivers_autoprobe >/dev/null
    echo "drivers_autoprobe = 0 -- safe to replug; the kernel will ignore the dongle"
    [ -n "$PID" ] && status
    ;;
  status) status ;;
  *) echo "usage: $0 {acquire|release|park|status} [pid]"; echo "       park: set autoprobe=0 before a physical replug"; exit 2 ;;
esac
