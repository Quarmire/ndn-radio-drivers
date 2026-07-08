#!/usr/bin/env bash
# Process-level TX supervisor for the RTL8731BU userspace driver.
#
# The per-boot analog TX state is LOCKED at process start and immutable for the process
# lifetime — no in-process reset (register re-init, re-open, or even a libusb USB port reset)
# re-randomizes it (proven). Only a FRESH OS PROCESS varies the outcome (~50% each when the
# device is fresh). So reliable TX = relaunch the process until delivery is confirmed.
#
# Here the "delivery check" is a witness radio (grep the injected src MAC). In production,
# replace it with a protocol-level confirmation (an NDN Data for a probe Interest, an ACK,
# a peer echo). At p/process, N launches give 1-(1-p)^N reliability.
#
#   TXBIN=./target/debug/examples/opi_tx WITNESS=wlu1 CH=36 MAXATT=8 ./supervise_tx.sh
set -u
TXBIN="${TXBIN:?set TXBIN to the tx binary}"; CH="${CH:-36}"; MAXATT="${MAXATT:-8}"
WITNESS="${WITNESS:-wlu1}"; SRC="${SRC:-4d:59:44:52:56}"; LIB="${LIB:-}"
for att in $(seq 1 "$MAXATT"); do
  sudo rmmod 8733bu 2>/dev/null; sleep 2
  sudo ip link set "$WITNESS" down 2>/dev/null; sudo iw dev "$WITNESS" set type monitor 2>/dev/null
  sudo ip link set "$WITNESS" up 2>/dev/null; sudo iw dev "$WITNESS" set channel "$CH" 2>/dev/null
  sudo rm -f /tmp/sup.pcap; sudo tcpdump -i "$WITNESS" -w /tmp/sup.pcap -U 2>/dev/null & A=$!
  sleep 1
  timeout 30 sudo env LD_LIBRARY_PATH="$LIB" "$TXBIN" "$CH" 5 >/dev/null 2>&1
  sleep 1; sudo kill "$A" 2>/dev/null; sleep 1
  if [ "$(sudo tcpdump -e -r /tmp/sup.pcap 2>/dev/null | grep -c "$SRC")" -gt 0 ]; then
    echo "TX confirmed on attempt $att"; exit 0
  fi
done
echo "TX not confirmed after $MAXATT attempts"; exit 1
