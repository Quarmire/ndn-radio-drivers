#!/bin/sh
# fwload.sh <image>  -- load a patched mm6108 image and restore the bench state.
# The unbind DESTROYS mon0 and RESETS the channel to 904.5 MHz / 1 MHz; both are restored here,
# because a run that silently reverts to the default PHY is a different experiment.
set -e
cp "$1" /tmp/fwpatch/morse/mm6108.bin
echo spi0.0 | sudo tee /sys/bus/spi/drivers/morse_spi/unbind >/dev/null
sleep 4
echo spi0.0 | sudo tee /sys/bus/spi/drivers/morse_spi/bind >/dev/null
sleep 7
sudo dmesg | grep -E "Loaded firmware|pkt_memory" | tail -2
PHY=$(ls /sys/class/ieee80211/ | while read p; do [ -e "/sys/class/ieee80211/$p/device/driver/module/drivers/spi:morse_spi" ] && echo $p; done)
[ -n "$PHY" ] || PHY=$(basename $(ls -d /sys/class/ieee80211/* | tail -1))
sudo ip link set wlan0 up || true
sudo iw phy $PHY interface add mon0 type monitor
sudo ip link set mon0 up
sudo ip link set morse0 up
timeout 25 sudo /tmp/morse_cli -i mon0 channel -c 908000 -o 8 -p 1 -n 0 >/dev/null 2>&1
sleep 1
timeout 25 sudo /tmp/morse_cli -i mon0 channel | head -4
