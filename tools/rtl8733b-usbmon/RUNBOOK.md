# RTL8733BU TX-enable capture runbook

**Goal:** capture the *vendor* `rtl8733bu` driver transmitting one frame, so we can
find the BB/RF register write that keys the antenna on TX — the one our userspace
driver is missing. RX works; the MAC accepts injects (FIFO drains) but nothing
radiates (an 8812au witness inches away hears 0/200 injects). Everything MAC/BB-side
is set (channel, TXPAUSE=0, ref power, per-rate AGC table `0x3a00`, external TRSW);
the remaining gap is in the **BB/RF TX-path activation**.

Run this on the **OPi** (Linux, real `0bda:f72b`). It produces two usbmon text
captures — vendor TX and ours — which the `usbmon` parser here diffs.

---

## 0. One-time setup on the OPi

```sh
# usbmon (kernel USB tracer)
sudo modprobe usbmon
ls /sys/kernel/debug/usb/usbmon/            # needs debugfs mounted (usually is)

# Find the 8733b's bus + device number, and its bus for usbmon:
lsusb | grep 0bda:f72b                       # e.g. "Bus 002 Device 005: ID 0bda:f72b"
#   -> BUS=2 (usbmon file is /sys/kernel/debug/usb/usbmon/2u), DEV=5

# Build the vendor driver (libc0607 fork) if not already:
git clone https://github.com/libc0607/rtl8733bu-20230626 && cd rtl8733bu-20230626
make -j4 && sudo make install     # or: sudo insmod 8733bu.ko
```

`2u` below = usbmon text file for **your** bus number (from `lsusb`). Substitute.

---

## 1. Capture A — VENDOR driver transmitting (the reference)

The cleanest single-frame TX: monitor mode + a raw inject. Fallback: a scan (probe
requests are TX). Bring the driver **fully up first**, *then* start the capture, *then*
transmit — so the capture is dominated by the per-TX writes, not the huge bring-up.

```sh
sudo modprobe 8733bu                          # or insmod; wlan iface appears, e.g. wlan0
IF=wlan0

# Monitor mode + up (the driver configures the full PHY/RF here):
sudo ip link set $IF down
sudo iw dev $IF set monitor none
sudo ip link set $IF up
sudo iw dev $IF set channel 1

# START the capture (in another shell / backgrounded):
sudo sh -c 'cat /sys/kernel/debug/usb/usbmon/2u > /tmp/vendor_tx.txt' &
CAP=$!
sleep 1

# TRANSMIT a handful of frames (pick ONE):
#  (a) aircrack injection test — sends test frames on the iface:
sudo aireplay-ng --test $IF ;                 # or: sudo aireplay-ng -9 $IF
#  (b) scapy raw inject (closest match to our data-path inject):
sudo python3 - <<'PY'
from scapy.all import RadioTap, Dot11, sendp
f = RadioTap()/Dot11(type=2, addr1="ff:ff:ff:ff:ff:ff",
                     addr2="02:11:22:33:44:55", addr3="02:11:22:33:44:55")/b"VENDOR-TX-PROBE"
sendp(f, iface="wlan0mon" if False else "wlan0", count=20, inter=0.05, verbose=1)
PY
#  (c) fallback if injection is blocked — a scan sends probe requests (TX):
#  sudo iw dev $IF scan

sleep 1
sudo kill $CAP                                # stop the capture
wc -l /tmp/vendor_tx.txt
```

If nothing lands in `vendor_tx.txt`, the wrong bus was used — recheck `lsusb`.

---

## 2. Capture B — OUR driver injecting (the current state)

Run our `probe8733b` (it does the full bring-up + an M8 inject) under usbmon on the
same OPi. Our inject writes succeed but don't radiate — so the capture shows what we
write, to diff against the vendor.

```sh
sudo rmmod 8733bu                             # release the device from the vendor driver
cd <ndn-radio-drivers>

sudo sh -c 'cat /sys/kernel/debug/usb/usbmon/2u > /tmp/mine_tx.txt' &
CAP=$!
sleep 1
sudo ./target/debug/examples/probe8733b       # M1..M8; the M8 line injects
sleep 1
sudo kill $CAP
```

(Build the example first: `cargo build --example probe8733b`.)

---

## 3. Analyze — find the missing TX-enable

```sh
cd tools/rtl8733b-usbmon && cargo build

# What the vendor writes right before each transmit (the per-TX enable):
./target/debug/usbmon around /tmp/vendor_tx.txt 30

# The TX-descriptor bytes the vendor puts on the wire (compare to ours):
./target/debug/usbmon bulk /tmp/vendor_tx.txt | head -40

# THE diff: registers the vendor writes that we never do, or set differently.
./target/debug/usbmon diff /tmp/vendor_tx.txt /tmp/mine_tx.txt
```

**What to look for** (annotations flag these automatically):
- `[BB (1e70 TX-blk)]` — `0x1e70[3:0]` = 0x4 (OFDM TX on) / 0x8 (CCK TX on). Our normal
  path may leave this idle; only the cal's PMAC path sets it.
- `RF-A window` / `RFx[0x..]` — an RF-register write enabling the TX mixer / PA bias /
  TX-RX switch (e.g. RF 0x00 mode, RF 0x1/0x83/0xdf gain, an RF PA-enable bit).
- `[TXAGC-table]` / `[... txagc-ref]` — the real TX power apply vs our `0x3a00`/`0x4308`.
- `[MAC-RCR/RXFLT]` / GPIO regs `0x40/0x4c/0x64` — TRSW/FEM routing.
- A **per-TX** write (appears in `around` before *every* bulk-OUT) is the prime
  suspect: a TX-path enable the driver toggles each transmit.

Send the three outputs (`around`, `bulk`, `diff`) back and the missing write drops
straight into `bring_up_monitor` / `inject_raw` in `src/libusb_rtl8733b.rs`.

---

## Notes
- usbmon shows Realtek VENQT as control transfers: `Co ... s 40 05 <addr> 0000 <len>`
  (write) / `Ci ... s c0 05 <addr> ...` (read). The parser reconstructs `addr = value`
  and little-endian-assembles multi-byte values.
- Bulk-OUT ep `0x05` = the TX frame (`[48B desc][802.11 frame]`).
- If `aireplay-ng`/scapy injection is unavailable, the scan fallback (2c) still shows
  the TX-enable — probe requests exercise the same RF TX path.
- Keep the captures short (a few seconds); usbmon text grows fast.
