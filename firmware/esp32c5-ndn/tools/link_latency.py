#!/usr/bin/env python3
"""Link-level ping latency decomposition on two ESP32-C5s.

Answers: signal is ns, airtime is us, so what makes the 'ping' ms?
Three measurements isolate each layer:
  A) INTERCONNECT tax  = T_READCLOCK round-trip (host->dev->host over USB, NO radio at all)
  B) FULL one-way link = host writes T_INJECT to A -> B receives -> B reports to host
  C) ANALYTIC airtime  = frame_bytes*8 / rate  (the physics floor)
B - (A/2 + A/2) - C  attributes the remainder.
"""
import serial, time, struct, glob, statistics as st

def op(p):
    s = serial.Serial(); s.port = p; s.baudrate = 115200; s.timeout = 0.005
    s.rts = False; s.dtr = False; s.open(); s.rts = False; s.dtr = False
    return s

def f(s, t, p):
    s.write(bytes([0x4E, 0x44, t, len(p) & 0xff, (len(p) >> 8) & 0xff]) + p); s.flush()

def frames(buf):
    i = 0
    while len(buf) - i >= 5:
        if buf[i] != 0x4E or buf[i + 1] != 0x44:
            i += 1; continue
        ty = buf[i + 2]; ln = buf[i + 3] | (buf[i + 4] << 8)
        if len(buf) - i < 5 + ln:
            break
        yield ty, bytes(buf[i + 5:i + 5 + ln])
        i += 5 + ln
    del buf[:i]

tx = op('/dev/cu.usbmodem1101')
rxp = [p for p in glob.glob('/dev/cu.usbmodem*') if '1101' not in p][0]
rx = op(rxp)
time.sleep(0.5)
for s in (tx, rx):
    f(s, 0x02, bytes([1]))
time.sleep(0.2)

# ---------- A) interconnect tax: T_READCLOCK round-trip, no radio ----------
tx.reset_input_buffer()
rtt = []
for _ in range(200):
    t0 = time.perf_counter()
    f(tx, 0x0B, b'')
    buf = bytearray(); got = False
    while time.perf_counter() - t0 < 0.05 and not got:
        buf += tx.read(64)
        for ty, pl in frames(buf):
            if ty == 0x85:
                got = True
    if got:
        rtt.append((time.perf_counter() - t0) * 1e6)
    time.sleep(0.005)

# ---------- B) full one-way link: host_send(A) -> B reports ----------
tx.reset_input_buffer(); rx.reset_input_buffer()
rxbuf = bytearray()
oneway = []
FRAME_BYTES = 38
for seq in range(120):
    src = [0x02, 0x43, 0x35, 0x09, 0x09, 0x09]
    body = bytes([0x08, 0, 0, 0, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff] + src +
                 [0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0, 0,
                  0xaa, 0xaa, 0x03, 0, 0, 0, 0x86, 0x24, 0x05, 0x08]) + struct.pack('<I', seq)
    t0 = time.perf_counter()
    f(tx, 0x01, body)  # plain T_INJECT
    got = False
    while time.perf_counter() - t0 < 0.05 and not got:
        rxbuf += rx.read(128)
        for ty, pl in frames(rxbuf):
            # captured frame carries a trailing 4-byte FCS, so the seq (our payload) is at pl[-8:-4].
            if ty == 0x82 and len(pl) >= 8 + FRAME_BYTES + 4 and struct.unpack('<I', pl[-8:-4])[0] == seq:
                oneway.append((time.perf_counter() - t0) * 1e6); got = True
    time.sleep(0.008)

def stat(x):
    return len(x), st.median(x), st.mean(x), min(x), max(x)

n_r, med_r, mean_r, lo_r, hi_r = stat(rtt)
n_o, med_o, mean_o, lo_o, hi_o = stat(oneway)
air_6m = FRAME_BYTES * 8 / 6.0 + 20  # us @ 6 Mbit/s + ~20us OFDM preamble
air_mcs7 = FRAME_BYTES * 8 / 65.0 + 20
print('=== A) INTERCONNECT tax (T_READCLOCK round-trip, NO radio) n=%d ===' % n_r)
print('    host->dev->host: median %.0f us = %.2f ms   [%.2f..%.2f ms]  (one USB crossing ~ %.0f us)' %
      (med_r, med_r / 1000, lo_r / 1000, hi_r / 1000, med_r / 2))
print('=== B) FULL one-way link (host->A->AIR->B->host) n=%d ===' % n_o)
print('    host_send->host_recv: median %.0f us = %.2f ms   [%.2f..%.2f ms]' %
      (med_o, med_o / 1000, lo_o / 1000, hi_o / 1000))
print('=== C) ANALYTIC airtime (physics floor) ===')
print('    38-byte frame: %.0f us @6 Mbit/s (+preamble) | %.0f us @MCS7 | propagation @1m = %.4f us' %
      (air_6m, air_mcs7, 1 / 300.0))
print('=== decomposition of the one-way link ping ===')
print('    airtime (radio):        ~%5.0f us  (%.1f%%)' % (air_6m, 100 * air_6m / med_o))
print('    interconnect (USB+fw):  ~%5.0f us  (%.1f%%)  <- the two USB crossings + firmware loops' % (med_r, 100 * med_r / med_o))
print('    host OS / python / rest:~%5.0f us  (%.1f%%)' % (med_o - air_6m - med_r, 100 * (med_o - air_6m - med_r) / med_o))
tx.close(); rx.close()
