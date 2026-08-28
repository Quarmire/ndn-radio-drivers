#!/usr/bin/env python3
"""Read EVT_HOPTRACE (0x8E) from a 7E-A5 node and print its own hop timeline.

Each node stamps its OWN hop events on its OWN clock, so no cross-vendor reception is
needed -- which is the point, because the link that would carry the comparison is the
thing under test.  See firmware/{lr2021-nrf54l15-rs,heltec-lora-rs}/README.md.

  python3 tools/hoptrace.py --port /dev/cu.usbmodemXXXX --lr2021 --run
  python3 tools/hoptrace.py --port /dev/cu.usbserial-XXXX --run

Needs pyserial.  Nothing here transmits unless you pass --tx.
"""
import argparse, struct, sys, time
import serial

SYNC = b"\x7e\xa5"
CMD_TX, CMD_SET_FREQ, CMD_SET_MOD, CMD_SET_DEBUG = 0x01, 0x02, 0x03, 0x15
CMD_GET_CAP, CMD_SET_PHY, CMD_SET_HOP, CMD_GET_HOPTRACE = 0x1A, 0x1D, 0x1E, 0x20
EVT_CAP, EVT_HOPTRACE, EVT_UNSUPPORTED = 0x8B, 0x8E, 0x8F


def frame(typ, pl=b""):
    c = typ ^ len(pl)
    for b in pl:
        c ^= b
    return SYNC + bytes([typ, len(pl)]) + pl + bytes([c])


def read_frames(ser, secs):
    """Yield (typ, payload) for `secs`.  Same state machine as the firmware parser."""
    end, buf = time.time() + secs, bytearray()
    while time.time() < end:
        d = ser.read(256)
        if d:
            buf += d
        while True:
            i = buf.find(SYNC)
            if i < 0 or len(buf) < i + 5:
                break
            typ, n = buf[i + 2], buf[i + 3]
            if len(buf) < i + 5 + n:
                break
            pl, crc = bytes(buf[i + 4 : i + 4 + n]), buf[i + 4 + n]
            c = typ ^ n
            for b in pl:
                c ^= b
            del buf[: i + 5 + n]
            if c == crc:
                yield typ, pl


def ask(ser, typ, pl=b"", want=None, secs=2.0):
    ser.reset_input_buffer()
    ser.write(frame(typ, pl))
    for t, p in read_frames(ser, secs):
        if want is None or t == want or t == EVT_UNSUPPORTED:
            return t, p
    return None, None


def decode_trace(pl):
    if len(pl) < 5:
        return None
    hz, n = struct.unpack(">IB", pl[:5])
    if len(pl) != 5 + 5 * n:                      # refuse, never truncate
        raise ValueError(f"n={n} disagrees with body length {len(pl)}")
    ev = [(pl[5 + 5 * k], struct.unpack(">I", pl[6 + 5 * k : 10 + 5 * k])[0]) for k in range(n)]
    return hz, ev


def report(hz, ev, sf, bw, period):
    tsym_us = (1 << sf) / bw * 1e6
    print(f"  stamp_hz = {hz}  ({1e6/hz:.4f} us/tick)   n = {len(ev)}")
    print(f"  expected hop period = {period} sym = {period*tsym_us/1000:.3f} ms "
          f"= {round(period*tsym_us*hz/1e6)} ticks")
    if not ev:
        print("  ** n = 0: this node recorded no hop event since its plan was armed. **")
        print("     Either it does not raise a hop interrupt the MCU can see, or the plan")
        print("     never armed.  Check CMD_SET_HOP's EVT_INFO reply before concluding.")
        return
    print("   #  idx  tx  t_ticks       dt_ticks     dt_us    dt/period  idx_step")
    prev = None
    for k, (idx, t) in enumerate(ev):
        tx = "TX" if idx & 0x80 else "  "
        i6 = idx & 0x3F
        if prev is None:
            print(f"  {k:2d}  {i6:3d}  {tx}  {t:10d}")
        else:
            dt = (t - prev[1]) & 0xFFFFFFFF
            step = (i6 - (prev[0] & 0x3F)) & 0x3F
            print(f"  {k:2d}  {i6:3d}  {tx}  {t:10d}   {dt:10d}  {dt*1e6/hz:9.1f}"
                  f"   {dt*1e6/hz/(period*tsym_us):8.3f}   {step:5d}"
                  + ("   <-- GAP: not one hop" if step not in (0, 1) else ""))
        prev = (idx, t)
    d = [((ev[k+1][1]-ev[k][1]) & 0xFFFFFFFF)*1e6/hz for k in range(len(ev)-1)]
    if d:
        d.sort()
        print(f"  interval us: min {d[0]:.1f}  median {d[len(d)//2]:.1f}  max {d[-1]:.1f}"
              f"   (spread bounds this node's stamp jitter)")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", required=True)
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--lr2021", action="store_true", help="send CMD_SET_PHY 0 (LoRa) first")
    ap.add_argument("--freq", type=int, default=915_000_000)
    ap.add_argument("--sf", type=int, default=7)
    ap.add_argument("--bw", type=int, default=125_000)
    ap.add_argument("--period", type=int, default=8, help="hop period in symbols")
    ap.add_argument("--n", type=int, default=1, help="hop-list depth (1 = machinery runs, carrier fixed)")
    ap.add_argument("--dwell", type=float, default=0.5, help="seconds of hopping before the read")
    ap.add_argument("--run", action="store_true", help="arm hopping, dwell, then read")
    ap.add_argument("--tx", action="store_true", help="also send one frame while hopping (phase vs a frame)")
    ap.add_argument("--again", type=float, default=0.0, help="re-read after N s (do hops continue?)")
    args = ap.parse_args()

    ser = serial.Serial(args.port, args.baud, timeout=0.1)
    time.sleep(0.3)

    if args.run:
        if args.lr2021:
            print("CMD_SET_PHY 0 (LoRa) ->", ask(ser, CMD_SET_PHY, b"\x00", EVT_CAP)[0])
        ask(ser, CMD_SET_MOD, bytes([args.sf, 0, 1]))          # bw_code 0 == 125 kHz on both
        ask(ser, CMD_SET_FREQ, struct.pack(">I", args.freq))
        pl = bytes([1]) + struct.pack(">H", args.period) + bytes([args.n])
        for k in range(args.n):
            pl += struct.pack(">I", args.freq + k * 400_000)
        t, p = ask(ser, CMD_SET_HOP, pl)
        if t == EVT_UNSUPPORTED:
            print(f"!! CMD_SET_HOP refused: cmd=0x{p[0]:02x} reason=0x{p[1]:02x}")
            sys.exit(1)
        print(f"CMD_SET_HOP on, period={args.period} sym, n={args.n} -> evt 0x{t:02x}")
        if args.tx:
            time.sleep(0.05)
            ask(ser, CMD_TX, b"HOPPHASE")
        time.sleep(args.dwell)

    # ** Do NOT disable hopping before reading.  Both nodes keep the ring across a disarm
    # ** (arming clears, disarming does not), but leaving it on also keeps the two runs
    # ** symmetric and avoids any question about what the last command did.
    t, p = ask(ser, CMD_GET_HOPTRACE, b"", EVT_HOPTRACE, secs=3.0)
    if t is None:
        print("!! no reply to CMD_GET_HOPTRACE (0x20) -- node did not answer at all")
        sys.exit(2)
    if t == EVT_UNSUPPORTED:
        why = {0x01: "UNKNOWN_OPCODE (firmware predates the H pass)",
               0x02: "NO_HARDWARE (understood, but this part cannot timestamp its hops)",
               0x03: "BAD_LENGTH", 0x04: "OUT_OF_RANGE"}.get(p[1], hex(p[1]))
        print(f"!! EVT_UNSUPPORTED [0x{p[0]:02x}, {why}]")
        sys.exit(3)
    print("EVT_HOPTRACE:")
    report(*decode_trace(p), args.sf, args.bw, args.period)

    if args.again:
        time.sleep(args.again)
        t, p = ask(ser, CMD_GET_HOPTRACE, b"", EVT_HOPTRACE, secs=3.0)
        print(f"\nEVT_HOPTRACE again, +{args.again}s (do hops continue with no traffic?):")
        report(*decode_trace(p), args.sf, args.bw, args.period)


if __name__ == "__main__":
    main()
