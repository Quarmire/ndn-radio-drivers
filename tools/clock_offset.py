#!/usr/bin/env python3
"""Wired NTP-style clock offset between two bench nodes — the term the named airtime
lease's cross-node score is limited by.

WHY THIS EXISTS. `examples/halow_lease.rs` places frames on a slot grid computed from
CLOCK_REALTIME, so the grid is only shared if the two nodes' clocks are. On this bench
they are NTP-synced to *different* pool servers and MEASURED ~6.9 ms apart — 7x the 1 ms
guard and 69% of a 10 ms slot. Folded on its raw clock the receiver put a perfectly-placed
run in the NEIGHBOURING slot and scored 0% in-lease; folded on a clock corrected by the
offset THIS script measures, the same run scored 100%.

★ The offset must be measured INDEPENDENTLY of the radio run it is applied to — over the
wired LAN, not from the frames being scored. An offset fitted to the data it then vindicates
measures nothing. Bracket the run: measure before and after, and report the drift (measured
78 us over ~90 s here, i.e. well inside the guard).

  server (the RECEIVER node):  python3 clkoff.py server 0.0.0.0 9931
  client (the SENDER node):    python3 clkoff.py client <receiver-ip> 9931 200

Prints offset = clock(client) - clock(server). For `halow_lease rx` on the server node pass
the NEGATION as --clock-offset-us (the option wants rx_clock - tx_clock).

The best-decile-by-RTT filter is the standard NTP trick: the lowest-RTT samples are the least
path-asymmetric, so they carry the least offset error. NixOS has no system python3 — run under
`nix-shell -I nixpkgs=channel:nixos-unstable -p python3 --run "..."`.
"""
import socket, sys, time, statistics

def now_us():
    return int(time.clock_gettime(time.CLOCK_REALTIME) * 1e6)

mode, host, port = sys.argv[1], sys.argv[2], int(sys.argv[3])
if mode == "server":
    s = socket.socket(); s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    s.bind((host, port)); s.listen(1)
    c, _ = s.accept(); c.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
    f = c.makefile("rwb", buffering=0)
    while True:
        line = f.readline()
        if not line: break
        t2 = now_us(); t3 = now_us()
        f.write(b"%d %d\n" % (t2, t3))
    c.close()
else:
    n = int(sys.argv[4])
    s = socket.create_connection((host, port), timeout=10)
    s.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
    f = s.makefile("rwb", buffering=0)
    samples = []
    for _ in range(n):
        t1 = now_us(); f.write(b"p\n")
        t2, t3 = map(int, f.readline().split()); t4 = now_us()
        rtt = (t4 - t1) - (t3 - t2)
        off = ((t2 - t1) + (t3 - t4)) // 2     # clock(server) - clock(client)
        samples.append((rtt, off))
        time.sleep(0.004)
    s.close()
    samples.sort()                              # lowest RTT = least asymmetric = best estimate
    best = samples[: max(1, len(samples) // 10)]
    offs = [-o for _, o in best]                # clock(client) - clock(server)
    rtts = [r for r, _ in best]
    allo = sorted(-o for _, o in samples)
    print("n=%d  best-decile n=%d" % (len(samples), len(best)))
    print("rtt_us      min=%d  median=%d" % (min(r for r, _ in samples), statistics.median(rtts)))
    print("offset_us   client-minus-server: best=%d  median_all=%d  spread(p10..p90)=%d..%d"
          % (statistics.median(offs), statistics.median(allo),
             allo[len(allo)//10], allo[9*len(allo)//10]))
