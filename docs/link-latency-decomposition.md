# Link-level latency: where the milliseconds go

**The question.** A radio signal travels in **nanoseconds** (1 m ≈ 3.3 ns). A packet at the data rate is
**microseconds** (a 38-byte frame is ~71 µs at 6 Mbit/s, ~25 µs at MCS7). So why is a link "ping"
**milliseconds**? This decomposes it, measured on two ESP32-C5 serial-bridged radios.

## Method

Three measurements isolate each layer (`firmware/esp32c5-ndn/tools/link_latency.py`, two C5s on USB):

- **A) Interconnect tax** — a `T_READCLOCK` round-trip: host → device → host over USB, **with no radio
  involved at all**. This is the pure serial-bridge cost.
- **B) Full one-way link** — host writes `T_INJECT` to radio A → A transmits → B receives → B reports the
  frame (with its hardware RX stamp) back to the host. This is the "ping" as observed.
- **C) Analytic airtime** — `frame_bytes × 8 / rate` + preamble: the physics floor.

## Measured (2 × ESP32-C5, native USB-Serial-JTAG, 38-byte frame)

| Layer | Latency | Share of the ping |
|---|---:|---:|
| Propagation @ 1 m | **0.0033 µs** (3.3 ns) | ~0.00006 % |
| Airtime @ 6 Mbit/s (+ preamble) | **71 µs** | **1.2 %** |
| Airtime @ MCS7 | 25 µs | 0.4 % |
| **Interconnect tax** (USB + firmware, no radio) | **~6.3 ms** median | **~99 %** |
| **Full one-way link ping** | **~5.8 ms** median | 100 % |

(Interconnect RTT and the full link ping are the same order because both cross USB twice — host→A and
B→host. The RTT best case was 0.35 ms and worst 16.7 ms: the spread is OS-scheduling jitter, not the radio.)

## The answer

**The milliseconds are the serial interconnect, not the radio.** On a serial-bridged radio the frame crosses
a USB-CDC link **twice** (host→radio to inject, radio→host to report), and each crossing costs milliseconds —
dominated not by the raw USB Full-Speed 1 ms frame but by the **host CDC driver + OS wakeup latency** and the
**firmware's serial loop**. The actual on-air event — propagation (ns) + airtime (µs) — is **~1 %** of the ping.

Concretely, for our C5 stack the ~6 ms breaks down as:
- **Host OS / pyserial / CDC driver** — the largest and jitteriest part (coarse wakeups, buffering).
- **USB Full-Speed transaction** — ~1 ms/direction floor.
- **Firmware serial loop** — the Rust firmware's TX-drain loop `sleep(2 ms)` when idle, plus read batching;
  this is a **fixable** contributor (event-driven serial / shorter idle sleep would cut it).
- **`esp_wifi_80211_tx` submission** — sub-millisecond.
- **Airtime + propagation** — ~71 µs + 3 ns. Negligible.

## Implications

- **A serial-bridged radio is a milliseconds-latency device by construction.** The bridge, not the PHY, sets
  the floor. Lowering the rate or shrinking the frame barely moves the ping (airtime is already ~1 %).
- **To get link latency near the physics floor you must remove the interconnect**, not tune the radio:
  a native (non-bridged) driver, or a lower-latency host transport (USB High-Speed on an S3/P4, or
  SDIO/SPI-slave à la ESP-Hosted where the host has the pins). See `link-latency` notes in the interconnect
  discussion.
- **For real-time control (CRSF wants < 10 ms end-to-end):** the ~6 ms serial bridge already eats most of a
  CRSF budget one-way; a round-trip control loop over the bridge would blow it. Firmware serial-loop tuning
  (drop the 2 ms idle sleep) recovers a few ms; beyond that, control-grade latency needs the native/HS/SDIO
  path, not the USB-CDC bridge. This is the key input to the DroneBridge/CRSF-over-NDR exploration.

## Reproduce

```sh
python3 firmware/esp32c5-ndn/tools/link_latency.py   # needs two C5s on /dev/cu.usbmodem*
```

The tool prints all three measurements and the percentage decomposition. Frame size and rate are constants
at the top; the analytic airtime is computed for 6 Mbit/s and MCS7.
