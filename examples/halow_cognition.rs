//! **The spectrum-cognition loop for 802.11ah (S1G): sense a non-cooperative emitter, decide, and
//! MOVE — automatically, in seconds.**
//!
//! A named airtime lease shares a channel between *our own* nodes, which cooperate by construction.
//! It buys nothing against an emitter that never agreed to anything: a beaconing AP does not honour
//! our slots, so the only lever left is **the channel itself**. This example is the loop that pulls
//! it, end to end, with no human and no offline correlation:
//!
//! ```text
//!   SENSE   read the MM6108 "Narrowband interference count" over a short dwell  → events/s
//!   DECIDE  threshold + hysteresis + a dwell budget                             → move / stay
//!   SCORE   retune through the candidate list, one dwell each                   → occupancy map
//!   MOVE    morse_cli channel, readback-verified                                → new channel
//!   RESUME  traffic continues; the counter keeps being read on the new channel
//! ```
//!
//! # Why the interference *count*, and not occupancy percent
//!
//! ☠ **The emitter this was built against is invisible to packet capture.** 60 s of passive
//! listening with our transmitters silent yields ~7 packets — it beacons at 906 MHz / 2 MHz while we
//! operate at 908 MHz / 8 MHz with a primary that is not aligned to it, so the PHY cannot turn its
//! energy into a frame. **The chip defers to it anyway.** `Narrowband interference count` is the
//! counter of exactly that energy: the strict complement of packet capture. Measured on this bench,
//! the two sum to the emitter's true 9.766 Hz beacon rate whichever way the radio is tuned.
//! **Never use "the capture was empty" as evidence of a clean channel.**
//!
//! It is also not an *airtime* detector, and that is the point. The emitter costs ~2.9 % of airtime
//! — 0.065 Mbit/s out of ~9.5 — but takes p99 frame placement from 52 µs to 1934 µs. A percentage
//! threshold would have to fire on a 2.9 pp shift buried in the scorer's own ±2 pp of noise. The
//! event rate keys on the interferer's *periodic structure* instead and moves ~50:1.
//!
//! # The three things that make this a loop and not a demo
//!
//! * **Hysteresis** (`--strikes`, `--min-dwell-s`). A radio that flaps is worse than a static one.
//!   One over-threshold window is not a decision; `--strikes` consecutive ones are, and after a
//!   move the loop is barred from moving again for `--min-dwell-s`.
//! * **A dwell budget** (`--window-ms`, `--period-ms`). Sensing is not free: every counter read is a
//!   `morse_cli` round trip over the same SPI bus the data plane uses, and every *candidate* dwell
//!   is time spent off our own channel. The loop prints the measured airtime it spent sensing, in
//!   Mbit/s foregone, so the budget can be checked against what it saves.
//! * **No coordination protocol.** A node that owns the sensor decides for itself. A node that does
//!   *not* — and in a real fleet most will not; on this bench only one of the two MM6108s runs the
//!   telemetry firmware that exposes the counter at all — **re-acquires by search** (`--follow`):
//!   when the stream goes silent for `--silence-ms` it walks the same candidate list until it hears
//!   frames again. No association, no AP, no beacon, no channel-switch announcement, and nothing
//!   that has to be delivered *through the channel that just became unusable*. The two halves are
//!   measured separately: the sensor's decision latency, and the follower's rendezvous latency.
//!
//! # Instrument honesty
//!
//! * `Narrowband interference detected` / `power (dBm)` / `SIR (dB)` are **stale latches** — on a
//!   channel whose count is provably frozen they still read `detected=1`. Only the count's *rate of
//!   change* is live, so this example reads nothing else.
//! * `DCF energy detect fired` looks like the ideal energy sensor and is **TX-gated**: it only
//!   advances when we have something to send. Useless for passive sensing.
//! * The detector is **dead below 4 MHz** — every 1 MHz and 2 MHz channel reads exactly 0.00,
//!   including ones known to carry the emitter. `--op-bw` therefore defaults to 8 and refuses < 4.
//! * `medium_eval enable` **arms** the detector and returns nothing. Before it, every interference
//!   field reads 0/empty. The loop arms it at startup and says so.
//!
//! # Usage
//!
//! ```text
//! sudo ./halow_cognition sense  --cli PATH [--wlan wlan0] [--window-ms 200] [--n 10]
//! sudo ./halow_cognition map    --cli PATH [--chans 908000,916000,924000] [--window-ms 500] [--passes 3]
//! sudo ./halow_cognition retune --cli PATH [--chans 908000,916000] [--n 20]
//! sudo ./halow_cognition run    --cli PATH --ifspec mon0:morse0 --role tx|rx [...]
//! ```
//!
//! The headline arm, both nodes started within a second of each other:
//!
//! ```text
//! node A:  sudo ./halow_cognition run --cli ~/tools/morse_cli --ifspec mon0:morse0 --role rx \
//!            --start 908000 --chans 908000,916000,924000 --secs 60
//! node B:  sudo ./halow_cognition run --cli ~/tools/morse_cli --ifspec mon0:morse0 --role tx \
//!            --start 908000 --chans 908000,916000,924000 --secs 60 --size 1400 --pps 850
//! ```
//!
//! and the do-nothing control is the same two commands with `--no-move` added to both.
//!
//! ⚠ `morse_cli` has **no internal timeout** and will hang forever on a wedged SPI transport, so
//! every invocation here is killed at `--cli-timeout-ms` and reported as a failure rather than
//! stalling the loop. A hung sense window must not become a stopped radio.

#[cfg(target_os = "linux")]
mod cog {
    use std::io::Read;
    use std::path::PathBuf;
    use std::process::{Command, Stdio};
    use std::sync::Arc;
    use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
    use std::time::{Duration, Instant, SystemTime, UNIX_EPOCH};

    use bytes::Bytes;
    use ndn_radio_drivers::halow::{MORSE_INJECT_BW_PARAM, MORSE_INJECT_MCS_PARAM, MorseFrameIo};
    use ndn_radio_drivers::{
        BROADCAST, DEFAULT_SRC, FrameFormat, FrameIo, InjectFrame, McsDescriptor, TxIntent,
    };

    /// magic(8) + seq(4) + send-µs(8) + len(2) + run-id(2) — the same identifiable payload the rest
    /// of the HaLow harness uses, so a capture can be grepped for *our* frames and one run cannot be
    /// mistaken for another.
    pub const MAGIC: &[u8; 8] = b"NDNONAIR";
    pub const HDR: usize = 24;

    // ───────────────────────────── options ─────────────────────────────

    #[derive(Clone, Debug)]
    pub struct Opts {
        pub cli: PathBuf,
        pub wlan: String,
        pub ifspec: String,
        pub role: Role,
        /// Candidate channels, in kHz. **Order is the tie-break**, and both ends must be given the
        /// same list or they cannot converge without talking to each other.
        pub chans: Vec<u32>,
        pub start: Option<u32>,
        pub op_bw: u8,
        pub pri_bw: u8,
        pub pri_idx: u8,
        pub size: usize,
        pub pps: u32,
        pub mcs: u8,
        pub inject_bw: u8,
        pub window_ms: u64,
        pub period_ms: u64,
        pub thresh: u64,
        pub strikes: u32,
        pub min_dwell_s: u64,
        pub secs: u64,
        pub no_move: bool,
        pub no_sense: bool,
        /// Arm `medium_eval` only for the sensing window and disarm immediately after.
        pub arm_per_window: bool,
        pub follow: bool,
        pub silence_ms: u64,
        pub probe_ms: u64,
        pub cli_timeout_ms: u64,
        pub n: u32,
        pub passes: u32,
        pub label: String,
    }

    #[derive(Clone, Copy, Debug, PartialEq, Eq)]
    pub enum Role {
        Tx,
        Rx,
        None,
    }

    impl Default for Opts {
        fn default() -> Self {
            Self {
                cli: PathBuf::from("morse_cli"),
                wlan: "wlan0".into(),
                ifspec: "mon0:morse0".into(),
                role: Role::None,
                chans: vec![908_000, 916_000, 924_000],
                start: None,
                op_bw: 8,
                pri_bw: 1,
                pri_idx: 0,
                size: 1400,
                pps: 850,
                mcs: 7,
                inject_bw: 8,
                window_ms: 200,
                period_ms: 1000,
                thresh: 2,
                strikes: 2,
                min_dwell_s: 10,
                secs: 60,
                no_move: false,
                no_sense: false,
                arm_per_window: false,
                follow: false,
                silence_ms: 400,
                probe_ms: 500,
                cli_timeout_ms: 20_000,
                n: 10,
                passes: 3,
                label: String::new(),
            }
        }
    }

    fn take<'a>(a: &'a [String], i: &mut usize, k: &str) -> Result<&'a str, String> {
        *i += 1;
        a.get(*i)
            .map(String::as_str)
            .ok_or_else(|| format!("{k} needs a value"))
    }

    pub fn parse(args: &[String]) -> Result<Opts, String> {
        let mut o = Opts::default();
        let mut i = 0usize;
        while i < args.len() {
            match args[i].as_str() {
                "--cli" => o.cli = PathBuf::from(take(args, &mut i, "--cli")?),
                "--wlan" => o.wlan = take(args, &mut i, "--wlan")?.to_string(),
                "--ifspec" => o.ifspec = take(args, &mut i, "--ifspec")?.to_string(),
                "--role" => {
                    o.role = match take(args, &mut i, "--role")? {
                        "tx" => Role::Tx,
                        "rx" => Role::Rx,
                        "none" => Role::None,
                        v => return Err(format!("--role tx|rx|none, not {v}")),
                    }
                }
                "--chans" => {
                    o.chans = take(args, &mut i, "--chans")?
                        .split(',')
                        .map(|s| s.trim().parse::<u32>().map_err(|e| e.to_string()))
                        .collect::<Result<_, _>>()?;
                    if o.chans.is_empty() {
                        return Err("--chans is empty".into());
                    }
                }
                "--start" => {
                    o.start = Some(
                        take(args, &mut i, "--start")?
                            .parse()
                            .map_err(|_| "bad --start")?,
                    )
                }
                "--op-bw" => {
                    o.op_bw = take(args, &mut i, "--op-bw")?
                        .parse()
                        .map_err(|_| "bad --op-bw")?
                }
                "--pri-idx" => {
                    o.pri_idx = take(args, &mut i, "--pri-idx")?
                        .parse()
                        .map_err(|_| "bad --pri-idx")?
                }
                "--size" => {
                    o.size = take(args, &mut i, "--size")?
                        .parse()
                        .map_err(|_| "bad --size")?
                }
                "--pps" => {
                    o.pps = take(args, &mut i, "--pps")?
                        .parse()
                        .map_err(|_| "bad --pps")?
                }
                "--mcs" => {
                    o.mcs = take(args, &mut i, "--mcs")?
                        .parse()
                        .map_err(|_| "bad --mcs")?
                }
                "--inject-bw" => {
                    o.inject_bw = take(args, &mut i, "--inject-bw")?
                        .parse()
                        .map_err(|_| "bad --inject-bw")?
                }
                "--window-ms" => {
                    o.window_ms = take(args, &mut i, "--window-ms")?
                        .parse()
                        .map_err(|_| "bad --window-ms")?
                }
                "--period-ms" => {
                    o.period_ms = take(args, &mut i, "--period-ms")?
                        .parse()
                        .map_err(|_| "bad --period-ms")?
                }
                "--thresh" => {
                    o.thresh = take(args, &mut i, "--thresh")?
                        .parse()
                        .map_err(|_| "bad --thresh")?
                }
                "--strikes" => {
                    o.strikes = take(args, &mut i, "--strikes")?
                        .parse()
                        .map_err(|_| "bad --strikes")?
                }
                "--min-dwell-s" => {
                    o.min_dwell_s = take(args, &mut i, "--min-dwell-s")?
                        .parse()
                        .map_err(|_| "bad --min-dwell-s")?
                }
                "--secs" => {
                    o.secs = take(args, &mut i, "--secs")?
                        .parse()
                        .map_err(|_| "bad --secs")?
                }
                "--no-move" => o.no_move = true,
                "--no-sense" => o.no_sense = true,
                "--arm-per-window" => o.arm_per_window = true,
                "--follow" => o.follow = true,
                "--silence-ms" => {
                    o.silence_ms = take(args, &mut i, "--silence-ms")?
                        .parse()
                        .map_err(|_| "bad --silence-ms")?
                }
                "--probe-ms" => {
                    o.probe_ms = take(args, &mut i, "--probe-ms")?
                        .parse()
                        .map_err(|_| "bad --probe-ms")?
                }
                "--cli-timeout-ms" => {
                    o.cli_timeout_ms = take(args, &mut i, "--cli-timeout-ms")?
                        .parse()
                        .map_err(|_| "bad --cli-timeout-ms")?
                }
                "--n" => o.n = take(args, &mut i, "--n")?.parse().map_err(|_| "bad --n")?,
                "--passes" => {
                    o.passes = take(args, &mut i, "--passes")?
                        .parse()
                        .map_err(|_| "bad --passes")?
                }
                "--label" => o.label = take(args, &mut i, "--label")?.to_string(),
                v => return Err(format!("unknown option {v}")),
            }
            i += 1;
        }
        // ☠ The interference detector reads exactly 0.00 on every 1 MHz and 2 MHz channel, including
        // ones carrying a strong emitter. A narrow operating width is not a clean band, it is an
        // unarmed instrument — refuse it rather than report zeros.
        // ...but only for a node that *senses*. A node running `--no-sense` is either a pure
        // data-plane follower or a deliberate narrowband emitter, and both are legitimate at 1/2
        // MHz. Binding them to >= 4 MHz would make it impossible to reproduce the very interferer
        // this loop exists to detect.
        if o.op_bw < 4 && !o.no_sense {
            return Err(format!(
                "--op-bw {} : the narrowband-interference detector is DEAD below 4 MHz (all 1/2 MHz \
                 channels read 0.00 even with a strong emitter present). Sense at >= 4 MHz, or pass \
                 --no-sense if this node is not the sensor.",
                o.op_bw
            ));
        }
        Ok(o)
    }

    // ───────────────────────────── morse_cli, timed and killable ─────────────────────────────

    /// One `morse_cli` invocation with a hard deadline.
    ///
    /// ⚠ `morse_cli` has no internal timeout: on a wedged SPI transport it blocks forever. A
    /// cognition loop that can be stopped by its own sensor is not a cognition loop, so the child is
    /// killed at the deadline and the failure is returned like any other.
    pub struct Cli {
        path: PathBuf,
        iface: String,
        timeout: Duration,
        /// Cumulative wall time spent inside `morse_cli`, which is the honest cost of sensing:
        /// every one of these round trips shares the SPI bus with the data plane.
        pub busy: Duration,
        pub calls: u64,
        pub timeouts: u64,
    }

    impl Cli {
        pub fn new(path: PathBuf, iface: &str, timeout_ms: u64) -> Self {
            Self {
                path,
                iface: iface.to_string(),
                timeout: Duration::from_millis(timeout_ms),
                busy: Duration::ZERO,
                calls: 0,
                timeouts: 0,
            }
        }

        fn run(&mut self, args: &[&str]) -> Result<(String, Duration), String> {
            let t0 = Instant::now();
            let mut child = Command::new(&self.path)
                .arg("-i")
                .arg(&self.iface)
                .args(args)
                .stdout(Stdio::piped())
                .stderr(Stdio::piped())
                .spawn()
                .map_err(|e| format!("spawn {}: {e}", self.path.display()))?;
            let mut killed = false;
            loop {
                match child.try_wait() {
                    Ok(Some(_)) => break,
                    Ok(None) => {
                        if t0.elapsed() > self.timeout {
                            let _ = child.kill();
                            let _ = child.wait();
                            killed = true;
                            break;
                        }
                        std::thread::sleep(Duration::from_millis(2));
                    }
                    Err(e) => return Err(format!("wait: {e}")),
                }
            }
            let mut out = String::new();
            if let Some(mut s) = child.stdout.take() {
                let _ = s.read_to_string(&mut out);
            }
            let d = t0.elapsed();
            self.calls += 1;
            self.busy += d;
            if killed {
                self.timeouts += 1;
                return Err(format!(
                    "morse_cli {args:?} TIMED OUT after {:?}",
                    self.timeout
                ));
            }
            Ok((out, d))
        }

        /// Arm the narrowband-interference detector. Returns whether the command succeeded; the
        /// verb prints nothing on success, so its own output is not evidence — the counter
        /// appearing in `stats` is.
        pub fn medium_eval(&mut self, on: bool) -> Result<(), String> {
            self.run(&["medium_eval", if on { "enable" } else { "disable" }])
                .map(|_| ())
        }

        /// The one live sensing field. Everything else in the interference block is a stale latch.
        pub fn interference_count(&mut self) -> Result<u64, String> {
            let (out, _) = self.run(&["stats"])?;
            for l in out.lines() {
                if let Some(rest) = l.strip_prefix("Narrowband interference count") {
                    if let Some((_, v)) = rest.split_once(':') {
                        return v
                            .trim()
                            .parse::<u64>()
                            .map_err(|e| format!("parse {v:?}: {e}"));
                    }
                }
            }
            Err("no 'Narrowband interference count' line in stats — is medium_eval armed?".into())
        }

        /// Retune, **readback-verified from the command's own report**. The four numbers are not
        /// sticky and must match on both ends; a move that silently did not happen is the worst
        /// possible outcome for this loop, so a mismatch is an error, not a warning.
        pub fn set_channel(
            &mut self,
            khz: u32,
            op: u8,
            pri: u8,
            idx: u8,
        ) -> Result<Duration, String> {
            let (k, o, p, n) = (
                khz.to_string(),
                op.to_string(),
                pri.to_string(),
                idx.to_string(),
            );
            let (out, d) = self.run(&["channel", "-c", &k, "-o", &o, "-p", &p, "-n", &n])?;
            let got = out
                .lines()
                .find_map(|l| l.trim().strip_prefix("Operating Frequency:"))
                .and_then(|v| v.trim().split_whitespace().next())
                .and_then(|v| v.parse::<u32>().ok());
            match got {
                Some(f) if f == khz => {}
                Some(f) => return Err(format!("retune asked {khz} kHz, radio reports {f} kHz")),
                None => return Err(format!("retune produced no readback:\n{out}")),
            }
            // ☠ **The command's own echo is not the radio.** MEASURED on this bench: with the
            // transmitter running flat out, the vendor command channel times out
            // (`morse_cmd_vendor 07:8521 timed out`, `Late response for timed out req`) and the
            // radio is left on a DIFFERENT frequency AND a DIFFERENT operating width — 915000 kHz
            // / 2 MHz after a `-c 916000 -o 8` that reported success. Both ends then sit on
            // channels that do not exist in the candidate list and the link is simply gone, with
            // every log line claiming the move succeeded. So the state is re-read with an
            // independent command, and the width is checked as well as the frequency.
            let (f2, bw2) = self.read_channel()?;
            if f2 != khz || bw2 != op {
                return Err(format!(
                    "retune to {khz} kHz / {op} MHz reported success but the radio reads back \
                     {f2} kHz / {bw2} MHz — the vendor command was lost mid-flight"
                ));
            }
            Ok(d)
        }

        pub fn get_channel(&mut self) -> Result<u32, String> {
            Ok(self.read_channel()?.0)
        }

        /// `(freq_khz, op_bw_mhz)` read back from the radio with an **independent** command.
        fn read_channel(&mut self) -> Result<(u32, u8), String> {
            let (out, _) = self.run(&["channel"])?;
            let f = out
                .lines()
                .find_map(|l| l.trim().strip_prefix("Operating Frequency:"))
                .and_then(|v| v.trim().split_whitespace().next())
                .and_then(|v| v.parse::<u32>().ok())
                .ok_or_else(|| format!("no frequency in:\n{out}"))?;
            let bw = out
                .lines()
                .find_map(|l| l.trim().strip_prefix("Operating BW:"))
                .and_then(|v| v.trim().split_whitespace().next())
                .and_then(|v| v.parse::<u8>().ok())
                .ok_or_else(|| format!("no operating BW in:\n{out}"))?;
            Ok((f, bw))
        }
    }

    // ───────────────────────────── sensing ─────────────────────────────

    /// One sense window: two counter reads `window` apart.
    ///
    /// The returned `elapsed` is the **whole** window including both `morse_cli` round trips,
    /// because that is what the loop actually pays. `events` is the raw difference — the headline
    /// unit is events, not a rate, because the threshold is compared against a fixed window and a
    /// rate would hide how few samples it rests on.
    pub struct Window {
        pub events: u64,
        pub elapsed: Duration,
        pub dwell: Duration,
    }

    /// ☠ **`medium_eval enable` is not free and not passive.** Leaving the narrowband detector
    /// armed costs this receiver **8.55 -> 6.6 Mbit/s of delivered payload (-21.4 %, PDR 99.5 % ->
    /// 78.1 %)**, measured with the sensor making no reads at all — arming alone does it. The cost
    /// persists after the loop stops and is undone by `medium_eval disable` (an SPI unbind/bind is
    /// not required, though it also works).
    ///
    /// So the detector must be **duty-cycled**: armed for the window, disarmed the instant it is
    /// read. The armed fraction is then the throughput one pays, and it is a knob
    /// (`--window-ms` / `--period-ms`) rather than a permanent 21 % tax.
    pub fn sense_armed(cli: &mut Cli, window: Duration) -> Result<Window, String> {
        let t0 = Instant::now();
        cli.medium_eval(true)?;
        let a = cli.interference_count()?;
        let t_a = Instant::now();
        std::thread::sleep(window);
        let b = cli.interference_count()?;
        let dwell = t_a.elapsed();
        // Disarm even if the read failed: a sensor that fails armed is a 21 % throughput leak.
        let _ = cli.medium_eval(false);
        Ok(Window {
            events: b.saturating_sub(a),
            elapsed: t0.elapsed(),
            dwell,
        })
    }

    pub fn sense(cli: &mut Cli, window: Duration) -> Result<Window, String> {
        let t0 = Instant::now();
        let a = cli.interference_count()?;
        let t_a = Instant::now();
        std::thread::sleep(window);
        let b = cli.interference_count()?;
        // A retune can reset the counter; a negative difference is a reset, not negative energy.
        let events = b.saturating_sub(a);
        Ok(Window {
            events,
            elapsed: t0.elapsed(),
            dwell: t_a.elapsed(),
        })
    }

    // ───────────────────────────── traffic ─────────────────────────────

    /// ★ **The metric the emitter actually moves.** It costs ~2.9 % of airtime — a rounding error
    /// in Mbit/s — but takes p99 frame *placement* from 52 us to 1934 us, because every deferral is
    /// a full DIFS+backoff the transmitter did not plan for. A throughput-only readout of this
    /// experiment would conclude, wrongly, that the interferer is harmless.
    ///
    /// TX side: microseconds inside `inject()` — submit to sendto-returned.
    /// RX side: `recv_us - send_us`, whose *spread* is meaningful even though the two hosts' clocks
    /// are not synchronised (a constant offset cancels in every percentile difference).
    #[derive(Default)]
    pub struct Latency {
        pub v: std::sync::Mutex<Vec<(f64, u32)>>,
    }

    impl Latency {
        pub fn push(&self, t: f64, us: u32) {
            if let Ok(mut g) = self.v.lock() {
                g.push((t, us));
            }
        }
        /// p50 / p99 / max over the samples whose timestamp falls in `[a, b)`, in microseconds.
        pub fn window(&self, a: f64, b: f64) -> Option<(usize, u32, u32, u32)> {
            let g = self.v.lock().ok()?;
            let mut w: Vec<u32> = g
                .iter()
                .filter(|&&(t, _)| t >= a && t < b)
                .map(|&(_, u)| u)
                .collect();
            if w.is_empty() {
                return None;
            }
            w.sort_unstable();
            let n = w.len();
            Some((n, w[n / 2], w[(n * 99 / 100).min(n - 1)], w[n - 1]))
        }
    }

    #[derive(Default)]
    pub struct Counters {
        pub frames: AtomicU64,
        pub bytes: AtomicU64,
        pub errs: AtomicU64,
        /// Host µs of the most recent RX frame — the only way to see the outage at sub-second
        /// resolution, which is exactly the resolution the answer lives at.
        pub last_us: AtomicU64,
    }

    pub fn now_us() -> u64 {
        SystemTime::now()
            .duration_since(UNIX_EPOCH)
            .unwrap_or_default()
            .as_micros() as u64
    }

    pub fn build_payload(seq: u32, run: u16, size: usize) -> Bytes {
        let mut v = vec![0u8; size.max(HDR)];
        v[0..8].copy_from_slice(MAGIC);
        v[8..12].copy_from_slice(&seq.to_le_bytes());
        v[12..20].copy_from_slice(&now_us().to_le_bytes());
        v[20..22].copy_from_slice(&(size as u16).to_le_bytes());
        v[22..24].copy_from_slice(&run.to_le_bytes());
        for (i, b) in v.iter_mut().enumerate().skip(HDR) {
            *b = (i % 251) as u8;
        }
        Bytes::from(v)
    }

    pub fn is_ours(p: &[u8]) -> bool {
        p.len() >= HDR && &p[0..8] == MAGIC
    }

    // ───────────────────────────── the loop ─────────────────────────────

    /// What the policy decided, and every instant that decision passed through. Printed as one line
    /// per event so the end-to-end latency can be read off without a second tool.
    pub struct Move {
        pub t_detect: Instant,
        pub t_scan_done: Instant,
        pub t_moved: Instant,
        pub from: u32,
        pub to: u32,
        pub events: u64,
        pub scan: Vec<(u32, u64)>,
    }

    /// Score every candidate except `skip`, one dwell each. Returns the map in candidate order, so
    /// the tie-break is the caller's list order and is therefore identical on both ends.
    pub fn score(
        cli: &mut Cli,
        o: &Opts,
        skip: u32,
        window: Duration,
    ) -> Result<Vec<(u32, u64)>, String> {
        let mut out = Vec::new();
        for &f in &o.chans {
            if f == skip {
                continue;
            }
            cli.set_channel(f, o.op_bw, o.pri_bw, o.pri_idx)?;
            let w = sense(cli, window)?;
            out.push((f, w.events));
        }
        Ok(out)
    }

    pub struct Loop {
        pub o: Opts,
        pub cur: u32,
        pub strikes: u32,
        pub last_move: Instant,
        /// Every move this loop made, oldest first — the flap record.
        pub moves: Vec<(f64, u32, u32)>,
        pub windows: u64,
        pub contaminated_windows: u64,
    }

    impl Loop {
        pub fn new(o: Opts, cur: u32) -> Self {
            Self {
                o,
                cur,
                strikes: 0,
                // Start the dwell clock in the past so the first decision is not blocked by the
                // hysteresis that exists to stop the *second* one.
                last_move: Instant::now() - Duration::from_secs(3600),
                moves: Vec::new(),
                windows: 0,
                contaminated_windows: 0,
            }
        }

        /// One turn of the loop. Returns `Some(Move)` if the radio moved.
        pub fn step(&mut self, cli: &mut Cli, paused: &AtomicBool) -> Result<Option<Move>, String> {
            let w = if self.o.arm_per_window {
                sense_armed(cli, Duration::from_millis(self.o.window_ms))?
            } else {
                sense(cli, Duration::from_millis(self.o.window_ms))?
            };
            self.windows += 1;
            let hot = w.events >= self.o.thresh;
            if hot {
                self.contaminated_windows += 1;
                self.strikes += 1;
            } else {
                self.strikes = 0;
            }
            println!(
                "  [{:>8.3}s] sense ch={} events={} ({:.2} ev/s) strikes={}/{}",
                wall(),
                self.cur,
                w.events,
                w.events as f64 / w.dwell.as_secs_f64(),
                self.strikes,
                self.o.strikes
            );
            if self.strikes < self.o.strikes {
                return Ok(None);
            }
            // ── hysteresis: a dwell budget on *moving*, not just on sensing ──────────────────
            if self.last_move.elapsed() < Duration::from_secs(self.o.min_dwell_s) {
                println!(
                    "  [{:>8.3}s] HOLD  ch={} — contaminated but only {:.1}s since the last move \
                     (min-dwell {}s); a flapping radio is worse than a static one",
                    wall(),
                    self.cur,
                    self.last_move.elapsed().as_secs_f64(),
                    self.o.min_dwell_s
                );
                return Ok(None);
            }
            if self.o.no_move {
                println!(
                    "  [{:>8.3}s] NO-MOVE control: would have left {} (events={})",
                    wall(),
                    self.cur,
                    w.events
                );
                self.strikes = 0;
                return Ok(None);
            }
            let t_detect = Instant::now();
            // The scan is off-channel: nothing of ours is heard while it runs, so the traffic is
            // paused to keep the outage attributable rather than smeared into loss.
            paused.store(true, Ordering::Relaxed);
            let scan = score(
                cli,
                &self.o,
                self.cur,
                Duration::from_millis(self.o.window_ms),
            )?;
            let t_scan_done = Instant::now();
            let best = scan
                .iter()
                .copied()
                .min_by_key(|&(f, e)| (e, self.o.chans.iter().position(|&c| c == f).unwrap_or(0)))
                .ok_or("no candidate other than the current channel")?;
            let from = self.cur;
            // Refuse to move to something no better than where we are — the move costs airtime.
            if best.1 >= w.events {
                println!(
                    "  [{:>8.3}s] STAY  ch={} — best alternative {} scored {} vs our {}; moving \
                     would cost airtime for nothing",
                    wall(),
                    from,
                    best.0,
                    best.1,
                    w.events
                );
                cli.set_channel(from, self.o.op_bw, self.o.pri_bw, self.o.pri_idx)?;
                paused.store(false, Ordering::Relaxed);
                self.strikes = 0;
                self.last_move = Instant::now();
                return Ok(None);
            }
            cli.set_channel(best.0, self.o.op_bw, self.o.pri_bw, self.o.pri_idx)?;
            let t_moved = Instant::now();
            paused.store(false, Ordering::Relaxed);
            self.cur = best.0;
            self.strikes = 0;
            self.last_move = t_moved;
            self.moves.push((wall(), from, best.0));
            let m = Move {
                t_detect,
                t_scan_done,
                t_moved,
                from,
                to: best.0,
                events: w.events,
                scan,
            };
            println!(
                "  [{:>8.3}s] MOVE  {} -> {}  (was {} ev/{}ms; map {:?})  t_scan={:.3}s \
                 t_retune={:.3}s",
                wall(),
                m.from,
                m.to,
                m.events,
                self.o.window_ms,
                m.scan,
                (m.t_scan_done - m.t_detect).as_secs_f64(),
                (m.t_moved - m.t_scan_done).as_secs_f64()
            );
            Ok(Some(m))
        }
    }

    static T0: std::sync::OnceLock<Instant> = std::sync::OnceLock::new();
    pub fn start_clock() {
        let _ = T0.set(Instant::now());
    }
    pub fn wall() -> f64 {
        T0.get().map(|t| t.elapsed().as_secs_f64()).unwrap_or(0.0)
    }

    // ───────────────────────────── face plumbing ─────────────────────────────

    pub fn open_face(o: &Opts) -> Result<MorseFrameIo, Box<dyn std::error::Error>> {
        let (tx, rx) = o
            .ifspec
            .split_once(':')
            .ok_or("--ifspec must be <tx>:<rx> (Morse split data plane, e.g. mon0:morse0)")?;
        let mut f = MorseFrameIo::new(tx, rx, FrameFormat::RawNdnS1g { ethertype: 0x8624 })?;
        let mp = PathBuf::from(MORSE_INJECT_MCS_PARAM);
        if mp.exists() {
            f = f.with_inject_mcs_param(Some(mp))?;
        } else {
            eprintln!(
                "!! {MORSE_INJECT_MCS_PARAM} absent: the monitor-injection patch is not loaded, so \
                 the rate will NOT reach the air."
            );
        }
        let bp = PathBuf::from(MORSE_INJECT_BW_PARAM);
        if bp.exists() {
            f = f.with_inject_bw_param(Some(bp))?;
        }
        f.set_rate(McsDescriptor {
            index: o.mcs,
            short_gi: false,
            vht: false,
            nss: 1,
            stbc: false,
            ldpc: false,
            he: false,
            dcm: false,
            er_su: false,
        })?;
        let _ = f.set_inject_bw_mhz(o.inject_bw);
        Ok(f)
    }

    pub fn spawn_tx(
        face: Arc<MorseFrameIo>,
        o: Opts,
        c: Arc<Counters>,
        lat: Arc<Latency>,
        paused: Arc<AtomicBool>,
        stop: Arc<AtomicBool>,
        run: u16,
    ) -> tokio::task::JoinHandle<()> {
        tokio::spawn(async move {
            let gap = if o.pps == 0 {
                Duration::ZERO
            } else {
                Duration::from_nanos(1_000_000_000 / o.pps as u64)
            };
            let mut seq = 0u32;
            let mut next = Instant::now();
            while !stop.load(Ordering::Relaxed) {
                if paused.load(Ordering::Relaxed) {
                    tokio::time::sleep(Duration::from_millis(2)).await;
                    next = Instant::now();
                    continue;
                }
                let f = InjectFrame {
                    payload: build_payload(seq, run, o.size),
                    tx: TxIntent::CONSERVATIVE,
                    dst: BROADCAST,
                    src: DEFAULT_SRC,
                    addr3: None,
                    extra: None,
                    htc: None,
                };
                let t_sub = Instant::now();
                match face.inject(f).await {
                    Ok(()) => {
                        if seq == 0 {
                            println!(
                                "  FIRST FRAME ON AIR epoch={:.6}",
                                SystemTime::now()
                                    .duration_since(UNIX_EPOCH)
                                    .map(|d| d.as_secs_f64())
                                    .unwrap_or(0.0)
                            );
                        }
                        lat.push(wall(), t_sub.elapsed().as_micros() as u32);
                        c.frames.fetch_add(1, Ordering::Relaxed);
                        c.bytes.fetch_add(o.size as u64, Ordering::Relaxed);
                    }
                    Err(_) => {
                        c.errs.fetch_add(1, Ordering::Relaxed);
                    }
                }
                seq = seq.wrapping_add(1);
                if gap.is_zero() {
                    tokio::task::yield_now().await;
                } else {
                    next += gap;
                    let now = Instant::now();
                    if next > now {
                        tokio::time::sleep(next - now).await;
                    } else {
                        next = now;
                    }
                }
            }
        })
    }

    pub fn spawn_rx(
        face: Arc<MorseFrameIo>,
        c: Arc<Counters>,
        lat: Arc<Latency>,
        stop: Arc<AtomicBool>,
    ) -> tokio::task::JoinHandle<()> {
        tokio::spawn(async move {
            while !stop.load(Ordering::Relaxed) {
                match tokio::time::timeout(Duration::from_millis(200), face.recv_frame()).await {
                    Ok(Ok(f)) => {
                        if is_ours(&f.payload) {
                            let sent = u64::from_le_bytes(f.payload[12..20].try_into().unwrap());
                            let d = now_us().saturating_sub(sent);
                            if d < 10_000_000 {
                                lat.push(wall(), d as u32);
                            }
                            c.frames.fetch_add(1, Ordering::Relaxed);
                            c.bytes.fetch_add(f.payload.len() as u64, Ordering::Relaxed);
                            c.last_us.store(now_us(), Ordering::Relaxed);
                        }
                    }
                    Ok(Err(_)) => {
                        c.errs.fetch_add(1, Ordering::Relaxed);
                    }
                    Err(_) => {}
                }
            }
        })
    }
}

#[cfg(target_os = "linux")]
#[tokio::main(flavor = "multi_thread", worker_threads = 4)]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    use cog::*;
    use std::sync::Arc;
    use std::sync::atomic::{AtomicBool, Ordering};
    use std::time::{Duration, Instant, SystemTime, UNIX_EPOCH};

    let argv: Vec<String> = std::env::args().collect();
    if argv.len() < 2 {
        eprintln!(
            "usage:\n  halow_cognition sense  --cli PATH [--wlan wlan0] [--window-ms 200] [--n 10]\n\
             \x20 halow_cognition trace  --cli PATH [--secs 30] [--period-ms 0]\n\
             \x20 halow_cognition map    --cli PATH [--chans a,b,c] [--window-ms 500] [--passes 3]\n\
             \x20 halow_cognition retune --cli PATH [--chans a,b] [--n 20]\n\
             \x20 halow_cognition run    --cli PATH --ifspec mon0:morse0 --role tx|rx [...]\n\n\
             run options: --start KHZ --chans a,b,c --op-bw 8 --size N --pps N --mcs N\n\
             \x20            --window-ms N --period-ms N --thresh N --strikes N --min-dwell-s N\n\
             \x20            --secs N --no-move --label TEXT\n"
        );
        std::process::exit(2);
    }
    let mode = argv[1].clone();
    let o = parse(&argv[2..]).map_err(|e| format!("{e}"))?;
    start_clock();
    let mut cli = Cli::new(o.cli.clone(), &o.wlan, o.cli_timeout_ms);

    // ★ Arming is not optional and not silent-by-default: before `medium_eval enable` every
    // interference field reads 0/empty, which is indistinguishable from a clean band.
    if o.no_sense {
        // A node with no sensor must not arm one. On this bench only the MM6108 running the
        // telemetry firmware exposes the counter at all; on the other, `stats` returns two lines.
        println!("  medium_eval      : NOT armed — this node senses nothing (--no-sense)");
        // ...and make sure a previous run did not leave the 21 % tax armed on this radio.
        let _ = cli.medium_eval(false);
    } else if o.arm_per_window {
        println!(
            "  medium_eval      : DUTY-CYCLED — armed only for each {}ms window and disarmed \
             immediately (leaving it armed costs this receiver 21.4 % of delivered Mbit/s)",
            o.window_ms
        );
        let _ = cli.medium_eval(false);
    } else {
        match cli.medium_eval(true) {
            Ok(()) => println!("  medium_eval      : armed (the detector is OFF until this runs)"),
            Err(e) => {
                println!("!! medium_eval enable failed: {e} — readings below are not trustworthy")
            }
        }
    }
    let cur = cli.get_channel()?;
    println!("  radio            : {} via {}", o.wlan, o.cli.display());
    println!(
        "  channel at start : {cur} kHz, op {} MHz, primary idx {}",
        o.op_bw, o.pri_idx
    );
    if !o.label.is_empty() {
        println!("  label            : {}", o.label);
    }

    match mode.as_str() {
        // ── instrument check: N windows on whatever channel we are on ────────────────────────
        "sense" => {
            let w = Duration::from_millis(o.window_ms);
            let mut ev = Vec::new();
            for k in 0..o.n {
                let s = if o.arm_per_window { sense_armed(&mut cli, w)? } else { sense(&mut cli, w)? };
                ev.push(s.events);
                println!(
                    "  {k:>3}  events={:<4} dwell={:.3}s  window_cost={:.3}s  {:.2} ev/s",
                    s.events,
                    s.dwell.as_secs_f64(),
                    s.elapsed.as_secs_f64(),
                    s.events as f64 / s.dwell.as_secs_f64()
                );
            }
            let n = ev.len() as f64;
            let mean = ev.iter().sum::<u64>() as f64 / n;
            let sd = (ev.iter().map(|&e| (e as f64 - mean).powi(2)).sum::<f64>() / n).sqrt();
            println!(
                "  ch {cur}: mean {mean:.2} +/- {sd:.2} events per {}ms window  (min {} max {})",
                o.window_ms,
                ev.iter().min().unwrap_or(&0),
                ev.iter().max().unwrap_or(&0)
            );
            println!(
                "  cli busy         : {:.3}s over {} calls ({} timeouts)",
                cli.busy.as_secs_f64(),
                cli.calls,
                cli.timeouts
            );
        }

        // ── raw timestamped counter poll: recover the interferer's PERIOD, with no decode ────
        //
        // `sense` reports a *window*, which is enough to decide but throws away the one piece of
        // evidence that distinguishes a real periodic emitter from a noisy detector: **when** the
        // increments happen. A 100 TU beacon is 102.400 ms apart. If the gaps between counter
        // increments pile up at 102.4 ms and its integer multiples, the counter is tracking that
        // emitter and not drifting — and that is provable from the counter alone, on a channel
        // where packet capture yields nothing.
        //
        // Sampling is as fast as `morse_cli stats` allows (~11 ms), i.e. ~9 samples per beacon
        // period, so the period is resolved rather than aliased. `--period-ms 0` = flat out.
        "trace" => {
            let t0 = Instant::now();
            let deadline = Duration::from_secs(o.secs);
            let mut last: Option<u64> = None;
            let mut samples = 0u64;
            let mut errs = 0u64;
            println!("# trace ch={cur} kHz op={} MHz label={}", o.op_bw, o.label);
            println!("# epoch_s\tt_s\tcount\tdelta\tread_ms");
            while t0.elapsed() < deadline {
                let ts = t0.elapsed().as_secs_f64();
                let r0 = Instant::now();
                match cli.interference_count() {
                    Ok(c) => {
                        // The read straddles an interval; stamp it at the *end*, which is when the
                        // chip's value was latched into our process.
                        let rd = r0.elapsed();
                        let d = last.map(|l| c.saturating_sub(l)).unwrap_or(0);
                        let epoch = SystemTime::now()
                            .duration_since(UNIX_EPOCH)
                            .map(|d| d.as_secs_f64())
                            .unwrap_or(0.0);
                        println!(
                            "{epoch:.6}\t{:.6}\t{c}\t{d}\t{:.2}",
                            ts + rd.as_secs_f64(),
                            rd.as_secs_f64() * 1e3
                        );
                        last = Some(c);
                        samples += 1;
                    }
                    Err(e) => {
                        errs += 1;
                        println!("# {ts:.6} ERR {e}");
                    }
                }
                if o.period_ms > 0 {
                    std::thread::sleep(Duration::from_millis(o.period_ms));
                }
            }
            println!(
                "# {samples} samples, {errs} errors, {:.3}s cli busy over {} calls ({} timeouts), \
                 mean sample period {:.1} ms",
                cli.busy.as_secs_f64(),
                cli.calls,
                cli.timeouts,
                t0.elapsed().as_secs_f64() * 1e3 / samples.max(1) as f64
            );
        }

        // ── the occupancy map, with its measured wall cost ───────────────────────────────────
        "map" => {
            let w = Duration::from_millis(o.window_ms);
            let t0 = Instant::now();
            let mut acc: Vec<Vec<u64>> = vec![Vec::new(); o.chans.len()];
            for _ in 0..o.passes {
                for (i, &f) in o.chans.iter().enumerate() {
                    cli.set_channel(f, o.op_bw, o.pri_bw, o.pri_idx)?;
                    acc[i].push(if o.arm_per_window { sense_armed(&mut cli, w)?.events } else { sense(&mut cli, w)?.events });
                }
            }
            let el = t0.elapsed();
            println!(
                "  ── occupancy map, {} passes, {}ms dwell ──",
                o.passes, o.window_ms
            );
            for (i, &f) in o.chans.iter().enumerate() {
                let v = &acc[i];
                let n = v.len() as f64;
                let mean = v.iter().sum::<u64>() as f64 / n;
                let sd = (v.iter().map(|&e| (e as f64 - mean).powi(2)).sum::<f64>() / n).sqrt();
                let rate = mean / (o.window_ms as f64 / 1000.0);
                println!(
                    "  {f:>7} kHz  {mean:6.2} +/- {sd:4.2} ev/window  = {rate:6.2} ev/s   {}",
                    if mean >= o.thresh as f64 {
                        "CONTAMINATED"
                    } else {
                        "clean"
                    }
                );
            }
            println!(
                "  full map cost    : {:.2}s for {} channels x {} passes ({:.2}s per sweep)",
                el.as_secs_f64(),
                o.chans.len(),
                o.passes,
                el.as_secs_f64() / o.passes as f64
            );
            cli.set_channel(cur, o.op_bw, o.pri_bw, o.pri_idx)?;
        }

        // ── the actuator, timed on its own ───────────────────────────────────────────────────
        "retune" => {
            let mut ms = Vec::new();
            for k in 0..o.n {
                let target = o.chans[(k as usize) % o.chans.len()];
                let d = cli.set_channel(target, o.op_bw, o.pri_bw, o.pri_idx)?;
                println!(
                    "  {k:>3}  -> {target} kHz  {:.1} ms",
                    d.as_secs_f64() * 1000.0
                );
                ms.push(d.as_secs_f64() * 1000.0);
            }
            ms.sort_by(|a, b| a.partial_cmp(b).unwrap());
            let n = ms.len();
            let mean = ms.iter().sum::<f64>() / n as f64;
            println!(
                "  retune (morse_cli channel, readback-verified), n={n}:\n  \
                 min {:.1} ms  p50 {:.1} ms  p90 {:.1} ms  max {:.1} ms  mean {mean:.1} ms",
                ms[0],
                ms[n / 2],
                ms[(n * 9 / 10).min(n - 1)],
                ms[n - 1]
            );
            cli.set_channel(cur, o.op_bw, o.pri_bw, o.pri_idx)?;
        }

        // ── the actuator, timed WITH TRAFFIC: how long until frames flow again ───────────────
        //
        // `retune` above times the command. This times the *link*: a peer transmits continuously on
        // the home channel, this radio hops away and back, and the gap between the last frame
        // before the hop and the first frame after the return is the receiver's real resume cost.
        // Command time and resume time are different numbers and only one of them is the outage.
        "hop" => {
            let face = Arc::new(open_face(&o)?);
            let c = Arc::new(Counters::default());
            let stop = Arc::new(AtomicBool::new(false));
            let lat = Arc::new(Latency::default());
            let h = spawn_rx(face.clone(), c.clone(), lat.clone(), stop.clone());
            let home = o.start.unwrap_or(cur);
            let alt = *o
                .chans
                .iter()
                .find(|&&f| f != home)
                .ok_or("--chans needs a channel other than home")?;
            cli.set_channel(home, o.op_bw, o.pri_bw, o.pri_idx)?;
            tokio::time::sleep(Duration::from_millis(500)).await;
            if c.frames.load(Ordering::Relaxed) == 0 {
                return Err(
                    "no traffic on the home channel — start the peer transmitter first; \
                            measuring a resume time with nothing to resume is the classic dead \
                            instrument"
                        .into(),
                );
            }
            let mut out = Vec::new();
            let mut cmd = Vec::new();
            for k in 0..o.n {
                // Away.
                let d_away = cli.set_channel(alt, o.op_bw, o.pri_bw, o.pri_idx)?;
                tokio::time::sleep(Duration::from_millis(300)).await;
                let n_before = c.frames.load(Ordering::Relaxed);
                // Home again, and wait for the first frame.
                let d_back = cli.set_channel(home, o.op_bw, o.pri_bw, o.pri_idx)?;
                let t_ret = Instant::now();
                let mut resumed = None;
                while t_ret.elapsed() < Duration::from_secs(3) {
                    if c.frames.load(Ordering::Relaxed) > n_before {
                        resumed = Some(t_ret.elapsed());
                        break;
                    }
                    tokio::time::sleep(Duration::from_millis(1)).await;
                }
                match resumed {
                    Some(r) => {
                        println!(
                            "  {k:>3}  away {:.1} ms  back {:.1} ms  first frame {:.1} ms after the \
                             retune returned",
                            d_away.as_secs_f64() * 1000.0,
                            d_back.as_secs_f64() * 1000.0,
                            r.as_secs_f64() * 1000.0
                        );
                        out.push(r.as_secs_f64() * 1000.0);
                        cmd.push(d_back.as_secs_f64() * 1000.0);
                    }
                    None => println!("  {k:>3}  NO FRAME within 3 s of returning to {home}"),
                }
                tokio::time::sleep(Duration::from_millis(500)).await;
            }
            stop.store(true, Ordering::Relaxed);
            let _ = tokio::time::timeout(Duration::from_millis(500), h).await;
            let mut v = out.clone();
            v.sort_by(|a, b| a.partial_cmp(b).unwrap());
            let mut cv = cmd.clone();
            cv.sort_by(|a, b| a.partial_cmp(b).unwrap());
            if !v.is_empty() {
                let n = v.len();
                println!(
                    "  resume after retune, n={n}: min {:.1} ms  p50 {:.1} ms  p90 {:.1} ms  max {:.1} ms",
                    v[0],
                    v[n / 2],
                    v[(n * 9 / 10).min(n - 1)],
                    v[n - 1]
                );
                println!(
                    "  of which the command itself: p50 {:.1} ms  max {:.1} ms",
                    cv[cv.len() / 2],
                    cv[cv.len() - 1]
                );
                let mean_out = v.iter().sum::<f64>() / n as f64;
                println!(
                    "  ⇒ payload foregone per move at {} B / {} f/s: {:.1} kbit ({:.3} Mbit/s x {:.3} s)",
                    o.size,
                    o.pps,
                    (o.size as f64 * 8.0 * o.pps as f64 * mean_out / 1000.0) / 1000.0,
                    o.size as f64 * 8.0 * o.pps as f64 / 1e6,
                    mean_out / 1000.0
                );
            }
        }

        // ── the loop, with traffic ───────────────────────────────────────────────────────────
        "run" => {
            let face = Arc::new(open_face(&o)?);
            println!("  rate_actuated    : {}", face.rate_actuated());
            println!(
                "  policy           : thresh {} ev / {}ms, strikes {}, min-dwell {}s, period {}ms{}",
                o.thresh,
                o.window_ms,
                o.strikes,
                o.min_dwell_s,
                o.period_ms,
                if o.no_move { "  [NO-MOVE CONTROL]" } else { "" }
            );
            let start = o.start.unwrap_or(cur);
            let t_onset = {
                cli.set_channel(start, o.op_bw, o.pri_bw, o.pri_idx)?;
                Instant::now()
            };
            println!(
                "  [{:>8.3}s] ONSET tuned to {start} kHz — from here the radio is co-channel with \
                 whatever lives there, and has not been told what",
                wall()
            );

            let c = Arc::new(Counters::default());
            let paused = Arc::new(AtomicBool::new(false));
            let stop = Arc::new(AtomicBool::new(false));
            let run_id: u16 = (now_us() & 0xffff) as u16;
            let lat = Arc::new(Latency::default());
            let h = match o.role {
                Role::Tx => Some(spawn_tx(
                    face.clone(),
                    o.clone(),
                    c.clone(),
                    lat.clone(),
                    paused.clone(),
                    stop.clone(),
                    run_id,
                )),
                Role::Rx => Some(spawn_rx(face.clone(), c.clone(), lat.clone(), stop.clone())),
                Role::None => None,
            };

            let mut lp = Loop::new(o.clone(), start);
            let deadline = Instant::now() + Duration::from_secs(o.secs);
            let mut last_bucket = Instant::now();
            let (mut pf, mut pb) = (0u64, 0u64);
            let mut first_move: Option<(f64, f64, f64)> = None; // detect, moved, flow (s since onset)
            let mut awaiting_flow = false;
            let mut t_moved_at = Instant::now();
            let mut next_sense = Instant::now();
            let mut timeline: Vec<(f64, u64, f64)> = Vec::new();
            // ── follower state: silence -> search ────────────────────────────────────────────
            let mut last_seen = Instant::now();
            let mut last_n = 0u64;
            let mut probe_i = 0usize;
            let mut searching_since: Option<Instant> = None;
            let mut rendezvous: Vec<(f64, f64, u32, u32)> = Vec::new(); // t, secs, probes, landed

            while Instant::now() < deadline {
                // 100 ms accounting buckets — a 1 s bucket cannot show a 300 ms outage.
                if last_bucket.elapsed() >= Duration::from_millis(1000) {
                    let f = c.frames.load(Ordering::Relaxed);
                    let b = c.bytes.load(Ordering::Relaxed);
                    let el = last_bucket.elapsed().as_secs_f64();
                    let mbit = (b - pb) as f64 * 8.0 / 1e6 / el;
                    timeline.push((wall(), f - pf, mbit));
                    println!(
                        "  [{:>8.3}s] t r a f f i c  {} frames/s  {:.3} Mbit/s  errs={}",
                        wall(),
                        ((f - pf) as f64 / el) as u64,
                        mbit,
                        c.errs.load(Ordering::Relaxed)
                    );
                    pf = f;
                    pb = b;
                    last_bucket = Instant::now();
                }
                // ── follower: the link going quiet IS the notification ───────────────────────
                if o.follow {
                    let n = c.frames.load(Ordering::Relaxed);
                    if n > last_n {
                        last_n = n;
                        last_seen = Instant::now();
                        if let Some(t0) = searching_since.take() {
                            let probes = probe_i as u32;
                            println!(
                                "  [{:>8.3}s] RENDEZVOUS re-acquired on {} kHz after {:.3}s and {} \
                                 probe(s)",
                                wall(),
                                lp.cur,
                                t0.elapsed().as_secs_f64(),
                                probes
                            );
                            rendezvous.push((wall(), t0.elapsed().as_secs_f64(), probes, lp.cur));
                            probe_i = 0;
                        }
                    } else if last_seen.elapsed()
                        >= Duration::from_millis(if last_n == 0 {
                            // Cold start: the peer may simply not be up yet. Do not mistake
                            // "has not started" for "has moved" — that is how a search loop turns
                            // into a channel-walking radio that is never anywhere long enough.
                            o.silence_ms * 4
                        } else {
                            o.silence_ms
                        })
                    {
                        if searching_since.is_none() {
                            searching_since = Some(Instant::now());
                            println!(
                                "  [{:>8.3}s] SILENT {} kHz for {}ms — searching the candidate list",
                                wall(),
                                lp.cur,
                                o.silence_ms
                            );
                        }
                        // Next candidate, skipping the one we are on.
                        probe_i += 1;
                        let idx = (o.chans.iter().position(|&f| f == lp.cur).unwrap_or(0) + 1)
                            % o.chans.len();
                        let target = o.chans[idx];
                        match cli.set_channel(target, o.op_bw, o.pri_bw, o.pri_idx) {
                            Ok(d) => {
                                lp.cur = target;
                                println!(
                                    "  [{:>8.3}s] PROBE {} -> {target} kHz ({:.1} ms)",
                                    wall(),
                                    o.chans[(idx + o.chans.len() - 1) % o.chans.len()],
                                    d.as_secs_f64() * 1000.0
                                );
                            }
                            Err(e) => println!("  [{:>8.3}s] !! probe retune failed: {e}", wall()),
                        }
                        last_seen = Instant::now() - Duration::from_millis(o.silence_ms)
                            + Duration::from_millis(o.probe_ms);
                    }
                }
                if !o.no_sense && Instant::now() >= next_sense {
                    next_sense = Instant::now() + Duration::from_millis(o.period_ms);
                    match lp.step(&mut cli, &paused) {
                        Ok(Some(m)) => {
                            t_moved_at = m.t_moved;
                            awaiting_flow = true;
                            let d = (m.t_detect - t_onset).as_secs_f64();
                            let mv = (m.t_moved - t_onset).as_secs_f64();
                            if first_move.is_none() {
                                first_move = Some((d, mv, f64::NAN));
                            }
                        }
                        Ok(None) => {}
                        Err(e) => println!("  [{:>8.3}s] !! sense/act failed: {e}", wall()),
                    }
                }
                // Detect the resumption of flow with 5 ms granularity.
                if awaiting_flow {
                    let n0 = c.frames.load(Ordering::Relaxed);
                    tokio::time::sleep(Duration::from_millis(5)).await;
                    if c.frames.load(Ordering::Relaxed) > n0 {
                        let flow = t_moved_at.elapsed().as_secs_f64();
                        let since_onset = (Instant::now() - t_onset).as_secs_f64();
                        println!(
                            "  [{:>8.3}s] FLOW  resumed {:.3}s after the local retune returned \
                             ({:.3}s after onset)",
                            wall(),
                            flow,
                            since_onset
                        );
                        if let Some(fm) = first_move.as_mut() {
                            if fm.2.is_nan() {
                                fm.2 = since_onset;
                            }
                        }
                        awaiting_flow = false;
                    }
                } else {
                    tokio::time::sleep(Duration::from_millis(10)).await;
                }
            }
            stop.store(true, Ordering::Relaxed);
            if let Some(h) = h {
                let _ = tokio::time::timeout(Duration::from_millis(500), h).await;
            }

            let f = c.frames.load(Ordering::Relaxed);
            let b = c.bytes.load(Ordering::Relaxed);
            println!("  ──────────────────────────────────────────────────────────────");
            println!(
                "  TOTAL            : {f} frames, {:.3} Mbit of payload over {:.1}s = {:.3} Mbit/s \
                 mean",
                b as f64 * 8.0 / 1e6,
                o.secs as f64,
                b as f64 * 8.0 / 1e6 / o.secs as f64
            );
            if let Some((d, mv, fl)) = first_move {
                println!(
                    "  FIRST MOVE       : detect {d:.3}s after onset; retuned {mv:.3}s; frames \
                     flowing {fl:.3}s  ⇒ move cost {:.3}s",
                    fl - d
                );
            } else {
                println!("  FIRST MOVE       : none — the loop stayed put for the whole run");
            }
            println!(
                "  sensing cost     : {} windows, {:.3}s inside morse_cli ({:.1}% of the run, {} \
                 timeouts) — the dwell budget, measured",
                lp.windows,
                cli.busy.as_secs_f64(),
                100.0 * cli.busy.as_secs_f64() / o.secs as f64,
                cli.timeouts
            );
            println!(
                "  contaminated     : {}/{} windows over threshold",
                lp.contaminated_windows, lp.windows
            );
            print!("  per-second Mbit/s:");
            for (_, _, m) in &timeline {
                print!(" {m:.2}");
            }
            println!();
            // ── the latency phases: before the move, and after it ────────────────────────────
            // Split at the instant the local radio landed on the new channel. If nothing moved the
            // whole run is one phase, which is exactly what the do-nothing control wants.
            let split = first_move.map(|(_, mv, _)| mv);
            let unit = if o.role == Role::Rx {
                "recv-send spread"
            } else {
                "inject()"
            };
            match split {
                Some(sp) => {
                    if let Some((n, p50, p99, mx)) = lat.window(0.0, sp) {
                        println!(
                            "  BEFORE the move  : {unit} n={n}  p50 {p50} us  p99 {p99} us  max {mx} us"
                        );
                    }
                    // Skip 1 s of settling so the move's own outage is not charged to the new channel.
                    if let Some((n, p50, p99, mx)) = lat.window(sp + 1.0, f64::INFINITY) {
                        println!(
                            "  AFTER  the move  : {unit} n={n}  p50 {p50} us  p99 {p99} us  max {mx} us"
                        );
                    }
                }
                None => {
                    if let Some((n, p50, p99, mx)) = lat.window(0.0, f64::INFINITY) {
                        println!(
                            "  WHOLE RUN        : {unit} n={n}  p50 {p50} us  p99 {p99} us  max {mx} us"
                        );
                    }
                }
            }
            if o.follow {
                if rendezvous.is_empty() {
                    println!("  RENDEZVOUS       : none needed — the stream never went silent");
                } else {
                    let mut v: Vec<f64> = rendezvous.iter().map(|r| r.1).collect();
                    v.sort_by(|a, b| a.partial_cmp(b).unwrap());
                    println!(
                        "  RENDEZVOUS       : {} re-acquisition(s); min {:.3}s p50 {:.3}s max {:.3}s \
                         {:?}",
                        v.len(),
                        v[0],
                        v[v.len() / 2],
                        v[v.len() - 1],
                        rendezvous
                    );
                }
            }
            if !lp.moves.is_empty() {
                println!("  MOVES            : {:?}", lp.moves);
            }
            println!("  final channel    : {} kHz", lp.cur);
        }
        v => {
            eprintln!("unknown mode {v}");
            std::process::exit(2);
        }
    }
    Ok(())
}

#[cfg(not(target_os = "linux"))]
fn main() {
    eprintln!("halow_cognition drives Linux netdevs and morse_cli; build it for the node.");
}
