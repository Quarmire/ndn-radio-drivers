//! **The on-air harness for the HaLow (802.11ah / S1G) data plane** — one binary that is either end
//! of a two-node test, driving the real [`MorseFrameIo`] / [`Nrc7292FrameIo`] rather than a socket
//! opened beside them.
//!
//! `src/halow/linux.rs` compiles and unit-tests clean, and until this example runs it has never
//! moved a frame. Nothing there is proven by a passing test: the tests cover the *rules* (the
//! declaration, the payload cap, the interface pairing), because neither `FrameIo` is constructible
//! without live netdevs. This is the part that can only be answered on air.
//!
//! # The measurement, not just the transmit
//!
//! * **Every injected frame is identifiable.** The payload opens with the ASCII magic `NDNONAIR`
//!   (hex `4e 44 4e 4f 4e 41 49 52`), then a 32-bit sequence number and a 16-bit *run id*, so a
//!   capture can be grepped for **our** frames and a second run cannot be mistaken for the first.
//!   "Some frames appeared" is not evidence.
//! * **Per-second buckets, never just a total.** A flat total hides a hard stall (the canonical one
//!   on this bench read `499` then `10 10 10 …`); the bucket line shows it in one run.
//! * **The negative half is half the experiment.** Run `rx` once while `tx` is running and once
//!   while it is not. A receiver that reports frames with the transmitter stopped is measuring
//!   ambient traffic, not the link.
//! * **The instrument is suspected first.** The `rxraw` arm bypasses the `FrameIo` wrapper entirely
//!   and reports what arrived at the socket *before* `frame::parse` had an opinion — so "0 frames"
//!   through `rx` can be separated into "nothing on the air" and "the decoder rejected it", which
//!   are different failures with different fixes.
//!
//! # Which radio
//!
//! Chosen from the interface spec, because the two data planes are shaped differently and the
//! difference is silent if you get it wrong:
//!
//! * `mon0:morse0` (a colon) → **Morse Micro MM6108**, [`MorseFrameIo`]. Split plane: inject on a
//!   mac80211 monitor vif, capture on the driver's `morseN` sniffer netdev. ☠ `morse0` accepts a
//!   `sendto()` and radiates nothing, which is why the constructor refuses `tx == rx` and refuses a
//!   sniffer netdev as TX.
//! * `halow0` (no colon) → **Newracom NRC7292**, [`Nrc7292FrameIo`]. One monitor netdev both ways.
//!
//! # Usage
//!
//! ```text
//! sudo ./halow_onair tx    <ifspec> <count> [options]
//! sudo ./halow_onair rx    <ifspec> <secs>  [options]
//! sudo ./halow_onair rxraw <iface>  <secs>  [options]
//!
//! # Morse → Morse, the pair this bench has:
//! node A:  sudo ./halow_onair rx  mon0:morse0 30
//! node B:  sudo ./halow_onair tx  mon0:morse0 2000 --pps 100 --size 200
//! # …then repeat the rx arm with no transmitter running (the negative control).
//!
//! # Newracom:
//! sudo ./halow_onair rx halow0 30 --cli /run/current-system/sw/bin/cli_app
//! sudo ./halow_onair tx halow0 2000
//!
//! # Instrument check — no FrameIo wrapper, no format filter, reports the S1G radiotap TLV:
//! sudo ./halow_onair rxraw morse0 15
//! ```
//!
//! Options (all optional; every one prints what it actually resolved to):
//!
//! ```text
//!   --size N          payload bytes, default 200. MM6108 refuses > 1546 (MEASURED byte-exact).
//!   --pps N           transmit pace in frames/s, default 100. 0 = unpaced (flat out).
//!   --poisson         ★ exponentially distributed gaps of the same mean instead of a fixed
//!                     interval. MANDATORY for any arm with two transmitters: two periodic
//!                     senders drift past each other only at their clock difference (~10 ppm), so
//!                     their relative phase is frozen for the whole arm and the collision rate is
//!                     decided by the start-time lottery, not the load.
//!   --seed N          seed that arrival process (printed either way, so a run is reproducible)
//!   --secs S          stop after S seconds as well as after `count` frames
//!   --mcs N           S1G MCS for injection. Reaches the air ONLY on a Morse carrying the
//!                     monitor-injection patch (writes /sys/module/morse/parameters/inject_mcs);
//!                     the run header prints rate_actuated so you are never guessing.
//!   --bw MHZ          injected S1G width, 1|2|4|8. Morse + patch only. Emitted width is
//!                     min(inject_bw, operating_bw) — set the operating width with morse_cli first.
//!   --mcs-param PATH  override the inject_mcs sysfs path (its spelling is a bench note, not a fact)
//!   --bw-param PATH   override the inject_bw sysfs path
//!   --ethertype HEX   LLC/SNAP ethertype, default 8624
//!   --src MAC         802.11 addr2, default 02:4e:44:4e:00:01 (DEFAULT_SRC; never a host MAC)
//!   --dst MAC         802.11 addr1, default ff:ff:ff:ff:ff:ff
//!   --cli PATH        NRC7292 only: compose the read-now µs clock (Nrc7292Clock) onto the face
//!   --infra-beacons   NRC7292 only: also harvest infrastructure beacons for mesh_common_view
//!   --show N          full metadata dump for the first N received frames, default 5
//!   --label TEXT      tag the run header (e.g. "control-tx-off") so a log says which half it is
//! ```
//!
//! Needs root (CAP_NET_RAW for the packet socket; CAP_NET_ADMIN is not required — this example
//! never touches the channel, which on S1G is four numbers and belongs to `morse_cli`).

#[cfg(target_os = "linux")]
mod harness {
    use std::collections::{BTreeMap, BTreeSet};
    use std::path::PathBuf;
    use std::time::{Duration, Instant, SystemTime, UNIX_EPOCH};

    use bytes::Bytes;
    use ndn_frame_io::AfPacketBackend;
    use ndn_radio_drivers::halow::{
        MORSE_INJECT_BW_PARAM, MORSE_INJECT_MCS_PARAM, MorseFrameIo, Nrc7292FrameIo, s1g_metadata,
    };
    use ndn_radio_drivers::nrc7292::Nrc7292Clock;
    use ndn_radio_drivers::{
        BROADCAST, CapturedFrame, DEFAULT_SRC, FaceError, FrameFormat, FrameIo, InjectFrame,
        McsDescriptor, NDN_ETHERTYPE, RadioProfile, RadioTime, TxIntent, frame, radiotap,
    };

    /// The 8-byte payload magic. Grep a capture for `4e444e4f4e414952` (`tcpdump -x`) or for
    /// `NDNONAIR` in an ASCII dump; nothing else on the air carries it.
    pub const MAGIC: &[u8; 8] = b"NDNONAIR";
    /// magic(8) + seq(4) + send-µs(8) + len(2) + run-id(2).
    pub const HDR: usize = 24;

    // ─────────────────────────────────────────────────────────────────────────────────────────
    // Options
    // ─────────────────────────────────────────────────────────────────────────────────────────

    pub struct Opts {
        pub size: usize,
        pub pps: u64,
        pub mcs: Option<u8>,
        pub bw: Option<u8>,
        pub mcs_param: Option<PathBuf>,
        pub bw_param: Option<PathBuf>,
        pub ethertype: u16,
        pub src: [u8; 6],
        pub dst: [u8; 6],
        pub cli: Option<String>,
        pub infra_beacons: bool,
        pub show: usize,
        pub label: Option<String>,
        pub poisson: bool,
        pub seed: u64,
        pub secs: Option<u64>,
    }

    impl Default for Opts {
        fn default() -> Self {
            Self {
                size: 200,
                pps: 100,
                mcs: None,
                bw: None,
                mcs_param: None,
                bw_param: None,
                ethertype: NDN_ETHERTYPE,
                src: DEFAULT_SRC,
                dst: BROADCAST,
                cli: None,
                infra_beacons: false,
                show: 5,
                label: None,
                poisson: false,
                seed: 0,
                secs: None,
            }
        }
    }

    pub fn parse_mac(s: &str) -> Result<[u8; 6], String> {
        let parts: Vec<&str> = s.split(|c| c == ':' || c == '-').collect();
        if parts.len() != 6 {
            return Err(format!("{s:?} is not a 6-octet MAC"));
        }
        let mut out = [0u8; 6];
        for (i, p) in parts.iter().enumerate() {
            out[i] = u8::from_str_radix(p, 16).map_err(|e| format!("{s:?}: {e}"))?;
        }
        Ok(out)
    }

    /// Take the value that follows `args[*i]`, advancing `i`. A plain function rather than a
    /// closure so nothing borrows `i` past its use.
    fn take<'a>(args: &'a [String], i: &mut usize, k: &str) -> Result<&'a str, String> {
        *i += 1;
        args.get(*i)
            .map(String::as_str)
            .ok_or_else(|| format!("{k} needs a value"))
    }

    pub fn parse_opts(args: &[String]) -> Result<Opts, String> {
        let mut o = Opts::default();
        let mut i = 0usize;
        while i < args.len() {
            let k = args[i].clone();
            match k.as_str() {
                "--size" => {
                    o.size = take(args, &mut i, &k)?
                        .parse()
                        .map_err(|e| format!("--size: {e}"))?
                }
                "--pps" => {
                    o.pps = take(args, &mut i, &k)?
                        .parse()
                        .map_err(|e| format!("--pps: {e}"))?
                }
                "--mcs" => {
                    o.mcs = Some(
                        take(args, &mut i, &k)?
                            .parse()
                            .map_err(|e| format!("--mcs: {e}"))?,
                    )
                }
                "--bw" => {
                    o.bw = Some(
                        take(args, &mut i, &k)?
                            .parse()
                            .map_err(|e| format!("--bw: {e}"))?,
                    )
                }
                "--mcs-param" => o.mcs_param = Some(PathBuf::from(take(args, &mut i, &k)?)),
                "--bw-param" => o.bw_param = Some(PathBuf::from(take(args, &mut i, &k)?)),
                "--ethertype" => {
                    let v = take(args, &mut i, &k)?.to_string();
                    o.ethertype = u16::from_str_radix(v.trim_start_matches("0x"), 16)
                        .map_err(|e| format!("--ethertype: {e}"))?;
                }
                "--src" => {
                    let v = take(args, &mut i, &k)?.to_string();
                    o.src = parse_mac(&v)?;
                }
                "--dst" => {
                    let v = take(args, &mut i, &k)?.to_string();
                    o.dst = parse_mac(&v)?;
                }
                "--cli" => o.cli = Some(take(args, &mut i, &k)?.to_string()),
                "--infra-beacons" => o.infra_beacons = true,
                "--show" => {
                    o.show = take(args, &mut i, &k)?
                        .parse()
                        .map_err(|e| format!("--show: {e}"))?
                }
                "--label" => o.label = Some(take(args, &mut i, &k)?.to_string()),
                "--poisson" => o.poisson = true,
                "--seed" => {
                    o.seed = take(args, &mut i, &k)?
                        .parse()
                        .map_err(|e| format!("--seed: {e}"))?
                }
                "--secs" => {
                    o.secs = Some(
                        take(args, &mut i, &k)?
                            .parse()
                            .map_err(|e| format!("--secs: {e}"))?,
                    )
                }
                other => return Err(format!("unknown option {other}")),
            }
            i += 1;
        }
        if o.size < HDR {
            return Err(format!(
                "--size {} is below the {HDR}-byte identifying header — an unidentifiable frame is \
                 not evidence",
                o.size
            ));
        }
        if let Some(w) = o.bw {
            if !matches!(w, 1 | 2 | 4 | 8) {
                return Err(format!("--bw {w} is not an S1G width (1|2|4|8)"));
            }
        }
        Ok(o)
    }

    // ─────────────────────────────────────────────────────────────────────────────────────────
    // The payload: what makes a captured frame OURS
    // ─────────────────────────────────────────────────────────────────────────────────────────

    pub fn now_us() -> u64 {
        SystemTime::now()
            .duration_since(UNIX_EPOCH)
            .map(|d| d.as_micros() as u64)
            .unwrap_or(0)
    }

    /// splitmix64 — a seeded PRNG with no dependency, so a run is reproducible from its printed
    /// seed and two nodes can be given provably independent streams.
    pub struct Rng(u64);

    impl Rng {
        pub fn new(seed: u64) -> Self {
            Self(seed)
        }
        pub fn next_u64(&mut self) -> u64 {
            self.0 = self.0.wrapping_add(0x9E37_79B9_7F4A_7C15);
            let mut z = self.0;
            z = (z ^ (z >> 30)).wrapping_mul(0xBF58_476D_1CE4_E5B9);
            z = (z ^ (z >> 27)).wrapping_mul(0x94D0_49BB_1331_11EB);
            z ^ (z >> 31)
        }
        /// U(0,1] — never exactly 0, so `ln` stays finite.
        pub fn unit(&mut self) -> f64 {
            ((self.next_u64() >> 11) as f64 + 1.0) / 9_007_199_254_740_992.0
        }
        /// An exponentially distributed gap with the given mean: the inter-arrival time of a
        /// Poisson process, which is the offered-load model a CSMA saturation curve is defined
        /// against.
        pub fn exp(&mut self, mean: f64) -> f64 {
            -mean * self.unit().ln()
        }
    }

    pub fn build_payload(seq: u32, run: u16, size: usize) -> Bytes {
        let mut v = vec![0u8; size];
        v[..8].copy_from_slice(MAGIC);
        v[8..12].copy_from_slice(&seq.to_le_bytes());
        v[12..20].copy_from_slice(&now_us().to_le_bytes());
        v[20..22].copy_from_slice(&(size as u16).to_le_bytes());
        v[22..24].copy_from_slice(&run.to_le_bytes());
        // Deterministic filler so a hex dump is readable and a truncation is obvious.
        for (i, b) in v.iter_mut().enumerate().skip(HDR) {
            *b = (i as u8) ^ 0x5a;
        }
        Bytes::from(v)
    }

    /// `(seq, run, sent_us)` when `p` is one of ours.
    pub fn read_payload(p: &[u8]) -> Option<(u32, u16, u64)> {
        if p.len() < HDR || &p[..8] != MAGIC {
            return None;
        }
        Some((
            u32::from_le_bytes([p[8], p[9], p[10], p[11]]),
            u16::from_le_bytes([p[22], p[23]]),
            u64::from_le_bytes([p[12], p[13], p[14], p[15], p[16], p[17], p[18], p[19]]),
        ))
    }

    /// Does this raw buffer contain our magic anywhere? Used by `rxraw`, which must not assume the
    /// frame parsed — that assumption is exactly what it exists to test.
    pub fn contains_magic(buf: &[u8]) -> bool {
        buf.len() >= 8 && buf.windows(8).any(|w| w == MAGIC)
    }

    pub fn hex(b: &[u8]) -> String {
        b.iter()
            .map(|x| format!("{x:02x}"))
            .collect::<Vec<_>>()
            .join(" ")
    }

    pub fn mac(m: &Option<[u8; 6]>) -> String {
        match m {
            None => "None".into(),
            Some(a) => a
                .iter()
                .map(|x| format!("{x:02x}"))
                .collect::<Vec<_>>()
                .join(":"),
        }
    }

    // ─────────────────────────────────────────────────────────────────────────────────────────
    // The two faces, behind one handle
    // ─────────────────────────────────────────────────────────────────────────────────────────

    pub enum Face {
        Morse(MorseFrameIo),
        Nrc(Nrc7292FrameIo),
    }

    impl Face {
        /// Open whichever radio the spec names, wiring every knob the impl really exposes and
        /// printing what each one resolved to.
        pub fn open(spec: &str, o: &Opts) -> Result<Self, Box<dyn std::error::Error>> {
            let fmt = FrameFormat::RawNdnS1g {
                ethertype: o.ethertype,
            };
            match spec.split_once(':') {
                Some((tx, rx)) => {
                    let mut f = MorseFrameIo::new(tx, rx, fmt)?;
                    // Opt in to the patched driver's parameters. `with_inject_*_param` CONSUMES
                    // self and errors when the patch is absent, so the existence check happens
                    // first: an absent parameter must degrade to "no actuator, and we said so",
                    // not to a lost face. The opt-in still re-verifies — this only decides whether
                    // to attempt it.
                    if o.mcs.is_some() {
                        let p = o
                            .mcs_param
                            .clone()
                            .unwrap_or_else(|| PathBuf::from(MORSE_INJECT_MCS_PARAM));
                        if p.exists() {
                            f = f.with_inject_mcs_param(Some(p))?;
                        } else {
                            eprintln!(
                                "!! --mcs given but {} is absent: the monitor-injection patch is \
                                 not loaded, so the rate will NOT reach the air (set_rate stores \
                                 bearer state and returns Ok). rate_actuated below says so.",
                                p.display()
                            );
                        }
                    }
                    if o.bw.is_some() {
                        let p = o
                            .bw_param
                            .clone()
                            .unwrap_or_else(|| PathBuf::from(MORSE_INJECT_BW_PARAM));
                        if p.exists() {
                            f = f.with_inject_bw_param(Some(p))?;
                        } else {
                            eprintln!(
                                "!! --bw given but {} is absent: no injection-width actuator on \
                                 this driver; the frame goes out at the operating width.",
                                p.display()
                            );
                        }
                    }
                    Ok(Face::Morse(f))
                }
                None => {
                    let mut f = Nrc7292FrameIo::new(spec, fmt)?;
                    if o.infra_beacons {
                        f = f.with_infrastructure_beacons();
                    }
                    if let Some(cli) = o.cli.as_ref() {
                        match Nrc7292Clock::new(spec, cli) {
                            // A domain mismatch here is a real error worth stopping for: it means
                            // the clock and the RX stamps are different radios.
                            Ok(c) => f = f.with_clock(c)?,
                            Err(e) => eprintln!(
                                "!! Nrc7292Clock::new({spec}, {cli}) failed, continuing with no \
                                 read-now clock: {e}"
                            ),
                        }
                    }
                    Ok(Face::Nrc(f))
                }
            }
        }

        pub fn kind(&self) -> &'static str {
            match self {
                Face::Morse(_) => "Morse MM6108 (MorseFrameIo, split TX/RX netdevs)",
                Face::Nrc(_) => "Newracom NRC7292 (Nrc7292FrameIo, one monitor netdev)",
            }
        }

        pub async fn inject(&self, f: InjectFrame) -> Result<(), FaceError> {
            match self {
                Face::Morse(m) => m.inject(f).await,
                Face::Nrc(n) => n.inject(f).await,
            }
        }

        pub async fn recv_frame(&self) -> Result<CapturedFrame, FaceError> {
            match self {
                Face::Morse(m) => m.recv_frame().await,
                Face::Nrc(n) => n.recv_frame().await,
            }
        }

        pub fn set_rate(&self, mcs: McsDescriptor) -> Result<(), FaceError> {
            match self {
                Face::Morse(m) => m.set_rate(mcs),
                Face::Nrc(n) => n.set_rate(mcs),
            }
        }

        /// ★ Does `set_rate` reach the air on THIS instance? The whole point of asking the impl
        /// instead of assuming: on an unpatched Morse and on every NRC7292 the answer is `false`
        /// and `set_rate` still returns `Ok`.
        pub fn rate_actuated(&self) -> bool {
            match self {
                Face::Morse(m) => m.rate_actuated(),
                Face::Nrc(n) => n.rate_actuated(),
            }
        }

        pub fn print_header(&self, o: &Opts, spec: &str) {
            let (cap, srcs) = match self {
                Face::Morse(m) => (m.capability(), m.time_sources()),
                Face::Nrc(n) => (n.capability(), n.time_sources()),
            };
            println!("  radio            : {}", self.kind());
            match self {
                Face::Morse(m) => {
                    let (tx, rx) = m.interfaces();
                    println!("  tx iface         : {tx}   (mac80211 monitor vif)");
                    println!("  rx iface         : {rx}   (driver sniffer netdev)");
                    println!(
                        "  clock domain     : {:?}  (ClockDomainId(rx ifindex))",
                        m.clock_domain()
                    );
                    println!("  rate_actuated    : {}", m.rate_actuated());
                    println!("  inject_bw_actuat.: {}", m.inject_bw_actuated());
                }
                Face::Nrc(n) => {
                    println!("  iface            : {spec}   (one monitor netdev, both directions)");
                    println!(
                        "  rate_actuated    : {}  (firmware owns the rate on this part)",
                        n.rate_actuated()
                    );
                    println!(
                        "  read_clock       : {:?}",
                        n.read_clock(
                            srcs.first()
                                .map(|s| s.domain)
                                .unwrap_or(ndn_frame_io::ClockDomainId(0))
                        )
                    );
                }
            }
            println!("  time_sources     : {srcs:?}");
            println!("  capability       : {cap:?}");
            let _ = o;
        }
    }

    // ─────────────────────────────────────────────────────────────────────────────────────────
    // Per-second buckets — the discipline, factored so both arms get it
    // ─────────────────────────────────────────────────────────────────────────────────────────

    #[derive(Default)]
    pub struct Buckets {
        pub v: Vec<u64>,
    }

    impl Buckets {
        pub fn bump(&mut self, sec: usize, n: u64) {
            while self.v.len() <= sec {
                self.v.push(0);
            }
            self.v[sec] += n;
        }
        pub fn line(&self) -> String {
            self.v
                .iter()
                .map(|n| n.to_string())
                .collect::<Vec<_>>()
                .join(" ")
        }
        pub fn total(&self) -> u64 {
            self.v.iter().sum()
        }
    }

    /// Everything one **transmitter** delivered, kept apart from every other transmitter's.
    ///
    /// ☠ Without this the receiver folds every sender into one sequence set, and the moment two
    /// nodes transmit at once — which is the whole point of a contention arm — the "% of the span
    /// delivered" line becomes meaningless: two senders each counting 0,1,2,… overlap, so the
    /// union of their sequence numbers is smaller than their sum and the loss is hidden. The run
    /// id is already in every payload; this just stops throwing it away.
    #[derive(Default)]
    pub struct PerRun {
        pub buckets: Buckets,
        pub seqs: BTreeSet<u32>,
        pub n: u64,
    }

    // ─────────────────────────────────────────────────────────────────────────────────────────
    // TX
    // ─────────────────────────────────────────────────────────────────────────────────────────

    pub async fn run_tx(
        spec: &str,
        count: u32,
        o: &Opts,
    ) -> Result<(), Box<dyn std::error::Error>> {
        let face = Face::open(spec, o)?;
        let run: u16 = (now_us() & 0xffff) as u16;

        println!("═══ halow_onair TX ═══════════════════════════════════════════════════════");
        if let Some(l) = o.label.as_ref() {
            println!("  label            : {l}");
        }
        face.print_header(o, spec);
        println!(
            "  format           : RawNdnS1g {{ ethertype: 0x{:04x} }}",
            o.ethertype
        );
        println!("  addr1(dst)       : {}", mac(&Some(o.dst)));
        println!("  addr2(src)       : {}", mac(&Some(o.src)));
        println!(
            "  payload          : {} B  (magic {:?} + seq + run id)",
            o.size,
            std::str::from_utf8(MAGIC).unwrap_or("?")
        );
        println!("  run id           : 0x{run:04x}   ← distinguishes this run from every other");
        println!(
            "  magic hex        : {}   ← grep a capture for this",
            hex(MAGIC)
        );
        println!("  count            : {count}");
        println!(
            "  pace             : {}",
            if o.pps == 0 {
                "unpaced (flat out)".to_string()
            } else {
                format!("{} frames/s", o.pps)
            }
        );

        // The rate knob, applied through the trait, with its honesty flag printed above.
        if let Some(idx) = o.mcs {
            let d = McsDescriptor {
                index: idx,
                short_gi: false,
                vht: false,
                nss: 1,
                stbc: false,
                ldpc: false,
                he: false,
                dcm: false,
                er_su: false,
            };
            match face.set_rate(d) {
                Ok(()) => println!(
                    "  set_rate({idx})     : Ok  — reaches the air: {}",
                    face.rate_actuated()
                ),
                Err(e) => println!("  set_rate({idx})     : ERR {e}"),
            }
        }
        if let (Some(w), Face::Morse(m)) = (o.bw, &face) {
            match m.set_inject_bw_mhz(w) {
                Ok(()) => println!(
                    "  inject_bw        : {w} MHz written (emitted = min(inject_bw, op_bw))"
                ),
                Err(e) => println!("  inject_bw        : ERR {e}"),
            }
        }
        println!("──────────────────────────────────────────────────────────────────────────");

        let start = Instant::now();
        // ☠ **PERIODIC PACING IS A TRAP THE MOMENT TWO NODES CONTEND.** Two hosts each sending
        // every 1/pps seconds slide past one another only at their clock *difference* (~10 ppm),
        // so across a 30 s arm their relative phase is FROZEN: the pair either never collides or
        // always collides, decided by the start-time lottery rather than by the offered load. The
        // identical defect was already paid for once on this bench, where an "unleased" control
        // locked into perfect turn-taking (nearest-neighbour separation flat at 6206 us across
        // p1/p50/p75) and scored 97.6% because it was rigged easy.
        //
        // `--poisson` replaces the fixed interval with exponentially distributed gaps of the SAME
        // mean, seeded independently per node. That destroys the phase lock, and it is also the
        // standard offered-load model against which a CSMA saturation curve is defined.
        let seed = if o.seed != 0 {
            o.seed
        } else {
            now_us() ^ ((run as u64) << 47)
        };
        let mut rng = Rng::new(seed);
        let mean_gap_us: f64 = if o.pps == 0 {
            0.0
        } else {
            1_000_000.0 / o.pps as f64
        };
        let mut next_off_us: f64 = 0.0;
        let deadline = o.secs.map(|s| start + Duration::from_secs(s));
        println!(
            "  arrivals         : {}",
            if o.pps == 0 {
                "unpaced (flat out) — no arrival process".to_string()
            } else if o.poisson {
                format!(
                    "POISSON, mean gap {mean_gap_us:.1} us (seed {seed}) — phase lock between \
                     nodes is impossible by construction"
                )
            } else {
                format!(
                    "PERIODIC, gap {mean_gap_us:.1} us  ⚠ two periodic nodes hold a FROZEN \
                     relative phase; use --poisson for any contention arm"
                )
            }
        );
        if let Some(s) = o.secs {
            println!("  stop             : after {s} s or {count} frames, whichever is first");
        }
        let mut buckets = Buckets::default();
        let mut errs = Buckets::default();
        let mut first_err: Option<String> = None;
        let mut printed = 0usize;

        for seq in 0..count {
            let f = InjectFrame {
                payload: build_payload(seq, run, o.size),
                tx: TxIntent::CONSERVATIVE,
                dst: o.dst,
                src: o.src,
                addr3: None,
                extra: None,
                htc: None,
            };
            let sec = start.elapsed().as_secs() as usize;
            match face.inject(f).await {
                Ok(()) => buckets.bump(sec, 1),
                Err(e) => {
                    errs.bump(sec, 1);
                    if first_err.is_none() {
                        first_err = Some(e.to_string());
                    }
                }
            }
            // Flush completed seconds as they close, so a stall is visible while it happens
            // rather than only in the post-mortem.
            while printed < buckets.v.len().saturating_sub(1) {
                println!(
                    "  t={printed:>3}s  sent={:<6} err={}",
                    buckets.v[printed],
                    errs.v.get(printed).copied().unwrap_or(0)
                );
                printed += 1;
            }
            if o.pps != 0 {
                next_off_us += if o.poisson {
                    rng.exp(mean_gap_us)
                } else {
                    mean_gap_us
                };
                // Absolute, accumulated from `start`: a re-based sleep would add every late
                // wake-up to the period permanently and the offered rate would not be the one
                // asked for.
                let target = start + Duration::from_nanos((next_off_us * 1_000.0) as u64);
                let now = Instant::now();
                if target > now {
                    tokio::time::sleep(target - now).await;
                }
            }
            if let Some(d) = deadline {
                if Instant::now() >= d {
                    break;
                }
            }
        }
        while printed < buckets.v.len() {
            println!(
                "  t={printed:>3}s  sent={:<6} err={}",
                buckets.v[printed],
                errs.v.get(printed).copied().unwrap_or(0)
            );
            printed += 1;
        }

        let el = start.elapsed().as_secs_f64();
        println!("──────────────────────────────────────────────────────────────────────────");
        println!("  buckets (frames/s): {}", buckets.line());
        println!(
            "  sent={} err={} in {el:.2}s = {:.1} frames/s, {:.3} Mbit/s of payload",
            buckets.total(),
            errs.total(),
            buckets.total() as f64 / el,
            (buckets.total() as f64 * o.size as f64 * 8.0) / el / 1e6
        );
        if let Some(e) = first_err {
            println!("  first error      : {e}");
        }
        println!(
            "  ⚠ a successful sendto() is NOT evidence of radiation — only the receiver is.\n\
             ⚠ now run the receiver again with this transmitter STOPPED (the negative control)."
        );
        Ok(())
    }

    // ─────────────────────────────────────────────────────────────────────────────────────────
    // TXDATA — the NRC7292's only working transmit path
    // ─────────────────────────────────────────────────────────────────────────────────────────

    /// Send the same identifiable payload out an **operating (managed/AP/mesh) netdev** as an
    /// ordinary Ethernet frame, letting the driver do the 802.11 encapsulation.
    ///
    /// ★ Why this arm exists: on the NRC7292, radiotap injection through a monitor vif is
    /// **structurally dead** in the stock driver — MEASURED (500 frames accepted by the socket,
    /// chip `MAC TX Statistics OK count` +0 against a +20 ping control, peer `mon0` 0, peer chip
    /// `MAC RX Statistics` +0). The frame is freed in `nrc_mac_tx` before it ever reaches
    /// `nrc_xmit_frame`. So `tx` on that radio can only ever prove the socket accepted bytes.
    ///
    /// The operating vif's data path *does* transmit, and — this is the point — under
    /// `FrameFormat::RawNdn*` the bytes it puts on the air are the **same bytes** injection would
    /// have put there: a data MPDU whose payload is `AA AA 03 00 00 00 <ethertype>` followed by
    /// ours. So this arm is the positive control the receiver otherwise has no way to get: it
    /// proves `recv_frame`'s decode, metadata and identification end to end while the transmit
    /// half is blocked in the vendor driver.
    ///
    /// ⚠ It is **not** a `FrameIo` and does not pretend to be one. It cannot set addr1/addr2 (the
    /// driver owns them, and `--src`/`--dst` are ignored beyond the Ethernet header), cannot reach
    /// a rate or width knob, and cannot carry the named-radio address layout. Do not read a
    /// success here as "the NRC7292 transmits through our stack".
    pub fn run_txdata(iface: &str, count: u32, o: &Opts) -> Result<(), Box<dyn std::error::Error>> {
        // Refuse a monitor vif loudly: sending Ethernet frames there is meaningless, and the
        // failure would be silent (the socket accepts them and the driver frees them).
        let arphrd: u32 = std::fs::read_to_string(format!("/sys/class/net/{iface}/type"))
            .map_err(|e| format!("/sys/class/net/{iface}/type: {e}"))?
            .trim()
            .parse()?;
        if arphrd != 1 {
            return Err(format!(
                "{iface} has ARPHRD {arphrd}, not 1 (ARPHRD_ETHER). `txdata` is the OPERATING-vif \
                 path — point it at the managed/AP/mesh netdev (halow0), not at a monitor vif. \
                 Injection on a monitor vif is what `tx` does, and on the NRC7292 that path is \
                 dead in the stock driver."
            )
            .into());
        }
        let own = std::fs::read_to_string(format!("/sys/class/net/{iface}/address"))?;
        let mut src = [0u8; 6];
        for (i, b) in own.trim().split(':').enumerate().take(6) {
            src[i] = u8::from_str_radix(b, 16)?;
        }

        // SAFETY: plain libc socket calls; every pointer is to a live local, every length is the
        // matching size_of. Errors are read from errno, never inferred from the return value alone.
        let fd = unsafe {
            libc::socket(
                libc::AF_PACKET,
                libc::SOCK_RAW,
                (libc::ETH_P_ALL as u16).to_be() as i32,
            )
        };
        if fd < 0 {
            return Err(format!(
                "socket(AF_PACKET, SOCK_RAW): {} (needs CAP_NET_RAW — run under sudo)",
                std::io::Error::last_os_error()
            )
            .into());
        }
        let ifindex = unsafe {
            let c = std::ffi::CString::new(iface)?;
            libc::if_nametoindex(c.as_ptr()) as i32
        };
        if ifindex == 0 {
            unsafe { libc::close(fd) };
            return Err(format!("if_nametoindex({iface}) = 0").into());
        }

        let run: u16 = (now_us() & 0xffff) as u16;
        println!("═══ halow_onair TXDATA (operating-vif data path) ═════════════════════════");
        if let Some(l) = o.label.as_ref() {
            println!("  label            : {l}");
        }
        println!("  iface            : {iface}  ifindex={ifindex}  (ARPHRD_ETHER operating vif)");
        println!("  ⚠ NOT a FrameIo: the driver owns the 802.11 header, the rate and the width.");
        println!("  ⚠ It exists because monitor injection is dead in the NRC7292 stock driver.");
        println!(
            "  eth src          : {}  (the netdev's own address — not ours to choose)",
            mac(&Some(src))
        );
        println!("  eth dst          : {}", mac(&Some(o.dst)));
        println!(
            "  ethertype        : 0x{:04x}  ← lands in the LLC/SNAP the receiver matches on",
            o.ethertype
        );
        println!(
            "  payload          : {} B  (magic {:?} + seq + run id)",
            o.size,
            std::str::from_utf8(MAGIC).unwrap_or("?")
        );
        println!("  run id           : 0x{run:04x}");
        println!("  magic hex        : {}", hex(MAGIC));
        println!("  count            : {count}");
        println!(
            "  pace             : {}",
            if o.pps == 0 {
                "unpaced (flat out)".to_string()
            } else {
                format!("{} frames/s", o.pps)
            }
        );
        println!("──────────────────────────────────────────────────────────────────────────");

        let mut sa: libc::sockaddr_ll = unsafe { std::mem::zeroed() };
        sa.sll_family = libc::AF_PACKET as u16;
        sa.sll_protocol = (o.ethertype).to_be();
        sa.sll_ifindex = ifindex;
        sa.sll_halen = 6;
        sa.sll_addr[..6].copy_from_slice(&o.dst);

        let start = Instant::now();
        let interval = if o.pps == 0 {
            Duration::ZERO
        } else {
            Duration::from_nanos(1_000_000_000 / o.pps)
        };
        let mut buckets = Buckets::default();
        let mut errs = Buckets::default();
        let mut first_err: Option<String> = None;
        let mut printed = 0usize;

        for seq in 0..count {
            let mut eth = Vec::with_capacity(14 + o.size);
            eth.extend_from_slice(&o.dst);
            eth.extend_from_slice(&src);
            eth.extend_from_slice(&o.ethertype.to_be_bytes());
            eth.extend_from_slice(&build_payload(seq, run, o.size));
            let sec = start.elapsed().as_secs() as usize;
            // SAFETY: `eth` and `sa` are live for the call; the length arguments are their sizes.
            let n = unsafe {
                libc::sendto(
                    fd,
                    eth.as_ptr() as *const libc::c_void,
                    eth.len(),
                    0,
                    &sa as *const libc::sockaddr_ll as *const libc::sockaddr,
                    std::mem::size_of::<libc::sockaddr_ll>() as libc::socklen_t,
                )
            };
            if n == eth.len() as isize {
                buckets.bump(sec, 1);
            } else {
                errs.bump(sec, 1);
                if first_err.is_none() {
                    first_err = Some(std::io::Error::last_os_error().to_string());
                }
            }
            while printed < buckets.v.len().saturating_sub(1) {
                println!(
                    "  t={printed:>3}s  sent={:<6} err={}",
                    buckets.v[printed],
                    errs.v.get(printed).copied().unwrap_or(0)
                );
                printed += 1;
            }
            if o.pps != 0 {
                let target = start + interval * (seq + 1);
                let now = Instant::now();
                if target > now {
                    std::thread::sleep(target - now);
                }
            }
        }
        while printed < buckets.v.len() {
            println!(
                "  t={printed:>3}s  sent={:<6} err={}",
                buckets.v[printed],
                errs.v.get(printed).copied().unwrap_or(0)
            );
            printed += 1;
        }
        unsafe { libc::close(fd) };

        let el = start.elapsed().as_secs_f64();
        println!("──────────────────────────────────────────────────────────────────────────");
        println!("  buckets (frames/s): {}", buckets.line());
        println!(
            "  sent={} err={} in {el:.2}s = {:.1} frames/s",
            buckets.total(),
            errs.total(),
            buckets.total() as f64 / el
        );
        if let Some(e) = first_err {
            println!("  first error      : {e}");
        }
        println!("  ⚠ still not evidence of radiation — read the receiver and the chip counters.");
        Ok(())
    }

    // ─────────────────────────────────────────────────────────────────────────────────────────
    // RX — through the FrameIo under test
    // ─────────────────────────────────────────────────────────────────────────────────────────

    pub async fn run_rx(spec: &str, secs: u64, o: &Opts) -> Result<(), Box<dyn std::error::Error>> {
        let face = Face::open(spec, o)?;
        println!("═══ halow_onair RX ═══════════════════════════════════════════════════════");
        if let Some(l) = o.label.as_ref() {
            println!("  label            : {l}");
        }
        face.print_header(o, spec);
        println!(
            "  format           : RawNdnS1g {{ ethertype: 0x{:04x} }}",
            o.ethertype
        );
        println!(
            "  magic            : {} ({:?})",
            hex(MAGIC),
            std::str::from_utf8(MAGIC).unwrap_or("?")
        );
        println!("  window           : {secs}s");
        println!("──────────────────────────────────────────────────────────────────────────");

        let start = Instant::now();
        let deadline = start + Duration::from_secs(secs);
        let mut all = Buckets::default();
        let mut ours = Buckets::default();
        let mut shown = 0usize;
        let mut errs = 0u64;
        let mut per: BTreeMap<u16, PerRun> = BTreeMap::new();
        let mut with_rssi = 0u64;
        let mut with_mcs = 0u64;
        let mut with_stamp = 0u64;
        let mut with_phy = 0u64;
        let mut rssi_sum: i64 = 0;
        let mut last_stamp: Option<u64> = None;
        let mut printed = 0usize;

        loop {
            let now = Instant::now();
            if now >= deadline {
                break;
            }
            let f = match tokio::time::timeout(deadline - now, face.recv_frame()).await {
                Err(_) => break,
                Ok(Err(e)) => {
                    errs += 1;
                    if errs <= 3 {
                        eprintln!("  recv error: {e}");
                    }
                    continue;
                }
                Ok(Ok(f)) => f,
            };
            let sec = start.elapsed().as_secs() as usize;
            all.bump(sec, 1);
            if f.rssi_dbm.is_some() {
                with_rssi += 1;
                rssi_sum += f.rssi_dbm.unwrap_or(0) as i64;
            }
            if f.mcs_index.is_some() {
                with_mcs += 1;
            }
            if let Some(s) = f.stamp {
                with_stamp += 1;
                last_stamp = Some(s.raw);
            }
            if f.phy.is_some() {
                with_phy += 1;
            }
            let mine = read_payload(&f.payload);
            if let Some((seq, run, _)) = mine {
                ours.bump(sec, 1);
                let e = per.entry(run).or_default();
                e.buckets.bump(sec, 1);
                e.seqs.insert(seq);
                e.n += 1;
            }

            if shown < o.show {
                shown += 1;
                println!(
                    "  ── frame #{shown} at t={:.3}s ──",
                    start.elapsed().as_secs_f64()
                );
                println!(
                    "     ours          : {}",
                    if mine.is_some() {
                        "YES (magic matched)"
                    } else {
                        "no (magic absent)"
                    }
                );
                if let Some((seq, run, sent)) = mine {
                    println!("     seq/run       : {seq} / 0x{run:04x}");
                    println!(
                        "     host Δ        : {} µs  ⚠ the two hosts' clocks are NOT synced — informational only",
                        now_us().wrapping_sub(sent) as i64
                    );
                }
                println!("     addr2 (src)   : {}", mac(&f.addr));
                println!("     addr1 (group) : {}", mac(&f.group));
                println!("     addr3         : {}", mac(&f.addr3));
                println!("     extra Blur    : {:02x?}", f.extra);
                println!("     htc           : {:?}", f.htc);
                println!("     rssi_dbm      : {:?}", f.rssi_dbm);
                println!(
                    "     mcs_index     : {:?}  (⚠ S1G MCS table, NOT 802.11n)",
                    f.mcs_index
                );
                println!("     phy           : {:?}", f.phy);
                match f.stamp {
                    None => println!("     stamp         : None"),
                    Some(s) => println!(
                        "     stamp         : raw={} domain={:?} precision_ns={} latch={:?}",
                        s.raw, s.domain, s.precision_ns, s.latch
                    ),
                }
                println!("     payload       : {} B", f.payload.len());
                println!(
                    "     head hex      : {}",
                    hex(&f.payload[..f.payload.len().min(32)])
                );
            }

            while printed < all.v.len().saturating_sub(1) {
                println!(
                    "  t={printed:>3}s  rx={:<6} ours={}",
                    all.v[printed],
                    ours.v.get(printed).copied().unwrap_or(0)
                );
                printed += 1;
            }
        }
        while printed < all.v.len() {
            println!(
                "  t={printed:>3}s  rx={:<6} ours={}",
                all.v[printed],
                ours.v.get(printed).copied().unwrap_or(0)
            );
            printed += 1;
        }

        let el = start.elapsed().as_secs_f64();
        println!("──────────────────────────────────────────────────────────────────────────");
        println!("  rx buckets   (f/s): {}", all.line());
        println!("  ours buckets (f/s): {}", ours.line());
        println!(
            "  captured={} ours={} recv_errors={} over {el:.2}s",
            all.total(),
            ours.total(),
            errs
        );
        // ── per transmitter, because a contention arm has more than one ─────────────────────
        //
        // `distinct / span` is the delivered fraction measured WITHOUT trusting the sender: the
        // span is read off the sequence numbers that actually arrived, so it needs no side
        // channel. It under-counts loss at the very edges of a run (frames lost before the first
        // arrival or after the last are outside the observed span), so the sender's own `sent`
        // total is the number to divide by when it is available, and both are printed.
        for (run, e) in per.iter() {
            let (lo, hi) = (
                e.seqs.iter().next().copied().unwrap_or(0),
                e.seqs.iter().next_back().copied().unwrap_or(0),
            );
            let span = (hi - lo) as u64 + 1;
            println!(
                "  run 0x{run:04x}   : n={} distinct={} seq {lo}..={hi} span={span} \u{21d2} {:.1}% of span",
                e.n,
                e.seqs.len(),
                e.seqs.len() as f64 * 100.0 / span as f64
            );
            println!("               buckets: {}", e.buckets.line());
        }
        if per.len() > 1 {
            println!(
                "  \u{2605} {} transmitters separated by run id \u{2014} never fold them together",
                per.len()
            );
        }
        println!(
            "  metadata populated: rssi {}/{}  mcs {}/{}  stamp {}/{}  phy {}/{}",
            with_rssi,
            all.total(),
            with_mcs,
            all.total(),
            with_stamp,
            all.total(),
            with_phy,
            all.total()
        );
        if with_rssi > 0 {
            println!(
                "  mean rssi    : {:.1} dBm",
                rssi_sum as f64 / with_rssi as f64
            );
        }
        if let Some(s) = last_stamp {
            println!("  last stamp   : {s} (radiotap TSFT, µs)");
        }
        if let Face::Nrc(n) = &face {
            println!("  mesh_common_view: {:?}", n.mesh_common_view());
        }
        if all.total() == 0 {
            println!(
                "  ⚠ ZERO frames through FrameIo. Before concluding nothing radiated, run the\n\
                 ⚠ instrument check: `halow_onair rxraw <rx-iface> {secs}` — it reports what the\n\
                 ⚠ socket saw BEFORE frame::parse filtered it, which separates a dead link from a\n\
                 ⚠ rejected decode (wrong ethertype, non-data frame, bad FCS)."
            );
        }
        Ok(())
    }

    // ─────────────────────────────────────────────────────────────────────────────────────────
    // RXRAW — the instrument check: no FrameIo, no format filter, plus the S1G TLV
    // ─────────────────────────────────────────────────────────────────────────────────────────

    pub async fn run_rxraw(
        iface: &str,
        secs: u64,
        o: &Opts,
    ) -> Result<(), Box<dyn std::error::Error>> {
        let fmt = FrameFormat::RawNdnS1g {
            ethertype: o.ethertype,
        };
        let af = AfPacketBackend::new(iface, fmt)?;
        let domain = ndn_frame_io::ClockDomainId(af.rx_ifindex() as u32);
        println!("═══ halow_onair RXRAW (instrument check) ═════════════════════════════════");
        if let Some(l) = o.label.as_ref() {
            println!("  label            : {l}");
        }
        println!(
            "  iface            : {iface}  ifindex={} domain={domain:?}",
            af.rx_ifindex()
        );
        println!(
            "  ⚠ no FrameIo wrapper and no format filter — this counts EVERY buffer the packet"
        );
        println!("  ⚠ socket delivered, so a difference against `rx` is the decoder, not the air.");
        println!("──────────────────────────────────────────────────────────────────────────");

        let start = Instant::now();
        let deadline = start + Duration::from_secs(secs);
        let mut all = Buckets::default();
        let mut magic = Buckets::default();
        let mut parsed = Buckets::default();
        let mut rt_ok = 0u64;
        let mut s1g_ok = 0u64;
        let mut shown = 0usize;
        let mut printed = 0usize;
        let mut buf = [0u8; 4096];

        loop {
            let now = Instant::now();
            if now >= deadline {
                break;
            }
            let n = match tokio::time::timeout(deadline - now, af.recv_into(&mut buf)).await {
                Err(_) => break,
                Ok(Err(e)) => {
                    eprintln!("  recv error: {e}");
                    continue;
                }
                Ok(Ok(n)) => n,
            };
            let b = &buf[..n];
            let sec = start.elapsed().as_secs() as usize;
            all.bump(sec, 1);
            let has_magic = contains_magic(b);
            if has_magic {
                magic.bump(sec, 1);
            }
            let rt = radiotap::parse(b);
            if rt.is_some() {
                rt_ok += 1;
            }
            let s1g = s1g_metadata(b);
            if s1g.is_some() {
                s1g_ok += 1;
            }
            let cf = frame::parse(fmt, b, None, None, domain);
            if cf.is_some() {
                parsed.bump(sec, 1);
            }

            // Show ours first — an unidentifiable frame is not what we came for.
            if shown < o.show && (has_magic || all.total() <= 2) {
                shown += 1;
                println!(
                    "  ── raw #{shown} at t={:.3}s, {n} B ──",
                    start.elapsed().as_secs_f64()
                );
                println!("     magic present : {has_magic}");
                println!(
                    "     radiotap      : {:?}",
                    rt.map(|r| (r.header_len, r.rssi_dbm, r.tsft, r.flags, r.freq_khz))
                );
                match s1g {
                    None => {
                        println!("     S1G TLV       : None  (⚠ not an S1G capture, or no TLV)")
                    }
                    Some((i, khz)) => println!(
                        "     S1G TLV       : bw={:?} MHz mcs={:?} ppdu={:?} sgi={:?} rssi={:?} colour={:?} uplink={:?} freq={khz:?} kHz",
                        i.bandwidth_mhz,
                        i.mcs,
                        i.ppdu_format,
                        i.short_gi,
                        i.rssi_dbm,
                        i.bss_color,
                        i.uplink
                    ),
                }
                println!(
                    "     frame::parse  : {}",
                    if cf.is_some() {
                        "accepted"
                    } else {
                        "REJECTED (not a data frame / wrong ethertype / bad FCS)"
                    }
                );
                if let Some(c) = cf.as_ref() {
                    println!("     src/group     : {} / {}", mac(&c.addr), mac(&c.group));
                    println!(
                        "     payload       : {} B, ours={}",
                        c.payload.len(),
                        read_payload(&c.payload).is_some()
                    );
                }
                println!("     first 64 B    : {}", hex(&b[..n.min(64)]));
            }

            while printed < all.v.len().saturating_sub(1) {
                println!(
                    "  t={printed:>3}s  raw={:<6} parsed={:<6} magic={}",
                    all.v[printed],
                    parsed.v.get(printed).copied().unwrap_or(0),
                    magic.v.get(printed).copied().unwrap_or(0)
                );
                printed += 1;
            }
        }
        while printed < all.v.len() {
            println!(
                "  t={printed:>3}s  raw={:<6} parsed={:<6} magic={}",
                all.v[printed],
                parsed.v.get(printed).copied().unwrap_or(0),
                magic.v.get(printed).copied().unwrap_or(0)
            );
            printed += 1;
        }

        println!("──────────────────────────────────────────────────────────────────────────");
        println!("  raw buckets    : {}", all.line());
        println!("  parsed buckets : {}", parsed.line());
        println!("  magic buckets  : {}", magic.line());
        println!(
            "  raw={} radiotap_ok={} s1g_tlv={} frame_parse_ok={} carrying_our_magic={}",
            all.total(),
            rt_ok,
            s1g_ok,
            parsed.total(),
            magic.total()
        );
        if all.total() > 0 && magic.total() == 0 {
            println!(
                "  ⇒ the socket IS delivering frames, but none are ours: the instrument works,"
            );
            println!(
                "    the transmitter is not reaching this receiver (channel? primary index?)."
            );
        }
        if magic.total() > 0 && parsed.total() == 0 {
            println!(
                "  ⇒ ★ OUR frames are on the air and frame::parse rejects them — a DECODER bug,"
            );
            println!("    not a radio failure. Check the ethertype and the FCS flag.");
        }
        Ok(())
    }
}

#[cfg(target_os = "linux")]
#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    use harness::{parse_opts, run_rx, run_rxraw, run_tx, run_txdata};

    const USAGE: &str = "\
usage:
  halow_onair tx     <ifspec> <count> [options]
  halow_onair txdata <iface>  <count> [options]
  halow_onair rx     <ifspec> <secs>  [options]
  halow_onair rxraw  <iface>  <secs>  [options]

ifspec:
  <tx>:<rx>   Morse MM6108 split data plane   (e.g. mon0:morse0)
  <iface>     Newracom NRC7292, one netdev    (e.g. halow0)

txdata sends the same identifiable payload out an OPERATING vif (ARPHRD_ETHER, e.g. halow0)
as an ordinary Ethernet frame, so the driver does the 802.11 encapsulation. On the NRC7292
that is the only transmit path that reaches the air at all — monitor injection is freed in
nrc_mac_tx before it reaches the chip. It is NOT a FrameIo; read its doc comment.

options:
  --size N   --pps N   --mcs N   --bw 1|2|4|8   --mcs-param P   --bw-param P
  --ethertype HEX   --src MAC   --dst MAC   --cli PATH   --infra-beacons
  --show N   --label TEXT

Run the rx arm TWICE: once with a transmitter running and once without.
A result with no negative control is not a measurement.";

    let args: Vec<String> = std::env::args().skip(1).collect();
    if args.len() < 3 {
        eprintln!("{USAGE}");
        std::process::exit(2);
    }
    let mode = args[0].clone();
    let spec = args[1].clone();
    let n: u64 = args[2].parse().map_err(|e| format!("{:?}: {e}", args[2]))?;
    let opts = parse_opts(&args[3..]).map_err(|e| {
        eprintln!("{USAGE}");
        e
    })?;

    match mode.as_str() {
        "tx" => run_tx(&spec, n as u32, &opts).await,
        "txdata" => run_txdata(&spec, n as u32, &opts),
        "rx" => run_rx(&spec, n, &opts).await,
        "rxraw" => run_rxraw(&spec, n, &opts).await,
        other => {
            eprintln!("unknown mode {other:?}\n{USAGE}");
            std::process::exit(2);
        }
    }
}

#[cfg(not(target_os = "linux"))]
fn main() {
    eprintln!(
        "halow_onair needs Linux AF_PACKET monitor-mode injection/capture (CAP_NET_RAW) — the \
         HaLow data plane in src/halow/linux.rs is Linux-only by construction."
    );
    std::process::exit(1);
}
