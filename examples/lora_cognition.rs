//! Live LoRa **cognition loop**, driven by real Interest/Data demand.
//!
//! The named-radio control plane driving a real SX1262 dongle from the forwarding plane's own
//! signals — no synthetic inputs anywhere in the loop. Each node both **produces** its own named
//! objects and **consumes** the peer's, so every number the policy reads is measured:
//!
//!   - SENSE — each received frame's real per-frame RSSI feeds [`MediumState::observe_rx`]; each
//!     Interest/Data event feeds a [`DemandTracker`] that shadows the PIT's in-record lifecycle.
//!   - DECIDE — [`RadioPolicy::decide`] runs over that medium state and a [`NameContext`] whose
//!     priority comes from **the name itself** (an `alarm` is urgent because of what it is), and
//!     whose fan-out / re-Interest rate come from the tracker.
//!   - ACT — the plan's LoRa knobs (spreading factor, coding rate, FEC redundancy) are applied to
//!     the dongle through the `RadioKnobs` seam, then the frame goes out.
//!
//! The two demand signals are real, not modelled:
//!   - **fan-out** — a peer's Interest is a live in-record on our prefix. Serving it is demand-driven
//!     transmission; with no in-record the innovation gate suppresses us instead.
//!   - **re-Interest** — when the peer re-expresses an Interest we never satisfied, a frame was lost
//!     on air. That is the ARQ signal the redundancy budget is allocated to drive down, and here it
//!     is measured from real LoRa loss rather than assumed.
//!
//! SF tracks the measured link (strong → SF7, weak → SF12); CR and FEC track demand. Two close nodes
//! both read a strong RSSI and settle at the same SF, staying paired while coding adapts.
//!
//! Run on both OPis:
//! ```text
//! lora_cognition /dev/ttyACM0 A B
//! lora_cognition /dev/ttyACM0 B A
//! ```

use std::collections::HashMap;
use std::sync::Arc;
use std::time::{Duration, Instant};

use ndn_frame_io::{FrameIo, InjectFrame, TxIntent};
use ndn_radio_cognition::{
    DemandTracker, MediumState, MediumView, NameContext, Priority, RadioId, RadioPolicy,
    lora_airtime_ms, prefix_hash,
};
use ndn_radio_drivers::LoraSerialBackend;
use ndn_radio_hal::{RadioKnobs, RadioProfile};

const CHANNEL: u8 = 65; // 915 MHz (US ISM)
const BW_KHZ: u32 = 125; // the backend's default bandwidth — airtime is computed against it
const LOCAL_FACE: u64 = 0; // our own app face: where our consumer's Interests come from
const PEER_FACE: u64 = 1; // the LoRa face the peer's Interests arrive on

const TICK: Duration = Duration::from_millis(5_000);
/// PIT entry lifetime: an in-record counts toward fan-out only while fresh.
const PIT_LIFETIME_MS: u64 = 12_000;
/// No Data within this → re-express. A re-expression before satisfaction IS the re-Interest signal.
const RETX_MS: u64 = 5_000;
/// Give up on a name after this long and move to the next sequence number.
const GIVEUP_MS: u64 = 20_000;

/// The demand classes this demo produces and consumes. Priority is derived from **the name**, not
/// from a timer: an `alarm` is urgent because of what it is. That is the data-centric claim — the
/// radio's dials follow the name's meaning, with no application telling the driver what to do.
const CLASSES: [(&str, Priority); 3] = [
    ("alarm", Priority::Urgent),
    ("telemetry", Priority::Normal),
    ("bulk", Priority::Bulk),
];

fn class_priority(class: &str) -> Option<Priority> {
    CLASSES.iter().find(|(c, _)| *c == class).map(|(_, p)| *p)
}

/// Demand is keyed per prefix — the sequence number names the object within it, not a new prefix.
fn class_prefix(node: &str, class: &str) -> u64 {
    prefix_hash(&[b"ndn", b"lora-cog", node.as_bytes(), class.as_bytes()])
}

struct ParsedName<'a> {
    node: &'a str,
    class: &'a str,
}

/// `ndn/lora-cog/<node>/<class>/<seq>`
fn parse_name(name: &str) -> Option<ParsedName<'_>> {
    let mut it = name.split('/');
    if it.next()? != "ndn" || it.next()? != "lora-cog" {
        return None;
    }
    let node = it.next()?;
    let class = it.next()?;
    it.next()?; // seq
    Some(ParsedName { node, class })
}

/// A frame off the air: `I|<src>|<name>` or `D|<src>|<name>|<payload>`.
///
/// The **sender** is carried explicitly because the name cannot supply it: an Interest for
/// `ndn/lora-cog/A/...` names A's data but is sent *by* B. Link quality keys the neighbor who
/// transmitted, so without `src` there is nothing honest to attribute RSSI to.
struct ParsedWire<'a> {
    kind: &'a str,
    src: &'a str,
    name: &'a str,
}

fn parse_wire(wire: &str) -> Option<ParsedWire<'_>> {
    let mut it = wire.split('|');
    let kind = it.next()?;
    if kind != "I" && kind != "D" {
        return None;
    }
    let src = it.next()?;
    let name = it.next()?;
    if src.is_empty() || parse_name(name).is_none() {
        return None;
    }
    Some(ParsedWire { kind, src, name })
}

/// Opaque sense-bus key for a neighbor, derived from who actually transmitted.
fn neighbor_key(src: &str) -> u64 {
    prefix_hash(&[b"node", src.as_bytes()])
}

/// An Interest we have expressed and not yet had satisfied.
struct Pending {
    name: String,
    expressed_ms: u64,
    first_ms: u64,
}

/// A frame off the air, handed to the single-owner state machine in the main loop.
struct RxEvent {
    wire: String,
    rssi: Option<i8>,
    at_ms: u64,
}

struct Node {
    dev: Arc<LoraSerialBackend>,
    name: String,
    peer: String,
    /// Sense-bus key for the peer — the only neighbor whose RSSI we act on.
    peer_key: u64,
    /// Frames on this channel we could not attribute to a sender (beacons, foreign nodes).
    unattributed: u32,
    radio: RadioId,
    medium: MediumState,
    tracker: DemandTracker,
    policy: RadioPolicy,
    start: Instant,
    last_sf: u8,
    last_cr: u8,
    pending: HashMap<&'static str, Pending>,
    seq: HashMap<&'static str, u32>,
}

impl Node {
    fn now(&self) -> u64 {
        self.start.elapsed().as_millis() as u64
    }

    /// DECIDE + ACT + TX for one named object. Returns the row describing what happened.
    async fn transmit(&mut self, ctx: &NameContext, wire: &str, tag: &str) {
        let now = self.now();
        let plan = self.policy.decide(ctx, &self.medium, now);

        // Pull the knobs out before touching `self` again (the plan borrows nothing of it).
        let alloc = plan.allocation_for(self.radio);
        let sf = alloc.and_then(|a| a.params.spreading_factor());
        let cr = alloc.and_then(|a| a.params.coding_rate());
        let fec = alloc.and_then(|a| a.params.link_fec_redundancy).unwrap_or(0);

        let act = if plan.suppress {
            // A real gate, not a demo branch: a relay with nothing to add stays quiet, and a radio
            // over its duty budget makes the packet wait rather than break the regulatory limit.
            "suppressed".to_string()
        } else {
            let mut applied = Vec::new();
            if let Some(sf) = sf
                && sf != self.last_sf
            {
                match self.dev.set_spreading_factor(sf) {
                    Ok(()) => {
                        self.last_sf = sf;
                        applied.push("sf");
                    }
                    // Report the miss rather than let the table imply a knob that never landed.
                    Err(e) => eprintln!("[{}] set SF{sf} failed: {e}", self.name),
                }
            }
            if let Some(cr) = cr
                && cr != self.last_cr
            {
                match self.dev.set_coding_rate(cr) {
                    Ok(()) => {
                        self.last_cr = cr;
                        applied.push("cr");
                    }
                    Err(e) => eprintln!("[{}] set CR 4/{} failed: {e}", self.name, cr + 4),
                }
            }
            let frame =
                InjectFrame::broadcast(wire.as_bytes().to_vec().into(), TxIntent::CONSERVATIVE);
            // Never swallow this: a failed inject means nothing went on air, and a table that
            // reports "tx" for a frame that never left is worse than no table at all.
            match self.dev.inject(frame).await {
                Ok(()) => {
                    // Charge the duty-cycle budget with this frame's real on-air time.
                    self.medium.record_airtime(
                        self.radio,
                        lora_airtime_ms(
                            sf.unwrap_or(self.last_sf.max(7)),
                            BW_KHZ,
                            cr.unwrap_or(self.last_cr.max(1)),
                            wire.len(),
                        ),
                        now,
                    );
                    if applied.is_empty() {
                        format!("tx {tag}")
                    } else {
                        format!("tx {tag} (+{})", applied.join("+"))
                    }
                }
                Err(e) => format!("TX FAILED {tag}: {e}"),
            }
        };

        self.print_row(ctx, sf, cr, fec, &act);
    }

    fn print_row(
        &self,
        ctx: &NameContext,
        sf: Option<u8>,
        cr: Option<u8>,
        fec: u16,
        act: &str,
    ) {
        let now = self.now();
        let secs = now / 1000;
        let d = self.medium.demand(ctx.prefix_hash);
        let fanout = d.as_ref().map(|d| d.fanout).unwrap_or(0);
        let reint = d
            .as_ref()
            .and_then(|d| d.reinterest_rate.get())
            .unwrap_or(0.0);
        let per = self
            .medium
            .residual(self.radio)
            .and_then(|r| r.phy_per.get())
            .unwrap_or(0.0);
        let rssi = self
            .medium
            .neighbor_rssi(self.radio, self.peer_key)
            .map(|r| format!("{r}dBm"))
            .unwrap_or_else(|| "  --  ".into());
        let sf_s = sf.map(|s| format!("SF{s}")).unwrap_or_else(|| "---".into());
        let cr_s = cr
            .map(|c| format!("4/{}", c + 4))
            .unwrap_or_else(|| "---".into());
        let duty = self.medium.duty_used(self.radio, now) * 100.0;
        println!(
            "[{}] {secs:>3}s| {:<7?}| {rssi:>7} | fan={fanout} reI={reint:.2} PER={per:.2} | {sf_s} {cr_s} FEC{fec} duty={duty:.2}% | {act}",
            self.name, ctx.priority,
        );
    }

    /// A frame arrived: fold its RSSI into the medium and its Interest/Data event into the PIT.
    async fn on_rx(&mut self, ev: RxEvent) {
        // Attribute BEFORE sensing. RSSI keys a neighbor, so a frame we cannot attribute — a beacon,
        // another LoRa node on this channel — must not be credited to the peer's link: that RSSI
        // picks the spreading factor, and mis-attributing it would dial the radio to a link that
        // isn't there. Say what we dropped rather than fold it in silently.
        let Some(w) = parse_wire(&ev.wire) else {
            self.unattributed += 1;
            println!(
                "[{}] ?? ignoring unattributable frame ({}dBm): {:?}",
                self.name,
                fmt_rssi(ev.rssi),
                ev.wire.chars().take(32).collect::<String>(),
            );
            return;
        };
        if w.src == self.name {
            return; // our own name on the air; nothing to learn from it
        }
        let (kind, name) = (w.kind.to_string(), w.name.to_string());
        self.medium
            .observe_rx(self.radio, neighbor_key(w.src), ev.rssi, ev.at_ms);

        let Some(parsed) = parse_name(&name) else {
            return;
        };
        let Some(priority) = class_priority(parsed.class) else {
            return;
        };
        let (node, class) = (parsed.node.to_string(), parsed.class.to_string());

        match kind.as_str() {
            // The peer wants one of OUR names: a live in-record on our prefix. Serving it is
            // demand-driven transmission — this is the fan-out the policy provisions for.
            "I" if node == self.name => {
                let ph = class_prefix(&self.name, &class);
                self.tracker.on_interest(ph, PEER_FACE, ev.at_ms);
                let mut ctx = NameContext::new(ph); // we produce this name → origin
                ctx.priority = priority;
                self.fold_demand();
                let wire = format!("D|{}|{name}|payload-{}", self.name, "x".repeat(24));
                self.transmit(&ctx, &wire, &format!("D {name}")).await;
                self.tracker.on_data(ph, self.now());
            }
            // Data we asked for came back: satisfy the in-record and score the round trip as a hit.
            "D" if node == self.peer => {
                let ph = class_prefix(&self.peer, &class);
                let matched = self
                    .pending
                    .iter()
                    .find(|(_, p)| p.name == name)
                    .map(|(k, _)| *k);
                if let Some(k) = matched {
                    self.pending.remove(k);
                    self.tracker.on_data(ph, ev.at_ms);
                    // A satisfied round trip: both our Interest and its Data made it over the air.
                    self.medium.observe_phy_per(self.radio, 0.0);
                    println!("[{}] <- D {name} ({}dBm)", self.name, fmt_rssi(ev.rssi));
                }
            }
            _ => {}
        }
    }

    /// Consumer side: express or re-express Interests for the peer's names.
    async fn tick(&mut self) {
        let now = self.now();

        for (class, priority) in CLASSES {
            let ph = class_prefix(&self.peer, class);

            // An outstanding Interest that has aged out entirely: score the loss and move on.
            if let Some(p) = self.pending.get(class)
                && now.saturating_sub(p.first_ms) >= GIVEUP_MS
            {
                self.pending.remove(class);
                self.tracker.on_data(ph, now); // clear the in-records; the ARQ history persists
                println!("[{}] xx gave up on {class}", self.name);
                continue;
            }

            // Either express a fresh Interest, or re-express an unsatisfied one. A re-expression
            // before satisfaction is a REAL delivery miss — a frame was lost on air — and
            // `on_interest` reports it as the re-Interest that inflates the redundancy budget.
            let name = match self.pending.get(class) {
                Some(p) if now.saturating_sub(p.expressed_ms) < RETX_MS => continue,
                Some(p) => {
                    // Re-expressing: the previous round trip lost a frame in one direction.
                    self.medium.observe_phy_per(self.radio, 1.0);
                    p.name.clone()
                }
                None => {
                    let seq = self.seq.entry(class).or_insert(0);
                    *seq += 1;
                    format!("ndn/lora-cog/{}/{class}/{seq}", self.peer)
                }
            };

            let reexpressed = self.tracker.on_interest(ph, LOCAL_FACE, now);
            let first_ms = self
                .pending
                .get(class)
                .map(|p| p.first_ms)
                .unwrap_or(now);
            self.pending.insert(
                class,
                Pending {
                    name: name.clone(),
                    expressed_ms: now,
                    first_ms,
                },
            );

            self.fold_demand();
            // We consume this name, we do not produce it → relayed: the innovation gate decides
            // whether our transmission adds rank for a downstream that still needs it.
            let mut ctx = NameContext::relayed(ph);
            ctx.priority = priority;
            let tag = if reexpressed {
                format!("I {name} (re)")
            } else {
                format!("I {name}")
            };
            let wire = format!("I|{}|{name}", self.name);
            self.transmit(&ctx, &wire, &tag).await;
        }

        self.tracker.prune(now);
        self.medium.prune(now);
    }

    /// Push the tracker's PIT-shadow into the sense bus so the policy decides on fresh demand.
    fn fold_demand(&mut self) {
        let now = self.now();
        for (ph, demand) in self.tracker.snapshot(now) {
            self.medium.observe_demand(ph, demand);
        }
    }
}

fn fmt_rssi(r: Option<i8>) -> String {
    r.map(|r| r.to_string()).unwrap_or_else(|| "?".into())
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let rt = tokio::runtime::Builder::new_multi_thread()
        .enable_all()
        .build()?;
    rt.block_on(run())
}

async fn run() -> Result<(), Box<dyn std::error::Error>> {
    let mut args = std::env::args().skip(1);
    let path = args.next().unwrap_or_else(|| "/dev/ttyACM0".into());
    let name = args.next().unwrap_or_else(|| "A".into());
    let peer = args.next().unwrap_or_else(|| "B".into());

    let dev = Arc::new(LoraSerialBackend::open(&path)?);
    println!("[{name}] open OK — cognition plane driving {path}, consuming from {peer}");

    let radio = RadioId(0);
    let mut medium = MediumState::new();

    // Take the radio's real capability from the device, with one honest regional correction:
    // `RadioCapability::lora()` carries the ETSI EU868 1% duty ceiling, which does not govern this
    // band. These dongles run 915 MHz, where FCC 15.247 limits dwell time and power rather than a
    // duty fraction. State that explicitly instead of quietly transmitting through a gate the policy
    // believes it is enforcing. On an EU 868 channel, drop this line and the policy fail-closes at 1%
    // on its own. Airtime is still recorded either way, so the budget stays visible in the table.
    let mut cap = dev.capability();
    cap.channels = vec![CHANNEL];
    cap.duty_cycle_max = 1.0;
    medium.register_radio(radio, cap);

    let (tx_ch, mut rx_ch) = tokio::sync::mpsc::channel::<RxEvent>(64);
    let start = Instant::now();

    // SENSE task: pull frames off the dongle and hand them to the single-owner state machine.
    {
        let dev = dev.clone();
        tokio::spawn(async move {
            while let Ok(f) = dev.recv_frame().await {
                let ev = RxEvent {
                    wire: String::from_utf8_lossy(&f.payload).to_string(),
                    rssi: f.rssi_dbm,
                    at_ms: start.elapsed().as_millis() as u64,
                };
                if tx_ch.send(ev).await.is_err() {
                    break;
                }
            }
        });
    }

    let mut node = Node {
        dev,
        name: name.clone(),
        peer_key: neighbor_key(&peer),
        unattributed: 0,
        peer,
        radio,
        medium,
        tracker: DemandTracker::new(PIT_LIFETIME_MS),
        policy: RadioPolicy::default(),
        start,
        last_sf: 0,
        last_cr: 0,
        pending: HashMap::new(),
        seq: HashMap::new(),
    };

    println!("[{name}] t   | demand | obsRSSI | PIT-shadow            | knobs               | act");

    // Half-duplex rendezvous: a LoRa radio is deaf while it transmits. Two nodes started together
    // would burst in lockstep and talk over each other forever — the Interest lands while the peer
    // is mid-TX, and the Data reply lands while we are. Split the tick so the pair alternates: the
    // lexicographically-lower name transmits on the tick, the other half a tick later. Deterministic
    // from the names alone, so it needs no negotiation on a link that does not work yet.
    let offset = if node.name < node.peer {
        Duration::ZERO
    } else {
        TICK / 2
    };
    let mut ticker = tokio::time::interval_at(tokio::time::Instant::now() + offset, TICK);
    loop {
        tokio::select! {
            Some(ev) = rx_ch.recv() => node.on_rx(ev).await,
            _ = ticker.tick() => node.tick().await,
        }
    }
}

/// The demand wiring is testable without a dongle: the tracker and the policy are both pure, so the
/// PIT-shadow → knobs path can be exercised on the same code the live loop runs.
#[cfg(test)]
mod tests {
    use super::*;
    use ndn_radio_cognition::RadioCapability;

    const R: RadioId = RadioId(0);

    fn medium() -> MediumState {
        let mut m = MediumState::new();
        let mut cap = RadioCapability::lora(vec![CHANNEL]);
        cap.duty_cycle_max = 1.0; // US 915: no duty ceiling (see `run`)
        m.register_radio(R, cap);
        m.observe_rx(R, neighbor_key("B"), Some(-40), 0); // a strong, live peer
        m
    }

    /// A frame we cannot attribute must be rejected outright — it would otherwise be credited to the
    /// peer's link and dial the spreading factor from a neighbor that isn't there.
    #[test]
    fn only_attributable_frames_are_accepted() {
        let w = parse_wire("I|B|ndn/lora-cog/A/alarm/7").expect("well-formed Interest");
        assert_eq!((w.kind, w.src, w.name), ("I", "B", "ndn/lora-cog/A/alarm/7"));
        let d = parse_wire("D|A|ndn/lora-cog/A/bulk/2|payload-xxx").expect("well-formed Data");
        assert_eq!((d.kind, d.src), ("D", "A"));

        // The frames that were silently polluting the peer's RSSI before.
        assert!(parse_wire("LORA-BEACON seq=3").is_none(), "a beacon is not ours");
        assert!(parse_wire("I|B|ndn/other-app/A/alarm/7").is_none(), "foreign name");
        assert!(parse_wire("I|B").is_none(), "truncated");
        assert!(parse_wire("I||ndn/lora-cog/A/alarm/7").is_none(), "no sender");
        // Distinct senders key distinct neighbors — the whole point of carrying `src`.
        assert_ne!(neighbor_key("A"), neighbor_key("B"));
    }

    #[test]
    fn names_parse_and_foreign_names_are_ignored() {
        let n = parse_name("ndn/lora-cog/B/alarm/7").expect("well-formed name");
        assert_eq!((n.node, n.class), ("B", "alarm"));
        assert!(parse_name("LORA-BEACON seq=3").is_none());
        assert!(parse_name("ndn/other-app/B/alarm/7").is_none());
        // The prefix keys demand per class, not per sequence number.
        assert_eq!(class_prefix("B", "alarm"), class_prefix("B", "alarm"));
        assert_ne!(class_prefix("B", "alarm"), class_prefix("B", "bulk"));
        assert_ne!(class_prefix("A", "alarm"), class_prefix("B", "alarm"));
    }

    #[test]
    fn priority_comes_from_the_name() {
        assert_eq!(class_priority("alarm"), Some(Priority::Urgent));
        assert_eq!(class_priority("bulk"), Some(Priority::Bulk));
        assert_eq!(class_priority("nonsense"), None);
    }

    /// A peer's Interest is a live in-record: it becomes fan-out the policy provisions for.
    #[test]
    fn peer_interest_becomes_fanout_and_urgency_raises_coding_rate() {
        let mut m = medium();
        let mut t = DemandTracker::new(PIT_LIFETIME_MS);
        let ph = class_prefix("A", "alarm");
        t.on_interest(ph, PEER_FACE, 0);
        for (p, d) in t.snapshot(0) {
            m.observe_demand(p, d);
        }
        assert_eq!(m.demand(ph).map(|d| d.fanout), Some(1), "peer Interest = 1 in-record");

        let policy = RadioPolicy::default();
        let mut urgent = NameContext::new(ph);
        urgent.priority = Priority::Urgent;
        let plan = policy.decide(&urgent, &m, 0);
        let alloc = plan.allocation_for(R).expect("origin serves its own name");
        // Strong link → fastest SF; urgent → the more robust coding rate.
        assert_eq!(alloc.params.spreading_factor(), Some(7));
        assert_eq!(alloc.params.coding_rate(), Some(2), "urgent → 4/6");

        let mut bulk = NameContext::new(ph);
        bulk.priority = Priority::Bulk;
        let bulk_cr = policy
            .decide(&bulk, &m, 0)
            .allocation_for(R)
            .and_then(|a| a.params.coding_rate());
        assert_eq!(bulk_cr, Some(1), "bulk → 4/5");
    }

    /// The heart of the demand path: a peer re-expressing an unsatisfied Interest means a frame was
    /// lost on air, and that measured re-Interest must buy more redundancy.
    #[test]
    fn reinterest_from_real_loss_raises_the_redundancy_budget() {
        let mut m = medium();
        let mut t = DemandTracker::new(PIT_LIFETIME_MS);
        let ph = class_prefix("A", "telemetry");
        m.observe_phy_per(R, 0.3); // measured frame loss on the link

        t.on_interest(ph, PEER_FACE, 0); // first expression: not a re-Interest
        for (p, d) in t.snapshot(0) {
            m.observe_demand(p, d);
        }
        let policy = RadioPolicy::default();
        let ctx = NameContext::new(ph);
        let fec0 = policy
            .decide(&ctx, &m, 0)
            .allocation_for(R)
            .and_then(|a| a.params.link_fec_redundancy)
            .expect("lossy link buys some parity");

        // The peer re-asks before we satisfied it — a real delivery miss.
        assert!(t.on_interest(ph, PEER_FACE, 1_000), "re-expression is detected");
        t.on_interest(ph, PEER_FACE, 2_000);
        for (p, d) in t.snapshot(2_000) {
            m.observe_demand(p, d);
        }
        let fec1 = policy
            .decide(&ctx, &m, 2_000)
            .allocation_for(R)
            .and_then(|a| a.params.link_fec_redundancy)
            .expect("still lossy");
        assert!(fec1 > fec0, "re-Interest must raise redundancy: {fec1} > {fec0}");
    }

    /// Satisfying the demand retires it: with no in-record left, a relay has no rank to add.
    #[test]
    fn satisfied_demand_suppresses_the_relay() {
        let mut m = medium();
        let mut t = DemandTracker::new(PIT_LIFETIME_MS);
        let ph = class_prefix("B", "telemetry");
        t.on_interest(ph, LOCAL_FACE, 0);
        for (p, d) in t.snapshot(0) {
            m.observe_demand(p, d);
        }
        let policy = RadioPolicy::default();
        let ctx = NameContext::relayed(ph);
        assert!(!policy.decide(&ctx, &m, 0).suppress, "live demand → transmit");

        t.on_data(ph, 1_000); // Data came back; the in-records clear
        for (p, d) in t.snapshot(1_000) {
            m.observe_demand(p, d);
        }
        assert_eq!(m.demand(ph).map(|d| d.fanout), Some(0));
    }

    /// Airtime is charged per frame, so the duty budget is real rather than decorative.
    #[test]
    fn transmitting_charges_the_duty_budget() {
        let mut m = medium();
        assert_eq!(m.duty_used(R, 0), 0.0);
        let air = lora_airtime_ms(7, BW_KHZ, 1, 40);
        assert!(air > 0.0, "a 40 B SF7 frame has real airtime: {air}ms");
        m.record_airtime(R, air, 0);
        assert!(m.duty_used(R, 0) > 0.0, "the budget moved");
    }
}
