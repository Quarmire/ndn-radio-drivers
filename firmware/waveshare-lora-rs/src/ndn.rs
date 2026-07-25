//! Data-centric offload on the MCU (#52 / doctrine tasks #43-45): name-hash RX filter, a Content
//! Store, duplicate suppression, and name-keyed frequency hopping — so the dongle processes by NAME
//! at the antenna instead of shipping every frame up the 115200 serial link to the host.
//!
//! Reuses `ndn-embedded` — the same no_std NDN forwarder that runs on Cortex-M / ESP32 / RISC-V — for
//! the two data-centric primitives: `pit::fnv1a64` (the #44 shared name-hash keyspace, so the firmware
//! and the host hash a name IDENTICALLY) and `cs::ContentStore` (its fixed-capacity, allocation-free
//! cache). The dependency selects `default-features = false, features = ["cs"]`, the crate's heap-free
//! path — no fork, no copy.
//!
//! Every feature defaults INERT: an empty filter passes everything, CS-serve is off, dedup is off,
//! hopping is off. So a freshly-flashed dongle behaves exactly like the plain smart-modem until the
//! host opts a feature in — which is what makes this batch safe to flash.
//!
//! Name location: the app wire frame is `KIND|SRC|SF|NAME[|...]` (ASCII, '|'-separated); the NAME is
//! field 3. We hash its bytes — the host hashes the same string, so the keyspace matches. (Real
//! NDN-TLV frames would parse the Name TLV via `ndn-embedded`'s wire parser here instead.)

use ndn_embedded::cs::ContentStore;
use ndn_embedded::pit::fnv1a64;

const CS_N: usize = 6;
pub const CS_MAX_LEN: usize = 96;
const FILTER_CAP: usize = 24;
const DEDUP_CAP: usize = 32;
const RELAY_CAP: usize = 16;

/// What the main loop should do with a freshly received frame.
pub enum RxAction<'a> {
    /// Hand it up to the host (the normal path).
    Deliver,
    /// Drop it silently (filtered out, or a duplicate) — never crosses the serial link.
    Drop,
    /// Serve this Data from the Content Store (Interest hit) — TX it, do not wake the host.
    Serve(&'a [u8]),
    /// Re-broadcast it (relay set match) AND deliver — cooperative forwarding.
    RelayAndDeliver,
}

pub struct DataPlane {
    filter: [u64; FILTER_CAP],
    filter_len: usize,
    filter_on: bool,
    cs: ContentStore<CS_N, CS_MAX_LEN>,
    cs_serve_on: bool,
    dedup: [u64; DEDUP_CAP],
    dedup_head: usize,
    dedup_on: bool,
    relay: [u64; RELAY_CAP],
    relay_len: usize,
    relay_on: bool,
    hop_on: bool,
    hop_base_ch: u8,
    hop_span: u8,
    // --- observability counters (queried via CMD_GET_STATS, cleared via CMD_RESET_STATS) ---
    pub rx: u32,
    pub filtered: u32,
    pub deduped: u32,
    pub served: u32,
    pub relayed: u32,
}

impl DataPlane {
    pub fn new() -> Self {
        Self {
            filter: [0; FILTER_CAP],
            filter_len: 0,
            filter_on: false,
            cs: ContentStore::new(),
            cs_serve_on: false,
            dedup: [0; DEDUP_CAP],
            dedup_head: 0,
            dedup_on: false,
            relay: [0; RELAY_CAP],
            relay_len: 0,
            relay_on: false,
            hop_on: false,
            hop_base_ch: 64,
            hop_span: 3,
            rx: 0,
            filtered: 0,
            deduped: 0,
            served: 0,
            relayed: 0,
        }
    }

    /// Clear the observability counters (re-baseline without a reset).
    pub fn reset_stats(&mut self) {
        self.rx = 0;
        self.filtered = 0;
        self.deduped = 0;
        self.served = 0;
        self.relayed = 0;
    }

    /// Extract the NAME field (index 3) of a `KIND|SRC|SF|NAME[|...]` frame.
    fn name_of(frame: &[u8]) -> Option<&[u8]> {
        frame.split(|&b| b == b'|').nth(3)
    }
    /// First byte is the KIND: b'I' Interest, b'D' Data.
    fn kind_of(frame: &[u8]) -> Option<u8> {
        frame.first().copied()
    }
    /// The name-hash keyspace shared with the host (#44).
    pub fn name_hash(name: &[u8]) -> u64 {
        fnv1a64(name)
    }

    // --- host configuration (all via serial commands; no reflash to change) ---
    pub fn set_filter(&mut self, hashes: &[u64]) {
        self.filter_len = hashes.len().min(FILTER_CAP);
        self.filter[..self.filter_len].copy_from_slice(&hashes[..self.filter_len]);
        self.filter_on = self.filter_len > 0;
    }
    pub fn set_relay(&mut self, hashes: &[u64]) {
        self.relay_len = hashes.len().min(RELAY_CAP);
        self.relay[..self.relay_len].copy_from_slice(&hashes[..self.relay_len]);
        self.relay_on = self.relay_len > 0;
    }
    pub fn set_cs_serve(&mut self, on: bool) {
        self.cs_serve_on = on;
    }
    pub fn set_dedup(&mut self, on: bool) {
        self.dedup_on = on;
    }
    pub fn set_hop(&mut self, on: bool, base_ch: u8, span: u8) {
        self.hop_on = on;
        self.hop_base_ch = base_ch;
        self.hop_span = span.max(1);
    }

    /// NDN longest-prefix match: does any prefix of `name` (at a '/' component boundary) hash into
    /// `set`? This is what lets cognition install a few forwarding/subscription PREFIXES
    /// (`ndn/lora-cog/A`) that cover every seq-varying name under them (`ndn/lora-cog/A/alarm/6`),
    /// exactly like a FIB. Filter/relay match by prefix; dedup/CS still key on the FULL name (object
    /// identity), because a duplicate or a cache hit is about the exact object, not its prefix.
    ///
    /// The rolling FNV-1a matches `fnv1a64(prefix_bytes)` at each boundary — the same hash the host
    /// computes for the prefix string, so the #44 keyspace holds.
    fn any_prefix_in(set: &[u64], name: &[u8]) -> bool {
        const OFFSET: u64 = 0xcbf2_9ce4_8422_2325;
        const PRIME: u64 = 0x0000_0100_0000_01b3;
        let mut hash = OFFSET;
        for (i, &b) in name.iter().enumerate() {
            // At a component boundary, the hash so far == fnv1a64 of the prefix before this '/'.
            if b == b'/' && i > 0 && set.contains(&hash) {
                return true;
            }
            hash ^= b as u64;
            hash = hash.wrapping_mul(PRIME);
        }
        // Full name is also a prefix of itself.
        set.contains(&hash)
    }
    fn filter_contains(&self, name: &[u8]) -> bool {
        Self::any_prefix_in(&self.filter[..self.filter_len], name)
    }
    fn relay_contains(&self, name: &[u8]) -> bool {
        Self::any_prefix_in(&self.relay[..self.relay_len], name)
    }
    /// Record a hash in the dedup ring; return true if it was already present (a duplicate).
    fn seen(&mut self, h: u64) -> bool {
        if self.dedup.contains(&h) {
            return true;
        }
        self.dedup[self.dedup_head] = h;
        self.dedup_head = (self.dedup_head + 1) % DEDUP_CAP;
        false
    }

    /// The channel a given name lives on under name-keyed hopping (#40). Deterministic, so both ends
    /// compute the same carrier for the same name. NOTE: a full FHSS still needs common-view time
    /// (#41) for a listener to know *when* to be on which name's channel; this is the hop *function*.
    pub fn hop_channel(&self, name: &[u8]) -> Option<u8> {
        self.hop_on
            .then(|| self.hop_base_ch + (fnv1a64(name) % self.hop_span as u64) as u8)
    }

    /// Classify a received frame. `now_ms` is the MCU clock (for CS freshness). Auto-caches Data it
    /// sees (real in-network caching) and answers Interests from cache when serving is enabled.
    pub fn on_rx(&mut self, frame: &[u8], now_ms: u32) -> RxAction<'_> {
        let Some(name) = Self::name_of(frame) else {
            return RxAction::Deliver; // not our wire shape — pass it up untouched
        };
        self.rx = self.rx.wrapping_add(1);
        let h = fnv1a64(name);

        match Self::kind_of(frame) {
            // A Data frame: suppress duplicate objects (the flooding-suppression win), else cache it
            // (edge caching) and let the host see it. Dedup is Data-ONLY: an Interest legitimately
            // repeats (re-expression is the ARQ signal; real NDN dedups Interests by Nonce, which our
            // wire has no field for), so name-dedup on Interests would kill retries.
            Some(b'D') => {
                if self.dedup_on && self.seen(h) {
                    self.deduped = self.deduped.wrapping_add(1);
                    return RxAction::Drop;
                }
                if frame.len() <= CS_MAX_LEN {
                    self.cs.insert(h, frame, 30_000, now_ms); // 30 s freshness
                }
                if self.relay_on && self.relay_contains(name) {
                    self.relayed = self.relayed.wrapping_add(1);
                    return RxAction::RelayAndDeliver;
                }
                RxAction::Deliver
            }
            // An Interest: serve from cache if we hold the Data and serving is on — host never wakes.
            Some(b'I') => {
                if self.cs_serve_on && self.cs.lookup(h, now_ms).is_some() {
                    self.served = self.served.wrapping_add(1);
                    // Re-borrow to satisfy the borrow checker (lookup above was a probe).
                    return RxAction::Serve(self.cs.lookup(h, now_ms).unwrap());
                }
                if self.relay_on && self.relay_contains(name) {
                    self.relayed = self.relayed.wrapping_add(1);
                    return RxAction::RelayAndDeliver;
                }
                if self.filter_on && !self.filter_contains(name) {
                    self.filtered = self.filtered.wrapping_add(1);
                    return RxAction::Drop;
                }
                RxAction::Deliver
            }
            _ => RxAction::Deliver,
        }
    }
}
