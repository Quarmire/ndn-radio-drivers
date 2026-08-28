//! Data-centric offload on the MCU (#52 / doctrine tasks #43-45): name RX filter, a Content Store,
//! duplicate suppression, and name-keyed frequency hopping — so the dongle processes by NAME at the
//! antenna instead of shipping every frame up the 115200 serial link to the host.
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
//! # The wire this plane is built for (fixed 2026-08-28, bug C2)
//!
//! It used to recognise ONLY the ASCII demo wire `KIND|SRC|SF|NAME[|...]`, taking field 3 as the name
//! and `frame[0]` as `b'I'`/`b'D'`. Nothing the real face puts on air looks like that, so every
//! offload path (filter / dedup / CS-serve / relay) was **inert on the real bearer** — the recorded
//! `filtered=22 deduped=26 served=17 relayed=31` numbers were all demo-wire traffic.
//!
//! What `ndn_phy_lora::LoraPhy::send_bytes` actually transmits is
//!
//! ```text
//!   [0xF5][len u8][GCS bytes]   <- OPTIONAL body-prefix filter TLV (`mac::gcs::BODY_PREFIX_TLV`),
//!                                  prepended only to a fragment that carries a name
//!   0x64 { … 0x52 FragIndex … 0x50 Fragment { network packet } … }   <- NDNLPv2 LpPacket
//!         or, unframed, the bare network packet
//!   network packet = 0x05 Interest | 0x06 Data { 0x07 Name { 0x08 comp … } … }
//! ```
//!
//! [`classify`] walks exactly that, bounded and allocation-free, and renders the Name TLV to the
//! `/`-joined byte form (`/a/b/c`) that `ndn_radio::mac::name::ndn_name_to_slash` produces on the
//! host — so `fnv1a64` over it lands in the same #44 keyspace the host's `set_name_filter` installs,
//! and `any_prefix_in`'s `/`-boundary walk is a real NDN longest-prefix match.
//!
//! The ASCII demo wire still works, behind an explicit second branch, so the older examples run
//! unchanged; but the real wire is the one this is built for and is tried first.

use ndn_embedded::cs::ContentStore;
use ndn_embedded::pit::fnv1a64;

/// Content Store depth. Kept small: each slot costs [`CS_MAX_LEN`] bytes of RAM.
const CS_N: usize = 4;
/// Largest frame the Content Store will cache/serve. Raised from 96 with bug C1 — the RX path used
/// to truncate at 64 B, so nothing bigger ever reached here; now a real NDN Data of up to
/// [`crate`-level] `RX_MAX` can arrive and a 96 B cache would silently skip most of them.
/// RAM cost is `CS_N * (CS_MAX_LEN + entry overhead)` ≈ 4 × 208 B ≈ 832 B of the GD32's 20 KB.
pub const CS_MAX_LEN: usize = 192;
const FILTER_CAP: usize = 24;
const DEDUP_CAP: usize = 32;
const RELAY_CAP: usize = 16;

/// Longest `/`-joined name this plane will hash. A name longer than this is treated as unparseable
/// (→ `Deliver`) rather than truncated: a truncated name hashes to a DIFFERENT value, which would
/// silently move it out of the host's keyspace and make filter/dedup/CS decisions wrong.
pub const NAME_MAX: usize = 96;

/// Packet kinds, kept as the ASCII letters the old wire used so the match arms read the same.
pub const KIND_INTEREST: u8 = b'I';
pub const KIND_DATA: u8 = b'D';

// --- The real wire's TLV types (NDN-TLV + NDNLPv2 + the named-radio body-prefix filter) ---
/// Self-signaling body-prefix GCS TLV — `ndn_radio::mac::gcs::BODY_PREFIX_TLV`. Must stay in sync.
pub const BODY_PREFIX_TLV: u8 = 0xF5;
const TLV_LP_PACKET: u8 = 0x64;
const TLV_LP_FRAGMENT: u64 = 0x50;
const TLV_LP_FRAG_INDEX: u64 = 0x52;
const TLV_INTEREST: u8 = 0x05;
const TLV_DATA: u8 = 0x06;
const TLV_NAME: u64 = 0x07;

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

// ---------------------------------------------------------------------------------------------
// Bounded, allocation-free NDN-TLV walking. `no_std`, no parser dependency: every read is a
// `.get()` and every offset an `checked_add`, so a malformed/hostile frame returns None instead of
// panicking the RX path.
// ---------------------------------------------------------------------------------------------

/// Read one NDN TLV VAR-NUMBER at `buf[*pos..]`, advancing `*pos` past it.
/// `<253` = the byte itself; `253` = u16 BE; `254` = u32 BE; `255` = u64 BE.
fn read_varnum(buf: &[u8], pos: &mut usize) -> Option<u64> {
    let first = *buf.get(*pos)?;
    let (val, adv) = match first {
        0..=252 => (first as u64, 1usize),
        253 => (be(buf, *pos + 1, 2)?, 3),
        254 => (be(buf, *pos + 1, 4)?, 5),
        _ => (be(buf, *pos + 1, 8)?, 9),
    };
    *pos = pos.checked_add(adv)?;
    Some(val)
}

/// `n` big-endian bytes at `off` as a u64 (`None` if they run off the end).
fn be(buf: &[u8], off: usize, n: usize) -> Option<u64> {
    let end = off.checked_add(n)?;
    let s = buf.get(off..end)?;
    let mut v = 0u64;
    for &b in s {
        v = (v << 8) | b as u64;
    }
    Some(v)
}

/// Strip the body-prefix GCS TLV (`[0xF5][len][gcs]`) the LoRa face prepends to a name-carrying
/// fragment. Fail-OPEN on a malformed header — hand the frame back untouched, exactly as the host's
/// gate does, so a parse slip can never manufacture a wrong classification.
fn strip_body_prefix(frame: &[u8]) -> &[u8] {
    if frame.first() == Some(&BODY_PREFIX_TLV) {
        if let Some(&len) = frame.get(1) {
            if let Some(off) = 2usize.checked_add(len as usize) {
                if let Some(rest) = frame.get(off..) {
                    return rest;
                }
            }
        }
    }
    frame
}

/// The NDN network packet inside an NDNLPv2 LpPacket (`0x64`), or the input itself when it is a bare
/// packet. `None` for a **continuation fragment** (FragIndex != 0 — it carries no name, so there is
/// nothing to filter/dedup on and it must reach the host's reassembler untouched) or a nameless /
/// malformed LP packet.
fn network_packet(body: &[u8]) -> Option<&[u8]> {
    if body.first() != Some(&TLV_LP_PACKET) {
        return Some(body);
    }
    let mut pos = 0usize;
    read_varnum(body, &mut pos)?; // 0x64
    let outer = read_varnum(body, &mut pos)? as usize;
    let end = pos.checked_add(outer)?;
    let inner = body.get(pos..end)?;

    let mut p = 0usize;
    let mut frag: Option<&[u8]> = None;
    while p < inner.len() {
        let t = read_varnum(inner, &mut p)?;
        let l = read_varnum(inner, &mut p)? as usize;
        let vend = p.checked_add(l)?;
        let val = inner.get(p..vend)?;
        match t {
            // A non-zero FragIndex is a continuation: no Name lives here.
            TLV_LP_FRAG_INDEX if val.iter().any(|&b| b != 0) => return None,
            TLV_LP_FRAGMENT => frag = Some(val),
            _ => {}
        }
        p = vend;
    }
    frag
}

/// Kind (`0x05` Interest / `0x06` Data) + the **value** bytes of the Name (`0x07`) TLV of a bare
/// network packet.
fn kind_and_name(pkt: &[u8]) -> Option<(u8, &[u8])> {
    let kind = match *pkt.first()? {
        TLV_INTEREST => KIND_INTEREST,
        TLV_DATA => KIND_DATA,
        _ => return None,
    };
    let mut pos = 0usize;
    read_varnum(pkt, &mut pos)?; // the packet type
    let len = read_varnum(pkt, &mut pos)? as usize;
    let end = pos.checked_add(len)?;
    let body = pkt.get(pos..end)?;

    // Name is the first sub-TLV of both Interest and Data, but scan rather than assume.
    let mut p = 0usize;
    while p < body.len() {
        let t = read_varnum(body, &mut p)?;
        let l = read_varnum(body, &mut p)? as usize;
        let vend = p.checked_add(l)?;
        if t == TLV_NAME {
            return Some((kind, body.get(p..vend)?));
        }
        p = vend;
    }
    None
}

/// Render a Name TLV's value (`0x08 len comp …`) into the `/`-joined byte form
/// (`/a/b/c`) that `ndn_radio::mac::name::ndn_name_to_slash` produces on the host. Component values
/// are copied verbatim, an empty name renders as the root `/`. Returns the byte count, or `None` if
/// it does not fit in `out` or does not parse — never a truncated name (see [`NAME_MAX`]).
fn name_to_slash(name_val: &[u8], out: &mut [u8]) -> Option<usize> {
    let mut n = 0usize;
    let mut p = 0usize;
    while p < name_val.len() {
        read_varnum(name_val, &mut p)?; // component type (GenericNameComponent 0x08, or typed)
        let cl = read_varnum(name_val, &mut p)? as usize;
        let vend = p.checked_add(cl)?;
        let val = name_val.get(p..vend)?;
        p = vend;
        let need = n.checked_add(1)?.checked_add(val.len())?;
        if need > out.len() {
            return None;
        }
        out[n] = b'/';
        n += 1;
        out[n..n + val.len()].copy_from_slice(val);
        n += val.len();
    }
    if n == 0 {
        *out.first_mut()? = b'/'; // the root name
        n = 1;
    }
    Some(n)
}

/// The legacy ASCII demo wire `KIND|SRC|SF|NAME[|...]` — kind in byte 0, name in `'|'`-field 3.
/// Kept so the pre-C2 examples still run; only reached when the frame is not NDN-TLV.
fn ascii_kind_and_name(frame: &[u8]) -> Option<(u8, &[u8])> {
    let kind = match *frame.first()? {
        KIND_INTEREST | KIND_DATA => *frame.first()?,
        _ => return None,
    };
    Some((kind, frame.split(|&b| b == b'|').nth(3)?))
}

/// Classify one received frame: strip any body-prefix GCS TLV, walk the LP/NDN TLVs, and write the
/// `/`-joined name into `out`. Returns `(kind, name_len)`; `None` when the frame carries no name we
/// can key on (a continuation fragment, an unparseable frame, or a name longer than `out`) — the
/// caller must then pass the frame up untouched.
///
/// Real wire first, ASCII demo wire second. The discriminators cannot collide: NDN-TLV starts
/// `0xF5`/`0x64`/`0x05`/`0x06`, the demo wire starts `b'I'` (0x49) / `b'D'` (0x44).
pub fn classify(frame: &[u8], out: &mut [u8]) -> Option<(u8, usize)> {
    let body = strip_body_prefix(frame);
    match body.first() {
        Some(&TLV_LP_PACKET) | Some(&TLV_INTEREST) | Some(&TLV_DATA) => {
            let pkt = network_packet(body)?;
            let (kind, name_val) = kind_and_name(pkt)?;
            Some((kind, name_to_slash(name_val, out)?))
        }
        _ => {
            let (kind, name) = ascii_kind_and_name(body)?;
            if name.len() > out.len() {
                return None; // truncating would hash to the wrong key
            }
            out[..name.len()].copy_from_slice(name);
            Some((kind, name.len()))
        }
    }
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

    /// The name-hash keyspace shared with the host (#44). Input is the `/`-joined name
    /// [`classify`] produces, so `lora_serial::name_hash("/a/b")` on the host and this agree.
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
    /// (`/ndn/lora-cog/A`) that cover every seq-varying name under them (`/ndn/lora-cog/A/alarm/6`),
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
    ///
    /// `frame` is the RAW on-air frame — body-prefix TLV, LP header and all. Everything cached,
    /// served and relayed is that same raw frame, so a re-broadcast is byte-identical to what a
    /// direct sender would have put on air (including the GCS filter a downstream node gates on).
    pub fn on_rx(&mut self, frame: &[u8], now_ms: u32) -> RxAction<'_> {
        let mut namebuf = [0u8; NAME_MAX];
        // Not a wire shape we can key on — a continuation fragment, an over-long name, or a frame
        // that is neither NDN-TLV nor the ASCII demo wire. Pass it up untouched: the host's
        // reassembler needs continuation fragments, and dropping an unparsed frame here would be a
        // silent hole in the face.
        let Some((kind, nlen)) = classify(frame, &mut namebuf) else {
            return RxAction::Deliver;
        };
        let name = &namebuf[..nlen];
        self.rx = self.rx.wrapping_add(1);
        let h = fnv1a64(name);

        match kind {
            // A Data frame: suppress duplicate objects (the flooding-suppression win), else cache it
            // (edge caching) and let the host see it. Dedup is Data-ONLY: an Interest legitimately
            // repeats (re-expression is the ARQ signal; real NDN dedups Interests by Nonce, which we
            // deliberately do not key on here), so name-dedup on Interests would kill retries.
            // On a fragmented Data only fragment 0 reaches this arm; the continuations Deliver above
            // and the host's reassembler discards the orphans of a suppressed duplicate.
            KIND_DATA => {
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
            KIND_INTEREST => {
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
