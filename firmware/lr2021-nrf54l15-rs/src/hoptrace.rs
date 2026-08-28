//! **A per-hop timeline this node takes of ITSELF** (`CMD_GET_HOPTRACE` / `EVT_HOPTRACE`).
//!
//! Pure logic — no MCU, no radio — so the wire encoding is host-testable. The bridge
//! (`src/bin/m6_bridge.rs`) owns the instance and feeds it; `src/timing.rs` owns the clock.
//!
//! ## The question it exists to answer
//!
//! An LR2021 and an SX1276 (Heltec) both do LoRa **intra-packet** frequency hopping, each
//! interoperates with its own kind, and they cannot hop with each other. Everything cheap has been
//! measured on hardware:
//!
//! | TX (hopping) | RX | RX hop | result |
//! |---|---|---|---|
//! | Heltec n=1 | Waveshare (no hop support) | off | **4/4** |
//! | LR2021 n=1 | Waveshare | off | **4/4** |
//! | Heltec n=1 | LR2021, hop **off** | off | **4/4** |
//! | Heltec n=1 | LR2021, hop **on** | on | **0/4** |
//! | LR2021 n=1 | Heltec, hop **on** | on | **1/20** |
//! | LR2021 n=1 | Heltec, hop **off** | off | **20/20** |
//!
//! `n = 1` is a **one-entry** hop list: the hop machinery runs and the carrier can never move. So it
//! is not the frequency sequence, not the frame format (a plain receiver decodes both parts' hopping
//! transmissions perfectly), and not structural (1/20 is not 0/20 — a format mismatch would be
//! absolute). What is left is that the two disagree about **when** a hop boundary falls, which is
//! exactly what §9.8 says: "the internal timing, frequency switching mechanisms, and control logic
//! evolved between the chip generations, making them unable to properly synchronize their hopping
//! sequences." A period sweep is already ruled out — with the Heltec's RX period fixed at 8 symbols,
//! sweeping the LR2021's TX period over 2/4/8/16 gave 0–1 of 10 at **every** setting.
//!
//! ★ **Each node timestamps its OWN hop events.** No cross-vendor reception is required to compare
//! the two, which is what makes this measurable at all: the link that would carry the comparison is
//! the very thing that is broken. At SF7/BW125 one symbol is 2^7/125000 = **1.024 ms**, so a nominal
//! 8-symbol hop period is **8.192 ms** and both parts should show that interval. Whatever differs —
//! the interval, the instant of the first hop relative to the start of a frame, or whether hops
//! continue between frames — is the answer.
//!
//! ## The wire shape
//!
//! ```text
//!   CMD_GET_HOPTRACE = 0x20   payload = []
//!   EVT_HOPTRACE     = 0x8E   payload = [stamp_hz u32 BE][n u8][ (idx u8, t_ticks u32 BE) ]*n
//! ```
//!
//! `stamp_hz` is **this node's own clock in its own units** — the same one `CMD_READ_CLOCK` returns
//! and `EVT_RX.ts` uses ([`STAMP_HZ`], 16 MHz here). The host divides; converting in firmware would
//! throw away 16× of resolution on this part just to match the Heltec's 1 MHz counter. The ring is
//! **free-running and wraps, and reading does not clear it** — the same contract as
//! `EVT_SENSE.activity`, so two reads can be differenced and a host that reads twice does not
//! destroy the timeline it is halfway through. Turning hopping **off** does not clear it either
//! ([`HopTrace::arm`]); only arming a new plan does.
//!
//! Identical, byte for byte, on the Heltec node. A second, subtly different encoding would not fail
//! loudly; it would produce two timelines that could not be compared, which is the one thing this
//! instrument exists to do.
//!
//! ## ☠ One field is NOT the same quantity on the two nodes: `idx`
//!
//! The layout is identical and every other field means the same thing, but `idx` does not, and a
//! host that compares it **across** nodes will read an artefact as a result:
//!
//! | | Heltec (SX1276) | this node (LR2021) |
//! |---|---|---|
//! | source | `RegHopChannel`'s 6-bit `FhssPresentChannel`, **the chip's own counter, verbatim** | a **firmware** count of hop interrupts, mod the table depth |
//! | detects a hop the MCU coalesced | **yes** — the field jumps by more than 1 | **no** — it advances once per *recorded* event |
//! | says whether the modem restarts at 0 per packet | **yes** | **no** — it is never reset per frame, by construction |
//!
//! The asymmetry is not a choice: the LR2021 exposes no hop-index register at all (see
//! [`HopTrace::push_hop`]). ★ **Compare `t_ticks` across the two nodes; compare `idx` only within
//! one.** On this node `idx` carries no information the entry's ordinal does not already carry, and
//! its absolute value has no defined phase relative to the chip's real table position — so "the two
//! nodes' first hop is labelled differently" is a fact about this firmware's counter, never about
//! the silicon.
//!
//! ## Where the stamp is taken, and what sits between it and the RF boundary
//!
//! The stamp is `TIMER20.CC[0]`, latched **by DPPI in silicon** at the rising edge of the LR2021's
//! DIO8 when the hop interrupt asserts — the same capture register, the same free-running 16 MHz
//! counter and the same GPIOTE/DPPI route that `crate::timing::RxCapture` already uses for
//! `EVT_RX.ts`. Sharing the timebase is the point: a hop instant and a frame arrival on one node are
//! directly subtractable.
//!
//! Between the real RF hop boundary and that latched tick sit, in order:
//!
//! | term | size |
//! |---|---|
//! | carrier transition → IRQ assertion inside the LR2021 | **NOT MEASURED.** `IRQ_MASK_FHSS` is documented as firing "after each ramp-up", i.e. deliberately *after* the PLL/PA settle, so it is a positive offset of unknown size. `IRQ_MASK_LORA_TX_RX_HOP` states no phase at all. |
//! | DIO8 pad + shield trace → P1.04 | ns; below one 62.5 ns tick |
//! | GPIOTE edge detect → DPPI → `CC[0].CAPTURE` | a few 16 MHz cycles, fixed. The same silicon path M4 measured for the RX stamp |
//! | CPU wake, SPI read of the IRQ status, `CC[0]` read | **lands after the latch and cannot contaminate it** — that is the whole reason for the DPPI route |
//!
//! **The first term is not folded into the timestamp and no guess is substituted for it.** A trace
//! whose own offset is unknown cannot answer a timing question on its own — but it can answer this
//! one, because the quantities being compared (the *interval* between hops, and the *phase* of the
//! first hop relative to a frame) are differences, and a constant offset cancels in both.
//!
//! ⚠ The one entry whose stamp is **not** a hardware capture is the TX key-up marker
//! ([`IDX_TX_KEYED`]); see its own note.
//!
//! ## The DIO8 sharing hazard, stated rather than hidden
//!
//! DIO8 is one line carrying every enabled interrupt, and it stays high until the IRQ is cleared
//! over SPI. A second event while it is already high therefore produces **no new rising edge**, so
//! `CC[0]` always holds the **first** edge since the last `get_and_clear_irq`.
//!
//! That is exact and usable, and it has one consequence worth stating plainly. If a hop and an
//! `RxDone` fall into the same poll window, both share the one capture, and **`CC[0]` is whichever
//! of the two happened FIRST — not necessarily the hop.** Inside a packet the hop does come first,
//! but the reverse case is real and not rare: after an `RxDone` the modem stays in RX and keeps
//! hopping, so a hop landing in the ≤1 ms before the next poll stamps its entry with the *frame's*
//! instant. At a 1 ms poll and an 8.192 ms period that is ~12% of receptions — **derived from those
//! two numbers, not measured.**
//!
//! Such an entry is therefore **ambiguous, not merely offset**, and the host must drop it rather
//! than difference it. Both halves of the detection are provided: the count is in
//! [`HopTrace::coalesced`] (reported by `CMD_SET_HOP`'s debug dump), and the entry itself is
//! identifiable on the wire because the offending `EVT_RX.ts` appears **verbatim** as a `t_ticks`
//! here. The corruption is bounded by one hop period and is visible; it is never silent.
//!
//! It is also gated: **the hop interrupt is added to the DIO8 mask only while a hop plan is
//! enabled**, so every non-hopping configuration (which is every measurement taken on this board to
//! date) has a bit-identical RX stamp path. ★ And the cleanest avoidance is a run discipline, not
//! firmware: **take the hop timeline on a node that is not also receiving frames.** With no `RxDone`
//! there is nothing to share the capture with, and the trace is unambiguous by construction.
//!
//! The alternative — a second radio DIO on its own MCU pin and its own capture channel — is not
//! available: the shield routes exactly one LR2021 DIO to the XIAO (`DIO8` → D0 → P1.04, see
//! [`crate::board`]), so there is no second line to capture without modifying hardware. The RX
//! capture path itself is **not** taken away: `RxCapture` keeps `GPIOTE20_CH0`, `PPI20_CH0` and
//! `CC[0]`, and the hop event is an additional source on the line it already watches.

/// Tick rate of every `t_ticks` in [`HopTrace::encode`] — **the node's own clock, in its own
/// units**, mirroring `crate::timing::MAC_CLOCK` (16 MHz ⇒ 62.5 ns).
///
/// Deliberately the same value `EVT_CAP.stamp_hz` and `EVT_RX.ts` already use, so a hop instant and
/// a frame arrival need no conversion to be compared. The device-side timer module cannot be
/// referenced from here (it is `cfg(target_os = "none")`), so the two are pinned together by a
/// const assert in `crate::timing` rather than by hope.
pub const STAMP_HZ: u32 = 16_000_000;

/// Ring depth: **32 events**.
///
/// Enough to cover several frames at an 8-symbol period, and small enough that the whole trace fits
/// one 255-byte serial payload: `4 + 1 + 32*5 = 165` bytes ([`MAX_BODY_LEN`]).
///
/// ⚠ **A host should read soon after arming.** At SF7/BW125 with an 8-symbol period a hop is 8.192 ms,
/// so 32 entries is ~260 ms of timeline — about 2–3 frames' worth. And if the chip turns out to hop
/// *between* frames as well as inside them (one of the open questions this exists to settle) the ring
/// churns at ~122 events/s whether or not anything is on air. The wrap is deliberate for the same
/// reason `EVT_SENSE.activity`'s is: a saturating ring silently stops recording, and a host reading a
/// stuck instrument sees a stopped clock as a stable one.
pub const TRACE_CAP: usize = 32;

/// Wire size of one entry: `idx u8` + `t_ticks u32 BE`.
pub const ENTRY_LEN: usize = 5;

/// Largest `EVT_HOPTRACE` body: `stamp_hz(4) + n(1) + TRACE_CAP entries`.
pub const MAX_BODY_LEN: usize = 4 + 1 + TRACE_CAP * ENTRY_LEN;

/// **Marker bit in `idx`: this entry is a TRANSMIT key-up, not a hop.**
///
/// The hop table is at most `crate::phy::MAX_HOPS` = 40 entries, so a real index is 0..39 and the
/// top two bits of the byte cannot occur naturally. Using one as a flag keeps the wire layout
/// **exactly** as specified — same field, same width, same entry size — while answering the second
/// half of the question: *where in the hop sequence did a frame start?* Mask with [`IDX_MASK`] to
/// read the index; test [`IDX_TX_KEYED`] to know what the entry is.
///
/// ⚠ **Its stamp is a software read, not a DPPI capture**, and it is the only entry in the ring that
/// is. It is taken on the same `TIMER20` counter immediately before the `SetTx` command leaves the
/// MCU, so between it and the RF key-up sit the `SetTx` SPI transaction (5 bytes at 8 MHz ≈ 5 µs)
/// and the chip's own PLL/PA ramp — **neither measured here**. The part offers no hardware key-up
/// event to capture instead: `IRQ_MASK_TX_TIMESTAMP` marks the *end* of a transmitted packet, not
/// its start. The **index** in this entry is exact regardless; only its instant carries that offset.
pub const IDX_TX_KEYED: u8 = 0x80;

/// Mask that recovers the hop-list index from an `idx` byte carrying [`IDX_TX_KEYED`].
pub const IDX_MASK: u8 = 0x3F;

/// One recorded event.
#[derive(Clone, Copy, PartialEq, Eq, Debug, Default)]
pub struct HopEvent {
    /// Hop-list index, plus [`IDX_TX_KEYED`] on a transmit marker.
    pub idx: u8,
    /// Raw `TIMER20` ticks at [`STAMP_HZ`], free-running, wrapping at 2^32 — the same counter and
    /// the same wrap as `EVT_RX.ts` and `CMD_READ_CLOCK`'s low word.
    pub ticks: u32,
}

/// The free-running ring of hop events.
///
/// **Reading does not clear it** and it wraps at [`TRACE_CAP`], keeping the most recent events. A
/// clearing read would make two hosts (or one host reading twice) destroy each other's view of a
/// timeline that is only a few hundred milliseconds long.
#[derive(Clone, Copy)]
pub struct HopTrace {
    buf: [HopEvent; TRACE_CAP],
    /// Where the next event goes.
    head: usize,
    /// How many valid entries, saturating at [`TRACE_CAP`].
    len: usize,
    /// Table depth of the live hop plan, 0 when hopping is off. **This is what arms the trace**:
    /// with hopping off the node must answer `n = 0`, never a stale timeline.
    n_hops: u8,
    /// The index the *next* hop event will move to — a firmware counter, see [`HopTrace::push_hop`].
    next_idx: u8,
    /// How many times a hop and an `RxDone` were seen in one poll, so the DIO8-sharing hazard is a
    /// counted quantity rather than a paragraph. Free-running; wraps.
    pub coalesced: u32,
}

impl Default for HopTrace {
    fn default() -> Self {
        Self::new()
    }
}

impl HopTrace {
    pub const fn new() -> Self {
        Self {
            buf: [HopEvent { idx: 0, ticks: 0 }; TRACE_CAP],
            head: 0,
            len: 0,
            n_hops: 0,
            next_idx: 0,
            coalesced: 0,
        }
    }

    /// **Arm or disarm the trace.** `n_hops` is the depth of the live hop table; 0 disarms.
    ///
    /// Called from `CMD_SET_HOP` (both directions) and from `CMD_SET_PHY`, which drops the hop plan.
    ///
    /// ★ **ARMING clears the ring; DISARMING does NOT.** The asymmetry is deliberate and it is the
    /// contract this node shares with the Heltec (`heltec-lora-rs`), whose ring is likewise never
    /// cleared by `CMD_SET_HOP 0`:
    ///
    /// * arming a plan **must** clear, or a timeline recorded under one plan would be replayed
    ///   under another — a fabricated answer to the exact question this instrument settles;
    /// * disarming **must not**, because *reading the trace after a run is the use case*. A host
    ///   that stops the hopping before pulling the timeline is doing the obvious thing, and a
    ///   clear-on-disable would hand it `n = 0` — which the measurement protocol reads as **"this
    ///   part does not signal its hops at all"**, the single most consequential wrong conclusion
    ///   available here. A stale timeline cannot cause that error: its stamps are on the wire and
    ///   say how old they are, and at SF7/8 symbols the 32-entry ring turns over in 262 ms anyway.
    ///
    /// So `n = 0` from this node means exactly one thing — **no hop event has been recorded since
    /// the last plan was armed** — on both nodes, which is what makes it readable as evidence.
    ///
    /// `next_idx` starts at `1 % n_hops`, because the carrier begins the frame on entry 0 and the
    /// first hop **moves to** entry 1. See [`HopTrace::push_hop`] for why that labelling is a
    /// firmware convention and not a chip readback — and [`HopTrace::push_hop`] again for why this
    /// node's `idx` is NOT the same quantity as the Heltec's and must never be compared across the
    /// two.
    pub fn arm(&mut self, n_hops: u8) {
        if n_hops > 0 {
            self.head = 0;
            self.len = 0;
            self.next_idx = if n_hops > 1 { 1 } else { 0 };
        }
        self.n_hops = n_hops;
    }

    /// Is a hop plan live? A disarmed trace **records nothing new**, but keeps and still serves
    /// whatever the last armed plan recorded — see [`HopTrace::arm`].
    pub const fn armed(&self) -> bool {
        self.n_hops > 0
    }

    /// Depth of the live hop table (0 when disarmed).
    pub const fn hop_count(&self) -> u8 {
        self.n_hops
    }

    /// The index the next hop event will be labelled with — also what a transmit key-up records.
    pub const fn next_idx(&self) -> u8 {
        self.next_idx
    }

    /// Number of entries currently held, 0..[`TRACE_CAP`].
    pub const fn len(&self) -> usize {
        self.len
    }

    pub const fn is_empty(&self) -> bool {
        self.len == 0
    }

    /// Record one hop event at `ticks`. No-op while disarmed. Returns the index it was labelled with.
    ///
    /// ★ **`idx` is a FIRMWARE COUNTER, not a chip register.** The SX1276 reports the live hop index
    /// in `RegHopChannel`; this part exposes no equivalent — `SetLoraHopping` (opcode 556) is
    /// commented out of the vendor command spec entirely, there is no `GetLoraHopStatus`, and no
    /// documented address in `lr2021::constants` reads one back. So the index is derived by counting
    /// hop interrupts modulo the table depth, and that derivation is exact **only if** the chip does
    /// not restart the sequence at entry 0 for each packet. Whether it does is one of the things
    /// this trace exists to find out, which is why the counter is deliberately **not** reset per
    /// frame. In the `n = 1` configuration the question that is actually open — the one every
    /// interoperability run above uses — every index is 0 either way, so the ambiguity does not
    /// touch the measurement.
    ///
    /// ☠ **Consequently this `idx` is NOT the Heltec's `idx`**, even though the byte, the mask and
    /// the flag bit are identical. The Heltec puts `RegHopChannel`'s own field on the wire, which is
    /// the chip counting; this puts a firmware count of *recorded* events, which cannot jump when a
    /// hop is coalesced and cannot reveal a per-packet reset. Compare `t_ticks` between the two
    /// nodes and `idx` only inside one — see the module note.
    pub fn push_hop(&mut self, ticks: u32) -> u8 {
        if !self.armed() {
            return 0;
        }
        let idx = self.next_idx;
        self.push(HopEvent { idx, ticks });
        self.next_idx = if self.n_hops > 1 {
            (idx + 1) % self.n_hops
        } else {
            0
        };
        idx
    }

    /// Record a transmit key-up at the current hop index. No-op while disarmed.
    ///
    /// Costs **no SPI**: the index is the firmware counter above and `ticks` is an MCU timer-register
    /// read, so nothing is added to the transmit hot path that could move the frame it is marking.
    /// See [`IDX_TX_KEYED`] for what sits between this stamp and the RF key-up.
    pub fn push_tx_keyed(&mut self, ticks: u32) {
        if !self.armed() {
            return;
        }
        let idx = (self.next_idx & IDX_MASK) | IDX_TX_KEYED;
        self.push(HopEvent { idx, ticks });
    }

    /// **Fold one interrupt-status poll into the trace** — the pure half of the bridge's `note_irq`,
    /// here so it can be tested on the host rather than only on a bench.
    ///
    /// `stamp` is the capture register read **once** for this status word: `CC[0]` holds the instant
    /// of the *first* DIO8 rising edge since the previous clear, because the line stays high until
    /// the interrupt is cleared over SPI and a second event therefore raises no new edge.
    ///
    /// ⚠ When `hop` and `rx_done` are both set they share that one capture, and `stamp` is
    /// **whichever of the two happened first** — inside a packet that is the hop, but after an
    /// `RxDone` the modem keeps hopping and the next hop can be stamped with the frame's instant
    /// instead. The entry is therefore *ambiguous* and a host must drop it, not correct it. Nothing
    /// here guesses which case it was: the occurrence is counted in [`HopTrace::coalesced`] and the
    /// entry is identifiable on the wire, because the frame's `EVT_RX.ts` reappears verbatim as this
    /// entry's `t_ticks`. Bounded by one hop period, and never silent.
    pub fn note(&mut self, hop: bool, rx_done: bool, stamp: u32) {
        if !self.armed() || !hop {
            return;
        }
        self.push_hop(stamp);
        if rx_done {
            self.coalesced = self.coalesced.wrapping_add(1);
        }
    }

    fn push(&mut self, e: HopEvent) {
        self.buf[self.head] = e;
        self.head = (self.head + 1) % TRACE_CAP;
        if self.len < TRACE_CAP {
            self.len += 1;
        }
    }

    /// The events, **oldest first** — the order [`HopTrace::encode`] writes them in.
    pub fn iter(&self) -> impl Iterator<Item = HopEvent> + '_ {
        let start = (self.head + TRACE_CAP - self.len) % TRACE_CAP;
        (0..self.len).map(move |i| self.buf[(start + i) % TRACE_CAP])
    }

    /// Encode the `EVT_HOPTRACE` body into `dst`, returning the byte count (0 if it would not fit).
    ///
    /// `[stamp_hz u32 BE][n u8][ (idx u8, t_ticks u32 BE) ]*n`, **most recent last**. A node with
    /// nothing recorded encodes `n = 0` and still carries `stamp_hz`, so a host learns the units
    /// even from a node with nothing to report. `n = 0` means *nothing recorded since the last plan
    /// was armed* — **not** "hopping is currently off"; see [`HopTrace::arm`] for why that
    /// distinction is the one thing this node must not get wrong.
    pub fn encode(&self, dst: &mut [u8]) -> usize {
        let n = self.len.min(TRACE_CAP);
        let need = 4 + 1 + n * ENTRY_LEN;
        if dst.len() < need {
            return 0;
        }
        dst[0..4].copy_from_slice(&STAMP_HZ.to_be_bytes());
        dst[4] = n as u8;
        for (i, e) in self.iter().enumerate() {
            let o = 5 + i * ENTRY_LEN;
            dst[o] = e.idx;
            dst[o + 1..o + 5].copy_from_slice(&e.ticks.to_be_bytes());
        }
        need
    }
}

/// Decode an `EVT_HOPTRACE` body into `(stamp_hz, events)` — the host's half of [`HopTrace::encode`].
///
/// Exists so the encoding has a round-trip test rather than a paragraph of prose each reader
/// re-derives, and so the fleet's two hop-capable nodes can be checked against one implementation.
/// Returns `None` for a body that is short, or whose `n` disagrees with its length: a trace that is
/// silently truncated is worse than one that is refused, because the missing tail looks like a gap
/// in the timeline.
pub fn decode(body: &[u8], out: &mut [HopEvent]) -> Option<(u32, usize)> {
    if body.len() < 5 {
        return None;
    }
    let stamp_hz = u32::from_be_bytes([body[0], body[1], body[2], body[3]]);
    let n = body[4] as usize;
    if body.len() != 5 + n * ENTRY_LEN || n > out.len() {
        return None;
    }
    for i in 0..n {
        let o = 5 + i * ENTRY_LEN;
        out[i] = HopEvent {
            idx: body[o],
            ticks: u32::from_be_bytes([body[o + 1], body[o + 2], body[o + 3], body[o + 4]]),
        };
    }
    Some((stamp_hz, n))
}

/// Symbol duration in nanoseconds for a LoRa `sf`/`bw_hz` pair — `2^sf / bw`.
///
/// Here rather than in `crate::airtime` because it is what turns a tick interval into a **symbol
/// count**, which is the unit the two chips' hop periods are configured in and therefore the unit
/// the comparison has to be made in. At SF7/125 kHz it is 1,024,000 ns, so a nominal 8-symbol hop
/// period is 8.192 ms = **131,072 ticks** at [`STAMP_HZ`].
pub const fn lora_symbol_ns(sf: u8, bw_hz: u32) -> u64 {
    (1u64 << sf) * 1_000_000_000 / bw_hz as u64
}

/// Ticks in one LoRa hop period of `period` symbols at [`STAMP_HZ`] — what a host expects to see
/// between two consecutive hop entries, and what a measurement is compared against.
pub const fn hop_period_ticks(sf: u8, bw_hz: u32, period: u16) -> u64 {
    lora_symbol_ns(sf, bw_hz) * period as u64 * STAMP_HZ as u64 / 1_000_000_000
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The exact bytes of a three-event trace, pinned. A wire encoding shared with a second firmware
    /// is only worth anything if a change to it fails here rather than on the bench.
    #[test]
    fn body_bytes_are_pinned() {
        let mut t = HopTrace::new();
        t.arm(1);
        t.push_hop(0x0000_1000);
        t.push_hop(0x0002_1000); // +131072 ticks = 8.192 ms = 8 symbols at SF7/125k
        t.push_hop(0x0004_1000);
        let mut b = [0u8; MAX_BODY_LEN];
        let n = t.encode(&mut b);
        assert_eq!(n, 4 + 1 + 3 * ENTRY_LEN);
        assert_eq!(
            &b[..n],
            &[
                0x00, 0xF4, 0x24, 0x00, // stamp_hz = 16_000_000
                0x03, // n
                0x00, 0x00, 0x00, 0x10, 0x00, // idx 0 @ 0x1000
                0x00, 0x00, 0x02, 0x10, 0x00, // idx 0 @ 0x21000
                0x00, 0x00, 0x04, 0x10, 0x00, // idx 0 @ 0x41000
            ]
        );
        // And the whole 7E-A5 frame the host sees.
        let mut f = [0u8; 5 + MAX_BODY_LEN];
        let k = crate::serial::encode(&mut f, crate::serial::EVT_HOPTRACE, &b[..n]);
        assert_eq!(&f[..5], &[0x7E, 0xA5, 0x8E, 0x14, 0x00]);
        assert_eq!(k, 5 + n);
    }

    /// Round trip through the host-side decoder, including the empty body a disarmed node emits.
    #[test]
    fn encode_decode_round_trips() {
        let mut t = HopTrace::new();
        let mut b = [0u8; MAX_BODY_LEN];
        let mut ev = [HopEvent::default(); TRACE_CAP];

        // Disarmed: n = 0, and the units still travel.
        let n = t.encode(&mut b);
        assert_eq!(n, 5);
        assert_eq!(decode(&b[..n], &mut ev), Some((STAMP_HZ, 0)));

        t.arm(4);
        for i in 0..7u32 {
            t.push_hop(1000 + i * 131_072);
        }
        let n = t.encode(&mut b);
        let (hz, k) = decode(&b[..n], &mut ev).unwrap();
        assert_eq!(hz, STAMP_HZ);
        assert_eq!(k, 7);
        for (i, e) in t.iter().enumerate() {
            assert_eq!(ev[i], e);
        }
        // The index walks the table and wraps: the carrier starts on 0, so the first event moves to 1.
        assert_eq!(
            ev[..7].iter().map(|e| e.idx).collect::<Vec<_>>(),
            [1, 2, 3, 0, 1, 2, 3]
        );

        // A body whose `n` disagrees with its length is refused, not truncated.
        assert_eq!(decode(&b[..n - 1], &mut ev), None);
        assert_eq!(decode(&b[..4], &mut ev), None);
    }

    /// **Free-running and wrapping, and a read does not clear it** — the `EVT_SENSE.activity`
    /// contract. A clearing read would let one host destroy another's view of a timeline that is
    /// only a few hundred milliseconds long.
    #[test]
    fn the_ring_wraps_and_a_read_does_not_clear_it() {
        let mut t = HopTrace::new();
        t.arm(1);
        for i in 0..TRACE_CAP as u32 + 5 {
            t.push_hop(i);
        }
        assert_eq!(t.len(), TRACE_CAP);
        // Oldest first, most recent last, and the first five are gone.
        let got: Vec<u32> = t.iter().map(|e| e.ticks).collect();
        assert_eq!(got.first(), Some(&5));
        assert_eq!(got.last(), Some(&(TRACE_CAP as u32 + 4)));
        assert_eq!(got.len(), TRACE_CAP);

        let mut a = [0u8; MAX_BODY_LEN];
        let mut b = [0u8; MAX_BODY_LEN];
        let na = t.encode(&mut a);
        let nb = t.encode(&mut b);
        assert_eq!(na, MAX_BODY_LEN);
        assert_eq!(a[..na], b[..nb], "reading must not consume the ring");
    }

    /// **`n = 0` means "nothing recorded since the last arm", on this node and on the Heltec.**
    ///
    /// A node that has never hopped records nothing and encodes `n = 0`; ARMING a plan clears the
    /// ring so one plan's timeline cannot be served under another; DISARMING keeps it, because
    /// reading the trace after a run is the use case and a clear-on-disable would answer `n = 0` to
    /// a host that merely stopped the hopping first — which the measurement protocol reads as "this
    /// part does not signal its hops", the worst available wrong conclusion. This is the same
    /// contract `heltec-lora-rs` implements; the two nodes must not differ here or the comparison
    /// depends on the order the harness happens to send its commands in.
    #[test]
    fn n_zero_means_nothing_recorded_and_a_disarm_keeps_the_timeline() {
        let mut t = HopTrace::new();
        assert!(!t.armed());
        t.push_hop(1234);
        t.push_tx_keyed(5678);
        assert_eq!(t.len(), 0);
        let mut b = [0u8; MAX_BODY_LEN];
        assert_eq!(t.encode(&mut b), 5);
        assert_eq!(b[4], 0);

        t.arm(2);
        t.push_hop(1);
        t.push_hop(2);
        assert_eq!(t.len(), 2);

        // ★ Disabling does NOT clear — the run is still readable, which is the whole point.
        t.arm(0);
        assert!(!t.armed());
        assert_eq!(t.len(), 2);
        assert_eq!(t.encode(&mut b), 5 + 2 * ENTRY_LEN);
        assert_eq!(b[4], 2);
        // …and a disarmed node still records nothing new.
        t.push_hop(3);
        t.push_tx_keyed(4);
        assert_eq!(t.len(), 2);

        // ARMING a new plan clears, so a timeline is never replayed under a plan that did not
        // produce it.
        t.arm(4);
        assert_eq!(t.len(), 0);
        assert_eq!(t.encode(&mut b), 5);
        assert_eq!(b[4], 0);
    }

    /// The transmit marker rides in the `idx` byte without changing the entry layout, and it cannot
    /// collide with a real index: the chip's table is 40 deep and the flag is bit 7.
    #[test]
    fn the_tx_marker_cannot_collide_with_a_hop_index() {
        assert!(crate::phy::MAX_HOPS as u8 <= IDX_MASK);
        assert_eq!(IDX_TX_KEYED & IDX_MASK, 0);

        let mut t = HopTrace::new();
        t.arm(4);
        t.push_hop(100); // -> idx 1
        t.push_tx_keyed(150); // keyed while the next hop would be 2
        t.push_hop(200); // -> idx 2
        let got: Vec<(u8, u32)> = t.iter().map(|e| (e.idx, e.ticks)).collect();
        assert_eq!(got, [(1, 100), (0x82, 150), (2, 200)]);
        // A host reads the two apart with the mask and the flag, and the entry is still 5 bytes.
        assert_ne!(got[1].0 & IDX_TX_KEYED, 0);
        assert_eq!(got[1].0 & IDX_MASK, 2);
        assert_eq!(got[0].0 & IDX_TX_KEYED, 0);
    }

    /// A full ring must fit one serial payload — the reason the depth is 32 and not more.
    #[test]
    fn a_full_trace_fits_one_serial_frame() {
        assert_eq!(MAX_BODY_LEN, 165);
        assert!(MAX_BODY_LEN <= crate::serial::MAX_PAYLOAD);
        let mut t = HopTrace::new();
        t.arm(1);
        for i in 0..TRACE_CAP as u32 * 3 {
            t.push_hop(i);
        }
        let mut b = [0u8; MAX_BODY_LEN];
        assert_eq!(t.encode(&mut b), MAX_BODY_LEN);
        // A short destination refuses rather than writing a partial trace.
        let mut small = [0u8; MAX_BODY_LEN - 1];
        assert_eq!(t.encode(&mut small), 0);
    }

    /// The wrap is the same 32-bit wrap `EVT_RX.ts` has, and a host differencing two stamps across
    /// it gets the right interval — which is what a hop *interval* measurement is made of.
    #[test]
    fn intervals_are_correct_across_the_32_bit_wrap() {
        let mut t = HopTrace::new();
        t.arm(1);
        let period = hop_period_ticks(7, 125_000, 8) as u32;
        t.push_hop(u32::MAX - period / 2);
        t.push_hop((u32::MAX - period / 2).wrapping_add(period));
        let v: Vec<u32> = t.iter().map(|e| e.ticks).collect();
        assert_eq!(v[1].wrapping_sub(v[0]), period);
    }

    /// **The numbers the measurement is read against.** SF7/125 kHz ⇒ 1.024 ms per symbol, so an
    /// 8-symbol hop period is 8.192 ms = 131,072 ticks at 16 MHz. If a trace shows something else,
    /// that difference is the answer.
    #[test]
    fn the_expected_hop_interval() {
        assert_eq!(lora_symbol_ns(7, 125_000), 1_024_000);
        assert_eq!(hop_period_ticks(7, 125_000, 8), 131_072);
        assert_eq!(hop_period_ticks(7, 125_000, 1), 16_384);
        // 8.192 ms, expressed the other way round: 8192 MICROSECONDS.
        assert_eq!(
            hop_period_ticks(7, 125_000, 8) * 1_000_000 / STAMP_HZ as u64,
            8192
        );
        // And the resolution the LR2021 side brings: 62.5 ns, i.e. 1/16384 of a symbol.
        assert_eq!(STAMP_HZ / 16, 1_000_000);
    }

    /// **The DIO8-sharing case, pinned.** A hop and an `RxDone` in one poll share one capture; the
    /// value belongs to the hop, the collision is counted, and the frame's `ts` — which the bridge
    /// takes from the same read — reappears verbatim in the ring, so a host can *see* it happened.
    #[test]
    fn a_coalesced_hop_and_rxdone_are_counted_and_visible() {
        let mut t = HopTrace::new();
        t.arm(1);
        t.note(true, false, 100); // an ordinary hop
        t.note(false, true, 200); // an ordinary frame: nothing recorded
        t.note(true, true, 300); // both in one poll
        assert_eq!(t.coalesced, 1);
        let v: Vec<u32> = t.iter().map(|e| e.ticks).collect();
        assert_eq!(v, [100, 300]);
        // The host's detection rule: an EVT_RX.ts that also appears as a hop t_ticks is a shared
        // capture, and the frame's true arrival is later by up to one hop period.
        assert!(v.contains(&300));
        // Disarmed, nothing is recorded and nothing is counted, whatever the interrupt said.
        let mut t = HopTrace::new();
        t.note(true, true, 400);
        assert_eq!((t.len(), t.coalesced), (0, 0));
    }

    /// The trace's units are the node's own and must be the ones `EVT_CAP` already advertises — a
    /// second timebase is exactly the error the "do not convert in firmware" rule exists to prevent.
    #[test]
    fn the_trace_shares_the_rx_stamps_timebase() {
        assert_eq!(STAMP_HZ, 16_000_000);
        assert_eq!(STAMP_HZ, crate::airtime::MAC_TICKS_PER_US * 1_000_000);
    }
}
