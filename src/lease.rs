//! **The named airtime lease, on the bearer side** — the grid arithmetic a driver needs to confine
//! its transmissions to time windows derived from NDN names, with **no association, no AP, no host
//! identity, no AID**.
//!
//! A name owns a slot: `slot = prefix_hash(name) % slots`. Every node holding the name computes the
//! same answer from the name and a shared clock, so the grant is *computed*, never negotiated and
//! never announced. There is no coordinator to associate with, nothing to be assigned, and nothing
//! about a host anywhere in the derivation — which is the whole reason this shape was chosen over
//! RAW (whose AID 0 means "does not apply, allow") and TWT (which presupposes association).
//!
//! # The hash is not a new one
//!
//! [`prefix_hash`](ndn_frame_io::prefix_hash) is the control plane's canonical name key (#44) —
//! `SlotSchedule::owner_slot`, `HopSchedule`, demand, the sense bus and the consistency digest all key
//! on it. It moved down into `ndn-frame-io` so a driver and the control plane share one
//! implementation instead of two copies free to drift; see `ndn_frame_io::keyspace`. This module adds
//! **no** hash of its own.
//!
//! # This is the measured floor, not an ideal one
//!
//! [`LeaseGrid`] is the geometry [`SlotSchedule`-style ownership] is actuated on, sized from what was
//! MEASURED on this bearer (MM6108 → MM6108, host-scheduled injection, no association):
//!
//! | measurement | value | what it fixes |
//! |---|---|---|
//! | placement error, host-scheduled | p50 88 µs, p90 261, p99 453, **p99.9 896** | the guard |
//! | commanded placement (R vs jittered control) | 0.9930 vs 0.095 | that the grid is real |
//! | CW forced to 0 | no change | backoff is not the dominant error |
//! | minimum useful slot | **2 ms** | [`MIN_SLOT_US`] |
//!
//! So [`LeaseGrid::new`] **refuses** a slot narrower than [`MIN_SLOT_US`] and a guard narrower than
//! [`MIN_GUARD_US`], rather than accepting a geometry the placement floor cannot hold. A schedule
//! whose slot is thinner than its own jitter does not divide the medium — it just relabels
//! collisions, and every node involved reports success.
//!
//! ⚠ **The guard is trailing, and that is deliberate.** The measured error is *lateness*: a frame
//! aimed at the slot boundary lands 88–896 µs after it, inside the slot. So the usable window opens
//! at the boundary and closes [`guard_us`](LeaseGrid::guard_us) early, and the last frame of a burst
//! must be *targeted* early enough that its own p99.9 lateness plus its airtime still fit
//! ([`fits_at`](LeaseGrid::fits_at)). Under-running the guard does not cost us anything we can see —
//! it costs the *next* name's turn, which is why it has to be checked here rather than noticed later.
//!
//! # What this module is not
//!
//! It is the geometry only: ownership, boundaries, the usable window, and whether a frame fits. It
//! deliberately does **not** reimplement the control plane's claimable-slot election, reserved
//! latency lanes, lease length `L`, or CCLF — those live in `ndn_radio_cognition::SlotSchedule` /
//! `mac::coop` and belong to the decision layer. This is what the driver needs to *place a frame*.

use ndn_frame_io::{prefix_hash, prefix_hash_slash, slash_components};

/// **The narrowest slot this bearer can actually own**, µs.
///
/// MEASURED, not chosen: host-scheduled injection places a frame to p99.9 = 896 µs, and a 1 ms
/// trailing guard is what contains that. A slot has to hold at least one frame's placement *plus*
/// that guard, so 2 ms is the floor at which the schedule still means something. Below it the guard
/// is either larger than the slot or too small to contain the jitter, and neighbouring names bleed
/// into each other while every node reports a clean send.
pub const MIN_SLOT_US: u64 = 2_000;

/// **The narrowest trailing guard**, µs — the 1 ms that contains the measured p99.9 placement error.
pub const MIN_GUARD_US: u64 = 1_000;

/// Why a [`LeaseGrid`] was refused. Every variant names the measurement it violates, because the
/// only useful thing to tell a caller who asked for a 200 µs slot is *what* says no.
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum GridError {
    /// `slot_us` below [`MIN_SLOT_US`].
    SlotBelowFloor { slot_us: u64 },
    /// `guard_us` below [`MIN_GUARD_US`].
    GuardBelowFloor { guard_us: u64 },
    /// The guard leaves no usable window (`guard_us >= slot_us`).
    GuardSwallowsSlot { slot_us: u64, guard_us: u64 },
    /// A superframe of zero (or one) slot is not a schedule.
    TooFewSlots { slots: u64 },
}

impl std::fmt::Display for GridError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::SlotBelowFloor { slot_us } => write!(
                f,
                "slot of {slot_us} us is below this bearer's MEASURED floor (guard + one frame's \
                 channel cost; {MIN_SLOT_US} us when no frame cost is supplied): host-scheduled \
                 injection on this bearer places a frame to p99.9 = 896 us, so a slot this narrow \
                 cannot contain its own placement jitter — it would relabel collisions, not prevent \
                 them"
            ),
            Self::GuardBelowFloor { guard_us } => write!(
                f,
                "guard of {guard_us} us is below the MEASURED {MIN_GUARD_US} us floor: the p99.9 \
                 placement error is 896 us and the guard is what keeps it out of the NEXT name's slot"
            ),
            Self::GuardSwallowsSlot { slot_us, guard_us } => write!(
                f,
                "a {guard_us} us guard leaves no usable window in a {slot_us} us slot — the guard \
                 exists to contain the p99.9 = 896 us placement error, so it cannot be shrunk to \
                 fit; widen the slot instead"
            ),
            Self::TooFewSlots { slots } => write!(
                f,
                "{slots} slot(s) is not a superframe: with fewer than 2 slots every name owns all \
                 the airtime and the schedule divides nothing"
            ),
        }
    }
}

impl std::error::Error for GridError {}

/// The slot geometry a name's lease is placed on: `slots` slots of `slot_us` each, the last
/// `guard_us` of every slot reserved as the trailing guard.
///
/// Ownership arithmetic is bit-identical to `ndn_radio_cognition::SlotSchedule` at its default
/// (`reserved_stride = 0`) — `owner_slot`, `epoch`, `current_slot`, `slot_start_us`,
/// `slot_remaining_us`, `wait_us` and `superframe_us` are the same expressions, and the golden
/// vectors in [`tests`] were taken from that type by running it. What this adds over it is the
/// **guard**, which the control-plane type folds into `slot_us` and the bearer has to see separately
/// in order to know when to stop transmitting inside a slot it owns.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct LeaseGrid {
    slot_us: u64,
    slots: u64,
    guard_us: u64,
}

impl LeaseGrid {
    /// Build a grid, refusing any geometry the measured placement floor cannot hold.
    ///
    /// There is deliberately no unchecked constructor. The one thing a caller could want it for is
    /// to demonstrate that a sub-floor slot fails — and that demonstration is the *refusal*.
    pub fn new(slot_us: u64, slots: u64, guard_us: u64) -> Result<Self, GridError> {
        Self::new_for_frame(
            slot_us,
            slots,
            guard_us,
            MIN_SLOT_US.saturating_sub(MIN_GUARD_US),
        )
    }

    /// The same grid, with the slot floor **derived from the frame this bearer actually sends**
    /// rather than from the flat [`MIN_SLOT_US`].
    ///
    /// [`MIN_SLOT_US`] is 2 ms because that is what a 1 MHz / MCS0 frame plus its guard needed, and
    /// while a frame cost 7 ms the constant never bound anything. At 8 MHz / MCS7 one 200-byte frame
    /// is a MEASURED 710 µs, so `guard + frame` is 1710 µs and the flat 2 ms floor starts refusing
    /// geometries that are perfectly sound — it would have cost 15 % of the rate ceiling for a reason
    /// that expired when the PHY got faster.
    ///
    /// The *principle* the floor encodes is unchanged and is what is enforced here: **a slot must
    /// hold one frame plus the guard that contains that frame's placement jitter.** Pass
    /// `frame_cost_us` as the measured channel cost of one frame (airtime + MAC turnaround +
    /// backoff), not the bare airtime — the slot has to contain all of it.
    pub fn new_for_frame(
        slot_us: u64,
        slots: u64,
        guard_us: u64,
        frame_cost_us: u64,
    ) -> Result<Self, GridError> {
        if slots < 2 {
            return Err(GridError::TooFewSlots { slots });
        }
        if guard_us < MIN_GUARD_US {
            return Err(GridError::GuardBelowFloor { guard_us });
        }
        // `GuardSwallowsSlot` is checked FIRST because it is the more specific diagnosis: a guard
        // wider than its own slot also fails the derived floor below, and reporting that instead
        // would tell the caller to widen the slot when the actual mistake is the guard.
        if guard_us >= slot_us {
            return Err(GridError::GuardSwallowsSlot { slot_us, guard_us });
        }
        // The derived floor: one frame, plus the guard that contains its placement jitter.
        if slot_us < guard_us.saturating_add(frame_cost_us.max(1)) {
            return Err(GridError::SlotBelowFloor { slot_us });
        }
        Ok(Self {
            slot_us,
            slots,
            guard_us,
        })
    }

    /// **Bench characterisation only** — build a grid with both floors bypassed.
    ///
    /// [`MIN_GUARD_US`] and the derived slot floor exist because a slot thinner than its own
    /// placement jitter relabels collisions instead of preventing them. That is exactly why a
    /// *measurement* of where the floor really sits has to be able to cross it: the only way to
    /// show that a 60 µs guard is too small on this bearer is to run one and watch confinement
    /// break. Nothing that ships may call this — production geometry comes from
    /// [`LeaseGrid::new_for_frame`], which says no.
    #[doc(hidden)]
    pub fn for_measurement(slot_us: u64, slots: u64, guard_us: u64) -> Result<Self, GridError> {
        if slots < 2 {
            return Err(GridError::TooFewSlots { slots });
        }
        if guard_us >= slot_us {
            return Err(GridError::GuardSwallowsSlot { slot_us, guard_us });
        }
        Ok(Self {
            slot_us,
            slots,
            guard_us,
        })
    }

    /// The width of one slot, µs.
    pub fn slot_us(&self) -> u64 {
        self.slot_us
    }
    /// Slots per superframe.
    pub fn slots(&self) -> u64 {
        self.slots
    }
    /// The trailing guard, µs.
    pub fn guard_us(&self) -> u64 {
        self.guard_us
    }
    /// The superframe length, µs — the worst-case wait between a name's turns.
    pub fn superframe_us(&self) -> u64 {
        self.slot_us * self.slots
    }
    /// The transmittable window inside an owned slot, µs (`slot_us − guard_us`).
    pub fn usable_us(&self) -> u64 {
        self.slot_us - self.guard_us
    }

    /// **Which slot this name owns** — `prefix_hash % slots`, the control plane's `owner_slot`.
    pub fn owner_slot(&self, prefix_hash: u64) -> u64 {
        prefix_hash % self.slots
    }

    /// The slot a `/`-joined NDN name owns. The one call the actuator makes.
    pub fn owner_slot_of_name(&self, name: &[u8]) -> u64 {
        self.owner_slot(prefix_hash_slash(name))
    }

    /// The common-view epoch (slot index since the clock origin) at `now_us`.
    pub fn epoch(&self, now_us: u64) -> u64 {
        now_us / self.slot_us
    }

    /// The slot index the superframe is in at `now_us`.
    pub fn current_slot(&self, now_us: u64) -> u64 {
        self.epoch(now_us) % self.slots
    }

    /// The start of the slot live at `now_us`.
    pub fn slot_start_us(&self, now_us: u64) -> u64 {
        self.epoch(now_us) * self.slot_us
    }

    /// How far into its slot `now_us` sits.
    pub fn phase_in_slot_us(&self, now_us: u64) -> u64 {
        now_us % self.slot_us
    }

    /// Microseconds left in the slot live at `now_us` (guard included).
    pub fn slot_remaining_us(&self, now_us: u64) -> u64 {
        self.slot_us - (now_us % self.slot_us)
    }

    /// Whether this name owns the slot live at `now_us`.
    pub fn owns_now(&self, prefix_hash: u64, now_us: u64) -> bool {
        self.owner_slot(prefix_hash) == self.current_slot(now_us)
    }

    /// **The verdict a run is scored on**: an instant is *inside the lease* only if the name owns the
    /// slot **and** the instant is before the trailing guard.
    ///
    /// `owns_now` alone is the check that looks right and lets a frame launched 100 µs before the
    /// boundary radiate into the next name's turn. The guard is the difference between a slot MAC
    /// and a slot-shaped hope.
    pub fn inside_lease(&self, prefix_hash: u64, at_us: u64) -> bool {
        self.owns_now(prefix_hash, at_us) && self.phase_in_slot_us(at_us) < self.usable_us()
    }

    /// Does a frame of `airtime_us` launched at `at_us` finish before the guard closes?
    ///
    /// Airtime is charged from the *actual* launch, so a caller must pass the instant it expects the
    /// frame to leave — i.e. the target plus the placement error it is budgeting for — not the
    /// target. Charging it from the target is the arithmetic that quietly spends the guard.
    pub fn fits_at(&self, prefix_hash: u64, at_us: u64, airtime_us: u64) -> bool {
        self.inside_lease(prefix_hash, at_us)
            && self.phase_in_slot_us(at_us) + airtime_us <= self.usable_us()
    }

    /// The weaker, purely physical question: does a frame launched at `at_us` **finish before the
    /// next slot begins**?
    ///
    /// [`fits_at`](Self::fits_at) is the MAC's rule and is deliberately stronger — it also keeps the
    /// frame out of the trailing guard, so the guard stays dead air for the next owner. The two
    /// coincide while a slot is much wider than a frame and diverge sharply once it is not: at
    /// 8 MHz / MCS7 the sound geometry is `slot = airtime + guard`, where every frame finishes a
    /// clear guard ahead of the next slot and yet **none** of them satisfy `fits_at`, because that
    /// predicate charges the guard a second time as dead air after the frame.
    ///
    /// Report both. `fits_at` says whether the MAC's own rule held; this says whether anyone could
    /// actually have been stepped on. A run where the two disagree is not a broken lease — it is a
    /// geometry whose guard has stopped being free, which is the whole story at a fast PHY.
    pub fn ends_inside_slot(&self, prefix_hash: u64, at_us: u64, airtime_us: u64) -> bool {
        self.owns_now(prefix_hash, at_us)
            && self.phase_in_slot_us(at_us) + airtime_us <= self.slot_us
    }

    /// Microseconds until this name's next owned slot **starts**; `0` when it owns the current one.
    pub fn wait_us(&self, prefix_hash: u64, now_us: u64) -> u64 {
        let cur_epoch = self.epoch(now_us);
        let cur_slot = cur_epoch % self.slots;
        let owner = self.owner_slot(prefix_hash);
        if owner == cur_slot {
            return 0;
        }
        let ahead = (owner + self.slots - cur_slot) % self.slots;
        (cur_epoch + ahead) * self.slot_us - now_us
    }

    /// **The absolute start of this name's first owned slot at or after `not_before_us`** — the
    /// target generator the scheduler iterates.
    ///
    /// `not_before_us` should already carry the caller's lead-in (the time it needs to build the
    /// frame and get into `clock_nanosleep`), because a target that is already in the past is not
    /// late — it is unscheduled, and it lands wherever the syscall happens to return, which then
    /// reads as jitter that never happened.
    pub fn next_slot_start_us(&self, prefix_hash: u64, not_before_us: u64) -> u64 {
        let owner = self.owner_slot(prefix_hash);
        // Round UP to a slot boundary first: if `not_before_us` is mid-slot, even our own, that
        // slot's start is behind us and unusable as a target.
        let next_epoch = self.epoch(not_before_us) + 1;
        let cur_slot = next_epoch % self.slots;
        let ahead = (owner + self.slots - cur_slot) % self.slots;
        (next_epoch + ahead) * self.slot_us
    }

    /// The slot-start following `slot_start_us` in which this name transmits again — one superframe
    /// on. Cheaper than re-deriving, and it cannot drift off the grid.
    pub fn following_slot_start_us(&self, slot_start_us: u64) -> u64 {
        slot_start_us + self.superframe_us()
    }
}

/// [`prefix_hash`] of a `/`-joined NDN name — re-exported at the point of use so an actuator does
/// not have to know which crate the keyspace lives in, only that there is exactly one.
pub fn name_key(name: &[u8]) -> u64 {
    prefix_hash_slash(name)
}

/// The name's components, as the keyspace splits them. For printing what a name resolved to, so a
/// run header shows the *derivation* and not just its result.
pub fn name_components(name: &[u8]) -> Vec<&[u8]> {
    slash_components(name)
}

/// **CLOCK_REALTIME in microseconds** — the shared grid two nodes fold on.
///
/// The slot map is only collision-free if the nodes agree on `epoch(t)`, so the clock has to be one
/// both can read. `CLOCK_MONOTONIC` (which the placement floor was measured on) cannot be: its origin
/// is per-boot, so two nodes would compute two different grids from the same name and the schedule
/// would be a private fiction on each. `CLOCK_REALTIME` is the cheapest clock that is *shared*, and
/// its disagreement between nodes is therefore a term in the guard budget — the reason a 2 ms slot
/// wants NTP-class agreement and a µs slot would want the hardware TSF common view (#41), which this
/// grid does not yet ride.
#[cfg(unix)]
pub fn wall_us() -> u64 {
    let mut ts: libc::timespec = unsafe { std::mem::zeroed() };
    // SAFETY: `ts` is a live local of exactly the type the call writes.
    unsafe { libc::clock_gettime(libc::CLOCK_REALTIME, &mut ts) };
    ts.tv_sec as u64 * 1_000_000 + ts.tv_nsec as u64 / 1_000
}

/// Re-export so an actuator can hash pre-split components without a second import.
pub fn key_of_components(components: &[&[u8]]) -> u64 {
    prefix_hash(components)
}

#[cfg(test)]
mod tests {
    use super::*;

    const SLOT: u64 = 2_000;
    const SLOTS: u64 = 8;
    const GUARD: u64 = 1_000;

    fn grid() -> LeaseGrid {
        LeaseGrid::new(SLOT, SLOTS, GUARD).expect("the measured floor geometry must be accepted")
    }

    /// ★ **Golden vectors from `ndn_radio_cognition::SlotSchedule::owner_slot(prefix_hash(name))`**,
    /// produced by running that code (2026-09-01) at `SlotSchedule::new(2000, 8)`.
    ///
    /// This is the test that makes the bearer-side grid trustworthy: it does not check that the slot
    /// map is *self*-consistent (any hash is), it checks that it is the map the control plane and
    /// every other node compute. A driver that slots names differently from the decider is worse
    /// than one that does not slot at all — it transmits confidently into someone else's turn.
    const GOLDEN_SLOTS: &[(&str, u64)] = &[
        ("/ndn/lease/a", 7),
        ("/ndn/lease/b", 0),
        ("/ndn/lease/node-a", 6),
        ("/ndn/lease/node-b", 1),
        ("/", 5),
        ("/ndn", 4),
        ("/ndn/test/halow", 5),
        ("/ndn/mds/o5p-1/telemetry", 4),
    ];

    #[test]
    fn slots_agree_with_the_control_planes_slot_schedule() {
        let g = grid();
        for (name, want) in GOLDEN_SLOTS {
            assert_eq!(
                g.owner_slot_of_name(name.as_bytes()),
                *want,
                "{name} owns a different slot here than SlotSchedule::owner_slot gives it"
            );
        }
    }

    /// The two properties the whole design rests on, stated as a test rather than as prose: the same
    /// name always lands in the same slot, and different names land in different slots.
    #[test]
    fn the_same_name_always_lands_in_the_same_slot() {
        let g = grid();
        let n = b"/ndn/mds/o5p-2/telemetry";
        let first = g.owner_slot_of_name(n);
        for _ in 0..1000 {
            assert_eq!(g.owner_slot_of_name(n), first);
        }
        // …and independently of when it is asked: ownership carries no clock term. The clock only
        // says which slot is CURRENT, never which slot is OURS.
        for t in [0u64, 1, 999_999, 1_777_000_000_000, u64::MAX / 2] {
            assert_eq!(g.owner_slot_of_name(n), first, "at t={t}");
            let _ = g.current_slot(t);
        }
    }

    #[test]
    fn different_names_land_in_different_slots() {
        let g = grid();
        let names: [&[u8]; 4] = [
            b"/ndn/lease/a",
            b"/ndn/lease/b",
            b"/ndn/lease/node-a",
            b"/ndn/lease/node-b",
        ];
        let slots: Vec<u64> = names.iter().map(|n| g.owner_slot_of_name(n)).collect();
        for i in 0..slots.len() {
            for j in i + 1..slots.len() {
                assert_ne!(
                    slots[i],
                    slots[j],
                    "{:?} and {:?} collided on slot {}",
                    std::str::from_utf8(names[i]),
                    std::str::from_utf8(names[j]),
                    slots[i]
                );
            }
        }
    }

    /// The floor is enforced, and the refusal says which measurement says no. A geometry the
    /// placement error cannot hold has to be a hard error: accepted, it produces a run that looks
    /// like a schedule and collides like none.
    #[test]
    fn a_slot_below_the_measured_floor_is_refused() {
        assert_eq!(
            LeaseGrid::new(MIN_SLOT_US - 1, 8, 1_000).unwrap_err(),
            GridError::SlotBelowFloor {
                slot_us: MIN_SLOT_US - 1
            }
        );
        let msg = LeaseGrid::new(200, 8, 1_000).unwrap_err().to_string();
        assert!(
            msg.contains("896"),
            "the refusal must cite the measurement: {msg}"
        );
        assert_eq!(
            LeaseGrid::new(4_000, 8, MIN_GUARD_US - 1).unwrap_err(),
            GridError::GuardBelowFloor {
                guard_us: MIN_GUARD_US - 1
            }
        );
        assert!(matches!(
            LeaseGrid::new(2_000, 8, 2_000).unwrap_err(),
            GridError::GuardSwallowsSlot { .. }
        ));
        assert_eq!(
            LeaseGrid::new(2_000, 1, 1_000).unwrap_err(),
            GridError::TooFewSlots { slots: 1 }
        );
    }

    /// Ownership is not enough: the trailing guard has to exclude the last microseconds of an owned
    /// slot, or a frame launched at the boundary lands in the next name's turn.
    #[test]
    fn the_guard_excludes_the_tail_of_a_slot_we_own() {
        let g = grid();
        let h = name_key(b"/ndn/lease/b"); // slot 0 — the superframe's first slot
        assert_eq!(g.owner_slot(h), 0);
        // Slot 0 of superframe 0 is [0, 2000); usable [0, 1000).
        assert!(g.owns_now(h, 0) && g.inside_lease(h, 0));
        assert!(g.inside_lease(h, 999));
        assert!(
            g.owns_now(h, 1_000) && !g.inside_lease(h, 1_000),
            "1000 us into a 2000 us slot with a 1000 us guard is OWNED but not TRANSMITTABLE — that \
             distinction is the whole guard"
        );
        assert!(!g.inside_lease(h, 1_999));
        assert!(
            !g.owns_now(h, 2_000),
            "the next slot belongs to someone else"
        );
    }

    /// A frame must finish inside the usable window, not merely start inside it.
    #[test]
    fn a_frame_that_would_overrun_the_guard_does_not_fit() {
        let g = grid();
        let h = name_key(b"/ndn/lease/b"); // slot 0
        assert!(g.fits_at(h, 0, 1_000), "exactly filling the window fits");
        assert!(
            !g.fits_at(h, 0, 1_001),
            "one microsecond of overrun does not"
        );
        assert!(g.fits_at(h, 500, 500));
        assert!(
            !g.fits_at(h, 900, 500),
            "starting late with a long frame must be refused even though the START is legal"
        );
    }

    /// `wait_us` has to land on the boundary of a slot we own — every time, from every instant.
    /// Off-by-one here is invisible in a single run and shows up as a few percent of frames in the
    /// neighbour's slot.
    #[test]
    fn waiting_always_lands_on_an_owned_boundary() {
        let g = grid();
        for name in ["/ndn/lease/a", "/ndn/lease/b", "/ndn/test/halow"] {
            let h = name_key(name.as_bytes());
            for now in (0u64..g.superframe_us() * 3).step_by(37) {
                let w = g.wait_us(h, now);
                assert!(
                    w < g.superframe_us(),
                    "{name}: wait {w} exceeds a superframe"
                );
                if w == 0 {
                    assert!(g.owns_now(h, now), "{name}: zero wait must mean we own now");
                    continue;
                }
                let t = now + w;
                assert_eq!(g.phase_in_slot_us(t), 0, "{name}: not a slot boundary");
                assert!(g.owns_now(h, t), "{name}: waited into someone else's slot");
                assert!(
                    g.inside_lease(h, t),
                    "{name}: boundary must be transmittable"
                );
            }
        }
    }

    /// The target generator must always be in the future and always on an owned boundary — and
    /// stepping by a superframe must stay on the same slot forever.
    #[test]
    fn the_target_generator_stays_on_the_owned_grid() {
        let g = grid();
        let h = name_key(b"/ndn/mds/o5p-1/telemetry");
        let own = g.owner_slot(h);
        for now in (0u64..g.superframe_us() * 2).step_by(53) {
            let mut t = g.next_slot_start_us(h, now);
            assert!(
                t > now,
                "a target at or before now is not scheduled, it is late"
            );
            for _ in 0..16 {
                assert_eq!(g.phase_in_slot_us(t), 0);
                assert_eq!(g.current_slot(t), own);
                assert!(g.inside_lease(h, t));
                t = g.following_slot_start_us(t);
            }
        }
    }

    /// The bearer grid's boundary arithmetic must be the control plane's, expression for
    /// expression — checked against hand-computed values rather than against itself.
    #[test]
    fn boundary_arithmetic_matches_the_slot_schedule_definitions() {
        let g = grid();
        // t = 5300 us: epoch 2 (slot 2 of the superframe), slot start 4000, 700 us remaining.
        assert_eq!(g.epoch(5_300), 2);
        assert_eq!(g.current_slot(5_300), 2);
        assert_eq!(g.slot_start_us(5_300), 4_000);
        assert_eq!(g.phase_in_slot_us(5_300), 1_300);
        assert_eq!(g.slot_remaining_us(5_300), 700);
        assert_eq!(g.superframe_us(), 16_000);
        assert_eq!(g.usable_us(), 1_000);
        // Superframe 1 starts at 16000, so slot 2 there is epoch 10.
        assert_eq!(g.current_slot(16_000 + 2 * SLOT + 5), 2);
    }

    /// `wall_us` must be a plausible UNIX time in µs, and must advance. A clock that reads zero (or
    /// nanoseconds mislabelled as microseconds) would put the whole fleet on a different grid while
    /// every slot computation stayed internally consistent.
    #[test]
    #[cfg(unix)]
    fn the_shared_clock_reads_a_plausible_unix_microsecond() {
        let a = wall_us();
        // 2026-01-01 .. 2100-01-01 in µs.
        assert!(
            (1_767_225_600_000_000..4_102_444_800_000_000).contains(&a),
            "wall_us() = {a} is not a UNIX time in microseconds"
        );
        std::thread::sleep(std::time::Duration::from_millis(2));
        assert!(wall_us() > a, "the shared clock did not advance");
    }
}
