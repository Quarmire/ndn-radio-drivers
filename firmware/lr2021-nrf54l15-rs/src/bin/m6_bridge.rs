//! **M6 — the host bridge.** Makes this node a peer of the Waveshare and Heltec LoRa nodes.
//!
//! Speaks the 7E-A5 protocol ([`serial`]) over **UART20**, which the XIAO's onboard CMSIS-DAP probe
//! bridges to `/dev/ttyACM0` — so the existing host driver reaches it with no new transport.
//!
//! What parity buys: the rig's five sub-GHz/2.4 GHz nodes (2 LR2021 + 2 Waveshare + 1 Heltec) become
//! one addressable fleet, which is what the N≥3 MAC experiments need — the claimable-slot and
//! hidden-terminal tests (#94/#95) cannot run on a two-node link.
//!
//! ## What this node is, in the fleet's own vocabulary
//!
//! `CMD_GET_CAP` answers `radio_kind = 2` (**the LR2021 part**, whatever mode it is in) and then
//! says the rest in fixed units, so the host never has to hard-code what this node can do:
//!
//! ★ **Modulation is a knob here (7E-A5 v3).** `CMD_SET_PHY` moves the node between **LoRa**, **FLRC**
//! and **LR-FHSS** at runtime and replies with a whole new `EVT_CAP`, because `max_payload`,
//! `sf_min`/`sf_max`, the airtime model and `sched_gran_ns` are all per-PHY. The figures below are
//! the **FLRC** ones, which is what this node boots in; in LoRa the same node carries 247 bytes and
//! has SF7..SF12, and the host is expected to replace its profile wholesale rather than patch it.
//!
//! - **`stamp_hz = 16_000_000`, `stamp_kind = 3`** — the `EVT_RX` `ts` field is the M4 hardware
//!   capture: DPPI-latched at the DIO edge, 62.5 ns per tick, no CPU in the loop. Same wire field
//!   the Waveshare fills with a millisecond software counter; three orders of magnitude better, and
//!   the host now learns that rather than assuming the worst case everywhere.
//! - **`max_payload = 47`** — one fixed 48-byte on-air PDU minus the in-frame length byte. Small,
//!   and true. See `flrc_link::build_frame`.
//! - **`sched_gran_ns = 50_000`, `CMD_TX_AT` implemented** — a *CPU-mediated* schedule: the same
//!   free-running counter the RX stamp uses, waited on by the CPU, then `SetTx` over SPI. Looser than
//!   the DPPI path `m5_tx` demonstrates, because that path needs DIO8 as a trigger INPUT and DIO8 is
//!   the RX stamp's capture source — see the `CMD_TX_AT` arm. The declared 50 µs describes what this
//!   firmware can actually hit, which is the only thing a guard band can be built from.
//! - **no spreading factor in FLRC** (`sf_min = sf_max = 0`); the `sf` slot of
//!   `CMD_SET_MOD`/`EVT_INFO` carries an FLRC bitrate rung instead. In **LoRa** that slot finally
//!   means what the fleet says it means, and `EVT_CAP.phy_current` is what tells the host which.
//!
//! ## Never silence
//!
//! Every command is answered — a `SET_*` with `EVT_INFO`, an unimplemented opcode with
//! `EVT_UNSUPPORTED`. Five commands here used to reply nothing at all, and the host's
//! `exec_idempotent` waits for `EVT_INFO`, retries four times and then fails: a knob that applied
//! perfectly looked like a dead node.
//!
//! The rule extends to **arguments**, not just opcodes. Three fleet commands carry fields this radio
//! has no equivalent for, and each refuses rather than reinterpreting:
//! `CMD_SET_CAD_CFG`'s LoRa correlator thresholds, `CMD_SET_PREAMBLE` outside the 4..32-bit register,
//! and `CMD_SET_SYNC` entirely (one byte cannot become a 32-bit FLRC syncword). A knob that silently
//! means something else is worse than one that is missing — the host believes what it is told.

#![no_std]
#![no_main]

use defmt_rtt as _;
use embassy_executor::Spawner;
use embassy_nrf::buffered_uarte::BufferedUarte;
use embassy_nrf::uarte;
use embassy_time::{Duration, Instant, Timer};
use embedded_io_async::{Read, Write};
use panic_probe as _;

use lr2021::flrc::{FlrcBitrate, FlrcCr};
use lr2021::lrfhss::{LrfhssBw, LrfhssCr};
use lr2021::radio::ExitMode;
use lr2021::status::{Intr, IRQ_MASK_FHSS, IRQ_MASK_LORA_TX_RX_HOP};
use lr2021::system::DioNum;
use lr2021::system::ChipMode;

use lr2021_nrf54l15_rs::hoptrace::{self as hoptrace_mod, HopTrace};
use lr2021_nrf54l15_rs::phy::{self, Phy};
use lr2021_nrf54l15_rs::phy_link::{self, dbm_of, PhyMode, PhyState};
use lr2021_nrf54l15_rs::serial::{self, Parser};
use lr2021_nrf54l15_rs::timing::{RxCapture, TICKS_PER_US};
use lr2021_nrf54l15_rs::{airtime, flrc_link, hw, lora_link, lrfhss_link};

/// The on-device NDN data plane, **shared by path with `waveshare-lora-rs` rather than copied**.
///
/// Filter, dedup and relay decisions must be byte-identical across nodes that interoperate: two
/// implementations that agree today would drift, and the failure mode is a node silently dropping
/// traffic its neighbour forwards — indistinguishable from a link problem.
#[path = "../../../waveshare-lora-rs/src/ndn.rs"]
pub mod ndn;

// ── Sensing / LBT tuning ────────────────────────────────────────────────────────────────────────

/// How often the background sampler looks at the channel while listening.
///
/// 5 ms is a compromise, not a measurement: fast enough that `EVT_SENSE.activity` differenced over a
/// one-second host window has ~200 samples of resolution, slow enough that the extra SPI traffic
/// cannot delay draining an RX FIFO.
const SENSE_PERIOD_MS: u64 = 5;

/// CCA window, in the command's 31.25 ns units. 4096 ≈ **128 µs**, chosen to be about one frame
/// time: a 48-byte frame at FLRC 2.6 Mbit/s is ~148 µs on air, so a shorter window can sit inside
/// the gap between two frames and call a busy channel idle.
///
/// The **base**: `CMD_SET_CAD_CFG`'s `sym` byte scales it `1/2/4/8/16 ×`, which is the same "how
/// long to listen" meaning the LoRa nodes' symbol count has. See [`Sense::cca_steps`].
const CCA_STEPS: u32 = 4096;

/// Largest `CMD_SET_CAD_CFG` `sym` code, matching the LoRa nodes' `cadSymbolNum` space (1/2/4/8/16).
const CAD_SYM_MAX: u8 = 4;

/// How close to the scheduled instant `CMD_TX_AT` stops sleeping and starts polling the MAC clock:
/// 3200 ticks = **200 µs**.
///
/// `Timer::after` is a 1 MHz GRTC compare plus an executor wake, and M4 measured that wake path at
/// 30.9 µs on this board — so a sleep cannot place a transmit to better than tens of microseconds.
/// The last stretch is spun on the same free-running counter `EVT_RX` stamps with. 200 µs is ~6×
/// the measured wake latency: enough that the sleep always lands short of the target, small enough
/// that the spin costs nothing anyone can see.
const SCHED_SPIN_TICKS: u32 = 200 * TICKS_PER_US;

/// Largest `CMD_TX_AT` / `CMD_TX_AT_ABS` horizon this node accepts: **1 s**.
///
/// Two reasons, and the first is the honest one: the scheduled wait is **blocking**. It runs inside
/// the single command loop, so while it waits the node is not draining the UART and not servicing
/// RX. A 60-second schedule would make the node look dead for a minute. One second is already four
/// orders of magnitude above the slot scale the MAC needs (#93 wants microseconds), so the bound
/// costs nothing real.
///
/// The second: the MAC clock is a 32-bit counter at 16 MHz and wraps every ~268 s, so a delay
/// approaching that is genuinely ambiguous — "4 minutes ahead" and "just now" become the same tick.
/// Refused with `REASON_OUT_OF_RANGE` rather than clamped: a frame placed in a slot other than the
/// one the host asked for is worse than a frame not sent.
const MAX_SCHED_DELAY_US: u32 = 1_000_000;

/// [`MAX_SCHED_DELAY_US`] in MAC-clock ticks — the horizon `CMD_TX_AT_ABS` measures a 64-bit target
/// against. Kept as a separate constant because the absolute path compares **ticks to ticks**: the
/// target arrives on the node's own counter, and converting it to microseconds first would throw
/// away the precision the opcode exists to provide.
const MAX_SCHED_DELAY_TICKS: u64 = MAX_SCHED_DELAY_US as u64 * TICKS_PER_US as u64;

/// The host's `AIRTIME_SLACK` (`ndn-radio-drivers/src/lora_serial.rs`), mirrored so the tripwire
/// below can see it. When the host consumes `EVT_TX_STARTED` it *replaces* its reply deadline with
/// `now + airtime + AIRTIME_SLACK`, so a node that announces the transmit at acceptance — as the
/// `CMD_TX_AT` arm does, to keep the UART out of the deadline→key-up window — is only safe while
/// the longest schedule it accepts still fits inside that slack.
const HOST_AIRTIME_SLACK_US: u32 = 2_000_000;

/// If the accepted delay ever grows past the host's slack, announcing at acceptance would make every
/// long schedule time out at the host and the announcement would have to move to a staging point
/// instead (which is why the Heltec, at 60 s, emits from one). Fail the build rather than the link.
const _: () = assert!(MAX_SCHED_DELAY_US < HOST_AIRTIME_SLACK_US);

/// Beacon base period. `CMD_SET_BEACON`'s optional second byte multiplies it (min ×1).
const BEACON_BASE_PERIOD_MS: u64 = 1_000;

/// `CMD_SET_RX_GAIN` 0 → hand the front end back to the AGC. The driver's documented meaning for
/// gain 0: "enable the automatic gain selection (default setting)".
const RX_GAIN_AUTO: u8 = 0;
/// `CMD_SET_RX_GAIN` 1 → the chip's maximum manual gain, the driver's documented ceiling.
///
/// ⚠ This **disables the AGC**. On a two-foot bench link that is a way to overload the front end —
/// the exact failure `flrc_link::apply`'s RxBoost note documents, where a −33 dBm signal into
/// maximum LNA boost demodulated into alternating runs of inverted bytes. `0` (auto) is the right
/// default and what the node boots with; `1` is for a genuinely weak link.
const RX_GAIN_MAX: u8 = 13;

/// Fixed floor of the TxDone watchdog, milliseconds — the part that is not airtime: the SPI round
/// trips, the PA ramp and the poll interval.
///
/// ★ **The watchdog is `2 × airtime + this`, not a constant, and that is a per-PHY correctness
/// requirement rather than tuning.** It was a flat 20 ms, which is three orders of magnitude of
/// slack for a sub-millisecond FLRC frame and **shorter than a single frame** on the other two: LoRa
/// at SF12/125 kHz is seconds, and an LR-FHSS frame is seconds at *any* setting (488.28125 bit/s).
/// A watchdog that expires mid-transmit reports `ok = 0` for a frame that aired perfectly and then
/// re-arms RX underneath it — a failure that looks like a dead link and is not one.
///
/// The 2× factor is deliberate margin over
/// [`airtime::lrfhss_airtime_us`](lr2021_nrf54l15_rs::airtime::lrfhss_airtime_us), whose frame-
/// structure constants are sourced from Semtech's driver and **not verified on this chip**: a model
/// that is wrong by up to 2× degrades into a longer wait rather than into a truncated transmit.
const TX_TIMEOUT_FLOOR_MS: u64 = 20;

/// The TxDone watchdog for the frame about to go out. See [`TX_TIMEOUT_FLOOR_MS`].
fn tx_timeout_ms(st: &PhyState, payload_len: usize) -> u64 {
    let air_ms = (st.airtime_us(payload_len) as u64 + 999) / 1000;
    air_ms.saturating_mul(2).saturating_add(TX_TIMEOUT_FLOOR_MS)
}

/// Upper bound on one LBT backoff draw. Purely a sanity clamp so a host writing a nonsense
/// `cw_ms`/`max_backoff` cannot park the node for hours.
const MAX_BACKOFF_US: u32 = 1_000_000;

/// Carrier-sense / LBT state, all host-tunable so tuning needs a serial command, not a reflash.
struct Sense {
    /// Busy threshold, dBm. **A starting point, not a measurement**: FLRC sensitivity at BR 2600 /
    /// CR 3/4 is about −100.5 dBm, so −90 dBm is ~10 dB of margin above the noise floor. Tune it on
    /// air with `CMD_SET_SENSE_CFG` and `EVT_SENSE`.
    thresh_dbm: i16,
    /// CCA samples per sense, OR'd. More cuts false-negatives at the cost of airtime.
    repeat: u8,
    /// Length of ONE sense, in the chip's 31.25 ns units — `CMD_SET_CAD_CFG`'s only mapped field.
    /// This is the acting knob: [`cca_busy`] passes it straight to `SetCca`.
    cca_steps: u32,
    /// Contention window, **milliseconds on the wire** (the fleet's unit) but drawn in
    /// **microseconds** internally. A 1 ms window would otherwise round to `rand % 1 == 0` — a
    /// deterministic zero backoff, which cannot separate two nodes. The LoRa work measured that
    /// exact failure: deterministic timing at N=2 starves one node.
    lbt_cw_ms: u16,
    lbt_max_backoff: u8,
    lbt_max_attempts: u8,
    /// xorshift32, seeded from the LR2021's hardware RNG XOR the free-running MAC clock. Both terms
    /// differ between the two boards; a constant seed makes both nodes draw the same backoff
    /// sequence, i.e. no backoff at all.
    rng: u32,
    /// FREE-RUNNING, WRAPPING count of channel-busy observations (`EVT_SENSE`). The host differences
    /// two reads; it is never an absolute occupancy. Wrapping, not saturating — a stuck counter
    /// reads as a permanently idle channel.
    activity: u16,
    /// Senses that came back busy during an LBT attempt.
    cad_busy: u16,
    /// Transmissions abandoned after `lbt_max_attempts`.
    defer: u16,
    /// Most recent **idle-channel** energy sample, dBm. `i16::MIN` = never sampled.
    ///
    /// This is the reference for the `EVT_RX` SNR field: FLRC has no SNR register on this chip
    /// (`GetFlrcPacketStatus` returns `pkt_len`, `rssi_avg`, `rssi_sync`, `sw_num` and nothing else,
    /// unlike LoRa), so the honest quantity available is `packet RSSI − noise floor`, both measured
    /// through the same front end. Reported as 0 until a floor sample exists.
    noise_floor_dbm: i16,
}

impl Sense {
    fn new(seed: u32) -> Self {
        Self {
            thresh_dbm: -90,
            repeat: 1,
            cca_steps: CCA_STEPS,
            lbt_cw_ms: 1,
            lbt_max_backoff: 4,
            lbt_max_attempts: 6,
            rng: if seed == 0 { 0x9E37_79B9 } else { seed },
            activity: 0,
            cad_busy: 0,
            defer: 0,
            noise_floor_dbm: i16::MIN,
        }
    }

    fn next_rand(&mut self) -> u32 {
        let mut x = self.rng;
        x ^= x << 13;
        x ^= x >> 17;
        x ^= x << 5;
        self.rng = x;
        x
    }

    /// SNR to report for a packet at `rssi_dbm`. 0 means "not known yet", never a guess.
    fn snr_for(&self, rssi_dbm: i16) -> i16 {
        if self.noise_floor_dbm == i16::MIN {
            0
        } else {
            rssi_dbm.saturating_sub(self.noise_floor_dbm)
        }
    }
}

// ── Radio helpers ───────────────────────────────────────────────────────────────────────────────

/// One energy-detect CCA. Leaves the chip in FS — the caller re-arms RX.
///
/// This is the only real carrier sense on a sub-GHz part in this tree: the LR2021's `SetCca`
/// measures channel energy over a window and reports min/max/avg RSSI. It is **not** LoRa CAD
/// (which correlates against a LoRa preamble and does not exist in FLRC mode), which is why
/// `CMD_SET_CAD_CFG` is refused while `CMD_SET_SENSE_CFG` is honoured.
///
/// A failed sense returns `false`. Blocking a transmit because the *instrument* failed would be the
/// worst of both worlds — it neither protects the channel nor delivers the frame.
async fn cca_busy(radio: &mut hw::Radio, s: &Sense) -> bool {
    // §: "Chip must be standby or FS before issuing the command."
    let _ = radio.set_chip_mode(ChipMode::Fs).await;
    match radio.set_and_get_cca(s.cca_steps, None).await {
        Ok(r) => dbm_of(r.rssi_max()) > s.thresh_dbm,
        Err(_) => false,
    }
}

/// **The DIO8 interrupt mask**, as a function of whether the hop trace is armed.
///
/// One helper rather than five literals, because DIO8 is not merely an interrupt line on this board
/// — it is the capture source of the hardware RX timestamp, and every `SetPacketType` re-issues the
/// routing. Five copies of a mask is how one of them ends up missing a bit after a PHY switch, and
/// nothing says so: the stamps simply stop.
///
/// `Intr::new_txrx()` is `RxDone | TxDone | Timeout`, the baseline. When a hop plan is live the two
/// interrupts the part documents for intra-packet hopping are added:
///
/// | bit | vendor crate | datasheet wording |
/// |---|---|---|
/// | `0x0000_1000` | `Intr::lora_tx_rx_hop()` | "IRq for LoRa intra-packet hopping" — no phase stated |
/// | `0x0200_0000` | `Intr::fhss()` | "IRQ after each ramp-up for intra-packet hopping" |
///
/// **Both are enabled and neither is presumed.** Which one the silicon actually raises (or whether
/// it raises both, at two different instants) is a measurement, not a reading of two doc strings —
/// and `CMD_SET_DEBUG` reports the raw pair so one run settles it. Enabling both cannot double-count:
/// [`hop_irq`] folds them into one event per IRQ poll.
///
/// ⚠ Added **only while hopping is enabled**. With hopping off — every measurement taken on this
/// board to date — the mask is bit-identical to what it has always been, and so is the RX stamp.
fn dio_irq_mask(hop_trace_armed: bool) -> Intr {
    if hop_trace_armed {
        Intr::new(Intr::new_txrx().value() | IRQ_MASK_LORA_TX_RX_HOP | IRQ_MASK_FHSS)
    } else {
        Intr::new_txrx()
    }
}

/// Did this interrupt status carry a hop event? See [`dio_irq_mask`] for why both bits count as one.
fn hop_irq(irq: &Intr) -> bool {
    irq.lora_tx_rx_hop() || irq.fhss()
}

/// **Fold one `get_and_clear_irq` result into the hop trace**, given the capture register read that
/// went with it.
///
/// `stamp` must be `cap.hw_stamp()` read **once** for this status word, because `CC[0]` holds the
/// instant of the *first* DIO8 rising edge since the previous clear — see [`RxCapture`]. Reading it
/// twice would not give two events; it would give the same value with a race in between.
///
/// The coalesced case is counted rather than hidden: when a hop and an `RxDone` land in one poll
/// window they share the one capture, and `CC[0]` is **whichever came first** — the hop inside a
/// packet, but the frame when a hop follows an `RxDone` before the next poll. Such an entry is
/// AMBIGUOUS and the host must drop it; both halves of the detection are on the wire, because the
/// count says how often it happened and the offending `EVT_RX.ts` reappears verbatim as a `t_ticks`
/// in the ring. Bounded by one hop period (8.192 ms at SF7/8 symbols), and avoided entirely by
/// taking the timeline on a node that is not also receiving.
fn note_irq(tr: &mut HopTrace, irq: &Intr, stamp: u32) {
    // The decision itself is host-tested in `HopTrace::note`; this function is only the translation
    // from the chip's status word to the two booleans it takes.
    tr.note(hop_irq(irq), irq.rx_done(), stamp);
}

/// Poll for TxDone. Returns whether the frame actually went out.
///
/// `get_and_clear_irq` clears **every** interrupt, so a frame that arrived in the microseconds
/// before this transmit can have its RxDone swallowed here. That is the right trade on a
/// half-duplex radio — the alternative is aborting our own transmission — and it is bounded by
/// `TX_TIMEOUT_MS`, but it is a real loss and not a free one.
///
/// The old bridge reported `ok` from "the SPI commands returned Ok" and re-armed RX immediately
/// afterwards — which races the transmission it just started, and only worked because the UART
/// write of the reply happened to take longer than a frame. Waiting for the chip's own TxDone makes
/// `EVT_TXDONE.ok` mean what the host reads it as: the frame aired.
///
/// ⚠ **This blocks the command loop, and on the slow PHYs that is seconds, not milliseconds.** The
/// node is not draining the UART while it waits: a 512-byte RX ring at 115200 8N1 holds ~44 ms of
/// host traffic, so a host that keeps sending commands during an LR-FHSS transmit (~3.4 s) will lose
/// some. The same is true of `CMD_TX_AT`'s wait and has been since v2 — it is bounded there by
/// `MAX_SCHED_DELAY_US`, and here by the airtime the host was just told in `EVT_TX_STARTED`. A host
/// that respects that deadline never hits it; one that pipelines into a slow PHY will.
///
/// ★ It also **feeds the hop trace**, and that is not incidental: intra-packet hopping happens
/// *during* this wait, so the transmitting node's own hop timeline exists only here. A version of
/// this loop that swallowed the hop interrupts without recording them would leave the TX side of the
/// comparison blank while looking perfectly correct. The 200 µs poll is 40× finer than an 8.192 ms
/// hop period, and the stamp itself is the DPPI capture, so the poll rate bounds *which* events are
/// separated, never the accuracy of the ones that are.
async fn wait_tx_done(
    radio: &mut hw::Radio,
    cap: &RxCapture,
    tr: &mut HopTrace,
    timeout_ms: u64,
) -> bool {
    let deadline = Instant::now() + Duration::from_millis(timeout_ms);
    while Instant::now() < deadline {
        if let Ok(irq) = radio.get_and_clear_irq().await {
            // One capture read per status word — see `note_irq`.
            note_irq(tr, &irq, cap.hw_stamp().ticks);
            if irq.tx_done() {
                return true;
            }
        }
        Timer::after(Duration::from_micros(200)).await;
    }
    false
}

/// Transmit one payload as a fixed on-air frame. Leaves the chip out of RX; the caller re-arms.
///
/// Both #108 fixes are applied here, on the same code path `m106_shadow_tx` uses and which this
/// bridge was missing entirely:
///
/// * **whitening** (inside `build_frame`) — the payload and its zero padding are otherwise
///   transition-starved and the GMSK demodulator slips polarity mid-frame;
/// * **`settle_before_tx`** — measured with a B210: going straight from standby to TX leaves the
///   PLL converging under the first 27 kHz of the payload.
async fn tx_once(
    radio: &mut hw::Radio,
    st: &PhyState,
    cap: &RxCapture,
    tr: &mut HopTrace,
    payload: &[u8],
) -> bool {
    phy_link::tx_stage(radio, st, payload).await
        && tx_fire(radio, cap, tr, tx_timeout_ms(st, payload.len())).await
}

/// Staging a transmit is per-PHY and lives in
/// [`phy_link::tx_stage`](lr2021_nrf54l15_rs::phy_link::tx_stage): FLRC writes a fixed whitened PDU
/// into the FIFO, LoRa re-programs `payload_len` and writes the bare payload, LR-FHSS hands the
/// payload to `LrFhssBuildFrame`. It stays split from the key-up for the two reasons that matter to
/// a scheduler and to the host — see that function.

/// Key up the frame [`tx_stage`] loaded, and wait for the chip's own TxDone.
///
/// This one SPI command is the whole of `CMD_TX_AT_ABS`'s residual latency, and it is why
/// `EVT_CAP.sched_gran_ns` is [`airtime::SCHED_GRAN_NS`] and not a timer tick.
///
/// **The hop index at key-up is recorded here** (`EVT_HOPTRACE`, `idx | IDX_TX_KEYED`), so a host can
/// tell where in the hop sequence a frame started. It costs **no SPI**: the index is a firmware
/// counter and the instant is one MCU timer-register read, both taken in the gap before `SetTx`
/// leaves the MCU. Nothing is added to the transmit hot path that could move the frame it is
/// marking — which was the condition on recording it at all. The offset between this software stamp
/// and the real RF key-up (the `SetTx` transaction, ≈5 µs at 8 MHz, plus the chip's PLL/PA ramp) is
/// **not measured**, and is documented on `hoptrace::IDX_TX_KEYED` rather than folded in. The part
/// offers no hardware key-up event to capture instead — `IRQ_MASK_TX_TIMESTAMP` marks the *end* of a
/// transmitted packet.
async fn tx_fire(
    radio: &mut hw::Radio,
    cap: &RxCapture,
    tr: &mut HopTrace,
    timeout_ms: u64,
) -> bool {
    tr.push_tx_keyed(cap.now().ticks);
    if radio.set_tx(0).await.is_err() {
        return false;
    }
    wait_tx_done(radio, cap, tr, timeout_ms).await
}

/// **Re-program the link and go back to listening** — the one supported way to change frequency,
/// power, rate or PHY at runtime.
///
/// Takes the whole [`PhyState`], not just the field that moved, precisely so it cannot half-apply: a
/// retune that re-issued only `set_rf` would silently revert power and rate to the build defaults on
/// every channel change, which is the same class of bug as the one it fixes. After a `SetPacketType`
/// it is not even optional — the modem registers the new mode uses have never been written.
///
/// Errors are swallowed here for the same reason the old `flrc_link::retune` call sites did: the
/// caller answers the host with `EVT_INFO`, whose `status` and `errors` fields carry the chip's own
/// verdict, so a failure is reported rather than turned into silence.
async fn retune(radio: &mut hw::Radio, st: &PhyState) {
    let _ = phy_link::apply(radio, st).await;
    let _ = phy_link::arm_rx(radio, st).await;
}

/// **The 64-bit MAC clock, sampled atomically with its own wrap check.**
///
/// `TIMER20` is 32 bits at 16 MHz and wraps every ~268 s, so `EVT_CLOCK` and `CMD_TX_AT_ABS` both
/// work in a firmware-extended 64-bit count: the LOW 32 bits are exactly the `EVT_RX` `ts` field and
/// the high 32 are the wrap count.
///
/// The wrap check happens **here**, against the same read that is returned, rather than once per
/// loop iteration. The main loop samples every ~1 ms so a wrap cannot be *missed*, but a wrap that
/// happens between the top of the loop and a command arm would pair a fresh low word with a stale
/// high word — and produce a timestamp 268 seconds in the past, exactly once every 4.5 minutes.
/// For `CMD_TX_AT_ABS` that is not a cosmetic error: the target would compare as far-future and be
/// refused, or worse, as far-past and fire immediately.
fn now64(cap: &RxCapture, hi: &mut u32, last: &mut u32) -> u64 {
    let now = cap.now().ticks;
    if now < *last {
        *hi = hi.wrapping_add(1);
    }
    *last = now;
    ((*hi as u64) << 32) | now as u64
}

/// **Wait until the MAC clock reaches `target`** — the CPU-mediated half of `CMD_TX_AT`.
///
/// `target` is a tick count on the **same free-running TIMER20** that `RxCapture` latches `EVT_RX`
/// stamps from and that `CMD_READ_CLOCK` reports, so a host can compute a target from one and hand
/// it to the other with no conversion and no second timebase.
///
/// Two phases: sleep on the executor's timer while the target is far away (so the node is not
/// spinning for milliseconds), then poll the counter directly for the last
/// [`SCHED_SPIN_TICKS`]. The handover exists because `Timer::after` carries the executor wake
/// latency M4 measured at 30.9 µs, and the counter poll does not.
///
/// **A target already in the past returns immediately.** On a 32-bit counter "just passed" and "268
/// seconds ahead" are the same bit pattern, and only the top-half test tells them apart — get it
/// wrong and a late frame waits for the wrap instead of transmitting. `m5_tx` measured exactly that
/// failure: 74 transmits and then silence until the counter came round. `delay_us = 0` lands here
/// too, and means inject now.
async fn wait_until(cap: &RxCapture, target: u32) {
    loop {
        let remaining = target.wrapping_sub(cap.now().ticks);
        if remaining == 0 || remaining > i32::MAX as u32 {
            return; // due, or already past
        }
        if remaining > SCHED_SPIN_TICKS {
            let sleep_us = (remaining - SCHED_SPIN_TICKS) / TICKS_PER_US;
            Timer::after(Duration::from_micros(sleep_us as u64)).await;
        }
        // else: fall through and poll. No `await` in the tail, so nothing can preempt the last
        // 200 µs and turn a scheduled transmit into a software-timed one.
    }
}

/// Atomic listen-before-talk: randomised backoff → sense → key-up, giving up after
/// `lbt_max_attempts`. Returns `(sent, attempts)`.
///
/// The backoff comes **before** the sense, and is randomised, for the reason the LoRa CSMA work
/// measured: a fixed offset makes two nodes take turns, but a third node then collides with
/// whichever it is phase-locked to. Note also what that work found — LBT *hurts* at N=2 on a clean
/// channel. This is here for N≥3, not as a default for every transmit.
async fn lbt_tx<F, Fut>(
    radio: &mut hw::Radio,
    st: &PhyState,
    s: &mut Sense,
    cap: &RxCapture,
    tr: &mut HopTrace,
    payload: &[u8],
    announce: F,
) -> (bool, u8)
where
    F: FnOnce() -> Fut,
    Fut: core::future::Future<Output = ()>,
{
    let mut attempt: u8 = 0;
    let mut sent = false;
    // Called once, at the last moment a transmit can still be abandoned — after the sense says the
    // channel is clear and the FIFO is loaded, before key-up. It is a parameter rather than a call
    // inside this function because the **autonomous** transmits (a Content-Store answer, a relay)
    // must stay silent: an unsolicited `EVT_TX_STARTED` would tell a host its own frame had begun.
    let mut announce = Some(announce);
    while attempt < s.lbt_max_attempts.max(1) {
        let shift = (attempt as u32).min(s.lbt_max_backoff as u32).min(16);
        let window_us = (s.lbt_cw_ms as u32)
            .max(1)
            .saturating_mul(1000)
            .saturating_mul(1u32 << shift)
            .min(MAX_BACKOFF_US);
        let wait_us = s.next_rand() % window_us.max(1);
        Timer::after(Duration::from_micros(wait_us as u64)).await;

        let mut busy = false;
        for _ in 0..s.repeat.max(1) {
            if cca_busy(radio, s).await {
                busy = true;
                break;
            }
        }
        if !busy {
            if phy_link::tx_stage(radio, st, payload).await {
                if let Some(a) = announce.take() {
                    a().await;
                }
                sent = tx_fire(radio, cap, tr, tx_timeout_ms(st, payload.len())).await;
            }
            break;
        }
        s.cad_busy = s.cad_busy.wrapping_add(1);
        s.activity = s.activity.wrapping_add(1);
        attempt += 1;
    }
    if !sent {
        s.defer = s.defer.wrapping_add(1);
    }
    (sent, attempt)
}

/// FLRC bitrate rung ← the chip's own code (`vendor/lr2021/src/cmd/cmd_flrc.rs`).
fn bitrate_of_code(c: u8) -> Option<FlrcBitrate> {
    Some(match c {
        0 => FlrcBitrate::Br2600,
        1 => FlrcBitrate::Br2080,
        2 => FlrcBitrate::Br1300,
        3 => FlrcBitrate::Br1040,
        4 => FlrcBitrate::Br0650,
        5 => FlrcBitrate::Br0520,
        6 => FlrcBitrate::Br0325,
        7 => FlrcBitrate::Br0260,
        _ => return None,
    })
}

/// FLRC coding rate ← the chip's own code. Note `1` is 3/4 and `2` is FEC OFF: counter-intuitive,
/// and it is what the silicon uses, so it is what travels on the wire.
fn cr_of_code(c: u8) -> Option<FlrcCr> {
    Some(match c {
        0 => FlrcCr::Cr12,
        1 => FlrcCr::Cr34,
        2 => FlrcCr::None,
        3 => FlrcCr::Cr23,
        _ => return None,
    })
}

/// LR-FHSS coding rate ← the chip's own code (`LrfhssCr`).
fn lrfhss_cr_of_code(c: u8) -> Option<LrfhssCr> {
    Some(match c {
        0 => LrfhssCr::Cr5p6,
        1 => LrfhssCr::Cr2p3,
        2 => LrfhssCr::Cr1p2,
        3 => LrfhssCr::Cr1p3,
        _ => return None,
    })
}

/// LR-FHSS hopping bandwidth ← the chip's own code (`LrfhssBw`), 0..9.
fn lrfhss_bw_of_code(c: u8) -> Option<LrfhssBw> {
    Some(match c {
        0 => LrfhssBw::Bw39p06,
        1 => LrfhssBw::Bw85p94,
        2 => LrfhssBw::Bw136p72,
        3 => LrfhssBw::Bw183p59,
        4 => LrfhssBw::Bw335p94,
        5 => LrfhssBw::Bw386p72,
        6 => LrfhssBw::Bw722p66,
        7 => LrfhssBw::Bw773p44,
        8 => LrfhssBw::Bw1523p4,
        9 => LrfhssBw::Bw1574p2,
        _ => return None,
    })
}

/// **`CMD_SET_MOD`'s `[a, b, c]` triple, decoded for the CURRENT PHY**, applied to `st` only if
/// every field is valid. Returns false — and leaves `st` untouched — otherwise, so a partially
/// valid triple can never half-configure the modem.
///
/// The byte positions are the fleet's `[sf, bw, cr]`, and what each one *means* is what
/// `EVT_CAP.phy_current` tells the host:
///
/// | | `[0]` | `[1]` | `[2]` |
/// |---|---|---|---|
/// | **LoRa** | spreading factor 7..12 | bandwidth code (both spaces) | coding rate 1..4 = 4/5..4/8 |
/// | **FLRC** | bitrate rung 0..7 | **ignored** — implied by the rung | `FlrcCr` 0..3 |
/// | **LR-FHSS** | must be 0 — no SF, no rung | `LrfhssBw` 0..9 | `LrfhssCr` 0..3 |
///
/// LR-FHSS's `sf` slot is **refused** when non-zero, on the same rule as `CMD_SET_CAD_CFG`'s detector
/// thresholds: a field this PHY has no analogue for is a field the host believes it set. LR-FHSS has
/// neither a spreading factor nor a bitrate ladder — its rate *is* its coding rate, which is why that
/// field carries it.
///
/// FLRC's `bw` slot is **ignored**, deliberately not refused, and the difference is a
/// back-compatibility one rather than a change of principle: v2 documented that slot as "ignored;
/// echoed back as 0" and a shipped host has been told so. Turning a *documented* no-op into an error
/// would break a host that is behaving exactly as specified — which is a different thing from
/// silently reinterpreting a field nobody was warned about.
fn set_mod(st: &mut PhyState, a: u8, b: u8, c: u8) -> bool {
    match &mut st.mode {
        PhyMode::Flrc(p) => match (bitrate_of_code(a), cr_of_code(c)) {
            (Some(br), Some(cr)) => {
                p.bitrate = br;
                p.coding = cr;
                true
            }
            _ => false,
        },
        PhyMode::Lora(p) => match phy::lora_bw_hz_of_code(b) {
            Some(bw_hz) if (phy::LORA_SF_MIN..=phy::LORA_SF_MAX).contains(&a) && (1..=4).contains(&c) => {
                p.sf = a;
                p.bw_hz = bw_hz;
                p.cr = c;
                true
            }
            _ => false,
        },
        PhyMode::LrFhss(p) => match (lrfhss_bw_of_code(b), lrfhss_cr_of_code(c)) {
            (Some(bw), Some(cr)) if a == 0 => {
                p.bw = bw;
                p.cr = cr;
                true
            }
            _ => false,
        },
    }
}

/// The `EVT_INFO` `[sf, bw, cr]` slots for the current PHY — the inverse of [`set_mod`], so a host
/// always reads back the space it wrote in.
fn mod_triple(st: &PhyState) -> [u8; 3] {
    match &st.mode {
        // `bw` = 0: FLRC bandwidth follows the rung, it is not an independent knob.
        PhyMode::Flrc(p) => [p.bitrate as u8, 0, p.coding as u8],
        PhyMode::Lora(p) => [p.sf, phy::lora_bw_code_of_hz(p.bw_hz), p.cr],
        PhyMode::LrFhss(p) => [0, p.bw as u8, p.cr as u8],
    }
}

/// The `EVT_INFO` `sync` slot: the low 16 bits of whatever syncword the current PHY is using.
///
/// Per-PHY because the syncwords are not the same object — FLRC's is a 32-bit correlation sequence,
/// LoRa's is the one-byte SX127x legacy value, LR-FHSS's is a 32-bit detection word — and a host
/// diffing this field across a PHY switch should see it move.
fn sync_word(st: &PhyState) -> u16 {
    match st.mode {
        PhyMode::Flrc(_) => flrc_link::SYNCWORD as u16,
        PhyMode::Lora(_) => lora_link::SYNCWORD as u16,
        PhyMode::LrFhss(_) => lrfhss_link::SYNCWORD as u16,
    }
}

/// **This node's self-description for the PHY it is running right now.**
///
/// Recomputed on every `CMD_SET_PHY` rather than built once at boot, which is the whole point of v3:
/// `max_payload`, `sf_min`/`sf_max` and `sched_gran_ns` are per-PHY, and a cached body would answer
/// `CMD_GET_CAP` with the previous mode's numbers. Everything else here is a property of the board —
/// band, PA range, timestamp — and does not move.
fn caps_for(st: &PhyState) -> serial::Capabilities {
    let p = st.phy();
    serial::Capabilities {
        proto_ver: serial::PROTO_VER,
        // The PART, not the mode. `phy_current` below says what it is running.
        radio_kind: serial::radio_kind::LR2021,
        freq_min_hz: phy::BAND_MIN_HZ,
        freq_max_hz: phy::BAND_MAX_HZ,
        pwr_min_dbm: phy::PWR_MIN_DBM,
        pwr_max_dbm: phy::PWR_MAX_DBM,
        // Not a guess: `timing::TICKS_PER_US` is the constant `TIMER20` is programmed with, and the
        // 16 MHz it implies was confirmed on air (15 frames at ~1 s spacing, 16,616,401..16,625,857
        // ticks apart).
        stamp_hz: TICKS_PER_US * 1_000_000,
        stamp_kind: serial::stamp_kind::HARDWARE_FREE_RUNNING,
        max_payload: phy::max_payload(p) as u16,
        // Per-PHY, like everything else here: `CMD_SET_HOP` is implemented and cannot act in FLRC,
        // and `cmd_bitmap` means "implemented and will act". See `serial::cmd_bitmap_for`.
        cmd_bitmap: serial::cmd_bitmap_for(phy::has_intra_packet_hopping(p)),
        sf_min: phy::sf_min(p),
        sf_max: phy::sf_max(p),
        // **50 µs, and it describes the CPU-mediated `CMD_TX_AT_ABS` path this firmware actually
        // has** — not the 62.5 ns timer tick, and not the DPPI path `m5_tx` shows off with a pin this
        // node cannot spare. `airtime::SCHED_GRAN_NS` carries the arithmetic. The host ANDs this with
        // the `CMD_TX_AT` bit (`NodeProfile::schedules_tx`), so the two move together or the claim is
        // discarded.
        sched_gran_ns: phy::sched_gran_ns(p),
        phy_bitmap: phy::PHY_BITMAP,
        phy_current: p.code(),
    }
}

/// The 19-byte `EVT_INFO` body — see [`serial::EVT_INFO`] for the per-field meaning here, and
/// [`mod_triple`] for what the `sf`/`bw`/`cr` slots carry in each PHY.
async fn info_body(radio: &mut hw::Radio, st: &PhyState, s: &Sense) -> [u8; 19] {
    let status = match radio.get_status().await {
        Ok((chip, _)) => (chip.chip_mode() as u8) | ((chip.cmd() as u8) << 4),
        Err(_) => 0xFF,
    };
    let errors = match radio.get_errors().await {
        Ok(e) => serial::err_bits([
            e.hf_xosc_start(),
            e.lf_xosc_start(),
            e.pll_lock(),
            e.lf_rc_calib(),
            e.hf_rc_calib(),
            e.pll_calib(),
            e.aaf_calib(),
            e.img_calib(),
            e.chip_busy(),
            e.rxfreq_no_fe_cal(),
            e.meas_unit_adc_calib(),
            e.pa_offset_calib(),
            e.ppf_calib(),
            e.src_calib(),
        ]),
        Err(_) => 0,
    };
    let sync = sync_word(st);
    let f = st.freq_hz.to_be_bytes();
    let m = mod_triple(st);
    [
        status,
        (sync >> 8) as u8,
        sync as u8,
        (errors >> 8) as u8,
        errors as u8,
        f[0],
        f[1],
        f[2],
        f[3],
        m[0],
        m[1],
        m[2],
        st.tx_power_dbm() as u8, // dBm ACTUALLY APPLIED, not what the host asked for
        // `lost`: the buffered UARTE ring exposes no overrun count on this MCU. 0, and said so,
        // rather than a number that would be believed.
        0,
        0,
        (s.cad_busy >> 8) as u8,
        s.cad_busy as u8,
        (s.defer >> 8) as u8,
        s.defer as u8,
    ]
}

/// Frame and send one event.
async fn send<W: Write>(uart: &mut W, out: &mut [u8], typ: u8, payload: &[u8]) {
    let k = serial::encode(out, typ, payload);
    let _ = uart.write_all(&out[..k]).await;
}

/// The "announce nothing" argument for [`lbt_tx`] — used by every transmit the host did not ask
/// for, so an autonomous relay cannot be mistaken for the host's own frame going out.
fn quiet() -> core::future::Ready<()> {
    core::future::ready(())
}

/// `prefix` followed by `v` in decimal, into `dst`; returns the byte count written.
///
/// A hand-rolled integer formatter rather than `core::fmt`: a single `write!` into a byte buffer
/// pulls the whole formatting machinery into the image, which is kilobytes of flash on this part for
/// one diagnostic line that is off by default. Truncates rather than panicking if `dst` is short —
/// a log line is never worth faulting a radio node for.
fn fmt_kv(dst: &mut [u8], prefix: &[u8], v: u32) -> usize {
    let mut n = 0;
    for &b in prefix {
        if n < dst.len() {
            dst[n] = b;
            n += 1;
        }
    }
    let mut digits = [0u8; 10];
    let mut i = 0;
    let mut x = v;
    loop {
        digits[i] = b'0' + (x % 10) as u8;
        x /= 10;
        i += 1;
        if x == 0 {
            break;
        }
    }
    while i > 0 {
        i -= 1;
        if n < dst.len() {
            dst[n] = digits[i];
            n += 1;
        }
    }
    n
}

/// One `EVT_LOG` line, but only while `CMD_SET_DEBUG` is on.
///
/// `EVT_LOG` was declared in the wire contract and never emitted; this is what gives it a purpose.
/// Gated because at 115200 baud the UART is shared with `EVT_RX`, and a diagnostic that costs frames
/// is not a diagnostic.
async fn log<W: Write>(uart: &mut W, out: &mut [u8], on: bool, msg: &[u8]) {
    if on {
        send(uart, out, serial::EVT_LOG, msg).await;
    }
}

/// **The LR-FHSS hopping-table write/read-back probe** (`CMD_SET_HOP`, debug on only).
///
/// `WriteLrFhssHoppingTable` (0x59) and `ReadLrFhssHoppingTable` (0x58) are the reason LR-FHSS is
/// interesting for a named-data MAC at all: the hop sequence is a **table**, not a seed the chip
/// expands, so a name-derived dwell schedule is expressible. But a written table is a hypothesis
/// until something reads it back — the vendor crate wraps neither command usefully (the write takes
/// a struct with private fields and no constructor; the read is absent entirely), and the
/// `pkt_length`/`nb_hopping_blocks` arguments are inferred from the frame structure rather than
/// sourced.
///
/// So this writes the current plan immediately and reads it straight back, emitting the first bytes
/// of the response as `EVT_LOG`. Two things it can settle without a reflash:
///
/// * whether the chip **accepts** the write at all (a `CmdErr` shows in the read-back's status);
/// * whether the couples come back in the `(freq u32, nb_symbols u16)` layout the write uses, and in
///   Hz — the `convert_freq` bit is set, so a frequency echoed as a PLL step would be visible
///   immediately.
///
/// Gated on `CMD_SET_DEBUG` because it is two extra SPI transactions and a UART line per hop change,
/// and off by default like every other diagnostic here. The per-frame write in
/// `phy_link::tx_stage` is the one that actually reaches the air; this one is an instrument.
async fn hop_probe<W: Write>(
    radio: &mut hw::Radio,
    uart: &mut W,
    out: &mut [u8],
    st: &PhyState,
    debug: bool,
) {
    if !debug || !matches!(st.mode, PhyMode::LrFhss(_)) {
        return;
    }
    let freqs = st.hop.freqs();
    let PhyMode::LrFhss(p) = &st.mode else { return };
    // A zero-length frame: this is a probe of the TABLE, not a staged transmit, so the length and
    // block count describe nothing that is about to go on air.
    let blocks = airtime::lrfhss_block_count(p.cr as u8, 0);
    if lrfhss_link::write_table(radio, st.hop.enabled, 0, blocks, freqs, st.hop.period)
        .await
        .is_err()
    {
        let chip = phy_link::status_byte(radio);
        let mut m = [0u8; 40];
        let k = fmt_kv(&mut m, b"HOP write REFUSED chip=", chip as u32);
        send(uart, out, serial::EVT_LOG, &m[..k]).await;
        return;
    }
    // One couple is enough to see the layout and the units; more would cost UART a quiet link needs.
    let mut rsp = [0u8; lrfhss_link::read_table_len(1)];
    if lrfhss_link::read_table(radio, &mut rsp).await.is_err() {
        let chip = phy_link::status_byte(radio);
        let mut m = [0u8; 40];
        let k = fmt_kv(&mut m, b"HOP read REFUSED chip=", chip as u32);
        send(uart, out, serial::EVT_LOG, &m[..k]).await;
        return;
    }
    let echoed = u32::from_be_bytes([rsp[2], rsp[3], rsp[4], rsp[5]]);
    let mut m = [0u8; 48];
    let k = fmt_kv(&mut m, b"HOP readback f0=", echoed);
    send(uart, out, serial::EVT_LOG, &m[..k]).await;
}

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    // Crystal, not the RC default — see `hw::init_peripherals` for the measured ~2000 ppm.
    let p = hw::init_peripherals();
    let (mut radio, timing, up) = hw::init(p);

    let mut ucfg = uarte::Config::default();
    ucfg.baudrate = uarte::Baudrate::Baud115200;
    // **Buffered**, not raw. The first version used single-byte `Uarte::read` in the same loop that
    // polls the radio over SPI, and dropped host commands: at 115200 a byte is ~87 µs, and an SPI
    // IRQ poll easily exceeds that, so bytes arriving mid-poll were simply lost (`CMD_GET_INFO`
    // vanished while later commands got through). This is the identical defect that cost the
    // Waveshare firmware ~50% of its commands (task #17) — an interrupt/DMA-backed ring is the fix
    // there and here. Any host-facing serial loop that also drives SPI needs one.
    static mut RXB: [u8; 512] = [0; 512];
    static mut TXB: [u8; 512] = [0; 512];
    let (rxb_, txb_) = unsafe {
        (
            &mut *core::ptr::addr_of_mut!(RXB),
            &mut *core::ptr::addr_of_mut!(TXB),
        )
    };
    let mut uart = BufferedUarte::new(up.uart, up.rx, up.tx, hw::Irqs, ucfg, rxb_, txb_);

    radio.reset().await.unwrap();
    Timer::after(Duration::from_millis(50)).await;
    if let Err(e) = radio.get_version().await {
        defmt::panic!("m6: no radio: {}", defmt::Debug2Format(&e));
    }

    // Boots in **FLRC**, which is what every milestone and every measurement to date ran on, and
    // what the peer LR2021 boots in too — two nodes that disagree about the packet type do not
    // error, they simply never hear each other. `CMD_SET_PHY` moves it from here.
    let mut state = PhyState::default();
    flrc_link::configure_with(&mut radio, &flrc_link::LinkState::default())
        .await
        .expect("FLRC configure");
    let cap = RxCapture::new(timing);

    // **The hop timeline this node takes of itself** (`CMD_GET_HOPTRACE`). Disarmed at boot, so it
    // records nothing and the DIO8 interrupt mask is exactly what it has always been until a host
    // enables hopping. See `lr2021_nrf54l15_rs::hoptrace` for the question it answers and for where
    // in the path the stamp is taken.
    let mut hoptrace = HopTrace::new();

    // Seed the backoff PRNG from two sources that differ between boards: the chip's hardware RNG and
    // the free-running MAC clock at the instant we get here. A deterministic seed would make both
    // nodes draw the SAME backoff sequence, which is not a backoff at all.
    let hw_rng = radio.get_random_number().await.unwrap_or(0);
    let mut sense = Sense::new(hw_rng ^ cap.now().ticks);

    phy_link::arm_rx(&mut radio, &state).await.expect("rx");

    defmt::info!(
        "m6_bridge: up — 7E-A5 v3 on UART20 @115200, phy {=u8} @ {=u32} Hz, {=i8} dBm, max_payload {=usize}, phy_bitmap {=u32:#x}",
        state.phy().code(),
        state.freq_hz,
        state.tx_power_dbm(),
        state.max_payload(),
        phy::PHY_BITMAP
    );

    // This node's self-description. Every field is a source constant or a measurement; nothing here
    // is a placeholder. **Rebuilt on every `CMD_SET_PHY`** — see `caps_for`.
    let mut cap_bytes = caps_for(&state).to_bytes();

    let mut dp = ndn::DataPlane::new();
    let mut parser = Parser::new();
    let (mut n_rx, mut n_tx) = (0u32, 0u32);

    // 32-bit counter → 64-bit clock, extended by `now64`. TIMER20 wraps every ~4.5 minutes at
    // 16 MHz, far shorter than a measurement session, so `EVT_CLOCK` and `CMD_TX_AT_ABS` both work
    // on a firmware-extended count: the LOW 32 bits are exactly the `EVT_RX` ts field, the high 32
    // are the wrap count. The loop samples every iteration (~1 ms), so a wrap cannot be missed.
    let mut clock_hi: u32 = 0;
    let mut clock_last: u32 = cap.now().ticks;

    let mut next_sense = Instant::now();
    let mut out = [0u8; 320];

    // Host-toggleable diagnostics (`CMD_SET_DEBUG`) — off, so a link that is working is quiet and
    // the 115200 UART is not competing with `EVT_RX` for bandwidth it does not have.
    let mut debug = false;

    // Heartbeat beacon (`CMD_SET_BEACON`). **OFF by default**, matching the Waveshare: a node that
    // beacons the moment it is powered would transmit into somebody else's measurement, and this
    // bench runs several nodes in one band.
    let mut beacon_on = false;
    let mut beacon_period_ms = BEACON_BASE_PERIOD_MS;
    let mut beacon_seq: u32 = 0;
    let mut next_beacon = Instant::now() + Duration::from_millis(beacon_period_ms);

    loop {
        // ── clock extension, sampled every pass ────────────────────────────────────────────────
        // One implementation of the wrap rule, in `now64`, which the command arms also call — two
        // copies of "did the counter wrap?" is how one of them ends up off by 2^32.
        let _ = now64(&cap, &mut clock_hi, &mut clock_last);

        // ── host → node: drain whatever the UART has, without blocking the radio ───────────────
        let mut byte = [0u8; 1];
        if embassy_time::with_timeout(Duration::from_millis(1), uart.read_exact(&mut byte))
            .await
            .is_ok()
        {
            if let Some((typ, pl)) = parser.push(byte[0]) {
                match typ {
                    serial::CMD_TX => {
                        // Stage → announce → key up. The announcement sits between the last point a
                        // transmit can be refused (an over-long payload) and the SPI command that
                        // actually keys the PA, so `EVT_TX_STARTED` never promises a frame that does
                        // not go.
                        let ok = if phy_link::tx_stage(&mut radio, &state, pl).await {
                            let air = state.airtime_ms_be(pl.len());
                            send(&mut uart, &mut out, serial::EVT_TX_STARTED, &air).await;
                            tx_fire(
                                &mut radio,
                                &cap,
                                &mut hoptrace,
                                tx_timeout_ms(&state, pl.len()),
                            )
                            .await
                        } else {
                            false
                        };
                        n_tx = n_tx.wrapping_add(1);
                        send(&mut uart, &mut out, serial::EVT_TXDONE, &[ok as u8, 0]).await;
                        // Continuous RX is dropped by a transmit; re-arm or the node goes deaf
                        // after its first frame — a failure that looks like "the link died".
                        let _ = phy_link::arm_rx(&mut radio, &state).await;
                    }
                    serial::CMD_TX_LBT => {
                        let air = state.airtime_ms_be(pl.len());
                        let (sent, attempts) =
                            lbt_tx(&mut radio, &state, &mut sense, &cap, &mut hoptrace, pl, || {
                                send(&mut uart, &mut out, serial::EVT_TX_STARTED, &air)
                            })
                            .await;
                        n_tx = n_tx.wrapping_add(1);
                        send(
                            &mut uart,
                            &mut out,
                            serial::EVT_TXDONE,
                            &[sent as u8, attempts],
                        )
                        .await;
                        let _ = phy_link::arm_rx(&mut radio, &state).await;
                    }
                    serial::CMD_SET_FREQ if pl.len() >= 4 => {
                        let f = u32::from_be_bytes([pl[0], pl[1], pl[2], pl[3]]);
                        if !phy::in_band(f) {
                            // Refusing beats bricking: a `SetRfFrequency` outside the calibrated
                            // front end succeeds and then goes deaf with RXFREQ_NO_FE_CAL set.
                            send(
                                &mut uart,
                                &mut out,
                                serial::EVT_UNSUPPORTED,
                                &[serial::CMD_SET_FREQ, serial::REASON_OUT_OF_RANGE],
                            )
                            .await;
                        } else {
                            state.freq_hz = f;
                            // ★ The whole RF chain, from Standby RC. Issuing `set_rf` from
                            // RX-continuous is what used to break TX permanently: every later
                            // transmit returned ok=0 until the board was power-cycled, and a
                            // following CMD_SET_PWR did not restore it.
                            retune(&mut radio, &state).await;
                            let body = info_body(&mut radio, &state, &sense).await;
                            send(&mut uart, &mut out, serial::EVT_INFO, &body).await;
                        }
                    }
                    serial::CMD_SET_PWR if !pl.is_empty() => {
                        // The host speaks dBm; the chip takes HALF-dB steps. Passing the host's
                        // value straight through made every power request come out 2x too low.
                        state.tx_power_half_db = phy::dbm_to_half_db(pl[0] as i8);
                        retune(&mut radio, &state).await;
                        let body = info_body(&mut radio, &state, &sense).await;
                        send(&mut uart, &mut out, serial::EVT_INFO, &body).await;
                    }
                    serial::CMD_SET_MOD if pl.len() >= 3 => {
                        // The fleet's `[sf, bw, cr]` byte positions, decoded for whichever PHY is
                        // current — see `set_mod` for the three tables. A rate change needed a
                        // reflash (`PHY_BR`) until v2; a MODULATION change needed one until v3.
                        if set_mod(&mut state, pl[0], pl[1], pl[2]) {
                            retune(&mut radio, &state).await;
                            let body = info_body(&mut radio, &state, &sense).await;
                            send(&mut uart, &mut out, serial::EVT_INFO, &body).await;
                        } else {
                            send(
                                &mut uart,
                                &mut out,
                                serial::EVT_UNSUPPORTED,
                                &[serial::CMD_SET_MOD, serial::REASON_OUT_OF_RANGE],
                            )
                            .await;
                        }
                    }
                    serial::CMD_GET_INFO => {
                        let body = info_body(&mut radio, &state, &sense).await;
                        send(&mut uart, &mut out, serial::EVT_INFO, &body).await;
                    }
                    serial::CMD_GET_RSSI => {
                        let r = radio.get_rssi_inst().await.map(dbm_of).unwrap_or(0);
                        send(&mut uart, &mut out, serial::EVT_RSSI, &r.to_be_bytes()).await;
                    }
                    serial::CMD_CAD => {
                        let busy = cca_busy(&mut radio, &sense).await;
                        if busy {
                            sense.activity = sense.activity.wrapping_add(1);
                        }
                        let _ = phy_link::arm_rx(&mut radio, &state).await;
                        send(&mut uart, &mut out, serial::EVT_CAD, &[busy as u8]).await;
                    }
                    serial::CMD_SENSE => {
                        // A gated CCA for the busy verdict, then `rssi_inst` for the dBm — read
                        // AFTER re-arming RX, because an instantaneous RSSI taken in FS is not a
                        // measurement of the channel.
                        let busy = cca_busy(&mut radio, &sense).await;
                        if busy {
                            sense.activity = sense.activity.wrapping_add(1);
                        }
                        let _ = phy_link::arm_rx(&mut radio, &state).await;
                        let rssi = radio.get_rssi_inst().await.map(dbm_of).unwrap_or(0);
                        let mut body = [0u8; 4];
                        body[..2].copy_from_slice(&sense.activity.to_be_bytes());
                        body[2..].copy_from_slice(&rssi.to_be_bytes());
                        send(&mut uart, &mut out, serial::EVT_SENSE, &body).await;
                    }
                    serial::CMD_SET_LBT_CFG if pl.len() >= 4 => {
                        sense.lbt_cw_ms = u16::from_be_bytes([pl[0], pl[1]]);
                        sense.lbt_max_backoff = pl[2];
                        sense.lbt_max_attempts = pl[3];
                        let body = info_body(&mut radio, &state, &sense).await;
                        send(&mut uart, &mut out, serial::EVT_INFO, &body).await;
                    }
                    serial::CMD_SET_SENSE_CFG if pl.len() >= 3 => {
                        sense.thresh_dbm = i16::from_be_bytes([pl[0], pl[1]]);
                        sense.repeat = pl[2];
                        let body = info_body(&mut radio, &state, &sense).await;
                        send(&mut uart, &mut out, serial::EVT_INFO, &body).await;
                    }
                    serial::CMD_SET_CAD_CFG if pl.len() >= 3 => {
                        // **One of the three fields maps; the other two are refused, not ignored.**
                        //
                        // On a LoRa node this is `[cadSymbolNum, det_peak, det_min]` — a listen
                        // length and a correlator's peak/min thresholds. This part has no LoRa
                        // correlator (there is no CAD in FLRC mode at all), so:
                        //
                        // * `sym` is the listen length, which DOES exist here: it scales the CCA
                        //   window `1/2/4/8/16 ×`, the same ladder the LoRa symbol codes walk.
                        // * `det_peak`/`det_min` have no analogue. Accepting them silently would be
                        //   a no-op knob; reinterpreting them as the RSSI threshold would be worse —
                        //   a LoRa host's `det_peak = 22` would become a −22 dBm busy threshold and
                        //   report a saturated channel as idle. The threshold on this node is
                        //   `CMD_SET_SENSE_CFG`'s, in real dBm, and one knob with one unit beats two
                        //   that can disagree.
                        //
                        // ⚠ **In LoRa mode this part DOES have a real correlator** (`SetLoraCadParams`
                        // with `nb_symbols`, a detection threshold and a per-symbol delta), so the
                        // two refused fields have a genuine analogue there. Wiring it is deliberately
                        // out of this pass: the sense/LBT path is energy-detect on every PHY today,
                        // and a `CMD_CAD` that means "correlator" in one mode and "energy" in another
                        // needs the host to key on `phy_current` before it can be trusted. Refusing
                        // is the safe direction until then — this is a gap, not a decision that LoRa
                        // CAD is unavailable.
                        if pl[0] > CAD_SYM_MAX || pl[1] != 0 || pl[2] != 0 {
                            send(
                                &mut uart,
                                &mut out,
                                serial::EVT_UNSUPPORTED,
                                &[serial::CMD_SET_CAD_CFG, serial::REASON_OUT_OF_RANGE],
                            )
                            .await;
                        } else {
                            sense.cca_steps = CCA_STEPS << pl[0];
                            // Program the chip's own RSSI-CAD engine to match, from FS as its
                            // configuration commands expect, so a future `SetCad` cannot disagree
                            // with the `SetCca` path that acts today. The threshold argument is in
                            // −dBm, which is the chip's unit, taken from the one threshold this node
                            // has.
                            let _ = radio.set_chip_mode(ChipMode::Fs).await;
                            let thresh = (-sense.thresh_dbm).clamp(0, 255) as u8;
                            let _ = radio
                                .set_cad_params(sense.cca_steps, thresh, ExitMode::Fallback, 0)
                                .await;
                            let _ = phy_link::arm_rx(&mut radio, &state).await;
                            let body = info_body(&mut radio, &state, &sense).await;
                            send(&mut uart, &mut out, serial::EVT_INFO, &body).await;
                        }
                    }
                    serial::CMD_SET_PREAMBLE if pl.len() >= 2 => {
                        // **Per-PHY, because the fleet's field is a LoRa SYMBOL count.**
                        //
                        // * **LoRa** — it means exactly what it says, and `pbl_len` is a u16 of
                        //   symbols. This is the one PHY where the field needs no reinterpretation
                        //   at all; 0 is refused because a preamble a receiver cannot acquire on is
                        //   not a shorter preamble, it is a broken link.
                        // * **FLRC** — no symbols exist. The value is taken as the AGC preamble in
                        //   **bits**, this PHY's own unit, rounded UP to the register's 4-bit step so
                        //   a caller never gets less settling than it asked for, and refused outside
                        //   4..32 rather than clamped: a host sending LoRa's typical 8-*symbol*
                        //   preamble means something this radio cannot do, and 8 bits is not it.
                        // * **LR-FHSS** — there is no preamble knob. Its acquisition structure is the
                        //   sync-header replica count, which is a `LrFhssBuildFrame` argument and a
                        //   different quantity; mapping one onto the other would be the exact kind of
                        //   silent reinterpretation this protocol refuses.
                        //
                        // ⚠ Both ends must move together on every PHY — the receiver is sized by
                        // this, and a peer whose preamble shortened does not report an error, it
                        // simply stops hearing you.
                        let v = u16::from_be_bytes([pl[0], pl[1]]);
                        let applied = match &mut state.mode {
                            PhyMode::Lora(p) if v > 0 => {
                                p.preamble_syms = v;
                                true
                            }
                            PhyMode::Flrc(p) => match flrc_link::preamble_from_bits(v) {
                                Some(pbl) => {
                                    p.preamble = pbl;
                                    true
                                }
                                None => false,
                            },
                            _ => false,
                        };
                        if applied {
                            retune(&mut radio, &state).await;
                            let body = info_body(&mut radio, &state, &sense).await;
                            send(&mut uart, &mut out, serial::EVT_INFO, &body).await;
                        } else {
                            send(
                                &mut uart,
                                &mut out,
                                serial::EVT_UNSUPPORTED,
                                &[serial::CMD_SET_PREAMBLE, serial::REASON_OUT_OF_RANGE],
                            )
                            .await;
                        }
                    }
                    serial::CMD_SET_BEACON if !pl.is_empty() => {
                        // Default OFF (see the declaration): a node with no host attached stays
                        // silent, so it cannot pollute a neighbour's measurement — this bench has
                        // been bitten by a chatty node before. The optional second byte scales the
                        // base period.
                        beacon_on = pl[0] != 0;
                        if pl.len() >= 2 {
                            beacon_period_ms =
                                BEACON_BASE_PERIOD_MS.saturating_mul(pl[1].max(1) as u64);
                        }
                        // Re-base the phase on the command, so enabling the beacon does not fire one
                        // immediately from a deadline that expired while it was off.
                        next_beacon = Instant::now() + Duration::from_millis(beacon_period_ms);
                        let body = info_body(&mut radio, &state, &sense).await;
                        send(&mut uart, &mut out, serial::EVT_INFO, &body).await;
                    }
                    serial::CMD_GET_HOPTRACE => {
                        // ★ **This node's own hop timeline** — the instrument for "when does a hop
                        // boundary fall?", and the reason no cross-vendor link is needed to answer
                        // it: each node stamps its OWN hops on its OWN clock, and the host compares
                        // two timelines. The link that would otherwise have to carry the comparison
                        // is the very thing under investigation.
                        //
                        // Free-running and wrapping; **reading does not clear it**, the same
                        // contract as `EVT_SENSE.activity` — and neither does turning hopping off,
                        // which is the contract the Heltec node implements too. `n = 0` therefore
                        // means "nothing recorded since the last plan was ARMED", never "hopping is
                        // off right now"; see `HopTrace::arm` for why that distinction is the one
                        // this instrument must not blur. Nothing is ever fabricated: a node that
                        // could not stamp its hops would answer
                        // `EVT_UNSUPPORTED [0x20, NO_HARDWARE]` instead, and this node can.
                        //
                        // ⚠ **Do not poll this during the window you are measuring.** The reply is
                        // up to 170 bytes on a 115200 UART ≈ 15 ms — longer than one 8.192 ms hop
                        // period — and the loop is not polling the chip's interrupt status while it
                        // writes, so hops landing inside that window coalesce into one entry.
                        //
                        // No argument is taken and none is rejected: an empty payload is the whole
                        // command, so a host that sends stray bytes still gets its trace rather than
                        // a `BAD_LENGTH` for a field that does not exist.
                        let mut body = [0u8; hoptrace_mod::MAX_BODY_LEN];
                        let k = hoptrace.encode(&mut body);
                        send(&mut uart, &mut out, serial::EVT_HOPTRACE, &body[..k]).await;
                        // The DIO8-sharing cost, reported rather than described: how many times a
                        // hop and an RxDone shared one capture. Debug-gated like every other
                        // diagnostic here, so a quiet link stays quiet.
                        if debug {
                            let mut m = [0u8; 40];
                            let n = fmt_kv(&mut m, b"HOPTRACE coalesced=", hoptrace.coalesced);
                            send(&mut uart, &mut out, serial::EVT_LOG, &m[..n]).await;
                        }
                    }
                    serial::CMD_SET_DEBUG if !pl.is_empty() => {
                        debug = pl[0] != 0;
                        let body = info_body(&mut radio, &state, &sense).await;
                        send(&mut uart, &mut out, serial::EVT_INFO, &body).await;
                    }
                    serial::CMD_SET_RX_GAIN if !pl.is_empty() => {
                        // The fleet's byte is a two-value LNA knob (0 = the chip's default,
                        // 1 = boosted). The LR2021 has a 0..13 manual ladder with 0 meaning "give it
                        // back to the AGC", so 0 maps exactly and 1 maps to the ceiling. Steps 2..13
                        // are NOT exposed through this byte: `1` would then mean "boosted" on the
                        // Waveshare and "the lowest manual gain" here — an inversion, which is a
                        // failure this rig has already paid for once on a TX-power knob.
                        match pl[0] {
                            0 | 1 => {
                                let gain = if pl[0] == 0 { RX_GAIN_AUTO } else { RX_GAIN_MAX };
                                let _ = radio.set_chip_mode(ChipMode::Fs).await;
                                let _ = radio.set_rx_gain(gain).await;
                                let _ = phy_link::arm_rx(&mut radio, &state).await;
                                let body = info_body(&mut radio, &state, &sense).await;
                                send(&mut uart, &mut out, serial::EVT_INFO, &body).await;
                            }
                            _ => {
                                send(
                                    &mut uart,
                                    &mut out,
                                    serial::EVT_UNSUPPORTED,
                                    &[serial::CMD_SET_RX_GAIN, serial::REASON_OUT_OF_RANGE],
                                )
                                .await;
                            }
                        }
                    }
                    serial::CMD_SET_NAME_FILTER | serial::CMD_SET_RELAY => {
                        let mut hashes = [0u64; 8];
                        let n = (pl.len() / 8).min(hashes.len());
                        for i in 0..n {
                            let mut h = [0u8; 8];
                            h.copy_from_slice(&pl[i * 8..i * 8 + 8]);
                            hashes[i] = u64::from_be_bytes(h);
                        }
                        if typ == serial::CMD_SET_NAME_FILTER {
                            dp.set_filter(&hashes[..n]);
                        } else {
                            dp.set_relay(&hashes[..n]);
                        }
                        let body = info_body(&mut radio, &state, &sense).await;
                        send(&mut uart, &mut out, serial::EVT_INFO, &body).await;
                    }
                    serial::CMD_DATAPLANE if pl.len() >= 2 => {
                        // Name-keyed hopping is refused rather than half-applied: it needs a
                        // channel-index convention (`carrier = (850 + ch) MHz` on the LoRa nodes)
                        // and this bearer has none. Validate before applying anything.
                        //
                        // Note this is a **different** mechanism from `CMD_SET_HOP`: that one is
                        // INTRA-packet hopping, where the carrier moves inside a single frame on a
                        // table the host writes in Hz. This field is inter-frame channel selection by
                        // name, which still has no channel index here.
                        if pl.len() >= 3 && pl[2] != 0 {
                            send(
                                &mut uart,
                                &mut out,
                                serial::EVT_UNSUPPORTED,
                                &[serial::CMD_DATAPLANE, serial::REASON_OUT_OF_RANGE],
                            )
                            .await;
                        } else {
                            dp.set_cs_serve(pl[0] != 0);
                            dp.set_dedup(pl[1] != 0);
                            let body = info_body(&mut radio, &state, &sense).await;
                            send(&mut uart, &mut out, serial::EVT_INFO, &body).await;
                        }
                    }
                    serial::CMD_GET_STATS => {
                        let mut body = [0u8; 24];
                        body[0..4].copy_from_slice(&dp.rx.to_be_bytes());
                        body[4..8].copy_from_slice(&dp.filtered.to_be_bytes());
                        body[8..12].copy_from_slice(&dp.deduped.to_be_bytes());
                        body[12..16].copy_from_slice(&dp.served.to_be_bytes());
                        body[16..20].copy_from_slice(&dp.relayed.to_be_bytes());
                        body[20..22].copy_from_slice(&sense.cad_busy.to_be_bytes());
                        body[22..24].copy_from_slice(&sense.defer.to_be_bytes());
                        send(&mut uart, &mut out, serial::EVT_STATS, &body).await;
                    }
                    serial::CMD_RESET_STATS => {
                        n_rx = 0;
                        n_tx = 0;
                        dp.reset_stats();
                        sense.cad_busy = 0;
                        sense.defer = 0;
                        // `activity` is deliberately NOT reset: it is documented free-running, and
                        // a host differencing it across a reset would read a huge negative window.
                        let body = info_body(&mut radio, &state, &sense).await;
                        send(&mut uart, &mut out, serial::EVT_INFO, &body).await;
                    }
                    serial::CMD_READ_CLOCK => {
                        let ticks = now64(&cap, &mut clock_hi, &mut clock_last);
                        send(&mut uart, &mut out, serial::EVT_CLOCK, &ticks.to_be_bytes()).await;
                    }
                    serial::CMD_GET_CAP => {
                        send(&mut uart, &mut out, serial::EVT_CAP, &cap_bytes).await;
                    }
                    serial::CMD_TX_AT if pl.len() >= 4 => {
                        // ★ **Scheduled TX, CPU-mediated — and the pin conflict that shapes it.**
                        //
                        // The DPPI path `m5_tx` demonstrates is `TIMER20.CC[2] --DPPI--> GPIOTE OUT`
                        // driving a DIO the radio has configured as `DioFunc::TxTrigger`. The shield
                        // brings out exactly ONE DIO: LR2021 **DIO8 → P1.04**. That same pin is the
                        // radio's IRQ output, and `RxCapture` holds it as a GPIOTE20_CH0
                        // **InputChannel** routed by PPI20_CH0 into `TIMER20.CC[0].CAPTURE` — the
                        // hardware RX timestamp. On the radio side DIO8 is either an interrupt
                        // OUTPUT or a trigger INPUT; on the MCU side P1.04 is either a GPIOTE input
                        // or an output. One pin, opposite directions, on both ends.
                        //
                        // That conflict is real and the RX capture wins it: 62.5 ns is the number
                        // this whole board exists to produce, and it is the fleet's only hardware
                        // stamp. So the schedule is kept on the CPU instead — wait on the **same
                        // free-running counter** the stamp and `CMD_READ_CLOCK` use, then issue
                        // `SetTx` over SPI. Looser than DPPI by the four terms in
                        // `airtime::SCHED_GRAN_NS` (~38 µs of tick + SPI + ramp + executor wake,
                        // declared as 50 µs), and REAL, and declared — which is the part that
                        // matters. A scheduler that is told 50 µs and gets 50 µs can build a guard
                        // band; one told 62.5 ns and given 38 µs cannot.
                        //
                        // ★ **But `delay_us` is counted from HERE — from the moment the firmware
                        // processes the arm — so the host→device serial latency is inside the
                        // placement, and MEASURED that dominates everything above.** An
                        // absolute-boundary slot train fired 45/45 with excellent accuracy (mean gap
                        // 2,399,818 ticks vs 2,400,000 nominal, 11 µs over 44 slots) and jitter
                        // **sd 553 µs / p2p 1875 µs**; this node's `CMD_GET_INFO` round trip is p2p
                        // **550 µs**. Same number. As exercised, host-armed relative scheduling is
                        // *worse* than the software path (sd 553 vs 155 µs) because it pays an extra
                        // round trip for nothing.
                        //
                        // `CMD_TX_AT_ABS` is the fix and lives just below. This opcode stays because
                        // it is still the right primitive for a delay the FIRMWARE computes — a
                        // beacon offset, a backoff — where no serial hop exists to pollute it.
                        let delay_us = u32::from_be_bytes([pl[0], pl[1], pl[2], pl[3]]);
                        let frame = &pl[4..];
                        if delay_us > MAX_SCHED_DELAY_US || frame.len() > state.max_payload() {
                            send(
                                &mut uart,
                                &mut out,
                                serial::EVT_UNSUPPORTED,
                                &[serial::CMD_TX_AT, serial::REASON_OUT_OF_RANGE],
                            )
                            .await;
                        } else {
                            // Compute the target BEFORE staging, so the SPI work that follows eats
                            // into the delay rather than being added to it. `delay_us = 0` yields
                            // `target = now`, which `wait_until` treats as due — inject now.
                            //
                            // **Floor: staging costs ~600 µs** — 500 µs of `settle_before_tx` (the
                            // measured PLL settle, #108) plus the FIFO write. A delay shorter than
                            // that cannot be honoured, and the frame goes as soon as staging
                            // finishes rather than being held for a wrap or refused. That is the
                            // same rule as a target already in the past, and it is why the `EVT_LOG`
                            // line below reports the lateness in ticks: it is the only place the
                            // difference between "placed" and "as soon as possible" is visible.
                            let target =
                                cap.now().ticks.wrapping_add(delay_us.wrapping_mul(TICKS_PER_US));
                            let ok = if phy_link::tx_stage(&mut radio, &state, frame).await {
                                // ★ `EVT_TX_STARTED` goes out HERE — after staging has committed
                                // the frame, and BEFORE the wait. It must NOT sit between
                                // `wait_until` returning and `tx_fire`, which is where it used to
                                // be: `send` is `BufferedUarte::write_all`, which returns once the
                                // 512-byte TX ring has taken the bytes and BLOCKS while it drains
                                // if a preceding `EVT_RX` still occupies it. That drain is up to
                                // 512 B / 115200 8N1 = 44 ms — unbounded against the 50 µs this
                                // node declares in `EVT_CAP.sched_gran_ns`, and a granularity a
                                // slot scheduler is told is 50 µs and given tens of milliseconds is
                                // exactly the lie that field exists to prevent. The Heltec node
                                // reached the same conclusion independently; see the
                                // `SchedTx::Pending` arm in `../heltec-lora-rs/src/main.rs`.
                                //
                                // Emitting at acceptance is safe on THIS node, where it would not
                                // be on the Heltec: the host re-bases its reply deadline to
                                // `now + airtime + AIRTIME_SLACK` (2 s, `lora_serial.rs`) the
                                // moment it sees this event, and `MAX_SCHED_DELAY_US` here is 1 s,
                                // so the re-based deadline still covers the longest schedule this
                                // node will accept. The Heltec allows 60 s and therefore has to
                                // emit from its staging point instead.
                                let air = state.airtime_ms_be(frame.len());
                                send(&mut uart, &mut out, serial::EVT_TX_STARTED, &air).await;
                                wait_until(&cap, target).await;
                                tx_fire(
                                    &mut radio,
                                    &cap,
                                    &mut hoptrace,
                                    tx_timeout_ms(&state, frame.len()),
                                )
                                .await
                            } else {
                                false
                            };
                            n_tx = n_tx.wrapping_add(1);
                            // Lateness, on the node's own clock, for whoever is calibrating the
                            // declared granularity. Wraps to a huge number if the key-up somehow
                            // preceded the target, which cannot happen and would be worth seeing.
                            let err_ticks = cap.now().ticks.wrapping_sub(target);
                            let mut m = [0u8; 32];
                            let n = fmt_kv(&mut m, b"TX_AT late_ticks=", err_ticks);
                            log(&mut uart, &mut out, debug, &m[..n]).await;
                            send(&mut uart, &mut out, serial::EVT_TXDONE, &[ok as u8, 0]).await;
                            let _ = phy_link::arm_rx(&mut radio, &state).await;
                        }
                    }
                    serial::CMD_TX_AT_ABS if pl.len() >= 8 => {
                        // ★ **The absolute-target transmit — the one that actually earns
                        // `sched_gran_ns`.**
                        //
                        // `CMD_TX_AT` says "in N microseconds", counted from when the firmware got
                        // the command, so the serial round trip lands inside the placement (measured
                        // above: sd 553 µs against a declared 50 µs). This says "at instant T" on the
                        // node's own free-running TIMER20 — the SAME counter `CMD_READ_CLOCK` reports
                        // and `EVT_RX.ts` is latched from — so the host can read the clock, name a
                        // slot boundary, and its own latency cannot move the frame: it is spent
                        // *before* the deadline, where it is free.
                        //
                        // ## The 32/64-bit arithmetic, explicitly
                        //
                        // The hardware counter is 32 bits and wraps every 2^32/16 MHz ≈ 268.4 s. The
                        // target is 64 bits on the firmware-extended clock (`now64`), so:
                        //
                        // ```text
                        //   now  = (wraps << 32) | timer32          both read atomically
                        //   past:      target <= now                -> fire immediately
                        //   far:       target - now > 1 s of ticks   -> OUT_OF_RANGE
                        //   otherwise: delta = target - now  <= 16e6 ticks  (< 2^24)
                        //              wait_until(target as u32)
                        // ```
                        //
                        // The cast to `u32` is safe *because* of the bound: with `delta < 2^31`,
                        // `target.wrapping_sub(now32)` in `wait_until` is the true remaining count
                        // whether or not the counter wraps in between — which is exactly the
                        // top-half test `wait_until` already makes. Without the bound the low 32 bits
                        // of a far-future target would be indistinguishable from a past one, and
                        // `m5_tx` measured that failure: 74 transmits and then silence until the
                        // counter came round.
                        //
                        // **A target already past fires immediately** rather than waiting ~268 s for
                        // the wrap. That is the same rule `CMD_TX_AT` uses for `delay_us = 0`, and
                        // the `EVT_LOG` line at the end reports the lateness in ticks so "placed" and
                        // "as soon as possible" are distinguishable from the host.
                        let target = u64::from_be_bytes([
                            pl[0], pl[1], pl[2], pl[3], pl[4], pl[5], pl[6], pl[7],
                        ]);
                        let frame = &pl[8..];
                        let now = now64(&cap, &mut clock_hi, &mut clock_last);
                        let ahead = target.saturating_sub(now);
                        if ahead > MAX_SCHED_DELAY_TICKS || frame.len() > state.max_payload() {
                            send(
                                &mut uart,
                                &mut out,
                                serial::EVT_UNSUPPORTED,
                                &[serial::CMD_TX_AT_ABS, serial::REASON_OUT_OF_RANGE],
                            )
                            .await;
                        } else {
                            let target32 = target as u32;
                            let ok = if phy_link::tx_stage(&mut radio, &state, frame).await {
                                // Announced at acceptance, for the reason spelled out in the
                                // `CMD_TX_AT` arm: `send` can block for up to 44 ms draining the
                                // UART ring, which must not sit between the deadline and key-up.
                                let air = state.airtime_ms_be(frame.len());
                                send(&mut uart, &mut out, serial::EVT_TX_STARTED, &air).await;
                                wait_until(&cap, target32).await;
                                tx_fire(
                                    &mut radio,
                                    &cap,
                                    &mut hoptrace,
                                    tx_timeout_ms(&state, frame.len()),
                                )
                                .await
                            } else {
                                false
                            };
                            n_tx = n_tx.wrapping_add(1);
                            let err_ticks = cap.now().ticks.wrapping_sub(target32);
                            let mut m = [0u8; 40];
                            let n = fmt_kv(&mut m, b"TX_AT_ABS late_ticks=", err_ticks);
                            log(&mut uart, &mut out, debug, &m[..n]).await;
                            send(&mut uart, &mut out, serial::EVT_TXDONE, &[ok as u8, 0]).await;
                            let _ = phy_link::arm_rx(&mut radio, &state).await;
                        }
                    }
                    serial::CMD_SET_PHY if !pl.is_empty() => {
                        // ★ **Modulation as a knob.** `SetPacketType` is a runtime command with 14
                        // modes; this node brings up three of them and says which in
                        // `EVT_CAP.phy_bitmap`.
                        //
                        // Three outcomes, and the difference between the last two is the whole point
                        // of `EVT_PHY_ERR`:
                        //
                        // 1. a mode this build does not advertise -> `EVT_UNSUPPORTED
                        //    [cmd, OUT_OF_RANGE]`. The firmware is saying "I do not offer that";
                        // 2. an advertised mode the CHIP refuses -> `EVT_PHY_ERR [requested,
                        //    chip_status]`, carrying the status byte the failing command itself
                        //    returned, and the node goes **back to the PHY it was running**. A failed
                        //    switch must not leave the radio half-programmed;
                        // 3. success -> the **full new `EVT_CAP`**, because `max_payload`,
                        //    `sf_min`/`sf_max` and `sched_gran_ns` all moved. The host replaces its
                        //    profile; it must not patch fields.
                        match Phy::from_code(pl[0]) {
                            Some(p) if phy::advertised(p) => {
                                let prev = state;
                                state.set_phy(p);
                                // `set_phy` drops the hop table (a table written for one modulation
                                // silently re-arming under another is the stale-state bug that
                                // whole path exists to remove), so the trace stops recording with
                                // it and the hop interrupts come back off DIO8, which every
                                // `set_dio_irq` below then re-issues correctly.
                                //
                                // It does NOT erase what was already recorded — the same rule
                                // `CMD_SET_HOP 0` follows, and the same one the Heltec follows: the
                                // stamps stay true across a PHY switch, and deleting a run because
                                // the host reconfigured afterwards is how a measurement disappears
                                // at the moment it was taken. What the host must not do is read the
                                // *symbol period* out of a trace recorded under a different
                                // modulation; the ticks are raw and the sf/bw is the host's to
                                // track.
                                hoptrace.arm(0);
                                if phy_link::apply(&mut radio, &state).await.is_err() {
                                    // ── (2) the CHIP refused the mode ─────────────────────────
                                    // Its literal status byte, from the cached status of the command
                                    // that just failed — no further SPI in between to overwrite it.
                                    let chip = phy_link::status_byte(&radio);
                                    // ☠ **Clear the latched error BEFORE reverting, or the revert
                                    // fails too.** MEASURED: the revert is itself an `apply`, so on
                                    // a chip carrying a pending error flag it inherits the very
                                    // condition it is trying to escape. The node then sits in the
                                    // half-entered mode refusing every later `CMD_SET_PHY` *and*
                                    // every `CMD_TX`, recoverable only by a board reset.
                                    // Reading `status_byte` first is deliberate — this is SPI
                                    // traffic and it overwrites the status we just reported.
                                    let _ = phy_link::clear_errors(&mut radio).await;
                                    // Back to the mode that was working, in full: `SetPacketType`
                                    // changed which modem registers exist, so nothing less than a
                                    // complete re-apply restores it.
                                    state = prev;
                                    // ☠ And if even the revert cannot take — MEASURED to happen
                                    // after a refused LR-FHSS arm — hard-reset and rebuild. A PHY
                                    // switch must never leave this node unable to transmit AND
                                    // unable to switch; see `phy_link::reset_and_apply`.
                                    if phy_link::apply(&mut radio, &state).await.is_err() {
                                        let _ = phy_link::reset_and_apply(&mut radio, &state).await;
                                        let _ =
                                            radio
                                                .set_dio_irq(
                                                    DioNum::Dio8,
                                                    dio_irq_mask(hoptrace.armed()),
                                                )
                                                .await;
                                    }
                                    let _ = phy_link::arm_rx(&mut radio, &state).await;
                                    send(&mut uart, &mut out, serial::EVT_PHY_ERR, &[p.code(), chip])
                                        .await;
                                } else {
                                    // The IRQ routing is re-issued because DIO8 is not just an
                                    // interrupt line here — it is the capture source of the hardware
                                    // RX timestamp, the one capability this whole board exists for.
                                    // If `SetPacketType` clears the routing, the stamps stop and
                                    // nothing says so; one command makes that impossible.
                                    let _ = radio
                                        .set_dio_irq(DioNum::Dio8, dio_irq_mask(hoptrace.armed()))
                                        .await;

                                    // ★ **Arming RX is a SEPARATE question from entering the mode,
                                    // and LR-FHSS is why.** §17.1 calls it transmit-only; §17.2.2
                                    // describes its syncword as being for "detection on the receiver
                                    // side". If the chip refuses `SetRxContinuous` here, that is a
                                    // finding about RX — it is NOT a reason to refuse the PHY, which
                                    // transmits perfectly well. Reverting would make LR-FHSS
                                    // unreachable and unmeasurable, which is the opposite of the
                                    // point.
                                    //
                                    // So: stay in the mode, report the chip's literal answer, and
                                    // still send the new `EVT_CAP` so the host gets its profile. A
                                    // host that sees both learns "you are in LR-FHSS, and this chip
                                    // would not arm its receiver" — which is exactly the measurement.
                                    //
                                    // ⚠ And a SUCCESS here is not evidence that LR-FHSS receives.
                                    // Arming is not receiving; only frames on air settle that.
                                    let armed = phy_link::arm_rx(&mut radio, &state).await;
                                    // Read the chip's word BEFORE any other SPI can overwrite the
                                    // cached status; the send below is UART only.
                                    let arm_err = match armed {
                                        Ok(()) => None,
                                        Err(_) => {
                                            // Chip's own word first — `clear_errors` below is SPI
                                            // and would overwrite the cached status.
                                            let chip = phy_link::status_byte(&radio);
                                            // ☠ **MEASURED: without this the node is WEDGED.** A
                                            // refused `SetRxContinuous` leaves a pending error flag,
                                            // and the chip does not clear it on its own (§6.7.3). Every
                                            // later command then fails — `CMD_SET_PHY` back to FLRC or
                                            // LoRa, and `CMD_TX` (`ok=0`) — so the mode we entered in
                                            // order to MEASURE became a one-way door, escapable only
                                            // by a board reset. Staying in the PHY after a failed arm
                                            // is right (LR-FHSS transmits fine); stranding the node is
                                            // not, and clearing the flag is the difference.
                                            let _ = phy_link::clear_errors(&mut radio).await;
                                            // ☠ `clear_errors` alone was MEASURED not to recover
                                            // this — the part stays stuck until NRESET. Rebuild it
                                            // now, in the mode we just entered, so LR-FHSS stays
                                            // reachable for TX (which is its only direction) without
                                            // the node becoming a one-way door.
                                            let _ =
                                                phy_link::reset_and_apply(&mut radio, &state).await;
                                            let _ = radio
                                                .set_dio_irq(DioNum::Dio8, dio_irq_mask(hoptrace.armed()))
                                                .await;
                                            Some(chip)
                                        }
                                    };
                                    // ★ **The reply goes first, and the order is a host-side fact
                                    // rather than a taste.** `EVT_CAP` is the reply to
                                    // `CMD_SET_PHY`; the host blocks on it, and it treats an
                                    // `EVT_PHY_ERR` that arrives *first* as a terminal refusal of
                                    // the whole command (`lora_serial::exec_on`). Emitting the arm
                                    // failure ahead of the capability would therefore tell the host
                                    // the switch did not happen while this node is sitting in the
                                    // new PHY — leaving the host's profile on the OLD mode's
                                    // `max_payload`, SF span and rate model. That stale-profile
                                    // desync is the exact bug class v3 exists to remove, and here
                                    // it would be caused by the one path v3 added to measure.
                                    //
                                    // So: the capability (the answer to the question asked), then
                                    // the finding about RX (a separate fact, and the one LR-FHSS is
                                    // reachable in order to establish).
                                    cap_bytes = caps_for(&state).to_bytes();
                                    send(&mut uart, &mut out, serial::EVT_CAP, &cap_bytes).await;
                                    if let Some(chip) = arm_err {
                                        send(
                                            &mut uart,
                                            &mut out,
                                            serial::EVT_PHY_ERR,
                                            &[p.code(), chip],
                                        )
                                        .await;
                                    }
                                }
                            }
                            _ => {
                                send(
                                    &mut uart,
                                    &mut out,
                                    serial::EVT_UNSUPPORTED,
                                    &[serial::CMD_SET_PHY, serial::REASON_OUT_OF_RANGE],
                                )
                                .await;
                            }
                        }
                    }
                    serial::CMD_SET_HOP if pl.len() >= 4 => {
                        // **Intra-packet frequency hopping** — the carrier moves inside one frame,
                        // on a table the host writes. `[hop_ctrl][period u16][n][freq u32]*n`.
                        //
                        // Validated as a unit before anything reaches the chip (`phy::check_hop`):
                        // wrong PHY, reserved control bits, more than 40 hops, a zero dwell, or a
                        // frequency outside the calibrated band. A table accepted for its first 20
                        // entries and rejected for its 21st would leave the modem in a state no host
                        // command describes.
                        //
                        // FLRC is refused rather than ignored: this part has no FLRC hopping command
                        // at all, and silently accepting the table would leave a host believing its
                        // frames were spread when they were sitting on one carrier.
                        let ctrl = pl[0];
                        let period = u16::from_be_bytes([pl[1], pl[2]]);
                        let n = pl[3] as usize;
                        let mut freqs = [0u32; phy::MAX_HOPS];
                        let have = ((pl.len() - 4) / 4).min(n);
                        for i in 0..have.min(phy::MAX_HOPS) {
                            let o = 4 + 4 * i;
                            freqs[i] = u32::from_be_bytes([pl[o], pl[o + 1], pl[o + 2], pl[o + 3]]);
                        }
                        // A short frame is a BAD_LENGTH, not an out-of-range table: the host said it
                        // was sending n frequencies and sent fewer, which is a framing bug on its
                        // side and should not be silently applied as a shorter hop sequence.
                        if have != n || n > phy::MAX_HOPS {
                            send(
                                &mut uart,
                                &mut out,
                                serial::EVT_UNSUPPORTED,
                                &[
                                    serial::CMD_SET_HOP,
                                    if n > phy::MAX_HOPS {
                                        serial::REASON_OUT_OF_RANGE
                                    } else {
                                        serial::REASON_BAD_LENGTH
                                    },
                                ],
                            )
                            .await;
                        } else {
                            match phy::check_hop(state.phy(), ctrl, period, &freqs[..n]) {
                                Ok(enable) => {
                                    state.hop.set(enable, period, &freqs[..n]);
                                    // **Arm the hop timeline with the plan.** The table depth is
                                    // what labels each event's index. ARMING clears the ring, so a
                                    // timeline taken under one plan is never served under another;
                                    // DISABLING does not, because reading the trace after a run is
                                    // the use case and a host that stops the hopping first must not
                                    // be handed `n = 0` — which the measurement protocol reads as
                                    // "this part does not signal its hops". Same rule on the Heltec.
                                    // See `hoptrace::HopTrace::arm`.
                                    hoptrace.arm(if enable { n as u8 } else { 0 });
                                    // A full re-apply: `SetLoraHopping` is programmed inside the
                                    // LoRa modem block, after the modulation it depends on, and the
                                    // SX127x compatibility bit lives in a register a
                                    // `SetLoraModulationParams` can reset.
                                    //
                                    // In **LR-FHSS** the table is written per frame instead, after
                                    // `LrFhssBuildFrame` (which computes one of its own and would
                                    // overwrite anything written before it) — so the re-apply below
                                    // stores the plan and the chip sees it at the next transmit.
                                    retune(&mut radio, &state).await;
                                    // ⚠ **The interrupt routing has to be re-issued here**, because
                                    // this is the one command that changes which interrupts DIO8
                                    // carries: the hop event joins the line while a plan is live and
                                    // leaves it again when the plan is dropped. `retune` re-applies
                                    // the modem, not the DIO configuration — and a hop trace that
                                    // silently recorded nothing because the interrupt never reached
                                    // the pin is exactly the failure M4 already paid for once.
                                    let _ = radio
                                        .set_dio_irq(DioNum::Dio8, dio_irq_mask(hoptrace.armed()))
                                        .await;
                                    // Flush whatever the status word was already carrying, so the
                                    // first entry of the fresh timeline is a real post-arm edge.
                                    // The hop bits latch in the status whether or not they are
                                    // routed to DIO8; a stale one surviving the arm would be paired
                                    // with a stale `CC[0]` — an invented instant at the head of the
                                    // trace, which is precisely the kind of plausible fiction this
                                    // instrument must not produce.
                                    let _ = radio.get_and_clear_irq().await;
                                    hop_probe(&mut radio, &mut uart, &mut out, &state, debug).await;
                                    let body = info_body(&mut radio, &state, &sense).await;
                                    send(&mut uart, &mut out, serial::EVT_INFO, &body).await;
                                }
                                Err(_) => {
                                    send(
                                        &mut uart,
                                        &mut out,
                                        serial::EVT_UNSUPPORTED,
                                        &[serial::CMD_SET_HOP, serial::REASON_OUT_OF_RANGE],
                                    )
                                    .await;
                                }
                            }
                        }
                    }
                    // ── Understood, and unreachable from firmware ──────────────────────────────
                    //
                    // `NO_HARDWARE`, not `UNKNOWN_OPCODE`: the difference tells a host whether to
                    // file a bug or stop asking.
                    //
                    // * `CMD_SET_SYNC` — the wire carries ONE byte; FLRC's syncword is 32 bits. No
                    //   faithful mapping exists, and a byte→word expansion would be the most
                    //   dangerous kind of guess: two nodes that disagree about a syncword do not
                    //   error, they simply never hear each other, which is indistinguishable from a
                    //   dead radio. (`flrc_link::SYNCWORD` also documents why the value's
                    //   correlation properties are an RF parameter, not a host's to pick.)
                    // * `CMD_SF_SCAN` — FLRC has no spreading factor to scan for.
                    // * `CMD_ENTER_BOOTLOADER` — the XIAO reflashes over its onboard CMSIS-DAP
                    //   probe; there is no ROM serial loader to jump to.
                    serial::CMD_SET_SYNC
                    | serial::CMD_SF_SCAN
                    | serial::CMD_ENTER_BOOTLOADER => {
                        send(
                            &mut uart,
                            &mut out,
                            serial::EVT_UNSUPPORTED,
                            &[typ, serial::REASON_NO_HARDWARE],
                        )
                        .await;
                    }
                    // Reached only when one of the guarded arms above rejected the payload LENGTH.
                    // Must come before `other`, or a short `CMD_SET_FREQ` would be answered
                    // "not implemented" — contradicting the bit this node sets for it in
                    // `EVT_CAP.cmd_bitmap`, which is the one place the host trusts.
                    serial::CMD_SET_FREQ
                    | serial::CMD_SET_PWR
                    | serial::CMD_SET_MOD
                    | serial::CMD_SET_BEACON
                    | serial::CMD_SET_CAD_CFG
                    | serial::CMD_SET_LBT_CFG
                    | serial::CMD_SET_PREAMBLE
                    | serial::CMD_SET_SENSE_CFG
                    | serial::CMD_DATAPLANE
                    | serial::CMD_SET_DEBUG
                    | serial::CMD_TX_AT
                    | serial::CMD_TX_AT_ABS
                    | serial::CMD_SET_PHY
                    | serial::CMD_SET_HOP
                    | serial::CMD_SET_RX_GAIN => {
                        send(
                            &mut uart,
                            &mut out,
                            serial::EVT_UNSUPPORTED,
                            &[typ, serial::REASON_BAD_LENGTH],
                        )
                        .await;
                    }
                    other => {
                        // Answered, not ignored — see the module note.
                        send(
                            &mut uart,
                            &mut out,
                            serial::EVT_UNSUPPORTED,
                            &[other, serial::REASON_UNKNOWN_OPCODE],
                        )
                        .await;
                    }
                }
            }
        }

        // ── node → host: a captured frame, carrying the HARDWARE timestamp ─────────────────────
        if let Ok(irq) = radio.get_and_clear_irq().await {
            // ONE capture read per status word. `CC[0]` holds the instant of the FIRST DIO8 rising
            // edge since the previous clear (the line stays high until the IRQ is cleared over SPI,
            // so a second event raises no new edge) — so reading it twice would not give two
            // instants, it would give the same value with a race in between.
            //
            // ⚠ This loop polls at roughly 1 ms (the UART read timeout above plus the SPI work), so
            // at an 8.192 ms hop period it separates hops with ~8x of margin — but only ROUGHLY.
            // Anything that keeps the loop away for longer than a hop period (a large `send` on the
            // 115200 UART is the realistic one: 170 B is ~15 ms) collapses every hop in that gap
            // into a single entry stamped at the FIRST of them. That failure is not silent at the
            // host: the surviving intervals come out as integer MULTIPLES of the true period, which
            // is exactly what a reader should check for before trusting an interval. See `note_irq`
            // for the other sharing case, a hop and an RxDone on one edge.
            let ts = cap.hw_stamp().ticks;
            note_irq(&mut hoptrace, &irq, ts);
            if irq.rx_done() {
                // Reading the frame is **per-PHY** — length, signal and framing all come from
                // different places in each mode. See `phy_link::rx_read`, which also documents why
                // FLRC deliberately does not use `get_rx_pkt_len()`.
                //
                // Sized for the largest payload ANY PHY here carries, not for the current one: a
                // buffer sized from `state` at boot would truncate every frame after a switch to a
                // wider PHY, and truncation is silent.
                let mut rxb = [0u8; phy::MAX_PAYLOAD_ANY];
                let info = phy_link::rx_read(&mut radio, &state, &mut rxb).await;
                // A frame the PHY CRC rejected is not delivered. Whitening plus the in-frame length
                // check would catch most of it anyway, but a corrupt frame reaching the data plane
                // pollutes the dedup ring with a hash of noise.
                if !irq.crc_error() {
                    if let Some(info) = info {
                        let n = info.len;
                        let rssi = info.rssi_dbm;
                        // The chip's own SNR where the PHY has one (LoRa), and
                        // `packet RSSI − noise floor` where it does not (FLRC has no SNR register at
                        // all). One field, two provenances, and `EVT_CAP.phy_current` is what tells
                        // the host which it is looking at.
                        let snr = info.snr_db.unwrap_or_else(|| sense.snr_for(rssi));
                        n_rx = n_rx.wrapping_add(1);
                        if n_rx % 100 == 0 {
                            defmt::info!(
                                "m6_bridge: rx {=u32} tx {=u32} | last {=i16} dBm snr {=i16} dB, {=usize} B",
                                n_rx,
                                n_tx,
                                rssi,
                                snr,
                                n
                            );
                        }
                        // `rx_read` already stripped whatever framing the PHY has — FLRC's
                        // in-frame length byte and whitening, LoRa's explicit header — so this is
                        // the payload the peer sent, on every PHY.
                        let payload = &rxb[..n];
                        let now_ms = Instant::now().as_millis() as u32;

                        // ── the on-device NDN data plane, finally invoked ──────────────────────
                        // Constructed and configured but never consulted until now: filter, dedup,
                        // relay and CS-serve had no effect on this node while the host believed it
                        // had installed them. Both nodes must decide identically, so this is the
                        // same shape as `waveshare-lora-rs`'s main loop.
                        let mut serve = [0u8; ndn::CS_MAX_LEN];
                        let mut serve_len = 0usize;
                        let (deliver, relay) = match dp.on_rx(payload, now_ms) {
                            ndn::RxAction::Drop => (false, false),
                            ndn::RxAction::Serve(data) => {
                                let m = data.len().min(serve.len());
                                serve[..m].copy_from_slice(&data[..m]);
                                serve_len = m;
                                (false, false)
                            }
                            ndn::RxAction::Deliver => (true, false),
                            ndn::RxAction::RelayAndDeliver => (true, true),
                        };

                        // What the on-device data plane decided, and why the host may not see this
                        // frame. `EVT_LOG` exists for exactly this: a frame filtered on the radio is
                        // indistinguishable from a frame never received, from the host's side of the
                        // serial link.
                        if debug {
                            let mut m = [0u8; 32];
                            let tag: &[u8] = if serve_len > 0 {
                                b"RX served n="
                            } else if relay {
                                b"RX relay n="
                            } else if deliver {
                                b"RX deliver n="
                            } else {
                                b"RX dropped n="
                            };
                            let k = fmt_kv(&mut m, tag, n as u32);
                            send(&mut uart, &mut out, serial::EVT_LOG, &m[..k]).await;
                        }

                        let mut left_rx = false;
                        if serve_len > 0 {
                            // Content-Store hit: we answer the Interest ourselves, the host never wakes.
                            let _ =
                                lbt_tx(
                                    &mut radio,
                                    &state,
                                    &mut sense,
                                    &cap,
                                    &mut hoptrace,
                                    &serve[..serve_len],
                                    quiet,
                                )
                                .await;
                            left_rx = true;
                        }
                        if relay {
                            let _ = lbt_tx(
                                &mut radio,
                                &state,
                                &mut sense,
                                &cap,
                                &mut hoptrace,
                                payload,
                                quiet,
                            )
                            .await;
                            left_rx = true;
                        }
                        if left_rx {
                            let _ = phy_link::arm_rx(&mut radio, &state).await;
                        }

                        if deliver {
                            let mut body = [0u8; 8 + phy::MAX_PAYLOAD_ANY];
                            body[..2].copy_from_slice(&rssi.to_be_bytes());
                            body[2..4].copy_from_slice(&snr.to_be_bytes());
                            body[4..8].copy_from_slice(&ts.to_be_bytes());
                            body[8..8 + n].copy_from_slice(payload);
                            send(&mut uart, &mut out, serial::EVT_RX, &body[..8 + n]).await;
                        }
                    }
                }
            }
        }

        // ── background channel sensing, WITHOUT leaving RX ─────────────────────────────────────
        //
        // `set_and_get_cca` needs Standby or FS, so a periodic CCA would make the node deaf on a
        // duty cycle — on a bearer whose frames are ~150 µs long. `get_rssi_inst` is a measurement
        // command that works while listening, so the free-running activity counter is built from
        // energy detect and the gated CCA is kept for the host-initiated `CMD_SENSE`/`CMD_CAD` and
        // for LBT, which is leaving RX anyway. Same front end, same threshold, same units.
        if Instant::now() >= next_sense {
            next_sense = Instant::now() + Duration::from_millis(SENSE_PERIOD_MS);
            if let Ok(raw) = radio.get_rssi_inst().await {
                let d = dbm_of(raw);
                if d > sense.thresh_dbm {
                    sense.activity = sense.activity.wrapping_add(1);
                } else {
                    // An idle sample IS the noise floor, and it is what the EVT_RX SNR field is
                    // measured against.
                    sense.noise_floor_dbm = d;
                }
            }
        }

        // ── optional heartbeat beacon (CMD_SET_BEACON) ─────────────────────────────────────────
        //
        // Timed off `Instant`, not off a loop-iteration count: this loop's period is set by whatever
        // SPI happens to be in flight, so an iteration count would make the beacon period drift with
        // traffic — and a node whose beacon interval changes under load is useless as a reference
        // for anyone measuring against it. Sent WITHOUT listen-before-talk, on purpose: this is a
        // discovery heartbeat, and the LoRa work measured LBT hurting at N=2 on a clean channel.
        if beacon_on && Instant::now() >= next_beacon {
            next_beacon = Instant::now() + Duration::from_millis(beacon_period_ms);
            let mut msg = [0u8; 17];
            msg[..13].copy_from_slice(b"LR2021-BEACON");
            msg[13..].copy_from_slice(&beacon_seq.to_be_bytes());
            let ok = tx_once(&mut radio, &state, &cap, &mut hoptrace, &msg).await;
            // A transmit drops continuous RX; re-arm or the node goes deaf after its first beacon.
            let _ = phy_link::arm_rx(&mut radio, &state).await;
            n_tx = n_tx.wrapping_add(1);
            let mut m = [0u8; 32];
            let n = fmt_kv(&mut m, if ok { b"BCN seq=" } else { b"BCN FAILED seq=" }, beacon_seq);
            log(&mut uart, &mut out, debug, &m[..n]).await;
            beacon_seq = beacon_seq.wrapping_add(1);
        }
    }
}
