//! The **7E-A5 host protocol, v3** — the same wire contract the Waveshare (SX1262) and Heltec
//! (SX1276) nodes speak, so one host driver talks to every sub-GHz/2.4 GHz node in the rig.
//!
//! ```text
//!   0x7E 0xA5 <type> <len> <payload…> <crc>       crc = XOR of type, len and every payload byte
//! ```
//!
//! Deliberately identical to `waveshare-lora-rs`. A second, subtly different framing would not fail
//! loudly — the host would simply mis-parse one node — so the constants below are copied verbatim
//! and **any change must be made in both places**.
//!
//! ## What v2 adds, and why
//!
//! v1 let a host talk to a node it already knew everything about. That does not survive a fleet of
//! five nodes on three different radio parts: the host had to hard-code which knobs each node has,
//! which units they are in, and how good its timestamps are — and every one of those assumptions
//! was wrong for at least one node. v2 adds [`CMD_GET_CAP`]/[`EVT_CAP`], where a node states its
//! own capabilities in fields with **fixed units**, plus the three primitives the MAC work needs
//! from every node ([`CMD_READ_CLOCK`], [`CMD_SENSE`], [`CMD_TX_AT`]).
//!
//! **Every field of [`EVT_CAP`] must be true for this node.** Where nothing is known the field is 0
//! and the reason is in a comment — a fabricated number is worse than a zero, because the host will
//! believe it and calibrate against it.
//!
//! ## Never silence
//!
//! Any opcode this node does not implement is answered [`EVT_UNSUPPORTED`] `[cmd, reason]`. A host
//! that assumes a knob gets an error instead of a four-attempt retry loop that ends in a timeout,
//! which is the difference between a diagnosable bug and a mystery. The same rule governs
//! *arguments*: where a fleet command carries a field this radio has no equivalent for — the LoRa
//! CAD detector thresholds, a one-byte syncword — the node refuses rather than reinterpreting it,
//! because a knob that silently means something else is worse than one that is missing.
//!
//! Three opcodes stay refused ([`CMD_SET_SYNC`], [`CMD_SF_SCAN`], [`CMD_ENTER_BOOTLOADER`]) and each
//! is [`REASON_NO_HARDWARE`]: understood, and unreachable from firmware.
//!
//! ## What v3 adds: **modulation is a knob, not an identity**
//!
//! v2 encoded this node's modulation in its [`radio_kind`]: `2` meant "LR2021 running FLRC" and `3`
//! was reserved for "LR2021 running LoRa", as though a node that changed modulation became a
//! different part. It does not. `SetPacketType` (datasheet Table 8-1) is a **runtime command with
//! 14 modes**, and the fleet's other radios are the same shape — the SX1262 does LoRa and GFSK, the
//! SX1276 does LoRa, FSK and OOK. So v3:
//!
//! * redefines [`radio_kind`] to name the **part** (`2` = LR2021, whatever it is running) and
//!   retires the v2 value `3`;
//! * adds [`CMD_SET_PHY`], which takes a `SetPacketType` value and **replies with the whole new
//!   [`EVT_CAP`]**;
//! * adds `phy_bitmap` / `phy_current` to [`EVT_CAP`], so the host learns which modes this node
//!   really brings up rather than inferring from a kind byte;
//! * adds [`EVT_PHY_ERR`] for a mode the node advertises and the **chip** then refuses, carrying the
//!   chip's literal status byte rather than a firmware opinion about it.
//!
//! ★ **[`EVT_CAP`] describes the CURRENT PHY, not the part's union of everything it could do.**
//! `max_payload`, `sf_min`/`sf_max`, the rate model and `sched_gran_ns` are all per-PHY: this chip in
//! FLRC carries 47 bytes and has no spreading factor, and the same chip in LoRa carries 247 and has
//! SF7..SF12. That is why [`CMD_SET_PHY`] answers with a full `EVT_CAP` and the host must **replace**
//! its profile wholesale. Patching individual fields is how a stale `sf_max` survives a PHY switch,
//! which is the exact class of bug this version exists to remove.
//!
//! v3 also adds [`CMD_SET_HOP`] (intra-packet frequency hopping, LoRa and LR-FHSS) and
//! [`CMD_TX_AT_ABS`] (schedule against an absolute instant on the node's own clock, so host serial
//! latency cannot move the frame).
//!
//! ## [`CMD_GET_HOPTRACE`] — a node timestamps its OWN hops
//!
//! The last addition, and the one whose shape is dictated by a broken link: an LR2021 and an SX1276
//! both do intra-packet hopping, each interoperates with its own kind, and they cannot hop with each
//! other — so the link that would carry a cross-vendor comparison is the very thing being measured.
//! [`CMD_GET_HOPTRACE`]/[`EVT_HOPTRACE`] therefore has each node report **its own** hop events on
//! **its own** clock, in its own units, and the host does the comparison. See [`crate::hoptrace`].
//!
//! ⚠ Its opcode is **0x20 = 32**, one past the end of the 32-bit [`CMD_BITMAP`] field, so it is the
//! first command in this protocol that cannot be advertised and must be **probed** for. That is a
//! property of the fleet's wire format rather than an oversight; see [`CMD_BITMAP_FULL`].
//!
//! ## Back-compatibility, in both directions
//!
//! A v2 node still parses here: [`Capabilities::from_bytes`] accepts the 29-byte body and
//! synthesises a single-entry `phy_bitmap` plus a `phy_current` inferred from the v2 `radio_kind`
//! (see [`phy_of_v2_radio_kind`]). A v2 *host* reading this node's 34-byte `EVT_CAP` reads the first
//! 29 bytes unchanged and gets `radio_kind = 2`, which is still a sane kind for an LR2021.

/// Frame start, byte 0.
pub const SYNC0: u8 = 0x7E;
/// Frame start, byte 1.
pub const SYNC1: u8 = 0xA5;

/// Protocol version reported in [`EVT_CAP`]. **3** — see the module note on what v3 adds.
pub const PROTO_VER: u8 = 3;

/// The [`EVT_CAP`] body length a v2 node emits, and the shortest body [`Capabilities::from_bytes`]
/// will accept. Kept as a named constant because it is a *contract with older firmware*, not an
/// arbitrary offset.
pub const CAP_LEN_V2: usize = 29;

// ── Host → node ────────────────────────────────────────────────────────────────────────────────
/// payload = frame bytes to transmit; replies [`EVT_TXDONE`] `[ok, 0]`.
///
/// `ok` means **the chip reported TxDone**, not "the SPI writes succeeded" — the difference matters,
/// because the bug that motivated most of this file made every transmit after a retune fail while
/// still reporting success. A payload longer than [`Capabilities::max_payload`] is refused with
/// `ok = 0` and never truncated: a silently shortened NDN packet is worse than a refused transmit.
pub const CMD_TX: u8 = 0x01;
/// payload = u32 BE Hz; replies [`EVT_INFO`], or [`EVT_UNSUPPORTED`] if out of the built band.
pub const CMD_SET_FREQ: u8 = 0x02;
/// payload = `[sf, bw_code, cr_code]` on a LoRa node.
///
/// **On this node (`radio_kind` = 2, FLRC) the triple is `[rate_code, _, cr_code]`**, because FLRC
/// has no spreading factor and its bandwidth is not an independent knob — it is implied by the
/// bitrate rung. The byte positions are kept so one host code path drives every node; what each
/// slot *means* is what `radio_kind` tells the host.
///
/// * `rate_code` 0..7 → `FlrcBitrate::{Br2600, Br2080, Br1300, Br1040, Br0650, Br0520, Br0325,
///   Br0260}` — the chip's own encoding (`vendor/lr2021/src/cmd/cmd_flrc.rs`), not a re-mapping.
/// * `bw_code` ignored; echoed back as 0 in [`EVT_INFO`].
/// * `cr_code` 0..3 → `FlrcCr::{Cr12, Cr34, None, Cr23}` — again the chip's own encoding, so
///   `1` is 3/4 and `2` is FEC OFF. Counter-intuitive, and it is what the silicon uses.
///
/// Replies [`EVT_INFO`]; a bad code replies [`EVT_UNSUPPORTED`] `[cmd, REASON_OUT_OF_RANGE]`.
pub const CMD_SET_MOD: u8 = 0x03;
/// payload = `[i8 dBm]` — **real dBm**, converted to the chip's half-dB register unit by the node
/// and clamped to the PA's range. Replies [`EVT_INFO`] carrying the value actually applied.
pub const CMD_SET_PWR: u8 = 0x04;
/// payload = `[sync byte]` — **refused here**, [`EVT_UNSUPPORTED`] `[cmd, REASON_NO_HARDWARE]`.
///
/// The wire carries **one byte** (an SX127x sync word) and FLRC's syncword is **32 bits**. There is
/// no faithful mapping, and inventing one is the most dangerous thing this file could do: two nodes
/// that disagree about a syncword do not error, they simply never hear each other, which is
/// indistinguishable from a dead radio. A byte→word expansion would also have to be byte-identical
/// in the host, the Waveshare and the Heltec to be worth anything, and it would still leave the
/// other 24 bits unaddressable. `NO_HARDWARE` rather than `UNKNOWN_OPCODE` because no amount of
/// firmware fixes it — see [`crate::flrc_link::SYNCWORD`] for why the value's correlation properties
/// are an RF parameter and not a host's to pick a byte of.
pub const CMD_SET_SYNC: u8 = 0x05;
/// payload = []; replies [`EVT_INFO`].
pub const CMD_GET_INFO: u8 = 0x06;
/// payload = `[enabled]` or `[enabled, period_mult]`; replies [`EVT_INFO`].
///
/// A periodic self-transmit, same semantics as the Waveshare's: **default OFF**, so a node with no
/// host attached is silent and cannot pollute somebody else's measurement. `period_mult` scales the
/// base period (min ×1).
pub const CMD_SET_BEACON: u8 = 0x07;
/// payload = []; replies [`EVT_CAD`] `[busy]`. On this node the sense is an **energy-detect CCA**
/// (the LR2021 has no LoRa CAD in FLRC mode), thresholded at the `CMD_SET_SENSE_CFG` level.
pub const CMD_CAD: u8 = 0x08;
/// payload = []; replies [`EVT_RSSI`] `[rssi i16 BE dBm]`.
pub const CMD_GET_RSSI: u8 = 0x09;
/// payload = `[sym, det_peak, det_min]`; replies [`EVT_INFO`].
///
/// **Only `sym` maps.** On a LoRa node the triple is the CAD correlator's listen length and its
/// peak/min detector thresholds; this part has no LoRa correlator, its sense is an RSSI comparison,
/// so:
///
/// * `sym` 0..4 → the sense window, `1/2/4/8/16 ×` the base CCA window, preserving the field's
///   "how long to listen" meaning. Anything above 4 is [`REASON_OUT_OF_RANGE`], matching the LoRa
///   nodes' code space.
/// * `det_peak`, `det_min` — **must be 0**, else [`REASON_OUT_OF_RANGE`]. A correlator ratio
///   reinterpreted as a dBm threshold would silently read a busy channel as idle; the threshold on
///   this node is [`CMD_SET_SENSE_CFG`]'s, in real dBm, and one knob with one unit beats two that
///   disagree.
pub const CMD_SET_CAD_CFG: u8 = 0x0A;
/// payload = `[cw_ms u16 BE, max_backoff, max_attempts]`; replies [`EVT_INFO`].
pub const CMD_SET_LBT_CFG: u8 = 0x0B;
/// payload = `[preamble u16 BE]`; replies [`EVT_INFO`].
///
/// The fleet's field is a LoRa **symbol** count and FLRC has no symbols, so the value is taken as
/// the AGC preamble in **bits** — this PHY's own unit — rounded UP to the register's 4-bit step and
/// accepted only in 4..32. Outside that the register cannot reach it and the answer is
/// [`REASON_OUT_OF_RANGE`] rather than a clamp, because a host asking for LoRa's typical 8-symbol
/// preamble means something this radio cannot do and should be told so.
///
/// ⚠ Both ends must move together: the receiver's AGC is sized by this. See
/// [`crate::flrc_link::LinkState::preamble`].
pub const CMD_SET_PREAMBLE: u8 = 0x0C;
/// payload = []; **refused here**, [`EVT_UNSUPPORTED`] `[cmd, REASON_NO_HARDWARE]` — FLRC has no
/// spreading factor to scan for.
pub const CMD_SF_SCAN: u8 = 0x0D;
/// payload = frame bytes; atomic sense → randomised backoff → key-up. Replies
/// [`EVT_TXDONE`] `[sent, attempts]`.
pub const CMD_TX_LBT: u8 = 0x0E;
/// payload = [u64 BE hash]* — empty clears (pass-all); replies [`EVT_INFO`].
pub const CMD_SET_NAME_FILTER: u8 = 0x0F;
/// payload = [u64 BE hash]* — the relay set; empty clears. Replies [`EVT_INFO`].
pub const CMD_SET_RELAY: u8 = 0x10;
/// payload = `[cs_serve, dedup, hop_on, hop_base_ch, hop_span]`; replies [`EVT_INFO`].
/// `hop_on != 0` is rejected here — see the bridge, this bearer has no channel-index convention.
pub const CMD_DATAPLANE: u8 = 0x11;
/// payload = `[rssi_thresh i16 BE dBm, cad_repeat]`; replies [`EVT_INFO`].
pub const CMD_SET_SENSE_CFG: u8 = 0x12;
/// payload = []; replies [`EVT_STATS`].
pub const CMD_GET_STATS: u8 = 0x13;
/// payload = [] — clear every counter; replies [`EVT_INFO`].
pub const CMD_RESET_STATS: u8 = 0x14;
/// payload = `[on]`; replies [`EVT_INFO`]. Toggles the [`EVT_LOG`] diagnostic stream — off by
/// default, so a quiet link stays quiet and the UART is not competing with `EVT_RX` for airtime it
/// does not have.
pub const CMD_SET_DEBUG: u8 = 0x15;
/// payload = guard bytes; **refused here**, [`EVT_UNSUPPORTED`] `[cmd, REASON_NO_HARDWARE]`. The
/// XIAO is reflashed over its own onboard CMSIS-DAP probe; there is no ROM serial loader to jump to,
/// and pretending otherwise would leave the host waiting for a device that never re-enumerates.
pub const CMD_ENTER_BOOTLOADER: u8 = 0x16;
/// **v2.** payload = []; replies [`EVT_CLOCK`] `[ticks u64 BE]` on the same counter [`EVT_RX`]
/// stamps with, so a host can relate an RX stamp to "now" on one timebase.
pub const CMD_READ_CLOCK: u8 = 0x17;
/// **v2.** payload = `[delay_us u32 BE][frame]` — scheduled transmit; emits [`EVT_TX_STARTED`] at
/// key-up and replies [`EVT_TXDONE`] when the frame has actually aired.
///
/// **CPU-mediated, not DPPI**, and [`EVT_CAP`]`.sched_gran_ns` says so
/// ([`crate::airtime::SCHED_GRAN_NS`], 50 µs). The DPPI path `m5_tx` demonstrates needs LR2021 DIO8
/// as a `TxTrigger` **input**, and that pin is simultaneously the hardware RX stamp's capture
/// source; the stamp is this node's highest-value capability and is not being traded away. So the
/// firmware waits on the **same free-running counter** [`CMD_READ_CLOCK`] and [`EVT_RX`]`.ts` use,
/// then issues `SetTx` over SPI. Looser, real, and declared.
///
/// `delay_us` = 0 means transmit now. A delay whose instant has already passed by the time the FIFO
/// is loaded also transmits immediately — never a wait for the 32-bit counter to come round. Staging
/// the frame costs ~600 µs (PLL settle + FIFO write), so that is the shortest delay this node can
/// actually place; below it the transmit is "as soon as possible", not "at that instant". See the
/// bridge for the upper bound and why it exists.
pub const CMD_TX_AT: u8 = 0x18;
/// **v2.** payload = []; replies [`EVT_CAP`].
pub const CMD_GET_CAP: u8 = 0x1A;
/// **v2.** payload = []; replies [`EVT_SENSE`].
pub const CMD_SENSE: u8 = 0x1B;
/// payload = `[0 = automatic AGC | 1 = boosted]`; replies [`EVT_INFO`].
///
/// The Waveshare defines this byte as power-saving/boosted LNA, so those are the only two values
/// this node accepts, even though the LR2021's `set_rx_gain` takes a 0..13 ladder. Exposing the
/// ladder through the same byte would make `1` mean "boosted" on one node and "gain step 1" — the
/// **lowest** manual gain — on another: an inversion, which is a failure mode this rig has already
/// paid for once on a TX-power knob. Anything but 0 or 1 is [`REASON_OUT_OF_RANGE`].
pub const CMD_SET_RX_GAIN: u8 = 0x1C;
/// **v3.** payload = `[packet_type u8]` — the chip's own `SetPacketType` value ([`phy_code`]).
/// Replies **the full new [`EVT_CAP`]**, because everything in it is per-PHY.
///
/// * a value outside [`Capabilities::phy_bitmap`] → [`EVT_UNSUPPORTED`] `[cmd, REASON_OUT_OF_RANGE]`
///   (the node knows the mode exists and knows this build does not bring it up);
/// * a value the node *advertises* whose **bring-up** the chip refuses → [`EVT_PHY_ERR`]
///   `[requested, chip_status]` **and no `EVT_CAP`**, with the node back on the PHY it was running:
///   a failed switch must not leave the radio half-programmed, and the host's profile is still
///   correct because nothing moved.
///
/// ★ There is a **third** outcome, and it is the one LR-FHSS exists here to expose: the mode comes
/// up and the chip then refuses to **arm RX**. That is a finding about reception, not a failed
/// switch — the node transmits perfectly well in it — so the node *stays* in the new PHY and emits
/// the full [`EVT_CAP`] **first**, then [`EVT_PHY_ERR`]. The order is load-bearing: a host blocked
/// on the reply treats an `EVT_PHY_ERR` that arrives first as a terminal refusal of the whole
/// command, which would leave it holding the previous mode's `max_payload` and rate model while
/// this node ran the new one.
///
/// So an `EVT_PHY_ERR` **preceded by an `EVT_CAP`** means "you are in the mode you asked for, and
/// here is what the silicon said about its receiver"; an `EVT_PHY_ERR` **alone** means "you are
/// still in the mode you were in".
///
/// The host must **replace** its whole node profile from the reply, never patch fields: see the
/// module note.
pub const CMD_SET_PHY: u8 = 0x1D;
/// **v3.** payload = `[hop_ctrl u8][hop_period u16 BE][n u8][freq_hz u32 BE]*n`, `n <= 40`; replies
/// [`EVT_INFO`].
///
/// **Intra-packet** frequency hopping — the carrier moves *inside* one frame, on a table the host
/// writes. Not a channel-hopping schedule between frames.
///
/// * `hop_ctrl` bit 0 = enable. Every other bit is reserved and must be 0.
/// * `hop_period` is the dwell in **symbols** (LoRa `SetLoraHopping`) or in symbols per hop block
///   (LR-FHSS hopping table). 0 with hopping enabled is [`REASON_OUT_OF_RANGE`].
/// * `n` = 0 with `hop_ctrl` = 0 disables hopping. `n > 40` is refused — 40 is the chip's table
///   depth for both mechanisms.
///
/// Valid only in a PHY that **has** intra-packet hopping. On this node that is LoRa and LR-FHSS;
/// asking for it in FLRC is [`REASON_OUT_OF_RANGE`] (the current PHY is the out-of-range argument),
/// never a silent no-op.
pub const CMD_SET_HOP: u8 = 0x1E;
/// **v3.** payload = `[target_ticks u64 BE][frame bytes]`; replies [`EVT_TXDONE`], with
/// [`EVT_TX_STARTED`] emitted at acceptance exactly as [`CMD_TX_AT`] does.
///
/// ★ **The absolute-target transmit, and the reason it exists.** [`CMD_TX_AT`]'s delay is counted
/// from the moment the *firmware* processes the arm, so the host→device serial latency lands inside
/// the placement. Measured on this node: an absolute-boundary slot train fired 45/45 with a mean gap
/// of 2,399,818 ticks against 2,400,000 nominal (11 µs of accuracy error over 44 slots) but a jitter
/// **sd of 553 µs and p2p 1875 µs** — against a declared 50 µs `sched_gran_ns`. The node's
/// `CMD_GET_INFO` round trip is p2p **550 µs**: the same number. As exercised, host-armed *relative*
/// scheduling is worse than the software path.
///
/// `target_ticks` is on the **same free-running counter** [`CMD_READ_CLOCK`] reports and [`EVT_RX`]
/// stamps with, so the host names an instant instead of a delay and its own serial latency cannot
/// move the frame. A target already past transmits immediately — never a wait for the counter to
/// come round. A target further ahead than the node's scheduling bound is
/// [`REASON_OUT_OF_RANGE`].
pub const CMD_TX_AT_ABS: u8 = 0x1F;
/// **v3.** payload = []; replies [`EVT_HOPTRACE`], or [`EVT_UNSUPPORTED`] `[0x20,
/// REASON_NO_HARDWARE]` on a node that cannot timestamp its own hops.
///
/// ★ **The instrument for "when does a hop boundary fall?", and the reason it needs no cross-vendor
/// link.** Each node timestamps its **own** hop events on its **own** clock, so an LR2021 and an
/// SX1276 can be compared without either having to receive the other — which matters here because
/// the link that would carry the comparison is the very thing that is broken. See
/// [`crate::hoptrace`] for the measurement this settles and for the exact latency between the RF hop
/// boundary and the stamp.
///
/// The ring is **free-running and wraps; reading does NOT clear it**, the same contract as
/// [`EVT_SENSE`]`.activity` — so two reads can be differenced and a second reader cannot destroy the
/// first's view. A node with hopping off answers `n = 0` and never a stale timeline.
///
/// ⚠ **This opcode is 0x20 = 32 and therefore CANNOT appear in [`CMD_BITMAP`]**, whose wire field is
/// four bytes fleet-wide (`EVT_CAP[19..23]`) and covers opcodes 0..31 only. It is the first opcode
/// past the end of that space. Widening the field would break every node and host in the fleet at
/// once, so discovery is by **probe** instead: send it, and get either `EVT_HOPTRACE` or
/// `EVT_UNSUPPORTED`. [`CMD_BITMAP_FULL`] carries the bit for firmware-side reasoning; the wire
/// field is its low 32 bits by construction.
pub const CMD_GET_HOPTRACE: u8 = 0x20;

// ── Node → host ────────────────────────────────────────────────────────────────────────────────
/// payload = `[rssi i16 BE dBm, snr i16 BE dB, ts u32 BE, frame bytes]`.
///
/// **`ts` is TICKS, not microseconds** — at [`EVT_CAP`]`.stamp_hz` (16 MHz here, so 62.5 ns per
/// tick), latched by DPPI at the DIO edge. The field was documented as `ts_us` while carrying
/// ticks, which is a 16× error waiting to happen in whichever host reads it first; the wire layout
/// is unchanged, only the documentation was wrong. The Waveshare node fills the same field with a
/// millisecond software counter and reports `stamp_hz = 1000`, which is exactly why the units have
/// to travel in [`EVT_CAP`] rather than being assumed.
pub const EVT_RX: u8 = 0x81;
/// payload = `[ok, attempts]`. `attempts` is 0 for a plain [`CMD_TX`].
pub const EVT_TXDONE: u8 = 0x82;
/// payload, **19 bytes**, matching the fleet:
/// `[status, sync(2), errors(2), freq(4), sf, bw, cr, pwr, lost(2), cad_busy(2), defer(2)]`.
///
/// Per-field meaning on this node — see [`CMD_SET_MOD`] for why the `sf`/`bw`/`cr` slots carry a
/// bitrate rung rather than a spreading factor:
///
/// | field | here |
/// |---|---|
/// | `status` | `chip_mode | (cmd_status << 4)` from the LR2021 status word |
/// | `sync` | low 16 bits of the 32-bit FLRC syncword |
/// | `errors` | the chip's sticky error flags, packed — see [`err_bits`] |
/// | `freq` | current carrier, Hz |
/// | `sf` | FLRC bitrate code 0..7 (**not** a spreading factor; there is none) |
/// | `bw` | 0 — FLRC bandwidth is implied by the bitrate rung, not an independent knob |
/// | `cr` | `FlrcCr` code 0..3 |
/// | `pwr` | TX power **actually applied**, in dBm |
/// | `lost` | 0 — the buffered UARTE ring exposes no overrun count on this MCU |
/// | `cad_busy` | senses that came back busy |
/// | `defer` | transmissions abandoned after `max_attempts` |
pub const EVT_INFO: u8 = 0x83;
/// payload = ascii. Emitted only while [`CMD_SET_DEBUG`] is on.
pub const EVT_LOG: u8 = 0x84;
/// payload = `[busy(0/1)]`.
pub const EVT_CAD: u8 = 0x85;
/// payload = `[rssi i16 BE dBm]`.
pub const EVT_RSSI: u8 = 0x86;
/// payload = `[sf | 0]`. **Fleet number, unimplemented here** — FLRC has no spreading factor.
/// Listed so nobody re-uses 0x87 for something else, which is exactly what happened once: this
/// firmware defined `EVT_STATS = 0x87` while the host mapped 0x89 to it and 0x87 to SF-detected, so
/// a stats read on this node decoded as an SF scan on the host.
pub const EVT_SF_DETECTED: u8 = 0x87;
/// payload = `[airtime_ms u16 BE]`, emitted immediately **before key-up** on [`CMD_TX`],
/// [`CMD_TX_LBT`] and [`CMD_TX_AT`], so a host can re-base its deadline on every transmit path.
///
/// The airtime is computed from the live link, not hardcoded — see
/// [`crate::flrc_link::LinkState::airtime_us`], whose terms `CMD_SET_MOD` and `CMD_SET_PREAMBLE`
/// both move. It is **rounded up, and never 0**: an FLRC frame is sub-millisecond at every rung
/// above 260 kbit/s, so the fleet's millisecond field cannot express this bearer and the only safe
/// direction to lose that precision is upwards. See [`crate::airtime::airtime_ms_ceil`].
pub const EVT_TX_STARTED: u8 = 0x88;
/// payload, **24 bytes**:
/// `[rx(4), filtered(4), deduped(4), served(4), relayed(4), cad_busy(2), defer(2)]`, all BE.
///
/// `rx` counts frames the on-device data plane **classified** — a frame that is not in the app wire
/// shape (`KIND|SRC|SF|NAME|…`) is delivered without being counted, exactly as on the Waveshare
/// node. One meaning per field across the fleet matters more than a second, node-local definition.
pub const EVT_STATS: u8 = 0x89;
/// **v2.** payload = `[ticks u64 BE]`, units = [`EVT_CAP`]`.stamp_hz`.
pub const EVT_CLOCK: u8 = 0x8A;
/// **v2, grown in v3.** payload = [`CAP_LEN`] = **34** bytes, see [`Capabilities`]. A v2 node emits
/// [`CAP_LEN_V2`] = 29 and [`Capabilities::from_bytes`] still accepts that.
pub const EVT_CAP: u8 = 0x8B;
/// **v2.** payload = `[activity u16 BE, rssi i16 BE dBm]`.
///
/// `activity` is a **free-running, wrapping** count of channel-busy observations. The host
/// differences two reads over a window; it is never an absolute occupancy. Wrapping is deliberate:
/// a saturating counter silently stops measuring, and a host differencing a stuck counter reads a
/// busy channel as idle.
pub const EVT_SENSE: u8 = 0x8C;
/// **v3.** payload = `[requested_phy u8, chip_status u8]` — a PHY this node **advertises** in
/// [`Capabilities::phy_bitmap`] that the **chip** refused at runtime.
///
/// Distinct from [`EVT_UNSUPPORTED`] on purpose. `EVT_UNSUPPORTED` is the firmware saying "I do not
/// offer that"; this is the firmware saying "I offer it, I asked the silicon, and the silicon said
/// no — here is what it said". `chip_status` is the LR2021 status byte packed exactly as
/// [`EVT_INFO`]`[0]`: `chip_mode | (cmd_status << 4)`, where `cmd_status` is 0 = Fail, 1 = ParamErr,
/// 2 = Ok, 3 = Data. One packing for the chip's status across this whole protocol, so a host does
/// not need a second decoder for one event.
///
/// It carries the chip's answer verbatim because the alternative — a firmware-invented reason code
/// — is exactly the kind of plausible fiction that gets calibrated against. The LR-FHSS RX arm is
/// the case this was added for: the datasheet contradicts itself about whether LR-FHSS can receive
/// at all (§17.1 "transmit-only mode" vs §17.2.2's syncword "for detection on the receiver side"),
/// so the node asks the chip and forwards the answer instead of pre-judging it.
pub const EVT_PHY_ERR: u8 = 0x8D;
/// **v3.** payload = `[stamp_hz u32 BE][n u8][ (idx u8, t_ticks u32 BE) ]*n`, `n <= 32` — this
/// node's own hop timeline, encoded by [`crate::hoptrace::HopTrace::encode`].
///
/// `stamp_hz` is **this node's clock in its own units** — the same one [`CMD_READ_CLOCK`] returns
/// and [`EVT_RX`]`.ts` uses (16 MHz here, 62.5 ns per tick). It is deliberately **not** converted to
/// microseconds in firmware: the host divides, and converting here would throw away 16× of
/// resolution on this part purely to match the Heltec's 1 MHz counter. Same reasoning as the `ts`
/// field, and the same trap — a field documented in one unit and filled in another is a silent
/// scale error in whichever host reads it first.
///
/// `idx` is the hop-list index the event moved **to**. On this node it is a **firmware counter**,
/// not a chip readback (the SX1276's `RegHopChannel` has no LR2021 equivalent) — see
/// [`crate::hoptrace::HopTrace::push_hop`]. Bit 7 ([`crate::hoptrace::IDX_TX_KEYED`]) marks a
/// **transmit key-up** rather than a hop, which is how a host learns where in the sequence a frame
/// started; the chip's table is 40 deep, so that bit cannot occur on a real index.
///
/// `n = 0` is the honest answer from a node whose hopping is off, and `stamp_hz` still travels so
/// the units are learned either way.
pub const EVT_HOPTRACE: u8 = 0x8E;
/// payload = `[cmd, reason]` — see [`REASON_UNKNOWN_OPCODE`] and friends.
pub const EVT_UNSUPPORTED: u8 = 0x8F;
/// **Not emitted by this firmware — reserved fleet-wide, and listed here because this file is where
/// the fleet's event numbering is written down and pinned.**
///
/// `[frame_stamp_kind u8, reason u8]`, emitted by the Waveshare node immediately before an `EVT_RX`
/// whose `ts` is not the hardware capture its `EVT_CAP` advertises. It is a *per-frame* qualifier on
/// a *node-level* capability byte, which is a shape this firmware may well want later (its own
/// `HwStamp` type makes the same distinction in the type system instead).
///
/// ☠ It was born at **0x8E**, which is [`EVT_HOPTRACE`] here and on the Heltec. Nothing caught it:
/// [`fleet_event_numbering`] listed only *this* node's constants, so a collision introduced in
/// another firmware could not fail it — precisely the failure that test was written to prevent, and
/// exactly how `EVT_STATS = 0x87` survived. The consequence was live rather than theoretical:
/// `tools/hoptrace.py` probes for hop support by sending `CMD_GET_HOPTRACE` (0x20, past the end of
/// `cmd_bitmap`) and taking the first `EVT_HOPTRACE` or `EVT_UNSUPPORTED` inside 3 s, so one
/// degraded frame from a Waveshare would have answered the probe with a "hop timeline" from a node
/// that structurally cannot hop. The constant lives here now so the registry covers the whole space.
pub const EVT_RX_STAMP: u8 = 0x90;

/// **`CMD_GET_CLOCK_REF` (0x21) → [`EVT_CLOCK_REF`] — what is the counter DERIVED FROM?**
///
/// `EVT_CAP.stamp_kind` says where a stamp is LATCHED and nothing about the oscillator underneath
/// it, and the two are independent: the Waveshare node scored ~16 us of common-view residual and
/// ~1.1 us with the SAME latch point, on an RC and then on a crystal. This node has the answer in
/// `hw::init_peripherals`, which pins `HfclkSource::ExternalXtal` precisely because the RC default
/// MEASURED ~+2000 ppm — so it should say so rather than leave a host to infer it.
pub const CMD_GET_CLOCK_REF: u8 = 0x21;
/// Reply to [`CMD_GET_CLOCK_REF`]: `[ref_class u8][accuracy_ppm u16 BE]`. Listed in the registry
/// below so the number is SPOKEN FOR — that test is the only thing standing between a new event and
/// a silent mis-decode on a host sharing one table across four firmwares.
pub const EVT_CLOCK_REF: u8 = 0x91;
/// [`EVT_CLOCK_REF`] `ref_class`: the node cannot say.
pub const CLOCK_REF_UNKNOWN: u8 = 0;
/// [`EVT_CLOCK_REF`] `ref_class`: an internal RC oscillator.
pub const CLOCK_REF_RC: u8 = 1;
/// [`EVT_CLOCK_REF`] `ref_class`: a crystal or TCXO.
pub const CLOCK_REF_XTAL: u8 = 2;
/// [`EVT_CLOCK_REF`] `accuracy_ppm` sentinel: **not measured**. This node's +16.7 ppm figure is an
/// inter-node comparison from one session, not an accuracy against a standard, and the fleet's rule
/// is that a number goes on the wire only when the node can stand behind it.
pub const CLOCK_ACCURACY_UNKNOWN: u16 = 0xFFFF;

// `EVT_UNSUPPORTED` reason codes. **Fleet-wide space** — these are the same four values the
// Waveshare and Heltec nodes use (`UNSUP_*` in their `main.rs`). They were 1/2/3 =
// not-implemented/param/hardware here, which put `NO_HARDWARE` and `BAD_LENGTH` in each other's
// slots relative to the other two nodes: a host decoding the byte would have read this node's
// `CMD_TX_AT` refusal as "you sent a short payload". Nothing on the wire distinguishes the two
// conventions, so the majority spelling is the fleet's.
/// This firmware does not know the opcode at all.
pub const REASON_UNKNOWN_OPCODE: u8 = 1;
/// The opcode is understood and **this board's wiring cannot do it**. Distinct from
/// [`REASON_UNKNOWN_OPCODE`] because it is not a firmware gap: no amount of firmware fixes it.
pub const REASON_NO_HARDWARE: u8 = 2;
/// The opcode is understood but the payload does not satisfy its argument requirements.
pub const REASON_BAD_LENGTH: u8 = 3;
/// An argument outside the range [`EVT_CAP`] advertises — out of band, an unknown rate code.
pub const REASON_OUT_OF_RANGE: u8 = 4;

/// Largest payload accepted in either direction **on the serial link**. Not the on-air cap — that
/// is [`Capabilities::max_payload`], which is far smaller here.
pub const MAX_PAYLOAD: usize = 255;

/// Pack the LR2021's sticky error flags into the 16-bit `errors` field of [`EVT_INFO`].
///
/// Bit order is fixed here so a host decoding it does not have to know the driver's struct layout:
/// 0 hf_xosc, 1 lf_xosc, 2 pll_lock, 3 lf_rc_calib, 4 hf_rc_calib, 5 pll_calib, 6 aaf_calib,
/// 7 img_calib, 8 chip_busy, 9 rxfreq_no_fe_cal, 10 meas_unit_adc_calib, 11 pa_offset_calib,
/// 12 ppf_calib, 13 src_calib.
pub const fn err_bits(flags: [bool; 14]) -> u16 {
    let mut v = 0u16;
    let mut i = 0;
    while i < 14 {
        if flags[i] {
            v |= 1 << i;
        }
        i += 1;
    }
    v
}

// ── EVT_CAP ────────────────────────────────────────────────────────────────────────────────────

/// Wire length of [`EVT_CAP`]. **34 in v3** — the v2 body plus `phy_bitmap` and `phy_current`.
pub const CAP_LEN: usize = 34;

/// `radio_kind` values — **the PART, not the mode it is running.**
///
/// v2 spent two values on one chip (`2` = LR2021-FLRC, `3` = LR2021-LoRa) because the modulation was
/// treated as identity. It is not: `SetPacketType` is a runtime command, and a node that switches
/// from FLRC to LoRa is the same node with a different knob position. v3 therefore keeps `2` for the
/// whole part and retires `3`; what the chip is running now lives in
/// [`Capabilities::phy_current`], and what it *can* run lives in [`Capabilities::phy_bitmap`].
///
/// The retirement is backward-safe in the direction that matters: a v2 host reading a v3 LR2021
/// still sees `2`, which it already understands. See [`phy_of_v2_radio_kind`] for the other
/// direction.
pub mod radio_kind {
    pub const SX1262: u8 = 0;
    pub const SX1276: u8 = 1;
    /// The Semtech LR2021, in **whatever** mode `phy_current` names.
    pub const LR2021: u8 = 2;
    /// **Retired in v3.** v2 used it for "LR2021 running LoRa"; that is now `LR2021` with
    /// `phy_current = `[`phy_code::LORA`]. Kept as a named constant only so a v2 body can still be
    /// decoded — never emitted.
    pub const V2_LR2021_LORA: u8 = 3;
}

/// `SetPacketType` values — **the chip's own encoding**, datasheet Table 8-1, not a re-mapping.
///
/// These are the numbers `CMD_SET_PHY` carries, the bit positions of
/// [`Capabilities::phy_bitmap`], and the value of [`Capabilities::phy_current`]. Using the silicon's
/// own code space is the same decision `CMD_SET_MOD` already makes for the FLRC rate rungs: a second
/// numbering would have to be kept identical in five firmwares and one host, and nothing on the wire
/// would say which one a given byte was in.
///
/// Every value the part defines is listed, including the ones this firmware does not bring up —
/// `phy_bitmap` is what says which are reachable here, and a name is cheaper than a magic number in
/// a future commit.
pub mod phy_code {
    pub const LORA: u8 = 0x0;
    pub const FSK_GENERIC: u8 = 0x1;
    pub const FSK_LEGACY: u8 = 0x2;
    pub const BLE: u8 = 0x3;
    pub const RTTOF: u8 = 0x4;
    pub const FLRC: u8 = 0x5;
    pub const BPSK: u8 = 0x6;
    pub const LR_FHSS: u8 = 0x7;
    pub const WM_BUS: u8 = 0x8;
    pub const WISUN: u8 = 0x9;
    pub const OOK: u8 = 0xA;
    pub const RAW: u8 = 0xB;
    pub const ZWAVE: u8 = 0xC;
    pub const O_QPSK_15_4: u8 = 0xD;
}

/// The PHY a **v2** node was running, inferred from its `radio_kind` — the back-compat half of the
/// v2→v3 upgrade.
///
/// v2 had no `phy_current`, but its `radio_kind` did encode the mode for the one part where it
/// varied, so the information is recoverable exactly rather than guessed:
///
/// | v2 `radio_kind` | node | mode it could only have been running |
/// |---|---|---|
/// | 0 | SX1262 | LoRa |
/// | 1 | SX1276 | LoRa |
/// | 2 | LR2021 | FLRC |
/// | 3 | LR2021 | LoRa |
///
/// (The SX1262 and SX1276 both have an FSK mode, but no v2 firmware in this fleet ever selected it
/// — every one of them is a LoRa node — so `LORA` is a fact about the fleet, not an assumption
/// about the part.) An unknown kind yields `None`, and the caller must not invent one.
pub const fn phy_of_v2_radio_kind(kind: u8) -> Option<u8> {
    match kind {
        radio_kind::SX1262 | radio_kind::SX1276 => Some(phy_code::LORA),
        radio_kind::LR2021 => Some(phy_code::FLRC),
        radio_kind::V2_LR2021_LORA => Some(phy_code::LORA),
        _ => None,
    }
}

/// `stamp_kind` values — how good the [`EVT_RX`] `ts` field actually is.
pub mod stamp_kind {
    /// No per-frame stamp at all; `stamp_hz` is 0.
    pub const NONE: u8 = 0;
    /// Latched when the serial line delivered the frame to the host.
    pub const HOST_RECV: u8 = 1;
    /// A counter incremented by firmware — carries task/interrupt latency.
    pub const SOFTWARE_COUNTER: u8 = 2;
    /// Latched in silicon at the radio's own event edge, free-running counter.
    pub const HARDWARE_FREE_RUNNING: u8 = 3;
}

/// **Every opcode this firmware implements, one bit each — including the ones the wire field cannot
/// carry.** `bit N == opcode N`, 64 bits wide.
///
/// Derived from the constants above rather than written out, so the bitmap cannot drift from the
/// dispatch table the way a hand-maintained list does. **Anything added here must be answered by
/// the bridge**, and anything the bridge answers `EVT_UNSUPPORTED` must be absent.
///
/// ⚠ [`CMD_BITMAP`] — the value that actually reaches the host — is the **low 32 bits** of this, and
/// [`CMD_GET_HOPTRACE`] (0x20 = 32) falls off the end. That is a property of the fleet's wire
/// format, not an oversight: `EVT_CAP[19..23]` is four bytes on four firmwares and two host crates,
/// and widening it would break all of them simultaneously to advertise one diagnostic opcode. The
/// truncation is made explicit here, and pinned by a test, so it is a known boundary rather than a
/// bit that silently vanished.
pub const CMD_BITMAP_FULL: u64 = (CMD_BITMAP as u64) | (1u64 << CMD_GET_HOPTRACE);

/// The `cmd_bitmap` field of [`EVT_CAP`] — opcodes 0..31 only. See [`CMD_BITMAP_FULL`] for what
/// does not fit and why the field is not widened.
pub const CMD_BITMAP: u32 = (1 << CMD_TX)
    | (1 << CMD_SET_FREQ)
    | (1 << CMD_SET_MOD)
    | (1 << CMD_SET_PWR)
    | (1 << CMD_GET_INFO)
    | (1 << CMD_SET_BEACON)
    | (1 << CMD_CAD)
    | (1 << CMD_GET_RSSI)
    | (1 << CMD_SET_CAD_CFG)
    | (1 << CMD_SET_LBT_CFG)
    | (1 << CMD_SET_PREAMBLE)
    | (1 << CMD_TX_LBT)
    | (1 << CMD_SET_NAME_FILTER)
    | (1 << CMD_SET_RELAY)
    | (1 << CMD_DATAPLANE)
    | (1 << CMD_SET_SENSE_CFG)
    | (1 << CMD_GET_STATS)
    | (1 << CMD_RESET_STATS)
    | (1 << CMD_SET_DEBUG)
    | (1 << CMD_READ_CLOCK)
    | (1 << CMD_TX_AT)
    | (1 << CMD_GET_CAP)
    | (1 << CMD_SENSE)
    | (1 << CMD_SET_RX_GAIN)
    | (1 << CMD_SET_PHY)
    | (1 << CMD_SET_HOP)
    | (1 << CMD_TX_AT_ABS);
// Deliberately ABSENT, and each absence is answered EVT_UNSUPPORTED rather than ignored. All three
// are NO_HARDWARE, not UNKNOWN_OPCODE: the opcode is understood and no firmware change reaches it.
//   0x05 CMD_SET_SYNC        — a one-byte LoRa sync word does not map onto a 32-bit FLRC syncword,
//                              and a wrong syncword is silent, not loud
//   0x0D CMD_SF_SCAN         — no spreading factor exists to scan for
//   0x16 CMD_ENTER_BOOTLOADER— the XIAO reflashes over its own CMSIS-DAP probe, not a ROM loader
// And one opcode is absent for a DIFFERENT reason — it is implemented and answered, it simply does
// not fit:
//   0x20 CMD_GET_HOPTRACE   — bit 32 of a 32-bit field. Discovered by probing, not by the bitmap;
//                             see CMD_BITMAP_FULL.

/// ★ **[`CMD_BITMAP`] for the PHY the node is running** — the command surface is per-PHY too.
///
/// `cmd_bitmap` means *implemented and will act*. [`CMD_SET_HOP`] is implemented on this node and
/// will **not** act in FLRC: there is no FLRC hopping command on this part, so
/// [`crate::phy::check_hop`] refuses it with [`REASON_OUT_OF_RANGE`] on every call. Advertising the
/// bit there would hand a host a knob that can only ever fail — and the host believes this field
/// hard enough to build a whole `HopCapability` out of it and plan a hop schedule against it
/// (`lora_serial::NodeProfile::hop_capability`), so the bit has to move with the mode the way
/// `max_payload` and `sf_min`/`sf_max` already do.
///
/// This is the same rule the Waveshare node applies to its GFSK profile, where `SET_MOD`, `CAD`,
/// `SET_CAD_CFG` and `SF_SCAN` all drop out of the bitmap. It is one function of one boolean rather
/// than a second bitmap literal, so the two answers cannot drift.
pub const fn cmd_bitmap_for(has_intra_packet_hopping: bool) -> u32 {
    if has_intra_packet_hopping {
        CMD_BITMAP
    } else {
        CMD_BITMAP & !(1 << CMD_SET_HOP)
    }
}

/// A node's self-description — the one place a node states what it is, in fixed units.
///
/// Every multi-byte field is **big-endian** on the wire.
///
/// ```text
///   [0]      proto_ver
///   [1]      radio_kind
///   [2..6]   freq_min_hz  u32
///   [6..10]  freq_max_hz  u32
///   [10]     pwr_min_dbm  i8     ** real dBm, never a chip register unit **
///   [11]     pwr_max_dbm  i8
///   [12..16] stamp_hz     u32    ticks/second of the EVT_RX ts field; 0 = no per-frame stamp
///   [16]     stamp_kind
///   [17..19] max_payload  u16    the REAL end-to-end cap
///   [19..23] cmd_bitmap   u32    OPCODES 0..31 ONLY — see CMD_BITMAP_FULL
///   [23]     sf_min              0 when the node has no spreading factor
///   [24]     sf_max
///   [25..29] sched_gran_ns u32   0 = no hardware-scheduled TX
///   [29..33] phy_bitmap    u32   v3: bit N set == SetPacketType value N is usable on this node
///   [33]     phy_current   u8    v3: the SetPacketType value in effect right now
/// ```
///
/// Bytes `[0..29]` keep their v2 meaning **except** `[1] radio_kind`, which now names the part
/// rather than the part-and-mode; see [`radio_kind`].
///
/// ★ Everything here describes the **current** PHY. `max_payload`, `sf_min`/`sf_max` and
/// `sched_gran_ns` all move when [`CMD_SET_PHY`] moves, which is why that command replies with a
/// whole new body.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub struct Capabilities {
    pub proto_ver: u8,
    pub radio_kind: u8,
    pub freq_min_hz: u32,
    pub freq_max_hz: u32,
    pub pwr_min_dbm: i8,
    pub pwr_max_dbm: i8,
    pub stamp_hz: u32,
    pub stamp_kind: u8,
    /// The **real** end-to-end cap: `min(TX accept, RX buffer, on-air PDU)`. A node that accepts
    /// 240 bytes on TX and truncates RX at 64 reports 64 — the smaller side, always.
    pub max_payload: u16,
    /// One bit per implemented opcode, **and it can only ever describe opcodes 0..31** — the field
    /// is four bytes across the whole fleet. [`CMD_GET_HOPTRACE`] (0x20) is the first opcode past
    /// it and is discovered by probing instead; see [`CMD_BITMAP_FULL`].
    pub cmd_bitmap: u32,
    pub sf_min: u8,
    pub sf_max: u8,
    pub sched_gran_ns: u32,
    /// **v3.** `bit N` set means `SetPacketType` value `N` ([`phy_code`]) is a mode this node really
    /// brings up — not a mode the part is capable of on paper. A PHY absent here is refused by
    /// [`CMD_SET_PHY`] with [`REASON_OUT_OF_RANGE`]; a PHY present here that the chip then rejects
    /// yields [`EVT_PHY_ERR`].
    pub phy_bitmap: u32,
    /// **v3.** The `SetPacketType` value in effect right now. Always a set bit of `phy_bitmap`.
    pub phy_current: u8,
}

impl Capabilities {
    /// Serialise to the 34 wire bytes.
    pub fn to_bytes(&self) -> [u8; CAP_LEN] {
        let mut b = [0u8; CAP_LEN];
        b[0] = self.proto_ver;
        b[1] = self.radio_kind;
        b[2..6].copy_from_slice(&self.freq_min_hz.to_be_bytes());
        b[6..10].copy_from_slice(&self.freq_max_hz.to_be_bytes());
        b[10] = self.pwr_min_dbm as u8;
        b[11] = self.pwr_max_dbm as u8;
        b[12..16].copy_from_slice(&self.stamp_hz.to_be_bytes());
        b[16] = self.stamp_kind;
        b[17..19].copy_from_slice(&self.max_payload.to_be_bytes());
        b[19..23].copy_from_slice(&self.cmd_bitmap.to_be_bytes());
        b[23] = self.sf_min;
        b[24] = self.sf_max;
        b[25..29].copy_from_slice(&self.sched_gran_ns.to_be_bytes());
        b[29..33].copy_from_slice(&self.phy_bitmap.to_be_bytes());
        b[33] = self.phy_current;
        b
    }

    /// Parse an `EVT_CAP` body — **v3's 34 bytes or a v2 node's 29.**
    ///
    /// Exists so the encoder can be round-trip tested on the host, and so the v2 compatibility rule
    /// has one implementation with tests rather than a paragraph of prose each host re-derives.
    ///
    /// A 29-byte body has no `phy_bitmap` and no `phy_current`, and inventing a rich one would be
    /// the worst outcome — a host would think it could switch a v2 node's modulation. So the body is
    /// given the **single-entry** bitmap for the one mode that node is running, recovered from its
    /// `radio_kind` by [`phy_of_v2_radio_kind`]. `CMD_SET_PHY` is then absent from its `cmd_bitmap`
    /// anyway, so the host can see the mode and see that it cannot change it.
    ///
    /// A v2 body whose `radio_kind` is unknown is **rejected** rather than given an invented PHY: a
    /// wrong `phy_current` is a unit error waiting to happen, and `None` is a diagnosable answer.
    pub fn from_bytes(b: &[u8]) -> Option<Self> {
        if b.len() < CAP_LEN_V2 {
            return None;
        }
        let u32be = |o: usize| u32::from_be_bytes([b[o], b[o + 1], b[o + 2], b[o + 3]]);
        // v2 → synthesise; v3 → read the real fields. Keyed on the LENGTH, not on `proto_ver`:
        // the length is what actually decides whether those bytes exist.
        let (phy_bitmap, phy_current) = if b.len() >= CAP_LEN {
            (u32be(29), b[33])
        } else {
            let phy = phy_of_v2_radio_kind(b[1])?;
            (1u32 << phy, phy)
        };
        Some(Self {
            proto_ver: b[0],
            radio_kind: b[1],
            freq_min_hz: u32be(2),
            freq_max_hz: u32be(6),
            pwr_min_dbm: b[10] as i8,
            pwr_max_dbm: b[11] as i8,
            stamp_hz: u32be(12),
            stamp_kind: b[16],
            max_payload: u16::from_be_bytes([b[17], b[18]]),
            cmd_bitmap: u32be(19),
            sf_min: b[23],
            sf_max: b[24],
            sched_gran_ns: u32be(25),
            phy_bitmap,
            phy_current,
        })
    }
}

/// Incremental frame parser: feed bytes, get whole frames.
///
/// A state machine rather than a buffer-and-scan so a partial frame across UART reads cannot be
/// lost, and so a stray `0x7E` inside a payload cannot resynchronise the parser mid-frame.
pub struct Parser {
    state: u8,
    typ: u8,
    len: u8,
    idx: usize,
    crc: u8,
    buf: [u8; MAX_PAYLOAD],
}

impl Default for Parser {
    fn default() -> Self {
        Self::new()
    }
}

impl Parser {
    pub const fn new() -> Self {
        Self {
            state: 0,
            typ: 0,
            len: 0,
            idx: 0,
            crc: 0,
            buf: [0; MAX_PAYLOAD],
        }
    }

    /// Feed one byte. Returns `Some((type, payload))` on a complete, CRC-valid frame.
    pub fn push(&mut self, b: u8) -> Option<(u8, &[u8])> {
        match self.state {
            0 => {
                if b == SYNC0 {
                    self.state = 1;
                }
            }
            1 => {
                // A second 0x7E keeps us waiting for 0xA5 rather than resetting: back-to-back frame
                // starts are otherwise mis-parsed.
                self.state = if b == SYNC1 {
                    2
                } else if b == SYNC0 {
                    1
                } else {
                    0
                };
            }
            2 => {
                self.typ = b;
                self.crc = b;
                self.state = 3;
            }
            3 => {
                self.len = b;
                self.crc ^= b;
                self.idx = 0;
                self.state = if b == 0 { 5 } else { 4 };
            }
            4 => {
                if self.idx < self.buf.len() {
                    self.buf[self.idx] = b;
                }
                self.crc ^= b;
                self.idx += 1;
                if self.idx as u8 >= self.len {
                    self.state = 5;
                }
            }
            _ => {
                self.state = 0;
                if b == self.crc {
                    let n = (self.len as usize).min(self.buf.len());
                    return Some((self.typ, &self.buf[..n]));
                }
                // CRC mismatch: drop silently and resynchronise. A corrupt frame must never reach
                // the radio as a transmit request.
            }
        }
        None
    }
}

/// Emit one frame through a byte sink.
pub fn write_frame<F: FnMut(u8)>(mut out: F, typ: u8, payload: &[u8]) {
    out(SYNC0);
    out(SYNC1);
    out(typ);
    out(payload.len() as u8);
    let mut crc = typ ^ (payload.len() as u8);
    for &b in payload {
        out(b);
        crc ^= b;
    }
    out(crc);
}

/// Encode a frame into `dst`, returning the byte count (0 if it would not fit).
pub fn encode(dst: &mut [u8], typ: u8, payload: &[u8]) -> usize {
    let need = 5 + payload.len();
    if dst.len() < need {
        return 0;
    }
    let mut i = 0;
    write_frame(
        |b| {
            dst[i] = b;
            i += 1;
        },
        typ,
        payload,
    );
    i
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Encode → parse must be the identity for every frame the node emits, INCLUDING the ones whose
    /// payload contains the frame-start bytes. This is not hypothetical: an `EVT_RX` carrying a
    /// whitened FLRC payload is uniformly distributed, so `7E A5` appears inside payloads routinely,
    /// and a parser that resynchronised on it would corrupt roughly one frame in 2^16.
    fn round_trip(typ: u8, payload: &[u8]) {
        let mut buf = [0u8; 5 + MAX_PAYLOAD];
        let n = encode(&mut buf, typ, payload);
        assert_eq!(n, 5 + payload.len());
        let mut p = Parser::new();
        let mut got = None;
        for (i, &b) in buf[..n].iter().enumerate() {
            if let Some((t, pl)) = p.push(b) {
                assert_eq!(i, n - 1, "frame completed early");
                got = Some((t, pl.to_vec()));
            }
        }
        let (t, pl) = got.expect("no frame parsed");
        assert_eq!(t, typ);
        assert_eq!(pl, payload);
    }

    #[test]
    fn parser_round_trips_every_event_shape() {
        round_trip(EVT_TXDONE, &[1, 0]);
        round_trip(CMD_GET_INFO, &[]);
        round_trip(EVT_INFO, &[0u8; 19]);
        round_trip(EVT_STATS, &[0xAAu8; 24]);
        round_trip(EVT_CAP, &[0x5Au8; CAP_LEN]);
        round_trip(EVT_PHY_ERR, &[phy_code::LR_FHSS, 0x01]);
        // The longest CMD_SET_HOP: 40 frequencies plus its 4-byte header.
        round_trip(CMD_SET_HOP, &[0xA5u8; 4 + 4 * crate::phy::MAX_HOPS]);
        round_trip(EVT_CLOCK, &[0, 0, 0, 0, 0xDE, 0xAD, 0xBE, 0xEF]);
        round_trip(CMD_GET_HOPTRACE, &[]);
        // The longest EVT_HOPTRACE: a full 32-entry ring.
        round_trip(EVT_HOPTRACE, &[0x5Au8; crate::hoptrace::MAX_BODY_LEN]);
        // …and the empty one a node with hopping off emits.
        round_trip(EVT_HOPTRACE, &[0x00, 0xF4, 0x24, 0x00, 0x00]);
        round_trip(EVT_SENSE, &[0xFF, 0xFE, 0xFF, 0xA2]);
        // Payload containing the frame-start pair and a lone SYNC0.
        round_trip(EVT_RX, &[SYNC0, SYNC1, 0x00, SYNC0, 0x7E, 0x7E, 0xA5]);
        // Longest legal payload.
        round_trip(EVT_LOG, &[0x41u8; MAX_PAYLOAD]);
    }

    #[test]
    fn parser_rejects_a_corrupt_crc() {
        let mut buf = [0u8; 32];
        let n = encode(&mut buf, EVT_TXDONE, &[1, 0]);
        buf[n - 1] ^= 0xFF; // clobber the CRC
        let mut p = Parser::new();
        for &b in &buf[..n] {
            assert!(
                p.push(b).is_none(),
                "a CRC-corrupt frame must never be delivered"
            );
        }
    }

    #[test]
    fn parser_skips_leading_garbage() {
        let mut buf = [0u8; 32];
        let n = encode(&mut buf, CMD_TX, b"hi");
        let mut p = Parser::new();
        // Line noise with a lone SYNC0 in it — the state machine must fall back to hunting rather
        // than sit waiting for a payload that is not coming.
        for &b in b"\x00\xff\x7e\x00\xff\x7e" {
            assert!(p.push(b).is_none());
        }
        let mut got = false;
        for &b in &buf[..n] {
            if let Some((t, pl)) = p.push(b) {
                assert_eq!(t, CMD_TX);
                assert_eq!(pl, b"hi");
                got = true;
            }
        }
        assert!(got);
    }

    /// **Resynchronisation is bounded, not immediate**, and the bound is worth knowing.
    ///
    /// A `7E A5` that appears in line noise (or as the tail of a truncated frame) puts the parser
    /// into a real frame's state machine: the next two bytes become a type and a *length*, and the
    /// parser then consumes up to 255 payload bytes plus a CRC before it can look for a frame start
    /// again. So a frame arriving immediately behind a bogus start is **eaten**, and this test
    /// pins how much stream it takes to get back in step: at most `2 + 255 + 1` bytes.
    ///
    /// Two consequences a reader should have in front of them rather than rediscover:
    /// * the resync can emit at most one *spurious* frame (a length and a CRC that happen to agree,
    ///   as an all-zero run does) — which is why an unrecognised opcode must be answered
    ///   `EVT_UNSUPPORTED` and never acted on;
    /// * this is inherent to length-prefixed framing with no inter-frame timeout, and is the reason
    ///   the CRC is checked before a frame is ever handed to the radio.
    #[test]
    fn parser_resynchronises_within_a_bounded_window() {
        let mut p = Parser::new();
        for &b in b"\x7e\x7e\xa5" {
            let _ = p.push(b); // a bogus frame start: the parser is now mid-"frame"
        }
        // Enough stream to flush the worst-case bogus length. Spurious frames here are allowed;
        // being permanently desynchronised is not.
        for _ in 0..(2 + 255 + 1) {
            let _ = p.push(0x00);
        }
        let mut buf = [0u8; 32];
        let n = encode(&mut buf, CMD_TX, b"hi");
        let mut got = false;
        for &b in &buf[..n] {
            if let Some((t, pl)) = p.push(b) {
                assert_eq!(t, CMD_TX);
                assert_eq!(pl, b"hi");
                got = true;
            }
        }
        assert!(got, "parser never resynchronised");
    }

    /// The FLRC boot state of the LF (915 MHz) build, as a value the two `EVT_CAP` tests share.
    fn flrc_boot_cap() -> Capabilities {
        Capabilities {
            proto_ver: PROTO_VER,
            radio_kind: radio_kind::LR2021,
            freq_min_hz: 902_000_000,
            freq_max_hz: 928_000_000,
            pwr_min_dbm: -9,
            pwr_max_dbm: 22,
            stamp_hz: 16_000_000,
            stamp_kind: stamp_kind::HARDWARE_FREE_RUNNING,
            max_payload: 47,
            // FLRC has no intra-packet hopping, so `CMD_SET_HOP` is not in this mode's
            // surface — see `cmd_bitmap_for`.
            cmd_bitmap: cmd_bitmap_for(false),
            sf_min: 0,
            sf_max: 0,
            sched_gran_ns: crate::airtime::SCHED_GRAN_NS,
            phy_bitmap: crate::phy::PHY_BITMAP,
            phy_current: phy_code::FLRC,
        }
    }

    /// The exact 34 bytes the LF (915 MHz) build of `m6_bridge` emits **while it is in FLRC**,
    /// pinned so a change to any contributing constant — band, PA range, `TICKS_PER_US`,
    /// `FLRC_FRAME_LEN`, the opcode set, the PHY set — shows up here rather than as a host that
    /// quietly mis-sizes packets or mis-scales a timestamp.
    ///
    /// ```text
    ///   03 02 35 C3 6D 80 37 50 28 00 F7 16 00 F4 24 00 03 00 2F BD BF DF DE 00 00 00 00 C3 50 00 00 00 A1 05
    ///   ^  ^  \__ 902 MHz __/ \__ 928 MHz __/ ^  ^  \_ 16 MHz __/ ^  \_47_/ \_ bitmap _/ ^ ^ \50 us/ \_ phys _/ ^
    ///   |  radio_kind = LR2021 (the PART)   -9 +22 dBm         stamp_kind=3    sf_min/max  sched_gran  |    phy_current
    ///   proto_ver = 3                                                                        LoRa|FLRC|LR-FHSS
    /// ```
    ///
    /// `sched_gran_ns` was 0 until `CMD_TX_AT` existed. The two move together **by construction** —
    /// the host's `NodeProfile::schedules_tx()` is `sched_gran_ns > 0 && supports(CMD_TX_AT)`, so a
    /// granularity without the opcode (or the reverse) is a claim it silently discards.
    #[test]
    fn capabilities_wire_bytes_for_this_node() {
        // The golden body is the **LF, no-opt-in** build's: a `PHY_HF=1` build has a different band,
        // and `PHY_LRFHSS=1` adds bit 7 back to `phy_bitmap`, so pinning all three here would just
        // pin whichever one CI happened to compile. The opcode bitmap is build-independent and is
        // checked either way.
        assert_eq!(CMD_BITMAP, 0xFDBF_DFDE);
        if crate::phy::IS_HF || option_env!("PHY_LRFHSS").is_some() {
            return;
        }
        let cap = flrc_boot_cap();
        assert_eq!(
            cap.to_bytes(),
            [
                0x03, 0x02, 0x35, 0xC3, 0x6D, 0x80, 0x37, 0x50, 0x28, 0x00, 0xF7, 0x16, 0x00, 0xF4,
                0x24, 0x00, 0x03, 0x00, 0x2F, 0xBD, 0xBF, 0xDF, 0xDE, 0x00, 0x00, 0x00, 0x00, 0x27,
                // phy_bitmap = 0x21 (LoRa | FLRC). LR-FHSS's bit 7 is deliberately absent — entering
                // that mode traps the part, MEASURED; see `phy::PHY_BITMAP`.
                0x10, 0x00, 0x00, 0x00, 0x21, 0x05,
            ]
        );
        // …and the whole 7E-A5 frame the host sees.
        let mut buf = [0u8; 64];
        let n = encode(&mut buf, EVT_CAP, &cap.to_bytes());
        assert_eq!(&buf[..5], &[0x7E, 0xA5, 0x8B, 0x22, 0x03]);
        // CRC tracks the body: LR-FHSS leaving `phy_bitmap` moved it 0xD5 -> 0x55 (0xA1^0x21 = 0x80),
        // then sched_gran_ns 50_000 -> 10_000 moved it 0x55 -> 0xF1 (0xC3^0x27 ^ 0x50^0x10 = 0xA4).
        assert_eq!(buf[n - 1], 0xF1);
    }

    #[test]
    fn capabilities_round_trip() {
        let cap = flrc_boot_cap();
        let b = cap.to_bytes();
        assert_eq!(b.len(), CAP_LEN);
        assert_eq!(Capabilities::from_bytes(&b), Some(cap));
        // Field offsets are the wire contract; pin the ones a host indexes directly.
        assert_eq!(b[0], 3);
        assert_eq!(b[1], 2);
        assert_eq!(b[10] as i8, -9);
        assert_eq!(u32::from_be_bytes([b[12], b[13], b[14], b[15]]), 16_000_000);
        assert_eq!(b[16], 3);
        assert_eq!(u16::from_be_bytes([b[17], b[18]]), 47);
        assert_eq!(u32::from_be_bytes([b[25], b[26], b[27], b[28]]), 10_000);
        assert_eq!(
            u32::from_be_bytes([b[29], b[30], b[31], b[32]]),
            crate::phy::PHY_BITMAP
        );
        assert_eq!(b[33], phy_code::FLRC);
    }

    /// ★ **A PHY switch must move every per-PHY field**, and `CMD_SET_PHY` replies with a whole new
    /// body so the host replaces its profile rather than patching it. This pins the pair of bodies
    /// the FLRC→LoRa switch produces, and asserts they differ everywhere they must.
    #[test]
    fn a_phy_switch_moves_every_per_phy_field() {
        use crate::phy::Phy;
        let flrc = flrc_boot_cap();
        let lora = Capabilities {
            max_payload: crate::phy::max_payload(Phy::Lora) as u16,
            sf_min: crate::phy::sf_min(Phy::Lora),
            sf_max: crate::phy::sf_max(Phy::Lora),
            sched_gran_ns: crate::phy::sched_gran_ns(Phy::Lora),
            phy_current: phy_code::LORA,
            ..flrc
        };
        assert_ne!(flrc.max_payload, lora.max_payload);
        assert_ne!(flrc.sf_max, lora.sf_max);
        assert_ne!(flrc.phy_current, lora.phy_current);
        // The part, the band, the PA and the timestamp do NOT move — they are properties of the
        // board, not of the mode, and a host that saw them change would re-derive them for nothing.
        assert_eq!(flrc.radio_kind, lora.radio_kind);
        assert_eq!(flrc.freq_min_hz, lora.freq_min_hz);
        assert_eq!(flrc.stamp_hz, lora.stamp_hz);
        assert_eq!(flrc.phy_bitmap, lora.phy_bitmap);
        // Both bodies round-trip, and `phy_current` is always a set bit of `phy_bitmap`.
        for c in [flrc, lora] {
            assert_eq!(Capabilities::from_bytes(&c.to_bytes()), Some(c));
            assert_ne!(c.phy_bitmap & (1 << c.phy_current), 0);
        }
    }

    /// **A v2 node still parses.** Its 29-byte body has no PHY fields, so it gets the single-entry
    /// bitmap for the one mode it is running — recovered from `radio_kind`, not invented — and the
    /// absent `CMD_SET_PHY` bit then tells the host it cannot be changed.
    #[test]
    fn a_v2_capabilities_body_still_parses() {
        // The exact bytes the v2 firmware emitted, from this test's own previous revision.
        let v2: [u8; CAP_LEN_V2] = [
            0x02, 0x02, 0x35, 0xC3, 0x6D, 0x80, 0x37, 0x50, 0x28, 0x00, 0xF7, 0x16, 0x00, 0xF4,
            0x24, 0x00, 0x03, 0x00, 0x2F, 0x1D, 0xBF, 0xDF, 0xDE, 0x00, 0x00, 0x00, 0x00, 0xC3,
            0x50,
        ];
        let c = Capabilities::from_bytes(&v2).expect("v2 body must still parse");
        assert_eq!(c.proto_ver, 2);
        assert_eq!(c.radio_kind, radio_kind::LR2021);
        assert_eq!(c.max_payload, 47);
        // One mode, and it is the one a v2 LR2021 could only have been running.
        assert_eq!(c.phy_current, phy_code::FLRC);
        assert_eq!(c.phy_bitmap, 1 << phy_code::FLRC);
        assert_eq!(c.cmd_bitmap & (1 << CMD_SET_PHY), 0, "v2 cannot switch PHY");

        // v2's retired kind 3 decodes to the same part running LoRa.
        let mut v2_lora = v2;
        v2_lora[1] = radio_kind::V2_LR2021_LORA;
        let c = Capabilities::from_bytes(&v2_lora).unwrap();
        assert_eq!(c.phy_current, phy_code::LORA);
        assert_eq!(c.phy_bitmap, 1 << phy_code::LORA);

        // The other fleet nodes.
        for (kind, phy) in [
            (radio_kind::SX1262, phy_code::LORA),
            (radio_kind::SX1276, phy_code::LORA),
        ] {
            let mut b = v2;
            b[1] = kind;
            assert_eq!(Capabilities::from_bytes(&b).unwrap().phy_current, phy);
        }

        // An unknown kind is REJECTED rather than given an invented PHY: `None` is diagnosable, a
        // wrong `phy_current` is a unit error that gets calibrated against.
        let mut b = v2;
        b[1] = 0x7F;
        assert_eq!(Capabilities::from_bytes(&b), None);
        // Shorter than a v2 body is not a body at all.
        assert_eq!(Capabilities::from_bytes(&v2[..CAP_LEN_V2 - 1]), None);
    }

    /// The bitmap and the `EVT_UNSUPPORTED` list must partition the opcode space: a bit set for a
    /// command the bridge rejects is a lie the host will act on.
    #[test]
    fn cmd_bitmap_matches_the_documented_set() {
        for op in [
            CMD_TX,
            CMD_SET_FREQ,
            CMD_SET_MOD,
            CMD_SET_PWR,
            CMD_GET_INFO,
            CMD_SET_BEACON,
            CMD_CAD,
            CMD_GET_RSSI,
            CMD_SET_CAD_CFG,
            CMD_SET_LBT_CFG,
            CMD_SET_PREAMBLE,
            CMD_TX_LBT,
            CMD_SET_NAME_FILTER,
            CMD_SET_RELAY,
            CMD_DATAPLANE,
            CMD_SET_SENSE_CFG,
            CMD_GET_STATS,
            CMD_RESET_STATS,
            CMD_SET_DEBUG,
            CMD_READ_CLOCK,
            CMD_TX_AT,
            CMD_GET_CAP,
            CMD_SENSE,
            CMD_SET_RX_GAIN,
            CMD_SET_PHY,
            CMD_SET_HOP,
            CMD_TX_AT_ABS,
        ] {
            assert!(
                CMD_BITMAP & (1 << op) != 0,
                "opcode {op:#04x} implemented but not in bitmap"
            );
        }
        // The three that stay refused. Each is a fact about the radio or the board, not a gap:
        // FLRC's 32-bit syncword cannot take a byte, FLRC has no spreading factor, and the XIAO has
        // no ROM serial loader.
        for op in [CMD_SET_SYNC, CMD_SF_SCAN, CMD_ENTER_BOOTLOADER] {
            assert_eq!(
                CMD_BITMAP & (1 << op),
                0,
                "opcode {op:#04x} is rejected but claimed"
            );
        }
        // Bit 0 is not an opcode, and 0x19 is still unassigned in the fleet's space. 0x1D..0x1F
        // were unassigned in v2 and are the three commands v3 adds.
        for op in [0u8, 0x19] {
            assert_eq!(CMD_BITMAP & (1u32 << op), 0, "bit {op} claimed but unassigned");
        }
    }

    /// **The command surface is per-PHY too, and `CMD_SET_HOP` is the one that moves.**
    ///
    /// `cmd_bitmap` means "implemented and will act". In FLRC this node implements `CMD_SET_HOP`
    /// and refuses it on every call (`phy::check_hop` -> `HopReject::WrongPhy`), because the part
    /// has no FLRC hopping command. A set bit there would hand the host a knob that can only fail,
    /// and the host builds a whole `HopCapability` out of this bit — so it drops in FLRC exactly as
    /// `max_payload` and the SF span already do.
    #[test]
    fn the_command_surface_moves_with_the_phy() {
        use crate::phy::{self, Phy};
        // One bit differs, and it is the hopping one.
        assert_eq!(cmd_bitmap_for(true), CMD_BITMAP);
        assert_eq!(cmd_bitmap_for(false), CMD_BITMAP & !(1 << CMD_SET_HOP));
        assert_eq!(cmd_bitmap_for(true) ^ cmd_bitmap_for(false), 1 << CMD_SET_HOP);
        assert_eq!(cmd_bitmap_for(false), 0xBDBF_DFDE);
        // ...and it is keyed on the property, never on the PHY name, so a fourth PHY cannot be
        // added to `phy::has_intra_packet_hopping` without its bitmap following.
        for p in [Phy::Lora, Phy::LrFhss] {
            assert_ne!(
                cmd_bitmap_for(phy::has_intra_packet_hopping(p)) & (1 << CMD_SET_HOP),
                0,
                "{p:?} hops inside a packet and must advertise the actuator"
            );
        }
        assert_eq!(
            cmd_bitmap_for(phy::has_intra_packet_hopping(Phy::Flrc)) & (1 << CMD_SET_HOP),
            0,
            "FLRC refuses every hop plan; the bit must not be claimed"
        );
        // Nothing ELSE moves: a per-PHY surface that quietly dropped a second opcode would be a
        // capability regression no host could distinguish from a firmware downgrade.
        assert_eq!(
            cmd_bitmap_for(true) & !(1 << CMD_SET_HOP),
            cmd_bitmap_for(false)
        );
    }

    /// **`CMD_TX_AT` and `sched_gran_ns` must move together.** The host ANDs them
    /// (`NodeProfile::schedules_tx`), so either alone is a claim it discards — and worse, a
    /// granularity advertised without the opcode reads to a human as a working scheduler.
    ///
    /// `CMD_TX_AT_ABS` is required alongside it because the declared granularity is **only** true of
    /// the absolute path: the relative opcode's placement is bounded by the serial round trip
    /// ([`crate::airtime::RELATIVE_SCHED_SPREAD_NS`], measured p2p 550 µs), which is 11× the number
    /// this node advertises. A node claiming 50 µs with no way to *name an instant* would be
    /// advertising something no host could reach.
    #[test]
    fn scheduled_tx_is_claimed_in_both_places_or_neither() {
        assert_ne!(CMD_BITMAP & (1 << CMD_TX_AT), 0);
        assert_ne!(CMD_BITMAP & (1 << CMD_TX_AT_ABS), 0);
        assert!(crate::airtime::SCHED_GRAN_NS > 0);
        // The relative path is an order of magnitude coarser, and that gap is the reason the
        // absolute opcode exists. If a future change ever made them comparable, this is where to
        // reconsider which one `EVT_CAP` describes.
        assert!(crate::airtime::RELATIVE_SCHED_SPREAD_NS > crate::airtime::SCHED_GRAN_NS * 5);
        // Every PHY must be able to honour the one granularity `EVT_CAP` carries, since the host
        // gets a fresh body on every switch and would otherwise see the claim change under it.
        for p in [crate::phy::Phy::Lora, crate::phy::Phy::Flrc, crate::phy::Phy::LrFhss] {
            assert_eq!(crate::phy::sched_gran_ns(p), crate::airtime::SCHED_GRAN_NS);
        }
    }

    #[test]
    fn error_bit_packing_is_positional() {
        let mut f = [false; 14];
        f[9] = true; // rxfreq_no_fe_cal
        assert_eq!(err_bits(f), 1 << 9);
        assert_eq!(err_bits([true; 14]), 0x3FFF);
        assert_eq!(err_bits([false; 14]), 0);
    }

    /// Event numbering is shared across four firmwares; a collision here is a silent mis-decode on
    /// the host, which is exactly how `EVT_STATS = 0x87` survived.
    #[test]
    fn fleet_event_numbering() {
        assert_eq!(
            [
                EVT_RX,
                EVT_TXDONE,
                EVT_INFO,
                EVT_LOG,
                EVT_CAD,
                EVT_RSSI,
                EVT_SF_DETECTED,
                EVT_TX_STARTED,
                EVT_STATS,
                EVT_CLOCK,
                EVT_CAP,
                EVT_SENSE,
                EVT_PHY_ERR,
                EVT_HOPTRACE,
                EVT_UNSUPPORTED,
                EVT_RX_STAMP,
                EVT_CLOCK_REF
            ],
            [
                0x81, 0x82, 0x83, 0x84, 0x85, 0x86, 0x87, 0x88, 0x89, 0x8A, 0x8B, 0x8C, 0x8D, 0x8E,
                0x8F, 0x90, 0x91
            ]
        );
        // ★ The registry only fails a collision if it covers events this firmware does not emit.
        // `EVT_RX_STAMP` is the Waveshare's; it sat on `EVT_HOPTRACE`'s 0x8E for a whole feature
        // because the array above listed only local constants. Pairwise-distinct, checked here, so
        // the next one fails a build instead of a bench run.
        let all = [
            EVT_RX,
            EVT_TXDONE,
            EVT_INFO,
            EVT_LOG,
            EVT_CAD,
            EVT_RSSI,
            EVT_SF_DETECTED,
            EVT_TX_STARTED,
            EVT_STATS,
            EVT_CLOCK,
            EVT_CAP,
            EVT_SENSE,
            EVT_PHY_ERR,
            EVT_HOPTRACE,
            EVT_UNSUPPORTED,
            EVT_RX_STAMP,
        ];
        for (i, a) in all.iter().enumerate() {
            assert!(*a >= 0x81, "events live above the command space");
            for b in &all[i + 1..] {
                assert_ne!(a, b, "two fleet events share an opcode");
            }
        }
    }

    /// ★ **`CMD_GET_HOPTRACE` does not fit `cmd_bitmap`, and that is pinned rather than discovered
    /// on the bench.**
    ///
    /// The field is four bytes on four firmwares and two host crates; opcode 0x20 is bit 32. Every
    /// *other* opcode must stay inside the field, and the hop-trace opcode must stay out of it — so
    /// this test fails the moment either a 33rd opcode is added without a plan or someone
    /// "fixes" the bitmap by silently truncating a claim.
    #[test]
    fn the_hop_trace_opcode_is_past_the_end_of_the_bitmap_field() {
        assert_eq!(CMD_GET_HOPTRACE, 0x20);
        assert_eq!(CMD_GET_HOPTRACE as u32, 32);
        // The wire field is exactly the low 32 bits of what the firmware implements.
        assert_eq!(CMD_BITMAP, CMD_BITMAP_FULL as u32);
        assert_ne!(CMD_BITMAP_FULL & (1u64 << CMD_GET_HOPTRACE), 0);
        // Every opcode the bitmap *does* claim is inside the field, so nothing else is lost.
        assert_eq!(CMD_BITMAP_FULL >> 33, 0, "an opcode above 0x20 would vanish unnoticed");
        // And the per-PHY surface still moves on exactly one bit — the hop-trace opcode is not in
        // it either way, because a bitmap cannot carry it at all.
        assert_eq!(cmd_bitmap_for(true) ^ cmd_bitmap_for(false), 1 << CMD_SET_HOP);
    }
}
