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
//! `CMD_GET_CAP` answers `radio_kind = 2` (LR2021-FLRC) and then says the rest in fixed units, so
//! the host never has to hard-code what this node can do:
//!
//! - **`stamp_hz = 16_000_000`, `stamp_kind = 3`** — the `EVT_RX` `ts` field is the M4 hardware
//!   capture: DPPI-latched at the DIO edge, 62.5 ns per tick, no CPU in the loop. Same wire field
//!   the Waveshare fills with a millisecond software counter; three orders of magnitude better, and
//!   the host now learns that rather than assuming the worst case everywhere.
//! - **`max_payload = 47`** — one fixed 48-byte on-air PDU minus the in-frame length byte. Small,
//!   and true. See `flrc_link::build_frame`.
//! - **`sched_gran_ns = 0`, `CMD_TX_AT` refused** — not a firmware gap. See the `CMD_TX_AT` arm.
//! - **no spreading factor** (`sf_min = sf_max = 0`); the `sf` slot of `CMD_SET_MOD`/`EVT_INFO`
//!   carries an FLRC bitrate rung instead, which is exactly what `radio_kind` exists to disambiguate.
//!
//! ## Never silence
//!
//! Every command is answered — a `SET_*` with `EVT_INFO`, an unimplemented opcode with
//! `EVT_UNSUPPORTED`. Five commands here used to reply nothing at all, and the host's
//! `exec_idempotent` waits for `EVT_INFO`, retries four times and then fails: a knob that applied
//! perfectly looked like a dead node.

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
use lr2021::system::ChipMode;

use lr2021_nrf54l15_rs::serial::{self, Parser};
use lr2021_nrf54l15_rs::timing::{RxCapture, TICKS_PER_US};
use lr2021_nrf54l15_rs::{flrc_link, hw};

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
const CCA_STEPS: u32 = 4096;

/// How long to wait for TxDone before giving up and re-arming RX. A frame is sub-millisecond; this
/// is three orders of magnitude of slack, and it exists only so a wedged chip cannot hang the loop.
const TX_TIMEOUT_MS: u64 = 20;

/// Upper bound on one LBT backoff draw. Purely a sanity clamp so a host writing a nonsense
/// `cw_ms`/`max_backoff` cannot park the node for hours.
const MAX_BACKOFF_US: u32 = 1_000_000;

/// Spec: actual signal power is `−value/2` dBm — for `rssi_inst`, `rssi_avg`, `rssi_sync` and every
/// CCA result alike. Every RSSI this firmware puts on the wire goes through here.
///
/// `EVT_RX` used to carry the **raw** register value cast to `i16` (163..193 on a two-foot link), so
/// the host read a strong bench link as a positive three-digit "dBm".
fn dbm_of(raw: u16) -> i16 {
    -((raw as i16) / 2)
}

/// Carrier-sense / LBT state, all host-tunable so tuning needs a serial command, not a reflash.
struct Sense {
    /// Busy threshold, dBm. **A starting point, not a measurement**: FLRC sensitivity at BR 2600 /
    /// CR 3/4 is about −100.5 dBm, so −90 dBm is ~10 dB of margin above the noise floor. Tune it on
    /// air with `CMD_SET_SENSE_CFG` and `EVT_SENSE`.
    thresh_dbm: i16,
    /// CCA samples per sense, OR'd. More cuts false-negatives at the cost of airtime.
    repeat: u8,
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
    match radio.set_and_get_cca(CCA_STEPS, None).await {
        Ok(r) => dbm_of(r.rssi_max()) > s.thresh_dbm,
        Err(_) => false,
    }
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
async fn wait_tx_done(radio: &mut hw::Radio, timeout_ms: u64) -> bool {
    let deadline = Instant::now() + Duration::from_millis(timeout_ms);
    while Instant::now() < deadline {
        if let Ok(irq) = radio.get_and_clear_irq().await {
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
async fn tx_once(radio: &mut hw::Radio, payload: &[u8]) -> bool {
    // Refuse over-long payloads BEFORE touching the radio, so a bad request cannot disturb RX.
    let Some(frame) = flrc_link::build_frame(payload) else {
        return false;
    };
    flrc_link::settle_before_tx(radio).await;
    // Clear first: `wr_tx_fifo_from` APPENDS, so a frame left by a transmit that never fired would
    // otherwise be prepended to this one.
    let _ = radio.clear_tx_fifo().await;
    if radio.wr_tx_fifo_from(&frame).await.is_err() {
        return false;
    }
    if radio.set_tx(0).await.is_err() {
        return false;
    }
    wait_tx_done(radio, TX_TIMEOUT_MS).await
}

/// Atomic listen-before-talk: randomised backoff → sense → key-up, giving up after
/// `lbt_max_attempts`. Returns `(sent, attempts)`.
///
/// The backoff comes **before** the sense, and is randomised, for the reason the LoRa CSMA work
/// measured: a fixed offset makes two nodes take turns, but a third node then collides with
/// whichever it is phase-locked to. Note also what that work found — LBT *hurts* at N=2 on a clean
/// channel. This is here for N≥3, not as a default for every transmit.
async fn lbt_tx(radio: &mut hw::Radio, s: &mut Sense, payload: &[u8]) -> (bool, u8) {
    let mut attempt: u8 = 0;
    let mut sent = false;
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
            sent = tx_once(radio, payload).await;
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

/// The 19-byte `EVT_INFO` body — see [`serial::EVT_INFO`] for the per-field meaning here.
async fn info_body(radio: &mut hw::Radio, link: &flrc_link::LinkState, s: &Sense) -> [u8; 19] {
    let status = match radio.get_status().await {
        Ok((st, _)) => (st.chip_mode() as u8) | ((st.cmd() as u8) << 4),
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
    let sync = flrc_link::SYNCWORD as u16;
    let f = link.freq_hz.to_be_bytes();
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
        link.bitrate as u8, // the fleet's `sf` slot: an FLRC rate rung, see CMD_SET_MOD
        0,                  // `bw`: FLRC bandwidth follows the rung; not an independent knob
        link.coding as u8,
        link.tx_power_dbm() as u8, // dBm ACTUALLY APPLIED, not what the host asked for
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

    let mut link = flrc_link::LinkState::default();
    flrc_link::configure_with(&mut radio, &link)
        .await
        .expect("FLRC configure");
    let cap = RxCapture::new(timing);

    // Seed the backoff PRNG from two sources that differ between boards: the chip's hardware RNG and
    // the free-running MAC clock at the instant we get here. A deterministic seed would make both
    // nodes draw the SAME backoff sequence, which is not a backoff at all.
    let hw_rng = radio.get_random_number().await.unwrap_or(0);
    let mut sense = Sense::new(hw_rng ^ cap.now().ticks);

    radio.set_rx_continous().await.expect("rx");

    defmt::info!(
        "m6_bridge: up — 7E-A5 v2 on UART20 @115200, FLRC {=u32} Hz, {=i8} dBm, max_payload {=usize}",
        link.freq_hz,
        link.tx_power_dbm(),
        flrc_link::PAYLOAD_MAX
    );

    // This node's self-description. Every field is a source constant or a measurement; nothing here
    // is a placeholder. `sched_gran_ns = 0` and the missing `CMD_TX_AT` bit are the honest answer to
    // P4, not an omission — see the `CMD_TX_AT` arm below.
    let capabilities = serial::Capabilities {
        proto_ver: serial::PROTO_VER,
        radio_kind: serial::radio_kind::LR2021_FLRC,
        freq_min_hz: flrc_link::BAND_MIN_HZ,
        freq_max_hz: flrc_link::BAND_MAX_HZ,
        pwr_min_dbm: flrc_link::PWR_MIN_DBM,
        pwr_max_dbm: flrc_link::PWR_MAX_DBM,
        // Not a guess: `timing::TICKS_PER_US` is the constant `TIMER20` is programmed with, and the
        // 16 MHz it implies was confirmed on air (15 frames at ~1 s spacing, 16,616,401..16,625,857
        // ticks apart).
        stamp_hz: TICKS_PER_US * 1_000_000,
        stamp_kind: serial::stamp_kind::HARDWARE_FREE_RUNNING,
        max_payload: flrc_link::PAYLOAD_MAX as u16,
        cmd_bitmap: serial::CMD_BITMAP,
        // FLRC has no spreading factor. 0/0 is the field's documented "none", and radio_kind = 2
        // tells the host why.
        sf_min: 0,
        sf_max: 0,
        sched_gran_ns: 0,
    };
    let cap_bytes = capabilities.to_bytes();

    let mut dp = ndn::DataPlane::new();
    let mut parser = Parser::new();
    let (mut n_rx, mut n_tx) = (0u32, 0u32);

    // 32-bit counter → 64-bit clock. TIMER20 wraps every ~4.5 minutes at 16 MHz, which is far
    // shorter than a measurement session, so `EVT_CLOCK` carries a firmware-extended count: the LOW
    // 32 bits are exactly the `EVT_RX` ts field, the high 32 are the wrap count. The loop samples
    // every iteration (~1 ms), so a wrap cannot be missed.
    let mut clock_hi: u32 = 0;
    let mut clock_last: u32 = cap.now().ticks;

    let mut next_sense = Instant::now();
    let mut out = [0u8; 320];

    loop {
        // ── clock extension, sampled every pass ────────────────────────────────────────────────
        let now_ticks = cap.now().ticks;
        if now_ticks < clock_last {
            clock_hi = clock_hi.wrapping_add(1);
        }
        clock_last = now_ticks;

        // ── host → node: drain whatever the UART has, without blocking the radio ───────────────
        let mut byte = [0u8; 1];
        if embassy_time::with_timeout(Duration::from_millis(1), uart.read_exact(&mut byte))
            .await
            .is_ok()
        {
            if let Some((typ, pl)) = parser.push(byte[0]) {
                match typ {
                    serial::CMD_TX => {
                        let ok = tx_once(&mut radio, pl).await;
                        n_tx = n_tx.wrapping_add(1);
                        send(&mut uart, &mut out, serial::EVT_TXDONE, &[ok as u8, 0]).await;
                        // Continuous RX is dropped by a transmit; re-arm or the node goes deaf
                        // after its first frame — a failure that looks like "the link died".
                        let _ = radio.set_rx_continous().await;
                    }
                    serial::CMD_TX_LBT => {
                        let (sent, attempts) = lbt_tx(&mut radio, &mut sense, pl).await;
                        n_tx = n_tx.wrapping_add(1);
                        send(
                            &mut uart,
                            &mut out,
                            serial::EVT_TXDONE,
                            &[sent as u8, attempts],
                        )
                        .await;
                        let _ = radio.set_rx_continous().await;
                    }
                    serial::CMD_SET_FREQ if pl.len() >= 4 => {
                        let f = u32::from_be_bytes([pl[0], pl[1], pl[2], pl[3]]);
                        if !flrc_link::in_band(f) {
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
                            link.freq_hz = f;
                            // ★ The whole RF chain, from Standby RC. Issuing `set_rf` from
                            // RX-continuous is what used to break TX permanently: every later
                            // transmit returned ok=0 until the board was power-cycled, and a
                            // following CMD_SET_PWR did not restore it.
                            let _ = flrc_link::retune(&mut radio, &link).await;
                            let body = info_body(&mut radio, &link, &sense).await;
                            send(&mut uart, &mut out, serial::EVT_INFO, &body).await;
                        }
                    }
                    serial::CMD_SET_PWR if !pl.is_empty() => {
                        // The host speaks dBm; the chip takes HALF-dB steps. Passing the host's
                        // value straight through made every power request come out 2x too low.
                        link.tx_power_half_db = flrc_link::dbm_to_half_db(pl[0] as i8);
                        let _ = flrc_link::retune(&mut radio, &link).await;
                        let body = info_body(&mut radio, &link, &sense).await;
                        send(&mut uart, &mut out, serial::EVT_INFO, &body).await;
                    }
                    serial::CMD_SET_MOD if pl.len() >= 3 => {
                        // `[rate_code, _, cr_code]` — the fleet's [sf, bw, cr] byte positions with
                        // FLRC meanings. A rate change needed a reflash (`PHY_BR`) until now.
                        match (bitrate_of_code(pl[0]), cr_of_code(pl[2])) {
                            (Some(br), Some(cr)) => {
                                link.bitrate = br;
                                link.coding = cr;
                                let _ = flrc_link::retune(&mut radio, &link).await;
                                let body = info_body(&mut radio, &link, &sense).await;
                                send(&mut uart, &mut out, serial::EVT_INFO, &body).await;
                            }
                            _ => {
                                send(
                                    &mut uart,
                                    &mut out,
                                    serial::EVT_UNSUPPORTED,
                                    &[serial::CMD_SET_MOD, serial::REASON_OUT_OF_RANGE],
                                )
                                .await;
                            }
                        }
                    }
                    serial::CMD_GET_INFO => {
                        let body = info_body(&mut radio, &link, &sense).await;
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
                        let _ = radio.set_rx_continous().await;
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
                        let _ = radio.set_rx_continous().await;
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
                        let body = info_body(&mut radio, &link, &sense).await;
                        send(&mut uart, &mut out, serial::EVT_INFO, &body).await;
                    }
                    serial::CMD_SET_SENSE_CFG if pl.len() >= 3 => {
                        sense.thresh_dbm = i16::from_be_bytes([pl[0], pl[1]]);
                        sense.repeat = pl[2];
                        let body = info_body(&mut radio, &link, &sense).await;
                        send(&mut uart, &mut out, serial::EVT_INFO, &body).await;
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
                        let body = info_body(&mut radio, &link, &sense).await;
                        send(&mut uart, &mut out, serial::EVT_INFO, &body).await;
                    }
                    serial::CMD_DATAPLANE if pl.len() >= 2 => {
                        // Name-keyed hopping is refused rather than half-applied: it needs a
                        // channel-index convention (`carrier = (850 + ch) MHz` on the LoRa nodes)
                        // and this bearer has none. Validate before applying anything.
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
                            let body = info_body(&mut radio, &link, &sense).await;
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
                        let body = info_body(&mut radio, &link, &sense).await;
                        send(&mut uart, &mut out, serial::EVT_INFO, &body).await;
                    }
                    serial::CMD_READ_CLOCK => {
                        let ticks = ((clock_hi as u64) << 32) | cap.now().ticks as u64;
                        send(&mut uart, &mut out, serial::EVT_CLOCK, &ticks.to_be_bytes()).await;
                    }
                    serial::CMD_GET_CAP => {
                        send(&mut uart, &mut out, serial::EVT_CAP, &cap_bytes).await;
                    }
                    serial::CMD_TX_AT => {
                        // ★ **Refused on hardware grounds, and it is a wiring fact, not a gap.**
                        //
                        // Hardware-scheduled TX on this part is `TIMER20.CC[2] --DPPI--> GPIOTE OUT`
                        // driving a DIO the radio is configured as `DioFunc::TxTrigger` (see
                        // `m5_tx`). The shield brings out exactly ONE DIO: LR2021 **DIO8 → P1.04**.
                        // That same pin is the radio's IRQ output, and `RxCapture` holds it as a
                        // GPIOTE20_CH0 **InputChannel** (LoToHi) routed by PPI20_CH0 into
                        // `TIMER20.CC[0].CAPTURE` — the hardware RX timestamp.
                        //
                        // So the conflict is doubled and unavoidable: on the radio side DIO8 is
                        // either an interrupt OUTPUT or a trigger INPUT, and on the MCU side P1.04
                        // is either a GPIOTE InputChannel or an OutputChannel. One pin, opposite
                        // directions, on both ends.
                        //
                        // The RX capture is this node's highest-value capability (62.5 ns, the
                        // number the whole board exists to produce), so it is not being traded for
                        // a scheduled TX that `m5_tx` can already demonstrate on its own. Answering
                        // `EVT_UNSUPPORTED` with `sched_gran_ns = 0` keeps `EVT_CAP` honest; a
                        // software-timed "scheduled" TX would report a granularity the host would
                        // build guard bands from, and it would be wrong by three orders of
                        // magnitude.
                        send(
                            &mut uart,
                            &mut out,
                            serial::EVT_UNSUPPORTED,
                            &[serial::CMD_TX_AT, serial::REASON_NO_HARDWARE],
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
                    | serial::CMD_SET_LBT_CFG
                    | serial::CMD_SET_SENSE_CFG
                    | serial::CMD_DATAPLANE => {
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
            if irq.rx_done() {
                let ts = cap.hw_stamp().ticks;
                // Per-PACKET signal level, read before anything else can overwrite it. `rssi_avg` is
                // averaged over the packet just received; the old code read `rssi_inst` AFTER the
                // packet, i.e. the idle channel, which is why back-to-back identical frames on a
                // two-foot link reported values ~30 units apart.
                let rssi = match radio.get_flrc_packet_status().await {
                    Ok(st) => dbm_of(st.rssi_avg()),
                    Err(_) => 0,
                };
                let snr = sense.snr_for(rssi);

                // Fixed format: the chip received exactly FRAME_BYTES. Deliberately NOT
                // `get_rx_pkt_len()` — the generic length register reads garbage in FLRC mode
                // (#108), which is what once made 43-byte frames look like 96-byte ones.
                let mut rxb = [0u8; flrc_link::FRAME_BYTES];
                let read_ok = radio.rd_rx_fifo_to(&mut rxb).await.is_ok();
                // A frame the PHY CRC rejected is not delivered. Whitening plus the in-frame length
                // check would catch most of it anyway, but a corrupt frame reaching the data plane
                // pollutes the dedup ring with a hash of noise.
                if read_ok && !irq.crc_error() {
                    if let Some(n) = flrc_link::unpack_frame(&mut rxb) {
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
                        let payload = &rxb[1..1 + n];
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

                        let mut left_rx = false;
                        if serve_len > 0 {
                            // Content-Store hit: we answer the Interest ourselves, the host never wakes.
                            let _ = lbt_tx(&mut radio, &mut sense, &serve[..serve_len]).await;
                            left_rx = true;
                        }
                        if relay {
                            let _ = lbt_tx(&mut radio, &mut sense, payload).await;
                            left_rx = true;
                        }
                        if left_rx {
                            let _ = radio.set_rx_continous().await;
                        }

                        if deliver {
                            let mut body = [0u8; 8 + flrc_link::PAYLOAD_MAX];
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
    }
}
