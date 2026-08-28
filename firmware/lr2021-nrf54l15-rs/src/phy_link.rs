//! **The PHY dispatcher and the front-end sequence every PHY shares.**
//!
//! One copy of the Semtech-ordered bring-up
//!
//! ```text
//!   StandbyRC -> regulator -> SetPacketType -> SetRfFrequency
//!             -> [ per-PHY modem block ] -> PA -> SetTxParams -> RX path -> Calibrate/CalibFe
//! ```
//!
//! with a hole in the middle where the modulation goes. That shape is [`crate::flrc_link`]'s
//! `LinkState`/`apply` split **generalised**, not duplicated per PHY: the obvious alternative — a
//! copy of the sequence inside each PHY's module — is how three copies drift apart three commits
//! later, and this file exists because that had already happened once between `configure` and
//! `retune`.
//!
//! The per-PHY numbers a host is told about live in [`crate::phy`], where they are host-testable.
//! The per-PHY register writes live in [`crate::flrc_link`], [`crate::lora_link`] and
//! [`crate::lrfhss_link`]. Everything here is either shared or a dispatch.
//!
//! ## Why a PHY switch re-runs the whole sequence
//!
//! `SetPacketType` selects **which modulation and packet registers exist**, so a mode change
//! invalidates the modem configuration, and the front end has to be re-programmed for it — the same
//! reason a retune does, and the reason `CMD_SET_FREQ` used to brick this node's transmitter until
//! `apply` became the single entry point. A switch is a full `apply`, never a partial one.

use embassy_time::{Duration, Timer};
use embedded_hal::digital::OutputPin;
use embedded_hal_async::spi::SpiBus;

use lr2021::radio::{PaLfMode, PacketType, RampTime, RxBoost, RxPath};
use lr2021::status::ChipModeStatus;
use lr2021::system::ChipMode;
use lr2021::{BusyPin, Lr2021, Lr2021Error};

use crate::phy::{self, Phy};
use crate::{flrc_link, lora_link, lrfhss_link};

/// `pa_hf_duty_cycle`, the other half of the HF power pair. **§7.4.1: only 16-31 are authorised**,
/// outside that range the datasheet warns of incorrect output power, excessive current and PA
/// damage. Unused on an LF build.
pub const PA_HF_DUTY: u8 = match option_env!("PHY_PWR") {
    Some(v) if matches!(v.as_bytes(), b"0") => 30,
    Some(v) if matches!(v.as_bytes(), b"6") => 30,
    _ => 16,
};

/// Three front-end calibration points, all on the path in use.
///
/// **4 MHz grid.** `CalibFe` takes frequencies in 4 MHz steps, so a calibration point can only ever
/// land on a multiple of 4 MHz. `GetErrors` reports `RXFREQ_NO_FE_CAL` ("front end calibration was
/// not available for Rx operation with specified RF frequency") on BOTH paths at our original
/// 2477 / 915 MHz — neither of which is a multiple of 4. LF survived it only because 915 MHz is
/// exactly where §6.4 says the chip self-calibrates at boot; HF had no such luck.
///
/// HF brackets the 2.4 GHz ISM band (2448 / 2464 / 2480 MHz, MSB set to select the HF path); LF
/// brackets 900 / 912 / 928 MHz.
pub const CAL_POINTS: [u16; 3] = if phy::IS_HF {
    [0x8000 | 612, 0x8000 | 616, 0x8000 | 620]
} else {
    [225, 228, 232]
};

/// PA ramp time. **`PHY_RAMP` overrides it**, because Ramp2u — the shortest the part offers — was
/// chosen for M5 timing precision and is now a suspect: the SDR shows the HF carrier still settling
/// ~90 kHz over the first 194 µs of a burst, and a 2 µs ramp gives the synthesizer essentially no
/// time between keying the PA and putting data on the air.
///
/// The ramp is also one of the four terms of [`crate::airtime::SCHED_GRAN_NS`], and it is
/// PHY-independent — which is why that granularity is the same on all three modes.
fn ramp_time() -> RampTime {
    match option_env!("PHY_RAMP") {
        Some(v) if matches!(v.as_bytes(), b"16") => RampTime::Ramp16u,
        Some(v) if matches!(v.as_bytes(), b"48") => RampTime::Ramp48u,
        Some(v) if matches!(v.as_bytes(), b"96") => RampTime::Ramp96u,
        Some(v) if matches!(v.as_bytes(), b"192") => RampTime::Ramp192u,
        _ => RampTime::Ramp2u,
    }
}

// ── Hop plan ────────────────────────────────────────────────────────────────────────────────────

/// **The intra-packet hop table the host wrote** (`CMD_SET_HOP`), in a shape both hopping PHYs can
/// be programmed from.
///
/// One representation for LoRa's `SetLoraHopping(period, freqs)` and LR-FHSS's
/// `WriteLrFhssHoppingTable` couples, because they are the same request: a list of carriers and a
/// dwell. Where they differ is in the unit of the dwell — LoRa symbols vs LR-FHSS hop symbols — and
/// that difference is documented on `CMD_SET_HOP` rather than modelled as two types the bridge would
/// have to choose between.
#[derive(Clone, Copy)]
pub struct HopPlan {
    /// Is hopping on? A disabled plan keeps its table so a host can toggle without re-sending it —
    /// but [`PhyState::set_phy`] clears the whole plan on a **PHY change**, because a table written
    /// for one modulation silently re-arming under another is the stale-state bug this pass exists
    /// to remove.
    pub enabled: bool,
    /// Dwell per hop, in symbols.
    pub period: u16,
    n: u8,
    freqs: [u32; phy::MAX_HOPS],
}

impl Default for HopPlan {
    fn default() -> Self {
        Self { enabled: false, period: 0, n: 0, freqs: [0; phy::MAX_HOPS] }
    }
}

impl HopPlan {
    /// The frequencies to hop between — **empty when hopping is off**, which is exactly the argument
    /// both chip commands take to mean "disable".
    pub fn freqs(&self) -> &[u32] {
        if self.enabled {
            &self.freqs[..self.n as usize]
        } else {
            &[]
        }
    }

    /// Replace the plan. `freqs` longer than [`phy::MAX_HOPS`] is truncated, and the caller is
    /// expected to have refused it already ([`phy::check_hop`]) — the truncation is a floor, not a
    /// policy.
    pub fn set(&mut self, enabled: bool, period: u16, freqs: &[u32]) {
        let n = freqs.len().min(phy::MAX_HOPS);
        self.freqs[..n].copy_from_slice(&freqs[..n]);
        self.n = n as u8;
        self.period = period;
        self.enabled = enabled && n > 0;
    }

    /// Forget the table entirely.
    pub fn clear(&mut self) {
        *self = Self::default();
    }
}

// ── PHY state ───────────────────────────────────────────────────────────────────────────────────

/// The modem parameters of whichever PHY is current.
#[derive(Clone, Copy)]
pub enum PhyMode {
    Lora(lora_link::LoraParams),
    Flrc(flrc_link::FlrcParams),
    LrFhss(lrfhss_link::LrFhssParams),
}

/// **The runtime-mutable RF state — the whole of it, across every PHY.**
///
/// [`crate::flrc_link::LinkState`] generalised. It exists for the same measured reason: `CMD_SET_FREQ`
/// used to call `set_rf` from RX-continuous, and after **any** retune every subsequent transmit
/// returned `ok = 0` until the board was reset, because a frequency change invalidates front-end and
/// PA state that was programmed for the old frequency. Holding the entire state in one value and
/// re-applying all of it is what makes that impossible; a PHY change makes it more true, not less,
/// since `SetPacketType` also changes which modem registers exist.
#[derive(Clone, Copy)]
pub struct PhyState {
    /// Carrier, Hz. Must satisfy [`phy::in_band`].
    pub freq_hz: u32,
    /// TX power in the chip's **half-dB register unit** — see [`phy::dbm_to_half_db`]. Deliberately
    /// stored in the register unit, at the one layer that is allowed to know it.
    pub tx_power_half_db: i8,
    /// The current PHY's parameters.
    pub mode: PhyMode,
    /// The host's intra-packet hop table. Applied only by PHYs that have hopping.
    pub hop: HopPlan,
}

impl Default for PhyState {
    /// The build-time link both nodes agree on with no host in the loop: **FLRC**, which is what
    /// every milestone binary and every measurement to date has run.
    fn default() -> Self {
        Self::from_link(&flrc_link::LinkState::default())
    }
}

impl PhyState {
    /// Lift an FLRC [`LinkState`](flrc_link::LinkState) into the general state — the bridge between
    /// twenty existing FLRC binaries and the dispatcher.
    pub fn from_link(link: &flrc_link::LinkState) -> Self {
        Self {
            freq_hz: link.freq_hz,
            tx_power_half_db: link.tx_power_half_db,
            mode: PhyMode::Flrc(link.flrc_params()),
            hop: HopPlan::default(),
        }
    }

    /// Which PHY is current.
    pub fn phy(&self) -> Phy {
        match self.mode {
            PhyMode::Lora(_) => Phy::Lora,
            PhyMode::Flrc(_) => Phy::Flrc,
            PhyMode::LrFhss(_) => Phy::LrFhss,
        }
    }

    /// The chip's `SetPacketType` argument for the current PHY.
    pub fn packet_type(&self) -> PacketType {
        match self.mode {
            PhyMode::Lora(_) => PacketType::Lora,
            PhyMode::Flrc(_) => PacketType::Flrc,
            PhyMode::LrFhss(_) => PacketType::LrFhss,
        }
    }

    /// **Switch PHY, taking that PHY's default parameters and dropping the hop table.**
    ///
    /// Defaults rather than a carried-over guess: the parameter sets do not correspond (an FLRC
    /// bitrate rung is not a spreading factor), so there is nothing to carry. The hop table goes
    /// with them for the same reason — a table written for LoRa symbols must not re-arm under
    /// LR-FHSS hop symbols just because the host switched twice.
    ///
    /// The frequency and the power **do** survive, because they are properties of the channel and
    /// the PA rather than of the modulation, and a host that had tuned the node would not expect a
    /// mode change to move it off channel.
    ///
    /// ★ **Switching to the PHY already running is a no-op on the parameters**, and that is a
    /// correctness requirement rather than a nicety: the host's `exec_idempotent` retries a `SET_*`
    /// up to four times when a reply is slow, so a `CMD_SET_PHY` that reset its own PHY's modulation
    /// on every call would silently wipe a `CMD_SET_MOD` the host had already made — and the retry
    /// that caused it would look like a successful command.
    pub fn set_phy(&mut self, p: Phy) {
        if self.phy() == p {
            return;
        }
        self.mode = match p {
            Phy::Lora => PhyMode::Lora(lora_link::LoraParams::default()),
            Phy::Flrc => PhyMode::Flrc(flrc_link::FlrcParams::default()),
            Phy::LrFhss => PhyMode::LrFhss(lrfhss_link::LrFhssParams::default()),
        };
        self.hop.clear();
    }

    /// TX power in real dBm — what `EVT_INFO`/`EVT_CAP` report, never the register unit.
    pub fn tx_power_dbm(&self) -> i8 {
        self.tx_power_half_db / 2
    }

    /// The largest payload this PHY carries end to end.
    pub fn max_payload(&self) -> usize {
        phy::max_payload(self.phy())
    }

    /// **Airtime of the frame a `payload_len`-byte payload becomes, µs.**
    ///
    /// Per-PHY in two ways that both matter. FLRC's is a **constant of the link** — `PktFormat::Fixed`
    /// puts the whole PDU on air whatever the payload was, which is the property the slot MAC wants —
    /// while LoRa's and LR-FHSS's scale with the payload. And the three answers differ by four orders
    /// of magnitude (230 µs, ~100 ms, ~3 s), which is why `EVT_TX_STARTED` cannot be computed from a
    /// remembered constant after a PHY switch.
    pub fn airtime_us(&self, payload_len: usize) -> u32 {
        let n = payload_len.min(u16::MAX as usize) as u16;
        match &self.mode {
            PhyMode::Flrc(p) => p.airtime_us(),
            PhyMode::Lora(p) => p.airtime_us(n),
            PhyMode::LrFhss(p) => p.airtime_us(n),
        }
    }

    /// [`airtime_us`](Self::airtime_us) as the two big-endian bytes `EVT_TX_STARTED` carries —
    /// whole milliseconds, rounded UP and never 0.
    pub fn airtime_ms_be(&self, payload_len: usize) -> [u8; 2] {
        crate::airtime::airtime_ms_ceil(self.airtime_us(payload_len)).to_be_bytes()
    }
}

// ── The shared sequence ─────────────────────────────────────────────────────────────────────────

/// Program the **entire** RF chain from `st`, in Semtech's validated order, ending in Standby RC.
///
/// Safe to call at any time and from any chip mode: it drops to Standby RC first, which is what
/// `Calibrate` and `CalibFe` require ("does not work if device is in Rx or Tx mode") and what makes
/// a retune out of RX-continuous legal. It does **not** re-arm RX and does not touch DIO routing —
/// the caller owns both, because `m5_tx` deliberately repurposes DIO8 after `configure` and an
/// `apply` that reset it would silently undo that.
pub async fn apply<O, SPI, M>(radio: &mut Lr2021<O, SPI, M>, st: &PhyState) -> Result<(), Lr2021Error>
where
    O: OutputPin,
    SPI: SpiBus<u8>,
    M: BusyPin,
{
    // **NO TCXO ON THIS BOARD — do not enable TCXO mode.** Proven, not assumed: `SetTcxoMode` with a
    // real (non-zero) start_time returns **CmdFail**, even from Standby RC, which §6.11.3 names as
    // the command's only valid mode. A non-zero start_time is a *timeout* for detecting the 32 MHz
    // oscillator, so failing it means no TCXO clock appeared. The shield devicetree lists
    // `tcxo-voltage = 1.8V` alongside `tcxo-wakeup-time = <0>`, and the zero is the operative half.
    //
    // Kept as a comment rather than a disabled call because a `set_tcxo(.., 0)` reads like it enables
    // something and does the opposite — that silent no-op was committed once already as a fix.

    // **§6.3.18: `SetRegMode` defaults to SIMO_OFF (LDO) and must be issued in Standby RC.**
    // The shield devicetree specifies `reg-mode = DCDC`, and every PA figure in the datasheet is
    // characterised at "3.3 V SIMO".
    radio.set_chip_mode(ChipMode::StandbyRc).await?;
    // `set_regulator_mode(true)` emits `SimoUsage::Auto` == 2 == the datasheet's **SIMO_NORMAL**.
    // The crate's enum names do not match Table 6-26 — it calls 1 `All` and 3 `Vdcc` where the
    // datasheet marks both **RFU** — so this is right by value, not by name.
    let _ = radio.set_regulator_mode(true).await;

    // ── Semtech's own order, from `ralf_lr20xx_setup_flrc()` in Lora-net/usp ────────────────────
    //   set_pkt_type -> set_rf_freq -> [modem] -> set_tx_cfg(power) -> rx_path -> calibrate
    //
    // ★ `SetPacketType` FIRST, and it is the line this whole pass is about: it selects which
    // modulation and packet registers exist, so it has to precede every one of them. It used to be
    // called once at bring-up with a compile-time constant; it is now driven from `st`.
    radio.set_packet_type(st.packet_type()).await?;
    radio.set_rf(st.freq_hz).await?;

    // ── the per-PHY modem block ─────────────────────────────────────────────────────────────────
    match &st.mode {
        PhyMode::Flrc(p) => flrc_link::apply_modem(radio, p).await?,
        PhyMode::Lora(p) => lora_link::apply_modem(radio, p, &st.hop).await?,
        PhyMode::LrFhss(p) => lrfhss_link::apply_modem(radio, p).await?,
    }

    // HF front end for the 2.4 GHz port: PA on the HF path, RX on the HF path with boost.
    if !phy::IS_HF {
        // LF PA: the crate's HF helper hard-codes the HF PA selection, so the sub-GHz path needs the
        // explicit form. Duty cycle / slices copied from the HF helper's own defaults (6, 7).
        radio.set_pa_lf(PaLfMode::LfPaFsm, 6, 7).await?;
    } else {
        // **The crate's `set_pa_hf()` never sets `pa_hf_duty_cycle`.** It calls the 4-byte
        // `set_pa_config_cmd(HfPa, LfPaFsm, 6, 7)` — all three of those are the *LF* fields — while
        // the spec defines a 5th byte, `pa_hf_duty_cycle` (5 bits, default 16), described as
        // controlling "the duty cycle and maximum output power of the PA in HF mode".
        //
        // **Configured per datasheet Table 7-18, not by sweeping.** §7.4.1: "Only values from 16-31
        // are authorized to avoid risk of aging the power amplifier", and non-optimized PA parameters
        // risk "incorrect output power, excessive current consumption, PA damage, regulatory
        // non-compliance". §1.5.2 + Table 7-18: `tx_power` and `pa_hf_duty_cycle` are a **matched
        // pair**, not two independent knobs. §7.4.3: `tx_power` is in 0.5 dB steps.
        //
        // A runtime `CMD_SET_PWR` moves `tx_power` and leaves [`PA_HF_DUTY`] at its build value,
        // because only three rows of the matched pair are known.
        let mut cmd = [0u8; 5];
        cmd[0] = 0x02;
        cmd[1] = 0x02;
        // pa_sel = 1 (HF), pa_lf_mode = 0, pa_lf_duty_cycle = 6, pa_lf_slices = 7 — the datasheet's
        // stated values for "LF PA not used".
        cmd[2] = (1 << 7) | (6 << 4);
        cmd[3] = 7;
        cmd[4] = PA_HF_DUTY & 0x1f;
        radio.cmd_wr(&cmd).await?;
    }
    // **Ramp2u, not Ramp16u** (#104 lever 3). The PA ramp sits between the TX trigger and the first
    // on-air symbol, so its duration is pure transmit-instant offset — and any variation in it is
    // transmit-instant jitter. One call for both paths, so they cannot disagree about the ramp.
    radio.set_tx_params(st.tx_power_half_db, ramp_time()).await?;

    // **RX boost: §7.3.2's recommended defaults, 0 on LF and 4 on HF** — not the shield
    // devicetree's `rx-boost-cfg = 7`, which is what the datasheet uses for its *sensitivity*
    // figures. Two boards two feet apart sit at −33 dBm, ~67 dB above that, and maximum LNA boost
    // into a signal that strong overloads the front end: the receiver syncs fine and the demodulator
    // distorts *during* the packet. Boost is a link-budget decision, not a constant — raise it when
    // the link is weak. `PHY_BOOST` sweeps it.
    let path = if phy::IS_HF { RxPath::HfPath } else { RxPath::LfPath };
    let boost = match option_env!("PHY_BOOST") {
        Some(v) if matches!(v.as_bytes(), b"0") => RxBoost::Off,
        Some(v) if matches!(v.as_bytes(), b"3") => RxBoost::B3,
        Some(v) if matches!(v.as_bytes(), b"4") => RxBoost::B4,
        Some(v) if matches!(v.as_bytes(), b"5") => RxBoost::B5,
        Some(v) if matches!(v.as_bytes(), b"7") => RxBoost::Max,
        _ if phy::IS_HF => RxBoost::B4,
        _ => RxBoost::Off,
    };
    radio.set_rx_path(path, boost).await?;
    dcdc_workaround(radio).await;

    // ── Calibration, LAST and from Standby RC ───────────────────────────────────────────────────
    //
    // §6.4: the chip boots with image rejection calibrated **at 915 MHz**, and "if operating at
    // another frequency the image calibration procedure must be restarted using command Calibrate
    // ... necessary if there is a frequency change > 10MHz". §6.4.1 adds PLL and AAF for changes
    // > 50MHz.
    //
    // Both commands take no frequency and calibrate for the *current* configuration, so they run
    // last — after the frequency, the modulation params (the AAF is sized by bandwidth, which is why
    // a PHY switch must re-run them) and the RX path. Neither can be issued in Rx or Tx, so drop to
    // Standby RC first rather than assuming which mode we are in.
    //
    // **Verified by `m113_errors`:** with this sequence `GetErrors` is clean through configure, TX
    // and RX entry. Without it the chip reports `RXFREQ_NO_FE_CAL` on entering RX.
    radio.set_chip_mode(ChipMode::StandbyRc).await?;
    radio.calibrate(true, true, true, true, false, false).await?; // PA_OFF, MU, AAF, PLL
    radio.calib_fe(&CAL_POINTS).await?;

    Ok(())
}

/// **DCDC switcher workaround — `lr20xx_workarounds_dcdc_configure()` from Semtech's driver.**
///
/// Not in the datasheet; it exists only in `lr20xx_workarounds.c` in Lora-net/usp, and the driver
/// calls it automatically at the end of **every** `set_*_modulation_params` and after `set_rx_path`,
/// via `LR20XX_WORKAROUNDS_CONDITIONAL_APPLY_AUTOMATIC_DCDC_CONFIGURE`. Which is why it lives here
/// rather than in one PHY's module: it is required on all of them.
///
/// It is required whenever the DCDC regulator is in use — which, since we enable SIMO_NORMAL,
/// includes us.
///
/// ```text
///   ana_dec  = (reg 0x00F40200 >> 8) & 0x7
///   is_rx_hf = (reg 0x00F40430 & 0x3) == 1
///   if !is_rx_hf && (ana_dec == 1 || ana_dec == 2)  -> switcher rise 11, fall 13
///   else                                           -> switcher rise 15, fall 15
/// ```
///
/// Errors are swallowed: this is a best-effort register poke on an undocumented address, and a
/// failure here must not take down a link that otherwise works.
pub async fn dcdc_workaround<O, SPI, M>(radio: &mut Lr2021<O, SPI, M>)
where
    O: OutputPin,
    SPI: SpiBus<u8>,
    M: BusyPin,
{
    const ADC_CTRL: u32 = 0x00F4_0200;
    const RX_PATH: u32 = 0x00F4_0430;
    const SWITCHER: u32 = 0x00F2_0024;
    const RISE_MASK: u32 = 0xF << 20;
    const FALL_MASK: u32 = 0xF << 16;

    let ana_dec = match radio.rd_reg(ADC_CTRL).await {
        Ok(v) => (v >> 8) & 0x7,
        Err(_) => return,
    };
    let is_rx_hf = match radio.rd_reg(RX_PATH).await {
        Ok(v) => (v & 0x3) == 1,
        Err(_) => return,
    };

    let (rise, fall) = if !is_rx_hf && (ana_dec == 1 || ana_dec == 2) { (11u32, 13u32) } else { (15u32, 15u32) };
    let _ = radio.wr_reg_mask(SWITCHER, RISE_MASK, rise << 20).await;
    let _ = radio.wr_reg_mask(SWITCHER, FALL_MASK, fall << 16).await;
}

/// **The chip's status as one byte** — `chip_mode | (cmd_status << 4)`, the packing `EVT_INFO[0]`
/// and [`EVT_PHY_ERR`](crate::serial::EVT_PHY_ERR) both carry.
///
/// Read from the driver's cached status word, i.e. **the status the failing command itself
/// returned**, with no further SPI traffic that could overwrite it. That is what makes
/// `EVT_PHY_ERR` the chip's literal answer rather than a later, unrelated status.
///
/// ⚠ Call [`clear_errors`] only AFTER reading this — it is SPI traffic and overwrites the cache.
pub fn status_byte<O, SPI, M>(radio: &Lr2021<O, SPI, M>) -> u8
where
    O: OutputPin,
    SPI: SpiBus<u8>,
    M: BusyPin,
{
    let st = radio.status();
    let mode = match st.chip_mode() {
        ChipModeStatus::Sleep => 0,
        ChipModeStatus::Rc => 1,
        ChipModeStatus::Xosc => 2,
        ChipModeStatus::Fs => 3,
        ChipModeStatus::Rx => 4,
        ChipModeStatus::Tx => 5,
        ChipModeStatus::Unknown => 8,
    };
    mode | ((st.cmd() as u8) << 4)
}

/// **`ClearErrors` (datasheet §6.7.3, opcode `0x01 0x11`) — clear every pending error flag at once.**
///
/// ☠ **MEASURED 2026-08-28: without this, one refused command WEDGES the node.** Entering LR-FHSS
/// arms RX, the chip refuses it (`CMD_FAIL`), and from then on *nothing* works: `CMD_SET_PHY` back to
/// FLRC or LoRa fails, `CMD_TX` returns `ok=0`, and only a board reset recovers. The revert path in
/// the `CMD_SET_PHY` handler could not save it either, because the revert is itself an `apply` and it
/// inherits the same latched condition — so a failure that was supposed to be contained became a
/// one-way door out of a mode we entered deliberately in order to measure it.
///
/// The chip does not clear these on its own: §6.7.3 says the command "clears all error flags that are
/// pending in the device… clears all error conditions at once". The **vendor crate does not wrap it**
/// (only `get_errors`), which is why this is a raw two-byte command rather than a driver call — an
/// unexposed command is not an unavailable one.
///
/// ⚠ Two ordering rules, both load-bearing:
/// * This is SPI traffic and it overwrites the driver's cached status, so read [`status_byte`]
///   FIRST if you intend to report the failing command's own answer.
/// * §6.7.3: "Calling ClearErrors does not clear the Error IRQ. The IRQ has to be cleared explicitly
///   with the ClearIrq command." We do not gate on the error IRQ here, so that is noted, not handled.
pub async fn clear_errors<O, SPI, M>(radio: &mut Lr2021<O, SPI, M>) -> Result<(), Lr2021Error>
where
    O: OutputPin,
    SPI: SpiBus<u8>,
    M: BusyPin,
{
    radio.cmd_wr(&[0x01, 0x11]).await
}

/// **Last-resort recovery: hard-reset the chip and re-apply `st` from scratch.**
///
/// ☠ **This exists because of a MEASURED one-way door.** Entering LR-FHSS arms RX, the chip refuses
/// it (`CMD_FAIL` from StandbyRc), and afterwards *every* command fails: `CMD_SET_PHY` back to FLRC
/// or LoRa, and `CMD_TX` (`ok=0`). The ordinary revert cannot help, because the revert is itself an
/// [`apply`] and inherits whatever state is blocking it. **[`clear_errors`] was tried first and did
/// NOT recover it** — so this is not a pending error flag, and the mechanism is still unidentified;
/// what is established is that only an NRESET brings the part back (verified: after a board reset the
/// node reports FLRC, `max_payload` 47, and `CMD_TX ok=1`).
///
/// So the rule this enforces is deliberately about the OUTCOME, not the cause: **a PHY switch must
/// never leave the node unable to transmit and unable to switch.** Reaching for a reset is heavy and
/// it is the honest floor until someone identifies what the LR-FHSS engine holds.
///
/// Costs ~20 ms of NRESET plus a full re-apply, and it drops any armed RX — acceptable on a path
/// that is only reached when the alternative is a dead node.
pub async fn reset_and_apply<O, SPI, M>(
    radio: &mut Lr2021<O, SPI, M>,
    st: &PhyState,
) -> Result<(), Lr2021Error>
where
    O: OutputPin,
    SPI: SpiBus<u8>,
    M: BusyPin,
{
    radio.reset().await?;
    Timer::after(Duration::from_millis(50)).await;
    // **The wake matters, and it was MEASURED.** A bare `reset()` + `apply` left the part reporting
    // `chip_mode = Sleep` and still refusing everything; the boot path recovers it and the only thing
    // boot does in between is read the version. So the first command after NRESET is a read whose
    // answer we discard — it exists to bring the part up, exactly as `main` does it.
    let _ = radio.get_version().await;
    apply(radio, st).await
}

// ── Transmit ────────────────────────────────────────────────────────────────────────────────────

/// Everything a transmit needs **before** key-up, per PHY: pack the frame, settle the synthesizer,
/// load the chip. Returns false if the payload does not fit, having touched nothing.
///
/// Split from the key-up for two reasons that both matter to the host:
///
/// * a scheduled transmit must do all of this *ahead* of the instant, so that when the instant
///   arrives the only thing left is one SPI command — everything slow that happens after the
///   deadline is scheduling error;
/// * `EVT_TX_STARTED` has to be emitted **between** the two, after the last point a transmit can be
///   refused and before the frame is keyed up.
///
/// The three PHYs load the chip differently and the difference is the whole reason this is
/// dispatched: FLRC writes a fixed, whitened PDU into the TX FIFO; LoRa re-programs `payload_len`
/// (its header is explicit, so the length goes on air) and writes the bare payload; LR-FHSS does not
/// use the FIFO write at all — `LrFhssBuildFrame` takes the payload as command data and encodes it.
pub async fn tx_stage<O, SPI, M>(
    radio: &mut Lr2021<O, SPI, M>,
    st: &PhyState,
    payload: &[u8],
) -> bool
where
    O: OutputPin,
    SPI: SpiBus<u8>,
    M: BusyPin,
{
    if payload.len() > st.max_payload() {
        return false;
    }
    // Measured with a B210 (#108): going straight from standby to TX leaves the PLL converging under
    // the first 27 kHz of the payload. PHY-independent — the synthesizer does not know what modem is
    // downstream of it.
    flrc_link::settle_before_tx(radio).await;
    // Clear first: `wr_tx_fifo_from` APPENDS, so a frame left by a transmit that never fired would
    // otherwise be prepended to this one.
    let _ = radio.clear_tx_fifo().await;
    match &st.mode {
        PhyMode::Flrc(_) => {
            let Some(frame) = flrc_link::build_frame(payload) else {
                return false;
            };
            radio.wr_tx_fifo_from(&frame).await.is_ok()
        }
        PhyMode::Lora(p) => {
            // `payload_len` is the TRANSMIT length in LoRa too, so it moves per frame. The receiver
            // is unaffected: with an explicit header it takes the length off the air.
            if radio.set_lora_packet(&p.packet(payload.len() as u8)).await.is_err() {
                return false;
            }
            radio.wr_tx_fifo_from(payload).await.is_ok()
        }
        PhyMode::LrFhss(p) => {
            if lrfhss_link::build_frame(radio, p, st.hop.enabled, payload).await.is_err() {
                return false;
            }
            // ★ The host's hop table is written AFTER the build, because `LrFhssBuildFrame`
            // configures the internal table itself and would overwrite one written before it. See
            // `lrfhss_link::write_table`.
            if st.hop.enabled {
                let n = payload.len().min(u16::MAX as usize) as u16;
                let blocks = crate::airtime::lrfhss_block_count(p.cr as u8, n);
                lrfhss_link::write_table(radio, true, n, blocks, st.hop.freqs(), st.hop.period)
                    .await
                    .is_ok()
            } else {
                true
            }
        }
    }
}

// ── Receive ─────────────────────────────────────────────────────────────────────────────────────

/// One received frame's metadata, in the units `EVT_RX` carries: **real dBm and real dB**.
pub struct RxInfo {
    /// Payload bytes written into the caller's buffer.
    pub len: usize,
    pub rssi_dbm: i16,
    /// SNR in dB, or `None` where this PHY has no SNR register and the caller must fall back to
    /// `packet RSSI − noise floor`.
    pub snr_db: Option<i16>,
}

/// Spec: actual signal power is `−value/2` dBm — for `rssi_inst`, `rssi_avg`, `rssi_pkt` and every
/// CCA result alike. Every RSSI this firmware puts on the wire goes through here.
pub fn dbm_of(raw: u16) -> i16 {
    -((raw as i16) / 2)
}

/// **Read the frame the chip just received**, per PHY, into `buf`.
///
/// Per-PHY in three separate ways, each of which was a bug on some node at some point:
///
/// * **length** — FLRC reads exactly `FRAME_BYTES` and deliberately does *not* use
///   `get_rx_pkt_len()`, whose generic length register reads garbage in FLRC mode (#108, where it
///   made 43-byte frames look like 96-byte ones). LoRa's length comes from
///   `GetLoraPacketStatus.pkt_length`, which is the explicit header's own field.
/// * **signal** — FLRC has `rssi_avg` and **no SNR register at all**; LoRa has both, with SNR in
///   quarter-dB two's complement. Reporting a LoRa SNR through FLRC's noise-floor estimate, or vice
///   versa, would make two nodes' `EVT_RX` incomparable.
/// * **framing** — FLRC un-whitens and strips the in-frame length byte; LoRa's payload is the frame.
///
/// `None` means nothing usable was read; the caller drops the frame rather than delivering padding.
pub async fn rx_read<O, SPI, M>(
    radio: &mut Lr2021<O, SPI, M>,
    st: &PhyState,
    buf: &mut [u8],
) -> Option<RxInfo>
where
    O: OutputPin,
    SPI: SpiBus<u8>,
    M: BusyPin,
{
    match &st.mode {
        PhyMode::Flrc(_) => {
            let rssi = match radio.get_flrc_packet_status().await {
                // `rssi_avg` is averaged over the packet just received. Reading `rssi_inst` after the
                // packet measures the idle channel instead, which is why back-to-back identical
                // frames on a two-foot link once reported values ~30 units apart.
                Ok(st) => dbm_of(st.rssi_avg()),
                Err(_) => 0,
            };
            let mut frame = [0u8; flrc_link::FRAME_BYTES];
            radio.rd_rx_fifo_to(&mut frame).await.ok()?;
            let n = flrc_link::unpack_frame(&mut frame)?;
            if n > buf.len() {
                return None;
            }
            buf[..n].copy_from_slice(&frame[1..1 + n]);
            Some(RxInfo { len: n, rssi_dbm: rssi, snr_db: None })
        }
        PhyMode::Lora(_) => {
            let status = radio.get_lora_packet_status().await.ok()?;
            let n = status.pkt_length() as usize;
            if n == 0 || n > buf.len() {
                // Drain it anyway. A frame we cannot deliver (LoRa's `pkt_length` is a u8 and can
                // exceed our 247-byte cap) still occupies the FIFO, and leaving it there prepends
                // its bytes to the NEXT frame — a corruption that outlives the frame that caused it.
                let _ = radio.clear_rx_fifo().await;
                return None;
            }
            // "Actual value is −rssi_pkt/2 (dBm)"; "actual SNR in dB is snr_pkt/4", two's complement.
            let rssi = dbm_of(status.rssi_pkt());
            let snr = (status.snr_pkt() as i16) / 4;
            radio.rd_rx_fifo_to(&mut buf[..n]).await.ok()?;
            Some(RxInfo { len: n, rssi_dbm: rssi, snr_db: Some(snr) })
        }
        PhyMode::LrFhss(_) => {
            // ⚠ **Unproven path.** There is no `GetLrFhssPacketStatus` on this part, so the length
            // comes from the generic RX length register and the level from `rssi_inst` — the honest
            // best available, and only reachable at all if `SetRxContinuous` was accepted in this
            // mode (see `lrfhss_link::arm_rx`). It exists so that "does LR-FHSS receive?" can be
            // ANSWERED on air rather than argued about; do not read a frame arriving here as
            // anything but the measurement it is.
            let n = radio.get_rx_pkt_len().await.ok()? as usize;
            if n == 0 || n > buf.len() {
                let _ = radio.clear_rx_fifo().await;
                return None;
            }
            let rssi = radio.get_rssi_inst().await.map(dbm_of).unwrap_or(0);
            radio.rd_rx_fifo_to(&mut buf[..n]).await.ok()?;
            Some(RxInfo { len: n, rssi_dbm: rssi, snr_db: None })
        }
    }
}

/// **Re-arm continuous RX after a transmit** — per PHY, because LoRa has to put `payload_len` back.
///
/// A transmit drops continuous RX on every PHY; a node that does not re-arm goes deaf after its
/// first frame, which reads as "the link died".
///
/// The LoRa step matters: [`tx_stage`] programs `payload_len` to the *transmitted* length, and while
/// an explicit header means the receiver takes the length off the air, leaving the register at 3
/// bytes because that is what we last sent is exactly the kind of state that turns into a
/// half-received frame under a configuration nobody remembers setting. One extra command per
/// re-arm buys that back.
pub async fn arm_rx<O, SPI, M>(radio: &mut Lr2021<O, SPI, M>, st: &PhyState) -> Result<(), Lr2021Error>
where
    O: OutputPin,
    SPI: SpiBus<u8>,
    M: BusyPin,
{
    if let PhyMode::Lora(p) = &st.mode {
        radio.set_lora_packet(&p.packet(phy::LORA_PAYLOAD_MAX as u8)).await?;
    }
    match st.mode {
        // Routed through the LR-FHSS module so the "does this arm at all?" question has exactly one
        // call site, and its answer reaches the host as EVT_PHY_ERR rather than a swallowed error.
        PhyMode::LrFhss(_) => lrfhss_link::arm_rx(radio).await,
        _ => radio.set_rx_continous().await,
    }
}
