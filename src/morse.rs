//! Morse Micro MM6108 (802.11ah / S1G) control surface, via the vendor `morse_cli` plus the two
//! things `morse_cli` cannot reach: the driver's debugfs TX-power knob and its module parameters.
//!
//! [`Mac80211Knobs`](crate::RadioKnobs)'s generic `iw dev … set channel` **cannot tune this radio**:
//! on a Morse monitor vif it returns `-16 EBUSY`, measured. The driver exposes tuning only through
//! its nl80211 vendor command, which the vendor tool `morse_cli` wraps. So a Morse-specific
//! [`RadioKnobs::set_channel`] is not a nicety — without it the bearer cannot be tuned at all.
//!
//! # What S1G tuning actually requires (measured, 2026-08-28)
//!
//! An S1G channel is **four** numbers, not one, and getting the fourth wrong is silent:
//!
//! * **operating frequency** in kHz (e.g. `906000`) — the real sub-GHz frequency. Do not use the
//!   5 GHz numbers `iw` reports; those are the driver's shadow mapping, not the air.
//! * **operating bandwidth** (1/2/4/8 MHz) — the width of the whole channel.
//! * **primary bandwidth** (usually 1 MHz) — the width of the primary sub-channel.
//! * ★ **primary channel index** — *which* 1 MHz sub-channel inside the operating width is primary.
//!
//! That last one decides whether you hear anything at all. Sweeping it against a known-good
//! transmitter (an NRC7292 AP beaconing at 906 MHz / 4 MHz) gave, on the same radio, same second:
//!
//! ```text
//! primary index 0 ->  0 frames / 8 s        primary index 2 -> 0
//! primary index 1 -> 10 frames / 8 s  ★     primary index 3 -> 0
//! ```
//!
//! Operating bandwidth alone is not enough: at `-o 1` the same AP yielded ~1 frame in 12 s (noise).
//! A receiver reading about −109 dBm is simply on the wrong channel; a correct one saw −46 dBm.
//!
//! The trait method cannot say any of that, so [`MorseKnobs::set_channel_s1g`] takes the four real
//! parameters and is the honest API. What [`RadioKnobs::set_channel`] *can* now do is recover three
//! of the four from the channel **number**, because in S1G the width is a property of the channel
//! (see [`us_s1g_channel`]); the primary index is the one that has to come from somewhere else, and
//! it comes from [`MorseKnobs::with_primary_index`].
//!
//! # Traps that produce silent failure
//!
//! * **The interface must be UP.** With it down, every vendor command fails
//!   `NL80211, code -100 (ENETDOWN)` / `Failed to rcvmsgs`. [`MorseKnobs::set_channel_s1g`]
//!   surfaces that as an error rather than letting it look like success.
//! * **Settings are not sticky.** A radio observed here had drifted to primary index 3 between
//!   measurements. Every setter in this module that has a read-back **verifies it**, and
//!   `set_channel_s1g` gets its verification for free: `morse_cli channel -c …` re-reads and prints
//!   the chip's channel after setting it (`channel.c` issues `GET_CHANNEL_FULL` unconditionally
//!   after a successful `SET_CHANNEL`), so the output of the set *is* the read-back.
//! * ☠ **`morse_cli` hangs.** It has no timeout of its own and blocks forever when the chip is
//!   wedged, which on this bench has meant a stuck process holding the transport. Every invocation
//!   here goes through one private runner, which kills the child at
//!   [`MorseKnobs::with_timeout`] (default [`DEFAULT_CLI_TIMEOUT`]) and says so.
//! * **An invalid channel combination is not a "Failed to …" line.** `channel.c` prints
//!   `Invalid combination of parameters - freq=…` and returns `MORSE_RET_SET_INVALID_CHAN_CONFIG`;
//!   the previous version of this module matched only `Failed to`, so that case read as success.
//!   `cli_error` now matches it, and the runner also checks the **exit status** — which
//!   on `morse_cli` is meaningful (`morsectrl.c:418` remaps the handler's return into 0..254),
//!   unlike the NRC7292's `cli_app`, which always exits 0.
//!
//! # The control surface, and what is deliberately absent
//!
//! Everything below was read out of the vendor sources on this machine
//! (`morse_cli` 1.16.4/1.17.9, `morse_driver` at both releases) or measured on the bench. Nothing
//! is inferred from a command name.
//!
//! **Implemented as `RadioKnobs`:** [`set_channel`](RadioKnobs::set_channel),
//! [`set_tx_power_dbm`](RadioKnobs::set_tx_power_dbm),
//! [`set_tx_hold`](RadioKnobs::set_tx_hold),
//! [`set_edcca_ignore`](RadioKnobs::set_edcca_ignore) (as an explicit refusal — see below),
//! [`set_bandwidth_khz`](RadioKnobs::set_bandwidth_khz) (also an explicit refusal).
//!
//! **Implemented as native methods, because the HAL has no seam for them** — each one is a real,
//! response-echoing actuator, and routing any of them through a trait method would mean fabricating
//! the fields that method's return type demands:
//!
//! * [`MorseKnobs::set_mpsw`] — Minimum Packet Spacing Window, the airtime shaper. The direct
//!   analogue of the NRC7292's `set tx_time`, and **not** a contention window: it has no
//!   `cw_min`/`cw_max`/`aifs`/`slot_us` to put in a [`ContentionApplied`](ndn_radio_hal::ContentionApplied).
//! * [`MorseKnobs::duty_cycle`] / [`MorseKnobs::set_duty_cycle`] / [`MorseKnobs::duty_cycle_airtime_us`]
//!   — a settable regulatory airtime ceiling. `RadioCapability::duty_cycle_max` is read-only.
//! * [`MorseKnobs::set_ampdu`] / [`MorseKnobs::set_max_ampdu_len`] — aggregation, the direct
//!   actuator for "make my burst fit the slot I own".
//! * [`MorseKnobs::set_tx_packet_lifetime_us`] — firmware-side TX expiry (50–500 ms). It *drops*
//!   what it cannot send in time, which is the complement of
//!   [`set_tx_hold`](RadioKnobs::set_tx_hold), which holds.
//! * [`MorseKnobs::tx_block`] — read back the transmit gate
//!   [`set_tx_hold`](RadioKnobs::set_tx_hold) writes.
//! * [`MorseKnobs::set_bss_color`] — the 3-bit S1G spatial-reuse identifier.
//!
//! **Left at the trait default, with the mechanism that was looked for and not found:**
//!
//! * `set_tx_power` (index scale) — no TXAGC index exists anywhere in the CLI, the driver, or the
//!   command header. This radio's power axis is dBm-native, which is the *good* case.
//! * `set_contention` — the firmware genuinely takes `aci/aifs/cw_min/cw_max/txop`
//!   (`.conf_tx` → `morse_cmd_cfg_qos`), and programming it is MEASURED worth 8.0 → 9.5 Mbit/s.
//!   But there is **no userspace path on a monitor vif**: nl80211 `TXQ_PARAMS` is AP-iftype-only,
//!   `iw` has no verb and `morse_cli` has no verb. It is programmed inside the patched driver or by
//!   `hostapd_s1g`. Until a raw nl80211 client and an AP vif exist, this stays defaulted.
//!   ⚠ `morse_cli channel -c …` WIPES the injection EDCA — re-assert it after every retune.
//! * `set_edcca_threshold_dbm` — there is **no** CCA/energy-detect threshold in `morse_cli`, in
//!   `morse_commands.h`, or in the driver. A real asymmetry with the NRC7292, which has
//!   `set cca_thresh {-100..-35 dBm}`: the spatial-reuse pair (back off power *and* raise the defer
//!   floor) cannot be completed on this part.
//! * `read_ofdm_counters` / `read_tx_counters` — a counter surface exists (debugfs `mcs_stats`,
//!   `tx_status`, `page_stats`; `morse_cli stats` against the telemetry firmware) but **no field
//!   has been verified to mean "the PHY began to demodulate and failed"**, which is the entire
//!   value of the method. `page_stats`' `Data Tx:` counts host→chip handoffs and must not be
//!   confused with the MAC→baseband `tx_en`/`tx_on` pair `read_tx_counters` specifies.
//! * `read_channel_activity` — ☠ the one that would have looked fine and reported a constant.
//!   `iw dev … survey dump` returns real numbers (`.get_survey`, and the driver stores firmware
//!   *busy* time into the field it reports as `time_rx`), but the record is refreshed only at
//!   `morse_mac_change_channel` and `sw_scan_complete`. The method's contract is a *free-running*
//!   counter to be differenced over a window; differencing two identical snapshots reports zero
//!   occupancy on a busy channel.
//! * `set_tx_csd` — single-chain part. `set_phy` — S1G only, no runtime modulation switch;
//!   `tx_polar` is an undocumented byte in the header's "Temporary commands" block and is **not**
//!   known to be one. `set_hop_plan` — no sequencer. `set_rx_gain` — no AGC posture control.
//!   `set_spreading_factor` / `set_coding_rate` — LoRa concepts.
//! * `configure_name_filter` — `whitelist` is a fixed IPv4/LLC 5-tuple struct feeding the standby
//!   wake path and cannot express a 16-byte Bloom mask under any encoding. The real candidate is
//!   **APF** (driver `apf.c`: `GET_PACKET_FILTER_CAPABILITIES` / `SET_PACKET_FILTER` /
//!   `READ_PACKET_FILTER_DATA`, present in 1.16.4) — bytecode over frame bytes, so a prefix-set
//!   compare is expressible in principle. It is not wired because the handlers are on the
//!   wiphy/fullmac path while we run mac80211, `morse_cli` has no APF verb, whether the program
//!   runs on all RX or only in the suspend path is UNVERIFIED, and the chip-reported maximum
//!   program length is unknown — so whether eight masks even fit is unknown.
//!
//! ★ `set_tx_hold` used to be listed here as "no hold/release gate exists". **That was wrong**, and
//! the correction is the `TX_BLOCK` generic parameter — see
//! [`set_tx_hold`](RadioKnobs::set_tx_hold) for the mechanism, the measurement and the two
//! prerequisites (a `morse_cli` patched with the `tx_block` entry, and the knowledge that this gate
//! is COARSE). `duty_cycle` burst mode remains what it always was, a *budget* rather than a gate.
//! ⚠ A caller still has no machine-readable way to ask whether a hold took: `RadioCapability` has a
//! `power_actuated` flag and no `tx_hold_actuated` counterpart, so on an unpatched `morse_cli` the
//! only signal is the `Err` this method returns. Stated as a HAL gap rather than papered over.
//!
//! ☠ **Two commands that must never be called from here.** `morse_cli power` is not a TX-power
//! knob: it is `power hibernate` → `MORSE_CMD_ID_FORCE_POWER_MODE`, and its own help says it
//! "requires reset to recover the chip" — calling it on a live radio costs a physical power cycle.
//! `morse_cli medium_eval {enable|disable}` *sounds* like an EDCCA bypass, but it is a bare enable
//! byte at command id `0x811C`, inside the block the header labels "Test commands starting at
//! 0x8000", with no documentation of what it does. Guessing and wiring it is precisely the pattern
//! that has cost this bench power cycles.
//!
//! # Why there is no `RadioTime`
//!
//! The MM6108 has a microsecond hardware clock — `mm6108.c` defines
//! `MM6108_REG_CLINT_MTIME_0/1_ADDR` (`0x0200bff8`/`0x0200bffc`) — and the firmware carries
//! `MORSE_CMD_ID_GET_TSF` (`0x0028`, returning both `now_tsf` and `now_chip_ts`) and
//! `MORSE_CMD_ID_SET_OFFSET_TSF` (`0x003A`). It is therefore *more* steerable in principle than the
//! NRC7292, whose TSF mirror MEASURED as unwritable. None of that is reachable:
//!
//! * **No host path issues either command.** Grepping the whole `morse_cli` tree finds no verb for
//!   `0x0028` or `0x003A`, and the driver exports no debugfs hook for them. A capability with no
//!   caller-reachable transport is not a capability.
//! * **`morse_cli stats` is not a substitute.** It does expose a "System uptime (usec)" counter, but
//!   (a) it requires the chip to be running the **telemetry firmware** `mm6108-tlm.bin` — with stock
//!   firmware it returns `-19 ENODEV` — so the clock would exist only in a configuration that is not
//!   the operational one; (b) it requires `-s <firmware image>` because the stat *names* are read
//!   out of the image, so the field cannot even be named from source on this machine; (c) a read is
//!   a full 412-line dump through a shelled-out process, against the NRC7292's single-word read; and
//!   (d) decisively, **nothing establishes that it is the same clock domain as the per-frame RX
//!   stamps**, which is the only property that makes `read_clock` useful — the NRC7292's clock
//!   earned that claim by bracketing an on-air stamp between two reads, and no equivalent
//!   measurement exists here.
//!
//! The radio's per-frame radiotap TSFT is real and already plumbed, but it belongs to whatever owns
//! the `morse0` capture socket, not to this control object — and its *hardware-ness* is itself
//! UNVERIFIED (the vendor header warns that monitor mode may use the chip's local timer and that
//! "currently TSF is not implemented"). So: no `RadioTime` here, deliberately.

use std::fs;
use std::io::Read;
use std::path::PathBuf;
use std::process::{Command, Stdio};
use std::time::{Duration, Instant};

use ndn_frame_io::FaceError;
use ndn_radio_hal::{
    Band, Bandwidth, CsiSupport, DbmRange, PhyModeSet, RadioCapability, RadioKind, RadioKnobs,
    RadioProfile, RateCapability,
};

/// How long a `morse_cli` invocation may run before it is killed.
///
/// ☠ The tool has **no timeout of its own** and hangs indefinitely when the chip is wedged. 25 s
/// matches the `timeout 25 sudo morse_cli …` the bench runbook wraps every manual invocation in.
pub const DEFAULT_CLI_TIMEOUT: Duration = Duration::from_secs(25);

/// The debugfs file that carries this radio's absolute dBm TX power.
///
/// ⚠ **Not a vendor knob** — it is our out-of-tree driver patch, which calls
/// `MORSE_CMD_ID_SET_TXPOWER` through the driver's clamping wrapper. See
/// [`MorseKnobs::tx_power_knob`] for why it exists rather than `iw dev … set txpower`.
const TX_POWER_DBM_KNOB: &str = "tx_power_dbm";

/// Bounds the [`TX_POWER_DBM_KNOB`] accepts. MEASURED, not asserted: commanded dB tracked radiated
/// dB at **0.986 dB/dB** with a 0.22 dB rms residual over a 21.5 dB span on an SDR, reversibly, and
/// the firmware clamped a commanded 30 to **27** on an FGH100M-H — which is why
/// [`RadioKnobs::set_tx_power_dbm`] here returns the read-back and never the request.
const TX_POWER_DBM_RANGE: DbmRange = DbmRange { min: 1, max: 30 };

/// `enable_auto_mpsw` module parameter — see [`MorseKnobs::set_auto_mpsw`].
const AUTO_MPSW_PARAM: &str = "/sys/module/morse/parameters/enable_auto_mpsw";

/// A tuned S1G channel, as the chip reports it.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct S1gChannel {
    /// Operating frequency, kHz (the real sub-GHz frequency, not the 5 GHz shadow).
    pub freq_khz: u32,
    /// Operating bandwidth, MHz (1/2/4/8).
    pub op_bw_mhz: u8,
    /// Primary sub-channel bandwidth, MHz (usually 1).
    pub pri_bw_mhz: u8,
    /// Which 1 MHz sub-channel within the operating width is primary. Getting this wrong is silent.
    pub pri_index: u8,
}

/// Minimum Packet Spacing Window — this radio's airtime shaper, as the chip echoes it back.
///
/// Semantics from `morse_cli/mpsw.c` and `struct morse_cmd_mpsw_configuration`: the two airtime
/// bounds select **which packets trigger spacing**, by the packet's own airtime duration; the
/// window length is **how long the TX window is held closed between packets**; `enabled` arms both
/// the bounds check and the enforcement.
///
/// This is the Morse equivalent of the NRC7292's `set tx_time {CS} {Pause}` — a monotone airtime
/// dial that is *not* a contention window, which is exactly why it has no `RadioKnobs` seam.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct MpswConfig {
    /// Bounds checking and spacing enforcement are armed.
    pub enabled: bool,
    /// Minimum packet airtime (µs) that triggers spacing.
    pub airtime_min_us: u32,
    /// Maximum packet airtime (µs) that triggers spacing; `0` = [`AIRTIME_UNLIMITED`].
    pub airtime_max_us: u32,
    /// How long the TX window is held closed between packets, µs.
    pub window_us: u32,
}

/// `airtime_max_us == 0` means "no upper bound", per `mpsw.c`'s `AIRTIME_UNLIMITED`.
pub const AIRTIME_UNLIMITED: u32 = 0;

/// How the firmware spends a duty-cycle budget.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum DutyCycleMode {
    /// Airtime is spread evenly (the CLI default).
    Spread,
    /// Airtime is spent in bursts against a window; only this mode reports remaining airtime.
    Burst,
}

/// The duty-cycle ceiling, as the chip reports it.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct DutyCycleStatus {
    /// Spread or burst.
    pub mode: DutyCycleMode,
    /// Configured ceiling as a **percentage** (the wire unit is percent × 100). `100.00` means
    /// unrestricted — `duty_cycle disable` is implemented as "set 100%".
    pub percent: f32,
    /// Control responses are excluded from the budget (`-o`).
    pub omit_control_responses: bool,
    /// Airtime left in the current burst window, µs. `None` outside burst mode.
    pub airtime_remaining_us: Option<u32>,
    /// Burst window duration, µs. `None` outside burst mode.
    pub burst_window_us: Option<u32>,
}

/// Control for an MM6108: the vendor CLI, plus the driver debugfs/module-parameter surface the CLI
/// cannot reach.
pub struct MorseKnobs {
    iface: String,
    cli: PathBuf,
    timeout: Duration,
    pri_index: u8,
    channels: Vec<u8>,
    duty_cycle_max: f32,
}

impl MorseKnobs {
    /// Bind to `iface` (the Morse netdev — verify with the `morse_spi` driver symlink, since a host
    /// may carry several `wlanN`), using the `morse_cli` binary at `cli`.
    ///
    /// Never touches the filesystem or the radio: everything that probes does so at call time, so a
    /// handle built before the driver is loaded still works afterwards.
    pub fn new(iface: impl Into<String>, cli: impl Into<PathBuf>) -> Self {
        Self {
            iface: iface.into(),
            cli: cli.into(),
            timeout: DEFAULT_CLI_TIMEOUT,
            pri_index: 0,
            channels: us_s1g_channels(),
            duty_cycle_max: 1.0,
        }
    }

    /// Override the per-invocation kill deadline. See [`DEFAULT_CLI_TIMEOUT`].
    pub fn with_timeout(mut self, timeout: Duration) -> Self {
        self.timeout = timeout;
        self
    }

    /// The primary 1 MHz sub-channel index [`RadioKnobs::set_channel`] should use.
    ///
    /// ★ This exists because the trait signature has nowhere to carry it and **the default of 0 is
    /// measured wrong** in the one case that was tested: against an NRC7292 AP at 906 MHz / 4 MHz,
    /// index 1 heard 10 frames per 8 s and indices 0/2/3 heard none. A deployment that knows its
    /// network sets this once; a deployment that does not should be using
    /// [`set_channel_s1g`](Self::set_channel_s1g), where the parameter is explicit.
    ///
    /// Clamped per call against the width the channel number implies (0 for 1 MHz, 0..1 for 2 MHz,
    /// 0..3 for 4 MHz, 0..7 for 8 MHz — the ranges `morse_commands.h` documents for
    /// `pri_1mhz_chan_idx`).
    pub fn with_primary_index(mut self, idx: u8) -> Self {
        self.pri_index = idx;
        self
    }

    /// Restrict the channel list this radio advertises in [`RadioProfile::capability`].
    ///
    /// The default is every US S1G channel [`us_s1g_channel`] knows (1/2/4/8 MHz). A deployment
    /// should usually narrow it to one width class, because cognition picks a channel by *number*
    /// and on this bearer the width rides the number — an unrestricted list lets a picker that
    /// takes the smallest number land on channel 1, which is a legal 1 MHz link and about an
    /// eighth of the rate.
    pub fn with_channels(mut self, channels: Vec<u8>) -> Self {
        self.channels = channels;
        self
    }

    /// Declare the duty-cycle ceiling for [`RadioProfile::capability`].
    ///
    /// Prefer feeding this from [`probe_duty_cycle_max`](Self::probe_duty_cycle_max) — this radio
    /// will *tell* you its ceiling, so asserting one is a choice, not a necessity. The default 1.0
    /// is inherited from the HAL preset and means "not yet asked".
    pub fn with_duty_cycle_max(mut self, frac: f32) -> Self {
        self.duty_cycle_max = frac;
        self
    }

    /// Run one `morse_cli` subcommand, killing it at [`with_timeout`](Self::with_timeout).
    ///
    /// Failure is detected three ways, because one is not enough on this tool: the diagnosis in
    /// [`cli_error`], the **exit status** (meaningful here — `morsectrl.c` remaps a handler's
    /// negative return into 0..254 — unlike the NRC7292's `cli_app`, which always exits 0), and the
    /// timeout. Output is merged stdout+stderr because `mctrl_print` and `mctrl_err` split across
    /// both and a diagnosis frequently spans them.
    ///
    /// ★ **Both pipes are drained by their own threads, started before the wait.** This used to
    /// read them only after the child exited, which is a deadlock: a child that fills a pipe buffer
    /// (~64 KB on Linux) blocks in `write()`, never exits, and the loop below then kills it at the
    /// timeout and reports *"the chip is wedged; power-cycle the radio"* — advice that would send a
    /// human to a lab bench over a full pipe. It never produced a false success, but a misleading
    /// diagnosis is expensive on this rig, and it capped what this function may ever be used for
    /// (`morse_cli stats` alone is 412 lines). Reader threads remove the cap and the trap; they
    /// finish on their own when the child exits or is killed, because either closes the pipe.
    fn run(&self, args: &[&str]) -> Result<String, FaceError> {
        let mut child = Command::new(&self.cli)
            .arg("-i")
            .arg(&self.iface)
            .args(args)
            .stdin(Stdio::null())
            .stdout(Stdio::piped())
            .stderr(Stdio::piped())
            .spawn()
            .map_err(FaceError::Io)?;

        // Take both pipes and drain them concurrently with the wait. `read_to_end` returning an
        // error still leaves what it managed to read in `buf`, which is the half of the diagnosis
        // worth keeping; a read that yields nothing surfaces downstream as a parse failure or a
        // non-zero exit, never as success.
        let drain = |pipe: Option<Box<dyn Read + Send>>| {
            std::thread::spawn(move || {
                let mut buf = Vec::new();
                if let Some(mut p) = pipe {
                    let _ = p.read_to_end(&mut buf);
                }
                buf
            })
        };
        let out_rx = drain(
            child
                .stdout
                .take()
                .map(|p| Box::new(p) as Box<dyn Read + Send>),
        );
        let err_rx = drain(
            child
                .stderr
                .take()
                .map(|p| Box::new(p) as Box<dyn Read + Send>),
        );

        // `None` = we killed it. Deliberately not an `ExitStatus` placeholder: the only value
        // available would be a *successful* one, and a success standing in for a timeout is the
        // shape of bug this module exists to avoid.
        let deadline = Instant::now() + self.timeout;
        let status: Option<std::process::ExitStatus> = loop {
            match child.try_wait().map_err(FaceError::Io)? {
                Some(s) => break Some(s),
                None if Instant::now() >= deadline => {
                    let _ = child.kill();
                    let _ = child.wait();
                    break None;
                }
                None => std::thread::sleep(Duration::from_millis(10)),
            }
        };

        // Joined unconditionally, including on the timeout path: the kill closed the pipes, so both
        // threads are already finishing, and whatever the tool managed to print before it hung is
        // the most useful thing in the error message.
        let mut text = String::new();
        for buf in [out_rx.join(), err_rx.join()].into_iter().flatten() {
            text.push_str(&String::from_utf8_lossy(&buf));
        }

        let Some(status) = status else {
            return Err(self.err(format!(
                "morse_cli {args:?} did not return within {:?} and was killed — the tool has no \
                 timeout of its own and blocks forever when the chip is wedged; power-cycle the \
                 radio rather than retrying. Partial output: {}",
                self.timeout,
                text.trim()
            )));
        };

        if let Some(msg) = cli_error(&text) {
            return Err(self.err(format!("morse_cli {args:?}: {msg}")));
        }
        if !status.success() {
            return Err(self.err(format!(
                "morse_cli {args:?} exited {} — output: {}",
                status.code().unwrap_or(-1),
                text.trim()
            )));
        }
        Ok(text)
    }

    fn err(&self, msg: String) -> FaceError {
        FaceError::Io(std::io::Error::other(format!("{}: {msg}", self.iface)))
    }

    // ── Channel ─────────────────────────────────────────────────────────────────────────────────

    /// Read the current channel.
    pub fn channel(&self) -> Result<S1gChannel, FaceError> {
        let text = self.run(&["channel"])?;
        parse_channel(&text)
            .ok_or_else(|| self.err("could not parse morse_cli channel output".to_string()))
    }

    /// Tune, using the four parameters S1G actually needs, and **verify the read-back**.
    ///
    /// This is the honest API; [`RadioKnobs::set_channel`] cannot express a primary index. The
    /// verification is free: `channel.c` re-reads the chip with `GET_CHANNEL_FULL` after a
    /// successful `SET_CHANNEL` and prints it, so the output of the set is the read-back — and this
    /// radio is MEASURED not to be sticky, so checking it is not ceremony.
    ///
    /// ⚠ A successful retune has two documented side effects: it **wipes the injection EDCA** (the
    /// patched driver re-applies it on every `SET_CHANNEL`; an unpatched one needs `mon0` bounced),
    /// and the driver re-programs MPSW and the duty cycle from the regulatory rule for the new
    /// channel — so any manual [`set_mpsw`](Self::set_mpsw) or
    /// [`set_duty_cycle`](Self::set_duty_cycle) must be re-asserted afterwards, or `enable_auto_mpsw`
    /// turned off first ([`set_auto_mpsw`](Self::set_auto_mpsw)).
    pub fn set_channel_s1g(&self, ch: S1gChannel) -> Result<(), FaceError> {
        let (f, o, p, n) = (
            ch.freq_khz.to_string(),
            ch.op_bw_mhz.to_string(),
            ch.pri_bw_mhz.to_string(),
            ch.pri_index.to_string(),
        );
        let text = self.run(&["channel", "-c", &f, "-o", &o, "-p", &p, "-n", &n])?;
        let got = parse_channel(&text).ok_or_else(|| {
            self.err(format!(
                "channel set to {ch:?} but the read-back could not be parsed — output: {}",
                text.trim()
            ))
        })?;
        if got != ch {
            return Err(self.err(format!(
                "channel set to {ch:?} but the chip reports {got:?} — the request did not take"
            )));
        }
        Ok(())
    }

    // ── TX power ────────────────────────────────────────────────────────────────────────────────

    /// Locate the driver's absolute-dBm debugfs knob, or `None` if the driver is unpatched.
    ///
    /// ★ **Why debugfs and not `iw dev … set txpower`.** The nl80211 path reaches
    /// `morse_mac_ops_config`, whose power block is gated on
    /// `(changed & IEEE80211_CONF_CHANGE_POWER) && !(conf->flags & IEEE80211_CONF_MONITOR)`
    /// (`mac.c:3923`). On a **monitor** configuration — which is exactly the configuration
    /// named-radio injects from — the request is silently dropped. A knob that returns success
    /// while actuating nothing is the defect this crate has already paid for once, so this module
    /// does not use that path at all. The debugfs knob calls `morse_mac_set_txpower` →
    /// `MORSE_CMD_ID_SET_TXPOWER` directly, below the gate.
    ///
    /// Both layouts are checked because drivers place knobs inconsistently; `debug.c:1085` creates
    /// this driver's own subdirectory as `morse` under the wiphy's debugfs dir.
    pub fn tx_power_knob(&self) -> Option<PathBuf> {
        let phy =
            fs::read_to_string(format!("/sys/class/net/{}/phy80211/name", self.iface)).ok()?;
        let root = PathBuf::from(format!("/sys/kernel/debug/ieee80211/{}", phy.trim()));
        [
            root.join("morse").join(TX_POWER_DBM_KNOB),
            root.join(TX_POWER_DBM_KNOB),
        ]
        .into_iter()
        .find(|p| p.is_file())
    }

    /// Read the applied TX power, dBm, from the debugfs knob.
    pub fn tx_power_dbm(&self) -> Result<i8, FaceError> {
        let path = self
            .tx_power_knob()
            .ok_or_else(|| self.err(format!("no {TX_POWER_DBM_KNOB} debugfs knob")))?;
        let text = fs::read_to_string(&path).map_err(FaceError::Io)?;
        parse_leading_i8(&text)
            .ok_or_else(|| self.err(format!("{}: not a dBm value: {text:?}", path.display())))
    }

    // ── MPSW: the airtime shaper ────────────────────────────────────────────────────────────────

    /// Read the current [`MpswConfig`]. A bare `mpsw` sets nothing and prints the chip's response.
    pub fn mpsw(&self) -> Result<MpswConfig, FaceError> {
        let text = self.run(&["mpsw"])?;
        parse_mpsw(&text).ok_or_else(|| self.err("could not parse morse_cli mpsw output".into()))
    }

    /// Program the Minimum Packet Spacing Window, returning **the configuration the chip echoed**.
    ///
    /// Believe the return, not the request: `MORSE_CMD_ID_MPSW_CONFIG` responds with
    /// `struct morse_cmd_resp_mpsw_config` carrying the applied config, which the CLI prints.
    ///
    /// The two validation rules are enforced here rather than left to the CLI, so a caller gets a
    /// real error instead of a subprocess failure: `airtime_min_us` must be **less than**
    /// `airtime_max_us` unless the maximum is [`AIRTIME_UNLIMITED`], and the two must not be equal.
    ///
    /// ⚠ **A manual setting does not survive a retune.** Module parameter `enable_auto_mpsw`
    /// defaults to `true` and `mac.c:3850` re-programs MPSW from the regulatory rule on every
    /// channel change. Either call [`set_auto_mpsw(false)`](Self::set_auto_mpsw) first or re-assert
    /// after every [`set_channel_s1g`](Self::set_channel_s1g).
    pub fn set_mpsw(&self, cfg: MpswConfig) -> Result<MpswConfig, FaceError> {
        if cfg.airtime_min_us == cfg.airtime_max_us
            || (cfg.airtime_max_us != AIRTIME_UNLIMITED && cfg.airtime_min_us > cfg.airtime_max_us)
        {
            return Err(self.err(format!(
                "mpsw airtime bounds {}..{} are rejected by morse_cli: min must be < max, or max \
                 must be {AIRTIME_UNLIMITED} (unlimited)",
                cfg.airtime_min_us, cfg.airtime_max_us
            )));
        }
        let bounds = format!("{},{}", cfg.airtime_min_us, cfg.airtime_max_us);
        let win = cfg.window_us.to_string();
        let en = if cfg.enabled { "1" } else { "0" };
        let text = self.run(&["mpsw", "-b", &bounds, "-w", &win, "-e", en])?;
        parse_mpsw(&text)
            .ok_or_else(|| self.err("could not parse the mpsw response echo".to_string()))
    }

    /// Turn the driver's automatic MPSW re-programming on or off, by writing the module parameter.
    ///
    /// Returns a real error when the parameter is absent (no `morse` module loaded, or a build
    /// without it) rather than reporting a success that shaped nothing.
    ///
    /// ⚠ Module parameters are writable (`0644`) but **not necessarily live** — each one only takes
    /// effect where the driver re-reads it. This one is read inside `morse_mac_change_channel`, so a
    /// post-load write *does* reach it, which is not true of, say, `mcs_mask` (consumed at band
    /// registration) or the `fixed_*` rate parameters (consumed at STA association, so they never
    /// touch injected or broadcast frames, which have no STA).
    pub fn set_auto_mpsw(&self, on: bool) -> Result<(), FaceError> {
        fs::write(AUTO_MPSW_PARAM, if on { "1\n" } else { "0\n" })
            .map_err(|e| self.err(format!("writing {AUTO_MPSW_PARAM}: {e}")))
    }

    // ── Duty cycle ──────────────────────────────────────────────────────────────────────────────

    /// Read the duty-cycle configuration the chip is enforcing.
    pub fn duty_cycle(&self) -> Result<DutyCycleStatus, FaceError> {
        let text = self.run(&["duty_cycle"])?;
        parse_duty_cycle(&text)
            .ok_or_else(|| self.err("could not parse morse_cli duty_cycle output".into()))
    }

    /// The duty-cycle ceiling as a fraction in `[0, 1]`, ready for
    /// [`RadioCapability::duty_cycle_max`] — read from the radio instead of asserted.
    pub fn probe_duty_cycle_max(&self) -> Result<f32, FaceError> {
        Ok((self.duty_cycle()?.percent / 100.0).clamp(0.0, 1.0))
    }

    /// Set the duty-cycle ceiling, in percent (`0.01..=100.0`), and how it is spent.
    ///
    /// `omit_control_responses` maps to `-o` and excludes control responses from the budget.
    /// `100.0` is how the CLI itself expresses "disabled", so it is the way to lift the ceiling.
    ///
    /// ⚠ Same override trap as MPSW: `enable_auto_duty_cycle` defaults to `true` and the driver
    /// programs duty from the regulatory rule on every channel change. This is also, most likely,
    /// the mechanism behind the NRC7292's `set duty` being MEASURED dead — same regulatory gating,
    /// different vendor — so do not assume a value that was accepted here is being enforced there.
    pub fn set_duty_cycle(
        &self,
        percent: f32,
        mode: DutyCycleMode,
        omit_control_responses: bool,
    ) -> Result<DutyCycleStatus, FaceError> {
        if !(0.01..=100.0).contains(&percent) {
            return Err(self.err(format!(
                "duty cycle {percent}% is outside the 0.01-100.00% morse_cli accepts"
            )));
        }
        let pct = format!("{percent:.2}");
        let m = match mode {
            DutyCycleMode::Spread => "0",
            DutyCycleMode::Burst => "1",
        };
        let mut args = vec!["duty_cycle", "enable", pct.as_str(), "-m", m];
        if omit_control_responses {
            args.push("-o");
        }
        self.run(&args)?;
        // `set` returns no payload, so the only honest confirmation is a fresh read.
        self.duty_cycle()
    }

    /// Airtime remaining in the current burst window, µs.
    ///
    /// Burst mode only — in spread mode the CLI answers `Command not supported when in spread mode`
    /// and this returns an error. Not an actuator, but a genuine readout of how much TX budget is
    /// left, which is the closest this radio comes to telling a scheduler what it can still spend.
    pub fn duty_cycle_airtime_us(&self) -> Result<u32, FaceError> {
        let text = self.run(&["duty_cycle", "airtime"])?;
        parse_duty_cycle_airtime(&text)
            .ok_or_else(|| self.err(format!("no airtime value in: {}", text.trim())))
    }

    // ── Aggregation, lifetime, colour ───────────────────────────────────────────────────────────

    /// Enable or disable A-MPDU sessions (`MORSE_CMD_ID_SET_AMPDU`).
    ///
    /// ⚠ The CLI's own remark is "Must be run before association", and it is
    /// `MM_DIRECT_CHIP_NOT_SUPPORTED` — i.e. it needs the driver transport, not a bare chip.
    pub fn set_ampdu(&self, on: bool) -> Result<(), FaceError> {
        self.run(&["ampdu", if on { "enable" } else { "disable" }])?;
        Ok(())
    }

    /// Cap the maximum A-MPDU length in bytes, or `None` to reset to the chip default.
    ///
    /// The direct actuator for "make my burst fit the slot I own": an aggregate that overruns a slot
    /// boundary is airtime bleed in a different costume. Chip-direct and settable at any time
    /// (unlike [`set_ampdu`](Self::set_ampdu)); the CLI sends `-1` for the reset.
    pub fn set_max_ampdu_len(&self, bytes: Option<u32>) -> Result<(), FaceError> {
        match bytes {
            Some(n) => self.run(&["maxampdulen", &n.to_string()])?,
            None => self.run(&["maxampdulen", "-r"])?,
        };
        Ok(())
    }

    /// Set the firmware's TX packet lifetime, µs — frames it could not send within the lifetime are
    /// dropped rather than queued indefinitely.
    ///
    /// The range is the CLI's own (`50000..=500000`), enforced here so an out-of-range value is a
    /// typed error rather than a subprocess failure. ⚠ The 50 ms floor is coarse against a 20 ms
    /// slot, so this disciplines **staleness**, not slots. It is the *complement* of
    /// [`set_tx_hold`](RadioKnobs::set_tx_hold), not a substitute for it: this one **drops** the
    /// frames it catches, `set_tx_hold` **holds** them.
    pub fn set_tx_packet_lifetime_us(&self, us: u32) -> Result<(), FaceError> {
        if !(50_000..=500_000).contains(&us) {
            return Err(self.err(format!(
                "tx packet lifetime {us} µs is outside the 50000-500000 morse_cli accepts"
            )));
        }
        self.run(&["tx_pkt_lifetime_us", &us.to_string()])?;
        Ok(())
    }

    // ── Transmit gate ───────────────────────────────────────────────────────────────────

    /// Read back the chip transmit gate that [`RadioKnobs::set_tx_hold`] writes
    /// (`MORSE_CMD_PARAM_ID_TX_BLOCK`, id 6).
    ///
    /// Separate from the setter on purpose. `set_tx_hold` does **not** verify, because the
    /// verification is not free here — a `get` costs another full `morse_cli` invocation, MEASURED
    /// **10.61 ms median** (10.55–10.83, n = 12, `examples/morse_txhold.rs` on an MM6108), which is
    /// the same price as the set — and the caller of a slot gate is on a slot boundary. Callers
    /// that want the confirmation take it explicitly, off the critical path, with this. It agreed
    /// with the commanded state 12 times out of 12 in that run.
    ///
    /// The chip stores a byte and the firmware tests it with `snez` (any non-zero blocks), so the
    /// wire value is not a bool; the CLI entry constrains it to 0/1 and this maps anything non-zero
    /// to `true` to match the firmware rather than the CLI.
    pub fn tx_block(&self) -> Result<bool, FaceError> {
        let text = self.run(&["get", "tx_block"])?;
        parse_tx_block(&text).ok_or_else(|| {
            self.err(format!(
                "morse_cli get tx_block printed {:?}, which is not a number — on a stock                  morse_cli this parameter does not exist (its params.c table has no id-6 entry);                  add it, or treat the gate as absent",
                text.trim()
            ))
        })
    }

    /// Set the 3-bit S1G BSS colour (0–7), the spatial-reuse identifier.
    ///
    /// ⚠ Under the named-radio no-host-identity doctrine a persistent 3-bit network tag is exactly
    /// the kind of thing that quietly becomes an address. It is exposed because it is a real
    /// reuse lever this part has and `set_edcca_threshold_dbm` does not; apply the soft-state test
    /// before writing it.
    pub fn set_bss_color(&self, color: u8) -> Result<(), FaceError> {
        if color > 7 {
            return Err(self.err(format!("BSS colour {color} is outside 0-7")));
        }
        self.run(&["bsscolor", &color.to_string()])?;
        Ok(())
    }
}

/// Detect a failed `morse_cli` invocation from its output.
///
/// Exit status is checked separately by [`MorseKnobs::run`]; this exists to turn the three failures
/// that have actually been hit into a sentence a reader can act on, and to catch the invalid-channel
/// case, which is neither a `Failed to …` line nor obviously an error at a glance.
fn cli_error(text: &str) -> Option<String> {
    for line in text.lines() {
        let l = line.trim();
        // ENETDOWN: the interface is down. The most misleading failure — nothing else looks wrong.
        if l.contains("Failed to rcvmsgs") || l.contains("code -100") {
            return Some("interface is DOWN (ENETDOWN) — bring it up before tuning".into());
        }
        // channel.c's invalid-combination path: it prints this and returns
        // MORSE_RET_SET_INVALID_CHAN_CONFIG, never a "Failed to" line. Missing it made a refused
        // channel read as success.
        if l.starts_with("Invalid combination of parameters") {
            return Some(l.to_string());
        }
        // params.c's `match_str_to_param` miss. It exits non-zero, so `run` would have failed
        // anyway, but the message matters: this is what a *stock* morse_cli says about a parameter
        // its `params[]` table does not carry, and the fix is a CLI patch, not a radio.
        if l.starts_with("Invalid parameter:") {
            return Some(format!(
                "{l} — this morse_cli's params.c table has no entry for it"
            ));
        }
        if l.starts_with("Failed to") || l.contains("No transports supported") {
            return Some(l.to_string());
        }
    }
    None
}

/// Parse `morse_cli … channel` output.
///
/// ⚠ Reads the **last** block it sees. With `-a` the CLI prints three (`Full`, `DTIM`, `Current`);
/// nothing here passes `-a`, and "current" is the right answer if anything ever does.
fn parse_channel(text: &str) -> Option<S1gChannel> {
    let (mut freq, mut op, mut pri, mut idx) = (None, None, None, None);
    for line in text.lines() {
        let l = line.trim();
        let val = |s: &str| -> Option<u32> {
            l.split_once(s)
                .and_then(|(_, v)| v.split_whitespace().next()?.parse().ok())
        };
        if l.starts_with("Operating Frequency") {
            freq = val(":");
        } else if l.starts_with("Operating BW") {
            op = val(":");
        } else if l.starts_with("Primary BW") {
            pri = val(":");
        } else if l.starts_with("Primary Channel Index") {
            idx = val(":");
        }
    }
    Some(S1gChannel {
        freq_khz: freq?,
        op_bw_mhz: op? as u8,
        pri_bw_mhz: pri? as u8,
        pri_index: idx? as u8,
    })
}

/// Value after the first `:` on a line whose trimmed key equals `key`.
fn labelled<'a>(text: &'a str, key: &str) -> Option<&'a str> {
    text.lines().find_map(|l| {
        let (k, v) = l.split_once(':')?;
        (k.trim() == key).then(|| v.trim())
    })
}

/// Parse the four lines `mpsw.c`'s `print_mpsw_cfg` emits.
fn parse_mpsw(text: &str) -> Option<MpswConfig> {
    Some(MpswConfig {
        enabled: labelled(text, "MPSW Active")?.parse::<u32>().ok()? != 0,
        airtime_min_us: labelled(text, "Airtime Minimum Bound")?.parse().ok()?,
        airtime_max_us: labelled(text, "Airtime Maximum Bound")?.parse().ok()?,
        window_us: labelled(text, "Packet Spacing Window Length")?
            .parse()
            .ok()?,
    })
}

/// Parse `duty_cycle.c`'s `get_duty_cycle` output.
fn parse_duty_cycle(text: &str) -> Option<DutyCycleStatus> {
    let mode = match labelled(text, "Mode")? {
        "burst" => DutyCycleMode::Burst,
        "spread" => DutyCycleMode::Spread,
        _ => return None,
    };
    let percent = labelled(text, "Configured duty cycle")?
        .trim_end_matches('%')
        .parse()
        .ok()?;
    let omit = labelled(
        text,
        "Control responses omitted from duty cycle calculation",
    )?
    .parse::<u32>()
    .ok()?
        != 0;
    Some(DutyCycleStatus {
        mode,
        percent,
        omit_control_responses: omit,
        airtime_remaining_us: labelled(text, "Airtime remaining (us)").and_then(|v| v.parse().ok()),
        burst_window_us: labelled(text, "Burst window duration (us)").and_then(|v| v.parse().ok()),
    })
}

/// Parse `duty_cycle airtime`, which prints a bare `%u`.
///
/// In spread mode the CLI prints an error to stderr instead and returns non-zero; because
/// [`MorseKnobs::run`] merges both streams, this must not be fooled by that text.
fn parse_duty_cycle_airtime(text: &str) -> Option<u32> {
    text.lines()
        .map(str::trim)
        .find(|l| !l.is_empty() && l.chars().all(|c| c.is_ascii_digit()))
        .and_then(|l| l.parse().ok())
}

/// Leading signed integer of a string (`"27\n"` → `27`).
/// Parse `morse_cli get tx_block` output, which is `param_get_uint32`'s bare `"%u\n"`.
///
/// Scans for the first line that is entirely a number rather than taking `text` whole, because
/// [`MorseKnobs::run`] concatenates stdout and stderr and the tool prints unrelated chatter (e.g.
/// transport notes) on the latter. Any non-zero means blocked — the firmware's own test is `snez`.
fn parse_tx_block(text: &str) -> Option<bool> {
    text.lines()
        .filter_map(|l| l.trim().parse::<u32>().ok())
        .map(|v| v != 0)
        .next()
}

fn parse_leading_i8(s: &str) -> Option<i8> {
    let t = s.trim();
    let end = t
        .char_indices()
        .position(|(i, c)| !(c.is_ascii_digit() || (i == 0 && (c == '-' || c == '+'))))
        .unwrap_or(t.len());
    t[..end].parse().ok()
}

/// **The US S1G channel table, as a formula**: `(centre frequency kHz, operating width MHz)` for a
/// channel number, or `None` if that number is not a US S1G channel.
///
/// ★ **The width is a property of the channel number.** That is the fact that dissolves most of the
/// [`Bandwidth`] problem on this bearer: the HAL enum cannot say 1/2/4/8 MHz, but it does not have
/// to, because naming the channel already says it.
///
/// Every centre frequency is `902_000 + channel × 500` kHz, and the width follows the number's
/// residue: odd = 1 MHz, `≡2 (mod 4)` = 2 MHz, `≡0 (mod 8)` = 4 MHz, `≡12 (mod 16)` = 8 MHz.
///
/// Derived from the vendor table `s1g_ch_table_us[]` in the NRC7292 driver's `nrc-s1g.c`, which
/// lists 1 MHz channels 1–51 odd, 2 MHz 2–50 step 4, and 4 MHz 8–48 step 8 with their exact
/// frequencies; the formula reproduces every row. The 8 MHz row set is not in that table (the
/// NRC7292 has no 8 MHz channels in the US at all) and comes from the independently MEASURED
/// `morse_cli`/`hostapd_s1g` result that US 8 MHz is channels **12 / 28 / 44** — 908.0, 916.0 and
/// 924.0 MHz, which the same formula produces. Two independent sources, one arithmetic.
///
/// Channel numbers `≡4 (mod 16)` (4, 20, 36) are the US **16 MHz** rows and are refused: the
/// MM6108 does not do 16 MHz, and `morse_commands.h` documents primary indices 0–15 for a width
/// this part cannot reach.
///
/// ⚠ **US only.** The channel set, the widths and the power ceiling are all regdomain-derived, and
/// `RadioCapability` has no field to name a regdomain. A non-US deployment must supply its own list
/// via [`MorseKnobs::with_channels`] and must not trust this function.
pub fn us_s1g_channel(channel: u8) -> Option<(u32, u8)> {
    let width = match channel {
        c if c % 2 == 1 && (1..=51).contains(&c) => 1,
        c if c % 4 == 2 && (2..=50).contains(&c) => 2,
        c if c % 8 == 0 && (8..=48).contains(&c) => 4,
        c if c % 16 == 12 && (12..=44).contains(&c) => 8,
        _ => return None,
    };
    Some((902_000 + u32::from(channel) * 500, width))
}

/// Every US S1G channel [`us_s1g_channel`] recognises, ascending — the default channel list.
pub fn us_s1g_channels() -> Vec<u8> {
    (1u8..=51)
        .filter(|c| us_s1g_channel(*c).is_some())
        .collect()
}

/// The largest primary 1 MHz index a given operating width admits, per the
/// `pri_1mhz_chan_idx` ranges documented on `struct morse_cmd_req_set_channel`
/// (0 for 1 MHz, 0–1 for 2, 0–3 for 4, 0–7 for 8, 0–15 for 16).
fn max_primary_index(op_bw_mhz: u8) -> u8 {
    op_bw_mhz.saturating_sub(1)
}

impl RadioKnobs for MorseKnobs {
    /// **The one power knob, routed to the absolute axis this part actually has.**
    ///
    /// ★ Wired so that `PowerRequest` means something on a HaLow radio too, rather than falling
    /// through to the trait's `Unsupported` default: `Dbm` goes straight to
    /// [`set_tx_power_dbm`](RadioKnobs::set_tx_power_dbm), and `Ceiling` means the top of the
    /// declared [`DbmRange`] — which on a part with a real dBm axis is the
    /// honest reading of "as loud as this part will legally go". An index request is refused by
    /// name: this radio has no index scale, and inventing a mapping onto its dBm axis would be
    /// exactly the invented number the contract forbids.
    fn set_tx_power(
        &self,
        req: ndn_radio_hal::PowerRequest,
    ) -> Result<ndn_radio_hal::AppliedPower, FaceError> {
        use ndn_radio_hal::PowerRequest as P;
        let want = match &req {
            P::Dbm(d) => *d,
            P::Ceiling(_) => match ndn_radio_hal::RadioProfile::capability(self).tx_power_dbm {
                Some(r) => r.max,
                None => {
                    return Err(ndn_radio_hal::power_unsupported(
                        "morse: Ceiling requested but this radio declares no dBm range",
                    ));
                }
            },
            P::Index(i, _) => {
                return Err(ndn_radio_hal::power_unsupported(format!(
                    "morse: PowerRequest::Index({i}) — this radio has an ABSOLUTE dBm axis and no \
                     index scale. Mapping an opaque index onto dBm would invent a number a link \
                     budget would believe. Use PowerRequest::Dbm.",
                )));
            }
            P::Raw { .. } => {
                return Err(ndn_radio_hal::power_unsupported(
                    "morse: there is no raw chip axis behind this knob — the vendor path is the \
                     only one, and it is already dBm-denominated and regulatory-clamped.",
                ));
            }
            P::NoActuator => {
                return Err(ndn_radio_hal::power_unsupported(
                    "morse: PowerRequest::NoActuator, but this radio DOES actuate power in dBm.",
                ));
            }
        };
        let applied = RadioKnobs::set_tx_power_dbm(self, want)?;
        Ok(ndn_radio_hal::AppliedPower::absolute_dbm(
            req.clone(),
            applied,
            applied != want,
        ))
    }

    /// Tune by **S1G channel number**, from which the frequency and the operating width both
    /// follow ([`us_s1g_channel`]).
    ///
    /// ★ This used to compute `902_500 + channel × 1000` kHz — a 1 MHz raster with the channel as an
    /// index — which is not the S1G channel numbering and put "channel 8" at 910.5 MHz where the
    /// standard, both vendors' tables and the bench all put it at **906.0 MHz**. Two radios told to
    /// use the same channel number would have tuned to different air. It now uses the real table.
    ///
    /// ★ **`bw` is read through [`crate::halow::s1g_width_request`], the same reading the NRC7292
    /// uses.** This used to be Morse-specific — every `Bandwidth` was ignored and a mismatch merely
    /// `tracing::debug!`-logged — which meant the two radios of one bearer answered an
    /// inexpressible width request two different ways: 40 MHz was a hard refusal on the NRC7292 and
    /// a shrug here, on a face whose entire premise is that it covers both. Now:
    /// `Bw20` (the `Default`, i.e. what `Bandwidth::from_code(0)` and therefore every Wi-Fi-shaped
    /// plan passes) means "no preference" and is accepted; `Nb5`/`Nb10` are honoured as 1/2 MHz only
    /// if the channel already has that width; `Bw40`/`Bw80` are refused, because no S1G channel is
    /// that wide and tuning at 1 MHz while the caller asked for 40 would be a silent lie about the
    /// width transmitted. See that function for the full reading, including the part of it that is
    /// this crate's convention rather than a HAL statement.
    ///
    /// ⚠ **The primary index still cannot be expressed** — see
    /// [`with_primary_index`](MorseKnobs::with_primary_index), and prefer
    /// [`set_channel_s1g`](MorseKnobs::set_channel_s1g) when it matters, which on this bearer is
    /// most of the time.
    fn set_channel(&self, channel: u8, bw: Bandwidth) -> Result<(), FaceError> {
        let (freq_khz, op_bw_mhz) = us_s1g_channel(channel).ok_or_else(|| {
            self.err(format!(
                "S1G channel {channel} is not a US channel — the US set is {:?}",
                us_s1g_channels()
            ))
        })?;
        let wanted_mhz = crate::halow::s1g_width_request(bw)
            .map_err(|e| self.err(format!("S1G channel {channel} ({op_bw_mhz} MHz): {e}")))?;
        if let Some(w) = wanted_mhz
            && w != op_bw_mhz
        {
            return Err(self.err(format!(
                "S1G channel {channel} is {op_bw_mhz} MHz wide, not {w} MHz — on S1G the width is a \
                 property of the channel, so pick the channel that has the width"
            )));
        }
        self.set_channel_s1g(S1gChannel {
            freq_khz,
            op_bw_mhz,
            pri_bw_mhz: 1,
            pri_index: self.pri_index.min(max_primary_index(op_bw_mhz)),
        })
    }

    /// Absolute dBm TX power through the driver's debugfs knob, returning **the value read back**.
    ///
    /// A genuine calibrated axis — MEASURED 0.986 dB/dB against an SDR over a 21.5 dB span, rms
    /// residual 0.22 dB, reversible — and one of only two in this fleet.
    ///
    /// ★ **The read-back is mandatory, and this is stricter than the generic mac80211 adapter on
    /// purpose.** The firmware clamps (a commanded 30 came back 27 on an FGH100M-H), and the whole
    /// reason this module avoids `iw` is that the nl80211 path is silently dropped in monitor mode.
    /// A write we cannot confirm is therefore indistinguishable from the failure mode this crate
    /// exists to avoid, so an unreadable knob is an error — even though the write was issued —
    /// rather than an assumed success.
    ///
    /// ⚠ **A retune wipes it.** The driver asserts the regulatory maximum on every channel change
    /// (`mac.c:2816`, `mac.c:3841`), so any back-off must be re-applied after
    /// [`set_channel`](Self::set_channel).
    fn set_tx_power_dbm(&self, dbm: i8) -> Result<i8, FaceError> {
        let path = self.tx_power_knob().ok_or_else(|| {
            self.err(format!(
                "no {TX_POWER_DBM_KNOB} debugfs knob under this phy — the driver is unpatched, and \
                 the nl80211 path is silently dropped on a monitor vif (mac.c:3923), so there is no \
                 honest dBm actuator here"
            ))
        })?;
        let want = TX_POWER_DBM_RANGE.clamp(dbm);
        fs::write(&path, format!("{want}\n"))
            .map_err(|e| self.err(format!("writing {}: {e}", path.display())))?;
        let back = fs::read_to_string(&path).map_err(|e| {
            self.err(format!(
                "wrote {want} dBm to {} but could not read it back ({e}) — the applied power is \
                 unknown, which is not the same as applied",
                path.display()
            ))
        })?;
        parse_leading_i8(&back).ok_or_else(|| {
            self.err(format!(
                "wrote {want} dBm to {} but the read-back {back:?} is not a dBm value",
                path.display()
            ))
        })
    }

    /// **Hold or release transmit at the chip MAC** — `MORSE_CMD_PARAM_ID_TX_BLOCK` (id 6), sent
    /// through `MORSE_CMD_ID_GET_SET_GENERIC_PARAM` (0x003E). ★ A real, **association-free** gate.
    ///
    /// ## Why this exists at all
    ///
    /// It is the piece the named airtime lease needs and the one this file used to say did not
    /// exist. Under the no-association doctrine the usual S1G answers are both unavailable — RAW
    /// keys on an AID and AID 0 means "RAW does not apply, allow", TWT has an association
    /// precondition — so a gate that reaches the chip with no vif, no AID and no AP is the whole
    /// point. This one does: the driver's `command.c:1671` switch names only AP_POWER_SAVE /
    /// NON_TIM_MODE / HOME_CHANNEL_DWELL / ACTIVE_SCAN_DWELL / CHANNELIZATION, so id 6 falls to
    /// `default:` → `morse_cmd_tx()` and goes straight to the chip. Firmware handler `0x00132eb4`
    /// is `snez` then `sb a5, 0x802006FE` — **any non-zero blocks**, no range check, always
    /// returns 0 — and the single enforcement point is `0x0012579e lbu a5,0x4e(s1)`.
    ///
    /// ## MEASURED (MM6108 → MM6108, 904.5 MHz / 1 MHz, monitor injection, no association)
    ///
    /// **Instrument** — the chip's own MAC-core `TX Total` (`morse_cli stats -m -s <fw>`), which
    /// idled at **+0 over 6 s** with nothing transmitting. 200 frames injected at 100 f/s per arm:
    ///
    /// | arm | `TX Total` Δ | `TX requests` Δ |
    /// |---|---|---|
    /// | `hold = false`, inject 200 | **+200** (exactly N) | +228 |
    /// | `hold = true`, inject 200 | **+0** (`DCF granted` +0 too) | **+0** |
    /// | `hold = false`, **inject nothing** | **+200** — the held frames | +229 |
    /// | `hold = false`, inject 200 again | +200 | +232 |
    ///
    /// **On air** — the same three arms repeated with a second MM6108 as the receiver, 300 frames
    /// at 100 f/s, negative control (receiver up, nobody transmitting) = 0 frames:
    ///
    /// | arm | received at the peer |
    /// |---|---|
    /// | `hold = false` | 293 / 300, mean RSSI −49.3 dBm |
    /// | `hold = true` | **0 frames in a 15 s window** |
    /// | `hold = false`, injecting nothing | 244 frames in ONE 1-second bucket (`TX Total` +250) |
    ///
    /// So it gates, it is reversible, and the last row is the part a caller must design around:
    ///
    /// * ⚠ **It HOLDS, it does not drop.** The held frames drained the instant the gate opened and
    ///   arrived at the peer as a single burst — 244 frames inside one second against a 100 f/s
    ///   offered rate. This is the same "quiet repaid with interest" shape as `REG_TXPAUSE` on the
    ///   RTL8733BU, and it is only safe where the release lands in a window you own.
    /// * ⚠ **The queue is finite and back-pressures the host.** Offering 300 frames while held made
    ///   `sendto()` on the monitor vif stall and error part-way; 250 of them survived to drain. A
    ///   caller must not assume "held" means "buffered without limit".
    /// * ⚠ **COARSE — it is a `set_tx_hold`, not a slot actuator.** MEASURED round trip through
    ///   *this method*: **10.65 ms median** (10.56–10.84, n = 12; `examples/morse_txhold.rs`), and
    ///   the spread is only 0.3 ms, so it is a predictable cost rather than a risky one. Nearly all
    ///   of it is `fork`/`exec` of `morse_cli`, not the radio: run against an identical shell
    ///   harness, `set tx_block` cost 24.2 ms median and `morse_cli -v` — which touches no chip —
    ///   cost 21.9 ms, so the netlink-plus-chip part is only **≈2.3 ms**. A caller that needs the
    ///   2 ms would have to speak the vendor netlink command directly instead of spawning the CLI.
    ///   As it stands 10.65 ms is **5× the 2 ms minimum useful slot** measured for host-scheduled
    ///   injection on this part, so this gate bounds a *window* — a lease of tens of ms, a "be
    ///   quiet until further notice" — and cannot place a frame. Placement stays with host
    ///   scheduling, which reaches p50 88 µs / p99 453 µs.
    ///
    /// ## Prerequisite
    ///
    /// `morse_cli`'s `params[]` table must carry a `tx_block` entry (id 6); the vendor build has
    /// none. The diff and the build line for it are in
    /// `patches/morse_cli-tx_block-param.patch`. Without it the tool prints
    /// `Invalid parameter: 'tx_block'` and exits non-zero, and this method returns that as an
    /// **error** — never a silent success, which is what an earlier override of this method (whose
    /// whole body was `Ok(())`) did, and why it was reverted.
    ///
    /// Deliberately not verified with a read-back: that would double the 10.65 ms on a path a
    /// scheduler calls at a boundary. Use [`MorseKnobs::tx_block`] to confirm, off the hot path —
    /// it agreed with the commanded state 12 / 12 when asked.
    fn set_tx_hold(&self, hold: bool) -> Result<(), FaceError> {
        let v = if hold { "1" } else { "0" };
        self.run(&["set", "tx_block", v]).map_err(|e| {
            self.err(format!(
                "could not {} transmit via the tx_block generic parameter: {e} — a stock morse_cli \
                 has no id-6 entry in its params.c table, so the gate is unreachable from this \
                 binary even though the chip and driver both accept it",
                if hold { "hold" } else { "release" }
            ))
        })?;
        Ok(())
    }

    /// **Refused.** There is no CCA/LBT bypass on this part.
    ///
    /// The trait default is a silent `Ok(())`, which would tell a caller that CSMA had been
    /// suppressed when nothing was done. Nothing in `morse_cli`, `morse_commands.h` or the driver
    /// suppresses listen-before-talk; the only candidate, `medium_eval`, is an undocumented enable
    /// byte in the header's test-command block and is deliberately not wired (see the module docs).
    /// `on == false` is accepted because there is nothing to undo.
    fn set_edcca_ignore(&self, on: bool) -> Result<(), FaceError> {
        if !on {
            return Ok(());
        }
        Err(FaceError::Io(std::io::Error::new(
            std::io::ErrorKind::Unsupported,
            "MM6108 exposes no EDCCA/LBT bypass; `medium_eval` is an undocumented test command and \
             is not it",
        )))
    }

    /// **Refused.** On this radio the channel width is not an independent dial.
    ///
    /// The trait frames this as the LoRa 125/250/500 kHz knob, and the tempting stopgap is to route
    /// S1G's 1/2/4/8 MHz through it as 1000/2000/4000/8000 kHz — the mechanism is real, it is the
    /// `-o` flag [`set_channel_s1g`](MorseKnobs::set_channel_s1g) already uses. It is refused
    /// because re-asserting a different `-o` at the current centre frequency generally lands the
    /// radio on a *non-standard* centre/width pair, and because the chip is MEASURED to accept
    /// widths it will not transmit at (8 MHz read back correctly while injection went out on the
    /// primary 1 MHz), so the read-back cannot even confirm it. Choose the width by choosing the
    /// channel number, or drive all four parameters explicitly.
    fn set_bandwidth_khz(&self, khz: u32) -> Result<(), FaceError> {
        Err(FaceError::Io(std::io::Error::new(
            std::io::ErrorKind::Unsupported,
            format!(
                "MM6108 is an S1G radio: width is carried by the channel number, not a kHz dial \
                 (asked for {khz} kHz) — use set_channel or MorseKnobs::set_channel_s1g"
            ),
        )))
    }

    // ---- Left at the trait default. See the module docs for the mechanism looked for in each. ----
    //
    // `set_tx_hold` used to be listed here. It is now implemented above against a real, measured
    // actuator; the rule that put it here still stands for the rest — an override whose whole body
    // is `Ok(())` is a method that reports success without actuating, and the explanation for a
    // missing knob belongs in prose, not in a function that returns success.
}

impl RadioProfile for MorseKnobs {
    /// The MM6108's capability, built from [`RadioCapability::wifi_halow_s1g`] with four
    /// corrections, each of which the preset gets wrong for this part.
    ///
    /// * **`kind: WifiHaLow`**, not the preset's `WifiMonitor` — which contradicts the preset's own
    ///   name and hides a HaLow radio from the only consumer that keys on the kind
    ///   (`RadioPolicy::data_plane`, which turns on dedup and CS-serve for duty-limited broadcast
    ///   bearers). This is a sub-GHz bearer where airtime is the scarce resource; it should be
    ///   treated as one.
    /// * ★ **`max_mcs: 7`**, not 10. S1G MCS10 is a 1 MHz-only repetition-coded BPSK mode: it is
    ///   *more* robust and *slower* than MCS0, i.e. it sits **below** the ladder, not above it, so
    ///   `max_mcs: 10` would not name a maximum rate. See [`crate::halow`]'s `halow_base` for the
    ///   corrected account of *which* consumers read this field unclamped (the contextual bandit's
    ///   `clamp(0, max_mcs)` and `rate_rank` — **not** `mcs_for_rssi`, which caps at
    ///   `MAX_RELIABLE_MCS = 7` on its own). It is additionally gated on module parameter
    ///   `mcs10_mode`, which **defaults to disabled**, so on a stock load the rate does not exist
    ///   at all. Reach it deliberately, never through the ladder.
    /// * **`max_payload: 1546`**, not 1500 — MEASURED byte-exact on air (1546 delivered, 1547 dead;
    ///   1584 B MPDU cutoff). The generic monitor constants overshoot into the silent-drop regime.
    /// * **`tx_power_dbm` and `power_actuated` are probed, not asserted.** Both are `Some(1..30)` /
    ///   `true` only when the debugfs knob is actually present; on an unpatched driver the radio
    ///   reports no dBm axis and no actuated power, which is the truth. The preset hardcodes
    ///   `power_actuated: true`, and on a monitor vif with no knob that is decorative — precisely
    ///   what the field was added to prevent.
    ///
    /// Left as the preset has them, with the reason: `max_bw: 0` is meaningless for S1G (the code
    /// space runs 0=20 MHz … 4=5 MHz and has no S1G value) and is the honest placeholder; `max_nss:
    /// 1` is correct, the part is single-chain; `max_tx_power: 63` is an inherited **index-scale
    /// fiction** on a part that has no TXAGC index at all — `tx_power_dbm: Some(..)` is already the
    /// HAL's declared signal that the index path is not the one to use, and `min_tx_power` /
    /// `db_per_power_idx` stay `None` because that scale has nothing to measure; `retune_us: None` because nobody has timed a
    /// `channel -c` here (it is cheap to measure and worth doing); `csi: None`; `phy_modes` empty
    /// because `PhyMode` is a sub-GHz-modem enum with no S1G member, so "I cannot say" is the only
    /// true answer; `hop: None`.
    fn capability(&self) -> RadioCapability {
        let power_knob = self.tx_power_knob();
        let mut cap = RadioCapability::wifi_halow_s1g(self.channels.clone());
        cap.kind = RadioKind::WifiHaLow;
        cap.rate = RateCapability::Wifi {
            max_mcs: 7,
            max_nss: 1,
            max_bw: 0,
        };
        cap.max_payload = 1546;
        cap.duty_cycle_max = self.duty_cycle_max;
        cap.power_actuated = power_knob.is_some();
        cap.tx_power_dbm = power_knob.map(|_| TX_POWER_DBM_RANGE);
        cap.csi = CsiSupport::None;
        cap.phy_modes = PhyModeSet::empty();
        cap.bands = vec![Band::Sub1GHz];
        cap
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Verbatim `morse_cli -i wlan0 channel` output from an MM6108.
    const OK: &str = "\
Full Channel Information
\tOperating Frequency: 906000 kHz
\tOperating BW: 4 MHz
\tPrimary BW: 1 MHz
\tPrimary Channel Index: 1
";

    #[test]
    fn parses_a_real_channel_readout() {
        assert_eq!(
            parse_channel(OK),
            Some(S1gChannel {
                freq_khz: 906_000,
                op_bw_mhz: 4,
                pri_bw_mhz: 1,
                pri_index: 1,
            })
        );
    }

    /// The ENETDOWN failure is the one that must never be mistaken for success: the tool prints it
    /// and the radio silently stays on its old channel.
    #[test]
    fn detects_the_interface_down_failure() {
        let down = "NL80211, code -100: Error callback called\n\
                    NL80211, code -1: Failed to rcvmsgs\n\
                    Failed to get channel frequency\n";
        let msg = cli_error(down).expect("must be reported as an error");
        assert!(msg.contains("ENETDOWN"), "got {msg:?}");
        assert!(parse_channel(down).is_none(), "must not parse as a channel");
    }

    /// A morse_cli built without CONFIG_MORSE_TRANS_NL80211=1 links fine and fails only at runtime.
    #[test]
    fn detects_the_missing_transport_build() {
        assert!(cli_error("No transports supported\n").is_some());
    }

    /// Success output must not be flagged as an error.
    #[test]
    fn success_is_not_an_error() {
        assert!(cli_error(OK).is_none());
    }

    /// ★ The failure the old matcher missed. `channel.c`'s invalid-combination path prints exactly
    /// this — no `Failed to` line anywhere — and returns MORSE_RET_SET_INVALID_CHAN_CONFIG, so a
    /// refused channel used to read as a successful tune.
    #[test]
    fn detects_the_invalid_channel_combination() {
        let bad = "Invalid combination of parameters - freq=906000, bw=8, \
                   primary bw=1, primary idx=7\n";
        let msg = cli_error(bad).expect("an invalid combination is a failure");
        assert!(msg.contains("Invalid combination"), "got {msg:?}");
    }

    /// Verbatim `print_mpsw_cfg` output (the column alignment is the CLI's, and the parser must not
    /// depend on it).
    #[test]
    fn parses_the_mpsw_echo() {
        let text = "\
                 MPSW Active: 1
       Airtime Minimum Bound: 500
       Airtime Maximum Bound: 0
Packet Spacing Window Length: 2000
";
        assert_eq!(
            parse_mpsw(text),
            Some(MpswConfig {
                enabled: true,
                airtime_min_us: 500,
                airtime_max_us: AIRTIME_UNLIMITED,
                window_us: 2000,
            })
        );
    }

    /// A disarmed MPSW reports `Active: 0`; that is a value, not an absence.
    #[test]
    fn parses_a_disabled_mpsw() {
        let text = "\
                 MPSW Active: 0
       Airtime Minimum Bound: 0
       Airtime Maximum Bound: 0
Packet Spacing Window Length: 0
";
        let cfg = parse_mpsw(text).expect("must parse");
        assert!(!cfg.enabled);
        assert_eq!(cfg.window_us, 0);
    }

    /// Truncated output must be a miss, never a partially-filled config.
    #[test]
    fn truncated_mpsw_is_none() {
        assert!(parse_mpsw("                 MPSW Active: 1\n").is_none());
    }

    /// Verbatim spread-mode `get_duty_cycle` output. 100.00% is how the CLI expresses "disabled".
    #[test]
    fn parses_spread_duty_cycle() {
        let text = "\
Mode: spread
Configured duty cycle: 100.00%
Control responses omitted from duty cycle calculation: 0
";
        let d = parse_duty_cycle(text).expect("must parse");
        assert_eq!(d.mode, DutyCycleMode::Spread);
        assert_eq!(d.percent, 100.0);
        assert!(!d.omit_control_responses);
        assert_eq!(d.airtime_remaining_us, None);
        assert_eq!(d.burst_window_us, None);
    }

    /// Burst mode adds two lines; both must be picked up, and the `(us)` in the key must not throw
    /// the label match.
    #[test]
    fn parses_burst_duty_cycle() {
        let text = "\
Mode: burst
Configured duty cycle: 2.50%
Control responses omitted from duty cycle calculation: 1
Airtime remaining (us): 41250
Burst window duration (us): 1000000
";
        let d = parse_duty_cycle(text).expect("must parse");
        assert_eq!(d.mode, DutyCycleMode::Burst);
        assert!((d.percent - 2.5).abs() < 1e-6);
        assert!(d.omit_control_responses);
        assert_eq!(d.airtime_remaining_us, Some(41_250));
        assert_eq!(d.burst_window_us, Some(1_000_000));
        // What `probe_duty_cycle_max` would report to the capability.
        assert!(((d.percent / 100.0) - 0.025).abs() < 1e-6);
    }

    /// `duty_cycle airtime` prints a bare number — and in spread mode prints an error instead,
    /// which `run` merges into the same string. The parser must not read a digit out of that.
    #[test]
    fn parses_duty_cycle_airtime_and_rejects_the_spread_mode_error() {
        assert_eq!(parse_duty_cycle_airtime("41250\n"), Some(41_250));
        assert_eq!(
            parse_duty_cycle_airtime("Command not supported when in spread mode\n"),
            None
        );
    }

    /// The debugfs TX-power knob reads back a plain decimal; the firmware's clamp (30 → 27) is
    /// exactly why the value read back is the one to believe.
    #[test]
    fn parses_the_debugfs_power_readback() {
        assert_eq!(parse_leading_i8("27\n"), Some(27));
        assert_eq!(parse_leading_i8("27"), Some(27));
        assert_eq!(parse_leading_i8("-3\n"), Some(-3));
        assert_eq!(parse_leading_i8("\n"), None);
        assert_eq!(parse_leading_i8("auto\n"), None);
        assert_eq!(TX_POWER_DBM_RANGE.clamp(40), 30);
        assert_eq!(TX_POWER_DBM_RANGE.clamp(0), 1);
    }

    /// ★ The channel table, checked row-by-row against `nrc-s1g.c`'s `s1g_ch_table_us[]` — the
    /// first, last and a middle entry of each width class, plus the bench's own 906 MHz / 4 MHz AP.
    #[test]
    fn us_channel_numbers_match_the_vendor_table() {
        // 1 MHz: {"US", 9025, 1}, {"US", 9045, 5}, {"US", 9275, 51}
        assert_eq!(us_s1g_channel(1), Some((902_500, 1)));
        assert_eq!(us_s1g_channel(5), Some((904_500, 1)));
        assert_eq!(us_s1g_channel(51), Some((927_500, 1)));
        // 2 MHz: {"US", 9030, 2}, {"US", 9250, 46}, {"US", 9270, 50}
        assert_eq!(us_s1g_channel(2), Some((903_000, 2)));
        assert_eq!(us_s1g_channel(46), Some((925_000, 2)));
        assert_eq!(us_s1g_channel(50), Some((927_000, 2)));
        // 4 MHz: {"US", 9060, 8} — the AP the primary-index sweep was measured against —
        // {"US", 9100, 16}, {"US", 9260, 48}
        assert_eq!(us_s1g_channel(8), Some((906_000, 4)));
        assert_eq!(us_s1g_channel(16), Some((910_000, 4)));
        assert_eq!(us_s1g_channel(48), Some((926_000, 4)));
        // 8 MHz: not in the NRC table at all (that part has none in the US); from the MEASURED
        // morse_cli/hostapd_s1g result that US 8 MHz is channels 12/28/44.
        assert_eq!(us_s1g_channel(12), Some((908_000, 8)));
        assert_eq!(us_s1g_channel(28), Some((916_000, 8)));
        assert_eq!(us_s1g_channel(44), Some((924_000, 8)));
    }

    /// Numbers outside the table must be refused, not extrapolated — an off-table channel is
    /// out-of-band transmission, which is the one error class that cannot be undone.
    #[test]
    fn off_table_channels_are_rejected() {
        assert_eq!(us_s1g_channel(0), None);
        assert_eq!(us_s1g_channel(4), None); // even, but no US row at 904.0
        assert_eq!(us_s1g_channel(52), None); // 928.0 — past the band edge
        assert_eq!(us_s1g_channel(53), None);
        assert_eq!(us_s1g_channel(255), None);
    }

    /// Every advertised channel must be tunable, every frequency must sit inside 902–928 MHz with
    /// its full width, and the list must be exactly what the four rules generate.
    #[test]
    fn the_advertised_channel_list_is_in_band_and_complete() {
        let chans = us_s1g_channels();
        assert_eq!(
            chans.len(),
            26 + 13 + 6 + 3,
            "1 MHz 26, 2 MHz 13, 4 MHz 6, 8 MHz 3"
        );
        for c in &chans {
            let (khz, bw) = us_s1g_channel(*c).expect("advertised channels must be tunable");
            let half = u32::from(bw) * 500 / 2;
            assert!(
                khz - half >= 902_000 && khz + half <= 928_000,
                "ch{c} at {khz} kHz / {bw} MHz leaves the 902-928 MHz band"
            );
        }
        assert!(chans.contains(&8) && chans.contains(&12) && chans.contains(&46));
    }

    /// The primary index a width admits, from `morse_cmd_req_set_channel`'s documented ranges.
    #[test]
    fn primary_index_bounds_follow_the_width() {
        assert_eq!(max_primary_index(1), 0);
        assert_eq!(max_primary_index(2), 1);
        assert_eq!(max_primary_index(4), 3);
        assert_eq!(max_primary_index(8), 7);
    }

    /// The capability must describe *this* radio, not the preset. The two corrections that are
    /// bugs rather than taste — the off-ladder MCS10 and the 1500 B payload — are asserted
    /// explicitly, and so is the probe-don't-assert rule for power: on a machine with no MM6108
    /// there is no debugfs knob, so the radio must report no dBm axis and no actuated power.
    #[test]
    fn capability_corrects_the_preset() {
        let k = MorseKnobs::new("wlan0", "/usr/bin/morse_cli");
        let cap = k.capability();
        assert_eq!(cap.kind, RadioKind::WifiHaLow);
        assert_eq!(cap.bands, vec![Band::Sub1GHz]);
        assert_eq!(
            cap.rate,
            RateCapability::Wifi {
                max_mcs: 7,
                max_nss: 1,
                max_bw: 0
            },
            "S1G MCS10 sits BELOW MCS0 and must not be the ladder's top"
        );
        assert_eq!(
            cap.max_payload, 1546,
            "MEASURED byte-exact: 1546 ok, 1547 dead"
        );
        assert!(cap.half_duplex);
        assert!(!cap.rx_only);
        assert_eq!(cap.channels, us_s1g_channels());

        // No MM6108 on the machine running the tests => no knob => no claim.
        assert!(k.tx_power_knob().is_none());
        assert_eq!(cap.tx_power_dbm, None);
        assert!(!cap.power_actuated);
        assert_eq!(cap.min_tx_power, None);
        assert_eq!(cap.db_per_power_idx, None);
    }

    /// A restricted channel list must reach the capability, since that is how a deployment stops
    /// cognition picking a 1 MHz channel by picking the smallest number.
    #[test]
    fn channel_list_can_be_restricted() {
        let k = MorseKnobs::new("wlan0", "/usr/bin/morse_cli").with_channels(vec![8, 16, 24]);
        assert_eq!(k.capability().channels, vec![8, 16, 24]);
    }

    /// `set_edcca_ignore(true)` must refuse rather than silently succeed; `false` is a no-op
    /// because there is nothing to undo.
    #[test]
    fn edcca_ignore_refuses_rather_than_lying() {
        let k = MorseKnobs::new("wlan0", "/usr/bin/morse_cli");
        assert!(k.set_edcca_ignore(false).is_ok());
        let e = k
            .set_edcca_ignore(true)
            .expect_err("no LBT bypass exists on this part");
        assert!(format!("{e}").contains("medium_eval"), "got {e}");
    }

    /// The LoRa width dial must not quietly become an S1G width dial.
    #[test]
    fn bandwidth_khz_refuses() {
        let k = MorseKnobs::new("wlan0", "/usr/bin/morse_cli");
        assert!(k.set_bandwidth_khz(250).is_err());
        assert!(k.set_bandwidth_khz(4000).is_err());
    }

    /// The two range checks that exist so a bad value is a typed error rather than a subprocess
    /// failure — both must reject *before* anything is spawned, which is what makes them testable
    /// on a machine with no radio.
    /// ★ Both HaLow radios must answer an inexpressible width the same way. This asserts the
    /// MM6108 half; `nrc7292::knob_tests::wifi_widths_are_refused` asserts the NRC7292 half, and
    /// `halow::tests::the_width_request_reading_is_the_one_both_radios_share` pins the reading they
    /// share. Before this, 40 MHz was refused on one part and silently ignored on the other.
    ///
    /// The width is checked before anything is spawned, so a nonexistent `morse_cli` is enough:
    /// a spawn error here would prove the check ran too late.
    #[test]
    fn wifi_widths_are_refused_and_no_preference_is_not() {
        let k = MorseKnobs::new("wlan0", "/nonexistent/morse_cli");
        for bw in [Bandwidth::Bw40, Bandwidth::Bw80] {
            let e = k.set_channel(8, bw).unwrap_err().to_string();
            assert!(
                e.contains("no S1G meaning"),
                "{bw:?} must be refused on its width, got: {e}"
            );
        }
        // Channel 8 is 4 MHz: asking for 1 MHz (Nb5) contradicts it and is refused...
        let e = k.set_channel(8, Bandwidth::Nb5).unwrap_err().to_string();
        assert!(e.contains("not 1 MHz"), "width mismatch must be named: {e}");
        // ...while Nb5 on a 1 MHz channel agrees, and `Bw20` means "no preference" on any channel.
        // Both then reach the spawn and fail there, which is a different error entirely.
        for (ch, bw) in [(1u8, Bandwidth::Nb5), (8, Bandwidth::Bw20)] {
            let e = k.set_channel(ch, bw).unwrap_err().to_string();
            assert!(
                !e.contains("no S1G meaning") && !e.contains("MHz wide, not"),
                "ch {ch} / {bw:?} must pass the width check, got: {e}"
            );
        }
    }

    /// Verbatim `sudo morse_cli_txblock -i mon0 get tx_block` from an MM6108, blocked and
    /// released. `param_get_uint32` prints a bare `"%u\n"` and nothing else.
    const TX_BLOCK_ON: &str = "1\n";
    const TX_BLOCK_OFF: &str = "0\n";

    #[test]
    fn parses_the_tx_block_readback() {
        assert_eq!(parse_tx_block(TX_BLOCK_ON), Some(true));
        assert_eq!(parse_tx_block(TX_BLOCK_OFF), Some(false));
        // The firmware's own test is `snez` (sb a5, 0x802006FE), so anything non-zero blocks even
        // though the CLI entry constrains input to 0..=1. Reading it as a bool must agree with the
        // chip, not with the CLI's validation.
        assert_eq!(parse_tx_block("7\n"), Some(true));
        // `run` concatenates stdout and stderr, so the number need not be the first line.
        assert_eq!(parse_tx_block("some transport note\n0\n"), Some(false));
        // No number at all is not "released" — it is an unparseable readout, and the getter must
        // say so rather than default to the safe-looking value.
        assert_eq!(parse_tx_block(""), None);
        assert_eq!(parse_tx_block("Invalid parameter: 'tx_block'\n"), None);
    }

    /// Verbatim output of a **stock** `morse_cli` asked to set `tx_block`: `match_str_to_param`
    /// misses, `param_get_set` prints this and returns `MORSE_ARG_ERR` (exit 1). MEASURED on
    /// mds-o5p-2 against the unpatched binary.
    #[test]
    fn an_unpatched_cli_is_reported_as_a_missing_table_entry_not_a_success() {
        let stock = "Invalid parameter: 'tx_block'\n    Available parameters:\n        \
                     traffic_delivery_wait\n";
        let msg = cli_error(stock).expect("a stock morse_cli has no id-6 entry; that is an error");
        assert!(msg.contains("Invalid parameter"), "got {msg:?}");
        assert!(msg.contains("params.c"), "must name the fix: {msg:?}");
    }

    /// The gate is real, so the failure mode to guard is the opposite of the usual one: not a knob
    /// that lies about actuating, but a knob that must never report success when the CLI it drives
    /// cannot reach the chip. With no binary to spawn, both directions must be errors.
    #[test]
    fn tx_hold_never_reports_success_without_a_cli_to_run() {
        let k = MorseKnobs::new("mon0", "/nonexistent/morse_cli");
        for hold in [true, false] {
            let e = k
                .set_tx_hold(hold)
                .expect_err("no binary to run: this must not read as a hold that took");
            let text = format!("{e}");
            assert!(text.contains("tx_block"), "must name the mechanism: {text}");
            assert!(
                text.contains(if hold { "hold" } else { "release" }),
                "must say which direction failed: {text}"
            );
        }
        assert!(
            k.tx_block().is_err(),
            "an unreadable gate is unknown, not released"
        );
    }

    #[test]
    fn out_of_range_values_are_refused_without_touching_the_radio() {
        let k = MorseKnobs::new("wlan0", "/nonexistent/morse_cli");
        assert!(k.set_tx_packet_lifetime_us(49_999).is_err());
        assert!(k.set_tx_packet_lifetime_us(500_001).is_err());
        assert!(k.set_bss_color(8).is_err());
        // min == max is rejected by mpsw.c; so is min > max unless max is AIRTIME_UNLIMITED.
        let bad = MpswConfig {
            enabled: true,
            airtime_min_us: 500,
            airtime_max_us: 500,
            window_us: 1000,
        };
        assert!(k.set_mpsw(bad).is_err());
        let inverted = MpswConfig {
            airtime_min_us: 900,
            airtime_max_us: 500,
            ..bad
        };
        assert!(k.set_mpsw(inverted).is_err());
        // ...but an unlimited upper bound is legal.
        let unlimited = MpswConfig {
            airtime_max_us: AIRTIME_UNLIMITED,
            ..bad
        };
        // Reaches the spawn and fails there (no binary), which is a *different* error — the point
        // is only that validation let it through.
        let e = format!("{}", k.set_mpsw(unlimited).expect_err("no binary to run"));
        assert!(
            !e.contains("rejected by morse_cli"),
            "must pass validation: {e}"
        );
    }
}
