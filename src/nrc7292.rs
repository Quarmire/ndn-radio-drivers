//! Newracom NRC7292 (802.11ah / S1G): the **read-now clock** that closes the gap the `AF_PACKET`
//! monitor backend documents as missing, plus this radio's **cognition control surface**
//! ([`Nrc7292Knobs`]).
//!
//! Two halves, both driven through the vendor `cli_app` (and, for tuning only, `iw`):
//!
//! * [`Nrc7292Clock`] — a microsecond TSF this radio will sample on demand, in the same domain as
//!   its per-frame RX stamps. Measured on air; see below.
//! * [`Nrc7292Knobs`] — [`RadioKnobs`] + [`RadioProfile`]. Eight methods have a real actuator or
//!   reader here — channel, absolute-dBm TX power, airtime shaping, the dBm CCA threshold, the
//!   FCS-error and carrier-sense counters, an EDCCA-ignore that *refuses* rather than lying, and
//!   the discipline declaration. **Eleven stay at the trait default**, each with a written reason
//!   in the `impl` block: an honest gap beats a false capability, and this crate has already paid
//!   once for a `set_tx_power` that returned `Ok(())` and actuated nothing.
//!
//! ★ **Read the four `cli_app` traps on [`Nrc7292Knobs`] before adding a knob.** Two of them —
//! the tool always exits 0, and it cannot be pointed at an interface — are exactly the kind that
//! make a backend report success for something that never happened.
//!
//! `ndn_frame_io::AfPacketBackend` (Linux-only, so not linkable from a macOS doc build) already
//! surfaces this radio's per-frame
//! hardware RX stamp — the driver emits radiotap TSFT on a monitor vif and our radiotap parser
//! turns it into a [`LinkStamp`](ndn_frame_io::LinkStamp) — but it reports `read_clock() = None`,
//! because a raw packet socket has no way to *sample* a NIC's clock on demand. That is true of
//! `AF_PACKET` in general and false of this particular radio: the NRC7292's firmware keeps a
//! microsecond counter in chip RAM that the vendor CLI can read at any time, and it is the **same
//! clock** that stamps received frames. This module makes that clock readable, which is what a
//! common-view/offset estimator needs in addition to the per-frame stamps.
//!
//! # What is measured, not assumed
//!
//! Everything below was established on-air on the bench (mds-o5p-0 / mds-o5p-3, 2026-08-27), not
//! read from a datasheet:
//!
//! * **The counter lives at [`TSF_MIRROR_ADDR`] and ticks in microseconds.** Over a 20 s interval
//!   it advanced 20_057_179 counts against 20_057_880 µs of wall clock — `0.999965 counts/µs`,
//!   i.e. **−35 ppm**, tight enough to be the genuine crystal offset rather than measurement error.
//! * **It is the same domain as the radiotap TSFT**, which is the property that makes it useful.
//!   Proof by bracketing: an on-air stamp read from a captured frame falls *between* two
//!   consecutive reads of this address, repeatedly —
//!   `447_602_878 < 447_749_419 < 448_791_099` and `451_838_288 < 452_050_242 < 453_018_788`.
//!   That is why [`Nrc7292Clock`] keys its domain on the interface index, exactly as
//!   `AfPacketBackend` does: the two must agree or the timekeeper cannot relate them.
//! * **A two-node common view over ordinary beacons measured `sd = 5.8 µs`** (n=108, span 19 µs)
//!   with a relative drift of `+1.50 ppm` between two NRC7292s, using only this clock and the
//!   per-frame stamps — no driver or firmware modification anywhere.
//! * **It cannot be steered.** Writing the address returns success and does not take: the value is
//!   a *software mirror* that firmware refreshes from the real hardware counter, so a written value
//!   is overwritten within a tick. [`RadioTime::clock_steering`] therefore reports `None`, honestly.
//!   Offset (phase) discipline still works — it is a correction applied to readings — but the rate
//!   cannot be changed, so a corrected offset will re-accumulate at the measured ppm.
//!
//! # Why the vendor CLI
//!
//! `cli_app` is the vendor tool shipped with the NRC7292 driver package; `read` is undocumented
//! (only `write` appears in its `help`) but present and stable. Shelling out is the honest
//! bootstrap, not the destination: the same request is carried by the driver's generic netlink
//! family (`NRC-NL-FAM`), and a native client there would remove the subprocess, the root
//! requirement and the ~1 ms per-read cost. This module is deliberately small so that swapping the
//! transport later touches one function (`Nrc7292Clock::read_raw`, private) and nothing else.

use std::path::PathBuf;
use std::process::Command;
use std::time::Duration;

use ndn_frame_io::{ClockDomainId, FaceError, RadioTime, RadioTimeSource};
use ndn_radio_hal::{
    Bandwidth, ContentionApplied, ContentionPosture, DbmRange, RadioCapability, RadioKnobs,
    RadioProfile, TxDiscipline,
};

/// Address of the firmware's microsecond TSF mirror in NRC7292 chip RAM.
///
/// Found empirically rather than from source: the counter's *magnitude* is known from the on-air
/// radiotap TSFT, so a single RAM sweep (`0x00054000..0x000b8000`) filtered to that magnitude
/// yields ~1.7k candidates (mostly code), and re-reading those twice and keeping only the ones
/// advancing at ~1e6/s leaves five — of which this one brackets the on-air stamp.
pub const TSF_MIRROR_ADDR: u32 = 0x0006_0a74;

/// Nanoseconds of precision to advertise for this radio's per-frame stamps: the TSF is a
/// microsecond counter, so one tick.
const STAMP_PRECISION_NS: u32 = 1_000;

/// Reads the NRC7292's microsecond clock on demand.
///
/// Construct one per monitor interface; the [`ClockDomainId`] is derived from the interface index
/// so that it matches the domain `AfPacketBackend` uses for the same interface's per-frame stamps.
pub struct Nrc7292Clock {
    /// Interface this clock belongs to (e.g. `halow0`), used only for diagnostics.
    iface: String,
    /// Clock domain, keyed on the interface index to agree with the `AF_PACKET` RX stamps.
    domain: ClockDomainId,
    /// Path to the vendor `cli_app` binary.
    cli: PathBuf,
    /// Wall-clock ceiling per `cli_app` invocation; `None` runs it bare. Defaults to 5 s.
    ///
    /// ⚠ **Not optional in practice.** `cli_app` has no timeout of its own and blocks forever when
    /// the firmware is down — MEASURED on this bench. This type is reached through
    /// [`RadioTime::read_clock`], which is a *synchronous* trait method, so an un-timed read wedges
    /// whichever thread the timekeeper called it from, and the face has no way to recover. The
    /// [`Nrc7292Knobs`] path has always wrapped its invocations; this one did not, and
    /// [`crate::halow`]'s `Nrc7292FrameIo::with_clock` is what newly routes a face into it.
    timeout: Option<Duration>,
}

impl Nrc7292Clock {
    /// Bind to `iface`, resolving its interface index to form the clock domain.
    ///
    /// `cli` is the vendor `cli_app` binary (absolute path recommended, since this typically runs
    /// under `sudo` with a reduced `PATH`).
    pub fn new(iface: impl Into<String>, cli: impl Into<PathBuf>) -> Result<Self, FaceError> {
        let iface = iface.into();
        let ifindex = std::fs::read_to_string(format!("/sys/class/net/{iface}/ifindex"))
            .map_err(FaceError::Io)?
            .trim()
            .parse::<u32>()
            .map_err(|e| FaceError::Io(std::io::Error::new(std::io::ErrorKind::InvalidData, e)))?;
        Ok(Self {
            iface,
            domain: ClockDomainId(ifindex),
            cli: cli.into(),
            timeout: Some(Duration::from_secs(5)),
        })
    }

    /// Change (or with `None`, remove) the per-invocation timeout — same contract and same warning
    /// as [`Nrc7292Knobs::with_timeout`]. The wrapper is GNU `timeout`.
    pub fn with_timeout(mut self, timeout: Option<Duration>) -> Self {
        self.timeout = timeout;
        self
    }

    /// The clock domain this radio stamps in — shared with its per-frame RX stamps.
    pub fn domain(&self) -> ClockDomainId {
        self.domain
    }

    /// Read one 32-bit word of chip memory through the vendor CLI.
    ///
    /// This is the single point of contact with the transport; a native `NRC-NL-FAM` netlink
    /// client would replace exactly this function.
    fn read_raw(&self, addr: u32) -> Result<u32, FaceError> {
        let mut cmd = match self.timeout {
            Some(d) => {
                let mut c = Command::new("timeout");
                c.arg(d.as_secs().max(1).to_string());
                c.arg(&self.cli);
                c
            }
            None => Command::new(&self.cli),
        };
        let out = cmd
            .args(["read", &format!("0x{addr:08x}"), "4"])
            .output()
            .map_err(FaceError::Io)?;
        if out.status.code() == Some(TIMEOUT_EXIT_STATUS) && self.timeout.is_some() {
            return Err(FaceError::Io(std::io::Error::new(
                std::io::ErrorKind::TimedOut,
                format!(
                    "{}: cli_app read 0x{addr:08x} timed out — the tool has no timeout of its own \
                     and hangs when the firmware is down; check the radio before retrying",
                    self.iface
                ),
            )));
        }
        let text = String::from_utf8_lossy(&out.stdout);
        parse_cli_word(&text, addr).ok_or_else(|| {
            FaceError::Io(std::io::Error::new(
                std::io::ErrorKind::InvalidData,
                format!(
                    "{}: no word for 0x{addr:08x} in cli_app output ({} bytes)",
                    self.iface,
                    text.len()
                ),
            ))
        })
    }

    /// Read the microsecond TSF.
    pub fn read_tsf_us(&self) -> Result<u64, FaceError> {
        Ok(u64::from(self.read_raw(TSF_MIRROR_ADDR)?))
    }
}

/// Extract the value for `addr` from `cli_app read` output.
///
/// The tool prints a banner, then `<addr>: <value>` lines, then a trailing `OK`. Two traps this
/// guards against, both hit on the bench: the `OK` trailer is *not* a value (a naive "last line"
/// parse yields it), and a block read returns at most ~24 words regardless of the requested
/// length, so the requested address is not necessarily the only — or last — line present.
fn parse_cli_word(out: &str, addr: u32) -> Option<u32> {
    let want = format!("{addr:08x}");
    for line in out.lines() {
        let line = line.trim();
        let Some((a, v)) = line.split_once(':') else {
            continue;
        };
        let (a, v) = (a.trim(), v.trim());
        if a.len() != 8 || !a.chars().all(|c| c.is_ascii_hexdigit()) {
            continue;
        }
        if a.eq_ignore_ascii_case(&want) {
            let v = v.split_whitespace().next()?;
            return u32::from_str_radix(v, 16).ok();
        }
    }
    None
}

/// Frame Control of an S1G Beacon: protocol version 0, type 3 (Extension), subtype 1.
///
/// `tcpdump` renders these as `unknown 802.11 frame type (3)` because it does not decode the S1G
/// extension frames.
const S1G_BEACON_FC0: u8 = 0x1c;

/// Byte offset of the transmitter address within an S1G Beacon.
const S1G_SA_OFFSET: usize = 4;

/// Byte offset of the 4-octet partial TSF (little-endian) within an S1G Beacon.
const S1G_TSF_OFFSET: usize = 10;

/// A peer's clock offset, derived from one S1G beacon.
///
/// This is the raw ingredient of a common view: the beacon carries the transmitter's own TSF at
/// transmission, and the receiver stamps its arrival from *its* TSF, so the difference is the
/// inter-node clock offset (plus a constant flight/processing term that cancels when you look at
/// the offset's *stability* rather than its absolute value).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct BeaconOffset {
    /// Transmitter address (BSSID) — group offsets by this; two APs are two clocks.
    pub sa: [u8; 6],
    /// Second Frame Control octet. **Group by this too** — see [`s1g_beacon_offset`].
    pub fc1: u8,
    /// The beacon's partial TSF, microseconds (32-bit, wraps every ~71.6 minutes).
    pub peer_tsf_us: u32,
    /// `peer_tsf_us - local_tsft_us`, the inter-node offset in microseconds.
    pub offset_us: i64,
}

/// Compute the peer-clock offset from one captured S1G beacon.
///
/// `frame` is the 802.11 frame *after* the radiotap header; `local_tsft_us` is the radiotap TSFT
/// this radio stamped it with. Returns `None` for anything that is not an S1G beacon.
///
/// ⚠ **Group results by `(sa, fc1)` before computing statistics.** This is not fastidiousness — it
/// is the difference between a right and a wrong answer, measured. Applying a fixed
/// `S1G_TSF_OFFSET` (private) to every captured frame mixes in S1G beacon variants whose layout
/// differs (the Frame Control bits select which of Next-TBTT / Compressed-SSID / ANO are present),
/// and the resulting offset series reads **sd = 1004 µs, span 3366 µs**. Grouping the *same
/// capture* by `fc1` collapses it to **sd = 5.4 µs (n=12, fc1=0x08)** and **sd = 5.8 µs (n=108,
/// fc1=0x0b)**, which independently agree on drift to 0.01 ppm. A 170x error that looks plausible
/// is exactly the kind this codebase keeps paying for.
pub fn s1g_beacon_offset(frame: &[u8], local_tsft_us: u64) -> Option<BeaconOffset> {
    if frame.len() < S1G_TSF_OFFSET + 4 || frame[0] != S1G_BEACON_FC0 {
        return None;
    }
    let mut sa = [0u8; 6];
    sa.copy_from_slice(&frame[S1G_SA_OFFSET..S1G_SA_OFFSET + 6]);
    let peer_tsf_us =
        u32::from_le_bytes(frame[S1G_TSF_OFFSET..S1G_TSF_OFFSET + 4].try_into().ok()?);
    // The beacon TSF is 32-bit while the local stamp is 64-bit; compare in the beacon's modulus so
    // the difference stays meaningful across the ~71.6 minute wrap.
    let local32 = local_tsft_us as u32;
    let offset_us = i64::from(peer_tsf_us.wrapping_sub(local32) as i32);
    Some(BeaconOffset {
        sa,
        fc1: frame[1],
        peer_tsf_us,
        offset_us,
    })
}

/// The NRC7292 exposes a per-frame hardware RX stamp (radiotap TSFT, surfaced by the `AF_PACKET`
/// backend) **and** — unlike a generic `AF_PACKET` NIC — the same clock is readable on demand.
impl RadioTime for Nrc7292Clock {
    /// Reference: **crystal**, from the module's own on-air measurement rather than a datasheet.
    /// Over 20 s the counter advanced 20_057_179 counts against 20_057_880 us of wall clock —
    /// **-35 ppm**, which is crystal territory (an RC reference is percent-class: the LR2021's
    /// MEASURED +2253 ppm, the Waveshare's ~-3100 ppm). Corroborated on the axis that actually
    /// matters: two NRC7292s held a common view at **sd 5.8 us** with a relative drift of
    /// **+1.50 ppm**, which is a rate that stays put.
    ///
    /// The witness recorded here is the host-clock figure, because it is the one taken against a
    /// stated span. The +1.50 ppm peer figure is the tighter statement about a common view and is
    /// in the module docs.
    fn time_sources(&self) -> Vec<RadioTimeSource> {
        vec![
            RadioTimeSource::free_run_rx_stamp(self.domain, STAMP_PRECISION_NS).with_reference(
                ndn_frame_io::ClockReference::crystal().measured(
                    ndn_frame_io::RateMeasurement::new(
                        -35.0,
                        20.06,
                        ndn_frame_io::RateWitness::HostClock,
                    ),
                ),
            ),
        ]
    }

    /// Sample the microsecond TSF. Answers only this radio's own domain.
    fn read_clock(&self, domain: ClockDomainId) -> Result<Option<u64>, FaceError> {
        if domain != self.domain {
            return Ok(None);
        }
        self.read_tsf_us().map(Some)
    }

    // `clock_steering` stays `None`: MEASURED — writing the mirror does not take, because firmware
    // refreshes it from the hardware counter. See the module docs.
}

// ===========================================================================
// The cognition control surface: `RadioKnobs` + `RadioProfile`
// ===========================================================================
//
// Everything below drives the same vendor `cli_app` the clock above uses, plus `iw` for the one
// thing `cli_app` cannot do (tune). Read the traps in [`Nrc7292Knobs`] before adding a knob here —
// two of them (the exit status, the missing interface selector) are the kind that make a backend
// report success for something that never happened.

/// One row of the NRC7292's channel table, **joined from the two vendor tables that each hold half
/// of it** — the board-data map `g_bd_ch_table` in `nrc-bd.c` (S1G frequency ↔ the 2.4/5 GHz
/// *shadow* frequency mac80211 sees ↔ the alias channel number) and the regulatory table
/// `s1g_ch_table_us` in `nrc-s1g.c` (S1G channel number → width).
///
/// ★ **The width is a property of the channel number, not an independent setting.** That is the
/// fact that dissolves most of the [`Bandwidth`] mismatch on this part: there is no width dial to
/// drive, so [`Nrc7292Knobs::set_channel`] *validates* the caller's `bw` against the row instead of
/// trying to program it.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct S1gChannelRow {
    /// The alias ("non-S1G") channel number. ★ **This is what `iw` and mac80211 speak**, because
    /// `CONFIG_S1G_CHANNEL` is commented out in the vendor `nrc-build-config.h`, so the driver
    /// registers a 2.4/5 GHz *shadow* band and the S1G map lives only in the tables below.
    pub alias: u8,
    /// The shadow centre frequency, MHz — what the driver puts in the WIM channel TLV
    /// (`nrc_mac_add_tlv_channel`: `ch_param.channel = chandef->chan->center_freq`) and therefore
    /// what `show config` reports as `MAC80211_freq`. The read-back check compares against this.
    pub shadow_mhz: u16,
    /// The real S1G centre frequency in units of 100 kHz, exactly as the vendor tables carry it
    /// (`9250` = 925.0 MHz). Kept in the vendor's own unit so a reader can diff this table against
    /// `nrc-bd.c` by eye.
    pub s1g_freq_100khz: u16,
    /// The S1G channel number (the *real* one, not the alias).
    pub s1g_channel: u8,
    /// Channel width, MHz. ★ The US table has **1, 2 and 4 MHz only — no 8 MHz channels at all**,
    /// unlike the Morse MM6108.
    pub bw_mhz: u8,
}

impl S1gChannelRow {
    /// The real S1G centre frequency in kHz — the unit `morse_cli` speaks, so this is what to use
    /// when aligning an NRC7292 and an MM6108 on the same air.
    ///
    /// ⚠ **Align cross-vendor work on frequency, never on channel numbers.** The Morse's fake
    /// channel 36 and the NRC's alias 5 are different numbers for different air; only the kHz is
    /// comparable.
    pub const fn s1g_khz(&self) -> u32 {
        self.s1g_freq_100khz as u32 * 100
    }
}

/// The **US** channel table, transcribed by joining `g_bd_ch_table[US]` (`nrc-bd.c`) with
/// `s1g_ch_table_us` (`nrc-s1g.c`) on the S1G frequency. Both tables have 45 rows and the join is
/// total — every board-data row has a regulatory row and the S1G channel numbers agree.
///
/// Cross-validated against two independently recorded bench facts: alias **161 → 925.0 MHz, 2 MHz**
/// (the `RadioCapability::wifi_halow_s1g` doc comment) and alias **8 → 906.0 MHz, 4 MHz** (the
/// NRC7292 AP the Morse note was tuned against). Alias 5 → 904.5 MHz at 1 MHz likewise matches.
///
/// ⚠ **This is the compiled ceiling, not the truth for a given unit.** The per-unit board data can
/// restrict it further (`nrc_set_supp_ch_list`), and no `cli_app` command enumerates what a
/// particular radio actually supports. A failed [`Nrc7292Knobs::set_channel`] is what prunes it —
/// which is precisely why that method insists on a read-back.
///
/// ⚠ Only the US table is transcribed. `nrc-s1g.c` carries ten per-country tables and `nrc-bd.c`
/// ten shadow maps; another regdomain needs its own pair transcribed and handed to
/// [`Nrc7292Knobs::with_channels`]. Nothing here guesses one.
/// `#[rustfmt::skip]`: one row per line is load-bearing — this table exists to be diffed by eye
/// against `g_bd_ch_table[US]` in `nrc-bd.c`, and rustfmt would split every row across six lines.
#[rustfmt::skip]
pub const US_S1G_CHANNELS: [S1gChannelRow; 45] = [
    // ---- 1 MHz ----
    S1gChannelRow { alias: 1, shadow_mhz: 2412, s1g_freq_100khz: 9025, s1g_channel: 1, bw_mhz: 1 },
    S1gChannelRow { alias: 3, shadow_mhz: 2422, s1g_freq_100khz: 9035, s1g_channel: 3, bw_mhz: 1 },
    S1gChannelRow { alias: 5, shadow_mhz: 2432, s1g_freq_100khz: 9045, s1g_channel: 5, bw_mhz: 1 },
    S1gChannelRow { alias: 7, shadow_mhz: 2442, s1g_freq_100khz: 9055, s1g_channel: 7, bw_mhz: 1 },
    S1gChannelRow { alias: 9, shadow_mhz: 2452, s1g_freq_100khz: 9065, s1g_channel: 9, bw_mhz: 1 },
    S1gChannelRow { alias: 11, shadow_mhz: 2462, s1g_freq_100khz: 9075, s1g_channel: 11, bw_mhz: 1 },
    S1gChannelRow { alias: 36, shadow_mhz: 5180, s1g_freq_100khz: 9085, s1g_channel: 13, bw_mhz: 1 },
    S1gChannelRow { alias: 37, shadow_mhz: 5185, s1g_freq_100khz: 9095, s1g_channel: 15, bw_mhz: 1 },
    S1gChannelRow { alias: 38, shadow_mhz: 5190, s1g_freq_100khz: 9105, s1g_channel: 17, bw_mhz: 1 },
    S1gChannelRow { alias: 39, shadow_mhz: 5195, s1g_freq_100khz: 9115, s1g_channel: 19, bw_mhz: 1 },
    S1gChannelRow { alias: 40, shadow_mhz: 5200, s1g_freq_100khz: 9125, s1g_channel: 21, bw_mhz: 1 },
    S1gChannelRow { alias: 41, shadow_mhz: 5205, s1g_freq_100khz: 9135, s1g_channel: 23, bw_mhz: 1 },
    S1gChannelRow { alias: 42, shadow_mhz: 5210, s1g_freq_100khz: 9145, s1g_channel: 25, bw_mhz: 1 },
    S1gChannelRow { alias: 43, shadow_mhz: 5215, s1g_freq_100khz: 9155, s1g_channel: 27, bw_mhz: 1 },
    S1gChannelRow { alias: 44, shadow_mhz: 5220, s1g_freq_100khz: 9165, s1g_channel: 29, bw_mhz: 1 },
    S1gChannelRow { alias: 45, shadow_mhz: 5225, s1g_freq_100khz: 9175, s1g_channel: 31, bw_mhz: 1 },
    S1gChannelRow { alias: 46, shadow_mhz: 5230, s1g_freq_100khz: 9185, s1g_channel: 33, bw_mhz: 1 },
    S1gChannelRow { alias: 47, shadow_mhz: 5235, s1g_freq_100khz: 9195, s1g_channel: 35, bw_mhz: 1 },
    S1gChannelRow { alias: 48, shadow_mhz: 5240, s1g_freq_100khz: 9205, s1g_channel: 37, bw_mhz: 1 },
    S1gChannelRow { alias: 100, shadow_mhz: 5500, s1g_freq_100khz: 9255, s1g_channel: 47, bw_mhz: 1 },
    S1gChannelRow { alias: 104, shadow_mhz: 5520, s1g_freq_100khz: 9265, s1g_channel: 49, bw_mhz: 1 },
    S1gChannelRow { alias: 108, shadow_mhz: 5540, s1g_freq_100khz: 9275, s1g_channel: 51, bw_mhz: 1 },
    S1gChannelRow { alias: 149, shadow_mhz: 5745, s1g_freq_100khz: 9215, s1g_channel: 39, bw_mhz: 1 },
    S1gChannelRow { alias: 150, shadow_mhz: 5750, s1g_freq_100khz: 9225, s1g_channel: 41, bw_mhz: 1 },
    S1gChannelRow { alias: 151, shadow_mhz: 5755, s1g_freq_100khz: 9235, s1g_channel: 43, bw_mhz: 1 },
    S1gChannelRow { alias: 152, shadow_mhz: 5760, s1g_freq_100khz: 9245, s1g_channel: 45, bw_mhz: 1 },
    // ---- 2 MHz ----
    S1gChannelRow { alias: 2, shadow_mhz: 2417, s1g_freq_100khz: 9030, s1g_channel: 2, bw_mhz: 2 },
    S1gChannelRow { alias: 6, shadow_mhz: 2437, s1g_freq_100khz: 9050, s1g_channel: 6, bw_mhz: 2 },
    S1gChannelRow { alias: 10, shadow_mhz: 2457, s1g_freq_100khz: 9070, s1g_channel: 10, bw_mhz: 2 },
    S1gChannelRow { alias: 112, shadow_mhz: 5560, s1g_freq_100khz: 9270, s1g_channel: 50, bw_mhz: 2 },
    S1gChannelRow { alias: 153, shadow_mhz: 5765, s1g_freq_100khz: 9090, s1g_channel: 14, bw_mhz: 2 },
    S1gChannelRow { alias: 154, shadow_mhz: 5770, s1g_freq_100khz: 9110, s1g_channel: 18, bw_mhz: 2 },
    S1gChannelRow { alias: 155, shadow_mhz: 5775, s1g_freq_100khz: 9130, s1g_channel: 22, bw_mhz: 2 },
    S1gChannelRow { alias: 156, shadow_mhz: 5780, s1g_freq_100khz: 9150, s1g_channel: 26, bw_mhz: 2 },
    S1gChannelRow { alias: 157, shadow_mhz: 5785, s1g_freq_100khz: 9170, s1g_channel: 30, bw_mhz: 2 },
    S1gChannelRow { alias: 158, shadow_mhz: 5790, s1g_freq_100khz: 9190, s1g_channel: 34, bw_mhz: 2 },
    S1gChannelRow { alias: 159, shadow_mhz: 5795, s1g_freq_100khz: 9210, s1g_channel: 38, bw_mhz: 2 },
    S1gChannelRow { alias: 160, shadow_mhz: 5800, s1g_freq_100khz: 9230, s1g_channel: 42, bw_mhz: 2 },
    S1gChannelRow { alias: 161, shadow_mhz: 5805, s1g_freq_100khz: 9250, s1g_channel: 46, bw_mhz: 2 },
    // ---- 4 MHz ----
    S1gChannelRow { alias: 8, shadow_mhz: 2447, s1g_freq_100khz: 9060, s1g_channel: 8, bw_mhz: 4 },
    S1gChannelRow { alias: 116, shadow_mhz: 5580, s1g_freq_100khz: 9260, s1g_channel: 48, bw_mhz: 4 },
    S1gChannelRow { alias: 162, shadow_mhz: 5810, s1g_freq_100khz: 9100, s1g_channel: 16, bw_mhz: 4 },
    S1gChannelRow { alias: 163, shadow_mhz: 5815, s1g_freq_100khz: 9140, s1g_channel: 24, bw_mhz: 4 },
    S1gChannelRow { alias: 164, shadow_mhz: 5820, s1g_freq_100khz: 9180, s1g_channel: 32, bw_mhz: 4 },
    S1gChannelRow { alias: 165, shadow_mhz: 5825, s1g_freq_100khz: 9220, s1g_channel: 40, bw_mhz: 4 },
];

/// Lowest TX power the vendor API accepts, dBm (`nrc_wifi_set_tx_power`, `api_wifi.h`: "1~30").
pub const TXPWR_DBM_MIN: i8 = 1;
/// Highest TX power the vendor API accepts, dBm. ⚠ Whether the radio *reaches* it is a board-data
/// and regulatory question; believe the value [`Nrc7292Knobs::set_tx_power_dbm`] returns.
pub const TXPWR_DBM_MAX: i8 = 30;

/// Most sensitive CCA threshold, dBm (`cli_app` usage string + `nrc_wifi_set_cca_threshold`).
pub const CCA_THRESH_MIN_DBM: i8 = -100;
/// Least sensitive CCA threshold, dBm — the "defer less" end, the spatial-reuse direction.
pub const CCA_THRESH_MAX_DBM: i8 = -35;

/// Largest carrier-sense time `set tx_time` accepts, µs.
///
/// ★ **13260, not 65535** — `wifi_api_set_tx_time` (`wifi_api.c`) rejects `cs_time > 13260` with
/// `-EINVAL`, while `api_wifi.h` documents the range as 0..12480 µs. The bench measured
/// "values > 65535 are REJECTED and the previous setting survives"; that is consistent, because
/// 65535 > 13260. A guard written against `u16::MAX` would let 13261..65535 through and the write
/// would silently no-op.
pub const TX_TIME_CS_MAX_US: u32 = 13_260;

/// Largest TX pause `set tx_time` accepts through the host CLI, µs.
///
/// ⚠ **The two layers disagree and the smaller one wins.** The modem API takes a `uint32_t`
/// (`system_modem_api_set_tx_pause_time`), but the bench MEASURED the host shell rejecting
/// anything above 65535 — so `u16` is the real bound and 65534 is the measured floor of the dial.
pub const TX_TIME_PAUSE_MAX_US: u32 = 65_535;

/// The S1G slot time this crate reports in [`ContentionApplied::slot_us`].
///
/// ⚠ **NOT read from the radio.** 52 µs is the 802.11ah aSlotTime from the standard; the NRC7292
/// exposes no slot reader anywhere in the `cli_app` surface, and `ContentionApplied` has no
/// "unknown" position. The HAL's own doc insists "read `slot_us`, do not assume 9 µs" — this part
/// makes that impossible, so the number is a *standards estimate* and every airtime budget derived
/// from it inherits that status. Settling it needs a bench measurement (e.g. inter-frame spacing
/// under a known window), not another datasheet.
pub const S1G_SLOT_US: u8 = 52;

/// Which `[AC]` index in `show edca` is Best Effort.
///
/// From `mac80211_to_nrc_aci_map[4] = {3, 2, 1, 0}` in `nrc-mac80211.c`: mac80211's AC order is
/// VO=0, VI=1, BE=2, BK=3, so the NRC's own index for Best Effort is **1**.
/// ⚠ UNVERIFIED that `show edca`'s `[AC]` column uses that same numbering — it is the firmware's
/// array index and the driver only ever writes into it. [`Nrc7292Knobs::edca`] returns all four so
/// a caller can check rather than trust this.
pub const EDCA_AC_BE: u8 = 1;

/// Exit status GNU `timeout` uses when it had to kill the child.
const TIMEOUT_EXIT_STATUS: i32 = 124;

/// `set tx_time` state, as `show tx_time` reports it (µs).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct TxTime {
    /// Listen-before-talk carrier-sense window. Source-confirmed as the LMAC's LBT time
    /// (`wifi_api_set_tx_time` → `system_modem_api_set_cs_time` → `lmac_lbt_set_cs_time`), which is
    /// why **raising it lowers throughput**.
    pub cs_us: u32,
    /// TX pause between transmissions.
    pub pause_us: u32,
    /// Resume time. Reported by the firmware, not settable through `set tx_time`; carried here
    /// because `show tx_time` prints it and dropping a field a radio volunteers is how readings
    /// get misattributed.
    pub resume_us: u32,
}

/// What `set txpwr` echoed back.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct TxPowerApplied {
    /// `auto` | `limit` | `fixed`.
    pub kind: String,
    /// The dBm figure the firmware printed.
    ///
    /// ⚠ **UNVERIFIED whether this is the REQUESTED value or the board-data/regulatory CLAMPED
    /// one**, and that distinction is the entire reason the HAL returns `i8` instead of `()`. The
    /// settling measurement: command 30 dBm on a channel whose `nrc_read_bd_tx_pwr` ceiling is
    /// lower and see whether the echo moves. Until someone runs it, treat this as "the number the
    /// firmware printed", not as "the power on the antenna".
    pub dbm: i8,
}

/// One access category from `show edca`.
///
/// ⚠ **Reported, never programmed by this crate.** The NRC7292 exposes no userspace EDCA setter —
/// `nrc_mac_conf_tx` is a kernel path driven by hostapd/mac80211 — so these values describe what
/// the firmware happens to be running, and [`Nrc7292Knobs::set_contention`] shapes airtime through
/// an entirely different mechanism.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct EdcaAc {
    /// The firmware's AC index (see [`EDCA_AC_BE`]).
    pub ac: u8,
    /// Arbitration inter-frame spacing number, in slots.
    pub aifsn: u8,
    /// Contention-window minimum **as a window value** (15, 1023, …), not an exponent — the WIM
    /// TLV field is fed straight from mac80211's `ieee80211_tx_queue_params.cw_min`
    /// (`nrc-mac80211.c`), which is the window. [`cw_window_to_exponent`] converts.
    pub cw_min: u16,
    /// Contention-window maximum, same units as [`cw_min`](Self::cw_min).
    pub cw_max: u16,
    /// TXOP limit in 32 µs units (mac80211's unit, passed through unchanged by the driver).
    pub txop_limit: u16,
    /// TXOP max, as the firmware reports it. No mac80211 counterpart; carried, not interpreted.
    pub txop_max: u16,
}

/// The header line of `show mac tx stats` — MAC-level MPDU accounting.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Default)]
pub struct MacTxStats {
    /// MPDUs that succeeded.
    pub ok: u32,
    /// Retransmissions.
    pub rtx: u32,
    /// Last MCS the MAC transmitted at.
    pub last_mcs: u8,
}

/// The header line of `show mac rx stats`.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Default)]
pub struct MacRxStats {
    /// MPDUs received OK.
    pub ok: u32,
    /// MPDUs received not-OK.
    pub nok: u32,
    /// Last MCS the MAC received at.
    pub last_mcs: u8,
    /// ★ **PPDUs the PHY began to demodulate and failed** — the collision / marginal-decode
    /// signature, and the one field on this radio that means exactly what
    /// [`RadioKnobs::read_ofdm_counters`] asks for. Present only in the RX direction.
    pub fcs_error: u32,
}

/// `show stats simple_rx` — the device-scoped receive counters.
///
/// Device-scoped matters: `show signal`, `show sta`, `show rc` and `show uinfo` are
/// **association-scoped** and print `N/A` on an unassociated monitor-mode node, which is exactly
/// what a named-radio node is.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Default)]
pub struct SimpleRxStats {
    /// Last RSSI, dBm.
    pub rssi_dbm: i32,
    /// ★ Carrier-sense count — activity the firmware counts without the host decoding anything,
    /// the same shape as the 8812au's `REG_RXERR_RPT`. See [`RadioKnobs::read_channel_activity`]
    /// for the UNVERIFIED part.
    pub cs_cnt: u32,
    /// PSDUs successfully received.
    pub psdu_succ: u32,
    /// MPDUs received.
    pub mpdu_rcv: u32,
    /// MPDUs successfully received.
    pub mpdu_succ: u32,
    /// Last SNR, dB.
    pub snr: u32,
}

/// The fields of `show config` this crate reads.
///
/// Deliberately partial and deliberately typed loosely: the *key names* are source-verified (they
/// are printed by `cli_app` itself from `SHOW_CONFIG_KEY_LIST`), but the *units* of the firmware's
/// values are not. Only [`mac80211_freq_mhz`](Self::mac80211_freq_mhz) is parsed as a number,
/// because only it has a source-grounded unit.
#[derive(Clone, Debug, Default, PartialEq, Eq)]
pub struct NrcConfig {
    /// `Device Mode` — e.g. `MONITOR`.
    pub device_mode: Option<String>,
    /// `Country` — the regdomain the channel table and the power ceiling both depend on.
    pub country: Option<String>,
    /// `Bandwidth`, verbatim. ⚠ Unit UNVERIFIED (MHz? a width code?), so it is not parsed.
    pub bandwidth: Option<String>,
    /// `Frequency`, verbatim. ⚠ Unit UNVERIFIED — the vendor tables carry S1G frequency in 100 kHz
    /// units, so `9250` would be 925.0 MHz, but nothing here proves the firmware prints that unit.
    pub frequency: Option<String>,
    /// `MAC80211_freq`, MHz. **The one field with a grounded unit**: the driver puts
    /// `chandef->chan->center_freq` — the shadow frequency, in MHz — into the WIM channel TLV, so
    /// this is directly comparable to [`S1gChannelRow::shadow_mhz`]. It is what
    /// [`Nrc7292Knobs::verify_channel`] checks.
    pub mac80211_freq_mhz: Option<u16>,
    /// `Tx Power Type` — `auto` / `limit` / `fixed`.
    pub tx_power_type: Option<String>,
}

/// **The NRC7292's cognition control surface.**
///
/// Wraps the vendor `cli_app` (every knob but tuning) and `iw` (tuning). Four traps a reader must
/// know before touching this, all source-verified against the vendor package on this machine:
///
/// * ★ **`cli_app` ALWAYS EXITS 0.** `main.c` is literally `cli_app_run_onetime(...); return 0;`,
///   and `cli_util.c` prints `OK` or `FAIL` and returns 0 either way. **Success is the trailing
///   `OK` token on stdout and nothing else.** A knob that reads the exit status reports every
///   failure as a success — the defect class this crate exists to avoid. [`Nrc7292Knobs::cli`] is
///   the single place that check lives.
/// * ★★ **`cli_app` has NO interface selector.** `cli_netlink.c` resolves one generic-netlink
///   family (`NRC-NL-FAM`); there is no ifindex and no `-i`. On a host with two NRC7292s every knob
///   below lands on an *unspecified* one, while [`RadioProfile::capability`] and
///   [`Nrc7292Clock`]'s domain are per-radio. The `iface` field here is honest about only what it
///   is used for: `iw` (which *is* per-interface) and diagnostics. A multi-radio node cannot
///   currently be driven safely through this backend, and the HAL has no way to say so.
/// * ⚠ **`cli_app` has no timeout of its own and HANGS when the firmware is down.** Every
///   invocation is wrapped in GNU `timeout` unless the caller disables it with
///   [`with_timeout`](Self::with_timeout).
/// * ⚠ **`show mac {tx,rx} stats` costs ≥ 300 ms.** `cmd_show_mac_stats` issues three netlink
///   round-trips with a 100 ms delay between them (and retries up to 100×). Fine occasionally,
///   wrong inside a per-second sampler.
pub struct Nrc7292Knobs {
    /// Interface this backend describes (e.g. `halow0`). Used by `iw` and in error messages —
    /// **not** by `cli_app`, which cannot be pointed at an interface at all.
    iface: String,
    /// Path to the vendor `cli_app` binary.
    cli: PathBuf,
    /// Path to `iw` (the only tool that can tune this radio).
    iw: PathBuf,
    /// Wall-clock ceiling per `cli_app` invocation; `None` runs it bare.
    timeout: Option<Duration>,
    /// The channel table this radio is being driven against — [`US_S1G_CHANNELS`] by default.
    channels: Vec<S1gChannelRow>,
}

impl Nrc7292Knobs {
    /// Bind to `iface`, driving the vendor `cli_app` at `cli`.
    ///
    /// Defaults: the US channel table, `iw` from `PATH`, and a 5 s timeout on every `cli_app` call.
    /// The process needs root for both tools.
    pub fn new(iface: impl Into<String>, cli: impl Into<PathBuf>) -> Self {
        Self {
            iface: iface.into(),
            cli: cli.into(),
            iw: PathBuf::from("iw"),
            timeout: Some(Duration::from_secs(5)),
            channels: US_S1G_CHANNELS.to_vec(),
        }
    }

    /// Point at a specific `iw` binary (absolute path recommended under `sudo`).
    pub fn with_iw(mut self, iw: impl Into<PathBuf>) -> Self {
        self.iw = iw.into();
        self
    }

    /// Change (or with `None`, remove) the per-invocation timeout.
    ///
    /// ⚠ Removing it means a firmware fault hangs the calling thread forever — which is exactly
    /// what happens on the bench. The wrapper is GNU `timeout`; if that binary is absent, set
    /// `None` and accept the risk rather than silently getting a spawn failure on every knob.
    pub fn with_timeout(mut self, timeout: Option<Duration>) -> Self {
        self.timeout = timeout;
        self
    }

    /// Drive a different regdomain's channel table (see [`US_S1G_CHANNELS`] for why only US ships).
    pub fn with_channels(mut self, channels: Vec<S1gChannelRow>) -> Self {
        self.channels = channels;
        self
    }

    /// The interface this backend was bound to.
    pub fn iface(&self) -> &str {
        &self.iface
    }

    /// Build the [`Nrc7292Clock`] for the same interface.
    ///
    /// Exists to close a composition gap: the `AF_PACKET` monitor backend supplies this radio's
    /// per-frame RX stamps but reports `read_clock() = None`, and the two halves have to come from
    /// one place for a common-view estimator to use them together.
    /// The clock inherits this backend's `cli_app` timeout, so a face composed through
    /// [`crate::halow`] cannot end up with an un-timed synchronous read.
    pub fn clock(&self) -> Result<Nrc7292Clock, FaceError> {
        Ok(Nrc7292Clock::new(self.iface.clone(), self.cli.clone())?.with_timeout(self.timeout))
    }

    /// The channel table row for an alias channel number.
    pub fn channel_row(&self, alias: u8) -> Option<S1gChannelRow> {
        self.channels.iter().copied().find(|r| r.alias == alias)
    }

    /// Run one `cli_app` command and return its combined stdout+stderr — **checking the `OK`/`FAIL`
    /// trailer, because the exit status is always 0**.
    ///
    /// This is the single point of contact with the vendor tool for every knob; a native
    /// `NRC-NL-FAM` netlink client would replace exactly this function (and, unlike `cli_app`,
    /// could address a specific radio).
    pub fn cli(&self, args: &[&str]) -> Result<String, FaceError> {
        let mut cmd = match self.timeout {
            Some(d) => {
                let mut c = Command::new("timeout");
                c.arg(d.as_secs().max(1).to_string());
                c.arg(&self.cli);
                c
            }
            None => Command::new(&self.cli),
        };
        let out = cmd.args(args).output().map_err(FaceError::Io)?;
        if out.status.code() == Some(TIMEOUT_EXIT_STATUS) && self.timeout.is_some() {
            return Err(FaceError::Io(std::io::Error::new(
                std::io::ErrorKind::TimedOut,
                format!(
                    "{}: cli_app {args:?} timed out — the tool has no timeout of its own and hangs \
                     when the firmware is down; check the radio before retrying",
                    self.iface
                ),
            )));
        }
        let text = format!(
            "{}{}",
            String::from_utf8_lossy(&out.stdout),
            String::from_utf8_lossy(&out.stderr)
        );
        match cli_status(&text) {
            CliStatus::Ok => Ok(text),
            CliStatus::Fail => Err(FaceError::Io(std::io::Error::other(format!(
                "{}: cli_app {args:?} reported FAIL: {}",
                self.iface,
                text.trim()
            )))),
            CliStatus::Unknown => Err(FaceError::Io(std::io::Error::new(
                std::io::ErrorKind::InvalidData,
                format!(
                    "{}: cli_app {args:?} produced no OK/FAIL trailer ({} bytes) — cli_app always \
                     exits 0, so the trailer is the only status there is",
                    self.iface,
                    text.len()
                ),
            ))),
        }
    }

    /// Read `show config`.
    pub fn config(&self) -> Result<NrcConfig, FaceError> {
        Ok(parse_config(&self.cli(&["show", "config"])?))
    }

    /// Read the current `set tx_time` state.
    pub fn tx_time(&self) -> Result<TxTime, FaceError> {
        let text = self.cli(&["show", "tx_time"])?;
        parse_tx_time(&text).ok_or_else(|| self.unparsed("show tx_time", &text))
    }

    /// **The honest airtime dial** — program `set tx_time {CS} {Pause}` and return what the radio
    /// reports afterwards.
    ///
    /// ★ MEASURED monotonic and exactly reversible over NRC↔NRC TCP goodput (receiver side,
    /// 3 reps/point):
    ///
    /// ```text
    /// CS=0    Pause=0     -> 6079 Kbit/s 100%   CS=2000 Pause=32000 -> 1728  28%
    /// CS=2000 Pause=2000  -> 4806         79%   CS=2000 Pause=48000 -> 1418  23%
    /// CS=2000 Pause=8000  -> 3699         61%   CS=2000 Pause=60000 -> 1155  19%
    /// CS=8000 Pause=8000  -> 2653         44%   CS=2000 Pause=65534 -> 1058  17% (floor)
    /// ```
    ///
    /// Three things that reading it as a duty cycle would get wrong:
    ///
    /// * It is **not** a duty cycle — the same ratio gives different results (compare 8000/2000
    ///   with 2000/8000).
    /// * CS is carrier-sense *overhead*: raising it **lowers** throughput. Source agrees
    ///   (`lmac_lbt_set_cs_time`).
    /// * It **cannot reach a hold**: the floor is 17 %. That is why this is
    ///   [`RadioKnobs::set_contention`]-class shaping and not [`RadioKnobs::set_tx_hold`].
    ///
    /// ⚠ **Out-of-range values are REJECTED, not clamped, and the previous setting stays in
    /// force** (measured). This method therefore refuses them here rather than sending a write that
    /// would leave the radio in a state the caller does not know about — and it re-reads afterwards
    /// so the return value is the radio's word, not ours.
    pub fn set_tx_time(&self, cs_us: u32, pause_us: u32) -> Result<TxTime, FaceError> {
        if cs_us > TX_TIME_CS_MAX_US {
            return Err(FaceError::Io(std::io::Error::new(
                std::io::ErrorKind::InvalidInput,
                format!(
                    "{}: CS time {cs_us} us exceeds {TX_TIME_CS_MAX_US} (wifi_api_set_tx_time \
                     rejects above this with -EINVAL and the PREVIOUS setting survives)",
                    self.iface
                ),
            )));
        }
        if pause_us > TX_TIME_PAUSE_MAX_US {
            return Err(FaceError::Io(std::io::Error::new(
                std::io::ErrorKind::InvalidInput,
                format!(
                    "{}: pause {pause_us} us exceeds {TX_TIME_PAUSE_MAX_US} (MEASURED: the host \
                     shell rejects it and the PREVIOUS setting survives)",
                    self.iface
                ),
            )));
        }
        let (cs, pause) = (cs_us.to_string(), pause_us.to_string());
        self.cli(&["set", "tx_time", &cs, &pause])?;
        let now = self.tx_time()?;
        if now.cs_us != cs_us || now.pause_us != pause_us {
            return Err(FaceError::Io(std::io::Error::other(format!(
                "{}: set tx_time {cs_us}/{pause_us} did not take — radio reports {}/{}",
                self.iface, now.cs_us, now.pause_us
            ))));
        }
        Ok(now)
    }

    /// Read the CCA threshold, dBm. `None` = the firmware answered `out-of-range`.
    pub fn cca_threshold(&self) -> Result<Option<i8>, FaceError> {
        let text = self.cli(&["show", "cca_thresh"])?;
        if text.contains("out-of-range") {
            return Ok(None);
        }
        parse_cca_thresh(&text)
            .map(Some)
            .ok_or_else(|| self.unparsed("show cca_thresh", &text))
    }

    /// Read all four access categories from `show edca`. **Reported, not programmed** — see
    /// [`EdcaAc`].
    pub fn edca(&self) -> Result<Vec<EdcaAc>, FaceError> {
        let text = self.cli(&["show", "edca"])?;
        let acs = parse_edca(&text);
        if acs.is_empty() {
            return Err(self.unparsed("show edca", &text));
        }
        Ok(acs)
    }

    /// Read `show mac tx stats`.
    ///
    /// ⚠ **Native on purpose.** This is MAC-level MPDU accounting (succeeded / retransmitted); it
    /// is *not* the `(tx_en, tx_on)` MAC→baseband / baseband→RF register pair
    /// [`RadioKnobs::read_tx_counters`] is specified as. Mapping one onto the other would preserve
    /// the shape and change the meaning — "we transmitted and got no ack" would be reported where
    /// the contract says "the baseband never keyed". So the trait method stays defaulted and this
    /// is the way to the numbers.
    ///
    /// ⚠ ≥ 300 ms per call (three netlink round-trips). `show mac tx clear` zeroes the counters.
    pub fn mac_tx_stats(&self) -> Result<MacTxStats, FaceError> {
        let text = self.cli(&["show", "mac", "tx", "stats"])?;
        parse_mac_tx_stats(&text).ok_or_else(|| self.unparsed("show mac tx stats", &text))
    }

    /// Read `show mac rx stats` — including the FCS-error count
    /// [`RadioKnobs::read_ofdm_counters`] reports. ⚠ ≥ 300 ms per call.
    pub fn mac_rx_stats(&self) -> Result<MacRxStats, FaceError> {
        let text = self.cli(&["show", "mac", "rx", "stats"])?;
        parse_mac_rx_stats(&text).ok_or_else(|| self.unparsed("show mac rx stats", &text))
    }

    /// Read `show stats simple_rx`.
    pub fn simple_rx_stats(&self) -> Result<SimpleRxStats, FaceError> {
        let text = self.cli(&["show", "stats", "simple_rx"])?;
        parse_simple_rx(&text).ok_or_else(|| self.unparsed("show stats simple_rx", &text))
    }

    /// Confirm the radio is actually on the channel we asked for.
    ///
    /// ★ **Not optional, and the reason is that `iw` will happily accept channels this radio cannot
    /// serve.** `nrc_channels_2ghz`/`nrc_channels_5ghz` register the *full* shadow lists while the
    /// board-data filter is applied only inside `nrc_mac_config` — which returns `-EINVAL`, except
    /// for a non-US regdomain at 2412 MHz, where `nrc-mac80211.c` **silently substitutes**
    /// `nons1g_ch_freq[0]`. `iw phy info` is therefore not a truthful enumerator either.
    ///
    /// The check compares `show config`'s `MAC80211_freq` against
    /// [`S1gChannelRow::shadow_mhz`] — the one field whose unit is source-grounded.
    pub fn verify_channel(&self, row: S1gChannelRow) -> Result<(), FaceError> {
        let cfg = self.config()?;
        match cfg.mac80211_freq_mhz {
            Some(f) if f == row.shadow_mhz => Ok(()),
            Some(f) => Err(FaceError::Io(std::io::Error::other(format!(
                "{}: channel {} ({} MHz shadow / {} kHz S1G) did not take — show config reports \
                 MAC80211_freq {f} MHz",
                self.iface,
                row.alias,
                row.shadow_mhz,
                row.s1g_khz()
            )))),
            None => Err(FaceError::Io(std::io::Error::new(
                std::io::ErrorKind::InvalidData,
                format!(
                    "{}: show config carried no MAC80211_freq, so the tune could not be verified",
                    self.iface
                ),
            ))),
        }
    }

    /// Error helper: a command that ran but whose output we could not read.
    fn unparsed(&self, what: &str, text: &str) -> FaceError {
        FaceError::Io(std::io::Error::new(
            std::io::ErrorKind::InvalidData,
            format!(
                "{}: could not parse `{what}` output ({} bytes)",
                self.iface,
                text.len()
            ),
        ))
    }
}

/// What a `cli_app` invocation reported. ★ The exit status is **always 0** — this trailer is the
/// only status the tool produces.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum CliStatus {
    Ok,
    Fail,
    /// Neither trailer present: a hang that was killed, a tool that is not `cli_app`, or output
    /// truncated. Never treat as success.
    Unknown,
}

/// Classify `cli_app` output by its trailing `OK` / `FAIL` token.
///
/// ⚠ **Token, not line.** `cli_app`'s `set txpwr` path prints with `display_per_line = 3`, whose
/// formatter emits *tabs* between pairs and never a final newline — so the trailer arrives glued to
/// the last value as `… : 20\t\t\tOK`. A line-oriented check misses it.
fn cli_status(text: &str) -> CliStatus {
    match text.split_whitespace().next_back() {
        Some("OK") => CliStatus::Ok,
        Some("FAIL") => CliStatus::Fail,
        _ => CliStatus::Unknown,
    }
}

/// Look up a `key<tabs> : value` pair in `cli_app` output and return the value's first
/// whitespace-delimited token.
///
/// Pass `key` **exactly as it appears in the vendor's key list**, leading space and all
/// (`" - aifsn"`, not `"- aifsn"`). Two properties of `cli_util.c`'s formatter this relies on:
/// the key is printed verbatim, and entries are separated by `\n` (one per line) or `\t`
/// (several per line). Requiring the key to start at one of those boundaries is what stops
/// `"Type"` from matching inside `"Tx Power Type"` — a real collision in `show config`.
fn cli_kv<'a>(text: &'a str, key: &str) -> Option<&'a str> {
    let bytes = text.as_bytes();
    let mut from = 0usize;
    while let Some(rel) = text[from..].find(key) {
        let at = from + rel;
        from = at + key.len();
        let boundary = at == 0 || matches!(bytes[at - 1], b'\n' | b'\t');
        if !boundary {
            continue;
        }
        let rest = &text[at + key.len()..];
        let rest = rest.trim_start_matches([' ', '\t']);
        let Some(rest) = rest.strip_prefix(':') else {
            continue;
        };
        return rest
            .trim_start_matches([' ', '\t'])
            .split_whitespace()
            .next();
    }
    None
}

/// `cli_kv` + integer parse, tolerating the `us` suffix the `set tx_time` key list formats with.
fn cli_kv_num<T: std::str::FromStr>(text: &str, key: &str) -> Option<T> {
    let v = cli_kv(text, key)?;
    v.strip_suffix("us").unwrap_or(v).parse().ok()
}

/// Parse `show tx_time`.
fn parse_tx_time(text: &str) -> Option<TxTime> {
    Some(TxTime {
        cs_us: cli_kv_num(text, "CS time")?,
        pause_us: cli_kv_num(text, "Pause time")?,
        resume_us: cli_kv_num(text, "Resume time")?,
    })
}

/// Parse the `Type` / `Tx power` pair `set txpwr` echoes.
fn parse_txpwr(text: &str) -> Option<TxPowerApplied> {
    Some(TxPowerApplied {
        kind: cli_kv(text, "Type")?.to_string(),
        dbm: cli_kv_num(text, "Tx power")?,
    })
}

/// Parse `show cca_thresh`, which prints the bare firmware response on its own line.
///
/// ⚠ The caller must check for the literal `out-of-range` first: `cmd_show_cca_thresh` renders a
/// firmware response of `-1` as that string, and `-1` is *also* a syntactically valid dBm reading.
fn parse_cca_thresh(text: &str) -> Option<i8> {
    for line in text.lines() {
        let l = line.trim();
        if l.is_empty() || l == "OK" || l == "FAIL" {
            continue;
        }
        if let Ok(v) = l.parse::<i8>() {
            return Some(v);
        }
    }
    None
}

/// One `show edca` block while it is still being filled — every field optional, because a block
/// that is missing one must be **dropped, not defaulted**: a fabricated `aifsn` would be spent by
/// an airtime budget as if it were real.
#[derive(Clone, Copy, Debug, Default)]
struct PartialEdca {
    ac: u8,
    aifsn: Option<u8>,
    cw_min: Option<u16>,
    cw_max: Option<u16>,
    txop_limit: Option<u16>,
    txop_max: Option<u16>,
}

impl PartialEdca {
    /// Complete blocks only.
    fn finish(self) -> Option<EdcaAc> {
        Some(EdcaAc {
            ac: self.ac,
            aifsn: self.aifsn?,
            cw_min: self.cw_min?,
            cw_max: self.cw_max?,
            txop_limit: self.txop_limit?,
            txop_max: self.txop_max?,
        })
    }
}

/// Parse `show edca` into its access categories.
///
/// The vendor formatter cycles one key list per AC, so the output is four blocks each opened by
/// `[AC]`. Anything before the first `[AC]` (the rule line) is ignored, and an incomplete block is
/// discarded — see [`PartialEdca`].
fn parse_edca(text: &str) -> Vec<EdcaAc> {
    let mut out = Vec::new();
    let mut cur: Option<PartialEdca> = None;
    for line in text.lines() {
        let Some((k, v)) = line.split_once(':') else {
            continue;
        };
        let (k, v) = (k.trim(), v.trim());
        let Some(v) = v.split_whitespace().next() else {
            continue;
        };
        match k {
            "[AC]" => {
                out.extend(cur.take().and_then(PartialEdca::finish));
                cur = v.parse::<u8>().ok().map(|ac| PartialEdca {
                    ac,
                    ..Default::default()
                });
            }
            "- aifsn" => {
                if let Some(c) = cur.as_mut() {
                    c.aifsn = v.parse().ok();
                }
            }
            "- cw min" => {
                if let Some(c) = cur.as_mut() {
                    c.cw_min = v.parse().ok();
                }
            }
            "- cw max" => {
                if let Some(c) = cur.as_mut() {
                    c.cw_max = v.parse().ok();
                }
            }
            "- txop limit" => {
                if let Some(c) = cur.as_mut() {
                    c.txop_limit = v.parse().ok();
                }
            }
            "- txop max" => {
                if let Some(c) = cur.as_mut() {
                    c.txop_max = v.parse().ok();
                }
            }
            _ => {}
        }
    }
    out.extend(cur.and_then(PartialEdca::finish));
    out
}

/// Pull one `name:value` field out of the `show mac … stats` header line.
///
/// ⚠ `"OK count:"` is a **substring of** `"NOK count:"`, so the OK field must be searched as
/// `"(OK count:"` — with the opening parenthesis the vendor's `printf` puts there.
fn mac_stats_field(text: &str, key: &str) -> Option<u32> {
    let at = text.find(key)? + key.len();
    let rest = text[at..].trim_start();
    let end = rest
        .find(|c: char| !c.is_ascii_digit())
        .unwrap_or(rest.len());
    rest[..end].parse().ok()
}

/// Parse the header line of `show mac tx stats`.
fn parse_mac_tx_stats(text: &str) -> Option<MacTxStats> {
    Some(MacTxStats {
        ok: mac_stats_field(text, "(OK count:")?,
        rtx: mac_stats_field(text, "RTX count:")?,
        last_mcs: mac_stats_field(text, "last MCS:")? as u8,
    })
}

/// Parse the header line of `show mac rx stats`.
fn parse_mac_rx_stats(text: &str) -> Option<MacRxStats> {
    Some(MacRxStats {
        ok: mac_stats_field(text, "(OK count:")?,
        nok: mac_stats_field(text, "NOK count:")?,
        last_mcs: mac_stats_field(text, "last MCS:")? as u8,
        fcs_error: mac_stats_field(text, "FCS error:")?,
    })
}

/// Parse `show stats simple_rx`.
fn parse_simple_rx(text: &str) -> Option<SimpleRxStats> {
    Some(SimpleRxStats {
        rssi_dbm: cli_kv_num(text, "RSSI")?,
        cs_cnt: cli_kv_num(text, "CS_Cnt")?,
        psdu_succ: cli_kv_num(text, "PSDU_Succ")?,
        mpdu_rcv: cli_kv_num(text, "MPDU_Rcv")?,
        mpdu_succ: cli_kv_num(text, "MPDU_Succ")?,
        snr: cli_kv_num(text, "SNR")?,
    })
}

/// Parse the fields of `show config` this crate uses. Missing fields stay `None` — the caller
/// decides whether an absent field is fatal (for `MAC80211_freq` it is).
fn parse_config(text: &str) -> NrcConfig {
    NrcConfig {
        device_mode: cli_kv(text, "Device Mode").map(str::to_string),
        country: cli_kv(text, "Country").map(str::to_string),
        bandwidth: cli_kv(text, "Bandwidth").map(str::to_string),
        frequency: cli_kv(text, "Frequency").map(str::to_string),
        mac80211_freq_mhz: cli_kv_num(text, "MAC80211_freq"),
        tx_power_type: cli_kv(text, "Tx Power Type").map(str::to_string),
    }
}

/// Convert a contention **window** (15, 1023, …) to the **exponent** `ContentionApplied` wants.
///
/// The NRC's WIM `cw_min`/`cw_max` are `uint16_t` fed straight from mac80211's
/// `ieee80211_tx_queue_params`, which carries windows, not exponents — so `show edca`'s `15` is
/// exponent 4. Values that are not `2^n − 1` are rounded up to the next window, which keeps the
/// conversion monotone; the HAL has no way to express "not a power of two".
pub fn cw_window_to_exponent(window: u16) -> u8 {
    (u32::from(window).saturating_add(1))
        .next_power_of_two()
        .trailing_zeros() as u8
}

/// The `set tx_time` point each posture programs.
///
/// ★ **This dial is ONE-DIRECTIONAL on this part, and the API cannot say so.** `CS=0 Pause=0` is
/// both the boot default and the MEASURED 100 % point (6079 Kbit/s); there is no position that
/// contends *harder* than default. So [`ContentionPosture::Owned`] and
/// [`ContentionPosture::Shared`] necessarily program the identical `0 0` — stated here rather than
/// manufacturing a difference between them. Only [`ContentionPosture::Yielding`] moves.
///
/// `Yielding` uses the measured 2000/8000 point (**61 %** of baseline goodput): enough to hand a
/// starving neighbour real airtime, well short of the 17 % floor. A caller that wants a specific
/// point on the ladder should use [`Nrc7292Knobs::set_tx_time`] directly.
fn posture_tx_time(posture: ContentionPosture) -> (u32, u32) {
    match posture {
        ContentionPosture::Owned | ContentionPosture::Shared => (0, 0),
        ContentionPosture::Yielding => (2_000, 8_000),
    }
}

impl RadioKnobs for Nrc7292Knobs {
    /// **The one power knob, routed to the absolute axis this part actually has.**
    ///
    /// ★ Wired so that `PowerRequest` means something on a HaLow radio too, rather than falling
    /// through to the trait's `Unsupported` default: `Dbm` goes straight to
    /// [`set_tx_power_dbm`](RadioKnobs::set_tx_power_dbm), and `Ceiling` means the top of the
    /// declared [`DbmRange`](ndn_radio_hal::DbmRange) — which on a part with a real dBm axis is the
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
                        "nrc7292: Ceiling requested but this radio declares no dBm range",
                    ));
                }
            },
            P::Index(i, _) => {
                return Err(ndn_radio_hal::power_unsupported(format!(
                    "nrc7292: PowerRequest::Index({i}) — this radio has an ABSOLUTE dBm axis and no \
                     index scale. Mapping an opaque index onto dBm would invent a number a link \
                     budget would believe. Use PowerRequest::Dbm.",
                )));
            }
            P::Raw { .. } => {
                return Err(ndn_radio_hal::power_unsupported(
                    "nrc7292: there is no raw chip axis behind this knob — the vendor path is the \
                     only one, and it is already dBm-denominated and regulatory-clamped.",
                ));
            }
            P::NoActuator => {
                return Err(ndn_radio_hal::power_unsupported(
                    "nrc7292: PowerRequest::NoActuator, but this radio DOES actuate power in dBm.",
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

    /// Tune by **alias channel number** (the 2.4/5 GHz shadow numbers `iw` and mac80211 speak), then
    /// verify.
    ///
    /// ★ **No `cli_app` command tunes this radio.** `cmd_set_s1g_freq` exists in `cli_cmd.c` but is
    /// absent from `set_sub_list`, so it is unreachable. The real path is nl80211:
    /// `nrc_mac_config` handles `IEEE80211_CONF_CHANGE_CHANNEL` and emits the WIM channel TLV, and
    /// `MONITOR` is in the driver's `interface_modes`. Unlike the Morse MM6108 (measured `-16
    /// EBUSY`), `iw` works here — MEASURED on the bench, where `iw dev halow0 set channel 161`
    /// lands on 925.0 MHz.
    ///
    /// ⚠ **The interface must already be in monitor mode**, and the whole down → `type monitor` →
    /// up → `reg set US` → channel sequence should be issued as one atomic command; a dropped
    /// session mid-sequence leaves the interface back in managed with RX dead.
    ///
    /// ★ **`bw` is validated, not programmed.** On S1G the width is a property of the channel
    /// number ([`S1gChannelRow::bw_mhz`]), and [`Bandwidth`] cannot express 1/2/4 MHz at all. So:
    /// [`Bandwidth::Bw20`] (the `Default`, i.e. "no preference") is accepted for any row;
    /// [`Bandwidth::Nb5`] is read as a request for 1 MHz and [`Bandwidth::Nb10`] as 2 MHz, honoured
    /// only if the row agrees; 40 and 80 MHz are refused outright, since no S1G channel is that
    /// wide and there is no width they could plausibly mean. The honest API for width is to pick
    /// the channel that has it.
    fn set_channel(&self, channel: u8, bw: Bandwidth) -> Result<(), FaceError> {
        let row = self.channel_row(channel).ok_or_else(|| {
            FaceError::Io(std::io::Error::new(
                std::io::ErrorKind::InvalidInput,
                format!(
                    "{}: alias channel {channel} is not in this radio's channel table \
                     ({} rows; US ships 1/2/4 MHz only, no 8 MHz)",
                    self.iface,
                    self.channels.len()
                ),
            ))
        })?;
        // Shared with the MM6108, deliberately: see `crate::halow::s1g_width_request` for the
        // reading and for why both HaLow radios must answer an inexpressible width the same way.
        let wanted_mhz = crate::halow::s1g_width_request(bw).map_err(|e| {
            FaceError::Io(std::io::Error::new(
                std::io::ErrorKind::InvalidInput,
                format!(
                    "{}: alias channel {channel} ({} MHz): {e}",
                    self.iface, row.bw_mhz
                ),
            ))
        })?;
        if let Some(w) = wanted_mhz
            && w != row.bw_mhz
        {
            return Err(FaceError::Io(std::io::Error::new(
                std::io::ErrorKind::InvalidInput,
                format!(
                    "{}: alias channel {channel} is {} MHz wide, not {w} MHz — on S1G the width is \
                     a property of the channel, so pick the channel that has the width",
                    self.iface, row.bw_mhz
                ),
            )));
        }

        let ch = channel.to_string();
        let out = Command::new(&self.iw)
            .args(["dev", &self.iface, "set", "channel", &ch])
            .output()
            .map_err(FaceError::Io)?;
        if !out.status.success() {
            return Err(FaceError::Io(std::io::Error::other(format!(
                "{}: iw set channel {channel} failed: {}{}",
                self.iface,
                String::from_utf8_lossy(&out.stdout).trim(),
                String::from_utf8_lossy(&out.stderr).trim()
            ))));
        }
        self.verify_channel(row)
    }

    /// **The genuine dBm axis** — `set txpwr fixed <dBm>` — returning the value the firmware echoed.
    ///
    /// The vendor API documents the range as 1..30 dBm and the types as Auto(0)/Limit(1)/Fixed(2),
    /// and states it must be invoked *after* the country code is set. Out-of-range requests are
    /// refused here rather than clamped: the capability already advertises the span, so a caller
    /// that exceeds it has a bug worth hearing about.
    ///
    /// ⚠ **UNVERIFIED whether the echo is the requested or the clamped value** — see
    /// [`TxPowerApplied::dbm`]. Nothing has metered this part, which is also why
    /// `db_per_power_idx` stays `None`: there is no measured dB-per-step to convert with.
    fn set_tx_power_dbm(&self, dbm: i8) -> Result<i8, FaceError> {
        if !(TXPWR_DBM_MIN..=TXPWR_DBM_MAX).contains(&dbm) {
            return Err(FaceError::Io(std::io::Error::new(
                std::io::ErrorKind::InvalidInput,
                format!(
                    "{}: {dbm} dBm outside the vendor range {TXPWR_DBM_MIN}..={TXPWR_DBM_MAX}",
                    self.iface
                ),
            )));
        }
        let v = dbm.to_string();
        let text = self.cli(&["set", "txpwr", "fixed", &v])?;
        let applied = parse_txpwr(&text).ok_or_else(|| self.unparsed("set txpwr", &text))?;
        Ok(applied.dbm)
    }

    /// **Airtime shaping through `set tx_time`** — the only contention-class actuator this part has.
    ///
    /// See [`Nrc7292Knobs::set_tx_time`] for the measured ladder and
    /// `posture_tx_time` (private, just below) for why `Owned` and `Shared` are the same point.
    ///
    /// ⚠ **The returned `ContentionApplied` describes fields this call did not program.** `set
    /// tx_time` lengthens the LMAC's listen-before-talk window; it does not touch cw/aifs/txop. The
    /// window values are *read* from `show edca` (Best Effort — see [`EDCA_AC_BE`]) so the caller
    /// gets the radio's real numbers rather than invented ones, and `slot_us` is the 802.11ah
    /// standard value because this part has no slot reader at all ([`S1G_SLOT_US`]). Every airtime
    /// figure derived from the return therefore carries that caveat, and
    /// [`ContentionApplied::medium_access_us`] additionally adds the HAL's OFDM `SIFS_US` of 16 µs
    /// where S1G's SIFS is far longer.
    ///
    /// If `show edca` cannot be read, this returns an error **after** the `tx_time` write has
    /// already taken — read [`Nrc7292Knobs::tx_time`] to see what is in force.
    fn set_contention(&self, posture: ContentionPosture) -> Result<ContentionApplied, FaceError> {
        let (cs, pause) = posture_tx_time(posture);
        self.set_tx_time(cs, pause)?;
        let acs = self.edca()?;
        let be = acs
            .iter()
            .find(|a| a.ac == EDCA_AC_BE)
            .or_else(|| acs.first())
            .copied()
            .ok_or_else(|| {
                FaceError::Io(std::io::Error::new(
                    std::io::ErrorKind::InvalidData,
                    format!("{}: show edca reported no access categories", self.iface),
                ))
            })?;
        let cw_min = cw_window_to_exponent(be.cw_min);
        Ok(ContentionApplied {
            cw_min,
            cw_max: cw_window_to_exponent(be.cw_max),
            aifs: be.aifsn,
            txop: be.txop_limit,
            slot_us: S1G_SLOT_US,
            avg_backoff_us: ContentionApplied::avg_backoff_us_at(cw_min, S1G_SLOT_US),
        })
    }

    /// **A real dBm defer threshold** — `set cca_thresh`, range −100..−35, with a read-back.
    ///
    /// ⚠ **This part has ONE threshold and the HAL passes a hysteresis PAIR.** `l2h` (the busy
    /// threshold) is programmed; **`h2l` has no actuator here and is dropped**, and the signature
    /// has no way to report a partially-honoured call. Callers doing spatial reuse should treat
    /// this as "the defer floor" and pair it with [`set_tx_power_dbm`](Self::set_tx_power_dbm).
    ///
    /// The firmware renders an out-of-range response as the literal string `out-of-range`, which is
    /// propagated as an error rather than swallowed — and the write is read back, because a
    /// threshold that did not take is indistinguishable from one that did until you look.
    ///
    /// ⚠ Prior art on what this knob *achieves*: on the RTL8812AU, EDCCA works and, on a saturated
    /// channel, trades collision loss for TX starvation (237 → 26 delivered/s). It has not been
    /// A/B'd on air on this part.
    fn set_edcca_threshold_dbm(&self, l2h: i8, h2l: i8) -> Result<(), FaceError> {
        if !(CCA_THRESH_MIN_DBM..=CCA_THRESH_MAX_DBM).contains(&l2h) {
            return Err(FaceError::Io(std::io::Error::new(
                std::io::ErrorKind::InvalidInput,
                format!(
                    "{}: CCA threshold {l2h} dBm outside \
                     {CCA_THRESH_MIN_DBM}..={CCA_THRESH_MAX_DBM}",
                    self.iface
                ),
            )));
        }
        let _ = h2l; // No second threshold on this part; see the doc comment.
        let v = l2h.to_string();
        self.cli(&["set", "cca_thresh", &v])?;
        match self.cca_threshold()? {
            Some(now) if now == l2h => Ok(()),
            Some(now) => Err(FaceError::Io(std::io::Error::other(format!(
                "{}: set cca_thresh {l2h} did not take — radio reports {now} dBm",
                self.iface
            )))),
            None => Err(FaceError::Io(std::io::Error::other(format!(
                "{}: firmware reported the CCA threshold out-of-range after setting {l2h} dBm",
                self.iface
            )))),
        }
    }

    /// **There is no CCA/LBT bypass on this radio**, so this refuses instead of succeeding quietly.
    ///
    /// ★ The HAL's default for this method is `Ok(())` — a silent success. Leaving it in place here
    /// would tell a caller that CSMA had been suppressed when the LMAC's listen-before-talk is
    /// still running (it is the very thing `set tx_time`'s CS field *lengthens*). There is no
    /// `force_rx_clear` equivalent anywhere in the `cli_app` surface.
    ///
    /// The nearest real lever is pushing the CCA threshold to its least-sensitive end
    /// ([`CCA_THRESH_MAX_DBM`]) with [`set_edcca_threshold_dbm`](Self::set_edcca_threshold_dbm) —
    /// but that is a different knob with different state, and a caller should have to ask for it by
    /// name rather than receive it as a side effect.
    fn set_edcca_ignore(&self, on: bool) -> Result<(), FaceError> {
        if on {
            return Err(FaceError::Io(std::io::Error::new(
                std::io::ErrorKind::Unsupported,
                "NRC7292 exposes no CCA/LBT bypass; raise set_edcca_threshold_dbm instead",
            )));
        }
        Ok(()) // Nothing was ever suppressed, so there is nothing to undo.
    }

    /// `(ok, err)` from `show mac rx stats` = **(OK count, FCS error)**.
    ///
    /// ★ The cleanest mapping in this backend. The HAL defines `err` as "PPDUs the PHY BEGAN to
    /// demodulate and failed", and the NRC's RX header carries a dedicated FCS-error count that is
    /// literally that. Truncated to `u16` so differences stay exact modulo 2^16, matching the HAL's
    /// free-running/wrapping contract.
    ///
    /// ⚠ **≥ 300 ms per call** (three netlink round-trips with 100 ms between them, plus a process
    /// spawn). Correct for an occasional read, wrong inside a per-second sampler.
    fn read_ofdm_counters(&self) -> Result<Option<(u16, u16)>, FaceError> {
        let s = self.mac_rx_stats()?;
        Ok(Some((s.ok as u16, s.fcs_error as u16)))
    }

    /// `CS_Cnt` from `show stats simple_rx` — a carrier-sense count the firmware maintains without
    /// the host decoding frames, the same shape as the 8812au's `REG_RXERR_RPT`.
    ///
    /// ⚠ **UNVERIFIED that it is free-running**, which is the contract this method is built on
    /// (two reads, differenced). If the firmware resets it per read or per window, the difference
    /// is meaningless. Settle it before trusting the rate: two reads on a quiet channel and two on
    /// a loaded one, checking that it only ever increases and that the rate tracks offered load.
    /// Its exact width is unverified too; it is truncated (wrapping) to `u16` on the assumption
    /// that it is a count.
    ///
    /// **A much better instrument exists and is deliberately not wired here**: `show
    /// optimal_channel {CC} {BW} {dwell}` returns a real per-channel **CCA busy percentage** from a
    /// firmware band scan. It blocks for roughly `channels × dwell` and moves the radio off
    /// channel, so it belongs behind a native method feeding the co-band occupancy map, never
    /// inside a periodic sampler.
    fn read_channel_activity(&self) -> Result<Option<u16>, FaceError> {
        Ok(Some(self.simple_rx_stats()?.cs_cnt as u16))
    }

    /// [`TxDiscipline::BestEffort`], stated rather than left silent.
    ///
    /// Three reasons this part cannot promise better, and they are the work items for earning
    /// `PromptBounded` later: no release-jitter measurement exists on this bearer; the MAC runs LBT
    /// that nothing here can bypass (see [`set_edcca_ignore`](Self::set_edcca_ignore)); and the
    /// only airtime dial makes the delay *worse*, not bounded. The precedent for not guessing is
    /// the AR9271, where a fabricated `ScheduledAt { 1 µs }` actively broke the scheduler.
    fn tx_discipline(&self) -> TxDiscipline {
        TxDiscipline::BestEffort
    }

    // ---- Left at the trait default, deliberately. Each of these has been looked for. ----
    //
    // `set_tx_power` (index scale): no index actuator exists. `show autotxgain` READS a per-MCS
    //   TX-power index table (MCS 0-7 plus MCS 10) and there is no `set` for it; `cli_app`
    //   registers no index setter. The dBm path is the real one, and `tx_power_dbm: Some(..)` in
    //   the capability is the HAL's declared signal to use it.
    //
    // `set_tx_hold`: no MAC-level TX gate. MEASURED, `set tx_time` bottoms out at 17 % of
    //   throughput (CS=2000/Pause=65534 -> 1058 Kbit/s), so it shapes, it cannot hold. The only
    //   other candidate, `set duty {on|off} {window} {duration}`, is MEASURED DEAD — it silently
    //   refuses, regdomain-gated — despite existing in the SDK as `nrc_wifi_enable_duty_cycle`.
    //   This is the one method whose default no-op `Ok(())` is genuinely correct here: the
    //   scheduler's software wait is the right fallback. Said out loud so the next reader does not
    //   mistake the silence for an unexplored seam.
    //
    // `read_tx_counters`: the mechanism exists but the SEMANTICS DIVERGE. The HAL's pair is
    //   (MAC->baseband requests, baseband->RF keys) — a register pair straddling the MAC/PHY
    //   boundary that separates "we never asked" from "we asked and the air ate it". The NRC has no
    //   such pair; `show mac tx stats` is MPDU accounting (OK / retransmissions), where a
    //   divergence would mean "we transmitted and got no ack". Mapping (OK+RTX, OK) would keep the
    //   shape and change the meaning, silently. The numbers are reachable through
    //   `Nrc7292Knobs::mac_tx_stats` instead.
    //
    // `set_tx_csd`: single chain, no cyclic-shift-diversity control anywhere in the surface.
    //
    // `set_phy`: S1G only. No runtime modulation command exists, and `PhyMode` has no S1G/OFDM
    //   member to name even if one did — hence `phy_modes: PhyModeSet::empty()` in the capability
    //   ("I cannot say"), never a set of one.
    //
    // `set_hop_plan`: no autonomous frequency-hopping sequencer. `set bgscan_trx` / `set
    //   scan_period` drive a background SCAN, which is not a hop plan and must not be mapped to
    //   one.
    //
    // `set_rx_gain`: no receive-gain actuator. `show config` REPORTS `Base RX_Gain` and
    //   `Compensated RX_Gain`; there is no `set` for either. The HAL's own doc for `RxGain::Reduced`
    //   says a radio with a genuinely dBm-denominated defer threshold should use
    //   `set_edcca_threshold_dbm` instead — which is exactly this part. Faking a posture through
    //   the CCA knob would be a different knob wearing this one's name.
    //
    // `set_spreading_factor` / `set_coding_rate` / `set_bandwidth_khz`: LoRa concepts. S1G width is
    //   carried by the channel number here, not by a kHz dial.
    //
    // `configure_name_filter`: no on-device name/prefix filtering. `set drop [vif] [mac] {on|off}`
    //   is a per-MAC-address blacklist, not a prefix-set mask match, and cannot express the 16-byte
    //   Bloom masks. The host filters in software.
    //
    // Also looked for and NOT wired, because nothing establishes what it does to the DATA rate:
    //   pinning the transmit MCS. `ieee80211_hw_set(hw, HAS_RATE_CONTROL)` means firmware owns the
    //   rate and the S1G injection radiotap names no MCS by design. The candidate is `set rc off`
    //   followed by `test mcs <n>` (a firmware *test*-namespace shell command); until someone
    //   measures that it pins the data rate rather than entering a test mode, it stays unwritten.
}

impl RadioProfile for Nrc7292Knobs {
    /// This radio's capability, built from [`RadioCapability::wifi_halow_s1g`] with three
    /// corrections the preset gets wrong for this part.
    ///
    /// ★★ **`max_mcs` is 7, not the preset's 10.** S1G MCS10 is a **1 MHz-only, repetition-coded
    /// BPSK** mode: it is the most *robust* rate and slower than MCS0, i.e. it sits **below** the
    /// ladder rather than above it, so 10 would not be a maximum rate.
    /// `show autotxgain`'s key list enumerates "MCS 0".."MCS 7" and then a separate "MCS 10",
    /// confirming it is off-ladder. Reaching MCS10 needs a native path (`test mcs 10` /
    /// `set bcn_mcs 10`), not this field.
    ///
    /// ⚠ An earlier version of this note blamed `RadioCapability::mcs_for_rssi`; that was wrong —
    /// it is `mcs_for_rssi(rssi).min(max_mcs)` and the free function already caps at
    /// `MAX_RELIABLE_MCS = 7`. The consumers that read `max_mcs` unclamped are the contextual
    /// bandit's `clamp(0, max_mcs)` and `RadioCapability::rate_rank`. See [`crate::halow`]'s
    /// `halow_base` for the full account.
    ///
    /// ★ **`kind` is `WifiHaLow`.** The preset sets `WifiMonitor`, contradicting its own name and
    /// the enum's dedicated variant, so nothing downstream could tell a HaLow radio from a 5 GHz
    /// one.
    ///
    /// ★ **`tx_power_dbm` is populated** (1..30, from `nrc_wifi_set_tx_power`) — the declaration
    /// that pushes cognition onto the real axis. `max_tx_power` is left at the preset's 63 but the
    /// index scale is **inert** on this part: `show autotxgain` is read-only and there is no index
    /// setter, so `set_tx_power` is Unsupported and `tx_power_dbm: Some(..)` is the signal to use
    /// instead. `min_tx_power` and `db_per_power_idx` stay `None` because nothing has metered this
    /// radio.
    ///
    /// Left as the preset has them, with reasons: `duty_cycle_max: 1.0` (802.11ah is CSMA/LBT, and
    /// `set duty` is measured dead anyway, so no actuator could enforce a fraction);
    /// `retune_us: None` (unmeasured — and note the real cost is the whole `iw` + read-back
    /// sequence, not just the `iw` call); `csi: None` (the per-frame SNR this radio reports belongs
    /// in `CapturedFrame.phy`, not in `CsiSupport::Coarse`, which means a phystatus channel
    /// estimate this part does not export); `max_payload: 1500` (**UNVERIFIED** for S1G here —
    /// `show uinfo` reports a per-peer `max mpdu_len` that could settle it).
    ///
    /// ★ **Identity caveat.** This describes a *radio*, while every actuator behind it addresses a
    /// *host*: `cli_app` has no interface selector. On a two-NRC node the capability is per-radio
    /// and the knobs are device-blind, and the HAL has no field in which to declare that.
    fn capability(&self) -> RadioCapability {
        // ★ ONE source for the parts both halves of this backend agree on. The data plane
        // ([`crate::halow::Nrc7292FrameIo`]) answers `capability()` too, and it used to disagree
        // with this one field-by-field — same radio, two different `kind`/`rate`/`max_payload`
        // stories depending on which object a caller happened to hold. Both now build from
        // [`crate::halow::nrc7292_capability`]; what follows is the explicit, documented delta this
        // object is entitled to add, which is exactly the power axis it can actuate.
        let mut cap =
            crate::halow::nrc7292_capability(self.channels.iter().map(|r| r.alias).collect());
        // ★ The genuine dBm axis — no Wi-Fi part in this fleet has another. The shared builder
        // leaves it `None`/`false` because a *monitor FrameIo* has no `cli_app` behind it and
        // therefore cannot drive power; this object does, through `set txpwr fixed`, and reads the
        // firmware's echo back. A deployment holding both should hand this capability to
        // `Nrc7292FrameIo::with_capability` so the face sees the axis too.
        cap.tx_power_dbm = Some(DbmRange::new(TXPWR_DBM_MIN, TXPWR_DBM_MAX));
        cap.power_actuated = true;
        cap
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Verbatim `cli_app read 0x00060a74 4` output, captured from the radio with `cat -A` so the
    /// exact bytes are known: LF-terminated (no CR), a banner, a *second* rule line after the
    /// value, then the `OK` trailer. The `Address :  Value` header is itself colon-separated, so a
    /// naive `split(':')` parse would try to read it as a word.
    const SAMPLE: &str = "\
-------------------
Address :  Value
-------------------
00060a74: 6609e6f9
-------------------
OK
";

    #[test]
    fn parses_the_requested_word() {
        assert_eq!(parse_cli_word(SAMPLE, TSF_MIRROR_ADDR), Some(0x6609_e6f9));
    }

    /// The trailing `OK` must never be mistaken for a value — the bug a "last line" parse hits.
    #[test]
    fn ok_trailer_is_not_a_value() {
        assert_eq!(parse_cli_word("OK\n", TSF_MIRROR_ADDR), None);
    }

    /// A block read returns many words; pick the requested one, not the first or last.
    #[test]
    fn picks_the_right_word_from_a_block() {
        let block = "\
00060a70: deadbeef
00060a74: 00c0ffee
00060a78: 12345678
OK
";
        assert_eq!(parse_cli_word(block, TSF_MIRROR_ADDR), Some(0x00c0_ffee));
        assert_eq!(parse_cli_word(block, 0x0006_0a70), Some(0xdead_beef));
    }

    /// An address that simply is not in the output is a miss, not a wrong answer.
    #[test]
    fn absent_address_is_none() {
        assert_eq!(parse_cli_word(SAMPLE, 0x0000_1234), None);
    }

    /// Two consecutive S1G beacons captured off-air from mds-o5p-3, with the radiotap TSFT
    /// mds-o5p-0 stamped them with. The two offsets must come out **identical** — that equality is
    /// the whole point of a common view, and it is what the bench measured.
    #[test]
    fn real_beacons_yield_a_stable_offset() {
        // 1c0b 0000 00c0cab465e2 acf06783 ...  @ tsft 1_308_937_865
        let a = [
            0x1c, 0x0b, 0x00, 0x00, 0x00, 0xc0, 0xca, 0xb4, 0x65, 0xe2, 0xac, 0xf0, 0x67, 0x83,
        ];
        // 1c0b 0000 00c0cab465e2 c9806983 ...  @ tsft 1_309_040_294
        let b = [
            0x1c, 0x0b, 0x00, 0x00, 0x00, 0xc0, 0xca, 0xb4, 0x65, 0xe2, 0xc9, 0x80, 0x69, 0x83,
        ];
        let oa = s1g_beacon_offset(&a, 1_308_937_865).expect("frame a is an S1G beacon");
        let ob = s1g_beacon_offset(&b, 1_309_040_294).expect("frame b is an S1G beacon");

        assert_eq!(oa.sa, [0x00, 0xc0, 0xca, 0xb4, 0x65, 0xe2]);
        assert_eq!(oa.fc1, 0x0b);
        assert_eq!(oa.peer_tsf_us, 0x8367_f0ac);
        assert_eq!(ob.peer_tsf_us, 0x8369_80c9);

        assert_eq!(oa.offset_us, 895_689_251);
        assert_eq!(
            ob.offset_us, 895_689_251,
            "offset must be stable across beacons"
        );

        // Both the peer's clock and ours advanced by one beacon interval between the two frames,
        // which is why the offset is unchanged.
        assert_eq!(ob.peer_tsf_us - oa.peer_tsf_us, 102_429);
        assert_eq!(1_309_040_294u64 - 1_308_937_865, 102_429);
    }

    /// Non-beacon frames must be rejected rather than parsed at a fixed offset — the trap that
    /// produced a 170x-too-large jitter figure on the bench.
    #[test]
    fn non_s1g_beacon_is_rejected() {
        let data_frame = [0x08u8, 0x00, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0];
        assert!(s1g_beacon_offset(&data_frame, 1_000_000).is_none());
        // Too short to contain the TSF field.
        assert!(s1g_beacon_offset(&[0x1c, 0x0b, 0x00], 1_000_000).is_none());
    }
}

/// Tests for the knob layer.
///
/// ⚠ **PROVENANCE.** No hardware was touched for this change, so these fixtures are **not** captures
/// off a radio — they are reconstructed byte-for-byte from the vendor's own formatter: the key
/// strings come from `cli_key_list.h` (`SHOW_CONFIG_KEY_LIST`, `SHOW_EDCA_KEY_LIST`,
/// `SHOW_STATS_SIMPLE_RX_KEY_LIST`, `SHOW_TX_TIME_KEY_LIST`, `SET_TXPWR_KEY_LIST`), and the tab and
/// rule layout is `cli_util.c`'s `print_merged_result` / `add_print_tab` / `print_line` run by hand
/// with the same `display_per_line` each command passes (1 everywhere except `set txpwr`, which
/// passes 3 — which is why that one has no newlines). The *values* are plausible, the *shape* is
/// derived. Replace each with a real `cat -A` capture the first time one is taken; the shape is what
/// the parsers depend on and the shape is what has been reproduced.
#[cfg(test)]
mod knob_tests {
    use super::*;
    // Read here rather than at module scope: `capability()` now delegates the shape of these two
    // fields to `crate::halow`, so only the assertions still name them.
    use ndn_radio_hal::{RadioKind, RateCapability};

    /// `show config`, `display_per_line = 1`, bracketed by `print_line('-', 52)`.
    /// Note the two collisions this fixture deliberately contains: `Type` also occurs inside
    /// `Tx Power Type`, and `MAC Address`'s value is itself full of colons.
    const SHOW_CONFIG: &str = "\
---------------------------------------------------
Device Mode\t\t\t : MONITOR
MAC Address\t\t\t : 00:c0:ca:b4:65:e2
Country\t\t\t\t : US
Bandwidth\t\t\t : 2
Frequency\t\t\t : 9250
MAC80211_freq\t\t\t : 5805
Default MCS\t\t\t : 7
Rate Control\t\t\t : on
 - Info\t\t\t\t : -
 - MCS10(MGMT)\t\t\t : off
Guard Interval\t\t\t : long
Security\t\t\t : none
Type\t\t\t\t : -
RTS\t\t\t\t : off
RTS threshold\t\t\t : 0
Format\t\t\t\t : S1G_SHORT
Preamble type\t\t\t : -
Promiscuous Mode\t\t : on
color\t\t\t\t : 0
Auto CFO Cal\t\t\t : on
BSSID\t\t\t\t : 00:00:00:00:00:00
AID\t\t\t\t : 0

[PHY Configuration]
TX_Gain\t\t\t\t : 0x30
Base RX_Gain\t\t\t : 0
Compensated RX_Gain\t\t : 0
Tx Power Type\t\t\t : fixed
---------------------------------------------------
OK
";

    /// `set txpwr fixed 20`. ★ `display_per_line = 3` means the formatter separates pairs with
    /// **tabs and never emits a final newline**, so the `OK` trailer arrives glued to the last
    /// value. This one fixture is the whole reason `cli_status` works on tokens, not lines.
    const SET_TXPWR: &str = "Type\t : fixed\tTx power : 20\t\t\tOK\n";

    /// `show tx_time` (the firmware's `test tx_time show`). No rule lines on this command.
    const SHOW_TX_TIME: &str = "\
CS time\t\t\t\t : 2000
Pause time\t\t\t : 8000
Resume time\t\t\t : 0
OK
";

    /// `show stats simple_rx`.
    const SHOW_SIMPLE_RX: &str = "\
---------------------------------------------------
RSSI\t\t\t\t : -62
CS_Cnt\t\t\t\t : 148392
PSDU_Succ\t\t\t : 10241
MPDU_Rcv\t\t\t : 10250
MPDU_Succ\t\t\t : 10203
SNR\t\t\t\t : 23
---------------------------------------------------
OK
";

    /// `show edca` — the key list cycles once per access category, with a blank line between
    /// blocks (`cmd_result_parse` prints one when it wraps back to the first key).
    const SHOW_EDCA: &str = "\
---------------------------------------------------
[AC]\t\t\t\t : 0
 - priority\t\t\t : 1
 - aggregation\t\t\t : 0
 - max agg num\t\t\t : 0
 - aifsn\t\t\t : 7
 - cw min\t\t\t : 15
 - cw max\t\t\t : 1023
 - txop limit\t\t\t : 0
 - txop max\t\t\t : 0

[AC]\t\t\t\t : 1
 - priority\t\t\t : 2
 - aggregation\t\t\t : 1
 - max agg num\t\t\t : 8
 - aifsn\t\t\t : 3
 - cw min\t\t\t : 15
 - cw max\t\t\t : 1023
 - txop limit\t\t\t : 0
 - txop max\t\t\t : 0

[AC]\t\t\t\t : 2
 - priority\t\t\t : 4
 - aggregation\t\t\t : 1
 - max agg num\t\t\t : 8
 - aifsn\t\t\t : 2
 - cw min\t\t\t : 7
 - cw max\t\t\t : 15
 - txop limit\t\t\t : 94
 - txop max\t\t\t : 94

[AC]\t\t\t\t : 3
 - priority\t\t\t : 6
 - aggregation\t\t\t : 1
 - max agg num\t\t\t : 8
 - aifsn\t\t\t : 2
 - cw min\t\t\t : 3
 - cw max\t\t\t : 7
 - txop limit\t\t\t : 47
 - txop max\t\t\t : 47
---------------------------------------------------
OK
";

    /// `show mac rx stats`, type 0 (the header the counters live in) plus one AC line.
    /// ★ The AC line contains `OK(` and `NOK(`, and the header's `OK count:` is a **substring of**
    /// its `NOK count:` — both traps a naive substring search falls into.
    const SHOW_MAC_RX: &str = "\
-----------------------------------------------------------------------------------------
 MAC RX Statistics (OK count:10250, NOK count:37, last MCS:7, FCS error:112)
-----------------------------------------------------------------------------------------
- AC[0]\t: OK(     10250/    148392)  NOK(        37/       912)
-----------------------------------------------------------------------------------------
OK
";

    /// `show mac tx stats`, type 0. The TX direction prints three fields, not four — there is no
    /// FCS-error count on this side.
    const SHOW_MAC_TX: &str = "\
-----------------------------------------------------------------------------------------
 MAC TX Statistics (OK count:9987, RTX count:412, last MCS:5)
-----------------------------------------------------------------------------------------
- AC[0]\t: OK(      9987/     92110)  RTX(       412/      4102)
-----------------------------------------------------------------------------------------
OK
";

    // ---- the OK/FAIL trailer, which is the ONLY status cli_app produces ----

    #[test]
    fn ok_trailer_is_the_only_status() {
        assert_eq!(cli_status(SHOW_CONFIG), CliStatus::Ok);
        assert_eq!(cli_status(SHOW_TX_TIME), CliStatus::Ok);
        assert_eq!(
            cli_status("usage : set txpwr {auto|limit|fixed} {value}\nFAIL\n"),
            CliStatus::Fail
        );
    }

    /// ★ The trailer is a TOKEN, not a line: `set txpwr` glues it to the last value with tabs.
    /// A line-oriented check reads this output as "no status" and a knob built on it either fails
    /// every call or, worse, treats the absence as success.
    #[test]
    fn ok_trailer_survives_the_tab_packed_set_txpwr_layout() {
        assert!(
            !SET_TXPWR.contains("\nOK"),
            "fixture must have OK glued to the value"
        );
        assert_eq!(cli_status(SET_TXPWR), CliStatus::Ok);
    }

    /// Empty output — a killed hang — must never read as success.
    #[test]
    fn no_trailer_is_not_success() {
        assert_eq!(cli_status(""), CliStatus::Unknown);
        assert_eq!(
            cli_status("Address :  Value\n00060a74: 6609e6f9\n"),
            CliStatus::Unknown
        );
    }

    // ---- key/value lookup ----

    /// ★ The collision that forces the boundary rule: `show config` carries both a `Type` key and a
    /// `Tx Power Type` key, and a plain substring search finds the wrong one first is not the
    /// problem — the problem is that a search for `Type` must NOT match inside `Tx Power Type`.
    #[test]
    fn key_lookup_respects_entry_boundaries() {
        assert_eq!(cli_kv(SHOW_CONFIG, "Type"), Some("-"));
        assert_eq!(cli_kv(SHOW_CONFIG, "Tx Power Type"), Some("fixed"));
        // "Frequency" must not be answered by "MAC80211_freq" or vice versa.
        assert_eq!(cli_kv(SHOW_CONFIG, "Frequency"), Some("9250"));
        assert_eq!(cli_kv(SHOW_CONFIG, "MAC80211_freq"), Some("5805"));
    }

    /// A value full of colons must not confuse the split.
    #[test]
    fn colon_rich_values_parse() {
        assert_eq!(
            cli_kv(SHOW_CONFIG, "MAC Address"),
            Some("00:c0:ca:b4:65:e2")
        );
    }

    #[test]
    fn absent_key_is_none() {
        assert_eq!(cli_kv(SHOW_CONFIG, "Duty Cycle"), None);
    }

    // ---- show config ----

    #[test]
    fn parses_show_config() {
        let c = parse_config(SHOW_CONFIG);
        assert_eq!(c.device_mode.as_deref(), Some("MONITOR"));
        assert_eq!(c.country.as_deref(), Some("US"));
        assert_eq!(c.bandwidth.as_deref(), Some("2"));
        // Kept as a string on purpose: the unit is UNVERIFIED (9250 would be 925.0 MHz if the
        // firmware prints the vendor tables' 100 kHz unit, but nothing proves that).
        assert_eq!(c.frequency.as_deref(), Some("9250"));
        // The one field with a grounded unit — the shadow frequency, MHz.
        assert_eq!(c.mac80211_freq_mhz, Some(5805));
        assert_eq!(c.tx_power_type.as_deref(), Some("fixed"));
    }

    /// 5805 MHz shadow is alias 161, which the vendor tables say is 925.0 MHz at 2 MHz. That
    /// equality is exactly what `verify_channel` checks, so pin it.
    #[test]
    fn show_config_shadow_matches_the_channel_table() {
        let row = US_S1G_CHANNELS.iter().find(|r| r.alias == 161).unwrap();
        assert_eq!(
            parse_config(SHOW_CONFIG).mac80211_freq_mhz,
            Some(row.shadow_mhz)
        );
        assert_eq!(row.s1g_khz(), 925_000);
        assert_eq!(row.bw_mhz, 2);
    }

    // ---- set txpwr ----

    #[test]
    fn parses_the_txpwr_echo() {
        assert_eq!(
            parse_txpwr(SET_TXPWR),
            Some(TxPowerApplied {
                kind: "fixed".into(),
                dbm: 20
            })
        );
    }

    // ---- tx_time ----

    #[test]
    fn parses_show_tx_time() {
        assert_eq!(
            parse_tx_time(SHOW_TX_TIME),
            Some(TxTime {
                cs_us: 2_000,
                pause_us: 8_000,
                resume_us: 0
            })
        );
    }

    /// The `set tx_time` key list formats with a `us` suffix (`SET_TXTIME_KEY_DISP`), so the number
    /// parse must tolerate it wherever it turns up.
    #[test]
    fn tx_time_tolerates_the_us_suffix() {
        let with_units =
            "CS time\t\t\t\t : 2000us\nPause time\t\t\t : 8000us\nResume time\t\t\t : 0us\nOK\n";
        assert_eq!(
            parse_tx_time(with_units),
            Some(TxTime {
                cs_us: 2_000,
                pause_us: 8_000,
                resume_us: 0
            })
        );
    }

    /// ★ MEASURED one-directionality: `CS=0 Pause=0` is both the boot default and the 100 % point,
    /// so there is no position more aggressive than default and `Owned` cannot differ from
    /// `Shared`. Asserting it here stops a future edit from inventing a difference.
    #[test]
    fn owned_and_shared_are_the_same_point() {
        assert_eq!(posture_tx_time(ContentionPosture::Owned), (0, 0));
        assert_eq!(posture_tx_time(ContentionPosture::Shared), (0, 0));
        let (cs, pause) = posture_tx_time(ContentionPosture::Yielding);
        assert!(cs > 0 || pause > 0, "Yielding must actually move");
        // On the measured ladder, 2000/8000 is the 61 % point — a real yield, well above the 17 %
        // floor at 2000/65534.
        assert_eq!((cs, pause), (2_000, 8_000));
        assert!(cs <= TX_TIME_CS_MAX_US && pause <= TX_TIME_PAUSE_MAX_US);
    }

    /// ★★ The bound is 13260, not 65535 — `wifi_api_set_tx_time` rejects above it with -EINVAL.
    /// A guard written against `u16::MAX` would let 13261..65535 through and the write would
    /// silently no-op while the PREVIOUS setting stayed in force.
    #[test]
    fn cs_time_bound_is_the_firmware_one_not_u16() {
        assert_eq!(TX_TIME_CS_MAX_US, 13_260);
        assert!(TX_TIME_CS_MAX_US < u32::from(u16::MAX));
        assert_eq!(TX_TIME_PAUSE_MAX_US, u32::from(u16::MAX));
    }

    // ---- cca_thresh ----

    #[test]
    fn parses_cca_threshold() {
        assert_eq!(parse_cca_thresh("-75\nOK\n"), Some(-75));
    }

    /// The firmware renders a `-1` response as the literal `out-of-range`, and `-1` is *also* a
    /// syntactically valid dBm reading — so the string check has to come first. This asserts the
    /// string is present to be checked, not that the parser handles it (the caller does).
    #[test]
    fn out_of_range_is_a_distinct_signal() {
        let text = "out-of-range\nOK\n";
        assert!(text.contains("out-of-range"));
        assert_eq!(parse_cca_thresh(text), None);
    }

    // ---- edca ----

    #[test]
    fn parses_all_four_access_categories() {
        let acs = parse_edca(SHOW_EDCA);
        assert_eq!(acs.len(), 4, "one block per AC");
        assert_eq!(
            acs[0],
            EdcaAc {
                ac: 0,
                aifsn: 7,
                cw_min: 15,
                cw_max: 1023,
                txop_limit: 0,
                txop_max: 0
            }
        );
        assert_eq!(
            acs[3],
            EdcaAc {
                ac: 3,
                aifsn: 2,
                cw_min: 3,
                cw_max: 7,
                txop_limit: 47,
                txop_max: 47
            }
        );
    }

    /// Best Effort is NRC index 1 (`mac80211_to_nrc_aci_map = {3,2,1,0}` maps mac80211's BE=2 to 1).
    #[test]
    fn best_effort_block_is_selectable() {
        let acs = parse_edca(SHOW_EDCA);
        let be = acs
            .iter()
            .find(|a| a.ac == EDCA_AC_BE)
            .expect("AC 1 present");
        assert_eq!(be.aifsn, 3);
        assert_eq!(be.cw_min, 15);
    }

    /// A block missing a field must be DROPPED, not defaulted — a fabricated `aifsn` would be spent
    /// by an airtime budget as if it were real.
    #[test]
    fn incomplete_edca_block_is_dropped() {
        let truncated = "[AC]\t\t\t\t : 0\n - aifsn\t\t\t : 7\n - cw min\t\t\t : 15\nOK\n";
        assert!(parse_edca(truncated).is_empty());
    }

    /// `show edca` reports contention WINDOWS (the WIM field is fed from mac80211's `cw_min`, which
    /// is a window); `ContentionApplied` wants EXPONENTS.
    #[test]
    fn contention_windows_convert_to_exponents() {
        assert_eq!(cw_window_to_exponent(15), 4);
        assert_eq!(cw_window_to_exponent(1023), 10);
        assert_eq!(cw_window_to_exponent(3), 2);
        assert_eq!(cw_window_to_exponent(0), 0);
        // Not a 2^n-1 value: rounded up, so the conversion stays monotone.
        assert_eq!(cw_window_to_exponent(20), 5);
    }

    // ---- MAC statistics ----

    /// ★ `"OK count:"` is a substring of `"NOK count:"`, and the AC lines below the header contain
    /// `OK(` and `NOK(` as well. Getting `ok` and `nok` the right way round is the whole test.
    #[test]
    fn parses_the_mac_rx_header() {
        assert_eq!(
            parse_mac_rx_stats(SHOW_MAC_RX),
            Some(MacRxStats {
                ok: 10_250,
                nok: 37,
                last_mcs: 7,
                fcs_error: 112
            })
        );
    }

    #[test]
    fn parses_the_mac_tx_header() {
        assert_eq!(
            parse_mac_tx_stats(SHOW_MAC_TX),
            Some(MacTxStats {
                ok: 9_987,
                rtx: 412,
                last_mcs: 5
            })
        );
    }

    /// The TX direction has no FCS-error field, so the RX parser must refuse TX output rather than
    /// invent a zero.
    #[test]
    fn rx_parser_rejects_tx_output() {
        assert!(parse_mac_rx_stats(SHOW_MAC_TX).is_none());
    }

    // ---- simple_rx ----

    #[test]
    fn parses_simple_rx_counters() {
        assert_eq!(
            parse_simple_rx(SHOW_SIMPLE_RX),
            Some(SimpleRxStats {
                rssi_dbm: -62,
                cs_cnt: 148_392,
                psdu_succ: 10_241,
                mpdu_rcv: 10_250,
                mpdu_succ: 10_203,
                snr: 23,
            })
        );
    }

    /// `read_channel_activity` differences a wrapping `u16`, so the truncation must be exact
    /// modulo 2^16 rather than a shift.
    #[test]
    fn channel_activity_truncation_is_exact_modulo_2_16() {
        let s = parse_simple_rx(SHOW_SIMPLE_RX).unwrap();
        assert_eq!(s.cs_cnt as u16, (148_392u32 % 65_536) as u16);
    }

    // ---- the channel table ----

    /// The join of `g_bd_ch_table[US]` with `s1g_ch_table_us` must be total and unambiguous:
    /// 45 rows, unique alias numbers, unique S1G channel numbers.
    #[test]
    fn channel_table_is_a_clean_join() {
        assert_eq!(US_S1G_CHANNELS.len(), 45);
        let mut aliases: Vec<u8> = US_S1G_CHANNELS.iter().map(|r| r.alias).collect();
        aliases.sort_unstable();
        let n = aliases.len();
        aliases.dedup();
        assert_eq!(
            aliases.len(),
            n,
            "alias numbers must be unique — they are the tuning key"
        );
        let mut chans: Vec<u8> = US_S1G_CHANNELS.iter().map(|r| r.s1g_channel).collect();
        chans.sort_unstable();
        let n = chans.len();
        chans.dedup();
        assert_eq!(chans.len(), n);
    }

    /// Two independently recorded bench facts, used as the table's cross-validation.
    #[test]
    fn channel_table_matches_the_recorded_bench_facts() {
        let by = |a: u8| *US_S1G_CHANNELS.iter().find(|r| r.alias == a).unwrap();
        // The `wifi_halow_s1g` preset's own comment: alias 161 = 925 MHz.
        assert_eq!(by(161).s1g_khz(), 925_000);
        assert_eq!(by(161).bw_mhz, 2);
        // The NRC7292 AP the Morse note was tuned against: 906 MHz at 4 MHz.
        assert_eq!(by(8).s1g_khz(), 906_000);
        assert_eq!(by(8).bw_mhz, 4);
        // The FrameIo survey's third anchor: alias 5 = 904.5 MHz at 1 MHz.
        assert_eq!(by(5).s1g_khz(), 904_500);
        assert_eq!(by(5).bw_mhz, 1);
    }

    /// ★ The US table has **no 8 MHz channels at all** — unlike the Morse MM6108. A planner that
    /// assumes the two HaLow radios have the same width menu is wrong here.
    #[test]
    fn us_table_has_no_8mhz_channels() {
        assert!(
            US_S1G_CHANNELS
                .iter()
                .all(|r| matches!(r.bw_mhz, 1 | 2 | 4))
        );
        assert_eq!(US_S1G_CHANNELS.iter().filter(|r| r.bw_mhz == 1).count(), 26);
        assert_eq!(US_S1G_CHANNELS.iter().filter(|r| r.bw_mhz == 2).count(), 13);
        assert_eq!(US_S1G_CHANNELS.iter().filter(|r| r.bw_mhz == 4).count(), 6);
    }

    // ---- set_channel argument validation (no process is spawned on any of these paths) ----

    fn knobs() -> Nrc7292Knobs {
        Nrc7292Knobs::new("halow0", "/usr/local/bin/cli_app")
    }

    #[test]
    fn unknown_alias_channel_is_refused() {
        let err = knobs().set_channel(200, Bandwidth::Bw20).unwrap_err();
        assert!(
            format!("{err}").contains("not in this radio's channel table"),
            "{err}"
        );
    }

    /// 40/80 MHz have no S1G meaning at all; refuse rather than silently tune at whatever width the
    /// channel happens to be.
    #[test]
    fn wifi_widths_are_refused() {
        for bw in [Bandwidth::Bw40, Bandwidth::Bw80] {
            let err = knobs().set_channel(161, bw).unwrap_err();
            assert!(format!("{err}").contains("no S1G meaning"), "{err}");
        }
    }

    /// The narrow variants are read as 1/2 MHz requests and must be checked against the row: alias
    /// 161 is a 2 MHz channel, so asking for 1 MHz on it is a contradiction, not a silent
    /// approximation.
    #[test]
    fn contradicting_width_is_refused_not_approximated() {
        let err = knobs().set_channel(161, Bandwidth::Nb5).unwrap_err();
        assert!(format!("{err}").contains("2 MHz wide, not 1 MHz"), "{err}");
    }

    // ---- capability ----

    /// ★★ The correction that matters most: S1G MCS10 is a 1 MHz-only rep-coded BPSK mode that is
    /// SLOWER and more robust than MCS0, so it sits below the ladder. `mcs_for_rssi` clamps a
    /// monotone ladder to `max_mcs`, so the preset's `10` hands the strongest link the slowest rate.
    #[test]
    fn capability_declares_a_monotone_mcs_ladder() {
        let cap = knobs().capability();
        match cap.rate {
            RateCapability::Wifi {
                max_mcs, max_nss, ..
            } => {
                assert_eq!(
                    max_mcs, 7,
                    "MCS10 is off-ladder and must not be declared as a ceiling"
                );
                assert_eq!(max_nss, 1, "single chain");
            }
            other => panic!("expected a Wi-Fi rate model, got {other:?}"),
        }
    }

    #[test]
    fn capability_declares_the_real_dbm_axis() {
        let cap = knobs().capability();
        assert_eq!(cap.tx_power_dbm, Some(DbmRange::new(1, 30)));
        // Nothing has metered this part, so no dB-per-index conversion may be derived.
        assert_eq!(cap.db_per_power_idx, None);
        assert_eq!(cap.min_tx_power, None);
    }

    /// The preset says `WifiMonitor`, which contradicts its own name and leaves nothing downstream
    /// able to tell a HaLow radio from a 5 GHz one.
    #[test]
    fn capability_names_the_bearer() {
        let cap = knobs().capability();
        assert_eq!(cap.kind, RadioKind::WifiHaLow);
        assert_eq!(cap.bands, vec![ndn_radio_hal::Band::Sub1GHz]);
        assert_eq!(cap.channels.len(), US_S1G_CHANNELS.len());
    }

    /// `set_edcca_ignore(true)` must ERROR, not silently succeed: the HAL's default `Ok(())` would
    /// tell a caller CSMA had been suppressed while the LMAC's LBT is still running.
    #[test]
    fn edcca_ignore_refuses_instead_of_lying() {
        let k = knobs();
        assert!(k.set_edcca_ignore(true).is_err());
        assert!(
            k.set_edcca_ignore(false).is_ok(),
            "nothing to undo is a real success"
        );
    }

    /// Out-of-range power is refused rather than clamped — the capability already advertises the
    /// span, so a caller that exceeds it has a bug worth hearing about. (No process is spawned:
    /// the range check runs first.)
    #[test]
    fn out_of_range_power_is_refused_before_the_radio_is_touched() {
        let k = knobs();
        assert!(k.set_tx_power_dbm(0).is_err());
        assert!(k.set_tx_power_dbm(31).is_err());
        assert!(k.set_tx_power_dbm(-10).is_err());
    }

    /// Same for the CCA threshold: −100..−35 is the vendor range, and out-of-range is an error, not
    /// a silent clamp.
    #[test]
    fn out_of_range_cca_threshold_is_refused() {
        let k = knobs();
        assert!(k.set_edcca_threshold_dbm(-101, -101).is_err());
        assert!(k.set_edcca_threshold_dbm(-34, -34).is_err());
    }

    /// No timing promise on this bearer — and the AR9271 precedent is that a fabricated one
    /// actively breaks the scheduler.
    #[test]
    fn tx_discipline_promises_nothing() {
        assert_eq!(knobs().tx_discipline(), TxDiscipline::BestEffort);
    }

    /// The trait defaults that must STAY defaults, asserted so an edit that "helpfully" implements
    /// one has to come past a test that says why it should not.
    #[test]
    fn knobs_without_an_actuator_stay_unsupported() {
        let k = knobs();
        // No TXAGC index scale exists; `show autotxgain` is read-only. ★ The refusal is now BY
        // NAME (`PowerRequest::Index` on an absolute-dBm part) rather than the trait's blanket
        // `Unsupported` — this part has a real power knob, just not an index one.
        assert!(
            k.set_tx_power(ndn_radio_hal::PowerRequest::index(10))
                .is_err(),
            "an absolute-dBm part must refuse an index request rather than invent a mapping"
        );
        // Single chain — so there is no second chain to cyclic-shift, and the honest answer is a
        // REFUSAL. This asserted `.is_ok()` on the old silent default, which contradicted this
        // test's own name: cognition decides CSD every tick, `apply_knobs` records a knob as
        // applied on `Ok(())`, and the bandit was then credited for a diversity gain no second
        // chain produced — verbatim the failure the power knob was already fixed for.
        assert!(
            k.set_tx_csd(true).is_err(),
            "a single-chain part must refuse CSD, not silently accept it"
        );
        // No MAC->baseband / baseband->RF counter pair on this silicon; the MPDU accounting that
        // does exist is reachable through `mac_tx_stats`, whose semantics differ.
        assert_eq!(k.read_tx_counters().unwrap(), None);
    }
}
