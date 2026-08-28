//! **The LR-FHSS PHY** — `SetPacketType 0x7`. Reachable, so it can be MEASURED.
//!
//! ## The datasheet contradicts itself, and the chip settles it
//!
//! * **§17.1**: *"In the LR20xx, LR-FHSS is implemented as a transmit-only mode."*
//! * **§17.2.2**: `LrFhssSetSyncword` *"configures the synchronization word utilized for LR-FHSS
//!   **detection on the receiver side**, and for building the Tx frame on the transmitter side."*
//!
//! Both cannot be true. There **is** a plausible mechanism for a genuine transmit-only limit:
//! LR-FHSS modulates at **488.28125 bit/s** ([`crate::airtime::LRFHSS_BIT_US`] = 2048 µs/bit) and
//! Table 11-2 gives the generic (G)FSK modem a `bitrate min` of **500 bps** — LR-FHSS sits 2.4%
//! *under* the demodulator's own floor, which would explain a part that can synthesise the waveform
//! and not detect it.
//!
//! That is a hypothesis. This module does not act on it: [`arm_rx`] issues the real
//! `SetRxContinuous` and the bridge forwards **the chip's literal status byte** in
//! `EVT_PHY_ERR`. If the chip refuses, the status byte says so and the reason is recorded where the
//! measurement is. If it arms, that is a finding — and still not a claim that it *receives*:
//! **arming is not receiving**, and only on-air traffic decides that.
//!
//! ## The hopping table is ours to write
//!
//! `WriteLrFhssHoppingTable` (0x59) takes up to 40 `(freq, nb_symbols)` couples with a `convert_freq`
//! bit that lets the frequencies be plain Hz, and `ReadLrFhssHoppingTable` (0x58) reads them back.
//! So the hop sequence is not Semtech's to compute from a seed — it is a table, and a table is the
//! substrate a **name-derived dwell schedule** needs. Both are exposed here; the vendor crate wraps
//! only the write, and wraps it in a form nothing outside the crate can call (see [`write_table`]).

use embedded_hal::digital::OutputPin;
use embedded_hal_async::spi::SpiBus;

use lr2021::lrfhss::{write_lr_fhss_hopping_table_cmd, Grid, Hopping, LrfhssBw, LrfhssCr};
use lr2021::{BusyPin, Lr2021, Lr2021Error};

use crate::phy;

/// LR-FHSS reset syncword, `0x2C0F7995` — the value §17.2.2 documents as the default. Kept rather
/// than replaced: it is what any other LR-FHSS transmitter on the bench will be using, and the
/// syncword is the one field the datasheet says is used for *detection*, so a non-default value
/// would confound the very experiment this PHY exists for.
pub const SYNCWORD: u32 = 0x2C0F_7995;

/// Default coding rate: **1/3** (`LrfhssCr::Cr1p3`), the most robust rung.
///
/// LR-FHSS's whole proposition is reach under interference, and the fast rungs give that away for a
/// bearer that is seconds-per-frame regardless. `CMD_SET_MOD` moves it.
pub const DEFAULT_CR: LrfhssCr = LrfhssCr::Cr1p3;

/// Default grid: **25.39 kHz** (`Grid::Grid25`), the FCC channel plan's grid. The 3.91 kHz grid is
/// the ETSI one; this bench is in the US 902-928 band.
pub const DEFAULT_GRID: Grid = Grid::Grid25;

/// Default hopping bandwidth: **1523.4 kHz** (`LrfhssBw::Bw1523p4`), which the driver's own example
/// labels "FCC use case" — the widest plan, and the one the 902-928 band is sized for.
pub const DEFAULT_BW: LrfhssBw = LrfhssBw::Bw1523p4;

/// Default sync-header replica count: **3**.
///
/// The header is what a receiver acquires on, and each replica costs
/// [`crate::airtime::LRFHSS_HEADER_BITS`] × 2048 µs ≈ 233 ms. Three is the LoRaWAN uplink
/// convention for the 902-928 plan. It is also the term that dominates a short frame's airtime,
/// which is worth knowing before reading a duration.
pub const DEFAULT_SYNC_HEADERS: u8 = 3;

/// **The LR-FHSS parameters.**
///
/// Note what is *not* here: there is no `SetLrFhssModulationParams`. Coding rate, grid, hopping mode
/// and bandwidth are arguments of **`LrFhssBuildFrame`**, i.e. they are chosen per frame at packet
/// build time, not programmed into the modem at bring-up. That is why [`apply_modem`] is so short —
/// the only standing modem state LR-FHSS has is its syncword.
#[derive(Clone, Copy, PartialEq, Eq)]
pub struct LrFhssParams {
    pub cr: LrfhssCr,
    pub grid: Grid,
    pub bw: LrfhssBw,
    /// Header replicas, 1..4 (the field is 4 bits, and more than 4 has no defined meaning).
    pub sync_header_cnt: u8,
    /// Hop-sequence selector handed to `LrFhssBuildFrame`. Only meaningful when the chip is
    /// computing the sequence itself; a host-written table ([`write_table`]) overrides it.
    pub sequence: u16,
    /// Per-device frequency offset, in grid steps.
    pub offset: i8,
}

impl Default for LrFhssParams {
    fn default() -> Self {
        Self {
            cr: DEFAULT_CR,
            grid: DEFAULT_GRID,
            bw: DEFAULT_BW,
            sync_header_cnt: DEFAULT_SYNC_HEADERS,
            sequence: 0,
            offset: 0,
        }
    }
}

impl LrFhssParams {
    /// Airtime of a `payload_len`-byte frame, µs. See [`crate::airtime::lrfhss_airtime_us`] — and
    /// its warning that the frame-structure constants are sourced from Semtech's `lr_fhss_mac.c` and
    /// **not verified against this chip**.
    pub fn airtime_us(&self, payload_len: u16) -> u32 {
        crate::airtime::lrfhss_airtime_us(self.cr as u8, self.sync_header_cnt, payload_len)
    }
}

/// **The LR-FHSS modem block of the shared bring-up sequence**: the syncword, and nothing else.
///
/// Everything a reader expects to find here — coding rate, grid, bandwidth, hopping — is an argument
/// of `LrFhssBuildFrame` instead, so it is applied in [`build_frame`] once per transmit. Kept as an
/// `apply_modem` anyway so [`crate::phy_link::apply`] has one shape for all three PHYs; a PHY whose
/// bring-up is one command is still a PHY with a bring-up.
pub async fn apply_modem<O, SPI, M>(
    radio: &mut Lr2021<O, SPI, M>,
    _p: &LrFhssParams,
) -> Result<(), Lr2021Error>
where
    O: OutputPin,
    SPI: SpiBus<u8>,
    M: BusyPin,
{
    radio.set_lrfhss_syncword(SYNCWORD).await
}

/// **Build one LR-FHSS frame into the chip** — `LrFhssBuildFrame`, which encodes the payload *and*
/// computes the internal hopping table. The caller then keys up with `SetTx`.
///
/// `hopping` is `Hopping::Hopping` when a hop plan is active and `Hopping::NoHopping` otherwise: the
/// intra-packet hopping this command enables is what LR-FHSS *is*, but a table nobody asked for is
/// still a configuration the host did not request.
pub async fn build_frame<O, SPI, M>(
    radio: &mut Lr2021<O, SPI, M>,
    p: &LrFhssParams,
    hopping: bool,
    payload: &[u8],
) -> Result<(), Lr2021Error>
where
    O: OutputPin,
    SPI: SpiBus<u8>,
    M: BusyPin,
{
    radio
        .lrfhss_build_packet(
            p.sync_header_cnt.clamp(1, 4),
            p.cr,
            p.grid,
            if hopping { Hopping::Hopping } else { Hopping::NoHopping },
            p.bw,
            p.sequence,
            p.offset,
            payload,
        )
        .await
}

/// **Write the hopping table** — `WriteLrFhssHoppingTable` (0x59), up to [`phy::MAX_HOPS`]
/// `(freq_hz, nb_symbols)` couples.
///
/// ★ **Not the vendor crate's `set_lrfhss_hopping`, which cannot be called from outside the crate**:
/// it takes `&[LrfhssHop]`, and `LrfhssHop`'s two fields are private with no constructor. The
/// command is issued here over `cmd_data_wr`, which is exactly the opcode-then-stream shape the
/// couples need.
///
/// `convert_freq` (bit 7 of the control byte) is set, so the four-byte frequencies are **plain Hz**
/// rather than PLL steps — the same unit `CMD_SET_HOP` carries and `SetRfFrequency` takes, so no
/// conversion sits between what the host asked for and what the chip hops to.
///
/// ⚠ **`pkt_length` and `nb_hopping_blocks` are inferred, not sourced.** `pkt_length` is passed as
/// the payload length in bytes and `nb_hopping_blocks` as
/// [`crate::airtime::lrfhss_block_count`] of the same frame, which is the only self-consistent
/// reading of a table that has to cover a frame cut into fragments. This is precisely why
/// [`read_table`] is exposed alongside it: the chip can be asked what it actually stored.
///
/// ## Ordering: this must come AFTER [`build_frame`]
///
/// `LrFhssBuildFrame` is documented as configuring the internal hopping table itself, so a table
/// written before it is overwritten by it. Writing afterwards is the only order in which a
/// host-supplied sequence survives to the air. Verify with [`read_table`] before trusting a hop
/// schedule.
pub async fn write_table<O, SPI, M>(
    radio: &mut Lr2021<O, SPI, M>,
    enable: bool,
    payload_len: u16,
    nb_blocks: u16,
    freqs: &[u32],
    nb_symbols: u16,
) -> Result<(), Lr2021Error>
where
    O: OutputPin,
    SPI: SpiBus<u8>,
    M: BusyPin,
{
    let n = freqs.len().min(phy::MAX_HOPS);
    let req = write_lr_fhss_hopping_table_cmd(
        enable,
        true, // convert_freq: the couples below are in Hz
        payload_len,
        n as u8,
        nb_blocks.min(u8::MAX as u16) as u8,
    );
    let mut couples = [0u8; 6 * phy::MAX_HOPS];
    for (i, f) in freqs[..n].iter().enumerate() {
        couples[6 * i..6 * i + 4].copy_from_slice(&f.to_be_bytes());
        couples[6 * i + 4..6 * i + 6].copy_from_slice(&nb_symbols.to_be_bytes());
    }
    radio.cmd_data_wr(&req, &couples[..6 * n]).await
}

/// **Read the hopping table back** — `ReadLrFhssHoppingTable` (0x58).
///
/// The vendor crate does not wrap it at all, and "the crate does not wrap it" is a reason to reach
/// the command, not a reason to stop: without a read-back, a written hop table is a hypothesis. `out`
/// receives the raw response, whose first two bytes are the status word as with every read command.
///
/// ⚠ **The response framing is inferred from the write command's own layout** — `(freq u32,
/// nb_symbols u16)` couples, six bytes each, after the two status bytes — and is unverified. It is
/// exposed as raw bytes for exactly that reason: a caller can look at what came back rather than
/// having it parsed into a shape that may be wrong.
pub async fn read_table<O, SPI, M>(
    radio: &mut Lr2021<O, SPI, M>,
    out: &mut [u8],
) -> Result<(), Lr2021Error>
where
    O: OutputPin,
    SPI: SpiBus<u8>,
    M: BusyPin,
{
    radio.cmd_rd(&[0x02, 0x58], out).await
}

/// Bytes a [`read_table`] response is expected to occupy for `n` couples: two status bytes plus six
/// per couple.
pub const fn read_table_len(n: usize) -> usize {
    2 + 6 * n
}

/// **Arm the receiver, and let the caller report what the chip said.**
///
/// This is the whole point of the module's opening note. It does not decide in advance whether
/// LR-FHSS can receive; it issues `SetRxContinuous` and returns the driver's result, so the bridge
/// can put the chip's own status byte on the wire in `EVT_PHY_ERR`.
///
/// ⚠ **A success here is NOT evidence that LR-FHSS receives.** Arming is not receiving: the command
/// may be accepted by a mode dispatcher that never engages a demodulator. Only frames delivered on
/// air settle it.
///
/// ## Why the extra `GetStatus`, and why `set_rx_continous` alone could not do this job
///
/// ★ **On this part the status clocked back during a command WRITE belongs to the PREVIOUS
/// command.** The vendor crate says so in its own words — `CmdStatus::Fail` is documented as *"Last
/// Command could not be executed"* — and its shape proves it: `cmd_wr` checks
/// `buffer.cmd_status()` from the bytes returned *while the request is being clocked out*, whereas
/// `cmd_rd` re-checks after the separate response phase, which is the transaction that finally
/// carries the request's own verdict.
///
/// So a bare `set_rx_continous()` cannot see whether `SetRx` was accepted: its `cmd_wr` inspects the
/// status left by [`apply_modem`]'s syncword write, and the only failures it can itself return are
/// an SPI fault or a BUSY that never drops. A chip that answers `SetRx` with `Fail`/`PErr` in a
/// transmit-only mode would return **`Ok(())` here** — and the bridge would report a PHY switch with
/// a receiver that was never armed. That is precisely the measurement this module exists to make,
/// and precisely the way to get it wrong.
///
/// One `GetStatus` read fixes it twice over: its own write phase carries `SetRx`'s status (so the
/// `?` below is the real verdict), and that value stays in the driver's cached status, so
/// [`crate::phy_link::status_byte`] puts *that* byte — with the chip mode the part was in — on the
/// wire in `EVT_PHY_ERR` rather than a stale one. The cost is one short SPI transaction on a path
/// that runs at PHY switches and between multi-second frames.
pub async fn arm_rx<O, SPI, M>(radio: &mut Lr2021<O, SPI, M>) -> Result<(), Lr2021Error>
where
    O: OutputPin,
    SPI: SpiBus<u8>,
    M: BusyPin,
{
    radio.set_rx_continous().await?;
    radio.get_status().await.map(|_| ())
}
