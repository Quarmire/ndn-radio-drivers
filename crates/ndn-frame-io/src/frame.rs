//! Platform-neutral on-air (de)framing: wrap an NDN-LP payload into the
//! `radiotap ++ 802.11 ++ <format body>` bytes the driver injects, and recover
//! it on capture. The socket I/O lives in the Linux [`af_packet`](crate)
//! backend; keeping the byte layout here makes every [`FrameFormat`] unit-
//! testable on any host.

use bytes::Bytes;
use ndn_transport::FaceError;

use crate::{
    CapturedFrame, ClockDomainId, FrameFormat, InjectFrame, LatchPoint, LinkStamp, radiotap,
};

/// 802.11 non-QoS data frame header (FC + Duration + 3×addr + SeqCtrl).
const DOT11_HDR_LEN: usize = 24;
/// QoS data frames carry an extra 2-byte QoS Control field.
const DOT11_QOS_HDR_LEN: usize = 26;
/// LLC/SNAP header preceding an EtherType-tagged payload in an 802.11 frame.
const LLC_SNAP_LEN: usize = 8;
pub const LLC_SNAP_PREFIX: [u8; 6] = [0xAA, 0xAA, 0x03, 0x00, 0x00, 0x00];

// ── ESP-NOW vendor-action-frame constants ────────────────────────────────────
// ESP-NOW is a vendor-specific 802.11 *Action* frame: a subset of raw
// injection. The layout (after the 24-byte MAC header) is the IDF/esp-wifi
// wire format: Category(0x7f) + OUI + 4-byte random + a vendor element
// (0xdd, len, OUI, Type=0x04, Version=0x01, Body). Matching it byte-for-byte
// is what lets a $5 ESP32 running stock `esp-wifi` ESP-NOW hear our frames.
const ESPNOW_CATEGORY: u8 = 0x7f; // vendor-specific action
const ESPNOW_ELEMENT_ID: u8 = 0xdd; // vendor-specific element
const ESPNOW_TYPE: u8 = 0x04; // ESP-NOW
/// ESP-NOW protocol version. esp-idf v5+/`esp-radio` emit **2**; older stacks
/// used 1. We transmit 2 and accept either on receive (see [`parse`]).
const ESPNOW_VERSION: u8 = 0x02;
const ESPNOW_RANDOM_LEN: usize = 4;
/// ESP-NOW body cap (the element `Length` is a single byte: OUI+Type+Ver = 5,
/// so body ≤ 250). A face speaking ESP-NOW must fragment to this via its MTU.
pub const ESPNOW_MAX_BODY: usize = 250;
/// Espressif's OUI — the default for [`FrameFormat::EspNow`].
pub const ESPNOW_OUI: [u8; 3] = [0x18, 0xfe, 0x34];

/// The broadcast/default-source addresses now live in `ndn-radio-hal`; re-exported
/// through the crate root so `frame::BROADCAST` / `frame::DEFAULT_SRC` still resolve.
pub use crate::{BROADCAST, DEFAULT_SRC};

/// **SipHash-2-4** (Aumasson & Bernstein) — a fast *keyed* PRF, vendored (no dep)
/// and shared with the FHSS rendezvous (`HopSchedule`). Keyed because the
/// name-group hash is the compiled form of a **public** name: an unkeyed hash lets
/// any outsider compute (or cheaply collide) a victim's group hash and flood its
/// receive filter. Under a private trust domain's secret key the group hash is
/// unforgeable and unlinkable to outsiders; under the well-known [`OPEN_GROUP_KEY`]
/// it is a strong public hash giving an open receiver set. (This is not the last
/// line of DoS defence — that is PIT-gated verification + rate-limiting; keying just
/// raises the bar for *outsiders* targeting a private group's pre-parse filter.)
pub fn siphash24(key: &[u8; 16], data: &[u8]) -> u64 {
    let k0 = u64::from_le_bytes(key[0..8].try_into().unwrap());
    let k1 = u64::from_le_bytes(key[8..16].try_into().unwrap());
    let mut v0 = 0x736f_6d65_7073_6575 ^ k0;
    let mut v1 = 0x646f_7261_6e64_6f6d ^ k1;
    let mut v2 = 0x6c79_6765_6e65_7261 ^ k0;
    let mut v3 = 0x7465_6462_7974_6573 ^ k1;
    macro_rules! round {
        () => {{
            v0 = v0.wrapping_add(v1);
            v1 = v1.rotate_left(13);
            v1 ^= v0;
            v0 = v0.rotate_left(32);
            v2 = v2.wrapping_add(v3);
            v3 = v3.rotate_left(16);
            v3 ^= v2;
            v0 = v0.wrapping_add(v3);
            v3 = v3.rotate_left(21);
            v3 ^= v0;
            v2 = v2.wrapping_add(v1);
            v1 = v1.rotate_left(17);
            v1 ^= v2;
            v2 = v2.rotate_left(32);
        }};
    }
    let mut chunks = data.chunks_exact(8);
    for c in &mut chunks {
        let m = u64::from_le_bytes(c.try_into().unwrap());
        v3 ^= m;
        round!();
        round!();
        v0 ^= m;
    }
    let mut last = (data.len() as u64 & 0xff) << 56;
    for (i, &b) in chunks.remainder().iter().enumerate() {
        last |= (b as u64) << (8 * i);
    }
    v3 ^= last;
    round!();
    round!();
    v0 ^= last;
    v2 ^= 0xff;
    round!();
    round!();
    round!();
    round!();
    v0 ^ v1 ^ v2 ^ v3
}

/// A trust-context key for name-group hashing (see [`siphash24`]). The well-known
/// [`OPEN_GROUP_KEY`] gives an open receiver set (anyone computes the hash and
/// filters/joins); a shared secret scopes a group to a trust domain.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct GroupKey(pub [u8; 16]);

/// The well-known key for open/public namespaces — an open receiver set.
pub const OPEN_GROUP_KEY: GroupKey = GroupKey(*b"ndn/open-group!!");

/// An **ephemeral, per-boot, rotating source tag** — the source-field value the named-radio doctrine
/// (mac-addressing-doctrine §2) calls for. It is a locally-administered, *individual* 6-byte address
/// with **no routing meaning**: randomized once per boot (from a caller-supplied `boot_seed`) and
/// rotated on a schedule to bound linkability. It is **not** a host identity — the doctrine forbids
/// keying any routing/forwarding state on it. Its only jobs are per-frame RSSI attribution (a
/// per-neighbour `SignalStore` key), per-source DoS rate-limiting, and disambiguating two producers
/// emitting under one prefix at once.
///
/// The nonce for a given instant is `SipHash(boot_seed, rotation_epoch)` truncated to the low 46
/// bits, with the first octet forced to U/L=local, I/G=individual — so it stays inert to real
/// networks and can never be mistaken for a manufacturer-assigned host MAC.
#[derive(Clone, Copy, Debug)]
pub struct EphemeralSource {
    boot_seed: u64,
    rotation_period_ms: u64,
}

impl EphemeralSource {
    /// `boot_seed` should be drawn from a per-boot entropy source by the caller (time ⊕ pid ⊕ face,
    /// or an RNG). `rotation_period_ms` is how long one nonce stays stable; `0` disables rotation
    /// (one fixed nonce for the whole boot — still per-boot random, just not rotating).
    pub const fn new(boot_seed: u64, rotation_period_ms: u64) -> Self {
        Self {
            boot_seed,
            rotation_period_ms,
        }
    }

    /// The source address in effect at `now_ms`. Frames within one rotation period share it (so a
    /// receiver can attribute their RSSI to one neighbour); it changes across periods and boots.
    pub fn current(&self, now_ms: u64) -> [u8; 6] {
        let epoch = if self.rotation_period_ms == 0 {
            0
        } else {
            now_ms / self.rotation_period_ms
        };
        let mut key = [0u8; 16];
        key[..8].copy_from_slice(&self.boot_seed.to_le_bytes());
        let h = siphash24(&key, &epoch.to_le_bytes()).to_le_bytes();
        // Low 46 bits into the address body; force U/L=local (0x02) + I/G=individual (clear 0x01).
        let mut m = [h[0], h[1], h[2], h[3], h[4], h[5]];
        m[0] = (m[0] & 0xFC) | 0x02;
        m
    }
}

/// Build `radiotap ++ 802.11 ++ <format body>` for one injected frame. The
/// 802.11 address fields are filled from `frame.dst`/`frame.src`/`frame.addr3` — under the
/// Tier-0 layout `addr1 ‖ addr2` are the name's prefix-set filter and `addr3` the ephemeral
/// nonce, so no host identity appears on the wire.
///
/// The radiotap TX header carries the rate: a per-frame MCS for `RawNdn`, or a
/// robust legacy rate for `EspNow` (1 Mbps on 2.4 GHz). The 802.11 frame itself
/// is built by [`build_dot11`] — backends that supply their own rate header
/// (e.g. the RTL88xx USB driver's TX descriptor) call that directly instead.
pub fn build(format: FrameFormat, frame: &InjectFrame) -> Result<Vec<u8>, FaceError> {
    // Resolve the frame's intent to an 802.11 rate for the radiotap header (a
    // conservative default capability is right for a header-only hint), then
    // build. The exact-rate path ([`build_at`]) is used when a caller has already
    // resolved a rate (the cognitive face, fixed-rate benches).
    let mcs = crate::McsDescriptor::for_intent(&frame.tx, crate::MAX_RELIABLE_MCS, false, false);
    build_at(format, frame, mcs)
}

/// Like [`build`], but at an explicit `mcs` — the radiotap TX header carries this
/// exact rate instead of resolving `frame.tx`. The counterpart of
/// [`WifiRadio::inject_at`](crate::WifiRadio::inject_at) for the AF_PACKET path.
pub fn build_at(
    format: FrameFormat,
    frame: &InjectFrame,
    mcs: crate::McsDescriptor,
) -> Result<Vec<u8>, FaceError> {
    // Build the 802.11 frame first; this also runs the per-format validation
    // (e.g. the ESP-NOW body cap), so a bad frame errors before the radiotap.
    let dot11 = build_dot11(format, frame)?;
    let mut out = Vec::with_capacity(16 + dot11.len());
    match format {
        // ESP-NOW rides a robust legacy rate (1 Mbps), not an MCS.
        FrameFormat::EspNow { .. } => out.extend_from_slice(&radiotap::build_tx_legacy(2)),
        // S1G/HaLow: no 11n/ac MCS — the on-chip MAC picks the sub-GHz rate.
        FrameFormat::RawNdnS1g { .. } => out.extend_from_slice(&radiotap::build_tx_s1g()),
        _ => out.extend_from_slice(&radiotap::build_tx_header(mcs.index, mcs.short_gi)),
    }
    out.extend_from_slice(&dot11);
    Ok(out)
}

/// Build one **A-MSDU** frame for the AF_PACKET monitor path: radiotap (per
/// `format`/`mcs`) ++ a single QoS-Data MPDU carrying `msdus` as A-MSDU
/// subframes under one PHY preamble and one FCS. The link-layer bundling
/// actuator — one preamble amortized over many NDN packets, the bigger win at
/// S1G where preambles are long and rates low. All subframes ride one MPDU
/// (addr1/RA = addr3 = `ra`, addr2/TA = `ta`); each subframe carries its own
/// DA/SA, so a broadcast face collapses to one A-MSDU. The caller bounds the
/// aggregate size (S1G caps the max MPDU per bandwidth). Byte layout matches the
/// RTL/MT USB backends' `build_amsdu_body` (they prepend a chip TX descriptor
/// instead of radiotap); the RX side de-aggregates via [`parse_dot11`], which
/// already handles QoS-Data for RawNdn/RawNdnS1g. Only RawNdn/RawNdnS1g support
/// A-MSDU.
pub fn build_amsdu(
    format: FrameFormat,
    ra: [u8; 6],
    ta: [u8; 6],
    msdus: &[([u8; 6], [u8; 6], Bytes)],
    seq: u16,
    mcs: crate::McsDescriptor,
) -> Result<Vec<u8>, FaceError> {
    let ethertype = match format {
        FrameFormat::RawNdn { ethertype } | FrameFormat::RawNdnS1g { ethertype } => ethertype,
        other => {
            return Err(FaceError::Io(std::io::Error::new(
                std::io::ErrorKind::InvalidInput,
                format!("A-MSDU unsupported for frame format {other:?}"),
            )));
        }
    };
    if msdus.is_empty() {
        return Err(FaceError::Io(std::io::Error::new(
            std::io::ErrorKind::InvalidInput,
            "A-MSDU needs at least one MSDU",
        )));
    }
    let mut out = Vec::with_capacity(
        16 + DOT11_QOS_HDR_LEN + msdus.iter().map(|(_, _, p)| 32 + p.len()).sum::<usize>(),
    );
    // Radiotap TX header — identical rate choice to `build_at`.
    match format {
        FrameFormat::RawNdnS1g { .. } => out.extend_from_slice(&radiotap::build_tx_s1g()),
        _ => out.extend_from_slice(&radiotap::build_tx_header(mcs.index, mcs.short_gi)),
    }
    // QoS-Data MPDU header (26 B): FC subtype 8 (QoS Data); A-MSDU-present in QoS Ctrl.
    out.extend_from_slice(&[0x88, 0x00]); // FC: type=Data, subtype=QoS Data
    out.extend_from_slice(&[0x00, 0x00]); // Duration
    out.extend_from_slice(&ra); // addr1 (RA)
    out.extend_from_slice(&ta); // addr2 (TA)
    out.extend_from_slice(&ra); // addr3 (BSSID)
    out.extend_from_slice(&((seq & 0x0fff) << 4).to_le_bytes()); // SeqCtrl
    out.extend_from_slice(&[0x80, 0x00]); // QoS Ctrl: A-MSDU Present (bit 7), TID 0
    let last = msdus.len() - 1;
    for (i, (da, sa, payload)) in msdus.iter().enumerate() {
        let msdu_len = LLC_SNAP_LEN + payload.len(); // LLC/SNAP + payload
        out.extend_from_slice(da); // subframe DA
        out.extend_from_slice(sa); // subframe SA
        out.extend_from_slice(&(msdu_len as u16).to_be_bytes()); // Length (big-endian)
        out.extend_from_slice(&LLC_SNAP_PREFIX);
        out.extend_from_slice(&ethertype.to_be_bytes());
        out.extend_from_slice(payload);
        if i != last {
            // Pad every subframe but the last to a 4-byte boundary.
            let sub_len = 14 + msdu_len; // DA+SA+Len + MSDU
            let pad = (4 - (sub_len % 4)) % 4;
            out.extend(std::iter::repeat_n(0u8, pad));
        }
    }
    Ok(out)
}

/// Build just the **802.11 frame** for `frame` under `format` — the bytes that
/// follow the radiotap header (or, for a hardware backend, its own TX
/// descriptor). Factored out of [`build`] so the RTL88xx USB driver — which
/// prepends a chip TX descriptor and sets the rate there, not via radiotap —
/// shares the exact same on-air byte layout (notably the ESP-NOW vendor-action
/// frame a stock `esp-wifi` peer keys on).
pub fn build_dot11(format: FrameFormat, frame: &InjectFrame) -> Result<Vec<u8>, FaceError> {
    let mut out = Vec::with_capacity(64 + frame.payload.len());
    match format {
        // RawNdn and RawNdnS1g share the exact data-frame body; they differ only
        // in the radiotap TX rate header chosen in `build_at`.
        FrameFormat::RawNdn { ethertype } | FrameFormat::RawNdnS1g { ethertype } => {
            match (frame.extra, frame.htc) {
                // ── THE FILTER FRAME: 4-address QoS-Data + HT Control, 190-bit Blur ────────────
                // ★ **THE ONE WIRE MAPPING.** `extra[0..6] → addr4`, `extra[6..8] → QoS Control`.
                // Every backend routes through here; nothing else may split the extra region, which
                // is why the HAL seam is named `extra` and not `addr4`.
                //
                // ToDS=FromDS=1 makes addr4 present; subtype QoS-Data makes QoS Control present;
                // the Order/+HTC bit makes HT Control present. HT Control carries the exact-match
                // fingerprint + the extra-region bitmap. The base 126-bit Blur still lives
                // byte-identically in addr1‖addr2‖addr3[0:4], so a base-only receiver reads this
                // frame with **zero false negatives** — the coexistence contract, and the whole
                // mid-upgrade story.
                //
                // The last 16 bits are literally free: this frame is already QoS-Data and was
                // already emitting two zero bytes here, so the 36-byte header is paid for whether
                // or not they carry entropy (MEASURED: P7 S7 reports 12 added bytes at 174 AND at
                // 190). A-MSDU keeps QoS Control on its own **3-address** frame (`build_amsdu`),
                // where the A-MSDU-Present bit has an actual reader; in this shape it had neither a
                // writer nor a reader anywhere in the tree and was being paid for in false
                // positives.
                (Some(extra), Some(htc)) => {
                    // FC: type=Data, subtype=QoS Data (0x88); ToDS+FromDS+Order (0x83).
                    out.extend_from_slice(&[0x88, 0x83]);
                    out.extend_from_slice(&[0x00, 0x00]); // Duration — NOT filter (see tier0.rs)
                    out.extend_from_slice(&frame.dst); // addr1 = Tier-0 filter hi
                    out.extend_from_slice(&frame.src); // addr2 = Tier-0 filter lo
                    out.extend_from_slice(&frame.addr3.unwrap_or(frame.dst)); // addr3 = base[12:16]‖id‖flags
                    out.extend_from_slice(&[0x00, 0x00]); // SeqCtrl
                    out.extend_from_slice(&extra[0..6]); // addr4      = extra Blur bits 0..48
                    out.extend_from_slice(&extra[6..8]); // QoS Control = extra Blur bits 48..64
                    out.extend_from_slice(&htc); // HT Control = fingerprint(24b LE) ‖ region bitmap
                    out.extend_from_slice(&LLC_SNAP_PREFIX);
                    out.extend_from_slice(&ethertype.to_be_bytes());
                    out.extend_from_slice(&frame.payload);
                }
                // ── BASE PROFILE: 802.11 non-QoS 3-address data frame ─────────────────────────
                // addr1/addr3 = destination group (or broadcast); addr2 = name-derived source.
                // The NDN name is the addressing — these fields are a name-keyed index, not host ids.
                _ => {
                    out.extend_from_slice(&[0x08, 0x00]); // FC: type=Data, subtype=0
                    out.extend_from_slice(&[0x00, 0x00]); // Duration
                    out.extend_from_slice(&frame.dst); // addr1 (RA/DA) = group / Tier-0 filter hi
                    out.extend_from_slice(&frame.src); // addr2 (TA/SA) = name-derived / filter lo
                    // addr3: the ephemeral source nonce when addr1‖addr2 is a Tier-0 filter, else
                    // the legacy BSSID slot (a copy of dst). Nothing on the RX path reads addr3 for
                    // the legacy layout, so the fallback is byte-compatible with prior deployments.
                    out.extend_from_slice(&frame.addr3.unwrap_or(frame.dst)); // addr3 (BSSID / nonce)
                    out.extend_from_slice(&[0x00, 0x00]); // SeqCtrl
                    out.extend_from_slice(&LLC_SNAP_PREFIX);
                    out.extend_from_slice(&ethertype.to_be_bytes());
                    out.extend_from_slice(&frame.payload);
                }
            }
        }
        FrameFormat::EspNow { oui } => {
            if frame.payload.len() > ESPNOW_MAX_BODY {
                return Err(FaceError::Io(std::io::Error::new(
                    std::io::ErrorKind::InvalidInput,
                    "ESP-NOW body > 250 B — set a smaller face MTU",
                )));
            }
            // 802.11 vendor-specific Action frame (management subtype 13).
            // ESP-NOW requires addr1 = broadcast (its receivers key on it).
            out.extend_from_slice(&[0xd0, 0x00]); // FC: type=Mgmt, subtype=Action
            out.extend_from_slice(&[0x00, 0x00]); // Duration
            out.extend_from_slice(&[0xff; 6]); // addr1 = broadcast
            out.extend_from_slice(&frame.src); // addr2 = src
            out.extend_from_slice(&[0xff; 6]); // addr3 (BSSID) = broadcast
            out.extend_from_slice(&[0x00, 0x00]); // SeqCtrl
            // Action body.
            out.push(ESPNOW_CATEGORY);
            out.extend_from_slice(&oui);
            out.extend_from_slice(&[0u8; ESPNOW_RANDOM_LEN]); // random value
            // Vendor-specific element carrying the ESP-NOW payload.
            out.push(ESPNOW_ELEMENT_ID);
            out.push((5 + frame.payload.len()) as u8); // OUI(3)+Type(1)+Ver(1)+body
            out.extend_from_slice(&oui);
            out.push(ESPNOW_TYPE);
            out.push(ESPNOW_VERSION);
            out.extend_from_slice(&frame.payload);
        }
        FrameFormat::Raw80211 => {
            // The payload IS the complete 802.11 frame; inject it verbatim.
            out.extend_from_slice(&frame.payload);
        }
        other => {
            return Err(FaceError::Io(std::io::Error::new(
                std::io::ErrorKind::Unsupported,
                format!("frame format {other:?} not yet implemented"),
            )));
        }
    }
    Ok(out)
}

/// Recover the NDN payload + transmitter address from a captured buffer
/// (`radiotap ++ 802.11 ++ …`). `None` if the frame isn't ours.
///
/// # Two things radiotap says that this used to ignore, both live defects on 802.11ah
///
/// ★ **The FCS.** `IEEE80211_RADIOTAP_F_FCS` means the driver left the frame's trailing 4-byte
/// checksum attached. Both HaLow drivers set it on every data frame — Newracom unconditionally
/// (`rt_flags = 0x10`, then `skb_put(skb, 4)` to append the FCS), Morse from
/// `MORSE_RX_STATUS_FLAGS_FCS_INCLUDED` — so before this, **four bytes of FCS rode inside the NDN
/// payload of every HaLow frame we received.** It stayed hidden because the on-air example only
/// checked `payload.starts_with(MARKER)`; an actual NDN-TLV parse would have rejected the packet.
/// `F_BADFCS` is the same class in the other direction: a frame the PHY *knows* is corrupt was
/// being delivered as good. Both are handled here, for every format and every backend — a 2.4 GHz
/// NIC that sets `F_FCS` benefits identically.
///
/// ★ **The S1G TLV.** On 802.11ah the per-frame MCS and (on the NRC7292) the *only* RSSI live in a
/// radiotap TLV, not in `MCS`/`DBM_ANTSIGNAL`. They are picked up here as a last-resort fallback,
/// after the caller's out-of-band values and after the standard fields.
///
/// ⚠ **`CapturedFrame.phy` is always `None` here, and that is a wire fact, not an omission.**
/// [`PhyMetrics`](ndn_radio_hal::PhyMetrics) is SNR / EVM / CFO, and **radiotap as these drivers
/// emit it carries none of the three**: no `DBM_ANTNOISE` field is present (so not even
/// `signal − noise` is available), and the 6-byte S1G TLV is format / response-indication / GI /
/// bandwidth / MCS / colour / uplink / signal. Backends that *do* fill `phy` read it from a chip
/// RX descriptor they parse themselves ([`crate::frame::parse_dot11`] callers), which is a path
/// an `AF_PACKET` monitor socket does not have. ★ Both HaLow chips measure a quality figure per
/// frame and both vendor drivers drop it before the netdev — Morse's `morse_skb_rx_status.
/// noise_dbm` is declared and never read, Newracom's `frame_hdr.flags.rx.snr` goes only into a
/// per-peer moving average — so closing this is a driver patch, not a parser change. Faking the
/// field from a per-peer average would be a different quantity wearing a per-frame label.
///
/// ⚠ **`CapturedFrame.mcs_index` is documented as an 802.11n MCS, and an S1G MCS is not one.**
/// Surfacing it there is deliberate — it is the only field that can hold it, and reporting *no*
/// rate on a HaLow link leaves rate adaptation with no receive-side feedback at all — but the two
/// ladders are different: S1G rates depend on the channel width, and MCS10 is a 1 MHz-only
/// repetition-coded BPSK mode that is **slower and more robust than MCS0**, i.e. the ladder is not
/// even monotone. Do not put an S1G index through [`crate::mcs_phy_rate_bps`] (the 11n 20 MHz
/// table); use [`ndn_radio_hal::s1g_phy_rate_bps`], which takes the width the same TLV reports.
pub fn parse(
    format: FrameFormat,
    buf: &[u8],
    rssi: Option<i8>,
    mcs: Option<u8>,
    domain: ClockDomainId,
) -> Option<CapturedFrame> {
    let info = radiotap::parse(buf)?;
    if info.bad_fcs() {
        return None; // the PHY told us these bytes are corrupt
    }
    let mut body = buf.get(info.header_len..)?;
    if info.fcs_included() {
        body = body.get(..body.len().checked_sub(4)?)?;
    }
    // If radiotap carried a TSFT, build a hardware receive stamp for it. The
    // caller supplies the clock `domain` (a TSF counter is per-NIC); the latch
    // is `MacDone` (~1 µs) and precision is clamped to that latch's floor.
    let stamp = info.tsft.map(|raw| {
        LinkStamp::new(
            raw,
            domain,
            LatchPoint::MacDone.precision_floor_ns(),
            LatchPoint::MacDone,
        )
    });
    // radiotap RSSI/rate are the fallback when the caller has no out-of-band read; the S1G TLV is
    // the fallback after that (and, on the NRC7292, the only source of either).
    let s1g_rssi = info.s1g.and_then(|s| s.rssi_dbm);
    let s1g_mcs = info.s1g.and_then(|s| s.mcs);
    parse_dot11(
        format,
        body,
        rssi.or(info.rssi_dbm).or(s1g_rssi),
        mcs.or(info.mcs_index).or(s1g_mcs),
        stamp,
    )
}

/// Recover the NDN payload + transmitter address from a bare **802.11 frame**
/// `body` (no radiotap) under `format`. `rssi`/`mcs`/`stamp` are passed through
/// to the returned [`CapturedFrame`] as-is. The counterpart to [`build_dot11`]:
/// a hardware backend that strips its own RX descriptor (reading RSSI/rate and
/// latching a receive timestamp from it) recovers the payload through this,
/// sharing the format byte layout with the radiotap-based [`parse`], which
/// instead builds the `stamp` from radiotap TSFT.
pub fn parse_dot11(
    format: FrameFormat,
    body: &[u8],
    rssi: Option<i8>,
    mcs: Option<u8>,
    stamp: Option<LinkStamp>,
) -> Option<CapturedFrame> {
    if body.len() < 2 {
        return None;
    }
    let fc0 = body[0];

    match format {
        FrameFormat::RawNdn { ethertype } | FrameFormat::RawNdnS1g { ethertype } => {
            if (fc0 >> 2) & 0x03 != 0x02 {
                return None; // not a data frame
            }
            let fc1 = body[1];
            // The 802.11 header grows by whichever optional fields the frame control announces,
            // in fixed order after SeqCtrl: addr4 (when ToDS=FromDS=1), QoS Control (subtype
            // QoS-Data), HT Control (Order/+HTC bit). Compute the true header length from the bits
            // rather than assuming — the wide profile sets all three (36 B), an A-MSDU sets only
            // QoS (26 B), a plain data frame none (24 B).
            let four_addr = (fc1 & 0x03) == 0x03; // ToDS && FromDS
            let qos = (fc0 >> 4) & 0x08 != 0; // subtype QoS-Data
            let htc = (fc1 & 0x80) != 0; // Order bit ⇒ HT Control present
            let hdr_len = DOT11_HDR_LEN
                + if four_addr { 6 } else { 0 }
                + if qos { 2 } else { 0 }
                + if htc { 4 } else { 0 };
            if body.len() < hdr_len + LLC_SNAP_LEN {
                return None;
            }
            let llc = &body[hdr_len..hdr_len + LLC_SNAP_LEN];
            if llc[..6] != LLC_SNAP_PREFIX || llc[6..8] != ethertype.to_be_bytes() {
                return None;
            }
            let mut ta = [0u8; 6];
            ta.copy_from_slice(&body[10..16]);
            let mut group = [0u8; 6];
            group.copy_from_slice(&body[4..10]); // addr1 (RA/DA)
            // addr3 (BSSID slot): the sender's ephemeral nonce under the Tier-0 layout, where
            // addr1‖addr2 is the prefix-set filter and so cannot also carry the source.
            let addr3 = body.get(16..22).map(|s| {
                let mut a = [0u8; 6];
                a.copy_from_slice(s);
                a
            });
            // ★ **THE ONE WIRE MAPPING, inverted.** The extra Blur region is reassembled from
            // `addr4` (offset 24, present when four_addr) ‖ QoS Control (immediately after it).
            // Both must be present or the region is absent: half a projection is not a coarser
            // projection, it is a different one, and testing 190-bit masks against it is the
            // false-negative direction. Whether the bytes may be *tested* is then the HT Control
            // bitmap's answer (`tier0::extra_regions_usable`), not this function's.
            let extra = if four_addr && qos {
                body.get(24..32).map(|s| {
                    let mut a = [0u8; 8];
                    a.copy_from_slice(s);
                    a
                })
            } else {
                None
            };
            let htc_bytes = if htc {
                let off = DOT11_HDR_LEN + if four_addr { 6 } else { 0 } + if qos { 2 } else { 0 };
                body.get(off..off + 4).map(|s| {
                    let mut h = [0u8; 4];
                    h.copy_from_slice(s);
                    h
                })
            } else {
                None
            };
            Some(CapturedFrame {
                payload: Bytes::copy_from_slice(&body[hdr_len + LLC_SNAP_LEN..]),
                addr: Some(ta),
                group: Some(group),
                addr3,
                extra,
                htc: htc_bytes,
                rssi_dbm: rssi,
                mcs_index: mcs,
                stamp,
                phy: None,
            })
        }
        FrameFormat::EspNow { oui } => {
            // Must be a vendor-specific Action frame (FC first octet 0xd0).
            if fc0 != 0xd0 {
                return None;
            }
            let action = body.get(DOT11_HDR_LEN..)?;
            // Category + OUI + random, then the vendor element.
            let elem_off = 1 + 3 + ESPNOW_RANDOM_LEN;
            if action.first() != Some(&ESPNOW_CATEGORY) || action.get(1..4)? != oui {
                return None;
            }
            let elem = action.get(elem_off..)?;
            if elem.first() != Some(&ESPNOW_ELEMENT_ID) {
                return None;
            }
            let len = *elem.get(1)? as usize;
            // Tolerate ESP-NOW version 1 or 2 (we transmit 2).
            if elem.get(2..5)? != oui
                || elem.get(5) != Some(&ESPNOW_TYPE)
                || !matches!(elem.get(6), Some(1 | 2))
            {
                return None;
            }
            let body_len = len.checked_sub(5)?; // minus OUI(3)+Type(1)+Ver(1)
            let payload = elem.get(7..7 + body_len)?;
            let mut ta = [0u8; 6];
            ta.copy_from_slice(&body[10..16]);
            let mut group = [0u8; 6];
            group.copy_from_slice(&body[4..10]); // addr1 (broadcast for ESP-NOW)
            Some(CapturedFrame {
                payload: Bytes::copy_from_slice(payload),
                addr: Some(ta),
                group: Some(group),
                addr3: None, // ESP-NOW addr3 is broadcast, not a nonce
                extra: None,
                htc: None,
                rssi_dbm: rssi,
                mcs_index: mcs,
                stamp,
                phy: None,
            })
        }
        FrameFormat::Raw80211 => {
            // The whole 802.11 frame is the payload (the caller parses it). Still
            // surface addr2 (TA) and addr1 (RA) from the fixed header offsets —
            // management and data frames share the first 16 bytes — so signal
            // plumbing and dedup work uniformly.
            if body.len() < DOT11_HDR_LEN {
                return None;
            }
            let mut ta = [0u8; 6];
            ta.copy_from_slice(&body[10..16]);
            let mut group = [0u8; 6];
            group.copy_from_slice(&body[4..10]);
            let addr3 = body.get(16..22).map(|s| {
                let mut a = [0u8; 6];
                a.copy_from_slice(s);
                a
            });
            Some(CapturedFrame {
                payload: Bytes::copy_from_slice(body),
                addr: Some(ta),
                group: Some(group),
                addr3,
                extra: None,
                htc: None,
                rssi_dbm: rssi,
                mcs_index: mcs,
                stamp,
                phy: None,
            })
        }
        _ => None,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::TxIntent;

    const SRC: [u8; 6] = [0x02, 0x4e, 0x44, 0x4e, 0x00, 0x01];

    fn frame(payload: &[u8]) -> InjectFrame {
        InjectFrame {
            payload: Bytes::copy_from_slice(payload),
            tx: TxIntent::CONSERVATIVE,
            dst: BROADCAST,
            src: SRC,
            addr3: None,
            extra: None,
            htc: None,
        }
    }

    #[test]
    fn raw_ndn_round_trips() {
        let fmt = FrameFormat::RawNdn { ethertype: 0x8624 };
        let wire = build(fmt, &frame(b"\x05\x03interest")).unwrap();
        let got = parse(fmt, &wire, Some(-50), Some(3), crate::ClockDomainId(0)).unwrap();
        assert_eq!(got.payload.as_ref(), b"\x05\x03interest");
        assert_eq!(got.addr, Some(SRC));
        assert_eq!(got.group, Some(BROADCAST));
        assert_eq!(got.rssi_dbm, Some(-50));
    }

    /// Build the exact bytes an NRC7292 monitor vif delivers for one of our S1G data frames:
    /// `struct nrc_radiotap_hdr` (34 B, `FLAGS = 0x10`, S1G TLV at 24) ++ the 802.11 frame ++ the
    /// 4-byte FCS the driver appends with `skb_put(skb, 4)`.
    fn nrc7292_capture(dot11: &[u8], mcs: u8, rssi: i8, flags: u8) -> Vec<u8> {
        let mut w = vec![0u8, 0];
        w.extend_from_slice(&34u16.to_le_bytes());
        w.extend_from_slice(&(((1u32) | (1 << 1) | (1 << 3) | (1 << 28)).to_le_bytes()));
        w.extend_from_slice(&1_234_567u64.to_le_bytes()); //  8..16 TSFT
        w.push(flags); //                                     16    FLAGS
        w.push(0); //                                         17    rt_pad
        w.extend_from_slice(&925u16.to_le_bytes()); //        18..20 CHANNEL
        w.extend_from_slice(&0x0140u16.to_le_bytes()); //     20..22
        w.extend_from_slice(&[0, 0]); //                      22..24 rt_pad2
        w.extend_from_slice(&32u16.to_le_bytes()); //         24..26 TLV type
        w.extend_from_slice(&6u16.to_le_bytes()); //          26..28 TLV length
        w.extend_from_slice(&0x007fu16.to_le_bytes()); //     28..30 known
        w.extend_from_slice(&(1u16 | (1 << 8) | ((mcs as u16) << 12)).to_le_bytes()); // data1
        w.extend_from_slice(&((rssi as u8 as u16) << 8).to_le_bytes()); //            data2
        assert_eq!(w.len(), 34);
        w.extend_from_slice(dot11);
        w.extend_from_slice(&[0xde, 0xad, 0xbe, 0xef]); // the appended FCS
        w
    }

    /// ★ The live defect this closes: with `F_FCS` set (which both HaLow drivers set on every data
    /// frame) the last four bytes of the buffer are the FCS, not payload. Before this they were
    /// delivered inside every HaLow NDN packet.
    #[test]
    fn fcs_is_stripped_from_the_payload_when_radiotap_says_it_is_there() {
        let fmt = FrameFormat::RawNdnS1g { ethertype: 0x8624 };
        let payload = b"\x05\x03interest";
        let dot11 = build_dot11(fmt, &frame(payload)).unwrap();
        let wire = nrc7292_capture(&dot11, 7, -46, 0x10);

        let got = parse(fmt, &wire, None, None, crate::ClockDomainId(9)).unwrap();
        assert_eq!(
            got.payload.as_ref(),
            payload,
            "the 4 FCS bytes must not appear in the NDN payload"
        );
        assert_eq!(got.addr, Some(SRC));
    }

    /// The same frame without the flag keeps every byte — the strip is driven by radiotap, not by
    /// an assumption about the bearer.
    #[test]
    fn no_fcs_flag_means_no_bytes_are_trimmed() {
        let fmt = FrameFormat::RawNdnS1g { ethertype: 0x8624 };
        let dot11 = build_dot11(fmt, &frame(b"\x05\x03abc")).unwrap();
        let wire = nrc7292_capture(&dot11, 0, -60, 0x00);
        let got = parse(fmt, &wire, None, None, crate::ClockDomainId(9)).unwrap();
        assert_eq!(got.payload.as_ref(), b"\x05\x03abc\xde\xad\xbe\xef");
    }

    /// A frame the PHY says failed its checksum must be dropped, not delivered as good.
    #[test]
    fn bad_fcs_frames_are_dropped() {
        let fmt = FrameFormat::RawNdnS1g { ethertype: 0x8624 };
        let dot11 = build_dot11(fmt, &frame(b"\x05\x03abc")).unwrap();
        let wire = nrc7292_capture(&dot11, 0, -60, 0x10 | 0x40);
        assert!(parse(fmt, &wire, None, None, crate::ClockDomainId(9)).is_none());
    }

    /// The NRC7292 emits **no** `DBM_ANTSIGNAL` and **no** `MCS` field: both live only in the S1G
    /// TLV. Before the TLV walk this radio reported `rssi_dbm: None` and `mcs_index: None` on every
    /// frame it ever received.
    #[test]
    fn s1g_tlv_supplies_rssi_and_mcs_when_nothing_else_does() {
        let fmt = FrameFormat::RawNdnS1g { ethertype: 0x8624 };
        let dot11 = build_dot11(fmt, &frame(b"\x05\x03x")).unwrap();
        let wire = nrc7292_capture(&dot11, 6, -73, 0x10);
        let got = parse(fmt, &wire, None, None, crate::ClockDomainId(9)).unwrap();
        assert_eq!(got.rssi_dbm, Some(-73));
        assert_eq!(got.mcs_index, Some(6), "an S1G MCS — see the parse() doc");
        assert!(
            got.stamp.is_some(),
            "radiotap TSFT is present on every frame"
        );
    }

    /// ★ The empty field, pinned with its reason. An on-air run populated `rssi_dbm`, `mcs_index`
    /// and `stamp` **1489/1489** and `phy` **0**/1489, and the question was whether our parser was
    /// dropping something. It is not: radiotap as either HaLow driver emits it carries no SNR, no
    /// EVM, no CFO and not even a `DBM_ANTNOISE` byte to subtract, so there is nothing to put in
    /// `PhyMetrics`. This asserts the two halves together — everything the wire DOES carry is
    /// surfaced, and the one thing it does not is left honestly empty rather than synthesised from
    /// a per-peer average or a channel-survey noise floor (both real, both different quantities).
    #[test]
    fn phy_metrics_stay_none_because_this_wire_carries_no_snr_evm_or_cfo() {
        let fmt = FrameFormat::RawNdnS1g { ethertype: 0x8624 };
        let dot11 = build_dot11(fmt, &frame(b"\x05\x03x")).unwrap();
        let wire = nrc7292_capture(&dot11, 6, -73, 0x10);
        let got = parse(fmt, &wire, None, None, crate::ClockDomainId(9)).unwrap();
        // The positive control: the metadata this header really does carry is all present.
        assert!(got.rssi_dbm.is_some() && got.mcs_index.is_some() && got.stamp.is_some());
        // And the field the header does not carry is empty, on every arm of the parser.
        assert!(
            got.phy.is_none(),
            "the S1G TLV has no SNR/EVM/CFO and neither driver sets DBM_ANTNOISE — if this ever \
             becomes Some, it must be because a driver started emitting one, not because we \
             invented it"
        );
        // Out-of-band caller values fill rssi/mcs and still cannot fill `phy`: there is no seam
        // for it on this path at all, which is the honest statement of the gap.
        let got = parse(fmt, &wire, Some(-11), Some(2), crate::ClockDomainId(9)).unwrap();
        assert!(got.phy.is_none());
    }

    /// An out-of-band value the caller already has always wins over radiotap, and radiotap's
    /// standard fields always win over the TLV — the TLV is the last resort, not an override.
    #[test]
    fn caller_supplied_metadata_still_takes_precedence() {
        let fmt = FrameFormat::RawNdnS1g { ethertype: 0x8624 };
        let dot11 = build_dot11(fmt, &frame(b"\x05\x03x")).unwrap();
        let wire = nrc7292_capture(&dot11, 6, -73, 0x10);
        let got = parse(fmt, &wire, Some(-11), Some(2), crate::ClockDomainId(9)).unwrap();
        assert_eq!(got.rssi_dbm, Some(-11));
        assert_eq!(got.mcs_index, Some(2));
    }

    /// A frame shorter than its own FCS must be refused rather than panicking on the subtraction.
    #[test]
    fn a_frame_too_short_to_hold_an_fcs_is_refused() {
        let fmt = FrameFormat::RawNdnS1g { ethertype: 0x8624 };
        let wire = nrc7292_capture(&[], 0, -60, 0x10);
        // 4 bytes of "FCS" and nothing else: the strip leaves an empty body, which is not a frame.
        assert!(parse(fmt, &wire, None, None, crate::ClockDomainId(9)).is_none());
        // And a buffer that stops inside the radiotap header is refused earlier still.
        assert!(parse(fmt, &wire[..20], None, None, crate::ClockDomainId(9)).is_none());
    }

    #[test]
    fn parse_populates_stamp_from_radiotap_tsft() {
        let fmt = FrameFormat::RawNdn { ethertype: 0x8624 };
        let dot11 = build_dot11(fmt, &frame(b"\x05\x03abc")).unwrap();
        // Hand-build a radiotap header carrying only a TSFT (bit 0), then the
        // 802.11 frame — the shape a monitor NIC delivers.
        let tsft: u64 = 0xdead_beef_0000_0001;
        let mut wire = vec![0u8, 0]; // version, pad
        let present: u32 = 1 << 0;
        let it_len = (8 + 8) as u16; // header + one 8-byte TSFT field
        wire.extend_from_slice(&it_len.to_le_bytes());
        wire.extend_from_slice(&present.to_le_bytes());
        wire.extend_from_slice(&tsft.to_le_bytes());
        wire.extend_from_slice(&dot11);

        let domain = crate::ClockDomainId(42);
        let stamp = parse(fmt, &wire, None, None, domain)
            .unwrap()
            .stamp
            .expect("a TSFT header must yield a hardware stamp");
        assert_eq!(stamp.raw, tsft, "raw counter preserved");
        assert_eq!(stamp.domain, domain, "the NIC's clock domain is carried");
        assert_eq!(stamp.latch, LatchPoint::MacDone);
        assert_eq!(
            stamp.precision_ns, 1_000,
            "clamped to the MacDone ~1µs floor"
        );

        // A frame built with an ordinary TX radiotap header (no TSFT) is
        // honestly unstamped.
        let plain = build(fmt, &frame(b"\x05\x03abc")).unwrap();
        assert!(
            parse(fmt, &plain, None, None, domain)
                .unwrap()
                .stamp
                .is_none(),
            "no TSFT => no stamp"
        );
    }

    #[test]
    fn espnow_round_trips() {
        let fmt = FrameFormat::EspNow { oui: ESPNOW_OUI };
        let payload = b"\x64\x0fNDN-LP-over-ESPNOW";
        let wire = build(fmt, &frame(payload)).unwrap();
        let got = parse(fmt, &wire, Some(-40), None, crate::ClockDomainId(0)).unwrap();
        assert_eq!(got.payload.as_ref(), payload.as_slice());
        assert_eq!(got.addr, Some(SRC));
    }

    /// §2 doctrine: the ephemeral source nonce is a locally-administered *individual* tag (U/L=local,
    /// I/G=individual), never a host MAC; it is stable within a rotation period, rotates across
    /// periods, and differs across boots — so it can attribute per-frame RSSI without being an identity.
    #[test]
    fn ephemeral_source_is_local_individual_and_rotates() {
        let src = EphemeralSource::new(0xDEAD_BEEF, 1000); // 1 s rotation
        let a = src.current(0);
        // Locally administered (not a vendor MAC) + individual (a source, not multicast).
        assert_eq!(
            a[0] & 0x02,
            0x02,
            "U/L local bit set — not a globally-unique host MAC"
        );
        assert_eq!(
            a[0] & 0x01,
            0x00,
            "I/G individual bit clear — a source address"
        );
        // Stable within a period; rotates across periods.
        assert_eq!(a, src.current(999), "stable within one rotation period");
        assert_ne!(a, src.current(1000), "rotates into the next period");
        // Different boot seed → different nonce (per-boot randomness, no persistent identity).
        assert_ne!(
            a,
            EphemeralSource::new(0x1234_5678, 1000).current(0),
            "differs across boots"
        );
        // No rotation when the period is 0 (still per-boot random, just fixed for the boot).
        let fixed = EphemeralSource::new(7, 0);
        assert_eq!(
            fixed.current(0),
            fixed.current(1_000_000),
            "period 0 → one nonce for the boot"
        );
    }

    /// SipHash-2-4 correctness against the reference vector (Aumasson & Bernstein):
    /// key = 00..0f, data = 00..0e (15 bytes) → 0xa129ca6149be45e5. Verifies the
    /// vendored primitive rather than trusting it.
    #[test]
    fn siphash24_reference_vector() {
        let key: [u8; 16] = core::array::from_fn(|i| i as u8);
        let data: [u8; 15] = core::array::from_fn(|i| i as u8);
        assert_eq!(siphash24(&key, &data), 0xa129_ca61_49be_45e5);
    }

    /// The injected bytes after radiotap must be a well-formed ESP-NOW vendor
    /// action frame (what a stock esp-wifi ESP-NOW receiver keys on).
    #[test]
    fn espnow_wire_layout_is_canonical() {
        let fmt = FrameFormat::EspNow { oui: ESPNOW_OUI };
        let wire = build(fmt, &frame(b"hi")).unwrap();
        let rt = radiotap::TX_LEGACY_HEADER_LEN;
        let b = &wire[rt..];
        assert_eq!(&b[0..2], &[0xd0, 0x00], "Action frame control");
        assert_eq!(&b[24], &ESPNOW_CATEGORY, "vendor-specific category");
        assert_eq!(&b[25..28], &ESPNOW_OUI, "action OUI");
        assert_eq!(b[32], ESPNOW_ELEMENT_ID, "vendor element id");
        assert_eq!(b[33] as usize, 5 + 2, "element length = OUI+Type+Ver+body");
        assert_eq!(&b[34..37], &ESPNOW_OUI, "element OUI");
        assert_eq!(b[37], ESPNOW_TYPE);
        assert_eq!(b[38], ESPNOW_VERSION);
        assert_eq!(&b[39..41], b"hi", "ESP-NOW body");
    }

    /// The radiotap-free `build_dot11`/`parse_dot11` helpers (used by hardware
    /// backends that carry the rate in their own TX/RX descriptor) round-trip,
    /// and `build` is exactly `radiotap ++ build_dot11`.
    #[test]
    fn dot11_helpers_round_trip_and_compose_build() {
        let fmt = FrameFormat::EspNow { oui: ESPNOW_OUI };
        let f = frame(b"\x05\x05hello");
        let dot11 = build_dot11(fmt, &f).unwrap();
        // No radiotap prefix — starts at the 802.11 Action frame control.
        assert_eq!(&dot11[0..2], &[0xd0, 0x00]);
        let got = parse_dot11(fmt, &dot11, Some(-33), Some(7), None).unwrap();
        assert_eq!(got.payload.as_ref(), b"\x05\x05hello");
        assert_eq!(got.addr, Some(SRC));
        assert_eq!(got.rssi_dbm, Some(-33), "descriptor RSSI passes through");
        assert_eq!(got.mcs_index, Some(7), "descriptor rate passes through");
        // `build` == radiotap header ++ the same 802.11 frame.
        assert!(build(fmt, &f).unwrap().ends_with(&dot11));
    }

    /// Raw80211 injects the payload verbatim (after radiotap) and recovers the
    /// whole 802.11 frame on parse, with addr2/addr1 surfaced from the fixed
    /// header — the path the userspace NAN stack uses for management frames.
    #[test]
    fn raw80211_passes_the_whole_frame_through() {
        let fmt = FrameFormat::Raw80211;
        // A fabricated NAN-beacon-shaped 802.11 frame: FC=80 00, dur, addr1..3, seq.
        let mut frame_bytes = vec![0x80, 0x00, 0x00, 0x00];
        frame_bytes.extend_from_slice(&BROADCAST); // addr1
        frame_bytes.extend_from_slice(&SRC); // addr2
        frame_bytes.extend_from_slice(&[0x50, 0x6F, 0x9A, 0x01, 0x00, 0x00]); // addr3
        frame_bytes.extend_from_slice(&[0x00, 0x00]); // seq
        frame_bytes.extend_from_slice(b"nan-attributes-here");

        let inj = InjectFrame {
            payload: Bytes::from(frame_bytes.clone()),
            tx: TxIntent::CONSERVATIVE,
            dst: BROADCAST,
            src: SRC,
            addr3: None,
            extra: None,
            htc: None,
        };
        // build_dot11 is the identity on the payload (no extra framing).
        assert_eq!(build_dot11(fmt, &inj).unwrap(), frame_bytes);

        let got = parse(
            fmt,
            &build(fmt, &inj).unwrap(),
            Some(-60),
            Some(0),
            crate::ClockDomainId(0),
        )
        .unwrap();
        assert_eq!(
            got.payload.as_ref(),
            &frame_bytes[..],
            "whole frame preserved"
        );
        assert_eq!(got.addr, Some(SRC), "addr2 surfaced");
        assert_eq!(got.group, Some(BROADCAST), "addr1 surfaced");
        assert_eq!(got.rssi_dbm, Some(-60));
    }

    /// The 190-bit filter frame is a 4-address QoS-Data+HT-Control MPDU whose **extra region is
    /// split by exactly one wire mapping** — `extra[0..6] → addr4`, `extra[6..8] → QoS Control` —
    /// round-trips through `parse_dot11`, AND stays readable by a base receiver: the base 126-bit
    /// Blur in addr1‖addr2‖addr3 is byte-identical whether or not the extra region is present, so a
    /// 190-bit sender and a commodity base-only receiver share one airspace with zero false
    /// negatives. That last property is the entire mid-upgrade story; it is asserted here.
    #[test]
    fn the_190_bit_filter_frame_round_trips_and_stays_base_readable() {
        let fmt = FrameFormat::RawNdn { ethertype: 0x8624 };
        // 8 extra bytes: the first 6 land in addr4, the last 2 in QoS Control. The QoS pattern is
        // deliberately one a MAC could not itself produce (A-MSDU-Present clear would be 0x00,
        // TID 0 would be 0x00) so a chip that rewrote the field is visible, per #96's rule.
        let extra = [0x54, 0x02, 0x88, 0x92, 0x6a, 0x10, 0xa5, 0x5a];
        let htc = [0xd0, 0x38, 0x4e, 0x03]; // fp=0x4e38d0 LE ‖ region bitmap 0x03 (addr4|QoS)
        let addr3 = [0x00, 0xc0, 0x81, 0x00, 0x37, 0x00];
        let wide = InjectFrame {
            payload: Bytes::copy_from_slice(b"\x05\x03abc"),
            tx: TxIntent::CONSERVATIVE,
            dst: [0x03, 0x80, 0x84, 0x00, 0x01, 0x00],
            src: [0x08, 0x00, 0x81, 0x00, 0x05, 0x01],
            addr3: Some(addr3),
            extra: Some(extra),
            htc: Some(htc),
        };
        let dot11 = build_dot11(fmt, &wide).unwrap();
        // Wire header pins: FC = QoS-Data + ToDS+FromDS+Order; then the 36-byte header.
        assert_eq!(
            &dot11[0..2],
            &[0x88, 0x83],
            "FC: QoS-Data, ToDS=FromDS=Order=1"
        );
        assert_eq!(&dot11[4..10], &wide.dst, "addr1 = base Blur hi");
        assert_eq!(&dot11[10..16], &wide.src, "addr2 = base Blur mid");
        assert_eq!(&dot11[16..22], &addr3, "addr3 = base[12:16]‖id‖flags");
        assert_eq!(
            &dot11[24..30],
            &extra[0..6],
            "addr4 = extra Blur bits 0..48"
        );
        assert_eq!(
            &dot11[30..32],
            &extra[6..8],
            "QoS Control = extra Blur bits 48..64 — the free 16 bits, no longer zero"
        );
        assert_eq!(
            &dot11[32..36],
            &htc,
            "HT Control = fingerprint ‖ region bitmap"
        );
        assert_eq!(&dot11[36..42], &LLC_SNAP_PREFIX, "LLC/SNAP at offset 36");

        // Full-fidelity parse recovers the extra region, reassembled across both fields.
        let got = parse_dot11(fmt, &dot11, Some(-42), Some(5), None).unwrap();
        assert_eq!(got.payload.as_ref(), b"\x05\x03abc");
        assert_eq!(got.group, Some(wide.dst));
        assert_eq!(got.addr, Some(wide.src));
        assert_eq!(got.addr3, Some(addr3));
        assert_eq!(got.extra, Some(extra), "64-bit extra Blur surfaced");
        assert_eq!(got.htc, Some(htc), "fingerprint + region bitmap surfaced");

        // Base coexistence: a base 3-address frame with the SAME addr1/2/3 yields identical
        // addr1‖addr2‖addr3 bytes — the Blur a base receiver ANDs its masks against is unchanged.
        let base = InjectFrame {
            extra: None,
            htc: None,
            ..wide.clone()
        };
        let base11 = build_dot11(fmt, &base).unwrap();
        assert_eq!(
            &base11[4..22],
            &dot11[4..22],
            "base Blur bytes identical whether or not the extra region rides along"
        );
        let base_got = parse_dot11(fmt, &base11, None, None, None).unwrap();
        assert_eq!(base_got.extra, None, "base frame carries no extra Blur");
        assert_eq!(base_got.htc, None, "base frame carries no fingerprint");
    }

    #[test]
    fn espnow_rejects_oversize_body() {
        let fmt = FrameFormat::EspNow { oui: ESPNOW_OUI };
        assert!(build(fmt, &frame(&[0u8; 251])).is_err());
    }

    #[test]
    fn formats_do_not_cross_parse() {
        let raw = FrameFormat::RawNdn { ethertype: 0x8624 };
        let esp = FrameFormat::EspNow { oui: ESPNOW_OUI };
        let raw_wire = build(raw, &frame(b"x")).unwrap();
        assert!(parse(esp, &raw_wire, None, None, crate::ClockDomainId(0)).is_none());
        let esp_wire = build(esp, &frame(b"x")).unwrap();
        assert!(parse(raw, &esp_wire, None, None, crate::ClockDomainId(0)).is_none());
    }
}

#[cfg(test)]
mod tier0_wire_cost {
    use super::*;

    /// ☠ **Tier-0's marginal wire cost is ZERO bytes — not the 12 bytes/frame the comparison
    /// tables claim**, and that error is what makes the filter look like it might not pay.
    ///
    /// An 802.11 data frame carries addr1‖addr2‖addr3 unconditionally: `build_dot11` emits addr3 as
    /// `addr3.unwrap_or(dst)` whether or not a filter is present. So the filter does not ADD bytes,
    /// it RECYCLES bytes the frame must carry regardless — which under the no-host-identity doctrine
    /// would otherwise hold a broadcast address and a nonce.
    #[test]
    fn the_filter_costs_no_additional_airtime() {
        let mk = |dst: [u8; 6], src: [u8; 6], a3: Option<[u8; 6]>| InjectFrame {
            payload: bytes::Bytes::from_static(b"x"),
            tx: Default::default(),
            dst,
            src,
            addr3: a3,
            extra: None,
            htc: None,
        };
        // ☠ **This test was VACUOUS in its first form and a sibling harness caught it.** It used
        // `FrameFormat::Raw80211`, which is a payload PASSTHROUGH — it builds no 802.11 header, so
        // both arms were trivially 32 B and the assertion could not have failed. Measuring an
        // invariant with an instrument that cannot see it is not evidence, which is the same error
        // as reading `Duration = 0` off broadcast frames whose correct duration is 0.
        //
        // `RawNdn` is the format that actually emits the MAC header.
        let plain = build_dot11(
            FrameFormat::RawNdn { ethertype: 0x8624 },
            &mk([0xff; 6], [0xaa; 6], None),
        )
        .unwrap();
        let tier0 = build_dot11(
            FrameFormat::RawNdn { ethertype: 0x8624 },
            &mk([0x01; 6], [0x02; 6], Some([0x03; 6])),
        )
        .unwrap();
        assert_eq!(
            plain.len(),
            tier0.len(),
            "same frame length ⇒ same airtime: the BASE filter is recycled address bytes, not added \
             ones (24 B header either way)"
        );
        // ⚠ The extra region is NOT free: addr4 + QoS Control + HT Control take the header
        // 24 B → 36 B, so the 190-bit filter costs 12 bytes/frame of real airtime. The recycling
        // argument covers the base region only — do not extend it to the extra region.
        //
        // ★ But the LAST 16 of those 190 bits ARE free: the frame is already QoS-Data and already
        // emitted two bytes there, so 190 costs exactly what 174 cost. Asserted, not assumed.
        let mk_wide = |extra: [u8; 8]| {
            build_dot11(
                FrameFormat::RawNdn { ethertype: 0x8624 },
                &InjectFrame {
                    extra: Some(extra),
                    htc: Some([0x05; 4]),
                    ..mk([0x01; 6], [0x02; 6], Some([0x03; 6]))
                },
            )
            .unwrap()
        };
        let wide = mk_wide([0x04; 8]);
        assert!(
            wide.len() > tier0.len(),
            "the extra region DOES cost airtime; only the base is free"
        );
        assert_eq!(
            wide.len() - tier0.len(),
            12,
            "12 B of pushed header, the MEASURED cost (P7 S7)"
        );
        assert_eq!(
            mk_wide([0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x00, 0x00]).len(),
            wide.len(),
            "the QoS Control bytes are emitted either way — the last 16 bits are FREE, which is why \
             190 and 174 have identical airtime"
        );
    }
}
