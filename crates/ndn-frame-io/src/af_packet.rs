//! Linux `AF_PACKET` `SOCK_RAW` backend: raw 802.11 injection and capture on a
//! monitor-mode interface.
//!
//! Unlike the Ethernet face (`ndn-face-native`'s `SOCK_DGRAM` + TPACKET ring,
//! where the kernel builds/strips the link header), monitor mode hands us the
//! *whole* frame: we prepend the [`radiotap`](crate::radiotap) TX header (which
//! names the MCS) and the 802.11 + LLC/SNAP headers ourselves, and on RX we
//! strip the radiotap header the driver prepended and the 802.11 header to
//! recover the NDN payload.
//!
//! Requires `CAP_NET_RAW` and an interface already in monitor mode
//! (`iw dev <if> set monitor none` / `ip link set <if> up`). Bringing the
//! interface into monitor mode is an operator/config step, not this backend's
//! job.
//!
//! # Split netdevs: when TX and RX are not the same interface
//!
//! [`new`](AfPacketBackend::new) binds one interface for both directions, which is what every
//! mac80211 monitor vif wants. The **Morse Micro MM6108 cannot do that**, and it is not a
//! preference — it is how the driver is built. `morse_mac_skb_recv` diverts *all* receive to the
//! driver's own global sniffer netdev `morse0` the moment monitor mode is on (`ieee80211_rx()` is
//! never called, so mac80211 — and therefore any `mon0` — sees nothing), while `morse_mon_xmit`
//! on that same `morse0` is `/* TODO: allow packet injection */ dev_kfree_skb(skb)`: a `sendto()`
//! there succeeds at the syscall and radiates nothing. So injection must go out on a mac80211
//! monitor vif and capture must come in on `morse0`. [`split`](AfPacketBackend::split) expresses
//! exactly that, and `ndn_radio_drivers::morse::MorseFrameIo` adds the guards that stop a caller
//! wiring it up the fatal way round.
//!
//! The TX-only socket is opened with **protocol 0**, so the kernel never queues received packets
//! on it (`packet(7)`: a socket bound with a protocol of zero receives no packets). Without that,
//! a second socket on a busy monitor interface would fill a receive buffer nobody ever drains.
//!
//! **Compile-verified on Linux only.** The platform-neutral core (radiotap
//! codec + loopback bus) is exercised by the crate's unit tests on every host.

// Raw-socket FFI boundary — the one module in this crate allowed to use
// `unsafe` under the workspace `deny(unsafe_code)` policy.
#![allow(unsafe_code)]

use std::os::unix::io::{AsRawFd, FromRawFd, OwnedFd, RawFd};

use std::time::Duration;

use async_trait::async_trait;
use ndn_transport::FaceError;
use tokio::io::unix::AsyncFd;

use crate::{CapturedFrame, DEFAULT_AMSDU_BODY, FrameFormat, InjectFrame};

const ETH_P_ALL: u16 = 0x0003;

/// Raw-802.11 monitor-mode injection/capture over one interface. The 802.11
/// addresses come from each [`InjectFrame`] (name-derived or default), so the
/// backend holds no source identity.
///
/// # ⚠ Last resort. Prefer a libusb backend for any node the MAC is expected to run on.
///
/// This backend is kernel-mediated: `mac80211` owns the decisions the named-data MAC exists to
/// make. That is not a missing feature list, it is the shape of the thing, and it has cost real
/// debugging time on real deployments. **Use it when there is no alternative** — a chip we have no
/// userspace driver for, a kernel-managed NIC that must stay associated, a quick capture — and
/// **say so in the run notes**, because a result taken over af-packet is not comparable with one
/// taken over a libusb backend.
///
/// What it structurally cannot do, each MEASURED or read from the tree:
///
/// * **No knobs.** `OpenRadio.knobs` is `None` on this path, so TX power, contention/EDCA, channel
///   width and the airtime lease have **no actuator**. Cognition's decisions are computed and then
///   dropped — this workspace's characteristic "decided but unactuated" shape. Every
///   `NDN_RADIO_*` tuning variable is libusb-only.
/// * **No per-frame Duration/NAV.** `ieee80211_duration()` recomputes `duration_id` in software on
///   the way down. MEASURED 2026-09-03: an injected `0x1234` came out 314 µs on ath9k_htc and 0 on
///   mt76x0. (The 190-bit Tier-0 filter is unaffected — it rides `addr4 ‖ QoS Control`, which does
///   survive verbatim, 1700/1700 and 1300/1300 frames.)
/// * **No µs common view.** No per-frame hardware TX timestamp, so a node on this backend cannot
///   source the slotted MAC's clock and cannot be a slot owner.
/// * **The kernel owns retry, aggregation and sequence numbering**, so airtime accounting and the
///   lease are advisory at best.
/// * ☠ **It is in a different crate from the coverage ratchet**, and that has already bitten: the
///   worst-receiver rate rule (doctrine §5) was taught to ten backends and missed this one, which
///   produced a **one-way link on a live deployment** — see the `inject` body below and
///   `tests/worst_receiver_rate.rs`, which now enforces the rule across every backend.
/// * ☠ **Forbidden for USB mt76 parts** — the kernel driver's `disconnect` stops the MCU; use
///   `scripts/mt76_acquire.sh` and the libusb backend instead.
pub struct AfPacketBackend {
    socket: AsyncFd<OwnedFd>,
    ifindex: i32,
    /// A separate transmit socket when TX and RX are different netdevs (see
    /// [`split`](Self::split)); `None` means `socket` carries both directions.
    tx_socket: Option<AsyncFd<OwnedFd>>,
    /// Interface index injection is addressed to — equal to `ifindex` unless split.
    tx_ifindex: i32,
    format: FrameFormat,
    /// Advertised capability. `AF_PACKET` wraps an arbitrary kernel NIC, so this is not known
    /// from the socket; a conservative placeholder by default, overridable with
    /// [`with_capability`](Self::with_capability) by a caller that knows its NIC (or, in future,
    /// auto-filled from an nl80211 `NL80211_CMD_GET_WIPHY` query).
    capability: crate::RadioCapability,
    /// Current transmit rate as state ([`crate::FrameIo::set_rate`]); `None` ⇒ the
    /// radiotap header resolves the frame's intent. Retires per-frame `inject_at`.
    cur_mcs: std::sync::Mutex<Option<crate::McsDescriptor>>,
    /// A-MSDU body budget for [`inject_batch_at`](Self::inject_batch_at), in bytes.
    /// [`DEFAULT_AMSDU_BODY`] unless a caller that knows its radio's real MPDU ceiling narrows it
    /// with [`with_amsdu_cap`](Self::with_amsdu_cap).
    amsdu_cap: usize,
}

impl AfPacketBackend {
    /// Open a `SOCK_RAW` `AF_PACKET` socket bound to monitor-mode interface
    /// `iface`, wrapping payloads per `format`.
    pub fn new(iface: &str, format: FrameFormat) -> std::io::Result<Self> {
        Self::split(iface, iface, format)
    }

    /// Open a backend that **transmits on `tx_iface` and receives on `rx_iface`**.
    ///
    /// When the two names are equal this is exactly [`new`](Self::new) — one socket, one ifindex.
    /// When they differ, two sockets are opened: the RX one bound to `ETH_P_ALL` with an enlarged
    /// receive buffer, the TX one with **protocol 0** so the kernel queues nothing on it. See the
    /// module docs for the radio that forces this (the MM6108).
    ///
    /// The clock domain, and therefore every [`crate::LinkStamp`] this backend produces, is keyed
    /// on the **receive** interface — stamps come from the receiving netdev's TSF, so pinning the
    /// domain to the transmit side would relate two clocks that never met.
    pub fn split(tx_iface: &str, rx_iface: &str, format: FrameFormat) -> std::io::Result<Self> {
        let rx_ifindex = ifindex_of(rx_iface)?;
        let rx_fd = open_packet_socket(rx_ifindex, ETH_P_ALL)?;

        // Enlarge the socket receive buffer so a fast on-air burst isn't dropped
        // between userspace reads (the default is small; a monitor sees every frame).
        //
        // ⚠ The kernel clamps the request to `net.core.rmem_max`, and it can fail outright —
        // so the result is checked rather than discarded. A silently-ignored `setsockopt` is
        // exactly the "returned success, actuated nothing" shape this crate is scarred by.
        //
        // `NDN_AF_RCVBUF` overrides the size, and `0` skips the call entirely. That escape hatch
        // exists because this one line is the only thing this backend does to its receive socket
        // that a stock `tcpdump` does not, which makes it the first thing to A/B when a monitor
        // netdev delivers to `tcpdump` and not to us.
        let rcvbuf: libc::c_int = std::env::var("NDN_AF_RCVBUF")
            .ok()
            .and_then(|v| v.parse().ok())
            .unwrap_or(4 * 1024 * 1024);
        if rcvbuf > 0 {
            let rc = unsafe {
                libc::setsockopt(
                    rx_fd.as_raw_fd(),
                    libc::SOL_SOCKET,
                    libc::SO_RCVBUF,
                    &rcvbuf as *const libc::c_int as *const libc::c_void,
                    std::mem::size_of::<libc::c_int>() as libc::socklen_t,
                )
            };
            if rc != 0 {
                eprintln!(
                    "!! SO_RCVBUF({rcvbuf}) not applied on {rx_iface}: {} — a fast on-air burst \
                     may be dropped between reads",
                    std::io::Error::last_os_error()
                );
            }
        }

        let (tx_ifindex, tx_socket) = if tx_iface == rx_iface {
            (rx_ifindex, None)
        } else {
            let idx = ifindex_of(tx_iface)?;
            // Protocol 0: a TX-only packet socket. Anything else would silently accumulate every
            // frame on the air in a buffer nothing reads.
            (idx, Some(AsyncFd::new(open_packet_socket(idx, 0)?)?))
        };

        Ok(Self {
            socket: AsyncFd::new(rx_fd)?,
            ifindex: rx_ifindex,
            tx_socket,
            tx_ifindex,
            format,
            // Placeholder until the caller overrides / an nl80211 query fills it in.
            capability: crate::RadioCapability::wifi_monitor_5ghz(vec![
                36, 40, 44, 48, 149, 153, 157, 161,
            ]),
            cur_mcs: std::sync::Mutex::new(None),
            amsdu_cap: DEFAULT_AMSDU_BODY,
        })
    }

    /// Override the advertised [`RadioCapability`] — a caller that knows the wrapped NIC (its
    /// band(s), rates, channels) supplies the real profile instead of the conservative default.
    pub fn with_capability(mut self, capability: crate::RadioCapability) -> Self {
        self.capability = capability;
        self
    }

    /// Narrow the A-MSDU body budget to what this radio's MAC will actually accept.
    ///
    /// ⚠ **The default is an 802.11n/ac number and it overshoots on 802.11ah.** On the MM6108 the
    /// on-air cutoff is byte-exact and MEASURED at **1546 B of payload / 1584 B of MPDU** — 1546
    /// delivers, 1547 never arrives — so a 3839 B aggregate is discarded by the chip with no error
    /// anywhere. That is the same silent-drop shape as the 2296 → 2272 MTU bug this crate already
    /// documents, so a HaLow caller must set this. Clamped to at least one small MSDU.
    pub fn with_amsdu_cap(mut self, bytes: usize) -> Self {
        self.amsdu_cap = bytes.max(64);
        self
    }

    /// The A-MSDU body budget in force (bytes).
    pub fn amsdu_cap(&self) -> usize {
        self.amsdu_cap
    }

    /// The interface index frames are captured on — the clock domain key.
    pub fn rx_ifindex(&self) -> i32 {
        self.ifindex
    }
}

/// Resolve an interface name to its index, mapping "no such interface" to a real error rather
/// than to a zero that would later address the wrong netdev.
fn ifindex_of(iface: &str) -> std::io::Result<i32> {
    let cname = std::ffi::CString::new(iface)
        .map_err(|_| std::io::Error::new(std::io::ErrorKind::InvalidInput, "iface has NUL"))?;
    let idx = unsafe { libc::if_nametoindex(cname.as_ptr()) };
    if idx == 0 {
        return Err(std::io::Error::new(
            std::io::ErrorKind::NotFound,
            format!("no such interface: {iface}"),
        ));
    }
    Ok(idx as i32)
}

/// Open a non-blocking `AF_PACKET`/`SOCK_RAW` socket bound to `ifindex`.
///
/// `protocol` is in host byte order and converted here; pass `0` for a transmit-only socket (the
/// kernel then delivers nothing to it) and `ETH_P_ALL` for a capture socket.
fn open_packet_socket(ifindex: i32, protocol: u16) -> std::io::Result<OwnedFd> {
    let be = protocol.to_be();
    let fd = unsafe {
        libc::socket(
            libc::AF_PACKET,
            libc::SOCK_RAW | libc::SOCK_NONBLOCK | libc::SOCK_CLOEXEC,
            be as i32,
        )
    };
    if fd == -1 {
        return Err(std::io::Error::last_os_error());
    }
    let owned = unsafe { OwnedFd::from_raw_fd(fd) };
    let mut addr: libc::sockaddr_ll = unsafe { std::mem::zeroed() };
    addr.sll_family = libc::AF_PACKET as u16;
    addr.sll_protocol = be;
    addr.sll_ifindex = ifindex;
    if unsafe {
        libc::bind(
            owned.as_raw_fd(),
            &addr as *const libc::sockaddr_ll as *const libc::sockaddr,
            std::mem::size_of::<libc::sockaddr_ll>() as libc::socklen_t,
        )
    } == -1
    {
        return Err(std::io::Error::last_os_error());
    }
    Ok(owned)
}

impl AfPacketBackend {
    /// Send pre-built bytes (radiotap ++ 802.11 ++ body) verbatim. For drivers that
    /// require a specific monitor-injection format (e.g. the rtl88x2eu cfg80211
    /// monitor path needs an exactly-14-byte radiotap + an 802.11 *Action* frame).
    pub async fn inject_raw(&self, buf: &[u8]) -> Result<(), FaceError> {
        self.send_buf(buf).await
    }

    /// Await one captured frame and return its **raw** bytes (`radiotap ++ 802.11 ++ …`),
    /// undecoded, into `buf`; returns how many bytes were written.
    ///
    /// [`recv_frame`](crate::FrameIo::recv_frame) drops anything `frame::parse` does not
    /// recognise — which is correct for the data plane and wrong for anything that needs to see
    /// the frames our own format filters out. The concrete case is the NRC7292's **S1G beacons**:
    /// they are 802.11 Extension frames, `parse` rightly refuses them as "not a data frame", and
    /// they are exactly the frames a common-view estimator wants (`nrc7292::s1g_beacon_offset`).
    /// A wrapper that needs both drives this and decides for itself.
    pub async fn recv_into(&self, buf: &mut [u8]) -> Result<usize, FaceError> {
        loop {
            let mut guard = self.socket.readable().await.map_err(FaceError::Io)?;
            let fd: RawFd = self.socket.get_ref().as_raw_fd();
            // try_io clears readiness on WouldBlock (so the next `.readable()`
            // re-registers with the edge-triggered epoll); a plain `recv` + manual
            // clear can wedge after the first packet on a busy monitor socket.
            let n = match guard.try_io(|_| {
                let n =
                    unsafe { libc::recv(fd, buf.as_mut_ptr() as *mut libc::c_void, buf.len(), 0) };
                if n < 0 {
                    Err(std::io::Error::last_os_error())
                } else {
                    Ok(n as usize)
                }
            }) {
                Ok(Ok(n)) => n,
                Ok(Err(e)) => return Err(FaceError::Io(e)),
                Err(_would_block) => continue,
            };
            return Ok(n);
        }
    }

    /// The frame format this backend wraps payloads in — so a wrapper that drives
    /// [`recv_into`](Self::recv_into) itself decodes with the same format.
    pub fn format(&self) -> FrameFormat {
        self.format
    }

    /// Overall deadline for one frame's send. A frame that cannot leave within
    /// this window (sustained backpressure) is dropped rather than stalling the
    /// shared TX task — for a broadcast radio a stale Interest/Data is worthless.
    const SEND_DEADLINE: Duration = Duration::from_secs(2);
    /// Cap on any single readiness wait. Bounds how long a *missing* EPOLLOUT
    /// edge (see [`send_buf`](Self::send_buf)) can delay a retry.
    const WRITABLE_POLL: Duration = Duration::from_millis(20);

    /// Send one pre-built frame (radiotap ++ 802.11 ++ body), tolerant of the
    /// AF_PACKET write-readiness quirk.
    ///
    /// Packet sockets do **not** reliably deliver an edge-triggered `EPOLLOUT`
    /// write-space wakeup when their send buffer drains, so parking on
    /// [`AsyncFd::writable`]`().await` after an `EAGAIN` can hang forever even
    /// though the buffer empties within seconds and a plain retry would succeed.
    /// (Root-caused 2026-08-25 with `examples/af_wedge_probe`: a bare socket
    /// drains in ~5 s while the async waiter never wakes — which silently wedged
    /// the whole radio face because one shared batcher task was blocked in this
    /// `.await`.) So we send optimistically first, and on `WouldBlock` we cap the
    /// readiness wait and retry the send regardless (level-triggered), giving up
    /// only after [`SEND_DEADLINE`](Self::SEND_DEADLINE).
    async fn send_buf(&self, buf: &[u8]) -> Result<(), FaceError> {
        // The transmit socket and its interface — the same as the receive one unless `split`.
        let tx = self.tx_socket.as_ref().unwrap_or(&self.socket);
        let mut dst: libc::sockaddr_ll = unsafe { std::mem::zeroed() };
        dst.sll_family = libc::AF_PACKET as u16;
        dst.sll_protocol = ETH_P_ALL.to_be();
        dst.sll_ifindex = self.tx_ifindex;

        let deadline = tokio::time::Instant::now() + Self::SEND_DEADLINE;
        loop {
            let fd: RawFd = tx.get_ref().as_raw_fd();
            let ret = unsafe {
                libc::sendto(
                    fd,
                    buf.as_ptr() as *const libc::c_void,
                    buf.len(),
                    0,
                    &dst as *const libc::sockaddr_ll as *const libc::sockaddr,
                    std::mem::size_of::<libc::sockaddr_ll>() as libc::socklen_t,
                )
            };
            if ret >= 0 {
                return Ok(());
            }
            let err = std::io::Error::last_os_error();
            if err.kind() != std::io::ErrorKind::WouldBlock {
                return Err(FaceError::Io(err));
            }
            // Backpressured. Wait for a writable edge, but never trust it alone:
            // cap the wait and retry the send anyway, so an omitted EPOLLOUT
            // cannot park us past the deadline.
            if tokio::time::Instant::now() >= deadline {
                return Err(FaceError::Io(err));
            }
            let _ = tokio::time::timeout(Self::WRITABLE_POLL, async {
                if let Ok(mut g) = tx.writable().await {
                    g.clear_ready();
                }
            })
            .await;
        }
    }
}

#[async_trait]
impl crate::FrameIo for AfPacketBackend {
    async fn inject(&self, frame: InjectFrame) -> Result<(), FaceError> {
        // Rate is state: build the radiotap header at the set MCS if present (the
        // kernel honours it), else resolve the frame's intent.
        //
        // ★ **A `MostRobust` frame ignores the stored rate** — cooperative reports, discovery and
        // control are exactly the traffic the *worst* receiver in range must decode, which is why
        // 802.11 sends beacons and probes at a basic rate. Every libusb backend already does this
        // (`rtl8812au::desc_rate_for`: `if intent.needs_basic_rate() { return DESC_RATE_6M }`).
        //
        // ☠ This backend did not, and it was a live production defect, reported from another
        // deployment: `ndn-fwd` attaches a rate actuator to **every** bearer including af-packet, so
        // `cur_mcs` is always `Some` in a real node — and from that moment every frame went at the
        // cognition MCS, `MostRobust` ones included. The observable symptom was a one-way link whose
        // only surviving traffic was the 1 Hz `/localhop/radio/report/<node>` cognition reports on
        // the *other* leg, whose radio did honour the intent. The fix that taught ten backends the
        // worst-receiver rule (doctrine §5) never reached this one, because it lives in a different
        // crate than the coverage ratchet walks.
        let buf = match *self.cur_mcs.lock().unwrap() {
            // `build` resolves `frame.tx` through `McsDescriptor::for_intent`, which is this
            // workspace's single definition of "most robust". ⚠ That is HT-MCS0 + STBC + LDPC, not
            // legacy 6 Mbps OFDM; on a peer whose HT demod is weak (MEASURED once on an 8812au at
            // 5 GHz) legacy is safer still, but changing `for_intent` would move every backend at
            // once and is a separate, witnessed decision.
            Some(_) if frame.tx.needs_basic_rate() => crate::frame::build(self.format, &frame)?,
            Some(mcs) => crate::frame::build_at(self.format, &frame, mcs)?,
            None => crate::frame::build(self.format, &frame)?,
        };

        self.send_buf(&buf).await
    }

    fn set_rate(&self, mcs: crate::McsDescriptor) -> Result<(), FaceError> {
        *self.cur_mcs.lock().unwrap() = Some(mcs);
        Ok(())
    }

    /// A-MSDU-aggregate a batch that carries **no per-frame rate**, by pinning it to the rate
    /// already set as bearer state — then it is the same aggregation as
    /// [`inject_batch_at`](Self::inject_batch_at).
    ///
    /// Both spellings exist because the two faces model rate differently: `MonitorWifiFace` resolves
    /// an MCS per frame and calls `inject_batch_at`, while `RadioMediumFace` holds rate as state in
    /// the bearer and sends `TxIntent`s, so it can only offer bare frames. Without this, moving
    /// A-MSDU down to the medium would have silently lost the aggregation on exactly the backend
    /// (AF_PACKET/S1G) where it matters most — the same class of quiet loss #82 part 2 fixed one
    /// layer up. Falls back to individual injection when no rate has been set yet.
    async fn inject_batch(&self, frames: Vec<InjectFrame>) -> Result<(), FaceError> {
        let Some(mcs) = *self.cur_mcs.lock().unwrap() else {
            for f in frames {
                self.inject(f).await?;
            }
            return Ok(());
        };
        self.inject_batch_at(frames.into_iter().map(|f| (f, mcs)).collect())
            .await
    }

    async fn recv_frame(&self) -> Result<CapturedFrame, FaceError> {
        let mut buf = [0u8; 4096];
        loop {
            let n = self.recv_into(&mut buf).await?;
            // A frame we can't decode (wrong format, foreign protocol) is
            // skipped, not an error — keep listening (readiness retained).
            // The TSFT clock domain is this NIC's — keyed by its ifindex, which
            // is unique per interface on this host.
            let domain = crate::ClockDomainId(self.ifindex as u32);
            if let Some(frame) = crate::frame::parse(self.format, &buf[..n], None, None, domain) {
                return Ok(frame);
            }
        }
    }

    /// A-MSDU-aggregate the batch — the actuator the face-level
    /// [`with_amsdu_batching`](../../../ndn_face_monitor_wifi/index.html) batcher
    /// drives (it calls `inject_batch_at`). One QoS-Data MPDU per destination
    /// (RA), greedily packed up to [`MAX_AMSDU_BODY`], all at the batch's rate:
    /// one PHY preamble for many NDN packets (the big lever at S1G). Each MSDU
    /// stays an independent NDN packet the receiver de-aggregates via
    /// [`parse_dot11`](crate::frame::parse_dot11), so PIT/FIB semantics are
    /// untouched. Non-`RawNdn`/`RawNdnS1g` formats fall back to the derived
    /// default (individual injection).
    ///
    /// **This override is the whole feature**, and it is reachable only through the trait object.
    /// It sat on `impl WifiRadio` until #82 part 2; when the face moved to `Arc<dyn FrameIo>` it
    /// became unreachable, and a caller that reimplements the default body instead — as #82 part 1
    /// did — loses the aggregation with no error and no log. It lives on `FrameIo` now for that
    /// reason. `ndn-face-monitor-wifi`'s `amsdu_batching_dispatches_to_the_backend_override` test
    /// guards the call.
    async fn inject_batch_at(
        &self,
        frames: Vec<(InjectFrame, crate::McsDescriptor)>,
    ) -> Result<(), FaceError> {
        match self.format {
            FrameFormat::RawNdn { .. } | FrameFormat::RawNdnS1g { .. } => {}
            _ => {
                for (f, mcs) in frames {
                    self.set_rate(mcs)?;
                    self.inject(f).await?;
                }
                return Ok(());
            }
        }
        if frames.is_empty() {
            return Ok(());
        }
        // One A-MSDU carries one MPDU rate; the batcher groups a run at one MCS.
        let mcs = frames[0].1;
        self.set_rate(mcs)?;

        // Group by RA (dst) preserving first-seen order — a broadcast face is one
        // group; split-addressed faces get one A-MSDU per destination (a single
        // MPDU has one RA, though each subframe still carries its own DA/SA).
        let mut order: Vec<[u8; 6]> = Vec::new();
        let mut groups: std::collections::HashMap<[u8; 6], Vec<([u8; 6], [u8; 6], bytes::Bytes)>> =
            std::collections::HashMap::new();
        for (f, _) in frames {
            let g = groups.entry(f.dst).or_default();
            if g.is_empty() {
                order.push(f.dst);
            }
            g.push((f.dst, f.src, f.payload));
        }

        for ra in order {
            let msdus = groups.remove(&ra).unwrap();
            let ta = msdus[0].1;
            // Greedily pack MSDUs into A-MSDUs bounded by MAX_AMSDU_BODY.
            let mut batch: Vec<([u8; 6], [u8; 6], bytes::Bytes)> = Vec::new();
            let mut acc = 0usize;
            for m in msdus {
                // Subframe on-air size: DA+SA+Len (14) + LLC/SNAP+ethertype (8) +
                // payload, 4-byte aligned.
                let sub = (14 + 8 + m.2.len() + 3) & !3;
                if !batch.is_empty() && acc + sub > self.amsdu_cap {
                    let buf = crate::frame::build_amsdu(self.format, ra, ta, &batch, 0, mcs)?;
                    self.inject_raw(&buf).await?;
                    batch.clear();
                    acc = 0;
                }
                acc += sub;
                batch.push(m);
            }
            if !batch.is_empty() {
                let buf = crate::frame::build_amsdu(self.format, ra, ta, &batch, 0, mcs)?;
                self.inject_raw(&buf).await?;
            }
        }
        Ok(())
    }
}

#[async_trait]

/// A monitor interface exposes the NIC's MAC TSF via radiotap TSFT (when the underlying driver
/// reports it), keyed by ifindex — a free-run per-frame RX-stamp clock. There is no read-now
/// clock over `AF_PACKET`, so `read_clock` stays the default `None`.
///
/// ★ **The reference is [`ClockReference::unknown`], and it is unknowable HERE by construction.**
/// This backend wraps an arbitrary NIC: the socket exposes an ifindex and a radiotap TSFT, and
/// nothing whatever about the oscillator the NIC's MAC counts. The frames of two very different
/// parts arrive through this same impl — the Morse MM6108 and the Newracom NRC7292 both do — and
/// they do not share a reference, so no single answer here could be true for both.
///
/// The consequence is deliberate: `FaceTimeProfile::can_common_view` is **false** for an
/// `AF_PACKET` face, where it used to be unconditionally true. That is not a lost capability, it is
/// a withdrawn claim; the honest way to get it back is for the specific radio behind the socket to
/// implement [`RadioTime`](crate::RadioTime) itself and say what it runs on — which is exactly what
/// `ndn_radio_drivers::nrc7292::Nrc7292Clock` does for the NRC7292, on the same ifindex domain so
/// the two agree.
impl crate::RadioTime for AfPacketBackend {
    fn time_sources(&self) -> Vec<crate::RadioTimeSource> {
        vec![
            crate::RadioTimeSource::free_run_rx_stamp(
                crate::ClockDomainId(self.ifindex as u32),
                1_000,
            )
            .with_reference(crate::ClockReference::unknown()),
        ]
    }
}

/// Returns the capability set at construction ([`with_capability`](AfPacketBackend::with_capability))
/// — a conservative 5 GHz placeholder by default, since `AF_PACKET` wraps an arbitrary NIC whose
/// real profile isn't visible from the socket (future: fill from an nl80211 wiphy query).
impl crate::RadioProfile for AfPacketBackend {
    fn capability(&self) -> crate::RadioCapability {
        self.capability.clone()
    }
}
