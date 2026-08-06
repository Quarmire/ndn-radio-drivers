//! Saturating monitor-mode injector — measures how fast a radio will actually transmit.
//!
//! Used for M3: does arming the AR9271's hardware quiet-time schedule (`AR_QUIET1/2`) really gate
//! transmission? With a saturated queue the host cannot outrun the dongle — HTC credits provide the
//! backpressure — so the sustained injection rate tracks the on-air TX rate. Halving the airtime
//! available to the MAC should roughly halve this number, with no second radio required.
//!
//! ```sh
//! sudo ./mon_inject_rate wlu1u2 10
//! ```

use std::ffi::CString;
use std::time::{Duration, Instant};

/// Radiotap with an explicit RATE field (present bit 2), value in 500 kbps units.
///
/// The rate matters for more than throughput: it sets how finely this injector samples the medium.
/// At 1 Mbit/s a 96-byte frame plus overhead occupies ~1.1 ms, so a gap edge can only be located to
/// ±1.1 ms — useless for measuring a microsecond-scale scheduling boundary. At 54 Mbit/s the same
/// frame is ~14 µs on air, which is the resolution the measurement actually needs.
fn radiotap(rate_500kbps: u8) -> Vec<u8> {
    if rate_500kbps == 0 {
        // No fields present: the driver picks the rate.
        return vec![0x00, 0x00, 0x08, 0x00, 0x00, 0x00, 0x00, 0x00];
    }
    vec![
        0x00, 0x00, // version, pad
        0x09, 0x00, // length = 9
        0x04, 0x00, 0x00, 0x00, // present = bit 2 (RATE)
        rate_500kbps,
    ]
}

/// A bare 802.11 ACK: FC(2) + Duration(2) + RA(6) = 10 bytes, 14 on air once the hardware appends
/// the FCS. This is the frame Tier-0 structurally cannot filter — it carries one address, where the
/// filter needs 16 bytes of addr1||addr2. Synthesising it lets the ACK fraction of a channel be set
/// exactly, which is better than hoping a real station ACKs at the ratio you want.
fn build_ack(rate_500kbps: u8, ra: [u8; 6]) -> Vec<u8> {
    let mut f = Vec::new();
    f.extend_from_slice(&radiotap(rate_500kbps));
    f.extend_from_slice(&[0xd4, 0x00]); // FC: control / ACK
    f.extend_from_slice(&[0x00, 0x00]); // duration
    f.extend_from_slice(&ra);
    f
}

fn build_frame(seq: u16, rate_500kbps: u8, addr1: [u8; 6], dur: u16) -> Vec<u8> {
    let mut f = Vec::with_capacity(9 + 24 + 64);
    f.extend_from_slice(&radiotap(rate_500kbps));

    // 802.11 data frame, ToDS=0 FromDS=0. addr2/addr3 stay locally-administered group, matching the
    // named-radio doctrine (no host identity on the air). addr1 is overridable: a UNICAST addr1
    // makes the addressed station hardware-ACK, which is how a data+ACK channel is synthesised
    // using only radios we own. `dur` writes the Duration/ID field, which is the NAV announcement
    // §5 proposes to carry the airtime lease.
    f.extend_from_slice(&[0x08, 0x00]); // frame control: data
    f.extend_from_slice(&dur.to_le_bytes()); // duration / NAV
    f.extend_from_slice(&addr1);
    f.extend_from_slice(&[0x03, 0x66, 0x77, 0x88, 0x99, 0xaa]); // addr2
    f.extend_from_slice(&[0x03, 0xbb, 0xcc, 0xdd, 0xee, 0xff]); // addr3
    f.extend_from_slice(&(seq << 4).to_le_bytes()); // sequence control
    f.extend_from_slice(&[0x42u8; 64]); // payload
    f
}

fn main() {
    let args: Vec<String> = std::env::args().collect();
    if args.len() < 2 {
        eprintln!(
            "usage: {} <iface> [secs] [rate_500kbps] [dest-mac] [duration_us] [ack_pct] [gap_us]",
            args[0]
        );
        std::process::exit(1);
    }
    let ifname = &args[1];
    let secs: u64 = args.get(2).and_then(|s| s.parse().ok()).unwrap_or(10);
    // Third arg: PHY rate in 500 kbps units (108 = 54 Mbit/s). 0 = let the driver choose.
    let rate: u8 = args.get(3).and_then(|s| s.parse().ok()).unwrap_or(0);
    // Fourth arg: destination MAC "aa:bb:.." (unicast => the peer ACKs). Default = group address.
    let addr1: [u8; 6] = args
        .get(4)
        .and_then(|s| {
            let v: Vec<u8> = s.split(':').filter_map(|b| u8::from_str_radix(b, 16).ok()).collect();
            if v.len() == 6 { Some([v[0], v[1], v[2], v[3], v[4], v[5]]) } else { None }
        })
        .unwrap_or([0x03, 0x11, 0x22, 0x33, 0x44, 0x55]);
    // Fifth arg: Duration/ID in microseconds (NAV announcement). Default 0.
    let dur: u16 = args.get(5).and_then(|s| s.parse().ok()).unwrap_or(0);
    // Sixth arg: percentage of emitted frames that are bare ACKs (0-100).
    let ack_pct: u32 = args.get(6).and_then(|s| s.parse().ok()).unwrap_or(0);
    // Seventh arg: microseconds to sleep between frames. A NAV test needs LOW channel occupancy
    // with HIGH NAV coverage -- sparse frames carrying a long Duration. Without this the NAV-setter
    // simply hogs the medium by carrier sense and the result says nothing about NAV.
    let gap_us: u64 = args.get(7).and_then(|s| s.parse().ok()).unwrap_or(0);

    unsafe {
        // AF_PACKET/SOCK_RAW so the radiotap header is passed straight through to the driver.
        let fd = libc::socket(libc::AF_PACKET, libc::SOCK_RAW, (libc::ETH_P_ALL as u16).to_be() as i32);
        if fd < 0 {
            eprintln!("socket: {} (need root)", std::io::Error::last_os_error());
            std::process::exit(1);
        }

        let cname = CString::new(ifname.as_str()).unwrap();
        let idx = libc::if_nametoindex(cname.as_ptr());
        if idx == 0 {
            eprintln!("if_nametoindex({ifname}): {}", std::io::Error::last_os_error());
            std::process::exit(1);
        }

        let mut sa: libc::sockaddr_ll = std::mem::zeroed();
        sa.sll_family = libc::AF_PACKET as u16;
        sa.sll_protocol = (libc::ETH_P_ALL as u16).to_be();
        sa.sll_ifindex = idx as i32;
        if libc::bind(
            fd,
            &sa as *const _ as *const libc::sockaddr,
            std::mem::size_of::<libc::sockaddr_ll>() as u32,
        ) < 0
        {
            eprintln!("bind: {}", std::io::Error::last_os_error());
            std::process::exit(1);
        }

        let deadline = Instant::now() + Duration::from_secs(secs);
        let (mut sent, mut eagain, mut errs) = (0u64, 0u64, 0u64);
        let mut seq: u16 = 0;
        let start = Instant::now();

        while Instant::now() < deadline {
            let is_ack = ack_pct > 0 && (seq as u32 % 100) < ack_pct;
            let frame = if is_ack {
                build_ack(rate, addr1)
            } else {
                build_frame(seq, rate, addr1, dur)
            };
            seq = seq.wrapping_add(1);
            let n = libc::send(fd, frame.as_ptr() as *const libc::c_void, frame.len(), 0);
            if n < 0 {
                let e = std::io::Error::last_os_error();
                match e.raw_os_error() {
                    // The queue is full: the dongle is the bottleneck, which is the whole point.
                    Some(libc::EAGAIN) | Some(libc::ENOBUFS) => eagain += 1,
                    _ => {
                        errs += 1;
                        if errs < 4 {
                            eprintln!("send: {e}");
                        }
                    }
                }
            } else {
                sent += 1;
            }
            if gap_us > 0 {
                std::thread::sleep(Duration::from_micros(gap_us));
            }
        }

        let el = start.elapsed().as_secs_f64();
        println!(
            "{ifname}: {sent} frames in {el:.2}s = {:.0} frames/s  (queue-full {eagain}, errors {errs})",
            sent as f64 / el
        );
        libc::close(fd);
    }
}
