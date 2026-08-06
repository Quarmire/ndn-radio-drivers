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

/// Minimal radiotap header: version(1) pad(1) len(2) present(4) — no fields present, so the driver
/// picks its own rate and power. Enough for injection.
const RADIOTAP: [u8; 8] = [0x00, 0x00, 0x08, 0x00, 0x00, 0x00, 0x00, 0x00];

fn build_frame(seq: u16) -> Vec<u8> {
    let mut f = Vec::with_capacity(8 + 24 + 64);
    f.extend_from_slice(&RADIOTAP);

    // 802.11 data frame, ToDS=0 FromDS=0. Addresses are locally-administered group, matching the
    // named-radio doctrine (no host identity on the air).
    f.extend_from_slice(&[0x08, 0x00]); // frame control: data
    f.extend_from_slice(&[0x00, 0x00]); // duration
    f.extend_from_slice(&[0x03, 0x11, 0x22, 0x33, 0x44, 0x55]); // addr1
    f.extend_from_slice(&[0x03, 0x66, 0x77, 0x88, 0x99, 0xaa]); // addr2
    f.extend_from_slice(&[0x03, 0xbb, 0xcc, 0xdd, 0xee, 0xff]); // addr3
    f.extend_from_slice(&(seq << 4).to_le_bytes()); // sequence control
    f.extend_from_slice(&[0x42u8; 64]); // payload
    f
}

fn main() {
    let args: Vec<String> = std::env::args().collect();
    if args.len() < 2 {
        eprintln!("usage: {} <monitor-iface> [seconds]", args[0]);
        std::process::exit(1);
    }
    let ifname = &args[1];
    let secs: u64 = args.get(2).and_then(|s| s.parse().ok()).unwrap_or(10);

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
            let frame = build_frame(seq);
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
        }

        let el = start.elapsed().as_secs_f64();
        println!(
            "{ifname}: {sent} frames in {el:.2}s = {:.0} frames/s  (queue-full {eagain}, errors {errs})",
            sent as f64 / el
        );
        libc::close(fd);
    }
}
