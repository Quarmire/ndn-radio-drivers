#![allow(unsafe_code)]
//! Root-cause probe for the radio-face TX wedge: reproduce it on a PLAIN
//! AF_PACKET socket with NO async runtime. If a bare blocking/nonblocking
//! socket fills its send buffer and never drains, the wedge is the kernel
//! socket send buffer (sk_wmem_alloc), and tokio's readiness tracking is
//! exonerated. Mirrors AfPacketBackend::new (SO_RCVBUF set, SO_SNDBUF NOT set).
//!
//!   sudo ./af_wedge_probe mon0
#[cfg(target_os = "linux")]
fn main() {
    use ndn_frame_io::{frame, FrameFormat};
    #[allow(unused_imports)]
    use ndn_radio_hal::{InjectFrame, TxIntent, DEFAULT_SRC};

    let iface = std::env::args().nth(1).unwrap_or_else(|| "mon0".into());
    let cname = std::ffi::CString::new(iface.clone()).unwrap();
    let ifindex = unsafe { libc::if_nametoindex(cname.as_ptr()) };
    assert!(ifindex != 0, "no such iface {iface}");

    const ETH_P_ALL: u16 = 0x0003;
    // Same as the backend: SOCK_RAW, but start BLOCKING (no O_NONBLOCK) so we
    // can flip modes later; no tokio anywhere.
    let fd = unsafe { libc::socket(libc::AF_PACKET, libc::SOCK_RAW, (ETH_P_ALL.to_be()) as i32) };
    assert!(fd >= 0, "socket: {}", std::io::Error::last_os_error());

    // Mirror the backend exactly: set SO_RCVBUF, DO NOT set SO_SNDBUF.
    let rcvbuf: libc::c_int = 4 * 1024 * 1024;
    unsafe {
        libc::setsockopt(fd, libc::SOL_SOCKET, libc::SO_RCVBUF,
            &rcvbuf as *const _ as *const libc::c_void, std::mem::size_of::<libc::c_int>() as u32);
    }
    // Report the effective send buffer.
    let mut sndbuf: libc::c_int = 0;
    let mut len = std::mem::size_of::<libc::c_int>() as libc::socklen_t;
    unsafe {
        libc::getsockopt(fd, libc::SOL_SOCKET, libc::SO_SNDBUF,
            &mut sndbuf as *mut _ as *mut libc::c_void, &mut len);
    }
    println!("effective SO_SNDBUF = {sndbuf} bytes");

    let mut addr: libc::sockaddr_ll = unsafe { std::mem::zeroed() };
    addr.sll_family = libc::AF_PACKET as u16;
    addr.sll_protocol = ETH_P_ALL.to_be();
    addr.sll_ifindex = ifindex as i32;
    let br = unsafe {
        libc::bind(fd, &addr as *const _ as *const libc::sockaddr,
            std::mem::size_of::<libc::sockaddr_ll>() as u32)
    };
    assert!(br == 0, "bind: {}", std::io::Error::last_os_error());

    // A realistic on-air frame — same builder + format the router's radio face uses.
    let inj = InjectFrame {
        payload: bytes::Bytes::from_static(&[0x5a; 1000]),
        tx: TxIntent::CONSERVATIVE,
        dst: [0xff; 6],
        src: DEFAULT_SRC,
        addr3: None,
    };
    let buf = frame::build(FrameFormat::RawNdnS1g { ethertype: 0x8624 }, &inj).expect("build");
    println!("frame = {} bytes on the wire", buf.len());

    let mut dst: libc::sockaddr_ll = unsafe { std::mem::zeroed() };
    dst.sll_family = libc::AF_PACKET as u16;
    dst.sll_protocol = ETH_P_ALL.to_be();
    dst.sll_ifindex = ifindex as i32;

    let send_once = |flags: libc::c_int| -> isize {
        unsafe {
            libc::sendto(fd, buf.as_ptr() as *const libc::c_void, buf.len(), flags,
                &dst as *const _ as *const libc::sockaddr,
                std::mem::size_of::<libc::sockaddr_ll>() as u32)
        }
    };
    let outq = || -> libc::c_int {
        let mut v: libc::c_int = -1;
        unsafe { libc::ioctl(fd, libc::TIOCOUTQ, &mut v); }
        v
    };

    // Make it nonblocking to observe the fill (EAGAIN) rather than block.
    unsafe {
        let fl = libc::fcntl(fd, libc::F_GETFL);
        libc::fcntl(fd, libc::F_SETFL, fl | libc::O_NONBLOCK);
    }

    // Phase 1: send as fast as possible until EAGAIN (buffer full).
    let mut n = 0u64;
    let mut wedged_at = None;
    for _ in 0..2_000_000u64 {
        let r = send_once(0);
        if r >= 0 { n += 1; continue; }
        let e = std::io::Error::last_os_error();
        if e.raw_os_error() == Some(libc::EAGAIN) || e.kind() == std::io::ErrorKind::WouldBlock {
            wedged_at = Some(n);
            break;
        }
        // ENOBUFS or other: report and keep trying a few, then stop.
        println!("send #{n} errored (not EAGAIN): {e} (errno {:?})", e.raw_os_error());
        wedged_at = Some(n);
        break;
    }
    match wedged_at {
        Some(k) => println!("PHASE1: send returned EAGAIN after {k} frames (~{} KiB queued), TIOCOUTQ={}",
            (k * buf.len() as u64) / 1024, outq()),
        None => { println!("PHASE1: never wedged in 2M frames — buffer drains fine, NOT the bug"); return; }
    }

    // Phase 2: idle, no traffic, no tokio. Does the buffer drain on its own?
    for secs in [5, 15, 30] {
        std::thread::sleep(std::time::Duration::from_secs(secs));
        let r = send_once(0);
        let ok = r >= 0;
        println!("PHASE2: after {secs}s idle -> send {} (TIOCOUTQ={})",
            if ok { "SUCCEEDED (drained)" } else { "still EAGAIN (permanent)" }, outq());
        if ok { println!("=> transient backpressure, drains when idle"); return; }
    }

    // Phase 3: no tokio in the picture at all — a BLOCKING send with a hard
    // 5s alarm. If it blocks forever (killed by alarm) the kernel send path is
    // genuinely stuck; tokio readiness was never involved.
    unsafe {
        let fl = libc::fcntl(fd, libc::F_GETFL);
        libc::fcntl(fd, libc::F_SETFL, fl & !libc::O_NONBLOCK);
        libc::alarm(5);
    }
    println!("PHASE3: attempting a BLOCKING send (5s alarm)…");
    let r = send_once(0);
    println!("PHASE3: blocking send returned {r} (if the process was killed by SIGALRM, it blocked forever = kernel send path stuck, tokio exonerated)");
}

#[cfg(not(target_os = "linux"))]
fn main() { eprintln!("linux only"); }
