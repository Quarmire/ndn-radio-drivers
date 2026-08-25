#![allow(unsafe_code)]
//! Controlled A/B for the AF_PACKET TX wedge. Opens a tokio AsyncFd AF_PACKET
//! socket (exactly like AfPacketBackend) and drives it with CONCURRENT injects
//! to force the send buffer to EAGAIN, using either the OLD send loop
//! (`writable().await` first, `clear_ready` on WouldBlock — the shipped bug) or
//! the NEW loop (optimistic send, bounded readiness wait + retry, deadline).
//!
//!   sudo ./af_ab_probe mon0 old   # expect: injects HANG (wedge reproduced)
//!   sudo ./af_ab_probe mon0 new   # expect: all injects complete (fixed)
use std::os::fd::{AsRawFd, FromRawFd, OwnedFd, RawFd};
use std::sync::Arc;
use std::time::Duration;
use tokio::io::unix::AsyncFd;

const ETH_P_ALL: u16 = 0x0003;

struct Sock { fd: AsyncFd<OwnedFd>, ifindex: i32 }

impl Sock {
    fn open(iface: &str) -> Self {
        let cname = std::ffi::CString::new(iface).unwrap();
        let ifindex = unsafe { libc::if_nametoindex(cname.as_ptr()) } as i32;
        assert!(ifindex != 0, "no iface");
        let fd = unsafe { libc::socket(libc::AF_PACKET, libc::SOCK_RAW | libc::SOCK_NONBLOCK, (ETH_P_ALL.to_be()) as i32) };
        assert!(fd >= 0);
        let owned = unsafe { OwnedFd::from_raw_fd(fd) };
        let mut a: libc::sockaddr_ll = unsafe { std::mem::zeroed() };
        a.sll_family = libc::AF_PACKET as u16; a.sll_protocol = ETH_P_ALL.to_be(); a.sll_ifindex = ifindex;
        assert!(unsafe { libc::bind(owned.as_raw_fd(), &a as *const _ as *const libc::sockaddr, std::mem::size_of::<libc::sockaddr_ll>() as u32) } == 0);
        Sock { fd: AsyncFd::new(owned).unwrap(), ifindex }
    }
    async fn recv_once(&self) -> std::io::Result<usize> {
        loop {
            let mut g = self.fd.readable().await?;
            let fd = self.fd.get_ref().as_raw_fd();
            match g.try_io(|_| {
                let mut b = [0u8; 4096];
                let n = unsafe { libc::recv(fd, b.as_mut_ptr() as *mut libc::c_void, b.len(), 0) };
                if n < 0 { Err(std::io::Error::last_os_error()) } else { Ok(n as usize) }
            }) {
                Ok(r) => return r,
                Err(_would_block) => continue,
            }
        }
    }
    fn raw_send(&self, buf: &[u8]) -> isize {
        let mut d: libc::sockaddr_ll = unsafe { std::mem::zeroed() };
        d.sll_family = libc::AF_PACKET as u16; d.sll_protocol = ETH_P_ALL.to_be(); d.sll_ifindex = self.ifindex;
        unsafe { libc::sendto(self.fd.get_ref().as_raw_fd(), buf.as_ptr() as *const libc::c_void, buf.len(), 0,
            &d as *const _ as *const libc::sockaddr, std::mem::size_of::<libc::sockaddr_ll>() as u32) }
    }
    // OLD: the shipped loop — await writable FIRST, clear_ready on WouldBlock.
    async fn inject_old(&self, buf: &[u8]) -> std::io::Result<()> {
        loop {
            let mut g = self.fd.writable().await?;
            if self.raw_send(buf) >= 0 { return Ok(()); }
            let e = std::io::Error::last_os_error();
            if e.kind() == std::io::ErrorKind::WouldBlock { g.clear_ready(); continue; }
            return Err(e);
        }
    }
    // NEW: optimistic send, bounded readiness wait + retry, 2s deadline.
    async fn inject_new(&self, buf: &[u8]) -> std::io::Result<()> {
        let deadline = tokio::time::Instant::now() + Duration::from_secs(2);
        loop {
            if self.raw_send(buf) >= 0 { return Ok(()); }
            let e = std::io::Error::last_os_error();
            if e.kind() != std::io::ErrorKind::WouldBlock { return Err(e); }
            if tokio::time::Instant::now() >= deadline { return Err(e); }
            let _ = tokio::time::timeout(Duration::from_millis(20), async {
                if let Ok(mut g) = self.fd.writable().await { g.clear_ready(); }
            }).await;
        }
    }
}

#[cfg(target_os = "linux")]
#[tokio::main(flavor = "current_thread")]
async fn main() {
    let iface = std::env::args().nth(1).unwrap_or_else(|| "mon0".into());
    let mode = std::env::args().nth(2).unwrap_or_else(|| "new".into());
    let sock = Arc::new(Sock::open(&iface));
    let buf: Arc<Vec<u8>> = Arc::new({
        // minimal radiotap (8B) + a data-ish body; validity is irrelevant to
        // send-buffer accounting, which is what we're stressing.
        let mut v = vec![0u8, 0, 8, 0, 0, 0, 0, 0]; v.extend_from_slice(&[0x5a; 900]); v
    });
    println!("mode={mode} iface={iface}: RX drainer + 400 CONCURRENT injects (matches the live face)…");
    // Continuously drain RX on the SAME socket, like the real recv_frame loop —
    // the one production condition the earlier A/B run lacked.
    {
        let s = sock.clone();
        tokio::spawn(async move { loop { let _ = s.recv_once().await; } });
    }

    let mut handles = Vec::new();
    for i in 0..400u32 {
        let (s, b, m) = (sock.clone(), buf.clone(), mode.clone());
        handles.push(tokio::spawn(async move {
            // 8s watchdog >> new's 2s deadline; a timeout here = a hung inject.
            let r = tokio::time::timeout(Duration::from_secs(8), async move {
                if m == "old" { s.inject_old(&b).await } else { s.inject_new(&b).await }
            }).await;
            match r { Err(_) => (i, "HUNG"), Ok(Ok(())) => (i, "ok"), Ok(Err(_)) => (i, "dropped") }
        }));
    }
    let mut ok = 0; let mut dropped = 0; let mut hung = 0;
    for h in handles { match h.await.unwrap().1 { "ok" => ok += 1, "dropped" => dropped += 1, _ => hung += 1 } }
    println!("RESULT mode={mode}: ok={ok} dropped={dropped} HUNG={hung} (of 400)");
    if hung > 0 { println!("=> {hung} injects never returned within 8s: THIS logic wedges."); }
    else { println!("=> every inject returned (no permanent hang): THIS logic is robust."); }
}

#[cfg(not(target_os = "linux"))]
fn main() { eprintln!("linux only"); }
