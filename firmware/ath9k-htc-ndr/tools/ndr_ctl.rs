//! Reconfigure the AR9271 firmware at runtime by injecting a control frame.
//!
//! The firmware intercepts frames addressed to 03:4e:44:52:43:54 in its transmit path and consumes
//! them as configuration instead of transmitting. That is the only way to reach `ndr_cfg` while
//! `ath9k_htc` owns the device — the WMI path needs the device, and the PHY needs the kernel driver.
//!
//! ```sh
//! sudo ndr_ctl wlu1u2 enable 1
//! sudo ndr_ctl wlu1u2 lease 2 8 0      # log2(slots)=2 -> 4 slots of 8 TU, take slot 0
//! sudo ndr_ctl wlu1u2 quiet-off
//! ```

use std::ffi::CString;

const CTL: [u8; 6] = [0x03, 0x4e, 0x44, 0x52, 0x43, 0x54];
const MAGIC: u32 = 0x4e44_5243;

fn frame(op: u8, payload: &[u8]) -> Vec<u8> {
    let mut f = vec![0x00, 0x00, 0x08, 0x00, 0x00, 0x00, 0x00, 0x00]; // radiotap, no fields
    f.extend_from_slice(&[0x08, 0x00, 0x00, 0x00]); // fc = data, duration
    f.extend_from_slice(&CTL); // addr1 = the control address
    f.extend_from_slice(&[0x03, 0x66, 0x77, 0x88, 0x99, 0xaa]); // addr2
    f.extend_from_slice(&[0x03, 0xbb, 0xcc, 0xdd, 0xee, 0xff]); // addr3
    f.extend_from_slice(&[0x00, 0x00]); // seq
    f.extend_from_slice(&MAGIC.to_be_bytes());
    f.push(op);
    f.push(payload.len() as u8);
    f.extend_from_slice(payload);
    while f.len() < 8 + 24 + 32 {
        f.push(0); // pad clear of any minimum-length handling
    }
    f
}

fn main() {
    let a: Vec<String> = std::env::args().collect();
    if a.len() < 3 {
        eprintln!(
            "usage: {} <iface> <enable|drop-foreign|key|nmasks|mask|clear-stats|lease|quiet-off> [args]",
            a[0]
        );
        std::process::exit(1);
    }
    let arg = |i: usize| -> String { a.get(i).cloned().unwrap_or_default() };

    let (op, payload): (u8, Vec<u8>) = match a[2].as_str() {
        "enable" => (0x01, vec![arg(3).parse().unwrap_or(1)]),
        "drop-foreign" => (0x02, vec![arg(3).parse().unwrap_or(0)]),
        "key" => (0x03, arg(3).parse::<u64>().unwrap_or(0).to_be_bytes().to_vec()),
        "nmasks" => (0x04, vec![arg(3).parse().unwrap_or(0)]),
        "mask" => {
            let mut p = vec![arg(3).parse().unwrap_or(0u8)];
            p.extend(
                arg(4)
                    .split(',')
                    .filter_map(|b| u8::from_str_radix(b.trim().trim_start_matches("0x"), 16).ok()),
            );
            (0x05, p)
        }
        "clear-stats" => (0x06, vec![]),
        "lease" => (
            0x07,
            vec![
                arg(3).parse().unwrap_or(2), // log2(slots)
                arg(4).parse().unwrap_or(8), // slot TU
                arg(5).parse().unwrap_or(0), // this node's slot
            ],
        ),
        "quiet-off" => (0x08, vec![]),
        other => {
            eprintln!("unknown op {other}");
            std::process::exit(1);
        }
    };

    unsafe {
        let fd = libc::socket(
            libc::AF_PACKET,
            libc::SOCK_RAW,
            (libc::ETH_P_ALL as u16).to_be() as i32,
        );
        if fd < 0 {
            eprintln!("socket: {} (need root)", std::io::Error::last_os_error());
            std::process::exit(1);
        }
        let cname = CString::new(a[1].as_str()).unwrap();
        let idx = libc::if_nametoindex(cname.as_ptr());
        if idx == 0 {
            eprintln!("no such iface {}", a[1]);
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
        let f = frame(op, &payload);
        // Send a few: a control frame is unacknowledged, and one lost frame silently does nothing.
        for _ in 0..3 {
            if libc::send(fd, f.as_ptr() as *const libc::c_void, f.len(), 0) < 0 {
                eprintln!("send: {}", std::io::Error::last_os_error());
                std::process::exit(1);
            }
            std::thread::sleep(std::time::Duration::from_millis(20));
        }
        println!("{} op={op:#04x} payload={payload:02x?} sent", a[2]);
        libc::close(fd);
    }
}
