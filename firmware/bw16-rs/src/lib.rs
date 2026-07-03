//! BW16 (RTL8720DN) named-radio firmware — **in Rust**.
//!
//! The first Rust-on-Ameba firmware: a `no_std` staticlib holding all the bridge
//! logic (serial framing, command dispatch, the promiscuous NDN filter, the radio
//! knobs), linked into the AmebaD image beside the closed Realtek WiFi blobs. A
//! ~40-line C++ shim (`bw16-rs-sketch.ino`) is the *only* C: it wraps the
//! Arduino/SDK calls behind the `c_*` FFI below and forwards `setup()`/`loop()`
//! and the promisc callback to us. Everything that thinks lives here.
//!
//! Wire protocol (unchanged, so the host `Bw16SerialBackend` needs no changes):
//!   `[0x4E 0x44] [type:u8] [len:u16 LE] [payload]`
//!   host→board: 0x01 INJECT / 0x02 CHANNEL / 0x04 RATE / 0x05 BW40
//!   board→host: 0x81 RX ([rssi:i8]+frame) / 0x82 LOG

#![no_std]
#![allow(static_mut_refs)]

use core::panic::PanicInfo;

// --- the C shim's wrappers (Arduino + Ameba SDK, the closed WiFi blob) ---
extern "C" {
    fn c_serial_begin(baud: u32);
    fn c_serial_write(buf: *const u8, len: u32);
    fn c_serial_read() -> i32; // -1 when no byte is ready
    fn c_serial_available() -> i32;
    fn c_delay(ms: u32);
    fn c_millis() -> u32;
    fn c_wifi_on_sta();
    fn c_wifi_set_channel(ch: i32);
    fn c_wifi_tx_raw_frame(buf: *const u8, len: u32);
    fn c_wifi_tx_raw_frame_attr(frame: *const u8, len: u32, pairs: *const u8, n_pairs: u32);
    fn c_wifi_set_promisc_enable();
    fn c_wifi_set_tx_data_rate(code: u8);
    fn c_wext_set_bw40(enable: u8);
    fn c_wifi_set_txpower(idx: i32);
}

const SYNC0: u8 = 0x4E;
const SYNC1: u8 = 0x44;
const T_INJECT: u8 = 0x01;
const T_CHANNEL: u8 = 0x02;
const T_TXPOWER: u8 = 0x03;
const T_RATE: u8 = 0x04;
const T_BW40: u8 = 0x05;
const T_INJECT_ATTR: u8 = 0x06; // [n_pairs][off,val]xN[frame]: poke pkt_attrib then inject
const T_RX: u8 = 0x81;
const T_LOG: u8 = 0x82;
const MAX_FRAME: usize = 1600;

/// Emit one `[SYNC0 SYNC1 type len payload]` frame (header + payload as separate
/// UART writes — no staging buffer needed).
fn send_framed(ty: u8, payload: &[u8]) {
    let len = payload.len() as u16;
    let hdr = [SYNC0, SYNC1, ty, (len & 0xff) as u8, (len >> 8) as u8];
    unsafe {
        c_serial_write(hdr.as_ptr(), hdr.len() as u32);
        if !payload.is_empty() {
            c_serial_write(payload.as_ptr(), payload.len() as u32);
        }
    }
}

fn logmsg(s: &[u8]) {
    send_framed(T_LOG, s);
}

/// Called by the C shim from `setup()`.
#[no_mangle]
pub extern "C" fn rust_setup() {
    unsafe {
        c_serial_begin(115_200);
        c_delay(200);
    }
    logmsg(b"boot-rs"); // distinctive marker: the Rust firmware is alive
    unsafe { c_wifi_on_sta() };
    logmsg(b"wifi_on");
    unsafe { c_wifi_set_channel(6) };
    logmsg(b"ch6");
    unsafe { c_wifi_set_promisc_enable() };
    logmsg(b"ready-rs");
}

/// Called by the C shim's promisc trampoline for every captured 802.11 frame.
/// NDN-radio filter: forward only non-QoS DATA frames carrying our ethertype
/// 0x8624 (after the 24-byte header + 6-byte LLC/SNAP), so the 115200 UART isn't
/// swamped by ambient beacons. RSSI byte is 0 (no stable rx-info in this SDK).
#[no_mangle]
pub extern "C" fn rust_promisc_cb(buf: *const u8, len: u32) {
    let len = len as usize;
    if len < 32 || len > MAX_FRAME - 1 || buf.is_null() {
        return;
    }
    let frame = unsafe { core::slice::from_raw_parts(buf, len) };
    if frame[0] & 0x0C != 0x08 {
        return; // must be a DATA frame
    }
    if frame[30] != 0x86 || frame[31] != 0x24 {
        return; // must carry the NDN ethertype
    }
    // [SYNC SYNC T_RX len] + [rssi=0] + frame, as three writes (len = 1 + frame).
    let total = (len + 1) as u16;
    let hdr = [SYNC0, SYNC1, T_RX, (total & 0xff) as u8, (total >> 8) as u8];
    let rssi = [0u8];
    unsafe {
        c_serial_write(hdr.as_ptr(), hdr.len() as u32);
        c_serial_write(rssi.as_ptr(), 1);
        c_serial_write(frame.as_ptr(), len as u32);
    }
}

/// Read at most one framed command from the host and act on it. Mirrors the
/// Arduino `loop()` cadence (called repeatedly by the shim).
#[no_mangle]
pub extern "C" fn rust_loop() {
    static mut RXBUF: [u8; MAX_FRAME] = [0; MAX_FRAME];
    unsafe {
        if c_serial_read() != SYNC0 as i32 {
            return;
        }
        while c_serial_available() < 1 {}
        if c_serial_read() != SYNC1 as i32 {
            return;
        }
        while c_serial_available() < 3 {}
        let ty = c_serial_read() as u8;
        let mut len = c_serial_read() as u16;
        len |= (c_serial_read() as u16) << 8;
        let len = len as usize;
        if len > MAX_FRAME {
            return;
        }
        let mut got = 0usize;
        let t0 = c_millis();
        while got < len && c_millis().wrapping_sub(t0) < 200 {
            if c_serial_available() > 0 {
                RXBUF[got] = c_serial_read() as u8;
                got += 1;
            }
        }
        if got != len {
            logmsg(b"short cmd");
            return;
        }
        let cmd = &RXBUF[..len];
        match ty {
            T_INJECT => c_wifi_tx_raw_frame(cmd.as_ptr(), len as u32),
            T_INJECT_ATTR if len >= 1 => {
                let n = cmd[0] as usize;
                let foff = 1 + 2 * n;
                if foff <= len {
                    let pairs = &cmd[1..foff];
                    let frame = &cmd[foff..];
                    c_wifi_tx_raw_frame_attr(frame.as_ptr(), frame.len() as u32, pairs.as_ptr(), n as u32);
                }
            }
            T_CHANNEL if len >= 1 => c_wifi_set_channel(cmd[0] as i32),
            T_TXPOWER if len >= 1 => c_wifi_set_txpower(cmd[0] as i32),
            T_RATE if len >= 1 => c_wifi_set_tx_data_rate(cmd[0]),
            T_BW40 if len >= 1 => c_wext_set_bw40((cmd[0] != 0) as u8),
            _ => {}
        }
    }
}

#[panic_handler]
fn panic(_: &PanicInfo) -> ! {
    loop {}
}
