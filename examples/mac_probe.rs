//! Minimal macOS de-risk: can libusb claim the RTL8812EU (0bda:a81a) here?
use ndn_radio_drivers::LibUsbRtl88xxBackend;
fn main() {
    match LibUsbRtl88xxBackend::open() {
        Ok(_) => println!("OK: claimed the RTL8812EU via libusb"),
        Err(e) => println!("ERR: {e}"),
    }
}
