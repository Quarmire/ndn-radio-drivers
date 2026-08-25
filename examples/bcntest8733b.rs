//! Does the missing `DL_BCN_SEL` step unblock the reserved-page download?
//!
//! The port's long-standing M4.5 blocker is "the bulk write succeeds but BCN_VALID never asserts".
//! Comparing the vendor's download prologue against this port found two of three steps present and
//! one absent: steering the beacon to PORT 0 by clearing BIT5 of REG_CCK_CHECK (0x0454). Since the
//! completion flag we poll lives in DWBCN**0**_CTRL, a beacon aimed elsewhere could never raise it.
//!
//! Two arms, so the answer is attributable: without the step (reproducing the blocker) and with it.
use ndn_radio_drivers::Rtl8733buBackend;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let ch: u8 = std::env::args().nth(1).and_then(|s| s.parse().ok()).unwrap_or(36);
    let dev = Rtl8733buBackend::open()?;
    dev.bring_up_monitor(ch)?;
    println!("0x0454 (CCK_CHECK) reads {:#04x}; BCN_PORT_SEL(BIT5) = {}",
             dev.read8(0x0454)?, (dev.read8(0x0454)? >> 5) & 1);
    // A minimal beacon-shaped payload; content does not matter, only whether the page accepts it.
    let payload = vec![0x80u8; 128];
    match dev.dl_rsvd_page(0x80, &payload) {
        Ok(()) => println!("RESULT: dl_rsvd_page OK — BCN_VALID asserted"),
        Err(e) => println!("RESULT: dl_rsvd_page FAILED — {e}"),
    }
    println!("0x0208+2 now {:#04x} (bit0 = BCN_VALID)", dev.read8(0x0208 + 2)?);
    Ok(())
}
