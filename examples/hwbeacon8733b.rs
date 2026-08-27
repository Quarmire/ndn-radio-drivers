//! Does a downloaded reserved page actually TRANSMIT at TBTT?
//!
//! `dl_rsvd_page` now succeeds (8b15f76), but a page that loads is not a beacon that goes on air.
//! This drives the rest of the vendor's port-0 beacon path and lets a second radio judge:
//!
//!   REG_CR+2 [1:0] = 3          net_type = AP        (_HW_STATE_AP_)
//!   0x0554 = interval in TU     REG_MBSSID_BCN_SPACE (port-0 bcn_space, mask 0xffff)
//!   0x0553 |= BIT0              REG_DUAL_TSF_RST     (BIT_TSFTR_RST)
//!   0x0550 |= BIT3              REG_BCN_CTRL         (BIT_EN_BCN_FUNCTION)
//!
//! The trick that makes this measurable: the page holds raw bytes, so instead of a real 802.11
//! beacon it gets ONE OF OUR OWN 0x8624 frames. A witness then decodes it like any other frame and
//! its hardware RX stamps show the cadence directly — a hardware-timed transmit should land on a
//! tight multiple of the beacon interval, far tighter than host-driven injection can manage.
use bytes::Bytes;
use ndn_radio_drivers::{BROADCAST, FrameFormat, InjectFrame, Rtl8733buBackend, TxIntent, frame};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let ch: u8 = std::env::args()
        .nth(1)
        .and_then(|s| s.parse().ok())
        .unwrap_or(36);
    let tu: u16 = std::env::args()
        .nth(2)
        .and_then(|s| s.parse().ok())
        .unwrap_or(100);
    let secs: u64 = std::env::args()
        .nth(3)
        .and_then(|s| s.parse().ok())
        .unwrap_or(20);
    let dev = Rtl8733buBackend::open()?;
    dev.bring_up_tx(ch)?;

    // Our own frame, tagged knob 7 so a witness can tell beacon-borne traffic from anything else.
    let fmt = FrameFormat::RawNdn {
        ethertype: ndn_radio_drivers::NDN_ETHERTYPE,
    };
    let mut p = vec![0xC3u8; 200];
    p[0] = 0;
    p[1] = 7;
    p[2] = 0xC3;
    let f = InjectFrame {
        payload: Bytes::from(p),
        tx: TxIntent::CONSERVATIVE,
        dst: BROADCAST,
        src: [0x02, 0x50, 0x33, 0x02, 7, 0],
        addr3: None,
        addr4: None,
        htc: None,
    };
    let dot11 = frame::build_dot11(fmt, &f)?;
    println!(
        "beacon payload = {} bytes of our own 0x8624 frame",
        dot11.len()
    );
    dev.dl_rsvd_page(0x80, &dot11)?;
    println!("reserved page loaded (BCN_VALID asserted)");

    // ★ REG_FWHW_TXQ_CTRL+2 BIT6 = "this page IS a real beacon frame". The vendor CLEARS it during
    // the download and restores it after; dl_rsvd_page saves/restores whatever it found, so if
    // bring-up leaves it clear the page is downloaded and then never treated as a beacon. Report
    // what it was, then set it explicitly — a candidate for why the page loads but never transmits.
    let txq = dev.read8(0x0420 + 2)?;
    dev.write8(0x0420 + 2, txq | (1 << 6))?;
    println!(
        "FWHW_TXQ_CTRL+2 was {txq:#04x} (bcn-queue bit6 = {}), now {:#04x}",
        (txq >> 6) & 1,
        dev.read8(0x0420 + 2)?
    );
    // net_type = AP on port 0
    let cr2 = dev.read8(0x0100 + 2)?;
    dev.write8(0x0100 + 2, (cr2 & !0x03) | 0x03)?;
    // beacon interval, TU
    dev.write8(0x0554, (tu & 0xff) as u8)?;
    dev.write8(0x0555, (tu >> 8) as u8)?;
    // reset TSF so TBTT starts from a known point
    let r = dev.read8(0x0553)?;
    dev.write8(0x0553, r | 0x01)?;
    // enable the beacon function
    let b = dev.read8(0x0550)?;
    dev.write8(0x0550, b | (1 << 3))?;
    println!(
        "armed: net_type={:#04x} bcn_space={} TU ({:.1} ms) BCN_CTRL={:#04x}",
        dev.read8(0x0100 + 2)? & 0x03,
        tu,
        f32::from(tu) * 1.024,
        dev.read8(0x0550)?
    );
    println!("beaconing for {secs}s — watch the witness");
    // Does TBTT even have a clock to fire on? Sample the port TSF across the window: if it does not
    // advance, "no beacon" means the timer is stopped, which is a different problem from "the timer
    // fires but the frame is not transmitted". Splits the remaining search space in one read.
    let t0 = dev.read_tsf()?;
    std::thread::sleep(std::time::Duration::from_secs(secs));
    let t1 = dev.read_tsf()?;
    let ticks = t1.wrapping_sub(t0);
    println!(
        "port TSF advanced {ticks} ticks in {secs}s = {:.0} ticks/s (4us/tick => expect ~{:.0})",
        ticks as f64 / secs as f64,
        250_000.0
    );
    println!(
        "expected TBTTs in that window: {:.0}",
        secs as f64 * 1000.0 / (f32::from(tu) * 1.024) as f64
    );
    // Disarm so the radio does not keep beaconing into later experiments.
    let b = dev.read8(0x0550)?;
    dev.write8(0x0550, b & !(1 << 3))?;
    dev.write8(0x0100 + 2, cr2)?;
    println!("disarmed");
    Ok(())
}
