//! **On-air airtime A/B (item 6): C5 TX (rate-swept) -> BW16 RX (reliable witness).**
//! Sweeps the HT MCS ladder, measures live delivery, computes airtime-per-delivered-frame per rate —
//! the airtime-efficiency win a rate-adaptive plane gets over a fixed basic-rate blast (wfb-ng style).
//!   cargo run --example ndr_airtime_ab --features serial-radio -- <c5_tx_port> <bw16_rx_port> [ch]
use bytes::Bytes;
use ndn_radio_drivers::{Bw16SerialBackend, Esp32SerialBackend};
use ndn_radio_hal::{
    Bandwidth, FrameIo, InjectFrame, McsDescriptor, RadioKnobs, TxIntent, mcs_phy_rate_bps,
};
use std::time::Duration;
fn varnum(n: usize, o: &mut Vec<u8>) {
    if n < 253 {
        o.push(n as u8)
    } else {
        o.push(0xfd);
        o.push((n >> 8) as u8);
        o.push(n as u8)
    }
}
fn tlv(t: u8, v: &[u8]) -> Vec<u8> {
    let mut o = vec![t];
    varnum(v.len(), &mut o);
    o.extend_from_slice(v);
    o
}
fn interest(tag: &str, seq: u32, pad: usize) -> Vec<u8> {
    let mut nv = Vec::new();
    for c in ["ndn", "ab", tag, &seq.to_string()] {
        nv.extend(tlv(0x08, c.as_bytes()));
    }
    nv.extend(tlv(0x08, &vec![b'x'; pad]));
    let mut body = tlv(0x07, &nv);
    body.extend(tlv(0x0a, &seq.to_be_bytes()));
    tlv(0x05, &body)
}
fn rd(b: &[u8], i: &mut usize) -> Option<usize> {
    let x = *b.get(*i)? as usize;
    if x < 253 {
        *i += 1;
        Some(x)
    } else if x == 0xfd {
        let v = ((*b.get(*i + 1)? as usize) << 8) | *b.get(*i + 2)? as usize;
        *i += 3;
        Some(v)
    } else {
        None
    }
}
fn tag_of(pkt: &[u8]) -> Option<String> {
    if pkt.first().map_or(true, |&t| t != 0x05 && t != 0x06) {
        return None;
    }
    let mut i = 0;
    rd(pkt, &mut i)?;
    let ln = rd(pkt, &mut i)?;
    let val = pkt.get(i..i + ln)?;
    let mut j = 0;
    while j < val.len() {
        let tt = rd(val, &mut j)?;
        let ll = rd(val, &mut j)?;
        let sub = val.get(j..j + ll)?;
        if tt == 0x07 {
            let mut k = 0;
            let mut cs = vec![];
            while k < sub.len() {
                rd(sub, &mut k)?;
                let cl = rd(sub, &mut k)?;
                cs.push(sub.get(k..k + cl)?.to_vec());
                k += cl;
            }
            if cs.len() >= 3 && cs[0] == b"ndn" && cs[1] == b"ab" {
                return Some(String::from_utf8_lossy(&cs[2]).into());
            }
            return None;
        }
        j += ll
    }
    None
}
async fn arm(
    rx: &Bw16SerialBackend,
    tx: &Esp32SerialBackend,
    tag: &str,
    n: u32,
    pad: usize,
) -> u32 {
    let want = tag.to_string();
    let rxr = async {
        let mut g = 0u32;
        let dl = tokio::time::Instant::now() + Duration::from_secs(6);
        while tokio::time::Instant::now() < dl {
            if let Ok(Ok(c)) =
                tokio::time::timeout(Duration::from_millis(300), rx.recv_frame()).await
            {
                if tag_of(&c.payload).as_deref() == Some(want.as_str()) {
                    g += 1
                }
            }
        }
        g
    };
    let txr = async {
        tokio::time::sleep(Duration::from_millis(300)).await;
        for seq in 0..n {
            let p = interest(tag, seq, pad);
            let _ = tx
                .inject(InjectFrame::broadcast(
                    Bytes::from(p),
                    TxIntent::CONSERVATIVE,
                ))
                .await;
            tokio::time::sleep(Duration::from_millis(40)).await;
        }
    };
    let (g, _) = tokio::join!(rxr, txr);
    g
}
#[tokio::main(flavor = "current_thread")]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    let a: Vec<String> = std::env::args().collect();
    let txp = a.get(1).cloned().unwrap_or("/dev/cu.usbmodem11401".into());
    let rxp = a
        .get(2)
        .cloned()
        .unwrap_or("/dev/cu.usbserial-11110".into());
    let ch: u8 = a.get(3).and_then(|s| s.parse().ok()).unwrap_or(6);
    let tx = Esp32SerialBackend::open_c5(&txp)?;
    let rx = Bw16SerialBackend::open(&rxp)?;
    tx.set_channel(ch, Bandwidth::Bw20)?;
    rx.set_channel(ch)?;
    tokio::time::sleep(Duration::from_millis(2500)).await;
    let n = 40u32;
    let pad = 480usize;
    let frame_bits = ((pad + 40) * 8) as f64;
    const OH: f64 = 60.0;
    println!(
        "On-air airtime A/B (C5 TX -> BW16 RX, ch{ch}, {n}/arm, ~{}B):",
        pad + 40
    );
    println!(" MCS  rate(Mb/s)  deliv/{n}  air/frame(us)  air/delivered(us)");
    let mut best = (255u8, f64::MAX);
    let mut fixed0 = f64::MAX;
    for &m in &[0u8, 2, 4, 6, 7] {
        tx.set_rate(McsDescriptor::ht(m))?;
        tokio::time::sleep(Duration::from_millis(150)).await;
        let g = arm(&rx, &tx, &format!("m{m}"), n, pad).await;
        let rate = mcs_phy_rate_bps(m) as f64;
        let air = frame_bits / rate * 1e6 + OH;
        let per = if g > 0 {
            air * n as f64 / g as f64
        } else {
            f64::INFINITY
        };
        println!(
            "  {m:2}  {:8.1}    {g:3}/{n}    {air:8.1}     {per:10.1}",
            rate / 1e6
        );
        if g > 0 && per < best.1 {
            best = (m, per)
        }
        if m == 0 {
            fixed0 = per
        }
    }
    println!(
        "\n  adaptive (best MCS {}): {:.1} us/delivered",
        best.0, best.1
    );
    if fixed0.is_finite() {
        println!("  fixed basic (MCS0, wfb-ng style): {fixed0:.1} us/delivered");
        println!(
            "  -> adaptive {:.2}x better on airtime-per-delivered, ON AIR",
            fixed0 / best.1
        );
    }
    Ok(())
}
