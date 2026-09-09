//! **Off-host parse relevance gate — on-air demo (NDR_MAC_SPEC §6).**
//!
//! Two C5 serial bridges over their native USB-Serial-JTAG (no RTS/DTR reset, via `open_c5`). The RX
//! C5 runs the parse-gate firmware. We inject `/ndn/keep/*` and `/ndn/other/*` Interests from the TX
//! C5 and count what the RX delivers across three phases: floor (no set) -> gated (/ndn/keep) ->
//! cleared. Proves the device parses the carried name and drops off-prefix frames pre-link.
//!
//!   cargo run --example ndr_parse_gate --features serial-radio -- <rx_port> <tx_port> [channel]
use std::time::Duration;
use bytes::Bytes;
use ndn_radio_drivers::Esp32SerialBackend;
use ndn_radio_hal::{Bandwidth, FrameIo, InjectFrame, RadioKnobs, TxIntent};

fn varnum(n: usize, out: &mut Vec<u8>) {
    if n < 253 { out.push(n as u8); } else { out.push(0xfd); out.push((n >> 8) as u8); out.push(n as u8); }
}
fn tlv(t: u8, v: &[u8]) -> Vec<u8> { let mut o = vec![t]; varnum(v.len(), &mut o); o.extend_from_slice(v); o }
fn interest(parts: &[&str], seq: u32) -> Vec<u8> {
    let mut nv = Vec::new();
    for p in parts { nv.extend(tlv(0x08, p.as_bytes())); }
    let mut body = tlv(0x07, &nv);
    body.extend(tlv(0x0a, &seq.to_be_bytes()));
    tlv(0x05, &body)
}
fn rd_var(b: &[u8], i: &mut usize) -> Option<usize> {
    let x = *b.get(*i)? as usize;
    if x < 253 { *i += 1; Some(x) }
    else if x == 0xfd { let v = ((*b.get(*i+1)? as usize) << 8) | *b.get(*i+2)? as usize; *i += 3; Some(v) }
    else { None }
}
fn parse_name(pkt: &[u8]) -> Option<String> {
    if pkt.first().map_or(true, |&t| t != 0x05 && t != 0x06) { return None; }
    let mut i = 0; let _t = rd_var(pkt, &mut i)?; let ln = rd_var(pkt, &mut i)?;
    let val = pkt.get(i..i+ln)?; let mut j = 0;
    while j < val.len() {
        let tt = rd_var(val, &mut j)?; let ll = rd_var(val, &mut j)?;
        let sub = val.get(j..j+ll)?;
        if tt == 0x07 {
            let mut k = 0; let mut s = String::new();
            while k < sub.len() {
                let _ct = rd_var(sub, &mut k)?; let cl = rd_var(sub, &mut k)?;
                s.push('/'); s.push_str(&String::from_utf8_lossy(sub.get(k..k+cl)?)); k += cl;
            }
            return Some(s);
        }
        j += ll;
    }
    None
}

async fn burst(rx: &Esp32SerialBackend, tx: &Esp32SerialBackend, n: u32) -> (u32, u32) {
    // drain concurrently while injecting keep+other
    let rxr = async {
        let (mut keep, mut other) = (0u32, 0u32);
        let deadline = tokio::time::Instant::now() + Duration::from_secs(6);
        while tokio::time::Instant::now() < deadline {
            if let Ok(Ok(cap)) = tokio::time::timeout(Duration::from_millis(300), rx.recv_frame()).await {
                if let Some(nm) = parse_name(&cap.payload) {
                    if nm.starts_with("/ndn/keep/") { keep += 1; }
                    else if nm.starts_with("/ndn/other/") { other += 1; }
                }
            }
        }
        (keep, other)
    };
    let txr = async {
        tokio::time::sleep(Duration::from_millis(300)).await;
        for seq in 0..n {
            for tag in ["keep", "other"] {
                let p = interest(&["ndn", tag, &seq.to_string()], seq);
                let _ = tx.inject(InjectFrame::broadcast(Bytes::from(p), TxIntent::CONSERVATIVE)).await;
                tokio::time::sleep(Duration::from_millis(50)).await;
            }
        }
    };
    let (counts, _) = tokio::join!(rxr, txr);
    counts
}

#[tokio::main(flavor = "current_thread")]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    let a: Vec<String> = std::env::args().collect();
    let rx_port = a.get(1).cloned().unwrap_or_else(|| "/dev/cu.usbmodem101".into());
    let tx_port = a.get(2).cloned().unwrap_or_else(|| "/dev/cu.usbmodem11401".into());
    let ch: u8 = a.get(3).and_then(|s| s.parse().ok()).unwrap_or(6);
    let rx = Esp32SerialBackend::open_c5(&rx_port)?;
    let tx = Esp32SerialBackend::open_c5(&tx_port)?;
    rx.set_channel(ch, Bandwidth::Bw20)?;
    tx.set_channel(ch, Bandwidth::Bw20)?;
    tokio::time::sleep(Duration::from_millis(500)).await;
    println!("RX {rx_port}  TX {tx_port}  ch{ch}\n");

    rx.set_relevance_prefixes(&[])?;
    let (k, o) = burst(&rx, &tx, 30).await;
    println!("A floor (no set):    keep={k} other={o}   (expect both > 0)");

    rx.set_relevance_prefixes(&[b"/ndn/keep"])?;
    let (k, o) = burst(&rx, &tx, 30).await;
    println!("B gated /ndn/keep:   keep={k} other={o}   (expect keep>0, other=0)");

    rx.set_relevance_prefixes(&[])?;
    let (k, o) = burst(&rx, &tx, 30).await;
    println!("C cleared (floor):   keep={k} other={o}   (expect both > 0)");
    Ok(())
}
