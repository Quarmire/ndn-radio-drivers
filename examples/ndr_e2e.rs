//! **End-to-end on-air test** — a real NDN Interest -> Data round-trip over two radios, exercising the
//! whole NDR MAC path: a consumer expresses an Interest, the producer PARSES the name (relevance =
//! parse-the-name, §6), decides it is under its served prefix, and serves a Data with that name; the
//! consumer receives it. Producer on the C5 (reliable TX for the Data leg), consumer on the BW16
//! (reliable RX). The Interest leg is re-expressed (as real NDN does) until the producer hears one.
//!   cargo run --example ndr_e2e --features serial-radio -- <c5_producer_port> <bw16_consumer_port> [ch]
use bytes::Bytes;
use ndn_radio_drivers::{Bw16SerialBackend, Esp32SerialBackend};
use ndn_radio_hal::{Bandwidth, FrameIo, InjectFrame, RadioKnobs, TxIntent};
use std::sync::Arc;
use std::sync::atomic::{AtomicU32, Ordering};
use std::time::Duration;

const PREFIX: &[&str] = &["ndn", "svc"];

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
fn name_tlv(parts: &[String]) -> Vec<u8> {
    let mut nv = Vec::new();
    for p in parts {
        nv.extend(tlv(0x08, p.as_bytes()));
    }
    tlv(0x07, &nv)
}
fn interest(parts: &[String], seq: u32) -> Vec<u8> {
    let mut b = name_tlv(parts);
    b.extend(tlv(0x0a, &seq.to_be_bytes()));
    tlv(0x05, &b)
}
fn data(parts: &[String], content: &[u8]) -> Vec<u8> {
    let mut b = name_tlv(parts);
    b.extend(tlv(0x15, content));
    tlv(0x06, &b)
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
/// (kind, /-joined name) from an NDN packet payload. kind 5=Interest 6=Data.
fn parse(pkt: &[u8]) -> Option<(u8, Vec<String>)> {
    let k = *pkt.first()?;
    if k != 0x05 && k != 0x06 {
        return None;
    }
    let mut i = 0;
    rd(pkt, &mut i)?;
    let ln = rd(pkt, &mut i)?;
    let val = pkt.get(i..i + ln)?;
    let mut j = 0;
    while j < val.len() {
        let t = rd(val, &mut j)?;
        let l = rd(val, &mut j)?;
        let sub = val.get(j..j + l)?;
        if t == 0x07 {
            let mut m = 0;
            let mut cs = vec![];
            while m < sub.len() {
                rd(sub, &mut m)?;
                let cl = rd(sub, &mut m)?;
                cs.push(String::from_utf8_lossy(sub.get(m..m + cl)?).into_owned());
                m += cl;
            }
            return Some((k, cs));
        }
        j += l;
    }
    None
}
fn under_prefix(name: &[String]) -> bool {
    name.len() >= PREFIX.len() && name[..PREFIX.len()].iter().zip(PREFIX).all(|(a, b)| a == b)
}

#[tokio::main(flavor = "current_thread")]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    let a: Vec<String> = std::env::args().collect();
    let prod_port = a.get(1).cloned().unwrap_or("/dev/cu.usbmodem11401".into());
    let cons_port = a
        .get(2)
        .cloned()
        .unwrap_or("/dev/cu.usbserial-11110".into());
    let ch: u8 = a.get(3).and_then(|s| s.parse().ok()).unwrap_or(6);
    let producer = Arc::new(Esp32SerialBackend::open_c5(&prod_port)?);
    let consumer = Arc::new(Bw16SerialBackend::open(&cons_port)?);
    producer.set_channel(ch, Bandwidth::Bw20)?;
    consumer.set_channel(ch)?;
    tokio::time::sleep(Duration::from_millis(2500)).await; // BW16 boot
    println!(
        "E2E: producer(C5) {prod_port} serves /ndn/svc ; consumer(BW16) {cons_port} ; ch{ch}\n"
    );

    let served = Arc::new(AtomicU32::new(0));
    // Producer: hear Interests, serve Data for /ndn/svc/*.
    let prod = producer.clone();
    let served_c = served.clone();
    let ph = tokio::spawn(async move {
        let deadline = tokio::time::Instant::now() + Duration::from_secs(30);
        while tokio::time::Instant::now() < deadline {
            if let Ok(Ok(cap)) =
                tokio::time::timeout(Duration::from_millis(300), prod.recv_frame()).await
            {
                if let Some((0x05, name)) = parse(&cap.payload) {
                    if under_prefix(&name) {
                        // relevance decided by parsing the name -> serve the Data for that exact name.
                        let d = data(&name, b"ndr-e2e-content");
                        let _ = prod
                            .inject(InjectFrame::broadcast(
                                Bytes::from(d),
                                TxIntent::CONSERVATIVE,
                            ))
                            .await;
                        served_c.fetch_add(1, Ordering::Relaxed);
                    }
                }
            }
        }
    });
    // Consumer: express Interests, re-expressing until the Data comes back.
    let rounds = 15u32;
    let mut ok = 0u32;
    for seq in 0..rounds {
        let name: Vec<String> = PREFIX
            .iter()
            .map(|s| s.to_string())
            .chain([seq.to_string()])
            .collect();
        let mut got = false;
        'req: for _try in 0..6u32 {
            // re-express up to 6x
            consumer
                .inject(InjectFrame::broadcast(
                    Bytes::from(interest(&name, seq)),
                    TxIntent::CONSERVATIVE,
                ))
                .await
                .ok();
            let win = tokio::time::Instant::now() + Duration::from_millis(500);
            while tokio::time::Instant::now() < win {
                if let Ok(Ok(cap)) =
                    tokio::time::timeout(Duration::from_millis(150), consumer.recv_frame()).await
                {
                    if let Some((0x06, dname)) = parse(&cap.payload) {
                        if dname == name {
                            got = true;
                            break 'req;
                        }
                    }
                }
            }
        }
        if got {
            ok += 1;
            println!("  round {seq:2}: Interest /ndn/svc/{seq} -> Data received  ✓");
        } else {
            println!("  round {seq:2}: /ndn/svc/{seq} — no Data (missed on air)");
        }
    }
    ph.abort();
    println!(
        "\nE2E round-trips: {ok}/{rounds} completed on air; producer served {} Interests.",
        served.load(Ordering::Relaxed)
    );
    if ok > 0 {
        println!(
            "✅ END-TO-END WORKS: Interest -> parse-name relevance -> Data, over real radios."
        );
    }
    Ok(())
}
