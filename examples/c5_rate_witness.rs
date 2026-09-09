//! Diagnose C5 set_rate: TX a fixed HT MCS from the C5, witness delivery + decoded rate on the BW16.
//!   cargo run --example c5_rate_witness --features serial-radio -- <c5_tx_port> <bw16_rx_port> [ch]
use std::time::Duration;
use bytes::Bytes;
use ndn_radio_drivers::{Bw16SerialBackend, Esp32SerialBackend};
use ndn_radio_hal::{Bandwidth, FrameIo, InjectFrame, McsDescriptor, RadioKnobs, TxIntent};
#[tokio::main(flavor="current_thread")]
async fn main()->Result<(),Box<dyn std::error::Error>>{
    let a:Vec<String>=std::env::args().collect();
    let txp=a.get(1).cloned().unwrap_or("/dev/cu.usbmodem11401".into());
    let rxp=a.get(2).cloned().unwrap_or("/dev/cu.usbserial-11110".into());
    let ch:u8=a.get(3).and_then(|s|s.parse().ok()).unwrap_or(6);
    let tx=Esp32SerialBackend::open_c5(&txp)?;
    let rx=Bw16SerialBackend::open(&rxp)?;
    tx.set_channel(ch,Bandwidth::Bw20)?; rx.set_channel(ch)?;
    tokio::time::sleep(Duration::from_millis(2500)).await;   // BW16 boot
    for m in [255u8, 0,2,4,7] {                              // 255 = default rate (no set_rate)
        if m!=255 { tx.set_rate(McsDescriptor::ht(m))?; tokio::time::sleep(Duration::from_millis(200)).await; }
        let rxr=async{
            let mut got=0u32; let mut mcs=std::collections::BTreeMap::<i32,u32>::new();
            let dl=tokio::time::Instant::now()+Duration::from_secs(4);
            while tokio::time::Instant::now()<dl {
                if let Ok(Ok(c))=tokio::time::timeout(Duration::from_millis(300),rx.recv_frame()).await {
                    got+=1; *mcs.entry(c.mcs_index.map(|x|x as i32).unwrap_or(-1)).or_default()+=1;
                }
            } (got,mcs)
        };
        let txr=async{ tokio::time::sleep(Duration::from_millis(200)).await;
            for seq in 0..30u32 {
                let p=format!("\x05\x08c5rt-{seq:02}");
                let _=tx.inject(InjectFrame::broadcast(Bytes::from(p.into_bytes()),TxIntent::CONSERVATIVE)).await;
                tokio::time::sleep(Duration::from_millis(50)).await;
            }
        };
        let ((got,mcs),_)=tokio::join!(rxr,txr);
        let label=if m==255 {"default".to_string()} else {format!("HT MCS{m}")};
        println!("TX {label:10} -> BW16 delivered {got}/30, decoded mcs_index counts {mcs:?}");
    }
    Ok(())
}
