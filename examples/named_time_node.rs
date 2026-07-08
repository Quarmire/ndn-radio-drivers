//! A live named-time node on a real monitor-wifi radio (the doc's "runs on hardware" seam).
//! Opens the RTL8812EU, pipelines RX with the async-URB pump (the better USB data path), wraps it
//! as a FrameIoTransport, and runs a TimeService (OS-clock source) that ingests peer beacons and
//! broadcasts its own. Proves the ndn-time pipeline stands up on-air.
use ndn_frame_io::FrameIo;
use ndn_radio_drivers::LibUsbRtl88xxBackend;
use ndn_time::{ClockCapability, KeyId, TimePolicy};
use ndn_time_driver::wifi::FrameIoTransport;
use ndn_time_driver::{ClockSink, DevAuth, TimeService};
use ndn_time_sources::OsClock;
use std::sync::{Arc, Mutex};
use std::time::Duration;

struct PrintClock(Mutex<Option<i64>>);
impl ClockSink for PrintClock {
    fn set_clock_ms(&self, ms: i64) {
        *self.0.lock().unwrap() = Some(ms);
    }
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    // Args: <node_id> [channel] [secs]. Give each of the two nodes a distinct id so their beacons
    // are attributed to different peers (0 on one machine, 1 on the other).
    let id: u64 = std::env::args().nth(1).and_then(|s| s.parse().ok()).unwrap_or(0);
    let ch: u8 = std::env::args().nth(2).and_then(|s| s.parse().ok()).unwrap_or(36);
    let secs: u64 = std::env::args().nth(3).and_then(|s| s.parse().ok()).unwrap_or(20);
    // Optional 4th arg: a specific radio's product id (hex, e.g. a81a / 881a) when several are
    // attached — the two nodes of an on-air test each pin their own dongle.
    let pid: Option<u16> = std::env::args()
        .nth(4)
        .and_then(|s| u16::from_str_radix(&s, 16).ok());
    let rt = tokio::runtime::Builder::new_multi_thread().enable_all().build()?;
    rt.block_on(async move {
        // Open + pipeline RX: spawn_rx_pump keeps 4 bulk-IN transfers in flight (async-URB path),
        // so a busy channel isn't dropped between reads and each frame's stamp is taken promptly.
        let radio = Arc::new(match pid {
            Some(p) => LibUsbRtl88xxBackend::open_monitor_pid(p, ch)?,
            None => LibUsbRtl88xxBackend::open_monitor(ch)?,
        });
        let _pump = radio.spawn_rx_pump(4);
        println!("named-time node {id} up: 8812EU ch{ch}, RX pump depth 4");

        let clock = Arc::new(PrintClock(Mutex::new(None)));
        let transport = Arc::new(FrameIoTransport::new(radio as Arc<dyn FrameIo>));
        let svc = Arc::new(TimeService::new(
            id,
            KeyId(id),
            ClockCapability::ntp_uplink(),
            // Relaxed demo policy: trust the local NTP clock at low stakes (the default high-stakes
            // floor correctly withholds clock_ms until authenticated peers agree + distance-bound).
            TimePolicy {
                required_uncertainty_ns: 10_000_000, // 10 ms, above the OS clock's 5 ms
                floor: ndn_time::provenance::StakesFloor::low(),
                ..TimePolicy::default()
            },
            vec![Box::new(OsClock::new(5_000_000))], // 5 ms OS clock as the local source
            transport,
            Arc::new(DevAuth),
            clock.clone(),
        ));
        // Run the live loop (RX-ingest task + cadence tick/beacon) and report status each second:
        // clock_ms is the disciplined estimate, peer_ingests counts beacons heard over the air.
        let runner = svc.clone();
        tokio::spawn(async move { runner.run(Duration::from_millis(500)).await });
        for t in 0..secs {
            tokio::time::sleep(Duration::from_secs(1)).await;
            println!(
                "[{t:>3}s] clock_ms={:?} peer_beacons_heard={}",
                *clock.0.lock().unwrap(),
                svc.peer_ingests()
            );
        }
        Ok::<(), Box<dyn std::error::Error>>(())
    })
}
