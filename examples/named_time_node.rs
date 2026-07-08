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
    let ch: u8 = std::env::args().nth(1).and_then(|s| s.parse().ok()).unwrap_or(36);
    let secs: u64 = std::env::args().nth(2).and_then(|s| s.parse().ok()).unwrap_or(8);
    let rt = tokio::runtime::Builder::new_multi_thread().enable_all().build()?;
    rt.block_on(async move {
        // Open + pipeline RX: spawn_rx_pump keeps 4 bulk-IN transfers in flight (async-URB path),
        // so a busy channel isn't dropped between reads and each frame's stamp is taken promptly.
        let radio = Arc::new(LibUsbRtl88xxBackend::open_monitor(ch)?);
        let _pump = radio.spawn_rx_pump(4);
        println!("named-time node up: 8812EU ch{ch}, RX pump depth 4");

        let clock = Arc::new(PrintClock(Mutex::new(None)));
        let transport = Arc::new(FrameIoTransport::new(radio as Arc<dyn FrameIo>));
        let svc = Arc::new(TimeService::new(
            0,
            KeyId(0),
            ClockCapability::ntp_uplink(),
            // Relaxed demo policy: a low-stakes lone node that trusts its own NTP clock (no peer
            // consensus / distance-bounding needed). The default high-stakes floor correctly
            // withholds clock_ms until authenticated peers agree — which is the 2-node case.
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
        // Run the live loop (RX-ingest task + cadence tick/beacon) for `secs`.
        let _ = tokio::time::timeout(Duration::from_secs(secs), svc.run(Duration::from_millis(500))).await;
        println!("disciplined clock_ms = {:?}", *clock.0.lock().unwrap());
        Ok::<(), Box<dyn std::error::Error>>(())
    })
}
