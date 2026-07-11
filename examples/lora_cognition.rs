//! Live LoRa **cognition loop**: the real named-radio control plane driving a real SX1262 dongle.
//!
//! Closes the loop the `lora_link` demo left open. Each node:
//!   - SENSE — feeds every received frame's real per-frame RSSI into [`MediumState::observe_rx`];
//!   - DECIDE — every tick runs [`RadioPolicy::decide`] over that medium state + a `NameContext`
//!     whose priority cycles (a stand-in for changing demand), yielding a `RadioPlan`;
//!   - ACT — reads the plan's LoRa knobs (spreading factor from the measured RSSI, coding rate from
//!     the demand priority) and applies them to the dongle through the `RadioKnobs` seam, then TXes.
//!
//! SF tracks the measured link (strong → SF7, weak → SF12); CR rises for urgent/broadcast demand and
//! is link-safe (LoRa's explicit header conveys it, so the peer keeps decoding). Two close nodes both
//! read a strong RSSI and settle at the same SF, staying paired while the coding rate adapts to demand.
//!
//! Run on both OPis:
//! ```text
//! lora_cognition /dev/ttyACM0 A
//! lora_cognition /dev/ttyACM0 B
//! ```

use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

use ndn_frame_io::{FrameIo, InjectFrame, TxIntent};
use ndn_radio_cognition::{
    MediumState, MediumView, NameContext, Priority, RadioCapability, RadioId, RadioPolicy,
    prefix_hash,
};
use ndn_radio_drivers::LoraSerialBackend;
use ndn_radio_hal::RadioKnobs;

const CHANNEL: u8 = 65; // 915 MHz (US)
const NEIGHBOR: u64 = 1; // opaque key for the peer link in the sense bus
const TICK: Duration = Duration::from_millis(3000);
const DEMAND_PHASE_S: u64 = 12; // seconds per priority phase in the cycling demand

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let rt = tokio::runtime::Builder::new_multi_thread()
        .enable_all()
        .build()?;
    rt.block_on(run())
}

async fn run() -> Result<(), Box<dyn std::error::Error>> {
    let mut args = std::env::args().skip(1);
    let path = args.next().unwrap_or_else(|| "/dev/ttyACM0".into());
    let node = args.next().unwrap_or_else(|| "A".into());

    let dev = Arc::new(LoraSerialBackend::open(&path)?);
    println!("[{node}] open OK — cognition plane driving {path}");

    let radio = RadioId(0);
    let medium = Arc::new(Mutex::new(MediumState::new()));
    medium
        .lock()
        .unwrap()
        .register_radio(radio, RadioCapability::lora(vec![CHANNEL]));
    let policy = RadioPolicy::default();
    let ctx_hash = prefix_hash(&[b"ndn", b"lora-cog-demo"]);
    let start = Instant::now();

    // SENSE task: every received frame's real RSSI updates the medium state (and shows the peer).
    {
        let dev = dev.clone();
        let medium = medium.clone();
        tokio::spawn(async move {
            loop {
                match dev.recv_frame().await {
                    Ok(f) => {
                        let now = start.elapsed().as_millis() as u64;
                        medium
                            .lock()
                            .unwrap()
                            .observe_rx(radio, NEIGHBOR, f.rssi_dbm, now);
                    }
                    Err(_) => break,
                }
            }
        });
    }

    let mut last_sf = 0u8;
    let mut last_cr = 0u8;
    let mut seq = 0u32;
    println!("[{node}] t   | demand  | obsRSSI | decide           | act");
    loop {
        let now = start.elapsed().as_millis() as u64;
        let secs = start.elapsed().as_secs();

        // DEMAND: cycle the priority as a stand-in for changing name demand.
        let priority = match (secs / DEMAND_PHASE_S) % 3 {
            0 => Priority::Normal,
            1 => Priority::Urgent,
            _ => Priority::Bulk,
        };
        let mut ctx = NameContext::new(ctx_hash);
        ctx.priority = priority;

        // DECIDE (+ read the observed link for logging) — hold the lock only for the sync decision.
        let (plan, obs_rssi) = {
            let m = medium.lock().unwrap();
            (policy.decide(&ctx, &*m, now), m.neighbor_rssi(radio, NEIGHBOR))
        };

        // ACT: pull the LoRa knobs out of the plan and apply the ones that changed.
        let mut acted = Vec::new();
        let (mut sf_s, mut cr_s) = ("--".to_string(), "-".to_string());
        if let Some(alloc) = plan.allocation_for(radio) {
            if let Some(sf) = alloc.params.spreading_factor() {
                sf_s = format!("SF{sf}");
                if sf != last_sf {
                    dev.set_spreading_factor(sf)?;
                    last_sf = sf;
                    acted.push("sf");
                }
            }
            if let Some(cr) = alloc.params.coding_rate() {
                cr_s = format!("4/{}", cr + 4);
                if cr != last_cr {
                    dev.set_coding_rate(cr)?;
                    last_cr = cr;
                    acted.push("cr");
                }
            }
        }
        let fec = plan
            .allocation_for(radio)
            .and_then(|a| a.params.link_fec_redundancy)
            .unwrap_or(0);

        let rssi_s = obs_rssi
            .map(|r| format!("{r}dBm"))
            .unwrap_or_else(|| "  --  ".into());
        let act_s = if acted.is_empty() { "(steady)".into() } else { format!("applied {}", acted.join("+")) };
        println!(
            "[{node}] {secs:>3}s| {priority:<7?}| {rssi_s:>7} | {sf_s} {cr_s} FEC{fec} | {act_s}",
        );

        // TX a named "Interest" so the peer has something to measure us by.
        let body = format!("ndn/lora-cog/{node}/seq={seq}");
        let frame = InjectFrame::broadcast(body.into_bytes().into(), TxIntent::CONSERVATIVE);
        let _ = dev.inject(frame).await;
        seq += 1;

        tokio::time::sleep(TICK).await;
    }
}
