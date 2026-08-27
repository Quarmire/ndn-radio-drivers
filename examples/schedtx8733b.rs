//! ★ RE: **scheduled TX** on the RTL8733BU via the CPU-management-queue TSF timer.
//!
//! `tx_discipline()` currently reports `PromptBounded{1 ms}` for this part: we can decide *when*
//! to transmit in software, but the frame leaves when the host gets around to it. The registers
//! below describe a hardware path that would make it `ScheduledAt` — the MAC transmits a
//! pre-loaded frame at an exact TSF instant, with the host out of the loop:
//!
//! | reg      | field                | meaning                                     |
//! |----------|----------------------|---------------------------------------------|
//! | `0x04F4` | `[27:16]` MGQ_TRI_HEAD | page index of the frame to send           |
//! |          | BIT8 / `[7:0]`       | trigger lifetime enable / lifetime          |
//! | `0x1500` | `[31:0]`             | **TSF target** (full 32-bit, not >>5)       |
//! | `0x1510` | BIT31 / BIT28 / `[26:24]` | TIMER_EN / TX_EN / TSF_SEL             |
//! | `0x1514` | `[7:0]`              | early-interrupt lead                        |
//! | `0x1518` | BIT16 / `[15:8]` / `[7:0]` | MAC_STOP / CW / AIFS                  |
//! | `0x013C` | BIT16 / BIT20        | FTISR: timer fired / early                  |
//! | `0x0138` | same bits            | FTIMR mask                                  |
//!
//! ⚠ This is REVERSE ENGINEERING, not integration. The vendor tree *defines* every one of these
//! and **programs none of them** — there is no reference sequence to copy, and this part has been
//! wedged for weeks before by poking an undocumented control. So: every touched register is saved
//! and restored, each phase is separately gated, and the run ends by proving ordinary injection
//! still works.
//!
//! Phases (`NDN_SCHED_PHASE`), in deliberate order — each is a precondition for the next:
//!   * `probe` (default) — read the registers at rest and prove they are *writable* at all
//!                         (the USB register window has to reach page 0x15), plus measure the
//!                         port-TSF tick, since this chip's TSF counts 4 us, not 1 us.
//!   * `timer`           — arm the timer with **no** TX and watch FTISR bit16. Does the comparator
//!                         fire, and at the programmed TSF? This isolates "the timer works" from
//!                         "a frame comes out", which are very different failures.
//!   * `tx`              — load a reserved page, point MGQ_TRI_HEAD at it, set TX_EN, and see
//!                         whether a frame actually airs. Needs a witness radio.
//!
//! Usage: sudo ./schedtx8733b [channel]
//!   NDN_SCHED_PHASE=probe|timer|tx   NDN_SCHED_DELAY_MS=200   NDN_SCHED_REPS=10
//!   NDN_SCHED_PAGE=0   NDN_SCHED_HEAD=<abs page>   NDN_SCHED_CW=0   NDN_SCHED_AIFS=2
//!   NDN_SCHED_TSFSEL=0   NDN_SCHED_LIFETIME=0   NDN_SCHED_KNOB=7  NDN_SCHED_IDX=0
use bytes::Bytes;
use ndn_radio_drivers::{BROADCAST, FrameFormat, InjectFrame, Rtl8733buBackend, TxIntent, frame};
use ndn_radio_drivers::FaceError as FaceErr;
use std::time::{Duration, Instant};

const REG_FTIMR: u16 = 0x0138;
const REG_FTISR: u16 = 0x013C;
const REG_TRI_CTRL: u16 = 0x04F4;
const REG_TX_TIMER: u16 = 0x1500;
const REG_CTRL: u16 = 0x1510;
const REG_EARLY: u16 = 0x1514;
const REG_PARAM: u16 = 0x1518;

const TIMER_EN: u32 = 1 << 31;
const TX_EN: u32 = 1 << 28;
const FTISR_FIRE: u32 = 1 << 16;
const FTISR_EARLY: u32 = 1 << 20;
/// `REG_BCNQ_BDNY` as programmed by `mac_normal_mode` — the reserved-page area's first page.
const BCNQ_BDNY: u32 = 0xEC;

struct Saved {
    tri: u32,
    timer: u32,
    ctrl: u32,
    early: u8,
    param: u32,
    ftimr: u32,
}

fn save(dev: &Rtl8733buBackend) -> Result<Saved, Box<dyn std::error::Error>> {
    Ok(Saved {
        tri: dev.read32(REG_TRI_CTRL)?,
        timer: dev.read32(REG_TX_TIMER)?,
        ctrl: dev.read32(REG_CTRL)?,
        early: dev.read8(REG_EARLY)?,
        param: dev.read32(REG_PARAM)?,
        ftimr: dev.read32(REG_FTIMR)?,
    })
}

fn restore(dev: &Rtl8733buBackend, s: &Saved) -> Result<(), Box<dyn std::error::Error>> {
    // Disarm FIRST: dropping TIMER_EN/TX_EN before rewriting the target avoids arming a stale
    // comparator for one instant on the way out.
    dev.write32(REG_CTRL, s.ctrl & !(TIMER_EN | TX_EN))?;
    dev.write32(REG_TX_TIMER, s.timer)?;
    dev.write32(REG_TRI_CTRL, s.tri)?;
    dev.write8(REG_EARLY, s.early)?;
    dev.write32(REG_PARAM, s.param)?;
    dev.write32(REG_FTIMR, s.ftimr)?;
    dev.write32(REG_CTRL, s.ctrl)?;
    Ok(())
}

fn env_u32(k: &str, d: u32) -> u32 {
    std::env::var(k)
        .ok()
        .and_then(|v| {
            let v = v.trim().to_string();
            v.strip_prefix("0x")
                .map(|h| u32::from_str_radix(h, 16))
                .unwrap_or_else(|| v.parse())
                .ok()
        })
        .unwrap_or(d)
}

#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    let ch: u8 = std::env::args()
        .nth(1)
        .and_then(|s| s.parse().ok())
        .unwrap_or(36);
    let phase = std::env::var("NDN_SCHED_PHASE").unwrap_or_else(|_| "probe".into());

    let dev = Rtl8733buBackend::open()?;
    dev.bring_up_tx(ch)?;
    // The CPUMGQ comparator watches a PORT TSF (TSF_SEL picks which). The port TSF only advances
    // while the port timer runs, so without this the target is compared against a frozen counter
    // and nothing can ever fire — a null that would look exactly like "the mechanism is inert".
    dev.set_tsf_run(true)?;

    let saved = save(&dev)?;
    println!(
        "at rest: 0x04F4={:08x} 0x1500={:08x} 0x1510={:08x} 0x1514={:02x} 0x1518={:08x} \
         FTIMR={:08x} FTISR={:08x}",
        saved.tri,
        saved.timer,
        saved.ctrl,
        saved.early,
        saved.param,
        saved.ftimr,
        dev.read32(REG_FTISR)?
    );

    let result = run(&dev, &phase, ch);
    // Restore before reporting, so a failing phase still leaves the chip as we found it.
    restore(&dev, &saved)?;
    let r = result?;

    // --- the radio must still work afterwards. A scheduled-TX experiment that silently bricks
    // the ordinary data path would poison every measurement taken after it in the same session.
    let mut p = vec![0xC3u8; 64];
    p[0] = 0xEE;
    p[1] = 0xEE;
    let f = InjectFrame {
        payload: Bytes::from(p),
        tx: TxIntent::CONSERVATIVE,
        dst: BROADCAST,
        src: [0x02, 0x50, 0x33, 0x07, 0xEE, 0x00],
        addr3: None,
        addr4: None,
        htc: None,
    };
    let dot11 = frame::build_dot11(FrameFormat::RawNdn { ethertype: ndn_radio_drivers::NDN_ETHERTYPE }, &f)?;
    let mut ok = 0;
    for seq in 0..env_u32("NDN_SCHED_CTRL", 60) as u16 {
        if dev.inject_raw(&dot11, env_u32("NDN_SCHED_RATE", 4) as u8, seq).is_ok() {
            ok += 1;
        }
    }
    println!("post-test sanity: ordinary inject {ok} accepted, registers restored");
    println!("{r}");
    Ok(())
}

fn run(
    dev: &Rtl8733buBackend,
    phase: &str,
    _ch: u8,
) -> Result<String, Box<dyn std::error::Error>> {
    match phase {
        "probe" => probe(dev),
        "timer" => timer(dev, false),
        "tx" => timer(dev, true),
        "qtx" => queued_tx(dev),
        "diag" => diag(dev),
        "poll" => poll_kick(dev),
        "when" => when_does_it_air(dev),
        other => Err(format!("unknown NDN_SCHED_PHASE={other}").into()),
    }
}

/// Is the register window even reaching page 0x15, and what is the TSF tick?
fn probe(dev: &Rtl8733buBackend) -> Result<String, Box<dyn std::error::Error>> {
    let mut notes = Vec::new();
    for (name, addr, pat) in [
        ("0x1500 TX_TIMER", REG_TX_TIMER, 0xDEAD_BEEFu32),
        ("0x1510 CTRL", REG_CTRL, 0x0700_0000),
        ("0x1518 PARAM", REG_PARAM, 0x0001_0203),
        ("0x04F4 TRI_CTRL", REG_TRI_CTRL, 0x0FFF_0000),
    ] {
        let orig = dev.read32(addr)?;
        dev.write32(addr, pat)?;
        let back = dev.read32(addr)?;
        dev.write32(addr, orig)?;
        // A register that reads back exactly what we wrote is live and host-writable. All-ones or
        // all-zeros means the window does not decode this address and every later result is void.
        let verdict = if back == pat {
            "WRITABLE"
        } else if back == orig {
            "read-only/ignored"
        } else {
            "partial"
        };
        notes.push(format!("  {name}: wrote {pat:08x} read {back:08x} -> {verdict}"));
    }

    // ★ QUEUE PLUMBING. Every HARDWARE-TRIGGERED transmit on this chip is silent (beacon at TBTT,
    // CPUMGQ at its target) while host injects work perfectly. That points past the trigger and at
    // the queue: if the beacon/mgmt queue is paused, unmapped, or disabled, no page-sourced frame
    // can ever air no matter how correct the page or the timer.
    notes.push(format!(
        "  queues: TXPAUSE(0x522)={:#04x}  FWHW_TXQ_CTRL(0x420)={:#010x}  TRXDMA_CTRL(0x10C)={:#06x}",
        dev.read8(0x0522)?,
        dev.read32(0x0420)?,
        dev.read16(0x010C)?
    ));
    notes.push(format!(
        "  pages:  RQPN(0x200)={:#010x}  RQPN_NPQ(0x214)={:#010x}  BCNQ_BDNY(0x424)={:#04x}  \
         DWBCN0_CTRL(0x208)={:#010x}  CR(0x100)={:#06x}",
        dev.read32(0x0200)?,
        dev.read32(0x0214)?,
        dev.read8(0x0424)?,
        dev.read32(0x0208)?,
        dev.read16(0x0100)?
    ));

    // Port-TSF tick. This chip's RX stamp counts 4 us per tick despite the 1 us declaration; if the
    // port TSF does too, every scheduling delta below is in 4 us units and a "200 ms" target is
    // really 800 ms of wall clock.
    let t0 = dev.read_tsf()?;
    let w0 = Instant::now();
    std::thread::sleep(Duration::from_millis(500));
    let t1 = dev.read_tsf()?;
    let secs = w0.elapsed().as_secs_f64();
    let counts = t1.wrapping_sub(t0) as f64;
    let ns_per_tick = secs * 1e9 / counts;

    Ok(format!(
        "probe:\n{}\n  port TSF: {counts:.0} counts in {:.3} s -> {ns_per_tick:.0} ns/tick \
         ({:.2} us)",
        notes.join("\n"),
        secs,
        ns_per_tick / 1000.0
    ))
}

/// Arm the comparator and watch FTISR. With `do_tx`, also load a reserved page and set TX_EN.
fn timer(dev: &Rtl8733buBackend, do_tx: bool) -> Result<String, Box<dyn std::error::Error>> {
    let reps = env_u32("NDN_SCHED_REPS", 10);
    let delay_ms = env_u32("NDN_SCHED_DELAY_MS", 200) as u64;
    let tsfsel = env_u32("NDN_SCHED_TSFSEL", 0) & 0x7;
    let cw = env_u32("NDN_SCHED_CW", 0) & 0xff;
    let aifs = env_u32("NDN_SCHED_AIFS", 2) & 0xff;
    let early = env_u32("NDN_SCHED_EARLY", 0) as u8;
    let lifetime = env_u32("NDN_SCHED_LIFETIME", 0);
    let page = env_u32("NDN_SCHED_PAGE", 0);
    // MGQ_TRI_HEAD units are unknown: the page could be numbered absolutely (reserved area starts
    // at BCNQ_BDNY) or relative to that boundary. Default to absolute; NDN_SCHED_HEAD overrides so
    // both can be tried without a rebuild.
    let head = env_u32("NDN_SCHED_HEAD", BCNQ_BDNY + page) & 0xfff;

    // Measure the TSF tick first — the target arithmetic depends on it.
    let t0 = dev.read_tsf()?;
    let w0 = Instant::now();
    std::thread::sleep(Duration::from_millis(200));
    let t1 = dev.read_tsf()?;
    let ns_per_tick = w0.elapsed().as_secs_f64() * 1e9 / (t1.wrapping_sub(t0) as f64);
    let ticks_per_ms = 1e6 / ns_per_tick;
    let delay_ticks = (delay_ms as f64 * ticks_per_ms) as u32;
    println!(
        "  tick={:.2} us -> delay {delay_ms} ms = {delay_ticks} TSF counts",
        ns_per_tick / 1000.0
    );

    if do_tx {
        let knob = env_u32("NDN_SCHED_KNOB", 7) as u8;
        let idx = env_u32("NDN_SCHED_IDX", 0) as u8;
        let mut p = vec![0xC3u8; 128];
        p[0] = idx;
        p[1] = knob;
        p[2] = 0xC3;
        let f = InjectFrame {
            payload: Bytes::from(p),
            tx: TxIntent::CONSERVATIVE,
            dst: BROADCAST,
            src: [0x02, 0x50, 0x33, 0x07, knob, idx],
            addr3: None,
            addr4: None,
            htc: None,
        };
        let dot11 = frame::build_dot11(
            FrameFormat::RawNdn { ethertype: ndn_radio_drivers::NDN_ETHERTYPE },
            &f,
        )?;
        // ★ PAGE LAYOUT. The vendor stores [TXDESC][frame] in a reserved page; our dl_rsvd_page
        // stores only the bare frame because its own descriptor is a transport header consumed by
        // the download path. NDN_SCHED_PAGEDESC=0 reproduces the old (descriptor-less) page so the
        // two layouts can be A/B'd in one session rather than argued about.
        let with_desc = env_u32("NDN_SCHED_PAGEDESC", 1) != 0;
        if with_desc {
            dev.dl_rsvd_page_frame(page as u8, &dot11, env_u32("NDN_SCHED_RATE", 4) as u8, 0)?;
        } else {
            dev.dl_rsvd_page(page as u8, &dot11)?;
        }
        println!(
            "  loaded {} B frame into rsvd page {page} (MGQ_TRI_HEAD={head} = 0x{head:03x}), \
             page layout = {}, tagged knob={knob} idx={idx}",
            dot11.len(),
            if with_desc { "[TXDESC][frame]" } else { "[frame] (old, no descriptor)" }
        );
        let tri = (head << 16) | if lifetime > 0 { (1 << 8) | (lifetime & 0xff) } else { 0 };
        dev.write32(REG_TRI_CTRL, tri)?;
        println!("  0x04F4 <- {tri:08x} (readback {:08x})", dev.read32(REG_TRI_CTRL)?);
    }

    // ⚠ BISECT KNOBS. Arming this timer was measured to break the ORDINARY TX path inside the same
    // process (control injects 200/200 -> 100/200, and 197 heard -> 0), so which write does it is
    // the question that has to be answered before any of this can be trusted.
    if env_u32("NDN_SCHED_NOPARAM", 0) == 0 {
        dev.write32(REG_PARAM, (cw << 8) | aifs)?;
        dev.write8(REG_EARLY, early)?;
    }
    // FTIMR is the *firmware* timer interrupt MASK: unmasking hands the on-chip WLAN CPU an
    // interrupt for a queue nobody set up. The status bits normally latch regardless of the mask,
    // so skipping this should still let us observe the fire.
    if env_u32("NDN_SCHED_NOMASK", 0) == 0 {
        let ftimr = dev.read32(REG_FTIMR)?;
        dev.write32(REG_FTIMR, ftimr | FTISR_FIRE | FTISR_EARLY)?;
    }

    let mut fired = 0u32;
    let mut deltas: Vec<i64> = Vec::new();
    for rep in 0..reps {
        // Clear any latched status (write-1-to-clear is the usual convention for these).
        dev.write32(REG_FTISR, FTISR_FIRE | FTISR_EARLY)?;
        let now = dev.read_tsf()?;
        let target = (now as u32).wrapping_add(delay_ticks);
        dev.write32(REG_TX_TIMER, target)?;
        let ctrl = TIMER_EN | (tsfsel << 24) | if do_tx { TX_EN } else { 0 };
        if env_u32("NDN_SCHED_NOARM", 0) == 0 {
            dev.write32(REG_CTRL, ctrl)?;
        }
        let armed_back = dev.read32(REG_CTRL)?;

        let host_start = Instant::now();
        // Poll past the target by a wide margin: a comparator that fires late is a different
        // finding from one that never fires, and only a generous deadline can tell them apart.
        let deadline = Duration::from_millis(delay_ms * 3 + 500);
        let mut isr = 0u32;
        let mut fire_tsf = 0u64;
        while host_start.elapsed() < deadline {
            isr = dev.read32(REG_FTISR)?;
            if isr & (FTISR_FIRE | FTISR_EARLY) != 0 {
                fire_tsf = dev.read_tsf()?;
                break;
            }
        }
        let el = host_start.elapsed().as_secs_f64() * 1e3;
        if isr & (FTISR_FIRE | FTISR_EARLY) != 0 {
            fired += 1;
            let d = (fire_tsf as u32).wrapping_sub(target) as i32 as i64;
            deltas.push(d);
            if rep < 5 {
                println!(
                    "  rep{rep}: FIRED isr={isr:08x} after {el:.1} ms, tsf-target={d} counts \
                     ({:.0} us late), ctrl now {:08x}",
                    d as f64 * ns_per_tick / 1000.0,
                    dev.read32(REG_CTRL)?
                );
            }
        } else if rep < 5 {
            println!(
                "  rep{rep}: no fire within {el:.0} ms (armed ctrl readback {armed_back:08x}, \
                 timer {:08x}, tsf now {:08x}, target {target:08x})",
                dev.read32(REG_TX_TIMER)?,
                dev.read_tsf()? as u32
            );
        }
        dev.write32(REG_CTRL, 0)?;
    }

    let summary = if deltas.is_empty() {
        format!("timer(do_tx={do_tx}): {fired}/{reps} fired — comparator INERT")
    } else {
        deltas.sort_unstable();
        let med = deltas[deltas.len() / 2];
        format!(
            "timer(do_tx={do_tx}): {fired}/{reps} fired; median tsf-target = {med} counts \
             ({:.0} us), spread {}..{}",
            med as f64 * ns_per_tick / 1000.0,
            deltas[0],
            deltas[deltas.len() - 1]
        )
    };
    Ok(summary)
}

/// ★ The decisive test, and a different theory of the mechanism.
///
/// `build_data_txdesc` sends ordinary injects with **QSEL = MGT (0x12)** — and CPUMGQ is the *CPU
/// Management Queue*. The bisect showed the arming write alone captures the TX path (injects are
/// accepted but never air), which is exactly what queue capture looks like. If that is right, the
/// frame the timer releases is not the reserved page at all: it is whatever we injected into MGT.
/// That would be `inject_at_clock` directly.
///
/// The trap in measuring this is that the witness's clock is not ours, so "did it air at the
/// target?" cannot be answered by comparing stamps across nodes. This avoids the question: loop at
/// a FIXED period and alternate the delay between short and long. If frames air at the target, the
/// witness's own inter-arrival gaps alternate long/short. If they air the moment we inject, the
/// gaps are uniformly one period. Single clock, no common-view needed.
fn queued_tx(dev: &Rtl8733buBackend) -> Result<String, Box<dyn std::error::Error>> {
    let reps = env_u32("NDN_SCHED_REPS", 40);
    let period_ms = env_u32("NDN_SCHED_PERIOD_MS", 400) as u64;
    let d_short = env_u32("NDN_SCHED_D1", 50) as u64;
    let d_long = env_u32("NDN_SCHED_D2", 350) as u64;
    let tsfsel = env_u32("NDN_SCHED_TSFSEL", 0) & 0x7;
    let rate = env_u32("NDN_SCHED_RATE", 4) as u8;
    let knob = env_u32("NDN_SCHED_KNOB", 9) as u8;

    let t0 = dev.read_tsf()?;
    let w0 = Instant::now();
    std::thread::sleep(Duration::from_millis(200));
    let ns_per_tick = w0.elapsed().as_secs_f64() * 1e9 / (dev.read_tsf()?.wrapping_sub(t0) as f64);
    let ticks_per_ms = 1e6 / ns_per_tick;
    println!(
        "  tick={:.2} us; period {period_ms} ms, delay alternates {d_short}/{d_long} ms, \
         knob={knob} (even seq = short, odd = long)",
        ns_per_tick / 1000.0
    );

    dev.write32(REG_PARAM, (env_u32("NDN_SCHED_CW", 0) << 8) | env_u32("NDN_SCHED_AIFS", 2))?;
    let mut sent = 0u32;
    let start = Instant::now();
    for rep in 0..reps {
        let d = if rep % 2 == 0 { d_short } else { d_long };
        let now = dev.read_tsf()?;
        let target = (now as u32).wrapping_add((d as f64 * ticks_per_ms) as u32);
        dev.write32(REG_TX_TIMER, target)?;
        dev.write32(REG_CTRL, TIMER_EN | TX_EN | (tsfsel << 24))?;

        let mut pl = vec![0xC3u8; 96];
        pl[0] = (rep % 2) as u8;
        pl[1] = knob;
        pl[2] = 0xC3;
        pl[4..8].copy_from_slice(&rep.to_le_bytes());
        let f = InjectFrame {
            payload: Bytes::from(pl),
            tx: TxIntent::CONSERVATIVE,
            dst: BROADCAST,
            src: [0x02, 0x50, 0x33, 0x09, knob, (rep % 2) as u8],
            addr3: None,
            addr4: None,
            htc: None,
        };
        let dot11 = frame::build_dot11(
            FrameFormat::RawNdn { ethertype: ndn_radio_drivers::NDN_ETHERTYPE },
            &f,
        )?;
        if dev.inject_raw(&dot11, rate, rep as u16).is_ok() {
            sent += 1;
        }
        // Hold the arm past the target so the MAC has its chance, then release before the next rep.
        let next = start + Duration::from_millis(period_ms * (rep as u64 + 1));
        while Instant::now() < next {
            std::thread::sleep(Duration::from_millis(2));
        }
        dev.write32(REG_CTRL, 0)?;
    }
    Ok(format!("qtx: {sent}/{reps} injected while armed (knob={knob}); \
         witness gaps alternate => scheduled, uniform {period_ms} ms => immediate"))
}

/// ★★ THE MISSING ORACLE. Every hardware-triggered transmit on this chip is silent while host
/// injects work, and until now we could not tell WHERE it dies. The vendor ships two cheap on-chip
/// answers we never read:
///
/// * `hw_dump_bb_tx_cnt` (`rtl8733b_ops.c:2576`): "TX_EN: signal which MAC to BB, TX_ON: signal
///   which BB to RF". `0x2de0`/`0x2de2` are the OFDM pair, `0x2de4`/`0x2de6` CCK. A TX_EN delta of
///   ZERO across a comparator firing proves the MAC never even asked the baseband to transmit — the
///   failure is upstream of the PHY (queue/FIFO/descriptor), not an air or EVM problem.
/// * The **CPUMGQ packet-source FIFO** at `0x1470..0x147B` — write/read pointers, ENABLE, PAUSE, a
///   VALID bitmap and a start page. Nothing in our driver, and nothing in the vendor *host* driver,
///   ever writes these. If the FIFO is disabled, paused, or has no VALID slot, the TSF comparator
///   can fire forever and no frame can ever be fetched.
///
/// This phase samples both across three moments: idle, after ordinary injects (the positive control
/// — these DO air, so TX_EN must move), and after an armed comparator fires.
fn diag(dev: &Rtl8733buBackend) -> Result<String, Box<dyn std::error::Error>> {
    // Exercise the HAL seam rather than the raw registers, so the accessor is proven by the same
    // measurement that justified adding it.
    let bb = |dev: &Rtl8733buBackend| -> Result<(u16, u16, u16, u16), FaceErr> {
        use ndn_radio_drivers::RadioKnobs;
        let (en, on) = dev.read_tx_counters()?.unwrap_or((0, 0));
        Ok((en, on, dev.read16(0x2de4)?, dev.read16(0x2de6)?))
    };
    let fifo = |dev: &Rtl8733buBackend| -> Result<String, FaceErr> {
        Ok(format!(
            "wp={:#04x} rp={:#04x} EN={:#06x} INTFLAG={:#06x} VALID={:#06x} LIFETIME={:#06x}",
            dev.read8(0x1470)?,
            dev.read8(0x1471)?,
            dev.read16(0x1472)?,
            dev.read16(0x1476)?,
            dev.read16(0x1478)?,
            dev.read16(0x147A)?
        ))
    };
    let mut out = Vec::new();
    // The USB control round-trip is the floor under every host-timed release. Measure each shape
    // rather than assuming: a release that is one write8 cannot beat the cost of one write8.
    let cost = |n: u32, f: &dyn Fn() -> Result<(), FaceErr>| -> Result<f64, FaceErr> {
        let t = Instant::now();
        for _ in 0..n { f()?; }
        Ok(t.elapsed().as_secs_f64() * 1e6 / n as f64)
    };
    out.push(format!(
        "  usb control cost: read32 {:.0} us  write32 {:.0} us  write8 {:.0} us",
        cost(200, &|| dev.read32(0x041C).map(|_| ()))?,
        cost(200, &|| dev.write32(0x1500, 0))?,
        cost(200, &|| dev.write8(0x1514, 0))?
    ));
    let b0 = bb(dev)?;
    out.push(format!("  idle:        BB ofdm TX_EN={} TX_ON={}  cck TX_EN={} TX_ON={}", b0.0, b0.1, b0.2, b0.3));
    out.push(format!("  idle:        MGQ_FIFO {}", fifo(dev)?));
    out.push(format!("  idle:        CPU_MGQ_INFO(0x041C)={:#010x}  status(0x041A)={:#06x}",
        dev.read32(0x041C)?, dev.read16(0x041A)?));

    // Positive control: 50 ordinary injects, which we know reach the air.
    let mut p = vec![0xC3u8; 64];
    p[0] = 0xEE; p[1] = 0xEE; p[2] = 0xC3;
    let f = InjectFrame {
        payload: Bytes::from(p),
        tx: TxIntent::CONSERVATIVE,
        dst: BROADCAST,
        src: [0x02, 0x50, 0x33, 0x07, 0xEE, 0x00],
        addr3: None,
        addr4: None,
        htc: None,
    };
    let dot11 = frame::build_dot11(
        FrameFormat::RawNdn { ethertype: ndn_radio_drivers::NDN_ETHERTYPE }, &f)?;
    for seq in 0..50u16 { let _ = dev.inject_raw(&dot11, 4, seq); }
    std::thread::sleep(Duration::from_millis(300));
    let b1 = bb(dev)?;
    out.push(format!("  after 50 injects (KNOWN TO AIR): BB ofdm TX_EN={} (+{}) TX_ON={} (+{})",
        b1.0, b1.0.wrapping_sub(b0.0), b1.1, b1.1.wrapping_sub(b0.1)));

    // Now arm the comparator with TX_EN and let it fire, with the page staged.
    let page = env_u32("NDN_SCHED_PAGE", 0) as u8;
    let head = env_u32("NDN_SCHED_HEAD", BCNQ_BDNY + page as u32) & 0xfff;
    dev.dl_rsvd_page(page, &dot11)?;
    dev.write32(REG_TRI_CTRL, head << 16)?;
    let t0 = dev.read_tsf()?;
    let w0 = Instant::now();
    std::thread::sleep(Duration::from_millis(200));
    let tps = (dev.read_tsf()?.wrapping_sub(t0)) as f64 / w0.elapsed().as_secs_f64();
    let b2 = bb(dev)?;
    let mut fired = 0;
    for _ in 0..10 {
        dev.write32(REG_FTISR, FTISR_FIRE | FTISR_EARLY)?;
        let now = dev.read_tsf()?;
        dev.write32(REG_TX_TIMER, (now as u32).wrapping_add((tps * 0.15) as u32))?;
        dev.write32(REG_CTRL, TIMER_EN | TX_EN)?;
        let dl = Instant::now();
        while dl.elapsed() < Duration::from_millis(600) {
            if dev.read32(REG_FTISR)? & FTISR_FIRE != 0 { fired += 1; break; }
        }
        dev.write32(REG_CTRL, 0)?;
    }
    let b3 = bb(dev)?;
    out.push(format!("  armed+{fired}/10 fired:  BB ofdm TX_EN={} (+{}) TX_ON={} (+{})  <-- +0 means the MAC never asked the BB",
        b3.0, b3.0.wrapping_sub(b2.0), b3.1, b3.1.wrapping_sub(b2.1)));
    out.push(format!("  after fires: MGQ_FIFO {}", fifo(dev)?));
    out.push(format!("  after fires: CPU_MGQ_INFO(0x041C)={:#010x}  status(0x041A)={:#06x}",
        dev.read32(0x041C)?, dev.read16(0x041A)?));
    Ok(out.join("\n"))
}

/// ★★★ THE KICK. `diag` proved the MAC never asks the baseband to transmit when the comparator
/// fires (BB TX_EN +0 across 10 firings, vs +50 for 50 ordinary injects — a perfectly calibrated
/// oracle). So the trigger works and the *queue* is empty. `REG_CPU_MGQ_INFO` (0x041C) is the
/// missing half: `[7:0] CPUMGQ_HEAD_PG` says where the packet is, and `BIT29 CPUMGT_POLL_SET`
/// tells the MAC one is queued there. We have never written it — and it already reads
/// `HEAD_PG = 0xf6` (246), while firings set `BIT8 CPUMGQ_FW_NUM`.
///
/// Decomposed deliberately: kick with NO timer first. If a frame airs, we have a page->air path at
/// last, and scheduling is then just arming the comparator on top. If it does not, the page image
/// or the queue is still wrong and the timer was never the issue.
fn poll_kick(dev: &Rtl8733buBackend) -> Result<String, Box<dyn std::error::Error>> {
    let page = env_u32("NDN_SCHED_PAGE", 0xf6) as u8;
    let with_timer = env_u32("NDN_SCHED_WITH_TIMER", 0) != 0;
    let fifo_en = env_u32("NDN_SCHED_FIFO_EN", 0) != 0;
    let knob = env_u32("NDN_SCHED_KNOB", 7) as u8;
    let reps = env_u32("NDN_SCHED_REPS", 10);

    // ★ FRAME SIZE is the dimension where the staged page should actually pay: a kick is ONE
    // fixed-cost control write no matter how big the frame, while an inject must push the whole
    // frame over the bulk pipe every time. The 96 B default is the case most favourable to inject.
    let size = env_u32("NDN_SCHED_SIZE", 96).max(16) as usize;
    let mut pl = vec![0xC3u8; size];
    pl[0] = 0; pl[1] = knob; pl[2] = 0xC3;
    let f = InjectFrame {
        payload: Bytes::from(pl),
        tx: TxIntent::CONSERVATIVE,
        dst: BROADCAST,
        src: [0x02, 0x50, 0x33, 0x07, knob, 0],
        addr3: None,
        addr4: None,
        htc: None,
    };
    let dot11 = frame::build_dot11(
        FrameFormat::RawNdn { ethertype: ndn_radio_drivers::NDN_ETHERTYPE }, &f)?;
    dev.dl_rsvd_page(page, &dot11)?;

    let info0 = dev.read32(0x041C)?;
    // Point the CPU management queue at the page we just wrote, preserving the upper control bits.
    dev.write32(0x041C, (info0 & !0xff) | u32::from(page))?;
    if fifo_en {
        // MGQ_FIFO_EN (BIT15) with the start page in [11:0]; the FIFO reads 0x1000 (disabled) at rest.
        let e = dev.read16(0x1472)?;
        dev.write16(0x1472, (e & 0xf000) | 0x8000 | (u16::from(page) & 0x0fff))?;
    }
    let bb0 = (dev.read16(0x2de0)?, dev.read16(0x2de2)?);

    let mut tsf_ticks_150ms = 0u32;
    if with_timer {
        let t0 = dev.read_tsf()?;
        let w0 = Instant::now();
        std::thread::sleep(Duration::from_millis(200));
        let tps = (dev.read_tsf()?.wrapping_sub(t0)) as f64 / w0.elapsed().as_secs_f64();
        tsf_ticks_150ms = (tps * 0.15) as u32;
        dev.write32(REG_TRI_CTRL, (u32::from(page)) << 16)?;
    }

    let mut per_rep = Vec::new();
    for rep in 0..reps {
        let pre = dev.read16(0x2de0)?;
        // ORDER MATTERS. Arming first was measured to BLOCK the kick (10 kicks -> 1 transmit).
        // The natural design is the reverse: POLL_SET queues the packet, then the armed comparator
        // releases it at the target. NDN_SCHED_ORDER=arm_first reproduces the blocking variant.
        let arm_first = std::env::var("NDN_SCHED_ORDER").map(|v| v == "arm_first").unwrap_or(false);
        let arm = |d: &Rtl8733buBackend| -> Result<(), FaceErr> {
            d.write32(REG_FTISR, FTISR_FIRE | FTISR_EARLY)?;
            let now = d.read_tsf()?;
            d.write32(REG_TX_TIMER, (now as u32).wrapping_add(tsf_ticks_150ms))?;
            // NDN_SCHED_CTRLVAL overrides the arm word so TIMER_EN (BIT31) and TX_EN (BIT28)
            // can be separated against the BB oracle: which bit actually captures the TX path?
            let ctrl = match std::env::var("NDN_SCHED_CTRLVAL").ok() {
                Some(v) => u32::from_str_radix(v.trim_start_matches("0x"), 16).unwrap_or(TIMER_EN | TX_EN),
                None => TIMER_EN | TX_EN,
            };
            d.write32(REG_CTRL, ctrl)
        };
        let kick = |d: &Rtl8733buBackend| -> Result<(), FaceErr> {
            // Clear any stale poll first, then set. CPUMGT_POLL_CLR is BIT27.
            let v = d.read32(0x041C)?;
            d.write32(0x041C, (v | (1 << 27)) & !(1 << 29))?;
            let v = d.read32(0x041C)?;
            d.write32(0x041C, (v & !(1 << 27)) | (1 << 29))
        };
        if with_timer && arm_first {
            arm(dev)?;
            kick(dev)?;
        } else if with_timer {
            kick(dev)?;
            arm(dev)?;
        } else {
            kick(dev)?;
        }
        std::thread::sleep(Duration::from_millis(if with_timer { 250 } else { 60 }));
        if with_timer {
            dev.write32(REG_CTRL, 0)?;
        }
        // With the timer armed exactly ONE frame goes out and then the queue latches
        // CPUMGQ_FW_NUM (BIT8) — consistent with the comparator having released the queued packet
        // and the poll state needing a reset before the next one. NDN_SCHED_RESET=1 does the full
        // reset each rep: clear the poll, and re-stage the page.
        // Per-rep BB delta + the comparator's own status: this shows exactly WHICH rep transmits
        // and whether the timer fired at all on the ones that do not.
        per_rep.push(format!(
            "r{rep}:tx+{} isr={:#x} ctrl={:#x} info={:#x}",
            dev.read16(0x2de0)?.wrapping_sub(pre),
            dev.read32(REG_FTISR)? & (FTISR_FIRE | FTISR_EARLY),
            dev.read32(REG_CTRL)?,
            dev.read32(0x041C)?
        ));
        if env_u32("NDN_SCHED_RESET", 0) != 0 {
            // ★ BIT8 CPUMGQ_FW_NUM LATCHES on the first scheduled transmit and never clears by
            // itself — the per-rep trace showed r0 transmitting and r1..r5 silent with info stuck
            // at 0x1f6. Clearing POLL_SET alone is not enough; BIT8 must be knocked down too.
            let v = dev.read32(0x041C)?;
            dev.write32(0x041C, (v | (1 << 27)) & !(1 << 29) & !(1 << 8))?;
            let v = dev.read32(0x041C)?;
            dev.write32(0x041C, v & !(1 << 27) & !(1 << 8))?;
            // BIT8 is hardware-driven status and will NOT clear by writing it. The documented way
            // to flush this queue is MAC_STOP_CPUMGQ (0x1518 BIT16): stop, then release.
            if env_u32("NDN_SCHED_MACSTOP", 1) != 0 {
                let pv = dev.read32(REG_PARAM)?;
                dev.write32(REG_PARAM, pv | (1 << 16))?;
                std::thread::sleep(Duration::from_millis(2));
                dev.write32(REG_PARAM, pv & !(1 << 16))?;
            }
            dev.dl_rsvd_page(page, &dot11)?;
        }
    }
    let bb1 = (dev.read16(0x2de0)?, dev.read16(0x2de2)?);
    dev.write32(0x041C, info0)?;
    if fifo_en {
        dev.write16(0x1472, 0x1000)?;
    }
    Ok(format!(
        "poll_kick(page={page:#04x}, timer={with_timer}, fifo_en={fifo_en}, reps={reps}):\n  \
         BB ofdm TX_EN {} -> {} (+{})   TX_ON {} -> {} (+{})\n  \
         CPU_MGQ_INFO {info0:#010x} -> {:#010x}   status(0x041A)={:#06x}   MGQ_FIFO EN={:#06x} VALID={:#06x} wp={:#04x} rp={:#04x}",
        bb0.0, bb1.0, bb1.0.wrapping_sub(bb0.0),
        bb0.1, bb1.1, bb1.1.wrapping_sub(bb0.1),
        dev.read32(0x041C)?, dev.read16(0x041A)?,
        dev.read16(0x1472)?, dev.read16(0x1478)?, dev.read8(0x1470)?, dev.read8(0x1471)?
    ) + "\n  per-rep: " + &per_rep.join("  "))
}

/// ★★★ IS THE SINGLE FRAME ACTUALLY *SCHEDULED*? The comparator-armed path airs exactly one frame
/// per bring-up. Two readings fit that equally well: the timer released it at the target, or the
/// kick escaped before `TIMER_EN` took hold. Only the frame's TIMING separates them — and this
/// needs no witness and no cross-clock comparison at all.
///
/// Arm with target = now + D, kick, then poll the BB TX_EN counter and record the host elapsed
/// time at the instant it increments. The USB control read is ~250 us, which is nothing against
/// D values of tens to hundreds of milliseconds. If the measured latency TRACKS D, the hardware
/// scheduled it. If it is ~0 regardless of D, the kick simply raced the arm.
///
/// One measurement per process, because the queue latches after the first transmit.
fn when_does_it_air(dev: &Rtl8733buBackend) -> Result<String, Box<dyn std::error::Error>> {
    let page = env_u32("NDN_SCHED_PAGE", 0xf6) as u8;
    let d_ms = env_u32("NDN_SCHED_D", 200) as u64;
    let armed = env_u32("NDN_SCHED_ARMED", 1) != 0;
    let knob = env_u32("NDN_SCHED_KNOB", 7) as u8;

    let mut pl = vec![0xC3u8; 96];
    pl[0] = 0; pl[1] = knob; pl[2] = 0xC3;
    let f = InjectFrame {
        payload: Bytes::from(pl),
        tx: TxIntent::CONSERVATIVE,
        dst: BROADCAST,
        src: [0x02, 0x50, 0x33, 0x07, knob, 0],
        addr3: None, addr4: None, htc: None,
    };
    let dot11 = frame::build_dot11(
        FrameFormat::RawNdn { ethertype: ndn_radio_drivers::NDN_ETHERTYPE }, &f)?;
    dev.dl_rsvd_page(page, &dot11)?;
    let info0 = dev.read32(0x041C)?;
    dev.write32(0x041C, (info0 & !0xff) | u32::from(page))?;

    // TSF ticks per ms, measured (this chip counts 4 us per tick).
    let t0 = dev.read_tsf()?;
    let w0 = Instant::now();
    std::thread::sleep(Duration::from_millis(200));
    let tps = (dev.read_tsf()?.wrapping_sub(t0)) as f64 / w0.elapsed().as_secs_f64();
    let target_ticks = (tps * (d_ms as f64) / 1000.0) as u32;

    // ★ JITTER MODE. The kick airs a pre-staged frame in ~0.5 ms; for a slot MAC the number that
    // matters is not the mean but the SPREAD. Unarmed (the comparator only breaks the queue), so
    // all N kicks work and we get a distribution rather than a single sample.
    let jitter_n = env_u32("NDN_SCHED_JITTER", 0);
    if jitter_n > 0 {
        // ⚠ The BB-counter poll is USB-control-limited (~250 us per read), so the latency it
        // reports is a floor, not the hardware's. NDN_SCHED_PACE_MS instead kicks on a FIXED host
        // cadence and leaves the timing to the witness's 4 us RX stamps: the spread of the stamp
        // intervals about the cadence is the real jitter of the whole host+kick path.
        let pace_ms = env_u32("NDN_SCHED_PACE_MS", 0) as u64;
        let mut lat = Vec::new();
        let start = Instant::now();
        for i in 0..jitter_n {
            let pre = dev.read16(0x2de0)?;
            if pace_ms > 0 {
                let next = start + Duration::from_millis(pace_ms * i as u64);
                while Instant::now() < next {
                    std::hint::spin_loop();
                }
            }
            let t = Instant::now();
            // ★ HEAD-TO-HEAD. NDN_SCHED_VIA=inject uses the ORDINARY path (build + bulk transfer
            // per frame) at the identical cadence, so the staged-kick's timing can be compared
            // against the thing it would replace. Without this arm, "the kick is precise" is an
            // unanchored number — the ordinary path might be just as good.
            match std::env::var("NDN_SCHED_VIA").as_deref() {
                Ok("inject") => { let _ = dev.inject_raw(&dot11, 4, i as u16); }
                // ★ FAST KICK: one 8-bit control write. BIT29 (CPUMGT_POLL_SET) lives in byte 3 of
                // 0x041C, i.e. 0x041F bit 5. The read-modify-write version cost TWO USB round trips
                // per release (~250 us each) — half the critical path was a read we did not need,
                // since the hardware triggers on the WRITE, not on the level (proved by 10/10
                // transmits when the bit was already set).
                Ok("fast") => { dev.write8(0x041F, 0x20)?; }
                _ => {
                    let v = dev.read32(0x041C)?;
                    dev.write32(0x041C, v | (1 << 29))?;
                }
            }
            if pace_ms == 0 {
                while t.elapsed() < Duration::from_millis(200) {
                    if dev.read16(0x2de0)? != pre {
                        lat.push(t.elapsed().as_secs_f64() * 1e6);
                        break;
                    }
                }
            } else {
                lat.push(0.0);
            }
            // ⚠ CONFOUND, now removed. This used to POLL_CLR (4 control transfers) and re-download
            // the whole page after EVERY kick — so the "one 125 us write8" arm was actually doing
            // far more USB work per release than the inject arm it was being compared against, and
            // its tails were correspondingly worse. The page PERSISTS across kicks (10 kicks from a
            // single download, measured in the `poll` phase), so none of it is needed.
            // NDN_SCHED_RESTAGE=1 restores the old behaviour for comparison.
            if env_u32("NDN_SCHED_RESTAGE", 0) != 0 {
                let v = dev.read32(0x041C)?;
                dev.write32(0x041C, (v | (1 << 27)) & !(1 << 29))?;
                let v = dev.read32(0x041C)?;
                dev.write32(0x041C, v & !(1 << 27))?;
                dev.dl_rsvd_page(page, &dot11)?;
            }
        }
        dev.write32(0x041C, info0)?;
        if lat.is_empty() {
            return Ok("jitter: no transmits".into());
        }
        if lat.iter().all(|v| *v == 0.0) {
            // Paced mode: report the HOST-SIDE cost of issuing each release, which is what the
            // frame-size question is really about.
            let pre_bb = dev.read16(0x2de0)?;
            let t = Instant::now();
            // ⚠ These 50 releases must be PACED, or this measures back-to-back behaviour while
            // claiming to measure a pacing sweep — which is exactly what an earlier version did.
            let gap = Duration::from_micros(env_u32("NDN_SCHED_GAP_US", 0) as u64);
            for i in 0..50u32 {
                if !gap.is_zero() {
                    let next = t + gap * i;
                    while Instant::now() < next { std::hint::spin_loop(); }
                }
                match std::env::var("NDN_SCHED_VIA").as_deref() {
                    Ok("inject") => { let _ = dev.inject_raw(&dot11, 4, i as u16); }
                    _ => { dev.write8(0x041F, 0x20)?; }
                }
            }
            let per = t.elapsed().as_secs_f64() * 1e6 / 50.0;
            // ★ AIRTIME, measured on the DUT so witness-side reception cannot confound it: BB TX_EN
            // counts MAC->baseband transmit requests. If one mechanism issues more requests per
            // logical frame, it is genuinely burning more airtime.
            std::thread::sleep(Duration::from_millis(200));
            let after = dev.read16(0x2de0)?;
            return Ok(format!(
                "gap {} us: release cost {per:.0} us   BB TX_EN +{} for 50 releases  \
                 => {:.2} transmits per logical frame",
                env_u32("NDN_SCHED_GAP_US", 0),
                after.wrapping_sub(pre_bb),
                f64::from(after.wrapping_sub(pre_bb)) / 50.0
            ));
        }
        lat.sort_by(|a, b| a.partial_cmp(b).unwrap());
        let n = lat.len();
        let mean = lat.iter().sum::<f64>() / n as f64;
        return Ok(format!(
            "kick->air latency over {n}/{jitter_n} kicks: min {:.0} us  p50 {:.0} us  p90 {:.0} us  \
             max {:.0} us  mean {mean:.0} us  (spread {:.0} us)",
            lat[0], lat[n / 2], lat[n * 9 / 10], lat[n - 1], lat[n - 1] - lat[0]
        ));
    }

    let pre = dev.read16(0x2de0)?;
    if armed {
        dev.write32(REG_FTISR, FTISR_FIRE | FTISR_EARLY)?;
        let now = dev.read_tsf()?;
        dev.write32(REG_TX_TIMER, (now as u32).wrapping_add(target_ticks))?;
        dev.write32(REG_CTRL, TIMER_EN | TX_EN)?;
    }
    let t_kick = Instant::now();
    let v = dev.read32(0x041C)?;
    dev.write32(0x041C, v | (1 << 29))?;

    // Poll until the baseband actually transmits.
    let mut aired_ms = f64::NAN;
    let deadline = Duration::from_millis(d_ms * 3 + 1500);
    while t_kick.elapsed() < deadline {
        if dev.read16(0x2de0)? != pre {
            aired_ms = t_kick.elapsed().as_secs_f64() * 1e3;
            break;
        }
    }
    let isr = dev.read32(REG_FTISR)?;
    dev.write32(REG_CTRL, 0)?;
    dev.write32(0x041C, info0)?;
    Ok(format!(
        "when(armed={armed}, D={d_ms} ms): aired at {:.1} ms after the kick  \
         (isr={isr:#x}, target={target_ticks} ticks, {:.0} ticks/s)\n  \
         => latency ~D means SCHEDULED; latency ~0 regardless of D means the kick raced the arm",
        aired_ms, tps
    ))
}
