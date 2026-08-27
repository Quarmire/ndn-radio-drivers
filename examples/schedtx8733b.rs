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
        dev.dl_rsvd_page(page as u8, &dot11)?;
        println!(
            "  loaded {} B frame into rsvd page {page} (MGQ_TRI_HEAD={head} = 0x{head:03x}), \
             tagged knob={knob} idx={idx}",
            dot11.len()
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
