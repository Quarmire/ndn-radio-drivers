//! **One H2C command id per invocation, with a witness on the air.**
//!
//! ## Why one id per run
//!
//! The RTL8812AU firmware's command dispatcher is a Keil `?C?CCASE` table at code `0x52CE`
//! (image `0x12EE`), 18 entries of `DW handler; DB id`, matched with a single `XRL A,R0` —
//! an **exact 8-bit compare on the raw mailbox byte**. No mask, no shift, no class field.
//! Implemented ids: `00 01 0F 10 11 12 14 1C 1E 20 24 25 40 41 42 47 49 87`. Everything else
//! falls to the default arm at code `0x535C`:
//!
//! ```text
//! 90 01 c0  e0  44 01  f0        REG 0x01C0 |= 0x01      ("unknown H2C id")
//! 90 9f b2  e0  90 01 c2  f0     REG 0x01C2 = xdata[9FB2] = the raw id byte
//! 22                             RET
//! ```
//!
//! Which is bit-for-bit the four-times-identical silicon result this fleet already measured for
//! `58 00 00 00` (status `0x00`→`0x01`, resp `0x00`→`0x58`). So **the echo is an error latch, not
//! an acknowledgement** — and the oracle is INVERTED: none of the 18 real handlers touches
//! `0x01C0` or `0x01C2`, so *the echo NOT changing* is what "serviced" looks like.
//!
//! ⚠ `0x01C0`/`0x01C1` are sticky set-only latches — nothing in the image ever clears them, so
//! "bit 0 is set" is worthless after the first rejection. `0x01C2` is a **full-byte store**, so
//! this probe primes it with a guaranteed-bogus id first and then reads whether the id under test
//! displaced it. That makes each invocation self-contained: no dependence on run history.
//!
//! ## Why a witness
//!
//! The transmitter cannot tell you it went silent. This radio has no `read_tx_counters()`, a
//! bulk-OUT queue absorbs writes, and the one handler on the danger list (`0x1E` with a non-zero
//! payload byte) drives `REG_TXPAUSE` — which HOLDS queues with no host-visible symptom except an
//! XDATA byte we cannot read. So a second radio counts frames on the air before and after every
//! command, and a collapse is the stop condition.
//!
//! ## Use
//!
//! ```text
//! # positive control first — a guaranteed-bogus id, to learn the rejection signature:
//! au_h2c_id ee
//! # the negative control — id 0x47 is a bare RET, dispatched and touching nothing:
//! au_h2c_id 47
//! # the C2H liveness oracle (payload byte 1 MUST be 0x00, see below):
//! au_h2c_id 1e 00 00 00
//! ```
//!
//! `NDN_AU_CH` channel (default 36) · `NDN_AU_WIT` witness backend, `xx` (a81a, default) or `none`
//! · `NDN_AU_BURST_MS` liveness burst length (default 1500).
//!
//! ☠ **`0x1E` with a NON-ZERO mailbox byte 1 writes `REG_TXPAUSE`.** Mailbox byte 1 is what the
//! firmware calls `payload[0]` (byte 0 is the id). Handler code `0x7586`:
//! `90 9f c6 f0 / 60 0f` stores payload[0] and `JZ` skips the actuator when it is zero; the
//! non-zero path runs `7b 57 / 12 4c c5` = set_txpause(reason 0x57). This probe refuses a
//! non-zero mailbox byte 1 on id `0x1E` unless `NDN_AU_ALLOW_TXPAUSE=1`, and restores `0x0522` from its
//! own baseline either way.
//!
//! ☠ **Never `0x20`.** It is SETPWRMODE, the one fully-implemented subsystem in this firmware; a
//! wrong neighbouring byte puts the MAC in a low-power state and reprograms 32 kHz sleep timing
//! behind the driver. This probe refuses it outright.

use std::sync::Arc;
use std::sync::atomic::{AtomicBool, Ordering};
use std::time::{Duration, Instant};

use ndn_frame_io::{FrameFormat, FrameIo};
use ndn_radio_drivers::{LibUsbRtl88xxBackend, Rtl8812auBackend};

/// The 18 ids the dispatch table at code `0x52CE` actually services. Read verbatim from
/// `fw/rtl8812au/rtl8812au_fw_nic.bin`; anything not in here takes the default (rejection) arm.
const IMPLEMENTED: [u8; 18] = [
    0x00, 0x01, 0x0f, 0x10, 0x11, 0x12, 0x14, 0x1c, 0x1e, 0x20, 0x24, 0x25, 0x40, 0x41, 0x42, 0x47,
    0x49, 0x87,
];

/// Firmware state that matters, all read as **bytes**: a `read32(0x1c0)` with a byte-index slip is
/// exactly how an earlier pass mislabelled which latch the default arm sets.
#[derive(Clone, Copy, Debug)]
struct Snap {
    hmetfr: u8, // 0x01CC — box-pending bits; ours must clear or the firmware stopped consuming
    s1c0: u8,   // sticky error bitmap: b0 unknown id, b1 box desync, b7 C2H emit timeout
    s1c1: u8,   // sticky: b0 H2C ring FULL (command DROPPED), b1 C2H ring full
    e1c2: u8,   // the echo — full-byte store, the only non-sticky signal here
    e1c3: u8,   // HMEBOX_0 byte-0 snapshot, written only on the box-desync path
    txpause: u8, // 0x0522 — the actuator most likely to silence this radio
}

fn snap(d: &Rtl8812auBackend) -> Result<Snap, Box<dyn std::error::Error>> {
    Ok(Snap {
        hmetfr: d.read8(0x01cc)?,
        s1c0: d.read8(0x01c0)?,
        s1c1: d.read8(0x01c1)?,
        e1c2: d.read8(0x01c2)?,
        e1c3: d.read8(0x01c3)?,
        txpause: d.read8(0x0522)?,
    })
}

fn show(tag: &str, s: &Snap) {
    println!(
        "  {tag:<9} HMETFR={:#04x}  0x1c0={:#04x} 0x1c1={:#04x}  echo(0x1c2)={:#04x} 0x1c3={:#04x}  TXPAUSE={:#04x}",
        s.hmetfr, s.s1c0, s.s1c1, s.e1c2, s.e1c3, s.txpause
    );
}

/// A broadcast data frame full of `0x42` filler — the same shape `au_witness` counts by default.
/// Broadcast on purpose: a unicast addr1 waits out an ACK timeout no monitor peer will ever send.
fn filler_frame() -> Vec<u8> {
    let mut p = Vec::with_capacity(24 + 64);
    p.extend_from_slice(&[0x08, 0x00, 0x00, 0x00]); // FC data/data, Duration 0
    p.extend_from_slice(&[0xff; 6]); // addr1 broadcast
    p.extend_from_slice(&[0x02, 0x00, 0x00, 0x00, 0x00, 0x01]); // addr2
    p.extend_from_slice(&[0xff; 6]); // addr3
    p.extend_from_slice(&[0x00, 0x00]); // SeqCtrl
    p.extend(std::iter::repeat_n(0x42u8, 64));
    p
}

/// Inject from the AU for `dur` while counting what the witness hears. Returns
/// `(offered, witnessed)` — the gap between them is the whole point of having a witness.
async fn liveness(
    au: &Arc<Rtl8812auBackend>,
    wit: &Option<Arc<dyn FrameIo>>,
    dur: Duration,
) -> (u32, Option<u32>) {
    let frame = filler_frame();
    let stop = Arc::new(AtomicBool::new(false));
    let (tx_au, tx_stop) = (au.clone(), stop.clone());
    let tx = std::thread::spawn(move || {
        let mut n = 0u32;
        while !tx_stop.load(Ordering::Relaxed) {
            // 0x04 = DESC_RATE6M. Legacy 6 Mbps: the rate this fleet MEASURED as the one a
            // second Realtek reliably demodulates for broadcast, after the HT-MCS RX finding.
            if tx_au.send_frame(&frame, 0x04).is_ok() {
                n += 1;
            }
            std::thread::sleep(Duration::from_micros(500));
        }
        n
    });

    let witnessed = match wit {
        None => {
            tokio::time::sleep(dur).await;
            None
        }
        Some(w) => {
            let deadline = Instant::now() + dur;
            let mut ours = 0u32;
            while Instant::now() < deadline {
                let left = deadline.saturating_duration_since(Instant::now());
                if left.is_zero() {
                    break;
                }
                let to = left.min(Duration::from_millis(200));
                if let Ok(Ok(f)) = tokio::time::timeout(to, w.recv_frame()).await {
                    // Match ANYWHERE in the buffer: in Raw80211 the payload is the whole
                    // 802.11 frame, header first, so matching at offset 0 finds nothing.
                    if f.payload.windows(16).any(|c| c.iter().all(|&b| b == 0x42)) {
                        ours += 1;
                    }
                }
            }
            Some(ours)
        }
    };
    stop.store(true, Ordering::Relaxed);
    (tx.join().unwrap_or(0), witnessed)
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    tokio::runtime::Runtime::new()?.block_on(run())
}

async fn run() -> Result<(), Box<dyn std::error::Error>> {
    let mut args = std::env::args().skip(1);
    let id = match args
        .next()
        .and_then(|s| u8::from_str_radix(s.trim_start_matches("0x"), 16).ok())
    {
        Some(v) => v,
        None => {
            eprintln!("usage: au_h2c_id <id-hex> [payload-byte1 byte2 byte3 (hex)]");
            std::process::exit(2);
        }
    };
    let mut payload = [id, 0, 0, 0];
    for (i, a) in args.take(3).enumerate() {
        payload[i + 1] = u8::from_str_radix(a.trim_start_matches("0x"), 16).unwrap_or(0);
    }

    // ── Refusals, before a single USB write ─────────────────────────────────────────────────
    if id == 0x20 {
        eprintln!(
            "REFUSED: id 0x20 is SETPWRMODE. Power-save is the ONE fully-implemented subsystem in\n\
             this firmware — handler 0x76C0 drives REG_CPWM (0x012F), REG_PS_RX_INFO (0x0692), the\n\
             BCN_PSR_RPT block and the 32K_CTRL sleep-timing registers (0x0194/0x0198-0x019F). A\n\
             wrong parameter byte hangs the dongle in a low-power state with no host-visible cause.\n\
             It is not a tonight probe."
        );
        std::process::exit(3);
    }
    if id == 0x1e && payload[1] != 0 && std::env::var_os("NDN_AU_ALLOW_TXPAUSE").is_none() {
        eprintln!(
            "REFUSED: id 0x1E with payload byte 1 = {:#04x}. Handler code 0x7586 does\n\
             `90 9f c6 f0 / 60 0f` — the JZ skips the actuator ONLY when that byte is zero; a\n\
             non-zero value reaches set_txpause (code 0x4CC5, reason 0x57) and writes REG_TXPAUSE.\n\
             Send `au_h2c_id 1e 00 00 00` first. Override with NDN_AU_ALLOW_TXPAUSE=1.",
            payload[1]
        );
        std::process::exit(3);
    }

    let ch: u8 = std::env::var("NDN_AU_CH")
        .ok()
        .and_then(|s| s.parse().ok())
        .unwrap_or(36);
    let burst = Duration::from_millis(
        std::env::var("NDN_AU_BURST_MS")
            .ok()
            .and_then(|s| s.parse().ok())
            .unwrap_or(1500),
    );

    // C2H buffers are recognised and dropped by the RX parser unless this is set; the pump prints
    // `C2H8812AU id=.. len=.. payload=[..]` on stderr for each one. Set before the pump starts.
    // SAFETY: single-threaded here — no pump, no witness, nothing else has started yet.
    unsafe { std::env::set_var("NDN_C2H_DBG", "1") };

    let au = Arc::new(Rtl8812auBackend::open()?.with_format(FrameFormat::Raw80211));
    au.bring_up_monitor(ch)?;
    au.spawn_rx_pump(8);

    let wit: Option<Arc<dyn FrameIo>> = match std::env::var("NDN_AU_WIT").as_deref() {
        Ok("none") => {
            eprintln!("⚠ NO WITNESS. Nothing in this run can tell you the transmitter survived.");
            None
        }
        _ => {
            let w = Arc::new(
                LibUsbRtl88xxBackend::open_monitor(ch)?.with_format(FrameFormat::Raw80211),
            );
            w.spawn_rx_pump(8);
            Some(w)
        }
    };

    let known = IMPLEMENTED.contains(&id);
    println!(
        "\nRTL8812AU H2C probe — id {id:#04x} ({}), payload {:02x?}, ch{ch}\n",
        if known {
            "IN the dispatch table"
        } else {
            "NOT in the dispatch table"
        },
        &payload[1..]
    );

    // ── 1. Baseline, including the airtime the radio is actually producing ───────────────────
    let base = snap(&au)?;
    show("baseline", &base);
    let (off0, wit0) = liveness(&au, &wit, burst).await;
    println!(
        "  liveness  before: offered {off0}, witnessed {}",
        fmt(wit0)
    );
    if let Some(0) = wit0 {
        eprintln!(
            "\nSTOP: the witness heard nothing BEFORE any command was sent. A null after the probe\n\
             would then prove nothing. Fix the witness (channel? NDN_AU_WIT=none to bypass) first."
        );
        std::process::exit(4);
    }

    // ── 2. Prime the echo latch with a guaranteed-bogus id ───────────────────────────────────
    // 0x1C2 is a full-byte store, so priming it makes the verdict independent of run history —
    // unlike 0x1C0/0x1C1, which are sticky and useless after the first rejection.
    let primer = if id == 0xee { 0xed } else { 0xee };
    zero_ext_boxes(&au)?;
    au.h2c([primer, 0, 0, 0])?;
    std::thread::sleep(Duration::from_millis(30));
    let primed = snap(&au)?;
    show("primed", &primed);
    if primed.e1c2 != primer {
        eprintln!(
            "\nSTOP: primed with {primer:#04x} but echo reads {:#04x}. The rejection path is not\n\
             behaving as the image predicts, so no verdict below can be trusted. Investigate\n\
             before sending {id:#04x}.",
            primed.e1c2
        );
        std::process::exit(5);
    }

    // ── 3. The command under test ────────────────────────────────────────────────────────────
    // ⚠ The firmware reads EIGHT bytes per command: HMEBOX_n (0x01D0+4n) AND HMEBOX_EXT_n
    // (0x01F0+4n), copied into one contiguous slot (ingest at code 0x62CE). `h2c()` writes only
    // the main box, so payload bytes 3..6 would be whatever the EXT box last held. Mainline writes
    // the EXT box FIRST for exactly this reason (rtw88 fw.c:429-430). Zero all four.
    zero_ext_boxes(&au)?;
    let sent = au.h2c(payload);
    if let Err(e) = &sent {
        eprintln!("\nSTOP: h2c() refused: {e}");
        eprintln!("  A stuck HMETFR bit means the firmware is no longer consuming commands.");
        eprintln!("  Nothing further is reachable. Power-cycle before continuing.");
        std::process::exit(6);
    }
    std::thread::sleep(Duration::from_millis(30));
    let after = snap(&au)?;
    show("after", &after);

    // ── 4. Verdict ───────────────────────────────────────────────────────────────────────────
    let displaced = after.e1c2 == id && id != primer;
    let ring_full = after.s1c1 & 0x01 != 0 && primed.s1c1 & 0x01 == 0;
    println!();
    if ring_full {
        println!("  VERDICT: VOID — 0x1C1 bit0 newly set = H2C ring FULL, the command was DROPPED");
        println!("           before dispatch (code 0x6301). Retry with more spacing.");
    } else if displaced {
        println!(
            "  VERDICT: IGNORED — the echo moved {:#04x} -> {id:#04x}, i.e. the default arm at",
            primed.e1c2
        );
        println!("           code 0x535C ran. {id:#04x} is NOT implemented by this firmware.");
        if known {
            println!(
                "  ★ AND THAT CONTRADICTS THE TABLE. {id:#04x} IS in the dispatch table read from"
            );
            println!(
                "    the blob. Either the table walk is wrong or the image on this dongle differs."
            );
        }
    } else if after.e1c2 == primed.e1c2 {
        println!(
            "  VERDICT: SERVICED — the echo still reads {:#04x}. None of the 18 real handlers",
            primed.e1c2
        );
        println!(
            "           touches 0x01C0 or 0x01C2; only the default arm does. So {id:#04x} was"
        );
        println!("           dispatched to a handler.");
        if !known {
            println!(
                "  ★ AND THAT CONTRADICTS THE TABLE. {id:#04x} is NOT in the table read from the"
            );
            println!(
                "    blob — which would mean a SECOND entry path (a ROM-level dispatcher). This"
            );
            println!(
                "    is the single most interesting result this probe can produce. Stop and think."
            );
        }
    } else {
        println!(
            "  VERDICT: UNEXPECTED — echo {:#04x} -> {:#04x}, neither the primer nor the id.",
            primed.e1c2, after.e1c2
        );
        println!("           Do not send another id until this is explained.");
    }
    for (bit, what) in [
        (0x02u8, "0x1C0 b1: HMETFR/box-index desync (code 0x63BD)"),
        (0x20, "0x1C0 b5: a TX-quiesce wait timed out (code 0x4D85)"),
        (
            0x80,
            "0x1C0 b7: C2H emit gave up waiting on REG 0x0296 (code 0x8D85)",
        ),
    ] {
        if after.s1c0 & bit != 0 && primed.s1c0 & bit == 0 {
            println!("  ⚠ NEW ERROR LATCH — {what}");
        }
    }
    if after.s1c1 & 0x02 != 0 && primed.s1c1 & 0x02 == 0 {
        println!(
            "  ⚠ NEW ERROR LATCH — 0x1C1 b1: the C2H report ring overflowed; reports were DROPPED."
        );
    }

    // ── 5. Did the radio survive? ────────────────────────────────────────────────────────────
    let (off1, wit1) = liveness(&au, &wit, burst).await;
    println!(
        "\n  liveness  after:  offered {off1}, witnessed {}",
        fmt(wit1)
    );
    let mut damaged = false;
    if let (Some(a), Some(b)) = (wit0, wit1) {
        let ratio = if a == 0 { 1.0 } else { b as f64 / a as f64 };
        println!("  on-air ratio after/before: {ratio:.2}");
        if ratio < 0.20 {
            damaged = true;
            println!(
                "  ☠ THE TRANSMITTER WENT SILENT. Offered stayed at {off1} — so the host is still"
            );
            println!(
                "    handing USB buffers and only the AIR stopped. That is the TXPAUSE signature."
            );
        }
    }
    let now = snap(&au)?;
    if now.txpause != base.txpause {
        damaged = true;
        println!(
            "  ☠ TXPAUSE MOVED {:#04x} -> {:#04x} — the firmware gated the transmit queues.",
            base.txpause, now.txpause
        );
        println!(
            "    Restoring 0x0522 = {:#04x} (host write; the firmware's own reason latch at",
            base.txpause
        );
        println!("    XDATA 0x9D05 is not host-readable, so this is the only recovery available).");
        au.set_tx_pause(base.txpause)?;
        let (off2, wit2) = liveness(&au, &wit, burst).await;
        println!(
            "  liveness  recovered: offered {off2}, witnessed {}",
            fmt(wit2)
        );
    }
    println!(
        "\n  {}",
        if damaged {
            "STOP. Do not send the next id. Record this one as DANGEROUS and power-cycle."
        } else {
            "Radio still transmitting. Safe to proceed to the next id."
        }
    );
    println!("  (C2H reports, if any, printed on stderr as `C2H8812AU id=...`.)");
    Ok(())
}

/// Zero every extended mailbox so the 4 payload bytes `h2c()` does not write are defined.
fn zero_ext_boxes(d: &Rtl8812auBackend) -> Result<(), Box<dyn std::error::Error>> {
    for n in 0..4u16 {
        d.write32(0x01f0 + n * 4, 0)?;
    }
    Ok(())
}

fn fmt(v: Option<u32>) -> String {
    v.map(|n| n.to_string()).unwrap_or_else(|| "n/a".into())
}
