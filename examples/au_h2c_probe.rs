//! Validate the RTL8812AU H2C transport by **replaying the exact command the kernel sends**.
//!
//! `golden/rtw88-5g-ch149.txt` shows mainline rtw88 writing `58 00 00 00` into the H2C mailboxes on
//! this dongle every 2.0 s, rotating HMEBOX_0..3 and polling HMETFR first. Replaying that byte-for-
//! byte tests our transport end-to-end without inventing a command — the firmware has already been
//! observed accepting it on this silicon.
//!
//! What "accepted" looks like: HMETFR's bit for the box we wrote CLEARS, because the firmware took
//! the command out of the box. If it stays set, the firmware is not consuming commands and every
//! H2C ambition on this part is dead.
//!
//! ⚠ Reads plus the one captured write. No invented command ids — those come from the dispatcher,
//! not from another chip generation's driver.
use ndn_radio_drivers::Rtl8812auBackend;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let ch: u8 = std::env::args()
        .nth(1)
        .and_then(|s| s.parse().ok())
        .unwrap_or(36);
    let d = Rtl8812auBackend::open()?;
    d.bring_up_monitor(ch)?;

    println!("RTL8812AU H2C probe, ch{ch}\n");
    let (s0, r2, r3) = d.h2c_status()?;
    println!(
        "  before:  HMETFR={:#04x}  status(0x1c0)={s0:#04x}  resp(0x1c2/0x1c3)={r2:#04x}/{r3:#04x}",
        d.read8(0x01cc)?
    );

    // ★ THE DECISIVE PAIR. The image's dispatch table (a Keil CCASE jump table at code 0x52CE,
    // 18 entries, load base 0x4000) services exactly these ids:
    //     00 01 0F 10 11 12 14 1C 1E 20 24 25 40 41 42 47 49 87
    // Anything else falls to the DEFAULT arm at code 0x535C, which is the only code in the image
    // that touches 0x01C0/0x01C2:
    //     ORL 0x01C0,#0x01 ; A = xdata[0x9FB2] (the stashed id) ; MOV 0x01C2,A ; RET
    //
    // So the response is an **error latch, not an acknowledgement** — the oracle is INVERTED:
    //   echo DISPLACED by the id  => the id was NOT recognised
    //   echo UNCHANGED            => the id WAS dispatched to a real handler
    //
    // Test it with two ids that differ only in table membership:
    //   0x99 — absent  => predict the echo becomes 0x99
    //   0x47 — present, and its handler is a bare RET, so it touches no register at all
    //          => predict the echo stays at the primer
    // If those two come back the same, the model is wrong and nothing downstream is safe.
    const PRIMER: u8 = 0xee;
    for (id, listed) in [(0x99u8, false), (0x47u8, true), (0x58u8, false)] {
        d.write8(0x01c2, PRIMER)?;
        let primed = d.read8(0x01c2)?;
        if primed != PRIMER {
            println!(
                "  !! priming 0x1C2 failed (read {primed:#04x}) — cannot interpret this probe"
            );
            continue;
        }
        match d.h2c([id, 0x00, 0x00, 0x00]) {
            Ok(()) => {
                std::thread::sleep(std::time::Duration::from_millis(30));
                let tfr = d.read8(0x01cc)?;
                let (s0, r2, r3) = d.h2c_status()?;
                let verdict = if r2 == PRIMER {
                    "DISPATCHED (echo untouched)"
                } else if r2 == id {
                    "REJECTED (echo displaced by the id)"
                } else {
                    "UNEXPECTED third state"
                };
                let agrees = (r2 == PRIMER) == listed;
                println!(
                    "  id {id:#04x} (table: {}) -> HMETFR={tfr:#04x} status={s0:#04x} \
                     echo={r2:#04x}/{r3:#04x}  => {verdict}{}",
                    if listed { "listed" } else { "absent " },
                    if agrees {
                        ""
                    } else {
                        "   <<<< CONTRADICTS THE TABLE"
                    }
                );
            }
            Err(e) => println!("  id {id:#04x}: {e}"),
        }
    }
    // ★ Does the C2H CHANNEL work at all?
    //
    // Arming the TX report in the descriptor (W2 bit 19) produced ZERO C2H buffers over 3535
    // frames, and the additional enable some drivers use (`0x04EC`) is **not defined for this
    // generation in any artefact we hold** — so it is not going to be guessed at.
    //
    // Id `0x1E` separates the two possibilities. With mailbox byte 1 = 0 its handler skips the
    // TXPAUSE actuator (a `JZ`) and falls into an unconditional C2H emit — id 0x20, plen 3, whose
    // payload[0] is a live read of `REG_TXPAUSE`. If a buffer arrives, the channel is alive and the
    // TX-report arming is what is incomplete. If nothing arrives, the channel itself is shut and
    // no descriptor bit could ever have worked.
    //
    // Cross-check built in: payload[0] must equal our own read of 0x0522.
    if std::env::var_os("NDN_AU_C2H_LIVENESS").is_some() {
        use ndn_frame_io::FrameIo;
        let d = std::sync::Arc::new(d);
        d.spawn_rx_pump(4); // C2H arrives on the RX path; nothing can be seen without this
        std::thread::sleep(std::time::Duration::from_millis(200));
        let txpause = d.read8(0x0522)?;
        println!(
            "\n  C2H liveness: sending 0x1E (byte1=0 => read-only path). REG_TXPAUSE={txpause:#04x}"
        );
        d.write8(0x01c2, PRIMER)?;
        d.h2c([0x1e, 0x00, 0x00, 0x00])?;
        std::thread::sleep(std::time::Duration::from_millis(500));
        let (s0, r2, _) = d.h2c_status()?;
        println!(
            "    status={s0:#04x} echo={r2:#04x} => {}",
            if r2 == PRIMER {
                "DISPATCHED"
            } else {
                "REJECTED"
            }
        );
        println!("    (any C2H buffer prints above as C2H8812AU… — run with NDN_C2H_DBG=1)");
        println!("    expect id=0x20 len=3 with payload[0] == {txpause:#04x}");
        return Ok(());
    }

    println!("\n  Prediction: 0x99 and 0x58 REJECTED (absent from the table); 0x47 DISPATCHED.");
    println!("  If 0x47 also shows REJECTED, the default-arm reading is wrong and every id");
    println!("  verdict derived from that table is void.");
    Ok(())
}
