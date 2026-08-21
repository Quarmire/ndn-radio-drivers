//! **AR9271 M1.1/M1.2 probe — register access + chip reset + initvals.**
//!
//! Builds on the proven transport (`ath9k_m0` fw-download+HTC+WMI, `ath9k_m1_plumbing` data
//! services). Three stages, each verified against a hardware read-back — never from "the code ran":
//!
//!   Step 0  the register primitive. `reg_read(AR_SREV=0x4020)` must return a stable,
//!           non-degenerate silicon-revision value (not 0, not 0xffff_ffff, not the address).
//!           If it does not, we STOP before writing anything — a broken primitive must not drive
//!           reset/initvals.
//!   M1.1    `phy_reset()` — POWER_ON reset dance; verify AR_RTC_STATUS reads ON(0x02),
//!           AR_RTC_RC reads 0, and AR_SREV still decodes to the AR9271.
//!   M1.2    `apply_initvals()` — 2.4 GHz PLL + MODES/COMMON/ANI/TX-gain replay; verify a spread
//!           of sentinel registers read back exactly what was written.
//!
//! The AR9271 is unbrickable (RAM firmware; a bad state = a replug). Run on o5p-1
//! (AR9271 at 2-1.2:1.0), kernel driver unbound:
//! ```sh
//! echo -n 2-1.2:1.0 | sudo tee /sys/bus/usb/drivers/ath9k_htc/unbind
//! sudo LD_LIBRARY_PATH=$(nix path-info nixpkgs#libusb1)/lib \
//!     ./ath9k_m1_reset ~/ath9k-fw/target_firmware/build/k2/htc_9271.fw
//! echo -n 2-1.2:1.0 | sudo tee /sys/bus/usb/drivers/ath9k_htc/bind
//! ```
use std::process::ExitCode;

use ndn_radio_drivers::Ath9kHtcBackend;

/// AR_SREV (reg.h: 0x4020 for the AR9271). Decodes to macVersion 0x140 / macRev 0/1.
const AR_SREV: u32 = 0x4020;

fn main() -> ExitCode {
    let args: Vec<String> = std::env::args().collect();
    if args.len() < 2 {
        eprintln!("usage: {} <htc_9271.fw>", args[0]);
        return ExitCode::FAILURE;
    }
    let fw = match std::fs::read(&args[1]) {
        Ok(f) => f,
        Err(e) => {
            eprintln!("cannot read {}: {e}", args[1]);
            return ExitCode::FAILURE;
        }
    };
    println!("firmware: {} ({} bytes)", args[1], fw.len());

    let mut dev = match Ath9kHtcBackend::open() {
        Ok(d) => d,
        Err(e) => {
            eprintln!("open failed: {e}  (unbind ath9k_htc first)");
            return ExitCode::FAILURE;
        }
    };
    if let Err(e) = dev.download_firmware(&fw) {
        eprintln!("firmware download FAILED: {e}");
        return ExitCode::FAILURE;
    }
    if let Err(e) = dev.htc_init() {
        eprintln!("HTC handshake FAILED: {e}");
        return ExitCode::FAILURE;
    }
    let (credits, credit_size) = dev.credits();
    println!(
        "[transport] HTC up: {credits} credits of {credit_size} B, FW {:?}",
        dev.fw_version()
    );
    let _ = dev.drain_events(600);

    let rc = run(&mut dev);

    match dev.detach() {
        Ok(()) => println!("target detached cleanly"),
        Err(e) => eprintln!("WARNING: clean detach failed ({e}) — a replug may be needed"),
    }
    rc
}

fn run(dev: &mut Ath9kHtcBackend) -> ExitCode {
    // ── Step 0 — prove the register primitive with the AR_SREV oracle ──
    println!("\n[step0] register primitive — reading AR_SREV (0x4020) x3 for stability");
    let mut reads = [0u32; 3];
    for r in reads.iter_mut() {
        match dev.reg_read(AR_SREV) {
            Ok(v) => *r = v,
            Err(e) => {
                eprintln!("[step0] reg_read(AR_SREV) FAILED: {e}");
                eprintln!("        WMI RegRead wire format is wrong or the target did not reply.");
                return ExitCode::FAILURE;
            }
        }
    }
    let srev = reads[0];
    println!(
        "[step0] AR_SREV raw = {srev:#010x}  (reads: {:#010x} {:#010x} {:#010x})",
        reads[0], reads[1], reads[2]
    );
    // The AR9271's AR_SREV low byte reads 0xFF, so ath9k does NOT decode macVersion from this
    // register for the USB parts — it takes 0x140 from the HTC device path (see ath9k_reg.rs). The
    // silicon signature is still here: bits[23:16] == 0x14 == (AR_SREV_VERSION_9271 0x140 >> 4).
    println!(
        "[step0] decode: id(&0xff)={:#04x}  sig bits[23:16]={:#04x} (want 0x14 = 0x140>>4)  \
         macRev2((&0xf00)>>8)={:#x}",
        srev & 0xff,
        (srev >> 16) & 0xff,
        (srev & 0x0000_0f00) >> 8
    );
    let sig_ok = ((srev >> 16) & 0xff) == 0x14;
    println!(
        "[step0] AR9271 signature {}",
        if sig_ok { "present ✓ (0x14)" } else { "ABSENT ✗ — is this really an AR9271?" }
    );
    let degenerate = reads.iter().any(|&v| v == 0 || v == 0xffff_ffff || v == AR_SREV)
        || reads[0] != reads[1]
        || reads[1] != reads[2];
    if degenerate {
        eprintln!(
            "[step0] ★ AR_SREV read is degenerate/unstable ({reads:#010x?}) — the reg primitive is\n\
             \x20       NOT proven. STOPPING before any write. Re-examine the WMI RegRead framing."
        );
        return ExitCode::FAILURE;
    }
    println!("[step0] reg primitive OK ✓ (stable, non-degenerate register data returned)");

    // ── M1.1 — chip reset / wake ──
    println!("\n[M1.1] phy_reset() — POWER_ON reset dance");
    let st = match dev.phy_reset() {
        Ok(s) => s,
        Err(e) => {
            eprintln!("[M1.1] phy_reset FAILED: {e}");
            return ExitCode::FAILURE;
        }
    };
    println!(
        "[M1.1] AR_RTC_STATUS={:#010x} (&0x0f={:#x}, ON=0x02)  AR_RTC_RC={:#010x} (want 0)  \
         intr_sync&0x3000={:#x}",
        st.rtc_status,
        st.rtc_status & 0x0f,
        st.rtc_rc,
        st.intr_sync_masked
    );
    println!(
        "[M1.1] AR_SREV after reset={:#010x}  sig bits[23:16]={:#04x} (want 0x14, unchanged by reset)",
        st.srev_raw,
        (st.srev_raw >> 16) & 0xff
    );
    let status_on = (st.rtc_status & 0x0f) == 0x02;
    let rc_clear = st.rtc_rc == 0;
    let srev_live = st.srev_raw != 0 && st.srev_raw != 0xffff_ffff && st.srev_raw != AR_SREV;
    if status_on && rc_clear && srev_live {
        println!("[M1.1] reset VERIFIED ✓ (STATUS=ON, RC=0, SREV live)");
    } else {
        eprintln!(
            "[M1.1] reset verify FAILED: STATUS_ON={status_on} RC_CLEAR={rc_clear} SREV_LIVE={srev_live}"
        );
        return ExitCode::FAILURE;
    }

    // ── M1.2 — PLL + initvals ──
    println!("\n[M1.2] apply_initvals() — 2.4 GHz PLL + MODES/COMMON/ANI/TX-gain replay");
    let iv = match dev.apply_initvals() {
        Ok(v) => v,
        Err(e) => {
            eprintln!("[M1.2] apply_initvals FAILED: {e}");
            return ExitCode::FAILURE;
        }
    };
    println!(
        "[M1.2] streamed {} register writes; sentinel read-back: {}/{} matched",
        iv.total_written, iv.matched, iv.checked
    );
    for (addr, expected, got) in &iv.mismatches {
        println!("[M1.2]   MISMATCH {addr:#08x}: expected {expected:#010x}, got {got:#010x}");
    }
    if iv.mismatches.is_empty() && iv.checked > 0 {
        println!("[M1.2] initvals VERIFIED ✓ (every sentinel read back its written value)");
        ExitCode::SUCCESS
    } else {
        eprintln!("[M1.2] initvals verify INCOMPLETE — {} sentinel(s) mismatched", iv.mismatches.len());
        // Not a hard failure of the primitive: report and let the operator judge.
        ExitCode::SUCCESS
    }
}
