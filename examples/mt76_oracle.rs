//! **mt76 register oracle** — EP0-only probe for the MediaTek USB parts.
//!
//! Answers, on live silicon and without loading firmware or writing anything by
//! default, the questions the mt76 parity port is blocked on:
//!
//!  * mt76x0/mt76x2 (`--part 7610|7612`): does `MT_TSF_TIMER_DW0/DW1`
//!    (**0x111c/0x1120** — NOT the 0x1104/0x1108 a previous probe read, which are
//!    `MT_BKOFF_SLOT_CFG` and an unnamed config word) actually tick? Are
//!    `MT_CH_IDLE`/`MT_CH_BUSY` (0x1130/0x1134) read-and-clear microsecond
//!    counters? What does a *working kernel monitor* leave in `MT_RX_FILTR_CFG`?
//!  * mt7921 / connac2 (`--part 7961`): is the part alive on EP0 at all
//!    (`MT_HW_CHIPID` 0x70010200 via the **extended** vendor request 0x63), and
//!    what is its firmware/power state (`MT_CONN_ON_MISC` 0x7c0600f0)?
//!
//! Reads only, unless `--write` is passed (which enables the TSF timer and
//! restores the original `MT_BEACON_TIME_CFG` on exit).
//!
//!   sudo ./mt76_oracle --part 7610 [--write]
//!
//! ## MEASURED RESULTS — mds-o5p-1's MT7610U, 2026-08-27 (kernel `mt76x0u` bound, monitor ch149)
//!
//! These are the answers this probe exists to produce. Recorded here so the next reader does not
//! have to re-run it, and so a later run that disagrees is visibly a change, not a discovery.
//!
//! * `MT_ASIC_VERSION 0x0000 = 0x76100002` — MT7610, rev 2.
//! * ★ **The TSF is real.** `MT_TSF_TIMER_DW0 0x111c` reads a static 0 as found, because
//!   `MT_BEACON_TIME_CFG 0x1114 = 0x00000640` has `TIMER_EN` (bit16) CLEAR — the kernel deliberately
//!   clears it in every non-beaconing vif. Set bit16 and clear `SYNC_MODE [18:17]` and DW0 advances
//!   **+11191 / +11250 / +11229 / +11022 / +12046** against host steps of ~11.2 ms: **1.000 µs/tick**.
//!   **DW0 is the LOW word** (DW1 stayed 0). Upstream's only reader, `mt76x02_usb_core.c:155`, has
//!   the two words the other way round; it feeds a `dev_dbg` and is validated by nothing.
//!   ⇒ This RETRACTS the 2026-08-18 "mt76 has no TSF" finding, which probed `0x1104` / `0x1108` /
//!   `0x110c` (`MT_BKOFF_SLOT_CFG`, an unnamed word, `MT_CH_TIME_CFG`) and wrote `0x1100`
//!   (`MT_XIFS_TIME_CFG`) — all configuration registers. It never touched a counter. The static
//!   `0x114` it reported is a slot-time config value (slot 20 µs, cc_delay 1).
//! * ★ **`MT_CH_IDLE 0x1130` / `MT_CH_BUSY 0x1134` are read-and-clear MICROSECOND counters**:
//!   `(idle + busy) / elapsed` = 1.00, 1.00, 1.01, 1.00 over four consecutive 100 ms windows (the
//!   first window reads 1.96 because it carries the accumulation since the previous reader).
//!   `MT_ED_CCA_TIMER 0x1140` ticks independently (762-3902 µs where CH_BUSY ran 2718-14112) — a
//!   second, energy-detect-only occupancy sense.
//! * `MT_RX_STAT_0 0x1700` / `_1 0x1704` are read-and-clear per window (crc 99→4→4→1 while the link
//!   was quiet).
//! * ★ **The direct RF CSR path WORKS over USB.** `MT_RF_CSR_CFG 0x0500`, `(bank<<15)|(reg<<8)|KICK`:
//!   `RF(0,1)=0x01`, `RF(0,2)=0x11`, `RF(0,4)=0x30`, `RF(7,73)=0x34` — **identical to
//!   `mt76x0_rf_central_tab`** — and `RF(7,6)=0x40`, the `rf_bw_switch_tab` entry for
//!   `RF_A_BAND|RF_BW_20`, matching the interface's actual ch149/20 MHz. Upstream branches on *bus*
//!   (USB → the MCU register-pair path) rather than on capability, so this had never been settled.
//!   ⇒ live retune can be direct register writes; the MCU round trip is optional.
//! * ★ **The USB EEPROM shadow is populated** — 512/512 bytes, 0 errors, word 0 = `0x7610`, bytes
//!   4..9 = `9c:ef:d5:f8:f1:b6` = the netdev MAC exactly. No `MT_EFUSE_CTRL` port is needed.
//! * `MT_USB_DMA_CFG` is plain MMIO `0x0238` here (mt76x2's CFG-space `0x9018` reads 0) and live =
//!   `0x00c00000`: `TX_BULK_EN`|`RX_BULK_EN` set, **`RX_BULK_AGG_EN` (bit21) CLEAR** ⇒ one RX unit
//!   per URB, measured rather than inferred.
//! * Kernel-monitor golden reference: `MT_RX_FILTR_CFG 0x1400 = 0x00001093`,
//!   `MT_MAC_SYS_CTRL 0x1004 = 0x0c`, `MT_EXT_CCA_CFG 0x141c = 0x0000f1e4`,
//!   `MT_TXOP_CTRL_CFG 0x1340 = 0x0000583f` (ED-CCA bit20 CLEAR), `MT_BBP(AGC,2) 0x2308 = 0x003a6464`.
//! * `MT_WLAN_FUN_CTRL 0x0080 = 0xff000413`, `MT_CMB_CTRL 0x0020 = 0x00e007ff` (XTAL_RDY + PLL_LD).
//! * **EP0 round trip = 151.1 µs** (200/200 ok). That is the floor on anything built from register
//!   reads — it rules out a per-frame register stamp, exactly as the 141 µs measured on the MT7612U.
//!
//! On the MT7921AU (`--part 7961`) at minidronesys-05: the descriptors read (WLAN is **interface 3**,
//! class ff/ff/ff, bulk IN 0x84/0x85 + bulk OUT 0x04..0x09; interfaces 0-2 are Bluetooth, class
//! e0/01/01) but `libusb_open` fails and a raw `open("/dev/bus/usb/002/007")` returns **ENODEV** —
//! the device is enumerated in sysfs but gone at the usbfs layer. That is a host/port fault, not a
//! driver question; it needs a replug or a bus re-enumeration before any port work can proceed.
//!
//! Every address here is transcribed from `mt76x02_regs.h` / `mt792x_regs.h` of
//! the upstream mt76 tree; the numeric value is in the constant name's comment so
//! a reader never has to trust a symbol.
use rusb::{Context, DeviceHandle, Direction, UsbContext};
use std::time::{Duration, Instant};

const VID: u16 = 0x0e8d;

// mt76 vendor requests (mt76.h `enum mt76_vendor_req`).
const REQ_IN: u8 = 0xc0;
const REQ_OUT: u8 = 0x40;
const MT_VEND_MULTI_WRITE: u8 = 0x06;
const MT_VEND_MULTI_READ: u8 = 0x07;
const MT_VEND_READ_EXT: u8 = 0x63;
const MT_VEND_READ_EEPROM: u8 = 0x09;
const MT_VEND_READ_CFG: u8 = 0x47;
const T: Duration = Duration::from_millis(500);

/// mt76x02 registers (`mt76x02_regs.h`), address in the name.
const R_MT76X02: &[(u32, &str)] = &[
    (0x0704, "MT_MCU_CPU_CTL"),
    (
        0x0708,
        "MT_MCU_CLOCK_CTL <- bit0 = ROM patch applied (the persistent latch)",
    ),
    (0x0730, "MT_MCU_COM_REG0  <- a MAILBOX, not a status flag"),
    (0x0734, "MT_MCU_COM_REG1"),
    (0x1004, "MT_MAC_SYS_CTRL"),
    (0x1100, "MT_XIFS_TIME_CFG"),
    (
        0x1104,
        "MT_BKOFF_SLOT_CFG  <- what the 2026-08-18 'TSF' probe actually read",
    ),
    (0x110c, "MT_CH_TIME_CFG"),
    (
        0x1114,
        "MT_BEACON_TIME_CFG (TIMER_EN=BIT16, SYNC_MODE=[18:17])",
    ),
    (0x111c, "MT_TSF_TIMER_DW0   <- the real TSF"),
    (0x1120, "MT_TSF_TIMER_DW1"),
    (0x1130, "MT_CH_IDLE"),
    (0x1134, "MT_CH_BUSY"),
    (0x1138, "MT_EXT_CH_BUSY"),
    (0x1140, "MT_ED_CCA_TIMER"),
    (0x1200, "MT_MAC_STATUS"),
    (0x1300, "MT_EDCA_CFG_AC(0)"),
    (0x1340, "MT_TXOP_CTRL_CFG   (ED_CCA_EN=BIT20)"),
    (0x1344, "MT_TX_RTS_CFG"),
    (0x1400, "MT_RX_FILTR_CFG    <- monitor golden reference"),
    (0x141c, "MT_EXT_CCA_CFG"),
    (0x1700, "MT_RX_STAT_0       CRC[15:0] PHY[31:16]"),
    (0x1704, "MT_RX_STAT_1       CCA[15:0] PLCP[31:16]"),
    (0x1708, "MT_RX_STAT_2       DUP[15:0] OVF[31:16]"),
    (0x2308, "MT_BBP(AGC,2)"),
];

/// connac2 / mt792x registers (`mt792x_regs.h`).
const R_CONNAC2: &[(u32, &str)] = &[
    (0x7001_0200, "MT_HW_CHIPID"),
    (0x7001_0204, "MT_HW_REV"),
    (
        0x7c06_00f0,
        "MT_CONN_ON_MISC (FW_PWR_ON=BIT0, FW_N9_ON=BIT1)",
    ),
    (0x1802_1204, "MT_UDMA_TX_QSEL-ish (WFDMA window)"),
    // ⚠ NOT MT_TOP_MISC. Upstream's MT_TOP_MISC is MT_TOP(0xf0) = 0x1806_00f0
    // (mt792x_regs.h:404,411). 0x7000_00f0 is an unnamed CB-TOP word that nothing in the mt76
    // tree references, so the 0x00000000 this probe read there says nothing about firmware
    // state — MT_CONN_ON_MISC is the register that does. Kept, correctly labelled, because a
    // wrong label on a real reading is worse than no reading.
    (0x1806_00f0, "MT_TOP_MISC (FW_STATE[2:0])"),
    (
        0x7000_00f0,
        "CB-TOP 0x700000f0 — unnamed upstream; NOT MT_TOP_MISC",
    ),
];

struct Dev {
    h: DeviceHandle<Context>,
    ext: bool,
}

impl Dev {
    fn rr(&self, addr: u32) -> Result<u32, rusb::Error> {
        let mut b = [0u8; 4];
        let req = if self.ext {
            MT_VEND_READ_EXT
        } else {
            MT_VEND_MULTI_READ
        };
        self.h.read_control(
            REQ_IN,
            req,
            (addr >> 16) as u16,
            (addr & 0xffff) as u16,
            &mut b,
            T,
        )?;
        Ok(u32::from_le_bytes(b))
    }
    /// EEPROM shadow read (`MT_VEND_READ_EEPROM` 0x09) — whether this shadow is
    /// populated at all on the mt76x0 (vs needing the `MT_EFUSE_CTRL` path) is
    /// exactly what this probe settles.
    fn ee(&self, off: u16) -> Result<u32, rusb::Error> {
        let mut b = [0u8; 4];
        self.h
            .read_control(REQ_IN, MT_VEND_READ_EEPROM, 0, off, &mut b, T)?;
        Ok(u32::from_le_bytes(b))
    }
    /// CFG-space read (`MT_VEND_READ_CFG` 0x47).
    fn cfg(&self, off: u16) -> Result<u32, rusb::Error> {
        let mut b = [0u8; 4];
        self.h
            .read_control(REQ_IN, MT_VEND_READ_CFG, 0, off, &mut b, T)?;
        Ok(u32::from_le_bytes(b))
    }
    fn wr(&self, addr: u32, val: u32) -> Result<(), rusb::Error> {
        self.h.write_control(
            REQ_OUT,
            MT_VEND_MULTI_WRITE,
            (addr >> 16) as u16,
            (addr & 0xffff) as u16,
            &val.to_le_bytes(),
            T,
        )?;
        Ok(())
    }
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let args: Vec<String> = std::env::args().collect();
    let part = args
        .windows(2)
        .find(|w| w[0] == "--part")
        .map(|w| u16::from_str_radix(&w[1], 16).unwrap())
        .unwrap_or(0x7610);
    let do_write = args.iter().any(|a| a == "--write");
    let ext = part == 0x7961;

    let ctx = Context::new()?;
    let dev = ctx
        .devices()?
        .iter()
        .find(|d| {
            d.device_descriptor()
                .map(|s| s.vendor_id() == VID && s.product_id() == part)
                .unwrap_or(false)
        })
        .ok_or_else(|| format!("no {VID:04x}:{part:04x} on this host"))?;
    let desc = dev.device_descriptor()?;
    println!(
        "found {:04x}:{:04x}  bus {} addr {}  speed {:?}",
        desc.vendor_id(),
        desc.product_id(),
        dev.bus_number(),
        dev.address(),
        dev.speed()
    );
    // Endpoint inventory — a port needs these before anything else.
    if let Ok(cfg) = dev.active_config_descriptor() {
        for i in cfg.interfaces() {
            for d in i.descriptors() {
                let eps: Vec<String> = d
                    .endpoint_descriptors()
                    .map(|e| {
                        format!(
                            "{:#04x}{}{:?}/{}",
                            e.address(),
                            if e.direction() == Direction::In {
                                "IN "
                            } else {
                                "OUT"
                            },
                            e.transfer_type(),
                            e.max_packet_size()
                        )
                    })
                    .collect();
                if !eps.is_empty() {
                    println!(
                        "  if{} alt{} class {:#04x}/{:#04x}/{:#04x}: {}",
                        d.interface_number(),
                        d.setting_number(),
                        d.class_code(),
                        d.sub_class_code(),
                        d.protocol_code(),
                        eps.join(" ")
                    );
                }
            }
        }
    }

    // NO usb reset: a blind reset is what wedges these parts (and mds-05's
    // 2-1.3-port4 refuses one outright). Open, and do not claim — EP0 vendor
    // requests are device-scoped, so a kernel-bound part can still be read.
    let h = match dev.open() {
        Ok(h) => h,
        Err(e) => {
            eprintln!("open() failed: {e:?} — retrying via open_device_with_vid_pid");
            ctx.open_device_with_vid_pid(VID, part)
                .ok_or_else(|| format!("open_device_with_vid_pid also failed: {e:?}"))?
        }
    };
    // Detach any kernel driver only for the interface we intend to use; on a
    // fully unbound part this is a no-op. Never reset.
    let _ = h.set_auto_detach_kernel_driver(true);
    let claim_if: Option<u8> = args
        .windows(2)
        .find(|w| w[0] == "--claim")
        .and_then(|w| w[1].parse().ok());
    if let Some(n) = claim_if {
        match h.claim_interface(n) {
            Ok(()) => println!("claimed interface {n}"),
            Err(e) => println!("claim_interface({n}) failed: {e:?}"),
        }
    }
    let d = Dev { h, ext };

    let regs = if ext { R_CONNAC2 } else { R_MT76X02 };
    println!(
        "\n=== register dump ({}) ===",
        if ext {
            "MT_VEND_READ_EXT 0x63"
        } else {
            "MT_VEND_MULTI_READ 0x07"
        }
    );
    for (a, name) in regs {
        match d.rr(*a) {
            Ok(v) => println!("  {a:#010x} = {v:#010x}  {name}"),
            Err(e) => println!("  {a:#010x} =  ERR({e})  {name}"),
        }
    }

    // Control-transfer round-trip cost — the floor on any per-frame register stamp.
    let t0 = Instant::now();
    let probe = if ext { 0x7001_0200 } else { 0x1004 };
    let mut ok = 0u32;
    for _ in 0..200 {
        if d.rr(probe).is_ok() {
            ok += 1;
        }
    }
    println!(
        "\nEP0 read latency: {:.1} us/read ({ok}/200 ok)",
        t0.elapsed().as_micros() as f64 / 200.0
    );

    if ext {
        return Ok(());
    }

    // ── Identity + EEPROM shadow ────────────────────────────────────────────
    println!("\n=== identity ===");
    for (a, n) in [
        (0x0000u32, "MT_ASIC_VERSION"),
        (0x0008, "MT_MAC_CSR0"),
        (0x0080, "MT_WLAN_FUN_CTRL"),
        (0x0020, "MT_CMB_CTRL"),
        (0x0238, "MT_USB_DMA_CFG (mt76x0 MMIO)"),
        (0x0500, "MT_RF_CSR_CFG"),
    ] {
        match d.rr(a) {
            Ok(v) => println!("  {a:#010x} = {v:#010x}  {n}"),
            Err(e) => println!("  {a:#010x} =  ERR({e})  {n}"),
        }
    }
    println!(
        "  CFG 0x9018 = {:?}  MT_USB_U3DMA_CFG (mt76x2 CFG-space)",
        d.cfg(0x9018)
    );

    if args.iter().any(|a| a == "--eeprom") {
        println!("\n=== EEPROM shadow via MT_VEND_READ_EEPROM (0x09), 512 B ===");
        let mut raw = Vec::with_capacity(512);
        let mut errs = 0;
        for off in (0u16..512).step_by(4) {
            match d.ee(off) {
                Ok(v) => raw.extend_from_slice(&v.to_le_bytes()),
                Err(_) => {
                    errs += 1;
                    raw.extend_from_slice(&[0xff; 4]);
                }
            }
        }
        println!("  read errors: {errs}/128");
        for (i, chunk) in raw.chunks(16).enumerate() {
            let hex: Vec<String> = chunk.iter().map(|b| format!("{b:02x}")).collect();
            println!("  {:03x}: {}", i * 16, hex.join(" "));
        }
        // The two fields whose correctness is checkable without any parse table.
        println!(
            "  MAC (MT_EE_MAC_ADDR 0x04) = {:02x}:{:02x}:{:02x}:{:02x}:{:02x}:{:02x}",
            raw[4], raw[5], raw[6], raw[7], raw[8], raw[9]
        );
        println!("  chip id word @0x00 = {:02x}{:02x}", raw[1], raw[0]);
    }

    // ── B6a: does the direct RF CSR path work on a USB mt76x0? ─────────────
    // Upstream branches on BUS (`mt76_is_usb` -> the MCU register-pair path), not
    // on capability, so whether MT_RF_CSR_CFG (0x0500) actually drives the synth
    // over USB has never been established either way. It decides whether live
    // retuning needs the MCU round trip or can be a direct register write.
    // Read-only: KICK with the WR bit CLEAR, exactly `mt76x0_rf_csr_rr`.
    if !ext {
        println!("\n=== RF CSR read path (MT_RF_CSR_CFG 0x0500) ===");
        let rf_rr = |bank: u32, reg: u32| -> Option<u8> {
            for _ in 0..100 {
                match d.rr(0x0500) {
                    Ok(v) if v & (1 << 31) == 0 => break,
                    Ok(_) => continue,
                    Err(_) => return None,
                }
            }
            d.wr(
                0x0500,
                ((bank & 0x7) << 15) | ((reg & 0x7f) << 8) | (1 << 31),
            )
            .ok()?;
            for _ in 0..100 {
                match d.rr(0x0500) {
                    Ok(v) if v & (1 << 31) == 0 => {
                        let rb = (v >> 15) & 0x7;
                        let rr = (v >> 8) & 0x7f;
                        return if rb == bank && rr == reg {
                            Some((v & 0xff) as u8)
                        } else {
                            None
                        };
                    }
                    Ok(_) => continue,
                    Err(_) => return None,
                }
            }
            None
        };
        // Bank 0 R0..R7 are the VCO/PLL block the frequency plan programs; a live
        // radio must return non-trivial, non-uniform values here.
        for reg in 0..8u32 {
            match rf_rr(0, reg) {
                Some(v) => println!("  RF(0,{reg}) = {v:#04x}"),
                None => println!("  RF(0,{reg}) = <no readback / bank-reg mismatch>"),
            }
        }
        for (b, r) in [(4u32, 0u32), (5, 0), (6, 0), (7, 6), (7, 73)] {
            match rf_rr(b, r) {
                Some(v) => println!("  RF({b},{r}) = {v:#04x}"),
                None => println!("  RF({b},{r}) = <no readback>"),
            }
        }
    }

    // ── The TSF question ────────────────────────────────────────────────────
    println!("\n=== TSF advance test: 0x111c / 0x1120 at 10 ms steps ===");
    let tsf_sample = |label: &str| {
        let t = Instant::now();
        let mut last: Option<(u32, u32)> = None;
        for _ in 0..6 {
            std::thread::sleep(Duration::from_millis(10));
            let dw0 = d.rr(0x111c).unwrap_or(0);
            let dw1 = d.rr(0x1120).unwrap_or(0);
            let (d0, d1) = last
                .map(|(a, b)| (dw0.wrapping_sub(a), dw1.wrapping_sub(b)))
                .unwrap_or((0, 0));
            println!(
                "  [{label}] host={:>8}us  DW0=0x{dw0:08x} (+{d0:<8})  DW1=0x{dw1:08x} (+{d1})",
                t.elapsed().as_micros()
            );
            last = Some((dw0, dw1));
        }
    };
    tsf_sample("as-found");

    if do_write {
        let bt = d.rr(0x1114)?;
        println!(
            "\n  MT_BEACON_TIME_CFG before = {bt:#010x} (TIMER_EN={})",
            (bt >> 16) & 1
        );
        // TIMER_EN on, SYNC_MODE (bits 18:17) cleared so received beacons cannot
        // slam the counter — a synced TSF is not a free-running clock.
        d.wr(0x1114, (bt & !0x0006_0000) | 0x0001_0000)?;
        println!("  MT_BEACON_TIME_CFG after  = {:#010x}", d.rr(0x1114)?);
        tsf_sample("timer-en");
        d.wr(0x1114, bt)?;
        println!("  restored MT_BEACON_TIME_CFG = {:#010x}", d.rr(0x1114)?);
    }

    // ── Channel-busy semantics ──────────────────────────────────────────────
    println!("\n=== CH_IDLE/CH_BUSY/ED_CCA over 100 ms windows (read-and-clear?) ===");
    for _ in 0..5 {
        let t = Instant::now();
        std::thread::sleep(Duration::from_millis(100));
        let idle = d.rr(0x1130).unwrap_or(0);
        let busy = d.rr(0x1134).unwrap_or(0);
        let ed = d.rr(0x1140).unwrap_or(0);
        let el = t.elapsed().as_micros() as u64;
        println!(
            "  elapsed={el:>7}us  idle={idle:<10} busy={busy:<10} sum={:<10} (sum/elapsed={:.2})  ed_cca_timer={ed}",
            idle as u64 + busy as u64,
            (idle as u64 + busy as u64) as f64 / el as f64
        );
    }

    println!("\n=== RX_STAT deltas over 100 ms (read-and-clear?) ===");
    for _ in 0..4 {
        std::thread::sleep(Duration::from_millis(100));
        let s0 = d.rr(0x1700).unwrap_or(0);
        let s1 = d.rr(0x1704).unwrap_or(0);
        println!(
            "  crc={:<6} phy={:<6} cca={:<6} plcp={:<6}",
            s0 & 0xffff,
            s0 >> 16,
            s1 & 0xffff,
            s1 >> 16
        );
    }
    Ok(())
}
