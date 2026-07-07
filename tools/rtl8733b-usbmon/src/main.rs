//! usbmon text-capture parser for the RTL8733BU TX-enable hunt.
//!
//! Reads a `cat /sys/kernel/debug/usb/usbmon/<bus>u` capture and reconstructs the
//! Realtek VENQT register writes (bmRequestType 0x40, bRequest 0x05, wValue=addr) and
//! bulk transfers. The point: find the BB/RF register write(s) the vendor driver makes
//! to key the antenna on TX that our userspace driver is missing — the bulk-OUT is the
//! frame going out; the writes just before it are the TX-enable.
//!
//! Modes (arg 1):
//!   list   <cap>            every reg write + bulk transfer, in order, annotated
//!   pretx  <cap>            reg writes before the FIRST bulk-OUT (the bring-up + enable)
//!   around <cap> [N]        the N reg writes before each bulk-OUT (per-TX enable; N=20)
//!   diff   <vendor> <mine>  reg addrs/values written by vendor but not (or differently) by mine
//!   bulk   <cap>            hex-dump each bulk-OUT (the TX descriptor + frame)

use std::collections::BTreeMap;

#[derive(Clone)]
enum Ev {
    Write { addr: u16, val: u32, len: u8 },
    Read { addr: u16, val: u32, len: u8 },
    BulkOut { ep: u8, data: Vec<u8> },
    BulkIn { ep: u8, len: usize },
}

struct Rec {
    #[allow(dead_code)]
    ts: u64,
    ev: Ev,
}

fn main() {
    let args: Vec<String> = std::env::args().collect();
    let mode = args.get(1).map(|s| s.as_str()).unwrap_or("");
    match mode {
        "list" | "pretx" | "around" | "bulk" => {
            let Some(path) = args.get(2) else { return usage() };
            let recs = parse(path);
            match mode {
                "list" => list(&recs),
                "pretx" => pretx(&recs),
                "around" => around(&recs, args.get(3).and_then(|s| s.parse().ok()).unwrap_or(20)),
                "bulk" => bulk(&recs),
                _ => unreachable!(),
            }
        }
        "diff" => {
            let (Some(a), Some(b)) = (args.get(2), args.get(3)) else { return usage() };
            diff(&parse(a), &parse(b));
        }
        _ => usage(),
    }
}

fn usage() {
    eprintln!(
        "usage: usbmon <list|pretx|around|bulk|diff> <capture> [args]\n\
         \x20 list   <cap>            every reg write + bulk, annotated\n\
         \x20 pretx  <cap>            reg writes before the first bulk-OUT\n\
         \x20 around <cap> [N=20]     N reg writes before each bulk-OUT (per-TX enable)\n\
         \x20 bulk   <cap>            hex-dump each bulk-OUT (TX descriptor + frame)\n\
         \x20 diff   <vendor> <mine>  regs vendor writes that mine doesn't / differs"
    );
}

/// Parse a usbmon text capture. Tolerant: unparseable lines are skipped. Reg reads
/// need the submission (addr) matched to the completion (data) by URB id.
fn parse(path: &str) -> Vec<Rec> {
    let text = match std::fs::read_to_string(path) {
        Ok(t) => t,
        Err(e) => {
            eprintln!("cannot read {path}: {e}");
            std::process::exit(1);
        }
    };
    // Pending control-IN reads: urb-id -> (addr, len) awaiting the completion's data.
    let mut pending: BTreeMap<String, (u16, u8)> = BTreeMap::new();
    let mut out = Vec::new();

    for line in text.lines() {
        let f: Vec<&str> = line.split_whitespace().collect();
        if f.len() < 4 {
            continue;
        }
        let urb = f[0];
        let ts: u64 = f[1].parse().unwrap_or(0);
        let sc = f[2]; // "S" submit / "C" complete
        let addr = f[3]; // e.g. Co:2:003:0 or Bo:2:003:2
        let mut it = addr.split(':');
        let typ = it.next().unwrap_or(""); // Co/Ci/Bo/Bi/Ii...
        let _bus = it.next();
        let _dev = it.next();
        let ep: u8 = it.next().and_then(|s| s.parse().ok()).unwrap_or(0);

        // Control transfers carry a setup packet after an 's' token on submission.
        if (typ == "Co" || typ == "Ci") && sc == "S" {
            if let Some(si) = f.iter().position(|&x| x == "s") {
                // s bmReqType bReq wValue wIndex wLength
                if f.len() >= si + 6 {
                    // setup fields: si+1 bmReqType, +2 bReq, +3 wValue, +4 wIndex, +5 wLength
                    let breq = u8::from_str_radix(f[si + 2], 16).unwrap_or(0xff);
                    let wval = u16::from_str_radix(f[si + 3], 16).unwrap_or(0);
                    let wlen = u16::from_str_radix(f[si + 5], 16).unwrap_or(0) as u8;
                    if breq == 0x05 {
                        if typ == "Co" {
                            // Write: data is on this submission line after '='.
                            let val = data_after_eq(&f).map(le_val).unwrap_or(0);
                            out.push(Rec { ts, ev: Ev::Write { addr: wval, val, len: wlen } });
                        } else {
                            // Read: remember addr; value arrives on the completion.
                            pending.insert(urb.to_string(), (wval, wlen));
                        }
                    }
                }
            }
        } else if typ == "Ci" && sc == "C" {
            if let Some((addr, len)) = pending.remove(urb) {
                let val = data_after_eq(&f).map(le_val).unwrap_or(0);
                out.push(Rec { ts, ev: Ev::Read { addr, val, len } });
            }
        } else if typ == "Bo" && sc == "S" {
            if let Some(bytes) = data_bytes_after_eq(&f) {
                out.push(Rec { ts, ev: Ev::BulkOut { ep, data: bytes } });
            }
        } else if typ == "Bi" && sc == "C" {
            let len = data_bytes_after_eq(&f).map(|b| b.len()).unwrap_or(0);
            out.push(Rec { ts, ev: Ev::BulkIn { ep, len } });
        }
    }
    out
}

/// The hex data tokens after the '=' marker, concatenated into bytes (transfer order).
fn data_bytes_after_eq(f: &[&str]) -> Option<Vec<u8>> {
    let i = f.iter().position(|&x| x == "=")?;
    let mut hex = String::new();
    for tok in &f[i + 1..] {
        if tok.chars().all(|c| c.is_ascii_hexdigit()) {
            hex.push_str(tok);
        }
    }
    if hex.is_empty() {
        return None;
    }
    let mut bytes = Vec::new();
    let h: Vec<char> = hex.chars().collect();
    let mut j = 0;
    while j + 1 < h.len() + 1 && j + 2 <= h.len() {
        let b = u8::from_str_radix(&format!("{}{}", h[j], h[j + 1]), 16).ok()?;
        bytes.push(b);
        j += 2;
    }
    Some(bytes)
}

fn data_after_eq(f: &[&str]) -> Option<Vec<u8>> {
    data_bytes_after_eq(f)
}

/// Assemble register bytes (little-endian, Realtek) into a value.
fn le_val(bytes: Vec<u8>) -> u32 {
    let mut v = 0u32;
    for (i, b) in bytes.iter().take(4).enumerate() {
        v |= (*b as u32) << (8 * i);
    }
    v
}

/// Name the subsystem/window an address falls in — so a TX-enable stands out.
fn annotate(addr: u16) -> &'static str {
    match addr {
        0x0000..=0x00ff => "SYS",
        0x0100..=0x01ff => "MAC-CR/queue",
        0x0200..=0x02ff => "MAC-RQPN/DMA",
        0x0300..=0x04ff => "MAC-EDCA/FIFO",
        0x0500..=0x05ff => "MAC (TXPAUSE@522)",
        0x0600..=0x06ff => "MAC-RCR/RXFLT",
        0x0700..=0x07ff => "MAC",
        0x0800..=0x0fff => "BB-PHY",
        0x1000..=0x10ff => "SYS-EXTFUNC/DDMA",
        0x1200..=0x17ff => "MAC/misc",
        0x1800..=0x1bff => "BB-KIP/IQK",
        0x1c00..=0x1fff => "BB (1e70 TX-blk)",
        0x2800..=0x2fff => "BB (2a08/2de0)",
        0x3a00..=0x3aff => "TXAGC-table",
        0x3c00..=0x3fff => "RF-A window",
        0x4000..=0x4bff => "BB (4308 txagc-ref)",
        0x4c00..=0x4fff => "RF-B window",
        _ => "?",
    }
}

/// RF register decode for the 0x3c00/0x4c00 windows: reg = (addr - base) >> 2.
fn rf_note(addr: u16) -> Option<String> {
    let (base, path) = if (0x3c00..=0x3fff).contains(&addr) {
        (0x3c00u16, 'A')
    } else if (0x4c00..=0x4fff).contains(&addr) {
        (0x4c00, 'B')
    } else {
        return None;
    };
    Some(format!("RF{path}[0x{:02x}]", (addr - base) >> 2))
}

fn show_write(addr: u16, val: u32, len: u8) -> String {
    let rf = rf_note(addr).map(|s| format!(" {s}")).unwrap_or_default();
    format!("W 0x{addr:04x} = 0x{val:0width$x}  [{}]{rf}", annotate(addr), width = (len as usize) * 2)
}

fn list(recs: &[Rec]) {
    let mut tx = 0;
    for r in recs {
        match &r.ev {
            Ev::Write { addr, val, len } => println!("{}", show_write(*addr, *val, *len)),
            Ev::Read { addr, val, len } => {
                println!("R 0x{addr:04x} = 0x{val:0w$x}  [{}]", annotate(*addr), w = (*len as usize) * 2)
            }
            Ev::BulkOut { ep, data } => {
                tx += 1;
                println!(">>> BULK-OUT #{tx} ep 0x{:02x}  {} bytes (TX frame) <<<", ep, data.len());
            }
            Ev::BulkIn { ep, len } => println!("<<< bulk-IN ep 0x{ep:02x} {len} bytes (RX)"),
        }
    }
}

fn pretx(recs: &[Rec]) {
    println!("# register writes before the FIRST bulk-OUT (bring-up + TX-enable):");
    for r in recs {
        match &r.ev {
            Ev::Write { addr, val, len } => println!("{}", show_write(*addr, *val, *len)),
            Ev::BulkOut { ep, data } => {
                println!(">>> first BULK-OUT ep 0x{:02x} {} bytes — stop <<<", ep, data.len());
                return;
            }
            _ => {}
        }
    }
}

fn around(recs: &[Rec], n: usize) {
    // For each bulk-OUT, print the preceding `n` register writes — the per-TX enable.
    let mut writes: Vec<(u16, u32, u8)> = Vec::new();
    let mut tx = 0;
    for r in recs {
        match &r.ev {
            Ev::Write { addr, val, len } => writes.push((*addr, *val, *len)),
            Ev::BulkOut { ep, data } => {
                tx += 1;
                println!("\n=== {n} writes before BULK-OUT #{tx} (ep 0x{:02x}, {} B) ===", ep, data.len());
                let start = writes.len().saturating_sub(n);
                for (a, v, l) in &writes[start..] {
                    println!("  {}", show_write(*a, *v, *l));
                }
                writes.clear();
            }
            _ => {}
        }
    }
}

fn bulk(recs: &[Rec]) {
    let mut tx = 0;
    for r in recs {
        if let Ev::BulkOut { ep, data } = &r.ev {
            tx += 1;
            println!("\n--- BULK-OUT #{tx} ep 0x{:02x} ({} bytes) ---", ep, data.len());
            for (i, chunk) in data.chunks(16).enumerate() {
                let hex: Vec<String> = chunk.iter().map(|b| format!("{b:02x}")).collect();
                let asc: String = chunk
                    .iter()
                    .map(|&b| if (0x20..0x7f).contains(&b) { b as char } else { '.' })
                    .collect();
                println!("  {:04x}  {:<48}  {asc}", i * 16, hex.join(" "));
            }
        }
    }
}

/// Registers the vendor writes (last value seen) that mine never writes, or writes a
/// different value. Focused on where a TX-enable would live.
fn diff(vendor: &[Rec], mine: &[Rec]) {
    let last_writes = |recs: &[Rec]| -> BTreeMap<u16, u32> {
        let mut m = BTreeMap::new();
        for r in recs {
            if let Ev::Write { addr, val, .. } = &r.ev {
                m.insert(*addr, *val);
            }
        }
        m
    };
    let v = last_writes(vendor);
    let m = last_writes(mine);
    println!("# regs VENDOR writes that MINE does not (candidate TX-enables):");
    for (addr, val) in &v {
        if !m.contains_key(addr) {
            let rf = rf_note(*addr).map(|s| format!(" {s}")).unwrap_or_default();
            println!("  ONLY-VENDOR 0x{addr:04x} = 0x{val:08x}  [{}]{rf}", annotate(*addr));
        }
    }
    println!("\n# regs both write but with DIFFERENT final values:");
    for (addr, val) in &v {
        if let Some(mv) = m.get(addr) {
            if mv != val {
                let rf = rf_note(*addr).map(|s| format!(" {s}")).unwrap_or_default();
                println!("  DIFF 0x{addr:04x}: vendor 0x{val:08x} vs mine 0x{mv:08x}  [{}]{rf}", annotate(*addr));
            }
        }
    }
}
