//! RTL8731BU/8733BU hardware probe — the M4.5 reverse-engineering bench.
//!
//! Self-contained (rusb only) so it builds on the OPi via nix-shell against the
//! real f72b. Carries the exact VENQT reg-I/O + M1–M4 sequence from the driver.
//!
//! M4.5 findings (mapped on silicon; the reserved-page write must raise BCN_VALID):
//!  - BASELINE (cpu_en(0) + fw_dl_setup, MGNT desc, HIQ ep): bulk write SUCCEEDS,
//!    but BCN_VALID never asserts — the packet lands in a TX queue, not the beacon
//!    page. That's the nut to crack.
//!  - Vendor order (download_firmware_87xx): EXT_CLK/IRAM → cpu_en(0) → PQ_MAP →
//!    CR → RQPN(D0/00/20/80) → BCN_CTRL → pltfm_reset → start_dlfw(MCUFW BIT0 →
//!    send_fwpkt → dl_rsvd_page). QSLT_BEACON=0x10 QSLT_HIGH=0x11 QSLT_MGNT=0x12.
//!  - pltfm_reset = toggle BIT0 of 0x1002, THEN re-apply EXT_CLK/IRAM/PQ_MAP/CR/
//!    RQPN (a full re-init, not just the pulse) — doing only the pulse breaks TX.
//!  - MCUFW_CTRL(0x0080) BIT0 = FW-download mode; setting it makes the bulk write
//!    stall (download DMA wants an allocated page) — needs the full pltfm_reset.
//!  - CAUTION: MCUFW BIT0 + reset-pulses WEDGE the WLAN TX-DMA across runs; neither
//!    software power_on, a MCUFW clear, nor a USB port reset recovers it — only a
//!    PHYSICAL replug of the f72b does. Iterate with a power-cycle between attempts.
//! NEXT: implement pltfm_reset's full re-apply body, then MCUFW, then dl_rsvd_page
//! with QSLT_BEACON — on a freshly-replugged chip.

use rusb::{Direction, TransferType};
use std::time::{Duration, Instant};

type R<T> = Result<T, Box<dyn std::error::Error>>;

const VID: u16 = 0x0bda;
const PID: u16 = 0xf72b;
const VENQT_READ: u8 = 0xC0;
const VENQT_WRITE: u8 = 0x40;
const VENQT_REQ: u8 = 0x05;
const CTRL: Duration = Duration::from_millis(500);

// Registers (halmac_reg_8733b, verified against the driver).
const REG_SYS_FUNC_EN: u16 = 0x0002;
const REG_MCUFW_CTRL: u16 = 0x0080;
const REG_PKTBUF_DBG_CTRL: u16 = 0x0140;
const REG_PKTBUF_DBG_DATA_L: u16 = 0x0144;
const REG_PKTBUF_DBG_DATA_H: u16 = 0x0148;
const REG_CR: u16 = 0x0100;
const REG_SYS_CFG1: u16 = 0x00F0;
const REG_SYS_CFG2: u16 = 0x00FC;
const REG_TXDMA_PQ_MAP: u16 = 0x010C;
const REG_RQPN_CTRL_HLPQ: u16 = 0x0200;
const REG_DWBCN0_CTRL: u16 = 0x0208;
const REG_FWHW_TXQ_CTRL: u16 = 0x0420;
const REG_BCN_CTRL: u16 = 0x0550;
const REG_EXT_SYS_FUNC_EN: u16 = 0x1000;
const REG_EXT_SYS_CLK_CTRL: u16 = 0x1008;
const QSLT_BEACON: u32 = 0x10; // vendor uses BEACON qsel for the rsvd-page download
const DMA_MAPPING_HIGH: u8 = 3;
const BIT_HCI_TXDMA_EN: u8 = 0x01;
const BIT_TXDMA_EN: u8 = 0x04;
const TX_DESC_SIZE: usize = 48; // HALMAC_TX_DESC_SIZE_8733B (NOT the generic 87xx 40)
const USB_BULK_SIZE: usize = 512; // USB2 (f72b enumerates @ 480M)

#[derive(Clone, Copy)]
enum Pwr {
    W,
    P,
}
struct Step {
    cmd: Pwr,
    off: u16,
    msk: u8,
    val: u8,
}
const fn s(cmd: Pwr, off: u16, msk: u8, val: u8) -> Step {
    Step { cmd, off, msk, val }
}
use Pwr::{P, W};
const POWER_ON: &[Step] = &[
    s(W, 0x0005, 0x08, 0x00),
    s(W, 0x004A, 0x01, 0x00),
    s(P, 0x0006, 0x02, 0x02),
    s(W, 0x0006, 0x01, 0x01),
    s(W, 0x0005, 0x01, 0x01),
    s(P, 0x0005, 0x01, 0x00),
    s(W, 0x1002, 0x01, 0x01),
    s(W, 0x0002, 0x03, 0x00),
    s(W, 0x0002, 0x03, 0x03),
    s(W, 0x0002, 0x03, 0x00),
    s(W, 0x0002, 0x03, 0x03),
    s(W, 0x0002, 0x03, 0x00),
    s(W, 0x0002, 0x03, 0x03),
    s(W, 0x001F, 0xFF, 0x00),
    s(W, 0x0077, 0xFF, 0x00),
    s(W, 0x001F, 0xFF, 0x87),
    s(W, 0x0077, 0xFF, 0x87),
    s(W, 0x001F, 0xFF, 0x00),
    s(W, 0x0077, 0xFF, 0x00),
    s(W, 0x001F, 0xFF, 0x87),
    s(W, 0x0077, 0xFF, 0x87),
    s(W, 0x001F, 0xFF, 0x00),
    s(W, 0x0077, 0xFF, 0x00),
    s(W, 0x001F, 0xFF, 0x87),
    s(W, 0x0077, 0xFF, 0x87),
];

// card_dis power-off (USB rows only), halmac_pwr_seq_8733b.c
// TRANS_ACT_TO_CARDEMU_card_dis + TRANS_CARDEMU_TO_CARDDIS — a clean MAC reset.
const POWER_OFF: &[Step] = &[
    s(W, 0x00CD, 0x01, 0x00),
    s(W, 0x0005, 0x02, 0x02),
    s(P, 0x0005, 0x02, 0x00),
    s(W, 0x0005, 0x08, 0x08),
    s(W, 0x0006, 0xC0, 0x00),
    s(W, 0x0007, 0xFF, 0x10),
    s(W, 0x1004, 0x0F, 0x02),
    s(W, 0x0024, 0x0F, 0x00),
    s(W, 0x0025, 0xF0, 0x10),
    s(W, 0x0021, 0xF0, 0x70),
    s(W, 0x004A, 0x01, 0x01),
];

/// Build the full USB packet (48-byte 8733b TX descriptor + optional 8-byte
/// packet-offset gap + payload) exactly as the vendor `usb_write_data_not_xmitframe`
/// does: dw0 [0:16]=TXPKTSIZE, [16:24]=OFFSET(=desclen, +8 if padded); dw1 [8:13]=
/// QSEL, [24:29]=PKT_OFFSET(=1 if padded). Pad by 8 (PACKET_OFFSET_SZ) when
/// desclen+payload is a multiple of the USB bulk size (ZLP avoidance).
fn build_dl_pkt(payload: &[u8], qsel: u32) -> Vec<u8> {
    let desclen = TX_DESC_SIZE;
    let pad = (desclen + payload.len()) % USB_BULK_SIZE == 0;
    let offset = if pad { desclen + 8 } else { desclen };
    let mut d = [0u8; TX_DESC_SIZE];
    let dw0 = (payload.len() as u32 & 0xFFFF) | ((offset as u32 & 0xFF) << 16);
    d[0..4].copy_from_slice(&dw0.to_le_bytes());
    let mut dw1 = (qsel & 0x1F) << 8;
    if pad {
        dw1 |= 1 << 24;
    }
    d[4..8].copy_from_slice(&dw1.to_le_bytes());
    let mut pkt = Vec::with_capacity(offset + payload.len());
    pkt.extend_from_slice(&d);
    if pad {
        pkt.extend_from_slice(&[0u8; 8]);
    }
    pkt.extend_from_slice(payload);
    pkt
}

struct Dev {
    h: rusb::DeviceHandle<rusb::GlobalContext>,
    bulk_outs: Vec<u8>,
}

impl Dev {
    fn open() -> R<Dev> {
        // USB port reset first — re-enumerate the device to clear accumulated chip
        // state from prior runs (no physical power-cycle available on the OPi).
        if let Some(h0) = rusb::open_device_with_vid_pid(VID, PID) {
            let _ = h0.reset();
            drop(h0);
            std::thread::sleep(Duration::from_millis(800));
        }
        let h = rusb::open_device_with_vid_pid(VID, PID).ok_or("f72b not found")?;
        let dev = h.device();
        let cfg = dev.active_config_descriptor()?;
        let mut bulk_outs = Vec::new();
        for iface in cfg.interfaces() {
            for d in iface.descriptors() {
                for ep in d.endpoint_descriptors() {
                    if ep.transfer_type() == TransferType::Bulk
                        && ep.direction() == Direction::Out
                    {
                        bulk_outs.push(ep.address());
                    }
                }
            }
        }
        let _ = h.set_auto_detach_kernel_driver(true);
        h.claim_interface(0)?;
        bulk_outs.sort_unstable();
        println!("opened f72b, bulk-OUT eps = {bulk_outs:02x?}");
        Ok(Dev { h, bulk_outs })
    }

    fn r8(&self, a: u16) -> R<u8> {
        let mut b = [0u8; 1];
        self.h.read_control(VENQT_READ, VENQT_REQ, a, 0, &mut b, CTRL)?;
        Ok(b[0])
    }
    fn r32(&self, a: u16) -> R<u32> {
        let mut b = [0u8; 4];
        self.h.read_control(VENQT_READ, VENQT_REQ, a, 0, &mut b, CTRL)?;
        Ok(u32::from_le_bytes(b))
    }
    fn w8(&self, a: u16, v: u8) -> R<()> {
        self.h.write_control(VENQT_WRITE, VENQT_REQ, a, 0, &[v], CTRL)?;
        Ok(())
    }
    fn w32(&self, a: u16, v: u32) -> R<()> {
        self.h.write_control(VENQT_WRITE, VENQT_REQ, a, 0, &v.to_le_bytes(), CTRL)?;
        Ok(())
    }
    fn r16(&self, a: u16) -> R<u16> {
        let mut b = [0u8; 2];
        self.h.read_control(VENQT_READ, VENQT_REQ, a, 0, &mut b, CTRL)?;
        Ok(u16::from_le_bytes(b))
    }
    fn w16(&self, a: u16, v: u16) -> R<()> {
        self.h.write_control(VENQT_WRITE, VENQT_REQ, a, 0, &v.to_le_bytes(), CTRL)?;
        Ok(())
    }
    /// Read `out.len()` bytes from the TX/RSVD packet buffer at byte `byte_off`,
    /// per read_buf_87xx: start_pg = (off>>12)+0x780 into REG_PKTBUF_DBG_CTRL, then
    /// the 4KB data window at register 0x8000+residue, 4 bytes at a time.
    fn read_pktbuf(&self, byte_off: u32, out: &mut [u8]) -> R<()> {
        let start_pg = ((byte_off >> 12) + 0x780) as u16;
        let residue = (byte_off & 0xFFF) as u16;
        let ctrl = self.r16(REG_PKTBUF_DBG_CTRL)? & 0xF000;
        self.w16(REG_PKTBUF_DBG_CTRL, start_pg | ctrl)?;
        let mut cnt = 0usize;
        let mut i = 0x8000u16.wrapping_add(residue);
        while cnt < out.len() && i <= 0x8FFF {
            let w = self.r32(i)?.to_le_bytes();
            for b in w {
                if cnt < out.len() {
                    out[cnt] = b;
                    cnt += 1;
                }
            }
            i = i.wrapping_add(4);
        }
        Ok(())
    }

    fn chip_version(&self) -> R<(u8, u8, u32)> {
        Ok((self.r8(REG_SYS_CFG2)?, self.r8(REG_SYS_CFG1 + 1)? >> 4, self.r32(REG_SYS_CFG1)?))
    }

    fn run_seq(&self, steps: &[Step]) -> R<()> {
        for st in steps {
            match st.cmd {
                Pwr::W => {
                    let cur = self.r8(st.off)?;
                    self.w8(st.off, (cur & !st.msk) | (st.val & st.msk))?;
                }
                Pwr::P => {
                    let deadline = Instant::now() + Duration::from_millis(200);
                    loop {
                        if self.r8(st.off)? & st.msk == st.val & st.msk {
                            break;
                        }
                        if Instant::now() >= deadline {
                            return Err(format!("pwr-seq poll timeout @0x{:04x}", st.off).into());
                        }
                    }
                }
            }
        }
        Ok(())
    }
    fn power_on(&self) -> R<()> {
        self.run_seq(POWER_ON)
    }
    /// Card-disable (power off) — bring the MAC down so the next power_on is a clean
    /// reset (recovers a wedged TX-DMA without a physical replug).
    fn power_off(&self) -> R<()> {
        self.run_seq(POWER_OFF)
    }

    fn is_powered(&self) -> R<bool> {
        let f = self.r8(REG_SYS_FUNC_EN)?;
        let cr = self.r8(REG_CR)?;
        Ok(f & 3 == 3 && cr != 0xEA && cr != 0xFF)
    }

    fn fw_dl_setup(&self) -> R<()> {
        self.w8(REG_EXT_SYS_CLK_CTRL, self.r8(REG_EXT_SYS_CLK_CTRL)? | 0x02)?;
        self.w32(REG_EXT_SYS_FUNC_EN, (self.r32(REG_EXT_SYS_FUNC_EN)? | 0x0000_3000) & 0xFFFF_FF3F)?;
        self.w8(REG_TXDMA_PQ_MAP + 1, DMA_MAPPING_HIGH << 6)?;
        self.w8(REG_CR, BIT_HCI_TXDMA_EN | BIT_TXDMA_EN)?;
        self.w8(REG_RQPN_CTRL_HLPQ, 0xD0)?;
        self.w8(REG_RQPN_CTRL_HLPQ + 1, 0x00)?;
        self.w8(REG_RQPN_CTRL_HLPQ + 2, 0x20)?;
        self.w8(REG_RQPN_CTRL_HLPQ + 3, 0x80)?;
        let bcn = self.r8(REG_BCN_CTRL)?;
        self.w8(REG_BCN_CTRL, (bcn & !(1 << 3)) | (1 << 4))?;
        if std::env::var("PLTFM_RST").is_ok() {
            // pltfm_reset: toggle BIT0 of REG_EXT_SYS_FUNC_EN+2 (0x1002).
            let r = self.r8(REG_EXT_SYS_FUNC_EN + 2)?;
            self.w8(REG_EXT_SYS_FUNC_EN + 2, r & !1)?;
            let r = self.r8(REG_EXT_SYS_FUNC_EN + 2)?;
            self.w8(REG_EXT_SYS_FUNC_EN + 2, r | 1)?;
            println!("(pltfm_reset applied)");
        }
        Ok(())
    }

    /// **M4.5 — the slot to fill.** Program the TX-FIFO page allocation +
    /// reserved-page boundary + RQPN (from init_mac_cfg_87xx / txff_alloc) so the
    /// reserved-page bulk write below actually drains. Empty for now: the first run
    /// should reproduce the dl_rsvd_page timeout, confirming the blocker on silicon.
    fn trx_init(&self) -> R<()> {
        // wlan_cpu_en(0): disable the WLAN CPU (clear BIT2 of REG_SYS_FUNC_EN+1) so
        // it doesn't own the TX/beacon buffer during a manual reserved-page write —
        // the piece of the vendor download_firmware prologue our fw_dl_setup missed.
        // wlan_cpu_en(0): disable the WLAN CPU (clear BIT2 of SYS_FUNC_EN+1). The
        // reset pulse (0x1002 BIT0 toggle) is NOT done here — it broke the write
        // (all endpoints timed out); pltfm_reset needs its full re-apply body.
        let v = self.r8(REG_SYS_FUNC_EN + 1)?;
        self.w8(REG_SYS_FUNC_EN + 1, v & !(1 << 2))?;
        println!("M4.5 cpu_en(0)");
        Ok(())
    }

    fn dl_rsvd_page(&self, pg_addr: u8, data: &[u8], ep: u8, qsel: u32) -> R<()> {
        self.w8(REG_DWBCN0_CTRL + 1, pg_addr)?;
        self.w8(REG_DWBCN0_CTRL + 2, self.r8(REG_DWBCN0_CTRL + 2)? | 0x01)?;
        let cr1 = self.r8(REG_CR + 1)?;
        self.w8(REG_CR + 1, cr1 | 0x01)?;
        let txq2 = self.r8(REG_FWHW_TXQ_CTRL + 2)?;
        self.w8(REG_FWHW_TXQ_CTRL + 2, txq2 & !(1 << 6))?;

        let pkt = build_dl_pkt(data, qsel);
        let wrote = self.h.write_bulk(ep, &pkt, Duration::from_millis(600));
        let write_res = match wrote {
            Ok(n) if n == pkt.len() => Ok(()),
            Ok(n) => Err(format!("short bulk write {n}/{}", pkt.len())),
            Err(e) => Err(format!("bulk write err: {e}")),
        };

        let mut valid = false;
        if write_res.is_ok() {
            let deadline = Instant::now() + Duration::from_millis(200);
            while Instant::now() < deadline {
                if self.r8(REG_DWBCN0_CTRL + 2)? & 0x01 != 0 {
                    valid = true;
                    break;
                }
            }
        }
        self.w8(REG_FWHW_TXQ_CTRL + 2, txq2)?;
        self.w8(REG_CR + 1, cr1)?;
        match write_res {
            Err(e) => Err(e.into()),
            Ok(()) if valid => Ok(()),
            Ok(()) => Err("BCN_VALID poll timeout".into()),
        }
    }
}

fn main() -> R<()> {
    let dev = Dev::open()?;
    let (id, ver, cfg1) = dev.chip_version()?;
    println!("M1  chip_id=0x{id:02x} chip_ver={ver} sys_cfg1=0x{cfg1:08x}  (golden id==0x16: {})", id == 0x16);
    // Clean per-run reset: card-disable (power off) then power on, so accumulated
    // download-mode/TX-DMA state from a prior run doesn't confound this one.
    dev.w8(REG_MCUFW_CTRL, 0)?;
    dev.w8(REG_MCUFW_CTRL + 1, 0)?;
    if std::env::var("PWROFF").is_ok() {
        let off = dev.power_off();
        println!("reset: power_off {}", if off.is_ok() { "ok" } else { "err" });
    }
    dev.power_on()?;
    println!("M2  power_on ok: powered={} CR=0x{:02x} func_en=0x{:02x}", dev.is_powered()?, dev.r8(REG_CR)?, dev.r8(REG_SYS_FUNC_EN)?);
    // Vendor download order: cpu_en(0) + reset pulse, THEN (re-)apply the page
    // config, THEN download-enable, THEN the reserved-page write.
    dev.trx_init()?;
    dev.fw_dl_setup()?;
    println!("M4  fw_dl_setup ok: CR=0x{:02x} EXT_FUNC=0x{:08x}", dev.r8(REG_CR)?, dev.r32(REG_EXT_SYS_FUNC_EN)?);
    if std::env::var("MCUFW").is_ok() {
        let hi = dev.r8(REG_MCUFW_CTRL + 1)? & 0x38;
        dev.w8(REG_MCUFW_CTRL, 0x01)?;
        dev.w8(REG_MCUFW_CTRL + 1, hi)?;
        println!("M4.5 MCUFW_CTRL BIT0 (download mode)");
    }
    // Vendor-exact descriptor now: 48-byte 8733b TX desc, QSEL=BEACON(0x10), and
    // the HIGH bulk-OUT endpoint (bulkout id 0 = first ep), which is where the
    // beacon/rsvd-page write must go to raise BCN_VALID.
    let ep = *dev.bulk_outs.first().unwrap();
    let mcufw = if std::env::var("MCUFW").is_ok() { "MCUFW" } else { "no-MCUFW" };
    let r = dev.dl_rsvd_page(0x00, &[0xA5u8; 128], ep, QSLT_BEACON);
    println!("M4.5 dl_rsvd_page (BEACON, pg=0, ep 0x{ep:02x}, {mcufw}): {}",
        match &r { Ok(()) => "BCN_VALID OK — DRAINED!".to_string(), Err(e) => e.to_string() });
    // Decisive: did the 0xA5 payload land in the TX packet buffer? Read the first
    // 4KB and count 0xA5 — independent of BCN_VALID.
    let mut buf = vec![0u8; 4096];
    dev.read_pktbuf(0, &mut buf)?;
    let found = buf.iter().filter(|&&x| x == 0xA5).count();
    // Show where the 0xA5 run starts (should be right after a 48B descriptor).
    if let Some(pos) = buf.windows(8).position(|w| w.iter().all(|&x| x == 0xA5)) {
        println!("  first 0xA5 run at pktbuf offset 0x{pos:x}; head = {:02x?}", &buf[..16.min(buf.len())]);
    }
    println!("pktbuf scan: {found} bytes of 0xA5 in first 4KB → write {} land",
        if found >= 64 { "DID" } else { "did NOT" });
    Ok(())
}
