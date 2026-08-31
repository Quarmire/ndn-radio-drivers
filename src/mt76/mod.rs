//! Shared MediaTek **mt76x02** layer — the register map, USB transport and knob
//! implementations common to the mt76x0 (MT7610U) and mt76x2 (MT7612U) parts.
//!
//! The two families ship one register header upstream (`mt76x02_regs.h`), so a
//! knob validated on either part is validated for both. That is not a
//! convenience: the MT7612U in this lab is currently unreachable (its control
//! endpoint stopped answering), while the MT7610U is healthy — so every mt76x02
//! register semantic recorded in [`knobs`] was MEASURED on the 7610 and applies
//! to the 7612 by shared silicon, not by hope.
//!
//! Deliberately additive: the existing [`crate::Mt7612uBackend`] keeps its own
//! proven transport. Shared code here is written against the [`Mt76Regs`] seam
//! rather than against a concrete backend, so either driver can adopt it one
//! method at a time instead of through a refactor of a working radio.

pub mod knobs;
pub mod regs;
pub mod transport;

/// The minimal register seam the shared mt76 code is written against.
///
/// A backend implements the two primitives; everything in [`knobs`] is then
/// available to it. Kept object-safe (`&dyn Mt76Regs`) so one copy of the knob
/// code serves both families without monomorphising per backend.
pub trait Mt76Regs: Send + Sync {
    /// Read a 32-bit MMIO register (USB vendor request `MT_VEND_MULTI_READ`).
    ///
    /// ⚠ MEASURED 151 µs per round trip on a high-speed mt76 USB part. That is
    /// the floor on anything built out of register reads — it rules out a
    /// per-frame register stamp, and it means a sampling loop costs real time.
    fn rr(&self, addr: u32) -> Result<u32, crate::FaceError>;

    /// Write a 32-bit MMIO register (`MT_VEND_MULTI_WRITE`).
    fn wr(&self, addr: u32, val: u32) -> Result<(), crate::FaceError>;

    /// Read-modify-write, returning the value **before** the write so a caller
    /// can restore it. Two round trips — see the latency note on [`rr`](Self::rr).
    fn rmw(&self, addr: u32, clear: u32, set: u32) -> Result<u32, crate::FaceError> {
        let old = self.rr(addr)?;
        self.wr(addr, (old & !clear) | set)?;
        Ok(old)
    }
}

/// Which mt76x02 family a transport is talking to. One register differs in
/// *space* rather than in value between them — the USB DMA config is CFG-space
/// `0x9018` on mt76x2 and plain MMIO `0x0238` on mt76x0 — so the transport needs
/// to know which part it holds. Everything else in this module is shared.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Family {
    /// MT7610U / MT7650 — 1×1, `mt76x0u` upstream.
    Mt76x0,
    /// MT7612U / MT7662 — 2×2, `mt76x2u` upstream.
    Mt76x2,
}
