//! Shared RX pipelining — the async-URB-style bulk-IN pump the USB backends use so a busy channel
//! is not dropped between userspace reads and each frame's hardware stamp is taken promptly
//! (named-time §3c / §13, "owning the RXWI parse puts the stamp immediately off bulk-IN").
//!
//! The loop is identical across chips — keep `depth` bulk-IN transfers in flight, parse each into a
//! shared queue, wake the consumer — only the per-transfer *parse* differs. So a backend embeds an
//! [`RxPumpState`], implements [`Pumpable`] (its handle, endpoint, and parse), and calls
//! [`spawn_rx_pump`]; `recv_frame` then drains the queue via [`RxPumpState::recv`]. One
//! implementation, every Realtek USB backend.

use crate::CapturedFrame;
use rusb::{Context, DeviceHandle};
use std::collections::VecDeque;
use std::sync::atomic::{AtomicBool, AtomicU64, AtomicUsize, Ordering};
use std::sync::{Arc, Condvar, Mutex, Weak};
use std::thread::JoinHandle;
use std::time::Duration;

/// The RX pipeline's shared state: the frame queue the pump fills and `recv_frame` drains, its
/// wake signal, and whether a pump is currently running.
pub struct RxPumpState {
    pending: Mutex<VecDeque<CapturedFrame>>,
    notify: tokio::sync::Notify,
    pumped: AtomicBool,
    /// Non-timeout bulk-IN errors since start (stall/pipe/other) — the wedge-approach signal. A
    /// healthy pump reports 0; a climbing count is a dongle going bad BEFORE the xhci gives up on
    /// it (both 8812au-family parts hit resets→disconnect under sustained campaign RX, 2026-08-13).
    rx_errors: AtomicU64,
    /// Endpoint stalls we cleared (clear_halt succeeded) — recoverable; distinct from fatal.
    rx_stalls_cleared: AtomicU64,
}

impl Default for RxPumpState {
    fn default() -> Self {
        Self::new()
    }
}

impl RxPumpState {
    /// An empty, un-pumped pipeline.
    pub fn new() -> Self {
        Self {
            pending: Mutex::new(VecDeque::new()),
            notify: tokio::sync::Notify::new(),
            pumped: AtomicBool::new(false),
            rx_errors: AtomicU64::new(0),
            rx_stalls_cleared: AtomicU64::new(0),
        }
    }

    /// Whether a background pump is filling the queue (vs. `recv_frame` doing its own one-shot read).
    pub fn is_pumped(&self) -> bool {
        self.pumped.load(Ordering::Relaxed)
    }

    /// `(non-timeout errors, stalls cleared)` — the pump-health counters. Poll during a run to
    /// see a dongle degrading before the host controller disconnects it.
    pub fn rx_health(&self) -> (u64, u64) {
        (self.rx_errors.load(Ordering::Relaxed), self.rx_stalls_cleared.load(Ordering::Relaxed))
    }

    /// Buffer decoded frames and wake one waiting [`recv`](Self::recv).
    pub fn push<I: IntoIterator<Item = CapturedFrame>>(&self, frames: I) {
        {
            let mut q = self.pending.lock().unwrap();
            q.extend(frames);
        }
        self.notify.notify_one();
    }

    /// Take one buffered frame if present.
    pub fn try_pop(&self) -> Option<CapturedFrame> {
        self.pending.lock().unwrap().pop_front()
    }

    /// Drain one frame, waiting on the pump's wake signal when the queue is empty (re-polling on a
    /// timeout in case a wake was missed).
    pub async fn recv(&self) -> CapturedFrame {
        loop {
            let notified = self.notify.notified();
            if let Some(f) = self.try_pop() {
                return f;
            }
            let _ = tokio::time::timeout(Duration::from_millis(200), notified).await;
        }
    }

    fn mark_pumped(&self) {
        self.pumped.store(true, Ordering::Relaxed);
    }
}

/// A USB backend whose bulk-IN can be pipelined by [`spawn_rx_pump`]. The chip-specific parse is
/// the only difference between backends; everything else the pump owns.
pub trait Pumpable: Send + Sync + 'static {
    /// A clone of the device handle the reader threads issue bulk-IN transfers on.
    fn pump_handle(&self) -> Arc<DeviceHandle<Context>>;
    /// The bulk-IN endpoint address.
    fn pump_bulk_in(&self) -> u8;
    /// Parse one bulk-IN transfer into the frames it carried (the chip aggregates several RX units
    /// per USB transfer).
    fn parse_transfer(&self, buf: &[u8]) -> Vec<CapturedFrame>;
    /// The shared pipeline state to fill / drain.
    fn pump_state(&self) -> &RxPumpState;
}

/// Spawn `depth` reader threads that keep `depth` bulk-IN transfers in flight (the async-URB path),
/// each parsing a transfer into the shared queue. The threads hold a [`Weak`](std::sync::Weak) ref,
/// so they exit when the backend is dropped. Returns their join handles.
pub fn spawn_rx_pump<B: Pumpable>(backend: &Arc<B>, depth: usize) -> Vec<JoinHandle<()>> {
    backend.pump_state().mark_pumped();
    let handle = backend.pump_handle();
    let ep = backend.pump_bulk_in();
    (0..depth.max(1))
        .map(|_| {
            let weak = Arc::downgrade(backend);
            let handle = handle.clone();
            std::thread::spawn(move || {
                // 32 KB so a USB-aggregated bulk-IN transfer (8812au REG_RXDMA_AGG_PG_TH size = 16 KB in
                // 512-B units + slack) fits in ONE read. With USB RX aggregation enabled (RXDMA_AGG_EN),
                // the chip packs many frames per transfer; a too-small buffer would truncate them.
                let mut buf = vec![0u8; 32768];
                let dbg = std::env::var_os("NDN_RX_AGG_DBG").is_some();
                let (mut reads, mut bytes) = (0u64, 0u64);
                // Consecutive hard errors → the endpoint is not coming back; stop hammering it.
                // A tight busy-loop re-submitting to a NAKing/stalled endpoint is a plausible
                // accelerant of the xhci reset→disconnect wedge (2026-08-13); this backs off and
                // records, instead of spinning.
                let mut consec_err = 0u32;
                loop {
                    let Some(b) = weak.upgrade() else { break };
                    match handle.read_bulk(ep, &mut buf, Duration::from_millis(200)) {
                        Ok(n) if n > 0 => {
                            consec_err = 0;
                            if dbg {
                                reads += 1;
                                bytes += n as u64;
                                if reads % 300 == 0 {
                                    eprintln!("PUMP: {reads} reads, avg {} B/transfer (last {n} B)", bytes / reads);
                                }
                            }
                            b.pump_state().push(b.parse_transfer(&buf[..n]))
                        }
                        // Benign: no frame arrived in the window. Re-submit immediately — this is
                        // the overwhelmingly common non-data case and must not back off.
                        Ok(_) | Err(rusb::Error::Timeout) => consec_err = 0,
                        // The device is gone (unplug / the wedge already happened): exit the thread
                        // cleanly rather than spin on a dead handle.
                        Err(rusb::Error::NoDevice) | Err(rusb::Error::NotFound) => break,
                        // A stalled endpoint is sometimes recoverable: clear the halt and continue.
                        Err(rusb::Error::Pipe) => {
                            let n = b.pump_state().rx_errors.fetch_add(1, Ordering::Relaxed) + 1;
                            if handle.clear_halt(ep).is_ok() {
                                b.pump_state().rx_stalls_cleared.fetch_add(1, Ordering::Relaxed);
                            }
                            consec_err += 1;
                            if n == 1 || n % 100 == 0 {
                                eprintln!("PUMP HEALTH: ep {ep:#04x} stall (clear_halt), {n} rx errors total — dongle degrading");
                            }
                            std::thread::sleep(Duration::from_millis(2));
                        }
                        // Any other error (I/O, overflow, busy): count and back off briefly so a
                        // failing endpoint is not hammered at CPU speed.
                        Err(e) => {
                            let n = b.pump_state().rx_errors.fetch_add(1, Ordering::Relaxed) + 1;
                            consec_err += 1;
                            if n == 1 || n % 100 == 0 {
                                eprintln!("PUMP HEALTH: ep {ep:#04x} err {e:?}, {n} rx errors total — dongle degrading");
                            }
                            std::thread::sleep(Duration::from_millis(2));
                        }
                    }
                    // ~1 s of solid errors: the endpoint is not recovering. Stop the thread; a
                    // dead pump is visible (rx_health), a busy-looping one wedges the bus.
                    if consec_err > 500 {
                        eprintln!("PUMP HEALTH: ep {ep:#04x} — 500 consecutive errors, stopping reader (endpoint dead; a busy-loop here is what wedges the xhci bus)");
                        break;
                    }
                }
            })
        })
        .collect()
}

// ── Async (submit-ahead) RX pump — libusb async FFI ──────────────────────────────────────────────
//
// The sync `spawn_rx_pump` above does one blocking `read_bulk` per thread: submit → wait → parse →
// resubmit, so each thread has NO transfer in flight while it parses, and libusb's SYNC API also
// serialises threads on its internal event lock (which is why 8→48 threads only moved 8812au RX
// 528→560). The KERNEL rtw88_8812au pulls ~1106 f/s off the SAME chip by keeping many URBs
// continuously in flight, completion-driven. This mirrors that: a pool of `depth` bulk-IN transfers,
// each resubmitted the instant its callback runs, driven by ONE `libusb_handle_events` thread.
// Enable with `NDN_ASYNC_PUMP=1`.

/// Raw bulk-IN buffers awaiting parse — filled by the completion callback (a cheap memcpy on the
/// event thread), drained by a pool of parse threads so de-aggregation/RSSI/TSF extraction runs in
/// PARALLEL and off the single event thread (which otherwise serialised parse and capped the rate).
struct RawQueue {
    q: Mutex<VecDeque<Vec<u8>>>,
    cv: Condvar,
}

/// Heap context carried as each transfer's `user_data` — leaked with `Box::into_raw`, reclaimed in
/// the callback only when the backend is gone (Weak fails).
struct AsyncXfer<B: Pumpable> {
    backend: Weak<B>,
    raw: Arc<RawQueue>,
    inflight: Arc<AtomicUsize>,
    buf: Vec<u8>,
}

/// libusb completion callback (runs on the event thread): hand the received bytes to the parse pool
/// (copy — the buffer is reused on resubmit) and resubmit immediately, so the transfer is back in
/// flight without waiting for parse. Frees the transfer only when the backend is gone.
extern "system" fn async_xfer_cb<B: Pumpable>(t: *mut rusb::ffi::libusb_transfer) {
    let ctxp = unsafe { (*t).user_data as *mut AsyncXfer<B> };
    let ctx = unsafe { &mut *ctxp };
    if ctx.backend.strong_count() > 0 {
        let status = unsafe { (*t).status };
        let n = unsafe { (*t).actual_length } as usize;
        if status == rusb::constants::LIBUSB_TRANSFER_COMPLETED && n > 0 {
            let mut q = ctx.raw.q.lock().unwrap();
            if q.len() < 2048 {
                // bounded — drop under sustained overload rather than OOM
                q.push_back(ctx.buf[..n].to_vec());
                drop(q);
                ctx.raw.cv.notify_one();
            }
        }
        // Resubmit the same transfer (buffer reused) — back in flight immediately.
        if unsafe { rusb::ffi::libusb_submit_transfer(t) } == 0 {
            return;
        }
    }
    // Backend gone (or resubmit failed): free the transfer + reclaim the leaked box.
    ctx.inflight.fetch_sub(1, Ordering::SeqCst);
    unsafe {
        rusb::ffi::libusb_free_transfer(t);
        drop(Box::from_raw(ctxp));
    }
}

/// Raw libusb context pointer wrapper so it can move into the event thread (`*mut` isn't `Send`).
struct SendCtx(*mut rusb::ffi::libusb_context);
unsafe impl Send for SendCtx {}

/// Start the async submit-ahead pump: `depth` bulk-IN transfers always in flight, one event thread,
/// and a pool of parse threads draining the raw queue in parallel.
pub fn spawn_rx_pump_async<B: Pumpable>(backend: &Arc<B>, depth: usize) -> JoinHandle<()> {
    use rusb::UsbContext;
    use std::os::raw::{c_int, c_void};
    backend.pump_state().mark_pumped();
    let handle = backend.pump_handle();
    let dev = handle.as_raw();
    let ctx = SendCtx(handle.context().as_raw());
    let ep = backend.pump_bulk_in();
    let inflight = Arc::new(AtomicUsize::new(0));
    let raw = Arc::new(RawQueue { q: Mutex::new(VecDeque::new()), cv: Condvar::new() });

    // Parse pool: de-aggregate each transfer into frames on its own thread, in parallel.
    let n_parse = 6.min(depth.max(1)).max(2);
    for _ in 0..n_parse {
        let raw = raw.clone();
        let wb = Arc::downgrade(backend);
        std::thread::spawn(move || loop {
            let buf = {
                let mut q = raw.q.lock().unwrap();
                while q.is_empty() {
                    q = raw.cv.wait(q).unwrap();
                }
                q.pop_front()
            };
            match (buf, wb.upgrade()) {
                (Some(buf), Some(b)) => b.pump_state().push(b.parse_transfer(&buf)),
                _ => break, // backend dropped
            }
        });
    }

    for _ in 0..depth.max(1) {
        let boxed = Box::new(AsyncXfer::<B> {
            backend: Arc::downgrade(backend),
            raw: raw.clone(),
            inflight: inflight.clone(),
            buf: vec![0u8; 32768],
        });
        let ctxp = Box::into_raw(boxed);
        unsafe {
            let t = rusb::ffi::libusb_alloc_transfer(0);
            if t.is_null() {
                drop(Box::from_raw(ctxp));
                continue;
            }
            rusb::ffi::libusb_fill_bulk_transfer(
                t,
                dev,
                ep,
                (*ctxp).buf.as_mut_ptr(),
                (*ctxp).buf.len() as c_int,
                async_xfer_cb::<B>,
                ctxp as *mut c_void,
                0, // timeout 0 = never expire — the transfer completes when a frame arrives
            );
            if rusb::ffi::libusb_submit_transfer(t) == 0 {
                inflight.fetch_add(1, Ordering::SeqCst);
            } else {
                rusb::ffi::libusb_free_transfer(t);
                drop(Box::from_raw(ctxp));
            }
        }
    }

    // Single event thread drives every transfer's completion (no per-transfer event-lock contention).
    // Runs until all transfers have been freed (which only happens after the backend is dropped).
    std::thread::spawn(move || {
        let ctx = ctx; // move the wrapper in
        while inflight.load(Ordering::SeqCst) > 0 {
            unsafe {
                rusb::ffi::libusb_handle_events(ctx.0);
            }
        }
    })
}
