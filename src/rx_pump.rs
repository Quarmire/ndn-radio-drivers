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
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex};
use std::thread::JoinHandle;
use std::time::Duration;

/// The RX pipeline's shared state: the frame queue the pump fills and `recv_frame` drains, its
/// wake signal, and whether a pump is currently running.
pub struct RxPumpState {
    pending: Mutex<VecDeque<CapturedFrame>>,
    notify: tokio::sync::Notify,
    pumped: AtomicBool,
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
        }
    }

    /// Whether a background pump is filling the queue (vs. `recv_frame` doing its own one-shot read).
    pub fn is_pumped(&self) -> bool {
        self.pumped.load(Ordering::Relaxed)
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
                let mut buf = vec![0u8; 16384];
                loop {
                    let Some(b) = weak.upgrade() else { break };
                    match handle.read_bulk(ep, &mut buf, Duration::from_millis(200)) {
                        Ok(n) if n > 0 => b.pump_state().push(b.parse_transfer(&buf[..n])),
                        _ => {} // timeout / empty / error: re-submit the read
                    }
                }
            })
        })
        .collect()
}
