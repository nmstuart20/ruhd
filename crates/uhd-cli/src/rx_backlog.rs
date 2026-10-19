//! Application samples received but not yet confirmed written to the output.

use std::sync::Mutex;
use std::sync::atomic::{AtomicU64, AtomicUsize, Ordering};
use std::time::Instant;

use crate::rx_latency::{BlockTiming, Latency};

#[derive(Debug, Default)]
pub(crate) struct Backlog {
    pending: AtomicUsize,
    peak: AtomicUsize,
    written: AtomicU64,
    latency: Mutex<Latency>,
}

#[derive(Clone, Copy, Debug, Default)]
pub(crate) struct Snapshot {
    pub(crate) pending: usize,
    pub(crate) peak: usize,
    pub(crate) written: u64,
    pub(crate) latency: Latency,
}

impl Backlog {
    pub(crate) fn received(&self, bytes: usize) {
        let pending = self.pending.fetch_add(bytes, Ordering::SeqCst) + bytes;
        self.peak.fetch_max(pending, Ordering::SeqCst);
    }

    pub(crate) fn written(&self, bytes: usize) {
        self.written.fetch_add(bytes as u64, Ordering::SeqCst);
        self.pending.fetch_sub(bytes, Ordering::SeqCst);
    }

    pub(crate) fn completed(&self, timing: BlockTiming, started: Instant, finished: Instant) {
        self.latency
            .lock()
            .expect("latency stats poisoned")
            .completed(timing, started, finished);
    }

    pub(crate) fn failed_write(&self) {
        self.latency
            .lock()
            .expect("latency stats poisoned")
            .failed_writes += 1;
    }

    pub(crate) fn snapshot(&self) -> Snapshot {
        let pending = self.pending.load(Ordering::SeqCst);
        Snapshot {
            pending,
            peak: self.peak.load(Ordering::SeqCst).max(pending),
            written: self.written.load(Ordering::SeqCst),
            latency: *self.latency.lock().expect("latency stats poisoned"),
        }
    }
}

impl Snapshot {
    pub(crate) fn report(self, host_bytes: usize, rate: f64) {
        eprintln!(
            "  application backlog: {} bytes ({} samples, {:.3} ms); peak {} bytes ({:.3} ms)",
            self.pending,
            self.pending / host_bytes,
            self.pending as f64 / host_bytes as f64 / rate * 1000.0,
            self.peak,
            self.peak as f64 / host_bytes as f64 / rate * 1000.0,
        );
        self.latency.report();
    }
}
