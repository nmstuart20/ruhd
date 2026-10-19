//! The receive driver thread: it owns the bulk IN endpoint and keeps it full.
//!
//! Reaping USB transfers on the thread that calls `recv` ties the kernel
//! queue to the application: while the caller converts samples, writes a
//! file or is descheduled, finished transfers sit unreaped and nothing new is
//! submitted. Once every queued transfer has filled, the FX3 has nowhere to
//! put data and the FPGA overflows. With 128 transfers of 8 KiB that is about
//! 8.5 ms at 30.72 MS/s, and a desktop scheduler misses that now and then.
//!
//! Here a dedicated thread does nothing but wait for a completion, submit a
//! replacement from a pool of empty buffers, and pass the filled buffer to
//! the receive side over a bounded queue. Buffers go back to the pool once
//! their samples have been copied out. The kernel queue stays at
//! `num_recv_frames`; the pool behind it adds `recv_buff_size` bytes of host
//! buffering (32 MiB by default, a quarter second at 30.72 MS/s `sc16`), so a
//! slow consumer only overflows the device once it is that far behind.
//!
//! The thread can be made real-time: `SCHED_FIFO`, pinned to one core, with
//! the process's pages locked. See [`DriverConfig`]. nusb's own event thread,
//! which reaps the URBs and wakes this one, gets the same treatment.

use std::fmt;
use std::sync::Arc;
use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::sync::mpsc::{self, Receiver, RecvTimeoutError, SyncSender, TryRecvError};
use std::thread::JoinHandle;
use std::time::Duration;

use nusb::transfer::{Buffer, Bulk, Completion, In, TransferError};
use tracing::{debug, warn};
use uhd_core::{Error, Result};

/// How long the thread blocks before checking whether it should stop. This
/// bounds how long dropping a stream waits once samples stop arriving.
const POLL: Duration = Duration::from_millis(10);
/// Failed transfers in a row after which the thread gives up instead of
/// resubmitting forever, which matters most when it runs at real-time
/// priority.
const MAX_CONSECUTIVE_ERRORS: u32 = 64;
/// Highest CPU index `sched_setaffinity` accepts through a `cpu_set_t`.
pub(crate) const MAX_CPU: usize = 1023;

/// Scheduling options for the driver thread. All are off by default.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub(crate) struct DriverConfig {
    /// Run under `SCHED_FIFO` at this priority, 1 to 99. Needs
    /// `CAP_SYS_NICE` or an `RLIMIT_RTPRIO` at least this high.
    pub(crate) rt_priority: Option<i32>,
    /// Pin the thread to this CPU.
    pub(crate) cpu: Option<usize>,
    /// `mlockall(MCL_CURRENT)` once the buffer pool exists, so neither the
    /// pool nor the thread's stack can be paged out. Needs `CAP_IPC_LOCK` or
    /// an `RLIMIT_MEMLOCK` larger than the process.
    pub(crate) mlock: bool,
}

/// A completed transfer: its buffer, or the error it failed with and the
/// buffer to return to the pool.
pub(crate) type Filled = std::result::Result<Buffer, (TransferError, Buffer)>;

/// What [`RxDriver::recv`] found.
#[derive(Debug)]
pub(crate) enum Recv {
    /// A completed transfer.
    Filled(Filled),
    /// Nothing completed before the timeout.
    Timeout,
    /// The driver thread has exited and will deliver nothing more.
    Stopped,
}

/// The endpoint operations the thread uses, so it can be tested without a
/// device.
pub(crate) trait TransferQueue: Send + 'static {
    fn submit(&mut self, buf: Buffer);
    fn pending(&self) -> usize;
    /// Only called with at least one transfer pending.
    fn wait_next_complete(&mut self, timeout: Duration) -> Option<Completion>;
    fn cancel_all(&mut self);
}

impl TransferQueue for nusb::Endpoint<Bulk, In> {
    fn submit(&mut self, buf: Buffer) {
        nusb::Endpoint::submit(self, buf);
    }

    fn pending(&self) -> usize {
        nusb::Endpoint::pending(self)
    }

    fn wait_next_complete(&mut self, timeout: Duration) -> Option<Completion> {
        nusb::Endpoint::wait_next_complete(self, timeout)
    }

    fn cancel_all(&mut self) {
        nusb::Endpoint::cancel_all(self);
    }
}

/// Counters the thread updates and the receive side reads.
#[derive(Debug, Default)]
struct Shared {
    stop: AtomicBool,
    /// Times a completed transfer could not be replaced at once because
    /// every spare buffer was waiting to be read: the host buffer was full.
    /// Counted once per episode, not once per transfer.
    starved: AtomicU64,
}

/// The receive side's handle on the driver thread.
pub(crate) struct RxDriver {
    filled: Receiver<Filled>,
    free: SyncSender<Buffer>,
    shared: Arc<Shared>,
    thread: Option<JoinHandle<()>>,
    buffers: usize,
}

impl fmt::Debug for RxDriver {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("RxDriver")
            .field("buffers", &self.buffers)
            .field("starved", &self.starved())
            .field("running", &self.thread.is_some())
            .finish_non_exhaustive()
    }
}

impl RxDriver {
    /// Submits the first `depth` buffers of `pool` on the calling thread,
    /// then hands `queue` and the rest of the pool to a new driver thread.
    ///
    /// Submitting here rather than on the thread means transfers are queued
    /// by the time this returns, so a flush straight afterwards sees any
    /// stale packets.
    pub(crate) fn start<Q: TransferQueue>(
        mut queue: Q,
        depth: usize,
        pool: Vec<Buffer>,
        config: DriverConfig,
    ) -> Result<Self> {
        assert!(
            depth >= 1 && pool.len() >= depth,
            "the pool must cover the kernel queue"
        );
        let buffers = pool.len();
        // Every buffer is in exactly one place, so neither queue can ever be
        // full and neither side ever blocks sending.
        let (filled_tx, filled) = mpsc::sync_channel(buffers);
        let (free, free_rx) = mpsc::sync_channel(buffers);
        let mut pool = pool.into_iter();
        for buf in pool.by_ref().take(depth) {
            queue.submit(buf);
        }
        for buf in pool {
            free.try_send(buf).expect("free queue holds the whole pool");
        }
        let shared = Arc::new(Shared::default());
        let thread_shared = Arc::clone(&shared);
        let thread = std::thread::Builder::new()
            .name("ruhd-rx-usb".into())
            .spawn(move || {
                apply_config(config);
                run(queue, depth, &filled_tx, &free_rx, &thread_shared);
            })
            .map_err(|e| Error::Transport(format!("cannot start the rx driver thread: {e}")))?;
        Ok(Self {
            filled,
            free,
            shared,
            thread: Some(thread),
            buffers,
        })
    }

    /// Waits up to `timeout` for the next completed transfer.
    pub(crate) fn recv(&self, timeout: Duration) -> Recv {
        match self.filled.recv_timeout(timeout) {
            Ok(filled) => Recv::Filled(filled),
            Err(RecvTimeoutError::Timeout) => Recv::Timeout,
            Err(RecvTimeoutError::Disconnected) => Recv::Stopped,
        }
    }

    /// Returns an emptied buffer to the pool.
    pub(crate) fn release(&self, buf: Buffer) {
        // Full is impossible (see `start`); disconnected means the thread
        // has exited and the buffer is simply freed.
        let _ = self.free.try_send(buf);
    }

    /// Host-buffer-full episodes so far.
    pub(crate) fn starved(&self) -> u64 {
        self.shared.starved.load(Ordering::Relaxed)
    }

    /// Stops the thread, cancelling and reaping every queued transfer.
    /// Idempotent.
    pub(crate) fn shutdown(&mut self) {
        self.shared.stop.store(true, Ordering::Release);
        if let Some(thread) = self.thread.take()
            && thread.join().is_err()
        {
            warn!("the b2xx rx driver thread panicked");
        }
    }
}

impl Drop for RxDriver {
    fn drop(&mut self) {
        self.shutdown();
    }
}

/// The driver thread's loop.
fn run<Q: TransferQueue>(
    mut queue: Q,
    depth: usize,
    filled: &SyncSender<Filled>,
    free: &Receiver<Buffer>,
    shared: &Shared,
) {
    let mut starved = false;
    let mut errors = 0;
    while !shared.stop.load(Ordering::Acquire) {
        // Top up after an episode where no spare buffer was available.
        while queue.pending() < depth {
            match free.try_recv() {
                Ok(buf) => queue.submit(buf),
                Err(_) => break,
            }
        }
        if queue.pending() == 0 {
            // The receive side holds every buffer; wait for one back.
            match free.recv_timeout(POLL) {
                Ok(buf) => queue.submit(buf),
                Err(RecvTimeoutError::Timeout) => {}
                Err(RecvTimeoutError::Disconnected) => break,
            }
            continue;
        }
        let Some(done) = queue.wait_next_complete(POLL) else {
            continue;
        };
        // Replace the transfer before anything else.
        match free.try_recv() {
            Ok(buf) => {
                queue.submit(buf);
                starved = false;
            }
            Err(TryRecvError::Empty) => {
                if !starved {
                    shared.starved.fetch_add(1, Ordering::Relaxed);
                    starved = true;
                }
            }
            Err(TryRecvError::Disconnected) => break,
        }
        let fatal = match &done.status {
            Ok(()) => {
                errors = 0;
                false
            }
            Err(e) => {
                errors += 1;
                *e == TransferError::Disconnected || errors >= MAX_CONSECUTIVE_ERRORS
            }
        };
        let msg = match done.status {
            Ok(()) => Ok(done.buffer),
            Err(e) => Err((e, done.buffer)),
        };
        if filled.try_send(msg).is_err() || fatal {
            break;
        }
    }
    queue.cancel_all();
    while queue.pending() > 0 {
        if queue.wait_next_complete(Duration::from_secs(1)).is_none() {
            warn!("b2xx rx transfers did not return after cancellation");
            break;
        }
    }
    debug!("b2xx rx driver thread exiting");
}

/// Applies `config` to the calling thread and to nusb's event thread,
/// warning about anything the OS refuses. Streaming works without any of it.
///
/// nusb reaps URBs on a background thread of its own (`nusb-events`) and
/// then wakes this one, so both are on the path from a completed transfer to
/// its replacement, and a real-time driver thread behind a descheduled event
/// thread would gain nothing. Both get the same priority and core.
fn apply_config(config: DriverConfig) {
    #[cfg(target_os = "linux")]
    {
        let mut threads = vec![("ruhd-rx-usb", 0)];
        let events = linux::tids_named("nusb-events");
        if events.is_empty() && (config.cpu.is_some() || config.rt_priority.is_some()) {
            warn!("cannot find nusb's event thread; only the rx driver thread is tuned");
        }
        threads.extend(events.into_iter().map(|tid| ("nusb-events", tid)));
        for (name, tid) in threads {
            if let Some(cpu) = config.cpu {
                match linux::pin(tid, cpu) {
                    Ok(()) => debug!(thread = name, cpu, "pinned"),
                    Err(error) => warn!(thread = name, cpu, %error, "cannot pin"),
                }
            }
            if let Some(priority) = config.rt_priority {
                match linux::fifo(tid, priority) {
                    Ok(()) => debug!(thread = name, priority, "SCHED_FIFO"),
                    Err(error) => warn!(
                        thread = name,
                        priority,
                        %error,
                        "cannot make the thread SCHED_FIFO; it needs CAP_SYS_NICE or a high \
                         enough `ulimit -r`"
                    ),
                }
            }
        }
        if config.mlock {
            match linux::mlock_current() {
                Ok(()) => debug!("process memory locked"),
                Err(error) => warn!(
                    %error,
                    "cannot lock process memory; it needs CAP_IPC_LOCK or a larger `ulimit -l`"
                ),
            }
        }
    }
    #[cfg(not(target_os = "linux"))]
    if config != DriverConfig::default() {
        warn!("rx driver thread priority, pinning and memory locking are only supported on linux");
    }
}

#[cfg(target_os = "linux")]
mod linux {
    use std::io;

    /// Thread IDs in this process whose name is `name`.
    pub(super) fn tids_named(name: &str) -> Vec<libc::pid_t> {
        let Ok(tasks) = std::fs::read_dir("/proc/self/task") else {
            return Vec::new();
        };
        tasks
            .filter_map(|task| {
                let task = task.ok()?;
                let comm = std::fs::read_to_string(task.path().join("comm")).ok()?;
                (comm.trim_end() == name)
                    .then(|| task.file_name().to_str()?.parse().ok())
                    .flatten()
            })
            .collect()
    }

    fn check(rc: libc::c_int) -> io::Result<()> {
        if rc == 0 {
            Ok(())
        } else {
            Err(io::Error::last_os_error())
        }
    }

    /// Pins thread `tid` (0 for the caller) to `cpu`.
    pub(super) fn pin(tid: libc::pid_t, cpu: usize) -> io::Result<()> {
        // SAFETY: cpu_set_t is plain data and all zeroes is the empty set;
        // `cpu` was checked against MAX_CPU when the stream opened.
        check(unsafe {
            let mut set: libc::cpu_set_t = std::mem::zeroed();
            libc::CPU_SET(cpu, &mut set);
            libc::sched_setaffinity(tid, size_of::<libc::cpu_set_t>(), &raw const set)
        })
    }

    /// Puts thread `tid` (0 for the caller) under SCHED_FIFO.
    pub(super) fn fifo(tid: libc::pid_t, priority: i32) -> io::Result<()> {
        let param = libc::sched_param {
            sched_priority: priority,
        };
        // SAFETY: plain FFI with a valid param. On Linux this applies to the
        // one thread, not the whole process.
        check(unsafe { libc::sched_setscheduler(tid, libc::SCHED_FIFO, &raw const param) })
    }

    /// `mlockall(MCL_CURRENT)`. Not MCL_FUTURE, which would make later
    /// allocations fail outright once RLIMIT_MEMLOCK is reached.
    pub(super) fn mlock_current() -> io::Result<()> {
        // SAFETY: plain FFI.
        check(unsafe { libc::mlockall(libc::MCL_CURRENT) })
    }
}

#[cfg(test)]
mod tests {
    use std::collections::VecDeque;
    use std::sync::Mutex;

    use super::*;

    /// A fake endpoint: completes submitted buffers in order, filling each
    /// with the next value of a counter, once `ready` allows it.
    #[derive(Default)]
    struct Fake {
        submitted: VecDeque<Buffer>,
        ready: Arc<Mutex<usize>>,
        next: u8,
        max_pending: Arc<Mutex<usize>>,
        cancelled: Arc<AtomicBool>,
        fail_all: bool,
    }

    impl TransferQueue for Fake {
        fn submit(&mut self, buf: Buffer) {
            self.submitted.push_back(buf);
            let mut max = self.max_pending.lock().unwrap();
            *max = (*max).max(self.submitted.len());
        }

        fn pending(&self) -> usize {
            self.submitted.len()
        }

        fn wait_next_complete(&mut self, timeout: Duration) -> Option<Completion> {
            assert!(
                !self.submitted.is_empty(),
                "nusb panics with nothing pending"
            );
            let cancelled = self.cancelled.load(Ordering::Relaxed);
            {
                let mut ready = self.ready.lock().unwrap();
                if *ready == 0 && !cancelled {
                    drop(ready);
                    std::thread::sleep(timeout.min(Duration::from_millis(1)));
                    return None;
                }
                *ready = ready.saturating_sub(1);
            }
            let mut buffer = self.submitted.pop_front().unwrap();
            buffer.clear();
            let status = if cancelled {
                Err(TransferError::Cancelled)
            } else if self.fail_all {
                Err(TransferError::Fault)
            } else {
                buffer.extend_fill(1, self.next);
                self.next = self.next.wrapping_add(1);
                Ok(())
            };
            Some(Completion {
                actual_len: buffer.len(),
                buffer,
                status,
            })
        }

        fn cancel_all(&mut self) {
            self.cancelled.store(true, Ordering::Relaxed);
        }
    }

    fn pool(n: usize) -> Vec<Buffer> {
        (0..n).map(|_| Buffer::new(64)).collect()
    }

    fn recv_ok(driver: &RxDriver) -> Buffer {
        match driver.recv(Duration::from_secs(5)) {
            Recv::Filled(Ok(buf)) => buf,
            other => panic!("expected a filled buffer, got {other:?}"),
        }
    }

    #[test]
    fn delivers_in_order_and_keeps_the_kernel_queue_at_depth() {
        let fake = Fake::default();
        let ready = Arc::clone(&fake.ready);
        let max_pending = Arc::clone(&fake.max_pending);
        let cancelled = Arc::clone(&fake.cancelled);
        let mut driver = RxDriver::start(fake, 4, pool(16), DriverConfig::default()).unwrap();

        // Ten completions with nobody reading: the thread keeps resubmitting
        // from the spare pool, so nothing is lost.
        *ready.lock().unwrap() = 10;
        let got: Vec<u8> = (0..10)
            .map(|_| {
                let buf = recv_ok(&driver);
                let value = buf[0];
                driver.release(buf);
                value
            })
            .collect();
        assert_eq!(got, (0..10).collect::<Vec<u8>>());
        assert_eq!(*max_pending.lock().unwrap(), 4, "never deeper than depth");
        assert_eq!(driver.starved(), 0);

        driver.shutdown();
        assert!(cancelled.load(Ordering::Relaxed));
        assert!(matches!(driver.recv(Duration::ZERO), Recv::Stopped));
    }

    #[test]
    fn a_full_host_buffer_is_counted_once_and_recovers() {
        let fake = Fake::default();
        let ready = Arc::clone(&fake.ready);
        // Depth 2 and 2 spares: the third and fourth completions find no
        // spare buffer.
        let driver = RxDriver::start(fake, 2, pool(4), DriverConfig::default()).unwrap();
        *ready.lock().unwrap() = 4;
        let held: Vec<Buffer> = (0..4).map(|_| recv_ok(&driver)).collect();
        assert_eq!(driver.starved(), 1);
        assert!(matches!(
            driver.recv(Duration::from_millis(30)),
            Recv::Timeout
        ));
        // Handing buffers back restarts the queue.
        for buf in held {
            driver.release(buf);
        }
        *ready.lock().unwrap() = 1;
        assert_eq!(recv_ok(&driver)[0], 4);
    }

    #[test]
    fn repeated_failures_stop_the_thread() {
        let fake = Fake {
            fail_all: true,
            ..Fake::default()
        };
        *fake.ready.lock().unwrap() = usize::MAX;
        let driver = RxDriver::start(fake, 2, pool(4), DriverConfig::default()).unwrap();
        let mut failures = 0;
        loop {
            match driver.recv(Duration::from_secs(5)) {
                Recv::Filled(Err((e, buf))) => {
                    assert_eq!(e, TransferError::Fault);
                    failures += 1;
                    driver.release(buf);
                }
                Recv::Stopped => break,
                other => panic!("unexpected {other:?}"),
            }
        }
        assert_eq!(failures, MAX_CONSECUTIVE_ERRORS);
    }
}
