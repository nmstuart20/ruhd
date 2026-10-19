//! `ruhd rx`: record IQ samples to a file, the rough equivalent of UHD's
//! `rx_samples_to_file`.
//!
//! Ctrl-C stops the stream cleanly and keeps
//! what was recorded, the recording gets a SigMF metadata file with every
//! overflow marked at the sample where it happened, and `--file -` streams to
//! stdout for piping into other tools.

use std::fs::File;
use std::io::{self, Write};
use std::path::{Path, PathBuf};
use std::sync::Arc;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::mpsc;
use std::time::{Duration, Instant, SystemTime};

use clap::{Args, ValueEnum};
use uhd_b200::RxAntenna;
use uhd_core::metadata::{RxError, RxMetadata};
use uhd_core::stream::{RawRxStreamer, StreamCmd};
use uhd_core::{
    DeviceArgs, Direction, Error, Result, SampleFormat, SensorData, SensorScope, TuneRequest,
    registry,
};

use crate::rx_backlog::{Backlog, Snapshot};
use crate::rx_latency::BlockTiming;
use crate::rx_profile::{RxConfig, RxProfile};
use crate::rx_stats::LossStats;
use crate::sigmf::{self, Event, EventKind, Recording};

/// How long the first samples may take to arrive after the stream starts.
const FIRST_TIMEOUT: Duration = Duration::from_secs(1);
/// How long a running stream may go quiet before it counts as stalled.
const RUNNING_TIMEOUT: Duration = Duration::from_millis(500);

struct FilledBlock {
    block: Vec<u8>,
    len: usize,
    timing: BlockTiming,
}

impl FilledBlock {
    fn hand_off(
        block: Vec<u8>,
        len: usize,
        first_sample: Instant,
        full: &mpsc::Sender<Self>,
    ) -> Result<()> {
        full.send(Self {
            block,
            len,
            timing: BlockTiming {
                first_sample,
                handed_off: Instant::now(),
            },
        })
        .map_err(|_| Error::Internal("the file writer stopped early".into()))
    }
}

#[derive(Debug, Args)]
pub(crate) struct RxArgs {
    /// Output file, or `-` for stdout. Samples are interleaved I/Q in host
    /// byte order with no header, the same layout `rx_samples_to_file`
    /// writes. A SigMF metadata file is written beside it; name the data file
    /// `*.sigmf-data` so SigMF tools pair the two.
    #[arg(long, short)]
    file: PathBuf,
    /// Where to write the SigMF metadata. Defaults to the data file with its
    /// extension replaced by `.sigmf-meta`; with `--file -` there is no
    /// default, and metadata is only written if this is given.
    #[arg(long, conflicts_with = "no_meta")]
    meta: Option<PathBuf>,
    /// Do not write a SigMF metadata file.
    #[arg(long)]
    no_meta: bool,
    /// RF centre frequency, in Hz.
    #[arg(long)]
    freq: f64,
    /// Sample rate, in samples per second.
    #[arg(long, default_value_t = 1e6)]
    rate: f64,
    /// RX gain, in dB. Left at the codec's default if not given.
    #[arg(long)]
    gain: Option<f64>,
    /// Analog bandwidth, in Hz. Left at the codec's default if not given.
    #[arg(long)]
    bw: Option<f64>,
    /// Receive port: RX2 or TX/RX.
    #[arg(long, default_value = "RX2", value_parser = parse_antenna)]
    ant: RxAntenna,
    /// Master clock rate, in Hz. Chosen from the sample rate the way UHD does
    /// if not given.
    #[arg(long)]
    master_clock_rate: Option<f64>,
    /// Number of samples to record.
    #[arg(long, conflicts_with = "duration")]
    nsamps: Option<u64>,
    /// Seconds to record. With neither this nor `--nsamps`, records until
    /// Ctrl-C.
    #[arg(long)]
    duration: Option<f64>,
    /// Sample type in the file.
    #[arg(long = "type", value_enum, default_value_t = FileType::Sc16)]
    file_type: FileType,
    /// Coordinated packet size, USB transfer count, application block size,
    /// and buffer pool. Defaults to low-latency for stdout, capture for files.
    #[arg(long, value_enum)]
    profile: Option<RxProfile>,
    /// Samples per packet; overrides the profile (capped by the USB frame).
    #[arg(long)]
    spp: Option<usize>,
    /// USB transfers kept queued with the kernel; overrides the profile.
    #[arg(long)]
    num_recv_frames: Option<usize>,
    /// Seconds to wait after tuning before streaming, for the LO and the
    /// analog front end to settle.
    #[arg(long, default_value_t = 0.5)]
    setup_time: f64,
    /// Only the device with this serial.
    #[arg(long)]
    serial: Option<String>,
}

impl RxArgs {
    fn config(&self) -> Result<RxConfig> {
        self.profile
            .unwrap_or_else(|| {
                if is_stdout(&self.file) {
                    RxProfile::LowLatency
                } else {
                    RxProfile::Capture
                }
            })
            .config()
            .with_overrides(self.spp, self.num_recv_frames)
    }
}

/// Sample type written to the file.
#[derive(Clone, Copy, Debug, PartialEq, Eq, ValueEnum)]
pub(crate) enum FileType {
    /// Complex 16-bit integers. UHD's `short`.
    #[value(alias = "short")]
    Sc16,
    /// Complex 32-bit floats scaled to [-1, 1). UHD's `float`.
    #[value(alias = "float")]
    Fc32,
    /// Complex 64-bit floats. UHD's `double`.
    #[value(alias = "double")]
    Fc64,
}

impl FileType {
    pub(crate) fn format(self) -> SampleFormat {
        match self {
            Self::Sc16 => SampleFormat::Sc16,
            Self::Fc32 => SampleFormat::Fc32,
            Self::Fc64 => SampleFormat::Fc64,
        }
    }
}

fn parse_antenna(s: &str) -> std::result::Result<RxAntenna, String> {
    RxAntenna::parse(s).map_err(|e| e.to_string())
}

/// What happened during a recording.
#[derive(Debug, Default)]
struct Tally {
    samples: u64,
    overflows: u64,
    gaps: u64,
    bad_packets: u64,
    time_jumps: u64,
    loss: LossStats,
    /// Every discontinuity, in the order they happened.
    events: Vec<Event>,
    /// Device time of the first sample, in seconds.
    first_device_time: Option<f64>,
    /// Host time the stream was started.
    started: Option<SystemTime>,
    /// The reader at the other end of the output went away.
    output_closed: bool,
    backlog: Snapshot,
    pool_waits: u64,
    pool_wait: Duration,
}

impl Tally {
    fn discontinuities(&self) -> u64 {
        self.events.len() as u64
    }
}

/// Whether the output is stdout rather than a file.
fn is_stdout(path: &Path) -> bool {
    path.as_os_str() == "-"
}

/// Installs a Ctrl-C handler that asks the receive loop to stop.
///
/// The first Ctrl-C stops the stream at the next block and keeps everything
/// recorded so far. A second one exits at once, for a device that has stopped
/// answering.
pub(crate) fn stop_on_ctrl_c() -> Result<Arc<AtomicBool>> {
    let stop = Arc::new(AtomicBool::new(false));
    let flag = Arc::clone(&stop);
    ctrlc::set_handler(move || {
        if flag.swap(true, Ordering::SeqCst) {
            eprintln!("\nInterrupted again; exiting without cleaning up.");
            std::process::exit(130);
        }
        eprintln!("\nStopping (Ctrl-C again to exit at once)...");
    })
    .map_err(|e| Error::Internal(format!("installing the Ctrl-C handler: {e}")))?;
    Ok(stop)
}

pub(crate) fn run(args: &RxArgs) -> Result<()> {
    // Checked before the device is touched, so a typo costs nothing.
    let config = args.config()?;
    if let Some(secs) = args.duration
        && !(secs.is_finite() && secs > 0.0)
    {
        return Err(Error::invalid_arg(format!(
            "--duration must be a positive number of seconds, not {secs}"
        )));
    }
    let meta_path = match (&args.meta, args.no_meta) {
        (_, true) => None,
        (Some(path), false) => Some(path.clone()),
        (None, false) if is_stdout(&args.file) => None,
        (None, false) => Some(sigmf::meta_path_for(&args.file)),
    };
    let stop = stop_on_ctrl_c()?;

    let mut device_args = DeviceArgs::new().with("type", "b200");
    if let Some(serial) = &args.serial {
        device_args.set("serial", serial);
    }
    if let Some(rate) = args.master_clock_rate {
        device_args.set("master_clock_rate", rate.to_string());
    }
    eprintln!("Initialising device...");
    let device = registry::open(&device_args)?;
    let info = device.info();
    let rate = device.set_sample_rate(Direction::Rx, 0, args.rate)?;
    let tick_rate = device.master_clock_rate(0)?;
    let freq = device
        .tune(Direction::Rx, 0, &TuneRequest::new(args.freq))?
        .actual_freq();
    let gain = args
        .gain
        .map(|gain| {
            device.set_gain(Direction::Rx, 0, None, gain)?;
            device.gain(Direction::Rx, 0, None)
        })
        .transpose()?;
    let bw = args
        .bw
        .map(|bw| device.set_bandwidth(Direction::Rx, 0, bw))
        .transpose()?;
    device.set_antenna(Direction::Rx, 0, args.ant.name())?;
    let antenna = device.antenna(Direction::Rx, 0)?;
    eprintln!("Using Device: {info}");

    let stream_args = config.stream_args(args.file_type.format());
    let mut rx = device.open_rx_stream(&stream_args)?;

    eprintln!("  master clock: {:.6} MHz", tick_rate / 1e6);
    eprintln!("  sample rate:  {:.6} MS/s", rate / 1e6);
    eprintln!("  frequency:    {:.6} MHz", freq / 1e6);
    if let Some(gain) = gain {
        eprintln!("  gain:         {gain:.1} dB");
    }
    if let Some(bw) = bw {
        eprintln!("  bandwidth:    {:.6} MHz", bw / 1e6);
    }
    eprintln!("  antenna:      {}", antenna);
    eprintln!(
        "  format:       {} ({} samples per packet)",
        args.file_type.format().name(),
        rx.max_num_samps()
    );
    config.report(args.file_type.format(), rate, rx.max_num_samps());

    std::thread::sleep(Duration::from_secs_f64(args.setup_time.max(0.0)));
    let locked = device.sensor(
        SensorScope::Channel {
            dir: Direction::Rx,
            chan: 0,
        },
        "lo_locked",
    )?;
    if locked.value == SensorData::Bool(false) {
        eprintln!("warning: the RX LO reports unlocked");
    }

    let total = match (args.nsamps, args.duration) {
        (Some(n), _) => Some(n),
        (None, Some(secs)) => Some((secs * rate).round() as u64),
        (None, None) => None,
    };
    if total.is_none() {
        eprintln!("Recording until Ctrl-C.");
    }

    let out: Box<dyn Write + Send> = if is_stdout(&args.file) {
        Box::new(io::stdout())
    } else {
        Box::new(File::create(&args.file)?)
    };
    let mut tally = Tally::default();
    let result = record(rx.as_mut(), out, total, &stop, &mut tally, config, rate);
    drop(rx);

    let target = if is_stdout(&args.file) {
        "stdout".to_string()
    } else {
        args.file.display().to_string()
    };
    let bytes = tally.backlog.written;
    let written_samples = bytes / (args.file_type.format().bits_per_sample() / 8) as u64;
    eprintln!(
        "Wrote {} samples ({:.1} MB) to {target}",
        written_samples,
        bytes as f64 / 1e6,
    );
    if tally.output_closed {
        eprintln!("  stopped because the reader closed the output");
    }
    if tally.discontinuities() > 0 {
        eprintln!(
            "  {} overflow(s), {} sequence gap(s), {} bad packet(s), {} timestamp jump(s); the file has \
             discontinuities",
            tally.overflows, tally.gaps, tally.bad_packets, tally.time_jumps
        );
        eprintln!(
            "  {} known missing samples, {:.9} s known gap duration; {} loss episode(s) with unknown count",
            tally.loss.missing_samples,
            tally.loss.gap_seconds,
            tally.loss.unknown_events()
        );
    }

    // Written even when the stream failed part-way, so the samples that did
    // land on disk are still described.
    if let (Some(path), Some(started)) = (meta_path, tally.started) {
        let hardware = format!("{} {}", info.product, info.serial)
            .trim()
            .to_owned();
        let recording = Recording {
            format: args.file_type.format(),
            sample_rate: rate,
            frequency: freq,
            gain,
            bandwidth: bw,
            antenna,
            master_clock_rate: tick_rate,
            hardware,
            started,
            first_device_time: tally.first_device_time,
            events: std::mem::take(&mut tally.events),
            loss: tally.loss,
        };
        let text = serde_json::to_string_pretty(&sigmf::metadata(&recording))
            .map_err(|e| Error::Internal(format!("encoding SigMF metadata: {e}")))?;
        std::fs::write(&path, text + "\n")?;
        eprintln!("  metadata: {}", path.display());
        if !is_stdout(&args.file)
            && args.meta.is_none()
            && args.file.extension().is_none_or(|ext| ext != "sigmf-data")
        {
            eprintln!(
                "  note: SigMF tools expect the data file to be named {}",
                path.with_extension("sigmf-data").display()
            );
        }
    }
    result
}

/// Streams until `total` samples are written, `stop` is set, or the reader
/// at the other end of `out` goes away.
///
/// The file is written on its own thread, so a slow write delays only that
/// thread: the receive loop keeps draining USB until every block in the pool
/// is waiting on the disk.
///
/// `tally` is filled in as the stream runs rather than returned, so a caller
/// still knows what was recorded when this fails part-way.
fn record(
    rx: &mut dyn RawRxStreamer,
    out: Box<dyn Write + Send>,
    total: Option<u64>,
    stop: &AtomicBool,
    tally: &mut Tally,
    config: RxConfig,
    rate: f64,
) -> Result<()> {
    let host_bytes = rx.host_format().bits_per_sample() / 8;
    let block_samples = config.block_samples(rx.host_format());
    let backlog = Arc::new(Backlog::default());

    // Live measurements follow the existing log filter. When enabled, keep
    // reporting even when the receive loop or writer is blocked.
    let (monitor_stop, monitor_rx) = mpsc::channel::<()>();
    let monitor = tracing::enabled!(tracing::Level::INFO).then(|| {
        let monitor_backlog = Arc::clone(&backlog);
        std::thread::spawn(move || {
            while matches!(
                monitor_rx.recv_timeout(Duration::from_secs(1)),
                Err(mpsc::RecvTimeoutError::Timeout)
            ) {
                monitor_backlog.snapshot().report(host_bytes, rate);
            }
        })
    });

    let (full_tx, full_rx) = mpsc::channel::<FilledBlock>();
    let (empty_tx, empty_rx) = mpsc::channel::<Vec<u8>>();
    for _ in 0..config.blocks {
        empty_tx
            .send(vec![0u8; block_samples * host_bytes])
            .expect("block pool receiver is still open");
    }
    let writer_backlog = Arc::clone(&backlog);
    let writer = std::thread::spawn(move || -> io::Result<()> {
        let mut out = out;
        for FilledBlock { block, len, timing } in full_rx {
            let started = Instant::now();
            let result = out.write_all(&block[..len]);
            let finished = Instant::now();
            if let Err(error) = result {
                writer_backlog.failed_write();
                return Err(error);
            }
            writer_backlog.completed(timing, started, finished);
            writer_backlog.written(len);
            // The receive loop may already have finished; a closed pool is fine.
            empty_tx.send(block).ok();
        }
        out.flush()
    });

    let result = stream_blocks(
        rx,
        total,
        stop,
        block_samples,
        host_bytes,
        &empty_rx,
        &full_tx,
        tally,
        &backlog,
    );
    drop(full_tx);
    let written = writer.join();
    drop(monitor_stop);
    if let Some(monitor) = monitor {
        let _ = monitor.join();
    }
    tally.backlog = backlog.snapshot();
    tally.backlog.report(host_bytes, rate);
    eprintln!(
        "  buffer pool exhausted {} time(s), {:.3} ms waiting for output",
        tally.pool_waits,
        tally.pool_wait.as_secs_f64() * 1000.0,
    );
    let written = written.map_err(|_| Error::Internal("the file writer thread panicked".into()))?;
    match written {
        // A pipe whose reader exits (`ruhd rx -f - | head -c 1M`) is a normal
        // way for a recording to end, not a failure. The receive loop will
        // have stopped on the closed pool; that error is the same event.
        Err(e) if e.kind() == io::ErrorKind::BrokenPipe => {
            tally.output_closed = true;
            Ok(())
        }
        Err(e) => {
            result?;
            Err(e.into())
        }
        Ok(()) => result,
    }
}

/// The receive half of [`record`]: starts the stream, runs it, and stops it
/// again however the run ended.
#[allow(clippy::too_many_arguments)]
fn stream_blocks(
    rx: &mut dyn RawRxStreamer,
    total: Option<u64>,
    stop: &AtomicBool,
    block_samples: usize,
    host_bytes: usize,
    empty: &mpsc::Receiver<Vec<u8>>,
    full: &mpsc::Sender<FilledBlock>,
    tally: &mut Tally,
    backlog: &Backlog,
) -> Result<()> {
    if stop.load(Ordering::SeqCst) {
        return Ok(());
    }
    rx.issue_stream_cmd(&StreamCmd::start_continuous())?;
    tally.started = Some(SystemTime::now());
    let started = Instant::now();

    let result = pump(
        rx,
        total,
        stop,
        block_samples,
        host_bytes,
        empty,
        full,
        tally,
        backlog,
    );
    let stopped = rx.issue_stream_cmd(&StreamCmd::stop());

    let elapsed = started.elapsed().as_secs_f64();
    if tally.overflows + tally.gaps > 0 {
        eprintln!();
    }
    eprintln!(
        "  streamed for {elapsed:.2} s ({:.3} MS/s average)",
        tally.samples as f64 / elapsed / 1e6
    );
    result?;
    stopped
}

/// Fills blocks from the pool and hands them to the writer.
#[allow(clippy::too_many_arguments)]
fn pump(
    rx: &mut dyn RawRxStreamer,
    total: Option<u64>,
    stop: &AtomicBool,
    block_samples: usize,
    host_bytes: usize,
    empty: &mpsc::Receiver<Vec<u8>>,
    full: &mpsc::Sender<FilledBlock>,
    tally: &mut Tally,
    backlog: &Backlog,
) -> Result<()> {
    let mut md = RxMetadata::default();
    let closed = || Error::Internal("the file writer stopped early".into());
    let remaining = |tally: &Tally| total.map_or(u64::MAX, |t| t - tally.samples);

    let mut timeout = FIRST_TIMEOUT;
    let mut first = true;
    let mut block = empty.recv().map_err(|_| closed())?;
    let mut filled = 0usize;
    let mut first_sample = None;

    let result = loop {
        if remaining(tally) == 0 || stop.load(Ordering::SeqCst) {
            break Ok(());
        }
        let room =
            (block_samples - filled).min(usize::try_from(remaining(tally)).unwrap_or(usize::MAX));
        let n = match rx.recv_raw_with_progress(
            &mut [&mut block[filled * host_bytes..(filled + room) * host_bytes]],
            timeout,
            &mut md,
            &mut |n, observed| {
                first_sample.get_or_insert(observed);
                backlog.received(n * host_bytes);
            },
        ) {
            Ok(n) => n,
            Err(e) => break Err(e),
        };
        tally.loss.observe(n, &md);
        let kind = match md.error {
            None => None,
            Some(RxError::Overflow) if md.out_of_sequence => {
                tally.gaps += 1;
                eprint!("D");
                Some(EventKind::SequenceGap)
            }
            Some(RxError::Overflow) => {
                tally.overflows += 1;
                eprint!("O");
                Some(EventKind::Overflow)
            }
            Some(RxError::BadPacket) => {
                tally.bad_packets += 1;
                Some(EventKind::BadPacket)
            }
            Some(RxError::TimestampJump) => {
                tally.time_jumps += 1;
                Some(EventKind::TimestampJump)
            }
            Some(RxError::Timeout) => break Err(Error::Timeout(timeout)),
            Some(other) => {
                break Err(Error::Device(format!("receive stream error: {other:?}")));
            }
        };
        if let Some(kind) = kind {
            // The discontinuity is before this call's samples, so it falls at
            // the count written so far.
            tally.events.push(Event {
                sample: tally.samples,
                kind,
                device_time: md.time_spec.map(|t| t.as_secs_f64()),
                loss: None,
            });
        }
        if n > 0
            && let Some(loss) = md.loss
        {
            // The preceding notification(s) and this recovery describe the
            // same boundary in the output file. Attach accounting once.
            if let Some(event) = tally
                .events
                .last_mut()
                .filter(|e| e.sample == tally.samples)
            {
                event.loss = Some(loss);
            } else {
                tally.events.push(Event {
                    sample: tally.samples,
                    kind: EventKind::from_loss(loss.cause),
                    device_time: md.time_spec.map(|t| t.as_secs_f64()),
                    loss: Some(loss),
                });
            }
        }
        if n > 0 && first {
            first = false;
            timeout = RUNNING_TIMEOUT;
            if let Some(time) = md.time_spec {
                tally.first_device_time = Some(time.as_secs_f64());
                eprintln!("  first sample: device time {:.9} s", time.as_secs_f64());
            }
        }
        filled += n;
        tally.samples += n as u64;
        if filled == block_samples {
            FilledBlock::hand_off(
                block,
                filled * host_bytes,
                first_sample
                    .take()
                    .expect("received samples have a host timestamp"),
                full,
            )?;
            filled = 0;
            // Do not wait on a slow writer before checking a finite limit or
            // Ctrl-C. Polling also makes Ctrl-C responsive under backpressure.
            if remaining(tally) == 0 || stop.load(Ordering::SeqCst) {
                return Ok(());
            }
            block = match empty.try_recv() {
                Ok(block) => block,
                Err(mpsc::TryRecvError::Disconnected) => return Err(closed()),
                Err(mpsc::TryRecvError::Empty) => {
                    tally.pool_waits += 1;
                    let waiting = Instant::now();
                    let next = loop {
                        if stop.load(Ordering::SeqCst) {
                            break Ok(None);
                        }
                        match empty.recv_timeout(Duration::from_millis(50)) {
                            Ok(block) => break Ok(Some(block)),
                            Err(mpsc::RecvTimeoutError::Timeout) => continue,
                            Err(mpsc::RecvTimeoutError::Disconnected) => break Err(closed()),
                        }
                    };
                    tally.pool_wait += waiting.elapsed();
                    match next? {
                        Some(block) => block,
                        None => return Ok(()),
                    }
                }
            };
        }
    };
    // Keep samples from earlier receives even when the next receive fails.
    // The writer drains this block before record() returns the stream error.
    if filled > 0 {
        FilledBlock::hand_off(
            block,
            filled * host_bytes,
            first_sample.expect("received samples have a host timestamp"),
            full,
        )?;
    }
    result
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::test_support::ScriptedRx;
    use clap::Parser;
    use std::sync::Mutex;
    use uhd_core::{RxLoss, RxLossCause};

    const BLOCK_BYTES: usize = 2 << 20;

    fn record(
        rx: &mut dyn RawRxStreamer,
        out: Box<dyn Write + Send>,
        total: Option<u64>,
        stop: &AtomicBool,
        tally: &mut Tally,
    ) -> Result<()> {
        super::record(
            rx,
            out,
            total,
            stop,
            tally,
            RxProfile::Capture.config(),
            1e6,
        )
    }

    fn parse_rx(extra: &[&str]) -> RxArgs {
        let cli = crate::Cli::try_parse_from(
            ["ruhd", "rx", "--freq", "100e6"]
                .into_iter()
                .chain(extra.iter().copied()),
        )
        .unwrap();
        let crate::Command::Rx(args) = cli.command else {
            unreachable!()
        };
        args
    }

    #[test]
    fn profiles_follow_output_target_and_explicit_settings_win() {
        assert_eq!(
            parse_rx(&["-f", "-"]).config().unwrap().profile,
            RxProfile::LowLatency
        );
        assert_eq!(
            parse_rx(&["-f", "capture.sc16"]).config().unwrap().profile,
            RxProfile::Capture
        );
        let config = parse_rx(&[
            "-f",
            "-",
            "--profile",
            "balanced",
            "--spp",
            "99",
            "--num-recv-frames",
            "7",
        ])
        .config()
        .unwrap();
        assert_eq!(config.profile, RxProfile::Balanced);
        assert_eq!(config.spp, 99);
        assert_eq!(config.transfers, 7);
        assert_eq!(config.block_bytes * config.blocks, 2 << 20);
        assert_eq!(
            parse_rx(&["-f", "out", "--profile", "low-latency"])
                .config()
                .unwrap()
                .profile,
            RxProfile::LowLatency
        );
        assert!(parse_rx(&["-f", "-", "--spp", "0"]).config().is_err());
        assert!(
            parse_rx(&["-f", "-", "--num-recv-frames", "1"])
                .config()
                .is_err()
        );
    }

    #[derive(Clone, Default)]
    struct Capture {
        bytes: Arc<Mutex<Vec<u8>>>,
        writes: Arc<Mutex<Vec<usize>>>,
        flushed: Arc<AtomicBool>,
    }

    impl Write for Capture {
        fn write(&mut self, bytes: &[u8]) -> io::Result<usize> {
            self.writes.lock().unwrap().push(bytes.len());
            self.bytes.lock().unwrap().extend_from_slice(bytes);
            Ok(bytes.len())
        }

        fn flush(&mut self) -> io::Result<()> {
            self.flushed.store(true, Ordering::SeqCst);
            Ok(())
        }
    }

    #[derive(Debug)]
    struct ProgressRx {
        inner: ScriptedRx,
        first_sample: Instant,
        gate: Option<(mpsc::Sender<()>, mpsc::Receiver<()>)>,
    }

    impl RawRxStreamer for ProgressRx {
        fn num_channels(&self) -> usize {
            self.inner.num_channels()
        }
        fn max_num_samps(&self) -> usize {
            self.inner.max_num_samps()
        }
        fn host_format(&self) -> SampleFormat {
            self.inner.host_format()
        }
        fn wire_format(&self) -> SampleFormat {
            self.inner.wire_format()
        }
        fn issue_stream_cmd(&mut self, cmd: &StreamCmd) -> Result<()> {
            self.inner.issue_stream_cmd(cmd)
        }
        fn recv_raw(
            &mut self,
            buffs: &mut [&mut [u8]],
            timeout: Duration,
            md: &mut RxMetadata,
        ) -> Result<usize> {
            self.inner.recv_raw(buffs, timeout, md)
        }
        fn recv_raw_with_progress(
            &mut self,
            buffs: &mut [&mut [u8]],
            timeout: Duration,
            md: &mut RxMetadata,
            progress: &mut dyn FnMut(usize, Instant),
        ) -> Result<usize> {
            let n = self.inner.recv_raw(buffs, timeout, md)?;
            if n > 0 {
                progress(
                    n,
                    if self.gate.is_some() {
                        self.first_sample
                    } else {
                        Instant::now()
                    },
                );
                if let Some((copied, release)) = self.gate.take() {
                    copied.send(()).unwrap();
                    release.recv_timeout(Duration::from_secs(2)).unwrap();
                }
            }
            Ok(n)
        }
    }

    #[test]
    fn progress_exposes_backlog_inside_receive_and_keeps_first_observation_for_partial_block() {
        let first_sample = Instant::now() - Duration::from_millis(50);
        let (copied_tx, copied_rx) = mpsc::channel();
        let (release_tx, release_rx) = mpsc::channel();
        let mut rx = ProgressRx {
            inner: ScriptedRx::new([
                Ok((vec![1; 8], RxMetadata::default())),
                Ok((
                    Vec::new(),
                    RxMetadata {
                        error: Some(RxError::Overflow),
                        ..Default::default()
                    },
                )),
                Ok((vec![2; 4], RxMetadata::default())),
            ]),
            first_sample,
            gate: Some((copied_tx, release_rx)),
        };
        let backlog = Backlog::default();
        let stop = AtomicBool::new(false);
        let mut tally = Tally::default();
        let (empty_tx, empty_rx) = mpsc::channel();
        let (full_tx, full_rx) = mpsc::channel();
        empty_tx.send(vec![0; 16 << 10]).unwrap();
        std::thread::scope(|scope| {
            let backlog = &backlog;
            let tally = &mut tally;
            let rx = &mut rx;
            let stop = &stop;
            let pump = scope.spawn(move || {
                stream_blocks(
                    rx,
                    Some(3),
                    stop,
                    (16 << 10) / 4,
                    4,
                    &empty_rx,
                    &full_tx,
                    tally,
                    backlog,
                )
            });
            copied_rx.recv_timeout(Duration::from_secs(2)).unwrap();
            assert_eq!(backlog.snapshot().pending, 8);
            assert!(matches!(full_rx.try_recv(), Err(mpsc::TryRecvError::Empty)));
            release_tx.send(()).unwrap();
            pump.join().unwrap().unwrap();
        });
        let full = full_rx.recv_timeout(Duration::from_secs(2)).unwrap();
        assert_eq!(full.len, 12);
        assert_eq!(full.timing.first_sample, first_sample);
        assert!(full.timing.handed_off.duration_since(first_sample) >= Duration::from_millis(50));
        assert_eq!(&full.block[..12], &[vec![1; 8], vec![2; 4]].concat());
        assert_eq!(backlog.snapshot().pending, 12);
        assert_eq!(tally.overflows, 1);
        assert_eq!(
            rx.inner.commands,
            [StreamCmd::start_continuous(), StreamCmd::stop()]
        );
    }

    #[test]
    fn each_profile_delivers_its_block_size_and_flushes_the_tail() {
        for profile in [
            RxProfile::LowLatency,
            RxProfile::Balanced,
            RxProfile::Capture,
        ] {
            let config = profile.config();
            let bytes = config.block_bytes;
            let mut rx = ScriptedRx::new([
                Ok((vec![1; bytes / 2], RxMetadata::default())),
                Ok((vec![2; bytes / 2], RxMetadata::default())),
                Ok((vec![3; bytes], RxMetadata::default())),
                Ok((vec![4; 12], RxMetadata::default())),
            ]);
            let capture = Capture::default();
            let mut tally = Tally::default();
            super::record(
                &mut rx,
                Box::new(capture.clone()),
                Some((bytes as u64 * 2 + 12) / 4),
                &AtomicBool::new(false),
                &mut tally,
                config,
                1e6,
            )
            .unwrap();
            assert_eq!(*capture.writes.lock().unwrap(), [bytes, bytes, 12]);
            assert_eq!(
                *capture.bytes.lock().unwrap(),
                [
                    vec![1; bytes / 2],
                    vec![2; bytes / 2],
                    vec![3; bytes],
                    vec![4; 12]
                ]
                .concat()
            );
            assert_eq!(tally.backlog.pending, 0);
            assert_eq!(tally.backlog.written, bytes as u64 * 2 + 12);
            assert_eq!(tally.backlog.latency.total.blocks, 3);
            assert_eq!(tally.backlog.latency.fill.blocks, 3);
            assert_eq!(tally.backlog.latency.queue.blocks, 3);
            assert_eq!(tally.backlog.latency.write.blocks, 3);
            assert!(tally.backlog.peak <= bytes * config.blocks);
        }
    }

    #[test]
    fn stalled_writer_exposes_full_backlog_and_stop_does_not_need_a_free_block() {
        let config = RxProfile::LowLatency.config();
        let mut rx = ScriptedRx::new(
            (0..config.blocks).map(|_| Ok((vec![7; config.block_bytes], RxMetadata::default()))),
        );
        let backlog = Backlog::default();
        let stop = AtomicBool::new(false);
        let mut tally = Tally::default();
        let (empty_tx, empty_rx) = mpsc::channel();
        let (full_tx, full_rx) = mpsc::channel();
        for _ in 0..config.blocks {
            empty_tx.send(vec![0; config.block_bytes]).unwrap();
        }
        std::thread::scope(|scope| {
            let rx = &mut rx;
            let tally = &mut tally;
            let stop = &stop;
            let backlog = &backlog;
            let pump = scope.spawn(move || {
                stream_blocks(
                    rx,
                    None,
                    stop,
                    config.block_bytes / 4,
                    4,
                    &empty_rx,
                    &full_tx,
                    tally,
                    backlog,
                )
            });
            // Deliberately hold all blocks without confirming any writes.
            for _ in 0..config.blocks {
                let full = full_rx.recv_timeout(Duration::from_secs(2)).unwrap();
                assert_eq!(full.len, config.block_bytes);
            }
            assert_eq!(
                backlog.snapshot().pending,
                config.block_bytes * config.blocks
            );
            assert_eq!(backlog.snapshot().peak, config.block_bytes * config.blocks);
            assert_eq!(backlog.snapshot().written, 0);
            stop.store(true, Ordering::SeqCst);
            pump.join().unwrap().unwrap();
        });
        assert_eq!(
            tally.samples as usize,
            config.block_bytes * config.blocks / 4
        );
        assert_eq!(
            rx.commands,
            [StreamCmd::start_continuous(), StreamCmd::stop()]
        );
        backlog.written(config.block_bytes);
        assert_eq!(
            backlog.snapshot().pending,
            config.block_bytes * (config.blocks - 1)
        );
        assert_eq!(backlog.snapshot().peak, config.block_bytes * config.blocks);
    }

    #[test]
    fn broken_pipe_retains_unwritten_backlog_and_stops_cleanly() {
        struct Closed;
        impl Write for Closed {
            fn write(&mut self, _: &[u8]) -> io::Result<usize> {
                Err(io::ErrorKind::BrokenPipe.into())
            }
            fn flush(&mut self) -> io::Result<()> {
                Ok(())
            }
        }
        let mut rx = ScriptedRx::new([Ok((vec![1; 12], RxMetadata::default()))]);
        let mut tally = Tally::default();
        record(
            &mut rx,
            Box::new(Closed),
            Some(3),
            &AtomicBool::new(false),
            &mut tally,
        )
        .unwrap();
        assert!(tally.output_closed);
        assert_eq!(tally.backlog.pending, 12);
        assert_eq!(tally.backlog.written, 0);
        assert_eq!(tally.backlog.latency.total.blocks, 0);
        assert_eq!(tally.backlog.latency.failed_writes, 1);
        assert_eq!(
            rx.commands,
            [StreamCmd::start_continuous(), StreamCmd::stop()]
        );
    }

    #[test]
    fn output_stall_is_included_in_write_and_total_latency() {
        struct GatedOutput {
            capture: Capture,
            started: mpsc::Sender<()>,
            release: mpsc::Receiver<()>,
        }
        impl Write for GatedOutput {
            fn write(&mut self, bytes: &[u8]) -> io::Result<usize> {
                self.started.send(()).unwrap();
                self.release.recv_timeout(Duration::from_secs(2)).unwrap();
                self.capture.write(bytes)
            }
            fn flush(&mut self) -> io::Result<()> {
                self.capture.flush()
            }
        }
        let (started_tx, started_rx) = mpsc::channel();
        let (release_tx, release_rx) = mpsc::channel();
        let capture = Capture::default();
        let out = GatedOutput {
            capture: capture.clone(),
            started: started_tx,
            release: release_rx,
        };
        let mut rx = ScriptedRx::new([Ok((vec![42; 12], RxMetadata::default()))]);
        let mut tally = Tally::default();
        std::thread::scope(|scope| {
            let recording = scope.spawn(|| {
                record(
                    &mut rx,
                    Box::new(out),
                    Some(3),
                    &AtomicBool::new(false),
                    &mut tally,
                )
            });
            started_rx.recv_timeout(Duration::from_secs(2)).unwrap();
            std::thread::sleep(Duration::from_millis(10));
            release_tx.send(()).unwrap();
            recording.join().unwrap().unwrap();
        });
        let latency = tally.backlog.latency;
        assert_eq!(latency.total.blocks, 1);
        assert!(latency.write.max >= Duration::from_millis(10));
        assert!(latency.total.max >= latency.write.max);
        assert_eq!(tally.backlog.pending, 0);
        assert_eq!(*capture.bytes.lock().unwrap(), vec![42; 12]);
    }

    #[test]
    fn recovered_loss_updates_the_annotation_without_duplicating_it() {
        let loss = RxLoss::new(RxLossCause::Overflow, Some(108), Some(148), 1e6, 4);
        let mut rx = ScriptedRx::new([
            Ok((vec![1; 8], RxMetadata::default())),
            Ok((
                Vec::new(),
                RxMetadata {
                    error: Some(RxError::Overflow),
                    ..Default::default()
                },
            )),
            Ok((
                vec![2; 8],
                RxMetadata {
                    loss: Some(loss),
                    ..Default::default()
                },
            )),
        ]);
        let capture = Capture::default();
        let mut tally = Tally::default();
        record(
            &mut rx,
            Box::new(capture.clone()),
            Some(4),
            &AtomicBool::new(false),
            &mut tally,
        )
        .unwrap();
        assert_eq!(
            *capture.bytes.lock().unwrap(),
            [vec![1; 8], vec![2; 8]].concat()
        );
        assert_eq!(tally.events.len(), 1);
        assert_eq!(tally.events[0].sample, 2);
        assert_eq!(tally.events[0].loss, Some(loss));
        assert_eq!(tally.loss.missing_samples, 10);
        assert_eq!(tally.loss.unknown_events(), 0);
    }

    #[test]
    fn receive_errors_flush_the_partial_block_and_keep_annotation_positions() {
        let failures = [
            (
                Err(Error::transport("device disconnected")),
                Error::transport("device disconnected"),
            ),
            (
                Ok((
                    Vec::new(),
                    RxMetadata {
                        error: Some(RxError::Timeout),
                        ..Default::default()
                    },
                )),
                Error::Timeout(RUNNING_TIMEOUT),
            ),
            (
                Ok((
                    Vec::new(),
                    RxMetadata {
                        error: Some(RxError::LateCommand),
                        ..Default::default()
                    },
                )),
                Error::device("receive stream error: LateCommand"),
            ),
        ];
        for (failure, expected_error) in failures {
            let first_time = uhd_core::TimeSpec::from_secs(1.0);
            let expected = [vec![11; BLOCK_BYTES], vec![22; 8]].concat();
            let mut rx = ScriptedRx::new([
                Ok((
                    expected[..BLOCK_BYTES].to_vec(),
                    RxMetadata {
                        time_spec: Some(first_time),
                        ..Default::default()
                    },
                )),
                Ok((
                    expected[BLOCK_BYTES..].to_vec(),
                    RxMetadata {
                        error: Some(RxError::Overflow),
                        ..Default::default()
                    },
                )),
                failure,
            ]);
            let capture = Capture::default();
            let mut tally = Tally::default();
            let result = record(
                &mut rx,
                Box::new(capture.clone()),
                None,
                &AtomicBool::new(false),
                &mut tally,
            );

            assert_eq!(result.unwrap_err().to_string(), expected_error.to_string());
            assert_eq!(*capture.bytes.lock().unwrap(), expected);
            assert!(capture.flushed.load(Ordering::SeqCst));
            assert_eq!(tally.samples as usize * 4, expected.len());
            assert_eq!(tally.first_device_time, Some(first_time.as_secs_f64()));
            assert!(tally.started.is_some());
            assert_eq!(tally.overflows, 1);
            assert_eq!(
                tally.events,
                [Event {
                    sample: (BLOCK_BYTES / 4) as u64,
                    kind: EventKind::Overflow,
                    device_time: None,
                    loss: None,
                }]
            );
            assert_eq!(
                rx.commands,
                [StreamCmd::start_continuous(), StreamCmd::stop()]
            );
        }
    }

    #[test]
    fn finite_recording_flushes_the_partial_block() {
        let expected = vec![42; 12];
        let mut rx = ScriptedRx::new([Ok((expected.clone(), RxMetadata::default()))]);
        let capture = Capture::default();
        let mut tally = Tally::default();
        record(
            &mut rx,
            Box::new(capture.clone()),
            Some(3),
            &AtomicBool::new(false),
            &mut tally,
        )
        .unwrap();

        assert_eq!(*capture.bytes.lock().unwrap(), expected);
        assert!(capture.flushed.load(Ordering::SeqCst));
        assert_eq!(tally.samples, 3);
        assert_eq!(tally.backlog.pending, 0);
        assert_eq!(tally.backlog.peak, 12);
        assert_eq!(tally.backlog.written, 12);
        assert_eq!(tally.backlog.latency.total.blocks, 1);
        assert_eq!(tally.backlog.latency.failed_writes, 0);
        assert!(tally.events.is_empty());
        assert_eq!(
            rx.commands,
            [StreamCmd::start_continuous(), StreamCmd::stop()]
        );
    }
}
