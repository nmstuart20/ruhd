//! `ruhd rx`: record IQ samples to a file, the rough equivalent of UHD's
//! `rx_samples_to_file`.
//!
//! Beyond what UHD's tool does: Ctrl-C stops the stream cleanly and keeps
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
use uhd_core::stream::{RawRxStreamer, StreamArgs, StreamCmd};
use uhd_core::{
    DeviceArgs, Direction, Error, Result, SampleFormat, SensorData, SensorScope, TuneRequest,
    registry,
};

use crate::sigmf::{self, Event, EventKind, Recording};

/// How long the first samples may take to arrive after the stream starts.
const FIRST_TIMEOUT: Duration = Duration::from_secs(1);
/// How long a running stream may go quiet before it counts as stalled.
const RUNNING_TIMEOUT: Duration = Duration::from_millis(500);
/// Bytes per block handed to the writer thread.
const BLOCK_BYTES: usize = 2 << 20;
/// Blocks in flight between the receive loop and the writer: 64 MiB, about a
/// quarter of a second at the AD936x's full rate, for the disk to stall in
/// before the receive loop has to wait for it.
const BLOCKS: usize = 32;

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
    /// Samples per packet. Defaults to as many as fit in a USB frame.
    #[arg(long)]
    spp: Option<usize>,
    /// USB transfers kept queued with the kernel. More gives the host longer
    /// to fall behind before the FPGA's buffer overflows.
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
    /// Every discontinuity, in the order they happened.
    events: Vec<Event>,
    /// Device time of the first sample, in seconds.
    first_device_time: Option<f64>,
    /// Host time the stream was started.
    started: Option<SystemTime>,
    /// The reader at the other end of the output went away.
    output_closed: bool,
}

impl Tally {
    fn discontinuities(&self) -> u64 {
        self.overflows + self.gaps + self.bad_packets
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
fn stop_on_ctrl_c() -> Result<Arc<AtomicBool>> {
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

    let mut stream_args = StreamArgs::new();
    stream_args.host_format = args.file_type.format();
    if let Some(spp) = args.spp {
        stream_args = stream_args.with_arg("spp", spp.to_string());
    }
    if let Some(frames) = args.num_recv_frames {
        stream_args = stream_args.with_arg("num_recv_frames", frames.to_string());
    }
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
    let result = record(rx.as_mut(), out, total, &stop, &mut tally);
    drop(rx);

    let target = if is_stdout(&args.file) {
        "stdout".to_string()
    } else {
        args.file.display().to_string()
    };
    let bytes = tally.samples * (args.file_type.format().bits_per_sample() / 8) as u64;
    eprintln!(
        "Wrote {} samples ({:.1} MB) to {target}",
        tally.samples,
        bytes as f64 / 1e6,
    );
    if tally.output_closed {
        eprintln!("  stopped because the reader closed the output");
    }
    if tally.discontinuities() > 0 {
        eprintln!(
            "  {} overflow(s), {} sequence gap(s), {} bad packet(s); the file has \
             discontinuities",
            tally.overflows, tally.gaps, tally.bad_packets
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
) -> Result<()> {
    let host_bytes = rx.host_format().bits_per_sample() / 8;
    let block_samples = BLOCK_BYTES / host_bytes;

    let (full_tx, full_rx) = mpsc::channel::<(Vec<u8>, usize)>();
    let (empty_tx, empty_rx) = mpsc::channel::<Vec<u8>>();
    for _ in 0..BLOCKS {
        empty_tx
            .send(vec![0u8; block_samples * host_bytes])
            .map_err(|_| Error::Internal("block pool closed".into()))?;
    }
    let writer = std::thread::spawn(move || -> io::Result<()> {
        let mut out = out;
        for (block, len) in full_rx {
            out.write_all(&block[..len])?;
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
    );
    drop(full_tx);
    let written = writer
        .join()
        .map_err(|_| Error::Internal("the file writer thread panicked".into()))?;
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
    full: &mpsc::Sender<(Vec<u8>, usize)>,
    tally: &mut Tally,
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
    full: &mpsc::Sender<(Vec<u8>, usize)>,
    tally: &mut Tally,
) -> Result<()> {
    let mut md = RxMetadata::default();
    let closed = || Error::Internal("the file writer stopped early".into());
    let remaining = |tally: &Tally| total.map_or(u64::MAX, |t| t - tally.samples);

    let mut timeout = FIRST_TIMEOUT;
    let mut first = true;
    let mut block = empty.recv().map_err(|_| closed())?;
    let mut filled = 0usize;

    while remaining(tally) > 0 && !stop.load(Ordering::SeqCst) {
        let room =
            (block_samples - filled).min(usize::try_from(remaining(tally)).unwrap_or(usize::MAX));
        let n = rx.recv_raw(
            &mut [&mut block[filled * host_bytes..(filled + room) * host_bytes]],
            timeout,
            &mut md,
        )?;
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
            Some(RxError::Timeout) => return Err(Error::Timeout(timeout)),
            Some(other) => {
                return Err(Error::Device(format!("receive stream error: {other:?}")));
            }
        };
        if let Some(kind) = kind {
            // The discontinuity is before this call's samples, so it falls at
            // the count written so far.
            tally.events.push(Event {
                sample: tally.samples,
                kind,
                device_time: md.time_spec.map(|t| t.as_secs_f64()),
            });
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
            full.send((block, filled * host_bytes))
                .map_err(|_| closed())?;
            filled = 0;
            block = empty.recv().map_err(|_| closed())?;
        }
    }
    if filled > 0 {
        full.send((block, filled * host_bytes))
            .map_err(|_| closed())?;
    }
    Ok(())
}
