//! `ruhd rx`: record IQ samples to a file, the rough equivalent of UHD's
//! `rx_samples_to_file`.

use std::fs::File;
use std::io::Write;
use std::path::PathBuf;
use std::sync::mpsc;
use std::time::{Duration, Instant};

use ad936x::{Chain, Direction};
use clap::{Args, ValueEnum};
use uhd_b200::{OpenOptions, RxAntenna, RxOptions};
use uhd_core::metadata::{RxError, RxMetadata};
use uhd_core::stream::{RawRxStreamer, StreamArgs, StreamCmd};
use uhd_core::{Error, Result, SampleFormat};

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
    /// Output file. Samples are interleaved I/Q in host byte order with no
    /// header, the same layout `rx_samples_to_file` writes.
    #[arg(long, short)]
    file: PathBuf,
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
    /// Seconds to record.
    #[arg(long, required_unless_present = "nsamps")]
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
enum FileType {
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
    fn format(self) -> SampleFormat {
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
}

pub(crate) fn run(args: &RxArgs) -> Result<()> {
    let mut open_options = OpenOptions::new();
    if let Some(serial) = &args.serial {
        open_options = open_options.with_serial(serial);
    }
    let session = uhd_b200::open(&open_options)?;

    let master_clock_rate = match args.master_clock_rate {
        Some(rate) => rate,
        None => uhd_b200::auto_master_clock_rate(args.rate, 1)?,
    };
    eprintln!(
        "Initialising {} ({}) at a {:.6} MHz master clock...",
        session.product(),
        session.endpoint(),
        master_clock_rate / 1e6
    );
    let tick_rate = session.init_radio(master_clock_rate)?;

    // Radio 0 is the codec's second chain when the board swaps its front ends.
    let chain = if session.misc().swap_atr() {
        Chain::Two
    } else {
        Chain::One
    };
    let codec = session.codec();
    let freq = codec.tune(Direction::Rx, args.freq)?;
    let gain = args
        .gain
        .map(|gain| codec.set_gain(Direction::Rx, chain, gain))
        .transpose()?;
    let bw = args
        .bw
        .map(|bw| codec.set_bandwidth(Direction::Rx, bw))
        .transpose()?;

    let mut stream_args = StreamArgs::new();
    stream_args.host_format = args.file_type.format();
    if let Some(spp) = args.spp {
        stream_args = stream_args.with_arg("spp", spp.to_string());
    }
    if let Some(frames) = args.num_recv_frames {
        stream_args = stream_args.with_arg("num_recv_frames", frames.to_string());
    }
    let mut rx = session.rx_stream(
        &RxOptions::new(args.rate).with_antenna(args.ant),
        &stream_args,
    )?;
    let rate = rx.sample_rate();

    eprintln!("  master clock: {:.6} MHz", tick_rate / 1e6);
    eprintln!("  sample rate:  {:.6} MS/s", rate / 1e6);
    eprintln!("  frequency:    {:.6} MHz", freq / 1e6);
    if let Some(gain) = gain {
        eprintln!("  gain:         {gain:.1} dB");
    }
    if let Some(bw) = bw {
        eprintln!("  bandwidth:    {:.6} MHz", bw / 1e6);
    }
    eprintln!("  antenna:      {}", rx.antenna());
    eprintln!(
        "  format:       {} ({} samples per packet)",
        args.file_type.format().name(),
        rx.samples_per_packet()
    );

    std::thread::sleep(Duration::from_secs_f64(args.setup_time.max(0.0)));
    let (_, rx_locked) = session.misc().fe_pll_locked()?;
    if !rx_locked {
        eprintln!("warning: the RX LO reports unlocked");
    }

    let total = match (args.nsamps, args.duration) {
        (Some(n), _) => n,
        (None, Some(secs)) if secs.is_finite() && secs > 0.0 => (secs * rate).round() as u64,
        (None, duration) => {
            return Err(Error::invalid_arg(format!(
                "--duration must be a positive number of seconds, not {duration:?}"
            )));
        }
    };

    let file = File::create(&args.file)?;
    let tally = record(&mut rx, file, total)?;
    drop(rx);

    let bytes = tally.samples * (args.file_type.format().bits_per_sample() / 8) as u64;
    eprintln!(
        "Wrote {} samples ({:.1} MB) to {}",
        tally.samples,
        bytes as f64 / 1e6,
        args.file.display()
    );
    if tally.overflows + tally.gaps + tally.bad_packets > 0 {
        eprintln!(
            "  {} overflow(s), {} sequence gap(s), {} bad packet(s); the file has \
             discontinuities",
            tally.overflows, tally.gaps, tally.bad_packets
        );
    }
    Ok(())
}

/// Streams until `total` samples are written.
///
/// The file is written on its own thread, so a slow write delays only that
/// thread: the receive loop keeps draining USB until every block in the pool
/// is waiting on the disk.
fn record(
    rx: &mut dyn RawRxStreamer,
    out: impl Write + Send + 'static,
    total: u64,
) -> Result<Tally> {
    let host_bytes = rx.host_format().bits_per_sample() / 8;
    let block_samples = BLOCK_BYTES / host_bytes;

    let (full_tx, full_rx) = mpsc::channel::<(Vec<u8>, usize)>();
    let (empty_tx, empty_rx) = mpsc::channel::<Vec<u8>>();
    for _ in 0..BLOCKS {
        empty_tx
            .send(vec![0u8; block_samples * host_bytes])
            .map_err(|_| Error::Internal("block pool closed".into()))?;
    }
    let writer = std::thread::spawn(move || -> std::io::Result<()> {
        let mut out = out;
        for (block, len) in full_rx {
            out.write_all(&block[..len])?;
            // The receive loop may already have finished; a closed pool is fine.
            empty_tx.send(block).ok();
        }
        out.flush()
    });

    let result = stream_blocks(rx, total, block_samples, host_bytes, &empty_rx, &full_tx);
    drop(full_tx);
    let written = writer
        .join()
        .map_err(|_| Error::Internal("the file writer thread panicked".into()))?;
    let tally = result?;
    written?;
    Ok(tally)
}

/// The receive half of [`record`]: fills blocks from the pool and hands them
/// to the writer.
fn stream_blocks(
    rx: &mut dyn RawRxStreamer,
    total: u64,
    block_samples: usize,
    host_bytes: usize,
    empty: &mpsc::Receiver<Vec<u8>>,
    full: &mpsc::Sender<(Vec<u8>, usize)>,
) -> Result<Tally> {
    let mut md = RxMetadata::default();
    let mut tally = Tally::default();
    let closed = || Error::Internal("the file writer stopped early".into());

    rx.issue_stream_cmd(&StreamCmd::start_continuous())?;
    let started = Instant::now();
    let mut timeout = FIRST_TIMEOUT;
    let mut first = true;
    let mut block = empty.recv().map_err(|_| closed())?;
    let mut filled = 0usize;

    while tally.samples < total {
        let room = (block_samples - filled).min((total - tally.samples) as usize);
        let n = rx.recv_raw(
            &mut [&mut block[filled * host_bytes..(filled + room) * host_bytes]],
            timeout,
            &mut md,
        )?;
        match md.error {
            None => {}
            Some(RxError::Overflow) if md.out_of_sequence => {
                tally.gaps += 1;
                eprint!("D");
            }
            Some(RxError::Overflow) => {
                tally.overflows += 1;
                eprint!("O");
            }
            Some(RxError::BadPacket) => tally.bad_packets += 1,
            Some(RxError::Timeout) => {
                rx.issue_stream_cmd(&StreamCmd::stop()).ok();
                return Err(Error::Timeout(timeout));
            }
            Some(other) => {
                rx.issue_stream_cmd(&StreamCmd::stop()).ok();
                return Err(Error::Device(format!("receive stream error: {other:?}")));
            }
        }
        if n > 0 && first {
            first = false;
            timeout = RUNNING_TIMEOUT;
            if let Some(time) = md.time_spec {
                eprintln!("  first sample: device time {:.9} s", time.as_secs_f64());
            }
        }
        filled += n;
        tally.samples += n as u64;
        if filled == block_samples || tally.samples == total {
            full.send((block, filled * host_bytes))
                .map_err(|_| closed())?;
            filled = 0;
            if tally.samples < total {
                block = empty.recv().map_err(|_| closed())?;
            } else {
                block = Vec::new();
            }
        }
    }
    drop(block);

    rx.issue_stream_cmd(&StreamCmd::stop())?;
    let elapsed = started.elapsed().as_secs_f64();
    if tally.overflows + tally.gaps > 0 {
        eprintln!();
    }
    eprintln!(
        "  streamed for {elapsed:.2} s ({:.3} MS/s average)",
        tally.samples as f64 / elapsed / 1e6
    );
    Ok(tally)
}
