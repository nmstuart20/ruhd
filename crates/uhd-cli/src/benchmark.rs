//! `ruhd benchmark`: measure how fast a device can stream to the host, the
//! receive half of UHD's `benchmark_rate`.
//!
//! The receive loop is `benchmark_rx_rate` translated: it pulls samples into
//! a buffer that is thrown away, counts overflows, sequence errors, dropped
//! samples, late commands and timeouts, and prints the same summary block.
//! Transmit is not implemented yet, so the TX lines of that block are always
//! zero.

use std::process::ExitCode;
use std::time::{Duration, Instant};

use ad936x::Direction;
use clap::Args;
use uhd_b200::{OpenOptions, RxOptions};
use uhd_core::metadata::{RxError, RxMetadata};
use uhd_core::stream::{RawRxStreamer, StreamArgs, StreamCmd};
use uhd_core::{Error, Result, TimeSpec};

use crate::rx::FileType;

#[derive(Debug, Args)]
pub(crate) struct BenchmarkArgs {
    /// RX sample rate, in samples per second.
    #[arg(long, alias = "rx_rate")]
    rx_rate: f64,
    /// Seconds to stream for.
    #[arg(long, default_value_t = 10.0)]
    duration: f64,
    /// Samples per packet from the device. Defaults to as many as fit in a
    /// USB frame.
    #[arg(long, alias = "rx_spp")]
    rx_spp: Option<usize>,
    /// Samples asked for per `recv` call. Defaults to one packet's worth.
    #[arg(long, alias = "rx_spb")]
    rx_spb: Option<usize>,
    /// Over-the-wire sample format. Only sc16 is supported.
    #[arg(long, alias = "rx_otw", default_value = "sc16")]
    rx_otw: String,
    /// Host sample format the samples are converted to.
    #[arg(long, alias = "rx_cpu", value_enum, default_value_t = FileType::Fc32)]
    rx_cpu: FileType,
    /// Ask for a random number of samples in every `recv` call, each with its
    /// own `num_samps_and_done` stream command, as `benchmark_rate --random`
    /// does.
    #[arg(long)]
    random: bool,
    /// Seconds after the device clock is read to start streaming. Zero
    /// starts immediately.
    #[arg(long, alias = "rx_delay", default_value_t = 0.0)]
    rx_delay: f64,
    /// Overruns above which the benchmark counts as failed.
    #[arg(long, alias = "overrun_threshold")]
    overrun_threshold: Option<u64>,
    /// RX sequence errors (dropped packets) above which the benchmark counts
    /// as failed.
    #[arg(long, alias = "drop_threshold")]
    drop_threshold: Option<u64>,
    /// Master clock rate, in Hz. Chosen from the sample rate the way UHD does
    /// if not given.
    #[arg(long)]
    master_clock_rate: Option<f64>,
    /// RF centre frequency, in Hz. The codec's default if not given; it does
    /// not affect the result.
    #[arg(long)]
    freq: Option<f64>,
    /// USB transfers kept queued with the kernel.
    #[arg(long)]
    num_recv_frames: Option<usize>,
    /// Only the device with this serial.
    #[arg(long)]
    serial: Option<String>,
}

/// The counters `benchmark_rate` reports.
#[derive(Debug, Default)]
struct Stats {
    rx_samps: u64,
    dropped_samps: u64,
    overruns: u64,
    seqrx_errors: u64,
    late_commands: u64,
    timeouts_rx: u64,
    bad_packets: u64,
    /// Samples received before the stop command went out.
    samps_before_stop: u64,
    /// Wall time from the first stream command (plus any start delay) to the
    /// stop command.
    streamed: Duration,
}

/// `benchmark_rate`'s `[HH:MM:SS.ffffff]` timestamp, relative to `start`.
fn stamp(start: Instant) -> String {
    let t = start.elapsed();
    let secs = t.as_secs();
    format!(
        "{:02}:{:02}:{:02}.{:06}",
        secs / 3600,
        secs / 60 % 60,
        secs % 60,
        t.subsec_micros()
    )
}

pub(crate) fn run(args: &BenchmarkArgs) -> Result<ExitCode> {
    if !args.rx_otw.eq_ignore_ascii_case("sc16") {
        return Err(Error::NotSupported(format!(
            "--rx-otw {}: only sc16 is supported",
            args.rx_otw
        )));
    }
    if !args.duration.is_finite() || args.duration <= 0.0 {
        return Err(Error::invalid_arg(format!(
            "--duration must be a positive number of seconds, not {}",
            args.duration
        )));
    }
    if !args.rx_delay.is_finite() || args.rx_delay < 0.0 {
        return Err(Error::invalid_arg(format!(
            "--rx-delay must be zero or more seconds, not {}",
            args.rx_delay
        )));
    }

    let start = Instant::now();
    println!();
    println!("[{}] Creating the usrp device...", stamp(start));
    let mut open_options = OpenOptions::new();
    if let Some(serial) = &args.serial {
        open_options = open_options.with_serial(serial);
    }
    let session = uhd_b200::open(&open_options)?;
    println!(
        "Using Device: {} ({})",
        session.product(),
        session.endpoint()
    );

    let master_clock_rate = match args.master_clock_rate {
        Some(rate) => rate,
        None => uhd_b200::auto_master_clock_rate(args.rx_rate, 1)?,
    };
    // `init_radio` also zeroes the device clock, which is benchmark_rate's
    // "Setting device timestamp to 0".
    println!(
        "[{}] Setting device timestamp to 0 (master clock {:.6} MHz)...",
        stamp(start),
        master_clock_rate / 1e6
    );
    let tick_rate = session.init_radio(master_clock_rate)?;
    if let Some(freq) = args.freq {
        session.codec().tune(Direction::Rx, freq)?;
    }

    let mut stream_args = StreamArgs::new();
    stream_args.host_format = args.rx_cpu.format();
    if let Some(spp) = args.rx_spp {
        println!("Setting RX samples per packet (spp) to {spp}");
        stream_args = stream_args.with_arg("spp", spp.to_string());
    }
    if let Some(frames) = args.num_recv_frames {
        stream_args = stream_args.with_arg("num_recv_frames", frames.to_string());
    }
    let mut rx = session.rx_stream(&RxOptions::new(args.rx_rate), &stream_args)?;
    let rate = rx.sample_rate();
    // As benchmark_rate: --rx_spb wins, then --rx_spp, then one packet.
    let spb = args
        .rx_spb
        .or(args.rx_spp)
        .unwrap_or_else(|| rx.max_num_samps());
    if spb == 0 {
        return Err(Error::invalid_arg("--rx-spb must be at least 1"));
    }
    let time = session.time_core(0)?;
    time.set_tick_rate(tick_rate)?;

    println!(
        "[{}] Testing receive rate {:.6} Msps on 1 channels ({}, {} samples per packet, {} per recv)",
        stamp(start),
        rate / 1e6,
        args.rx_cpu.format().name(),
        rx.samples_per_packet(),
        spb
    );
    let stats = benchmark_rx(&mut rx, args, rate, spb, &|| time.get_time_now(), start)?;
    drop(rx);
    println!("[{}] Benchmark complete.", stamp(start));
    println!();

    print_summary(&stats, rate);

    let overrun_err = args.overrun_threshold.is_some_and(|t| stats.overruns > t);
    let drop_err = args.drop_threshold.is_some_and(|t| stats.seqrx_errors > t);
    if overrun_err || drop_err {
        println!("The following error thresholds were exceeded:");
        if let (true, Some(t)) = (overrun_err, args.overrun_threshold) {
            println!("  * Overruns ({}/{t})", stats.overruns);
        }
        if let (true, Some(t)) = (drop_err, args.drop_threshold) {
            println!("  * Dropped packets (RX) ({}/{t})", stats.seqrx_errors);
        }
        return Ok(ExitCode::FAILURE);
    }
    Ok(ExitCode::SUCCESS)
}

fn print_summary(stats: &Stats, rate: f64) {
    println!();
    println!("Benchmark rate summary:");
    println!("  Num received samples:     {}", stats.rx_samps);
    println!("  Num dropped samples:      {}", stats.dropped_samps);
    println!("  Num overruns detected:    {}", stats.overruns);
    println!("  Num transmitted samples:  0");
    println!("  Num sequence errors (Tx): 0");
    println!("  Num sequence errors (Rx): {}", stats.seqrx_errors);
    println!("  Num underruns detected:   0");
    println!("  Num late commands:        {}", stats.late_commands);
    println!("  Num timeouts (Tx):        0");
    println!("  Num timeouts (Rx):        {}", stats.timeouts_rx);
    if stats.bad_packets > 0 {
        println!("  Num bad packets (Rx):     {}", stats.bad_packets);
    }
    // Not in benchmark_rate: what the host actually achieved, which is the
    // number to compare against the requested rate.
    let secs = stats.streamed.as_secs_f64();
    if secs > 0.0 {
        let achieved = stats.samps_before_stop as f64 / secs;
        println!(
            "  Achieved receive rate:    {:.6} Msps over {secs:.3} s ({:.2}% of requested)",
            achieved / 1e6,
            100.0 * achieved / rate
        );
    }
    println!();
    println!("Done!");
    println!();
}

/// A small xorshift generator for `--random`: the sizes only need to vary,
/// not to be good random numbers, and this keeps a dependency out.
#[derive(Debug)]
struct XorShift(u64);

impl XorShift {
    fn seeded() -> Self {
        let nanos = std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)
            .map_or(0, |d| d.as_nanos() as u64);
        Self(nanos | 1)
    }

    /// A number in `1..=n`.
    fn up_to(&mut self, n: usize) -> usize {
        self.0 ^= self.0 << 13;
        self.0 ^= self.0 >> 7;
        self.0 ^= self.0 << 17;
        (self.0 % n as u64) as usize + 1
    }
}

/// `benchmark_rx_rate`: receive into a scratch buffer until `args.duration`
/// has passed, counting what went wrong.
fn benchmark_rx(
    rx: &mut dyn RawRxStreamer,
    args: &BenchmarkArgs,
    rate: f64,
    spb: usize,
    time_now: &dyn Fn() -> Result<TimeSpec>,
    start: Instant,
) -> Result<Stats> {
    let host_bytes = rx.host_format().bits_per_sample() / 8;
    let mut buff = vec![0u8; spb * host_bytes];
    let mut md = RxMetadata::default();
    let mut stats = Stats::default();
    let mut rng = XorShift::seeded();

    let delay = TimeSpec::from_secs(args.rx_delay);
    let timed = args.rx_delay > 0.0;
    // Builds the command for one pass: continuous, or a random-length burst
    // when --random is given; timed when --rx-delay is.
    let make_cmd = |rng: &mut XorShift| -> Result<(StreamCmd, usize)> {
        let (cmd, nsamps) = if args.random {
            let n = rng.up_to(spb);
            (StreamCmd::num_samps(n as u64), n)
        } else {
            (StreamCmd::start_continuous(), spb)
        };
        Ok(if timed {
            (cmd.at(time_now()? + delay), nsamps)
        } else {
            (cmd, nsamps)
        })
    };

    let (cmd, mut nsamps) = make_cmd(&mut rng)?;
    rx.issue_stream_cmd(&cmd)?;
    let started = Instant::now();
    // The duration is counted from the stream command, plus the start delay,
    // as benchmark_rate sleeps for `duration + rx_delay`.
    let deadline = started + Duration::from_secs_f64(args.duration + args.rx_delay);

    let burst_pkt_time = Duration::from_secs_f64((2.0 * spb as f64 / rate).max(0.1));
    let mut timeout = burst_pkt_time + Duration::from_secs_f64(args.rx_delay);

    let mut had_an_overflow = false;
    let mut last_time = TimeSpec::default();
    let mut stop_called = false;

    loop {
        let elapsed = Instant::now() >= deadline;
        if elapsed && !stop_called {
            rx.issue_stream_cmd(&StreamCmd::stop())?;
            stop_called = true;
            stats.samps_before_stop = stats.rx_samps;
            stats.streamed = started
                .elapsed()
                .saturating_sub(Duration::from_secs_f64(args.rx_delay));
        }
        if args.random && !stop_called {
            let (cmd, n) = make_cmd(&mut rng)?;
            nsamps = n;
            rx.issue_stream_cmd(&cmd)?;
        }
        let n = match rx.recv_raw(&mut [&mut buff[..nsamps * host_bytes]], timeout, &mut md) {
            Ok(n) => n,
            Err(e) => {
                eprintln!("[{}] Caught an IO exception.", stamp(start));
                eprintln!("{e}");
                break;
            }
        };
        stats.rx_samps += n as u64;
        timeout = burst_pkt_time;

        match md.error {
            None => {
                if had_an_overflow {
                    had_an_overflow = false;
                    if let Some(now) = md.time_spec {
                        let dropped = (now - last_time).to_ticks(rate);
                        if dropped < 0 {
                            eprintln!(
                                "[{}] Timestamp after overrun recovery ahead of error \
                                 timestamp! Unable to calculate number of dropped \
                                 samples.(Delta: {dropped} ticks)",
                                stamp(start)
                            );
                        }
                        stats.dropped_samps += dropped.max(1) as u64;
                    } else {
                        stats.dropped_samps += 1;
                    }
                }
                if stop_called && md.end_of_burst {
                    break;
                }
            }
            Some(RxError::Overflow) => {
                last_time = md.time_spec.unwrap_or(last_time);
                had_an_overflow = true;
                if md.out_of_sequence {
                    stats.seqrx_errors += 1;
                    eprintln!("[{}] Detected Rx sequence error.", stamp(start));
                } else {
                    stats.overruns += 1;
                }
            }
            Some(RxError::LateCommand) => {
                eprintln!(
                    "[{}] Receiver error: late command, restart streaming...",
                    stamp(start)
                );
                stats.late_commands += 1;
                let (cmd, n) = make_cmd(&mut rng)?;
                nsamps = n;
                let cmd = cmd.at(time_now()? + TimeSpec::from_secs(0.05));
                rx.issue_stream_cmd(&cmd)?;
            }
            Some(RxError::Timeout) => {
                if stop_called {
                    break;
                }
                eprintln!("[{}] Receiver error: timeout, continuing...", stamp(start));
                stats.timeouts_rx += 1;
            }
            Some(RxError::BadPacket) => {
                stats.bad_packets += 1;
                eprintln!(
                    "[{}] Receiver error: bad packet, continuing...",
                    stamp(start)
                );
            }
            Some(other) => {
                eprintln!("[{}] Receiver error: {other:?}", stamp(start));
                eprintln!("[{}] Unexpected error on recv, continuing...", stamp(start));
            }
        }
    }
    if !stop_called {
        // The loop ended on an error before the duration was up.
        stats.samps_before_stop = stats.rx_samps;
        stats.streamed = started
            .elapsed()
            .saturating_sub(Duration::from_secs_f64(args.rx_delay));
    }
    Ok(stats)
}
