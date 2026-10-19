//! `ruhd benchmark`: measure how fast a device can stream to the host, the
//! receive half of UHD's `benchmark_rate`.
//!
//! The receive loop is `benchmark_rx_rate` translated: it pulls samples into
//! a buffer that is thrown away, counts overflows, sequence errors, dropped
//! samples, late commands and timeouts, and prints the same summary block.
//! This benchmark measures receive only, so the TX lines of that block are always
//! zero.

use std::path::PathBuf;
use std::process::ExitCode;
use std::time::{Duration, Instant};

use clap::Args;
use serde_json::json;
use uhd_core::metadata::{RxError, RxMetadata};
use uhd_core::stream::{RawRxStreamer, StreamArgs, StreamCmd};
use uhd_core::{DeviceArgs, Direction, Error, Result, TimeSpec, TuneRequest, registry};

use crate::benchmark_metrics::{ProcessUsage, RecvTiming};
use crate::rx::FileType;
use crate::rx_stats::LossStats;

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
    /// Fail when the exact known missing sample count exceeds this value.
    #[arg(long)]
    loss_threshold: Option<u64>,
    /// Fail when loss episodes with unknown sample counts exceed this value.
    #[arg(long)]
    unknown_loss_threshold: Option<u64>,
    /// Write a versioned JSON report, including results from a failed run.
    #[arg(long)]
    json: Option<PathBuf>,
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
    loss: LossStats,
    recv_timing: RecvTiming,
    first_samples_after_command: Option<Duration>,
    overruns: u64,
    seqrx_errors: u64,
    late_commands: u64,
    timeouts_rx: u64,
    bad_packets: u64,
    /// Timestamp discontinuities detected by the streamer, including backward
    /// jumps whose missing sample counts cannot be established.
    time_jumps: u64,
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
    let mut device_args = DeviceArgs::new().with("type", "b200");
    if let Some(serial) = &args.serial {
        device_args.set("serial", serial);
    }
    if let Some(rate) = args.master_clock_rate {
        device_args.set("master_clock_rate", rate.to_string());
    }
    let device = registry::open(&device_args)?;
    println!("Using Device: {}", device.info());
    let rate = device.set_sample_rate(Direction::Rx, 0, args.rx_rate)?;
    device.set_time_now(0, TimeSpec::default())?;
    let tick_rate = device.master_clock_rate(0)?;
    println!(
        "[{}] Setting device timestamp to 0 (master clock {:.6} MHz)...",
        stamp(start),
        tick_rate / 1e6
    );
    if let Some(freq) = args.freq {
        device.tune(Direction::Rx, 0, &TuneRequest::new(freq))?;
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
    let mut rx = device.open_rx_stream(&stream_args)?;
    let rx_spp = rx.max_num_samps();
    // As benchmark_rate: --rx_spb wins, then --rx_spp, then one packet.
    let spb = args
        .rx_spb
        .or(args.rx_spp)
        .unwrap_or_else(|| rx.max_num_samps());
    if spb == 0 {
        return Err(Error::invalid_arg("--rx-spb must be at least 1"));
    }

    println!(
        "[{}] Testing receive rate {:.6} Msps on 1 channels ({}, {} samples per packet, {} per recv)",
        stamp(start),
        rate / 1e6,
        args.rx_cpu.format().name(),
        rx.max_num_samps(),
        spb
    );
    let mut stats = Stats::default();
    let usage_start = ProcessUsage::now();
    let measurement_start = Instant::now();
    let result = benchmark_rx(
        rx.as_mut(),
        args,
        rate,
        spb,
        &|| device.time_now(0),
        start,
        &mut stats,
    );
    let measurement_wall = measurement_start.elapsed();
    let usage_end = ProcessUsage::now();
    let process = ProcessUsage::report(usage_start, usage_end, measurement_wall);
    drop(rx);
    println!(
        "[{}] Benchmark {}.",
        stamp(start),
        if result.is_ok() { "complete" } else { "failed" }
    );
    println!();

    print_summary(&stats, rate);
    let failures = args.threshold_failures(&stats);
    println!("  Host resources: {process}");
    let report_result = if let Some(path) = &args.json {
        let config = RunConfiguration {
            device: device.info().to_string(),
            rate,
            tick_rate,
            spp: rx_spp,
            spb,
        };
        let report = args.report(&stats, &config, &process, result.as_ref().err());
        std::fs::write(
            path,
            serde_json::to_string_pretty(&report).expect("JSON value serializes") + "\n",
        )
    } else {
        Ok(())
    };
    // Keep a receive failure primary if writing the report also failed.
    if let Err(error) = &report_result {
        eprintln!("Could not write benchmark report: {error}");
    }
    result?;
    report_result?;
    if !failures.is_empty() {
        println!("The following error thresholds were exceeded:");
        for failure in failures {
            println!("  * {failure}");
        }
        return Ok(ExitCode::FAILURE);
    }
    Ok(ExitCode::SUCCESS)
}

struct RunConfiguration {
    device: String,
    rate: f64,
    tick_rate: f64,
    spp: usize,
    spb: usize,
}

impl BenchmarkArgs {
    fn threshold_failures(&self, stats: &Stats) -> Vec<String> {
        [
            ("Overruns", stats.overruns, self.overrun_threshold),
            (
                "Sequence errors (RX)",
                stats.seqrx_errors,
                self.drop_threshold,
            ),
            (
                "Known missing samples",
                stats.loss.missing_samples,
                self.loss_threshold,
            ),
            (
                "Unknown loss episodes",
                stats.loss.unknown_events(),
                self.unknown_loss_threshold,
            ),
        ]
        .into_iter()
        .filter_map(|(name, count, limit)| {
            limit
                .filter(|limit| count > *limit)
                .map(|limit| format!("{name} ({count}/{limit})"))
        })
        .collect()
    }

    fn report(
        &self,
        stats: &Stats,
        config: &RunConfiguration,
        process: &serde_json::Value,
        error: Option<&Error>,
    ) -> serde_json::Value {
        let failures = self.threshold_failures(stats);
        json!({
            "schema_version": 1,
            "ruhd_version": env!("CARGO_PKG_VERSION"),
            "host": { "os": std::env::consts::OS, "arch": std::env::consts::ARCH },
            "device": config.device,
            "completed": error.is_none(),
            "passed": error.is_none() && failures.is_empty(),
            "error": error.map(ToString::to_string),
            "configuration": {
                "requested_rate_sps": self.rx_rate,
                "actual_rate_sps": config.rate,
                "master_clock_rate_hz": config.tick_rate,
                "host_format": self.rx_cpu.format().name(),
                "wire_format": self.rx_otw,
                "samples_per_packet": config.spp,
                "samples_per_recv": config.spb,
                "num_recv_frames": self.num_recv_frames.unwrap_or(uhd_b200::rx_stream::DEFAULT_NUM_RECV_FRAMES),
                "duration_seconds": self.duration,
                "start_delay_seconds": self.rx_delay,
                "random_bursts": self.random,
            },
            "thresholds": {
                "overrun": self.overrun_threshold,
                "sequence_error": self.drop_threshold,
                "known_missing_samples": self.loss_threshold,
                "unknown_loss_episodes": self.unknown_loss_threshold,
                "passed": failures.is_empty(),
                "failures": failures,
            },
            "results": stats.report(config.rate),
            "process": process,
        })
    }
}

fn print_summary(stats: &Stats, rate: f64) {
    println!();
    println!("Benchmark rate summary:");
    println!("  Num received samples:     {}", stats.rx_samps);
    println!(
        "  Num dropped samples:      {} (known exactly)",
        stats.loss.missing_samples
    );
    println!("  Num loss episodes:        {}", stats.loss.events());
    println!(
        "  Unknown loss counts:      {}",
        stats.loss.unknown_events()
    );
    println!(
        "  Known gap duration:       {:.9} s",
        stats.loss.gap_seconds
    );
    println!("  Num overruns detected:    {}", stats.overruns);
    println!("  Num transmitted samples:  0");
    println!("  Num sequence errors (Tx): 0");
    println!("  Num sequence errors (Rx): {}", stats.seqrx_errors);
    println!("  Num underruns detected:   0");
    println!("  Num late commands:        {}", stats.late_commands);
    println!("  Num timeouts (Tx):        0");
    println!("  Num timeouts (Rx):        {}", stats.timeouts_rx);
    println!("  Num timestamp jumps (Rx): {}", stats.time_jumps);
    if stats.bad_packets > 0 {
        println!("  Num bad packets (Rx):     {}", stats.bad_packets);
    }
    println!("  Receive call timing (us): {}", stats.recv_timing.report());
    if let Some(first) = stats.first_samples_after_command {
        println!(
            "  First samples after cmd:  {:.3} ms",
            first.as_secs_f64() * 1000.0
        );
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
}

impl Stats {
    fn report(&self, rate: f64) -> serde_json::Value {
        let secs = self.streamed.as_secs_f64();
        let achieved = (secs > 0.0).then(|| self.samps_before_stop as f64 / secs);
        json!({
            "received_samples": self.rx_samps,
            "samples_before_stop": self.samps_before_stop,
            "stream_seconds": secs,
            "achieved_rate_sps": achieved,
            "achieved_percent": achieved.map(|r| 100.0 * r / rate),
            "known_missing_samples": self.loss.missing_samples,
            "loss_episodes": self.loss.events(),
            "unknown_loss_episodes": self.loss.unknown_events(),
            "unresolved_loss": self.loss.unresolved(),
            "known_gap_seconds": self.loss.gap_seconds,
            "overruns": self.overruns,
            "sequence_errors": self.seqrx_errors,
            "timestamp_jumps": self.time_jumps,
            "bad_packets": self.bad_packets,
            "late_commands": self.late_commands,
            "timeouts": self.timeouts_rx,
            "recv_timing": self.recv_timing.report(),
            "first_samples_after_command_seconds": self.first_samples_after_command.map(|d| d.as_secs_f64()),
        })
    }
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
    stats: &mut Stats,
) -> Result<()> {
    let host_bytes = rx.host_format().bits_per_sample() / 8;
    let mut buff = vec![0u8; spb * host_bytes];
    let mut md = RxMetadata::default();
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

    let mut stop_called = false;

    let result: Result<()> = (|| {
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
            let call_start = Instant::now();
            let received = rx.recv_raw(&mut [&mut buff[..nsamps * host_bytes]], timeout, &mut md);
            if !stop_called {
                stats.recv_timing.observe(
                    call_start.elapsed(),
                    received.as_ref().ok().copied(),
                    nsamps,
                );
            }
            let n = received?;
            if n > 0 && stats.first_samples_after_command.is_none() {
                stats.first_samples_after_command = Some(started.elapsed());
            }
            stats.loss.observe(n, &md);
            stats.rx_samps += n as u64;
            timeout = burst_pkt_time;

            match md.error {
                None => {
                    if stop_called && md.end_of_burst {
                        break;
                    }
                }
                Some(RxError::Overflow) => {
                    if md.out_of_sequence {
                        stats.seqrx_errors += 1;
                        eprintln!("[{}] Detected Rx sequence error.", stamp(start));
                    } else {
                        stats.overruns += 1;
                    }
                }
                Some(RxError::TimestampJump) => {
                    stats.time_jumps += 1;
                    eprintln!("[{}] Detected Rx timestamp discontinuity.", stamp(start));
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
        Ok(())
    })();
    let stopped = if !stop_called {
        // The loop ended on an error before the duration was up.
        stats.samps_before_stop = stats.rx_samps;
        stats.streamed = started
            .elapsed()
            .saturating_sub(Duration::from_secs_f64(args.rx_delay));
        rx.issue_stream_cmd(&StreamCmd::stop())
    } else {
        Ok(())
    };
    // Keep the original failure if stopping an already broken stream fails too.
    result?;
    stopped
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::test_support::ScriptedRx;
    use clap::Parser;
    use uhd_core::{RxLoss, RxLossCause};

    fn args() -> BenchmarkArgs {
        let cli = crate::Cli::parse_from(["ruhd", "benchmark", "--rx-rate", "1e6"]);
        let crate::Command::Benchmark(args) = cli.command else {
            unreachable!();
        };
        args
    }

    #[test]
    fn reports_preserve_failures_and_gate_known_and_unknown_loss_separately() {
        let mut args = args();
        args.loss_threshold = Some(0);
        args.unknown_loss_threshold = Some(0);
        let config = RunConfiguration {
            device: "B205mini test".into(),
            rate: 999_999.0,
            tick_rate: 32e6,
            spp: 2040,
            spb: 4096,
        };
        let mut stats = Stats::default();
        stats.loss.observe(
            0,
            &RxMetadata {
                error: Some(RxError::Overflow),
                ..Default::default()
            },
        );
        let report = args.report(&stats, &config, &json!({}), None);
        assert_eq!(report["completed"], true);
        assert_eq!(report["passed"], false);
        assert_eq!(report["results"]["known_missing_samples"], 0);
        assert_eq!(report["results"]["unknown_loss_episodes"], 1);
        assert_eq!(
            report["thresholds"]["failures"].as_array().unwrap().len(),
            1
        );
        assert_eq!(report["configuration"]["requested_rate_sps"], 1e6);
        assert_eq!(report["configuration"]["actual_rate_sps"], config.rate);
        stats.loss.observe(
            1,
            &RxMetadata {
                loss: Some(RxLoss::new(
                    RxLossCause::Overflow,
                    Some(0),
                    Some(40),
                    1e6,
                    4,
                )),
                ..Default::default()
            },
        );
        args.loss_threshold = Some(10);
        assert!(args.threshold_failures(&stats).is_empty());
        let failure = Error::transport("USB disconnected");
        let report = args.report(&stats, &config, &json!({}), Some(&failure));
        assert_eq!(report["completed"], false);
        assert_eq!(report["passed"], false);
        assert_eq!(report["thresholds"]["passed"], true);
        assert!(
            report["error"]
                .as_str()
                .unwrap()
                .contains("USB disconnected")
        );
        assert_eq!(report["results"]["known_missing_samples"], 10);
        // Ensure the report can be persisted and read as the same schema.
        let text = serde_json::to_string_pretty(&report).unwrap();
        assert_eq!(
            serde_json::from_str::<serde_json::Value>(&text).unwrap(),
            report
        );
    }

    #[test]
    fn benchmark_counts_known_unknown_and_unresolved_loss_without_double_counting() {
        let error = |error| {
            Ok((
                Vec::new(),
                RxMetadata {
                    error: Some(error),
                    ..Default::default()
                },
            ))
        };
        let mut rx = ScriptedRx::new([
            Ok((vec![1; 8], RxMetadata::default())),
            error(RxError::Overflow),
            error(RxError::BadPacket),
            Ok((
                vec![1; 8],
                RxMetadata {
                    loss: Some(RxLoss::new(
                        RxLossCause::Overflow,
                        Some(8),
                        Some(48),
                        1e6,
                        4,
                    )),
                    ..Default::default()
                },
            )),
            error(RxError::TimestampJump),
            Ok((
                vec![1; 4],
                RxMetadata {
                    loss: Some(RxLoss::new(
                        RxLossCause::TimestampJump,
                        Some(56),
                        Some(4),
                        1e6,
                        4,
                    )),
                    ..Default::default()
                },
            )),
            error(RxError::Overflow),
            Err(Error::transport("disconnected during recovery")),
        ]);
        let mut stats = Stats::default();
        let result = benchmark_rx(
            &mut rx,
            &args(),
            1e6,
            4,
            &|| Ok(TimeSpec::ZERO),
            Instant::now(),
            &mut stats,
        );
        assert!(result.is_err());
        let report = stats.report(1e6);
        assert_eq!(report["received_samples"], 5);
        assert_eq!(report["known_missing_samples"], 10);
        assert_eq!(report["loss_episodes"], 3);
        assert_eq!(report["unknown_loss_episodes"], 2);
        assert_eq!(report["unresolved_loss"], true);
        assert_eq!(report["overruns"], 2);
        assert_eq!(report["bad_packets"], 1);
        assert_eq!(report["timestamp_jumps"], 1);
        assert_eq!(report["recv_timing"]["calls"], 8);
        assert_eq!(report["recv_timing"]["failed_calls"], 1);
        assert_eq!(rx.commands.last(), Some(&StreamCmd::stop()));
    }

    #[test]
    fn receive_failure_preserves_stats_and_stops_the_stream() {
        let mut rx = ScriptedRx::new([
            Ok((vec![1; 8], RxMetadata::default())),
            Err(Error::transport("device disconnected")),
        ]);
        let mut stats = Stats::default();
        let result = benchmark_rx(
            &mut rx,
            &args(),
            1e6,
            4,
            &|| Ok(TimeSpec::ZERO),
            Instant::now(),
            &mut stats,
        );

        assert!(
            matches!(result, Err(Error::Transport(ref message)) if message == "device disconnected")
        );
        assert_eq!(stats.rx_samps, 2);
        assert_eq!(stats.samps_before_stop, 2);
        assert!(stats.streamed > Duration::ZERO);
        assert_eq!(
            rx.commands,
            [StreamCmd::start_continuous(), StreamCmd::stop()]
        );
    }

    #[test]
    fn cleanup_failure_does_not_hide_the_receive_failure() {
        let mut rx = ScriptedRx::new([Err(Error::transport("receive failed"))]);
        rx.stop_error = Some(Error::device("stop failed"));
        let result = benchmark_rx(
            &mut rx,
            &args(),
            1e6,
            4,
            &|| Ok(TimeSpec::ZERO),
            Instant::now(),
            &mut Stats::default(),
        );

        assert!(
            matches!(result, Err(Error::Transport(ref message)) if message == "receive failed")
        );
        assert_eq!(rx.commands.last(), Some(&StreamCmd::stop()));
    }

    #[test]
    fn normal_stop_drains_the_stream_successfully() {
        let mut args = args();
        // Rounds to a zero duration, so the stop command precedes the receive.
        args.duration = f64::MIN_POSITIVE;
        let mut rx = ScriptedRx::new([Ok((
            vec![1; 8],
            RxMetadata {
                end_of_burst: true,
                ..Default::default()
            },
        ))]);
        let mut stats = Stats::default();
        benchmark_rx(
            &mut rx,
            &args,
            1e6,
            4,
            &|| Ok(TimeSpec::ZERO),
            Instant::now(),
            &mut stats,
        )
        .unwrap();

        assert_eq!(stats.rx_samps, 2);
        assert_eq!(stats.samps_before_stop, 0);
        assert_eq!(
            rx.commands,
            [StreamCmd::start_continuous(), StreamCmd::stop()]
        );
    }
}
