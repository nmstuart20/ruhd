//! Raw IQ playback through the generic transmit API.

use crate::rx::FileType;
use clap::Args;
use std::fs::File;
use std::io::{self, BufReader, Read};
use std::path::PathBuf;
use std::sync::atomic::Ordering;
use std::time::{Duration, Instant};
use uhd_core::{
    AsyncEvent, DeviceArgs, Direction, Error, RawTxStreamer, Result, StreamArgs, TimeSpec,
    TuneRequest, TxMetadata, registry,
};

#[derive(Debug, Args)]
pub(crate) struct TxArgs {
    /// Raw interleaved IQ in host byte order, or `-` for stdin.
    #[arg(long, short)]
    file: PathBuf,
    /// RF center frequency in Hz. The TX/RX port is used.
    #[arg(long)]
    freq: f64,
    /// Sample rate in samples per second.
    #[arg(long, default_value_t = 1e6)]
    rate: f64,
    /// TX gain in dB.
    #[arg(long, default_value_t = 0.0)]
    gain: f64,
    /// Analog bandwidth in Hz.
    #[arg(long)]
    bw: Option<f64>,
    /// Input sample format; floats are normalized to [-1, 1].
    #[arg(long = "type", value_enum, default_value = "sc16")]
    file_type: FileType,
    /// Stop after this many input samples; default is EOF or Ctrl-C.
    #[arg(long)]
    nsamps: Option<u64>,
    /// Schedule the first sample this many seconds into the future.
    #[arg(long, default_value_t = 0.0)]
    delay: f64,
    /// Fix the master clock instead of selecting it from the sample rate.
    #[arg(long)]
    master_clock_rate: Option<f64>,
    /// Device serial number.
    #[arg(long)]
    serial: Option<String>,
    /// Samples per USB packet.
    #[arg(long)]
    spp: Option<usize>,
    /// Number of USB transmit buffers.
    #[arg(long)]
    num_send_frames: Option<usize>,
}

// Fill a block across short reads; reject a truncated I/Q sample at EOF.
fn read_samples(reader: &mut dyn Read, buffer: &mut [u8], sample_bytes: usize) -> Result<usize> {
    let mut filled = 0;
    while filled < buffer.len() {
        match reader.read(&mut buffer[filled..]) {
            Ok(0) => break,
            Ok(n) => filled += n,
            Err(e) if e.kind() == io::ErrorKind::Interrupted => continue,
            Err(e) => return Err(e.into()),
        }
    }
    if filled % sample_bytes != 0 {
        return Err(Error::invalid_arg("input ends in a partial complex sample"));
    }
    Ok(filled)
}

fn report(event: AsyncEvent) -> Result<()> {
    match event {
        AsyncEvent::BurstAck => Ok(()),
        AsyncEvent::Underflow | AsyncEvent::UnderflowInPacket => {
            eprint!("U");
            Ok(())
        }
        _ => Err(Error::device(format!("transmit event: {event:?}"))),
    }
}

fn finish(tx: &mut dyn RawTxStreamer, timeout: Duration) -> Result<()> {
    tx.send_raw(
        &[&[]],
        Duration::from_secs(2),
        &TxMetadata {
            end_of_burst: true,
            ..Default::default()
        },
    )?;
    let deadline = Instant::now() + timeout;
    loop {
        let Some(md) = tx.recv_async_msg(deadline.saturating_duration_since(Instant::now()))?
        else {
            return Err(Error::Timeout(timeout));
        };
        if md.event == AsyncEvent::BurstAck {
            return Ok(());
        }
        report(md.event)?;
        if Instant::now() >= deadline {
            return Err(Error::Timeout(timeout));
        }
    }
}

pub(crate) fn run(args: &TxArgs) -> Result<()> {
    if !args.rate.is_finite()
        || args.rate <= 0.0
        || !args.freq.is_finite()
        || !args.gain.is_finite()
        || !args.delay.is_finite()
        || args.delay < 0.0
        || args.delay > 3600.0
    {
        return Err(Error::invalid_arg(
            "rate must be positive; frequency/gain finite; delay in 0..=3600 seconds",
        ));
    }
    let mut input: Box<dyn Read> = if args.file.as_os_str() == "-" {
        Box::new(io::stdin())
    } else {
        Box::new(BufReader::with_capacity(4 << 20, File::open(&args.file)?))
    };
    let sample_bytes = args.file_type.format().bytes_per_sample().unwrap();
    let mut buffer = vec![0u8; 65536 * sample_bytes];
    let first_bytes = buffer
        .len()
        .min(args.nsamps.unwrap_or(u64::MAX).min(65536) as usize * sample_bytes);
    let mut filled = read_samples(&mut input, &mut buffer[..first_bytes], sample_bytes)?;
    if filled == 0 {
        return Ok(());
    }
    let stop = crate::rx::stop_on_ctrl_c()?;
    let mut device_args = DeviceArgs::new().with("type", "b200");
    if let Some(serial) = &args.serial {
        device_args.set("serial", serial);
    }
    if let Some(clock) = args.master_clock_rate {
        device_args.set("master_clock_rate", clock.to_string());
    }
    let device = registry::open(&device_args)?;
    let rate = device.set_sample_rate(Direction::Tx, 0, args.rate)?;
    let freq = device
        .tune(Direction::Tx, 0, &TuneRequest::new(args.freq))?
        .actual_freq();
    device.set_gain(Direction::Tx, 0, None, args.gain)?;
    if let Some(bw) = args.bw {
        device.set_bandwidth(Direction::Tx, 0, bw)?;
    }
    let mut stream_args = StreamArgs::new();
    stream_args.host_format = args.file_type.format();
    if let Some(spp) = args.spp {
        stream_args = stream_args.with_arg("spp", spp.to_string());
    }
    if let Some(frames) = args.num_send_frames {
        stream_args = stream_args.with_arg("num_send_frames", frames.to_string());
    }
    let mut tx = device.open_tx_stream(&stream_args)?;
    let start = (args.delay > 0.0)
        .then(|| {
            device
                .time_now(0)
                .map(|now| now + TimeSpec::from_secs(args.delay))
        })
        .transpose()?;
    eprintln!(
        "Transmitting {} at {:.6} MS/s, {:.6} MHz, {:.2} dB on TX/RX",
        device.info(),
        rate / 1e6,
        freq / 1e6,
        device.gain(Direction::Tx, 0, None)?
    );
    let mut sent = 0u64;
    let mut packets = 0usize;
    let send_timeout = Duration::from_secs_f64(args.delay + 2.0);
    let result = (|| {
        loop {
            let mut offset = 0;
            while offset < filled && !stop.load(Ordering::Relaxed) {
                let count = tx.send_raw(
                    &[&buffer[offset..filled]],
                    send_timeout,
                    &TxMetadata {
                        time_spec: start.map(|t| t + TimeSpec::from_secs(sent as f64 / rate)),
                        start_of_burst: sent == 0,
                        end_of_burst: false,
                    },
                )?;
                if count == 0 {
                    return Err(Error::Timeout(send_timeout));
                }
                sent += count as u64;
                offset += count * sample_bytes;
                packets += 1;
                if packets.is_multiple_of(128) {
                    while let Some(md) = tx.recv_async_msg(Duration::ZERO)? {
                        report(md.event)?;
                    }
                }
            }
            if stop.load(Ordering::Relaxed) || args.nsamps.is_some_and(|n| sent >= n) {
                break;
            }
            let remaining = args
                .nsamps
                .unwrap_or(u64::MAX)
                .saturating_sub(sent)
                .min(65536) as usize;
            filled = read_samples(
                &mut input,
                &mut buffer[..remaining * sample_bytes],
                sample_bytes,
            )?;
            if filled == 0 {
                break;
            }
        }
        Ok(())
    })();
    // Finish even after an input error; a USB failure can still prevent cleanup.
    let ended = if sent > 0 {
        finish(tx.as_mut(), Duration::from_secs_f64(args.delay + 10.0))
    } else {
        Ok(())
    };
    result?;
    ended?;
    eprintln!("\nTransmitted {sent} input samples; burst acknowledged.");
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn raw_input_requires_whole_iq_samples() {
        let mut buffer = [0; 8];
        assert_eq!(
            read_samples(&mut &b"abcdefgh"[..], &mut buffer, 4).unwrap(),
            8
        );
        assert!(read_samples(&mut &b"abc"[..], &mut buffer, 4).is_err());
        assert_eq!(read_samples(&mut &b""[..], &mut buffer, 4).unwrap(), 0);
    }
}
