//! Snapshot the complete AD9364 register file, optionally after RUHD init.
//!
//! This is the hardware-side helper used by
//! `tools/validate_ad9364_registers.py`. Its stdout is intentionally only the
//! machine-readable dump; progress and setting readback go to stderr.
//!
//! ```text
//! cargo run -p uhd-b200 --example ad9364_regdump -- \
//!   --mode init --rate 16000000 --rx-freq 2450000000 --tx-freq 2450000000 \
//!   --rx-gain 37 --tx-gain 10 --bandwidth 16000000
//! ```
//! 
//! As a note, ruhd reports the actual frequency representable by the AD9364's fractional-N PLL
//! So when requesting 2.45 GHz, that lies exactly halfway between two synthesizable LO frequencies
//! 2,449,999,998.808 Hz
//! 2,450,000,001.192 Hz  ← RUHD chooses this by rounding (UHD 4.6 truncates the fractional word, selecting the lower frequency and reports the requested value instead)

use std::error::Error;
use std::io::{self, BufWriter, Write};

use ad936x::{Chain, Direction, LAST_DEVICE_REG};
use uhd_b200::{OpenOptions, open};

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
enum Mode {
    ReadOnly,
    Init,
}

#[derive(Debug)]
struct Args {
    mode: Mode,
    serial: Option<String>,
    rate: f64,
    rx_freq: f64,
    tx_freq: f64,
    rx_gain: f64,
    tx_gain: f64,
    bandwidth: f64,
}

impl Default for Args {
    fn default() -> Self {
        Self {
            mode: Mode::ReadOnly,
            serial: None,
            rate: 16e6,
            rx_freq: 2.45e9,
            tx_freq: 2.45e9,
            rx_gain: 37.0,
            tx_gain: 10.0,
            bandwidth: 16e6,
        }
    }
}

fn invalid(message: impl Into<String>) -> io::Error {
    io::Error::new(io::ErrorKind::InvalidInput, message.into())
}

fn value(iter: &mut impl Iterator<Item = String>, flag: &str) -> io::Result<String> {
    iter.next()
        .ok_or_else(|| invalid(format!("{flag} requires a value")))
}

fn number(iter: &mut impl Iterator<Item = String>, flag: &str) -> io::Result<f64> {
    let raw = value(iter, flag)?;
    let parsed = raw
        .parse::<f64>()
        .map_err(|e| invalid(format!("invalid {flag} value {raw:?}: {e}")))?;
    if parsed.is_finite() {
        Ok(parsed)
    } else {
        Err(invalid(format!("{flag} must be finite")))
    }
}

fn parse_args() -> io::Result<Args> {
    let mut parsed = Args::default();
    let mut iter = std::env::args().skip(1);
    while let Some(flag) = iter.next() {
        match flag.as_str() {
            "--mode" => {
                parsed.mode = match value(&mut iter, &flag)?.as_str() {
                    "read-only" => Mode::ReadOnly,
                    "init" => Mode::Init,
                    other => {
                        return Err(invalid(format!(
                            "unknown mode {other:?}; expected read-only or init"
                        )));
                    }
                };
            }
            "--serial" => parsed.serial = Some(value(&mut iter, &flag)?),
            "--rate" => parsed.rate = number(&mut iter, &flag)?,
            "--rx-freq" => parsed.rx_freq = number(&mut iter, &flag)?,
            "--tx-freq" => parsed.tx_freq = number(&mut iter, &flag)?,
            "--rx-gain" => parsed.rx_gain = number(&mut iter, &flag)?,
            "--tx-gain" => parsed.tx_gain = number(&mut iter, &flag)?,
            "--bandwidth" => parsed.bandwidth = number(&mut iter, &flag)?,
            "--help" | "-h" => {
                eprintln!(
                    "usage: ad9364_regdump --mode read-only|init [--serial SERIAL] \
                     [--rate HZ] [--rx-freq HZ] [--tx-freq HZ] \
                     [--rx-gain DB] [--tx-gain DB] [--bandwidth HZ]"
                );
                std::process::exit(0);
            }
            _ => return Err(invalid(format!("unknown argument {flag:?}"))),
        }
    }
    Ok(parsed)
}

fn main() -> Result<(), Box<dyn Error>> {
    let args = parse_args()?;
    let mut options = OpenOptions::new();
    if let Some(serial) = &args.serial {
        options = options.with_serial(serial);
    }
    let session = open(&options)?;
    let codec = session.codec();

    match args.mode {
        Mode::ReadOnly => {
            let id = codec.identify()?;
            eprintln!("read-only snapshot of {id}; the codec was not reset");
        }
        Mode::Init => {
            session.misc().reset_codec()?;
            codec.init()?;
            codec.set_active_chains(true, false, true, false)?;
            let rate = codec.set_sample_rate(args.rate)?;
            let rx_freq = codec.tune(Direction::Rx, args.rx_freq)?;
            let tx_freq = codec.tune(Direction::Tx, args.tx_freq)?;
            let rx_gain = codec.set_gain(Direction::Rx, Chain::One, args.rx_gain)?;
            let tx_gain = codec.set_gain(Direction::Tx, Chain::One, args.tx_gain)?;
            let rx_bw = codec.set_bandwidth(Direction::Rx, args.bandwidth)?;
            let tx_bw = codec.set_bandwidth(Direction::Tx, args.bandwidth)?;
            eprintln!(
                "RUHD actual settings: rate={rate:.3} rx_freq={rx_freq:.3} \
                 tx_freq={tx_freq:.3} rx_gain={rx_gain:.2} tx_gain={tx_gain:.2} \
                 rx_bandwidth={rx_bw:.3} tx_bandwidth={tx_bw:.3}"
            );
        }
    }

    let dump = codec.debug_register_dump()?;
    if dump.len() != usize::from(LAST_DEVICE_REG) + 1 {
        return Err(invalid(format!(
            "debug API returned {} registers, expected {}",
            dump.len(),
            usize::from(LAST_DEVICE_REG) + 1
        ))
        .into());
    }

    let stdout = io::stdout();
    let mut output = BufWriter::new(stdout.lock());
    for (address, value) in dump {
        writeln!(output, "0x{address:03x}\t0x{value:02x}")?;
    }
    output.flush()?;
    Ok(())
}
