//! `ruhd` — command-line front end, the rough equivalent of `uhd_find_devices`
//! and `uhd_usrp_probe`.

use std::process::ExitCode;

use clap::{Parser, Subcommand};
use tracing::debug;
use tracing_subscriber::EnvFilter;
use uhd_core::prelude::*;
use uhd_core::{DeviceArgs, SensorScope, registry};

#[derive(Debug, Parser)]
#[command(name = "ruhd", version, about, long_about = None)]
struct Cli {
    /// Log level: error, warn, info, debug, trace. `RUST_LOG` overrides this.
    #[arg(long, short, global = true, default_value = "warn")]
    log_level: String,

    #[command(subcommand)]
    command: Command,
}

#[derive(Debug, Subcommand)]
enum Command {
    /// List attached devices.
    Find {
        /// Device arguments to narrow the search, e.g. `type=b200,serial=30A1B2C`.
        #[arg(long, default_value = "")]
        args: String,
    },
    /// Open a device and print what it can do.
    Probe {
        /// Device arguments identifying exactly one device.
        #[arg(long, default_value = "")]
        args: String,
    },
    /// List the device families this build knows about.
    Backends,
}

fn main() -> ExitCode {
    let cli = Cli::parse();

    tracing_subscriber::fmt()
        .with_env_filter(
            EnvFilter::try_from_default_env()
                .unwrap_or_else(|_| EnvFilter::new(cli.log_level.clone())),
        )
        .with_writer(std::io::stderr)
        .init();

    register_backends();

    match run(&cli.command) {
        Ok(()) => ExitCode::SUCCESS,
        Err(e) => {
            eprintln!("error: {e}");
            ExitCode::FAILURE
        }
    }
}

/// Registers every backend compiled into this binary.
///
/// Empty until `uhd-b200` gains a factory; at that point this becomes
/// `uhd_b200::register();` and nothing else in the CLI changes.
fn register_backends() {}

fn run(command: &Command) -> Result<()> {
    debug!(?command, backends = ?registry::registered_types(), "dispatching");
    match command {
        Command::Find { args } => find(args),
        Command::Probe { args } => probe(args),
        Command::Backends => {
            let types = registry::registered_types();
            if types.is_empty() {
                println!("no device backends are registered in this build");
            } else {
                for t in types {
                    println!("{t}");
                }
            }
            Ok(())
        }
    }
}

fn find(args: &str) -> Result<()> {
    let hint: DeviceArgs = args.parse()?;
    let devices = registry::find(&hint)?;

    if devices.is_empty() {
        println!("no devices found");
        if registry::registered_types().is_empty() {
            println!("(no backends are registered in this build yet)");
        }
        return Ok(());
    }

    for (i, info) in devices.iter().enumerate() {
        println!("Device {i}");
        println!("  type:    {}", info.device_type);
        println!("  product: {}", info.product);
        println!("  serial:  {}", info.serial);
        if !info.name.is_empty() {
            println!("  name:    {}", info.name);
        }
        if let Some(revision) = &info.revision {
            println!("  rev:     {revision}");
        }
        println!("  args:    {}", info.args);
    }
    Ok(())
}

fn probe(args: &str) -> Result<()> {
    let hint: DeviceArgs = args.parse()?;
    let device = registry::open(&hint)?;
    let info = device.info();

    println!("{info}");
    println!("  motherboards: {}", device.num_mboards());

    for mboard in 0..device.num_mboards() {
        println!("  mboard {mboard}:");
        if let Ok(rate) = device.master_clock_rate(mboard) {
            println!("    master clock: {:.6} MHz", rate / 1e6);
        }
        if let Ok(source) = device.clock_source(mboard) {
            println!("    clock source: {source}");
        }
        if let Ok(source) = device.time_source(mboard) {
            println!("    time source:  {source}");
        }
        print_sensors(device.as_ref(), SensorScope::Mboard(mboard));
    }

    for dir in Direction::ALL {
        let count = device.num_channels(dir);
        println!("  {dir}: {count} channel(s)");
        for chan in 0..count {
            print_channel(device.as_ref(), dir, chan);
        }
    }

    Ok(())
}

fn print_channel(device: &dyn Device, dir: Direction, chan: usize) {
    println!("    {dir} channel {chan}:");

    match device.freq_range(dir, chan) {
        Ok(range) => println!("      freq range:  {range} Hz"),
        Err(e) => println!("      freq range:  <{e}>"),
    }
    match device.center_freq(dir, chan) {
        Ok(freq) => println!("      center freq: {:.6} MHz", freq / 1e6),
        Err(e) => println!("      center freq: <{e}>"),
    }
    match device.sample_rate(dir, chan) {
        Ok(rate) => println!("      rate:        {:.6} Msps", rate / 1e6),
        Err(e) => println!("      rate:        <{e}>"),
    }
    match device.gain_range(dir, chan, None) {
        Ok(range) => println!("      gain range:  {range} dB"),
        Err(e) => println!("      gain range:  <{e}>"),
    }
    if let Ok(names) = device.gain_names(dir, chan)
        && !names.is_empty()
    {
        println!("      gain stages: {}", names.join(", "));
    }
    if let Ok(bandwidth) = device.bandwidth(dir, chan) {
        println!("      bandwidth:   {:.6} MHz", bandwidth / 1e6);
    }
    match device.antennas(dir, chan) {
        Ok(antennas) => println!("      antennas:    {}", antennas.join(", ")),
        Err(e) => println!("      antennas:    <{e}>"),
    }

    print_sensors(device, SensorScope::Channel { dir, chan });
}

fn print_sensors(device: &dyn Device, scope: SensorScope) {
    let Ok(names) = device.sensor_names(scope) else {
        return;
    };
    for name in names {
        match device.sensor(scope, &name) {
            Ok(value) => println!("      sensor {name}: {value}"),
            Err(e) => println!("      sensor {name}: <{e}>"),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn cli_definition_is_valid() {
        use clap::CommandFactory;
        Cli::command().debug_assert();
    }
}
