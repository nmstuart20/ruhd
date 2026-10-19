//! Send a timed tone burst on TX/RX. Use a terminated port or attenuated setup.
//! `cargo run --release -p uhd-b200 --example transmit -- [serial]`

use std::time::Duration;
use uhd_core::num_complex::Complex;
use uhd_core::{
    AsyncEvent, DeviceArgs, DeviceExt, Direction, StreamArgs, TimeSpec, TxMetadata, registry,
};

fn main() -> uhd_core::Result<()> {
    uhd_b200::register();
    let mut args = DeviceArgs::new().with("type", "b200");
    if let Some(serial) = std::env::args().nth(1) {
        args.set("serial", serial);
    }
    let device = registry::open(&args)?;
    let rate = device.set_sample_rate(Direction::Tx, 0, 1e6)?;
    device.tune_to(Direction::Tx, 0, 915e6)?;
    device.set_gain(Direction::Tx, 0, None, 0.0)?;
    let mut tx = device.tx_stream::<Complex<f32>>(&StreamArgs::new())?;
    let start = device.time_now(0)? + TimeSpec::from_secs(0.25);
    let samples: Vec<_> = (0..10_000)
        .map(|i| {
            let phase = std::f32::consts::TAU * i as f32 / 16.0;
            Complex::new(0.1 * phase.cos(), 0.1 * phase.sin())
        })
        .collect();
    let mut sent = 0;
    while sent < samples.len() {
        let md = TxMetadata {
            time_spec: Some(start + TimeSpec::from_secs(sent as f64 / rate)),
            start_of_burst: sent == 0,
            end_of_burst: true,
        };
        let n = tx.send_one(&samples[sent..], Duration::from_secs(1), &md)?;
        if n == 0 {
            return Err(uhd_core::Error::Timeout(Duration::from_secs(1)));
        }
        sent += n;
    }
    let event = tx
        .recv_async_msg(Duration::from_secs(2))?
        .ok_or(uhd_core::Error::Timeout(Duration::from_secs(2)))?;
    if event.event != AsyncEvent::BurstAck {
        return Err(uhd_core::Error::device(format!(
            "TX event: {:?}",
            event.event
        )));
    }
    println!("Transmitted {sent} samples; burst acknowledged.");
    Ok(())
}
