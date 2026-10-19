//! Receive a finite burst through the generic device API.
//!
//! Run `cargo run -p uhd-b200 --example receive -- [serial]` after `ruhd init`.

use std::time::Duration;
use uhd_core::num_complex::Complex;
use uhd_core::{DeviceArgs, DeviceExt, Direction, RxMetadata, StreamArgs, StreamCmd, registry};

fn main() -> uhd_core::Result<()> {
    uhd_b200::register();
    let mut args = DeviceArgs::new().with("type", "b200");
    if let Some(serial) = std::env::args().nth(1) {
        args.set("serial", serial);
    }
    let device = registry::open(&args)?;
    device.set_sample_rate(Direction::Rx, 0, 1e6)?;
    device.tune_to(Direction::Rx, 0, 98e6)?;
    device.set_gain(Direction::Rx, 0, None, 40.0)?;
    let mut rx = device.rx_stream::<Complex<f32>>(&StreamArgs::new())?;
    rx.issue_stream_cmd(&StreamCmd::num_samps(10_000))?;
    let mut samples = vec![Complex::new(0.0, 0.0); 4096];
    let mut total = 0;
    loop {
        let mut md = RxMetadata::default();
        total += rx.recv_one(&mut samples, Duration::from_secs(1), &mut md)?;
        md.check()?;
        if md.end_of_burst {
            break;
        }
    }
    println!("Received {total} samples from {}", device.info());
    Ok(())
}
