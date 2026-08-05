//! Run the B2xx control-channel compatibility and scratch-register gate.

use uhd_b200::RadioCtrl;
use uhd_core::{Error, Result};
use uhd_transport::usb::{Fx3Iface, UsbFilter, probe_filtered};

fn main() -> Result<()> {
    let serial = std::env::args().nth(1);
    let filter = serial
        .as_deref()
        .map_or_else(UsbFilter::new, |s| UsbFilter::new().with_serial(s));
    let mut devices = probe_filtered(&filter)?;
    if devices.len() != 1 {
        return Err(Error::InvalidArg(format!(
            "scratch gate requires exactly one B2xx; found {}{}",
            devices.len(),
            serial.map_or_else(String::new, |s| format!(" matching serial {s}"))
        )));
    }
    let device = devices.pop().expect("length checked");
    if device.needs_firmware() {
        return Err(Error::Firmware(
            "B2xx firmware is not running; run `cargo run -p uhd-cli -- init` first".into(),
        ));
    }
    let product = device.product.ok_or_else(|| {
        Error::Device(
            "USB cannot distinguish this B200/B210; initialize through the full backend first"
                .into(),
        )
    })?;
    println!(
        "testing {} serial {}",
        product,
        device.serial().unwrap_or("<none>")
    );
    let iface = Fx3Iface::open(&device)?;
    let ctrl = RadioCtrl::open(&iface)?;
    let result = ctrl.gate(product)?;
    println!(
        "PASS: FPGA {}.{}, signature {:#010x}; {} scratch iterations, zero mismatches",
        result.fpga.major, result.fpga.minor, result.fpga.signature, result.scratch_iterations
    );
    Ok(())
}
