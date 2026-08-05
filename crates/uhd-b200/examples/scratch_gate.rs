//! Negotiate FPGA compatibility, then run the control-channel scratch gate.
//!
//! ```sh
//! cargo run -p uhd-b200 --example scratch_gate -- [SERIAL] [--auto-flash]
//! ```
//!
//! With `--auto-flash`, a device whose FPGA is out of the supported window is
//! reloaded from the image cache first, which is what makes this runnable
//! straight after `ruhd init --no-fpga`.
//!
//! # The gate needs the codec clocking
//!
//! Everything up to and including the compatibility read is on the FPGA's local
//! core and works as soon as the bitstream is in. The scratch registers are on
//! a *radio* core, which runs off the AD936x's clock — and the codec
//! initialisation sequence that starts that clock is not ported yet. Until it
//! is, run a stock `uhd_usrp_probe` first; without that the gate reports the
//! full two-second ack timeout, which is a missing driver rather than a
//! compatibility problem.
//!
//! For a gate that runs today on an untouched device, see `codec_id`: it stays
//! on the local core, which is exactly where the part that supplies this one's
//! clock is reached.

use uhd_b200::{OpenOptions, open};
use uhd_core::Result;

fn main() -> Result<()> {
    let mut options = OpenOptions::new();
    for arg in std::env::args().skip(1) {
        if arg == "--auto-flash" {
            options = options.with_auto_flash(true);
        } else {
            options = options.with_serial(arg);
        }
    }

    let session = open(&options)?;
    if let Some(flashed) = session.flashed() {
        println!("{flashed}");
    }
    println!(
        "testing {} serial {}: fpga compat {} on the {} shim, {} radio chain(s)",
        session.product(),
        session
            .report()
            .eeprom
            .serial
            .as_deref()
            .unwrap_or("<none>"),
        session.compat().major(),
        session.shim().codename(),
        session.num_radio_chains()?,
    );

    let result = session.ctrl().gate()?;
    println!(
        "PASS: FPGA {}.{}, signature {:#010x}; {} scratch iterations, zero mismatches",
        result.fpga.major, result.fpga.minor, result.fpga.signature, result.scratch_iterations
    );
    Ok(())
}
