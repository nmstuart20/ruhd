//! Reach the AD936x over SPI and read its product ID.
//!
//! ```sh
//! cargo run -p uhd-b200 --example codec_id -- [SERIAL] [--auto-flash]
//! ```
//!
//! With `--auto-flash`, a device whose FPGA is out of the supported window is
//! reloaded from the image cache first, which makes this runnable straight after
//! `ruhd init --no-fpga`.
//!
//! # What passing this proves
//!
//! This is the first gate in the workspace that says anything about the radio
//! rather than about the FPGA. It exercises, in one line, the whole path from
//! the host to the transceiver:
//!
//! ```text
//!   USB bulk endpoints  ->  CHDR control packet
//!     -> local core, stream id 0x40
//!       -> SR_CORE_MISC  : release the codec's reset pin
//!       -> SR_CORE_SPI   : spi_core_3000, clocked at 1 MHz
//!         -> AD936x slave 0, 24-bit command word
//!           -> register 0x037
//! ```
//!
//! Every layer of that is a place the bring-up can fail, and the reason this is
//! worth running before anything else is that the failures are distinguishable:
//! an unconfigured FPGA times out on the bus, a wrong SPI address reads `0x00`,
//! a codec still held in reset reads `0xff`.
//!
//! # It does not need the radio cores
//!
//! Unlike `scratch_gate`, this runs on a device that has never had its codec
//! initialised — which is the point. The SPI master and the reset line are both
//! on the FPGA's local core, precisely because the part they reach is the one
//! that supplies the radio cores' clock.

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
        "{} serial {}: fpga compat {} on the {} shim",
        session.product(),
        session
            .report()
            .eeprom
            .serial
            .as_deref()
            .unwrap_or("<none>"),
        session.compat().major(),
        session.shim().codename(),
    );

    let gate = session.codec_gate()?;
    println!("PASS: {gate}");
    println!(
        "  register 0x037 read back {:#04x}: family {:#04x}, silicon revision {}",
        gate.product_id.raw,
        gate.product_id.family(),
        gate.product_id.revision(),
    );
    Ok(())
}
