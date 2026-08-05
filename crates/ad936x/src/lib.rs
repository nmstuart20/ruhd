//! Driver for the Analog Devices AD9361/AD9364 RF transceiver.
//!
//! Reference: `reference/uhd/host/lib/usrp/common/ad9361_driver/ad9361_device.cpp`
//! and `host/lib/usrp/common/ad9361_ctrl.cpp` — see `NOTES.md`.
//!
//! # This crate knows no board
//!
//! Everything here reaches the part through [`SpiInterface`], a trait defined in
//! this crate and implemented by whatever the part is soldered to. The B205mini
//! implements it by driving the FPGA's `spi_core_3000` at the AD9361 slave
//! select; an E310 would implement it over the SoC's SPI controller. Neither
//! board appears in this crate's dependencies, and the only things either of them
//! has to tell the AD936x are the four facts in [`ClientParams`]: which reference
//! clock pin is driven, how the data port is wired, what its timing trims are,
//! and where its RF port switching changes over.
//!
//! ```no_run
//! use std::sync::Arc;
//! use ad936x::{Ad936x, ClientParams, SpiInterface};
//!
//! # fn main() -> uhd_core::Result<()> {
//! # let spi: Arc<dyn SpiInterface> = unimplemented!();
//! # let params: ClientParams = unimplemented!();
//! let codec = Ad936x::new(spi, params);
//! println!("{}", codec.identify()?); // AD936x rev 2 (0x0a)
//! let rate = codec.init()?; // complete bring-up at 50 MHz
//! # Ok(())
//! # }
//! ```
//!
//! # A faithful port, deliberately
//!
//! The AD936x is configured by long sequences of writes whose order matters,
//! against tables of constants that were measured rather than derived. UHD's
//! driver is the artefact of that measurement: its gain tables, filter
//! coefficients, calibration thresholds and rate partition are what a decade of
//! shipped radios were characterised with. This port reproduces them exactly —
//! byte for byte, in the same order, with the same rounding, including the
//! quirks — and cites the C++ line ranges throughout. Where the reference does
//! something odd, the comment says so and the code still does it. A divergence
//! that made the driver "better" would make it a different radio, and would move
//! the question "does this behave like UHD?" out of reach.
//!
//! What is *not* copied is structure. Pure arithmetic lives in [`clocking`],
//! where it can be checked against the reference without a board; the board's
//! settings are data rather than a virtual interface ([`params`]); register
//! framing stays in [`spi`]; and failures come back as
//! [`Error`](uhd_core::Error)s rather than exceptions thrown from half-way
//! through a sequence.
//!
//! # State of the port
//!
//! Done, and testable without hardware:
//!
//! - [`spi`] — the 24-bit command word and the register file.
//! - [`regs`] — register addresses and the shadow copies of the ones that are
//!   read-modify-write in software.
//! - [`params`] — what the board tells the part about itself.
//! - [`clocking`] — the rate table, the BBPLL divide word, the DAC divider and
//!   the FIR tap budgets, as pure functions.
//! - [`fir`] — the eight coefficient tables and the indirect write sequence.
//! - [`device`] — complete initialization, RF and sample-rate tuning, manual and
//!   automatic gain control, analog filters, ADC setup, DC calibration, and RX/TX
//!   quadrature calibration.
//!
//! RSSI, temperature, test-tone, loopback, and the generic filter introspection
//! API are intentionally outside this bring-up slice.

pub mod clocking;
pub mod device;
pub mod fir;
pub mod params;
pub mod regs;
mod rfpll;
pub mod spi;
mod tables;

pub use device::{Ad936x, CLOCK_SETTLING, INIT_CLOCK_RATE, RESET_SETTLING};
pub use params::{
    BandEdges, ClientParams, ClockingMode, DigitalInterface, InterfaceDelays, TimingMode,
};
pub use regs::{ChipRegs, PRODUCT_ID_AD936X, ProductId};
pub use rfpll::RfPll;
pub use spi::{MAX_REG, Registers, SPI_NUM_BITS, SpiInterface};

/// Highest RX gain the part reports, in dB. `AD9361_MAX_GAIN`.
pub const MAX_GAIN: f64 = 89.75;

/// Slowest sample rate the part will clock, in Hz. `AD9361_MIN_CLOCK_RATE`.
///
/// Not enforced by the rate table, which has no lower arm — a slower rate is
/// rejected when the BBPLL runs out of dividers. See
/// [`BbPll::solve`](clocking::BbPll::solve).
pub const MIN_CLOCK_RATE: f64 = 220e3;

/// Fastest sample rate the part will clock, in Hz. `AD9361_MAX_CLOCK_RATE`.
pub const MAX_CLOCK_RATE: f64 = 61.44e6;

/// How far the RF frequency may move before the calibrations taken at the old
/// one stop being trusted, in Hz. `AD9361_CAL_VALID_WINDOW`.
pub const CAL_VALID_WINDOW: f64 = 100e6;

/// Narrowest analog filter bandwidth, in Hz. `AD9361_MIN_BW`.
pub const MIN_BW: f64 = 200e3;

/// Widest analog filter bandwidth, in Hz. `AD9361_MAX_BW`.
///
/// Set by the rolloff of the analog filter stage rather than by anything
/// digital.
pub const MAX_BW: f64 = 56e6;

/// RX frequency the part is brought up at, in Hz. `DEFAULT_RX_FREQ`.
pub const DEFAULT_RX_FREQ: f64 = 800e6;

/// TX frequency the part is brought up at, in Hz. `DEFAULT_TX_FREQ`.
pub const DEFAULT_TX_FREQ: f64 = 850e6;

/// Which half of the transceiver an operation applies to.
///
/// `ad9361_device_t::direction_t`.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum Direction {
    /// The receive chain.
    Rx,
    /// The transmit chain.
    Tx,
}

impl Direction {
    /// `"rx"` or `"tx"`, for messages.
    #[must_use]
    pub const fn name(self) -> &'static str {
        match self {
            Self::Rx => "rx",
            Self::Tx => "tx",
        }
    }
}

/// Which of the part's two chains an operation applies to.
///
/// `ad9361_device_t::chain_t`. An AD9364 has only [`Chain::One`] bonded out, but
/// the register fields are the AD9361's either way, and the reference programs
/// [`Chain::Both`] regardless — so this is a property of the register map rather
/// than of the part in the socket.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum Chain {
    /// RX1 or TX1.
    One,
    /// RX2 or TX2.
    Two,
    /// Both, which is what everything in the initialisation path uses.
    Both,
}

/// Receive gain-control mode.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum GainMode {
    /// Host writes the gain-table index directly.
    Manual,
    /// Slow attack automatic gain control.
    SlowAgc,
    /// Fast attack automatic gain control.
    FastAgc,
}
