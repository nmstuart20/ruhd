//! What the board has to tell the transceiver about itself.
//!
//! Ports `ad9361_params` from
//! `reference/uhd/host/lib/usrp/common/ad9361_driver/ad9361_client.h`.
//!
//! # Why this is data and not a trait
//!
//! UHD makes this a virtual interface — `ad9361_params` with four pure virtuals,
//! implemented by `b200_ad9361_client_t` and `b2xxmini_ad9361_client_t` in
//! `b200_impl.cpp:47-118`. Both implementations return compile-time constants,
//! and there is nowhere in the driver that calls them at a moment when the answer
//! could have changed. So here they are a struct of values, handed over once at
//! construction:
//!
//! - it cannot deadlock or fail, whereas a trait object called from inside the
//!   codec's own lock could do both;
//! - the board's settings are printable, comparable and testable, which matters
//!   because "which clocking mode did we come up in" is a real bring-up question;
//! - a board that genuinely needs to compute one of these can still do so — it
//!   just has to decide before it constructs the driver, which is the only point
//!   at which the answer is allowed to change anyway.
//!
//! The values themselves stay on the board's side of the boundary: `uhd-b200`
//! builds a [`ClientParams`] and this crate never names a board.

/// Which reference-clock path the part is wired for.
///
/// `clocking_mode_t`. The B2xx family uses [`ClockingMode::XtalN`] — the TCXO
/// drives the negative crystal pin — for every product.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum ClockingMode {
    /// `AD9361_XTAL_N_CLK_PATH`: reference into `XTALN`. One register write.
    XtalN,
    /// `AD9361_XTAL_P_CLK_PATH`: reference into `XTALP`, which also needs the
    /// four-register RFPLL reference block at `0x292`-`0x295`.
    XtalP,
}

/// Electrical standard and framing of the parallel data port.
///
/// `digital_interface_mode_t`. Every B2xx runs [`DigitalInterface::DdrFddLvcmos`];
/// LVDS is here because the reference supports it and the register sequences
/// differ, and getting it wrong is a silent bring-up failure rather than an
/// error.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum DigitalInterface {
    /// `AD9361_DDR_FDD_LVCMOS`: dual-port DDR CMOS.
    DdrFddLvcmos,
    /// `AD9361_DDR_FDD_LVDS`: dual-port DDR LVDS.
    DdrFddLvds,
}

/// Data-port timing mode.
///
/// `timing_mode_t`. Only meaningful on LVDS: CMOS is 1R1T and asking for 2R2T is
/// an error, exactly as in `set_timing_mode`.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum TimingMode {
    /// One receiver, one transmitter.
    OneROneT,
    /// Two receivers, two transmitters.
    TwoRTwoT,
}

/// Clock and data delays on the parallel port, in bit periods.
///
/// `digital_interface_delays_t`. Each field is four bits wide; the driver masks
/// with `0xF` on the way into [`RX_CLOCK_DATA_DELAY`](crate::regs::RX_CLOCK_DATA_DELAY)
/// and its TX twin, so a larger value is silently truncated rather than rejected —
/// kept faithfully, because a board that has been running with a truncated delay
/// is a board whose timing is already characterised.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq, Hash)]
pub struct InterfaceDelays {
    /// RX clock delay.
    pub rx_clk: u8,
    /// RX data delay.
    pub rx_data: u8,
    /// TX clock delay.
    pub tx_clk: u8,
    /// TX data delay.
    pub tx_data: u8,
}

/// The three band edges the RF port selection is decided against.
///
/// `frequency_band_t` and `get_band_edge`. Not used by anything in part 1 — RF
/// port selection is tuning's business — but it is part of the same client
/// interface and a board that hands over its clocking should hand over its ports
/// at the same time, rather than being asked again later.
#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct BandEdges {
    /// `AD9361_RX_BAND0`, in Hz. Below this the receiver uses one RF port.
    pub rx_band0: f64,
    /// `AD9361_RX_BAND1`, in Hz.
    pub rx_band1: f64,
    /// `AD9361_TX_BAND0`, in Hz.
    pub tx_band0: f64,
}

/// Everything the board tells the transceiver about itself.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ClientParams {
    /// Which reference clock pin the board drives.
    pub clocking_mode: ClockingMode,
    /// How the parallel data port is wired.
    pub digital_interface: DigitalInterface,
    /// Timing trims for that port, measured on the board.
    pub delays: InterfaceDelays,
    /// Where the board's RF port switching changes over.
    pub band_edges: BandEdges,
}
