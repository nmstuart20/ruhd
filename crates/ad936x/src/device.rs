//! The transceiver, and the part of its bring-up sequence that gets it clocking.
//!
//! Ports `ad9361_device_t` from
//! `reference/uhd/host/lib/usrp/common/ad9361_driver/ad9361_device.cpp`.
//!
//! # What is here
//!
//! The reference's `initialize` (`1578-1844`) runs, in order: shadow-register
//! setup, chip reset, the product-ID check, a block of fixed housekeeping pokes,
//! the reference clock path, `_setup_rates(50e6)`, the data port, the interface
//! delays, AuxDAC and AuxADC setup, GPO, the ENSM transition to ALERT, and then
//! ten calibration and tuning steps.
//!
//! Everything through the interface delays is here, as
//! [`Ad936x::initialize_clocking`]. The steps below it — synthesiser charge
//! pumps, RF tuning, gain tables, analog filter calibration, DC offset and
//! quadrature — arrive with the rest of the port. So does the public
//! `set_clock_rate`, and for a reason worth stating plainly: between
//! `_setup_rates` and returning, it re-runs seven of those calibrations
//! (`1916-1942`). A `set_clock_rate` that skipped them would leave a part that
//! clocks at the new rate and receives badly, which is a worse thing to ship
//! than a missing function. [`Ad936x::setup_rates`] — the half that is a faithful
//! port today — is public, and the caller that drives it into a working radio
//! comes next.
//!
//! # State that is not in the part
//!
//! Seven registers are written from more than one place, with different owners
//! for different bits, and cannot be read back reliably; UHD shadows them and so
//! does this ([`ChipRegs`]). Alongside them sits the clock chain's own memory:
//! the last requested rate, the last core clock, and the frequencies that came
//! out. The rate cache is not an optimisation — `set_clock_rate` is called
//! repeatedly with the same value during startup, and each call that got through
//! would re-run every calibration.
//!
//! All of it lives behind one mutex, held for whole operations rather than
//! individual writes: a rate change is a sequence, and another thread's write
//! landing in the middle of it would be indistinguishable from a hardware fault.

use std::sync::{Arc, Mutex, MutexGuard, PoisonError};
use std::thread::sleep;
use std::time::Duration;

use tracing::{debug, trace};
use uhd_core::{Error, Result};

use crate::clocking::{BbPll, ChainRates, RateProfile, freq_is_nearly_equal};
use crate::fir::{setup_rx_fir, setup_tx_fir};
use crate::params::{ClientParams, ClockingMode, DigitalInterface, TimingMode};
use crate::regs;
use crate::regs::{ChipRegs, PRODUCT_ID_AD936X, ProductId};
use crate::spi::{Registers, SpiInterface};

/// How long the part needs after a soft reset before it will answer.
///
/// `initialize` sleeps 20 ms between dropping the reset bit and reading the
/// product ID. The datasheet does not name a figure; this is the reference's,
/// kept because the alternative is discovering the real one on somebody else's
/// board.
pub const RESET_SETTLING: Duration = Duration::from_millis(20);

/// How long the clock trees are given after they are enabled.
///
/// The second of `initialize`'s two 20 ms sleeps, this one after the clock path
/// is selected and before the BBPLL is tuned.
pub const CLOCK_SETTLING: Duration = Duration::from_millis(20);

/// Interval between BBPLL lock polls.
const BBPLL_LOCK_POLL: Duration = Duration::from_millis(2);

/// How many times the BBPLL lock bit is polled before giving up.
///
/// The reference's `count > 1000`, which with the 2 ms poll is a two-second
/// ceiling. A BBPLL that has not locked in two seconds is not going to.
const BBPLL_LOCK_TRIES: usize = 1000;

/// The rate `initialize` brings the part up at, in Hz.
///
/// Hardcoded in the reference (`ad9361_device.cpp:1664`) and hardcoded here.
/// Nothing in a B2xx runs at 50 MHz for long — the host sets a real rate almost
/// immediately — but the calibrations that follow initialisation are run against
/// this rate, so it is part of the sequence rather than a default.
pub const INIT_CLOCK_RATE: f64 = 50e6;

/// The driver state that is not readable from the part.
///
/// One field per member of `ad9361_device_t` that part 1 touches. The rest of
/// the reference's members — RF frequencies, gains, bandwidths, AGC modes —
/// arrive with the code that owns them.
#[derive(Debug, Clone, Copy, PartialEq)]
struct State {
    /// Shadow copies of the read-modify-write registers.
    regs: ChipRegs,
    /// `_bbpll_freq`: the BBPLL VCO frequency, in Hz.
    bbpll_freq: f64,
    /// `_adcclock_freq`: the ADC clock, in Hz.
    adc_freq: f64,
    /// `_req_clock_rate`: the last sample rate asked for, in Hz.
    req_clock_rate: f64,
    /// `_req_coreclk`: the last core clock asked of the BBPLL, in Hz.
    req_coreclk: f64,
    /// `_baseband_bw`: the rate the part is actually running at, in Hz. This is
    /// what `get_clock_rate` returns.
    baseband_bw: f64,
    /// `_tfir_factor`: TX FIR interpolation.
    tfir_factor: i32,
    /// `_rfir_factor`: RX FIR decimation.
    rfir_factor: i32,
}

impl Default for State {
    /// `ad9361_device.cpp:1582-1618`, the block `initialize` runs before it
    /// touches the part.
    fn default() -> Self {
        Self {
            regs: ChipRegs::default(),
            bbpll_freq: 0.0,
            adc_freq: 0.0,
            req_clock_rate: 0.0,
            req_coreclk: 0.0,
            baseband_bw: 0.0,
            tfir_factor: 0,
            rfir_factor: 0,
        }
    }
}

/// An AD9361 or AD9364, reached over a [`SpiInterface`].
#[derive(Debug)]
pub struct Ad936x {
    regs: Registers,
    params: ClientParams,
    state: Mutex<State>,
}

impl Ad936x {
    /// Binds the driver to a bus and to the board's settings.
    ///
    /// Touches nothing: constructing this is not an operation on the part, so a
    /// caller can build one and then decide whether to reset it.
    #[must_use]
    pub fn new(spi: Arc<dyn SpiInterface>, params: ClientParams) -> Self {
        Self {
            regs: Registers::new(spi),
            params,
            state: Mutex::new(State::default()),
        }
    }

    /// The register file, for callers that need an address this crate has not
    /// named yet.
    #[must_use]
    pub fn regs(&self) -> &Registers {
        &self.regs
    }

    /// What the board said about itself at construction.
    #[must_use]
    pub fn params(&self) -> &ClientParams {
        &self.params
    }

    /// Soft-resets the digital section and waits for it to settle.
    ///
    /// Writes 1 then 0 to bit 0 of [`SPI_CONF`](regs::SPI_CONF), which is how
    /// `initialize` opens. This does *not* reset the part's analog reset pin —
    /// on a B2xx that is a board GPIO and the backend's business — and it does
    /// not clear this driver's shadow state, because a caller who resets the
    /// part mid-session has not necessarily abandoned what it knows.
    /// [`Ad936x::initialize_clocking`] clears both.
    ///
    /// # Errors
    ///
    /// Whatever the bus reports. A reset that fails to write is worth failing
    /// on: every later register write would be landing on an unknown state.
    pub fn reset(&self) -> Result<()> {
        self.regs.poke8(regs::SPI_CONF, 0x01)?;
        self.regs.poke8(regs::SPI_CONF, 0x00)?;
        sleep(RESET_SETTLING);
        Ok(())
    }

    /// Reads the product ID without judging it.
    ///
    /// # Errors
    ///
    /// Whatever the bus reports.
    pub fn product_id(&self) -> Result<ProductId> {
        self.regs.peek8(regs::PRODUCT_ID).map(ProductId::from_raw)
    }

    /// Reads the product ID and requires it to be an AD936x's.
    ///
    /// This is the bring-up gate. A failure here is almost never a wrong part —
    /// it is the SPI path: an unconfigured FPGA, a settings-bus address that
    /// does not land on the SPI core, a clock divider the part cannot follow, or
    /// a reset line still held. The error says so, because "expected 0x8, got
    /// 0x0" on its own has sent people looking for a dead transceiver.
    ///
    /// # Errors
    ///
    /// [`Error::Device`] if the family field is not [`PRODUCT_ID_AD936X`];
    /// otherwise whatever the bus reports.
    pub fn identify(&self) -> Result<ProductId> {
        let id = self.product_id()?;
        if !id.is_ad936x() {
            return Err(Error::Device(format!(
                "ad936x product id readback is {:#04x}, expected {PRODUCT_ID_AD936X:#04x} \
                 in the top five bits; {}",
                id.raw,
                diagnose(id.raw)
            )));
        }
        Ok(id)
    }

    /// The rate the part is clocking samples at, in Hz.
    ///
    /// `get_clock_rate`, which returns `_baseband_bw`: not the rate that was
    /// requested, but the one the BBPLL's finite divide word actually produced.
    /// Zero until the clock chain has been set up.
    #[must_use]
    pub fn clock_rate(&self) -> f64 {
        self.locked().baseband_bw
    }

    /// The BBPLL VCO frequency, in Hz. Zero until the chain is set up.
    #[must_use]
    pub fn bbpll_freq(&self) -> f64 {
        self.locked().bbpll_freq
    }

    /// The ADC clock, in Hz. Zero until the chain is set up.
    #[must_use]
    pub fn adc_freq(&self) -> f64 {
        self.locked().adc_freq
    }

    /// The current shadow copies of the read-modify-write registers.
    ///
    /// Exposed because the difference between what this thinks it wrote and what
    /// the part reads back is the first thing worth looking at when a rate
    /// change does not take.
    #[must_use]
    pub fn chip_regs(&self) -> ChipRegs {
        self.locked().regs
    }

    /// Brings the part up as far as a running clock chain.
    ///
    /// `initialize`, `ad9361_device.cpp:1582-1698`: shadow state, reset, the
    /// identity gate, the fixed housekeeping block, the reference clock path,
    /// the clock chain at [`INIT_CLOCK_RATE`], the data port and the interface
    /// delays. Returns the rate the chain came up at, which is
    /// [`INIT_CLOCK_RATE`] to within the BBPLL's resolution.
    ///
    /// The part is *not* ready to receive when this returns: it has not been
    /// tuned, its gain tables are empty, its analog filters are uncalibrated and
    /// its ENSM has not been moved out of the wait state. What it does have is a
    /// sample clock, which is what everything else on the board is waiting for —
    /// on a B2xx the radio cores do not tick until this succeeds.
    ///
    /// # Errors
    ///
    /// [`Error::Device`] if the identity gate fails or the BBPLL will not lock;
    /// [`Error::InvalidArg`] if the clock chain has no solution; otherwise
    /// whatever the bus reports.
    pub fn initialize_clocking(&self) -> Result<f64> {
        *self.locked() = State::default();

        self.reset()?;
        let id = self.identify()?;
        debug!(%id, "ad936x reset and identified");

        self.housekeeping()?;
        self.select_clock_path()?;

        let rate = self.setup_rates(INIT_CLOCK_RATE)?;
        self.set_data_port()?;
        self.set_interface_delays()?;

        debug!(rate, "ad936x clock chain up");
        Ok(rate)
    }

    /// The fixed pokes between the identity gate and the clock path.
    ///
    /// `ad9361_device.cpp:1635-1642`. Master bias, bandgap trim, and the RFPLL
    /// reference scaler at REFCLK * 2. The first write is unexplained in the
    /// reference beyond the comment reproduced below, and it is reproduced too:
    /// an undocumented write in a bring-up sequence that is known to work is not
    /// a write to drop.
    fn housekeeping(&self) -> Result<()> {
        // "There is not a WAT big enough for this."
        self.regs.poke8(0x3df, 0x01)?;

        self.regs.poke8(0x2a6, 0x0e)?; // Enable master bias
        self.regs.poke8(0x2a8, 0x0e)?; // Set bandgap trim

        // Set RFPLL ref clock scale to REFCLK * 2
        self.regs.poke8(0x2ab, 0x07)?;
        self.regs.poke8(0x2ac, 0xff)?;
        Ok(())
    }

    /// Enables the clock trees on the path the board is wired for.
    ///
    /// `ad9361_device.cpp:1644-1661`. The `XTALP` case needs four more registers
    /// than the `XTALN` case; both end with the same 20 ms settling sleep.
    fn select_clock_path(&self) -> Result<()> {
        match self.params.clocking_mode {
            ClockingMode::XtalN => {
                self.regs.poke8(regs::CLOCK_ENABLE, 0x17)?;
            }
            ClockingMode::XtalP => {
                self.regs.poke8(regs::CLOCK_ENABLE, 0x07)?;
                self.regs.poke8(0x292, 0x08)?;
                self.regs.poke8(0x293, 0x80)?;
                self.regs.poke8(0x294, 0x00)?;
                self.regs.poke8(0x295, 0x14)?;
            }
        }
        sleep(CLOCK_SETTLING);
        Ok(())
    }

    /// Configures the whole clock chain for a sample rate.
    ///
    /// `_setup_rates`, `ad9361_device.cpp:1416-1573`: picks the decimation
    /// profile, tunes the BBPLL to the core clock it implies, decides the DAC
    /// divider, writes the four filter and PLL registers, and programs both FIR
    /// filters. Returns the rate the part will actually run at.
    ///
    /// # This is not `set_clock_rate`
    ///
    /// The reference wraps this in a public `set_clock_rate` that first walks the
    /// ENSM into a state where the clocks may be changed and afterwards re-runs
    /// the calibrations the new rate invalidates. Calling this on a part that is
    /// already tuned and calibrated will change its rate and leave those
    /// calibrations stale. Until the rest of the port lands, that makes this a
    /// bring-up entry point rather than a runtime one.
    ///
    /// # Errors
    ///
    /// [`Error::InvalidArg`] for a rate the chain cannot reach; [`Error::Device`]
    /// if the BBPLL will not lock; otherwise whatever the bus reports.
    pub fn setup_rates(&self, rate: f64) -> Result<f64> {
        let mut state = self.locked();

        // If we make it into this function, then we are tuning to a new rate.
        // Store the new rate.
        state.req_clock_rate = rate;
        trace!(rate, "ad936x setting up rates");

        // Set the decimation and interpolation values in the RX and TX chains.
        // This also switches filters in and out. Note that all transmitters and
        // receivers are turned on here for the calibration portion of bring-up,
        // and are switched out afterwards to reflect the user's actual antenna
        // selections.
        let profile = RateProfile::for_rate(rate)?;
        state.regs.rxfilt = profile.rxfilt;
        state.regs.txfilt = profile.txfilt;
        state.tfir_factor = profile.tfir_factor;
        state.rfir_factor = profile.rfir_factor;
        trace!(divfactor = profile.divfactor, "ad936x rate profile");

        // Tune the BBPLL to get the ADC and DAC clocks.
        let adc_freq = self.tune_bbvco(&mut state, profile.core_clock(rate))?;
        let rates = ChainRates::derive(rate, &profile, adc_freq);
        state.regs.bbpll = if rates.halve_dac {
            state.regs.bbpll | 0x08
        } else {
            state.regs.bbpll & 0xf7
        };

        // Set the dividers / interpolators in the AD936x.
        self.regs
            .poke8(regs::TX_ENABLE_AND_FILTER_CTRL, state.regs.txfilt)?;
        self.regs
            .poke8(regs::RX_ENABLE_AND_FILTER_CTRL, state.regs.rxfilt)?;
        self.regs.poke8(regs::INPUT_SELECT, state.regs.inputsel)?;
        self.regs.poke8(regs::BBPLL, state.regs.bbpll)?;

        state.baseband_bw = rates.baseband_bw;
        trace!(
            adc_freq = rates.adc_freq,
            dac_freq = rates.dac_freq,
            baseband_bw = rates.baseband_bw,
            num_tx_taps = rates.num_tx_taps,
            num_rx_taps = rates.num_rx_taps,
            "ad936x clock chain solved"
        );

        setup_tx_fir(&self.regs, rates.num_tx_taps, profile.tfir_factor)?;
        setup_rx_fir(&self.regs, rates.num_rx_taps, profile.rfir_factor)?;

        Ok(rates.baseband_bw)
    }

    /// Tunes the baseband VCO, the clock that feeds the ADCs and DACs.
    ///
    /// `_tune_bbvco`, `ad9361_device.cpp:1201-1278`. Returns the ADC clock.
    ///
    /// A request within a hertz of the last one returns the cached ADC clock
    /// without touching the part, which is the reference's behaviour and matters
    /// for more than speed: re-tuning the BBPLL means re-locking it, and a
    /// re-lock is a discontinuity in the sample clock of a running radio.
    fn tune_bbvco(&self, state: &mut State, core_clock: f64) -> Result<f64> {
        if freq_is_nearly_equal(core_clock, state.req_coreclk) {
            trace!(core_clock, "ad936x bbpll already at this rate");
            return Ok(state.adc_freq);
        }
        state.req_coreclk = core_clock;

        let pll = BbPll::solve(core_clock)?;

        self.regs.poke8(regs::BBPLL_REF_CLOCK_SCALER, 0x00)?; // REFCLK / 1 to BBPLL
        self.regs
            .poke8(regs::BBPLL_CP_CURRENT, (pll.icp_reg & 0x3f) as u8)?;

        // BBPLL loop filters, and "max accuracy" on the last. A fixed block:
        // ad9361_device.cpp:1258-1263.
        for (reg, value) in [
            (0x048, 0xe8),
            (0x049, 0x5b),
            (0x04a, 0x35),
            (0x04b, 0xe0),
            (0x04e, 0x10),
        ] {
            self.regs.poke8(reg, value)?;
        }

        self.regs
            .poke8(regs::BBPLL_NFRAC_7_0, (pll.nfrac & 0xff) as u8)?;
        self.regs
            .poke8(regs::BBPLL_NFRAC_15_8, ((pll.nfrac >> 8) & 0xff) as u8)?;
        self.regs
            .poke8(regs::BBPLL_NFRAC_23_16, ((pll.nfrac >> 16) & 0xff) as u8)?;
        self.regs.poke8(regs::BBPLL_NINT, pll.nint as u8)?;

        self.calibrate_lock_bbpll()?;

        state.regs.bbpll = (state.regs.bbpll & 0xf8) | pll.vcodiv_exp;
        state.bbpll_freq = pll.vco_freq;
        state.adc_freq = pll.adc_freq;

        Ok(pll.adc_freq)
    }

    /// Calibrates the BBPLL and waits for it to lock.
    ///
    /// `_calibrate_lock_bbpll`, `ad9361_device.cpp:257-277`. Must be run after
    /// any change to the BBPLL's divide word.
    ///
    /// # Errors
    ///
    /// [`Error::Device`] if the lock bit has not come up after
    /// [`BBPLL_LOCK_TRIES`] polls.
    fn calibrate_lock_bbpll(&self) -> Result<()> {
        self.regs.poke8(0x03f, 0x05)?; // Start the BBPLL calibration
        self.regs.poke8(0x03f, 0x01)?; // Clear the 'start' bit

        // Increase BBPLL KV and phase margin.
        self.regs.poke8(0x04c, 0x86)?;
        self.regs.poke8(0x04d, 0x01)?;
        self.regs.poke8(0x04d, 0x05)?;

        // Wait for BBPLL lock. Poll shape and counter bound are the
        // reference's, down to the check happening after the first read.
        let mut count = 0;
        while self.regs.peek8(regs::BBPLL_LOCK_STATUS)? & regs::BBPLL_LOCKED == 0 {
            if count > BBPLL_LOCK_TRIES {
                return Err(Error::Device(format!(
                    "ad936x bbpll did not lock within {:?}; the part is not receiving its \
                     reference clock, or the divide word puts the vco out of range",
                    BBPLL_LOCK_POLL * u32::try_from(BBPLL_LOCK_TRIES).unwrap_or(u32::MAX)
                )));
            }
            count += 1;
            sleep(BBPLL_LOCK_POLL);
        }
        Ok(())
    }

    /// Sets up the parallel data port for the board's digital interface.
    ///
    /// `ad9361_device.cpp:1666-1691`. FDD dual port DDR, TX forced onto one port
    /// and RX onto the other.
    ///
    /// # Errors
    ///
    /// Whatever the bus reports.
    pub fn set_data_port(&self) -> Result<()> {
        match self.params.digital_interface {
            DigitalInterface::DdrFddLvcmos => {
                // Swap I&Q on Tx, Swap I&Q on Rx, Toggle frame sync mode
                self.regs.poke8(regs::PARALLEL_PORT_CONF_1, 0xc8)?;
                self.regs.poke8(regs::PARALLEL_PORT_CONF_2, 0x00)?;
                self.regs.poke8(regs::PARALLEL_PORT_CONF_3, 0x02)?;
            }
            DigitalInterface::DdrFddLvds => {
                // Swap I&Q on Tx, Swap I&Q on Rx, Toggle frame sync mode,
                // 2R2T timing.
                self.regs.poke8(regs::PARALLEL_PORT_CONF_1, 0xcc)?;
                self.regs.poke8(regs::PARALLEL_PORT_CONF_2, 0x00)?;
                self.regs.poke8(regs::PARALLEL_PORT_CONF_3, 0x10)?;

                // LVDS specific.
                self.regs.poke8(regs::LVDS_BIAS_CTRL, 0x23)?;
                self.regs.poke8(regs::LVDS_INVERT_CTRL_1, 0xff)?;
                self.regs.poke8(regs::LVDS_INVERT_CTRL_2, 0x0f)?;
            }
        }
        Ok(())
    }

    /// Rewrites the port-mode register after a calibration pass.
    ///
    /// `ad9361_device.cpp:1800-1812`, the "cals done, set PPORT config" step
    /// that both `initialize` and `set_clock_rate` end with. The calibrations in
    /// between change this register, so it has to be put back; it is public
    /// because the code that runs those calibrations is not here yet.
    ///
    /// # Errors
    ///
    /// Whatever the bus reports.
    pub fn restore_parallel_port_config(&self) -> Result<()> {
        let conf = match self.params.digital_interface {
            DigitalInterface::DdrFddLvcmos => 0x02,
            DigitalInterface::DdrFddLvds => 0x10,
        };
        self.regs.poke8(regs::PARALLEL_PORT_CONF_3, conf)
    }

    /// Writes the board's clock and data delays.
    ///
    /// `ad9361_device.cpp:1693-1698`. These are the numbers the board's timing
    /// was characterised with; four bits each, masked rather than range-checked,
    /// exactly as the reference does it.
    ///
    /// # Errors
    ///
    /// Whatever the bus reports.
    pub fn set_interface_delays(&self) -> Result<()> {
        let delays = self.params.delays;
        let rx = ((delays.rx_clk & 0xf) << 4) | (delays.rx_data & 0xf);
        let tx = ((delays.tx_clk & 0xf) << 4) | (delays.tx_data & 0xf);
        self.regs.poke8(regs::RX_CLOCK_DATA_DELAY, rx)?;
        self.regs.poke8(regs::TX_CLOCK_DATA_DELAY, tx)?;
        Ok(())
    }

    /// Selects the data port's timing mode.
    ///
    /// `set_timing_mode`, `ad9361_device.cpp:2070-2110`. Called when the number
    /// of active channels changes: an LVDS port carries two chains' samples in
    /// the time it carries one's, so the framing has to say which it is.
    ///
    /// # Errors
    ///
    /// [`Error::NotSupported`] for 2R2T on a CMOS port, which has no such mode;
    /// otherwise whatever the bus reports.
    pub fn set_timing_mode(&self, timing_mode: TimingMode) -> Result<()> {
        let conf = match (self.params.digital_interface, timing_mode) {
            // Swap I&Q on Tx, Swap I&Q on Rx, Toggle frame sync mode.
            (_, TimingMode::OneROneT) => 0xc8,
            // As above, plus 2R2T timing.
            (DigitalInterface::DdrFddLvds, TimingMode::TwoRTwoT) => 0xcc,
            (DigitalInterface::DdrFddLvcmos, TimingMode::TwoRTwoT) => {
                return Err(Error::not_supported(
                    "ad936x 2r2t timing mode on a cmos data port",
                ));
            }
        };
        self.regs.poke8(regs::PARALLEL_PORT_CONF_1, conf)
    }

    /// The state, with a poisoned lock treated as usable.
    ///
    /// A panic while holding this leaves shadow registers that may not match the
    /// part, which is worth knowing about — but the way to find out is to read
    /// them, and refusing to hand them over would take that away.
    fn locked(&self) -> MutexGuard<'_, State> {
        self.state.lock().unwrap_or_else(PoisonError::into_inner)
    }
}

/// The likely cause of a product-ID readback that is not an AD936x's.
fn diagnose(raw: u8) -> &'static str {
    match raw {
        0x00 => {
            "all-zero usually means nothing is driving MISO — the fpga's spi core is not \
             reachable, or the part is held in reset"
        }
        0xff => {
            "all-ones usually means MISO is floating high — check that the codec's reset \
             line has been released"
        }
        _ => {
            "the spi path is answering but with the wrong data; suspect the clock divider \
              or the sampling edge"
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::params::{BandEdges, InterfaceDelays};
    use crate::spi::tests::FakeSpi;

    /// A B205mini's settings: `b200_impl.cpp:84-118`.
    fn mini_params() -> ClientParams {
        ClientParams {
            clocking_mode: ClockingMode::XtalN,
            digital_interface: DigitalInterface::DdrFddLvcmos,
            delays: InterfaceDelays {
                rx_clk: 0,
                rx_data: 0xf,
                tx_clk: 0,
                tx_data: 0xf,
            },
            band_edges: BandEdges::default(),
        }
    }

    /// A part that answers the identity gate and reports a locked BBPLL.
    fn working_part() -> Arc<FakeSpi> {
        Arc::new(FakeSpi::with_regs(&[
            (regs::PRODUCT_ID, 0x0a),
            (regs::BBPLL_LOCK_STATUS, regs::BBPLL_LOCKED),
        ]))
    }

    #[test]
    fn reset_writes_the_bit_and_then_clears_it() {
        let spi = Arc::new(FakeSpi::default());
        Ad936x::new(spi.clone(), mini_params()).reset().unwrap();

        // Two writes to 0x000, 1 then 0. The order is the whole content of a
        // reset, so it is what the test asserts.
        assert_eq!(spi.words(), vec![0x0080_0001, 0x0080_0000]);
    }

    #[test]
    fn identify_accepts_the_family_field_whatever_the_revision() {
        let spi = Arc::new(FakeSpi::with_reads(&[0x0a]));
        let id = Ad936x::new(spi, mini_params()).identify().unwrap();
        assert_eq!(id.revision(), 2);
    }

    #[test]
    fn identify_names_the_bus_rather_than_the_part_when_it_fails() {
        // The failure mode this wording exists for: a gate run before the FPGA
        // is configured reads zeros, and the useful next step is to look at the
        // bus, not at the transceiver.
        let spi = Arc::new(FakeSpi::with_reads(&[0x00]));
        let error = Ad936x::new(spi, mini_params())
            .identify()
            .unwrap_err()
            .to_string();
        assert!(error.contains("0x00"), "{error}");
        assert!(error.contains("spi core"), "{error}");
    }

    #[test]
    fn a_floating_bus_is_diagnosed_differently_from_a_dead_one() {
        let spi = Arc::new(FakeSpi::with_reads(&[0xff]));
        let error = Ad936x::new(spi, mini_params())
            .identify()
            .unwrap_err()
            .to_string();
        assert!(error.contains("reset line"), "{error}");
    }

    #[test]
    fn setting_up_rates_writes_the_bbpll_word_and_the_filter_registers() {
        let spi = working_part();
        let codec = Ad936x::new(spi.clone(), mini_params());

        assert_eq!(codec.setup_rates(50e6).unwrap(), 50e6);

        let pokes = spi.pokes();
        let value = |reg: u16| pokes.iter().rev().find(|p| p.0 == reg).map(|p| p.1);

        // The divide word for a 600 MHz core clock: Nint 30, Nfrac 0, charge
        // pump register 4. See the clocking tests for the arithmetic.
        assert_eq!(value(regs::BBPLL_NINT), Some(30));
        assert_eq!(value(regs::BBPLL_NFRAC_7_0), Some(0));
        assert_eq!(value(regs::BBPLL_NFRAC_15_8), Some(0));
        assert_eq!(value(regs::BBPLL_NFRAC_23_16), Some(0));
        assert_eq!(value(regs::BBPLL_CP_CURRENT), Some(4));

        // The filter registers come from the rate table, and the input select
        // from the shadow copy's initial value.
        assert_eq!(value(regs::RX_ENABLE_AND_FILTER_CTRL), Some(0xe6));
        assert_eq!(value(regs::TX_ENABLE_AND_FILTER_CTRL), Some(0xe2));
        assert_eq!(value(regs::INPUT_SELECT), Some(0x30));

        // BBPLL: initial 0x02, VCO divider exponent 1 in bits 2:0, and bit 3
        // set because a 600 MHz ADC clock is above the 336 MHz threshold.
        assert_eq!(value(regs::BBPLL), Some(0x09));
        assert_eq!(codec.chip_regs().bbpll, 0x09);

        assert_eq!(codec.clock_rate(), 50e6);
        assert_eq!(codec.adc_freq(), 600e6);
        assert_eq!(codec.bbpll_freq(), 1200e6);
    }

    #[test]
    fn the_filter_registers_are_written_before_the_firs_are_programmed() {
        // Ordering matters: the FIR programming clocks off the rates these four
        // registers select, so a port that wrote them afterwards would be
        // loading taps at the wrong clock.
        let spi = working_part();
        Ad936x::new(spi.clone(), mini_params())
            .setup_rates(50e6)
            .unwrap();

        let pokes = spi.pokes();
        let bbpll = pokes.iter().position(|p| p.0 == regs::BBPLL).unwrap();
        let first_fir = pokes
            .iter()
            .position(|p| p.0 == regs::TX_FIR_BASE + 5)
            .unwrap();
        assert!(bbpll < first_fir, "{bbpll} should precede {first_fir}");
    }

    #[test]
    fn a_repeated_rate_does_not_retune_the_bbpll() {
        // `_tune_bbvco`'s cache. A second pass at the same rate re-writes the
        // filter registers and re-programs the FIRs, but must not touch the
        // divide word — a re-lock is a discontinuity in a running radio's
        // sample clock.
        let spi = working_part();
        let codec = Ad936x::new(spi.clone(), mini_params());

        codec.setup_rates(50e6).unwrap();
        let before = spi.pokes().len();
        codec.setup_rates(50e6).unwrap();
        let after = spi.pokes();

        assert!(
            !after[before..].iter().any(|p| p.0 == regs::BBPLL_NINT),
            "the second pass re-tuned the bbpll"
        );
        assert!(
            after[before..]
                .iter()
                .any(|p| p.0 == regs::RX_ENABLE_AND_FILTER_CTRL),
            "the second pass should still write the filter registers"
        );
    }

    #[test]
    fn the_cache_is_on_the_core_clock_and_not_on_the_rate() {
        // The one-hertz window is applied to the BBPLL's input, so at a
        // divfactor of 12 a half-hertz change in the sample rate is a six-hertz
        // change there and does re-tune. The reference has a second, coarser
        // cache on the requested rate itself, but it lives in `set_clock_rate`
        // — which needs the calibration chain, and is not ported yet.
        let spi = working_part();
        let codec = Ad936x::new(spi.clone(), mini_params());

        codec.setup_rates(50e6).unwrap();
        let before = spi.pokes().len();
        codec.setup_rates(50e6 + 0.5).unwrap();

        assert!(
            spi.pokes()[before..]
                .iter()
                .any(|p| p.0 == regs::BBPLL_NINT),
            "a six-hertz move of the core clock should re-tune the bbpll"
        );
    }

    #[test]
    fn a_different_rate_does_retune_the_bbpll() {
        let spi = working_part();
        let codec = Ad936x::new(spi.clone(), mini_params());

        codec.setup_rates(50e6).unwrap();
        let before = spi.pokes().len();
        codec.setup_rates(30.72e6).unwrap();

        let after = spi.pokes();
        let nint = after[before..]
            .iter()
            .find(|p| p.0 == regs::BBPLL_NINT)
            .map(|p| p.1);
        // 30.72 MHz * 16 = 491.52 MHz core, VCO 983.04 MHz: Nint 24.
        assert_eq!(nint, Some(24));
        assert!(freq_is_nearly_equal(codec.clock_rate(), 30.72e6));
    }

    #[test]
    fn a_bbpll_that_never_locks_is_a_device_error() {
        // The lock bit reads back clear. This is the slow path — a thousand
        // polls — so it is checked once, here, on the shortest sequence that
        // reaches it.
        let spi = Arc::new(FakeSpi::with_regs(&[(regs::PRODUCT_ID, 0x0a)]));
        let codec = Ad936x::new(spi, mini_params());

        let error = codec.setup_rates(50e6).unwrap_err().to_string();
        assert!(error.contains("did not lock"), "{error}");
        assert!(error.contains("reference clock"), "{error}");
    }

    #[test]
    fn a_rate_the_chain_cannot_reach_is_rejected_before_anything_is_written() {
        let spi = working_part();
        let codec = Ad936x::new(spi.clone(), mini_params());

        assert!(codec.setup_rates(100e6).is_err());
        assert!(spi.pokes().is_empty(), "a rejected rate wrote registers");
        assert_eq!(codec.clock_rate(), 0.0);
    }

    #[test]
    fn the_cmos_data_port_and_delays_are_the_reference_bytes() {
        let spi = working_part();
        let codec = Ad936x::new(spi.clone(), mini_params());

        codec.set_data_port().unwrap();
        codec.set_interface_delays().unwrap();

        assert_eq!(
            spi.pokes(),
            vec![
                (0x010, 0xc8),
                (0x011, 0x00),
                (0x012, 0x02),
                // rx_data and tx_data are 0xF on every B2xx; the clock delays are 0.
                (0x006, 0x0f),
                (0x007, 0x0f),
            ]
        );
    }

    #[test]
    fn the_lvds_data_port_writes_three_registers_the_cmos_one_does_not() {
        let spi = working_part();
        let params = ClientParams {
            digital_interface: DigitalInterface::DdrFddLvds,
            ..mini_params()
        };
        let codec = Ad936x::new(spi.clone(), params);

        codec.set_data_port().unwrap();
        assert_eq!(
            spi.pokes(),
            vec![
                (0x010, 0xcc),
                (0x011, 0x00),
                (0x012, 0x10),
                (0x03c, 0x23),
                (0x03d, 0xff),
                (0x03e, 0x0f),
            ]
        );
    }

    #[test]
    fn two_by_two_timing_is_an_lvds_only_mode() {
        let spi = working_part();
        let cmos = Ad936x::new(spi.clone(), mini_params());
        assert!(cmos.set_timing_mode(TimingMode::TwoRTwoT).is_err());
        cmos.set_timing_mode(TimingMode::OneROneT).unwrap();
        assert_eq!(spi.pokes(), vec![(0x010, 0xc8)]);

        let spi = working_part();
        let lvds = Ad936x::new(
            spi.clone(),
            ClientParams {
                digital_interface: DigitalInterface::DdrFddLvds,
                ..mini_params()
            },
        );
        lvds.set_timing_mode(TimingMode::TwoRTwoT).unwrap();
        assert_eq!(spi.pokes(), vec![(0x010, 0xcc)]);
    }

    #[test]
    fn the_post_calibration_port_config_is_the_mode_byte_alone() {
        let spi = working_part();
        Ad936x::new(spi.clone(), mini_params())
            .restore_parallel_port_config()
            .unwrap();
        assert_eq!(spi.pokes(), vec![(0x012, 0x02)]);
    }

    #[test]
    fn initialisation_runs_the_reference_sequence_in_order() {
        let spi = working_part();
        let codec = Ad936x::new(spi.clone(), mini_params());

        assert_eq!(codec.initialize_clocking().unwrap(), INIT_CLOCK_RATE);

        // The order of the prologue is the reference's, and it is an order that
        // matters: the clock path cannot be selected before the part has been
        // reset, and the BBPLL cannot be tuned before the clock trees are on.
        let pokes = spi.pokes();
        let at = |reg: u16| pokes.iter().position(|p| p.0 == reg).unwrap();
        assert_eq!(pokes[0], (regs::SPI_CONF, 0x01));
        assert_eq!(pokes[1], (regs::SPI_CONF, 0x00));
        assert_eq!(pokes[2], (0x3df, 0x01));
        assert!(at(0x2a6) < at(0x2ab));
        assert!(at(0x2ac) < at(regs::CLOCK_ENABLE));
        assert!(at(regs::CLOCK_ENABLE) < at(regs::BBPLL_NINT));
        assert!(at(regs::BBPLL_NINT) < at(regs::PARALLEL_PORT_CONF_1));
        assert!(at(regs::PARALLEL_PORT_CONF_1) < at(regs::RX_CLOCK_DATA_DELAY));

        // An XTALN board writes one clock register and none of the XTALP block.
        assert_eq!(
            pokes.iter().filter(|p| p.0 == regs::CLOCK_ENABLE).count(),
            1
        );
        assert!(!pokes.iter().any(|p| p.0 == 0x292));

        assert_eq!(codec.clock_rate(), INIT_CLOCK_RATE);
    }

    #[test]
    fn initialisation_starts_from_the_reference_shadow_state() {
        let spi = working_part();
        let codec = Ad936x::new(spi, mini_params());
        codec.setup_rates(30.72e6).unwrap();
        assert_ne!(codec.chip_regs(), ChipRegs::default());

        // A re-initialisation is a fresh start, including the rate cache: the
        // part has been reset, so nothing the driver believed about it holds.
        codec.initialize_clocking().unwrap();
        assert_eq!(codec.chip_regs().inputsel, 0x30);
        assert_eq!(codec.chip_regs().bbftune_config, 0x1e);
        assert_eq!(codec.clock_rate(), INIT_CLOCK_RATE);
    }

    #[test]
    fn an_xtal_p_board_writes_the_extra_reference_block() {
        let spi = working_part();
        let codec = Ad936x::new(
            spi.clone(),
            ClientParams {
                clocking_mode: ClockingMode::XtalP,
                ..mini_params()
            },
        );
        codec.initialize_clocking().unwrap();

        let pokes = spi.pokes();
        let clock = pokes
            .iter()
            .position(|p| p.0 == regs::CLOCK_ENABLE)
            .unwrap();
        assert_eq!(pokes[clock], (regs::CLOCK_ENABLE, 0x07));
        assert_eq!(
            pokes[clock + 1..clock + 5],
            [(0x292, 0x08), (0x293, 0x80), (0x294, 0x00), (0x295, 0x14),]
        );
    }
}
