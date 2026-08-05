//! Complete AD9361/AD9364 bring-up, tuning, gain, filters, and calibration.
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
//! [`Ad936x::init`] reproduces that path end to end. Runtime
//! [`Ad936x::set_sample_rate`] performs the ENSM transitions and repeats the
//! seven rate-dependent calibrations; [`Ad936x::tune`] repeats the
//! frequency-dependent subset after a 100 MHz move.
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
use crate::rfpll::{RfPll, SynthConfig};
use crate::spi::{Registers, SpiInterface};
use crate::tables::{
    GAIN_TABLE_1300_TO_4000_MHZ, GAIN_TABLE_4000_TO_6000_MHZ, GAIN_TABLE_SUB_1300_MHZ,
};
use crate::{CAL_VALID_WINDOW, Chain, Direction, GainMode, MAX_BW, MAX_GAIN, MIN_BW};

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
/// The fields in `ad9361_device_t` that cannot be recovered reliably from the
/// register file.
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
    rx_freq: f64,
    tx_freq: f64,
    req_rx_freq: f64,
    req_tx_freq: f64,
    last_rx_cal_freq: f64,
    last_tx_cal_freq: f64,
    rx_analog_bw: f64,
    tx_analog_bw: f64,
    rx_bb_lp_bw: f64,
    tx_bb_lp_bw: f64,
    rx_tia_lp_bw: f64,
    tx_sec_lp_bw: f64,
    rx_bbf_tunediv: u16,
    curr_gain_table: u8,
    rx_gain: [f64; 2],
    tx_gain: [f64; 2],
    agc_mode: [GainMode; 2],
    agc_enabled: [bool; 2],
    use_dc_offset_tracking: bool,
    use_iq_balance_tracking: bool,
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
            rx_freq: crate::DEFAULT_RX_FREQ,
            tx_freq: crate::DEFAULT_TX_FREQ,
            req_rx_freq: 0.0,
            req_tx_freq: 0.0,
            last_rx_cal_freq: 0.0,
            last_tx_cal_freq: 0.0,
            rx_analog_bw: 0.0,
            tx_analog_bw: 0.0,
            rx_bb_lp_bw: 0.0,
            tx_bb_lp_bw: 0.0,
            rx_tia_lp_bw: 0.0,
            tx_sec_lp_bw: 0.0,
            rx_bbf_tunediv: 0,
            curr_gain_table: 0,
            rx_gain: [0.0; 2],
            tx_gain: [0.0; 2],
            agc_mode: [GainMode::SlowAgc; 2],
            agc_enabled: [false; 2],
            use_dc_offset_tracking: true,
            use_iq_balance_tracking: true,
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
        self.setup_rates_inner(&mut state, rate)
    }

    fn setup_rates_inner(&self, state: &mut State, rate: f64) -> Result<f64> {
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
        let adc_freq = self.tune_bbvco(state, profile.core_clock(rate))?;
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
    /// between change this register, so it has to be put back.
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

    /// Current exact RX or TX LO frequency.
    #[must_use]
    pub fn freq(&self, direction: Direction) -> f64 {
        let state = self.locked();
        match direction {
            Direction::Rx => state.rx_freq,
            Direction::Tx => state.tx_freq,
        }
    }

    /// Tune an RFPLL and run the frequency-dependent calibrations UHD runs.
    pub fn tune(&self, direction: Direction, value: f64) -> Result<f64> {
        let mut state = self.locked();
        let (requested, current, last_cal) = match direction {
            Direction::Rx => (state.req_rx_freq, state.rx_freq, state.last_rx_cal_freq),
            Direction::Tx => (state.req_tx_freq, state.tx_freq, state.last_tx_cal_freq),
        };
        if freq_is_nearly_equal(value, requested) {
            return Ok(current);
        }

        let was_fdd = self.regs.peek8(0x017)? & 0x0f != 5;
        if was_fdd {
            self.regs.poke8(0x014, 0x01)?;
        }

        let tuned = self.tune_helper(&mut state, direction, value)?;
        if direction == Direction::Rx {
            self.program_gain_table(&mut state)?;
        }
        self.reprogram_gains(&mut state)?;

        if (last_cal - tuned).abs() > CAL_VALID_WINDOW {
            match direction {
                Direction::Rx => {
                    self.calibrate_rf_dc_offset(&state)?;
                    if !state.use_iq_balance_tracking {
                        self.calibrate_rx_quadrature(&mut state)?;
                    }
                    if state.use_dc_offset_tracking {
                        self.configure_bb_dc_tracking(true)?;
                    }
                    state.last_rx_cal_freq = tuned;
                }
                Direction::Tx => {
                    self.calibrate_tx_quadrature(&mut state)?;
                    state.last_tx_cal_freq = tuned;
                }
            }
            if state.use_iq_balance_tracking {
                self.configure_rx_iq_tracking(true)?;
            }
        }

        if was_fdd {
            self.regs.poke8(0x014, 0x21)?;
        }
        Ok(match direction {
            Direction::Rx => state.rx_freq,
            Direction::Tx => state.tx_freq,
        })
    }

    fn tune_helper(&self, state: &mut State, direction: Direction, value: f64) -> Result<f64> {
        let pll = RfPll::solve(value)?;
        match direction {
            Direction::Rx => {
                state.req_rx_freq = value;
                let edges = self.params.band_edges;
                state.regs.inputsel = (state.regs.inputsel & 0xc0)
                    | if value < edges.rx_band0 {
                        0x30
                    } else if value < edges.rx_band1 {
                        0x0c
                    } else if value <= 6e9 {
                        0x03
                    } else {
                        return Err(Error::invalid_arg(format!("ad936x rx frequency {value}")));
                    };
                self.regs.poke8(0x004, state.regs.inputsel)?;
                state.regs.vcodivs = (state.regs.vcodivs & 0xf0) | pll.divider_exp;
                self.setup_synth(Direction::Rx, pll.vco_freq)?;
                self.write_rfpll_word(0x230, pll)?;
                self.regs.poke8(0x005, state.regs.vcodivs)?;
                sleep(Duration::from_millis(2));
                if self.regs.peek8(0x247)? & 0x02 == 0 {
                    return Err(Error::device("ad936x rx pll did not lock"));
                }
                state.rx_freq = pll.lo_freq;
            }
            Direction::Tx => {
                state.req_tx_freq = value;
                let edge = self.params.band_edges.tx_band0;
                if value < edge {
                    state.regs.inputsel |= 0x40;
                } else if value <= 6e9 {
                    state.regs.inputsel &= 0xbf;
                } else {
                    return Err(Error::invalid_arg(format!("ad936x tx frequency {value}")));
                }
                self.regs.poke8(0x004, state.regs.inputsel)?;
                state.regs.vcodivs = (state.regs.vcodivs & 0x0f) | (pll.divider_exp << 4);
                self.setup_synth(Direction::Tx, pll.vco_freq)?;
                self.write_rfpll_word(0x270, pll)?;
                self.regs.poke8(0x005, state.regs.vcodivs)?;
                sleep(Duration::from_millis(2));
                if self.regs.peek8(0x287)? & 0x02 == 0 {
                    return Err(Error::device("ad936x tx pll did not lock"));
                }
                state.tx_freq = pll.lo_freq;
            }
        }
        Ok(pll.lo_freq)
    }

    fn write_rfpll_word(&self, base: u16, pll: RfPll) -> Result<()> {
        self.regs.poke8(base + 3, (pll.nfrac & 0xff) as u8)?;
        self.regs.poke8(base + 4, ((pll.nfrac >> 8) & 0xff) as u8)?;
        self.regs
            .poke8(base + 5, ((pll.nfrac >> 16) & 0xff) as u8)?;
        self.regs.poke8(base + 2, (pll.nint >> 8) as u8)?;
        self.regs.poke8(base + 1, (pll.nint & 0xff) as u8)
    }

    fn setup_synth(&self, direction: Direction, vco_freq: f64) -> Result<()> {
        let SynthConfig(c) = SynthConfig::for_vco(vco_freq);
        let base = match direction {
            Direction::Rx => 0x230,
            Direction::Tx => 0x270,
        };
        for (offset, value) in [
            (0x0a, 0x40 | c[0]),
            (0x09, 0xc0 | c[1]),
            (0x12, c[2] | (c[3] << 3)),
            (0x08, c[4] << 3),
            (0x15, 0x00),
            (0x21, c[5]),
            (0x20, 0x70),
            (0x0b, 0x80 | c[6]),
            (0x0e, c[8] | (c[7] << 4)),
            (0x0f, c[10] | (c[9] << 4)),
            (0x10, c[11]),
        ] {
            self.regs.poke8(base + offset, value)?;
        }
        Ok(())
    }

    /// Set a manual RX gain-table index or TX gain in dB.
    pub fn set_gain(&self, direction: Direction, chain: Chain, value: f64) -> Result<f64> {
        if !value.is_finite() {
            return Err(Error::invalid_arg("ad936x gain must be finite"));
        }
        let mut state = self.locked();
        self.set_gain_inner(&mut state, direction, chain, value)
    }

    fn set_gain_inner(
        &self,
        state: &mut State,
        direction: Direction,
        chain: Chain,
        value: f64,
    ) -> Result<f64> {
        match direction {
            Direction::Rx => {
                let index = (value as i32).clamp(0, 76) as u8;
                if matches!(chain, Chain::One | Chain::Both) {
                    state.rx_gain[0] = value;
                    self.regs.poke8(0x109, index)?;
                }
                if matches!(chain, Chain::Two | Chain::Both) {
                    state.rx_gain[1] = value;
                    self.regs.poke8(0x10c, index)?;
                }
                Ok(f64::from(index))
            }
            Direction::Tx => {
                let attenuation = MAX_GAIN - value;
                if !(0.0..=511.75 / 4.0).contains(&attenuation) {
                    return Err(Error::OutOfRange {
                        what: "ad936x tx gain".into(),
                        value,
                        start: MAX_GAIN - 127.75,
                        stop: MAX_GAIN,
                    });
                }
                let word = (attenuation * 4.0) as u16;
                if matches!(chain, Chain::One | Chain::Both) {
                    state.tx_gain[0] = value;
                    self.regs.poke8(0x073, word as u8)?;
                    self.regs.poke8(0x074, ((word >> 8) & 1) as u8)?;
                }
                if matches!(chain, Chain::Two | Chain::Both) {
                    state.tx_gain[1] = value;
                    self.regs.poke8(0x075, word as u8)?;
                    self.regs.poke8(0x076, ((word >> 8) & 1) as u8)?;
                }
                self.regs.poke8(0x07c, 0x40)?;
                Ok(MAX_GAIN - f64::from(word) / 4.0)
            }
        }
    }

    fn reprogram_gains(&self, state: &mut State) -> Result<()> {
        for (direction, values) in [
            (Direction::Rx, state.rx_gain),
            (Direction::Tx, state.tx_gain),
        ] {
            self.set_gain_inner(state, direction, Chain::One, values[0])?;
            self.set_gain_inner(state, direction, Chain::Two, values[1])?;
        }
        Ok(())
    }

    fn program_mixer_gm_subtable(&self) -> Result<()> {
        const GAIN: [u8; 16] = [
            0x78, 0x74, 0x70, 0x6c, 0x68, 0x64, 0x60, 0x5c, 0x58, 0x54, 0x50, 0x4c, 0x48, 0x30,
            0x18, 0x00,
        ];
        const GM: [u8; 16] = [
            0x00, 0x0d, 0x15, 0x1b, 0x21, 0x25, 0x29, 0x2c, 0x2f, 0x31, 0x33, 0x34, 0x35, 0x3a,
            0x3d, 0x3e,
        ];
        self.regs.poke8(0x13f, 0x02)?;
        for (slot, (&gain, &gm)) in (0u8..16).rev().zip(GAIN.iter().zip(GM.iter())) {
            self.regs.poke8(0x138, slot)?;
            self.regs.poke8(0x139, gain)?;
            self.regs.poke8(0x13a, 0x00)?;
            self.regs.poke8(0x13b, gm)?;
            self.regs.poke8(0x13f, 0x06)?;
            self.regs.poke8(0x13c, 0x00)?;
            self.regs.poke8(0x13c, 0x00)?;
        }
        self.regs.poke8(0x13f, 0x02)?;
        self.regs.poke8(0x13c, 0x00)?;
        self.regs.poke8(0x13c, 0x00)?;
        self.regs.poke8(0x13f, 0x00)
    }

    fn program_gain_table(&self, state: &mut State) -> Result<()> {
        let (number, table) = if state.rx_freq < 1.3e9 {
            (1, &GAIN_TABLE_SUB_1300_MHZ)
        } else if state.rx_freq < 4e9 {
            (2, &GAIN_TABLE_1300_TO_4000_MHZ)
        } else if state.rx_freq <= 6e9 {
            (3, &GAIN_TABLE_4000_TO_6000_MHZ)
        } else {
            return Err(Error::invalid_arg(format!(
                "ad936x rx frequency {} has no gain table",
                state.rx_freq
            )));
        };
        if state.curr_gain_table == number {
            return Ok(());
        }
        state.curr_gain_table = number;

        self.regs.poke8(0x137, 0x1a)?;
        for index in 0u8..91 {
            let row = table.get(usize::from(index)).copied().unwrap_or([0; 3]);
            self.regs.poke8(0x130, index)?;
            self.regs.poke8(0x131, row[0])?;
            self.regs.poke8(0x132, row[1])?;
            self.regs.poke8(0x133, row[2])?;
            self.regs.poke8(0x137, 0x1e)?;
            self.regs.poke8(0x134, 0x00)?;
            self.regs.poke8(0x134, 0x00)?;
        }
        self.regs.poke8(0x137, 0x1a)?;
        self.regs.poke8(0x134, 0x00)?;
        self.regs.poke8(0x134, 0x00)?;
        self.regs.poke8(0x137, 0x00)
    }

    fn setup_gain_control(&self, agc: bool) -> Result<()> {
        const MANUAL: &[(u16, u8)] = &[
            (0x0fa, 0xe0),
            (0x0fb, 0x08),
            (0x0fc, 0x23),
            (0x0fd, 0x4c),
            (0x0fe, 0x44),
            (0x100, 0x6f),
            (0x104, 0x2f),
            (0x105, 0x3a),
            (0x107, 0x31),
            (0x108, 0x39),
            (0x109, 0x23),
            (0x10a, 0x58),
            (0x10b, 0x00),
            (0x10c, 0x23),
            (0x10d, 0x18),
            (0x10e, 0x00),
            (0x114, 0x30),
            (0x11a, 0x27),
            (0x081, 0x00),
        ];
        const AGC: &[(u16, u8)] = &[
            (0x0fb, 0x08),
            (0x0fc, 0x23),
            (0x0fd, 0x4c),
            (0x0fe, 0x44),
            (0x100, 0x6f),
            (0x101, 0x0a),
            (0x103, 0x08),
            (0x104, 0x2f),
            (0x105, 0x3a),
            (0x106, 0x22),
            (0x107, 0x2b),
            (0x108, 0x31),
            (0x111, 0x0a),
            (0x11a, 0x1c),
            (0x120, 0x0c),
            (0x121, 0x44),
            (0x122, 0x44),
            (0x123, 0x11),
            (0x124, 0xf5),
            (0x125, 0x3b),
            (0x128, 0x03),
            (0x129, 0x56),
            (0x12a, 0x22),
        ];
        for &(reg, value) in if agc { AGC } else { MANUAL } {
            self.regs.poke8(reg, value)?;
        }
        Ok(())
    }

    /// Select the slow or fast AGC algorithm used when AGC is enabled.
    pub fn set_agc_mode(&self, chain: Chain, mode: GainMode) -> Result<()> {
        let mut state = self.locked();
        for index in chain_indices(chain) {
            state.agc_mode[index] = mode;
            if state.agc_enabled[index] {
                self.setup_agc(index, mode)?;
            }
        }
        Ok(())
    }

    /// Enable or disable receive AGC on a chain.
    pub fn set_agc(&self, chain: Chain, enable: bool) -> Result<()> {
        let mut state = self.locked();
        for index in chain_indices(chain) {
            state.agc_enabled[index] = enable;
            self.setup_agc(
                index,
                if enable {
                    state.agc_mode[index]
                } else {
                    GainMode::Manual
                },
            )?;
        }
        Ok(())
    }

    fn setup_agc(&self, index: usize, mode: GainMode) -> Result<()> {
        let previous = self.regs.peek8(0x0fa)? & 0x0f;
        let shift = if index == 0 { 0 } else { 2 };
        let bits = match mode {
            GainMode::Manual => 0,
            GainMode::SlowAgc => 2,
            GainMode::FastAgc => 1,
        };
        self.regs.poke8(
            0x0fa,
            (self.regs.peek8(0x0fa)? & !(3 << shift)) | (bits << shift),
        )?;
        let current = self.regs.peek8(0x0fa)? & 0x0f;
        if (previous == 0) != (current == 0) {
            self.setup_gain_control(current != 0)?;
        }
        Ok(())
    }

    /// Configure the RX or TX analog low-pass chain for an RF bandwidth.
    pub fn set_bandwidth(&self, direction: Direction, rf_bw: f64) -> Result<f64> {
        if !rf_bw.is_finite() {
            return Err(Error::invalid_arg("ad936x bandwidth must be finite"));
        }
        let mut state = self.locked();
        self.set_bandwidth_inner(&mut state, direction, rf_bw)
    }

    fn set_bandwidth_inner(
        &self,
        state: &mut State,
        direction: Direction,
        rf_bw: f64,
    ) -> Result<f64> {
        let clipped = rf_bw.clamp(MIN_BW, MAX_BW);
        match direction {
            Direction::Rx => {
                state.rx_bb_lp_bw = self.calibrate_baseband_rx_filter(state, clipped)?;
                state.rx_tia_lp_bw = self.calibrate_rx_tias(state, clipped)?;
                state.rx_analog_bw = clipped;
            }
            Direction::Tx => {
                state.tx_bb_lp_bw = self.calibrate_baseband_tx_filter(state, clipped)?;
                state.tx_sec_lp_bw = self.calibrate_secondary_tx_filter(state, clipped)?;
                state.tx_analog_bw = clipped;
            }
        }
        Ok(clipped)
    }

    fn one_sided_bw(&self, state: &State, requested: f64, low: f64, high: f64) -> f64 {
        (requested / 2.0)
            .min(state.baseband_bw / 2.0)
            .clamp(low, high)
    }

    fn calibrate_baseband_rx_filter(&self, state: &mut State, rf_bw: f64) -> Result<f64> {
        let bbbw = self.one_sided_bw(state, rf_bw, 0.143e6, 28e6);
        let tune_clock = 1.4 * bbbw * 2.0 * std::f64::consts::PI / std::f64::consts::LN_2;
        state.rx_bbf_tunediv = ((state.bbpll_freq / tune_clock).ceil() as u16).min(511);
        state.regs.bbftune_config =
            (state.regs.bbftune_config & 0xfe) | ((state.rx_bbf_tunediv >> 8) as u8 & 1);

        let mhz = bbbw / 1e6;
        let khz = (((mhz - mhz.floor()) * 1000.0 / 7.8125 + 0.5).floor() as u8).min(127);
        for (reg, value) in [
            (0x1fb, mhz as u8),
            (0x1fc, khz),
            (0x1f8, state.rx_bbf_tunediv as u8),
            (0x1f9, state.regs.bbftune_config),
            (0x1d5, 0x3f),
            (0x1c0, 0x03),
            (0x1e2, 0x02),
            (0x1e3, 0x02),
        ] {
            self.regs.poke8(reg, value)?;
        }
        self.regs.poke8(0x016, 0x80)?;
        self.wait_clear(
            0x016,
            0x80,
            100,
            Duration::from_millis(1),
            "rx baseband filter",
        )?;
        self.regs.poke8(0x1e2, 0x03)?;
        self.regs.poke8(0x1e3, 0x03)?;
        Ok(bbbw)
    }

    fn calibrate_baseband_tx_filter(&self, state: &mut State, rf_bw: f64) -> Result<f64> {
        let bbbw = self.one_sided_bw(state, rf_bw, 0.391e6, 20e6);
        let tune_clock = 1.6 * bbbw * 2.0 * std::f64::consts::PI / std::f64::consts::LN_2;
        let divider = ((state.bbpll_freq / tune_clock).ceil() as u16).min(511);
        state.regs.bbftune_mode = (state.regs.bbftune_mode & 0xfe) | ((divider >> 8) as u8 & 1);
        self.regs.poke8(0x0d6, divider as u8)?;
        self.regs.poke8(0x0d7, state.regs.bbftune_mode)?;
        self.regs.poke8(0x0ca, 0x22)?;
        self.regs.poke8(0x016, 0x40)?;
        self.wait_clear(
            0x016,
            0x40,
            100,
            Duration::from_millis(1),
            "tx baseband filter",
        )?;
        self.regs.poke8(0x0ca, 0x26)?;
        Ok(bbbw)
    }

    fn calibrate_secondary_tx_filter(&self, state: &State, rf_bw: f64) -> Result<f64> {
        let bbbw = self.one_sided_bw(state, rf_bw, 0.54e6, 20e6);
        let mhz = bbbw / 1e6;
        let corner = 5.0 * mhz * 2.0 * std::f64::consts::PI;
        let mut resistance = 100;
        let mut capacitance = 0;
        for _ in 0..=3 {
            capacitance =
                (0.5 + (1.0 / (corner * f64::from(resistance) * 1e6)) * 1e12).floor() as i32 - 12;
            if capacitance <= 63 {
                break;
            }
            resistance *= 2;
        }
        capacitance = capacitance.min(63);
        let reg0d0 = if mhz * 2.0 <= 9.0 {
            0x59
        } else if mhz * 2.0 <= 24.0 {
            0x56
        } else {
            0x57
        };
        let reg0d1 = match resistance {
            100 => 0x0c,
            200 => 0x04,
            400 => 0x03,
            800 => 0x01,
            _ => 0x0c,
        };
        self.regs.poke8(0x0d2, capacitance as u8)?;
        self.regs.poke8(0x0d1, reg0d1)?;
        self.regs.poke8(0x0d0, reg0d0)?;
        Ok(bbbw)
    }

    fn calibrate_rx_tias(&self, state: &State, rf_bw: f64) -> Result<f64> {
        let c3_msb = i32::from(self.regs.peek8(0x1eb)? & 0x3f);
        let c3_lsb = i32::from(self.regs.peek8(0x1ec)? & 0x7f);
        let r2346 = i32::from(self.regs.peek8(0x1e6)? & 0x07);
        let bbbw = self.one_sided_bw(state, rf_bw, 0.40e6, 28e6);
        let ceil_mhz = (bbbw / 1e6).ceil();
        let cbbf = c3_msb * 160 + c3_lsb * 10 + 140;
        let ctia = f64::from(cbbf * (18_300 * r2346)) * 0.56 / 3500.0;
        let reg1db = if ceil_mhz <= 3.0 {
            0xe0
        } else if ceil_mhz <= 10.0 {
            0x60
        } else {
            0x20
        };
        let (reg1dc, reg1dd) = if ctia > 2920.0 {
            (
                0x40,
                (((ctia - 400.0) / 320.0 + 0.5).floor() as u8).min(127),
            )
        } else {
            (
                (((ctia - 400.0) / 40.0 + 0.5).floor() as i32 + 0x40) as u8,
                0,
            )
        };
        for (reg, value) in [
            (0x1db, reg1db),
            (0x1dd, reg1dd),
            (0x1df, reg1dd),
            (0x1dc, reg1dc),
            (0x1de, reg1dc),
        ] {
            self.regs.poke8(reg, value)?;
        }
        Ok(bbbw)
    }

    fn setup_adc(&self, state: &State) -> Result<()> {
        let mut bbbw_mhz = ((state.bbpll_freq / 1e6 / f64::from(state.rx_bbf_tunediv))
            * std::f64::consts::LN_2)
            / (1.4 * 2.0 * std::f64::consts::PI);
        bbbw_mhz = bbbw_mhz.clamp(0.20, 28.0);
        let c3_msb = f64::from(self.regs.peek8(0x1eb)? & 0x3f);
        let c3_lsb = f64::from(self.regs.peek8(0x1ec)? & 0x7f);
        let r2346 = f64::from(self.regs.peek8(0x1e6)? & 0x07);
        let fsadc = state.adc_freq / 1e6;
        let correction = if bbbw_mhz < 18.0 {
            1.0
        } else {
            1.0 + 0.01 * (bbbw_mhz - 18.0)
        };
        let rc_time = 1.0
            / ((1.4 * 2.0 * std::f64::consts::PI)
                * (18_300.0 * r2346)
                * (160e-15 * c3_msb + 10e-15 * c3_lsb + 140e-15)
                * (bbbw_mhz * 1e6)
                * correction);
        let scale_res = (1.0 / rc_time).sqrt();
        let scale_cap = scale_res;
        let scale_snr = if state.adc_freq < 80e6 {
            1.0
        } else {
            1.584_893_192
        };
        let maxsnr = 4.0;
        let snr_root = (maxsnr * fsadc / 640.0).sqrt().min(1.0);

        let mut data = [0u8; 40];
        data[3] = 0x24;
        data[4] = 0x24;
        data[7] = byte_floor(-0.5 + 80.0 * scale_snr * scale_res * snr_root).min(124);
        let d7 = f64::from(data[7]);
        data[8] = byte_floor(0.5 + 20.0 * (640.0 / fsadc) * (d7 / 80.0) / (scale_res * scale_cap));
        data[10] = byte_floor(-0.5 + 77.0 * scale_res * snr_root).min(127);
        let d10 = f64::from(data[10]);
        data[9] = byte_floor(0.8 * d10).min(127);
        data[11] =
            byte_floor(0.5 + 20.0 * (640.0 / fsadc) * (d10 / 77.0) / (scale_res * scale_cap));
        data[12] = byte_floor(-0.5 + 80.0 * scale_res * snr_root).min(127);
        let d12 = f64::from(data[12]);
        data[13] =
            byte_floor(-1.5 + 20.0 * (640.0 / fsadc) * (d12 / 80.0) / (scale_res * scale_cap));
        data[14] = 21u8.wrapping_mul(byte_floor(0.1 * 640.0 / fsadc));
        data[15] = byte_trunc(1.025 * d7).min(127);
        let gain_shape = |value: u8| {
            byte_floor(f64::from(value) * (0.98 + 0.02 * (640.0 / fsadc / maxsnr).max(1.0)))
                .min(127)
        };
        data[16] = gain_shape(data[15]);
        data[17] = data[15];
        data[18] = byte_trunc(0.975 * d10).min(127);
        data[19] = gain_shape(data[18]);
        data[20] = data[18];
        data[21] = byte_trunc(0.975 * d12).min(127);
        data[22] = gain_shape(data[21]);
        data[23] = data[21];
        data[24] = 0x2e;
        let a = byte_floor(128.0 + (63.0 * fsadc / 640.0).min(63.0));
        let b = byte_floor((63.0 * fsadc / 640.0 * (0.92 + 0.08 * 640.0 / fsadc)).min(63.0));
        data[25] = a;
        data[26] = b;
        data[27] = byte_floor((32.0 * (fsadc / 640.0).sqrt()).min(63.0));
        data[28] = a;
        data[29] = b;
        data[30] = data[27];
        data[31] = a;
        data[32] = b;
        data[33] = byte_floor((63.0 * (fsadc / 640.0).sqrt()).min(63.0));
        data[34] = byte_floor(64.0 * (fsadc / 640.0).sqrt()).min(127);
        data[35] = 0x40;
        data[36] = 0x40;
        data[37] = 0x2c;

        for (offset, value) in data.into_iter().enumerate() {
            self.regs.poke8(0x200 + offset as u16, value)?;
        }
        Ok(())
    }

    fn calibrate_synth_charge_pumps(&self) -> Result<()> {
        if self.regs.peek8(0x017)? & 0x0f != 5 {
            return Err(Error::device(
                "ad936x is not in ALERT during charge-pump calibration",
            ));
        }
        self.regs.poke8(0x23d, 0x04)?;
        self.wait_set(0x244, 0x80, 5, Duration::from_millis(1), "rx charge pump")?;
        self.regs.poke8(0x23d, 0x00)?;
        self.regs.poke8(0x27d, 0x04)?;
        self.wait_set(0x284, 0x80, 5, Duration::from_millis(1), "tx charge pump")?;
        self.regs.poke8(0x27d, 0x00)
    }

    fn calibrate_baseband_dc_offset(&self) -> Result<()> {
        for (reg, value) in [(0x18b, 0x83), (0x193, 0x3f), (0x190, 0x0f), (0x194, 0x01)] {
            self.regs.poke8(reg, value)?;
        }
        self.regs.poke8(0x016, 0x01)?;
        self.wait_clear(
            0x016,
            0x01,
            100,
            Duration::from_millis(5),
            "baseband dc offset",
        )
    }

    fn calibrate_rf_dc_offset(&self, state: &State) -> Result<()> {
        let frequency_settings = if state.rx_freq < 4e9 {
            [(0x186, 0x32), (0x187, 0x24), (0x188, 0x05)]
        } else {
            [(0x186, 0x28), (0x187, 0x34), (0x188, 0x06)]
        };
        for (reg, value) in
            frequency_settings
                .into_iter()
                .chain([(0x185, 0x20), (0x18b, 0x83), (0x189, 0x30)])
        {
            self.regs.poke8(reg, value)?;
        }
        self.regs.poke8(0x016, 0x02)?;
        self.wait_clear(0x016, 0x02, 200, Duration::from_millis(50), "rf dc offset")?;
        self.regs.poke8(0x18b, 0x8d)
    }

    fn configure_bb_dc_tracking(&self, enabled: bool) -> Result<()> {
        self.regs.poke8(0x18b, if enabled { 0xad } else { 0x8d })
    }

    fn configure_rx_iq_tracking(&self, enabled: bool) -> Result<()> {
        self.regs.poke8(0x169, if enabled { 0xcf } else { 0xc0 })
    }

    /// Enable or disable receive baseband-DC tracking.
    pub fn set_dc_offset_auto(&self, direction: Direction, enabled: bool) -> Result<()> {
        if direction == Direction::Tx {
            return Err(Error::not_supported("ad936x tx dc tracking"));
        }
        let mut state = self.locked();
        state.use_dc_offset_tracking = enabled;
        self.configure_bb_dc_tracking(enabled)
    }

    /// Enable or disable receive IQ tracking. Disabling it runs the static RX
    /// quadrature calibration, matching UHD's fallback path.
    pub fn set_iq_balance_auto(&self, direction: Direction, enabled: bool) -> Result<()> {
        if direction == Direction::Tx {
            return Err(Error::not_supported("ad936x tx iq tracking"));
        }
        let mut state = self.locked();
        state.use_iq_balance_tracking = enabled;
        self.configure_rx_iq_tracking(enabled)?;
        if !enabled {
            self.regs.poke8(0x014, 0x05)?;
            self.calibrate_rx_quadrature(&mut state)?;
            self.regs.poke8(0x014, 0x21)?;
        }
        Ok(())
    }

    fn calibrate_rx_quadrature(&self, state: &mut State) -> Result<()> {
        for (reg, value) in [
            (0x168, 0x03),
            (0x16e, 0x25),
            (0x16a, 0x75),
            (0x16b, 0x95),
            (0x057, 0x33),
            (0x169, 0xc0),
        ] {
            self.regs.poke8(reg, value)?;
        }
        let tx_freq = state.tx_freq;
        self.tune_helper(
            state,
            Direction::Tx,
            state.rx_freq + state.rx_bb_lp_bw / 2.0,
        )?;
        self.regs.poke8(0x016, 0x20)?;
        self.wait_clear(0x016, 0x20, 1000, Duration::from_millis(5), "rx quadrature")?;
        self.regs.poke8(0x057, 0x30)?;
        self.tune_helper(state, Direction::Tx, tx_freq)?;
        Ok(())
    }

    fn tx_quadrature_cal_routine(&self, state: &State) -> Result<()> {
        let original = self.regs.peek8(0x0a3)?;
        let nco = original & 0xc0;
        self.regs.poke8(0x0a0, 0x15 | (nco >> 1))?;
        let reread = self.regs.peek8(0x0a3)?;
        self.regs.poke8(0x0a3, (reread & 0x3f) | nco)?;

        let max_cal_freq =
            state.baseband_bw * f64::from(state.tfir_factor) * f64::from((nco >> 6) + 1) / 16.0;
        let bbbw = (state.baseband_bw / 2.0).clamp(0.20e6, 28e6);
        if max_cal_freq > bbbw {
            return Err(Error::device(format!(
                "ad936x tx quadrature tones {max_cal_freq} exceed rx baseband {bbbw}"
            )));
        }
        for (reg, value) in [
            (0x0a1, 0x7b),
            (0x0a9, 0xff),
            (0x0a2, 0x7f),
            (0x0a5, 0x01),
            (0x0a6, 0x01),
            (0x0aa, if state.rx_freq < 1.3e9 { 0x22 } else { 0x25 }),
            (0x0a4, 0xf0),
            (0x0ae, 0x00),
        ] {
            self.regs.poke8(reg, value)?;
        }
        self.regs.poke8(0x016, 0x10)?;
        self.wait_clear(0x016, 0x10, 100, Duration::from_millis(10), "tx quadrature")
    }

    fn calibrate_tx_quadrature(&self, state: &mut State) -> Result<()> {
        if self.regs.peek8(0x017)? & 0x0f != 5 {
            return Err(Error::device(
                "ad936x tx quadrature calibration requires ALERT",
            ));
        }
        self.regs.poke8(0x169, 0xc0)?;
        let original = state.regs.inputsel;
        state.regs.inputsel &= 0xbf;
        self.regs.poke8(0x004, state.regs.inputsel)?;
        self.tx_quadrature_cal_routine(state)?;
        state.regs.inputsel |= 0x40;
        self.regs.poke8(0x004, state.regs.inputsel)?;
        self.tx_quadrature_cal_routine(state)?;
        state.regs.inputsel = original;
        self.regs.poke8(0x004, original)
    }

    /// Enable the requested converter chains, calibrating TX quadrature when a
    /// transmitter is enabled.
    pub fn set_active_chains(&self, tx1: bool, tx2: bool, rx1: bool, rx2: bool) -> Result<()> {
        let mut state = self.locked();
        self.set_active_chains_inner(&mut state, tx1, tx2, rx1, rx2)
    }

    fn set_active_chains_inner(
        &self,
        state: &mut State,
        tx1: bool,
        tx2: bool,
        rx1: bool,
        rx2: bool,
    ) -> Result<()> {
        state.regs.txfilt &= 0x3f;
        state.regs.rxfilt &= 0x3f;
        state.regs.txfilt |= (u8::from(tx1) << 6) | (u8::from(tx2) << 7);
        state.regs.rxfilt |= (u8::from(rx1) << 6) | (u8::from(rx2) << 7);

        let mut ensm = self.regs.peek8(0x017)? & 0x0f;
        let return_to_fdd = ensm == 0x0a;
        if return_to_fdd {
            self.regs.poke8(0x014, 0x01)?;
        }
        let mut count = 0usize;
        while matches!(ensm, 0x0a | 0x0b) {
            if count > 1000 {
                return Err(Error::device("ad936x ENSM did not leave FDD flush"));
            }
            ensm = self.regs.peek8(0x017)? & 0x0f;
            count += 1;
        }
        self.regs.poke8(0x002, state.regs.txfilt)?;
        self.regs.poke8(0x003, state.regs.rxfilt)?;
        if tx1 || tx2 {
            self.calibrate_tx_quadrature(state)?;
        }
        if return_to_fdd {
            self.regs.poke8(0x014, 0x21)?;
        }
        Ok(())
    }

    fn wait_clear(
        &self,
        reg: u16,
        mask: u8,
        tries: usize,
        interval: Duration,
        name: &str,
    ) -> Result<()> {
        self.wait_for(reg, mask, false, tries, interval, name)
    }

    fn wait_set(
        &self,
        reg: u16,
        mask: u8,
        tries: usize,
        interval: Duration,
        name: &str,
    ) -> Result<()> {
        self.wait_for(reg, mask, true, tries, interval, name)
    }

    fn wait_for(
        &self,
        reg: u16,
        mask: u8,
        set: bool,
        tries: usize,
        interval: Duration,
        name: &str,
    ) -> Result<()> {
        let mut count = 0usize;
        while (self.regs.peek8(reg)? & mask != 0) != set {
            if count > tries {
                return Err(Error::device(format!("ad936x {name} calibration failed")));
            }
            count += 1;
            sleep(interval);
        }
        Ok(())
    }

    /// Bring the AD9361/AD9364 from reset through the complete UHD
    /// `ad9361_device_t::initialize` sequence.
    pub fn init(&self) -> Result<f64> {
        *self.locked() = State::default();
        self.reset()?;
        let id = self.identify()?;
        debug!(%id, "ad936x reset and identified");
        self.housekeeping()?;
        self.select_clock_path()?;

        let mut state = self.locked();
        self.setup_rates_inner(&mut state, INIT_CLOCK_RATE)?;
        self.set_data_port()?;
        self.set_interface_delays()?;
        self.setup_aux_and_gpo()?;
        self.setup_rf_frontend()?;

        self.enter_alert()?;
        self.run_rate_calibrations(&mut state, false)?;

        for (reg, value) in [
            (0x073, 0x00),
            (0x074, 0x00),
            (0x075, 0x00),
            (0x076, 0x00),
            (0x150, 0x0e),
            (0x151, 0x00),
            (0x152, 0xff),
            (0x153, 0x00),
            (0x154, 0x00),
            (0x155, 0x00),
            (0x156, 0x00),
            (0x157, 0x00),
            (0x158, 0x0d),
            (0x15c, 0x67),
        ] {
            self.regs.poke8(reg, value)?;
        }
        self.set_active_chains_inner(&mut state, true, false, false, false)?;
        self.regs.poke8(0x014, 0x21)?;
        self.regs.poke8(0x077, 0x00)?;
        debug!(rate = state.baseband_bw, "ad936x initialized");
        Ok(state.baseband_bw)
    }

    fn setup_aux_and_gpo(&self) -> Result<()> {
        const VALUES: &[(u16, u8)] = &[
            (0x018, 0x00),
            (0x019, 0x00),
            (0x01a, 0x00),
            (0x01b, 0x00),
            (0x023, 0xff),
            (0x026, 0x00),
            (0x030, 0x00),
            (0x031, 0x00),
            (0x032, 0x00),
            (0x033, 0x00),
            (0x022, 0x0a),
            (0x00b, 0x00),
            (0x00c, 0x00),
            (0x00d, 0x00),
            (0x00f, 0x04),
            (0x01c, 0x10),
            (0x01d, 0x01),
            (0x035, 0x01),
            (0x036, 0xff),
            (0x03a, 0x27),
            (0x020, 0x00),
            (0x027, 0x03),
            (0x028, 0x00),
            (0x029, 0x00),
            (0x02a, 0x00),
            (0x02b, 0x00),
            (0x02c, 0x00),
            (0x02d, 0x00),
            (0x02e, 0x00),
            (0x02f, 0x00),
        ];
        for &(reg, value) in VALUES {
            self.regs.poke8(reg, value)?;
        }
        Ok(())
    }

    fn setup_rf_frontend(&self) -> Result<()> {
        for (reg, value) in [
            (0x261, 0x00),
            (0x2a1, 0x00),
            (0x248, 0x0b),
            (0x288, 0x0b),
            (0x246, 0x02),
            (0x286, 0x02),
            (0x249, 0x8e),
            (0x289, 0x8e),
            (0x23b, 0x80),
            (0x27b, 0x80),
            (0x243, 0x0d),
            (0x283, 0x0d),
            (0x23d, 0x00),
            (0x27d, 0x00),
        ] {
            self.regs.poke8(reg, value)?;
        }
        Ok(())
    }

    fn enter_alert(&self) -> Result<()> {
        self.regs.poke8(0x015, 0x04)?;
        self.regs.poke8(0x014, 0x05)?;
        self.regs.poke8(0x013, 0x01)?;
        sleep(Duration::from_millis(1));
        Ok(())
    }

    fn run_rate_calibrations(&self, state: &mut State, reprogram_gains: bool) -> Result<()> {
        self.calibrate_synth_charge_pumps()?;
        self.tune_helper(state, Direction::Rx, state.rx_freq)?;
        self.tune_helper(state, Direction::Tx, state.tx_freq)?;
        self.program_mixer_gm_subtable()?;
        self.program_gain_table(state)?;
        self.setup_gain_control(false)?;
        if reprogram_gains {
            self.reprogram_gains(state)?;
        }
        self.set_bandwidth_inner(state, Direction::Rx, state.baseband_bw)?;
        self.set_bandwidth_inner(state, Direction::Tx, state.baseband_bw)?;
        self.setup_adc(state)?;
        self.calibrate_baseband_dc_offset()?;
        self.calibrate_rf_dc_offset(state)?;
        self.calibrate_rx_quadrature(state)?;
        if state.use_dc_offset_tracking {
            self.configure_bb_dc_tracking(true)?;
        }
        if state.use_iq_balance_tracking {
            self.configure_rx_iq_tracking(true)?;
        }
        state.last_rx_cal_freq = state.rx_freq;
        state.last_tx_cal_freq = state.tx_freq;
        self.restore_parallel_port_config()?;
        self.regs.poke8(0x013, 0x01)?;
        self.regs.poke8(0x015, 0x04)
    }

    /// Change the sample rate and repeat every calibration invalidated by the
    /// clock-chain change.
    pub fn set_sample_rate(&self, requested: f64) -> Result<f64> {
        if !requested.is_finite() || requested > crate::MAX_CLOCK_RATE {
            return Err(Error::OutOfRange {
                what: "ad936x sample rate".into(),
                value: requested,
                start: crate::MIN_CLOCK_RATE,
                stop: crate::MAX_CLOCK_RATE,
            });
        }
        let mut state = self.locked();
        if freq_is_nearly_equal(requested, state.req_clock_rate) {
            return Ok(state.baseband_bw);
        }
        let ensm = self.regs.peek8(0x017)? & 0x0f;
        match ensm {
            0x05 => {
                self.regs.poke8(0x014, 0x21)?;
                sleep(Duration::from_millis(5));
                self.regs.poke8(0x014, 0x00)?;
            }
            0x0a => self.regs.poke8(0x014, 0x00)?,
            _ => {
                return Err(Error::device(format!(
                    "ad936x cannot change rate from ENSM state {ensm:#x}"
                )));
            }
        }
        let tx_chains = state.regs.txfilt & 0xc0;
        let rx_chains = state.regs.rxfilt & 0xc0;
        self.setup_rates_inner(&mut state, requested)?;
        self.enter_alert()?;
        self.run_rate_calibrations(&mut state, true)?;
        if ensm == 0x0a {
            state.regs.txfilt = (state.regs.txfilt & 0x3f) | tx_chains;
            state.regs.rxfilt = (state.regs.rxfilt & 0x3f) | rx_chains;
            self.regs.poke8(0x002, state.regs.txfilt)?;
            self.regs.poke8(0x003, state.regs.rxfilt)?;
            self.regs.poke8(0x014, 0x21)?;
        }
        Ok(state.baseband_bw)
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

fn chain_indices(chain: Chain) -> impl Iterator<Item = usize> {
    let indices: &[usize] = match chain {
        Chain::One => &[0],
        Chain::Two => &[1],
        Chain::Both => &[0, 1],
    };
    indices.iter().copied()
}

fn byte_floor(value: f64) -> u8 {
    value.floor().clamp(0.0, 255.0) as u8
}

fn byte_trunc(value: f64) -> u8 {
    value.clamp(0.0, 255.0) as u8
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

    fn calibrated_part() -> Arc<FakeSpi> {
        Arc::new(FakeSpi::with_regs(&[
            (regs::PRODUCT_ID, 0x0a),
            (regs::BBPLL_LOCK_STATUS, regs::BBPLL_LOCKED),
            (0x017, 0x05),
            (0x244, 0x80),
            (0x284, 0x80),
            (0x247, 0x02),
            (0x287, 0x02),
            (0x1eb, 0x01),
            (0x1ec, 0x01),
            (0x1e6, 0x01),
        ]))
    }

    #[test]
    fn full_init_runs_tuning_filters_and_calibrations_in_reference_order() {
        let spi = calibrated_part();
        let codec = Ad936x::new(spi.clone(), mini_params());
        assert_eq!(codec.init().unwrap(), INIT_CLOCK_RATE);
        assert_eq!(codec.freq(Direction::Rx), crate::DEFAULT_RX_FREQ);
        assert_eq!(codec.freq(Direction::Tx), crate::DEFAULT_TX_FREQ);

        let pokes = spi.pokes();
        let first = |reg: u16| {
            pokes
                .iter()
                .position(|&(address, _)| address == reg)
                .unwrap()
        };
        let first_value = |reg: u16, value: u8| {
            pokes
                .iter()
                .position(|&write| write == (reg, value))
                .unwrap()
        };
        let charge_pump = first_value(0x23d, 0x04);
        assert!(first(0x013) < charge_pump); // ALERT before charge-pump cal
        assert!(charge_pump < first(0x233)); // charge pump before RX tune
        assert!(first(0x233) < first(0x273)); // RX tune before TX tune
        assert!(first(0x273) < first(0x13f)); // tuning before mixer table
        assert!(first(0x13f) < first(0x137)); // mixer before gain table
        assert!(first(0x137) < first(0x1fb)); // gain before analog filters
        assert!(first(0x1fb) < first(0x200)); // filters before ADC setup
        assert!(first(0x200) < first(0x193)); // ADC before BB DC
        assert!(first(0x193) < first(0x186)); // BB DC before RF DC
        assert!(first(0x186) < first(0x168)); // RF DC before RX quadrature
        assert_eq!(pokes.last(), Some(&(0x077, 0x00)));
    }

    #[test]
    fn manual_gain_paths_program_the_ad9364_chain_one_registers() {
        let spi = working_part();
        let codec = Ad936x::new(spi.clone(), mini_params());
        assert_eq!(
            codec.set_gain(Direction::Rx, Chain::One, 37.9).unwrap(),
            37.0
        );
        assert_eq!(
            codec.set_gain(Direction::Tx, Chain::One, 10.0).unwrap(),
            10.0
        );
        assert_eq!(
            spi.pokes(),
            vec![(0x109, 37), (0x073, 0x3f), (0x074, 0x01), (0x07c, 0x40)]
        );
    }

    #[test]
    fn public_tune_programs_the_rfpll_and_returns_its_exact_lo() {
        let spi = calibrated_part();
        let codec = Ad936x::new(spi.clone(), mini_params());
        let requested = 2_450_123_456.0;
        let actual = codec.tune(Direction::Rx, requested).unwrap();
        assert!((actual - requested).abs() < 5.0);
        assert_eq!(codec.freq(Direction::Rx), actual);

        let pll = RfPll::solve(requested).unwrap();
        let pokes = spi.pokes();
        let last = |reg: u16| {
            pokes
                .iter()
                .rev()
                .find(|&&(address, _)| address == reg)
                .map(|&(_, value)| value)
        };
        assert_eq!(last(0x231), Some(pll.nint as u8));
        assert_eq!(last(0x233), Some(pll.nfrac as u8));
        assert_eq!(last(0x005), Some(pll.divider_exp));
    }

    #[test]
    fn public_sample_rate_change_recalibrates_and_caches_the_request() {
        let spi = calibrated_part();
        let codec = Ad936x::new(spi.clone(), mini_params());
        codec.init().unwrap();
        let before = spi.pokes().len();
        let actual = codec.set_sample_rate(30.72e6).unwrap();
        assert!((actual - 30.72e6).abs() < 1.0);
        let changed = spi.pokes();
        assert!(changed[before..].iter().any(|&(reg, _)| reg == 0x193));
        assert!(changed[before..].iter().any(|&(reg, _)| reg == 0x168));

        let before_cached = changed.len();
        assert_eq!(codec.set_sample_rate(30.72e6).unwrap(), actual);
        assert_eq!(spi.pokes().len(), before_cached);
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
        // cache on the requested rate itself, but it lives in `set_sample_rate`.
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
