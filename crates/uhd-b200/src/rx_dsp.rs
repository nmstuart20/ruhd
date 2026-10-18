//! One radio chain's receive DSP: `rx_dsp_core_3000`, in its B2xx form.
//!
//! Ports `host/lib/usrp/cores/rx_dsp_core_3000.cpp` with `is_b200 = true`,
//! which is the only way `b200_impl::setup_radio` constructs it.
//!
//! # What the chain does
//!
//! Samples leave the AD936x at the master clock rate (the *tick rate*). This
//! core shifts them in frequency with a CORDIC, decimates them with a CIC and
//! up to two half-band filters, and rescales the result so the CIC's
//! algorithmic gain does not eat the 16-bit output range. The host rate is
//! therefore always `tick_rate / decim` for an integer `decim`.
//!
//! The B2xx build has only two half-bands. The third half-band, and the
//! decimations above 512 that it makes possible, exist only in the other
//! `rx_dsp_core_3000` builds.
//!
//! # Register writes
//!
//! Every setter writes straight through. The core has no readback, so the
//! values held here are the only record of its state.

use std::fmt;

use tracing::warn;
use uhd_core::{Error, Result};
use uhd_transport::RegisterIface;

/// `SR_RX_DSP`, the base of the DSP registers. A byte address, `TOREG(144)`.
pub const SR_RX_DSP: u32 = 144 * 4;

/// CORDIC phase increment, as a signed fraction of the tick rate.
const REG_FREQ: u32 = 0;
/// Fixed-point output scale, applied to I and Q alike.
const REG_SCALE_IQ: u32 = 4;
/// Half-band enables and CIC decimation.
const REG_DECIM: u32 = 8;
/// Input routing: swap, real mode and inversion flags.
const REG_MUX: u32 = 12;

/// `FLAG_DSP_RX_MUX_SWAP_IQ`.
pub const MUX_SWAP_IQ: u32 = 1 << 0;
/// `FLAG_DSP_RX_MUX_REAL_MODE`.
pub const MUX_REAL_MODE: u32 = 1 << 1;
/// `FLAG_DSP_RX_MUX_INVERT_Q`.
pub const MUX_INVERT_Q: u32 = 1 << 2;
/// `FLAG_DSP_RX_MUX_INVERT_I`.
pub const MUX_INVERT_I: u32 = 1 << 3;

/// The link rate `setup_radio` hands the core, in samples per second.
///
/// `set_link_rate(10e9 / 8)`, divided by two bytes per sample the way
/// `set_link_rate` does it. The reference's own comment on the number is
/// "whatever": its only effect is to put a floor under the decimation, and at
/// any master clock rate an AD936x can produce that floor is 1.
const LINK_RATE: f64 = 10e9 / 8.0 / 2.0;

/// One radio's receive DSP chain.
pub struct RxDsp {
    bus: Box<dyn RegisterIface>,
    base: u32,
    tick_rate: f64,
    freq: f64,
    decim: u32,
    scaling_adjustment: f64,
    dsp_extra_scaling: f64,
    host_extra_scaling: f64,
    fxpt_scalar_correction: f64,
}

impl fmt::Debug for RxDsp {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("RxDsp")
            .field("base", &self.base)
            .field("tick_rate", &self.tick_rate)
            .field("freq", &self.freq)
            .field("decim", &self.decim)
            .finish_non_exhaustive()
    }
}

impl RxDsp {
    /// Binds the core at the standard B2xx offset, without writing anything.
    ///
    /// `bus` must be a port onto a *radio* core. The tick rate starts at 1, as
    /// the reference's does, and has to be set before any other setter means
    /// anything.
    #[must_use]
    pub fn new(bus: Box<dyn RegisterIface>) -> Self {
        Self {
            bus,
            base: SR_RX_DSP,
            tick_rate: 1.0,
            freq: 0.0,
            decim: 1,
            scaling_adjustment: 1.0,
            dsp_extra_scaling: 1.0,
            host_extra_scaling: 1.0,
            fxpt_scalar_correction: 1.0,
        }
    }

    /// Tells the core the master clock rate, and re-applies the CORDIC
    /// frequency, whose register value depends on it.
    ///
    /// # Errors
    ///
    /// [`Error::InvalidArg`] if the rate is not positive and finite; otherwise
    /// whatever the settings bus reports.
    pub fn set_tick_rate(&mut self, rate: f64) -> Result<()> {
        if !rate.is_finite() || rate <= 0.0 {
            return Err(Error::invalid_arg(format!(
                "a tick rate is a positive number of ticks per second, not {rate}"
            )));
        }
        self.tick_rate = rate;
        self.set_freq(self.freq).map(|_| ())
    }

    /// The master clock rate this core is computing against.
    #[must_use]
    pub fn tick_rate(&self) -> f64 {
        self.tick_rate
    }

    /// Writes the input-routing flags: some combination of the `MUX_*`
    /// constants, or 0 for plain I/Q, which is what every B2xx radio uses.
    ///
    /// # Errors
    ///
    /// Whatever the settings bus reports.
    pub fn set_mux(&mut self, flags: u32) -> Result<()> {
        self.bus.poke32(self.base + REG_MUX, flags)
    }

    /// Every decimation the core accepts at the current tick rate, largest
    /// first.
    ///
    /// The reference publishes these as a list of host rates; it is the same
    /// list divided into the tick rate.
    #[must_use]
    pub fn decimations(&self) -> Vec<u32> {
        decimations(self.tick_rate)
    }

    /// Selects the decimation whose host rate is nearest `rate`, programs the
    /// filters and the output scale, and returns the host rate that results.
    ///
    /// # Errors
    ///
    /// [`Error::InvalidArg`] if the rate is not positive and finite; otherwise
    /// whatever the settings bus reports.
    pub fn set_host_rate(&mut self, rate: f64) -> Result<f64> {
        if !rate.is_finite() || rate <= 0.0 {
            return Err(Error::invalid_arg(format!(
                "a sample rate is a positive number of samples per second, not {rate}"
            )));
        }
        let decim = nearest_decimation(self.tick_rate, rate);
        let (word, cic) = decim_word(decim);
        self.bus.poke32(self.base + REG_DECIM, word)?;
        if cic > 1 && word & (3 << 8) == 0 {
            warn!(
                decim,
                tick_rate = self.tick_rate,
                rate,
                "the requested decimation is odd, so no half-band filter is enabled and \
                 the passband will show CIC rolloff; choose an even decimation"
            );
        }
        self.decim = decim;
        self.scaling_adjustment = cic_scaling_adjustment(cic);
        self.update_scalar()?;
        Ok(self.host_rate())
    }

    /// The host rate the core is currently producing.
    #[must_use]
    pub fn host_rate(&self) -> f64 {
        self.tick_rate / f64::from(self.decim)
    }

    /// The current decimation.
    #[must_use]
    pub fn decimation(&self) -> u32 {
        self.decim
    }

    /// Sets the CORDIC shift, returning the frequency the 32-bit phase word
    /// actually produces.
    ///
    /// # Errors
    ///
    /// Whatever the settings bus reports.
    pub fn set_freq(&mut self, freq: f64) -> Result<f64> {
        let (word, actual) = freq_word(freq, self.tick_rate);
        self.bus.poke32(self.base + REG_FREQ, word as u32)?;
        self.freq = actual;
        Ok(actual)
    }

    /// The CORDIC shift as last programmed.
    #[must_use]
    pub fn freq(&self) -> f64 {
        self.freq
    }

    /// Configures the output scale for an `sc16` wire format.
    ///
    /// `setup` for `otw_format == "sc16"`. The other wire formats change the
    /// extra scaling factors and are not wired up yet.
    ///
    /// # Errors
    ///
    /// Whatever the settings bus reports.
    pub fn setup_sc16(&mut self) -> Result<()> {
        self.dsp_extra_scaling = 1.0;
        self.host_extra_scaling = 1.0;
        self.update_scalar()
    }

    /// The factor that turns a received `sc16` integer into a float in
    /// `[-1, 1)`, corrected for the rounding of the fixed-point scale.
    ///
    /// `get_scaling_adjustment`. A float host format multiplies by this; an
    /// integer one leaves samples as they arrived.
    #[must_use]
    pub fn scaling_adjustment(&self) -> f64 {
        self.fxpt_scalar_correction * self.host_extra_scaling / 32767.0
    }

    fn update_scalar(&mut self) -> Result<()> {
        let (actual, correction) = scalar(self.scaling_adjustment, self.dsp_extra_scaling);
        self.fxpt_scalar_correction = correction;
        self.bus.poke32(self.base + REG_SCALE_IQ, actual as u32)
    }
}

/// Every decimation the B2xx core accepts, largest first.
///
/// `get_host_rates` with `is_b200`: steps of 4 above 256, of 2 above 128, and
/// of 1 down to the link-rate floor.
fn decimations(tick_rate: f64) -> Vec<u32> {
    let floor = ((tick_rate / LINK_RATE).ceil() as u32).max(1);
    let mut out: Vec<u32> = (257..=512).rev().step_by(4).collect();
    out.extend((129..=256).rev().step_by(2));
    out.extend((floor..=128).rev());
    out
}

/// The accepted decimation whose host rate lies closest to `rate`.
fn nearest_decimation(tick_rate: f64, rate: f64) -> u32 {
    decimations(tick_rate)
        .into_iter()
        .min_by(|&a, &b| {
            let da = (tick_rate / f64::from(a) - rate).abs();
            let db = (tick_rate / f64::from(b) - rate).abs();
            da.total_cmp(&db)
        })
        .unwrap_or(1)
}

/// The decimation register word for `decim`, and the decimation left for the
/// CIC once the half-bands have taken their factors of two.
///
/// Bit 9 enables the small half-band, bit 8 the large one.
fn decim_word(decim: u32) -> (u32, u32) {
    let mut cic = decim;
    let mut hb0 = 0;
    let mut hb1 = 0;
    if cic % 2 == 0 {
        hb0 = 1;
        cic /= 2;
    }
    if cic % 2 == 0 {
        hb1 = 1;
        cic /= 2;
    }
    ((hb0 << 9) | (hb1 << 8) | (cic & 0xff), cic & 0xff)
}

/// Compensation for the CIC's gain of `cic^4` and the CORDIC's of about 1.648,
/// less the power-of-two compensation already built into the core.
fn cic_scaling_adjustment(cic: u32) -> f64 {
    let rate_pow = f64::from(cic).powi(4);
    // The reference's ceil_log2, float rounding and all, so that a decimation
    // landing exactly on a power of two computes the same scale it does.
    let ceil_log2 = (rate_pow.ln() / 2f64.ln()).ceil();
    2f64.powf(ceil_log2) / (1.648 * rate_pow)
}

/// The integer scale register value, and the ratio between the scale wanted
/// and the scale that integer gives, which the host corrects for.
fn scalar(scaling_adjustment: f64, dsp_extra_scaling: f64) -> (i32, f64) {
    let target = f64::from(1u32 << 16) * scaling_adjustment / dsp_extra_scaling;
    let actual = target.round() as i32;
    (actual, target / f64::from(actual))
}

/// The CORDIC phase word for `freq` at `tick_rate`, and the frequency it
/// actually produces.
///
/// `get_freq_and_freq_word<32>`: the frequency is wrapped into the first
/// Nyquist zone, scaled to a signed 32-bit fraction of the tick rate, and
/// clamped where the scaling would overflow.
fn freq_word(freq: f64, tick_rate: f64) -> (i32, f64) {
    let scale = 4_294_967_296.0;
    let scaled = wrap_frequency(freq / tick_rate, 1.0) * scale;
    let word = if scaled >= f64::from(i32::MAX) {
        i32::MAX
    } else if scaled <= f64::from(i32::MIN) {
        i32::MIN
    } else {
        scaled.round() as i32
    };
    (word, f64::from(word) / scale * tick_rate)
}

/// `uhd::math::wrap_frequency`: fold `freq` into `[-rate/2, rate/2]`.
fn wrap_frequency(freq: f64, rate: f64) -> f64 {
    let mut f = freq % rate;
    if f.abs() > rate / 2.0 {
        f -= f.signum() * rate;
    }
    f
}

#[cfg(test)]
mod tests {
    use std::sync::Mutex;

    use super::*;

    #[derive(Debug, Default)]
    struct FakeBus {
        pokes: Mutex<Vec<(u32, u32)>>,
    }

    impl RegisterIface for &'static FakeBus {
        fn poke32(&self, addr: u32, data: u32) -> Result<()> {
            self.pokes.lock().unwrap().push((addr, data));
            Ok(())
        }
        fn peek32(&self, _addr: u32) -> Result<u32> {
            Ok(0)
        }
        fn peek64(&self, _addr: u32) -> Result<u64> {
            Ok(0)
        }
    }

    fn bus() -> &'static FakeBus {
        Box::leak(Box::default())
    }

    #[test]
    fn register_offsets_match_the_reference() {
        assert_eq!(SR_RX_DSP, 576);
        assert_eq!((REG_FREQ, REG_SCALE_IQ, REG_DECIM, REG_MUX), (0, 4, 8, 12));
    }

    #[test]
    fn the_decimation_list_has_the_b200_steps() {
        let list = decimations(61.44e6);
        assert_eq!(list.first(), Some(&512));
        assert_eq!(list.last(), Some(&1));
        assert!(list.contains(&260) && !list.contains(&258));
        assert!(list.contains(&130) && !list.contains(&129));
        assert!(list.contains(&127));
        // Nothing the third half-band would allow.
        assert!(list.iter().all(|&d| d <= 512));
    }

    #[test]
    fn half_bands_take_the_factors_of_two() {
        assert_eq!(decim_word(1), (1, 1));
        assert_eq!(decim_word(2), (0x200 | 1, 1));
        assert_eq!(decim_word(4), (0x300 | 1, 1));
        assert_eq!(decim_word(32), (0x300 | 8, 8));
        assert_eq!(decim_word(6), (0x200 | 3, 3));
        assert_eq!(decim_word(5), (5, 5));
    }

    #[test]
    fn unity_decimation_scales_like_uhd() {
        // 65536 / 1.648, the number UHD writes for decim 1 on sc16.
        let (actual, correction) = scalar(cic_scaling_adjustment(1), 1.0);
        assert_eq!(actual, 39767);
        assert!((correction - 1.0).abs() < 1e-4);
    }

    #[test]
    fn cic_gain_is_compensated_to_the_next_power_of_two() {
        // cic 8: 8^4 = 4096 is already a power of two, so only the CORDIC's
        // 1.648 is left to correct.
        assert!((cic_scaling_adjustment(8) - 1.0 / 1.648).abs() < 1e-12);
        // cic 3: 81 -> 128.
        assert!((cic_scaling_adjustment(3) - 128.0 / (1.648 * 81.0)).abs() < 1e-12);
    }

    #[test]
    fn freq_words_round_and_wrap() {
        assert_eq!(freq_word(0.0, 32e6), (0, 0.0));
        let (word, actual) = freq_word(1e6, 32e6);
        assert_eq!(word, 134_217_728);
        assert!((actual - 1e6).abs() < 1e-6);
        // A shift above Nyquist folds to its alias.
        let (word, _) = freq_word(20e6, 32e6);
        assert_eq!(word, freq_word(-12e6, 32e6).0);
        // Negative shifts are two's complement.
        assert_eq!(freq_word(-1e6, 32e6).0, -134_217_728);
    }

    #[test]
    fn set_host_rate_picks_the_nearest_decimation_and_writes_both_registers() {
        let bus = bus();
        let mut dsp = RxDsp::new(Box::new(bus));
        dsp.set_tick_rate(32e6).unwrap();
        let rate = dsp.set_host_rate(1e6).unwrap();
        assert_eq!(rate, 1e6);
        assert_eq!(dsp.decimation(), 32);

        let pokes = bus.pokes.lock().unwrap().clone();
        assert_eq!(pokes[0], (SR_RX_DSP + REG_FREQ, 0));
        assert_eq!(pokes[1], (SR_RX_DSP + REG_DECIM, 0x300 | 8));
        assert_eq!(pokes[2].0, SR_RX_DSP + REG_SCALE_IQ);

        // 3.3 MHz at 32 MHz is between decim 9 and 10; 10 is nearer.
        assert_eq!(dsp.set_host_rate(3.3e6).unwrap(), 3.2e6);
    }

    #[test]
    fn bad_rates_are_refused_before_anything_is_written() {
        let bus = bus();
        let mut dsp = RxDsp::new(Box::new(bus));
        assert!(dsp.set_tick_rate(0.0).is_err());
        assert!(dsp.set_host_rate(f64::NAN).is_err());
        assert!(bus.pokes.lock().unwrap().is_empty());
    }

    #[test]
    fn float_scale_is_one_over_full_scale_at_unity_decimation() {
        let mut dsp = RxDsp::new(Box::new(bus()));
        dsp.set_tick_rate(16e6).unwrap();
        dsp.set_host_rate(16e6).unwrap();
        dsp.setup_sc16().unwrap();
        assert!((dsp.scaling_adjustment() * 32767.0 - 1.0).abs() < 1e-4);
    }
}
