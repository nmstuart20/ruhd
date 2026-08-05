//! Tune requests and results.
//!
//! Tuning a USRP is two moves, not one: the RF front end mixes to some LO
//! frequency, and a digital down/up-converter in the FPGA shifts the remainder.
//! A [`TuneRequest`] states the target and how much freedom each stage has; a
//! [`TuneResult`] reports where each stage actually landed.

use crate::args::DeviceArgs;

/// How much freedom a tuning stage has.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq, Hash)]
pub enum TunePolicy {
    /// Leave this stage where it is.
    None,
    /// Let the driver choose. This is what you want unless you have a reason.
    #[default]
    Auto,
    /// Use the frequency given in the request verbatim.
    Manual,
}

/// A request to tune a channel to `target_freq`.
///
/// Build one with [`TuneRequest::new`] and adjust from there; the defaults put
/// both stages on [`TunePolicy::Auto`], which is UHD's behaviour for a bare
/// `tune_request_t(freq)`.
#[derive(Debug, Clone, PartialEq)]
pub struct TuneRequest {
    /// The overall centre frequency wanted, in Hz.
    pub target_freq: f64,
    /// Freedom given to the RF front end.
    pub rf_freq_policy: TunePolicy,
    /// RF LO frequency in Hz. Only read when `rf_freq_policy` is
    /// [`TunePolicy::Manual`].
    pub rf_freq: f64,
    /// Freedom given to the digital converter.
    pub dsp_freq_policy: TunePolicy,
    /// DSP shift frequency in Hz. Only read when `dsp_freq_policy` is
    /// [`TunePolicy::Manual`].
    pub dsp_freq: f64,
    /// Backend-specific knobs, e.g. `mode_n=integer`.
    pub args: DeviceArgs,
}

impl TuneRequest {
    /// A request to tune to `target_freq` with both stages on
    /// [`TunePolicy::Auto`].
    #[must_use]
    pub fn new(target_freq: f64) -> Self {
        Self {
            target_freq,
            rf_freq_policy: TunePolicy::Auto,
            rf_freq: 0.0,
            dsp_freq_policy: TunePolicy::Auto,
            dsp_freq: 0.0,
            args: DeviceArgs::new(),
        }
    }

    /// Places the LO `offset` Hz away from the target and lets the DSP make up
    /// the difference.
    ///
    /// This is the standard trick for moving LO leakage and DC offset out of the
    /// band of interest — worth knowing about on the B205mini, whose AD9361 is a
    /// direct-conversion (zero-IF) transceiver and so puts a DC spike right in
    /// the middle of the band by default.
    #[must_use]
    pub fn with_lo_offset(mut self, offset: f64) -> Self {
        self.rf_freq_policy = TunePolicy::Manual;
        self.rf_freq = self.target_freq + offset;
        self.dsp_freq_policy = TunePolicy::Auto;
        self
    }

    /// Pins the RF LO to `rf_freq` exactly.
    #[must_use]
    pub fn with_rf_freq(mut self, rf_freq: f64) -> Self {
        self.rf_freq_policy = TunePolicy::Manual;
        self.rf_freq = rf_freq;
        self
    }

    /// Pins the DSP shift to `dsp_freq` exactly.
    #[must_use]
    pub fn with_dsp_freq(mut self, dsp_freq: f64) -> Self {
        self.dsp_freq_policy = TunePolicy::Manual;
        self.dsp_freq = dsp_freq;
        self
    }

    /// Attaches a backend-specific argument.
    #[must_use]
    pub fn with_arg(mut self, key: impl Into<String>, value: impl Into<String>) -> Self {
        self.args.set(key, value);
        self
    }
}

impl From<f64> for TuneRequest {
    fn from(target_freq: f64) -> Self {
        Self::new(target_freq)
    }
}

/// Where a tune actually landed.
#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct TuneResult {
    /// The requested frequency after clipping to what the hardware can reach.
    pub clipped_rf_freq: f64,
    /// The RF LO frequency the driver asked the front end for.
    pub target_rf_freq: f64,
    /// The RF LO frequency the front end reports.
    pub actual_rf_freq: f64,
    /// The DSP shift the driver asked for.
    pub target_dsp_freq: f64,
    /// The DSP shift actually programmed.
    pub actual_dsp_freq: f64,
}

impl TuneResult {
    /// The centre frequency the channel is really at.
    ///
    /// The DSP shift is *subtracted*: the digital converter moves the band back
    /// down by the amount the LO overshot. This sign convention matches UHD's
    /// `tune_result_t`, and getting it backwards is a classic way to end up
    /// tuned to twice your LO offset.
    #[must_use]
    pub fn actual_freq(&self) -> f64 {
        self.actual_rf_freq - self.actual_dsp_freq
    }

    /// How far the result missed the request, in Hz.
    #[must_use]
    pub fn error(&self, request: &TuneRequest) -> f64 {
        self.actual_freq() - request.target_freq
    }
}

impl std::fmt::Display for TuneResult {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(
            f,
            "actual {:.1} Hz (rf {:.1}, dsp {:.1})",
            self.actual_freq(),
            self.actual_rf_freq,
            self.actual_dsp_freq
        )
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn default_request_is_fully_automatic() {
        let req = TuneRequest::new(2.4e9);
        assert_eq!(req.rf_freq_policy, TunePolicy::Auto);
        assert_eq!(req.dsp_freq_policy, TunePolicy::Auto);
    }

    #[test]
    fn lo_offset_pins_the_rf_stage_above_the_target() {
        let req = TuneRequest::new(100e6).with_lo_offset(1e6);
        assert_eq!(req.rf_freq_policy, TunePolicy::Manual);
        assert_eq!(req.rf_freq, 101e6);
    }

    #[test]
    fn actual_freq_subtracts_the_dsp_shift() {
        let result = TuneResult {
            actual_rf_freq: 101e6,
            actual_dsp_freq: 1e6,
            ..Default::default()
        };
        assert_eq!(result.actual_freq(), 100e6);
        assert_eq!(result.error(&TuneRequest::new(100e6)), 0.0);
    }
}
