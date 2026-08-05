//! The clock chain, as arithmetic.
//!
//! Ports the calculations in `_setup_rates` (`ad9361_device.cpp:1416-1573`) and
//! `_tune_bbvco` (`1201-1278`). Nothing here touches the part: every function is
//! pure, so the numbers the driver is about to program can be checked against
//! the reference implementation without a board, and a rate that comes out wrong
//! can be bisected without a USB capture.
//!
//! # The chain
//!
//! ```text
//!   40 MHz TCXO
//!        |                       _tune_bbvco
//!        +--> BBPLL: Fvco = 40 MHz * (Nint + Nfrac / 2088960)
//!                     |          672 MHz <= Fvco <= 1430 MHz
//!                     +--> / 2^i (i = 1..6)  --> ADC clock
//!                                                  |
//!                                                  +--> / 1 or / 2 --> DAC clock
//!                                                  |
//!                                                  +--> / divfactor --> sample rate
//! ```
//!
//! The driver works this backwards. It is handed a sample rate; it looks that
//! rate up in a table ([`RateProfile`]) which fixes the total decimation
//! `divfactor` and the per-stage filter settings that make it up; it multiplies
//! the rate by `divfactor` to get the required ADC clock; then it searches for
//! the VCO divider that puts the resulting VCO rate inside the PLL's range and
//! solves the fractional-N divide word for what is left.
//!
//! # Why the table and not a search
//!
//! Several `divfactor` values can serve the same rate, and the choice decides
//! which halfband filters are in circuit, how much of the FIR's tap budget is
//! available and whether the DAC ends up overclocked. UHD picked one per band
//! and has shipped it for a decade; every calibration constant downstream was
//! measured against those choices. Reproducing the table exactly, boundaries
//! included, is the whole point — a "better" partition would be a different
//! radio.

use tracing::trace;
use uhd_core::{Error, Result};

/// Reference clock into the BBPLL, in Hz.
///
/// `_tune_bbvco` hardcodes 40 MHz and so does this. It is not the part's only
/// legal reference — the AD9361 will take 10 to 80 MHz — but it is the only one
/// the reference driver's arithmetic admits, and every B2xx carries a 40 MHz
/// TCXO. A board with a different crystal would need this to become a parameter
/// *and* would invalidate the register sequences that were characterised at
/// 40 MHz, so it stays a constant until such a board exists.
pub const REFERENCE_FREQ: f64 = 40e6;

/// Denominator of the BBPLL's fractional divide word.
pub const BBPLL_MODULUS: i32 = 2_088_960;

/// [`BBPLL_MODULUS`] as the `f64` the divide-word arithmetic uses. Exact: it is
/// far below 2^53.
const MODULUS: f64 = BBPLL_MODULUS as f64;

/// Lowest frequency the BBPLL's VCO will run at, in Hz.
pub const BBPLL_VCO_MIN: f64 = 672e6;

/// Highest frequency the BBPLL's VCO will run at, in Hz.
pub const BBPLL_VCO_MAX: f64 = 1430e6;

/// Charge pump current at [`ICP_BASELINE_FREQ`], in amps.
const ICP_BASELINE: f64 = 150e-6;

/// VCO rate the charge pump current is specified at, in Hz.
const ICP_BASELINE_FREQ: f64 = 1280e6;

/// Step size of the charge pump current register, in amps.
const ICP_STEP: f64 = 25e-6;

/// Above this ADC clock the DAC is run at half the ADC rate, in Hz.
pub const DAC_HALF_RATE_ABOVE: f64 = 336e6;

/// ADI's current recommendation for the maximum DAC clock, in Hz.
///
/// The reference driver runs past this deliberately — see [`ChainRates::derive`].
pub const DAC_RECOMMENDED_MAX: f64 = 320e6;

/// ADI's current recommendation for the maximum ADC clock, in Hz.
pub const ADC_RECOMMENDED_MAX: f64 = 640e6;

/// Compares two frequencies the way the reference driver does.
///
/// `freq_is_nearly_equal`, `ad9361_device.cpp:51-54`: equal if they are within
/// 1 Hz of each other. It exists because a rate that has been through a
/// double-precision divide and multiply does not come back bit-identical, and
/// re-tuning on that difference would mean re-running every calibration for
/// nothing.
#[must_use]
pub fn freq_is_nearly_equal(a: f64, b: f64) -> bool {
    a.max(b) - a.min(b) < 1.0
}

/// The filter and divider settings for one band of sample rates.
///
/// One row of the `if / else if` ladder in `_setup_rates`
/// (`ad9361_device.cpp:1432-1506`). The two register bytes are reproduced
/// exactly, including the enable bits in 7:6 that `set_active_chains` will
/// immediately overwrite: the reference turns every chain on across a rate
/// change because the calibrations that follow need them all running, and
/// restores the user's selection afterwards.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct RateProfile {
    /// Value for [`RX_ENABLE_AND_FILTER_CTRL`](crate::regs::RX_ENABLE_AND_FILTER_CTRL).
    pub rxfilt: u8,
    /// Value for [`TX_ENABLE_AND_FILTER_CTRL`](crate::regs::TX_ENABLE_AND_FILTER_CTRL).
    pub txfilt: u8,
    /// Total decimation from the ADC clock down to the sample rate.
    pub divfactor: u32,
    /// TX FIR interpolation, 1, 2 or 4.
    pub tfir_factor: i32,
    /// RX FIR decimation, 1, 2 or 4.
    pub rfir_factor: i32,
}

impl RateProfile {
    /// Looks a sample rate up in the table.
    ///
    /// The comment on each arm is the reference's own, and the four numbers in
    /// it are the decimation (or interpolation) of the four stages the register
    /// byte configures: HB3/DEC3, HB2, HB1 and the FIR.
    ///
    /// # Errors
    ///
    /// [`Error::InvalidArg`] for a rate above 61.44 MHz, which is where the
    /// reference throws `INVALID_CODE_PATH`. Note that there is no lower bound
    /// here — a rate below the part's minimum falls into the first arm and then
    /// fails in [`BbPll::solve`], where the VCO runs out of range. That is the
    /// reference's behaviour and the error it produces is the more informative
    /// one.
    pub fn for_rate(rate: f64) -> Result<Self> {
        // ad9361_device.cpp:1432-1506. Register bytes given as the reference
        // gives them, in binary, so they can be read against the bit fields.
        let profile = if rate < 0.33e6 {
            Self {
                // RX1 + RX2 enabled, 3, 2, 2, 4
                rxfilt: 0b1110_1111,
                // TX1 + TX2 enabled, 3, 2, 2, 4
                txfilt: 0b1110_1111,
                divfactor: 48,
                tfir_factor: 4,
                rfir_factor: 4,
            }
        } else if rate < 0.66e6 {
            Self {
                // RX1 + RX2 enabled, 2, 2, 2, 4
                rxfilt: 0b1101_1111,
                // TX1 + TX2 enabled, 2, 2, 2, 4
                txfilt: 0b1101_1111,
                divfactor: 32,
                tfir_factor: 4,
                rfir_factor: 4,
            }
        } else if rate <= 20e6 {
            Self {
                // RX1 + RX2 enabled, 2, 2, 2, 2
                rxfilt: 0b1101_1110,
                // TX1 + TX2 enabled, 2, 2, 2, 2
                txfilt: 0b1101_1110,
                divfactor: 16,
                tfir_factor: 2,
                rfir_factor: 2,
            }
        } else if rate < 23e6 {
            Self {
                // RX1 + RX2 enabled, 3, 2, 2, 2
                rxfilt: 0b1110_1110,
                // TX1 + TX2 enabled, 3, 1, 2, 2
                txfilt: 0b1110_0110,
                divfactor: 24,
                tfir_factor: 2,
                rfir_factor: 2,
            }
        } else if rate < 41e6 {
            Self {
                // RX1 + RX2 enabled, 2, 2, 2, 2
                rxfilt: 0b1101_1110,
                // TX1 + TX2 enabled, 1, 2, 2, 2
                txfilt: 0b1100_1110,
                divfactor: 16,
                tfir_factor: 2,
                rfir_factor: 2,
            }
        } else if rate <= 58e6 {
            Self {
                // RX1 + RX2 enabled, 3, 1, 2, 2
                rxfilt: 0b1110_0110,
                // TX1 + TX2 enabled, 3, 1, 1, 2
                txfilt: 0b1110_0010,
                divfactor: 12,
                tfir_factor: 2,
                rfir_factor: 2,
            }
        } else if rate <= 61.44e6 {
            Self {
                // RX1 + RX2 enabled, 2, 1, 2, 2
                rxfilt: 0b1100_1110,
                // TX1 + TX2 enabled, 2, 1, 1, 2
                txfilt: 0b1101_0010,
                divfactor: 8,
                tfir_factor: 2,
                rfir_factor: 2,
            }
        } else {
            return Err(Error::invalid_arg(format!(
                "ad936x sample rate {} MHz is above the part's 61.44 MHz maximum",
                rate / 1e6
            )));
        };
        Ok(profile)
    }

    /// The ADC clock this profile needs to produce `rate`, in Hz.
    #[must_use]
    pub fn core_clock(&self, rate: f64) -> f64 {
        rate * f64::from(self.divfactor)
    }
}

/// A solved BBPLL divide word.
///
/// The arithmetic half of `_tune_bbvco` (`ad9361_device.cpp:1212-1254`), with the
/// register writes left to the caller.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct BbPll {
    /// The exponent `i` in `vcodiv = 1 << i`, and the value that goes into bits
    /// 2:0 of [`BBPLL`](crate::regs::BBPLL).
    pub vcodiv_exp: u8,
    /// The VCO divider itself, `1 << vcodiv_exp`.
    pub vcodiv: u32,
    /// Integer part of the divide word.
    pub nint: i32,
    /// Fractional part of the divide word, over [`BBPLL_MODULUS`].
    pub nfrac: i32,
    /// Charge pump current register value, before masking to 6 bits.
    pub icp_reg: i32,
    /// The VCO frequency the solved word actually produces, in Hz.
    ///
    /// Not the requested one: the fractional-N word is finite, so this is what
    /// the rest of the driver has to believe. `_bbpll_freq`.
    pub vco_freq: f64,
    /// The ADC clock that falls out of it, in Hz. `_adcclock_freq`.
    pub adc_freq: f64,
}

impl BbPll {
    /// Solves the divide word for a requested ADC clock.
    ///
    /// # Errors
    ///
    /// [`Error::InvalidArg`] if no VCO divider from 2 to 64 puts the VCO inside
    /// [`BBPLL_VCO_MIN`]..=[`BBPLL_VCO_MAX`] — the reference's
    /// `"_tune_bbvco: wrong vcorate"`. In practice this is what rejects a sample
    /// rate below the part's 220 kHz minimum: 220 kHz needs a 10.56 MHz core
    /// clock, and 10.56 MHz times the largest divider is 675.84 MHz, three MHz
    /// above the VCO's floor. Anything slower has nowhere to go.
    pub fn solve(core_clock: f64) -> Result<Self> {
        // Iterate over VCO dividers until an appropriate divider is found.
        // ad9361_device.cpp:1219-1229.
        let (vcodiv_exp, vcodiv, vcorate) = (1u8..=6)
            .map(|i| (i, 1u32 << i))
            .map(|(i, div)| (i, div, core_clock * f64::from(div)))
            .find(|&(_, _, vcorate)| (BBPLL_VCO_MIN..=BBPLL_VCO_MAX).contains(&vcorate))
            .ok_or_else(|| {
                Error::invalid_arg(format!(
                    "ad936x bbpll cannot reach a {} MHz core clock: no divider from 2 to 64 \
                     puts the vco between {} and {} MHz",
                    core_clock / 1e6,
                    BBPLL_VCO_MIN / 1e6,
                    BBPLL_VCO_MAX / 1e6
                ))
            })?;

        // Fo = Fref * (Nint + Nfrac / mod). The truncation and the round are
        // the reference's `static_cast<int>` and `std::lround`; both are
        // reproduced exactly, because a divide word that is one LSB out is a
        // clock that is a few Hz out, and the tick rate the host computes from
        // it would no longer be the one the part is running.
        let n = vcorate / REFERENCE_FREQ;
        let nint = n as i32;
        let nfrac = ((n - f64::from(nint)) * MODULUS).round() as i32;
        let vco_freq = REFERENCE_FREQ * (f64::from(nint) + f64::from(nfrac) / MODULUS);

        // Scale CP current according to VCO rate.
        let icp = ICP_BASELINE * (vco_freq / ICP_BASELINE_FREQ);
        let icp_reg = (icp / ICP_STEP) as i32 - 1;

        let solved = Self {
            vcodiv_exp,
            vcodiv,
            nint,
            nfrac,
            icp_reg,
            vco_freq,
            adc_freq: vco_freq / f64::from(vcodiv),
        };
        trace!(
            core_clock,
            vcodiv,
            nint,
            nfrac,
            vco_freq,
            adc_freq = solved.adc_freq,
            "bbpll divide word solved"
        );
        Ok(solved)
    }
}

/// Everything `_setup_rates` derives once the ADC clock is known.
///
/// Split out from the BBPLL solution because on a repeated rate request the
/// reference does *not* re-solve the PLL — `_tune_bbvco` returns the cached
/// `_adcclock_freq` — and everything here is still recomputed from it. Feeding
/// this the ADC clock rather than the requested rate is what keeps the two paths
/// identical.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ChainRates {
    /// The ADC clock, in Hz.
    pub adc_freq: f64,
    /// The DAC clock, in Hz: the ADC clock or half of it.
    pub dac_freq: f64,
    /// Whether the DAC is running at half the ADC clock — bit 3 of
    /// [`BBPLL`](crate::regs::BBPLL).
    pub halve_dac: bool,
    /// The resulting baseband bandwidth, which is also the rate the part will
    /// actually clock samples at. `_baseband_bw`, and what `get_clock_rate`
    /// returns.
    pub baseband_bw: f64,
    /// Number of TX FIR taps to program.
    pub num_tx_taps: usize,
    /// Number of RX FIR taps to program.
    pub num_rx_taps: usize,
}

impl ChainRates {
    /// Derives the rest of the chain. `ad9361_device.cpp:1512-1567`.
    ///
    /// # Overclocking
    ///
    /// ADI's current recommendation is 640 MHz for the ADC and 320 MHz for the
    /// DAC, and the 336 MHz threshold below leaves the B2xx over both at the top
    /// of its rate range. UHD keeps the older numbers on purpose — its comment
    /// says the risk of changing them is judged higher than the benefit, since
    /// every calibration in the driver was characterised against them — and logs
    /// when it exceeds them so that suboptimal RF performance can be traced back.
    /// Same here, for the same reason: this is a port, not a redesign.
    #[must_use]
    pub fn derive(rate: f64, profile: &RateProfile, adc_freq: f64) -> Self {
        let halve_dac = adc_freq > DAC_HALF_RATE_ABOVE;
        let dac_freq = if halve_dac { adc_freq / 2.0 } else { adc_freq };

        if dac_freq > DAC_RECOMMENDED_MAX {
            trace!(dac_freq, "ad936x dac rate is overclocked");
        }
        // The reference tests the *DAC* clock against the ADC's limit here and
        // then prints the ADC clock (ad9361_device.cpp:1534-1538). Kept as-is:
        // it is a log line, and a port that quietly fixes conditions is a port
        // whose divergences cannot be enumerated.
        if dac_freq > ADC_RECOMMENDED_MAX {
            trace!(adc_freq, "ad936x adc rate is overclocked");
        }

        Self {
            adc_freq,
            dac_freq,
            halve_dac,
            baseband_bw: adc_freq / f64::from(profile.divfactor),
            num_tx_taps: num_taps(max_tx_taps(dac_freq, rate, profile.tfir_factor)),
            num_rx_taps: num_taps(max_rx_taps(adc_freq, rate)),
        }
    }
}

/// How many TX FIR taps the clock ratio leaves room for.
///
/// The FIRs compute 16 taps per clock cycle, so the tap budget is 16 times the
/// ratio of the converter clock to the data rate. The TX FIR is additionally
/// capped at 64 taps in 1x interpolation mode. `ad9361_device.cpp:1560-1562`.
#[must_use]
pub fn max_tx_taps(dac_freq: f64, rate: f64, tfir_factor: i32) -> usize {
    let by_clock = 16 * clocks_per_sample(dac_freq, rate);
    let by_mode = if tfir_factor == 1 { 64 } else { 128 };
    by_clock.min(128).min(by_mode) as usize
}

/// How many RX FIR taps the clock ratio leaves room for.
///
/// As [`max_tx_taps`], without the interpolation-mode cap: the RX FIR always has
/// memory for 128. `ad9361_device.cpp:1563-1564`.
#[must_use]
pub fn max_rx_taps(adc_freq: f64, rate: f64) -> usize {
    (16 * clocks_per_sample(adc_freq, rate)).min(128) as usize
}

/// `(int)((clock / rate) + 0.5)`, the reference's rounding.
///
/// Adding a half and truncating is *not* `round`: it rounds a negative halfway
/// case the other way. The distinction cannot arise here — both arguments are
/// positive — but reproducing the expression rather than its intent is the
/// cheaper thing to keep right. The clamp at zero replaces C++'s undefined
/// behaviour on a negative cast; it can only be reached with a rate large enough
/// to have already been rejected.
fn clocks_per_sample(clock: f64, rate: f64) -> i64 {
    (((clock / rate) + 0.5) as i64).max(0)
}

/// Rounds a tap budget down to a tap count the part will accept.
///
/// `get_num_taps`, `ad9361_device.cpp:62-80`. The AD936x's FIRs come in
/// multiples of 16 up to 128; this picks the largest that fits. Two edges are
/// the reference's and are kept:
///
/// - a budget of 128 or more returns 128, via the `num_taps == 0` fallback after
///   the loop runs off the end;
/// - a budget below 16 still returns 16, because the loop breaks at `i == 1` and
///   takes the entry *before* it. There is no smaller filter to fall back to.
#[must_use]
pub fn num_taps(max_num_taps: usize) -> usize {
    const NUM_TAPS_LIST: [usize; 8] = [16, 32, 48, 64, 80, 96, 112, 128];

    for i in 1..NUM_TAPS_LIST.len() {
        if max_num_taps >= NUM_TAPS_LIST[i] {
            continue;
        }
        return NUM_TAPS_LIST[i - 1];
    }
    128
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Every boundary in the `_setup_rates` ladder, on both sides.
    ///
    /// The table is the one thing in this file that cannot be derived from
    /// anything, so it is checked value by value against the C++ rather than
    /// spot-checked. A wrong byte here is a filter chain that is not the one the
    /// gain and calibration tables were measured with.
    #[test]
    fn the_rate_table_matches_the_reference_row_for_row() {
        let rows: &[(f64, u8, u8, u32, i32, i32)] = &[
            // rate, rxfilt, txfilt, divfactor, tfir, rfir
            (0.0, 0xef, 0xef, 48, 4, 4),
            (0.22e6, 0xef, 0xef, 48, 4, 4),
            (0.329_999e6, 0xef, 0xef, 48, 4, 4),
            (0.33e6, 0xdf, 0xdf, 32, 4, 4),
            (0.659_999e6, 0xdf, 0xdf, 32, 4, 4),
            (0.66e6, 0xde, 0xde, 16, 2, 2),
            (20e6, 0xde, 0xde, 16, 2, 2),
            (20.000_001e6, 0xee, 0xe6, 24, 2, 2),
            (22.999_999e6, 0xee, 0xe6, 24, 2, 2),
            (23e6, 0xde, 0xce, 16, 2, 2),
            (40.999_999e6, 0xde, 0xce, 16, 2, 2),
            (41e6, 0xe6, 0xe2, 12, 2, 2),
            (58e6, 0xe6, 0xe2, 12, 2, 2),
            (58.000_001e6, 0xce, 0xd2, 8, 2, 2),
            (61.44e6, 0xce, 0xd2, 8, 2, 2),
        ];

        for &(rate, rxfilt, txfilt, divfactor, tfir_factor, rfir_factor) in rows {
            assert_eq!(
                RateProfile::for_rate(rate).unwrap(),
                RateProfile {
                    rxfilt,
                    txfilt,
                    divfactor,
                    tfir_factor,
                    rfir_factor,
                },
                "rate {rate}"
            );
        }
    }

    #[test]
    fn a_rate_above_the_parts_maximum_is_rejected() {
        let error = RateProfile::for_rate(61.44e6 + 1.0)
            .unwrap_err()
            .to_string();
        assert!(error.contains("61.44"), "{error}");
    }

    #[test]
    fn the_default_initialisation_rate_solves_to_known_values() {
        // 50 MHz is what `initialize` brings the part up at, so this is the one
        // solution every B2xx passes through. Worked by hand from the C++:
        // divfactor 12 -> 600 MHz core -> vcodiv 2 -> 1200 MHz VCO, which is an
        // exact multiple of the 40 MHz reference, so Nfrac is zero.
        let profile = RateProfile::for_rate(50e6).unwrap();
        assert_eq!(profile.divfactor, 12);
        assert_eq!(profile.core_clock(50e6), 600e6);

        let pll = BbPll::solve(600e6).unwrap();
        assert_eq!(pll.vcodiv_exp, 1);
        assert_eq!(pll.vcodiv, 2);
        assert_eq!(pll.nint, 30);
        assert_eq!(pll.nfrac, 0);
        assert_eq!(pll.vco_freq, 1200e6);
        assert_eq!(pll.adc_freq, 600e6);
        // icp = 150 uA * 1200/1280 = 140.625 uA; 140.625 / 25 truncates to 5.
        assert_eq!(pll.icp_reg, 4);

        let rates = ChainRates::derive(50e6, &profile, pll.adc_freq);
        assert!(rates.halve_dac, "600 MHz is above the 336 MHz threshold");
        assert_eq!(rates.dac_freq, 300e6);
        assert_eq!(rates.baseband_bw, 50e6);
        // 16 * trunc(300/50 + 0.5) = 96 taps of TX budget, 16 * trunc(12.5) =
        // 192 of RX budget, capped at 128.
        assert_eq!(rates.num_tx_taps, 96);
        assert_eq!(rates.num_rx_taps, 128);
    }

    #[test]
    fn the_top_of_the_rate_range_needs_a_fractional_divide_word() {
        // 61.44 MHz: divfactor 8 -> 491.52 MHz core -> vcodiv 2 -> 983.04 MHz,
        // which is 24.576 * 40 MHz. Nfrac = round(0.576 * 2088960).
        let pll =
            BbPll::solve(RateProfile::for_rate(61.44e6).unwrap().core_clock(61.44e6)).unwrap();
        assert_eq!(pll.nint, 24);
        assert_eq!(pll.nfrac, 1_203_241);
        // The word is finite, so the VCO lands near 983.04 MHz rather than on
        // it. Within a Hz is what the reference's own equality test calls the
        // same frequency.
        assert!(freq_is_nearly_equal(pll.vco_freq, 983.04e6), "{pll:?}");
    }

    #[test]
    fn the_vco_divider_search_walks_up_from_two() {
        // A 1 MHz rate needs a 16 MHz core clock, and only the largest divider
        // gets the VCO above its floor.
        let pll = BbPll::solve(16e6).unwrap();
        assert_eq!(pll.vcodiv, 64);
        assert_eq!(pll.vco_freq, 1024e6);
        assert_eq!(pll.adc_freq, 16e6);
    }

    #[test]
    fn the_slowest_rate_the_part_supports_only_just_fits_the_vco() {
        // AD9361_MIN_CLOCK_RATE is 220 kHz, and this is why: at divfactor 48 it
        // asks for a 675.84 MHz VCO against a 672 MHz floor. A few kHz slower
        // and the search runs out.
        let profile = RateProfile::for_rate(220e3).unwrap();
        let pll = BbPll::solve(profile.core_clock(220e3)).unwrap();
        assert_eq!(pll.vcodiv, 64);
        assert!(pll.vco_freq >= BBPLL_VCO_MIN, "{pll:?}");

        let too_slow = RateProfile::for_rate(200e3).unwrap();
        let error = BbPll::solve(too_slow.core_clock(200e3))
            .unwrap_err()
            .to_string();
        assert!(error.contains("no divider"), "{error}");
    }

    #[test]
    fn a_slow_rate_leaves_the_dac_alone() {
        let profile = RateProfile::for_rate(1e6).unwrap();
        let rates = ChainRates::derive(1e6, &profile, 16e6);
        assert!(!rates.halve_dac);
        assert_eq!(rates.dac_freq, 16e6);
        assert_eq!(rates.baseband_bw, 1e6);
        // 16 * 16 = 256 and 16 * 16 = 256, both capped to 128.
        assert_eq!(rates.num_tx_taps, 128);
        assert_eq!(rates.num_rx_taps, 128);
    }

    #[test]
    fn the_dac_threshold_is_exclusive() {
        let profile = RateProfile::for_rate(21e6).unwrap();
        assert!(!ChainRates::derive(21e6, &profile, 336e6).halve_dac);
        assert!(ChainRates::derive(21e6, &profile, 336.000_001e6).halve_dac);
    }

    #[test]
    fn tap_budgets_round_the_way_the_reference_rounds() {
        // trunc(x + 0.5): the budget steps up at the half, so a ratio of 6.49
        // still buys six clocks' worth of taps and 6.5 buys seven.
        assert_eq!(max_rx_taps(600e6, 100e6), 96);
        assert_eq!(max_rx_taps(649e6, 100e6), 96);
        assert_eq!(max_rx_taps(650e6, 100e6), 112);
        assert_eq!(max_rx_taps(651e6, 100e6), 112);
        // And the budget is capped at the 128 taps the part has memory for,
        // however fast the converter runs.
        assert_eq!(max_rx_taps(6000e6, 100e6), 128);
        // 1x interpolation caps the TX FIR at 64 taps however much room the
        // clock ratio leaves.
        assert_eq!(max_tx_taps(600e6, 10e6, 1), 64);
        assert_eq!(max_tx_taps(600e6, 10e6, 2), 128);
    }

    #[test]
    fn tap_counts_round_down_to_a_multiple_of_sixteen() {
        // get_num_taps: the largest legal count that fits, with both of the
        // reference's edges.
        assert_eq!(num_taps(0), 16);
        assert_eq!(num_taps(15), 16);
        assert_eq!(num_taps(16), 16);
        assert_eq!(num_taps(31), 16);
        assert_eq!(num_taps(32), 32);
        assert_eq!(num_taps(95), 80);
        assert_eq!(num_taps(96), 96);
        assert_eq!(num_taps(127), 112);
        assert_eq!(num_taps(128), 128);
        assert_eq!(num_taps(usize::MAX), 128);
    }

    #[test]
    fn nearly_equal_is_a_one_hertz_window() {
        assert!(freq_is_nearly_equal(50e6, 50e6));
        assert!(freq_is_nearly_equal(50e6, 50e6 + 0.99));
        assert!(!freq_is_nearly_equal(50e6, 50e6 + 1.0));
        assert!(!freq_is_nearly_equal(50e6 + 1.0, 50e6));
        assert!(!freq_is_nearly_equal(0.0, 50e6));
    }

    /// Every rate a B2xx is likely to be asked for has to survive the whole
    /// chain, and the rate that comes back has to be one the host can use as a
    /// tick rate.
    #[test]
    fn the_common_rates_all_solve() {
        for rate in [
            0.25e6, 0.5e6, 1e6, 2e6, 4e6, 5e6, 8e6, 10e6, 15.36e6, 16e6, 20e6, 22e6, 25e6, 30.72e6,
            32e6, 40e6, 50e6, 56e6, 61.44e6,
        ] {
            let profile = RateProfile::for_rate(rate).unwrap();
            let pll = BbPll::solve(profile.core_clock(rate)).unwrap();
            let rates = ChainRates::derive(rate, &profile, pll.adc_freq);

            assert!(
                freq_is_nearly_equal(rates.baseband_bw, rate),
                "{rate} came back as {}",
                rates.baseband_bw
            );
            assert!(
                (BBPLL_VCO_MIN..=BBPLL_VCO_MAX).contains(&pll.vco_freq),
                "{rate}: vco {}",
                pll.vco_freq
            );
            assert!(
                matches!(rates.num_tx_taps, 48 | 64 | 96 | 128),
                "{rate}: {} tx taps has no filter",
                rates.num_tx_taps
            );
            assert!(
                matches!(rates.num_rx_taps, 48 | 64 | 96 | 128),
                "{rate}: {} rx taps has no filter",
                rates.num_rx_taps
            );
        }
    }
}
