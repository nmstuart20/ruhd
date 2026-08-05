//! RX/TX RFPLL arithmetic and the characterized VCO setup table.

use uhd_core::{Error, Result};

use crate::tables::{SYNTH_CAL_LUT, VCO_INDEX};

pub(crate) const RFPLL_REF: f64 = 80e6;
pub(crate) const RFPLL_MODULUS: u32 = 8_388_593;
pub(crate) const RF_VCO_MIN: f64 = 6e9;
pub(crate) const RF_VCO_MAX: f64 = 12e9;

/// The integer/fractional word and output divider for one RF LO request.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct RfPll {
    /// Integer part of the feedback divider.
    pub nint: u16,
    /// Fractional numerator over [`RFPLL_MODULUS`].
    pub nfrac: u32,
    /// Divider exponent written to register 0x005: 0 means /2, 6 means /128.
    pub divider_exp: u8,
    /// Exact VCO frequency produced by the finite divider word.
    pub vco_freq: f64,
    /// Exact LO frequency after the output divider.
    pub lo_freq: f64,
}

impl RfPll {
    /// Ports `_tune_helper`'s RFPLL math exactly, including its 8,388,593
    /// fractional modulus and nearest-integer rounding.
    pub fn solve(requested: f64) -> Result<Self> {
        if !requested.is_finite() || requested <= 0.0 {
            return Err(Error::invalid_arg(format!(
                "ad936x rf frequency {requested}; expected a finite positive value"
            )));
        }

        let (divider_exp, divider, requested_vco) = (0u8..=6)
            .find_map(|exp| {
                let divider = 2u32 << exp;
                let vco = requested * f64::from(divider);
                (RF_VCO_MIN..=RF_VCO_MAX)
                    .contains(&vco)
                    .then_some((exp, divider, vco))
            })
            .ok_or_else(|| {
                Error::invalid_arg(format!(
                    "ad936x rf frequency {requested} Hz cannot place the vco in 6..=12 GHz"
                ))
            })?;

        let ratio = requested_vco / RFPLL_REF;
        let nint = ratio as u16;
        let nfrac = ((ratio - f64::from(nint)) * f64::from(RFPLL_MODULUS)).round() as u32;
        let vco_freq = RFPLL_REF * (f64::from(nint) + f64::from(nfrac) / f64::from(RFPLL_MODULUS));

        Ok(Self {
            nint,
            nfrac,
            divider_exp,
            vco_freq,
            lo_freq: vco_freq / f64::from(divider),
        })
    }
}

/// One characterized row used by `_setup_synth`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) struct SynthConfig(pub(crate) [u8; 12]);

impl SynthConfig {
    pub(crate) fn for_vco(vco_freq: f64) -> Self {
        // The table is descending and each value is a lower boundary. This is
        // deliberately the reference's `>` rather than `>=`.
        let index = VCO_INDEX
            .iter()
            .position(|&boundary| vco_freq > boundary)
            .unwrap_or(VCO_INDEX.len() - 1);
        Self(SYNTH_CAL_LUT[index])
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn startup_los_produce_the_reference_words() {
        let rx = RfPll::solve(800e6).unwrap();
        assert_eq!(rx.divider_exp, 2); // /8
        assert_eq!(rx.nint, 80);
        assert_eq!(rx.nfrac, 0);
        assert_eq!(rx.lo_freq, 800e6);

        let tx = RfPll::solve(850e6).unwrap();
        assert_eq!(tx.divider_exp, 2);
        assert_eq!(tx.nint, 85);
        assert_eq!(tx.nfrac, 0);
        assert_eq!(tx.lo_freq, 850e6);
    }

    #[test]
    fn fractional_tuning_uses_the_uhd_modulus() {
        let pll = RfPll::solve(2_450_123_456.0).unwrap();
        assert_eq!(pll.divider_exp, 1);
        assert_eq!(pll.nint, 122);
        assert_eq!(pll.nfrac, 4_246_078);
        assert!((pll.lo_freq - 2_450_123_456.0).abs() < 5.0);
    }

    #[test]
    fn output_range_is_set_by_the_seven_dividers() {
        assert!(RfPll::solve(RF_VCO_MIN / 128.0).is_ok());
        assert!(RfPll::solve(RF_VCO_MAX / 2.0).is_ok());
        assert!(RfPll::solve(RF_VCO_MIN / 128.0 - 1.0).is_err());
        assert!(RfPll::solve(RF_VCO_MAX / 2.0 + 1.0).is_err());
    }
}
