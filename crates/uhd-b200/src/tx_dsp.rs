//! B2xx transmit interpolation and CORDIC, ported from `tx_dsp_core_3000.cpp`.

use std::fmt;
use uhd_core::{Error, Result};
use uhd_transport::RegisterIface;

/// Transmit DSP base, `TOREG(184)`.
pub const SR_TX_DSP: u32 = 184 * 4;

/// One radio's transmit DSP. Set the tick rate before configuring it.
pub struct TxDsp {
    bus: Box<dyn RegisterIface>,
    tick_rate: f64,
    freq: f64,
    interp: u32,
    correction: f64,
}

impl fmt::Debug for TxDsp {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("TxDsp")
            .field("tick_rate", &self.tick_rate)
            .field("freq", &self.freq)
            .field("interp", &self.interp)
            .finish_non_exhaustive()
    }
}

impl TxDsp {
    /// Binds the standard transmit DSP without writing registers.
    #[must_use]
    pub fn new(bus: Box<dyn RegisterIface>) -> Self {
        Self {
            bus,
            tick_rate: 1.0,
            freq: 0.0,
            interp: 1,
            correction: 1.0,
        }
    }

    /// Updates the master clock and reapplies the CORDIC frequency.
    pub fn set_tick_rate(&mut self, rate: f64) -> Result<()> {
        if !rate.is_finite() || rate <= 0.0 {
            return Err(Error::invalid_arg("tick rate must be positive and finite"));
        }
        self.tick_rate = rate;
        self.set_freq(self.freq).map(|_| ())
    }

    /// Programs the nearest supported interpolation and its gain compensation.
    pub fn set_host_rate(&mut self, rate: f64) -> Result<f64> {
        if !rate.is_finite() || rate <= 0.0 {
            return Err(Error::invalid_arg(
                "sample rate must be positive and finite",
            ));
        }
        let interp = crate::rx_dsp::decimations(self.tick_rate)
            .into_iter()
            .min_by(|&a, &b| {
                (self.tick_rate / f64::from(a) - rate)
                    .abs()
                    .total_cmp(&(self.tick_rate / f64::from(b) - rate).abs())
            })
            .unwrap_or(1);
        let (word, cic) = interp_word(interp);
        self.bus.poke32(SR_TX_DSP + 8, word)?;
        let gain = f64::from(cic).powi(3);
        let adjustment = 2f64.powf((gain.ln() / 2f64.ln()).ceil()) / (1.648 * gain);
        let target = 65536.0 * adjustment;
        let actual = target.round() as u32;
        self.bus.poke32(SR_TX_DSP + 4, actual)?;
        self.interp = interp;
        self.correction = target / f64::from(actual);
        if cic > 1 && word & 0x300 == 0 {
            tracing::warn!(
                interp,
                "odd TX interpolation uses only the CIC; expect passband rolloff"
            );
        }
        Ok(self.host_rate())
    }

    /// The actual host sample rate.
    #[must_use]
    pub fn host_rate(&self) -> f64 {
        self.tick_rate / f64::from(self.interp)
    }

    /// Programs the physical upconverter frequency, returning its quantized value.
    pub fn set_freq(&mut self, freq: f64) -> Result<f64> {
        if !freq.is_finite() {
            return Err(Error::invalid_arg("DSP frequency must be finite"));
        }
        let (word, actual) = crate::rx_dsp::freq_word(freq, self.tick_rate);
        self.bus.poke32(SR_TX_DSP, word as u32)?;
        self.freq = actual;
        Ok(actual)
    }

    /// The physical upconverter frequency.
    #[must_use]
    pub fn freq(&self) -> f64 {
        self.freq
    }

    /// Scale from normalized floats to sc16 wire integers.
    #[must_use]
    pub fn scaling_adjustment(&self) -> f64 {
        self.correction * 32767.0
    }
}

fn interp_word(interp: u32) -> (u32, u32) {
    let mut cic = interp;
    let mut word = 0;
    // TX enables HB0 in bit 8; RX enables its first half-band in bit 9.
    for bit in [8, 9] {
        if cic % 2 == 0 {
            word |= 1 << bit;
            cic /= 2;
        }
    }
    (word | (cic & 0xff), cic)
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn tx_halfband_order_and_cic_interpolation() {
        assert_eq!(interp_word(1), (1, 1));
        assert_eq!(interp_word(2), (0x101, 1));
        assert_eq!(interp_word(6), (0x103, 3));
        assert_eq!(interp_word(32), (0x308, 8));
        assert_eq!(interp_word(512), (0x380, 128));
    }
}

#[cfg(test)]
mod register_tests {
    use super::*;
    use std::sync::{Arc, Mutex};
    #[derive(Debug, Clone, Default)]
    struct Bus(Arc<Mutex<Vec<(u32, u32)>>>);
    impl RegisterIface for Bus {
        fn poke32(&self, addr: u32, value: u32) -> Result<()> {
            self.0.lock().unwrap().push((addr, value));
            Ok(())
        }
        fn peek32(&self, _: u32) -> Result<u32> {
            Ok(0)
        }
        fn peek64(&self, _: u32) -> Result<u64> {
            Ok(0)
        }
    }
    #[test]
    fn reference_tx_gain_and_frequency_registers() {
        let bus = Bus::default();
        let mut dsp = TxDsp::new(Box::new(bus.clone()));
        dsp.set_tick_rate(30e6).unwrap();
        assert_eq!(dsp.set_host_rate(5e6).unwrap(), 5e6);
        assert_eq!(dsp.set_freq(-7.5e6).unwrap(), -7.5e6);
        let writes = bus.0.lock().unwrap();
        assert!(writes.contains(&(736 + 8, 0x103)));
        // Cubic CIC gain, from tx_dsp_core_3000; RX uses the fourth power.
        assert!(writes.contains(&(736 + 4, 47131)));
        assert_eq!(writes.last(), Some(&(736, 0xc000_0000)));
        assert!((dsp.scaling_adjustment() - 32767.0).abs() < 1.0);
    }
}
