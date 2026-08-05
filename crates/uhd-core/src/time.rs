//! Device time, expressed the way the hardware counts it.

use std::fmt;
use std::ops::{Add, AddAssign, Sub, SubAssign};

/// A point in time (or an interval) on the device's clock.
///
/// Stored as whole seconds plus a normalised fractional part in `[0, 1)`, which
/// is what UHD's `time_spec_t` does and for the same reason: a single `f64`
/// loses sub-nanosecond resolution once the device has been up for a few hours,
/// and timed commands care about ticks.
#[derive(Debug, Clone, Copy, Default, PartialEq, PartialOrd)]
pub struct TimeSpec {
    full_secs: i64,
    frac_secs: f64,
}

impl TimeSpec {
    /// The zero instant.
    pub const ZERO: Self = Self {
        full_secs: 0,
        frac_secs: 0.0,
    };

    /// Builds a time spec from whole and fractional seconds, normalising so that
    /// the fractional part lands in `[0, 1)`.
    ///
    /// A non-finite `frac_secs` is treated as zero.
    #[must_use]
    pub fn new(full_secs: i64, frac_secs: f64) -> Self {
        if !frac_secs.is_finite() {
            return Self {
                full_secs,
                frac_secs: 0.0,
            };
        }
        let carry = frac_secs.floor();
        Self {
            full_secs: full_secs.saturating_add(carry as i64),
            frac_secs: frac_secs - carry,
        }
    }

    /// Builds a time spec from a count of seconds.
    #[must_use]
    pub fn from_secs(secs: f64) -> Self {
        Self::new(0, secs)
    }

    /// Builds a time spec from a tick count at `tick_rate` ticks per second.
    #[must_use]
    pub fn from_ticks(ticks: i64, tick_rate: f64) -> Self {
        if tick_rate <= 0.0 || !tick_rate.is_finite() {
            return Self::ZERO;
        }
        let rate = tick_rate as i64;
        // Split before dividing so that large tick counts keep full precision.
        let (full, rem) = if rate > 0 {
            (ticks.div_euclid(rate), ticks.rem_euclid(rate))
        } else {
            (0, ticks)
        };
        Self::new(full, rem as f64 / tick_rate)
    }

    /// The whole-seconds part.
    #[must_use]
    pub fn full_secs(&self) -> i64 {
        self.full_secs
    }

    /// The fractional-seconds part, always in `[0, 1)`.
    #[must_use]
    pub fn frac_secs(&self) -> f64 {
        self.frac_secs
    }

    /// The whole time as seconds. Loses resolution for large values; use
    /// [`TimeSpec::to_ticks`] when talking to hardware.
    #[must_use]
    pub fn as_secs_f64(&self) -> f64 {
        self.full_secs as f64 + self.frac_secs
    }

    /// Converts to a tick count at `tick_rate` ticks per second.
    #[must_use]
    pub fn to_ticks(&self, tick_rate: f64) -> i64 {
        let whole = (self.full_secs as f64 * tick_rate) as i64;
        whole.saturating_add((self.frac_secs * tick_rate).round() as i64)
    }
}

impl Add for TimeSpec {
    type Output = Self;
    fn add(self, rhs: Self) -> Self {
        Self::new(
            self.full_secs.saturating_add(rhs.full_secs),
            self.frac_secs + rhs.frac_secs,
        )
    }
}

impl Sub for TimeSpec {
    type Output = Self;
    fn sub(self, rhs: Self) -> Self {
        Self::new(
            self.full_secs.saturating_sub(rhs.full_secs),
            self.frac_secs - rhs.frac_secs,
        )
    }
}

impl AddAssign for TimeSpec {
    fn add_assign(&mut self, rhs: Self) {
        *self = *self + rhs;
    }
}

impl SubAssign for TimeSpec {
    fn sub_assign(&mut self, rhs: Self) {
        *self = *self - rhs;
    }
}

impl From<std::time::Duration> for TimeSpec {
    fn from(d: std::time::Duration) -> Self {
        Self::new(
            i64::try_from(d.as_secs()).unwrap_or(i64::MAX),
            f64::from(d.subsec_nanos()) / 1e9,
        )
    }
}

impl fmt::Display for TimeSpec {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{}.{:09.0}", self.full_secs, self.frac_secs * 1e9)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn normalises_fractional_overflow() {
        let t = TimeSpec::new(1, 2.25);
        assert_eq!(t.full_secs(), 3);
        assert!((t.frac_secs() - 0.25).abs() < 1e-12);
    }

    #[test]
    fn normalises_negative_fractions() {
        let t = TimeSpec::new(1, -0.25);
        assert_eq!(t.full_secs(), 0);
        assert!((t.frac_secs() - 0.75).abs() < 1e-12);
    }

    #[test]
    fn tick_round_trip_keeps_precision_when_uptime_is_long() {
        // 61.44 MHz is a B2xx master clock rate; 100_000 s is ~27 h of uptime.
        let rate = 61.44e6;
        let t = TimeSpec::new(100_000, 0.5);
        assert_eq!(TimeSpec::from_ticks(t.to_ticks(rate), rate), t);
    }

    #[test]
    fn arithmetic_borrows_across_the_second_boundary() {
        let a = TimeSpec::new(2, 0.25);
        let b = TimeSpec::new(1, 0.75);
        let d = a - b;
        assert_eq!(d.full_secs(), 0);
        assert!((d.frac_secs() - 0.5).abs() < 1e-12);
    }

    #[test]
    fn degenerate_tick_rates_do_not_panic() {
        assert_eq!(TimeSpec::from_ticks(1234, 0.0), TimeSpec::ZERO);
        assert_eq!(TimeSpec::from_ticks(1234, f64::NAN), TimeSpec::ZERO);
    }
}
