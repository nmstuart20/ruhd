//! Ranges of settable values (frequencies, gains, sample rates, bandwidths).

use std::fmt;
use std::ops::Deref;

use crate::error::{Error, Result};

/// A single inclusive `start..=stop` range with an optional step.
///
/// A `step` of `0.0` means the range is continuous.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Range {
    /// Lowest value in the range.
    pub start: f64,
    /// Highest value in the range.
    pub stop: f64,
    /// Distance between adjacent settable values, or `0.0` if continuous.
    pub step: f64,
}

impl Range {
    /// Builds a range, swapping the bounds if they arrive backwards.
    #[must_use]
    pub fn new(start: f64, stop: f64, step: f64) -> Self {
        let (start, stop) = if start <= stop {
            (start, stop)
        } else {
            (stop, start)
        };
        Self {
            start,
            stop,
            step: step.abs(),
        }
    }

    /// Builds a continuous range with no step constraint.
    #[must_use]
    pub fn continuous(start: f64, stop: f64) -> Self {
        Self::new(start, stop, 0.0)
    }

    /// Builds a range holding exactly one value.
    #[must_use]
    pub fn constant(value: f64) -> Self {
        Self {
            start: value,
            stop: value,
            step: 0.0,
        }
    }

    /// Returns `true` if `value` lies within the bounds, ignoring the step.
    #[must_use]
    pub fn contains(&self, value: f64) -> bool {
        (self.start..=self.stop).contains(&value)
    }

    /// Clamps `value` into the range.
    ///
    /// With `clip_step`, the result is additionally snapped to the grid of
    /// [`Range::step`] multiples measured from [`Range::start`].
    ///
    /// Snapping steps *down* rather than clamping when rounding to nearest would
    /// overshoot `stop`. UHD's `meta_range_t::clip` clamps instead, which can
    /// hand back a value that is off the grid and above the maximum — and then
    /// gets programmed into a register that cannot represent it. Staying on the
    /// grid is the property callers actually rely on.
    #[must_use]
    pub fn clip(&self, value: f64, clip_step: bool) -> f64 {
        let value = value.clamp(self.start, self.stop);
        if !clip_step || self.step == 0.0 {
            return value;
        }
        let steps = ((value - self.start) / self.step).round();
        let snapped = self.start + steps * self.step;
        if snapped > self.stop {
            // Rounding to nearest overshot; take the grid point below.
            (self.start + (steps - 1.0) * self.step).max(self.start)
        } else {
            snapped.max(self.start)
        }
    }
}

impl fmt::Display for Range {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        if self.start == self.stop {
            write!(f, "{}", self.start)
        } else if self.step == 0.0 {
            write!(f, "{}..{}", self.start, self.stop)
        } else {
            write!(f, "{}..{} step {}", self.start, self.stop, self.step)
        }
    }
}

/// A non-empty, ordered collection of [`Range`]s.
///
/// Real hardware is rarely one contiguous span — the AD9361 front end has
/// distinct gain tables per band, and B2xx tuning has gaps — so anything a
/// device advertises as "the settable values" is a `MetaRange`.
#[derive(Debug, Clone, PartialEq)]
pub struct MetaRange {
    ranges: Vec<Range>,
}

impl MetaRange {
    /// Builds a meta-range from one or more sub-ranges, sorted by `start`.
    ///
    /// Returns [`Error::InvalidArg`] if `ranges` is empty: an empty meta-range
    /// has no meaningful `start`, `stop` or `clip`, and letting one exist just
    /// pushes the panic somewhere less obvious.
    pub fn new(ranges: impl IntoIterator<Item = Range>) -> Result<Self> {
        let mut ranges: Vec<Range> = ranges.into_iter().collect();
        if ranges.is_empty() {
            return Err(Error::invalid_arg("a meta-range needs at least one range"));
        }
        ranges.sort_by(|a, b| a.start.total_cmp(&b.start));
        Ok(Self { ranges })
    }

    /// The lowest value across all sub-ranges.
    #[must_use]
    pub fn start(&self) -> f64 {
        self.ranges
            .iter()
            .map(|r| r.start)
            .fold(f64::INFINITY, f64::min)
    }

    /// The highest value across all sub-ranges.
    #[must_use]
    pub fn stop(&self) -> f64 {
        self.ranges
            .iter()
            .map(|r| r.stop)
            .fold(f64::NEG_INFINITY, f64::max)
    }

    /// The smallest non-zero step across all sub-ranges, or `0.0` if every
    /// sub-range is continuous.
    #[must_use]
    pub fn step(&self) -> f64 {
        self.ranges
            .iter()
            .map(|r| r.step)
            .filter(|s| *s > 0.0)
            .min_by(f64::total_cmp)
            .unwrap_or(0.0)
    }

    /// Clamps `value` to the nearest reachable point in any sub-range.
    #[must_use]
    pub fn clip(&self, value: f64, clip_step: bool) -> f64 {
        let mut best = self.ranges[0].clip(value, clip_step);
        let mut best_dist = (best - value).abs();
        for range in &self.ranges[1..] {
            let candidate = range.clip(value, clip_step);
            let dist = (candidate - value).abs();
            if dist < best_dist {
                best = candidate;
                best_dist = dist;
            }
        }
        best
    }

    /// Returns `true` if `value` lies within any sub-range.
    #[must_use]
    pub fn contains(&self, value: f64) -> bool {
        self.ranges.iter().any(|r| r.contains(value))
    }

    /// The sub-ranges, sorted by `start`.
    #[must_use]
    pub fn as_slice(&self) -> &[Range] {
        &self.ranges
    }
}

impl Deref for MetaRange {
    type Target = [Range];
    fn deref(&self) -> &[Range] {
        &self.ranges
    }
}

impl From<Range> for MetaRange {
    fn from(range: Range) -> Self {
        Self {
            ranges: vec![range],
        }
    }
}

impl fmt::Display for MetaRange {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        for (i, r) in self.ranges.iter().enumerate() {
            if i > 0 {
                f.write_str(", ")?;
            }
            write!(f, "{r}")?;
        }
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn clip_clamps_to_bounds() {
        let r = Range::continuous(70e6, 6e9);
        assert_eq!(r.clip(1e6, false), 70e6);
        assert_eq!(r.clip(7e9, false), 6e9);
        assert_eq!(r.clip(2.4e9, false), 2.4e9);
    }

    #[test]
    fn clip_snaps_to_step() {
        // AD9361 RX gain: 0..76 dB in 1 dB steps.
        let r = Range::new(0.0, 76.0, 1.0);
        assert_eq!(r.clip(30.4, true), 30.0);
        assert_eq!(r.clip(30.6, true), 31.0);
    }

    #[test]
    fn snapping_stays_on_the_grid_and_inside_the_bounds() {
        // stop is not itself a grid point: 0, 3, 6, 9 are, 10 is not.
        assert_eq!(Range::new(0.0, 10.0, 3.0).clip(10.0, true), 9.0);
        // Rounding to nearest would give 12; stepping down gives 8.
        assert_eq!(Range::new(0.0, 10.0, 4.0).clip(10.0, true), 8.0);
        // A step coarser than the whole range collapses to start.
        assert_eq!(Range::new(0.0, 1.0, 5.0).clip(1.0, true), 0.0);
    }

    #[test]
    fn meta_range_picks_the_closest_sub_range() {
        let mr =
            MetaRange::new([Range::continuous(0.0, 10.0), Range::continuous(90.0, 100.0)]).unwrap();
        assert_eq!(mr.start(), 0.0);
        assert_eq!(mr.stop(), 100.0);
        // In the gap: 40 is nearer the low range, 60 nearer the high one.
        assert_eq!(mr.clip(40.0, false), 10.0);
        assert_eq!(mr.clip(60.0, false), 90.0);
        assert!(!mr.contains(50.0));
    }

    #[test]
    fn meta_range_step_is_zero_when_all_continuous() {
        let mr = MetaRange::from(Range::continuous(0.0, 1.0));
        assert_eq!(mr.step(), 0.0);
        let mr = MetaRange::new([Range::new(0.0, 1.0, 0.5), Range::new(2.0, 3.0, 0.25)]).unwrap();
        assert_eq!(mr.step(), 0.25);
    }

    #[test]
    fn empty_meta_range_is_rejected() {
        assert!(MetaRange::new([]).is_err());
    }
}
