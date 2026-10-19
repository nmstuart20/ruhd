//! Bounded-memory host instrumentation. Receive timings measure blocking
//! calls, including conversion and the initial scheduled wait, not RF latency.

use serde_json::{Value, json};
use std::time::Duration;

#[derive(Debug)]
pub(crate) struct RecvTiming {
    bins: [u64; 65],
    calls: u64,
    total_ns: u128,
    max_ns: u64,
    pub(crate) short_reads: u64,
    pub(crate) empty_reads: u64,
    pub(crate) failed_calls: u64,
}

impl Default for RecvTiming {
    fn default() -> Self {
        Self {
            bins: [0; 65],
            calls: 0,
            total_ns: 0,
            max_ns: 0,
            short_reads: 0,
            empty_reads: 0,
            failed_calls: 0,
        }
    }
}

impl RecvTiming {
    pub(crate) fn observe(&mut self, elapsed: Duration, samples: Option<usize>, requested: usize) {
        let ns = elapsed.as_nanos().min(u128::from(u64::MAX)) as u64;
        let bucket = (u64::BITS - ns.leading_zeros()) as usize;
        self.bins[bucket] += 1;
        self.calls += 1;
        self.total_ns += u128::from(ns);
        self.max_ns = self.max_ns.max(ns);
        match samples {
            Some(0) => self.empty_reads += 1,
            Some(n) if n < requested => self.short_reads += 1,
            None => self.failed_calls += 1,
            _ => {}
        }
    }

    // Logarithmic bins provide conservative percentile bounds, without
    // allocating per receive or retaining an unbounded timing history.
    fn percentile_upper_us(&self, percentile: u64) -> Option<f64> {
        if self.calls == 0 {
            return None;
        }
        let rank = (u128::from(self.calls) * u128::from(percentile)).div_ceil(100);
        let mut count = 0u128;
        for (bucket, n) in self.bins.iter().enumerate() {
            count += u128::from(*n);
            if count >= rank {
                return Some(((1u128 << bucket) - 1) as f64 / 1000.0);
            }
        }
        None
    }

    pub(crate) fn report(&self) -> Value {
        json!({
            "calls": self.calls,
            "short_reads": self.short_reads,
            "empty_reads": self.empty_reads,
            "failed_calls": self.failed_calls,
            "mean_us": (self.calls > 0).then(|| self.total_ns as f64 / self.calls as f64 / 1000.0),
            "max_us": (self.calls > 0).then_some(self.max_ns as f64 / 1000.0),
            "p50_upper_us": self.percentile_upper_us(50),
            "p95_upper_us": self.percentile_upper_us(95),
            "p99_upper_us": self.percentile_upper_us(99),
        })
    }
}

#[derive(Debug, Clone, Copy)]
pub(crate) struct ProcessUsage {
    cpu_seconds: f64,
    peak_rss_bytes: u64,
}

impl ProcessUsage {
    pub(crate) fn now() -> Option<Self> {
        #[cfg(any(target_os = "linux", target_os = "macos"))]
        {
            let mut usage = std::mem::MaybeUninit::<libc::rusage>::uninit();
            // SAFETY: getrusage writes a complete rusage to this valid pointer
            // on success. It is read only after the return code is checked.
            if unsafe { libc::getrusage(libc::RUSAGE_SELF, usage.as_mut_ptr()) } != 0 {
                return None;
            }
            // SAFETY: the successful call above initialized the structure.
            let usage = unsafe { usage.assume_init() };
            let cpu_seconds = usage.ru_utime.tv_sec as f64
                + usage.ru_stime.tv_sec as f64
                + (usage.ru_utime.tv_usec as f64 + usage.ru_stime.tv_usec as f64) / 1e6;
            let rss = u64::try_from(usage.ru_maxrss).ok()?;
            let peak_rss_bytes = if cfg!(target_os = "macos") {
                rss
            } else {
                rss.saturating_mul(1024)
            };
            Some(Self {
                cpu_seconds,
                peak_rss_bytes,
            })
        }
        #[cfg(not(any(target_os = "linux", target_os = "macos")))]
        {
            None
        }
    }

    pub(crate) fn report(start: Option<Self>, end: Option<Self>, wall: Duration) -> Value {
        let cpu = start
            .zip(end)
            .map(|(a, b)| (b.cpu_seconds - a.cpu_seconds).max(0.0));
        let secs = wall.as_secs_f64();
        json!({
            "wall_seconds": secs,
            "cpu_seconds": cpu,
            "cpu_percent": cpu.filter(|_| secs > 0.0).map(|c| 100.0 * c / secs),
            "process_peak_rss_bytes": end.map(|u| u.peak_rss_bytes),
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn resource_usage_has_consistent_units_and_unavailable_values_stay_null() {
        let start = ProcessUsage {
            cpu_seconds: 100.0,
            peak_rss_bytes: 1024,
        };
        let end = ProcessUsage {
            cpu_seconds: 101.0,
            peak_rss_bytes: 2048,
        };
        let report = ProcessUsage::report(Some(start), Some(end), Duration::from_millis(500));
        assert_eq!(report["cpu_seconds"], 1.0);
        assert_eq!(report["cpu_percent"], 200.0);
        assert_eq!(report["process_peak_rss_bytes"], 2048);
        let unavailable = ProcessUsage::report(None, None, Duration::ZERO);
        assert_eq!(unavailable["cpu_seconds"], Value::Null);
        assert_eq!(unavailable["cpu_percent"], Value::Null);
        assert_eq!(unavailable["process_peak_rss_bytes"], Value::Null);
    }

    #[test]
    fn timing_percentiles_are_bounds_and_counters_include_failures() {
        let mut timing = RecvTiming::default();
        assert_eq!(timing.report()["mean_us"], Value::Null);
        for ns in [0, 1, 1000, 1_000_000] {
            timing.observe(Duration::from_nanos(ns), Some(8), 8);
        }
        timing.observe(Duration::from_nanos(2000), Some(2), 8);
        timing.observe(Duration::ZERO, Some(0), 8);
        timing.observe(Duration::ZERO, None, 8);
        let report = timing.report();
        assert_eq!(report["calls"], 7);
        assert_eq!(report["short_reads"], 1);
        assert_eq!(report["empty_reads"], 1);
        assert_eq!(report["failed_calls"], 1);
        assert_eq!(report["p50_upper_us"], 0.001);
        assert_eq!(report["p99_upper_us"], 1048.575);
        assert_eq!(report["max_us"], 1000.0);
    }
}
