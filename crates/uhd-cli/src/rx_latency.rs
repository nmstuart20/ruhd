//! Host latency from first packet copy into a block to completed output write.

use std::time::{Duration, Instant};

#[derive(Clone, Copy, Debug)]
pub(crate) struct BlockTiming {
    pub(crate) first_sample: Instant,
    pub(crate) handed_off: Instant,
}

#[derive(Clone, Copy, Debug, Default)]
pub(crate) struct Timing {
    pub(crate) blocks: u64,
    total_ns: u128,
    pub(crate) max: Duration,
}

impl Timing {
    fn observe(&mut self, duration: Duration) {
        self.blocks += 1;
        self.total_ns += duration.as_nanos();
        self.max = self.max.max(duration);
    }

    pub(crate) fn mean_ms(self) -> Option<f64> {
        (self.blocks > 0).then(|| self.total_ns as f64 / self.blocks as f64 / 1e6)
    }

    fn summary(self) -> String {
        self.mean_ms().map_or_else(
            || "n/a".into(),
            |mean| format!("{mean:.3}/{:.3}", self.max.as_secs_f64() * 1000.0),
        )
    }
}

#[derive(Clone, Copy, Debug, Default)]
pub(crate) struct Latency {
    pub(crate) fill: Timing,
    pub(crate) queue: Timing,
    pub(crate) write: Timing,
    pub(crate) total: Timing,
    pub(crate) failed_writes: u64,
}

impl Latency {
    pub(crate) fn completed(&mut self, timing: BlockTiming, started: Instant, finished: Instant) {
        self.fill
            .observe(timing.handed_off.duration_since(timing.first_sample));
        self.queue
            .observe(started.duration_since(timing.handed_off));
        self.write.observe(finished.duration_since(started));
        self.total
            .observe(finished.duration_since(timing.first_sample));
    }

    pub(crate) fn report(self) {
        eprintln!(
            "  ruhd latency, cumulative mean/max ms ({} completed blocks, {} failed writes):",
            self.total.blocks, self.failed_writes,
        );
        eprintln!(
            "    fill {}; queue {}; write {}; total {}",
            self.fill.summary(),
            self.queue.summary(),
            self.write.summary(),
            self.total.summary(),
        );
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn stages_and_total_use_the_same_block_and_keep_mean_and_max_separate() {
        let epoch = Instant::now();
        let at = |ms| epoch + Duration::from_millis(ms);
        let mut latency = Latency::default();
        assert_eq!(latency.total.mean_ms(), None);
        assert_eq!(latency.total.summary(), "n/a");
        latency.completed(
            BlockTiming {
                first_sample: at(0),
                handed_off: at(10),
            },
            at(30),
            at(35),
        );
        latency.completed(
            BlockTiming {
                first_sample: at(100),
                handed_off: at(130),
            },
            at(140),
            at(155),
        );
        assert_eq!(latency.fill.mean_ms(), Some(20.0));
        assert_eq!(latency.queue.mean_ms(), Some(15.0));
        assert_eq!(latency.write.mean_ms(), Some(10.0));
        assert_eq!(latency.total.mean_ms(), Some(45.0));
        assert_eq!(latency.fill.max, Duration::from_millis(30));
        assert_eq!(latency.queue.max, Duration::from_millis(20));
        assert_eq!(latency.write.max, Duration::from_millis(15));
        assert_eq!(latency.total.max, Duration::from_millis(55));
        assert_eq!(latency.total.blocks, 2);
    }
}
