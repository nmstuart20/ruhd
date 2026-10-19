//! Aggregate discontinuities without counting an error notification and its
//! recovery as two losses. Unrecovered errors remain explicitly unknown.

use uhd_core::{RxError, RxMetadata};

#[derive(Debug, Clone, Copy, Default)]
pub(crate) struct LossStats {
    pub(crate) recovered: u64,
    pub(crate) missing_samples: u64,
    pub(crate) unknown_recovered: u64,
    pub(crate) gap_seconds: f64,
    pending: bool,
}

impl LossStats {
    pub(crate) fn observe(&mut self, n: usize, md: &RxMetadata) {
        if matches!(
            md.error,
            Some(RxError::Overflow | RxError::BadPacket | RxError::TimestampJump)
        ) {
            self.pending = true;
        }
        if n == 0 {
            return;
        }
        if let Some(loss) = md.loss {
            self.recovered += 1;
            if let Some(missing) = loss.missing_samples {
                self.missing_samples = self.missing_samples.saturating_add(missing);
            } else {
                self.unknown_recovered += 1;
            }
            self.gap_seconds += loss.duration_secs().unwrap_or(0.0);
        } else if self.pending {
            // Backends that do not provide accounting still report errors.
            self.recovered += 1;
            self.unknown_recovered += 1;
        }
        self.pending = false;
    }

    pub(crate) fn events(&self) -> u64 {
        self.recovered + u64::from(self.pending)
    }

    pub(crate) fn unknown_events(&self) -> u64 {
        self.unknown_recovered + u64::from(self.pending)
    }

    pub(crate) fn unresolved(&self) -> bool {
        self.pending
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use uhd_core::{RxLoss, RxLossCause};

    #[test]
    fn backends_without_accounting_are_unknown_and_zero_loss_is_valid() {
        let mut stats = LossStats::default();
        stats.observe(
            0,
            &RxMetadata {
                error: Some(RxError::Overflow),
                ..Default::default()
            },
        );
        stats.observe(1, &RxMetadata::default());
        assert_eq!(stats.events(), 1);
        assert_eq!(stats.unknown_events(), 1);
        stats.observe(
            1,
            &RxMetadata {
                loss: Some(RxLoss::new(
                    RxLossCause::SequenceGap,
                    Some(100),
                    Some(100),
                    1e6,
                    4,
                )),
                ..Default::default()
            },
        );
        assert_eq!(stats.events(), 2);
        assert_eq!(stats.unknown_events(), 1);
        assert_eq!(stats.missing_samples, 0);
        assert!(!stats.unresolved());
    }

    #[test]
    fn notifications_coalesce_until_samples_resume_and_unresolved_loss_is_unknown() {
        let mut stats = LossStats::default();
        let error = RxMetadata {
            error: Some(RxError::Overflow),
            ..Default::default()
        };
        stats.observe(0, &error);
        stats.observe(0, &error);
        assert_eq!(stats.events(), 1);
        assert_eq!(stats.unknown_events(), 1);
        stats.observe(
            2,
            &RxMetadata {
                loss: Some(RxLoss::new(
                    RxLossCause::Overflow,
                    Some(100),
                    Some(140),
                    1000.0,
                    4,
                )),
                ..Default::default()
            },
        );
        assert_eq!(stats.events(), 1);
        assert_eq!(stats.missing_samples, 10);
        assert_eq!(stats.unknown_events(), 0);
        assert_eq!(stats.gap_seconds, 0.04);
        stats.observe(0, &error);
        assert_eq!(stats.events(), 2);
        assert_eq!(stats.unknown_events(), 1);
        assert!(stats.unresolved());
    }
}
