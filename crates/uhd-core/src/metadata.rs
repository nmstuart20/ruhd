//! Per-burst metadata attached to received and transmitted samples.

use crate::error::{Error, Result};
use crate::time::TimeSpec;

/// Metadata describing a block of samples returned by
/// [`RxStream::recv`](crate::stream::RxStream::recv).
///
/// Note the division of labour with [`Error`]: conditions that are *part of
/// normal streaming* — an overflow because the host fell behind, a burst that
/// ended — land here, because the caller usually wants to log them and keep
/// going. Conditions that mean the stream is unusable come back as `Err`.
#[derive(Debug, Clone, Default, PartialEq)]
pub struct RxMetadata {
    /// Time of the first sample, if the device provided one.
    pub time_spec: Option<TimeSpec>,
    /// This block is one fragment of a larger packet; more are pending.
    pub more_fragments: bool,
    /// Sample offset of this fragment within its packet.
    pub fragment_offset: usize,
    /// This block begins a burst.
    pub start_of_burst: bool,
    /// This block ends a burst.
    pub end_of_burst: bool,
    /// A packet-sequence discontinuity was seen before this block.
    pub out_of_sequence: bool,
    /// A stream condition the device reported alongside the samples.
    pub error: Option<RxError>,
    /// Discontinuity immediately before this call's first sample. Reported
    /// once, on the first successful receive after recovery, never on a
    /// zero-sample error notification. Several error notifications may refer
    /// to the same discontinuity. `None` does not imply that no errors occurred.
    pub loss: Option<RxLoss>,
}

/// The first observed indication of an RX discontinuity. This describes what
/// was observed, not necessarily the physical cause of the loss.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[non_exhaustive]
pub enum RxLossCause {
    /// The device reported a full receive buffer.
    Overflow,
    /// The packet sequence counter was unexpected.
    SequenceGap,
    /// Device timestamps were discontinuous without a preceding error.
    TimestampJump,
    /// A malformed packet was discarded.
    BadPacket,
    /// A USB or other transport operation failed and reception continued.
    Transport,
}

/// Loss accounting at the boundary between two received spans.
///
/// Timestamps remain integer device ticks so long captures retain sample
/// precision. A count is exact only when both timestamps are available, move
/// forward, and their difference is a whole number of samples. A backward
/// timestamp or an absent timestamp leaves the count unknown; it is never
/// replaced by a guessed sample. A sequence anomaly can have a known count of
/// zero. Deliberate gaps between completed bursts are excluded.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct RxLoss {
    /// First indication of this discontinuity.
    pub cause: RxLossCause,
    /// Timestamp expected for the first sample after the previous span.
    pub expected_ticks: Option<u64>,
    /// Timestamp of the first sample received after recovery.
    pub resumed_ticks: Option<u64>,
    /// Device ticks per second.
    pub tick_rate: f64,
    /// Device ticks between adjacent output samples.
    pub ticks_per_sample: u64,
    /// Exact missing sample count, or `None` when it cannot be established.
    pub missing_samples: Option<u64>,
}

impl RxLoss {
    /// Constructs accounting from integer timestamps. Zero ticks per sample
    /// or an invalid tick rate leaves the sample count unknown.
    #[must_use]
    pub fn new(
        cause: RxLossCause,
        expected_ticks: Option<u64>,
        resumed_ticks: Option<u64>,
        tick_rate: f64,
        ticks_per_sample: u64,
    ) -> Self {
        let missing_samples = expected_ticks
            .zip(resumed_ticks)
            .and_then(|(expected, resumed)| resumed.checked_sub(expected))
            .filter(|delta| {
                tick_rate.is_finite()
                    && tick_rate > 0.0
                    && ticks_per_sample > 0
                    && delta.is_multiple_of(ticks_per_sample)
            })
            .map(|delta| delta / ticks_per_sample);
        Self {
            cause,
            expected_ticks,
            resumed_ticks,
            tick_rate,
            ticks_per_sample,
            missing_samples,
        }
    }

    /// Duration of a forward timestamp gap, in seconds. This can be known
    /// even when the gap does not align to a whole number of samples.
    #[must_use]
    pub fn duration_secs(&self) -> Option<f64> {
        if !self.tick_rate.is_finite() || self.tick_rate <= 0.0 {
            return None;
        }
        self.expected_ticks
            .zip(self.resumed_ticks)
            .and_then(|(expected, resumed)| resumed.checked_sub(expected))
            .map(|delta| delta as f64 / self.tick_rate)
    }
}

/// A stream condition reported in [`RxMetadata::error`].
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
#[non_exhaustive]
pub enum RxError {
    /// No samples arrived before the timeout expired.
    Timeout,
    /// A stream command's time had already passed when the device saw it.
    LateCommand,
    /// A `NumSampsAndMore` chain was broken by a missing follow-up command.
    BrokenChain,
    /// The device produced samples faster than the host consumed them.
    Overflow,
    /// Channels of a multi-channel stream could not be time-aligned.
    Alignment,
    /// A packet could not be parsed.
    BadPacket,
    /// Device timestamps jumped within a burst, without another error
    /// explaining the discontinuity. See [`RxMetadata::loss`] on recovery.
    TimestampJump,
}

impl RxMetadata {
    /// Turns a reported [`RxError`] into an `Err`, for callers that would rather
    /// use `?` than inspect the field.
    ///
    /// `Overflow` and `TimestampJump` are recoverable discontinuities, so
    /// they stay `Ok`. Inspect [`RxMetadata::error`] to count notifications
    /// and [`RxMetadata::loss`] to account for missing samples on recovery.
    pub fn check(&self) -> Result<()> {
        match self.error {
            None | Some(RxError::Overflow | RxError::TimestampJump) => Ok(()),
            Some(e) => Err(Error::Device(format!("receive stream error: {e:?}"))),
        }
    }

    /// Whether the device reported an overflow for this block.
    #[must_use]
    pub fn overflowed(&self) -> bool {
        self.error == Some(RxError::Overflow)
    }
}

/// Metadata controlling a block of samples passed to
/// [`TxStream::send`](crate::stream::TxStream::send).
#[derive(Debug, Clone, Default, PartialEq)]
pub struct TxMetadata {
    /// Transmit at this device time; `None` means "as soon as possible".
    pub time_spec: Option<TimeSpec>,
    /// This block begins a burst.
    pub start_of_burst: bool,
    /// This block ends a burst; the device may power down the transmit chain
    /// after it drains.
    pub end_of_burst: bool,
}

impl TxMetadata {
    /// Metadata for a single self-contained burst sent immediately.
    #[must_use]
    pub fn one_shot() -> Self {
        Self {
            time_spec: None,
            start_of_burst: true,
            end_of_burst: true,
        }
    }

    /// Marks the burst as starting at `time`.
    #[must_use]
    pub fn at(mut self, time: TimeSpec) -> Self {
        self.time_spec = Some(time);
        self
    }
}

/// An asynchronous message from the transmit path, retrieved with
/// [`TxStream::recv_async_msg`](crate::stream::TxStream::recv_async_msg).
#[derive(Debug, Clone, PartialEq)]
pub struct AsyncMetadata {
    /// Channel the message refers to.
    pub channel: usize,
    /// Device time the event occurred, if known.
    pub time_spec: Option<TimeSpec>,
    /// What happened.
    pub event: AsyncEvent,
}

/// The event carried by an [`AsyncMetadata`].
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
#[non_exhaustive]
pub enum AsyncEvent {
    /// A burst was transmitted and acknowledged.
    BurstAck,
    /// The transmit buffer ran dry between bursts.
    Underflow,
    /// The transmit buffer ran dry mid-packet.
    UnderflowInPacket,
    /// A packet arrived out of sequence.
    SeqError,
    /// A packet arrived out of sequence inside a burst.
    SeqErrorInBurst,
    /// A packet's transmit time had already passed.
    TimeError,
    /// A user-defined payload from custom FPGA logic.
    UserPayload([u32; 4]),
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn loss_keeps_integer_precision_and_never_guesses_a_count() {
        let expected = (1u64 << 54) + 3;
        let loss = RxLoss::new(
            RxLossCause::SequenceGap,
            Some(expected),
            Some(expected + 320),
            32e6,
            32,
        );
        assert_eq!(loss.missing_samples, Some(10));
        assert_eq!(loss.duration_secs(), Some(0.00001));
        for (before, after, ticks) in [
            (Some(100), None, 4),
            (None, Some(100), 4),
            (Some(100), Some(99), 4),
            (Some(100), Some(103), 4),
            (Some(100), Some(104), 0),
        ] {
            let loss = RxLoss::new(RxLossCause::Overflow, before, after, 1e6, ticks);
            assert_eq!(loss.missing_samples, None);
        }
        let zero = RxLoss::new(RxLossCause::SequenceGap, Some(100), Some(100), 1e6, 4);
        assert_eq!(zero.missing_samples, Some(0));
        assert_eq!(zero.duration_secs(), Some(0.0));
        for rate in [0.0, f64::NAN, f64::INFINITY] {
            let loss = RxLoss::new(RxLossCause::Overflow, Some(0), Some(4), rate, 4);
            assert_eq!(loss.missing_samples, None);
            assert_eq!(loss.duration_secs(), None);
        }
    }

    #[test]
    fn overflow_is_not_fatal_but_late_command_is() {
        let md = RxMetadata {
            error: Some(RxError::Overflow),
            ..Default::default()
        };
        assert!(md.check().is_ok());
        assert!(md.overflowed());
        let md = RxMetadata {
            error: Some(RxError::TimestampJump),
            ..Default::default()
        };
        assert!(md.check().is_ok());
        assert!(!md.overflowed());

        let md = RxMetadata {
            error: Some(RxError::LateCommand),
            ..Default::default()
        };
        assert!(md.check().is_err());
    }

    #[test]
    fn one_shot_marks_both_burst_edges() {
        let md = TxMetadata::one_shot().at(TimeSpec::from_secs(1.5));
        assert!(md.start_of_burst && md.end_of_burst);
        assert_eq!(md.time_spec, Some(TimeSpec::from_secs(1.5)));
    }
}
