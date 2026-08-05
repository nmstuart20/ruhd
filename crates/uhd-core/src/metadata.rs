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
}

impl RxMetadata {
    /// Turns a reported [`RxError`] into an `Err`, for callers that would rather
    /// use `?` than inspect the field.
    ///
    /// `Overflow` is deliberately *not* fatal here — it is the one condition a
    /// well-written receive loop is expected to absorb — so it stays `Ok`.
    /// Check [`RxMetadata::error`] directly if you want to count overflows.
    pub fn check(&self) -> Result<()> {
        match self.error {
            None | Some(RxError::Overflow) => Ok(()),
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
    fn overflow_is_not_fatal_but_late_command_is() {
        let md = RxMetadata {
            error: Some(RxError::Overflow),
            ..Default::default()
        };
        assert!(md.check().is_ok());
        assert!(md.overflowed());

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
