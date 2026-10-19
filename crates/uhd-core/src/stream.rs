//! Streamers.
//!
//! # Why there are two layers here
//!
//! [`Device`](crate::device::Device) must be object-safe, so it cannot have a
//! generic `rx_stream::<T>()` method. Two ways out: give `Device` one
//! constructor per sample type, or erase the sample type at the trait boundary.
//! This module does the latter.
//!
//! A backend implements [`RawRxStreamer`] / [`RawTxStreamer`], which speak in
//! `&mut [u8]` — one buffer per channel. Callers hold [`RxStream<T>`] /
//! [`TxStream<T>`], which speak in `&mut [T]` for a [`Sample`] type `T` and do
//! the (free, [`bytemuck`]-checked) reinterpretation on the way through.
//!
//! The payoff: a backend writes *one* streamer and supports every host format,
//! and adding `sc4` later touches only [`sample`](crate::sample) — no trait in
//! this module changes, so no backend breaks. It also matches how the hardware
//! actually works, since the format conversion is a stage in the streamer's
//! pipeline either way.

use std::fmt;
use std::marker::PhantomData;
use std::time::{Duration, Instant};

use crate::args::DeviceArgs;
use crate::error::{Error, Result};
use crate::metadata::{AsyncMetadata, RxMetadata, TxMetadata};
use crate::sample::{Sample, SampleFormat};
use crate::time::TimeSpec;

/// What a stream command tells the device to do.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
#[non_exhaustive]
pub enum StreamMode {
    /// Stream until told to stop.
    StartContinuous,
    /// Stop streaming.
    StopContinuous,
    /// Send exactly `n` samples, then end the burst.
    NumSampsAndDone(u64),
    /// Send exactly `n` samples and expect another command to continue the
    /// burst.
    NumSampsAndMore(u64),
}

/// When a [`StreamCmd`] takes effect.
#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub enum StreamTime {
    /// Act on receipt.
    #[default]
    Now,
    /// Act at this device time. The command is dropped (and an
    /// [`RxError::LateCommand`](crate::metadata::RxError::LateCommand) reported)
    /// if the time has already passed.
    At(TimeSpec),
}

/// A command issued to a receive streamer.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct StreamCmd {
    /// What to do.
    pub mode: StreamMode,
    /// When to do it.
    pub time: StreamTime,
}

impl StreamCmd {
    /// Start streaming immediately and keep going.
    #[must_use]
    pub fn start_continuous() -> Self {
        Self {
            mode: StreamMode::StartContinuous,
            time: StreamTime::Now,
        }
    }

    /// Stop streaming immediately.
    #[must_use]
    pub fn stop() -> Self {
        Self {
            mode: StreamMode::StopContinuous,
            time: StreamTime::Now,
        }
    }

    /// Capture exactly `n` samples, then stop.
    #[must_use]
    pub fn num_samps(n: u64) -> Self {
        Self {
            mode: StreamMode::NumSampsAndDone(n),
            time: StreamTime::Now,
        }
    }

    /// Defers this command to `time` on the device clock.
    #[must_use]
    pub fn at(mut self, time: TimeSpec) -> Self {
        self.time = StreamTime::At(time);
        self
    }
}

/// How to open a streamer.
#[derive(Debug, Clone, PartialEq)]
pub struct StreamArgs {
    /// Format the caller's buffers hold.
    ///
    /// [`DeviceExt::rx_stream`](crate::device::DeviceExt::rx_stream) overwrites
    /// this from the buffer type, so you only set it by hand when calling
    /// [`Device::open_rx_stream`](crate::device::Device::open_rx_stream)
    /// directly.
    pub host_format: SampleFormat,
    /// Format that crosses the link.
    pub wire_format: SampleFormat,
    /// Device channels to stream, in the order they map to buffers.
    pub channels: Vec<usize>,
    /// Backend-specific knobs, e.g. `spp=1996`, `num_recv_frames=32`.
    pub args: DeviceArgs,
}

impl StreamArgs {
    /// Single-channel `fc32`-over-`sc16`, the sensible default.
    #[must_use]
    pub fn new() -> Self {
        Self::default()
    }

    /// Sets the channel list.
    #[must_use]
    pub fn with_channels(mut self, channels: impl IntoIterator<Item = usize>) -> Self {
        self.channels = channels.into_iter().collect();
        self
    }

    /// Sets the over-the-wire format.
    #[must_use]
    pub fn with_wire_format(mut self, format: SampleFormat) -> Self {
        self.wire_format = format;
        self
    }

    /// Attaches a backend-specific argument.
    #[must_use]
    pub fn with_arg(mut self, key: impl Into<String>, value: impl Into<String>) -> Self {
        self.args.set(key, value);
        self
    }

    /// Number of channels requested.
    #[must_use]
    pub fn num_channels(&self) -> usize {
        self.channels.len()
    }
}

impl Default for StreamArgs {
    fn default() -> Self {
        Self {
            host_format: SampleFormat::Fc32,
            wire_format: SampleFormat::Sc16,
            channels: vec![0],
            args: DeviceArgs::new(),
        }
    }
}

/// A receive streamer with its sample type erased.
///
/// Backends implement this; callers normally use [`RxStream<T>`]. All buffer
/// slices are one-per-channel, in the order given by
/// [`StreamArgs::channels`], and must all be the same length.
pub trait RawRxStreamer: Send + fmt::Debug {
    /// How many channels this streamer carries.
    fn num_channels(&self) -> usize;

    /// The largest number of samples a single [`RawRxStreamer::recv_raw`] can
    /// return per channel.
    fn max_num_samps(&self) -> usize;

    /// The host format this streamer produces.
    fn host_format(&self) -> SampleFormat;

    /// The format samples arrive in over the link.
    fn wire_format(&self) -> SampleFormat;

    /// Starts, stops or schedules the flow of samples.
    fn issue_stream_cmd(&mut self, cmd: &StreamCmd) -> Result<()>;

    /// Fills `buffs` with up to `buffs[i].len() / bytes_per_sample` samples per
    /// channel.
    ///
    /// Returns the number of *samples* (not bytes) written to each channel.
    /// A return of `Ok(0)` with `md.error == Some(RxError::Timeout)` means
    /// nothing arrived in time, which is not by itself fatal.
    fn recv_raw(
        &mut self,
        buffs: &mut [&mut [u8]],
        timeout: Duration,
        md: &mut RxMetadata,
    ) -> Result<usize>;

    /// Receives samples while observing progress inside a blocking receive.
    ///
    /// `progress` is called after copying each nonempty chunk, with its sample
    /// count per channel and a monotonic host timestamp taken when copying
    /// began. Chunk counts sum to the returned sample count. Implementations
    /// must return already observed samples before reporting a later error.
    /// The callback should do minimal work; it runs on the receive thread.
    ///
    /// The default implementation observes the entire result after
    /// [`Self::recv_raw`] returns. Backends may override this for finer timing;
    /// the B2xx backend observes each packet fragment. These timestamps do not
    /// measure device time or time spent waiting in transport buffers.
    fn recv_raw_with_progress(
        &mut self,
        buffs: &mut [&mut [u8]],
        timeout: Duration,
        md: &mut RxMetadata,
        progress: &mut dyn FnMut(usize, Instant),
    ) -> Result<usize> {
        let n = self.recv_raw(buffs, timeout, md)?;
        if n > 0 {
            progress(n, Instant::now());
        }
        Ok(n)
    }
}

/// A transmit streamer with its sample type erased.
pub trait RawTxStreamer: Send + fmt::Debug {
    /// How many channels this streamer carries.
    fn num_channels(&self) -> usize;

    /// The largest number of samples a single [`RawTxStreamer::send_raw`] can
    /// accept per channel.
    fn max_num_samps(&self) -> usize;

    /// The host format this streamer consumes.
    fn host_format(&self) -> SampleFormat;

    /// The format samples leave in over the link.
    fn wire_format(&self) -> SampleFormat;

    /// Sends up to one buffer's worth of samples per channel.
    ///
    /// Returns the number of *samples* (not bytes) consumed from each channel.
    fn send_raw(&mut self, buffs: &[&[u8]], timeout: Duration, md: &TxMetadata) -> Result<usize>;

    /// Polls for a transmit-side event.
    ///
    /// `Ok(None)` means nothing was pending before `timeout` elapsed.
    fn recv_async_msg(&mut self, timeout: Duration) -> Result<Option<AsyncMetadata>>;
}

/// A receive streamer producing samples of type `T`.
///
/// Obtained from
/// [`DeviceExt::rx_stream`](crate::device::DeviceExt::rx_stream).
#[derive(Debug)]
pub struct RxStream<T: Sample> {
    inner: Box<dyn RawRxStreamer>,
    _sample: PhantomData<fn() -> T>,
}

impl<T: Sample> RxStream<T> {
    /// Wraps a raw streamer, checking that it really produces `T`.
    pub fn new(inner: Box<dyn RawRxStreamer>) -> Result<Self> {
        let actual = inner.host_format();
        if actual != T::FORMAT {
            return Err(Error::FormatMismatch {
                streamer: actual,
                buffer: T::FORMAT,
            });
        }
        Ok(Self {
            inner,
            _sample: PhantomData,
        })
    }

    /// How many channels this streamer carries.
    #[must_use]
    pub fn num_channels(&self) -> usize {
        self.inner.num_channels()
    }

    /// The largest number of samples one [`RxStream::recv`] can return per
    /// channel. Sizing buffers to this avoids needless round trips.
    #[must_use]
    pub fn max_num_samps(&self) -> usize {
        self.inner.max_num_samps()
    }

    /// The format samples arrive in over the link.
    #[must_use]
    pub fn wire_format(&self) -> SampleFormat {
        self.inner.wire_format()
    }

    /// Starts, stops or schedules the flow of samples.
    pub fn issue_stream_cmd(&mut self, cmd: &StreamCmd) -> Result<()> {
        self.inner.issue_stream_cmd(cmd)
    }

    /// Receives into one buffer per channel, returning samples written per
    /// channel.
    ///
    /// `buffs` must have exactly [`RxStream::num_channels`] entries, all the
    /// same length.
    pub fn recv(
        &mut self,
        buffs: &mut [&mut [T]],
        timeout: Duration,
        md: &mut RxMetadata,
    ) -> Result<usize> {
        check_shape(buffs.len(), self.inner.num_channels(), |b| buffs[b].len())?;
        // `cast_slice_mut` compiles to nothing; the only real work is rebuilding
        // the array of fat pointers, which for the 1-4 channels real hardware has
        // is far cheaper than the link transaction that follows.
        let mut byte_buffs: Vec<&mut [u8]> = buffs
            .iter_mut()
            .map(|b| bytemuck::cast_slice_mut(b))
            .collect();
        self.inner.recv_raw(&mut byte_buffs, timeout, md)
    }

    /// Convenience wrapper over [`RxStream::recv`] for single-channel streams.
    ///
    /// Returns [`Error::Buffer`] if the streamer has more than one channel.
    pub fn recv_one(
        &mut self,
        buff: &mut [T],
        timeout: Duration,
        md: &mut RxMetadata,
    ) -> Result<usize> {
        self.recv(&mut [buff], timeout, md)
    }

    /// Borrows the underlying type-erased streamer.
    #[must_use]
    pub fn as_raw(&self) -> &dyn RawRxStreamer {
        self.inner.as_ref()
    }
}

/// A transmit streamer consuming samples of type `T`.
///
/// Obtained from
/// [`DeviceExt::tx_stream`](crate::device::DeviceExt::tx_stream).
#[derive(Debug)]
pub struct TxStream<T: Sample> {
    inner: Box<dyn RawTxStreamer>,
    _sample: PhantomData<fn(T)>,
}

impl<T: Sample> TxStream<T> {
    /// Wraps a raw streamer, checking that it really consumes `T`.
    pub fn new(inner: Box<dyn RawTxStreamer>) -> Result<Self> {
        let actual = inner.host_format();
        if actual != T::FORMAT {
            return Err(Error::FormatMismatch {
                streamer: actual,
                buffer: T::FORMAT,
            });
        }
        Ok(Self {
            inner,
            _sample: PhantomData,
        })
    }

    /// How many channels this streamer carries.
    #[must_use]
    pub fn num_channels(&self) -> usize {
        self.inner.num_channels()
    }

    /// The largest number of samples one [`TxStream::send`] can accept per
    /// channel.
    #[must_use]
    pub fn max_num_samps(&self) -> usize {
        self.inner.max_num_samps()
    }

    /// The format samples leave in over the link.
    #[must_use]
    pub fn wire_format(&self) -> SampleFormat {
        self.inner.wire_format()
    }

    /// Sends one buffer per channel, returning samples consumed per channel.
    ///
    /// `buffs` must have exactly [`TxStream::num_channels`] entries, all the
    /// same length.
    pub fn send(&mut self, buffs: &[&[T]], timeout: Duration, md: &TxMetadata) -> Result<usize> {
        check_shape(buffs.len(), self.inner.num_channels(), |b| buffs[b].len())?;
        let byte_buffs: Vec<&[u8]> = buffs.iter().map(|b| bytemuck::cast_slice(b)).collect();
        self.inner.send_raw(&byte_buffs, timeout, md)
    }

    /// Convenience wrapper over [`TxStream::send`] for single-channel streams.
    pub fn send_one(&mut self, buff: &[T], timeout: Duration, md: &TxMetadata) -> Result<usize> {
        self.send(&[buff], timeout, md)
    }

    /// Polls for a transmit-side event such as an underflow.
    pub fn recv_async_msg(&mut self, timeout: Duration) -> Result<Option<AsyncMetadata>> {
        self.inner.recv_async_msg(timeout)
    }

    /// Borrows the underlying type-erased streamer.
    #[must_use]
    pub fn as_raw(&self) -> &dyn RawTxStreamer {
        self.inner.as_ref()
    }
}

/// Checks that the caller passed one equal-length buffer per channel.
fn check_shape(given: usize, expected: usize, len_of: impl Fn(usize) -> usize) -> Result<()> {
    if given != expected {
        return Err(Error::Buffer(format!(
            "streamer has {expected} channels but {given} buffers were supplied"
        )));
    }
    if given > 1 {
        let first = len_of(0);
        for i in 1..given {
            if len_of(i) != first {
                return Err(Error::Buffer(format!(
                    "channel buffers must be equal length: buffer 0 holds {first} samples, \
                     buffer {i} holds {}",
                    len_of(i)
                )));
            }
        }
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use num_complex::Complex;

    use super::*;

    /// A streamer that hands back a fixed ramp, used to prove the typed wrapper
    /// reinterprets buffers correctly.
    #[derive(Debug)]
    struct FakeRx {
        channels: usize,
        host_format: SampleFormat,
    }

    impl RawRxStreamer for FakeRx {
        fn num_channels(&self) -> usize {
            self.channels
        }
        fn max_num_samps(&self) -> usize {
            8
        }
        fn host_format(&self) -> SampleFormat {
            self.host_format
        }
        fn wire_format(&self) -> SampleFormat {
            SampleFormat::Sc16
        }
        fn issue_stream_cmd(&mut self, _cmd: &StreamCmd) -> Result<()> {
            Ok(())
        }
        fn recv_raw(
            &mut self,
            buffs: &mut [&mut [u8]],
            _timeout: Duration,
            _md: &mut RxMetadata,
        ) -> Result<usize> {
            for (ch, buf) in buffs.iter_mut().enumerate() {
                for (i, byte) in buf.iter_mut().enumerate() {
                    *byte = u8::try_from((i + ch) % 256).unwrap_or(0);
                }
            }
            Ok(buffs[0].len() / 8)
        }
    }

    fn fake(channels: usize, host_format: SampleFormat) -> Box<dyn RawRxStreamer> {
        Box::new(FakeRx {
            channels,
            host_format,
        })
    }

    #[test]
    fn default_progress_preserves_data_and_reports_only_nonempty_results() {
        let mut rx = fake(1, SampleFormat::Fc32);
        let mut bytes = [0; 16];
        let mut md = RxMetadata::default();
        let mut observations = Vec::new();
        let before = Instant::now();
        let n = rx
            .recv_raw_with_progress(
                &mut [&mut bytes],
                Duration::ZERO,
                &mut md,
                &mut |count, when| observations.push((count, when)),
            )
            .unwrap();
        let after = Instant::now();
        assert_eq!(n, 2);
        assert_eq!(bytes, std::array::from_fn(|i| i as u8));
        assert_eq!(observations.len(), 1);
        assert_eq!(observations[0].0, n);
        assert!((before..=after).contains(&observations[0].1));
        assert_eq!(
            rx.recv_raw_with_progress(&mut [&mut []], Duration::ZERO, &mut md, &mut |_, _| panic!(
                "empty receives must not report progress"
            ),)
                .unwrap(),
            0
        );
    }

    #[test]
    fn typed_wrapper_rejects_a_format_mismatch() {
        let err = RxStream::<Complex<i16>>::new(fake(1, SampleFormat::Fc32)).unwrap_err();
        assert!(matches!(
            err,
            Error::FormatMismatch {
                streamer: SampleFormat::Fc32,
                buffer: SampleFormat::Sc16
            }
        ));
    }

    #[test]
    fn recv_writes_through_to_the_typed_buffer() {
        let mut stream = RxStream::<Complex<f32>>::new(fake(1, SampleFormat::Fc32)).unwrap();
        let mut buf = vec![Complex::new(0.0f32, 0.0); 4];
        let mut md = RxMetadata::default();
        let n = stream
            .recv_one(&mut buf, Duration::from_millis(10), &mut md)
            .unwrap();
        assert_eq!(n, 4);
        // The fake wrote a byte ramp; anything non-zero proves the bytes landed
        // in the caller's buffer rather than a copy.
        assert_ne!(buf[1], Complex::new(0.0, 0.0));
    }

    #[test]
    fn wrong_buffer_count_is_rejected() {
        let mut stream = RxStream::<Complex<f32>>::new(fake(2, SampleFormat::Fc32)).unwrap();
        let mut buf = vec![Complex::new(0.0f32, 0.0); 4];
        let mut md = RxMetadata::default();
        let err = stream
            .recv_one(&mut buf, Duration::ZERO, &mut md)
            .unwrap_err();
        assert!(matches!(err, Error::Buffer(_)), "got {err:?}");
    }

    #[test]
    fn ragged_buffers_are_rejected() {
        let mut stream = RxStream::<Complex<f32>>::new(fake(2, SampleFormat::Fc32)).unwrap();
        let mut a = vec![Complex::new(0.0f32, 0.0); 4];
        let mut b = vec![Complex::new(0.0f32, 0.0); 3];
        let mut md = RxMetadata::default();
        let err = stream
            .recv(&mut [&mut a, &mut b], Duration::ZERO, &mut md)
            .unwrap_err();
        assert!(matches!(err, Error::Buffer(_)), "got {err:?}");
    }

    #[test]
    fn stream_cmd_builders() {
        let cmd = StreamCmd::num_samps(1024).at(TimeSpec::from_secs(1.0));
        assert_eq!(cmd.mode, StreamMode::NumSampsAndDone(1024));
        assert_eq!(cmd.time, StreamTime::At(TimeSpec::from_secs(1.0)));
    }
}
