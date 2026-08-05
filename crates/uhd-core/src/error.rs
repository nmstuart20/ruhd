//! The workspace-wide error type.

use std::time::Duration;

use crate::args::DeviceArgs;
use crate::sample::SampleFormat;

/// Result alias used throughout the `ruhd` crates.
pub type Result<T, E = Error> = std::result::Result<T, E>;

/// Everything that can go wrong while finding, configuring or streaming from a
/// device.
///
/// The enum is `#[non_exhaustive]`: new variants will be added as backends land
/// (a UDP transport will want its own cases, for instance), so downstream code
/// must always carry a wildcard arm.
#[derive(Debug, thiserror::Error)]
#[non_exhaustive]
pub enum Error {
    /// A host-side I/O operation failed.
    #[error("i/o error")]
    Io(#[from] std::io::Error),

    /// An operation did not complete within its deadline.
    ///
    /// This is the *fatal* timeout: a control transaction that never got a
    /// reply, or a stream that produced nothing at all. A `recv` that simply
    /// returned no samples this round reports through
    /// [`RxMetadata`](crate::metadata::RxMetadata) instead.
    #[error("operation timed out after {0:?}")]
    Timeout(Duration),

    /// No device matched the supplied hint.
    #[error("no device found matching `{0}`")]
    NoDeviceFound(DeviceArgs),

    /// More than one device matched a hint that had to resolve to exactly one.
    #[error("`{hint}` matched {count} devices; narrow it down (e.g. with `serial=`)")]
    AmbiguousDevice {
        /// The hint that was too loose.
        hint: DeviceArgs,
        /// How many devices it matched.
        count: usize,
    },

    /// The device (or this driver) does not implement the requested operation.
    #[error("not supported: {0}")]
    NotSupported(String),

    /// An argument was malformed or nonsensical.
    #[error("invalid argument: {0}")]
    InvalidArg(String),

    /// A value fell outside the range the device accepts.
    #[error("{what}: {value} is outside {start}..={stop}")]
    OutOfRange {
        /// What was being set, e.g. `"rx gain"`.
        what: String,
        /// The offending value.
        value: f64,
        /// Lowest accepted value.
        start: f64,
        /// Highest accepted value.
        stop: f64,
    },

    /// A channel, motherboard or daughterboard index was out of bounds.
    #[error("{what} index {index} out of bounds ({count} available)")]
    IndexOutOfBounds {
        /// What was being indexed, e.g. `"rx channel"`.
        what: &'static str,
        /// The index that was asked for.
        index: usize,
        /// How many exist.
        count: usize,
    },

    /// A key was not present in a device-args string or property lookup.
    #[error("key not found: {0}")]
    KeyNotFound(String),

    /// The device cannot stream in the requested sample format.
    #[error("unsupported sample format: {0}")]
    UnsupportedFormat(SampleFormat),

    /// A typed streamer wrapper was built over a streamer of a different format.
    #[error("sample format mismatch: streamer produces {streamer}, buffer holds {buffer}")]
    FormatMismatch {
        /// The format the underlying streamer was opened with.
        streamer: SampleFormat,
        /// The format implied by the caller's buffer type.
        buffer: SampleFormat,
    },

    /// Buffers handed to `recv`/`send` did not match the streamer's shape.
    #[error("buffer error: {0}")]
    Buffer(String),

    /// The underlying link (USB, UDP, ...) failed.
    #[error("transport error: {0}")]
    Transport(String),

    /// USB-specific failure, kept separate so callers can react to unplugs.
    #[error("usb error: {0}")]
    Usb(String),

    /// Loading or talking to firmware / FPGA images failed.
    #[error("firmware error: {0}")]
    Firmware(String),

    /// The device replied, but reported a failure of its own.
    #[error("device error: {0}")]
    Device(String),

    /// An invariant this driver relies on did not hold.
    #[error("internal error: {0}")]
    Internal(String),
}

impl Error {
    /// Shorthand for [`Error::NotSupported`].
    pub fn not_supported(what: impl Into<String>) -> Self {
        Self::NotSupported(what.into())
    }

    /// Shorthand for [`Error::InvalidArg`].
    pub fn invalid_arg(what: impl Into<String>) -> Self {
        Self::InvalidArg(what.into())
    }

    /// Shorthand for [`Error::Transport`].
    pub fn transport(what: impl Into<String>) -> Self {
        Self::Transport(what.into())
    }

    /// Shorthand for [`Error::Device`].
    pub fn device(what: impl Into<String>) -> Self {
        Self::Device(what.into())
    }

    /// Shorthand for [`Error::Internal`].
    pub fn internal(what: impl Into<String>) -> Self {
        Self::Internal(what.into())
    }

    /// Returns `true` if retrying the operation could plausibly succeed.
    ///
    /// Backends use this to decide whether to re-issue a control transaction.
    #[must_use]
    pub fn is_transient(&self) -> bool {
        matches!(self, Self::Timeout(_) | Self::Transport(_) | Self::Usb(_))
    }
}
