//! Sample formats, on the host and on the wire.
//!
//! Every streamer has two formats. The *host* format is what lands in the
//! caller's buffer; the *wire* (UHD calls it "over-the-wire") format is what
//! actually crosses the link. `fc32` host over `sc16` wire is the usual choice:
//! floats are convenient, but sending them would double the bus load for no
//! extra information, since the AD9361 only produces 12 significant bits.

use bytemuck::Pod;
use num_complex::Complex;

use crate::error::{Error, Result};

/// A complex sample format.
///
/// `#[non_exhaustive]`: `sc4` and friends may appear later, so match arms need a
/// wildcard.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
#[non_exhaustive]
pub enum SampleFormat {
    /// Complex 32-bit float per component. Host only.
    Fc32,
    /// Complex 64-bit float per component. Host only.
    Fc64,
    /// Complex signed 16-bit integer per component.
    Sc16,
    /// Complex signed 12-bit integer per component. Wire only: a complex sample
    /// is a whole 3 bytes, but no Rust integer type is 12 bits wide, so there is
    /// nothing for a host buffer to hold.
    Sc12,
    /// Complex signed 8-bit integer per component.
    Sc8,
}

impl SampleFormat {
    /// The UHD spelling of this format, as used in device-args strings.
    #[must_use]
    pub fn name(self) -> &'static str {
        match self {
            Self::Fc32 => "fc32",
            Self::Fc64 => "fc64",
            Self::Sc16 => "sc16",
            Self::Sc12 => "sc12",
            Self::Sc8 => "sc8",
        }
    }

    /// Parses a UHD format name.
    pub fn from_name(name: &str) -> Result<Self> {
        match name {
            "fc32" => Ok(Self::Fc32),
            "fc64" => Ok(Self::Fc64),
            "sc16" => Ok(Self::Sc16),
            "sc12" => Ok(Self::Sc12),
            "sc8" => Ok(Self::Sc8),
            other => Err(Error::InvalidArg(format!(
                "unknown sample format `{other}`"
            ))),
        }
    }

    /// Bits occupied by one complex sample (both components).
    #[must_use]
    pub fn bits_per_sample(self) -> usize {
        match self {
            Self::Fc64 => 128,
            Self::Fc32 => 64,
            Self::Sc16 => 32,
            Self::Sc12 => 24,
            Self::Sc8 => 16,
        }
    }

    /// Bytes occupied by one complex sample, or `None` if a complex sample does
    /// not occupy a whole number of bytes.
    ///
    /// Every format defined so far is byte-aligned per complex sample —
    /// including `sc12`, whose 12-bit components pair up into 3 bytes — so this
    /// currently never returns `None`. It stays an `Option` because sub-byte
    /// formats such as `sc4` are a real possibility.
    #[must_use]
    pub fn bytes_per_sample(self) -> Option<usize> {
        let bits = self.bits_per_sample();
        (bits % 8 == 0).then_some(bits / 8)
    }

    /// Whether this format has a host-side Rust type, i.e. whether some
    /// [`Sample`] implementor reports it.
    ///
    /// `sc12` does not: it is a link-efficiency trick, decoded on the way in.
    #[must_use]
    pub fn is_host_format(self) -> bool {
        matches!(self, Self::Fc32 | Self::Fc64 | Self::Sc16 | Self::Sc8)
    }
}

impl std::fmt::Display for SampleFormat {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.write_str(self.name())
    }
}

impl std::str::FromStr for SampleFormat {
    type Err = Error;
    fn from_str(s: &str) -> Result<Self> {
        Self::from_name(s)
    }
}

mod sealed {
    pub trait Sealed {}
}

/// A host-side sample type that a streamer can fill or drain.
///
/// The trait is sealed. Its whole job is to promise that a `&[Self]` is
/// bit-for-bit the buffer layout that `Self::FORMAT` describes — that promise is
/// what lets [`RxStream`](crate::stream::RxStream) hand a byte view of the
/// caller's typed buffer straight to a backend. Only this crate can make that
/// promise truthfully, so only this crate can add implementors.
///
/// Adding a format is therefore a new `impl` here and nothing else: neither
/// [`Device`](crate::device::Device) nor the raw streamer traits mention any
/// concrete sample type.
pub trait Sample: sealed::Sealed + Pod + Copy + Send + Sync + 'static {
    /// The format this type lays out in memory.
    const FORMAT: SampleFormat;
}

macro_rules! impl_sample {
    ($ty:ty, $format:expr) => {
        impl sealed::Sealed for $ty {}
        impl Sample for $ty {
            const FORMAT: SampleFormat = $format;
        }
    };
}

impl_sample!(Complex<f32>, SampleFormat::Fc32);
impl_sample!(Complex<f64>, SampleFormat::Fc64);
impl_sample!(Complex<i16>, SampleFormat::Sc16);
impl_sample!(Complex<i8>, SampleFormat::Sc8);

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn format_names_round_trip() {
        for f in [
            SampleFormat::Fc32,
            SampleFormat::Fc64,
            SampleFormat::Sc16,
            SampleFormat::Sc12,
            SampleFormat::Sc8,
        ] {
            assert_eq!(SampleFormat::from_name(f.name()).unwrap(), f);
        }
        assert!(SampleFormat::from_name("sc4").is_err());
    }

    #[test]
    fn sc12_is_byte_aligned_but_not_a_host_format() {
        assert_eq!(SampleFormat::Sc12.bytes_per_sample(), Some(3));
        assert!(!SampleFormat::Sc12.is_host_format());
        assert_eq!(SampleFormat::Sc16.bytes_per_sample(), Some(4));
    }

    #[test]
    fn every_host_format_has_a_sample_impl() {
        // `is_host_format` is a hand-maintained list; this keeps it honest
        // against the formats that actually have a `Sample` impl below.
        let with_impls = [
            <Complex<f32> as Sample>::FORMAT,
            <Complex<f64> as Sample>::FORMAT,
            <Complex<i16> as Sample>::FORMAT,
            <Complex<i8> as Sample>::FORMAT,
        ];
        for f in [
            SampleFormat::Fc32,
            SampleFormat::Fc64,
            SampleFormat::Sc16,
            SampleFormat::Sc12,
            SampleFormat::Sc8,
        ] {
            assert_eq!(
                f.is_host_format(),
                with_impls.contains(&f),
                "{f}: is_host_format disagrees with the Sample impls"
            );
        }
    }

    #[test]
    fn sample_impls_agree_with_their_declared_size() {
        fn check<T: Sample>() {
            assert_eq!(
                size_of::<T>(),
                T::FORMAT.bytes_per_sample().unwrap(),
                "{} layout does not match its format",
                T::FORMAT
            );
            assert!(T::FORMAT.is_host_format());
        }
        check::<Complex<f32>>();
        check::<Complex<f64>>();
        check::<Complex<i16>>();
        check::<Complex<i8>>();
    }
}
