//! The device trait: everything a USRP can be asked to do.
//!
//! # Design constraints
//!
//! [`Device`] is **object-safe**. Applications hold `Arc<dyn Device>` and never
//! name a concrete backend, so a B205mini and a future network-attached X310 are
//! interchangeable at the call site. Three rules follow from that, and they are
//! the reason the trait looks the way it does:
//!
//! 1. **No generic methods.** Sample types are erased at the streamer boundary
//!    instead — see [`stream`](crate::stream). Generic conveniences live on
//!    [`DeviceExt`], which is blanket-implemented and therefore free.
//! 2. **Channel counts are runtime queries, never constants or type
//!    parameters.** A B205mini has one channel per direction, a B210 two, an
//!    N310 four; `num_channels(dir)` covers all of them, and code written
//!    against it does not care which is plugged in.
//! 3. **`&self`, not `&mut self`.** A receive thread and a transmit thread need
//!    to reconfigure the same device concurrently, so backends use interior
//!    mutability and the trait requires `Send + Sync`.
//!
//! # Adding methods later without breaking backends
//!
//! Every method that some plausible device cannot do has a default
//! implementation returning [`Error::NotSupported`]. New methods added in the
//! same style are not breaking changes. The genuinely universal operations —
//! identity, channel counts, rate, tuning, gain, antenna, opening a streamer —
//! have no default, so a backend cannot forget them by accident.
//!
//! # Directions, motherboards and channels
//!
//! Rather than doubling the trait into `set_rx_gain`/`set_tx_gain` pairs,
//! direction is a [`Direction`] argument. Channel indices are flat across the
//! whole device: on a multi-motherboard setup, channels are numbered in
//! motherboard order, and `num_mboards` exists so that clock and time
//! operations — which are per-motherboard, not per-channel — can address each
//! one.

use std::fmt;

use crate::args::DeviceArgs;
use crate::error::{Error, Result};
use crate::range::{MetaRange, Range};
use crate::sample::Sample;
use crate::stream::{RawRxStreamer, RawTxStreamer, RxStream, StreamArgs, TxStream};
use crate::time::TimeSpec;
use crate::tune::{TuneRequest, TuneResult};

/// Which signal path an operation refers to.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum Direction {
    /// Receive: antenna to host.
    Rx,
    /// Transmit: host to antenna.
    Tx,
}

impl Direction {
    /// Both directions, for iterating.
    pub const ALL: [Self; 2] = [Self::Rx, Self::Tx];

    /// The short name UHD uses in property paths and log messages.
    #[must_use]
    pub fn as_str(self) -> &'static str {
        match self {
            Self::Rx => "rx",
            Self::Tx => "tx",
        }
    }
}

impl fmt::Display for Direction {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(self.as_str())
    }
}

/// What a device is, and how to open it again.
#[derive(Debug, Clone, Default, PartialEq, Eq)]
pub struct DeviceInfo {
    /// Driver family, e.g. `b200`. Matches the `type=` device arg.
    pub device_type: String,
    /// Specific product, e.g. `B205mini`.
    pub product: String,
    /// Serial number as printed by the device.
    pub serial: String,
    /// Human-readable name, if the device carries one in its EEPROM.
    pub name: String,
    /// Hardware revision, if reported.
    pub revision: Option<String>,
    /// Arguments that will re-open exactly this device.
    ///
    /// Discovery fills this in, so `open(info.args)` is always unambiguous even
    /// when the hint the user typed was not.
    pub args: DeviceArgs,
}

impl fmt::Display for DeviceInfo {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{} ({})", self.product, self.serial)
    }
}

/// What a sensor reading is attached to.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum SensorScope {
    /// A motherboard-wide sensor, e.g. `ref_locked`.
    Mboard(usize),
    /// A per-channel front-end sensor, e.g. `lo_locked` or `rssi`.
    Channel {
        /// Which direction the channel belongs to.
        dir: Direction,
        /// Channel index.
        chan: usize,
    },
}

/// A sensor reading.
#[derive(Debug, Clone, PartialEq)]
pub struct SensorValue {
    /// Sensor name, e.g. `lo_locked`.
    pub name: String,
    /// The reading.
    pub value: SensorData,
    /// Unit string, e.g. `dBm`. Empty for booleans.
    pub unit: String,
}

/// The payload of a [`SensorValue`].
#[derive(Debug, Clone, PartialEq)]
#[non_exhaustive]
pub enum SensorData {
    /// A lock or presence indication.
    Bool(bool),
    /// An integer reading.
    Int(i64),
    /// A real-valued reading.
    Real(f64),
    /// Anything else.
    String(String),
}

impl fmt::Display for SensorValue {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match &self.value {
            SensorData::Bool(b) => write!(f, "{}", if *b { "true" } else { "false" }),
            SensorData::Int(i) => write!(f, "{i} {}", self.unit),
            SensorData::Real(r) => write!(f, "{r} {}", self.unit),
            SensorData::String(s) => write!(f, "{s}"),
        }
    }
}

/// An open USRP.
///
/// See the [module docs](self) for why the trait is shaped this way.
pub trait Device: Send + Sync + fmt::Debug {
    // ---------------------------------------------------------------- identity

    /// What this device is.
    fn info(&self) -> DeviceInfo;

    /// How many motherboards this device spans. Almost always 1.
    fn num_mboards(&self) -> usize {
        1
    }

    /// How many channels exist in `dir`.
    ///
    /// Callers must treat this as a runtime property — it is 1 on a B205mini and
    /// 2 on a B210, from the same driver.
    fn num_channels(&self, dir: Direction) -> usize;

    // ------------------------------------------------------------ sample rates

    /// Sets the sample rate on `chan`, returning the rate actually programmed.
    fn set_sample_rate(&self, dir: Direction, chan: usize, rate: f64) -> Result<f64>;

    /// The sample rate currently programmed on `chan`.
    fn sample_rate(&self, dir: Direction, chan: usize) -> Result<f64>;

    /// The sample rates `chan` can be set to.
    fn sample_rates(&self, dir: Direction, chan: usize) -> Result<MetaRange>;

    // ------------------------------------------------------------------ tuning

    /// Tunes `chan` and reports where it landed.
    fn tune(&self, dir: Direction, chan: usize, request: &TuneRequest) -> Result<TuneResult>;

    /// The centre frequency `chan` is currently at.
    fn center_freq(&self, dir: Direction, chan: usize) -> Result<f64>;

    /// The overall tunable range of `chan`, RF and DSP stages combined.
    fn freq_range(&self, dir: Direction, chan: usize) -> Result<MetaRange>;

    /// The range the RF front end alone can reach on `chan`.
    fn fe_freq_range(&self, dir: Direction, chan: usize) -> Result<MetaRange> {
        self.freq_range(dir, chan)
    }

    // -------------------------------------------------------------------- gain

    /// Names of the individual gain stages on `chan`.
    ///
    /// An empty list means the channel has a single unnamed overall gain.
    fn gain_names(&self, _dir: Direction, _chan: usize) -> Result<Vec<String>> {
        Ok(Vec::new())
    }

    /// Sets a gain stage, or the overall gain when `name` is `None`.
    fn set_gain(&self, dir: Direction, chan: usize, name: Option<&str>, gain: f64) -> Result<()>;

    /// Reads a gain stage, or the overall gain when `name` is `None`.
    fn gain(&self, dir: Direction, chan: usize, name: Option<&str>) -> Result<f64>;

    /// The settable range of a gain stage.
    fn gain_range(&self, dir: Direction, chan: usize, name: Option<&str>) -> Result<Range>;

    /// Enables or disables automatic gain control on `chan`.
    fn set_agc(&self, _dir: Direction, _chan: usize, _enable: bool) -> Result<()> {
        Err(Error::not_supported("automatic gain control"))
    }

    // --------------------------------------------------------------- bandwidth

    /// Sets the analog filter bandwidth on `chan`, returning what was
    /// programmed.
    fn set_bandwidth(&self, _dir: Direction, _chan: usize, _bandwidth: f64) -> Result<f64> {
        Err(Error::not_supported("analog bandwidth control"))
    }

    /// The analog filter bandwidth currently set on `chan`.
    fn bandwidth(&self, _dir: Direction, _chan: usize) -> Result<f64> {
        Err(Error::not_supported("analog bandwidth control"))
    }

    /// The analog filter bandwidths `chan` supports.
    fn bandwidth_range(&self, _dir: Direction, _chan: usize) -> Result<MetaRange> {
        Err(Error::not_supported("analog bandwidth control"))
    }

    // ----------------------------------------------------------------- antenna

    /// Antenna ports selectable on `chan`.
    fn antennas(&self, dir: Direction, chan: usize) -> Result<Vec<String>>;

    /// Selects an antenna port on `chan`.
    fn set_antenna(&self, dir: Direction, chan: usize, antenna: &str) -> Result<()>;

    /// The antenna port currently selected on `chan`.
    fn antenna(&self, dir: Direction, chan: usize) -> Result<String>;

    // ----------------------------------------------------------- clock and time

    /// Reference clock sources `mboard` accepts, e.g. `internal`, `external`,
    /// `gpsdo`.
    fn clock_sources(&self, _mboard: usize) -> Result<Vec<String>> {
        Ok(vec!["internal".to_owned()])
    }

    /// Selects the reference clock source on `mboard`.
    fn set_clock_source(&self, _mboard: usize, _source: &str) -> Result<()> {
        Err(Error::not_supported("clock source selection"))
    }

    /// The reference clock source currently selected on `mboard`.
    fn clock_source(&self, _mboard: usize) -> Result<String> {
        Ok("internal".to_owned())
    }

    /// Time sources `mboard` accepts, e.g. `internal`, `external`, `gpsdo`.
    fn time_sources(&self, _mboard: usize) -> Result<Vec<String>> {
        Ok(vec!["internal".to_owned()])
    }

    /// Selects the time source on `mboard`.
    fn set_time_source(&self, _mboard: usize, _source: &str) -> Result<()> {
        Err(Error::not_supported("time source selection"))
    }

    /// The time source currently selected on `mboard`.
    fn time_source(&self, _mboard: usize) -> Result<String> {
        Ok("internal".to_owned())
    }

    /// Sets the master clock rate on `mboard`, returning what was programmed.
    fn set_master_clock_rate(&self, _mboard: usize, _rate: f64) -> Result<f64> {
        Err(Error::not_supported("master clock rate control"))
    }

    /// The master clock rate on `mboard`.
    fn master_clock_rate(&self, _mboard: usize) -> Result<f64> {
        Err(Error::not_supported("master clock rate query"))
    }

    /// The device's current time on `mboard`.
    fn time_now(&self, _mboard: usize) -> Result<TimeSpec> {
        Err(Error::not_supported("device time"))
    }

    /// Sets the device's time on `mboard`, effective immediately.
    fn set_time_now(&self, _mboard: usize, _time: TimeSpec) -> Result<()> {
        Err(Error::not_supported("device time"))
    }

    /// Sets the device's time on `mboard`, effective at the next PPS edge.
    ///
    /// This is how multiple devices are aligned to a common time base.
    fn set_time_next_pps(&self, _mboard: usize, _time: TimeSpec) -> Result<()> {
        Err(Error::not_supported("pps time synchronisation"))
    }

    // ----------------------------------------------------------------- sensors

    /// Sensors readable in `scope`.
    fn sensor_names(&self, _scope: SensorScope) -> Result<Vec<String>> {
        Ok(Vec::new())
    }

    /// Reads a sensor.
    fn sensor(&self, _scope: SensorScope, name: &str) -> Result<SensorValue> {
        Err(Error::KeyNotFound(name.to_owned()))
    }

    // --------------------------------------------------------------- streaming

    /// Opens a receive streamer.
    ///
    /// Prefer [`DeviceExt::rx_stream`], which infers `args.host_format` from the
    /// sample type you intend to use and hands back a typed [`RxStream`].
    fn open_rx_stream(&self, args: &StreamArgs) -> Result<Box<dyn RawRxStreamer>>;

    /// Opens a transmit streamer.
    ///
    /// Prefer [`DeviceExt::tx_stream`].
    fn open_tx_stream(&self, args: &StreamArgs) -> Result<Box<dyn RawTxStreamer>>;
}

/// Generic conveniences over [`Device`].
///
/// Split out so that `Device` itself stays object-safe. Blanket-implemented for
/// every `Device`, including `dyn Device`, so `Arc<dyn Device>` gets these for
/// free.
pub trait DeviceExt: Device {
    /// Opens a receive streamer producing samples of type `T`.
    ///
    /// Overwrites `args.host_format` from `T`; everything else in `args` is
    /// used as given.
    fn rx_stream<T: Sample>(&self, args: &StreamArgs) -> Result<RxStream<T>> {
        let mut args = args.clone();
        args.host_format = T::FORMAT;
        RxStream::new(self.open_rx_stream(&args)?)
    }

    /// Opens a transmit streamer consuming samples of type `T`.
    fn tx_stream<T: Sample>(&self, args: &StreamArgs) -> Result<TxStream<T>> {
        let mut args = args.clone();
        args.host_format = T::FORMAT;
        TxStream::new(self.open_tx_stream(&args)?)
    }

    /// Tunes `chan`, accepting a bare frequency in place of a full request.
    fn tune_to(
        &self,
        dir: Direction,
        chan: usize,
        request: impl Into<TuneRequest>,
    ) -> Result<TuneResult> {
        self.tune(dir, chan, &request.into())
    }

    /// Checks that `chan` is a valid index for `dir`.
    ///
    /// Backends should call this at the top of channel-indexed methods so the
    /// error is uniform.
    fn check_channel(&self, dir: Direction, chan: usize) -> Result<()> {
        let count = self.num_channels(dir);
        if chan < count {
            return Ok(());
        }
        Err(Error::IndexOutOfBounds {
            what: match dir {
                Direction::Rx => "rx channel",
                Direction::Tx => "tx channel",
            },
            index: chan,
            count,
        })
    }
}

impl<D: Device + ?Sized> DeviceExt for D {}

/// Asserts at compile time that [`Device`] stays usable as a trait object.
///
/// Object safety is load-bearing here — it is what lets an application hold an
/// `Arc<dyn Device>` without knowing whether it points at a USB B205mini or a
/// UDP-attached X310. Naming the type is enough to check it, so a future method
/// that breaks dyn-compatibility fails here with a clear pointer to why, rather
/// than at some distant call site.
const _: Option<&dyn Device> = None;
