//! Core traits and value types for `ruhd`, a pure-Rust reimplementation of the
//! Ettus USRP Hardware Driver.
//!
//! This crate knows what a USRP *is* and nothing about how one is attached. It
//! has no USB dependency, no socket code and no product-specific logic; those
//! live in `uhd-transport` and the backend crates. Everything here is host-side
//! types and traits.
//!
//! # Map of the crate
//!
//! - [`device`] — the object-safe [`Device`] trait plus [`DeviceExt`] generics.
//! - [`stream`] — [`RxStream`]/[`TxStream`] for callers, and the type-erased
//!   [`RawRxStreamer`]/[`RawTxStreamer`] backends implement.
//! - [`registry`] — how a backend advertises itself and how devices are found.
//! - [`sample`], [`tune`], [`range`], [`time`], [`metadata`], [`args`] — the
//!   value types those traits trade in.
//! - [`error`] — one [`Error`] for the whole workspace.
//!
//! # Example
//!
//! ```no_run
//! use std::time::Duration;
//! use num_complex::Complex;
//! use uhd_core::prelude::*;
//!
//! # fn main() -> uhd_core::Result<()> {
//! let device = uhd_core::registry::open(&"type=b200".parse()?)?;
//!
//! device.set_sample_rate(Direction::Rx, 0, 10e6)?;
//! let tuned = device.tune_to(Direction::Rx, 0, 2.4e9)?;
//! println!("tuned to {tuned}");
//!
//! let mut rx = device.rx_stream::<Complex<f32>>(&StreamArgs::new())?;
//! let mut buffer = vec![Complex::new(0.0f32, 0.0); rx.max_num_samps()];
//! let mut md = RxMetadata::default();
//!
//! rx.issue_stream_cmd(&StreamCmd::num_samps(1_000_000))?;
//! loop {
//!     let n = rx.recv_one(&mut buffer, Duration::from_millis(100), &mut md)?;
//!     md.check()?;
//!     if n == 0 && md.end_of_burst {
//!         break;
//!     }
//! }
//! # Ok(())
//! # }
//! ```
//!
//! [`Device`]: device::Device
//! [`DeviceExt`]: device::DeviceExt
//! [`RxStream`]: stream::RxStream
//! [`TxStream`]: stream::TxStream
//! [`RawRxStreamer`]: stream::RawRxStreamer
//! [`RawTxStreamer`]: stream::RawTxStreamer
//! [`Error`]: error::Error

pub mod args;
pub mod device;
pub mod error;
pub mod metadata;
pub mod range;
pub mod registry;
pub mod sample;
pub mod stream;
pub mod time;
pub mod tune;

pub use args::DeviceArgs;
pub use device::{Device, DeviceExt, DeviceInfo, Direction, SensorData, SensorScope, SensorValue};
pub use error::{Error, Result};
pub use metadata::{AsyncEvent, AsyncMetadata, RxError, RxMetadata, TxMetadata};
pub use range::{MetaRange, Range};
pub use registry::{DeviceFactory, Registry};
pub use sample::{Sample, SampleFormat};
pub use stream::{
    RawRxStreamer, RawTxStreamer, RxStream, StreamArgs, StreamCmd, StreamMode, StreamTime, TxStream,
};
pub use time::TimeSpec;
pub use tune::{TunePolicy, TuneRequest, TuneResult};

/// Re-exported `num_complex`, so downstream crates get the same `Complex` this
/// crate's [`Sample`] impls are written against.
pub use num_complex;

/// The imports a typical application wants.
pub mod prelude {
    pub use crate::device::{Device, DeviceExt, Direction};
    pub use crate::error::{Error, Result};
    pub use crate::metadata::{RxMetadata, TxMetadata};
    pub use crate::stream::{StreamArgs, StreamCmd};
    pub use crate::time::TimeSpec;
    pub use crate::tune::{TuneRequest, TuneResult};
}
