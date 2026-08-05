//! Backend for the USRP B200/B210/B205mini family. Initial target: B205mini.
//!
//! Reference: `reference/uhd/host/lib/usrp/b200/{b200_iface,b200_impl,b200_io_impl}.cpp`
//! — see `NOTES.md`.
//!
//! # Compatibility is negotiated, once, at open time
//!
//! A B2xx's FPGA image carries a compatibility major that identifies the
//! register interface the host is about to speak. This crate resolves it before
//! it configures anything, and everything version-dependent below that point
//! goes through the resulting shim:
//!
//! - [`compat`] — the [`FpgaCompat`] enum, one variant per supported major, and
//!   the [`FpgaShim`] behind each. This is where "what changes between FPGA
//!   builds" lives, instead of in `is_b2xx_mini` tests scattered down the file.
//! - [`policy`] — what to do about a device outside the window: proceed, offer
//!   a reload from the image cache, or refuse with a diagnostic naming the
//!   device's compat, the supported window and the manifest entry that would
//!   fix it. Pure, and unit-tested as a matrix.
//! - [`session`] — the open path that puts the two together, and [`checkup`],
//!   the read-only form behind `ruhd doctor`.
//! - [`radio_ctrl`] — the settings bus, opened against a negotiated major.
//!
//! # The cores on that bus
//!
//! Everything below the settings bus is a peripheral at a settings-bus address,
//! and which core it lives on decides when it can be reached:
//!
//! | Module | Core | Reachable once |
//! | --- | --- | --- |
//! | [`spi`] | local | the bitstream is in |
//! | [`misc`] | local | the bitstream is in |
//! | [`atr`] | radio | the AD936x is clocking |
//! | [`time_core`] | radio | the AD936x is clocking |
//!
//! That table is a bootstrap order, not a coincidence. The radio cores run off
//! the codec's sample clock, the codec is started over SPI, and the SPI master
//! is therefore on the local core — so [`spi`] and [`misc`] are the two things
//! that have to work before anything else can. Reading the AD936x's product ID
//! over that path is the first hardware gate this driver has that proves
//! anything about the radio; see [`B200Session::codec_gate`].
//!
//! ```no_run
//! # fn main() -> uhd_core::Result<()> {
//! let session = uhd_b200::open(&uhd_b200::OpenOptions::new().with_auto_flash(true))?;
//! println!(
//!     "{} on fpga compat {} ({})",
//!     session.product(),
//!     session.compat().major(),
//!     session.shim().codename(),
//! );
//! # Ok(())
//! # }
//! ```
//!
//! # The commitment this creates
//!
//! Every variant of [`FpgaCompat`] is a promise: its shim stays alive and stays
//! CI-tested, and removing one is a breaking change for this crate. `NOTES.md`
//! records the terms.

pub mod atr;
pub mod codec;
pub mod compat;
pub mod factory;
pub mod misc;
pub mod policy;
pub mod radio_ctrl;
pub mod session;
pub mod spi;
pub mod time_core;

pub use atr::{AtrMode, AtrOffsets, AtrReg, AtrStates, GpioAtr, Side};
pub use codec::client_params;
pub use compat::{CoreRegs, FpgaCompat, FpgaShim, FullShim, MiniShim, RadioRegs, RefLock};
pub use factory::{B200Factory, find, register};
pub use misc::{MiscGpio, RefSel, RxBand, TxBand};
pub use policy::{
    DeviceCompat, Diagnosis, FoundFpga, ImageCandidate, ImageCatalog, ImageKind, Reload, Trouble,
    UhdImagesCatalog, Verdict, negotiate,
};
pub use radio_ctrl::{ControlGate, CtrlPort, RadioCtrl};
pub use session::{
    B200Session, Checkup, CodecGate, Flashed, OpenOptions, checkup, open, open_probed,
};
pub use spi::{CodecSpi, LocalSpi, Perif, SpiCore};
pub use time_core::{Latch, TimeCore, TimeSource, TimerCheck};
pub use uhd_transport::usb::FpgaVersion;
