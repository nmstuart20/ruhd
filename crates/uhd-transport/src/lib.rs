//! Transport abstractions shared by `ruhd` device backends.
//!
//! Mostly traits: the shape of the boundary is settled here before a backend is
//! written against it. The parts with an implementation behind them are
//! [`usb`], which finds B2xx devices on the bus and gets firmware running on
//! them, and [`images`], which finds the firmware.
//!
//! The boundary is drawn where it is so that a network-attached device can be
//! added later without touching anything above:
//!
//! - [`endpoint`] names a device before it is opened, and already has both a USB
//!   and a UDP variant.
//! - [`usb`] fills those names in: it enumerates the USB bus, classifies what it
//!   finds as a B2xx or an unprogrammed FX3, and loads FX3 firmware into one
//!   that needs it.
//! - [`images`] locates the UHD images package, which is where that firmware
//!   lives — a B2xx has no flash, so this is on the critical path to opening
//!   one at all.
//! - [`link`] moves frames. USB bulk transfers and UDP datagrams are both
//!   fixed-size frames from a pool, so one pair of traits covers both.
//! - [`register`] is peek/poke against the settings bus, whether that is an FX3
//!   control transfer or a ZPU packet.
//! - [`serial`] is SPI/I2C/GPIO in their general forms, kept here rather than in
//!   a backend so a caller can reach a bus without naming the board behind it. A
//!   driver for a specific part defines its own narrower trait — see the module
//!   docs.

pub mod endpoint;
pub mod images;
pub mod link;
pub mod register;
pub mod serial;
pub mod usb;

pub use endpoint::{Endpoint, UsbEndpoint};
pub use link::{FrameId, LinkInfo, RecvLink, SendLink};
pub use register::RegisterIface;
pub use serial::{ClockEdge, GpioDirection, GpioIface, I2cIface, SpiConfig, SpiIface};
pub use usb::{
    B2xxProduct, DeviceState, FirmwareVersion, Fx3Iface, HexImage, InitOutcome, UsbFilter, UsbProbe,
};
