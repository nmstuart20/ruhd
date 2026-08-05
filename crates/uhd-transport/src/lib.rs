//! Transport abstractions shared by `ruhd` device backends.
//!
//! Nothing here is implemented yet — this crate is deliberately traits only, so
//! that the shape of the boundary is settled before the first USB backend is
//! written against it.
//!
//! The boundary is drawn where it is so that a network-attached device can be
//! added later without touching anything above:
//!
//! - [`endpoint`] names a device before it is opened, and already has both a USB
//!   and a UDP variant.
//! - [`link`] moves frames. USB bulk transfers and UDP datagrams are both
//!   fixed-size frames from a pool, so one pair of traits covers both.
//! - [`register`] is peek/poke against the settings bus, whether that is an FX3
//!   control transfer or a ZPU packet.
//! - [`serial`] is SPI/I2C/GPIO, kept here rather than in a backend so the
//!   AD9361 driver can depend on the bus rather than on the board.

pub mod endpoint;
pub mod link;
pub mod register;
pub mod serial;

pub use endpoint::{Endpoint, UsbEndpoint};
pub use link::{FrameId, LinkInfo, RecvLink, SendLink};
pub use register::RegisterIface;
pub use serial::{ClockEdge, GpioDirection, GpioIface, I2cIface, SpiConfig, SpiIface};
