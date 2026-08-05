//! How to reach a device, before anything has been opened.

use std::net::SocketAddr;

/// A USB device, identified specifically enough to open exactly one.
#[derive(Debug, Clone, Default, PartialEq, Eq, Hash)]
pub struct UsbEndpoint {
    /// USB vendor ID.
    pub vid: u16,
    /// USB product ID.
    pub pid: u16,
    /// Serial string, when known. This is what distinguishes two identical
    /// B205minis on the same host.
    pub serial: Option<String>,
    /// Bus number, for picking a device by physical port instead of serial.
    pub bus: Option<u8>,
    /// Address on the bus.
    pub address: Option<u8>,
}

impl std::fmt::Display for UsbEndpoint {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "usb:{:04x}:{:04x}", self.vid, self.pid)?;
        match &self.serial {
            Some(serial) => write!(f, "/{serial}"),
            // No serial: name the port instead, which is all an unprogrammed
            // device has to distinguish it from its neighbour.
            None => match (self.bus, self.address) {
                (Some(bus), Some(address)) => write!(f, "@{bus}.{address}"),
                _ => Ok(()),
            },
        }
    }
}

/// Where a device lives.
///
/// The point of this enum is that the layers above it — `chdr` packet framing,
/// the streamer plumbing, the [`Device`](uhd_core::Device) impl — take an
/// endpoint and a link and do not otherwise care which variant they got. A
/// network-attached X310 is a [`Endpoint::Udp`] with the same code above it.
#[derive(Debug, Clone, PartialEq, Eq, Hash)]
#[non_exhaustive]
pub enum Endpoint {
    /// Attached over USB. B200/B210/B205mini.
    Usb(UsbEndpoint),
    /// Attached over UDP. X300/X310, N3xx, E3xx.
    Udp(SocketAddr),
}

impl Endpoint {
    /// Whether this endpoint is reached over a network, which implies packet
    /// loss is possible and MTU matters.
    #[must_use]
    pub fn is_network(&self) -> bool {
        matches!(self, Self::Udp(_))
    }
}

impl std::fmt::Display for Endpoint {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::Usb(usb) => write!(f, "{usb}"),
            Self::Udp(addr) => write!(f, "udp:{addr}"),
        }
    }
}
