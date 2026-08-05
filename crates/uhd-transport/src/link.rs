//! Frame-oriented links: USB bulk endpoints and UDP sockets.
//!
//! # Why frames are borrowed by ID rather than returned
//!
//! The obvious signature, `fn get_recv_buff(&mut self) -> Option<&[u8]>`, ties
//! the frame's lifetime to the borrow of the link, so you cannot hold a frame
//! and ask for the next one — which is exactly what a streamer doing packet
//! reassembly needs to do. The other obvious signature, returning
//! `Box<dyn Buffer>`, allocates on every packet in the hot path.
//!
//! So [`RecvLink`] hands back a [`FrameId`] into a pool the link owns.
//! `frame_bytes(id)` reads it, `release_recv_buff(id)` returns it. Zero
//! allocation, object-safe, and it maps directly onto both a libusb transfer
//! ring and a `recvmmsg` buffer array.

use std::time::Duration;

use uhd_core::Result;

/// A handle to a frame in a link's buffer pool.
///
/// Only meaningful to the link that issued it. A link may reuse an ID as soon
/// as it has been released.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, PartialOrd, Ord)]
pub struct FrameId(pub u32);

/// Fixed properties of a link, known before any traffic flows.
pub trait LinkInfo {
    /// Number of frames in the pool. This bounds how many frames can be held at
    /// once, and on a network link it sets how much jitter can be absorbed.
    fn num_frames(&self) -> usize;

    /// Capacity of one frame, in bytes.
    ///
    /// For UDP this is the MTU minus headers; for USB bulk it is the transfer
    /// size the backend chose.
    fn frame_size(&self) -> usize;
}

/// The receive half of a link.
pub trait RecvLink: LinkInfo + Send {
    /// Waits for a frame, up to `timeout`.
    ///
    /// `Ok(None)` means the timeout expired with nothing to read, which is
    /// ordinary. The returned frame stays checked out until
    /// [`RecvLink::release_recv_buff`] is called for it.
    fn get_recv_buff(&mut self, timeout: Duration) -> Result<Option<FrameId>>;

    /// The bytes actually received in `frame`, which may be shorter than
    /// [`LinkInfo::frame_size`].
    ///
    /// # Panics
    ///
    /// Implementations may panic if `frame` is not currently checked out.
    fn frame_bytes(&self, frame: FrameId) -> &[u8];

    /// Returns `frame` to the pool.
    fn release_recv_buff(&mut self, frame: FrameId);
}

/// The send half of a link.
pub trait SendLink: LinkInfo + Send {
    /// Claims a frame to fill, waiting up to `timeout` for one to free up.
    ///
    /// `Ok(None)` means every frame is still in flight.
    fn get_send_buff(&mut self, timeout: Duration) -> Result<Option<FrameId>>;

    /// The writable bytes of `frame`, [`LinkInfo::frame_size`] long.
    ///
    /// # Panics
    ///
    /// Implementations may panic if `frame` is not currently checked out.
    fn frame_bytes_mut(&mut self, frame: FrameId) -> &mut [u8];

    /// Sends the first `len` bytes of `frame` and returns it to the pool.
    fn commit_send_buff(&mut self, frame: FrameId, len: usize) -> Result<()>;

    /// Returns `frame` to the pool without sending anything.
    fn release_send_buff(&mut self, frame: FrameId);
}

/// Asserts the link traits stay object-safe: a streamer holds
/// `Box<dyn RecvLink>` without caring whether it wraps USB or UDP.
const _: Option<&dyn RecvLink> = None;
const _: Option<&dyn SendLink> = None;
