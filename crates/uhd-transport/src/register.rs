//! Register access to the FPGA's settings bus.

use uhd_core::Result;

/// Reads and writes device registers.
///
/// Addresses are byte addresses into the device's settings-bus space, and all
/// values are little-endian on the wire regardless of host byte order.
///
/// The same trait covers wildly different plumbing: on a B2xx these become USB
/// control transfers to the FX3, and on an X310 they become ZPU packets over
/// UDP. Everything above only ever sees peek and poke, so the AD9361 driver and
/// the register maps do not have to know which they are talking to.
///
/// `&self`, not `&mut self`, because a receive thread checking overflow counters
/// and a control thread retuning need concurrent access; implementations
/// serialise internally.
pub trait RegisterIface: Send + Sync + std::fmt::Debug {
    /// Writes a 32-bit register.
    fn poke32(&self, addr: u32, data: u32) -> Result<()>;

    /// Reads a 32-bit register.
    fn peek32(&self, addr: u32) -> Result<u32>;

    /// Writes a 64-bit register as two 32-bit writes, low word first.
    ///
    /// Override when the device has a native 64-bit path; the default's two
    /// transactions are not atomic, which matters for registers that latch on
    /// the high word.
    fn poke64(&self, addr: u32, data: u64) -> Result<()> {
        self.poke32(addr, data as u32)?;
        self.poke32(addr.wrapping_add(4), (data >> 32) as u32)
    }

    /// Reads a 64-bit register as two 32-bit reads, low word first.
    fn peek64(&self, addr: u32) -> Result<u64> {
        let lo = u64::from(self.peek32(addr)?);
        let hi = u64::from(self.peek32(addr.wrapping_add(4))?);
        Ok((hi << 32) | lo)
    }
}
