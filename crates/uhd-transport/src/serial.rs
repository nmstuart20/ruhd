//! Serial buses hanging off the device: SPI, I2C, GPIO.
//!
//! These are the *general* forms — a SPI master with a slave select and clock
//! edges, an I2C master, a bank of pins — and they live here so that a backend
//! can offer them without a caller having to name the backend. `uhd-b200`'s
//! `spi_core_3000` port implements [`SpiIface`]; a `ruhd` command that wants to
//! poke a daughterboard EEPROM asks for [`I2cIface`].
//!
//! # What a device driver should ask for instead
//!
//! A driver for a *part* — the AD936x, a clock chip — should define the narrow
//! trait it actually needs in its own crate rather than taking one of these.
//! `ad936x::SpiInterface` is one transaction with no slave select and no edges,
//! because slave selects and edges are facts about the board and the part driver
//! has no business knowing them. `uhd-b200` implements both traits and is the
//! only place the two meet.
//!
//! The distinction matters at the dependency level: `ad936x` does not depend on
//! this crate at all, so the transceiver driver cannot accidentally acquire an
//! opinion about how it is wired up.

use uhd_core::{Error, Result};

/// Which clock edge a SPI signal is sampled or driven on.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum ClockEdge {
    /// Rising edge.
    Rise,
    /// Falling edge.
    Fall,
}

/// SPI timing for one transaction.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct SpiConfig {
    /// Edge on which the master drives MOSI.
    pub mosi_edge: ClockEdge,
    /// Edge on which the master samples MISO.
    pub miso_edge: ClockEdge,
    /// SPI clock divider, if the default is not suitable.
    pub divider: Option<u16>,
}

impl Default for SpiConfig {
    /// SPI mode 0: drive on the falling edge, sample on the rising edge. This is
    /// what the AD9361 expects.
    fn default() -> Self {
        Self {
            mosi_edge: ClockEdge::Fall,
            miso_edge: ClockEdge::Rise,
            divider: None,
        }
    }
}

/// A SPI master.
pub trait SpiIface: Send + Sync + std::fmt::Debug {
    /// Runs one transaction against `slave`.
    ///
    /// `data` is left-justified into `num_bits`: the most significant of
    /// `num_bits` goes out first. Returns the value shifted back in, or zero
    /// when `readback` is false.
    fn transact_spi(
        &self,
        slave: u32,
        config: &SpiConfig,
        data: u32,
        num_bits: usize,
        readback: bool,
    ) -> Result<u32>;

    /// Write-only transaction.
    fn write_spi(&self, slave: u32, config: &SpiConfig, data: u32, num_bits: usize) -> Result<()> {
        self.transact_spi(slave, config, data, num_bits, false)
            .map(|_| ())
    }

    /// Read-back transaction.
    fn read_spi(&self, slave: u32, config: &SpiConfig, data: u32, num_bits: usize) -> Result<u32> {
        self.transact_spi(slave, config, data, num_bits, true)
    }
}

/// An I2C master, used for EEPROMs and clock chips.
pub trait I2cIface: Send + Sync + std::fmt::Debug {
    /// Writes `bytes` to the device at `addr`.
    fn write_i2c(&self, addr: u16, bytes: &[u8]) -> Result<()>;

    /// Reads `len` bytes from the device at `addr`.
    fn read_i2c(&self, addr: u16, len: usize) -> Result<Vec<u8>>;

    /// Writes to an EEPROM at `addr`, starting at `offset`.
    ///
    /// The default sends `offset` followed by the data as one transaction, which
    /// is right for the single-byte-addressed 24Cxx parts USRPs use.
    fn write_eeprom(&self, addr: u16, offset: u8, bytes: &[u8]) -> Result<()> {
        let mut buf = Vec::with_capacity(bytes.len() + 1);
        buf.push(offset);
        buf.extend_from_slice(bytes);
        self.write_i2c(addr, &buf)
    }

    /// Reads `len` bytes from an EEPROM at `addr`, starting at `offset`.
    fn read_eeprom(&self, addr: u16, offset: u8, len: usize) -> Result<Vec<u8>> {
        self.write_i2c(addr, &[offset])?;
        self.read_i2c(addr, len)
    }
}

/// Direction of a GPIO pin bank.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum GpioDirection {
    /// Pin is driven by the device.
    Output,
    /// Pin is read by the device.
    Input,
}

/// A bank of general-purpose I/O pins.
pub trait GpioIface: Send + Sync + std::fmt::Debug {
    /// Sets which pins in `bank` are outputs. A set bit means output.
    fn set_gpio_direction(&self, bank: &str, mask: u32, value: u32) -> Result<()>;

    /// Drives the output pins of `bank`, restricted to `mask`.
    fn set_gpio_out(&self, bank: &str, mask: u32, value: u32) -> Result<()>;

    /// Reads the current pin states of `bank`.
    fn get_gpio_in(&self, bank: &str) -> Result<u32>;

    /// The GPIO bank names this device exposes.
    fn gpio_banks(&self) -> Vec<String> {
        Vec::new()
    }
}

/// Convenience for backends that expose no serial buses at all.
///
/// Returns [`Error::NotSupported`] from everything, so a partially implemented
/// backend can be honest about it rather than silently no-op.
#[derive(Debug, Clone, Copy, Default)]
pub struct Unsupported;

impl SpiIface for Unsupported {
    fn transact_spi(
        &self,
        _slave: u32,
        _config: &SpiConfig,
        _data: u32,
        _num_bits: usize,
        _readback: bool,
    ) -> Result<u32> {
        Err(Error::not_supported("spi"))
    }
}

impl I2cIface for Unsupported {
    fn write_i2c(&self, _addr: u16, _bytes: &[u8]) -> Result<()> {
        Err(Error::not_supported("i2c"))
    }

    fn read_i2c(&self, _addr: u16, _len: usize) -> Result<Vec<u8>> {
        Err(Error::not_supported("i2c"))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn default_spi_config_is_mode_zero() {
        let cfg = SpiConfig::default();
        assert_eq!(cfg.mosi_edge, ClockEdge::Fall);
        assert_eq!(cfg.miso_edge, ClockEdge::Rise);
    }

    #[test]
    fn unsupported_refuses_rather_than_no_ops() {
        assert!(Unsupported.write_i2c(0x50, &[0]).is_err());
        assert!(
            Unsupported
                .write_spi(0, &SpiConfig::default(), 0, 24)
                .is_err()
        );
    }
}
