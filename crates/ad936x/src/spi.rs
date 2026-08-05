//! The bus this driver reaches the part over, and the command words it puts on
//! it.
//!
//! # Why the trait lives here
//!
//! [`SpiInterface`] is deliberately the *narrowest* thing an AD936x needs: one
//! transaction, `num_bits` wide, MSB first, optionally reading back. It is not a
//! general SPI master — there is no slave select, no clock edge, no divider,
//! because none of those are facts about the transceiver. They are facts about
//! the board it is soldered to, and a board that changes any of them should not
//! be able to change this driver.
//!
//! That split puts each fact in exactly one place:
//!
//! | Fact | Whose | Where it lives |
//! | --- | --- | --- |
//! | 24-bit word, R/W bit at 23, address at 8, data at 0 | the part | [`Registers`], below |
//! | which slave line, what clock rate, which edges | the board | the backend's `SpiInterface` impl |
//!
//! `uhd-b200` implements [`SpiInterface`] by driving `spi_core_3000` at the
//! AD9361 slave select and 1 MHz; an E310 would implement it over its SoC's SPI
//! controller. Neither appears in this crate, and this crate appears in neither
//! of theirs beyond the trait.
//!
//! UHD draws the line one layer lower: `ad9361_io_spi` in `ad9361_ctrl.cpp`
//! holds a `uhd::spi_iface` and builds a `spi_config_t` itself, which is why the
//! FPGA MISO-edge workaround — a fact about a *Xilinx SPI core* — is written
//! into the transceiver driver. Here that workaround is the backend's, where it
//! is true.

use std::fmt;
use std::sync::Arc;

use uhd_core::{Error, Result};

/// One SPI transaction against an AD936x.
///
/// Implemented by whatever board the part sits on. The implementor already
/// knows which slave select the part is on and how fast to clock it; all that is
/// left to say is what bits to send.
///
/// `&self` rather than `&mut self`, and `Send + Sync`, for the same reason the
/// rest of the workspace works that way: a control thread retuning and a
/// monitoring thread reading a lock bit are the same device. Implementors
/// serialise internally.
pub trait SpiInterface: Send + Sync + fmt::Debug {
    /// Clocks the low `num_bits` of `data` out to the part, most significant of
    /// those bits first, and returns what came back.
    ///
    /// `data` is right-justified on the way in — bit 0 is the last bit on the
    /// wire — and the return value is right-justified too. When `readback` is
    /// false the return value is unspecified and should be ignored;
    /// implementations that can skip the read should.
    ///
    /// # Errors
    ///
    /// Whatever the underlying bus reports. [`Error::InvalidArg`] if `num_bits`
    /// is zero or above 32.
    fn transact(&self, data: u32, num_bits: usize, readback: bool) -> Result<u32>;
}

/// Width of every AD936x SPI transaction, in bits.
///
/// `AD9361_SPI_NUM_BITS` in `ad9361_ctrl.cpp`. One read/write bit, one
/// burst-count bit this driver leaves clear, 14 address bits and 8 data bits —
/// the AD9361 datasheet's "SPI Interface" section.
pub const SPI_NUM_BITS: usize = 24;

/// Bit 23, set on a write.
const WRITE_CMD: u32 = 0x0080_0000;

/// Bits 21:8, the register address.
const ADDR_MASK: u32 = 0x003f_ff00;
const ADDR_SHIFT: u32 = 8;

/// Bits 7:0, the data byte.
const DATA_MASK: u32 = 0x0000_00ff;

/// The highest register address the command word can express.
pub const MAX_REG: u16 = (ADDR_MASK >> ADDR_SHIFT) as u16;

/// The AD936x register file, reached over a [`SpiInterface`].
///
/// This is where the part's command-word format lives, and it is the only place
/// in the workspace that knows it. Everything above works in registers and
/// fields; everything below works in bits on a wire.
#[derive(Clone)]
pub struct Registers {
    spi: Arc<dyn SpiInterface>,
}

impl fmt::Debug for Registers {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("Registers").field("spi", &self.spi).finish()
    }
}

impl Registers {
    /// Wraps a bus.
    #[must_use]
    pub fn new(spi: Arc<dyn SpiInterface>) -> Self {
        Self { spi }
    }

    /// The bus underneath, for a caller that needs to hand it on.
    #[must_use]
    pub fn spi(&self) -> &Arc<dyn SpiInterface> {
        &self.spi
    }

    /// Reads one register.
    ///
    /// # Errors
    ///
    /// [`Error::InvalidArg`] if `reg` is above [`MAX_REG`]; otherwise whatever
    /// the bus reports.
    pub fn peek8(&self, reg: u16) -> Result<u8> {
        let word = Self::command(reg, None)?;
        let value = self.spi.transact(word, SPI_NUM_BITS, true)?;
        Ok((value & DATA_MASK) as u8)
    }

    /// Writes one register.
    ///
    /// # Errors
    ///
    /// As [`Registers::peek8`].
    pub fn poke8(&self, reg: u16, value: u8) -> Result<()> {
        let word = Self::command(reg, Some(value))?;
        self.spi.transact(word, SPI_NUM_BITS, false).map(|_| ())
    }

    /// Reads `len` consecutive registers, starting at `reg`.
    ///
    /// Register by register rather than as a burst: the AD936x's burst mode
    /// counts *down* from the supplied address, and no caller so far wants that
    /// enough to justify the asymmetry.
    ///
    /// # Errors
    ///
    /// As [`Registers::peek8`].
    pub fn peek_bytes(&self, reg: u16, len: usize) -> Result<Vec<u8>> {
        (0..len)
            .map(|i| {
                let addr = u16::try_from(usize::from(reg) + i)
                    .map_err(|_| Error::invalid_arg(format!("ad936x register {reg:#06x}+{i}")))?;
                self.peek8(addr)
            })
            .collect()
    }

    /// Reads a register, applies `f` to it, and writes the result back.
    ///
    /// Returns the value written. Not atomic — nothing on this bus is — but it
    /// keeps read-modify-write out of every call site that needs one bit.
    ///
    /// # Errors
    ///
    /// As [`Registers::peek8`].
    pub fn modify8(&self, reg: u16, f: impl FnOnce(u8) -> u8) -> Result<u8> {
        let value = f(self.peek8(reg)?);
        self.poke8(reg, value)?;
        Ok(value)
    }

    /// Builds the 24-bit command word for a read (`data` is `None`) or a write.
    fn command(reg: u16, data: Option<u8>) -> Result<u32> {
        if reg > MAX_REG {
            return Err(Error::invalid_arg(format!(
                "ad936x register {reg:#06x} is above the {MAX_REG:#06x} the \
                 {SPI_NUM_BITS}-bit command word can address"
            )));
        }
        let mut word = (u32::from(reg) << ADDR_SHIFT) & ADDR_MASK;
        if let Some(value) = data {
            word |= WRITE_CMD | u32::from(value);
        }
        Ok(word)
    }
}

#[cfg(test)]
pub(crate) mod tests {
    use std::collections::BTreeMap;
    use std::sync::Mutex;

    use super::*;

    /// A bus that records what it was asked to send and answers reads.
    ///
    /// The register file's whole job is turning `(reg, value)` into a 24-bit
    /// word, so a fake that captures the word is the only way to test it
    /// without hardware — and the words are checkable by hand against the
    /// AD9361 datasheet.
    ///
    /// Reads come from one of two places. [`FakeSpi::with_reads`] queues answers
    /// in order, which suits a test that cares about a single readback;
    /// [`FakeSpi::with_regs`] answers by address, which is what a test of a
    /// long sequence needs, because such a sequence polls a status bit an
    /// unpredictable number of times and reads registers this crate has not
    /// named. Anything unaccounted for reads back zero, the same as a bus with
    /// nothing on it.
    #[derive(Debug, Default)]
    pub(crate) struct FakeSpi {
        pub(crate) sent: Mutex<Vec<(u32, usize, bool)>>,
        reads: Mutex<Vec<u32>>,
        regs: Mutex<BTreeMap<u16, u8>>,
    }

    impl FakeSpi {
        pub(crate) fn with_reads(reads: &[u32]) -> Self {
            Self {
                reads: Mutex::new(reads.iter().rev().copied().collect()),
                ..Self::default()
            }
        }

        /// A part whose registers hold the given values.
        pub(crate) fn with_regs(regs: &[(u16, u8)]) -> Self {
            Self {
                regs: Mutex::new(regs.iter().copied().collect()),
                ..Self::default()
            }
        }

        pub(crate) fn words(&self) -> Vec<u32> {
            self.sent.lock().unwrap().iter().map(|s| s.0).collect()
        }

        /// Every write, decoded back into `(register, value)`.
        ///
        /// Decoding rather than asserting on raw words is what makes a
        /// sequence test readable against the C++, which is written in
        /// `poke8(reg, val)` too.
        pub(crate) fn pokes(&self) -> Vec<(u16, u8)> {
            self.sent
                .lock()
                .unwrap()
                .iter()
                .filter(|&&(word, _, _)| word & WRITE_CMD != 0)
                .map(|&(word, _, _)| (address_of(word), (word & DATA_MASK) as u8))
                .collect()
        }
    }

    impl SpiInterface for FakeSpi {
        fn transact(&self, data: u32, num_bits: usize, readback: bool) -> Result<u32> {
            self.sent.lock().unwrap().push((data, num_bits, readback));
            if !readback {
                return Ok(0);
            }
            if let Some(queued) = self.reads.lock().unwrap().pop() {
                return Ok(queued);
            }
            let value = self
                .regs
                .lock()
                .unwrap()
                .get(&address_of(data))
                .copied()
                .unwrap_or(0);
            Ok(u32::from(value))
        }
    }

    /// The register a command word addresses.
    fn address_of(word: u32) -> u16 {
        ((word & ADDR_MASK) >> ADDR_SHIFT) as u16
    }

    #[test]
    fn a_write_word_matches_the_reference_framing() {
        // ad9361_io_spi::poke8: WRITE_CMD | (reg << 8) | val, 24 bits, no
        // readback. Worked by hand: 0x800000 | 0x003700 | 0x9a.
        let spi = Arc::new(FakeSpi::default());
        Registers::new(spi.clone()).poke8(0x037, 0x9a).unwrap();

        assert_eq!(
            spi.sent.lock().unwrap().as_slice(),
            &[(0x0080_379a, 24, false)]
        );
    }

    #[test]
    fn a_read_word_leaves_the_write_bit_and_the_data_byte_clear() {
        // ad9361_io_spi::peek8: READ_CMD is zero, so the word is address only,
        // and the part drives the low byte back.
        let spi = Arc::new(FakeSpi::with_reads(&[0x00ff_ff08]));

        assert_eq!(Registers::new(spi.clone()).peek8(0x037).unwrap(), 0x08);
        assert_eq!(
            spi.sent.lock().unwrap().as_slice(),
            &[(0x0000_3700, 24, true)]
        );
    }

    #[test]
    fn only_the_low_byte_of_a_readback_is_the_register() {
        // The bus returns a full 32-bit shift register, and the top bits are
        // whatever was on MISO while the command was going out. UHD masks with
        // 0xFF for exactly this reason; without it the product-id gate would
        // compare against garbage.
        let spi = Arc::new(FakeSpi::with_reads(&[0xdead_be08]));
        assert_eq!(Registers::new(spi).peek8(0x037).unwrap(), 0x08);
    }

    #[test]
    fn the_address_field_is_fourteen_bits_wide() {
        let spi = Arc::new(FakeSpi::default());
        let regs = Registers::new(spi.clone());

        assert_eq!(MAX_REG, 0x3fff);
        regs.poke8(MAX_REG, 0).unwrap();
        assert_eq!(spi.words(), vec![WRITE_CMD | ADDR_MASK]);
        assert!(regs.poke8(MAX_REG + 1, 0).is_err());
        assert!(regs.peek8(MAX_REG + 1).is_err());
    }

    #[test]
    fn an_address_never_bleeds_into_the_write_bit() {
        // A register number large enough to reach bit 23 if the mask were
        // wrong. That failure would turn every high-address read into a write,
        // so it is worth an assertion of its own.
        let spi = Arc::new(FakeSpi::default());
        Registers::new(spi.clone()).peek8(MAX_REG).unwrap();
        assert_eq!(spi.words()[0] & WRITE_CMD, 0, "a read must not set bit 23");
    }

    #[test]
    fn modify_reads_then_writes_the_same_register() {
        let spi = Arc::new(FakeSpi::with_reads(&[0x0000_0001]));

        assert_eq!(
            Registers::new(spi.clone())
                .modify8(0x000, |v| v | 0x02)
                .unwrap(),
            0x03
        );
        assert_eq!(spi.words(), vec![0x0000_0000, WRITE_CMD | 0x03]);
    }
}
