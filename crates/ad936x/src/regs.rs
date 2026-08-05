//! Register addresses, the shadow copies of the ones that are read-modify-write
//! in software, and the one register decoded so far.
//!
//! Only the registers something in this crate actually touches appear here. The
//! AD936x has around a thousand of them and transcribing the map wholesale would
//! be a large file that nothing reads; the ones the initialisation sequence
//! walks stay beside the fixed sequences that use them.
//!
//! # About the names
//!
//! Where the AD9361 register map names a register unambiguously, that name is
//! used. Where the reference driver pokes a register whose map name would be a
//! guess, the constant is named for *what the poke does* instead, and the
//! address is the authority — `ad9361_device.cpp` addresses everything
//! numerically, so a wrong name here can never turn into a wrong write. Fixed
//! magic sequences are not given names at all: they stay as `(address, value)`
//! tables next to the code that plays them, so they can be diffed against the
//! C++ line for line.

use std::fmt;

/// `SPI Configuration` — the soft reset lives in bit 0.
///
/// Writing 1 then 0 resets the digital section; `ad9361_device_t::initialize`
/// opens with exactly that.
pub const SPI_CONF: u16 = 0x000;

/// `TX Enable and Filter Control` — which TX chains are on, and the
/// interpolation of each stage of the TX chain.
///
/// Bits 7:6 enable TX2 / TX1; the rest select the HB3/interpolator, HB2, HB1 and
/// TX FIR rates. Written from the [`ChipRegs::txfilt`] shadow, because
/// `set_active_chains` changes the top two bits and `_setup_rates` changes the
/// bottom six, and neither can read the register back to find the other's work:
/// the enable bits do not read back as written.
pub const TX_ENABLE_AND_FILTER_CTRL: u16 = 0x002;

/// `RX Enable and Filter Control` — the receive-side mirror of
/// [`TX_ENABLE_AND_FILTER_CTRL`], holding decimations instead of interpolations.
pub const RX_ENABLE_AND_FILTER_CTRL: u16 = 0x003;

/// `Input Select` — which RF port feeds the receiver and which is driven by the
/// transmitter.
///
/// Shadowed as [`ChipRegs::inputsel`] and written whenever the filter registers
/// are, which is why it appears in the clock path at all: `_setup_rates` pokes
/// all four of `0x002`, `0x003`, `0x004` and `0x00A` together.
pub const INPUT_SELECT: u16 = 0x004;

/// `RX Clock and Data Delay` — clock delay in bits 7:4, data delay in bits 3:0.
pub const RX_CLOCK_DATA_DELAY: u16 = 0x006;

/// `TX Clock and Data Delay` — as [`RX_CLOCK_DATA_DELAY`].
pub const TX_CLOCK_DATA_DELAY: u16 = 0x007;

/// `Clock Enable` — selects the reference clock path (crystal or external) and
/// turns the clock trees on.
pub const CLOCK_ENABLE: u16 = 0x009;

/// `BBPLL` — the BBPLL VCO divider in bits 2:0, and DAC-clock-equals-half-ADC in
/// bit 3.
///
/// Shadowed as [`ChipRegs::bbpll`]: `_tune_bbvco` owns the divider bits and
/// `_setup_rates` owns bit 3, and they run one after the other.
pub const BBPLL: u16 = 0x00a;

/// `Parallel Port Configuration 1` — I/Q swap, frame-sync mode and, on LVDS, the
/// 1R1T / 2R2T timing selection.
pub const PARALLEL_PORT_CONF_1: u16 = 0x010;

/// `Parallel Port Configuration 2`.
pub const PARALLEL_PORT_CONF_2: u16 = 0x011;

/// `Parallel Port Configuration 3` — CMOS / LVDS mode and the port's bus width.
pub const PARALLEL_PORT_CONF_3: u16 = 0x012;

/// `Product ID`.
///
/// Bits 7:3 identify the part family and bits 2:0 its silicon revision. This is
/// the first register `initialize` reads, and its purpose there is not really to
/// learn the part number — the board already knows what is soldered to it — but
/// to prove the SPI path works at all. A bus that is not wired up reads back
/// `0x00` or `0xff`, and both fail the check.
pub const PRODUCT_ID: u16 = 0x037;

/// The value bits 7:3 of [`PRODUCT_ID`] hold on a working part.
///
/// `0x8` after masking with `0xf8`, which is the comparison
/// `ad9361_device_t::initialize` makes.
pub const PRODUCT_ID_AD936X: u8 = 0x08;

/// Mask selecting the family field of [`PRODUCT_ID`].
pub const PRODUCT_ID_MASK: u8 = 0xf8;

/// Mask selecting the revision field of [`PRODUCT_ID`].
pub const REVISION_MASK: u8 = 0x07;

/// `LVDS Bias Control`. Written only in LVDS mode.
pub const LVDS_BIAS_CTRL: u16 = 0x03c;

/// `LVDS Invert Control 1`. Written only in LVDS mode.
pub const LVDS_INVERT_CTRL_1: u16 = 0x03d;

/// `LVDS Invert Control 2`. Written only in LVDS mode.
pub const LVDS_INVERT_CTRL_2: u16 = 0x03e;

/// Fractional part of the BBPLL divide word, bits 23:16.
pub const BBPLL_NFRAC_23_16: u16 = 0x041;

/// Fractional part of the BBPLL divide word, bits 15:8.
pub const BBPLL_NFRAC_15_8: u16 = 0x042;

/// Fractional part of the BBPLL divide word, bits 7:0.
pub const BBPLL_NFRAC_7_0: u16 = 0x043;

/// Integer part of the BBPLL divide word.
pub const BBPLL_NINT: u16 = 0x044;

/// BBPLL reference clock scaler — how the 40 MHz reference is divided before it
/// reaches the PLL. The reference driver always writes `0x00`, REFCLK / 1.
pub const BBPLL_REF_CLOCK_SCALER: u16 = 0x045;

/// BBPLL charge pump current, in 25 µA steps, minus one. Bits 5:0.
pub const BBPLL_CP_CURRENT: u16 = 0x046;

/// BBPLL lock status — bit 7 is the lock flag `_calibrate_lock_bbpll` polls.
///
/// The AD9361 register map calls this the channel-1 overflow register; the
/// BBPLL's lock indicator shares it.
pub const BBPLL_LOCK_STATUS: u16 = 0x05e;

/// Mask of the BBPLL lock flag within [`BBPLL_LOCK_STATUS`].
pub const BBPLL_LOCKED: u8 = 0x80;

/// First register of the TX FIR programming bank.
///
/// The RX and TX FIR filters have identical register layouts at different bases;
/// `_program_fir_filter` picks between them and then works in `base + n`.
pub const TX_FIR_BASE: u16 = 0x060;

/// First register of the RX FIR programming bank.
pub const RX_FIR_BASE: u16 = 0x0f0;

/// The state this driver keeps a software copy of.
///
/// `ad9361_device_t::chip_regs_t`. These seven registers are written from more
/// than one place with each writer owning different bits of them, and the part
/// will not read them back reliably — the chain-enable bits of the filter
/// registers in particular do not survive a read. UHD's answer is a shadow copy,
/// and this is it, kept field-for-field so the two can be compared.
///
/// The initial values are the ones `initialize` assigns before it touches the
/// part, not the part's reset values.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ChipRegs {
    /// RF VCO dividers. Owned by the RF tuning code.
    pub vcodivs: u8,
    /// [`INPUT_SELECT`]. Owned by the RF tuning code; `_setup_rates` only
    /// writes it out.
    pub inputsel: u8,
    /// [`RX_ENABLE_AND_FILTER_CTRL`]. Bits 7:6 from `set_active_chains`, bits
    /// 5:0 from `_setup_rates`.
    pub rxfilt: u8,
    /// [`TX_ENABLE_AND_FILTER_CTRL`]. As [`ChipRegs::rxfilt`].
    pub txfilt: u8,
    /// [`BBPLL`]. Bits 2:0 from `_tune_bbvco`, bit 3 from `_setup_rates`.
    pub bbpll: u8,
    /// Baseband filter tune config. Owned by the analog filter calibration.
    pub bbftune_config: u8,
    /// Baseband filter tune mode. Owned by the analog filter calibration.
    pub bbftune_mode: u8,
}

impl Default for ChipRegs {
    /// The values `initialize` starts from — `ad9361_device.cpp:1583-1589`, not
    /// the part's power-on state.
    fn default() -> Self {
        Self {
            vcodivs: 0x00,
            inputsel: 0x30,
            rxfilt: 0x00,
            txfilt: 0x00,
            bbpll: 0x02,
            bbftune_config: 0x1e,
            bbftune_mode: 0x1e,
        }
    }
}

/// A decoded [`PRODUCT_ID`] readback.
///
/// # It does not say which part this is
///
/// An AD9361 and an AD9364 report the same family field — the AD9364 is the
/// same die with one receive and one transmit chain bonded out — so this cannot
/// tell a B210's AD9361 from a B205mini's AD9364. Channel counts come from the
/// board, which is why they are a runtime query on `Device` rather than
/// something the transceiver driver reports. What this *does* say is that
/// something on the other end of the SPI bus is an AD936x and is answering.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct ProductId {
    /// The register exactly as it read back.
    pub raw: u8,
}

impl ProductId {
    /// Wraps a raw readback.
    #[must_use]
    pub const fn from_raw(raw: u8) -> Self {
        Self { raw }
    }

    /// The family field, bits 7:3.
    #[must_use]
    pub const fn family(self) -> u8 {
        self.raw & PRODUCT_ID_MASK
    }

    /// The silicon revision, bits 2:0.
    #[must_use]
    pub const fn revision(self) -> u8 {
        self.raw & REVISION_MASK
    }

    /// Whether the family field is the one an AD936x reports.
    #[must_use]
    pub const fn is_ad936x(self) -> bool {
        self.family() == PRODUCT_ID_AD936X
    }
}

impl fmt::Display for ProductId {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        if self.is_ad936x() {
            write!(f, "AD936x rev {} ({:#04x})", self.revision(), self.raw)
        } else {
            write!(f, "unrecognised product id {:#04x}", self.raw)
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn the_family_and_revision_fields_split_where_the_datasheet_says() {
        // Family 0x08 in the top five bits, revision in the bottom three.
        let id = ProductId::from_raw(0x0a);
        assert!(id.is_ad936x());
        assert_eq!(id.family(), PRODUCT_ID_AD936X);
        assert_eq!(id.revision(), 2);
        assert_eq!(id.to_string(), "AD936x rev 2 (0x0a)");
    }

    #[test]
    fn a_dead_bus_fails_the_check_either_way_it_floats() {
        // The two things an unconnected SPI bus reads back. Both must be
        // rejected, because "0x00 & 0xf8 == 0" and "0xff & 0xf8 == 0xf8" are
        // the readings that mean the gate is testing nothing.
        for raw in [0x00, 0xff] {
            let id = ProductId::from_raw(raw);
            assert!(!id.is_ad936x(), "{raw:#04x}");
            assert!(id.to_string().contains("unrecognised"));
        }
    }

    #[test]
    fn the_revision_bits_never_reach_the_family_comparison() {
        // Every revision of a real part passes, which is the point of masking
        // rather than comparing the whole byte.
        for revision in 0..=REVISION_MASK {
            assert!(ProductId::from_raw(PRODUCT_ID_AD936X | revision).is_ad936x());
        }
    }
}
