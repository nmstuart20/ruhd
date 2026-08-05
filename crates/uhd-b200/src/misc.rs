//! The board's miscellaneous GPIO word, and the two status registers beside it.
//!
//! Ports `b200_impl::update_gpio_state`, `reset_codec`, `get_ref_locked` and
//! `get_fe_pll_locked`.
//!
//! # One register, nine unrelated things
//!
//! `SR_CORE_MISC` is a single write-only settings register whose bits drive
//! board-level pins: the codec's reset line, the reference-clock select, the
//! band-select filters, the MIMO enable, the ATR swap. Write-only is the whole
//! difficulty — there is no way to change one bit without knowing all the
//! others, so the driver has to keep a shadow copy, and every path that touches
//! a bit has to go through the same one.
//!
//! `b200_impl` does this with a plain struct of nine `uint32_t`s that any method
//! may assign to, followed by a call to `update_gpio_state()` that the assigning
//! method must remember to make. [`MiscGpio`] keeps the same bit layout but the
//! shadow is behind a mutex and every setter writes the register itself, so
//! "forgot to flush" is not a reachable state. The one place that ordering is
//! load-bearing — [`reset_codec`](MiscGpio::reset_codec), which is two writes
//! whose *sequence* is the operation — is a method rather than two assignments.
//!
//! # This is on the local core
//!
//! Like the SPI master, and for the same reason: releasing the AD936x's reset
//! is a precondition for the codec clocking, and the radio cores do not answer
//! until it does.

use std::fmt;
use std::sync::Mutex;

use tracing::debug;
use uhd_core::{Error, Result};
use uhd_transport::RegisterIface;
use uhd_transport::usb::B2xxProduct;

use crate::compat::{CoreRegs, FpgaCompat, FpgaShim};

/// Bit positions within `SR_CORE_MISC`, from `update_gpio_state`.
mod bit {
    pub(super) const REF_SEL: u32 = 0;
    pub(super) const MIMO: u32 = 1;
    pub(super) const CODEC_ARST: u32 = 2;
    pub(super) const RX_BANDSEL_C: u32 = 3;
    pub(super) const RX_BANDSEL_B: u32 = 4;
    pub(super) const RX_BANDSEL_A: u32 = 5;
    pub(super) const TX_BANDSEL_B: u32 = 6;
    pub(super) const TX_BANDSEL_A: u32 = 7;
    pub(super) const SWAP_ATR: u32 = 8;
}

/// Bit 0 of `RB32_CORE_MISC`: the external reference is locked.
const REF_LOCKED_BIT: u32 = 1 << 0;

/// Bits 0 and 1 of `RB32_CORE_PLL`: the TX and RX synthesisers are locked.
const TX_PLL_LOCKED_BIT: u32 = 1 << 0;
const RX_PLL_LOCKED_BIT: u32 = 1 << 1;

/// Where the reference clock comes from.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum RefSel {
    /// The board's own TCXO.
    Internal,
    /// The external 10 MHz input.
    External,
}

impl RefSel {
    fn bit(self) -> u32 {
        u32::from(self == Self::External)
    }
}

/// The RX band-select filter path, as three GPIO bits.
///
/// `update_bandsel`'s RX arm. Only one of the three lines is ever high, so the
/// three-bit field is really a one-of-three selector and is spelled as one here.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum RxBand {
    /// Below 2.2 GHz.
    Low,
    /// 2.2 GHz to 4 GHz.
    Mid,
    /// Above 4 GHz.
    High,
}

/// The TX band-select filter path, as two GPIO bits.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum TxBand {
    /// Below 2.5 GHz.
    Low,
    /// At or above 2.5 GHz.
    High,
}

/// Crossover frequencies `update_bandsel` compares against, in Hz.
pub const RX_BAND_EDGES: [f64; 2] = [2.2e9, 4e9];
/// The single TX crossover frequency, in Hz.
pub const TX_BAND_EDGE: f64 = 2.5e9;

impl RxBand {
    /// The band `update_bandsel` picks for a centre frequency.
    #[must_use]
    pub fn for_freq(freq: f64) -> Self {
        if freq < RX_BAND_EDGES[0] {
            Self::Low
        } else if freq < RX_BAND_EDGES[1] {
            Self::Mid
        } else {
            Self::High
        }
    }
}

impl TxBand {
    /// The band `update_bandsel` picks for a centre frequency.
    #[must_use]
    pub fn for_freq(freq: f64) -> Self {
        if freq < TX_BAND_EDGE {
            Self::Low
        } else {
            Self::High
        }
    }
}

/// The shadow copy of `SR_CORE_MISC`.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
struct MiscState {
    ref_sel: bool,
    mimo: bool,
    codec_arst: bool,
    rx_band: Option<RxBand>,
    tx_band: Option<TxBand>,
    swap_atr: bool,
}

impl MiscState {
    /// Packs the state into the word `update_gpio_state` writes.
    fn word(self) -> u32 {
        let mut word = 0;
        word |= u32::from(self.ref_sel) << bit::REF_SEL;
        word |= u32::from(self.mimo) << bit::MIMO;
        word |= u32::from(self.codec_arst) << bit::CODEC_ARST;
        word |= u32::from(self.swap_atr) << bit::SWAP_ATR;
        match self.rx_band {
            Some(RxBand::Low) => word |= 1 << bit::RX_BANDSEL_C,
            Some(RxBand::Mid) => word |= 1 << bit::RX_BANDSEL_B,
            Some(RxBand::High) => word |= 1 << bit::RX_BANDSEL_A,
            None => {}
        }
        match self.tx_band {
            Some(TxBand::Low) => word |= 1 << bit::TX_BANDSEL_B,
            Some(TxBand::High) => word |= 1 << bit::TX_BANDSEL_A,
            None => {}
        }
        word
    }
}

/// The board GPIO word on the local core, plus the status registers beside it.
pub struct MiscGpio {
    bus: Box<dyn RegisterIface>,
    regs: &'static CoreRegs,
    shim: &'static dyn FpgaShim,
    state: Mutex<MiscState>,
}

impl fmt::Debug for MiscGpio {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("MiscGpio")
            .field("state", &self.locked())
            .field("shim", &self.shim.codename())
            .finish_non_exhaustive()
    }
}

impl MiscGpio {
    /// Builds the shadow for a board, without writing anything.
    ///
    /// `board_revision` is the EEPROM's, and together with `product` it decides
    /// [`FpgaShim::frontends_swapped`] — which is the `swap_atr` bit, and the
    /// only field of the initial state that is not zero. Construction stays
    /// write-free so that opening a device remains an operation that does not
    /// change it; call [`MiscGpio::flush`] to put the shadow on the wire.
    #[must_use]
    pub fn new(
        bus: Box<dyn RegisterIface>,
        compat: FpgaCompat,
        product: B2xxProduct,
        board_revision: Option<u16>,
    ) -> Self {
        let shim = compat.shim();
        Self {
            bus,
            regs: shim.core_regs(),
            shim,
            state: Mutex::new(MiscState {
                swap_atr: shim.frontends_swapped(product, board_revision),
                ..MiscState::default()
            }),
        }
    }

    /// The word the shadow currently holds.
    #[must_use]
    pub fn word(&self) -> u32 {
        self.locked().word()
    }

    /// Writes the shadow to `SR_CORE_MISC`.
    ///
    /// # Errors
    ///
    /// Whatever the settings bus reports.
    pub fn flush(&self) -> Result<()> {
        let word = self.locked().word();
        self.bus.poke32(self.regs.sr_misc, word)
    }

    /// Pulses the AD936x's reset line: asserted, then released.
    ///
    /// `b200_impl::reset_codec`, and the step immediately before the codec
    /// driver is constructed. Two writes whose order is the entire content of
    /// the operation, which is why they are one method.
    ///
    /// This is the *analog* reset pin on the board, not the part's soft reset —
    /// [`Ad936x::reset`](ad936x::Ad936x::reset) is that, and it goes over SPI.
    /// Bring-up wants both, this one first.
    ///
    /// # Errors
    ///
    /// Whatever the settings bus reports. A failure between the two writes
    /// leaves the part held in reset, and the error says so rather than being
    /// discovered later as a codec that reads back all zeros.
    pub fn reset_codec(&self) -> Result<()> {
        self.set(|s| s.codec_arst = true)
            .map_err(|e| Error::Device(format!("could not assert the codec reset line: {e}")))?;
        self.set(|s| s.codec_arst = false)
            .map_err(|e| Error::Device(format!("the codec is held in reset: {e}")))?;
        debug!("pulsed the AD936x reset line");
        Ok(())
    }

    /// Selects the reference clock.
    ///
    /// # Errors
    ///
    /// Whatever the settings bus reports.
    pub fn set_ref_sel(&self, sel: RefSel) -> Result<()> {
        self.set(|s| s.ref_sel = sel.bit() != 0)
    }

    /// Sets the MIMO bit, which the FPGA uses to keep two radio chains in step.
    ///
    /// # Errors
    ///
    /// Whatever the settings bus reports.
    pub fn set_mimo(&self, mimo: bool) -> Result<()> {
        self.set(|s| s.mimo = mimo)
    }

    /// Whether radio 0's ATR lines are routed to the AD936x's second frontend.
    #[must_use]
    pub fn swap_atr(&self) -> bool {
        self.locked().swap_atr
    }

    /// Drives the band-select filters for a tuned frequency.
    ///
    /// # Errors
    ///
    /// [`Error::NotSupported`] on a build whose board has no band-select
    /// filters — the minis, where `update_bandsel` returns immediately.
    /// Otherwise whatever the settings bus reports.
    pub fn set_bandsel(&self, rx_freq: Option<f64>, tx_freq: Option<f64>) -> Result<()> {
        if !self.shim.has_bandsel() {
            return Err(Error::not_supported(format!(
                "a {} board has no band-select filters",
                self.shim.codename()
            )));
        }
        self.set(|s| {
            if let Some(freq) = rx_freq {
                s.rx_band = Some(RxBand::for_freq(freq));
            }
            if let Some(freq) = tx_freq {
                s.tx_band = Some(TxBand::for_freq(freq));
            }
        })
    }

    /// Whether the reference clock is locked.
    ///
    /// Bit 0 of `RB32_CORE_MISC`. On a mini this is the whole of the
    /// reference-lock story, because there is no ADF4001 —
    /// [`RefLock::GpioRefSel`](crate::RefLock).
    ///
    /// # Errors
    ///
    /// Whatever the settings bus reports.
    pub fn ref_locked(&self) -> Result<bool> {
        Ok(self.bus.peek32(self.regs.rb32_misc)? & REF_LOCKED_BIT != 0)
    }

    /// Whether the AD936x's transmit and receive synthesisers are locked.
    ///
    /// Bits 0 and 1 of `RB32_CORE_PLL`, returned as `(tx, rx)`.
    ///
    /// # Errors
    ///
    /// Whatever the settings bus reports.
    pub fn fe_pll_locked(&self) -> Result<(bool, bool)> {
        let status = self.bus.peek32(self.regs.rb32_pll)?;
        Ok((
            status & TX_PLL_LOCKED_BIT != 0,
            status & RX_PLL_LOCKED_BIT != 0,
        ))
    }

    /// Mutates the shadow and writes it, under one lock.
    fn set(&self, f: impl FnOnce(&mut MiscState)) -> Result<()> {
        let mut state = self.locked();
        f(&mut state);
        self.bus.poke32(self.regs.sr_misc, state.word())
    }

    fn locked(&self) -> std::sync::MutexGuard<'_, MiscState> {
        self.state
            .lock()
            .unwrap_or_else(std::sync::PoisonError::into_inner)
    }
}

#[cfg(test)]
mod tests {
    use std::sync::Mutex as StdMutex;

    use super::*;

    #[derive(Debug, Default)]
    struct FakeBus {
        pokes: StdMutex<Vec<(u32, u32)>>,
        reads: StdMutex<Vec<u32>>,
    }

    impl FakeBus {
        fn words(&self) -> Vec<u32> {
            self.pokes.lock().unwrap().iter().map(|p| p.1).collect()
        }
    }

    impl RegisterIface for &'static FakeBus {
        fn poke32(&self, addr: u32, data: u32) -> Result<()> {
            self.pokes.lock().unwrap().push((addr, data));
            Ok(())
        }

        fn peek32(&self, _addr: u32) -> Result<u32> {
            Ok(self.reads.lock().unwrap().pop().unwrap_or(0))
        }
    }

    /// A leaked bus, so the test can still read what the GPIO wrote to it.
    fn bus() -> &'static FakeBus {
        Box::leak(Box::new(FakeBus::default()))
    }

    fn gpio(bus: &'static FakeBus, compat: FpgaCompat, product: B2xxProduct) -> MiscGpio {
        MiscGpio::new(Box::new(bus), compat, product, Some(3))
    }

    #[test]
    fn the_bit_layout_matches_update_gpio_state() {
        // Each bit checked against the shift in b200_impl::update_gpio_state.
        let state = MiscState {
            ref_sel: true,
            mimo: true,
            codec_arst: true,
            rx_band: Some(RxBand::Low),
            tx_band: Some(TxBand::Low),
            swap_atr: true,
        };
        assert_eq!(
            state.word(),
            (1 << 8) | (1 << 6) | (1 << 3) | (1 << 2) | (1 << 1) | 1
        );

        // And the other band selections, which pick different bits.
        assert_eq!(
            MiscState {
                rx_band: Some(RxBand::High),
                tx_band: Some(TxBand::High),
                ..MiscState::default()
            }
            .word(),
            (1 << 7) | (1 << 5)
        );
        assert_eq!(
            MiscState {
                rx_band: Some(RxBand::Mid),
                ..MiscState::default()
            }
            .word(),
            1 << 4
        );
    }

    #[test]
    fn construction_writes_nothing() {
        // Opening a device must not change it; the swap_atr bit sits in the
        // shadow until something asks for it.
        let bus = bus();
        let gpio = gpio(bus, FpgaCompat::V16, B2xxProduct::B210);
        assert!(bus.words().is_empty());
        assert_eq!(gpio.word(), 1 << 8, "a B210 swaps its frontends");
    }

    #[test]
    fn resetting_the_codec_asserts_then_releases() {
        // The order is the operation. A device left with bit 2 high is a codec
        // that reads back nothing at all, which is the failure this sequence
        // exists to avoid.
        let bus = bus();
        let gpio = gpio(bus, FpgaCompat::V7, B2xxProduct::B205Mini);
        gpio.reset_codec().unwrap();

        assert_eq!(bus.words(), vec![1 << 2, 0]);
        assert_eq!(gpio.word(), 0);
    }

    #[test]
    fn every_setter_leaves_the_other_bits_alone() {
        // The register is write-only, so a setter that rebuilt the word from
        // nothing would silently drop whatever else was set. A B210's swap_atr
        // bit is the one most likely to be lost that way.
        let bus = bus();
        let gpio = gpio(bus, FpgaCompat::V16, B2xxProduct::B210);

        gpio.set_ref_sel(RefSel::External).unwrap();
        gpio.set_mimo(true).unwrap();
        gpio.set_bandsel(Some(5e9), Some(1e9)).unwrap();

        assert_eq!(
            *bus.words().last().unwrap(),
            (1 << 8) | (1 << 6) | (1 << 5) | (1 << 1) | 1
        );
        assert!(gpio.swap_atr());
    }

    #[test]
    fn a_mini_refuses_bandsel_rather_than_writing_bits_its_board_ignores() {
        // update_bandsel returns early for a mini. Returning an error rather
        // than silently succeeding is the divergence: a caller asking a
        // B205mini to switch filters has misunderstood the board.
        let bus = bus();
        let gpio = gpio(bus, FpgaCompat::V7, B2xxProduct::B205Mini);
        let error = gpio.set_bandsel(Some(5e9), None).unwrap_err();
        assert!(matches!(error, Error::NotSupported(_)), "{error}");
        assert!(bus.words().is_empty());
    }

    #[test]
    fn the_band_edges_are_the_ones_update_bandsel_compares_against() {
        assert_eq!(RxBand::for_freq(2.199e9), RxBand::Low);
        assert_eq!(RxBand::for_freq(2.2e9), RxBand::Mid);
        assert_eq!(RxBand::for_freq(3.999e9), RxBand::Mid);
        assert_eq!(RxBand::for_freq(4e9), RxBand::High);
        assert_eq!(TxBand::for_freq(2.499e9), TxBand::Low);
        assert_eq!(TxBand::for_freq(2.5e9), TxBand::High);
    }

    #[test]
    fn the_lock_bits_are_read_from_the_registers_that_hold_them() {
        let bus = bus();
        bus.reads.lock().unwrap().extend([0b10, 0b1]);
        let gpio = gpio(bus, FpgaCompat::V16, B2xxProduct::B210);

        // RB32_CORE_MISC bit 0, then RB32_CORE_PLL bits 0 and 1.
        assert!(gpio.ref_locked().unwrap());
        assert_eq!(gpio.fe_pll_locked().unwrap(), (false, true));
    }
}
