//! The GPIO/ATR core, and the B2xx front-end states it drives.
//!
//! Ports `gpio_atr_3000.cpp` and `b200_impl::update_atrs`.
//!
//! # What ATR is for
//!
//! Automatic Transmit/Receive is a small state machine in the FPGA that drives a
//! bank of GPIO pins from the *streaming* state rather than from the host. Four
//! registers hold the pin pattern for four conditions — idle, receiving,
//! transmitting, both — and the FPGA selects between them as the radio's
//! transmit and receive chains turn on and off. On a B2xx those pins are the
//! transmit/receive switches and the front-panel LEDs, and the point of doing it
//! in hardware is that a switch has to change within one sample of the burst it
//! is switching for, which a USB round trip cannot promise.
//!
//! Each bit is independently either ATR-driven or a static output, chosen by the
//! "ATR disable" register, and the idle register does double duty: it holds the
//! idle pattern for ATR bits and the static level for GPIO bits. That is why
//! [`GpioAtr`] keeps two shadows for one register and composes them on the way
//! out.
//!
//! # These are radio-core registers
//!
//! `SR_ATR` is per radio chain, so an ATR core is reached through
//! [`RadioCtrl::radio`](crate::RadioCtrl::radio) and answers nothing until the
//! AD936x is clocking. The front-panel bank at `SR_FP_GPIO` is on the same core.
//! Contrast [`misc`](crate::misc), whose board GPIO word is on the local core
//! and works from the moment the bitstream is in.

use std::fmt;
use std::sync::Mutex;

use uhd_core::{Error, Result};
use uhd_transport::{GpioDirection, GpioIface, RegisterIface};

/// `SR_ATR`, the per-radio ATR bank. A byte address, `TOREG(12)`.
pub const SR_ATR: u32 = 12 * 4;

/// `SR_FP_GPIO`, the front-panel bank. `TOREG(200)`.
pub const SR_FP_GPIO: u32 = 200 * 4;

/// `RB32_FP_GPIO`, where the front-panel bank reads back.
pub const RB32_FP_GPIO: u32 = 32;

/// Name of the front-panel bank, matching UHD's `/gpio/FP0` property path.
pub const FP_BANK: &str = "FP0";

/// Which of the four ATR patterns a write targets.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum AtrReg {
    /// Neither chain streaming.
    Idle,
    /// Receiving only.
    RxOnly,
    /// Transmitting only.
    TxOnly,
    /// Both, at once.
    FullDuplex,
}

impl AtrReg {
    /// Every pattern, in register order.
    pub const ALL: [Self; 4] = [Self::Idle, Self::RxOnly, Self::TxOnly, Self::FullDuplex];
}

/// Whether a pin follows the ATR state machine or a static register.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum AtrMode {
    /// Driven by the ATR state machine.
    Atr,
    /// Driven by the static GPIO output register.
    Gpio,
}

/// Settings-bus offsets of one `gpio_atr_3000` bank.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct AtrOffsets {
    /// Idle pattern, and the static output level for GPIO-mode bits.
    pub idle: u32,
    /// Receive-only pattern.
    pub rx: u32,
    /// Transmit-only pattern.
    pub tx: u32,
    /// Full-duplex pattern.
    pub duplex: u32,
    /// Data direction: a set bit is an output.
    pub ddr: u32,
    /// ATR disable: a set bit is static rather than ATR-driven.
    pub disable: u32,
    /// Where the pin state reads back, or `None` on a write-only bank.
    pub readback: Option<u32>,
}

impl AtrOffsets {
    /// Six consecutive settings registers from `base`, plus a readback.
    ///
    /// `gpio_atr_offsets::make_default`, with the stride fixed at one register.
    #[must_use]
    pub const fn make_default(base: u32, readback: u32) -> Self {
        Self {
            idle: base,
            rx: base + 4,
            tx: base + 8,
            duplex: base + 12,
            ddr: base + 16,
            disable: base + 20,
            readback: Some(readback),
        }
    }

    /// The same without a readback address.
    ///
    /// `gpio_atr_offsets::make_write_only`. UHD spells "no readback" as the
    /// sentinel address `0xFFFFFFFF`, relying on it not being a multiple of
    /// four; an `Option` says it without depending on that.
    #[must_use]
    pub const fn write_only(base: u32) -> Self {
        Self {
            readback: None,
            ..Self::make_default(base, 0)
        }
    }
}

/// The shadow state of one bank.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
struct AtrState {
    /// Idle pattern for ATR-driven bits.
    idle: u32,
    /// Static level for GPIO-mode bits. Shares the idle register.
    gpio_out: u32,
    rx: u32,
    tx: u32,
    duplex: u32,
    ddr: u32,
    /// A set bit is static rather than ATR-driven.
    disable: u32,
}

impl AtrState {
    /// What the idle register should actually hold: the ATR pattern for
    /// ATR-driven bits, the static level for the rest.
    ///
    /// `atr_idle_reg_t::flush`.
    fn idle_word(self) -> u32 {
        (self.idle & !self.disable) | (self.gpio_out & self.disable)
    }
}

/// A `gpio_atr_3000` bank.
///
/// # Writes only what changed
///
/// Every register here is write-only in the FPGA, so the shadow is the only
/// record of what is in them. Keeping the last word written as well lets a
/// no-op setter cost nothing, which matters because
/// [`set_b2xx_states`](GpioAtr::set_b2xx_states) is called on every streamer
/// start and stop.
pub struct GpioAtr {
    bus: Box<dyn RegisterIface>,
    offsets: AtrOffsets,
    state: Mutex<(AtrState, Written)>,
}

/// The last word written to each register, so an unchanged one is not rewritten.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
struct Written {
    idle: Option<u32>,
    rx: Option<u32>,
    tx: Option<u32>,
    duplex: Option<u32>,
    ddr: Option<u32>,
    disable: Option<u32>,
}

impl fmt::Debug for GpioAtr {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("GpioAtr")
            .field("offsets", &self.offsets)
            .field("state", &self.locked().0)
            .finish_non_exhaustive()
    }
}

impl GpioAtr {
    /// Binds a bank, without writing anything.
    #[must_use]
    pub fn new(bus: Box<dyn RegisterIface>, offsets: AtrOffsets) -> Self {
        Self {
            bus,
            offsets,
            state: Mutex::new((AtrState::default(), Written::default())),
        }
    }

    /// The per-radio ATR bank, which has no readback.
    #[must_use]
    pub fn radio(bus: Box<dyn RegisterIface>) -> Self {
        Self::new(bus, AtrOffsets::write_only(SR_ATR))
    }

    /// The front-panel bank, which does.
    #[must_use]
    pub fn front_panel(bus: Box<dyn RegisterIface>) -> Self {
        Self::new(bus, AtrOffsets::make_default(SR_FP_GPIO, RB32_FP_GPIO))
    }

    /// Chooses, for every bit in `mask`, whether the ATR state machine or the
    /// static output register drives it.
    ///
    /// # Errors
    ///
    /// Whatever the settings bus reports.
    pub fn set_atr_mode(&self, mode: AtrMode, mask: u32) -> Result<()> {
        let (mut state, mut written) = *self.locked();
        // A set bit in the disable register means *static*, so ATR mode clears.
        let value = if mode == AtrMode::Atr { 0 } else { u32::MAX };
        state.disable = (value & mask) | (state.disable & !mask);
        self.write(&mut state, &mut written)
    }

    /// Chooses, for every bit in `mask`, whether the pin is an output.
    ///
    /// # Errors
    ///
    /// Whatever the settings bus reports.
    pub fn set_gpio_ddr(&self, dir: GpioDirection, mask: u32) -> Result<()> {
        let (mut state, mut written) = *self.locked();
        let value = if dir == GpioDirection::Output {
            u32::MAX
        } else {
            0
        };
        state.ddr = (value & mask) | (state.ddr & !mask);
        self.write(&mut state, &mut written)
    }

    /// Writes one of the four ATR patterns, restricted to `mask`.
    ///
    /// # Errors
    ///
    /// Whatever the settings bus reports.
    pub fn set_atr_reg(&self, reg: AtrReg, value: u32, mask: u32) -> Result<()> {
        let (mut state, mut written) = *self.locked();
        let target = match reg {
            AtrReg::Idle => &mut state.idle,
            AtrReg::RxOnly => &mut state.rx,
            AtrReg::TxOnly => &mut state.tx,
            AtrReg::FullDuplex => &mut state.duplex,
        };
        *target = (value & mask) | (*target & !mask);
        self.write(&mut state, &mut written)
    }

    /// Drives the bits that are in [`AtrMode::Gpio`], restricted to `mask`.
    ///
    /// # Errors
    ///
    /// Whatever the settings bus reports.
    pub fn set_gpio_out(&self, value: u32, mask: u32) -> Result<()> {
        let (mut state, mut written) = *self.locked();
        state.gpio_out = (value & mask) | (state.gpio_out & !mask);
        self.write(&mut state, &mut written)
    }

    /// Reads the pin states back.
    ///
    /// # Errors
    ///
    /// [`Error::NotSupported`] on a write-only bank — the per-radio ATR bank has
    /// no readback address at all. Otherwise whatever the settings bus reports.
    pub fn read_gpio(&self) -> Result<u32> {
        let addr = self.offsets.readback.ok_or_else(|| {
            Error::not_supported("this gpio bank is write-only and has no readback register")
        })?;
        self.bus.peek32(addr)
    }

    /// The pattern currently shadowed for one ATR register.
    #[must_use]
    pub fn atr_reg(&self, reg: AtrReg) -> u32 {
        let state = self.locked().0;
        match reg {
            AtrReg::Idle => state.idle,
            AtrReg::RxOnly => state.rx,
            AtrReg::TxOnly => state.tx,
            AtrReg::FullDuplex => state.duplex,
        }
    }

    /// Loads the four B2xx front-end patterns for one radio chain.
    ///
    /// `b200_impl::update_atrs`, for the side named by `side`. `rx` and `tx`
    /// say whether that direction has a streamer attached, and `rx2` picks the
    /// RX2 antenna over TX/RX.
    ///
    /// # Errors
    ///
    /// Whatever the settings bus reports.
    pub fn set_b2xx_states(&self, side: Side, rx: bool, tx: bool, rx2: bool) -> Result<()> {
        let states = side.states(rx, tx, rx2);
        self.set_atr_reg(AtrReg::Idle, states.idle, u32::MAX)?;
        self.set_atr_reg(AtrReg::RxOnly, states.rx_only, u32::MAX)?;
        self.set_atr_reg(AtrReg::TxOnly, states.tx_only, u32::MAX)?;
        self.set_atr_reg(AtrReg::FullDuplex, states.full_duplex, u32::MAX)
    }

    /// Puts every bit under the ATR state machine, which is how `setup_radio`
    /// leaves the per-radio bank.
    ///
    /// # Errors
    ///
    /// Whatever the settings bus reports.
    pub fn enable_atr(&self) -> Result<()> {
        self.set_atr_mode(AtrMode::Atr, u32::MAX)
    }

    /// Writes every register whose composed word differs from the last one
    /// written, then commits the shadow.
    ///
    /// # A deliberate divergence
    ///
    /// The idle register is rewritten whenever the disable register changes,
    /// because its contents are a function of both. `gpio_atr_3000` flushes
    /// only the register the setter touched, so a `set_atr_mode` after the
    /// patterns are loaded leaves the *old* composition on the wire until
    /// something else happens to write the idle register — the pins are wrong
    /// in the meantime, and how long that lasts depends on call order.
    fn write(&self, state: &mut AtrState, written: &mut Written) -> Result<()> {
        let updates = [
            (self.offsets.disable, state.disable, &mut written.disable),
            (self.offsets.ddr, state.ddr, &mut written.ddr),
            (self.offsets.idle, state.idle_word(), &mut written.idle),
            (self.offsets.rx, state.rx, &mut written.rx),
            (self.offsets.tx, state.tx, &mut written.tx),
            (self.offsets.duplex, state.duplex, &mut written.duplex),
        ];
        for (addr, value, last) in updates {
            if *last != Some(value) {
                self.bus.poke32(addr, value)?;
                *last = Some(value);
            }
        }
        *self.locked() = (*state, *written);
        Ok(())
    }

    fn locked(&self) -> std::sync::MutexGuard<'_, (AtrState, Written)> {
        self.state
            .lock()
            .unwrap_or_else(std::sync::PoisonError::into_inner)
    }
}

impl GpioIface for GpioAtr {
    fn set_gpio_direction(&self, bank: &str, mask: u32, value: u32) -> Result<()> {
        self.check_bank(bank)?;
        self.set_gpio_ddr(GpioDirection::Output, mask & value)?;
        self.set_gpio_ddr(GpioDirection::Input, mask & !value)
    }

    fn set_gpio_out(&self, bank: &str, mask: u32, value: u32) -> Result<()> {
        self.check_bank(bank)?;
        GpioAtr::set_gpio_out(self, value, mask)
    }

    fn get_gpio_in(&self, bank: &str) -> Result<u32> {
        self.check_bank(bank)?;
        self.read_gpio()
    }

    fn gpio_banks(&self) -> Vec<String> {
        vec![FP_BANK.to_owned()]
    }
}

impl GpioAtr {
    fn check_bank(&self, bank: &str) -> Result<()> {
        if bank == FP_BANK {
            Ok(())
        } else {
            Err(Error::KeyNotFound(format!(
                "gpio bank `{bank}`; this device has only `{FP_BANK}`"
            )))
        }
    }
}

/// Which of the AD936x's two front ends an ATR bank drives.
///
/// `b200_regs.hpp` declares two full sets of constants, `TX_ENABLE1`/`SFDX1_RX`
/// and `TX_ENABLE2`/`SFDX2_RX`, and they occupy the same bit positions — each
/// side has its own register bank, so the pins do not have to share a word. The
/// side is carried here anyway because it is what `update_atrs` branches on and
/// because a board that did wire the two differently would have nowhere else to
/// say so.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum Side {
    /// Front end 1, the ports labelled "RF B" on a B200/B210.
    Fe1,
    /// Front end 2, "RF A". A B205mini's AD9364 has no second front end.
    Fe2,
}

/// The four patterns one side's ATR bank should hold.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct AtrStates {
    /// Neither chain streaming.
    pub idle: u32,
    /// Receiving only.
    pub rx_only: u32,
    /// Transmitting only.
    pub tx_only: u32,
    /// Both.
    pub full_duplex: u32,
}

/// The ATR control bits, from `b200_regs.hpp`. Identical positions on both
/// sides; only the names differ there.
mod ctrl {
    pub(super) const TX_ENABLE: u32 = 1 << 7;
    pub(super) const SFDX_RX: u32 = 1 << 6;
    pub(super) const SFDX_TX: u32 = 1 << 5;
    pub(super) const SRX_RX: u32 = 1 << 4;
    pub(super) const SRX_TX: u32 = 1 << 3;
    pub(super) const LED_RX: u32 = 1 << 2;
    pub(super) const LED_TXRX_RX: u32 = 1 << 1;
    pub(super) const LED_TXRX_TX: u32 = 1 << 0;
}

impl Side {
    /// Every side, in front-end order.
    pub const ALL: [Self; 2] = [Self::Fe1, Self::Fe2];

    /// `STATE_RX*_OFF`.
    #[must_use]
    pub fn off(self) -> u32 {
        ctrl::SFDX_RX | ctrl::SRX_TX
    }

    /// `STATE_RX*_RX2` — receiving on the RX2 port.
    #[must_use]
    pub fn rx_rx2(self) -> u32 {
        self.off() | ctrl::LED_RX
    }

    /// `STATE_RX*_TXRX` — receiving on the TX/RX port.
    ///
    /// The two sides differ here in the reference: side 1 is written
    /// `SRX1_RX | SRX1_TX | LED_TXRX_RX1` and side 2 `SRX2_TX | SRX2_RX |
    /// LED_TXRX_RX2`, which is the same three bits in a different order.
    #[must_use]
    pub fn rx_txrx(self) -> u32 {
        ctrl::SRX_RX | ctrl::SRX_TX | ctrl::LED_TXRX_RX
    }

    /// `STATE_FDX*_TXRX`.
    #[must_use]
    pub fn full_duplex(self) -> u32 {
        ctrl::TX_ENABLE | ctrl::SFDX_RX | ctrl::SFDX_TX | ctrl::LED_TXRX_TX | ctrl::LED_RX
    }

    /// `STATE_TX*_TXRX`.
    #[must_use]
    pub fn tx_txrx(self) -> u32 {
        ctrl::TX_ENABLE | ctrl::SFDX_RX | ctrl::SFDX_TX | ctrl::LED_TXRX_TX
    }

    /// The four patterns for a given set of enables.
    ///
    /// `update_atrs`, which computes the full-duplex pattern from the other
    /// three rather than storing it: with only one direction streaming, the
    /// full-duplex slot holds that direction's pattern, because the state
    /// machine can still enter it.
    #[must_use]
    pub fn states(self, rx: bool, tx: bool, rx2: bool) -> AtrStates {
        let rx_only = if rx {
            if rx2 { self.rx_rx2() } else { self.rx_txrx() }
        } else {
            self.off()
        };
        let tx_only = if tx { self.tx_txrx() } else { self.off() };
        let full_duplex = match (rx, tx) {
            (true, true) => self.full_duplex(),
            (true, false) => rx_only,
            (false, true) => tx_only,
            (false, false) => self.off(),
        };
        AtrStates {
            idle: self.off(),
            rx_only,
            tx_only,
            full_duplex,
        }
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
        fn pokes(&self) -> Vec<(u32, u32)> {
            self.pokes.lock().unwrap().clone()
        }

        fn last(&self, addr: u32) -> Option<u32> {
            self.pokes()
                .into_iter()
                .filter(|(a, _)| *a == addr)
                .map(|(_, v)| v)
                .next_back()
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

    fn bus() -> &'static FakeBus {
        Box::leak(Box::new(FakeBus::default()))
    }

    #[test]
    fn the_register_layout_matches_gpio_atr_offsets() {
        // make_default: four ATR registers, then duplex, ddr, disable, at a
        // stride of one settings register.
        let offsets = AtrOffsets::make_default(SR_FP_GPIO, RB32_FP_GPIO);
        assert_eq!(SR_FP_GPIO, 800);
        assert_eq!(
            [
                offsets.idle,
                offsets.rx,
                offsets.tx,
                offsets.duplex,
                offsets.ddr,
                offsets.disable
            ],
            [800, 804, 808, 812, 816, 820]
        );
        assert_eq!(offsets.readback, Some(32));

        // The per-radio bank is the same shape with no readback.
        assert_eq!(SR_ATR, 48);
        assert_eq!(AtrOffsets::write_only(SR_ATR).readback, None);
    }

    #[test]
    fn the_b2xx_state_table_matches_b200_regs_hpp() {
        // Values transcribed from the STATE_* constants and checked as
        // literals, so a mistyped shift shows up here rather than as a switch
        // that never closes.
        let fe1 = Side::Fe1;
        assert_eq!(fe1.off(), 0b0100_1000); // SFDX1_RX | SRX1_TX
        assert_eq!(fe1.rx_rx2(), 0b0100_1100); // ... | LED_RX1
        assert_eq!(fe1.rx_txrx(), 0b0001_1010); // SRX1_RX | SRX1_TX | LED_TXRX_RX1
        assert_eq!(fe1.full_duplex(), 0b1110_0101);
        assert_eq!(fe1.tx_txrx(), 0b1110_0001);

        // Both sides use the same bit positions; only the names differ.
        for side in Side::ALL {
            assert_eq!(side.off(), fe1.off());
            assert_eq!(side.tx_txrx(), fe1.tx_txrx());
        }
    }

    #[test]
    fn the_full_duplex_slot_follows_whichever_direction_is_live() {
        // update_atrs: with one direction streaming, the full-duplex pattern
        // is that direction's, not the idle one — the state machine can still
        // enter the slot and the switches must be right when it does.
        let side = Side::Fe1;

        let rx_only = side.states(true, false, false);
        assert_eq!(rx_only.full_duplex, rx_only.rx_only);
        assert_eq!(rx_only.rx_only, side.rx_txrx());

        let tx_only = side.states(false, true, false);
        assert_eq!(tx_only.full_duplex, tx_only.tx_only);

        let both = side.states(true, true, false);
        assert_eq!(both.full_duplex, side.full_duplex());

        let neither = side.states(false, false, false);
        assert_eq!(neither.full_duplex, side.off());
        assert_eq!(neither.idle, side.off());
    }

    #[test]
    fn the_antenna_choice_only_moves_the_receive_pattern() {
        let txrx = Side::Fe1.states(true, true, false);
        let rx2 = Side::Fe1.states(true, true, true);
        assert_eq!(rx2.rx_only, Side::Fe1.rx_rx2());
        assert_ne!(txrx.rx_only, rx2.rx_only);
        assert_eq!(txrx.tx_only, rx2.tx_only);
        assert_eq!(txrx.idle, rx2.idle);
    }

    #[test]
    fn the_idle_register_composes_atr_and_static_bits() {
        // atr_idle_reg_t::flush: ATR-driven bits take the idle pattern, static
        // bits take the GPIO output level, in one register.
        let bus = bus();
        let atr = GpioAtr::front_panel(Box::new(bus));

        atr.set_atr_mode(AtrMode::Gpio, 0x0000_00f0).unwrap();
        atr.set_atr_reg(AtrReg::Idle, 0xffff_ffff, u32::MAX)
            .unwrap();
        atr.set_gpio_out(0x0000_00a0, u32::MAX).unwrap();

        // Low nibble from the idle pattern, bits 4-7 from the static level.
        assert_eq!(atr.locked().0.idle_word(), 0xffff_ffaf);
        assert_eq!(bus.last(800), Some(0xffff_ffaf));
    }

    #[test]
    fn a_mode_change_rewrites_the_idle_register_it_invalidates() {
        // The divergence from gpio_atr_3000, which flushes only the register
        // the setter named. Here the composed idle word is written whenever
        // either of its two inputs moves.
        let bus = bus();
        let atr = GpioAtr::front_panel(Box::new(bus));

        atr.set_atr_reg(AtrReg::Idle, 0x0000_000f, u32::MAX)
            .unwrap();
        atr.set_gpio_out(0x0000_00f0, u32::MAX).unwrap();
        assert_eq!(bus.last(800), Some(0x0000_000f), "all bits are ATR-driven");

        // Handing the low nibble to the static register must change what is on
        // the pins immediately, not at the next unrelated write.
        atr.set_atr_mode(AtrMode::Gpio, 0x0000_00ff).unwrap();
        assert_eq!(bus.last(820), Some(0x0000_00ff));
        assert_eq!(bus.last(800), Some(0x0000_00f0));
    }

    #[test]
    fn an_unchanged_register_is_not_rewritten() {
        // update_atrs runs on every streamer start and stop, and each poke is
        // a USB round trip.
        let bus = bus();
        let atr = GpioAtr::radio(Box::new(bus));

        atr.set_b2xx_states(Side::Fe1, true, false, false).unwrap();
        let after_first = bus.pokes().len();
        atr.set_b2xx_states(Side::Fe1, true, false, false).unwrap();
        assert_eq!(bus.pokes().len(), after_first, "the second call is a no-op");

        atr.set_b2xx_states(Side::Fe1, true, true, false).unwrap();
        assert!(bus.pokes().len() > after_first);
    }

    #[test]
    fn a_masked_write_leaves_the_bits_outside_the_mask_alone() {
        let bus = bus();
        let atr = GpioAtr::radio(Box::new(bus));

        atr.set_atr_reg(AtrReg::RxOnly, 0xffff_ffff, 0x0000_00ff)
            .unwrap();
        atr.set_atr_reg(AtrReg::RxOnly, 0x0000_0000, 0x0000_000f)
            .unwrap();
        assert_eq!(atr.atr_reg(AtrReg::RxOnly), 0x0000_00f0);
    }

    #[test]
    fn a_write_only_bank_refuses_a_readback_rather_than_peeking_at_nothing() {
        // The per-radio bank genuinely has no readback address. UHD reaches
        // the same conclusion through a sentinel value; this cannot be
        // configured wrong.
        assert!(GpioAtr::radio(Box::new(bus())).read_gpio().is_err());
        assert!(GpioAtr::front_panel(Box::new(bus())).read_gpio().is_ok());
    }

    #[test]
    fn the_gpio_trait_splits_a_direction_mask_into_two_writes() {
        let bus = bus();
        let atr = GpioAtr::front_panel(Box::new(bus));

        // Bits 0-3 out, bits 4-7 in, and nothing outside the mask touched.
        atr.set_gpio_direction(FP_BANK, 0x0000_00ff, 0x0000_000f)
            .unwrap();
        assert_eq!(bus.last(816), Some(0x0000_000f));

        assert!(atr.set_gpio_direction("FP1", 0, 0).is_err());
        assert_eq!(atr.gpio_banks(), vec![FP_BANK.to_owned()]);
    }
}
