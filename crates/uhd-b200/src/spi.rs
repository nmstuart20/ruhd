//! The FPGA's SPI master, and the two things hanging off it.
//!
//! Ports `spi_core_3000.cpp` and `b200_local_spi_core` from `b200_cores.cpp`.
//!
//! # Three registers and a readback
//!
//! [`SpiCore`] is a thin thing: a divider register, a control register holding
//! the slave select, the transaction width and the clock edges, and a data
//! register whose write *is* the trigger. The result, if one was asked for,
//! appears at a readback address. Divider and control are cached, so a run of
//! identical transactions costs one poke each — which matters, because every
//! poke is a USB round trip and the AD936x initialisation sequence is hundreds
//! of register writes long.
//!
//! # Why this is on the local core
//!
//! `SR_CORE_SPI` is a *local*-core register, clocked by the FPGA's own bus
//! clock, and that is not an accident of the register map. The AD936x supplies
//! the radio cores' sample clock and is started over SPI, so a SPI master on a
//! radio core could never be reached in time to start the part that clocks it.
//! The practical consequence is the useful one: [`SpiCore`] works the moment the
//! bitstream is in, which is what makes reading the codec's product ID the
//! earliest meaningful hardware gate this driver has.
//!
//! # Two slaves, two clock rates
//!
//! A full-size B200/B210 has an ADF4001 reference PLL on slave 1 alongside the
//! AD936x on slave 0, and it wants a far slower clock — 10 kHz against 1 MHz —
//! because of the time constant on its SPI lines. [`LocalSpi`] owns that switch.
//! A mini has no ADF4001 at all ([`RefLock::GpioRefSel`](crate::RefLock)), so on
//! a B205mini the PLL half is dead code that costs one enum variant.

use std::fmt;
use std::sync::{Arc, Mutex};

use tracing::trace;
use uhd_core::{Error, Result};
use uhd_transport::{ClockEdge, RegisterIface, SpiConfig, SpiIface};

use crate::compat::FpgaCompat;
use crate::radio_ctrl::CtrlPort;

/// The FPGA's settings-bus clock, in Hz. `B200_BUS_CLOCK_RATE`.
pub const BUS_CLOCK_RATE: f64 = 100e6;

/// Clock rate to run the AD936x's SPI at. `AD9361_SPI_RATE`.
pub const AD9361_SPI_RATE: f64 = 1e6;

/// Clock rate to run the ADF4001's SPI at. `ADF4001_SPI_RATE`.
///
/// Deliberately slow: the reference PLL's SPI lines have a large time constant.
pub const ADF4001_SPI_RATE: f64 = 10e3;

/// Slave select of the AD936x. `AD9361_SLAVENO`.
pub const AD9361_SLAVE: u32 = 1 << 0;

/// Slave select of the ADF4001. `ADF4001_SLAVENO`.
pub const ADF4001_SLAVE: u32 = 1 << 1;

/// SPI timing the AD936x is driven with.
///
/// Mode 0 would sample MISO on the rising edge, and that is what the part
/// expects. The falling edge here is a workaround for the FPGA's SPI core,
/// carried over from `ad9361_io_spi` where it sits under a `TODO (Ashish)`
/// noting it "should be EDGE_RISE".
///
/// It lives on this side of the [`SpiInterface`](ad936x::SpiInterface) boundary
/// rather than in `ad936x`, which is where UHD keeps it, because it is a fact
/// about *this FPGA image* and not about the transceiver: a board reaching an
/// AD936x over a SoC SPI controller has no reason to inherit it.
pub const CODEC_SPI_CONFIG: SpiConfig = SpiConfig {
    mosi_edge: ClockEdge::Fall,
    miso_edge: ClockEdge::Fall,
    divider: None,
};

/// Largest transaction the control word's width field can express.
const MAX_NUM_BITS: usize = 0x3f;

/// Register offsets of one `spi_core_3000` instance.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
struct SpiOffsets {
    divider: u32,
    control: u32,
    data: u32,
    readback: u32,
}

impl SpiOffsets {
    /// Three consecutive settings registers from `base`, plus a readback.
    const fn new(base: u32, readback: u32) -> Self {
        Self {
            divider: base,
            control: base + 4,
            data: base + 8,
            readback,
        }
    }
}

#[derive(Debug, Default)]
struct SpiCache {
    /// `None` until the first transaction writes one.
    ///
    /// UHD seeds both caches with `0`, which means a computed divider of zero
    /// is never actually poked. Harmless there — the FPGA's register also comes
    /// out of configuration as zero — but an `Option` says what is meant and
    /// costs at most one extra poke per session.
    divider: Option<u32>,
    control: Option<u32>,
    /// Divider to use when a config does not name its own.
    default_divider: u32,
}

/// A `spi_core_3000` instance on the settings bus.
///
/// Serialises internally: a transaction is three pokes and a peek, and two
/// callers interleaving those would swap each other's control words.
pub struct SpiCore {
    regs: Arc<dyn RegisterIface>,
    offsets: SpiOffsets,
    cache: Mutex<SpiCache>,
}

impl fmt::Debug for SpiCore {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("SpiCore")
            .field("offsets", &self.offsets)
            .finish_non_exhaustive()
    }
}

impl SpiCore {
    /// Binds a core at `base` with its readback at `readback`.
    ///
    /// Writes nothing: the divider and control registers are written lazily by
    /// the first transaction, so constructing this is not an operation on the
    /// device.
    #[must_use]
    pub fn new(regs: Arc<dyn RegisterIface>, base: u32, readback: u32) -> Self {
        Self {
            regs,
            offsets: SpiOffsets::new(base, readback),
            cache: Mutex::new(SpiCache::default()),
        }
    }

    /// Sets the divider used when a [`SpiConfig`] does not name one.
    ///
    /// `div` is the ratio of the bus clock to the wanted SPI clock;
    /// [`divider_for`] turns a rate into it. The register holds
    /// `div / 2 - 0.5` truncated, because the core's output is
    /// `f_bus / (2 * (reg + 1))`.
    pub fn set_divider(&self, div: f64) {
        self.locked().default_divider = encode_divider(div);
    }

    /// Runs one transaction.
    ///
    /// # Errors
    ///
    /// [`Error::InvalidArg`] if `num_bits` is zero or wider than the control
    /// word's field allows; otherwise whatever the settings bus reports.
    pub fn transact(
        &self,
        slave: u32,
        config: &SpiConfig,
        data: u32,
        num_bits: usize,
        readback: bool,
    ) -> Result<u32> {
        if num_bits == 0 || num_bits > 32 {
            return Err(Error::invalid_arg(format!(
                "a spi transaction is 1 to 32 bits wide, not {num_bits}"
            )));
        }

        let mut cache = self.locked();
        let divider = match config.divider {
            // The resulting rate is f_bus / (2 * (reg + 1)), so this rounds the
            // clock down rather than up — a slave that cannot follow is worse
            // than one clocked slower than asked.
            Some(div) => u32::from(div.saturating_sub(1)) / 2,
            None => cache.default_divider,
        };
        if cache.divider != Some(divider) {
            self.regs.poke32(self.offsets.divider, divider)?;
            cache.divider = Some(divider);
        }

        let control = control_word(slave, config, num_bits);
        if cache.control != Some(control) {
            self.regs.poke32(self.offsets.control, control)?;
            cache.control = Some(control);
        }

        // Left-justified: the core shifts out from the top of the word, and
        // writing this register is what starts the transaction.
        let data_out = data << (32 - num_bits);
        self.regs.poke32(self.offsets.data, data_out)?;
        trace!(slave, num_bits, data_out, readback, "spi transaction");

        if readback {
            self.regs.peek32(self.offsets.readback)
        } else {
            Ok(0)
        }
    }

    fn locked(&self) -> std::sync::MutexGuard<'_, SpiCache> {
        self.cache
            .lock()
            .unwrap_or_else(std::sync::PoisonError::into_inner)
    }
}

impl SpiIface for SpiCore {
    fn transact_spi(
        &self,
        slave: u32,
        config: &SpiConfig,
        data: u32,
        num_bits: usize,
        readback: bool,
    ) -> Result<u32> {
        self.transact(slave, config, data, num_bits, readback)
    }
}

/// The divider register value for a bus-clock-to-SPI-clock ratio.
///
/// `spi_core_3000::set_divider`. Truncating rather than rounding is UHD's, and
/// is the safe direction: it can only make the clock faster than asked by less
/// than one step, and the two rates this driver uses — 100 and 10 000 — both
/// land exactly.
#[must_use]
pub fn encode_divider(div: f64) -> u32 {
    if !div.is_finite() || div <= 0.0 {
        return 0;
    }
    ((div / 2.0) - 0.5) as u32
}

/// The ratio to pass [`SpiCore::set_divider`] for a wanted SPI clock.
#[must_use]
pub fn divider_for(spi_rate: f64) -> f64 {
    BUS_CLOCK_RATE / spi_rate
}

/// Packs a control word: slave select, width, and the two clock edges.
fn control_word(slave: u32, config: &SpiConfig, num_bits: usize) -> u32 {
    let mut word = slave & 0x00ff_ffff;
    word |= ((num_bits as u32) & (MAX_NUM_BITS as u32)) << 24;
    if config.mosi_edge == ClockEdge::Fall {
        word |= 1 << 31;
    }
    if config.miso_edge == ClockEdge::Rise {
        word |= 1 << 30;
    }
    word
}

/// Which peripheral the shared SPI master is currently clocked for.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum Perif {
    /// The AD936x transceiver, at [`AD9361_SPI_RATE`].
    Codec,
    /// The ADF4001 reference PLL, at [`ADF4001_SPI_RATE`]. Full-size boards
    /// only.
    Pll,
}

impl Perif {
    /// The SPI clock this peripheral is driven at.
    #[must_use]
    pub fn spi_rate(self) -> f64 {
        match self {
            Self::Codec => AD9361_SPI_RATE,
            Self::Pll => ADF4001_SPI_RATE,
        }
    }

    /// Its slave select.
    #[must_use]
    pub fn slave(self) -> u32 {
        match self {
            Self::Codec => AD9361_SLAVE,
            Self::Pll => ADF4001_SLAVE,
        }
    }
}

impl fmt::Display for Perif {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Codec => f.write_str("codec"),
            Self::Pll => f.write_str("reference pll"),
        }
    }
}

/// The board's one SPI master, shared between the codec and the reference PLL.
///
/// `b200_local_spi_core`. Both slaves hang off the same core at different clock
/// rates, so the rate switch and the transaction have to be one atomic step —
/// otherwise a PLL write can land while the divider still says 1 MHz.
///
/// # Scoped rather than sticky
///
/// UHD exposes `change_perif` and `restore_perif` as two calls and relies on
/// `b200_ref_pll_ctrl` bracketing every PLL access by hand; a path that returns
/// early between them leaves the master clocked for the wrong slave. Here the
/// peripheral is an argument to the transaction, so the two cannot come apart.
#[derive(Debug)]
pub struct LocalSpi {
    core: SpiCore,
    /// Which peripheral the core's divider is currently set for.
    perif: Mutex<Option<Perif>>,
}

impl LocalSpi {
    /// Builds the master on the local core of a negotiated FPGA build.
    ///
    /// `bus` must be a port onto the *local* core — see the module docs for why
    /// the SPI master could not be anywhere else. The register offsets come
    /// from the shim rather than from constants, so a future major that moved
    /// `SR_CORE_SPI` is a table edit.
    #[must_use]
    pub fn new(bus: Arc<dyn RegisterIface>, compat: FpgaCompat) -> Self {
        let regs = compat.shim().core_regs();
        Self {
            core: SpiCore::new(bus, regs.sr_spi, regs.rb32_spi),
            perif: Mutex::new(None),
        }
    }

    /// Builds the master on a [`RadioCtrl`](crate::RadioCtrl)'s local port.
    #[must_use]
    pub fn on_local_core(port: CtrlPort, compat: FpgaCompat) -> Self {
        Self::new(Arc::new(port), compat)
    }

    /// The core underneath, for a caller that wants a raw transaction.
    #[must_use]
    pub fn core(&self) -> &SpiCore {
        &self.core
    }

    /// Runs one transaction against `perif`, at that peripheral's clock rate.
    ///
    /// # Errors
    ///
    /// As [`SpiCore::transact`].
    pub fn transact_as(
        &self,
        perif: Perif,
        config: &SpiConfig,
        data: u32,
        num_bits: usize,
        readback: bool,
    ) -> Result<u32> {
        let mut current = self
            .perif
            .lock()
            .unwrap_or_else(std::sync::PoisonError::into_inner);
        if *current != Some(perif) {
            self.core.set_divider(divider_for(perif.spi_rate()));
            *current = Some(perif);
            trace!(%perif, "spi master reclocked");
        }
        self.core
            .transact(perif.slave(), config, data, num_bits, readback)
    }

    /// Which peripheral the master is currently clocked for, if any transaction
    /// has run yet.
    #[must_use]
    pub fn perif(&self) -> Option<Perif> {
        *self
            .perif
            .lock()
            .unwrap_or_else(std::sync::PoisonError::into_inner)
    }
}

/// The AD936x's view of the board's SPI master.
///
/// This is the whole of what `ad936x` is allowed to know about a B2xx: it hands
/// down a word and gets one back. Slave select, clock rate and the FPGA's
/// MISO-edge workaround are all applied here, which is why the transceiver
/// driver compiles without this crate in its dependency graph.
#[derive(Debug, Clone)]
pub struct CodecSpi {
    spi: Arc<LocalSpi>,
}

impl CodecSpi {
    /// Binds the codec's slave select to a board SPI master.
    #[must_use]
    pub fn new(spi: Arc<LocalSpi>) -> Self {
        Self { spi }
    }
}

impl ad936x::SpiInterface for CodecSpi {
    fn transact(&self, data: u32, num_bits: usize, readback: bool) -> Result<u32> {
        self.spi
            .transact_as(Perif::Codec, &CODEC_SPI_CONFIG, data, num_bits, readback)
    }
}

#[cfg(test)]
mod tests {
    use std::sync::Mutex as StdMutex;

    use super::*;

    /// A settings bus that records pokes and answers peeks from a queue.
    #[derive(Debug, Default)]
    struct FakeBus {
        pokes: StdMutex<Vec<(u32, u32)>>,
        reads: StdMutex<Vec<u32>>,
    }

    impl FakeBus {
        fn with_reads(reads: &[u32]) -> Arc<Self> {
            Arc::new(Self {
                pokes: StdMutex::new(Vec::new()),
                reads: StdMutex::new(reads.iter().rev().copied().collect()),
            })
        }

        fn pokes(&self) -> Vec<(u32, u32)> {
            self.pokes.lock().unwrap().clone()
        }
    }

    impl RegisterIface for FakeBus {
        fn poke32(&self, addr: u32, data: u32) -> Result<()> {
            self.pokes.lock().unwrap().push((addr, data));
            Ok(())
        }

        fn peek32(&self, _addr: u32) -> Result<u32> {
            Ok(self.reads.lock().unwrap().pop().unwrap_or(0))
        }
    }

    /// A core at the offsets a major-7 build puts it at.
    fn core(bus: &Arc<FakeBus>) -> SpiCore {
        let regs = FpgaCompat::V7.shim().core_regs();
        SpiCore::new(bus.clone(), regs.sr_spi, regs.rb32_spi)
    }

    #[test]
    fn the_register_trio_sits_where_spi_core_3000_puts_it() {
        // base + 0/4/8, stride 4, which is what spi_core_3000::make passes.
        let offsets = SpiOffsets::new(32, 8);
        assert_eq!(offsets.divider, 32);
        assert_eq!(offsets.control, 36);
        assert_eq!(offsets.data, 40);
        assert_eq!(offsets.readback, 8);

        // And the base is TOREG(SR_CORE_SPI) = 8 * 4 on both supported majors.
        for compat in FpgaCompat::WINDOW {
            assert_eq!(compat.shim().core_regs().sr_spi, 32, "{compat}");
            assert_eq!(compat.shim().core_regs().rb32_spi, 8, "{compat}");
        }
    }

    #[test]
    fn a_transaction_writes_divider_control_then_data() {
        let bus = FakeBus::with_reads(&[0xa5]);
        let core = core(&bus);
        core.set_divider(divider_for(AD9361_SPI_RATE));

        let value = core
            .transact(AD9361_SLAVE, &CODEC_SPI_CONFIG, 0x0000_3700, 24, true)
            .unwrap();

        // 100 MHz / 1 MHz = 100 -> (100/2) - 0.5 = 49.5, truncated to 49, which
        // is 100e6 / (2 * 50) = 1 MHz exactly.
        // Control: slave 1, 24 bits at bit 24, mosi falling at bit 31, and bit
        // 30 clear because the codec samples on the falling edge too.
        // Data: left-justified, so 0x3700 becomes 0x3700 << 8.
        assert_eq!(
            bus.pokes(),
            vec![(32, 49), (36, 0x9800_0001), (40, 0x0037_0000)]
        );
        assert_eq!(value, 0xa5);
    }

    #[test]
    fn the_divider_and_control_words_are_only_written_when_they_change() {
        // The AD936x init sequence is hundreds of identical-shaped writes and
        // every poke is a USB round trip, so this is worth an assertion.
        let bus = FakeBus::with_reads(&[]);
        let core = core(&bus);
        core.set_divider(divider_for(AD9361_SPI_RATE));

        for value in 0..4u32 {
            core.transact(AD9361_SLAVE, &CODEC_SPI_CONFIG, value, 24, false)
                .unwrap();
        }

        let pokes = bus.pokes();
        assert_eq!(pokes.len(), 2 + 4, "one divider, one control, four data");
        assert!(pokes[2..].iter().all(|(addr, _)| *addr == 40));
    }

    #[test]
    fn changing_slave_rewrites_the_control_word() {
        let bus = FakeBus::with_reads(&[]);
        let core = core(&bus);

        core.transact(AD9361_SLAVE, &CODEC_SPI_CONFIG, 0, 24, false)
            .unwrap();
        core.transact(ADF4001_SLAVE, &CODEC_SPI_CONFIG, 0, 24, false)
            .unwrap();

        let controls: Vec<u32> = bus
            .pokes()
            .into_iter()
            .filter(|(addr, _)| *addr == 36)
            .map(|(_, data)| data)
            .collect();
        assert_eq!(controls, vec![0x9800_0001, 0x9800_0002]);
    }

    #[test]
    fn both_clock_edges_land_in_the_bits_spi_core_3000_reads() {
        let rise_fall = SpiConfig {
            mosi_edge: ClockEdge::Rise,
            miso_edge: ClockEdge::Rise,
            divider: None,
        };
        // MOSI rising clears bit 31; MISO rising sets bit 30.
        assert_eq!(control_word(1, &rise_fall, 24) >> 30, 0b01);
        // The codec's config is the other way round in both bits.
        assert_eq!(control_word(1, &CODEC_SPI_CONFIG, 24) >> 30, 0b10);
    }

    #[test]
    fn a_custom_divider_rounds_the_clock_down() {
        let bus = FakeBus::with_reads(&[]);
        let core = core(&bus);
        let slow = SpiConfig {
            divider: Some(101),
            ..CODEC_SPI_CONFIG
        };

        core.transact(AD9361_SLAVE, &slow, 0, 24, false).unwrap();

        // (101 - 1) / 2 = 50 -> 100e6 / (2 * 51), which is below the 1 MHz the
        // ratio names rather than above it.
        assert_eq!(bus.pokes()[0], (32, 50));
    }

    #[test]
    fn a_degenerate_width_is_refused_rather_than_shifted() {
        // `data << (32 - num_bits)` panics in debug and is nonsense in release
        // for num_bits == 0. UHD has no check here.
        let bus = FakeBus::with_reads(&[]);
        let core = core(&bus);
        assert!(core.transact(1, &CODEC_SPI_CONFIG, 0, 0, false).is_err());
        assert!(core.transact(1, &CODEC_SPI_CONFIG, 0, 33, false).is_err());
        assert!(bus.pokes().is_empty(), "nothing reached the bus");
    }

    #[test]
    fn a_full_width_transaction_is_not_shifted_out_of_the_register() {
        let bus = FakeBus::with_reads(&[]);
        let core = core(&bus);
        core.transact(1, &CODEC_SPI_CONFIG, 0xdead_beef, 32, false)
            .unwrap();
        assert_eq!(bus.pokes().last(), Some(&(40, 0xdead_beef)));
    }

    #[test]
    fn the_two_board_rates_encode_to_exact_dividers() {
        // 100e6 / (2 * (49 + 1)) == 1e6 and 100e6 / (2 * (4999 + 1)) == 10e3.
        assert_eq!(encode_divider(divider_for(AD9361_SPI_RATE)), 49);
        assert_eq!(encode_divider(divider_for(ADF4001_SPI_RATE)), 4999);
        assert_eq!(BUS_CLOCK_RATE / (2.0 * 50.0), AD9361_SPI_RATE);
        assert_eq!(BUS_CLOCK_RATE / (2.0 * 5000.0), ADF4001_SPI_RATE);
    }

    #[test]
    fn a_nonsense_ratio_does_not_produce_a_nonsense_divider() {
        for div in [0.0, -1.0, f64::NAN, f64::INFINITY] {
            assert_eq!(encode_divider(div), 0, "{div}");
        }
    }

    #[test]
    fn the_master_reclocks_when_the_peripheral_changes_and_not_otherwise() {
        let bus = FakeBus::with_reads(&[]);
        let spi = LocalSpi::new(bus.clone(), FpgaCompat::V16);

        spi.transact_as(Perif::Codec, &CODEC_SPI_CONFIG, 0, 24, false)
            .unwrap();
        spi.transact_as(Perif::Codec, &CODEC_SPI_CONFIG, 1, 24, false)
            .unwrap();
        spi.transact_as(Perif::Pll, &CODEC_SPI_CONFIG, 2, 24, false)
            .unwrap();
        spi.transact_as(Perif::Codec, &CODEC_SPI_CONFIG, 3, 24, false)
            .unwrap();

        let dividers: Vec<u32> = bus
            .pokes()
            .into_iter()
            .filter(|(addr, _)| *addr == 32)
            .map(|(_, data)| data)
            .collect();
        // Codec, then PLL, then back — the repeat in the middle costs nothing.
        assert_eq!(dividers, vec![49, 4999, 49]);
        assert_eq!(spi.perif(), Some(Perif::Codec));
    }

    #[test]
    fn a_codec_transaction_carries_the_codecs_slave_and_edges() {
        // The whole of what `ad936x` delegates: it supplies a word, and this
        // side supplies everything about the board.
        use ad936x::SpiInterface;

        let bus = FakeBus::with_reads(&[0x0000_000a]);
        let codec = CodecSpi::new(Arc::new(LocalSpi::new(bus.clone(), FpgaCompat::V7)));

        assert_eq!(codec.transact(0x0000_3700, 24, true).unwrap(), 0x0a);
        assert_eq!(
            bus.pokes(),
            vec![(32, 49), (36, 0x9800_0001), (40, 0x0037_0000)]
        );
    }

    #[test]
    fn the_slave_selects_are_one_hot_and_distinct() {
        // They are OR-ed into the control word's low 24 bits, so overlapping
        // would select both parts at once.
        assert_eq!(AD9361_SLAVE, 1);
        assert_eq!(ADF4001_SLAVE, 2);
        assert_eq!(AD9361_SLAVE & ADF4001_SLAVE, 0);
        assert_eq!(Perif::Codec.slave(), AD9361_SLAVE);
        assert_eq!(Perif::Pll.slave(), ADF4001_SLAVE);
    }
}
