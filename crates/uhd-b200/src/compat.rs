//! Which FPGA build this driver is talking to, and everything that changes
//! with it.
//!
//! A B2xx's FPGA image carries a compatibility number in the bottom settings-bus
//! register: a signature word, a major, and a minor. The major is the one that
//! matters — it is bumped whenever the host/FPGA register interface changes
//! incompatibly, which means it identifies *which register interface the host is
//! about to talk to*. Two majors are in the field and both are supported here:
//!
//! | Major | UHD constant | FPGA builds | Boards |
//! | ---: | --- | --- | --- |
//! | 7 | `B205_FPGA_COMPAT_NUM` | `usrp_b2xxmini_fpga.bin` family | B200mini, B205mini, B206mini |
//! | 16 | `B200_FPGA_COMPAT_NUM` | `usrp_b200_fpga.bin`, `usrp_b210_fpga.bin` | B200, B210 |
//!
//! # Why a trait rather than `if is_mini`
//!
//! `b200_impl.cpp` spreads the difference between the two builds across a dozen
//! `is_b2xx_mini(_product)` tests — in the constructor, in `update_clock_source`,
//! in `update_time_source`, in `update_bandsel`, in the codec client selection.
//! Each one is individually reasonable and collectively they are the reason that
//! adding a third FPGA generation means auditing the whole file.
//!
//! Here the difference is resolved **once, at open time**, into a
//! [`FpgaCompat`] and the [`FpgaShim`] behind it. Nothing below the open path
//! branches on the product or on a version number again; it asks the shim. The
//! practical test for whether a new behavioural difference belongs here is
//! whether a future major 17 build could plausibly want the other answer.
//!
//! Note that the discriminator is the FPGA build, not the board. The board only
//! enters through [`FpgaShim::accepts`], which says which majors are legal for a
//! given product, and through [`FpgaShim::frontends_swapped`], which is a
//! property of how the board is wired to the AD936x rather than of the
//! bitstream.
//!
//! # Register addresses
//!
//! Settings (`sr_*`) addresses are **byte** addresses — UHD's `TOREG(x)`, which
//! is `x * 4` — because that is what [`RegisterIface::poke32`] takes. Readback
//! (`rb32_*`) addresses are byte addresses too, and are already spelled that way
//! in `b200_regs.hpp`; they select a half of the 64-bit readback word the way
//! [`RegisterIface::peek32`] does.
//!
//! [`RegisterIface::poke32`]: uhd_transport::RegisterIface::poke32
//! [`RegisterIface::peek32`]: uhd_transport::RegisterIface::peek32

use std::fmt;

use uhd_transport::usb::B2xxProduct;

/// Compatibility major of the B2xxmini FPGA builds.
///
/// `B205_FPGA_COMPAT_NUM` in `b200_impl.hpp`, where the comment notes it applies
/// to every B2xxmini rather than only to the B205mini.
pub const MAJOR_MINI: u16 = 7;

/// Compatibility major of the full-size B200/B210 FPGA builds.
///
/// `B200_FPGA_COMPAT_NUM` in `b200_impl.hpp`.
pub const MAJOR_FULL: u16 = 16;

/// The signature word the compatibility register's top half must hold.
///
/// Without it there is no way to tell a genuine version number from the zeros
/// an unconfigured settings bus reads back as.
pub const FPGA_SIGNATURE: u32 = 0xace0_ba5e;

/// A supported FPGA compatibility major, and the behaviour that goes with it.
///
/// One variant per major this driver has a [`FpgaShim`] for. The set of variants
/// **is** the supported window: a device reporting a major with no variant here
/// is out of the window, and that is what the policy layer in
/// [`policy`](crate::policy) acts on.
///
/// Adding a variant is additive. Removing one is a breaking change for
/// `uhd-b200` — see the commitment recorded in `NOTES.md`.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
#[non_exhaustive]
pub enum FpgaCompat {
    /// Major 7: the B2xxmini builds. One radio chain, no GPSDO, no reference
    /// PLL, no band-select GPIO.
    V7,
    /// Major 16: the full-size B200/B210 builds. Up to two radio chains, a
    /// GPSDO slot, an ADF4001 reference PLL, band-select GPIO.
    V16,
}

impl FpgaCompat {
    /// Every major this build supports, ascending.
    ///
    /// This is the window quoted in diagnostics, and the list CI is required to
    /// exercise.
    pub const WINDOW: [Self; 2] = [Self::V7, Self::V16];

    /// The compatibility major this variant stands for.
    #[must_use]
    pub const fn major(self) -> u16 {
        match self {
            Self::V7 => MAJOR_MINI,
            Self::V16 => MAJOR_FULL,
        }
    }

    /// The variant for a major, or `None` if it is outside the window.
    #[must_use]
    pub const fn from_major(major: u16) -> Option<Self> {
        match major {
            MAJOR_MINI => Some(Self::V7),
            MAJOR_FULL => Some(Self::V16),
            _ => None,
        }
    }

    /// The major a given board's FPGA image must report.
    ///
    /// `check_fpga_compat`'s `is_b2xx_mini(_product) ? B205 : B200` in one
    /// place. A board can only ever be running one of the two builds, so the
    /// per-device window is a single major even though the driver's window is
    /// two.
    #[must_use]
    pub fn for_product(product: B2xxProduct) -> Self {
        if product.is_mini() {
            Self::V7
        } else {
            Self::V16
        }
    }

    /// The behaviour shim for this major.
    ///
    /// Selected once at open time and then handed around; see the
    /// [module docs](self).
    #[must_use]
    pub fn shim(self) -> &'static dyn FpgaShim {
        match self {
            Self::V7 => &MINI_SHIM,
            Self::V16 => &FULL_SHIM,
        }
    }

    /// Whether this FPGA build is a legal one for `product`.
    #[must_use]
    pub fn accepts(self, product: B2xxProduct) -> bool {
        self == Self::for_product(product)
    }
}

impl fmt::Display for FpgaCompat {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "major {}", self.major())
    }
}

/// Settings-bus offsets of the local (non-radio) core.
///
/// Names and values come from `b200_regs.hpp`; see the [module docs](self) for
/// the byte-address convention.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[non_exhaustive]
pub struct CoreRegs {
    /// `SR_CORE_SPI`, the local SPI core's command register.
    pub sr_spi: u32,
    /// `SR_CORE_MISC`, which carries the board's GPIO state word.
    pub sr_misc: u32,
    /// `SR_CORE_COMPAT`.
    pub sr_compat: u32,
    /// `SR_CORE_SYNC`, which selects the time source.
    pub sr_sync: u32,
    /// `SR_CORE_GPSDO_ST`, or `None` on a build with no GPSDO core behind it.
    pub sr_gpsdo_status: Option<u32>,
    /// `RB32_CORE_SPI`.
    pub rb32_spi: u32,
    /// `RB32_CORE_MISC`.
    pub rb32_misc: u32,
    /// `RB32_CORE_STATUS`. The low byte is the GPSDO status, the next byte up
    /// is the number of radio chains the build was compiled with.
    pub rb32_status: u32,
    /// `RB32_CORE_PLL`, the reference-lock status.
    pub rb32_pll: u32,
}

/// Settings-bus offsets within one radio chain.
///
/// Identical for both majors currently in the window. They are reached through
/// the shim anyway, because "identical today" is a fact about these two builds
/// rather than a guarantee about the next one, and the alternative is a
/// constant that a future shim cannot override.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[non_exhaustive]
pub struct RadioRegs {
    /// `SR_TEST`, the scratch register the loopback self-test writes.
    pub sr_test: u32,
    /// `RB32_TEST`, where it reads back.
    pub rb32_test: u32,
    /// `SR_READBACK`, as a **register index** rather than a byte address: it is
    /// the address selector for a readback, not a poke target.
    pub sr_readback: u32,
}

/// How a build locks to an external 10 MHz reference.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
#[non_exhaustive]
pub enum RefLock {
    /// The `ref_sel` bit of the misc GPIO word gates locking directly. This is
    /// the mini behaviour, and it is why `update_clock_source` has a special
    /// case that re-reads the lock sensor for a B205.
    GpioRefSel,
    /// An ADF4001 on SPI slave 1 does the locking. Full-size boards only —
    /// `_adf4001_iface` is left null on a mini.
    Adf4001,
}

/// The behaviour that varies between FPGA compatibility majors.
///
/// Implementors are zero-sized and `'static`: a shim is a table of answers, not
/// state. Reach one through [`FpgaCompat::shim`].
pub trait FpgaShim: fmt::Debug + Send + Sync {
    /// Which major this shim implements.
    fn compat(&self) -> FpgaCompat;

    /// Whether this build is legal on `product`.
    fn accepts(&self, product: B2xxProduct) -> bool {
        self.compat().accepts(product)
    }

    /// The name Ettus gives this generation of board, which is what UHD puts in
    /// `/mboards/0/codename`.
    fn codename(&self) -> &'static str;

    /// Local-core register offsets.
    fn core_regs(&self) -> &'static CoreRegs;

    /// Per-radio register offsets.
    fn radio_regs(&self) -> &'static RadioRegs;

    /// The control and response stream IDs for a radio chain, or `None` if this
    /// build has no such chain.
    fn radio_sids(&self, radio: usize) -> Option<(u32, u32)>;

    /// The largest number of radio chains this build can have been compiled
    /// with.
    ///
    /// An upper bound, not the answer: the actual count is the second byte of
    /// [`CoreRegs::rb32_status`], because a B200 and a B210 share the major-16
    /// build family and report one and two respectively. The bound is what
    /// makes `num_radio_chains > 2` a corrupt readback rather than a surprise.
    fn max_radio_chains(&self) -> usize;

    /// Whether this build has a GPSDO core, a GPSDO UART on the control
    /// endpoint, and `gpsdo` among its clock and time sources.
    fn has_gpsdo(&self) -> bool;

    /// Whether this build drives the RX/TX band-select GPIO.
    ///
    /// False on the minis, whose front end has no band-select filters —
    /// `update_bandsel` returns immediately for them.
    fn has_bandsel(&self) -> bool;

    /// How locking to an external reference is done.
    fn ref_lock(&self) -> RefLock;

    /// Clock sources this build offers, in UHD's order.
    fn clock_sources(&self) -> &'static [&'static str];

    /// Time sources this build offers, in UHD's order.
    fn time_sources(&self) -> &'static [&'static str];

    /// Prefix of the power-calibration key for boards running this build.
    fn power_cal_prefix(&self) -> &'static str;

    /// Whether radio 0 maps to the AD936x's second frontend.
    ///
    /// A board property rather than a bitstream one, but it is asked at exactly
    /// the moment the shim is chosen and there is nowhere better for it. On a
    /// B210 the radios are swapped relative to the AD9361's frontends; on a
    /// B200 that is true only up to board revision 4, because revision 5
    /// switched to an AD9364, which has no second frontend; on a mini it is
    /// never true.
    ///
    /// `board_revision` is [`MbEeprom::revision`], and `None` — an unprogrammed
    /// revision field — is treated as an early board, which is UHD's behaviour:
    /// `_revision` is left at its default when the EEPROM field is empty.
    ///
    /// [`MbEeprom::revision`]: uhd_transport::usb::MbEeprom::revision
    fn frontends_swapped(&self, product: B2xxProduct, board_revision: Option<u16>) -> bool;
}

/// The major-7 shim: B2xxmini builds.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct MiniShim;

/// The major-16 shim: full-size B200/B210 builds.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct FullShim;

static MINI_SHIM: MiniShim = MiniShim;
static FULL_SHIM: FullShim = FullShim;

/// `SR_CORE_*` and `RB32_CORE_*` from `b200_regs.hpp`, minus the GPSDO status
/// register, which the mini builds have no core behind.
static MINI_CORE_REGS: CoreRegs = CoreRegs {
    sr_spi: 8 * 4,
    sr_misc: 16 * 4,
    sr_compat: 24 * 4,
    sr_sync: 48 * 4,
    sr_gpsdo_status: None,
    rb32_spi: 8,
    rb32_misc: 16,
    rb32_status: 20,
    rb32_pll: 24,
};

/// The same table with `SR_CORE_GPSDO_ST` present.
static FULL_CORE_REGS: CoreRegs = CoreRegs {
    sr_gpsdo_status: Some(40 * 4),
    ..MINI_CORE_REGS
};

/// `SR_TEST`, `RB32_TEST` and `SR_READBACK`, which both majors agree on.
static RADIO_REGS: RadioRegs = RadioRegs {
    sr_test: 21 * 4,
    rb32_test: 0,
    sr_readback: 32,
};

/// Control and response stream IDs for radio chain 0.
///
/// `B200_CTRL0_MSG_SID` and its response in `b200_impl.hpp`.
const RADIO0_SIDS: (u32, u32) = (0x0000_0010, 0x0010_0000);

/// The same for radio chain 1, which only a two-chain build has.
const RADIO1_SIDS: (u32, u32) = (0x0000_0020, 0x0020_0000);

impl FpgaShim for MiniShim {
    fn compat(&self) -> FpgaCompat {
        FpgaCompat::V7
    }
    fn codename(&self) -> &'static str {
        "Pixie"
    }
    fn core_regs(&self) -> &'static CoreRegs {
        &MINI_CORE_REGS
    }
    fn radio_regs(&self) -> &'static RadioRegs {
        &RADIO_REGS
    }
    fn radio_sids(&self, radio: usize) -> Option<(u32, u32)> {
        (radio == 0).then_some(RADIO0_SIDS)
    }
    fn max_radio_chains(&self) -> usize {
        1
    }
    fn has_gpsdo(&self) -> bool {
        false
    }
    fn has_bandsel(&self) -> bool {
        false
    }
    fn ref_lock(&self) -> RefLock {
        RefLock::GpioRefSel
    }
    fn clock_sources(&self) -> &'static [&'static str] {
        &["internal", "external"]
    }
    fn time_sources(&self) -> &'static [&'static str] {
        &["none", "internal", "external"]
    }
    fn power_cal_prefix(&self) -> &'static str {
        "b2xxmini_pwr_"
    }
    fn frontends_swapped(&self, _product: B2xxProduct, _board_revision: Option<u16>) -> bool {
        false
    }
}

impl FpgaShim for FullShim {
    fn compat(&self) -> FpgaCompat {
        FpgaCompat::V16
    }
    fn codename(&self) -> &'static str {
        "Sasquatch"
    }
    fn core_regs(&self) -> &'static CoreRegs {
        &FULL_CORE_REGS
    }
    fn radio_regs(&self) -> &'static RadioRegs {
        &RADIO_REGS
    }
    fn radio_sids(&self, radio: usize) -> Option<(u32, u32)> {
        match radio {
            0 => Some(RADIO0_SIDS),
            1 => Some(RADIO1_SIDS),
            _ => None,
        }
    }
    fn max_radio_chains(&self) -> usize {
        2
    }
    fn has_gpsdo(&self) -> bool {
        true
    }
    fn has_bandsel(&self) -> bool {
        true
    }
    fn ref_lock(&self) -> RefLock {
        RefLock::Adf4001
    }
    fn clock_sources(&self) -> &'static [&'static str] {
        &["internal", "external", "gpsdo"]
    }
    fn time_sources(&self) -> &'static [&'static str] {
        &["none", "internal", "external", "gpsdo"]
    }
    fn power_cal_prefix(&self) -> &'static str {
        "b2xx_pwr_"
    }
    fn frontends_swapped(&self, product: B2xxProduct, board_revision: Option<u16>) -> bool {
        // A B200 at revision 5 or later carries an AD9364, which has no second
        // frontend, so radio 0 has to map to the first one. Everything else in
        // this build family is swapped.
        !(product == B2xxProduct::B200 && board_revision.unwrap_or(0) >= 5)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn the_window_matches_the_reference_constants() {
        // B205_FPGA_COMPAT_NUM / B200_FPGA_COMPAT_NUM in b200_impl.hpp, UHD
        // v4.10.0.0, and the signature check_fpga_compat asserts.
        assert_eq!(MAJOR_MINI, 7);
        assert_eq!(MAJOR_FULL, 16);
        assert_eq!(FPGA_SIGNATURE, 0xace0_ba5e);

        assert_eq!(FpgaCompat::WINDOW, [FpgaCompat::V7, FpgaCompat::V16]);
        for compat in FpgaCompat::WINDOW {
            assert_eq!(FpgaCompat::from_major(compat.major()), Some(compat));
            assert_eq!(compat.shim().compat(), compat);
        }
    }

    #[test]
    fn majors_outside_the_window_have_no_variant() {
        // The whole point: an unknown major does not silently pick a shim.
        for major in [0, 1, 6, 8, 15, 17, u16::MAX] {
            assert_eq!(FpgaCompat::from_major(major), None, "{major}");
        }
    }

    #[test]
    fn every_product_resolves_to_exactly_one_supported_major() {
        // This is is_b2xx_mini's whole role in check_fpga_compat, and the
        // reason the per-device window is one major wide.
        for product in B2xxProduct::ALL {
            let expected = if product.is_mini() {
                FpgaCompat::V7
            } else {
                FpgaCompat::V16
            };
            assert_eq!(FpgaCompat::for_product(product), expected, "{product}");
            assert!(expected.accepts(product));

            let wrong = FpgaCompat::WINDOW
                .into_iter()
                .find(|c| *c != expected)
                .expect("the window has more than one major");
            assert!(!wrong.accepts(product), "{product} must reject {wrong}");
            assert!(!wrong.shim().accepts(product), "{product} via the shim");
        }
    }

    #[test]
    fn the_shims_differ_where_b200_impl_branches_on_is_b2xx_mini() {
        // Each assertion below stands in for one `is_b2xx_mini(_product)` test
        // in b200_impl.cpp. If a shim ever stops differing here, the branch it
        // replaced has been lost rather than unified.
        let mini = FpgaCompat::V7.shim();
        let full = FpgaCompat::V16.shim();

        assert!(!mini.has_gpsdo() && full.has_gpsdo());
        assert!(!mini.has_bandsel() && full.has_bandsel());
        assert_eq!(mini.ref_lock(), RefLock::GpioRefSel);
        assert_eq!(full.ref_lock(), RefLock::Adf4001);
        assert_eq!(mini.codename(), "Pixie");
        assert_eq!(full.codename(), "Sasquatch");
        assert_eq!(mini.power_cal_prefix(), "b2xxmini_pwr_");
        assert_eq!(full.power_cal_prefix(), "b2xx_pwr_");
        assert_eq!(mini.max_radio_chains(), 1);
        assert_eq!(full.max_radio_chains(), 2);

        // `gpsdo` appears in both source lists exactly when the core does.
        assert!(!mini.clock_sources().contains(&"gpsdo"));
        assert!(full.clock_sources().contains(&"gpsdo"));
        assert!(!mini.time_sources().contains(&"gpsdo"));
        assert!(full.time_sources().contains(&"gpsdo"));

        // And the register table differs in exactly one entry.
        assert_eq!(mini.core_regs().sr_gpsdo_status, None);
        assert_eq!(full.core_regs().sr_gpsdo_status, Some(160));
        assert_eq!(mini.core_regs().rb32_status, full.core_regs().rb32_status);
    }

    #[test]
    fn register_offsets_match_b200_regs_hpp() {
        // TOREG(x) is x*4 for settings registers; readback constants are
        // already byte addresses.
        let regs = FpgaCompat::V16.shim().core_regs();
        assert_eq!(regs.sr_spi, 32); // SR_CORE_SPI      = 8
        assert_eq!(regs.sr_misc, 64); // SR_CORE_MISC     = 16
        assert_eq!(regs.sr_compat, 96); // SR_CORE_COMPAT   = 24
        assert_eq!(regs.sr_gpsdo_status, Some(160)); // SR_CORE_GPSDO_ST = 40
        assert_eq!(regs.sr_sync, 192); // SR_CORE_SYNC     = 48
        assert_eq!(regs.rb32_spi, 8);
        assert_eq!(regs.rb32_misc, 16);
        assert_eq!(regs.rb32_status, 20);
        assert_eq!(regs.rb32_pll, 24);

        let radio = FpgaCompat::V7.shim().radio_regs();
        assert_eq!(radio.sr_test, 84); // SR_TEST      = 21
        assert_eq!(radio.rb32_test, 0); // RB32_TEST    = 0
        assert_eq!(radio.sr_readback, 32); // SR_READBACK  = 32, an index
    }

    #[test]
    fn only_the_two_chain_build_has_a_second_radio() {
        assert_eq!(FpgaCompat::V7.shim().radio_sids(0), Some(RADIO0_SIDS));
        assert_eq!(FpgaCompat::V7.shim().radio_sids(1), None);
        assert_eq!(FpgaCompat::V16.shim().radio_sids(1), Some(RADIO1_SIDS));
        assert_eq!(FpgaCompat::V16.shim().radio_sids(2), None);
    }

    #[test]
    fn frontend_swapping_follows_the_board_not_the_bitstream() {
        let mini = FpgaCompat::V7.shim();
        let full = FpgaCompat::V16.shim();

        // A B210 is always swapped: FE1 is the B side.
        assert!(full.frontends_swapped(B2xxProduct::B210, Some(9)));
        // A B200 stops being swapped at revision 5, where the AD9364 arrives.
        assert!(full.frontends_swapped(B2xxProduct::B200, Some(4)));
        assert!(!full.frontends_swapped(B2xxProduct::B200, Some(5)));
        // An unprogrammed revision field reads as an early board, which is what
        // b200_impl does by leaving _revision at its default.
        assert!(full.frontends_swapped(B2xxProduct::B200, None));
        // A mini never swaps.
        assert!(!mini.frontends_swapped(B2xxProduct::B205Mini, Some(2)));
    }

    #[test]
    fn a_compat_renders_as_the_major_it_is() {
        assert_eq!(FpgaCompat::V7.to_string(), "major 7");
        assert_eq!(FpgaCompat::V16.to_string(), "major 16");
    }
}
