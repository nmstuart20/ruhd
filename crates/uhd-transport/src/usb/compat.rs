//! What the device says it is, gathered in one place for someone else to
//! judge.
//!
//! A B2xx carries three independent version numbers that all have to line up
//! before it will work: the FX3 firmware's compatibility number, the FPGA
//! build's, and the EEPROM layout's. UHD checks all three, and throws on any
//! mismatch, from three different places inside device construction —
//! `b200_impl::check_fw_compat`, `b200_impl::check_fpga_compat` and
//! `get_mb_eeprom`.
//!
//! # The divergence
//!
//! **This does not decide anything.** [`CompatReport`] gathers the numbers, says
//! what this driver was written against, and stops. Whether a given mismatch is
//! fatal, a warning, or fine is a policy question, and it is the caller's:
//!
//! - A minor-version FPGA mismatch is usually harmless and UHD treats it as
//!   such, but a *major* mismatch is fatal there with no override — so a
//!   developer with a locally built FPGA image cannot use it without patching
//!   and rebuilding the host code.
//! - The compatibility numbers are the only diagnostic when a device does not
//!   come up, and throwing from inside the constructor is precisely the moment
//!   they become unreadable.
//! - A tool whose job is to *report* what is attached must be able to describe
//!   a device it would refuse to open.
//!
//! So [`CompatReport::problems`] returns the complaints as text and
//! [`CompatReport::is_compatible`] summarises them, and neither is called
//! automatically. Compatibility negotiation — deciding what to do about a
//! mismatch, including reloading a matching image — is a layer that will sit on
//! top of this.
//!
//! # What it costs to gather
//!
//! Everything except the FPGA numbers is a control transfer to the FX3 and is
//! always available once firmware is running. The FPGA compatibility register
//! lives on the settings bus, which means an unconfigured FPGA has no answer
//! and [`FpgaCompat::Unconfigured`] is the honest report rather than an error.

use tracing::debug;
use uhd_core::Result;

use crate::endpoint::UsbEndpoint;
use crate::register::RegisterIface;

use super::Fx3State;
use super::ctrl::LocalCtrl;
use super::eeprom::MbEeprom;
use super::fx3::{B200_FW_COMPAT, FirmwareVersion, Fx3Iface};
use super::ids::B2xxProduct;
use super::probe::UsbProbe;

/// FPGA compatibility number this driver is written against, for the full-size
/// boards. `B200_FPGA_COMPAT_NUM` in `b200_impl.hpp`.
pub const B200_FPGA_COMPAT: u16 = 16;

/// FPGA compatibility number for the mini boards, which have their own FPGA
/// build and their own numbering. `B205_FPGA_COMPAT_NUM` in `b200_impl.hpp`,
/// where the comment notes it applies to every B2xxmini.
pub const B205_FPGA_COMPAT: u16 = 7;

/// Settings-bus address of the FPGA's compatibility register.
///
/// `check_fpga_compat` reads it as `_local_ctrl->peek64(0)`.
pub const FPGA_COMPAT_ADDR: u32 = 0;

/// Signature the top half of the compatibility register must hold.
///
/// A readback of zeros or of noise is what an unconfigured or misbehaving FPGA
/// gives you, and without this there is no way to tell that from a genuine
/// version number.
pub const FPGA_SIGNATURE: u32 = 0xace0_ba5e;

/// Which FPGA compatibility number a given product is expected to report.
#[must_use]
pub fn expected_fpga_compat(product: B2xxProduct) -> u16 {
    if product.is_mini() {
        B205_FPGA_COMPAT
    } else {
        B200_FPGA_COMPAT
    }
}

/// The FPGA build's compatibility number.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct FpgaVersion {
    /// The signature word, which should be [`FPGA_SIGNATURE`].
    pub signature: u32,
    /// Major number. Bumped when the host/FPGA register interface changes
    /// incompatibly; this is the one UHD compares.
    pub major: u16,
    /// Minor number. Informational.
    pub minor: u16,
}

impl FpgaVersion {
    /// Splits the 64-bit compatibility register.
    #[must_use]
    pub fn from_register(value: u64) -> Self {
        Self {
            signature: (value >> 32) as u32,
            major: (value >> 16) as u16,
            minor: value as u16,
        }
    }

    /// Whether the register held the signature, i.e. whether the numbers below
    /// it mean anything.
    #[must_use]
    pub fn is_signed(self) -> bool {
        self.signature == FPGA_SIGNATURE
    }
}

impl std::fmt::Display for FpgaVersion {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "{}.{}", self.major, self.minor)
    }
}

/// Whether the FPGA's compatibility number could be read, and if not, why not.
#[derive(Debug, Clone, PartialEq, Eq)]
#[non_exhaustive]
pub enum FpgaCompat {
    /// Read from the settings bus.
    Known(FpgaVersion),
    /// The FPGA has no image loaded, so there is no settings bus to ask. Not a
    /// failure — it is the state a B2xx is in from the moment firmware starts
    /// until a bitstream is pushed.
    Unconfigured,
    /// The FPGA should have answered and did not. Carries the reason, which is
    /// usually another process holding the bulk interfaces.
    Unavailable(String),
}

impl FpgaCompat {
    /// The version, when there is one.
    #[must_use]
    pub fn version(&self) -> Option<FpgaVersion> {
        match self {
            Self::Known(version) => Some(*version),
            _ => None,
        }
    }
}

impl std::fmt::Display for FpgaCompat {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::Known(version) => write!(f, "{version}"),
            Self::Unconfigured => f.write_str("<no image loaded>"),
            Self::Unavailable(why) => write!(f, "<unavailable: {why}>"),
        }
    }
}

/// Everything a device says about its own versions, and what this driver
/// expected.
///
/// Both halves are kept: a report that only said "mismatch" would be useless
/// for the thing these numbers are actually for, which is working out which of
/// the images on disk is the wrong one.
#[derive(Debug, Clone)]
#[non_exhaustive]
pub struct CompatReport {
    /// Which device this describes.
    pub endpoint: UsbEndpoint,
    /// The product, from USB or from the EEPROM. `None` when neither
    /// identified it.
    pub product: Option<B2xxProduct>,

    /// The firmware compatibility number the FX3 reports.
    pub firmware: FirmwareVersion,
    /// What this driver was written against, [`B200_FW_COMPAT`].
    pub expected_firmware: FirmwareVersion,

    /// The FPGA compatibility number, when the FPGA can be asked.
    pub fpga: FpgaCompat,
    /// The FPGA major number this driver expects for [`product`](Self::product),
    /// which differs between the mini and full-size boards. `None` when the
    /// product is unknown, since the expectation depends on it.
    pub expected_fpga_major: Option<u16>,

    /// The motherboard EEPROM, including its own layout revision and the board
    /// revision. See [`MbEeprom::problems`].
    pub eeprom: MbEeprom,

    /// The FX3 state machine's state, which is how
    /// [`fpga`](Self::fpga) came to be [`Unconfigured`](FpgaCompat::Unconfigured).
    pub fx3_state: Fx3State,
    /// The negotiated USB speed, `2` or `3`. Not a compatibility number, but
    /// it changes the maximum sample rate and belongs with the rest of what a
    /// device reports about itself.
    pub usb_speed: u8,
}

impl CompatReport {
    /// Whether the firmware's major number is the expected one.
    ///
    /// Only the major number counts, which is `check_fw_compat`'s rule.
    #[must_use]
    pub fn firmware_ok(&self) -> bool {
        self.firmware.major == self.expected_firmware.major
    }

    /// Whether the FPGA's major number is the expected one.
    ///
    /// `None` when there is nothing to compare: no image loaded, no answer, or
    /// an unknown product so no expectation. That is deliberately distinct from
    /// `Some(false)` — "cannot say" and "wrong" call for different responses.
    #[must_use]
    pub fn fpga_ok(&self) -> Option<bool> {
        let version = self.fpga.version()?;
        let expected = self.expected_fpga_major?;
        Some(version.is_signed() && version.major == expected)
    }

    /// Everything about this device that a strict reading would reject.
    ///
    /// The union of what UHD's three checks would have thrown on, phrased for
    /// someone who has to fix it. Empty means every number lined up.
    #[must_use]
    pub fn problems(&self) -> Vec<String> {
        let mut problems = Vec::new();

        if !self.firmware_ok() {
            problems.push(format!(
                "firmware reports compatibility number {}, but this driver expects {} \
                 (only the major number has to match)",
                self.firmware, self.expected_firmware
            ));
        }

        match (&self.fpga, self.expected_fpga_major) {
            (FpgaCompat::Known(version), _) if !version.is_signed() => problems.push(format!(
                "the fpga compatibility register holds signature {:#010x}, not \
                 {FPGA_SIGNATURE:#010x}; the readback is not a version number",
                version.signature
            )),
            (FpgaCompat::Known(version), Some(expected)) if version.major != expected => {
                problems.push(format!(
                    "the fpga reports compatibility number {version}, but a {} needs \
                     major {expected}",
                    self.product
                        .map_or("device of this kind", B2xxProduct::name)
                ));
            }
            (FpgaCompat::Unavailable(why), _) => {
                problems.push(format!(
                    "the fpga compatibility number could not be read: {why}"
                ));
            }
            _ => {}
        }

        problems.extend(self.eeprom.problems());
        problems
    }

    /// Whether a strict reading would accept this device.
    ///
    /// Shorthand for `problems().is_empty()`. Note that a device with no FPGA
    /// image loaded passes: not being configured yet is not an incompatibility,
    /// and the caller that cares is the one about to load one.
    #[must_use]
    pub fn is_compatible(&self) -> bool {
        self.problems().is_empty()
    }

    /// Gathers everything a running device will tell you about itself.
    ///
    /// Reads the firmware compatibility number, the USB speed, the FX3 state
    /// and the EEPROM over endpoint zero, then — if the FX3 says the FPGA is
    /// configured — claims the bulk interfaces and reads the FPGA
    /// compatibility register.
    ///
    /// The FPGA read failing is recorded in
    /// [`FpgaCompat::Unavailable`] rather than raised, because everything else
    /// in the report is still true and still worth having. The transfers
    /// against the FX3 are not treated that way: if those fail the device is
    /// not answering at all, and there is nothing to report.
    ///
    /// # Errors
    ///
    /// [`Error::Usb`](uhd_core::Error::Usb) if a control transfer to the FX3
    /// fails. Firmware must already be running — see
    /// [`initialize`](super::initialize).
    pub fn read(iface: &Fx3Iface, probe: &UsbProbe) -> Result<Self> {
        let firmware = iface.firmware_version()?;
        let usb_speed = iface.usb_speed()?;
        let fx3_state = iface.fx3_state()?;
        let eeprom = super::eeprom::read(iface)?;

        // USB pins the product down for every board but the B200/B210 pair,
        // and the EEPROM settles those — the same order `get_b200_product`
        // resolves them in.
        let product = probe.product.or_else(|| eeprom.product());

        let fpga = read_fpga_compat(iface, fx3_state);
        debug!(
            %firmware,
            usb_speed,
            state = %fx3_state,
            fpga = %fpga,
            "gathered compatibility report"
        );

        Ok(Self {
            endpoint: probe.endpoint.clone(),
            product,
            firmware,
            expected_firmware: B200_FW_COMPAT,
            fpga,
            expected_fpga_major: product.map(expected_fpga_compat),
            eeprom,
            fx3_state,
            usb_speed,
        })
    }
}

/// Reads the FPGA compatibility register, if there is an FPGA to read it from.
///
/// The FX3 state is the gate. [`Unconfigured`](Fx3State::Unconfigured) means no
/// bitstream has been pushed since power-up, and asking anyway costs the full
/// two-second ctrl timeout to learn what the previous request already said.
fn read_fpga_compat(iface: &Fx3Iface, state: Fx3State) -> FpgaCompat {
    if matches!(state, Fx3State::Unconfigured | Fx3State::Undefined) {
        return FpgaCompat::Unconfigured;
    }

    let ctrl = match LocalCtrl::open(iface) {
        Ok(ctrl) => ctrl,
        Err(e) => return FpgaCompat::Unavailable(e.to_string()),
    };
    match ctrl.peek64(FPGA_COMPAT_ADDR) {
        Ok(value) => FpgaCompat::Known(FpgaVersion::from_register(value)),
        Err(e) => FpgaCompat::Unavailable(e.to_string()),
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::usb::eeprom::{EepromLayout, REV1_SIGNATURE};

    fn eeprom() -> MbEeprom {
        let mut raw = vec![0u8; 46];
        raw[0..2].copy_from_slice(&0xb200u16.to_le_bytes());
        raw[2..4].copy_from_slice(&1u16.to_le_bytes());
        raw[4..6].copy_from_slice(&1u16.to_le_bytes());
        raw[6..8].copy_from_slice(&0x2500u16.to_le_bytes());
        raw[8..10].copy_from_slice(&0x0022u16.to_le_bytes());
        raw[10..12].copy_from_slice(&2u16.to_le_bytes());
        raw[12..14].copy_from_slice(&0x0004u16.to_le_bytes());
        raw[14..19].copy_from_slice(b"bench");
        raw[37..44].copy_from_slice(b"3168B41");
        crate::usb::eeprom::decode(REV1_SIGNATURE, EepromLayout::Rev1, raw)
    }

    fn report(firmware: FirmwareVersion, fpga: FpgaCompat) -> CompatReport {
        CompatReport {
            endpoint: UsbEndpoint::default(),
            product: Some(B2xxProduct::B205Mini),
            firmware,
            expected_firmware: B200_FW_COMPAT,
            fpga,
            expected_fpga_major: Some(expected_fpga_compat(B2xxProduct::B205Mini)),
            eeprom: eeprom(),
            fx3_state: Fx3State::Running,
            usb_speed: 3,
        }
    }

    fn good_fpga() -> FpgaCompat {
        FpgaCompat::Known(FpgaVersion {
            signature: FPGA_SIGNATURE,
            major: B205_FPGA_COMPAT,
            minor: 0,
        })
    }

    #[test]
    fn a_matching_device_has_nothing_to_complain_about() {
        let report = report(B200_FW_COMPAT, good_fpga());
        assert!(report.is_compatible(), "{:?}", report.problems());
        assert!(report.firmware_ok());
        assert_eq!(report.fpga_ok(), Some(true));
    }

    #[test]
    fn the_compat_numbers_match_the_reference() {
        // B200_FPGA_COMPAT_NUM / B205_FPGA_COMPAT_NUM in b200_impl.hpp,
        // UHD v4.10.0.0, and the signature check_fpga_compat asserts.
        assert_eq!(B200_FPGA_COMPAT, 16);
        assert_eq!(B205_FPGA_COMPAT, 7);
        assert_eq!(FPGA_SIGNATURE, 0xace0_ba5e);

        // Every mini shares the B205 number; the full-size boards share the
        // other. This is the whole of is_b2xx_mini's role in check_fpga_compat.
        for product in B2xxProduct::ALL {
            let expected = if product.is_mini() {
                B205_FPGA_COMPAT
            } else {
                B200_FPGA_COMPAT
            };
            assert_eq!(expected_fpga_compat(product), expected, "{product}");
        }
    }

    #[test]
    fn the_compat_register_splits_into_signature_major_minor() {
        let version = FpgaVersion::from_register(0xace0_ba5e_0007_0003);
        assert!(version.is_signed());
        assert_eq!(version.major, 7);
        assert_eq!(version.minor, 3);
        assert_eq!(version.to_string(), "7.3");
    }

    #[test]
    fn a_mismatch_is_described_rather_than_raised() {
        // This is the divergence in one test: UHD throws from inside device
        // construction, and here the numbers survive to be printed.
        let report = report(
            FirmwareVersion { major: 7, minor: 0 },
            FpgaCompat::Known(FpgaVersion {
                signature: FPGA_SIGNATURE,
                major: 16,
                minor: 0,
            }),
        );
        assert!(!report.is_compatible());
        assert!(!report.firmware_ok());
        assert_eq!(report.fpga_ok(), Some(false));

        let problems = report.problems();
        assert_eq!(problems.len(), 2, "{problems:?}");
        assert!(problems[0].contains("7.0"), "{problems:?}");
        assert!(problems[1].contains("B205mini"), "{problems:?}");

        // And the numbers themselves are still there to act on.
        assert_eq!(report.firmware.major, 7);
        assert_eq!(report.fpga.version().map(|v| v.major), Some(16));
    }

    #[test]
    fn a_newer_minor_firmware_is_fine() {
        let report = report(FirmwareVersion { major: 8, minor: 9 }, good_fpga());
        assert!(report.firmware_ok());
        assert!(report.is_compatible(), "{:?}", report.problems());
    }

    #[test]
    fn an_unconfigured_fpga_is_not_an_incompatibility() {
        // The state every B2xx is in between firmware starting and a bitstream
        // being pushed. Reporting it as a mismatch would make `ruhd probe`
        // shout at a perfectly healthy device.
        let mut report = report(B200_FW_COMPAT, FpgaCompat::Unconfigured);
        report.fx3_state = Fx3State::Unconfigured;
        assert_eq!(report.fpga_ok(), None);
        assert!(report.is_compatible(), "{:?}", report.problems());
    }

    #[test]
    fn an_unreadable_fpga_is_a_problem_but_a_different_one() {
        let report = report(
            B200_FW_COMPAT,
            FpgaCompat::Unavailable("device busy".into()),
        );
        assert_eq!(report.fpga_ok(), None);
        assert!(!report.is_compatible());
        assert!(report.problems()[0].contains("device busy"));
    }

    #[test]
    fn an_unsigned_readback_is_not_read_as_a_version() {
        // Zeros are what a settings bus that is not there gives you, and
        // reporting that as "version 0.0" would be worse than useless.
        let report = report(
            B200_FW_COMPAT,
            FpgaCompat::Known(FpgaVersion::from_register(0)),
        );
        assert_eq!(report.fpga_ok(), Some(false));
        assert!(
            report.problems()[0].contains("signature"),
            "{:?}",
            report.problems()
        );
    }

    #[test]
    fn eeprom_problems_reach_the_report() {
        let mut report = report(B200_FW_COMPAT, good_fpga());
        report.eeprom = MbEeprom {
            signature: 0,
            ..MbEeprom::default()
        };
        assert!(!report.is_compatible());
        assert!(
            report.problems()[0].contains("not been programmed"),
            "{:?}",
            report.problems()
        );
    }
}
