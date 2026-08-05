//! What to do about a device whose FPGA build is not one this driver supports.
//!
//! [`uhd_transport::usb::CompatReport`] gathers a device's version numbers and
//! deliberately judges none of them; this is the layer that judges. It is kept
//! separate from the transport for the reasons in `NOTES.md` — a tool that
//! reports on hardware has to be able to describe hardware it would refuse to
//! open — and separate from [`session`](crate::session) so that the decision can
//! be unit-tested without a B2xx on the bench.
//!
//! # The policy
//!
//! Three outcomes, in the order they are tried:
//!
//! 1. **In window.** The FPGA reports a major with a [`FpgaShim`] behind it, and
//!    that major is legal for this board. [`Verdict::Proceed`], carrying the
//!    [`FpgaCompat`] the rest of the open path runs on.
//! 2. **Out of window, fixable.** The FPGA reports something else — an older
//!    major, an unsigned readback, or no image at all — *and* a verified image
//!    in the [`uhd_images`] cache would put it in window.
//!    [`Verdict::Reload`], which the caller either offers to the user or, with
//!    `--auto-flash`, performs through `usb::configure`.
//! 3. **Neither.** [`Verdict::Unsupported`], carrying a [`Diagnosis`] that names
//!    the device's compatibility number, this build's supported window, and the
//!    manifest entry that would fix it if one exists.
//!
//! The firmware compatibility number is checked first and separately, because a
//! firmware major this driver does not know is not something an FPGA reload can
//! fix — and pushing a bitstream through unknown firmware is how you hang a
//! device rather than how you diagnose one.
//!
//! # Why an FPGA reload is a reasonable answer at all
//!
//! Because a B2xx has no configuration flash. The image in the FPGA got there
//! over USB since the last power cycle, so replacing it costs eight seconds and
//! loses nothing — quite unlike a device where "reflash the FPGA" is a
//! last-resort operation. The out-of-window case is genuinely routine: it is
//! what a freshly-initialised board with no bitstream looks like.
//!
//! # Testing without hardware
//!
//! Everything here is a function of [`DeviceCompat`] and an [`ImageCatalog`],
//! both of which are plain data. [`UhdImagesCatalog`] is the real one;
//! the tests below use a table-driven fake, which is what makes the policy
//! matrix a unit test rather than a bench procedure.

use std::fmt;
use std::path::PathBuf;

use uhd_core::{Error, Result};
use uhd_transport::usb::{
    B2xxProduct, CompatReport, FirmwareVersion, FpgaCompat as FpgaReadback, FpgaVersion,
};

use crate::compat::{FpgaCompat, FpgaShim};

/// Which of a device's two loadable images a catalogue entry is.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum ImageKind {
    /// FX3 firmware, `usrp_b200_fw.hex`.
    Firmware,
    /// An FPGA bitstream.
    Fpga,
}

impl ImageKind {
    /// The `kind` string this maps to in the `uhd-images` manifest.
    #[must_use]
    pub const fn manifest_key(self) -> &'static str {
        match self {
            Self::Firmware => "firmware",
            Self::Fpga => "fpga",
        }
    }
}

impl fmt::Display for ImageKind {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(self.manifest_key())
    }
}

/// One image this driver could load, and whether it is already on disk.
///
/// A flattened [`uhd_images::Image`] plus the cache lookup, so that the policy
/// does not have to know how images are stored and the tests do not have to
/// build a cache.
#[derive(Debug, Clone, PartialEq, Eq)]
#[non_exhaustive]
pub struct ImageCandidate {
    /// Manifest device key, e.g. `b205mini`.
    pub device: String,
    /// Which image this is.
    pub kind: ImageKind,
    /// File name inside the official archive, which is also the name the image
    /// search path looks for.
    pub filename: String,
    /// SHA-256 of the extracted file — the entry's identity.
    pub sha256: String,
    /// FPGA compatibility major this image builds to. Meaningful for
    /// [`ImageKind::Fpga`]; carried but unused for firmware.
    pub compat_major: u16,
    /// FPGA compatibility minor.
    pub compat_minor: u16,
    /// UHD releases that shipped this exact image, oldest first.
    pub releases: Vec<String>,
    /// Where the verified object sits in the cache, or `None` if it is only a
    /// manifest entry.
    pub cached: Option<PathBuf>,
}

impl ImageCandidate {
    /// The supported major this image would put a device on, if any.
    #[must_use]
    pub fn compat(&self) -> Option<FpgaCompat> {
        FpgaCompat::from_major(self.compat_major)
    }

    /// Whether a verified copy is already in the cache.
    #[must_use]
    pub fn is_cached(&self) -> bool {
        self.cached.is_some()
    }

    /// The newest UHD release that shipped this image.
    #[must_use]
    pub fn newest_release(&self) -> Option<&str> {
        self.releases.last().map(String::as_str)
    }

    /// The first eight hex digits of the SHA-256, which is enough to pick the
    /// entry out of a manifest by eye.
    #[must_use]
    pub fn short_hash(&self) -> &str {
        let end = self.sha256.len().min(8);
        &self.sha256[..end]
    }
}

impl fmt::Display for ImageCandidate {
    /// Renders the entry the way a diagnostic wants to name it.
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{} ({} {}", self.filename, self.device, self.kind)?;
        if self.kind == ImageKind::Fpga {
            write!(
                f,
                ", fpga compat {}.{}",
                self.compat_major, self.compat_minor
            )?;
        }
        if let Some(release) = self.newest_release() {
            write!(f, ", UHD {release}")?;
        }
        write!(f, ", sha256 {}…)", self.short_hash())
    }
}

/// Where the policy layer looks for images it could load.
///
/// Implemented for real by [`UhdImagesCatalog`]; implemented by a fixture in
/// this module's tests, which is the point of the indirection.
pub trait ImageCatalog: fmt::Debug {
    /// Every known image of `kind` for `product`, **newest first**.
    ///
    /// Order matters: the policy takes the first cached, in-window entry, so
    /// newest-first means a device gets the newest image that would work.
    fn images(&self, product: B2xxProduct, kind: ImageKind) -> Vec<ImageCandidate>;
}

/// What the FPGA compatibility register actually said.
///
/// [`FpgaReadback`] with the "it worked" case removed, so that this can only
/// describe a device that needs something done to it.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[non_exhaustive]
pub enum FoundFpga {
    /// No bitstream has been pushed since power-up. The normal state of a
    /// freshly initialised board, not a fault.
    Unconfigured,
    /// The register answered, but the top half was not [`FPGA_SIGNATURE`], so
    /// the numbers under it are not version numbers.
    ///
    /// [`FPGA_SIGNATURE`]: crate::compat::FPGA_SIGNATURE
    Unsigned {
        /// The signature word that came back.
        signature: u32,
    },
    /// A genuine version number, for a major this build has no shim for or
    /// which is not legal on this board.
    OutOfWindow {
        /// The major reported.
        major: u16,
        /// The minor reported.
        minor: u16,
    },
}

impl fmt::Display for FoundFpga {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Unconfigured => f.write_str("no fpga image is loaded"),
            Self::Unsigned { signature } => write!(
                f,
                "the fpga compatibility register holds signature {signature:#010x}, \
                 so the readback is not a version number"
            ),
            Self::OutOfWindow { major, minor } => {
                write!(f, "the fpga reports compatibility {major}.{minor}")
            }
        }
    }
}

/// A device's own account of its versions, reduced to what the policy needs.
///
/// Built from a [`CompatReport`] with [`DeviceCompat::from_report`]. Kept as a
/// separate type because it is `Copy`-cheap plain data with no USB handle
/// behind it, which is what makes the matrix below a unit test.
#[derive(Debug, Clone, PartialEq, Eq)]
#[non_exhaustive]
pub struct DeviceCompat {
    /// Which board this is. The policy cannot run without it, since the legal
    /// FPGA major depends on it.
    pub product: B2xxProduct,
    /// The firmware compatibility number the FX3 reports.
    pub firmware: FirmwareVersion,
    /// The firmware compatibility number this driver was written against.
    pub expected_firmware: FirmwareVersion,
    /// What the FPGA compatibility register said, or why it could not be read.
    pub fpga: FpgaReadback,
    /// The board revision from the motherboard EEPROM, which the shim needs for
    /// [`FpgaShim::frontends_swapped`].
    pub board_revision: Option<u16>,
}

impl DeviceCompat {
    /// Reduces a gathered report.
    ///
    /// # Errors
    ///
    /// [`Error::Device`] if neither USB nor the EEPROM identified the product.
    /// There is no useful policy for a board whose model is unknown: the legal
    /// FPGA major, the image to load and the shim to run on all follow from it,
    /// and guessing would be worse than saying so.
    pub fn from_report(report: &CompatReport) -> Result<Self> {
        let product = report.product.ok_or_else(|| {
            Error::Device(format!(
                "cannot decide fpga compatibility for {}: usb reports the product id the \
                 b200 and b210 share and the eeprom product code is {}, so which fpga \
                 image this board needs is unknown",
                report.endpoint,
                match report.eeprom.product_code {
                    Some(code) => format!("{code:#06x}"),
                    None => "unset".to_owned(),
                }
            ))
        })?;

        Ok(Self {
            product,
            firmware: report.firmware,
            expected_firmware: report.expected_firmware,
            fpga: report.fpga.clone(),
            board_revision: report.eeprom.revision,
        })
    }

    /// The FPGA major this board must be running.
    #[must_use]
    pub fn required_compat(&self) -> FpgaCompat {
        FpgaCompat::for_product(self.product)
    }

    /// Whether the firmware's major is the expected one, which is
    /// `check_fw_compat`'s rule.
    #[must_use]
    pub fn firmware_ok(&self) -> bool {
        self.firmware.major == self.expected_firmware.major
    }
}

/// An out-of-window device that a cached image would fix.
#[derive(Debug, Clone, PartialEq, Eq)]
#[non_exhaustive]
pub struct Reload {
    /// The major the device would be on afterwards.
    pub target: FpgaCompat,
    /// The cached image to push. [`ImageCandidate::cached`] is always `Some`
    /// here; that is what distinguishes this from [`Verdict::Unsupported`].
    pub image: ImageCandidate,
    /// What the device is reporting now.
    pub found: FoundFpga,
}

impl Reload {
    /// The path of the cached object to load.
    ///
    /// # Errors
    ///
    /// [`Error::Internal`] if the candidate is not cached, which
    /// [`negotiate`] never produces.
    pub fn path(&self) -> Result<&std::path::Path> {
        self.image.cached.as_deref().ok_or_else(|| {
            Error::internal(format!(
                "reload candidate {} is not in the image cache",
                self.image.filename
            ))
        })
    }
}

impl fmt::Display for Reload {
    /// The offer, phrased for someone deciding whether to accept it.
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(
            f,
            "{}, and this build needs {}. A verified image in the cache would fix \
             it: {}. Re-run with --auto-flash to load it, or run `ruhd init` by hand.",
            self.found, self.target, self.image
        )
    }
}

/// Why a device cannot be opened, in enough detail to fix it.
#[derive(Debug, Clone, PartialEq, Eq)]
#[non_exhaustive]
pub struct Diagnosis {
    /// Which board this is about.
    pub product: B2xxProduct,
    /// What is wrong.
    pub trouble: Trouble,
    /// Every FPGA major this build supports, which is [`FpgaCompat::WINDOW`].
    pub window: Vec<FpgaCompat>,
    /// The major this particular board would have to be on.
    pub required: FpgaCompat,
    /// The manifest entry that would fix it, if the manifest has one. `None`
    /// means the manifest does not cover this board at all — which is the
    /// honest answer for a B200 or B210 today.
    pub remedy: Option<ImageCandidate>,
}

/// The kinds of trouble [`negotiate`] refuses on.
#[derive(Debug, Clone, PartialEq, Eq)]
#[non_exhaustive]
pub enum Trouble {
    /// The FX3 firmware's major is not the one this driver knows. Checked
    /// first, and never fixed by an FPGA reload.
    Firmware {
        /// What the device reports.
        found: FirmwareVersion,
        /// What this driver was built against.
        expected: FirmwareVersion,
    },
    /// The FPGA compatibility register could not be read at all. Not treated as
    /// an out-of-window FPGA, because the usual cause is another process
    /// holding the bulk interfaces and reflashing a perfectly good FPGA is the
    /// wrong response to that.
    Unreadable {
        /// What the read failed with.
        why: String,
    },
    /// The FPGA is out of window and nothing in the cache would fix it.
    Fpga {
        /// What the device reports.
        found: FoundFpga,
    },
}

impl Diagnosis {
    /// The single line a summary wants.
    #[must_use]
    pub fn headline(&self) -> String {
        match &self.trouble {
            Trouble::Firmware { found, expected } => format!(
                "firmware compatibility {found}, but this driver speaks {expected} \
                 (only the major has to match)"
            ),
            Trouble::Unreadable { why } => {
                format!("the fpga compatibility number could not be read: {why}")
            }
            Trouble::Fpga { found } => found.to_string(),
        }
    }

    /// The suggested fix, if there is one that can be stated.
    #[must_use]
    pub fn recommended_fix(&self) -> Option<String> {
        let device = self.product.name().to_ascii_lowercase();
        match (&self.trouble, &self.remedy) {
            (Trouble::Unreadable { .. }, _) => Some(
                "close anything else holding the device (`uhd_usrp_probe`, a running \
                 stream) and try again; if nothing is, reload the fpga with `ruhd init \
                 --force-fpga`"
                    .to_owned(),
            ),
            (_, Some(remedy)) => Some(format!(
                "fetch {remedy} with `ruhd images sync --device {device}`, then \
                 `ruhd init`"
            )),
            (_, None) => None,
        }
    }

    /// The supported window, rendered as the list a diagnostic quotes.
    #[must_use]
    pub fn window_text(&self) -> String {
        let majors: Vec<String> = self.window.iter().map(|c| c.major().to_string()).collect();
        format!(
            "this build supports fpga compat major {} and a {} must be on {}",
            majors.join(" and "),
            self.product,
            self.required.major()
        )
    }
}

impl fmt::Display for Diagnosis {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{}; {}", self.headline(), self.window_text())?;
        match (&self.remedy, self.recommended_fix()) {
            (Some(_), Some(fix)) => write!(f, ". {fix}"),
            (None, Some(fix)) => write!(f, ". {fix}"),
            (_, None) => write!(
                f,
                ". No image in the manifest covers a {}, so the image has to come from \
                 somewhere else — pass one to `ruhd init --fpga`",
                self.product
            ),
        }
    }
}

/// What to do about a device.
///
/// Deliberately not `Result`-shaped: "this needs a reload" is neither success
/// nor failure until a caller has decided whether it is allowed to perform one.
#[derive(Debug, Clone, PartialEq, Eq)]
#[non_exhaustive]
pub enum Verdict {
    /// The FPGA is in window. Run on this major's shim.
    Proceed {
        /// The negotiated major. [`FpgaCompat::shim`] is the behaviour.
        compat: FpgaCompat,
    },
    /// Out of window, and a cached image would fix it.
    Reload(Box<Reload>),
    /// Out of window with no way out from here.
    Unsupported(Box<Diagnosis>),
}

impl Verdict {
    /// The negotiated major, when there is one.
    #[must_use]
    pub fn compat(&self) -> Option<FpgaCompat> {
        match self {
            Self::Proceed { compat } => Some(*compat),
            _ => None,
        }
    }

    /// The shim to run on, when the device is usable as it stands.
    #[must_use]
    pub fn shim(&self) -> Option<&'static dyn FpgaShim> {
        self.compat().map(FpgaCompat::shim)
    }

    /// Whether the device can be opened without doing anything to it.
    #[must_use]
    pub fn is_ok(&self) -> bool {
        matches!(self, Self::Proceed { .. })
    }

    /// One line naming the verdict, for `doctor` and for logs.
    #[must_use]
    pub fn headline(&self) -> String {
        match self {
            Self::Proceed { compat } => format!(
                "supported — running fpga compat {} on the {} shim",
                compat.major(),
                compat.shim().codename()
            ),
            Self::Reload(reload) => format!(
                "needs an fpga reload — {}, and a cached image would fix it",
                reload.found
            ),
            Self::Unsupported(diagnosis) => format!("unsupported — {}", diagnosis.headline()),
        }
    }

    /// The recommended fix, if the device needs one.
    #[must_use]
    pub fn recommended_fix(&self) -> Option<String> {
        match self {
            Self::Proceed { .. } => None,
            Self::Reload(reload) => Some(format!(
                "load {} — `ruhd doctor --auto-flash`, or `ruhd init`",
                reload.image
            )),
            Self::Unsupported(diagnosis) => diagnosis.recommended_fix(),
        }
    }
}

/// Decides what to do about a device.
///
/// The whole policy, in the order described in the [module docs](self). Pure:
/// no I/O beyond whatever `catalog` does, and no decision depends on anything
/// but its two arguments.
#[must_use]
pub fn negotiate(device: &DeviceCompat, catalog: &dyn ImageCatalog) -> Verdict {
    let required = device.required_compat();

    // Firmware first. An FPGA reload cannot fix a firmware mismatch, and
    // attempting one against firmware whose protocol this driver does not know
    // is how a device hangs rather than how it gets diagnosed.
    if !device.firmware_ok() {
        return unsupported(
            device,
            Trouble::Firmware {
                found: device.firmware,
                expected: device.expected_firmware,
            },
            catalog.images(device.product, ImageKind::Firmware).first(),
        );
    }

    let found = match &device.fpga {
        FpgaReadback::Known(version) if version.is_signed() => {
            match FpgaCompat::from_major(version.major) {
                Some(compat) if compat.accepts(device.product) => {
                    return Verdict::Proceed { compat };
                }
                _ => FoundFpga::OutOfWindow {
                    major: version.major,
                    minor: version.minor,
                },
            }
        }
        FpgaReadback::Known(version) => FoundFpga::Unsigned {
            signature: version.signature,
        },
        FpgaReadback::Unconfigured => FoundFpga::Unconfigured,
        FpgaReadback::Unavailable(why) => {
            return unsupported(device, Trouble::Unreadable { why: why.clone() }, None);
        }
        // `FpgaCompat` is `#[non_exhaustive]`, so a variant added upstream
        // lands here. Refusing is the only safe default: an unknown way for
        // the read to have gone is not evidence that reflashing would help.
        other => {
            return unsupported(
                device,
                Trouble::Unreadable {
                    why: format!("the compatibility read reported `{other}`"),
                },
                None,
            );
        }
    };

    // Out of window. The newest cached image that would land the device on the
    // major it needs wins; the newest manifest entry that would is the fallback
    // remedy to name.
    let candidates = catalog.images(device.product, ImageKind::Fpga);
    let on_target = |candidate: &&ImageCandidate| candidate.compat() == Some(required);

    if let Some(image) = candidates
        .iter()
        .filter(on_target)
        .find(|candidate| candidate.is_cached())
    {
        return Verdict::Reload(Box::new(Reload {
            target: required,
            image: image.clone(),
            found,
        }));
    }

    unsupported(
        device,
        Trouble::Fpga { found },
        candidates.iter().find(on_target),
    )
}

/// Builds an [`Verdict::Unsupported`] with the window filled in.
fn unsupported(
    device: &DeviceCompat,
    trouble: Trouble,
    remedy: Option<&ImageCandidate>,
) -> Verdict {
    Verdict::Unsupported(Box::new(Diagnosis {
        product: device.product,
        trouble,
        window: FpgaCompat::WINDOW.to_vec(),
        required: device.required_compat(),
        remedy: remedy.cloned(),
    }))
}

/// Convenience: a signed [`FpgaReadback`] for a major and minor.
///
/// Saves every caller spelling out the signature word, and keeps the one place
/// that knows what a well-formed readback looks like in this crate.
#[must_use]
pub fn signed_readback(major: u16, minor: u16) -> FpgaReadback {
    FpgaReadback::Known(FpgaVersion {
        signature: crate::compat::FPGA_SIGNATURE,
        major,
        minor,
    })
}

/// The [`ImageCatalog`] backed by the built-in `uhd-images` manifest and the
/// user's image cache.
///
/// The cache is consulted with [`uhd_images::Cache::verify`], so an entry only
/// counts as cached if the object is present *and* hashes to what the manifest
/// says. A truncated download therefore reads as "not cached" and produces a
/// diagnostic telling you to sync, rather than a reload that pushes garbage
/// into an FPGA.
#[derive(Debug)]
pub struct UhdImagesCatalog {
    manifest: uhd_images::Manifest,
    cache: Option<uhd_images::Cache>,
}

impl UhdImagesCatalog {
    /// The built-in manifest and the user's cache.
    ///
    /// A cache that cannot be located is not an error — it means nothing is
    /// cached, which is a policy input rather than a failure.
    ///
    /// # Errors
    ///
    /// [`Error::Firmware`] if the compiled-in manifest does not parse, which is
    /// a build defect rather than a runtime condition.
    pub fn builtin() -> Result<Self> {
        let manifest = uhd_images::Manifest::builtin()
            .map_err(|e| Error::Firmware(format!("the built-in image manifest is invalid: {e}")))?;
        Ok(Self {
            manifest,
            cache: uhd_images::Cache::discover().ok(),
        })
    }

    /// An explicit manifest and cache, for tests and for an air-gapped cache in
    /// a non-default location.
    #[must_use]
    pub fn new(manifest: uhd_images::Manifest, cache: Option<uhd_images::Cache>) -> Self {
        Self { manifest, cache }
    }

    /// Where cached objects are looked for, if anywhere.
    #[must_use]
    pub fn cache_root(&self) -> Option<&std::path::Path> {
        self.cache.as_ref().map(uhd_images::Cache::root)
    }
}

impl ImageCatalog for UhdImagesCatalog {
    fn images(&self, product: B2xxProduct, kind: ImageKind) -> Vec<ImageCandidate> {
        let device = product.name().to_ascii_lowercase();
        // The manifest is oldest-first; the contract here is newest-first.
        self.manifest
            .images
            .iter()
            .rev()
            .filter(|image| image.device.eq_ignore_ascii_case(&device))
            .filter(|image| image.kind == kind.manifest_key())
            .map(|image| ImageCandidate {
                device: image.device.clone(),
                kind,
                filename: image.filename.clone(),
                sha256: image.sha256.clone(),
                compat_major: image.fpga_compat_major,
                compat_minor: image.fpga_compat_minor,
                releases: image.release.clone(),
                cached: self.cache.as_ref().and_then(|cache| {
                    cache
                        .verify(image)
                        .ok()
                        .map(|()| cache.object_path(&image.sha256))
                }),
            })
            .collect()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use uhd_transport::usb::B200_FW_COMPAT;

    /// A catalogue built from a literal table, which is what makes the matrix
    /// below independent of the shipped manifest.
    #[derive(Debug, Default)]
    struct FakeCatalog(Vec<ImageCandidate>);

    impl FakeCatalog {
        /// Newest-first, as the trait requires.
        fn with(mut self, kind: ImageKind, major: u16, minor: u16, cached: bool) -> Self {
            self.0.insert(
                0,
                ImageCandidate {
                    device: "b205mini".into(),
                    kind,
                    filename: match kind {
                        ImageKind::Firmware => "usrp_b200_fw.hex".into(),
                        ImageKind::Fpga => "usrp_b205mini_fpga.bin".into(),
                    },
                    sha256: format!("{:064x}", u64::from(major) << 16 | u64::from(minor)),
                    compat_major: major,
                    compat_minor: minor,
                    releases: vec![format!("v4.{minor}.0.0")],
                    cached: cached.then(|| PathBuf::from("/cache/objects/deadbeef")),
                },
            );
            self
        }
    }

    impl ImageCatalog for FakeCatalog {
        fn images(&self, _product: B2xxProduct, kind: ImageKind) -> Vec<ImageCandidate> {
            self.0.iter().filter(|i| i.kind == kind).cloned().collect()
        }
    }

    /// The catalogue a correctly synced B205mini has: a cached major-7 image,
    /// plus an older one.
    fn synced() -> FakeCatalog {
        FakeCatalog::default()
            .with(ImageKind::Fpga, 7, 0, true)
            .with(ImageKind::Fpga, 7, 1, true)
            .with(ImageKind::Firmware, 0, 0, true)
    }

    /// The manifest knows an image but nothing has been downloaded.
    fn unsynced() -> FakeCatalog {
        FakeCatalog::default()
            .with(ImageKind::Fpga, 7, 1, false)
            .with(ImageKind::Firmware, 0, 0, false)
    }

    fn device(product: B2xxProduct, fpga: FpgaReadback) -> DeviceCompat {
        DeviceCompat {
            product,
            firmware: B200_FW_COMPAT,
            expected_firmware: B200_FW_COMPAT,
            fpga,
            board_revision: Some(2),
        }
    }

    fn mini(fpga: FpgaReadback) -> DeviceCompat {
        device(B2xxProduct::B205Mini, fpga)
    }

    // ---- (a) in window -------------------------------------------------

    #[test]
    fn a_mini_on_major_7_proceeds_on_the_mini_shim() {
        let verdict = negotiate(&mini(signed_readback(7, 0)), &synced());
        assert_eq!(verdict.compat(), Some(FpgaCompat::V7));
        assert_eq!(verdict.shim().map(FpgaShim::codename), Some("Pixie"));
        assert!(verdict.is_ok());
        assert_eq!(verdict.recommended_fix(), None);
    }

    #[test]
    fn a_b210_on_major_16_proceeds_on_the_full_shim() {
        let verdict = negotiate(
            &device(B2xxProduct::B210, signed_readback(16, 0)),
            &FakeCatalog::default(),
        );
        assert_eq!(verdict.compat(), Some(FpgaCompat::V16));
        assert_eq!(verdict.shim().map(FpgaShim::max_radio_chains), Some(2));
    }

    #[test]
    fn the_minor_number_is_not_part_of_the_decision() {
        // check_fpga_compat compares only the major, and so does this.
        for minor in [0, 1, 9, u16::MAX] {
            assert!(
                negotiate(&mini(signed_readback(7, minor)), &synced()).is_ok(),
                "minor {minor} should not matter"
            );
        }
    }

    #[test]
    fn a_supported_major_on_the_wrong_board_is_still_out_of_window() {
        // Major 16 has a shim, but it is not a shim a B205mini can run on.
        // "In the window" is per-device, not per-driver.
        let verdict = negotiate(&mini(signed_readback(16, 0)), &synced());
        assert!(matches!(verdict, Verdict::Reload(_)), "{verdict:?}");
    }

    // ---- (b) out of window, a cached image fixes it --------------------

    #[test]
    fn an_unconfigured_fpga_with_a_cached_image_is_a_reload() {
        // The routine case: a board that has just had firmware loaded.
        let verdict = negotiate(&mini(FpgaReadback::Unconfigured), &synced());
        let Verdict::Reload(reload) = verdict else {
            panic!("expected a reload, got {verdict:?}");
        };
        assert_eq!(reload.target, FpgaCompat::V7);
        assert_eq!(reload.found, FoundFpga::Unconfigured);
        assert!(reload.image.is_cached());
        assert!(reload.path().is_ok());
    }

    #[test]
    fn an_older_major_with_a_cached_image_is_a_reload() {
        let verdict = negotiate(&mini(signed_readback(6, 3)), &synced());
        let Verdict::Reload(reload) = verdict else {
            panic!("expected a reload, got {verdict:?}");
        };
        assert_eq!(reload.found, FoundFpga::OutOfWindow { major: 6, minor: 3 });
        assert_eq!(reload.image.compat(), Some(FpgaCompat::V7));
    }

    #[test]
    fn an_unsigned_readback_is_a_reload_rather_than_a_version_zero() {
        // Zeros are what a settings bus that is not there reads back as.
        // Reading that as "version 0.0" would be worse than useless.
        let verdict = negotiate(
            &mini(FpgaReadback::Known(FpgaVersion::from_register(0))),
            &synced(),
        );
        let Verdict::Reload(reload) = verdict else {
            panic!("expected a reload, got {verdict:?}");
        };
        assert_eq!(reload.found, FoundFpga::Unsigned { signature: 0 });
    }

    #[test]
    fn the_newest_cached_in_window_image_wins() {
        // Two cached major-7 images; the catalogue's newest-first order picks.
        let catalog = FakeCatalog::default()
            .with(ImageKind::Fpga, 7, 0, true)
            .with(ImageKind::Fpga, 7, 5, true);
        let Verdict::Reload(reload) = negotiate(&mini(FpgaReadback::Unconfigured), &catalog) else {
            panic!("expected a reload");
        };
        assert_eq!(reload.image.compat_minor, 5);
    }

    #[test]
    fn a_cached_but_out_of_window_image_does_not_count() {
        // A cached major-6 image is not a fix for a board that needs 7.
        let catalog = FakeCatalog::default().with(ImageKind::Fpga, 6, 0, true);
        let verdict = negotiate(&mini(FpgaReadback::Unconfigured), &catalog);
        assert!(matches!(verdict, Verdict::Unsupported(_)), "{verdict:?}");
    }

    // ---- (c) out of window, nothing to do about it ---------------------

    #[test]
    fn an_unsynced_manifest_entry_is_named_as_the_fix() {
        let verdict = negotiate(&mini(signed_readback(6, 0)), &unsynced());
        let Verdict::Unsupported(diagnosis) = &verdict else {
            panic!("expected unsupported, got {verdict:?}");
        };

        // The three things the diagnostic is required to name.
        let text = diagnosis.to_string();
        assert!(text.contains("compatibility 6.0"), "{text}");
        assert!(text.contains("major 7 and 16"), "{text}");
        assert!(text.contains("usrp_b205mini_fpga.bin"), "{text}");
        assert!(
            text.contains("ruhd images sync --device b205mini"),
            "{text}"
        );

        assert_eq!(diagnosis.window, FpgaCompat::WINDOW.to_vec());
        assert_eq!(diagnosis.required, FpgaCompat::V7);
        assert_eq!(
            diagnosis.remedy.as_ref().map(ImageCandidate::is_cached),
            Some(false)
        );
    }

    #[test]
    fn an_empty_manifest_says_so_rather_than_inventing_a_fix() {
        // What a B210 gets today: the manifest does not cover it.
        let verdict = negotiate(
            &device(B2xxProduct::B210, FpgaReadback::Unconfigured),
            &FakeCatalog::default(),
        );
        let Verdict::Unsupported(diagnosis) = &verdict else {
            panic!("expected unsupported, got {verdict:?}");
        };
        assert_eq!(diagnosis.remedy, None);
        assert_eq!(diagnosis.recommended_fix(), None);
        assert!(diagnosis.to_string().contains("`ruhd init --fpga`"));
    }

    #[test]
    fn an_unreadable_register_is_not_answered_with_a_reflash() {
        // The usual cause is another process holding the bulk interfaces, and
        // reflashing a working FPGA is the wrong response to that.
        let verdict = negotiate(
            &mini(FpgaReadback::Unavailable("device or resource busy".into())),
            &synced(),
        );
        let Verdict::Unsupported(diagnosis) = &verdict else {
            panic!("expected unsupported, got {verdict:?}");
        };
        assert!(matches!(diagnosis.trouble, Trouble::Unreadable { .. }));
        assert_eq!(
            diagnosis.remedy, None,
            "a busy device is not an image problem"
        );
        assert!(diagnosis.to_string().contains("device or resource busy"));
        assert!(
            diagnosis
                .recommended_fix()
                .is_some_and(|fix| fix.contains("uhd_usrp_probe"))
        );
    }

    // ---- firmware ------------------------------------------------------

    #[test]
    fn a_firmware_mismatch_is_refused_before_the_fpga_is_considered() {
        // Even with a perfectly good FPGA and a full cache: an FPGA reload
        // cannot fix firmware, and pushing a bitstream through firmware this
        // driver does not know is how a device hangs.
        let mut device = mini(signed_readback(7, 0));
        device.firmware = FirmwareVersion { major: 7, minor: 0 };

        let verdict = negotiate(&device, &synced());
        let Verdict::Unsupported(diagnosis) = &verdict else {
            panic!("expected unsupported, got {verdict:?}");
        };
        assert!(matches!(diagnosis.trouble, Trouble::Firmware { .. }));
        assert!(
            diagnosis.headline().contains("7.0"),
            "{}",
            diagnosis.headline()
        );
        assert_eq!(
            diagnosis.remedy.as_ref().map(|r| r.filename.as_str()),
            Some("usrp_b200_fw.hex"),
            "the firmware entry is the one that would fix it"
        );
    }

    #[test]
    fn a_newer_firmware_minor_is_fine() {
        let mut device = mini(signed_readback(7, 0));
        device.firmware = FirmwareVersion {
            major: B200_FW_COMPAT.major,
            minor: B200_FW_COMPAT.minor + 9,
        };
        assert!(negotiate(&device, &synced()).is_ok());
    }

    // ---- the matrix as a whole -----------------------------------------

    #[test]
    fn the_policy_matrix() {
        // Every (device state, catalogue state) pair, and the verdict each has
        // to produce. Adding a row here is how a new policy case gets pinned
        // down; the shape of the table is the specification.
        #[derive(Debug)]
        enum Want {
            Proceed,
            Reload,
            Unsupported,
        }
        use Want::{Proceed, Reload, Unsupported};

        let cases: Vec<(&str, DeviceCompat, FakeCatalog, Want)> = vec![
            (
                "in window, synced",
                mini(signed_readback(7, 0)),
                synced(),
                Proceed,
            ),
            (
                "in window, unsynced",
                mini(signed_readback(7, 0)),
                unsynced(),
                Proceed,
            ),
            (
                "in window, nothing cached at all",
                mini(signed_readback(7, 0)),
                FakeCatalog::default(),
                Proceed,
            ),
            (
                "no image, synced",
                mini(FpgaReadback::Unconfigured),
                synced(),
                Reload,
            ),
            (
                "no image, unsynced",
                mini(FpgaReadback::Unconfigured),
                unsynced(),
                Unsupported,
            ),
            (
                "old major, synced",
                mini(signed_readback(6, 0)),
                synced(),
                Reload,
            ),
            (
                "old major, unsynced",
                mini(signed_readback(6, 0)),
                unsynced(),
                Unsupported,
            ),
            (
                "future major, synced",
                mini(signed_readback(99, 0)),
                synced(),
                Reload,
            ),
            (
                "other board's major, synced",
                mini(signed_readback(16, 0)),
                synced(),
                Reload,
            ),
            (
                "unsigned readback, synced",
                mini(FpgaReadback::Known(FpgaVersion::from_register(0))),
                synced(),
                Reload,
            ),
            (
                "unreadable, synced",
                mini(FpgaReadback::Unavailable("busy".into())),
                synced(),
                Unsupported,
            ),
            (
                "full-size board in window",
                device(B2xxProduct::B200, signed_readback(16, 2)),
                FakeCatalog::default(),
                Proceed,
            ),
        ];

        for (name, device, catalog, want) in cases {
            let got = negotiate(&device, &catalog);
            let matched = match want {
                Proceed => matches!(got, Verdict::Proceed { .. }),
                Reload => matches!(got, Verdict::Reload(_)),
                Unsupported => matches!(got, Verdict::Unsupported(_)),
            };
            assert!(matched, "{name}: wanted {want:?}, got {got:?}");

            // Whatever the verdict, it can always be explained.
            assert!(!got.headline().is_empty(), "{name}");
            assert_eq!(
                got.recommended_fix().is_some(),
                !matches!(want, Proceed),
                "{name}: a device that needs something done must say what"
            );
        }
    }

    // ---- the real catalogue --------------------------------------------

    #[test]
    fn the_shipped_manifest_covers_the_b205mini_in_window() {
        // The commitment in NOTES.md in one assertion: every supported major
        // that the manifest claims to serve has to be one this build has a
        // shim for, or `ruhd images sync` installs an image we would then
        // refuse.
        let catalog = UhdImagesCatalog::new(uhd_images::Manifest::builtin().unwrap(), None);
        let images = catalog.images(B2xxProduct::B205Mini, ImageKind::Fpga);
        assert!(
            !images.is_empty(),
            "the manifest ships a b205mini fpga image"
        );
        for image in &images {
            assert_eq!(
                image.compat(),
                Some(FpgaCompat::V7),
                "{} is major {} — outside the window",
                image.filename,
                image.compat_major
            );
            assert!(!image.is_cached(), "no cache was given");
        }

        // Newest first, which is the order the policy relies on.
        assert_eq!(images[0].newest_release(), Some("v4.10.0.0"));
    }

    #[test]
    fn a_candidate_renders_with_everything_needed_to_find_it() {
        let catalog = UhdImagesCatalog::new(uhd_images::Manifest::builtin().unwrap(), None);
        let image = &catalog.images(B2xxProduct::B205Mini, ImageKind::Fpga)[0];
        let text = image.to_string();
        assert!(text.contains("usrp_b205mini_fpga.bin"), "{text}");
        assert!(text.contains("fpga compat 7.0"), "{text}");
        assert!(text.contains("UHD v4.10.0.0"), "{text}");
        assert!(text.contains(image.short_hash()), "{text}");
    }
}
