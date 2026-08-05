//! Compatibility negotiation against a real B2xx, once per cached image
//! version.
//!
//! Gated behind the `hw-tests` feature so that `cargo test --workspace` on a
//! machine with no hardware stays green. CI runs this on a self-hosted runner
//! with a B205mini attached, as a matrix over UHD releases: each leg gets its
//! own image cache holding exactly one FPGA image, and each leg proves that the
//! policy layer put *that* image into the device.
//!
//! # Running it by hand
//!
//! ```sh
//! export UHD_IMAGES_CACHE_DIR=$(mktemp -d)
//! export RUHD_HW_RELEASE=v4.10.0.0
//! cargo run --bin ruhd -- images sync --device b205mini --release $RUHD_HW_RELEASE
//! cargo run --bin ruhd -- reset          # back to needing firmware
//! cargo run --bin ruhd -- init --no-fpga # firmware up, fpga left unconfigured
//! cargo test -p uhd-b200 --features hw-tests -- --test-threads 1
//! ```
//!
//! The `init --no-fpga` matters: it is what leaves the device in the
//! out-of-window state the policy has to resolve. Without it the device may
//! already be holding the other leg's bitstream, and the final assertion —
//! that the FX3 is holding this leg's image hash — is what catches that.
//!
//! `--test-threads 1` matters too: the tests all want the same device, and two
//! of them claiming its bulk interfaces at once is a permission error, not a
//! finding.
//!
//! # What is deliberately not tested here
//!
//! The radio-chain scratch gate ([`RadioCtrl::gate`]) needs the AD936x to be
//! clocking, because the radio cores run off the codec clock — and the codec
//! initialisation sequence that starts it is not ported yet. Until it is, the
//! radio registers only answer after a stock `uhd_usrp_probe` has initialised
//! the codec, which is not a dependency a CI job should have.
//! [`radio_chain_scratch_gate`] therefore runs only when `RUHD_HW_RADIO_CLOCKED`
//! is set, and everything else here stays on the local core, which answers as
//! soon as the FPGA is configured.
//!
//! That is not a gap in what this file is for. Compat negotiation is settled
//! entirely on the local core: the compatibility register lives at address 0 of
//! it, and so does the radio-chain count.
//!
//! `hw_codec.rs` covers the other local-core work — SPI, the codec reset line,
//! and reaching the AD936x itself — and runs in the same matrix with no such
//! caveat.
//!
//! [`RadioCtrl::gate`]: uhd_b200::RadioCtrl::gate
#![cfg(feature = "hw-tests")]

use std::path::PathBuf;

use uhd_b200::{
    FpgaCompat, ImageCandidate, ImageCatalog, ImageKind, OpenOptions, UhdImagesCatalog, Verdict,
    checkup, open_probed,
};
use uhd_transport::usb::{self, B2xxProduct, UsbFilter, UsbProbe};

/// UHD release this leg of the matrix is pinned to, e.g. `v4.10.0.0`.
const RELEASE_ENV: &str = "RUHD_HW_RELEASE";

/// Serial of the device to use, when more than one is attached.
const SERIAL_ENV: &str = "RUHD_HW_SERIAL";

/// The release this leg is running, or a message saying how to set one.
fn release() -> String {
    std::env::var(RELEASE_ENV).unwrap_or_else(|_| {
        panic!("set {RELEASE_ENV} to the UHD release whose images are cached, e.g. v4.10.0.0")
    })
}

/// The one attached device, in a state that can answer questions.
fn device() -> UsbProbe {
    let mut filter = UsbFilter::new();
    filter.serial = std::env::var(SERIAL_ENV).ok();

    let mut found = usb::probe_filtered(&filter).expect("usb enumeration");
    assert_eq!(
        found.len(),
        1,
        "hw tests need exactly one B2xx attached; found {} (set {SERIAL_ENV} to choose)",
        found.len()
    );
    let device = found.remove(0);
    assert!(
        !device.needs_firmware(),
        "{} has no firmware running ({}); run `ruhd init --no-fpga` first",
        device.endpoint,
        device.state
    );
    device
}

/// The FPGA image this leg is supposed to install.
///
/// Also the isolation check: a leg whose cache holds two FPGA images is not
/// testing what it thinks it is, because the policy would pick the newer one
/// regardless of which release the leg names.
fn leg_image(catalog: &UhdImagesCatalog, product: B2xxProduct) -> ImageCandidate {
    let release = release();
    let cached: Vec<ImageCandidate> = catalog
        .images(product, ImageKind::Fpga)
        .into_iter()
        .filter(ImageCandidate::is_cached)
        .collect();

    assert_eq!(
        cached.len(),
        1,
        "this leg's cache ({:?}) holds {} fpga images for a {product}; it must hold \
         exactly one, or the matrix is not testing one version per leg",
        catalog.cache_root(),
        cached.len()
    );

    let image = cached.into_iter().next().expect("length checked");
    assert!(
        image.releases.contains(&release),
        "the cached image {image} is not the one {release} shipped"
    );
    image
}

/// What the FX3 says is loaded, and what the leg's image hashes to.
fn loaded_hash(session: &uhd_b200::B200Session, path: &PathBuf) -> (u32, u32) {
    let stored = session.iface().fpga_hash().expect("reading the fpga hash");
    let expected = usb::image_hash(&std::fs::read(path).expect("reading the cached image"));
    (stored, expected)
}

#[test]
fn the_cached_image_is_the_one_this_leg_names() {
    let catalog = UhdImagesCatalog::builtin().expect("the built-in manifest");
    let probe = device();
    let product = probe
        .product
        .expect("usb identifies the product for every board but the B200/B210 pair");

    let image = leg_image(&catalog, product);
    assert_eq!(
        image.compat(),
        Some(FpgaCompat::for_product(product)),
        "{image} is outside the supported window, so no leg of this matrix could pass"
    );
    println!("{}: {image}", release());
}

#[test]
fn negotiation_puts_this_legs_image_into_the_device() {
    let catalog = UhdImagesCatalog::builtin().expect("the built-in manifest");
    let probe = device();
    let product = probe.product.expect("usb identifies the product");
    let image = leg_image(&catalog, product);
    let path = image.cached.clone().expect("filtered on is_cached");

    // Before: whatever `ruhd init --no-fpga` left behind. Recorded rather than
    // asserted, because a leg re-run against an already-configured device is a
    // legitimate thing to do and the end state is what matters.
    let before = checkup(&probe, &catalog);
    println!("before: {}", before.headline());

    let session = open_probed(
        &probe,
        &OpenOptions::new().with_auto_flash(true),
        &catalog,
        |sent, total| {
            if sent == total {
                println!("loaded {sent} bytes");
            }
        },
    )
    .expect("open with auto-flash");

    // The verdict resolved to a supported major, and the shim that came with it
    // is the one this board needs.
    assert_eq!(session.compat(), FpgaCompat::for_product(product));
    assert!(session.shim().accepts(product));

    // And the compatibility register agrees with the shim that was selected —
    // this is the claim, read back off the settings bus rather than inferred.
    let version = session
        .report()
        .fpga
        .version()
        .expect("a configured fpga answers its compatibility register");
    assert!(version.is_signed(), "signature {:#010x}", version.signature);
    assert_eq!(version.major, session.compat().major());
    assert_eq!(
        u32::from(version.major) << 16 | u32::from(version.minor),
        u32::from(image.compat_major) << 16 | u32::from(image.compat_minor),
        "the device reports {version}, but {image} claims a different compat"
    );

    // The end state: the device is holding *this leg's* bytes. This is what
    // makes two legs against two cached versions a real matrix rather than the
    // same run twice.
    let (stored, expected) = loaded_hash(&session, &path);
    assert_eq!(
        stored,
        expected,
        "the fx3 holds hash {stored:#010x} but {} hashes to {expected:#010x}; run \
         `ruhd reset && ruhd init --no-fpga` and try again",
        path.display()
    );

    if let Some(flashed) = session.flashed() {
        println!("flashed: {flashed}");
    } else {
        println!("no reload was needed; the device already held this leg's image");
    }
}

#[test]
fn the_local_core_answers_through_the_negotiated_shim() {
    let catalog = UhdImagesCatalog::builtin().expect("the built-in manifest");
    let probe = device();
    let session = open_probed(
        &probe,
        &OpenOptions::new().with_auto_flash(true),
        &catalog,
        |_, _| {},
    )
    .expect("open with auto-flash");

    // Both readbacks go through addresses the shim supplied, which is the part
    // that would break first if a shim's register table were wrong.
    let version = session.ctrl().fpga_version().expect("the compat register");
    assert_eq!(version.major, session.compat().major());
    assert!(version.is_signed(), "signature {:#010x}", version.signature);

    let chains = session.num_radio_chains().expect("reading the chain count");
    assert!(
        chains >= 1 && chains <= session.shim().max_radio_chains(),
        "{chains} radio chains on an fpga compat {} build, whose ceiling is {}",
        session.compat().major(),
        session.shim().max_radio_chains()
    );
    println!("local core: fpga {version}, {chains} radio chain(s)");
}

/// The radio-chain scratch gate, which needs the codec clocking.
///
/// Not part of the matrix: the radio cores run off the AD936x's clock, and the
/// codec initialisation sequence that starts it is not ported yet. Against a
/// device whose codec has not been brought up the readback simply never arrives
/// and the symptom is the full two-second ack timeout, which is a missing driver
/// rather than a compatibility finding.
///
/// Set `RUHD_HW_RADIO_CLOCKED=1` after a stock `uhd_usrp_probe` has run to
/// include it. Delete the gate on this variable once `ad936x` can bring the
/// codec up itself; `hw_codec.rs` is the evidence that the SPI path it will use
/// to do so is already correct.
#[test]
fn radio_chain_scratch_gate() {
    if std::env::var_os("RUHD_HW_RADIO_CLOCKED").is_none() {
        println!(
            "skipped: the radio cores need the AD936x clocking, which this workspace \
             cannot yet do. Run `uhd_usrp_probe` first and set RUHD_HW_RADIO_CLOCKED=1."
        );
        return;
    }

    let catalog = UhdImagesCatalog::builtin().expect("the built-in manifest");
    let probe = device();
    let session = open_probed(&probe, &OpenOptions::new(), &catalog, |_, _| {})
        .expect("open on an already-configured device");

    let gate = session.ctrl().gate().expect("the control-channel gate");
    assert_eq!(gate.fpga.major, session.compat().major());
    assert_eq!(gate.scratch_iterations, 10_000);
    println!(
        "gate passed: fpga {}.{}, {} scratch iterations",
        gate.fpga.major, gate.fpga.minor, gate.scratch_iterations
    );
}

#[test]
fn the_checkup_path_reports_a_supported_device() {
    // The read-only path `ruhd doctor` takes. Every test in this file is
    // self-sufficient — each opens with auto-flash — because libtest's order
    // is alphabetical rather than guaranteed, and a test that only passed
    // second would be a trap.
    let catalog = UhdImagesCatalog::builtin().expect("the built-in manifest");
    let probe = device();
    let _ = open_probed(
        &probe,
        &OpenOptions::new().with_auto_flash(true),
        &catalog,
        |_, _| {},
    )
    .expect("open with auto-flash");

    let checkup = checkup(&probe, &catalog);
    let verdict = checkup.verdict.as_ref().expect("a verdict");
    assert!(
        matches!(verdict, Verdict::Proceed { .. }),
        "{}",
        checkup.headline()
    );
    assert_eq!(checkup.recommended_fix(), None);

    // The four numbers `doctor` prints are all present.
    let report = checkup.report.as_ref().expect("a report");
    assert!(report.firmware_ok(), "firmware {}", report.firmware);
    assert_eq!(report.fpga_ok(), Some(true));
    assert!(
        report.eeprom.revision.is_some(),
        "the eeprom revision is unset; the board's eeprom needs programming"
    );
}
