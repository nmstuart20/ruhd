//! Reaching the AD936x over SPI, against a real B2xx.
//!
//! Gated behind the `hw-tests` feature, like [`hw_compat`](../hw_compat.rs), and
//! run the same way:
//!
//! ```sh
//! cargo run --bin ruhd -- init            # firmware and fpga
//! cargo test -p uhd-b200 --features hw-tests -- --test-threads 1
//! ```
//!
//! # Why this one can run in CI and the scratch gate cannot
//!
//! Everything here is on the FPGA's local core: the misc GPIO word that holds
//! the codec's reset line, and `spi_core_3000`. Both answer as soon as the
//! bitstream is in. The radio cores — where `hw_compat`'s scratch gate lives —
//! need the AD936x to be clocking *and* out of its wait state, and the ENSM
//! transitions that do the second half are not ported yet.
//!
//! So this file is the boundary of what can be proven today without a stock
//! `uhd_usrp_probe` having run first, and it is a real boundary rather than a
//! convenient one: a passing product-ID read is the evidence that the SPI path
//! the initialisation sequence uses is correct, and a locked BBPLL is the
//! evidence that the clock chain built on top of it is.
//!
//! # These tests change the device
//!
//! Several of them reset the codec, and one runs the initialisation sequence as
//! far as it goes. A device that a stock `uhd_usrp_probe` had configured will
//! not still be configured afterwards — re-run the probe, or `ruhd init`, before
//! expecting it to stream.
#![cfg(feature = "hw-tests")]

use ad936x::clocking::freq_is_nearly_equal;
use uhd_b200::{OpenOptions, UhdImagesCatalog, open_probed};
use uhd_transport::usb::{self, UsbFilter, UsbProbe};

/// Serial of the device to use, when more than one is attached.
const SERIAL_ENV: &str = "RUHD_HW_SERIAL";

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

/// An open session with the FPGA in the supported window.
fn session() -> uhd_b200::B200Session {
    let catalog = UhdImagesCatalog::builtin().expect("the built-in manifest");
    open_probed(
        &device(),
        &OpenOptions::new().with_auto_flash(true),
        &catalog,
        |_, _| {},
    )
    .expect("open with auto-flash")
}

#[test]
fn the_codec_answers_its_product_id_over_spi() {
    let session = session();
    let gate = session.codec_gate().expect("the codec bring-up gate");

    assert!(
        gate.product_id.is_ad936x(),
        "product id {:#04x}",
        gate.product_id.raw
    );
    println!("{gate}");
    println!(
        "  raw {:#04x}, family {:#04x}, silicon revision {}",
        gate.product_id.raw,
        gate.product_id.family(),
        gate.product_id.revision()
    );
}

#[test]
fn the_product_id_is_stable_across_repeated_reads() {
    // A readback that changes between reads is the signature of a sampling-edge
    // or divider problem, and it would otherwise pass the gate intermittently —
    // which is far worse than failing it. Ten reads with no reset in between,
    // so anything but a constant is a bus fault.
    let session = session();
    let first = session.codec_gate().expect("the codec bring-up gate");

    for _ in 0..10 {
        let again = session.codec().product_id().expect("re-reading 0x037");
        assert_eq!(
            again, first.product_id,
            "0x037 read back {:#04x} then {:#04x}; the spi path is marginal",
            first.product_id.raw, again.raw
        );
    }
}

#[test]
fn a_register_written_over_spi_reads_back() {
    // The product ID is read-only, so on its own it proves only the read
    // direction. Register 0x03d is one of the LVDS interface registers that
    // `ad9361_device_t::initialize` writes, so it is known read/write on this
    // part and its value means nothing until initialisation reaches it — which
    // it has not, at this point in bring-up. The original is restored anyway,
    // because this test also runs against a device a stock `uhd_usrp_probe` has
    // already configured.
    const SCRATCH: u16 = 0x03d;

    let session = session();
    session.codec_gate().expect("the codec bring-up gate");
    let regs = session.codec().regs();

    let original = regs.peek8(SCRATCH).expect("reading the scratch register");
    for value in [0x00, 0x55, 0xaa, 0xff] {
        regs.poke8(SCRATCH, value).expect("writing scratch");
        let got = regs.peek8(SCRATCH).expect("reading scratch");
        assert_eq!(
            got, value,
            "wrote {value:#04x} to {SCRATCH:#05x} and read {got:#04x}"
        );
    }
    regs.poke8(SCRATCH, original).expect("restoring scratch");
}

#[test]
fn the_reset_line_is_reachable_and_leaves_the_word_clear() {
    // reset_codec is two writes to a write-only register, so the only evidence
    // that the second one happened is the shadow. A device left with the codec
    // reset asserted answers every SPI read with zeros, and the failure would
    // land on whatever ran next rather than here.
    let session = session();
    session.misc().reset_codec().expect("pulsing codec reset");
    assert_eq!(
        session.misc().word() & (1 << 2),
        0,
        "the codec is still held in reset"
    );

    // And the part answers afterwards, which is what makes that assertion mean
    // something rather than just describing the shadow.
    session.codec().reset().expect("the part's soft reset");
    assert!(session.codec().identify().is_ok());
}

#[test]
fn the_clock_chain_comes_up_and_the_bbpll_locks() {
    // The gate for the clock half of the initialisation sequence. Every step of
    // it is a write the part accepts silently, so the lock bit is the only
    // externally visible evidence that any of the arithmetic was right: a wrong
    // divide word, a reference clock that never arrives or a charge pump left at
    // the wrong current all end here, and nowhere else.
    let session = session();
    session.codec_gate().expect("the codec bring-up gate");
    let codec = session.codec();

    let rate = codec
        .initialize_clocking()
        .expect("the clock chain initialisation");

    assert!(
        freq_is_nearly_equal(rate, ad936x::INIT_CLOCK_RATE),
        "came up at {rate} rather than {}",
        ad936x::INIT_CLOCK_RATE
    );
    // 50 MHz through a divfactor of 12 is a 600 MHz core clock, which the BBPLL
    // reaches exactly at a VCO divider of 2. Any other numbers here mean the
    // part solved a different chain from the one this driver programmed.
    assert_eq!(codec.adc_freq(), 600e6);
    assert_eq!(codec.bbpll_freq(), 1200e6);

    // Read the lock bit back directly rather than trusting that the poll inside
    // the driver saw it: a lock that dropped again after the poll returned is
    // exactly the marginal case this test exists to catch.
    let status = codec
        .regs()
        .peek8(ad936x::regs::BBPLL_LOCK_STATUS)
        .expect("reading the bbpll lock status");
    assert_ne!(
        status & ad936x::regs::BBPLL_LOCKED,
        0,
        "the bbpll reported lock during setup and {status:#04x} afterwards"
    );

    println!(
        "clock chain up: {:.3} MHz sample rate, {:.3} MHz adc clock, {:.3} MHz bbpll vco",
        rate / 1e6,
        codec.adc_freq() / 1e6,
        codec.bbpll_freq() / 1e6
    );
}

#[test]
fn the_chain_re_solves_for_every_rate_the_board_is_asked_for() {
    // The rates a B2xx is actually driven at, each one programmed into the part
    // for real. Two things can only fail here: a divide word the BBPLL will not
    // lock to, and a tap count with no filter table behind it — both of which
    // are a table lookup away from the arithmetic that the unit tests cover
    // without a board.
    let session = session();
    session.codec_gate().expect("the codec bring-up gate");
    let codec = session.codec();
    codec
        .initialize_clocking()
        .expect("the clock chain initialisation");

    for requested in [1e6, 4e6, 8e6, 15.36e6, 20e6, 30.72e6, 40e6, 56e6, 61.44e6] {
        let actual = codec
            .setup_rates(requested)
            .unwrap_or_else(|e| panic!("setting up {requested} Hz: {e}"));
        assert!(
            freq_is_nearly_equal(actual, requested),
            "asked for {requested} Hz and got {actual} Hz"
        );
        println!(
            "  {:>8.3} MHz: adc {:>8.3} MHz, vco {:>8.3} MHz",
            actual / 1e6,
            codec.adc_freq() / 1e6,
            codec.bbpll_freq() / 1e6
        );
    }
}
