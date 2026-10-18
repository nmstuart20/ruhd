//! Opening a B2xx, with compatibility negotiated before anything is configured.
//!
//! # Order of operations
//!
//! The first two things this does to a device are read its firmware
//! compatibility number and read its FPGA compatibility major and minor.
//! Nothing else — no register writes, no codec reset, no clock setup — happens
//! until [`negotiate`] has said which [`FpgaCompat`] the rest of the session
//! runs on. That ordering is the whole point:
//!
//! - The register interface those later writes would use is exactly what the
//!   compatibility major identifies, so configuring first and checking second
//!   means poking a register map that may not be there.
//! - A device that turns out to need a reload is one that must not have been
//!   half-configured first.
//! - The numbers are the diagnostic. `b200_impl`'s constructor throws from
//!   three different places part-way through construction, by which point what
//!   it read is unreachable; here the report survives into
//!   [`B200Session::report`] and into [`checkup`].
//!
//! ```text
//!   probe            -> which devices are attached, and in what state
//!   Fx3Iface::open   -> the control endpoint
//!   CompatReport     -> fw compat, fpga compat major/minor, eeprom
//!   negotiate        -> Proceed | Reload | Unsupported
//!     Reload + --auto-flash -> usb::configure, then read and negotiate again
//!   RadioCtrl::open  -> the settings bus, on the negotiated shim
//! ```
//!
//! # Firmware is not loaded here
//!
//! A device that has no firmware running is reported rather than fixed. Loading
//! firmware makes the device leave the USB bus and come back with a different
//! serial, manufacturer and bus address (see `NOTES.md`), which is a
//! re-enumeration this API would have to hide and `ruhd init` already does
//! explicitly. Opening a device should not be a thing that makes it disappear.

use std::fmt;
use std::sync::Arc;
use std::time::Duration;

use ad936x::{Ad936x, ProductId};
use tracing::{debug, info, warn};
use uhd_core::{Error, Result};
use uhd_transport::UsbEndpoint;
use uhd_transport::usb::{
    B2xxProduct, CompatReport, DeviceState, Fx3Iface, UsbFilter, UsbProbe, probe_filtered,
};

use crate::atr::GpioAtr;
use crate::codec;
use crate::compat::{FpgaCompat, FpgaShim};
use crate::misc::MiscGpio;
use crate::policy::{
    DeviceCompat, ImageCandidate, ImageCatalog, UhdImagesCatalog, Verdict, negotiate,
};
use crate::radio_ctrl::RadioCtrl;
use crate::spi::{CodecSpi, LocalSpi};
use crate::time_core::TimeCore;

/// How a caller wants [`open`] to behave.
#[derive(Debug, Clone, Default)]
#[non_exhaustive]
pub struct OpenOptions {
    /// Only the device with this serial. Required when more than one B2xx is
    /// attached.
    pub serial: Option<String>,
    /// Perform an offered FPGA reload rather than reporting it.
    ///
    /// Off by default because it writes to the device, and the caller that
    /// wants it — a bring-up tool, a CI job — is not the same caller as a
    /// streaming application that should fail loudly instead.
    pub auto_flash: bool,
}

impl OpenOptions {
    /// Default options.
    #[must_use]
    pub fn new() -> Self {
        Self::default()
    }

    /// Restricts the search to one serial.
    #[must_use]
    pub fn with_serial(mut self, serial: impl Into<String>) -> Self {
        self.serial = Some(serial.into());
        self
    }

    /// Allows [`open`] to load a cached in-window image.
    #[must_use]
    pub fn with_auto_flash(mut self, auto_flash: bool) -> Self {
        self.auto_flash = auto_flash;
        self
    }
}

/// An FPGA image [`open`] loaded on its way past.
#[derive(Debug, Clone)]
#[non_exhaustive]
pub struct Flashed {
    /// Which manifest entry was loaded.
    pub image: ImageCandidate,
    /// Bytes pushed.
    pub bytes: usize,
    /// How long the load took.
    pub took: Duration,
    /// The compatibility the device reported before the load.
    pub was: String,
}

impl fmt::Display for Flashed {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(
            f,
            "loaded {} ({} bytes in {} ms); before the load, {}",
            self.image,
            self.bytes,
            self.took.as_millis(),
            self.was
        )
    }
}

/// An open B2xx, running on a known FPGA compatibility major.
///
/// Holding one of these is proof that negotiation succeeded: there is no way to
/// construct it without a [`Verdict::Proceed`].
#[derive(Debug)]
pub struct B200Session {
    endpoint: UsbEndpoint,
    product: B2xxProduct,
    compat: FpgaCompat,
    report: CompatReport,
    iface: Fx3Iface,
    ctrl: RadioCtrl,
    spi: Arc<LocalSpi>,
    misc: MiscGpio,
    codec: Ad936x,
    flashed: Option<Flashed>,
}

impl B200Session {
    /// Which device this is.
    #[must_use]
    pub fn endpoint(&self) -> &UsbEndpoint {
        &self.endpoint
    }

    /// Which board this is.
    #[must_use]
    pub fn product(&self) -> B2xxProduct {
        self.product
    }

    /// The negotiated FPGA compatibility major.
    #[must_use]
    pub fn compat(&self) -> FpgaCompat {
        self.compat
    }

    /// The behaviour shim for that major. Everything version-dependent below
    /// this point asks this rather than testing the product.
    #[must_use]
    pub fn shim(&self) -> &'static dyn FpgaShim {
        self.compat.shim()
    }

    /// Every version number the device reported, as of the last read.
    ///
    /// After an auto-flash this is the report taken *after* the load, so it
    /// describes the device as it now stands.
    #[must_use]
    pub fn report(&self) -> &CompatReport {
        &self.report
    }

    /// The FX3 control endpoint.
    #[must_use]
    pub fn iface(&self) -> &Fx3Iface {
        &self.iface
    }

    /// The settings bus.
    #[must_use]
    pub fn ctrl(&self) -> &RadioCtrl {
        &self.ctrl
    }

    /// The FPGA image loaded during this open, if one was.
    #[must_use]
    pub fn flashed(&self) -> Option<&Flashed> {
        self.flashed.as_ref()
    }

    /// The board's SPI master, shared between the codec and the reference PLL.
    #[must_use]
    pub fn spi(&self) -> &Arc<LocalSpi> {
        &self.spi
    }

    /// The board GPIO word: codec reset, reference select, band select.
    #[must_use]
    pub fn misc(&self) -> &MiscGpio {
        &self.misc
    }

    /// The AD936x transceiver, reached over [`B200Session::spi`].
    #[must_use]
    pub fn codec(&self) -> &Ad936x {
        &self.codec
    }

    /// The ATR bank of one radio chain.
    ///
    /// Answers nothing until the codec is clocking — see the
    /// [crate docs](crate).
    ///
    /// # Errors
    ///
    /// [`Error::Device`] if this build has no such chain.
    pub fn atr(&self, radio: usize) -> Result<GpioAtr> {
        Ok(GpioAtr::radio(Box::new(self.ctrl.radio(radio)?)))
    }

    /// The front-panel GPIO bank of one radio chain.
    ///
    /// # Errors
    ///
    /// As [`B200Session::atr`].
    pub fn front_panel_gpio(&self, radio: usize) -> Result<GpioAtr> {
        Ok(GpioAtr::front_panel(Box::new(self.ctrl.radio(radio)?)))
    }

    /// The time core of one radio chain.
    ///
    /// The returned core is set to a tick rate of 1 and has to be told the real
    /// one — see [`TimeCore::set_tick_rate`].
    ///
    /// # Errors
    ///
    /// As [`B200Session::atr`].
    pub fn time_core(&self, radio: usize) -> Result<TimeCore> {
        Ok(TimeCore::new(Box::new(self.ctrl.radio(radio)?)))
    }

    /// Brings the AD936x out of reset and reads its product ID.
    ///
    /// This is the bring-up gate for everything below the settings bus, and it
    /// is the earliest point at which anything about the *radio* has been
    /// proven. Passing it means five things are simultaneously true: the FPGA is
    /// configured, the settings bus reaches the local core, `spi_core_3000` is
    /// where the shim says it is and is clocking at a rate the part can follow,
    /// the board's codec reset line is under control, and there is a live
    /// AD936x on the other end answering with its own identity.
    ///
    /// It needs nothing from the radio cores, which is what makes it runnable
    /// today: the SPI master and the reset line are both on the local core, and
    /// the radio cores will not answer until this has been followed by the
    /// codec initialisation that starts their clock.
    ///
    /// The sequence is `b200_impl`'s, in order:
    ///
    /// 1. `reset_codec()` — pulse the board's reset pin through the misc word;
    /// 2. `poke8(0x000, 1)` then `0` — the part's own soft reset, over SPI;
    /// 3. `peek8(0x037)` — the product ID, masked and compared.
    ///
    /// # Errors
    ///
    /// [`Error::Device`] if the readback is not an AD936x's, carrying the
    /// diagnosis; [`Error::Transport`] if the settings bus does not answer, which
    /// on an unconfigured FPGA is the full acknowledgement timeout.
    pub fn codec_gate(&self) -> Result<CodecGate> {
        self.misc.reset_codec()?;
        self.codec.reset()?;
        let product_id = self.codec.identify()?;
        info!(
            endpoint = %self.endpoint,
            %product_id,
            "AD936x answered over spi"
        );
        Ok(CodecGate {
            product_id,
            spi_rate: crate::spi::AD9361_SPI_RATE,
        })
    }

    /// How many radio chains the FPGA build reports.
    ///
    /// The second byte of the core status register, bounded by what the shim
    /// says the build family can have. `b200_impl` reads the same field and
    /// asserts the same bounds.
    ///
    /// # Errors
    ///
    /// [`Error::Transport`] if the settings bus does not answer;
    /// [`Error::Device`] if the count is outside `1..=`[`FpgaShim::max_radio_chains`],
    /// which means the readback is not what it claims to be.
    pub fn num_radio_chains(&self) -> Result<usize> {
        use uhd_transport::RegisterIface;

        let status = self.ctrl.peek32(self.shim().core_regs().rb32_status)?;
        let count = ((status >> 8) & 0xff) as usize;
        let max = self.shim().max_radio_chains();
        if count == 0 || count > max {
            return Err(Error::Device(format!(
                "the fpga reports {count} radio chains, but an fpga compat {} build has \
                 between 1 and {max}; the status readback is {status:#010x}",
                self.compat.major()
            )));
        }
        Ok(count)
    }

    /// Brings the radio up from cold: resets and initialises the AD936x, sets
    /// the master clock rate, and readies every radio core.
    ///
    /// This is the part of `b200_impl`'s constructor that comes after the
    /// settings bus is up: `reset_codec`, `init_codec`, the "enable something"
    /// branch of `update_enables`, the default tick rate, the ATR banks in
    /// their idle state, and the device time set to zero. Returns the master
    /// clock rate the codec actually produced, which is the tick rate every
    /// timestamp is counted in.
    ///
    /// Resetting the codec discards every tuning, gain and rate set before
    /// the call. Tune after it, not before.
    ///
    /// # Errors
    ///
    /// Whatever the codec initialisation or the settings bus reports.
    pub fn init_radio(&self, master_clock_rate: f64) -> Result<f64> {
        self.misc.reset_codec()?;
        self.codec.init()?;
        self.codec.set_active_chains(true, false, true, false)?;
        let tick_rate = self.set_master_clock_rate(master_clock_rate)?;
        for radio in 0..self.num_radio_chains()? {
            let side = crate::rx_stream::side_for(radio, self.misc.swap_atr());
            let atr = self.atr(radio)?;
            atr.enable_atr()?;
            atr.set_b2xx_states(side, false, false, true)?;
            let time = self.time_core(radio)?;
            time.set_tick_rate(tick_rate)?;
            time.set_time_now(uhd_core::TimeSpec::default())?;
        }
        info!(
            endpoint = %self.endpoint,
            tick_rate,
            "radio initialised"
        );
        Ok(tick_rate)
    }

    /// Changes the master clock rate, returning the rate the codec's BBPLL
    /// actually produced.
    ///
    /// Streams opened before the change keep the old rate's timestamps and
    /// decimation; reopen them.
    ///
    /// # Errors
    ///
    /// [`Error::OutOfRange`] above the AD936x's 61.44 MHz; otherwise whatever
    /// the codec reports.
    pub fn set_master_clock_rate(&self, rate: f64) -> Result<f64> {
        self.codec.set_sample_rate(rate)?;
        Ok(self.codec.clock_rate())
    }

    /// Opens a single-channel receive stream.
    ///
    /// Programs the radio's DSP chain and framer for `options`, enables the
    /// receive chain in the codec, switches the front end to receive on the
    /// chosen port, and queues the USB transfers. Samples do not flow until a
    /// stream command is issued.
    ///
    /// The codec must already be clocking — see [`B200Session::init_radio`] —
    /// and tuning, gain and bandwidth are the codec's business, not the
    /// stream's.
    ///
    /// `args.args` takes UHD's `spp`, `recv_frame_size` and `num_recv_frames`.
    ///
    /// # Errors
    ///
    /// [`Error::NotSupported`] for more than one channel or a wire format other
    /// than `sc16`; [`Error::Usb`] if the data interface is already claimed;
    /// [`Error::Device`] if the codec is not clocking.
    pub fn rx_stream(
        &self,
        options: &crate::rx_stream::RxOptions,
        args: &uhd_core::stream::StreamArgs,
    ) -> Result<crate::rx_stream::B200RxStreamer> {
        crate::rx_stream::open(self, options, args)
    }
}

/// The master clock rate UHD would pick for a sample rate.
///
/// `ad936x_manager::get_auto_tick_rate`: the sample rate times the largest
/// power of two that keeps it at or below the AD936x's maximum divided among
/// the channels, so the DSP chain decimates by a power of two and both
/// half-band filters are in use where they can be. A multiplier of exactly two
/// is dropped to one when the sample rate is already a legal clock rate.
///
/// # Errors
///
/// [`Error::OutOfRange`] if the sample rate exceeds the per-channel maximum or
/// no legal clock rate results.
pub fn auto_master_clock_rate(rate: f64, num_chans: usize) -> Result<f64> {
    let max = ad936x::MAX_CLOCK_RATE / num_chans.max(1) as f64;
    let min = ad936x::MIN_CLOCK_RATE;
    let out_of_range = |value: f64| Error::OutOfRange {
        what: "sample rate".into(),
        value,
        start: 0.0,
        stop: max,
    };
    if !rate.is_finite() || rate <= 0.0 || rate > max + 1.0 {
        return Err(out_of_range(rate));
    }
    let lcm = rate.round().max(1.0);
    let mut multiplier = 1u64 << ((max / lcm).log2().max(0.0) as u32);
    if multiplier == 2 && lcm >= min {
        multiplier = 1;
    }
    let tick = lcm * multiplier as f64;
    if tick < min - 1.0 || tick > max + 1.0 {
        return Err(out_of_range(rate));
    }
    Ok(tick)
}

/// Finds one B2xx and opens it.
///
/// Uses the built-in image manifest and the user's image cache. See
/// [`open_probed`] for the form that takes an explicit catalogue.
///
/// # Errors
///
/// [`Error::Usb`] if no device matches, or if more than one does and no serial
/// was given. Otherwise as [`open_probed`].
pub fn open(options: &OpenOptions) -> Result<B200Session> {
    let catalog = UhdImagesCatalog::builtin()?;
    let probe = select(options.serial.as_deref())?;
    open_probed(&probe, options, &catalog, |_sent, _total| {})
}

/// Opens a device that has already been found.
///
/// `progress` is called with `(bytes_sent, total_bytes)` during an auto-flash,
/// and not at all otherwise.
///
/// # Errors
///
/// [`Error::Firmware`] if the device has no firmware running, if negotiation
/// returns [`Verdict::Unsupported`], or if it returns [`Verdict::Reload`] and
/// `options.auto_flash` is not set — in every case carrying the diagnostic.
/// [`Error::Device`] if the board's product cannot be identified. [`Error::Usb`]
/// if the device cannot be opened or a transfer fails.
pub fn open_probed(
    probe: &UsbProbe,
    options: &OpenOptions,
    catalog: &dyn ImageCatalog,
    progress: impl FnMut(usize, usize),
) -> Result<B200Session> {
    if probe.needs_firmware() {
        return Err(Error::Firmware(format!(
            "{} has no firmware running ({}); run `ruhd init` first",
            probe.endpoint, probe.state
        )));
    }

    let iface = Fx3Iface::open(probe)?;

    // Before anything else is done to this device.
    let report = CompatReport::read(&iface, probe)?;
    let device = DeviceCompat::from_report(&report)?;
    debug!(
        endpoint = %probe.endpoint,
        firmware = %report.firmware,
        fpga = %report.fpga,
        "read compatibility before configuring anything"
    );

    let (compat, report, flashed) = match negotiate(&device, catalog) {
        Verdict::Proceed { compat } => (compat, report, None),
        Verdict::Reload(reload) if options.auto_flash => {
            let flashed = flash(&iface, &reload, progress)?;
            // Whatever the loader claims, the device's own answer decides. A
            // second negotiation on a fresh read is the only thing that proves
            // the reload worked.
            let report = CompatReport::read(&iface, probe)?;
            let device = DeviceCompat::from_report(&report)?;
            match negotiate(&device, catalog) {
                Verdict::Proceed { compat } => (compat, report, Some(flashed)),
                after => {
                    return Err(Error::Firmware(format!(
                        "loaded {} into {}, and the device still reports a build this \
                         driver cannot use: {}",
                        flashed.image,
                        probe.endpoint,
                        describe(&after)
                    )));
                }
            }
        }
        Verdict::Reload(reload) => {
            return Err(Error::Firmware(format!("{}: {reload}", probe.endpoint)));
        }
        Verdict::Unsupported(diagnosis) => {
            return Err(Error::Firmware(format!("{}: {diagnosis}", probe.endpoint)));
        }
    };

    let ctrl = RadioCtrl::open(&iface, compat)?;

    // All three are local-core peripherals and all three are built write-free,
    // so opening a device still does not change it. `swap_atr` needs the board
    // revision, which is why this comes after the EEPROM read rather than
    // before it.
    let spi = Arc::new(LocalSpi::on_local_core(ctrl.local(), compat));
    let misc = MiscGpio::new(
        Box::new(ctrl.local()),
        compat,
        device.product,
        report.eeprom.revision,
    );
    let codec = Ad936x::new(
        Arc::new(CodecSpi::new(Arc::clone(&spi))),
        codec::client_params(device.product),
    );

    info!(
        endpoint = %probe.endpoint,
        product = %device.product,
        compat = compat.major(),
        shim = compat.shim().codename(),
        "opened b2xx on a negotiated fpga compat"
    );

    Ok(B200Session {
        endpoint: probe.endpoint.clone(),
        product: device.product,
        compat,
        report,
        iface,
        ctrl,
        spi,
        misc,
        codec,
        flashed,
    })
}

/// Successful result of the codec bring-up gate.
///
/// See [`B200Session::codec_gate`].
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct CodecGate {
    /// What the part reported at register `0x037`.
    pub product_id: ProductId,
    /// The SPI clock it answered at, in Hz.
    pub spi_rate: f64,
}

impl fmt::Display for CodecGate {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(
            f,
            "{} answered at {:.0} kHz on spi",
            self.product_id,
            self.spi_rate / 1e3
        )
    }
}

/// What [`checkup`] found out about one device.
///
/// Never a failure: `doctor` exists to describe devices that do not work, so a
/// device that cannot be opened has to come back as a description rather than
/// as an error that stops the other devices being described.
#[derive(Debug)]
#[non_exhaustive]
pub struct Checkup {
    /// Which device this is.
    pub endpoint: UsbEndpoint,
    /// What discovery classified it as.
    pub state: DeviceState,
    /// The USB serial, when the device has one.
    pub serial: Option<String>,
    /// Every version number it reported, if it could be asked.
    pub report: Option<CompatReport>,
    /// The verdict, if there was enough to reach one.
    pub verdict: Option<Verdict>,
    /// Why there is no report or no verdict.
    pub trouble: Option<String>,
}

impl Checkup {
    /// One line naming the outcome.
    #[must_use]
    pub fn headline(&self) -> String {
        match (&self.verdict, &self.trouble) {
            (Some(verdict), _) => verdict.headline(),
            (None, Some(trouble)) => format!("cannot say — {trouble}"),
            (None, None) => "cannot say".to_owned(),
        }
    }

    /// The recommended fix, if the device needs one.
    #[must_use]
    pub fn recommended_fix(&self) -> Option<String> {
        match (&self.verdict, self.state) {
            (Some(verdict), _) => verdict.recommended_fix(),
            (None, DeviceState::NeedsFirmware) => Some("run `ruhd init`".to_owned()),
            (None, DeviceState::Uninitialized) => Some(
                "the eeprom has not been programmed; recover the board with \
                 `b2xx_fx3_utils` before anything else"
                    .to_owned(),
            ),
            (None, _) => None,
        }
    }
}

/// Reads a device's compatibility numbers and says what it makes of them,
/// without changing anything.
///
/// The read-only half of [`open_probed`]: same two reads, same policy, no
/// settings-bus session and no reload. This is what `ruhd doctor` prints.
#[must_use]
pub fn checkup(probe: &UsbProbe, catalog: &dyn ImageCatalog) -> Checkup {
    let mut checkup = Checkup {
        endpoint: probe.endpoint.clone(),
        state: probe.state,
        serial: probe.serial().map(str::to_owned),
        report: None,
        verdict: None,
        trouble: None,
    };

    if probe.needs_firmware() {
        checkup.trouble = Some(format!(
            "no firmware is running ({}), so the device has nothing to answer with",
            probe.state
        ));
        return checkup;
    }

    let report = match Fx3Iface::open(probe).and_then(|iface| CompatReport::read(&iface, probe)) {
        Ok(report) => report,
        Err(e) => {
            checkup.trouble = Some(e.to_string());
            return checkup;
        }
    };

    match DeviceCompat::from_report(&report) {
        Ok(device) => checkup.verdict = Some(negotiate(&device, catalog)),
        Err(e) => checkup.trouble = Some(e.to_string()),
    }
    checkup.report = Some(report);
    checkup
}

/// Picks the one device to open, or explains why it cannot.
fn select(serial: Option<&str>) -> Result<UsbProbe> {
    let mut filter = UsbFilter::new();
    filter.serial = serial.map(str::to_owned);

    let mut devices = probe_filtered(&filter)?;
    if devices.is_empty() {
        return Err(Error::Usb(match serial {
            Some(serial) => format!("no b2xx device with serial `{serial}` is attached"),
            None => "no b2xx devices found on the usb bus; run `ruhd probe usb`".into(),
        }));
    }
    if devices.len() > 1 {
        let mut message = format!("{} devices are attached; pass a serial:\n", devices.len());
        for device in &devices {
            message.push_str(&format!("  {device}\n"));
        }
        return Err(Error::Usb(message));
    }
    Ok(devices.remove(0))
}

/// Pushes a reload's cached image.
///
/// `force` is true: the FX3's stored hash is a note the host left, and a device
/// that reports an out-of-window compatibility number while holding a note
/// saying the right image is loaded is precisely the case where the note is
/// wrong.
fn flash(
    iface: &Fx3Iface,
    reload: &crate::policy::Reload,
    progress: impl FnMut(usize, usize),
) -> Result<Flashed> {
    let path = reload.path()?;
    let image = uhd_transport::usb::FpgaImage::load(path)?;
    warn!(
        path = %path.display(),
        target = reload.target.major(),
        "reloading the fpga to reach a supported compatibility major"
    );

    let outcome = uhd_transport::usb::configure(iface, &image, true, progress)?;
    Ok(Flashed {
        image: reload.image.clone(),
        bytes: outcome.bytes,
        took: outcome.took,
        was: reload.found.to_string(),
    })
}

/// Renders a verdict for an error message.
fn describe(verdict: &Verdict) -> String {
    match verdict {
        Verdict::Unsupported(diagnosis) => diagnosis.to_string(),
        other => other.headline(),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn auto_master_clock_rate_matches_ad936x_manager() {
        // 1 MS/s: 61.44 / 1 -> 2^5, so 32 MHz and a decimation of 32.
        assert_eq!(auto_master_clock_rate(1e6, 1).unwrap(), 32e6);
        // 16 MS/s: the multiplier would be 2, which is dropped.
        assert_eq!(auto_master_clock_rate(16e6, 1).unwrap(), 16e6);
        assert_eq!(auto_master_clock_rate(2e6, 1).unwrap(), 32e6);
        assert_eq!(auto_master_clock_rate(61.44e6, 1).unwrap(), 61.44e6);
        // Two channels halve the ceiling.
        assert_eq!(auto_master_clock_rate(16e6, 2).unwrap(), 16e6);
        assert!(auto_master_clock_rate(40e6, 2).is_err());
        assert!(auto_master_clock_rate(0.0, 1).is_err());
    }

    #[test]
    fn options_build_up() {
        let options = OpenOptions::new()
            .with_serial("3168B41")
            .with_auto_flash(true);
        assert_eq!(options.serial.as_deref(), Some("3168B41"));
        assert!(options.auto_flash);

        // Auto-flash is off unless asked for: opening a device must not write
        // to it by default.
        assert!(!OpenOptions::new().auto_flash);
    }

    #[test]
    fn a_checkup_with_no_firmware_still_recommends_something() {
        let checkup = Checkup {
            endpoint: UsbEndpoint::default(),
            state: DeviceState::NeedsFirmware,
            serial: None,
            report: None,
            verdict: None,
            trouble: Some("no firmware is running".into()),
        };
        assert!(checkup.headline().contains("no firmware"));
        assert_eq!(
            checkup.recommended_fix().as_deref(),
            Some("run `ruhd init`")
        );
    }
}
