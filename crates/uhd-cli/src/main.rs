//! `ruhd` — command-line front end, the rough equivalent of `uhd_find_devices`,
//! `uhd_usrp_probe` and `uhd_images_downloader`.

mod images;

use std::path::PathBuf;
use std::process::ExitCode;
use std::time::Duration;

use clap::{Args, Parser, Subcommand};
use tracing::debug;
use tracing_subscriber::EnvFilter;
use uhd_core::prelude::*;
use uhd_core::{DeviceArgs, Error, SensorScope, registry};
use uhd_transport::images as image_paths;
use uhd_transport::usb;

#[derive(Debug, Parser)]
#[command(name = "ruhd", version, about, long_about = None)]
struct Cli {
    /// Log level: error, warn, info, debug, trace. `RUST_LOG` overrides this.
    #[arg(long, short, global = true, default_value = "warn")]
    log_level: String,

    #[command(subcommand)]
    command: Command,
}

#[derive(Debug, Subcommand)]
enum Command {
    /// List attached devices.
    Find {
        /// Device arguments to narrow the search, e.g. `type=b200,serial=30A1B2C`.
        #[arg(long, default_value = "")]
        args: String,
    },
    /// Inspect what is attached, at the USB level or the device level.
    ///
    /// With no subcommand this is `probe usb`, so `ruhd probe --full` is the
    /// short way to ask an attached board for everything it knows about
    /// itself.
    Probe {
        #[command(subcommand)]
        target: Option<ProbeTarget>,

        #[command(flatten)]
        usb: UsbProbeArgs,
    },
    /// Load FX3 firmware and the FPGA bitstream into a B2xx.
    ///
    /// A B2xx has no flash: the FX3 firmware *and* the FPGA image are volatile
    /// and have to be pushed over USB on every plug-in. This takes a board from
    /// the state it enumerates in to one that answers vendor requests and has a
    /// live settings bus, which is the precondition for everything else.
    ///
    /// Idempotent — firmware that is already running is left alone unless
    /// `--force` is given, and an FPGA already holding this image is left alone
    /// unless `--force-fpga` is.
    Init(InitArgs),
    /// Reset the FX3, discarding the firmware it is running.
    ///
    /// The device drops off the bus and comes back needing firmware again —
    /// the state it is in after a power cycle. The equivalent of
    /// `b2xx_fx3_utils --reset-device`, and the way to recover a device that
    /// has got itself wedged without reaching behind the desk for the cable.
    Reset {
        /// Only the device with this serial.
        #[arg(long)]
        serial: Option<String>,
    },
    /// Fetch and inspect the UHD images package.
    Images {
        #[command(subcommand)]
        action: ImagesAction,
    },
    /// List the device families this build knows about.
    Backends,
}

#[derive(Debug, Subcommand)]
enum ProbeTarget {
    /// Enumerate the USB bus and classify every B2xx and unprogrammed FX3.
    ///
    /// This works before any backend exists and before firmware is loaded,
    /// which is what makes it the first thing to run when a device is not
    /// showing up.
    Usb(UsbProbeArgs),
    /// Open a device through the registry and print what it can do.
    Device {
        /// Device arguments identifying exactly one device.
        #[arg(long, default_value = "")]
        args: String,
    },
}

#[derive(Debug, Args)]
struct UsbProbeArgs {
    /// Only devices with this USB vendor ID, e.g. `2500` or `0x2500`.
    #[arg(long, value_parser = parse_hex_u16)]
    vid: Option<u16>,
    /// Only devices with this USB product ID, e.g. `0022`.
    #[arg(long, value_parser = parse_hex_u16)]
    pid: Option<u16>,
    /// Only the device with this serial. Case-insensitive.
    #[arg(long)]
    serial: Option<String>,
    /// Skip the open-and-close test that reports whether this user has
    /// permission to use each device.
    #[arg(long)]
    no_access_check: bool,
    /// Open each running device and print what it says about itself: firmware
    /// and FPGA compatibility numbers, EEPROM contents, USB speed, FX3 state.
    ///
    /// This needs permission to open the device, and it needs firmware to be
    /// running — a board that has not been through `ruhd init` has nothing to
    /// answer with.
    #[arg(long)]
    full: bool,
}

#[derive(Debug, Args)]
struct InitArgs {
    /// Only the device with this serial. Required when more than one B2xx is
    /// attached, because the serial is also what identifies the device again
    /// after it re-enumerates.
    #[arg(long)]
    serial: Option<String>,
    /// Firmware image to load. Defaults to `usrp_b200_fw.hex` from the images
    /// package; a path here overrides the search entirely, for a locally built
    /// image.
    #[arg(long, value_name = "PATH")]
    fw: Option<PathBuf>,
    /// FPGA bitstream to load. Defaults to the image the board's product code
    /// calls for; a path here overrides the search entirely.
    #[arg(long, value_name = "PATH")]
    fpga: Option<PathBuf>,
    /// Load firmware only, leaving the FPGA unconfigured.
    #[arg(long)]
    no_fpga: bool,
    /// Reload firmware even if the device already has some running. Requires
    /// resetting the FX3 first, since firmware cannot be overwritten in place.
    #[arg(long)]
    force: bool,
    /// Reconfigure the FPGA even when it already holds this image.
    ///
    /// The skip is decided from a hash the host last stored on the FX3, so it
    /// is a claim rather than evidence; this is the way to overrule it.
    #[arg(long)]
    force_fpga: bool,
    /// How long to wait for the device to re-enumerate, in milliseconds.
    #[arg(long, default_value_t = 3000)]
    timeout_ms: u64,
}

#[derive(Debug, Subcommand)]
enum ImagesAction {
    /// Report where images are being searched for and whether they are current.
    Status,
    /// Download the B2xx images package.
    Download(DownloadArgs),
}

#[derive(Debug, Args)]
struct DownloadArgs {
    /// Only this manifest target, e.g. `b2xx_common_fw_default`. Repeatable;
    /// the default is every B2xx target.
    #[arg(long, value_name = "NAME")]
    target: Vec<String>,
    /// Where to install. Defaults to a user-owned directory that is already
    /// first on the image search path, so no privileges are needed.
    #[arg(long, value_name = "PATH")]
    dir: Option<PathBuf>,
    /// Base URL to fetch from. Also settable with `UHD_IMAGES_BASE_URL`.
    #[arg(long, value_name = "URL")]
    base_url: Option<String>,
    /// Re-download targets the inventory already lists as current.
    #[arg(long)]
    force: bool,
}

/// Parses a VID or PID, with or without a `0x` prefix. Always hex, matching
/// UHD's `hexstr_cast`: a bare `2500` is 0x2500, not 2500 decimal.
fn parse_hex_u16(s: &str) -> std::result::Result<u16, String> {
    let digits = s
        .strip_prefix("0x")
        .or_else(|| s.strip_prefix("0X"))
        .unwrap_or(s);
    u16::from_str_radix(digits, 16).map_err(|e| format!("`{s}` is not a 16-bit hex value: {e}"))
}

fn main() -> ExitCode {
    let cli = Cli::parse();

    tracing_subscriber::fmt()
        .with_env_filter(
            EnvFilter::try_from_default_env()
                .unwrap_or_else(|_| EnvFilter::new(cli.log_level.clone())),
        )
        .with_writer(std::io::stderr)
        .init();

    register_backends();

    match run(&cli.command) {
        Ok(()) => ExitCode::SUCCESS,
        Err(e) => {
            eprintln!("error: {e}");
            ExitCode::FAILURE
        }
    }
}

/// Registers every backend compiled into this binary.
///
/// Empty until `uhd-b200` gains a factory; at that point this becomes
/// `uhd_b200::register();` and nothing else in the CLI changes.
fn register_backends() {}

fn run(command: &Command) -> Result<()> {
    debug!(?command, backends = ?registry::registered_types(), "dispatching");
    match command {
        Command::Find { args } => find(args),
        Command::Probe { target, usb } => match target {
            Some(ProbeTarget::Usb(args)) => probe_usb(args),
            Some(ProbeTarget::Device { args }) => probe_device(args),
            None => probe_usb(usb),
        },
        Command::Init(args) => init(args),
        Command::Reset { serial } => reset(serial.as_deref()),
        Command::Images { action } => match action {
            ImagesAction::Status => images_status(),
            ImagesAction::Download(args) => images_download(args),
        },
        Command::Backends => {
            let types = registry::registered_types();
            if types.is_empty() {
                println!("no device backends are registered in this build");
            } else {
                for t in types {
                    println!("{t}");
                }
            }
            Ok(())
        }
    }
}

fn find(args: &str) -> Result<()> {
    let hint: DeviceArgs = args.parse()?;
    let devices = registry::find(&hint)?;

    if devices.is_empty() {
        println!("no devices found");
        if registry::registered_types().is_empty() {
            println!("(no backends are registered in this build yet)");
        }
        return Ok(());
    }

    for (i, info) in devices.iter().enumerate() {
        println!("Device {i}");
        println!("  type:    {}", info.device_type);
        println!("  product: {}", info.product);
        println!("  serial:  {}", info.serial);
        if !info.name.is_empty() {
            println!("  name:    {}", info.name);
        }
        if let Some(revision) = &info.revision {
            println!("  rev:     {revision}");
        }
        println!("  args:    {}", info.args);
    }
    Ok(())
}

/// Prints what USB discovery found, one paragraph per device.
///
/// Deliberately says something useful about a device that is *not* ready: the
/// common case for a freshly plugged B2xx is that firmware has not been loaded,
/// and the second most common is that udev has not been set up. Both look like
/// "it does not work" from any higher layer, so they get spelled out here.
fn probe_usb(args: &UsbProbeArgs) -> Result<()> {
    let mut filter = usb::UsbFilter::new();
    filter.vid = args.vid;
    filter.pid = args.pid;
    filter.serial.clone_from(&args.serial);

    let devices = usb::probe_filtered(&filter)?;
    if devices.is_empty() {
        println!("no b2xx devices found on the usb bus");
        return Ok(());
    }

    for (i, device) in devices.iter().enumerate() {
        if i > 0 {
            println!();
        }
        println!("Device {i}");
        println!("  product: {}", device.describe());
        println!("  state:   {}", device.state);
        println!(
            "  serial:  {}",
            device.serial().unwrap_or("<none — EEPROM not programmed>")
        );
        println!("  usb id:  {:04x}:{:04x}", device.vid(), device.pid());
        println!("  bus:     {} address {}", device.bus_id, device.address);
        if let Some(manufacturer) = &device.manufacturer {
            println!("  vendor:  {manufacturer}");
        }
        if let Some(product) = &device.product_string {
            println!("  descr:   {product}");
        }

        if !args.no_access_check {
            match device.check_access() {
                Ok(()) => println!("  access:  ok"),
                Err(e) => {
                    println!("  access:  {e}");
                    println!("           see the udev section in README.md");
                }
            }
        }

        match device.state {
            usb::DeviceState::Uninitialized => println!(
                "  next:    program the eeprom, then load {}",
                usb::B200_FW_IMAGE
            ),
            usb::DeviceState::NeedsFirmware => {
                println!("  next:    load {}", usb::B200_FW_IMAGE);
            }
            _ => {}
        }

        if args.full {
            print_full(device);
        }
    }
    Ok(())
}

/// Opens a device and prints everything it says about itself.
///
/// Never fails the whole command: `probe` is the tool you reach for when
/// something is wrong, so one device that cannot be opened must not stop the
/// others being described. The reason is printed in place of the block.
fn print_full(device: &usb::UsbProbe) {
    println!();
    if device.needs_firmware() {
        println!("  detail:  <no firmware running; run `ruhd init` first>");
        return;
    }

    let report = usb::Fx3Iface::open(device).and_then(|iface| {
        // The compat read claims the bulk interfaces on its way past. Dropping
        // the interface as soon as the report is built hands the device back,
        // so a `uhd_usrp_probe` straight afterwards is not locked out.
        usb::CompatReport::read(&iface, device)
    });

    match report {
        Ok(report) => print_compat(&report),
        Err(e) => println!("  detail:  <{e}>"),
    }
}

/// The body of `probe --full` and the tail of `init`: one device's own account
/// of itself, versions first.
fn print_compat(report: &usb::CompatReport) {
    println!(
        "  firmware: {} (expected {})",
        report.firmware, report.expected_firmware
    );
    println!("  usb:      operating over usb {}", report.usb_speed);
    println!("  fx3:      {}", report.fx3_state);
    match report.expected_fpga_major {
        Some(expected) => println!("  fpga:     {} (expected major {expected})", report.fpga),
        None => println!("  fpga:     {}", report.fpga),
    }

    let eeprom = &report.eeprom;
    match eeprom.layout {
        Some(layout) => println!("  eeprom:   {layout} ({} bytes)", eeprom.raw.len()),
        None => println!(
            "  eeprom:   <unrecognised signature {:#010x}>",
            eeprom.signature
        ),
    }
    if eeprom.layout.is_some() {
        println!("    serial:   {}", opt(eeprom.serial.as_deref()));
        println!("    name:     {}", opt(eeprom.name.as_deref()));
        println!(
            "    product:  {}",
            match (eeprom.product_code, eeprom.product()) {
                (Some(code), Some(product)) => format!("{code:#06x} ({product})"),
                (Some(code), None) => format!("{code:#06x} (unrecognised)"),
                (None, _) => "<unset>".to_owned(),
            }
        );
        println!("    revision: {}", opt_u16(eeprom.revision));
        if let (Some(vid), Some(pid)) = (eeprom.vendor_id, eeprom.product_id) {
            println!("    usb id:   {vid:04x}:{pid:04x}");
        }
        if let Some(rev) = eeprom.eeprom_revision {
            println!(
                "    layout:   revision {rev}, compat {}",
                opt_u16(eeprom.eeprom_compat)
            );
        }
        if let Some(magic) = eeprom.magic {
            println!("    magic:    {magic:#06x}");
        }
    }

    // The departure from UHD in one line of output: the numbers are reported,
    // a mismatch is described, and nothing here decides to refuse the device.
    let problems = report.problems();
    if problems.is_empty() {
        println!("  compat:   every version number this driver checks lines up");
    } else {
        println!(
            "  compat:   {} problem(s), reported not enforced:",
            problems.len()
        );
        for problem in problems {
            println!("            - {problem}");
        }
    }
}

/// A string field, or a marker that it is unset.
fn opt(value: Option<&str>) -> String {
    value.unwrap_or("<unset>").to_owned()
}

/// A numeric field, or a marker that it is unset.
fn opt_u16(value: Option<u16>) -> String {
    value.map_or_else(|| "<unset>".to_owned(), |v| v.to_string())
}

/// Takes one B2xx from wherever it is to firmware-running and verified.
///
/// The gate this command exists for: a board that enumerates as
/// `2500:0022` with Cypress descriptors goes in, and the same board serving
/// Ettus descriptors and answering vendor requests comes out — the same
/// transition `uhd_usrp_probe` performs silently on its way to opening a
/// device.
fn init(args: &InitArgs) -> Result<()> {
    let firmware = resolve_firmware(args.fw.as_deref())?;
    println!("firmware:  {}", firmware.display());

    let image = usb::HexImage::load(&firmware)?;
    println!(
        "           {} records, {} bytes, entry {:#010x}",
        image.records.len(),
        image.payload_len(),
        image.entry_point()?
    );

    let device = select_device(args.serial.as_deref())?;
    println!(
        "device:    {} [{}] {:04x}:{:04x} bus {} address {}",
        device.describe(),
        device.state,
        device.vid(),
        device.pid(),
        device.bus_id,
        device.address
    );

    // Say what is about to happen before it happens: a firmware load makes the
    // device vanish from the bus for a second or two, and an unexplained pause
    // followed by a disappearing device looks like a crash.
    match (device.state, args.force) {
        (usb::DeviceState::Ready, false) => println!("           firmware already running"),
        (usb::DeviceState::Ready, true) => {
            println!("           resetting the fx3 to overwrite running firmware");
        }
        _ => println!("           loading firmware, this takes a moment"),
    }

    let outcome = usb::initialize(
        &device,
        &image,
        args.force,
        Duration::from_millis(args.timeout_ms),
    )?;

    println!();
    println!("Device");
    println!("  product:  {}", outcome.device.describe());
    println!(
        "  serial:   {}",
        outcome.device.serial().unwrap_or("<none>")
    );
    println!(
        "  usb id:   {:04x}:{:04x}",
        outcome.device.vid(),
        outcome.device.pid()
    );
    println!("  state:    {}", outcome.device.state);
    if let Some(manufacturer) = &outcome.device.manufacturer {
        println!("  vendor:   {manufacturer}");
    }
    if let Some(took) = outcome.reenumeration {
        println!("  loaded:   yes, re-enumerated in {} ms", took.as_millis());
    } else {
        println!("  loaded:   no, firmware was already running");
    }

    // Firmware is up, so the FX3 answers vendor requests and the EEPROM is
    // readable — which is what makes the next step possible at all, since on a
    // B200 or B210 the EEPROM is the only thing that says which bitstream to
    // load.
    let iface = usb::Fx3Iface::open(&outcome.device)?;

    if args.no_fpga {
        println!("  fpga:     skipped (--no-fpga)");
    } else {
        load_fpga(&iface, &outcome.device, args)?;
    }

    println!();
    println!("Compatibility");
    let report = usb::CompatReport::read(&iface, &outcome.device)?;
    print_compat(&report);

    Ok(())
}

/// Pushes the FPGA bitstream and resets the GPIF.
///
/// Printed rather than logged because it is the slow part: 4 MB in 512-byte
/// control transfers is a few seconds during which an unadorned `ruhd init`
/// looks hung.
fn load_fpga(iface: &usb::Fx3Iface, device: &usb::UsbProbe, args: &InitArgs) -> Result<()> {
    let path = match &args.fpga {
        Some(path) => {
            if !path.is_file() {
                return Err(Error::Firmware(format!(
                    "no such fpga image: {}",
                    path.display()
                )));
            }
            path.clone()
        }
        None => image_paths::find_image(usb::image_for(device, iface)?)?,
    };

    println!();
    let image = usb::FpgaImage::load(&path)?;
    println!("fpga:      {}", path.display());
    println!(
        "           {} bytes, hash {:#010x}",
        image.len(),
        image.hash
    );

    // A carriage return redraws in place on a terminal and produces one very
    // long line in a log file, so the progress line is only drawn when there is
    // someone watching it. Piped output gets the summary and nothing else.
    let interactive = std::io::IsTerminal::is_terminal(&std::io::stdout());
    let mut last_decile = 0;
    let outcome = usb::configure(iface, &image, args.force_fpga, |sent, total| {
        let decile = sent * 10 / total;
        if interactive && decile != last_decile {
            last_decile = decile;
            print!("\r           loading: {:>3}%", decile * 10);
            let _ = std::io::Write::flush(&mut std::io::stdout());
        }
    })?;

    if outcome.loaded {
        if interactive {
            print!("\r");
        }
        println!(
            "           loaded {} bytes in {} ms, {}-byte chunks; fx3 is {}",
            outcome.bytes,
            outcome.took.as_millis(),
            outcome.chunk_size,
            outcome.state,
        );
    } else {
        println!(
            "           already loaded (the fx3 holds hash {:#010x}); \
             pass --force-fpga to reload",
            outcome.previous_hash
        );
    }
    println!("           gpif reset");
    Ok(())
}

/// Resets the FX3 and waits for it to come back needing firmware.
fn reset(serial: Option<&str>) -> Result<()> {
    let device = select_device(serial)?;
    println!("device:  {device}");

    if device.state != usb::DeviceState::Ready {
        println!("         already has no firmware running; nothing to reset");
        return Ok(());
    }

    let outcome = usb::reset(&device, usb::RESET_TIMEOUT)?;
    println!(
        "         reset; back on the bus in {} ms as {}",
        outcome.1.as_millis(),
        outcome.0
    );
    println!("\nRun `ruhd init` to load firmware again.");
    Ok(())
}

/// Finds the firmware image, and warns if the images package looks stale.
///
/// The staleness check is a warning rather than an error on purpose. An
/// inventory entry is only evidence about images this driver or UHD's
/// downloader installed; a distribution package that shipped images with no
/// `inventory.json` reads as "missing" while being perfectly good. The
/// authoritative check is the compatibility number the device itself reports
/// after the load, which is a hard error — so a genuinely wrong image is
/// caught either way, and a merely unrecognised one does not block anybody.
fn resolve_firmware(override_path: Option<&std::path::Path>) -> Result<PathBuf> {
    if let Some(path) = override_path {
        if !path.is_file() {
            return Err(Error::Firmware(format!(
                "no such firmware image: {}",
                path.display()
            )));
        }
        return Ok(path.to_path_buf());
    }

    let path = image_paths::find_image(usb::B200_FW_IMAGE)?;

    let target = images::FIRMWARE_TARGET;
    match images::status(target) {
        images::Status::UpToDate => {}
        images::Status::Stale(found) => {
            eprintln!(
                "warning: the installed B2xx firmware images are at {found}, but this build \
                 expects\n         {} (UHD {}). Run `{}` to update them.",
                target.repo_hash,
                images::MANIFEST_UHD_VERSION,
                image_paths::DOWNLOAD_COMMAND,
            );
        }
        images::Status::Missing => {
            eprintln!(
                "warning: no inventory records the version of the images in use, so this \
                 cannot\n         confirm they match UHD {}. The firmware compatibility \
                 check below is\n         the real test; run `{}` if it fails.",
                images::MANIFEST_UHD_VERSION,
                image_paths::DOWNLOAD_COMMAND,
            );
        }
    }

    Ok(path)
}

/// Picks the one device to initialise, or explains why it cannot.
fn select_device(serial: Option<&str>) -> Result<usb::UsbProbe> {
    let mut filter = usb::UsbFilter::new();
    filter.serial = serial.map(str::to_owned);

    let mut devices = usb::probe_filtered(&filter)?;

    if devices.is_empty() {
        return Err(Error::Usb(match serial {
            Some(serial) => format!("no b2xx device with serial `{serial}` is attached"),
            None => "no b2xx devices found on the usb bus; run `ruhd probe usb`".into(),
        }));
    }

    if devices.len() > 1 {
        let mut msg = format!("{} devices are attached; pass --serial:\n", devices.len());
        for device in &devices {
            msg.push_str(&format!("  {device}\n"));
        }
        return Err(Error::Usb(msg));
    }

    Ok(devices.remove(0))
}

/// Prints where images are searched for and whether they are current.
fn images_status() -> Result<()> {
    println!("search path (first match wins):");
    for dir in image_paths::search_paths() {
        println!(
            "  {} {}",
            dir.display(),
            if dir.is_dir() { "" } else { "(does not exist)" }
        );
    }
    match std::env::var(image_paths::IMAGES_DIR_VAR) {
        Ok(value) => println!("\n{} = {value}", image_paths::IMAGES_DIR_VAR),
        Err(_) => println!("\n{} is not set", image_paths::IMAGES_DIR_VAR),
    }

    println!(
        "\nb2xx targets (manifest pinned to UHD {}):",
        images::MANIFEST_UHD_VERSION
    );
    for target in &images::MANIFEST {
        let status = match images::status(target) {
            images::Status::UpToDate => format!("ok ({})", target.repo_hash),
            images::Status::Stale(found) => {
                format!("stale (have {found}, want {})", target.repo_hash)
            }
            images::Status::Missing => "not in any inventory".to_owned(),
        };
        println!("  {:<28} {status}", target.name);
    }

    println!("\nfiles:");
    for name in [usb::B200_FW_IMAGE, usb::B200_BL_IMAGE] {
        report_image(name);
    }
    for product in usb::B2xxProduct::ALL {
        report_image(product.fpga_image());
    }

    Ok(())
}

/// One line per image file: where it resolved to, or that it did not.
fn report_image(name: &str) {
    match image_paths::find_image(name) {
        Ok(path) => println!("  {name:<28} {}", path.display()),
        Err(_) => println!("  {name:<28} not found"),
    }
}

/// Downloads the B2xx images package.
fn images_download(args: &DownloadArgs) -> Result<()> {
    let targets: Vec<&'static images::Target> = if args.target.is_empty() {
        images::MANIFEST.iter().collect()
    } else {
        args.target
            .iter()
            .map(|name| {
                images::target(name).ok_or_else(|| {
                    Error::invalid_arg(format!(
                        "unknown manifest target `{name}`; run `ruhd images status` for the list"
                    ))
                })
            })
            .collect::<Result<_>>()?
    };

    let dir = match &args.dir {
        Some(dir) => dir.clone(),
        None => image_paths::default_download_dir()?,
    };
    let base = images::base_url(args.base_url.as_deref());

    println!("installing into {}", dir.display());
    let outcomes = images::download(&targets, &dir, &base, args.force, |line| println!("{line}"))?;

    let installed = outcomes
        .iter()
        .filter(|o| matches!(o, images::Fetched::Installed { .. }))
        .count();
    println!(
        "\n{installed} of {} target(s) installed, {} already current",
        targets.len(),
        outcomes.len() - installed
    );

    // The directory is only useful if it is on the search path, and it will
    // not be if the caller pointed --dir somewhere unusual.
    if !image_paths::search_paths().contains(&dir) {
        println!(
            "\nNote: {} is not on the image search path.\nSet {}={} to use it.",
            dir.display(),
            image_paths::IMAGES_DIR_VAR,
            dir.display()
        );
    }

    Ok(())
}

fn probe_device(args: &str) -> Result<()> {
    let hint: DeviceArgs = args.parse()?;
    let device = registry::open(&hint)?;
    let info = device.info();

    println!("{info}");
    println!("  motherboards: {}", device.num_mboards());

    for mboard in 0..device.num_mboards() {
        println!("  mboard {mboard}:");
        if let Ok(rate) = device.master_clock_rate(mboard) {
            println!("    master clock: {:.6} MHz", rate / 1e6);
        }
        if let Ok(source) = device.clock_source(mboard) {
            println!("    clock source: {source}");
        }
        if let Ok(source) = device.time_source(mboard) {
            println!("    time source:  {source}");
        }
        print_sensors(device.as_ref(), SensorScope::Mboard(mboard));
    }

    for dir in Direction::ALL {
        let count = device.num_channels(dir);
        println!("  {dir}: {count} channel(s)");
        for chan in 0..count {
            print_channel(device.as_ref(), dir, chan);
        }
    }

    Ok(())
}

fn print_channel(device: &dyn Device, dir: Direction, chan: usize) {
    println!("    {dir} channel {chan}:");

    match device.freq_range(dir, chan) {
        Ok(range) => println!("      freq range:  {range} Hz"),
        Err(e) => println!("      freq range:  <{e}>"),
    }
    match device.center_freq(dir, chan) {
        Ok(freq) => println!("      center freq: {:.6} MHz", freq / 1e6),
        Err(e) => println!("      center freq: <{e}>"),
    }
    match device.sample_rate(dir, chan) {
        Ok(rate) => println!("      rate:        {:.6} Msps", rate / 1e6),
        Err(e) => println!("      rate:        <{e}>"),
    }
    match device.gain_range(dir, chan, None) {
        Ok(range) => println!("      gain range:  {range} dB"),
        Err(e) => println!("      gain range:  <{e}>"),
    }
    if let Ok(names) = device.gain_names(dir, chan)
        && !names.is_empty()
    {
        println!("      gain stages: {}", names.join(", "));
    }
    if let Ok(bandwidth) = device.bandwidth(dir, chan) {
        println!("      bandwidth:   {:.6} MHz", bandwidth / 1e6);
    }
    match device.antennas(dir, chan) {
        Ok(antennas) => println!("      antennas:    {}", antennas.join(", ")),
        Err(e) => println!("      antennas:    <{e}>"),
    }

    print_sensors(device, SensorScope::Channel { dir, chan });
}

fn print_sensors(device: &dyn Device, scope: SensorScope) {
    let Ok(names) = device.sensor_names(scope) else {
        return;
    };
    for name in names {
        match device.sensor(scope, &name) {
            Ok(value) => println!("      sensor {name}: {value}"),
            Err(e) => println!("      sensor {name}: <{e}>"),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn cli_definition_is_valid() {
        use clap::CommandFactory;
        Cli::command().debug_assert();
    }
}
