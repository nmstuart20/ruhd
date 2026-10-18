//! Finding B2xx devices on the USB bus, and getting firmware running on them.
//!
//! This is the first thing in the workspace that touches hardware. Two layers,
//! and the split matters: [`mod@probe`] only reads descriptors — it opens
//! nothing, transfers nothing, and changes nothing — while [`fx3`] opens the
//! control endpoint and pushes the FX3 firmware image. Discovery stays
//! side-effect-free so that it can describe a device it has no permission to
//! open, and so that listing devices is not an operation that mutates them.
//!
//! ```no_run
//! use uhd_transport::{images, usb};
//!
//! # fn main() -> uhd_core::Result<()> {
//! let firmware = usb::HexImage::load(&images::find_image(usb::B200_FW_IMAGE)?)?;
//!
//! for device in usb::probe()? {
//!     println!("{device}");
//!
//!     // Firmware first: nothing below this line works without it.
//!     let outcome = usb::initialize(&device, &firmware, false, usb::REENUMERATION_TIMEOUT)?;
//!     let iface = usb::Fx3Iface::open(&outcome.device)?;
//!
//!     // Then the FPGA. Which bitstream depends on the board, and on a
//!     // B200/B210 only the EEPROM can say which board it is.
//!     let bitstream = usb::FpgaImage::load(&images::find_image(usb::image_for(
//!         &outcome.device,
//!         &iface,
//!     )?)?)?;
//!     usb::configure(&iface, &bitstream, false, |_sent, _total| {})?;
//!
//!     // Now ask it what it is. Nothing here refuses anything — see `compat`.
//!     let report = usb::CompatReport::read(&iface, &outcome.device)?;
//!     println!("  -> firmware {}, fpga {}", report.firmware, report.fpga);
//!     for problem in report.problems() {
//!         println!("  !! {problem}");
//!     }
//! }
//! # Ok(())
//! # }
//! ```
//!
//! # What discovery can and cannot tell you
//!
//! A B2xx has no flash. The FX3 microcontroller boots from a small EEPROM that
//! holds only the USB identity — VID, PID, serial — and the firmware itself is
//! volatile, pushed over USB on every plug-in, with the FPGA image after it. So
//! a device found on the bus is in one of three conditions, and
//! [`DeviceState`] is exactly that distinction:
//!
//! | Enumerates as | State | What it needs |
//! | --- | --- | --- |
//! | `04b4:00f3` (Cypress FX3) | [`Uninitialized`](DeviceState::Uninitialized) | EEPROM programming, then firmware |
//! | `2500:0022`, Cypress descriptors | [`NeedsFirmware`](DeviceState::NeedsFirmware) | `usrp_b200_fw.hex` |
//! | `2500:0022`, Ettus descriptors | [`Ready`](DeviceState::Ready) | nothing; open it |
//!
//! The middle row is the common one — that is what a healthy board looks like
//! the moment it is plugged in. The top row is a board whose EEPROM was never
//! written or has been erased, and it is genuinely anonymous: no serial, no
//! product, because those fields live in the EEPROM.
//!
//! One ambiguity survives even for a ready device. The B200 and the B210 share
//! product ID `0x0020`, and telling them apart means opening the device and
//! reading the product code out of the motherboard EEPROM. [`UsbProbe::product`]
//! is `None` in that case rather than guessing; [`B2xxProduct::from_eeprom_id`]
//! is what resolves it once there is a control endpoint to read with.
//!
//! # udev rules for non-root access
//!
//! On Linux, enumeration reads sysfs and works for any user, but *opening* a
//! device goes through `/dev/bus/usb/…`, which is `root:root 0664` by default.
//! The result is the confusing failure mode this module tries to make obvious:
//! `probe` lists the device in full detail, and [`UsbProbe::check_access`] then
//! reports permission denied, with the fix in the message.
//!
//! The fix is a udev rule. The rules themselves live in the repository's
//! `udev/` directory and the instructions in the README's "USB permissions"
//! section, so that there is one copy of each. Two facts matter in code:
//!
//! - A distro's UHD package often installs an equivalent rule already, so a
//!   working `check_access` on one machine says nothing about the next.
//! - No standard rule covers the Cypress bootloader IDs
//!   ([`FX3_BOOTLOADER_VID_PIDS`]), so a board in
//!   [`DeviceState::Uninitialized`] usually needs root even where everything
//!   else works. The opt-in `udev/61-ruhd-fx3-bootloader.rules` covers it.
//!
//! # Bringing a device all the way up
//!
//! Firmware is only the first half. A B2xx's FPGA is volatile too, so the full
//! sequence from "just plugged in" to "usable" is:
//!
//! ```text
//!   probe          -> DeviceState::NeedsFirmware
//!   initialize     -> firmware running, FX3 answers vendor requests
//!   eeprom::read   -> which board this is, which bitstream it needs
//!   fpga::configure-> bitstream pushed, GPIF reset, settings bus alive
//!   CompatReport   -> every version number the device reports
//! ```
//!
//! The EEPROM read sits in the middle because it is the only thing that can
//! tell a B200 from a B210, and they need different bitstreams.
//!
//! # Module layout
//!
//! - [`ids`] — the VID/PID and EEPROM tables, each mirroring a named table in
//!   the UHD reference.
//! - [`mod@probe`] — enumeration, classification, and the post-load
//!   rediscovery poll.
//! - [`ihex`] — Intel HEX parsing, which is the format the FX3 firmware image
//!   comes in.
//! - [`fx3`] — the control endpoint: the boot ROM's firmware-load protocol,
//!   and the vendor requests running firmware answers.
//! - [`fpga`] — pushing a bitstream through the FX3 into the FPGA.
//! - [`eeprom`] — the motherboard EEPROM map, which is where a board's identity
//!   lives.
//! - [`ctrl`] — peek and poke against the FPGA's settings bus, over the bulk
//!   endpoints rather than endpoint zero.
//! - [`compat`] — the version numbers, gathered and *not* judged.
//!
//! [`fx3::initialize`] and [`fpga::configure`] are the two halves of "make this
//! device usable", and are what `ruhd init` calls.

pub mod compat;
pub mod ctrl;
pub mod eeprom;
pub mod fpga;
pub mod fx3;
pub mod ids;
pub mod ihex;
pub mod probe;

pub use compat::{
    B200_FPGA_COMPAT, B205_FPGA_COMPAT, CompatReport, FpgaCompat, FpgaVersion, expected_fpga_compat,
};
pub use ctrl::LocalCtrl;
pub use eeprom::{EepromLayout, MbEeprom};
pub use fpga::{FpgaImage, FpgaLoad, configure, image_for, image_hash, load_fpga};
pub use fx3::{
    B200_FW_COMPAT, FirmwareVersion, Fx3Iface, Fx3State, InitOutcome, REENUMERATION_TIMEOUT,
    RESET_TIMEOUT, initialize, reset,
};
pub use ids::{
    B2XX_VID_PIDS, B2xxProduct, B200_BL_IMAGE, B200_FW_IMAGE, B200_NI_PID, B200_PID, B200MINI_PID,
    B205MINI_PID, B206MINI_PID, B210_NI_PID, ETTUS_VID, FX3_BOOTLOADER_PID,
    FX3_BOOTLOADER_VID_PIDS, FX3_REENUM_PID, FX3_VID, NI_VID, vid_pid_search_list,
};
pub use ihex::{HexImage, HexRecord};
pub use probe::{
    BusIdentity, Classification, DeviceState, UsbFilter, UsbProbe, classify, probe, probe_filtered,
    wait_for_state,
};
