//! The FX3 control endpoint: boot ROM firmware load, and the vendor requests
//! the running firmware answers.
//!
//! Mirrors `b200_iface_impl` in `host/lib/usrp/b200/b200_iface.cpp`, for the
//! part of it that has to work before anything else does. Everything here goes
//! over USB control transfers on endpoint 0, so it needs no interface claimed,
//! no bulk endpoints, and no FPGA — which is exactly why it is the first thing
//! that can talk to a freshly plugged board.
//!
//! # Two protocols on one endpoint
//!
//! The FX3 answers to a different set of vendor requests depending on what is
//! running on it, and [`DeviceState`] says which:
//!
//! - **Boot ROM** ([`NeedsFirmware`](super::DeviceState::NeedsFirmware) or
//!   [`Uninitialized`](super::DeviceState::Uninitialized)). Cypress's ROM
//!   loader, which understands exactly one request: [`FX3_FIRMWARE_LOAD`]
//!   (`0xA0`), "write these bytes to this address in RAM". A zero-length write
//!   means "jump to this address", and that is what ends the load.
//! - **B200 firmware** ([`Ready`](super::DeviceState::Ready)). The `0x1x`–`0xBx`
//!   requests below: compatibility number, USB speed, FX3 state machine status,
//!   EEPROM access, FPGA configuration, resets.
//!
//! The two do not overlap. Issuing [`VREQ_GET_COMPAT`] to a boot ROM stalls the
//! endpoint, and issuing [`FX3_FIRMWARE_LOAD`] to running firmware does
//! nothing useful — so [`load_firmware`](Fx3Iface::load_firmware) and
//! [`firmware_version`](Fx3Iface::firmware_version) are separated by a
//! re-enumeration, not just by a state flag.
//!
//! # The load sequence
//!
//! ```text
//!   probe            -> 2500:0022, Cypress descriptors, serial 0000000004BE,
//!                       bus 001 -> NeedsFirmware
//!   load_firmware    -> 11 516 control writes, then a zero-length write to
//!                       the entry point at 0x4001_9dec
//!   <device drops off the bus and re-enumerates>
//!   wait_for_state   -> 2500:0022, Ettus descriptors, serial 3168B41,
//!                       bus 002 -> Ready
//!   firmware_version -> 8.0, checked against B200_FW_COMPAT
//! ```
//!
//! Those are real numbers from a B205mini, and the serial and bus changing
//! across the middle line is not a typo — it is the thing that makes
//! rediscovery hard, and [`wait_for_state`] is where that is dealt with.
//!
//! [`initialize`] is the whole sequence, and is what `ruhd init` runs.

use std::time::{Duration, Instant};

use nusb::MaybeFuture;
use nusb::transfer::{ControlIn, ControlOut, ControlType, Recipient};
use tracing::{debug, info, trace};
use uhd_core::{Error, Result};

use super::ihex::HexImage;
use super::probe::{BusIdentity, DeviceState, UsbProbe, probe, wait_for_state};

/// Cypress boot ROM request: write a block of bytes into FX3 RAM.
///
/// `wValue` is the low half of the destination address and `wIndex` the high
/// half. A zero-length transfer is the terminator: it tells the core to start
/// executing at that address.
///
/// `FX3_FIRMWARE_LOAD` in `b200_iface.cpp`.
pub const FX3_FIRMWARE_LOAD: u8 = 0xA0;

/// Begin FPGA configuration.
pub const VREQ_FPGA_START: u8 = 0x02;
/// Push a block of FPGA bitstream.
pub const VREQ_FPGA_DATA: u8 = 0x12;
/// Read the firmware compatibility number. Two bytes, major then minor.
pub const VREQ_GET_COMPAT: u8 = 0x15;
/// Store the host's hash of the loaded FPGA image.
pub const VREQ_SET_FPGA_HASH: u8 = 0x1C;
/// Read back the stored FPGA image hash, to decide whether to reload it.
pub const VREQ_GET_FPGA_HASH: u8 = 0x1D;
/// Store the host's hash of the loaded firmware image.
pub const VREQ_SET_FW_HASH: u8 = 0x1E;
/// Read back the stored firmware image hash.
pub const VREQ_GET_FW_HASH: u8 = 0x1F;
/// Loopback test.
pub const VREQ_LOOP: u8 = 0x22;
/// Finish FPGA configuration.
pub const VREQ_FPGA_CONFIG: u8 = 0x55;
/// Reset the GPIF state machine.
pub const VREQ_GPIF_RESET: u8 = 0x72;
/// Read the negotiated USB speed. One byte, `2` or `3`.
pub const VREQ_GET_USB: u8 = 0x80;
/// Read the FX3 state machine's state. One byte, see [`Fx3State`].
pub const VREQ_GET_STATUS: u8 = 0x83;
/// Reset the FX3, which drops the device off the bus.
pub const VREQ_FX3_RESET: u8 = 0x99;
/// Write to the motherboard EEPROM.
pub const VREQ_EEPROM_WRITE: u8 = 0xBA;
/// Read from the motherboard EEPROM.
pub const VREQ_EEPROM_READ: u8 = 0xBB;

/// Firmware compatibility number this driver is written against.
///
/// `B200_FW_COMPAT_NUM_MAJOR` / `_MINOR` in `b200_impl.hpp`. Only the major
/// number is checked — see [`FirmwareVersion::is_compatible`].
pub const B200_FW_COMPAT: FirmwareVersion = FirmwareVersion { major: 8, minor: 0 };

/// Largest vendor-request payload the firmware accepts over a USB 2 link.
///
/// `VREQ_MAX_SIZE_USB2` in `b200_iface.cpp`, and also its `VREQ_DEFAULT_SIZE`.
pub const VREQ_MAX_SIZE_USB2: usize = 64;

/// Largest vendor-request payload the firmware accepts over a USB 3 link.
///
/// `VREQ_MAX_SIZE_USB3` in `b200_iface.cpp`. This is the chunk size the FPGA
/// bitstream is pushed in, so it is the difference between a two-second and a
/// fifteen-second FPGA load.
pub const VREQ_MAX_SIZE_USB3: usize = 512;

/// How long to give a single control transfer.
///
/// UHD passes `0` to libusb here, which means *no timeout at all*: a device
/// that stops answering hangs the process. These transfers are at most a few
/// hundred bytes over endpoint 0, so a second is already generous, and having
/// a bound is the difference between an error message and a wedged `ruhd`.
pub const CONTROL_TIMEOUT: Duration = Duration::from_secs(1);

/// How long to wait for a device to come back after a firmware load.
///
/// `REENUMERATION_TIMEOUT_MS` in `b200_impl.cpp`.
pub const REENUMERATION_TIMEOUT: Duration = Duration::from_millis(3000);

/// Timeout on the image-hash requests. UHD uses 500 ms for the read and no
/// timeout at all for the write; this uses it for both, since they are the
/// same size request in opposite directions.
pub const HASH_TIMEOUT: Duration = Duration::from_millis(500);

/// Timeout on the two FPGA configuration-start requests, per `load_fpga`.
pub const FPGA_CONFIG_TIMEOUT: Duration = Duration::from_millis(1000);

/// Timeout on a single chunk of bitstream. Generous because the FX3 is
/// clocking the chunk out to the FPGA's slave-serial pins while it holds the
/// transfer open. `load_fpga` uses the same 5 s.
pub const FPGA_DATA_TIMEOUT: Duration = Duration::from_millis(5000);

/// The FX3's internal state machine, as reported by [`VREQ_GET_STATUS`].
///
/// Mirrors `b200_iface::fx3_state_string`. This says nothing about firmware —
/// only running firmware can answer the request at all — it says how far along
/// FPGA configuration is.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
#[non_exhaustive]
pub enum Fx3State {
    /// Powered up, nothing configured.
    Undefined,
    /// FPGA is configured and ready.
    FpgaReady,
    /// FPGA configuration in progress.
    ConfiguringFpga,
    /// Busy.
    Busy,
    /// Running.
    Running,
    /// No FPGA image has been loaded.
    Unconfigured,
    /// The FX3 reported an error.
    Error,
    /// A code this driver does not know.
    Unknown(u8),
}

impl From<u8> for Fx3State {
    fn from(code: u8) -> Self {
        match code {
            0x00 => Self::Undefined,
            0x01 => Self::FpgaReady,
            0x02 => Self::ConfiguringFpga,
            0x03 => Self::Busy,
            0x04 => Self::Running,
            0x05 => Self::Unconfigured,
            0x06 => Self::Error,
            other => Self::Unknown(other),
        }
    }
}

impl std::fmt::Display for Fx3State {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::Undefined => f.write_str("undefined"),
            Self::FpgaReady => f.write_str("ready"),
            Self::ConfiguringFpga => f.write_str("configuring fpga"),
            Self::Busy => f.write_str("busy"),
            Self::Running => f.write_str("running"),
            Self::Unconfigured => f.write_str("unconfigured"),
            Self::Error => f.write_str("error"),
            Self::Unknown(code) => write!(f, "unknown ({code:#04x})"),
        }
    }
}

/// The firmware compatibility number, as reported by [`VREQ_GET_COMPAT`].
///
/// This is a compatibility contract, not a build version: the major number is
/// bumped when the host/firmware protocol changes incompatibly, and the minor
/// when firmware gains something a host may use but need not.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, PartialOrd, Ord)]
pub struct FirmwareVersion {
    /// Major number. Must match exactly.
    pub major: u8,
    /// Minor number. Informational.
    pub minor: u8,
}

impl FirmwareVersion {
    /// Splits the 16-bit value [`VREQ_GET_COMPAT`] returns.
    #[must_use]
    pub fn from_compat_num(compat: u16) -> Self {
        Self {
            major: (compat >> 8) as u8,
            minor: (compat & 0xff) as u8,
        }
    }

    /// Whether firmware reporting this version can drive this host code.
    ///
    /// Only the major number is compared, which is UHD's rule in
    /// `b200_impl::check_fw_compat`: a minor bump adds capability without
    /// removing any, so newer-minor firmware is fine and so is older-minor.
    #[must_use]
    pub fn is_compatible(self) -> bool {
        self.major == B200_FW_COMPAT.major
    }
}

impl std::fmt::Display for FirmwareVersion {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "{}.{}", self.major, self.minor)
    }
}

/// An open control endpoint on an FX3.
///
/// Holds a `nusb::Device` and nothing else. No interface is claimed: on Linux
/// and macOS endpoint 0 is available without one, and claiming would fail
/// against a boot ROM that has no interface worth claiming. UHD claims
/// interface 0 in `usb_control::make` because libusb on Windows insists on it,
/// which is a Windows problem to solve when Windows support is a goal.
#[derive(Debug)]
pub struct Fx3Iface {
    device: nusb::Device,
    endpoint: crate::endpoint::UsbEndpoint,
}

impl Fx3Iface {
    /// Opens the control endpoint of a probed device.
    ///
    /// # Errors
    ///
    /// [`Error::Usb`] if the device cannot be opened — no permission (see the
    /// udev section in [the module docs](super)), already claimed, or gone
    /// since it was probed.
    pub fn open(probe: &UsbProbe) -> Result<Self> {
        let device = probe.device_info().open().wait().map_err(|e| {
            if e.kind() == nusb::ErrorKind::PermissionDenied {
                Error::Usb(format!(
                    "permission denied opening {}; install the udev rules for non-root access",
                    probe.endpoint
                ))
            } else {
                Error::Usb(format!("cannot open {}: {e}", probe.endpoint))
            }
        })?;
        Ok(Self {
            device,
            endpoint: probe.endpoint.clone(),
        })
    }

    /// How this device was named when it was opened, for error messages.
    #[must_use]
    pub fn endpoint(&self) -> &crate::endpoint::UsbEndpoint {
        &self.endpoint
    }

    /// The open device handle, for the one thing that needs more than endpoint
    /// zero.
    ///
    /// Crate-private, like [`UsbProbe::device_info`]: `nusb` types are not part
    /// of this API. [`LocalCtrl`](super::ctrl::LocalCtrl) uses it to claim the
    /// bulk interfaces, and it takes a clone rather than a second `open` so
    /// that both halves of the device are reached through one handle — which
    /// is what UHD's cached-handle scheme achieves the hard way.
    #[doc(hidden)]
    #[must_use]
    pub fn device(&self) -> nusb::Device {
        self.device.clone()
    }

    /// A vendor control write. `fx3_control_write` in `b200_iface.cpp`.
    ///
    /// # Errors
    ///
    /// [`Error::Usb`] if the transfer fails or the device stalls the request.
    pub fn control_write(
        &self,
        request: u8,
        value: u16,
        index: u16,
        data: &[u8],
        timeout: Duration,
    ) -> Result<()> {
        trace!(request, value, index, len = data.len(), "fx3 control write");
        self.device
            .control_out(
                ControlOut {
                    control_type: ControlType::Vendor,
                    recipient: Recipient::Device,
                    request,
                    value,
                    index,
                    data,
                },
                timeout,
            )
            .wait()
            .map_err(|e| {
                Error::Usb(format!(
                    "control write {request:#04x} (value {value:#06x}, index {index:#06x}, \
                     {} bytes) to {} failed: {e}",
                    data.len(),
                    self.endpoint
                ))
            })
    }

    /// A vendor control read. `fx3_control_read` in `b200_iface.cpp`.
    ///
    /// # Errors
    ///
    /// [`Error::Usb`] if the transfer fails, and also on a short read: UHD
    /// treats a partial reply as an error for every one of these requests,
    /// because they all return fixed-width fields.
    pub fn control_read(
        &self,
        request: u8,
        value: u16,
        index: u16,
        length: u16,
        timeout: Duration,
    ) -> Result<Vec<u8>> {
        trace!(request, value, index, length, "fx3 control read");
        let data = self
            .device
            .control_in(
                ControlIn {
                    control_type: ControlType::Vendor,
                    recipient: Recipient::Device,
                    request,
                    value,
                    index,
                    length,
                },
                timeout,
            )
            .wait()
            .map_err(|e| {
                Error::Usb(format!(
                    "control read {request:#04x} (value {value:#06x}, index {index:#06x}, \
                     {length} bytes) from {} failed: {e}",
                    self.endpoint
                ))
            })?;

        if data.len() != usize::from(length) {
            return Err(Error::Usb(format!(
                "short read on control request {request:#04x} from {}: expected {length} \
                 bytes, got {}",
                self.endpoint,
                data.len()
            )));
        }
        Ok(data)
    }

    /// Pushes a parsed firmware image into FX3 RAM and starts it.
    ///
    /// Mirrors `b200_iface_impl::load_firmware`: one control write per Intel
    /// HEX record, then a zero-length write to the entry point. The device
    /// drops off the bus as soon as the last transfer lands — the firmware
    /// re-enumerates with its own descriptors — so this returning `Ok` means
    /// the image was accepted, not that the device is usable yet. That is what
    /// [`wait_for_state`] is for, and [`initialize`] does both.
    ///
    /// One transfer per record is UHD's behaviour and is deliberately kept.
    /// The B200 image is ~11 500 records of 16 bytes, so coalescing them into
    /// 512-byte writes would cut the transfer count by a factor of thirty —
    /// but the boot ROM's tolerance for that is undocumented, and a firmware
    /// load is a two-second operation that happens once per plug-in.
    ///
    /// # Errors
    ///
    /// [`Error::Firmware`] if the image has no entry point. [`Error::Usb`] if
    /// any transfer fails, with the record index in the message so a load that
    /// dies part way says where.
    pub fn load_firmware(&self, image: &HexImage) -> Result<()> {
        let entry = image.entry_point()?;

        info!(
            records = image.records.len(),
            bytes = image.payload_len(),
            entry = format_args!("{entry:#010x}"),
            "loading fx3 firmware"
        );

        for (i, record) in image.records.iter().enumerate() {
            self.control_write(
                FX3_FIRMWARE_LOAD,
                record.value(),
                record.index(),
                &record.data,
                CONTROL_TIMEOUT,
            )
            .map_err(|e| {
                Error::Usb(format!(
                    "firmware load failed at record {i} of {} (address {:#010x}): {e}",
                    image.records.len(),
                    record.address
                ))
            })?;
        }

        // Zero-length write to the entry point: "stop taking records, start
        // executing". The device leaves the bus while this is in flight, so a
        // failure here is not necessarily a failure — but the successful case
        // does return cleanly, and treating an error as success would hide a
        // genuinely rejected image.
        debug!(
            entry = format_args!("{entry:#010x}"),
            "jumping to entry point"
        );
        self.control_write(
            FX3_FIRMWARE_LOAD,
            entry as u16,
            (entry >> 16) as u16,
            &[],
            CONTROL_TIMEOUT,
        )
    }

    /// Reads the firmware compatibility number.
    ///
    /// Only running firmware answers this; a boot ROM stalls it. `get_compat_num`
    /// in `b200_iface.cpp`.
    ///
    /// # Errors
    ///
    /// [`Error::Usb`] if the request fails or is stalled.
    pub fn firmware_version(&self) -> Result<FirmwareVersion> {
        let data = self.control_read(VREQ_GET_COMPAT, 0, 0, 2, CONTROL_TIMEOUT)?;
        Ok(FirmwareVersion::from_compat_num(u16::from_be_bytes([
            data[0], data[1],
        ])))
    }

    /// Reads the firmware compatibility number and rejects an incompatible one.
    ///
    /// Mirrors `b200_impl::check_fw_compat`, including that only the major
    /// number decides. The message points at the images downloader, because a
    /// major mismatch always means the image on disk and this driver came from
    /// different UHD generations.
    ///
    /// # Errors
    ///
    /// [`Error::Usb`] if the request fails; [`Error::Firmware`] if the major
    /// number does not match [`B200_FW_COMPAT`].
    pub fn check_firmware_version(&self) -> Result<FirmwareVersion> {
        let found = self.firmware_version()?;
        if !found.is_compatible() {
            return Err(Error::Firmware(format!(
                "expected firmware compatibility number {B200_FW_COMPAT}, but {} reports \
                 {found}.\nThe firmware build is not compatible with this driver.\nRun \
                 `{}` to fetch a matching images package.",
                self.endpoint,
                crate::images::DOWNLOAD_COMMAND,
            )));
        }
        debug!(version = %found, "firmware compatibility number accepted");
        Ok(found)
    }

    /// The negotiated USB speed, `2` or `3`. `get_usb_speed` in `b200_iface.cpp`.
    ///
    /// # Errors
    ///
    /// [`Error::Usb`] if the request fails.
    pub fn usb_speed(&self) -> Result<u8> {
        Ok(self.control_read(VREQ_GET_USB, 0, 0, 1, CONTROL_TIMEOUT)?[0])
    }

    /// The FX3 state machine's state. `get_fx3_status` in `b200_iface.cpp`.
    ///
    /// # Errors
    ///
    /// [`Error::Usb`] if the request fails.
    pub fn fx3_state(&self) -> Result<Fx3State> {
        Ok(Fx3State::from(
            self.control_read(VREQ_GET_STATUS, 0, 0, 1, CONTROL_TIMEOUT)?[0],
        ))
    }

    /// Resets the FX3, which drops the device off the bus and takes the loaded
    /// firmware with it. `reset_fx3` in `b200_iface.cpp`.
    ///
    /// The device comes back in [`DeviceState::NeedsFirmware`] a second or two
    /// later. This is how a firmware *reload* is done: there is no way to
    /// overwrite running firmware in place.
    ///
    /// # Errors
    ///
    /// [`Error::Usb`] if the request fails. A device that leaves the bus
    /// before acknowledging is a normal outcome of a reset, so callers that
    /// only want the side effect may reasonably ignore the error.
    pub fn reset_fx3(&self) -> Result<()> {
        debug!(endpoint = %self.endpoint, "resetting fx3");
        self.control_write(VREQ_FX3_RESET, 0, 0, &[0; 4], CONTROL_TIMEOUT)
    }

    /// Resets the GPIF state machine. `reset_gpif` in `b200_iface.cpp`.
    ///
    /// # Errors
    ///
    /// [`Error::Usb`] if the request fails.
    pub fn reset_gpif(&self) -> Result<()> {
        self.control_write(VREQ_GPIF_RESET, 0, 0, &[0; 4], CONTROL_TIMEOUT)
    }

    /// Reads `len` bytes from the motherboard EEPROM at `addr`/`offset`.
    ///
    /// `read_eeprom` in `b200_iface.cpp`. This is what resolves a B200 from a
    /// B210, which share USB product ID `0x0020`.
    ///
    /// `addr` and `offset` are the high and low bytes of a 16-bit EEPROM
    /// address, split exactly as UHD splits them at every call site
    /// (`read_eeprom(addr >> 8, addr & 0xFF, len)`); the firmware reassembles
    /// them and does the I²C transaction. A read may run past `0xff` into the
    /// next page — the rev 0 map at `0x04dc` does — and the firmware handles
    /// that, so `offset` is a starting point rather than a page-local limit.
    ///
    /// See [`eeprom`](super::eeprom) for the map these addresses index into.
    ///
    /// # Errors
    ///
    /// [`Error::Usb`] if the request fails or comes back short.
    pub fn read_eeprom(&self, addr: u8, offset: u8, len: u16) -> Result<Vec<u8>> {
        let index = u16::from(offset) | (u16::from(addr) << 8);
        self.control_read(VREQ_EEPROM_READ, 0, index, len, CONTROL_TIMEOUT)
    }

    /// Writes `data` to the motherboard EEPROM at `addr`/`offset`.
    ///
    /// `write_eeprom` in `b200_iface.cpp`, addressed the same way as
    /// [`read_eeprom`](Self::read_eeprom).
    ///
    /// This rewrites the board's identity — serial, product code, revision —
    /// and a bad write leaves a device that no longer enumerates as a USRP.
    /// Nothing in this crate calls it; it exists so that the recovery path
    /// (UHD's `b2xx_fx3_utils -D`) can be built on top without reaching around
    /// the interface.
    ///
    /// # Errors
    ///
    /// [`Error::Usb`] if the request fails.
    pub fn write_eeprom(&self, addr: u8, offset: u8, data: &[u8]) -> Result<()> {
        let index = u16::from(offset) | (u16::from(addr) << 8);
        self.control_write(VREQ_EEPROM_WRITE, 0, index, data, CONTROL_TIMEOUT)
    }

    /// Largest vendor-request payload this link will carry, from the negotiated
    /// USB speed.
    ///
    /// `_get_transfer_size` in `b200_iface.cpp`: 64 bytes over USB 2, 512 over
    /// USB 3. This is the FPGA bitstream chunk size, and it is asked of the
    /// device rather than assumed because the same board gives you either
    /// depending on which port it is in.
    ///
    /// # Errors
    ///
    /// [`Error::Usb`] if the request fails; [`Error::Firmware`] if the device
    /// reports a speed that is neither 2 nor 3, which UHD also treats as fatal
    /// rather than guessing the smaller size.
    pub fn transfer_size(&self) -> Result<usize> {
        match self.usb_speed()? {
            2 => Ok(VREQ_MAX_SIZE_USB2),
            3 => Ok(VREQ_MAX_SIZE_USB3),
            other => Err(Error::Firmware(format!(
                "{} reports usb speed {other}, which is neither 2 nor 3",
                self.endpoint
            ))),
        }
    }

    /// Reads `len` bytes back from the firmware's control-request buffer.
    ///
    /// `B200_VREQ_LOOP`. The firmware answers with whatever is in the buffer,
    /// so the bytes are meaningless; what is being tested is that a transfer of
    /// exactly `len` succeeds, which proves the firmware's buffer is at least
    /// that large. `load_fpga` does this before pushing a bitstream, so that a
    /// host that has overestimated the chunk size fails on one harmless request
    /// rather than part way through configuring the FPGA.
    ///
    /// # Errors
    ///
    /// [`Error::Usb`] if the request fails or comes back short.
    pub fn loopback(&self, len: u16) -> Result<Vec<u8>> {
        self.control_read(VREQ_LOOP, 0, 0, len, FPGA_CONFIG_TIMEOUT)
    }

    /// Reads the host-supplied hash of the FPGA image the firmware last saw
    /// loaded. `usrp_get_fpga_hash` in `b200_iface.cpp`.
    ///
    /// The FX3 stores this and gives it back; it never computes or checks
    /// anything. It is a note the host leaves for its future self — see
    /// [`image_hash`](super::fpga::image_hash) for why the note has to be
    /// written in UHD's handwriting.
    ///
    /// # Errors
    ///
    /// [`Error::Usb`] if the request fails or comes back short.
    pub fn fpga_hash(&self) -> Result<u32> {
        let data = self.control_read(VREQ_GET_FPGA_HASH, 0, 0, 4, HASH_TIMEOUT)?;
        Ok(u32::from_le_bytes([data[0], data[1], data[2], data[3]]))
    }

    /// Records the host's hash of the FPGA image just loaded.
    /// `usrp_set_fpga_hash` in `b200_iface.cpp`.
    ///
    /// # Errors
    ///
    /// [`Error::Usb`] if the request fails.
    pub fn set_fpga_hash(&self, hash: u32) -> Result<()> {
        self.control_write(VREQ_SET_FPGA_HASH, 0, 0, &hash.to_le_bytes(), HASH_TIMEOUT)
    }

    /// Reads the host-supplied hash of the running firmware image.
    /// `usrp_get_firmware_hash` in `b200_iface.cpp`.
    ///
    /// UHD never writes this one — the `usrp_set_firmware_hash` call in
    /// `load_firmware` is commented out with a `TODO` — so on a device brought
    /// up by either driver it reads back as whatever the firmware initialised
    /// it to. Provided for completeness of the request set.
    ///
    /// # Errors
    ///
    /// [`Error::Usb`] if the request fails or comes back short.
    pub fn firmware_hash(&self) -> Result<u32> {
        let data = self.control_read(VREQ_GET_FW_HASH, 0, 0, 4, HASH_TIMEOUT)?;
        Ok(u32::from_le_bytes([data[0], data[1], data[2], data[3]]))
    }

    /// Records the host's hash of the firmware image just loaded.
    /// `usrp_set_firmware_hash` in `b200_iface.cpp`.
    ///
    /// # Errors
    ///
    /// [`Error::Usb`] if the request fails.
    pub fn set_firmware_hash(&self, hash: u32) -> Result<()> {
        self.control_write(VREQ_SET_FW_HASH, 0, 0, &hash.to_le_bytes(), HASH_TIMEOUT)
    }

    /// Tells the FX3 to bring the FPGA into slave-serial configuration mode.
    ///
    /// The first of the two starts in `load_fpga`: this one asserts `PROGRAM_B`
    /// and waits for the FPGA to raise `INIT_B`, after which
    /// [`fx3_state`](Self::fx3_state) reports
    /// [`FpgaReady`](Fx3State::FpgaReady). The payload is a single zero byte,
    /// which the firmware ignores — the request code is the whole message.
    ///
    /// # Errors
    ///
    /// [`Error::Usb`] if the request fails.
    pub fn start_fpga_config(&self) -> Result<()> {
        self.control_write(VREQ_FPGA_CONFIG, 0, 0, &[0], FPGA_CONFIG_TIMEOUT)
    }

    /// Tells the FX3 to start accepting bitstream data.
    ///
    /// The second start: after this the state machine moves to
    /// [`ConfiguringFpga`](Fx3State::ConfiguringFpga) and
    /// [`write_fpga_data`](Self::write_fpga_data) is what it wants next.
    ///
    /// # Errors
    ///
    /// [`Error::Usb`] if the request fails.
    pub fn start_fpga_bitstream(&self) -> Result<()> {
        self.control_write(VREQ_FPGA_START, 0, 0, &[0], FPGA_CONFIG_TIMEOUT)
    }

    /// Pushes one chunk of FPGA bitstream.
    ///
    /// `chunk` must be no longer than [`transfer_size`](Self::transfer_size)
    /// reported, which is the firmware's control-request buffer.
    ///
    /// # Errors
    ///
    /// [`Error::Usb`] if the transfer fails.
    pub fn write_fpga_data(&self, chunk: &[u8]) -> Result<()> {
        self.control_write(VREQ_FPGA_DATA, 0, 0, chunk, FPGA_DATA_TIMEOUT)
    }
}

/// What [`initialize`] did.
#[derive(Debug, Clone)]
#[non_exhaustive]
pub struct InitOutcome {
    /// The device as it is now: re-probed, so the state and descriptors are
    /// post-load.
    pub device: UsbProbe,
    /// The firmware compatibility number the device reports.
    pub version: FirmwareVersion,
    /// Whether firmware was actually pushed, as opposed to already running.
    pub loaded: bool,
    /// How long the device took to come back, when it was loaded.
    pub reenumeration: Option<Duration>,
}

/// How long to allow for an FX3 reset to bring the device back as a boot ROM.
///
/// A reset tears down the SuperSpeed link and brings the device back up at
/// high speed on a different bus, so it is a heavier event than the plain
/// re-enumeration after a load — but not by much: a B205mini comes back in
/// about 375 ms. This is a generous upper bound rather than a measured
/// requirement. UHD does not poll for this at all; `b2xx_fx3_utils` sleeps a
/// flat two seconds and hopes.
pub const RESET_TIMEOUT: Duration = Duration::from_secs(10);

/// How long to keep retrying an open after the device has re-enumerated.
///
/// A freshly re-enumerated device shows up in the kernel's device model — and
/// so in an enumeration — before udev has finished applying permissions to its
/// `/dev/bus/usb/…` node. The window is short, but it is squarely in the
/// middle of a firmware load, so opening the device the instant rediscovery
/// finds it reliably fails with `EACCES` on a machine where opening it a
/// moment later works fine.
///
/// This has nothing to do with whether udev rules are installed: it is the
/// difference between "not yet" and "not allowed", and without a grace period
/// the two are indistinguishable at exactly the wrong moment.
pub const OPEN_GRACE_PERIOD: Duration = Duration::from_secs(2);

/// Takes one device from wherever it is to firmware-running and verified.
///
/// This is `ruhd init`, and it is the sequence at the top of these docs:
///
/// 1. If the device is already [`Ready`](DeviceState::Ready) and `force` is
///    false, skip to step 5. UHD's `b200_find` makes the same check, via
///    `usb_device_handle::firmware_loaded()`.
/// 2. If it is already `Ready` and `force` is true, reset the FX3 and wait for
///    it to come back as a boot ROM — running firmware cannot be overwritten in
///    place. This mirrors `b2xx_fx3_utils`'s `--load-fw` path.
/// 3. Push the image.
/// 4. Poll the bus until the device reappears.
/// 5. Read the compatibility number and reject a mismatch.
///
/// Steps 2 and 4 both take a snapshot of the bus first and identify the
/// returning device by elimination against it, because a B2xx's serial number
/// does not survive a firmware transition — see [`wait_for_state`] for the
/// measurements. The snapshot is taken *before* the transfer that makes the
/// device leave the bus, which is the only moment it is meaningful.
///
/// `image` is a parsed Intel HEX file — parse it once and initialize many
/// devices from it if there are several attached.
///
/// # Errors
///
/// [`Error::Usb`] if the device cannot be opened or a transfer fails;
/// [`Error::Timeout`] if it does not come back within `timeout`;
/// [`Error::Firmware`] if the image has no entry point or the firmware that
/// comes up reports an incompatible major version.
pub fn initialize(
    probe: &UsbProbe,
    image: &HexImage,
    force: bool,
    timeout: Duration,
) -> Result<InitOutcome> {
    let mut probe = probe.clone();
    let mut reset = false;

    if probe.state == DeviceState::Ready {
        if !force {
            debug!(endpoint = %probe.endpoint, "firmware already running");
            let iface = Fx3Iface::open(&probe)?;
            let version = iface.check_firmware_version()?;
            return Ok(InitOutcome {
                device: probe,
                version,
                loaded: false,
                reenumeration: None,
            });
        }

        info!(endpoint = %probe.endpoint, "resetting fx3 to overwrite running firmware");
        probe = self::reset(&probe, RESET_TIMEOUT.max(timeout))?.0;
        reset = true;
    }

    // The serial is recorded, but only as a hint: on a B2xx it is about to
    // change from the FX3's own to the USRP's. `before` is what actually
    // identifies the device when it returns.
    let serial = probe.serial().map(str::to_owned);
    let before = snapshot(DeviceState::Ready)?;

    // Only the device that just came back from a reset needs the grace period.
    // Everywhere else, a permission error is the real thing and deserves to be
    // reported at once — "install the udev rules" is the most common first-run
    // failure there is, and taking two seconds to say so helps nobody.
    let iface = if reset {
        open_after_reenumeration(&probe)?
    } else {
        Fx3Iface::open(&probe)?
    };
    iface.load_firmware(image)?;
    // The handle now refers to a device that has left the bus. Closing it
    // before re-probing keeps the OS from handing back the stale one, which is
    // the same reason UHD tears its handles down inside the discovery loop.
    drop(iface);

    let (device, reenumeration) =
        wait_timed(serial.as_deref(), &before, DeviceState::Ready, timeout).map_err(|e| {
            Error::Firmware(format!(
                "firmware was pushed, but the device did not re-enumerate: {e}.\n\
                 The image was accepted, so this is usually the link rather than the \
                 image: try a different cable or a port that is not behind a hub. If it \
                 persists, the image on disk may be for a different device."
            ))
        })?;
    info!(
        endpoint = %device.endpoint,
        took_ms = reenumeration.as_millis(),
        "device re-enumerated"
    );

    let iface = open_after_reenumeration(&device)?;
    let version = iface.check_firmware_version()?;

    Ok(InitOutcome {
        device,
        version,
        loaded: true,
        reenumeration: Some(reenumeration),
    })
}

/// Resets the FX3 and waits for the device to come back needing firmware.
///
/// Running firmware cannot be overwritten in place, so this is the first half
/// of any reload, and it is also the recovery path for a device that has got
/// itself into a state nothing else will shift. Returns the device as it now
/// is — with the FX3's own descriptors, so a *different* serial and quite
/// possibly a different bus — and how long it took to come back.
///
/// `b2xx_fx3_utils --reset-device`, except that UHD sleeps a flat two seconds
/// afterwards rather than watching the bus.
///
/// # Errors
///
/// [`Error::Usb`] if the device cannot be opened; [`Error::Firmware`] if it
/// does not return within `timeout`. The reset request itself failing is *not*
/// an error: a device that leaves the bus before acknowledging has done
/// exactly what was asked.
pub fn reset(probe: &UsbProbe, timeout: Duration) -> Result<(UsbProbe, Duration)> {
    let before = snapshot(DeviceState::NeedsFirmware)?;

    let iface = Fx3Iface::open(probe)?;
    if let Err(e) = iface.reset_fx3() {
        debug!("fx3 reset did not acknowledge, which is expected: {e}");
    }
    drop(iface);

    wait_timed(None, &before, DeviceState::NeedsFirmware, timeout).map_err(|e| {
        Error::Firmware(format!(
            "the fx3 did not come back as a bootloader after being reset: {e}"
        ))
    })
}

/// Opens a device that has just re-enumerated, tolerating the udev race.
///
/// See [`OPEN_GRACE_PERIOD`]. Any error is retried, not just permission
/// denied: the device node can also be briefly absent altogether. The last
/// error is returned unchanged once the grace period is up, so a machine that
/// genuinely lacks udev rules still gets the message that says so.
fn open_after_reenumeration(probe: &UsbProbe) -> Result<Fx3Iface> {
    let deadline = Instant::now() + OPEN_GRACE_PERIOD;
    loop {
        match Fx3Iface::open(probe) {
            Ok(iface) => return Ok(iface),
            Err(e) if Instant::now() >= deadline => return Err(e),
            Err(e) => {
                trace!("device not openable yet, retrying: {e}");
                std::thread::sleep(super::probe::REDISCOVERY_POLL_INTERVAL);
            }
        }
    }
}

/// Records which devices are already in `state`, so that the one that arrives
/// there later can be told apart from the ones that were always there.
fn snapshot(state: DeviceState) -> Result<Vec<BusIdentity>> {
    Ok(probe()?
        .into_iter()
        .filter(|d| d.state == state)
        .map(|d| d.identity())
        .collect())
}

/// [`wait_for_state`] with a stopwatch on it.
///
/// The elapsed time is worth reporting: a re-enumeration that takes most of
/// the timeout is a sign of a marginal cable or hub, and is the thing to look
/// at when initialisation is intermittent rather than broken.
fn wait_timed(
    serial: Option<&str>,
    exclude: &[BusIdentity],
    state: DeviceState,
    timeout: Duration,
) -> Result<(UsbProbe, Duration)> {
    let started = Instant::now();
    let device = wait_for_state(serial, exclude, state, timeout)?;
    Ok((device, started.elapsed()))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn compat_numbers_split_major_from_minor() {
        assert_eq!(
            FirmwareVersion::from_compat_num(0x0800),
            FirmwareVersion { major: 8, minor: 0 }
        );
        assert_eq!(
            FirmwareVersion::from_compat_num(0x0803),
            FirmwareVersion { major: 8, minor: 3 }
        );
        assert_eq!(FirmwareVersion::from_compat_num(0x0800).to_string(), "8.0");
    }

    #[test]
    fn only_the_major_number_decides_compatibility() {
        // This is UHD's rule, and it is the reason a device reporting 8.3
        // against a host built for 8.0 is fine.
        assert!(FirmwareVersion { major: 8, minor: 0 }.is_compatible());
        assert!(FirmwareVersion { major: 8, minor: 9 }.is_compatible());
        assert!(!FirmwareVersion { major: 7, minor: 0 }.is_compatible());
        assert!(!FirmwareVersion { major: 9, minor: 0 }.is_compatible());
    }

    #[test]
    fn the_expected_version_matches_the_reference() {
        // B200_FW_COMPAT_NUM_MAJOR / _MINOR in b200_impl.hpp, UHD v4.10.0.0.
        assert_eq!(B200_FW_COMPAT, FirmwareVersion { major: 8, minor: 0 });
    }

    #[test]
    fn vendor_request_codes_match_the_reference() {
        // Transcription check against b200_iface.cpp. A wrong code here is a
        // silent stall on hardware and nothing at all in CI, so it is worth
        // pinning even though the test looks like a tautology.
        assert_eq!(FX3_FIRMWARE_LOAD, 0xA0);
        assert_eq!(VREQ_FPGA_START, 0x02);
        assert_eq!(VREQ_FPGA_DATA, 0x12);
        assert_eq!(VREQ_GET_COMPAT, 0x15);
        assert_eq!(VREQ_SET_FPGA_HASH, 0x1C);
        assert_eq!(VREQ_GET_FPGA_HASH, 0x1D);
        assert_eq!(VREQ_SET_FW_HASH, 0x1E);
        assert_eq!(VREQ_GET_FW_HASH, 0x1F);
        assert_eq!(VREQ_LOOP, 0x22);
        assert_eq!(VREQ_FPGA_CONFIG, 0x55);
        assert_eq!(VREQ_GPIF_RESET, 0x72);
        assert_eq!(VREQ_GET_USB, 0x80);
        assert_eq!(VREQ_GET_STATUS, 0x83);
        assert_eq!(VREQ_FX3_RESET, 0x99);
        assert_eq!(VREQ_EEPROM_WRITE, 0xBA);
        assert_eq!(VREQ_EEPROM_READ, 0xBB);
    }

    #[test]
    fn fx3_states_are_named_the_way_uhd_names_them() {
        assert_eq!(Fx3State::from(0x01), Fx3State::FpgaReady);
        assert_eq!(Fx3State::from(0x05).to_string(), "unconfigured");
        assert_eq!(Fx3State::from(0x42), Fx3State::Unknown(0x42));
        assert_eq!(Fx3State::from(0x42).to_string(), "unknown (0x42)");
    }

    #[test]
    fn the_reenumeration_timeout_matches_the_reference() {
        assert_eq!(REENUMERATION_TIMEOUT, Duration::from_millis(3000));
    }
}
