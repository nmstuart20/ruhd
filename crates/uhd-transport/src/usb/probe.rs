//! Enumerating the USB bus and classifying what is on it.

use std::time::{Duration, Instant};

use nusb::MaybeFuture;
use tracing::trace;
use uhd_core::{Error, Result};

use crate::endpoint::UsbEndpoint;

use super::ids::{B2XX_VID_PIDS, B2xxProduct, FX3_BOOTLOADER_VID_PIDS, vid_pid_search_list};

/// Manufacturer strings a B2xx reports once its firmware is running.
///
/// This is UHD's `firmware_loaded()` test, from `libusb1_base.cpp`: the FX3
/// serves descriptors out of the EEPROM before firmware is loaded and out of
/// the firmware image afterwards, and the manufacturer string is the field that
/// differs. "Free Software Folks" is what the open-source firmware reports.
const FIRMWARE_MANUFACTURERS: [&str; 3] = [
    "Ettus Research LLC",
    "National Instruments Corp.",
    "Free Software Folks",
];

/// How far along a device is towards being usable.
///
/// A B2xx has no flash: the FX3 firmware is volatile and has to be pushed over
/// USB on every plug-in, and the FPGA image after that. Discovery therefore
/// finds devices in three quite different conditions, and the difference
/// decides what the caller has to do next.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
#[non_exhaustive]
pub enum DeviceState {
    /// Enumerated as a bare Cypress FX3, so the EEPROM has not been programmed
    /// (a factory-fresh board, or one whose EEPROM was erased).
    ///
    /// Nothing about the board's identity is knowable yet — not the product,
    /// not the serial — because those live in the EEPROM. Recovering one means
    /// writing the EEPROM over the bootloader's vendor requests, which is what
    /// `uhd_images_downloader` + `b2xx_fx3_utils -D` do in UHD.
    Uninitialized,

    /// Enumerated under an Ettus or NI VID/PID, so the EEPROM is good, but the
    /// descriptors still come from the EEPROM rather than from firmware.
    ///
    /// The usual state of a board that was just plugged in. Loading
    /// `usrp_b200_fw.hex` makes it re-enumerate as [`Ready`](Self::Ready),
    /// keeping the same VID/PID.
    NeedsFirmware,

    /// Firmware is running and the device is ready to be opened.
    ///
    /// The FPGA may still be unprogrammed; that is invisible from USB and is
    /// settled after the control endpoint is up.
    Ready,
}

impl DeviceState {
    /// Whether FX3 firmware has to be loaded before this device can be opened.
    ///
    /// True for both [`Uninitialized`](Self::Uninitialized) and
    /// [`NeedsFirmware`](Self::NeedsFirmware) — they differ in what else is
    /// needed, not in whether firmware is.
    #[must_use]
    pub fn needs_firmware(self) -> bool {
        !matches!(self, Self::Ready)
    }
}

impl std::fmt::Display for DeviceState {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.write_str(match self {
            Self::Uninitialized => "uninitialized",
            Self::NeedsFirmware => "needs firmware",
            Self::Ready => "ready",
        })
    }
}

/// What a device's descriptors say about it, before anything is opened.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct Classification {
    /// How far along the device is.
    pub state: DeviceState,
    /// Which product it is, when the VID/PID pins that down. `None` for a
    /// bootloader, and for the B200/B210's shared PID — see
    /// [`B2xxProduct::from_usb_id`].
    pub product: Option<B2xxProduct>,
}

/// Classifies a device from its USB descriptors alone. No I/O, no `unsafe`, no
/// device access — this is the whole of the decision procedure.
///
/// Returns `None` if the VID/PID is not one this driver claims.
///
/// # Platform note
///
/// `manufacturer` is `None` on Windows, which does not cache descriptor strings
/// for unopened devices. Firmware presence is then unknowable without opening
/// the device, and this reports [`DeviceState::Ready`] — the optimistic answer,
/// on the grounds that a caller who then fails to talk to the device learns the
/// truth immediately, whereas the pessimistic answer would send every Windows
/// user into a firmware reload they usually do not need.
#[must_use]
pub fn classify(vid: u16, pid: u16, manufacturer: Option<&str>) -> Option<Classification> {
    if FX3_BOOTLOADER_VID_PIDS.contains(&(vid, pid)) {
        return Some(Classification {
            state: DeviceState::Uninitialized,
            product: None,
        });
    }

    if !B2XX_VID_PIDS.contains(&(vid, pid)) {
        return None;
    }

    let state = match manufacturer {
        Some(m) if FIRMWARE_MANUFACTURERS.contains(&m) => DeviceState::Ready,
        Some(_) => DeviceState::NeedsFirmware,
        None if cfg!(windows) => DeviceState::Ready,
        None => DeviceState::NeedsFirmware,
    };

    Some(Classification {
        state,
        product: B2xxProduct::from_usb_id(vid, pid),
    })
}

/// Enough to tell two devices apart *at one moment in time*.
///
/// Deliberately not a stable device identity — there is no such thing over a
/// firmware load, as [`wait_for_state`] documents at length. Bus address and
/// serial both change when a B2xx re-enumerates. What this is good for is the
/// opposite question: "is this the same enumeration record I saw a moment
/// ago?", which is what makes a before/after diff possible.
#[derive(Debug, Clone, PartialEq, Eq, Hash)]
pub struct BusIdentity {
    /// Bus identifier as the OS spells it.
    pub bus_id: String,
    /// Device address on that bus.
    pub address: u8,
    /// Serial from the descriptors, when there is one.
    pub serial: Option<String>,
}

/// One device found on the bus, with everything the descriptors gave up.
///
/// Holds no OS handle: building one opens nothing, and the device may be
/// unplugged by the time it is used.
#[derive(Debug, Clone)]
pub struct UsbProbe {
    /// Enough to find this device again and open it.
    pub endpoint: UsbEndpoint,
    /// How far along the device is.
    pub state: DeviceState,
    /// The product, when USB alone identifies it.
    pub product: Option<B2xxProduct>,
    /// `iManufacturer`, which is what [`state`](Self::state) is decided from.
    pub manufacturer: Option<String>,
    /// `iProduct`, for display only — it is a marketing string, not an ID.
    pub product_string: Option<String>,
    /// Bus identifier, as the OS spells it (a number on Linux, a path
    /// elsewhere). With [`address`](Self::address) this names the physical
    /// port, which is the only handle on a device with no serial.
    pub bus_id: String,
    /// Device address on the bus.
    pub address: u8,

    /// Kept so [`check_access`](Self::check_access) can reopen without a
    /// second enumeration. Private: `nusb` types do not belong in this API
    /// until there is a link to hand out.
    info: nusb::DeviceInfo,
}

impl UsbProbe {
    /// USB vendor ID.
    #[must_use]
    pub fn vid(&self) -> u16 {
        self.endpoint.vid
    }

    /// USB product ID.
    #[must_use]
    pub fn pid(&self) -> u16 {
        self.endpoint.pid
    }

    /// Serial from the USB descriptors, when the device has one.
    ///
    /// A device in [`DeviceState::Uninitialized`] generally has none: the
    /// serial lives in the EEPROM that has not been written yet.
    #[must_use]
    pub fn serial(&self) -> Option<&str> {
        self.endpoint.serial.as_deref()
    }

    /// Whether FX3 firmware has to be loaded before this device can be opened.
    #[must_use]
    pub fn needs_firmware(&self) -> bool {
        self.state.needs_firmware()
    }

    /// What to call this device in output.
    ///
    /// The product name when USB identifies it, `"B200 or B210"` for the PID
    /// those two share, and `"uninitialized B2xx"` for a bootloader.
    #[must_use]
    pub fn describe(&self) -> &'static str {
        match (self.product, self.state) {
            (Some(product), _) => product.name(),
            (None, DeviceState::Uninitialized) => "uninitialized B2xx",
            (None, _) => "B200 or B210",
        }
    }

    /// What distinguishes this device from the others on the bus right now.
    ///
    /// See [`BusIdentity`] for why "right now" is the load-bearing part.
    #[must_use]
    pub fn identity(&self) -> BusIdentity {
        BusIdentity {
            bus_id: self.bus_id.clone(),
            address: self.address,
            serial: self.endpoint.serial.clone(),
        }
    }

    /// The enumeration record this device was built from.
    ///
    /// Crate-private on purpose: `nusb` types are not part of this API, and
    /// the only thing inside the crate that needs one is
    /// [`Fx3Iface::open`](super::fx3::Fx3Iface::open).
    pub(crate) fn device_info(&self) -> &nusb::DeviceInfo {
        &self.info
    }

    /// Opens the device and closes it again, to find out whether this process
    /// is allowed to talk to it at all.
    ///
    /// On Linux, enumeration reads sysfs and always succeeds, but opening goes
    /// through `/dev/bus/usb`, which is root-only unless udev has been told
    /// otherwise. So a device can be listed in full detail and still be
    /// unusable, and this is the call that says which. The error names the
    /// fix; see also [the module docs](super#udev-rules-for-non-root-access).
    ///
    /// # Errors
    ///
    /// [`Error::Usb`] if the device cannot be opened — no permission, already
    /// claimed by another process, or unplugged since it was probed.
    pub fn check_access(&self) -> Result<()> {
        self.info
            .open()
            .wait()
            .map(drop)
            .map_err(|e| self.open_error(&e))
    }

    /// Turns a failed open into an error that says what to do about it.
    ///
    /// Permission denied is the common first-run failure, so it names the
    /// rules file and the recipe that installs it. A board in the FX3
    /// bootloader gets the opt-in bootloader rule instead, because the normal
    /// rules (UHD's included) do not cover that VID/PID.
    pub(crate) fn open_error(&self, e: &nusb::Error) -> Error {
        if e.kind() != nusb::ErrorKind::PermissionDenied {
            return Error::Usb(format!("cannot open {}: {e}", self.endpoint));
        }
        let fix = if FX3_BOOTLOADER_VID_PIDS.contains(&(self.vid(), self.pid())) {
            "this board is in the Cypress FX3 bootloader, which the usual udev rules \
             do not cover; run `just udev-install-bootloader` (udev/61-ruhd-fx3-bootloader.rules) \
             or use sudo"
        } else {
            "install the udev rules with `just udev-install` (udev/60-ruhd-b2xx.rules), \
             then replug; over SSH also join the usrp group. See docs/hardware.md"
        };
        Error::Usb(format!(
            "permission denied opening {}; {fix}",
            self.endpoint
        ))
    }
}

impl std::fmt::Display for UsbProbe {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "{} [{}]", self.describe(), self.state)?;
        if let Some(serial) = self.serial() {
            write!(f, " serial={serial}")?;
        }
        write!(f, " {:04x}:{:04x}", self.vid(), self.pid())
    }
}

/// Which devices [`probe_filtered`] should return.
///
/// An empty filter (`UsbFilter::default()`) means every B2xx and every FX3
/// bootloader. Fields narrow rather than widen: setting `vid` alone still
/// searches all the B2xx PIDs under that VID, exactly as a bare `vid=` in a
/// UHD device-args string does.
#[derive(Debug, Clone, Default, PartialEq, Eq)]
pub struct UsbFilter {
    /// Restrict to this vendor ID.
    pub vid: Option<u16>,
    /// Restrict to this product ID.
    pub pid: Option<u16>,
    /// Restrict to this serial. Compared case-insensitively, because B2xx
    /// serials are hex and get written down both ways.
    pub serial: Option<String>,
}

impl UsbFilter {
    /// An empty filter: everything this driver recognises.
    #[must_use]
    pub fn new() -> Self {
        Self::default()
    }

    /// Restricts to one vendor ID.
    #[must_use]
    pub fn with_vid(mut self, vid: u16) -> Self {
        self.vid = Some(vid);
        self
    }

    /// Restricts to one product ID.
    #[must_use]
    pub fn with_pid(mut self, pid: u16) -> Self {
        self.pid = Some(pid);
        self
    }

    /// Restricts to one serial.
    #[must_use]
    pub fn with_serial(mut self, serial: impl Into<String>) -> Self {
        self.serial = Some(serial.into());
        self
    }

    /// Whether a device with these descriptors passes the filter.
    fn matches(&self, vid: u16, pid: u16, serial: Option<&str>) -> bool {
        if !vid_pid_search_list(self.vid, self.pid).contains(&(vid, pid)) {
            return false;
        }
        match (&self.serial, serial) {
            (None, _) => true,
            (Some(wanted), Some(found)) => wanted.eq_ignore_ascii_case(found),
            (Some(_), None) => false,
        }
    }
}

/// Enumerates the bus and returns every B2xx and every FX3 bootloader on it.
///
/// # Errors
///
/// [`Error::Usb`] if the platform's USB enumeration fails outright. An empty
/// vector is not an error: it means nothing matched.
pub fn probe() -> Result<Vec<UsbProbe>> {
    probe_filtered(&UsbFilter::new())
}

/// Enumerates the bus and returns the devices matching `filter`.
///
/// Results are sorted by bus and address, so repeated probes of an unchanged
/// bus list devices in the same order.
///
/// # Errors
///
/// [`Error::Usb`] if the platform's USB enumeration fails outright.
pub fn probe_filtered(filter: &UsbFilter) -> Result<Vec<UsbProbe>> {
    let devices = nusb::list_devices()
        .wait()
        .map_err(|e| Error::Usb(format!("cannot enumerate the usb bus: {e}")))?;

    let mut found: Vec<UsbProbe> = devices
        .filter(|info| filter.matches(info.vendor_id(), info.product_id(), info.serial_number()))
        .filter_map(|info| {
            let class = classify(
                info.vendor_id(),
                info.product_id(),
                info.manufacturer_string(),
            )?;
            Some(UsbProbe {
                endpoint: UsbEndpoint {
                    vid: info.vendor_id(),
                    pid: info.product_id(),
                    serial: info.serial_number().map(str::to_owned),
                    // `bus_id` is only a number on Linux; elsewhere the pair
                    // below is the portable way to name the port.
                    bus: info.bus_id().parse().ok(),
                    address: Some(info.device_address()),
                },
                state: class.state,
                product: class.product,
                manufacturer: info.manufacturer_string().map(str::to_owned),
                product_string: info.product_string().map(str::to_owned),
                bus_id: info.bus_id().to_owned(),
                address: info.device_address(),
                info,
            })
        })
        .collect();

    found.sort_by(|a, b| {
        (&a.bus_id, a.address)
            .cmp(&(&b.bus_id, b.address))
            .then_with(|| a.endpoint.serial.cmp(&b.endpoint.serial))
    });
    Ok(found)
}

/// How often [`wait_for_state`] re-enumerates while waiting.
///
/// Enumeration is a sysfs walk, so this is cheap; the number is chosen to
/// resolve the wait finely enough that the reported duration is a useful
/// diagnostic rather than to keep polling costs down.
pub const REDISCOVERY_POLL_INTERVAL: Duration = Duration::from_millis(50);

/// Waits for a device to appear on the bus in a particular state.
///
/// This is the rediscovery half of a firmware load, and it is fiddlier than it
/// sounds, because **almost nothing about a B2xx's USB identity survives the
/// firmware boundary**. Measured on a B205mini, across a single
/// [`NeedsFirmware`](DeviceState::NeedsFirmware) →
/// [`Ready`](DeviceState::Ready) transition:
///
/// | | Before | After |
/// | --- | --- | --- |
/// | VID:PID | `2500:0022` | `2500:0022` |
/// | serial | `0000000004BE` | `3168B41` |
/// | manufacturer | `Cypress` | `Ettus Research LLC` |
/// | product | `WestBridge` | `USRP B205-mini` |
/// | bus / address | `001` / `14` | `002` / `7` |
///
/// The serial changes because *every* string descriptor changes: before
/// firmware the FX3 serves them out of its own EEPROM, and afterwards out of
/// the firmware image, which reports the USRP serial it read from the
/// motherboard EEPROM over I²C. The bus changes because the boot ROM
/// enumerates at high speed while the firmware brings up SuperSpeed, and Linux
/// puts those on different buses. The VID/PID is all that holds still, and
/// that identifies the *model*, not the device.
///
/// So the primary key is elimination, not matching. `exclude` is the set of
/// devices already in `state` before the operation started; anything reaching
/// `state` that is not in it must be the device that just transitioned.
/// `serial` is still honoured, and taken first when it matches, because it is
/// the stronger signal on the transitions where the serial *does* survive —
/// but it can never be the only one.
///
/// Returns as soon as a match is found; the bus is polled every
/// [`REDISCOVERY_POLL_INTERVAL`] until then.
///
/// # Errors
///
/// [`Error::Timeout`] if nothing reaches `state` within `timeout`.
/// [`Error::Usb`] if enumeration fails right at the deadline, or if more than
/// one unaccounted-for device reaches `state` — with several boards attached
/// and no serial to tell them apart, guessing would be worse than failing.
/// A device merely *missing* from the bus is not an error while the deadline
/// has not passed: that is the normal condition for most of a re-enumeration.
pub fn wait_for_state(
    serial: Option<&str>,
    exclude: &[BusIdentity],
    state: DeviceState,
    timeout: Duration,
) -> Result<UsbProbe> {
    let deadline = Instant::now() + timeout;
    loop {
        // A device caught mid-re-enumeration can make the platform's
        // enumeration fail outright rather than just come back empty. That is
        // transient by definition here, so it is retried rather than raised.
        match probe_filtered(&UsbFilter::new()) {
            Ok(mut devices) => {
                let view: Vec<(DeviceState, BusIdentity)> =
                    devices.iter().map(|d| (d.state, d.identity())).collect();
                if let Some(i) = choose_transitioned(&view, state, exclude, serial)? {
                    return Ok(devices.swap_remove(i));
                }
            }
            Err(e) if Instant::now() >= deadline => return Err(e),
            Err(e) => trace!("enumeration failed while waiting, retrying: {e}"),
        }

        let now = Instant::now();
        if now >= deadline {
            return Err(Error::Timeout(timeout));
        }
        std::thread::sleep(REDISCOVERY_POLL_INTERVAL.min(deadline - now));
    }
}

/// Decides which device on the bus is the one that just re-enumerated.
///
/// The whole of [`wait_for_state`]'s judgement, separated from the polling and
/// the USB so that it can be tested: `UsbProbe` wraps an OS handle that cannot
/// be conjured up in a unit test, but the decision it feeds is exactly the
/// part worth testing.
///
/// `Ok(None)` means "nothing yet, keep waiting" — which is the answer for most
/// of a re-enumeration and is emphatically not a failure.
fn choose_transitioned(
    devices: &[(DeviceState, BusIdentity)],
    state: DeviceState,
    exclude: &[BusIdentity],
    serial: Option<&str>,
) -> Result<Option<usize>> {
    let candidates: Vec<usize> = devices
        .iter()
        .enumerate()
        .filter(|(_, (s, id))| *s == state && !exclude.contains(id))
        .map(|(i, _)| i)
        .collect();

    // A surviving serial is the stronger answer when there is one, so it wins
    // over the count. On a B2xx firmware transition there never is one.
    if let Some(wanted) = serial
        && let Some(&i) = candidates.iter().find(|&&i| {
            devices[i]
                .1
                .serial
                .as_deref()
                .is_some_and(|s| s.eq_ignore_ascii_case(wanted))
        })
    {
        return Ok(Some(i));
    }

    match candidates.len() {
        0 => Ok(None),
        1 => Ok(Some(candidates[0])),
        n => Err(Error::Usb(format!(
            "{n} devices reached `{state}` while waiting for one to re-enumerate, and \
             none carries the expected serial; cannot tell which is which"
        ))),
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::usb::ids::{
        B200_PID, B205MINI_PID, B210_NI_PID, ETTUS_VID, FX3_BOOTLOADER_PID, FX3_REENUM_PID,
        FX3_VID, NI_VID,
    };

    const ETTUS: Option<&str> = Some("Ettus Research LLC");
    const CYPRESS: Option<&str> = Some("Cypress");

    #[test]
    fn bootloader_is_uninitialized_whatever_it_calls_itself() {
        for (vid, pid) in FX3_BOOTLOADER_VID_PIDS {
            for manufacturer in [None, CYPRESS, ETTUS] {
                let class = classify(vid, pid, manufacturer).expect("recognised");
                assert_eq!(class.state, DeviceState::Uninitialized);
                assert_eq!(class.product, None);
                assert!(class.state.needs_firmware());
            }
        }
        assert_eq!(FX3_BOOTLOADER_PID, 0x00f3);
        assert_eq!(FX3_REENUM_PID, 0x00f0);
    }

    #[test]
    fn ettus_descriptors_mean_firmware_is_running() {
        let class = classify(ETTUS_VID, B205MINI_PID, ETTUS).expect("recognised");
        assert_eq!(class.state, DeviceState::Ready);
        assert_eq!(class.product, Some(B2xxProduct::B205Mini));
        assert!(!class.state.needs_firmware());
    }

    #[test]
    fn eeprom_descriptors_mean_firmware_is_not() {
        // Programmed EEPROM, so the Ettus VID/PID, but the FX3 is still
        // serving Cypress's strings: firmware has not been pushed yet.
        let class = classify(ETTUS_VID, B205MINI_PID, CYPRESS).expect("recognised");
        assert_eq!(class.state, DeviceState::NeedsFirmware);
        // The product is still known — it comes from the PID, not the firmware.
        assert_eq!(class.product, Some(B2xxProduct::B205Mini));
    }

    #[test]
    fn the_ni_oem_boards_are_recognised() {
        let class =
            classify(NI_VID, B210_NI_PID, Some("National Instruments Corp.")).expect("recognised");
        assert_eq!(class.state, DeviceState::Ready);
        assert_eq!(class.product, Some(B2xxProduct::B210));
    }

    #[test]
    fn open_firmware_counts_as_firmware() {
        let class = classify(ETTUS_VID, B200_PID, Some("Free Software Folks")).expect("recognised");
        assert_eq!(class.state, DeviceState::Ready);
        // 0x2500:0x0020 can't say which of the two it is.
        assert_eq!(class.product, None);
    }

    #[test]
    fn foreign_devices_are_not_ours() {
        assert!(classify(0x1d6b, 0x0002, Some("Linux Foundation")).is_none());
        assert!(
            classify(ETTUS_VID, 0x0002, ETTUS).is_none(),
            "B100 is not a B2xx"
        );
        assert!(
            classify(FX3_VID, 0x00bc, CYPRESS).is_none(),
            "other FX3 PID"
        );
    }

    #[test]
    fn empty_filter_matches_the_whole_family() {
        let filter = UsbFilter::new();
        assert!(filter.matches(ETTUS_VID, B205MINI_PID, Some("30A1B2C")));
        assert!(filter.matches(FX3_VID, FX3_BOOTLOADER_PID, None));
        assert!(!filter.matches(0x1d6b, 0x0002, None));
    }

    #[test]
    fn filters_narrow() {
        let by_vid = UsbFilter::new().with_vid(NI_VID);
        assert!(by_vid.matches(NI_VID, B210_NI_PID, None));
        assert!(!by_vid.matches(ETTUS_VID, B205MINI_PID, None));
        // A bare vid= still sweeps every B2xx PID under it.
        assert!(by_vid.matches(NI_VID, B200_PID, None));

        let by_pid = UsbFilter::new().with_pid(B205MINI_PID);
        assert!(by_pid.matches(ETTUS_VID, B205MINI_PID, None));
        assert!(!by_pid.matches(ETTUS_VID, B200_PID, None));

        // Both given is the escape hatch: it matches whatever was asked for,
        // even a pair not in the family.
        let exact = UsbFilter::new().with_vid(0x1234).with_pid(0x5678);
        assert!(exact.matches(0x1234, 0x5678, None));
    }

    #[test]
    fn serial_matching_ignores_case_and_needs_a_serial() {
        let filter = UsbFilter::new().with_serial("30a1b2c");
        assert!(filter.matches(ETTUS_VID, B205MINI_PID, Some("30A1B2C")));
        assert!(!filter.matches(ETTUS_VID, B205MINI_PID, Some("30A1B2D")));
        assert!(
            !filter.matches(FX3_VID, FX3_BOOTLOADER_PID, None),
            "a device with no serial cannot match one"
        );
    }

    fn id(bus: &str, address: u8, serial: Option<&str>) -> BusIdentity {
        BusIdentity {
            bus_id: bus.to_owned(),
            address,
            serial: serial.map(str::to_owned),
        }
    }

    /// The exact transition measured on a B205mini, as a fixture: the device
    /// leaves the bus as `001/14` with the FX3's serial and comes back as
    /// `002/7` with the USRP's. Nothing but the VID/PID is the same.
    #[test]
    fn a_b2xx_is_found_again_even_though_its_serial_changed() {
        let before = [id("001", 14, Some("0000000004BE"))];
        let after = [(DeviceState::Ready, id("002", 7, Some("3168B41")))];

        // The caller passes the pre-load serial, which no longer exists
        // anywhere on the bus. Elimination still finds the device.
        let chosen = choose_transitioned(&after, DeviceState::Ready, &[], Some("0000000004BE"))
            .expect("not ambiguous");
        assert_eq!(chosen, Some(0));

        // And the bootloader snapshot does not accidentally exclude it, since
        // the identity is wholly different.
        assert!(!before.contains(&after[0].1));
    }

    #[test]
    fn a_device_already_in_the_target_state_is_not_mistaken_for_the_new_one() {
        // Two boards attached; one was already running firmware before the
        // load started. Without the exclusion set, that one would be returned
        // immediately and the caller would talk to the wrong device.
        let already_ready = id("002", 3, Some("AAAAAAA"));
        let exclude = [already_ready.clone()];

        let during = [(DeviceState::Ready, already_ready.clone())];
        assert_eq!(
            choose_transitioned(&during, DeviceState::Ready, &exclude, None).expect("ok"),
            None,
            "the pre-existing device must not count as the one that transitioned"
        );

        let after = [
            (DeviceState::Ready, already_ready),
            (DeviceState::Ready, id("002", 9, Some("3168B41"))),
        ];
        assert_eq!(
            choose_transitioned(&after, DeviceState::Ready, &exclude, None).expect("ok"),
            Some(1)
        );
    }

    #[test]
    fn a_surviving_serial_wins_over_the_count() {
        // When the serial does survive, it picks the right one out of several
        // rather than failing as ambiguous.
        let devices = [
            (DeviceState::Ready, id("002", 3, Some("AAAAAAA"))),
            (DeviceState::Ready, id("002", 9, Some("3168B41"))),
        ];
        assert_eq!(
            choose_transitioned(&devices, DeviceState::Ready, &[], Some("3168b41")).expect("ok"),
            Some(1),
            "and case-insensitively, since b2xx serials get written down both ways"
        );
    }

    #[test]
    fn several_unaccounted_for_devices_is_an_error_not_a_guess() {
        let devices = [
            (DeviceState::Ready, id("002", 3, Some("AAAAAAA"))),
            (DeviceState::Ready, id("002", 9, Some("BBBBBBB"))),
        ];
        let e =
            choose_transitioned(&devices, DeviceState::Ready, &[], None).expect_err("ambiguous");
        assert!(e.to_string().contains("cannot tell which is which"), "{e}");
    }

    #[test]
    fn devices_in_other_states_are_ignored_while_waiting() {
        let devices = [
            (
                DeviceState::NeedsFirmware,
                id("001", 14, Some("0000000004BE")),
            ),
            (DeviceState::Uninitialized, id("001", 15, None)),
        ];
        assert_eq!(
            choose_transitioned(&devices, DeviceState::Ready, &[], None).expect("ok"),
            None
        );
        // And the mid-flight bootloader is exactly what the reset path waits for.
        assert_eq!(
            choose_transitioned(&devices, DeviceState::NeedsFirmware, &[], None).expect("ok"),
            Some(0)
        );
    }

    #[test]
    fn an_empty_bus_means_keep_waiting() {
        assert_eq!(
            choose_transitioned(&[], DeviceState::Ready, &[], Some("3168B41")).expect("ok"),
            None
        );
    }

    #[test]
    fn probing_a_machine_with_no_usrp_is_not_an_error() {
        // Runs in CI, where there is no hardware: what is asserted is that
        // enumeration works and classifies nothing as a USRP, not that some
        // particular device is absent.
        match probe() {
            Ok(devices) => {
                for device in devices {
                    assert!(
                        device.vid() == ETTUS_VID
                            || device.vid() == NI_VID
                            || device.vid() == FX3_VID
                    );
                }
            }
            Err(e) => {
                // Containers without a USB subsystem cannot enumerate at all.
                assert!(matches!(e, Error::Usb(_)), "unexpected error: {e}");
            }
        }
    }
}
