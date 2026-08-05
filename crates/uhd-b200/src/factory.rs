//! Registry integration and read-only B2xx discovery.
//!
//! Discovery stops at the motherboard EEPROM: that is enough to report the
//! product, serial, name and revision without configuring the FPGA or codec.
//! Devices without FX3 firmware are intentionally left to `ruhd probe usb`
//! and `ruhd init`; a `find` operation must not make a device re-enumerate.

use std::sync::Arc;

use tracing::{debug, warn};
use uhd_core::registry::DeviceFactory;
use uhd_core::{Device, DeviceArgs, DeviceInfo, Error, Result, registry};
use uhd_transport::UsbEndpoint;
use uhd_transport::usb::{self, B2xxProduct, DeviceState, Fx3Iface, UsbFilter};

/// The `type=b200` registry backend.
#[derive(Debug, Default)]
pub struct B200Factory;

impl DeviceFactory for B200Factory {
    fn device_type(&self) -> &'static str {
        "b200"
    }

    fn find(&self, hint: &DeviceArgs) -> Result<Vec<DeviceInfo>> {
        find(hint)
    }

    fn open(&self, args: &DeviceArgs) -> Result<Arc<dyn Device>> {
        Err(Error::not_supported(format!(
            "opening `{args}` through uhd_core::Device (B2xx discovery is implemented, but the \
             generic Device and streamer implementation is not yet complete)"
        )))
    }
}

/// Registers the B2xx backend in the process-wide device registry.
pub fn register() {
    registry::register(Arc::new(B200Factory));
}

/// Finds firmware-ready B2xx devices without changing their state.
///
/// `type`, `product`, `serial`, `name`, `revision`, `vid`, `pid`, `bus` and
/// `address` are discovery predicates. `addr` and `resource` describe non-USB
/// transports and therefore produce no B2xx results. Other arguments are
/// open-time options and do not affect discovery.
pub fn find(hint: &DeviceArgs) -> Result<Vec<DeviceInfo>> {
    if hint
        .get("type")
        .is_some_and(|value| !value.eq_ignore_ascii_case("b200"))
        || hint.contains("addr")
        || hint.contains("resource")
    {
        return Ok(Vec::new());
    }

    let wanted_product = match hint.get("product") {
        Some(value) => match product_named(value) {
            Some(product) => Some(product),
            None => return Ok(Vec::new()),
        },
        None => None,
    };

    let filter = UsbFilter {
        vid: hex_u16_arg(hint, "vid")?,
        pid: hex_u16_arg(hint, "pid")?,
        serial: hint.get("serial").map(str::to_owned),
    };
    let wanted_bus = decimal_u8_arg(hint, "bus")?;
    let wanted_address = decimal_u8_arg(hint, "address")?;

    let mut found = Vec::new();
    for probe in usb::probe_filtered(&filter)? {
        if wanted_bus.is_some_and(|bus| probe.endpoint.bus != Some(bus))
            || wanted_address.is_some_and(|address| probe.endpoint.address != Some(address))
        {
            continue;
        }
        if probe.state != DeviceState::Ready {
            debug!(
                endpoint = %probe.endpoint,
                state = %probe.state,
                "not returning a b2xx from device discovery because its firmware is not running"
            );
            continue;
        }

        let eeprom = match Fx3Iface::open(&probe).and_then(|iface| usb::eeprom::read(&iface)) {
            Ok(eeprom) => eeprom,
            Err(error) => {
                // One claimed or unplugged device must not hide the other B2xx
                // devices on the bus.
                warn!(
                    endpoint = %probe.endpoint,
                    %error,
                    "could not read b2xx identity during discovery; skipping it"
                );
                continue;
            }
        };

        let info = device_info(
            &probe.endpoint,
            probe.product.or_else(|| eeprom.product()),
            probe.serial(),
            eeprom.serial.as_deref(),
            eeprom.name.as_deref(),
            eeprom.revision,
        );
        if matches_identity(hint, wanted_product, &info) {
            found.push(info);
        }
    }

    Ok(found)
}

fn device_info(
    endpoint: &UsbEndpoint,
    product: Option<B2xxProduct>,
    descriptor_serial: Option<&str>,
    eeprom_serial: Option<&str>,
    name: Option<&str>,
    revision: Option<u16>,
) -> DeviceInfo {
    let serial = descriptor_serial
        .or(eeprom_serial)
        .unwrap_or_default()
        .to_owned();
    let product = product.map_or_else(|| "B2??".to_owned(), |value| value.name().to_owned());

    // A programmed B2xx serial is its stable, unambiguous reopen key. Retain a
    // bus/address fallback for a damaged EEPROM so DeviceInfo::args still
    // identifies the enumeration record as precisely as the OS permits.
    let mut args = DeviceArgs::new().with("type", "b200");
    if serial.is_empty() {
        args.set("vid", format!("{:04x}", endpoint.vid));
        args.set("pid", format!("{:04x}", endpoint.pid));
        if let Some(bus) = endpoint.bus {
            args.set("bus", bus.to_string());
        }
        if let Some(address) = endpoint.address {
            args.set("address", address.to_string());
        }
    } else {
        args.set("serial", serial.clone());
    }

    DeviceInfo {
        device_type: "b200".to_owned(),
        product,
        serial,
        name: name.unwrap_or_default().to_owned(),
        revision: revision.map(|value| value.to_string()),
        args,
    }
}

fn matches_identity(
    hint: &DeviceArgs,
    wanted_product: Option<B2xxProduct>,
    info: &DeviceInfo,
) -> bool {
    if wanted_product.is_some_and(|product| !info.product.eq_ignore_ascii_case(product.name())) {
        return false;
    }
    for (key, actual) in [
        ("serial", info.serial.as_str()),
        ("name", info.name.as_str()),
        ("revision", info.revision.as_deref().unwrap_or_default()),
    ] {
        if hint
            .get(key)
            .is_some_and(|wanted| !wanted.eq_ignore_ascii_case(actual))
        {
            return false;
        }
    }
    true
}

fn product_named(name: &str) -> Option<B2xxProduct> {
    B2xxProduct::ALL
        .into_iter()
        .find(|product| product.name().eq_ignore_ascii_case(name))
}

fn hex_u16_arg(args: &DeviceArgs, key: &str) -> Result<Option<u16>> {
    let Some(raw) = args.get(key) else {
        return Ok(None);
    };
    let digits = raw
        .strip_prefix("0x")
        .or_else(|| raw.strip_prefix("0X"))
        .unwrap_or(raw);
    u16::from_str_radix(digits, 16).map(Some).map_err(|error| {
        Error::invalid_arg(format!("{key}={raw}: expected a 16-bit hex value: {error}"))
    })
}

fn decimal_u8_arg(args: &DeviceArgs, key: &str) -> Result<Option<u8>> {
    args.get_parsed(key)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn endpoint() -> UsbEndpoint {
        UsbEndpoint {
            vid: 0x2500,
            pid: 0x0022,
            serial: Some("3168B41".to_owned()),
            bus: Some(2),
            address: Some(7),
        }
    }

    #[test]
    fn identity_becomes_device_info_with_an_unambiguous_reopen_hint() {
        let info = device_info(
            &endpoint(),
            Some(B2xxProduct::B205Mini),
            Some("3168B41"),
            Some("3168B41"),
            Some("lab radio"),
            Some(3),
        );

        assert_eq!(info.device_type, "b200");
        assert_eq!(info.product, "B205mini");
        assert_eq!(info.serial, "3168B41");
        assert_eq!(info.name, "lab radio");
        assert_eq!(info.revision.as_deref(), Some("3"));
        assert_eq!(info.args.to_string(), "serial=3168B41,type=b200");
    }

    #[test]
    fn identity_filters_are_case_insensitive() {
        let info = device_info(
            &endpoint(),
            Some(B2xxProduct::B205Mini),
            Some("3168B41"),
            Some("3168B41"),
            Some("lab radio"),
            Some(3),
        );
        let hint: DeviceArgs = "product=b205MINI,serial=3168b41,name=LAB RADIO,revision=3"
            .parse()
            .unwrap();

        assert!(matches_identity(
            &hint,
            product_named(hint.get("product").unwrap()),
            &info
        ));
        assert!(!matches_identity(
            &"serial=wrong".parse().unwrap(),
            None,
            &info
        ));
    }

    #[test]
    fn vid_and_pid_hints_use_uhds_hex_convention() {
        let args: DeviceArgs = "vid=2500,pid=0x0022".parse().unwrap();
        assert_eq!(hex_u16_arg(&args, "vid").unwrap(), Some(0x2500));
        assert_eq!(hex_u16_arg(&args, "pid").unwrap(), Some(0x0022));
        assert!(hex_u16_arg(&"vid=nope".parse().unwrap(), "vid").is_err());
    }

    #[test]
    fn bus_and_address_hints_are_decimal() {
        let args: DeviceArgs = "bus=2,address=17".parse().unwrap();
        assert_eq!(decimal_u8_arg(&args, "bus").unwrap(), Some(2));
        assert_eq!(decimal_u8_arg(&args, "address").unwrap(), Some(17));
        assert!(decimal_u8_arg(&"bus=999".parse().unwrap(), "bus").is_err());
    }

    #[test]
    fn a_missing_serial_falls_back_to_the_usb_enumeration_record() {
        let mut endpoint = endpoint();
        endpoint.serial = None;
        let info = device_info(
            &endpoint,
            Some(B2xxProduct::B205Mini),
            None,
            None,
            None,
            None,
        );

        assert_eq!(
            info.args.to_string(),
            "address=7,bus=2,pid=0022,type=b200,vid=2500"
        );
    }
}
