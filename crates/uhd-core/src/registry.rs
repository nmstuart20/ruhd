//! Device discovery and construction.
//!
//! Backends register a [`DeviceFactory`]; applications call [`find`] and
//! [`open`] and never name a backend crate. This is the seam that keeps
//! `uhd-core` free of any knowledge of USB, UDP, or any particular product —
//! adding X310-over-UDP later means adding a factory, not editing this module.

use std::sync::{Arc, OnceLock, RwLock};

use tracing::{debug, warn};

use crate::args::DeviceArgs;
use crate::device::{Device, DeviceInfo};
use crate::error::{Error, Result};

/// Discovers and opens devices of one family.
pub trait DeviceFactory: Send + Sync + std::fmt::Debug {
    /// The `type=` value this factory answers to, e.g. `b200`.
    fn device_type(&self) -> &'static str;

    /// Enumerates devices matching `hint`.
    ///
    /// An empty hint means "everything you can see". A factory must return an
    /// empty vector — not an error — when nothing matches; errors are for
    /// enumeration itself failing, such as a USB context that would not open.
    fn find(&self, hint: &DeviceArgs) -> Result<Vec<DeviceInfo>>;

    /// Opens the single device described by `args`.
    ///
    /// `args` is normally a [`DeviceInfo::args`] handed back by
    /// [`DeviceFactory::find`], which is guaranteed unambiguous.
    fn open(&self, args: &DeviceArgs) -> Result<Arc<dyn Device>>;
}

/// A set of registered [`DeviceFactory`] implementations.
#[derive(Debug, Default)]
pub struct Registry {
    factories: Vec<Arc<dyn DeviceFactory>>,
}

impl Registry {
    /// An empty registry.
    #[must_use]
    pub fn new() -> Self {
        Self::default()
    }

    /// Adds a factory, replacing any previous one with the same
    /// [`DeviceFactory::device_type`].
    pub fn register(&mut self, factory: Arc<dyn DeviceFactory>) {
        let device_type = factory.device_type();
        if let Some(slot) = self
            .factories
            .iter_mut()
            .find(|f| f.device_type() == device_type)
        {
            warn!(
                device_type,
                "replacing an already-registered device factory"
            );
            *slot = factory;
        } else {
            debug!(device_type, "registered device factory");
            self.factories.push(factory);
        }
    }

    /// The registered factory type names.
    #[must_use]
    pub fn device_types(&self) -> Vec<&'static str> {
        self.factories.iter().map(|f| f.device_type()).collect()
    }

    /// Enumerates every device matching `hint` across all factories.
    ///
    /// A `type=` in the hint restricts which factories are consulted. A factory
    /// that fails to enumerate is logged and skipped rather than failing the
    /// whole search — one wedged USB device should not hide a working network
    /// one.
    pub fn find(&self, hint: &DeviceArgs) -> Result<Vec<DeviceInfo>> {
        let wanted_type = hint.get("type");
        let mut found = Vec::new();
        for factory in &self.factories {
            if wanted_type.is_some_and(|t| !t.eq_ignore_ascii_case(factory.device_type())) {
                continue;
            }
            match factory.find(hint) {
                Ok(devices) => found.extend(devices),
                Err(e) => warn!(
                    device_type = factory.device_type(),
                    error = %e,
                    "device enumeration failed; skipping this factory"
                ),
            }
        }
        Ok(found)
    }

    /// Opens the one device matching `args`.
    ///
    /// Returns [`Error::NoDeviceFound`] if nothing matched and
    /// [`Error::AmbiguousDevice`] if more than one did.
    /// The factory receives the discovered identity together with the caller's
    /// remaining options, such as a requested master clock rate.
    pub fn open(&self, args: &DeviceArgs) -> Result<Arc<dyn Device>> {
        let candidates = self.find(args)?;
        match candidates.len() {
            0 => Err(Error::NoDeviceFound(args.clone())),
            1 => {
                let info = &candidates[0];
                let factory = self
                    .factories
                    .iter()
                    .find(|f| f.device_type() == info.device_type)
                    .ok_or_else(|| {
                        Error::internal(format!(
                            "factory `{}` enumerated a device then vanished",
                            info.device_type
                        ))
                    })?;
                // Discovery supplies the canonical identity; retain options
                // such as master_clock_rate that are not discovery predicates.
                let mut resolved = info.args.clone();
                resolved.merge_defaults(args);
                factory.open(&resolved)
            }
            count => Err(Error::AmbiguousDevice {
                hint: args.clone(),
                count,
            }),
        }
    }
}

/// The process-wide registry.
fn global() -> &'static RwLock<Registry> {
    static GLOBAL: OnceLock<RwLock<Registry>> = OnceLock::new();
    GLOBAL.get_or_init(|| RwLock::new(Registry::new()))
}

/// Registers `factory` in the process-wide registry.
///
/// Backend crates expose a `register()` that calls this; `uhd-cli` calls each
/// one at start-up. Deliberately explicit rather than automatic at link time:
/// link-time registration tricks are fragile under `--gc-sections` and make it
/// impossible to run a test against a stub device.
///
/// # Panics
///
/// If the registry lock was poisoned by a panic in another thread.
pub fn register(factory: Arc<dyn DeviceFactory>) {
    global()
        .write()
        .expect("device registry lock poisoned")
        .register(factory);
}

/// Enumerates devices matching `hint` using the process-wide registry.
///
/// # Panics
///
/// If the registry lock was poisoned by a panic in another thread.
pub fn find(hint: &DeviceArgs) -> Result<Vec<DeviceInfo>> {
    global()
        .read()
        .expect("device registry lock poisoned")
        .find(hint)
}

/// Opens the one device matching `args` using the process-wide registry.
///
/// # Panics
///
/// If the registry lock was poisoned by a panic in another thread.
pub fn open(args: &DeviceArgs) -> Result<Arc<dyn Device>> {
    global()
        .read()
        .expect("device registry lock poisoned")
        .open(args)
}

/// The device types registered process-wide.
///
/// # Panics
///
/// If the registry lock was poisoned by a panic in another thread.
#[must_use]
pub fn registered_types() -> Vec<&'static str> {
    global()
        .read()
        .expect("device registry lock poisoned")
        .device_types()
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A factory that pretends to see `serials` worth of devices.
    #[derive(Debug)]
    struct StubFactory {
        device_type: &'static str,
        serials: Vec<&'static str>,
    }

    impl DeviceFactory for StubFactory {
        fn device_type(&self) -> &'static str {
            self.device_type
        }

        fn find(&self, hint: &DeviceArgs) -> Result<Vec<DeviceInfo>> {
            Ok(self
                .serials
                .iter()
                .map(|serial| DeviceInfo {
                    device_type: self.device_type.to_owned(),
                    product: "stub".to_owned(),
                    serial: (*serial).to_owned(),
                    args: DeviceArgs::new()
                        .with("type", self.device_type)
                        .with("serial", *serial),
                    ..Default::default()
                })
                .filter(|info| hint.matches(&info.args))
                .collect())
        }

        fn open(&self, args: &DeviceArgs) -> Result<Arc<dyn Device>> {
            Err(Error::not_supported(format!("opening stub `{args}`")))
        }
    }

    fn registry() -> Registry {
        let mut r = Registry::new();
        r.register(Arc::new(StubFactory {
            device_type: "b200",
            serials: vec!["30A1B2C", "30D4E5F"],
        }));
        r.register(Arc::new(StubFactory {
            device_type: "x300",
            serials: vec!["31FFFFF"],
        }));
        r
    }

    #[test]
    fn empty_hint_finds_every_registered_family() {
        let found = registry().find(&DeviceArgs::new()).unwrap();
        assert_eq!(found.len(), 3);
    }

    #[test]
    fn type_hint_restricts_to_one_factory() {
        let hint: DeviceArgs = "type=x300".parse().unwrap();
        let found = registry().find(&hint).unwrap();
        assert_eq!(found.len(), 1);
        assert_eq!(found[0].serial, "31FFFFF");
    }

    #[test]
    fn open_reports_ambiguity_rather_than_guessing() {
        let hint: DeviceArgs = "type=b200".parse().unwrap();
        let err = registry().open(&hint).unwrap_err();
        assert!(
            matches!(err, Error::AmbiguousDevice { count: 2, .. }),
            "got {err:?}"
        );
    }

    #[test]
    fn open_reports_when_nothing_matches() {
        let hint: DeviceArgs = "serial=NOPE".parse().unwrap();
        let err = registry().open(&hint).unwrap_err();
        assert!(matches!(err, Error::NoDeviceFound(_)), "got {err:?}");
    }

    #[test]
    fn unique_hint_reaches_the_factory() {
        let hint: DeviceArgs = "serial=30A1B2C".parse().unwrap();
        // The stub always refuses to open, but reaching NotSupported proves the
        // hint resolved to exactly one device and dispatch found its factory.
        let err = registry().open(&hint).unwrap_err();
        assert!(matches!(err, Error::NotSupported(_)), "got {err:?}");
    }

    #[test]
    fn open_preserves_options_and_uses_the_discovered_identity() {
        #[derive(Debug)]
        struct CaptureFactory(std::sync::Mutex<Option<DeviceArgs>>);
        impl DeviceFactory for CaptureFactory {
            fn device_type(&self) -> &'static str {
                "b200"
            }
            fn find(&self, _hint: &DeviceArgs) -> Result<Vec<DeviceInfo>> {
                Ok(vec![DeviceInfo {
                    device_type: "b200".into(),
                    args: "type=b200,serial=CANONICAL".parse().unwrap(),
                    ..Default::default()
                }])
            }
            fn open(&self, args: &DeviceArgs) -> Result<Arc<dyn Device>> {
                *self.0.lock().unwrap() = Some(args.clone());
                Err(Error::not_supported("capture only"))
            }
        }
        let factory = Arc::new(CaptureFactory(std::sync::Mutex::new(None)));
        let mut registry = Registry::new();
        registry.register(factory.clone());
        let hint = "type=B200,serial=canonical,master_clock_rate=40e6,auto_flash=true"
            .parse()
            .unwrap();
        assert!(matches!(registry.open(&hint), Err(Error::NotSupported(_))));
        assert_eq!(
            factory.0.lock().unwrap().as_ref().unwrap().to_string(),
            "auto_flash=true,master_clock_rate=40e6,serial=CANONICAL,type=b200"
        );
    }

    #[test]
    fn re_registering_a_type_replaces_it() {
        let mut r = registry();
        r.register(Arc::new(StubFactory {
            device_type: "b200",
            serials: vec!["NEWONE"],
        }));
        assert_eq!(r.device_types().len(), 2);
        let found = r.find(&"type=b200".parse().unwrap()).unwrap();
        assert_eq!(found.len(), 1);
        assert_eq!(found[0].serial, "NEWONE");
    }
}
