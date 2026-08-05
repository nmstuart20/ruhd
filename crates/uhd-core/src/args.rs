//! Device argument strings — the `type=b200,serial=30A1B2C` key/value soup that
//! UHD passes around as `device_addr_t`.
//!
//! Arguments are how a device is named without the API having to know what kind
//! of device it is. A USB device is picked out by `serial=`; a future
//! network-attached X310 by `addr=192.168.10.2`. Neither the [`Device`] trait
//! nor the [`Registry`] has to change to accommodate the second case.
//!
//! [`Device`]: crate::device::Device
//! [`Registry`]: crate::registry::Registry

use std::collections::BTreeMap;
use std::fmt;
use std::str::FromStr;

use crate::error::{Error, Result};

/// An ordered set of `key=value` device arguments.
///
/// Ordering is deterministic (keys sort lexicographically) so that `Display`
/// output is stable and usable as a cache key.
#[derive(Debug, Clone, Default, PartialEq, Eq)]
pub struct DeviceArgs {
    map: BTreeMap<String, String>,
}

impl DeviceArgs {
    /// Creates an empty argument set.
    #[must_use]
    pub fn new() -> Self {
        Self::default()
    }

    /// Returns the raw value for `key`, if present.
    #[must_use]
    pub fn get(&self, key: &str) -> Option<&str> {
        self.map.get(key).map(String::as_str)
    }

    /// Returns the value for `key`, or [`Error::KeyNotFound`].
    pub fn require(&self, key: &str) -> Result<&str> {
        self.get(key)
            .ok_or_else(|| Error::KeyNotFound(key.to_owned()))
    }

    /// Returns the value for `key` parsed as `T`.
    ///
    /// `Ok(None)` means the key was absent; a parse failure is an
    /// [`Error::InvalidArg`].
    pub fn get_parsed<T>(&self, key: &str) -> Result<Option<T>>
    where
        T: FromStr,
        T::Err: fmt::Display,
    {
        match self.get(key) {
            None => Ok(None),
            Some(raw) => raw
                .parse::<T>()
                .map(Some)
                .map_err(|e| Error::InvalidArg(format!("{key}={raw}: {e}"))),
        }
    }

    /// Returns the value for `key` parsed as `T`, or `default` if absent.
    pub fn get_or<T>(&self, key: &str, default: T) -> Result<T>
    where
        T: FromStr,
        T::Err: fmt::Display,
    {
        Ok(self.get_parsed(key)?.unwrap_or(default))
    }

    /// Inserts or replaces `key`, returning `self` for chaining.
    #[must_use]
    pub fn with(mut self, key: impl Into<String>, value: impl Into<String>) -> Self {
        self.set(key, value);
        self
    }

    /// Inserts or replaces `key`, returning the previous value.
    pub fn set(&mut self, key: impl Into<String>, value: impl Into<String>) -> Option<String> {
        self.map.insert(key.into(), value.into())
    }

    /// Removes `key`, returning its value.
    pub fn remove(&mut self, key: &str) -> Option<String> {
        self.map.remove(key)
    }

    /// Returns `true` if `key` is present.
    #[must_use]
    pub fn contains(&self, key: &str) -> bool {
        self.map.contains_key(key)
    }

    /// Returns `true` if there are no arguments at all.
    #[must_use]
    pub fn is_empty(&self) -> bool {
        self.map.is_empty()
    }

    /// Returns the number of arguments.
    #[must_use]
    pub fn len(&self) -> usize {
        self.map.len()
    }

    /// Iterates over `(key, value)` pairs in key order.
    pub fn iter(&self) -> impl Iterator<Item = (&str, &str)> {
        self.map.iter().map(|(k, v)| (k.as_str(), v.as_str()))
    }

    /// Adds every entry of `other` that is not already present.
    pub fn merge_defaults(&mut self, other: &Self) {
        for (k, v) in other.iter() {
            self.map.entry(k.to_owned()).or_insert_with(|| v.to_owned());
        }
    }

    /// Returns `true` if every key `self` declares also matches in `candidate`.
    ///
    /// This is the discovery predicate: a hint of `type=b200` matches any B-series
    /// device, and `type=b200,serial=30A1B2C` matches exactly one. Keys absent
    /// from `self` are ignored, so a hint is always a *subset* test.
    #[must_use]
    pub fn matches(&self, candidate: &Self) -> bool {
        self.iter()
            .all(|(k, v)| candidate.get(k).is_some_and(|c| c.eq_ignore_ascii_case(v)))
    }
}

impl FromStr for DeviceArgs {
    type Err = Error;

    /// Parses `key=value,key2=value2`.
    ///
    /// Whitespace around keys and values is trimmed, empty segments are skipped,
    /// and a bare segment with no `=` is an error (rather than UHD's silent
    /// `key=""`, which hides typos).
    fn from_str(s: &str) -> Result<Self> {
        let mut args = Self::new();
        for segment in s.split(',') {
            let segment = segment.trim();
            if segment.is_empty() {
                continue;
            }
            let (key, value) = segment
                .split_once('=')
                .ok_or_else(|| Error::InvalidArg(format!("`{segment}` is not `key=value`")))?;
            let key = key.trim();
            if key.is_empty() {
                return Err(Error::InvalidArg(format!("`{segment}` has an empty key")));
            }
            args.set(key, value.trim());
        }
        Ok(args)
    }
}

impl fmt::Display for DeviceArgs {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        for (i, (k, v)) in self.iter().enumerate() {
            if i > 0 {
                f.write_str(",")?;
            }
            write!(f, "{k}={v}")?;
        }
        Ok(())
    }
}

impl<K, V> FromIterator<(K, V)> for DeviceArgs
where
    K: Into<String>,
    V: Into<String>,
{
    fn from_iter<I: IntoIterator<Item = (K, V)>>(iter: I) -> Self {
        Self {
            map: iter
                .into_iter()
                .map(|(k, v)| (k.into(), v.into()))
                .collect(),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn parses_and_round_trips() {
        let args: DeviceArgs = "type=b200, serial=30A1B2C".parse().unwrap();
        assert_eq!(args.get("type"), Some("b200"));
        assert_eq!(args.get("serial"), Some("30A1B2C"));
        // Keys come back sorted, not in input order.
        assert_eq!(args.to_string(), "serial=30A1B2C,type=b200");
    }

    #[test]
    fn empty_segments_are_skipped() {
        let args: DeviceArgs = ",,type=b200,".parse().unwrap();
        assert_eq!(args.len(), 1);
    }

    #[test]
    fn bare_segment_is_rejected() {
        assert!("type=b200,oops".parse::<DeviceArgs>().is_err());
        assert!("=b200".parse::<DeviceArgs>().is_err());
    }

    #[test]
    fn hint_is_a_subset_test() {
        let device: DeviceArgs = "type=b200,serial=30A1B2C,product=B205mini".parse().unwrap();
        assert!("".parse::<DeviceArgs>().unwrap().matches(&device));
        assert!("type=B200".parse::<DeviceArgs>().unwrap().matches(&device));
        assert!(
            !"serial=DEADBEEF"
                .parse::<DeviceArgs>()
                .unwrap()
                .matches(&device)
        );
        // A network hint must not match a USB device.
        assert!(
            !"addr=192.168.10.2"
                .parse::<DeviceArgs>()
                .unwrap()
                .matches(&device)
        );
    }

    #[test]
    fn typed_lookup() {
        let args: DeviceArgs = "spp=1996,rate=1e6".parse().unwrap();
        assert_eq!(args.get_parsed::<usize>("spp").unwrap(), Some(1996));
        assert_eq!(args.get_or("num_recv_frames", 16usize).unwrap(), 16);
        assert!(args.get_parsed::<usize>("rate").is_err());
    }
}
