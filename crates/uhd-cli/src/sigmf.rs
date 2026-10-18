//! SigMF metadata for `ruhd rx` recordings.
//!
//! `rx_samples_to_file` writes bare samples and leaves the frequency, rate and
//! gain for the user to remember. `ruhd rx` writes a `.sigmf-meta` file beside
//! the data so the recording describes itself, and marks every overflow and
//! sequence gap as an annotation at the sample where the discontinuity falls.
//!
//! Only the fields this recorder knows are written. Settings that SigMF core
//! has no field for (gain, bandwidth, antenna, device time) go under a `ruhd:`
//! extension that is declared optional, so readers that do not know it can
//! ignore it.

use std::path::{Path, PathBuf};
use std::time::{SystemTime, UNIX_EPOCH};

use serde_json::{Map, Value, json};
use uhd_core::SampleFormat;

/// SigMF specification version the metadata is written against.
const SIGMF_VERSION: &str = "1.2.0";

/// A discontinuity in the recording.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) enum EventKind {
    /// The device's buffer overflowed and samples were dropped.
    Overflow,
    /// Packets arrived out of sequence; samples between them are missing.
    SequenceGap,
    /// A packet could not be parsed and was skipped.
    BadPacket,
}

impl EventKind {
    fn label(self) -> &'static str {
        match self {
            Self::Overflow => "overflow",
            Self::SequenceGap => "sequence gap",
            Self::BadPacket => "bad packet",
        }
    }

    fn comment(self) -> &'static str {
        match self {
            Self::Overflow => "device buffer overflowed; samples were dropped before this point",
            Self::SequenceGap => "packet sequence gap; samples are missing before this point",
            Self::BadPacket => "an unparseable packet was skipped before this point",
        }
    }
}

/// One discontinuity, at the index of the first sample after it.
#[derive(Debug, Clone, Copy, PartialEq)]
pub(crate) struct Event {
    /// Index into the file of the first sample after the discontinuity.
    pub(crate) sample: u64,
    /// What happened.
    pub(crate) kind: EventKind,
    /// Device time the stream reported alongside the event, in seconds.
    pub(crate) device_time: Option<f64>,
}

/// Everything about a recording that goes into its metadata.
#[derive(Debug, Clone)]
pub(crate) struct Recording {
    pub(crate) format: SampleFormat,
    pub(crate) sample_rate: f64,
    pub(crate) frequency: f64,
    pub(crate) gain: Option<f64>,
    pub(crate) bandwidth: Option<f64>,
    pub(crate) antenna: String,
    pub(crate) master_clock_rate: f64,
    /// Product and serial, e.g. `B205mini 3168B41`.
    pub(crate) hardware: String,
    /// Host wall-clock time the stream was started.
    pub(crate) started: SystemTime,
    /// Device time of the first sample, in seconds.
    pub(crate) first_device_time: Option<f64>,
    pub(crate) events: Vec<Event>,
}

/// The `.sigmf-meta` path that goes with a data file.
///
/// SigMF pairs files by basename, so `capture.sigmf-data` gets
/// `capture.sigmf-meta`. Any other extension is replaced the same way; a
/// file with no extension gets one appended.
pub(crate) fn meta_path_for(data: &Path) -> PathBuf {
    data.with_extension("sigmf-meta")
}

/// The SigMF `core:datatype` for a host sample format.
///
/// The file is written in host byte order, so the suffix follows the build
/// target rather than being fixed.
pub(crate) fn datatype(format: SampleFormat) -> Option<String> {
    let base = match format {
        SampleFormat::Sc16 => "ci16",
        SampleFormat::Fc32 => "cf32",
        SampleFormat::Fc64 => "cf64",
        // One byte per component has no byte order.
        SampleFormat::Sc8 => return Some("ci8".into()),
        _ => return None,
    };
    let endian = if cfg!(target_endian = "little") {
        "le"
    } else {
        "be"
    };
    Some(format!("{base}_{endian}"))
}

/// Builds the metadata document.
pub(crate) fn metadata(rec: &Recording) -> Value {
    let mut global = Map::new();
    global.insert(
        "core:datatype".into(),
        json!(datatype(rec.format).unwrap_or_else(|| rec.format.name().to_string())),
    );
    global.insert("core:sample_rate".into(), json!(rec.sample_rate));
    global.insert("core:version".into(), json!(SIGMF_VERSION));
    global.insert("core:num_channels".into(), json!(1));
    global.insert("core:hw".into(), json!(rec.hardware));
    global.insert(
        "core:recorder".into(),
        json!(concat!("ruhd ", env!("CARGO_PKG_VERSION"))),
    );
    global.insert(
        "core:extensions".into(),
        json!([{
            "name": "ruhd",
            "version": env!("CARGO_PKG_VERSION"),
            "optional": true,
        }]),
    );
    global.insert("ruhd:antenna".into(), json!(rec.antenna));
    global.insert(
        "ruhd:master_clock_rate".into(),
        json!(rec.master_clock_rate),
    );
    if let Some(gain) = rec.gain {
        global.insert("ruhd:gain".into(), json!(gain));
    }
    if let Some(bw) = rec.bandwidth {
        global.insert("ruhd:bandwidth".into(), json!(bw));
    }
    global.insert("ruhd:discontinuities".into(), json!(rec.events.len()));

    let mut capture = Map::new();
    capture.insert("core:sample_start".into(), json!(0));
    capture.insert("core:frequency".into(), json!(rec.frequency));
    capture.insert("core:datetime".into(), json!(iso8601(rec.started)));
    if let Some(t) = rec.first_device_time {
        capture.insert("ruhd:device_time".into(), json!(t));
    }

    let annotations: Vec<Value> = rec
        .events
        .iter()
        .map(|event| {
            let mut a = Map::new();
            a.insert("core:sample_start".into(), json!(event.sample));
            a.insert("core:label".into(), json!(event.kind.label()));
            a.insert("core:comment".into(), json!(event.kind.comment()));
            if let Some(t) = event.device_time {
                a.insert("ruhd:device_time".into(), json!(t));
            }
            Value::Object(a)
        })
        .collect();

    json!({
        "global": global,
        "captures": [capture],
        "annotations": annotations,
    })
}

/// Formats a time as UTC ISO 8601 with microseconds, which is what
/// `core:datetime` asks for.
fn iso8601(t: SystemTime) -> String {
    let since = t.duration_since(UNIX_EPOCH).unwrap_or_default();
    let secs = since.as_secs();
    let days = (secs / 86_400) as i64;
    let rem = secs % 86_400;
    let (y, m, d) = civil_from_days(days);
    format!(
        "{y:04}-{m:02}-{d:02}T{:02}:{:02}:{:02}.{:06}Z",
        rem / 3600,
        rem % 3600 / 60,
        rem % 60,
        since.subsec_micros()
    )
}

/// Days since 1970-01-01 to a proleptic Gregorian date (Howard Hinnant's
/// `civil_from_days`).
fn civil_from_days(z: i64) -> (i64, u32, u32) {
    let z = z + 719_468;
    let era = z.div_euclid(146_097);
    let doe = z.rem_euclid(146_097);
    let yoe = (doe - doe / 1460 + doe / 36_524 - doe / 146_096) / 365;
    let doy = doe - (365 * yoe + yoe / 4 - yoe / 100);
    let mp = (5 * doy + 2) / 153;
    let d = (doy - (153 * mp + 2) / 5 + 1) as u32;
    let m = if mp < 10 { mp + 3 } else { mp - 9 } as u32;
    let y = yoe + era * 400 + i64::from(m <= 2);
    (y, m, d)
}

#[cfg(test)]
mod tests {
    use std::time::Duration;

    use super::*;

    fn recording() -> Recording {
        Recording {
            format: SampleFormat::Sc16,
            sample_rate: 1e6,
            frequency: 915e6,
            gain: Some(40.0),
            bandwidth: None,
            antenna: "RX2".into(),
            master_clock_rate: 32e6,
            hardware: "B205mini 3168B41".into(),
            started: UNIX_EPOCH + Duration::from_micros(1_700_000_000_123_456),
            first_device_time: Some(1.5),
            events: vec![
                Event {
                    sample: 1000,
                    kind: EventKind::Overflow,
                    device_time: Some(1.501),
                },
                Event {
                    sample: 5000,
                    kind: EventKind::SequenceGap,
                    device_time: None,
                },
            ],
        }
    }

    #[test]
    fn dates_are_utc_iso8601() {
        assert_eq!(iso8601(UNIX_EPOCH), "1970-01-01T00:00:00.000000Z");
        let t = UNIX_EPOCH + Duration::from_micros(1_700_000_000_123_456);
        assert_eq!(iso8601(t), "2023-11-14T22:13:20.123456Z");
        // A leap day.
        let t = UNIX_EPOCH + Duration::from_secs(951_782_400);
        assert_eq!(iso8601(t), "2000-02-29T00:00:00.000000Z");
    }

    #[test]
    fn meta_path_pairs_by_basename() {
        assert_eq!(
            meta_path_for(Path::new("a/cap.sigmf-data")),
            Path::new("a/cap.sigmf-meta")
        );
        assert_eq!(
            meta_path_for(Path::new("cap.dat")),
            Path::new("cap.sigmf-meta")
        );
        assert_eq!(meta_path_for(Path::new("cap")), Path::new("cap.sigmf-meta"));
    }

    #[test]
    fn datatypes_follow_host_order() {
        let suffix = if cfg!(target_endian = "little") {
            "le"
        } else {
            "be"
        };
        assert_eq!(
            datatype(SampleFormat::Sc16).unwrap(),
            format!("ci16_{suffix}")
        );
        assert_eq!(
            datatype(SampleFormat::Fc32).unwrap(),
            format!("cf32_{suffix}")
        );
        assert_eq!(
            datatype(SampleFormat::Fc64).unwrap(),
            format!("cf64_{suffix}")
        );
    }

    #[test]
    fn metadata_carries_settings_and_discontinuities() {
        let meta = metadata(&recording());
        let global = &meta["global"];
        assert_eq!(global["core:sample_rate"], 1e6);
        assert_eq!(global["core:version"], SIGMF_VERSION);
        assert_eq!(global["ruhd:gain"], 40.0);
        assert!(global.get("ruhd:bandwidth").is_none());
        assert_eq!(global["ruhd:discontinuities"], 2);
        assert_eq!(global["core:extensions"][0]["name"], "ruhd");

        let capture = &meta["captures"][0];
        assert_eq!(capture["core:sample_start"], 0);
        assert_eq!(capture["core:frequency"], 915e6);
        assert_eq!(capture["core:datetime"], "2023-11-14T22:13:20.123456Z");

        let annotations = meta["annotations"].as_array().unwrap();
        assert_eq!(annotations.len(), 2);
        assert_eq!(annotations[0]["core:sample_start"], 1000);
        assert_eq!(annotations[0]["core:label"], "overflow");
        assert_eq!(annotations[0]["ruhd:device_time"], 1.501);
        assert_eq!(annotations[1]["core:label"], "sequence gap");
        assert!(annotations[1].get("ruhd:device_time").is_none());
    }
}
