//! Public Device API against an attached B2xx. Run with `hw-tests` and
//! `--test-threads 1`; select a board with RUHD_HW_SERIAL.
#![cfg(feature = "hw-tests")]

use std::sync::Arc;
use std::time::Duration;
use uhd_core::num_complex::Complex;
use uhd_core::{
    Device, DeviceArgs, DeviceExt, Direction, Error, Registry, RxError, RxMetadata, SensorData,
    SensorScope, StreamArgs, StreamCmd, TimeSpec, TunePolicy, TuneRequest,
};

fn open_device(fixed_clock: bool) -> Arc<dyn Device> {
    let mut registry = Registry::new();
    registry.register(Arc::new(uhd_b200::B200Factory));
    let mut args = DeviceArgs::new()
        .with("type", "b200")
        .with("auto_flash", "true");
    if fixed_clock {
        args.set("master_clock_rate", "32e6");
    }
    if let Ok(serial) = std::env::var("RUHD_HW_SERIAL") {
        args.set("serial", serial);
    }
    registry.open(&args).expect("open through DeviceFactory")
}

#[test]
fn typed_receive_survives_dropping_the_device_and_releases_its_claim() {
    let device = open_device(true);
    assert_eq!(device.master_clock_rate(0).unwrap(), 32e6);
    assert_eq!(device.set_sample_rate(Direction::Rx, 0, 1e6).unwrap(), 1e6);
    device.tune_to(Direction::Rx, 0, 915e6).unwrap();
    let mut rx = device
        .rx_stream::<Complex<i16>>(&StreamArgs::new())
        .unwrap();
    assert!(device.open_rx_stream(&StreamArgs::new()).is_err());
    assert!(device.set_sample_rate(Direction::Rx, 0, 2e6).is_err());
    assert!(device.set_master_clock_rate(0, 40e6).is_err());
    assert!(device.set_time_now(0, TimeSpec::default()).is_err());
    assert!(device.set_time_next_pps(0, TimeSpec::default()).is_err());
    assert!(device.set_time_source(0, "internal").is_err());
    assert_eq!(device.sample_rate(Direction::Rx, 0).unwrap(), 1e6);
    assert_eq!(device.master_clock_rate(0).unwrap(), 32e6);
    drop(device);
    rx.issue_stream_cmd(&StreamCmd::num_samps(10_000)).unwrap();
    let mut buf = vec![Complex::new(0i16, 0); 1500];
    let mut total = 0;
    let mut next = None;
    loop {
        let mut md = RxMetadata::default();
        let n = rx
            .recv_one(&mut buf, Duration::from_secs(1), &mut md)
            .unwrap();
        assert_eq!(md.error, None);
        assert!(n > 0);
        let ticks = md.time_spec.unwrap().to_ticks(32e6);
        if let Some(want) = next {
            assert_eq!(ticks, want);
        }
        next = Some(ticks + n as i64 * 32);
        total += n;
        if md.end_of_burst {
            break;
        }
        assert!(total < 10_000);
    }
    assert_eq!(total, 10_000);
    drop(rx);
    let device = open_device(true);
    device.set_master_clock_rate(0, 40e6).unwrap();
    device.set_sample_rate(Direction::Rx, 0, 2e6).unwrap();
    let _rx = device
        .rx_stream::<Complex<f32>>(&StreamArgs::new())
        .unwrap();
}

#[test]
fn configuration_reports_actual_values_and_rejects_invalid_requests() {
    let device = open_device(false);
    assert_eq!(
        device.set_sample_rate(Direction::Rx, 0, 20e6).unwrap(),
        20e6
    );
    assert!((device.master_clock_rate(0).unwrap() - 20e6).abs() < 1.0);
    let result = device
        .tune_to(Direction::Rx, 0, TuneRequest::new(98e6).with_lo_offset(2e6))
        .unwrap();
    assert!((result.actual_freq() - 98e6).abs() < 1.0);
    assert!((result.actual_dsp_freq - 2e6).abs() < 1.0);
    assert!((device.center_freq(Direction::Rx, 0).unwrap() - result.actual_freq()).abs() < 1.0);
    let mut request = TuneRequest::new(99e6);
    request.rf_freq_policy = TunePolicy::None;
    let preserved = device.tune_to(Direction::Rx, 0, request).unwrap();
    assert_eq!(preserved.actual_rf_freq, result.actual_rf_freq);
    assert!((preserved.actual_freq() - 99e6).abs() < 1.0);
    device.set_gain(Direction::Rx, 0, None, 40.7).unwrap();
    assert_eq!(device.gain(Direction::Rx, 0, Some("PGA")).unwrap(), 40.0);
    let bw = device.set_bandwidth(Direction::Rx, 0, 5e6).unwrap();
    assert_eq!(device.bandwidth(Direction::Rx, 0).unwrap(), bw);
    device.set_antenna(Direction::Rx, 0, "TX/RX").unwrap();
    assert_eq!(device.antenna(Direction::Rx, 0).unwrap(), "TX/RX");
    let scope = SensorScope::Channel {
        dir: Direction::Rx,
        chan: 0,
    };
    assert_eq!(
        device.sensor(scope, "lo_locked").unwrap().value,
        SensorData::Bool(true)
    );
    assert!(matches!(
        device.sample_rate(Direction::Rx, device.num_channels(Direction::Rx)),
        Err(Error::IndexOutOfBounds { .. })
    ));
    assert!(matches!(
        device.time_now(1),
        Err(Error::IndexOutOfBounds { .. })
    ));
    assert!(device.set_sample_rate(Direction::Rx, 0, f64::NAN).is_err());
    assert!(device.set_master_clock_rate(0, 0.0).is_err());
    assert!(device.set_antenna(Direction::Rx, 0, "invalid").is_err());
    assert!(
        device
            .set_gain(Direction::Rx, 0, Some("invalid"), 1.0)
            .is_err()
    );
    device.set_sample_rate(Direction::Tx, 0, 2e6).unwrap();
    let tx_tune = device
        .tune_to(
            Direction::Tx,
            0,
            TuneRequest::new(915e6).with_lo_offset(1e6),
        )
        .unwrap();
    assert!((tx_tune.actual_freq() - 915e6).abs() < 1.0);
    assert!((device.center_freq(Direction::Tx, 0).unwrap() - 915e6).abs() < 1.0);
    device.set_gain(Direction::Tx, 0, None, 10.1).unwrap();
    assert_eq!(device.gain(Direction::Tx, 0, None).unwrap(), 10.25);
    assert_eq!(device.antennas(Direction::Tx, 0).unwrap(), ["TX/RX"]);
    assert!(device.set_antenna(Direction::Tx, 0, "RX2").is_err());
    assert!(device.set_agc(Direction::Tx, 0, true).is_err());
    // A rejected stream format must leave the device available.
    let mut args = StreamArgs::new();
    args.wire_format = uhd_core::SampleFormat::Sc8;
    assert!(device.open_rx_stream(&args).is_err());
    let rx = device.open_rx_stream(&StreamArgs::new()).unwrap();
    device.tune_to(Direction::Rx, 0, 100e6).unwrap();
    device.set_antenna(Direction::Rx, 0, "RX2").unwrap();
    device.set_gain(Direction::Rx, 0, None, 20.0).unwrap();
    drop(rx);
    device.set_time_now(0, TimeSpec::from_secs(10.0)).unwrap();
    assert!((device.time_now(0).unwrap().as_secs_f64() - 10.0).abs() < 0.1);
    device.set_master_clock_rate(0, 32e6).unwrap();
    assert_eq!(device.bandwidth(Direction::Rx, 0).unwrap(), bw);
    let now = device.time_now(0).unwrap().as_secs_f64();
    assert!(
        (10.0..11.0).contains(&now),
        "time after clock change: {now}"
    );
}

#[test]
fn timed_and_late_commands_work_through_the_device_api() {
    let device = open_device(true);
    device.set_sample_rate(Direction::Rx, 0, 1e6).unwrap();
    let mut rx = device
        .rx_stream::<Complex<f32>>(&StreamArgs::new())
        .unwrap();
    let start = TimeSpec::from_secs(device.time_now(0).unwrap().as_secs_f64() + 0.25);
    rx.issue_stream_cmd(&StreamCmd::num_samps(1000).at(start))
        .unwrap();
    let mut buf = vec![Complex::new(0f32, 0.0); 1000];
    let mut md = RxMetadata::default();
    assert_eq!(
        rx.recv_one(&mut buf, Duration::from_secs(2), &mut md)
            .unwrap(),
        1000
    );
    assert_eq!(md.error, None);
    assert!(md.end_of_burst);
    let late = md.time_spec.unwrap().to_ticks(32e6) - start.to_ticks(32e6);
    assert!((0..=128).contains(&late), "timed start {late} ticks late");
    rx.issue_stream_cmd(&StreamCmd::num_samps(1000).at(TimeSpec::default()))
        .unwrap();
    assert_eq!(
        rx.recv_one(&mut buf, Duration::from_secs(1), &mut md)
            .unwrap(),
        0
    );
    assert_eq!(md.error, Some(RxError::LateCommand));
}
