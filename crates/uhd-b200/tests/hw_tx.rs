//! Explicit TX bench tests. Requires a terminated TX/RX port or attenuated
//! receiver connection. Run with `--features hw-tests --test hw_tx -- --ignored
//! --test-threads 1 --nocapture`. Select the board with RUHD_HW_SERIAL.
#![cfg(feature = "hw-tests")]

use std::time::{Duration, Instant};
use uhd_core::num_complex::Complex;
use uhd_core::{
    AsyncEvent, DeviceArgs, DeviceExt, Direction, StreamArgs, TimeSpec, TxMetadata, registry,
};

#[test]
#[ignore = "transmits RF: requires a terminated TX/RX port or attenuated test setup"]
fn timed_burst_ack_late_error_and_stream_ownership() {
    uhd_b200::register();
    let mut args = DeviceArgs::new()
        .with("type", "b200")
        .with("master_clock_rate", "32e6");
    if let Ok(serial) = std::env::var("RUHD_HW_SERIAL") {
        args.set("serial", serial);
    }
    let device = registry::open(&args).unwrap();
    device.set_sample_rate(Direction::Tx, 0, 1e6).unwrap();
    device.tune_to(Direction::Tx, 0, 915e6).unwrap();
    device.set_gain(Direction::Tx, 0, None, 0.0).unwrap();
    let mut tx = device
        .tx_stream::<Complex<f32>>(&StreamArgs::new().with_arg("spp", "127"))
        .unwrap();
    assert!(device.open_tx_stream(&StreamArgs::new()).is_err());
    assert!(device.open_rx_stream(&StreamArgs::new()).is_err());
    assert!(device.set_sample_rate(Direction::Tx, 0, 2e6).is_err());
    assert!(device.set_master_clock_rate(0, 40e6).is_err());
    assert!(device.set_time_now(0, TimeSpec::ZERO).is_err());
    let start = device.time_now(0).unwrap() + TimeSpec::from_secs(0.25);
    let samples: Vec<_> = (0..1001)
        .map(|i| {
            let phase = std::f32::consts::TAU * i as f32 / 16.0;
            Complex::new(0.1 * phase.cos(), 0.1 * phase.sin())
        })
        .collect();
    let mut sent = 0;
    while sent < samples.len() {
        let n = tx
            .send_one(
                &samples[sent..],
                Duration::from_secs(1),
                &TxMetadata {
                    time_spec: Some(start + TimeSpec::from_ticks(sent as i64, 1e6)),
                    start_of_burst: sent == 0,
                    end_of_burst: true,
                },
            )
            .unwrap();
        assert!(n > 0);
        sent += n;
    }
    // Read a register while events can arrive; its reply must survive the demux.
    let now = device.time_now(0).unwrap();
    assert!(now < start);
    let event = tx
        .recv_async_msg(Duration::from_secs(2))
        .unwrap()
        .expect("burst ACK");
    assert_eq!(event.channel, 0);
    assert_eq!(event.event, AsyncEvent::BurstAck);
    assert!(device.time_now(0).unwrap() >= start);
    let before = Instant::now();
    assert!(
        tx.recv_async_msg(Duration::from_millis(20))
            .unwrap()
            .is_none()
    );
    assert!(before.elapsed() < Duration::from_secs(1));
    tx.send_one(
        &samples[..1],
        Duration::from_secs(1),
        &TxMetadata::one_shot().at(TimeSpec::ZERO),
    )
    .unwrap();
    let event = tx
        .recv_async_msg(Duration::from_secs(1))
        .unwrap()
        .expect("late TX event");
    assert_eq!(event.event, AsyncEvent::TimeError);
    drop(tx);
    device.set_sample_rate(Direction::Tx, 0, 2e6).unwrap();
    let mut tx = device
        .tx_stream::<Complex<i16>>(&StreamArgs::new())
        .unwrap();
    drop(device);
    tx.send_one(
        &[Complex::new(0i16, 0); 10],
        Duration::from_secs(1),
        &TxMetadata::one_shot(),
    )
    .unwrap();
    assert_eq!(
        tx.recv_async_msg(Duration::from_secs(1))
            .unwrap()
            .unwrap()
            .event,
        AsyncEvent::BurstAck
    );
}
