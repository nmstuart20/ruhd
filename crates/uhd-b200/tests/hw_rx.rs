//! Receiving samples from a real B2xx.
//!
//! Gated behind the `hw-tests` feature and run like the other hardware tests:
//!
//! ```sh
//! cargo run --bin ruhd -- init
//! cargo test -p uhd-b200 --features hw-tests --test hw_rx -- --test-threads 1
//! ```
//!
//! These check the parts of the streamer that a plain recording does not
//! exercise: finite bursts and their end-of-burst flag, timed starts, the
//! framer's late-command message, and that consecutive packets' timestamps
//! account for every sample in between. Each test re-initialises the codec.
#![cfg(feature = "hw-tests")]

use std::time::Duration;

use uhd_b200::{B200RxStreamer, OpenOptions, RxOptions, UhdImagesCatalog, open_probed};
use uhd_core::metadata::{RxError, RxMetadata};
use uhd_core::stream::{RawRxStreamer, StreamArgs, StreamCmd};
use uhd_core::{SampleFormat, TimeSpec};
use uhd_transport::usb::{self, UsbFilter};

/// Serial of the device to use, when more than one is attached.
const SERIAL_ENV: &str = "RUHD_HW_SERIAL";
const MASTER_CLOCK: f64 = 32e6;
const RATE: f64 = 1e6;

fn session() -> uhd_b200::B200Session {
    let mut filter = UsbFilter::new();
    filter.serial = std::env::var(SERIAL_ENV).ok();
    let mut found = usb::probe_filtered(&filter).expect("usb enumeration");
    assert_eq!(
        found.len(),
        1,
        "hw tests need exactly one B2xx attached; found {} (set {SERIAL_ENV} to choose)",
        found.len()
    );
    let catalog = UhdImagesCatalog::builtin().expect("the built-in manifest");
    let session = open_probed(
        &found.remove(0),
        &OpenOptions::new().with_auto_flash(true),
        &catalog,
        |_, _| {},
    )
    .expect("open with auto-flash");
    session.init_radio(MASTER_CLOCK).expect("radio init");
    session
        .codec()
        .tune(ad936x::Direction::Rx, 915e6)
        .expect("tune");
    session
}

fn stream(session: &uhd_b200::B200Session) -> B200RxStreamer {
    let mut args = StreamArgs::new();
    args.host_format = SampleFormat::Sc16;
    session
        .rx_stream(&RxOptions::new(RATE), &args)
        .expect("open rx stream")
}

fn recv(rx: &mut B200RxStreamer, buf: &mut [u8], timeout: Duration) -> (usize, RxMetadata) {
    let mut md = RxMetadata::default();
    let n = rx.recv_raw(&mut [buf], timeout, &mut md).expect("recv_raw");
    (n, md)
}

#[test]
fn a_finite_burst_delivers_exactly_n_samples_and_ends_the_burst() {
    let session = session();
    let mut rx = stream(&session);
    assert_eq!(rx.sample_rate(), RATE);

    const N: u64 = 10_000;
    rx.issue_stream_cmd(&StreamCmd::num_samps(N)).unwrap();
    let mut buf = vec![0u8; 4096 * 4];
    let mut total = 0u64;
    loop {
        let (n, md) = recv(&mut rx, &mut buf, Duration::from_secs(1));
        assert_eq!(md.error, None, "after {total} samples: {md:?}");
        assert_eq!(md.loss, None, "unexpected loss after {total} samples");
        total += n as u64;
        if md.end_of_burst {
            break;
        }
        assert!(total < N, "no end of burst after {total} samples");
    }
    assert_eq!(total, N);

    // Nothing follows the burst.
    let (n, md) = recv(&mut rx, &mut buf, Duration::from_millis(200));
    assert_eq!((n, md.error), (0, Some(RxError::Timeout)));
}

#[test]
fn continuous_timestamps_account_for_every_sample() {
    let session = session();
    let mut rx = stream(&session);
    let ticks_per_sample = (rx.tick_rate() / rx.sample_rate()).round() as i64;
    let tick_rate = rx.tick_rate();

    // An odd buffer size, so fragments straddle packet boundaries.
    let mut buf = vec![0u8; 1500 * 4];
    rx.issue_stream_cmd(&StreamCmd::start_continuous()).unwrap();
    let mut next: Option<i64> = None;
    let mut total = 0usize;
    while total < 200_000 {
        let (n, md) = recv(&mut rx, &mut buf, Duration::from_secs(1));
        assert_eq!(md.error, None, "after {total} samples: {md:?}");
        assert_eq!(md.loss, None, "unexpected loss after {total} samples");
        let ticks = md
            .time_spec
            .expect("b2xx data carries a timestamp")
            .to_ticks(tick_rate);
        if let Some(want) = next {
            assert_eq!(ticks, want, "after {total} samples");
        }
        next = Some(ticks + n as i64 * ticks_per_sample);
        total += n;
    }
    rx.issue_stream_cmd(&StreamCmd::stop()).unwrap();
}

#[test]
fn a_timed_start_begins_on_the_requested_tick() {
    let session = session();
    let mut rx = stream(&session);
    let time = session.time_core(0).unwrap();
    time.set_tick_rate(rx.tick_rate()).unwrap();

    let now = time.get_time_now().unwrap();
    let start = TimeSpec::from_ticks(
        now.to_ticks(rx.tick_rate()) + (0.25 * rx.tick_rate()) as i64,
        rx.tick_rate(),
    );
    rx.issue_stream_cmd(&StreamCmd::num_samps(1000).at(start))
        .unwrap();

    let mut buf = vec![0u8; 1000 * 4];
    let (n, md) = recv(&mut rx, &mut buf, Duration::from_secs(2));
    assert_eq!(md.error, None, "{md:?}");
    assert!(n > 0);

    // The first sample is stamped a little after the commanded tick: the
    // command gates the DSP chain's input, and the timestamp is taken at the
    // framer, after the decimation filters. Stock UHD 4.10 measures the same
    // thing on a B205mini — 67 ticks at a 32 MHz clock and decimation 32 — so
    // the bound here is a few output samples, not zero.
    let late = md.time_spec.unwrap().to_ticks(rx.tick_rate()) - start.to_ticks(rx.tick_rate());
    let ticks_per_sample = (rx.tick_rate() / rx.sample_rate()).round() as i64;
    assert!(
        (0..=4 * ticks_per_sample).contains(&late),
        "first sample {late} ticks after the commanded start"
    );
}

#[test]
fn a_command_timed_in_the_past_is_reported_late() {
    let session = session();
    let mut rx = stream(&session);
    // init_radio set the clock to zero well over a second ago.
    rx.issue_stream_cmd(&StreamCmd::num_samps(1000).at(TimeSpec::from_secs(0.001)))
        .unwrap();
    let mut buf = vec![0u8; 1000 * 4];
    let (n, md) = recv(&mut rx, &mut buf, Duration::from_secs(1));
    assert_eq!((n, md.error), (0, Some(RxError::LateCommand)), "{md:?}");
}
