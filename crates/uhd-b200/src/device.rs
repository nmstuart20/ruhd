//! Public device API over a B2xx session.
//!
//! Single-channel receive and transmit configuration and streaming. One RX or
//! TX streamer may exist at a time. Each direction shares its LO and analog
//! bandwidth across B210 channels; gains and DSP settings are per channel.

use std::sync::{Arc, Mutex, MutexGuard};
use std::time::Duration;

use ad936x::{Chain, Direction as CodecDirection};
use uhd_core::{
    AsyncMetadata, Device, DeviceArgs, DeviceExt, DeviceInfo, Direction, Error, MetaRange, Range,
    RawRxStreamer, RawTxStreamer, Result, RxMetadata, SampleFormat, SensorData, SensorScope,
    SensorValue, StreamArgs, StreamCmd, TimeSpec, TunePolicy, TuneRequest, TuneResult, TxMetadata,
};

use crate::time_core::{pulse_sync, set_time_source};
use crate::{
    B200RxStreamer, B200Session, B200TxStreamer, RefSel, RxAntenna, RxDsp, RxOptions, TimeCore,
    TimeSource, TxDsp, TxOptions,
};

const MIN_RF: f64 = 70e6;
const MAX_RF: f64 = 6e9;
const DEFAULT_RATE: f64 = 1e6;

/// An initialized B2xx exposed through [`uhd_core::Device`].
///
/// Open through [`uhd_core::registry::open`] after [`crate::register`], or use
/// [`Self::from_session`] when supplying your own compatibility policy.
/// `master_clock_rate` fixes the clock; otherwise `set_sample_rate` selects it
/// automatically. `auto_flash=true` allows a cached compatible FPGA reload.
/// Firmware must already be running (use `ruhd init` first).
///
/// Configuration calls are serialized. Rate, clock, time and time-source
/// changes are refused while a streamer exists, even after a stop command;
/// drop the streamer to release that restriction. Tuning, manual gain,
/// bandwidth, RX AGC and antenna changes are permitted during streaming and can
/// introduce discontinuities. Streamers retain the device until they close.
#[derive(Debug)]
pub struct B200Device {
    shared: Arc<Shared>,
    info: DeviceInfo,
    channels: usize,
}

#[derive(Debug)]
struct Shared {
    session: B200Session,
    state: Mutex<State>,
}

#[derive(Debug)]
struct State {
    tick_rate: f64,
    fixed_clock: bool,
    requested_rates: Vec<f64>,
    dsp: Vec<RxDsp>,
    tx_dsp: Vec<TxDsp>,
    tx_requested_rates: Vec<f64>,
    tx_gains: Vec<f64>,
    tx_bandwidth: Option<f64>,
    time: Vec<TimeCore>,
    antennas: Vec<RxAntenna>,
    gains: Vec<f64>,
    agc: Vec<bool>,
    bandwidth: Option<f64>,
    stream_channel: Option<(Direction, usize)>,
    time_source: TimeSource,
}

impl State {
    fn require_idle(&self) -> Result<()> {
        if self.stream_channel.is_some() {
            return Err(Error::device(
                "drop the existing RX or TX streamer before opening another stream or changing timing configuration",
            ));
        }
        Ok(())
    }
}

pub(crate) fn validate_open_args(args: &DeviceArgs) -> Result<()> {
    args.get_parsed::<bool>("auto_flash")?;
    if let Some(rate) = args.get_parsed("master_clock_rate")? {
        validate_clock(rate)?;
    }
    Ok(())
}

fn validate_clock(rate: f64) -> Result<()> {
    if !rate.is_finite() || !(ad936x::MIN_CLOCK_RATE..=ad936x::MAX_CLOCK_RATE).contains(&rate) {
        return Err(Error::OutOfRange {
            what: "master clock rate".into(),
            value: rate,
            start: ad936x::MIN_CLOCK_RATE,
            stop: ad936x::MAX_CLOCK_RATE,
        });
    }
    Ok(())
}

impl B200Device {
    /// Initializes an owned session and its receive/transmit controls.
    ///
    /// This resets the codec and discards its previous configuration. Supported
    /// options are `master_clock_rate` and `auto_flash` (the latter is applied
    /// by the factory before creating the session). Other device arguments are
    /// discovery predicates, not radio settings.
    pub fn from_session(session: B200Session, args: &DeviceArgs) -> Result<Self> {
        validate_open_args(args)?;
        let clock = args.get_parsed("master_clock_rate")?;
        let tick_rate = session.init_radio(clock.unwrap_or(32e6))?;
        let channels = session.num_radio_chains()?;
        let report = session.report();
        let info = crate::factory::device_info(
            session.endpoint(),
            Some(session.product()),
            session.endpoint().serial.as_deref(),
            report.eeprom.serial.as_deref(),
            report.eeprom.name.as_deref(),
            report.eeprom.revision,
        );
        let mut dsp = Vec::new();
        let mut time = Vec::new();
        let mut tx_dsp = Vec::new();
        for chan in 0..channels {
            let mut core = RxDsp::new(Box::new(session.ctrl().radio(chan)?));
            core.set_tick_rate(tick_rate)?;
            core.set_mux(0)?;
            core.set_host_rate(DEFAULT_RATE)?;
            core.setup_sc16()?;
            dsp.push(core);
            let mut tx = TxDsp::new(Box::new(session.ctrl().radio(chan)?));
            tx.set_tick_rate(tick_rate)?;
            tx.set_host_rate(DEFAULT_RATE)?;
            tx_dsp.push(tx);
            session.codec().set_gain(
                CodecDirection::Tx,
                chain_for(chan, session.misc().swap_atr()),
                0.0,
            )?;
            let timer = session.time_core(chan)?;
            timer.set_tick_rate(tick_rate)?;
            time.push(timer);
            session.codec().set_gain(
                CodecDirection::Rx,
                chain_for(chan, session.misc().swap_atr()),
                0.0,
            )?;
        }
        let time_source = TimeSource::Internal;
        set_time_source(&session.ctrl().local(), session.compat(), time_source)?;
        for timer in &time {
            timer.set_time_sync(TimeSpec::default())?;
        }
        pulse_sync(&session.ctrl().local(), session.compat(), time_source)?;
        if session.shim().has_bandsel() {
            session.misc().set_bandsel(
                Some(session.codec().freq(CodecDirection::Rx)),
                Some(session.codec().freq(CodecDirection::Tx)),
            )?;
        }
        Ok(Self {
            info,
            channels,
            shared: Arc::new(Shared {
                session,
                state: Mutex::new(State {
                    tick_rate,
                    fixed_clock: clock.is_some(),
                    requested_rates: vec![DEFAULT_RATE; channels],
                    dsp,
                    tx_dsp,
                    tx_requested_rates: vec![DEFAULT_RATE; channels],
                    tx_gains: vec![0.0; channels],
                    tx_bandwidth: None,
                    time,
                    antennas: vec![RxAntenna::Rx2; channels],
                    gains: vec![0.0; channels],
                    agc: vec![false; channels],
                    bandwidth: None,
                    stream_channel: None,
                    time_source,
                }),
            }),
        })
    }

    fn state(&self) -> Result<MutexGuard<'_, State>> {
        self.shared
            .state
            .lock()
            .map_err(|_| Error::internal("B2xx configuration lock poisoned; reopen the device"))
    }

    fn mboard(&self, index: usize) -> Result<()> {
        if index != 0 {
            return Err(Error::IndexOutOfBounds {
                what: "motherboard",
                index,
                count: 1,
            });
        }
        Ok(())
    }

    fn chain(&self, chan: usize) -> Chain {
        chain_for(chan, self.shared.session.misc().swap_atr())
    }

    fn change_clock(&self, state: &mut State, rate: f64) -> Result<f64> {
        validate_clock(rate)?;
        state.require_idle()?;
        if (rate - state.tick_rate).abs() < 1.0 {
            return Ok(state.tick_rate);
        }
        // Keep seconds meaningful when the tick frequency changes. Calibration
        // pauses are excluded; every radio resumes at the same device time.
        let now = state.time[0].get_time_now()?;
        state.tick_rate = self.shared.session.set_master_clock_rate(rate)?;
        // The codec's rate calibration chooses new default analog filters.
        // Preserve the bandwidth selected through the public device API.
        if let Some(bandwidth) = state.bandwidth {
            self.shared
                .session
                .codec()
                .set_bandwidth(CodecDirection::Rx, bandwidth)?;
        }
        if let Some(bandwidth) = state.tx_bandwidth {
            self.shared
                .session
                .codec()
                .set_bandwidth(CodecDirection::Tx, bandwidth)?;
        }
        for (chan, dsp) in state.tx_dsp.iter_mut().enumerate() {
            dsp.set_tick_rate(state.tick_rate)?;
            dsp.set_host_rate(state.tx_requested_rates[chan])?;
        }
        for (chan, dsp) in state.dsp.iter_mut().enumerate() {
            dsp.set_tick_rate(state.tick_rate)?;
            dsp.set_host_rate(state.requested_rates[chan])?;
            dsp.setup_sc16()?;
        }
        for timer in &state.time {
            timer.set_tick_rate(state.tick_rate)?;
            timer.set_time_sync(now)?;
        }
        pulse_sync(
            &self.shared.session.ctrl().local(),
            self.shared.session.compat(),
            state.time_source,
        )?;
        Ok(state.tick_rate)
    }
}

fn codec_direction(dir: Direction) -> CodecDirection {
    match dir {
        Direction::Rx => CodecDirection::Rx,
        Direction::Tx => CodecDirection::Tx,
    }
}

fn chain_for(chan: usize, swap: bool) -> Chain {
    if (chan == 0) != swap {
        Chain::One
    } else {
        Chain::Two
    }
}

fn gain_name(name: Option<&str>) -> Result<()> {
    if name.is_some_and(|n| !n.is_empty() && n != "PGA") {
        return Err(Error::KeyNotFound(name.unwrap().to_owned()));
    }
    Ok(())
}

// Validate the entire request before programming either stage.
fn tune_plan(request: &TuneRequest, current_rf: f64, half_clock: f64) -> Result<(f64, f64)> {
    if !request.target_freq.is_finite()
        || (request.rf_freq_policy == TunePolicy::Manual && !request.rf_freq.is_finite())
        || (request.dsp_freq_policy == TunePolicy::Manual && !request.dsp_freq.is_finite())
    {
        return Err(Error::invalid_arg("tune frequencies must be finite"));
    }
    if !request.args.is_empty() {
        return Err(Error::not_supported(format!(
            "B2xx tune arguments: {}",
            request.args
        )));
    }
    let target = request
        .target_freq
        .clamp(MIN_RF - half_clock, MAX_RF + half_clock);
    let rf = match request.rf_freq_policy {
        TunePolicy::None => current_rf,
        TunePolicy::Auto => target.clamp(MIN_RF, MAX_RF),
        TunePolicy::Manual => request.rf_freq.clamp(MIN_RF, MAX_RF),
    };
    Ok((target, rf))
}

impl Device for B200Device {
    fn info(&self) -> DeviceInfo {
        self.info.clone()
    }
    fn num_channels(&self, _dir: Direction) -> usize {
        self.channels
    }

    fn set_sample_rate(&self, dir: Direction, chan: usize, rate: f64) -> Result<f64> {
        self.check_channel(dir, chan)?;
        if !rate.is_finite() || rate <= 0.0 {
            return Err(Error::invalid_arg(
                "sample rate must be positive and finite",
            ));
        }
        let mut state = self.state()?;
        state.require_idle()?;
        let previous_rate = if dir == Direction::Rx {
            state.requested_rates[chan]
        } else {
            state.tx_requested_rates[chan]
        };
        if dir == Direction::Rx {
            state.requested_rates[chan] = rate;
        } else {
            state.tx_requested_rates[chan] = rate;
        }
        // Reapply this channel's new rate during the clock change, avoiding
        // an intermediate decimation (and warnings) for its previous rate.
        let result = (|| {
            if !state.fixed_clock {
                self.change_clock(&mut state, crate::auto_master_clock_rate(rate, 1)?)?;
            }
            if dir == Direction::Rx {
                state.dsp[chan].set_host_rate(rate)
            } else {
                state.tx_dsp[chan].set_host_rate(rate)
            }
        })();
        if result.is_err() {
            if dir == Direction::Rx {
                state.requested_rates[chan] = previous_rate;
            } else {
                state.tx_requested_rates[chan] = previous_rate;
            }
        }
        result
    }
    fn sample_rate(&self, dir: Direction, chan: usize) -> Result<f64> {
        self.check_channel(dir, chan)?;
        let state = self.state()?;
        Ok(if dir == Direction::Rx {
            state.dsp[chan].host_rate()
        } else {
            state.tx_dsp[chan].host_rate()
        })
    }
    fn sample_rates(&self, dir: Direction, chan: usize) -> Result<MetaRange> {
        self.check_channel(dir, chan)?;
        let state = self.state()?;
        // Advertise the discrete rates at the current master clock.
        MetaRange::new(
            crate::rx_dsp::decimations(state.tick_rate)
                .into_iter()
                .map(|d| Range::constant(state.tick_rate / f64::from(d))),
        )
    }
    fn tune(&self, dir: Direction, chan: usize, request: &TuneRequest) -> Result<TuneResult> {
        self.check_channel(dir, chan)?;
        let mut state = self.state()?;
        let codec = self.shared.session.codec();
        let (target, rf) = tune_plan(
            request,
            codec.freq(codec_direction(dir)),
            state.tick_rate.min(codec.bandwidth(codec_direction(dir))) / 2.0,
        )?;
        let actual_rf = if request.rf_freq_policy == TunePolicy::None {
            rf
        } else {
            codec.tune(codec_direction(dir), rf)?
        };
        if self.shared.session.shim().has_bandsel() {
            self.shared.session.misc().set_bandsel(
                (dir == Direction::Rx).then_some(actual_rf),
                (dir == Direction::Tx).then_some(actual_rf),
            )?;
        }
        let dsp = match request.dsp_freq_policy {
            TunePolicy::None => {
                if dir == Direction::Rx {
                    state.dsp[chan].freq()
                } else {
                    -state.tx_dsp[chan].freq()
                }
            }
            TunePolicy::Auto => actual_rf - target,
            TunePolicy::Manual => request.dsp_freq,
        }
        .clamp(-state.tick_rate / 2.0, state.tick_rate / 2.0);
        let actual_dsp = if request.dsp_freq_policy == TunePolicy::None {
            dsp
        } else if dir == Direction::Rx {
            state.dsp[chan].set_freq(dsp)?
        } else {
            // The core API reports RF minus DSP in either direction; the
            // physical TX upconverter adds its frequency to the RF LO.
            -state.tx_dsp[chan].set_freq(-dsp)?
        };
        Ok(TuneResult {
            clipped_rf_freq: target,
            target_rf_freq: rf,
            actual_rf_freq: actual_rf,
            target_dsp_freq: dsp,
            actual_dsp_freq: actual_dsp,
        })
    }
    fn center_freq(&self, dir: Direction, chan: usize) -> Result<f64> {
        self.check_channel(dir, chan)?;
        let state = self.state()?;
        Ok(self.shared.session.codec().freq(codec_direction(dir))
            - (if dir == Direction::Rx {
                state.dsp[chan].freq()
            } else {
                -state.tx_dsp[chan].freq()
            }))
    }
    fn freq_range(&self, dir: Direction, chan: usize) -> Result<MetaRange> {
        self.check_channel(dir, chan)?;
        let state = self.state()?;
        let half = state
            .tick_rate
            .min(self.shared.session.codec().bandwidth(codec_direction(dir)))
            / 2.0;
        Ok(Range::continuous(MIN_RF - half, MAX_RF + half).into())
    }
    fn fe_freq_range(&self, dir: Direction, chan: usize) -> Result<MetaRange> {
        self.check_channel(dir, chan)?;
        Ok(Range::continuous(MIN_RF, MAX_RF).into())
    }
    fn gain_names(&self, dir: Direction, chan: usize) -> Result<Vec<String>> {
        self.check_channel(dir, chan)?;
        Ok(vec!["PGA".into()])
    }
    fn set_gain(&self, dir: Direction, chan: usize, name: Option<&str>, gain: f64) -> Result<()> {
        self.check_channel(dir, chan)?;
        gain_name(name)?;
        let mut state = self.state()?;
        let actual = self.shared.session.codec().set_gain(
            codec_direction(dir),
            self.chain(chan),
            if dir == Direction::Tx && gain.is_finite() {
                gain.clamp(0.0, ad936x::MAX_GAIN)
            } else {
                gain
            },
        )?;
        if dir == Direction::Rx {
            state.gains[chan] = actual;
        } else {
            state.tx_gains[chan] = actual;
        }
        Ok(())
    }
    fn gain(&self, dir: Direction, chan: usize, name: Option<&str>) -> Result<f64> {
        self.check_channel(dir, chan)?;
        gain_name(name)?;
        let state = self.state()?;
        if dir == Direction::Rx && state.agc[chan] {
            return Err(Error::not_supported(
                "reading instantaneous gain during AGC",
            ));
        }
        Ok(if dir == Direction::Rx {
            state.gains[chan]
        } else {
            state.tx_gains[chan]
        })
    }
    fn gain_range(&self, dir: Direction, chan: usize, name: Option<&str>) -> Result<Range> {
        self.check_channel(dir, chan)?;
        gain_name(name)?;
        Ok(if dir == Direction::Rx {
            Range::new(0.0, 76.0, 1.0)
        } else {
            Range::new(0.0, ad936x::MAX_GAIN, 0.25)
        })
    }
    fn set_agc(&self, dir: Direction, chan: usize, enable: bool) -> Result<()> {
        self.check_channel(dir, chan)?;
        if dir == Direction::Tx {
            return Err(Error::not_supported("TX AGC"));
        }
        let mut state = self.state()?;
        self.shared
            .session
            .codec()
            .set_agc(self.chain(chan), enable)?;
        state.agc[chan] = enable;
        Ok(())
    }
    fn set_bandwidth(&self, dir: Direction, chan: usize, bandwidth: f64) -> Result<f64> {
        self.check_channel(dir, chan)?;
        let mut state = self.state()?;
        let actual = self
            .shared
            .session
            .codec()
            .set_bandwidth(codec_direction(dir), bandwidth)?;
        if dir == Direction::Rx {
            state.bandwidth = Some(actual);
        } else {
            state.tx_bandwidth = Some(actual);
        }
        Ok(actual)
    }
    fn bandwidth(&self, dir: Direction, chan: usize) -> Result<f64> {
        self.check_channel(dir, chan)?;
        let _state = self.state()?;
        Ok(self.shared.session.codec().bandwidth(codec_direction(dir)))
    }
    fn bandwidth_range(&self, dir: Direction, chan: usize) -> Result<MetaRange> {
        self.check_channel(dir, chan)?;
        Ok(Range::continuous(ad936x::MIN_BW, ad936x::MAX_BW).into())
    }
    fn antennas(&self, dir: Direction, chan: usize) -> Result<Vec<String>> {
        self.check_channel(dir, chan)?;
        Ok(if dir == Direction::Rx {
            vec!["RX2".into(), "TX/RX".into()]
        } else {
            vec!["TX/RX".into()]
        })
    }
    fn set_antenna(&self, dir: Direction, chan: usize, antenna: &str) -> Result<()> {
        self.check_channel(dir, chan)?;
        let antenna = RxAntenna::parse(antenna)?;
        if dir == Direction::Tx {
            return if antenna == RxAntenna::TxRx {
                Ok(())
            } else {
                Err(Error::invalid_arg("TX antenna must be TX/RX"))
            };
        }
        let mut state = self.state()?;
        let session = &self.shared.session;
        session.atr(chan)?.set_b2xx_states(
            crate::rx_stream::side_for(chan, session.misc().swap_atr()),
            state.stream_channel == Some((Direction::Rx, chan)),
            state.stream_channel == Some((Direction::Tx, chan)),
            antenna == RxAntenna::Rx2,
        )?;
        state.antennas[chan] = antenna;
        Ok(())
    }
    fn antenna(&self, dir: Direction, chan: usize) -> Result<String> {
        self.check_channel(dir, chan)?;
        Ok(if dir == Direction::Rx {
            self.state()?.antennas[chan].name().into()
        } else {
            "TX/RX".into()
        })
    }
    fn clock_sources(&self, mboard: usize) -> Result<Vec<String>> {
        self.mboard(mboard)?;
        // Full-size boards also need ADF4001 programming for external clocks.
        Ok(vec!["internal".into()])
    }
    fn clock_source(&self, mboard: usize) -> Result<String> {
        self.mboard(mboard)?;
        Ok("internal".into())
    }
    fn set_clock_source(&self, mboard: usize, source: &str) -> Result<()> {
        self.mboard(mboard)?;
        if source != "internal" {
            return Err(Error::not_supported(
                "external B2xx reference clock configuration",
            ));
        }
        let state = self.state()?;
        state.require_idle()?;
        self.shared.session.misc().set_ref_sel(RefSel::Internal)
    }
    fn time_sources(&self, mboard: usize) -> Result<Vec<String>> {
        self.mboard(mboard)?;
        Ok(self
            .shared
            .session
            .shim()
            .time_sources()
            .iter()
            .map(|s| (*s).into())
            .collect())
    }
    fn time_source(&self, mboard: usize) -> Result<String> {
        self.mboard(mboard)?;
        Ok(self.state()?.time_source.name().into())
    }
    fn set_time_source(&self, mboard: usize, source: &str) -> Result<()> {
        self.mboard(mboard)?;
        let session = &self.shared.session;
        let source = TimeSource::parse(source, session.shim())?;
        let mut state = self.state()?;
        state.require_idle()?;
        set_time_source(&session.ctrl().local(), session.compat(), source)?;
        state.time_source = source;
        Ok(())
    }
    fn set_master_clock_rate(&self, mboard: usize, rate: f64) -> Result<f64> {
        self.mboard(mboard)?;
        let mut state = self.state()?;
        let actual = self.change_clock(&mut state, rate)?;
        state.fixed_clock = true;
        Ok(actual)
    }
    fn master_clock_rate(&self, mboard: usize) -> Result<f64> {
        self.mboard(mboard)?;
        Ok(self.state()?.tick_rate)
    }
    fn time_now(&self, mboard: usize) -> Result<TimeSpec> {
        self.mboard(mboard)?;
        self.state()?.time[0].get_time_now()
    }
    fn set_time_now(&self, mboard: usize, time: TimeSpec) -> Result<()> {
        self.mboard(mboard)?;
        let state = self.state()?;
        state.require_idle()?;
        for timer in &state.time {
            timer.set_time_sync(time)?;
        }
        pulse_sync(
            &self.shared.session.ctrl().local(),
            self.shared.session.compat(),
            state.time_source,
        )
    }
    fn set_time_next_pps(&self, mboard: usize, time: TimeSpec) -> Result<()> {
        self.mboard(mboard)?;
        let state = self.state()?;
        state.require_idle()?;
        for timer in &state.time {
            timer.set_time_next_pps(time)?;
        }
        Ok(())
    }
    fn sensor_names(&self, scope: SensorScope) -> Result<Vec<String>> {
        match scope {
            SensorScope::Mboard(index) => {
                self.mboard(index)?;
                Ok(vec!["ref_locked".into()])
            }
            SensorScope::Channel { dir, chan } => {
                self.check_channel(dir, chan)?;
                Ok(vec!["lo_locked".into()])
            }
        }
    }
    fn sensor(&self, scope: SensorScope, name: &str) -> Result<SensorValue> {
        let _state = self.state()?;
        let value = match scope {
            SensorScope::Mboard(index) => {
                self.mboard(index)?;
                if name != "ref_locked" {
                    return Err(Error::KeyNotFound(name.into()));
                }
                self.shared.session.misc().ref_locked()?
            }
            SensorScope::Channel { dir, chan } => {
                self.check_channel(dir, chan)?;
                if name != "lo_locked" {
                    return Err(Error::KeyNotFound(name.into()));
                }
                {
                    let (tx, rx) = self.shared.session.misc().fe_pll_locked()?;
                    if dir == Direction::Rx { rx } else { tx }
                }
            }
        };
        Ok(SensorValue {
            name: name.into(),
            value: SensorData::Bool(value),
            unit: String::new(),
        })
    }
    fn open_rx_stream(&self, args: &StreamArgs) -> Result<Box<dyn RawRxStreamer>> {
        let chan = match args.channels.as_slice() {
            [] => 0,
            [chan] => *chan,
            _ => {
                return Err(Error::not_supported(
                    "simultaneous multi-channel B2xx receive",
                ));
            }
        };
        self.check_channel(Direction::Rx, chan)?;
        let mut state = self.state()?;
        state.require_idle()?;
        let options = RxOptions::new(state.dsp[chan].host_rate())
            .with_dsp_freq(state.dsp[chan].freq())
            .with_antenna(state.antennas[chan]);
        let rx = self.shared.session.rx_stream(&options, args)?;
        state.stream_channel = Some((Direction::Rx, chan));
        Ok(Box::new(ManagedRx {
            rx: Some(rx),
            shared: Arc::clone(&self.shared),
        }))
    }
    fn open_tx_stream(&self, args: &StreamArgs) -> Result<Box<dyn RawTxStreamer>> {
        let chan = match args.channels.as_slice() {
            [] => 0,
            [chan] => *chan,
            _ => return Err(Error::not_supported("multi-channel B2xx transmit")),
        };
        self.check_channel(Direction::Tx, chan)?;
        let mut state = self.state()?;
        state.require_idle()?;
        let options = TxOptions {
            rate: state.tx_dsp[chan].host_rate(),
            dsp_freq: state.tx_dsp[chan].freq(),
        };
        let tx = self.shared.session.tx_stream(&options, args)?;
        state.stream_channel = Some((Direction::Tx, chan));
        Ok(Box::new(ManagedTx {
            tx: Some(tx),
            shared: Arc::clone(&self.shared),
        }))
    }
}

#[derive(Debug)]
struct ManagedRx {
    rx: Option<B200RxStreamer>,
    shared: Arc<Shared>,
}

impl RawRxStreamer for ManagedRx {
    fn num_channels(&self) -> usize {
        1
    }
    fn max_num_samps(&self) -> usize {
        self.rx.as_ref().unwrap().max_num_samps()
    }
    fn host_format(&self) -> SampleFormat {
        self.rx.as_ref().unwrap().host_format()
    }
    fn wire_format(&self) -> SampleFormat {
        self.rx.as_ref().unwrap().wire_format()
    }
    fn issue_stream_cmd(&mut self, cmd: &StreamCmd) -> Result<()> {
        self.rx.as_mut().unwrap().issue_stream_cmd(cmd)
    }
    fn recv_raw(
        &mut self,
        buffs: &mut [&mut [u8]],
        timeout: Duration,
        md: &mut RxMetadata,
    ) -> Result<usize> {
        self.rx.as_mut().unwrap().recv_raw(buffs, timeout, md)
    }
}

impl Drop for ManagedRx {
    fn drop(&mut self) {
        let mut state = self.shared.state.lock().unwrap_or_else(|e| e.into_inner());
        // Finish USB cancellation and hardware cleanup before allowing a new
        // streamer or a clock change. The session stays alive throughout.
        drop(self.rx.take());
        state.stream_channel = None;
    }
}

#[derive(Debug)]
struct ManagedTx {
    tx: Option<B200TxStreamer>,
    shared: Arc<Shared>,
}

impl RawTxStreamer for ManagedTx {
    fn num_channels(&self) -> usize {
        1
    }
    fn max_num_samps(&self) -> usize {
        self.tx.as_ref().unwrap().max_num_samps()
    }
    fn host_format(&self) -> SampleFormat {
        self.tx.as_ref().unwrap().host_format()
    }
    fn wire_format(&self) -> SampleFormat {
        SampleFormat::Sc16
    }
    fn send_raw(&mut self, buffs: &[&[u8]], timeout: Duration, md: &TxMetadata) -> Result<usize> {
        self.tx.as_mut().unwrap().send_raw(buffs, timeout, md)
    }
    fn recv_async_msg(&mut self, timeout: Duration) -> Result<Option<AsyncMetadata>> {
        self.tx.as_mut().unwrap().recv_async_msg(timeout)
    }
}

impl Drop for ManagedTx {
    fn drop(&mut self) {
        let mut state = self.shared.state.lock().unwrap_or_else(|e| e.into_inner());
        drop(self.tx.take());
        state.stream_channel = None;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn tune_policies_preserve_the_rf_stage_and_offset_sign() {
        let request = TuneRequest::new(98e6).with_lo_offset(2e6);
        let (target, rf) = tune_plan(&request, 800e6, 16e6).unwrap();
        assert_eq!((target, rf, rf - target), (98e6, 100e6, 2e6));
        let mut request = TuneRequest::new(99e6);
        request.rf_freq_policy = TunePolicy::None;
        assert_eq!(tune_plan(&request, 100e6, 16e6).unwrap(), (99e6, 100e6));
        assert_eq!(
            tune_plan(&TuneRequest::new(1e6), 800e6, 16e6).unwrap(),
            (54e6, 70e6)
        );
    }

    #[test]
    fn invalid_tuning_and_open_options_are_rejected() {
        assert!(tune_plan(&TuneRequest::new(f64::NAN), 800e6, 16e6).is_err());
        assert!(
            tune_plan(
                &TuneRequest::new(98e6).with_dsp_freq(f64::INFINITY),
                800e6,
                16e6
            )
            .is_err()
        );
        for arg in [
            "master_clock_rate=0",
            "master_clock_rate=NaN",
            "master_clock_rate=62e6",
            "auto_flash=yes",
        ] {
            assert!(validate_open_args(&arg.parse().unwrap()).is_err(), "{arg}");
        }
        assert!(
            validate_open_args(&"master_clock_rate=32e6,auto_flash=true".parse().unwrap()).is_ok()
        );
    }

    #[test]
    fn radio_to_codec_mapping_accounts_for_atr_swap() {
        assert_eq!(chain_for(0, false), Chain::One);
        assert_eq!(chain_for(1, false), Chain::Two);
        assert_eq!(chain_for(0, true), Chain::Two);
        assert_eq!(chain_for(1, true), Chain::One);
    }
}
