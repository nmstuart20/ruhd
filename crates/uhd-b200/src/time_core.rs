//! The radio's 64-bit time counter, and where its PPS comes from.
//!
//! Ports `time_core_3000.cpp`, plus `b200_impl::update_time_source`,
//! `set_time`, `set_time_next_pps` and `sync_times`.
//!
//! # Three registers, three ways to load them
//!
//! The counter itself is read-only from the host: two readback addresses, one
//! holding the count now and one holding the count at the last PPS edge. Writing
//! it is a two-step — load a 64-bit value into a holding register, then poke a
//! control register to say *when* it should take effect:
//!
//! | Latch | What it means | Used by |
//! | --- | --- | --- |
//! | [`Latch::Now`] | immediately | `set_time_now` |
//! | [`Latch::NextPps`] | on the next PPS edge | `set_time_next_pps` |
//! | [`Latch::Sync`] | when the sync pulse arrives | `set_time`, to align two radios |
//!
//! The distinction is the whole reason the core exists. Loading "now" over USB
//! is accurate to whenever the packet happened to arrive, which is tens of
//! microseconds of jitter; loading on the next PPS is accurate to the edge.
//!
//! # Ticks, not seconds
//!
//! The counter runs at the radio's tick rate, which on a B2xx is the AD936x's
//! sample rate and therefore changes whenever the rate does. [`TimeCore`] holds
//! the rate it was told and converts, and `b200_impl` re-synchronises the
//! counters on every tick-rate change for exactly this reason — the count means
//! nothing without the rate that produced it. [`TimeCore::set_tick_rate`] is
//! therefore not a formality: getting it wrong silently scales every timestamp.
//!
//! # These are radio-core registers, except the source select
//!
//! `SR_TIME` and both readbacks are per radio chain, so a [`TimeCore`] is built
//! on a [`RadioCtrl::radio`](crate::RadioCtrl::radio) port and answers nothing
//! until the AD936x is clocking. The *source* select at `SR_CORE_SYNC` is on the
//! local core, which is why [`set_time_source`] takes its own bus.

use std::fmt;
use std::sync::Mutex;
use std::time::{Duration, Instant};

use tracing::{debug, warn};
use uhd_core::{Error, Result, TimeSpec};
use uhd_transport::RegisterIface;

use crate::compat::{FpgaCompat, FpgaShim};

/// `SR_TIME`, the base of the per-radio time core. `TOREG(128)`.
pub const SR_TIME: u32 = 128 * 4;

/// `RB64_TIME_NOW`, the running count.
pub const RB64_TIME_NOW: u32 = 8;

/// `RB64_TIME_PPS`, the count latched at the last PPS edge.
pub const RB64_TIME_PPS: u32 = 16;

/// When a loaded time value should take effect.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum Latch {
    /// Immediately, as the control write lands.
    Now,
    /// On the next PPS edge.
    NextPps,
    /// On the next sync pulse, which is how two radio chains are aligned.
    Sync,
}

impl Latch {
    /// The bit in the control register, from `time_core_3000.cpp`.
    fn bit(self) -> u32 {
        match self {
            Self::Now => 1 << 0,
            Self::NextPps => 1 << 1,
            Self::Sync => 1 << 2,
        }
    }
}

/// Where the radio's PPS and time reference come from.
///
/// The values are the `time_source_t` enum in `b200_impl.hpp`, and they are what
/// is written to `SR_CORE_SYNC` — so the numbering is the register's, not a
/// convention of ours.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum TimeSource {
    /// The onboard GPSDO. Full-size boards only.
    Gpsdo,
    /// The external PPS input.
    External,
    /// The internal reference.
    Internal,
    /// No PPS at all.
    None,
}

impl TimeSource {
    /// The register value. `time_source_t`.
    #[must_use]
    pub fn code(self) -> u32 {
        match self {
            Self::Gpsdo => 0,
            Self::External => 1,
            Self::Internal => 2,
            Self::None => 3,
        }
    }

    /// The name UHD uses in `/mboards/0/time_source`.
    #[must_use]
    pub fn name(self) -> &'static str {
        match self {
            Self::Gpsdo => "gpsdo",
            Self::External => "external",
            Self::Internal => "internal",
            Self::None => "none",
        }
    }

    /// Parses one of those names.
    ///
    /// # Errors
    ///
    /// [`Error::KeyNotFound`] if the name is not a time source at all, and
    /// [`Error::NotSupported`] if it is one this FPGA build does not have —
    /// `gpsdo` on a mini, which has no GPSDO slot.
    pub fn parse(name: &str, shim: &'static dyn FpgaShim) -> Result<Self> {
        let source = match name {
            "gpsdo" => Self::Gpsdo,
            "external" => Self::External,
            "internal" => Self::Internal,
            "none" => Self::None,
            other => {
                return Err(Error::KeyNotFound(format!(
                    "time source `{other}`; this device offers {}",
                    shim.time_sources().join(", ")
                )));
            }
        };
        if !shim.time_sources().contains(&source.name()) {
            return Err(Error::not_supported(format!(
                "a {} board has no `{}` time source; it offers {}",
                shim.codename(),
                source.name(),
                shim.time_sources().join(", ")
            )));
        }
        Ok(source)
    }
}

impl fmt::Display for TimeSource {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(self.name())
    }
}

/// Selects the PPS source on the local core.
///
/// `b200_impl::update_time_source`, which is one poke to `SR_CORE_SYNC`.
///
/// # Errors
///
/// Whatever the settings bus reports.
pub fn set_time_source(
    bus: &dyn RegisterIface,
    compat: FpgaCompat,
    source: TimeSource,
) -> Result<()> {
    bus.poke32(compat.shim().core_regs().sr_sync, source.code())?;
    debug!(%source, "selected the b2xx time source");
    Ok(())
}

/// Arms the sync pulse, then disarms it.
///
/// `b200_impl::set_time`'s two writes after the radios have been loaded: the
/// first sets bit 2 alongside the current source, which fires the pulse the
/// radios' [`Latch::Sync`] is waiting on, and the second puts the register back.
/// Without the second write the pulse would stay asserted and the next load
/// would take effect the instant it was armed.
///
/// # Errors
///
/// Whatever the settings bus reports.
pub fn pulse_sync(bus: &dyn RegisterIface, compat: FpgaCompat, source: TimeSource) -> Result<()> {
    let addr = compat.shim().core_regs().sr_sync;
    bus.poke32(addr, (1 << 2) | source.code())?;
    bus.poke32(addr, source.code())
}

/// One radio chain's `time_core_3000`.
#[derive(Debug)]
pub struct TimeCore {
    bus: Box<dyn RegisterIface>,
    base: u32,
    rb_now: u32,
    rb_pps: u32,
    tick_rate: Mutex<f64>,
}

impl TimeCore {
    /// Binds the core at the standard B2xx offsets.
    ///
    /// `bus` must be a port onto a *radio* core. The tick rate starts at 1, as
    /// `time_core_3000_impl`'s constructor does, so that a division by it is
    /// never a division by zero — but it is not a usable rate and
    /// [`TimeCore::set_tick_rate`] has to be called before any timestamp means
    /// anything.
    #[must_use]
    pub fn new(bus: Box<dyn RegisterIface>) -> Self {
        Self {
            bus,
            base: SR_TIME,
            rb_now: RB64_TIME_NOW,
            rb_pps: RB64_TIME_PPS,
            tick_rate: Mutex::new(1.0),
        }
    }

    /// Tells the core what rate the counter is running at.
    ///
    /// Writes nothing: the rate is a host-side conversion factor, and the
    /// counter's actual rate is set by the codec.
    ///
    /// # Errors
    ///
    /// [`Error::InvalidArg`] if the rate is not a positive, finite number. A
    /// zero or NaN rate would turn every timestamp into a silent zero, which is
    /// worse than a refusal.
    pub fn set_tick_rate(&self, rate: f64) -> Result<()> {
        if !rate.is_finite() || rate <= 0.0 {
            return Err(Error::invalid_arg(format!(
                "a tick rate is a positive number of ticks per second, not {rate}"
            )));
        }
        *self.locked() = rate;
        Ok(())
    }

    /// The rate the core is converting with.
    #[must_use]
    pub fn tick_rate(&self) -> f64 {
        *self.locked()
    }

    /// The counter, now.
    ///
    /// # Errors
    ///
    /// Whatever the settings bus reports. A radio core whose codec is not
    /// clocking does not answer at all, so this is where that shows up.
    pub fn get_time_now(&self) -> Result<TimeSpec> {
        self.read(self.rb_now)
    }

    /// The counter as it stood at the last PPS edge.
    ///
    /// # Errors
    ///
    /// As [`TimeCore::get_time_now`].
    pub fn get_time_last_pps(&self) -> Result<TimeSpec> {
        self.read(self.rb_pps)
    }

    /// Loads a time, to take effect immediately.
    ///
    /// # Errors
    ///
    /// Whatever the settings bus reports.
    pub fn set_time_now(&self, time: TimeSpec) -> Result<()> {
        self.load(time, Latch::Now)
    }

    /// Loads a time, to take effect on the next PPS edge.
    ///
    /// # Errors
    ///
    /// Whatever the settings bus reports.
    pub fn set_time_next_pps(&self, time: TimeSpec) -> Result<()> {
        self.load(time, Latch::NextPps)
    }

    /// Loads a time, to take effect on the next sync pulse.
    ///
    /// Pair with [`pulse_sync`] on the local core, which is what fires it.
    ///
    /// # Errors
    ///
    /// Whatever the settings bus reports.
    pub fn set_time_sync(&self, time: TimeSpec) -> Result<()> {
        self.load(time, Latch::Sync)
    }

    /// Reads the counter twice around a known wall-clock delay and checks that
    /// it advanced by roughly the right amount.
    ///
    /// `time_core_3000::self_test`, with one change: this returns the result
    /// instead of logging a warning and continuing. A counter that is not
    /// advancing at the rate the host believes means every timed command and
    /// every timestamp is wrong, and that is worth a caller's decision rather
    /// than a line in a log.
    ///
    /// # Errors
    ///
    /// [`Error::Device`] if the elapsed count is outside half to one-and-a-half
    /// times the expected — the same window as the reference. Otherwise
    /// whatever the settings bus reports.
    pub fn self_test(&self) -> Result<TimerCheck> {
        const SLEEP: Duration = Duration::from_millis(100);

        let before = self.get_time_now()?;
        let started = Instant::now();
        std::thread::sleep(SLEEP);
        let after = self.get_time_now()?;
        // The host clock, not the nominal sleep: a loaded machine oversleeps,
        // and blaming the FPGA for that is how a flaky test is born.
        let host_elapsed = started.elapsed().as_secs_f64();
        let counted = (after - before).as_secs_f64();

        let check = TimerCheck {
            counted,
            host_elapsed,
            apparent_rate: if host_elapsed > 0.0 {
                counted * self.tick_rate() / host_elapsed
            } else {
                0.0
            },
            expected_rate: self.tick_rate(),
        };
        if counted < host_elapsed * 0.5 || counted > host_elapsed * 1.5 {
            warn!(?check, "b2xx timer loopback test failed");
            return Err(Error::Device(format!(
                "the radio's time counter advanced {counted:.6} s while the host clock \
                 advanced {host_elapsed:.6} s; it appears to run at {:.3} MHz rather than \
                 the {:.3} MHz this core was told",
                check.apparent_rate / 1e6,
                check.expected_rate / 1e6
            )));
        }
        Ok(check)
    }

    fn read(&self, addr: u32) -> Result<TimeSpec> {
        let ticks = self.bus.peek64(addr)?;
        Ok(TimeSpec::from_ticks(ticks as i64, self.tick_rate()))
    }

    fn load(&self, time: TimeSpec, latch: Latch) -> Result<()> {
        let ticks = time.to_ticks(self.tick_rate());
        // High word first, then low, then the latch. The order is the
        // reference's and it is load-bearing: the control write is what commits
        // the pair, so a partially written value never takes effect.
        self.bus.poke32(self.base, (ticks >> 32) as u32)?;
        self.bus.poke32(self.base + 4, ticks as u32)?;
        self.bus.poke32(self.base + 8, latch.bit())
    }

    fn locked(&self) -> std::sync::MutexGuard<'_, f64> {
        self.tick_rate
            .lock()
            .unwrap_or_else(std::sync::PoisonError::into_inner)
    }
}

/// What [`TimeCore::self_test`] measured.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct TimerCheck {
    /// Seconds the counter advanced by.
    pub counted: f64,
    /// Seconds the host clock advanced by over the same interval.
    pub host_elapsed: f64,
    /// The tick rate implied by those two.
    pub apparent_rate: f64,
    /// The tick rate the core was told to expect.
    pub expected_rate: f64,
}

impl fmt::Display for TimerCheck {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(
            f,
            "counted {:.6} s against {:.6} s of host clock ({:.3} MHz, expected {:.3} MHz)",
            self.counted,
            self.host_elapsed,
            self.apparent_rate / 1e6,
            self.expected_rate / 1e6
        )
    }
}

#[cfg(test)]
mod tests {
    use std::sync::Mutex as StdMutex;

    use super::*;

    #[derive(Debug, Default)]
    struct FakeBus {
        pokes: StdMutex<Vec<(u32, u32)>>,
        peek64: StdMutex<Vec<(u32, u64)>>,
    }

    impl FakeBus {
        fn pokes(&self) -> Vec<(u32, u32)> {
            self.pokes.lock().unwrap().clone()
        }
    }

    impl RegisterIface for &'static FakeBus {
        fn poke32(&self, addr: u32, data: u32) -> Result<()> {
            self.pokes.lock().unwrap().push((addr, data));
            Ok(())
        }

        fn peek32(&self, addr: u32) -> Result<u32> {
            Ok(self.peek64(addr)? as u32)
        }

        fn peek64(&self, addr: u32) -> Result<u64> {
            let queue = self.peek64.lock().unwrap();
            Ok(queue
                .iter()
                .find(|(a, _)| *a == addr)
                .map_or(0, |(_, v)| *v))
        }
    }

    fn bus() -> &'static FakeBus {
        Box::leak(Box::new(FakeBus::default()))
    }

    /// 61.44 MHz, a real B2xx master clock rate.
    const RATE: f64 = 61.44e6;

    #[test]
    fn the_register_offsets_match_the_reference() {
        // SR_TIME is TOREG(128); the readbacks are already byte addresses and
        // are 64 bits wide, which is why they are 8 apart rather than 4.
        assert_eq!(SR_TIME, 512);
        assert_eq!((RB64_TIME_NOW, RB64_TIME_PPS), (8, 16));

        // REG_TIME_HI/LO/CTRL are base + 0/4/8.
        let core = TimeCore::new(Box::new(bus()));
        assert_eq!((core.base, core.base + 4, core.base + 8), (512, 516, 520));
    }

    #[test]
    fn the_latch_bits_match_time_core_3000() {
        assert_eq!(Latch::Now.bit(), 1);
        assert_eq!(Latch::NextPps.bit(), 2);
        assert_eq!(Latch::Sync.bit(), 4);
    }

    #[test]
    fn loading_a_time_writes_high_word_low_word_then_the_latch() {
        // The order is what makes the write atomic from the FPGA's side: the
        // control poke is the commit, so a half-written value cannot take
        // effect.
        let bus = bus();
        let core = TimeCore::new(Box::new(bus));
        core.set_tick_rate(RATE).unwrap();

        core.set_time_next_pps(TimeSpec::new(100, 0.5)).unwrap();

        let ticks = TimeSpec::new(100, 0.5).to_ticks(RATE);
        assert_eq!(
            bus.pokes(),
            vec![
                (512, (ticks >> 32) as u32),
                (516, ticks as u32),
                (520, Latch::NextPps.bit()),
            ]
        );
    }

    #[test]
    fn each_latch_writes_its_own_control_bit() {
        let bus = bus();
        let core = TimeCore::new(Box::new(bus));
        core.set_tick_rate(RATE).unwrap();

        core.set_time_now(TimeSpec::ZERO).unwrap();
        core.set_time_next_pps(TimeSpec::ZERO).unwrap();
        core.set_time_sync(TimeSpec::ZERO).unwrap();

        let latches: Vec<u32> = bus
            .pokes()
            .into_iter()
            .filter(|(addr, _)| *addr == 520)
            .map(|(_, v)| v)
            .collect();
        assert_eq!(latches, vec![1, 2, 4]);
    }

    #[test]
    fn the_two_readbacks_come_from_different_addresses() {
        // Reading "now" where "last pps" was meant is a bug that looks like
        // working code until a PPS actually arrives.
        let bus = bus();
        bus.peek64
            .lock()
            .unwrap()
            .extend([(RB64_TIME_NOW, 61_440_000), (RB64_TIME_PPS, 122_880_000)]);

        let core = TimeCore::new(Box::new(bus));
        core.set_tick_rate(RATE).unwrap();

        assert_eq!(core.get_time_now().unwrap(), TimeSpec::new(1, 0.0));
        assert_eq!(core.get_time_last_pps().unwrap(), TimeSpec::new(2, 0.0));
    }

    #[test]
    fn a_time_round_trips_through_the_tick_rate() {
        let bus = bus();
        let core = TimeCore::new(Box::new(bus));
        core.set_tick_rate(RATE).unwrap();

        let wanted = TimeSpec::new(3600, 0.25);
        core.set_time_now(wanted).unwrap();
        let written = bus.pokes();
        let ticks = (u64::from(written[0].1) << 32) | u64::from(written[1].1);
        assert_eq!(TimeSpec::from_ticks(ticks as i64, RATE), wanted);
    }

    #[test]
    fn an_unusable_tick_rate_is_refused_rather_than_silently_zeroing_time() {
        let core = TimeCore::new(Box::new(bus()));
        for rate in [0.0, -1.0, f64::NAN, f64::INFINITY] {
            assert!(core.set_tick_rate(rate).is_err(), "{rate}");
        }
        // And the constructor's placeholder is left in place, not overwritten
        // by a rejected value.
        assert_eq!(core.tick_rate(), 1.0);
    }

    #[test]
    fn the_time_source_codes_are_the_register_values() {
        // time_source_t in b200_impl.hpp. These go straight into SR_CORE_SYNC,
        // so the numbering is not ours to choose.
        assert_eq!(TimeSource::Gpsdo.code(), 0);
        assert_eq!(TimeSource::External.code(), 1);
        assert_eq!(TimeSource::Internal.code(), 2);
        assert_eq!(TimeSource::None.code(), 3);
    }

    #[test]
    fn a_mini_has_no_gpsdo_to_select() {
        let mini = FpgaCompat::V7.shim();
        let full = FpgaCompat::V16.shim();

        assert_eq!(
            TimeSource::parse("external", mini).unwrap(),
            TimeSource::External
        );
        assert_eq!(TimeSource::parse("gpsdo", full).unwrap(), TimeSource::Gpsdo);

        // The two failure modes are different and the messages say so: one is a
        // typo, the other is a board that does not have the thing.
        let missing = TimeSource::parse("gpsdo", mini).unwrap_err();
        assert!(matches!(missing, Error::NotSupported(_)), "{missing}");
        let typo = TimeSource::parse("interal", mini).unwrap_err();
        assert!(matches!(typo, Error::KeyNotFound(_)), "{typo}");
    }

    #[test]
    fn selecting_a_source_pokes_the_local_sync_register() {
        let bus = bus();
        set_time_source(&bus, FpgaCompat::V7, TimeSource::External).unwrap();
        assert_eq!(bus.pokes(), vec![(192, 1)], "SR_CORE_SYNC is TOREG(48)");
    }

    #[test]
    fn the_sync_pulse_is_armed_and_then_cleared() {
        // b200_impl::set_time. Leaving bit 2 asserted would make the next
        // sync-latched load take effect the moment it was armed.
        let bus = bus();
        pulse_sync(&bus, FpgaCompat::V16, TimeSource::Internal).unwrap();
        assert_eq!(bus.pokes(), vec![(192, (1 << 2) | 2), (192, 2)]);
    }
}
