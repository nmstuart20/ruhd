//! One radio chain's receive framer: `rx_vita_core_3000`.
//!
//! Ports `host/lib/usrp/cores/rx_vita_core_3000.cpp`. The framer takes the
//! DSP chain's output, cuts it into CHDR packets of a fixed number of samples,
//! stamps each with the stream ID and the time of its first sample, and sends
//! it to the host. It produces nothing until it is given a stream command.
//!
//! # Commands are three writes, and the last one fires
//!
//! A stream command is a 32-bit command word followed by a 64-bit time, and the
//! write of the time's low half is what latches the command into the core's
//! queue. The order of the three writes therefore matters, and the low half is
//! written even for a command that takes effect immediately.
//!
//! # Overflows stop the framer
//!
//! When the host falls behind and the FPGA's buffer fills, the framer sends an
//! inline overflow message and stops. A continuous stream has to be restarted
//! by the host, which is what [`RxFramer::handle_overflow`] does.

use std::fmt;

use tracing::warn;
use uhd_core::stream::{StreamCmd, StreamMode, StreamTime};
use uhd_core::{Error, Result};
use uhd_transport::RegisterIface;

/// `SR_RX_CTRL`, the base of the framer's registers. A byte address,
/// `TOREG(96)`.
pub const SR_RX_CTRL: u32 = 96 * 4;

/// Command word: flags in the top four bits, sample count below.
const REG_CTRL_CMD: u32 = 0;
/// Upper half of the command time, in ticks.
const REG_CTRL_TIME_HI: u32 = 4;
/// Lower half. Writing this latches the command.
const REG_CTRL_TIME_LO: u32 = 8;
/// Samples per packet.
const REG_FRAMER_MAXLEN: u32 = 4 * 4;
/// Stream ID stamped on outgoing packets.
const REG_FRAMER_SID: u32 = 4 * 4 + 4;

/// The largest burst one command can request; longer ones are chained.
pub const MAX_NUM_SAMPS: u64 = 0x0fff_ffff;

/// One radio's receive framer.
pub struct RxFramer {
    bus: Box<dyn RegisterIface>,
    base: u32,
    continuous: bool,
}

impl fmt::Debug for RxFramer {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("RxFramer")
            .field("base", &self.base)
            .field("continuous", &self.continuous)
            .finish_non_exhaustive()
    }
}

impl RxFramer {
    /// Binds the core at the standard B2xx offset, without writing anything.
    ///
    /// The reference's constructor writes a placeholder packet length; nothing
    /// here does, because [`RxFramer::set_nsamps_per_packet`] is always called
    /// before a stream starts.
    #[must_use]
    pub fn new(bus: Box<dyn RegisterIface>) -> Self {
        Self {
            bus,
            base: SR_RX_CTRL,
            continuous: false,
        }
    }

    /// Sets how many samples go in each packet.
    ///
    /// # Errors
    ///
    /// Whatever the settings bus reports.
    pub fn set_nsamps_per_packet(&self, nsamps: usize) -> Result<()> {
        let nsamps = u32::try_from(nsamps)
            .map_err(|_| Error::invalid_arg(format!("{nsamps} samples per packet")))?;
        self.bus.poke32(self.base + REG_FRAMER_MAXLEN, nsamps)
    }

    /// Sets the stream ID the framer stamps on its packets.
    ///
    /// # Errors
    ///
    /// Whatever the settings bus reports.
    pub fn set_sid(&self, sid: u32) -> Result<()> {
        self.bus.poke32(self.base + REG_FRAMER_SID, sid)
    }

    /// Queues a stream command.
    ///
    /// `tick_rate` converts a timed command's start time into ticks. A finite
    /// request for zero samples is ignored, as the reference ignores it.
    ///
    /// # Errors
    ///
    /// [`Error::InvalidArg`] for a burst longer than [`MAX_NUM_SAMPS`] or a
    /// time before zero; [`Error::NotSupported`] for a stream mode this core
    /// does not know; otherwise whatever the settings bus reports.
    pub fn issue_stream_cmd(&mut self, cmd: &StreamCmd, tick_rate: f64) -> Result<()> {
        if matches!(
            cmd.mode,
            StreamMode::NumSampsAndDone(0) | StreamMode::NumSampsAndMore(0)
        ) {
            warn!("ignoring a stream command for a finite acquisition of zero samples");
            return Ok(());
        }
        let word = command_word(cmd)?;
        let ticks = match cmd.time {
            StreamTime::Now => 0u64,
            StreamTime::At(time) => u64::try_from(time.to_ticks(tick_rate)).map_err(|_| {
                Error::invalid_arg(format!(
                    "a stream command cannot be timed before zero ({time:?})"
                ))
            })?,
        };
        self.continuous = cmd.mode == StreamMode::StartContinuous;
        self.bus.poke32(self.base + REG_CTRL_CMD, word)?;
        self.bus
            .poke32(self.base + REG_CTRL_TIME_HI, (ticks >> 32) as u32)?;
        self.bus.poke32(self.base + REG_CTRL_TIME_LO, ticks as u32)
    }

    /// Restarts a continuous stream after the framer stopped on an overflow.
    ///
    /// Does nothing for a finite or stopped stream, which the host did not
    /// expect to keep running.
    ///
    /// # Errors
    ///
    /// Whatever the settings bus reports.
    pub fn handle_overflow(&mut self, tick_rate: f64) -> Result<()> {
        if self.continuous {
            self.issue_stream_cmd(&StreamCmd::start_continuous(), tick_rate)?;
        }
        Ok(())
    }

    /// Whether the last command started a continuous stream.
    #[must_use]
    pub fn in_continuous_mode(&self) -> bool {
        self.continuous
    }
}

/// The framer's command word for `cmd`.
///
/// Bit 31 is "now", 30 "chain", 29 "reload", 28 "stop", and the low 28 bits
/// are the sample count. A continuous stream is a reloading, chained one-sample
/// burst, which is how the FPGA spells "forever".
fn command_word(cmd: &StreamCmd) -> Result<u32> {
    // (reload, chain, samps, stop)
    let (reload, chain, samps, stop) = match cmd.mode {
        StreamMode::StartContinuous => (true, true, None, false),
        StreamMode::StopContinuous => (false, false, None, true),
        StreamMode::NumSampsAndDone(n) => (false, false, Some(n), false),
        StreamMode::NumSampsAndMore(n) => (false, true, Some(n), false),
        other => {
            return Err(Error::not_supported(format!(
                "the b2xx rx framer has no encoding for stream mode {other:?}"
            )));
        }
    };
    if let Some(n) = samps.filter(|&n| n > MAX_NUM_SAMPS) {
        return Err(Error::invalid_arg(format!(
            "a stream command can request at most {MAX_NUM_SAMPS} samples, not {n}; \
             chain NumSampsAndMore commands for a longer burst"
        )));
    }
    let now = matches!(cmd.time, StreamTime::Now);
    let mut word = u32::from(now) << 31
        | u32::from(chain) << 30
        | u32::from(reload) << 29
        | u32::from(stop) << 28;
    word |= match samps {
        Some(n) => n as u32,
        None if stop => 0,
        None => 1,
    };
    Ok(word)
}

#[cfg(test)]
mod tests {
    use std::sync::Mutex;

    use uhd_core::TimeSpec;

    use super::*;

    #[derive(Debug, Default)]
    struct FakeBus {
        pokes: Mutex<Vec<(u32, u32)>>,
    }

    impl RegisterIface for &'static FakeBus {
        fn poke32(&self, addr: u32, data: u32) -> Result<()> {
            self.pokes.lock().unwrap().push((addr, data));
            Ok(())
        }
        fn peek32(&self, _addr: u32) -> Result<u32> {
            Ok(0)
        }
        fn peek64(&self, _addr: u32) -> Result<u64> {
            Ok(0)
        }
    }

    fn bus() -> &'static FakeBus {
        Box::leak(Box::default())
    }

    #[test]
    fn register_offsets_match_the_reference() {
        assert_eq!(SR_RX_CTRL, 384);
        assert_eq!((REG_FRAMER_MAXLEN, REG_FRAMER_SID), (16, 20));
    }

    #[test]
    fn command_words_match_rx_vita_core_3000() {
        assert_eq!(
            command_word(&StreamCmd::start_continuous()).unwrap(),
            0xe000_0001
        );
        assert_eq!(command_word(&StreamCmd::stop()).unwrap(), 0x9000_0000);
        assert_eq!(
            command_word(&StreamCmd::num_samps(1000)).unwrap(),
            0x8000_03e8
        );
        let chained = StreamCmd {
            mode: StreamMode::NumSampsAndMore(5),
            time: StreamTime::At(TimeSpec::from_secs(1.0)),
        };
        assert_eq!(command_word(&chained).unwrap(), 0x4000_0005);
        assert!(command_word(&StreamCmd::num_samps(MAX_NUM_SAMPS + 1)).is_err());
    }

    #[test]
    fn the_time_low_half_is_written_last() {
        let bus = bus();
        let mut framer = RxFramer::new(Box::new(bus));
        let cmd = StreamCmd::num_samps(10).at(TimeSpec::from_secs(2.0));
        framer.issue_stream_cmd(&cmd, 16e6).unwrap();
        let pokes = bus.pokes.lock().unwrap().clone();
        assert_eq!(
            pokes,
            [
                (SR_RX_CTRL, 10),
                (SR_RX_CTRL + REG_CTRL_TIME_HI, 0),
                (SR_RX_CTRL + REG_CTRL_TIME_LO, 32_000_000),
            ]
        );
    }

    #[test]
    fn only_a_continuous_stream_is_restarted_after_an_overflow() {
        let bus = bus();
        let mut framer = RxFramer::new(Box::new(bus));
        framer
            .issue_stream_cmd(&StreamCmd::num_samps(10), 1e6)
            .unwrap();
        framer.handle_overflow(1e6).unwrap();
        assert_eq!(bus.pokes.lock().unwrap().len(), 3);

        framer
            .issue_stream_cmd(&StreamCmd::start_continuous(), 1e6)
            .unwrap();
        framer.handle_overflow(1e6).unwrap();
        let pokes = bus.pokes.lock().unwrap().clone();
        assert_eq!(pokes.len(), 9);
        assert_eq!(pokes[6], (SR_RX_CTRL, 0xe000_0001));
    }

    #[test]
    fn zero_sample_bursts_write_nothing() {
        let bus = bus();
        let mut framer = RxFramer::new(Box::new(bus));
        framer
            .issue_stream_cmd(&StreamCmd::num_samps(0), 1e6)
            .unwrap();
        assert!(bus.pokes.lock().unwrap().is_empty());
    }
}
