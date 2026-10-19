//! B2xx FPGA settings-bus control over the dedicated USB bulk endpoints.
//!
//! # One transport, several cores
//!
//! The bulk endpoint pair is a single pipe, and which core in the FPGA a packet
//! reaches is decided by the stream ID it carries. There are two kinds:
//!
//! | Core | Stream ID | Holds | Clocked by |
//! | --- | --- | --- | --- |
//! | local | [`LOCAL_CTRL_SID`] | compat register, SPI, the misc GPIO word, time-source select | the FPGA's bus clock |
//! | radio *n* | [`FpgaShim::radio_sids`] | ATR, the time core, the DSP chains, scratch | the AD936x's sample clock |
//!
//! The second column of that last row is the reason this module is shaped the
//! way it is. The local core answers as soon as the bitstream is in; the radio
//! cores do not answer at all until the codec is running, and the codec is
//! started over SPI — which is on the local core. So bring-up is necessarily a
//! sequence of local-core work followed by radio-core work, on the same wire,
//! and something has to carry the stream ID.
//!
//! [`CtrlPort`] is that something: a cheap, cloneable [`RegisterIface`] bound to
//! one stream ID, sharing the single transport underneath. `RadioCtrl::local()`
//! and [`RadioCtrl::radio`] hand them out, and a core built on one — the SPI
//! core, the ATR core, the time core — never learns which it got.
//!
//! [`FpgaShim::radio_sids`]: crate::compat::FpgaShim::radio_sids

use std::collections::VecDeque;
use std::sync::{Arc, Mutex};
use std::time::Duration;

use chdr::{Endian, IfPacketInfo, LinkType, PacketType};
use nusb::MaybeFuture;
use nusb::transfer::{Buffer, Bulk, In, Out};
use tracing::{debug, trace, warn};
use uhd_core::{AsyncEvent, AsyncMetadata, Error, Result, TimeSpec};
use uhd_transport::usb::{FpgaVersion, Fx3Iface};
use uhd_transport::{RegisterIface, UsbEndpoint};

use crate::compat::{FPGA_SIGNATURE, FpgaCompat, FpgaShim};

/// USB interface carrying settings-bus commands. Wired by `b200_impl.hpp`.
pub const CTRL_SEND_INTERFACE: u8 = 3;
/// Bulk OUT endpoint on [`CTRL_SEND_INTERFACE`].
pub const CTRL_SEND_ENDPOINT: u8 = 0x04;
/// USB interface carrying settings-bus responses.
pub const CTRL_RECV_INTERFACE: u8 = 4;
/// Bulk IN endpoint on [`CTRL_RECV_INTERFACE`].
pub const CTRL_RECV_ENDPOINT: u8 = 0x88;
/// Stream ID of the local (non-radio) control core. `B200_LOCAL_CTRL_SID`.
pub const LOCAL_CTRL_SID: u32 = 0x0000_0040;
/// Its response stream ID. `B200_LOCAL_RESP_SID`.
pub const LOCAL_RESP_SID: u32 = 0x0040_0000;
/// Settings-bus address of the FPGA compatibility register.
pub const FPGA_COMPAT_ADDR: u32 = 0;
const ACK_TIMEOUT: Duration = Duration::from_secs(2);
const FLUSH_TIMEOUT: Duration = Duration::from_millis(10);
const MAX_OUTSTANDING: usize = 3;

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
enum SequenceRelation {
    Match,
    Stale,
    Future,
}

fn sequence_relation(expected: u16, got: u16) -> SequenceRelation {
    if expected == got {
        SequenceRelation::Match
    } else if got.wrapping_sub(expected) & 0xfff < 0x800 {
        SequenceRelation::Future
    } else {
        SequenceRelation::Stale
    }
}

/// Successful result of the control-channel hardware gate.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct ControlGate {
    /// The compatibility register as the FPGA reported it.
    pub fpga: FpgaVersion,
    /// How many poke/readback round trips completed without a mismatch.
    pub scratch_iterations: usize,
}

/// Thread-safe B2xx radio control core, bound to one FPGA compatibility major.
///
/// The major is negotiated before this is opened — see
/// [`session`](crate::session) — and every register address below comes from
/// its [`FpgaShim`] rather than from a constant. That is what keeps a second
/// supported major from meaning a second set of `if` statements in here.
#[derive(Clone, Debug)]
pub struct RadioCtrl {
    inner: Arc<Mutex<Transport>>,
    endpoint: UsbEndpoint,
    compat: FpgaCompat,
    sr_readback: u32,
}

/// A command awaiting its acknowledgement.
///
/// The response stream ID travels with the sequence number because commands to
/// different cores are in flight on the same pipe: matching a reply needs to
/// know which core it should have come from, and the queue is the only thing
/// that still remembers.
#[derive(Clone, Copy, Debug)]
struct Pending {
    seq: u16,
    resp_sid: u32,
}

#[derive(Debug)]
struct Transport {
    send: nusb::Endpoint<Bulk, Out>,
    recv: nusb::Endpoint<Bulk, In>,
    seq_out: u64,
    outstanding: VecDeque<Pending>,
    frame_size: usize,
    async_events: VecDeque<TxEvent>,
}

#[derive(Debug)]
struct TxEvent {
    channel: usize,
    tsf: Option<u64>,
    event: AsyncEvent,
}

fn tx_event(packet: &chdr::IfPacket) -> Option<TxEvent> {
    let channel = match packet.info.sid? {
        0x0050_0000 => 0,
        0x0060_0000 => 1,
        _ => return None,
    };
    if packet.info.packet_type != PacketType::Context {
        return None;
    }
    let event = match packet.payload.first()? & 0xff {
        1 => AsyncEvent::BurstAck,
        2 => AsyncEvent::Underflow,
        4 => AsyncEvent::SeqError,
        8 => AsyncEvent::TimeError,
        16 => AsyncEvent::UnderflowInPacket,
        32 => AsyncEvent::SeqErrorInBurst,
        64 => {
            let mut words = [0; 4];
            for (dst, src) in words.iter_mut().zip(packet.payload.iter().skip(1)) {
                *dst = *src;
            }
            AsyncEvent::UserPayload(words)
        }
        _ => return None,
    };
    Some(TxEvent {
        channel,
        tsf: packet.info.tsf,
        event,
    })
}

/// A settings-bus port bound to one core's stream ID.
///
/// Handed out by [`RadioCtrl::local`] and [`RadioCtrl::radio`]. Cloning is
/// cheap and every clone shares the one transport, so a core can hold its own
/// port for its lifetime without the caller managing routing.
#[derive(Clone, Debug)]
pub struct CtrlPort {
    inner: Arc<Mutex<Transport>>,
    endpoint: UsbEndpoint,
    ctrl_sid: u32,
    resp_sid: u32,
    sr_readback: u32,
}

impl CtrlPort {
    /// The command stream ID this port sends under.
    #[must_use]
    pub fn ctrl_sid(&self) -> u32 {
        self.ctrl_sid
    }

    /// The response stream ID it expects back.
    #[must_use]
    pub fn resp_sid(&self) -> u32 {
        self.resp_sid
    }

    /// Whether this port addresses the local (non-radio) core.
    ///
    /// Worth asking before a bring-up step: a write to a radio core before the
    /// codec is clocking does not fail, it hangs for the full acknowledgement
    /// timeout.
    #[must_use]
    pub fn is_local(&self) -> bool {
        self.ctrl_sid == LOCAL_CTRL_SID
    }

    fn locked(&self) -> std::sync::MutexGuard<'_, Transport> {
        self.inner
            .lock()
            .unwrap_or_else(std::sync::PoisonError::into_inner)
    }

    /// Names the endpoint and the core in a transport error, which is the
    /// difference between "the device stopped answering" and "the radio core
    /// has no clock yet".
    fn attribute(&self, e: Error) -> Error {
        match e {
            Error::Transport(s) => Error::Transport(format!(
                "{s} on {} (sid {:#06x}, {} core)",
                self.endpoint,
                self.ctrl_sid,
                if self.is_local() { "local" } else { "radio" }
            )),
            other => other,
        }
    }
}

impl RadioCtrl {
    /// Claim interfaces 3/4 and open endpoints 0x04/0x88.
    ///
    /// `compat` is the major [`negotiate`](crate::policy::negotiate) settled on;
    /// it selects the register map this core will speak.
    ///
    /// # Errors
    ///
    /// [`Error::Usb`] if an interface cannot be claimed or an endpoint opened,
    /// which on Linux usually means another process — a running
    /// `uhd_usrp_probe`, say — is holding the device.
    pub fn open(iface: &Fx3Iface, compat: FpgaCompat) -> Result<Self> {
        let device = iface.device();
        let claim = |n| {
            device
                .claim_interface(n)
                .wait()
                .map_err(|e| Error::Usb(format!("cannot claim B2xx ctrl interface {n}: {e}")))
        };
        let send = claim(CTRL_SEND_INTERFACE)?
            .endpoint::<Bulk, Out>(CTRL_SEND_ENDPOINT)
            .map_err(|e| Error::Usb(format!("cannot open B2xx ctrl OUT endpoint: {e}")))?;
        let recv = claim(CTRL_RECV_INTERFACE)?
            .endpoint::<Bulk, In>(CTRL_RECV_ENDPOINT)
            .map_err(|e| Error::Usb(format!("cannot open B2xx ctrl IN endpoint: {e}")))?;
        let frame_size = recv.max_packet_size().max(32);
        let mut inner = Transport {
            send,
            recv,
            seq_out: 0,
            outstanding: VecDeque::new(),
            frame_size,
            async_events: VecDeque::new(),
        };
        inner.flush();
        Ok(Self {
            inner: Arc::new(Mutex::new(inner)),
            endpoint: iface.endpoint().clone(),
            compat,
            sr_readback: compat.shim().radio_regs().sr_readback,
        })
    }

    /// The FPGA compatibility major this core was opened against.
    #[must_use]
    pub fn compat(&self) -> FpgaCompat {
        self.compat
    }

    /// The behaviour shim for that major.
    #[must_use]
    pub fn shim(&self) -> &'static dyn FpgaShim {
        self.compat.shim()
    }

    /// Which device this is.
    #[must_use]
    pub fn endpoint(&self) -> &UsbEndpoint {
        &self.endpoint
    }

    /// Polls TX events on the shared control endpoint without consuming register replies.
    pub(crate) fn recv_tx_event(
        &self,
        channel: usize,
        tick_rate: f64,
        timeout: Duration,
    ) -> Result<Option<AsyncMetadata>> {
        let mut t = self
            .inner
            .lock()
            .map_err(|_| Error::internal("B2xx control lock poisoned"))?;
        // Retire pending register operations through the same demultiplexer.
        while !t.outstanding.is_empty() {
            t.wait_one(false)?;
        }
        let deadline = std::time::Instant::now() + timeout;
        let mut polled = false;
        loop {
            if let Some(index) = t.async_events.iter().position(|e| e.channel == channel) {
                let event = t.async_events.remove(index).unwrap();
                return Ok(Some(AsyncMetadata {
                    channel,
                    time_spec: event
                        .tsf
                        .map(|ticks| TimeSpec::from_ticks(ticks as i64, tick_rate)),
                    event: event.event,
                }));
            }
            if polled && std::time::Instant::now() >= deadline {
                return Ok(None);
            }
            polled = true;
            let wait = deadline.saturating_duration_since(std::time::Instant::now());
            let frame_size = t.frame_size;
            let c = t.recv.transfer_blocking(Buffer::new(frame_size), wait);
            if c.status == Err(nusb::transfer::TransferError::Cancelled) {
                return Ok(None);
            }
            let bytes = c
                .into_result()
                .map_err(|e| Error::Transport(format!("TX event receive failed: {e}")))?;
            if let Ok(packet) = chdr::unpack(&bytes, LinkType::Chdr, Endian::Little) {
                t.queue_tx_event(&packet);
            }
        }
    }

    pub(crate) fn clear_tx_events(&self) -> Result<()> {
        // A readback is a barrier after the deframer reset and routes stale events.
        self.radio(0)?.peek32(0)?;
        self.inner
            .lock()
            .map_err(|_| Error::internal("B2xx control lock poisoned"))?
            .async_events
            .clear();
        Ok(())
    }

    /// A port onto the local core: the compatibility register, the SPI core,
    /// the misc GPIO word, the time-source select.
    ///
    /// Everything reachable through this answers as soon as the FPGA is
    /// configured.
    #[must_use]
    pub fn local(&self) -> CtrlPort {
        self.port(LOCAL_CTRL_SID, LOCAL_RESP_SID)
    }

    /// A port onto radio chain `radio`: ATR, the time core, the DSP chains.
    ///
    /// Nothing here answers until the AD936x is clocking, because the radio
    /// cores run off the codec's sample clock. Against a codec that has not
    /// been brought up the symptom is the full acknowledgement timeout rather
    /// than an error.
    ///
    /// # Errors
    ///
    /// [`Error::Device`] if this FPGA build has no such chain.
    pub fn radio(&self, radio: usize) -> Result<CtrlPort> {
        let (ctrl_sid, resp_sid) = self.shim().radio_sids(radio).ok_or_else(|| {
            Error::Device(format!(
                "an fpga compat {} build has no radio chain {radio}",
                self.compat.major()
            ))
        })?;
        Ok(self.port(ctrl_sid, resp_sid))
    }

    fn port(&self, ctrl_sid: u32, resp_sid: u32) -> CtrlPort {
        CtrlPort {
            inner: Arc::clone(&self.inner),
            endpoint: self.endpoint.clone(),
            ctrl_sid,
            resp_sid,
            sr_readback: self.sr_readback,
        }
    }

    /// Reads the FPGA compatibility register.
    ///
    /// The same 64-bit readback `check_fpga_compat` performs. This is a
    /// *measurement*, not a decision: what to do about the answer is
    /// [`negotiate`](crate::policy::negotiate)'s.
    ///
    /// # Errors
    ///
    /// [`Error::Transport`] if the settings bus does not answer.
    pub fn fpga_version(&self) -> Result<FpgaVersion> {
        Ok(FpgaVersion::from_register(self.peek64(FPGA_COMPAT_ADDR)?))
    }

    /// Confirms the FPGA is the build this core was opened for, then performs
    /// 10,000 deterministic scratch-register poke/readback transactions with
    /// zero mismatches.
    ///
    /// The compatibility check here is a re-verification against the negotiated
    /// major rather than a second policy: [`RadioCtrl::open`] was handed the
    /// answer, and this is where the hardware is asked to agree with it. A
    /// disagreement means the FPGA was reconfigured between negotiation and
    /// now, which is worth failing on rather than streaming through.
    ///
    /// # Errors
    ///
    /// [`Error::Firmware`] if the signature or the major does not match;
    /// [`Error::Device`] on a scratch mismatch, naming the iteration and both
    /// values; [`Error::Transport`] if the bus stops answering.
    pub fn gate(&self) -> Result<ControlGate> {
        let fpga = self.fpga_version()?;
        if fpga.signature != FPGA_SIGNATURE {
            return Err(Error::Firmware(format!(
                "B2xx FPGA signature is {:#010x}, expected {FPGA_SIGNATURE:#010x}",
                fpga.signature
            )));
        }
        if fpga.major != self.compat.major() {
            return Err(Error::Firmware(format!(
                "B2xx FPGA compatibility is {fpga}, but this control core was opened \
                 against {}; the fpga has been reconfigured since it was negotiated",
                self.compat
            )));
        }

        // SR_TEST/RB32_TEST are radio-core registers, not local-core
        // registers. UHD runs register_loopback_self_test() on perif.ctrl,
        // whose first radio is routed under SID 0x10.
        let regs = self.shim().radio_regs();
        let radio = self.radio(0)?;

        let mut value = 0x6d2b_79f5u32;
        for iteration in 0..10_000 {
            // Fixed xorshift sequence makes a failure exactly reproducible.
            value ^= value << 13;
            value ^= value >> 17;
            value ^= value << 5;
            radio.poke32(regs.sr_test, value)?;
            let got = radio.peek32(regs.rb32_test)?;
            if got != value {
                return Err(Error::Device(format!(
                    "B2xx scratch mismatch at iteration {iteration}: wrote {value:#010x}, \
                     read {got:#010x}"
                )));
            }
        }
        debug!(
            ?fpga,
            compat = self.compat.major(),
            "B2xx radio control gate passed"
        );
        Ok(ControlGate {
            fpga,
            scratch_iterations: 10_000,
        })
    }
}

impl Transport {
    fn queue_tx_event(&mut self, packet: &chdr::IfPacket) -> bool {
        if let Some(event) = tx_event(packet) {
            if self.async_events.len() >= 1000 {
                self.async_events.pop_front();
            }
            self.async_events.push_back(event);
            true
        } else {
            false
        }
    }
    fn flush(&mut self) {
        for _ in 0..100 {
            let c = self
                .recv
                .transfer_blocking(Buffer::new(self.frame_size), FLUSH_TIMEOUT);
            if c.status.is_err() || c.actual_len == 0 {
                break;
            }
        }
        self.outstanding.clear();
    }

    fn send(&mut self, port: &CtrlPort, addr: u32, data: u32) -> Result<()> {
        let seq = (self.seq_out & 0xfff) as u16;
        let info = IfPacketInfo {
            link_type: LinkType::Chdr,
            packet_type: PacketType::Context,
            packet_count: seq,
            sid: Some(port.ctrl_sid),
            payload_bytes: 8,
            ..Default::default()
        };
        let packet = chdr::pack(&info, &[addr, data], Endian::Little)
            .map_err(|e| Error::Internal(format!("cannot build B2xx control packet: {e}")))?;
        self.send
            .transfer_blocking(packet.into(), ACK_TIMEOUT)
            .into_result()
            .map_err(|e| Error::Transport(format!("B2xx control seq {seq} send failed: {e}")))?;
        self.outstanding.push_back(Pending {
            seq,
            resp_sid: port.resp_sid,
        });
        self.seq_out = self.seq_out.wrapping_add(1);
        trace!(
            seq,
            sid = port.ctrl_sid,
            addr,
            data,
            "B2xx control packet sent"
        );
        Ok(())
    }

    fn wait_one(&mut self, readback: bool) -> Result<Option<u64>> {
        let Pending { seq, resp_sid } = self.outstanding.pop_front().ok_or_else(|| {
            Error::Internal("control response requested with nothing outstanding".into())
        })?;
        loop {
            let completion = self
                .recv
                .transfer_blocking(Buffer::new(self.frame_size), ACK_TIMEOUT);
            let bytes = match completion.into_result() {
                Ok(b) => b,
                Err(e) => {
                    self.flush();
                    return Err(Error::Transport(format!(
                        "B2xx control seq {seq} timed out: {e}; response state resynchronized"
                    )));
                }
            };
            let packet = match chdr::unpack(&bytes, LinkType::Chdr, Endian::Little) {
                Ok(p) => p,
                Err(e) => {
                    warn!(%e, "discarding malformed B2xx control response");
                    continue;
                }
            };
            if self.queue_tx_event(&packet) {
                continue;
            }
            if packet.info.sid != Some(resp_sid)
                || packet.info.packet_type != PacketType::Context
                || packet.payload.len() != 2
            {
                warn!(sid=?packet.info.sid, want = resp_sid, "discarding unrelated B2xx control response");
                continue;
            }
            let got = packet.info.packet_count;
            if sequence_relation(seq, got) != SequenceRelation::Match {
                // A late response is safe to discard. A response for a queued
                // future command proves this acknowledgement was lost; clear
                // the queue because USB has already consumed the future one.
                if sequence_relation(seq, got) == SequenceRelation::Future {
                    self.flush();
                    return Err(Error::Transport(format!(
                        "B2xx control response jumped from seq {seq} to {got}; response \
                         state resynchronized"
                    )));
                }
                warn!(
                    expected = seq,
                    got, "discarding stale B2xx control response"
                );
                continue;
            }
            return Ok(readback
                .then(|| (u64::from(packet.payload[0]) << 32) | u64::from(packet.payload[1])));
        }
    }

    fn drain_for_read(&mut self) -> Result<u64> {
        loop {
            let last = self.outstanding.len() == 1;
            if let Some(value) = self.wait_one(last)? {
                return Ok(value);
            }
        }
    }
}

impl RegisterIface for CtrlPort {
    fn poke32(&self, addr: u32, data: u32) -> Result<()> {
        let mut t = self.locked();
        t.send(self, addr / 4, data)?;
        if t.outstanding.len() >= MAX_OUTSTANDING {
            t.wait_one(false).map_err(|e| self.attribute(e))?;
        }
        Ok(())
    }

    fn peek32(&self, addr: u32) -> Result<u32> {
        let value = self.peek64(addr & !7)?;
        Ok(if (addr / 4) & 1 == 0 {
            value as u32
        } else {
            (value >> 32) as u32
        })
    }

    fn peek64(&self, addr: u32) -> Result<u64> {
        let mut t = self.locked();
        t.send(self, self.sr_readback, addr / 8)?;
        t.drain_for_read().map_err(|e| self.attribute(e))
    }
}

/// The local core, so that `RadioCtrl` itself still reads the registers
/// negotiation and `ruhd doctor` ask for.
impl RegisterIface for RadioCtrl {
    fn poke32(&self, addr: u32, data: u32) -> Result<()> {
        self.local().poke32(addr, data)
    }

    fn peek32(&self, addr: u32) -> Result<u32> {
        self.local().peek32(addr)
    }

    fn peek64(&self, addr: u32) -> Result<u64> {
        self.local().peek64(addr)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn constants_match_b200_impl() {
        assert_eq!((CTRL_SEND_INTERFACE, CTRL_SEND_ENDPOINT), (3, 4));
        assert_eq!((CTRL_RECV_INTERFACE, CTRL_RECV_ENDPOINT), (4, 0x88));
        assert_eq!((LOCAL_CTRL_SID, LOCAL_RESP_SID), (0x40, 0x0040_0000));
        assert_eq!(FPGA_COMPAT_ADDR, 0);
    }

    #[test]
    fn every_supported_major_supplies_the_registers_the_gate_needs() {
        // The gate reaches for radio chain 0's registers and stream IDs
        // through the shim. A major that could not answer would panic at the
        // bench rather than here.
        for compat in FpgaCompat::WINDOW {
            let shim = compat.shim();
            assert_eq!(shim.radio_sids(0), Some((0x10, 0x0010_0000)), "{compat}");
            assert_eq!(shim.radio_regs().sr_test, 84, "{compat}");
            assert_eq!(shim.radio_regs().rb32_test, 0, "{compat}");
            assert_eq!(shim.radio_regs().sr_readback, 32, "{compat}");
        }
    }

    #[test]
    fn response_layout_is_high_word_first() {
        let info = IfPacketInfo {
            link_type: LinkType::Chdr,
            packet_type: PacketType::Context,
            packet_count: 9,
            sid: Some(LOCAL_RESP_SID),
            payload_bytes: 8,
            ..Default::default()
        };
        let bytes = chdr::pack(&info, &[0xace0_ba5e, 0x0007_0001], Endian::Little).unwrap();
        let packet = chdr::unpack(&bytes, LinkType::Chdr, Endian::Little).unwrap();
        let value = (u64::from(packet.payload[0]) << 32) | u64::from(packet.payload[1]);
        assert_eq!(value, 0xace0_ba5e_0007_0001);
    }

    #[test]
    fn sequence_matching_handles_wrap_and_resync_direction() {
        assert_eq!(sequence_relation(17, 17), SequenceRelation::Match);
        assert_eq!(sequence_relation(17, 16), SequenceRelation::Stale);
        assert_eq!(sequence_relation(17, 18), SequenceRelation::Future);
        assert_eq!(sequence_relation(0xfff, 0), SequenceRelation::Future);
        assert_eq!(sequence_relation(0, 0xfff), SequenceRelation::Stale);
    }

    #[test]
    fn a_pending_command_remembers_which_core_owes_the_reply() {
        // Two cores are in flight on one pipe, so a reply is only matched when
        // *both* the sequence number and the response stream ID line up.
        // Dropping the sid from the queue would make a radio reply satisfy a
        // local read, which is a wrong value rather than a timeout.
        let local = Pending {
            seq: 4,
            resp_sid: LOCAL_RESP_SID,
        };
        let radio = Pending {
            seq: 4,
            resp_sid: 0x0010_0000,
        };
        assert_eq!(
            sequence_relation(local.seq, radio.seq),
            SequenceRelation::Match
        );
        assert_ne!(local.resp_sid, radio.resp_sid);
    }
}

#[cfg(test)]
mod tx_event_tests {
    use super::*;
    fn packet(sid: u32, payload: &[u32]) -> chdr::IfPacket {
        let info = IfPacketInfo {
            link_type: LinkType::Chdr,
            packet_type: PacketType::Context,
            sid: Some(sid),
            tsf: Some(1234),
            payload_bytes: payload.len() * 4,
            ..Default::default()
        };
        chdr::unpack(
            &chdr::pack(&info, payload, Endian::Little).unwrap(),
            LinkType::Chdr,
            Endian::Little,
        )
        .unwrap()
    }
    #[test]
    fn tx_messages_are_distinct_from_register_replies_and_keep_time_and_channel() {
        for (code, expected) in [
            (1, AsyncEvent::BurstAck),
            (2, AsyncEvent::Underflow),
            (4, AsyncEvent::SeqError),
            (8, AsyncEvent::TimeError),
            (16, AsyncEvent::UnderflowInPacket),
            (32, AsyncEvent::SeqErrorInBurst),
        ] {
            let event = tx_event(&packet(0x0060_0000, &[0x100 | code])).unwrap();
            assert_eq!(event.channel, 1);
            assert_eq!(event.tsf, Some(1234));
            assert_eq!(event.event, expected);
        }
        assert!(tx_event(&packet(LOCAL_RESP_SID, &[1, 2])).is_none());
        assert!(tx_event(&packet(0x0050_0000, &[])).is_none());
        assert!(tx_event(&packet(0x0050_0000, &[0])).is_none());
        assert_eq!(
            tx_event(&packet(0x0050_0000, &[64, 1, 2])).unwrap().event,
            AsyncEvent::UserPayload([1, 2, 0, 0])
        );
    }
}
