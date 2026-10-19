//! Receiving samples from a B2xx.
//!
//! Ports the receive half of `b200_io_impl.cpp` (`get_rx_stream`,
//! `handle_overflow`) and the single-channel paths of
//! `super_recv_packet_handler.hpp`.
//!
//! # The data path
//!
//! ```text
//!   AD936x -> rx_dsp_core_3000 -> rx_vita_core_3000 -> FX3 -> bulk IN 0x86 -> here
//!             (CORDIC, decim)    (CHDR framing, SID)
//! ```
//!
//! Samples arrive on their own USB interface, separate from the settings bus,
//! as one CHDR packet per bulk transfer: a header word, the stream ID, a 64-bit
//! timestamp, then `spp` samples of `sc16`. Inline messages (overflow, late
//! command) arrive on the same endpoint as context packets.
//!
//! # Keeping the pipe full
//!
//! At 50 MS/s an `sc16` stream is 200 MB/s, and a single outstanding USB
//! transfer cannot keep up with that: the host would be idle between each
//! completion and the next submission. So [`B200RxStreamer`] keeps
//! `num_recv_frames` transfers queued with the kernel at all times — 128 by
//! default, eight times UHD's 16; see [`DEFAULT_NUM_RECV_FRAMES`]. A packet's buffer goes back in the queue as soon as its
//! last sample has been copied out.
//!
//! # A difference from libusb
//!
//! nusb insists that an IN transfer request a whole number of max-size USB
//! packets; libusb does not, and UHD relies on that, asking for 8176 bytes at a
//! time. Here the request is rounded up to the next multiple (8192 on
//! SuperSpeed). Nothing changes on the wire: the FPGA's packet is still 8176
//! bytes, which is deliberately not a multiple of the USB packet size, so it
//! ends with a short USB packet and the transfer completes there.

use std::fmt;
use std::ops::Range;
use std::time::{Duration, Instant};

use chdr::{Endian, LinkType, PacketType};
use nusb::MaybeFuture;
use nusb::transfer::{Buffer, Bulk, In};
use tracing::{debug, trace, warn};
use uhd_core::metadata::{RxError, RxMetadata};
use uhd_core::stream::{RawRxStreamer, StreamArgs, StreamCmd};
use uhd_core::{Error, Result, SampleFormat, TimeSpec};
use uhd_transport::RegisterIface;

use crate::atr::{GpioAtr, Side};
use crate::rx_dsp::RxDsp;
use crate::rx_framer::RxFramer;
use crate::session::B200Session;

/// USB interface carrying received samples. `B200_USB_DATA_RECV_INTERFACE`.
pub const DATA_RECV_INTERFACE: u8 = 2;
/// Its bulk IN endpoint: endpoint 6, with the IN bit set.
/// `B200_USB_DATA_RECV_ENDPOINT`.
pub const DATA_RECV_ENDPOINT: u8 = 0x86;
/// Stream IDs the framers stamp on received data, per radio.
/// `B200_RX_DATA0_SID` and `B200_RX_DATA1_SID`.
pub const RX_DATA_SIDS: [u32; 2] = [0xa0, 0xb0];
/// `SR_RX_FMT`, the wire-format select. A byte address, `TOREG(136)`.
pub const SR_RX_FMT: u32 = 136 * 4;

/// Default bytes per received packet. `B200_USB_DATA_DEFAULT_FRAME_SIZE`; the
/// reference notes it must not be a multiple of 512.
pub const DEFAULT_RECV_FRAME_SIZE: usize = 8176;
/// Smallest frame size accepted. `B200_USB_DATA_MIN_RECV_FRAME_SIZE`.
pub const MIN_RECV_FRAME_SIZE: usize = 40;
/// Largest frame size accepted. `B200_USB_DATA_MAX_RECV_FRAME_SIZE`.
pub const MAX_RECV_FRAME_SIZE: usize = 16360;
/// Transfers kept queued by default.
///
/// UHD queues 16, which at 8 KiB each is about a millisecond of samples at
/// 30 MS/s: any scheduling delay longer than that and the FPGA's buffer
/// overflows. On a B205mini over USB 3, 16 overflowed repeatedly at 61.44 MS/s
/// where 128 (1 MiB in all) ran clean or nearly so, so this queues 128.
pub const DEFAULT_NUM_RECV_FRAMES: usize = 128;
/// The framer's largest packet at full rate, in samples.
pub const MAX_SPP: usize = 4092;

/// Header bytes on a received data packet: CHDR word, SID, 64-bit time.
///
/// The reference computes this as the largest VRT header less the class ID and
/// integer timestamp, neither of which a B2xx sends.
const HEADER_BYTES: usize = 16;
/// Bytes per `sc16` sample on the wire.
const WIRE_BYTES: usize = 4;
/// How long the flush waits for another stale packet before deciding there
/// are none. The reference's overflow handler uses the same 1 ms.
const FLUSH_TIMEOUT: Duration = Duration::from_millis(1);
/// Upper bound on packets one flush will discard, so a stream that is somehow
/// still running cannot hold the flush forever.
const FLUSH_LIMIT: usize = 100_000;

/// Which RX port a radio listens on.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Hash)]
pub enum RxAntenna {
    /// The receive-only port. UHD's default.
    #[default]
    Rx2,
    /// The shared transmit/receive port.
    TxRx,
}

impl RxAntenna {
    /// UHD's name for the port.
    #[must_use]
    pub fn name(self) -> &'static str {
        match self {
            Self::Rx2 => "RX2",
            Self::TxRx => "TX/RX",
        }
    }

    /// Parses UHD's name for a port.
    ///
    /// # Errors
    ///
    /// [`Error::InvalidArg`] for anything but `RX2` or `TX/RX`.
    pub fn parse(name: &str) -> Result<Self> {
        match name.to_ascii_uppercase().as_str() {
            "RX2" => Ok(Self::Rx2),
            "TX/RX" | "TXRX" => Ok(Self::TxRx),
            _ => Err(Error::invalid_arg(format!(
                "unknown rx antenna `{name}`; a b2xx has RX2 and TX/RX"
            ))),
        }
    }
}

impl fmt::Display for RxAntenna {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(self.name())
    }
}

/// Radio settings that belong to a receive stream rather than to the codec.
#[derive(Clone, Debug, PartialEq)]
#[non_exhaustive]
pub struct RxOptions {
    /// Host sample rate, in samples per second. Coerced to the master clock
    /// rate over an integer decimation.
    pub rate: f64,
    /// Frequency shift applied by the DSP chain's CORDIC, in Hz.
    pub dsp_freq: f64,
    /// Which port to receive on.
    pub antenna: RxAntenna,
}

impl RxOptions {
    /// Receive at `rate` on RX2 with no DSP shift.
    #[must_use]
    pub fn new(rate: f64) -> Self {
        Self {
            rate,
            dsp_freq: 0.0,
            antenna: RxAntenna::default(),
        }
    }

    /// Selects the receive port.
    #[must_use]
    pub fn with_antenna(mut self, antenna: RxAntenna) -> Self {
        self.antenna = antenna;
        self
    }

    /// Sets the CORDIC shift.
    #[must_use]
    pub fn with_dsp_freq(mut self, freq: f64) -> Self {
        self.dsp_freq = freq;
        self
    }
}

/// A packet whose samples are being handed out, possibly over several calls.
#[derive(Debug)]
struct Packet {
    buf: Buffer,
    payload: Range<usize>,
    /// Samples already copied out.
    offset: usize,
    /// Samples in the packet.
    nsamps: usize,
    tsf: Option<u64>,
    sob: bool,
    eob: bool,
}

/// What the next completed transfer turned out to be.
#[derive(Debug)]
enum Next {
    /// Samples, in sequence.
    Data(Packet),
    /// Samples, after one or more packets went missing.
    Gap(Packet),
    /// An inline message from the framer.
    Inline { error: RxError, tsf: Option<u64> },
    /// Something that could not be parsed.
    Bad,
    /// Nothing arrived before the deadline.
    Timeout,
}

/// A single-channel B2xx receive stream.
///
/// Obtained from [`B200Session::rx_stream`]. Dropping it stops the framer,
/// cancels the queued transfers and returns the front end to its idle state.
#[derive(Debug)]
pub struct B200RxStreamer {
    ep: nusb::Endpoint<Bulk, In>,
    framer: RxFramer,
    dsp: RxDsp,
    atr: GpioAtr,
    side: Side,
    antenna: RxAntenna,
    sid: u32,
    tick_rate: f64,
    ticks_per_sample: u64,
    host_format: SampleFormat,
    scale: f64,
    spp: usize,
    current: Option<Packet>,
    expected_seq: Option<u16>,
    /// Where the last delivered packet ended, in ticks.
    next_tsf: Option<u64>,
    /// An event found while filling a buffer that already had samples in it,
    /// held back for the next call.
    queued: Option<RxMetadata>,
}

/// The frame size UHD would use for a requested one.
///
/// `b200_impl`'s coercion, unchanged: clamped to the legal range, and, if it
/// is not a multiple of 8 or is a multiple of the USB packet size, moved to a
/// multiple of 8 that is not, preferring one that is also 24-byte aligned past
/// the header so `sc8`, `sc12` and `sc16` all fill whole 64-bit words.
#[must_use]
pub fn coerce_recv_frame_size(requested: usize, max_transfer: usize) -> usize {
    if requested < MIN_RECV_FRAME_SIZE {
        return MIN_RECV_FRAME_SIZE;
    }
    if requested > MAX_RECV_FRAME_SIZE {
        return MAX_RECV_FRAME_SIZE;
    }
    let mut size = requested;
    if size.is_multiple_of(max_transfer) || !size.is_multiple_of(8) {
        size += 8 - (size % 8);
        if size.is_multiple_of(max_transfer) {
            size = ((size - 16) / 24) * 24 + 16;
        }
    }
    size
}

/// The CORDIC-free part of opening a stream, checked before anything is
/// written to the device.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
struct Shape {
    radio: usize,
    host_format: SampleFormat,
    frame_size: usize,
    num_frames: usize,
    spp: usize,
}

fn shape(args: &StreamArgs, max_transfer: usize) -> Result<Shape> {
    let radio = match args.channels.as_slice() {
        [chan] => *chan,
        [] => 0,
        more => {
            return Err(Error::not_supported(format!(
                "multi-channel b2xx receive is not implemented yet; asked for channels {more:?}"
            )));
        }
    };
    if args.wire_format != SampleFormat::Sc16 {
        return Err(Error::not_supported(format!(
            "b2xx receive supports only the sc16 wire format so far, not {}",
            args.wire_format.name()
        )));
    }
    let host_format = args.host_format;
    if !matches!(
        host_format,
        SampleFormat::Sc16 | SampleFormat::Fc32 | SampleFormat::Fc64
    ) {
        return Err(Error::not_supported(format!(
            "b2xx receive cannot convert sc16 to a {} host buffer",
            host_format.name()
        )));
    }

    let requested: usize = args
        .args
        .get_parsed("recv_frame_size")?
        .unwrap_or(DEFAULT_RECV_FRAME_SIZE);
    let frame_size = coerce_recv_frame_size(requested, max_transfer);
    if frame_size != requested {
        warn!(
            requested,
            frame_size,
            max_transfer,
            "recv_frame_size must be a multiple of 8 bytes, not a multiple of the usb \
             packet size, and within {MIN_RECV_FRAME_SIZE}..={MAX_RECV_FRAME_SIZE}; coerced"
        );
    }

    let num_frames: usize = args
        .args
        .get_parsed("num_recv_frames")?
        .unwrap_or(DEFAULT_NUM_RECV_FRAMES);
    if num_frames < 2 {
        return Err(Error::invalid_arg(format!(
            "num_recv_frames={num_frames}: at least 2 transfers must be queued"
        )));
    }

    let fits = (frame_size - HEADER_BYTES) / WIRE_BYTES;
    let spp = match args.args.get_parsed::<f64>("spp")? {
        Some(spp) if spp.is_finite() && spp >= 1.0 => (spp as usize).min(fits),
        Some(spp) => return Err(Error::invalid_arg(format!("spp={spp}"))),
        None => fits,
    }
    .min(MAX_SPP);

    Ok(Shape {
        radio,
        host_format,
        frame_size,
        num_frames,
        spp,
    })
}

/// The ATR side radio `radio` drives. See `b200_impl`'s `_fe1`/`_fe2`.
pub(crate) fn side_for(radio: usize, swap_atr: bool) -> Side {
    match (radio, swap_atr) {
        (0, false) | (1, true) => Side::Fe1,
        _ => Side::Fe2,
    }
}

/// Opens a receive stream. See [`B200Session::rx_stream`].
pub(crate) fn open(
    session: &B200Session,
    options: &RxOptions,
    args: &StreamArgs,
) -> Result<B200RxStreamer> {
    let tick_rate = session.codec().clock_rate();
    if tick_rate <= 0.0 {
        return Err(Error::Device(
            "the ad936x is not clocking; call B200Session::init_radio before opening a stream"
                .into(),
        ));
    }

    let device = session.iface().device();
    let ep = device
        .claim_interface(DATA_RECV_INTERFACE)
        .wait()
        .map_err(|e| {
            Error::Usb(format!(
                "cannot claim b2xx rx data interface {DATA_RECV_INTERFACE}: {e}"
            ))
        })?
        .endpoint::<Bulk, In>(DATA_RECV_ENDPOINT)
        .map_err(|e| Error::Usb(format!("cannot open b2xx rx data endpoint: {e}")))?;
    let max_transfer = ep.max_packet_size();

    let shape = shape(args, max_transfer)?;
    let chains = session.num_radio_chains()?;
    if shape.radio >= chains {
        return Err(Error::invalid_arg(format!(
            "channel {} does not exist; this fpga build has {chains} radio chain(s)",
            shape.radio
        )));
    }
    let port = session.ctrl().radio(shape.radio)?;
    let sid = RX_DATA_SIDS[shape.radio];

    // The order is get_rx_stream's: format, DSP, framer, then the enables.
    port.poke32(SR_RX_FMT, 0)?;

    let mut dsp = RxDsp::new(Box::new(port.clone()));
    dsp.set_tick_rate(tick_rate)?;
    dsp.set_mux(0)?;
    dsp.set_freq(options.dsp_freq)?;
    let rate = dsp.set_host_rate(options.rate)?;
    dsp.setup_sc16()?;
    if (rate - options.rate).abs() > 1.0 {
        warn!(
            requested = options.rate,
            actual = rate,
            tick_rate,
            "the sample rate was coerced to the nearest master clock rate / decimation"
        );
    }
    if let Some(message) = bandwidth_warning(rate) {
        warn!("{message}");
    }

    // A previous process may have left the framer running; stop it before
    // the stale packets are flushed.
    let mut framer = RxFramer::new(Box::new(port.clone()));
    framer.issue_stream_cmd(&StreamCmd::stop(), tick_rate)?;
    framer.set_nsamps_per_packet(shape.spp)?;
    framer.set_sid(sid)?;

    let side = side_for(shape.radio, session.misc().swap_atr());
    session
        .codec()
        .set_active_chains(false, false, side == Side::Fe1, side == Side::Fe2)?;
    let atr = GpioAtr::radio(Box::new(port));
    atr.enable_atr()?;
    atr.set_b2xx_states(side, true, false, options.antenna == RxAntenna::Rx2)?;

    let transfer_len = shape.frame_size.div_ceil(max_transfer) * max_transfer;
    let mut streamer = B200RxStreamer {
        ep,
        framer,
        dsp,
        atr,
        side,
        antenna: options.antenna,
        sid,
        tick_rate,
        ticks_per_sample: u64::from(1u32.max((tick_rate / rate).round() as u32)),
        host_format: shape.host_format,
        scale: 0.0,
        spp: shape.spp,
        current: None,
        expected_seq: None,
        next_tsf: None,
        queued: None,
    };
    streamer.scale = streamer.dsp.scaling_adjustment();
    for _ in 0..shape.num_frames {
        let mut buf = streamer.ep.allocate(transfer_len);
        buf.set_requested_len(transfer_len);
        streamer.ep.submit(buf);
    }
    let stale = streamer.flush();
    debug!(
        radio = shape.radio,
        sid,
        tick_rate,
        rate,
        decim = streamer.dsp.decimation(),
        spp = shape.spp,
        transfer_len,
        num_frames = shape.num_frames,
        stale,
        antenna = %options.antenna,
        "b2xx rx stream open"
    );
    Ok(streamer)
}

impl B200RxStreamer {
    /// The host sample rate the DSP chain is producing.
    #[must_use]
    pub fn sample_rate(&self) -> f64 {
        self.dsp.host_rate()
    }

    /// The master clock rate timestamps are counted in.
    #[must_use]
    pub fn tick_rate(&self) -> f64 {
        self.tick_rate
    }

    /// The CORDIC shift actually programmed.
    #[must_use]
    pub fn dsp_freq(&self) -> f64 {
        self.dsp.freq()
    }

    /// The port this stream is receiving on.
    #[must_use]
    pub fn antenna(&self) -> RxAntenna {
        self.antenna
    }

    /// Samples per packet.
    #[must_use]
    pub fn samples_per_packet(&self) -> usize {
        self.spp
    }

    /// Discards everything the endpoint has delivered or will deliver within
    /// the flush timeout. Returns how many packets went.
    fn flush(&mut self) -> usize {
        if let Some(packet) = self.current.take() {
            self.ep.submit(packet.buf);
        }
        let mut count = 0;
        while count < FLUSH_LIMIT {
            let Some(completion) = self.ep.wait_next_complete(FLUSH_TIMEOUT) else {
                break;
            };
            self.ep.submit(completion.buffer);
            count += 1;
        }
        self.expected_seq = None;
        self.next_tsf = None;
        count
    }

    /// `handle_overflow` for a single-channel stream: discard what is
    /// buffered, then restart the framer if it was streaming continuously.
    fn recover_overflow(&mut self) -> Result<()> {
        let flushed = self.flush();
        trace!(flushed, "flushed after overflow");
        self.framer.handle_overflow(self.tick_rate)
    }

    fn time_of(&self, tsf: u64) -> TimeSpec {
        TimeSpec::from_ticks(tsf as i64, self.tick_rate)
    }

    /// Waits for the next completed transfer and classifies it.
    fn next_packet(&mut self, deadline: Instant) -> Result<Next> {
        loop {
            let wait = deadline.saturating_duration_since(Instant::now());
            let Some(completion) = self.ep.wait_next_complete(wait) else {
                return Ok(Next::Timeout);
            };
            if let Err(e) = completion.status {
                // Put the buffer back so the queue stays its full depth if the
                // caller decides to carry on.
                self.ep.submit(completion.buffer);
                return Err(Error::Transport(format!(
                    "b2xx rx data transfer failed: {e}"
                )));
            }
            let buf = completion.buffer;
            let header = match chdr::parse_header(&buf, LinkType::Chdr, Endian::Little) {
                Ok(header) => header,
                Err(e) => {
                    warn!(%e, len = buf.len(), "discarding an unparseable b2xx rx packet");
                    self.ep.submit(buf);
                    return Ok(Next::Bad);
                }
            };
            let info = &header.info;
            if info.sid != Some(self.sid) {
                trace!(sid = ?info.sid, want = self.sid, "skipping a packet for another stream");
                self.ep.submit(buf);
                continue;
            }
            let payload = header.payload_range();

            if info.packet_type != PacketType::Data {
                let code = context_code(&buf[payload]);
                let tsf = info.tsf;
                self.ep.submit(buf);
                match inline_error(code) {
                    Some(error) => return Ok(Next::Inline { error, tsf }),
                    None => {
                        debug!(code, "ignoring a b2xx rx inline message with no error");
                        continue;
                    }
                }
            }

            let count = info.packet_count;
            let gap = self.expected_seq.is_some_and(|want| want != count);
            if gap {
                trace!(expected = ?self.expected_seq, got = count, "b2xx rx sequence gap");
            }
            self.expected_seq = Some((count + 1) & 0xfff);
            if payload.len() % WIRE_BYTES != 0 {
                warn!(
                    bytes = payload.len(),
                    "discarding a b2xx rx packet that is not whole sc16 samples"
                );
                self.ep.submit(buf);
                return Ok(Next::Bad);
            }
            let packet = Packet {
                nsamps: payload.len() / WIRE_BYTES,
                payload,
                offset: 0,
                tsf: info.tsf,
                sob: info.sob,
                eob: info.eob,
                buf,
            };
            return Ok(if gap {
                Next::Gap(packet)
            } else {
                Next::Data(packet)
            });
        }
    }

    /// Hands an event to the caller now if nothing has been copied yet, or
    /// holds it for the next call if something has.
    fn report(&mut self, accum: usize, event: RxMetadata, md: &mut RxMetadata) -> usize {
        if accum == 0 {
            *md = event;
        } else {
            self.queued = Some(event);
        }
        accum
    }
}

impl RawRxStreamer for B200RxStreamer {
    fn num_channels(&self) -> usize {
        1
    }

    fn max_num_samps(&self) -> usize {
        self.spp
    }

    fn host_format(&self) -> SampleFormat {
        self.host_format
    }

    fn wire_format(&self) -> SampleFormat {
        SampleFormat::Sc16
    }

    fn issue_stream_cmd(&mut self, cmd: &StreamCmd) -> Result<()> {
        self.framer.issue_stream_cmd(cmd, self.tick_rate)
    }

    fn recv_raw(
        &mut self,
        buffs: &mut [&mut [u8]],
        timeout: Duration,
        md: &mut RxMetadata,
    ) -> Result<usize> {
        *md = RxMetadata::default();
        let [out] = buffs else {
            return Err(Error::Buffer(format!(
                "a b2xx rx stream has 1 channel but {} buffers were supplied",
                buffs.len()
            )));
        };
        if let Some(event) = self.queued.take() {
            *md = event;
            return Ok(0);
        }
        let host_bytes = self.host_format.bits_per_sample() / 8;
        let want = out.len() / host_bytes;
        if want == 0 {
            return Ok(0);
        }

        let deadline = Instant::now() + timeout;
        let mut accum = 0;
        while accum < want {
            if self.current.is_none() {
                match self.next_packet(deadline)? {
                    Next::Data(packet) => self.current = Some(packet),
                    Next::Timeout => {
                        if accum == 0 {
                            md.error = Some(RxError::Timeout);
                        }
                        return Ok(accum);
                    }
                    Next::Gap(packet) => {
                        // Keep the packet that revealed the gap: its samples
                        // are good and are delivered on the next call.
                        let event = RxMetadata {
                            time_spec: self.next_tsf.or(packet.tsf).map(|t| self.time_of(t)),
                            out_of_sequence: true,
                            error: Some(RxError::Overflow),
                            ..RxMetadata::default()
                        };
                        self.current = Some(packet);
                        return Ok(self.report(accum, event, md));
                    }
                    Next::Inline { error, tsf } => {
                        if error == RxError::Overflow {
                            self.recover_overflow()?;
                        }
                        let event = RxMetadata {
                            time_spec: tsf.map(|t| self.time_of(t)),
                            error: Some(error),
                            ..RxMetadata::default()
                        };
                        return Ok(self.report(accum, event, md));
                    }
                    Next::Bad => {
                        let event = RxMetadata {
                            error: Some(RxError::BadPacket),
                            ..RxMetadata::default()
                        };
                        return Ok(self.report(accum, event, md));
                    }
                }
            }

            let Some(packet) = self.current.as_mut() else {
                continue;
            };
            let n = (packet.nsamps - packet.offset).min(want - accum);
            if accum == 0 {
                md.time_spec = packet.tsf.map(|t| {
                    TimeSpec::from_ticks(
                        (t + packet.offset as u64 * self.ticks_per_sample) as i64,
                        self.tick_rate,
                    )
                });
                md.start_of_burst = packet.sob && packet.offset == 0;
                md.fragment_offset = packet.offset;
            }
            let start = packet.payload.start + packet.offset * WIRE_BYTES;
            convert(
                &packet.buf[start..start + n * WIRE_BYTES],
                &mut out[accum * host_bytes..(accum + n) * host_bytes],
                self.host_format,
                self.scale,
            );
            packet.offset += n;
            accum += n;

            if packet.offset == packet.nsamps {
                let eob = packet.eob;
                if let Some(packet) = self.current.take() {
                    self.next_tsf = packet
                        .tsf
                        .map(|t| t + packet.nsamps as u64 * self.ticks_per_sample);
                    self.ep.submit(packet.buf);
                }
                if eob {
                    md.end_of_burst = true;
                    break;
                }
            }
        }
        md.more_fragments = self.current.as_ref().is_some_and(|p| p.offset > 0);
        Ok(accum)
    }
}

impl Drop for B200RxStreamer {
    fn drop(&mut self) {
        if let Err(e) = self
            .framer
            .issue_stream_cmd(&StreamCmd::stop(), self.tick_rate)
        {
            debug!(%e, "could not stop the b2xx rx framer while closing the stream");
        }
        self.ep.cancel_all();
        while self.ep.pending() > 0 {
            if self.ep.wait_next_complete(Duration::from_secs(1)).is_none() {
                warn!("b2xx rx transfers did not return after cancellation");
                break;
            }
        }
        if let Err(e) =
            self.atr
                .set_b2xx_states(self.side, false, false, self.antenna == RxAntenna::Rx2)
        {
            debug!(%e, "could not idle the b2xx front end while closing the stream");
        }
    }
}

/// The error code in an inline message's first payload word.
///
/// `get_context_code`: the code is in the low byte, and the reference mirrors
/// the word's bytes before masking so it does not have to know the link's byte
/// order. An empty payload is reported as a bad packet.
fn context_code(payload: &[u8]) -> u32 {
    match payload.get(..4) {
        Some(b) => {
            let word = u32::from_le_bytes([b[0], b[1], b[2], b[3]]);
            (word | word.swap_bytes()) & 0xff
        }
        None => 0xf,
    }
}

/// The metadata error for an inline message code, from `rx_metadata_t`.
fn inline_error(code: u32) -> Option<RxError> {
    match code {
        0x1 => Some(RxError::Timeout),
        0x2 => Some(RxError::LateCommand),
        0x4 => Some(RxError::BrokenChain),
        0x8 => Some(RxError::Overflow),
        0xc => Some(RxError::Alignment),
        0xf => Some(RxError::BadPacket),
        _ => None,
    }
}

/// Converts `sc16` wire samples to the host format.
///
/// Each wire sample is a little-endian 32-bit word with I in the high half and
/// Q in the low half (`sc16_item32_le`). Host buffers hold native-endian
/// `Complex<T>`, real part first. Floats are scaled by `scale`, which is
/// [`RxDsp::scaling_adjustment`].
fn convert(wire: &[u8], out: &mut [u8], host: SampleFormat, scale: f64) {
    let samples = wire.chunks_exact(WIRE_BYTES).map(|b| {
        let word = u32::from_le_bytes([b[0], b[1], b[2], b[3]]);
        ((word >> 16) as i16, word as i16)
    });
    match host {
        SampleFormat::Sc16 => {
            for ((i, q), o) in samples.zip(out.chunks_exact_mut(4)) {
                o[..2].copy_from_slice(&i.to_ne_bytes());
                o[2..].copy_from_slice(&q.to_ne_bytes());
            }
        }
        SampleFormat::Fc32 => {
            let scale = scale as f32;
            for ((i, q), o) in samples.zip(out.chunks_exact_mut(8)) {
                o[..4].copy_from_slice(&(f32::from(i) * scale).to_ne_bytes());
                o[4..].copy_from_slice(&(f32::from(q) * scale).to_ne_bytes());
            }
        }
        SampleFormat::Fc64 => {
            for ((i, q), o) in samples.zip(out.chunks_exact_mut(16)) {
                o[..8].copy_from_slice(&(f64::from(i) * scale).to_ne_bytes());
                o[8..].copy_from_slice(&(f64::from(q) * scale).to_ne_bytes());
            }
        }
        // `shape` refuses every other host format before a stream exists.
        _ => {}
    }
}

/// `ad936x_manager::check_bandwidth`'s warning for a sample rate the codec's
/// analog filters cannot match, which `b200_impl::update_rx_samp_rate` gives
/// every time the RX rate is set.
///
/// UHD stays quiet when the user has set a bandwidth explicitly. This driver
/// does not track that, so it always warns; the warning is still accurate,
/// since the filters cannot go beyond these limits whatever is asked of them.
fn bandwidth_warning(rate: f64) -> Option<String> {
    if rate > ad936x::MAX_BW {
        Some(format!(
            "Selected Rx sample rate ({} MHz) is greater than analog frontend filter \
             bandwidth ({} MHz).",
            rate / 1e6,
            ad936x::MAX_BW / 1e6
        ))
    } else if rate < ad936x::MIN_BW {
        Some(format!(
            "Selected Rx sample rate ({} MHz) is less than analog frontend filter \
             bandwidth ({} MHz).",
            rate / 1e6,
            ad936x::MIN_BW / 1e6
        ))
    } else {
        None
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn bandwidth_warning_matches_check_bandwidth() {
        assert_eq!(
            bandwidth_warning(60e6).as_deref(),
            Some(
                "Selected Rx sample rate (60 MHz) is greater than analog frontend filter \
                 bandwidth (56 MHz)."
            )
        );
        assert_eq!(
            bandwidth_warning(100e3).as_deref(),
            Some(
                "Selected Rx sample rate (0.1 MHz) is less than analog frontend filter \
                 bandwidth (0.2 MHz)."
            )
        );
        assert_eq!(bandwidth_warning(56e6), None);
        assert_eq!(bandwidth_warning(200e3), None);
        assert_eq!(bandwidth_warning(10e6), None);
    }

    #[test]
    fn constants_match_b200_impl() {
        assert_eq!((DATA_RECV_INTERFACE, DATA_RECV_ENDPOINT), (2, 0x86));
        assert_eq!(RX_DATA_SIDS, [0xa0, 0xb0]);
        assert_eq!(SR_RX_FMT, 544);
    }

    #[test]
    fn frame_size_coercion_matches_uhd() {
        assert_eq!(coerce_recv_frame_size(8176, 1024), 8176);
        assert_eq!(coerce_recv_frame_size(8, 1024), MIN_RECV_FRAME_SIZE);
        assert_eq!(coerce_recv_frame_size(1 << 20, 1024), MAX_RECV_FRAME_SIZE);
        // Not a multiple of 8: rounded up.
        assert_eq!(coerce_recv_frame_size(8170, 1024), 8176);
        // A multiple of the USB packet size: bumped, then 24-byte aligned.
        assert_eq!(coerce_recv_frame_size(8192, 1024), 8200);
        // 2044 -> 2048, a multiple of 1024 -> 24-byte aligned past the header.
        assert_eq!(coerce_recv_frame_size(2044, 1024), 2032);
        // Already a multiple of 8 and not of the packet size: unchanged.
        assert_eq!(coerce_recv_frame_size(1016, 1024), 1016);
        assert_eq!(coerce_recv_frame_size(8192, 512), 8200);
    }

    #[test]
    fn default_shape_is_uhds() {
        let shape = shape(&StreamArgs::new(), 1024).unwrap();
        assert_eq!(shape.radio, 0);
        assert_eq!(shape.frame_size, 8176);
        assert_eq!(shape.num_frames, DEFAULT_NUM_RECV_FRAMES);
        // (8176 - 16) / 4.
        assert_eq!(shape.spp, 2040);
    }

    #[test]
    fn spp_is_capped_by_the_frame_and_by_the_fpga() {
        let args = StreamArgs::new().with_arg("spp", "100");
        assert_eq!(shape(&args, 1024).unwrap().spp, 100);
        let args = StreamArgs::new().with_arg("spp", "100000");
        assert_eq!(shape(&args, 1024).unwrap().spp, 2040);
        let args = StreamArgs::new()
            .with_arg("recv_frame_size", "16360")
            .with_arg("spp", "100000");
        assert_eq!(shape(&args, 1024).unwrap().spp, 4086);
    }

    #[test]
    fn unsupported_shapes_are_refused() {
        let two = StreamArgs::new().with_channels([0, 1]);
        assert!(matches!(shape(&two, 1024), Err(Error::NotSupported(_))));
        let sc8 = StreamArgs::new().with_wire_format(SampleFormat::Sc8);
        assert!(matches!(shape(&sc8, 1024), Err(Error::NotSupported(_))));
        let one_frame = StreamArgs::new().with_arg("num_recv_frames", "1");
        assert!(matches!(shape(&one_frame, 1024), Err(Error::InvalidArg(_))));
    }

    #[test]
    fn radio_zero_is_fe1_unless_the_atrs_are_swapped() {
        assert_eq!(side_for(0, false), Side::Fe1);
        assert_eq!(side_for(1, false), Side::Fe2);
        assert_eq!(side_for(0, true), Side::Fe2);
        assert_eq!(side_for(1, true), Side::Fe1);
    }

    #[test]
    fn i_is_the_high_half_of_the_wire_word() {
        // I = 0x1234, Q = -2.
        let wire = ((0x1234u32 << 16) | 0xfffe).to_le_bytes();
        let mut out = [0u8; 4];
        convert(&wire, &mut out, SampleFormat::Sc16, 1.0);
        assert_eq!(i16::from_ne_bytes([out[0], out[1]]), 0x1234);
        assert_eq!(i16::from_ne_bytes([out[2], out[3]]), -2);

        let mut out = [0u8; 8];
        convert(&wire, &mut out, SampleFormat::Fc32, 0.5);
        assert_eq!(
            f32::from_ne_bytes(out[..4].try_into().unwrap()),
            0x1234 as f32 * 0.5
        );
        assert_eq!(f32::from_ne_bytes(out[4..].try_into().unwrap()), -1.0);
    }

    #[test]
    fn inline_codes_map_to_rx_errors() {
        assert_eq!(inline_error(0x8), Some(RxError::Overflow));
        assert_eq!(inline_error(0x2), Some(RxError::LateCommand));
        assert_eq!(inline_error(0), None);
        // Either byte order of the same word gives the same code.
        assert_eq!(context_code(&0x8u32.to_le_bytes()), 0x8);
        assert_eq!(context_code(&0x8u32.to_be_bytes()), 0x8);
        assert_eq!(context_code(&[]), 0xf);
    }

    #[test]
    fn antenna_names_round_trip() {
        for antenna in [RxAntenna::Rx2, RxAntenna::TxRx] {
            assert_eq!(RxAntenna::parse(antenna.name()).unwrap(), antenna);
        }
        assert!(RxAntenna::parse("RX1").is_err());
    }
}
