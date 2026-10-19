//! Single-channel B2xx transmission: sc16 CHDR packets on bulk OUT 0x02.
//!
//! A send accepts at most `max_num_samps()` samples. On a short send the EOB
//! flag is deferred; advance the buffer and timestamp before retrying. Empty
//! SOB calls cache metadata, and empty EOB calls send one zero sample, as UHD
//! does because this FPGA does not accept zero-length payloads.

use crate::{B200Session, GpioAtr, RadioCtrl, Side, TxDsp};
use nusb::MaybeFuture;
use nusb::transfer::{Buffer, Bulk, Out};
use std::collections::VecDeque;
use std::fmt;
use std::time::Duration;
use uhd_core::{AsyncMetadata, Error, RawTxStreamer, Result, SampleFormat, StreamArgs, TxMetadata};
use uhd_transport::{FrameId, LinkInfo, RegisterIface, SendLink};

/// USB interface carrying transmitted samples.
pub const DATA_SEND_INTERFACE: u8 = 1;
/// Bulk OUT endpoint on the transmit interface.
pub const DATA_SEND_ENDPOINT: u8 = 0x02;
/// Transmit stream IDs, one per radio.
pub const TX_DATA_SIDS: [u32; 2] = [0x50, 0x60];
/// Transmit wire-format register, `TOREG(138)`.
pub const SR_TX_FMT: u32 = 138 * 4;
/// Transmit deframer base, `TOREG(64)`.
pub const SR_TX_CTRL: u32 = 64 * 4;

/// Radio settings for a session-level transmit stream.
#[derive(Debug, Clone, Copy)]
pub struct TxOptions {
    /// Requested host sample rate.
    pub rate: f64,
    /// Physical upconverter shift in Hz (added to the RF LO).
    pub dsp_freq: f64,
}

impl TxOptions {
    /// Transmit at `rate` with no digital frequency shift.
    #[must_use]
    pub fn new(rate: f64) -> Self {
        Self {
            rate,
            dsp_freq: 0.0,
        }
    }
}

#[derive(Debug)]
struct UsbSend {
    ep: nusb::Endpoint<Bulk, Out>,
    free: Vec<Buffer>,
    current: Option<Buffer>,
    staging: Vec<u8>,
    lengths: VecDeque<usize>,
    size: usize,
    count: usize,
}

impl UsbSend {
    fn reap(&mut self, timeout: Duration) -> Result<bool> {
        let Some(c) = self.ep.wait_next_complete(timeout) else {
            return Ok(false);
        };
        let expected = self.lengths.pop_front().unwrap();
        let len = c.actual_len;
        let buffer = c
            .into_result()
            .map_err(|e| Error::Transport(format!("B2xx TX transfer failed: {e}")))?;
        if len != expected {
            return Err(Error::Transport(format!(
                "short TX transfer: {len} of {expected} bytes"
            )));
        }
        self.free.push(buffer);
        Ok(true)
    }
}

impl LinkInfo for UsbSend {
    fn num_frames(&self) -> usize {
        self.count
    }
    fn frame_size(&self) -> usize {
        self.size
    }
}

impl SendLink for UsbSend {
    fn get_send_buff(&mut self, timeout: Duration) -> Result<Option<FrameId>> {
        // Reap ready completions even while the free pool is nonempty, so a
        // USB failure is reported on the next send rather than a full ring later.
        while self.ep.pending() > 0 && self.reap(Duration::ZERO)? {}
        if self.free.is_empty() && !self.reap(timeout)? {
            return Ok(None);
        }
        let mut buffer = self.free.pop().unwrap();
        buffer.clear();
        self.current = Some(buffer);
        Ok(Some(FrameId(0)))
    }
    fn frame_bytes_mut(&mut self, _frame: FrameId) -> &mut [u8] {
        &mut self.staging
    }
    fn commit_send_buff(&mut self, _frame: FrameId, len: usize) -> Result<()> {
        let mut buffer = self.current.take().unwrap();
        buffer.extend_from_slice(&self.staging[..len]);
        self.lengths.push_back(len);
        self.ep.submit(buffer);
        Ok(())
    }
    fn release_send_buff(&mut self, _frame: FrameId) {
        self.free.push(self.current.take().unwrap());
    }
}

impl Drop for UsbSend {
    fn drop(&mut self) {
        self.ep.cancel_all();
        while self.ep.pending() > 0 {
            if self.ep.wait_next_complete(Duration::from_secs(1)).is_none() {
                break;
            }
        }
    }
}

#[derive(Debug)]
struct Hardware {
    ctrl: RadioCtrl,
    atr: GpioAtr,
    side: Side,
    radio: usize,
    policy: u32,
}

/// A single-channel transmitter. Drop cancels USB transfers and idles the front end.
/// Wait for a burst acknowledgement before dropping when delivery matters.
pub struct B200TxStreamer {
    link: Box<dyn SendLink>,
    hardware: Option<Hardware>,
    format: SampleFormat,
    spp: usize,
    sid: u32,
    seq: u16,
    tick_rate: f64,
    rate: f64,
    scale: f64,
    cached: Option<TxMetadata>,
}

impl fmt::Debug for B200TxStreamer {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("B200TxStreamer")
            .field("format", &self.format)
            .field("spp", &self.spp)
            .field("sid", &self.sid)
            .field("rate", &self.rate)
            .finish_non_exhaustive()
    }
}

#[derive(Debug)]
struct Shape {
    radio: usize,
    size: usize,
    frames: usize,
    spp: usize,
    policy: u32,
}

fn shape(args: &StreamArgs) -> Result<Shape> {
    let radio = match args.channels.as_slice() {
        [] => 0,
        [chan] => *chan,
        _ => return Err(Error::not_supported("multi-channel B2xx transmit")),
    };
    if args.wire_format != SampleFormat::Sc16
        || !matches!(
            args.host_format,
            SampleFormat::Sc16 | SampleFormat::Fc32 | SampleFormat::Fc64
        )
    {
        return Err(Error::not_supported(
            "B2xx TX requires sc16 wire and sc16/fc32/fc64 host samples",
        ));
    }
    let size = args
        .args
        .get_parsed::<usize>("send_frame_size")?
        .unwrap_or(8176);
    if !(24..=16360).contains(&size) || size % 8 != 0 {
        return Err(Error::invalid_arg(
            "send_frame_size must be a multiple of 8 in 24..=16360",
        ));
    }
    let frames = args
        .args
        .get_parsed::<usize>("num_send_frames")?
        .unwrap_or(128);
    if !(2..=4096).contains(&frames) {
        return Err(Error::invalid_arg("num_send_frames must be in 2..=4096"));
    }
    let fits = (size - 16) / 4;
    let spp = args.args.get_parsed::<usize>("spp")?.unwrap_or(fits);
    if spp == 0 || spp > fits {
        return Err(Error::invalid_arg(format!("spp must be in 1..={fits}")));
    }
    let policy = match args.args.get("underflow_policy").unwrap_or("next_packet") {
        "next_packet" => 2,
        "next_burst" => 4,
        "wait" => 1,
        _ => {
            return Err(Error::invalid_arg(
                "underflow_policy must be next_packet, next_burst or wait",
            ));
        }
    };
    Ok(Shape {
        radio,
        size,
        frames,
        spp,
        policy,
    })
}

pub(crate) fn open(
    session: &B200Session,
    options: &TxOptions,
    args: &StreamArgs,
) -> Result<B200TxStreamer> {
    let shape = shape(args)?;
    if shape.radio >= session.num_radio_chains()? {
        return Err(Error::invalid_arg("TX channel does not exist"));
    }
    if !options.rate.is_finite() || options.rate <= 0.0 || !options.dsp_freq.is_finite() {
        return Err(Error::invalid_arg(
            "TX rate must be positive and frequencies finite",
        ));
    }
    let tick_rate = session.codec().clock_rate();
    if tick_rate <= 0.0 {
        return Err(Error::device("initialize the radio before opening TX"));
    }
    let ep = session
        .iface()
        .device()
        .claim_interface(DATA_SEND_INTERFACE)
        .wait()
        .map_err(|e| Error::Usb(format!("cannot claim B2xx TX interface: {e}")))?
        .endpoint::<Bulk, Out>(DATA_SEND_ENDPOINT)
        .map_err(|e| Error::Usb(format!("cannot open B2xx TX endpoint: {e}")))?;
    let port = session.ctrl().radio(shape.radio)?;
    port.poke32(SR_TX_FMT, 0)?;
    port.poke32(SR_TX_CTRL + 8, 0)?;
    port.poke32(SR_TX_CTRL + 12, 0)?;
    port.poke32(SR_TX_CTRL, shape.policy)?;
    let mut dsp = TxDsp::new(Box::new(port.clone()));
    dsp.set_tick_rate(tick_rate)?;
    dsp.set_freq(options.dsp_freq)?;
    let rate = dsp.set_host_rate(options.rate)?;
    session.ctrl().clear_tx_events()?;
    let side = crate::rx_stream::side_for(shape.radio, session.misc().swap_atr());
    let free = (0..shape.frames).map(|_| ep.allocate(shape.size)).collect();
    let mut stream = B200TxStreamer {
        link: Box::new(UsbSend {
            ep,
            free,
            current: None,
            staging: vec![0; shape.size],
            lengths: VecDeque::with_capacity(shape.frames),
            size: shape.size,
            count: shape.frames,
        }),
        hardware: Some(Hardware {
            ctrl: session.ctrl().clone(),
            atr: GpioAtr::radio(Box::new(port)),
            side,
            radio: shape.radio,
            policy: shape.policy,
        }),
        format: args.host_format,
        spp: shape.spp,
        sid: TX_DATA_SIDS[shape.radio],
        seq: 0,
        tick_rate,
        rate,
        scale: dsp.scaling_adjustment(),
        cached: None,
    };
    // Construct the cleanup guard before enabling RF, including failures here.
    let hardware = stream.hardware.as_mut().unwrap();
    session
        .codec()
        .set_active_chains(side == Side::Fe1, side == Side::Fe2, false, false)?;
    hardware.atr.enable_atr()?;
    hardware.atr.set_b2xx_states(side, false, true, true)?;
    Ok(stream)
}

impl B200TxStreamer {
    /// The actual host sample rate after interpolation coercion.
    #[must_use]
    pub fn sample_rate(&self) -> f64 {
        self.rate
    }

    /// Master clock rate used for packet and event timestamps.
    #[must_use]
    pub fn tick_rate(&self) -> f64 {
        self.tick_rate
    }
}

impl RawTxStreamer for B200TxStreamer {
    fn num_channels(&self) -> usize {
        1
    }
    fn max_num_samps(&self) -> usize {
        self.spp
    }
    fn host_format(&self) -> SampleFormat {
        self.format
    }
    fn wire_format(&self) -> SampleFormat {
        SampleFormat::Sc16
    }

    fn send_raw(&mut self, buffs: &[&[u8]], timeout: Duration, md: &TxMetadata) -> Result<usize> {
        let host_bytes = self.format.bytes_per_sample().unwrap();
        if buffs.len() != 1 || !buffs[0].len().is_multiple_of(host_bytes) {
            return Err(Error::Buffer(
                "TX needs one buffer containing whole complex samples".into(),
            ));
        }
        if md.time_spec.is_some_and(|t| t.to_ticks(self.tick_rate) < 0) {
            return Err(Error::invalid_arg("TX timestamp must be nonnegative"));
        }
        let total = buffs[0].len() / host_bytes;
        if total == 0 && md.start_of_burst {
            self.cached = Some(md.clone());
            return Ok(0);
        }
        if total == 0 && !md.end_of_burst {
            return Ok(0);
        }
        let Some(frame) = self.link.get_send_buff(timeout)? else {
            if total == 0 {
                return Err(Error::Timeout(timeout));
            }
            return Ok(0);
        };
        let mut metadata = md.clone();
        if total > 0
            && let Some(cached) = self.cached.take()
        {
            metadata.time_spec = metadata.time_spec.or(cached.time_spec);
            metadata.start_of_burst = cached.start_of_burst;
            metadata.end_of_burst |= cached.end_of_burst;
        }
        let count = total.min(self.spp);
        let wire_count = count.max(1);
        let tsf = metadata
            .time_spec
            .map(|t| t.to_ticks(self.tick_rate) as u64);
        let end = metadata.end_of_burst && count == total;
        let header = if tsf.is_some() { 16 } else { 8 };
        let len = header + wire_count * 4;
        let dest = self.link.frame_bytes_mut(frame);
        let word = len as u32
            | (u32::from(self.seq & 0xfff) << 16)
            | (u32::from(tsf.is_some()) << 29)
            | (u32::from(end) << 28);
        dest[..4].copy_from_slice(&word.to_le_bytes());
        dest[4..8].copy_from_slice(&self.sid.to_le_bytes());
        if let Some(ticks) = tsf {
            dest[8..12].copy_from_slice(&((ticks >> 32) as u32).to_le_bytes());
            dest[12..16].copy_from_slice(&(ticks as u32).to_le_bytes());
        }
        if count == 0 {
            dest[header..len].fill(0);
        } else {
            convert(
                &buffs[0][..count * host_bytes],
                &mut dest[header..len],
                self.format,
                self.scale,
            );
        }
        self.link.commit_send_buff(frame, len)?;
        self.seq = (self.seq + 1) & 0xfff;
        Ok(count)
    }

    fn recv_async_msg(&mut self, timeout: Duration) -> Result<Option<AsyncMetadata>> {
        match &self.hardware {
            Some(h) => h.ctrl.recv_tx_event(h.radio, self.tick_rate, timeout),
            None => Ok(None),
        }
    }
}

fn convert(src: &[u8], dest: &mut [u8], format: SampleFormat, scale: f64) {
    for (input, output) in src
        .chunks_exact(format.bytes_per_sample().unwrap())
        .zip(dest.chunks_exact_mut(4))
    {
        let (i, q) = match format {
            SampleFormat::Sc16 => (
                i16::from_ne_bytes(input[..2].try_into().unwrap()),
                i16::from_ne_bytes(input[2..].try_into().unwrap()),
            ),
            SampleFormat::Fc32 => (
                quantize(f32::from_ne_bytes(input[..4].try_into().unwrap()) as f64 * scale),
                quantize(f32::from_ne_bytes(input[4..].try_into().unwrap()) as f64 * scale),
            ),
            SampleFormat::Fc64 => (
                quantize(f64::from_ne_bytes(input[..8].try_into().unwrap()) * scale),
                quantize(f64::from_ne_bytes(input[8..].try_into().unwrap()) * scale),
            ),
            _ => unreachable!(),
        };
        output.copy_from_slice(&((u32::from(i as u16) << 16) | u32::from(q as u16)).to_le_bytes());
    }
}

fn quantize(value: f64) -> i16 {
    value.round().clamp(-32768.0, 32767.0) as i16
}

impl Drop for B200TxStreamer {
    fn drop(&mut self) {
        if let Some(h) = &self.hardware {
            // Idle the RF switches first; the link then cancels outstanding USB I/O.
            let _ = h.atr.set_b2xx_states(h.side, false, false, true);
            if let Ok(port) = h.ctrl.radio(h.radio) {
                let _ = port.poke32(SR_TX_CTRL + 8, 0);
                let _ = port.poke32(SR_TX_CTRL + 12, 0);
                let _ = port.poke32(SR_TX_CTRL, h.policy);
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::sync::{Arc, Mutex};
    use uhd_core::{TimeSpec, TxStream, num_complex::Complex};

    #[derive(Debug, Default)]
    struct Sent {
        packets: Vec<Vec<u8>>,
        stalled: bool,
    }
    struct FakeLink {
        sent: Arc<Mutex<Sent>>,
        bytes: [u8; 64],
    }
    impl LinkInfo for FakeLink {
        fn num_frames(&self) -> usize {
            2
        }
        fn frame_size(&self) -> usize {
            64
        }
    }
    impl SendLink for FakeLink {
        fn get_send_buff(&mut self, _: Duration) -> Result<Option<FrameId>> {
            Ok((!self.sent.lock().unwrap().stalled).then_some(FrameId(0)))
        }
        fn frame_bytes_mut(&mut self, _: FrameId) -> &mut [u8] {
            &mut self.bytes
        }
        fn commit_send_buff(&mut self, _: FrameId, len: usize) -> Result<()> {
            self.sent
                .lock()
                .unwrap()
                .packets
                .push(self.bytes[..len].to_vec());
            Ok(())
        }
        fn release_send_buff(&mut self, _: FrameId) {}
    }
    fn stream(format: SampleFormat) -> (B200TxStreamer, Arc<Mutex<Sent>>) {
        let sent = Arc::new(Mutex::new(Sent::default()));
        (
            B200TxStreamer {
                link: Box::new(FakeLink {
                    sent: sent.clone(),
                    bytes: [0; 64],
                }),
                hardware: None,
                format,
                spp: 2,
                sid: 0x50,
                seq: 0xfff,
                tick_rate: 32e6,
                rate: 1e6,
                scale: 32767.0,
                cached: None,
            },
            sent,
        )
    }
    fn unpack(bytes: &[u8]) -> chdr::IfPacket {
        chdr::unpack(bytes, chdr::LinkType::Chdr, chdr::Endian::Little).unwrap()
    }

    #[test]
    fn typed_send_preserves_iq_timestamps_and_eob_across_short_sends() {
        let (raw, sent) = stream(SampleFormat::Sc16);
        let mut tx = TxStream::<Complex<i16>>::new(Box::new(raw)).unwrap();
        let samples = [
            Complex::new(0x1234i16, -2),
            Complex::new(-32768, 32767),
            Complex::new(3, -4),
        ];
        let md = TxMetadata::one_shot().at(TimeSpec::from_secs(1.0));
        assert_eq!(tx.send_one(&samples, Duration::ZERO, &md).unwrap(), 2);
        let tail = TxMetadata {
            time_spec: Some(TimeSpec::from_ticks(32_000_064, 32e6)),
            start_of_burst: false,
            end_of_burst: true,
        };
        assert_eq!(
            tx.send_one(&samples[2..], Duration::ZERO, &tail).unwrap(),
            1
        );
        let sent = sent.lock().unwrap();
        let first = unpack(&sent.packets[0]);
        assert_eq!(first.info.sid, Some(0x50));
        assert_eq!(first.info.packet_count, 0xfff);
        assert_eq!(first.info.tsf, Some(32_000_000));
        assert!(!first.info.eob);
        assert_eq!(first.payload, [0x1234_fffe, 0x8000_7fff]);
        let last = unpack(&sent.packets[1]);
        assert_eq!(last.info.packet_count, 0);
        assert_eq!(last.info.tsf, Some(32_000_064));
        assert!(last.info.eob);
        assert_eq!(last.payload, [0x0003_fffc]);
    }

    #[test]
    fn empty_sob_survives_a_timeout_and_empty_eob_pads_one_sample() {
        let (mut tx, sent) = stream(SampleFormat::Sc16);
        let sob = TxMetadata {
            start_of_burst: true,
            time_spec: Some(TimeSpec::from_secs(2.0)),
            ..Default::default()
        };
        assert_eq!(tx.send_raw(&[&[]], Duration::ZERO, &sob).unwrap(), 0);
        assert!(sent.lock().unwrap().packets.is_empty());
        sent.lock().unwrap().stalled = true;
        assert_eq!(
            tx.send_raw(&[&[0; 4]], Duration::ZERO, &TxMetadata::default())
                .unwrap(),
            0
        );
        assert_eq!(tx.seq, 0xfff);
        sent.lock().unwrap().stalled = false;
        assert_eq!(
            tx.send_raw(&[&[0; 4]], Duration::ZERO, &TxMetadata::default())
                .unwrap(),
            1
        );
        assert_eq!(
            unpack(&sent.lock().unwrap().packets[0]).info.tsf,
            Some(64_000_000)
        );
        let eob = TxMetadata {
            end_of_burst: true,
            ..Default::default()
        };
        tx.send_raw(&[&[]], Duration::ZERO, &eob).unwrap();
        let end = unpack(&sent.lock().unwrap().packets[1]);
        assert!(end.info.eob);
        assert_eq!(end.info.tsf, None);
        assert_eq!(end.payload, [0]);
        sent.lock().unwrap().stalled = true;
        assert!(matches!(
            tx.send_raw(&[&[]], Duration::ZERO, &eob),
            Err(Error::Timeout(_))
        ));
    }

    #[test]
    fn float_conversion_scales_and_saturates_without_swapping_iq() {
        let (raw, sent) = stream(SampleFormat::Fc32);
        let mut tx = TxStream::<Complex<f32>>::new(Box::new(raw)).unwrap();
        tx.send_one(
            &[Complex::new(0.5, -0.5), Complex::new(2.0, -2.0)],
            Duration::ZERO,
            &TxMetadata::one_shot(),
        )
        .unwrap();
        assert_eq!(
            unpack(&sent.lock().unwrap().packets[0]).payload,
            [0x4000_c000, 0x7fff_8000]
        );
        let (raw, sent) = stream(SampleFormat::Fc64);
        let mut tx = TxStream::<Complex<f64>>::new(Box::new(raw)).unwrap();
        tx.send_one(
            &[Complex::new(f64::NAN, f64::INFINITY)],
            Duration::ZERO,
            &TxMetadata::one_shot(),
        )
        .unwrap();
        assert_eq!(
            unpack(&sent.lock().unwrap().packets[0]).payload,
            [0x0000_7fff]
        );
    }

    #[test]
    fn invalid_buffer_shapes_and_negative_times_do_not_send() {
        let (mut tx, sent) = stream(SampleFormat::Sc16);
        for buffs in [
            vec![],
            vec![&[0u8; 3][..]],
            vec![&[0u8; 4][..], &[0u8; 4][..]],
        ] {
            assert!(
                tx.send_raw(&buffs, Duration::ZERO, &TxMetadata::default())
                    .is_err()
            );
        }
        assert!(
            tx.send_raw(
                &[&[0; 4]],
                Duration::ZERO,
                &TxMetadata::one_shot().at(TimeSpec::from_secs(-1.0))
            )
            .is_err()
        );
        assert!(sent.lock().unwrap().packets.is_empty());
    }

    #[test]
    fn validates_stream_parameters_before_touching_hardware() {
        assert_eq!(shape(&StreamArgs::new()).unwrap().spp, 2040);
        for args in [
            StreamArgs::new().with_channels([0, 1]),
            StreamArgs::new().with_wire_format(SampleFormat::Sc8),
            StreamArgs::new().with_arg("spp", "0"),
            StreamArgs::new().with_arg("spp", "2041"),
            StreamArgs::new().with_arg("send_frame_size", "7"),
            StreamArgs::new().with_arg("num_send_frames", "1"),
            StreamArgs::new().with_arg("underflow_policy", "unknown"),
        ] {
            assert!(shape(&args).is_err(), "{args:?}");
        }
    }
}
