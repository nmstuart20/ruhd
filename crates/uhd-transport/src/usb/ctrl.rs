//! Peek and poke against the FPGA's settings bus, over the FX3's bulk
//! endpoints.
//!
//! Mirrors the subset of `b200_radio_ctrl_core.cpp` that a compatibility check
//! needs: build one CHDR context packet, send it, read the reply, take the
//! 64-bit readback out of it.
//!
//! # Why this is not a control transfer
//!
//! Everything else in this module talks to the *FX3* over endpoint 0. This
//! talks to the *FPGA*, which the FX3 cannot answer for — the vendor requests
//! stop at the microcontroller. So the settings bus is reached the same way
//! samples are: bulk endpoints carrying CHDR packets, which the FX3 forwards
//! across the GPIF onto the FPGA's bus and back. That is also why nothing here
//! works until [`fpga::configure`](super::fpga::configure) has run: an
//! unconfigured FPGA has no settings bus, and the request simply times out.
//!
//! ```text
//!   interface 3, endpoint 0x04 (out)  ->  ctrl packets to the FPGA
//!   interface 4, endpoint 0x88 (in)   <-  responses
//! ```
//!
//! # The packet
//!
//! Four little-endian 32-bit words, and no more — a ctrl packet is a fixed
//! shape:
//!
//! | Word | Contents |
//! | --- | --- |
//! | 0 | CHDR header: context bit, 12-bit sequence number, length in bytes (16) |
//! | 1 | stream ID, [`LOCAL_CTRL_SID`] |
//! | 2 | settings-bus address, or [`SR_READBACK`] for a read |
//! | 3 | data to write, or the readback index for a read |
//!
//! The reply comes back with the stream ID reversed and the same sequence
//! number, and its two payload words are the 64-bit readback, high word first.
//!
//! # What is deliberately missing
//!
//! UHD's ctrl core also carries timestamps, queues several outstanding
//! transactions before it waits for acknowledgements, and demultiplexes replies
//! from the GPSDO UART out of the same endpoint. None of that is here: this
//! sends one packet and waits for its reply, under a mutex. That is the right
//! trade while the only caller is a compatibility read, and it is the wrong
//! trade for a streaming device — when the radio control core lands it should
//! replace this, not wrap it.

use std::sync::Mutex;
use std::time::Duration;

use nusb::MaybeFuture;
use nusb::transfer::{Buffer, Bulk, In, Out};
use tracing::{debug, trace};
use uhd_core::{Error, Result};

use crate::endpoint::UsbEndpoint;
use crate::register::RegisterIface;

use super::fx3::Fx3Iface;

/// USB interface carrying ctrl packets to the device.
/// `B200_USB_CTRL_SEND_INTERFACE` in `b200_impl.hpp`.
pub const CTRL_SEND_INTERFACE: u8 = 3;

/// Bulk OUT endpoint for ctrl packets.
/// `B200_USB_CTRL_SEND_ENDPOINT` in `b200_impl.hpp`, which names it `4`; the
/// direction bit makes the address `0x04`.
pub const CTRL_SEND_ENDPOINT: u8 = 0x04;

/// USB interface carrying ctrl responses back.
/// `B200_USB_CTRL_RECV_INTERFACE` in `b200_impl.hpp`.
pub const CTRL_RECV_INTERFACE: u8 = 4;

/// Bulk IN endpoint for ctrl responses.
/// `B200_USB_CTRL_RECV_ENDPOINT` in `b200_impl.hpp` names it `8`; the IN
/// direction bit makes the address `0x88`.
pub const CTRL_RECV_ENDPOINT: u8 = 0x88;

/// Stream ID of the host's own control endpoint into the FPGA.
///
/// `B200_LOCAL_CTRL_SID` in `b200_impl.hpp`. Responses arrive under the
/// halves-swapped form, `0x0040_0000` — see [`flip_sid`].
pub const LOCAL_CTRL_SID: u32 = 0x0000_0040;

/// Settings register that starts a readback.
///
/// `SR_READBACK` in `b200_radio_ctrl_core.cpp`. Writing a readback *index*
/// here makes the FPGA answer with the corresponding 64-bit register rather
/// than merely acknowledging.
pub const SR_READBACK: u32 = 32;

/// How long to wait for a ctrl response.
///
/// `ACK_TIMEOUT` in `b200_radio_ctrl_core.cpp`, described there as "supposed to
/// be worst case practical timeout".
pub const ACK_TIMEOUT: Duration = Duration::from_secs(2);

/// How long a single read gets while draining stale responses at open time.
const FLUSH_TIMEOUT: Duration = Duration::from_millis(10);

/// How many stale responses to drain before giving up on a quiet endpoint.
///
/// The same bound `libusb_zero_copy_single` uses when it flushes a receive
/// endpoint it has just claimed.
const FLUSH_LIMIT: usize = 100;

/// Words in a ctrl packet: header, stream ID, address, data.
const CTRL_PACKET_WORDS: usize = 4;

/// Swaps the halves of a stream ID, which is how the FPGA addresses a reply
/// back to its sender. `FLIP_SID` in `b200_impl.hpp`.
#[must_use]
pub fn flip_sid(sid: u32) -> u32 {
    sid.rotate_right(16)
}

/// An open ctrl path to the FPGA's settings bus.
///
/// Holds both bulk endpoints and the sequence counter behind one mutex, so a
/// transaction is atomic against other threads — which is what
/// [`RegisterIface`]'s `&self` promises.
#[derive(Debug)]
pub struct LocalCtrl {
    inner: Mutex<Transport>,
    endpoint: UsbEndpoint,
}

/// The two endpoints and the sequence number, which only ever move together.
#[derive(Debug)]
struct Transport {
    send: nusb::Endpoint<Bulk, Out>,
    recv: nusb::Endpoint<Bulk, In>,
    /// Next sequence number. Twelve bits on the wire; kept wide and masked at
    /// use, exactly as `_seq_out` is.
    seq: u32,
    /// Response buffer size, from the endpoint's maximum packet size — 1024
    /// over USB 3 and 512 over USB 2, which is what `b200_impl` computes from
    /// the USB speed and passes as `recv_frame_size`.
    frame_size: usize,
}

impl LocalCtrl {
    /// Claims the ctrl interfaces and opens the endpoints.
    ///
    /// The FPGA must already be configured — see the [module docs](self). The
    /// receive endpoint is drained on the way in, because a previous process
    /// that died mid-transaction leaves its reply sitting in the FX3's buffer,
    /// and reading that as the answer to the *next* question is a very
    /// confusing way to fail.
    ///
    /// # Errors
    ///
    /// [`Error::Usb`] if an interface cannot be claimed — the usual cause is
    /// another process (a running `uhd_usrp_probe`, or a stale one) holding the
    /// device — or if an endpoint is missing, which means the firmware running
    /// is not the B200 firmware.
    pub fn open(iface: &Fx3Iface) -> Result<Self> {
        let device = iface.device();
        let endpoint = iface.endpoint().clone();

        let claim = |number: u8| -> Result<nusb::Interface> {
            device.claim_interface(number).wait().map_err(|e| {
                Error::Usb(format!(
                    "cannot claim usb interface {number} on {endpoint}: {e}; another \
                     process may have the device open"
                ))
            })
        };

        let send_iface = claim(CTRL_SEND_INTERFACE)?;
        let recv_iface = claim(CTRL_RECV_INTERFACE)?;

        let send = send_iface
            .endpoint::<Bulk, Out>(CTRL_SEND_ENDPOINT)
            .map_err(|e| {
                Error::Usb(format!(
                    "cannot open ctrl endpoint {CTRL_SEND_ENDPOINT:#04x} on {endpoint}: {e}"
                ))
            })?;
        let recv = recv_iface
            .endpoint::<Bulk, In>(CTRL_RECV_ENDPOINT)
            .map_err(|e| {
                Error::Usb(format!(
                    "cannot open ctrl endpoint {CTRL_RECV_ENDPOINT:#04x} on {endpoint}: {e}"
                ))
            })?;

        let frame_size = recv.max_packet_size().max(CTRL_PACKET_WORDS * 4);
        let mut transport = Transport {
            send,
            recv,
            seq: 0,
            frame_size,
        };
        transport.flush();

        debug!(%endpoint, frame_size, "opened fpga ctrl endpoints");
        Ok(Self {
            inner: Mutex::new(transport),
            endpoint,
        })
    }

    /// How this device was named when it was opened, for error messages.
    #[must_use]
    pub fn endpoint(&self) -> &UsbEndpoint {
        &self.endpoint
    }

    /// One request-response exchange, under the lock.
    fn transact(&self, addr_word: u32, data_word: u32, readback: bool) -> Result<u64> {
        let mut t = self
            .inner
            .lock()
            .unwrap_or_else(std::sync::PoisonError::into_inner);
        t.transact(addr_word, data_word, readback)
            .map_err(|e| match e {
                // Give the caller the device name; the transport does not have
                // it and threading it through every message is noise.
                Error::Transport(msg) => Error::Transport(format!("{} on {}", msg, self.endpoint)),
                other => other,
            })
    }
}

impl Transport {
    /// Discards anything already waiting on the receive endpoint.
    ///
    /// Errors are the *expected* outcome — a timeout is what "nothing there"
    /// looks like — so nothing here is reported. A genuinely broken endpoint
    /// fails again on the first real transaction, with a message about that
    /// transaction rather than about the flush.
    fn flush(&mut self) {
        for _ in 0..FLUSH_LIMIT {
            let completion = self
                .recv
                .transfer_blocking(Buffer::new(self.frame_size), FLUSH_TIMEOUT);
            if completion.status.is_err() || completion.actual_len == 0 {
                return;
            }
            trace!(
                bytes = completion.actual_len,
                "discarded stale ctrl response"
            );
        }
    }

    fn transact(&mut self, addr_word: u32, data_word: u32, readback: bool) -> Result<u64> {
        let seq = self.seq;
        self.seq = self.seq.wrapping_add(1);

        let packet = build_packet(seq, addr_word, data_word);
        trace!(
            seq,
            addr = format_args!("{addr_word:#x}"),
            data = format_args!("{data_word:#x}"),
            "ctrl packet out"
        );
        self.send
            .transfer_blocking(Vec::from(packet).into(), ACK_TIMEOUT)
            .into_result()
            .map_err(|e| Error::Transport(format!("ctrl packet (seq {seq}) was not sent: {e}")))?;

        let completion = self
            .recv
            .transfer_blocking(Buffer::new(self.frame_size), ACK_TIMEOUT);
        let buffer = completion.into_result().map_err(|e| {
            Error::Transport(format!(
                "no response to ctrl packet (seq {seq}): {e}; if the fpga has not been \
                 configured it cannot answer"
            ))
        })?;

        let value = parse_response(&buffer, seq)?;
        Ok(if readback { value } else { 0 })
    }
}

/// Builds the four-word ctrl packet.
///
/// The header word is what `vrt_to_chdr(if_hdr_pack_le(...))` produces for the
/// packet `b200_radio_ctrl_core::send_pkt` builds — a context packet, with a
/// stream ID, no timestamp, no trailer, two payload words:
///
/// ```text
///   bit 31      context packet
///   bits 30-16  sequence number (12 bits, at bit 16)
///   bits 15-0   packet length in bytes: 4 words = 16
/// ```
///
/// The `has_tsf` and `eob` bits at 29 and 28 stay clear because the local ctrl
/// endpoint is untimed — `b200_impl` never calls `set_time` on it, so
/// `_use_time` is false for its whole life.
fn build_packet(seq: u32, addr_word: u32, data_word: u32) -> [u8; CTRL_PACKET_WORDS * 4] {
    let length_bytes = (CTRL_PACKET_WORDS * 4) as u32;
    let chdr = (1 << 31) | ((seq & 0xfff) << 16) | length_bytes;

    let mut packet = [0u8; CTRL_PACKET_WORDS * 4];
    for (word, chunk) in [chdr, LOCAL_CTRL_SID, addr_word, data_word]
        .into_iter()
        .zip(packet.chunks_exact_mut(4))
    {
        chunk.copy_from_slice(&word.to_le_bytes());
    }
    packet
}

/// Pulls the 64-bit readback out of a response, checking it is the right one.
///
/// The checks are `wait_for_ack`'s: the stream ID must be the flipped local
/// one, the sequence number must be the one just sent, it must be a context
/// packet, and it must carry exactly two payload words. A response failing any
/// of them is a response to something else, and treating it as the answer would
/// silently return the wrong register.
fn parse_response(buffer: &[u8], seq: u32) -> Result<u64> {
    let words: Vec<u32> = buffer
        .chunks_exact(4)
        .map(|c| u32::from_le_bytes([c[0], c[1], c[2], c[3]]))
        .collect();

    if words.len() < 2 {
        return Err(Error::Transport(format!(
            "ctrl response (seq {seq}) is {} bytes, too short to be a packet",
            buffer.len()
        )));
    }

    let chdr = words[0];
    if chdr >> 31 == 0 {
        return Err(Error::Transport(format!(
            "ctrl response (seq {seq}) is not a context packet: header {chdr:#010x}"
        )));
    }

    // `chdr_to_vrt` puts the timestamp flag at bit 29, and a timestamp is two
    // more header words. The local ctrl endpoint does not use one, but the
    // header says so rather than this assuming it.
    let has_tsf = (chdr >> 29) & 1 != 0;
    let header_words = 2 + if has_tsf { 2 } else { 0 };

    let got_seq = (chdr >> 16) & 0xfff;
    if got_seq != seq & 0xfff {
        return Err(Error::Transport(format!(
            "ctrl response is out of sequence: expected {}, got {got_seq}",
            seq & 0xfff
        )));
    }

    let expected_sid = flip_sid(LOCAL_CTRL_SID);
    if words[1] != expected_sid {
        return Err(Error::Transport(format!(
            "ctrl response (seq {seq}) carries stream id {:#010x}, not {expected_sid:#010x}",
            words[1]
        )));
    }

    let payload = words.get(header_words..header_words + 2).ok_or_else(|| {
        Error::Transport(format!(
            "ctrl response (seq {seq}) has {} words, too few for a {header_words}-word \
             header and a 2-word payload",
            words.len()
        ))
    })?;

    // High word first: `wait_for_ack` reads payload[0] into `hi`.
    Ok((u64::from(payload[0]) << 32) | u64::from(payload[1]))
}

impl RegisterIface for LocalCtrl {
    /// Writes a settings-bus register.
    ///
    /// `addr` is a byte address, and the settings bus is word-addressed, so the
    /// register index sent is `addr / 4` — `poke32` in
    /// `b200_radio_ctrl_core.cpp`.
    fn poke32(&self, addr: u32, data: u32) -> Result<()> {
        self.transact(addr / 4, data, false).map(|_| ())
    }

    /// Reads a settings-bus register.
    ///
    /// Readback is 64 bits wide, so this reads the pair and picks the half
    /// `addr` names — `(addr / 4) & 1` chooses, exactly as `peek32` does.
    fn peek32(&self, addr: u32) -> Result<u32> {
        let value = self.peek64(addr & !0x7)?;
        Ok(if (addr / 4) & 1 == 1 {
            (value >> 32) as u32
        } else {
            value as u32
        })
    }

    /// Reads a 64-bit readback register.
    ///
    /// Native rather than two 32-bit reads: the FPGA answers a readback with
    /// all 64 bits, so the default implementation would do two round trips for
    /// no reason and, worse, could straddle a change between them.
    fn peek64(&self, addr: u32) -> Result<u64> {
        self.transact(SR_READBACK, addr / 8, true)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn the_header_word_matches_what_uhd_packs() {
        // Worked through gen_vrt_if_packet.py for the packet send_pkt builds:
        //   pred = has_sid only, so 2 header words and 2 payload words;
        //   vrt = (PACKET_TYPE_CONTEXT << 29) | (1 << 28) | (seq << 16) | 4
        //       = 0x5000_0004 for seq 0
        //   chdr = 4 words * 4 bytes = 16, | (seq & 0xfff) << 16,
        //          | ((vrt >> 30) & 1) << 31 = 1 << 31
        let packet = build_packet(0, SR_READBACK, 0);
        let chdr = u32::from_le_bytes([packet[0], packet[1], packet[2], packet[3]]);
        assert_eq!(chdr, 0x8000_0010);

        // Neither the timestamp bit (29) nor the end-of-burst bit (28) is set.
        assert_eq!((chdr >> 29) & 1, 0);
        assert_eq!((chdr >> 28) & 1, 0);
    }

    #[test]
    fn the_sequence_number_lands_at_bit_16_and_wraps_at_twelve_bits() {
        let chdr = |seq| {
            let p = build_packet(seq, 0, 0);
            u32::from_le_bytes([p[0], p[1], p[2], p[3]])
        };
        assert_eq!(chdr(1), 0x8001_0010);
        assert_eq!(chdr(0xfff), 0x8fff_0010);
        assert_eq!(chdr(0x1000), 0x8000_0010, "the 13th bit is not carried");
    }

    #[test]
    fn a_packet_is_four_little_endian_words() {
        let packet = build_packet(7, SR_READBACK, 3);
        assert_eq!(packet.len(), 16);
        let words: Vec<u32> = packet
            .chunks_exact(4)
            .map(|c| u32::from_le_bytes([c[0], c[1], c[2], c[3]]))
            .collect();
        assert_eq!(words[1], LOCAL_CTRL_SID);
        assert_eq!(words[2], SR_READBACK);
        assert_eq!(words[3], 3);
    }

    #[test]
    fn stream_ids_come_back_with_their_halves_swapped() {
        // FLIP_SID in b200_impl.hpp, and the check wait_for_ack makes.
        assert_eq!(flip_sid(LOCAL_CTRL_SID), 0x0040_0000);
        assert_eq!(flip_sid(flip_sid(LOCAL_CTRL_SID)), LOCAL_CTRL_SID);
    }

    /// Builds a reply the way the FPGA does, for the parser tests.
    fn response(seq: u32, sid: u32, hi: u32, lo: u32) -> Vec<u8> {
        let chdr = (1 << 31) | ((seq & 0xfff) << 16) | 16u32;
        [chdr, sid, hi, lo]
            .iter()
            .flat_map(|w| w.to_le_bytes())
            .collect()
    }

    #[test]
    fn a_readback_is_high_word_first() {
        // The compatibility register a B205mini answers with: signature
        // 0xACE0BA5E in the high word, then major and minor.
        let reply = response(0, flip_sid(LOCAL_CTRL_SID), 0xace0_ba5e, 0x0007_0000);
        let value = parse_response(&reply, 0).expect("parsed");
        assert_eq!(value >> 32, 0xace0_ba5e);
        assert_eq!((value >> 16) & 0xffff, 7);
        assert_eq!(value & 0xffff, 0);
    }

    #[test]
    fn a_reply_to_someone_elses_packet_is_refused() {
        let wrong_sid = response(0, 0x0030_0000, 1, 2);
        let e = parse_response(&wrong_sid, 0).expect_err("wrong stream id");
        assert!(e.to_string().contains("stream id"), "{e}");

        let wrong_seq = response(4, flip_sid(LOCAL_CTRL_SID), 1, 2);
        let e = parse_response(&wrong_seq, 0).expect_err("stale response");
        assert!(e.to_string().contains("out of sequence"), "{e}");
    }

    #[test]
    fn a_timestamped_reply_has_its_payload_two_words_further_in() {
        // Not what the local ctrl endpoint produces, but the header says
        // where the payload is and the parser believes it rather than the
        // other way round.
        let chdr = (1u32 << 31) | (1 << 29) | 24u32;
        let mut reply: Vec<u8> = [
            chdr,
            flip_sid(LOCAL_CTRL_SID),
            0,
            0,
            0xdead_beef,
            0x0000_0001,
        ]
        .iter()
        .flat_map(|w| w.to_le_bytes())
        .collect();
        assert_eq!(reply.len(), 24);
        assert_eq!(
            parse_response(&reply, 0).expect("parsed"),
            0xdead_beef_0000_0001
        );

        // And a packet that claims a timestamp but is too short to hold one
        // is rejected rather than read off the end.
        reply.truncate(16);
        assert!(parse_response(&reply, 0).is_err());
    }

    #[test]
    fn a_runt_response_is_rejected() {
        assert!(parse_response(&[], 0).is_err());
        assert!(parse_response(&[0; 4], 0).is_err());
    }

    #[test]
    fn a_data_packet_is_not_a_ctrl_response() {
        // Context bit clear: this is sample data that ended up on the ctrl
        // endpoint, and reading a register out of it would be nonsense.
        let mut reply = response(0, flip_sid(LOCAL_CTRL_SID), 1, 2);
        reply[3] &= 0x7f;
        let e = parse_response(&reply, 0).expect_err("not a context packet");
        assert!(e.to_string().contains("context packet"), "{e}");
    }

    #[test]
    fn endpoint_addresses_match_the_reference() {
        // B200_USB_CTRL_* in b200_impl.hpp, with libusb1_zero_copy's
        // `(endpoint & 0x7f) | 0x80` applied to the receive side.
        assert_eq!(CTRL_SEND_INTERFACE, 3);
        assert_eq!(CTRL_SEND_ENDPOINT, 0x04);
        assert_eq!(CTRL_RECV_INTERFACE, 4);
        assert_eq!(CTRL_RECV_ENDPOINT, 0x88);
        assert_eq!(LOCAL_CTRL_SID, 0x0000_0040);
        assert_eq!(SR_READBACK, 32);
    }
}
