//! B2xx FPGA settings-bus control over the dedicated USB bulk endpoints.

use std::collections::VecDeque;
use std::sync::Mutex;
use std::time::Duration;

use chdr::{Endian, IfPacketInfo, LinkType, PacketType};
use nusb::MaybeFuture;
use nusb::transfer::{Buffer, Bulk, In, Out};
use tracing::{debug, trace, warn};
use uhd_core::{Error, Result};
use uhd_transport::usb::{B2xxProduct, Fx3Iface, expected_fpga_compat};
use uhd_transport::{RegisterIface, UsbEndpoint};

/// Values wired by `b200_impl.hpp`.
pub const CTRL_SEND_INTERFACE: u8 = 3;
pub const CTRL_SEND_ENDPOINT: u8 = 0x04;
pub const CTRL_RECV_INTERFACE: u8 = 4;
pub const CTRL_RECV_ENDPOINT: u8 = 0x88;
pub const LOCAL_CTRL_SID: u32 = 0x0000_0040;
pub const LOCAL_RESP_SID: u32 = 0x0040_0000;
/// Settings-bus command SID for radio chain 0.
pub const RADIO0_CTRL_SID: u32 = 0x0000_0010;
/// Settings-bus response SID for radio chain 0.
pub const RADIO0_RESP_SID: u32 = 0x0010_0000;
const SR_READBACK: u32 = 32;
const SR_TEST_BYTES: u32 = 21 * 4;
const RB32_TEST_BYTES: u32 = 0;
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

/// Decoded FPGA compatibility register.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct FpgaVersion {
    pub signature: u32,
    pub major: u16,
    pub minor: u16,
}

/// Successful result of the control-channel hardware gate.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct ControlGate {
    pub fpga: FpgaVersion,
    pub scratch_iterations: usize,
}

/// Thread-safe B2xx radio control core.
#[derive(Debug)]
pub struct RadioCtrl {
    inner: Mutex<Transport>,
    endpoint: UsbEndpoint,
}

#[derive(Debug)]
struct Transport {
    send: nusb::Endpoint<Bulk, Out>,
    recv: nusb::Endpoint<Bulk, In>,
    seq_out: u64,
    outstanding: VecDeque<u16>,
    frame_size: usize,
    ctrl_sid: u32,
    resp_sid: u32,
}

impl RadioCtrl {
    /// Claim interfaces 3/4 and open endpoints 0x04/0x88.
    pub fn open(iface: &Fx3Iface) -> Result<Self> {
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
            ctrl_sid: LOCAL_CTRL_SID,
            resp_sid: LOCAL_RESP_SID,
        };
        inner.flush();
        Ok(Self {
            inner: Mutex::new(inner),
            endpoint: iface.endpoint().clone(),
        })
    }

    /// Read and validate FPGA compatibility, then perform 10,000 deterministic
    /// scratch-register poke/readback transactions with zero mismatches.
    pub fn gate(&self, product: B2xxProduct) -> Result<ControlGate> {
        let raw = self.peek64(0)?;
        let fpga = FpgaVersion {
            signature: (raw >> 32) as u32,
            major: (raw >> 16) as u16,
            minor: raw as u16,
        };
        if fpga.signature != 0xace0_ba5e {
            return Err(Error::Firmware(format!(
                "B2xx FPGA signature is {:#010x}, expected 0xace0ba5e",
                fpga.signature
            )));
        }
        let expected = expected_fpga_compat(product);
        if fpga.major != expected {
            return Err(Error::Firmware(format!(
                "B2xx FPGA compatibility is {}.{}, expected major {expected}",
                fpga.major, fpga.minor
            )));
        }
        // SR_TEST/RB32_TEST are radio-core registers, not local-core
        // registers. UHD runs register_loopback_self_test() on perif.ctrl,
        // whose first radio is routed under SID 0x10.
        self.set_route(RADIO0_CTRL_SID, RADIO0_RESP_SID)?;
        let scratch_result = (|| {
            let mut value = 0x6d2b_79f5u32;
            for iteration in 0..10_000 {
                // Fixed xorshift sequence makes a failure exactly reproducible.
                value ^= value << 13;
                value ^= value >> 17;
                value ^= value << 5;
                self.poke32(SR_TEST_BYTES, value)?;
                let got = self.peek32(RB32_TEST_BYTES)?;
                if got != value {
                    return Err(Error::Device(format!(
                        "B2xx scratch mismatch at iteration {iteration}: wrote {value:#010x}, read {got:#010x}"
                    )));
                }
            }
            Ok(())
        })();
        let restore_result = self.set_route(LOCAL_CTRL_SID, LOCAL_RESP_SID);
        scratch_result?;
        restore_result?;
        debug!(?fpga, "B2xx radio control gate passed");
        Ok(ControlGate {
            fpga,
            scratch_iterations: 10_000,
        })
    }

    fn locked(&self) -> std::sync::MutexGuard<'_, Transport> {
        self.inner
            .lock()
            .unwrap_or_else(std::sync::PoisonError::into_inner)
    }

    fn set_route(&self, ctrl_sid: u32, resp_sid: u32) -> Result<()> {
        let mut t = self.locked();
        if !t.outstanding.is_empty() {
            return Err(Error::Internal(
                "cannot change B2xx control SID with responses outstanding".into(),
            ));
        }
        t.ctrl_sid = ctrl_sid;
        t.resp_sid = resp_sid;
        Ok(())
    }
}

impl Transport {
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

    fn send(&mut self, addr: u32, data: u32) -> Result<()> {
        let seq = (self.seq_out & 0xfff) as u16;
        let info = IfPacketInfo {
            link_type: LinkType::Chdr,
            packet_type: PacketType::Context,
            packet_count: seq,
            sid: Some(self.ctrl_sid),
            payload_bytes: 8,
            ..Default::default()
        };
        let packet = chdr::pack(&info, &[addr, data], Endian::Little)
            .map_err(|e| Error::Internal(format!("cannot build B2xx control packet: {e}")))?;
        self.send
            .transfer_blocking(packet.into(), ACK_TIMEOUT)
            .into_result()
            .map_err(|e| Error::Transport(format!("B2xx control seq {seq} send failed: {e}")))?;
        self.outstanding.push_back(seq);
        self.seq_out = self.seq_out.wrapping_add(1);
        trace!(seq, addr, data, "B2xx control packet sent");
        Ok(())
    }

    fn wait_one(&mut self, readback: bool) -> Result<Option<u64>> {
        let expected = self.outstanding.pop_front().ok_or_else(|| {
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
                        "B2xx control seq {expected} timed out: {e}; response state resynchronized"
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
            if packet.info.sid != Some(self.resp_sid)
                || packet.info.packet_type != PacketType::Context
                || packet.payload.len() != 2
            {
                warn!(sid=?packet.info.sid, "discarding unrelated B2xx control response");
                continue;
            }
            let got = packet.info.packet_count;
            if sequence_relation(expected, got) != SequenceRelation::Match {
                // A late response is safe to discard. A response for a queued
                // future command proves this acknowledgement was lost; clear
                // the queue because USB has already consumed the future one.
                if sequence_relation(expected, got) == SequenceRelation::Future {
                    self.flush();
                    return Err(Error::Transport(format!(
                        "B2xx control response jumped from seq {expected} to {got}; response state resynchronized"
                    )));
                }
                warn!(expected, got, "discarding stale B2xx control response");
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

impl RegisterIface for RadioCtrl {
    fn poke32(&self, addr: u32, data: u32) -> Result<()> {
        let mut t = self.locked();
        t.send(addr / 4, data)?;
        if t.outstanding.len() >= MAX_OUTSTANDING {
            t.wait_one(false)?;
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
        t.send(SR_READBACK, addr / 8)?;
        t.drain_for_read().map_err(|e| match e {
            Error::Transport(s) => Error::Transport(format!("{s} on {}", self.endpoint)),
            other => other,
        })
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
        assert_eq!((RADIO0_CTRL_SID, RADIO0_RESP_SID), (0x10, 0x0010_0000));
        assert_eq!((SR_TEST_BYTES, RB32_TEST_BYTES), (84, 0));
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
}
