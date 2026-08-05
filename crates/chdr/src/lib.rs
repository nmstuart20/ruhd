//! Legacy VITA-49/CVITA interface packets used by pre-RFNoC USRPs.
//!
//! B2xx devices use the compact CHDR link header variant of UHD's
//! `vrt_if_packet.cpp`: words are transferred in host-selected byte order, the
//! SID is always present on CHDR, and the 12-bit CHDR packet count replaces
//! VRT's four-bit count.

use core::fmt;

/// Link-layer wrapper around the VRT interface header.
#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
pub enum LinkType {
    /// A plain VRT interface packet.
    #[default]
    Vrt,
    /// UHD's 32-bit compressed header (CVITA/CHDR).
    Chdr,
    /// VITA Radio Link Packet (`VRLP` ... `VEND`).
    Vrlp,
}

/// VRT packet type. Values are the three high bits of the VRT header.
#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
#[repr(u8)]
pub enum PacketType {
    /// Sample data.
    #[default]
    Data = 0,
    /// An IF-extension packet, which nothing in the B2xx path emits.
    IfExtension = 1,
    /// Context: the flow-control and settings-bus packets B2xx control uses.
    Context = 2,
}

/// Byte order of words on the link.
#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
pub enum Endian {
    /// Network byte order, which is what a UDP-attached USRP uses.
    Big,
    /// Host byte order on every platform this driver targets, and what the
    /// B2xx's USB path uses.
    #[default]
    Little,
}

/// Interface-packet metadata. Payload words are supplied separately to
/// [`pack`] and returned separately by [`unpack`].
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct IfPacketInfo {
    /// Which link header wraps the VRT header.
    pub link_type: LinkType,
    /// VRT packet type.
    pub packet_type: PacketType,
    /// Sequence number: 12 bits on CHDR, four on VRT.
    pub packet_count: u16,
    /// Start of burst.
    pub sob: bool,
    /// End of burst.
    pub eob: bool,
    /// Stream ID. Always present on CHDR.
    pub sid: Option<u32>,
    /// Class ID.
    pub cid: Option<u64>,
    /// Integer-seconds timestamp.
    pub tsi: Option<u32>,
    /// Fractional-seconds timestamp, in ticks.
    pub tsf: Option<u64>,
    /// Request/preserve a VRT trailer. The occupancy and burst indicator bits
    /// are normalized by the packer, as in UHD.
    pub trailer: Option<u32>,
    /// Number of meaningful payload bytes. This may be up to three bytes less
    /// than `payload.len() * 4`.
    pub payload_bytes: usize,
}

impl Default for IfPacketInfo {
    fn default() -> Self {
        Self {
            link_type: LinkType::Vrt,
            packet_type: PacketType::Data,
            packet_count: 0,
            sob: false,
            eob: false,
            sid: None,
            cid: None,
            tsi: None,
            tsf: None,
            trailer: None,
            payload_bytes: 0,
        }
    }
}

/// A decoded interface packet.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct IfPacket {
    /// The decoded header.
    pub info: IfPacketInfo,
    /// Payload words, in host order.
    pub payload: Vec<u32>,
    /// How many words the header occupied.
    pub header_words: usize,
    /// How many words the whole packet occupied, header and trailer included.
    pub packet_words: usize,
}

/// Invalid or truncated VRT/CVITA data.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct Error(pub &'static str);

impl fmt::Display for Error {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(self.0)
    }
}
impl std::error::Error for Error {}

const VRLP: u32 = u32::from_be_bytes(*b"VRLP");
const VEND: u32 = u32::from_be_bytes(*b"VEND");
const OCC: [u32; 4] = [0, 2, 1, 3];

fn encode_word(word: u32, endian: Endian) -> [u8; 4] {
    match endian {
        Endian::Big => word.to_be_bytes(),
        Endian::Little => word.to_le_bytes(),
    }
}
fn decode_word(bytes: &[u8], endian: Endian) -> u32 {
    let b = [bytes[0], bytes[1], bytes[2], bytes[3]];
    match endian {
        Endian::Big => u32::from_be_bytes(b),
        Endian::Little => u32::from_le_bytes(b),
    }
}

/// Pack a complete legacy interface packet.
pub fn pack(info: &IfPacketInfo, payload: &[u32], endian: Endian) -> Result<Vec<u8>, Error> {
    if info.payload_bytes > payload.len() * 4 || payload.len() * 4 - info.payload_bytes > 3 {
        return Err(Error("payload byte count does not match payload words"));
    }
    if info.link_type == LinkType::Chdr && info.sid.is_none() {
        return Err(Error("CHDR packets require a stream ID"));
    }

    let mut body = vec![0];
    let mut flags = 0u32;
    if let Some(sid) = info.sid {
        flags |= 1 << 28;
        body.push(sid);
    }
    if let Some(cid) = info.cid {
        flags |= 1 << 27;
        body.push((cid >> 32) as u32);
        body.push(cid as u32);
    }
    if let Some(tsi) = info.tsi {
        flags |= 3 << 22;
        body.push(tsi);
    }
    if let Some(tsf) = info.tsf {
        flags |= 1 << 20;
        body.push((tsf >> 32) as u32);
        body.push(tsf as u32);
    }
    if info.sob {
        flags |= 1 << 25;
    }
    if info.eob {
        flags |= 1 << 24;
    }
    let header_words = body.len();
    body.extend_from_slice(payload);
    if let Some(mut trailer) = info.trailer {
        flags |= 1 << 26;
        let empty = payload.len() * 4 - info.payload_bytes;
        trailer &= !((3 << 22) | (3 << 10));
        trailer |= (3 << 22) | (OCC[empty] << 10);
        body.push(trailer);
    }
    let vrt_words = body.len() as u32;
    let vrt = ((info.packet_type as u32) << 29)
        | flags
        | (u32::from(info.packet_count & 0xf) << 16)
        | vrt_words;
    body[0] = vrt;

    match info.link_type {
        LinkType::Vrt => {}
        LinkType::Chdr => {
            // CVITA only retains context, TSF and EOB from the VRT flags.
            let bytes = vrt_words * 4 - (payload.len() * 4 - info.payload_bytes) as u32;
            body[0] = bytes
                | (u32::from(info.packet_count & 0xfff) << 16)
                | ((vrt >> 30) & 1) << 31
                | ((vrt >> 20) & 1) << 29
                | ((vrt >> 24) & 1) << 28;
        }
        LinkType::Vrlp => {
            let total = body.len() + 3;
            let mut wrapped = Vec::with_capacity(total);
            wrapped.push(VRLP);
            wrapped.push((total as u32 & 0xfffff) | (u32::from(info.packet_count & 0xfff) << 20));
            wrapped.extend(body);
            wrapped.push(VEND);
            body = wrapped;
        }
    }
    debug_assert!(header_words <= 7);
    Ok(body
        .into_iter()
        .flat_map(|w| encode_word(w, endian))
        .collect())
}

/// Decode one complete legacy interface packet.
pub fn unpack(bytes: &[u8], link_type: LinkType, endian: Endian) -> Result<IfPacket, Error> {
    if bytes.len() < 4 || bytes.len() % 4 != 0 {
        return Err(Error("packet is not whole 32-bit words"));
    }
    let words: Vec<u32> = bytes
        .chunks_exact(4)
        .map(|b| decode_word(b, endian))
        .collect();
    let (body, mut count, outer_header) = match link_type {
        LinkType::Vrt | LinkType::Chdr => (&words[..], 0u16, 0usize),
        LinkType::Vrlp => {
            if words.len() < 4 || words[0] != VRLP {
                return Err(Error("bad VRLP header"));
            }
            let n = (words[1] & 0xfffff) as usize;
            if n > words.len() || n < 4 || words[n - 1] != VEND {
                return Err(Error("bad VRLP length or trailer"));
            }
            (&words[2..n - 1], (words[1] >> 20) as u16, 2)
        }
    };
    let mut vrt = body[0];
    let mut chdr_bytes = None;
    if link_type == LinkType::Chdr {
        let chdr = vrt;
        let nbytes = (chdr & 0xffff) as usize;
        if nbytes == 0 || nbytes > bytes.len() {
            return Err(Error("bad CHDR packet length"));
        }
        count = ((chdr >> 16) & 0xfff) as u16;
        vrt = nbytes.div_ceil(4) as u32
            | ((chdr >> 31) & 1) << 30
            | ((chdr >> 29) & 1) << 20
            | ((chdr >> 28) & 1) << 24
            | 1 << 28;
        chdr_bytes = Some(nbytes);
    }
    let packet_words = (vrt & 0xffff) as usize;
    if packet_words > body.len() || packet_words == 0 {
        return Err(Error("bad VRT length or packet fragment"));
    }
    if link_type == LinkType::Vrt {
        count = ((vrt >> 16) & 0xf) as u16;
    }
    let packet_type = match (vrt >> 29) & 7 {
        0 => PacketType::Data,
        1 => PacketType::IfExtension,
        2 => PacketType::Context,
        _ => return Err(Error("unsupported VRT packet type")),
    };
    let mut at = 1usize;
    let take = |at: &mut usize| -> Result<u32, Error> {
        let v = *body.get(*at).ok_or(Error("truncated VRT header"))?;
        *at += 1;
        Ok(v)
    };
    let sid = if vrt & (1 << 28) != 0 {
        Some(take(&mut at)?)
    } else {
        None
    };
    let cid = if vrt & (1 << 27) != 0 {
        Some((u64::from(take(&mut at)?) << 32) | u64::from(take(&mut at)?))
    } else {
        None
    };
    let tsi = if vrt & (3 << 22) != 0 {
        Some(take(&mut at)?)
    } else {
        None
    };
    let tsf = if vrt & (3 << 20) != 0 {
        Some((u64::from(take(&mut at)?) << 32) | u64::from(take(&mut at)?))
    } else {
        None
    };
    let has_tlr = vrt & (1 << 26) != 0;
    if at + usize::from(has_tlr) > packet_words {
        return Err(Error("invalid VRT header length"));
    }
    let trailer = has_tlr.then(|| body[packet_words - 1]);
    let end = packet_words - usize::from(has_tlr);
    let payload = body[at..end].to_vec();
    let empty = trailer.map_or(0, |t| {
        let indicators = (t >> 20) & (t >> 8);
        OCC[((indicators >> 2) & 3) as usize] as usize
    });
    let payload_bytes = if let Some(n) = chdr_bytes {
        let overhead = at * 4 + usize::from(has_tlr) * 4;
        if n < overhead {
            return Err(Error("CHDR byte length is shorter than its header"));
        }
        n - overhead
    } else {
        payload.len() * 4 - empty
    };
    let indicators = trailer.map_or(0, |t| (t >> 20) & (t >> 8));
    Ok(IfPacket {
        info: IfPacketInfo {
            link_type,
            packet_type,
            packet_count: count,
            sob: vrt & (1 << 25) != 0 || indicators & 2 != 0,
            eob: vrt & (1 << 24) != 0 || indicators & 1 != 0,
            sid,
            cid,
            tsi,
            tsf,
            trailer,
            payload_bytes,
        },
        payload,
        header_words: at + outer_header,
        packet_words: packet_words + if link_type == LinkType::Vrlp { 3 } else { 0 },
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn b200_control_vector_from_vrt_cpp_layout() {
        let info = IfPacketInfo {
            link_type: LinkType::Chdr,
            packet_type: PacketType::Context,
            packet_count: 7,
            sid: Some(0x40),
            payload_bytes: 8,
            ..Default::default()
        };
        let bytes = pack(&info, &[32, 3], Endian::Little).unwrap();
        assert_eq!(
            bytes,
            [0x10, 0, 7, 0x80, 0x40, 0, 0, 0, 0x20, 0, 0, 0, 3, 0, 0, 0]
        );
        let got = unpack(&bytes, LinkType::Chdr, Endian::Little).unwrap();
        assert_eq!(got.info.packet_count, 7);
        assert_eq!(got.info.sid, Some(0x40));
        assert_eq!(got.payload, [32, 3]);
        assert_eq!(got.header_words, 2);
    }

    #[test]
    fn chdr_timestamp_and_partial_payload_vector() {
        let info = IfPacketInfo {
            link_type: LinkType::Chdr,
            packet_type: PacketType::Data,
            packet_count: 0xabc,
            eob: true,
            sid: Some(0x1234_5678),
            tsf: Some(0x1122_3344_5566_7788),
            payload_bytes: 3,
            ..Default::default()
        };
        let bytes = pack(&info, &[0x00cc_bbaa], Endian::Big).unwrap();
        assert_eq!(&bytes[..4], &0x3abc_0013u32.to_be_bytes());
        let got = unpack(&bytes, LinkType::Chdr, Endian::Big).unwrap();
        assert_eq!(got.info.tsf, info.tsf);
        assert_eq!(got.info.payload_bytes, 3);
        assert!(got.info.eob);
    }

    #[test]
    fn plain_vrt_optional_fields_and_trailer_round_trip() {
        let info = IfPacketInfo {
            packet_count: 0x15,
            sob: true,
            sid: Some(1),
            cid: Some(2),
            tsi: Some(3),
            tsf: Some(4),
            trailer: Some(0),
            payload_bytes: 5,
            ..Default::default()
        };
        let bytes = pack(&info, &[0x01020304, 5], Endian::Little).unwrap();
        // type=0, SID/CID/trailer/SOB, TSI=UTC, TSF=sample count, count=5, 9 words.
        assert_eq!(
            u32::from_le_bytes(bytes[..4].try_into().unwrap()),
            0x1ed5_000a
        );
        let got = unpack(&bytes, LinkType::Vrt, Endian::Little).unwrap();
        assert_eq!(got.info.packet_count, 5);
        assert_eq!(got.info.payload_bytes, 5);
        assert_eq!(got.payload, [0x01020304, 5]);
    }

    #[test]
    fn vrlp_documented_markers_and_twelve_bit_sequence() {
        let info = IfPacketInfo {
            link_type: LinkType::Vrlp,
            packet_count: 0xabc,
            sid: Some(9),
            payload_bytes: 4,
            ..Default::default()
        };
        let bytes = pack(&info, &[10], Endian::Big).unwrap();
        let words: Vec<_> = bytes
            .chunks_exact(4)
            .map(|b| u32::from_be_bytes(b.try_into().unwrap()))
            .collect();
        assert_eq!(words, [VRLP, 0xabc0_0006, 0x100c_0003, 9, 10, VEND]);
        assert_eq!(
            unpack(&bytes, LinkType::Vrlp, Endian::Big)
                .unwrap()
                .info
                .packet_count,
            0xabc
        );
    }
}
