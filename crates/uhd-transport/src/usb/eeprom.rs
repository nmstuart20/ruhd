//! The motherboard EEPROM map: what the board says it is.
//!
//! Mirrors `b200_mb_eeprom.cpp`. The FX3 reaches the EEPROM over I²C on the
//! board's behalf, so reading it is just [`Fx3Iface::read_eeprom`] against the
//! right addresses — but *which* addresses depends on a signature word, because
//! two incompatible layouts are in the field.
//!
//! # Why this matters before anything else works
//!
//! This is the only place a B200 and a B210 can be told apart: they share USB
//! product ID `0x0020`, and the FPGA image, the channel count and the frontend
//! mapping all differ between them. So the EEPROM read sits on the critical
//! path between "firmware is running" and "the right bitstream is loaded".
//!
//! # Two layouts
//!
//! A 32-bit signature at EEPROM address `0x0000` says which:
//!
//! | Signature | Layout | Data at | Length |
//! | --- | --- | --- | ---: |
//! | `0xb214_5943` | [`Rev0`](EepromLayout::Rev0) | `0x04dc` | 36 |
//! | `0xb01a_5943` | [`Rev1`](EepromLayout::Rev1) | `0x7f00` | 46 |
//!
//! Rev 1 moved the data to the top of the EEPROM to make room for the
//! second-stage bootloader image, and prepended a ten-byte header — magic,
//! layout revision, layout compatibility number, USB VID and PID — in front of
//! the rev 0 fields, which are otherwise unchanged and in the same order. That
//! is why `b2xx_fx3_utils`'s upgrade path is a straight copy of 36 bytes from
//! one address to the other.
//!
//! # Divergence from UHD
//!
//! `get_mb_eeprom` throws if the rev 1 magic or compatibility number is not
//! what it expects, from inside device construction. Here those are reported —
//! see [`MbEeprom::problems`] — and the decision is the caller's, which is the
//! same split [`CompatReport`](super::compat::CompatReport) makes for the
//! firmware and FPGA numbers. A board with a slightly-off EEPROM header is
//! still a board you want to be able to look at, and refusing to read it is a
//! poor way to help someone fix it.

use tracing::{debug, warn};
use uhd_core::Result;

use super::fx3::Fx3Iface;
use super::ids::B2xxProduct;

/// EEPROM address of the layout signature.
pub const SIGNATURE_ADDR: u16 = 0x0000;

/// Length of the layout signature, in bytes.
pub const SIGNATURE_LEN: u16 = 4;

/// Signature of the original layout. `REV0_SIGNATURE` in `b200_mb_eeprom.cpp`.
pub const REV0_SIGNATURE: u32 = 0xb214_5943;

/// Signature of the layout that makes room for the bootloader.
/// `REV1_SIGNATURE` in `b200_mb_eeprom.cpp`.
pub const REV1_SIGNATURE: u32 = 0xb01a_5943;

/// Where the rev 0 data block starts.
pub const REV0_BASE_ADDR: u16 = 0x04dc;

/// How long the rev 0 data block is.
pub const REV0_LEN: u16 = 36;

/// Where the rev 1 data block starts.
pub const REV1_BASE_ADDR: u16 = 0x7f00;

/// How long the rev 1 data block is.
pub const REV1_LEN: u16 = 46;

/// Longest board name the map has room for.
///
/// `NAME_MAX_LEN` in `eeprom_utils.hpp`, which is `32 - SERIAL_LEN`: the name
/// and serial share a 32-byte field between them.
pub const NAME_MAX_LEN: usize = 32 - SERIAL_LEN;

/// Length of the serial field. `SERIAL_LEN` in `eeprom_utils.hpp`.
pub const SERIAL_LEN: usize = 9;

/// The value the rev 1 `magic` field must hold: `0xb200`, spelled in decimal
/// (`"45568"`) in UHD because the field goes through a string.
pub const EXPECTED_MAGIC: u16 = 0xb200;

/// The layout compatibility number this driver understands.
/// `EXPECTED_COMPAT` in `b200_mb_eeprom.cpp`.
pub const EXPECTED_EEPROM_COMPAT: u16 = 1;

/// Which of the two EEPROM layouts a board uses.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum EepromLayout {
    /// The original layout: 36 bytes at `0x04dc`, no header.
    Rev0,
    /// The current layout: 46 bytes at `0x7f00`, with a ten-byte header in
    /// front of the same fields.
    Rev1,
}

impl EepromLayout {
    /// Identifies a layout from the signature word at [`SIGNATURE_ADDR`].
    ///
    /// `None` for anything else, which is what `_get_rev`'s `UNKNOWN_REV` is:
    /// a board whose EEPROM has never been written, or has been written by
    /// something else.
    #[must_use]
    pub fn from_signature(signature: u32) -> Option<Self> {
        match signature {
            REV0_SIGNATURE => Some(Self::Rev0),
            REV1_SIGNATURE => Some(Self::Rev1),
            _ => None,
        }
    }

    /// Where this layout's data block starts, and how long it is.
    #[must_use]
    pub fn extent(self) -> (u16, u16) {
        match self {
            Self::Rev0 => (REV0_BASE_ADDR, REV0_LEN),
            Self::Rev1 => (REV1_BASE_ADDR, REV1_LEN),
        }
    }

    /// How far into the block the fields rev 0 and rev 1 share begin.
    ///
    /// Ten: the rev 1 header (magic, revision, compat, VID, PID) sits in front
    /// of an otherwise identical block.
    #[must_use]
    pub fn field_base(self) -> usize {
        match self {
            Self::Rev0 => 0,
            Self::Rev1 => 10,
        }
    }
}

impl std::fmt::Display for EepromLayout {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::Rev0 => f.write_str("rev 0"),
            Self::Rev1 => f.write_str("rev 1"),
        }
    }
}

/// The motherboard EEPROM's contents, decoded.
///
/// Every field is optional because every field can be unwritten, and an unset
/// field is not an error — UHD's `mboard_eeprom_t` says the same thing with an
/// empty string. Fields that only exist in [`Rev1`](EepromLayout::Rev1) are
/// `None` on a rev 0 board.
#[derive(Debug, Clone, Default, PartialEq, Eq)]
#[non_exhaustive]
pub struct MbEeprom {
    /// The signature word, whether or not it was recognised.
    pub signature: u32,
    /// Which layout the signature named. `None` means the block was not read
    /// at all and every field below is `None`.
    pub layout: Option<EepromLayout>,

    /// Rev 1 only: `0xb200`, if the header is intact.
    pub magic: Option<u16>,
    /// Rev 1 only: the layout revision, which is `1` on every board in the
    /// field. Distinct from [`revision`](Self::revision), which is the board's.
    pub eeprom_revision: Option<u16>,
    /// Rev 1 only: the layout compatibility number, expected to be
    /// [`EXPECTED_EEPROM_COMPAT`].
    pub eeprom_compat: Option<u16>,
    /// Rev 1 only: the USB vendor ID the FX3 enumerates with.
    pub vendor_id: Option<u16>,
    /// Rev 1 only: the USB product ID the FX3 enumerates with.
    pub product_id: Option<u16>,

    /// The board's hardware revision. This is the one that changes behaviour:
    /// a B200 at revision 5 or above carries an AD9364 and maps its frontends
    /// differently from an earlier one.
    pub revision: Option<u16>,
    /// The product code — see [`B2xxProduct::from_eeprom_id`]. This is what
    /// resolves a B200 from a B210.
    pub product_code: Option<u16>,
    /// The board name, which is a user-settable label rather than an identity.
    pub name: Option<String>,
    /// The USRP serial. This is the one the firmware reports in its USB
    /// descriptors once it is running.
    pub serial: Option<String>,

    /// The data block exactly as read, for anything this struct does not
    /// model and for diagnosing a board that decodes to nonsense.
    pub raw: Vec<u8>,
}

impl MbEeprom {
    /// Which B2xx the product code names, if this driver knows it.
    #[must_use]
    pub fn product(&self) -> Option<B2xxProduct> {
        self.product_code.and_then(B2xxProduct::from_eeprom_id)
    }

    /// Everything wrong with this EEPROM, in plain words. Empty means fine.
    ///
    /// These are the conditions `get_mb_eeprom` throws on, plus the unreadable
    /// signature it only warns about. Returned rather than raised so that a
    /// caller can print them next to the fields they impugn — a board with a
    /// bad magic value still has a serial worth showing, and the person reading
    /// the output is the one who has to decide whether to reprogram it.
    #[must_use]
    pub fn problems(&self) -> Vec<String> {
        let mut problems = Vec::new();

        let Some(layout) = self.layout else {
            problems.push(format!(
                "the eeprom signature is {:#010x}, which is neither the rev 0 \
                 ({REV0_SIGNATURE:#010x}) nor the rev 1 ({REV1_SIGNATURE:#010x}) value; \
                 the eeprom has not been programmed",
                self.signature
            ));
            return problems;
        };

        if layout == EepromLayout::Rev1 {
            if self.magic != Some(EXPECTED_MAGIC) {
                problems.push(format!(
                    "the eeprom magic value is {}, not {EXPECTED_MAGIC:#06x}",
                    describe(self.magic)
                ));
            }
            if self.eeprom_compat != Some(EXPECTED_EEPROM_COMPAT) {
                problems.push(format!(
                    "the eeprom layout compatibility number is {}, not \
                     {EXPECTED_EEPROM_COMPAT}",
                    describe(self.eeprom_compat)
                ));
            }
        }

        if self.product_code.is_none() {
            problems.push("no product code is set, so the board model is unknown".into());
        } else if self.product().is_none() {
            problems.push(format!(
                "the product code is {:#06x}, which this driver does not recognise",
                self.product_code.unwrap_or_default()
            ));
        }

        if self.serial.is_none() {
            problems.push("no serial is set".into());
        }

        problems
    }

    /// Whether the EEPROM decoded cleanly. Shorthand for `problems().is_empty()`.
    #[must_use]
    pub fn is_valid(&self) -> bool {
        self.problems().is_empty()
    }
}

/// Renders an optional field for a message.
fn describe(value: Option<u16>) -> String {
    match value {
        Some(value) => format!("{value:#06x}"),
        None => "unset".to_owned(),
    }
}

/// Reads and decodes the motherboard EEPROM.
///
/// Two control transfers: the signature, then the data block the signature
/// points at. `get_mb_eeprom` does three, because it reads the signature twice.
///
/// An unrecognised signature is not an error — the returned [`MbEeprom`] has
/// `layout: None` and says so through [`MbEeprom::problems`], which is what
/// `_get_rev`'s warning-and-empty-map does, minus the part where the caller
/// cannot tell the difference between "unprogrammed" and "all fields blank".
///
/// # Errors
///
/// [`Error::Usb`](uhd_core::Error::Usb) if either transfer fails. Only
/// firmware answers these requests; against a device that has not been
/// initialised, the endpoint stalls.
pub fn read(iface: &Fx3Iface) -> Result<MbEeprom> {
    let bytes = iface.read_eeprom(
        (SIGNATURE_ADDR >> 8) as u8,
        (SIGNATURE_ADDR & 0xff) as u8,
        SIGNATURE_LEN,
    )?;
    let signature = u32::from_le_bytes([bytes[0], bytes[1], bytes[2], bytes[3]]);

    let Some(layout) = EepromLayout::from_signature(signature) else {
        warn!(
            signature = format_args!("{signature:#010x}"),
            "unrecognised eeprom signature; the eeprom has not been programmed"
        );
        return Ok(MbEeprom {
            signature,
            ..MbEeprom::default()
        });
    };

    let (base, len) = layout.extent();
    let raw = iface.read_eeprom((base >> 8) as u8, (base & 0xff) as u8, len)?;
    let eeprom = decode(signature, layout, raw);
    debug!(
        layout = %layout,
        serial = ?eeprom.serial,
        product = ?eeprom.product_code,
        revision = ?eeprom.revision,
        "read motherboard eeprom"
    );
    Ok(eeprom)
}

/// Decodes a data block that has already been read.
///
/// Split out from [`read`] because it is the whole of the format knowledge and
/// none of the I/O, so it can be tested against a byte block captured from a
/// real board.
///
/// A block shorter than the layout calls for decodes as far as it can and
/// leaves the rest `None`, rather than panicking: a short read is already an
/// error at the transfer layer, and this being total means a truncated capture
/// in a test is still readable.
#[must_use]
pub fn decode(signature: u32, layout: EepromLayout, raw: Vec<u8>) -> MbEeprom {
    let base = layout.field_base();
    let header = layout == EepromLayout::Rev1;

    MbEeprom {
        signature,
        layout: Some(layout),
        magic: header.then(|| u16_at(&raw, 0)).flatten(),
        eeprom_revision: header.then(|| u16_at(&raw, 2)).flatten(),
        eeprom_compat: header.then(|| u16_at(&raw, 4)).flatten(),
        vendor_id: header.then(|| u16_at(&raw, 6)).flatten(),
        product_id: header.then(|| u16_at(&raw, 8)).flatten(),
        revision: u16_at(&raw, base),
        product_code: u16_at(&raw, base + 2),
        name: string_at(&raw, base + 4, NAME_MAX_LEN),
        serial: string_at(&raw, base + 4 + NAME_MAX_LEN, SERIAL_LEN),
        raw,
    }
}

/// A two-byte little-endian field, `uint16_bytes_to_string` in
/// `eeprom_utils.cpp`.
///
/// Both `0x0000` and `0xffff` mean "unset": zero because that is what a written
/// but unused field looks like, and `0xffff` because that is what erased flash
/// reads back as. UHD returns an empty string for both, which callers then
/// test with `.empty()`; `None` says the same thing in a way that cannot be
/// confused with a legitimately empty value.
fn u16_at(raw: &[u8], offset: usize) -> Option<u16> {
    let bytes = raw.get(offset..offset + 2)?;
    match u16::from_le_bytes([bytes[0], bytes[1]]) {
        0 | 0xffff => None,
        value => Some(value),
    }
}

/// A fixed-width string field, `bytes_to_string` in `byte_vector.cpp`.
///
/// Stops at the first byte outside printable ASCII, which is how the NUL
/// terminator ends a short name — and also how a garbled field stops early
/// rather than producing mojibake. `None` for a field that is empty by that
/// rule.
fn string_at(raw: &[u8], offset: usize, len: usize) -> Option<String> {
    let bytes = raw.get(offset..offset + len)?;
    let text: String = bytes
        .iter()
        .take_while(|&&b| (32..=127).contains(&b))
        .map(|&b| b as char)
        .collect();
    (!text.is_empty()).then_some(text)
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A rev 1 block as a B205mini serves it: the ten-byte header, then the
    /// rev 0 fields.
    fn rev1_block() -> Vec<u8> {
        let mut raw = vec![0u8; REV1_LEN as usize];
        raw[0..2].copy_from_slice(&EXPECTED_MAGIC.to_le_bytes()); // magic
        raw[2..4].copy_from_slice(&1u16.to_le_bytes()); // eeprom_revision
        raw[4..6].copy_from_slice(&1u16.to_le_bytes()); // eeprom_compat
        raw[6..8].copy_from_slice(&0x2500u16.to_le_bytes()); // vendor_id
        raw[8..10].copy_from_slice(&0x0022u16.to_le_bytes()); // product_id
        raw[10..12].copy_from_slice(&2u16.to_le_bytes()); // revision
        raw[12..14].copy_from_slice(&0x0004u16.to_le_bytes()); // product
        raw[14..14 + 5].copy_from_slice(b"bench"); // name, NUL padded
        raw[37..37 + 7].copy_from_slice(b"3168B41"); // serial, NUL padded
        raw
    }

    #[test]
    fn a_rev1_board_decodes_to_its_identity() {
        let eeprom = decode(REV1_SIGNATURE, EepromLayout::Rev1, rev1_block());
        assert_eq!(eeprom.layout, Some(EepromLayout::Rev1));
        assert_eq!(eeprom.magic, Some(0xb200));
        assert_eq!(eeprom.eeprom_revision, Some(1));
        assert_eq!(eeprom.eeprom_compat, Some(1));
        assert_eq!(eeprom.vendor_id, Some(0x2500));
        assert_eq!(eeprom.product_id, Some(0x0022));
        assert_eq!(eeprom.revision, Some(2));
        assert_eq!(eeprom.product_code, Some(0x0004));
        assert_eq!(eeprom.product(), Some(B2xxProduct::B205Mini));
        assert_eq!(eeprom.name.as_deref(), Some("bench"));
        assert_eq!(eeprom.serial.as_deref(), Some("3168B41"));
        assert!(eeprom.is_valid(), "{:?}", eeprom.problems());
    }

    #[test]
    fn rev0_holds_the_same_fields_ten_bytes_earlier() {
        // The upgrade path in b2xx_fx3_utils is a straight copy of these 36
        // bytes to offset 10 of the rev 1 block, so decoding one from the
        // other's tail is exactly the compatibility claim being made.
        let raw = rev1_block()[10..].to_vec();
        assert_eq!(raw.len(), REV0_LEN as usize);

        let rev0 = decode(REV0_SIGNATURE, EepromLayout::Rev0, raw);
        let rev1 = decode(REV1_SIGNATURE, EepromLayout::Rev1, rev1_block());
        assert_eq!(rev0.serial, rev1.serial);
        assert_eq!(rev0.name, rev1.name);
        assert_eq!(rev0.product_code, rev1.product_code);
        assert_eq!(rev0.revision, rev1.revision);

        // And the header fields simply do not exist on a rev 0 board.
        assert_eq!(rev0.magic, None);
        assert_eq!(rev0.vendor_id, None);
        assert!(rev0.is_valid(), "{:?}", rev0.problems());
    }

    #[test]
    fn signatures_match_the_reference() {
        // OLD_/NEW_EEPROM_SIGNATURE in b2xx_fx3_utils.cpp, as byte vectors.
        assert_eq!(u32::from_le_bytes([0x43, 0x59, 0x14, 0xb2]), REV0_SIGNATURE);
        assert_eq!(u32::from_le_bytes([0x43, 0x59, 0x1a, 0xb0]), REV1_SIGNATURE);
        assert_eq!(
            EepromLayout::from_signature(REV0_SIGNATURE),
            Some(EepromLayout::Rev0)
        );
        assert_eq!(
            EepromLayout::from_signature(REV1_SIGNATURE),
            Some(EepromLayout::Rev1)
        );
        assert_eq!(EepromLayout::from_signature(0), None);
    }

    #[test]
    fn the_field_map_lengths_add_up() {
        // name and serial share 32 bytes, and the block is exactly long
        // enough to hold everything the map claims.
        assert_eq!(NAME_MAX_LEN + SERIAL_LEN, 32);
        assert_eq!(REV0_LEN as usize, 4 + NAME_MAX_LEN + SERIAL_LEN);
        assert_eq!(REV1_LEN as usize, 10 + REV0_LEN as usize);
        assert_eq!(EepromLayout::Rev1.field_base(), 10);
    }

    #[test]
    fn unset_fields_read_as_none_both_ways_round() {
        // 0x0000 is a written-but-unused field; 0xffff is an erased one.
        let mut raw = rev1_block();
        raw[10..12].copy_from_slice(&0u16.to_le_bytes());
        raw[12..14].copy_from_slice(&0xffffu16.to_le_bytes());
        let eeprom = decode(REV1_SIGNATURE, EepromLayout::Rev1, raw);
        assert_eq!(eeprom.revision, None);
        assert_eq!(eeprom.product_code, None);
        assert_eq!(eeprom.product(), None);
    }

    #[test]
    fn a_string_field_stops_at_the_first_unprintable_byte() {
        let mut raw = rev1_block();
        raw[14..14 + 8].copy_from_slice(b"ab\0cdefg");
        let eeprom = decode(REV1_SIGNATURE, EepromLayout::Rev1, raw);
        assert_eq!(
            eeprom.name.as_deref(),
            Some("ab"),
            "the NUL ends the name; the bytes after it are padding, not text"
        );
    }

    #[test]
    fn an_unprogrammed_eeprom_is_reported_rather_than_raised() {
        let eeprom = MbEeprom {
            signature: 0xffff_ffff,
            ..MbEeprom::default()
        };
        assert!(!eeprom.is_valid());
        assert_eq!(eeprom.problems().len(), 1);
        assert!(
            eeprom.problems()[0].contains("has not been programmed"),
            "{:?}",
            eeprom.problems()
        );
    }

    #[test]
    fn a_bad_magic_is_a_problem_and_not_a_refusal_to_decode() {
        // UHD throws here, from inside device construction. The fields are
        // still readable and still worth showing to whoever has to fix it.
        let mut raw = rev1_block();
        raw[0..2].copy_from_slice(&0x1234u16.to_le_bytes());
        let eeprom = decode(REV1_SIGNATURE, EepromLayout::Rev1, raw);
        assert_eq!(eeprom.serial.as_deref(), Some("3168B41"));
        assert!(!eeprom.is_valid());
        assert!(
            eeprom.problems()[0].contains("magic"),
            "{:?}",
            eeprom.problems()
        );
    }

    #[test]
    fn an_unknown_product_code_is_reported_with_the_code() {
        let mut raw = rev1_block();
        raw[12..14].copy_from_slice(&0x0099u16.to_le_bytes());
        let eeprom = decode(REV1_SIGNATURE, EepromLayout::Rev1, raw);
        assert_eq!(eeprom.product(), None);
        assert!(
            eeprom.problems().iter().any(|p| p.contains("0x0099")),
            "{:?}",
            eeprom.problems()
        );
    }

    #[test]
    fn a_truncated_block_decodes_as_far_as_it_goes() {
        let raw = rev1_block()[..20].to_vec();
        let eeprom = decode(REV1_SIGNATURE, EepromLayout::Rev1, raw);
        assert_eq!(eeprom.product_code, Some(0x0004));
        assert_eq!(eeprom.serial, None, "the serial is past the end");
    }
}
