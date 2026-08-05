//! Intel HEX parsing, as much of it as an FX3 firmware image uses.
//!
//! Mirrors `ihex_reader::read` in `host/lib/utils/ihex.cpp`. That reader is a
//! push parser — it calls a handler per record, and the B200 firmware load
//! binds that handler to a USB control write. This one produces a value
//! instead, because the parse and the transfers are worth testing apart: the
//! whole of `usrp_b200_fw.hex` can be run through [`HexImage::parse`] in CI
//! with no device attached.
//!
//! # The format
//!
//! One record per line, each `:LLAAAATTDD…DDCC` in ASCII hex — length, 16-bit
//! address, type, `LL` data bytes, and a checksum that makes the whole record
//! sum to zero mod 256. Four record types matter here:
//!
//! | Type | Name | What it does |
//! | --- | --- | --- |
//! | `00` | Data | `LL` bytes to be written at the record's address |
//! | `01` | End of file | terminates the file; must be empty and at address 0 |
//! | `04` | Extended linear address | sets the upper 16 bits of the address for every later data record |
//! | `05` | Start linear address | the 32-bit entry point to jump to once loading finishes |
//!
//! Type `04` is what makes 32-bit addressing possible at all — a data record's
//! own address field is only 16 bits, and most of FX3 RAM is at `0x4000_0000`.
//!
//! `usrp_b200_fw.hex` is 11 522 records and uses that in earnest. The first 595
//! data records have no `04` before them, so they load at `0x0000_0100`
//! upwards — the FX3's tightly-coupled instruction memory, where the exception
//! vectors and the hot path live. Four `04` records then walk the high half
//! from `0x4000` to `0x4003`, placing the remaining ~180 KB in system RAM. A
//! parser that ignored type `04`, or that assumed a single one at the top of
//! the file, would put four fifths of the firmware at the wrong address, and
//! the device would simply never come back.
//!
//! # What this does not implement
//!
//! Types `02` and `03`, the 8086 segment-addressing records, are rejected
//! rather than ignored — same as UHD, and for the same reason: an image using
//! them is not an image for this device, and silently dropping the segment base
//! would write every record to the wrong address.

use uhd_core::{Error, Result};

/// A run of bytes and the address to write them at.
///
/// The address is the full 32 bits: the record's own 16-bit field in the low
/// half, and whatever the last type `04` record set in the high half. The FX3
/// loader splits it apart again — `wValue` gets the low half and `wIndex` the
/// high half, which is exactly how UHD's handler binding passes them.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct HexRecord {
    /// Where the data goes.
    pub address: u32,
    /// The data itself. Never empty: a zero-length data record carries no
    /// information and is dropped during parsing.
    pub data: Vec<u8>,
}

impl HexRecord {
    /// Low 16 bits of the address, which the FX3 protocol sends as `wValue`.
    #[must_use]
    pub fn value(&self) -> u16 {
        self.address as u16
    }

    /// High 16 bits of the address, which the FX3 protocol sends as `wIndex`.
    #[must_use]
    pub fn index(&self) -> u16 {
        (self.address >> 16) as u16
    }
}

/// A parsed Intel HEX file.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct HexImage {
    /// Data records, in file order. The FX3 boot ROM has no notion of a
    /// transaction, so order is the load order and is not to be sorted.
    pub records: Vec<HexRecord>,
    /// Entry point from the type `05` record, if the file had one.
    ///
    /// `None` is legal Intel HEX and is what a data-only image looks like, but
    /// it is not loadable onto an FX3: without an entry point there is nothing
    /// to tell the core to start running. [`HexImage::entry_point`] is the
    /// checked accessor the loader uses.
    pub entry: Option<u32>,
}

impl HexImage {
    /// Parses a whole Intel HEX file.
    ///
    /// Records are split on ASCII whitespace rather than on line endings, which
    /// is what UHD's `file >> record` does — so CRLF, stray blank lines and a
    /// missing final newline are all tolerated.
    ///
    /// # Errors
    ///
    /// [`Error::Firmware`] for a malformed record, a bad checksum, an
    /// unsupported record type, a data record after the entry point, or a file
    /// with no end-of-file record. Every message names the line it came from,
    /// because a truncated or line-ending-mangled image is the usual cause and
    /// the line number is what identifies it.
    pub fn parse(text: &str) -> Result<Self> {
        let mut records = Vec::new();
        let mut entry = None;
        let mut upper = 0u32;

        for (line_no, line) in text.split_ascii_whitespace().enumerate() {
            let record = Record::parse(line)
                .map_err(|e| Error::Firmware(format!("intel hex record {line_no}: {e}")))?;

            match record.kind {
                // Data.
                0x00 => {
                    if entry.is_some() {
                        return Err(Error::Firmware(format!(
                            "intel hex record {line_no}: data record after the start \
                             linear address record; the entry point must come last"
                        )));
                    }
                    if !record.data.is_empty() {
                        records.push(HexRecord {
                            address: upper << 16 | u32::from(record.address),
                            data: record.data,
                        });
                    }
                }

                // End of file.
                0x01 => {
                    if record.address != 0 || !record.data.is_empty() {
                        return Err(Error::Firmware(format!(
                            "intel hex record {line_no}: end-of-file record must have \
                             address 0 and length 0"
                        )));
                    }
                    return Ok(Self { records, entry });
                }

                // Extended linear address: the high half of every later address.
                0x04 => {
                    let [hi, lo] = record.data[..] else {
                        return Err(Error::Firmware(format!(
                            "intel hex record {line_no}: extended linear address record \
                             must have length 2, got {}",
                            record.data.len()
                        )));
                    };
                    if record.address != 0 {
                        return Err(Error::Firmware(format!(
                            "intel hex record {line_no}: extended linear address record \
                             must have address 0"
                        )));
                    }
                    upper = u32::from(u16::from_be_bytes([hi, lo]));
                }

                // Start linear address: the entry point.
                0x05 => {
                    let [a, b, c, d] = record.data[..] else {
                        return Err(Error::Firmware(format!(
                            "intel hex record {line_no}: start linear address record must \
                             have length 4, got {}",
                            record.data.len()
                        )));
                    };
                    if record.address != 0 {
                        return Err(Error::Firmware(format!(
                            "intel hex record {line_no}: start linear address record must \
                             have address 0"
                        )));
                    }
                    entry = Some(u32::from_be_bytes([a, b, c, d]));
                }

                kind => {
                    return Err(Error::Firmware(format!(
                        "intel hex record {line_no}: unsupported record type {kind:#04x}"
                    )));
                }
            }
        }

        Err(Error::Firmware(
            "intel hex: no end-of-file record; the image is truncated".into(),
        ))
    }

    /// Reads and parses an Intel HEX file from disk.
    ///
    /// # Errors
    ///
    /// [`Error::Firmware`] if the file cannot be read or does not parse. The
    /// path is folded into the message — a firmware load that fails needs to
    /// say *which* image failed, and the caller has usually resolved the path
    /// out of `UHD_IMAGES_DIR` rather than typed it.
    pub fn load(path: &std::path::Path) -> Result<Self> {
        let text = std::fs::read_to_string(path)
            .map_err(|e| Error::Firmware(format!("cannot read {}: {e}", path.display())))?;
        Self::parse(&text)
            .map_err(|e| Error::Firmware(format!("cannot parse {}: {e}", path.display())))
    }

    /// The entry point, as a firmware image must have one.
    ///
    /// # Errors
    ///
    /// [`Error::Firmware`] if the image had no type `05` record.
    pub fn entry_point(&self) -> Result<u32> {
        self.entry.ok_or_else(|| {
            Error::Firmware(
                "intel hex: no start linear address record, so the image has no entry \
                 point and cannot be booted"
                    .into(),
            )
        })
    }

    /// Total number of data bytes across every record — the size of the image
    /// that will actually be pushed over the wire.
    #[must_use]
    pub fn payload_len(&self) -> usize {
        self.records.iter().map(|r| r.data.len()).sum()
    }
}

/// One record, straight out of the file and not yet interpreted.
struct Record {
    kind: u8,
    address: u16,
    data: Vec<u8>,
}

impl Record {
    /// Parses a single `:LLAAAATTDD…CC` record.
    ///
    /// The checksum is verified the way UHD verifies it — sum every byte of the
    /// record including the checksum itself and require zero — which catches a
    /// wrong checksum and a truncated record with one test.
    fn parse(line: &str) -> std::result::Result<Self, String> {
        let body = line
            .strip_prefix(':')
            .ok_or_else(|| format!("does not start with ':': {line:?}"))?;

        if body.len() % 2 != 0 {
            return Err(format!("odd number of hex digits ({})", body.len()));
        }
        // Length, address, type and checksum: 5 bytes before any data.
        if body.len() < 10 {
            return Err(format!("too short to be a record ({} digits)", body.len()));
        }

        let bytes: Vec<u8> = (0..body.len() / 2)
            .map(|i| {
                u8::from_str_radix(&body[2 * i..2 * i + 2], 16)
                    .map_err(|_| format!("`{}` is not a hex byte", &body[2 * i..2 * i + 2]))
            })
            .collect::<std::result::Result<_, _>>()?;

        if bytes.iter().fold(0u8, |sum, b| sum.wrapping_add(*b)) != 0 {
            return Err("bad checksum".into());
        }

        let len = usize::from(bytes[0]);
        if bytes.len() != len + 5 {
            return Err(format!(
                "declares {len} data bytes but carries {}",
                bytes.len().saturating_sub(5)
            ));
        }

        Ok(Self {
            kind: bytes[3],
            address: u16::from_be_bytes([bytes[1], bytes[2]]),
            data: bytes[4..4 + len].to_vec(),
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Builds a record with a correct checksum, so the tests can state what
    /// they mean rather than what the arithmetic came out to.
    fn record(kind: u8, address: u16, data: &[u8]) -> String {
        let mut bytes = vec![data.len() as u8];
        bytes.extend_from_slice(&address.to_be_bytes());
        bytes.push(kind);
        bytes.extend_from_slice(data);
        let sum = bytes.iter().fold(0u8, |sum, b| sum.wrapping_add(*b));
        bytes.push(sum.wrapping_neg());
        let mut out = String::from(":");
        for b in bytes {
            out.push_str(&format!("{b:02X}"));
        }
        out
    }

    const EOF: &str = ":00000001FF";

    #[test]
    fn parses_a_minimal_image() {
        let text = format!(
            "{}\n{}\n{EOF}\n",
            record(0x00, 0x0100, &[0xde, 0xad, 0xbe, 0xef]),
            record(0x05, 0x0000, &[0x40, 0x01, 0x9d, 0xec]),
        );
        let image = HexImage::parse(&text).expect("parses");
        assert_eq!(
            image.records,
            vec![HexRecord {
                address: 0x0000_0100,
                data: vec![0xde, 0xad, 0xbe, 0xef],
            }]
        );
        assert_eq!(image.entry, Some(0x4001_9dec));
        assert_eq!(image.entry_point().unwrap(), 0x4001_9dec);
        assert_eq!(image.payload_len(), 4);
    }

    #[test]
    fn extended_linear_address_sets_the_high_half() {
        // This is the whole reason FX3 images parse at all: the data records
        // say 0x0100, and RAM is at 0x40000000.
        let text = format!(
            "{}\n{}\n{}\n{EOF}\n",
            record(0x00, 0x0100, &[0x11]),
            record(0x04, 0x0000, &[0x40, 0x00]),
            record(0x00, 0x0100, &[0x22]),
        );
        let image = HexImage::parse(&text).expect("parses");
        assert_eq!(image.records[0].address, 0x0000_0100);
        assert_eq!(image.records[1].address, 0x4000_0100);
        // And that is what the control transfer's value/index carry.
        assert_eq!(image.records[1].value(), 0x0100);
        assert_eq!(image.records[1].index(), 0x4000);
    }

    #[test]
    fn a_later_ela_record_replaces_the_earlier_one() {
        let text = format!(
            "{}\n{}\n{}\n{}\n{EOF}\n",
            record(0x04, 0x0000, &[0x40, 0x00]),
            record(0x00, 0x0000, &[0x11]),
            record(0x04, 0x0000, &[0x40, 0x01]),
            record(0x00, 0x0000, &[0x22]),
        );
        let image = HexImage::parse(&text).expect("parses");
        assert_eq!(image.records[0].address, 0x4000_0000);
        assert_eq!(image.records[1].address, 0x4001_0000);
    }

    #[test]
    fn a_bad_checksum_is_rejected() {
        let mut good = record(0x00, 0x0100, &[0x01, 0x02]);
        good.pop();
        good.pop();
        good.push_str("00");
        let text = format!("{good}\n{EOF}\n");
        let e = HexImage::parse(&text).expect_err("bad checksum");
        assert!(e.to_string().contains("bad checksum"), "{e}");
    }

    #[test]
    fn a_truncated_record_is_rejected() {
        // Declares four data bytes, carries two. The checksum test alone would
        // not catch this, which is why the length is checked separately.
        let text = format!(":04010000{:02X}\n{EOF}\n", 0xfbu8);
        let e = HexImage::parse(&text).expect_err("truncated");
        assert!(e.to_string().contains("record 0"), "{e}");
    }

    #[test]
    fn a_missing_eof_record_is_rejected() {
        // The failure mode this catches is a half-written or half-downloaded
        // image, which otherwise loads most of the firmware and then hangs.
        let text = format!("{}\n", record(0x00, 0x0000, &[0x01]));
        let e = HexImage::parse(&text).expect_err("no eof");
        assert!(e.to_string().contains("truncated"), "{e}");
    }

    #[test]
    fn segment_address_records_are_rejected_not_ignored() {
        let text = format!("{}\n{EOF}\n", record(0x02, 0x0000, &[0x12, 0x34]));
        let e = HexImage::parse(&text).expect_err("unsupported type");
        assert!(
            e.to_string().contains("unsupported record type 0x02"),
            "{e}"
        );
    }

    #[test]
    fn data_after_the_entry_point_is_rejected() {
        // The loader emits the entry-point transfer last, which is only
        // equivalent to UHD's in-order push if nothing follows it.
        let text = format!(
            "{}\n{}\n{EOF}\n",
            record(0x05, 0x0000, &[0x40, 0x01, 0x9d, 0xec]),
            record(0x00, 0x0100, &[0x01]),
        );
        let e = HexImage::parse(&text).expect_err("data after entry");
        assert!(e.to_string().contains("entry point must come last"), "{e}");
    }

    #[test]
    fn an_image_without_an_entry_point_parses_but_cannot_boot() {
        let text = format!("{}\n{EOF}\n", record(0x00, 0x0000, &[0x01]));
        let image = HexImage::parse(&text).expect("parses");
        assert_eq!(image.entry, None);
        assert!(image.entry_point().is_err());
    }

    #[test]
    fn whitespace_and_line_endings_do_not_matter() {
        let text = format!(
            "\r\n  {}\r\n\r\n{}\r\n{EOF}\r\n",
            record(0x00, 0x0000, &[0x01]),
            record(0x00, 0x0002, &[0x02]),
        );
        let image = HexImage::parse(&text).expect("parses");
        assert_eq!(image.records.len(), 2);
    }

    #[test]
    fn trailing_records_after_eof_are_ignored() {
        // Matching UHD, which returns the moment it sees the EOF record.
        let text = format!("{}\n{EOF}\ngarbage\n", record(0x00, 0x0000, &[0x01]));
        assert_eq!(HexImage::parse(&text).expect("parses").records.len(), 1);
    }

    /// The real thing, when it is installed. Skipped rather than failed in CI,
    /// where there is no images package — but on a developer machine this is
    /// the test that matters, because it is the only one running against bytes
    /// nobody here wrote.
    #[test]
    fn the_real_firmware_image_parses() {
        let Ok(path) = crate::images::find_image(crate::usb::B200_FW_IMAGE) else {
            eprintln!("skipping: {} is not installed", crate::usb::B200_FW_IMAGE);
            return;
        };
        let image = HexImage::load(&path).expect("the shipped firmware image parses");

        // The FX3 memory map, and the reason type 04 records have to be
        // handled rather than skipped: the image straddles both regions.
        const ITCM: std::ops::Range<u32> = 0x0000_0000..0x0001_0000;
        const SYSMEM: std::ops::Range<u32> = 0x4000_0000..0x4008_0000;

        assert!(!image.records.is_empty());
        for r in &image.records {
            assert!(
                ITCM.contains(&r.address) || SYSMEM.contains(&r.address),
                "record at {:#010x} is in neither i-tcm nor system ram",
                r.address
            );
            assert!(r.data.len() <= 255);
        }
        assert!(
            image.records.iter().any(|r| ITCM.contains(&r.address)),
            "expected part of the image in i-tcm"
        );
        assert!(
            image.records.iter().any(|r| SYSMEM.contains(&r.address)),
            "expected part of the image in system ram"
        );

        // Execution starts in system RAM, not in the low block.
        let entry = image.entry_point().expect("firmware has an entry point");
        assert!(SYSMEM.contains(&entry), "{entry:#010x}");
        assert!(image.payload_len() > 64 * 1024, "{}", image.payload_len());
    }
}
