//! USB and EEPROM identifiers for the B2xx family.
//!
//! Every table here mirrors one in the UHD reference, so that a device this
//! driver refuses to recognise is a deliberate difference rather than a
//! transcription slip:
//!
//! | Here | UHD |
//! | --- | --- |
//! | the `*_VID` / `*_PID` constants | `B200_VENDOR_ID` … `FX3_REENUM_PID` in `b200_iface.hpp` |
//! | [`B2XX_VID_PIDS`] | `b200_vid_pid_pairs` in `b200_impl.hpp` |
//! | [`FX3_BOOTLOADER_VID_PIDS`] | the `FX3_VID` entries of `known_vid_pids` in `utils/b2xx_fx3_utils.cpp` |
//! | [`B2xxProduct::from_usb_id`] | `B2XX_PID_TO_PRODUCT` in `b200_iface.hpp` |
//! | [`B2xxProduct::from_eeprom_id`] | `B2XX_PRODUCT_ID` in `b200_iface.hpp` |
//! | [`B2xxProduct::name`] | `B2XX_STR_NAMES` in `b200_iface.hpp` |
//! | [`B2xxProduct::fpga_image`] | `B2XX_FPGA_FILE_NAME` in `b200_iface.hpp` |

/// Cypress vendor ID. An FX3 shows up under this VID until the EEPROM on the
/// board tells it to enumerate as something else.
pub const FX3_VID: u16 = 0x04b4;

/// Product ID of an FX3 running its ROM bootloader — a B2xx whose EEPROM has
/// never been programmed, or whose EEPROM has been erased.
///
/// `FX3_DEFAULT_PID` in `b200_iface.hpp`.
pub const FX3_BOOTLOADER_PID: u16 = 0x00f3;

/// Product ID an FX3 takes after the second-stage bootloader image
/// (`usrp_b200_bl.img`) has been loaded into it.
///
/// `FX3_REENUM_PID` in `b200_iface.hpp`. UHD only ever uses it as a fallback
/// when opening a device by VID/PID, which is exactly how it is used here: a
/// device sitting at this PID is still a bootloader, not a USRP.
pub const FX3_REENUM_PID: u16 = 0x00f0;

/// Ettus Research vendor ID, used by every B2xx built for Ettus.
pub const ETTUS_VID: u16 = 0x2500;

/// National Instruments vendor ID, used by the NI-branded OEM B200/B210.
pub const NI_VID: u16 = 0x3923;

/// Product ID shared by the B200 and the B210. The two are told apart by the
/// product code in the motherboard EEPROM, not by USB — see
/// [`B2xxProduct::from_eeprom_id`].
pub const B200_PID: u16 = 0x0020;

/// B200mini product ID.
pub const B200MINI_PID: u16 = 0x0021;

/// B205mini product ID.
pub const B205MINI_PID: u16 = 0x0022;

/// B206mini product ID.
pub const B206MINI_PID: u16 = 0x0023;

/// NI-branded B200 product ID (NI USRP-2900).
pub const B200_NI_PID: u16 = 0x7813;

/// NI-branded B210 product ID (NI USRP-2901).
pub const B210_NI_PID: u16 = 0x7814;

/// Every VID/PID pair a B2xx enumerates as once its EEPROM has been programmed.
///
/// Order matches `b200_vid_pid_pairs` in `b200_impl.hpp`.
pub const B2XX_VID_PIDS: [(u16, u16); 6] = [
    (ETTUS_VID, B200_PID),
    (ETTUS_VID, B200MINI_PID),
    (ETTUS_VID, B205MINI_PID),
    (ETTUS_VID, B206MINI_PID),
    (NI_VID, B200_NI_PID),
    (NI_VID, B210_NI_PID),
];

/// VID/PID pairs an unprogrammed or bootloader-mode FX3 enumerates as.
pub const FX3_BOOTLOADER_VID_PIDS: [(u16, u16); 2] =
    [(FX3_VID, FX3_BOOTLOADER_PID), (FX3_VID, FX3_REENUM_PID)];

/// A member of the B2xx family.
///
/// This is the product *identity* — what the board is — and says nothing about
/// whether it is currently usable; that is [`DeviceState`](super::DeviceState).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, PartialOrd, Ord)]
#[non_exhaustive]
pub enum B2xxProduct {
    /// B200: one RX and one TX channel on an AD9364.
    B200,
    /// B210: two RX and two TX channels on an AD9361.
    B210,
    /// B200mini.
    B200Mini,
    /// B205mini.
    B205Mini,
    /// B206mini.
    B206Mini,
}

impl B2xxProduct {
    /// Every product this driver knows about.
    pub const ALL: [Self; 5] = [
        Self::B200,
        Self::B210,
        Self::B200Mini,
        Self::B205Mini,
        Self::B206Mini,
    ];

    /// The product name UHD prints, e.g. `"B205mini"`.
    ///
    /// These strings are the ones that appear in `product=` device args, so
    /// they are spelled exactly as `B2XX_STR_NAMES` spells them — mixed case
    /// and all.
    #[must_use]
    pub fn name(self) -> &'static str {
        match self {
            Self::B200 => "B200",
            Self::B210 => "B210",
            Self::B200Mini => "B200mini",
            Self::B205Mini => "B205mini",
            Self::B206Mini => "B206mini",
        }
    }

    /// Identifies a product from a USB VID/PID pair, where that is possible.
    ///
    /// Returns `None` for [`ETTUS_VID`]`:`[`B200_PID`], which a B200 and a B210
    /// share: those two are only distinguished by the EEPROM product code, so
    /// resolving them means opening the device and reading it. Also `None` for
    /// anything not in the family at all, including an FX3 in bootloader mode.
    #[must_use]
    pub fn from_usb_id(vid: u16, pid: u16) -> Option<Self> {
        match (vid, pid) {
            (ETTUS_VID, B200MINI_PID) => Some(Self::B200Mini),
            (ETTUS_VID, B205MINI_PID) => Some(Self::B205Mini),
            (ETTUS_VID, B206MINI_PID) => Some(Self::B206Mini),
            (NI_VID, B200_NI_PID) => Some(Self::B200),
            (NI_VID, B210_NI_PID) => Some(Self::B210),
            _ => None,
        }
    }

    /// Identifies a product from the `product` code in the motherboard EEPROM.
    ///
    /// This is the authority for a device at [`ETTUS_VID`]`:`[`B200_PID`],
    /// which is the only case USB alone cannot resolve. Two codes map to each
    /// product because the NI-branded boards were programmed differently; the
    /// `0x77xx` codes are the older Ettus ones.
    #[must_use]
    pub fn from_eeprom_id(code: u16) -> Option<Self> {
        match code {
            0x0001 | 0x7737 | B200_NI_PID => Some(Self::B200),
            0x0002 | 0x7738 | B210_NI_PID => Some(Self::B210),
            0x0003 | 0x7739 => Some(Self::B200Mini),
            0x0004 | 0x773a => Some(Self::B205Mini),
            0x0005 | 0x773b => Some(Self::B206Mini),
            _ => None,
        }
    }

    /// Whether this is one of the mini boards, which have no GPSDO, no
    /// external reference input and a single AD9364 channel pair.
    #[must_use]
    pub fn is_mini(self) -> bool {
        matches!(self, Self::B200Mini | Self::B205Mini | Self::B206Mini)
    }

    /// File name of the FPGA image this product needs, as it appears in the
    /// UHD images archive. The B206mini shares the B205mini's image.
    #[must_use]
    pub fn fpga_image(self) -> &'static str {
        match self {
            Self::B200 => "usrp_b200_fpga.bin",
            Self::B210 => "usrp_b210_fpga.bin",
            Self::B200Mini => "usrp_b200mini_fpga.bin",
            Self::B205Mini | Self::B206Mini => "usrp_b205mini_fpga.bin",
        }
    }
}

impl std::fmt::Display for B2xxProduct {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.write_str(self.name())
    }
}

/// File name of the FX3 firmware image a B2xx needs loaded on every plug-in.
///
/// `B200_FW_FILE_NAME` in `b200_iface.hpp`.
pub const B200_FW_IMAGE: &str = "usrp_b200_fw.hex";

/// File name of the second-stage bootloader image, used to recover a board
/// whose EEPROM has been corrupted.
///
/// `B200_BL_FILE_NAME` in `b200_iface.hpp`.
pub const B200_BL_IMAGE: &str = "usrp_b200_bl.img";

/// Builds the VID/PID search list for a hint, mirroring the fallback ladder in
/// `b200_impl`'s constructor.
///
/// - Both given: exactly that pair, whatever it is. This is the escape hatch
///   for a board with a hand-programmed EEPROM.
/// - VID only: that VID against every B2xx PID.
/// - PID only: that PID against both [`ETTUS_VID`] and [`NI_VID`].
/// - Neither: every pair in [`B2XX_VID_PIDS`], followed by
///   [`FX3_BOOTLOADER_VID_PIDS`].
///
/// The last case is where this diverges from `b200_find`, which only looks for
/// programmed devices. Discovery here is meant to explain a board that does
/// *not* work yet, so an FX3 sitting in bootloader mode has to show up.
#[must_use]
pub fn vid_pid_search_list(vid: Option<u16>, pid: Option<u16>) -> Vec<(u16, u16)> {
    match (vid, pid) {
        (Some(vid), Some(pid)) => vec![(vid, pid)],
        (Some(vid), None) => [
            B200_PID,
            B200MINI_PID,
            B205MINI_PID,
            B206MINI_PID,
            B200_NI_PID,
            B210_NI_PID,
        ]
        .into_iter()
        .map(|pid| (vid, pid))
        .collect(),
        (None, Some(pid)) => vec![(ETTUS_VID, pid), (NI_VID, pid)],
        (None, None) => B2XX_VID_PIDS
            .into_iter()
            .chain(FX3_BOOTLOADER_VID_PIDS)
            .collect(),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn usb_ids_resolve_to_products() {
        assert_eq!(
            B2xxProduct::from_usb_id(ETTUS_VID, B205MINI_PID),
            Some(B2xxProduct::B205Mini)
        );
        assert_eq!(
            B2xxProduct::from_usb_id(NI_VID, B210_NI_PID),
            Some(B2xxProduct::B210)
        );
    }

    #[test]
    fn b200_and_b210_share_a_pid_so_usb_cannot_resolve_them() {
        assert_eq!(B2xxProduct::from_usb_id(ETTUS_VID, B200_PID), None);
        assert_eq!(B2xxProduct::from_eeprom_id(0x0001), Some(B2xxProduct::B200));
        assert_eq!(B2xxProduct::from_eeprom_id(0x0002), Some(B2xxProduct::B210));
    }

    #[test]
    fn a_bootloader_is_not_a_product() {
        assert_eq!(B2xxProduct::from_usb_id(FX3_VID, FX3_BOOTLOADER_PID), None);
    }

    #[test]
    fn every_eeprom_code_maps_and_no_two_products_share_one() {
        // Both spellings of every product, per B2XX_PRODUCT_ID.
        let codes = [
            (0x0001, B2xxProduct::B200),
            (0x7737, B2xxProduct::B200),
            (0x7813, B2xxProduct::B200),
            (0x0002, B2xxProduct::B210),
            (0x7738, B2xxProduct::B210),
            (0x7814, B2xxProduct::B210),
            (0x0003, B2xxProduct::B200Mini),
            (0x7739, B2xxProduct::B200Mini),
            (0x0004, B2xxProduct::B205Mini),
            (0x773a, B2xxProduct::B205Mini),
            (0x0005, B2xxProduct::B206Mini),
            (0x773b, B2xxProduct::B206Mini),
        ];
        for (code, expected) in codes {
            assert_eq!(
                B2xxProduct::from_eeprom_id(code),
                Some(expected),
                "{code:#06x}"
            );
        }
        assert_eq!(B2xxProduct::from_eeprom_id(0x0000), None);
        assert_eq!(B2xxProduct::from_eeprom_id(0xffff), None);
    }

    #[test]
    fn search_list_follows_the_hint() {
        assert_eq!(
            vid_pid_search_list(Some(0x1234), Some(0x5678)),
            vec![(0x1234, 0x5678)]
        );
        assert_eq!(
            vid_pid_search_list(None, Some(B205MINI_PID)),
            vec![(ETTUS_VID, B205MINI_PID), (NI_VID, B205MINI_PID)]
        );
        assert_eq!(vid_pid_search_list(Some(ETTUS_VID), None).len(), 6);

        let all = vid_pid_search_list(None, None);
        assert!(all.contains(&(ETTUS_VID, B205MINI_PID)));
        assert!(all.contains(&(FX3_VID, FX3_BOOTLOADER_PID)));
        assert_eq!(
            all.len(),
            B2XX_VID_PIDS.len() + FX3_BOOTLOADER_VID_PIDS.len()
        );
    }

    #[test]
    fn minis_are_minis() {
        assert!(B2xxProduct::B205Mini.is_mini());
        assert!(!B2xxProduct::B210.is_mini());
        assert_eq!(
            B2xxProduct::B206Mini.fpga_image(),
            B2xxProduct::B205Mini.fpga_image()
        );
    }
}
