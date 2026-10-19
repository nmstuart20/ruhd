//! The RX and TX FIR filters: the tap tables, and the indirect write sequence
//! that gets them into the part.
//!
//! Ports `_program_fir_filter` (`ad9361_device.cpp:100-175`), `_setup_rx_fir`
//! and `_setup_tx_fir` (`179-248`), and the tables from
//! `ad9361_filter_taps.h`.
//!
//! # Eight tables, transcribed
//!
//! There are two families. The halfband tables are used when the FIR is
//! decimating or interpolating by two; the `x4` tables are equiripple low-passes
//! for the by-four case. Within each family there is one table per tap count,
//! and [`crate::clocking::num_taps`] has already decided which count fits in the
//! clock budget. The reference's own comments record how each was designed, and
//! they are kept, because "round(2^16 * halfgen4(0.9/4,32))" is the only
//! provenance these numbers have.
//!
//! The values are transcribed mechanically and byte for byte. They are not
//! regenerated from the design expressions and must not be: the tables that ship
//! are what every gain and calibration constant in the driver was measured
//! against, and a coefficient that differs in the last place is a filter
//! response that differs from the reference radio's.
//!
//! # The write sequence is not a register write
//!
//! The tap memory is behind an indirect port — address register, two data
//! registers, and a write strobe that has to be raised and lowered in a
//! particular order with the filter clock running. UG-671 page 25 spells out the
//! ending, and the reference quotes it; the quote is kept here too. The cost is
//! six SPI transactions per tap, so programming a 128-tap filter is 768 round
//! trips. On a B2xx, over USB, that is most of what a rate change spends its
//! time doing.

use std::thread::sleep;
use std::time::Duration;

use uhd_core::{Error, Result};

use crate::regs::{RX_FIR_BASE, TX_FIR_BASE};
use crate::spi::Registers;
use crate::{Chain, Direction};

/// How long the filter clock is given to start before the taps are written.
///
/// The reference's one-millisecond sleep after enabling it.
const FILTER_CLOCK_SETTLING: Duration = Duration::from_millis(1);

/// How many taps the part has memory for, in either direction.
pub const MAX_TAPS: usize = 128;

/// Selects the RX tap table for a tap count and decimation.
///
/// # Errors
///
/// [`Error::InvalidArg`] if the decimation is not 1, 2 or 4, or if no table
/// exists for that tap count — the reference's `"Invalid Rx FIR decimation"` and
/// `"Unsupported number of Rx FIR taps"`.
pub fn rx_taps(num_taps: usize, decimation: i32) -> Result<&'static [i16]> {
    if !matches!(decimation, 1 | 2 | 4) {
        return Err(Error::invalid_arg(format!(
            "ad936x rx fir decimation {decimation}; the part does 1, 2 or 4"
        )));
    }
    table(num_taps, decimation, Direction::Rx)
}

/// Selects the TX tap table for a tap count and interpolation.
///
/// # Errors
///
/// As [`rx_taps`], plus [`Error::InvalidArg`] for more than 64 taps at 1x
/// interpolation: the TX FIR only has memory for 128 taps in the 2x and 4x
/// modes.
pub fn tx_taps(num_taps: usize, interpolation: i32) -> Result<&'static [i16]> {
    if !matches!(interpolation, 1 | 2 | 4) {
        return Err(Error::invalid_arg(format!(
            "ad936x tx fir interpolation {interpolation}; the part does 1, 2 or 4"
        )));
    }
    if interpolation == 1 && num_taps > 64 {
        return Err(Error::invalid_arg(format!(
            "ad936x tx fir: {num_taps} taps at 1x interpolation; the maximum there is 64"
        )));
    }
    table(num_taps, interpolation, Direction::Tx)
}

/// The table both directions share. `_setup_rx_fir` and `_setup_tx_fir` differ
/// only in their guards; the switch on tap count is the same in both.
fn table(num_taps: usize, factor: i32, direction: Direction) -> Result<&'static [i16]> {
    let by_four = factor == 4;
    let taps: &'static [i16] = match num_taps {
        128 if by_four => &taps::FIR_128_X4_COEFFS,
        128 => &taps::HB127_COEFFS,
        96 if by_four => &taps::FIR_96_X4_COEFFS,
        96 => &taps::HB95_COEFFS,
        64 if by_four => &taps::FIR_64_X4_COEFFS,
        64 => &taps::HB63_COEFFS,
        48 if by_four => &taps::FIR_48_X4_COEFFS,
        48 => &taps::HB47_COEFFS,
        _ => {
            return Err(Error::invalid_arg(format!(
                "ad936x has no {num_taps}-tap {} fir table; it has 48, 64, 96 and 128",
                direction.name()
            )));
        }
    };
    Ok(taps)
}

/// Writes a set of taps into one FIR filter.
///
/// `_program_fir_filter`. `coeffs` is `num_taps` long; taps beyond it are zeroed
/// out to `MAX_TAPS`, because stale coefficients left in the unused half of the
/// memory are still coefficients.
///
/// # Errors
///
/// [`Error::InvalidArg`] if the tap count is not a multiple of 16 between 16 and
/// 128, which is what the part's `numtaps` field can express; otherwise whatever
/// the bus reports.
pub fn program(regs: &Registers, direction: Direction, chain: Chain, coeffs: &[i16]) -> Result<()> {
    let num_taps = coeffs.len();
    if num_taps == 0 || num_taps > MAX_TAPS || !num_taps.is_multiple_of(16) {
        return Err(Error::invalid_arg(format!(
            "ad936x fir tap count {num_taps}; it must be a multiple of 16 up to {MAX_TAPS}"
        )));
    }

    // RX and TX filters use largely identical sets of programming registers;
    // select the appropriate bank here.
    let base = match direction {
        Direction::Rx => RX_FIR_BASE,
        Direction::Tx => TX_FIR_BASE,
    };

    // Encode number of filter taps for programming register.
    let reg_numtaps = u8::try_from(((num_taps / 16) - 1) & 0x07).unwrap_or(0) << 5;
    let reg_chain = match chain {
        Chain::One => 0x01 << 3,
        Chain::Two => 0x02 << 3,
        Chain::Both => 0x03 << 3,
    };
    let ctrl = reg_numtaps | reg_chain;

    // Turn on the filter clock.
    regs.poke8(base + 5, ctrl | 0x02)?;
    sleep(FILTER_CLOCK_SETTLING);

    // Zero the unused taps just in case they have stale data, and only then
    // write the real ones. Two loops in the reference; one here, but in the
    // reference's order — the high addresses first — because the sequence on
    // the wire is the thing being reproduced, and a capture taken against UHD
    // should diff clean.
    let zeros = (num_taps..MAX_TAPS).map(|addr| (addr, 0i16));
    let taps = coeffs.iter().copied().enumerate();
    for (addr, coeff) in zeros.chain(taps) {
        let coeff = coeff as u16;
        regs.poke8(base, addr as u8)?;
        regs.poke8(base + 1, (coeff & 0xff) as u8)?;
        regs.poke8(base + 2, ((coeff >> 8) & 0xff) as u8)?;
        regs.poke8(base + 5, ctrl | (1 << 1) | (1 << 2))?;
        regs.poke8(base + 4, 0x00)?;
        regs.poke8(base + 4, 0x00)?;
    }

    // UG-671 states (page 25) (paraphrased and clarified):
    //   "After the table has been programmed, write to register BASE+5 with the
    //   write bit D2 cleared and D1 high. Then, write to register BASE+5 again
    //   with D1 clear, thus ensuring that the write bit resets internally before
    //   the clock stops. Wait 4 sample clock periods after setting D2 high while
    //   that data writes into the table"
    regs.poke8(base + 5, ctrl | (1 << 1))?;
    regs.poke8(base + 5, ctrl)?;

    if direction == Direction::Rx {
        // Rx Gain, set to prevent digital overflow/saturation in filters
        //   0:+6dB, 1:0dB, 2:-6dB, 3:-12dB
        // page 35 of UG-671. -6 dB, to stop filter overflow.
        regs.poke8(base + 6, 0x02)?;
    }

    Ok(())
}

/// Programs the RX FIR for a tap count and decimation, on both chains.
///
/// `_setup_rx_fir`.
///
/// # Errors
///
/// As [`rx_taps`] and [`program`].
pub fn setup_rx_fir(regs: &Registers, num_taps: usize, decimation: i32) -> Result<()> {
    program(
        regs,
        Direction::Rx,
        Chain::Both,
        rx_taps(num_taps, decimation)?,
    )
}

/// Programs the TX FIR for a tap count and interpolation, on both chains.
///
/// `_setup_tx_fir`.
///
/// # Errors
///
/// As [`tx_taps`] and [`program`].
pub fn setup_tx_fir(regs: &Registers, num_taps: usize, interpolation: i32) -> Result<()> {
    program(
        regs,
        Direction::Tx,
        Chain::Both,
        tx_taps(num_taps, interpolation)?,
    )
}

/// The coefficient tables, transcribed from `ad9361_filter_taps.h`.
///
/// Each is exactly as long as the C++ array it came from, which is one entry
/// *more* than the filter's name suggests for the halfbands: `hb127_coeffs` is
/// 127 taps plus a trailing zero, so that it can be programmed as a 128-tap
/// filter. The tap counts [`crate::clocking::num_taps`] produces are the array
/// lengths, not the names.
pub mod taps {
    /// 127 tap Halfband designed with: `round(2^16 * halfgen4(0.9/4,32))`
    /// (center tap tweaked to 32767).
    pub const HB127_COEFFS: [i16; 128] = [
        0, 0, 1, 0, -2, 0, 3, 0, -5, 0, 8, 0, -11, 0, 17, 0, -24, 0, 33, 0, -45, 0, 61, 0, -80, 0,
        104, 0, -134, 0, 169, 0, -213, 0, 264, 0, -327, 0, 401, 0, -489, 0, 595, 0, -724, 0, 880,
        0, -1075, 0, 1323, 0, -1652, 0, 2114, 0, -2819, 0, 4056, 0, -6883, 0, 20837, 32767, 20837,
        0, -6883, 0, 4056, 0, -2819, 0, 2114, 0, -1652, 0, 1323, 0, -1075, 0, 880, 0, -724, 0, 595,
        0, -489, 0, 401, 0, -327, 0, 264, 0, -213, 0, 169, 0, -134, 0, 104, 0, -80, 0, 61, 0, -45,
        0, 33, 0, -24, 0, 17, 0, -11, 0, 8, 0, -5, 0, 3, 0, -2, 0, 1, 0, 0, 0,
    ];

    /// 95 tap Halfband designed with: `round(2^16 * halfgen4(0.9/4,24))`
    /// (center tap tweaked to 32767).
    pub const HB95_COEFFS: [i16; 96] = [
        -4, 0, 8, 0, -14, 0, 23, 0, -36, 0, 52, 0, -75, 0, 104, 0, -140, 0, 186, 0, -243, 0, 314,
        0, -400, 0, 505, 0, -634, 0, 793, 0, -993, 0, 1247, 0, -1585, 0, 2056, 0, -2773, 0, 4022,
        0, -6862, 0, 20830, 32767, 20830, 0, -6862, 0, 4022, 0, -2773, 0, 2056, 0, -1585, 0, 1247,
        0, -993, 0, 793, 0, -634, 0, 505, 0, -400, 0, 314, 0, -243, 0, 186, 0, -140, 0, 104, 0,
        -75, 0, 52, 0, -36, 0, 23, 0, -14, 0, 8, 0, -4, 0,
    ];

    /// 63 tap Halfband designed with: `round(2^16 * halfgen4(0.9/4,16))`
    /// (center tap tweaked to 32767).
    pub const HB63_COEFFS: [i16; 64] = [
        -58, 0, 83, 0, -127, 0, 185, 0, -262, 0, 361, 0, -488, 0, 648, 0, -853, 0, 1117, 0, -1466,
        0, 1954, 0, -2689, 0, 3960, 0, -6825, 0, 20818, 32767, 20818, 0, -6825, 0, 3960, 0, -2689,
        0, 1954, 0, -1466, 0, 1117, 0, -853, 0, 648, 0, -488, 0, 361, 0, -262, 0, 185, 0, -127, 0,
        83, 0, -58, 0,
    ];

    /// 47 tap Halfband designed with: `round(2^16 * halfgen4(0.85/4,12))`
    /// (center tap tweaked to 32767).
    pub const HB47_COEFFS: [i16; 48] = [
        -50, 0, 98, 0, -181, 0, 307, 0, -489, 0, 747, 0, -1109, 0, 1628, 0, -2413, 0, 3750, 0,
        -6693, 0, 20773, 32767, 20773, 0, -6693, 0, 3750, 0, -2413, 0, 1628, 0, -1109, 0, 747, 0,
        -489, 0, 307, 0, -181, 0, 98, 0, -50, 0,
    ];

    /// 128 tap equiripple FIR low-pass designed with:
    /// `round(2^16 * fir1(127,0.25))`. Decimation / interpolation by 4, -3 dB
    /// rolloff at Fs/4.
    pub const FIR_128_X4_COEFFS: [i16; 128] = [
        -15, -27, -23, -6, 17, 33, 31, 9, -23, -47, -45, -13, 34, 69, 67, 21, -49, -102, -99, -32,
        69, 146, 143, 48, -96, -204, -200, -69, 129, 278, 275, 97, -170, -372, -371, -135, 222,
        494, 497, 187, -288, -654, -665, -258, 376, 875, 902, 363, -500, -1201, -1265, -530, 699,
        1748, 1906, 845, -1089, -2922, -3424, -1697, 2326, 7714, 12821, 15921, 15921, 12821, 7714,
        2326, -1697, -3424, -2922, -1089, 845, 1906, 1748, 699, -530, -1265, -1201, -500, 363, 902,
        875, 376, -258, -665, -654, -288, 187, 497, 494, 222, -135, -371, -372, -170, 97, 275, 278,
        129, -69, -200, -204, -96, 48, 143, 146, 69, -32, -99, -102, -49, 21, 67, 69, 34, -13, -45,
        -47, -23, 9, 31, 33, 17, -6, -23, -27, -15,
    ];

    /// 96 tap equiripple FIR low-pass designed with:
    /// `round(2^16 * fir1(95,0.25))`.
    pub const FIR_96_X4_COEFFS: [i16; 96] = [
        -18, -35, -33, -11, 23, 50, 51, 18, -37, -83, -86, -31, 62, 140, 145, 54, -98, -224, -232,
        -88, 149, 343, 356, 138, -218, -509, -530, -211, 313, 743, 781, 320, -447, -1089, -1163,
        -494, 658, 1663, 1830, 819, -1062, -2868, -3379, -1682, 2314, 7695, 12812, 15924, 15924,
        12812, 7695, 2314, -1682, -3379, -2868, -1062, 819, 1830, 1663, 658, -494, -1163, -1089,
        -447, 320, 781, 743, 313, -211, -530, -509, -218, 138, 356, 343, 149, -88, -232, -224, -98,
        54, 145, 140, 62, -31, -86, -83, -37, 18, 51, 50, 23, -11, -33, -35, -18,
    ];

    /// 64 tap equiripple FIR low-pass designed with:
    /// `round(2^16 * fir1(63,0.25))`.
    pub const FIR_64_X4_COEFFS: [i16; 64] = [
        -25, -54, -56, -22, 41, 102, 117, 50, -87, -223, -253, -109, 174, 443, 496, 215, -317,
        -809, -903, -398, 550, 1434, 1623, 744, -987, -2715, -3251, -1640, 2279, 7638, 12782,
        15928, 15928, 12782, 7638, 2279, -1640, -3251, -2715, -987, 744, 1623, 1434, 550, -398,
        -903, -809, -317, 215, 496, 443, 174, -109, -253, -223, -87, 50, 117, 102, 41, -22, -56,
        -54, -25,
    ];

    /// 48 tap equiripple FIR low-pass designed with:
    /// `round(2^16 * fir1(47,0.25))`.
    pub const FIR_48_X4_COEFFS: [i16; 48] = [
        -32, -74, -84, -39, 68, 191, 237, 114, -183, -508, -609, -287, 419, 1149, 1358, 647, -887,
        -2508, -3073, -1580, 2230, 7555, 12736, 15928, 15928, 12736, 7555, 2230, -1580, -3073,
        -2508, -887, 647, 1358, 1149, 419, -287, -609, -508, -183, 114, 237, 191, 68, -39, -84,
        -74, -32,
    ];
}

#[cfg(test)]
mod tests {
    use std::sync::Arc;

    use super::*;
    use crate::spi::tests::FakeSpi;

    /// The tables are the reference's, and the properties that would catch a
    /// transcription slip are their length, their symmetry and their centre.
    #[test]
    fn the_tap_tables_are_the_shape_the_reference_gives_them() {
        for (name, table) in [
            ("hb127", &taps::HB127_COEFFS[..]),
            ("hb95", &taps::HB95_COEFFS[..]),
            ("hb63", &taps::HB63_COEFFS[..]),
            ("hb47", &taps::HB47_COEFFS[..]),
        ] {
            assert_eq!(table.len() % 16, 0, "{name} length");

            // An odd-length halfband padded with one trailing zero puts its
            // centre tap just before the middle. That tap is the one the
            // reference notes was "tweaked to 32767" — the largest positive
            // value an i16 holds, so that unity gain does not overflow.
            let centre = table.len() / 2 - 1;
            assert_eq!(table[centre], 32767, "{name} centre tap");

            // Every odd tap but the centre is zero, which is what makes it a
            // halfband, and the taps mirror about the centre.
            assert!(
                (table.iter().enumerate()).all(|(i, &t)| i % 2 == 0 || i == centre || t == 0),
                "{name} has a nonzero odd tap"
            );
            for k in 1..=centre {
                assert_eq!(table[centre - k], table[centre + k], "{name} tap {k}");
            }
        }

        for (name, table) in [
            ("fir_128_x4", &taps::FIR_128_X4_COEFFS[..]),
            ("fir_96_x4", &taps::FIR_96_X4_COEFFS[..]),
            ("fir_64_x4", &taps::FIR_64_X4_COEFFS[..]),
            ("fir_48_x4", &taps::FIR_48_X4_COEFFS[..]),
        ] {
            // These are linear phase: the coefficients read the same backwards.
            let reversed: Vec<i16> = table.iter().rev().copied().collect();
            assert_eq!(reversed, table, "{name} is not symmetric");
        }
    }

    #[test]
    fn table_selection_follows_the_decimation() {
        assert_eq!(rx_taps(128, 2).unwrap(), &taps::HB127_COEFFS[..]);
        assert_eq!(rx_taps(128, 4).unwrap(), &taps::FIR_128_X4_COEFFS[..]);
        assert_eq!(rx_taps(96, 1).unwrap(), &taps::HB95_COEFFS[..]);
        assert_eq!(tx_taps(48, 4).unwrap(), &taps::FIR_48_X4_COEFFS[..]);
        // A decimation of 1 uses the halfband tables too: only the by-four case
        // switches.
        assert_eq!(tx_taps(64, 1).unwrap(), &taps::HB63_COEFFS[..]);
    }

    #[test]
    fn unsupported_tap_counts_and_factors_are_refused() {
        assert!(rx_taps(80, 2).is_err(), "no 80-tap table exists");
        assert!(rx_taps(128, 3).is_err(), "decimation must be 1, 2 or 4");
        assert!(tx_taps(128, 1).is_err(), "1x interpolation caps at 64 taps");
        assert!(tx_taps(64, 1).is_ok());
    }

    #[test]
    fn programming_a_filter_writes_the_reference_sequence() {
        // A 48-tap RX filter: enable, 128 indirect writes of six pokes each,
        // the UG-671 ending, and the -6 dB gain byte.
        let spi = Arc::new(FakeSpi::default());
        let regs = Registers::new(spi.clone());
        program(&regs, Direction::Rx, Chain::Both, &taps::HB47_COEFFS).unwrap();

        let pokes = spi.pokes();
        assert_eq!(pokes.len(), 1 + MAX_TAPS * 6 + 2 + 1);

        // numtaps field is (48/16 - 1) = 2 in bits 7:5; chain is 3 in bits 4:3.
        let ctrl = (2 << 5) | (3 << 3);
        assert_eq!(pokes[0], (0x0f5, ctrl | 0x02), "filter clock on");

        // The unused taps go first, starting at address 48: address, low byte,
        // high byte, strobe, two zero writes.
        assert_eq!(
            &pokes[1..7],
            &[
                (0x0f0, 48),
                (0x0f1, 0x00),
                (0x0f2, 0x00),
                (0x0f5, ctrl | 0x06),
                (0x0f4, 0x00),
                (0x0f4, 0x00),
            ]
        );

        // Then the real ones from address 0. HB47_COEFFS[0] is -50, which is
        // 0xffce as a sixteen-bit word.
        let coeffs_at = 1 + (MAX_TAPS - 48) * 6;
        assert_eq!(
            &pokes[coeffs_at..coeffs_at + 3],
            &[(0x0f0, 0x00), (0x0f1, 0xce), (0x0f2, 0xff)]
        );
        // Tap 46 mirrors tap 0, and tap 47 is the trailing zero the 47-tap
        // table carries to make its length a multiple of 16 — written as a
        // coefficient here, not skipped.
        assert_eq!(
            &pokes[coeffs_at + 46 * 6..coeffs_at + 46 * 6 + 3],
            &[(0x0f0, 46), (0x0f1, 0xce), (0x0f2, 0xff)]
        );
        assert_eq!(
            &pokes[coeffs_at + 47 * 6..coeffs_at + 47 * 6 + 3],
            &[(0x0f0, 47), (0x0f1, 0x00), (0x0f2, 0x00)]
        );

        // The ending: strobe clear with D1 high, then D1 clear, then the gain.
        assert_eq!(
            &pokes[1 + MAX_TAPS * 6..],
            &[(0x0f5, ctrl | 0x02), (0x0f5, ctrl), (0x0f6, 0x02)]
        );
    }

    #[test]
    fn the_tx_filter_uses_its_own_bank_and_skips_the_rx_gain_byte() {
        let spi = Arc::new(FakeSpi::default());
        let regs = Registers::new(spi.clone());
        program(&regs, Direction::Tx, Chain::One, &taps::HB63_COEFFS).unwrap();

        let pokes = spi.pokes();
        // No trailing gain write on the TX side.
        assert_eq!(pokes.len(), 1 + MAX_TAPS * 6 + 2);
        assert!(
            pokes.iter().all(|&(reg, _)| (0x060..=0x066).contains(&reg)),
            "a tx filter must stay in the 0x060 bank"
        );
        // 64 taps is (64/16 - 1) = 3, chain 1 is 0x01 << 3.
        assert_eq!(pokes[0], (0x065, (3 << 5) | (1 << 3) | 0x02));
    }

    #[test]
    fn a_tap_count_the_numtaps_field_cannot_express_is_refused() {
        let regs = Registers::new(Arc::new(FakeSpi::default()));
        assert!(program(&regs, Direction::Rx, Chain::Both, &[0; 24]).is_err());
        assert!(program(&regs, Direction::Rx, Chain::Both, &[]).is_err());
        assert!(program(&regs, Direction::Rx, Chain::Both, &[0; 144]).is_err());
    }
}
