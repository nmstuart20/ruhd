//! What a B2xx tells its AD936x about the board it is soldered to.
//!
//! Ports `b200_ad9361_client_t` and `b2xxmini_ad9361_client_t` from
//! `b200_impl.cpp:47-118`.
//!
//! # Why this lives here and not in `ad936x`
//!
//! These four settings are the entire content of the boundary between the
//! transceiver driver and the board. `ad936x` defines their shape and this
//! module supplies the values, which is what lets the transceiver driver compile
//! without knowing that USB, an FX3 or a B205mini exist. Putting a
//! `B205mini` constant in `ad936x` would be a one-line convenience that reverses
//! the dependency the crate was split to create.
//!
//! # What actually differs between the boards
//!
//! Only the band edges. Every B2xx clocks its codec the same way, wires the same
//! CMOS data port and uses the same timing trims; a full-size B200/B210 switches
//! RF ports as it tunes, and a mini does not — its edges are all zero so that
//! port A is used across the whole range, which is the reference's own comment on
//! the matter.

use ad936x::{BandEdges, ClientParams, ClockingMode, DigitalInterface, InterfaceDelays};
use uhd_transport::usb::B2xxProduct;

/// Data and clock delays every B2xx uses.
///
/// `get_digital_interface_timing`, identical in both client classes: no clock
/// delay either way, and the data delay at the top of its four-bit range.
pub const B2XX_DELAYS: InterfaceDelays = InterfaceDelays {
    rx_clk: 0,
    rx_data: 0xf,
    tx_clk: 0,
    tx_data: 0xf,
};

/// Band edges of a B200 or B210, in Hz. `b200_impl.cpp:51-63`.
///
/// The reference names the port each range selects: RX below 2.2 GHz on port C,
/// below 4.0 GHz on port B, and TX below 2.5 GHz on port B.
pub const FULL_BAND_EDGES: BandEdges = BandEdges {
    rx_band0: 2.2e9,
    rx_band1: 4.0e9,
    tx_band0: 2.5e9,
};

/// Band edges of a B2xxmini, in Hz. `b200_impl.cpp:88-100`.
///
/// All zero, "so RF port A is used all the time, on both Rx and Tx".
pub const MINI_BAND_EDGES: BandEdges = BandEdges {
    rx_band0: 0.0,
    rx_band1: 0.0,
    tx_band0: 0.0,
};

/// The settings to hand an [`Ad936x`](ad936x::Ad936x) on this board.
#[must_use]
pub fn client_params(product: B2xxProduct) -> ClientParams {
    ClientParams {
        clocking_mode: ClockingMode::XtalN,
        digital_interface: DigitalInterface::DdrFddLvcmos,
        delays: B2XX_DELAYS,
        band_edges: if is_mini(product) {
            MINI_BAND_EDGES
        } else {
            FULL_BAND_EDGES
        },
    }
}

/// Whether this is one of the mini boards. `is_b2xx_mini`, `b200_impl.cpp:41-45`.
#[must_use]
pub fn is_mini(product: B2xxProduct) -> bool {
    matches!(
        product,
        B2xxProduct::B200Mini | B2xxProduct::B205Mini | B2xxProduct::B206Mini
    )
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn every_board_clocks_and_wires_its_codec_the_same_way() {
        for product in B2xxProduct::ALL {
            let params = client_params(product);
            assert_eq!(params.clocking_mode, ClockingMode::XtalN, "{product}");
            assert_eq!(
                params.digital_interface,
                DigitalInterface::DdrFddLvcmos,
                "{product}"
            );
            assert_eq!(params.delays, B2XX_DELAYS, "{product}");
        }
    }

    #[test]
    fn only_the_full_size_boards_switch_rf_ports() {
        // A mini's zero edges are what keep it on port A across the whole
        // range; giving it the full-size edges would switch a B205mini onto
        // ports it does not have wired.
        assert_eq!(
            client_params(B2xxProduct::B205Mini).band_edges,
            MINI_BAND_EDGES
        );
        assert_eq!(
            client_params(B2xxProduct::B200Mini).band_edges,
            MINI_BAND_EDGES
        );
        assert_eq!(
            client_params(B2xxProduct::B206Mini).band_edges,
            MINI_BAND_EDGES
        );
        assert_eq!(client_params(B2xxProduct::B200).band_edges, FULL_BAND_EDGES);
        assert_eq!(client_params(B2xxProduct::B210).band_edges, FULL_BAND_EDGES);
    }
}
