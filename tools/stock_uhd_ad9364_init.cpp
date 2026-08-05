// Initialize one B2xx through stock UHD at the settings used by the register
// comparison harness. Raw AD936x SPI access is not part of UHD's public API;
// after this process releases the USB interfaces, the RUHD read-only helper
// snapshots the state this process left in the chip.

#include <uhd/types/tune_request.hpp>
#include <uhd/usrp/multi_usrp.hpp>

#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <string>

namespace {

double number(const char* text, const char* name)
{
    char* end       = nullptr;
    const double out = std::strtod(text, &end);
    if (end == text || *end != '\0') {
        throw std::runtime_error(std::string("invalid ") + name + ": " + text);
    }
    return out;
}

} // namespace

int main(int argc, char** argv)
{
    if (argc != 8) {
        std::cerr
            << "usage: stock_uhd_ad9364_init DEVICE_ARGS RATE RX_FREQ TX_FREQ "
               "RX_GAIN TX_GAIN BANDWIDTH\n";
        return 2;
    }

    try {
        const std::string device_args(argv[1]);
        const double rate      = number(argv[2], "rate");
        const double rx_freq   = number(argv[3], "rx frequency");
        const double tx_freq   = number(argv[4], "tx frequency");
        const double rx_gain   = number(argv[5], "rx gain");
        const double tx_gain   = number(argv[6], "tx gain");
        const double bandwidth = number(argv[7], "bandwidth");

        auto usrp = uhd::usrp::multi_usrp::make(device_args);
        if (usrp->get_rx_num_channels() != 1 || usrp->get_tx_num_channels() != 1) {
            throw std::runtime_error(
                "the Phase 3 register gate requires a single-channel AD9364 B2xx");
        }

        // master_clock_rate is also present in DEVICE_ARGS so the codec is
        // initialized at the requested rate before frontend properties are
        // populated. Repeating it here both disables auto-MCR explicitly and
        // gives us an actual-value readback to report.
        usrp->set_master_clock_rate(rate);
        usrp->set_rx_rate(rate, 0);
        usrp->set_tx_rate(rate, 0);
        usrp->set_rx_freq(uhd::tune_request_t(rx_freq), 0);
        usrp->set_tx_freq(uhd::tune_request_t(tx_freq), 0);
        usrp->set_rx_agc(false, 0);
        usrp->set_rx_gain(rx_gain, 0);
        usrp->set_tx_gain(tx_gain, 0);
        usrp->set_rx_bandwidth(bandwidth, 0);
        usrp->set_tx_bandwidth(bandwidth, 0);

        std::cerr << std::fixed << std::setprecision(3)
                  << "stock UHD actual settings: master_clock_rate="
                  << usrp->get_master_clock_rate() << " rx_rate=" << usrp->get_rx_rate(0)
                  << " tx_rate=" << usrp->get_tx_rate(0)
                  << " rx_freq=" << usrp->get_rx_freq(0)
                  << " tx_freq=" << usrp->get_tx_freq(0) << std::setprecision(2)
                  << " rx_gain=" << usrp->get_rx_gain(0)
                  << " tx_gain=" << usrp->get_tx_gain(0) << std::setprecision(3)
                  << " rx_bandwidth=" << usrp->get_rx_bandwidth(0)
                  << " tx_bandwidth=" << usrp->get_tx_bandwidth(0) << '\n';
    } catch (const std::exception& error) {
        std::cerr << "stock UHD initialization failed: " << error.what() << '\n';
        return 1;
    }
    return 0;
}
