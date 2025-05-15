#ifndef DRIVER_HPP
#define DRIVER_HPP

#include "NetworkLink.hpp"

#define REFON 1

using std::array;

namespace LTC6810 {

enum class AdcMode {
    KHZ_27,
    KHZ_14,
    KHZ_7,
    KHZ_3,
    KHZ_2,
    KHZ_1,
    HZ_422,
    HZ_26
};

template <size_t N_LTC6810, AdcMode mode>
class Driver {
    static constexpr uint16_t build_ADCV() {
        if constexpr (mode == AdcMode::HZ_422 || mode == AdcMode::KHZ_1) {
            return 0b0000001001100000;
        } else if constexpr (mode == AdcMode::KHZ_27 ||
                             mode == AdcMode::KHZ_14) {
            return 0b0000001011100000;
        } else if constexpr (mode == AdcMode::KHZ_7 || mode == AdcMode::KHZ_3) {
            return 0b0000001101100000;
        } else if constexpr (mode == AdcMode::HZ_26 || mode == AdcMode::KHZ_2) {
            return 0b0000001111100000;
        }
    }

    static constexpr uint16_t build_ADCVSC() {
        if constexpr (mode == AdcMode::HZ_422 || mode == AdcMode::KHZ_1) {
            return 0b0000010001110111;
        } else if constexpr (mode == AdcMode::KHZ_27 ||
                             mode == AdcMode::KHZ_14) {
            return 0b0000010011110111;
        } else if constexpr (mode == AdcMode::KHZ_7 || mode == AdcMode::KHZ_3) {
            return 0b0000010101110111;
        } else if constexpr (mode == AdcMode::HZ_26 || mode == AdcMode::KHZ_2) {
            return 0b0000010111110111;
        }
    }

    static constexpr uint16_t build_ADAX() {
        if constexpr (mode == AdcMode::HZ_422 || mode == AdcMode::KHZ_1) {
            return 0b0000010001100000;
        } else if constexpr (mode == AdcMode::KHZ_27 ||
                             mode == AdcMode::KHZ_14) {
            return 0b0000010011100000;
        } else if constexpr (mode == AdcMode::KHZ_7 || mode == AdcMode::KHZ_3) {
            return 0b0000010101100000;
        } else if constexpr (mode == AdcMode::HZ_26 || mode == AdcMode::KHZ_2) {
            return 0b0000010111100000;
        }
    }

    static constexpr array<uint8_t, 8> build_CRG() {
        if constexpr (REFON) {
            return {0x04, 0x00, 0x00, 0x00, 0x00, 0x00};
        } else {
            return {0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
        }
    }

    // Commands
    Command ADCV{build_ADCV()};
    Command ADCVSC{build_ADCVSC()};
    Command ADAX{build_ADAX()};
    Command WRCFG{0};
    Command RDCVA{0b0000000000000100};
    Command RDCVB{0b0000000000000110};
    Command RDAUXA{0b0000000000001100};
    Command RDAUXB{0b0000000000001110};

    // Registers
    Register CFG{build_CRG()};

    LTC6810::NetworkLink<N_LTC6810> link;

   public:
    consteval Driver(const SPIConfig& config) : link(config) {}

    void wake_up() {
        link.wake_up();
        link.write(WRCFG, CFG);
    }

    void start_cell_conversion() { link.send(ADCV); }
    void start_GPIOs_conversion() { link.send(ADAX); }
    // CAREFUL!!!! At the moment, you are not taking into account that the first
    // N_LTC6810 bits aren't valid!

    bool is_conv_done() { return link.is_conv_done(); }

    array<array<float, 6>, N_LTC6810> read_cells() {
        [[maybe_unused]] array<Register, N_LTC6810> CVA{link.read(RDCVA)};
        [[maybe_unused]] array<Register, N_LTC6810> CVB{link.read(RDCVB)};
        // TODO
        return {};
    }

    array<array<uint16_t, 4>, N_LTC6810> read_GPIOs() {
        [[maybe_unused]] array<Register, N_LTC6810> AUXA{link.read(RDAUXA)};
        [[maybe_unused]] array<Register, N_LTC6810> AUXB{link.read(RDAUXB)};
        // TODO
        return {};
    }
};
}  // namespace LTC6810

#endif