#ifndef DRIVER_HPP
#define DRIVER_HPP

#include "NetworkLink.hpp"

#define ADC_RESOLUTION 100e-6  // 100 microV

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
    Command RDSTATA{0b0000000000010000};

    // Registers
    Register CFG{build_CRG()};

    LTC6810::NetworkLink<N_LTC6810> link;

   public:
    consteval Driver(const SPIConfig& config) : link(config) {}

    void wake_up() {
        link.wake_up();
        link.write(WRCFG, CFG);
    }

    void start_cell_conversion() { link.send(ADCVSC); }
    void start_GPIOs_conversion() { link.send(ADAX); }

    bool is_conv_done() { return link.is_conv_done(); }

    array<array<std::optional<float>, 7>, N_LTC6810> read_cells() {
        array<Register, N_LTC6810> CVA{link.read(RDCVA)};
        array<Register, N_LTC6810> CVB{link.read(RDCVB)};
        array<Register, N_LTC6810> STATA{link.read(RDSTATA)};

        array<array<std::optional<float>, 7>, N_LTC6810> cells{};

        for (uint i{0}; i < N_LTC6810; ++i) {
            if (CVA[i].is_pec_valid()) {
                auto data_CVA = CVA[i].get_16bit_data();
                cells[i][0] = data_CVA[0] * ADC_RESOLUTION;
                cells[i][1] = data_CVA[1] * ADC_RESOLUTION;
                cells[i][2] = data_CVA[2] * ADC_RESOLUTION;
            }
            if (CVB[i].is_pec_valid()) {
                auto data_CVB = CVB[i].get_16bit_data();
                cells[i][3] = data_CVB[0] * ADC_RESOLUTION;
                cells[i][4] = data_CVB[1] * ADC_RESOLUTION;
                cells[i][5] = data_CVB[2] * ADC_RESOLUTION;
            }
            if (STATA[i].is_pec_valid()) {
                auto data_STATA = STATA[i].get_16bit_data();
                cells[i][6] = data_STATA[0] * ADC_RESOLUTION * 10;
            }
        }
        return cells;
    }

    array<array<std::optional<uint16_t>, 4>, N_LTC6810> read_GPIOs() {
        array<Register, N_LTC6810> AUXA{link.read(RDAUXA)};
        array<Register, N_LTC6810> AUXB{link.read(RDAUXB)};

        array<array<std::optional<uint16_t>, 4>, N_LTC6810> GPIOs;

        for (uint i{0}; i < N_LTC6810; ++i) {
            if (AUXA[i].is_pec_valid()) {
                auto data_AUXA = AUXA[i].get_16bit_data();
                GPIOs[i][0] = data_AUXA[1];
                GPIOs[i][1] = data_AUXA[2];
            }

            if (AUXB[i].is_pec_valid()) {
                auto data_AUXB = AUXB[i].get_16bit_data();
                GPIOs[i][2] = data_AUXB[0];
                GPIOs[i][3] = data_AUXB[1];
            }
        }

        return GPIOs;
    }
};
}  // namespace LTC6810

#endif