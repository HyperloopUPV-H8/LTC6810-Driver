#ifndef DRIVER_HPP
#define DRIVER_HPP

#include <optional>
#include "NetworkLink.hpp"

#define ADC_RESOLUTION 100e-6  // 100 microV

#define REFON 1

using std::array;

namespace LTC6810Driver {

enum class AdcMode : uint8_t {
    KHZ_27 = 0,
    KHZ_14 = 1,
    KHZ_7 = 2,
    KHZ_3 = 3,
    KHZ_2 = 4,
    KHZ_1 = 5,
    HZ_422 = 6,
    HZ_26 = 7
};

template <size_t N_LTC6810>
class Driver {
    constexpr uint16_t build_ADCV(AdcMode mode) {
        switch (mode) {
            case AdcMode::HZ_422:
            case AdcMode::KHZ_1:
                return 0b0000001001100000;

            case AdcMode::KHZ_27:
            case AdcMode::KHZ_14:
                return 0b0000001011100000;

            case AdcMode::KHZ_7:
            case AdcMode::KHZ_3:
                return 0b0000001101100000;

            case AdcMode::HZ_26:
            case AdcMode::KHZ_2:
                return 0b0000001111100000;

            default:
                return 0;
        }
    }

    constexpr uint16_t build_ADCVSC(AdcMode mode) {
        switch (mode) {
            case AdcMode::HZ_422:
            case AdcMode::KHZ_1:
                return 0b0000010001110111;

            case AdcMode::KHZ_27:
            case AdcMode::KHZ_14:
                return 0b0000010011110111;

            case AdcMode::KHZ_7:
            case AdcMode::KHZ_3:
                return 0b0000010101110111;

            case AdcMode::HZ_26:
            case AdcMode::KHZ_2:
                return 0b0000010111110111;

            default:
                return 0;
        }
    }

    constexpr uint16_t build_ADAX(AdcMode mode) {
        switch (mode) {
            case AdcMode::HZ_422:
            case AdcMode::KHZ_1:
                return 0b0000010001100000;

            case AdcMode::KHZ_27:
            case AdcMode::KHZ_14:
                return 0b0000010011100000;

            case AdcMode::KHZ_7:
            case AdcMode::KHZ_3:
                return 0b0000010101100000;

            case AdcMode::HZ_26:
            case AdcMode::KHZ_2:
                return 0b0000010111100000;

            default:
                return 0;
        }
    }

    static constexpr array<uint8_t, 6> build_CRG() {
        if constexpr (REFON) {
            return {0x7C, 0x00, 0x00, 0x00, 0x00, 0x00};
        } else {
            return {0x78, 0x00, 0x00, 0x00, 0x00, 0x00};
        }
    }

    AdcMode current_mode{AdcMode::HZ_26};

    // Commands
    Command ADCV{build_ADCV(current_mode)};
    Command ADCVSC{build_ADCVSC(current_mode)};
    Command ADAX{build_ADAX(current_mode)};
    Command WRCFG{0b0000000000000001};
    Command RDCVA{0b0000000000000100};
    Command RDCVB{0b0000000000000110};
    Command RDAUXA{0b0000000000001100};
    Command RDAUXB{0b0000000000001110};
    Command RDSTATA{0b0000000000010000};

    // Registers
    Register CFG{build_CRG()};

    LTC6810Driver::NetworkLink<N_LTC6810> link;

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

    array<array<std::optional<float>, 4>, N_LTC6810> read_GPIOs() {
        array<Register, N_LTC6810> AUXA{link.read(RDAUXA)};
        array<Register, N_LTC6810> AUXB{link.read(RDAUXB)};

        array<array<std::optional<float>, 4>, N_LTC6810> GPIOs;

        for (uint i{0}; i < N_LTC6810; ++i) {
            if (AUXA[i].is_pec_valid()) {
                auto data_AUXA = AUXA[i].get_16bit_data();
                GPIOs[i][0] = data_AUXA[1] * ADC_RESOLUTION;
                GPIOs[i][1] = data_AUXA[2] * ADC_RESOLUTION;
            }

            if (AUXB[i].is_pec_valid()) {
                auto data_AUXB = AUXB[i].get_16bit_data();
                GPIOs[i][2] = data_AUXB[0] * ADC_RESOLUTION;
                GPIOs[i][3] = data_AUXB[1] * ADC_RESOLUTION;
            }
        }

        return GPIOs;
    }

    void faster_conv() {
        if (static_cast<int>(current_mode) > 0) {
            current_mode =
                static_cast<AdcMode>(static_cast<int>(current_mode) - 1);
            ADCV = build_ADCV(current_mode);
            ADCVSC = build_ADCVSC(current_mode);
            ADAX = build_ADAX(current_mode);
        }
    }
};
}  // namespace LTC6810Driver

#endif