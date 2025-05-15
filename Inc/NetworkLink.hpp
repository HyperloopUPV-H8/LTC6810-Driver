#ifndef NETWORK_LINK_HPP
#define NETWORK_LINK_HPP

#include <algorithm>
#include <array>
#include <chrono>
#include <cstddef>
#include <span>

#include "LTC6810Utilities.hpp"

using std::array;
using std::span;

#define REFON 1
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

struct SPIConfig {
    void (*const SPI_transmit)(const std::span<uint8_t>);
    void (*const SPI_receive)(std::span<uint8_t>);
    void (*const SPI_CS_turn_on)(void);
    void (*const SPI_CS_turn_off)(void);
};

template <size_t N_LTC6810, AdcMode mode>
class NetworkLink {
    const SPIConfig spi_link;

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
    Register CRG{build_CRG()};

   public:
    consteval NetworkLink(const SPIConfig& config) : spi_link{config} {}

    void wake_up() {
        array<uint8_t, 1> byte{0XFF};
        for (uint i{0}; i < N_LTC6810; ++i) {
            spi_link.SPI_CS_turn_off();
            spi_link.SPI_transmit(byte);
            spi_link.SPI_CS_turn_on();
        }
    }

    void config() {
        spi_link.SPI_CS_turn_off();
        spi_link.SPI_transmit(WRCFG.command);
        spi_link.SPI_transmit(CRG.reg);
        spi_link.SPI_CS_turn_on();
    }

    void start_cells_reading() {
        spi_link.SPI_CS_turn_off();
        spi_link.SPI_transmit(ADCV.command);

        // Conversion status valid only after N clock pulses
        array<uint8_t, (N_LTC6810 + 8 - 1) / 8> dummy{};
        spi_link.SPI_receive(dummy);
    };

    void start_GPIOs_reading() {
        spi_link.SPI_CS_turn_off();
        spi_link.SPI_transmit(ADAX.command);

        // Conversion status valid only after N clock pulses
        array<uint8_t, (N_LTC6810 + 8 - 1) / 8> dummy{};
        spi_link.SPI_receive(dummy);
    };

    bool is_conv_done() {
        std::array<uint8_t, 1> data;
        spi_link.SPI_receive(data);
        if (data[0] > 0) {
            spi_link.SPI_CS_turn_on();
            return true;
        }
        return false;
    }

    array<Register, N_LTC6810 * 2> read_cells() {
        array<Register, N_LTC6810> cell_group_A;
        array<Register, N_LTC6810> cell_group_B;

        spi_link.SPI_CS_turn_off();
        spi_link.SPI_transmit(RDCVA.command);
        for (uint i{0}; i < N_LTC6810; ++i) {
            array<uint8_t, 8> reg;
            spi_link.SPI_receive(reg);
            cell_group_A[i] = Register(std::move(reg));
        }
        spi_link.SPI_CS_turn_on();

        spi_link.SPI_CS_turn_off();
        spi_link.SPI_transmit(RDCVB.command);
        for (uint i{0}; i < N_LTC6810; ++i) {
            array<uint8_t, 8> reg;
            spi_link.SPI_receive(reg);
            cell_group_B[i] = Register(std::move(reg));
        }
        spi_link.SPI_CS_turn_on();

        array<Register, N_LTC6810 * 2> result;
        for (uint i{0}; i < N_LTC6810; ++i) {
            result[2 * i] = cell_group_A[i];
            result[2 * i + 1] = cell_group_B[i];
        }

        return result;
    }

    array<Register, N_LTC6810 * 2> read_GPIOs() {
        array<Register, N_LTC6810> auxiliary_group_A;
        array<Register, N_LTC6810> auxiliary_group_B;

        spi_link.SPI_CS_turn_off();
        spi_link.SPI_transmit(RDAUXA.command);
        for (uint i{0}; i < N_LTC6810; ++i) {
            array<uint8_t, 8> reg;
            spi_link.SPI_receive(reg);
            auxiliary_group_A[i] = Register(std::move(reg));
        }
        spi_link.SPI_CS_turn_on();

        spi_link.SPI_CS_turn_off();
        spi_link.SPI_transmit(RDAUXB.command);
        for (uint i{0}; i < N_LTC6810; ++i) {
            array<uint8_t, 8> reg;
            spi_link.SPI_receive(reg);
            auxiliary_group_B[i] = Register(std::move(reg));
        }
        spi_link.SPI_CS_turn_on();

        array<Register, N_LTC6810 * 2> result;
        for (uint i{0}; i < N_LTC6810; ++i) {
            result[2 * i] = auxiliary_group_A[i];
            result[2 * i + 1] = auxiliary_group_B[i];
        }

        return result;
    }
};
}  // namespace LTC6810
#endif