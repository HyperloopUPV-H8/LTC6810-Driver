#ifndef NETWORK_LINK_HPP
#define NETWORK_LINK_HPP

#include <algorithm>
#include <chrono>
#include <cstddef>
#include <span>

using std::array;
using Command = array<uint8_t, 2>;
using map_block = array<uint8_t, 6>;

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

template <size_t N_LTC6810>
class NetworkLink {
    const SPIConfig spi_link;

    constexpr static uint16_t CRC15_POLY = 0x4599;

    constexpr static array<uint16_t, 256> init_PEC15_Table() {
        array<uint16_t, 256> table{};

        for (int i = 0; i < 256; ++i) {
            uint16_t remainder = static_cast<uint16_t>(i << 7);
            for (int bit = 0; bit < 8; ++bit) {
                if (remainder & 0x4000) {
                    remainder =
                        static_cast<uint16_t>((remainder << 1) ^ CRC15_POLY);
                } else {
                    remainder = static_cast<uint16_t>(remainder << 1);
                }
            }
            table[i] = remainder & 0xFFFF;
        }

        return table;
    }

    constexpr static auto pec15Table = init_PEC15_Table();

    static constexpr uint16_t calculate_pec(
        const std::span<const uint8_t>& data) {
        uint16_t remainder = 16;
        uint16_t address;

        for (uint i = 0; i < data.size(); i++) {
            address = (remainder >> 7) ^ data.data()[i];
            remainder = (remainder << 8) ^ pec15Table[address];
        }

        return (remainder * 2);
    }

    static constexpr array<uint8_t, 4> add_pec(
        const array<uint8_t, 2>& command) {
        array<uint8_t, 4> command_with_pec;
        command_with_pec[0] = command[0];
        command_with_pec[1] = command[1];
        uint16_t pec = calculate_pec(command);
        command_with_pec[2] = pec >> 8;
        command_with_pec[3] = pec;
        return command_with_pec;
    }

    template <AdcMode mode>
    static constexpr Command build_ADCV() {
        Command adcv{};
        adcv[0] = 0b00000010;

        if constexpr (mode == AdcMode::HZ_422 || mode == AdcMode::KHZ_1) {
            adcv[0] |= 0b0;
            adcv[1] |= 0b0 << 7;
        } else if constexpr (mode == AdcMode::KHZ_27 ||
                             mode == AdcMode::KHZ_14) {
            adcv[0] |= 0b0;
            adcv[1] |= 0b1 << 7;
        } else if constexpr (mode == AdcMode::KHZ_7 || mode == AdcMode::KHZ_3) {
            adcv[0] |= 0b1;
            adcv[1] |= 0b0 << 7;
        } else if constexpr (mode == AdcMode::HZ_26 || mode == AdcMode::KHZ_2) {
            adcv[0] |= 0b1;
            adcv[1] |= 0b1 << 7;
        }

        adcv[1] |= 0b11 << 5;

        if constexpr (mode == AdcMode::HZ_422 || mode == AdcMode::KHZ_27 ||
                      mode == AdcMode::KHZ_7 || mode == AdcMode::HZ_26) {
            adcv[1] |= (0b0 << 4);
        } else {
            adcv[1] |= (0b1 << 4);
        }

        adcv[1] |= 0b0000;

        return adcv;
    }

   public:
    consteval NetworkLink(const SPIConfig& config) : spi_link{config} {}

    void wake_up() {
        array<uint8_t, 1> byte{0XFF};
        spi_link.SPI_CS_turn_off();
        spi_link.SPI_transmit(byte);
        spi_link.SPI_transmit(byte);
        spi_link.SPI_CS_turn_on();
    }

    void config() {
        array<uint8_t, 10> command_packet;

        Command WRCFG{0x00, 0x01};

        array<uint8_t, 4> framed_command = add_pec(WRCFG);

        std::copy_n(framed_command.begin(), 4, command_packet.begin());

        // Harcoded WRCFG at the moment for testing
        command_packet[4] = 0x00;
        command_packet[5] = 0x00;
        command_packet[6] = 0x00;
        command_packet[7] = 0x00;
        command_packet[8] = 0x00;
        command_packet[9] = 0x00;

        spi_link.SPI_CS_turn_off();
        spi_link.SPI_transmit(command_packet);
        spi_link.SPI_CS_turn_on();
    }

    void start_cells_reading() {
        // Hardcoded ADC mode at the moment, only to test
        Command adcv = build_ADCV<AdcMode::KHZ_7>();

        array<uint8_t, 4> framed_command = add_pec(adcv);

        spi_link.SPI_CS_turn_off();
        spi_link.SPI_transmit(framed_command);
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

    array<uint16_t, N_LTC6810 * 6> read_cells() {
        array<uint8_t, (6 + 2) * N_LTC6810> cell_group_A{};
        array<uint8_t, (6 + 2) * N_LTC6810> cell_group_B{};
        array<uint16_t, N_LTC6810 * 6> data{};
        array<uint8_t, 4> command{};

        const Command RDCVA{0x00, 0x04};
        const Command RDCVB{0x00, 0x06};

        spi_link.SPI_CS_turn_off();
        command = add_pec(RDCVA);
        spi_link.SPI_transmit(command);
        spi_link.SPI_receive(cell_group_A);
        spi_link.SPI_CS_turn_on();

        // TODO check_pec

        spi_link.SPI_CS_turn_off();
        command = add_pec(RDCVB);
        spi_link.SPI_transmit(command);
        spi_link.SPI_receive(cell_group_B);
        spi_link.SPI_CS_turn_on();

        for (uint i = 0; i < N_LTC6810; ++i) {
            const auto offset = i * 8;

            data[i * 6 + 0] =
                cell_group_A[offset + 1] << 8 | cell_group_A[offset + 0];
            data[i * 6 + 1] =
                cell_group_A[offset + 3] << 8 | cell_group_A[offset + 2];
            data[i * 6 + 2] =
                cell_group_A[offset + 5] << 8 | cell_group_A[offset + 4];

            data[i * 6 + 3] =
                cell_group_B[offset + 1] << 8 | cell_group_B[offset + 0];
            data[i * 6 + 4] =
                cell_group_B[offset + 3] << 8 | cell_group_B[offset + 2];
            data[i * 6 + 5] =
                cell_group_B[offset + 5] << 8 | cell_group_B[offset + 4];
        }

        return data;
    }
};

#endif