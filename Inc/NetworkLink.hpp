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
namespace LTC6810 {

struct SPIConfig {
    void (*const SPI_transmit)(const std::span<uint8_t>);
    void (*const SPI_receive)(std::span<uint8_t>);
    void (*const SPI_CS_turn_on)(void);
    void (*const SPI_CS_turn_off)(void);
};

template <size_t N_LTC6810>
class NetworkLink {
    const SPIConfig spi_link;

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

    bool is_conv_done() {
        std::array<uint8_t, 1> data;
        spi_link.SPI_receive(data);
        if (data[0] > 0) {
            spi_link.SPI_CS_turn_on();
            return true;
        }
        return false;
    }

    array<Register, N_LTC6810> read(Command command) {
        array<Register, N_LTC6810> registers;

        spi_link.SPI_CS_turn_off();
        spi_link.SPI_transmit(command.command);

        array<uint8_t, 8> reg;
        for (uint i{0}; i < N_LTC6810; ++i) {
            spi_link.SPI_receive(reg);
            registers[i] = Register(std::move(reg));
        }

        return registers;
    }

    void write(Command command, Register reg) {}

    void send(Command command) {}
};
}  // namespace LTC6810
#endif