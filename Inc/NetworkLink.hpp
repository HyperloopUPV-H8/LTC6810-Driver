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

    void wake_up() const {
        array<uint8_t, 1> byte{0XFF};
        for (uint i{0}; i < N_LTC6810; ++i) {
            spi_link.SPI_CS_turn_off();
            spi_link.SPI_transmit(byte);
            spi_link.SPI_CS_turn_on();
        }
    }

    bool is_conv_done() const {
        std::array<uint8_t, 1> data;

        for (uint i{0}; i < (N_LTC6810 / 8) + 1; ++i) {
            spi_link.SPI_receive(data);
        }
        spi_link.SPI_receive(data);
        
        return data[0] > 0;
    }

    array<Register, N_LTC6810> read(Command command) const {
        array<Register, N_LTC6810> registers;

        spi_link.SPI_CS_turn_off();
        spi_link.SPI_transmit(command.command);

        array<uint8_t, 8> reg;
        for (uint i{0}; i < N_LTC6810; ++i) {
            spi_link.SPI_receive(reg);
            registers[i] = Register(std::move(reg));
        }

        spi_link.SPI_CS_turn_on();

        return registers;
    }

    void write(Command command, Register reg) const {
        spi_link.SPI_CS_turn_off();
        spi_link.SPI_transmit(command.command);
        spi_link.SPI_transmit(reg.reg);
        spi_link.SPI_CS_turn_on();
    }

    void send(Command command) const {
        spi_link.SPI_CS_turn_off();
        spi_link.SPI_transmit(command.command);
        spi_link.SPI_CS_turn_on();
    }
};
}  // namespace LTC6810
#endif