#ifndef COMMAND_HPP
#define COMMAND_HPP

#include <array>
#include <span>

using std::array;
using std::span;

namespace LTC6810 {
const uint16_t CRC15_POLY = 0x4599;

constexpr array<uint16_t, 256> init_PEC15_Table() {
    array<uint16_t, 256> pec15Table;
    for (int i = 0; i < 256; i++) {
        uint16_t remainder = i << 7;
        for (int bit = 8; bit > 0; --bit) {
            if (remainder & 0x4000) {
                remainder = ((remainder << 1));
                remainder = (remainder ^ CRC15_POLY);
            } else {
                remainder = ((remainder << 1));
            }
        }
        pec15Table[i] = remainder & 0xFFFF;
    }
    return pec15Table;
}

constexpr array<uint16_t, 256> pec15Table{init_PEC15_Table()};

constexpr uint16_t calculate_pec(std::span<uint8_t> data) {
    uint16_t remainder = 16;
    uint16_t address;

    for (uint i = 0; i < data.size(); i++) {
        address = static_cast<uint8_t>(remainder >> 7) ^ data.data()[i];
        remainder = (remainder << 8) ^ pec15Table[address];
    }

    return (remainder * 2);
}

struct Command {
    array<uint8_t, 4> command{};

    constexpr Command() = default;
    constexpr Command(uint16_t data) {
        command[0] = data >> 8;
        command[1] = data;
        uint16_t pec{calculate_pec({command.data(), 2})};
        command[2] = pec >> 8;
        command[3] = pec;
    }
};

struct Register {
    array<uint8_t, 8> reg{};

    constexpr Register() = default;
    constexpr Register(array<uint8_t, 8>&& reg) : reg(std::move(reg)) {}
    constexpr Register(array<uint8_t, 6>&& data) {
        std::move(data.begin(), data.end(), reg.begin());
        uint16_t pec{calculate_pec({reg.data(), 6})};
        reg[6] = pec >> 8;
        reg[7] = pec;
    }

    constexpr bool is_pec_valid() {
        uint16_t valid_pec{calculate_pec({reg.data(), 6})};
        uint8_t high = static_cast<uint8_t>(valid_pec >> 8);
        uint8_t low = static_cast<uint8_t>(valid_pec);
        return high == reg[6] && low == reg[7];
    }

    array<uint16_t, 3> get_16bit_data() const {
        array<uint16_t, 3> data;
        for (uint i{0}; i < 3; ++i) {
            data[i] = reg[2 * i] | (reg[2 * i + 1] << 8);
        }
        return data;
    }
};

}  // namespace LTC6810
#endif
