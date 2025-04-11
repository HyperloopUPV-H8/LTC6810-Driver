#pragma once
#include <stdint.h>

#include <array>

#include "Cell.hpp"

template <uint8_t N_CELLS>
class Battery {
   public:
    Battery() = default;

    std::array<Cell, N_CELLS> cells{};
    float total_voltage{};

    float temperature_1{};
    float temperature_2{};

    void update_total_voltage();
};

template <uint8_t N_CELLS>
void Battery<N_CELLS>::update_total_voltage() {
    total_voltage = 0;
    for (const auto& cell : cells) {
        total_voltage += cell.voltage;
    }
}