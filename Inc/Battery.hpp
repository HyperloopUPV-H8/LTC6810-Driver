#pragma once
#include <stdint.h>

#include <array>

#include "Cell.hpp"

template <std::size_t N_CELLS>
class Battery {
   public:
    consteval Battery() : cells{}, total_voltage{}, temperature_1{}, temperature_2{} {};

    std::array<Cell, N_CELLS> cells{};
    float total_voltage{};

    float temperature_1{};
    float temperature_2{};

    void update_total_voltage();
};

template <std::size_t N_CELLS>
void Battery<N_CELLS>::update_total_voltage() {
    total_voltage = 0;
    for (const auto& cell : cells) {
        total_voltage += cell.voltage;
    }
}