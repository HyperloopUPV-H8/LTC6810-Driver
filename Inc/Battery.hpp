#ifndef BATTERY_HPP
#define BATTERY_HPP

#include <array>

template <std::size_t N_CELLS>
struct Battery {
    std::array<float, N_CELLS> cells{};
    std::array<float, 4> GPIOs{};
    float total_voltage{};
};

#endif