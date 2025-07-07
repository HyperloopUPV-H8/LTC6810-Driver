#ifndef LTC6810_HPP
#define LTC6810_HPP

#include <array>

constexpr size_t N_GPIOS{4};
constexpr float CONV_RATE_TIME{1.0};

namespace LTC6810Driver {
template <std::size_t N_CELLS, size_t READING_PERIOD_US>
class LTC6810 {
    const float CONV_STEP =
        1 / ((10 / (static_cast<float>(READING_PERIOD_US) / 1000000)) *
             CONV_RATE_TIME);

   public:
    std::array<float, N_CELLS> cells{};
    std::array<float, N_GPIOS> GPIOs{};
    float conv_rate{1.0};
    float total_voltage{};

    void conv_successful() {
        if (conv_rate < 1.0) {
            conv_rate += CONV_STEP;
        }
    }

    void conv_failed() {
        if (conv_rate > 0.0) {
            conv_rate -= CONV_STEP;
        }
    }
};
}  // namespace LTC6810Driver

#endif