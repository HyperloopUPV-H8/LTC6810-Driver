#ifndef BMS_HPP
#define BMS_HPP

#include <cstdint>
#include <numeric>
#include <span>

#include "Driver.hpp"
#include "LTC6810.hpp"
#include "NetworkLink.hpp"
#include "StateMachine.hpp"

constexpr size_t N_CELLS{6};
constexpr bool DIAG{true};
constexpr int32_t TIME_SLEEP_US{1800000};
constexpr int32_t TIME_REFUP_US{4400};

enum class CoreState {
    SLEEP,
    STANDBY,
    MEASURING_CELLS,
    READING_CELLS,
    MEASURING_GPIOS,
    READING_GPIOS
};

template <typename T>
concept BMSConfig = requires(T) {
    { std::unsigned_integral<decltype(T::n_LTC6810)> };
    {
        T::SPI_transmit(std::declval<std::span<uint8_t>>())
    } -> std::same_as<void>;
    {
        T::SPI_receive(std::declval<std::span<uint8_t>>())
    } -> std::same_as<void>;
    { T::SPI_CS_turn_off() } -> std::same_as<void>;
    { T::SPI_CS_turn_on() } -> std::same_as<void>;
    { T::get_tick() } -> std::same_as<int32_t>;
    { std::integral<decltype(T::tick_resolution_us)> };
    { std::integral<decltype(T::period_us)> };
    { std::integral<decltype(T::conv_rate_time_ms)> };
};

template <BMSConfig config>
class BMS {
    static consteval LTC6810Driver::StateMachine<CoreState, 6, 7>
    make_core_sm() {
        constexpr LTC6810Driver::State sleep =
            make_state(CoreState::SLEEP, sleep_action,
                       LTC6810Driver::Transition{CoreState::MEASURING_CELLS,
                                                 sleep_timeout_guard});
        constexpr LTC6810Driver::State standby =
            make_state(CoreState::STANDBY, standby_action,
                       LTC6810Driver::Transition{CoreState::SLEEP, sleep_guard},
                       LTC6810Driver::Transition{CoreState::MEASURING_CELLS,
                                                 period_timeout_guard});
        constexpr LTC6810Driver::State measuring_cells =
            make_state(CoreState::MEASURING_CELLS, measure_cells,
                       LTC6810Driver::Transition{CoreState::READING_CELLS,
                                                 conversion_done_guard});
        constexpr LTC6810Driver::State reading_cells =
            make_state(CoreState::READING_CELLS, read_cells,
                       LTC6810Driver::Transition{CoreState::MEASURING_GPIOS,
                                                 +[]() { return true; }});
        constexpr LTC6810Driver::State measuring_gpios =
            make_state(CoreState::MEASURING_GPIOS, measure_GPIOs,
                       LTC6810Driver::Transition{CoreState::READING_GPIOS,
                                                 conversion_done_guard});
        constexpr LTC6810Driver::State reading_gpios =
            make_state(CoreState::READING_GPIOS, read_GPIOs,
                       LTC6810Driver::Transition{CoreState::STANDBY,
                                                 +[]() { return true; }});

        return make_state_machine(CoreState::SLEEP, sleep, standby,
                                  measuring_cells, reading_cells,
                                  measuring_gpios, reading_gpios);
    }

    static inline LTC6810Driver::StateMachine<CoreState, 6, 7> core_sm{
        make_core_sm()};

    static inline LTC6810Driver::Driver<config::n_LTC6810> driver{
        LTC6810Driver::SPIConfig{config::SPI_transmit, config::SPI_receive,
                                 config::SPI_CS_turn_off,
                                 config::SPI_CS_turn_on}};

    static inline uint32_t init_conv{};
    static inline uint32_t final_conv{};

    static inline array<LTC6810Driver::LTC6810<N_CELLS, config::period_us,
                                               config::conv_rate_time_ms>,
                        config::n_LTC6810>
        ltcs{};

    static inline int32_t current_time{};
    static inline int32_t sleep_reference{};
    static inline int32_t last_read{};

    static inline int32_t time_to_read{};

    // Actions
    static void sleep_action() {}
    static void standby_action() {
        sleep_reference = config::get_tick() * config::tick_resolution_us;
    }
    static void measure_cells() {
        init_conv = config::get_tick() * config::tick_resolution_us;
        driver.start_cell_conversion();
    }
    static void read_cells() {
        auto cells = driver.read_cells();
        for (uint i{}; i < config::n_LTC6810; ++i) {
            for (uint j{}; j < N_CELLS; ++j) {
                if (cells[i][j]) {
                    ltcs[i].cells[j] = cells[i][j].value();
                    if constexpr (DIAG) {
                        ltcs[i].conv_successful();
                    }
                } else if constexpr (DIAG) {
                    ltcs[i].conv_failed();
                }
            }
            if (cells[i][6]) {
                ltcs[i].total_voltage = cells[i][6].value();
            }
        }
    }
    static void measure_GPIOs() { driver.start_GPIOs_conversion(); }
    static void read_GPIOs() {
        auto GPIOs = driver.read_GPIOs();
        for (uint i{}; i < config::n_LTC6810; ++i) {
            for (uint j{}; j < 4; ++j) {
                if (GPIOs[i][j]) {
                    ltcs[i].GPIOs[j] = GPIOs[i][j].value();
                    if constexpr (DIAG) {
                        ltcs[i].conv_successful();
                    }
                } else if constexpr (DIAG) {
                    ltcs[i].conv_failed();
                }
            }
        }

        final_conv = config::get_tick() * config::tick_resolution_us;
        time_to_read = final_conv - init_conv;
        int32_t timestamp = config::get_tick() * config::tick_resolution_us;
        reading_period = timestamp - last_read;
        last_read = timestamp;

        if (reading_period > config::period_us + config::period_us * 0.1) {
            driver.faster_conv();
        }
    }

    // Transitions
    static bool sleep_timeout_guard() {
        if ((current_time - last_read) >= (config::period_us - time_to_read)) {
            driver.wake_up();
            return true;
        }
        return false;
    }
    static bool period_timeout_guard() {
        auto period_time{current_time - last_read};
        return (period_time >= (config::period_us - time_to_read));
    }
    static bool sleep_guard() {
        return (current_time - sleep_reference) >= TIME_SLEEP_US;
    }
    static bool conversion_done_guard() { return driver.is_conv_done(); }

   public:
    static inline int32_t reading_period{};

    static void update() {
        current_time = config::get_tick() * config::tick_resolution_us;
        core_sm.update();
    }

    static array<LTC6810Driver::LTC6810<N_CELLS, config::period_us,
                                        config::conv_rate_time_ms>,
                 config::n_LTC6810>&
    get_data() {
        return ltcs;
    }

    static float get_period() { return reading_period; }
};
#endif