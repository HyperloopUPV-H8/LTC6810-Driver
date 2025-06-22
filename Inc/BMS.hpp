#ifndef BMS_HPP
#define BMS_HPP

#include <cstdint>
#include <span>

#include "Battery.hpp"
#include "Driver.hpp"
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
};

template <std::size_t N_LTC66810>
struct BMSDiag {
   private:
    array<uint, N_LTC66810> n_fail_conv{};
    array<uint, N_LTC66810> n_success_conv{};
    void calculate_rate(uint id) {
        success_conv_rates[id] = static_cast<float>(n_success_conv[id]) /
                                 (n_success_conv[id] + n_fail_conv[id]);
    }

   public:
    array<float, N_LTC66810> success_conv_rates{};
    int32_t reading_period{};
    int32_t time_to_read{};

    void conv_succesfull(uint id) {
        ++n_success_conv[id];
        calculate_rate(id);
    };
    void conv_failed(uint id) {
        ++n_fail_conv[id];
        calculate_rate(id);
    }
};

template <BMSConfig config>
class BMS {
    static consteval LTC6810::StateMachine<CoreState, 6, 7> make_core_sm() {
        constexpr LTC6810::State sleep =
            make_state(CoreState::SLEEP, sleep_action,
                       LTC6810::Transition{CoreState::MEASURING_CELLS,
                                           sleep_timeout_guard});
        constexpr LTC6810::State standby =
            make_state(CoreState::STANDBY, standby_action,
                       LTC6810::Transition{CoreState::SLEEP, sleep_guard},
                       LTC6810::Transition{CoreState::MEASURING_CELLS,
                                           period_timeout_guard});
        constexpr LTC6810::State measuring_cells =
            make_state(CoreState::MEASURING_CELLS, measure_cells,
                       LTC6810::Transition{CoreState::READING_CELLS,
                                           conversion_done_guard});
        constexpr LTC6810::State reading_cells =
            make_state(CoreState::READING_CELLS, read_cells,
                       LTC6810::Transition{CoreState::MEASURING_GPIOS,
                                           +[]() { return true; }});
        constexpr LTC6810::State measuring_gpios =
            make_state(CoreState::MEASURING_GPIOS, measure_GPIOs,
                       LTC6810::Transition{CoreState::READING_GPIOS,
                                           conversion_done_guard});
        constexpr LTC6810::State reading_gpios = make_state(
            CoreState::READING_GPIOS, read_GPIOs,
            LTC6810::Transition{CoreState::STANDBY, +[]() { return true; }});

        return make_state_machine(CoreState::SLEEP, sleep, standby,
                                  measuring_cells, reading_cells,
                                  measuring_gpios, reading_gpios);
    }

    static inline LTC6810::StateMachine<CoreState, 6, 7> core_sm{
        make_core_sm()};

    static inline LTC6810::Driver<config::n_LTC6810> driver{
        LTC6810::SPIConfig{config::SPI_transmit, config::SPI_receive,
                           config::SPI_CS_turn_off, config::SPI_CS_turn_on}};

    static inline BMSDiag<config::n_LTC6810> bms_diag{};
    static inline uint32_t init_conv{};
    static inline uint32_t final_conv{};

    static inline array<Battery<N_CELLS>, config::n_LTC6810> batteries{};
    static inline array<uint16_t, config::n_LTC6810 * 4> GPIOs{};

    static inline int32_t current_time{};
    static inline int32_t sleep_reference{};
    static inline int32_t last_read{};

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
                    batteries[i].cells[j] = cells[i][j].value();
                    if constexpr (DIAG) {
                        bms_diag.conv_succesfull(i);
                    }
                } else if constexpr (DIAG) {
                    bms_diag.conv_failed(i);
                }
            }
            if (cells[i][6]) {
                batteries[i].total_voltage = cells[i][6].value();
            }
        }
    }
    static void measure_GPIOs() { driver.start_GPIOs_conversion(); }
    static void read_GPIOs() {
        auto GPIOs = driver.read_GPIOs();
        for (uint i{}; i < config::n_LTC6810; ++i) {
            for (uint j{}; j < 4; ++j) {
                if (GPIOs[i][j]) {
                    batteries[i].GPIOs[j] = GPIOs[i][j].value();
                    if constexpr (DIAG) {
                        bms_diag.conv_succesfull();
                    }
                } else if constexpr (DIAG) {
                    bms_diag.conv_failed();
                }
            }
        }

        final_conv = config::get_tick() * config::tick_resolution_us;
        bms_diag.time_to_read = final_conv - init_conv;
        auto timestamp = config::get_tick() * config::tick_resolution_us;
        bms_diag.reading_period = timestamp - last_read;
        last_read = timestamp;

        if (bms_diag.reading_period >
            config::period_us + config::period_us * 0.1) {
            driver.faster_conv();
        }
    }

    // Transitions
    static bool sleep_timeout_guard() {
        if ((current_time - last_read) >=
            (config::period_us - bms_diag.time_to_read)) {
            driver.wake_up();
            return true;
        }
        return false;
    }
    static bool period_timeout_guard() {
        auto period_time{current_time - last_read};
        return (period_time >= (config::period_us - bms_diag.time_to_read));
    }
    static bool sleep_guard() {
        return (current_time - sleep_reference) >= TIME_SLEEP_US;
    }
    static bool conversion_done_guard() { return driver.is_conv_done(); }

   public:
    static void update() {
        current_time = config::get_tick() * config::tick_resolution_us;
        core_sm.update();
    }

    static array<Battery<N_CELLS>, config::n_LTC6810>& get_data() {
        return batteries;
    }
    static BMSDiag<config::n_LTC6810>& get_diag() { return bms_diag; }
};
#endif