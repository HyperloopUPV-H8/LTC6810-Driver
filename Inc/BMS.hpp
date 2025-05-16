#ifndef BMS_HPP
#define BMS_HPP

#include <cstdint>
#include <span>

#include "Battery.hpp"
#include "Driver.hpp"
#include "NetworkLink.hpp"
#include "StateMachine.hpp"

#define N_CELLS 6

const uint32_t TIME_SLEEP{1800};
const uint32_t TIME_REFUP{4400};

enum class CoreState {
    SLEEP,
    STANDBY,
    MEASURING_CELLS,
    READING_CELLS,
    MEASURING_GPIOS,
    READING_GPIOS
};

template <size_t N_LTC6810,
          void (*const SPI_TRANSMIT)(const std::span<uint8_t>),
          void (*const SPI_RECEIVE)(std::span<uint8_t>),
          void (*const SPI_CS_TURN_ON)(void),
          void (*const SPI_CS_TURN_OFF)(void), uint32_t (*const GET_TICK)(void),
          uint8_t TICK_RESOLUTION_MS, uint32_t PERIOD_MS>
class BMS {
    static consteval LTC6810::StateMachine<CoreState, 6, 7> make_core_sm() {
        constexpr LTC6810::State sleep = make_state(
            CoreState::SLEEP, sleep_action,
            LTC6810::Transition{CoreState::STANDBY, sleep_standby_guard});
        constexpr LTC6810::State standby =
            make_state(CoreState::STANDBY, standby_action,
                       LTC6810::Transition{CoreState::SLEEP, sleep_guard},
                       LTC6810::Transition{CoreState::MEASURING_CELLS,
                                           period_timeout_guard});
        constexpr LTC6810::State measuring_cells =
            make_state(CoreState::MEASURING_CELLS, measure_cells,
                       LTC6810::Transition{CoreState::MEASURING_GPIOS,
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

    constexpr static uint32_t (*const get_tick)(void) = GET_TICK;

    static inline LTC6810::Driver<N_LTC6810, LTC6810::AdcMode::KHZ_7> driver{
        LTC6810::SPIConfig{SPI_TRANSMIT, SPI_RECEIVE, SPI_CS_TURN_ON,
                           SPI_CS_TURN_OFF}};

    static inline array<Battery<N_CELLS>, N_LTC6810> batteries{};
    static inline array<uint16_t, N_LTC6810 * 4> GPIOs{};
    static inline uint32_t current_time{};
    static inline uint32_t sleep_reference{};
    static inline uint32_t last_read{};

    // Actions
    static void sleep_action() {}
    static void standby_action() { sleep_reference = get_tick(); }
    static void measure_cells() { driver.start_cell_conversion(); }
    static void read_cells() {
        auto cells = driver.read_cells();
        for (uint i{}; i < N_LTC6810; ++i) {
            for (uint j{}; j < N_CELLS; ++j) {
                if (cells[i][j]) {
                    batteries[i].cells[j] = cells[i][j].value();
                }
            }
        }
    }
    static void measure_GPIOs() { driver.start_GPIOs_conversion(); }
    static void read_GPIOs() {
        auto GPIOs = driver.read_GPIOs();
        for (uint i{}; i < N_LTC6810; ++i) {
            for (uint j{}; j < 4; ++j) {
                if (GPIOs[i][j]) {
                    batteries[i].GPIOs[j] = GPIOs[i][j].value();
                }
            }
        }
        last_read = get_tick();
    }

    // LTC6810::Transitions
    static bool sleep_standby_guard() {
        if ((current_time - last_read) * TICK_RESOLUTION_MS >= PERIOD_MS) {
            driver.wake_up();
            return true;
        }
        return false;
    }
    static bool period_timeout_guard() {
        return ((current_time - last_read) * TICK_RESOLUTION_MS >= PERIOD_MS);
    }
    static bool sleep_guard() {
        uint32_t sleep_time{(current_time - sleep_reference) *
                            TICK_RESOLUTION_MS};
        return sleep_time >= TIME_SLEEP;
    }
    static bool conversion_done_guard() { return driver.is_conv_done(); }

   public:
    static void update() {
        current_time = get_tick();
        core_sm.update();
    }

    static array<Battery<N_CELLS>, N_LTC6810>& get_data() { return batteries; }
};
#endif