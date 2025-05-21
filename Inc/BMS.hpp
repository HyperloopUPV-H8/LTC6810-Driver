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
constexpr uint32_t TIME_SLEEP_US{1800000};
constexpr uint32_t TIME_REFUP_US{4400};

enum class CoreState {
    SLEEP,
    STANDBY,
    MEASURING_CELLS,
    READING_CELLS,
    MEASURING_GPIOS,
    READING_GPIOS
};

struct BMSDiag {
   private:
    uint n_fail_conv{};
    uint n_success_conv{};
    void calculate_rate() {
        success_conv_rate =
            static_cast<float>(n_success_conv) / (n_success_conv + n_fail_conv);
    }

   public:
    float success_conv_rate{};
    uint32_t reading_period{};
    uint32_t time_to_read{};

    void conv_succesfull() {
        ++n_success_conv;
        calculate_rate();
    };
    void conv_failed() {
        ++n_fail_conv;
        calculate_rate();
    }
};

template <size_t N_LTC6810,
          void (*const SPI_TRANSMIT)(const std::span<uint8_t>),
          void (*const SPI_RECEIVE)(std::span<uint8_t>),
          void (*const SPI_CS_TURN_ON)(void),
          void (*const SPI_CS_TURN_OFF)(void), uint32_t (*const GET_TICK)(void),
          uint32_t TICK_RESOLUTION_US, uint32_t PERIOD_US>
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

    constexpr static uint32_t (*const get_tick)(void) = GET_TICK;

    static inline LTC6810::Driver<N_LTC6810> driver{LTC6810::SPIConfig{
        SPI_TRANSMIT, SPI_RECEIVE, SPI_CS_TURN_ON, SPI_CS_TURN_OFF}};

    static inline BMSDiag bms_diag{};
    static inline uint32_t init_conv{};
    static inline uint32_t final_conv{};

    static inline array<Battery<N_CELLS>, N_LTC6810> batteries{};
    static inline array<uint16_t, N_LTC6810 * 4> GPIOs{};

    static inline uint32_t current_time{};
    static inline uint32_t sleep_reference{};
    static inline uint32_t last_read{};

    // Actions
    static void sleep_action() {}
    static void standby_action() {
        sleep_reference = get_tick() * TICK_RESOLUTION_US;
    }
    static void measure_cells() {
        init_conv = get_tick() * TICK_RESOLUTION_US;
        driver.start_cell_conversion();
    }
    static void read_cells() {
        auto cells = driver.read_cells();
        for (uint i{}; i < N_LTC6810; ++i) {
            for (uint j{}; j < N_CELLS; ++j) {
                if (cells[i][j]) {
                    batteries[i].cells[j] = cells[i][j].value();
                    if constexpr (DIAG) {
                        bms_diag.conv_succesfull();
                    }
                } else if constexpr (DIAG) {
                    bms_diag.conv_failed();
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
        for (uint i{}; i < N_LTC6810; ++i) {
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

        final_conv = get_tick() * TICK_RESOLUTION_US;
        bms_diag.time_to_read = final_conv - init_conv;
        auto timestamp = get_tick() * TICK_RESOLUTION_US;
        bms_diag.reading_period = timestamp - last_read;
        last_read = timestamp;

        if (bms_diag.reading_period > PERIOD_US + PERIOD_US * 0.1) {
            driver.faster_conv();
        }
    }

    // Transitions
    static bool sleep_standby_guard() {
        if ((current_time - last_read) >= (PERIOD_US - bms_diag.time_to_read)) {
            driver.wake_up();
            return true;
        }
        return false;
    }
    static bool period_timeout_guard() {
        auto period_time{current_time - last_read};
        return (period_time >= (PERIOD_US - bms_diag.time_to_read) &&
                period_time < TIME_SLEEP_US);
    }
    static bool sleep_guard() {
        return (current_time - sleep_reference) >= TIME_SLEEP_US;
    }
    static bool conversion_done_guard() { return driver.is_conv_done(); }

   public:
    static void update() {
        current_time = get_tick() * TICK_RESOLUTION_US;
        core_sm.update();
    }

    static array<Battery<N_CELLS>, N_LTC6810>& get_data() { return batteries; }
    static BMSDiag& get_diag() { return bms_diag; }
};
#endif