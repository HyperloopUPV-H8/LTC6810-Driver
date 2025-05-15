#ifndef BMS_HPP
#define BMS_HPP

#include <cstdint>
#include <span>

#include "Battery.hpp"
#include "Driver.hpp"
#include "NetworkLink.hpp"
#include "StateMachine.hpp"

const uint32_t TIME_SLEEP{1800};
const uint32_t TIME_REFUP{4400};

enum class CoreState {
    SLEEP,
    STANDBY,
    REFUP,
    MEASURE,
    EXTENDED_BALANCING,
    DTM_MEASURE
};
enum IsoSPIState { IDLE, READY, ACTIVE };

template <size_t N_LTC6810, size_t N_CELLS,
          void (*const SPI_TRANSMIT)(const std::span<uint8_t>),
          void (*const SPI_RECEIVE)(std::span<uint8_t>),
          void (*const SPI_CS_TURN_ON)(void),
          void (*const SPI_CS_TURN_OFF)(void), uint32_t (*const GET_TICK)(void),
          uint8_t TICK_RESOLUTION_MS, uint32_t PERIOD_MS>
class BMS {
    static consteval LTC6810::StateMachine<CoreState, 6, 16> make_core_sm() {
        constexpr LTC6810::State sleep = make_state(
            CoreState::SLEEP, sleep_action,
            LTC6810::Transition{CoreState::STANDBY, sleep_standby_guard});
        constexpr LTC6810::State standby = make_state(
            CoreState::STANDBY, standby_action,
            LTC6810::Transition{CoreState::SLEEP, standby_sleep_guard},
            LTC6810::Transition{CoreState::REFUP, standby_refup_guard},
            LTC6810::Transition{CoreState::MEASURE, standby_measure_guard},
            LTC6810::Transition{CoreState::EXTENDED_BALANCING,
                                standby_extended_balancing_guard});
        constexpr LTC6810::State refup = make_state(
            CoreState::REFUP, refup_action,
            LTC6810::Transition{CoreState::SLEEP, refup_sleep_guard},
            LTC6810::Transition{CoreState::STANDBY, refup_standby_guard},
            LTC6810::Transition{CoreState::MEASURE, refup_measure_guard},
            LTC6810::Transition{CoreState::EXTENDED_BALANCING,
                                refup_extended_balancing_guard});
        constexpr LTC6810::State measure = make_state(
            CoreState::MEASURE, measure_action,
            LTC6810::Transition{CoreState::REFUP, measure_refup_guard},
            LTC6810::Transition{CoreState::STANDBY, measure_standby_guard});
        constexpr LTC6810::State extended_balancing = make_state(
            CoreState::EXTENDED_BALANCING, extended_balancing_action,
            LTC6810::Transition{CoreState::SLEEP,
                                extended_balancing_sleep_guard},
            LTC6810::Transition{CoreState::STANDBY,
                                extended_balancing_standby_guard},
            LTC6810::Transition{CoreState::DTM_MEASURE,
                                extended_balancing_dtm_measure_guard});
        constexpr LTC6810::State dtm_measure = make_state(
            CoreState::DTM_MEASURE, dtm_measure_action,
            LTC6810::Transition{CoreState::STANDBY, dtm_measure_standby_guard},
            LTC6810::Transition{CoreState::EXTENDED_BALANCING,
                                dtm_measure_extended_balancing_guard});

        return make_state_machine(CoreState::SLEEP, sleep, standby, refup,
                                  measure, extended_balancing, dtm_measure);
    }

    static consteval LTC6810::StateMachine<IsoSPIState, 3, 4> make_isospi_sm() {
        constexpr LTC6810::State idle = make_state(
            IsoSPIState::IDLE, idle_action,
            LTC6810::Transition{IsoSPIState::READY, idle_ready_guard});
        constexpr LTC6810::State ready = make_state(
            IsoSPIState::READY, ready_action,
            LTC6810::Transition{IsoSPIState::IDLE, ready_idle_guard},
            LTC6810::Transition{IsoSPIState::ACTIVE, ready_active_guard});
        constexpr LTC6810::State active = make_state(
            IsoSPIState::ACTIVE, active_action,
            LTC6810::Transition{IsoSPIState::READY, active_ready_guard});

        return make_state_machine(IsoSPIState::IDLE, idle, ready, active);
    }

    static inline LTC6810::StateMachine<CoreState, 6, 16> core_sm{
        make_core_sm()};
    static inline LTC6810::StateMachine<IsoSPIState, 3, 4> isospi_sm{
        make_isospi_sm()};

    constexpr static uint32_t (*const get_tick)(void) = GET_TICK;

    static inline LTC6810::Driver<N_LTC6810, LTC6810::AdcMode::KHZ_7> driver{
        LTC6810::SPIConfig{SPI_TRANSMIT, SPI_RECEIVE, SPI_CS_TURN_ON,
                           SPI_CS_TURN_OFF}};

    static inline array<Battery<N_CELLS>, N_LTC6810> batteries{};
    static inline array<uint16_t, N_LTC6810 * 4> GPIOs{};
    static inline bool waked_up{false};
    static inline bool cells_read{false};
    static inline bool GPIOs_read{false};
    static inline uint32_t current_time{};
    static inline uint32_t sleep_reference{};
    static inline uint32_t last_read{};
    static inline uint32_t read_time{};

    // Actions
    // Core SM
    static void sleep_action() {}
    static void standby_action() { sleep_reference = get_tick(); }
    static void refup_action() { sleep_reference = get_tick(); }
    static void measure_action() {}
    static void extended_balancing_action() {}
    static void dtm_measure_action() {}

    // IsoSPI SM
    static void idle_action() {}
    static void ready_action() {}
    static void active_action() {}

    // LTC6810::Transitions
    // Core SM
    static bool sleep_standby_guard() {
        if ((current_time - last_read) * TICK_RESOLUTION_MS >= PERIOD_MS) {
            driver.wake_up();
            return true;
        }
        return false;
    }

    static bool standby_sleep_guard() {
        uint32_t sleep_time{(current_time - sleep_reference) *
                            TICK_RESOLUTION_MS};
        if (sleep_time >= TIME_SLEEP) {
            return true;
        }
        return false;
    }
    static bool standby_refup_guard() {
        uint32_t time{(current_time - sleep_reference) * TICK_RESOLUTION_MS};
        if (REFON && time >= TIME_REFUP) {
            return true;
        }
        return false;
    }
    static bool standby_measure_guard() {
        if (!cells_read) {
            driver.start_cell_conversion();
            return true;
        }
        if (!GPIOs_read) {
            driver.start_GPIOs_conversion();
            return true;
        }
        return false;
    }
    static bool standby_extended_balancing_guard() { return false; }

    static bool refup_sleep_guard() {
        uint32_t time{(current_time - sleep_reference) * TICK_RESOLUTION_MS};
        if (time >= TIME_SLEEP) {
            return true;
        }
        return false;
    }
    static bool refup_standby_guard() { return false; }
    static bool refup_measure_guard() { return false; }
    static bool refup_extended_balancing_guard() { return false; }

    static bool measure_refup_guard() { return false; }
    static bool measure_standby_guard() {
        if (driver.is_conv_done()) {
            last_read = get_tick();
            if (!cells_read) {
                driver.read_cells();
                //TODO
                cells_read = true;
                return true;
            }
            if (!GPIOs_read) {
                driver.read_GPIOs();
                //TODO
                GPIOs_read = true;
                return true;
            }
        }
        return false;
    }

    static bool extended_balancing_sleep_guard() { return false; }
    static bool extended_balancing_standby_guard() { return false; }
    static bool extended_balancing_dtm_measure_guard() { return false; }

    static bool dtm_measure_standby_guard() { return false; }
    static bool dtm_measure_extended_balancing_guard() { return false; }

    // IsoSPI SM
    static bool idle_ready_guard() { return false; }
    static bool ready_idle_guard() { return false; }
    static bool ready_active_guard() { return false; }
    static bool active_ready_guard() { return false; }

   public:
    static void update() {
        current_time = get_tick();
        uint32_t measure_time{(current_time - last_read) * TICK_RESOLUTION_MS};
        if (measure_time >= PERIOD_MS) {
            cells_read = false;
            GPIOs_read = false;
        }
        core_sm.update();
        isospi_sm.update();
    }

    static array<Battery<N_CELLS>, N_LTC6810>& get_data() { return batteries; }
};
#endif