#ifndef DRIVER_HPP
#define DRIVER_HPP

#include "StateMachine.hpp"
namespace Driver {
enum class CoreState {
    SLEEP,
    STANDBY,
    REFUP,
    MEASURE,
    EXTENDED_BALANCING,
    DTM_MEASURE
};

enum IsoSPIState { IDLE, READY, ACTIVE };

using CoreSmMachine = StateMachineBuilder<CoreState, 6, 16>;
using IsoSPIMachine = StateMachineBuilder<IsoSPIState, 3, 4>;

class Runtime {
    CoreSmMachine::StateMachine core_sm;
    IsoSPIMachine::StateMachine isospi_sm;

   public:
    Runtime(CoreSmMachine::StateMachine core_sm,
            IsoSPIMachine::StateMachine isospi_sm)
        : core_sm(core_sm), isospi_sm(isospi_sm) {}
};

class Builder {
    CoreSmMachine core_sm_builder;
    IsoSPIMachine isospi_sm_builder;

   public:
    consteval Builder();

    Runtime init() {
        return Runtime(core_sm_builder.init(CoreState::SLEEP),
                       isospi_sm_builder.init(IsoSPIState::IDLE));
    }

    // Actions
    static void sleep_action();
    static void standby_action();
    static void refup_action();
    static void measure_action();
    static void extended_balancing_action();
    static void dtm_measure_action();

    // Transitions
    static bool sleep_standby_guard();

    static bool standby_sleep_guard();
    static bool standby_refup_guard();
    static bool standby_measure_guard();
    static bool standby_extended_balancing_guard();

    static bool refup_sleep_guard();
    static bool refup_standby_guard();
    static bool refup_measure_guard();
    static bool refup_extended_balancing_guard();

    static bool measure_refup_guard();
    static bool measure_standby_guard();

    static bool extended_balancing_sleep_guard();
    static bool extended_balancing_standby_guard();
    static bool extended_balancing_dtm_measure_guard();

    static bool dtm_measure_standby_guard();
    static bool dtm_measure_extended_balancing_guard();

   public:
    static void init(const float &freq);
};
}  // namespace Driver
#endif