#include "Driver.hpp"
namespace Driver {
consteval Builder::Builder() {
    constexpr State sleep = make_state(
        CoreState::SLEEP, []() {},
        Transition{CoreState::STANDBY, []() { return true; }});
    constexpr State standby = make_state(
        CoreState::STANDBY, []() {},
        Transition{CoreState::SLEEP, []() { return true; }},
        Transition{CoreState::REFUP, []() { return true; }},
        Transition{CoreState::MEASURE, []() { return true; }},
        Transition{CoreState::EXTENDED_BALANCING, []() { return true; }});
    constexpr State refup = make_state(
        CoreState::REFUP, []() {},
        Transition{CoreState::SLEEP, []() { return true; }},
        Transition{CoreState::STANDBY, []() { return true; }},
        Transition{CoreState::MEASURE, []() { return true; }},
        Transition{CoreState::EXTENDED_BALANCING, []() { return true; }});
    constexpr State measure = make_state(
        CoreState::MEASURE, []() {},
        Transition{CoreState::REFUP, []() { return true; }},
        Transition{CoreState::STANDBY, []() { return true; }});
    constexpr State extended_balancing = make_state(
        CoreState::EXTENDED_BALANCING, []() {},
        Transition{CoreState::SLEEP, []() { return true; }},
        Transition{CoreState::STANDBY, []() { return true; }},
        Transition{CoreState::DTM_MEASURE, []() { return true; }});
    constexpr State dtm_measure = make_state(
        CoreState::DTM_MEASURE, []() {},
        Transition{CoreState::STANDBY, []() { return true; }},
        Transition{CoreState::EXTENDED_BALANCING, []() { return true; }});

    core_sm_builder = make_state_machine<CoreState>(
        sleep, standby, refup, measure, extended_balancing, dtm_measure);

    constexpr State idle = make_state(
        IsoSPIState::IDLE, []() {},
        Transition{IsoSPIState::READY, []() { return true; }});
    constexpr State ready = make_state(
        IsoSPIState::READY, []() {},
        Transition{IsoSPIState::IDLE, []() { return true; }},
        Transition{IsoSPIState::ACTIVE, []() { return true; }});
    constexpr State active = make_state(
        IsoSPIState::ACTIVE, []() {},
        Transition{IsoSPIState::READY, []() { return true; }});
    isospi_sm_builder = make_state_machine<IsoSPIState>(idle, ready, active);
}

// // Actions
// void Driver::sleep_action() {}
// void Driver::standby_action() {}
// void Driver::refup_action() {}
// void Driver::measure_action() {}
// void Driver::extended_balancing_action() {}
// void Driver::dtm_measure_action() {}

// // Transitions
// bool Driver::sleep_standby_transition() { return false; }

// bool Driver::standby_sleep_transition() { return false; }
// bool Driver::standby_refup_transition() { return false; }
// bool Driver::standby_measure_transition() { return false; }
// bool Driver::standby_extended_balancing_transition() { return false; }

// bool Driver::refup_sleep_transition() { return false; }
// bool Driver::refup_standby_transition() { return false; }
// bool Driver::refup_measure_transition() { return false; }
// bool Driver::refup_extended_balancing_transition() { return false; }

// bool Driver::measure_refup_transition() { return false; }
// bool Driver::measure_standby_transition() { return false; }

// bool Driver::extended_balancing_sleep_transition() { return false; }
// bool Driver::extended_balancing_standby_transition() { return false; }
// bool Driver::extended_balancing_dtm_measure_transition() { return false; }

// bool Driver::dtm_measure_standby_transition() { return false; }
// bool Driver::dtm_measure_extended_balancing_transition() { return false; }

// void Driver::init(const float &freq) {
//     state_id sleep = core_sm.add_state(sleep_action);
//     state_id standby = core_sm.add_state(standby_action);
//     state_id refup = core_sm.add_state(refup_action);
//     state_id measure = core_sm.add_state(measure_action);
//     state_id extended_balancing =
//     core_sm.add_state(extended_balancing_action); state_id dtm_measure =
//     core_sm.add_state(dtm_measure_action);

//     core_sm.add_transition(sleep, sleep_standby_transition, standby);
// }
}  // namespace Driver