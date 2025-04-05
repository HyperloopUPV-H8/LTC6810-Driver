#include "Driver.hpp"

StateMachine Driver::core_sm = StateMachine();
StateMachine Driver::isospi_sm = StateMachine();

// Actions
void Driver::sleep_action() {}
void Driver::standby_action() {}
void Driver::refup_action() {}
void Driver::measure_action() {}
void Driver::extended_balancing_action() {}
void Driver::dtm_measure_action() {}

// Transitions
bool Driver::sleep_standby_transition() { return false; }

bool Driver::standby_sleep_transition() { return false; }
bool Driver::standby_refup_transition() { return false; }
bool Driver::standby_measure_transition() { return false; }
bool Driver::standby_extended_balancing_transition() { return false; }

bool Driver::refup_sleep_transition() { return false; }
bool Driver::refup_standby_transition() { return false; }
bool Driver::refup_measure_transition() { return false; }
bool Driver::refup_extended_balancing_transition() { return false; }

bool Driver::measure_refup_transition() { return false; }
bool Driver::measure_standby_transition() { return false; }

bool Driver::extended_balancing_sleep_transition() { return false; }
bool Driver::extended_balancing_standby_transition() { return false; }
bool Driver::extended_balancing_dtm_measure_transition() { return false; }

bool Driver::dtm_measure_standby_transition() { return false; }
bool Driver::dtm_measure_extended_balancing_transition() { return false; }

void Driver::init(const float &freq) {
    state_id sleep = core_sm.add_state(sleep_action);
    state_id standby = core_sm.add_state(standby_action);
    state_id refup = core_sm.add_state(refup_action);
    state_id measure = core_sm.add_state(measure_action);
    state_id extended_balancing = core_sm.add_state(extended_balancing_action);
    state_id dtm_measure = core_sm.add_state(dtm_measure_action);

    core_sm.add_transition(sleep, sleep_standby_transition, standby);
}