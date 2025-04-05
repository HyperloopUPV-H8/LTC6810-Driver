#include "StateMachine.hpp"

#include <cassert>

state_id StateMachine::add_state(action action) {
    ++n_states;
    assert(n_states == 255 && "Maximum number of states is 254");
    states[n_states] = action;
    return n_states;
}

void StateMachine::add_transition(state_id old_state, std::function<bool()> predicate, state_id new_state) {
    assert(states.count(old_state) && "Old state must exist before adding a transition");
    assert(states.count(new_state) && "New state must exist before adding a transition");
    
    transitions[old_state].push_back(transition(predicate, new_state));
}

void StateMachine::init(state_id initial_state) {
    assert(states.count(initial_state) && "Initial state must exist");

    current_state = initial_state;
}

void StateMachine::update() {
    assert(current_state == 255 && "You must configure initial state using init()");

    for(const auto& [predicate, state] : transitions[current_state]) {
        if(predicate()) {
            current_state = state;
            states[state]();
            break;
        }
    }
}
