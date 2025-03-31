#include "StateMachine.hpp"

#include <cassert>

void StateMachine::add_state(state_id id, std::function<void()> action) {
    actions[id] = action;
}

void StateMachine::add_transition(state_id old_state, std::function<bool()> predicate, state_id new_state) {
    assert(actions.count(old_state) && "Old state must exist before adding a transition");
    assert(actions.count(new_state) && "New state must exist before adding a transition");
    transitions[old_state].push_back(transition(predicate, new_state));
}

void StateMachine::update() {
    for(transition t : transitions[current_state]) {
        if(t.first()) {
            current_state = t.second;
            actions[current_state]();
            break;
        }
    }
}