#ifndef STATEMACHINE_HPP
#define STATEMACHINE_HPP

#include <concepts>
#include <cstdint>
#include <utility>
#include <unordered_map>
#include <vector>
#include <functional>

using state_id = uint8_t;

using transition = std::pair<std::function<bool()>, state_id>;
using action = std::function<void()>;

class StateMachine {
private:
    state_id current_state;
    std::unordered_map<state_id, std::vector<transition>> transitions;
    std::unordered_map<state_id, action> actions;

public:

    void add_state(state_id id, std::function<void()> action);
    void add_transition(state_id old_state, std::function<bool()> predicate, state_id new_state);
    
    void update();

};

#endif