#ifndef STATEMACHINE_HPP
#define STATEMACHINE_HPP

#include <unordered_map>
#include <vector>

using Callback = void (*)();
using Guard = bool (*)();

template <class StateEnum>
struct Transition {
    StateEnum root;
    StateEnum target;
    Guard predicate;
};

template <class StateEnum>
struct State {
    StateEnum state;
    Callback action;
};

template <class StateEnum>
class StateMachine {
    using States = std::unordered_map<StateEnum, State<StateEnum>>;
    using Transitions =
        std::unordered_map<StateEnum, std::vector<Transition<StateEnum>>>;

    StateEnum current_state;
    States states;
    Transitions transitions;

   public:
    template <size_t NStates, size_t NTransitions>
    StateMachine(
        StateEnum initial_state,
        const std::array<State<StateEnum>, NStates>& states,
        const std::array<Transition<StateEnum>, NTransitions>& transitions)
        : current_state(initial_state) {
        for (const State<StateEnum>& s : states) {
            this->states[s.state] = s;
        }

        for (const Transition<StateEnum>& t : transitions) {
            this->transitions[t.root].push_back(t);
        }

        this->states[current_state].action();
    }

    void update() {
        for (const Transition<StateEnum>& t : transitions[current_state]) {
            if (t.predicate()) {
                current_state = t.target;
                states[current_state].action();
                break;
            }
        }
    };
};

#endif