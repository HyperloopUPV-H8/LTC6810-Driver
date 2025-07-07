#ifndef STATEMACHINE_HPP
#define STATEMACHINE_HPP

#include <array>
#include <concepts>
#include <cstddef>
#include <type_traits>
#include <utility>

using Callback = void (*)();
using Guard = bool (*)();

namespace LTC6810Driver {
template <class StateEnum>
struct Transition {
    StateEnum target;
    Guard predicate;
};

template <class StateEnum, typename... T>
concept are_transitions = (std::same_as<T, Transition<StateEnum>> && ...);

template <class StateEnum, size_t NTransitions>
class State {
    StateEnum state;
    Callback action;
    std::array<Transition<StateEnum>, NTransitions> transitions;

   public:
    template <typename... T>
        requires are_transitions<StateEnum, T...>
    consteval State(StateEnum state, Callback action, T... transitions)
        : state(state), action(action), transitions({transitions...}) {}

    consteval const StateEnum& get_state() const { return state; };
    consteval const Callback& get_action() const { return action; };
    consteval const auto& get_transitions() const { return transitions; };
};

template <typename T, class StateEnum>
struct is_state : std::false_type {};

template <class StateEnum, size_t N>
struct is_state<State<StateEnum, N>, StateEnum> : std::true_type {};

template <class StateEnum, typename... Ts>
concept are_states = (is_state<Ts, StateEnum>::value && ...);

template <class StateEnum, size_t NStates, size_t NTransitions>
struct StateMachine {
    using Actions = std::array<Callback, NStates>;
    using Transitions = std::array<Transition<StateEnum>, NTransitions>;
    using TAssocs = std::array<std::pair<size_t, size_t>, NStates>;

    StateEnum current_state;
    Actions actions;
    Transitions transitions;
    TAssocs transitions_assoc;

    consteval void process_state(auto state, size_t offset) {
        actions[static_cast<size_t>(state.get_state())] = state.get_action();

        for (const auto& t : state.get_transitions()) {
            transitions[offset++] = t;
        }

        transitions_assoc[static_cast<size_t>(state.get_state())] = {
            offset - state.get_transitions().size(),
            state.get_transitions().size()};
    }

    void execute() const { actions[static_cast<size_t>(current_state)](); }

   public:
    template <typename... S>
        requires are_states<StateEnum, S...>
    consteval StateMachine(StateEnum initial_state, S... states) {
        current_state = initial_state;
        size_t offset = 0;
        ((process_state(states, offset),
          offset += states.get_transitions().size()),
         ...);
    }

    void update() {
        auto& [i, n] = transitions_assoc[static_cast<size_t>(current_state)];
        for (auto index = i; index < i + n; ++index) {
            const auto& t = transitions[index];
            if (t.predicate()) {
                current_state = t.target;
                execute();
                break;
            }
        }
    };
};
}  // namespace LTC6810Driver

template <typename StateEnum, typename... Transitions>
    requires LTC6810Driver::are_transitions<StateEnum, Transitions...>
consteval auto make_state(StateEnum state, Callback action,
                          Transitions... transitions) {
    constexpr size_t NTransitions = sizeof...(transitions);
    return LTC6810Driver::State<StateEnum, NTransitions>(state, action,
                                                         transitions...);
}

template <typename StateEnum, typename... States>
    requires LTC6810Driver::are_states<StateEnum, States...>
consteval auto make_state_machine(StateEnum initial_state, States... states) {
    constexpr size_t NStates = sizeof...(states);
    constexpr size_t NTransitions = (states.get_transitions().size() + ... + 0);
    return LTC6810Driver::StateMachine<StateEnum, NStates, NTransitions>(
        initial_state, states...);
}

#endif