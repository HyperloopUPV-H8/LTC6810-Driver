#ifndef STATEMACHINE_HPP
#define STATEMACHINE_HPP

using Callback = void (*)();
using Guard = bool (*)();

template <class StateEnum>
struct Transition {
    StateEnum target;
    Guard predicate;
};

template <class StateEnum, typename... T>
concept are_transitions = (std::same_as<T, Transition<StateEnum>> && ...);

template <class StateEnum, size_t NTransitions>
struct State {
   private:
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

template <typename StateEnum, typename... Transitions>
    requires are_transitions<StateEnum, Transitions...>
consteval auto make_state(StateEnum state, Callback action,
                          Transitions... transitions) {
    constexpr size_t NTransitions = sizeof...(transitions);
    return State<StateEnum, NTransitions>(state, action, transitions...);
}

template <typename T, class StateEnum>
struct is_state : std::false_type {};

template <class StateEnum, size_t N>
struct is_state<State<StateEnum, N>, StateEnum> : std::true_type {};

template <class StateEnum, typename... Ts>
concept are_states = (is_state<Ts, StateEnum>::value && ...);

template <class StateEnum, size_t NStates, size_t NTransitions>
class StateMachineBuilder {
    using Actions = std::array<Callback, NStates>;
    using Transitions = std::array<Transition<StateEnum>, NTransitions>;
    using TAssocs = std::array<std::pair<size_t, size_t>, NStates>;

    class StateMachine {
       public:
        StateEnum current_state;
        const Actions& actions;
        const Transitions& transitions;
        const TAssocs& transitions_assoc;

        void execute() const { actions[static_cast<size_t>(current_state)](); }

       public:
        StateMachine(StateEnum& initial_state, const Actions& actions,
                     const Transitions& transitions, const TAssocs& assocs)
            : current_state(initial_state),
              actions(actions),
              transitions(transitions),
              transitions_assoc(assocs) {
            execute();
        }

        void update() {
            auto& [i, n] =
                transitions_assoc[static_cast<size_t>(current_state)];
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

   public:
    template <typename... S>
        requires are_states<StateEnum, S...>
    consteval StateMachineBuilder(S... states) {
        size_t offset = 0;
        ((process_state(states, offset),
          offset += states.get_transitions().size()),
         ...);
    }

    auto init(StateEnum initial_state) const {
        return StateMachine(initial_state, actions, transitions,
                            transitions_assoc);
    }
};

template <typename StateEnum, typename... States>
    requires are_states<StateEnum, States...>
consteval auto make_state_machine(States... states) {
    constexpr size_t NStates = sizeof...(states);
    constexpr size_t NTransitions = (states.get_transitions().size() + ...);
    return StateMachineBuilder<StateEnum, NStates, NTransitions>(states...);
}

#endif