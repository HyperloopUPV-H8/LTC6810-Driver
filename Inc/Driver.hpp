#ifndef DRIVER_HPP
#define DRIVER_HPP

#include "StateMachine.hpp"

class Driver {
   private:
    static StateMachine core_sm;
    static StateMachine isospi_sm;

    // enum CORE_STATE {
    //     SLEEP,
    //     STANDBY,
    //     REFUP,
    //     MEASURE,
    //     EXTENDED_BALANCING,
    //     DTM_MEASURE
    // };

    // enum ISOSPI_STATE { IDLE, READY, ACTIVE };

    // Actions
    static void sleep_action();
    static void standby_action();
    static void refup_action();
    static void measure_action();
    static void extended_balancing_action();
    static void dtm_measure_action();

    // Transitions
    static bool sleep_standby_transition();

    static bool standby_sleep_transition();
    static bool standby_refup_transition();
    static bool standby_measure_transition();
    static bool standby_extended_balancing_transition();

    static bool refup_sleep_transition();
    static bool refup_standby_transition();
    static bool refup_measure_transition();
    static bool refup_extended_balancing_transition();

    static bool measure_refup_transition();
    static bool measure_standby_transition();

    static bool extended_balancing_sleep_transition();
    static bool extended_balancing_standby_transition();
    static bool extended_balancing_dtm_measure_transition();

    static bool dtm_measure_standby_transition();
    static bool dtm_measure_extended_balancing_transition();

   public:
    static void init(const float &freq);
};

#endif