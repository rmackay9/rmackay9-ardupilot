/// @file	AP_Perf.h
/// @brief	Landing gear control library
#pragma once

#include <AP_Common/AP_Common.h>

#define AP_PERF_MAX_RECORDERS 10

/// @class	AP_Perf
/// @brief	Class record performance info
class AP_Perf {
public:

    // constructor that start timer
    AP_Perf(uint8_t id);

    // destructor that stops timer
    ~AP_Perf();

    /* Do not allow copies */
    AP_Perf(const AP_Perf &other) = delete;
    AP_Perf &operator=(const AP_Perf&) = delete;

    // display latest timing data and reset timers
    static void print_timing_data();

private:

    // internal variables for this instance
    uint8_t _id;

    // totals for all instances
    static uint64_t _start_us[AP_PERF_MAX_RECORDERS];  // last time time recorder was started
    static uint64_t _total_us[AP_PERF_MAX_RECORDERS];  // total time for each id
};
