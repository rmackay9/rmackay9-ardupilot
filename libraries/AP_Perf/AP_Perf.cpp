#include <AP_Math/AP_Math.h>
#include <GCS_MAVLink/GCS.h>
#include <AP_HAL/AP_HAL.h>

#include "AP_Perf.h"

extern const AP_HAL::HAL& hal;

// static array implementations
uint64_t AP_Perf::_start_us[AP_PERF_MAX_RECORDERS];
uint64_t AP_Perf::_total_us[AP_PERF_MAX_RECORDERS];

AP_Perf::AP_Perf(uint8_t id) :
        _id(id)
{
    _start_us[id] = AP_HAL::micros64();
}

AP_Perf::~AP_Perf()
{
    if (_start_us[_id] > 0) {
        _total_us[_id] += AP_HAL::micros64() - _start_us[_id];
        _start_us[_id] = 0;
    }
}

// display latest timing data and reset timers
void AP_Perf::print_timing_data()
{
    // calculate total time
    uint64_t total_us = 0;
    for (uint8_t i=0; i<AP_PERF_MAX_RECORDERS; i++) {
        total_us += _total_us[i];
    }

    gcs().send_text(MAV_SEVERITY_CRITICAL,"AP_Perf timings, total:%ldms", (unsigned long)total_us/1000);

    // send total times and percentages for each counter
    for (uint8_t i=0; i<AP_PERF_MAX_RECORDERS; i++) {
        uint32_t total_ms = _total_us[i] / 1000;
        float percentage = (total_us > 0) ? ((float)total_ms/(total_us/1000) * 100.0f) : 0.0f;
        gcs().send_text(MAV_SEVERITY_CRITICAL,"%d: %ldms %4.1f%%",(int)i, (unsigned long)total_ms, (double)percentage);

        // clear timer
        _total_us[i] = 0;
    }
}
