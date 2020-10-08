#pragma once

#include <AP_Math/AP_Math.h>

#define AP_OPTICALFLOW_CAL_NUM_SAMPLES  20 // number of samples required before calibration begins

class AP_OpticalFlow_Calibrator {
public:
    AP_OpticalFlow_Calibrator() {};

    // start or stop the calibration
    void start();
    void stop();

    // add new sample to the calibrator
    void new_sample(const Vector2f& flow_rate, const Vector2f& gyro_rate);

    // running is true if actively calculating scaling
    //bool running() const { return _cal_state == CalState::RUNNING; }

    // update the state machine and calculate scaling
    // returns true if new scaling values have been found
    bool update() WARN_IF_UNUSED;

    // get scaling values as multipliers
    bool get_scaling(Vector2f &scaling);

private:

    // calibration states
    enum class CalState {
        NOT_STARTED = 0,
        RUNNING,
        SUCCESS
    } _cal_state;

    // local variables
    uint32_t _start_time_ms;                            // time the calibration was started
    struct CalData {
        float samples[AP_OPTICALFLOW_CAL_NUM_SAMPLES];  // buffer of scaling values derived from flow and gyro rates
        uint8_t num_samples;                            // number of samples in _samples buffer
        uint8_t cal_success_count;                      // number of successful calibrations
        float best_scale;                               // best scaling value found so far
        float best_scale_fitness;                       // fitness (rms of error) of best scaling value
    } _cal_data[2];                                     // x and y axis

    // failure reporting
    enum class FailureReasons : uint8_t {
        AccelerationHigh = 0,
        YawMovementHigh = 1,
        RotationLow = 2,
        NoReason = 4                                    // must always be last reason
    };

    // record the reason for a sample being rejected
    void record_failure_reason(FailureReasons reason);

    // get most common reason samples are being rejected
    // returns NoReason if all counts are zero
    FailureReasons get_failure_reason();

    // clear failure counts
    void clear_failure_count();

    uint8_t _failure_reason_count[(uint8_t)FailureReasons::NoReason];   // count of reasons samples are being rejected
    uint16_t _success_count;                            // number of samples that were accepted since last reported
    uint32_t _last_report_ms;                           // system time of last status report
};
