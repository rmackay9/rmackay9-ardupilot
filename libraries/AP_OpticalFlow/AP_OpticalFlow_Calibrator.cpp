/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#include "AP_OpticalFlow.h"
#include "AP_OpticalFlow_Calibrator.h"
#include <AP_HAL/AP_HAL.h>
#include <AP_AHRS/AP_AHRS.h>
#include <GCS_MAVLink/GCS.h>
#include <AP_Notify/AP_Notify.h>

const uint32_t AP_OPTICALFLOW_CAL_TIMEOUT_MS = 60000;       // calibration timesout after 60 seconds
const uint32_t AP_OPTICALFLOW_CAL_STATUSINTERVAL_MS = 3000; // status updates printed at 3second intervals
const float AP_OPTICALFLOW_CAL_ACCEL_MAX_MSS = 1;           // maximum acceptable acceleration (must be low to ensure good scaling)
const float AP_OPTICALFLOW_CAL_YAW_MAX_RADS = radians(20);  // maximum yaw rotation (must be low to ensure good scaling)
const float AP_OPTICALFLOW_CAL_ROLLPITCH_MIN_RADS = radians(20);    // minimum acceptable roll or pitch rotation
const float AP_OPTICALFLOW_CAL_SCALE_MIN = 0.80f;           // min acceptable scaling value.  If resulting scaling is below this then the calibration fails
const float AP_OPTICALFLOW_CAL_SCALE_MAX = 1.2f;            // max acceptable scaling value.  If resulting scaling is above this then the calibration fails
const float AP_OPTICALFLOW_CAL_TOLERANCE = 0.5f;            // worst acceptable RMS tolerance
const uint8_t AP_OPTICALFLOW_CAL_SUCCES_COUNT = 3;          // require 3 successful scaling values per axis

extern const AP_HAL::HAL& hal;

// public methods

// start the calibration
void AP_OpticalFlow_Calibrator::start()
{
    // exit immediately if already running
    if (_cal_state == CalState::RUNNING) {
        return;
    }

    _cal_state = CalState::RUNNING;
    _start_time_ms = AP_HAL::millis();

    // clear samples buffers
    _cal_data[0].num_samples = 0;
    _cal_data[1].num_samples = 0;
    _cal_data[0].cal_success_count = 0;
    _cal_data[1].cal_success_count = 0;

    gcs().send_text(MAV_SEVERITY_INFO, "Flow Cal Started");
    AP_Notify::flags.compass_cal_running = true;
}

void AP_OpticalFlow_Calibrator::stop()
{
    // exit immediately if already stopped
    if (_cal_state == CalState::NOT_STARTED) {
        return;
    }

    _cal_state = CalState::NOT_STARTED;

    gcs().send_text(MAV_SEVERITY_INFO, "Flow Cal Stopped");
    AP_Notify::flags.compass_cal_running = false;
}

// add new sample to the calibrator
void AP_OpticalFlow_Calibrator::new_sample(const Vector2f& flow_rate, const Vector2f& gyro_rate)
{
    // return immediately if not running
    if (_cal_state != CalState::RUNNING) {
        return;
    }

    // check vehicle movement is low
    Vector3f accel_ef = AP::ahrs().get_accel_ef_blended();
    accel_ef.z += GRAVITY_MSS;
    if (accel_ef.length() > AP_OPTICALFLOW_CAL_ACCEL_MAX_MSS) {
        record_failure_reason(FailureReasons::AccelerationHigh);
        return;
    }

    // check yaw rotation is low
    const Vector3f gyro = AP::ahrs().get_gyro();
    if (fabsf(gyro.z) > AP_OPTICALFLOW_CAL_YAW_MAX_RADS) {
        record_failure_reason(FailureReasons::YawMovementHigh);
        return;
    }

    // check enough roll or pitch movement
    if ((fabsf(gyro_rate.x) < AP_OPTICALFLOW_CAL_ROLLPITCH_MIN_RADS) && (fabsf(gyro_rate.y) < AP_OPTICALFLOW_CAL_ROLLPITCH_MIN_RADS)) {
        record_failure_reason(FailureReasons::RotationLow);
        return;
    }

    // check minimum quality?

    // calculate x scaling
    bool good_sample = false;
    for (uint8_t i = 0; i < ARRAY_SIZE(_cal_data); i++) {
        const float gyro_rate_xy = (i == 0) ? gyro_rate.x : gyro_rate.y;
        const float flow_rate_xy = (i == 0) ? flow_rate.x : flow_rate.y;
        if ((_cal_data[i].num_samples < ARRAY_SIZE(_cal_data[i].samples)) && (fabsf(gyro_rate_xy) >= AP_OPTICALFLOW_CAL_ROLLPITCH_MIN_RADS) && !is_zero(flow_rate_xy)) {
            _cal_data[i].samples[_cal_data[i].num_samples] = gyro_rate_xy / flow_rate_xy;
            _cal_data[i].num_samples++;
            good_sample = true;
        }
    }
    if (good_sample) {
        _success_count++;
    }
}

// update the state machine and calculate scaling
bool AP_OpticalFlow_Calibrator::update()
{
    // return immediately if not running
    if (_cal_state != CalState::RUNNING) {
        return false;
    }

    // check x and y axis
    for (uint8_t i = 0; i < ARRAY_SIZE(_cal_data); i++) {

        // if sample buffer is full calculate scaling
        if (_cal_data[i].num_samples >= ARRAY_SIZE(_cal_data[i].samples)) {

            // calculate average scaling
            float scale_average = 0.0f;
            for (uint16_t j = 0; j < _cal_data[i].num_samples; j++) {
                scale_average += _cal_data[i].samples[j];
            }
            scale_average /= _cal_data[i].num_samples;

            // calculate variation in scaling values
            float diff_total = 0.0f;
            for (uint16_t j = 0; j < _cal_data[i].num_samples; j++) {
                diff_total += sq(_cal_data[i].samples[j] - scale_average);
            }
            const float fitness = safe_sqrt(diff_total);

            // check fitness and record scaling if better than any previous calibrations
            if ((fitness <= AP_OPTICALFLOW_CAL_TOLERANCE) &&
                (scale_average >= AP_OPTICALFLOW_CAL_SCALE_MIN) &&
                (scale_average <= AP_OPTICALFLOW_CAL_SCALE_MAX)) {
                if ((_cal_data[i].cal_success_count == 0) || (fitness < _cal_data[i].best_scale_fitness)) {
                    _cal_data[i].best_scale_fitness = fitness;
                    _cal_data[i].best_scale = scale_average;
                    _cal_data[i].cal_success_count++;
                }
            }

            // clear sample buffer
            _cal_data[i].num_samples = 0;
        }
    }

    // success if at least three successful scaling values found for both axis
    const char* prefix_str = "FlowCal:";
    if ((_cal_data[0].cal_success_count >= AP_OPTICALFLOW_CAL_SUCCES_COUNT) && (_cal_data[1].cal_success_count >= AP_OPTICALFLOW_CAL_SUCCES_COUNT)) {
        gcs().send_text(MAV_SEVERITY_INFO, "%s Success X:%4.2f fit:%4.2f Y:%4.2f fit:%4.2f",
                prefix_str,
                (double)_cal_data[0].best_scale,
                (double)_cal_data[0].best_scale_fitness,
                (double)_cal_data[1].best_scale,
                (double)_cal_data[1].best_scale_fitness);
        stop();
        _cal_state = CalState::SUCCESS;
        return true;
    }

    // check for timeout
    uint32_t now_ms = AP_HAL::millis();
    if (now_ms - _start_time_ms > AP_OPTICALFLOW_CAL_TIMEOUT_MS) {
        gcs().send_text(MAV_SEVERITY_INFO, "%s Timeout", prefix_str);
        stop();
        return false;
    }

    // status reporting
    if (now_ms - _last_report_ms > AP_OPTICALFLOW_CAL_STATUSINTERVAL_MS) {
        _last_report_ms = now_ms;

        // display percentage complete
        gcs().send_text(MAV_SEVERITY_INFO, "%s X:%d%% Y:%d%%",
                prefix_str,
                (int)MIN(100.0f, (100.0f * _cal_data[0].cal_success_count / AP_OPTICALFLOW_CAL_SUCCES_COUNT)),
                (int)MIN(100.0f, (100.0f * _cal_data[1].cal_success_count / AP_OPTICALFLOW_CAL_SUCCES_COUNT)));

        // print most common failure reason
        switch (get_failure_reason()) {
        case FailureReasons::AccelerationHigh:
            gcs().send_text(MAV_SEVERITY_INFO, "%s High acceleration", prefix_str);
            break;
        case FailureReasons::YawMovementHigh:
            gcs().send_text(MAV_SEVERITY_INFO, "%s Yaw movement", prefix_str);
            break;
        case FailureReasons::RotationLow:
            gcs().send_text(MAV_SEVERITY_INFO, "%s Rock vehicle", prefix_str);
            break;
        case FailureReasons::NoReason:
            // no issues
            break;
        }
        clear_failure_count();
    }

    return false;
}

// get scaling values as multipliers
bool AP_OpticalFlow_Calibrator::get_scaling(Vector2f &scaling)
{
    // return immediately if not running
    if (_cal_state != CalState::SUCCESS) {
        return false;
    }

    // return best scaling values
    scaling.x = _cal_data[0].best_scale;
    scaling.y = _cal_data[1].best_scale;
    return true;
}

void AP_OpticalFlow_Calibrator::record_failure_reason(FailureReasons reason)
{
    // sanity check reason
    if (reason == FailureReasons::NoReason) {
        return;
    }

    // increment failure reason counter
    uint8_t index = (uint8_t)reason;
    if (_failure_reason_count[index] < UINT8_MAX) {
        _failure_reason_count[index]++;
    }
}

// get most common reason samples are being rejected
AP_OpticalFlow_Calibrator::FailureReasons AP_OpticalFlow_Calibrator::get_failure_reason()
{
    FailureReasons main_reason = FailureReasons::NoReason;
    uint8_t main_reason_count = 0;
    uint16_t failure_count = 0;
    for (uint8_t i = 0; i < ARRAY_SIZE(_failure_reason_count); i++) {
        failure_count += _failure_reason_count[i];
        if (_failure_reason_count[i] > main_reason_count) {
            main_reason_count = _failure_reason_count[i];
            main_reason = (FailureReasons)i;
        }
    }

    // if failures exceed successes then return main reason
    if (failure_count >= _success_count) {
        return main_reason;
    } else {
        return FailureReasons::NoReason;
    }
}

// clear failure counts
void AP_OpticalFlow_Calibrator::clear_failure_count()
{
    memset(&_failure_reason_count, 0, sizeof(_failure_reason_count));
}
