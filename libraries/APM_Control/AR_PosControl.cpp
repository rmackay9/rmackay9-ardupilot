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

#include <AP_Math/AP_Math.h>
#include <AP_HAL/AP_HAL.h>
#include <GCS_MAVLink/GCS.h>
#include "AR_PosControl.h"

extern const AP_HAL::HAL& hal;

#define AR_POSCON_POS_ERR_MAX           2.0f    // maximum position error in meters
#define AR_POSCON_TIMEOUT_MS            100     // timeout after 0.1 sec
#define AR_POSCON_POS_P                 0.2f    // default position P gain
#define AR_POSCON_VEL_P                 0.2f    // default velocity P gain
#define AR_POSCON_VEL_I                 0.2f    // default velocity I gain
#define AR_POSCON_VEL_D                 0.0f    // default velocity D gain
#define AR_POSCON_VEL_FF                0.1f    // default velocity FF gain
#define AR_POSCON_VEL_IMAX              1.0f    // default velocity IMAX
#define AR_POSCON_VEL_FILT              5.0f    // default velocity filter
#define AR_POSCON_VEL_FILT_D            5.0f    // default velocity D term filter
#define AR_POSCON_DT                    0.02f   // default dt for PID controllers

const AP_Param::GroupInfo AR_PosControl::var_info[] = {

    // @Param: _POS_P
    // @DisplayName: Position controller P gain
    // @Description: Position controller P gain.  Converts the distance to the target location into a desired speed which is then passed to the loiter latitude rate controller
    // @Range: 0.500 2.000
    // @User: Standard
    AP_SUBGROUPINFO(_p_pos, "_POS_", 1, AR_PosControl, AC_P_2D),

    // @Param: _VEL_P
    // @DisplayName: Velocity (horizontal) P gain
    // @Description: Velocity (horizontal) P gain.  Converts the difference between desired and actual velocity to a target acceleration
    // @Range: 0.1 6.0
    // @Increment: 0.1
    // @User: Advanced

    // @Param: _VEL_I
    // @DisplayName: Velocity (horizontal) I gain
    // @Description: Velocity (horizontal) I gain.  Corrects long-term difference between desired and actual velocity to a target acceleration
    // @Range: 0.02 1.00
    // @Increment: 0.01
    // @User: Advanced

    // @Param: _VEL_D
    // @DisplayName: Velocity (horizontal) D gain
    // @Description: Velocity (horizontal) D gain.  Corrects short-term changes in velocity
    // @Range: 0.00 1.00
    // @Increment: 0.001
    // @User: Advanced

    // @Param: _VEL_IMAX
    // @DisplayName: Velocity (horizontal) integrator maximum
    // @Description: Velocity (horizontal) integrator maximum.  Constrains the target acceleration that the I gain will output
    // @Range: 0 4500
    // @Increment: 10
    // @Units: cm/s/s
    // @User: Advanced

    // @Param: _VEL_FILT
    // @DisplayName: Velocity (horizontal) input filter
    // @Description: Velocity (horizontal) input filter.  This filter (in Hz) is applied to the input for P and I terms
    // @Range: 0 100
    // @Units: Hz
    // @User: Advanced

    // @Param: _VEL_D_FILT
    // @DisplayName: Velocity (horizontal) input filter
    // @Description: Velocity (horizontal) input filter.  This filter (in Hz) is applied to the input for D term
    // @Range: 0 100
    // @Units: Hz
    // @User: Advanced

    // @Param: _VEL_FF
    // @DisplayName: Velocity (horizontal) feed forward gain
    // @Description: Velocity (horizontal) feed forward gain.  Converts the difference between desired velocity to a target acceleration
    // @Range: 0 6
    // @Increment: 0.01
    // @User: Advanced
    AP_SUBGROUPINFO(_pid_vel, "_VEL_", 2, AR_PosControl, AC_PID_2D),

    AP_GROUPEND
};

AR_PosControl::AR_PosControl(AR_AttitudeControl& atc) :
    _atc(atc),
    _p_pos(AR_POSCON_POS_P, AR_POSCON_DT),
    _pid_vel(AR_POSCON_VEL_P, AR_POSCON_VEL_I, AR_POSCON_VEL_D, AR_POSCON_VEL_FF, AR_POSCON_VEL_IMAX, AR_POSCON_VEL_FILT, AR_POSCON_VEL_FILT_D, AR_POSCON_DT)
{
    AP_Param::setup_object_defaults(this, var_info);
}

// update navigation
void AR_PosControl::update(float dt)
{
    static uint32_t last_print_ms = 0;
    uint32_t now_ms = AP_HAL::millis();
    bool print_now = false;
    if (now_ms - last_print_ms > 1000) {
        print_now = true;
        last_print_ms = now_ms;
    }

    // exit immediately if no current location, destination or disarmed
    Vector2f curr_pos_NE;
    Vector3f curr_vel_NED;
    if (!hal.util->get_soft_armed() || !AP::ahrs().get_relative_position_NE_origin(curr_pos_NE) ||
        !AP::ahrs().get_velocity_NED(curr_vel_NED)) {
        _desired_speed = _atc.get_desired_speed_accel_limited(0.0f, dt);
        //_desired_lat_accel = 0.0f;
        _desired_turn_rate_rads = 0.0f;
        if (print_now) {
            gcs().send_text(MAV_SEVERITY_CRITICAL, "exit early");
        }
        return;
    }

    // convert earth-frame velocity to body frame in order to get forward-back speed
    const Vector2f curr_vel_NE(curr_vel_NED.x, curr_vel_NED.y);
    const Vector2f curr_vel_FR = AP::ahrs().earth_to_body2D(curr_vel_NE);

    // if no recent calls initialise desired_speed_limited to current speed
    // To-Do: init lateral accel, speed and turn rate from current values
    if (!is_active()) {
        _desired_speed = curr_vel_FR.x;
        _desired_lat_accel = 0.0f;
        _desired_turn_rate_rads = 0.0f;
    }
    _last_update_ms = AP_HAL::millis();

    // calculate position error and convert to desired velocity
    Vector2f des_vel;
    if (_pos_target_valid) {
        des_vel = _p_pos.update_all(_pos_target.x, _pos_target.y, curr_pos_NE, AR_POSCON_POS_ERR_MAX, _speed_max);
    }

    // calculation velocity error
    if (_vel_target_valid) {
        // add target velocity to desired velocity from position error
        des_vel += _vel_target;
    }

    // calculate desired acceleration
    // To-Do: set limits flag based on whether length is beyond max speed and/or motor outputs?
    Vector2f des_accel_ne = _pid_vel.update_all(des_vel, curr_vel_NE, false);

    // convert desired acceleration to desired forward-back speed, desired lateral speed and desired turn rate

    // rotate acceleration into body frame using current heading
    const Vector2f des_accel_FR = AP::ahrs().earth_to_body2D(des_accel_ne);
    _desired_lat_accel = des_accel_FR.y;

    // calculate turn rate from desired lateral acceleration
    _desired_turn_rate_rads = _atc.get_turn_rate_from_lat_accel(_desired_lat_accel, curr_vel_FR.x);

    // if y-component is beyond lateral limit we should slow down?  If not we are free to speed up?
    // x component should turn into desired speed?
    // we will maintain a target body-frame speed, lat-speed and turn rate?

    // updated the desired speed using forward-back acceleration
    _desired_speed = _atc.get_desired_speed_accel_limited(_atc.get_desired_speed()+des_accel_FR.x, dt);

    // debug
    if (print_now) {
        gcs().send_text(MAV_SEVERITY_CRITICAL, "spd:%4.2f lat:%4.2f tr:%4.2f", (double)_desired_speed, (double)_desired_lat_accel, (double)_desired_turn_rate_rads);
    }
}

// set limits
void AR_PosControl::set_limits(float accel_max, float speed_max, float lat_accel_max,
                               bool has_lateral)
{
    _accel_max = accel_max;
    _speed_max = speed_max;
    _lat_accel_max = lat_accel_max;
    _has_lateral = has_lateral;
}

// set desired location
bool AR_PosControl::set_desired_location(const Location &destination)
{
   // convert location to offset from EKF origin
    Vector2f pos_NE;
    if (!destination.get_vector_xy_from_origin_NE(pos_NE)) {
        return false;
    }

    // set position target
    set_pos_target(pos_NE * 0.01f);
    return true;
}

// set position target
void AR_PosControl::set_pos_target(const Vector2f &pos)
{
    _pos_target = pos;
    _pos_target_valid = true;
    _vel_target_valid = false;
}

// set position and velocity targets
void AR_PosControl::set_pos_vel_target(const Vector2f &pos, const Vector2f &vel)
{
    _pos_target = pos;
    _vel_target = vel;
    _pos_target_valid = true;
    _vel_target_valid = true;
}

// true if update has been called recently
bool AR_PosControl::is_active() const
{
    return ((AP_HAL::millis() - _last_update_ms) < AR_POSCON_TIMEOUT_MS);
}
