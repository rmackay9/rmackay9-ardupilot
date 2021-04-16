#include "AC_PosControl_Sub.h"

AC_PosControl_Sub::AC_PosControl_Sub(AP_AHRS_View& ahrs, const AP_InertialNav& inav,
                                     const AP_Motors& motors, AC_AttitudeControl& attitude_control) :
    AC_PosControl(ahrs, inav, motors, attitude_control),
    _alt_max(0.0f),
    _alt_min(0.0f)
{}

/// input_vel_z calculate a jerk limited path from the current position, velocity and acceleration to an input velocity.
///     The function takes the current position, velocity, and acceleration and calculates the required jerk limited adjustment to the acceleration for the next time dt.
///     The kinematic path is constrained by :
///         maximum velocity - vel_max,
///         maximum acceleration - accel_max,
///         time constant - tc.
///     The time constant defines the acceleration error decay in the kinematic path as the system approaches constant acceleration.
///     The time constant also defines the time taken to achieve the maximum acceleration.
///     The time constant must be positive.
///     The function alters the input velocity to be the velocity that the system could reach zero acceleration in the minimum time.
//void AC_PosControl_Sub::input_vel_accel_z(const Vector3f& vel, const Vector3f& accel, bool force_descend)
//{
//    // check for ekf z position reset
//    check_for_ekf_z_reset();
//
//    // calculated increased maximum acceleration if over speed
//    float accel_z_cms = _accel_z_cms;
//    if (_vel_desired.z < _speed_down_cms && !is_zero(_speed_down_cms)) {
//        accel_z_cms *= POSCONTROL_OVERSPEED_GAIN_Z * _vel_desired.z / _speed_down_cms;
//    }
//    if (_vel_desired.z > _speed_up_cms && !is_zero(_speed_up_cms)) {
//        accel_z_cms *= POSCONTROL_OVERSPEED_GAIN_Z * _vel_desired.z / _speed_up_cms;
//    }
//    shape_vel_accel(vel.z, accel.z,
//        _vel_desired.z, _accel_desired.z,
//        _speed_down_cms, _speed_up_cms,
//        -constrain_float(accel_z_cms, 0.0f, 750.0f), accel_z_cms,
//        POSCONTROL_Z_SHAPER_TC, _dt);
//
//    // adjust desired alt if motors have not hit their limits
//    update_pos_vel_accel(_pos_target.z, _vel_desired.z, _accel_desired.z, _dt,
//        (_motors.limit.throttle_lower && !force_descend), _motors.limit.throttle_upper,
//        _p_pos_z.get_error(), _pid_vel_z.get_error());
//
//    // do not let target alt get above limit
//    if (_alt_max < 100 && _pos_target.z > _alt_max) {
//        _pos_target.z = _alt_max;
//        _limit.pos_up = true;
//        // decelerate feed forward to zero
//        _vel_desired.z = constrain_float(0.0f, _vel_desired.z-vel_change_limit, _vel_desired.z+vel_change_limit);
//    }
//
//    // do not let target alt get below limit
//    if (_alt_min < 0 && _alt_min < _alt_max && _pos_target.z < _alt_min) {
//        _pos_target.z = _alt_min;
//        _limit.pos_down = true;
//        // decelerate feed forward to zero
//        _vel_desired.z = constrain_float(0.0f, _vel_desired.z-vel_change_limit, _vel_desired.z+vel_change_limit);
//    }
//
//    // run horizontal position controller
//    run_z_controller();
//}

/// relax_alt_hold_controllers - set all desired and targets to measured
void AC_PosControl_Sub::relax_alt_hold_controllers()
{
    _pos_target.z = _inav.get_altitude();
    _vel_desired.z = 0.0f;
    _vel_target.z = _inav.get_velocity_z();
    _accel_desired.z = 0.0f;
    _accel_target.z = -(_ahrs.get_accel_ef_blended().z + GRAVITY_MSS) * 100.0f;
    _pid_accel_z.reset_filter();
}
