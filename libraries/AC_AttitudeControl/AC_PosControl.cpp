#include <AP_HAL/AP_HAL.h>
#include "AC_PosControl.h"
#include <AP_Math/AP_Math.h>
#include <AP_Logger/AP_Logger.h>

extern const AP_HAL::HAL& hal;

#if APM_BUILD_TYPE(APM_BUILD_ArduPlane)
 // default gains for Plane
 # define POSCONTROL_POS_Z_P                    1.0f    // vertical position controller P gain default
 # define POSCONTROL_VEL_Z_P                    5.0f    // vertical velocity controller P gain default
 # define POSCONTROL_VEL_Z_IMAX                 1000.0f // vertical velocity controller IMAX gain default
 # define POSCONTROL_VEL_Z_FILT_HZ              5.0f    // vertical velocity controller input filter
 # define POSCONTROL_VEL_Z_FILT_D_HZ            5.0f    // vertical velocity controller input filter for D
 # define POSCONTROL_ACC_Z_P                    0.3f    // vertical acceleration controller P gain default
 # define POSCONTROL_ACC_Z_I                    1.0f    // vertical acceleration controller I gain default
 # define POSCONTROL_ACC_Z_D                    0.0f    // vertical acceleration controller D gain default
 # define POSCONTROL_ACC_Z_IMAX                 800     // vertical acceleration controller IMAX gain default
 # define POSCONTROL_ACC_Z_FILT_HZ              10.0f   // vertical acceleration controller input filter default
 # define POSCONTROL_ACC_Z_DT                   0.02f   // vertical acceleration controller dt default
 # define POSCONTROL_POS_XY_P                   1.0f    // horizontal position controller P gain default
 # define POSCONTROL_VEL_XY_P                   1.4f    // horizontal velocity controller P gain default
 # define POSCONTROL_VEL_XY_I                   0.7f    // horizontal velocity controller I gain default
 # define POSCONTROL_VEL_XY_D                   0.35f   // horizontal velocity controller D gain default
 # define POSCONTROL_VEL_XY_IMAX                1000.0f // horizontal velocity controller IMAX gain default
 # define POSCONTROL_VEL_XY_FILT_HZ             5.0f    // horizontal velocity controller input filter
 # define POSCONTROL_VEL_XY_FILT_D_HZ           5.0f    // horizontal velocity controller input filter for D
#elif APM_BUILD_TYPE(APM_BUILD_ArduSub)
 // default gains for Sub
 # define POSCONTROL_POS_Z_P                    3.0f    // vertical position controller P gain default
 # define POSCONTROL_VEL_Z_P                    8.0f    // vertical velocity controller P gain default
 # define POSCONTROL_VEL_Z_IMAX                 1000.0f // vertical velocity controller IMAX gain default
 # define POSCONTROL_VEL_Z_FILT_HZ              5.0f    // vertical velocity controller input filter
 # define POSCONTROL_VEL_Z_FILT_D_HZ            5.0f    // vertical velocity controller input filter for D
 # define POSCONTROL_ACC_Z_P                    0.5f    // vertical acceleration controller P gain default
 # define POSCONTROL_ACC_Z_I                    0.1f    // vertical acceleration controller I gain default
 # define POSCONTROL_ACC_Z_D                    0.0f    // vertical acceleration controller D gain default
 # define POSCONTROL_ACC_Z_IMAX                 100     // vertical acceleration controller IMAX gain default
 # define POSCONTROL_ACC_Z_FILT_HZ              20.0f   // vertical acceleration controller input filter default
 # define POSCONTROL_ACC_Z_DT                   0.0025f // vertical acceleration controller dt default
 # define POSCONTROL_POS_XY_P                   1.0f    // horizontal position controller P gain default
 # define POSCONTROL_VEL_XY_P                   1.0f    // horizontal velocity controller P gain default
 # define POSCONTROL_VEL_XY_I                   0.5f    // horizontal velocity controller I gain default
 # define POSCONTROL_VEL_XY_D                   0.0f    // horizontal velocity controller D gain default
 # define POSCONTROL_VEL_XY_IMAX                1000.0f // horizontal velocity controller IMAX gain default
 # define POSCONTROL_VEL_XY_FILT_HZ             5.0f    // horizontal velocity controller input filter
 # define POSCONTROL_VEL_XY_FILT_D_HZ           5.0f    // horizontal velocity controller input filter for D
#else
 // default gains for Copter / TradHeli
 # define POSCONTROL_POS_Z_P                    1.0f    // vertical position controller P gain default
 # define POSCONTROL_VEL_Z_P                    5.0f    // vertical velocity controller P gain default
 # define POSCONTROL_VEL_Z_IMAX                 1000.0f // vertical velocity controller IMAX gain default
 # define POSCONTROL_VEL_Z_FILT_HZ              5.0f    // vertical velocity controller input filter
 # define POSCONTROL_VEL_Z_FILT_D_HZ            5.0f    // vertical velocity controller input filter for D
 # define POSCONTROL_ACC_Z_P                    0.5f    // vertical acceleration controller P gain default
 # define POSCONTROL_ACC_Z_I                    1.0f    // vertical acceleration controller I gain default
 # define POSCONTROL_ACC_Z_D                    0.0f    // vertical acceleration controller D gain default
 # define POSCONTROL_ACC_Z_IMAX                 800     // vertical acceleration controller IMAX gain default
 # define POSCONTROL_ACC_Z_FILT_HZ              20.0f   // vertical acceleration controller input filter default
 # define POSCONTROL_ACC_Z_DT                   0.0025f // vertical acceleration controller dt default
 # define POSCONTROL_POS_XY_P                   1.0f    // horizontal position controller P gain default
 # define POSCONTROL_VEL_XY_P                   2.0f    // horizontal velocity controller P gain default
 # define POSCONTROL_VEL_XY_I                   1.0f    // horizontal velocity controller I gain default
 # define POSCONTROL_VEL_XY_D                   0.5f    // horizontal velocity controller D gain default
 # define POSCONTROL_VEL_XY_IMAX                1000.0f // horizontal velocity controller IMAX gain default
 # define POSCONTROL_VEL_XY_FILT_HZ             5.0f    // horizontal velocity controller input filter
 # define POSCONTROL_VEL_XY_FILT_D_HZ           5.0f    // horizontal velocity controller input filter for D
#endif

// vibration compensation gains
#define POSCONTROL_VIBE_COMP_P_GAIN 0.250f
#define POSCONTROL_VIBE_COMP_I_GAIN 0.125f

const AP_Param::GroupInfo AC_PosControl::var_info[] = {
    // 0 was used for HOVER

    // @Param: _ACC_XY_FILT
    // @DisplayName: XY Acceleration filter cutoff frequency
    // @Description: Lower values will slow the response of the navigation controller and reduce twitchiness
    // @Units: Hz
    // @Range: 0.5 5
    // @Increment: 0.1
    // @User: Advanced
    AP_GROUPINFO("_ACC_XY_FILT", 1, AC_PosControl, _accel_xy_filt_hz, POSCONTROL_ACCEL_FILTER_HZ),

    // @Param: _POSZ_P
    // @DisplayName: Position (vertical) controller P gain
    // @Description: Position (vertical) controller P gain.  Converts the difference between the desired altitude and actual altitude into a climb or descent rate which is passed to the throttle rate controller
    // @Range: 1.000 3.000
    // @User: Standard
    AP_SUBGROUPINFO(_p_pos_z, "_POSZ_", 2, AC_PosControl, AC_P_1D),

    // @Param: _VELZ_P
    // @DisplayName: Velocity (vertical) controller P gain
    // @Description: Velocity (vertical) controller P gain.  Converts the difference between desired vertical speed and actual speed into a desired acceleration that is passed to the throttle acceleration controller
    // @Range: 1.000 8.000
    // @User: Standard

    // @Param: _VELZ_I
    // @DisplayName: Velocity (vertical) controller I gain
    // @Description: Velocity (vertical) controller I gain.  Corrects long-term difference in desired velocity to a target acceleration
    // @Range: 0.02 1.00
    // @Increment: 0.01
    // @User: Advanced

    // @Param: _VELZ_IMAX
    // @DisplayName: Velocity (vertical) controller I gain maximum
    // @Description: Velocity (vertical) controller I gain maximum.  Constrains the target acceleration that the I gain will output
    // @Range: 1.000 8.000
    // @User: Standard

    // @Param: _VELZ_D
    // @DisplayName: Velocity (vertical) controller D gain
    // @Description: Velocity (vertical) controller D gain.  Corrects short-term changes in velocity
    // @Range: 0.00 1.00
    // @Increment: 0.001
    // @User: Advanced

    // @Param: _VELZ_FF
    // @DisplayName: Velocity (vertical) controller Feed Forward gain
    // @Description: Velocity (vertical) controller Feed Forward gain.  Produces an output that is proportional to the magnitude of the target
    // @Range: 0 1
    // @Increment: 0.01
    // @User: Advanced

    // @Param: _VELZ_FLTE
    // @DisplayName: Velocity (vertical) error filter
    // @Description: Velocity (vertical) error filter.  This filter (in Hz) is applied to the input for P and I terms
    // @Range: 0 100
    // @Units: Hz
    // @User: Advanced

    // @Param: _VELZ_FLTD
    // @DisplayName: Velocity (vertical) input filter for D term
    // @Description: Velocity (vertical) input filter for D term.  This filter (in Hz) is applied to the input for D terms
    // @Range: 0 100
    // @Units: Hz
    // @User: Advanced
    AP_SUBGROUPINFO(_pid_vel_z, "_VELZ_", 3, AC_PosControl, AC_PID_Basic),

    // @Param: _ACCZ_P
    // @DisplayName: Acceleration (vertical) controller P gain
    // @Description: Acceleration (vertical) controller P gain.  Converts the difference between desired vertical acceleration and actual acceleration into a motor output
    // @Range: 0.500 1.500
    // @Increment: 0.05
    // @User: Standard

    // @Param: _ACCZ_I
    // @DisplayName: Acceleration (vertical) controller I gain
    // @Description: Acceleration (vertical) controller I gain.  Corrects long-term difference in desired vertical acceleration and actual acceleration
    // @Range: 0.000 3.000
    // @User: Standard

    // @Param: _ACCZ_IMAX
    // @DisplayName: Acceleration (vertical) controller I gain maximum
    // @Description: Acceleration (vertical) controller I gain maximum.  Constrains the maximum pwm that the I term will generate
    // @Range: 0 1000
    // @Units: d%
    // @User: Standard

    // @Param: _ACCZ_D
    // @DisplayName: Acceleration (vertical) controller D gain
    // @Description: Acceleration (vertical) controller D gain.  Compensates for short-term change in desired vertical acceleration vs actual acceleration
    // @Range: 0.000 0.400
    // @User: Standard

    // @Param: _ACCZ_FF
    // @DisplayName: Acceleration (vertical) controller feed forward
    // @Description: Acceleration (vertical) controller feed forward
    // @Range: 0 0.5
    // @Increment: 0.001
    // @User: Standard

    // @Param: _ACCZ_FLTT
    // @DisplayName: Acceleration (vertical) controller target frequency in Hz
    // @Description: Acceleration (vertical) controller target frequency in Hz
    // @Range: 1 50
    // @Increment: 1
    // @Units: Hz
    // @User: Standard

    // @Param: _ACCZ_FLTE
    // @DisplayName: Acceleration (vertical) controller error frequency in Hz
    // @Description: Acceleration (vertical) controller error frequency in Hz
    // @Range: 1 100
    // @Increment: 1
    // @Units: Hz
    // @User: Standard

    // @Param: _ACCZ_FLTD
    // @DisplayName: Acceleration (vertical) controller derivative frequency in Hz
    // @Description: Acceleration (vertical) controller derivative frequency in Hz
    // @Range: 1 100
    // @Increment: 1
    // @Units: Hz
    // @User: Standard

    // @Param: _ACCZ_SMAX
    // @DisplayName: Accel (vertical) slew rate limit
    // @Description: Sets an upper limit on the slew rate produced by the combined P and D gains. If the amplitude of the control action produced by the rate feedback exceeds this value, then the D+P gain is reduced to respect the limit. This limits the amplitude of high frequency oscillations caused by an excessive gain. The limit should be set to no more than 25% of the actuators maximum slew rate to allow for load effects. Note: The gain will not be reduced to less than 10% of the nominal value. A value of zero will disable this feature.
    // @Range: 0 200
    // @Increment: 0.5
    // @User: Advanced

    AP_SUBGROUPINFO(_pid_accel_z, "_ACCZ_", 4, AC_PosControl, AC_PID),

    // @Param: _POSXY_P
    // @DisplayName: Position (horizontal) controller P gain
    // @Description: Position controller P gain.  Converts the distance (in the latitude direction) to the target location into a desired speed which is then passed to the loiter latitude rate controller
    // @Range: 0.500 2.000
    // @User: Standard
    AP_SUBGROUPINFO(_p_pos_xy, "_POSXY_", 5, AC_PosControl, AC_P_2D),

    // @Param: _VELXY_P
    // @DisplayName: Velocity (horizontal) P gain
    // @Description: Velocity (horizontal) P gain.  Converts the difference between desired and actual velocity to a target acceleration
    // @Range: 0.1 6.0
    // @Increment: 0.1
    // @User: Advanced

    // @Param: _VELXY_I
    // @DisplayName: Velocity (horizontal) I gain
    // @Description: Velocity (horizontal) I gain.  Corrects long-term difference between desired and actual velocity to a target acceleration
    // @Range: 0.02 1.00
    // @Increment: 0.01
    // @User: Advanced

    // @Param: _VELXY_D
    // @DisplayName: Velocity (horizontal) D gain
    // @Description: Velocity (horizontal) D gain.  Corrects short-term changes in velocity
    // @Range: 0.00 1.00
    // @Increment: 0.001
    // @User: Advanced

    // @Param: _VELXY_IMAX
    // @DisplayName: Velocity (horizontal) integrator maximum
    // @Description: Velocity (horizontal) integrator maximum.  Constrains the target acceleration that the I gain will output
    // @Range: 0 4500
    // @Increment: 10
    // @Units: cm/s/s
    // @User: Advanced

    // @Param: _VELXY_FILT
    // @DisplayName: Velocity (horizontal) input filter
    // @Description: Velocity (horizontal) input filter.  This filter (in Hz) is applied to the input for P and I terms
    // @Range: 0 100
    // @Units: Hz
    // @User: Advanced

    // @Param: _VELXY_D_FILT
    // @DisplayName: Velocity (horizontal) input filter
    // @Description: Velocity (horizontal) input filter.  This filter (in Hz) is applied to the input for D term
    // @Range: 0 100
    // @Units: Hz
    // @User: Advanced

    // @Param: _VELXY_FF
    // @DisplayName: Velocity (horizontal) feed forward gain
    // @Description: Velocity (horizontal) feed forward gain.  Converts the difference between desired velocity to a target acceleration
    // @Range: 0 6
    // @Increment: 0.01
    // @User: Advanced
    AP_SUBGROUPINFO(_pid_vel_xy, "_VELXY_", 6, AC_PosControl, AC_PID_2D),

    // @Param: _ANGLE_MAX
    // @DisplayName: Position Control Angle Max
    // @Description: Maximum lean angle autopilot can request.  Set to zero to use ANGLE_MAX parameter value
    // @Units: deg
    // @Range: 0 45
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("_ANGLE_MAX", 7, AC_PosControl, _lean_angle_max, 0.0f),

    // @Param: _TC
    // @DisplayName: Time constant for kinimatic input shaping
    // @Description: This is the time constant used to determine how quickly the aircraft varies the acceleration target
    // @Units: s
    // @Range: 0 10
    // @Increment: 0.01
    // @User: Advanced
    AP_GROUPINFO("_TC", 8, AC_PosControl, _shaping_tc, 0.25f),

    AP_GROUPEND
};

// Default constructor.
// Note that the Vector/Matrix constructors already implicitly zero
// their values.
//
AC_PosControl::AC_PosControl(AP_AHRS_View& ahrs, const AP_InertialNav& inav,
                             const AP_Motors& motors, AC_AttitudeControl& attitude_control) :
    _ahrs(ahrs),
    _inav(inav),
    _motors(motors),
    _attitude_control(attitude_control),
    _p_pos_z(POSCONTROL_POS_Z_P, POSCONTROL_DT_400HZ),
    _pid_vel_z(POSCONTROL_VEL_Z_P, 0.0f, 0.0f, 0.0f, POSCONTROL_VEL_Z_IMAX, POSCONTROL_VEL_Z_FILT_HZ, POSCONTROL_VEL_Z_FILT_D_HZ, POSCONTROL_DT_400HZ),
    _pid_accel_z(POSCONTROL_ACC_Z_P, POSCONTROL_ACC_Z_I, POSCONTROL_ACC_Z_D, 0.0f, POSCONTROL_ACC_Z_IMAX, 0.0f, POSCONTROL_ACC_Z_FILT_HZ, 0.0f, POSCONTROL_DT_400HZ),
    _p_pos_xy(POSCONTROL_POS_XY_P, POSCONTROL_DT_400HZ),
    _pid_vel_xy(POSCONTROL_VEL_XY_P, POSCONTROL_VEL_XY_I, POSCONTROL_VEL_XY_D, 0.0f, POSCONTROL_VEL_XY_IMAX, POSCONTROL_VEL_XY_FILT_HZ, POSCONTROL_VEL_XY_FILT_D_HZ, POSCONTROL_DT_400HZ),
    _dt(POSCONTROL_DT_400HZ),
    _speed_down_cms(POSCONTROL_SPEED_DOWN),
    _speed_up_cms(POSCONTROL_SPEED_UP),
    _speed_cms(POSCONTROL_SPEED),
    _accel_z_cms(POSCONTROL_ACCEL_Z),
    _accel_cms(POSCONTROL_ACCEL_XY)
{
    AP_Param::setup_object_defaults(this, var_info);

    // initialise flags
    _limit.pos_up = true;
    _limit.pos_down = true;
    _limit.vel_up = true;
    _limit.vel_down = true;
    _limit.accel_xy = true;
}

///
/// z-axis position controller
///

/// set_dt - sets time delta in seconds for all controllers (i.e. 100hz = 0.01, 400hz = 0.0025)
void AC_PosControl::set_dt(float delta_sec)
{
    _dt = delta_sec;

    // update PID controller dt
    _p_pos_z.set_dt(_dt);
    _pid_vel_z.set_dt(_dt);
    _pid_accel_z.set_dt(_dt);
    _p_pos_xy.set_dt(_dt);
    _pid_vel_xy.set_dt(_dt);

    // update rate z-axis velocity error and accel error filters
    _vel_error_filter.set_cutoff_frequency(POSCONTROL_VEL_ERROR_CUTOFF_FREQ);
}

/// set_max_speed_accel_z - set the maximum climb and descent rates and acceleration
/// To-Do: call this in the main code as part of flight mode initialisation
void AC_PosControl::set_max_speed_accel_z(float speed_down, float speed_up, float accel_cmss)
{
    // ensure speed_down is always negative
    speed_down = -fabsf(speed_down);

    // exit immediately if no change in speed up or down
    if (is_equal(_speed_down_cms, speed_down) && is_equal(_speed_up_cms, speed_up) && is_equal(_accel_z_cms, accel_cmss)) {
        return;
    }

    // sanity check and update
    if (is_positive(speed_up)) {
        _speed_down_cms = speed_down;
    }
    if (is_negative(speed_down)) {
        _speed_up_cms = speed_up;
    }
    if (is_positive(accel_cmss)) {
        _accel_z_cms = accel_cmss;
    }
    // define maximum position error and maximum first and second differential limits
    _p_pos_z.set_limits(0.0f, 0.0f, -fabsf(_speed_down_cms), _speed_up_cms, _accel_z_cms, 0.0f);
}

// get_alt_error - returns altitude error in cm
float AC_PosControl::get_alt_error() const
{
    return (_pos_target.z - _inav.get_altitude());
}

/// set_target_to_stopping_point_z - returns reasonable stopping altitude in cm above home
// todo: this function should not be needed.
void AC_PosControl::set_target_to_stopping_point_z()
{
    get_stopping_point_z(_pos_target);
}

/// get_stopping_point_z - calculates stopping point based on current position, velocity, vehicle acceleration
void AC_PosControl::get_stopping_point_z(Vector3f& stopping_point) const
{
    const float curr_pos_z = _inav.get_altitude();
    float curr_vel_z = _inav.get_velocity_z();

    // if position controller is active add current velocity error to avoid sudden jump in acceleration
    if (is_active_z()) {
        curr_vel_z -= _vel_desired.z;
    }

    // avoid divide by zero by using current position if kP is very low or acceleration is zero
    if (_p_pos_z.kP() <= 0.0f || _accel_z_cms <= 0.0f) {
        stopping_point.z = curr_pos_z;
        return;
    }

    stopping_point.z = curr_pos_z + constrain_float(stopping_distance(curr_vel_z, _p_pos_z.kP(), _accel_z_cms), - POSCONTROL_STOPPING_DIST_DOWN_MAX, POSCONTROL_STOPPING_DIST_UP_MAX);
}

/// init_takeoff - initialises target altitude if we are taking off
void AC_PosControl::init_takeoff()
{
    const Vector3f& curr_pos = _inav.get_position();

    _pos_target.z = curr_pos.z;

    // shift difference between last motor out and hover throttle into accelerometer I
    _pid_accel_z.set_integrator((_attitude_control.get_throttle_in() - _motors.get_throttle_hover()) * 1000.0f);

    // initialise ekf reset handler
    init_ekf_z_reset();

    // initialise z_controller time out
    _last_update_z_us = AP_HAL::micros64();
}

// is_active_z - returns true if the z-axis position controller has been run very recently
bool AC_PosControl::is_active_z() const
{
    return ((AP_HAL::micros64() - _last_update_z_us) <= POSCONTROL_ACTIVE_TIMEOUT_US);
}

/// update_z_controller - fly to altitude in cm above home
void AC_PosControl::update_z_controller()
{
    // call z-axis position controller
    run_z_controller();
}

/// relax_alt_hold_controllers - set all desired and targets to measured
void AC_PosControl::relax_alt_hold_controllers(float throttle_setting)
{
    Vector3f curr_pos = _inav.get_position();
    _pos_target.z = curr_pos.z;

    const Vector3f &curr_vel = _inav.get_velocity();
    _vel_desired.z = curr_vel.z;
    _vel_target.z = 0;

    const Vector3f &curr_accel = _ahrs.get_accel_ef_blended();

    // Reset I term of velocity PID
    _pid_vel_z.reset_filter();
    _pid_vel_z.set_integrator(0.0f);

    // Set accel PID I term based on the current throttle
    _pid_accel_z.set_integrator((throttle_setting - _motors.get_throttle_hover()) * 1000.0f);
    _accel_desired.z = -(curr_accel.z + GRAVITY_MSS) * 100.0f;
    _accel_target.z = 0;
    _pid_accel_z.reset_filter();

    // initialise ekf z reset handler
    init_ekf_z_reset();

    // initialise z_controller time out
    _last_update_z_us = AP_HAL::micros64();
}

/// init_z - initialise the position controller to the current position and velocity with zero acceleration.
///     This function should be called before input_vel_z or input_pos_vel_z are used.
void AC_PosControl::init_z_controller()
{
    Vector3f curr_pos = _inav.get_position();
    _pos_target.z = curr_pos.z;

    const Vector3f &curr_vel = _inav.get_velocity();
    _vel_desired.z = curr_vel.z;
    _vel_target.z = 0;

    const Vector3f &curr_accel = _ahrs.get_accel_ef_blended();

    // Reset I term of velocity PID
    _pid_vel_z.reset_filter();
    _pid_vel_z.set_integrator(0.0f);

    // Set accel PID I term based on the current throttle
    _pid_accel_z.set_integrator((_attitude_control.get_throttle_in() - _motors.get_throttle_hover()) * 1000.0f);
    _accel_desired.z = -(curr_accel.z + GRAVITY_MSS) * 100.0f;
    _accel_target.z = 0;
    _pid_accel_z.reset_filter();

    // initialise ekf z reset handler
    init_ekf_z_reset();

    // initialise z_controller time out
    _last_update_z_us = AP_HAL::micros64();
}

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
void AC_PosControl::input_vel_accel_z(const Vector3f& vel, const Vector3f& accel, bool force_descend)
{
    // check for ekf z position reset
    check_for_ekf_z_reset();

    // calculated increased maximum acceleration if over speed
    float accel_z_cms = _accel_z_cms;
    if (_vel_desired.z < _speed_down_cms && !is_zero(_speed_down_cms)) {
        accel_z_cms *= POSCONTROL_OVERSPEED_GAIN_Z * _vel_desired.z / _speed_down_cms;
    }
    if (_vel_desired.z > _speed_up_cms && !is_zero(_speed_up_cms)) {
        accel_z_cms *= POSCONTROL_OVERSPEED_GAIN_Z * _vel_desired.z / _speed_up_cms;
    }
    shape_vel_accel(vel.z, accel.z,
        _vel_desired.z, _accel_desired.z,
        _speed_down_cms, _speed_up_cms,
        -constrain_float(accel_z_cms, 0.0f, 750.0f), accel_z_cms,
        POSCONTROL_Z_SHAPER_TC, _dt);

    // adjust desired alt if motors have not hit their limits
    update_pos_vel_accel(_pos_target.z, _vel_desired.z, _accel_desired.z, _dt,
        (_motors.limit.throttle_lower && !force_descend), _motors.limit.throttle_upper,
        _p_pos_z.get_error(), _pid_vel_z.get_error());
}

/// input_pos_vel_z calculate a jerk limited path from the current position, velocity and acceleration to an input position and velocity.
///     The function takes the current position, velocity, and acceleration and calculates the required jerk limited adjustment to the acceleration for the next time dt.
///     The kinematic path is constrained by :
///         maximum velocity - vel_max,
///         maximum acceleration - accel_max,
///         time constant - tc.
///     The time constant defines the acceleration error decay in the kinematic path as the system approaches constant acceleration.
///     The time constant also defines the time taken to achieve the maximum acceleration.
///     The time constant must be positive.
///     The function alters the input position to be the closest position that the system could reach zero acceleration in the minimum time.
void AC_PosControl::input_pos_vel_accel_z(const Vector3f& pos, const Vector3f& vel, const Vector3f& accel)
{
    // check for ekf xy position reset
    check_for_ekf_z_reset();

    // calculated increased maximum acceleration if over speed
    float accel_z_cms = _accel_z_cms;
    if (_vel_desired.z < _speed_down_cms && !is_zero(_speed_down_cms)) {
        accel_z_cms *= POSCONTROL_OVERSPEED_GAIN_Z * _vel_desired.z / _speed_down_cms;
    }
    if (_vel_desired.z > _speed_up_cms && !is_zero(_speed_up_cms)) {
        accel_z_cms *= POSCONTROL_OVERSPEED_GAIN_Z * _vel_desired.z / _speed_up_cms;
    }
    shape_pos_vel_accel(pos.z, vel.z, accel.z,
        _pos_target.z, _vel_desired.z, _accel_desired.z,
        0.0f, _speed_down_cms, _speed_up_cms,
        -constrain_float(accel_z_cms, 0.0f, 750.0f), accel_z_cms,
        POSCONTROL_Z_SHAPER_TC, _dt);

    // adjust desired alt if motors have not hit their limits
    update_pos_vel_accel(_pos_target.z, _vel_desired.z, _accel_desired.z, _dt,
        _motors.limit.throttle_lower, _motors.limit.throttle_upper,
        _p_pos_z.get_error(), _pid_vel_z.get_error());
}

// run position control for Z axis
// target altitude should be set with one of these functions: set_alt_target, set_target_to_stopping_point_z, init_takeoff
// calculates desired rate in earth-frame z axis and passes to rate controller
// vel_up_max, vel_down_max should have already been set before calling this method
void AC_PosControl::run_z_controller()
{
    // Check for z_controller time out
    const uint64_t now_us = AP_HAL::micros64();
    if ((now_us - _last_update_z_us) >= POSCONTROL_ACTIVE_TIMEOUT_US) {
        init_z_controller();
        INTERNAL_ERROR(AP_InternalError::error_t::flow_of_control);
    }
    _last_update_z_us = now_us;

    float curr_alt = _inav.get_altitude();
    // calculate the target velocity correction
    _vel_target.z = _p_pos_z.update_all(_pos_target.z, curr_alt, _limit.pos_down, _limit.pos_up);
    // add feed forward component
    _vel_target.z += constrain_float(_vel_desired.z, -fabsf(_speed_down_cms), _speed_up_cms);

    // Velocity Controller

    const Vector3f& curr_vel = _inav.get_velocity();
    _accel_target.z = _pid_vel_z.update_all(_vel_target.z, curr_vel.z);
    _accel_target.z += _accel_desired.z;

    // Acceleration Controller

    // Calculate Earth Frame Z acceleration
    const float z_accel_meas = get_z_accel_cmss();

    // ensure imax is always large enough to overpower hover throttle
    if (_motors.get_throttle_hover() * 1000.0f > _pid_accel_z.imax()) {
        _pid_accel_z.imax(_motors.get_throttle_hover() * 1000.0f);
    }
    float thr_out;
    if (_vibe_comp_enabled) {
        thr_out = get_throttle_with_vibration_override();
    } else {
        thr_out = _pid_accel_z.update_all(_accel_target.z, z_accel_meas, (_motors.limit.throttle_lower || _motors.limit.throttle_upper)) * 0.001f;
        thr_out += _pid_accel_z.get_ff() * 0.001f;
    }
    thr_out += _motors.get_throttle_hover();

    // Actuator commands

    // send throttle to attitude controller with angle boost
    _attitude_control.set_throttle_out(thr_out, true, POSCONTROL_THROTTLE_CUTOFF_FREQ);

    // Check for vertical controller health

    // _speed_down_cms is checked to be non-zero when set
    float error_ratio = _vel_error.z/_speed_down_cms;
    _vel_z_control_ratio += _dt*0.1f*(0.5-error_ratio);
    _vel_z_control_ratio = constrain_float(_vel_z_control_ratio, 0.0f, 2.0f);
}

// get throttle using vibration-resistant calculation (uses feed forward with manually calculated gain)
float AC_PosControl::get_throttle_with_vibration_override()
{
    _accel_desired.z = 0.0f;
    const float thr_per_accelz_cmss = _motors.get_throttle_hover() / (GRAVITY_MSS * 100.0f);
    // during vibration compensation use feed forward with manually calculated gain
    // ToDo: clear pid_info P, I and D terms for logging
    if (!(_motors.limit.throttle_lower || _motors.limit.throttle_upper) || ((is_positive(_pid_accel_z.get_i()) && is_negative(_vel_error.z)) || (is_negative(_pid_accel_z.get_i()) && is_positive(_vel_error.z)))) {
        _pid_accel_z.set_integrator(_pid_accel_z.get_i() + _dt * thr_per_accelz_cmss * 1000.0f * _vel_error.z * _pid_vel_z.kP() * POSCONTROL_VIBE_COMP_I_GAIN);
    }
    return POSCONTROL_VIBE_COMP_P_GAIN * thr_per_accelz_cmss * _accel_target.z + _pid_accel_z.get_i() * 0.001f;
}

///
/// lateral position controller
///

/// set_max_accel_xy - set the maximum horizontal acceleration in cm/s/s
void AC_PosControl::set_max_speed_accel_xy(float speed_cms, float accel_cmss)
{
    // return immediately if no change
    if (is_equal(_speed_cms, speed_cms) && is_equal(_accel_cms, accel_cmss)) {
        return;
    }
    _speed_cms = speed_cms;
    _accel_cms = accel_cmss;
    _p_pos_xy.set_limits(0.0f, _speed_cms, _accel_cms, 0.0f);
}

/// set_pos_target in cm from home
void AC_PosControl::set_pos_target(const Vector3f& position)
{
    _pos_target = position;
    _vel_desired.z = 0.0f;
}

/// set position, velocity and acceleration targets
void AC_PosControl::set_pos_vel_accel_target(const Vector3f& pos, const Vector3f& vel, const Vector3f& accel)
{
    _pos_target = pos;
    _vel_desired = vel;
    _accel_desired = accel;
}

/// set_target_to_stopping_point_xy - sets horizontal target to reasonable stopping position in cm from home
void AC_PosControl::set_target_to_stopping_point_xy()
{
    get_stopping_point_xy(_pos_target);
}

/// get_stopping_point_xy - calculates stopping point based on current position, velocity, vehicle acceleration
///     distance_max allows limiting distance to stopping point
///     results placed in stopping_position vector
///     set_max_accel_xy() should be called before this method to set vehicle acceleration
void AC_PosControl::get_stopping_point_xy(Vector3f &stopping_point) const
{
    const Vector3f curr_pos = _inav.get_position();
    Vector3f curr_vel = _inav.get_velocity();
    float stopping_dist;	    // the distance within the vehicle can stop
    float kP = _p_pos_xy.kP();

    // add velocity error to current velocity
    if (is_active_xy()) {
        curr_vel.x += _vel_error.x;
        curr_vel.y += _vel_error.y;
    }

    // calculate current velocity
    float vel_total = norm(curr_vel.x, curr_vel.y);

    stopping_dist = stopping_distance(constrain_float(vel_total, 0.0, _speed_cms), kP, _accel_cms);

    // convert the stopping distance into a stopping point using velocity vector
    stopping_point.x = curr_pos.x + (stopping_dist * curr_vel.x / vel_total);
    stopping_point.y = curr_pos.y + (stopping_dist * curr_vel.y / vel_total);
}

/// get_bearing_to_target - get bearing to target position in centi-degrees
int32_t AC_PosControl::get_bearing_to_target() const
{
    return get_bearing_cd(_inav.get_position(), _pos_target);
}

// is_active_xy - returns true if the xy position controller has been run very recently
bool AC_PosControl::is_active_xy() const
{
    return ((AP_HAL::micros64() - _last_update_xy_us) <= POSCONTROL_ACTIVE_TIMEOUT_US);
}

/// get_lean_angle_max_cd - returns the maximum lean angle the autopilot may request
float AC_PosControl::get_lean_angle_max_cd() const
{
    if (is_zero(_lean_angle_max)) {
        return _attitude_control.lean_angle_max();
    }
    return _lean_angle_max * 100.0f;
}

/// standby_xyz_reset - resets I terms and removes position error
///     This function will let Loiter and Alt Hold continue to operate
///     in the event that the flight controller is in control of the
///     aircraft when in standby.
void AC_PosControl::standby_xyz_reset()
{
    // Set _pid_accel_z integrator to zero.
    _pid_accel_z.set_integrator(0.0f);

    // Set the target position to the current pos.
    _pos_target = _inav.get_position();

    // Set _pid_vel_xy integrator and derivative to zero.
    _pid_vel_xy.reset_filter();

    // initialise ekf xy reset handler
    init_ekf_xy_reset();
}

float AC_PosControl::time_since_last_xy_update() const
{
    const uint64_t now_us = AP_HAL::micros64();
    return (now_us - _last_update_xy_us) * 1.0e-6f;
}

// write log to dataflash
void AC_PosControl::write_log()
{
    float accel_x, accel_y;
    lean_angles_to_accel_xy(accel_x, accel_y);

    AP::logger().Write_PSC(get_pos_target(), _inav.get_position(), get_vel_target(), _inav.get_velocity(), get_accel_target(), accel_x, accel_y);
    AP::logger().Write_PSCZ(get_pos_target().z, _inav.get_position().z,
                            get_desired_velocity().z, get_vel_target().z, _inav.get_velocity().z,
                            _accel_desired.z, get_accel_target().z, get_z_accel_cmss(), _attitude_control.get_throttle_in());
}

/// init_pos_vel_accel_xyz - initialise the velocity controller - should be called once before the caller attempts to use the controller
// todo: remove
void AC_PosControl::init_xyz()
{
    init_z_controller();
    init_xy_controller();
}

// relax_velocity_controller_xy - initialise the position controller to the current position, velocity, acceleration and level attitude.
///     This function is similar to the initialisation but conditions the controller to request level attitude
void AC_PosControl::relax_velocity_controller_xy()
{
    // set roll, pitch lean angle targets to current attitude
    const Vector3f &att_target_euler_cd = _attitude_control.get_att_target_euler_cd();
    _roll_target = 0.0f;
    _pitch_target = 0.0f;
    _yaw_target = att_target_euler_cd.z; // todo: this should be thrust vector heading, not yaw.
    _yaw_rate_target = 0.0f;

    Vector3f curr_pos = _inav.get_position();
    _pos_target.x = curr_pos.x;
    _pos_target.y = curr_pos.y;

    const Vector3f &curr_vel = _inav.get_velocity();
    _vel_desired.x = curr_vel.x;
    _vel_desired.y = curr_vel.y;
    _vel_target.x = 0.0f;
    _vel_target.y = 0.0f;

    _accel_desired.x = 0.0f;
    _accel_desired.y = 0.0f;
    _accel_target.x = 0.0f;
    _accel_target.y = 0.0f;

    // initialise I terms from lean angles
    _pid_vel_xy.reset_filter();
    _pid_vel_xy.reset_I();

    // initialise ekf xy reset handler
    init_ekf_xy_reset();

    // initialise z_controller time out
    _last_update_xy_us = AP_HAL::micros64();
}

/// init_pos_vel_accel_xy - initialise the position controller to the current position, velocity, acceleration and attitude.
///     This function should be called before input_vel_xy or input_pos_vel_xy are used.
void AC_PosControl::init_xy_controller()
{
    // set roll, pitch lean angle targets to current attitude
    const Vector3f &att_target_euler_cd = _attitude_control.get_att_target_euler_cd();
    _roll_target = att_target_euler_cd.x;
    _pitch_target = att_target_euler_cd.y;
    _yaw_target = att_target_euler_cd.z; // todo: this should be thrust vector heading, not yaw.
    _yaw_rate_target = 0.0f;

    Vector3f curr_pos = _inav.get_position();
    _pos_target.x = curr_pos.x;
    _pos_target.y = curr_pos.y;

    const Vector3f &curr_vel = _inav.get_velocity();
    _vel_desired.x = curr_vel.x;
    _vel_desired.y = curr_vel.y;
    _vel_target.x = 0.0f;
    _vel_target.y = 0.0f;

    const Vector3f &curr_accel = _ahrs.get_accel_ef_blended() * 100.0f;
    _accel_desired.x = curr_accel.x;
    _accel_desired.y = curr_accel.y;
    _accel_target.x = 0.0f;
    _accel_target.y = 0.0f;

    // initialise I terms from lean angles
    _pid_vel_xy.reset_filter();
    Vector3f accel_target;
    lean_angles_to_accel_xy(accel_target.x, accel_target.y);
    _pid_vel_xy.set_integrator(accel_target - _accel_desired);

    // initialise ekf xy reset handler
    init_ekf_xy_reset();

    // initialise z_controller time out
    _last_update_xy_us = AP_HAL::micros64();
}

/// input_vel_xy calculate a jerk limited path from the current position, velocity and acceleration to an input velocity.
///     The function takes the current position, velocity, and acceleration and calculates the required jerk limited adjustment to the acceleration for the next time dt.
///     The kinematic path is constrained by:
///         accel_max : maximum acceleration
///         tc : time constant
///     The time constant defines the acceleration error decay in the kinematic path as the system approaches constant acceleration.
///     The time constant also defines the time taken to achieve the maximum acceleration.
///     The time constant must be positive.
///     The function alters the input velocity to be the velocity that the system could reach zero acceleration in the minimum time.
void AC_PosControl::input_vel_accel_xy(const Vector3f& vel, const Vector3f& accel)
{
    // check for ekf xy position reset
    check_for_ekf_xy_reset();

    update_pos_vel_accel_xy(_pos_target, _vel_desired, _accel_desired, _dt,
        _limit.accel_xy, _p_pos_xy.get_error(), _pid_vel_xy.get_error());

    shape_vel_accel_xy(vel, accel, _vel_desired, _accel_desired, _speed_cms, _accel_cms, _shaping_tc, _dt);
}

/// input_pos_vel_xy calculate a jerk limited path from the current position, velocity and acceleration to an input position and velocity.
///     The function takes the current position, velocity, and acceleration and calculates the required jerk limited adjustment to the acceleration for the next time dt.
///     The kinematic path is constrained by:
///         vel_max : maximum velocity
///         accel_max : maximum acceleration
///         tc : time constant
///     The time constant defines the acceleration error decay in the kinematic path as the system approaches constant acceleration.
///     The time constant also defines the time taken to achieve the maximum acceleration.
///     The time constant must be positive.
///     The function alters the input position to be the closest position that the system could reach zero acceleration in the minimum time.
void AC_PosControl::input_pos_vel_accel_xy(const Vector3f& pos, const Vector3f& vel, const Vector3f& accel)
{
    // check for ekf xy position reset
    check_for_ekf_xy_reset();

    update_pos_vel_accel_xy(_pos_target, _vel_desired, _accel_desired, _dt,
        _limit.accel_xy, _p_pos_xy.get_error(), _pid_vel_xy.get_error());

    shape_pos_vel_accel_xy(pos, vel, accel, _pos_target, _vel_desired, _accel_desired, _speed_cms, _speed_cms, _accel_cms, _shaping_tc, _dt);
}

/// run horizontal position controller correcting position and velocity
///     converts position (_pos_target) to target velocity (_vel_target)
///     desired velocity (_vel_desired) is combined into final target velocity
///     converts desired velocities in lat/lon directions to accelerations in lat/lon frame
///     converts desired accelerations provided in lat/lon frame to roll/pitch angles
void AC_PosControl::run_xy_controller()
{
    // Check for position control time out
    const uint64_t now_us = AP_HAL::micros64();
    if ((now_us - _last_update_xy_us) >= POSCONTROL_ACTIVE_TIMEOUT_US) {
        init_xy_controller();
        // todo: prevent internal error going off after initialisation
        INTERNAL_ERROR(AP_InternalError::error_t::flow_of_control);
    }
    _last_update_xy_us = now_us;

    float ekfGndSpdLimit, ekfNavVelGainScaler;
    AP::ahrs_navekf().getEkfControlLimits(ekfGndSpdLimit, ekfNavVelGainScaler);

    // Position Controller

    const Vector3f &curr_pos = _inav.get_position();
    Vector2f vel_target = _p_pos_xy.update_all(_pos_target.x, _pos_target.y, curr_pos);

    // add velocity feed-forward scaled to compensate for optical flow measurement induced EKF noise
    vel_target *= ekfNavVelGainScaler;
    _vel_target.x = vel_target.x;
    _vel_target.y = vel_target.y;
    // acceleration to correct for velocity error and scale PID output to compensate for optical flow measurement induced EKF noise
    _vel_target.x += _vel_desired.x;
    _vel_target.y += _vel_desired.y;

    // Velocity Controller

    // check if vehicle velocity is being overridden
    // todo: remove this and use input shaping
    if (_flags.vehicle_horiz_vel_override) {
        _flags.vehicle_horiz_vel_override = false;
    } else {
        _vehicle_horiz_vel.x = _inav.get_velocity().x;
        _vehicle_horiz_vel.y = _inav.get_velocity().y;
    }
    Vector2f accel_target = _pid_vel_xy.update_all(Vector2f{_vel_target.x, _vel_target.y}, _vehicle_horiz_vel, _limit.accel_xy);
    // acceleration to correct for velocity error and scale PID output to compensate for optical flow measurement induced EKF noise
    accel_target *= ekfNavVelGainScaler;

    // pass the correction acceleration to the target acceleration output
    _accel_target.x = accel_target.x;
    _accel_target.y = accel_target.y;

    // Add feed forward into the target acceleration output
    _accel_target.x += _accel_desired.x;
    _accel_target.y += _accel_desired.y;

    // Acceleration Controller

    // limit acceleration using maximum lean angles
    float angle_max = MIN(_attitude_control.get_althold_lean_angle_max(), get_lean_angle_max_cd());
    float accel_max = MIN(GRAVITY_MSS * 100.0f * tanf(ToRad(angle_max * 0.01f)), POSCONTROL_ACCEL_XY_MAX);
    _limit.accel_xy = _accel_target.limit_length_xy(accel_max);

    // update angle targets that will be passed to stabilize controller
    accel_to_lean_angles(_accel_target.x, _accel_target.y, _roll_target, _pitch_target);
    calculate_yaw_and_rate_yaw();
}

///
/// private methods
///

// get_lean_angles_to_accel - convert roll, pitch lean angles to lat/lon frame accelerations in cm/s/s
void AC_PosControl::accel_to_lean_angles(float accel_x_cmss, float accel_y_cmss, float& roll_target, float& pitch_target) const
{
    // rotate accelerations into body forward-right frame
    const float accel_forward = accel_x_cmss * _ahrs.cos_yaw() + accel_y_cmss * _ahrs.sin_yaw();
    const float accel_right = -accel_x_cmss * _ahrs.sin_yaw() + accel_y_cmss * _ahrs.cos_yaw();

    // update angle targets that will be passed to stabilize controller
    pitch_target = atanf(-accel_forward / (GRAVITY_MSS * 100.0f)) * (18000.0f / M_PI);
    float cos_pitch_target = cosf(pitch_target * M_PI / 18000.0f);
    roll_target = atanf(accel_right * cos_pitch_target / (GRAVITY_MSS * 100.0f)) * (18000.0f / M_PI);
}

// get_lean_angles_to_accel_xy - convert roll, pitch lean target angles to lat/lon frame accelerations in cm/s/s
void AC_PosControl::lean_angles_to_accel_xy(float& accel_x_cmss, float& accel_y_cmss) const
{
    // rotate our roll, pitch angles into lat/lon frame
    Vector3f att_target_euler = _attitude_control.get_att_target_euler_rad();
    att_target_euler.z = _ahrs.yaw;
    Vector3f accel_cmss = lean_angles_to_accel(att_target_euler);

    accel_x_cmss = accel_cmss.x;
    accel_y_cmss = accel_cmss.y;
}

// get_lean_angles_to_accel - convert roll, pitch lean target angles to lat/lon frame accelerations in cm/s/s
Vector3f AC_PosControl::lean_angles_to_accel(const Vector3f& att_target_euler) const
{
    // rotate our roll, pitch angles into lat/lon frame
    const float sin_roll = sinf(att_target_euler.x);
    const float cos_roll = cosf(att_target_euler.x);
    const float sin_pitch = sinf(att_target_euler.y);
    const float cos_pitch = cosf(att_target_euler.y);
    const float sin_yaw = sinf(att_target_euler.z);
    const float cos_yaw = cosf(att_target_euler.z);

    Vector3f accel_cmss;
    accel_cmss.x = (GRAVITY_MSS * 100) * (-cos_yaw * sin_pitch * cos_roll - sin_yaw * sin_roll) / MAX(cos_roll * cos_pitch, 0.1f);
    accel_cmss.y = (GRAVITY_MSS * 100) * (-sin_yaw * sin_pitch * cos_roll + cos_yaw * sin_roll) / MAX(cos_roll * cos_pitch, 0.1f);
    accel_cmss.z = (GRAVITY_MSS * 100);
    return accel_cmss;
}

// returns the NED target acceleration vector for attitude control
Vector3f AC_PosControl::get_thrust_vector() const
{
    Vector3f accel_target = get_accel_target();
    accel_target.z = -GRAVITY_MSS * 100.0f;
    return accel_target;
}

// calculate_yaw_and_rate_yaw - calculate the vehicle yaw and rate of yaw.
bool AC_PosControl::calculate_yaw_and_rate_yaw()
{
    // Calculate the turn rate
    float turn_rate = 0.0f;
    const Vector2f vel_desired_xy(_vel_desired.x, _vel_desired.y);
    const Vector2f accel_desired_xy(_accel_desired.x, _accel_desired.y);
    const float vel_desired_xy_len = vel_desired_xy.length();
    if (is_positive(vel_desired_xy_len)) {
        const float accel_forward = (accel_desired_xy.x * vel_desired_xy.x + accel_desired_xy.y * vel_desired_xy.y)/vel_desired_xy_len;
        const Vector2f accel_turn = accel_desired_xy - vel_desired_xy * accel_forward / vel_desired_xy_len;
        const float accel_turn_xy_len = accel_turn.length();
        turn_rate = accel_turn_xy_len / vel_desired_xy_len;
        if ((accel_turn.y * vel_desired_xy.x - accel_turn.x * vel_desired_xy.y) < 0.0) {
            turn_rate = -turn_rate;
        }
    }

    // update the target yaw if origin and destination are at least 2m apart horizontally
    if (vel_desired_xy_len > _speed_cms * 0.05f) {
        _yaw_target = degrees(vel_desired_xy.angle()) * 100.0f;
        _yaw_rate_target = turn_rate*degrees(100.0f);
        return true;
    }
    return false;
}

/// initialise ekf xy position reset check
void AC_PosControl::init_ekf_xy_reset()
{
    Vector2f pos_shift;
    _ekf_xy_reset_ms = _ahrs.getLastPosNorthEastReset(pos_shift);
}

/// check for ekf position reset and adjust loiter or brake target position
void AC_PosControl::check_for_ekf_xy_reset()
{
    // check for position shift
    // todo: replace with with a simple getLastReset();
    Vector2f pos_shift;
    uint32_t reset_ms = _ahrs.getLastPosNorthEastReset(pos_shift);
    if (reset_ms != _ekf_xy_reset_ms) {

        const Vector3f& curr_pos = _inav.get_position();
        _pos_target.x = curr_pos.x + _p_pos_xy.get_error().x;
        _pos_target.y = curr_pos.y + _p_pos_xy.get_error().y;

        const Vector3f& curr_vel = _inav.get_velocity();
        _vel_target.x = curr_vel.x + _pid_vel_xy.get_error().x;
        _vel_target.y = curr_vel.y + _pid_vel_xy.get_error().y;

        _ekf_xy_reset_ms = reset_ms;
    }
}

/// initialise ekf z axis reset check
void AC_PosControl::init_ekf_z_reset()
{
    float alt_shift;
    _ekf_z_reset_ms = _ahrs.getLastPosDownReset(alt_shift);
}

/// check for ekf position reset and adjust loiter or brake target position
void AC_PosControl::check_for_ekf_z_reset()
{
    // check for position shift
    float alt_shift;
    uint32_t reset_ms = _ahrs.getLastPosDownReset(alt_shift);
    if (reset_ms != 0 && reset_ms != _ekf_z_reset_ms) {

        const Vector3f& curr_pos = _inav.get_position();
        _pos_target.z = curr_pos.z + _p_pos_z.get_error();

        const Vector3f& curr_vel = _inav.get_velocity();
        _vel_target.z = curr_vel.z + _pid_vel_z.get_error();

        _ekf_z_reset_ms = reset_ms;
    }
}

bool AC_PosControl::pre_arm_checks(const char *param_prefix,
                                   char *failure_msg,
                                   const uint8_t failure_msg_len)
{
    if (!is_positive(get_pos_xy_p().kP())) {
        hal.util->snprintf(failure_msg, failure_msg_len, "%s_POSXY_P must be > 0", param_prefix);
        return false;
    }
    if (!is_positive(get_pos_z_p().kP())) {
        hal.util->snprintf(failure_msg, failure_msg_len, "%s_POSZ_P must be > 0", param_prefix);
        return false;
    }
    if (!is_positive(get_vel_z_pid().kP())) {
        hal.util->snprintf(failure_msg, failure_msg_len, "%s_VELZ_P must be > 0", param_prefix);
        return false;
    }
    if (!is_positive(get_accel_z_pid().kP())) {
        hal.util->snprintf(failure_msg, failure_msg_len, "%s_ACCZ_P must be > 0", param_prefix);
        return false;
    }
    if (!is_positive(get_accel_z_pid().kI())) {
        hal.util->snprintf(failure_msg, failure_msg_len, "%s_ACCZ_I must be > 0", param_prefix);
        return false;
    }

    return true;
}
