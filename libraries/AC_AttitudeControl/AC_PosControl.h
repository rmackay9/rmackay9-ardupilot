/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
#ifndef AC_POSCONTROL_H
#define AC_POSCONTROL_H

#include <inttypes.h>
#include <AP_Common.h>
#include <AP_Param.h>
#include <AP_Math.h>
#include <AC_PID.h>             // PID library
#include <APM_PI.h>             // PID library
#include <AP_InertialNav.h>     // Inertial Navigation library
#include <AC_AttitudeControl.h> // Attitude control library
#include <AP_Motors.h>          // motors library


// loiter maximum velocities and accelerations
#define POSCONTROL_ACCELERATION                 100.0f  // defines the default velocity vs distant curve.  maximum acceleration in cm/s/s that position controller asks for from acceleration controller
#define POSCONTROL_ACCELERATION_MIN             50.0f   // minimum acceleration in cm/s/s - used for sanity checking _wp_accel parameter
#define POSCONTROL_ACCEL_XY_MAX                 980.0f  // max horizontal acceleration in cm/s/s that the position velocity controller will ask from the lower accel controller
#define POSCONTROL_STOPPING_DIST_Z_MAX          200.0f  // max stopping distance vertically   
                                                        // should be 1.5 times larger than POSCONTROL_ACCELERATION.
                                                        // max acceleration = max lean angle * 980 * pi / 180.  i.e. 23deg * 980 * 3.141 / 180 = 393 cm/s/s

#define POSCONTROL_SPEED                        500.0f  // maximum default loiter speed in cm/s
#define POSCONTROL_SPEED_UP                     250.0f  // default maximum climb velocity
#define POSCONTROL_SPEED_DOWN                   150.0f  // default maximum descent velocity
#define POSCONTROL_SPEED_MAX_TO_CORRECT_ERROR   200.0f  // maximum speed used to correct position error (i.e. not including feed forward)

#define POSCONTROL_LEAN_ANGLE_MAX               4500    // default maximum lean angle

#define POSCONTROL_ALT_HOLD_P                   1.0f    // default throttle controller's altitude hold's P gain.
#define POSCONTROL_ALT_HOLD_ACCEL_MAX           250.0f  // hard coded copy of throttle controller's maximum acceleration in cm/s.  To-Do: remove duplication with throttle controller definition

#define POSCONTROL_LEASH_LENGTH_MIN             100.0f  // minimum leash lengths in cm

#define POSCONTROL_DT_10HZ                      0.10f   // time difference in seconds for 10hz update rate

class AC_PosControl
{
public:

    /// Constructor
    AC_PosControl(const AP_AHRS& ahrs, const AP_InertialNav& inav,
                  const AP_Motors& motors, const AC_AttitudeControl& attitude_control,
                  APM_PI& pi_alt_pos, AC_PID& pid_alt_rate, AC_PID& pid_alt_accel,
                  APM_PI& pi_pos_lat, APM_PI& pi_pos_lon, AC_PID& pid_rate_lat, AC_PID& pid_rate_lon,
                  int16_t& motor_throttle);

    ///
    /// initialisation functions
    ///

    /// set_dt - sets time delta in seconds for all controllers (i.e. 100hz = 0.01, 400hz = 0.0025)
    void set_dt(float delta_sec) { _dt = delta_sec; }
    float get_dt() { return _dt; }

    ///
    /// z position controller
    ///

    /// get_stopping_point_z - returns reasonable stopping altitude in cm above home
    float get_stopping_point_z();

    /// fly_to_z - fly to altitude in cm above home
    void fly_to_z(const float alt_cm);

    /// climb_at_rate - climb at rate provided in cm/s
    void climb_at_rate(const float rate_cms);

/*
    ///
    /// xy position controller
    ///

    /// get_pos_target - get target as position vector (from home in cm)
    const Vector3f &get_pos_target() const { return _target; }

    /// set_pos_target in cm from home
    void set_pos_target(const Vector3f& position);

    /// init_pos_target - set initial loiter target based on current position and velocity
    void init_pos_target(const Vector3f& position, const Vector3f& velocity);

    /// get_distance_to_target - get horizontal distance to loiter target in cm
    float get_distance_to_target() const;

    /// get_bearing_to_target - get bearing to loiter target in centi-degrees
    int32_t get_bearing_to_target() const;

    /// update_loiter - run the loiter controller - should be called at 10hz
    void update_pos_controller();

    /// get_stopping_point - returns vector to stopping point based on a horizontal position and velocity
    void get_stopping_point(const Vector3f& position, const Vector3f& velocity, Vector3f &target) const;

    ///
    /// shared methods
    ///

    /// get desired roll, pitch which should be fed into stabilize controllers
    int32_t get_desired_roll() const { return _desired_roll; };
    int32_t get_desired_pitch() const { return _desired_pitch; };
*/
    /// get_desired_alt - get desired altitude (in cm above home) from loiter or wp controller which should be fed into throttle controller
    float get_desired_alt() const { return _target.z; }

    /// set_desired_alt - set desired altitude (in cm above home)
    void set_desired_alt(float desired_alt) { _target.z = desired_alt; }

    /// set_cos_sin_yaw - short-cut to save on calculations to convert from roll-pitch frame to lat-lon frame
    void set_cos_sin_yaw(float cos_yaw, float sin_yaw, float cos_pitch) {
        _cos_yaw = cos_yaw;
        _sin_yaw = sin_yaw;
        _cos_pitch = cos_pitch;
    }

/*
    /// set_horizontal_velocity - allows main code to pass target horizontal velocity for wp navigation
    void set_horizontal_velocity(float velocity_cms) { _wp_speed_cms = velocity_cms; };

    /// get_horizontal_velocity - allows main code to retrieve target horizontal velocity for wp navigation
    float get_horizontal_velocity() { return _wp_speed_cms; };

    /// get_climb_velocity - returns target climb speed in cm/s during missions
    float get_climb_velocity() const { return _wp_speed_up_cms; };

    /// get_descent_velocity - returns target descent speed in cm/s during missions.  Note: always positive
    float get_descent_velocity() const { return _wp_speed_down_cms; };

    /// get_waypoint_radius - access for waypoint radius in cm
    float get_waypoint_radius() const { return _wp_radius_cm; }

    /// get_waypoint_acceleration - returns acceleration in cm/s/s during missions
    float get_waypoint_acceleration() const { return _wp_accel_cms.get(); }

    /// set_lean_angle_max - limits maximum lean angle
    void set_lean_angle_max(int16_t angle_cd) { if (angle_cd >= 1000 && angle_cd <= 8000) {_lean_angle_max_cd = angle_cd;} }

    static const struct AP_Param::GroupInfo var_info[];

protected:
    // flags structure
    struct poscontroller_flags {
        uint8_t dummy     : 1;    // dummy flag
    } _flags;

    /// get_loiter_position_to_velocity - loiter position controller
    ///     converts desired position held in _target vector to desired velocity
    void get_position_to_velocity(float dt, float max_speed_cms);

    /// get_loiter_velocity_to_acceleration - loiter velocity controller
    ///    converts desired velocities in lat/lon directions to accelerations in lat/lon frame
    void get_velocity_to_acceleration(float vel_lat_cms, float vel_lon_cms, float dt);

    /// get_loiter_acceleration_to_lean_angles - loiter acceleration controller
    ///    converts desired accelerations provided in lat/lon frame to roll/pitch angles
    void get_acceleration_to_lean_angles(float accel_lat_cmss, float accel_lon_cmss);

    /// get_bearing_cd - return bearing in centi-degrees between two positions
    float get_bearing_cd(const Vector3f &origin, const Vector3f &destination) const;

    /// reset_I - clears I terms from position PID controller
    void reset_I();

    /// calculate_leash_length - calculates the maximum distance in cm that the target position may be from the current location
    void calculate_leash_length();

    // references to inertial nav and ahrs libraries
    const AP_InertialNav&       _inav;
    const AP_AHRS&              _ahrs;
    const AP_Motors&            _motors;
    const AC_AttitudeControl&   _attitude_control;

    // references to pid controllers and motors
    APM_PI&     _pi_alt_pos;
    AC_PID&     _pid_alt_rate;
    AC_PID&     _pid_alt_accel;
    APM_PI&	    _pid_pos_lat;
    APM_PI&	    _pid_pos_lon;
    AC_PID&	    _pid_rate_lat;
    AC_PID&	    _pid_rate_lon;
    int16_t&    _motor_throttle;

    // internal variables
    float       _dt;                    // time difference (in seconds) between calls from the main program
    uint32_t    _last_update_ms;        // system time of last update_position_controller call
    uint32_t    _last_update_rate_ms;   // system time of last call to rate_to_accel_z (alt hold accel controller)
    uint32_t    _last_update_accel_ms;  // system time of last call to accel_to_motor_throttle (alt hold accel controller)
    uint8_t     _step;                  // used to decide which portion of position controller to run during this iteration
    float       _speed_cms;             // max horizontal speed in cm/s
    float       _speed_up_cms;          // max climb rate in cm/s
    float       _speed_down_cms;        // max descent rate in cm/s
    float       _accel_cms;             // max horizontal acceleration in cm/s/s
    float       _cos_yaw;               // short-cut to save on calcs required to convert roll-pitch frame to lat-lon frame
    float       _sin_yaw;
    float       _cos_pitch;

    // output from controller
    float       _desired_roll;          // desired roll angle in centi-degrees calculated by position controller
    float       _desired_pitch;         // desired roll pitch in centi-degrees calculated by position controller

    // position controller internal variables
    Vector3f    _pos_target;            // target location in cm from home
    Vector3f    _pos_error;             // error between desired and actual position in cm
    Vector3f    _vel_target;            // desired velocity in cm/s
    Vector3f    _vel_error;             // error between desired and actual acceleration in cm/s.  To-Do: x & y actually used?
    Vector3f    _vel_last;              // previous iterations velocity in cm/s
    float       _vel_target_filt_z;     // filtered target vertical velocity
    Vector3f    _accel_target;          // desired acceleration in cm/s/s  // To-Do: are xy actually required?
    Vector3f    _accel_error;           // desired acceleration in cm/s/s  // To-Do: are xy actually required?
    float       _leash;                 // horizontal leash length in cm.  used to stop the pilot from pushing the target location too far from the current location
    int16_t     _lean_angle_max_cd;     // maximum lean angle in centi-degrees

public:
    // for logging purposes
    Vector2f dist_error;                // distance error calculated by loiter controller
    Vector2f desired_vel;               // loiter controller desired velocity
    Vector2f desired_accel;             // the resulting desired acceleration
};
#endif	// AC_POSCONTROL_H
