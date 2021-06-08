#pragma once

#include <AP_Common/AP_Common.h>
#include <APM_Control/AR_AttitudeControl.h>
#include <AC_PID/AC_P_2D.h>            // P library (2-axis)
#include <AC_PID/AC_PID_2D.h>          // PID library (2-axis)

class AR_PosControl {
public:

    // constructor
    AR_PosControl(AR_AttitudeControl& atc);

    // update navigation
    void update(float dt);

    // set limits
    void set_limits(float speed_max, float accel_max, float lat_accel_max);

    // get limits
    float get_speed_max() const { return _speed_max; }
    float get_accel_max() const { return _accel_max; }
    float get_lat_accel_max() const { return _lat_accel_max; }

    // set desired location
    bool set_desired_location(const Location &destination)  WARN_IF_UNUSED;

    // set position target
    void set_pos_target(const Vector2f &pos);

    // set position and velocity targets
    void set_pos_vel_target(const Vector2f &pos, const Vector2f &vel);

    // set position, velocity and acceleration targets
    void set_pos_vel_accel_target(const Vector2f &pos, const Vector2f &vel, const Vector2f &accel);

    // get outputs for forward-back speed (in m/s), lateral speed (in m/s) and turn rate (in rad/sec)
    float get_desired_speed() const { return _desired_speed; }
    float get_desired_turn_rate_rads() const { return _desired_turn_rate_rads; }
    float get_desired_lat_accel() const { return _desired_lat_accel; }
    //float get_desired_lateral_speed() const { return _desired_lateral_speed; }

    // return latest position error
    Vector2f get_pos_error() const;

    // returns desired velocity vector (i.e. feed forward) in cm/s in lat and lon direction
    Vector2f get_desired_velocity() const;

    // get desired lateral acceleration (for reporting purposes only because will be zero during pivot turns)

    // get pid controllers
    AC_P_2D& get_pos_p() { return _p_pos; }

    // parameter var table
    static const struct AP_Param::GroupInfo var_info[];

private:

    // true if update has been called recently
    bool is_active() const;

    // references
    AR_AttitudeControl &_atc;       // rover attitude control library

    // parameters
    AC_P_2D   _p_pos;               // position P controller to convert distance error to desired velocity
    AC_PID_2D _pid_vel;             // velocity PID controller to convert velocity error to desired acceleration

    // limits
    float _speed_max;               // maximum forward speed in m/s
    float _accel_max;               // maximum forward/back acceleration in m/s/s
    float _lat_accel_max;           // lateral acceleration maximum in m/s/s
    bool _has_lateral;              // true if vehicle is capable of lateral movement
    bool _limit_pos;                // true if the vehicle has hit a position limit
    Vector2f _limit_vel;            // To-Do: explain what this is

    // other limits (not yet implemented)
    float _turn_radius;             // vehicle turn radius in meters
    bool _pivot_possible;           // true if vehicle can pivot
    bool _pivot_active;             // true if vehicle is currently pivoting

    // position and velocity targets
    Vector2f _pos_target;           // position target as an offset (in meters) from the EKF origin
    Vector2f _vel_target;           // velocity target in m/s in NE frame
    Vector2f _accel_target;         // accel target in m/s/s in NE frame
    bool _pos_target_valid;         // true if _pos_target is valid
    bool _vel_target_valid;         // true if _vel_target is valid
    bool _accel_target_valid;       // true if _accel_target is valid

    // variables for navigation
    uint32_t _last_update_ms;       // system time of last call to update

    // main outputs
    float _desired_speed;           // desired forward_back speed in m/s
    float _desired_turn_rate_rads;  // desired turn-rate in rad/sec (negative is counter clockwise, positive is clockwise)
    float _desired_lat_accel;       // desired lateral acceleration (for reporting only)
    Vector2f _pos_error;            // latest position error
    //float _desired_lateral_speed;   // desired lateral speed in m/s
    //float _desired_heading_cd;      // desired heading (back towards line between origin and destination)
};
