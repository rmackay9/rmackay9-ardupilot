/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
#ifndef AC_CIRCLE_H
#define AC_CIRCLE_H

#include <AP_Common.h>
#include <AP_Param.h>
#include <AP_Math.h>
#include <AC_PID.h>             // PID library
#include <APM_PI.h>             // PID library
#include <AP_InertialNav.h>     // Inertial Navigation library
#include <AC_PosControl.h>      // Position control library

// loiter maximum velocities and accelerations
#define AC_CIRCLE_RADIUS_DEFAULT    1000.0f     // radius of the circle in cm that the vehicle will fly
#define AC_CIRCLE_RATE_DEFAULT      20.0f       // turn rate in deg/sec.  Positive to turn clockwise, negative for counter clockwise

class AC_Circle
{
public:

    /// Constructor
    AC_Circle(const AP_InertialNav& inav, const AP_AHRS& ahrs, AC_PosControl& pos_control);

    /// set_circle_center in cm from home
    void set_center(const Vector3f& position);

    /// init_center in cm from home using stopping point and projecting out based on the copter's heading
    void init_center();

    /// get_circle_center in cm from home
    const Vector3f& get_center() const { return _center; }

    /// set_radius - sets circle radius in cm
    void set_radius(float radius_cm) { _radius = radius_cm; };

    /// set_circle_rate - set circle rate in degrees per second
    void set_rate(float deg_per_sec) { _rate = deg_per_sec; }

    /// get_angle_total - return total angle in radians that vehicle has circled
    float get_angle_total() const { return _angle_total; }

    /// update - update circle controller
    void update();

    /// get desired roll, pitch which should be fed into stabilize controllers
    int32_t get_roll() const { return _pos_control.get_roll(); };
    int32_t get_pitch() const { return _pos_control.get_pitch(); };
    int32_t get_yaw() const { return _yaw; };

    /// set_cos_sin_yaw - short-cut to save on calculations to convert from roll-pitch frame to lat-lon frame
    void set_cos_sin_yaw(float cos_yaw, float sin_yaw) {
        _cos_yaw = cos_yaw;
        _sin_yaw = sin_yaw;
    }

    static const struct AP_Param::GroupInfo var_info[];

private:

    // calc_velocities - calculate angular velocity max and acceleration based on radius and rate
    //      this should be called whenever the radius or rate are changed
    //      initialises the yaw and current position around the circle
    void calc_velocities();

    // flags structure
    struct circle_flags {
        uint8_t panorama    : 1;    // true if we are doing a panorama
        uint8_t dir         : 1;    // 0 = clockwise, 1 = counter clockwise
    } _flags;

    // references to inertial nav and ahrs libraries
    const AP_InertialNav&       _inav;
    const AP_AHRS&              _ahrs;
    AC_PosControl&              _pos_control;

    // parameters
    AP_Float    _radius;        // maximum horizontal speed in cm/s during missions
    AP_Float    _rate;          // rotation speed in deg/sec

    // internal variables
    uint32_t    _last_update;   // time of last update_loiter call
    Vector3f    _center;        // center of circle in cm from home
    float       _yaw;           // yaw heading (normally towards circle center)
    float       _angle;         // current angular position around circle in radians (0=directly north of the center of the circle)
    float       _angle_total;   // total angle travelled in radians
    float       _angular_vel;   // angular velocity in radians/sec
    float       _angular_vel_max;   // maximum velocity in radians/sec
    float       _angular_accel; // angular acceleration in radians/sec/sec

    // helper variables
    float       _cos_yaw;       // short-cut to save on calcs required to convert roll-pitch frame to lat-lon frame
    float       _sin_yaw;       // To-Do: move these to ahrs or attitude control class to save on memory
};
#endif	// AC_CIRCLE_H
