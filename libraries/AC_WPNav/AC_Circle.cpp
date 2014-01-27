/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
#include <AP_HAL.h>
#include <AC_Circle.h>

extern const AP_HAL::HAL& hal;

const AP_Param::GroupInfo AC_Circle::var_info[] PROGMEM = {
    // @Param: RADIUS
    // @DisplayName: Circle Radius
    // @Description: Defines the radius of the circle the vehicle will fly when in Circle flight mode
    // @Units: cm
    // @Range: 0 10000
    // @Increment: 100
    // @User: Standard
    AP_GROUPINFO("RADIUS",  0,  AC_Circle, _radius, AC_CIRCLE_RADIUS_DEFAULT),

    // @Param: RATE
    // @DisplayName: Circle rate
    // @Description: Circle mode's turn rate in deg/sec.  Positive to turn clockwise, negative for counter clockwise
    // @Units: deg/s
    // @Range: -90 90
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("RATE",    1, AC_Circle, _rate,    AC_CIRCLE_RATE_DEFAULT),

    AP_GROUPEND
};

// Default constructor.
// Note that the Vector/Matrix constructors already implicitly zero
// their values.
//
AC_Circle::AC_Circle(const AP_InertialNav& inav, const AP_AHRS& ahrs, AC_PosControl& pos_control) :
    _inav(inav),
    _ahrs(ahrs),
    _pos_control(pos_control),
    _last_update(0),
    _step(0),
    _yaw(0),
    _angle(0)
{
    AP_Param::setup_object_defaults(this, var_info);
}

/// set_circle_center in cm from home
void AC_Circle::set_center(const Vector3f& position)
{
    _center = position;

	// set target position
	_pos_control.set_pos_target(_inav.get_position());

	// initialise pos controller speed and acceleration
	// To-Do: pull these from waypoint controller?
	_pos_control.set_speed_xy(500);
    _pos_control.set_accel_xy(100);
}

/// init_center in cm from home using stopping point and projecting out based on the copter's heading
void AC_Circle::init_center()
{
    Vector3f stopping_point;

    // get reasonable stopping point
    _pos_control.get_stopping_point_xy(stopping_point);
    _pos_control.get_stopping_point_z(stopping_point);

    // To-Do: replace this simple circle center calculation with projected position
    _center = stopping_point;
/*
    // set circle center to circle_radius ahead of stopping point
    _center.x = stopping_point.x + _radius * _cos_yaw;
    _center.y = stopping_point.y + _radius * _sin_yaw;
    _center.z = stopping_point.z;

    // if we are doing a panorama set the circle_angle to the current heading
    if (_radius <= 0) {
        _angle = heading_in_radians;
        _angular_velocity_max = ToRad(g.circle_rate);
        _angular_acceleration = _angular_velocity_max;  // reach maximum yaw velocity in 1 second
    }else{
        // set starting angle to current heading - 180 degrees
        circle_angle = wrap_PI(heading_in_radians-PI);

        // calculate max velocity based on waypoint speed ensuring we do not use more than half our max acceleration for accelerating towards the center of the circle
        max_velocity = min(wp_nav.get_horizontal_velocity(), safe_sqrt(0.5f*wp_nav.get_wp_acceleration()*g.circle_radius*100.0f));

        // angular_velocity in radians per second
        circle_angular_velocity_max = max_velocity/((float)g.circle_radius * 100.0f);
        circle_angular_velocity_max = constrain_float(ToRad(g.circle_rate),-circle_angular_velocity_max,circle_angular_velocity_max);

        // angular_velocity in radians per second
        circle_angular_acceleration = wp_nav.get_wp_acceleration()/((float)g.circle_radius * 100);
        if (g.circle_rate < 0.0f) {
            circle_angular_acceleration = -circle_angular_acceleration;
        }
    }

        // initialise other variables
        circle_angle_total = 0;
        circle_angular_velocity = 0;

        // initialise loiter target.  Note: feed forward velocity set to zero
        // To-Do: modify circle to use position controller and pass in zero velocity.  Vector3f(0,0,0)
        wp_nav.init_loiter_target();
    }
*/
}

/// update - update circle controller
void AC_Circle::update()
{
    // calculate dt
    uint32_t now = hal.scheduler->millis();
    float dt = (now - _last_update) / 1000.0f;

    // reset step back to 0 if 0.1 seconds has passed and we completed the last full cycle
    if (dt > 0.095f) {
        // double check dt is reasonable
        if (dt >= 1.0f) {
            dt = 0.0;
        }

        // To-Do: update the pos controller target

        // capture time since last iteration
        _last_update = now;
        // trigger position controller on next update
        _pos_control.trigger_xy();
    }else{
        // run loiter's position to velocity step
        _pos_control.update_pos_controller(true);
    }
}
