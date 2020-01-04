#include <AP_HAL/AP_HAL.h>
#include "AC_WPNav.h"

extern const AP_HAL::HAL& hal;

const AP_Param::GroupInfo AC_WPNav::var_info[] = {
    // index 0 was used for the old orientation matrix

    // @Param: SPEED
    // @DisplayName: Waypoint Horizontal Speed Target
    // @Description: Defines the speed in cm/s which the aircraft will attempt to maintain horizontally during a WP mission
    // @Units: cm/s
    // @Range: 20 2000
    // @Increment: 50
    // @User: Standard
    AP_GROUPINFO("SPEED",       0, AC_WPNav, _wp_speed_cms, WPNAV_WP_SPEED),

    // @Param: RADIUS
    // @DisplayName: Waypoint Radius
    // @Description: Defines the distance from a waypoint, that when crossed indicates the wp has been hit.
    // @Units: cm
    // @Range: 5 1000
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("RADIUS",      1, AC_WPNav, _wp_radius_cm, WPNAV_WP_RADIUS),

    // @Param: SPEED_UP
    // @DisplayName: Waypoint Climb Speed Target
    // @Description: Defines the speed in cm/s which the aircraft will attempt to maintain while climbing during a WP mission
    // @Units: cm/s
    // @Range: 10 1000
    // @Increment: 50
    // @User: Standard
    AP_GROUPINFO("SPEED_UP",    2, AC_WPNav, _wp_speed_up_cms, WPNAV_WP_SPEED_UP),

    // @Param: SPEED_DN
    // @DisplayName: Waypoint Descent Speed Target
    // @Description: Defines the speed in cm/s which the aircraft will attempt to maintain while descending during a WP mission
    // @Units: cm/s
    // @Range: 10 500
    // @Increment: 10
    // @User: Standard
    AP_GROUPINFO("SPEED_DN",    3, AC_WPNav, _wp_speed_down_cms, WPNAV_WP_SPEED_DOWN),

    // @Param: ACCEL
    // @DisplayName: Waypoint Acceleration 
    // @Description: Defines the horizontal acceleration in cm/s/s used during missions
    // @Units: cm/s/s
    // @Range: 50 500
    // @Increment: 10
    // @User: Standard
    AP_GROUPINFO("ACCEL",       5, AC_WPNav, _wp_accel_cmss, WPNAV_ACCELERATION),

    // @Param: ACCEL_Z
    // @DisplayName: Waypoint Vertical Acceleration
    // @Description: Defines the vertical acceleration in cm/s/s used during missions
    // @Units: cm/s/s
    // @Range: 50 500
    // @Increment: 10
    // @User: Standard
    AP_GROUPINFO("ACCEL_Z",     6, AC_WPNav, _wp_accel_z_cmss, WPNAV_WP_ACCEL_Z_DEFAULT),

    // @Param: RFND_USE
    // @DisplayName: Waypoint missions use rangefinder for terrain following
    // @Description: This controls if waypoint missions use rangefinder for terrain following
    // @Values: 0:Disable,1:Enable
    // @User: Advanced
    AP_GROUPINFO("RFND_USE",   10, AC_WPNav, _rangefinder_use, 1),

    // @Param: JERK
    // @DisplayName: Waypoint Jerk
    // @Description: Defines the horizontal jerk in m/s/s used during missions
    // @Units: m/s/s
    // @Range: 1 20
    // @User: Standard
    AP_GROUPINFO("JERK",   11, AC_WPNav, _wp_jerk, 1),

    AP_GROUPEND
};

// Default constructor.
// Note that the Vector/Matrix constructors already implicitly zero
// their values.
//
AC_WPNav::AC_WPNav(const AP_InertialNav& inav, const AP_AHRS_View& ahrs, AC_PosControl& pos_control, const AC_AttitudeControl& attitude_control) :
    _inav(inav),
    _ahrs(ahrs),
    _pos_control(pos_control),
    _attitude_control(attitude_control)
{
    AP_Param::setup_object_defaults(this, var_info);

    // init flags
    _flags.reached_destination = false;
    _flags.fast_waypoint = false;
    _flags.segment_type = SEGMENT_STRAIGHT;

    // sanity check some parameters
    _wp_accel_cmss = MIN(_wp_accel_cmss, GRAVITY_MSS * 100.0f * tanf(ToRad(_attitude_control.lean_angle_max() * 0.01f)));
    _wp_radius_cm = MAX(_wp_radius_cm, WPNAV_WP_RADIUS_MIN);
}

// get expected source of terrain data if alt-above-terrain command is executed (used by Copter's ModeRTL)
AC_WPNav::TerrainSource AC_WPNav::get_terrain_source() const
{
    // use range finder if connected
    if (_rangefinder_available && _rangefinder_use) {
        return AC_WPNav::TerrainSource::TERRAIN_FROM_RANGEFINDER;
    }
#if AP_TERRAIN_AVAILABLE
    if ((_terrain != nullptr) && _terrain->enabled()) {
        return AC_WPNav::TerrainSource::TERRAIN_FROM_TERRAINDATABASE;
    } else {
        return AC_WPNav::TerrainSource::TERRAIN_UNAVAILABLE;
    }
#else
    return AC_WPNav::TerrainSource::TERRAIN_UNAVAILABLE;
#endif
}

///
/// waypoint navigation
///

/// wp_and_spline_init - initialise straight line and spline waypoint controllers
///     speed_cms should be set to a positive value or left at zero to use the default speed
///     updates target roll, pitch targets and I terms based on vehicle lean angles
///     should be called once before the waypoint controller is used but does not need to be called before subsequent updates to destination
void AC_WPNav::wp_and_spline_init(float speed_cms)
{
    // check _wp_accel_cmss is reasonable
    if (_wp_accel_cmss <= 0) {
        _wp_accel_cmss.set_and_save(WPNAV_ACCELERATION);
    }

    // initialise position controller
    _pos_control.set_desired_accel_xy(0.0f,0.0f);
    _pos_control.init_xy_controller();

    // initialise feed forward velocity to zero
    _pos_control.set_desired_velocity_xy(0.0f, 0.0f);

    // initialize the desired wp speed if not already done
    _wp_desired_speed_xy_cms = is_positive(speed_cms) ? speed_cms : _wp_speed_cms;

    // initialise position controller speed and acceleration
    _pos_control.set_max_speed_xy(_wp_desired_speed_xy_cms);
    _pos_control.set_max_accel_xy(_wp_accel_cmss);
    _pos_control.set_max_speed_z(-_wp_speed_down_cms, _wp_speed_up_cms);
    _pos_control.set_max_accel_z(_wp_accel_z_cmss);
    _pos_control.calc_leash_length_xy();
    _pos_control.calc_leash_length_z();

    float jerk = MIN(_attitude_control.get_ang_vel_roll_max() * GRAVITY_MSS, _attitude_control.get_ang_vel_pitch_max() * GRAVITY_MSS);
    if (is_zero(jerk)) {
        jerk = _wp_jerk;
    } else {
        jerk = MIN(jerk, _wp_jerk);
    }
    float jounce = MIN(_attitude_control.get_accel_roll_max() * GRAVITY_MSS, _attitude_control.get_accel_pitch_max() * GRAVITY_MSS);

    // time constant may also be a useful parameter
    float tc;
    if (is_positive(jounce)) {
        tc = MAX(_attitude_control.get_input_tc(), 0.5 * jerk * M_PI / jounce);
    } else {
        tc = MAX(_attitude_control.get_input_tc(), 0.1f);
    }

    _scurve_prev_leg = scurves(tc * 2.0, jerk * 100.0, _wp_accel_cmss, _wp_desired_speed_xy_cms);
    _scurve_this_leg = scurves(tc * 2.0, jerk * 100.0, _wp_accel_cmss, _wp_desired_speed_xy_cms);
    _scurve_next_leg = scurves(tc * 2.0, jerk * 100.0, _wp_accel_cmss, _wp_desired_speed_xy_cms);
    _scurve_prev_leg.init();
    _scurve_this_leg.init();

    // set flag so get_yaw() returns current heading target
    _flags.wp_yaw_set = false;
    _flags.reached_destination = false;
}

/// set_speed_xy - allows main code to pass target horizontal velocity for wp navigation
void AC_WPNav::set_speed_xy(float speed_cms)
{
    // range check target speed
    if (speed_cms >= WPNAV_WP_SPEED_MIN) {
        _wp_desired_speed_xy_cms = speed_cms;
    }
}

/// set current target climb rate during wp navigation
void AC_WPNav::set_speed_up(float speed_up_cms)
{
    _pos_control.set_max_speed_z(_pos_control.get_max_speed_down(), speed_up_cms);
}

/// set current target descent rate during wp navigation
void AC_WPNav::set_speed_down(float speed_down_cms)
{
    _pos_control.set_max_speed_z(speed_down_cms, _pos_control.get_max_speed_up());
}

/// set_wp_destination waypoint using location class
///     returns false if conversion from location to vector from ekf origin cannot be calculated
bool AC_WPNav::set_wp_destination_loc(const Location& destination)
{
    bool terr_alt;
    Vector3f dest_neu;

    // convert destination location to vector
    if (!get_vector_NEU(destination, dest_neu, terr_alt)) {
        return false;
    }

    // set target as vector from EKF origin
    return set_wp_destination(dest_neu, terr_alt);
}

/// set next destination using location class
///     returns false if conversion from location to vector from ekf origin cannot be calculated
bool AC_WPNav::set_wp_destination_loc_next(const Location& destination)
{
    bool terr_alt;
    Vector3f dest_neu;

    // convert destination location to vector
    if (!get_vector_NEU(destination, dest_neu, terr_alt)) {
        return false;
    }

    // set target as vector from EKF origin
    return set_wp_destination_next(dest_neu, terr_alt);
}

bool AC_WPNav::get_wp_destination_loc(Location& destination) const
{
    Vector3f dest = get_wp_destination();
    if (!AP::ahrs().get_origin(destination)) {
        return false;
    }
    destination.offset(dest.x*0.01f, dest.y*0.01f);
    destination.alt += dest.z;
    return true;
}

/// set_wp_destination - set destination waypoints using position vectors (distance from ekf origin in cm)
///     terrain_alt should be true if origin.z and destination.z are desired altitudes above terrain (false if these are alt-above-ekf-origin)
///     returns false on failure (likely caused by missing terrain data)
bool AC_WPNav::set_wp_destination(const Vector3f& destination, bool terrain_alt)
{
    if (is_active() && _flags.reached_destination && (terrain_alt == _terrain_alt)) {
        // use previous destination as origin
        _origin = _destination;

        // store previous leg
        _scurve_prev_leg = _scurve_this_leg;
    } else {

        // use stopping point as the origin
        get_wp_stopping_point(_origin);

        // convert origin to alt-above-terrain if necessary
        if (terrain_alt) {
            float origin_terr_offset;
            if (!get_terrain_offset(origin_terr_offset)) {
                return false;
            }
            _origin.z -= origin_terr_offset;
        }

        _scurve_prev_leg.init();
    }

    // update destination
    _destination = destination;
    _terrain_alt = terrain_alt;

    set_kinematic_limits(_scurve_this_leg, _origin, _destination);
    _scurve_this_leg.calculate_straight_leg(_origin, _destination);
    _scurve_next_leg.init();
    _flags.fast_waypoint = false;   // default waypoint back to slow
    _flags.reached_destination = false;
    _flags.wp_yaw_set = false;

    return true;
}

/// set next destination using position vector (distance from ekf origin in cm)
///     terrain_alt should be true if destination.z is a desired altitude above terrain
///     provide next_destination
bool AC_WPNav::set_wp_destination_next(const Vector3f& destination, bool terrain_alt)
{
    // ToDo: improve handling of mismatching terrain alt
    if (terrain_alt != _terrain_alt) {
        return false;
    }

    set_kinematic_limits(_scurve_next_leg, _destination, destination);
    _scurve_next_leg.calculate_straight_leg(_destination, destination);

    // next destination provided so fast waypoint
    _flags.fast_waypoint = true;

    return true;
}

/// set waypoint destination using NED position vector from ekf origin in meters
bool AC_WPNav::set_wp_destination_NED(const Vector3f& destination_NED)
{
    // convert NED to NEU and do not use terrain following
    return set_wp_destination(Vector3f(destination_NED.x * 100.0f, destination_NED.y * 100.0f, -destination_NED.z * 100.0f), false);
}

/// set waypoint destination using NED position vector from ekf origin in meters
bool AC_WPNav::set_wp_destination_NED_next(const Vector3f& destination_NED)
{
    // convert NED to NEU and do not use terrain following
    return set_wp_destination_next(Vector3f(destination_NED.x * 100.0f, destination_NED.y * 100.0f, -destination_NED.z * 100.0f), false);
}

/// shift_wp_origin_to_current_pos - shifts the origin and destination so the origin starts at the current position
///     used to reset the position just before takeoff
///     relies on set_wp_destination or set_wp_origin_and_destination having been called first
void AC_WPNav::shift_wp_origin_to_current_pos()
{
    // return immediately if vehicle is not at the origin
    if (_track_desired > 0.0f) {
        return;
    }

    // get current and target locations
    const Vector3f &curr_pos = _inav.get_position();
    const Vector3f pos_target = _pos_control.get_pos_target();

    // calculate difference between current position and target
    Vector3f pos_diff = curr_pos - pos_target;

    // shift origin and destination
    _origin += pos_diff;
    _destination += pos_diff;

    // move pos controller target and disable feed forward
    _pos_control.set_pos_target(curr_pos);
}

/// shifts the origin and destination horizontally to the current position
///     used to reset the track when taking off without horizontal position control
///     relies on set_wp_destination or set_wp_origin_and_destination having been called first
void AC_WPNav::shift_wp_origin_and_destination_to_current_pos_xy()
{
    // get current and target locations
    const Vector3f& curr_pos = _inav.get_position();

    // shift origin and destination horizontally
    _origin.x = curr_pos.x;
    _origin.y = curr_pos.y;
    _destination.x = curr_pos.x;
    _destination.y = curr_pos.y;

    // move pos controller target horizontally
    _pos_control.set_xy_target(curr_pos.x, curr_pos.y);
}

/// shifts the origin and destination horizontally to the achievable stopping point
///     used to reset the track when horizontal navigation is enabled after having been disabled (see Copter's wp_navalt_min)
///     relies on set_wp_destination or set_wp_origin_and_destination having been called first
void AC_WPNav::shift_wp_origin_and_destination_to_stopping_point_xy()
{
    // relax position control in xy axis
    // removing velocity error also impacts stopping point calculation
    _pos_control.relax_velocity_controller_xy();

    // get current and target locations
    Vector3f stopping_point;
    get_wp_stopping_point_xy(stopping_point);

    // shift origin and destination horizontally
    _origin.x = stopping_point.x;
    _origin.y = stopping_point.y;
    _destination.x = stopping_point.x;
    _destination.y = stopping_point.y;

    // move pos controller target horizontally
    _pos_control.set_xy_target(stopping_point.x, stopping_point.y);
}

/// get_wp_stopping_point_xy - returns vector to stopping point based on a horizontal position and velocity
void AC_WPNav::get_wp_stopping_point_xy(Vector3f& stopping_point) const
{
	_pos_control.get_stopping_point_xy(stopping_point);
}

/// get_wp_stopping_point - returns vector to stopping point based on 3D position and velocity
void AC_WPNav::get_wp_stopping_point(Vector3f& stopping_point) const
{
    _pos_control.get_stopping_point_xy(stopping_point);
    _pos_control.get_stopping_point_z(stopping_point);
}

/// advance_wp_target_along_track - move target location along track from origin to destination
bool AC_WPNav::advance_wp_target_along_track(float dt)
{
    // calculate terrain adjustments
    float terr_offset = 0.0f;
    if (_terrain_alt && !get_terrain_offset(terr_offset)) {
        return false;
    }

    // get current position and adjust altitude to origin and destination's frame (i.e. _frame)
    const Vector3f &curr_pos = _inav.get_position() - Vector3f(0, 0, terr_offset);

    // ToDo: adjust time to prevent target moving too far in front of aircraft
    _track_scaler_dt = 1.0f;

    // generate current position, velocity and acceleration
    Vector3f target_pos, target_vel, target_accel;
    target_pos = _origin;
    _scurve_prev_leg.move_to_pos_vel_accel(dt, _track_scaler_dt, target_pos, target_vel, target_accel);
    bool s_finish = _scurve_this_leg.move_from_pos_vel_accel(dt, _track_scaler_dt, target_pos, target_vel, target_accel);

    // add input shaped offset
    //update_targets_with_offset(origin_alt_offset, target_pos, target_vel, target_accel, dt);

    // convert final_target.z to altitude above the ekf origin
    target_pos.z += terr_offset;

    // pass new target to the position controller
    _pos_control.set_pos_vel_accel(target_pos, target_vel, target_accel);

    // Check for change of leg on fast waypoint
    if (_flags.fast_waypoint && _scurve_this_leg.braking()) {
        float time_to_destination = _scurve_this_leg.get_time_remaining();
        Vector3f turn_pos, turn_vel, turn_accel;
        turn_pos = -_scurve_this_leg.get_track();
        _scurve_this_leg.move_from_time_pos_vel_accel(_scurve_this_leg.get_time_elapsed() + time_to_destination/2.0, 1.0, turn_pos, turn_vel, turn_accel);
        _scurve_next_leg.move_from_time_pos_vel_accel(time_to_destination/2.0, _track_scaler_dt, turn_pos, turn_vel, turn_accel);
        const float vel_min = MIN(_scurve_this_leg.get_vel_max(), _scurve_next_leg.get_vel_max());
        const float accel_min = MIN(_scurve_this_leg.get_accel_max(), _scurve_next_leg.get_accel_max());
        s_finish = s_finish || ((_scurve_this_leg.get_time_remaining() < _scurve_next_leg.time_end()/2.0) && (turn_pos.length() < _wp_radius_cm) && (Vector2f(turn_vel.x, turn_vel.y).length() < vel_min) && (Vector2f(turn_accel.x, turn_accel.y).length() < 2*accel_min));
    }

    // check if we've reached the waypoint
    if (!_flags.reached_destination) {
        if (s_finish) {
            // "fast" waypoints are complete once the intermediate point reaches the destination
            if (_flags.fast_waypoint) {
                _flags.reached_destination = true;
            } else {
                // regular waypoints also require the copter to be within the waypoint radius
                Vector3f dist_to_dest = curr_pos - _destination;
                if (dist_to_dest.length() <= _wp_radius_cm) {
                    _flags.reached_destination = true;
                }
            }
        }
    }

    // Calculate the turn rate
    // todo: consider doing this just in the xy
    float turn_rate, accel_forward;
    Vector3f accel_turn;
    if (is_positive(target_vel.length())) {
        accel_forward = (target_accel.x * target_vel.x + target_accel.y * target_vel.y + target_accel.z * target_vel.z)/target_vel.length();
        accel_turn = target_accel - target_vel * accel_forward / target_vel.length();
        turn_rate = accel_turn.length() / (_track_scaler_dt*target_vel.length());
        if (accel_turn.y * target_vel.x - accel_turn.x * target_vel.y < 0.0) {
            turn_rate *= -1.0f;
        }
    } else {
        turn_rate = 0.0f;
    }

    // update the target yaw if origin and destination are at least 2m apart horizontally
    const Vector2f target_vel_xy(target_vel.x, target_vel.y);
    if ((_scurve_this_leg.pos_end() >= WPNAV_YAW_DIST_MIN) && (target_vel_xy.length() > WPNAV_YAW_VEL_MIN)) {
        // todo: add feed forward yaw for coordinated turns
        set_yaw_cd(degrees(target_vel_xy.angle()) * 100.0f);
        set_yaw_rate_cds(turn_rate*degrees(100.0f));
    }

    // successfully advanced along track
    return true;
}

/// get_wp_distance_to_destination - get horizontal distance to destination in cm
float AC_WPNav::get_wp_distance_to_destination() const
{
    // get current location
    const Vector3f &curr = _inav.get_position();
    return norm(_destination.x-curr.x,_destination.y-curr.y);
}

/// get_wp_bearing_to_destination - get bearing to next waypoint in centi-degrees
int32_t AC_WPNav::get_wp_bearing_to_destination() const
{
    return get_bearing_cd(_inav.get_position(), _destination);
}

/// update_wpnav - run the wp controller - should be called at 100hz or higher
bool AC_WPNav::update_wpnav()
{
    bool ret = true;

    // get dt from pos controller
    float dt = _pos_control.get_dt();

    // allow the accel and speed values to be set without changing
    // out of auto mode. This makes it easier to tune auto flight
    _pos_control.set_max_accel_xy(_wp_accel_cmss);
    _pos_control.set_max_accel_z(_wp_accel_z_cmss);

    // wp_speed_update - update _pos_control.set_max_speed_xy if speed change has been requested
    wp_speed_update(dt);

    // advance the target if necessary
    if (!advance_wp_target_along_track(dt)) {
        // To-Do: handle inability to advance along track (probably because of missing terrain data)
        ret = false;
    }

    _pos_control.update_xy_controller();

    _wp_last_update = AP_HAL::millis();

    return ret;
}

// returns true if update_wpnav has been run very recently
bool AC_WPNav::is_active() const
{
    return (AP_HAL::millis() - _wp_last_update) < 200;
}

// returns target yaw in centi-degrees (used for wp and spline navigation)
float AC_WPNav::get_yaw() const
{
    if (_flags.wp_yaw_set) {
        return _yaw;
    } else {
        // if yaw has not been set return attitude controller's current target
        return _attitude_control.get_att_target_euler_cd().z;
    }
}

// returns target yaw in centi-degrees (used for wp and spline navigation)
float AC_WPNav::get_yaw_rate() const
{
    if (_flags.wp_yaw_set) {
        return _yaw_rate;
    } else {
        // if yaw has not been set return attitude controller's current target
        return 0.0f;
    }
}

// set heading used for spline and waypoint navigation
void AC_WPNav::set_yaw_cd(float heading_cd)
{
    _yaw = heading_cd;
    _flags.wp_yaw_set = true;
}

// set yaw rate used for spline and waypoint navigation
void AC_WPNav::set_yaw_rate_cds(float yaw_rate_cds)
{
    _yaw_rate = yaw_rate_cds;
}

// get terrain's altitude (in cm above the ekf origin) at the current position (+ve means terrain below vehicle is above ekf origin's altitude)
bool AC_WPNav::get_terrain_offset(float& offset_cm)
{
    // calculate offset based on source (rangefinder or terrain database)
    switch (get_terrain_source()) {
    case AC_WPNav::TerrainSource::TERRAIN_UNAVAILABLE:
        return false;
    case AC_WPNav::TerrainSource::TERRAIN_FROM_RANGEFINDER:
        if (_rangefinder_healthy) {
            offset_cm = _inav.get_altitude() - _rangefinder_alt_cm;
            return true;
        }
        return false;
    case AC_WPNav::TerrainSource::TERRAIN_FROM_TERRAINDATABASE:
#if AP_TERRAIN_AVAILABLE
        float terr_alt = 0.0f;
        if (_terrain != nullptr && _terrain->height_above_terrain(terr_alt, true)) {
            offset_cm = _inav.get_altitude() - (terr_alt * 100.0f);
            return true;
        }
#endif
        return false;
    }

    // we should never get here but just in case
    return false;
}

///
/// spline methods
///

/// set_spline_destination waypoint using location class
///     returns false if conversion from location to vector from ekf origin cannot be calculated
///     next_destination should be set to the next segment's destination
///     next_is_spline should be true if path to next_destination should be a spline
bool AC_WPNav::set_spline_destination_loc(const Location& destination, Location next_destination, bool next_is_spline)
{
    // convert destination location to vector
    Vector3f dest_neu;
    bool dest_terr_alt;
    if (!get_vector_NEU(destination, dest_neu, dest_terr_alt)) {
        return false;
    }

    // convert next destination to vector
    Vector3f next_dest_neu;
    bool next_dest_terr_alt;
    if (!get_vector_NEU(next_destination, next_dest_neu, next_dest_terr_alt)) {
        return false;
    }

    // set target as vector from EKF origin
    return set_spline_destination(dest_neu, dest_terr_alt, next_dest_neu, next_dest_terr_alt, next_is_spline);
}

/// set_spline_destination waypoint using location class
///     returns false if conversion from location to vector from ekf origin cannot be calculated
///     next_destination should be set to the next segment's destination
bool AC_WPNav::set_spline_destination_next_loc(const Location& destination, Location next_destination, bool next_is_spline)
{
    // convert destination location to vector
    Vector3f dest_neu;
    bool dest_terr_alt;
    // convert destination location to vector
    if (!get_vector_NEU(destination, dest_neu, dest_terr_alt)) {
        return false;
    }

    // convert next destination to vector
    Vector3f next_dest_neu;
    bool next_dest_terr_alt;
    if (!get_vector_NEU(next_destination, next_dest_neu, next_dest_terr_alt)) {
        return false;
    }

    // set target as vector from EKF origin
    return set_spline_destination_next(dest_neu, dest_terr_alt, next_dest_neu, next_dest_terr_alt, next_is_spline);
}

/// set_spline_destination waypoint using position vector (distance from ekf origin in cm)
///     terrain_alt should be true if destination.z is a desired altitude above terrain (false if its desired altitudes above ekf origin)
///     next_destination should be set to the next segment's destination
///     next_terrain_alt should be true if next_destination.z is a desired altitude above terrain (false if its desired altitudes above ekf origin)
///     next_destination.z  must be in the same "frame" as destination.z (i.e. if destination is a alt-above-terrain, next_destination should be too)
bool AC_WPNav::set_spline_destination(const Vector3f& destination, bool terrain_alt, const Vector3f& next_destination, bool next_terrain_alt, bool next_is_spline)
{
    if (is_active() && _flags.reached_destination && (terrain_alt == _terrain_alt)) {
        // use previous destination as origin
        _origin = _destination;

        // store previous leg
        _scurve_prev_leg = _scurve_this_leg;
    } else {
        // use stopping point as the origin
        get_wp_stopping_point(_origin);

        // convert origin to alt-above-terrain if necessary
        if (terrain_alt) {
            float origin_terr_offset;
            if (!get_terrain_offset(origin_terr_offset)) {
                return false;
            }
            _origin.z -= origin_terr_offset;
        }

        _scurve_prev_leg.init();

        // make sure we do not use next leg below
        _flags.fast_waypoint = false;
    }

    // store destination location
    _destination = destination;
    _terrain_alt = terrain_alt;

    if (_flags.fast_waypoint) {
        // if we have next leg then store it as this leg
        // ToDo: ensure the destination has not changed
        _scurve_this_leg = _scurve_next_leg;
    } else {
        // vehicle has stopped so origin vector is simply vector from origin to destination
        Vector3f origin_vector = _destination - _origin;

        Vector3f destination_vector;
        if (terrain_alt != next_terrain_alt) {
            // change in terrain alt type so leave destination vector as zero
            origin_vector.zero();
        } else if (next_is_spline) {
            // leave this segment moving parallel to vector from origin to next destination
            destination_vector = next_destination - _origin;
        } else {
            // leave this segment moving parallel to next segment
            destination_vector = next_destination - _destination;
        }
        set_kinematic_limits(_scurve_this_leg, _origin, _destination);
        _scurve_this_leg.calculate_spline_leg(_origin, _destination, origin_vector, destination_vector);
    }

    _scurve_next_leg.init();
    _flags.fast_waypoint = false;   // default waypoint back to slow
    _flags.reached_destination = false;
    _flags.wp_yaw_set = false;

    return true;
}

/// set_spline_destination_next waypoint using position vector (distance from ekf origin in cm)
///     terrain_alt should be true if destination.z is a desired altitude above terrain (false if its desired altitudes above ekf origin)
///     next_destination should be set to the next segment's destination
///     next_terrain_alt should be true if next_destination.z is a desired altitude above terrain (false if its desired altitudes above ekf origin)
///     next_destination.z  must be in the same "frame" as destination.z (i.e. if destination is a alt-above-terrain, next_destination should be too)
bool AC_WPNav::set_spline_destination_next(const Vector3f& destination, bool terrain_alt, const Vector3f& next_destination, bool next_terrain_alt, bool next_is_spline)
{
    // only use next waypoint if terrain_alt matches
    if (terrain_alt != _terrain_alt) {
        return false;
    }

    Vector3f origin_vector, destination_vector;
    if (_scurve_this_leg.is_straight()) {
        origin_vector = _destination - _origin;
    } else {
        origin_vector = destination - _origin;
    }
    if (terrain_alt != next_terrain_alt) {
        // change in terrain alt type so leave destination vector as zero
    } else if (next_is_spline) {
        destination_vector = next_destination - _destination;
    } else {
        destination_vector = next_destination - destination;
    }

    set_kinematic_limits(_scurve_this_leg, _destination, destination);
    _scurve_next_leg.calculate_spline_leg(_destination, destination, origin_vector, destination_vector);

    // next destination provided so fast waypoint
    _flags.fast_waypoint = true;

    return true;
}

// convert location to vector from ekf origin.  terrain_alt is set to true if resulting vector's z-axis should be treated as alt-above-terrain
//      returns false if conversion failed (likely because terrain data was not available)
bool AC_WPNav::get_vector_NEU(const Location &loc, Vector3f &vec, bool &terrain_alt)
{
    // convert location to NE vector2f
    Vector2f res_vec;
    if (!loc.get_vector_xy_from_origin_NE(res_vec)) {
        return false;
    }

    // convert altitude
    if (loc.get_alt_frame() == Location::AltFrame::ABOVE_TERRAIN) {
        int32_t terr_alt;
        if (!loc.get_alt_cm(Location::AltFrame::ABOVE_TERRAIN, terr_alt)) {
            return false;
        }
        vec.z = terr_alt;
        terrain_alt = true;
    } else {
        terrain_alt = false;
        int32_t temp_alt;
        if (!loc.get_alt_cm(Location::AltFrame::ABOVE_ORIGIN, temp_alt)) {
            return false;
        }
        vec.z = temp_alt;
        terrain_alt = false;
    }

    // copy xy (we do this to ensure we do not adjust vector unless the overall conversion is successful
    vec.x = res_vec.x;
    vec.y = res_vec.y;

    return true;
}

// update target position (an offset from ekf origin), velocity and acceleration to consume the altitude offset
void AC_WPNav::update_targets_with_offset(float alt_offset, Vector3f &target_pos, Vector3f &target_vel, Vector3f &target_accel, float dt)
{
    const float timeconstant = 2.0f;
    const float kp_v = 1.0f/timeconstant;
    const float kp_a = 4.0f/timeconstant;
    const float jerk_max = _wp_accel_z_cmss * kp_a;

    const float pos_error = _pos_target_offset + alt_offset;
    const float accel_v_max = _wp_accel_z_cmss*(1-kp_v/kp_a);
    const float linear_error = accel_v_max / (kp_v * kp_v);
    const float pos_error_mag = fabsf(pos_error);
    float offset_vel_target;
    if (pos_error_mag > linear_error) {
        float kp_s = safe_sqrt(2.0 * accel_v_max * (pos_error_mag - (linear_error / 2.0))) / pos_error_mag;
        offset_vel_target = kp_s * pos_error;
    } else {
        offset_vel_target = kp_v * pos_error;
    }
    offset_vel_target = constrain_float(offset_vel_target, -_wp_speed_down_cms, _wp_speed_up_cms);

    const float vel_error = (offset_vel_target - _vel_target_offset);
    _accel_target_offset = constrain_float(kp_a * vel_error, MAX(-_wp_accel_z_cmss, _accel_target_offset - jerk_max * dt), MIN(_wp_accel_z_cmss, _accel_target_offset + jerk_max * dt));

    _pos_target_offset += _vel_target_offset * dt + _accel_target_offset * dt * dt;
    _vel_target_offset += _accel_target_offset * dt;

    target_pos.z += _pos_target_offset;
    target_vel.z += _vel_target_offset;
    target_accel.z += _accel_target_offset;
}

/// wp_speed_update - calculates how to handle speed change requests
void AC_WPNav::wp_speed_update(float dt)
{
    // return if speed has not changed
    float curr_max_speed_xy_cms = _pos_control.get_max_speed_xy();
    if (is_equal(_wp_desired_speed_xy_cms, curr_max_speed_xy_cms)) {
        return;
    }
    // calculate speed change
    if (_wp_desired_speed_xy_cms > curr_max_speed_xy_cms) {
        // speed up is requested so increase speed within limit set by WPNAV_ACCEL
        curr_max_speed_xy_cms += _wp_accel_cmss * dt;
        if (curr_max_speed_xy_cms > _wp_desired_speed_xy_cms) {
            curr_max_speed_xy_cms = _wp_desired_speed_xy_cms;
        }
    } else if (_wp_desired_speed_xy_cms < curr_max_speed_xy_cms) {
        // slow down is requested so reduce speed within limit set by WPNAV_ACCEL
        curr_max_speed_xy_cms -= _wp_accel_cmss * dt;
        if (curr_max_speed_xy_cms < _wp_desired_speed_xy_cms) {
            curr_max_speed_xy_cms = _wp_desired_speed_xy_cms;
        }
    }

    // update position controller speed
    _pos_control.set_max_speed_xy(curr_max_speed_xy_cms);
}

/// update scruve with acceleration and velocity limits
void AC_WPNav::set_kinematic_limits(scurves &scurve_leg, const Vector3f &origin, const Vector3f &destination)
{
    float accel_max, vel_max;
    const float z_length = destination.z - origin.z;
    const float xy_length = Vector2f(destination.x - origin.x, destination.y - origin.y).length();
    const float wp_speed_xy_cms = MAX(_wp_desired_speed_xy_cms, WPNAV_WP_SPEED_MIN);
    const float wp_accel_xy_cmss = _pos_control.get_max_accel_xy();
    const float wp_speed_up_cms = _pos_control.get_max_speed_up();
    const float wp_speed_down_cms = fabsf(_pos_control.get_max_speed_down());
    const float wp_accel_z_cmss = _pos_control.get_max_accel_z();

    if (is_zero(xy_length)) {
        accel_max = wp_accel_z_cmss;
        if (is_positive(z_length)) {
            vel_max = wp_speed_up_cms;
        } else {
            vel_max = wp_speed_down_cms;
        }
    } else {
        float slope = z_length/xy_length;

        if (fabsf(slope) < wp_accel_z_cmss/wp_accel_xy_cmss) {
            accel_max = wp_accel_xy_cmss;
        } else {
            accel_max = wp_accel_z_cmss;
        }

        if (is_positive(slope)) {
            if (fabsf(slope) < wp_speed_up_cms/wp_speed_xy_cms) {
                vel_max = wp_speed_xy_cms;
            } else {
                vel_max = wp_speed_up_cms;
            }
        } else {
            if (fabsf(slope) < wp_speed_down_cms/wp_speed_xy_cms) {
                vel_max = wp_speed_xy_cms;
            } else {
                vel_max = wp_speed_down_cms;
            }
        }
    }
    scurve_leg.set_vel_max(vel_max);
    scurve_leg.set_accel_max(accel_max);
}
