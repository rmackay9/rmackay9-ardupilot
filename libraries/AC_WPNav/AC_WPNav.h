#pragma once

#include <AP_Common/AP_Common.h>
#include <AP_Param/AP_Param.h>
#include <AP_Math/AP_Math.h>
#include <AP_Math/scurves.h>
#include <AP_Common/Location.h>
#include <AP_InertialNav/AP_InertialNav.h>     // Inertial Navigation library
#include <AC_AttitudeControl/AC_PosControl.h>      // Position control library
#include <AC_AttitudeControl/AC_AttitudeControl.h> // Attitude control library
#include <AP_Terrain/AP_Terrain.h>
#include <AC_Avoidance/AC_Avoid.h>                 // Stop at fence library

// maximum velocities and accelerations
#define WPNAV_ACCELERATION              100.0f      // defines the default velocity vs distant curve.  maximum acceleration in cm/s/s that position controller asks for from acceleration controller
#define WPNAV_ACCELERATION_MIN           50.0f      // minimum acceleration in cm/s/s - used for sanity checking _wp_accel parameter

#define WPNAV_WP_SPEED                  500.0f      // default horizontal speed between waypoints in cm/s
#define WPNAV_WP_SPEED_MIN               20.0f      // minimum horizontal speed between waypoints in cm/s
#define WPNAV_WP_TRACK_SPEED_MIN         50.0f      // minimum speed along track of the target point the vehicle is chasing in cm/s (used as target slows down before reaching destination)
#define WPNAV_WP_RADIUS                 200.0f      // default waypoint radius in cm
#define WPNAV_WP_RADIUS_MIN               5.0f      // minimum waypoint radius in cm

#define WPNAV_WP_SPEED_UP               250.0f      // default maximum climb velocity
#define WPNAV_WP_SPEED_DOWN             150.0f      // default maximum descent velocity

#define WPNAV_WP_ACCEL_Z_DEFAULT        100.0f      // default vertical acceleration between waypoints in cm/s/s

#define WPNAV_LEASH_LENGTH_MIN          100.0f      // minimum leash lengths in cm

#define WPNAV_WP_FAST_OVERSHOOT_MAX     200.0f      // 2m overshoot is allowed during fast waypoints to allow for smooth transitions to next waypoint

#define WPNAV_YAW_DIST_MIN                 200      // minimum track length which will lead to target yaw being updated to point at next waypoint.  Under this distance the yaw target will be frozen at the current heading
#define WPNAV_YAW_VEL_MIN                   10      // target velocity must be at least 10cm/s for vehicle's yaw to change


#define WPNAV_RANGEFINDER_FILT_Z         0.25f      // range finder distance filtered at 0.25hz

class AC_WPNav
{
public:

    // spline segment end types enum
    enum spline_segment_end_type {
        SEGMENT_END_STOP = 0,
        SEGMENT_END_STRAIGHT,
        SEGMENT_END_SPLINE
    };

    /// Constructor
    AC_WPNav(const AP_InertialNav& inav, const AP_AHRS_View& ahrs, AC_PosControl& pos_control, const AC_AttitudeControl& attitude_control);

    /// provide pointer to terrain database
    void set_terrain(AP_Terrain* terrain_ptr) { _terrain = terrain_ptr; }

    /// provide rangefinder altitude
    void set_rangefinder_alt(bool use, bool healthy, float alt_cm) { _rangefinder_available = use; _rangefinder_healthy = healthy; _rangefinder_alt_cm = alt_cm; }

    // return true if range finder may be used for terrain following
    bool rangefinder_used() const { return _rangefinder_use; }
    bool rangefinder_used_and_healthy() const { return _rangefinder_use && _rangefinder_healthy; }

    // get expected source of terrain data if alt-above-terrain command is executed (used by Copter's ModeRTL)
    enum class TerrainSource {
        TERRAIN_UNAVAILABLE,
        TERRAIN_FROM_RANGEFINDER,
        TERRAIN_FROM_TERRAINDATABASE,
    };
    AC_WPNav::TerrainSource get_terrain_source() const;

    ///
    /// waypoint controller
    ///

    /// wp_and_spline_init - initialise straight line and spline waypoint controllers
    ///     speed_cms should be set to a positive value or left at zero to use the default speed
    ///     updates target roll, pitch targets and I terms based on vehicle lean angles
    ///     should be called once before the waypoint controller is used but does not need to be called before subsequent updates to destination
    void wp_and_spline_init(float speed_cms = 0.0f);

    /// set current target horizontal speed during wp navigation
    void set_speed_xy(float speed_cms);

    /// set current target climb or descent rate during wp navigation
    void set_speed_up(float speed_up_cms);
    void set_speed_down(float speed_down_cms);

    /// get default target horizontal velocity during wp navigation
    float get_default_speed_xy() const { return _wp_speed_cms; }

    /// get default target climb speed in cm/s during missions
    float get_default_speed_up() const { return _wp_speed_up_cms; }

    /// get default target descent rate in cm/s during missions.  Note: always positive
    float get_default_speed_down() const { return _wp_speed_down_cms; }

    /// get_speed_z - returns target descent speed in cm/s during missions.  Note: always positive
    float get_accel_z() const { return _wp_accel_z_cmss; }

    /// get_wp_acceleration - returns acceleration in cm/s/s during missions
    float get_wp_acceleration() const { return _wp_accel_cmss.get(); }

    /// get_wp_destination waypoint using position vector (distance from ekf origin in cm)
    const Vector3f &get_wp_destination() const { return _destination; }

    /// get origin using position vector (distance from ekf origin in cm)
    const Vector3f &get_wp_origin() const { return _origin; }

    /// true if origin.z and destination.z are alt-above-terrain, false if alt-above-ekf-origin
    bool origin_and_destination_are_terrain_alt() const { return _terrain_alt; }

    /// set_wp_destination waypoint using location class
    ///     provide the next_destination if known
    ///     returns false if conversion from location to vector from ekf origin cannot be calculated
    bool set_wp_destination_loc(const Location& destination);
    bool set_wp_destination_loc_next(const Location& destination);

    // returns wp location using location class.
    // returns false if unable to convert from target vector to global
    // coordinates
    bool get_wp_destination_loc(Location& destination) const;

    // returns object avoidance adjusted destination which is always the same as get_wp_destination
    // having this function unifies the AC_WPNav_OA and AC_WPNav interfaces making vehicle code simpler
    virtual bool get_oa_wp_destination(Location& destination) const WARN_IF_UNUSED { return get_wp_destination_loc(destination); }

    /// set_wp_destination waypoint using position vector (distance from ekf origin in cm)
    ///     terrain_alt should be true if destination.z is a desired altitude above terrain
    virtual bool set_wp_destination(const Vector3f& destination, bool terrain_alt = false);
    bool set_wp_destination_next(const Vector3f& destination, bool terrain_alt = false) WARN_IF_UNUSED;

    /// set waypoint destination using NED position vector from ekf origin in meters
    ///     provide next_destination_NED if known
    bool set_wp_destination_NED(const Vector3f& destination_NED);
    bool set_wp_destination_NED_next(const Vector3f& destination_NED);

    /// shift_wp_origin_to_current_pos - shifts the origin and destination so the origin starts at the current position
    ///     used to reset the position just before takeoff
    ///     relies on set_wp_destination or set_wp_origin_and_destination having been called first
    void shift_wp_origin_to_current_pos();

    /// shifts the origin and destination horizontally to the current position
    ///     used to reset the track when taking off without horizontal position control
    ///     relies on set_wp_destination or set_wp_origin_and_destination having been called first
    void shift_wp_origin_and_destination_to_current_pos_xy();

    /// shifts the origin and destination horizontally to the achievable stopping point
    ///     used to reset the track when horizontal navigation is enabled after having been disabled (see Copter's wp_navalt_min)
    ///     relies on set_wp_destination or set_wp_origin_and_destination having been called first
    void shift_wp_origin_and_destination_to_stopping_point_xy();

    /// get_wp_stopping_point_xy - calculates stopping point based on current position, velocity, waypoint acceleration
    ///		results placed in stopping_position vector
    void get_wp_stopping_point_xy(Vector3f& stopping_point) const;
    void get_wp_stopping_point(Vector3f& stopping_point) const;

    /// get_wp_distance_to_destination - get horizontal distance to destination in cm
    virtual float get_wp_distance_to_destination() const;

    /// get_bearing_to_destination - get bearing to next waypoint in centi-degrees
    virtual int32_t get_wp_bearing_to_destination() const;

    /// reached_destination - true when we have come within RADIUS cm of the waypoint
    virtual bool reached_wp_destination() const { return _flags.reached_destination; }

    // reached_wp_destination_xy - true if within RADIUS_CM of waypoint in x/y
    bool reached_wp_destination_xy() const {
        return get_wp_distance_to_destination() < _wp_radius_cm;
    }

    /// update_wpnav - run the wp controller - should be called at 100hz or higher
    virtual bool update_wpnav();

    // returns true if update_wpnav has been run very recently
    bool is_active() const;

    ///
    /// spline methods
    ///

    // get target yaw in centi-degrees (used for wp and spline navigation)
    float get_yaw() const;
    float get_yaw_rate() const;

    /// set_spline_destination waypoint using location class
    ///     returns false if conversion from location to vector from ekf origin cannot be calculated
    ///     next_destination should be set to the next segment's destination
    bool set_spline_destination_loc(const Location& destination, Location next_destination, bool next_is_spline);
    bool set_spline_destination_next_loc(const Location& destination, Location next_destination, bool next_is_spline);

    /// set_spline_destination waypoint using position vector (distance from ekf origin in cm)
    ///     terrain_alt should be true if destination.z is a desired altitude above terrain (false if its desired altitudes above ekf origin)
    ///     next_destination should be set to the next segment's destination
    ///     next_terrain_alt should be true if next_destination.z is a desired altitude above terrain (false if its desired altitudes above ekf origin)
    ///     next_destination.z  must be in the same "frame" as destination.z (i.e. if destination is a alt-above-terrain, next_destination should be too)
    bool set_spline_destination(const Vector3f& destination, bool terrain_alt, const Vector3f& next_destination, bool next_terrain_alt, bool next_is_spline) WARN_IF_UNUSED;
    bool set_spline_destination_next(const Vector3f& destination, bool terrain_alt, const Vector3f& next_destination, bool next_terrain_alt, bool next_is_spline) WARN_IF_UNUSED;

    ///
    /// shared methods
    ///

    /// get desired roll, pitch which should be fed into stabilize controllers
    float get_roll() const { return _pos_control.get_roll(); }
    float get_pitch() const { return _pos_control.get_pitch(); }

    /// advance_wp_target_along_track - move target location along track from origin to destination
    bool advance_wp_target_along_track(float dt) WARN_IF_UNUSED;

    /// return the crosstrack_error - horizontal error of the actual position vs the desired position
    float crosstrack_error() const { return _track_error_xy;}

    // update target position (an offset from ekf origin), velocity and acceleration to consume the altitude offset
    void update_targets_with_offset(float alt_offset, Vector3f &target_pos, Vector3f &target_vel, Vector3f &target_accel, float dt);

    static const struct AP_Param::GroupInfo var_info[];

protected:

    // segment types, either straight or spine
    enum SegmentType {
        SEGMENT_STRAIGHT = 0,
        SEGMENT_SPLINE = 1
    };

    // flags structure
    struct wpnav_flags {
        uint8_t reached_destination     : 1;    // true if we have reached the destination
        uint8_t fast_waypoint           : 1;    // true if we should ignore the waypoint radius and consider the waypoint complete once the intermediate target has reached the waypoint
        SegmentType segment_type        : 1;    // active segment is either straight or spline
        uint8_t wp_yaw_set              : 1;    // true if yaw target has been set
    } _flags;

    
    /// wp_speed_update - calculates how to change speed when changes are requested
    void wp_speed_update(float dt);

    // get terrain's altitude (in cm above the ekf origin) at the current position (+ve means terrain below vehicle is above ekf origin's altitude)
    bool get_terrain_offset(float& offset_cm) WARN_IF_UNUSED;

    // convert location to vector from ekf origin.  terrain_alt is set to true if resulting vector's z-axis should be treated as alt-above-terrain
    //      returns false if conversion failed (likely because terrain data was not available)
    bool get_vector_NEU(const Location &loc, Vector3f &vec, bool &terrain_alt) WARN_IF_UNUSED;

    // set heading used for spline and waypoint navigation
    void set_yaw_cd(float heading_cd);
    void set_yaw_rate_cds(float yaw_rate_cds);

    /// update scurve with acceleration and velocity limits
    void set_kinematic_limits(scurves &scurve_leg, const Vector3f &origin, const Vector3f &destination);

    // references and pointers to external libraries
    const AP_InertialNav&   _inav;
    const AP_AHRS_View&     _ahrs;
    AC_PosControl&          _pos_control;
    const AC_AttitudeControl& _attitude_control;
    AP_Terrain              *_terrain;

    // parameters
    AP_Float    _wp_speed_cms;          // default maximum horizontal speed in cm/s during missions
    AP_Float    _wp_speed_up_cms;       // default maximum climb rate in cm/s
    AP_Float    _wp_speed_down_cms;     // default maximum descent rate in cm/s
    AP_Float    _wp_radius_cm;          // distance from a waypoint in cm that, when crossed, indicates the wp has been reached
    AP_Float    _wp_accel_cmss;          // horizontal acceleration in cm/s/s during missions
    AP_Float    _wp_jerk;               // maximum jerk used to generate s-curve trajectories
    AP_Float    _wp_accel_z_cmss;        // vertical acceleration in cm/s/s during missions

    // scurve
    scurves _scurve_prev_leg;           // previous spline trajectory used to blend with current s-curve trajectory
    scurves _scurve_this_leg;           // current spline trajectory
    scurves _scurve_next_leg;           // next spline trajectory used to blend with current s-curve trajectory

    // waypoint controller internal variables
    uint32_t    _wp_last_update;        // time of last update_wpnav call
    float       _wp_desired_speed_xy_cms;   // desired wp speed in cm/sec
    Vector3f    _origin;                // starting point of trip to next waypoint in cm from ekf origin
    Vector3f    _destination;           // target destination in cm from ekf origin
    float       _track_error_xy;        // horizontal error of the actual position vs the desired position
    float       _track_desired;         // our desired distance along the track in cm
    float       _track_scaler_dt;       // time compression multiplier to slow the progress along the track
    float       _yaw;                   // current yaw heading based on track direction
    float       _yaw_rate;              // current yaw rate based on track curvature

    // terrain following variables
    bool        _terrain_alt;   // true if origin and destination.z are alt-above-terrain, false if alt-above-ekf-origin
    bool        _rangefinder_available; // true if rangefinder is enabled (user switch can turn this true/false)
    AP_Int8     _rangefinder_use;       // parameter that specifies if the range finder should be used for terrain following commands
    bool        _rangefinder_healthy;   // true if rangefinder distance is healthy (i.e. between min and maximum)
    float       _rangefinder_alt_cm;    // latest distance from the rangefinder

    // position, velocity and acceleration targets passed to position controller
    float       _pos_target_offset;
    float       _vel_target_offset;
    float       _accel_target_offset;

};
