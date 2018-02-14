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

#include <AP_HAL/AP_HAL.h>
#include <GCS_MAVLink/GCS.h>
#include "AP_Follow.h"
#include <ctype.h>
#include <stdio.h>

extern const AP_HAL::HAL& hal;

#define AP_FOLLOW_TIMEOUT_MS    1000    // position estimate timeout after 1 second

#define AP_FOLLOW_OFFSET_TYPE_NED       0   // offsets are in north-east-down frame
#define AP_FOLLOW_OFFSET_TYPE_RELATIVE  0   // offsets are relative to lead vehicle's heading

// table of user settable parameters
const AP_Param::GroupInfo AP_Follow::var_info[] = {

    // @Param: _ENABLE
    // @DisplayName: Follow enable/disable
    // @Description: Enabled/disable following a target
    // @Values: 0:Disabled,1:Enabled
    // @User: Standard
    AP_GROUPINFO_FLAGS("_ENABLE", 1, AP_Follow, _enabled, 0, AP_PARAM_FLAG_ENABLE),

    // 2 is reserved for TYPE parameter

    // @Param: _SYSID
    // @DisplayName: Follow target's mavlink system id
    // @Description: Follow target's mavlink system id
    // @Range: 0 255
    // @User: Standard
    AP_GROUPINFO("_SYSID", 3, AP_Follow, _sysid, 0),

    // 4 is reserved for MARGIN parameter

    // @Param: _DIST_MAX
    // @DisplayName: Follow distance maximum
    // @Description: Follow distance maximum.  targets further than this will be ignored
    // @Units: m
    // @Range: 1 1000
    // @User: Standard
    AP_GROUPINFO("_DIST_MAX", 5, AP_Follow, _dist_max, 100),

    // @Param: _OFS_TYPE
    // @DisplayName: Follow offset type
    // @Description: Follow offset type
    // @Values: 0:North-East-Down, 1:Relative to lead vehicle heading
    // @User: Standard
    AP_GROUPINFO("_OFS_TYPE", 6, AP_Follow, _offset_type, AP_FOLLOW_OFFSET_TYPE_NED),

    // @Param: _OFS_X
    // @DisplayName: Follow offsets in meters north/forward
    // @Description: Follow offsets in meters north/forward.  If positive, this vehicle fly ahead or north of lead vehicle
    // @Range: -100 100
    // @Units: m
    // @Increment: 1
    // @User: Standard

    // @Param: _OFS_Y
    // @DisplayName: Follow offsets in meters east/right
    // @Description: Follow offsets in meters east/right.  If positive, this vehicle fly to the right or east of lead vehicle
    // @Range: -100 100
    // @Units: m
    // @Increment: 1
    // @User: Standard

    // @Param: _OFS_Z
    // @DisplayName: Follow offsets in meters down
    // @Description: Follow offsets in meters down.  If positive, this vehicle fly below the lead vehicle
    // @Range: -100 100
    // @Units: m
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("_OFS", 7, AP_Follow, _offset, 0),

    // @Param: _YAW_BEHAVE
    // @DisplayName: Follow yaw behaviour
    // @Description: Follow yaw behaviour
    // @Values: 0:None,1:Face Lead Vehicle,2:Same as Lead vehicle,3:Direction of Flight
    // @User: Standard
    AP_GROUPINFO("_YAW_BEHAVE", 8, AP_Follow, _yaw_behave, 1),

    AP_GROUPEND
};

/* 
   The constructor also initialises the proximity sensor. Note that this
   constructor is not called until detect() returns true, so we
   already know that we should setup the proximity sensor
*/
AP_Follow::AP_Follow(const AP_AHRS &ahrs) :
        _ahrs(ahrs)
{
    AP_Param::setup_object_defaults(this, var_info);
}

// get target's estimated location
bool AP_Follow::get_target_location_and_velocity(Location &loc, Vector3f& vel) const
{
    // exit immediately if not enabled
    if (!_enabled) {
        return false;
    }

    // check for timeout
    if ((_last_location_update_ms == 0) || (AP_HAL::millis() - _last_location_update_ms > AP_FOLLOW_TIMEOUT_MS)) {
        return false;
    }

    // calculate time since last actual position update
    float dt = (AP_HAL::millis() - _last_location_update_ms) / 1000.0f;

    // get velocity estimate
    if (!get_velocity_ned(vel, dt)) {
        return false;
    }

    // project the vehicle position
    Location last_loc = _target_location;
    location_offset(last_loc, vel.x * dt, vel.y * dt);
    last_loc.alt -= vel.z * 10.0f * dt; // convert m/s to cm/s, multiply by dt.  minus because NED

    // return latest position estimate
    loc = last_loc;
    return true;
}

// get distance vector to target (in meters) and target's velocity all in NED frame
bool AP_Follow::get_target_dist_and_vel_ned(Vector3f &dist, Vector3f &dist_with_offs, Vector3f &vel)
{
    // get our location
    Location current_loc;
    if (!_ahrs.get_position(current_loc)) {
         return false;
     }

    // get target location and velocity
    Location target_loc;
    Vector3f veh_vel;
    if (!get_target_location_and_velocity(target_loc, veh_vel)) {
        return false;
    }

    // calculate difference
    Vector3f dist_vec = location_3d_diff_NED(current_loc, target_loc);

    // fail if too far
    if (is_positive(_dist_max.get()) && (dist_vec.length() > _dist_max)) {
        return false;
    }

    // initialise offsets from distance vector if required
    init_offsets_if_required(dist_vec);

    // get offsets
    Vector3f offsets;
    if (!get_offsets_ned(offsets)) {
        return false;
    }

    // return results
    dist = dist_vec;
    dist_with_offs = dist_vec + offsets;
    vel = veh_vel;
    return true;
}

// get target's heading in degrees (0 = north, 90 = east)
bool AP_Follow::get_target_heading(float &heading) const
{
    // exit immediately if not enabled
    if (!_enabled) {
        return false;
    }

    // check for timeout
    if ((_last_heading_update_ms == 0) || (AP_HAL::millis() - _last_heading_update_ms > AP_FOLLOW_TIMEOUT_MS)) {
        return false;
    }

    // return latest heading estimate
    heading = _target_heading;
    return true;
}

// handle mavlink DISTANCE_SENSOR messages
void AP_Follow::handle_msg(const mavlink_message_t &msg)
{
    // exit immediately if not enabled
    if (!_enabled) {
        return;
    }

    // skip our own messages
    if (msg.sysid == mavlink_system.sysid) {
        return;
    }

    // skip message if not from our target
    if ((_sysid != 0) && (msg.sysid != _sysid)) {
        return;
    }

    // decode global-position-int message
    if (msg.msgid == MAVLINK_MSG_ID_GLOBAL_POSITION_INT) {
        uint32_t now = AP_HAL::millis();
        // decode message
        mavlink_global_position_int_t packet;
        mavlink_msg_global_position_int_decode(&msg, &packet);
        _target_location.lat = packet.lat;
        _target_location.lng = packet.lon;
        _target_location.alt = packet.alt/10;       // convert millimeters to cm
        _target_velocity_ned.x = packet.vx/100.0f;  // velocity north
        _target_velocity_ned.y = packet.vy/100.0f;  // velocity east
        _target_velocity_ned.z = packet.vz/100.0f;  // velocity down
        _last_location_update_ms = now;
        if (packet.hdg <= 36000) {                  // heading (UINT16_MAX if unknown)
            _target_heading = packet.hdg / 100.0f;  // convert centi-degrees to degrees
            _last_heading_update_ms = now;
        }
        // initialise _sysid if zero to sender's id
        if (_sysid == 0) {
            _sysid = msg.msgid;
        }
    }
}

// get velocity estimate in NED frame
bool AP_Follow::get_velocity_ned(Vector3f& vel, float dt) const
{
    vel = _target_velocity_ned + (_target_accel_ned * dt);
    return true;
}

// initialise offsets to target if required
void AP_Follow::init_offsets_if_required(const Vector3f& dist_vec_ned)
{
    if (_offset.get().is_zero()) {
        _offset = dist_vec_ned;
    }
}

// get offsets in NED frame
bool AP_Follow::get_offsets_ned(Vector3f& offset) const
{
    const Vector3f &off = _offset.get();

    // if offsets are zero or type if NED, simply return offset vector
    if (off.is_zero() || (_offset_type == AP_FOLLOW_OFFSET_TYPE_NED)) {
        offset = off;
        return true;
    }

    // offset_type == AP_FOLLOW_OFFSET_TYPE_RELATIVE
    // check if we have a valid heading for target vehicle
    if ((_last_heading_update_ms == 0) || (AP_HAL::millis() - _last_heading_update_ms > AP_FOLLOW_TIMEOUT_MS)) {
        return false;
    }

    // rotate roll, pitch input from north facing to vehicle's perspective
    const float veh_cos_yaw = cosf(radians(_target_heading));
    const float veh_sin_yaw = sinf(radians(_target_heading));
    offset.x = (off.x * veh_cos_yaw) - (off.y * veh_sin_yaw);
    offset.y = (off.y * veh_cos_yaw) + (off.x * veh_sin_yaw);
    offset.z = off.z;
    return true;
}
