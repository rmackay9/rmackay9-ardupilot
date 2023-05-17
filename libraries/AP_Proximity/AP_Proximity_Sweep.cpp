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

#include "AP_Proximity_Sweep.h"
#include <GCS_MAVLink/GCS.h>
#include <AP_Logger/AP_Logger.h>

// add an angle and distance to the sweep buffer
// this should be called by the proximity driver
void AP_Proximity_Sweep::add_distance(float angle_deg, float distance_m)
{
    // ignore invalid distances or out-of-range angles
    if (distance_m <= 0) {
        return;
    }

    // ensure angle is in -180 to +180 range
    angle_deg = wrap_180(angle_deg);

    // ignore out-of-range angles
    if ((angle_deg < object_detect_params.angle_min_deg) || (angle_deg > object_detect_params.angle_max_deg)) {
        angle_in_range = false;
        return;
    }

    // throw away duplicate angles
    if (is_equal(angle_deg, prev_angle_deg)) {
        return;
    }

    // apply mode filter to reduce noise
    //distance_m = dist_filt.apply(distance_m);

    // check for reversal in direction
    //SweepDirection new_sweep_dir = angle_deg > prev_angle_deg ? SweepDirection::ANGLE_INCREASING : SweepDirection::ANGLE_DECREASING;
    //const bool sweep_dir_changed = new_sweep_dir != internal_sweep_dir;
    //internal_sweep_dir = new_sweep_dir;
    prev_angle_deg = angle_deg;

    // check for wrap around
    const bool wrapped = !angle_in_range;
    angle_in_range = true;

    //if (sweep_dir_changed || wrapped) {
    if (wrapped) {
        //gcs().send_text(MAV_SEVERITY_CRITICAL,"Dir:%d prev:%f now:%f", (int)new_sweep_dir, (double)prev_angle_deg, (double)angle_deg);
        // advance sweep id
        advance_sweep();

        // calculate closest object so cache is ready for external caller
        calculate_closest_object();

        //gcs().send_text(MAV_SEVERITY_CRITICAL,"Sweep advanced dir:%d wrap:%d", (int)sweep_dir_changed, (int)wrapped);
    }

    // exit immediately if items array is full
    if (sweeps[internal_sweep_index].count >= ARRAY_SIZE(sweeps[internal_sweep_index].items)) {
        return;
    }

    // add an angle and distance to the sweep buffer
    sweeps[internal_sweep_index].items[sweeps[internal_sweep_index].count++] = {angle_deg, distance_m};
}

// return current sweep id for use by external callers (required to retrieve distances using get_distance)
uint8_t AP_Proximity_Sweep::get_current_sweep_id() const
{
    return sweeps[internal_sweep_index].sweep_id;
}

// get the number of items for a given sweep_id
uint16_t AP_Proximity_Sweep::get_num_items(uint8_t sweep_id) const
{
    for (uint8_t i=0; i<ARRAY_SIZE(sweeps); i++) {
        if (sweeps[i].sweep_id == sweep_id) {
            return sweeps[i].count;
        }
    }

    // expired or invalid sweep id
    return 0;
}

// retrieve a single item from a sweep
// sweep_id should have been retrieved from a call to get_latest_sweep_id
// item_num should be between 0 and the return value of get_num_items
bool AP_Proximity_Sweep::get_item(uint8_t sweep_id, uint16_t item_num, float &angle_deg, float &distance_m) const
{
    // check sweep id matches external sweep id
    // this protects against callers trying to access old distance data
    uint8_t external_sweep_index = internal_sweep_index == 0 ? 1 : 0;
    if (sweep_id != sweeps[external_sweep_index].sweep_id) {
        return false;
    }

    // sanity check item_num
    if (item_num >= sweeps[external_sweep_index].count) {
        return false;
    }

    // return angle and distance
    angle_deg = sweeps[external_sweep_index].items[item_num].angle_deg;
    distance_m = sweeps[external_sweep_index].items[item_num].distance_m;
    return true;
}

// set object detection params
void AP_Proximity_Sweep::set_params(uint8_t debug, float dist_jump_m, float angle_min_deg, float angle_max_deg)
{
    debug_level = debug;
    object_detect_params.dist_jump_m = dist_jump_m;
    object_detect_params.angle_min_deg = angle_min_deg;
    object_detect_params.angle_max_deg = angle_max_deg;
}

// get the angle to the closest object
// returns true on success, false on failure
bool AP_Proximity_Sweep::get_closest_object(float &angle_deg) const
{
    uint8_t external_sweep_index = internal_sweep_index == 0 ? 1 : 0;

    // check cache
    if (closest_cache.sweep_id == sweeps[external_sweep_index].sweep_id) {
        angle_deg = closest_cache.angle_deg;
        return true;
    }
    return false;
}

// calculate the angle to the closest object
void AP_Proximity_Sweep::calculate_closest_object()
{
    uint8_t external_sweep_index = internal_sweep_index == 0 ? 1 : 0;

    // sanity check we have data
    if (sweeps[external_sweep_index].count == 0) {
        return;
    }

    // find index of shortest distance
    uint16_t closest_dist_index = 0;
    for (uint16_t i=1; i<sweeps[external_sweep_index].count; i++) {
        if (sweeps[external_sweep_index].items[i].distance_m < sweeps[external_sweep_index].items[closest_dist_index].distance_m) {
            closest_dist_index = i;
        }
    }

    // search for jump in distance to left
    bool left_edge_found = false;
    uint16_t left_edge_index = 0;
    for (int16_t i=closest_dist_index; i>=0; i--) {
        if (fabsf(sweeps[external_sweep_index].items[i].distance_m - sweeps[external_sweep_index].items[closest_dist_index].distance_m) >= object_detect_params.dist_jump_m) {
            left_edge_found = true;
            left_edge_index = MAX(i,0);
            break;
        }
    }

    // search for jump in distance to right
    bool right_edge_found = false;
    uint16_t right_edge_index = sweeps[external_sweep_index].count - 1;
    for (uint16_t i=closest_dist_index; i<sweeps[external_sweep_index].count; i++) {
        if (fabsf(sweeps[external_sweep_index].items[i].distance_m - sweeps[external_sweep_index].items[closest_dist_index].distance_m) >= object_detect_params.dist_jump_m) {
            right_edge_found = true;
            right_edge_index = i;
            break;
        }
    }

    // check at least left or right edge was found
    bool object_found = left_edge_found || right_edge_found;
    if (object_found) {
        // calculate middle angle and store results in cache
        closest_cache.sweep_id = sweeps[external_sweep_index].sweep_id;
        closest_cache.angle_deg = (sweeps[external_sweep_index].items[left_edge_index].angle_deg + sweeps[external_sweep_index].items[right_edge_index].angle_deg) * 0.5;

        if (debug_level >= 1) {
            gcs().send_text(MAV_SEVERITY_CRITICAL,"Sweep: num:%u C:%u L:%u R:%u ang:%4.2f", (unsigned)sweeps[external_sweep_index].count, (unsigned)closest_dist_index, (unsigned)left_edge_index, (unsigned)right_edge_index, (double)closest_cache.angle_deg);
        }
    } else if (debug_level >= 1) {
        gcs().send_text(MAV_SEVERITY_CRITICAL, "Sweep: edge not found");
        return;
    }

    // log at high debug levels
    if (debug_level >= 2) {
        static uint8_t log_counter = 0;
        log_counter++;
        if (log_counter >= 10 || debug_level >= 3) {
            log_counter = 0;

            // log each reading separately
            for (uint16_t i=0; i<sweeps[external_sweep_index].count; i++) {
                // @LoggerMessage: SWEP
                // @Description: Sweep data
                // @Field: TimeUS: Time since system startup
                // @Field: SweepId: Sweep Id
                // @Field: Angle: Angle of measurement
                // @Field: Dist: Distance measurement
                // @Field: Left: True if left edge
                // @Field: Center: True if center of object
                // @Field: Right: True if right edge
                AP::logger().WriteStreaming(
                    "SWEP",
                    "TimeUS,SweepId,Angle,Dist,Left,Center,Right",
                    "s-dm---",
                    "F-00---",
                    "QBffBBB",
                    AP_HAL::micros64(),
                    (uint8_t)sweeps[external_sweep_index].sweep_id,
                    (double)sweeps[external_sweep_index].items[i].angle_deg,
                    (double)sweeps[external_sweep_index].items[i].distance_m,
                    (uint8_t)(left_edge_found && (i == left_edge_index)),
                    (uint8_t)closest_dist_index,
                    (uint8_t)(right_edge_found && (i == right_edge_index)));
            }
        }
    }
}

// start a new sweep
// make the current internal sweep available to external callers
void AP_Proximity_Sweep::advance_sweep()
{
    const uint8_t prev_sweep_id = sweeps[internal_sweep_index].sweep_id;

    // increment the internal sweep id
    if (internal_sweep_index == 0) {
        internal_sweep_index = 1;
    } else {
        internal_sweep_index = 0;
    }

    // clear the internal sweep array
    sweeps[internal_sweep_index].count = 0;

    // increment sweep ids
    sweeps[internal_sweep_index].sweep_id = prev_sweep_id + 1;

}
