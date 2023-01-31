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

#pragma once

#include <AP_Common/AP_Common.h>
#include <AP_Math/AP_Math.h>

#define PROXIMITY_SWEEP_NUM_ITEMS_MAX 500     // maximum number of items (angle+distances) that can be stored

class AP_Proximity_Sweep
{
public:

    // add an angle and distance to the sweep buffer
    // this should be called by the proximity driver
    void add_distance(float angle_deg, float distance_m);

    // return current sweep id for use by external callers (required to retrieve distances using get_distance)
    uint8_t get_current_sweep_id() const;

    // get the number of items for a given sweep_id
    uint16_t get_num_items(uint8_t sweep_id) const;

    // retrieve a single item from a sweep
    // sweep_id should have been retrieved from a call to get_current_sweep_id
    // item_num should be between 0 and the return value of get_num_items
    // returns true on success, false on failure
    bool get_item(uint8_t sweep_id, uint16_t item_num, float &angle_deg, float &distance_m) const;

    // set object detection params
    void set_params(uint8_t debug, float dist_jump_m, float angle_min_deg, float angle_max_deg);

    // get the angle to the closest object
    // dist_jump is the distance change in meters used to detect the edge of objects
    // returns true on success, false on failure
    bool get_closest_object(float &angle_deg);

private:

    // start a new sweep
    void advance_sweep();

    // Item holding a single measurement of angle and distance
    struct Item
    {
        float angle_deg;    // angle in degrees
        float distance_m;   // distance in meters
    };

    // structure holding distances for two sweeps
    struct {
        Item items[PROXIMITY_SWEEP_NUM_ITEMS_MAX];  // buffer of items (distances and angles)
        uint16_t count;                             // number of items in items array
        uint8_t sweep_id;                           // sweep id of items
    } sweeps[2];
    uint8_t internal_sweep_index;                   // the internal sweep's index in the items array

    enum class SweepDirection {
        ANGLE_DECREASING = 0,
        ANGLE_INCREASING = 1
    } internal_sweep_dir;                           // direction of internal sweep
    float prev_angle_deg;                           // previous recorded angle from sensor.  Used to check sweep direction

    uint8_t debug_level = 0;

    // closest object variables
    struct {
        float angle_min_deg;
        float angle_max_deg;
        float dist_jump_m;
    } object_detect_params;
    struct {
        uint8_t sweep_id = 0xFF;
        float angle_deg;
    } closest_cache;
};
