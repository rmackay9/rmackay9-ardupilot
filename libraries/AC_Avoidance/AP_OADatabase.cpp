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

#include "AP_OADatabase.h"

#include <AP_AHRS/AP_AHRS.h>
#include <GCS_MAVLink/GCS.h>
#include <AP_Math/AP_Math.h>
#include <AP_Vehicle/AP_Vehicle_Type.h>

extern const AP_HAL::HAL& hal;

#ifndef AP_OADATABASE_TIMEOUT_SECONDS_DEFAULT
    #define AP_OADATABASE_TIMEOUT_SECONDS_DEFAULT   10
#endif

#ifndef AP_OADATABASE_SIZE_DEFAULT
    #define AP_OADATABASE_SIZE_DEFAULT          100
#endif

#ifndef AP_OADATABASE_QUEUE_SIZE_DEFAULT
    #define AP_OADATABASE_QUEUE_SIZE_DEFAULT 80
#endif

#ifndef AP_OADATABASE_DISTANCE_FROM_HOME
    #define AP_OADATABASE_DISTANCE_FROM_HOME 3
#endif

const AP_Param::GroupInfo AP_OADatabase::var_info[] = {

    // @Param: SIZE
    // @DisplayName: OADatabase maximum number of points
    // @Description: OADatabase maximum number of points. Set to 0 to disable the OA Database. Larger means more points but is more cpu intensive to process
    // @Range: 0 10000
    // @User: Advanced
    // @RebootRequired: True
    AP_GROUPINFO("SIZE", 1, AP_OADatabase, _database_size_param, AP_OADATABASE_SIZE_DEFAULT),

    // @Param: EXPIRE
    // @DisplayName: OADatabase item timeout
    // @Description: OADatabase item timeout. The time an item will linger without any updates before it expires. Zero means never expires which is useful for a sent-once static environment but terrible for dynamic ones.
    // @Units: s
    // @Range: 0 127
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("EXPIRE", 2, AP_OADatabase, _database_expiry_seconds, AP_OADATABASE_TIMEOUT_SECONDS_DEFAULT),

    // @Param: QUEUE_SIZE
    // @DisplayName: OADatabase queue maximum number of points
    // @Description: OADatabase queue maximum number of points. This in an input buffer size. Larger means it can handle larger bursts of incoming data points to filter into the database. No impact on cpu, only RAM. Recommend larger for faster datalinks or for sensors that generate a lot of data.
    // @Range: 1 200
    // @User: Advanced
    // @RebootRequired: True
    AP_GROUPINFO("QUEUE_SIZE", 3, AP_OADatabase, _queue_size_param, AP_OADATABASE_QUEUE_SIZE_DEFAULT),

    // @Param: OUTPUT
    // @DisplayName: OADatabase output level
    // @Description: OADatabase output level to configure which database objects are sent to the ground station. All data is always available internally for avoidance algorithms.
    // @Values: 0:Disabled,1:Send only HIGH importance items,2:Send HIGH and NORMAL importance items,3:Send all items
    // @User: Advanced
    AP_GROUPINFO("OUTPUT", 4, AP_OADatabase, _output_level, (float)OutputLevel::HIGH),

    // @Param: BEAM_WIDTH
    // @DisplayName: OADatabase beam width
    // @Description: Beam width of incoming lidar data
    // @Units: deg
    // @Range: 1 10
    // @User: Advanced
    // @RebootRequired: True
    AP_GROUPINFO("BEAM_WIDTH", 5, AP_OADatabase, _beam_width, 5.0f),

    // @Param: RADIUS_MIN
    // @DisplayName: OADatabase Minimum  radius
    // @Description: Minimum radius of objects held in database
    // @Units: m
    // @Range: 0 10
    // @User: Advanced
    AP_GROUPINFO("RADIUS_MIN", 6, AP_OADatabase, _radius_min, 0.01f),

    // @Param: DIST_MAX
    // @DisplayName: OADatabase Distance Maximum
    // @Description: Maximum distance of objects held in database.  Set to zero to disable the limits
    // @Units: m
    // @Range: 0 10
    // @User: Advanced
    AP_GROUPINFO("DIST_MAX", 7, AP_OADatabase, _dist_max, 0.0f),

    // @Param{Copter}: ALT_MIN
    // @DisplayName: OADatabase minimum altitude above home before storing obstacles
    // @Description: OADatabase will reject obstacles if vehicle's altitude above home is below this parameter, in a 3 meter radius around home. Set 0 to disable this feature.
    // @Units: m
    // @Range: 0 4
    // @User: Advanced
    AP_GROUPINFO_FRAME("ALT_MIN", 8, AP_OADatabase, _min_alt, 0.0f, AP_PARAM_FRAME_COPTER | AP_PARAM_FRAME_HELI | AP_PARAM_FRAME_TRICOPTER),

    AP_GROUPEND
};

AP_OADatabase::AP_OADatabase()
{
    if (_singleton != nullptr) {
        AP_HAL::panic("AP_OADatabase must be singleton");
    }
    _singleton = this;

    AP_Param::setup_object_defaults(this, var_info);
}

void AP_OADatabase::init()
{
    init_database();
    init_queue();

    // initialise scalar using beam width of at least 1deg
    dist_to_radius_scalar = tanf(radians(MAX(_beam_width, 1.0f)));

    if (!healthy()) {
        gcs().send_text(MAV_SEVERITY_INFO, "DB init failed . Sizes queue:%u, db:%u", (unsigned int)_queue.size, (unsigned int)_database.size);
        delete _queue.items;
        delete[] _database.items;
        return;
    }
}

void AP_OADatabase::update()
{
    if (!healthy()) {
        return;
    }

    process_queue();
    database_items_remove_all_expired();

    // calculate distance to closest object
    static uint32_t last_closest_object_ms = 0;
    uint32_t now_ms = AP_HAL::millis();
    if (now_ms - last_closest_object_ms > 5000) {
        last_closest_object_ms = now_ms;
        Vector2f veh_pos_NE;
        if (AP::ahrs().get_relative_position_NE_origin(veh_pos_NE)) {
            float yaw_to_obj_rad;
            if (dir_to_largest_object(veh_pos_NE, yaw_to_obj_rad)) {
                gcs().send_text(MAV_SEVERITY_INFO, "OAdb: dir to obj:%4.1f", (double)degrees(yaw_to_obj_rad));
            } else {
                gcs().send_text(MAV_SEVERITY_INFO, "OAdb: no object");
            }
        }
        // display largest objects
        //uint8_t object_count = 0;
        //for (uint8_t i=0; i<ARRAY_SIZE(_object_item_count) && (object_count < 10); i++) {
        //    if (_object_item_count[i] > 0) {
        //        object_count++;
        //        gcs().send_text(MAV_SEVERITY_INFO, "OAdb: obj:%u cnt:%u", (unsigned)i, (unsigned)_object_item_count[i]);
        //    }
        //}
    }
}

// push a location into the database
void AP_OADatabase::queue_push(const Vector3f &pos, uint32_t timestamp_ms, float distance)
{
    if (!healthy()) {
        return;
    }

    // check if this obstacle needs to be rejected from DB because of low altitude near home
#if APM_BUILD_COPTER_OR_HELI
    if (!is_zero(_min_alt)) { 
        Vector3f current_pos;
        if (!AP::ahrs().get_relative_position_NED_home(current_pos)) {
            // we do not know where the vehicle is
            return;
        }
        if (current_pos.xy().length() < AP_OADATABASE_DISTANCE_FROM_HOME) {
            // vehicle is within a small radius of home 
            if (-current_pos.z < _min_alt) {
                // vehicle is below the minimum alt
                return;
            }
        }
    }
#endif
    
    // ignore objects that are far away
    if ((_dist_max > 0.0f) && (distance > _dist_max)) {
        return;
    }

    const OA_DbItem item = {pos, timestamp_ms, MAX(_radius_min, distance * dist_to_radius_scalar), 0, AP_OADatabase::OA_DbItemImportance::Normal};
    {
        WITH_SEMAPHORE(_queue.sem);
        _queue.items->push(item);
    }
}

void AP_OADatabase::init_queue()
{
    _queue.size = _queue_size_param;
    if (_queue.size == 0) {
        return;
    }

    _queue.items = new ObjectBuffer<OA_DbItem>(_queue.size);
    if (_queue.items != nullptr && _queue.items->get_size() == 0) {
        // allocation failed
        delete _queue.items;
        _queue.items = nullptr;
    }
}

void AP_OADatabase::init_database()
{
    _database.size = _database_size_param;
    if (_database_size_param == 0) {
        return;
    }

    _database.items = new OA_DbItem[_database.size];
}

// get bitmask of gcs channels item should be sent to based on its importance
// returns 0xFF (send to all channels) if should be sent, 0 if it should not be sent
uint8_t AP_OADatabase::get_send_to_gcs_flags(const OA_DbItemImportance importance)
{
    switch (importance) {
    case OA_DbItemImportance::Low:
        if (_output_level >= OutputLevel::ALL) {
            return 0xFF;
        }
        break;

    case OA_DbItemImportance::Normal:
        if (_output_level >= OutputLevel::HIGH_AND_NORMAL) {
            return 0xFF;
        }
        break;

    case OA_DbItemImportance::High:
        if (_output_level >= OutputLevel::HIGH) {
            return 0xFF;
        }
        break;
    }
    return 0x0;
}

// returns true when there's more work inthe queue to do
bool AP_OADatabase::process_queue()
{
    if (!healthy()) {
        return false;
    }

    // processing queue by moving those entries into the database
    // Using a for with fixed size is better than while(!empty) because the
    // while could get us stuck here longer than expected if we're getting
    // a lot of values pushing into it while we're trying to empty it. With
    // the for we know we will exit at an expected time
    const uint16_t queue_available = MIN(_queue.items->available(), 100U);
    if (queue_available == 0) {
        return false;
    }

    for (uint16_t queue_index=0; queue_index<queue_available; queue_index++) {
        OA_DbItem item;

        bool pop_success;
        {
            WITH_SEMAPHORE(_queue.sem);
            pop_success = _queue.items->pop(item);
        }
        if (!pop_success) {
            return false;
        }

        item.send_to_gcs = get_send_to_gcs_flags(item.importance);

        // compare item to all items in database. If an existing point is close, update its timestamp
        bool found = false;
        float distance_sq_min = 0;
        uint16_t distance_sq_min_index = INT16_MAX;
        const float item_radius_sq = sq(item.radius);
        for (uint16_t i=0; i<_database.count; i++) {
            // calculate distance (squared) to existing item in database
            const float distance_sq = (_database.items[i].pos - item.pos).length_squared();

            // record index and distance of the closest item
            if (distance_sq_min_index == INT16_MAX || distance_sq < distance_sq_min) {
                distance_sq_min_index = i;
                distance_sq_min = distance_sq;
            }

            // if existing item is close, update its timestamp
            if ((distance_sq < item_radius_sq) || (distance_sq < sq(_database.items[i].radius))) {
                database_item_refresh(i, item.timestamp_ms, item.radius);
                found = true;
                break;
            }
        }

        // if no existing items are close, add new item
        if (!found) {
            // use object id of closest item if within twice the radius
            const float item_radius_x2_sq = sq(2.0 * item.radius);
            if ((distance_sq_min_index != INT16_MAX) && (distance_sq_min < item_radius_x2_sq)) {
                item.object_id = _database.items[distance_sq_min_index].object_id;
            } else {
                // otherwise use new object id
                item.object_id = get_next_object_id();
            }
            database_item_add(item);
        }
    }
    return (_queue.items->available() > 0);
}

void AP_OADatabase::database_item_add(const OA_DbItem &item)
{
    if (_database.count >= _database.size) {
        return;
    }
    _database.items[_database.count] = item;
    _database.items[_database.count].send_to_gcs = get_send_to_gcs_flags(_database.items[_database.count].importance);
    _database.count++;

    // update object count
    if (item.object_id < ARRAY_SIZE(_object_item_count)) {
        _object_item_count[item.object_id]++;
    }
}

void AP_OADatabase::database_item_remove(const uint16_t index)
{
    if (index >= _database.count || _database.count == 0) {
        // index out of range
        return;
    }

    // update object count
    const uint8_t object_id = _database.items[index].object_id;
    if ((object_id < ARRAY_SIZE(_object_item_count)) && (_object_item_count[object_id] > 0)) {
        _object_item_count[object_id]--;
    }

    // radius of 0 tells the GCS we don't care about it any more (aka it expired)
    _database.items[index].radius = 0;
    _database.items[index].send_to_gcs = get_send_to_gcs_flags(_database.items[index].importance);

    _database.count--;
    if (_database.count == 0) {
        return;
    }

    if (index != _database.count) {
        // copy last object in array over expired object
        _database.items[index] = _database.items[_database.count];
        _database.items[index].send_to_gcs = get_send_to_gcs_flags(_database.items[index].importance);
    }
}

void AP_OADatabase::database_item_refresh(const uint16_t index, const uint32_t timestamp_ms, const float radius)
{
    if (index >= _database.count) {
        // index out of range
        return;
    }

    const bool is_different =
            (!is_equal(_database.items[index].radius, radius)) ||
            (timestamp_ms - _database.items[index].timestamp_ms >= 500);

    if (is_different) {
        // update timestamp and radius on close object so it stays around longer
        // and trigger resending to GCS
        _database.items[index].timestamp_ms = timestamp_ms;
        _database.items[index].radius = radius;
        _database.items[index].send_to_gcs = get_send_to_gcs_flags(_database.items[index].importance);
    }
}

void AP_OADatabase::database_items_remove_all_expired()
{
    // calculate age of all items in the _database

    if (_database_expiry_seconds <= 0) {
        // zero means never expire. This is not normal behavior but perhaps you could send a static
        // environment once that you don't want to have to constantly update
        return;
    }

    const uint32_t now_ms = AP_HAL::millis();
    const uint32_t expiry_ms = (uint32_t)_database_expiry_seconds * 1000;
    uint16_t index = 0;
    while (index < _database.count) {
        if (now_ms - _database.items[index].timestamp_ms > expiry_ms) {
            database_item_remove(index);
        } else {
            index++;
        }
    }
}

// send ADSB_VEHICLE mavlink messages
void AP_OADatabase::send_adsb_vehicle(mavlink_channel_t chan, uint16_t interval_ms)
{
    // ensure database's send_to_gcs field is large enough
    static_assert(MAVLINK_COMM_NUM_BUFFERS <= sizeof(OA_DbItem::send_to_gcs) * 8,
                  "AP_OADatabase's OA_DBItem.send_to_gcs bitmask must be large enough to hold MAVLINK_COMM_NUM_BUFFERS");

    if ((_output_level <= OutputLevel::NONE) || !healthy()) {
        return;
    }

    const uint8_t chan_as_bitmask = 1 << chan;
    const char callsign[9] = "OA_DB";

    // calculate how many messages we should send
    const uint32_t now_ms = AP_HAL::millis();
    uint16_t num_to_send = 1;
    uint16_t num_sent = 0;
    if ((_last_send_to_gcs_ms[chan] != 0) && (interval_ms > 0)) {
        uint32_t diff_ms = now_ms - _last_send_to_gcs_ms[chan];
        num_to_send = MAX(diff_ms / interval_ms, 1U);
    }
    _last_send_to_gcs_ms[chan] = now_ms;

    // send unsent objects until output buffer is full or have sent enough
    for (uint16_t i=0; i < _database.count; i++) {
        if (!HAVE_PAYLOAD_SPACE(chan, ADSB_VEHICLE) || (num_sent >= num_to_send)) {
            // all done for now
            return;
        }

        const uint16_t idx = _next_index_to_send[chan];

        // prepare to send next object
        _next_index_to_send[chan]++;
        if (_next_index_to_send[chan] >= _database.count) {
            _next_index_to_send[chan] = 0;
        }

        if ((_database.items[idx].send_to_gcs & chan_as_bitmask) == 0) {
            continue;
        }

        // convert object's position as an offset from EKF origin to Location
        const Location item_loc(Vector3f(_database.items[idx].pos.x * 100.0f, _database.items[idx].pos.y * 100.0f, _database.items[idx].pos.z * 100.0f), Location::AltFrame::ABOVE_ORIGIN);

        mavlink_msg_adsb_vehicle_send(chan,
            idx,
            item_loc.lat,
            item_loc.lng,
            0,                          // altitude_type
            item_loc.alt,               
            0,                          // heading
            0,                          // hor_velocity
            0,                          // ver_velocity
            callsign,                   // callsign
            255,                        // emitter_type
            0,                          // tslc
            0,                          // flags
            (uint16_t)(_database.items[idx].radius * 100.f));   // squawk

        // unmark item for sending to gcs
        _database.items[idx].send_to_gcs &= ~chan_as_bitmask;

        // update highest index sent to GCS
        _highest_index_sent[chan] = MAX(idx, _highest_index_sent[chan]);

        // update count sent
        num_sent++;
    }

    // clear expired items in case the database size shrank
    while (_highest_index_sent[chan] > _database.count) {
        if (!HAVE_PAYLOAD_SPACE(chan, ADSB_VEHICLE) || (num_sent >= num_to_send)) {
            // all done for now
            return;
        }

        const uint16_t idx = _highest_index_sent[chan];
        _highest_index_sent[chan]--;

        if (_database.items[idx].importance != OA_DbItemImportance::High) {
            continue;
        }

        mavlink_msg_adsb_vehicle_send(chan,
            idx,        // id
            0,          // latitude
            0,          // longitude
            0,          // altitude_type
            0,          // altitude
            0,          // heading
            0,          // hor_velocity
            0,          // ver_velocity
            callsign,   // callsign
            255,        // emitter_type
            0,          // tslc
            0,          // flags
            0);         // squawk

        // update count sent
        num_sent++;
    }
}

// returns the next new object id
uint8_t AP_OADatabase::get_next_object_id()
{
    // find lowest object id with no items
    for (uint8_t i=0; i<ARRAY_SIZE(_object_item_count); i++) {
        if (_object_item_count[i] == 0) {
            return i;
        }
    }
    // if we got this far then we've run out of object ids
    return UINT8_MAX;
}

// find earth-frame yaw angle to largest object
// veh_pos should be the vehicle's horiztonal position in meters offset from the EKF origin
// returns true on success and fills in yaw_to_object_rad
bool AP_OADatabase::dir_to_largest_object(const Vector2f &veh_posxy, float& yaw_to_object_rad) const
{
    // find object with the most items
    uint8_t largest_object_id = 0;
    for (uint8_t i=1; i<ARRAY_SIZE(_object_item_count); i++) {
        if (_object_item_count[i] > _object_item_count[largest_object_id]) {
            largest_object_id = i;
        }
    }

    // return failure if no objects found
    if (_object_item_count[largest_object_id] == 0) {
        gcs().send_text(MAV_SEVERITY_INFO, "dtlo: zero objects");
        return false;
    }
    gcs().send_text(MAV_SEVERITY_INFO, "dtlo: obj:%u cnt:%u", (unsigned)largest_object_id, (unsigned)_object_item_count[largest_object_id]);

    // calculate average position of items in object
    Vector2f posxy_sum;
    uint16_t posxy_sum_count = 0;
    for (uint16_t i=0; i<_database.count; i++) {
        if (_database.items[i].object_id == largest_object_id) {
            posxy_sum += _database.items[i].pos.xy();
            posxy_sum_count++;
        }
    }
    if (posxy_sum_count == 0) {
        // this should never happen because we have already checked above
        return false;
    }
    posxy_sum /= posxy_sum_count;

    // calculate angle to average position
    Vector2f posxy_diff = posxy_sum - veh_posxy;
    if (posxy_diff.is_zero()) {
        // catch unlikely case that average position is exactly on vehicle
        return false;
    }
    yaw_to_object_rad = posxy_diff.angle();
    return true;
}

// singleton instance
AP_OADatabase *AP_OADatabase::_singleton;

namespace AP {
AP_OADatabase *oadatabase()
{
    return AP_OADatabase::get_singleton();
}

}
