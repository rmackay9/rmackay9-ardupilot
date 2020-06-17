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
#include <AP_Math/AP_Math.h>
#include <AP_Baro/AP_Baro.h>
#include <AP_Beacon/AP_Beacon.h>
#include <AP_Compass/AP_Compass.h>
#include <AP_GPS/AP_GPS.h>
#include <AP_OpticalFlow/AP_OpticalFlow.h>
#include <AP_RangeFinder/AP_RangeFinder.h>
#include <AP_VisualOdom/AP_VisualOdom.h>
#include "AP_NavEKF_Source.h"

extern const AP_HAL::HAL& hal;

const AP_Param::GroupInfo AP_NavEKF_Source::var_info[] = {

    // @Param: POSXY
    // @DisplayName: Position Horizontal Source (Primary)
    // @Description: Position Horizontal Source (Primary)
    // @Values: 0:None, 3:GPS, 4:Beacon, 6:ExternalNav
    // @User: Advanced
    AP_GROUPINFO("POSXY", 1, AP_NavEKF_Source, _source[0].posxy, (int8_t)AP_NavEKF_Source::SourceXY::GPS),

    // @Param: VELXY
    // @DisplayName: Velocity Horizontal Source
    // @Description: Velocity Horizontal Source
    // @Values: 0:None, 3:GPS, 4:Beacon, 5:OpticalFlow, 6:ExternalNav, 7:WheelEncoder
    // @User: Advanced
    AP_GROUPINFO("VELXY", 2, AP_NavEKF_Source, _source[0].velxy, (int8_t)AP_NavEKF_Source::SourceXY::GPS),

    // @Param: POSZ
    // @DisplayName: Position Vertical Source
    // @Description: Position Vertical Source
    // @Values: 0:None, 1:Baro, 2:RangeFinder, 3:GPS, 4:Beacon, 6:ExternalNav
    // @User: Advanced
    AP_GROUPINFO("POSZ", 3, AP_NavEKF_Source, _source[0].posz, (int8_t)AP_NavEKF_Source::SourceZ::BARO),

    // @Param: VELZ
    // @DisplayName: Velocity Vertical Source
    // @Description: Velocity Vertical Source
    // @Values: 0:None, 3:GPS, 4:Beacon, 6:ExternalNav
    // @User: Advanced
    AP_GROUPINFO("VELZ", 4, AP_NavEKF_Source, _source[0].velz, (int8_t)AP_NavEKF_Source::SourceZ::GPS),

    // @Param: YAW
    // @DisplayName: Yaw Source
    // @Description: Yaw Source
    // @Values: 0:None, 1:Compass, 2:External, 3:External with Compass Fallback
    // @User: Advanced
    AP_GROUPINFO("YAW", 5, AP_NavEKF_Source, _source[0].yaw, (int8_t)AP_NavEKF_Source::SourceYaw::COMPASS),

#if AP_NAKEKF_SOURCE_COUNT >= 2
    // @Param: POSXY2
    // @DisplayName: Position Horizontal Source (Secondary)
    // @Description: Position Horizontal Source (Secondary)
    // @Values: 0:None, 3:GPS, 4:Beacon, 6:ExternalNav
    // @User: Advanced
    AP_GROUPINFO("POSXY2", 6, AP_NavEKF_Source, _source[1].posxy, (int8_t)AP_NavEKF_Source::SourceXY::NONE),

    // @Param: VELXY2
    // @DisplayName: Velocity Horizontal Source (Secondary)
    // @Description: Velocity Horizontal Source (Secondary)
    // @Values: 0:None, 3:GPS, 4:Beacon, 5:OpticalFlow, 6:ExternalNav, 7:WheelEncoder
    // @User: Advanced
    AP_GROUPINFO("VELXY2", 7, AP_NavEKF_Source, _source[1].velxy, (int8_t)AP_NavEKF_Source::SourceXY::NONE),

    // @Param: POSZ2
    // @DisplayName: Position Vertical Source (Secondary)
    // @Description: Position Vertical Source (Secondary)
    // @Values: 0:None, 1:Baro, 2:RangeFinder, 3:GPS, 4:Beacon, 6:ExternalNav
    // @User: Advanced
    AP_GROUPINFO("POSZ2", 8, AP_NavEKF_Source, _source[1].posz, (int8_t)AP_NavEKF_Source::SourceZ::BARO),

    // @Param: VELZ2
    // @DisplayName: Velocity Vertical Source (Secondary)
    // @Description: Velocity Vertical Source (Secondary)
    // @Values: 0:None, 3:GPS, 4:Beacon, 6:ExternalNav
    // @User: Advanced
    AP_GROUPINFO("VELZ2", 9, AP_NavEKF_Source, _source[1].velz, (int8_t)AP_NavEKF_Source::SourceZ::NONE),

    // @Param: YAW2
    // @DisplayName: Yaw Source (Secondary)
    // @Description: Yaw Source (Secondary)
    // @Values: 0:None, 1:Compass, 2:External, 3:External with Compass Fallback
    // @User: Advanced
    AP_GROUPINFO("YAW2", 10, AP_NavEKF_Source, _source[1].yaw, (int8_t)AP_NavEKF_Source::SourceYaw::COMPASS),
#endif

#if AP_NAKEKF_SOURCE_COUNT >= 3
    // @Param: POSXY3
    // @DisplayName: Position Horizontal Source (Tertiary)
    // @Description: Position Horizontal Source (Tertiary)
    // @Values: 0:None, 3:GPS, 4:Beacon, 6:ExternalNav
    // @User: Advanced
    AP_GROUPINFO("POSXY3", 11, AP_NavEKF_Source, _source[2].posxy, (int8_t)AP_NavEKF_Source::SourceXY::NONE),

    // @Param: VELXY3
    // @DisplayName: Velocity Horizontal Source (Tertiary)
    // @Description: Velocity Horizontal Source (Tertiary)
    // @Values: 0:None, 3:GPS, 4:Beacon, 5:OpticalFlow, 6:ExternalNav, 7:WheelEncoder
    // @User: Advanced
    AP_GROUPINFO("VELXY3", 12, AP_NavEKF_Source, _source[2].velxy, (int8_t)AP_NavEKF_Source::SourceXY::NONE),

    // @Param: POSZ3
    // @DisplayName: Position Vertical Source (Tertiary)
    // @Description: Position Vertical Source (Tertiary)
    // @Values: 0:None, 1:Baro, 2:RangeFinder, 3:GPS, 4:Beacon, 6:ExternalNav
    // @User: Advanced
    AP_GROUPINFO("POSZ3", 13, AP_NavEKF_Source, _source[2].posz, (int8_t)AP_NavEKF_Source::SourceZ::BARO),

    // @Param: VELZ3
    // @DisplayName: Velocity Vertical Source (Tertiary)
    // @Description: Velocity Vertical Source (Tertiary)
    // @Values: 0:None, 3:GPS, 4:Beacon, 6:ExternalNav
    // @User: Advanced
    AP_GROUPINFO("VELZ3", 14, AP_NavEKF_Source, _source[2].velz, (int8_t)AP_NavEKF_Source::SourceZ::NONE),

    // @Param: YAW3
    // @DisplayName: Yaw Source (Tertiary)
    // @Description: Yaw Source (Tertiary)
    // @Values: 0:None, 1:Compass, 2:External, 3:External with Compass Fallback
    // @User: Advanced
    AP_GROUPINFO("YAW3", 15, AP_NavEKF_Source, _source[2].yaw, (int8_t)AP_NavEKF_Source::SourceYaw::COMPASS),
#endif

    AP_GROUPEND
};

AP_NavEKF_Source::AP_NavEKF_Source()
{
    AP_Param::setup_object_defaults(this, var_info);
}

void AP_NavEKF_Source::init()
{
    // ensure init is only run once
    if (_active_source.initialised) {
        return;
    }

    // initialise active sources
    _active_source.posxy = (SourceXY)_source[0].posxy.get();
    _active_source.velxy = (SourceXY)_source[0].velxy.get();
    _active_source.posz = (SourceZ)_source[0].posz.get();
    _active_source.velz = (SourceZ)_source[0].velz.get();
    _active_source.yaw = (SourceYaw)_source[0].yaw.get();

    _active_source.initialised = true;
}

// set position source to either 0=primary or 1=secondary
void AP_NavEKF_Source::setPosVelXYZSource(uint8_t source_idx)
{
    // ensure init has been run
    init();

    // sanity check source idx
    if (source_idx >= AP_NAKEKF_SOURCE_COUNT) {
        return;
    }

    _active_source.posxy = (SourceXY)_source[source_idx].posxy.get();
    _active_source.velxy = (SourceXY)_source[source_idx].velxy.get();
    _active_source.posz = (SourceZ)_source[source_idx].posz.get();
    _active_source.velz = (SourceZ)_source[source_idx].velz.get();
    _active_source.yaw = (SourceYaw)_source[source_idx].yaw.get();
}

// align position of inactive sources to ahrs
void AP_NavEKF_Source::align_inactive_sources()
{
    // align visual odometry to GPS
#if HAL_VISUALODOM_ENABLED
    bool posxy_could_use_extnav = false;
    bool posz_could_use_extnav = false;

    for (uint8_t i=0; i<AP_NAKEKF_SOURCE_COUNT; i++) {
        posxy_could_use_extnav |= (getPosXYSourceByIndex(i) == SourceXY::EXTNAV);
        posz_could_use_extnav |= (getPosZSourceByIndex(i) == SourceZ::EXTNAV);
    }

    const bool align_posxy = posxy_could_use_extnav && ((getPosXYSource() == SourceXY::GPS) || (getPosXYSource() == SourceXY::BEACON));
    const bool align_posz = posz_could_use_extnav &&
                            ((getPosZSource() == SourceZ::BARO) || (getPosZSource() == SourceZ::RANGEFINDER) ||
                             (getPosZSource() == SourceZ::GPS) || (getPosZSource() == SourceZ::BEACON));

    if (align_posxy || align_posz) {
        AP_VisualOdom *visual_odom = AP::visualodom();
        if ((visual_odom != nullptr) && visual_odom->enabled()) {
            if (align_posxy || align_posz) {
                visual_odom->align_position_to_ahrs(align_posxy, align_posz);
            }
        }
    }
#endif
}

// sensor specific helper functions
bool AP_NavEKF_Source::usingGPS() const
{
    return getPosXYSource() == SourceXY::GPS ||
           getPosZSource() == SourceZ::GPS ||
           getVelXYSource() == SourceXY::GPS ||
           getVelZSource() == SourceZ::GPS;
}

// true if some parameters have been configured (used during parameter conversion)
bool AP_NavEKF_Source::params_configured_in_storage() const
{
    return _source[0].posxy.configured_in_storage() ||
           _source[0].velxy.configured_in_storage() ||
           _source[0].posz.configured_in_storage() ||
           _source[0].velz.configured_in_storage() ||
           _source[0].yaw.configured_in_storage();
}

// returns false if we fail arming checks, in which case the buffer will be populated with a failure message
bool AP_NavEKF_Source::pre_arm_check(char *failure_msg, uint8_t failure_msg_len) const
{
    bool baro_required = false;
    bool beacon_required = false;
    bool compass_required = false;
    bool gps_required = false;
    bool rangefinder_required = false;
    bool visualodom_required = false;
    bool optflow_required = false;

    // string array for error messages
    const char* idx_str[AP_NAKEKF_SOURCE_COUNT] = {"", "2", "3"};

    // check source params are valid
    for (uint8_t i=0; i<AP_NAKEKF_SOURCE_COUNT; i++) {

        // check posxy
        switch ((SourceXY)_source[i].posxy.get()) {
        case SourceXY::NONE:
            break;
        case SourceXY::GPS:
            gps_required = true;
            break;
        case SourceXY::BEACON:
            beacon_required = true;
            break;
        case SourceXY::EXTNAV:
            visualodom_required = true;
            break;
        case SourceXY::OPTFLOW:
        case SourceXY::WHEEL_ENCODER:
        default:
            // invalid posxy value
            hal.util->snprintf(failure_msg, failure_msg_len, "Check EK3_SRC_POSXY%s", idx_str[i]);
            return false;
        }

        // check velxy
        switch ((SourceXY)_source[i].velxy.get()) {
        case SourceXY::NONE:
            break;
        case SourceXY::GPS:
            gps_required = true;
            break;
        case SourceXY::OPTFLOW:
            optflow_required = true;
            break;
        case SourceXY::EXTNAV:
            visualodom_required = true;
            break;
        case SourceXY::WHEEL_ENCODER:
            // ToDo: add wheelencoder_required and test below
            break;
        case SourceXY::BEACON:
        default:
            // invalid velxy value
            hal.util->snprintf(failure_msg, failure_msg_len, "Check EK3_SRC_VELXY%s", idx_str[i]);
            return false;
        }

        // check posz
        switch ((SourceZ)_source[i].posz.get()) {
        case SourceZ::BARO:
            baro_required = true;
            break;
        case SourceZ::RANGEFINDER:
            rangefinder_required = true;
            break;
        case SourceZ::GPS:
            gps_required = true;
            break;
        case SourceZ::BEACON:
            beacon_required = true;
            break;
        case SourceZ::EXTNAV:
            visualodom_required = true;
            break;
        case SourceZ::NONE:
        default:
            // invalid posz value
            hal.util->snprintf(failure_msg, failure_msg_len, "Check EK3_SRC_POSZ%s", idx_str[i]);
            return false;
        }

        // check velz
        switch ((SourceZ)_source[i].velz.get()) {
        case SourceZ::NONE:
            break;
        case SourceZ::GPS:
            gps_required = true;
            break;
        case SourceZ::EXTNAV:
            visualodom_required = true;
            break;
        case SourceZ::BARO:
        case SourceZ::RANGEFINDER:
        case SourceZ::BEACON:
        default:
            // invalid velz value
            hal.util->snprintf(failure_msg, failure_msg_len, "Check EK3_SRC_VELZ%s", idx_str[i]);
            return false;
        }

        // check yaw
        switch ((SourceYaw)_source[i].yaw.get()) {
        case SourceYaw::NONE:
        case SourceYaw::COMPASS:
        case SourceYaw::EXTERNAL:
        case SourceYaw::EXTERNAL_COMPASS_FALLBACK:
            // valid yaw value
            break;
        default:
            // invalid posz value
            hal.util->snprintf(failure_msg, failure_msg_len, "Check EK3_SRC_YAW%s", idx_str[i]);
            return false;
        }
    }

    // check all required sensor are available
    const char* ekf_requires_msg = "EK3_SRC requires %s";
    if (baro_required && (AP::baro().num_instances() == 0)) {
        hal.util->snprintf(failure_msg, failure_msg_len, ekf_requires_msg, "Baro");
        return false;
    }

    if (beacon_required && (AP::beacon() == nullptr || !AP::beacon()->enabled())) {
        hal.util->snprintf(failure_msg, failure_msg_len, ekf_requires_msg, "Beacon");
        return false;
    }

    if (compass_required && AP::compass().get_num_enabled() == 0) {
        hal.util->snprintf(failure_msg, failure_msg_len, ekf_requires_msg, "Compass");
        return false;
    }

    if (gps_required && (AP::gps().num_sensors() == 0)) {
        hal.util->snprintf(failure_msg, failure_msg_len, ekf_requires_msg, "GPS");
        return false;
    }

    if (optflow_required && (AP::opticalflow() == nullptr || !AP::opticalflow()->enabled())) {
        hal.util->snprintf(failure_msg, failure_msg_len, ekf_requires_msg, "OpticalFlow");
        return false;
    }

    if (rangefinder_required && (AP::rangefinder() == nullptr || AP::rangefinder()->num_sensors() == 0)) {
        hal.util->snprintf(failure_msg, failure_msg_len, ekf_requires_msg, "RangeFinder");
        return false;
    }

    if (visualodom_required) {
        bool visualodom_available = false;
#if HAL_VISUALODOM_ENABLED
        visualodom_available = (AP::visualodom() != nullptr) && AP::visualodom()->enabled();
#endif
        if (!visualodom_available) {
            hal.util->snprintf(failure_msg, failure_msg_len, ekf_requires_msg, "VisualOdom");
            return false;
        }
    }

    return true;
}

// get source by id
AP_NavEKF_Source::SourceXY AP_NavEKF_Source::getPosXYSourceByIndex(uint8_t idx) const
{
    if (idx >= AP_NAKEKF_SOURCE_COUNT) {
        return SourceXY::NONE;
    }
    return (SourceXY)_source[idx].posxy.get();
}

AP_NavEKF_Source::SourceZ AP_NavEKF_Source::getPosZSourceByIndex(uint8_t idx) const
{
    if (idx >= AP_NAKEKF_SOURCE_COUNT) {
        return SourceZ::NONE;
    }
    return (SourceZ)_source[idx].posz.get();
}

AP_NavEKF_Source::SourceXY AP_NavEKF_Source::getVelXYSourceByIndex(uint8_t idx) const
{
    if (idx >= AP_NAKEKF_SOURCE_COUNT) {
        return SourceXY::NONE;
    }
    return (SourceXY)_source[idx].velxy.get();
}

AP_NavEKF_Source::SourceZ AP_NavEKF_Source::getVelZSourceByIndex(uint8_t idx) const
{
    if (idx >= AP_NAKEKF_SOURCE_COUNT) {
        return SourceZ::NONE;
    }
    return (SourceZ)_source[idx].velz.get();
}

AP_NavEKF_Source::SourceYaw AP_NavEKF_Source::getYawSourceByIndex(uint8_t idx) const
{
    if (idx >= AP_NAKEKF_SOURCE_COUNT) {
        return SourceYaw::NONE;
    }
    return (SourceYaw)_source[idx].yaw.get();
}

