/// @file    AP_Mission_ChangeDetector_Copter.cpp
/// @brief   Detects changes in the next few nav commands in the mission

#include "AP_Mission_ChangeDetector_Copter.h"

extern const AP_HAL::HAL& hal;

// check for changes to mission. returns required response to change (if any)
// using_next_command should be set to try if waypoint controller is already using the next navigation command
AP_Mission_ChangeDetector_Copter::ChangeResponseType AP_Mission_ChangeDetector_Copter::check_for_mission_change(bool using_next_command)
{
    // take backup of command list
    MissionCommandList cmd_list_bak = mis_change_detect;

    uint8_t first_changed_cmd_idx = 0;
    if (!AP_Mission_ChangeDetector::check_for_mission_change(first_changed_cmd_idx)) {
        // the mission has not changed
        return AP_Mission_ChangeDetector_Copter::ChangeResponseType::NONE;
    }

    // if the current command has change a reset is always required
    // ToDo: check this handles mission erased
    if (first_changed_cmd_idx == 0) {
        return AP_Mission_ChangeDetector_Copter::ChangeResponseType::RESET_REQUIRED;
    }

    // 2nd or 3rd command has been added, changed or deleted

    // if 1st segment (wp or spline) has pause then return NONE
    if (((mis_change_detect.cmd[0].id == MAV_CMD_NAV_WAYPOINT) || (mis_change_detect.cmd[0].id == MAV_CMD_NAV_SPLINE_WAYPOINT)) &&
        (mis_change_detect.cmd[0].p1 > 0)) {
        return AP_Mission_ChangeDetector_Copter::ChangeResponseType::NONE;
    }

    // if !using_next_command && 2nd cmd has changed and is wp then return NONE
    // ToDo: check 2nd waypoint hasn't been deleted
    if (!using_next_command && (mis_change_detect.cmd[1].id == MAV_CMD_NAV_WAYPOINT)) {
        return AP_Mission_ChangeDetector_Copter::ChangeResponseType::NONE;
    }

    // if 2nd or 3rd cmd has changed and 2nd is spline then RESET_REQUIRED
    // ToDo: check 2nd waypoint hasn't been deleted
    if (mis_change_detect.cmd[1].id == MAV_CMD_NAV_SPLINE_WAYPOINT) {
        return AP_Mission_ChangeDetector_Copter::ChangeResponseType::RESET_REQUIRED;
    }

    // if 1st is a spline (without a pause), if add of 2nd then RESET_REQUIRED
    if (mis_change_detect.cmd[1].id == MAV_CMD_NAV_SPLINE_WAYPOINT && (mis_change_detect.cmd[1].p1 > 0)) {
        return AP_Mission_ChangeDetector_Copter::ChangeResponseType::RESET_REQUIRED;
    }

    // if 2nd is a spline (without a pause), if add of 3rd then RESET_REQUIRED

    // if 1st is wp (without a pause), if add of 2nd then ADD_NEXT_WAYPOINT

    if (using_next_command) {
        //
    } else {
        // not using next command

        // if we only had one command before, a new command must have been added
        if (cmd_list_bak.cmd_count == 1) {
            return AP_Mission_ChangeDetector_Copter::ChangeResponseType::ADD_NEXT_WAYPOINT;
        }

        // 2nd or 3rd command was changed but we are not using it so no action required
        return AP_Mission_ChangeDetector_Copter::ChangeResponseType::NONE;
    }

    // next command is being used and it has changed so reset required
    return AP_Mission_ChangeDetector_Copter::ChangeResponseType::RESET_REQUIRED;
}

