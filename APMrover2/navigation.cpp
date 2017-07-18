#include "Rover.h"

//****************************************************************
// Function that will calculate the desired direction to fly and distance
//****************************************************************
void Rover::navigate()
{
    // do not navigate with corrupt data
    // ---------------------------------
    if (!have_position) {
        return;
    }

    if ((next_WP.lat == 0 && next_WP.lng == 0) || (home_is_set == HOME_UNSET)) {
        return;
    }

    // waypoint distance from rover
    // ----------------------------
    wp_distance = get_distance(current_loc, next_WP);

    // control mode specific updates to nav_bearing
    // --------------------------------------------
    update_navigation();
}

void Rover::restart_nav()
{
    g.pidSpeedThrottle.reset_I();
    prev_WP = current_loc;
    mission.start_or_resume();
}

bool Rover::verify_nav_wp()
{
    if (wp_distance <= g.waypoint_radius) {
        gcs().send_text(MAV_SEVERITY_INFO, "Reached destination");
        nav_wp_complete = true;
        return true;
    }

    // have we gone past the waypoint?
    if (location_passed_point(current_loc, prev_WP, next_WP)) {
        gcs().send_text(MAV_SEVERITY_INFO, "Reached destination. Distance away %dm",
                        static_cast<int32_t>(fabsf(get_distance(current_loc, next_WP))));
        nav_wp_complete = true;
        return true;
    }

    return false;
}

void Rover::nav_set_yaw_speed()
{
    // if we haven't received a MAV_CMD_NAV_SET_YAW_SPEED message within the last 3 seconds bring the rover to a halt
    if ((millis() - guided_control.msg_time_ms) > 3000) {
        gcs().send_text(MAV_SEVERITY_WARNING, "NAV_SET_YAW_SPEED not recvd last 3secs, stopping");
        g2.motors.set_throttle(g.throttle_min.get());
        g2.motors.set_steering(0.0f);
        return;
    }

    const int32_t steering = steerController.get_steering_out_angle_error(guided_control.turn_angle);
    g2.motors.set_steering(steering);

    // speed param in the message gives speed as a proportion of cruise speed.
    // 0.5 would set speed to the cruise speed
    // 1 is double the cruise speed.
    const float target_speed = g.speed_cruise * guided_control.target_speed * 2.0f;
    rover.control_mode->calc_throttle(target_speed);

    Log_Write_GuidedTarget(rover.mode_guided.guided_mode, Vector3f(steering, 0.0f, 0.0f), Vector3f(target_speed, 0.0f, 0.0f));
}

void Rover::nav_set_speed()
{
    // if we haven't received a MAVLINK_MSG_ID_SET_POSITION_TARGET_LOCAL_NED message within the last 3 seconds bring the rover to a halt
    if ((millis() - guided_control.msg_time_ms) > 3000) {
        gcs().send_text(MAV_SEVERITY_WARNING, "SET_VELOCITY not recvd last 3secs, stopping");
        g2.motors.set_throttle(g.throttle_min.get());
        g2.motors.set_steering(0.0f);
        prev_WP = current_loc;
        next_WP = current_loc;
        set_guided_WP(current_loc);  // exit Guided_Velocity to prevent spam
        return;
    }
    prev_WP = current_loc;
    next_WP = current_loc;

    const int32_t steer_value = steerController.get_steering_out_rate(guided_control.target_steer_speed);
    location_update(next_WP, (steer_value + ahrs.yaw_sensor) * 0.01f, 4.0f);  // put the next wp at 4m forward at steer direction
    nav_controller->update_waypoint(current_loc, next_WP);

    g2.motors.set_steering(steer_value);
    rover.control_mode->calc_throttle(guided_control.target_speed);

    Log_Write_GuidedTarget(rover.mode_guided.guided_mode, Vector3f(steer_value, 0.0f, 0.0f), Vector3f(guided_control.target_speed, 0.0f, 0.0f));
}
