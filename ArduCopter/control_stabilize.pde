/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

/*
 * control_stabilize.pde - init and run calls for stabilize flight mode
 */

// stabilize_init - initialise stabilize controller
static bool stabilize_init(bool ignore_checks)
{
    // set target altitude to zero for reporting
    // To-Do: make pos controller aware when it's active/inactive so it can always report the altitude error?
    pos_control.set_alt_target(0);
    return true;
}

// stabilize_run - runs the main stabilize controller
// should be called at 100hz or more
static void stabilize_run()
{
    int16_t target_roll, target_pitch;
    float target_yaw_rate;
    int16_t pilot_throttle_scaled;

    // if not armed or throttle at zero, set throttle to zero and exit immediately
    if(!motors.armed() || g.rc_3.control_in <= 0) {
        attitude_control.init_targets();
        attitude_control.set_throttle_out(0, false);
        return;
    }

    // apply SIMPLE mode transform to pilot inputs
    update_simple_mode();

    // convert pilot input to lean angles
    // To-Do: convert get_pilot_desired_lean_angles to return angles as floats
    get_pilot_desired_lean_angles(g.rc_1.control_in, g.rc_2.control_in, target_roll, target_pitch);

    // get pilot's desired yaw rate
    target_yaw_rate = get_pilot_desired_yaw_rate(g.rc_4.control_in);

    // get pilot's desired throttle
    pilot_throttle_scaled = get_pilot_desired_throttle(g.rc_3.control_in);

    // reset target lean angles and heading while landed
    if (ap.land_complete) {
        attitude_control.init_targets();
    }else{
        // call attitude controller
        attitude_control.angleef_rp_rateef_y(target_roll, target_pitch, target_yaw_rate);

        // body-frame rate controller is run directly from 100hz loop
    }

    // output pilot's throttle
    attitude_control.set_throttle_out(pilot_throttle_scaled, true);

    // refetch angle targets for reporting
    const Vector3f angle_target = attitude_control.angle_ef_targets();
    control_roll = angle_target.x;
    control_pitch = angle_target.y;
    control_yaw = angle_target.z;

    // update estimate of throttle cruise
    #if FRAME_CONFIG == HELI_FRAME
    update_throttle_cruise(motors.get_collective_out());
    #else
    update_throttle_cruise(pilot_throttle_scaled);
    #endif  //HELI_FRAME

    // update take-off complete flag
    if (!ap.takeoff_complete) {
        if (pilot_throttle_scaled > g.throttle_cruise) {
            // we must be in the air by now
            set_takeoff_complete(true);
        }
    }
}
