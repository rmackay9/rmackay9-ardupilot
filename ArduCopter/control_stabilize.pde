/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

// acro_run - runs the acro controller
// should be called at 100hz or more
static void acro_run()
{
}

// stabilize_run - runs the main stabilize controller
// should be called at 100hz or more
static void stabilize_run()
{
    Vector3f angle_target;          // for roll and pitch angle targets
    Vector3f rate_stab_ef_target;    // for yaw rate target
    int16_t target_roll, target_pitch;

    // debug -- remove me!
    cliSerial->printf_P(PSTR("\nstabilize_run!"));

    // apply SIMPLE mode transform to pilot inputs
    update_simple_mode();

    // convert pilot input to lean angles
    // To-Do: convert get_pilot_desired_lean_angles to return angles as floats
    get_pilot_desired_lean_angles(g.rc_1.control_in, g.rc_2.control_in, target_roll, target_pitch);
    angle_target.x = target_roll;
    angle_target.y = target_pitch;

    // set earth-frame angular targets
    attitude_control.angle_ef_targets(angle_target);

    // convert earth-frame angle targets to earth-frame rate targets
    attitude_control.angle_to_rate_ef_roll();
    attitude_control.angle_to_rate_ef_pitch();

    // get pilot's desired yaw rate
    rate_stab_ef_target.z = get_pilot_desired_yaw_rate(g.rc_4.control_in);

    // set earth-frame rate stabilize target for yaw with pilot's desired yaw
    // To-Do: this is quite wasteful to update the entire target vector when only yaw is used
    attitude_control.rate_stab_ef_targets(rate_stab_ef_target);

    // convert earth-frame stabilize rate to regular rate target
    // To-Do: replace G_Dt below
    attitude_control.rate_stab_ef_to_rate_ef_yaw(G_Dt);

    // convert earth-frame rates to body-frame rates
    attitude_control.rate_ef_targets_to_bf();

    // body-frame rate controller is run directly from 100hz loop

    // To-Do: add throttle control for stabilize mode here?
}

// althold_run - runs the althold controller
// should be called at 100hz or more
static void althold_run()
{
}

// auto_run - runs the auto controller
// should be called at 100hz or more
static void auto_run()
{
    Vector3f angle_target;

    // user input, although ignored is put into control_roll and pitch for reporting purposes
    control_roll = g.rc_1.control_in;
    control_pitch = g.rc_2.control_in;

    // copy latest output from nav controller to stabilize controller
    angle_target.x = wp_nav.get_desired_roll();
    angle_target.y = wp_nav.get_desired_pitch();

    // To-Do: handle yaw
    angle_target.z = control_yaw;

    // copy angle targets for reporting purposes
    control_roll = angle_target.x;
    control_pitch = angle_target.y;

    // To-Do: shorten below by moving these often used steps into a single function in the AC_AttitudeControl lib

    // set earth-frame angular targets
    attitude_control.angle_ef_targets(angle_target);

    // convert earth-frame angle targets to earth-frame rate targets
    attitude_control.angle_to_rate_ef_roll();
    attitude_control.angle_to_rate_ef_pitch();
    attitude_control.angle_to_rate_ef_yaw();

    // convert earth-frame rates to body-frame rates
    attitude_control.rate_ef_targets_to_bf();

    // body-frame rate controller is run directly from 100hz loop
}

// circle_run - runs the circle controller
// should be called at 100hz or more
static void circle_run()
{
}

// loiter_run - runs the loiter controller
// should be called at 100hz or more
static void loiter_run()
{
}

// guided_run - runs the guided controller
// should be called at 100hz or more
static void guided_run()
{
}

// land_run - runs the land controller
// should be called at 100hz or more
static void land_run()
{
    verify_land();
}

// rtl_run - runs the return-to-launch controller
// should be called at 100hz or more
static void rtl_run()
{
    verify_RTL();
}

// ofloiter_run - runs the optical flow loiter controller
// should be called at 100hz or more
static void ofloiter_run()
{
}

// drift_run - runs the drift controller
// should be called at 100hz or more
static void drift_run()
{
}

// sport_run - runs the sport controller
// should be called at 100hz or more
static void sport_run()
{
}