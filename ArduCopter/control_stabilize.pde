/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

// acro_run - runs the acro controller
// should be called at 100hz or more
static void acro_run()
{
    Vector3f rate_target;          // for roll, pitch, yaw body-frame rate targets

    // convert the input to the desired body frame rate
    rate_target.x = g.rc_1.control_in * g.acro_rp_p;
    rate_target.y = g.rc_2.control_in * g.acro_rp_p;
    rate_target.z = g.rc_4.control_in * g.acro_yaw_p;

    // To-Do: handle acro trainer here?

    // To-Do: handle helicopter

    acro_level_mix = constrain_float(1-max(max(abs(g.rc_1.control_in), abs(g.rc_2.control_in)), abs(g.rc_4.control_in))/4500.0, 0, 1)*cos_pitch_x;

    // set targets for body frame rate controller
    attitude_control.rate_stab_bf_targets(rate_target);

    // convert stabilize rates to regular rates
    // To-Do: replace G_Dt below
    attitude_control.rate_stab_bf_to_rate_bf_roll(G_Dt);
    attitude_control.rate_stab_bf_to_rate_bf_pitch(G_Dt);
    attitude_control.rate_stab_bf_to_rate_bf_yaw(G_Dt);

    // call get_acro_level_rates() here?

    // To-Do: convert body-frame stabilized angles to earth frame angles and update controll_roll, pitch and yaw?

    // body-frame rate controller is run directly from 100hz loop
}

// stabilize_run - runs the main stabilize controller
// should be called at 100hz or more
static void stabilize_run()
{
    Vector3f angle_target;          // for roll and pitch angle targets
    Vector3f rate_stab_ef_target;    // for yaw rate target
    int16_t target_roll, target_pitch;

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

    // copy latest output from nav controller to stabilize controller
    control_roll = wp_nav.get_desired_roll();
    control_pitch = wp_nav.get_desired_pitch();

    // copy angle targets for reporting purposes
    angle_target.x = control_roll;
    angle_target.y = control_pitch;

    // To-Do: handle pilot input for yaw and different methods to update yaw (ROI, face next wp)
    angle_target.z = control_yaw;

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