/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

/*
 * control_stabilize.pde - init and run calls for acro flight mode
 */

// acro_init - initialise acro controller
static bool acro_init(bool ignore_checks)
{
    return true;
}

// acro_run - runs the acro controller
// should be called at 100hz or more
static void acro_run()
{
    int16_t target_roll, target_pitch, target_yaw;

    // convert the input to the desired body frame rate
    get_pilot_desired_angle_rates(g.rc_1.control_in, g.rc_2.control_in, g.rc_4.control_in, target_roll, target_pitch, target_yaw);

    attitude_control.ratebf_rpy(target_roll, target_pitch, target_yaw);

    // To-Do: handle helicopter

    // pilot controlled yaw using rate controller
    //get_yaw_rate_stabilized_bf(pilot_yaw);

    // call get_acro_level_rates() here?

    // To-Do: convert body-frame stabilized angles to earth frame angles and update controll_roll, pitch and yaw?

    // body-frame rate controller is run directly from 100hz loop
}


// get_pilot_desired_angle_rates - transform pilot's roll pitch and yaw input into a desired lean angle rates
// returns desired angle rates in centi-degrees-per-second
static void get_pilot_desired_angle_rates(int16_t roll_in, int16_t pitch_in, int16_t yaw_in, int16_t &roll_out, int16_t &pitch_out, int16_t &yaw_out)
{

    // Calculate trainer mode earth frame rate command for roll
    int32_t target_rate;
    Vector3f rate_ef_level, rate_bf_level, rate_bf_request;

    // calculate rate requests
    rate_bf_request.x = roll_in * g.acro_rp_p;
    rate_bf_request.y = pitch_in * g.acro_rp_p;
    rate_bf_request.z = yaw_in * g.acro_yaw_p;
    // todo: add acceleration slew

    // calculate earth frame rate corrections to pull the copter back to level while in ACRO mode

    acro_level_mix = constrain_float(1-max(max(abs(roll_in), abs(pitch_in)), abs(yaw_in))/4500.0, 0, 1)*cos_pitch_x;

    // Calculate trainer mode earth frame rate command for roll
    int32_t roll_angle = wrap_180_cd(ahrs.roll_sensor);
    target_rate = 0;

    if (g.acro_trainer == ACRO_TRAINER_LIMITED) {
        if (roll_angle > aparm.angle_max){
            target_rate =  g.pi_stabilize_roll.get_p(aparm.angle_max-roll_angle);
        }else if (roll_angle < -aparm.angle_max) {
            target_rate =  g.pi_stabilize_roll.get_p(-aparm.angle_max-roll_angle);
        }
    }
    roll_angle   = constrain_int32(roll_angle, -ACRO_LEVEL_MAX_ANGLE, ACRO_LEVEL_MAX_ANGLE);
    rate_ef_level.x -= roll_angle * g.acro_balance_roll;

    // Calculate trainer mode earth frame rate command for pitch
    int32_t pitch_angle = wrap_180_cd(ahrs.pitch_sensor);
    target_rate = 0;

    if (g.acro_trainer == ACRO_TRAINER_LIMITED) {
        if (pitch_angle > aparm.angle_max){
            target_rate =  g.pi_stabilize_pitch.get_p(aparm.angle_max-pitch_angle);
        }else if (pitch_angle < -aparm.angle_max) {
            target_rate =  g.pi_stabilize_pitch.get_p(-aparm.angle_max-pitch_angle);
        }
    }
    pitch_angle  = constrain_int32(pitch_angle, -ACRO_LEVEL_MAX_ANGLE, ACRO_LEVEL_MAX_ANGLE);
    rate_ef_level.y -= pitch_angle * g.acro_balance_pitch;

    // Calculate trainer mode earth frame rate command for yaw
    rate_ef_level.z = 0;

    // convert earth-frame level rates to body-frame level rates
    attitude_control.rate_ef_targets_to_bf(rate_ef_level, rate_bf_level);


    // combine earth frame rate corrections with rate requests

    // combine bf roll leveling with requested bf roll rate
    if (g.acro_trainer == ACRO_TRAINER_LIMITED) {
        rate_bf_request.x += rate_bf_level.x;
    }else{
        // Scale pitch leveling by stick input
        rate_bf_level.x = (float)rate_bf_level.x*acro_level_mix;

        // Calculate rate limit to prevent change of rate through inverted
        int32_t rate_limit = labs(labs(rate_bf_request.x)-labs(rate_bf_level.x));

        rate_bf_request.x += acro_roll_rate;
        rate_bf_request.x = constrain_int32(rate_bf_request.x, -rate_limit, rate_limit);
    }

    // combine bf pitch leveling with requested bf pitch rate
    if (g.acro_trainer == ACRO_TRAINER_LIMITED) {
        rate_bf_request.y += rate_bf_level.y;
    }else{
        // Scale pitch leveling by stick input
        rate_bf_level.y = (float)rate_bf_level.y*acro_level_mix;

        // Calculate rate limit to prevent change of rate through inverted
        int32_t rate_limit = labs(labs(rate_bf_request.y)-labs(rate_bf_level.y));

        rate_bf_request.y += acro_roll_rate;
        rate_bf_request.y = constrain_int32(rate_bf_request.y, -rate_limit, rate_limit);
    }

    // combine bf yaw leveling with requested bf yaw rate
    if (g.acro_trainer == ACRO_TRAINER_LIMITED) {
        rate_bf_request.z += rate_bf_level.z;
    }else{
        // Scale pitch leveling by stick input
        rate_bf_level.z = (float)rate_bf_level.z*acro_level_mix;

        // Calculate rate limit to prevent change of rate through inverted
        int32_t rate_limit = labs(labs(rate_bf_request.z)-labs(rate_bf_level.z));

        rate_bf_request.z += acro_roll_rate;
        rate_bf_request.z = constrain_int32(rate_bf_request.z, -rate_limit, rate_limit);
    }

    // hand back rate request
    roll_out = rate_bf_request.x;
    pitch_out = rate_bf_request.y;
    yaw_out = rate_bf_request.z;
}
