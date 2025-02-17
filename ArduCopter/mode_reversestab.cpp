#include "Copter.h"

bool ModeReverseStab::init(bool ignore_checks)
{
    return true;
}

void ModeReverseStab::run()
{
    float target_roll, target_pitch;
    float target_yaw_rate;
    float pilot_throttle_scaled;

    // apply SIMPLE mode transform to pilot inputs
    update_simple_mode();

    // get pilot's desired throttle
    pilot_throttle_scaled = get_pilot_desired_throttle();

    // Reverse roll and pitch inputs
    target_roll = -channel_roll->get_control_in();
    target_pitch = -channel_pitch->get_control_in();

    // get pilot's desired yaw rate
    target_yaw_rate = get_pilot_desired_yaw_rate();

    // adjust targets for angle error
    attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(target_roll, target_pitch, target_yaw_rate);

    // output pilot's throttle
    attitude_control->set_throttle_out(pilot_throttle_scaled, true, g.throttle_filt);
}
