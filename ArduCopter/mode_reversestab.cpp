#include "Copter.h"

ModeReverseStab::ModeReverseStab(void) : Mode()
{
}

bool ModeReverseStab::init(bool ignore_checks)
{
    return true;
}

void ModeReverseStab::run()
{
    float target_roll, target_pitch;
    float target_yaw_rate;
    float pilot_throttle_scaled;

    // process pilot inputs
    if (!copter.failsafe.radio) {
        // apply SIMPLE mode transform to pilot inputs
        update_simple_mode();

        // get pilot's desired throttle
        pilot_throttle_scaled = copter.get_pilot_desired_throttle();

        // Reverse roll and pitch inputs
        target_roll = -channel_roll->get_control_in();
        target_pitch = -channel_pitch->get_control_in();

        // get pilot's desired yaw rate
        target_yaw_rate = get_pilot_desired_yaw_rate(channel_yaw->get_control_in());

        // adjust targets for angle error
        attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(target_roll, target_pitch, target_yaw_rate);

        // output pilot's throttle
        attitude_control->set_throttle_out(pilot_throttle_scaled, true, g.throttle_filt);
    } else {
        // clear out pilot desired acceleration in case radio failsafe event occurs and we do not switch to RTL for some reason
        copter.pilot_thrust_z = 0.0f;
    }
}
