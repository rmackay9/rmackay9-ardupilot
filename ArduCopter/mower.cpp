#include "Copter.h"

void Copter::mower_init()
{
    mower.amplitude_control = rc().find_channel_for_option(RC_Channel::AUX_FUNC::MOWER_AMPLITUDE);
    mower.period_control = rc().find_channel_for_option(RC_Channel::AUX_FUNC::MOWER_PERIOD);

    if (mower.amplitude_control != nullptr) {
        mower.amplitude_control->set_range(1000);
    }
    if (mower.period_control != nullptr) {
        mower.period_control->set_range(1000);
    }
}

// update mower features.  Should be called from ModeLoiter::run()
void Copter::mower_update()
{
    // exit immediately if disarmed or landed
    if (!motors->armed() || !ap.auto_armed || ap.land_complete) {
        return;
    }

    // exit immediately if neither control has been setup
    if ((mower.amplitude_control == nullptr) && (mower.period_control == nullptr)) {
        return;
    }

    // if not called recently initialise everything
    uint32_t now_ms = AP_HAL::millis();
    if (now_ms - mower.last_update_ms > 1000) {
        mower.pos_offset_m = 0;
        mower.time_sec = 0;
    }
    mower.last_update_ms = now_ms;

    // get latest pilot input of amplitude
    // if pilot is not controlling amplitude default to MOWER_AMPLITUDE parameter value
    float pilot_amplitude_m = g2.mower_amplitude_m;
    if (mower.amplitude_control != nullptr) {
        pilot_amplitude_m *= mower.amplitude_control->percent_input() * 0.01;
    }

    // get pilot input of period
    // if pilot is not controlling period, default to MOWER_PERIOPD parameter value
    float pilot_period_sec = g2.mower_period_sec;
    bool stop_movement = false;
    if (mower.period_control != nullptr) {
        const uint8_t pilot_period_pct = mower.period_control->percent_input();
        stop_movement = (pilot_period_pct >= 100);
        pilot_period_sec *= (pilot_period_pct * 0.01);
    }

    // limit the period using the maximum speed
    float speed_max_z_m = MIN(fabsf(pos_control->get_max_speed_up_cms()), fabsf(pos_control->get_max_speed_down_cms())) * 0.01;
    if (is_positive(speed_max_z_m)) {
        const float period_min_sec = pilot_amplitude_m * 2.0 / speed_max_z_m;
        pilot_period_sec = MAX(pilot_period_sec, period_min_sec);
    }

    // update a "time" value
    if (pilot_period_sec > 0 && !stop_movement) {
        mower.time_sec += (scheduler.get_loop_period_s() / pilot_period_sec);
    }
    if (mower.time_sec > 1) {
        mower.time_sec -= 1;
    }
    if (is_zero(pilot_amplitude_m)) {
        mower.time_sec = 0;
    }

    // use cos(time) to calculate a position offset
    mower.pos_offset_m = (cosf(mower.time_sec * M_2PI) - 1.0) * 0.5 * pilot_amplitude_m;

    // set position controller offset
    pos_control->set_pos_offset_target_z_cm(mower.pos_offset_m * 100.0);

    /*
    // debug
    static uint32_t last_print_ms = 0;
    if (now_ms - last_print_ms > 1000) {
        last_print_ms = now_ms;
        gcs().send_text(MAV_SEVERITY_CRITICAL, "amp:%4.2f per:%4.2f mt:%4.2f pos:%4.2f",
                (double)pilot_amplitude_m, (double)pilot_period_sec,
                (double)mower.time_sec, (double)mower.pos_offset_m);
    }
    */
}
