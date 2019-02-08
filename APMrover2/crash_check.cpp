#include "Rover.h"

// Code to detect a crash or block
static const uint16_t CRASH_CHECK_TRIGGER_SEC = 2;   // 2 seconds blocked indicates a crash
static const float CRASH_CHECK_THROTTLE_MIN = 5.0f;  // vehicle must have a throttle greater that 5% to be considered crashed
static const float CRASH_CHECK_VEL_MIN = 0.08f;      // vehicle must have a velocity under 0.08 m/s or rad/s to be considered crashed

void Rover::crash_check_init()
{
    crashcheck.stage = CrashStage_None;
    crashcheck.recovery_counter = 0;
    crashcheck.detected_angle_ms = -1;
    crashcheck.detected_vel_ms = -1;
}

// crash_check - disarms motors if a crash or block has been detected
// crashes are detected by the vehicle being static (no speed) for more than CRASH_CHECK_TRIGGER_SEC and motor are running
// called at 10Hz
void Rover::crash_check()
{
    // return immediately if disarmed, crash checking is disabled or vehicle is Hold, Manual or Acro mode(if it is not a balance bot)
    if (!arming.is_armed() || g.fs_crash_check == FS_CRASH_DISABLE || crashcheck.stage != CrashStage_Recovery || ((!control_mode->is_autopilot_mode()) && (!is_balancebot()))) {
        return;
    }

    // Crashed if pitch/roll > crash_angle
    if ((g2.crash_angle != 0) && ((fabsf(ahrs.pitch) > radians(g2.crash_angle)) || (fabsf(ahrs.roll) > radians(g2.crash_angle)))) {
        if (crashcheck.detected_angle_ms != -1 && AP_HAL::micros() - crashcheck.detected_angle_ms >= 2000) {
            crashcheck.stage = CrashStage_Emergency;
        } else {
            crashcheck.detected_angle_ms = AP_HAL::micros();
            crashcheck.stage = CrashStage_Detected;
        }
    }

    // TODO : Check if min vel can be calculated
    // min_vel = ( CRASH_CHECK_THROTTLE_MIN * g.speed_cruise) / g.throttle_cruise;

    if (!is_balancebot()) {
        if (crashcheck.detected_angle_ms == -1 && ((ahrs.groundspeed() >= CRASH_CHECK_VEL_MIN) ||  // Check velocity
            (fabsf(ahrs.get_gyro().z) >= CRASH_CHECK_VEL_MIN) ||  // Check turn speed
            (fabsf(g2.motors.get_throttle()) < CRASH_CHECK_THROTTLE_MIN))) {
            return;
        }

        // check if crashing for 2 seconds
        if (crashcheck.detected_angle_ms != -1 && AP_HAL::micros() - crashcheck.detected_vel_ms >= 2000) {
            crashcheck.stage = CrashStage_Emergency;
        } else {
            crashcheck.detected_vel_ms = AP_HAL::micros();
            crashcheck.stage = CrashStage_Detected;
        }
    }

    crash_action();
}

void Rover::crash_action()
{
    switch (crashcheck.stage) {
    case CrashStage_Detected:
        // log an error in the dataflash
        Log_Write_Error(ERROR_SUBSYSTEM_CRASH_CHECK, ERROR_CODE_CRASH_CHECK_CRASH);

        if (is_balancebot()) {
            // send message to gcs
            gcs().send_text(MAV_SEVERITY_EMERGENCY, "Crash: Disarming");
            disarm_motors();
        } else {
            if (!g2.crash_recover_enable) {
                // send message to gcs
                gcs().send_text(MAV_SEVERITY_EMERGENCY, "Crash: Going to HOLD");
                // change mode to hold and disarm
                set_mode(mode_hold, MODE_REASON_CRASH_FAILSAFE);
                if (g.fs_crash_check == FS_CRASH_HOLD_AND_DISARM) {
                    disarm_motors();
                }
            } else {
                // send message to gcs
                gcs().send_text(MAV_SEVERITY_EMERGENCY, "Crash: Going to RECOVERY");
                crashcheck.stage = CrashStage_Recovery;
            }
        }
        break;
    
    case CrashStage_Recovery:
        crashcheck.recovery_counter += 1;
        crashcheck.detected_angle_ms = -1;
        crashcheck.detected_vel_ms = -1;

        if (crashcheck.recovery_counter == 1) {
            crashcheck.recovery_mode = control_mode;
        }

        // recvery action
        if (!crash_recovery_action()) {
            crashcheck.stage = CrashStage_Emergency;
        }
        break;
    
    case CrashStage_Emergency:
        // send message to gcs
        gcs().send_text(MAV_SEVERITY_EMERGENCY, "Crash: Going to EMERGENCY");
        // change mode to hold and disarm
        set_mode(mode_hold, MODE_REASON_CRASH_FAILSAFE);
        if (g.fs_crash_check == FS_CRASH_HOLD_AND_DISARM) {
            disarm_motors();
        }
        break;
    }
}

bool Rover::crash_recovery_action()
{
    if (crashcheck.recovery_counter > 20) {
        set_mode(*(crashcheck.recovery_mode), MODE_REASON_CRASH_RECOVERY);
        crash_check_init();
        return true;
    }

    switch (g2.crash_recover_action) {
    case CrashAction_Stay:
        set_mode(mode_hold, MODE_REASON_CRASH_RECOVERY);
        break;
    case CrashAction_Back:
        // TODO vehicle go back
        break;
    }
    return true;
}

bool Rover::is_recoverable_crash()
{
    return crashcheck.stage != CrashStage_Emergency;
}
