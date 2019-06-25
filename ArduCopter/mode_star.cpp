#include "Copter.h"

#if MODE_STAR_ENABLED == ENABLED

/*
 * Init and run calls for "star" flight mode
 */

double Sin18deg = sin(M_PI / 10);
double Cos18deg = cos(M_PI / 10);
double Sin54deg = sin(3 * M_PI / 10);
double Cos54deg = cos(3 * M_PI / 10);

double DefaultStartAlt = 20 * 100;	// 20m (2000cm)

// initialise star controller
bool ModeStar::init(bool ignore_checks)
{
	bool takingoff = takeoff.running();
	bool armed = motors->armed();
	fprintf(stderr,"[%s:%d] Entering... ignore_checks=%d, armed=%d, takingoff=%d\n",
			__FUNCTION__, __LINE__, ignore_checks, armed, takingoff);

	//int16_t numCmd = mission.num_commands();

	if (ignore_checks) {
		_mode = Auto_Loiter;

		if (!armed) {
			fprintf(stderr,"[%s:%d] Leaving w/ return false. Disarmed !!\n",
					__FUNCTION__, __LINE__);
			return false;
		}

		// stop ROI from carrying over from previous runs of the mission
		// To-Do: reset the yaw as part of auto_wp_start when the previous command was not a wp command to remove the need for this special ROI check
		if (auto_yaw.mode() == AUTO_YAW_ROI) {
			auto_yaw.set_mode(AUTO_YAW_HOLD);
		}

		// initialise waypoint and spline controller
		wp_nav->wp_and_spline_init();

		// clear guided limits
		copter.mode_guided.limit_clear();
	}

	if (!setup_star_wp()) {
		fprintf(stderr,"[%s:%d] Leaving w/ return false. Setup wp failed!!\n",
				__FUNCTION__, __LINE__);
		return false;
	}

	// initialize speeds and accelerations
	pos_control->set_max_speed_xy(wp_nav->get_default_speed_xy());
	pos_control->set_max_accel_xy(wp_nav->get_wp_acceleration());
	pos_control->set_max_speed_z(-get_pilot_speed_dn(), g.pilot_speed_up);
	pos_control->set_max_accel_z(g.pilot_accel_z);

	fprintf(stderr,"[%s:%d] Leaving w/ return true.\n", __FUNCTION__, __LINE__);
	return true;
}

// run the star controller
// should be called at 100hz or more
void ModeStar::run()
{
	switch (_mode) {

		case Auto_TakeOff:
			takeoff_run();
			break;

		case Auto_WP:
			wp_run();
			break;

		case Auto_Land:
			land_run();
			break;

		case Auto_RTL:
			rtl_run();
			break;

		case Auto_Loiter:
			loiter_run();
			break;

		default:
			break;
	}
}

uint32_t ModeStar::wp_distance() const
{
	return copter.circle_nav->get_distance_to_target();
}

int32_t ModeStar::wp_bearing() const
{
	return copter.circle_nav->get_bearing_to_target();
}

bool ModeStar::start_command(const AP_Mission::Mission_Command& cmd)
{
	return true;
}

bool ModeStar::verify_command(const AP_Mission::Mission_Command& cmd)
{
	return true;
}

void ModeStar::exit_mission()
{
}

bool ModeStar::setup_star_wp()
{
	startLoc = copter.current_loc;

	_mode = Auto_WP;
	double start_alt = MAX(MIN(g2.star_start_alt, 100 * 100), 15 * 100);
	if (startLoc.alt < start_alt) {	// Taking off if alt is less than 20m.
		startLoc.alt = start_alt;
		if (!wp_nav->set_wp_destination(startLoc)) {
			return false;
		}
		fprintf(stderr,"[%s:%d] Taking off first !!!\n", __FUNCTION__, __LINE__);
		takeoff_start(startLoc);
	} else {
	}
	fprintf(stderr,"[%s:%d] startLoc=(%f, %f, %f)\n",
			__FUNCTION__, __LINE__, startLoc.lat * 1e-7, startLoc.lng * 1e-7, startLoc.alt * 0.01);

	curDest = 0;

	float scale = calc_scale();
	wPnts[0] = Location(
			startLoc.lat,
			startLoc.lng,
			startLoc.alt,
			Location::AltFrame::ABOVE_HOME);
	wPnts[1] = Location(
			startLoc.lat - scale * (1 + Sin54deg),	// deg * 1e7
			startLoc.lng - scale * Cos54deg,		// deg * 1e7
			startLoc.alt - 1000,	// cm
			Location::AltFrame::ABOVE_HOME);
	wPnts[2] = Location(
			startLoc.lat - scale * (1 - Sin18deg),	// deg * 1e7
			startLoc.lng + scale * Cos18deg,		// deg * 1e7
			startLoc.alt - 500,		// cm
			Location::AltFrame::ABOVE_HOME);
	wPnts[3] = Location(
			startLoc.lat - scale * (1 - Sin18deg),	// deg * 1e7
			startLoc.lng - scale * Cos18deg,		// deg * 1e7
			startLoc.alt - 500,		// cm
			Location::AltFrame::ABOVE_HOME);
	wPnts[4] = Location(
			startLoc.lat - scale * (1 + Sin54deg),	// deg * 1e7
			startLoc.lng + scale * Cos54deg,		// deg * 1e7
			startLoc.alt - 1000,	// cm
			Location::AltFrame::ABOVE_HOME);
	wPnts[5] = Location(
			startLoc.lat,
			startLoc.lng,
			startLoc.alt,
			Location::AltFrame::ABOVE_HOME);

	bool ret = wp_nav->set_wp_destination(wPnts[0]);
	if (!ret) {
		fprintf(stderr,"[%s:%d] Leaving w/ return false. WP could not be set...\n",
				__FUNCTION__, __LINE__);
        return false;
    }

    origin = wp_nav->get_wp_origin();
    Vector3f wp = wp_nav->get_wp_destination();
    fprintf(stderr,"[%s:%d] Next: (%f, %f, %f) -> (%f, %f, %f), scale = %f\n",
            __FUNCTION__, __LINE__, origin.x, origin.y, origin.z, wp.x, wp.y, wp.z,
			scale);

    return true;
}

float ModeStar::calc_scale()
{
	Location unitDest = Location(
			copter.current_loc.lat - (1 + Sin54deg) * 1e7,  // deg * 1e7
			copter.current_loc.lng - Cos54deg * 1e7,        // deg * 1e7
			copter.current_loc.alt,                         // cm
			Location::AltFrame::ABOVE_HOME);
	wp_nav->set_wp_destination(unitDest);

	float unit = wp_nav->get_wp_distance_to_destination();  // cm
    fprintf(stderr,"[%s:%d] SideLen=%d[cm], unit=%f[m], StartAlt=%d[cm]\n",
            __FUNCTION__, __LINE__, g2.star_side_len, unit / 100, g2.star_start_alt);

	return g2.star_side_len * 1e7 / unit;
}

// do_guided - start guided mode
bool ModeStar::do_guided(const AP_Mission::Mission_Command& cmd)
{
    // only process guided waypoint if we are in guided mode
    if (copter.control_mode != GUIDED && !(copter.control_mode == AUTO && mode() == Auto_NavGuided)) {
        return false;
    }

    // switch to handle different commands
    switch (cmd.id) {

        case MAV_CMD_NAV_WAYPOINT:
            {
                // set wp_nav's destination
                Location dest(cmd.content.location);
                return copter.mode_guided.set_destination(dest);
            }

        case MAV_CMD_CONDITION_YAW:
            do_yaw(cmd);
            return true;

        default:
            // reject unrecognised command
            return false;
    }

    return true;
}

// auto_rtl_start - initialises RTL in AUTO flight mode
void ModeStar::rtl_start()
{
    _mode = Auto_RTL;

    // call regular rtl flight mode initialisation and ask it to ignore checks
    copter.mode_rtl.init(true);
}

// auto_takeoff_start - initialises waypoint controller to implement take-off
void ModeStar::takeoff_start(const Location& dest_loc)
{
    fprintf(stderr,"[%s:%d] Entering...\n", __FUNCTION__, __LINE__);
    _mode = Auto_TakeOff;

    Location dest(dest_loc);

    if (!copter.current_loc.initialised()) {
        // vehicle doesn't know where it is ATM.  We should not
        // initialise our takeoff destination without knowing this!
        return;
    }
    fprintf(stderr,"[%s:%d] Start taking off...\n", __FUNCTION__, __LINE__);

    // set horizontal target
    dest.lat = copter.current_loc.lat;
    dest.lng = copter.current_loc.lng;

    // get altitude target
    int32_t alt_target;
    if (!dest.get_alt_cm(Location::AltFrame::ABOVE_HOME, alt_target)) {
        // this failure could only happen if take-off alt was specified as an alt-above terrain and we have no terrain data
        AP::logger().Write_Error(LogErrorSubsystem::TERRAIN, LogErrorCode::MISSING_TERRAIN_DATA);
        // fall back to altitude above current altitude
        alt_target = copter.current_loc.alt + dest.alt;
    }

    // sanity check target
    if (alt_target < copter.current_loc.alt) {
        dest.set_alt_cm(copter.current_loc.alt, Location::AltFrame::ABOVE_HOME);
    }

    // Note: if taking off from below home this could cause a climb to an unexpectedly high altitude
    if (alt_target < 100) {
        dest.set_alt_cm(100, Location::AltFrame::ABOVE_HOME);
    }

    // set waypoint controller target
    if (!wp_nav->set_wp_destination(dest)) {
        // failure to set destination can only be because of missing terrain data
        copter.failsafe_terrain_on_event();
        return;
    }

    // initialise yaw
    auto_yaw.set_mode(AUTO_YAW_HOLD);

    // clear i term when we're taking off
    set_throttle_takeoff();

    // get initial alt for WP_NAVALT_MIN
    auto_takeoff_set_start_alt();
}

// auto_wp_start - initialises waypoint controller to implement flying to a particular destination
void ModeStar::wp_start(const Location& dest_loc)
{
    _mode = Auto_WP;

    // send target to waypoint controller
    if (!wp_nav->set_wp_destination(dest_loc)) {
        // failure to set destination can only be because of missing terrain data
        copter.failsafe_terrain_on_event();
        return;
    }

    // initialise yaw
    // To-Do: reset the yaw only when the previous navigation command is not a WP.  this would allow removing the special check for ROI
    if (auto_yaw.mode() != AUTO_YAW_ROI) {
        auto_yaw.set_mode_to_default(false);
    }
}

// auto_takeoff_run - takeoff in auto mode
//      called by auto_run at 100hz or more
void ModeStar::takeoff_run()
{
    auto_takeoff_run();
    if (wp_nav->reached_wp_destination()) {
        fprintf(stderr,"[%s:%d] Reached Starting point.\n", __FUNCTION__, __LINE__);
        _mode = Auto_WP;
    }
}

// auto_wp_run - runs the auto waypoint controller
//      called by auto_run at 100hz or more
void ModeStar::wp_run()
{
    //fprintf(stderr,"[%s:%d] Entering...\n", __FUNCTION__, __LINE__);

    if (wp_nav->reached_wp_destination()) {
        fprintf(stderr,"[%s:%d] Reached Dest# %d.\n",
                __FUNCTION__, __LINE__, curDest);

        if (++curDest < numDest) {
            wp_nav->shift_wp_origin_to_current_pos();
            if (!wp_nav->set_wp_destination(wPnts[curDest])) {
                fprintf(stderr,"[%s:%d] Failed to set a new WP Dest# %d.\n",
                        __FUNCTION__, __LINE__, curDest);

                // ToDo
                return;
            }
            Vector3f newOrigin = wp_nav->get_wp_origin();
            Vector3f newDest = wp_nav->get_wp_destination();
            fprintf(stderr,"[%s:%d] Next: (%f, %f, %f) -> (%f, %f, %f) : %f[m]\n",
                    __FUNCTION__, __LINE__, newOrigin.x, newOrigin.y, newOrigin.z,
					newDest.x, newDest.y, newDest.z,
					wp_nav->get_wp_distance_to_destination() / 100);
		} else {
			fprintf(stderr,"[%s:%d] Mission compleleted !!\n",
					__FUNCTION__, __LINE__);
			_mode = Auto_Loiter;

			return;
		}
	}

	// process pilot's yaw input
	float target_yaw_rate = 0;
	if (!copter.failsafe.radio) {
		// get pilot's desired yaw rate
		target_yaw_rate = get_pilot_desired_yaw_rate(channel_yaw->get_control_in());
		if (!is_zero(target_yaw_rate)) {
			auto_yaw.set_mode(AUTO_YAW_HOLD);
		}
	}

	// if not armed set throttle to zero and exit immediately
	if (is_disarmed_or_landed()) {
		make_safe_spool_down();
		wp_nav->wp_and_spline_init();
		return;
	}

	// set motors to full range
	motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

	// run waypoint controller
	copter.failsafe_terrain_set_status(wp_nav->update_wpnav());

	// call z-axis position controller (wpnav should have already updated it's alt target)
	pos_control->update_z_controller();

	// call attitude controller
	if (auto_yaw.mode() == AUTO_YAW_HOLD) {
		// roll & pitch from waypoint controller, yaw rate from pilot
		attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(wp_nav->get_roll(), wp_nav->get_pitch(), target_yaw_rate);
	} else {
		// roll, pitch from waypoint controller, yaw heading from auto_heading()
		attitude_control->input_euler_angle_roll_pitch_yaw(wp_nav->get_roll(), wp_nav->get_pitch(), auto_yaw.yaw(), true);
	}
}

void ModeStar::land_run()
{
}

void ModeStar::rtl_run()
{
}

// auto_loiter_run - loiter in AUTO flight mode
//      called by auto_run at 100hz or more
void ModeStar::loiter_run()
{
	//fprintf(stderr,"[%s:%d] Entering...\n", __FUNCTION__, __LINE__);
	// if not armed set throttle to zero and exit immediately
	if (is_disarmed_or_landed()) {
		make_safe_spool_down();
		wp_nav->wp_and_spline_init();
		return;
	}

	// accept pilot input of yaw
	float target_yaw_rate = 0;
	if (!copter.failsafe.radio) {
		target_yaw_rate = get_pilot_desired_yaw_rate(channel_yaw->get_control_in());
	}

	// set motors to full range
	motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

	// run waypoint and z-axis position controller
	copter.failsafe_terrain_set_status(wp_nav->update_wpnav());

	pos_control->update_z_controller();
	attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(wp_nav->get_roll(), wp_nav->get_pitch(), target_yaw_rate);
}

#endif // MODE_STAR_ENABLED == ENABLED
