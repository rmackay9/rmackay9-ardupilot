/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
/*
  support for autotune of helicopters
 */

#pragma once

#include "AC_AutoTune.h"

class AC_AutoTune_Heli : public AC_AutoTune
{
public:
    // constructor
    AC_AutoTune_Heli()
    {
        tune_seq[0] = TUNE_COMPLETE;
    };
    // save gained, called on disarm
    void save_tuning_gains() override;

protected:

    void load_test_gains() override;

    // get intra test rate I gain for the specified axis
    float get_intra_test_ri(AxisType test_axis) override { return 0.0f; }

    // get tuned rate I gain for the specified axis
    float get_tuned_ri(AxisType test_axis) override { return 0.0f; }

    // get tuned yaw rate d gain
    float get_tuned_yaw_rd() override { return tune_yaw_rd; }

    void test_init() override;
    void test_run(AxisType test_axis, const float dir_sign) override;

    // update gains for the rate p up tune type
    void updating_rate_p_up_all(AxisType test_axis) override {};

    // update gains for the rate p down tune type
    void updating_rate_p_down_all(AxisType test_axis) override {};

    // update gains for the rate d up tune type
    void updating_rate_d_up_all(AxisType test_axis) override {};

    // update gains for the rate d down tune type
    void updating_rate_d_down_all(AxisType test_axis) override {};

    // update gains for the angle p up tune type
    void updating_angle_p_up_all(AxisType test_axis) override {};

    // update gains for the angle p down tune type
    void updating_angle_p_down_all(AxisType test_axis) override {};

    // get minimum rate P (for any axis)
    float get_rp_min() const override { return 0.0f; }

    // get minimum angle P (for any axis)
    float get_sp_min() const override { return 0.0f; }

    // get minimum rate Yaw filter value
    float get_yaw_rate_filt_min() const override { return 0.0f; }

    void Log_AutoTune() override {};
    void Log_AutoTuneDetails() override {};

    // returns true if rate P gain of zero is acceptable for this vehicle
    bool allow_zero_rate_p() override { return true; }

    // send intermittant updates to user on status of tune
    void do_gcs_announcements() override;
};
