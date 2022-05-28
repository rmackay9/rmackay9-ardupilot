/*
  SToRM32 mount backend class
 */
#pragma once

#include "AP_Mount_MAVLink.h"

#if HAL_MOUNT_ENABLED

#include <AP_Math/AP_Math.h>
#include <AP_Common/AP_Common.h>
#include <RC_Channel/RC_Channel.h>
#include <AP_AHRS/AP_AHRS.h>

class AP_Mount_Gremsy : public AP_Mount_MAVLink
{

public:
    // Constructor
    AP_Mount_Gremsy(AP_Mount &frontend, AP_Mount::mount_state &state, uint8_t instance);

    // Gremsy gimbals are all 3-axis
    bool has_pan_control() const override { return true; }

private:

    // no private methods or members

};
#endif // HAL_MOUNT_ENABLED
