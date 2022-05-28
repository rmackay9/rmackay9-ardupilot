/*
  SToRM32 mount backend class
 */
#pragma once

#include "AP_Mount_MAVLink.h"

#if HAL_MOUNT_ENABLED

class AP_Mount_SToRM32 : public AP_Mount_MAVLink
{

public:
    // Constructor
    AP_Mount_SToRM32(AP_Mount &frontend, AP_Mount::mount_state &state, uint8_t instance);

    // SToRM32 gimbals do not support 3-axis
    bool has_pan_control() const override { return false; }

private:

    // no private methods or members

};
#endif // HAL_MOUNT_ENABLED
