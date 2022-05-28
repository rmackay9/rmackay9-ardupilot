#include "AP_Mount_SToRM32.h"
#if HAL_MOUNT_ENABLED
#include <AP_HAL/AP_HAL.h>

extern const AP_HAL::HAL& hal;

AP_Mount_SToRM32::AP_Mount_SToRM32(AP_Mount &frontend, AP_Mount::mount_state &state, uint8_t instance) :
    AP_Mount_MAVLink(frontend, state, instance)
{}

#endif // HAL_MOUNT_ENABLED
