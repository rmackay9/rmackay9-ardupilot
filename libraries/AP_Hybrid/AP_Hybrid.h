// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
#pragma once

#include <AP_Param/AP_Param.h>
#include <AP_Common/AP_Common.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_Param/AP_Param.h>
#include <AP_Math/AP_Math.h>
#include <AP_SerialManager/AP_SerialManager.h>
#include <AP_GPS/AP_GPS.h>
#include <AP_AHRS/AP_AHRS.h>

class AP_Hybrid
{

public:

	struct HybridReadData {
		uint32_t    runTime;
		uint32_t    nextRepairTime;
		uint16_t	runStatus;
		uint16_t	rate;
		uint16_t	voltage;
		uint16_t	current;
		uint8_t		status;
		bool		checksum;
	};

	void init(AP_SerialManager &serial_manager);
    // static detection function
    static bool detect(AP_SerialManager &serial_manager);
    // update state
    void update(void);
    HybridReadData get_data(void)
    {
        return hybridData;
    }

private:
    // get a reading
    bool get_reading(HybridReadData &reading);
    HybridReadData hybridData;
    AP_HAL::UARTDriver *uart = nullptr;
    uint32_t last_reading_ms = 0;
    //data + checksum
    uint8_t linebuf[64 + 2];
    uint8_t linebuf_len = 0;

	uint32_t        _last_record_time;   // last time
	struct Location _last_location;
};
