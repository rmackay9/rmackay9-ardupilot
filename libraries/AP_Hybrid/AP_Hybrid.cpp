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

#include <AP_HAL/AP_HAL.h>
#include <AP_Common/AP_Common.h>
#include <AP_Math/AP_Math.h>
#include <AP_SerialManager/AP_SerialManager.h>
#include <ctype.h>
#include <stdio.h>
#include "AP_Hybrid.h"
#include <GCS_MAVLink/GCS.h>

#define DEP_DEBUG DISABLED

extern const AP_HAL::HAL& hal;
/*
   The constructor also initialises the . Note that this
   constructor is not called until detect() returns true, so we
   already know that we should setup the
*/
void AP_Hybrid::init(AP_SerialManager &serial_manager)
{
//	hybridData = 0;
    uart = serial_manager.find_serial(AP_SerialManager::SerialProtocol_Hybrid, 0);
	if (uart != nullptr) {
        uart->begin(serial_manager.find_baudrate(AP_SerialManager::SerialProtocol_Hybrid, 0));
	}
}
/*
   detect if a Depth is connected. We'll detect by
   trying to take a reading on Serial. If we get a result the sensor is
   there.
*/
bool AP_Hybrid::detect(AP_SerialManager &serial_manager)
{
    return serial_manager.find_serial(AP_SerialManager::SerialProtocol_Hybrid, 0) != nullptr;
}

// read - return last value measured by sensor
bool AP_Hybrid::get_reading(HybridReadData &reading)
{

    if (uart == nullptr) {
        return false;
    }
    // read any available lines
//    float sum = 0;
    uint8_t  index = 0;
    uint16_t count = 0;
    uint16_t checksum = 0;
    int16_t nbytes = uart->available();
    uint8_t bitCnt = 0;

    //AA 55 00 0A 00 00 00 00 00 04 1E B0 00 10 00 00 23 7A 23
    //7A 11 1D 00 00 00 00 01 00 00 00 00 00 00 00 00 00 00 00
    //00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
    //00 00 00 00 00 00 00 00 00 1E BE 55 AA
    //16 bit one data Hight + Low
    while (nbytes-- > 0)
    {
		uint8_t c = uart->read();
		bitCnt++;
//		gcs().send_text(MAV_SEVERITY_NOTICE, "Read hybrid %d %x", bitCnt, c);
		if (c == 0xAA && index == 0)
		{
			linebuf_len = 0;
			checksum = 0;
			index = 1;
		}
		if (c == 0x55 && index == 1)
		{
			index = 2;
			continue;
		}
		// now it is ready to decode index information
		//version
		//2byte
		if(index == 2)
		{
			linebuf[linebuf_len] = c;
			linebuf_len ++;
			if (linebuf_len == 2)
			{
				index = 3;
				checksum = checksum + (linebuf[0] << 8 | linebuf[1]);
				continue;
			}
		}
		//run time
		//4byte
		if(index == 3)
		{
			linebuf[linebuf_len] = c;
			linebuf_len ++;
			if (linebuf_len == (4 + 2))
			{
				index = 4;
				reading.runTime = ((linebuf[4] << 24 | linebuf[5] << 16 | linebuf[2] << 8 | linebuf[3]));
				checksum = checksum + (linebuf[2] << 8 | linebuf[3]);	//min sec
				checksum = checksum + (linebuf[4] << 8 | linebuf[5]);	//hour
//				gcs().send_text(MAV_SEVERITY_NOTICE, "Run time %x", reading.runTime);
				continue;
			}
		}
		//next repair time
		//4byte
		if(index == 4)
		{
			linebuf[linebuf_len] = c;
			linebuf_len ++;
			if (linebuf_len == (4 + 4 + 2))
			{
				index = 5;
				reading.nextRepairTime = (linebuf[6] << 24 | linebuf[7] << 16 | (linebuf[8] << 8 | linebuf[9]));
				checksum = checksum + (linebuf[6] << 8 | linebuf[7]);
				checksum = checksum + (linebuf[8] << 8 | linebuf[9]);
//				gcs().send_text(MAV_SEVERITY_NOTICE, "Next time %x", reading.nextRepairTime);
				continue;
			}
		}
		//run status
		//2byte
		if(index == 5)
		{
			linebuf[linebuf_len] = c;
			linebuf_len ++;
			if (linebuf_len == (2 + 4 + 4 + 2))
			{
				index = 6;
				reading.runStatus = linebuf[10] << 8 | linebuf[11];
//				gcs().send_text(MAV_SEVERITY_NOTICE, "Run status %x", reading.runStatus);
				continue;
			}
		}
		//rate
		//2byte
		if(index == 6)
		{
			linebuf[linebuf_len] = c;
			linebuf_len ++;
			if (linebuf_len == (2 + 2 + 4 + 4 + 2))
			{
				index = 7;
				reading.rate = linebuf[12] << 8 | linebuf[13];
				continue;
			}
		}
		//skip byte
		//4byte
		if(index == 7)
		{
			linebuf[linebuf_len] = c;
			linebuf_len ++;
			if (linebuf_len == (4 + 2 + 2 + 4 + 4 + 2))
			{
				index = 8;
				continue;
			}
		}
		//voltage
		//2byte
		if(index == 8)
		{
			linebuf[linebuf_len] = c;
			linebuf_len ++;
			if (linebuf_len == (2 + 4 + 2 + 2 + 4 + 4 + 2))
			{
				index = 9;
				reading.voltage = linebuf[18] << 8 | linebuf[19];
				continue;
			}
		}
		//power
		//2byte
		if(index == 9)
		{
			linebuf[linebuf_len] = c;
			linebuf_len ++;
			if (linebuf_len == (2 + 2 + 4 + 2 + 2 + 4 + 4 + 2))
			{
				index = 10;
				reading.current = linebuf[20] << 8 | linebuf[21];
				continue;
			}
		}
		//skip
		//3byte
		if(index == 10)
		{
			linebuf[linebuf_len] = c;
			linebuf_len ++;
			if (linebuf_len == (3 + 2 + 2 + 4 + 2 + 2 + 4 + 4 + 2))
			{
				index = 11;
				continue;
			}
		}
		//status
		//1byte
		if(index == 11)
		{
			linebuf[linebuf_len] = c;
			linebuf_len ++;
			if (linebuf_len == (1 + 3 + 2 + 2 + 4 + 2 + 2 + 4 + 4 + 2))
			{
				index = 12;
				reading.status = linebuf[25];
				continue;
			}
		}
		//skip data
		//38byte
		if(index == 12)
		{
			linebuf[linebuf_len] = c;
			linebuf_len ++;
			if (linebuf_len == (38 + 1 + 3 + 2 + 2 + 4 + 2 + 2 + 4 + 4 + 2))
			{
				index = 13;
				continue;
			}
		}
		//checksum
		//2byte
		if(index == 13)
		{
			linebuf[linebuf_len] = c;
			linebuf_len ++;
			if (linebuf_len == (2 + 38 + 1 + 3 + 2 + 2 + 4 + 2 + 2 + 4 + 4 + 2))
			{
				index = 14;
				uint16_t sum = linebuf[64] << 8 | linebuf[65];
//				gcs().send_text(MAV_SEVERITY_NOTICE, "Sum checksum %x %x", sum, checksum);
				if(checksum == sum)
				{
					reading.checksum = true;
				}
				else
				{
					reading.checksum = false;
				}
				continue;
			}
		}
		//trail
		if(c == 0x55 && index == 14)
		{
			index = 15;
		}
		if(c == 0xAA && index == 15)
		{
			index = 0;
			count++;
		}
	}

    if (count == 0) {
        return false;
    }

//    reading = sum / count;
    return true;
}

/*
   update the state of the sensor
*/
void AP_Hybrid::update(void)
{
    if (get_reading(hybridData))
    {
        // update
        last_reading_ms = AP_HAL::millis();
        //printf("Data is %.2f\r\n",hybridData);
    } else if (AP_HAL::millis() - last_reading_ms > 2000)
    {
//        printf("No hybrid data\r\n");
    }
}
