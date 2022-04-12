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

#include "AP_EPropulsion.h"

#if HAL_EPROPULSION_ENABLED

#include <AP_Common/AP_Common.h>
#include <AP_Math/AP_Math.h>
#include <SRV_Channel/SRV_Channel.h>
#include <AP_Logger/AP_Logger.h>
#include <GCS_MAVLink/GCS.h>

#define EPROPULSION_SERIAL_BAUD        38400   // communication is always at 38400
#define EPROPULSION_PACKET_HEADER      0x28    // communication packet header
#define EPROPULSION_PACKET_FOOTER      0x29    // communication packet footer
#define EPROPULSION_PARSE_TIMEOUT_MS   25      // message parsing times out if no characters received for this many milliseconds
#define EPROPULSION_BATT_TIMEOUT_MS    5000    // battery info timeouts after 5 seconds
#define EPROPULSION_LOG_EPRP_INTERVAL_MS            5000   // log EPRP messages at this interval in milliseconds
#define EPROPULSION_ERROR_REPORT_INTERVAL_MAX_MS   10000   // errors reported to user at no less than once every 10 seconds

extern const AP_HAL::HAL& hal;

// parameters
const AP_Param::GroupInfo AP_EPropulsion::var_info[] = {

    // @Param: TYPE
    // @DisplayName: EPropulsion connection type
    // @Description: EPropulsion connection type
    // @Values: 0:Disabled, 1:Throttle Control Board
    // @User: Standard
    // @RebootRequired: True
    AP_GROUPINFO_FLAGS("TYPE", 1, AP_EPropulsion, _type, (int8_t)ConnectionType::TYPE_DISABLED, AP_PARAM_FLAG_ENABLE),

    // @Param: DE_PIN
    // @DisplayName: EPropulsion DE pin
    // @Description: Pin number connected to RS485 to Serial converter's DE pin. -1 to use serial port's CTS pin if available
    // @Values: -1:Disabled,50:AUX1,51:AUX2,52:AUX3,53:AUX4,54:AUX5,55:AUX6
    // @User: Standard
    // @RebootRequired: True
    AP_GROUPINFO("DE_PIN", 3, AP_EPropulsion, _pin_de, -1),

    // @Param: OPTIONS
    // @DisplayName: EPropulsion Options
    // @Description: EPropulsion Options Bitmask
    // @Bitmask: 0:Log,1:Send debug to GCS
    // @User: Advanced
    AP_GROUPINFO("OPTIONS", 4, AP_EPropulsion, _options, (int8_t)options::LOG),

    // @Param: SLEW_TIME
    // @DisplayName: EPropulsion Throttle Slew Time
    // @Description: EPropulsion slew rate specified as the minimum number of seconds required to increase the throttle from 0 to 100%.  Higher values cause a slower response, lower values cause a faster response.  A value of zero disables the limit
    // @Units: s
    // @Range: 0 5
    // @Increment: 0.1
    // @User: Advanced
    AP_GROUPINFO("SLEW_TIME", 6, AP_EPropulsion, _slew_time, 2.0),

    // @Param: DIR_DELAY
    // @DisplayName: EPropulsion Direction Change Delay
    // @Description: EPropulsion direction change delay.  Output will remain at zero for this many seconds when transitioning between forward and backwards rotation
    // @Units: s
    // @Range: 0 5
    // @Increment: 0.1
    // @User: Advanced
    AP_GROUPINFO("DIR_DELAY", 7, AP_EPropulsion, _dir_delay, 1.0),

    AP_GROUPEND
};

AP_EPropulsion::AP_EPropulsion()
{
    _singleton = this;
    AP_Param::setup_object_defaults(this, var_info);
}

// initialise driver
void AP_EPropulsion::init()
{
    // exit immediately if not enabled
    if (!enabled()) {
        return;
    }

    // only init once
    // Note: a race condition exists here if init is called multiple times quickly before thread_main has a chance to set _initialise
    if (_initialised) {
        return;
    }

    // create background thread to process serial input and output
    if (!hal.scheduler->thread_create(FUNCTOR_BIND_MEMBER(&AP_EPropulsion::thread_main, void), "epropulsion", 2048, AP_HAL::Scheduler::PRIORITY_RCOUT, 1)) {
        return;
    }
}

// initialise serial port and gpio pins (run from background thread)
bool AP_EPropulsion::init_internals()
{
    // find serial driver and initialise
    const AP_SerialManager &serial_manager = AP::serialmanager();
    _uart = serial_manager.find_serial(AP_SerialManager::SerialProtocol_Torqeedo, 0);
    if (_uart == nullptr) {
        return false;
    }
    _uart->begin(EPROPULSION_SERIAL_BAUD);
    _uart->set_flow_control(AP_HAL::UARTDriver::FLOW_CONTROL_DISABLE);
    _uart->set_unbuffered_writes(true);

    // initialise RS485 DE pin (when high, allows send to motor)
    if (_pin_de > -1) {
        hal.gpio->pinMode(_pin_de, HAL_GPIO_OUTPUT);
        hal.gpio->write(_pin_de, 0);
    } else {
        _uart->set_CTS_pin(false);
    }

    return true;
}

// returns true if the driver is enabled
bool AP_EPropulsion::enabled() const
{
    switch ((ConnectionType)_type) {
    case ConnectionType::TYPE_DISABLED:
        return false;
    case ConnectionType::TYPE_THROTTLE_CONTROL_BOARD:
        return true;
    }

    return false;
}

// consume incoming messages, reply with latest desired motor speed
// runs in background thread
void AP_EPropulsion::thread_main()
{
    // initialisation
    if (!init_internals()) {
        return;
    }
    _initialised = true;

    while (true) {
        // 1ms loop delay
        hal.scheduler->delay_microseconds(1000);

        // check if transmit pin should be unset
        check_for_send_end();

        // debug
        uint8_t rcv_debug_buf[20] = {};
        uint8_t rcv_debug_buf_len = 0;

        // parse incoming characters
        uint32_t nbytes = MIN(_uart->available(), 1024U);
        while (nbytes-- > 0) {
            int16_t b = _uart->read();
            if (b >= 0 ) {
                if (parse_byte((uint8_t)b)) {
                    // complete message received, parse it!
                    parse_message();
                }
                // debug
                if (rcv_debug_buf_len < ARRAY_SIZE(rcv_debug_buf)) {
                    rcv_debug_buf[rcv_debug_buf_len++] = b;
                }
            }
        }

        // print debug bug
        if (rcv_debug_buf_len > 0) {
            gcs().send_text(MAV_SEVERITY_CRITICAL,"n:%u %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x",
                //(unsigned long)AP_HAL::millis(),
                (unsigned)rcv_debug_buf_len,
                (unsigned)rcv_debug_buf[0],
                (unsigned)rcv_debug_buf[1],
                (unsigned)rcv_debug_buf[2],
                (unsigned)rcv_debug_buf[3],
                (unsigned)rcv_debug_buf[4],
                (unsigned)rcv_debug_buf[5],
                (unsigned)rcv_debug_buf[6],
                (unsigned)rcv_debug_buf[7],
                (unsigned)rcv_debug_buf[8],
                (unsigned)rcv_debug_buf[9],
                (unsigned)rcv_debug_buf[10],
                (unsigned)rcv_debug_buf[11],
                (unsigned)rcv_debug_buf[12],
                (unsigned)rcv_debug_buf[13],
                (unsigned)rcv_debug_buf[14],
                (unsigned)rcv_debug_buf[15],
                (unsigned)rcv_debug_buf[16],
                (unsigned)rcv_debug_buf[17],
                (unsigned)rcv_debug_buf[18],
                (unsigned)rcv_debug_buf[19]
                );
        }

        // check if should send a reply
        bool log_update = false;
        if (safe_to_send()) {
            // send motor speed
            if (_send_motor_speed) {
                send_motor_speed_cmd();
                _send_motor_speed = false;
                log_update = true;
            } else if (_version_info_reply_cmd > 0) {
                // send version info reply
                send_version_info_reply(_version_info_reply_cmd);
                _version_info_reply_cmd = 0;
            }
        }

        // log high level status and motor speed
        log_EPRP(log_update);
    }
}

// returns true if communicating with the motor
bool AP_EPropulsion::healthy()
{
    if (!_initialised) {
        return false;
    }
    {
        // healthy if both receive and send have occurred in the last 3 seconds
        WITH_SEMAPHORE(_last_healthy_sem);
        const uint32_t now_ms = AP_HAL::millis();
        return ((now_ms - _last_parsed_ms < 3000) && (now_ms - _last_send_motor_ms < 3000));
    }
}

// run pre-arm check.  returns false on failure and fills in failure_msg
// any failure_msg returned will not include a prefix
bool AP_EPropulsion::pre_arm_checks(char *failure_msg, uint8_t failure_msg_len)
{
    // exit immediately if not enabled
    if (!enabled()) {
        return true;
    }

    if (!_initialised) {
        strncpy(failure_msg, "not initialised", failure_msg_len);
        return false;
    }
    if (!healthy()) {
        strncpy(failure_msg, "not healthy", failure_msg_len);
        return false;
    }
    return true;
}

// get latest battery status info.  returns true on success and populates arguments
bool AP_EPropulsion::get_batt_info(float &voltage, float &current_amps, float &temp_C, uint8_t &pct_remaining) const
{
    // use battery info from _battery_status if available
    if ((AP_HAL::millis() - _battery_status.last_update_ms) <= EPROPULSION_BATT_TIMEOUT_MS) {
        voltage = _battery_status.voltage;
        current_amps = _battery_status.current;
        temp_C = _battery_status.temp;
        pct_remaining = _battery_status.percent_remaining;
        return true;
    }

    return false;
}

// process a single byte received on serial port
// return true if a complete message has been received (the message will be held in _received_buff)
bool AP_EPropulsion::parse_byte(uint8_t b)
{
    bool complete_msg_received = false;

    // reset parsing state if this is the first character received for some time
    const uint32_t now_ms = AP_HAL::millis();
    if ((_parse_state != ParseState::WAITING_FOR_HEADER) && (now_ms - _last_received_ms > EPROPULSION_PARSE_TIMEOUT_MS)) {
        _parse_state = ParseState::WAITING_FOR_HEADER;
        _parse_error_count++;
    }
    _last_received_ms = now_ms;

    switch (_parse_state) {
    case ParseState::WAITING_FOR_HEADER:
        if (b == EPROPULSION_PACKET_HEADER) {
            _parse_state = ParseState::WAITING_FOR_ADDR;
        }
        _received_buff_len = 0;
        _crc_expected = 0;
        break;

    case ParseState::WAITING_FOR_ADDR:
        _received_addr = b;
        _crc_expected = 0;
        _parse_state = ParseState::WAITING_FOR_DATALEN;
        break;

    case ParseState::WAITING_FOR_DATALEN:
        if (b > EPROPULSION_MESSAGE_LEN_MAX) {
            // message too long
            _parse_state = ParseState::WAITING_FOR_HEADER;
            _parse_error_count++;
        } else {
            _received_datalen = b;
            _crc_expected ^= b;
            _parse_state = ParseState::WAITING_FOR_DATA;
        }
        break;

    case ParseState::WAITING_FOR_DATA:
        // add to buffer and update crc
        _received_buff[_received_buff_len++] = b;
        _crc_expected ^= b;
        if (_received_buff_len >= _received_datalen) {
            _parse_state = ParseState::WAITING_FOR_CRC;
        }
        break;

    case ParseState::WAITING_FOR_CRC:
        if (b != _crc_expected) {
            // failed crc check
            _parse_state = ParseState::WAITING_FOR_HEADER;
            _parse_error_count++;
        } else {
            _parse_state = ParseState::WAITING_FOR_FOOTER;
        }
        break;

    case ParseState::WAITING_FOR_FOOTER:
        if (b != EPROPULSION_PACKET_FOOTER) {
            _parse_error_count++;
        } else {
            _parse_success_count++;
            {
                // record time of successful receive for health reporting
                WITH_SEMAPHORE(_last_healthy_sem);
                _last_parsed_ms = now_ms;
            }
            complete_msg_received = true;
        }
        _parse_state = ParseState::WAITING_FOR_HEADER;
        break;
    }

    return complete_msg_received;
}

// process message held in _received_buff
void AP_EPropulsion::parse_message()
{
    // message id
    const MsgId msg_id = (MsgId)_received_buff[0];

    // handle messages sent to "remote" (aka tiller)
    switch (msg_id) {

    case MsgId::ADAPTER_BOARD_VERSION_INFO:
    case MsgId::DRIVER_VERSION_INFO:
        if (_received_buff_len != 8) {
            gcs().send_text(MAV_SEVERITY_CRITICAL, "EPrp: ADAPTER_BOARD_VERSION_INFO unexpected len %u", (unsigned)_received_buff_len);
            _parse_error_count++;
        } else {
            VersionInfo vi {};
            vi.product_type = _received_buff[1];
            vi.software_version = _received_buff[2];
            vi.year = _received_buff[3];
            vi.month = _received_buff[4];
            vi.hardware_version = _received_buff[5];
            vi.year2 = _received_buff[6];
            vi.month2 = _received_buff[7];

            // update user
            gcs().send_text(MAV_SEVERITY_CRITICAL, "EPrp: prod=%u swver=%f hwver=%f",
                          (unsigned)vi.product_type, (double)vi.software_version/10.0, vi.hardware_version/10.0);
        }

        // reply with VERSION_INFO_REPLY message
        _version_info_reply_cmd = (uint8_t)msg_id;
        break;

    case MsgId::BATTERY_STATUS_INFO:
        if (_received_buff_len != 11) {
            gcs().send_text(MAV_SEVERITY_CRITICAL, "EPrp: BATTERY_STATUS_INFO unexpected len %u", (unsigned)_received_buff_len);
            _parse_error_count++;
        } else {
            // Data0: Battery temp high byte, signed number
            // Data1: Battery temp low byte
            _battery_status.temp = (int16_t)UINT16_VALUE(_received_buff[1], _received_buff[2]);

            // Data2: Battery voltage high byte (0.1V)
            // Data3: Battery voltage low byte (0.1V)
            _battery_status.voltage = UINT16_VALUE(_received_buff[3], _received_buff[4]) * 0.1;

            // Data4: Current high byte (0.1A)
            // Data5: Current low byte (0.1A)
            _battery_status.current = UINT16_VALUE(_received_buff[5], _received_buff[6]) * 0.1;

            // Data6: Fault alarm high byte
            // Data7: Fault alarm low byte
            _battery_status.fault_alarm = UINT16_VALUE(_received_buff[7], _received_buff[8]) * 0.1;

            // Data8: Type
            _battery_status.type = _received_buff[9];

            // Data9: Capacity percentage
            _battery_status.percent_remaining = _received_buff[10];

            _battery_status.last_update_ms = AP_HAL::millis();

            // reply with THROTTLE_COMMAND
            _send_motor_speed = true;
        }
        break;

    case MsgId::MOTOR_OPERATION_INFO1:
        if (_received_buff_len != 15) {
            gcs().send_text(MAV_SEVERITY_CRITICAL, "EPrp: MOTOR_OPERATION_INFO1 unexpected len %u", (unsigned)_received_buff_len);
            _parse_error_count++;
        } else {
            _motor_operation_info1.flags_value = UINT16_VALUE(_received_buff[2], _received_buff[1]);
            _motor_operation_info1.motor_power = UINT16_VALUE(_received_buff[3], _received_buff[4]);
            _motor_operation_info1.motor_voltage = UINT16_VALUE(_received_buff[5], _received_buff[6]) * 0.1;
            _motor_operation_info1.motor_rpm = UINT16_VALUE(_received_buff[7], _received_buff[8]);
            _motor_operation_info1.phase_current = UINT16_VALUE(_received_buff[9], _received_buff[10]) * 0.1;
            _motor_operation_info1.motor_temp = (int16_t)UINT16_VALUE(_received_buff[11], _received_buff[12]);
            _motor_operation_info1.MOS_temp = (int16_t)UINT16_VALUE(_received_buff[13], _received_buff[14]);
            _motor_operation_info1.last_update_ms = AP_HAL::millis();

            // update esc telem sent to ground station
            update_esc_telem(_motor_operation_info1.motor_rpm,
                             _motor_operation_info1.motor_voltage,
                             _motor_operation_info1.phase_current,
                             _motor_operation_info1.MOS_temp,
                             _motor_operation_info1.motor_temp);

            // log data
            if ((_options & options::LOG) != 0) {
                // @LoggerMessage: EPM1
                // @Description: EPropulsion Motor Info1
                // @Field: TimeUS: Time since system startup
                // @Field: F: Flags bitmask
                // @Field: Pow: Motor power
                // @Field: Volt: Motor voltage
                // @Field: RPM: Motor RPM
                // @Field: Cur: Phase current
                // @Field: MTemp: Motor Temp
                // @Field: MOSTemp: MOS Temp
                AP::logger().Write("EPM1", "TimeUS,F,Pow,Volt,RPM,Cur,MTemp,MOSTemp", "QHHfHfhh",
                                   AP_HAL::micros64(),
                                   _motor_operation_info1.flags_value,
                                   _motor_operation_info1.motor_power,
                                   _motor_operation_info1.motor_voltage,
                                   _motor_operation_info1.motor_rpm,
                                   _motor_operation_info1.phase_current,
                                   _motor_operation_info1.motor_temp,
                                   _motor_operation_info1.MOS_temp);
            }

            // send to GCS
            if ((_options & options::DEBUG_TO_GCS) != 0) {
                gcs().send_text(MAV_SEVERITY_INFO, "EPM1 F:%u Pow:%u V:%4.1f RPM:%u Cur:%4.1f MT:%d ET:%d",
                        (unsigned)_motor_operation_info1.flags_value,
                        (unsigned)_motor_operation_info1.motor_power,
                        (double)_motor_operation_info1.motor_voltage,
                        (unsigned)_motor_operation_info1.motor_rpm,
                        (double)_motor_operation_info1.phase_current,
                        (int)_motor_operation_info1.motor_temp,
                        (int)_motor_operation_info1.MOS_temp);
            }

            // report any errors
            report_error_codes();
        }
        // reply with THROTTLE_COMMAND
        _send_motor_speed = true;
        break;

    case MsgId::MOTOR_OPERATION_INFO2:
        if (_received_buff_len != 13) {
            gcs().send_text(MAV_SEVERITY_CRITICAL, "EPrp: MOTOR_OPERATION_INFO2 unexpected len %u", (unsigned)_received_buff_len);
            _parse_error_count++;
        } else {
            MotorOperationInfo2 motor_operation_info2 = {};
            motor_operation_info2.MOS_temp = (int16_t)UINT16_VALUE(_received_buff[1], _received_buff[2]);
            motor_operation_info2.power_supply_temp = (int16_t)UINT16_VALUE(_received_buff[3], _received_buff[4]);
            motor_operation_info2.bus_current = UINT16_VALUE(_received_buff[5], _received_buff[6]) * 0.1;
            motor_operation_info2.run_time_min = UINT16_VALUE(_received_buff[7], _received_buff[8]);
            motor_operation_info2.total_run_time_min = UINT16_VALUE(_received_buff[9], _received_buff[10]);
            motor_operation_info2.hydrogen_time_min = UINT16_VALUE(_received_buff[11], _received_buff[12]);

            // log data
            if ((_options & options::LOG) != 0) {
                // @LoggerMessage: EPM2
                // @Description: EPropulsion Motor Info1
                // @Field: TimeUS: Time since system startup
                // @Field: MOSTemp: MOS temperature
                // @Field: PTemp: Power supply temperature
                // @Field: BusCur: Bus current
                // @Field: RunT: Run time since power-on in minutes
                // @Field: TotRunT: Run time total in minutes
                // @Field: HydroT: Hydrogeneration time
                AP::logger().Write("EPM2", "TimeUS,MOSTemp,PTemp,BusCur,RunT,TotRunT,HydroT", "QhhfHHH",
                                   AP_HAL::micros64(),
                                   motor_operation_info2.MOS_temp,
                                   motor_operation_info2.power_supply_temp,
                                   motor_operation_info2.bus_current,
                                   motor_operation_info2.run_time_min,
                                   motor_operation_info2.total_run_time_min,
                                   motor_operation_info2.hydrogen_time_min);
            }

            // send to GCS
            if ((_options & options::DEBUG_TO_GCS) != 0) {
                gcs().send_text(MAV_SEVERITY_INFO, "EPM2 MOST:%d PT:%d BCur:%4.1f RT:%u TRT:%u HT:%u",
                        (int)motor_operation_info2.MOS_temp,
                        (int)motor_operation_info2.power_supply_temp,
                        (double)motor_operation_info2.bus_current,
                        (unsigned)motor_operation_info2.run_time_min,
                        (unsigned)motor_operation_info2.total_run_time_min,
                        (unsigned)motor_operation_info2.hydrogen_time_min);
            }
        }
        // reply with THROTTLE_COMMAND
        _send_motor_speed = true;
        break;

    case MsgId::THROTTLE_COMMAND:
    case MsgId::VERSION_INFO_REPLY:
    default:
        // warn of unexpected messages
        gcs().send_text(MAV_SEVERITY_CRITICAL, "EPrp: unexpected msg %u", (unsigned)msg_id);
        break;
    }
}

// set DE Serial CTS pin to enable sending commands to motor
void AP_EPropulsion::send_start()
{
    // set gpio pin or serial port's CTS pin
    if (_pin_de > -1) {
        hal.gpio->write(_pin_de, 1);
    } else {
        _uart->set_CTS_pin(true);
    }
}

// check for timeout after sending and unset pin if required
void AP_EPropulsion::check_for_send_end()
{
    if (_send_delay_us == 0) {
        // not sending
        return;
    }

    if (AP_HAL::micros() - _send_start_us < _send_delay_us) {
        // return if delay has not yet elapsed
        return;
    }
    _send_delay_us = 0;

    // unset gpio or serial port's CTS pin
    if (_pin_de > -1) {
        hal.gpio->write(_pin_de, 0);
    } else {
        _uart->set_CTS_pin(false);
    }
}

// calculate delay require to allow bytes to be sent
uint32_t AP_EPropulsion::calc_send_delay_us(uint8_t num_bytes)
{
    // baud rate of 38400 bits/sec
    // total number of bits = 10 x num_bytes
    // convert from seconds to micros by multiplying by 1,000,000
    // plus additional 300us safety margin
    const uint32_t delay_us = 1e6 * num_bytes * 10 / EPROPULSION_SERIAL_BAUD + 300;
    return delay_us;
}

// send a message with the specified message contents
// msg_contents should the command byte and data bytes only
void AP_EPropulsion::send_message(const uint8_t msg_contents[], uint8_t num_bytes)
{
    // sanity check buffer size
    if (num_bytes > EPROPULSION_MESSAGE_LEN_MAX) {
        INTERNAL_ERROR(AP_InternalError::error_t::invalid_arg_or_result);
        return;
    }

    // buffer for outgoing message
    uint8_t send_buff[EPROPULSION_MESSAGE_LEN_MAX] = {};
    uint8_t send_buff_num_bytes = 0;
    uint8_t crc = 0;

    // add header and address
    send_buff[send_buff_num_bytes++] = EPROPULSION_PACKET_HEADER;
    send_buff[send_buff_num_bytes++] = (uint8_t)MsgAddress::THROTTLE_CONTROL_BOARD;

    // add length
    send_buff[send_buff_num_bytes++] = num_bytes;
    crc ^= num_bytes;

    // add contents
    for (uint8_t i=0; i<num_bytes; i++) {
        send_buff[send_buff_num_bytes++] = msg_contents[i];
        crc ^= msg_contents[i];
    }

    // add crc
    send_buff[send_buff_num_bytes++] = crc;

    // add footer
    send_buff[send_buff_num_bytes++] = EPROPULSION_PACKET_FOOTER;

    // set send pin
    send_start();

    // write message
    _uart->write(send_buff, send_buff_num_bytes);

    // record start and expected delay to send message
    _send_start_us = AP_HAL::micros();
    _send_delay_us = calc_send_delay_us(send_buff_num_bytes);
}

// Send THROTTLE_COMMAND (0x40) as Throttle Control Board to Adapter Board
// Should be sent immediately after receiving one of these message from the Adapter Board
//     Battery status information msg (0x25)
//     Motor status information 1 (0x25)
//     Motor status information 2 (0x27)
//
// THROTTLE_COMMAND (0x40) format
// Byte        Field Definition    Example Value   Comments
// ---------------------------------------------------------------------------------
// byte 0      Head code           0x28
// byte 1      Sender Address      0x04            MotorDriver=0x01, AdapterBoard=0x02, ThrottleControlBoard=0x04
// byte 2      Data length         0x03            command bytes + data bytes
// byte 3      Command             0x40            THROTTLE_COMMAND=0x40
// byte 4      Data0               0x01            forward=1, backward=0
// byte 5      Data1               0x7F            stop=0, full power=7F (e.g. dec 127)
// byte 6      Checksum            0x42            data length ^ command ^ data0 ^ .. ^ dataN
// byte 7      End code            0x29
//
// example message when spinning motor forwards:  "28 04 03 40 01 00 42 29" (0 forward throttle)
// example message when spinning motor forwards:  "28 04 03 40 01 FF 3C 29" (full forward throttle)
// example message when spinning motor forwards:  "28 04 03 40 00 FF 3D 29" (full reverse throttle)
//
// send a motor speed command as a value from -127 to +127
// value is taken directly from SRV_Channel
void AP_EPropulsion::send_motor_speed_cmd()
{
    // calculate desired motor speed
    if (!hal.util->get_soft_armed()) {
        _motor_speed_desired = 0;
    } else {
        // convert throttle output to motor output in range -127 to +127
        // ToDo: convert PWM output to motor output so that SERVOx_MIN, MAX and TRIM take effect
        _motor_speed_desired = constrain_int16(SRV_Channels::get_output_norm(SRV_Channel::Aux_servo_function_t::k_throttle) * 127.0, -127, 127);
    }

    // update limited motor speed
    int16_t mot_speed_limited = calc_motor_speed_limited(_motor_speed_desired);

    // prepare message data (length, direction, speed)
    const uint8_t mot_dir = mot_speed_limited >= 0 ? 0x1 : 0x0;
    const uint8_t mot_speed_abs = (uint8_t)abs(mot_speed_limited);
    uint8_t mot_speed_cmd_buff[] = {(uint8_t)MsgId::THROTTLE_COMMAND, mot_dir, mot_speed_abs};

    // send message and record time of send for health reporting
    send_message(mot_speed_cmd_buff, ARRAY_SIZE(mot_speed_cmd_buff));
    {
        WITH_SEMAPHORE(_last_healthy_sem);
        _last_send_motor_ms = AP_HAL::millis();
    }
}

// calculate the limited motor speed that is sent to the motors
// desired_motor_speed argument and returned value are in the range -127 to 127
int16_t AP_EPropulsion::calc_motor_speed_limited(int16_t desired_motor_speed)
{
    const uint32_t now_ms = AP_HAL::millis();

    // update dir_limit flag for forward-reverse transition delay
    const bool dir_delay_active = is_positive(_dir_delay);
    if (!dir_delay_active) {
        // allow movement in either direction
        _dir_limit = 0;
    } else {
        // by default limit motor direction to previous iteration's direction
        if (is_positive(_motor_speed_limited)) {
            _dir_limit = 1;
        } else if (is_negative(_motor_speed_limited)) {
            _dir_limit = -1;
        } else {
            // motor speed is zero
            if ((_motor_speed_zero_ms != 0) && ((now_ms - _motor_speed_zero_ms) > (_dir_delay * 1000))) {
                // delay has passed so allow movement in either direction
                _dir_limit = 0;
                _motor_speed_zero_ms = 0;
            }
        }
    }

    // calculate upper and lower limits for forward-reverse transition delay
    int16_t lower_limit = -127;
    int16_t upper_limit = 127;
    if (_dir_limit < 0) {
        upper_limit = 0;
    }
    if (_dir_limit > 0) {
        lower_limit = 0;
    }

    // calculate dt since last update
    float dt = (now_ms - _motor_speed_limited_ms) * 0.001f;
    if (dt > 1.0) {
        // after a long delay limit motor output to zero to avoid sudden starts
        lower_limit = 0;
        upper_limit = 0;
    }
    _motor_speed_limited_ms = now_ms;

    // apply slew limit
    if (_slew_time > 0) {
       const float chg_max = 127.0 * dt / _slew_time;
       _motor_speed_limited = constrain_float(desired_motor_speed, _motor_speed_limited - chg_max, _motor_speed_limited + chg_max);
    } else {
        // no slew limit
        _motor_speed_limited = desired_motor_speed;
    }

    // apply upper and lower limits
    _motor_speed_limited = constrain_float(_motor_speed_limited, lower_limit, upper_limit);

    // record time motor speed becomes zero
    if (is_zero(_motor_speed_limited)) {
        if (_motor_speed_zero_ms == 0) {
            _motor_speed_zero_ms = now_ms;
        }
    } else {
        // clear timer
        _motor_speed_zero_ms = 0;
    }

    return (int16_t)_motor_speed_limited;
}

// VERSION_INFO_REPLY (0x44) format sent in response to ADAPTER_BOARD_VERSION_INFO or DRIVER_VERSION_INFO
// Byte        Field Definition    Example Value   Comments
// ---------------------------------------------------------------------------------
// byte 0      Head code           0x28
// byte 1      Sender Address      0x04            MotorDriver=0x01, AdapterBoard=0x02, ThrottleControlBoard=0x04
// byte 2      Data length         0x02            command bytes + data bytes
// byte 3      Command             0x44            VersionInfoReply=0x44
// byte 4      Data0               0x22            ADAPTER_BOARD_VERSION_INFO (0x22) or DRIVER_VERSION_INFO (0x26)
// byte 5      Checksum            0x64            data length ^ command ^ data0
// byte 6      End code            0x29
//
// send version info reply message
// sent in response to a ADAPTER_BOARD_VERSION_INFO or DRIVER_VERSION_INFO received from Adapter board
// command_code should be either ADAPTER_BOARD_VERSION_INFO (0x22) or DRIVER_VERSION_INFO (0x26)
void AP_EPropulsion::send_version_info_reply(uint8_t command_code)
{
    // prepare message data (command, in-response-to-command-id)
    uint8_t version_info_reply_buff[] = {(uint8_t)MsgId::VERSION_INFO_REPLY, _version_info_reply_cmd};

    // send message
    send_message(version_info_reply_buff, ARRAY_SIZE(version_info_reply_buff));
}

// report changes in error codes to user
void AP_EPropulsion::report_error_codes()
{
    const uint32_t now_ms = AP_HAL::millis();

    // skip reporting if no changes in flags and already reported within 10 seconds
    const bool flags_changed = (_motor_operation_info1.flags_value != _motor_operation_info1_flags_prev);
    if (!flags_changed && ((now_ms - _last_error_report_ms) < EPROPULSION_ERROR_REPORT_INTERVAL_MAX_MS)) {
        return;
    }

    // report display system errors
    const char* msg_prefix = "EPrp:";
    if (_motor_operation_info1.flags.motor_blocked) {
        gcs().send_text(MAV_SEVERITY_CRITICAL, "%s motor blocked", msg_prefix);
    }
    if (_motor_operation_info1.flags.motor_overtemp) {
        gcs().send_text(MAV_SEVERITY_CRITICAL, "%s motor overtemp", msg_prefix);
    }
    if (_motor_operation_info1.flags.MOS_overtemp) {
        gcs().send_text(MAV_SEVERITY_CRITICAL, "%s MOS overtemp", msg_prefix);
    }
    if (_motor_operation_info1.flags.overcurrent) {
        gcs().send_text(MAV_SEVERITY_CRITICAL, "%s overcurrent", msg_prefix);
    }
    if (_motor_operation_info1.flags.fault_8301) {
        gcs().send_text(MAV_SEVERITY_CRITICAL, "%s fault_8301", msg_prefix);
    }
    if (_motor_operation_info1.flags.comm_fault) {
        gcs().send_text(MAV_SEVERITY_CRITICAL, "%s comm fault", msg_prefix);
    }
    if (_motor_operation_info1.flags.motor_temp_error) {
        gcs().send_text(MAV_SEVERITY_CRITICAL, "%s motor temp err", msg_prefix);
    }
    if (_motor_operation_info1.flags.MOS_temp_alarm) {
        gcs().send_text(MAV_SEVERITY_CRITICAL, "%s MOS temp alarm", msg_prefix);
    }
    if (_motor_operation_info1.flags.overvoltage) {
        gcs().send_text(MAV_SEVERITY_CRITICAL, "%s overvoltage", msg_prefix);
    }
    if (_motor_operation_info1.flags.undervoltage) {
        gcs().send_text(MAV_SEVERITY_CRITICAL, "%s undervoltage", msg_prefix);
    }
    if (_motor_operation_info1.flags.circuit_failure) {
        gcs().send_text(MAV_SEVERITY_CRITICAL, "%s circuit failure", msg_prefix);
    }
    if (_motor_operation_info1.flags.charging) {
        gcs().send_text(MAV_SEVERITY_INFO, "%s charging", msg_prefix);
    }
    if (_motor_operation_info1.flags.fan_fault) {
        gcs().send_text(MAV_SEVERITY_CRITICAL, "%s fan fault", msg_prefix);
    }
    if (_motor_operation_info1.flags.unused13to15 > 0) {
        gcs().send_text(MAV_SEVERITY_CRITICAL, "%s unknown err", msg_prefix);
    }

    // display OK if all errors cleared
    if ((_motor_operation_info1.flags_value == 0) && (_motor_operation_info1_flags_prev != 0)) {
        gcs().send_text(MAV_SEVERITY_INFO, "%s OK", msg_prefix);
    }

    // record change in state and reporting time
    _motor_operation_info1_flags_prev = _motor_operation_info1.flags_value;
    _last_error_report_ms = now_ms;
}

// output logging and debug messages (if required)
// force_logging should be true if caller wants to ensure the latest status is logged
void AP_EPropulsion::log_EPRP(bool force_logging)
{
    // exit immediately if options are all unset
    if (_options == 0) {
        return;
    }

    // return if not enough time has passed since last output
    const uint32_t now_ms = AP_HAL::millis();
    if (!force_logging && (now_ms - _last_log_EPRP_ms < EPROPULSION_LOG_EPRP_INTERVAL_MS)) {
        return;
    }
    _last_log_EPRP_ms = now_ms;

    const bool health = healthy();
    const int16_t actual_motor_speed = get_motor_speed_limited();

    if ((_options & options::LOG) != 0) {
        // @LoggerMessage: EPRP
        // @Description: EPropulsion Status
        // @Field: TimeUS: Time since system startup
        // @Field: Health: Health
        // @Field: DesMotSpeed: Desired Motor Speed (-127 to 127)
        // @Field: MotSpeed: Motor Speed (-127 to 127)
        // @Field: SuccCnt: Success Count
        // @Field: ErrCnt: Error Count
        AP::logger().Write("EPRP", "TimeUS,Health,DesMotSpeed,MotSpeed,SuccCnt,ErrCnt", "QBhhII",
                           AP_HAL::micros64(),
                           health,
                           _motor_speed_desired,
                           actual_motor_speed,
                           _parse_success_count,
                           _parse_error_count);
    }

    if ((_options & options::DEBUG_TO_GCS) != 0) {
        gcs().send_text(MAV_SEVERITY_INFO,"EPRP h:%u dspd:%d spd:%d succ:%ld err:%ld",
                (unsigned)health,
                (int)_motor_speed_desired,
                (int)actual_motor_speed,
                (unsigned long)_parse_success_count,
                (unsigned long)_parse_error_count);
    }
}

// send ESC telemetry
void AP_EPropulsion::update_esc_telem(float rpm, float voltage, float current_amps, float esc_tempC, float motor_tempC)
{
#if HAL_WITH_ESC_TELEM
    // find servo output channel
    uint8_t telem_esc_index = 0;
    IGNORE_RETURN(SRV_Channels::find_channel(SRV_Channel::Aux_servo_function_t::k_throttle, telem_esc_index));

    // fill in telemetry data structure
    AP_ESC_Telem_Backend::TelemetryData telem_dat {};
    telem_dat.temperature_cdeg = esc_tempC * 100;   // temperature in centi-degrees
    telem_dat.voltage = voltage;                    // voltage in volts
    telem_dat.current = current_amps;               // current in amps
    telem_dat.motor_temp_cdeg = motor_tempC * 100;  // motor temperature in centi-degrees

    // send telem and rpm data
    update_telem_data(telem_esc_index, telem_dat, AP_ESC_Telem_Backend::TelemetryType::TEMPERATURE |
                                                  AP_ESC_Telem_Backend::TelemetryType::MOTOR_TEMPERATURE |
                                                  AP_ESC_Telem_Backend::TelemetryType::CURRENT |
                                                  AP_ESC_Telem_Backend::TelemetryType::VOLTAGE);

    update_rpm(telem_esc_index, rpm);
#endif
}

// get the AP_EPropulsion singleton
AP_EPropulsion *AP_EPropulsion::get_singleton()
{
    return _singleton;
}

AP_EPropulsion *AP_EPropulsion::_singleton = nullptr;

namespace AP {
AP_EPropulsion *epropulsion()
{
    return AP_EPropulsion::get_singleton();
}
};

#endif // HAL_EPROPULSION_ENABLED
