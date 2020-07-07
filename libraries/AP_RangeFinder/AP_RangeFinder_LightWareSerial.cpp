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

   Advanced serial protocol is described here:
       Packet structure: https://github.com/LightWare-Optoelectronics/LW20-docs/blob/master/serial.md
       Commands: https://github.com/LightWare-Optoelectronics/LW20-docs/blob/master/command_detail.md
 */

#include "AP_RangeFinder_LightWareSerial.h"

#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/utility/sparse-endian.h>
#include <AP_Math/crc.h>
#include <ctype.h>
#include <GCS_MAVLink/GCS.h>
#include <AP_Logger/AP_Logger.h>

extern const AP_HAL::HAL& hal;

#define LIGHTWARE_DIST_MAX_CM                       10000
#define LIGHTWARE_OUT_OF_RANGE_ADD_CM               100
#define LIGHTWARE_ADVANCED_HEADER                   0xAA
#define LIGHTWARE_ADVANCED_STREAMING_DISTANCE_DATA  5
#define LIGHTWARE_ADVANCED_DISTANCE_OUTPUT_CONFIG_BITMASK   0xFF    // bitmask of desired distance data (lowest 8 bits set)

// read - return last value measured by sensor
bool AP_RangeFinder_LightWareSerial::get_reading(uint16_t &reading_cm)
{
    if (uart == nullptr) {
        return false;
    }

    float sum = 0;              // sum of all readings taken
    uint16_t valid_count = 0;   // number of valid readings
    uint16_t invalid_count = 0; // number of invalid readings

    // read any available lines from the lidar
    int16_t nbytes = uart->available();
    while (nbytes-- > 0) {
        char c = uart->read();

        // use legacy protocol
        if (protocol_state == ProtocolState::UNKNOWN || protocol_state == ProtocolState::LEGACY) {
            if (c == '\r') {
                linebuf[linebuf_len] = 0;
                const float dist = strtof(linebuf, nullptr);
                if (!is_negative(dist)) {
                    sum += dist;
                    valid_count++;
                    // if still determining protocol update legacy valid count
                    if (protocol_state == ProtocolState::UNKNOWN) {
                        legacy_valid_count++;
                    }
                } else {
                    invalid_count++;
                }
                linebuf_len = 0;
            } else if (isdigit(c) || c == '.' || c == '-') {
                linebuf[linebuf_len++] = c;
                if (linebuf_len == sizeof(linebuf)) {
                    // too long, discard the line
                    linebuf_len = 0;
                }
            }
        }

        // use binary protocol
        if (protocol_state == ProtocolState::UNKNOWN || protocol_state == ProtocolState::BINARY) {
            bool msb_set = BIT_IS_SET(c, 7);
            if (msb_set) {
                // received the high byte
                high_byte = c;
                high_byte_received = true;
            } else {
                // received the low byte which should be second
                if (high_byte_received) {
                    const float dist = (high_byte & 0x7f) << 7 | (c & 0x7f);
                    if (!is_negative(dist)) {
                        sum += dist * 0.01f;
                        valid_count++;
                        // if still determining protocol update binary valid count
                        if (protocol_state == ProtocolState::UNKNOWN) {
                            binary_valid_count++;
                        }
                    } else {
                        invalid_count++;
                    }
                }
                high_byte_received = false;
            }
        }

        // use advanced serial protocol
        if (protocol_state == ProtocolState::ADVANCED) {
            if (advanced_parse_byte(c)) {
                int16_t dist_cm;
                if (advanced_process_message(dist_cm)) {
                    if (dist_cm >= 0) {
                        sum += dist_cm * 0.01f;
                        valid_count++;
                    }
                }
            }
        }
    }

    // protocol set after 10 successful reads
    if (protocol_state == ProtocolState::UNKNOWN) {
        if (binary_valid_count > 10) {
            protocol_state = ProtocolState::BINARY;
        } else if (legacy_valid_count > 10) {
            protocol_state = ProtocolState::LEGACY;
        }
    }

    uint32_t now = AP_HAL::millis();
    if (last_init_ms == 0 ||
        (now - last_init_ms > 1000 &&
         now - state.last_reading_ms > 1000)) {
        // send enough serial transitions to trigger LW20 into serial
        // mode. It starts in dual I2C/serial mode, and wants to see
        // enough transitions to switch into serial mode.
        uart->write("www\r\n");
        last_init_ms = now;
    } else {
        uart->write('d');
    }

    // if using advanced serial protocol, send request to start streaming
    // if distances data messages are not arriving
    if (protocol_state == ProtocolState::ADVANCED &&
        (now - _advance_last_distance_ms > 1000) &&
        (now - _advance_last_req_ms > 1000)) {
        advanced_request_stream_start();
        _advance_last_req_ms = now;
    }

    // return average of all valid readings
    if (valid_count > 0) {
        reading_cm = 100 * sum / valid_count;
        return true;
    }

    // all readings were invalid so return out-of-range-high value
    if (invalid_count > 0) {
        reading_cm = MIN(MAX(LIGHTWARE_DIST_MAX_CM, max_distance_cm() + LIGHTWARE_OUT_OF_RANGE_ADD_CM), UINT16_MAX);
        return true;
    }

    // no readings so return false
    return false;
}

// send message to lidar using advanced serial protocol
void AP_RangeFinder_LightWareSerial::advanced_send_message(AdvancedMessageID msgid, bool write, const uint8_t *payload, uint16_t payload_len)
{
    if ((uart == nullptr) || (payload_len > LIGHTWARE_ADVANCED_PAYLOAD_LEN_MAX)) {
        return;
    }

    // check for sufficient space in outgoing buffer
    if (uart->txspace() < payload_len + 6U) {
        return;
    }

    // write header
    uart->write((uint8_t)LIGHTWARE_ADVANCED_HEADER);
    uint16_t crc = crc_xmodem_update(0, LIGHTWARE_ADVANCED_HEADER);

    // write flags including payload length
    const uint16_t flags = ((payload_len+1) << 6) | (write ? 0x01 : 0);
    uart->write(LOWBYTE(flags));
    crc = crc_xmodem_update(crc, LOWBYTE(flags));
    uart->write(HIGHBYTE(flags));
    crc = crc_xmodem_update(crc, HIGHBYTE(flags));

    // msgid
    uart->write((uint8_t)msgid);
    crc = crc_xmodem_update(crc, (uint8_t)msgid);

    // payload
    if ((payload_len > 0) && (payload != nullptr)) {
        for (uint16_t i = 0; i < payload_len; i++) {
            uart->write(payload[i]);
            crc = crc_xmodem_update(crc, payload[i]);
        }
    }

    // checksum
    uart->write(LOWBYTE(crc));
    uart->write(HIGHBYTE(crc));
}

// request sending of distance data using advanced serial protocol
void AP_RangeFinder_LightWareSerial::advanced_request_stream_start()
{
    // request product name to switch sensor to advanced serial protocol
    advanced_send_message(AdvancedMessageID::PRODUCT_NAME, false, nullptr, 0);

    // request which data we would like in distance data messages
    uint8_t desired_data_buff[] = {LIGHTWARE_ADVANCED_DISTANCE_OUTPUT_CONFIG_BITMASK, 0x0, 0x0, 0x0};
    advanced_send_message(AdvancedMessageID::DISTANCE_OUTPUT_CONFIG, true, desired_data_buff, ARRAY_SIZE(desired_data_buff));

    // request streaming to start
    const le32_t stream_req = htole32(LIGHTWARE_ADVANCED_STREAMING_DISTANCE_DATA);
    advanced_send_message(AdvancedMessageID::STREAM, true, (const uint8_t*)&stream_req, sizeof(stream_req));
}

// process one byte received on serial port
// returns true if a complete message has been received, message is stored in _advanced_msg structure
bool AP_RangeFinder_LightWareSerial::advanced_parse_byte(uint8_t b)
{
    // check that payload buffer is large enough
    static_assert(ARRAY_SIZE(_advanced_msg.payload) == LIGHTWARE_ADVANCED_PAYLOAD_LEN_MAX, "AP_Proximity_LightwareSF40C: check _advanced_msg.payload array size ");

    // process byte depending upon current state
    switch (_advanced_parse_state) {

    case AdvancedParseState::HEADER:
        if (b == LIGHTWARE_ADVANCED_HEADER) {
            _advanced_msg.crc_expected = crc_xmodem_update(0, b);
            _advanced_parse_state = AdvancedParseState::FLAGS_L;
        }
        break;

    case AdvancedParseState::FLAGS_L:
        _advanced_msg.flags_low = b;
        _advanced_msg.crc_expected = crc_xmodem_update(_advanced_msg.crc_expected, b);
        _advanced_parse_state = AdvancedParseState::FLAGS_H;
        break;

    case AdvancedParseState::FLAGS_H:
        _advanced_msg.flags_high = b;
        _advanced_msg.crc_expected = crc_xmodem_update(_advanced_msg.crc_expected, b);
        _advanced_msg.payload_len = (UINT16_VALUE(_advanced_msg.flags_high, _advanced_msg.flags_low) >> 6) & 0x3FF;
        if ((_advanced_msg.payload_len == 0) || (_advanced_msg.payload_len > LIGHTWARE_ADVANCED_PAYLOAD_LEN_MAX)) {
            // invalid payload length, abandon message
            _advanced_parse_state = AdvancedParseState::HEADER;
        } else {
            _advanced_parse_state = AdvancedParseState::MSG_ID;
        }
        break;

    case AdvancedParseState::MSG_ID:
        _advanced_msg.msgid = (AdvancedMessageID)b;
        _advanced_msg.crc_expected = crc_xmodem_update(_advanced_msg.crc_expected, b);
        if (_advanced_msg.payload_len > 1) {
            _advanced_parse_state = AdvancedParseState::PAYLOAD;
        } else {
            _advanced_parse_state = AdvancedParseState::CRC_L;
        }
        _advanced_msg.payload_recv = 0;
        break;

    case AdvancedParseState::PAYLOAD:
        if (_advanced_msg.payload_recv < (_advanced_msg.payload_len - 1)) {
            _advanced_msg.payload[_advanced_msg.payload_recv] = b;
            _advanced_msg.payload_recv++;
            _advanced_msg.crc_expected = crc_xmodem_update(_advanced_msg.crc_expected, b);
        }
        if (_advanced_msg.payload_recv >= (_advanced_msg.payload_len - 1)) {
            _advanced_parse_state = AdvancedParseState::CRC_L;
        }
        break;

    case AdvancedParseState::CRC_L:
        _advanced_msg.crc_low = b;
        _advanced_parse_state = AdvancedParseState::CRC_H;
        break;

    case AdvancedParseState::CRC_H:
        _advanced_parse_state = AdvancedParseState::HEADER;
        _advanced_msg.crc_high = b;
        const bool crc_ok = _advanced_msg.crc_expected == UINT16_VALUE(_advanced_msg.crc_high, _advanced_msg.crc_low);
        return crc_ok;
    }

    return false;
}

// process the latest message held in the _advanced_msg structure
// returns true if a distance was received and places distance in distance_cm argument
bool AP_RangeFinder_LightWareSerial::advanced_process_message(int16_t &distance_cm)
{
    // process payload
    switch (_advanced_msg.msgid) {

    case AdvancedMessageID::DISTANCE_OUTPUT:
    case AdvancedMessageID::DISTANCE_OUTPUT_CONFIG:
    {
        // consume only the lowest byte
        _advanced_sensor_state.distance_output_config = _advanced_msg.payload[0];
        break;
    }

    case AdvancedMessageID::STREAM:
        if (_advanced_msg.payload_recv == sizeof(uint32_t)) {
            // valid stream values are 0:disabled, 1:raw distance data, 5:distance data, 10:signal probability data
            const uint32_t stream_value = buff_to_uint32(_advanced_msg.payload[0], _advanced_msg.payload[1], _advanced_msg.payload[2], _advanced_msg.payload[3]);
            _advanced_sensor_state.streaming = (stream_value == LIGHTWARE_ADVANCED_STREAMING_DISTANCE_DATA);
        }
        break;

    case AdvancedMessageID::DISTANCE_DATA: {
        // we should have received eight 2byte values
        if (_advanced_msg.payload_recv != 16) {
            return false;
        }

        // places extra data into the correct place in _advanced_distance_output
        _advanced_distance_output.first_dist_raw_cm = buff_to_int16(_advanced_msg.payload[0], _advanced_msg.payload[1]);
        _advanced_distance_output.first_dist_filt_cm = buff_to_int16(_advanced_msg.payload[2], _advanced_msg.payload[3]);
        _advanced_distance_output.first_strength_pct  = buff_to_int16(_advanced_msg.payload[4], _advanced_msg.payload[5]);
        _advanced_distance_output.last_dist_raw_cm = buff_to_int16(_advanced_msg.payload[6], _advanced_msg.payload[7]);
        _advanced_distance_output.last_dist_filt_cm = buff_to_int16(_advanced_msg.payload[8], _advanced_msg.payload[9]);
        _advanced_distance_output.last_strength_pct = buff_to_int16(_advanced_msg.payload[10], _advanced_msg.payload[11]);
        _advanced_distance_output.noise = buff_to_int16(_advanced_msg.payload[12], _advanced_msg.payload[13]);
        _advanced_distance_output.temp_cd = buff_to_int16(_advanced_msg.payload[14], _advanced_msg.payload[15]);

#ifndef HAL_BUILD_AP_PERIPH
        // debug
        static uint32_t last_print_ms = 0;
        uint32_t now_ms = AP_HAL::millis();
        if (now_ms - last_print_ms > 1000) {
            last_print_ms = now_ms;
            gcs().send_text(MAV_SEVERITY_CRITICAL,"first raw:%d filt:%d str:%d",
                    (int)_advanced_distance_output.first_dist_raw_cm,
                    (int)_advanced_distance_output.first_dist_filt_cm,
                    (int)_advanced_distance_output.first_strength_pct);
            gcs().send_text(MAV_SEVERITY_CRITICAL,"last raw:%d filt:%d str:%d",
                    (int)_advanced_distance_output.last_dist_raw_cm,
                    (int)_advanced_distance_output.last_dist_filt_cm,
                    (int)_advanced_distance_output.last_strength_pct);
            gcs().send_text(MAV_SEVERITY_CRITICAL,"noise:%d, temp:%d", (int)_advanced_distance_output.noise, (int)_advanced_distance_output.temp_cd);
        }
#endif

        // return first distance (filtered)
        distance_cm = _advanced_distance_output.first_dist_filt_cm;
        _advance_last_distance_ms = AP_HAL::millis();

        // log extra data held in _advanced_distance_output
        Log_RFNE();

        // return true to indicate new distance has been consumed
        return true;
    }

    // unsupported messages
    case AdvancedMessageID::PRODUCT_NAME:
    case AdvancedMessageID::HARDWARE_VERSION:
    case AdvancedMessageID::FIRMWARE_VERSION:
    case AdvancedMessageID::SERIAL_NUMBER:
    case AdvancedMessageID::TEXT_MESSAGE:
    case AdvancedMessageID::USER_DATA:
    case AdvancedMessageID::TOKEN:
    case AdvancedMessageID::SAVE_PARAMETERS:
    case AdvancedMessageID::RESET:
    case AdvancedMessageID::STAGE_FIRMWARE:
    case AdvancedMessageID::COMMIT_FIRMWARE:
    case AdvancedMessageID::INCOMING_VOLTAGE:
    case AdvancedMessageID::COMMUNICATION_MODE:
    case AdvancedMessageID::STREAM_STATISTICS:
    case AdvancedMessageID::STATISTICS_DATA:
    case AdvancedMessageID::RAW_DISTANCE_DATA:
    case AdvancedMessageID::SIGNAL_PROBABILITY_DATA:
    case AdvancedMessageID::LASER_FIRING:
    case AdvancedMessageID::TEMPERATURE:
    case AdvancedMessageID::HIGH_SPEED_MODE:
    case AdvancedMessageID::NOISE:
    case AdvancedMessageID::ALARM_STATUS:
    case AdvancedMessageID::BAUD_RATE:
    case AdvancedMessageID::I2C_ADDRESS:
    case AdvancedMessageID::MEASUREMENT_MODE:
    case AdvancedMessageID::ZERO_OFFSET:
    case AdvancedMessageID::LOST_SIGNAL_COUNTER:
    case AdvancedMessageID::ALARM_A_DISTANCE:
    case AdvancedMessageID::ALARM_B_DISTANCE:
    case AdvancedMessageID::ALARM_HYSTERESIS:
    case AdvancedMessageID::SERVO_CONNECTED:
    case AdvancedMessageID::SERVO_SCANNING:
    case AdvancedMessageID::SERVO_POSITION:
    case AdvancedMessageID::SERVO_PWM_LOW:
    case AdvancedMessageID::SERVO_PWM_HIGH:
    case AdvancedMessageID::SERVO_PWM_SCALE:
    case AdvancedMessageID::SERVO_SCAN_TYPE:
    case AdvancedMessageID::SERVO_SCAN_SPEED:
    case AdvancedMessageID::SERVO_LAG:
    case AdvancedMessageID::SERVO_FOV_LOW:
    case AdvancedMessageID::SERVO_FOV_HIGH:
    case AdvancedMessageID::SERVO_ALARM_A_LOW:
    case AdvancedMessageID::SERVO_ALARM_A_HIGH:
    case AdvancedMessageID::SERVO_ALARM_B_LOW:
    case AdvancedMessageID::SERVO_ALARM_B_HIGH:
        break;
    }

    return false;
}

// convert buffer to uint32, uint16
uint32_t AP_RangeFinder_LightWareSerial::buff_to_uint32(uint8_t b0, uint8_t b1, uint8_t b2, uint8_t b3) const
{
    uint32_t leval = (uint32_t)b0 | (uint32_t)b1 << 8 | (uint32_t)b2 << 16 | (uint32_t)b3 << 24;
    return leval;
}

int16_t AP_RangeFinder_LightWareSerial::buff_to_int16(uint8_t b0, uint8_t b1) const
{
    uint16_t leval = (uint16_t)b0 | (uint16_t)b1 << 8;
    return (int16_t)leval;
}

// log extra distances, strengths, noise and temperature
void AP_RangeFinder_LightWareSerial::Log_RFNE()
{
#ifndef HAL_BUILD_AP_PERIPH
    const struct log_RFNE pkt = {
            LOG_PACKET_HEADER_INIT(LOG_RFNE_MSG),
            time_us             : AP_HAL::micros64(),
            instance            : 0,    // we do not know our instance
            first_dist_raw_cm   : _advanced_distance_output.first_dist_raw_cm,
            first_dist_filt_cm  : _advanced_distance_output.first_dist_filt_cm,
            first_strength_pct  : _advanced_distance_output.first_strength_pct,
            last_dist_raw_cm    : _advanced_distance_output.last_dist_raw_cm,
            last_dist_filt_cm   : _advanced_distance_output.last_dist_filt_cm,
            last_strength_pct   : _advanced_distance_output.last_strength_pct,
            noise               : _advanced_distance_output.noise,
            temp_cd             : _advanced_distance_output.temp_cd
    };

    AP::logger().WriteBlock(&pkt, sizeof(pkt));
#endif
}
