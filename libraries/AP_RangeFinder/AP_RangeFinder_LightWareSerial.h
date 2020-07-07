#pragma once

#include "AP_RangeFinder.h"
#include "AP_RangeFinder_Backend_Serial.h"

#define LIGHTWARE_ADVANCED_PAYLOAD_LEN_MAX  30      // packets are payload + 6 bytes (header, flags low+high, message id, payload, crc low+high)

class AP_RangeFinder_LightWareSerial : public AP_RangeFinder_Backend_Serial
{

public:

    using AP_RangeFinder_Backend_Serial::AP_RangeFinder_Backend_Serial;

protected:

    MAV_DISTANCE_SENSOR _get_mav_distance_sensor_type() const override {
        return MAV_DISTANCE_SENSOR_LASER;
    }

private:
    // get a reading
    bool get_reading(uint16_t &reading_cm) override;

    // advanced serial protocol message ids
    enum class AdvancedMessageID : uint8_t {
        PRODUCT_NAME = 0,
        HARDWARE_VERSION = 1,
        FIRMWARE_VERSION = 2,
        SERIAL_NUMBER = 3,
        TEXT_MESSAGE = 7,
        USER_DATA = 9,
        TOKEN = 10,
        SAVE_PARAMETERS = 12,
        RESET = 14,
        STAGE_FIRMWARE = 16,
        COMMIT_FIRMWARE = 17,
        INCOMING_VOLTAGE = 20,
        DISTANCE_OUTPUT = 27,
        COMMUNICATION_MODE = 28,
        DISTANCE_OUTPUT_CONFIG = 29,
        STREAM = 30,
        STREAM_STATISTICS = 31,
        STATISTICS_DATA = 35,
        RAW_DISTANCE_DATA = 40,
        SIGNAL_PROBABILITY_DATA = 43,
        DISTANCE_DATA = 44,
        LASER_FIRING = 50,
        TEMPERATURE = 55,
        HIGH_SPEED_MODE = 70,
        NOISE = 85,
        ALARM_STATUS = 88,
        BAUD_RATE = 90,
        I2C_ADDRESS = 91,
        MEASUREMENT_MODE = 93,
        ZERO_OFFSET = 94,
        LOST_SIGNAL_COUNTER = 95,
        ALARM_A_DISTANCE = 96,
        ALARM_B_DISTANCE = 97,
        ALARM_HYSTERESIS = 98,
        SERVO_CONNECTED = 121,
        SERVO_SCANNING = 122,
        SERVO_POSITION = 123,
        SERVO_PWM_LOW = 124,
        SERVO_PWM_HIGH = 125,
        SERVO_PWM_SCALE = 126,
        SERVO_SCAN_TYPE = 127,
        SERVO_SCAN_SPEED = 128,
        SERVO_LAG = 129,
        SERVO_FOV_LOW = 130,
        SERVO_FOV_HIGH = 131,
        SERVO_ALARM_A_LOW = 132,
        SERVO_ALARM_A_HIGH = 133,
        SERVO_ALARM_B_LOW = 134,
        SERVO_ALARM_B_HIGH = 135,
    };

    // send message to sensor using advanced serial protocol
    void advanced_send_message(AdvancedMessageID msgid, bool write, const uint8_t *payload, uint16_t payload_len);

    // request sending of distance data using advanced serial protocol
    void advanced_request_stream_start();

    // process one byte received on serial port using advanced serial protocol
    // returns true if a complete message has been received, message is stored in _advanced_msg structure
    bool advanced_parse_byte(uint8_t b);

    // process the latest message held in the _advanced_msg structure
    // returns true if a distance was received and places distance in distance_cm argument
    bool advanced_process_message(int16_t &distance_cm);

    // convert buffer to uint32, uint16
    uint32_t buff_to_uint32(uint8_t b0, uint8_t b1, uint8_t b2, uint8_t b3) const;
    int16_t buff_to_int16(uint8_t b0, uint8_t b1) const;

    // log extra distances, strengths, noise and temperature
    void Log_RFNE();

    enum class AdvancedParseState : uint8_t {
        HEADER = 0,
        FLAGS_L,
        FLAGS_H,
        MSG_ID,
        PAYLOAD,
        CRC_L,
        CRC_H
    } _advanced_parse_state;
    uint32_t _advance_last_req_ms;      // system time of last request for distance data to be streamed
    uint32_t _advance_last_distance_ms; // system time of last distance data packet received

    // sensor state retrieved using advanced serial protocol
    struct {
        bool streaming;                 // true if distance messages are being streamed
        uint8_t distance_output_config; // bitmask of extra info being sent
    } _advanced_sensor_state;

    // extra data retrieved from sensor in distance_output messages
    struct {
        int16_t first_dist_raw_cm;
        int16_t first_dist_filt_cm;
        int16_t first_strength_pct;
        int16_t last_dist_raw_cm;
        int16_t last_dist_filt_cm;
        int16_t last_strength_pct;
        int16_t noise;
        int16_t temp_cd;
    } _advanced_distance_output;

    // latest advanced message contents
    struct {
        uint8_t flags_low;      // flags low byte
        uint8_t flags_high;     // flags high byte
        uint16_t payload_len;   // latest message payload length including msgid byte (i.e. bytes in payload +1)
        uint8_t payload[LIGHTWARE_ADVANCED_PAYLOAD_LEN_MAX];    // payload
        AdvancedMessageID msgid;// latest message's message id
        uint16_t payload_recv;  // number of message's payload bytes received so far (does not include msgid byte)
        uint8_t crc_low;        // crc low byte
        uint8_t crc_high;       // crc high byte
        uint16_t crc_expected;  // latest message's expected crc
    } _advanced_msg;

    char linebuf[10];           // legacy protocol buffer
    uint8_t linebuf_len;        // legacy protocol buffer length
    uint32_t last_init_ms;      // init time used to switch lw20 to serial mode
    uint8_t high_byte;          // binary protocol high byte
    bool high_byte_received;    // true if high byte has been received

    // automatic protocol decision variables
    enum class ProtocolState {
        UNKNOWN,    // the protocol used is not yet known
        LEGACY,     // legacy protocol, distances are sent as strings
        BINARY,     // binary protocol, distances are sent using two bytes
        ADVANCED    // advanced serial protocol, allows changing settings, packets include header, payload and checksum
    } protocol_state = ProtocolState::ADVANCED;
    uint8_t legacy_valid_count;
    uint8_t binary_valid_count;
};
