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
   This driver supports communicating with EPropulsion motors that implement the Evo RS485 protocol
   including the Spirit 1.0, Navy 3.0 and Navy 6.0

   The autopilot should be connected to the communication connector as described on the ArduPilot wiki.
   https://ardupilot.org/rover/docs/common-epropulsion.html
   The protocol is serial over RS-485 meaning that a serial to RS-485 converter is required.

    Communication between the components is always initiated by the master (aka communication adapter board)
    with replies sent from the slaves (e.g. throttle control board, battery or motor) within 5ms

    Baud rate is 38400 bits/sec

    The Master (aka Adapter Board) sends messages every 50ms to us (the Throttle Control Board).
    The possible messages we may receive and the expected response are
        - Battery Status Information (0x25).  We should respond with a Throttle Command (0x40)
        - Adapter Board Version information (0x22).  We should respond with a Version Info Reply (0x44) message
        - (Motor) Driver Version information (0x26).  We should respond with a Version Info Reply (0x44) message
        - Motor Operation Info1 (aka Motor Status Information 1) (0x25).  We should respond with a Throttle Command (0x40)
        - Motor Operation Info2 (aka Motor Status Information 2) (0x25).  We should respond with a Throttle Command (0x40)

    Example "Throttle control board" sends message to control motor speed
    Byte        Field Definition    Example Value   Comments
    ---------------------------------------------------------------------------------
    byte 0      Head code           0x28
    byte 1      Sender Address      0x04            MotorDriver=0x01, AdapterBoard=0x02, ThrottleControlBoard=0x04
    byte 2      Data length         0x03            command byte + data bytes
    byte 3      Command             0x40            ThrottleCommand=0x40, VersionInfoReply=0x44
    byte 4      Data0               0x01            forward=1, backward=0
    byte 5      Data1               0x10            stop=0, full power=127
    byte 6      Checksum            0x52            Data length ^ Command ^ Data0 ^ .. ^ DataN
    byte 7      End code            0x29

   More details of the protocol are available from EPropulsion after signing an NDA.
 */

#pragma once

#include <AP_HAL/AP_HAL.h>

#ifndef HAL_EPROPULSION_ENABLED
#define HAL_EPROPULSION_ENABLED !HAL_MINIMIZE_FEATURES && (BOARD_FLASH_SIZE > 1024) && !defined(HAL_BUILD_AP_PERIPH)
#endif

#if HAL_EPROPULSION_ENABLED

#include <AP_ESC_Telem/AP_ESC_Telem_Backend.h>
#include <AP_Param/AP_Param.h>

#define EPROPULSION_MESSAGE_LEN_MAX    20  // messages are no more than 20 bytes

class AP_EPropulsion : public AP_ESC_Telem_Backend {
public:
    AP_EPropulsion();

    CLASS_NO_COPY(AP_EPropulsion);

    static AP_EPropulsion* get_singleton();

    // initialise driver
    void init();

    // consume incoming messages from motor, reply with latest motor speed
    // runs in background thread
    void thread_main();

    // returns true if communicating with the motor
    bool healthy();

    // run pre-arm check.  returns false on failure and fills in failure_msg
    // any failure_msg returned will not include a prefix
    bool pre_arm_checks(char *failure_msg, uint8_t failure_msg_len);

    // get latest battery status info.  returns true on success and populates arguments
    bool get_batt_info(float &voltage, float &current_amps, float &temp_C, uint8_t &pct_remaining) const WARN_IF_UNUSED;
    bool get_batt_capacity_Ah(uint16_t &amp_hours) const { return false; }

    static const struct AP_Param::GroupInfo var_info[];

private:

    // message addresses
    enum class MsgAddress : uint8_t {
        MOTOR_DRIVER = 0x01,
        ADAPTER_BOARD = 0x02,
        THROTTLE_CONTROL_BOARD = 0x04
        // battery could be anything?
    };

    // Message Ids
    enum class MsgId : uint8_t {
        ADAPTER_BOARD_VERSION_INFO = 0x22,  // sent from adapter board, throttle control board should reply with VERSION_INFO_REPLY
        BATTERY_STATUS_INFO = 0x23,         // sent from adapter board, throttle control board should reply with THROTTLE_COMMAND
        MOTOR_OPERATION_INFO1 = 0x25,       // sent from Motor Driver and forwarded by the Adapter Board to the Throttle Control Board
        DRIVER_VERSION_INFO = 0x26,         // sent from adapter board, throttle control board should reply with VERSION_INFO_REPLY
        MOTOR_OPERATION_INFO2 = 0x27,       // sent from Motor Driver and forwarded by the Adapter Board to the Throttle Control Board
        THROTTLE_COMMAND = 0x40,            // sent by throttle control board
        VERSION_INFO_REPLY = 0x44           // sent by throttle control board
    };

    enum class ParseState {
        WAITING_FOR_HEADER = 0,
        WAITING_FOR_ADDR,
        WAITING_FOR_DATALEN,
        WAITING_FOR_DATA,
        WAITING_FOR_CRC,
        WAITING_FOR_FOOTER,
    };

    // TYPE parameter values
    enum class ConnectionType : uint8_t {
        TYPE_DISABLED = 0,
        TYPE_THROTTLE_CONTROL_BOARD = 1
    };

    // OPTIONS parameter values
    enum options {
        LOG             = 1<<0,
        DEBUG_TO_GCS    = 1<<1,
    };

    // initialise serial port and gpio pins (run from background thread)
    // returns true on success
    bool init_internals();

    // returns true if the driver is enabled
    bool enabled() const;

    // process a single byte received on serial port
    // return true if a complete message has been received (the message will be held in _received_buff)
    bool parse_byte(uint8_t b);

    // process message held in _received_buff
    void parse_message();

    // returns true if it is safe to send a message
    bool safe_to_send() const { return (_send_delay_us == 0); }

    // set pin to enable sending a message
    void send_start();

    // check for timeout after sending a message and unset pin if required
    void check_for_send_end();

    // calculate delay required to allow message to be completely sent
    uint32_t calc_send_delay_us(uint8_t num_bytes);

    // send a message with the specified message contents
    // msg_contents should the command byte and data bytes only
    void send_message(const uint8_t msg_contents[], uint8_t num_bytes);

    // send a motor speed command as a value from -127 to +127
    // value is taken directly from SRV_Channel
    void send_motor_speed_cmd();

    // calculate the limited motor speed that is sent to the motors
    // desired_motor_speed argument and returned value are in the range -1000 to 1000
    int16_t calc_motor_speed_limited(int16_t desired_motor_speed);
    int16_t get_motor_speed_limited() const { return (int16_t)_motor_speed_limited; }

    // send version info reply message
    // sent in response to a ADAPTER_BOARD_VERSION_INFO or DRIVER_VERSION_INFO received from Adapter board
    // command_code should be either ADAPTER_BOARD_VERSION_INFO (0x22) or DRIVER_VERSION_INFO (0x26)
    void send_version_info_reply(uint8_t command_code);

    // report changes in error codes to user
    void report_error_codes();

    // log EPRP message which holds high level status and latest desired motor speed
    // force_logging should be true to immediately write log bypassing timing check to avoid spamming
    void log_EPRP(bool force_logging);

    // send ESC telemetry
    void update_esc_telem(float rpm, float voltage, float current_amps, float esc_tempC, float motor_tempC);

    // parameters
    AP_Enum<ConnectionType> _type;      // connector type used (0:disabled, 1:tiller connector, 2: motor connector)
    AP_Int8 _pin_de;        // Pin number connected to RS485 to Serial converter's DE pin. -1 to disable sending commands to motor
    AP_Int16 _options;      // options bitmask
    AP_Float _slew_time;    // slew rate specified as the minimum number of seconds required to increase the throttle from 0 to 100%.  A value of zero disables the limit
    AP_Float _dir_delay;    // direction change delay.  output will remain at zero for this many seconds when transitioning between forward and backwards rotation

    // members
    AP_HAL::UARTDriver *_uart;      // serial port to communicate with motor
    bool _initialised;              // true once driver has been initialised
    bool _send_motor_speed;         // true if motor speed should be sent at next opportunity
    int16_t _motor_speed_desired;   // desired motor speed in range -127 to +127 (set from within update method)
    uint32_t _last_send_motor_ms;   // system time (in millis) last motor speed command was sent (used for health reporting)
    uint32_t _send_start_us;        // system time (in micros) when last message started being sent (used for timing to unset DE pin)
    uint32_t _send_delay_us;        // delay (in micros) to allow bytes to be sent after which pin can be unset.  0 if not delaying

    // motor speed limit variables
    float _motor_speed_limited;     // limited desired motor speed. this value is actually sent to the motor
    uint32_t _motor_speed_limited_ms; // system time that _motor_speed_limited was last updated
    int8_t _dir_limit;              // acceptable directions for output to motor (+1 = positive OK, -1 = negative OK, 0 = either positive or negative OK)
    uint32_t _motor_speed_zero_ms;  // system time that _motor_speed_limited reached zero.  0 if currently not zero

    // version info reply variables
    uint8_t _version_info_reply_cmd;// non-zero value means driver should reply to either ADAPTER_BOARD_VERSION_INFO (0x22) or DRIVER_VERSION_INFO (0x26)

    // health reporting
    HAL_Semaphore _last_healthy_sem;// semaphore protecting reading and updating of _last_send_motor_ms and _last_parsed_ms
    uint32_t _last_log_EPRP_ms;     // system time (in millis) that EPRP was last logged

    // message parsing members
    ParseState _parse_state;        // current state of parsing
    uint32_t _parse_error_count;    // total number of parsing errors (for reporting)
    uint32_t _parse_success_count;  // number of messages successfully parsed (for reporting)
    uint8_t _received_addr;         // address received (unused)
    uint8_t _received_datalen;      // datalen received
    uint8_t _received_buff[EPROPULSION_MESSAGE_LEN_MAX];   // characters received
    uint8_t _received_buff_len;     // number of characters in _received_buff
    uint32_t _last_received_ms;     // system time (in millis) that a byte was last received
    uint32_t _last_parsed_ms;       // system time (in millis) that a message was successfully parsed (for health reporting)
    uint8_t _crc_expected;          // expected crc.  this is compared to the actual crc received to confirm an uncorrupted message

    // Adapter board information received via ADAPTER_BOARD_VERSION_INFO (0x22) OR
    // Motor driver version info received via DRIVER_VERSION_INFO (0x26)
    struct VersionInfo {
        uint8_t product_type;       // 1=Navy6, 2=Navy3, 3=Spirit1
        uint8_t software_version;   // software version. e.g. "25" means 2.5
        uint8_t year;               // e.g. "20" = 2020
        uint8_t month;              // e.g. "11" = November
        uint8_t hardware_version;   // hardware version.  e.g. "25" means 2.5
        uint8_t year2;              // e.g. "20" = 2020
        uint8_t month2;             // e.g. "11" = November
    };

    // Battery status received via BATTERY_STATUS_INFO (0x23)
    struct {
        int16_t temp;               // battery temp in C
        float voltage;              // battery voltage in volts
        float current;              // battery current in amps
        uint16_t fault_alarm;       // battery fault alarm code
        uint8_t type;               // battery type
        uint8_t percent_remaining;  // battery percentage remaining
        uint32_t last_update_ms;    // system time that battery status was last received
    } _battery_status;

    // Motor operation info 1 received via MOTOR_OPERATION_INFO1 (0x25)
    struct {
        union PACKED {
            struct {
                uint16_t motor_blocked      : 1;    // Data0 bit0, motor blocked
                uint16_t motor_overtemp     : 1;    // Data0 bit1, motor over temperature
                uint16_t MOS_overtemp       : 1;    // Data0 bit2, MOS over temperature
                uint16_t overcurrent        : 1;    // Data0 bit3, over current
                uint16_t fault_8301         : 1;    // Data0 bit4, 8301 fault
                uint16_t comm_fault         : 1;    // Data0 bit5, communication fault
                uint16_t motor_temp_error   : 1;    // Data0 bit6, motor temperature error
                uint16_t MOS_temp_alarm     : 1;    // Data0 bit7, MOS temperature alarm
                uint16_t overvoltage        : 1;    // Data1 bit0, over voltage
                uint16_t undervoltage       : 1;    // Data1 bit1, under voltage
                uint16_t circuit_failure    : 1;    // Data1 bit2, circuit failiure
                uint16_t charging           : 1;    // Data1 bit3, charging
                uint16_t fan_fault          : 1;    // Data1 bit4, fan fault
                uint16_t unused13to15       : 3;    // Data1 bit5 ~ bit7, unused
            } flags;
            uint16_t flags_value;
        };
        uint16_t motor_power;           // motor power (in watts?)
        float motor_voltage;            // motor voltage in volts
        uint16_t motor_rpm;             // motor RPM
        float phase_current;            // phase current in amps
        int16_t motor_temp;             // motor temperature in C
        int16_t MOS_temp;               // MOS temperature in C
        uint32_t last_update_ms;    // system time that battery status was last received
    } _motor_operation_info1;

    // error reporting
    uint16_t _motor_operation_info1_flags_prev; // backup of Motor operation info 1 flags
    uint32_t _last_error_report_ms;             // system time that flag changes were last reported (used to prevent spamming user)

    // Motor operation info 2 received via MOTOR_OPERATION_INFO2 (0x27)
    struct MotorOperationInfo2 {
        int16_t MOS_temp;               // MOS temperature in C
        int16_t power_supply_temp;      // power supply temperature in C
        float bus_current;              // bus current in amps
        uint16_t run_time_min;          // single operation time in minutes
        uint16_t total_run_time_min;    // total operation time in minutes
        uint16_t hydrogen_time_min;     // hydrogeneration time in minutes
    };

    static AP_EPropulsion *_singleton;
};

namespace AP {
    AP_EPropulsion *epropulsion();
};

#endif // HAL_EPROPULSION_ENABLED
