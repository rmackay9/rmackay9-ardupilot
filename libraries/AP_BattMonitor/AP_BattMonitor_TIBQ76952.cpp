#include "AP_BattMonitor_config.h"

#if AP_BATTERY_TIBQ76952_ENABLED

#include <GCS_MAVLink/GCS.h>
#include <AP_HAL/utility/sparse-endian.h>

#include "AP_BattMonitor_TIBQ76952.h"

extern const AP_HAL::HAL& hal;

/*
  TI bq76952 register definitions from datasheet SLUUBY2B
  Uses direct commands (7-bit addresses) for voltage readings
  Data stored in little endian byte order
 */
// Direct commands for voltage measurements (from Table 4-1)
#define REG_OTP_CHECK           0xA0    // OTP memory check register (one-time-programmable memory)
#define REG_STACK_VOLTAGE_L     0x34    // Stack (VC16 pin) voltage LSB (µV units)
#define REG_STACK_VOLTAGE_H     0x35    // Stack (VC16 pin) voltage MSB (µV units)
#define REG_PACK_VOLTAGE_L      0x36    // PACK pin voltage LSB (µV units)  
#define REG_PACK_VOLTAGE_H      0x37    // PACK pin voltage MSB (µV units)
#define REG_LD_VOLTAGE_L        0x38    // LD pin voltage LSB (µV units)
#define REG_LD_VOLTAGE_H        0x39    // LD pin voltage MSB (µV units)

// Individual cell voltage registers (mV units)
#define REG_CELL1_VOLTAGE_L     0x14    // Cell 1 voltage LSB
#define REG_CELL1_VOLTAGE_H     0x15    // Cell 1 voltage MSB

// Note: BQ76952 doesn't have a simple device ID register at 0x00
// Device identification requires subcommands, not direct commands

#ifndef HAL_BATTMON_BQ76952_MAX_VOLTAGE
#define HAL_BATTMON_BQ76952_MAX_VOLTAGE 50.0f
#endif

const AP_Param::GroupInfo AP_BattMonitor_TIBQ76952::var_info[] = {

    // @Param: MAX_VOLT
    // @DisplayName: Battery monitor max voltage
    // @Description: This controls the maximum voltage the BQ76952 sensor will work with.
    // @Range: 10 60
    // @Units: V
    // @User: Advanced
    AP_GROUPINFO("MAX_VOLT", 27, AP_BattMonitor_TIBQ76952, max_voltage, HAL_BATTMON_BQ76952_MAX_VOLTAGE),

    // CHECK/UPDATE INDEX TABLE IN AP_BattMonitor_Backend.cpp WHEN CHANGING OR ADDING PARAMETERS

    AP_GROUPEND
};

AP_BattMonitor_TIBQ76952::AP_BattMonitor_TIBQ76952(AP_BattMonitor &mon,
                                             AP_BattMonitor::BattMonitor_State &mon_state,
                                             AP_BattMonitor_Params &params)
        : AP_BattMonitor_Backend(mon, mon_state, params)
{
    AP_Param::setup_object_defaults(this, var_info);
    _state.var_info = var_info;
}

void AP_BattMonitor_TIBQ76952::init(void)
{
    dev = hal.spi->get_device_ptr(AP_BATTERY_TIBQ76952_SPI_DEVICE);
    if (!dev) {
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "BQ76952: device fail");
        return;
    }

    // Register periodic callback at 10hz for reading voltage
    dev->register_periodic_callback(1000000, FUNCTOR_BIND_MEMBER(&AP_BattMonitor_TIBQ76952::timer, void));
}

/// read the battery_voltage, should be called at 10hz
void AP_BattMonitor_TIBQ76952::read(void)
{
    WITH_SEMAPHORE(accumulate.sem);
    /*_state.healthy = accumulate.count > 0;
    if (!_state.healthy) {
        return;
    }

    _state.voltage = accumulate.volt_sum / accumulate.count;
    // For now, we're only implementing voltage reading
    _state.current_amps = 0.0f;  // BQ76952 can measure current but we'll implement that later
    accumulate.volt_sum = 0;
    accumulate.count = 0;

    const uint32_t tnow = AP_HAL::micros();
    _state.last_time_micros = tnow;
    */
   _state.healthy = true;
   _state.voltage = accumulate.voltage;
   _state.current_amps = accumulate.current;
   _state.temperature = accumulate.temp;
   _state.last_time_micros = AP_HAL::micros();

   // debug toggle 1st LED on each read (10hz)
   hal.gpio->toggle(HAL_GPIO_PIN_BMS_LED1);
}

/*
 read word from register - BQ76952 uses direct commands with little endian data
 returns true if read was successful, false if failed
*/
bool AP_BattMonitor_TIBQ76952::read_word(uint8_t reg, int16_t& data) const
{
    // BQ76952 uses 7-bit direct commands for voltage readings
    // Data is stored in little endian format as per datasheet
    if (!dev->read_registers(reg, (uint8_t *)&data, sizeof(data))) {
        return false;
    }

    // BQ76952 uses little endian byte order (confirmed in datasheet)
    data = int16_t(le16toh(uint16_t(data)));
    return true;
}

/*
  write word to a register - BQ76952 specific
  returns true if write was successful, false if failed
*/
bool AP_BattMonitor_TIBQ76952::write_word(uint8_t reg, uint16_t data) const
{
    // BQ76952 write protocol - this will need to be adjusted based on datasheet
    /*const uint8_t b[3] { reg, uint8_t(data & 0xff), uint8_t(data >> 8) };
    return dev->transfer(b, sizeof(b), nullptr, 0);
    */
    return false;
}

void AP_BattMonitor_TIBQ76952::timer(void)
{
    // debug toggle 8th LED on each timer update
    hal.gpio->toggle(HAL_GPIO_PIN_BMS_LED8);

    // Read stack voltage from BQ76952 using direct commands 0x34/0x35
    // According to datasheet Table 4-1: Stack (VC16 pin) voltage in µV units
    /*int16_t voltage_lsb;
    if (!read_word(REG_STACK_VOLTAGE_L, voltage_lsb)) {
        voltage_lsb = 99;
    }*/
    //int16_t voltage_msb;
    /*if (!read_word(REG_STACK_VOLTAGE_L, voltage_lsb) ||
        !read_word(REG_STACK_VOLTAGE_H, voltage_msb)) {
        return;
    }*/
    /*if (!read_word(REG_PACK_VOLTAGE_L, voltage_lsb) ||
        !read_word(REG_PACK_VOLTAGE_H, voltage_msb)) {
        return;
    }*/
    int16_t otp_check;
    if (!read_word(REG_OTP_CHECK, otp_check)) {
        otp_check = 99;
    }

    // calculate voltage
    //float voltage_v = UINT16_VALUE(LOWBYTE(voltage_msb),LOWBYTE(voltage_lsb)) * 0.001;

    WITH_SEMAPHORE(accumulate.sem);
    accumulate.voltage = otp_check;
    accumulate.temp = 4;
    accumulate.current = 3;
    accumulate.count = 1;

    // debug toggle 7th LED
    hal.gpio->toggle(HAL_GPIO_PIN_BMS_LED7);
}

#endif // AP_BATTERY_TIBQ76952_ENABLED
