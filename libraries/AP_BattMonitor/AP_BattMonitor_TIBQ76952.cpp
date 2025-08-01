#include "AP_BattMonitor_config.h"

#if AP_BATTERY_TIBQ76952_ENABLED

#include <GCS_MAVLink/GCS.h>
#include <AP_HAL/utility/sparse-endian.h>
#include <AP_HAL/AP_HAL.h>

// Include board-specific definitions for GPIO
#if CONFIG_HAL_BOARD == HAL_BOARD_CHIBIOS
#include <AP_HAL/board/chibios.h>
#include <ch.h>
#include <hal.h>
#endif

#include "AP_BattMonitor_TIBQ76952.h"

#include "stdio.h"

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

// BQ76952 Communication Settings
#define MAX_BUFFER_SIZE 10
#define MAX_RETRIES 10
#define CRC_MODE 1  // BQ76952 requires CRC for reliable SPI communication
#define DISABLE_PROTECTION_A
#define DISABLE_PROTECTION_B 
#define DISABLE_TS1
#define DISABLE_TS3

#define CELL_COUNT 6

// Expected device IDs
#define DEVICE_ID_TIBQ7695 0x7695

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
    dev = hal.spi->get_device(AP_BATTERY_TIBQ76952_SPI_DEVICE);
    if (!dev) {
        return;
    }

    // change to high speed
    dev->set_speed(AP_HAL::Device::SPEED_HIGH);

    // configure device
    if (!configure()) {
        return;
    }

    // Register periodic callback at 10hz for reading voltage
    dev->register_periodic_callback(100000, FUNCTOR_BIND_MEMBER(&AP_BattMonitor_TIBQ76952::timer, void));
}


// periodic timer callback
void AP_BattMonitor_TIBQ76952::timer(void)
{
    // permanent_fail_status_update();
    alarm_status_update();
    //battery_status_update();
    return;

    // debug toggle 8th LED on each timer update
    hal.gpio->toggle(HAL_GPIO_PIN_BMS_LED8);

    // // debug toggle 7th LED
    // hal.gpio->toggle(HAL_GPIO_PIN_BMS_LED7);
}

// configure device, called repeatedly from timer()
bool AP_BattMonitor_TIBQ76952::configure() const
{
    // check device id, exit on failure
    uint32_t device_number = read_device_number();
    if (device_number != DEVICE_ID_TIBQ7695) {
        printf("BQ76952: Unknown device detected - ID: 0x%08lX\n", (unsigned long)device_number);
        return false;
    }

    // check device's firmware and hardware versions
    uint32_t fw_version = read_fw_version();
    uint32_t hv_version = read_hv_version();

    // debug
    printf("BQ76952 detected, fw: 0x%08lX, hw: 0x%08lX\n", (unsigned long)fw_version, (unsigned long)hv_version);

    // reload and clear failure status bits
    if (read_permanent_fail_status_A() > 0 || read_permanent_fail_status_B() > 0 || read_permanent_fail_status_C() > 0 || read_permanent_fail_status_D() > 0) {
        printf("BQ76952: Permanent fail status detected\n");
        sub_command(TIBQ769x2_PF_RESET);
        hal.scheduler->delay_microseconds(50000);
        permanent_fail_status_update();
    }

    // wake up device
    sub_command(TIBQ769x2_EXIT_DEEPSLEEP);
    hal.scheduler->delay_microseconds(10000);
    sub_command(TIBQ769x2_SLEEP_DISABLE);
    hal.scheduler->delay_microseconds(10000);

    // Clear any remaining permanent failure alerts
    direct_command(TIBQ769x2_PFAlertA, 0xFF, WRITE);
    direct_command(TIBQ769x2_PFAlertB, 0xFF, WRITE);
    direct_command(TIBQ769x2_PFAlertC, 0xFF, WRITE);
    direct_command(TIBQ769x2_PFAlertD, 0xFF, WRITE);

    // enable FETs so we can switch between charging and discharging
    sub_command(TIBQ769x2_FET_ENABLE);

    // Enter CONFIGUPDATE mode (Subcommand 0x0090) - Required to program device RAM settings
    sub_command(TIBQ769x2_SET_CFGUPDATE);
    
    // 'Power Config' - 0x9234 = 0x2D80
    // Setting the DSLP_LDO bit allows the LDOs to remain active when the device goes into Deep Sleep mode
    // Set wake speed bits to 00 for best performance
    set_register(TIBQ769x2_PowerConfig, 0x2D80, 2);
    
    // 'REG0 Config' - set REG0_EN bit to enable pre-regulator
    set_register(TIBQ769x2_REG0Config, 0x01, 1);
    
    // 'REG12 Config' - Enable REG1 with 3.3V output (0x0D for 3.3V, 0x0F for 5V)
    set_register(TIBQ769x2_REG12Config, 0x0D, 1);
    
    // Set DFETOFF pin to control BOTH CHG and DSG FET - 0x92FB = 0x42 (set to 0x00 to disable)
    set_register(TIBQ769x2_DFETOFFPinConfig, 0x42, 1);
    
    // Set up ALERT Pin - 0x92FC = 0x2A
    // This configures the ALERT pin to drive high (REG1 voltage) when enabled.
    // The ALERT pin can be used as an interrupt to the MCU when a protection has triggered or new measurements are available
    set_register(TIBQ769x2_ALERTPinConfig, 0x2A, 1);
    
#if defined(DISABLE_TS1)
    set_register(TIBQ769x2_TS1Config, 0x00, 1);
#else
    // Set TS1 to measure Cell Temperature - 0x92FD = 0x07
    set_register(TIBQ769x2_TS1Config, 0x07, 1);
#endif

#if defined(DISABLE_TS3)
    set_register(TIBQ769x2_TS3Config, 0x00, 1);
#else
    // Set TS3 to measure FET Temperature - 0x92FF = 0x0F
    set_register(TIBQ769x2_TS3Config, 0x0F, 1);
#endif
    
    // Set HDQ to measure Cell Temperature - 0x9300 = 0x00 (No thermistor installed on EVM HDQ pin)
    set_register(TIBQ769x2_HDQPinConfig, 0x00, 1);
    
    // 'VCell Mode' - Enable 16 cells - 0x9304
    // Writing 0x0000 sets the default of 16 cells, but we'll calculate based on actual cell count
    uint16_t vcell_mode = (1 << CELL_COUNT) - 1;
    set_register(TIBQ769x2_VCellMode, vcell_mode, 2);
    
#if defined(DISABLE_PROTECTION_A)
    set_register(TIBQ769x2_EnabledProtectionsA, 0x00, 1);
#else
    // Enable protections in 'Enabled Protections A' 0x9261 = 0xBC
    // Enables SCD (short-circuit), OCD1 (over-current in discharge), OCC (over-current in charge),
    // COV (over-voltage), CUV (under-voltage)
    set_register(TIBQ769x2_EnabledProtectionsA, 0xBC, 1);
#endif
    
#if defined(DISABLE_PROTECTION_B)
    set_register(TIBQ769x2_EnabledProtectionsB, 0x00, 1);
#else
    // Enable all protections in 'Enabled Protections B' 0x9262 = 0xF7
    // Enables OTF (over-temperature FET), OTINT (internal over-temperature), OTD (over-temperature in discharge),
    // OTC (over-temperature in charge), UTINT (internal under-temperature), UTD (under-temperature in discharge), UTC (under-temperature in charge)
    set_register(TIBQ769x2_EnabledProtectionsB, 0xF7, 1);
#endif
    
    // 'Default Alarm Mask' - 0x926D Enables the FullScan and ADScan bits, default value = 0xF800
    set_register(TIBQ769x2_DefaultAlarmMask, 0xF882, 2);
    
    // Set up Cell Balancing Configuration - 0x9335 = 0x03 - Automated balancing while in Relax or Charge modes
    set_register(TIBQ769x2_BalancingConfiguration, 0x03, 1);
    
    // Set up CUV (under-voltage) Threshold - 0x9275 = 0x31 (2479 mV)
    // CUV Threshold is this value multiplied by 50.6mV
    set_register(TIBQ769x2_CUVThreshold, 0x31, 1);
    
    // Set up COV (over-voltage) Threshold - 0x9278 = 0x55 (4301 mV)
    // COV Threshold is this value multiplied by 50.6mV
    set_register(TIBQ769x2_COVThreshold, 0x55, 1);
    
    // Set up OCC (over-current in charge) Threshold - 0x9280 = 0x05 (10 mV = 10A across 1mOhm sense resistor) Units in 2mV
    set_register(TIBQ769x2_OCCThreshold, 0x05, 1);
    
    // Set up OCD1 Threshold - 0x9282 = 0x0A (20 mV = 20A across 1mOhm sense resistor) units of 2mV
    set_register(TIBQ769x2_OCD1Threshold, 0x0A, 1);
    
    // Set up SCD Threshold - 0x9286 = 0x05 (100 mV = 100A across 1mOhm sense resistor) 0x05=100mV
    set_register(TIBQ769x2_SCDThreshold, 0x05, 1);
    
    // Set up SCD Delay - 0x9287 = 0x03 (30 us) Enabled with a delay of (value - 1) * 15 μs; min value of 1
    set_register(TIBQ769x2_SCDDelay, 0x03, 1);
    
    // Set up SCDL Latch Limit to 1 to set SCD recovery only with load removal 0x9295 = 0x01
    // If this is not set, then SCD will recover based on time (SCD Recovery Time parameter).
    set_register(TIBQ769x2_SCDLLatchLimit, 0x01, 1);
    
    // Exit CONFIGUPDATE mode - Subcommand 0x0092
    sub_command(TIBQ769x2_EXIT_CFGUPDATE);

    // report success
    return true;
}

/// read the battery_voltage, should be called at 10hz
void AP_BattMonitor_TIBQ76952::read(void)
{
    WITH_SEMAPHORE(accumulate.sem);

    // exit immediately if no sensor updates
    if (accumulate.count == 0) {
        return;
    }

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
    _state.voltage = accumulate.voltage / accumulate.count;
    _state.state_of_health_pct = accumulate.health_pct / accumulate.count;
    _state.has_state_of_health_pct = true;

    // debug toggle 1st LED on each read (10hz)
    // hal.gpio->toggle(HAL_GPIO_PIN_BMS_LED1);
}

uint8_t AP_BattMonitor_TIBQ76952::calc_crc8(const uint8_t *data, uint8_t len) const
{
    uint8_t crc = 0;
    for (uint8_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (uint8_t j = 0; j < 8; j++) {
            if (crc & 0x80) {
                crc = (crc << 1) ^ 0x07;
            } else {
                crc <<= 1;
            }
        }
    }
    return crc;
}


// read from register
bool AP_BattMonitor_TIBQ76952::read_register(uint8_t reg_addr, uint8_t *reg_data, uint8_t count) const
{
    WITH_SEMAPHORE(dev->get_semaphore());
    AP_HAL::SPIDevice* spi_dev = (AP_HAL::SPIDevice*)dev.get();
    
    uint8_t addr = reg_addr;
    uint8_t retries = MAX_RETRIES;
    uint8_t tx_buffer[MAX_BUFFER_SIZE] = {0};
    uint8_t rx_buffer[MAX_BUFFER_SIZE] = {0};
    bool match = false;
    addr = reg_addr;
    uint32_t len = 2;

    for (uint8_t i = 0; i < count; i++) {
        tx_buffer[0] = addr;
        tx_buffer[1] = 0xFF;

        if (CRC_MODE) {
            tx_buffer[2] = calc_crc8(tx_buffer, 2);
            len = 3; // addr + data + crc
        }

        while (!match && retries > 0) {
            hal.scheduler->delay_microseconds(500);

            if (!spi_dev->transfer_fullduplex(tx_buffer, rx_buffer, len)) {
                return false;
            }

            if (rx_buffer[0] == addr) {
                match = true;
                reg_data[i] = rx_buffer[1];
            }
            retries--;
        }
        match = false;
        addr++;
        hal.scheduler->delay_microseconds(500);
    }
    
    return true;
}

// write to register
bool AP_BattMonitor_TIBQ76952::write_register(uint8_t reg_addr, const uint8_t *reg_data, uint8_t count) const
{
    WITH_SEMAPHORE(dev->get_semaphore());
    AP_HAL::SPIDevice* spi_dev = (AP_HAL::SPIDevice*)dev.get();

    // For write operations, set bit 7 of the address
    uint8_t addr = 0x80 | reg_addr;
    uint8_t tx_buffer[MAX_BUFFER_SIZE] = {0};
    uint8_t rx_buffer[MAX_BUFFER_SIZE] = {0};

    for (uint8_t i = 0; i < count; i++) {
        // Prepare transmit buffer
        tx_buffer[0] = addr;
        tx_buffer[1] = reg_data[i];

        uint32_t len;
        if (CRC_MODE) {
            tx_buffer[2] = calc_crc8(tx_buffer, 2);
            len = 3; // addr + data + crc
        } else {
            len = 2; // addr + data
        }

        // Attempt transfer with retries
        bool success = false;
        for (uint8_t retry = 0; retry < MAX_RETRIES && !success; retry++) {
            if (retry > 0) {
                hal.scheduler->delay_microseconds(500);
            }

            if (spi_dev->transfer_fullduplex(tx_buffer, rx_buffer, len)) {
                // Verify response
                if (rx_buffer[0] == addr && rx_buffer[1] == reg_data[i]) {
                    success = true;
                }
            }
        }

        if (!success) {
            return false;
        }

        addr++; // Increment address for next byte
    }

    return true;
}

uint8_t AP_BattMonitor_TIBQ76952::checksum(const uint8_t *data, uint8_t len) const
{
    // Simple checksum calculation - sum of all bytes
    uint8_t sum = 0;
    for (uint8_t i = 0; i < len; i++) {
        sum += data[i];
    }
    return sum;
}

bool AP_BattMonitor_TIBQ76952::sub_command(uint16_t command) const
{
    //printf("BQ76952: sub_command called - cmd=0x%04X\n", command);
    uint8_t tx_buffer[2] = {0,0};
    tx_buffer[0] = command & 0xFF;
    tx_buffer[1] = (command >> 8) & 0xFF;

    return write_register(0x3E, tx_buffer, 2);
}

bool AP_BattMonitor_TIBQ76952::sub_command(uint16_t command, uint16_t data, SubcommandType type, uint8_t *rx_data, uint8_t rx_len) const
{
    //printf("BQ76952: sub_command called - cmd=0x%04X, type=%d\n", command, (int)type);
    
    uint8_t tx_reg[4] = {0};
    uint8_t tx_buffer[2] = {0};
    
    // TX_Reg in little endian format
    tx_reg[0] = command & 0xFF;
    tx_reg[1] = (command >> 8) & 0xFF;
    
    // printf("BQ76952: sub_command tx_reg = [0x%02X, 0x%02X]\n", tx_reg[0], tx_reg[1]);
    
    switch (type) {
        case READ:
            // Read subcommand
            // printf("BQ76952: Writing subcommand to register 0x3E\n");
            if (!write_register(0x3E, tx_reg, 2)) {
                // printf("BQ76952: sub_command failed - write_register to 0x3E failed\n");
                return false;
            }
            hal.scheduler->delay_microseconds(2000);
            
            // printf("BQ76952: Reading data from register 0x40\n");
            if (rx_data && rx_len > 0) {
                // Read into provided buffer
                if (!read_register(0x40, rx_data, rx_len)) {
                    // printf("BQ76952: sub_command failed - read_register from 0x40 failed\n");
                    return false;
                }
                // printf("BQ76952: sub_command read success - first 4 bytes: %02X %02X %02X %02X\n", 
                //        rx_data[0], rx_data[1], rx_data[2], rx_data[3]);
                return true;
            } else {
            }
            break;
            
        case WRITE:
            // Write subcommand with 1 byte data (FET_Control, REG12_Control)
            tx_reg[2] = data & 0xFF;
            if (!write_register(0x3E, tx_reg, 3)) {
                return false;
            }
            hal.scheduler->delay_microseconds(1000);
            
            tx_buffer[0] = checksum(tx_reg, 3);
            tx_buffer[1] = 0x05; // Combined length of registers address and data
            if (!write_register(0x60, tx_buffer, 2)) {
                return false;
            }
            hal.scheduler->delay_microseconds(1000);
            break;
            
        case WRITE2:
            // Write subcommand with 2 bytes data (CB_Active_Cells, CB_SET_LVL)
            tx_reg[2] = data & 0xFF;
            tx_reg[3] = (data >> 8) & 0xFF;
            if (!write_register(0x3E, tx_reg, 4)) {
                return false;
            }
            hal.scheduler->delay_microseconds(1000);
            
            tx_buffer[0] = checksum(tx_reg, 4);
            tx_buffer[1] = 0x06; // Combined length of registers address and data
            if (!write_register(0x60, tx_buffer, 2)) {
                return false;
            }
            hal.scheduler->delay_microseconds(1000);
            break;
            
        default:
            return false;
    }
    
    return true;
}

bool AP_BattMonitor_TIBQ76952::direct_command(uint16_t command, uint16_t data, SubcommandType type, uint8_t *rx_data, uint8_t rx_len) const
{
    //printf("BQ76952: direct_command called - cmd=0x%04X, data=0x%04X, type=%d\n", command, data, (int)type);

    // sanity check read buffer
    if (type == READ && rx_data == nullptr) {
        return false;
    }

    uint8_t tx_buffer[2] = {0, 0};
    tx_buffer[0] = data & 0xFF;
    tx_buffer[1] = (data >> 8) & 0xFF;

    if (type == READ) {
        read_register(command, rx_data, rx_len);
    } else if (type == WRITE) {
        write_register(command, tx_buffer, rx_len);
    }

    return true;
}

void AP_BattMonitor_TIBQ76952::set_register(uint16_t reg_addr, uint32_t reg_data, uint8_t len) const
{
    uint8_t tx_buffer[2] = {0, 0};
    uint8_t tx_reg_data[6] = {0, 0, 0, 0, 0, 0};

    tx_reg_data[0] = reg_addr & 0xFF;
    tx_reg_data[1] = (reg_addr >> 8) & 0xFF;
    tx_reg_data[2] = reg_data & 0xFF;

    switch (len) {
        case 1:
        {
            write_register(0x3E, tx_reg_data, 3);
            hal.scheduler->delay_microseconds(2000);
            tx_buffer[0] = checksum(tx_reg_data, 3);
            tx_buffer[1] = 0x05;
            write_register(0x60, tx_buffer, 2);
            hal.scheduler->delay_microseconds(2000);
            break;
        }
        case 2:
        {
            tx_reg_data[3] = (reg_data >> 8) & 0xFF;
            write_register(0x3E, tx_reg_data, 4);
            hal.scheduler->delay_microseconds(2000);
            tx_buffer[0] = checksum(tx_reg_data, 4);
            tx_buffer[1] = 0x06;
            write_register(0x60, tx_buffer, 2);
            hal.scheduler->delay_microseconds(2000);
            break;
        }
        case 4:
        {
            tx_reg_data[3] = (reg_data >> 8) & 0xFF;
            tx_reg_data[4] = (reg_data >> 16) & 0xFF;
            tx_reg_data[5] = (reg_data >> 24) & 0xFF;
            write_register(0x3E, tx_reg_data, 6);
            hal.scheduler->delay_microseconds(2000);
            tx_buffer[0] = checksum(tx_reg_data, 6);
            tx_buffer[1] = 0x08;
            write_register(0x60, tx_buffer, 2);
            hal.scheduler->delay_microseconds(2000);
            break;
        }
    }
}

uint32_t AP_BattMonitor_TIBQ76952::read_device_number(void) const
{
    printf("BQ76952: Detecting device...\n");
    uint8_t rx_data[32];
    if (sub_command(TIBQ769x2_DEVICE_NUMBER, 0x00, READ, rx_data, 32)) {
        return (rx_data[3] << 24) | (rx_data[2] << 16) | (rx_data[1] << 8) | rx_data[0];
    } 
    return 0;
}

uint32_t AP_BattMonitor_TIBQ76952::read_fw_version(void) const
{
    uint8_t rx_data[32];
    if (sub_command(TIBQ769x2_FW_VERSION, 0x00, READ, rx_data, 32)) {
        return (rx_data[3] << 24) | (rx_data[2] << 16) | (rx_data[1] << 8) | rx_data[0];
    }
    return 0;
}

uint32_t AP_BattMonitor_TIBQ76952::read_hv_version(void) const
{
    uint8_t rx_data[32];
    if (sub_command(TIBQ769x2_HW_VERSION, 0x00, READ, rx_data, 32)) {
        // Hardware version is typically in the first 2 bytes (little endian)
        return (rx_data[3] << 24) | (rx_data[2] << 16) | (rx_data[1] << 8) | rx_data[0];
    }
    return 0;
}

uint16_t AP_BattMonitor_TIBQ76952::read_voltage(uint8_t command) const
{
    if (command != TIBQ769x2_StackVoltage &&
        command != TIBQ769x2_PACKPinVoltage &&
        command != TIBQ769x2_LDPinVoltage &&
        (command < TIBQ769x2_Cell1Voltage || command > TIBQ769x2_Cell16Voltage)) {
            printf("BQ76952: Invalid voltage command: 0x%02X\n", command);
        return 0;
    }
    uint8_t rx_data[2];
    if (direct_command(command, 0x00, READ, rx_data, 2)) {
        if (command >= TIBQ769x2_Cell1Voltage && command <= TIBQ769x2_Cell16Voltage) {
            return rx_data[1] << 8 | rx_data[0];
        } else {
            return 10 * (rx_data[1] << 8 | rx_data[0]);
        }
    }
    printf("BQ76952: Failed to read voltage: 0x%02X\n", command);
    return 0;
}

uint16_t AP_BattMonitor_TIBQ76952::read_alarm_status(void) const
{
    uint8_t rx_data[2];
    if (direct_command(TIBQ769x2_AlarmStatus, 0x00, READ, rx_data, 2)) {
        return rx_data[1] << 8 | rx_data[0];
    }
    return 0;
}

void AP_BattMonitor_TIBQ76952::alarm_status_update(void)
{
    uint16_t alarm_status = read_alarm_status();
    printf("BQ76952: Alarm status: 0x%04X\n", alarm_status);
    /*if (alarm_status & 0x8000) {
        printf("BQ76952: Alarm status protection triggered\n");
    }
    if (alarm_status & 0x4000) {
        printf("BQ76952: Alarm status overvoltage triggered\n");
    }
    if (alarm_status & 0x0008) {
        printf("BQ76952: Alarm status fuse triggered\n");
    }
    if (alarm_status & 0x0004) {
        printf("BQ76952: Alarm status cell balancing is active\n");
    }*/

    if (alarm_status & 0x0002) {
        //uint16_t load_voltage = read_voltage(TIBQ769x2_LDPinVoltage);
        //printf("ld_voltage: %f mV\n", ld_voltage);

        //uint16_t stack_voltage = read_voltage(TIBQ769x2_StackVoltage);
        //printf("stack_voltage: %f mV\n", stack_voltage);
        /*uint16_t cell_voltages[16];
        for (int i = 0; i < 3; i++) {
            cell_voltages[i] = read_voltage(TIBQ769x2_Cell1Voltage + i*2);
            printf("cell_voltage[%d]: %d mV\n", i, cell_voltages[i]);
        }*/

        // read voltage
        WITH_SEMAPHORE(accumulate.sem);
        uint16_t pack_voltage_mv = read_voltage(TIBQ769x2_PACKPinVoltage);
        uint16_t stack_voltage_mv = read_voltage(TIBQ769x2_StackVoltage);
        uint16_t ld_voltage_mv = read_voltage(TIBQ769x2_LDPinVoltage);
        printf("pack_voltage: %f mV\n", pack_voltage_mv);
        printf("stack_voltage: %f mV\n", stack_voltage_mv);
        printf("ld_voltage: %f mV\n", ld_voltage_mv);
        
        uint16_t cell_voltage_mv[CELL_COUNT];
        uint32_t cell_voltage_sum_mv = 0;
        for (uint8_t i = 0; i < CELL_COUNT; i++) {
            cell_voltage_mv[i] = read_voltage(TIBQ769x2_Cell1Voltage + i*2);
            printf("cell_voltage[%d]: %d mV\n", i, cell_voltage_mv[i]);
            cell_voltage_sum_mv += cell_voltage_mv[i];
            _state.cell_voltages.cells[i] = cell_voltage_mv[i];
        }
        accumulate.voltage += cell_voltage_sum_mv * 0.001;
        accumulate.count++;
    }

    /*if (alarm_status & 0x0001) {
        printf("BQ76952: Alarm status wakened from SLEEP mode\n");
    }*/
}

uint16_t AP_BattMonitor_TIBQ76952::read_permanent_fail_status_A(void) const
{
    uint8_t rx_data[2];
    if (direct_command(TIBQ769x2_PFStatusA, 0x00, READ, rx_data, 2)) {
        return rx_data[1] << 8 | rx_data[0];
    }
    return 0;
}

uint16_t AP_BattMonitor_TIBQ76952::read_permanent_fail_status_B(void) const
{
    uint8_t rx_data[2];
    if (direct_command(TIBQ769x2_PFStatusB, 0x00, READ, rx_data, 2)) {
        return rx_data[1] << 8 | rx_data[0];
    }
    return 0;
}

uint16_t AP_BattMonitor_TIBQ76952::read_permanent_fail_status_C(void) const
{
    uint8_t rx_data[2];
    if (direct_command(TIBQ769x2_PFStatusC, 0x00, READ, rx_data, 2)) {
        return rx_data[1] << 8 | rx_data[0];
    }
    return 0;
}

uint16_t AP_BattMonitor_TIBQ76952::read_permanent_fail_status_D(void) const
{
    uint8_t rx_data[2];
    if (direct_command(TIBQ769x2_PFStatusD, 0x00, READ, rx_data, 2)) {
        return rx_data[1] << 8 | rx_data[0];
    }
    return 0;
}

void AP_BattMonitor_TIBQ76952::permanent_fail_status_update(void) const
{
    /*uint16_t status_A = read_permanent_fail_status_A();
    uint16_t status_B = read_permanent_fail_status_B();
    uint16_t status_C = read_permanent_fail_status_C();
    uint16_t status_D = read_permanent_fail_status_D();

    printf("BQ76952: Permanent Fail Status A: 0x%04X\n", status_A);
    printf("BQ76952: Permanent Fail Status B: 0x%04X\n", status_B);
    printf("BQ76952: Permanent Fail Status C: 0x%04X\n", status_C);
    printf("BQ76952: Permanent Fail Status D: 0x%04X\n", status_D);
    */
}

uint16_t AP_BattMonitor_TIBQ76952::read_battery_status(void) const
{
    uint8_t rx_data[2];
    if (direct_command(TIBQ769x2_BatteryStatus, 0x00, READ, rx_data, 2)) {
        return rx_data[1] << 8 | rx_data[0];
    }
    return 0;
}

void AP_BattMonitor_TIBQ76952::battery_status_update(void) const
{
    /*uint16_t battery_status = read_battery_status();
    printf("BQ76952: Battery status: 0x%04X\n", battery_status);
    if (battery_status & 0x8000) {
        printf("BQ76952: Battery status: in sleep mode\n");
    }
    if (battery_status & 0x2000) {
        printf("BQ76952: Battery status: in shutdown mode\n");
    }
    if (battery_status & 0x1000) {
        printf("BQ76952: Battery status: permanent failed fault triggered\n");
    }
    if (battery_status & 0x0800) {
        printf("BQ76952: Battery status: safety fault triggered\n");
    }
    if (battery_status & 0x0400) {
        printf("BQ76952: Battery status: fuse pin active\n");
    }
    if (battery_status & 0x0200) {
        printf("BQ76952: Battery status: fuse pin inactive\n");
    }*/
}

#endif // AP_BATTERY_TIBQ76952_ENABLED
