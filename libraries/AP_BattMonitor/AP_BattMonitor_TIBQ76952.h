#pragma once

#include <AP_Common/AP_Common.h>
#include <AP_HAL/SPIDevice.h>
#include "AP_BattMonitor_Backend.h"
#include <utility>

#if AP_BATTERY_TIBQ76952_ENABLED

class AP_BattMonitor_TIBQ76952 : public AP_BattMonitor_Backend
{
public:
    /// Constructor
    AP_BattMonitor_TIBQ76952(AP_BattMonitor &mon,
                          AP_BattMonitor::BattMonitor_State &mon_state,
                          AP_BattMonitor_Params &params);

    bool has_cell_voltages() const override { return false; }  // TODO: BQ76952 can read individual cells
    bool has_temperature() const override { return true; }   // TODO: BQ76952 has temperature sensors
    bool has_current() const override { return true; }       // For now, only voltage implemented
    bool get_cycle_count(uint16_t &cycles) const override { return false; }

    void init(void) override;
    void read() override;

    static const struct AP_Param::GroupInfo var_info[];

protected:
    AP_HAL::Device *dev;

    bool read_word(uint8_t reg, int16_t& data) const;
    bool write_word(uint8_t reg, uint16_t data) const;
    void timer(void);

    bool configured;
    bool callback_registered;
    uint32_t failed_reads;
    uint32_t last_configure_ms;

    struct {
        uint16_t count;
        float voltage;
        float current;
        float temp;
        HAL_Semaphore sem;
    } accumulate;

    AP_Float max_voltage;  // Maximum pack voltage parameter
};

#endif // AP_BATTERY_TIBQ76952_ENABLED
