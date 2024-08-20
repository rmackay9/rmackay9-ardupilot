-- This script is an example of manipulating the message stream rates
--
-- It will periodically run and adjust all the messages to their desired loop rates
-- It can be modified to only run once, however some GCS's will manipulate the rates
-- during the initial connection, so by resetting them periodically we ensure we get
-- the expected telemetry rate

-- local GLOBAL_POSITION_INT = uint32_t(33)
-- local ATTITUDE = uint32_t(30)
-- local AHRS = uint32_t(163)

local MAV_SEVERITY = {EMERGENCY=0, ALERT=1, CRITICAL=2, ERROR=3, WARNING=4, NOTICE=5, INFO=6, DEBUG=7}

-- intervals is a table which contains a table for each entry we want to adjust
-- each entry is arranged as {the serial link to adjust, the mavlink message ID, and the broadcast interval in Hz}
local intervals = {{0, uint32_t(33), 10.0}, -- First serial, GLOBAL_POSITION_INT, 10Hz
                    {0, uint32_t(30), 1.0}, -- ATTITUDE, 1Hz
                    {0, uint32_t(163), 0}, -- AHRS, 0Hz
                    {0, uint32_t(178), 0}, -- AHRS2, 0Hz
                    {0, uint32_t(193), 1.0}, -- EKF_STATUS_REPORT, 0Hz
                    {0, uint32_t(49), 0}, -- GPS_GLOBAL_ORIGIN, 0Hz
                    {0, uint32_t(24), 1.0}, -- GPS_RAW_INT, 0Hz
                    {0, uint32_t(242), 0}, -- HOME_POSITION, 0Hz
                    {0, uint32_t(32), 0}, -- LOCAL_POSITION_NED, 0Hz
                    {0, uint32_t(152), 0}, -- MEMINFO, 0Hz
                    {0, uint32_t(42), 0}, -- MISSION_CURRENT, 0Hz
                    {0, uint32_t(62), 0}, -- NAV_CONTROLLER_OUTPUT, 0Hz
                    {0, uint32_t(22), 0}, -- PARAM_VALUE, 0Hz
                    {0, uint32_t(125), 0}, -- POWER_STATUS, 0Hz
                    {0, uint32_t(27), 0}, -- RAW_INT, 0Hz
                    {0, uint32_t(65), 0}, -- RC_CANNELS, 0Hz
                    {0, uint32_t(29), 0}, -- SCALED_PRESSURE, 0Hz
                    {0, uint32_t(36), 0}, -- SERVO_OUTPUT_RAW, 0Hz
                    {0, uint32_t(1), 0}, -- SYS_STATUS, 0Hz
                    {0, uint32_t(2), 0}, -- SYSTEM_TIME, 0Hz
                    {0, uint32_t(74), 0}, -- VFR_HUD, 0Hz
                    {0, uint32_t(241), 0}, -- VIBRATION, 0Hz
                    {0, uint32_t(168), 0} -- WIND, 0Hz
}

local loop_time = 5000 -- number of ms between runs

gcs:send_text(MAV_SEVERITY.INFO, "Loaded message_interval.lua")

function update() -- this is the loop which periodically runs
  for i = 1, #intervals do -- we want to iterate over every specified interval
    local channel, message, interval_hz = table.unpack(intervals[i]) -- this extracts the channel, MAVLink ID, and interval
    local interval_us = -1
    if interval_hz > 0 then
      interval_us = math.floor(1000000 / interval_hz) -- convert the interval to microseconds
    end
    gcs:set_message_interval(channel, message, interval_us) -- actually sets the interval as appropriate
  end
  return update, loop_time -- reschedules the loop
end

return update() -- run immediately before starting to reschedule
