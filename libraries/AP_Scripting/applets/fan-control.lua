-- fan-control.lua: controls a fan by turning on/off a relay
--
-- How To Use
--   1. Setup a relay connected to the fan's voltage input
--   2. Set RC6_OPTION to 300 (Scripting1)
--   3. Setup the relay feature including:
--       a. SERVO14_FUNCTION = -1 (GPIO)
--       b. RELAY1_FUNCTION = 1 (Relay)
--       c. RELAY1_PIN = 55 (AuxOut6)
--   4. Test the RC6 can control the speed of the fan
--
-- How It Works
--   1. RC6's input is consumed and converted in a fan speed expressed as a percentage (0 to 100)
--   2. Relay is turned on/off at 1000hz so that the average output for the last second equals the desired percentage

---@diagnostic disable: param-type-mismatch
---@diagnostic disable: cast-local-type
---@diagnostic disable: missing-parameter

-- global definitions
local MAV_SEVERITY = {EMERGENCY=0, ALERT=1, CRITICAL=2, ERROR=3, WARNING=4, NOTICE=5, INFO=6, DEBUG=7}
local UPDATE_INTERVAL_MS = 1
local RC_OPTION_SCRIPTING = 300
local RELAY_NUM = 0

-- add new param POI_DIST_MAX
local PARAM_TABLE_KEY = 94
assert(param:add_table(PARAM_TABLE_KEY, "FAN_", 1), "could not add param table")
assert(param:add_param(PARAM_TABLE_KEY, 1, "ENABLE", 1), "could not add FAN_ENABLE param")

--[[
  // @Param: FAN_ENABLE
  // @DisplayName: Fan Control Enable
  // @Description: Enable or disable fan control
  // @Values: 0:Disable, 1:Enable
  // @User: Standard
--]]
local FAN_ENABLE = Parameter("FAN_ENABLE")

-- local variables and definitions
local last_send_text_ms = 0     -- system time of last message to user (used to prevent spamming)
local relay_output_total = 0    -- total number of relay outputs (including both low and high)
local relay_output_high = 0     -- number of high relay outputs (never more than relay_output_total)
local relay_output_last = 0     -- last output to relay (used to avoid unnecessarily changing relay output)

-- send text message to user at no more than 1hz
function send_text(priority, warning_msg)
    local current_time = millis()
    if (current_time - last_send_text_ms) > 1000 then
        gcs:send_text(priority, warning_msg)
        last_send_text_ms = current_time
    end
end

-- initialise relay to off
relay:off(RELAY_NUM)

-- send welcome message
gcs:send_text(MAV_SEVERITY.INFO, "Fan control script started")

-- the main update function called at 1kHz
function update()

    -- exit immediately if not enabled
    if FAN_ENABLE:get() == 0 then
        return update, 1000
    end

    -- get RC fan control channel
    local rc_fan_control = rc:find_channel_for_option(RC_OPTION_SCRIPTING)
    if rc_fan_control == nil then
          send_text(MAV_SEVERITY.ERROR, "Fan control: set RCx_OPTION = " .. RC_OPTION_SCRIPTING)
          return update, UPDATE_INTERVAL_MS
    end

    -- get RC input and convert to value between 0 and 1
    local rc_fan_control_norm = rc_fan_control:norm_input_ignore_trim()
    if not rc_fan_control_norm then
        send_text(MAV_SEVERITY.ERROR, "Fan control: error retrieving RC fan control")
        return update, UPDATE_INTERVAL_MS
    end
    rc_fan_control_norm = (rc_fan_control_norm + 1.0) / 2.0

    -- determine if relay output should be moved high or low
    local relay_output_average = 0
    if relay_output_total > 0 then
        relay_output_average = relay_output_high / relay_output_total
    end
    local relay_output_new = 0
    if relay_output_average < rc_fan_control_norm then
        relay_output_new = 1
    end

    -- set relay to high or low
    if relay_output_new ~= relay_output_last then
        if relay_output_new > 0 then
            relay:on(RELAY_NUM)
        else
            relay:off(RELAY_NUM)
        end
    end
    relay_output_last = relay_output_new

    -- update totals
    relay_output_total = relay_output_total + 1
    if relay_output_new > 0 then
        relay_output_high = relay_output_high + 1
    end

    -- reset totals each 1000 updates
    if relay_output_total >= 1000 then
        -- debug output to user
        gcs:send_text(MAV_SEVERITY.DEBUG, string.format("Fan control: %.2f outputs, %.2f high", rc_fan_control_norm, relay_output_average))
        relay_output_total = 0
        relay_output_high = 0
    end

    return update, UPDATE_INTERVAL_MS
end

return update()
