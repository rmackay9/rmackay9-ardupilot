 --[[
    Test barometer altitude change during takeoff
 --]]

local PARAM_TABLE_KEY = 92
local PARAM_TABLE_PREFIX = "BALT_"

local MAV_SEVERITY = {EMERGENCY=0, ALERT=1, CRITICAL=2, ERROR=3, WARNING=4, NOTICE=5, INFO=6, DEBUG=7}
local TEXT_PREFIX_STR = "copter-bara-alt-test:"
local COPTER_MODE_GUIDED = 4
local COPTER_MODE_GUIDED_NOGPS = 20
local AUXFN_SCRIPTING1 = 300
local DISARM_DELAY_MS = 1000          -- number of milliseconds to keep thrust low after test completes before disarming

 -- add a parameter and bind it to a variable
function bind_add_param(name, idx, default_value)
    assert(param:add_param(PARAM_TABLE_KEY, idx, name, default_value), string.format('could not add param %s', name))
    return Parameter(PARAM_TABLE_PREFIX .. name)
end

-- setup script specific parameters
assert(param:add_table(PARAM_TABLE_KEY, PARAM_TABLE_PREFIX, 3), 'could not add param table')

--[[
  // @Param: BALT_ENABLE
  // @DisplayName: Barometer Alt Test Enable
  // @Description: Barometer Alt Test Enable
  // @Values: 0:Disabled, 1:Enabled
  // @User: Standard
--]]
-- values should match CAMMODEL_ENUM
local BALT_ENABLE = bind_add_param('ENABLE', 1, 1)

--[[
  // @Param: BALT_THST_MAX
  // @DisplayName: Barometer Alt Test Thrust Max
  // @Description: Barometer Alt Test Thrust Max
  // @Range: 0 100
  // @User: Standard
--]]
local BALT_THST_MAX = bind_add_param('THST_MAX', 2, 10)

--[[
  // @Param: BALT_THST_SEC
  // @DisplayName: Barometer Alt Test Duration in Seconds
  // @Description: Barometer Alt Test Duration in Seconds
  // @Range: 0 20
  // @User: Standard
--]]
local BALT_THST_SEC = bind_add_param('THST_SEC', 3, 3)

-- local variables
local switch_high_prev = true    -- last known switch high state.  used to detect changes in switch position. initialised to high for safety
local start_time_ms = uint32_t(0)
local baro_alt_start = 0          -- barometer altitude at start of test
local baro_alt_rel_min = 0        -- minimum barometer altitude relative to start altitude
local baro_alt_print_ms = uint32_t(0)

-- update function. should be called at 10hz or highter
function run_baro_alt_test()

    -- return immediately if not enabled
    if BALT_ENABLE:get() == 0 then
        do return end
    end

    -- check mode
    local mode_ok = vehicle:get_mode() == COPTER_MODE_GUIDED or vehicle:get_mode() == COPTER_MODE_GUIDED_NOGPS

    -- check scripting aux function switch position used to start/stop test
    local aux_switch_pos = rc:get_aux_cached(AUXFN_SCRIPTING1)
    if not aux_switch_pos then
      aux_switch_pos = -1
    end
    local switch_high = aux_switch_pos > 0

    -- detect change in aux switch position
    local switch_high_changed = switch_high ~= switch_high_prev
    switch_high_prev = switch_high

    -- if aux switch moved high
    if switch_high_changed and switch_high then
        -- check flight mode
        if not mode_ok then
            gcs:send_text(MAV_SEVERITY.ERROR, TEXT_PREFIX_STR .. "change to Guided mode")
            do return end
        end
        -- attempt to arm vehicle
        if not arming:is_armed() then
            gcs:send_text(MAV_SEVERITY.INFO, TEXT_PREFIX_STR .. "arming vehicle")
            if not arming:arm() then
                gcs:send_text(MAV_SEVERITY.INFO, TEXT_PREFIX_STR .. "arming failed")
                do return end
            end
        end
        -- set timer to start test
        start_time_ms = millis()
        baro_alt_start = baro:get_altitude()
        baro_alt_rel_min = 0
    end

    -- if aux switch moved low
    if switch_high_changed and not switch_high then
        -- check flight mode and disarm vehicle
        if mode_ok and arming:is_armed() then
            gcs:send_text(MAV_SEVERITY.INFO, TEXT_PREFIX_STR .. "disarming vehicle")
            arming:disarm()
            start_time_ms = uint32_t(0)
            do return end
        end
    end

    -- if switch is high while armed in Guided or Guide_NoGPS then we must be running test
    if switch_high and mode_ok and arming:is_armed() then
        -- sanity check timer
        if start_time_ms == uint32_t(0) then
            gcs:send_text(MAV_SEVERITY.ERROR, TEXT_PREFIX_STR .. "timer was not set")
            start_time_ms = millis()
        end
        -- check how long test has been running
        local now_ms = millis()
        local test_duration_ms = uint32_t(BALT_THST_SEC:get() * 1000)
        local test_complete = (now_ms - start_time_ms) > test_duration_ms
        if not test_complete then
            -- test is running
            -- set vehicle throttle according to BALT_THST_MAX parameter
            local thrust_pct = BALT_THST_MAX:get() * 0.01
            vehicle:set_target_rate_and_throttle(0, 0, 0, thrust_pct)
           
            -- record change in barometer altitude and print at 1hz
            local baro_alt_rel = baro:get_altitude() - baro_alt_start
            if baro_alt_rel < baro_alt_rel_min then
                baro_alt_rel_min = baro_alt_rel
            end
            if (now_ms - baro_alt_print_ms) > 1000 then
                gcs:send_text(MAV_SEVERITY.INFO, TEXT_PREFIX_STR .. string.format("baro alt:%.2f min:%.2f", baro_alt_rel, baro_alt_rel_min))
                baro_alt_print_ms = now_ms
            end
        else
            -- test completed
            local should_disarm = (now_ms - start_time_ms) > (test_duration_ms + DISARM_DELAY_MS)
            if should_disarm then
                gcs:send_text(MAV_SEVERITY.INFO, TEXT_PREFIX_STR .. "disarming vehicle")
                arming:disarm()
                start_time_ms = uint32_t(0)
                gcs:send_text(MAV_SEVERITY.INFO, TEXT_PREFIX_STR .. string.format("baro alt min: %.2f", baro_alt_rel_min))
            end
            vehicle:set_target_rate_and_throttle(0, 0, 0, 0)
            do return end
        end
    end
end

-- print welcome message
gcs:send_text(MAV_SEVERITY.INFO, "copter-baro-alt-test script loaded")

-- update function runs every 100ms
function update()
  run_baro_alt_test()
  return update, 100
end

return update()
