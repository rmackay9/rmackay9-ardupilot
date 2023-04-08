-- iyobe-square-mission.lua: creates a square mission based on the vehicle's current Location and heading
--
-- How to use
--   Set RCx_OPTION = 300 (Scripting1) to enable triggering the script from an RC switch (optional)
--   Set SCR_ENABLE = 1 to enable scripting and reboot the autopilot
--   Copy this script to the autopilot's SD card in the APM/scripts directory and reboot the autopilot
--   Set IYOB_ALT to the alt-above-home that the vehicle should fly at
--   Set IYOB_WP1_DIST to the distance from home to the first waypoint
--   Set IYOB_WP2_DIST to the distance from WP1 to WP2 (length)
--   Set IYOB_WP3_DIST to the distance from WP2 to WP3 (width)
--
--   Switch the RC switch (see above) to the high position to rewrite the mission
--   Alternatively use MP's Data scren's Aux Function tab to "Scripting1" and push the "High" button
--   From MP's Plan screen, push the "Read" button to see the updated mission

-- global definitions
local MAV_SEVERITY_ERROR = 3            -- send text severity of error
local MAV_SEVERITY_INFO = 6             -- send text severity of info
local MAV_CMD_NAV_WAYPOINT = 16         -- waypoint command
local MAV_CMD_NAV_LAND = 21             -- land command
local MAV_CMD_NAV_TAKEOFF = 22          -- takeoff command
local MAV_CMD_NAV_DELAY = 93            -- nav delay command
local MAV_CMD_DO_JUMP = 177             -- do-jump command
local MAV_CMD_DO_SET_SERVO = 183        -- do-set-servo command
local MAV_FRAME_GLOBAL_RELATIVE_ALT = 3 -- relative alt
local UPDATE_INTERVAL_MS = 1000         -- update at 1hz
local AUX_FUNCTION_NUM = 300            -- RCx_OPTION == 300

-- setup param block for aerobatics, reserving 30 params beginning with AERO_
local PARAM_TABLE_KEY = 113
local PARAM_TABLE_PREFIX = "IYOB_"
assert(param:add_table(PARAM_TABLE_KEY, "IYOB_", 9), 'could not add param table')

-- add a parameter and bind it to a variable
function bind_add_param(name, idx, default_value)
    assert(param:add_param(PARAM_TABLE_KEY, idx, name, default_value), string.format('could not add param %s', name))
    return Parameter(PARAM_TABLE_PREFIX .. name)
end

local IYOB_ALT = bind_add_param("ALT", 1, 3)                -- mission altitude.  default is 3m
local IYOB_WP1_DIST = bind_add_param("WP1_DIST", 2, 5)      -- distance from takeoff to 1st waypoint.  default is 5m
local IYOB_WP2_DIST = bind_add_param("WP2_DIST", 3, 8)      -- distance from 1st waypoint to 2nd.  default is 8m
local IYOB_WP3_DIST = bind_add_param("WP3_DIST", 4, 5)      -- distance from 2nd waypoint to 3rd.  default is 5m
local IYOB_WP_DELAY = bind_add_param("WP_DELAY", 5, 5)      -- 5 second delay at each waypoint
local IYOB_REPEAT = bind_add_param("REPEAT", 6, 0)          -- number of times to repeat the square (0=fly square only once)
local IYOB_SERVO_CH = bind_add_param("SERVO_CH", 7, 9)      -- servo channel that will be set at each waypoint
local IYOB_SERVO_LOW = bind_add_param("SERVO_LOW", 8, 1000) -- servo channel pwm value between waypoints
local IYOB_SERVO_HIGH = bind_add_param("SERVO_HIGH", 9, 1400) -- servo channel pwm value at waypoints

-- local variables and definitions
local last_aux_pos = nil                -- auxiliary switch last know position.  used to detect change in switch position
local last_func_val = nil
local last_print_ms = 0                 -- debug variables

-- wrap yaw angle to between 0 and 360 degrees
function wrap_360(angle)
  local res = math.fmod(angle, 360.0)
  if res < 0 then
    res = res + 360.0
  end
  return res
end

-- main update function
function update()

  -- check for change in auxiliary switch state
  local aux_pos = rc:get_aux_cached(AUX_FUNCTION_NUM)
  if aux_pos ~= last_aux_pos then
    last_aux_pos = aux_pos
    if aux_pos ~= 2 then
      return update, UPDATE_INTERVAL_MS
    end
  else
    return update, UPDATE_INTERVAL_MS
  end

  -- check vehicle is disarmed
  if arming:is_armed() then
    gcs:send_text(MAV_SEVERITY_ERROR, "IYOB: please disarm vehicle")
    return update, UPDATE_INTERVAL_MS
  end

  -- check we have valid location
  local curr_loc = ahrs:get_location()
  if not curr_loc then
    gcs:send_text(MAV_SEVERITY_ERROR, "IYOB: need location")
    return update, UPDATE_INTERVAL_MS
  end

  -- clear mission
  if not mission:clear() then
    gcs:send_text(MAV_SEVERITY_ERROR, "IYOB: failed to clear mission")
    return update, UPDATE_INTERVAL_MS
  end

  -- add home.  Does not really matter what we write because it will be overwritten
  local m = mavlink_mission_item_int_t()
  m:command(MAV_CMD_NAV_WAYPOINT)
  m:x(curr_loc:lat())
  m:y(curr_loc:lng())
  m:z(0)
  m:frame(MAV_FRAME_GLOBAL_RELATIVE_ALT)
  mission:set_item(mission:num_commands(), m)
  
  -- add takeoff
  m:command(MAV_CMD_NAV_TAKEOFF)
  m:x(0)
  m:y(0)
  m:z(IYOB_ALT:get())
  m:frame(MAV_FRAME_GLOBAL_RELATIVE_ALT)
  mission:set_item(mission:num_commands(), m)

  -- add wp1 using takeoff as a template
  local curr_yaw_ef_deg = math.deg(ahrs:get_yaw())
  local wp_loc = curr_loc:copy()
  wp_loc:offset_bearing(curr_yaw_ef_deg, IYOB_WP1_DIST:get())
  m:command(MAV_CMD_NAV_WAYPOINT)
  m:param1(1)
  m:x(wp_loc:lat())
  m:y(wp_loc:lng())
  mission:set_item(mission:num_commands(), m)

  -- add do-set-servo to set servo pwm output high
  m:command(MAV_CMD_DO_SET_SERVO)
  m:param1(IYOB_SERVO_CH:get())
  m:param2(IYOB_SERVO_HIGH:get())
  m:x(0)
  m:y(0)
  mission:set_item(mission:num_commands(), m)

  -- add nav delay
  m:command(MAV_CMD_NAV_DELAY)
  m:param1(IYOB_WP_DELAY:get())
  m:param2(0)
  m:x(0)
  m:y(0)
  mission:set_item(mission:num_commands(), m)

  -- add do-set-servo to set servo pwm output low
  m:command(MAV_CMD_DO_SET_SERVO)
  m:param1(IYOB_SERVO_CH:get())
  m:param2(IYOB_SERVO_LOW:get())
  m:x(0)
  m:y(0)
  mission:set_item(mission:num_commands(), m)

  -- add wp2
  wp_loc:offset_bearing(wrap_360(curr_yaw_ef_deg), IYOB_WP2_DIST:get())
  m:command(MAV_CMD_NAV_WAYPOINT)
  m:param1(1)
  m:x(wp_loc:lat())
  m:y(wp_loc:lng())
  mission:set_item(mission:num_commands(), m)

  -- add do-set-servo to set servo pwm output high
  m:command(MAV_CMD_DO_SET_SERVO)
  m:param1(IYOB_SERVO_CH:get())
  m:param2(IYOB_SERVO_HIGH:get())
  m:x(0)
  m:y(0)
  mission:set_item(mission:num_commands(), m)

  -- add nav delay
  m:command(MAV_CMD_NAV_DELAY)
  m:param1(IYOB_WP_DELAY:get())
  m:param2(0)
  m:x(0)
  m:y(0)
  mission:set_item(mission:num_commands(), m)

  -- add do-set-servo to set servo pwm output low
  m:command(MAV_CMD_DO_SET_SERVO)
  m:param1(IYOB_SERVO_CH:get())
  m:param2(IYOB_SERVO_LOW:get())
  m:x(0)
  m:y(0)
  mission:set_item(mission:num_commands(), m)

  -- add wp3
  wp_loc:offset_bearing(wrap_360(curr_yaw_ef_deg-90), IYOB_WP3_DIST:get())
  m:command(MAV_CMD_NAV_WAYPOINT)
  m:param1(1)
  m:x(wp_loc:lat())
  m:y(wp_loc:lng())
  mission:set_item(mission:num_commands(), m)

  -- add do-set-servo to set servo pwm output high
  m:command(MAV_CMD_DO_SET_SERVO)
  m:param1(IYOB_SERVO_CH:get())
  m:param2(IYOB_SERVO_HIGH:get())
  m:x(0)
  m:y(0)
  mission:set_item(mission:num_commands(), m)

  -- add nav delay
  m:command(MAV_CMD_NAV_DELAY)
  m:param1(IYOB_WP_DELAY:get())
  m:param2(0)
  m:x(0)
  m:y(0)
  mission:set_item(mission:num_commands(), m)

  -- add do-set-servo to set servo pwm output low
  m:command(MAV_CMD_DO_SET_SERVO)
  m:param1(IYOB_SERVO_CH:get())
  m:param2(IYOB_SERVO_LOW:get())
  m:x(0)
  m:y(0)
  mission:set_item(mission:num_commands(), m)

  -- add wp4
  wp_loc:offset_bearing(wrap_360(curr_yaw_ef_deg-180), IYOB_WP2_DIST:get())
  m:command(MAV_CMD_NAV_WAYPOINT)
  m:param1(1)
  m:x(wp_loc:lat())
  m:y(wp_loc:lng())
  mission:set_item(mission:num_commands(), m)

  -- add do-set-servo to set servo pwm output high
  m:command(MAV_CMD_DO_SET_SERVO)
  m:param1(IYOB_SERVO_CH:get())
  m:param2(IYOB_SERVO_HIGH:get())
  m:x(0)
  m:y(0)
  mission:set_item(mission:num_commands(), m)

  -- add nav delay
  m:command(MAV_CMD_NAV_DELAY)
  m:param1(IYOB_WP_DELAY:get())
  m:param2(0)
  m:x(0)
  m:y(0)
  mission:set_item(mission:num_commands(), m)

  -- add do-set-servo to set servo pwm output low
  m:command(MAV_CMD_DO_SET_SERVO)
  m:param1(IYOB_SERVO_CH:get())
  m:param2(IYOB_SERVO_LOW:get())
  m:x(0)
  m:y(0)
  mission:set_item(mission:num_commands(), m)

  -- add wp5, same as wp1
  wp_loc:offset_bearing(wrap_360(curr_yaw_ef_deg+90), IYOB_WP3_DIST:get())
  m:command(MAV_CMD_NAV_WAYPOINT)
  m:param1(1)
  m:x(wp_loc:lat())
  m:y(wp_loc:lng())
  mission:set_item(mission:num_commands(), m)

  -- add do-set-servo to set servo pwm output high
  m:command(MAV_CMD_DO_SET_SERVO)
  m:param1(IYOB_SERVO_CH:get())
  m:param2(IYOB_SERVO_HIGH:get())
  m:x(0)
  m:y(0)
  mission:set_item(mission:num_commands(), m)

  -- add nav delay
  m:command(MAV_CMD_NAV_DELAY)
  m:param1(IYOB_WP_DELAY:get())
  m:param2(0)
  m:x(0)
  m:y(0)
  mission:set_item(mission:num_commands(), m)

  -- add do-set-servo to set servo pwm output low
  m:command(MAV_CMD_DO_SET_SERVO)
  m:param1(IYOB_SERVO_CH:get())
  m:param2(IYOB_SERVO_LOW:get())
  m:x(0)
  m:y(0)
  mission:set_item(mission:num_commands(), m)

  -- add do-jump
  m:command(MAV_CMD_DO_JUMP)
  m:param1(6)                   -- jump to wp2
  m:param2(IYOB_REPEAT:get())   -- repeat count
  m:x(0)
  m:y(0)
  m:z(0)
  mission:set_item(mission:num_commands(), m)

  -- add wp6, land at current location
  m:command(MAV_CMD_NAV_LAND)
  m:x(curr_loc:lat())
  m:y(curr_loc:lng())
  m:z(0)
  mission:set_item(mission:num_commands(), m)

  gcs:send_text(MAV_SEVERITY_INFO, "IYOB: wrote mission")

  return update, UPDATE_INTERVAL_MS
end

return update()
