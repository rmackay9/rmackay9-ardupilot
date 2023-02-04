-- eams-point-at-turbine.lua: aims gimbal at wind turbine using AP_Proximity's sweep feature
--
-- How to use
--   Connect SF45b to one of the autopilot's serial ports
--   Set SERIALx_PROTOCOL = 28 (Scripting) where "x" corresponds to the serial port connected to the AI camera
--   Set SCR_ENABLE = 1 to enable scripting and reboot the autopilot
--   Copy this script to the autopilot's SD card in the APM/scripts directory and reboot the autopilot

-- setup param block for aerobatics, reserving 30 params beginning with AERO_
local PARAM_TABLE_KEY = 112
local PARAM_TABLE_PREFIX = "EAMS_"
assert(param:add_table(PARAM_TABLE_KEY, "EAMS_", 4), 'could not add param table')

-- add a parameter and bind it to a variable
function bind_add_param(name, idx, default_value)
    assert(param:add_param(PARAM_TABLE_KEY, idx, name, default_value), string.format('could not add param %s', name))
    return Parameter(PARAM_TABLE_PREFIX .. name)
end

local JUMP_DIST = bind_add_param("JUMP_DIST", 1, 0.5)   -- object edge detected by a lidar jump of this many meters
local DEBUG = bind_add_param("DEBUG", 2, 0)             -- debug 0:Disabled, 1:Enabled
local ANGLE_MIN = bind_add_param("ANGLE_MIN", 3, -45)   -- minimum angle to check for obstacles
local ANGLE_MAX = bind_add_param("ANGLE_MAX", 4, 45)    -- maximum angle to check for obstacles

local INIT_INTERVAL_MS = 3000       -- attempt to initialise at this interval
local UPDATE_INTERVAL_MS = 1000     -- update at this interval
local MAV_SEVERITY = {EMERGENCY=0, ALERT=1, CRITICAL=2, ERROR=3, WARNING=4, NOTICE=5, INFO=6, DEBUG=7}
local initialised = false           -- true once applet has been initialised

-- run initialisation checks
-- returns true on success, false on failure to initialise
function init()
  -- check proximity sensor has been enabled
  if proximity:num_sensors() < 1 then
    gcs:send_text(MAV_SEVERITY.CRITICAL, "EAMS: no proximity sensor found")
    return false
  end

  -- if we get this far then initialisation has completed successfully
  gcs:send_text(MAV_SEVERITY.CRITICAL, "EAMS: started")
  return true
end

function update()

  -- init
  if not initialised then
    initialised = init()
    return update, INIT_INTERVAL_MS
  end

  -- send detction parameters
  proximity:set_sweep_params(DEBUG:get(), JUMP_DIST:get(), ANGLE_MIN:get(), ANGLE_MAX:get())

  -- check for closest object found during last sweep  
  local angle_deg = proximity:get_closest_sweep_object()
  if angle_deg then
    if DEBUG:get() > 0 then
      gcs:send_text(MAV_SEVERITY.INFO, string.format("EAMS: closest at %f deg", angle_deg))
    end
    mount:set_angle_target(0, 0, 0, angle_deg, false)
  end

  return update, UPDATE_INTERVAL_MS
end

return update(), 2000 -- first message may be displayed 2 seconds after start-up
