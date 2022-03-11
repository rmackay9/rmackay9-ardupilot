-- Rover/Boat saves the active mission item while in Auto and restores it if the autopilot is rebooted
-- Caution: This script only works for Rover 4.2 (and higher)

-- constants
local rover_auto_mode_num = 10      -- auto mode is 3 on rover
local pilot_update_sec = 5          -- update sent to pilot every 5 seconds

-- timing and state machine variables
local interval_ms = 1000            -- update interval in ms
local last_pilot_update_ms = 0      -- system time of last update to pilot
local last_save_cmd = -1            -- command that was last saved (to detect change in active command)
local startup_check_done = false    -- true once startup check has completed

-- create and initialise parameters
local PARAM_TABLE_KEY = 76          -- parameter table key must be used by only one script on a particular flight controller
assert(param:add_table(PARAM_TABLE_KEY, "MSAV_", 2), 'could not add param table')
assert(param:add_param(PARAM_TABLE_KEY, 1, 'ENABLE', 0), 'could not add MSAV_ENABLE param') -- 0:disabled, 1:enabled
assert(param:add_param(PARAM_TABLE_KEY, 2, 'LAST_CMD', -1), 'could not add MSAV_LAST_CMD')  -- latest active auto command number.  -1 if auto not active

-- bind parameters to variables
function bind_param(name)
   local p = Parameter()
   assert(p:init(name), string.format('could not find %s parameter', name))
   return p
end
local msav_enabled = bind_param("MSAV_ENABLE")       -- 0:disabled, 1:enabled
local msav_last_cmd = bind_param("MSAV_LAST_CMD")    -- latest active auto command number.  -1 if auto not active

-- the main update function
function update()

  -- exit immediately if not enabled
  if msav_enabled:get() == 0 then
    return update, interval_ms
  end

  -- startup check
  if not startup_check_done then
    if msav_last_cmd:get() > 0 then
      mission:set_current_cmd(msav_last_cmd:get())
      gcs:send_text(6, "MissionSave: set cmd=" .. tostring(msav_last_cmd:get()))
    else
      gcs:send_text(6, "MissionSave: auto was not active")
    end
    startup_check_done = true
  end

  -- determine if progress update should be sent to user
  local now_ms = millis()
  local update_user = false
  if ((now_ms - last_pilot_update_ms) > (pilot_update_sec * 1000)) then
    last_pilot_update_ms = now_ms
    update_user = true
  end

  -- if armed, in auto mode and running a mission then save active mission command
  local reset_last_cmd = false
  if arming:is_armed() and (vehicle:get_mode() == rover_auto_mode_num) and (mission:state() == 1) then
    local active_cmd = mission:get_current_nav_index()
    if active_cmd ~= msav_last_cmd:get() then
      msav_last_cmd:set_and_save(active_cmd)
      gcs:send_text(6, "MissionSave: cmd=" .. tostring(active_cmd))
    end
  else
    reset_last_cmd = true
  end
  
  -- reset last cmd to -1 if necessary
  if reset_last_cmd and (msav_last_cmd:get() ~= -1) then
    msav_last_cmd:set_and_save(-1)
  end

  return update, interval_ms
end

return update()
