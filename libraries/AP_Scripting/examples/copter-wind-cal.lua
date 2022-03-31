-- Copter flies and drifts within a specified radius to collect data to be used to calibrate the wind speed estimation feature
--
-- CAUTION: This script only works for Copter 4.2 (and higher)
-- How to use:
--    a) find a wide open space on a day with low wind
--    b) set WICA_RADIUS to the radius of the testing area minus some safety margin
--    c) arm the vehicle in Loiter and fly to the center of the testing area
--    d) switch the vehicle to Guided mode
--    e) the vehicle will flatten out and drift to the edge of the circle
--    f) the vehicle will stop, rotate back to face towards the center of the circle and accelerate hard and then level out again
--    g) the vehicle will reposition itself back at the center of the testing area and repeat 3 more times in the right, back and left directions
--    h) the vehicle will reposition itself back at the center allowing the pilot to retake control
-- Note: the pilot may retake control at any time by changing the flight mode (to Loiter).  If interrupted the calibration will restart from the beginning

-- constants
COPTER_GUIDED_MODE = 4              -- Guided mode is 4 on copter
COPTER_RTL_MODE = 6                 -- RTL mode is 6 on copter
STAGE_START = 0
STAGE_MOVE_TO_CENTER = 1
STAGE_DRIFT = 2
STAGE_REPOSITION_AT_EDGE = 3
STAGE_ACCELERATE = 4
STAGE_SLOWDOWN = 5

-- timing and state machine variables
local stage = STAGE_START           -- 0=starting, 1=drifting-to-edge, 2=stopping-at-edge, 3=accelerating, 4=drifting-to-stop, 5=return-to-center, 5=done
local stage_relyaw = 0              -- target yaw relative to center.  This is advanced by 90 degrees after SLOWDOWN completes (0=forward, 90=right, 180=back, 270=left)
local stage_repo_delay_start_ms = 0    -- system time that reposition stage delay start
local stage_accelerate_start_ms = 0 -- system time that acceleration stage started
local last_update_ms                -- system time of last update
local dt = 0.01                     -- update rate of script (0.01 = 100hz)
local interval_ms = 10              -- update interval in ms
local last_print_ms = 0             -- pilot update timer

-- control related variables
local start_pos = Vector3f()        -- original position when calibration is started
local repo_at_edge_pos = Vector3f() -- target position at edge of circle used by STAGE_REPOSITION_AT_EDGE
local target_yaw_deg = 0            -- target yaw in degrees (degrees is more convenient based on interface)

-- create and initialise parameters
local PARAM_TABLE_KEY = 79          -- parameter table key must be used by only one script on a particular flight controller
assert(param:add_table(PARAM_TABLE_KEY, "WICA_", 3), 'could not add param table')
assert(param:add_param(PARAM_TABLE_KEY, 1, 'RADIUS', 20), 'could not add WICA_RADIUS param')        -- testing area radius in meters
assert(param:add_param(PARAM_TABLE_KEY, 2, 'ANGLE_MAX', 30), 'could not add WICA_ANGLE_MAX param')  -- lean angle during acceleration
assert(param:add_param(PARAM_TABLE_KEY, 3, 'ACCEL_SEC', 3), 'could not add WICA_ACCEL_SEC param')   -- number of seconds vehicle accelerates

-- bind parameters to variables
function bind_param(name)
 local p = Parameter()
 assert(p:init(name), string.format('could not find %s parameter', name))
 return p
end

local WICA_RADIUS = bind_param("WICA_RADIUS")   -- testing area radius in meters
local WICA_ANGLE_MAX = bind_param("WICA_ANGLE_MAX")   -- lean angle during acceleration
local WICA_ACCEL_SEC = bind_param("WICA_ACCEL_SEC")   -- number of seconds vehicle accelerates

-- wrap yaw angle to be within -180 to +180
function wrap_180(yaw)
 local wrapped_yaw = yaw
 if wrapped_yaw > 180 then
   wrapped_yaw = wrapped_yaw - 360
 end
 if wrapped_yaw < -180 then
   wrapped_yaw = wrapped_yaw + 360
 end
 return wrapped_yaw
end

-- the main update function
function update()

  -- update dt
  local now_ms = millis()
  if (last_update_ms) then
      dt = (now_ms - last_update_ms):tofloat() / 1000.0
  end
  if (dt > 1) then
    dt = 0
  end
  last_update_ms = now_ms

  -- determine if progress update should be sent to user
  local update_user = false
  if (now_ms - last_print_ms > 5000) then
    last_print_ms = now_ms
    update_user = true
  end

  -- reset stage until activated in Guided 
  if not arming:is_armed() or (vehicle:get_mode() ~= COPTER_GUIDED_MODE) then 
    if (update_user and arming:is_armed()) then
      gcs:send_text(6, "WindEstCal: waiting for Guided")
    end

    -- initialise state
    stage = STAGE_START
    stage_relyaw = 0
    return update, interval_ms
  end

  local curr_pos = ahrs:get_relative_position_NED_origin()

  -- START: capture current position as starting point and begin drifting
  if (stage == STAGE_START) then
    if curr_pos then
      gcs:send_text(6, "WindEstCal: starting")
      start_pos = curr_pos      
      stage = STAGE_DRIFT
      target_yaw_deg = math.deg(ahrs:get_yaw())
    end
  end

  -- MOVE_TO_CENTER: move to center of circle
  if (stage == STAGE_MOVE_TO_CENTER) then
    if curr_pos then
      vehicle:set_target_pos_NED(start_pos, false, 0, true, 0, false, false)
      local dist_from_center = (curr_pos:xy() - start_pos:xy()):length()
      if update_user then
        gcs:send_text(6, "WindEstCal: moving to center")
      end
      if dist_from_center < 0.2 then
        stage = STAGE_DRIFT
      end
    end
  end

  -- DRIFT: drift to edge of circle
  if (stage == STAGE_DRIFT) then
    if curr_pos then
      vehicle:set_target_angle_and_climbrate(0, 0, target_yaw_deg, 0, true, 0)  -- level vehicle
      local dist_from_center = (curr_pos:xy() - start_pos:xy()):length()
      if update_user then
        gcs:send_text(6, string.format("WindEstCal: drifted %d of %d", math.floor(dist_from_center), WICA_RADIUS:get()))
      end
      if dist_from_center > WICA_RADIUS:get() then
        gcs:send_text(6, "WindEstCal: reached edge, prepare for acceleration")
        stage = STAGE_REPOSITION_AT_EDGE
        repo_at_edge_pos = curr_pos
        repo_at_edge_pos:z(start_pos:z())
        stage_repo_delay_start_ms = 0
      end
    end
  end

  -- REPOSITION_AT_EDGE: reposition at edge of circle and rotate towards center in preparation for acceleration
  if (stage == STAGE_REPOSITION_AT_EDGE) then
    if curr_pos then

      -- calculate and set yaw target which is heading to center + stage_relyaw (0,90,180 or 270)
      local center_to_curr_pos = start_pos - curr_pos
      local yaw_to_center = math.deg(center_to_curr_pos:xy():angle())
      target_yaw_deg = wrap_180(yaw_to_center + stage_relyaw)
      vehicle:set_target_pos_NED(repo_at_edge_pos, true, target_yaw_deg, false, 0, false, false)

      -- calculate yaw error and update user
      local yaw_diff = wrap_180(math.abs(math.deg(ahrs:get_yaw()) - target_yaw_deg))
      if update_user then
        gcs:send_text(6, string.format("WindEstCal: yaw %d, target %d", math.floor(math.deg(ahrs:get_yaw())), math.floor(target_yaw_deg)))
      end

      -- advance to acceleration stage when yaw is very close to target
      if yaw_diff < 2 then
        -- start delay counter
        if stage_repo_delay_start_ms == 0 then
          stage_repo_delay_start_ms = now_ms
        elseif now_ms - stage_repo_delay_start_ms > 2000 then
          -- after two second delay accelerate
          gcs:send_text(6, "WindEstCal: accelerating!")
          stage = STAGE_ACCELERATE
          stage_accelerate_start_ms = now_ms
        end
      end
    end
  end

  -- ACCELERATE: accelerate towards center of circle
  if (stage == STAGE_ACCELERATE) then
    if curr_pos then

      -- pitch forward
      if stage_relyaw == 0 then      
        vehicle:set_target_angle_and_climbrate(0, -WICA_ANGLE_MAX:get(), target_yaw_deg, 0, false, 0)
      end

      -- roll left
      if stage_relyaw == 90 then
        vehicle:set_target_angle_and_climbrate(-WICA_ANGLE_MAX:get(), 0, target_yaw_deg, 0, false, 0)
      end

      -- pitch back
      if stage_relyaw == 180 then
        vehicle:set_target_angle_and_climbrate(0, WICA_ANGLE_MAX:get(), target_yaw_deg, 0, false, 0)
      end

      -- roll right
      if stage_relyaw == 270 then
        vehicle:set_target_angle_and_climbrate(WICA_ANGLE_MAX:get(), 0, target_yaw_deg, 0, false, 0)
      end

      -- check for timeout
      if (now_ms - stage_accelerate_start_ms > (WICA_ACCEL_SEC:get() * 1000)) then
        stage = STAGE_SLOWDOWN
      end
    end
  end

  -- SLOWDOWN: drift to edge of circle again
  if (stage == STAGE_SLOWDOWN) then
    if curr_pos then
      vehicle:set_target_angle_and_climbrate(0, 0, target_yaw_deg, 0, true, 0)  -- level vehicle
      local dist_from_center = (curr_pos:xy() - start_pos:xy()):length()
      if update_user then
        gcs:send_text(6, string.format("WindEstCal: slowing %d of %d", math.floor(dist_from_center), WICA_RADIUS:get()))
      end
      if dist_from_center > WICA_RADIUS:get() then
        -- advance to next side
        stage_relyaw = stage_relyaw + 90
        if stage_relyaw > 270 then
          -- done! switch to RTL
          vehicle:set_mode(COPTER_RTL_MODE)
        else
          gcs:send_text(6, "WindEstCal: reached edge, returning to center")
          stage = STAGE_MOVE_TO_CENTER
        end
      end
    end
  end

  return update, interval_ms
end

return update()
