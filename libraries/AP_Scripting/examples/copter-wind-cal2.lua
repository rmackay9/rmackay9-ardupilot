-- Copter flies and drifts within a specified radius to collect data to be used to calibrate the wind speed estimation feature
--
-- CAUTION: This script only works for Copter 4.2 (and higher)
-- How to use:
--    a) find a wide open space on a day with low wind
--    b) set WICA_RADIUS to the radius of the testing area minus some safety margin
--    c) arm the vehicle in Loiter and fly to the center of the testing area
--    d) switch the vehicle to Guided mode
--    e) the vehicle will flatten out and drift to the edge of the circle.  The average measured wind speed and direction will be displayed on the HUD
--    f) the vehicle will fly around the circle twice.  The EKF estimated windspeed, direction and RMS difference will be displayed to help the user adjust the EK3_DRAG_MCOEF parameter
--    g) the vehicle will reposition itself back at the center of the testing area and repeat from stage (e)
-- Note: the pilot may retake control at any time by changing the flight mode (to Loiter).  If interrupted the calibration will restart from the beginning

-- constants
COPTER_GUIDED_MODE = 4              -- Guided mode is 4 on copter
COPTER_RTL_MODE = 6                 -- RTL mode is 6 on copter
STAGE_START = 0
STAGE_DRIFT = 1
STAGE_REPOSITION_AT_EDGE = 2
STAGE_FLY_CIRCLE = 3

-- timing and state machine variables
local stage = STAGE_START           -- 0=starting, 1=drifting-to-edge, 2=stopping-at-edge, 3=accelerating, 4=drifting-to-stop, 5=return-to-center, 5=done
local stage_drift_start_ms = 0      -- system time that drift stage started
local stage_drift_yaw_deg = 0       -- drift stage's yaw heading
local stage_circle_speed = 0        -- circling stage's horizontal speed
local stage_circle_angle_rad = 0    -- circling stage's position around the circle in radians (0=North)
local stage_circle_angle_tot_rad = 0    -- total angular rotation around center (used to determine when circling is complete)
local last_update_ms                -- system time of last update
local dt = 0.01                     -- update rate of script (0.01 = 100hz)
local interval_ms = 10              -- update interval in ms (10 = 100hz)
local last_print_ms = 0             -- pilot update timer

-- control related variables
local start_pos = Vector3f()        -- original position when calibration is started

-- create and initialise parameters
local PARAM_TABLE_KEY = 79          -- parameter table key must be used by only one script on a particular flight controller
assert(param:add_table(PARAM_TABLE_KEY, "WICA_", 4), 'could not add param table')
assert(param:add_param(PARAM_TABLE_KEY, 1, 'RADIUS', 40), 'could not add WICA_RADIUS param')    -- testing area radius in meters
assert(param:add_param(PARAM_TABLE_KEY, 2, 'SPEED', 5), 'could not add WICA_SPEED param')       -- horizontal speed while circling
assert(param:add_param(PARAM_TABLE_KEY, 3, 'ACCEL', 1), 'could not add WICA_ACCEL param')       -- horizontal acceleration
assert(param:add_param(PARAM_TABLE_KEY, 4, 'TURNS', 2), 'could not add WICA_TURNS param')       -- number of complete movements around the circle before completion

-- bind parameters to variables
function bind_param(name)
 local p = Parameter()
 assert(p:init(name), string.format('could not find %s parameter', name))
 return p
end

local WICA_RADIUS = bind_param("WICA_RADIUS")   -- testing area radius in meters
local WICA_SPEED = bind_param("WICA_SPEED") -- horizontal speed while circling
local WICA_ACCEL = bind_param("WICA_ACCEL") -- horizontal acceleration 
local WICA_TURNS = bind_param("WICA_TURNS") -- hnumber of complete movements around the circle before completion

-- wrap yaw angle to be within -180 to +180
function wrap_180(yaw_deg)
 local wrapped_yaw = yaw_deg
 if wrapped_yaw > 180 then
   wrapped_yaw = wrapped_yaw - 360
 end
 if wrapped_yaw < -180 then
   wrapped_yaw = wrapped_yaw + 360
 end
 return wrapped_yaw
end

-- wrap yaw angle to be within -PI to +PI (-180deg to + 180deg)
function wrap_PI(yaw_rad)
 local wrapped_yaw_rad = yaw_rad
 if wrapped_yaw_rad > math.pi then
   wrapped_yaw_rad = wrapped_yaw_rad - (math.pi * 2)
 end
 if wrapped_yaw_rad < -math.pi then
   wrapped_yaw_rad = wrapped_yaw_rad + (math.pi * 2)
 end
 return wrapped_yaw_rad
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
    return update, interval_ms
  end

  local curr_pos = ahrs:get_relative_position_NED_origin()

  -- START: capture current position as starting point and begin drifting
  if (stage == STAGE_START) then
    if curr_pos then
      gcs:send_text(6, "WindEstCal: starting")
      start_pos = curr_pos
      stage = STAGE_DRIFT
      stage_drift_start_ms = now_ms
      stage_drift_yaw_deg = math.deg(ahrs:get_yaw())
    end
  end

  -- DRIFT: drift to edge of circle
  if (stage == STAGE_DRIFT) then
    if curr_pos then
      vehicle:set_target_angle_and_climbrate(0, 0, stage_drift_yaw_deg, 0, true, 0)  -- level vehicle
      local dist_from_center = (curr_pos:xy() - start_pos:xy()):length()
      if update_user then
        gcs:send_text(6, string.format("WindEstCal: drifted %d of %d", math.floor(dist_from_center), WICA_RADIUS:get()))
      end
      if dist_from_center >= WICA_RADIUS:get() then
        local drift_time_sec = (now_ms - stage_drift_start_ms):tofloat() / 1000.0
        local drift_speed = dist_from_center / drift_time_sec
        stage_circle_angle_rad = (curr_pos:xy() - start_pos:xy()):angle()
        stage_circle_angle_tot_rad = 0
        gcs:send_text(6, "WindEstCal: wspd:" .. tostring(drift_speed) .. " dir:" .. tostring(math.floor(wrap_180(math.deg(stage_circle_angle_rad) + 180))))
        stage = STAGE_FLY_CIRCLE
      end
    end
  end

  -- FLY_CIRCLE: fly clockwise in circle
  if (stage == STAGE_FLY_CIRCLE) then
    if curr_pos then

      -- check for slowing
      local slowing = false
      if (stage_circle_angle_tot_rad > WICA_TURNS:get() * math.pi * 2) then
        slowing = true
      end

      -- update speeds
      if slowing then
        stage_circle_speed = math.max(stage_circle_speed - (WICA_ACCEL:get() * dt), 0)  -- decelerate horizontal to zero
      else
        stage_circle_speed = math.min(stage_circle_speed + (WICA_ACCEL:get() * dt), WICA_SPEED:get()) -- accelerate horizontal speed to maximum
      end
      local ang_vel_rads = stage_circle_speed / WICA_RADIUS:get()

      -- check for completion
      if (slowing and (ang_vel_rads <= 0)) then
        gcs:send_text(6, "WindEstCal: complete")
        vehicle:set_mode(COPTER_RTL_MODE)
      end

      -- increment angular position
      local angle_increment_rad = (ang_vel_rads * dt)
      stage_circle_angle_rad = wrap_PI(stage_circle_angle_rad + angle_increment_rad)
      stage_circle_angle_tot_rad = stage_circle_angle_tot_rad + angle_increment_rad

      -- calculate target position
      local cos_ang = math.cos(stage_circle_angle_rad)
      local sin_ang = math.sin(stage_circle_angle_rad)
      local target_pos = Vector3f()
      target_pos:x(start_pos:x() + (WICA_RADIUS:get() * cos_ang))
      target_pos:y(start_pos:y() + (WICA_RADIUS:get() * sin_ang))
      target_pos:z(start_pos:z())

      -- calculate target velocity
      target_vel = Vector3f()
      target_vel:x(stage_circle_speed * -sin_ang)
      target_vel:y(stage_circle_speed * cos_ang)

      -- calculate target acceleration
      local centrip_accel = 0
      if (WICA_RADIUS:get() > 0) then
        centrip_accel = stage_circle_speed * stage_circle_speed / WICA_RADIUS:get()
      end
      target_accel = Vector3f()
      target_accel:x(centrip_accel * -cos_ang)
      target_accel:y(centrip_accel * -sin_ang)

      -- target yaw is angle to position target on edge of circle + 90 degrees
      local target_yaw_deg = wrap_180(math.deg(stage_circle_angle_rad + math.rad(90)))

      -- send targets to vehicle with yaw target
      vehicle:set_target_posvelaccel_NED(target_pos, target_vel, target_accel, true, target_yaw_deg, false, 0, false)
    end
  end

  return update, interval_ms
end

return update()
