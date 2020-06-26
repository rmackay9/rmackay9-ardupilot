-- command a Copter to arm and takeoff quickly
-- CAUTION: This script only works for Copter
-- this script waits for the vehicle to be armed and RC6 input > 1800 and then:
--    a) arms the vehicle
--    b) switches to Guided mode
--    c) takeoff to 3m
--    d) switches to RTL mode

local takeoff_alt_above_home = 3
local copter_guided_mode_num = 4
local copter_rtl_mode_num = 6
local stage = 0

-- the main update function that uses the takeoff and velocity controllers to fly a rough square pattern
function update()
  if not arming:is_armed() then -- reset state when disarmed
    stage = 0
  end
  pwm6 = rc:get_pwm(6)
  if pwm6 and pwm6 > 1800 then    -- check if RC6 input has moved high
    if (stage == 0) then          -- Stage0: arm and change to guided mode
      if arming:arm() then        -- arm
        if (vehicle:set_mode(copter_guided_mode_num)) then  -- change to Guided mode
          stage = stage + 1
        end
      end
    elseif (stage == 1) then      -- Stage1: takeoff
      if (vehicle:start_takeoff(takeoff_alt_above_home)) then
        stage = stage + 1
      end
    elseif (stage == 2) then      -- Stage2: check if vehicle has reached target altitude
      local home = ahrs:get_home()
      local curr_loc = ahrs:get_position()
      if home and curr_loc then
        local vec_from_home = home:get_distance_NED(curr_loc)
        gcs:send_text(0, "alt above home: " .. tostring(math.floor(-vec_from_home:z())))
        if (math.abs(takeoff_alt_above_home + vec_from_home:z()) < 1) then
          stage = stage + 1
          bottom_left_loc = curr_loc          -- record location when starting square
        end
      end
    elseif (stage == 3) then  -- Stage3: change to RTL mode
      vehicle:set_mode(copter_rtl_mode_num)
      stage = stage + 1
      gcs:send_text(0, "switching to RTL")
    end
  end

  return update, 100
end

return update()
