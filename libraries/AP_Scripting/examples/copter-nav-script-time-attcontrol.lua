-- Copter flies in a specified attitude in Auto mode in response to a NAV_SCRIPT_TIME mission command
--
-- How to use:
--   1. Create a simple mission like below
--        TAKEOFF, Alt:10m
--        NAV_SCRIPT_TIME, command=0 (not used), timeout=10 (seconds), arg1=-10 (pitch forward 10 deg), arg2=90 (face east)
--        NAV_SCRIPT_TIME, command=0 (not used), timeout=10 (seconds), arg1=10 (pitch back 10 deg), arg2=90 (face east)
--        RETURN-TO-LAUNCH
--   2. Arm vehicle in Loiter, arm, switch to Auto mode (or set AUTO_OPTIONS=3 and arm in Auto)
--   3. Vehicle should do this
--        Climb to 10m
--        Face east and pitch forward at 10 deg for 10 sec
--        Face east and pitch backwards at 10 deg for 10 sec
--        Return-to-home
--
-- Testing that it does not require GPS (in SITL):
--   a. set map setshowsimpos 1 (to allow seeing where vehicle really is in simulator even with GPS disabled)
--   b. follow steps 1 and 2 above
--   c. after vehicle begins first NAV_SCRIPT_TIME command set SIM_GPS_DISABLE = 1 to disable GPS (confirm vehicle keeps flying)
--   c. before 2nd NAV_SCRIPT_TIME command completes set SIM_GPS_DISABLE = 0 to re-enable GPS
--
-- Testing that it does not require RC (in SITL):
--   a. set FS_OPTIONS's "Continue if in Guided on RC failsafe" bit
--   b. set FS_GCS_ENABLE = 1 (to enable GCS failsafe otherwise RC failsafe will trigger anyway)
--   c. optionally set SYSID_MYGCS = 77 (or almost any other unused system id) to trick the above check so that GCS failsafe can really be disabled
--   d. set SIM_RC_FAIL = 1 (to simulate RC failure, note vehicle keeps flying)
--   e. set SIM_RC_FAIL = 0 (to simulate RC recovery)
--
-- Test with wind (in SITL)
--   a. SIM_WIND_DIR <-- sets direction wind is coming from
--   b. SIM_WIND_SPD <-- sets wind speed in m/s
--
-- NAV_SCRIPT_TIME argument interpretation
--    command = 0 (not used)
--    timeout = 10 (seconds to fly at the specified attitude).  If 0 the command will never complete (vehicle will keep flying!)
--    arg1 = pitch angle in degrees.  -10 = pitch forward, +10 = pitch backwards
--    arg2 = yaw angle in degrees. 0=North, 90=East, 180=South, 270=West

local running = false           -- true if a NAV_SCRIPT_TIME command is being executed
local last_id = -1              -- unique id used to detect if a new NAV_SCRIPT_TIME command has started
local start_time_ms = 0         -- system time (in milliseconds) current command started
local start_yaw_deg = 0         -- vehicle yaw when script started (used in case of invalid yaw)
local interval_ms = 100         -- update at 10hz

function update()

  id, cmd, arg1, arg2 = vehicle:nav_script_time()   -- cmd not used, arg1 = pitch, arg2 = yaw
  if id then
    -- handle start of new command
    local update_user = false
    if id ~= last_id then
      last_id = id
      start_time_ms = now_ms
      start_yaw_deg = math.deg(ahrs:get_yaw())
      running = true
      update_user = true
    end

    -- sanity check angles
    local pitch_deg = arg1
    local yaw_deg = arg2
    if math.abs(pitch_deg) > 45 or math.abs(yaw_deg) > 360 then
      pitch_deg = 0
      yaw_deg = 0
      if update_user then
        gcs:send_text(0, "nav-script-time: invalid pit:" .. tostring(arg1) .. " or yaw:" .. tostring(arg2))
      end
      -- mark command done so mission can continue
      vehicle:nav_script_time_done(last_id)
    end

    -- set attitude target
    local climb_rate = 0
    vehicle:set_target_angle_and_climbrate(0, pitch_deg, yaw_deg, climb_rate, false, 0)

    -- update user
    if update_user then
      gcs:send_text(5, "nav-script-time-att: pit:" .. tostring(pitch_deg) .. " yaw:" .. tostring(yaw_deg))
    end

    -- never set command as done, rely on higher level timeout
  else
    -- no active command
    if running then
      gcs:send_text(5, "nav-script-time-att: complete")
    end
    last_id = -1
    running = false
  end

  return update, interval_ms
end

return update()
