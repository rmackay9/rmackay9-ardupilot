--[[
 Display message is vehicle (a copter) enters EKF and/or RC failsafe
--]]

-- global definitions
local MAV_SEVERITY = {EMERGENCY=0, ALERT=1, CRITICAL=2, ERROR=3, WARNING=4, NOTICE=5, INFO=6, DEBUG=7}
local UPDATE_INTERVAL_MS = 100

-- variables to hold last know failsafe state
local last_ekf_failsafe = false
local last_rc_failsafe = false

-- main update function
function update()
  -- check EKF failsafe state change
  local ekf_failsafe = vehicle:has_ekf_failsafed()
  if ekf_failsafe and ekf_failsafe ~= last_ekf_failsafe then
    last_ekf_failsafe = ekf_failsafe
    if ekf_failsafe then
      gcs:send_text(MAV_SEVERITY.EMERGENCY, "FailsafeCheck: EKF failsafe!")
    else
      gcs:send_text(MAV_SEVERITY.INFO, "FailsafeCheck: EKF failsafe cleared")
    end
  end

  -- check RC failsafe state change
  local rc_failsafe = vehicle:has_rc_failsafed()
  if rc_failsafe ~= last_rc_failsafe then
    last_rc_failsafe = rc_failsafe
    if rc_failsafe then
      gcs:send_text(MAV_SEVERITY.EMERGENCY, "FailsafeCheck: RC failsafe!")
    else
      gcs:send_text(MAV_SEVERITY.INFO, "FailsafeCheck: RC failsafe cleared")
    end
  end

  -- check again in 0.1 second
  return update, UPDATE_INTERVAL_MS
end

-- start running update loop
return update()
