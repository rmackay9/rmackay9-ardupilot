-- sends External Nav odometry quality to GCS at 1hz

local MAV_SEVERITY = {EMERGENCY=0, ALERT=1, CRITICAL=2, ERROR=3, WARNING=4, NOTICE=5, INFO=6, DEBUG=7}

assert(visual_odom, 'could not access visual odometry')

-- play tune on buzzer to alert user to change in active source set
function play_source_tune(source)
  if (source) then
    if (source == 0) then
      notify:play_tune("L8C")       -- one long lower tone
    elseif (source == 1) then
      notify:play_tune("L12DD")     -- two fast medium tones
    elseif (source == 2) then
      notify:play_tune("L16FFF")    -- three very fast, high tones
    end
  end
end

-- convert a boolean to a number
function bool_to_int(b)
  if b then
    return 1
  end
  return 0
end

-- display welcome message
gcs:send_text(MAV_SEVERITY.INFO, string.format("extnav-odometry-quality.lua script loaded"))

-- the main update function
function update()

  -- check external nav quality and innovations
  local extnav_innov
  extnav_innov, _ = ahrs:get_vel_innovations_and_variances_for_source(6)
  if (extnav_innov) then
    local extnav_xyz_innov = extnav_innov:length()
    gcs:send_named_float("ExtNInnov", extnav_xyz_innov)
  else
    gcs:send_named_float("ExtNInnov", -1)
  end
  if visual_odom then
    gcs:send_named_float("ExtNQual", visual_odom:quality())
  else
    gcs:send_named_float("ExtNQual", -1)
  end

  play_source_tune(0)

  -- update at 1hz
  return update, 1000
end

return update()
