-- eams-point-at-turbine2.lua: aims gimbal at wind turbine using 360 lidar data
--
-- How to use
--    Connect 360 lidar (e.g. RPLidarS1, SF45b) to one of the autopilot's serial ports
--    Set SERIALx_PROTOCOL = 11 (Lidar360) where "x" corresponds to the serial port connected to the lidar
--    Set PRX1_TYPE = 5 (RPLidar) or 8 (SF45b)
--    Set SCR_ENABLE = 1 to enable scripting and reboot the autopilot

-- setup param block for aerobatics, reserving 30 params beginning with AERO_
local PARAM_TABLE_KEY = 112
local PARAM_TABLE_PREFIX = "EAMS_"
assert(param:add_table(PARAM_TABLE_KEY, "EAMS_", 5), 'could not add param table')

-- add a parameter and bind it to a variable
function bind_add_param(name, idx, default_value)
    assert(param:add_param(PARAM_TABLE_KEY, idx, name, default_value), string.format('could not add param %s', name))
    return Parameter(PARAM_TABLE_PREFIX .. name)
end

local DEBUG = bind_add_param("DEBUG", 2, 0)             -- debug 0:Disabled, 1:Enabled
local PITCH = bind_add_param("PITCH", 5, 0)             -- pitch angle default in degrees (+ve up, -ve down).  zero to use current pitch target

local INIT_INTERVAL_MS = 3000       -- attempt to initialise at this interval
local UPDATE_INTERVAL_MS = 1000     -- update at this interval
local MAV_SEVERITY = {EMERGENCY=0, ALERT=1, CRITICAL=2, ERROR=3, WARNING=4, NOTICE=5, INFO=6, DEBUG=7}
local initialised = false           -- true once applet has been initialised
local success_count = 0             -- count of the number of camera feedback messages sent to GCS

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

-- mavlink message definition
-- initialise mavlink rx with number of messages, and buffer depth
mavlink.init(1, 10)
local messages = {}
messages[180] = { -- CAMERA_FEEDBACK
             { "time_usec", "<I8" },
             { "lat", "<i4" },
             { "lng", "<i4" },
             { "alt_msl", "<f" },
             { "alt_rel", "<f" },
             { "roll", "<f" },
             { "pitch", "<f" },
             { "yaw", "<f" },
             { "foc_len", "<f" },
             { "img_idx", "<I2" },
             { "target_system", "<B" },
             { "cam_idx", "<B" },
             { "flags", "<B" },
             { "completed_captures", "<I2" },
             }

function encode(msgid, message, messages_array)
  local message_map = messages_array[msgid]
  if not message_map then
    -- we don't know how to encode this message, bail on it
    error("Unknown MAVLink message " .. msgid)
  end

  local packString = "<"
  local packedTable = {}                  
  local packedIndex = 1
  for i,v in ipairs(message_map) do
    if v[3] then
      packString = (packString .. string.rep(string.sub(v[2], 2), v[3]))
      for j = 1, v[3] do
        packedTable[packedIndex] = message[message_map[i][1]][j]
        packedIndex = packedIndex + 1
      end
    else
      packString = (packString .. string.sub(v[2], 2))
      packedTable[packedIndex] = message[message_map[i][1]]
      packedIndex = packedIndex + 1
    end
  end

  return string.pack(packString, table.unpack(packedTable))
end

-- send CAMERA_FEEDBACK message to GCS
function send_camera_feedback(lat_degE7, lon_degE7, alt_msl_m, alt_rel_m, roll_deg, pitch_deg, yaw_deg, foc_len_mm, feedback_flags, captures_count)
  -- prepare camera feedback msg
  local camera_feedback_msg = {
      time_usec = micros():toint(),
      target_system = 0,
      cam_idx = 0,
      img_idx = 1,
      lat = lat_degE7,
      lng = lon_degE7,
      alt_msl = alt_msl_m,
      alt_rel = alt_rel_m,
      roll = roll_deg,
      pitch = pitch_deg,
      yaw = yaw_deg,
      foc_len = foc_len_mm,
      flags = feedback_flags,
      completed_captures = captures_count
  }

  -- send camera feedback msg
  local encoded_msg = encode(180, camera_feedback_msg, messages)
  mavlink.send_chan(0, 180, encoded_msg)
  mavlink.send_chan(1, 180, encoded_msg)
end

function update()

  -- init
  if not initialised then
    initialised = init()
    return update, INIT_INTERVAL_MS
  end

  -- send detction parameters
  --proximity:set_sweep_params(DEBUG:get(), JUMP_DIST:get(), ANGLE_MIN:get(), ANGLE_MAX:get())

  -- check for closest object found during last sweep  
  local obj_loc, angle_deg = oadatabase:get_largest_object()
  if angle_deg then
    if DEBUG:get() > 0 then
      gcs:send_text(MAV_SEVERITY.INFO, string.format("EAMS: closest at %f deg", angle_deg))
    end
    -- pitch angle comes from parameter if non-zero, otherwise uses current target pitch angle
    local target_pitch = PITCH:get()
    if target_pitch == 0 then
      local _, curr_target_pitch, _, _ = mount:get_angle_target(0)
      if curr_target_pitch then
        target_pitch = curr_target_pitch
      end
    end
    mount:set_roi_and_pitch_target(0, obj_loc, target_pitch)

    -- send feedback to GCS so it can display icon on map
    success_count = success_count + 1
    send_camera_feedback(obj_loc:lat(), obj_loc:lng(), obj_loc:alt(), obj_loc:alt(), 0, 0, 0, 0, 0, success_count)
  end

  return update, UPDATE_INTERVAL_MS
end

return update(), 2000 -- first message may be displayed 2 seconds after start-up
