-- eams-windturbine.lua: aims gimbal at wind turbine based on packets received from an external AI camera
--
-- How to use
--   Connect AI camera UART to one of the autopilot's serial ports
--   Set SERIALx_PROTOCOL = 28 (Scripting) where "x" corresponds to the serial port connected to the AI camera
--   Set SCR_ENABLE = 1 to enable scripting and reboot the autopilot
--   Copy this script to the autopilot's SD card in the APM/scripts directory and reboot the autopilot
--   Adjust EATB_RATE_P and I to adjust the control response to the camera input
--   Set EATB_RATE_MAX to the maximum rate in deg/sec that you want the gimbal to rotate
--   Set EATB_CONFID to the maximum usable confidence from the camera (note the confidence is strangely reversed)
--   Set EATB_DEBUG = 1 to see debug output on the GCS messages tab.  Set to "2" to use test input messages (e.g. fake the camera input)

--
-- Packet format
--    byte      description         notes
--    0         header              EA
--    1         message id          2=margin info
--    2         data length         number of bytes of data
--    3         left margin MSB     percent * 10
--    4         left margin LSB
--    5         right margin MSB    percent * 10
--    6         right margin LSB
--    7         confidence (rev)    percent (0 ~ 100).  0 = full confidence, 100 = no confidence
--    8         checksum            covers message id to confidence (inclusive). see https://crccalc.com
--

-- global definitions
local INIT_INTERVAL_MS = 3000           -- attempt initialisation at 3hz
local UPDATE_INTERVAL_MS = 20           -- update at 50hz
local PID_UPDATE_INTERVAL_MS = 1000     -- update PIDs from parametsre at 1hz
local CONTROL_TIMEOUT_MS = 1000         -- stop moving gimbal after this many seconds with no successful input from AI camera
local HEALTHY_TIMEOUT_MS = 3000         -- camera considered unhealthy if no messages for this many ms
local RECENTER_TIMEOUT_MS = 5000        -- camera is recentered if no messages for this many ms
local MOUNT_INSTANCE = 0                -- always control the first mount/gimbal
local HEADER = 0xEA                     -- AI camera packet header
local MARGIN_MSGID = 0x02               -- AI camera margin message id
local MARGIN_DATALEN = 0x05             -- margin message is always datalen of 5

-- setup param block for aerobatics, reserving 30 params beginning with AERO_
local PARAM_TABLE_KEY = 111
local PARAM_TABLE_PREFIX = "EATB_"
assert(param:add_table(PARAM_TABLE_KEY, "EATB_", 6), 'could not add param table')

-- add a parameter and bind it to a variable
function bind_add_param(name, idx, default_value)
    assert(param:add_param(PARAM_TABLE_KEY, idx, name, default_value), string.format('could not add param %s', name))
    return Parameter(PARAM_TABLE_PREFIX .. name)
end

local PID_RATE_MAX = bind_add_param("RATE_MAX", 1, 20)      -- mount rate controller max in degs
local PID_RATE_P = bind_add_param("RATE_P", 2, 1)           -- mount rate controller P gain
local PID_RATE_I = bind_add_param("RATE_I", 3, 0.1)         -- mount rate controller I gain
local PID_RATE_IMAX = bind_add_param("RATE_IMAX", 4, 20)    -- mount rate controller I max
local CONFID = bind_add_param("CONFID", 5, 50)              -- acceptable confidence threshold from AI camera (note: 0=full confidence, 100=no confidence)
local DEBUG = bind_add_param("DEBUG", 6, 0)                 -- debug 0:Disabled, 1:Enabled, 2:Enabled and use fake data

-- parsing state definitions
local PARSE_STATE_WAITING_FOR_HEADER    = 0
local PARSE_STATE_WAITING_FOR_MSGID     = 1
local PARSE_STATE_WAITING_FOR_DATALEN   = 2
local PARSE_STATE_WAITING_FOR_DATA      = 3
local PARSE_STATE_WAITING_FOR_CHECKSUM  = 4

-- hardcoded example messages for testing
local TEST_MSG1 = {0xEA,0x02,0x05,0x00,0x64,0x00,0xc8,0x63,0xE1}
local test_msg_byte = 0

-- local variables and definitions
local uart                              -- uart object connected to mount
local initialised = false               -- true once connection to gimbal has been initialised
local parse_state = PARSE_STATE_WAITING_FOR_HEADER -- parse state
local parse_expected_crc = 0            -- incoming messages expected crc.  this is checked against actual crc received
local parse_buff = {}                   -- buffer holding data portion of message received so far
local parse_buff_num = 0                -- number of data bytes received so far
local parse_msgid = 0                   -- most recently received message id
local parse_datalen = 0                 -- most recently received datalen
local parse_success_ms = 0              -- system time message was last successful parsed (used for health reporting)
local control_success_ms = 0            -- system time of last rate control message sent to gimbal
local control_stopped = false           -- gimbal is stopped after this long with low confidence messages
local camera_healthy = false            -- used for health reporting to use
local camera_recentered = false         -- used to recenter the camera after for health reporting to use
local last_pid_update_ms = 0            -- system time of last PID parameter update

-- debug variables
local last_print_ms = 0
local bytes_read = 0

-- CRC8 calculation table
local crc8_table = {
    0x00, 0x07, 0x0e, 0x09, 0x1c, 0x1b, 0x12, 0x15, 0x38, 0x3f, 0x36, 0x31,
    0x24, 0x23, 0x2a, 0x2d, 0x70, 0x77, 0x7e, 0x79, 0x6c, 0x6b, 0x62, 0x65,
    0x48, 0x4f, 0x46, 0x41, 0x54, 0x53, 0x5a, 0x5d, 0xe0, 0xe7, 0xee, 0xe9,
    0xfc, 0xfb, 0xf2, 0xf5, 0xd8, 0xdf, 0xd6, 0xd1, 0xc4, 0xc3, 0xca, 0xcd,
    0x90, 0x97, 0x9e, 0x99, 0x8c, 0x8b, 0x82, 0x85, 0xa8, 0xaf, 0xa6, 0xa1,
    0xb4, 0xb3, 0xba, 0xbd, 0xc7, 0xc0, 0xc9, 0xce, 0xdb, 0xdc, 0xd5, 0xd2,
    0xff, 0xf8, 0xf1, 0xf6, 0xe3, 0xe4, 0xed, 0xea, 0xb7, 0xb0, 0xb9, 0xbe,
    0xab, 0xac, 0xa5, 0xa2, 0x8f, 0x88, 0x81, 0x86, 0x93, 0x94, 0x9d, 0x9a,
    0x27, 0x20, 0x29, 0x2e, 0x3b, 0x3c, 0x35, 0x32, 0x1f, 0x18, 0x11, 0x16,
    0x03, 0x04, 0x0d, 0x0a, 0x57, 0x50, 0x59, 0x5e, 0x4b, 0x4c, 0x45, 0x42,
    0x6f, 0x68, 0x61, 0x66, 0x73, 0x74, 0x7d, 0x7a, 0x89, 0x8e, 0x87, 0x80,
    0x95, 0x92, 0x9b, 0x9c, 0xb1, 0xb6, 0xbf, 0xb8, 0xad, 0xaa, 0xa3, 0xa4,
    0xf9, 0xfe, 0xf7, 0xf0, 0xe5, 0xe2, 0xeb, 0xec, 0xc1, 0xc6, 0xcf, 0xc8,
    0xdd, 0xda, 0xd3, 0xd4, 0x69, 0x6e, 0x67, 0x60, 0x75, 0x72, 0x7b, 0x7c,
    0x51, 0x56, 0x5f, 0x58, 0x4d, 0x4a, 0x43, 0x44, 0x19, 0x1e, 0x17, 0x10,
    0x05, 0x02, 0x0b, 0x0c, 0x21, 0x26, 0x2f, 0x28, 0x3d, 0x3a, 0x33, 0x34,
    0x4e, 0x49, 0x40, 0x47, 0x52, 0x55, 0x5c, 0x5b, 0x76, 0x71, 0x78, 0x7f,
    0x6a, 0x6d, 0x64, 0x63, 0x3e, 0x39, 0x30, 0x37, 0x22, 0x25, 0x2c, 0x2b,
    0x06, 0x01, 0x08, 0x0f, 0x1a, 0x1d, 0x14, 0x13, 0xae, 0xa9, 0xa0, 0xa7,
    0xb2, 0xb5, 0xbc, 0xbb, 0x96, 0x91, 0x98, 0x9f, 0x8a, 0x8d, 0x84, 0x83,
    0xde, 0xd9, 0xd0, 0xd7, 0xc2, 0xc5, 0xcc, 0xcb, 0xe6, 0xe1, 0xe8, 0xef,
    0xfa, 0xfd, 0xf4, 0xf3
}

-- constrain a value between limits
function constrain(v, vmin, vmax)
   if v < vmin then
      v = vmin
   end
   if v > vmax then
      v = vmax
   end
   return v
end

-- a PI controller implemented as a Lua object
local function PI_controller(kP, kI, iMax)
  -- the new instance. You can put public variables inside this self
  -- declaration if you want to
  local self = {}

  -- private fields as locals
  local _kP = kP or 0.0    -- P gain
  local _kI = kI or 0.0    -- I gain
  local _iMax = iMax or 0.0-- IMAX value
  local _last_t = nil      -- system time (in millis) that update was last called
  local _P = 0             -- most recent P output
  local _I = 0             -- most recent I output
  local _total = 0         -- most recent output (used for logging)
  local _target = 0        -- most recent target (used for logging)
  local _current = 0       -- most recent current (used for logging)

  -- update the controller.
  function self.update(target, current)
    local now = millis():tofloat() * 0.001
    if not _last_t then
      _last_t = now
    end
    local dt = now - _last_t
    _last_t = now
    local err = target - current

    _P = _kP * err
    _I = _I + _kI * err * dt
    _I = constrain(_I, -_iMax, iMax)

    _target = target
    _current = current
    _total = ret
    return _P + _I
  end

  -- reset integrator to an initial value
  function self.reset(integrator)
    if integrator then
      _I = integrator
    else
      _I = 0
    end
  end

  function self.set_P(P)
    _kP = P
  end

  function self.set_I(I)
    _kI = I
  end

  function self.set_Imax(Imax)
    _iMax = Imax
  end

  -- log the controller internals
  function self.log(name, add_total)
    -- allow for an external addition to total
    logger.write(name,'Targ,Curr,P,I,Total,Add','ffffff',_target,_current,_P,_I,_total,add_total)
  end
  -- return the instance
  return self
end

local rate_PI = PI_controller(PID_RATE_P:get(), PID_RATE_I:get(), PID_RATE_IMAX:get())

-- find and initialise serial port connected to gimbal
function init()
  uart = serial:find_serial(0)    -- 1st instance of SERIALx_PROTOCOL = 28 (Scripting)
  if uart == nil then
    gcs:send_text(3, "EAMSTurbine: no SERIALx_PROTOCOL = 28") -- MAV_SEVERITY_ERR
  else
    --uart:begin(115200)
    uart:begin(921600)
    uart:set_flow_control(0)
    initialised = true
  end
end

-- send hard coded message
function get_test_byte()
  test_msg_byte = test_msg_byte + 1
  if test_msg_byte > #TEST_MSG1 then
    test_msg_byte = 1
  end
  return TEST_MSG1[test_msg_byte]
end

-- return checksum based on a new byte
function update_crc(b, checksum)
  local b_lowbyte = b & 0xFF
  local i = (checksum ~ b_lowbyte) & 0xFF
  local checksum_updated = (crc8_table[i+1] ~ (checksum << 8)) & 0xFF
  return checksum_updated
end

-- rotate a mount's yaw at a given rate
function rotate_mount(yaw_rate_degs)
  -- roll_rate_degs=0, pitch_rate_degs=0, yaw in earth-frame
  mount:set_rate_target(MOUNT_INSTANCE, 0, 0, yaw_rate_degs, true)
end

-- recenter mount
function recenter_mount()
  target_pitch = 0
  -- get current mount pitch target
  local _curr_target_roll, curr_target_pitch, _curr_target_yaw, _yaw_is_ef = mount:get_angle_target(0)
  if curr_target_pitch then
    target_pitch = curr_target_pitch
  end
  mount:set_angle_target(0, target_pitch, 0, 0, false)
end

-- reading incoming packets from gimbal
function read_incoming_packets()
  local n_bytes = uart:available()

  -- EATB_DEBUG=2 passes in test bytes
  if DEBUG:get() >= 2 then
    n_bytes = 1
  end

  while n_bytes > 0 do
    n_bytes = n_bytes - 1
    bytes_read = bytes_read + 1

    -- EATB_DEBUG=2 passes in test bytes
    if DEBUG:get() >= 2 then
      b = get_test_byte();
    else
      b = uart:read()
    end

    -- waiting for header
    if parse_state == PARSE_STATE_WAITING_FOR_HEADER and b == HEADER then
      parse_state = PARSE_STATE_WAITING_FOR_MSGID
      parse_expected_crc = 0
      parse_buff_num = 0
      break
    end

    -- waiting for message id
    if parse_state == PARSE_STATE_WAITING_FOR_MSGID then
      parse_msgid = b
      if parse_msgid ~= MARGIN_MSGID then
        -- unexpected message id so reset parsing state
        parse_state = PARSE_STATE_WAITING_FOR_HEADER
        if DEBUG:get() > 0 then
          gcs:send_text(3, string.format("EAMSTurbine: unexpected msgid:%x", parse_msgid)) -- MAV_SEVERITY_ERR
        end
      else
        parse_state = PARSE_STATE_WAITING_FOR_DATALEN

        -- update checksum.  checksum covers message id to confidence fields
        parse_expected_crc = update_crc(b, parse_expected_crc)
      end
      break
    end

    -- waiting for datalen
    if parse_state == PARSE_STATE_WAITING_FOR_DATALEN then
      parse_datalen = b
      if parse_datalen ~= MARGIN_DATALEN then
        -- unexpected datalen so reset parsing state
        parse_state = PARSE_STATE_WAITING_FOR_HEADER
        if DEBUG:get() > 0 then
          gcs:send_text(3, string.format("EAMSTurbine: unexpected len:%d", parse_datalen)) -- MAV_SEVERITY_ERR
        end
      else
        parse_state = PARSE_STATE_WAITING_FOR_DATA

        -- update checksum.  checksum covers message id to confidence fields
        parse_expected_crc = update_crc(b, parse_expected_crc)
      end
      break
    end

    -- waiting for data
    if parse_state == PARSE_STATE_WAITING_FOR_DATA then

      -- check for crc
      if parse_buff_num >= parse_datalen then
        if b ~= parse_expected_crc then
          -- report checksum  error
          if DEBUG:get() > 0 then
            gcs:send_text(3, string.format("EAMSTurbine: checksum expected:%x got:%x", parse_expected_crc, b)) -- MAV_SEVERITY_ERR
          end
        else
          -- checksum OK, record successful parse
          parse_success_ms = millis()

          -- extract fields from message
          local left_margin = (parse_buff[1] << 8 | parse_buff[2]) * 0.1
          local right_margin = (parse_buff[3] << 8 | parse_buff[4]) * 0.1
          local confidence = parse_buff[5] * 1.0
          local yaw_rate_degs = 0
          if confidence <= CONFID:get() then
             -- ToDo: add scaling based on distance to blade?
            local yaw_err = (right_margin - left_margin) * 0.5
            yaw_rate_degs = rate_PI.update(0, yaw_err)
            rotate_mount(yaw_rate_degs)
            control_success_ms = parse_success_ms
          end
          -- debug
          if DEBUG:get() > 0 then
            gcs:send_text(6, "EAMSTurbine: lm:" .. left_margin .. " rm:" .. right_margin .. " c:" .. confidence .. " yr:" .. yaw_rate_degs)
          end
       end
       parse_state = PARSE_STATE_WAITING_FOR_HEADER
       break
      end

      -- add latest byte to checksum and buffer
      parse_expected_crc = update_crc(b, parse_expected_crc)
      parse_buff_num = parse_buff_num + 1
      parse_buff[parse_buff_num] = b
    end
  end
end

-- the main update function that performs a simplified version of RTL
function update()

  -- get current system time
  local now_ms = millis()

  -- initialise connection to gimbal
  if not initialised then
    init()
    return update, INIT_INTERVAL_MS
  end

  -- debug
  if DEBUG:get() > 0 and now_ms - last_print_ms > 5000 then
    last_print_ms = now_ms
    gcs:send_text(6, string.format("EAMSTurbine: read:%u state:%d", bytes_read, parse_state)) -- MAV_SEVERITY_INFO
  end

  -- update PIDs from parameters
  if now_ms - last_pid_update_ms > PID_UPDATE_INTERVAL_MS then
    last_pid_update_ms = now_ms
    rate_PI.set_P(PID_RATE_P:get())
    rate_PI.set_I(PID_RATE_I:get())
    rate_PI.set_Imax(PID_RATE_IMAX:get())
  end

  -- consume incoming bytes
  read_incoming_packets()

  -- handle control timeouts
  -- these occur if messages stop arriving or the confidence is low
  now_ms = millis()
  if now_ms - control_success_ms > CONTROL_TIMEOUT_MS then
    if not control_stopped then
      rotate_mount(0)
      rate_PI.reset(0)
      control_stopped = true
    end
  else
    control_stopped = false
  end

  -- calc time since last message from camera
  local time_since_update_ms = now_ms - parse_success_ms

  -- recenter gimbal once after 5 sec timout
  if time_since_update_ms > RECENTER_TIMEOUT_MS then
    if not camera_recentered then
      recenter_mount()
      camera_recentered = true
    end
  else
    camera_recentered = false
  end

  -- health reporting
  local health_timeout = time_since_update_ms > HEALTHY_TIMEOUT_MS
  if camera_healthy then
    -- check for camera becoming unhealthy
    if health_timeout then
      camera_healthy = false
      gcs:send_text(3, "EAMSTurbine: camera unhealthy") -- MAV_SEVERITY_ERR
    end
  else
    -- check for camera becoming healthy
    if not health_timeout then
      camera_healthy = true
      gcs:send_text(6, "EAMSTurbine: camera OK") -- MAV_SEVERITY_INFO
    end
  end

  return update, UPDATE_INTERVAL_MS
end

return update()
