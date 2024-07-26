-- Lua script retrieve the position and velocity of a slung payload
--
-- How To Use
-- 1. copy this script to the autopilot's "scripts" directory
-- 2. within the "scripts" directory create a "modules" directory
-- 3. copy the MAVLink/mavlink_msgs_xxx files to the "scripts" directory
--

-- load mavlink message definitions from modules/MAVLink directory
local mavlink_msgs = require("MAVLink/mavlink_msgs")

-- global definitions
local MAV_SEVERITY = {EMERGENCY=0, ALERT=1, CRITICAL=2, ERROR=3, WARNING=4, NOTICE=5, INFO=6, DEBUG=7}
local UPDATE_INTERVAL_MS = 100

 -- setup script specific parameters
local PARAM_TABLE_KEY = 82
local PARAM_TABLE_PREFIX = "SLUP_"
assert(param:add_table(PARAM_TABLE_KEY, PARAM_TABLE_PREFIX, 4), 'could not add param table')

-- add a parameter and bind it to a variable
function bind_add_param(name, idx, default_value)
    assert(param:add_param(PARAM_TABLE_KEY, idx, name, default_value), string.format('could not add param %s', PARAM_TABLE_PREFIX .. name))
    return Parameter(PARAM_TABLE_PREFIX .. name)
 end

--[[
  // @Param: SLUP_ENABLE
  // @DisplayName: Slung Payload enable
  // @Description: Slung Payload enable
  // @Values: 0:Disabled,1:Enabled
  // @User: Standard
--]]
local SLUP_ENABLE = bind_add_param("ENABLE", 1, 1)

--[[
  // @Param: SLUP_POS_P
  // @DisplayName: Slung Payload Position P gain
  // @Description: Slung Payload Position P gain, higher values will result in faster movement towards the payload
  // @Range: 0 5
  // @User: Standard
--]]
local SLUP_POS_P = bind_add_param("POS_P", 2, 0.2)

--[[
  // @Param: SLUP_DIST_MAX
  // @DisplayName: Slung Payload horizontal distance max
  // @Description: Oscillation is suppressed when vehicle and payload are no more than this distance horizontally.  Set to 0 to always suppress
  // @Range: 0 30
  // @User: Standard
--]]
local SLUP_DIST_MAX = bind_add_param("DIST_MAX", 3, 15)

--[[
  // @Param: SLUP_SYSID
  // @DisplayName: Slung Payload mavlink system id
  // @Description: Slung Payload mavlink system id.  0 to use any/all system ids
  // @Range: 0 255
  // @User: Standard
--]]
local SLUP_SYSID = bind_add_param("SYSID", 4, 0)

-- mavlink definitions
local HEARTBEAT_ID = 0
local GLOBAL_POSITION_INT_ID = 33
local msg_map = {}
msg_map[HEARTBEAT_ID] = "HEARTBEAT"
msg_map[GLOBAL_POSITION_INT_ID] = "GLOBAL_POSITION_INT"

-- initialize MAVLink rx with number of messages, and buffer depth
mavlink:init(1, 10)

-- register message id to receive
mavlink:register_rx_msgid(HEARTBEAT_ID)
mavlink:register_rx_msgid(GLOBAL_POSITION_INT_ID)

-- variables to calculate payload's resting location
local velx_dir_last = 0 -- North/South velocity direction (+ve = North, -ve = South)
local vely_dir_last = 0 -- East/West velocity direction (+ve = East, -ve = West)
local lat_total = 0     -- sum of latest latitudes
local lat_count = 0     -- number of readings included in lat_total
local lon_total = 0     -- sum of latest longitudes
local lon_count = 0     -- number of readings included in lon_total
local resting_lat = 0   -- estimated resting latitude
local resting_lon = 0   -- estimated resting longitude
local found_heartbeat = false       -- true if a heartbeat message has been received
local found_payload_sysid = false   -- true if a global position int message has been received

-- handle heartbeat message
function handle_heartbeat(msg)
    if not found_heartbeat then
      found_heartbeat = true
      gcs:send_text(MAV_SEVERITY.INFO, string.format("slung-payload: first heartbeat sysid:%d", msg.sysid))
    end
end

-- handle global position int message
function handle_global_position_int(msg)
    -- check if message is from the correct system id
    if (SLUP_SYSID:get() > 0 and msg.sysid ~= SLUP_SYSID:get()) then
        return false
    end
    if not found_payload_sysid then
        found_payload_sysid = true
        gcs:send_text(MAV_SEVERITY.INFO, string.format("slung-payload: found sysid:%d", msg.sysid))
    end

    --gcs:send_text(MAV_SEVERITY.INFO, string.format("global position int sysid:%d", msg.sysid))
    --local sp_lat = msg.lat / 1.0e7
    --local sp_lon = msg.lon / 1.0e7
    --local sp_alt = msg.alt / 1.0e3
    --gcs:send_text(MAV_SEVERITY.INFO, string.format("GPI lat:%f lon:%f alt:%f", sp_lat, sp_lon, sp_alt))

    -- get payload location
    local payload_loc = Location()
    payload_loc:lat(msg.lat)
    payload_loc:lng(msg.lon)
    payload_loc:alt(msg.alt * 0.1)

    -- get payload velocity
    local payload_vel = Vector3f()
    payload_vel:x(msg.vx * 0.01)
    payload_vel:y(msg.vy * 0.01)
    payload_vel:z(msg.vz * 0.01)

    -- calculate payload's resting location
    calc_payload_resting_loc(payload_loc, payload_vel)

    -- calculate position difference vs vehicle
    local curr_loc = ahrs:get_location()
    if curr_loc == nil then
        gcs:send_text(MAV_SEVERITY.WARNING, "slung-payload: failed to get vehicle location")
        return false
    end
    local dist_NED = curr_loc:get_distance_NED(payload_loc)

    -- check horizontal distance is less than SLUP_DIST_MAX
    if SLUP_DIST_MAX:get() > 0 then
        local dist_xy = dist_NED:xy():length()
        if (dist_xy > SLUP_DIST_MAX:get()) then
            gcs:send_text(MAV_SEVERITY.WARNING, string.format("slung-payload: payload too far %4.1fm", dist_xy))
            return false
        end
    end

    -- calculate and send desired velocity
    local target_vel = Vector3f()
    target_vel:x(dist_NED:x() * SLUP_POS_P:get())
    target_vel:y(dist_NED:y() * SLUP_POS_P:get())
    if not (vehicle:set_target_velocity_NED(target_vel)) then
      -- ignore errors
      --gcs:send_text(0, "failed to execute velocity command")
    end

    --gcs:send_text(MAV_SEVERITY.INFO, string.format("PL N:%f E:%f D:%f", dist_NED:x(), dist_NED:y(), dist_NED:z()))
    --gcs:send_text(MAV_SEVERITY.INFO, string.format("PLV N:%f E:%f D:%f", payload_vel:x(), payload_vel:y(), payload_vel:z()))
    return true
end

-- estimate the payload's resting location based on its current location and velocity
function calc_payload_resting_loc(loc, vel_NED)

    -- calculate velocity directions
    local velx_dir = 0
    local vely_dir = 0
    if (vel_NED:x() < 0) then
        velx_dir = -1
    else
        velx_dir = 1
    end
    if (vel_NED:y() < 0) then
        vely_dir = -1
    else
        vely_dir = 1
    end

    -- check for change in velocity direction
    local update_user = false
    if (velx_dir ~= velx_dir_last) then
        velx_dir_last = velx_dir
        -- update resting latitude
        if (lat_count > 0) then
            resting_lat = lat_total / lat_count
            update_user = true
        end
        lat_total = 0
        lat_count = 0
    end
    if (vely_dir ~= vely_dir_last) then
        vely_dir_last = vely_dir
        -- update resting longitude
        if (lon_count > 0) then
            resting_lon = lon_total / lon_count
            update_user = true
        end
        lon_total = 0
        lon_count = 0
    end

    -- add current location to total
    lat_total = lat_total + loc:lat() * 1.0e-7
    lat_count = lat_count + 1
    lon_total = lon_total + loc:lng() * 1.0e-7
    lon_count = lon_count + 1

    -- update user
    if (update_user) then
        gcs:send_text(MAV_SEVERITY.INFO, string.format("resting lat:%f lon:%f", resting_lat, resting_lon))
    end
end

-- display welcome message
gcs:send_text(MAV_SEVERITY.INFO, "slung-payload script loaded")

-- update function to receive location from payload and move vehicle to reduce payload's oscillation
function update()

    -- exit immediately if not enabled
    if (SLUP_ENABLE:get() <= 0) then
        return update, 1000
    end

    -- consume mavlink messages from payload
    local msg, _ = mavlink:receive_chan()
    if (msg ~= nil) then
        local parsed_msg = mavlink_msgs.decode(msg, msg_map)
        if (parsed_msg ~= nil) then
            if parsed_msg.msgid == HEARTBEAT_ID then
                handle_heartbeat(parsed_msg)
            end
            if parsed_msg.msgid == GLOBAL_POSITION_INT_ID then
                handle_global_position_int(parsed_msg)
            end
        end
    end

    -- if vehicle is in GUIDED mode and payload's location has been received recently
    -- run controller to calculate vehicle's desired velocity
    -- apply acceleration limit to the desired velocity
    -- send desired velocity to vehicle

    return update, UPDATE_INTERVAL_MS
end

return update()
