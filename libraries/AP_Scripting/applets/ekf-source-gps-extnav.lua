-- switches between AHRS/EKF sources based on the pilot's source selection switch or using an automatic source selection algorithm
-- this script is intended to help vehicles move between GPS and Non-GPS environments
--
-- setup RCx_OPTION = 90 (EKF Pos Source) to select the source (low=primary, middle=secondary, high=tertiary)
-- setup RCx_OPTION = 300 (Scripting1).  When this switch is pulled high, the source will be automatically selected
-- setup EK3_SRCn_ parameters so that GPS is the primary source, ExternalNav is secondary
--
-- When the auxiliary switch (Scripting1) is pulled high automatic source selection uses these thresholds:
-- ESRC_GPS_THRESH holds the threshold for GPS horizontal speed accuracy.  High values lead to GPS being used more easily.  Default is 0.3
-- ESRC_ENAV_THRESH holds the threshold for ExternalNav vertical speed innovation.  Higher values lead to ExternalNav being used more easily.  About 0.3 is a good choice
-- If both GPS and ExternalNav are working well (e.g. below thresholds), ExtNav will be used

-- create and initialise parameters
local PARAM_TABLE_KEY = 81  -- parameter table key must be used by only one script on a particular flight controller
assert(param:add_table(PARAM_TABLE_KEY, "ESRC_", 3), 'could not add param table')
assert(param:add_param(PARAM_TABLE_KEY, 1, 'ENABLE', 1), 'could not add ESRC_ENABLE param') -- 1 = enabled, 0 = disabled
assert(param:add_param(PARAM_TABLE_KEY, 2, 'GPS_THRESH', 0.3), 'could not add ESRC_GPS_THRESH param')   -- threshold for GPS speed accuracy
assert(param:add_param(PARAM_TABLE_KEY, 3, 'ENAV_THRESH', 0.3), 'could not add ESRC_ENAV_THRESH param') -- threshold for ExternalNav vertical speed innovations

-- bind parameters to variables
local enable = Parameter("ESRC_ENABLE")                         -- 1 = enabled, 0 = disabled
local gps_speedaccuracy_thresh = Parameter("ESRC_GPS_THRESH")   -- threshold for GPS speed accuracy
local extnav_innov_thresh = Parameter("ESRC_ENAV_THRESH")       -- threshold for ExternalNav vertical speed innovation

local source_prev = 0               -- previous source, defaults to primary source
local sw_auto_pos_prev = -1         -- previous auto source switch position
local auto_switch = false           -- true when auto switching between sources is active
local gps_usable_accuracy = 1.0     -- GPS is usable if speed accuracy is at or below this value
local vote_counter_max = 20         -- when a vote counter reaches this number (i.e. 2sec) source may be switched
local gps_vs_extnav_vote = 0        -- vote counter for GPS vs NonGPS (-20 = GPS, +20 = NonGPS)

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

-- the main update function
function update()

  -- exit immediately if not enabled
  if (enable:get() < 1) then
    return update, 1000
  end

  -- check switches are configured
  -- at least one switch must be set
  -- source selection from RCx_FUNCTION = 90 (EKF Source Select)
  -- auto source from RCx_FUNCTION = 300 (Scripting1)
  local rc_option_source = rc:find_channel_for_option(90)
  local rc_option_autosource = rc:find_channel_for_option(300)
  if (rc_option_source == nil) then
    gcs:send_text(0, "ekf-source.lua: RCx_OPTION=90 not set!")
    return update, 1000
  end
  if (rc_option_autosource == nil) then
    gcs:send_text(0, "ekf-source.lua: RCx_OPTION=300 not set!")
    return update, 1000
  end

  -- check GPS speed accuracy threshold has been set
  if (gps_speedaccuracy_thresh:get() <= 0) then
    gcs:send_text(0, "ekf-source.lua: set ESRC_GPS_THRESH to GPS speed accuracy threshold")
    return update, 1000
  end

  -- check external nav innovation threshold has been set
  if (extnav_innov_thresh:get() <= 0) then
    gcs:send_text(0, "ekf-source.lua: set ESRC_ENAV_THRESH to ExtNav innovation threshold")
    return update, 1000
  end

  -- check if GPS speed accuracy is over threshold
  local gps_speed_accuracy = gps:speed_accuracy(gps:primary_sensor())
  local gps_over_threshold = (gps_speed_accuracy == nil) or (gps:speed_accuracy(gps:primary_sensor()) > gps_speedaccuracy_thresh:get())
  local gps_usable = (gps_speed_accuracy ~= nil) and (gps_speed_accuracy <= gps_usable_accuracy)

  -- get external nav innovations from ahrs
  local extnav_innov = Vector3f()
  local extnav_var = Vector3f()
  extnav_innov, extnav_var = ahrs:get_vel_innovations_and_variances_for_source(6)
  local extnav_usable = (extnav_innov ~= nil) and (extnav_innov:z() > 0.0) and (math.abs(extnav_innov:z()) <= extnav_innov_thresh:get())

  -- automatic selection logic --

  -- GPS vs ExtNav vote. "-1" to move towards GPS, "+1" to move to ExtNav
  if (not gps_over_threshold) or (gps_usable and not extnav_usable) then
    -- vote for GPS if GPS accuracy good OR usable GPS and ExtNav unusable
    gps_vs_extnav_vote = math.max(gps_vs_extnav_vote - 1, -vote_counter_max)
  elseif extnav_usable then
    -- vote for ExtNav if usable
    gps_vs_extnav_vote = math.min(gps_vs_extnav_vote + 1, vote_counter_max)
  end

  -- auto source vote collation
  local auto_source = -1                        -- auto source undecided if -1
  if gps_vs_extnav_vote <= -vote_counter_max then
    auto_source = 0                             -- GPS
  elseif gps_vs_extnav_vote >= vote_counter_max then
    auto_source = 1                             -- ExtNav
  end

  -- read source switch position from RCx_FUNCTION = 90 (EKF Source Select)
  local sw_source_pos = rc_option_source:get_aux_switch_pos()
  if sw_source_pos ~= sw_source_pos_prev then    -- check for changes in source switch position
    sw_source_pos_prev = sw_source_pos           -- record new switch position so we can detect changes
    auto_switch = false                          -- disable auto switching of source
    if source_prev ~= sw_source_pos then         -- check if switch position does not match source (there is a one-to-one mapping of switch to source)
      source_prev = sw_source_pos                -- record what source should now be (changed by ArduPilot vehicle code)
      gcs:send_text(0, "Pilot switched to Source " .. string.format("%d", source_prev+1))
    else
      gcs:send_text(0, "Pilot switched but already Source " .. string.format("%d", source_prev+1))
    end
    play_source_tune(source_prev)                -- alert user of source whether changed or not
  end

  -- read auto source switch position from RCx_FUNCTION = 300 (Scripting1)
  if rc_option_autosource then
    local sw_auto_pos = rc_option_autosource:get_aux_switch_pos()
    if sw_auto_pos ~= sw_auto_pos_prev  then       -- check for changes in source auto switch position
      sw_auto_pos_prev = sw_auto_pos               -- record new switch position so we can detect changes
      if sw_auto_pos == 0 then                     -- pilot has pulled switch low
        auto_switch = false                        -- disable auto switching of source
        if sw_source_pos ~= source_prev then       -- check if source will change
          source_prev = sw_source_pos              -- record pilot's selected source
          ahrs:set_posvelyaw_source_set(source_prev)   -- switch to pilot's selected source
          gcs:send_text(0, "Auto source disabled, switched to Source " .. string.format("%d", source_prev+1))
        else
          gcs:send_text(0, "Auto source disabled, already Source " .. string.format("%d", source_prev+1))
        end
      elseif sw_auto_pos == 2 then                 -- pilot has pulled switch high
        auto_switch = true                         -- enable auto switching of source
        if auto_source < 0 then
          gcs:send_text(0, "Auto source enabled, undecided, Source " .. string.format("%d", source_prev+1))
        elseif auto_source ~= source_prev then     -- check if source will change
          source_prev = auto_source                -- record pilot's selected source
          ahrs:set_posvelyaw_source_set(source_prev)   -- switch to pilot's selected source
          gcs:send_text(0, "Auto source enabled, switched to Source " .. string.format("%d", source_prev+1))
        else
          gcs:send_text(0, "Auto source enabled, already Source " .. string.format("%d", source_prev+1))
        end
      end
      play_source_tune(source_prev)
    end
  end

  -- auto switching
  if auto_switch and (auto_source >= 0) and (auto_source ~= source_prev) then
    source_prev = auto_source                   -- record selected source
    ahrs:set_posvelyaw_source_set(source_prev)  -- switch to pilot's selected source
    gcs:send_text(0, "Auto switched to Source " .. string.format("%d", source_prev+1))
    play_source_tune(source_prev)
  end

  return update, 100
end

return update()
