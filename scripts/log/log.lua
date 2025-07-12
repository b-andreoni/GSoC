--[[------------------------------------------------------------------
  flight_logger.lua  ·  v1.9
  Logs telemetry (position, attitude), FSM state, armed status, battery readings, yaw behaviour
  Inserts separator on new episode start
--------------------------------------------------------------------]]

------------------------------------------------------------------
-- CONFIGURATION
------------------------------------------------------------------
local LOG_PERIOD_MS   = 100      -- Logging interval in milliseconds (10 Hz)
local FMT             = "%.7f"   -- Decimal format for latitude & longitude

-- Variables initialized on first loop execution
local log_filename    = nil
local log_file        = nil
local start_time      = nil
local header =
  "elapsed_ms,fsm_state,flight_mode,armed,"  
.. "lat_deg,lon_deg,alt_m,"  
.. "vel_n,vel_e,vel_d,"  
.. "roll,pitch,yaw,yaw_bhv,"  
.. "gyro_x,gyro_y,gyro_z,"  
.. "volt_v,curr_a\n"

------------------------------------------------------------------
-- HELPER FUNCTION FOR BATTERY
------------------------------------------------------------------
local function read_battery()
    local voltage, current = 0, 0
    if battery and battery.voltage then
        voltage = battery:voltage(0) or 0
    end
    if battery and battery.current_amps then
        current = battery:current_amps(0) or 0
    end
    return voltage, current
end

------------------------------------------------------------------
-- INTERNAL STATE VARIABLES
------------------------------------------------------------------
local last_log_time    = 0
local last_fsm_state   = -999

------------------------------------------------------------------
-- MAIN LOOP
------------------------------------------------------------------
function logger_update()
    -- Capture timestamp once per iteration
    local now = tonumber(tostring(millis())) or 0

    -- Create log file on first call
    if not log_file then
        start_time   = now
        local ts     = math.floor(now / 1000)
        log_filename = string.format("scripts/flight_log_%d.csv", ts)
        log_file     = assert(io.open(log_filename, "w"),
                              "Failed to create log file "..log_filename)
        log_file:write(header)
        log_file:flush()
        gcs:send_text(0, "Flight logger started: "..log_filename)
    end

    -- Write log entry at defined period
    if now - last_log_time >= LOG_PERIOD_MS then
        last_log_time = now
        local elapsed = now - (start_time or now)

        -- Sensor readings
        local location       = ahrs:get_location() or Location()
        local velocity       = ahrs:get_velocity_NED() or Vector3f()
        local gyro           = ahrs:get_gyro() or Vector3f()
        local fsm_state      = param:get("SIM_ERES_STATE") or -1
        local armed          = arming:is_armed() and 1 or 0
        local yaw_behavior  = param:get("WP_YAW_BEHAVIOR") or 0

        -- Insert separator when returning to state 0
        if fsm_state == 0 and last_fsm_state ~= 0 and last_fsm_state ~= -1 then
            log_file:write("RESET\n")
        end

        -- Euler angles (degrees)
        local roll     = math.deg(ahrs:get_roll_rad() or 0)
        local pitch    = math.deg(ahrs:get_pitch_rad() or 0)
        local yaw      = math.deg(ahrs:get_yaw_rad() or 0)

        -- Notify on state change
        if fsm_state ~= last_fsm_state then
            gcs:send_text(6, string.format("FSM_STATE -> %d", fsm_state))
            last_fsm_state = fsm_state
        end

        -- Battery data
        local volt_v, curr_a = read_battery()

        -- Write CSV line
        log_file:write(string.format(
            "%d,%d,%d,%d," ..            -- elapsed,fsm_state,mode,armed
            FMT..","..FMT..",%.2f," ..  -- lat,lon,alt
            "%.2f,%.2f,%.2f," ..           -- vel_n,e,d
            "%.2f,%.2f,%.2f,%d," ..        -- roll,pitch,yaw,yaw_behavior
            "%.3f,%.3f,%.3f," ..           -- gyro_x,y,z
            "%.2f,%.2f\n",              -- volt_v,curr_a
            elapsed,
            fsm_state,
            vehicle:get_mode(),
            armed,
            location:lat()/1e7,
            location:lng()/1e7,
            location:alt()/100,
            velocity:x(), velocity:y(), velocity:z(),
            roll, pitch, yaw,
            yaw_behavior,
            gyro:x(), gyro:y(), gyro:z(),
            volt_v, curr_a))
        log_file:flush()
    end

    return logger_update, 50  -- schedule next update in 50 ms
end

return logger_update, 100
