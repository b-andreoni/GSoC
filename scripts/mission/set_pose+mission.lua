--[[
    Allow forcing the simulated vehicle's position when the script is enabled,
    then wait ~10 seconds, arm the vehicle, and start the mission, ensuring that
    AUTO mode is actually entered and the mission is loaded.
--]]

local PARAM_TABLE_KEY    = 16
local PARAM_TABLE_PREFIX = "SIM_APOS_"
local takeoff_alt = 5  -- takeoff altitude in meters

-- Mode numbers for the Copter (as per ArduPilot)
local copter_guided_mode_num = 4
local copter_auto_mode_num   = 3
local copter_land_mode_num   = 9

-- How many loop iterations (~50 ms each) to wait before arming
-- 12.5 seconds / 0.05 s ≈ 250 iterations
local WAIT_BEFORE_ARM = 250

-- How many iterations to wait after AUTO is confirmed
-- to give time for EKF/GPS/IMU to stabilize and for takeoff to start
local WAIT_AFTER_AUTO = 60  -- 60 × 50ms ≈ 3 s

-- Set up parameters (only once)
assert(param:add_table(PARAM_TABLE_KEY, PARAM_TABLE_PREFIX, 16),
       "could not add parameter table")

-- Helper function to create a parameter
local function bind_add_param(name, idx, default_value)
    assert(param:add_param(PARAM_TABLE_KEY, idx, name, default_value),
           string.format("could not add parameter %s", name))
    return Parameter(PARAM_TABLE_PREFIX .. name)
end

-- Parameters to enable and control the script
local SIM_APOS_ENABLE = bind_add_param("ENABLE",  1, 0)
local SIM_APOS_POS_N  = bind_add_param("POS_N",  2, 0)
local SIM_APOS_POS_E  = bind_add_param("POS_E",  3, 0)
local SIM_APOS_POS_D  = bind_add_param("POS_D",  4, 0)
local SIM_APOS_VEL_X  = bind_add_param("VEL_X",  5, 0)
local SIM_APOS_VEL_Y  = bind_add_param("VEL_Y",  6, 0)
local SIM_APOS_VEL_Z  = bind_add_param("VEL_Z",  7, 0)
local SIM_APOS_RLL    = bind_add_param("RLL",    8, 0)
local SIM_APOS_PIT    = bind_add_param("PIT",    9, 0)
local SIM_APOS_YAW    = bind_add_param("YAW",   10, 0)
local SIM_APOS_GX     = bind_add_param("GX",    11, 0)
local SIM_APOS_GY     = bind_add_param("GY",    12, 0)
local SIM_APOS_GZ     = bind_add_param("GZ",    13, 0)
local SIM_APOS_MODE   = bind_add_param("MODE",  14, -1)

-- State flags
local first_run  = true   -- to distinguish initial pose forcing
local phase      = 0      -- 0 = not started, 1 = waiting to arm, 2 = waiting for AUTO confirm
local wait_count = 0

-- Main function that runs every 50 ms
local function update()
    if SIM_APOS_ENABLE:get() == 0 then
        return
    end
    local armed = arming:is_armed()
    -- If this is the very first execution after ENABLE, force pose immediately
    if first_run and armed then
        first_run = false
        gcs:send_text(0, "Forcing pose as soon as script is enabled")

        -- Build quaternion from Euler angles
        local quat = Quaternion()
        quat:from_euler(
            math.rad(SIM_APOS_RLL:get()),
            math.rad(SIM_APOS_PIT:get()),
            math.rad(SIM_APOS_YAW:get())
        )

        -- Build body-frame velocity vector
        local vel = Vector3f()
        vel:x(SIM_APOS_VEL_X:get())
        vel:y(SIM_APOS_VEL_Y:get())
        vel:z(SIM_APOS_VEL_Z:get())
        quat:earth_to_body(vel)

        -- Create and set a fixed location
        local loc = Location()
        if not loc then
            return
        end
        loc:lat(math.floor(-23.55604194 * 1e7 + 0.5))
        loc:lng(math.floor(-46.73024763 * 1e7 + 0.5))
        loc:alt(0)

        -- Build gyro vector (radians)
        local gyro = Vector3f()
        gyro:x(math.rad(SIM_APOS_GX:get()))
        gyro:y(math.rad(SIM_APOS_GY:get()))
        gyro:z(math.rad(SIM_APOS_GZ:get()))

        -- Set the simulated pose
        sim:set_pose(0, loc, quat, vel, gyro)

        -- If a MODE parameter is set (>= 0), switch to that mode immediately
        if SIM_APOS_MODE:get() >= 0 then
            vehicle:set_mode(SIM_APOS_MODE:get())
        end

        -- Start the 10-second timer to arm and initiate the mission
        gcs:send_text(6, "Pose forced → starting 10-second timer to arm and start mission")
        phase = 1
        wait_count = 0
        return
    end

    ----------------------------------------------------------------
    -- Phase 1: wait WAIT_BEFORE_ARM iterations (~10 seconds), then arm and request GUIDED/AUTO
    if phase == 1 then
        wait_count = wait_count + 1

        if wait_count == 1 then
            -- Debug: show how many commands are in the mission
            local mcount = mission:num_commands() or 0
            gcs:send_text(6, "Mission has " .. tostring(mcount) .. " commands")
        end

        if wait_count >= WAIT_BEFORE_ARM then
            -- 1) Arm the vehicle
            gcs:send_text(6, "Requesting ARM now after ~10 seconds")
            arming:arm()
            if arming:is_armed() then
                gcs:send_text(6, "ARM command sent successfully")
            else
                gcs:send_text(3, "Failed to send ARM command; retrying")
                wait_count = WAIT_BEFORE_ARM - 1  -- retry next loop
                return
            end

            -- 2) Switch to GUIDED mode and request takeoff
            gcs:send_text(6, "Attempting to switch to GUIDED mode")
            local ok_guided = vehicle:set_mode(copter_guided_mode_num)
            if ok_guided then
                gcs:send_text(6, "GUIDED mode requested → requesting takeoff")
                if vehicle:start_takeoff(takeoff_alt) then
                    gcs:send_text(6, "Takeoff command sent → awaiting stabilization")
                else
                    gcs:send_text(3, "Failed to request takeoff; retrying")
                end
            else
                gcs:send_text(3, "Failed to request GUIDED mode; retrying")
            end

            -- 3) Switch to AUTO mode
            gcs:send_text(6, "Attempting to switch to AUTO mode")
            local ok_auto = vehicle:set_mode(copter_auto_mode_num)
            if ok_auto then
                gcs:send_text(6, "AUTO mode requested → entering phase 2 (waiting for AUTO confirmation)")
                phase = 2
                wait_count = 0
            else
                gcs:send_text(3, "Failed to request AUTO mode; retrying")
                wait_count = WAIT_BEFORE_ARM - 1  -- retry next loop
            end
        else
            -- Optional halfway debug
            if wait_count == math.floor(WAIT_BEFORE_ARM / 2) then
                gcs:send_text(6, string.format(
                    "Waiting to arm (%d/%d)", wait_count, WAIT_BEFORE_ARM))
            end
        end

        return
    end

    ----------------------------------------------------------------
    -- Phase 2: wait for the autopilot to confirm AUTO mode, then wait WAIT_AFTER_AUTO, then start mission
    if phase == 2 then
        local current_mode = vehicle:get_mode()
        if current_mode == copter_auto_mode_num then
            wait_count = wait_count + 1

            if wait_count == 1 then
                gcs:send_text(6, "AUTO confirmed! Waiting " .. WAIT_AFTER_AUTO .. " iterations to stabilize")
            end

            if wait_count >= WAIT_AFTER_AUTO then
                -- Time to set mission to first command
                gcs:send_text(6, "Starting mission → mission:set_current_cmd(0)")
                mission:set_current_cmd(0)
                phase = 0
                wait_count = 0
            elseif wait_count == math.floor(WAIT_AFTER_AUTO / 2) then
                gcs:send_text(6, string.format(
                    "Still waiting to stabilize in AUTO (%d/%d)", wait_count, WAIT_AFTER_AUTO))
            end
        else
            -- Not yet in AUTO mode; reset counter and keep checking
            wait_count = 0
            gcs:send_text(6, "Waiting for AUTO mode confirmation...")
        end

        return
    end
end

-- Loop function: runs update() every 50 ms if ENABLE=1, otherwise every 500 ms
local function loop()
    if SIM_APOS_ENABLE:get() == 0 then
        return loop, 500
    end
    update()
    return loop, 50
end

gcs:send_text(0, "Loaded arm pose with 10s timer to arm and start mission")
return loop, 1000
