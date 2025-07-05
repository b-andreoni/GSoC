--[[--------------------------------------------------------------------
    Episodic-Reset Script
    -- Fly to a target, teleport back to the start, repeat
------------------------------------------------------------------------]]

------------------------------------------------------------------
-- PARAMETER-TABLE DEFINITIONS
------------------------------------------------------------------
local PARAM_TABLE_KEY = 90
local PARAM_TABLE_PREFIX = "SIM_ERES_"
local TABLE_LEN = 17

-- register a parameter table for this script
assert(param:add_table(PARAM_TABLE_KEY, PARAM_TABLE_PREFIX, 18), "failed to create parameter table")

-- helper: create a new parameter and bind it to a variable
function bind_add_param(name, idx, default_value)
   assert(param:add_param(PARAM_TABLE_KEY, idx, name, default_value), string.format('could not add param %s', name))
   return Parameter(PARAM_TABLE_PREFIX .. name)
end

-- user-configurable parameters (visible from the ground station)
local P_ENABLE   = bind_add_param("ENABLE",  1,  1)   -- 0 = idle, 1 = run
local P_OFST_N   = bind_add_param("OFST_N",  2, 0)    -- N offset (m)
local P_OFST_E   = bind_add_param("OFST_E",  3, 0)    -- E offset (m)
local P_OFST_D   = bind_add_param("OFST_D",  4,  0)   -- D offset (m, +down)
local P_VEL_X    = bind_add_param("VEL_X",   5,  0)   -- velocity X
local P_VEL_Y    = bind_add_param("VEL_Y",   6,  0)   -- velocity Y
local P_VEL_Z    = bind_add_param("VEL_Z",   7,  0)   -- velocity Z
local P_RLL      = bind_add_param("RLL",     8,  0)   -- roll
local P_PIT      = bind_add_param("PIT",     9,  0)   -- pitch
local P_YAW      = bind_add_param("YAW",    10,  0)   -- yaw 
local P_GX       = bind_add_param("GX",     11,  0)   -- gyro x
local P_GY       = bind_add_param("GY",     12,  0)   -- gyro y
local P_GZ       = bind_add_param("GZ",     13,  0)   -- gyro z
local P_MODE     = bind_add_param("MODE",   14, -1)   -- flight-mode override
local P_POS_LAT  = bind_add_param("POS_N",  15,  -35.3627351)   -- absolute target latitude
local P_POS_LNG  = bind_add_param("POS_E",  16,  149.1651426)   -- absolute target longitude
local P_POS_ALT  = bind_add_param("POS_D",  17,  10)   -- absolute target altitude (+m)

-- disable arming checks (SITL only)
param:set("ARMING_CHECK", 0)
param:set("GPS_TYPE", 0)
param:set("GPS_AUTO_CONFIG", 0)
param:set("GPS_AUTO_SWITCH", 0)
param:set("AHRS_EKF_TYPE", 10)

------------------------------------------------------------------
-- SCRIPT CONSTANTS
------------------------------------------------------------------
local TAKEOFF_ALT_M   = 10    -- take-off altitude (m AGL)
local TARGET_THRESH_M  = 0.2     -- arrival tolerance (m)
local LOOP_FAST       = 50    -- loop when ENABLE=1 (ms)
local LOOP_IDLE       = 500   -- loop when ENABLE=0 (ms)
local ZERO_VEC = Vector3f()
local ZERO_GYRO = Vector3f()

------------------------------------------------------------------
-- STATE VARIABLES
------------------------------------------------------------------
local state            = -1          -- -1: wait EKF | 0: arm | 1: climb | 2: cruise | 3: reset
local home_location    = nil         -- Location() at start of episode
local target_location  = nil         -- Location() to fly to
local home_attitude    = Quaternion()
local takeoff_sent     = false
local MODE_GUIDED      = 4

------------------------------------------------------------------
-- HELPER FUNCTIONS
------------------------------------------------------------------
local function ekf_ready()                 return ahrs:get_origin() end
local function current_location()          return ahrs:get_location() end
local function home_position()             return ahrs:get_home() end

local function build_location(lat_deg, lng_deg)
    local loc = Location()
    loc:lat(math.floor(lat_deg * 1e7 + 0.5))
    loc:lng(math.floor(lng_deg * 1e7 + 0.5))
    loc:alt(current_location() and current_location():alt() or 0)
    return loc
end

local function rel_altitude_m()            -- AGL in metres
    local here, home = current_location(), home_position()
    return (here and home) and math.abs(here:alt() - home:alt()) / 100 or 0
end

------------------------------------------------------------------
-- MAIN LOOP
------------------------------------------------------------------
function update()
    -- stop looping if script disabled
    if P_ENABLE:get() == 0 then return LOOP_IDLE end

    ------------------------------------------------------------------
    -- STATE -1 : wait until EKF origin & home are initialised
    ------------------------------------------------------------------
    if state == -1 then
        if ekf_ready() then
            home_location = home_position() or current_location()
            home_attitude = ahrs:get_quaternion()
            state         = 0
            takeoff_sent  = false
        end

    ------------------------------------------------------------------
    -- STATE 0 : arm motors and issue take-off
    ------------------------------------------------------------------
    elseif state == 0 then
        if vehicle:get_mode() ~= MODE_GUIDED then
            vehicle:set_mode(MODE_GUIDED)
        end
        if not arming:is_armed() then
            arming:arm()
        end
        if arming:is_armed() and not takeoff_sent then
            vehicle:start_takeoff(TAKEOFF_ALT_M)
            takeoff_sent = true
        end

        -- proceed once altitude achieved
        if rel_altitude_m() >= TAKEOFF_ALT_M - 0.5 then
            -- pick absolute or relative target
            if P_POS_LAT:get() ~= 0 and P_POS_LNG:get() ~= 0 then
                target_location = build_location(P_POS_LAT:get(), P_POS_LNG:get())
                target_location:alt(home_location:alt() + P_POS_ALT:get() * 100)
            else
                target_location = Location(home_location)
                target_location:offset(P_OFST_N:get(), P_OFST_E:get())
                target_location:alt(home_location:alt() - P_OFST_D:get() * 100)
            end
            state = 2
        end

    ------------------------------------------------------------------
    -- STATE 2 : navigate toward the target
    ------------------------------------------------------------------
    elseif state == 2 then
        vehicle:set_target_location(target_location)
        local dist = current_location():get_distance(target_location)
        if dist <= TARGET_THRESH_M then
            state = 3
        end

    ------------------------------------------------------------------
    -- STATE 3 : episodic reset → teleport back and restart
    ------------------------------------------------------------------
    elseif state == 3 then
        -- instantaneously move UAV back to home pose
        sim:set_pose(0, home_location, home_attitude, ZERO_VEC, ZERO_GYRO)
        if arming:is_armed() then
            arming:disarm()
        end
        vehicle:set_target_location(home_location)
        vehicle:set_target_velocity_NED(ZERO_VEC)

        -- prepare next cycle
        state        = 0
        takeoff_sent = false
    end

    return LOOP_INTERVAL_ACTIVE_MS
end

local function loop()
    if P_ENABLE:get() == 0 then
        return loop, 500
    end
    update()
    return loop, 50
end

gcs:send_text(0, "Loaded episodic reset script!")
return loop,1000