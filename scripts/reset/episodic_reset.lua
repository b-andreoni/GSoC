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
local P_OFST_N   = bind_add_param("OFST_N",  2, 20)   -- N offset (m)
local P_OFST_E   = bind_add_param("OFST_E",  3, 20)   -- E offset (m)
local P_OFST_D   = bind_add_param("OFST_D",  4,  -20) -- D offset (m, +down)
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
local P_POS_LAT  = bind_add_param("POS_N",  15,  0)   -- absolute target latitude
local P_POS_LNG  = bind_add_param("POS_E",  16,  0)   -- absolute target longitude
local P_POS_ALT  = bind_add_param("POS_D",  17,  0)   -- absolute target altitude (+m)
local P_STATE    = bind_add_param("STATE",  18,  -1)  -- FSM state

-- disable arming checks (SITL only)
param:set("ARMING_CHECK", 0)
param:set("GPS_TYPE", 0)
param:set("GPS_AUTO_CONFIG", 0)
param:set("GPS_AUTO_SWITCH", 0)
param:set("AHRS_EKF_TYPE", 10)

------------------------------------------------------------------
-- SCRIPT CONSTANTS
------------------------------------------------------------------
local TAKEOFF_ALT_M   = 10     -- takeoff altitude (m AGL)
local TARGET_THRESH_M = 0.01  -- arrival tolerance (m)
local LOOP_FAST       = 50     -- active loop interval (ms)
local LOOP_IDLE       = 500    -- idle loop interval (ms)

------------------------------------------------------------------
-- STATE VARIABLES
------------------------------------------------------------------
local state           = -1     -- -1: wait EKF | 0: arm+takeoff | 1: cruise | 2: reset
local home_location   = nil
local target_location = nil
local base_attitude   = Quaternion()
local takeoff_sent    = false
local MODE_GUIDED     = 4

------------------------------------------------------------------
-- HELPERS
------------------------------------------------------------------
local function ready()    return ahrs:get_origin() end
local function loc()      return ahrs:get_location() end
local function home()     return ahrs:get_home() end
local function build(lat, lon)
    local L = Location()
    L:lat(math.floor(lat * 1e7 + 0.5))
    L:lng(math.floor(lon * 1e7 + 0.5))
    L:alt(home_location:alt())
    return L
end
local function rel_alt()   return math.abs((loc():alt() - home_location:alt())/100) end
local function vec3(x,y,z) local v=Vector3f() v:x(x);v:y(y);v:z(z); return v end

------------------------------------------------------------------
-- MAIN LOOP
------------------------------------------------------------------
function update()
    if P_ENABLE:get()==0 then return LOOP_IDLE end

    if state==-1 then
        if ready() then
            home_location = home() or loc()
            base_attitude = ahrs:get_quaternion()
            state = 0
        end

    elseif state==0 then
        -- arm & takeoff
        if vehicle:get_mode()~=MODE_GUIDED then vehicle:set_mode(MODE_GUIDED) end
        if not arming:is_armed() then
            arming:arm()
        elseif not takeoff_sent then
            vehicle:start_takeoff(TAKEOFF_ALT_M)
            takeoff_sent = true
        end
        if rel_alt()>=TAKEOFF_ALT_M-0.5 then
            -- set target
            if P_POS_LAT:get()~=0 and P_POS_LNG:get()~=0 then
                target_location = build(P_POS_LAT:get(),P_POS_LNG:get())
                target_location:alt(home_location:alt()+P_POS_ALT:get()*100)
            else
                target_location = ahrs:get_location()
                target_location:offset(P_OFST_N:get(),P_OFST_E:get())
                target_location:alt(home_location:alt()-P_OFST_D:get()*100)
            end
            state=1
        end

    elseif state==1 then
        -- cruise to target
        vehicle:set_target_location(target_location)
        if loc():get_distance(target_location)<=TARGET_THRESH_M then
            state=2
        end

    elseif state==2 then
    -- increment yaw param by 45°
        local new_yaw = (P_YAW:get() or 0) + 45
        P_YAW:set(new_yaw)
        -- build combined attitude
        local yaw_rad = math.rad(new_yaw)
        local yaw_q = Quaternion()
        yaw_q:from_euler(0,0,yaw_rad)
        local att = base_attitude * yaw_q
        -- teleport reset
        sim:set_pose(0,
            home_location,
            att,
            vec3(P_VEL_X:get(),P_VEL_Y:get(),P_VEL_Z:get()),
            vec3(P_GX:get(),P_GY:get(),P_GZ:get()))
        if arming:is_armed() then arming:disarm() end
        takeoff_sent=false
        state=0
    end

    P_STATE:set(state)
    return LOOP_FAST
end

function loop()
    if P_ENABLE:get()==0 then return loop,LOOP_IDLE end
    update()
    return loop,LOOP_FAST
end

gcs:send_text(0,"Loaded episodic reset script!")
return loop,1000
