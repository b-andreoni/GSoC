--[[--------------------------------------------------------------------
    RL Episodic-Reset Script with Energy Logging
    -- Fly through 5 waypoints using Q-learning,
       log per-segment consumption and track best paths.
------------------------------------------------------------------------]]

------------------------------------------------------------------
-- PARAMETER-TABLE DEFINITIONS
------------------------------------------------------------------
local PARAM_TABLE_KEY    = 90
local PARAM_TABLE_PREFIX = "SIM_ERES_"
local TABLE_LEN          = 18

assert(param:add_table(PARAM_TABLE_KEY, PARAM_TABLE_PREFIX, TABLE_LEN),
       "failed to create parameter table")

-- Function to bind parameters
function bind_add_param(name, idx, default_value)
    assert(param:add_param(PARAM_TABLE_KEY, idx, name, default_value),
           string.format('could not add param %s', name))
    return Parameter(PARAM_TABLE_PREFIX .. name)
end

-- user-configurable parameters
local P_ENABLE   = bind_add_param("ENABLE",  1,  1)   -- 0 = idle, 1 = run
local P_OFST_N   = bind_add_param("OFST_N",  2, 20)   -- N offset (m)
local P_OFST_E   = bind_add_param("OFST_E",  3, 20)   -- E offset (m)
local P_OFST_D   = bind_add_param("OFST_D",  4, -20)  -- D offset (m, +down)
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
local P_STATE    = bind_add_param("STATE",  18, -1)   -- FSM state

-- Disable arming checks (SITL only)
param:set("ARMING_CHECK",     0)
param:set("GPS_TYPE",         0)
param:set("GPS_AUTO_CONFIG",  0)
param:set("GPS_AUTO_SWITCH",  0)
param:set("AHRS_EKF_TYPE",   10)

-- SIM SPEEDUP
param:set("SIM_SPEEDUP",   1000)

------------------------------------------------------------------
-- SCRIPT CONSTANTS
------------------------------------------------------------------
local TAKEOFF_ALT_M   = 10     -- takeoff altitude (m AGL)
local TARGET_THRESH_M = 0.5    -- arrival tolerance (m)
local LOOP_FAST       = 50     -- active loop interval (ms)
local LOOP_IDLE       = 500    -- idle loop interval (ms)

------------------------------------------------------------------
-- FSM STATES
------------------------------------------------------------------
local STATE_WAIT      = -1
local STATE_ARM_TAKE  =  0
local STATE_CRUISE    =  1
local STATE_LEARN     =  2
local STATE_RESET     =  3

------------------------------------------------------------------
-- STATE VARIABLES
------------------------------------------------------------------
local state        = STATE_WAIT
local home_loc     = nil
local base_att     = Quaternion()
local takeoff_sent = false

------------------------------------------------------------------
-- RL CONFIGURATION
------------------------------------------------------------------
local ALPHA           = 0.1
local GAMMA           = 0.9
local INITIAL_EPSILON = 0.2
local EPSILON_DECAY   = 0.98
local MIN_EPSILON     = 0.01
local epsilon         = INITIAL_EPSILON
local stableCount = 0

local STABLE_THRESHOLD = 50    -- number of episodes without best path to stop
local stableCount      = 0     -- counts episodes without improvement
local stopScript       = false -- flag to stop the script


-- Define 5 waypoints as N/E/D offsets (meters)
local wp_offsets = {
    {N= 20, E=  0, D=-20},
    {N=  0, E= 20, D=-20},
    {N=-20, E=  0, D=-20},
    {N=  0, E=-20, D=-20},
    {N= 20, E= 20, D=-20},
}
local numWP     = #wp_offsets
local waypoints = {}    -- will hold absolute Locations
local Q         = {}    -- Q-table

------------------------------------------------------------------
-- LOGGING VARIABLES
------------------------------------------------------------------
local epCount     = 1
local episodePath = {}
local bestEnergy  = math.huge
local bestPath    = {}
local bestStr = "a"
local bestEP = 0
local totalEnergy = 0    -- energy accumulator for entire episode

------------------------------------------------------------------
-- HELPERS
------------------------------------------------------------------
-- Encode a (mask,idx) pair into a unique key
local function stateKey(mask, idx)
    return mask * 10 + idx
end

-- Build an absolute Location from home + offset
local function build_wp(home, off)
    local L = Location()
    L:lat(home:lat())
    L:lng(home:lng())
    L:offset(off.N, off.E)
    L:alt(home:alt() - off.D * 100)  -- off.D in m (+down), alt in cm
    return L
end

-- Get max Q for future actions
local function maxFutureQ(mask)
    local best = 0
    for i=1,numWP do
        if (mask & (1 << (i-1))) == 0 then
            local key = stateKey(mask, i)
            local v   = (Q[key] and Q[key][i]) or 0
            if v > best then best = v end
        end
    end
    return best
end

-- ε-greedy action selection
local function chooseNext(mask)
    local cands = {}
    for i=1,numWP do
        if (mask & (1 << (i-1))) == 0 then
            table.insert(cands, i)
        end
    end
    if #cands == 0 then return nil end
    if math.random() < epsilon then
        return cands[math.random(#cands)]
    else
        local bestIdx, bestVal = cands[1], -1e9
        for _,i in ipairs(cands) do
            local key = stateKey(mask, i)
            local v   = (Q[key] and Q[key][i]) or 0
            if v > bestVal then bestVal, bestIdx = v, i end
        end
        return bestIdx
    end
end

-- Relative altitude above home in meters
local function rel_alt()
    return math.abs((ahrs:get_location():alt() - home_loc:alt()) / 100)
end

-- Vector helper
local function vec3(x,y,z)
    local v = Vector3f()
    v:x(x); v:y(y); v:z(z)
    return v
end

-- Battery shortcut
local batt = battery

------------------------------------------------------------------
-- MAIN UPDATE LOOP
------------------------------------------------------------------
function update()
    if P_ENABLE:get()==0 then
        return LOOP_IDLE
    end

    if state==STATE_WAIT then
        if ahrs:get_origin() then
            home_loc = ahrs:get_home() or ahrs:get_location()
            base_att = ahrs:get_quaternion()
            -- Build absolute waypoints
            for i,off in ipairs(wp_offsets) do
                waypoints[i] = build_wp(home_loc, off)
            end
            -- Initialize an episode
            visitedMask  = 0
            currentWP    = chooseNext(visitedMask)
            accumEnergy  = 0
            totalEnergy  = 0
            episodePath  = {}
            state        = STATE_ARM_TAKE
        end

    elseif state==STATE_ARM_TAKE then
        if vehicle:get_mode()~=4 then vehicle:set_mode(4) end
        if not arming:is_armed() then
            arming:arm()
        elseif not takeoff_sent then
            vehicle:start_takeoff(TAKEOFF_ALT_M)
            takeoff_sent = true
        elseif rel_alt() >= TAKEOFF_ALT_M - 0.5 then
            vehicle:set_target_location(waypoints[currentWP])
            state = STATE_CRUISE
        end

    elseif state==STATE_CRUISE then
        -- Accumulate energy consumption
        local v      = batt:voltage(0)
        local c      = batt:current_amps(0)
        local deltaE = v * c * (LOOP_FAST/1000)
        accumEnergy  = accumEnergy + deltaE
        totalEnergy  = totalEnergy + deltaE
        -- Check if target waypoint is reached
        if ahrs:get_location():get_distance(waypoints[currentWP]) <= TARGET_THRESH_M then
            state = STATE_LEARN
        end

    elseif state==STATE_LEARN then
        -- Record visit
        table.insert(episodePath, currentWP)
        -- Q-learning update
        local m       = 1 << (currentWP-1)
        local newMask = visitedMask | m
        local oldKey  = stateKey(visitedMask, currentWP)
        local oldQ    = (Q[oldKey] and Q[oldKey][currentWP]) or 0
        local nextWP  = chooseNext(newMask)
        local reward  = -accumEnergy
        local futureQ = nextWP and maxFutureQ(newMask) or 0
        local newQ    = oldQ + ALPHA * (reward + GAMMA * futureQ - oldQ)
        Q[oldKey]     = Q[oldKey] or {}
        Q[oldKey][currentWP] = newQ
        gcs:send_text(0, string.format(
            "EP %d: Learn Q[%d,%d]=%.3f, nextWP=%s",
            epCount, visitedMask, currentWP, newQ, tostring(nextWP or "nil")))
        -- Decay epsilon
        epsilon = math.max(MIN_EPSILON, epsilon * EPSILON_DECAY)
        -- Prepare next
        visitedMask = newMask
        currentWP   = nextWP
        accumEnergy = 0
        if currentWP then
            vehicle:set_target_location(waypoints[currentWP])
            state = STATE_CRUISE
        else
            state = STATE_RESET
        end

    elseif state==STATE_RESET then
        -- Episode summary
        local pathStr = table.concat(episodePath, " -> ")
        gcs:send_text(0, string.format(
            "EP %d END path: %s total_E=%.3f J",
            epCount, pathStr, totalEnergy))
            
        if totalEnergy < bestEnergy then
            bestEnergy = totalEnergy
            bestPath   = {table.unpack(episodePath)}
            bestEP = epCount
            bestStr = table.concat(bestPath, " -> ")
            stableCount = 0
        else
            stableCount = stableCount + 1
        end
        gcs:send_text(0, string.format(
            "EP %d [BEST] path: %s E=%.3f J",
            bestEP, bestStr, bestEnergy))
        
        if stableCount >= STABLE_THRESHOLD then
            stopScript = true
            gcs:send_text(0, string.format("Best path stabilized after %d episodes. Stopping script.", epCount))
            
        epCount = epCount + 1
        end
        -- Teleport reset
        sim:set_pose(0,
            home_loc,
            base_att,
            vec3(0,0,0),
            vec3(0,0,0))
        if arming:is_armed() then arming:disarm() end
        takeoff_sent = false
        state        = STATE_WAIT
    end

    P_STATE:set(state)
    return (state==STATE_WAIT and LOOP_IDLE) or LOOP_FAST
end

function loop()
    if stopScript then
        return nil
    end

    if P_ENABLE:get()==0 then
        return loop, LOOP_IDLE
    end

    update()
    return loop, LOOP_FAST
end

gcs:send_text(0, "Loaded RL waypoints path script!")
return loop, 1000
