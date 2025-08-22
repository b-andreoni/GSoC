--[[--------------------------------------------------------------------
    RL Episodic-Reset Script with Energy Logging
    -- Fly through 50 fixed waypoints using Q-learning,
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
param:set("RTL_TYPE", 0) -- Disable SmartRTL to save memory

-- SIM SPEEDUP
param:set("SIM_SPEEDUP",   10000)

param:set("LOG_BACKEND_TYPE",  0)

------------------------------------------------------------------
-- SCRIPT CONSTANTS
------------------------------------------------------------------
local TAKEOFF_ALT_M   = 10     -- takeoff altitude (m AGL)
local TARGET_THRESH_M = 0.5    -- arrival tolerance (m)
local LOOP_FAST       = 50     -- active loop interval (ms)
local LOOP_IDLE       = 500    -- idle loop interval (ms)
local MAX_PATH_DISPLAY = 20    -- Max waypoints to show in GCS message before truncating

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
local log_file     = nil -- Handle for the log file

------------------------------------------------------------------
-- RL CONFIGURATION
------------------------------------------------------------------
local ALPHA           = 0.1
local GAMMA           = 0.9
local INITIAL_EPSILON = 0.3
local EPSILON_DECAY   = 0.99
local MIN_EPSILON     = 0.01
local epsilon         = INITIAL_EPSILON

local STABLE_THRESHOLD = 2000
local stableCount      = 0
local stopScript       = false


-- Define 50 waypoints in concentric rings
local wp_offsets = {
    -- 10 waypoints in a 200m radius
    {N=200, E=0, D=-20}, {N=162, E=118, D=-20}, {N=62, E=190, D=-20}, {N=-62, E=190, D=-20},
    {N=-162, E=118, D=-20}, {N=-200, E=0, D=-20}, {N=-162, E=-118, D=-20}, {N=-62, E=-190, D=-20},
    {N=62, E=-190, D=-20}, {N=162, E=-118, D=-20},

    -- 10 waypoints in a 100m radius
    {N=95, E=31, D=-20}, {N=59, E=81, D=-20}, {N=0, E=100, D=-20}, {N=-59, E=81, D=-20},
    {N=-95, E=31, D=-20}, {N=-95, E=-31, D=-20}, {N=-59, E=-81, D=-20}, {N=0, E=-100, D=-20},
    {N=59, E=-81, D=-20}, {N=95, E=-31, D=-20},

    -- 10 waypoints in a 75m radius
    {N=75, E=0, D=-20}, {N=61, E=44, D=-20}, {N=23, E=71, D=-20}, {N=-23, E=71, D=-20},
    {N=-61, E=44, D=-20}, {N=-75, E=0, D=-20}, {N=-61, E=-44, D=-20}, {N=-23, E=-71, D=-20},
    {N=23, E=-71, D=-20}, {N=61, E=-44, D=-20},

    -- 10 waypoints in a 50m radius
    {N=48, E=15, D=-20}, {N=29, E=40, D=-20}, {N=0, E=50, D=-20}, {N=-29, E=40, D=-20},
    {N=-48, E=15, D=-20}, {N=-48, E=-15, D=-20}, {N=-29, E=-40, D=-20}, {N=0, E=-50, D=-20},
    {N=29, E=-40, D=-20}, {N=48, E=-15, D=-20},

    -- 10 waypoints in a 25m radius
    {N=25, E=0, D=-20}, {N=20, E=15, D=-20}, {N=8, E=24, D=-20}, {N=-8, E=24, D=-20},
    {N=-20, E=15, D=-20}, {N=-25, E=0, D=-20}, {N=-20, E=-15, D=-20}, {N=-8, E=-24, D=-20},
    {N=8, E=-24, D=-20}, {N=20, E=-15, D=-20}
}
local numWP     = #wp_offsets
local waypoints = {}
local Q         = {}

------------------------------------------------------------------
-- LOGGING VARIABLES
------------------------------------------------------------------
local epCount     = 1
local episodePath = {}
local bestEnergy  = math.huge
local bestPath    = {}
local bestStr     = ""
local bestEP      = 0
local totalEnergy = 0

------------------------------------------------------------------
-- HELPERS
------------------------------------------------------------------

-- Creates a summarized path string for display if it's too long
function get_display_path(path_table)
    local count = #path_table
    if count > MAX_PATH_DISPLAY then
        local head = {}
        local tail = {}
        local half = math.floor(MAX_PATH_DISPLAY / 2)
        for i = 1, half do
            table.insert(head, path_table[i])
        end
        for i = count - half + 1, count do
            table.insert(tail, path_table[i])
        end
        return table.concat(head, " -> ") .. " -> ... -> " .. table.concat(tail, " -> ")
    else
        return table.concat(path_table, " -> ")
    end
end

-- Generate a canonical (sorted) string key from a table of visited waypoints.
local function get_visited_key(visited_set)
    local keys = {}
    for k in pairs(visited_set) do
        table.insert(keys, k)
    end
    table.sort(keys)
    return table.concat(keys, "-")
end

-- Build an absolute Location from home + offset
local function build_wp(home, off)
    local L = Location()
    L:lat(home:lat())
    L:lng(home:lng())
    L:offset(off.N, off.E)
    L:alt(home:alt() - off.D * 100)
    return L
end

-- Get max Q-value for a given state (represented by visited_set and its key)
local function maxFutureQ(visited_set, visited_key)
    local best = -1e9
    for i=1,numWP do
        if not visited_set[i] then
            local q_val = (Q[visited_key] and Q[visited_key][i]) or 0
            if q_val > best then best = q_val end
        end
    end
    if best == -1e9 then return 0 end
    return best
end

-- ε-greedy action selection for a given state
local function chooseNext(visited_set, visited_key)
    local cands = {}
    for i=1,numWP do
        if not visited_set[i] then
            table.insert(cands, i)
        end
    end

    if #cands == 0 then return nil end

    if math.random() < epsilon then
        return cands[math.random(#cands)]
    else
        local bestIdx, bestVal = cands[1], -1e9
        for _,i in ipairs(cands) do
            local q_val = (Q[visited_key] and Q[visited_key][i]) or 0
            if q_val > bestVal then bestVal, bestIdx = q_val, i end
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

            if not log_file then
                log_file = io.open("logs/rl_path_log.txt", "w")
                if log_file then
                    log_file:write("EP,TotalEnergy,Path\n")
                    gcs:send_text(0, "Log file opened: logs/rl_path_log.txt")
                else
                    gcs:send_text(0, "ERROR: Failed to open log file!")
                end
            end

            if #waypoints == 0 then
                for i,off in ipairs(wp_offsets) do
                    waypoints[i] = build_wp(home_loc, off)
                end
            end
            
            -- Initialize an episode
            visitedSet  = {}
            visitedKey = ""
            currentWP    = chooseNext(visitedSet, visitedKey)
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
        local v      = batt:voltage(0)
        local c      = batt:current_amps(0)
        local deltaE = v * c * (LOOP_FAST/1000)
        accumEnergy  = accumEnergy + deltaE
        totalEnergy  = totalEnergy + deltaE
        if ahrs:get_location():get_distance(waypoints[currentWP]) <= TARGET_THRESH_M then
            state = STATE_LEARN
        end

    elseif state==STATE_LEARN then
        table.insert(episodePath, currentWP)

        local oldVisitedKey = visitedKey
        local newVisitedSet = {}
        for k,v in pairs(visitedSet) do newVisitedSet[k] = v end
        newVisitedSet[currentWP] = true
        local newVisitedKey = get_visited_key(newVisitedSet)

        local oldQ    = (Q[oldVisitedKey] and Q[oldVisitedKey][currentWP]) or 0
        local reward  = -accumEnergy
        local futureQ = maxFutureQ(newVisitedSet, newVisitedKey)
        local newQ    = oldQ + ALPHA * (reward + GAMMA * futureQ - oldQ)
        
        Q[oldVisitedKey] = Q[oldVisitedKey] or {}
        Q[oldVisitedKey][currentWP] = newQ
        
        epsilon = math.max(MIN_EPSILON, epsilon * EPSILON_DECAY)

        visitedSet = newVisitedSet
        visitedKey = newVisitedKey
        currentWP  = chooseNext(visitedSet, visitedKey)
        accumEnergy = 0

        if currentWP then
            vehicle:set_target_location(waypoints[currentWP])
            state = STATE_CRUISE
        else
            state = STATE_RESET
        end

    elseif state==STATE_RESET then
        -- Generate full path for log file and truncated path for display
        local fullPathStr = table.concat(episodePath, " -> ")
        local displayPathStr = get_display_path(episodePath)
        
        gcs:send_text(0, string.format(
            "EP %d END path: %s total_E=%.3f J",
            epCount, displayPathStr, totalEnergy))
        
        if log_file then
            local log_line = string.format("%d,%.3f,\"%s\"\n", epCount, totalEnergy, fullPathStr)
            log_file:write(log_line)
            log_file:flush()
        end
            
        if totalEnergy < bestEnergy then
            bestEnergy = totalEnergy
            bestPath   = {table.unpack(episodePath)}
            bestEP = epCount
            bestStr = get_display_path(bestPath) -- Update best string with truncated version
            stableCount = 0
        end
        gcs:send_text(0, string.format(
            "EP %d [BEST] path: %s E=%.3f J",
            bestEP, bestStr, bestEnergy))
        
        if stableCount >= STABLE_THRESHOLD then
            stopScript = true
            gcs:send_text(0, string.format("Best path stabilized after %d episodes. Stopping script.", epCount))
        end
        epCount = epCount + 1

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
        if log_file then
            log_file:close()
            log_file = nil
            gcs:send_text(0, "Log file closed.")
        end
        return nil
    end

    if P_ENABLE:get()==0 then
        return loop, LOOP_IDLE
    end

    update()
    return loop, LOOP_FAST
end

gcs:send_text(0, "Loaded RL 50-Waypoints (Fixed, Robust State) script!")
return loop, 1000
