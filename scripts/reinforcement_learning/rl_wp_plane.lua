--[[--------------------------------------------------------------------
    RL Episodic-Reset Script (ArduPlane, safe for builds ±SIM API)
    - Fly through 5 waypoints via Q-learning,
      log segment energy and track best paths.
------------------------------------------------------------------------]]

------------------------------------------------------------------
-- PARAMETER-TABLE DEFINITIONS
------------------------------------------------------------------
local PARAM_TABLE_KEY, PARAM_TABLE_PREFIX, TABLE_LEN = 90, "SIM_ERES_", 18
assert(param:add_table(PARAM_TABLE_KEY, PARAM_TABLE_PREFIX, TABLE_LEN),
       "failed to create parameter table")

local function bind_add_param(name, idx, default)
    assert(param:add_param(PARAM_TABLE_KEY, idx, name, default),
           string.format("could not add param %s", name))
    return Parameter(PARAM_TABLE_PREFIX .. name)
end

-- user-configurable parameters
local P_ENABLE = bind_add_param("ENABLE", 1, 1)   -- 0=idle, 1=run
local P_STATE  = bind_add_param("STATE", 18, -1)  -- FSM state

------------------------------------------------------------------
-- GLOBAL PARAM OVERRIDES (SITL only)
------------------------------------------------------------------
param:set("ARMING_CHECK",    0)
param:set("GPS_TYPE",        0)
param:set("GPS_AUTO_CONFIG", 0)
param:set("GPS_AUTO_SWITCH", 0)
param:set("AHRS_EKF_TYPE",  10)
param:set("SIM_SPEEDUP",  5)

------------------------------------------------------------------
-- CONSTANTS
------------------------------------------------------------------
local TAKEOFF_ALT_M, TARGET_THRESH_M = 30, 2
local LOOP_FAST, LOOP_IDLE           = 50, 500
local GUIDED_PLANE_MODE              = 15
local TAKEOFF_MODE                   = 13

------------------------------------------------------------------
-- FSM STATES
------------------------------------------------------------------
local STATE_WAIT   = -1
local STATE_LAUNCH =  0
local STATE_CRUISE =  1
local STATE_LEARN  =  2
local STATE_RESET  =  3

------------------------------------------------------------------
-- STATE VARIABLES
------------------------------------------------------------------
local state, home_loc, base_att = STATE_WAIT, nil, Quaternion()
local launch_done = false

------------------------------------------------------------------
-- RL CONFIG
------------------------------------------------------------------
local ALPHA, GAMMA                = 0.1, 0.9
local INITIAL_EPSILON             = 0.2
local EPSILON_DECAY, MIN_EPSILON  = 0.98, 0.01
local epsilon                     = INITIAL_EPSILON
local STABLE_THRESHOLD            = 50
local stableCount, stopScript     = 0, false

-- five waypoint offsets (N/E/D, m)
local wp_offsets = {
    {N= 20, E=  0, D=-30},
    {N=  0, E= 20, D=-30},
    {N=-20, E=  0, D=-30},
    {N=  0, E=-20, D=-30},
    {N= 20, E= 20, D=-30},
}

local GLOBAL_WP = {
    { lat = -35.361347, lng = 149.165085, alt_m = 615 },
    { lat = -35.362000, lng = 149.166000, alt_m = 615 },
    { lat = -35.363000, lng = 149.164000, alt_m = 615 },
    { lat = -35.362000, lng = 149.163000, alt_m = 615 },
    { lat = -35.361000, lng = 149.166000, alt_m = 615 },
}

local numWP, waypoints, Q = #wp_offsets, {}, {}

------------------------------------------------------------------
-- LOGGING
------------------------------------------------------------------
local epCount, episodePath, bestEnergy = 1, {}, math.huge
local bestPath, bestStr, bestEP, totalEnergy = {}, "a", 0, 0

------------------------------------------------------------------
-- HELPERS
------------------------------------------------------------------
local function vec3(x,y,z) local v=Vector3f(); v:x(x); v:y(y); v:z(z); return v end
local function stateKey(mask, idx) return mask*10 + idx end

local function build_wp(home, off)
    local L = Location()
    L:lat(home:lat()); L:lng(home:lng())
    L:offset(off.N, off.E)
    L:alt(home:alt() - off.D*100)  -- cm
    return L
end

local function build_global_wp(coord)
    local L = Location()
    L:lat(1e7*(coord.lat))
    L:lng(1e7*(coord.lng))
    L:alt(coord.alt_m * 100) -- metres → centimetres
    return L
end

local function maxFutureQ(mask)
    local best = 0
    for i=1,numWP do
        if (mask & (1<<(i-1))) == 0 then
            local key, v = stateKey(mask,i), (Q[key] and Q[key][i]) or 0
            if v>best then best=v end
        end
    end
    return best
end

local function chooseNext(mask)
    local cands={}
    for i=1,numWP do if (mask & (1<<(i-1)))==0 then cands[#cands+1]=i end end
    if #cands==0 then return nil end
    if math.random()<epsilon then return cands[math.random(#cands)] end
    local bestIdx,bestVal=cands[1],-1e9
    for _,i in ipairs(cands) do
        local key, v = stateKey(mask,i), (Q[key] and Q[key][i]) or 0
        if v>bestVal then bestVal,bestIdx=v,i end
    end
    return bestIdx
end

-- Detect if SIM API is available
local HAS_SIM = (type(sim)=="table" and sim.set_pose) and true or false
if not HAS_SIM then
    gcs:send_text(0,"[WARN] SIM API missing - teleport/reset disabled")
end

local function rel_alt()
    return math.abs((ahrs:get_location():alt() - home_loc:alt()) / 100)
end

local timers = timers or {}
local function wait_s(sec)
    local id = "_"

    local dl = timers[id]
    if not dl then
        dl = millis() + math.floor(sec * 1000 + 0.5)  -- deadline (ms)
        timers[id] = dl
    end

    local now = millis()
    if now >= dl then
        timers[id] = nil         
        return true, 0          
    end
    return false, (dl - now) / 1000.0 
end

------------------------------------------------------------------
-- MAIN UPDATE
------------------------------------------------------------------
local batt = battery

function update()
    if P_ENABLE:get()==0 then return LOOP_IDLE end

    if state==STATE_WAIT then
        if ahrs:get_origin() then
            home_loc = ahrs:get_origin()
            base_att = ahrs:get_quaternion()
            -- build the GLOBAL waypoint list exactly once
            waypoints = {}
            for i,coord in ipairs(GLOBAL_WP) do waypoints[i]=build_global_wp(coord) end
            visitedMask, currentWP, accumEnergy, totalEnergy = 0, chooseNext(0), 0, 0
            episodePath, launch_done, state = {}, false, STATE_LAUNCH
            gcs:send_text(0,"[FSM] WAIT → LAUNCH (global waypoints)")
        end

    elseif state == STATE_LAUNCH then
        -- 1) switch into Plane's TAKEOFF mode
        if vehicle:get_mode() ~= TAKEOFF_MODE then
            vehicle:set_mode(TAKEOFF_MODE)
            gcs:send_text(0,"takeoff mode")
        -- 2) arm motors
        elseif not arming:is_armed() then
            arming:arm()
            gcs:send_text(0,"arming motors")
        -- 3) wait until we reach a safe altitude before moving on
        gcs:send_text(0,string.format("(%.1f m)", rel_alt()))
        elseif rel_alt() < (20) then
            -- just idle here until above ~20 m
            gcs:send_text(0,string.format("waiting for takeoff altitude (%.1f m)", rel_alt()))
        -- 4) once high enough, switch to GUIDED and head to first WP
        else
            vehicle:set_mode(GUIDED_PLANE_MODE)
            vehicle:set_target_location(waypoints[currentWP])
            state = STATE_CRUISE
        end

    elseif state==STATE_CRUISE then
        totalEnergy = totalEnergy + batt:voltage(0)*batt:current_amps(0)*(LOOP_FAST/1000)
        accumEnergy = accumEnergy + batt:voltage(0)*batt:current_amps(0)*(LOOP_FAST/1000)
        if wait_s(20) then
            state = STATE_LEARN
        end

    elseif state==STATE_LEARN then
        table.insert(episodePath,currentWP)
        local bit=1<<(currentWP-1)
        local newMask=visitedMask|bit
        local oldKey=stateKey(visitedMask,currentWP)
        local oldQ=(Q[oldKey] and Q[oldKey][currentWP]) or 0
        local nextWP=chooseNext(newMask)
        local reward=-accumEnergy
        local futureQ=nextWP and maxFutureQ(newMask) or 0
        local newQ=oldQ+ALPHA*(reward+GAMMA*futureQ-oldQ)
        Q[oldKey]=Q[oldKey] or {}; Q[oldKey][currentWP]=newQ
        gcs:send_text(0,string.format("EP %d: Q[%d,%d]=%.3f next=%s",
            epCount,visitedMask,currentWP,newQ,tostring(nextWP)))
        epsilon=math.max(MIN_EPSILON,epsilon*EPSILON_DECAY)
        visitedMask,currentWP,accumEnergy = newMask,nextWP,0
        if currentWP then
            vehicle:set_target_location(waypoints[currentWP])
            state=STATE_CRUISE
        else
            state=STATE_RESET
        end

    elseif state==STATE_RESET then
        local pathStr=table.concat(episodePath," -> ")
        gcs:send_text(0,string.format("EP %d END %s E=%.3f J",epCount,pathStr,totalEnergy))
        if totalEnergy<bestEnergy then
            bestEnergy,bestPath,bestEP,stableCount=totalEnergy,{table.unpack(episodePath)},epCount,0
            bestStr=table.concat(bestPath," -> ")
        else stableCount=stableCount+1 end
        gcs:send_text(0,string.format("BEST[%d] %s E=%.3f J",bestEP,bestStr,bestEnergy))
        if stableCount>=STABLE_THRESHOLD then
            stopScript=true
            gcs:send_text(0,"Best path stabilized - stopping.")
        end
        epCount=epCount+1
        if true then
            sim:set_pose(0, home_loc, base_att, vec3(0,0,0), vec3(0,0,0))
            if ahrs:get_location() ~= ahrs:get_origin() then
                gcs:send_text(0,"[WARN] SIM API failed to reset pose")
            else
                gcs:send_text(0,"Reset pose to home location")
            end
        else
            gcs:send_text(0,"No SIM API - no reset pose")
        end
        if arming:is_armed() then arming:disarm() end
        launch_done, state = false, STATE_WAIT
    end

    P_STATE:set(state)
    return (state==STATE_WAIT and LOOP_IDLE) or LOOP_FAST
end

------------------------------------------------------------------
-- SCHEDULER ENTRY POINT
------------------------------------------------------------------
function loop()
    if stopScript then return nil end
    if P_ENABLE:get()==0 then return loop, LOOP_IDLE end
    update()
    return loop, LOOP_FAST
end

gcs:send_text(0,"Loaded RL waypoint script (Plane) - SIM fallback ready!")
return loop, 1000
