--[[--------------------------------------------------------------------
    RL Episodic-Reset Script – Path + Speed Optimisation (time reward)
    • Learns both waypoint order (5 points) and WPNAV_SPEED choice
      via ε-greedy Q-learning.  Reward = –segment_time_s
------------------------------------------------------------------------]]

------------------------------------------------------------------ PARAMS --
local PARAM_TABLE_KEY, PARAM_TABLE_PREFIX, TABLE_LEN = 90, "SIM_ERES_", 18
assert(param:add_table(PARAM_TABLE_KEY, PARAM_TABLE_PREFIX, TABLE_LEN),
       "failed to create parameter table")

local function bind_add_param(name, idx, def)
    assert(param:add_param(PARAM_TABLE_KEY, idx, name, def),
           string.format("could not add param %s", name))
    return Parameter(PARAM_TABLE_PREFIX .. name)
end
local P_ENABLE   = bind_add_param("ENABLE",  1,  1)   -- 0 = idle, 1 = run
local P_OFST_N   = bind_add_param("OFST_N",  2, 20)   -- N offset (m)
local P_OFST_E   = bind_add_param("OFST_E",  3, 20)   -- E offset (m)
local P_OFST_D   = bind_add_param("OFST_D",  4, -20)  -- D offset (m, +down)
local P_VEL_X    = bind_add_param("VEL_X",   5,  0)
local P_VEL_Y    = bind_add_param("VEL_Y",   6,  0)
local P_VEL_Z    = bind_add_param("VEL_Z",   7,  0)
local P_RLL      = bind_add_param("RLL",     8,  0)
local P_PIT      = bind_add_param("PIT",     9,  0)
local P_YAW      = bind_add_param("YAW",    10,  0)
local P_GX       = bind_add_param("GX",     11,  0)
local P_GY       = bind_add_param("GY",     12,  0)
local P_GZ       = bind_add_param("GZ",     13,  0)
local P_MODE     = bind_add_param("MODE",   14, -1)
local P_POS_LAT  = bind_add_param("POS_N",  15,  0)
local P_POS_LNG  = bind_add_param("POS_E",  16,  0)
local P_POS_ALT  = bind_add_param("POS_D",  17,  0)
local P_STATE    = bind_add_param("STATE",  18, -1)

-- Disable arming checks (SITL only)
param:set("ARMING_CHECK", 0); param:set("GPS_TYPE", 0)
param:set("GPS_AUTO_CONFIG", 0); param:set("GPS_AUTO_SWITCH", 0)
param:set("AHRS_EKF_TYPE", 10)
param:set("SIM_SPEEDUP", 1000)

---------------------------------------------------------------- CONSTANTS --
local TAKEOFF_ALT_M, TARGET_THRESH_M = 10, 0.5
local LOOP_FAST,  LOOP_IDLE          = 50, 500      -- ms

---------------------------------------------------------------- FSM ENUM --
local STATE_WAIT, STATE_ARM_TAKE, STATE_CRUISE, STATE_LEARN, STATE_RESET =
      -1,         0,               1,              2,             3

---------------------------------------------------------------- VARIABLES --
local state, home_loc, base_att, takeoff_sent =
      STATE_WAIT, nil, Quaternion(), false

-- RL hyper-parameters
local ALPHA, GAMMA              = 0.1, 0.9
local EPSILON, EPS_DECAY, EPS_MIN= 0.2, 0.98, 0.01

local STABLE_THRESHOLD, stableCnt, stopScript = 600, 0, false

-- Waypoint offsets (m)
local wp_offsets = {
    {N= 20,E= 0,D=-20}, {N= 0,E=20,D=-20}, {N=-20,E= 0,D=-20},
    {N= 0,E=-20,D=-20}, {N=20,E=20,D=-20},
}
local numWP,  waypoints = #wp_offsets, {}

-- Speed choices (cm/s) to tune
local speed_opts = { 500, 700, 1300, 2000 }
local numSPD     = #speed_opts

-- Q-table  ==>  Q[mask][action]   where action = (spd_idx-1)*numWP + wp_idx
local Q = {}

-- Episode bookkeeping
local epCnt,  bestTime, bestPath, bestSPD, bestEP =
      1, math.huge, {}, {}, 0
local visitedMask, curAct, curWP, curSPD
local episodePath, episodeSpeed = {}, {}
local segStart_ms, totalTime_s  = 0, 0

---------------------------------------------------------------- HELPERS ----
local function build_wp(home, off)
    local L=Location();  L:lat(home:lat()); L:lng(home:lng())
    L:offset(off.N, off.E);  L:alt(home:alt() - off.D*100);  return L
end
local function encode(wp_idx, spd_idx)  return (spd_idx-1)*numWP + wp_idx end
local function decode(action)
    local spd_idx = math.floor((action-1)/numWP)+1
    local wp_idx  = ((action-1) % numWP)+1
    return wp_idx, spd_idx
end
local function ms() return tonumber(tostring(millis())) end
local function rel_alt()
    return math.abs((ahrs:get_location():alt() - home_loc:alt())/100)
end
local function vec3(x,y,z)
    local v=Vector3f(); v:x(x); v:y(y); v:z(z); return v
end
local function chooseNext(mask)
    local cands={}
    for wp=1,numWP do
        if (mask & (1<<(wp-1)))==0 then
            for s=1,numSPD do table.insert(cands, encode(wp,s)) end
        end
    end
    if #cands==0 then return nil end
    if math.random() < EPSILON then return cands[math.random(#cands)] end
    local bestA, bestQ = cands[1], -1e9
    for _,a in ipairs(cands) do
        local q = (Q[mask] and Q[mask][a]) or 0
        if q > bestQ then bestQ, bestA = q, a end
    end
    return bestA
end
local function maxFutureQ(mask)
    local best=0
    for wp=1,numWP do
        if (mask & (1<<(wp-1)))==0 then
            for s=1,numSPD do
                local a=encode(wp,s)
                local v=(Q[mask] and Q[mask][a]) or 0
                if v>best then best=v end
            end
        end
    end
    return best
end

-------------------------------------------------------------- MAIN LOOP ----
function update()
    if P_ENABLE:get()==0 then return LOOP_IDLE end

    -- ============ WAIT =================
    if state==STATE_WAIT then
        if ahrs:get_origin() then
            home_loc = ahrs:get_home() or ahrs:get_location()
            base_att = ahrs:get_quaternion()
            for i,off in ipairs(wp_offsets) do waypoints[i]=build_wp(home_loc,off) end

            visitedMask      = 0
            curAct           = chooseNext(visitedMask)
            curWP, curSPD    = decode(curAct)
            param:set("WPNAV_SPEED", speed_opts[curSPD])

            episodePath, episodeSpeed = {}, {}
            totalTime_s = 0
            state       = STATE_ARM_TAKE
        end

    -- ============ ARM & TAKEOFF =========
    elseif state==STATE_ARM_TAKE then
        if vehicle:get_mode()~=4 then vehicle:set_mode(4) end
        if not arming:is_armed() then
            arming:arm()
        elseif not takeoff_sent then
            vehicle:start_takeoff(TAKEOFF_ALT_M); takeoff_sent=true
        elseif rel_alt() >= TAKEOFF_ALT_M-0.5 then
            vehicle:set_target_location(waypoints[curWP])
            segStart_ms = ms()
            state = STATE_CRUISE
        end

    -- ============ CRUISE ================
    elseif state==STATE_CRUISE then
        if ahrs:get_location():get_distance(waypoints[curWP]) <= TARGET_THRESH_M then
            state = STATE_LEARN
        end

    -- ============ LEARN =================
    elseif state==STATE_LEARN then
        local segTime = (ms() - segStart_ms)/1000
        totalTime_s   = totalTime_s + segTime
        table.insert(episodePath, curWP)
        table.insert(episodeSpeed, speed_opts[curSPD])

        local reward      = -segTime
        Q[visitedMask]    = Q[visitedMask] or {}
        local oldQ        = Q[visitedMask][curAct] or 0
        local futureQ     = (chooseNext(visitedMask | (1<<(curWP-1))) and
                             maxFutureQ(visitedMask | (1<<(curWP-1)))) or 0
        local newQ        = oldQ + ALPHA*(reward + GAMMA*futureQ - oldQ)
        Q[visitedMask][curAct] = newQ

        gcs:send_text(0,string.format(
            "EP%3d WP%d @%dcm/s Δt=%.2fs Q=%.2f",
            epCnt, curWP, speed_opts[curSPD], segTime, newQ))

        EPSILON     = math.max(EPS_MIN, EPSILON*EPS_DECAY)
        visitedMask = visitedMask | (1<<(curWP-1))
        curAct      = chooseNext(visitedMask)

        if curAct then
            curWP, curSPD = decode(curAct)
            param:set("WPNAV_SPEED", speed_opts[curSPD])
            vehicle:set_target_location(waypoints[curWP])
            segStart_ms = ms()
            state = STATE_CRUISE
        else
            state = STATE_RESET
        end

    -- ============ RESET =================
    elseif state==STATE_RESET then
        local pathStr = table.concat(episodePath,"->")
        local spdStr  = table.concat(episodeSpeed,",")
        gcs:send_text(0,string.format(
            "EP %d END path:%s  SPD:%s  T=%.2fs",
            epCnt, pathStr, spdStr, totalTime_s))

        if totalTime_s < bestTime then
            bestPathStr = pathStr
            bestSPDStr = spdStr
            bestTime, bestPath, bestSPD, bestEP =
                totalTime_s, {table.unpack(episodePath)},
                {table.unpack(episodeSpeed)}, epCnt
            stableCnt = 0
        else
            stableCnt = stableCnt + 1
        end
        gcs:send_text(0, string.format(
            "BEST (EP %d): path:%s  SPD:%s  T=%.2fs",
            bestEP, bestPathStr, bestSPDStr, bestTime))

        if stableCnt >= STABLE_THRESHOLD then
            stopScript=true
            gcs:send_text(0,"Converged – stopping.")
        end

        epCnt = epCnt + 1
        sim:set_pose(0, home_loc, base_att, vec3(0,0,0), vec3(0,0,0))
        if arming:is_armed() then arming:disarm() end
        takeoff_sent, state = false, STATE_WAIT
    end

    return (state==STATE_WAIT and LOOP_IDLE) or LOOP_FAST
end

-------------------------------------------------------------- SCHEDULER ----
function loop()
    if stopScript then return nil end
    if P_ENABLE:get()==0 then return loop, LOOP_IDLE end
    update()
    return loop, LOOP_FAST
end

gcs:send_text(0,"Loaded RL path+speed optimisation script!")
return loop, 0
