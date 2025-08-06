--[[--------------------------------------------------------------------
   TECS Auto-Tuner – 3-parameter version (ArduPlane, SITL)
   * Q-learning explores:
       ▸ TECS_SPDWEIGHT  ∈ {0.0, 1.0, 2.0}
       ▸ TECS_TIME_CONST ∈ {6, 8, 10}
       ▸ TECS_PTCH_DAMP  ∈ {0.0, 0.2, 0.4}
   * Cost = |altitude error| + 0.1 × power (integrated)
------------------------------------------------------------------------]]

------------------------------------------------------------------ SETUP
local PARAM_TABLE_KEY, PREFIX, LEN = 90, "SIM_ERES_", 6
assert(param:add_table(PARAM_TABLE_KEY, PREFIX, LEN))
local function p(n,i,d) assert(param:add_param(PARAM_TABLE_KEY,i,n,d)); return Parameter(PREFIX..n) end
local P_ENABLE, P_STATE = p("ENABLE",1,1), p("STATE",6,-1)

-- SITL overrides
param:set("ARMING_CHECK",0)
param:set("GPS_TYPE",0)
param:set("AHRS_EKF_TYPE",10)
param:set("SIM_SPEEDUP",10000)
param:set("EK3_IMU_MASK",1)
param:set("COMPASS_USE",0)

------------------------------------------------------------------ GRIDS
local SPDW = {0.0, 1.0, 2.0}
local TCON = {6, 8, 10}
local PDMP = {0.0, 0.2, 0.4}

------------------------------------------------------------------ HELPERS
local function key(i,j,k) return i*100 + j*10 + k end
local function vec3(x,y,z) local v=Vector3f(); v:x(x); v:y(y); v:z(z); return v end
local function set_tecs(i,j,k)
    param:set("TECS_SPDWEIGHT", SPDW[i])
    param:set("TECS_TIME_CONST", TCON[j])
    param:set("TECS_PTCH_DAMP",  PDMP[k])
end

------------------------------------------------------------------ RL STATE
local Q, epi, bestCost, bestEP, bestCombo = {}, 0, 1e9, 0, {1,1,1}
local idx_i, idx_j, idx_k
local ALPHA, GAMMA        = 0.3, 0.9
local EPSI, EPS_DEC, EPS_MIN = 0.5, 0.97, 0.05

------------------------------------------------------------------ CONSTANTS
local EP_SECONDS, TARGET_HGT_M, LOITER_RAD_M = 30, 20, 10
local batt = battery
local S_IDLE,S_TKOFF,S_LOITER,S_EVAL,S_RESET = -1,0,1,2,3
local state, home, att = S_IDLE, nil, Quaternion()
local epi_time, totalCost

------------------------------------------------------------------ MAIN UPDATE
function update()
    if P_ENABLE:get()==0 then return 500 end

    ------------------------------------------------------------------ IDLE
    if state==S_IDLE then
        if ahrs:get_origin() then
            home, att = ahrs:get_origin(), ahrs:get_quaternion()

            -- ε-greedy choice
            idx_i, idx_j, idx_k = math.random(#SPDW), math.random(#TCON), math.random(#PDMP)
            if math.random() > EPSI then
                local best = -1e9
                for i=1,#SPDW do for j=1,#TCON do for k=1,#PDMP do
                    local q = Q[key(i,j,k)] or 0
                    if q>best then best, idx_i, idx_j, idx_k = q,i,j,k end
                end end end
            end
            set_tecs(idx_i, idx_j, idx_k)

            vehicle:set_mode(13)           -- TAKEOFF
            state, epi_time, totalCost = S_TKOFF, 0, 0
            gcs:send_text(0, string.format(
              "EP %d ------------------------> SPDW=%.1f  TCON=%d  PDMP=%.1f",
              epi, SPDW[idx_i], TCON[idx_j], PDMP[idx_k]))
        end

    ---------------------------------------------------------------- TAKEOFF
    elseif state==S_TKOFF then
        if not arming:is_armed() then arming:arm() end
        if ahrs:get_location():alt() - home:alt() > 15*100 then
            vehicle:set_mode(12)           -- LOITER
            param:set("LOITER_RAD", LOITER_RAD_M)
            state = S_LOITER
        end

    ---------------------------------------------------------------- LOITER
    elseif state==S_LOITER then
        epi_time = epi_time + 0.05
        local altErr = math.abs((ahrs:get_location():alt() -
                                 (home:alt()+TARGET_HGT_M*100))/100)
        local power  = batt:voltage(0) * batt:current_amps(0)
        totalCost = totalCost + (altErr + 0.1*power)*0.05
        if epi_time >= EP_SECONDS then state = S_EVAL end

    ---------------------------------------------------------------- EVAL
    elseif state==S_EVAL then
        local k, old = key(idx_i,idx_j,idx_k), Q[key(idx_i,idx_j,idx_k)] or 0
        Q[k] = old + ALPHA*((-totalCost) - old)
        if totalCost < bestCost then
            bestEP, bestCost, bestCombo = epi, totalCost, {idx_i, idx_j, idx_k}
        end
        gcs:send_text(0, string.format(
           "EP %d cost=%.1f ======================> best=%.1f @ EP %d combo=[%.1f,%d,%.1f]",
           epi, totalCost, bestCost, bestEP,
           SPDW[bestCombo[1]], TCON[bestCombo[2]], PDMP[bestCombo[3]]))

        EPSI = math.max(EPS_MIN, EPSI*EPS_DEC)
        epi, state = epi+1, S_RESET

    ---------------------------------------------------------------- RESET
    elseif state==S_RESET then
        if arming:is_armed() then arming:disarm() end
        if sim and sim.set_pose then
            sim:set_pose(0, home, att, vec3(0,0,0), vec3(0,0,0))
        end
        state = S_IDLE
    end  -- closes big if-chain

    P_STATE:set(state)
    return 50
end  -- closes function update()

------------------------------------------------------------------ LOOPER
function loop()
    update()
    return loop, 50
end

gcs:send_text(0,"Loaded TECS 3-parameter RL-tuner (SPDW, TCON, PDMP)")
return loop, 1000
