--[[--------------------------------------------------------------------
   TECS Auto-Tuner – single-parameter version (ArduPlane, SITL)
   * Q-learning ajusta APENAS o TECS_SPDWEIGHT.
   * TECS_TIME_CONST e TECS_PTCH_DAMP ficam fixos.
------------------------------------------------------------------------]]

------------------------------------------------------------------ SETUP
local PARAM_TABLE_KEY, PREFIX, LEN = 90, "SIM_ERES_", 6
assert(param:add_table(PARAM_TABLE_KEY, PREFIX, LEN))
local function p(n,i,d) assert(param:add_param(PARAM_TABLE_KEY,i,n,d)); return Parameter(PREFIX..n) end
local P_ENABLE, P_STATE = p("ENABLE",1,1), p("STATE",6,-1)

-- overrides úteis em SITL
param:set("ARMING_CHECK",0)
param:set("GPS_TYPE",0)
param:set("AHRS_EKF_TYPE",10)
param:set("SIM_SPEEDUP",5000)
param:set("EK3_IMU_MASK",    1)
param:set("COMPASS_USE",     0)

------------------------------------------------------------------ GRADE
local SPDW = {0.0, 1.0, 2.0}   -- valores a explorar
local FIX_TCON = 5             -- s  (constante)
local FIX_PDMP = 0.2           --     (constante)

------------------------------------------------------------------ FUNÇÕES
local function key(i) return i end
local function vec3(x,y,z) local v=Vector3f(); v:x(x); v:y(y); v:z(z); return v end
local function set_tecs(i)
    param:set("TECS_SPDWEIGHT", SPDW[i])
    param:set("TECS_TIME_CONST", FIX_TCON)
    param:set("TECS_PTCH_DAMP",  FIX_PDMP)
end

------------------------------------------------------------------ RL VARS
local Q, epi, bestCost, bestEP, SPDWi = {}, 0, 1e9, 0, 1111
local idx_i, epi_time, totalCost
local ALPHA, GAMMA = 0.3, 0.9
local EPSI, EPS_DEC, EPS_MIN = 0.4, 0.97, 0.05

------------------------------------------------------------------ CONSTS
local EP_SECONDS, TARGET_HGT_M, LOITER_RAD_M = 30, 20, 10
local batt = battery
local S_IDLE,S_TKOFF,S_LOITER,S_EVAL,S_RESET = -1,0,1,2,3
local state, home, att = S_IDLE, nil, Quaternion()

------------------------------------------------------------------ LOOP
function update()
    if P_ENABLE:get()==0 then return 500 end

    if state==S_IDLE then
        if ahrs:get_origin() then
            home, att = ahrs:get_origin(), ahrs:get_quaternion()

            -- ε-greedy escolha de SPDW
            if math.random() > EPSI then
                local best = -1e9
                for i=1,#SPDW do
                    local q = Q[key(i)] or 0
                    if q > best then best, idx_i = q, i end
                end
            else
                idx_i = math.random(#SPDW)
            end
            set_tecs(idx_i)

            vehicle:set_mode(13)           -- TAKEOFF
            state, epi_time, totalCost = S_TKOFF, 0, 0
            gcs:send_text(0,string.format(
              "EP %d -------------> SPDW=%g", epi, SPDW[idx_i]))
            SPDWi = SPDW[idx_i]
        end

    elseif state==S_TKOFF then
        if not arming:is_armed() then arming:arm() end
        if ahrs:get_location():alt() - home:alt() > 15*100 then
            vehicle:set_mode(12)           -- LOITER
            param:set("LOITER_RAD",LOITER_RAD_M)
            state = S_LOITER
        end

    elseif state==S_LOITER then
        epi_time = epi_time + 0.05
        local err = math.abs((ahrs:get_location():alt()
                      - (home:alt()+TARGET_HGT_M*100)) / 100)
        local pwr = batt:voltage(0) * batt:current_amps(0)
        totalCost = totalCost + (err + 0.1*pwr) * 0.05
        if epi_time >= EP_SECONDS then state = S_EVAL end

    elseif state==S_EVAL then
        local k, old = key(idx_i), Q[key(idx_i)] or 0
        Q[k] = old + ALPHA * ((-totalCost) - old)       -- reward = –cost
        if totalCost < bestCost then bestCost, bestEP, bestSPDW = totalCost, epi, SPDWi end
        gcs:send_text(0,string.format(
          "EP %d cost=%.1f  best=%.1f at ep %d at SPDW=%d", epi, totalCost, bestCost, bestEP, bestSPDW))

        EPSI = math.max(EPS_MIN, EPSI * EPS_DEC)
        epi, state = epi + 1, S_RESET

    elseif state==S_RESET then
        if arming:is_armed() then arming:disarm() end
        if sim and sim.set_pose then sim:set_pose(0, home, att, vec3(0,0,0), vec3(0,0,0)) end
        state = S_IDLE
    end

    P_STATE:set(state)
    return 50
end

function loop() update(); return loop, 50 end
gcs:send_text(0,"Loaded TECS single-parameter RL-tuner (SPDW)")
return loop, 1000
