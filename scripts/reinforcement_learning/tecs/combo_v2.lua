--[[--------------------------------------------------------------------
   TECS Auto-Tuner - Multi-Parameter (ArduPlane, SITL)
   * Implements the full RL problem as described by the mentor.

   * Episode with 3 segments (~45-60s each):
       A) Climb:   +75 m @ AIRSPEED_CRUISE
       B) Speed step: ±3–4 m/s @ constant altitude
       C) Descent: −75 m @ same speed as B (constant)

   * Reward:
       R = -IAE_speed - IAE_alt - w1*Var(throttle) - w2*RMS(pitch_rate)
       + large penalties for critical events.

   * Actions (one change per episode, bounds enforced):
       {NoChange, SPDW+, SPDW−, TCON+, TCON−, PDMP+, PDMP−}
------------------------------------------------------------------------]]

------------------------------------------------------------------
-- SETUP
------------------------------------------------------------------
local PARAM_TABLE_KEY, PREFIX, LEN = 92, "RL_", 6
assert(param:add_table(PARAM_TABLE_KEY, PREFIX, LEN))
function p(n,i,d) assert(param:add_param(PARAM_TABLE_KEY,i,n,d)); return Parameter(PREFIX..n) end
local P_ENABLE, P_STATE = p("ENABLE",1,1), p("STATE",6,-1)

-- SITL overrides
param:set("ARMING_CHECK", 0); param:set("GPS_TYPE", 0); param:set("AHRS_EKF_TYPE", 10)
param:set("SIM_SPEEDUP", 10000000); param:set("EK3_IMU_MASK", 1)
param:set("COMPASS_USE", 0); param:set("COMPASS_USE2", 0); param:set("COMPASS_USE3", 0)

-- Airspeed settings
param:set("ARSPD_USE", 1); param:set("ARSPD_TYPE", 1); param:set("SIM_ARSPD_FAIL", 0)
param:set("ARSPD_FBW_MIN", 12); param:set("ARSPD_FBW_MAX", 28)
param:set("AIRSPEED_CRUISE", 18)

------------------------------------------------------------------
-- PARAMETER GRIDS AND CONSTANTS
------------------------------------------------------------------
local SPDW = {0, 1, 2}
local TCON = {1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 10.0, 11.0}
local PDMP = {0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1.0, 1.1, 1.2}

-- Episode constants
local SEG_DUR_S = 60.0
local CLIMB_DH_M = 75; local DESC_DH_M  = 75; local SPEED_STEP_MPS = 4.0
local THR_CLIP_HI, THR_CLIP_LO = 0.98, 0.02; local CLIP_FRAC_LIM = 0.35
local OVERSPEED_MARG, STALL_MARG = 1.0, 1.0; local BIG_PENALTY = 500.0

-- Reward weights
local W1_VAR_THR = 0.5; local W2_RMS_PR  = 1.0

-- Safety constants
local AGL_SAFE_M = 70.0; local AGL_MIN_M = 35.0; local GROUND_AGL_ABORT_M = 10.0

------------------------------------------------------------------
-- REINFORCEMENT LEARNING VARIABLES
------------------------------------------------------------------
local Q = {}; local epi = 0; local bestReward, bestEP, bestParams = -1e18, 0, {}
local ALPHA, GAMMA = 0.10, 0.95; local EPSI, EPS_DEC, EPS_MIN = 1.0, 0.99, 0.05
local ACTIONS = { "NC", "SPDW+", "SPDW-", "TCON+", "TCON-", "PDMP+", "PDMP-" }
local s_prev_key, a_prev; local idx_spdw, idx_tcon, idx_pdmp

-- Store original parameter values to be restored after each episode
local original_thr_max = param:get("THR_MAX") or 100
local original_pitch_min = param:get("TECS_PITCH_MIN") or -15

------------------------------------------------------------------
-- HELPER FUNCTIONS
------------------------------------------------------------------
function zero_vector3f() local v=Vector3f(); v:x(0); v:y(0); v:z(0); return v end
function clamp(v, a, b) return math.max(a, math.min(b, v)) end
function atan2(y,x) if math.atan2 then return math.atan2(y,x) end; local ok,v=pcall(math.atan,y,x) if ok and v then return v end if x>0 then return math.atan(y/x) elseif x<0 and y>=0 then return math.atan(y/x)+math.pi elseif x<0 and y<0 then return math.atan(y/x)-math.pi elseif x==0 and y>0 then return math.pi/2 elseif x==0 and y<0 then return -math.pi/2 else return 0.0 end end
function get_airspeed_mps() if not ahrs then return nil end; local v=ahrs:airspeed_estimate() if v and type(v)=='number' then return v end; return nil end
function get_throttle_norm() if SRV_Channels and SRV_Channels:find_channel(70) and SRV_Channels.get_output_norm then return SRV_Channels:get_output_norm(SRV_Channels:find_channel(70)) end; return nil end
function get_pitch_rate_abs() local g=ahrs:get_gyro() return g and math.abs(g:y()) or 0 end
function get_agl_m(base) local loc=ahrs:get_location() if not loc then return 0 end; return (loc:alt()-base)/100.0 end
function project_waypoint_ahead(d) local l=ahrs:get_location() if not l then return nil end; local y=ahrs:get_yaw_rad() if y==nil then return nil end; local R=6378137.0; local la1=math.rad(l:lat()/1e7); local lo1=math.rad(l:lng()/1e7); local r=d/R; local si_r,co_r=math.sin(r),math.cos(r); local si_la1,co_la1=math.sin(la1),math.cos(la1); local si_la2=si_la1*co_r+co_la1*si_r*math.cos(y); local la2=math.asin(si_la2); local Y=math.sin(y)*si_r*co_la1; local X=co_r-si_la1*si_la2; local lo2=lo1+atan2(Y,X); local t=Location(); t:lat(math.deg(la2)*1e7); t:lng(math.deg(lo2)*1e7); return t end
function set_guided_target(t,alt,base) local f=base+AGL_MIN_M*100; local s=math.max(alt,f); t:alt(s); vehicle:set_target_location(t) end
function bin(x, e) for i=1,#e do if x<e[i] then return i end end; return #e+1 end
function qget(s_key,a) Q[s_key]=Q[s_key] or {}; return Q[s_key][a] or 0.0 end
function qset(s_key,a,v) Q[s_key]=Q[s_key] or {}; Q[s_key][a]=v end
function argmax_a(s_key) local b_a,b_q=1,-1e18 for a=1,#ACTIONS do local q=qget(s_key,a); if q>b_q then b_q,b_a=q,a end end; return b_a,b_q end
function set_tecs_params(i_spdw, i_tcon, i_pdmp) param:set("TECS_SPDWEIGHT", SPDW[i_spdw]); param:set("TECS_TIME_CONST", TCON[i_tcon]); param:set("TECS_PTCH_DAMP",  PDMP[i_pdmp]) end

------------------------------------------------------------------
-- RUNTIME STATE AND EPISODE MANAGEMENT
------------------------------------------------------------------
local S_IDLE, S_TKOFF, S_INITIAL_CLIMB, S_START_SEG_A, S_SEG_A, S_START_SEG_B, S_SEG_B, S_START_SEG_C, S_SEG_C, S_EVAL, S_RESET = -1,0,1,2,3,4,5,6,7,8,9
local state, home = S_IDLE, nil
local base_alt_cm, target_alt_cm, A0_alt_cm; local base_cruise, speed_step_val
local seg_timer, dt = 0.0, 0.05
local iae_alt, iae_spd, n_thr, thr_mean, thr_M2, pr_sum2, pr_n, clip_hi, clip_lo, clip_n, max_alt_err, max_spd_err, abort_flag, abort_reason

function reset_metrics()
  iae_alt, iae_spd = 0, 0; n_thr, thr_mean, thr_M2 = 0, 0, 0; pr_sum2, pr_n = 0, 0
  clip_hi, clip_lo, clip_n = 0, 0, 0; max_alt_err, max_spd_err = 0, 0
  abort_flag, abort_reason = false, ""
end

------------------------------------------------------------------
-- MAIN UPDATE FUNCTION
------------------------------------------------------------------
function update()
  if P_ENABLE:get()==0 then return end

  -------------------------------------------------------------- IDLE
  if state==S_IDLE then
    if ahrs:get_origin() and ahrs:get_location() then
      home = ahrs:get_origin(); base_alt_cm = home:alt(); base_cruise = param:get("AIRSPEED_CRUISE") or 18.0; A0_alt_cm = base_alt_cm + AGL_SAFE_M * 100
      if epi == 0 then idx_spdw=math.floor(#SPDW/2)+1; idx_tcon=math.floor(#TCON/2)+1; idx_pdmp=math.floor(#PDMP/2)+1; s_prev_key="1-1-1" end
      a_prev = (math.random() < EPSI) and math.random(#ACTIONS) or argmax_a(s_prev_key)
      if a_prev == 2 then idx_spdw = math.min(idx_spdw + 1, #SPDW) elseif a_prev == 3 then idx_spdw = math.max(idx_spdw - 1, 1) elseif a_prev == 4 then idx_tcon = math.min(idx_tcon + 1, #TCON) elseif a_prev == 5 then idx_tcon = math.max(idx_tcon - 1, 1) elseif a_prev == 6 then idx_pdmp = math.min(idx_pdmp + 1, #PDMP) elseif a_prev == 7 then idx_pdmp = math.max(idx_pdmp - 1, 1) end
      set_tecs_params(idx_spdw, idx_tcon, idx_pdmp)
      vehicle:set_mode(13); state = S_TKOFF; reset_metrics()
      gcs:send_text(0, string.format("EP %d | A: %s | PARAMS: S=%d T=%.1f P=%.1f", epi, ACTIONS[a_prev], SPDW[idx_spdw], TCON[idx_tcon], PDMP[idx_pdmp]))
    end
  -------------------------------------------------------------- TAKEOFF & INITIAL CLIMB
  elseif state==S_TKOFF then
    if not arming:is_armed() then arming:arm() end
    if get_agl_m(base_alt_cm) > 20 then vehicle:set_mode(15); state = S_INITIAL_CLIMB; gcs:send_text(0, "Takeoff complete. Starting initial climb.") end
  elseif state == S_INITIAL_CLIMB then
    local climb_wp = project_waypoint_ahead(600); if climb_wp then set_guided_target(climb_wp, A0_alt_cm, base_alt_cm) end
    if get_agl_m(base_alt_cm) >= AGL_SAFE_M - 2.0 then gcs:send_text(0, "Initial climb complete. Starting Segments."); state = S_START_SEG_A end
  -------------------------------------------------------------- SEGMENT LOGIC
  -- Prepare Segment A (Climb)
  elseif state == S_START_SEG_A then
    speed_step_val=(math.random()<0.5 and -1 or 1)*SPEED_STEP_MPS; param:set("AIRSPEED_CRUISE",base_cruise); target_alt_cm=A0_alt_cm+CLIMB_DH_M*100
    local wp=project_waypoint_ahead(8000); if wp then set_guided_target(wp,target_alt_cm,base_alt_cm) end; seg_timer=0.0; state=S_SEG_A
  -- Prepare Segment B (Speed Step)
  elseif state == S_START_SEG_B then
    gcs:send_text(0,"EP "..epi..": Seg B Start"); local min_cmd=(param:get("ARSPD_FBW_MIN")or 12)+1.0; local max_cmd=(param:get("ARSPD_FBW_MAX")or 28)-1.0
    local cmdB=clamp(base_cruise+speed_step_val,min_cmd,max_cmd); param:set("AIRSPEED_CRUISE",cmdB); target_alt_cm=ahrs:get_location():alt()
    local wp=project_waypoint_ahead(8000); if wp then set_guided_target(wp,target_alt_cm,base_alt_cm) end; seg_timer=0.0; state=S_SEG_B
  -- Prepare Segment C (Descent)
  elseif state == S_START_SEG_C then
    gcs:send_text(0,"EP "..epi..": Seg C Start"); param:set("THR_MAX",60); param:set("TECS_PITCH_MIN", -10)
    target_alt_cm=A0_alt_cm
    local wp=project_waypoint_ahead(8000); if wp then set_guided_target(wp,target_alt_cm,base_alt_cm) end; seg_timer=0.0; state=S_SEG_C
  -- Execute Segments (A, B, or C)
  elseif state == S_SEG_A or state == S_SEG_B or state == S_SEG_C then
    seg_timer=seg_timer+dt; local alt_cm=ahrs:get_location():alt(); local alt_err_m=math.abs((alt_cm-target_alt_cm)/100.0)
    iae_alt=iae_alt+alt_err_m*dt; if alt_err_m>max_alt_err then max_alt_err=alt_err_m end
    local v_mps=get_airspeed_mps()
    if v_mps then
      local v_cmd=param:get("AIRSPEED_CRUISE"); local spd_err=math.abs(v_mps-v_cmd); iae_spd=iae_spd+spd_err*dt
      if spd_err>max_spd_err then max_spd_err=spd_err end
      local vmin=(param:get("ARSPD_FBW_MIN")or 12)-STALL_MARG; local vmax=(param:get("ARSPD_FBW_MAX")or 28)+OVERSPEED_MARG
      if v_mps<vmin then abort_flag,abort_reason=true,"STALL" end; if v_mps>vmax then abort_flag,abort_reason=true,"OVERSPEED" end
    end
    local thr=get_throttle_norm()
    if thr then n_thr=n_thr+1; local d=thr-thr_mean; thr_mean=thr_mean+d/n_thr; thr_M2=thr_M2+d*(thr-thr_mean); clip_n=clip_n+1; if thr>=THR_CLIP_HI then clip_hi=clip_hi+1 end; if thr<=THR_CLIP_LO then clip_lo=clip_lo+1 end end
    local pr=get_pitch_rate_abs(); pr_n=pr_n+1; pr_sum2=pr_sum2+pr*pr
    if get_agl_m(base_alt_cm)<GROUND_AGL_ABORT_M then abort_flag,abort_reason=true,"GROUND_PROXIMITY" end
    if abort_flag or seg_timer>=SEG_DUR_S then
      if abort_flag then state=S_EVAL elseif state==S_SEG_A then state=S_START_SEG_B elseif state==S_SEG_B then state=S_START_SEG_C elseif state==S_SEG_C then state=S_EVAL end
    end
  -------------------------------------------------------------- EVAL (Q-LEARNING UPDATE)
  elseif state==S_EVAL then
    param:set("THR_MAX", original_thr_max); param:set("TECS_PITCH_MIN", original_pitch_min); param:set("AIRSPEED_CRUISE", base_cruise)
    local t_time=(pr_n*dt); local m_spd_err=(t_time>0) and (iae_spd/t_time)or 0; local m_alt_err=(t_time>0)and(iae_alt/t_time)or 0
    local var_thr=(n_thr>1)and(thr_M2/(n_thr-1))or 0; local rms_pr=(pr_n>0)and math.sqrt(pr_sum2/pr_n)or 0.0
    local clip_frac=(clip_n>0)and((clip_hi+clip_lo)/clip_n)or 0
    local penalty=0.0
    if abort_flag then penalty = penalty + BIG_PENALTY end
    if clip_frac > CLIP_FRAC_LIM then penalty = penalty + BIG_PENALTY * (clip_frac / CLIP_FRAC_LIM) end
    if max_alt_err > 8.0 then penalty = penalty + 300.0 end
    if max_spd_err > 5.0 then penalty = penalty + 200.0 end
    local R_track = -iae_spd - (iae_alt / 10.0); local R_smooth = -W1_VAR_THR*var_thr - W2_RMS_PR*rms_pr
    local R = R_track + R_smooth - penalty
    local b_spd=bin(m_spd_err,{0.5,1.5,3.0}); local b_alt=bin(m_alt_err,{1.0,3.0,6.0}); local b_osc=bin(rms_pr,{0.05,0.15,0.30})
    
    -- MODIFIED: Using a string key for the Q-table for better readability
    local s_next_key = string.format("%d-%d-%d", b_spd, b_alt, b_osc)
    
    local _,max_q=argmax_a(s_next_key); local old_q=qget(s_prev_key,a_prev)
    local new_q=old_q+ALPHA*(R+GAMMA*max_q-old_q); qset(s_prev_key,a_prev,new_q)
    s_prev_key=s_next_key
    
    if R>bestReward then bestReward,bestEP,bestParams=R,epi,{SPDW[idx_spdw],TCON[idx_tcon],PDMP[idx_pdmp]} end
    
    -- MODIFIED: Improved abort message printing
    local abort_msg = "None"
    if abort_flag then abort_msg = abort_reason or "Unknown" end
    
    gcs:send_text(0, string.format("EVAL EP %d | R:%.2f | Pen:%.1f | Abort:%s | MaxErr A:%.1f S:%.1f", epi, R, penalty, abort_msg, max_alt_err, max_spd_err))
    gcs:send_text(0, string.format("Best R: %.2f @ EP %d -> {S:%d, T:%.1f, P:%.1f}", bestReward, bestEP, bestParams[1] or -1, bestParams[2] or -1, bestParams[3] or -1))
    EPSI=math.max(EPS_MIN,EPSI*EPS_DEC); epi=epi+1; state=S_RESET
  -------------------------------------------------------------- RESET
  elseif state==S_RESET then
    if arming:is_armed() then arming:disarm() end
    if sim and sim.set_pose and home then sim:set_pose(0,home,Quaternion(),zero_vector3f(),zero_vector3f()) end
    state=S_IDLE
  end
  P_STATE:set(state)
end

------------------------------------------------------------------
-- SCRIPT INITIALIZATION
------------------------------------------------------------------
function loop() update(); return loop, 50 end
gcs:send_text(0, "TECS RL Script (Final v1.1) Loaded.")
return loop, 1000