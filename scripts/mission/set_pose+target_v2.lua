------------------------------------------------------------------
-- PARAMETER-TABLE HANDLING
------------------------------------------------------------------
local PX            = "SIM_ERES_"   -- parameter table prefix
local DEFAULT_PKEY  = 17            -- preferred table slot (may be taken)
local TABLE_LEN     = 16            -- number of params we actually create
local PKEY = DEFAULT_PKEY           -- may be overwritten below
gcs:send_text(6, string.format("Começando script"))
local function ensure_param_table()
    if param:add_table(PKEY, PX, TABLE_LEN) then return end
    for id = 0, 63 do
        if param:add_table(id, PX, TABLE_LEN) then
            PKEY = id
            gcs:send_text(6, string.format("Param-table %d busy, using %d", DEFAULT_PKEY, id))
            return
        end
    end
    error("No free parameter-table slots (0-63)")
end
ensure_param_table()

------------------------------------------------------------------
-- CONSTANTS
------------------------------------------------------------------
local TAKEOFF_ALT_M   = 10    -- take-off altitude (m AGL)
local GOTO_FWD_N_M    = 15    -- north offset for waypoint (m)
local GOTO_FWD_E_M    = 15    -- east  offset for waypoint (m)
local CLOSE_THRESH_M  = 1     -- arrival tolerance (m)
local LOOP_FAST       = 50    -- loop when ENABLE=1 (ms)
local LOOP_IDLE       = 500   -- loop when ENABLE=0 (ms)

------------------------------------------------------------------
-- TIMER HELPERS
------------------------------------------------------------------
local timers = {}
local function wait_s(sec, id)
    id = id or "_"
    local dl = timers[id]
    if not dl then
        dl = millis() + math.floor(sec*1000 + 0.5)
        timers[id] = dl
    end
    local now = millis()
    if now >= dl then timers[id] = nil; return true, 0 end
    return false, (dl - now)/1000
end
local function reset_timers() for k in pairs(timers) do timers[k] = nil end end

------------------------------------------------------------------
-- PARAM BINDING
------------------------------------------------------------------
local function bind(name, idx, default)
    assert(param:add_param(PKEY, idx, name, default))
    return Parameter(PX .. name)
end

local P_ENABLE = bind("ENABLE", 1, 1)
local P_H_LAT  = bind("H_LAT",  2, -35.36299690)
local P_H_LNG  = bind("H_LNG",  3, 149.16517990)
local P_RLL    = bind("RLL",    4, 0)
local P_PIT    = bind("PIT",    5, 0)
local P_YAW    = bind("YAW",    6, 0)
local P_MODE   = bind("MODE",   7, -1) -- optional flight-mode override

------------------------------------------------------------------
-- OPTIONAL: silence noisy checks (SITL only)
------------------------------------------------------------------
param:set("ARMING_CHECK", 0)
param:set("GPS_TYPE", 0)
param:set("GPS_AUTO_CONFIG", 0)
param:set("GPS_AUTO_SWITCH", 0)

------------------------------------------------------------------
-- STATE VARIABLES
------------------------------------------------------------------
local phase = -1              -- -1 wait-EKF | 0 arm | 1 climb | 2 goto | 3 cooldown
local init_loc, target_loc
local init_quat = Quaternion()
local takeoff_cmd_sent, home_reasserted = false, false
local MODE_GUIDED = 4
local ZERO_VEC = Vector3f()

------------------------------------------------------------------
-- HELPER FUNCTIONS
------------------------------------------------------------------
local function ekf_ready() return ahrs:get_origin() end

local function offset_north(loc, m)
    local res = Location()
    res:lat( loc:lat() + math.floor((m/111319.5)*1e7 + 0.5) )
    res:lng( loc:lng() )
    res:alt( loc:alt() )
    return res
end

local function offset_east(loc, m)
    local lat_deg = loc:lat()/1e7
    local meters_per_deg = 111319.5 * math.cos(math.rad(lat_deg))
    local res = Location()
    res:lat( loc:lat() )
    res:lng( loc:lng() + math.floor((m/meters_per_deg)*1e7 + 0.5) )
    res:alt( loc:alt() )
    return res
end

local function rel_alt_m()
    local here, home = ahrs:get_location(), ahrs:get_home()
    return (here and home) and math.abs(here:alt() - home:alt())/100 or 0
end

local function build_loc(lat_deg, lng_deg)
    local loc = Location()
    loc:lat( math.floor(lat_deg*1e7 + 0.5) )
    loc:lng( math.floor(lng_deg*1e7 + 0.5) )
    loc:alt( ahrs:get_location() and ahrs:get_location():alt() or 0 )
    return loc
end

--- RESET EKF + TELEPORT ----------------------------------------------
local function reset_pose()
    gcs:send_text(6, string.format("Reset_pose"))
    if arming:is_armed() then
        arming:disarm()
    end

    -- 1) hard-reset EKF
    if ahrs.reset then
        ahrs:reset()
    end

    -- 2) DEFINE NEW ORIGIN *before* moving the sim
    ahrs:set_origin(init_loc)

    -- 3) teleport to absolute coordinates
    sim:set_pose(0, init_loc, init_quat, ZERO_VEC, ZERO_VEC)

    -- 4) keep Home and stop any residual motion
    ahrs:set_home(init_loc)
    vehicle:set_target_location(init_loc)
    vehicle:set_target_velocity_NED(ZERO_VEC)
end


------------------------------------------------------------------
-- MAIN UPDATE
------------------------------------------------------------------
local function update()
    if P_ENABLE:get() == 0 then return end

    -- -1 ▸ wait EKF origin
    if phase == -1 then
        if not ekf_ready() then
            if wait_s(2, "ekf_msg") then gcs:send_text(6, "Waiting for EKF origin…") end
            return
        end
        -- Initial pose & home
        init_quat:from_euler(math.rad(P_RLL:get()), math.rad(P_PIT:get()), math.rad(P_YAW:get()))
        init_loc = build_loc(P_H_LAT:get(), P_H_LNG:get())
        reset_pose()
        ahrs:set_home(init_loc)
        vehicle:set_target_location(init_loc)
        vehicle:set_target_velocity_NED(ZERO_VEC)
        gcs:send_text(0, "SET_POSE + HOME set (episode start)")
        if P_MODE:get() >= 0 then vehicle:set_mode(P_MODE:get()) end
        phase = 0; reset_timers(); return
    end

    -- 0 ▸ arm & take-off
    if phase == 0 then
        if vehicle:get_mode() ~= MODE_GUIDED then vehicle:set_mode(MODE_GUIDED) end
        if not arming:is_armed() then arming:arm(); timers["rehome"] = nil; return end
        if not home_reasserted and wait_s(0.5, "rehome") then
            local here = ahrs:get_location(); if here then init_loc:alt(here:alt()) end
            ahrs:set_home(init_loc); home_reasserted = true; gcs:send_text(6, "Home reset after arm")
        end
        if home_reasserted and not takeoff_cmd_sent then
            if wait_s(3, "toff_delay") then
                vehicle:start_takeoff(TAKEOFF_ALT_M)
                takeoff_cmd_sent = true
                gcs:send_text(6, "Take-off initiated")
            end
            return
        end
        if takeoff_cmd_sent then phase = 1 end; return
    end

    -- 1 ▸ climb then set waypoint
    if phase == 1 then
        local alt = rel_alt_m()
        if wait_s(1, "msg_take") then gcs:send_text(6, string.format("Ascending %.1f m", alt)) end
        if alt >= TAKEOFF_ALT_M*0.8 or wait_s(6, "climb_to") then
            local here = ahrs:get_location()
            target_loc = offset_east( offset_north(here, GOTO_FWD_N_M), GOTO_FWD_E_M )
            target_loc:alt(here:alt())
            vehicle:set_target_location(target_loc)
            gcs:send_text(6, "Goto 15 m ahead")
            phase = 2
        return
    end
end

    -- 2 ▸ navigate, then reset
    if phase == 2 then
        local dist = ahrs:get_location():get_distance(target_loc)
        if wait_s(1, "msg_nav") then gcs:send_text(6, string.format("Dist %.2f m", dist)) end
        if dist <= CLOSE_THRESH_M then
            gcs:send_text(6, "Reached -> resetting EKF & pose…")
            reset_pose()
            reset_timers()
            home_reasserted, takeoff_cmd_sent = false, false
            phase = 3
        end; return
    end

    -- 3 ▸ cooldown then restart (EKF will re-initialise during esta fase)
    if phase == 3 then
        if not wait_s(10, "ep_wait") then
            if wait_s(2, "msg_ep") then gcs:send_text(6, "Restarting…") end
            return
        end
        phase = -1; reset_timers(); return
    end
end

------------------------------------------------------------------
-- SCHEDULER
------------------------------------------------------------------
local function loop()
    if P_ENABLE:get() == 0 then return loop, LOOP_IDLE end
    update(); return loop, LOOP_FAST
end

gcs:send_text(0, "episodic-reset v12 loaded - set SIM_ERES_ENABLE=1 to run")
return loop, 1000
