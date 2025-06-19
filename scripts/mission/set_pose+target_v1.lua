-- SITL Episodic Reset v12 – timers via wait_s()
-- ---------------------------------------------------------------
local PKEY = 16
local PX   = "SIM_APOS_"

-- ——— parâmetros de missão ———
local TAKEOFF_ALT_M   = 10
local GOTO_FWD_N_M    = 15
local CLOSE_THRESH_M  = 1
local LOOP_FAST       = 50      -- ms
local LOOP_IDLE       = 500     -- ms

------------------------------------------------------------------
-- TIMERS
------------------------------------------------------------------
local timers = {}   -- [id] = deadline_ms
local function wait_s(sec, id)
    id = id or "_"
    local dl = timers[id]
    if not dl then
        dl = millis() + math.floor(sec*1000 + 0.5)
        timers[id] = dl
    end
    local now = millis()
    if now >= dl then
        timers[id] = nil
        return true, 0
    end
    return false, (dl - now) / 1000.0
end
local function reset_timers() for k in pairs(timers) do timers[k] = nil end end

------------------------------------------------------------------
-- PARAM BINDING
------------------------------------------------------------------
assert(param:add_table(PKEY, PX, 16), "param table err")
local function bind(name, idx, default)
    assert(param:add_param(PKEY, idx, name, default))
    return Parameter(PX .. name)
end
local P_ENABLE = bind("ENABLE", 1, 0)
local P_D_LAT  = bind("D_LAT",  2, -23.55629769)
local P_D_LNG  = bind("D_LNG",  3, -46.73008452)
local P_D_ALT  = bind("D_ALT",  4,  7.32)
local P_RLL    = bind("RLL",    8, 0)
local P_PIT    = bind("PIT",    9, 0)
local P_YAW    = bind("YAW",   10, 0)
local P_MODE   = bind("MODE",  14, -1)

------------------------------------------------------------------
-- STATE
------------------------------------------------------------------
-- fases: −1 wait-EKF | 0 arm | 1 decola | 2 goto | 3 espera-reset
local phase = -1
local init_loc, target_loc
local init_quat = Quaternion()
local takeoff_cmd_sent, home_reasserted = false, false
local MODE_GUIDED = 4

------------------------------------------------------------------
-- HELPERS
------------------------------------------------------------------
local function ekf_ready() return ahrs:get_origin() end
local function offset_north(loc, meters)
    local res = Location()
    res:lat(loc:lat() + math.floor((meters/111319.5) * 1e7 + 0.5))
    res:lng(loc:lng())
    res:alt(loc:alt())
    return res
end
-- altura relativa sempre positiva (m)
local function rel_alt_m()
    local here = ahrs:get_location()
    local home = ahrs:get_home()
    return (here and home) and math.abs(here:alt() - home:alt())/100 or 0
end

-- constrói destino (lat/lon fixos) + ALT = altitude MSL atual
local function build_dest_loc()
    local loc = Location()
    loc:lat(math.floor(P_D_LAT:get()*1e7 + 0.5))
    loc:lng(math.floor(P_D_LNG:get()*1e7 + 0.5))
    local here = ahrs:get_location()
    if here then                   -- usa MSL real
        loc:alt(here:alt())
    else                           -- fallback se ainda não houver EKF
        loc:alt(0)
    end
    return loc
end
local function reset_pose()
    local dest = build_dest_loc()
    sim:set_pose(0, dest, init_quat, Vector3f(), Vector3f())
    if arming:is_armed() then arming:disarm() end
    ahrs:set_home(dest)
    vehicle:set_target_location(dest)
end

------------------------------------------------------------------
-- MAIN UPDATE
------------------------------------------------------------------
local function update()
    if P_ENABLE:get() == 0 then return end

    -- −1 ▸ aguarda origem EKF
    if phase == -1 then
        if not ekf_ready() then
            if wait_s(2, "ekf_msg") then
                gcs:send_text(6, "Aguardando origem EKF…")
            end
            return
        end

        -- EKF pronto → SET_POSE inicial + HOME coerente
        init_quat:from_euler(math.rad(P_RLL:get()),
                             math.rad(P_PIT:get()),
                             math.rad(P_YAW:get()))
        init_loc = build_dest_loc()                -- já vem com alt MSL correto
        sim:set_pose(0, init_loc, init_quat, Vector3f(), Vector3f())
        ahrs:set_home(init_loc)                    -- home completo (lat/lon/alt)
        gcs:send_text(0, "SET_POSE + HOME OK")

        if P_MODE:get() >= 0 then vehicle:set_mode(P_MODE:get()) end
        phase = 0
        reset_timers()
        return
    end

    -- 0 ▸ ARM + GUIDED + re-home pós-arm
    if phase == 0 then
        if vehicle:get_mode() ~= MODE_GUIDED then
            vehicle:set_mode(MODE_GUIDED)
        end
        if not arming:is_armed() then
            arming:arm()
            timers["rehome"] = nil   -- zera timer da re-home
            return
        end

        -- 0,5 s pós-arm → regrava Home
        if not home_reasserted and wait_s(0.5, "rehome") then
            local here = ahrs:get_location()
            if here then init_loc:alt(here:alt()) end  -- sincronia de alt
            ahrs:set_home(init_loc)
            home_reasserted = true
            gcs:send_text(6, "Home re-set pós-arm ✔")
        end

        -- envia comando de decolagem 3 s após re-home
        if home_reasserted and not takeoff_cmd_sent then
            if wait_s(3, "toff_delay") then
                vehicle:start_takeoff(TAKEOFF_ALT_M)
                takeoff_cmd_sent = true
                gcs:send_text(6, "Decolagem iniciada, fase 1")
            end
            return
        end
        if takeoff_cmd_sent then phase = 1 end
        return
    end

    -- 1 ▸ sobe até ~80 % ou 6 s
    if phase == 1 then
        local alt = rel_alt_m()
        if wait_s(1, "msg_take") then
            gcs:send_text(6, string.format("Subindo %.1fm…", alt))
        end
        if alt >= TAKEOFF_ALT_M*0.8 or wait_s(6, "to_timeout") then
            local here = ahrs:get_location()
            target_loc = offset_north(here, GOTO_FWD_N_M)
            target_loc:alt(here:alt())
            vehicle:set_target_location(target_loc)
            gcs:send_text(6, "Goto 15 m à frente")
            phase = 2
        end
        return
    end

    -- 2 ▸ navega, depois reset
    if phase == 2 then
        local dist = ahrs:get_location():get_distance(target_loc)
        if wait_s(1, "msg_nav") then
            gcs:send_text(6, string.format("Dist %.2fm", dist))
        end
        if dist <= CLOSE_THRESH_M then
            gcs:send_text(6, "Destino alcançado → reset pose + 10 s")
            reset_pose()
            reset_timers()
            home_reasserted, takeoff_cmd_sent = false, false
            phase = 3
        end
        return
    end

    -- 3 ▸ espera 10 s e recomeça
    if phase == 3 then
        if not wait_s(10, "ep_wait") then
            if wait_s(2, "msg_ep") then
                gcs:send_text(6, "Reiniciando em breve…")
            end
            return
        end
        gcs:send_text(6, "Reiniciando episódio…")
        phase = -1
        reset_timers()
        return
    end
end

------------------------------------------------------------------
-- SCHEDULER
------------------------------------------------------------------
local function loop()
    if P_ENABLE:get() == 0 then return loop, LOOP_IDLE end
    update()
    return loop, LOOP_FAST
end

gcs:send_text(0,
  "episodic-reset v12 carregado → set SIM_APOS_ENABLE=1 para rodar")
return loop, 1000
