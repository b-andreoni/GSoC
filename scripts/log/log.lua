--[[------------------------------------------------------------------
  flight_logger.lua  ·  v1.2
  Loga telemetria (posição, atitude), estado da FSM, tensão e corrente
  Testado em Copter 4.5 SITL.
--------------------------------------------------------------------]]

------------------------------------------------------------------
-- CONFIGURAÇÃO
------------------------------------------------------------------
local LOG_PERIOD_MS = 100      -- 10 Hz
local FMT           = "%.7f"   -- casas decimais p/ lat & lon

-- variáveis preenchidas na 1.ª execução do loop
local filename = nil
local file     = nil
local header =
  "epoch_ms,state,mode,"
.. "lat_deg,lng_deg,alt_m,"
.. "velN,velE,velD,"
.. "roll,pitch,yaw,"
.. "gyroX,gyroY,gyroZ,"
.. "volt_V,curr_A\n"

------------------------------------------------------------------
-- FUNÇÃO AUXILIAR PARA BATERIA
------------------------------------------------------------------
local function batt_readings()
    local v, a = 0, 0
    if battery and battery.voltage then
        v = battery:voltage(0) or 0
    end
    if battery and battery.current_amps then
        a = battery:current_amps(0) or 0
    end
    return v, a
end

------------------------------------------------------------------
-- VARIÁVEIS INTERNAS
------------------------------------------------------------------
local last_log_t = 0
local prev_state = -999

------------------------------------------------------------------
-- LOOP PRINCIPAL
------------------------------------------------------------------
function logger_update()
    -- cria o arquivo SÓ na 1.ª chamada (millis() já é válido aqui)
    if not file then
        local ms = tonumber(millis())              -- uint32_t → number
        filename = string.format("flight_log_%d.csv",
                                 math.floor(ms / 1000))
        file = assert(io.open(filename, "w"),
                      "não consegui criar "..filename)
        file:write(header)
        file:flush()
        gcs:send_text(0, "Flight logger ON → "..filename)
    end

    local now = millis()

    -- faz log a cada LOG_PERIOD_MS
    if now - last_log_t >= LOG_PERIOD_MS then
        last_log_t = now

        -- leituras
        local loc   = ahrs:get_location() or Location()
        local vel   = ahrs:get_velocity_NED() or Vector3f()
        local quat  = ahrs:get_quaternion() or Quaternion()
        local gyro  = ahrs:get_gyro() or Vector3f()
        local eul   = quat:to_euler()
        local state = param:get("SIM_ERES_STATE") or -1

        -- aviso de mudança de estado
        if state ~= prev_state then
            gcs:send_text(6, string.format("STATE → %d", state))
            prev_state = state
        end

        -- bateria
        local volt, curr = batt_readings()

        -- grava CSV
        file:write(string.format(
            "%d,%d,%d,"            -- epoch_ms,state,mode
          ..FMT..","..FMT..",%.2f," -- lat,lng,alt
          .."%.2f,%.2f,%.2f,"      -- velN,E,D
          .."%.2f,%.2f,%.2f,"      -- roll,pitch,yaw
          .."%.3f,%.3f,%.3f,"      -- gyroX,Y,Z
          .."%.2f,%.2f\n",         -- volt,curr
            now, state, vehicle:get_mode(),
            loc:lat()/1e7, loc:lng()/1e7, loc:alt()/100,
            vel:x(), vel:y(), vel:z(),
            math.deg(eul:x()), math.deg(eul:y()), math.deg(eul:z()),
            gyro:x(), gyro:y(), gyro:z(),
            volt, curr))
        file:flush()
    end

    return logger_update, 50   -- executa a cada 50 ms
end

return logger_update, 100
