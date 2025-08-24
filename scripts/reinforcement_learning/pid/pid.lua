--[[--------------------------------------------------------------------
    RL Episodic-Reset Script for Altitude Hold PID Tuning
    -- This script uses Q-learning to automatically tune the vertical
       position controller parameters (PSC_ACCZ_P/I/D) to minimize
       altitude error during a hover test.
------------------------------------------------------------------------]]

------------------------------------------------------------------
-- PARAMETER-TABLE DEFINITIONS
------------------------------------------------------------------
local PARAM_TABLE_KEY    = 95
local PARAM_TABLE_PREFIX = "SIM_PID_"
local TABLE_LEN          = 5

assert(param:add_table(PARAM_TABLE_KEY, PARAM_TABLE_PREFIX, TABLE_LEN),
       "failed to create parameter table")

-- Function to bind parameters
function bind_add_param(name, idx, default_value)
    assert(param:add_param(PARAM_TABLE_KEY, idx, name, default_value),
           string.format('could not add param %s', name))
    return Parameter(PARAM_TABLE_PREFIX .. name)
end

-- User-configurable parameters
local P_ENABLE       = bind_add_param("ENABLE",       1, 1)    -- 0 = idle, 1 = run
local P_TRGT_ALT     = bind_add_param("TRGT_ALT",   2, 10)   -- Target hover altitude (m)
local P_HVR_TIME     = bind_add_param("HVR_TIME",   3, 15)   -- Duration of each hover test (s)
local P_MAX_EP       = bind_add_param("MAX_EP", 4, 1000)  -- Stop after this many episodes
local P_STATE        = bind_add_param("STATE",        5, -1)   -- FSM state for monitoring

-- SITL-specific setup
param:set("ARMING_CHECK",     0)
param:set("GPS_TYPE",         0)
param:set("AHRS_EKF_TYPE",   10)
param:set("RTL_TYPE", 0) -- Disable SmartRTL to save memory
param:set("SIM_SPEEDUP",   1000)

------------------------------------------------------------------
-- SCRIPT CONSTANTS
------------------------------------------------------------------
local LOOP_FAST       = 100    -- Active loop interval (ms). Slower is fine for this task.
local LOOP_IDLE       = 500    -- Idle loop interval (ms)
local TARGET_THRESH_M = 0.5    -- Arrival tolerance for initial altitude target (m)

------------------------------------------------------------------
-- FSM STATES
------------------------------------------------------------------
local STATE_WAIT      = -1
local STATE_INIT      = 0
local STATE_TAKEOFF   = 1
local STATE_HOVER     = 2
local STATE_EVALUATE  = 3
local STATE_RESET     = 4

------------------------------------------------------------------
-- RL CONFIGURATION
------------------------------------------------------------------
-- Q-learning parameters
local ALPHA           = 0.1  -- Learning rate
local GAMMA           = 0.9  -- Discount factor
local INITIAL_EPSILON = 0.5  -- Initial exploration rate
local EPSILON_DECAY   = 0.99 -- Decay rate for exploration
local MIN_EPSILON     = 0.01 -- Minimum exploration rate
local epsilon         = INITIAL_EPSILON

-- State discretization
-- We define the state by altitude error and the rate of change of that error (velocity)
local ALT_ERROR_BINS = {-2, -1, -0.5, -0.1, 0.1, 0.5, 1, 2} -- meters
local VEL_ERROR_BINS = {-1, -0.5, -0.1, 0.1, 0.5, 1}       -- m/s

-- Action definition
-- Define discrete actions for modifying P, I, and D parameters
local actions = {
    {param="PSC_ACCZ_P", change=0.02},   -- 1: Increase P
    {param="PSC_ACCZ_P", change=-0.02},  -- 2: Decrease P
    {param="PSC_ACCZ_I", change=0.02},   -- 3: Increase I
    {param="PSC_ACCZ_I", change=-0.02},  -- 4: Decrease I
    {param="PSC_ACCZ_D", change=0.0002}, -- 5: Increase D
    {param="PSC_ACCZ_D", change=-0.0002},-- 6: Decrease D
    {param=nil, change=0}               -- 7: Do nothing
}
local numActions = #actions
local Q = {} -- Q-table: Q[state_key][action_idx] = value

------------------------------------------------------------------
-- STATE & LOGGING VARIABLES
------------------------------------------------------------------
local state           = STATE_WAIT
local home_loc        = nil
local base_att        = Quaternion()
local epCount         = 1
local bestMSE         = math.huge
local bestParams      = {}
local hover_start_time = 0
local altitude_errors = {} -- To store errors during hover phase
local log_file        = nil -- Handle for the log file

------------------------------------------------------------------
-- HELPERS
------------------------------------------------------------------

-- Discretize a continuous value into a bin index
function get_bin(value, bins)
    for i = 1, #bins do
        if value < bins[i] then
            return i
        end
    end
    return #bins + 1
end

-- Get the current state key based on altitude error and vertical velocity
function get_current_state_key()
    local current_alt = (ahrs:get_location():alt() - home_loc:alt()) / 100
    local alt_error = P_TRGT_ALT:get() - current_alt
    -- Corrected method to get vertical velocity (NED frame)
    local vel_vec = ahrs:get_velocity_NED()
    local vert_vel = 0
    if vel_vec then
        -- Corrected accessor for Vector3f component: use :z() not .z
        vert_vel = vel_vec:z() * -1 -- Invert Down to get Up velocity
    end
    
    local alt_bin = get_bin(alt_error, ALT_ERROR_BINS)
    local vel_bin = get_bin(vert_vel, VEL_ERROR_BINS)
    
    -- Combine bins into a unique key
    return "alt" .. alt_bin .. "_vel" .. vel_bin
end

-- Choose an action using epsilon-greedy strategy
function choose_action(state_key)
    if math.random() < epsilon then
        -- Exploration: choose a random action
        return math.random(numActions)
    else
        -- Exploitation: choose the best known action for this state
        local bestAction, bestVal = 1, -1e9
        local q_actions = Q[state_key] or {}
        for i = 1, numActions do
            local val = q_actions[i] or 0
            if val > bestVal then
                bestVal = val
                bestAction = i
            end
        end
        return bestAction
    end
end

-- Apply the chosen action to the corresponding parameter
function apply_action(action_idx, state_key)
    local action = actions[action_idx]
    if action.param then
        local p = param:get(action.param)
        local newValue = p + action.change
        -- Add constraints to prevent extreme values
        if newValue > 0 then
            param:set(action.param, newValue)
            gcs:send_text(0, string.format("EP %d: Action %d -> Set %s to %.4f", epCount, action_idx, action.param, newValue))
        else
            gcs:send_text(0, string.format("EP %d: Action %d -> %s change ignored (would be <= 0)", epCount, action_idx, action.param))
            -- **FIX**: Penalize the invalid action directly in the Q-table to prevent getting stuck.
            Q[state_key] = Q[state_key] or {}
            Q[state_key][action_idx] = -1e9 -- Assign a very large negative value
            gcs:send_text(0, string.format("Penalizing invalid action for state %s", state_key))
        end
    else
        gcs:send_text(0, string.format("EP %d: Action %d -> No change", epCount, action_idx))
    end
end

-- Calculate Mean Squared Error from a table of errors
function calculate_mse(errors)
    if #errors == 0 then return math.huge end
    local sum_sq = 0
    for _, err in ipairs(errors) do
        sum_sq = sum_sq + (err * err)
    end
    return sum_sq / #errors
end

------------------------------------------------------------------
-- MAIN UPDATE LOOP
------------------------------------------------------------------
function update()
    if P_ENABLE:get() == 0 then
        state = STATE_WAIT
        return LOOP_IDLE
    end

    if state == STATE_WAIT then
        if ahrs:get_origin() then
            home_loc = ahrs:get_home() or ahrs:get_location()
            base_att = ahrs:get_quaternion()
            state = STATE_INIT
        end

    elseif state == STATE_INIT then
        -- Open log file only once at the start of the script run
        if not log_file then
            log_file = io.open("logs/pid_log.csv", "w")
            if log_file then
                log_file:write("EP,Reward,MSE,P,I,D\n") -- CSV Header
                gcs:send_text(0, "Log file opened: logs/pid_log.csv")
            else
                gcs:send_text(0, "ERROR: Failed to open log file!")
            end
        end

        gcs:send_text(0, "------------------------------------")
        gcs:send_text(0, string.format("Starting Episode %d / %d", epCount, P_MAX_EP:get()))
        
        -- Choose and apply an action based on the initial ground state
        local initial_state_key = get_current_state_key()
        local action_to_take = choose_action(initial_state_key)
        -- Pass the state key to apply_action so it can penalize if necessary
        apply_action(action_to_take, initial_state_key)
        
        -- Store the state and action for the Q-table update later
        last_state_key = initial_state_key
        last_action_idx = action_to_take

        -- Reset for new episode
        altitude_errors = {}
        vehicle:set_mode(4) -- GUIDED
        state = STATE_TAKEOFF
        
    elseif state == STATE_TAKEOFF then
        if not arming:is_armed() then
            arming:arm()
        else
            -- Use a non-blocking takeoff command
            if not vehicle:get_likely_flying() then
                vehicle:start_takeoff(P_TRGT_ALT:get())
            end
            local current_alt = (ahrs:get_location():alt() - home_loc:alt()) / 100
            if current_alt >= P_TRGT_ALT:get() - TARGET_THRESH_M then
                gcs:send_text(0, "Reached target altitude, starting hover test.")
                hover_start_time = tonumber(tostring(millis()))
                state = STATE_HOVER
            end
        end

    elseif state == STATE_HOVER then
        local elapsed = (tonumber(tostring(millis())) - hover_start_time) / 1000
        if elapsed < P_HVR_TIME:get() then
            local current_alt = (ahrs:get_location():alt() - home_loc:alt()) / 100
            local alt_error = P_TRGT_ALT:get() - current_alt
            table.insert(altitude_errors, alt_error)
        else
            gcs:send_text(0, "Hover test finished.")
            state = STATE_EVALUATE
        end

    elseif state == STATE_EVALUATE then
        local mse = calculate_mse(altitude_errors)
        -- Reward is inversely proportional to MSE. Add a small number to avoid division by zero.
        local reward = 1 / (mse + 0.001)

        gcs:send_text(0, string.format("Episode %d finished. MSE: %.4f, Reward: %.4f", epCount, mse, reward))

        -- Log the results to the CSV file
        if log_file then
            local p_val = param:get("PSC_ACCZ_P")
            local i_val = param:get("PSC_ACCZ_I")
            local d_val = param:get("PSC_ACCZ_D")
            local log_line = string.format("%d,%.4f,%.4f,%.4f,%.4f,%.4f\n", epCount, reward, mse, p_val, i_val, d_val)
            log_file:write(log_line)
            log_file:flush() -- Ensure data is written to disk immediately
        end

        -- Q-learning update
        local oldQ = (Q[last_state_key] and Q[last_state_key][last_action_idx]) or 0
        -- For this problem, we can simplify: the "future Q" is 0 as each episode is terminal.
        local newQ = oldQ + ALPHA * (reward - oldQ)
        
        Q[last_state_key] = Q[last_state_key] or {}
        Q[last_state_key][last_action_idx] = newQ
        
        -- Track best performance
        if mse < bestMSE then
            bestMSE = mse
            bestParams.P = param:get("PSC_ACCZ_P")
            bestParams.I = param:get("PSC_ACCZ_I")
            bestParams.D = param:get("PSC_ACCZ_D")
            gcs:send_text(0, string.format("NEW BEST PARAMS FOUND! P:%.4f, I:%.4f, D:%.4f (MSE: %.4f)",
                bestParams.P, bestParams.I, bestParams.D, bestMSE))
        end
        
        -- Decay epsilon for less exploration over time
        epsilon = math.max(MIN_EPSILON, epsilon * EPSILON_DECAY)
        gcs:send_text(0, string.format("Epsilon decayed to: %.4f", epsilon))
        
        state = STATE_RESET

    elseif state == STATE_RESET then
        -- Land the vehicle
        vehicle:set_mode(9) -- LAND
        if not vehicle:get_likely_flying() then
            epCount = epCount + 1
            if epCount > P_MAX_EP:get() then
                gcs:send_text(0, "Max episodes reached. Stopping script.")
                gcs:send_text(0, string.format("Final Best Params: P:%.4f, I:%.4f, D:%.4f (MSE: %.4f)",
                    bestParams.P, bestParams.I, bestParams.D, bestMSE))
                P_ENABLE:set(0)
                state = STATE_WAIT
            else
                -- Teleport for a faster reset
                sim:set_pose(0, home_loc, base_att, Vector3f(), Vector3f())
                if arming:is_armed() then arming:disarm() end
                state = STATE_INIT
            end
        end
    end

    P_STATE:set(state)
    return LOOP_FAST
end

-- This is the main script entry point
function loop()
    if P_ENABLE:get() == 0 then
        if log_file then
            log_file:close()
            log_file = nil
            gcs:send_text(0, "Log file closed.")
        end
        return loop, LOOP_IDLE
    end
    update()
    return loop, LOOP_FAST
end

gcs:send_text(0, "Loaded RL Altitude PID Tuning script!")
return loop, 1000
