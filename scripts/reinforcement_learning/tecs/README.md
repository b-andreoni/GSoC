# TECS Tuning Agents

This directory contains "toy examples" for tuning the Total Energy Control System (TECS) using Reinforcement Learning. The primary example, `combo.lua`, is a powerful script for joint optimization of multiple parameters and serves as an adaptable template.

### 1. The Problem: Complex Energy Management

The Total Energy Control System (TECS) is a sophisticated controller used in fixed-wing aircraft to manage airspeed and altitude simultaneously by trading potential energy (altitude) for kinetic energy (airspeed). Its numerous interdependent parameters make it one of the most challenging parts of the autopilot to tune manually. Incorrect tuning leads to poor climb performance, unstable altitude holds, or inefficient flight.

### 2. Why Solve It: The Value of Optimal Performance

A well-tuned TECS is critical for fixed-wing missions:
* **Flight Efficiency:** Ensures the aircraft flies at the most efficient airspeed and throttle setting, maximizing endurance and range.
* **Mission Accuracy:** Precise altitude and speed control is essential for tasks like aerial surveying.
* **Smooth Control:** Provides a smooth, reliable response, reducing stress on the airframe.

### 3. How It's Solved: A Q-Learning Toy Example

We frame TECS tuning as a game where a **Q-learning** agent discovers the best parameter combination by repeatedly flying a dynamic test maneuver in simulation.

* **Concept:** The agent runs many "episodes." In each episode, it applies a new set of TECS parameters and flies a challenging "pitch cresting" maneuver. It gets a "reward" based on its performance, and over time, it learns which parameter changes lead to the highest reward.
* **The "Brain" (Q-Table):** The agent's memory is a Q-table, which stores the expected long-term reward for taking a specific action (changing a parameter) given the current state (the aircraft's performance).

### 4. Anatomy of the `combo.lua` Example

This script teaches a Plane to find the optimal combination of `TECS_SPDWEIGHT`, `TECS_TIME_CONST`, and `TECS_PTCH_DAMP`.

* **The Test Maneuver:** Instead of a simple loiter, each episode consists of a dynamic **"pitch cresting"** maneuver. The aircraft is commanded to climb steeply and then descend steeply. This is designed to stress the controller and clearly reveal how well the current TECS parameters handle large energy changes.

* **RL Concepts:**
    * **State:** The performance of the aircraft during the test maneuver, summarized into discrete "bins" for:
        1.  Average airspeed error.
        2.  Average altitude error.
        3.  Pitch rate oscillation (RMS of gyro Y).
    * **Action:** A single, small adjustment to one of the three TECS parameters. The agent can choose to increase or decrease one parameter, or do nothing (`NoChange`).
    * **Reward:** A sophisticated reward function designed to encourage ideal flight characteristics. The final reward is a sum of:
        * **Negative Tracking Error:** Penalizes deviations from the target airspeed and altitude (`-IAE_speed`, `-IAE_alt`).
        * **Negative Control Effort:** Penalizes unsmooth flight, measured by throttle fluctuations and pitch rate oscillations.
        * **Large Penalties:** Applies a very large negative reward if the aircraft enters an unsafe state (e.g., stalls, overspeeds, gets too close to the ground, or has its throttle saturated for too long). This teaches the agent to prioritize safety above all else.

### 5. How to Adapt This Example for Your Aircraft

This script can be adapted for different fixed-wing airframes by modifying its core components.

#### Step 1: Adjust Aircraft Physics & Limits
In the `SETUP` section, change the simulation and airspeed parameters to match your aircraft's performance envelope.
```lua
param:set("ARSPD_FBW_MIN", 15) -- Set to your stall speed + margin
param:set("ARSPD_FBW_MAX", 35) -- Set to your VNE
param:set("AIRSPEED_CRUISE", 22) -- Set your typical cruise speed
```

#### Step 2: Tune the Test Maneuver
The intensity of the "pitch cresting" maneuver can be adjusted for different aircraft dynamics.
```lua
local TEST_DUR_S = 60.0 -- Increase for slower, less agile aircraft
local PITCH_CREST_DH_M = 30 -- Decrease for aircraft with lower climb rates
```

#### Step 3: Refine the Reward Function
You can change the agent's priorities by adjusting the reward weights. For example, if you care more about a smooth flight than perfect altitude tracking, you can increase the weights for smoothness.
```lua
-- Reward weights
local W1_VAR_THR = 1.0 -- Increase to penalize throttle changes more
local W2_RMS_PR  = 2.0 -- Increase to penalize pitch oscillations more
```

### 6. Running the Example

1.  **Prerequisites:** Ensure you have a working ArduPilot SITL build environment for ArduPlane.
2.  **Load the Script:** Place the script in your `scripts` directory and launch SITL.
    ```bash
    sim_vehicle.py -v ArduPlane --console --map
    script reinforcement_learning/tecs/combo.lua
    ```
3.  **Enable:** Set the `RL_ENABLE` parameter to `1` to start the learning process.
4.  **Monitor:** Watch the GCS console. The script will output the chosen parameters for each episode, the final reward, and the best parameters found so far.
