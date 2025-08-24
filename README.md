# Google Summer of Code 2025: SITL AI Reinforcement Learning Concept Script

![GSoC banner](docs/images/GSoC-Banner.png)

> **A Lua script that lets ArduPilot learn to tune itself, live, within SITL.**

**Mentors:** Nate Mailhot & Sanket Sharma  |  **Timeframe:** May → Sep 2025  |  **Project:** [GSoC](https://summerofcode.withgoogle.com/programs/2025/projects/w7EYZSIz)

## 1. Introduction 📣

Hello ArduPilot community! I'm Bruno Andreoni Sarmento, a 4th‑year student at the Polytechnic School of the University of São Paulo (USP), pursuing a bachelor’s degree in Electrical Engineering. For the past two years I’ve been a member of Skyrats (USP’s autonomous‑drone team), where I’ve worked with many open‑source tools, ArduPilot being our core flight‑control framework.

I've been inspired by other team members that took part in GSoC projects, and it was a dream come true being selected for this awesome project!

## 2. Problem Statement ❗

Today, UAV parameter tuning still relies on manual adjustments, often requiring trial‑and‑error sweeps and test flights. This approach is:

* Time‑consuming and inefficient;
* Poorly reproducible;
* Not scalable for adaptive or autonomous systems.

Meanwhile, **Reinforcement Learning (RL)** offers a principled method for adaptive control, but the **ArduPilot SITL environment lacks built‑in support for episodic interactions**, which are essential for reliable training and evaluation.

## 3. Project Proposal 🚀

> This project builds a lightweight Lua‑based scripting layer on top of the existing SITL engine to enable episodic RL interactions.

![RL Illustrated](docs/images/rl.jpg)

The project introduces:

* **Episodic Replay** – a structured way to test and evaluate UAV behavior across multiple runs, starting from known or randomized states;
* **State Reset Mechanism** – using `sim:set_pose()` in Lua scripts to programmatically reset UAV position, velocity, and attitude between episodes;
* **Metric Logging** – automated telemetry collection (e.g., tracking error, convergence time) to evaluate each episode’s performance;
* **Offline RL Phase (baseline)** – log rich episodes and train an initial Q‑table/value‑function *offline* before any in‑place updates;
* **Online RL Phase (final goal)** – once the baseline is set, the same agent keeps learning *live*, updating parameters in real time during simulation.

This project builds the necessary infrastructure for **online learning directly in SITL**, unlocking new workflows for intelligent parameter tuning and experimental reinforcement learning within the ArduPilot ecosystem.

## 4. Solution Diagram 🖼️

![Solution diagram](docs/images/diagram.png)

### 4.1 Block‑by‑Block Breakdown 🔍

| Block                              | Role                                                                                                                                                                                                                                                    | Key ArduPilot / Lua APIs                                       |
| ---------------------------------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- | -------------------------------------------------------------- |
| **Episode Manager**                | Detects episode termination (e.g. time‑out, crash, success) and fires a reset.                                                                                                                                                                          | `time:millis()`, custom state checks                           |
| **Teleport / Reset**               | Instantaneously re‑positions vehicle & zeros attitude/vel using **`sim:set_pose()`**.                                                                                                                                                                   | `sim:set_pose(instance, loc, orient, vel_bf, gyro)`            |
| **Metrics Collector**              | Computes reward signals (tracking error, overshoot, energy) every tick.                                                                                                                                                                                 | `ahrs:get_pitch()`, `vehicle:get_target_alt()`, custom math    |
| **Reinforcement Learning**                      | Updates Q‑table / value function in‑place at ≈ 10 Hz; selects next action. During the **Offline RL Phase** it runs in *log‑only* mode to gather state–action–reward traces for baseline training, switching to live updates in the **Online RL Phase**. | Pure Lua tables for Q‑table; math lib                          |
| **Policy Executor**                | Applies action by tweaking params or sending direct control:                                                                                                                                                                                            | `param:set("ATC_RAT_PIT_P", val)`, `vehicle:set_target_roll()` |
| **Direct State Feed (EKF Opt 10)** | Supplies fused state to Lua via binding helpers (`ahrs`, `ins`, etc.).                                                                                                                                                                                  | Firmware internal                                              |
| **Sensor Simulation**              | Generates IMU/GPS from physics backend (Gazebo or built‑in).                                                                                                                                                                                            | SITL C++                                                       |

## 5. How the Loop Runs 🔄

1. **Reset** → Script arms vehicle, immediately calls `sim:set_pose()` to starting state.
2. **Episode** → RL chooses an action (e.g., ±5 % on `ATC_RAT_RLL_P`).
3. **Flight** → Controller responds; Metrics Collector accumulates reward.
4. **Terminate** → After `N` steps or goal reached, Episode Manager logs result and triggers new reset.
5. **Learn** → RL updates its policy online before next episode begins.

Every tick is scheduled via `return loop, 100` (100 ms).

### 5.1 Code sketch 💻

```lua
-- init
local episode   = EpisodeManager.new{max_steps=200}
local policy    = QLearning.new{alpha=0.1, gamma=0.9, eps=0.2}
local collector = Metrics.new()

function loop()
  if episode:done() then
    sim:set_pose(0, episode.start_loc, episode.start_ori)
    policy:learn(collector:get())
    episode:reset()
    collector:clear()
  else
    local s = episode:get_state()
    local a = policy:select(s)
    PolicyExecutor.apply(a)
    collector:record(s, a)
  end
  return loop, 100 -- ms
end

return loop, 100
```

*(Full implementation lives in [https://github.com/b-andreoni/GSoC/blob/main/scripts](https://github.com/b-andreoni/GSoC/blob/main/scripts))*

---

## 6. Timeline 📅

This GSoC project is structured into five main phases:

| Phase                              | Timeframe                   | Key Deliverables                                                                                                          |
| ---------------------------------- | --------------------------- | ------------------------------------------------------------------------------------------------------------------------- |
| **1. Reset Mechanism**             | June 2025                   | • Integrate `sim:set_pose()` into SITL Lua API<br>• Enable repeatable episode resets                                      |
| **2. Offline RL Phase (baseline)** | late June – July 2025       | • Collect episodic logs with a fixed policy<br>• Train baseline value/Q‑function offline<br>• Report baseline performance |
| **3. Online RL Loop**              | July 2025                   | • Implement reward & metrics collection<br>• Activate in‑place learning (SARSA/Q‑learning)                                |
| **4. Policy Refinement**           | August 2025                 | • Add exploration strategies (e.g., ε‑decay)<br>• Validate on varied flight scenarios                                     |
| **5. Docs & Releases**             | Early Sep 2025              | • Publish user guide and examples<br>• Open PRs/issues for community review                                               |

## 7. Call for Feedback 💬

I’d love to hear your field‑tested wisdom, war stories, and wish‑list items:

* **Reset mechanics** – any EKF glitches, timing tricks, or failsafes you’ve hit when using `sim:set_pose()`?
* **Practical benefits** – where could an in‑sim offline+online RL loop spare you manual PID sweeps or repetitive test flights?

Drop a comment on the PR, open an issue, or ping me on Discord (@bruno\_as). Your input will directly shape the next commits.

Thanks in advance!

## 8. GitHub 🔗

[https://github.com/b-andreoni/GSoC/](https://github.com/b-andreoni/GSoC/)

### Related PRs & Code Snippets

* [https://github.com/ArduPilot/ardupilot/pull/29616](https://github.com/ArduPilot/ardupilot/pull/29616)
* [https://github.com/ArduPilot/ardupilot/blob/master/libraries/AP\_Scripting/examples/sim\_arming\_pos.lua](https://github.com/ArduPilot/ardupilot/blob/master/libraries/AP_Scripting/examples/sim_arming_pos.lua)
* [https://github.com/ArduPilot/ardupilot/pull/29498/commits/51364c7f43af09c5f42d3e95315dc9c642dc093c](https://github.com/ArduPilot/ardupilot/pull/29498/commits/51364c7f43af09c5f42d3e95315dc9c642dc093c)



# Mid-Term Project Update  
> **Reinforcement Learning is already possible**

Hello again, ArduPilot community! Below is a concise overview of the milestones reached, and obstacles overcome during the first half of GSoC 2025.

## 1. What has been accomplished so far

### 1.1 `set_pose()` + mission integration  
The initial task was to extend the example script [`sim_arming_pos.lua`](https://github.com/ArduPilot/ardupilot/blob/c90c3ae6a67356c59f0a07a498f682d94b874459/libraries/AP_Scripting/examples/sim_arming_pos.lua#L64) so that a teleport could be executed after arming, followed immediately by any mission of interest.

* [**AUTO mode**](https://github.com/b-andreoni/GSoC/blob/main/scripts/mission/set_pose%2Bmission.lua) - After arming, the vehicle is teleported anywhere in the world, takes off, and then switches to AUTO to fly a pre-saved mission.  
* [**GUIDED mode**](https://github.com/b-andreoni/GSoC/blob/main/scripts/mission/set_pose%2Btarget.lua) - With AUTO working, waypoints are now injected dynamically via GUIDED, providing full path control from Lua.

A limitation emerged: subsequent teleports reused the *initial* NED offset, which quickly became impractical. This motivated a deeper change to `sim:set_pose()`.

### 1.2 Local modifications to `sim:set_pose()`  
Thanks to @tridge, we already had a working `set_pose`! 
However, episodic resets were still impossible because only the NED offset was updated; updating global position more than once was impossible. The solution required editing **`ardupilot/libraries/SITL/SIM_Aircraft.cpp`**:

```cpp
// original
aircraft.position = aircraft.home.get_distance_NED_double(loc);

// replacement
aircraft.home   = loc;
aircraft.origin = loc;
aircraft.position = Vector3d(0, 0, 0);
```

With *home*, *origin*, and *position* realigned, each call to `sim:set_pose()` now places the vehicle at the exact intended global location, enabling clean episode restarts.
**The complete patch is already pushed to my public fork:**
https://github.com/b-andreoni/ardupilot
Feel free to clone it instead of patching the file manually.

### 1.3 Parameters logging  
To evaluate learning performance, a [CSV logger](https://github.com/b-andreoni/GSoC/blob/main/scripts/log/log.lua) was created that records

```
elapsed_ms, fsm_state, flight_mode, is_armed,
lat_deg, lon_deg, alt_m,
vel_n, vel_e, vel_d,
roll, pitch, yaw, yaw_bhv,
gyro_x, gyro_y, gyro_z,
volt_v, curr_a
```
A flag is inserted whenever a reset occurs, allowing precise segmentation of episodes.

### 1.4 Customisable initial conditions at [episodic resets](https://github.com/b-andreoni/GSoC/blob/main/scripts/reset/episodic_reset.lua)
Reset hooks allow deterministic tweaks - e.g. yaw can be rotated by 45° at every episode start.
![Yaw increment|442x84, 75%](upload://nf21jHs38hso3D7ptAog17R5iuI.png)


### 1.5 First Reinforcement-Learning prototype 
![|545x500, 75%](upload://e3xp7XGUwEoT8r1chnLEem3It8c.png)
A lightweight **online Q-learning loop** was implemented to optimise best path to visit five waypoints energy-minimizing (inspired by the [IMAV 2024](https://2024.imavs.org/competition) first task). Workflow:

1. Teleport to home via `sim:set_pose()`.  
2. Take off to 10 m AGL.  
3. Select the next waypoint ε-greedily (ε = 0.2, decaying).  
4. Log voltage Ã current every 50 ms; **negative energy** is the reward.  
5. Apply a Q-update at each waypoint (`α = 0.1`, `γ = 0.9`).  
6. End-of-episode: compare total energy to best-so-far; repeat.  

Convergence to the optimal path occurs after â 200 episodes.

---

## 2. Current implementation

To reproduce the results:

1. Patch **`SIM_Aircraft.cpp`** as below and rebuild SITL, or clone the forked repo:
![SIM_Aircraft modifications|690x232](upload://yiUpSiHn0QMYRHrhqsgS1N7fjRd.png)
```
git clone git@github.com:b-andreoni/ardupilot.git
cd ardupilot
./waf configure --board sitl
./waf copter
```
2. Clone the scripts repo at https://github.com/b-andreoni/GSoC/tree/main. 
`git clone git@github.com:b-andreoni/GSoC.git`
4. Enable Lua scripts: `param set SCR_ENABLE 1`.  
5. Copy `scripts/reinforcement_learning/rl_wp.lua` into the working `scripts` folder and run  

```bash
sim_vehicle.py -v ArduCopter --map
```

---

## 3. Results so far

* **Teleport reliability**: A stress test of consecutive [resets](https://github.com/b-andreoni/GSoC/blob/main/scripts/reset/episodic_reset.lua) shows no EKF divergence.  

https://youtu.be/MgVrWxLkT7Y

* **Energy-optimal path**: Q-learning finds the best path between waypoints.

https://youtu.be/zplOf3A-gOk  


---

## 4. Next steps and final-submission goals

* Publish additional **toy examples** (e.g., PID tuning for precision landing and waypoint centralization, plane flare optimization).  
* Write a **wiki-style guide** highlighting the reset API, logging template, and common pitfalls.  
* Polish reinforcement-learning.
* Gather community feedback.
* Stretch goal: **Gazebo / Isaac Sim bridge** - Investigate adapter layers for their reset functions, enabling cross-sim RL experiments.

Feedback and suggestions, mainly for desired tunable parameters, are very welcome!
