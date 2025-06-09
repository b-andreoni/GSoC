![|690x205](upload://xxHpjSo6MjBR78pnpE624gt8K52.png)

> **A Lua script that lets ArduPilot learn to tune itself, live, within SITL.**

**Mentors:** Nate Mailhot @NDev & Sanket Sharma @snktshrma &nbsp;&nbsp;|&nbsp;&nbsp;**Timeframe:** May → Sep 2025&nbsp;&nbsp;|&nbsp;&nbsp;**Project:** [GSoC](https://summerofcode.withgoogle.com/programs/2025/projects/w7EYZSIz)

## 1. Introduction 📣

Hello ArduPilot community! I'm Bruno Andreoni Sarmento, a 4th year student at the Polytechnic School of the University of São Paulo (USP), pursuing a bachelor’s degree in Electrical Engineering. For the past two years I’ve been a member of Skyrats (USP’s autonomous-drone team), where I’ve worked with many open-source tools, ArduPilot being our core flight-control framework.

I've been inspired by other team members that took part in GSoC project, and it was a dream come true being selected to this awesome project!

## 2. Problem Statement ❗

Today, UAV parameter tuning still relies on manual adjustments, often requiring trial-and-error sweeps and test flights. This approach is:

- Time-consuming and inefficient;
- Poorly reproducible;
- Not scalable for adaptive or autonomous systems.

Meanwhile, **Reinforcement Learning (RL)** offers a principled method for adaptive control, but the **ArduPilot SITL environment lacks built-in support for episodic interactions**, which are essential for reliable training and evaluation.

## 3. Project Proposal 🚀
> This project builds a lightweight Lua-based scripting layer on top of the existing SITL engine to enable episodic RL interactions.

![RL Illustrated|690x370](upload://fxhFDlPAXijWp4RNjHTR8CqNpWq.jpeg)


The project introduces:

- **Episodic Replay**: A structured way to test and evaluate UAV behavior across multiple runs, starting from known or randomized states.
- **State Reset Mechanism**: Using `sim:set_pose()` in Lua scripts to programmatically reset UAV position, velocity, and attitude between episodes.
- **Metric Logging**: Automated telemetry collection (e.g., tracking error, convergence time) to evaluate each episode’s performance.
- **Online RL Loop**: A lightweight **online algorithm** (e.g., SARSA or Q-learning) will run in real-time within SITL to update parameters during simulation.

This project builds the necessary infrastructure for **online learning directly in SITL**, unlocking new workflows for intelligent parameter tuning and experimental reinforcement learning within the ArduPilot ecosystem.

## 4. Solution Diagram 🖼️
![Solution Diagram|576x500](upload://kGoStJvVlYuzYH29CLV0wP84rbJ.jpeg)


This diagram shows how **online reinforcement learning** is embedded into **ArduPilot SITL** using **Lua scripting**.

### 4.1 Block‑by‑Block Breakdown 🔍

| Block                 | Role                                                                                  | Key ArduPilot / Lua APIs                                       |
| --------------------- | ------------------------------------------------------------------------------------- | -------------------------------------------------------------- |
| **Episode Manager**   | Detects episode termination (e.g. time‑out, crash, success) and fires a reset.        | `time:millis()`, custom state checks                           |
| **Teleport / Reset**  | Instantaneously re‑positions vehicle & zeros attitude/vel using **`sim:set_pose()`**. | `sim:set_pose(instance, loc, orient, vel_bf, gyro)`            |
| **Metrics Collector** | Computes reward signals (tracking error, overshoot, energy) every tick.               | `ahrs:get_pitch()`, `vehicle:get_target_alt()`, custom math    |
| **Online RL**         | Updates Q‑table / value function in‑place at \~10 Hz; selects next action.            | Pure Lua tables for Q‑table; math lib                          |
| **Policy Executor**   | Applies action by tweaking params or sending direct control:                          | `param:set("ATC_RAT_PIT_P", val)`, `vehicle:set_target_roll()` |
| **EKF3**              | Supplies fused state to Lua via binding helpers (`ahrs`, `ins`, etc.).                | Firmware internal                                              |
| **Sensor Simulation** | Generates IMU/GPS from physics backend (Gazebo or built‑in).                          | SITL C++                                                       |

## 5. How the Loop Runs 🔄
1. **Reset** → Script arms vehicle, immediately calls `sim:set_pose()` to starting state.
2. **Episode** → RL chooses an action (e.g., ±5 % on `ATC_RAT_RLL_P`).
3. **Flight** → Controller responds; Metrics Collector accumulates reward.
4. **Terminate** → After `N` steps or goal reached, Episode Manager logs result and triggers new reset.
5. **Learn** → RL updates its policy online before next episode begins.

Every tick is scheduled via `return loop, 100` (100 ms).
### 5.1 Code sketch 💻
A simplified sketch of how the episodic learning loop works in Lua:

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

*(Full implementation lives in https://github.com/b-andreoni/GSoC/blob/main/scripts)*

---


## 6. Timeline 📅
This GSoC project is structured into four main phases:

| Phase                    | Timeframe    | Key Deliverables                                           |
|--------------------------|--------------|------------------------------------------------------------|
| **1. Reset Mechanism**   | June 2025    | - Integrate `sim:set_pose()` into SITL Lua API<br>- Enable repeatable episode resets |
| **2. Episodic RL Loop**  | July 2025    | - Implement reward & metrics collection<br>- Build basic online RL loop (Q-learning/SARSA) |
| **3. Policy Refinement** | August 2025  | - Add exploration strategies (e.g. ε-decay)<br>- Validate on varied flight scenarios |
| **4. Docs & Releases**   | Early Sept   | - Publish user guide and examples<br>- Open PRs/issues for community review |

## 7. Call for Feedback 💬

I’d love to hear your field-tested wisdom, war stories, and wish-list items:

- Reset mechanics – any EKF glitches, timing tricks, or failsafes you’ve hit when using sim:set_pose()?


- Practical benefits – where could an in-sim online RL loop spare you manual PID sweeps or repetitive test flights?

Drop a comment on the PR, open an issue, or ping me on Discord (@bruno_as). Your input will directly shape the next commits.
Thanks in advance!

## 8. Github 🔗
https://github.com/b-andreoni/GSoC/

### Related PRs & Code Snippets:
https://github.com/ArduPilot/ardupilot/pull/29616
https://github.com/ArduPilot/ardupilot/blob/master/libraries/AP_Scripting/examples/sim_arming_pos.lua
https://github.com/ArduPilot/ardupilot/commit/a03d4422721917bfa5eabdf0305727960f016bbd
https://github.com/ArduPilot/ardupilot/pull/29498/commits/51364c7f43af09c5f42d3e95315dc9c642dc093c



