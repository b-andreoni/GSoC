# GSoC 2025: SITL AI Reinforcement Learning Concept Script

![|690x205](upload://xxHpjSo6MjBR78pnpE624gt8K52.png)


> **A Lua script that lets ArduPilot learn to tune itself, live, within SITL.**

**Mentors:** Nate Mailhot & Sanket Sharma&nbsp;&nbsp;|&nbsp;&nbsp;**Timeframe:** May → Sep 2025&nbsp;&nbsp;|&nbsp;&nbsp;**Project:** [GSoC](https://summerofcode.withgoogle.com/programs/2025/projects/w7EYZSIz)

## Introduction

Hello Ardupilot community! I'm Bruno Andreoni Sarmento, a 4th year student at the Polytechnic School of the University of São Paulo (USP), pursuing a bachelor’s degree in Electrical Engineering. For the past two years I’ve been a member of Skyrats (USP’s autonomous-drone team), where I’ve worked with many open-source tools, ArduPilot being our core flight-control framework.

I've been inspired by other team members that took part in GSoC project, and it was a dream come true being selected to this awesome project!

## Problem Statement

Today, UAV parameter tuning still relies on manual adjustments, often requiring trial-and-error sweeps and test flights. This approach is:

- Time-consuming and inefficient;
- Poorly reproducible;
- Not scalable for adaptive or autonomous systems.

Meanwhile, **Reinforcement Learning (RL)** offers a principled method for adaptive control, but the **ArduPilot SITL environment lacks built-in support for episodic interactions**, which are essential for reliable training and evaluation.

## Project Proposal
> This project builds a lightweight Lua-based scripting layer on top of the existing SITL engine to enable episodic RL interactions.

![Reinforcement Learning Illustrated](upload://fxhFDlPAXijWp4RNjHTR8CqNpWq.jpeg)

The project introduces:

- **Episodic Replay**: A structured way to test and evaluate UAV behavior across multiple runs, starting from known or randomized states.
- **State Reset Mechanism**: Using `sim:set_pose()` in Lua scripts to programmatically reset UAV position, velocity, and attitude between episodes.
- **Metric Logging**: Automated telemetry collection (e.g., tracking error, convergence time) to evaluate each episode’s performance.
- **Online RL Loop**: A lightweight **online algorithm** (e.g., SARSA or Q-learning) will run in real-time within SITL to update parameters during simulation.

This project builds the necessary infrastructure for **online learning directly in SITL**, unlocking new workflows for intelligent parameter tuning and experimental reinforcement learning within the ArduPilot ecosystem.

## Solution Diagram
![Solution Diagram](upload://nybo4sE23UPrmlUY2lCamjmewGH.jpeg)

This diagram shows how **online reinforcement learning** is embedded into **ArduPilot SITL** using **Lua scripting**.

Key components:
- `Episode Manager` controls episodes and triggers resets via `sim:set_pose()`.
- `Metrics Collector` gathers rewards.
- `Online RL` updates the policy live (Q-learning, SARSA, actor-critic).
- `Policy Executor` sends actions or updates parameters.

**Legend**:
- **Solid arrows** represent *events or commands*.
- **Dashed arrows** represent *data flow* (e.g., telemetry, reward, state).

## Code sketch
Here's a simplified sketch of how the episodic learning loop works in Lua:

```lua
-- Initialize
episode = EpisodeManager:new{max_steps = 200, initial_state = {...}}
policy  = QLearningPolicy:new{alpha=0.1, gamma=0.9, epsilon=0.2}
metrics = MetricsCollector:new()

-- Core tick function
function tick()
  if episode:done() then
    sim:set_pose(
      instance,
      episode.initial_state.pos,
      episode.initial_state.orient,
      episode.initial_state.vel,
      episode.initial_state.gyro_rads
    )
    policy:learn(metrics:get_all())
    episode:start_new()
    metrics:clear()
  else
    local state  = state_estimator:get_state()
    local action = policy:select(state)
    param:set_value(action.param, action.value)
    metrics:record(state, action)
  end
  return tick, 500
end

return tick, 500

```

## Timeline
This GSoC project is structured into four main phases:

| Phase                    | Timeframe    | Key Deliverables                                           |
|--------------------------|--------------|------------------------------------------------------------|
| **1. Reset Mechanism**   | June 2025    | - Integrate `sim:set_pose()` into SITL Lua API<br>- Enable repeatable episode resets |
| **2. Episodic RL Loop**  | July 2025    | - Implement reward & metrics collection<br>- Build basic online RL loop (Q-learning/SARSA) |
| **3. Policy Refinement** | August 2025  | - Add exploration strategies (e.g. ε-decay)<br>- Validate on varied flight scenarios |
| **4. Docs & Releases**   | Early Sept   | - Publish user guide and examples<br>- Open PRs/issues for community review |

## Call for feedback
I'm looking for feedback from the ArduPilot community on:

- Reset strategies using sim:set_pose()

- Good reward signals for tuning (especially velocity/PID)

- Use cases where episodic RL could help your workflow

