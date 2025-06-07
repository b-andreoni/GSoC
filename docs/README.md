# GSoC 2025: SITL AI Reinforcement Learning Concept Script

![|690x205](upload://xxHpjSo6MjBR78pnpE624gt8K52.png)


> **A Lua script that lets ArduPilot learn to tune itself, live, within SITL.**

**Mentors:** Nate Mailhot & Sanket Sharma&nbsp;&nbsp;|&nbsp;&nbsp;**Timeframe:** May → Sep 2025&nbsp;&nbsp;|&nbsp;&nbsp;**Project:** [GSoC](https://summerofcode.withgoogle.com/programs/2025/projects/w7EYZSIz)

## Introduction

Hello Ardupilot community! I'm Bruno Andreoni Sarmento, a 4th year student at the Polytechnic School of the University of São Paulo (USP), pursuing a bachelor’s degree in Electrical Engineering. For the past two years I’ve been a member of USP’s autonomous-drone team, where I’ve worked with many open-source tools, ArduPilot being our core flight-control framework.

I've been inspired by other team members that took part in GSoC project, and it was a dream come true being selected to this awesome project!

## Problem Statement
- Current gap: Autonomous-drone parameter tuning still relies heavily on manual PID sweeps.

- Solution: Lua scripts to teleport the vehicle (sim:set_pose), collect metrics, and generate reproducible episodes.

- Final objective: Demonstrate an offline RL loop (e.g., Q-learning, SARSA) using the generated logs.

## Project Proposal
![Reinforcement Learning Illustrated|690x370](upload://fxhFDlPAXijWp4RNjHTR8CqNpWq.jpeg)

The goal of this project is to embed a lightweight reinforcement-learning layer (written in Lua and running natively inside ArduPilot SITL) that:

- Segments flights into reproducible episodes,

- Adjusts key parameters such as velocity and attitude gains,

- Learns from reward signals like tracking error and settling time.

By the end of GSoC 2025 the community will have: a generic episode-reset API for SITL, a plug-and-play RL script with Q-learning/SARSA, and step-by-step docs so others can extend it.

## Solution Diagram

## Architecture sketch

## Code sketch

## Timeline

## Call for feedback


flowchart LR
  subgraph SITL_Core
    A[jmavsim/Gazebo]
    B[Sensor Simulation]
  end
  subgraph ArduPilot
    C[EKF3 State Estimator]
    D[Attitude & Rate Controller]
    E[Parameter System]
  end
  subgraph Lua_Scripts
    F[Episodic Controller]
    G[Data Logger]
    H[Policy Executor]
  end
  subgraph Offline_RL
    I[Python Q-Learning]
  end

  A --> B
  B --> C
  C --> F
  F --> D
  F --> G
  G --> I
  I --> H
  H --> E
  H --> D
  F -- reset --> A
