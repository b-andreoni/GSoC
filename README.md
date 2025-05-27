# SITL AI Reinforcement Learning Concept Script

📡 Reinforcement Learning inside ArduPilot SITL using Lua – Google Summer of Code 2025 Project  
**Author**: Bruno Andreoni Sarmento  
**Mentors**: Nathaniel Mailhot · Sanket Sharma  
**Organization**: ArduPilot

---

## 🧠 Project Overview

This repository contains a proof-of-concept implementation of a reinforcement learning (RL) agent running **natively inside ArduPilot’s SITL** using Lua.

Instead of relying on external RL pipelines with MAVLink, this project demonstrates how to embed an **episodic training loop**, define rewards, apply velocity setpoints, and update a Q-table — all directly from within the ArduPilot Lua scripting environment.

This work is part of **Google Summer of Code 2025**, under the ArduPilot organization.

---

## 🎯 Goals

- Enable **episodic resets** within SITL.
- Define a simple **RL task**: UAV centralizes over a fixed target point.
- Implement **Q-learning** or **SARSA** inside ArduPilot SITL using Lua.
- Log performance metrics to track training progress.
- Build a base for **AI-native flight behaviors**.

---

## 🚀 Quick Start

### Requirements

- ArduPilot SITL (compiled with Lua support)
- Python 3
- Lua 5.3 (used internally by ArduPilot)

### 1. Clone and Build ArduPilot

```bash
git clone https://github.com/ArduPilot/ardupilot.git
cd ardupilot
git submodule update --init --recursive
Tools/environment_install/install-prereqs-ubuntu.sh -y
. ~/.profile
```

### 2. Run SITL + load script

```bash
sim_vehicle.py -v ArduCopter --console --map \
  --script scripts/reset_rl.lua
```

## :wrench: How It Works
1. Reset   – A Lua routine uses `sim:set_pose()` to teleport the drone to the start coordinates and zero its velocity.
2. Act     – The RL core selects a velocity set-point (`vehicle:set_target_velocity_NED`).
3. Observe – Position error is measured and converted to a scalar reward.
4. Learn   – Q-table (or SARSA) is updated in place.
5. Loop    – After `N` seconds, return to step 1 for the next episode.

## 📂 Repo Layout

```plaintext
.
└── scripts
    └── reset
        └── sim_arming_pos.lua      # initial position setting logic
```


## 🤝 Contributing
Issues and pull requests are welcome, especially bug reports on reset stability or ideas for richer reward functions!