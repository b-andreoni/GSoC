# SITL AI Reinforcement Learning Concept Script

📡 Reinforcement Learning inside ArduPilot SITL using Lua – Google Summer of Code 2025 Project  
**Author**: Bruno Andreoni Sarmento  
**Mentors**: Nathaniel Mailhot, PhD · Sanket Sharma  
**Organization**: ArduPilot

---

## 🧠 Project Overview

This repository contains a proof-of-concept implementation of a reinforcement learning (RL) agent running **natively inside ArduPilot’s SITL** using Lua.

Instead of relying on external RL pipelines with MAVLink, this project demonstrates how to embed an **episodic training loop**, define rewards, apply velocity setpoints, and update a Q-table — all directly from within the ArduPilot Lua scripting environment.

This work is part of **Google Summer of Code 2025**, under the ArduPilot organization.

---

## 🎯 Goals

- Define a simple **RL task**: UAV centralizes over a fixed target point.
- Implement **Q-learning** or **SARSA** inside ArduPilot SITL using Lua.
- Enable **episodic resets** within SITL.
- Log performance metrics to track training progress.
- Build a base for **AI-native flight behaviors**.

---

## 🚀 Quick Start

### Requirements

- ArduPilot SITL (compiled with Lua support)
- Python 3 (optional: for plotting logs)
- Lua 5.3 (used internally by ArduPilot)

### 1. Clone and Build ArduPilot

```bash
git clone https://github.com/ArduPilot/ardupilot.git
cd ardupilot
git submodule update --init --recursive
Tools/environment_install/install-prereqs-ubuntu.sh -y
. ~/.profile
