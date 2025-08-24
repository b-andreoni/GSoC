# Reinforcement Learning Agents
## 1. Overview
This is the core directory of the GSoC project, containing all the "brains" of the operation. The Lua scripts within this folder and its subdirectories implement various Reinforcement Learning (RL) agents designed to solve specific autonomous control and tuning problems within the ArduPilot SITL environment.

The central idea is to frame complex autopilot tuning tasks as "games" that an intelligent agent can learn to master through trial and error, ultimately discovering solutions that are often superior to manual, human-driven tuning.

## 2. The "Toy Example" Philosophy
Each subdirectory here represents a self-contained "toy example" — a focused experiment that isolates a single, well-defined problem. This approach has several advantages:

Clarity: It allows us to clearly define the state, action, and reward for a specific task without a confusion of unrelated variables.

Modularity: Each agent can be developed, tested, and understood independently.

Foundation: These simple examples serve as foundational building blocks and proofs-of-concept for more complex, multi-objective learning agents in the future.

## 3. Agent Categories
The RL agents are organized into the following categories. Please refer to the README.md within each folder for a detailed breakdown of the problem, the solution, and the specific RL implementation.

📁 ./path_optimization/
Problem: Finding the most efficient route (in terms of energy or time) to visit a series of waypoints.

Use Case: Ideal for mission planning, logistics, and maximizing flight endurance.

📁 ./pid/
Problem: Automating the tedious and complex process of tuning PID controller gains for stable and responsive flight.

Use Case: Foundational for vehicle setup, ensuring crisp attitude control and stability.

📁 ./tecs/
Problem: Tuning the interdependent parameters of the Total Energy Control System (TECS) for fixed-wing aircraft.

Use Case: Critical for efficient and smooth management of airspeed and altitude in fixed-wing missions.
