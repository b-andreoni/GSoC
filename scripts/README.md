# GSoC Scripts

## 0. Patching `SIM_Aircraft.cpp`

The stock `sim:set_pose()` only updates the NED offset, preventing multiple global relocations in the same run. Apply the following patch to `libraries/SITL/SIM_Aircraft.cpp` and rebuild SITL:

```diff
- aircraft.position = aircraft.home.get_distance_NED_double(loc)

+ aircraft.home     = loc            // new reference
+ aircraft.origin   = loc            // align EKF origin
+ aircraft.position = Vector3d(0,0,0) // zero NED offset
```

Re‑compile:

```bash
./waf configure --board sitl
./waf copter        # or plane
```

Alternatively, clone the pre‑patched fork:

```bash
git clone git@github.com:b-andreoni/ardupilot.git
```

## 1. Overview

This directory contains all the Lua scripts developed for the GSoC 2025 project: "SITL AI Reinforcement Learning Concept Script." The scripts here build the necessary infrastructure to enable ArduPilot to learn and auto-tune its parameters in real-time within the SITL environment.

## 2. Directory Structure

The repository is organized as follows to separate concerns and maintain a clean workflow.

```
/GSoC/scripts/
│
├── log/
│   └── log.lua
│
├── mission/
│   ├── set_pose+mission.lua
│   └── set_pose+target.lua
│
├── reinforcement_learning/
│   ├── path_optimization/
│   │   └── energy_copter_50wp.lua
│   ├── pid/
│   └── tecs/
│
├── reset/
│   ├── episodic_reset.lua
│   ├── sim_arming_pos_global.lua
│   └── sim_arming_pos_offset.lua
│
└── README.md
```

### 2.1 Component Breakdown

* **`log/`**: Contains the `log.lua` utility, a helper script for creating and managing structured log files during simulation runs.
* **`mission/`**: Holds high-level scripts that combine the reset mechanism with a simple flight mission. These are often the main entry points for running an experiment.
* **`reinforcement_learning/`**: The core of the project, containing the RL agent implementations. It is subdivided by application:
    * `path_optimization/`: Scripts related to trajectory and path planning agents.
    * `pid/`: Agents designed specifically for tuning PID controllers.
    * `tecs/`: Agents focused on tuning the Total Energy Control System (TECS).
* **`reset/`**: Contains low-level scripts focused on the "teleport" or reset mechanism, which leverages the `sim:set_pose()` function to reset the vehicle's state between training episodes.

## 3. How to Use

To run these scripts, you must load them into a running SITL instance. The typical workflow is as follows:

1.  **Start SITL:** Launch your desired vehicle, for instance:
    ```bash
    sim_vehicle.py -v ArduCopter --console --map
    ```

2.  **Load a Script:** From the SITL console, load a high-level mission script. For example:
    ```bash
    script GSoC/scripts/reinforcement_learning/path_optimization/energy_copter_50wp.lua
    ```

3.  **Monitor:** Watch the console output to track the learning progress and the results of each episode.

4.  **Analyze:** After the simulation completes, check the `logs/` directory to analyze the collected data.

## 4. Core Scripts

This section highlights the key scripts that demonstrate the project's capabilities.

### 4.1 `reinforcement_learning/path_optimization/energy_copter_50wp.lua`

This is the flagship script for path optimization. It uses Q-learning to find the most energy-efficient route to visit 50 waypoints.

* **Purpose:** To train an agent that minimizes total battery consumption by learning the optimal flight path.
* **Logic:**
    1.  **Episodic Training:** The script runs hundreds of episodes. In each episode, the copter attempts to visit all 50 waypoints.
    2.  **State & Action:** The "state" is the set of waypoints already visited. The "action" is choosing the next waypoint.
    3.  **Reward:** The reward is the *negative* energy consumed during a flight segment. Maximizing this reward is equivalent to minimizing energy use.
    4.  **Learning:** After each segment, a Q-table is updated to store the long-term value of choosing a specific waypoint from a given state.
    5.  **Reset:** At the end of an episode, the script logs the results, teleports the vehicle back to the start using `sim:set_pose()`, and begins a new learning cycle.

### 4.2 `mission/set_pose+mission.lua`

This script serves as a simpler example of a complete training loop, combining a reset with a standard mission file.

* **Purpose:** To execute a full episodic session for testing or data collection.
* **Logic:**
    1.  **Reset:** Initializes the vehicle's position using a script from the `/reset/` directory.
    2.  **Execute:** Runs a predefined mission file (e.g., from `mission/`).
    3.  **Repeat:** At the end of the mission, the cycle repeats, allowing for consistent, repeatable tests.

### 4.3 `reset/episodic_reset.lua`

This script provides the foundational reset functionality used by all other learning scripts.

* **Purpose:** To reliably and repeatedly reset the vehicle's state (position, attitude, velocity) to a consistent starting point for each training episode.
* **Function:** Wraps the `sim:set_pose()` binding in a reusable function that can be called at the start or end of an episode.
