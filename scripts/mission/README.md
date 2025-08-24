# Mission Directory

## 1. Overview

This directory contains high-level scripts designed to orchestrate complete, repeatable test sequences in SITL. These scripts act as the main entry point for an experiment, combining the vehicle reset functionality (`sim:set_pose`) with mission execution or specific flight tasks.

They are essential for ensuring that every test run starts from a consistent and precisely controlled initial state.

## 2. Core Scripts

### 2.1 `set_pose+mission.lua`

This script provides a robust framework for starting a pre-loaded mission from a specific, programmatically defined pose. It is ideal for tests where the vehicle must execute a full flight plan after being initialized in a non-standard state.

* **Purpose:** To reliably teleport a vehicle and then automatically arm, take off, and begin a mission.
* **Operational Flow:**
    1.  **Immediate Pose Set:** As soon as the script is enabled, it uses `sim:set_pose()` to force the vehicle's position, attitude, and velocity to the values defined in its parameters.
    2.  **Stabilization Wait:** It waits for a fixed period (approx. 12 seconds) to allow the simulator's physics and EKF to stabilize after the teleport.
    3.  **Automated Takeoff:** The script programmatically arms the vehicle, requests a takeoff in GUIDED mode to a safe altitude, and then immediately switches to AUTO mode.
    4.  **Mission Start:** After confirming the vehicle is in AUTO mode, it waits a few more seconds for stabilization and then commands the mission to start from the first waypoint.

### 2.2 `set_pose+target.lua`

This script is a variation designed for simpler "go-to-point" tasks. Instead of executing a full mission, it teleports the vehicle and then commands it to fly to a single target location.

* **Purpose:** To test vehicle behavior (like PID response or TECS tuning) in response to a single step-input or target change from a known initial state.
* **Typical Use:** Useful for RL agents that need to learn how to fly efficiently from point A to point B.

## 3. How to Use

These scripts are intended to be the main file you load to run an experiment.

1.  **Configure Parameters:** Adjust the `SIM_APOS_*` parameters in your `user.parm` file or via the GCS to define the desired starting pose.
2.  **Load Mission:** Ensure a mission file is loaded into the SITL instance if you are using `set_pose+mission.lua`.
3.  **Run Script:** From the SITL console, load the script:
    ```bash
    script GSoC/scripts/mission/set_pose+mission.lua
    ```
4.  **Enable:** Set the `SIM_APOS_ENABLE` parameter to 1 to trigger the sequence.
