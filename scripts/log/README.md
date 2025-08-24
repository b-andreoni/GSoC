# Log Directory

## 1. Overview

This directory contains the primary telemetry logging utility for the GSoC project, `flight_logger.lua`. This script is designed to run in the background during SITL sessions, capturing a comprehensive set of flight data to a timestamped CSV file for offline analysis.

It is a crucial tool for debugging, performance analysis, and gathering the rich datasets required for training Reinforcement Learning agents.

## 2. Core Utility: `flight_logger.lua`

This is a robust, modular script for high-frequency data logging. It is designed to be self-contained and resilient to common issues in the simulation environment.

### 2.1 Purpose

The main goal of `flight_logger.lua` is to provide a consistent and detailed record of the vehicle's state throughout a simulation. It logs data at a configurable frequency (default is 10 Hz) and automatically handles file creation and management.

### 2.2 Key Features

* **Modular Design:** The script is built around a `Logger` object, encapsulating all state and logic. This makes the code clean, readable, and easy to maintain or extend.
* **Automatic File Naming:** Creates a unique log file for each session (e.g., `flight_log_1678886400.csv`) to prevent accidental overwrites.
* **Episodic Reset Detection:** The logger intelligently monitors the Finite State Machine (FSM) state (`SIM_ERES_STATE` parameter). When it detects a reset (a transition back to state 0), it inserts an `EPISODE_RESET` marker into the log file, making it easy to separate episodic data during analysis.
* **Comprehensive Data:** It captures a wide range of telemetry, including position, velocity, attitude, gyro rates, battery status, and control parameters.
* **Robust File Handling:** Uses `pcall` for safe file operations to ensure that a file system error does not crash the entire simulation script.

## 3. Log File Format (`.csv`)

The script generates a CSV file with a detailed header, making it directly importable into data analysis tools like Python (Pandas), MATLAB, or Excel.

### 3.1 Column Descriptions

| Column          | Unit          | Description                                                                 |
| --------------- | ------------- | --------------------------------------------------------------------------- |
| `elapsed_ms`    | milliseconds  | Time elapsed since the logger started.                                      |
| `fsm_state`     | integer       | The current state of the main RL script's Finite State Machine.             |
| `flight_mode`   | integer       | The current ArduPilot flight mode (e.g., 4 for GUIDED).                     |
| `armed`         | boolean (0/1) | Vehicle arming status. 1 if armed, 0 if disarmed.                           |
| `lat_deg`       | degrees       | Latitude of the vehicle.                                                    |
| `lon_deg`       | degrees       | Longitude of the vehicle.                                                   |
| `alt_m`         | meters        | Altitude above mean sea level.                                              |
| `vel_n_ms`      | m/s           | Velocity in the North direction (NED frame).                                |
| `vel_e_ms`      | m/s           | Velocity in the East direction (NED frame).                                 |
| `vel_d_ms`      | m/s           | Velocity in the Down direction (NED frame).                                 |
| `roll_deg`      | degrees       | Vehicle roll angle.                                                         |
| `pitch_deg`     | degrees       | Vehicle pitch angle.                                                        |
| `yaw_deg`       | degrees       | Vehicle yaw angle (heading).                                                |
| `yaw_bhv`       | integer       | The `WP_YAW_BEHAVIOR` parameter value.                                      |
| `gyro_x_rads`   | rad/s         | Angular velocity around the X-axis.                                         |
| `gyro_y_rads`   | rad/s         | Angular velocity around the Y-axis.                                         |
| `gyro_z_rads`   | rad/s         | Angular velocity around the Z-axis.                                         |
| `volt_v`        | Volts         | Battery voltage.                                                            |
| `curr_a`        | Amps          | Battery current draw.                                                       |

## 4. How to Use

The `flight_logger.lua` script is intended to be run as a background service alongside your main control or learning script.

1.  **Start your main script** (e.g., `energy_copter_50wp.lua`).
2.  **From a separate MAVProxy console or terminal**, load the logger to the scripts folder:
    ```bash
    cp GSoC/scripts/log/flight_logger.lua ~/scripts
    ```
3.  The logger will automatically create a new file in the `~/logs/` directory and begin recording data. It will run until the simulation is terminated.
