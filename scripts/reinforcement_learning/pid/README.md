# PID Controller Tuning Agents

This directory contains a "toy example" for automatically tuning PID controller gains using Reinforcement Learning. The script `pid.lua` is a clear, adaptable template for creating your own automated tuning agents.

### 1. The Problem: The Manual Tuning Grind

PID (Proportional-Integral-Derivative) controllers are the foundation of stable flight. However, tuning their gains is a notoriously difficult and time-consuming process that often involves guesswork, repetitive flight tests, and expert knowledge to diagnose issues like oscillations or sluggish response.

### 2. Why Solve It: The Value of Automation

Automating the PID tuning process provides immense value:
* **Time Savings:** It drastically reduces the hours of manual effort required.
* **Optimal Performance:** An RL agent can systematically explore parameter combinations to find gains that result in a crisp, responsive, and stable vehicle.
* **Repeatability:** The process is entirely repeatable, ensuring consistent performance.

### 3. How It's Solved: A Q-Learning Toy Example

We frame PID tuning as a game where a **Q-learning** agent learns to select the best gains by running many short experiments in simulation.

* **Concept:** The agent runs many "episodes." In each episode, it tries a new set of PID gains and performs a test maneuver. It gets a "reward" based on its performance and learns which changes lead to better results.
* **The "Brain" (Q-Table):** The agent's memory is a Q-table, which stores the expected long-term reward for taking a specific action (changing a gain) given the current state (the vehicle's performance).

### 4. Anatomy of the `pid.lua` Example

This script teaches a Copter to find the optimal `P`, `I`, and `D` gains for the **vertical position controller** (`PSC_ACCZ_*` parameters).

* **The Test Maneuver:** Each episode consists of a simple hover test. The agent takes off to a target altitude and attempts to hold it steady for a fixed duration.
* **RL Concepts:**
    * **State:** The vehicle's performance, defined by two discretized metrics:
        1.  **Altitude Error:** The difference between the target and actual altitude.
        2.  **Vertical Velocity:** The rate of change of the altitude error.
    * **Action:** A single, small, discrete adjustment to one of the three PID gains (e.g., "increase P by 0.02," "decrease I by 0.02," etc.) or "do nothing."
    * **Reward:** A score calculated from the **Mean Squared Error (MSE)** of the altitude during the hover test. The reward is `1 / (MSE + 0.001)`. A lower, more stable error results in a higher reward, teaching the agent to prioritize stability.

### 5. How to Adapt This Example for Your Problem

This script is a template. To tune a different PID controller (e.g., attitude control), you only need to modify a few key areas.

#### Step 1: Change the Target Parameters
Modify the `actions` table to target the parameters you want to tune and adjust the step sizes (`change`) to be appropriate for that controller.

```lua
-- Example for tuning Pitch Rate P and D gains
local actions = {
    {param="ATC_RAT_PIT_P", change=0.01},
    {param="ATC_RAT_PIT_P", change=-0.01},
    {param="ATC_RAT_PIT_D", change=0.0005},
    {param="ATC_RAT_PIT_D", change=-0.0005},
    {param=nil, change=0}
}
```

#### Step 2: Design a New Test Maneuver & State
For attitude tuning, a simple hover is not enough. You would need to modify the FSM to command a series of sharp attitude changes (e.g., step inputs in roll or pitch). Your **State** would then need to be based on metrics from that maneuver, such as overshoot, rise time, or oscillation.

#### Step 3: Define a New Reward Function
Your reward function must match your goal. For attitude tuning, you might calculate a reward based on how quickly and accurately the vehicle reaches the target attitude, while penalizing any overshoot.

### 6. Running the Example

1.  **Prerequisites:** Ensure you have a working ArduPilot SITL build environment for Copter.
2.  **Load the Script:** Place the script in your `scripts` directory and launch SITL.
    ```bash
    sim_vehicle.py -v ArduCopter --console --map
    script reinforcement_learning/pid/pid.lua
    ```
3.  **Enable:** Set the `SIM_PID_ENABLE` parameter to `1` to start the learning process.
4.  **Monitor:** Watch the GCS console. The script will output the chosen action for each episode, the resulting MSE, and the best parameters found so far.
