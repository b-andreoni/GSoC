# Path Optimization Agents

This directory contains "toy examples" for solving path optimization problems using Reinforcement Learning. The primary example, `energy_copter_50wp.lua`, is designed to be a clear, adaptable template for your own projects.

### 1. The Problem: Finding the Best Route

For any UAV, determining the most efficient path to visit a series of waypoints is a complex challenge. Manually planning an optimal route is impractical and rarely yields the best result, especially as the number of waypoints grows.

### 2. Why Solve It: The Value of Efficiency

Automating this process has direct, practical benefits:
* **Energy Efficiency:** Minimizing energy consumption directly translates to longer flight times and greater operational range.
* **Time Efficiency:** For time-sensitive tasks, minimizing total flight time is the primary objective.

### 3. How It's Solved: A Q-Learning Toy Example

We frame this as a game that a **Q-learning** agent learns to master through trial and error in simulation.

* **Concept:** The agent runs many "episodes" (attempts to fly a route). In each episode, it makes decisions and receives a "reward" based on the outcome. The goal is to learn a strategy, or "policy," that maximizes the total reward over time.
* **The "Brain" (Q-Table):** The agent's memory is a Q-table, which stores the expected long-term reward for taking a specific action from a specific state. Over many episodes, this table converges to represent the optimal strategy.

### 4. Anatomy of the `energy_copter_50wp.lua` Example

This script teaches a Copter to find the most energy-efficient path through 50 waypoints. Here is how it maps to the RL concepts:

* **State:** The set of waypoints already visited in the current episode. A unique key (e.g., `"1-5-12"`) represents this state in the Q-table.
* **Action:** Choosing the next waypoint to fly to from the list of unvisited waypoints.
* **Reward:** The **negative energy consumed** to travel between waypoints. By learning to maximize this value, the agent is implicitly learning to **minimize** energy consumption.

The script's logic is managed by a simple Finite-State Machine (FSM):

| State | Value | Meaning                               |
| :---- | ----: | :------------------------------------ |
| WAIT  |    -1 | Waiting for EKF origin                |
| ARM   |     0 | Arm & automated take-off              |
| CRUISE|     1 | Cruise to target, measure energy      |
| LEARN |     2 | Arrived at WP, apply Q-update         |
| RESET |     3 | Episode finished, reset simulation    |

### 5. How to Adapt This Example for Your Problem

This script is designed to be a template. To solve your own pathing problem, you only need to modify a few key areas:

#### Step 1: Define Your Waypoints
Change the `wp_offsets` table to define the locations you want the agent to visit.

```lua
-- Define your waypoints relative to the home location
local wp_offsets = {
    {N=100, E=50, D=-20},
    {N=-100, E=50, D=-20},
    -- ... add as many waypoints as you need
}
```

#### Step 2: Define Your Reward Function
This is the most critical change. The reward signal tells the agent what you want it to optimize for.

* **To Minimize Time:** Instead of energy, calculate the time elapsed between waypoints and use the negative of that value as the reward.

    ```lua
    -- In STATE_CRUISE, track time instead of energy
    -- local startTime = tonumber(tostring(millis()))

    -- In STATE_LEARN, calculate reward
    -- local elapsedTime = tonumber(tostring(millis())) - startTime
    -- local reward = -elapsedTime
    ```

* **To Prioritize High-Value Targets:** Assign a positive reward for visiting certain waypoints and a negative reward (cost) for the travel time/energy between them.

#### Step 3: Adjust Learning Parameters
You can tune the agent's learning behavior by adjusting these constants at the top of the script:
* `ALPHA`: The learning rate (how much it updates its Q-table after each step).
* `GAMMA`: The discount factor (how much it values future rewards over immediate ones).
* `INITIAL_EPSILON`: The starting exploration rate (how often it chooses a random action).

### 6. Running the Example

1.  **Prerequisites:** Ensure you have a working ArduPilot SITL build environment.
2.  **Load the Script:** Place the script in your `scripts` directory and launch SITL.
    ```bash
    sim_vehicle.py -v ArduCopter --console --map
    script reinforcement_learning/path_optimization/energy_copter_50wp.lua
    ```
3.  **Enable:** Set the `SIM_ERES_ENABLE` parameter to `1` to start the learning process.
4.  **Monitor:** Watch the GCS console to see the agent learn and find the best path.
