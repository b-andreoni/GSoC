# Mid-Term Project Update  
> **Reinforcement Learning is already possible**

Hello again, ArduPilot community! Below is a concise overview of the milestones reached, and obstacles overcome during the first half of GSoC 2025.

## 1. What has been accomplished so far

### 1.1 `set_pose()` + mission integration  
The initial task was to extend the example script [`sim_arming_pos.lua`](https://github.com/ArduPilot/ardupilot/blob/c90c3ae6a67356c59f0a07a498f682d94b874459/libraries/AP_Scripting/examples/sim_arming_pos.lua#L64) so that a teleport could be executed after arming, followed immediately by any mission of interest.

* [**AUTO mode**](https://github.com/b-andreoni/GSoC/blob/main/scripts/mission/set_pose%2Bmission.lua) - After arming, the vehicle is teleported anywhere in the world, takes off, and then switches to AUTO to fly a pre-saved mission.  
* [**GUIDED mode**](https://github.com/b-andreoni/GSoC/blob/main/scripts/mission/set_pose%2Btarget.lua) - With AUTO working, waypoints are now injected dynamically via GUIDED, providing full path control from Lua.

A limitation emerged: subsequent teleports reused the *initial* NED offset, which quickly became impractical. This motivated a deeper change to `sim:set_pose()`.

### 1.2 Local modifications to `sim:set_pose()`  
Thanks to @tridge, we already had a working `set_pose`! 
However, episodic resets were still impossible because only the NED offset was updated; updating global position more than once was impossible. The solution required editing **`ardupilot/libraries/SITL/SIM_Aircraft.cpp`**:

```cpp
// original
aircraft.position = aircraft.home.get_distance_NED_double(loc);

// replacement
aircraft.home   = loc;
aircraft.origin = loc;
aircraft.position = Vector3d(0, 0, 0);
```

With *home*, *origin*, and *position* realigned, each call to `sim:set_pose()` now places the vehicle at the exact intended global location, enabling clean episode restarts.
**The complete patch is already pushed to my public fork:**
https://github.com/b-andreoni/ardupilot
Feel free to clone it instead of patching the file manually.

### 1.3 Parameters logging  
To evaluate learning performance, a [CSV logger](https://github.com/b-andreoni/GSoC/blob/main/scripts/log/log.lua) was created that records

```
elapsed_ms, fsm_state, flight_mode, is_armed,
lat_deg, lon_deg, alt_m,
vel_n, vel_e, vel_d,
roll, pitch, yaw, yaw_bhv,
gyro_x, gyro_y, gyro_z,
volt_v, curr_a
```
A flag is inserted whenever a reset occurs, allowing precise segmentation of episodes.

### 1.4 Customisable initial conditions at [episodic resets](https://github.com/b-andreoni/GSoC/blob/main/scripts/reset/episodic_reset.lua)
Reset hooks allow deterministic tweaks - e.g. yaw can be rotated by 45° at every episode start.
![Yaw increment|442x84, 75%](upload://nf21jHs38hso3D7ptAog17R5iuI.png)


### 1.5 First Reinforcement-Learning prototype 
![|545x500, 75%](upload://e3xp7XGUwEoT8r1chnLEem3It8c.png)
A lightweight **online Q-learning loop** was implemented to optimise best path to visit five waypoints energy-minimizing (inspired by the [IMAV 2024](https://2024.imavs.org/competition) first task). Workflow:

1. Teleport to home via `sim:set_pose()`.  
2. Take off to 10 m AGL.  
3. Select the next waypoint ε-greedily (ε = 0.2, decaying).  
4. Log voltage Ã current every 50 ms; **negative energy** is the reward.  
5. Apply a Q-update at each waypoint (`α = 0.1`, `γ = 0.9`).  
6. End-of-episode: compare total energy to best-so-far; repeat.  

Convergence to the optimal path occurs after â 200 episodes.

---

## 2. Current implementation

To reproduce the results:

1. Patch **`SIM_Aircraft.cpp`** as below and rebuild SITL, or clone the forked repo:
![SIM_Aircraft modifications|690x232](upload://yiUpSiHn0QMYRHrhqsgS1N7fjRd.png)
```
git clone git@github.com:b-andreoni/ardupilot.git
cd ardupilot
./waf configure --board sitl
./waf copter
```
2. Clone the scripts repo at https://github.com/b-andreoni/GSoC/tree/main. 
`git clone git@github.com:b-andreoni/GSoC.git`
4. Enable Lua scripts: `param set SCR_ENABLE 1`.  
5. Copy `scripts/reinforcement_learning/rl_wp.lua` into the working `scripts` folder and run  

```bash
sim_vehicle.py -v ArduCopter --map
```

---

## 3. Results so far

* **Teleport reliability**: A stress test of consecutive [resets](https://github.com/b-andreoni/GSoC/blob/main/scripts/reset/episodic_reset.lua) shows no EKF divergence.  

https://youtu.be/MgVrWxLkT7Y

* **Energy-optimal path**: Q-learning finds the best path between waypoints.

https://youtu.be/zplOf3A-gOk  


---

## 4. Next steps and final-submission goals

* Publish additional **toy examples** (e.g., PID tuning for precision landing and waypoint centralization, plane flare optimization).  
* Write a **wiki-style guide** highlighting the reset API, logging template, and common pitfalls.  
* Polish reinforcement-learning.
* Gather community feedback.
* Stretch goal: **Gazebo / Isaac Sim bridge** - Investigate adapter layers for their reset functions, enabling cross-sim RL experiments.

Feedback and suggestions, mainly for desired tunable parameters, are very welcome!