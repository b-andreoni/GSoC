====================================================================
Reinforcement Learning Episodic-Reset Lua Script for SITL
====================================================================
This page describes the *energy-optimised path* example used in the
**SITL AI Reinforcement Learning Concept Script** (GSoC 2025).  It shows
how to patch **ArduPilot SITL** for deterministic resets, install the Lua
script, and reproduce the results discussed in the Mid-Term update.

`energy_copter.lua <https://github.com/b-andreoni/GSoC/blob/main/scripts/reinforcement_learning/path_optimization/energy_copter.lua>`_


Overview
========

The script implements an online **Q-learning** loop that discovers the
least-energy path through five way-points.  Each episode:

1. *Resets* the vehicle to **home** using `sim:set_pose()`.
2. Performs a **GUIDED** **take-off** to 10 m AGL.
3. Selects the next waypoint ε-greedily and commands the vehicle in
   **GUIDED**.
4. Logs current x voltage every 50 ms; the integrated energy is used as a
   (negative) reward.
5. Updates the Q-table, compares the total energy against the best path
   so far, and restarts the episode.

After ≈ 200 episodes the policy converges.

Prerequisites
=============

- Linux build of **ArduPilot master** (`waf` build system).
- Lua scripting enabled in parameters (`SCR_ENABLE = 1`).
- **SITL** vehicle type: *Copter* or *Plane* (the example targets
  *Copter*).
- Patch to *SIM_Aircraft.cpp* (see below) **or** clone the fork
  `github.com/b-andreoni/ardupilot`.

Patching *SIM_Aircraft.cpp*
===========================

The stock `sim:set_pose()` only updates the NED offset, preventing
multiple global relocations in the same run.  Apply the following patch
(to *libraries/SITL/SIM_Aircraft.cpp*) and rebuild SITL:

.. code-block:: diff

    -aircraft.position = aircraft.home.get_distance_NED_double(loc)

    +aircraft.home     = loc           // new reference
    +aircraft.origin   = loc           // align EKF origin
    +aircraft.position = Vector3d(0,0,0) // zero NED offset
     

Re-compile::

    ./waf configure --board sitl
    ./waf copter   # or plane

Alternatively, clone the pre-patched fork::

    git clone git@github.com:b-andreoni/ardupilot.git

Installing the Lua script
=========================

1. Clone the script repository::

       git clone git@github.com:b-andreoni/GSoC.git

2. Copy `scripts/reinforcement_learning/path_optimization/energy_copter.lua` into the
   active `scripts` directory used by SITL.

3. Ensure Lua scripts are enabled::

       param set SCR_ENABLE 1

Quick start
===========

Launch SITL with the default *QuadPlane* model and open Map/GCS::

    sim_vehicle.py -v ArduCopter --map

You should see *“Loaded RL waypoints path script!”* in the GCS console.
After take-off the message feed will resemble::

  AP: EP 26: Learn Q[0,4]=-543.532, nextWP=1
  AP: EP 26: Learn Q[8,1]=-267.082, nextWP=3
  AP: EP 26: Learn Q[9,3]=-173.476, nextWP=5
  AP: EP 26: Learn Q[13,5]=-270.106, nextWP=2
  AP: FSM_STATE -> 2
  AP: EP 26: Learn Q[29,2]=-522.501, nextWP=nil
  AP: EP 26 END path: 4 -> 1 -> 3 -> 5 -> 2 total_E=7239.470 J
  AP: EP 11 [BEST] path: 3 -> 4 -> 1 -> 5 -> 2 E=5889.613 J
  AP: Disarming motors
  AP: FSM_STATE -> -1


Parameter table
===============

The script creates a custom table `SIM_ERES_*` (key 90) inherited from https://github.com/b-andreoni/GSoC/blob/main/scripts/reset/episodic_reset.lua.  Important
fields:

================  =====  ================================
Name              Idx    Purpose
================  =====  ================================
`ENABLE`          1      0 = idle script, 1 = run
`STATE`           18     Current FSM state
`OFST_N/E/D`      2-4    Episodic initial offset (m)
`VEL_X/Y/Z`       5-7    Initial body-frame velocities
`RLL/PIT/YAW`     8-10   Initial attitude (deg)
================  =====  ================================

Finite-state machine
====================

`STATE` values emitted to the GCS log:

===========  ============================
State value  Meaning
===========  ============================
-1           Waiting for EKF origin
0            Arm & automated take-off
1            Cruise to target waypoint
2            Apply Q-update (learning)
3            Reset episode
===========  ============================
  


Learning parameters
===================

- `ALPHA` = 0.1
- `GAMMA` = 0.9
- `EPSILON` decays 0.2 → 0.01 (by x0.98/episode)
- `STABLE_THRESHOLD` = 600 episodes without new best path terminates
  the script automatically.

Data logging
============

A helper script `log.lua` runs alongside the RL agent and writes a plain-text
CSV called `flight_log.csv` in the working directory.  Each row contains a
snapshot of battery metrics and vehicle state taken every 50 ms; a "RESET"
row separates episodes so they are easy to segment later.
Header::

 elapsed_ms, fsm_state, flight_mode, armed,
 lat_deg, lon_deg, alt_m,  
 vel_n, vel_e, vel_d,   
 roll, pitch, yaw, yaw_bhv,  
 gyro_x, gyro_y, gyro_z,  
 volt_v, curr_a

Extending the example
=====================

- **New tasks**: replace the waypoint list or reward definition.
- **Plane/Rover**: the same reset patch works for other SITL vehicles;
  only the take-off logic must be adapted.


Troubleshooting
===============

- *Script does nothing*: check `SCR_ENABLE` and ensure the table key
  90 is not used by other scripts.
- *Vehicle drifts after reset*: confirm that the patch was rebuilt.
- *High CPU/storage load*: lower `SIM_SPEEDUP` or reduce the logging rate.

References
==========

- Mid-Term update blog post: https://discuss.ardupilot.org/t/gsoc-2025-sitl-ai-reinforcement-learning-concept-script/135423/4
- Source code: https://github.com/b-andreoni/GSoC
- Fork with patch: https://github.com/b-andreoni/ardupilot

- Sutton, R. & Barto, A. **Reinforcement Learning: An Introduction** (2nd ed.). MIT Press, 2018. 
- *Reinforcement Learning By the Book* - video series: https://www.youtube.com/playlist?list=PLzvYlJMoZ02Dxtwe-MmH4nOB5jYlMGBjr
