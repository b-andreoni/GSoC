# TECS RL Scripts  
_GSoC 2025 • `scripts/reinforcement_learning/tecs`_

This folder contains lightweight Lua applets that use **reinforcement learning (ε-greedy Q-learning)** to auto-tune key gains of the **Total Energy Control System (TECS)** in ArduPlane SITL.

---

## 1. What is TECS?  
TECS is ArduPilot’s algorithm that balances an aircraft’s **total energy**:

| Energy term | Controlled by | TECS job |
|-------------|---------------|----------|
| **Potential** (altitude) | **Throttle** | Add or bleed total energy |
| **Kinetic** (airspeed)   | **Pitch**    | Trade speed ↔ height |

Correct gain settings keep the aircraft on-speed and on-altitude without oscillations or throttle surges. Manual tuning is time-consuming; these scripts automate the search.

---

## 2. File overview

| Script | What it tunes | Fixed values | Typical use |
|--------|---------------|--------------|-------------|
| `spdweight.lua` | `TECS_SPDWEIGHT` (0 – 2) | TIME CONST = 5 s, PTCH_DAMP = 0.2 | Decide how much pitch should prioritise **altitude (0)** vs **speed (2)**. |
| `time_const.lua` | `TECS_TIME_CONST` (3 – 10 s) | SPDW = 0, PTCH_DAMP = 0.2 | Remove height “porpoising” by slowing or speeding the TECS response. |
| `ptch_damp.lua` | `TECS_PTCH_DAMP` (0 – 0.6) | SPDW = 0, TIME CONST = 10 s | Add extra damping to erase residual pitch wobble. |
| `combo.lua` | **All three** (grid search) | — | Joint optimisation once individual best ranges are known. |

Each script:

1. Teleports to home (SIM API), arms, takes off and enters a small **LOITER**.  
2. Runs an episode (`~30 s`), computing cost = `|altErr| + 0.1 × power`.  
3. Updates a Q-table and restarts; stops when cost plateaus.

---

## 3. Quick start

```bash
cp GSoC/scripts/reinforcement_learning/tecs ~/scripts
sim_vehicle.py -v ArduPlane --map
```

- *Stop / resume*: set SIM_ERES_ENABLE param (1 = run, 0 = idle).

- *Watch progress*: Q-updates print to the MAVProxy console.

- *Best combo found (example)*: SPDW=0.0, TIME_CONST=10, PTCH_DAMP=0.4.
