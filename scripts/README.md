# /mission
## \:computer: sim\:set\_pose & sim\_set\_pose\_mission.lua

This short guide shows how to use `sim:set_pose` inside ArduPilot SITL and explains the companion script **`sim_set_pose_mission.lua`** that teleports the vehicle when it arms, performs an automated take‑off, then kicks off the loaded mission in **AUTO** mode — perfect for batch‑testing or reinforcement‑learning loops where you need fully repeatable starts.

---

### 1. `sim:set_pose`

```lua
sim:set_pose(instance, loc, orient, vel_bf, gyro_rads)
```

| Argument    | Meaning                               | Typical value                 |
| ----------- | ------------------------------------- | ----------------------------- |
| `instance`  | SITL instance index                   | `0`                           |
| `loc`       | Absolute location (`Location` object) | `ahrs:get_origin()` + offsets |
| `orient`    | Attitude quaternion                   | `quat:from_euler(r,p,y)`      |
| `vel_bf`    | Velocity vector in **body** frame     | `Vector3f(0,0,0)`             |
| `gyro_rads` | Gyro rates (rad/s)                    | `Vector3f(0,0,0)`             |

Calling this function **instantly teleports** your simulated drone and overrides its velocity and gyro readings, letting you start every episode from an identical state.

---

### 2. `set_pose+mission.lua`

The script waits for an arm event, forces a configurable pose with `sim:set_pose`, switches to **GUIDED** for take‑off, then slides into **AUTO** and begins the first mission command. Everything is driven by a handful of parameters, so you can trigger it from a GCS or via a parameter file.

#### 2.1 Configurable parameters

| Parameter              | Purpose                                              | Default |
| ---------------------- | ---------------------------------------------------- | ------- |
| `SIM_APOS_ENABLE`      | Master switch (0 = idle, 1 = active)                 | 0       |
| `SIM_APOS_POS_N/E/D`   | Position offsets (m) relative to origin              | 0       |
| `SIM_APOS_VEL_X/Y/Z`   | Body‑frame velocity (m/s)                            | 0       |
| `SIM_APOS_RLL/PIT/YAW` | Initial attitude (deg)                               | 0       |
| `SIM_APOS_GX/GY/GZ`    | Initial gyro (deg/s)                                 | 0       |
| `SIM_APOS_MODE`        | Flight‑mode to switch to after pose (‑1 = no change) | ‑1      |
| `WAIT_BEFORE_ARM`      | Loops (\~50 ms each) before auto‑arming              | 250     |
| `WAIT_AFTER_AUTO`      | Loops to wait once AUTO confirmed                    | 60      |

*(All parameters live in the **********`SIM_APOS_`********** table; see the script for exact names.)*

#### 2.2 How it works

1. **Enable** — set `SIM_APOS_ENABLE = 1` and load a mission.
2. **Pose fix** — on the very first execution the script calls `sim:set_pose` with your offsets & attitude.
3. **Countdown** — waits `WAIT_BEFORE_ARM` ticks (\~10 s) before sending an **ARM** command.
4. **Take‑off** — switches to **GUIDED**, issues `vehicle:start_takeoff(alt)`, then requests **AUTO**.
5. **Mission start** — after `WAIT_AFTER_AUTO` ticks (\~3 s), `mission:set_current_cmd(0)` pushes the autopilot into the first waypoint.

The loop runs every 50 ms while enabled, falling back to 500 ms when idle to minimise CPU load.

---

### 3. Tips

* Keep `EK3_OPTN = 10` (Direct State Feed) to avoid long EKF convergence after teleporting.
* If you see `SCRIPTING: mem alloc failed`, raise `SCR_HEAP_SIZE` or trim print statements.
* Want a different launch profile? Tweak `WAIT_BEFORE_ARM`, `WAIT_AFTER_AUTO`, or bypass **GUIDED** entirely by setting `SIM_APOS_MODE` straight to `AUTO`.

# /reset
## :computer: sim:set_pose & sim_arming_pos.lua

This document describes a simple script (`sim_arming_pos.lua`) that forces the vehicle’s pose when it arms.

---

## sim_arming_pos.lua

This script forces a predefined pose at the moment the vehicle arms.

### How It Works

1. Wait for `ENABLE = 1`
2. Detect arm event (transition from disarmed → armed)
3. On arming:
   - Build quaternion from `RLL`, `PIT`, `YAW`
   - Create `Vector3f` for velocity and gyro
   - Build location with offsets `POS_N`, `POS_E`, `POS_D`
   - Call `sim:set_pose(0, loc, quat, vel, gyro)`
   - If `MODE >= 0`, call `vehicle:set_mode(MODE)`
4. The script runs in a 50 ms loop and only triggers on the arm transition.

## Example:
```bash
param set SIM_APOS_ENABLE 1
param set SIM_APOS_POS_N 100
param set SIM_APOS_POS_N -50
arm throttle
```
