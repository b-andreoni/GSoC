# :computer: sim:set_pose & sim_arming_pos.lua

This document explains how to use `sim:set_pose` in ArduPilot SITL Lua and describes a simple script (`sim_arming_pos.lua`) that forces the vehicle’s pose when it arms.

---

## sim:set_pose

```lua
sim:set_pose(instance, loc, orient, vel_bf, gyro_rads)
```

- **instance**: SITL instance index (usually `0`)
- **loc**: Location from `ahrs:get_origin()`
- **orient**: Quaternion (e.g. `quat:from_euler(roll, pitch, yaw)`)
- **vel_bf**: `Vector3f` velocity in body frame
- **gyro_rads**: `Vector3f` gyro rates in rad/s

This function teleports the drone and sets its velocity and gyro readings.

---

## sim_arming_pos.lua

This script forces a predefined pose at the moment the vehicle arms.

### Configurable Parameters

| Name      | Meaning                | Default |
|-----------|------------------------|---------|
| ENABLE    | Enable script (0/1)    | 0       |
| POS_N     | North offset (m)       | 0       |
| POS_E     | East offset (m)        | 0       |
| POS_D     | Down offset (m)        | 0       |
| VEL_X     | Velocity X (m/s)       | 0       |
| VEL_Y     | Velocity Y (m/s)       | 0       |
| VEL_Z     | Velocity Z (m/s)       | 0       |
| RLL       | Roll (°)               | 0       |
| PIT       | Pitch (°)              | 0       |
| YAW       | Yaw (°)                | 0       |
| GX        | Gyro X (°/s)           | 0       |
| GY        | Gyro Y (°/s)           | 0       |
| GZ        | Gyro Z (°/s)           | 0       |
| MODE      | Optional flight mode   | -1      |

---

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