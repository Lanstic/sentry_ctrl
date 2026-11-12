# 2026 Infantry Robot Firmware

This document reflects the current source code. See the repository root `readme.md` for the full Chinese guide.

## Controls

### Remote controller

| Input | Function |
|---|---|
| Right stick | Chassis forward/backward and strafing |
| Left stick | Gimbal yaw and pitch |
| Left switch up | Gimbal enabled; chassis follows gimbal |
| Left switch middle | Gimbal enabled; chassis auto-spin |
| Left switch down | Normally zero-force; with both switches down, keyboard mode |
| Right switch down | Friction wheels off; shooting stopped |
| Right switch middle | Friction wheels on; feeder stopped |
| Right switch up | Friction wheels on; continuous feeding |

The side dial is decoded as channel 4 but is not used. The current input path does not provide a single-shot command.

### Keyboard and mouse

Put both switches down first.

| Input | Function |
|---|---|
| `Ctrl+Z` | Auto-spin mode |
| `Ctrl+X` | Chassis follows gimbal |
| `Ctrl+C` | Chassis and gimbal zero-force |
| `WASD` | Chassis translation |
| Mouse movement | Gimbal yaw/pitch |
| `F` | Turn gimbal by approximately 180 degrees |
| `Q` / `E` | Friction wheels on / off |
| Left mouse button | Continuous forward feeding |
| Right mouse button | Reverse feeding for unloading/jam clearing |

The servo task uses `Z/X/C/V` to increase servo 1–4 PWM and `Shift+Z/X/C/V` to decrease it. Therefore `Ctrl+Z/X/C` also changes servo 1–3 PWM; this is a known key conflict.

When switching from auto-spin to gimbal-follow mode, yaw alignment keeps the sign of the last spin command and limits command slew to 12 rad/s². A later equivalent alignment point is selected if the nearest one cannot be reached with smooth braking. Slew limiting remains active after alignment; zero-force mode cancels the transition immediately. See [the implementation and validation notes](../底盘同向回正修改说明.md). Hardware validation is still required.

## Calibration

Keep both switches down. Hold both sticks down-and-in for 2 seconds, then complete the second gesture within 20 seconds:

| Calibration | Second gesture, 2 seconds | Result |
|---|---|---|
| Gyroscope | Sticks down-and-out | Approximately 20 seconds of stationary bias sampling |
| Gimbal | Sticks up-and-out | Automatic pitch maximum/minimum and yaw limit search |
| Chassis motor IDs | Sticks up-and-in | Send the `0x700` quick-ID command |

Keep the robot level and clear all moving parts. Gimbal calibration is motor-driven; do not manually force yaw.

## Communication

- Chassis motors: CAN1, group command ID `0x200`.
- Yaw, pitch and feeder: CAN1; feedback IDs `0x205/0x206/0x207`, group command ID `0x1FF`.
- Friction wheels: open-loop PWM on TIM1 CH1/CH2.
- BMI088: data-ready interrupts and SPI DMA.
- Remote controller: 18-byte DBUS/SBUS frame.

CAN2 is initialized, but both `CHASSIS_CAN` and `GIMBAL_CAN` currently map to `hcan1`.

## Current shooting parameters

- Friction PWM: off `1000`, active ramp target `1600`, defined upper value `1800`.
- Feeding becomes available after both friction ramps reach `1500`.
- Continuous feeder speed: `7.0f`.
- Right mouse reverse feeder speed: `-4.0f`.

The friction wheels have no feedback and are not part of offline detection.
