# 12-Ball Autonomous

## Overview

- **3 preloaded** balls + **9** picked up from spike marks = **12 balls** shot.
- Uses the **same logic as teleop**: Pinpoint odometry, IntakeV2 regression (hood + flywheel from distance), Turret aim at goal (field angle, wrap ±180°).
- Road Runner trajectories: `strafeToLinearHeading`, `strafeTo` with optional `TranslationalVelConstraint`.

## OpMode

- **Auto 12-Ball (Blue)** — Blue alliance: start (-55, -47) @ -130°, goal (-70, -64).

## Flow

1. **Start** → drive to **shoot position** (38, -52) facing goal.
2. **Rev + aim** 2 s (hood/flywheel from distance, turret to goal).
3. **Shoot 3** (feed 4 s).
4. **Spike 1** (-12, -53): drive there while **intake** runs 3 s.
5. **Back to shoot** → rev + aim → **shoot 3**.
6. **Spike 2** (12, -53): drive + intake 3 s.
7. **Back to shoot** → rev + aim → **shoot 3**.
8. **Spike 3** (30, -55): drive + intake 3 s.
9. **Back to shoot** → rev + aim → **shoot 3**.
10. **Stop** launcher.

## Tuning

- **Shoot position** `SHOOT_X`, `SHOOT_Y`: distance to goal should be in range where your regression is tuned (~30–50 in).
- **Spike waypoints** `SPIKE*_X`, `SPIKE*_Y_END`: match your field and spike mark layout (DECODE: three spikes on audience side).
- **Rev time** (2 s) and **shoot feed time** (4 s): increase if flywheel doesn’t reach speed or 3 balls don’t clear.
- **Intake time** (3 s) per spike: tune so 3 balls are pulled in.

## Red alliance

To add Red: mirror Y (positive) and heading (e.g. 130°). Start (-55, 47), goal (-70, 64). Duplicate the opmode and flip sign on all Y and heading constants, or add an alliance selector in init.

## IntakeV2 / Turret in auto

- **IntakeV2**: `setHoodAndFlywheelFromDistance(dist)` and `runLauncherAuto(feed)` — same regression as teleop. `runIntakeAuto(true/false)` for pickup.
- **Turret**: `botHeading` and `targetAngle` set from current pose and goal; `runTurretGyro()` each loop (same wrap and PID as teleop).
- **PoseBridge**: end pose is saved so teleop can start from the auto end position.
