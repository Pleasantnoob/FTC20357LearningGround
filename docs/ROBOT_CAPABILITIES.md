# FTC Robot Capabilities (MainDrive + Subsystems)

This document summarizes what the robot can do, how systems work together, and logic checks from a full code review.

---

## 1. Drive & Localization

- **Drive**: Mecanum, robot-centric. Left stick = translate, right stick X = rotate. X and R toggles (dpad left/right) scale movement/rotation to 25%.
- **Localization**: GoBilda Pinpoint (odometry pods + IMU). Pose and velocity come from `MecanumDrive` / `PinpointLocalizer`. Same Pinpoint driver is used by IntakeV2 for heading/vel (shared hardware).
- **Start pose**: Set from alliance (red/blue). Red: (-55, 47) @ 130°; Blue: (-55, -47) @ -130°. Auto can pass end pose via `PoseBridge` so teleop starts from auto’s end position.
- **IMU re-zero**: Driver 1 Start re-zeros Pinpoint IMU when robot is stationary (reduces drift).

---

## 2. Turret

- **Default**: Holds a field angle. Target = angle to goal (with optional velocity compensation). Robot-centric angle is wrapped to **[-180°, 180°)** so the turret always takes the **shortest path** and doesn’t spin past ±180° (avoids wire tangling).
- **Aim sources**:  
  - Dpad Up = 0°, Dpad Down = 180°.  
  - Otherwise target = angle to goal (or to virtual goal when velocity comp is on).  
  - Left stick X (gamepad2) = nudge turret angle.  
  - A = set target to 0°. X = reset turret encoder and set target to current field angle.
- **Lock-on (Y/B)**: Y = enable. Freezes robot heading (k) and uses vision to set turret target from AprilTag bearing (IDs 20 or 24). B = disable. When bearing error ≤ 2°, target is held (no drift). Rumble when aimed within 2°.
- **Heading source**: Turret uses `turret.botHeading` from MainDrive, which is set from **pose.heading** (RR/Pinpoint), so turret and pose use the same heading.

---

## 3. Launcher (IntakeV2)

- **Modes** (in order of precedence):
  1. **Left bumper (gamepad2)** – fixed preset (hood 0.8, target -1480 ticks/s).
  2. **Manual (Dashboard)** – `manualMode` true: use `manualHoodAngleDeg` and `manualTargetVel` (hood 40–70°, velocity in ticks/s).
  3. **AirSort** – when `airSortEnabled` and `airSortPresetActive`: hood and velocity from fast/slow shot (see AirSort below).
  4. **Auto from distance** – regression from odometry distance: cubic hood, linear flywheel (hood 40–70°, velocity from regression). Distance from `MainDrive.getDistanceFromOdometry()`.
- **Shooting**: Right trigger (gamepad2) runs transfer + intake only when `|launcher velocity − target| ≤ SHOOT_VEL_TOLERANCE` (default 20 ticks/s).
- **Distance for velocity comp**: Time-of-flight for virtual goal uses `MainDrive.getDistance()` (camera range when tag 20/24 visible and in 12–200 in, else odometry). Launcher auto and AirSort use odometry only (`getDistanceFromOdometry()`).
- **Flywheel**: PID + feedforward (kS, kV, kA), tunable via FTC Dashboard (IntakeV2). Hood deadband reduces servo jitter.

---

## 4. Velocity Compensation

- **Purpose**: Aim at a **virtual goal** so the note still lands at the real goal while the robot is moving.
- **Formula**: `aimGoal = goal − velocity × (gain × timeOfFlight)`. Backwards → virtual goal farther; sideways → aim opposite to velocity. Tuned by `turretVelocityCompGain` (default 1.0).
- **Time of flight**: From current hood and flywheel target (manual or auto from distance). Uses `ShotPhysics.timeInAir`; when in manual mode uses `launchSpeedMPSFromTicksPerSec(manualTargetVel)`.
- **Display**: FTC Dashboard shows goal (green), virtual goal (orange when comp on), and yellow aim line robot → aim point.

---

## 5. AirSort (Motif-Based Fast/Slow Shot)

- **Off by default**: `IntakeV2.airSortEnabled = false` until Operator Back is pressed to turn it on.
- **Toggle**: Operator Back (gamepad2) toggles AirSort on/off. When turned **on**, shot index resets to 0 and AprilTag decimation is set to 2 for better obelisk tag range.
- **Motif**: From AprilTags **21, 22, 23** (obelisk). Stored **once** when first seen; not overwritten. 21 → G,P,P; 22 → P,G,P; 23 → P,P,G (first/second/third shot).
- **Ball at launcher**: Only **c3** (top/launcher sensor) is used for the shot decision (no c1/c2) for consistency.
- **Shot decision**: For current shot index (0,1,2), desired color = `motif[shotIndex]`. If ball at launcher (c3) matches → **fast shot** (min time in air); else → **slow shot** (max time in air). Hood 40–70°, flywheel ≤ 1700 ticks/s; regression used as floor so shots stay makeable; hood band ±12° from regression.
- **Index advance**: When “ball left launcher” is detected (c3 goes from ball present to empty), a **time-based** advance is scheduled (default 250 ms). After that delay, shot index increments (0→1→2, then stays 2). Delay is tunable via `airsortAdvanceDelayMs` on Dashboard.
- **Distance**: AirSort preset uses **odometry distance** only (`getDistanceFromOdometry()`).

---

## 6. Intake & Transfer

- **Intake**: Right trigger = pull in; left trigger = reverse. B = hard stop (intake + transfer off).
- **Transfer**: Right bumper = transfer (logic depends on c3 blue). Left bumper = transfer at -0.35. When right trigger (shoot) is held, transfer is controlled in `runLauncher()` for feeding.

---

## 7. Vision & AprilTag

- **Processor**: AprilTag with current game tag library, custom lens intrinsics, decimation 3 (2 when AirSort is on).
- **Goal tags (20, 24)**: Used for lock-on (turret track), camera distance when in range, optional camera relocalization (Dashboard), and rumble when aimed.
- **Obelisk tags (21, 22, 23)**: Used only for AirSort motif; stored once.
- **Camera relocalization**: Optional (`useCameraRelocalization`). When enabled and blue goal tag (20) is seen, robot pose is estimated from tag and drawn on Dashboard (orange). Odometry path still drawn (blue). Red goal tag (24) reloc not implemented (getTagFieldPosition only returns blue goal).

---

## 8. Alliance & Goal

- **Alliance**: Dpad Left = Blue, Dpad Right = Red (in init and during run). Sets start pose and goal; pose is not reset when switching.
- **Goal**: Red default. Red goal (-70, 64); Blue goal (-70, -64). Field frame: origin center; from red side, -X = right, -Y = forward.

---

## 9. Logic Checks & Fixes Applied

- **Turret wrap**: Robot-centric angle and target are in [-180°, 180°); target position is nearest equivalent so turret always takes shortest path → no continuous spin, less wire wrap.
- **Lock-on**: When bearing error ≤ 2°, target is set to **current** turret angle (hold). Previously multiplied by 1.01, which caused drift; that’s fixed.
- **AirSort**: Single sensor (c3) for decision; index reset on toggle; time-based advance; regression floor and hood band keep shots makeable.
- **Distance**: Velocity comp can use camera when available; launcher auto and AirSort use odometry only (consistent and stable).
- **Heading**: Turret uses RR pose heading; DriveTrain uses `IntakeV2.getHeading()` which is updated in `runLauncher()` from the same Pinpoint driver (shared hardware), so they stay in sync.
- **Removed**: Stray `import com.sun.tools.javac.Main` from IntakeV2.

---

## 10. What the Robot Can Do (Summary)

| Capability | Description |
|------------|-------------|
| **Drive** | Mecanum, robot-centric, with speed toggles. Pinpoint odometry + IMU for pose and velocity. |
| **Aim at goal** | Turret holds field angle to goal (or virtual goal with velocity comp). Shortest-path wrap in [-180°, 180°). |
| **Velocity compensation** | Virtual goal = goal − v× gain× T; aim line and virtual goal on Dashboard. |
| **Lock-on** | Y/B to track goal AprilTag (20 or 24); rumble when aimed within 2°. |
| **Shoot** | Regression-based hood + flywheel from distance; shoot when within velocity tolerance (right trigger). |
| **Manual launcher** | Dashboard: manual mode, hood angle, flywheel velocity, PID/FF tuning. |
| **AirSort** | Operator Back toggles. Motif from tags 21/22/23 (stored once). Launcher c3: correct color → fast shot, wrong → slow shot. Index advances after shot with 250 ms delay (tunable). |
| **Alliance** | Red/Blue; start and goal set from Dpad; pose not reset on switch. |
| **Camera reloc** | Optional pose from blue goal tag on Dashboard. |
| **Debug** | FTC Dashboard telemetry; optional file log (maindrive_log.txt) for turret/pose/camera. |

---

## 11. Control Reference (MainDrive)

| Control | Action |
|---------|--------|
| **Driver (gamepad1)** | Left stick = drive, right stick X = turn. X toggle = strafe scale. R toggle = turn scale. Dpad L/R = alliance. Start = re-zero IMU (stationary). B = intake/transfer stop. |
| **Operator (gamepad2)** | Left trigger = spin flywheel. Right trigger = feed when at speed. Left bumper = launcher preset. Back = AirSort on/off. Y = lock-on, B = lock-off. Dpad Up/Down = turret 0°/180°. Left stick X = turret nudge. A = turret target 0°. X = turret encoder reset + hold. |
