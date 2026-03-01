# Velocity compensation – step-by-step logic

## What it does

Velocity compensation adjusts **where the turret aims** so that when the robot is moving, the shot still lands on the goal. It does this by shifting the aim point using the robot’s velocity and the note’s **time of flight**.

---

## Step-by-step (code flow)

### 1. Get robot velocity (field frame, in/s)

- **Source:** `driveVel = mecanumDrive.updatePoseEstimate()` → `driveVel.linearVel` is in **robot frame** (Road Runner: x = forward, y = left).
- **Transform to field frame:**  
  `worldVx = cos(heading)*robotVel.x - sin(heading)*robotVel.y`  
  `worldVy = sin(heading)*robotVel.x + cos(heading)*robotVel.y`  
  So `(worldVx, worldVy)` is the robot’s velocity in **field coordinates**, in **inches per second**.

### 2. Get distance to goal (inches)

- **Source:** `distForShotVel = getDistance()`  
  - Prefers **camera** distance (AprilTag 20/24) when valid (12–200 in); otherwise **odometry** distance.
- Used only to compute **hood angle and launch speed** for the time-of-flight (next step). It does **not** change the goal position.

### 3. Get hood angle and launch speed for time-of-flight

- **Regression (auto):**  
  `hoodSpeedVel = ShotPhysics.hoodAndSpeedFromDistanceInches(distForShotVel)`  
  → `[hoodAngleDeg, speedMPS]` from the distance-based model.
- **Manual mode:**  
  - Hood: `IntakeV2.manualHoodAngleDeg`  
  - Speed: `IntakeV2.launchSpeedMPSFromTicksPerSec(IntakeV2.manualTargetVel)` (ticks/s → m/s).

So the **actual** hood and flywheel setup (manual or auto) are used for the comp.

### 4. Compute time of flight (seconds)

- **Formula:**  
  `timeOfFlightS = ShotPhysics.timeInAir(speedMPSForVelComp, Math.toRadians(hoodDegForVelComp))`  
  with `timeInAir(speedMS, thetaRad) = 2 * speedMS * sin(theta) / G`.
- Same physics as the shot (launch speed and angle), so the comp matches the real shot.

### 5. Compute compensated aim point (inches)

- **Default:** `aimGoalX = goalX`, `aimGoalY = goalY`.
- **With velocity comp (when enabled and gain > 0):**  
  `aimGoalX = goalX - turretVelocityCompGain * (worldVx * timeOfFlightS)`  
  `aimGoalY = goalY - turretVelocityCompGain * (worldVy * timeOfFlightS)`  
  So: **aim = goal − gain × (velocity × time of flight)**.  
  Units: in − (in/s × s) = in ✓

### 6. Turret aim angle

- **Angle to aim point:**  
  `angleToGoalRad = atan2(aimGoalY - pose.position.y, aimGoalX - pose.position.x)`  
  then converted to degrees and wrapped 0–360, plus `turretAngleOffsetDeg`.
- When not locked on, `turret.targetAngle` is set to this angle so the turret points at `(aimGoalX, aimGoalY)`.

---

## Logic check

- **Units:** `goal` (in), `worldVx/worldVy` (in/s), `timeOfFlightS` (s) → `goal - vel*T` is in inches ✓  
- **Hood/speed:** Time of flight uses the same hood and speed as the real shot (manual or auto) ✓  
- **Distance for T:** In auto, distance comes from `getDistance()` (camera or odom); in manual, hood/speed are from manual settings ✓  

**Sign convention:** The code uses **aim = goal − vel×T**. So when the robot moves in the +X direction, it aims at a point with **smaller** X than the goal (i.e. “behind” the goal along the direction of motion). That makes the note land at that point, not at the goal. If you want the note to land **on** the goal while moving, you usually want to **lead** the goal (aim **ahead**), i.e. **aim = goal + vel×T**. If you see the note consistently missing in the direction of motion, try **negative** `turretVelocityCompGain` (e.g. -1.0) so the comp becomes goal + vel×T.

---

## Summary: how it works

1. **Pose and velocity** are updated from the localizer; velocity is converted to field frame (in/s).
2. **Distance to goal** (camera or odom) is used to get **hood angle and launch speed** (from regression or manual).
3. **Time of flight** is computed from that launch speed and hood angle (same physics as the shot).
4. **Aim point** is set to **goal − gain×(velocity × time of flight)** (in inches).
5. The **turret** is commanded to point at that aim point.

So velocity compensation **does** use the actual hood angle and flywheel speed (manual or auto) for the time-of-flight, and the rest of the logic is consistent in units and frame. The only thing to verify on the field is whether the sign of the compensation (lead vs lag) matches your intent; use `turretVelocityCompGain` (or its sign) to correct that.
