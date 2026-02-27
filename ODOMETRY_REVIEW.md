# Odometry tracking review

## 1. Build (verified)

From project root (PowerShell):
```powershell
.\gradlew.bat :TeamCode:assembleDebug
```
See `BUILD_INSTRUCTIONS.md` for details. Build completes successfully.

---

## 2. Turret odometry (teleop)

**Where:** `TeamCode/.../Toros/Drive/Subsystems/Turret.java`

**What it does:** Tracks turret angle from the **turret motor encoder** and combines it with robot heading for field-relative aim.

**Constants (do not change without verifying on robot):**
- **TICKS_PER_MOTOR_REV = 384.5** — encoder ticks per full motor shaft rotation.
- **GEAR_RATIO = 2/5** — turret output rotation per motor revolution (e.g. 144° output per 360° motor).

**Formulas:**
- **Encoder → angle (degrees):**  
  `currentAngle = (ticks / 384.5) * 360 * (2/5) [+ heading]`  
  The `+ heading` is robot heading (degrees) so the result is in **field frame**.
- **Field angle → target ticks:**  
  `targetPos = (384.5 * (targetAngle + heading)) / 360 * (5/2)`

**Modes:**
- **runTurretGyro()** — Uses **Pinpoint IMU** for live robot heading (`driver.getHeading(DEGREES)`). Turret angle = encoder angle + heading (field frame).
- **runTurretNoGyro(k)** — Uses **frozen heading k** (set when you press Y for lock-on). Same math with `k` instead of live heading.

**Important:** Turret and `PinpointLocalizer` both use the same hardware device `"pinpoint"`. Turret uses it only for **IMU heading**; drive pose uses it for **position + heading** in `PinpointLocalizer.update()`. Turret’s constructor calls `driver.resetPosAndIMU()` and uses `Turret.PARAMS` (offsets 0,0) for the Pinpoint; when `MecanumDrive` is created it uses `PinpointLocalizer`, which re-applies its own encoder resolution and offsets. Heading from the IMU is unaffected by those offsets.

---

## 3. Drive / pose odometry (Pinpoint 2-wheel)

**Where:** `TeamCode/.../RR/PinpointLocalizer.java` and `MecanumDrive.java`

**What it does:** Uses the **GoBilda Pinpoint** (2-wheel odometry + IMU) to estimate robot pose (x, y, heading) in inches and radians.

**Params (in PinpointLocalizer):**
- **parYTicks, perpXTicks** — encoder offsets (tick units) for the parallel and perpendicular pods. Tuned for your robot; changing them changes where the “robot” frame is.

**Flow:**  
`MecanumDrive` has a `PinpointLocalizer`. Each loop, `mecanumDrive.updatePoseEstimate()` → `localizer.update()` → Pinpoint `driver.update()`, then pose is read from `driver.getPosX/Y()` and `driver.getHeading()` (in inches and radians). That pose is used for Dashboard drawing and for `distanceX`, `distanceY`, `distance` in MainDrive.

**Note:** `PinpointLocalizer` uses **REVERSED** for the parallel encoder and **FORWARD** for the perpendicular one. If odometry direction is wrong, encoder directions are the first thing to check (see TODOs in that file).

**FTC field frame (Dashboard and start/goal):** Origin at **center of mat**. **+X** = to the right when facing the red wall; **+Y** = away from the red wall (toward blue). Robot position drawn on FTC Dashboard is **this same pose** from Pinpoint (pods + IMU). If the robot appears in the wrong place: (1) confirm start pose in MainDrive (red* / blue*) matches where you actually start; (2) tune **PinpointLocalizer.PARAMS** (parYTicks, perpXTicks) and encoder directions so dead-reckoned position matches the field.

---

## 4. Heading sources (avoiding mix-ups)

| Use | Source | Units |
|-----|--------|--------|
| Turret field angle (gyro mode) | `Turret.driver.getHeading(AngleUnit.DEGREES)` (Pinpoint) | degrees |
| Turret lock-on frozen heading | `k = turret.botHeading` when Y pressed | degrees |
| Drive pose (RR) | PinpointLocalizer `driver.getHeading(UnnormalizedAngleUnit.RADIANS)` | radians |
| IntakeV2 / calcShot | `IntakeV2.getHeading()` ← `heading` ← `pinpoint.getHeading()` in runLauncher() | radians |
| DriveTrain botHeading | Set in driveRobotCentric from `IntakeV2.getHeading()` | radians |
| MainDrive telemetry "heading" | `drivetrain.getHeading()` = `botHeading` | radians |

**Bug fix applied:**  
- `IntakeV2.getHeading()` used to return `h` (launcher height constant 0.69). It now returns `heading` (Pinpoint heading in radians).  
- Lock-on now sets `k = turret.botHeading` (degrees from Pinpoint in the turret) instead of `drivetrain.getHeading()`, so lock-on uses the correct heading in the units the turret expects.

---

## 5. Pinpoint device shared by Turret and MecanumDrive

Both get the same hardware: `hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint")`.

- **Turret constructor:** Sets resolution, offsets (Turret.PARAMS = 0,0), directions, then `resetPosAndIMU()`.
- **MecanumDrive** (and thus **PinpointLocalizer**) is created after Turret and gives the same driver different resolution/offsets/directions and uses it for pose.

So the **last** code to configure the Pinpoint wins for **position**; **heading** from the IMU is the same either way. Init order in MainDrive: Turret, then MecanumDrive, so drive odometry uses PinpointLocalizer’s config. Turret only needs heading, which is correct.

---

## 6. Turret field-relative hold and angles (MainDrive)

**Field-relative hold:** When not locked on, MainDrive aims turret at goal by default: `angleToGoalDeg` = atan2(goal − pose) then wrap to **[0, 360)** (RR convention). Dpad Up = 0°, Dpad Down = 180°; else `fieldHoldAngle = angleToGoalDeg`. Left stick X nudge updates hold. **Angle APIs:** `getTurretAngleField()` / `getTurretAngleRobot()` return [0, 360). **Alliance:** Dpad Left = Blue, Right = Red; start and goal from `red*` / `blue*` (Dashboard-tunable). Pose and goal are in FTC field frame (see §3); robot position on Dashboard = Pinpoint (pods+IMU).

---

## 7. Quick checklist

- [x] Turret encoder → angle uses 384.5 and 2/5; formulas documented in `Turret.java`.
- [x] Turret feedforward uses velocity 0 for position hold (correct for SimpleMotorFeedforward).
- [x] Lock-on uses `k = turret.botHeading` (degrees).
- [x] `IntakeV2.getHeading()` returns `heading` (radians), not `h`.
- [x] Field-relative turret hold uses odometry; getTurretAngleField/Robot for wrap-around.
- [ ] If turret or drive odometry is wrong: re-check 384.5 and gear ratio (turret), or encoder directions and pod offsets (Pinpoint), before changing constants.
