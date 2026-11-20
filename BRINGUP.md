# Robot bring-up checklist

Everything to test and tune to get from "it compiles" to "it works", in order. Each step depends on the ones before it, so don't skip ahead (the turret can't aim until odometry is right, and the shot table means nothing until the turret aims).

Tools: TuningStation OpMode + FTC Dashboard (`192.168.43.1:8080/dash`), Pedro tuner (`192.168.43.1:10158`), tape measure, protractor or phone level app, multimeter, phone slow-mo video.

**Dashboard edits are lost when the robot restarts.** Write every final number down (or send it to whoever is updating the code) so it gets typed into the code.

Until a mechanism's switch in `RobotContainer/Tuned.java` is `true`, MainDrive and the autos leave it off. Flip each switch only after that mechanism is tuned (step 10).

---

## 0. Before powering on

- [ ] **Driver Station config names** match exactly. A missing or misspelled name crashes init in every OpMode:
  - Motors: `lf_motor`, `lb_motor`, `rf_motor`, `rb_motor`, `intakeMotor`, `transferMotor`, `leftShooterMotor`
  - Servos: `turretServoLeft`, `turretServoRight`, `turretGate`, `turretHood`, `matchLoaderLeft`, `matchLoaderRight`
  - I2C: `pinpoint` (goBILDA Pinpoint)
  - Camera: `Webcam 1`
  - Optional analog input: `turretEncoder` (only if the Axon feedback wire is plugged in)
- [ ] **Axon programmer**, on both turret servos (details in CLAUDE.md):
  - Loss-of-signal = **Release**. If it's set to Hold, the one-servo-at-a-time tuning makes the two servos fight.
  - Full range
  - Power limit: about 66% on hub ports, about 85% on a Servo Power Module
  - Sensitivity as fine as it goes without humming
  - No inversion
- [ ] **Power the programmer** from USB-C or up to 9 V on the XT60. Never the 12 V robot battery.
- [ ] **Turret servos on a REV Servo Power Module** if you have one. Two MAX servos can pull more current than the hub's servo ports supply.
- [ ] **Turret wiring:** everything that rides on the turret (wires, camera cable) reaches ±175° without pulling or snagging. Turn it by hand with the power off.
- [ ] **Encoder wire voltage:** if you wire it, check with a multimeter that it never goes above 3.3 V before plugging it into the hub.
- [ ] **Charged battery,** and **deploy the code** (CLAUDE.md → "Getting code onto the robot").
- [ ] **Gamepads:** in Logitech mode with the mode light on X (MainDrive telemetry warns about this).

## 1. Basic directions (MainDrive)
Every `Tuned` switch is still off here, so only the drive and intake move.

- [ ] **Driving:** stick forward drives forward, strafe goes the right way, and right stick X turns the right way. Fix wrong wheels in `Drivetrain.java` AND `pedro/Constants.java` (both must match).
- [ ] **Intake:** right trigger pulls balls in on both the intake and the transfer. Right bumper spits them out. Fix in `Intake.java`.
- [ ] **Loop time:** check "loop ms" on the dashboard. Under 20 ms is good. Bulk reads are on, so slow loops point at something else.

## 2. Odometry (Pedro tuner, `192.168.43.1:10158`)
The turret aims from the robot's position, so this has to be right first.

- [ ] **Pinpoint pod offsets** (Offsets Identification: spin 180° CCW, then stop). Copy them into `pedro/Constants.java` (`xPodOffset`, `yPodOffset`). The numbers there now are the old robot's.
- [ ] **Pod directions and pod type** (`xPodDirection`, `yPodDirection`, `podType`). Check by hand:
  - Push the robot 24 in forward: x goes **+24**.
  - Push it 24 in left: y goes **+24**.
  - Turn it left (counter-clockwise): heading goes **up**.

  If any of these is wrong, the turret will aim mirrored.
- [ ] **Distance check:** push the robot 96 in. If it reads 96 ± 0.5, good. If not, check the pod type.
- [ ] **Pedro drive tuning:**
  1. Forward and strafe velocity
  2. Zero-power acceleration
  3. Foresight
  4. Translational, heading and drive PIDF

  Robot mass in `pedro/Constants.java`. All the Foresight numbers there are from the old robot.
- [ ] **Field size:** our code assumes the visualizer's field, **141.5 in** wall to wall. Measure your practice field. If it's different, change `Utility/Field.java`, because autos on a different-size field end up offset at the far walls.

## 3. Dashboard field view
- [ ] Drive forward and watch the robot icon on the dashboard field. If the picture is rotated or flipped, change `DashboardField.ROTATION_DEG` (0/90/180/270) or `MIRROR_X`.

## 4. Turret (TuningStation, `TURRET_MODE`)
- [ ] **`LEFT_ONLY`:**
  - Slide `TURRET_RAW_POS` until the turret points straight forward. That's `Turret.centerPos`.
  - Set it to 0 and to 1, and measure the angle between the two spots. That's `Turret.servoRangeDeg`.
  - Check a higher position turns the turret **left** (counter-clockwise from above). If not, set `invertDirection = true`.
  - Check nothing hits at the ends. If something does, lower `travelLimitDeg`.
- [ ] **`RIGHT_ONLY`:** the same numbers should give the same spots. If the turret turns the opposite way, flip `reverseRight`.
- [ ] **`BOTH_RAW` at `centerPos`:** nudge `Turret.rightOffsetPos` until "hub current A" is lowest and the buzzing stops. Feel both servos after a minute; they shouldn't be hot.
- [ ] **`FIXED_DEG`:** 0, +90 and −90 should point forward, left and right. If 90 comes out as 80 or 100, `servoRangeDeg` is off.
- [ ] **Encoder (if wired):**
  - At center, read "turret encoder V". That's `encoderVoltsAtCenter`.
  - Go to +90 and take the voltage change. `encoderServoDegPerVolt` = 90 ÷ that change.
  - Check "turret encoder deg" follows "turret cmd deg".
- [ ] **`turretDegPerSec`** (no encoder): time a 180° swing with slow-mo video. `turretDegPerSec` = 180 ÷ seconds. Too high and the gate opens early; too low and every shot waits.
- [ ] **`AUTO_AIM`:**
  - Set the robot's pose (gamepad 2 Back at a reset spot, or run from an auto).
  - The thin dashboard line and the real turret both point at the middle of our up cell.
  - Drive across y = 70.75 and check it switches cells.
  - If it's always off to one side, adjust the cell targets (`Turret.redHighX/Y` etc.). Those are estimates from the manual drawings.
- [ ] **`latencySec`:** spin the robot in place and drive past the hive while watching the aim.
  - Lags behind: raise it.
  - Leads ahead: lower it.
- [ ] **`onTargetTolDeg`** (1.5°): if good shots wait too long, try 2–3.

## 5. Flywheel (TuningStation, `FLYWHEEL_ON`)
- [ ] **Direction and sign:** spin at a low `TARGET_RPM` (1000).
  - The wheel must spin the shooting direction, and "rpm" must read **positive**.
  - If rpm is negative, the PID runs away to full power. Fix with the motor direction in `Shooter.java`.
- [ ] **kS / kV / kP** (TuningStation header has the steps):
  1. kP = 0. Raise kS until the wheel just turns.
  2. Raise kV until "rpm" settles on "target rpm".
  3. Raise kP for fast recovery after a shot without wobble.
- [ ] **Top speed:** check the highest RPM the wheel actually reaches on a normal battery. Keep `ShotRegression.MAX_RPM` under it.
- [ ] **`READY_RPM_TOLERANCE` (40):** shots at the edge of the tolerance should still score. If they don't, tighten it.

## 6. Gate, hood, match loader
- [ ] **Gate:** raw tester (`RAW_SERVO_NAME = turretGate`) → `TurretGate.closeGatePos` / `openGatePos`. Closed must hold a ball; open must not jam.
- [ ] **Hood:** raw tester (`turretHood`) → `TurretHood.minPos` / `maxPos` / `defaultPos`. Find the hard stops and stay a little inside them. Note which direction is a higher arc.
- [ ] **Match loader:** `LOADER_MODE` LEFT_ONLY → RIGHT_ONLY → BOTH_RAW (`MatchLoader.rightOffsetPos`) → GAMEPAD. Set `stowedPos` / `deployedPos` and `reverseRight`. Stowed has to be inside the starting size limit.

## 7. Shot table (Desmos)
- [ ] **Collect points:** at 5–6 distances across where you'll shoot from (about 40–100 in):
  - Turret on `AUTO_AIM`, `USE_REGRESSION = false`.
  - Adjust `TARGET_RPM` and `HOOD_POS` until shots score.
  - Write down the dashboard's "distance to target", the RPM and the hood value.
- [ ] **Fit in Desmos:** rpm = A + B·d + C·d² and hood the same way. Type the coefficients into `ShotRegression` (`RPM_A/B/C`, `HOOD_A/B/C`).
- [ ] **Check:** `USE_REGRESSION = true` and shoot from distances in between your points.
- [ ] **Distance limits:** set `MIN_DIST` / `MAX_DIST` to the range you actually measured.

## 8. Shoot on the move
- [ ] **Velocity check:** drive around and graph "vel x pinpoint" against "vel x pose-diff" (and the y pair). They should overlap. If the Pinpoint one is flipped or wrong, set `USE_PINPOINT_VELOCITY = false`.
- [ ] **`FEED_TIME_SEC`:** slow-mo video from the gate opening to the ball leaving the flywheel.
- [ ] **Flight time:** slow-mo video at 2–3 distances. Fit `TOF_A + TOF_B·distance`. The ball is light, so expect less than the placeholders.
- [ ] **Test:** shoot while strafing slowly past the hive, then faster. If it's worse than standing still, turn it off (gamepad 2 RB or `SHOOT_ON_THE_MOVE = false`).

## 9. MainDrive reset poses
- [ ] Pick a spot on each alliance side the robot can drive into the same way every time, like a corner or against a wall. Measure its pose in the visualizer frame and set `MainDrive.RED_RESET_*` / `BLUE_RESET_*`. The current numbers are placeholders.

## 10. Turn things on
- [ ] **One switch at a time:** flip `Tuned.TURRET`, `HOOD`, `GATE`, `MATCH_LOADER`, `FLYWHEEL` to `true` in the code, re-deploy, and test in MainDrive. The "NOT TUNED (off)" line on the Driver Station shows what's still off.
- [ ] **Full controls test:** gamepad 1 shoot (LT), collect (RT), eject (RB) and loader (LB). Gamepad 2 manual turret, A/B, trim, hood, alliance X/Y, shoot-on-the-move RB and pose reset (Back).

## 11. Autos
- [ ] **Placement:** for the garden auto, put the robot at exactly **(53.75, 6.75) facing 90°**. That's 53.75 in from the left wall (by the red garden) to the robot's center, against the audience wall. Blue is the same spot rotated: (87.75, 134.75) facing 270°. A 1 in placement error is a 1 in error in every path.
- [ ] **First runs:** run **Red Garden + Flowers** with the robot in the air first, then slowly on the field:
  - The garden sweep picks up all 4.
  - The flower creeps line up with the flower openings (path 3 and path 5 ends).
  - The robot never holds more than 4 (rule G407).
  - The park ends inside the loading zone.
- [ ] **Time it:** it has to finish inside 30 s. To speed up, shorten `VOLLEY_MS`, `FLOWER_DWELL_MS` and `READY_TIMEOUT_MS` (dashboard group "GardenFlowersAuto").
- [ ] **Blue version:** run **Blue Garden + Flowers** once to confirm the rotation.
- [ ] **TeleOp hand-off:** after an auto, start MainDrive without resetting the pose. The turret should already be aiming at the right cell, since the pose and alliance carry over.
- [ ] **Old autos:** `Auto/Base/*` and `RedGardenCornerSoloAuto` were rotated into the current frame but never run, and the base autos don't shoot. Check them in the visualizer before using them.

---

## Known risks to watch for
- **Turret aims mirrored or way off:** a Pinpoint direction is wrong (step 2), or the alliance is wrong (gamepad 2 X/Y).
- **Turret servos buzzing or hot:** loss-of-signal isn't Release, `reverseRight` is wrong, or `rightOffsetPos` isn't tuned. Two fighting MAX servos can also brown out the hub, which makes everything glitch at once.
- **Flywheel screaming to full power:** the RPM sign is negative (step 5).
- **Gate never opens in MainDrive:** check, in order:
  - Flywheel not reaching target within 40 rpm
  - Turret not "on target"
  - `Tuned.GATE` / `Tuned.FLYWHEEL` still off
- **Auto drifts at the far side of the field:** Pedro isn't tuned, or the field isn't 141.5 in (`Field.SIZE`).
- **Hive cell positions** in `Turret` are calculated from the manual's drawings (cells about 15 in from center). Check them on a real field.
- **The auto's volley plan assumes** the first 4 tip our first cell (3 NECTAR + 4) and 8 tip the second. If the hive doesn't tip when expected, the side rule may aim at a cell that's down.
- **Camera / AprilTag up-cell detection isn't built yet.** It needs the camera mount position (forward, left, height, tilt) and a fresh calibration.
