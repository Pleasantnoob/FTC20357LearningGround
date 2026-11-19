# FTC 20357 BioBuzz robot (2026-27)

Robot code for the BioBuzz season. Branch `BIOBUZZ2627`. Everything we write lives in `TeamCode/src/main/java/teamcode/`; `FtcRobotController/` is the stock FTC SDK, so don't edit it.

## Build
Check it compiles (no robot needed):
```bash
./gradlew :TeamCode:compileDebugJavaWithJavac
```
On Windows, clone with `git config core.longpaths true` or the checkout fails.

## Git
- Work on `BIOBUZZ2627`. Commit with a short summary line, then a few plain sentences on what changed and why.
- Compile before every commit.
- Ask before pushing. Push with `git push origin BIOBUZZ2627`.

## Getting code onto the robot (ADB)
ADB (Android Debug Bridge) is how the laptop talks to the Control Hub. It comes with Android Studio, in `<Android SDK>/platform-tools/adb` (the SDK path is in `local.properties`). Add that folder to PATH or use the full path.

1. Connect the laptop to the robot's Wi-Fi (Control Hub network), or plug in USB-C.
2. Connect ADB (Wi-Fi only; USB connects by itself):
   ```bash
   adb connect 192.168.43.1:5555
   adb devices
   ```
   `adb devices` should list the hub. If it says "offline" or "unauthorized", run `adb disconnect` and connect again.
3. Build and install. Either press Run in Android Studio with the TeamCode config, or:
   ```bash
   ./gradlew :TeamCode:installDebug
   ```
   The Robot Controller app restarts by itself after the install. If it doesn't:
   ```bash
   adb shell am force-stop com.qualcomm.ftcrobotcontroller
   adb shell monkey -p com.qualcomm.ftcrobotcontroller 1
   ```
4. Pick the OpMode on the Driver Station as usual.

Useful while debugging:
- Live log of the robot app: `adb logcat -s RobotCore`. Crashes show up as `FATAL EXCEPTION` or `AndroidRuntime`, so grep for those.
- Pull the full log: `adb pull /sdcard/robotControllerLog.txt`
- Web pages on the robot Wi-Fi: FTC Dashboard at `192.168.43.1:8080/dash`, Pedro tuner at `192.168.43.1:10158`, robot config / manage page at `192.168.43.1:8080`.
- If the code installed but an OpMode is missing from the list, check it isn't `@Disabled` and has `@TeleOp` / `@Autonomous`.

## Stack
- FTC SDK 12.0. Pedro Pathing 3.0.1 (core, revhub, tuning) plus Ivy commands for the autos.
- goBILDA Pinpoint odometry.
- FTC Dashboard (the slothboard fork) at `192.168.43.1:8080/dash`.
- Pedro's auto-tuner is a web page at `192.168.43.1:10158`. The tuners are registered with `@Tuner` in `pedro/Tuning.java`.

## Layout
| Path | What |
|---|---|
| `RobotContainer/Robot.java` | Builds every subsystem: `new Robot(hardwareMap)` |
| `RobotContainer/DriveProcesses.java` | TeleOp controls (button map at the top of the file) |
| `RobotContainer/SubsystemCommands.java` | Ivy commands the autos use |
| `Subsystems/` | Drivetrain, Intake, Shooter, Turret, Vision |
| `Subsystems/Microsystems/` | TurretGate, MatchLoader (flower ramp), TurretHood |
| `ComplexMath/ShotRegression.java` | Distance → flywheel RPM + hood (coefficients from Desmos) |
| `ComplexMath/Storage/PoseStorage.java` | Static pose and alliance handed from auto to TeleOp |
| `Utility/DashboardField.java` | Field drawing for FTC Dashboard |
| `OpModes/TeleOp/MainDrive.java` | Match TeleOp |
| `OpModes/Auto/` | Autos (the base autos drive and park; shooting isn't written yet) |
| `CustomTuning/TuningStation.java` | One dashboard OpMode for flywheel PID, servo positions, RPM/hood testing |
| `pedro/Constants.java` | Drive, Pinpoint and Foresight config (still from the old robot, needs retuning) |
| `Tests/` | Old test OpModes, all `@Disabled` |

## Hardware config names (must match the Driver Station config)
| Device | Name |
|---|---|
| Drive motors | `lf_motor`, `lb_motor`, `rf_motor`, `rb_motor` (500 rpm, belted) |
| Intake / transfer | `intakeMotor`, `transferMotor` (1620 rpm) |
| Flywheel | `leftShooterMotor` (6000 rpm, 28 CPR) |
| Turret | `turretServoLeft`, `turretServoRight` |
| Gate / match loader / hood | `turretGate`, `matchLoader`, `turretHood` |
| Odometry / camera | `pinpoint`, `Webcam 1` (Arducam) |

The turret is two Axon MAX servos, 1:1. Both servo gears drive one center gear, so both servos spin the same way and `reverseRight = false`. Setting it to `true` makes them fight. Travel is ±175°. The turret pivot is 15.021 mm forward of the robot center (from CAD).

## Tuned switches (`RobotContainer/Tuned.java`)
`TURRET`, `HOOD`, `GATE`, `MATCH_LOADER`, `FLYWHEEL` are all `false` until the real values are in the code. While one is false, MainDrive and the autos leave that mechanism off: its servos stay limp, the flywheel gets 0 power. TuningStation ignores them and has its own on/off switches (everything starts off there too; the turret has `TURRET_MODE` with LEFT_ONLY / RIGHT_ONLY so one servo can be tuned while the other is limp). Only flip a switch after the user gives the tuned values.

## How the main pieces work
- **Turret:** jumps straight to the target, leads the target by `latencySec`, and aims from the pivot point. There's no aim dead zone. `isOnTarget()` gates shooting.
- **Shoot on the move:** aims at a "virtual goal" shifted by robot velocity × (feed time + flight time). Toggle with `Turret.SHOOT_ON_THE_MOVE` or gamepad 2 right bumper. `FEED_TIME_SEC` and `TOF_A`/`TOF_B` are placeholders.
- **Shooter:** feedforward (kS + kV·rpm, scaled by battery voltage) plus a P term. The gate opens only when RPM and turret are both ready, then stays open for the volley.
- **Targets:** right now the turret picks our alliance's cell on the robot's side of the field (y = 72 split). The plan is to switch to "aim at our cell that's up", using AprilTags.

## Game facts that matter for code (manual TU09: https://ftc-resources.firstinspires.org/ftc/game/manual)
- Each alliance has a HIVE in the field center: a see-saw with 2 CELLS, and exactly one CELL faces up. Launching into the up CELL tips it, and then the other one is up. Only the up CELL scores.
- AprilTags (36h11, 3.25 in) come in clusters of 4, stuck on the BOTTOM of each CELL facing DOWN. IDs:
  - 30–33: red, far from the audience
  - 34–37: red, audience side
  - 38–41: blue, audience side
  - 42–45: blue, far from the audience
- `AprilTagGameDatabase.getBioBuzzTagLibrary()` has these clusters but no field positions.

## Values still to measure or tune
- Pinpoint pod offsets and directions, then Foresight (Pedro web tuner).
- Flywheel kS/kV/kP (TuningStation).
- Gate, match loader and hood positions (placeholders).
- The ShotRegression Desmos coefficients.
- Shoot-on-the-move feed time and flight time.
- Camera mount pose and Arducam calibration. Last season: fx=fy=909.96, cx=634.50, cy=347.54 at 1280×720.
- The reset poses in MainDrive.

## Conventions
- Tunable values are `public static` in `@Config` classes so they show up in FTC Dashboard. Dashboard edits are lost on restart, so copy final numbers into the code.
- Comments: short and plain. Say what a thing does and why, and mark placeholders clearly.
- Check that it compiles before committing.
- Ask before pushing.
- Keep this file and code comments free of other teams' names and private info (emails, passwords, keys).
