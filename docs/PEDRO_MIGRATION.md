# Pedro Pathing Migration

The project has been migrated from **Road Runner** to **Pedro Pathing** for autonomous path following.

## What Was Done

- **Dependencies**: Added Pedro Pathing (`com.pedropathing:ftc:2.0.6`, telemetry, fullpanels) and the Pedro Maven repo. Road Runner is still present for unconverted opmodes and the RR package; remove it from `TeamCode/build.gradle` and `build.dependencies.gradle` once migration is complete.
- **Drive**: `PedroDrive` (`Toros/Drive/PedroDrive.java`) wraps Pedro’s `Follower` and converts between field coordinates (center origin, inches) and Pedro’s coordinate system (0–144, bottom-left origin).
- **Pose types**: `org.firstinspires.ftc.teamcode.util.Pose2d` and `Vector2d` are used for poses/vectors so Drawing, PoseBridge, and autos no longer depend on Road Runner types.
- **Actions**: `util.Action`, `Actions.runBlocking`, `SequentialAction`, `ParallelAction`, `SleepAction`, `FollowPathAction` replace Road Runner’s action API.
- **Teleop**: `MainDrive` uses `PedroDrive` and `setTeleOpDrive` / `update()` for drive and pose.
- **Constants**: `pedroPathing/Constants.java` maps your existing RR tuning to Pedro: `MecanumDrive.Params` (inPerTick, maxWheelVel, gains, feedforward) → `FOLLOWER_CONSTANTS`, `DRIVE_CONSTANTS`, `PATH_CONSTRAINTS`; `PinpointLocalizer.Params` (parYTicks, perpXTicks, encoder directions) → `LOCALIZER_CONSTANTS`. Tune further with Pedro’s Tuning opmode if needed.
- **Autos converted to Pedro**: Auto2025RedNear, Auto2025BlueNear, Auto12Ball, Auto2025RedFar, Auto2025BlueFar. They use `PedroDrive`, `buildPath` / `buildPathChain`, and `FollowPathAction`. Path waypoints and sequences are unchanged.

## What You Still Need To Do

1. **Convert remaining autos** (they still use Road Runner and `MecanumDrive`):
   - Auto2025RedFarMovesRight  
   - Auto2025RedNearOdometryAim  
   - AutoV2  
   - AutoV3  
   - BlueNearExp  

   For each: switch imports to `org.firstinspires.ftc.teamcode.util.*` and `PedroDrive`, replace `drive.actionBuilder(...).build()` with `new FollowPathAction(drive, drive.buildPath(start, end))` or `buildPathChain(waypoints)`, and replace `MecanumDrive` / `drive.localizer.getPose()` / `drive.updatePoseEstimate()` / `pose.heading.toDouble()` with `PedroDrive` / `drive.getPose()` / `drive.update()` / `pose.heading`. Use `Actions.runBlocking(this, action)`.

2. **Remove Road Runner**: After all autos are converted, delete the RR drive/localizer/tuning/message classes (or repurpose the RR folder) and remove the Road Runner dependencies from `TeamCode/build.gradle` and `build.dependencies.gradle`.

3. **Tune Pedro**: `pedroPathing/Constants.java` already uses values derived from your RR `MecanumDrive.Params` and `PinpointLocalizer.Params`. Use Pedro’s tuning tools (e.g. copy `Tuning.java` from [Pedro Quickstart](https://github.com/Pedro-Pathing/Quickstart)) to fine-tune. Pedro’s coordinate system: origin bottom-left, field 0–144; we convert to center-origin inches in `PedroDrive`.

4. **MeepMeep**: The MeepMeep pathing module still uses Road Runner for visualization. You can keep it for design and then translate waypoints into Pedro paths in code, or switch to Pedro’s path visualizer (see below).

## Pedro path visualizer: in-code vs web

You can use **either** (or both):

- **In-code (FullPanels)**  
  The project already includes `com.bylazar:fullpanels` in `build.dependencies.gradle`. FullPanels provides path/telemetry panels that run **inside the FTC app** (e.g. on the Driver Station or when using FTC Dashboard). Use it to view paths and robot state while running your code. No separate app or URL needed.

- **Web visualizer**  
  Open **https://visualizer.pedropathing.com/** in a browser to design and optimize paths (waypoints, obstacles, timing). Use this to plan paths off the robot, then implement the same waypoints in your autonomous (e.g. in `PedroDrive.buildPath` / `buildPathChain`).

Use the **web** for designing and tuning paths; use **FullPanels** in-code for live path/pose view when running the robot or simulations.

## Dashboard toggle: FTC Dashboard vs Panels

**MainDrive** supports switching between **FTC Dashboard** (acmerobotics) and **[Panels](https://panels.bylazar.com)** ([FTControl Panels](https://github.com/ftcontrol/ftcontrol-panels)) for telemetry and visualization.

- Set **`usePanelsDashboard`** on the FTC Dashboard config (MainDrive) or in code: `true` = use Panels, `false` = use FTC Dashboard (default).
- When **usePanelsDashboard** is true: telemetry is sent to Panels (and Driver Station); the custom field overlay is not sent (use Panels’ Field widget). Open https://panels.bylazar.com in a browser and connect to the robot.
- When **usePanelsDashboard** is false: behavior is unchanged (FTC Dashboard + TelemetryPacket field overlay).

The project already includes **FullPanels** (`com.bylazar:fullpanels`); the Panels toggle uses it when the Panels class is available at runtime.

## References

- [Pedro Pathing](https://pedropathing.com/)
- [Pedro Quickstart](https://github.com/Pedro-Pathing/Quickstart)
- [Path Builder](https://pedropathing.com/docs/pathing/reference/path-builder)
