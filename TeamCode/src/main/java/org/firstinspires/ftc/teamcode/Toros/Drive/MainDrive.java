package org.firstinspires.ftc.teamcode.Toros.Drive;

import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.RR.Drawing;
import org.firstinspires.ftc.teamcode.RR.MecanumDrive;
import org.firstinspires.ftc.teamcode.RR.PinpointLocalizer;
import org.firstinspires.ftc.teamcode.RR.PoseBridge;
import org.firstinspires.ftc.teamcode.Toros.Drive.CameraRelocalization;
import org.firstinspires.ftc.teamcode.Toros.Drive.ShootingZones;
import org.firstinspires.ftc.teamcode.Toros.Drive.ShotPhysics;
import org.firstinspires.ftc.teamcode.Toros.Drive.Subsystems.AirSort;
import org.firstinspires.ftc.teamcode.Toros.Drive.Subsystems.DriveTrain;
import org.firstinspires.ftc.teamcode.Toros.Drive.Subsystems.IntakeV2;
import org.firstinspires.ftc.teamcode.Toros.Drive.Subsystems.Turret;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.util.LinkedList;
import java.util.List;

/**
 * Main teleop: drive, odometry, turret, intake, FTC Dashboard.
 *
 * Pose (robot position + heading): from PinpointLocalizer = GoBilda Pinpoint odometry pods + IMU (same as RR).
 * Start position and goal: origin at center of mat. Standing on red side: X to the right is negative (-X), Y forward is negative (-Y).
 * So -X = right, +X = left; -Y = forward (toward center/blue), +Y = back (toward red wall).
 * If the robot appears in the wrong place on Dashboard, tune PinpointLocalizer.PARAMS (parYTicks, perpXTicks) and encoder directions.
 *
 * Turret: When !lockedOn we aim at goal: angleToGoal uses pose + goal. Angles in [0, 360) (RR convention).
 * Dpad Up = 0°, Dpad Down = 180°; else angleToGoal. Lock-on (Y) = vision; B = exit. Left stick X = nudge.
 *
 * Alliance: Dpad Left = Blue, Dpad Right = Red; start and goal set from red* / blue*.
 *
 * Airsort: Operator Back toggles on/off. When on, motif from AprilTags 21/22/23 + ball color from c1,c2,c3
 * → fast shot (correct color) or slow shot (wrong color). Shot index auto-advances when ball leaves launcher.
 */
@TeleOp(name = "MainDrive")
@Config
public class MainDrive extends LinearOpMode {
    private static final boolean USE_WEBCAM = true;

    /** Start pose and goal (inches, degrees). Set from alliance mode (blue/red). From red side: -X = right, -Y = forward (see class doc). */
    public static double startX = -55.0;
    public static double startY = 47.0;
    public static double startHeadingDeg = 130.0;
    public static double goalX = -70.0;
    public static double goalY = 70.0;

    /** Red alliance: same as blue but +Y. Start (-50, 50); goal -X left, +Y; heading 128°. */
    public static double redStartX = -50.0, redStartY = 50.0, redStartHeadingDeg = 128.0;
    public static double redGoalX = -70.0, redGoalY = 70.0;
    /** Blue alliance: mirror of red with -Y and -heading. Start (-50, -50); goal -X -Y; heading -128°. */
    public static double blueStartX = -50.0, blueStartY = -50.0, blueStartHeadingDeg = -128.0;
    public static double blueGoalX = -70.0, blueGoalY = -70.0;

    /** If turret mechanical 0° is not aligned with robot +X, add offset here (e.g. 90 or -90). Tune on Dashboard. */
    public static double turretAngleOffsetDeg = 0.0;

    /** Velocity compensation: virtual goal = goal - velocity * (gain * timeOfFlight). Turret = aim point; hood/speed = distance for shot. */
    public static boolean turretVelocityCompensation = false;
    /** Turret aim: lower = less lead (e.g. 0.4–0.6 if turret comp was too strong). */
    public static double turretVelocityCompGain = 0.5;
    /** Hood/speed: distance to virtual goal. Higher = more comp (e.g. 1.2–1.8 if hood/speed comp was too weak). */
    public static double hoodSpeedVelocityCompGain = 1.5;
    /** Set each loop for IntakeV2: velocity-compensated distance for hood/flywheel when turretVelocityCompensation is on. */
    public static double distanceForHoodSpeedInches = Double.NaN;

    /** When true, use AprilTag to estimate robot pose for display/aim; when false, use odometry only. */
    public static boolean useCameraRelocalization = false;

    /** Min R+G+B for a color sensor to count as "artifact present" (color-agnostic). */
    public static int artifactThreshold = 80;
    /** Gamepad2 rumble duration (ms) when entering close zone with 2+ artifacts. */
    public static int closeZoneRumbleMs = 300;
    /** GoBilda LED (servo PWM 0–1). Tune on FTC Dashboard -> MainDrive.LedConfig */
    @Config
    public static class LedConfig {
        public static double posNormal = 0.35;
        public static double posReady = 0.5;
        /** Red: shown when < 2 balls detected. Tune per Product Insight #4. */
        public static double posRed = 0.2799;
    }

    /** Position hold: when sticks below deadband, robot resists external pushes. Tune deadband if too sensitive. */
    public static boolean positionHoldEnabled = true;
    public static double positionHoldDeadband = 0.08;
    public static double driveScale = 0.75;

    public AprilTagProcessor aprilTag;
    public String[] motif = new String[3];
    public VisionPortal visionPortal;
    DriveTrain drivetrain;
    IntakeV2 intake;
    Turret turret;
    AirSort airSort;
    public ColorSensor c3;
    /** true = airsort mode: launcher uses fast/slow shot from motif + ball color. Toggle: Operator Back. */
    private boolean airSortActive = false;
    private boolean prevBack = false;
    private boolean prevStart = false;
    private boolean prevReadyToShoot = false;
    private boolean justResynced = false;
    /** Delay (ms) from shot detected until shot index advances. Tune in Dashboard if index was too slow. */
    public static double airsortAdvanceDelayMs = 250;
    List<LynxModule> allHubs;
    Servo led;
    private boolean lockedOn = false;
    /** true = blue alliance (start/goal from blue*), false = red (red*). Dpad Left = Blue, Dpad Right = Red. */
    private boolean mode = true;
    /** Frozen heading (deg) when manual turret mode is on; used by turret instead of live gyro. */
    double k = 0;
    /** Robot-relative turret angle (deg) in manual mode. Controlled by gamepad2 left stick X. Clamped to ±170. */
    private double manualTurretDeg = 0;
    /** Field angle (deg) for turret when !lockedOn: angleToGoal (aim) or 0 / 180 from dpad. [0, 360). */
    private double fieldHoldAngle = 0;

    /** Distance to goal (inches); set each loop from odometry pose. When goal tag (20 or 24) is seen, prefer camera. */
    public static double distance = 0;
    public static double distanceX = 0;
    public static double distanceY = 0;
    /** Camera range to goal tag in inches (ftcPose.range is meters). Set when tag 20 or 24 seen; NaN otherwise. Same IDs as lock-on. */
    public static double cameraDistanceInches = Double.NaN;
    private static final double M_TO_IN = 39.37007874;
    MecanumDrive mecanumDrive;

    /** Camera-derived pose for Dashboard only; null when no tag 20/24 seen. */
    private Pose2d cameraPose = null;

    /** Pose history for Dashboard path (localization viz); same as used in tuning opmodes. */
    private final LinkedList<Pose2d> poseHistory = new LinkedList<>();

    public static double getDistanceX(){
        return distanceX;
    }

    public static double getDistanceY(){
        return distanceY;
    }
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());


        // Bulk caching reduces USB traffic to expansion hubs and can improve loop time.
        allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        initAprilTag();
        drivetrain = new DriveTrain(hardwareMap, gamepad1);
        led = hardwareMap.get(Servo.class, "LED");
        // If Auto just ran, use its alliance so Teleop starts with same color
        if (PoseBridge.hasAlliance()) {
            mode = PoseBridge.getAlliance();
            PoseBridge.clearAlliance();
        }
        applyAllianceMode();
        // If Auto just ran and saved its end pose, use it so Teleop starts from that position/heading.
        Pose2d initialPose;
        boolean cameFromAuto = PoseBridge.hasPose();
        if (cameFromAuto) {
            initialPose = PoseBridge.getPose();
            PoseBridge.clear();
            startX = initialPose.position.x;
            startY = initialPose.position.y;
            startHeadingDeg = Math.toDegrees(initialPose.heading.toDouble());
        } else {
            initialPose = new Pose2d(startX, startY, Math.toRadians(startHeadingDeg));
        }
        // Create MecanumDrive first so PinpointLocalizer is the only component that configures
        // the Pinpoint (encoder resolution, offsets, directions, resetPosAndIMU). This avoids
        // multiple inits overwriting each other and prevents heading drift from wrong config.
        mecanumDrive = new MecanumDrive(hardwareMap, initialPose);
        mecanumDrive.localizer.setPose(initialPose);
        intake = new IntakeV2(hardwareMap, gamepad1, gamepad2, aprilTag);
        turret = new Turret(hardwareMap, gamepad2, cameFromAuto);
        airSort = new AirSort(aprilTag, intake.c1, intake.c2, intake.c3);

        // During init: Dpad Left = Blue, Dpad Right = Red. Change alliance anytime before Play.
        while (!isStarted() && opModeIsActive()) {
            if (gamepad1.dpad_left) {
                mode = true;
                applyAllianceMode();
                mecanumDrive.localizer.setPose(new Pose2d(startX, startY, Math.toRadians(startHeadingDeg)));
            } else if (gamepad1.dpad_right) {
                mode = false;
                applyAllianceMode();
                mecanumDrive.localizer.setPose(new Pose2d(startX, startY, Math.toRadians(startHeadingDeg)));
            }
            telemetry.addLine("--- INIT: Change alliance before Play ---");
            telemetry.addData("Alliance", mode ? "BLUE" : "RED");
            telemetry.addData("Start", "%.0f, %.0f @ %.0f deg", startX, startY, startHeadingDeg);
            telemetry.addData("Goal", "%.0f, %.0f", goalX, goalY);
            telemetry.addData(">", "Dpad L=Blue  Dpad R=Red  Play=start");
            telemetry.update();
            sleep(20);
        }
        waitForStart();

        // Debug log: uses app context so it works on Control Hub. Pull with:
        //   adb pull /sdcard/Android/data/com.qualcomm.ftcrobotcontroller/files/maindrive_log.txt
        // If that path doesn't exist, try: adb shell run-as com.qualcomm.ftcrobotcontroller ls files
        // Delete after use: adb shell rm /sdcard/Android/data/com.qualcomm.ftcrobotcontroller/files/maindrive_log.txt
        File logDir = hardwareMap.appContext.getExternalFilesDir(null);
        if (logDir == null) logDir = hardwareMap.appContext.getFilesDir();
        File logFile = new File(logDir, "maindrive_log.txt");
        FileWriter logWriter = null;
        try { logWriter = new FileWriter(logFile, true); logWriter.append("--- run " + System.currentTimeMillis() + " ---\n"); logWriter.flush(); } catch (IOException e) { /* ignore */ }
        int logCounter = 0;
        double lastLogTicks = -1e9, lastLogCurAng = -1e9, lastLogTgtAng = -1e9, lastLogBotHd = -1e9;
        double lastLogPoseX = -1e9, lastLogPoseY = -1e9, lastLogTargetPos = -1e9, lastLogRange = -1e9, lastLogCamX = -1e9, lastLogCamY = -1e9;
        final double LOG_ANG_THRESH = 1.5;
        final double LOG_POS_THRESH = 2.0;
        final double LOG_TICKS_THRESH = 20;

        try {
        while (opModeIsActive()) {
            // --- 1. Odometry: update first so pose is valid every loop from the start ---
            PoseVelocity2d driveVel = mecanumDrive.updatePoseEstimate();
            Pose2d pose = mecanumDrive.localizer.getPose();
            distanceX = pose.position.x - goalX;
            distanceY = pose.position.y - goalY;
            distance = Math.hypot(distanceX, distanceY);

            // Pose history for Dashboard path (localization viz)
            poseHistory.add(pose);
            while (poseHistory.size() > 200) poseHistory.removeFirst();

            // Re-zero Pinpoint IMU when Start is pressed (robot should be stationary). Reduces heading drift over time.
            if (gamepad1.start && mecanumDrive.localizer instanceof PinpointLocalizer) {
                PinpointLocalizer pl = (PinpointLocalizer) mecanumDrive.localizer;
                pl.driver.resetPosAndIMU();
                mecanumDrive.localizer.setPose(new Pose2d(pose.position.x, pose.position.y, 0));
            }

            // --- 2. Turret: field-relative aim at goal. Optional velocity comp moves aim point so note lands at goal while moving. ---
            turret.botHeading = Turret.wrapDeg360(Math.toDegrees(pose.heading.toDouble()));
            // Robot velocity in field frame (in/s). Road Runner robot frame: x=forward, y=left; rotate by heading to get world.
            Vector2d robotVel = driveVel.linearVel;
            double h = pose.heading.toDouble();
            double c = Math.cos(h), s = Math.sin(h);
            double worldVx = c * robotVel.x - s * robotVel.y;
            double worldVy = s * robotVel.x + c * robotVel.y;
            // First pass: raw distance for time-of-flight estimate.
            double distRaw = getDistance();
            double[] hoodSpeedVel = ShotPhysics.hoodAndSpeedFromDistanceInches(distRaw);
            double hoodDegForVelComp = IntakeV2.manualMode ? IntakeV2.manualHoodAngleDeg : hoodSpeedVel[0];
            double speedMPSForVelComp = IntakeV2.manualMode
                ? IntakeV2.launchSpeedMPSFromTicksPerSec(IntakeV2.manualTargetVel)
                : hoodSpeedVel[1];
            double timeOfFlightS = ShotPhysics.timeInAir(speedMPSForVelComp, Math.toRadians(hoodDegForVelComp));
            // Turret: virtual aim goal (weaker comp). Hood/speed: velocity-compensated distance (stronger comp).
            double aimGoalX = goalX;
            double aimGoalY = goalY;
            double distForShotVel = distRaw;
            if (turretVelocityCompensation) {
                if (turretVelocityCompGain != 0) {
                    double scaleTurret = turretVelocityCompGain * timeOfFlightS;
                    aimGoalX = goalX - worldVx * scaleTurret;
                    aimGoalY = goalY - worldVy * scaleTurret;
                }
                if (hoodSpeedVelocityCompGain != 0) {
                    double scaleHood = hoodSpeedVelocityCompGain * timeOfFlightS;
                    double vgX = goalX - worldVx * scaleHood;
                    double vgY = goalY - worldVy * scaleHood;
                    distForShotVel = Math.hypot(pose.position.x - vgX, pose.position.y - vgY);
                    hoodSpeedVel = ShotPhysics.hoodAndSpeedFromDistanceInches(distForShotVel);
                }
            }
            distanceForHoodSpeedInches = distForShotVel;  // IntakeV2 uses this when vel comp on for hood/flywheel
            double angleToGoalRad = Math.atan2(aimGoalY - pose.position.y, aimGoalX - pose.position.x);
            double angleToGoalDeg = Turret.wrapDeg360(Math.toDegrees(angleToGoalRad) + turretAngleOffsetDeg);
            if (!lockedOn) {
                if (!justResynced) {
                    if (gamepad2.dpad_up) fieldHoldAngle = 0;
                    else if (gamepad2.dpad_down) fieldHoldAngle = 180;
                    else if (Math.abs(gamepad2.left_stick_x) <= 0.1) fieldHoldAngle = Turret.wrapDeg360(angleToGoalDeg + turret.aimOffsetDeg);  // aim at goal + persistent nudge offset when not nudging
                    // When left_stick_x nudge is active, keep fieldHoldAngle (Turret updates targetAngle, we store it after runTurretGyro)
                    turret.targetAngle = fieldHoldAngle;
                }
            }
            // Alliance can be changed during run: goal/start update (pose not reset)
            if (gamepad1.dpad_left) { mode = true; applyAllianceMode(); }
            else if (gamepad1.dpad_right) { mode = false; applyAllianceMode(); }
            // --- 3. Close + far zone triangles. 8" circle around robot: if it crosses zone boundary, ready to shoot. ---
            int artifactCount = countArtifacts();
            double zoneRadius = ShootingZones.robotZoneCircleRadius;
            boolean circleInClose = ShootingZones.circleIntersectsCloseLaunchTriangle(pose.position.x, pose.position.y, zoneRadius);
            boolean circleInFar = ShootingZones.circleIntersectsFarZoneTriangle(pose.position.x, pose.position.y, zoneRadius);
            boolean readyToShoot = (circleInClose || circleInFar) && artifactCount >= 2;
            if (readyToShoot && !prevReadyToShoot) gamepad2.rumble(closeZoneRumbleMs);  // edge-triggered so we don't rumble every loop
            prevReadyToShoot = readyToShoot;
            // LED: use at least 2 balls (c*.blue() > 150). If 2+ balls → ready/normal; else red.
            boolean inZone = circleInClose || circleInFar;
            double ledPos;
            if (intake.hasAtLeastTwoBalls()) {
                ledPos = inZone ? LedConfig.posReady : LedConfig.posNormal;
            } else {
                ledPos = LedConfig.posRed;
            }
            led.setPosition(ledPos);
            initTelemetry();
            telemetryAprilTag();
            getMotif();
            // Drive: Road Runner active; when sticks idle, position hold resists pushes
            driveWithPositionHold(pose);
            // Turret re-sync: X or Start (fixes gear skip). Center turret first, then press.
            // Encoder reset = turret at 0° robot-relative. Target = angle to goal so turret aims at goal.
            if (gamepad2.xWasPressed() || (gamepad2.start && !prevStart)) {
                turret.resyncEncoder();
                justResynced = true;
                if (lockedOn) {
                    manualTurretDeg = 0;
                    turret.targetAngle = k;  // manual: hold center
                } else {
                    turret.targetAngle = Turret.wrapDeg360(angleToGoalDeg + turret.aimOffsetDeg);  // aim: angle to goal
                    fieldHoldAngle = turret.targetAngle;
                }
                gamepad2.rumble(closeZoneRumbleMs);  // feedback that it worked
            } else {
                justResynced = false;
            }
            prevStart = gamepad2.start;
            if (lockedOn) turret.runTurretNoGyro(k);
            else {
                turret.runTurretGyro();
                fieldHoldAngle = turret.targetAngle;
            }
            lockOn();
            // Airsort toggle: Operator Back. When on: motif + ball color → fast/slow shot.
            if (gamepad2.back && !prevBack) {
                airSortActive = !airSortActive;
                IntakeV2.airSortEnabled = airSortActive;
                if (airSortActive) {
                    airSort.setShotIndex(0);  // reset to first shot when starting a run
                    aprilTag.setDecimation(2);  // better range for obelisk tags 21/22/23
                } else {
                    intake.clearAirSortPreset();
                    aprilTag.setDecimation(3);
                }
            }
            prevBack = gamepad2.back;
            if (airSortActive) {
                airSort.setAdvanceDelayMs((long) airsortAdvanceDelayMs);
                airSort.update();
                intake.setAirSortPreset(airSort.getShotMode(), getDistanceFromOdometry());
            }
            intake.runLauncher();
            intake.runIntake();
            intake.transfer();

            // --- 4. Vision: camera distance (tags 20/24) and optional relocalization. ftcPose.range is meters; we store inches. ---
            cameraPose = null;
            cameraDistanceInches = Double.NaN;
            double headingRad = pose.heading.toDouble();
            for (AprilTagDetection d : aprilTag.getDetections()) {
                if (d.metadata == null) continue;
                if (d.id == 20 || d.id == 24) {
                    cameraDistanceInches = d.ftcPose.range * M_TO_IN;
                }
                Vector2d tagPos = CameraRelocalization.getTagFieldPosition(d.id);
                if (useCameraRelocalization && tagPos != null && d.id == CameraRelocalization.BLUE_GOAL_TAG_ID) {
                    cameraPose = CameraRelocalization.robotPoseFromTag(d, tagPos.x, tagPos.y, headingRad);
                }
            }

            // --- 5. FTC Dashboard: field drawing. Use camera relocalization as robot's estimated location when tag visible, else odometry. ---
            TelemetryPacket packet = new TelemetryPacket();
            Pose2d displayPose = (cameraPose != null) ? cameraPose : pose;  // estimated robot location for field overlay
            Drawing.drawPoseHistory(packet.fieldOverlay(), poseHistory, "#3F51B5");   // path (odometry history)
            Drawing.drawRobot(packet.fieldOverlay(), displayPose, 1, cameraPose != null ? "#FF5722" : "#3F51B5"); // robot at estimated pose (orange = camera, blue = odom)
            if (cameraPose != null) Drawing.drawRobot(packet.fieldOverlay(), pose, 1, "#9FA8DA"); // odometry as lighter circle when camera active (compare)
            Drawing.drawGoal(packet.fieldOverlay(), goalX, goalY, "#4CAF50");         // goal (green)
            Drawing.drawCloseLaunchTriangle(packet.fieldOverlay(),
                    ShootingZones.closeTriX1, ShootingZones.closeTriY1,
                    ShootingZones.closeTriX2, ShootingZones.closeTriY2,
                    ShootingZones.closeTriX3, ShootingZones.closeTriY3, "#80E27E");
            Drawing.drawFarZoneTriangle(packet.fieldOverlay(),
                    ShootingZones.farTriX1, ShootingZones.farTriY1,
                    ShootingZones.farTriX2, ShootingZones.farTriY2,
                    ShootingZones.farTriX3, ShootingZones.farTriY3, "#4FC3F7");
            ShootingZones.Zone shootingZone = ShootingZones.getShootingZone(pose.position.x, pose.position.y, goalX, goalY);
            boolean inField = ShootingZones.isInsideField(pose.position.x, pose.position.y);
            packet.put("shooting_zone", shootingZone.name().toLowerCase());
            packet.put("in_field", inField);
            packet.put("artifact_count", artifactCount);
            packet.put("ready_to_shoot", readyToShoot);
            if (turretVelocityCompensation && turretVelocityCompGain != 0) {
                Drawing.drawVirtualGoal(packet.fieldOverlay(), aimGoalX, aimGoalY, "#FF9800"); // virtual goal (orange) when vel comp on
            }
            Drawing.drawRobotToGoalLine(packet.fieldOverlay(), displayPose, aimGoalX, aimGoalY, "#FFC107"); // aim line from robot to aim point (yellow)
            double distToGoalIn = Math.hypot(pose.position.x - goalX, pose.position.y - goalY);
            double distForShot = getDistance();
            double hoodDeg = Math.max(40, Math.min(70, 70 - intake.getHood() * 30));
            double[] shot = ShotPhysics.speedAndTimeInAir(distForShot, hoodDeg);
            packet.put("dist_to_goal_in", distToGoalIn);
            packet.put("dist_for_shot_in", distForShot);
            packet.put("dist_src", Double.isFinite(cameraDistanceInches) && cameraDistanceInches >= 12 && cameraDistanceInches <= 200 ? "camera" : "odom");
            packet.put("speed_needed_mps", shot[0]);
            packet.put("time_in_air_s", shot[1]);
            packet.put("odom_x", pose.position.x);
            packet.put("odom_y", pose.position.y);
            packet.put("est_x", displayPose.position.x);
            packet.put("est_y", displayPose.position.y);
            packet.put("odom_heading_deg", Math.toDegrees(pose.heading.toDouble()));
            packet.put("pose_src", cameraPose != null ? "camera (reloc)" : "Pinpoint (pods+IMU)");
            packet.put("angle_to_goal_deg", angleToGoalDeg);
            packet.put("turret_field_deg", turret.getTurretAngleField());
            packet.put("turret_robot_deg", turret.getTurretAngleRobot());
            packet.put("field_hold_deg", fieldHoldAngle);
            packet.put("vel_comp_on", turretVelocityCompensation && turretVelocityCompGain != 0);
            packet.put("position_hold", positionHoldEnabled);
            packet.put("launcher_mode", airSortActive ? "airsort (fast/slow)" : (IntakeV2.manualMode ? "manual (Dashboard)" : "auto (from distance)"));
            packet.put("airsort_on", airSortActive);
            if (airSortActive) {
                packet.put("airsort_shot", airSort.getShotIndex() + 1);
                packet.put("airsort_mode", airSort.getShotMode().name());
                packet.put("airsort_ball", airSort.getBallAtLauncher());
                packet.put("airsort_want", airSort.getDesiredColorForCurrentShot());
                packet.put("airsort_motif_stored", airSort.isMotifStored());
                packet.put("airsort_advance_delay_ms", airsortAdvanceDelayMs);
                List<AprilTagDetection> det = aprilTag.getDetections();
                StringBuilder tagIds = new StringBuilder();
                for (AprilTagDetection d : det) { if (tagIds.length() > 0) tagIds.append(","); tagIds.append(d.id); }
                packet.put("airsort_detected_tag_ids", tagIds.length() > 0 ? tagIds.toString() : "none");
            }
            packet.put("manual_hood_deg", IntakeV2.manualHoodAngleDeg);
            packet.put("manual_target_vel", IntakeV2.manualTargetVel);
            packet.put("flywheel_p", IntakeV2.p1);
            packet.put("flywheel_i", IntakeV2.i1);
            packet.put("flywheel_d", IntakeV2.d1);
            packet.put("flywheel_kS", IntakeV2.kS);
            packet.put("flywheel_kV", IntakeV2.kV);
            packet.put("flywheel_kA", IntakeV2.kA);
            packet.put("flywheel_accel", IntakeV2.accel);
            packet.put("aim_goal_x", aimGoalX);
            packet.put("aim_goal_y", aimGoalY);
            packet.put("world_vel_x_in_s", worldVx);
            packet.put("world_vel_y_in_s", worldVy);
            packet.put("time_of_flight_s", timeOfFlightS);
            if (cameraPose != null) {
                packet.put("camera_x", cameraPose.position.x);
                packet.put("camera_y", cameraPose.position.y);
            }
            FtcDashboard.getInstance().sendTelemetryPacket(packet);

            // Log when something changed meaningfully OR at fixed intervals (more frequent = more data)
            if (logWriter != null) {
                logCounter++;
                double rawRange = -1;
                for (AprilTagDetection d : aprilTag.getDetections()) { if (d.metadata != null && d.id == CameraRelocalization.BLUE_GOAL_TAG_ID) { rawRange = d.ftcPose.range; break; } }
                double poseHdDeg = Math.toDegrees(pose.heading.toDouble());
                double camX = cameraPose != null ? cameraPose.position.x : 0, camY = cameraPose != null ? cameraPose.position.y : 0;
                double errDeg = turret.targetOutputDeg - turret.outputDeg;
                double distToGoal = Math.hypot(pose.position.x - goalX, pose.position.y - goalY);
                boolean angChanged = Math.abs(turret.getTurretAngle() - lastLogCurAng) > LOG_ANG_THRESH || Math.abs(turret.targetAngle - lastLogTgtAng) > LOG_ANG_THRESH || Math.abs(turret.botHeading - lastLogBotHd) > LOG_ANG_THRESH;
                boolean posChanged = Math.abs(pose.position.x - lastLogPoseX) > LOG_POS_THRESH || Math.abs(pose.position.y - lastLogPoseY) > LOG_POS_THRESH;
                boolean ticksChanged = Math.abs(turret.motorPosition - lastLogTicks) > LOG_TICKS_THRESH || Math.abs(turret.targetPos - lastLogTargetPos) > LOG_TICKS_THRESH;
                boolean camChanged = Math.abs(rawRange - lastLogRange) > 2 || Math.abs(camX - lastLogCamX) > LOG_POS_THRESH || Math.abs(camY - lastLogCamY) > LOG_POS_THRESH;
                boolean periodic = (logCounter % 15 == 0);
                if (angChanged || posChanged || ticksChanged || camChanged || periodic) {
                    try {
                        logWriter.append(String.format("%d ticks=%.0f tgtPos=%.0f outDeg=%.1f tgtOutDeg=%.1f errDeg=%.1f curAng=%.1f tgtAng=%.1f botHd=%.1f poseHd=%.1f pose=(%.1f,%.1f) distGoal=%.1f turretPwr=%.3f rangeRaw=%.2f camX=%.1f camY=%.1f\n",
                                logCounter, turret.motorPosition, turret.targetPos, turret.outputDeg, turret.targetOutputDeg, errDeg,
                                turret.getTurretAngle(), turret.targetAngle, turret.botHeading, poseHdDeg,
                                pose.position.x, pose.position.y, distToGoal, turret.power, rawRange, camX, camY));
                        logWriter.flush();
                        lastLogTicks = turret.motorPosition; lastLogCurAng = turret.getTurretAngle(); lastLogTgtAng = turret.targetAngle; lastLogBotHd = turret.botHeading;
                        lastLogPoseX = pose.position.x; lastLogPoseY = pose.position.y; lastLogTargetPos = turret.targetPos; lastLogRange = rawRange; lastLogCamX = camX; lastLogCamY = camY;
                    } catch (IOException e) { /* ignore */ }
                }
            }
        }
        } finally {
            if (logWriter != null) try { logWriter.close(); } catch (IOException e) { /* ignore */ }
        }
    }


    public void initAprilTag() {

        // Create the AprilTag processor.
        aprilTag = new AprilTagProcessor.Builder()

                // The following default settings are available to un-comment and edit as needed.
                //.setDrawAxes(false)
                //.setDrawCubeProjection(false)
                //.setDrawTagOutline(true)
                //.setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                .setTagLibrary(AprilTagGameDatabase.getCurrentGameTagLibrary())
                //.setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)
                // == CAMERA CALIBRATION ==
                // If you do not manually specify calibration parameters, the SDK will attempt
                // to load a predefined calibration for your camera.
                //.setLensIntrinsics(578.272, 578.272, 402.145, 221.506)
                // ... these parameters are fx, fy, cx, cy.
                .setLensIntrinsics(909.963833736,909.963833736,634.495942075,347.541434786)

                .build();

        // Adjust Image Decimation to trade-off detection-range for detection-rate.
        // eg: Some typical detection data using a Logitech C920 WebCam
        // Decimation = 1 ..  Detect 2" Tag from 10 feet away at 10 Frames per second
        // Decimation = 2 ..  Detect 2" Tag from 6  feet away at 22 Frames per second
        // Decimation = 3 ..  Detect 2" Tag from 4  feet away at 30 Frames Per Second (default)
        // Decimation = 3 ..  Detect 5" Tag from 10 feet away at 30 Frames Per Second (default)
        // Note: Decimation can be changed on-the-fly to adapt during a match.
        aprilTag.setDecimation(3);

        // Create the vision portal by using a builder.
        VisionPortal.Builder builder = new VisionPortal.Builder();

        // Set the camera (webcam vs. built-in RC phone camera).
        if (USE_WEBCAM) {
            builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
            builder.setCameraResolution(new Size(1280, 720));
        } else {
            builder.setCamera(BuiltinCameraDirection.BACK);
        }
        // Choose a camera resolution. Not all cameras support all resolutions.
        //builder.setCameraResolution(new Size(640, 480));

        // Enable the RC preview (LiveView).  Set "false" to omit camera monitoring.
        //builder.enableLiveView(true);

        // Set the stream format; MJPEG uses less bandwidth than default YUY2.
        builder.setStreamFormat(VisionPortal.StreamFormat.MJPEG);

        // Choose whether or not LiveView stops if no processors are enabled.
        // If set "true", monitor shows solid orange screen if no processors enabled.
        // If set "false", monitor shows camera view without annotations.
        //builder.setAutoStopLiveView(false);

        // Set and enable the processor.
        builder.addProcessor(aprilTag);

        // Build the Vision Portal, using the above settings.
        visionPortal = builder.build();

        // Disable or re-enable the aprilTag processor at any time.
        //visionPortal.setProcessorEnabled(aprilTag, true);

    }   // end method initAprilTag()
    /**
     * Drive with Road Runner. When sticks idle (below deadband): zero power so BRAKE mode resists.
     * Standard approach per Road Runner teleop—odometry feedback causes noise-driven movement.
     */
    private void driveWithPositionHold(Pose2d pose) {
        double x = gamepad1.left_stick_x;
        double y = -gamepad1.left_stick_y;
        double turn = gamepad1.right_stick_x;
        // Apply DriveTrain toggles (XYtoggle, Rtoggle) if we ever add a way to trigger them
        if (drivetrain.getXToggle()) { x *= 0.25; y *= 0.25; }
        if (drivetrain.getRToggle()) turn *= 0.25;
        double db = positionHoldDeadband;
        boolean idle = positionHoldEnabled
                && Math.abs(x) < db && Math.abs(y) < db && Math.abs(turn) < db;
        if (idle) {
            mecanumDrive.setDrivePowers(new PoseVelocity2d(new Vector2d(0, 0), 0));
        } else {
            PoseVelocity2d vel = new PoseVelocity2d(
                    new Vector2d(y * driveScale, -x * driveScale),
                    -turn * driveScale);
            mecanumDrive.setDrivePowers(vel);
        }
    }

    /** Sets start pose and goal from current alliance mode (red vs blue). Called in init and when dpad L/R pressed. */
    private void applyAllianceMode() {
        if (mode) {
            startX = blueStartX; startY = blueStartY; startHeadingDeg = blueStartHeadingDeg;
            goalX = blueGoalX; goalY = blueGoalY;
        } else {
            startX = redStartX; startY = redStartY; startHeadingDeg = redStartHeadingDeg;
            goalX = redGoalX; goalY = redGoalY;
        }
    }

    /** Telemetry grouped by subsystem so you can find what you need to debug. */
    private void initTelemetry() {
        // --- 1. Alliance & localization (odometry, pose, goal) ---
        telemetry.addLine("--- Alliance & localization ---");
        telemetry.addData("Alliance", mode ? "BLUE" : "RED");
        telemetry.addData("Pose (x, y, heading)", mecanumDrive.localizer.getPose());
        telemetry.addData("Heading deg", Math.toDegrees(mecanumDrive.localizer.getPose().heading.toDouble()));
        telemetry.addData("Dist to goal in", distance);
        telemetry.addData("Goal (x, y)", "%.0f, %.0f", goalX, goalY);

        // --- 2. Drive ---
        telemetry.addLine("");
        telemetry.addLine("--- Drive ---");
        telemetry.addData("Position hold", positionHoldEnabled ? "ON (idle=brake)" : "OFF");
        telemetry.addData("X toggle (strafe)", drivetrain.getXToggle());
        telemetry.addData("R toggle (rotate)", drivetrain.getRToggle());

        // --- 3. Turret (field-relative hold) ---
        telemetry.addLine("");
        telemetry.addLine("--- Turret ---");
        telemetry.addData("Field deg", turret.getTurretAngleField());
        telemetry.addData("Robot deg", turret.getTurretAngleRobot());
        telemetry.addData("Target (hold) field deg", turret.targetAngle);
            telemetry.addData("Aim offset (nudge) deg", turret.aimOffsetDeg);
        telemetry.addData("Lock-on", lockedOn);
        telemetry.addData("Turret re-sync", "X or Start (center first)");

        // --- 4. Intake / launcher ---
        telemetry.addLine("");
        telemetry.addLine("--- Airsort (Operator Back = toggle) ---");
        telemetry.addData("Airsort", airSortActive ? "ON" : "OFF");
        if (airSortActive) {
            telemetry.addData("Shot", "%d/3", airSort.getShotIndex() + 1);
            telemetry.addData("Mode", airSort.getShotMode().name());
            telemetry.addData("Ball at launcher", airSort.getBallAtLauncher());
            telemetry.addData("Want", airSort.getDesiredColorForCurrentShot());
            telemetry.addData("Motif stored", airSort.isMotifStored() ? "yes" : "no (need tag 21/22/23)");
            List<AprilTagDetection> det = aprilTag.getDetections();
            StringBuilder ids = new StringBuilder();
            boolean seenMotif = false;
            for (AprilTagDetection d : det) {
                if (ids.length() > 0) ids.append(", ");
                ids.append(d.id);
                if (d.id == 21 || d.id == 22 || d.id == 23) seenMotif = true;
            }
            telemetry.addData("Detected tag IDs", ids.length() > 0 ? ids.toString() : "none");
            if (seenMotif) telemetry.addLine("(Motif tag 21/22/23 visible → will store)");
        }
        telemetry.addLine("");
        telemetry.addLine("--- Intake / launcher ---");
        telemetry.addData("Launcher vel", intake.getLauncherSpeed());
        telemetry.addData("Target vel", intake.getTargetVel());
        telemetry.addData("Hood", intake.getHood());
        telemetry.addData("Comp (calcShot)", intake.calcShot(IntakeV2.getHeading()));
        telemetry.addData("Color R/G/B", "%d %d %d", intake.c3.red(), intake.c3.green(), intake.c3.blue());
        telemetry.addData("c3 RGB sum", intake.getC3RgbSum());
        telemetry.addData("Ball at launcher (blocks transfer)", intake.getBallAtLauncher());
        telemetry.addData("G1 right trigger", "%.2f (intake+transfer when >0.25)", gamepad1.right_trigger);

        telemetry.update();
    }
    /**
     * Manual turret mode: Y = enable, B = disable.
     * When on, gamepad2 left stick X controls turret (robot-relative). Limit ±170°.
     */
    private void lockOn() {
        if (gamepad2.yWasPressed() && !lockedOn) {
            lockedOn = true;
            k = turret.botHeading;
            manualTurretDeg = turret.getTurretAngleRobot();  // start from current position
        } else if (gamepad2.bWasPressed() && lockedOn) {
            lockedOn = false;
        }
        if (lockedOn) {
            if (gamepad2.aWasPressed()) manualTurretDeg = 0;  // A = center turret
            // X / Start re-sync handled above (resyncEncoder + manualTurretDeg = 0)
            final double manualDegPerUnit = 4.0;
            manualTurretDeg += gamepad2.left_stick_x * manualDegPerUnit;  // stick right = turret right (Turret.manualModeInvert flips power if needed)
            manualTurretDeg = Math.max(-Turret.manualModeLimitDeg, Math.min(Turret.manualModeLimitDeg, manualTurretDeg));
            turret.setAngle(k + manualTurretDeg);
        }
    }


    private void telemetryAprilTag() {
        telemetry.addLine("");
        telemetry.addLine("--- AprilTag / vision ---");
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        telemetry.addData("# AprilTags Detected", currentDetections.size());

        // Step through the list of detections and display info for each one.
        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null) {
                telemetry.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
                telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)", detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));
                telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)", detection.ftcPose.pitch, detection.ftcPose.roll, detection.ftcPose.yaw));
                telemetry.addLine(String.format("RBE %6.1f %6.1f %6.1f  (M, deg, deg)", (detection.ftcPose.range* 0.0254), detection.ftcPose.bearing, detection.ftcPose.elevation));
            } else {
                telemetry.addLine(String.format("\n==== (ID %d) Unknown", detection.id));
                telemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)", detection.center.x, detection.center.y));
            }
        }   // end for() loop

        // Add "key" information to telemetry
        telemetry.addLine("\nkey:\nXYZ = X (Right), Y (Forward), Z (Up) dist.");
        telemetry.addLine("PRY = Pitch, Roll & Yaw (XYZ Rotation)");
        telemetry.addLine("RBE = Range, Bearing & Elevation");

    }

    /** Updates motif from AprilTag 21/22/23 if seen (backup to AirSort; AirSort is source of truth when airsort on). */
    public String[] getMotif() {
        for (AprilTagDetection detection : aprilTag.getDetections()) {
            switch (detection.id) {
                case 21: motif[0] = "green"; motif[1] = "purple"; motif[2] = "purple"; return motif;
                case 22: motif[0] = "purple"; motif[1] = "green"; motif[2] = "purple"; return motif;
                case 23: motif[0] = "purple"; motif[1] = "purple"; motif[2] = "green"; return motif;
                default: break;
            }
        }
        return motif;
    }

    /** Count of color sensors (c1,c2,c3) with R+G+B >= artifactThreshold. Color-agnostic. */
    private int countArtifacts() {
        int t = artifactThreshold;
        int n = 0;
        if (intake.c1.red() + intake.c1.green() + intake.c1.blue() >= t) n++;
        if (intake.c2.red() + intake.c2.green() + intake.c2.blue() >= t) n++;
        if (intake.c3.red() + intake.c3.green() + intake.c3.blue() >= t) n++;
        return n;
    }

    /** Distance to goal in inches. Prefers camera (tags 20/24) when visible and in 12–200"; else odometry. */
    public static double getDistance() {
        distance = Math.hypot(distanceX, distanceY);
        if (Double.isFinite(cameraDistanceInches) && cameraDistanceInches >= 12 && cameraDistanceInches <= 200) {
            return cameraDistanceInches;
        }
        return distance;
    }

    /** Distance to goal in inches from odometry only. Used by launcher regression and AirSort (not camera). */
    public static double getDistanceFromOdometry() {
        return Math.hypot(distanceX, distanceY);
    }



}
