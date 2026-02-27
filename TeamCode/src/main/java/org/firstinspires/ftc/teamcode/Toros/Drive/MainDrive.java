package org.firstinspires.ftc.teamcode.Toros.Drive;

import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.LED;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.RR.Drawing;
import org.firstinspires.ftc.teamcode.RR.MecanumDrive;
import org.firstinspires.ftc.teamcode.Toros.Drive.CameraRelocalization;
import org.firstinspires.ftc.teamcode.Toros.Drive.ShotPhysics;
import org.firstinspires.ftc.teamcode.Toros.Drive.Subsystems.DriveTrain;
import org.firstinspires.ftc.teamcode.Toros.Drive.Subsystems.IntakeV2;
import org.firstinspires.ftc.teamcode.Toros.Drive.Subsystems.Turret;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
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
 * Start position and goal are in FTC official field frame: origin at center of mat, +X to the right from red wall,
 * +Y away from red wall (toward blue). If the robot appears in the wrong place on Dashboard, tune
 * PinpointLocalizer.PARAMS (parYTicks, perpXTicks) and encoder directions in PinpointLocalizer.
 *
 * Turret: When !lockedOn we aim at goal: angleToGoal uses pose + goal. Angles in [0, 360) (RR convention).
 * Dpad Up = 0°, Dpad Down = 180°; else angleToGoal. Lock-on (Y) = vision; B = exit. Left stick X = nudge.
 *
 * Alliance: Dpad Left = Blue, Dpad Right = Red; start and goal set from red* / blue*.
 */
@TeleOp(name = "MainDrive")
@Config
public class MainDrive extends LinearOpMode {
    private static final boolean USE_WEBCAM = true;

    /** Start pose and goal (inches, degrees). Set from alliance mode (blue/red). Frame: from audience +Y=forward, -X=left, +X=right, -Y=back. */
    public static double startX = -48.0;
    public static double startY = -50.0;
    public static double startHeadingDeg = -90.0;
    public static double goalX = -64.0;
    public static double goalY = -60.0;

    /** Red alliance: start -X left, +Y; goal -X left, +Y; heading 90°. */
    public static double redStartX = -48.0, redStartY = 50.0, redStartHeadingDeg = 90.0;
    public static double redGoalX = -64.0, redGoalY = 60.0;
    /** Blue alliance: start -X -Y (left, back); goal -X -Y; heading -90°. Angle-to-goal uses atan2(goal−pose) so any (goalX,goalY) works. */
    public static double blueStartX = -48.0, blueStartY = -50.0, blueStartHeadingDeg = -90.0;
    public static double blueGoalX = -70.0, blueGoalY = -64.0;

    /** If turret mechanical 0° is not aligned with robot +X, add offset here (e.g. 90 or -90). Tune on Dashboard. */
    public static double turretAngleOffsetDeg = 0.0;

    public AprilTagProcessor aprilTag;
    public String[] motif = new String[3];
    public VisionPortal visionPortal;
    DriveTrain drivetrain;
    IntakeV2 intake;
    Turret turret;
    public ColorSensor c3;
    List<LynxModule> allHubs;
    Servo led;
    private boolean lockedOn = false;
    /** true = blue alliance (start/goal from blue*), false = red (red*). Dpad Left = Blue, Dpad Right = Red. */
    private boolean mode = true;
    /** Frozen heading (deg) when lock-on is on; used by turret instead of live gyro. */
    double k = 0;
    /** Field angle (deg) for turret when !lockedOn: angleToGoal (aim) or 0 / 180 from dpad. [0, 360). */
    private double fieldHoldAngle = 0;

    /** Distance to goal (inches); set each loop from odometry pose. */
    public static double distance = 0;
    public static double distanceX = 0;
    public static double distanceY = 0;
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


        //Constructs the systems and makes them objects allowing to use a method to run the system and allows for other methods to be used
        allHubs = hardwareMap.getAll(LynxModule.class);
        for(LynxModule hub: allHubs){
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }
        initAprilTag();
        drivetrain = new DriveTrain(hardwareMap, gamepad1);
        led = hardwareMap.get(Servo.class, "LED");
        intake = new IntakeV2(hardwareMap, gamepad1, gamepad2, aprilTag);
        turret = new Turret(hardwareMap, gamepad2);
        applyAllianceMode();
        Pose2d initialPose = new Pose2d(startX, startY, Math.toRadians(startHeadingDeg));
        mecanumDrive = new MecanumDrive(hardwareMap, initialPose);
        mecanumDrive.localizer.setPose(initialPose);

        // During init: Dpad Left = Blue, Dpad Right = Red (start/goal and pose update)
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
            telemetry.addData("Alliance", mode ? "BLUE" : "RED");
            telemetry.addData("Start", "%.0f, %.0f @ %.0f deg", startX, startY, startHeadingDeg);
            telemetry.addData("Goal", "%.0f, %.0f", goalX, goalY);
            telemetry.addData(">", "Dpad L=Blue R=Red, Play=start");
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
            mecanumDrive.updatePoseEstimate();
            Pose2d pose = mecanumDrive.localizer.getPose();
            distanceX = pose.position.x - goalX;
            distanceY = pose.position.y - goalY;
            distance = Math.sqrt(distanceX * distanceX + distanceY * distanceY);

            // Pose history for Dashboard path (localization viz)
            poseHistory.add(pose);
            while (poseHistory.size() > 200) poseHistory.removeFirst();

            // --- 2. Turret: odometry aiming at goal. Field angles [0, 360); 0° = toward +Y (goals), 180° = audience. ---
            turret.botHeading = Turret.wrapDeg360(Math.toDegrees(pose.heading.toDouble()));
            // atan2(dy,dx) in same frame as RR: 0° = +X, 90° = +Y, CCW positive. No -90 shift (was double-rotating vs heading).
            double angleToGoalRad = Math.atan2(goalY - pose.position.y, goalX - pose.position.x);
            double angleToGoalDeg = Turret.wrapDeg360(Math.toDegrees(angleToGoalRad) + turretAngleOffsetDeg);
            if (!lockedOn) {
                if (gamepad2.dpad_up) fieldHoldAngle = 0;
                else if (gamepad2.dpad_down) fieldHoldAngle = 180;
                else fieldHoldAngle = angleToGoalDeg;  // aim at goal by default
                turret.targetAngle = fieldHoldAngle;
            }
            // Alliance can be changed during run: goal/start update (pose not reset)
            if (gamepad1.dpad_left) { mode = true; applyAllianceMode(); }
            else if (gamepad1.dpad_right) { mode = false; applyAllianceMode(); }
            // --- 3. Telemetry, drive, turret run, lock-on, intake ---
            led.setPosition(1);
            initTelemetry();
            telemetryAprilTag();
            getMotif();
            drivetrain.driveRobotCentric();
            if (lockedOn) turret.runTurretNoGyro(k);
            else {
                turret.runTurretGyro();
                fieldHoldAngle = turret.targetAngle;
            }
            lockOn();
            intake.runLauncher();
            intake.runIntake();
            intake.transfer();

            // --- 4. Camera relocalization: pose from AprilTag (15° tilt, odometry heading); display only ---
            cameraPose = null;
            double headingRad = pose.heading.toDouble();
            for (AprilTagDetection d : aprilTag.getDetections()) {
                if (d.metadata == null) continue;
                Vector2d tagPos = CameraRelocalization.getTagFieldPosition(d.id);
                if (tagPos != null && d.id == CameraRelocalization.BLUE_GOAL_TAG_ID) {
                    cameraPose = CameraRelocalization.robotPoseFromTag(d, tagPos.x, tagPos.y, headingRad);
                    break;
                }
            }

            // --- 5. FTC Dashboard: field drawing + numbers (all localization viz here) ---
            TelemetryPacket packet = new TelemetryPacket();
            Drawing.drawPoseHistory(packet.fieldOverlay(), poseHistory, "#3F51B5"); // path (blue)
            Drawing.drawRobot(packet.fieldOverlay(), pose, 1, "#3F51B5");             // odometry robot (blue)
            Drawing.drawGoal(packet.fieldOverlay(), goalX, goalY, "#4CAF50");         // goal (green)
            Drawing.drawRobotToGoalLine(packet.fieldOverlay(), pose, goalX, goalY, "#FFC107"); // aim line (yellow)
            if (cameraPose != null) Drawing.drawCameraPose(packet.fieldOverlay(), cameraPose, "#FF5722"); // camera reloc (orange, inches)
            double distToGoalIn = Math.hypot(pose.position.x - goalX, pose.position.y - goalY);
            double hoodDeg = Math.max(40, Math.min(60, 60 - intake.getHood() * 20));
            double[] shot = ShotPhysics.speedAndTimeInAir(distToGoalIn, hoodDeg);
            packet.put("dist_to_goal_in", distToGoalIn);
            packet.put("speed_needed_mps", shot[0]);
            packet.put("time_in_air_s", shot[1]);
            packet.put("odom_x", pose.position.x);
            packet.put("odom_y", pose.position.y);
            packet.put("odom_heading_deg", Math.toDegrees(pose.heading.toDouble()));
            packet.put("pose_src", "Pinpoint (pods+IMU)");
            packet.put("angle_to_goal_deg", angleToGoalDeg);
            packet.put("turret_field_deg", turret.getTurretAngleField());
            packet.put("turret_robot_deg", turret.getTurretAngleRobot());
            packet.put("field_hold_deg", fieldHoldAngle);
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
                //.setTagLibrary(AprilTagGameDatabase.getCenterStageTagLibrary())
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
    //Telemetry which is good for debugging and seeing how we preform
    /** Sets start pose and goal from current alliance mode (red vs blue). */
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
        telemetry.addData("X toggle (strafe)", drivetrain.getXToggle());
        telemetry.addData("R toggle (rotate)", drivetrain.getRToggle());

        // --- 3. Turret (field-relative hold) ---
        telemetry.addLine("");
        telemetry.addLine("--- Turret ---");
        telemetry.addData("Field deg", turret.getTurretAngleField());
        telemetry.addData("Robot deg", turret.getTurretAngleRobot());
        telemetry.addData("Target (hold) field deg", turret.targetAngle);
        telemetry.addData("Lock-on", lockedOn);

        // --- 4. Intake / launcher ---
        telemetry.addLine("");
        telemetry.addLine("--- Intake / launcher ---");
        telemetry.addData("Launcher vel", intake.getLauncherSpeed());
        telemetry.addData("Target vel", intake.getTargetVel());
        telemetry.addData("Hood", intake.getHood());
        telemetry.addData("Comp (calcShot)", intake.calcShot(IntakeV2.getHeading()));
        telemetry.addData("Color R/G/B", "%d %d %d", intake.c3.red(), intake.c3.green(), intake.c3.blue());

        telemetry.update();
    }
    /**
     * Lock-on: Y = enable (freeze current heading as k for turret). B = disable.
     * When locked on, vision updates targetAngle so turret tracks AprilTag bearing (IDs 20 or 24).
     */
    private void lockOn() {
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        if (gamepad2.yWasPressed() && !lockedOn) {
            lockedOn = true;
            // Use turret's Pinpoint heading (degrees); turret.runTurretGyro() already ran this loop so botHeading is current
            k = turret.botHeading;
        } else if (gamepad2.bWasPressed() && lockedOn) {
            lockedOn = false;
        }
        for (AprilTagDetection detection : currentDetections) {
            // Rumble when aimed at target tag (bearing within 2°)
            if (detection.metadata != null && Math.abs(detection.ftcPose.bearing) < 2 && (detection.id == 20 || detection.id == 24)) {
                gamepad2.rumble(500);
            }

            // Lock-on: adjust turret target so it tracks the AprilTag. Bearing = angle from camera to tag.
            if (detection.metadata != null && lockedOn && (detection.id == 20 || detection.id == 24)) {
                double error = detection.ftcPose.bearing; // degrees: positive = tag left of center

                if (error > 5) {
                    turret.setAngle(turret.getTurretAngle() - error * 0.5);
                }
                if (error < -5) {
                    turret.setAngle(turret.getTurretAngle() + error * -0.5);
                }
                if (Math.abs(error) <= 2) {
                    turret.setAngle(turret.getTurretAngle() * 1.01);
                }
            }
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

    public String[] getMotif () {
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();

        for (AprilTagDetection detection : currentDetections)
            switch (detection.id) {

                case 21 :  motif[0] = "green"; motif[1] = "purple"; motif[2] = "purple"; break;
                case 22 :  motif[0] = "purple"; motif[1] = "green"; motif[2] = "purple"; break;
                case 23 :  motif[0] = "purple"; motif[1] = "purple"; motif[2] = "green"; break;

            }
    return motif;
    }

    public static double getDistance(){
        return distance = Math.sqrt(Math.pow(distanceX,2)+Math.pow(distanceY,2));
    }


    //notes
    // make the code set allaince upon looking at the corresponding apriltag
    //




}
