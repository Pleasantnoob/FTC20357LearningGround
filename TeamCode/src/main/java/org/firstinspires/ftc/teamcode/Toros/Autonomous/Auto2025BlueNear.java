package org.firstinspires.ftc.teamcode.Toros.Autonomous;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.controller.wpilibcontroller.SimpleMotorFeedforward;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.RR.PoseBridge;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.RR.MecanumDrive;
import org.firstinspires.ftc.teamcode.Toros.Drive.MainDrive;
import org.firstinspires.ftc.teamcode.Toros.Drive.Subsystems.IntakeV2;
import org.firstinspires.ftc.teamcode.Toros.Drive.Subsystems.Turret;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.ArrayList;
import java.util.List;

/**
 * Blue near autonomous. Uses same start pose as teleop (MainDrive blue) and same shot physics
 * and goal aiming: IntakeV2 (distance → hood/flywheel) and Turret (aim at goal).
 * Path waypoints from MeepMeepTesting.java blue bot.
 */
@Autonomous(name = "Auto2025BlueNear")
public class Auto2025BlueNear extends LinearOpMode {

    /** Goal for aiming; match MainDrive.blueGoalX/Y. */
    private static final double GOAL_X = MainDrive.blueGoalX;
    private static final double GOAL_Y = MainDrive.blueGoalY;
    public DcMotorEx launch, turretMotor, trans;
    public Servo hood;
    public ColorSensor c1,c2,c3;
    private DcMotor intake;
    private PIDController controller;

    public static double p1 = 0.0045, i1 = 0, d1 = 0;
    public static double kS1 = 0.001, kV1 = 0.00055, kA1 = -0;
    public static double accel = 20;

    public static double p2 = 0.00725 , i2 = 0.0, d2 = 0.00055;
    public static double kS2 = 0, kV2 = 0.000125, kA2 = 0;

    double gearRatio = 2.0 / 5.0;
    public static int defaultTargetVel = -1275; ////default
    public static int targetVel = defaultTargetVel;
    public static int targetAngle = 0;


    public class colorSensors {
        public colorSensors(HardwareMap hardwareMap){
            c1 = hardwareMap.get(ColorSensor.class,"c1");
            c2 = hardwareMap.get(ColorSensor.class,"c2");
            c3 = hardwareMap.get(ColorSensor.class,"c3");
        }
        public class flash implements Action {
            private boolean init = false;


            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                //logic here for color sensors
                return stored.size() == 3;

            }
        }
    }

    public class scan implements Action {
        private boolean init = false;

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if (!init) {
                List<AprilTagDetection> currentDetections = aprilTag.getDetections();
                initAprilTag();
                for(AprilTagDetection detection: currentDetections){
                    if(detection != null){
                        if (detection.id == 21){
                            motif.add("g");
                            motif.add("p");
                            motif.add("p");
                            //turn table swap to position of green
                            //swap to purple
                            //swap to purple
                        } else if (detection.id == 22) {
                            motif.add("p");
                            motif.add("g");
                            motif.add("p");
                            //swap to purple
                            //swap to green
                            //swap to purple
                        }
                        else if (detection.id == 23){
                            motif.add("p");
                            motif.add("p");
                            motif.add("g");
                            //swap to purple
                            //swap to purple
                            //swap to green
                        }
                    }
                }

            }
            return motif.size() != 3;
        }
    }

    public Action scanMotif(){return new scan();}
    private static final boolean USE_WEBCAM = true;  // true for webcam, false for phone camera

    /**
     * The variable to store our instance of the AprilTag processor.
     */
    private AprilTagProcessor aprilTag;

    /**
     * The variable to store our instance of the vision portal.
     */

    private VisionPortal visionPortal;

    ArrayList<String> stored = new ArrayList<>();
    ArrayList<String> motif = new ArrayList<>();

    /** Set false when main sequence ends so background TurretAimAction stops. */
    private volatile boolean autoRunning = true;

    @Override
    public void runOpMode() {


        // Start pose = teleop blue (MainDrive). Path waypoints from MeepMeepTesting.java blue bot.
        Pose2d initialPose = new Pose2d(MainDrive.blueStartX, MainDrive.blueStartY, Math.toRadians(MainDrive.blueStartHeadingDeg));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
        drive.localizer.setPose(initialPose);
        AprilTagProcessor aprilTag = new AprilTagProcessor.Builder().build();
        IntakeV2 intake = new IntakeV2(hardwareMap, gamepad1, gamepad2, aprilTag);
        Turret turret = new Turret(hardwareMap, gamepad2);

        telemetry.addData(">", "Blue Near. Start = teleop blue. Shot physics + goal aim from teleop.");
        telemetry.update();
        waitForStart();

        // Trajectories from MeepMeepTesting.java blue bot — single source of truth
        Action tab1 = drive.actionBuilder(initialPose)
                .strafeToLinearHeading(new Vector2d(-12, -12), Math.toRadians(270)) // Preload
                .build();
        Action tab2 = drive.actionBuilder(new Pose2d(-12, -12, Math.toRadians(270)))
                .strafeToLinearHeading(new Vector2d(-12, -28), Math.toRadians(270)) // first spike
                .strafeTo(new Vector2d(-12, -53)) // first spike intake
                .build();
        Action tab3 = drive.actionBuilder(new Pose2d(-12, -53, Math.toRadians(270)))
                .strafeToLinearHeading(new Vector2d(-12, -13), Math.toRadians(270)) // launch first spike
                .build();
        Action tab4 = drive.actionBuilder(new Pose2d(-12, -13, Math.toRadians(270)))
                .setTangent(Math.toRadians(315))
                .splineToLinearHeading(new Pose2d(12, -28, Math.toRadians(270)), Math.toRadians(270)) // second spike
                .strafeTo(new Vector2d(12, -53)) // second spike intake
                .build();
        Action tab5 = drive.actionBuilder(new Pose2d(12, -53, Math.toRadians(270)))
                .setTangent(Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(-13, -12, Math.toRadians(270)), Math.toRadians(90))
                .build();
        Action tab6_park = drive.actionBuilder(new Pose2d(-13, -12, Math.toRadians(270)))
                .strafeToLinearHeading(new Vector2d(0, -30), Math.toRadians(180))
                .build();


        if (opModeIsActive()) {
            autoRunning = true;
            // Passive odometry aim: turret aims at blue goal every tick in parallel with main sequence (same as Red Near Odometry Aim).
            Action mainSequence = new SequentialAction(
                    tab1,
                    new RevAndAimAction(drive, intake, turret, 2.0),
                    new ShootAction(drive, intake, turret, 4.0),
                    new ParallelAction(tab2, new IntakeRunAction(intake, 3.0)),
                    tab3,
                    new RevAndAimAction(drive, intake, turret, 2.0),
                    new ShootAction(drive, intake, turret, 4.0),
                    new ParallelAction(tab4, new SequentialAction(new SleepAction(1.1), new IntakeRunAction(intake, 3.0))),
                    tab5,
                    new RevAndAimAction(drive, intake, turret, 2.0),
                    new ShootAction(drive, intake, turret, 4.0),
                    tab6_park,
                    new StopLauncherAction(intake),
                    new SetFlagAndEndAction()
            );
            Actions.runBlocking(new ParallelAction(
                    new TurretAimAction(drive, turret, GOAL_X, GOAL_Y),
                    new HoodAndFlywheelUpdateAction(drive, intake, GOAL_X, GOAL_Y),
                    mainSequence
            ));

            PoseBridge.save(drive.localizer.getPose());
            intake.stopLauncherAuto();
            intake.runIntakeAuto(false);

            while (opModeIsActive()) {
                telemetry.addData("Pose", drive.localizer.getPose());
                telemetry.update();
                sleep(20);
            }
        }

    }   // end method runOpMode()

    /** Rev + turret aim at goal for duration; uses pose distance for hood/flywheel (same as teleop). */
    private static class RevAndAimAction implements Action {
        private final MecanumDrive drive;
        private final IntakeV2 intake;
        private final Turret turret;
        private final double duration;
        private final ElapsedTime timer = new ElapsedTime();
        private boolean init;

        RevAndAimAction(MecanumDrive drive, IntakeV2 intake, Turret turret, double duration) {
            this.drive = drive;
            this.intake = intake;
            this.turret = turret;
            this.duration = duration;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket p) {
            if (!init) {
                timer.reset();
                init = true;
            }
            drive.updatePoseEstimate();
            Pose2d pose = drive.localizer.getPose();
            double dx = GOAL_X - pose.position.x;
            double dy = GOAL_Y - pose.position.y;
            double dist = Math.hypot(dx, dy);
            double angleToGoalDeg = Turret.wrapDeg360(Math.toDegrees(Math.atan2(dy, dx)));
            intake.setHoodAndFlywheelFromDistance(dist);
            intake.runLauncherAuto(false);
            turret.botHeading = Turret.wrapDeg360(Math.toDegrees(pose.heading.toDouble()));
            turret.targetAngle = angleToGoalDeg;
            turret.runTurretGyro();
            return timer.seconds() < duration;
        }
    }

    /** Shoot: rev + aim + feed for duration (same shot physics as teleop). */
    private static class ShootAction implements Action {
        private final MecanumDrive drive;
        private final IntakeV2 intake;
        private final Turret turret;
        private final double duration;
        private final ElapsedTime timer = new ElapsedTime();
        private boolean init;

        ShootAction(MecanumDrive drive, IntakeV2 intake, Turret turret, double duration) {
            this.drive = drive;
            this.intake = intake;
            this.turret = turret;
            this.duration = duration;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket p) {
            if (!init) {
                timer.reset();
                init = true;
            }
            drive.updatePoseEstimate();
            Pose2d pose = drive.localizer.getPose();
            double dx = GOAL_X - pose.position.x;
            double dy = GOAL_Y - pose.position.y;
            double dist = Math.hypot(dx, dy);
            double angleToGoalDeg = Turret.wrapDeg360(Math.toDegrees(Math.atan2(dy, dx)));
            intake.setHoodAndFlywheelFromDistance(dist);
            intake.runLauncherAuto(true);
            turret.botHeading = Turret.wrapDeg360(Math.toDegrees(pose.heading.toDouble()));
            turret.targetAngle = angleToGoalDeg;
            turret.runTurretGyro();
            if (timer.seconds() >= duration) {
                intake.stopLauncherAuto();
                return false;
            }
            return true;
        }
    }

    private static class IntakeRunAction implements Action {
        private final IntakeV2 intake;
        private final double duration;
        private final ElapsedTime timer = new ElapsedTime();
        private boolean init;

        IntakeRunAction(IntakeV2 intake, double duration) {
            this.intake = intake;
            this.duration = duration;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket p) {
            if (!init) {
                timer.reset();
                init = true;
                intake.runIntakeAuto(true);
            }
            if (timer.seconds() >= duration) {
                intake.runIntakeAuto(false);
                return false;
            }
            return true;
        }
    }

    private static class StopLauncherAction implements Action {
        private final IntakeV2 intake;

        StopLauncherAction(IntakeV2 intake) {
            this.intake = intake;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket p) {
            intake.stopLauncherAuto();
            return false;
        }
    }

    /** Sets autoRunning = false so background TurretAimAction stops. */
    private class SetFlagAndEndAction implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket p) {
            autoRunning = false;
            return false;
        }
    }

    /** Continuously aims turret at blue goal using odometry pose. Runs in parallel with main sequence until it ends. */
    private class TurretAimAction implements Action {
        private final MecanumDrive drive;
        private final Turret turret;
        private final double goalX;
        private final double goalY;

        TurretAimAction(MecanumDrive drive, Turret turret, double goalX, double goalY) {
            this.drive = drive;
            this.turret = turret;
            this.goalX = goalX;
            this.goalY = goalY;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket p) {
            drive.updatePoseEstimate();
            Pose2d pose = drive.localizer.getPose();
            double dx = goalX - pose.position.x;
            double dy = goalY - pose.position.y;
            double angleToGoalDeg = Turret.wrapDeg360(Math.toDegrees(Math.atan2(dy, dx)));
            turret.botHeading = Turret.wrapDeg360(Math.toDegrees(pose.heading.toDouble()));
            turret.targetAngle = angleToGoalDeg;
            turret.runTurretGyro();
            return autoRunning;
        }
    }

    /** Continuously updates hood angle and flywheel target velocity from current odometry distance to blue goal. Runs in parallel with main sequence. */
    private class HoodAndFlywheelUpdateAction implements Action {
        private final MecanumDrive drive;
        private final IntakeV2 intake;
        private final double goalX;
        private final double goalY;

        HoodAndFlywheelUpdateAction(MecanumDrive drive, IntakeV2 intake, double goalX, double goalY) {
            this.drive = drive;
            this.intake = intake;
            this.goalX = goalX;
            this.goalY = goalY;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket p) {
            drive.updatePoseEstimate();
            Pose2d pose = drive.localizer.getPose();
            double dist = Math.hypot(goalX - pose.position.x, goalY - pose.position.y);
            intake.setHoodAndFlywheelFromDistance(dist);
            return autoRunning;
        }
    }

    /**
     * Initialize the AprilTag processor.
     */
    private void initAprilTag() {

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

                .build();

        // Adjust Image Decimation to trade-off detection-range for detection-rate.
        // eg: Some typical detection data using a Logitech C920 WebCam
        // Decimation = 1 ..  Detect 2" Tag from 10 feet away at 10 Frames per second
        // Decimation = 2 ..  Detect 2" Tag from 6  feet away at 22 Frames per second
        // Decimation = 3 ..  Detect 2" Tag from 4  feet away at 30 Frames Per Second (default)
        // Decimation = 3 ..  Detect 5" Tag from 10 feet away at 30 Frames Per Second (default)
        // Note: Decimation can be changed on-the-fly to adapt during a match.
        //aprilTag.setDecimation(3);

        // Create the vision portal by using a builder.
        VisionPortal.Builder builder = new VisionPortal.Builder();

        // Set the camera (webcam vs. built-in RC phone camera).
        if (USE_WEBCAM) {
            builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam1"));
        } else {
            builder.setCamera(BuiltinCameraDirection.BACK);
        }

        // Choose a camera resolution. Not all cameras support all resolutions.
        //builder.setCameraResolution(new Size(640, 480));

        // Enable the RC preview (LiveView).  Set "false" to omit camera monitoring.
        //builder.enableLiveView(true);

        // Set the stream format; MJPEG uses less bandwidth than default YUY2.
        //builder.setStreamFormat(VisionPortal.StreamFormat.YUY2);

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
     * Add telemetry about AprilTag detections.
     */
    private void telemetryAprilTag() {

        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        telemetry.addData("# AprilTags Detected", currentDetections.size());

        // Step through the list of detections and display info for each one.
        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null) {
                telemetry.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
                telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)", detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));
                telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)", detection.ftcPose.pitch, detection.ftcPose.roll, detection.ftcPose.yaw));
                telemetry.addLine(String.format("RBE %6.1f %6.1f %6.1f  (inch, deg, deg)", detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.elevation));

            } else {
                telemetry.addLine(String.format("\n==== (ID %d) Unknown", detection.id));
                telemetry.addLine(String.format("Center %6.0f %6.0f (pixels)", detection.center.x, detection.center.y));

            }
            if (detection.id == 21){
                motif.add("g");
                motif.add("p");
                motif.add("p");
                //turn table swap to position of green
                //swap to purple
                //swap to purple
            } else if (detection.id == 22) {
                motif.add("p");
                motif.add("g");
                motif.add("p");
                //swap to purple
                //swap to green
                //swap to purple
            }
            else if (detection.id == 23){
                motif.add("p");
                motif.add("p");
                motif.add("g");
                //swap to purple
                //swap to purple
                //swap to green
            }
        }   // end for() loop

        // Add "key" information to telemetry
        telemetry.addLine("\nkey:\nXYZ = X (Right), Y (Forward), Z (Up) dist.");
        telemetry.addLine("PRY = Pitch, Roll & Yaw (XYZ Rotation)");
        telemetry.addLine("RBE = Range, Bearing & Elevation");


    }   // end method telemetryAprilTag()


}




