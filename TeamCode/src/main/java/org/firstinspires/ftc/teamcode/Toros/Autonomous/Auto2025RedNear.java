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
 * Red near autonomous. Mirror of Blue Near: Y and heading flipped for red alliance.
 * Uses same start pose as teleop (MainDrive red) and same shot physics and goal aiming.
 */
@Autonomous(name = "Auto2025RedNear")
public class Auto2025RedNear extends LinearOpMode {

    /** Goal for aiming; match MainDrive.redGoalX/Y. */
    private static final double GOAL_X = MainDrive.redGoalX;
    private static final double GOAL_Y = MainDrive.redGoalY;
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


    /** Wraps IntakeV2 to provide template-style intakeRun() and transRun() actions for parallel use. */
    public static class IntakeSub {
        private final IntakeV2 intakeV2;

        public IntakeSub(IntakeV2 intakeV2) {
            this.intakeV2 = intakeV2;
        }

        /** Runs intake motor for duration. Use with transRun() in ParallelAction. */
        public Action intakeRun() {
            return new IntakeRunAction(intakeV2, 2.0, -1.0, 0);
        }

        /** Runs transfer motor briefly (0.3s) to move ball into launcher. Use with intakeRun() in ParallelAction. */
        public Action transRun() {
            return new TransRunAction(intakeV2, 0.3, -0.2, 0);
        }
    }

    private static class IntakeRunAction implements Action {
        private final IntakeV2 intakeV2;
        private final double duration;
        private final double runPower;
        private final double stopPower;
        private final ElapsedTime timer = new ElapsedTime();
        private boolean init;

        IntakeRunAction(IntakeV2 intakeV2, double duration, double runPower, double stopPower) {
            this.intakeV2 = intakeV2;
            this.duration = duration;
            this.runPower = runPower;
            this.stopPower = stopPower;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket p) {
            if (!init) {
                timer.reset();
                init = true;
                intakeV2.setIntakeMotorAuto(runPower);
            }
            if (timer.seconds() >= duration) {
                intakeV2.setIntakeMotorAuto(stopPower);
                return false;
            }
            return true;
        }
    }

    private static class TransRunAction implements Action {
        private final IntakeV2 intakeV2;
        private final double duration;
        private final double runPower;
        private final double stopPower;
        private final ElapsedTime timer = new ElapsedTime();
        private boolean init;

        TransRunAction(IntakeV2 intakeV2, double duration, double runPower, double stopPower) {
            this.intakeV2 = intakeV2;
            this.duration = duration;
            this.runPower = runPower;
            this.stopPower = stopPower;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket p) {
            if (!init) {
                timer.reset();
                init = true;
                intakeV2.setTransferMotorAuto(runPower);
            }
            if (timer.seconds() >= duration) {
                intakeV2.setTransferMotorAuto(stopPower);
                return false;
            }
            return true;
        }
    }

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
                        } else if (detection.id == 22) {
                            motif.add("p");
                            motif.add("g");
                            motif.add("p");
                        }
                        else if (detection.id == 23){
                            motif.add("p");
                            motif.add("p");
                            motif.add("g");
                        }
                    }
                }

            }
            return motif.size() != 3;
        }
    }

    public Action scanMotif(){return new scan();}
    private static final boolean USE_WEBCAM = true;  // true for webcam, false for phone camera

    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;

    ArrayList<String> stored = new ArrayList<>();
    ArrayList<String> motif = new ArrayList<>();

    /** Set false when main sequence ends so background TurretAimAction stops. */
    private volatile boolean autoRunning = true;

    @Override
    public void runOpMode() {


        // Start pose = teleop red (MainDrive). Path waypoints mirrored from Blue Near (Y flipped, heading 270→90).
        Pose2d initialPose = new Pose2d(MainDrive.redStartX, MainDrive.redStartY, Math.toRadians(MainDrive.redStartHeadingDeg));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
        drive.localizer.setPose(initialPose);
        AprilTagProcessor aprilTag = new AprilTagProcessor.Builder().build();
        IntakeV2 intake = new IntakeV2(hardwareMap, gamepad1, gamepad2, aprilTag);
        IntakeSub intakeSub = new IntakeSub(intake);
        Turret turret = new Turret(hardwareMap, gamepad2);
        Servo led = hardwareMap.get(Servo.class, "LED");

        telemetry.addData(">", "Red Near. Start = teleop red. Shot physics + goal aim from teleop.");
        telemetry.update();
        waitForStart();

        // Trajectories mirrored from Blue Near: Y flipped, 270°→90°, 315°→45°, 90°→270°
        Action tab1 = drive.actionBuilder(initialPose)
                .strafeToLinearHeading(new Vector2d(-12, 12), Math.toRadians(90)) // Preload
                .build();
        Action tab2 = drive.actionBuilder(new Pose2d(-12, 12, Math.toRadians(90)))
                .strafeToLinearHeading(new Vector2d(-12, 28), Math.toRadians(90)) // first spike
                .strafeTo(new Vector2d(-12, 53)) // first spike intake
                .build();
        Action tab3 = drive.actionBuilder(new Pose2d(-12, 53, Math.toRadians(90)))
                .strafeToLinearHeading(new Vector2d(-12, 13), Math.toRadians(90)) // launch first spike
                .build();
        Action tab4 = drive.actionBuilder(new Pose2d(-12, 13, Math.toRadians(90)))
                .setTangent(Math.toRadians(45))
                .splineToLinearHeading(new Pose2d(12, 28, Math.toRadians(90)), Math.toRadians(90)) // second spike
                .strafeTo(new Vector2d(12, 53)) // second spike intake
                .build();
        Action tab5 = drive.actionBuilder(new Pose2d(12, 53, Math.toRadians(90)))
                .setTangent(Math.toRadians(270))
                .splineToLinearHeading(new Pose2d(-13, 12, Math.toRadians(90)), Math.toRadians(270))
                .build();
        // Gate: drive to gate, intake, back to shoot zone (mirrored from Blue: Y flipped, 235→125, 250→110)
        Action tab6_gate = drive.actionBuilder(new Pose2d(-13, 12, Math.toRadians(90)))
                .strafeToLinearHeading(new Vector2d(12, 59), Math.toRadians(125))
                .build();
        Action tab7_back = drive.actionBuilder(new Pose2d(12, 59, Math.toRadians(125)))
                .strafeToLinearHeading(new Vector2d(-13, 12), Math.toRadians(110))
                .build();
        Action tab8_gate = drive.actionBuilder(new Pose2d(-13, 12, Math.toRadians(110)))
                .strafeToLinearHeading(new Vector2d(12, 59), Math.toRadians(125))
                .build();
        Action tab10_park = drive.actionBuilder(new Pose2d(12, 59, Math.toRadians(125)))
                .strafeToLinearHeading(new Vector2d(-35, 15), Math.toRadians(128))
                .build();


        if (opModeIsActive()) {
            autoRunning = true;
            Action mainSequence = new SequentialAction(
                    tab1,
                    new RevAndAimAction(drive, intake, turret, 1.0),
                    new ShootAction(drive, intake, turret, 1.0),
                    new ParallelAction(tab2, intakeSub.intakeRun(), intakeSub.transRun()),
                    tab3,
                    new RevAndAimAction(drive, intake, turret, 1.0),
                    new ShootAction(drive, intake, turret, 1.0),
                    new ParallelAction(
                            tab4,
                            new SequentialAction(
                                    new SleepAction(0.5),
                                    new ParallelAction(intakeSub.intakeRun(), intakeSub.transRun())
                            )
                    ),
                    tab5,
                    new RevAndAimAction(drive, intake, turret, 1.0),
                    new ShootAction(drive, intake, turret, 1.0),
                    new ParallelAction(
                            tab6_gate,
                            new SequentialAction(
                                    new SleepAction(1.0),
                                    new ParallelAction(intakeSub.intakeRun(), intakeSub.transRun())
                            )
                    ),
                    tab7_back,
                    new RevAndAimAction(drive, intake, turret, 1.0),
                    new ShootAction(drive, intake, turret, 1.0),
                    new ParallelAction(
                            tab8_gate,
                            new SequentialAction(
                                    new SleepAction(1.0),
                                    new ParallelAction(intakeSub.intakeRun(), intakeSub.transRun())
                            )
                    ),
                    tab10_park,
                    new StopLauncherAction(intake),
                    new SetFlagAndEndAction()
            );
            Actions.runBlocking(new ParallelAction(
                    new TurretAimAction(drive, turret, GOAL_X, GOAL_Y),
                    new HoodAndFlywheelUpdateAction(drive, intake, GOAL_X, GOAL_Y),
                    new LedFadeAction(led, () -> autoRunning),
                    mainSequence
            ));

            PoseBridge.save(drive.localizer.getPose());
            PoseBridge.saveAlliance(false);  // Red
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
                return false;  // Launcher keeps revving via HoodAndFlywheelUpdateAction
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

    /** Continuously aims turret at red goal using odometry pose. Runs in parallel with main sequence until it ends. */
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
            if (!autoRunning) {
                turret.turretPow(0);
                return false;
            }
            drive.updatePoseEstimate();
            Pose2d pose = drive.localizer.getPose();
            double dx = goalX - pose.position.x;
            double dy = goalY - pose.position.y;
            double angleToGoalDeg = Turret.wrapDeg360(Math.toDegrees(Math.atan2(dy, dx)));
            turret.botHeading = Turret.wrapDeg360(Math.toDegrees(pose.heading.toDouble()));
            turret.targetAngle = angleToGoalDeg;
            turret.runTurretGyro();
            return true;
        }
    }

    /** Continuously updates hood angle and flywheel target velocity from current odometry distance to red goal. Runs in parallel with main sequence. */
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
            intake.runLauncherAuto(false); // rev the whole time
            return autoRunning;
        }
    }

    private void initAprilTag() {
        aprilTag = new AprilTagProcessor.Builder().build();
        VisionPortal.Builder builder = new VisionPortal.Builder();
        if (USE_WEBCAM) {
            builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam1"));
        } else {
            builder.setCamera(BuiltinCameraDirection.BACK);
        }
        builder.addProcessor(aprilTag);
        visionPortal = builder.build();
    }

    private void telemetryAprilTag() {
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        telemetry.addData("# AprilTags Detected", currentDetections.size());
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
            } else if (detection.id == 22) {
                motif.add("p");
                motif.add("g");
                motif.add("p");
            }
            else if (detection.id == 23){
                motif.add("p");
                motif.add("p");
                motif.add("g");
            }
        }
        telemetry.addLine("\nkey:\nXYZ = X (Right), Y (Forward), Z (Up) dist.");
        telemetry.addLine("PRY = Pitch, Roll & Yaw (XYZ Rotation)");
        telemetry.addLine("RBE = Range, Bearing & Elevation");
    }

}
