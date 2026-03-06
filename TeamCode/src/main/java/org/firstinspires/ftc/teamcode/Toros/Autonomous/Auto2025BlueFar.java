package org.firstinspires.ftc.teamcode.Toros.Autonomous;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.RR.MecanumDrive;
import org.firstinspires.ftc.teamcode.RR.PoseBridge;
import org.firstinspires.ftc.teamcode.Toros.Drive.MainDrive;
import org.firstinspires.ftc.teamcode.Toros.Drive.Subsystems.IntakeV2;
import org.firstinspires.ftc.teamcode.Toros.Drive.Subsystems.Turret;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

/**
 * Blue far autonomous. Mirror of Red Far: uses blueFar trajectories from MeepMeep (negative Y).
 * Start: (60, -10, 270°). Same structure as Red Far (intake off when back-to-shoot, shooter revs whole time).
 */
@Autonomous(name = "Auto2025BlueFar")
public class Auto2025BlueFar extends LinearOpMode {

    private static final double GOAL_X = MainDrive.blueGoalX;
    private static final double GOAL_Y = MainDrive.blueGoalY;

    private static class StartIntakeAction implements Action {
        private final IntakeV2 intakeV2;

        StartIntakeAction(IntakeV2 intakeV2) {
            this.intakeV2 = intakeV2;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket p) {
            intakeV2.setIntakeMotorAuto(-1.0);
            intakeV2.setTransferMotorAuto(0);  // Transfer off when picking up balls (Far autos)
            return false;
        }
    }

    private static class StopIntakeAction implements Action {
        private final IntakeV2 intakeV2;

        StopIntakeAction(IntakeV2 intakeV2) {
            this.intakeV2 = intakeV2;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket p) {
            intakeV2.setIntakeMotorAuto(0);
            intakeV2.setTransferMotorAuto(0);
            return false;
        }
    }

    private AprilTagProcessor aprilTag;
    private volatile boolean autoRunning = true;
    private volatile boolean isShooting = false;

    @Override
    public void runOpMode() {
        Pose2d initialPose = new Pose2d(60, -10, Math.toRadians(270));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
        drive.localizer.setPose(initialPose);
        aprilTag = new AprilTagProcessor.Builder().build();
        IntakeV2 intake = new IntakeV2(hardwareMap, gamepad1, gamepad2, aprilTag);
        Turret turret = new Turret(hardwareMap, gamepad2);
        Servo led = hardwareMap.get(Servo.class, "LED");

        telemetry.addData(">", "Blue Far. Trajectories from blueFar (MeepMeep).");
        telemetry.update();
        waitForStart();

        // Trajectories from blueFar (MeepMeep), negative Y
        Action tab1 = drive.actionBuilder(initialPose)
                .strafeToLinearHeading(new Vector2d(60, -62), Math.toRadians(270))
                .build();
        Action tab2 = drive.actionBuilder(new Pose2d(60, -62, Math.toRadians(270)))
                .strafeToLinearHeading(new Vector2d(50, -62), Math.toRadians(270))
                .strafeToLinearHeading(new Vector2d(60, -50), Math.toRadians(270))
                .strafeToLinearHeading(new Vector2d(60, -62), Math.toRadians(270))
                .strafeToLinearHeading(new Vector2d(50, -62), Math.toRadians(270))
                .build();
        Action tab3 = drive.actionBuilder(new Pose2d(50, -62, Math.toRadians(270)))
                .strafeToLinearHeading(new Vector2d(60, -20), Math.toRadians(270), new TranslationalVelConstraint(100))
                .build();
        Action tab4 = drive.actionBuilder(new Pose2d(60, -20, Math.toRadians(270)))
                .strafeTo(new Vector2d(60, -60))
                .build();
        Action tab5 = drive.actionBuilder(new Pose2d(60, -60, Math.toRadians(270)))
                .strafeTo(new Vector2d(60, -20))
                .build();
        Action tab6 = drive.actionBuilder(new Pose2d(60, -20, Math.toRadians(270)))
                .strafeTo(new Vector2d(60, -60))
                .build();
        Action tab7 = drive.actionBuilder(new Pose2d(60, -60, Math.toRadians(270)))
                .strafeTo(new Vector2d(60, -20))
                .build();
        Action tab10 = drive.actionBuilder(new Pose2d(60, -20, Math.toRadians(270)))
                .strafeTo(new Vector2d(50, -30))
                .build();

        if (opModeIsActive()) {
            try {
            autoRunning = true;
            IntakeV2.shootVelToleranceOverride = 15;  // ±15 for Far auto (tighter than default 20)
            // Rev 4s, shoot preload, then start pathing. Shoot every time we arrive at (60,-20).
            Action mainSequence = new SequentialAction(
                    new RevAndAimAction(drive, intake, turret, 4.0),
                    new ShootAction(this, drive, intake, turret, 1.0),
                    tab1,
                    new StartIntakeAction(intake),
                    tab2,
                    new StopIntakeAction(intake),
                    tab3,
                    new RevAndAimAction(drive, intake, turret, 1.0),
                    new ShootAction(this, drive, intake, turret, 1.0),
                    new StartIntakeAction(intake),
                    tab4,
                    new StopIntakeAction(intake),
                    tab5,
                    new RevAndAimAction(drive, intake, turret, 1.0),
                    new ShootAction(this, drive, intake, turret, 1.0),
                    new StartIntakeAction(intake),
                    tab6,
                    new StopIntakeAction(intake),
                    tab7,
                    new RevAndAimAction(drive, intake, turret, 1.0),
                    new ShootAction(this, drive, intake, turret, 1.0),
                    tab10,
                    new StopIntakeAction(intake),
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
            PoseBridge.saveAlliance(true);  // Blue
            intake.stopLauncherAuto();
            intake.runIntakeAuto(false);

            while (opModeIsActive()) {
                telemetry.addData("Pose", drive.localizer.getPose());
                telemetry.update();
                sleep(20);
            }
            } finally {
                IntakeV2.shootVelToleranceOverride = 0;  // reset for other opmodes
            }
        }
    }

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

    private static class ShootAction implements Action {
        private final Auto2025BlueFar outer;
        private final MecanumDrive drive;
        private final IntakeV2 intake;
        private final Turret turret;
        private final double duration;
        private final ElapsedTime timer = new ElapsedTime();
        private boolean init;

        ShootAction(Auto2025BlueFar outer, MecanumDrive drive, IntakeV2 intake, Turret turret, double duration) {
            this.outer = outer;
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
                outer.isShooting = true;
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
                outer.isShooting = false;
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

    private class SetFlagAndEndAction implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket p) {
            autoRunning = false;
            return false;
        }
    }

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
            intake.runLauncherAuto(isShooting);
            return autoRunning;
        }
    }
}
