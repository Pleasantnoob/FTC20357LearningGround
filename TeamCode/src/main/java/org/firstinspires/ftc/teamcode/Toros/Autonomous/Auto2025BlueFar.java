package org.firstinspires.ftc.teamcode.Toros.Autonomous;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import org.firstinspires.ftc.teamcode.util.Action;
import org.firstinspires.ftc.teamcode.util.Actions;
import org.firstinspires.ftc.teamcode.util.FollowPathAction;
import org.firstinspires.ftc.teamcode.util.ParallelAction;
import org.firstinspires.ftc.teamcode.util.Pose2d;
import org.firstinspires.ftc.teamcode.util.SequentialAction;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Toros.Drive.PedroDrive;
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
        PedroDrive drive = new PedroDrive(hardwareMap, initialPose);
        aprilTag = new AprilTagProcessor.Builder().build();
        IntakeV2 intake = new IntakeV2(hardwareMap, gamepad1, gamepad2, aprilTag);
        Turret turret = new Turret(hardwareMap, gamepad2);
        Servo led = hardwareMap.get(Servo.class, "LED");

        telemetry.addData(">", "Blue Far. Trajectories from blueFar (MeepMeep).");
        telemetry.update();
        waitForStart();

        double r270 = Math.toRadians(270);
        Pose2d p1 = new Pose2d(60, -62, r270);
        Pose2d p2a = new Pose2d(50, -62, r270);
        Pose2d p2b = new Pose2d(60, -50, r270);
        Pose2d p3 = new Pose2d(60, -20, r270);
        Pose2d p4 = new Pose2d(60, -60, r270);
        Pose2d p5 = new Pose2d(50, -30, r270);
        Action tab1 = new FollowPathAction(drive, drive.buildPath(initialPose, p1));
        Action tab2 = new FollowPathAction(drive, drive.buildPathChain(p1, p2a, p2b, p1, p2a));
        Action tab3 = new FollowPathAction(drive, drive.buildPath(p2a, p3));
        Action tab4 = new FollowPathAction(drive, drive.buildPath(p3, p4));
        Action tab5 = new FollowPathAction(drive, drive.buildPath(p4, p3));
        Action tab6 = new FollowPathAction(drive, drive.buildPath(p3, p4));
        Action tab7 = new FollowPathAction(drive, drive.buildPath(p4, p3));
        Action tab10 = new FollowPathAction(drive, drive.buildPath(p3, p5));

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
            Actions.runBlocking(this, new ParallelAction(
                    new TurretAimAction(drive, turret, GOAL_X, GOAL_Y),
                    new HoodAndFlywheelUpdateAction(drive, intake, GOAL_X, GOAL_Y),
                    new LedFadeAction(led, () -> autoRunning),
                    mainSequence
            ));

            PoseBridge.save(drive.getPose());
            PoseBridge.saveAlliance(true);  // Blue
            intake.stopLauncherAuto();
            intake.runIntakeAuto(false);

            while (opModeIsActive()) {
                telemetry.addData("Pose", drive.getPose());
                telemetry.update();
                sleep(20);
            }
            } finally {
                IntakeV2.shootVelToleranceOverride = 0;  // reset for other opmodes
            }
        }
    }

    private static class RevAndAimAction implements Action {
        private final PedroDrive drive;
        private final IntakeV2 intake;
        private final Turret turret;
        private final double duration;
        private final ElapsedTime timer = new ElapsedTime();
        private boolean init;

        RevAndAimAction(PedroDrive drive, IntakeV2 intake, Turret turret, double duration) {
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
            drive.update();
            Pose2d pose = drive.getPose();
            double dx = GOAL_X - pose.position.x;
            double dy = GOAL_Y - pose.position.y;
            double dist = Math.hypot(dx, dy);
            double angleToGoalDeg = Turret.wrapDeg360(Math.toDegrees(Math.atan2(dy, dx)));
            intake.setHoodAndFlywheelFromDistance(dist);
            intake.runLauncherAuto(false);
            turret.botHeading = Turret.wrapDeg360(Math.toDegrees(pose.heading));
            turret.targetAngle = angleToGoalDeg;
            turret.runTurretGyro();
            return timer.seconds() < duration;
        }
    }

    private static class ShootAction implements Action {
        private final Auto2025BlueFar outer;
        private final PedroDrive drive;
        private final IntakeV2 intake;
        private final Turret turret;
        private final double duration;
        private final ElapsedTime timer = new ElapsedTime();
        private boolean init;

        ShootAction(Auto2025BlueFar outer, PedroDrive drive, IntakeV2 intake, Turret turret, double duration) {
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
            drive.update();
            Pose2d pose = drive.getPose();
            double dx = GOAL_X - pose.position.x;
            double dy = GOAL_Y - pose.position.y;
            double dist = Math.hypot(dx, dy);
            double angleToGoalDeg = Turret.wrapDeg360(Math.toDegrees(Math.atan2(dy, dx)));
            intake.setHoodAndFlywheelFromDistance(dist);
            intake.runLauncherAuto(true);
            turret.botHeading = Turret.wrapDeg360(Math.toDegrees(pose.heading));
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
        private final PedroDrive drive;
        private final Turret turret;
        private final double goalX;
        private final double goalY;

        TurretAimAction(PedroDrive drive, Turret turret, double goalX, double goalY) {
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
            drive.update();
            Pose2d pose = drive.getPose();
            double dx = goalX - pose.position.x;
            double dy = goalY - pose.position.y;
            double angleToGoalDeg = Turret.wrapDeg360(Math.toDegrees(Math.atan2(dy, dx)));
            turret.botHeading = Turret.wrapDeg360(Math.toDegrees(pose.heading));
            turret.targetAngle = angleToGoalDeg;
            turret.runTurretGyro();
            return true;
        }
    }

    private class HoodAndFlywheelUpdateAction implements Action {
        private final PedroDrive drive;
        private final IntakeV2 intake;
        private final double goalX;
        private final double goalY;

        HoodAndFlywheelUpdateAction(PedroDrive drive, IntakeV2 intake, double goalX, double goalY) {
            this.drive = drive;
            this.intake = intake;
            this.goalX = goalX;
            this.goalY = goalY;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket p) {
            drive.update();
            Pose2d pose = drive.getPose();
            double dist = Math.hypot(goalX - pose.position.x, goalY - pose.position.y);
            intake.setHoodAndFlywheelFromDistance(dist);
            intake.runLauncherAuto(isShooting);
            return autoRunning;
        }
    }
}
