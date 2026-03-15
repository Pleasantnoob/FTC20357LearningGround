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
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Toros.Drive.PedroDrive;
import org.firstinspires.ftc.teamcode.RR.PoseBridge;
import org.firstinspires.ftc.teamcode.Toros.Drive.Subsystems.IntakeV2;
import org.firstinspires.ftc.teamcode.Toros.Drive.Subsystems.Turret;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

/**
 * 12-ball autonomous: 3 preloaded + 9 from spike marks.
 * Uses same subsystems as teleop: MecanumDrive (Pinpoint), IntakeV2 (regression launcher), Turret (aim at goal).
 * Blue alliance: start (-55, -47) @ -130°, goal (-70, -64). Path: shoot 3 → spike 1 → shoot 3 → spike 2 → shoot 3 → spike 3 → shoot 3.
 */
@Autonomous(name = "Auto 12-Ball (Blue)")
public class Auto12Ball extends LinearOpMode {

    /** Blue goal (inches). Match MainDrive.blueGoalX/Y. */
    private static final double GOAL_X = -70.0;
    private static final double GOAL_Y = -64.0;
    /** Blue start. Match MainDrive.blueStart*. */
    private static final double START_X = -55.0;
    private static final double START_Y = -47.0;
    private static final double START_HEADING_RAD = Math.toRadians(-130.0);
    /** Shoot position: face goal, ~35–40 in from goal. */
    private static final double SHOOT_X = 38.0;
    private static final double SHOOT_Y = -52.0;
    private static final double SHOOT_HEADING_RAD = Math.PI;
    /** Spike mark waypoints (audience side, then into spike). Blue: negative Y. */
    private static final double SPIKE1_X = -12.0, SPIKE1_Y_END = -53.0;
    private static final double SPIKE2_X = 12.0,  SPIKE2_Y_END = -53.0;
    private static final double SPIKE3_X = 30.0, SPIKE3_Y_END = -55.0;

    private PedroDrive drive;
    private IntakeV2 intake;
    private Turret turret;
    private AprilTagProcessor aprilTag;

    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d initialPose = new Pose2d(START_X, START_Y, START_HEADING_RAD);
        drive = new PedroDrive(hardwareMap, initialPose);
        aprilTag = new AprilTagProcessor.Builder().build();
        intake = new IntakeV2(hardwareMap, gamepad1, gamepad2, aprilTag);
        turret = new Turret(hardwareMap, gamepad2);

        telemetry.addData(">", "12-ball Blue. Start when ready.");
        telemetry.update();
        waitForStart();

        if (!opModeIsActive()) return;

        Pose2d shootPose = new Pose2d(SHOOT_X, SHOOT_Y, SHOOT_HEADING_RAD);
        Pose2d spike1 = new Pose2d(SPIKE1_X, SPIKE1_Y_END - 15, SHOOT_HEADING_RAD);
        Pose2d spike1End = new Pose2d(SPIKE1_X, SPIKE1_Y_END, SHOOT_HEADING_RAD);
        Pose2d spike2 = new Pose2d(SPIKE2_X, SPIKE2_Y_END - 15, SHOOT_HEADING_RAD);
        Pose2d spike2End = new Pose2d(SPIKE2_X, SPIKE2_Y_END, SHOOT_HEADING_RAD);
        Pose2d spike3 = new Pose2d(SPIKE3_X, SPIKE3_Y_END - 15, SHOOT_HEADING_RAD);
        Pose2d spike3End = new Pose2d(SPIKE3_X, SPIKE3_Y_END, SHOOT_HEADING_RAD);
        Action toShoot = new FollowPathAction(drive, drive.buildPath(initialPose, shootPose));
        Action revAndAim = new RevAndAimAction(drive, intake, turret, 2.0);
        Action shoot3 = new ShootAction(drive, intake, turret, 4.0);
        Action toSpike1 = new FollowPathAction(drive, drive.buildPathChain(shootPose, spike1, spike1End));
        Action intakeRun1 = new IntakeRunAction(intake, 3.0);
        Action backToShoot1 = new FollowPathAction(drive, drive.buildPath(spike1End, shootPose));
        Action toSpike2 = new FollowPathAction(drive, drive.buildPathChain(shootPose, spike2, spike2End));
        Action intakeRun2 = new IntakeRunAction(intake, 3.0);
        Action backToShoot2 = new FollowPathAction(drive, drive.buildPath(spike2End, shootPose));
        Action toSpike3 = new FollowPathAction(drive, drive.buildPathChain(shootPose, spike3, spike3End));
        Action intakeRun3 = new IntakeRunAction(intake, 3.0);
        Action backToShoot3 = new FollowPathAction(drive, drive.buildPath(spike3End, shootPose));

        SequentialAction main = new SequentialAction(
                toShoot,
                revAndAim,
                shoot3,
                new ParallelAction(toSpike1, intakeRun1),
                backToShoot1,
                revAndAim,
                shoot3,
                new ParallelAction(toSpike2, intakeRun2),
                backToShoot2,
                revAndAim,
                shoot3,
                new ParallelAction(toSpike3, intakeRun3),
                backToShoot3,
                revAndAim,
                shoot3,
                new StopLauncherAction(intake)
        );

        Actions.runBlocking(this, main);

        PoseBridge.save(drive.getPose());
        PoseBridge.saveAlliance(true);  // Blue
        intake.stopLauncherAuto();
        intake.runIntakeAuto(false);
    }

    /** Runs rev + turret aim for duration seconds. Uses pose for distance and angle to goal. */
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

    /** Runs rev + aim + feed for duration seconds (shoot). */
    private static class ShootAction implements Action {
        private final PedroDrive drive;
        private final IntakeV2 intake;
        private final Turret turret;
        private final double duration;
        private final ElapsedTime timer = new ElapsedTime();
        private boolean init;

        ShootAction(PedroDrive drive, IntakeV2 intake, Turret turret, double duration) {
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
            intake.runLauncherAuto(true);
            turret.botHeading = Turret.wrapDeg360(Math.toDegrees(pose.heading));
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
}
