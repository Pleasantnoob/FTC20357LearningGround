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
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.controller.wpilibcontroller.SimpleMotorFeedforward;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.RR.MecanumDrive;
import org.firstinspires.ftc.teamcode.Toros.Drive.MainDrive;
import org.firstinspires.ftc.teamcode.Toros.Drive.Subsystems.Turret;

import java.util.function.BooleanSupplier;

/**
 * Red Near test: same paths, shoot times, and launcher/intake logic as Auto2025RedNear.
 * Turret runs in parallel and continuously aims at red goal via odometry (no fixed angles).
 */
@Autonomous(name = "Red Near Odometry Aim Test")
public class Auto2025RedNearOdometryAim extends LinearOpMode {

    public DcMotorEx launch, trans;
    public Servo hood;
    public ColorSensor c1, c2, c3;
    private DcMotor intake;
    private PIDController controller;

    public static double p1 = 0.0045, i1 = 0, d1 = 0;
    public static double kS1 = 0.001, kV1 = 0.00055, kA1 = 0;
    public static double accel = 20;
    public static int targetVel = -1275;  // match Auto2025RedNear

    /** Set to false when main sequence ends so parallel turret/rev actions stop. */
    private volatile boolean autoRunning = true;

    public class Launcher {
        public Launcher(HardwareMap hardwareMap) {
            launch = hardwareMap.get(DcMotorEx.class, "launch");
            launch.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            controller = new PIDController(p1, i1, d1);
            intake = hardwareMap.get(DcMotor.class, "intake");
            intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            trans = hardwareMap.get(DcMotorEx.class, "trans");
            trans.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            hood = hardwareMap.get(Servo.class, "hood");
        }

        public class launcherAction implements Action {
            private boolean init = false;
            ElapsedTime timer = new ElapsedTime();

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                if (!init) {
                    timer.reset();
                    init = true;
                    hood.setPosition(1);
                }
                if (launch.getVelocity() <= -1230) {
                    trans.setPower(-1);
                    intake.setPower(-1);
                } else {
                    trans.setPower(0);
                    intake.setPower(0);
                }
                if (timer.seconds() < 1.5) return true;
                launch.setPower(0);
                trans.setPower(0);
                intake.setPower(0);
                return false;
            }
        }

        public Action fireBall() { return new launcherAction(); }

        public class launcherActionPre implements Action {
            private boolean init = false;
            ElapsedTime timer = new ElapsedTime();

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                if (!init) {
                    timer.reset();
                    init = true;
                    hood.setPosition(1);
                }
                if (launch.getVelocity() <= -1230) {
                    trans.setPower(-1);
                    intake.setPower(-0.67);
                } else {
                    trans.setPower(0);
                    intake.setPower(0);
                }
                if (timer.seconds() < 3) return true;
                launch.setPower(0);
                trans.setPower(0);
                intake.setPower(0);
                return false;
            }
        }

        public Action fireBallPre() { return new launcherActionPre(); }

        public class revLaunch implements Action {
            boolean init = false;

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                if (!init) {
                    controller.setPID(p1, i1, d1);
                    init = true;
                }
                double launchVel = launch.getVelocity();
                SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(kS1, kV1, kA1);
                double pid = controller.calculate(launchVel, targetVel);
                double ff = feedforward.calculate(targetVel, accel);
                launch.setPower(pid + ff);
                return true;
            }
        }

        public Action revMotor() { return new revLaunch(); }
    }

    public class Intake {
        public Intake(HardwareMap hardwareMap) {
            intake = hardwareMap.get(DcMotor.class, "intake");
            intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            trans = hardwareMap.get(DcMotorEx.class, "trans");
            trans.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        public class intakeAction implements Action {
            private boolean init = false;
            ElapsedTime timer;

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                if (!init) {
                    trans.setPower(-0.18);
                    intake.setPower(-0.55);
                    init = true;
                    timer = new ElapsedTime();
                }
                return timer.seconds() < 3;
            }
        }

        public Action takeBall() { return new intakeAction(); }

        public class runTrans implements Action {
            boolean init = false;
            ElapsedTime timer;

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                if (!init) {
                    timer = new ElapsedTime();
                    trans.setPower(-0.1);
                    init = true;
                }
                if (timer.seconds() < 0.8) return true;
                trans.setPower(0);
                return false;
            }
        }

        public Action transRun() { return new runTrans(); }

        public class runIntake implements Action {
            private boolean init = false;
            ElapsedTime timer;

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                if (!init) {
                    intake.setPower(-0.57);
                    init = true;
                    timer = new ElapsedTime();
                }
                if (timer.seconds() < 1.9) return true;
                intake.setPower(0);
                return false;
            }
        }

        public Action intakeRun() { return new runIntake(); }
    }

    /** Runs inner action but returns false when keepRunning returns false (so parallel ends with main sequence). */
    private class RunUntilFlagAction implements Action {
        private final Action inner;
        private final BooleanSupplier keepRunning;

        RunUntilFlagAction(Action inner, BooleanSupplier keepRunning) {
            this.inner = inner;
            this.keepRunning = keepRunning;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket p) {
            return inner.run(p) && keepRunning.getAsBoolean();
        }
    }

    /** Sets autoRunning = false and returns false so this action (and parallel) can end. */
    private class SetFlagAndEndAction implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket p) {
            autoRunning = false;
            return false;
        }
    }

    /** Continuously aims turret at red goal using odometry pose. Returns false when autoRunning is false. */
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

    @Override
    public void runOpMode() {
        Pose2d initialPose = new Pose2d(-48, 50, Math.toRadians(90));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
        Launcher launcher = new Launcher(hardwareMap);
        Intake intake = new Intake(hardwareMap);
        Turret turret = new Turret(hardwareMap, gamepad2);

        telemetry.addData(">", "Red Near Odometry Aim Test: same paths & shoot times, turret aims at goal whole time");
        telemetry.update();
        waitForStart();

        autoRunning = true;

        // Same paths as Auto2025RedNear
        Action tab1 = drive.actionBuilder(initialPose)
                .strafeTo(new Vector2d(-13, 13), new TranslationalVelConstraint(15.0))
                .build();
        Action tab2 = drive.actionBuilder(new Pose2d(-13, 13, Math.toRadians(90)))
                .strafeTo(new Vector2d(-13, 49), new TranslationalVelConstraint(100.0))
                .build();
        Action tab3 = drive.actionBuilder(new Pose2d(-13, 49, Math.toRadians(90)))
                .strafeTo(new Vector2d(-13, 13))
                .build();
        Action tab4 = drive.actionBuilder(new Pose2d(-13, 13, Math.toRadians(90)))
                .strafeTo(new Vector2d(9, 28))
                .strafeTo(new Vector2d(9, 55), new TranslationalVelConstraint(100.0))
                .build();
        Action tab5 = drive.actionBuilder(new Pose2d(9, 55, Math.toRadians(90)))
                .strafeTo(new Vector2d(-13, 13))
                .build();
        Action tab6 = drive.actionBuilder(new Pose2d(-13, 13, Math.toRadians(90)))
                .strafeTo(new Vector2d(37, 20), new TranslationalVelConstraint(100.0))
                .strafeTo(new Vector2d(37, 50), new TranslationalVelConstraint(100.0))
                .build();
        Action tab7 = drive.actionBuilder(new Pose2d(35, 50, Math.toRadians(90)))
                .strafeTo(new Vector2d(-13, 13))
                .build();
        Action tab8 = drive.actionBuilder(new Pose2d(-13, 13, Math.toRadians(90)))
                .strafeToLinearHeading(new Vector2d(0, 20), Math.toRadians(180))
                .build();

        Action mainSequence = new SequentialAction(
                tab1,
                launcher.fireBallPre(),
                new ParallelAction(tab2, intake.intakeRun(), intake.transRun()),
                tab3,
                launcher.fireBall(),
                new ParallelAction(
                        tab4,
                        new SequentialAction(
                                new SleepAction(1.1),
                                new ParallelAction(intake.intakeRun(), intake.transRun())
                        )
                ),
                tab5,
                launcher.fireBall(),
                new ParallelAction(
                        tab6,
                        new SequentialAction(
                                new SleepAction(2.1),
                                new ParallelAction(intake.intakeRun(), intake.transRun())
                        )
                ),
                tab7,
                launcher.fireBall(),
                tab8,
                new SetFlagAndEndAction()
        );

        Actions.runBlocking(
                new ParallelAction(
                        new RunUntilFlagAction(launcher.revMotor(), () -> autoRunning),
                        new TurretAimAction(drive, turret, MainDrive.redGoalX, MainDrive.redGoalY),
                        mainSequence
                )
        );

        launch.setPower(0);
        trans.setPower(0);
        intake.setPower(0);

        while (opModeIsActive()) {
            telemetry.addData("Pose", drive.localizer.getPose());
            telemetry.update();
            sleep(20);
        }
    }
}
