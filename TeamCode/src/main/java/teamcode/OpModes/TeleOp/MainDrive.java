package teamcode.OpModes.TeleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import teamcode.ComplexMath.Storage.PoseStorage;
import teamcode.RobotContainer.DriveProcesses;
import teamcode.RobotContainer.Robot;
import teamcode.pedro.Constants;


@TeleOp
public class MainDrive extends LinearOpMode {

    private final PoseStorage poseStorage = new PoseStorage();
    private Robot robot;

    @Override
    public void runOpMode() {
        robot = new Robot(hardwareMap);
        try {
            DriveProcesses driveProcesses = new DriveProcesses(robot, gamepad1, gamepad2);

            Pose driveStartPose = new Pose(
                    poseStorage.getPosX(),
                    poseStorage.getPosY(),
                    poseStorage.getRobotHeading());

            Follower driveFollower = Constants.create(hardwareMap);
            driveFollower.setPose(driveStartPose);

            telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

            waitForStart();
            ElapsedTime loopTimer = new ElapsedTime();
            while (opModeIsActive()) {
                double dt = loopTimer.seconds();
                loopTimer.reset();

                double vertical = gamepad1.left_stick_y;
                double turn = gamepad1.right_stick_x;
                double strafe = gamepad1.left_stick_x;

                driveFollower.update();
                Pose pose = driveFollower.pose();
                poseStorage.savePoseToStorage(pose.x(), pose.y(), pose.heading());

                driveProcesses.update(pose.x(), pose.y(), pose.heading(), dt);
                robot.drivetrain.updateDriveMotorPowers(vertical, turn, strafe);

                telemetry.addData("lf_motor_power", robot.drivetrain.getLFMotorPower());
                telemetry.addData("lb_motor_power", robot.drivetrain.getLBMotorPower());
                telemetry.addData("rf_motor_power", robot.drivetrain.getRFMotorPower());
                telemetry.addData("rb_motor_power", robot.drivetrain.getRBMotorPower());
                telemetry.addData("ShooterRPM", robot.shooter.getRPM());
                telemetry.addData("Target RPM", robot.shooter.getTargetRPM());
                telemetry.addData("Position", "%.1f, %.1f, %.1f deg",
                        pose.x(), pose.y(), Math.toDegrees(pose.heading()));
                telemetry.addData("alliance", poseStorage.getAlliance());
                telemetry.addData("targetPosX", robot.turret.getTurretTargetX());
                telemetry.addData("targetPosY", robot.turret.getTurretTargetY());
                telemetry.addData("turret heading", robot.turret.getTurretHeading());
                telemetry.addData("turret servo", robot.turret.getLeftServoPosition());
                telemetry.addData("turret mode", robot.turret.getAimMode());
                telemetry.addData("gp1", controllerLine(gamepad1));
                telemetry.addData("gp2", controllerLine(gamepad2));
                telemetry.update();
            }
        } finally {
            robot.stop();
        }
    }

    private String controllerLine(Gamepad pad) {
        String typeName = pad.type == null ? "UNKNOWN" : pad.type.name();
        if (typeName.equals("UNKNOWN") || typeName.equals("LOGITECH_DUAL_ACTION")) {
            return typeName + " MODE LIGHT MUST BE X";
        }
        return typeName;
    }
}
