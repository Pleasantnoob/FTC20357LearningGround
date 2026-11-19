package teamcode.OpModes.TeleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import teamcode.ComplexMath.Storage.PoseStorage;
import teamcode.RobotContainer.DriveProcesses;
import teamcode.RobotContainer.Robot;
import teamcode.Utility.DashboardField;
import teamcode.pedro.Constants;


@Config
@TeleOp
public class MainDrive extends LinearOpMode {

    // Gamepad 2 Back/Share resets the robot's position to this spot. Put the robot there first.
    // PLACEHOLDERS: the Near Red / Near Blue auto start poses (inches, degrees).
    public static double RED_RESET_X = 133, RED_RESET_Y = 108, RED_RESET_HEADING_DEG = 180;
    public static double BLUE_RESET_X = 15, BLUE_RESET_Y = 135, BLUE_RESET_HEADING_DEG = 270;

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

            // No auto ran (or the robot controller restarted): default to RED. Gamepad 2 X/Y changes it.
            if (poseStorage.getAlliance() == null || poseStorage.getAlliance() == PoseStorage.Alliance.NONE) {
                poseStorage.setAlliance(PoseStorage.Alliance.RED);
            }
            boolean lastBack = false;

            // Driver Station gets plain telemetry. The dashboard gets one packet per loop with
            // graphs + field, so the field view doesn't flicker.
            FtcDashboard dashboard = FtcDashboard.getInstance();

            waitForStart();
            ElapsedTime loopTimer = new ElapsedTime();
            while (opModeIsActive()) {
                double dt = loopTimer.seconds();
                loopTimer.reset();

                double vertical = gamepad1.left_stick_y;
                double turn = gamepad1.right_stick_x;
                double strafe = gamepad1.left_stick_x;

                if (gamepad2.back && !lastBack) {
                    boolean blue = poseStorage.getAlliance() == PoseStorage.Alliance.BLUE;
                    driveFollower.setPose(blue
                            ? new Pose(BLUE_RESET_X, BLUE_RESET_Y, Math.toRadians(BLUE_RESET_HEADING_DEG))
                            : new Pose(RED_RESET_X, RED_RESET_Y, Math.toRadians(RED_RESET_HEADING_DEG)));
                }
                lastBack = gamepad2.back;

                driveFollower.update();
                Pose pose = driveFollower.pose();
                poseStorage.savePoseToStorage(pose.x(), pose.y(), pose.heading());

                driveProcesses.update(pose.x(), pose.y(), pose.heading(), dt);
                robot.drivetrain.updateDriveMotorPowers(vertical, turn, strafe);

                TelemetryPacket packet = new TelemetryPacket();
                DashboardField.draw(packet.fieldOverlay(), pose.x(), pose.y(), pose.heading(), robot.turret);
                packet.put("rpm", robot.shooter.getRPM());
                packet.put("target rpm", robot.shooter.getTargetRPM());
                packet.put("flywheel power", robot.shooter.getLastPower());
                packet.put("distance to target", robot.turret.getDistanceToTarget());
                packet.put("hood pos", robot.turretHood.getTargetPos());
                packet.put("turret cmd deg", robot.turret.getTurretHeading());
                packet.put("turret actual deg", robot.turret.getActualDeg());
                packet.put("turret on target", robot.turret.isOnTarget());
                packet.put("x", pose.x());
                packet.put("y", pose.y());
                packet.put("heading deg", Math.toDegrees(pose.heading()));
                packet.put("loop ms", dt * 1000);
                dashboard.sendTelemetryPacket(packet);

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
                telemetry.addData("turret trim deg", robot.turret.getTrimDeg());
                telemetry.addData("target side", robot.turret.isOnHighSide() ? "HIGH y" : "LOW y");
                telemetry.addData("hood pos", robot.turretHood.getTargetPos());
                telemetry.addData("match loader", robot.matchLoader.isDeployed() ? "DEPLOYED" : "stowed");
                telemetry.addData("turret actual / on target", "%.1f / %b",
                        robot.turret.getActualDeg(), robot.turret.isOnTarget());
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
