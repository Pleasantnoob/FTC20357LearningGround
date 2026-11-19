package teamcode.Tests;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import teamcode.ComplexMath.Storage.PoseStorage;
import teamcode.RobotContainer.Robot;
import teamcode.pedro.Constants;


/**
 * (Hidden) Turret auto-aim test from a fixed blue start pose (5, 138, 270). Drive around and watch it track.
 */
@Disabled
@TeleOp
public class PositionTurretTest extends LinearOpMode {

    @Override
    public void runOpMode() {
        Robot robot = new Robot(hardwareMap);
        try {
            Follower follower = Constants.create(hardwareMap);
            PoseStorage poseStorage = new PoseStorage();
            poseStorage.setAlliance(PoseStorage.Alliance.BLUE);
            follower.setPose(PoseFactory.degrees().of(5, 138, 270));

            waitForStart();
            ElapsedTime loopTimer = new ElapsedTime();
            while (opModeIsActive()) {
                double dt = loopTimer.seconds();
                loopTimer.reset();

                follower.update();
                Pose pose = follower.pose();
                poseStorage.savePoseToStorage(pose.x(), pose.y(), pose.heading());

                robot.turret.update(pose.x(), pose.y(), pose.heading(), gamepad2, dt);
                robot.drivetrain.updateDriveMotorPowers(
                        gamepad1.left_stick_y,
                        gamepad1.right_stick_x,
                        gamepad1.left_stick_x);

                telemetry.addData("Position", "%.1f, %.1f, %.1f deg",
                        pose.x(), pose.y(), Math.toDegrees(pose.heading()));
                telemetry.addData("alliance", poseStorage.getAlliance());
                telemetry.addData("targetPosX", robot.turret.getTurretTargetX());
                telemetry.addData("targetPosY", robot.turret.getTurretTargetY());
                telemetry.addData("turret heading", robot.turret.getTurretHeading());
                telemetry.addData("turret servo", robot.turret.getLeftServoPosition());
                telemetry.addData("turret mode", robot.turret.getAimMode());
                telemetry.update();
            }
        } finally {
            robot.stop();
        }
    }
}
