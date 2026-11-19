package teamcode.CustomTuning;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import teamcode.Constants.GameConstants;
import teamcode.Subsystems.Turret;
import teamcode.pedro.Constants;

/**
 * (Hidden) Older turret test: pick a start pose with gamepad 1 X/B/Y/A during init, then
 * the dpad picks which cell to aim at. TuningStation covers this now.
 */
@Disabled
@Config
@TeleOp
public class TurretTuning extends LinearOpMode {

    public void runOpMode() {
        Turret turret = new Turret(hardwareMap);
        Follower follower = Constants.create(hardwareMap);
        PoseFactory poseFactory = PoseFactory.degrees();
        GameConstants gameConstants = new GameConstants();
        Pose startPos = null;

        double targetX = 72;
        double targetY = 72;

        while (!isStarted() && !isStopRequested() && startPos == null) {
            if (gamepad1.x) {
                startPos = poseFactory.of(32, 9, 90);
            } else if (gamepad1.b) {
                startPos = poseFactory.of(32, 132, 270);
            } else if (gamepad1.y) {
                startPos = poseFactory.of(110, 133, 270);
            } else if (gamepad1.a) {
                startPos = poseFactory.of(129, 8, 90);
            }
            idle();
        }

        if (startPos == null) {
            startPos = poseFactory.of(72, 72, 0);
        }

        follower.setPose(startPos);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        waitForStart();

        try {
        while (opModeIsActive()) {
            follower.update();

            if (gamepad1.dpad_up) {
                targetX = gameConstants.redGardenCellPos[0];
                targetY = gameConstants.redGardenCellPos[1];
            } else if (gamepad1.dpad_down) {
                targetX = gameConstants.redParkCellPos[0];
                targetY = gameConstants.redParkCellPos[1];
            } else if (gamepad1.dpad_left) {
                targetX = gameConstants.blueGardenCellPos[0];
                targetY = gameConstants.blueGardenCellPos[1];
            } else if (gamepad1.dpad_right) {
                targetX = gameConstants.blueParkCellPos[0];
                targetY = gameConstants.blueParkCellPos[1];
            }

            turret.setTarget(targetX, targetY);
            turret.track(follower.pose().x(), follower.pose().y(), follower.pose().heading());

            double robotHeadingDeg = Math.toDegrees(follower.pose().heading());
            telemetry.addData("robotX", follower.pose().x());
            telemetry.addData("robotY", follower.pose().y());
            telemetry.addData("robotHeadingDeg", robotHeadingDeg);
            telemetry.addData("targetX", targetX);
            telemetry.addData("targetY", targetY);
            telemetry.addData("commandedDeg", turret.getTurretHeading());
            telemetry.addData("leftServo", turret.getLeftServoPosition());
            telemetry.addData("aimMode", turret.getAimMode());
            telemetry.update();
        }
        } finally {
            stopDrive();
        }
    }

    private void stopDrive() {
        hardwareMap.get(DcMotor.class, "lf_motor").setPower(0);
        hardwareMap.get(DcMotor.class, "lb_motor").setPower(0);
        hardwareMap.get(DcMotor.class, "rf_motor").setPower(0);
        hardwareMap.get(DcMotor.class, "rb_motor").setPower(0);
    }
}
