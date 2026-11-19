package teamcode.Tests;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import teamcode.RobotContainer.Robot;

/**
 * (Hidden) Drives with the TeleOp mix and shows each wheel's power, for checking strafing.
 */
@Disabled
@TeleOp
public class StrafeTest extends LinearOpMode {

    public Robot robot;

    @Override
    public void runOpMode(){
        robot = new Robot(hardwareMap);

        waitForStart();
        while(opModeIsActive()){
            double vertical = gamepad1.left_stick_y;
            double horizontal = gamepad1.right_stick_x;
            double strafe = gamepad1.left_stick_x;


            robot.drivetrain.updateDriveMotorPowers(vertical, horizontal, strafe);


            telemetry.addData("Vertical", vertical);
            telemetry.addData("Horizontal", horizontal);
            telemetry.addData("Strafe", strafe);

            telemetry.addData("LF Power", robot.drivetrain.getLFMotorPower());
            telemetry.addData("LB Power", robot.drivetrain.getLBMotorPower());
            telemetry.addData("RF Power", robot.drivetrain.getRFMotorPower());
            telemetry.addData("RB Power", robot.drivetrain.getRBMotorPower());


            telemetry.update();
        }


    }

















}
