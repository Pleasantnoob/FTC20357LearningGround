package org.firstinspires.ftc.teamcode.Toros.Drive;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;

@TeleOp(name = "TestDrive")

public class TestDrive extends LinearOpMode {

    private boolean Rtoggle;
    private boolean Xtoggle;
    private DcMotor FrontLeftMotor, BackLeftMotor, FrontRightMotor, BackRightMotor;
    private Gamepad currentGamepad1 = new Gamepad(), previousGamepad1 = new Gamepad();
    @Override
    public void runOpMode() throws InterruptedException {
//        controller = new PIDController(p, i, d);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        //Initializing Hardware in method down below
        initHardware();
        waitForStart();

        if (opModeIsActive()) {
            while (opModeIsActive()) {
                previousGamepad1.copy(currentGamepad1);
                currentGamepad1.copy(gamepad1);
                drive();
                initTelemetry();
                telemetry.update();

            }
        }
    }

    private void initHardware () {

        //Motors
        FrontLeftMotor = hardwareMap.get(DcMotor.class, "fl");
        BackLeftMotor = hardwareMap.get(DcMotor.class, "bl");
        FrontRightMotor = hardwareMap.get(DcMotor.class, "fr");
        BackRightMotor = hardwareMap.get(DcMotor.class, "br");
        FrontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        BackRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        //Zero Power Behaviors
        FrontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BackLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FrontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BackRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    }

    private void initTelemetry() {
        telemetry.addData("X Toggle", Xtoggle);
        telemetry.addData("R Toggle", Rtoggle);
        telemetry.update();
    }
    private void drive () throws InterruptedException {


        if(currentGamepad1.x && !previousGamepad1.x){
            Xtoggle = !Xtoggle;
        }
        if(currentGamepad1.b && !previousGamepad1.b){
            Rtoggle = !Rtoggle;
        }






        double x = gamepad1.left_stick_x;
        double y = -gamepad1.left_stick_y;
        double turn = gamepad1.right_stick_x;

        if(Xtoggle){
            x *= 0.75;
        }
        else{
            x*=1;
        }
        if(Rtoggle){
            turn *= 0.75;
        }
        else{
            turn*=1;
        }




        //Drive variables used in the calculations to run our motors
        double theta = Math.atan2(y, x);
        double power = Math.hypot(x, y);
        double sin = Math.sin(theta - Math.PI / 4);
        double cos = Math.cos(theta - Math.PI / 4);
        double max = Math.max(Math.abs(sin), Math.abs(cos));

        /**
         In basics this is taking the x and y of the left stick making them into an angle
         with the power being the hypot which is the square root of the sum of squares of the inputs
         more info here https://developer.mozilla.org/en-US/docs/Web/JavaScript/Reference/Global_Objects/Math/hypot
         then takes the sin and cos of the angle making sure to convert to radians. It then creates a max
         using the absolute value of the sin and cos.

         The idea is that where you are going is angle theta with each wheel being a vector and when combined make the target direction when rotated 45 degrees

         Found on YT www.youtube.com/watch?v=gnSW2QpkGXQ which is a video about coding for mecanum drive wheels
         */


        //Calculations for our drive motors

        double fl = (power * cos / max + turn);
        double fr = (power * sin / max - turn);
        double bl = (power * sin / max + turn);
        double br = (power * cos / max - turn);

        /**
         In continuation the power is then calculated with the angles multiplied by the sin or cos divided the difference or sum of the max and turn
         */

        //If statement below is to make sure one motor does not exceed the power limit making it scale down

        if ((power + Math.abs(turn)) > 1) {
            fl /= power + Math.abs(turn);
            fr /= power + Math.abs(turn);
            bl /= power + Math.abs(turn);
            br /= power + Math.abs(turn);
        }



        //Motor Drive
        FrontLeftMotor.setPower(fl);
        FrontRightMotor.setPower(fr);
        BackLeftMotor.setPower(bl);
        BackRightMotor.setPower(br);
    }
}


//:3