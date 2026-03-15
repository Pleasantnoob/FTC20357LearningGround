package org.firstinspires.ftc.teamcode.Toros.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous(name = "AutoV1")
@Disabled
// Autonomous for first meet of 2024/25 (drive + wrist/fingers/elbow only; pivot/slides removed)
public class AutoV1 extends LinearOpMode {
    private DcMotor FrontLeftMotor, BackLeftMotor, FrontRightMotor, BackRightMotor;
    private Servo fingers, wrist, elbow;

    @Override
    public void runOpMode() throws InterruptedException {
        initHardware();
        waitForStart();
        runMotors(1, -0.75, -0.75, 1, 1200);
    }
    private void initHardware(){
        //Motors
        FrontLeftMotor = hardwareMap.get(DcMotor.class, "fl");
        BackLeftMotor = hardwareMap.get(DcMotor.class, "bl");
        FrontRightMotor = hardwareMap.get(DcMotor.class, "fr");
        BackRightMotor = hardwareMap.get(DcMotor.class, "br");
        fingers = hardwareMap.get(Servo.class, "fingers");
        wrist = hardwareMap.get(Servo.class, "wrist");
        elbow = hardwareMap.get(Servo.class, "elbow");
        FrontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        BackRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        FrontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BackLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FrontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BackRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fingers.setPosition(1);
        elbow.setPosition(0);
        wrist.setPosition(0);
    }
    private void runMotors(double p1, double p2, double p3, double p4, int sleepTime){
        FrontLeftMotor.setPower(p1);
        FrontRightMotor.setPower(p2);
        BackLeftMotor.setPower(p3);
        BackRightMotor.setPower(p4);
        sleep(sleepTime);
    }
}
