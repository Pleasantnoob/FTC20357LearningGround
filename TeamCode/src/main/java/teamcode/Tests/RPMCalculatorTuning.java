package teamcode.Tests;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import teamcode.Constants.GameConstants;
import teamcode.RobotContainer.DriveProcesses;
import teamcode.RobotContainer.Robot;
import teamcode.pedro.Constants;

/**
 * (Hidden) Old shot-data helper: shows distance to a cell while you set RPM/hood from the dashboard.
 * Start the robot at (82, 8, 90). TuningStation replaces this.
 */
@Disabled
@Config
@TeleOp
public class RPMCalculatorTuning extends LinearOpMode {
    Robot robot;
    DriveProcesses driveProcesses;
    Follower follower;
    GameConstants gameConstants;

    public static double hoodAngle = 0;
    public static int targetRPM = 0;

    double targetX = 0;
    double targetY = 0;

    private double calculateDistance(){

            double distance = Math.sqrt(Math.pow(targetX - follower.pose().x(), 2) + Math.pow(targetY - follower.pose().y(), 2));

            if(targetX == 0 & targetY == 0) {
                return 0;
            } else {
                return distance;
            }

    }


    public void runOpMode(){
        robot = new Robot(hardwareMap);
        gameConstants = new GameConstants();
        driveProcesses = new DriveProcesses(robot, gamepad1, gamepad2);

        //must always start on the red side of the flower cell\
        // I could make another while loop that changes the start Pose too
        // but I dont think that's necessary; Just start here and drive the robot
        // to different distances
        Pose constantStartPos = new Pose(82, 8, 90);
        follower = Constants.create(hardwareMap);
        follower.setPose(constantStartPos);

        double vertical, horizontal, strafe;



        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());


        waitForStart();
        ElapsedTime loopTimer = new ElapsedTime();
        try {
        while(opModeIsActive()){
            double dt = loopTimer.seconds();
            loopTimer.reset();
            follower.update();

            if(gamepad1.dpad_up){
                targetX = gameConstants.redGardenCellPos[0];
                targetY = gameConstants.redGardenCellPos[1];
            } else if (gamepad1.dpad_down){
                targetX = gameConstants.redParkCellPos[0];
                targetY = gameConstants.redParkCellPos[1];
            } else if (gamepad1.dpad_left){
                targetX = gameConstants.blueGardenCellPos[0];
                targetY = gameConstants.blueGardenCellPos[1];
            } else if (gamepad1.dpad_right){
                targetX = gameConstants.blueParkCellPos[0];
                targetY = gameConstants.blueParkCellPos[1];
            } else if (gamepad1.left_bumper){
                targetX = 72;
                targetY = 72;
            }

            vertical = gamepad1.left_stick_y;
            horizontal = gamepad1.right_stick_x;
            strafe = gamepad1.left_stick_x;

            robot.turretHood.setHoodAngle(hoodAngle);

            driveProcesses.update(follower.pose().x(), follower.pose().y(), follower.pose().heading(), dt);
            robot.shooter.setBaseTargetRPM(targetRPM);
            robot.shooter.calculateShooterPID();
            robot.drivetrain.updateDriveMotorPowers(vertical, horizontal, strafe);

            telemetry.addData("TargetRPM: ", targetRPM);
            telemetry.addData("ShooterRPM: ", robot.shooter.getRPM());
            //if this value returns 0, the targetX and targetY are not set
            telemetry.addData("distanceFromTarget: ", calculateDistance());
            telemetry.update();
        }
        } finally {
            robot.stop();
        }
    }



}
