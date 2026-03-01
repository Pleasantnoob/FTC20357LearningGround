package org.firstinspires.ftc.teamcode.Toros.Drive.Subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.controller.wpilibcontroller.SimpleMotorFeedforward;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.RR.MecanumDrive;
import org.firstinspires.ftc.teamcode.RR.PinpointLocalizer;
import org.firstinspires.ftc.teamcode.Toros.Drive.MainDrive;
import org.firstinspires.ftc.teamcode.Toros.Drive.ShotPhysics;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.opencv.core.Mat;

import com.arcrobotics.ftclib.util.LUT;
import com.sun.tools.javac.Main;

@Config
public class IntakeV2 {
    private DcMotorEx intakeMotor;
    public DcMotorEx launch;
    private Servo hood;
    private DcMotorEx trans;
    public ColorSensor c1, c2, c3;
    private AprilTagProcessor aprilTag;


    Gamepad gamepad1;
    private PIDController controller;

    /** Flywheel PID — tune in FTC Dashboard (IntakeV2) while running MainDrive. */
    public static double p1 = 0.006, i1 = 0.005, d1 = 0.0001;
    /** Flywheel feedforward — tune in FTC Dashboard (IntakeV2). kS=static, kV=velocity, kA=accel gain (often 0). */
    public static double kS = 0, kV = 0.00047, kA = 0.005;
    /** Target acceleration (ticks/s²) passed to feedforward. FF output uses kS + kV*vel + kA*accel. */
    public static double accel = -30;

    public static double f1 = 0;  // unused; FF uses kS, kV, kA
    /** Effective target (used by PID). Set from manualTargetVel when manualMode, or from auto/left_bumper. Not for Dashboard—edit manualTargetVel. */
    private static double targetVel = -1800;
    private Gamepad gamepad2;
    /** Grace band (ticks/s): can shoot when |launcherVel - targetVel| <= SHOOT_VEL_TOLERANCE (e.g. ±20). */
    public static int SHOOT_VEL_TOLERANCE = 20;

    public double ticksPerSecond = 1500;


   public static double vel;

   Pinpoint pinpoint;

    public IntakeV2(HardwareMap hardwareMap, Gamepad gamepad, Gamepad gamepadA, AprilTagProcessor aprilTag) {
        gamepad1 = gamepad;
        gamepad2 = gamepadA;
        intakeMotor = hardwareMap.get(DcMotorEx.class, " intake");
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        hood = hardwareMap.get(Servo.class,("hood"));
        hood.setDirection(Servo.Direction.FORWARD);
        launch = hardwareMap.get(DcMotorEx.class, ("launch"));
        launch.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        launch.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        trans = hardwareMap.get(DcMotorEx.class, ("trans"));
        trans.setDirection(DcMotorSimple.Direction.FORWARD);
        controller = new PIDController(p1, i1, d1);
        controller.setPID(p1, i1, d1);
        c1 = hardwareMap.get(ColorSensor.class,"c1");
        c2 = hardwareMap.get(ColorSensor.class,"c2");
        c3 = hardwareMap.get(ColorSensor.class,"c3");
        this.aprilTag = aprilTag;


        pinpoint = new Pinpoint(hardwareMap);
    }
    double hoodAngle = 0;
    public static double heading;

    public void runLauncher() {
        vel = Math.sqrt(Math.pow(pinpoint.getVelx(),2)+ Math.pow(pinpoint.getVelY(),2));
        heading = pinpoint.getHeading();
                //targetVel = -1* calcLaunch(0);
        //double ff = Math.cos(Math.toRadians(targetVel /ticks_in_degrees)) * f1;
            if (gamepad2.left_bumper) {
                targetVel = -1480;
                hood.setPosition(0.8);
                lastHoodValue = -1; // so next odometry update writes
            } else if (manualMode) {
                // Dashboard-tunable: set manualHoodAngleDeg and manualTargetVel (toggle manualMode in Config to use these)
                double hoodValue = minServo + ((70.0 - manualHoodAngleDeg) / 30.0) * (maxServo - minServo);
                hood.setPosition(hoodValue);
                lastHoodValue = -1; // so next odometry update writes when switching back
                targetVel = (int) manualTargetVel;
            } else {
                updateHoodAndFlywheelFromOdometry();
            }
        if (gamepad1.b) {
            intakeMotor.setPower(0);
            trans.setPower(0);
        }

        if (gamepad1.dpadUpWasPressed()) {
            targetVel -= 50;
        } else if (gamepad1.dpadDownWasPressed()) {
            targetVel += 50;
        }


        //launch normal — PID(F) tunable live via FTC Dashboard (IntakeV2 section when running MainDrive)
            if (gamepad2.left_trigger > 0.1) {
                controller.setPID(p1, i1, d1); // apply Dashboard tuning each loop
                double launchVel = launch.getVelocity();
                SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(kS, kV, kA);
                double pid = controller.calculate(launchVel, targetVel);
                double ff = feedforward.calculate(targetVel, accel);
                double power = pid + ff;
                launch.setPower(power);
            }
            if (gamepad2.left_trigger < 0.1) {
                launch.setPower(0);
            }

            if (gamepad2.right_trigger > 0.1) {
                if (Math.abs(launch.getVelocity() - targetVel) <= SHOOT_VEL_TOLERANCE) {
                    // Feed to launcher: both transfer and intake at full power (direction for feed)
                    trans.setPower(-1.0);
                    intakeMotor.setPower(-1.0);
                } else {
                    trans.setPower(0);
                    intakeMotor.setPower(0);
                }
            } else {
                trans.setPower(0);
                intakeMotor.setPower(0);
            }

    }



    public void runIntake() {
        //Moves ball into robot
        if (gamepad1.right_trigger > 0.25) {
            intakeMotor.setPower(-gamepad1.right_trigger);
        }

        //Moves ball out of robot
        if (gamepad1.left_trigger > 0.25) {
            intakeMotor.setPower(gamepad1.left_trigger);
        }
        if (gamepad1.left_trigger < 0.25 && gamepad1.right_trigger < 0.25 && gamepad2.right_trigger <0.25) {// turns off the motor if both triggers are not pressed
            intakeMotor.setPower(0);
        }


        //hardstop for all systems
        if (gamepad1.b) {
            intakeMotor.setPower(0);
            trans.setPower(0);
        }

        if(c3.blue() > 150 && c2.blue() > 150 && c1.blue() > 150){
            gamepad1.rumble(1500);
        }

        // Hood is set only in runLauncher() (manual or auto from distance). Do not overwrite here or it jitters.
        }

    public void transfer() {
        if (gamepad2.right_trigger > 0.1) {
            return; // Launcher feed owns trans in runLauncher(); don't overwrite
        }
        if (gamepad1.right_bumper && c3.blue() > 150) {
            trans.setPower(0.15);   // push up (was -0.15)
        } else if (gamepad1.right_bumper) {
            trans.setPower(-0.35);  // push up (was 0.35, reversed)
        } else {
            trans.setPower(0);
        }
        if (gamepad1.left_bumper) {
            trans.setPower(-0.35);
        }
    }
    public double getLauncherSpeed() {
        return launch.getVelocity();
    }
    public double getTargetVel() {
        return targetVel;
    }


    public static double k = -2.6;

    public double lastDistance = 1;
    public static double h = 0.69;
    public static double flywheelRadius = 0.048;
    /** Encoder ticks per flywheel revolution (for ticks/s → launch speed m/s). Tune if velocity comp is off in manual mode. */
    public static double TICKS_PER_REV = 28.0;
    public static double minAngle = 40;
    public static double maxAngle = 70;
    public static double minServo = 0.0;
    public static double maxServo = 1.0;

    // ---------- Dashboard launcher tuning (IntakeV2). Toggle manualMode and tune manualHoodAngleDeg / manualTargetVel. ----------
    /** true = use manual hood + velocity below; false = use regression from distance. Toggle in FTC Dashboard Config (IntakeV2). */
    public static boolean manualMode = false;
    /** Hood angle in degrees. 70 = steep (close), 40 = flat (far). Edit this on Dashboard to change hood. */
    public static double manualHoodAngleDeg = 55.0;
    /** Flywheel target velocity (encoder ticks/s). Negative = launch direction. Edit this on Dashboard to change shot speed; this is the value the PID uses. */
    public static double manualTargetVel = -1600.0;

    /** Convert flywheel ticks/s (magnitude) to approximate launch speed in m/s. Used for velocity-comp time-of-flight in manual mode. */
    public static double launchSpeedMPSFromTicksPerSec(double ticksPerSec) {
        return Math.abs(ticksPerSec) * 2 * Math.PI * flywheelRadius / TICKS_PER_REV;
    }

    LUT<Double, Double> speeds = new LUT<Double, Double>()
    {{
        add(0.0, 900.0);
        add(0.8, 1000.0);
        add(1.4, 1180.0);
        add(3.1, 1450.0);
    }};




    public double calcShot(double robotHeading){
        double g = 32.174 * 12;
        double x =  MainDrive.getDistance(); //distance - shoot past point radius
        double y = 26;
        double a = Math.toRadians(-45);

        hoodAngle = Math.toDegrees(Math.atan(2 * y/x - Math.tan(a))); ///clamp / round
        int flywheelSpeed = (int) Math.sqrt(g * x * x / (2* Math.pow(Math.cos(hoodAngle),2) * (x * Math.tan(hoodAngle)-y)));



        double robotVelocity = getVel();

        double coordinateTheta =  Math.atan(pinpoint.getVelY()/pinpoint.getVelY()) - Math.atan(MainDrive.getDistanceY()/ MainDrive.getDistanceX());

        double parallel = -Math.cos(coordinateTheta) * Math.abs(robotVelocity);
        double perpendicular = Math.sin(coordinateTheta) * Math.abs(robotVelocity);


        return  hoodAngle;
    }

    private void setHood(double hoodDegrees){
        hood.setPosition(hoodDegrees /300*5);
    }
    public double getHood(){
        return hood.getPosition();
    }




    /** Returns current robot heading in radians (from Pinpoint). Updated each loop in runLauncher(). */
    public static double getHeading(){
        return heading;
    }

    /** Last hood servo position written; only update when change exceeds this to reduce jitter. */
    private double lastHoodValue = -1;
    private static final double HOOD_DEADBAND = 0.015;

    // Regression from distance (inches) to hood angle and flywheel speed
    // Hood: y = 157.89707 * x^-0.276756  (degrees)
    // Flywheel: 853.95141 * 1.00455^x (encoder ticks/s magnitude; launch direction = negative)
    private static final double HOOD_COEF = 157.89707;
    private static final double HOOD_EXP = -0.276756;
    private static final double FLYWHEEL_COEF = 853.95141;
    private static final double FLYWHEEL_BASE = 1.00455;
    private static final double MIN_HOOD_DEG = 40.0;
    private static final double MAX_HOOD_DEG = 70.0;

    /**
     * Sets hood angle and flywheel target from distance to goal using regression.
     * Distance from odometry only. Hood: 157.89707 * distance^-0.276756 (clamped 40–70°). Flywheel: -(853.95141 * 1.00455^distance) ticks/s.
     */
    public void updateHoodAndFlywheelFromOdometry() {
        double distanceInches = MainDrive.getDistanceFromOdometry();
        distanceInches = Math.max(12.0, Math.min(180.0, distanceInches)); // clamp to valid range

        double hoodAngleDeg = HOOD_COEF * Math.pow(distanceInches, HOOD_EXP);
        hoodAngleDeg = Math.max(MIN_HOOD_DEG, Math.min(MAX_HOOD_DEG, hoodAngleDeg));

        double flywheelMagnitude = FLYWHEEL_COEF * Math.pow(FLYWHEEL_BASE, distanceInches);
        targetVel = (int) -flywheelMagnitude; // negative = launch direction

        double hoodValue = minServo + ((70.0 - hoodAngleDeg) / 30.0) * (maxServo - minServo);
        if (lastHoodValue < 0 || Math.abs(hoodValue - lastHoodValue) > HOOD_DEADBAND) {
            hood.setPosition(hoodValue);
            lastHoodValue = hoodValue;
        }
    }

    /** Unused: do not call. Kept only for reference. Hood is set by updateHoodAndFlywheelFromOdometry() or manual params. */
    public double calcLaunch1() {
        double distance = lastDistance;
        boolean tagSeen = false;
        double hoodAngleDeg = 60;
        for (AprilTagDetection d : aprilTag.getDetections()) {
            if (d.metadata != null && (d.id == 24 || d.id == 20)) {
                distance = d.ftcPose.range * 0.0254;
                tagSeen = true;
                break;
            }
        }
        if (tagSeen) lastDistance = distance;
        distance = Math.max(0.1, Math.min(4, distance));
        hoodAngleDeg = 60 + (distance - 0.6) * (40 - 60) / (1 - 0.5);
        hoodAngleDeg = Math.max(40, Math.min(60, hoodAngleDeg));
        double hoodValue = minServo + ((60 - hoodAngleDeg) / 20) * (maxServo - minServo);
        targetVel = -(speeds.getClosest(distance));
        return targetVel;
    }




    /** Unused: do not call. Hood is set by updateHoodAndFlywheelFromOdometry() or manual params only. */
    public double calcLaunch2() {

        //vars
        double distance = lastDistance;
        boolean tagSeen = false;
        double hoodAngleDeg = 60;
        // Get distance
        for (AprilTagDetection d : aprilTag.getDetections()) {
            if (d.metadata != null && (d.id == 24 || d.id == 20)) {
                distance = d.ftcPose.range * 0.0254;
                tagSeen = true;
                break;
            }
        }
        if (tagSeen){
            lastDistance = distance;
        }
        // Define distance
        distance = Math.max(0.1, Math.min(4, distance));
        //get hood angle (degrees)
        hoodAngleDeg = 60 + (distance - 0.6) * (40 - 60) / (1 - 0.5);
        // Define hood angle
        hoodAngleDeg = Math.max(40, Math.min(60, hoodAngleDeg));
        double hoodValue = minServo + ((60-hoodAngleDeg) / 20) * (maxServo - minServo);
        hood.setPosition(hoodValue);
        // Convert to rad
        double theta = Math.toRadians(hoodAngleDeg);
        // sqrt not 0 (very annoying)
        double denom = 2*Math.pow(Math.cos(theta), 2) * (distance *Math.tan(theta) - h);
        if (denom <= 0) return targetVel;
        // the actual calculation
        double v = distance * Math.sqrt(9.81 /denom);
        // Linear to angular to ticks/sec
        double omega = v / flywheelRadius;
        ticksPerSecond = omega * 28 / (2 * Math.PI);
        //tuning
        ticksPerSecond *= k;
        targetVel = (int) ticksPerSecond;
        return (int) ticksPerSecond;
    }
public static double getVel(){
    return vel;
}


}

//:3