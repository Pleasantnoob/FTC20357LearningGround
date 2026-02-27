package org.firstinspires.ftc.teamcode.Toros.Drive.Subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.controller.wpilibcontroller.SimpleMotorFeedforward;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.RR.PinpointLocalizer;

/**
 * Turret subsystem: rotates the launcher turret and tracks its angle using motor encoder odometry.
 *
 * Odometry: The turret motor has an encoder. We convert encoder ticks to turret angle (degrees) using:
 *   - TICKS_PER_MOTOR_REV: encoder ticks per one full rotation of the motor shaft (384.5 for the motor used).
 *   - GEAR_RATIO: output rotation / motor rotation (2/5 means turret rotates 2/5 of a full turn per motor revolution).
 *
 * Target is the turret's field angle (set to angle to goal = line robot→goal). No shortest-path wrap: targetPos = (targetAngle - botHeading) * ticksPerDeg.
 * At init: encoder 0 → turret output 0 → turret field angle = robot heading (turret same as robot). So robot at 90° → turret at 90° field, 0° relative to robot.
 * When on target, curAng (turret field angle) should match tgtAng (angle to goal).
 */
@Config
public class Turret {
    /** When false, turret motor gets 0 power (target angle still computed and logged). true = turret moves to hold field angle. */
    public static boolean turretDriveEnabled = true;

    // ========== Tuning constants (tune via FTC Dashboard or here). Do not change odometry constants below without verifying on robot. ==========
    public static double p1 = 0.00625, i1 = 0.00, d1 = 0.00055;
    public static double kS = 0, kV = 0.000125, kA = 0;

    /** Encoder ticks per one full motor revolution. Used for encoder ↔ angle conversion. */
    public static final double TICKS_PER_MOTOR_REV = 384.5;
    /** Turret output rotation per motor rotation (e.g. 2/5 = 0.4). Inverse is used for angle→ticks. */
    public static final double GEAR_RATIO = 2.0 / 5.0;
    private static final double INV_GEAR_RATIO = 5.0 / 2.0;

    private final DcMotorEx turretMotor;
    private final PIDController controller;
    /** Desired turret angle in field frame (degrees). Updated by gamepad or vision. */
    public double targetAngle = 0;
    /** Current encoder position (ticks); updated each loop for telemetry/debug. */
    public double motorPosition;
    private final Gamepad gamepad2;
    /** Last commanded motor power (for debugging). */
    public double power;
    /** Target position in encoder ticks (computed from targetAngle + heading). */
    public double targetPos;
    /** Robot heading from Pinpoint IMU (degrees). Used to convert field angle to motor ticks. */
    public double botHeading;
    /** When not using gyro (lock-on), this is the frozen heading used for angle conversion. */
    public double k = 0;
    /** True = use Pinpoint gyro for heading; false = use frozen k (lock-on mode). */
    public boolean gyro = true;
    public double ticksPerSecond = 1500;

    /** Pinpoint localizer params (encoder offsets); used when Turret constructs the Pinpoint driver for heading only. */
    public static class Params {
        public double parYTicks = 0.0;
        public double perpXTicks = 0.0;
    }

    public static PinpointLocalizer.Params PARAMS = new PinpointLocalizer.Params();

    /** GoBilda Pinpoint driver: provides IMU heading for field-relative turret angle. */
    public GoBildaPinpointDriver driver;
    public GoBildaPinpointDriver.EncoderDirection initialParDirection, initialPerpDirection;

    /** Current turret angle in field frame (degrees). Same as getTurretAngle(). */
    double currentAngle;
    /** Turret output angle (degrees, robot-relative). For logging. */
    public double outputDeg;
    /** Target turret output angle (degrees). For logging. */
    public double targetOutputDeg;

    /** Wrap angle to [-180, 180] degrees. Public for use by MainDrive (e.g. field-hold init). */
    public static double wrapDeg180(double a) {
        while (a > 180) a -= 360;
        while (a < -180) a += 360;
        return a;
    }

    /** Ticks per degree of turret output rotation. */
    private static final double TICKS_PER_DEG = (TICKS_PER_MOTOR_REV / 360.0) * INV_GEAR_RATIO;

    public Turret(HardwareMap hardwareMap, Gamepad gamepad) {
        turretMotor = hardwareMap.get(DcMotorEx.class, "turret");
        turretMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        gamepad2 = gamepad;
        controller = new PIDController(p1, i1, d1);
        controller.setPID(p1, i1, d1);

        // Pinpoint is used here only for IMU heading (degrees). Encoder resolution/offsets must match your odometry pod setup.
        driver = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        double inPerTick = 0.02285196738095238;
        double mmPerTick = inPerTick * 25.4;
        driver.setEncoderResolution(1 / mmPerTick, DistanceUnit.MM);
        driver.setOffsets(PARAMS.parYTicks, PARAMS.perpXTicks, DistanceUnit.MM);
        initialParDirection = GoBildaPinpointDriver.EncoderDirection.FORWARD;
        initialPerpDirection = GoBildaPinpointDriver.EncoderDirection.FORWARD;
        driver.setEncoderDirections(initialParDirection, initialPerpDirection);
        driver.resetPosAndIMU();
    }

    /**
     * Runs turret control using Pinpoint IMU for robot heading (field-relative angle).
     * Call this every teleop loop when not in lock-on mode.
     */
    public void runTurretGyro() {
        // botHeading is set by MainDrive from pose.heading (odometry)
        double ticks = turretMotor.getCurrentPosition();
        outputDeg = (ticks / TICKS_PER_MOTOR_REV) * 360.0 * GEAR_RATIO;
        motorPosition = ticks;
        // Field angle: we command robot angle = (botHeading - targetAngle), so field = botHeading - outputDeg to read targetAngle when on target
        currentAngle = wrapDeg180(botHeading - outputDeg);

        // Robot-relative target limited to [-180, 180]
        targetOutputDeg = wrapDeg180(botHeading - targetAngle);
        // Target position = nearest tick position that equals targetOutputDeg (mod 360) so turret holds
        double revs = (ticks / TICKS_PER_DEG - targetOutputDeg) / 360.0;
        double k = Math.round(revs);
        targetPos = (targetOutputDeg + 360.0 * k) * TICKS_PER_DEG;

        SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(kS, kV, kA);
        controller.setPID(p1, i1, d1);
        double pid2 = controller.calculate(ticks, targetPos);
        double ff = feedforward.calculate(0);
        power = pid2 + ff;
        turretMotor.setPower(turretDriveEnabled ? power : 0);

        // Gamepad: left stick X nudge (reversed so left = turret left); wrap to [-180, 180]
        if (Math.abs(gamepad2.left_stick_x) > 0.1) {
            targetAngle = wrapDeg180(targetAngle - gamepad2.left_stick_x * 4);
        }
        if (gamepad2.aWasPressed()) {
            targetAngle = 0;
        }
        // X: reset encoder and set target to current field angle so turret holds position
        if (gamepad2.xWasPressed()) {
            turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            turretMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            targetAngle = currentAngle;
        }
    }

    /**
     * Runs turret control using a fixed heading value k (e.g. when lock-on is active and we freeze robot heading).
     * Call this every teleop loop when in lock-on mode.
     */
    public void runTurretNoGyro(double k) {
        double ticks = turretMotor.getCurrentPosition();
        outputDeg = (ticks / TICKS_PER_MOTOR_REV) * 360.0 * GEAR_RATIO;
        currentAngle = wrapDeg180(outputDeg) + k;
        currentAngle = wrapDeg180(currentAngle);
        motorPosition = ticks;

        targetOutputDeg = targetAngle - k;
        targetPos = targetOutputDeg * TICKS_PER_DEG;

        SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(kS, kV, kA);
        controller.setPID(p1, i1, d1);
        double pid2 = controller.calculate(ticks, targetPos);
        double ff = feedforward.calculate(0);
        power = pid2 + ff;
        turretMotor.setPower(turretDriveEnabled ? power : 0);

        if (Math.abs(gamepad2.left_stick_x) > 0.1) {
            targetAngle = wrapDeg180(targetAngle - gamepad2.left_stick_x * 4);
        }
        if (gamepad2.aWasPressed()) {
            targetAngle = 0;
        }
        if (gamepad2.xWasPressed()) {
            turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            turretMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    /** Sets the target turret angle in field frame (degrees). */
    public void setAngle(double target) {
        targetAngle = (int) target;
    }

    /** Returns current turret angle in field frame (degrees), [-180, 180]. For field-relative hold and wrap-around. */
    public double getTurretAngle() {
        return currentAngle;
    }

    /** Same as getTurretAngle(); use when you explicitly need field-relative angle. */
    public double getTurretAngleField() {
        return currentAngle;
    }

    /**
     * Returns current turret angle relative to robot (degrees), clamped to [-180, 180].
     * Uses current encoder position only (no heading).
     */
    public double getTurretAngleRobot() {
        double ticks = turretMotor.getCurrentPosition();
        double robotDeg = (ticks / TICKS_PER_MOTOR_REV) * 360.0 * GEAR_RATIO;
        return Math.max(-180, Math.min(180, wrapDeg180(robotDeg)));
    }

    /** Direct power control (no PID). Use for manual testing only. */
    public void turretPow(double calc) {
        turretMotor.setPower(calc);
    }
}
