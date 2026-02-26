package org.firstinspires.ftc.teamcode.Toros.Drive.Subsystems;

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
 * Important: We always take the SHORTEST path. (targetAngle - botHeading) is wrapped to [-180, 180] and
 * we command targetPos = currentTicks + (wrapped error in degrees)*ticksPerDeg so the turret never rotates
 * more than 180° and correctly counter-rotates when the robot turns (stays on goal).
 */
public class Turret {
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

    /** Wrap angle to [-180, 180] degrees. */
    private static double wrapDeg180(double a) {
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
        // botHeading is set by MainDrive from pose.heading (odometry) so turret and angleToGoal use same frame

        double ticks = turretMotor.getCurrentPosition();
        double outputDeg = (ticks / TICKS_PER_MOTOR_REV) * 360.0 * GEAR_RATIO;
        currentAngle = wrapDeg180(outputDeg) + botHeading;
        currentAngle = wrapDeg180(currentAngle);
        motorPosition = ticks;

        // Shortest path: target output wrapped to [-180,180], error wrapped, then targetPos = current + error (ticks)
        double targetOutputDeg = wrapDeg180(targetAngle - botHeading);
        double currentOutputWrapped = wrapDeg180(outputDeg);
        double errorDeg = wrapDeg180(targetOutputDeg - currentOutputWrapped);
        targetPos = ticks + errorDeg * TICKS_PER_DEG;

        SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(kS, kV, kA);
        controller.setPID(p1, i1, d1);
        double pid2 = controller.calculate(ticks, targetPos);
        double ff = feedforward.calculate(0);
        power = pid2 + ff;
        turretMotor.setPower(power);

        // Wrap target angle to avoid unbounded growth when turret is driven past ±150°
        if (Math.abs(targetAngle) > 150) {
            targetAngle = -targetAngle + Math.copySign(10, targetAngle);
        }

        // Gamepad: left stick X adjusts target angle
        if (Math.abs(gamepad2.left_stick_x) > 0.1) {
            targetAngle += gamepad2.left_stick_x * 4;
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
        double outputDeg = (ticks / TICKS_PER_MOTOR_REV) * 360.0 * GEAR_RATIO;
        currentAngle = wrapDeg180(outputDeg) + k;
        currentAngle = wrapDeg180(currentAngle);
        motorPosition = ticks;

        double targetOutputDeg = wrapDeg180(targetAngle - k);
        double currentOutputWrapped = wrapDeg180(outputDeg);
        double errorDeg = wrapDeg180(targetOutputDeg - currentOutputWrapped);
        targetPos = ticks + errorDeg * TICKS_PER_DEG;

        SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(kS, kV, kA);
        controller.setPID(p1, i1, d1);
        double pid2 = controller.calculate(ticks, targetPos);
        double ff = feedforward.calculate(0);
        power = pid2 + ff;
        turretMotor.setPower(power);

        if (Math.abs(targetAngle) > 80) {
            targetAngle = -targetAngle + Math.copySign(10, targetAngle);
        }

        if (Math.abs(gamepad2.left_stick_x) > 0.1) {
            targetAngle += gamepad2.left_stick_x * 4;
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

    /** Returns current turret angle in field frame (degrees). */
    public double getTurretAngle() {
        return currentAngle;
    }

    /** Direct power control (no PID). Use for manual testing only. */
    public void turretPow(double calc) {
        turretMotor.setPower(calc);
    }
}
