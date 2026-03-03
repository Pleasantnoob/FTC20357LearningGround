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
 * Target is the turret's field angle (set to angle to goal = line robot→goal). When robot-relative angle exceeds ±wrapLimitDeg,
 * the encoder is reset via setCurrentPosition to the wrapped equivalent so the turret stays in [-180, 180] and wires don't tangle.
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

    /** Wrap angle to [0, 360) degrees. RR / field convention. Public for use by MainDrive. */
    public static double wrapDeg360(double a) {
        while (a >= 360) a -= 360;
        while (a < 0) a += 360;
        return a;
    }

    /** Wrap angle to [-180, 180) degrees (robot-centric). Use so turret takes shortest path. */
    public static double wrapDeg180(double a) {
        while (a > 180) a -= 360;
        while (a < -180) a += 360;
        return a;
    }

    /** Ticks per degree of turret output rotation. */
    private static final double TICKS_PER_DEG = (TICKS_PER_MOTOR_REV / 360.0) * INV_GEAR_RATIO;

    /** Robot-relative angle limits (deg). When exceeded, encoder is wrapped via setCurrentPosition to prevent wire tangling. */
    public static double wrapLimitDeg = 180.0;

    /** Nudge sensitivity: degrees per unit of left_stick_x. Adjust target when stick pushed to correct drift/gear skip. */
    public static double nudgeDegPerUnit = 4.0;

    public Turret(HardwareMap hardwareMap, Gamepad gamepad) {
        turretMotor = hardwareMap.get(DcMotorEx.class, "turret");
        turretMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        gamepad2 = gamepad;
        controller = new PIDController(p1, i1, d1);
        controller.setPID(p1, i1, d1);

        // Pinpoint: use shared driver for IMU heading only. Do not configure or reset — PinpointLocalizer does that once (avoids heading drift).
        driver = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        initialParDirection = GoBildaPinpointDriver.EncoderDirection.FORWARD;
        initialPerpDirection = GoBildaPinpointDriver.EncoderDirection.FORWARD;
    }

    /**
     * Runs turret control using Pinpoint IMU for robot heading (field-relative angle).
     * Call this every teleop loop when not in lock-on mode.
     */
    public void runTurretGyro() {
        // botHeading is set by MainDrive from pose.heading (odometry)
        double ticks = turretMotor.getCurrentPosition();
        double rawOutputDeg = (ticks / TICKS_PER_MOTOR_REV) * 360.0 * GEAR_RATIO;
        motorPosition = ticks;

        // Wrap-around: use wrapped position for PID so turret stays in [-180, 180] and avoids wire tangling.
        // FTC DcMotorEx has no setCurrentPosition; we use virtual wrapped ticks for the controller.
        outputDeg = wrapDeg180(rawOutputDeg);
        double wrappedTicks = outputDeg * TICKS_PER_DEG;
        // Field angle: botHeading - robot-relative (use raw for consistent field angle)
        currentAngle = wrapDeg360(botHeading - rawOutputDeg);

        // Robot-relative target in [-180, 180]; pick nearest equivalent for shortest path
        targetOutputDeg = wrapDeg180(botHeading - targetAngle);
        double revs = (outputDeg - targetOutputDeg) / 360.0;
        double k = Math.round(revs);
        targetPos = (targetOutputDeg + 360.0 * k) * TICKS_PER_DEG;

        SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(kS, kV, kA);
        controller.setPID(p1, i1, d1);
        double pid2 = controller.calculate(wrappedTicks, targetPos);
        double ff = feedforward.calculate(0);
        power = pid2 + ff;
        turretMotor.setPower(turretDriveEnabled ? power : 0);

        // Gamepad: left stick X nudge — adjust target to correct drift/gear skip (MainDrive must not overwrite when nudging)
        if (Math.abs(gamepad2.left_stick_x) > 0.1) {
            targetAngle = wrapDeg360(targetAngle - gamepad2.left_stick_x * nudgeDegPerUnit);
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
        double rawOutputDeg = (ticks / TICKS_PER_MOTOR_REV) * 360.0 * GEAR_RATIO;
        motorPosition = ticks;

        // Wrap-around: use wrapped position for PID (same as runTurretGyro)
        outputDeg = wrapDeg180(rawOutputDeg);
        double wrappedTicks = outputDeg * TICKS_PER_DEG;
        currentAngle = wrapDeg360(rawOutputDeg + k);

        targetOutputDeg = wrapDeg180(targetAngle - k);
        double revs = (outputDeg - targetOutputDeg) / 360.0;
        double kRev = Math.round(revs);
        targetPos = (targetOutputDeg + 360.0 * kRev) * TICKS_PER_DEG;

        SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(kS, kV, kA);
        controller.setPID(p1, i1, d1);
        double pid2 = controller.calculate(wrappedTicks, targetPos);
        double ff = feedforward.calculate(0);
        power = pid2 + ff;
        turretMotor.setPower(turretDriveEnabled ? power : 0);

        if (Math.abs(gamepad2.left_stick_x) > 0.1) {
            targetAngle = wrapDeg360(targetAngle - gamepad2.left_stick_x * nudgeDegPerUnit);
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
        // Keep full precision; callers expect this to be continuous (e.g. lock-on and fine aiming).
        targetAngle = wrapDeg360(target);
    }

    /** Returns current turret angle in field frame (degrees), [0, 360). */
    public double getTurretAngle() {
        return currentAngle;
    }

    /** Same as getTurretAngle(); use when you explicitly need field-relative angle. */
    public double getTurretAngleField() {
        return currentAngle;
    }

    /** Returns current turret angle relative to robot (degrees), [-180, 180). Uses encoder only; wrapped for shortest path. */
    public double getTurretAngleRobot() {
        double ticks = turretMotor.getCurrentPosition();
        double robotDeg = (ticks / TICKS_PER_MOTOR_REV) * 360.0 * GEAR_RATIO;
        return wrapDeg180(robotDeg);
    }

    /** Direct power control (no PID). Use for manual testing only. */
    public void turretPow(double calc) {
        turretMotor.setPower(calc);
    }
}
