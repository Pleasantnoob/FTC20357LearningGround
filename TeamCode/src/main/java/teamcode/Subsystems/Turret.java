package teamcode.Subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.Range;

import teamcode.ComplexMath.Storage.PoseStorage;

/**
 * Two Axon MAX servos driving the turret (merged V2-robot + Meet-2.1).
 *
 * Speed comes from:
 *  - Commanding the final angle in one step in AUTO (no slew), so the servo's own
 *    controller runs at full speed.
 *  - Leading the target: robot velocity and turn rate are estimated from the pose
 *    and the aim is computed for where the robot will be latencySec from now.
 *  - Aiming from the turret pivot, not the robot center.
 *  - No aim dead zone: every change in the aim is sent. Only exact repeat values
 *    are skipped, which saves hub traffic without losing accuracy.
 *  - isOnTarget() so the gate opens the moment the turret arrives, not after a
 *    fixed wait.
 *
 * Everything public static can be tuned live in FTC Dashboard.
 */
@Config
public class Turret {

    public enum AimMode { IDLE, AUTO, MANUAL, HOLD_ZERO }

    // ---------------- Hardware names (must match the Driver Station config) ----------------
    public static String LEFT_SERVO_NAME = "turretServoLeft";
    public static String RIGHT_SERVO_NAME = "turretServoRight";
    /** Optional Axon analog feedback wire into an analog port. If not configured, a timing model is used instead. */
    public static String ENCODER_NAME = "turretEncoder";

    // ---------------- Mechanics (MEASURE THESE) ----------------
    /** Servo degrees covered by setPosition 0 -> 1 with the PWM range below. */
    public static double servoRangeDeg = 355;
    /** Servo degrees per turret degree. 1:1 on V2 (both servo gears drive one center gear). */
    public static double gearRatio = 1.0;
    /** Max turret travel each side of center. 175 = 350 deg total with a 10 deg gap at the back. */
    public static double travelLimitDeg = 175;
    /** Servo position where the turret points straight forward. */
    public static double centerPos = 0.5;
    /**
     * V2: both servo gears mesh the same center gear, so both servos spin the SAME way -> false.
     * WRONG VALUE = THE SERVOS FIGHT. Check each servo alone first (TuningStation LEFT_ONLY / RIGHT_ONLY).
     */
    public static boolean reverseRight = false;
    /** Flip if positive degrees turn the turret clockwise (seen from above). Positive should be CCW / left. */
    public static boolean invertDirection = false;
    /** Axon accepts 500-2500us. Wider PWM = full travel. Recalibrate servoRangeDeg if changed. */
    public static boolean useWidePwm = true;
    public static double pwmLowUs = 500;
    public static double pwmHighUs = 2500;
    /**
     * Added to the RIGHT servo only, in servo units. Two servos on one gear never line up perfectly
     * (the spline teeth are coarse), so they push against each other a little. Tune this in
     * TuningStation (TURRET_MODE = BOTH_RAW) until the "hub current" graph is lowest and the servos
     * stop buzzing.
     */
    public static double rightOffsetPos = 0;

    // ---------------- Turret pivot in robot frame (inches, +forward, +left) ----------------
    public static double pivotForwardIn = 15.021 / 25.4; // CAD: 15.021 mm toward the front (0.591 in)
    public static double pivotLeftIn = 0;                // CAD: centered side to side

    // ---------------- Speed / prediction ----------------
    /** How far ahead to aim. Roughly loop time + servo reaction. */
    public static double latencySec = 0.06;
    /** Turret speed in deg/s, used by the on-target model when there is no encoder. */
    public static double turretDegPerSec = 500;
    /** Velocity filter, 0..1. Higher = reacts faster but noisier. */
    public static double velocityAlpha = 0.5;
    public static double onTargetTolDeg = 1.5; // only gates the shot; aim itself has no dead zone
    /** Write deadband in servo units. 0 = send every change (no aim dead zone); only exact repeats are skipped. */
    public static double writeDeadbandPos = 0;

    // ---------------- Optional analog feedback (Axon: 0-3.3V over 360 deg of servo) ----------------
    public static double encoderVoltsAtCenter = 1.65;
    public static double encoderServoDegPerVolt = 360.0 / 3.3;
    public static boolean encoderInverted = false;

    // ---------------- Targets (field inches). "High" = the half of the field with y above sideSwitchY ----------------
    public static double redHighX = 84.79, redHighY = 87.32;
    public static double redLowX = 83.84, redLowY = 50.27;
    public static double blueHighX = 58.05, blueHighY = 86.41;
    public static double blueLowX = 57.82, blueLowY = 53.41;
    public static double sideSwitchY = 72;
    /** The robot must cross this far past the line before the target switches, so it doesn't flicker on the line. */
    public static double sideSwitchHysteresisIn = 3;

    // ---------------- Trim (gamepad 2 fine adjust) ----------------
    public static double trimStepDeg = 1.0;

    // ---------------- Shoot on the move (virtual goal) ----------------
    /**
     * true = aim at a "virtual goal" shifted against the robot's motion, so the ball's
     * inherited velocity carries it into the real goal. Toggle on gamepad 2 right bumper.
     * Has no effect while the robot is standing still.
     */
    public static boolean SHOOT_ON_THE_MOVE = true;
    // PLACEHOLDERS below are rough starting guesses, not measured on this robot. The BioBuzz ball is
    // light, so expect a short flight time - measure before trusting them.
    /** Seconds from the gate opening until the ball leaves the flywheel. PLACEHOLDER - measure it. */
    public static double FEED_TIME_SEC = 0.15;
    /** Ball flight time (s) = TOF_A + TOF_B * distanceInches. PLACEHOLDER - fit it in Desmos. */
    public static double TOF_A = 0.35;
    public static double TOF_B = 0.0026;
    public static int SOTM_ITERATIONS = 5;
    /** true = robot velocity from the Pinpoint (follower.velocity()); false = worked out from pose changes. */
    public static boolean USE_PINPOINT_VELOCITY = true;

    // ---------------- Manual override ----------------
    public static double slewDegPerSec = 200;
    public static double stickDeadband = 0.08;

    private final Servo leftServo;
    private final Servo rightServo;
    private final AnalogInput encoder; // may be null
    private final PoseStorage poseStorage = new PoseStorage();

    private double turretTargetX = 72;
    private double turretTargetY = 72;
    private double commandedDeg = 0;
    private double estimatedDeg = 0;
    private boolean reachable = true;
    private double lastWrittenPos = Double.NaN;
    private boolean zeroHeld = false;
    private boolean onHighSide = false;
    private boolean sideKnown = false;
    private double trimDeg = 0;
    private double lastDistance = 0;
    private AimMode aimMode = AimMode.IDLE;
    // false = that servo's PWM is off, so it goes limp and nothing in here will write to it
    private boolean leftPowered = true, rightPowered = true;

    // velocity estimate
    private long lastNanos = 0;
    private double lastX, lastY, lastH;
    private boolean havePose = false;
    private double vx = 0, vy = 0, omega = 0;          // from pose differences
    private double measVx, measVy, measOmega;           // from the Pinpoint
    private boolean haveMeasured = false;
    private double virtualX = 72, virtualY = 72;

    public Turret(HardwareMap hardwareMap) {
        this(hardwareMap, true);
    }

    /**
     * @param holdCenter true = power both servos and hold center from init (matches, autos).
     *                   false = leave both servos limp until something commands them (TuningStation),
     *                   so an untuned turret doesn't fight itself the moment you hit init.
     */
    public Turret(HardwareMap hardwareMap, boolean holdCenter) {
        leftServo = hardwareMap.get(Servo.class, LEFT_SERVO_NAME);
        rightServo = hardwareMap.get(Servo.class, RIGHT_SERVO_NAME);
        encoder = hardwareMap.tryGet(AnalogInput.class, ENCODER_NAME);

        leftServo.setDirection(Servo.Direction.FORWARD);
        rightServo.setDirection(reverseRight ? Servo.Direction.REVERSE : Servo.Direction.FORWARD);

        if (useWidePwm) {
            PwmControl.PwmRange range = new PwmControl.PwmRange(pwmLowUs, pwmHighUs);
            if (leftServo instanceof ServoImplEx) ((ServoImplEx) leftServo).setPwmRange(range);
            if (rightServo instanceof ServoImplEx) ((ServoImplEx) rightServo).setPwmRange(range);
        }

        if (holdCenter) {
            // Hold center from init so the turret isn't limp. Both servos get the same command
            // in the same call, so they can't drift apart.
            commandDegrees(0);
        } else {
            setPowered(false, false);
        }
        estimatedDeg = 0;
    }

    /**
     * Turn each servo's signal on or off. An Axon with no signal goes limp, so with one servo off
     * the other can turn the turret without a fight. Writes to an unpowered servo are skipped,
     * because setPosition() would switch its signal back on.
     */
    public void setPowered(boolean left, boolean right) {
        if (left != leftPowered || right != rightPowered) lastWrittenPos = Double.NaN; // rewrite on next command
        leftPowered = left;
        rightPowered = right;
        setPwm(leftServo, left);
        setPwm(rightServo, right);
    }

    private static void setPwm(Servo servo, boolean on) {
        if (!(servo instanceof ServoImplEx)) return;
        if (on) ((ServoImplEx) servo).setPwmEnable();
        else ((ServoImplEx) servo).setPwmDisable();
    }

    public boolean isLeftPowered() { return leftPowered; }
    public boolean isRightPowered() { return rightPowered; }

    /**
     * Tuning only: send raw positions straight to the servos (no angle math, no rightOffsetPos).
     * Unpowered servos are skipped. reverseRight still applies to the right servo.
     */
    public void writeRaw(double leftPos, double rightPos) {
        if (leftPowered) leftServo.setPosition(Range.clip(leftPos, 0, 1));
        if (rightPowered) rightServo.setPosition(Range.clip(rightPos, 0, 1));
        lastWrittenPos = Double.NaN; // so the next normal command is always sent
        aimMode = AimMode.IDLE;
    }

    // =====================================================================================
    // Main entry (TeleOp). Call once per loop.
    // =====================================================================================
    public void update(double posX, double posY, double robotHeadingRad, Gamepad overridePad, double dt) {
        updateVelocity(posX, posY, robotHeadingRad);
        // No alliance set (auto didn't run) -> aims as RED, see updateTurretTarget.
        updateTurretTarget(posY);
        lastDistance = distanceFromPivot(posX, posY, robotHeadingRad);
        virtualX = turretTargetX;
        virtualY = turretTargetY;

        double stick = overridePad.right_stick_x;
        if (Math.abs(stick) > stickDeadband) {
            zeroHeld = false;
            slew(stick, dt);
            return;
        }

        if (overridePad.b) {
            zeroHeld = false;
        } else if (overridePad.a) {
            zeroHeld = true;
        }

        if (zeroHeld) {
            commandDegrees(0);
            reachable = true;
            aimMode = AimMode.HOLD_ZERO;
            return;
        }

        aimAtTarget(posX, posY, robotHeadingRad);
    }

    /**
     * Autos: picks our alliance's target for the side of the field we're on and aims at it
     * with lead compensation. Call once per loop.
     */
    public void autoAim(double posX, double posY, double robotHeadingRad) {
        updateVelocity(posX, posY, robotHeadingRad);
        updateTurretTarget(posY);
        lastDistance = distanceFromPivot(posX, posY, robotHeadingRad);
        aimAtTarget(posX, posY, robotHeadingRad);
    }

    /** Aim at whatever setTarget() last set (no alliance/side logic). Autos should use autoAim(). */
    public void track(double posX, double posY, double robotHeadingRad) {
        updateVelocity(posX, posY, robotHeadingRad);
        lastDistance = distanceFromPivot(posX, posY, robotHeadingRad);
        aimAtTarget(posX, posY, robotHeadingRad);
    }

    public void setTarget(double x, double y) {
        turretTargetX = x;
        turretTargetY = y;
    }

    public void slew(double stick, double dt) {
        double cappedDt = Range.clip(dt, 0, 0.05);
        commandDegrees(commandedDeg + stick * slewDegPerSec * cappedDt);
        reachable = true;
        aimMode = AimMode.MANUAL;
    }

    /** Point the turret at a robot-relative angle (degrees, + = left). Used for presets / auto. */
    public void setAngleDegrees(double turretDeg) {
        commandDegrees(turretDeg);
        reachable = Math.abs(turretDeg) <= Math.abs(travelLimitDeg);
    }

    // =====================================================================================
    // Aiming math
    // =====================================================================================
    private void aimAtTarget(double posX, double posY, double headingRad) {
        double fvx = getVelX(), fvy = getVelY(), fw = getOmega();

        // 1. predict where the robot will be when the turret gets there
        double px = posX + fvx * latencySec;
        double py = posY + fvy * latencySec;
        double ph = headingRad + fw * latencySec;

        // 2. turret pivot (field) and its velocity: robot velocity + spin around the robot center
        double cos = Math.cos(ph), sin = Math.sin(ph);
        double offX = cos * pivotForwardIn - sin * pivotLeftIn;
        double offY = sin * pivotForwardIn + cos * pivotLeftIn;
        double pivotX = px + offX, pivotY = py + offY;
        double baseVx = fvx - fw * offY;
        double baseVy = fvy + fw * offX;

        // 3. virtual goal: shift the target against our motion by (feed time + flight time).
        //    Flight time depends on distance, which depends on the shift, so iterate.
        double gx = turretTargetX, gy = turretTargetY;
        double dist = Math.hypot(gx - pivotX, gy - pivotY);
        if (SHOOT_ON_THE_MOVE) {
            for (int i = 0; i < SOTM_ITERATIONS; i++) {
                double t = FEED_TIME_SEC + Math.max(0, TOF_A + TOF_B * dist);
                gx = turretTargetX - baseVx * t;
                gy = turretTargetY - baseVy * t;
                dist = Math.hypot(gx - pivotX, gy - pivotY);
            }
        }
        virtualX = gx;
        virtualY = gy;
        lastDistance = dist; // effective distance -> ShotRegression picks RPM/hood for it

        double bearing = Math.atan2(gy - pivotY, gx - pivotX);
        double desired = Math.toDegrees(normalizeRad(bearing - ph)) + trimDeg;
        double limit = Math.abs(travelLimitDeg);
        reachable = Math.abs(desired) <= limit;
        commandDegrees(desired);
        aimMode = AimMode.AUTO;
    }

    /** Robot-relative turret angle to the target, clipped to travel. Kept for compatibility. */
    public double robotRelativeDegrees(double posX, double posY, double robotHeadingRad) {
        double limit = Math.abs(travelLimitDeg);
        return Range.clip(robotRelativeDegreesRaw(posX, posY, robotHeadingRad), -limit, limit);
    }

    private double robotRelativeDegreesRaw(double posX, double posY, double headingRad) {
        double cos = Math.cos(headingRad);
        double sin = Math.sin(headingRad);
        // turret pivot in field coordinates
        double pivotX = posX + cos * pivotForwardIn - sin * pivotLeftIn;
        double pivotY = posY + sin * pivotForwardIn + cos * pivotLeftIn;

        double bearingRad = Math.atan2(turretTargetY - pivotY, turretTargetX - pivotX);
        return Math.toDegrees(normalizeRad(bearingRad - headingRad));
    }

    /** Pinpoint velocity from follower.velocity(): field inches/s and rad/s. Call every loop before aiming. */
    public void setMeasuredVelocity(double vxField, double vyField, double omegaRadPerSec) {
        measVx = vxField;
        measVy = vyField;
        measOmega = omegaRadPerSec;
        haveMeasured = true;
    }

    public double getVelX() { return USE_PINPOINT_VELOCITY && haveMeasured ? measVx : vx; }
    public double getVelY() { return USE_PINPOINT_VELOCITY && haveMeasured ? measVy : vy; }
    public double getOmega() { return USE_PINPOINT_VELOCITY && haveMeasured ? measOmega : omega; }
    /** Pose-difference velocity, for comparing against the Pinpoint on the dashboard. */
    public double getDiffVelX() { return vx; }
    public double getDiffVelY() { return vy; }
    public double getDiffOmega() { return omega; }

    public void toggleShootOnTheMove() { SHOOT_ON_THE_MOVE = !SHOOT_ON_THE_MOVE; }
    public boolean isShootOnTheMove() { return SHOOT_ON_THE_MOVE; }
    public double getVirtualTargetX() { return virtualX; }
    public double getVirtualTargetY() { return virtualY; }

    /** Inches from the turret pivot to the current target. */
    public double distanceFromPivot(double posX, double posY, double headingRad) {
        double cos = Math.cos(headingRad);
        double sin = Math.sin(headingRad);
        double pivotX = posX + cos * pivotForwardIn - sin * pivotLeftIn;
        double pivotY = posY + sin * pivotForwardIn + cos * pivotLeftIn;
        return Math.hypot(turretTargetX - pivotX, turretTargetY - pivotY);
    }

    /** Distance (in) from the turret pivot to the target, as of the last update/autoAim/track. */
    public double getDistanceToTarget() { return lastDistance; }

    private void updateVelocity(double x, double y, double h) {
        long now = System.nanoTime();
        if (havePose) {
            double dt = (now - lastNanos) / 1e9;
            if (dt > 1e-3 && dt < 0.25) {
                double a = Range.clip(velocityAlpha, 0, 1);
                vx = a * ((x - lastX) / dt) + (1 - a) * vx;
                vy = a * ((y - lastY) / dt) + (1 - a) * vy;
                omega = a * (normalizeRad(h - lastH) / dt) + (1 - a) * omega;
            }
            // advance the timing model of where the turret actually is
            if (dt > 0 && dt < 0.25) {
                double step = turretDegPerSec * dt;
                estimatedDeg += Range.clip(commandedDeg - estimatedDeg, -step, step);
            }
        }
        lastNanos = now;
        lastX = x;
        lastY = y;
        lastH = h;
        havePose = true;
    }

    // =====================================================================================
    // Target selection (field coordinates, inches)
    // =====================================================================================
    /**
     * Picks the target on the SAME half of the field as the robot, for our alliance.
     * Switches when the robot crosses sideSwitchY (with hysteresis). No alliance -> RED.
     */
    public void updateTurretTarget(double y) {
        if (!sideKnown) {
            onHighSide = y >= sideSwitchY;
            sideKnown = true;
        } else if (onHighSide && y < sideSwitchY - sideSwitchHysteresisIn) {
            onHighSide = false;
        } else if (!onHighSide && y > sideSwitchY + sideSwitchHysteresisIn) {
            onHighSide = true;
        }

        if (poseStorage.getAlliance() == PoseStorage.Alliance.BLUE) {
            turretTargetX = onHighSide ? blueHighX : blueLowX;
            turretTargetY = onHighSide ? blueHighY : blueLowY;
        } else {
            turretTargetX = onHighSide ? redHighX : redLowX;
            turretTargetY = onHighSide ? redHighY : redLowY;
        }
    }

    public void addTrimDeg(double deg) { trimDeg += deg; }
    public void nudgeTrim(int direction) { trimDeg += direction * trimStepDeg; }
    public void resetTrim() { trimDeg = 0; }
    public double getTrimDeg() { return trimDeg; }
    public boolean isOnHighSide() { return onHighSide; }

    // =====================================================================================
    // Servo output
    // =====================================================================================
    private void commandDegrees(double degrees) {
        double limit = Math.abs(travelLimitDeg);
        commandedDeg = Range.clip(degrees, -limit, limit);
        writeServos();
    }

    private void writeServos() {
        double pos = servoPosition(commandedDeg);
        if (Double.isNaN(lastWrittenPos) || Math.abs(pos - lastWrittenPos) > writeDeadbandPos) {
            // Direction.REVERSE on the right servo does the mirroring, so both get the same number
            // (plus the small offset that lines the two servos up).
            if (leftPowered) leftServo.setPosition(pos);
            if (rightPowered) rightServo.setPosition(Range.clip(pos + rightOffsetPos, 0.0, 1.0));
            lastWrittenPos = pos;
        }
    }

    private double servoPosition(double turretDeg) {
        double range = servoRangeDeg == 0 ? 355 : Math.abs(servoRangeDeg);
        double sign = invertDirection ? -1 : 1;
        return Range.clip(centerPos + sign * turretDeg * gearRatio / range, 0.0, 1.0);
    }

    // =====================================================================================
    // Feedback / status
    // =====================================================================================
    /** Turret angle (deg): measured if an encoder is wired, otherwise the timing model. */
    public double getActualDeg() {
        if (encoder != null) {
            double servoDeg = (encoder.getVoltage() - encoderVoltsAtCenter) * encoderServoDegPerVolt;
            if (encoderInverted) servoDeg = -servoDeg;
            if (invertDirection) servoDeg = -servoDeg;
            return servoDeg / (gearRatio == 0 ? 1 : gearRatio);
        }
        return estimatedDeg;
    }

    /** True when the target is within travel and the turret has arrived. Use it to gate the shooter. */
    public boolean isOnTarget() {
        return reachable && Math.abs(getActualDeg() - commandedDeg) <= onTargetTolDeg;
    }

    public boolean isTargetReachable() { return reachable; }
    public boolean hasEncoder() { return encoder != null; }
    public double getTurretTargetX() { return turretTargetX; }
    public double getTurretTargetY() { return turretTargetY; }
    public double getTurretHeading() { return commandedDeg; }
    public double getLeftServoPosition() { return servoPosition(commandedDeg); }
    public AimMode getAimMode() { return aimMode; }

    private static double normalizeRad(double a) {
        while (a > Math.PI) a -= 2 * Math.PI;
        while (a < -Math.PI) a += 2 * Math.PI;
        return a;
    }
}
