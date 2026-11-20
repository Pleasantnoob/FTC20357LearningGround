package teamcode.Subsystems.Microsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.Range;

/**
 * Match loader: the deployable flower intake ramp, moved by two goBILDA torque servos
 * (one on each side). Both get the same position every time so they move together.
 *
 * The two servos are linked through the ramp, so if one is set wrong they push against
 * each other. Tune them one at a time in TuningStation (LOADER_MODE) before turning
 * Tuned.MATCH_LOADER on.
 */
@Config
public class MatchLoader {

    public static String LEFT_SERVO_NAME = "matchLoaderLeft";
    public static String RIGHT_SERVO_NAME = "matchLoaderRight";

    // PLACEHOLDERS - tune on the robot (FTC Dashboard -> MatchLoader)
    public static double stowedPos = 0.0;
    public static double deployedPos = 0.5;
    /**
     * The servos sit on opposite sides of the ramp facing each other, so one has to spin the
     * other way. PLACEHOLDER - check with LOADER_MODE LEFT_ONLY / RIGHT_ONLY first.
     */
    public static boolean reverseRight = true;
    /** Added to the right servo only, to line the two up so they stop pushing on each other. */
    public static double rightOffsetPos = 0;

    private final Servo left;
    private final Servo right;
    private boolean deployed = false;
    private double lastCommand = Double.NaN;
    private boolean leftPowered = true, rightPowered = true;

    // false = never written, servos stay limp (see RobotContainer/Tuned)
    private boolean enabled = true;
    public void setEnabled(boolean on) { enabled = on; }
    public boolean isEnabled() { return enabled; }

    public MatchLoader(HardwareMap hardwareMap) {
        left = hardwareMap.get(Servo.class, LEFT_SERVO_NAME);
        right = hardwareMap.get(Servo.class, RIGHT_SERVO_NAME);
        left.setDirection(Servo.Direction.FORWARD);
        right.setDirection(reverseRight ? Servo.Direction.REVERSE : Servo.Direction.FORWARD);
        // nothing is written here, so both stay limp until the first deploy/stow
    }

    public void deploy() { deployed = true; command(deployedPos); }
    public void stow() { deployed = false; command(stowedPos); }
    public void toggle() { if (deployed) stow(); else deploy(); }
    public boolean isDeployed() { return deployed; }

    // old names, still used by SubsystemCommands
    public void lowerMatchLoader() { deploy(); }
    public void raiseMatchLoader() { stow(); }

    private void command(double position) {
        if (!enabled || position == lastCommand) return;
        if (leftPowered) left.setPosition(position);
        if (rightPowered) right.setPosition(Range.clip(position + rightOffsetPos, 0, 1));
        lastCommand = position;
    }

    // ---------------- Tuning only ----------------

    /** Turn each servo's signal on/off. With no signal a servo goes limp, so the other can move the ramp alone. */
    public void setPowered(boolean leftOn, boolean rightOn) {
        leftPowered = leftOn;
        rightPowered = rightOn;
        setPwm(left, leftOn);
        setPwm(right, rightOn);
        lastCommand = Double.NaN; // so the next command is always sent
    }

    /** Raw positions straight to the servos. Unpowered ones are skipped (setPosition would wake them up). */
    public void writeRaw(double leftPos, double rightPos) {
        if (leftPowered) left.setPosition(Range.clip(leftPos, 0, 1));
        if (rightPowered) right.setPosition(Range.clip(rightPos, 0, 1));
        lastCommand = Double.NaN;
    }

    private static void setPwm(Servo servo, boolean on) {
        if (!(servo instanceof ServoImplEx)) return;
        if (on) ((ServoImplEx) servo).setPwmEnable();
        else ((ServoImplEx) servo).setPwmDisable();
    }
}
