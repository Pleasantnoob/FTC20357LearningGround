package teamcode.RobotContainer;

import com.acmerobotics.dashboard.config.Config;

/**
 * "Has this been tuned yet?" switches. While a switch is false, that mechanism does nothing in
 * MainDrive and the autos: its servos get no signal (limp) and the flywheel stays at 0 power.
 * That way a placeholder value can't slam a servo into a hard stop or make the two turret
 * servos fight.
 *
 * TuningStation ignores these and uses its own on/off switches, so you can still tune.
 *
 * Flip a switch to true ONLY after the real values for that mechanism are in the code.
 * Read once when the OpMode inits, so changing them in FTC Dashboard needs a re-init.
 */
@Config
public class Tuned {
    /** Turret.centerPos, servoRangeDeg, reverseRight, invertDirection, rightOffsetPos. */
    public static boolean TURRET = false;
    /** TurretHood minPos / maxPos / defaultPos. */
    public static boolean HOOD = false;
    /** TurretGate closeGatePos / openGatePos. */
    public static boolean GATE = false;
    /** MatchLoader stowedPos / deployedPos. */
    public static boolean MATCH_LOADER = false;
    /** Shooter kS / kV / kP (still the old robot's numbers). */
    public static boolean FLYWHEEL = false;
}
