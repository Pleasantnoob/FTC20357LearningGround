package teamcode.ComplexMath;

import com.acmerobotics.dashboard.config.Config;

/**
 * Distance -> flywheel RPM and hood position, from equations you fit in Desmos.
 *
 *   d    = inches from the turret pivot to the target ("distance to target" on the dashboard)
 *   rpm  = RPM_A  + RPM_B  * d + RPM_C  * d^2
 *   hood = HOOD_A + HOOD_B * d + HOOD_C * d^2
 *
 * Use a straight line by leaving the C terms at 0.
 * You can try values live in FTC Dashboard (group "ShotRegression"), but dashboard edits are
 * lost when the robot restarts, so type the final numbers in here.
 *
 * While all RPM terms are 0, everything uses FALLBACK_RPM.
 * While all HOOD terms are 0, the hood stays at TurretHood.defaultPos.
 */
@Config
public class ShotRegression {

    public static double RPM_A = 0;
    public static double RPM_B = 0;
    public static double RPM_C = 0;

    public static double HOOD_A = 0;
    public static double HOOD_B = 0;
    public static double HOOD_C = 0;

    /** Distances outside this range are clamped, so the curve doesn't run off past where you tested. */
    public static double MIN_DIST = 0;
    public static double MAX_DIST = 200;

    public static double FALLBACK_RPM = 2240;
    public static double MAX_RPM = 5800;

    public static boolean hasData() {
        return RPM_A != 0 || RPM_B != 0 || RPM_C != 0;
    }

    public static boolean hasHood() {
        return HOOD_A != 0 || HOOD_B != 0 || HOOD_C != 0;
    }

    public static double rpmFor(double distanceIn) {
        if (!hasData()) return FALLBACK_RPM;
        double d = clampDist(distanceIn);
        double rpm = RPM_A + RPM_B * d + RPM_C * d * d;
        return Math.max(0, Math.min(MAX_RPM, rpm));
    }

    /** NaN when no hood equation is set; the hood then keeps its default. */
    public static double hoodFor(double distanceIn) {
        if (!hasHood()) return Double.NaN;
        double d = clampDist(distanceIn);
        return HOOD_A + HOOD_B * d + HOOD_C * d * d;
    }

    private static double clampDist(double d) {
        return Math.max(MIN_DIST, Math.min(MAX_DIST, d));
    }
}
