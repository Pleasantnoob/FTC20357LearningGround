package org.firstinspires.ftc.teamcode.Toros.Drive;

/**
 * Shot physics: time in air and launch speed from distance and hood angle.
 * Projectile model: launch at height H, flat ground; same H and g as IntakeV2 for consistency.
 */
public final class ShotPhysics {
    private static final double G = 9.81;
    /** Launch height above target plane (m). Must match IntakeV2 if used for feedforward. */
    private static final double H = 0.69;

    /**
     * Required launch speed (m/s) for a shot at horizontal distance d (m) and launch angle theta (rad).
     * Derived from: range d = v*cos(θ)*t and 0 = v*sin(θ)*t - 0.5*g*t² + h. Eliminate t and solve for v.
     */
    public static double speedForShot(double distanceM, double thetaRad) {
        double cos = Math.cos(thetaRad);
        double tan = Math.tan(thetaRad);
        double denom = 2 * cos * cos * (distanceM * tan - H);
        if (denom <= 0) return 0;
        return distanceM * Math.sqrt(G / denom);
    }

    /** Time in air (s) for that shot: t = 2*v*sin(theta)/g (time to same height). */
    public static double timeInAir(double speedMS, double thetaRad) {
        return 2 * speedMS * Math.sin(thetaRad) / G;
    }

    /**
     * Distance to goal in inches -> meters, hood angle in degrees -> rad.
     * Returns { speed (m/s), time in air (s) }.
     */
    public static double[] speedAndTimeInAir(double distanceInches, double hoodAngleDeg) {
        double dM = distanceInches * 0.0254;
        double theta = Math.toRadians(hoodAngleDeg);
        double v = speedForShot(dM, theta);
        double t = timeInAir(v, theta);
        return new double[] { v, t };
    }

    /**
     * Hood angle (deg) and launch speed (m/s) from distance to goal (inches).
     * Linear hood: 0.5 m → 70°, 1.5 m → 40° (closer = steeper). Distance clamped 0.3–3.5 m for valid physics.
     */
    public static double[] hoodAndSpeedFromDistanceInches(double distanceInches) {
        double dM = distanceInches * 0.0254;
        dM = Math.max(0.3, Math.min(3.5, dM));
        double hoodAngleDeg = 70.0 + (dM - 0.5) * (40.0 - 70.0) / 1.0;
        hoodAngleDeg = Math.max(40, Math.min(70, hoodAngleDeg));
        double theta = Math.toRadians(hoodAngleDeg);
        double v = speedForShot(dM, theta);
        return new double[] { hoodAngleDeg, v };
    }

    private static final double MIN_HOOD_DEG = 40.0;
    private static final double MAX_HOOD_DEG = 70.0;
    private static final double DEG_STEP = 0.5;

    /**
     * Fast shot: minimize time in air subject to hood [40°, 70°] and launch speed ≤ vMaxMPS.
     * Picks the flattest feasible angle (smallest θ with v_required ≤ vMax) for minimum T.
     * Returns { hoodAngleDeg, speedMPS }. Distance clamped to valid physics range.
     */
    public static double[] fastShotHoodAndSpeedMPS(double distanceInches, double vMaxMPS) {
        double dM = distanceInches * 0.0254;
        dM = Math.max(0.3, Math.min(3.5, dM));
        double thetaDeg = MIN_HOOD_DEG;
        double vRequired = speedForShot(dM, Math.toRadians(thetaDeg));
        while (vRequired > vMaxMPS && thetaDeg <= MAX_HOOD_DEG) {
            thetaDeg += DEG_STEP;
            vRequired = speedForShot(dM, Math.toRadians(thetaDeg));
        }
        thetaDeg = Math.min(thetaDeg, MAX_HOOD_DEG);
        double v = Math.min(speedForShot(dM, Math.toRadians(thetaDeg)), vMaxMPS);
        return new double[] { thetaDeg, v };
    }

    /**
     * Slow shot: maximize time in air subject to hood [40°, 70°] and launch speed ≤ vMaxMPS.
     * Finds the steepest angle (max θ) that still reaches the goal with v ≤ vMax.
     * Returns { hoodAngleDeg, speedMPS }. Distance clamped to valid physics range.
     */
    public static double[] slowShotHoodAndSpeedMPS(double distanceInches, double vMaxMPS) {
        double dM = distanceInches * 0.0254;
        dM = Math.max(0.3, Math.min(3.5, dM));
        double thetaDeg = MAX_HOOD_DEG;
        double vRequired = speedForShot(dM, Math.toRadians(thetaDeg));
        while (vRequired > vMaxMPS && thetaDeg >= MIN_HOOD_DEG) {
            thetaDeg -= DEG_STEP;
            vRequired = speedForShot(dM, Math.toRadians(thetaDeg));
        }
        thetaDeg = Math.max(thetaDeg, MIN_HOOD_DEG);
        double v = Math.min(speedForShot(dM, Math.toRadians(thetaDeg)), vMaxMPS);
        return new double[] { thetaDeg, v };
    }
}
