package org.firstinspires.ftc.teamcode.Toros.Drive;

/**
 * Shot physics for Dashboard: time in air and launch speed from distance and hood angle.
 * Uses same projectile model as IntakeV2 (launch height h, gravity g).
 */
public final class ShotPhysics {
    private static final double G = 9.81;
    /** Launch height above target plane (m). Match IntakeV2.h if needed. */
    private static final double H = 0.69;

    /**
     * Required launch speed (m/s) for a shot at horizontal distance d (meters) and launch angle theta (radians).
     * From: d = v*cos(theta)*t, 0 = v*sin(theta)*t - 0.5*g*t^2 + h => v = d*sqrt(g / (2*cos²(theta)*(d*tan(theta)-h))).
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
     * From odometry distance to goal (inches), compute hood angle (deg) and launch speed (m/s).
     * Uses same linear hood mapping as IntakeV2: closer = steeper (70°), farther = flatter (40°). Range 40–70°.
     * Returns { hoodAngleDeg, speedMPS }. Distance clamped to valid range for physics.
     */
    public static double[] hoodAndSpeedFromDistanceInches(double distanceInches) {
        double dM = distanceInches * 0.0254;
        dM = Math.max(0.3, Math.min(3.5, dM));
        // Linear: at 0.5 m use 70°, at 1.5 m use 40° (range 40–70)
        double hoodAngleDeg = 70.0 + (dM - 0.5) * (40.0 - 70.0) / 1.0;
        hoodAngleDeg = Math.max(40, Math.min(70, hoodAngleDeg));
        double theta = Math.toRadians(hoodAngleDeg);
        double v = speedForShot(dM, theta);
        return new double[] { hoodAngleDeg, v };
    }
}
