package org.firstinspires.ftc.teamcode.Toros.Drive;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Vector2d;

/**
 * Shooting zone boundaries and field bounds. Uses robot pose (odometry) to determine
 * whether the robot is in the close launching triangle or far distance band.
 *
 * Field: 12 ft × 12 ft → X and Y from -72 to +72 inches (origin at center).
 * Close launching zone: triangle (-72, 72) → (0, 0) → (-72, -72). Far zone by distance from goal.
 */
@Config
public final class ShootingZones {

    /** Field half-width (inches). Full field 144" so ±72. */
    public static double FIELD_HALF_SIZE = 72.0;

    /** Close launching zone: triangle vertices (field coords, inches). (-72,72) → (0,0) → (-72,-72). */
    public static double closeTriX1 = -72.0, closeTriY1 = 72.0;   // top-left
    public static double closeTriX2 = 0.0, closeTriY2 = 0.0;      // center
    public static double closeTriX3 = -72.0, closeTriY3 = -72.0; // bottom-left

    /** Far shooting zone: distance from goal in [farMinIn, farMaxIn]. */
    public static double farMinIn = 48.0;
    public static double farMaxIn = 120.0;

    public enum Zone {
        NONE,   // outside close triangle and far band
        CLOSE,  // inside close launching triangle
        FAR     // in far distance band (and not in close triangle)
    }

    /** Cross product (b - a) × (p - a) in 2D. Sign tells which side of segment ab point p is on. */
    private static double crossSign(double ax, double ay, double bx, double by, double px, double py) {
        return (bx - ax) * (py - ay) - (by - ay) * (px - ax);
    }

    /** True if (x, y) is inside the close launching triangle. Uses same-side-of-edge test: p is inside iff it lies on the same side of each edge as the opposite vertex. */
    public static boolean isInCloseLaunchTriangle(double x, double y) {
        double s1 = crossSign(closeTriX1, closeTriY1, closeTriX2, closeTriY2, closeTriX3, closeTriY3);
        double p1 = crossSign(closeTriX1, closeTriY1, closeTriX2, closeTriY2, x, y);
        if (Math.signum(s1) != Math.signum(p1) && p1 != 0) return false;
        double s2 = crossSign(closeTriX2, closeTriY2, closeTriX3, closeTriY3, closeTriX1, closeTriY1);
        double p2 = crossSign(closeTriX2, closeTriY2, closeTriX3, closeTriY3, x, y);
        if (Math.signum(s2) != Math.signum(p2) && p2 != 0) return false;
        double s3 = crossSign(closeTriX3, closeTriY3, closeTriX1, closeTriY1, closeTriX2, closeTriY2);
        double p3 = crossSign(closeTriX3, closeTriY3, closeTriX1, closeTriY1, x, y);
        return Math.signum(s3) == Math.signum(p3) || p3 == 0;
    }

    /** True if (x,y) is inside the field rectangle [-FIELD_HALF_SIZE, FIELD_HALF_SIZE]. */
    public static boolean isInsideField(double x, double y) {
        return Math.abs(x) <= FIELD_HALF_SIZE && Math.abs(y) <= FIELD_HALF_SIZE;
    }

    /** True if (x,y) is inside field; overload for Vector2d. */
    public static boolean isInsideField(Vector2d position) {
        return isInsideField(position.x, position.y);
    }

    /** Distance from (robotX, robotY) to (goalX, goalY) in inches. */
    public static double distanceToGoal(double robotX, double robotY, double goalX, double goalY) {
        return Math.hypot(robotX - goalX, robotY - goalY);
    }

    /** True if robot is in the far shooting zone by distance to goal. */
    public static boolean isInFarShootingZone(double robotX, double robotY, double goalX, double goalY) {
        double d = distanceToGoal(robotX, robotY, goalX, goalY);
        return d >= farMinIn && d <= farMaxIn;
    }

    /**
     * Which zone the robot is in. CLOSE = inside the launching triangle; FAR = in far distance band; else NONE.
     */
    public static Zone getShootingZone(double robotX, double robotY, double goalX, double goalY) {
        if (isInCloseLaunchTriangle(robotX, robotY)) return Zone.CLOSE;
        if (isInFarShootingZone(robotX, robotY, goalX, goalY)) return Zone.FAR;
        return Zone.NONE;
    }
}
