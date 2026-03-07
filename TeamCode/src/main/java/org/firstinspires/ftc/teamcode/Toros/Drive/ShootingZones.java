package org.firstinspires.ftc.teamcode.Toros.Drive;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Vector2d;

/**
 * Shooting zone boundaries and field bounds. Uses robot pose (odometry) to determine
 * whether the robot is in the close launching triangle, far zone triangle, or far distance band.
 *
 * Field: 12 ft × 12 ft → X and Y from -72 to +72 inches (origin at center).
 * Close launching zone: triangle (-72, 72) → (0, 0) → (-72, -72).
 * Far zone triangle: little triangle in back of field (47, 0) → (72, 24) → (72, -24).
 */
@Config
public final class ShootingZones {

    /** Field half-width (inches). Full field 144" so ±72. */
    public static double FIELD_HALF_SIZE = 72.0;

    /** Close launching zone: triangle vertices (field coords, inches). (-72,72) → (0,0) → (-72,-72). */
    public static double closeTriX1 = -72.0, closeTriY1 = 72.0;   // top-left
    public static double closeTriX2 = 0.0, closeTriY2 = 0.0;      // center
    public static double closeTriX3 = -72.0, closeTriY3 = -72.0; // bottom-left

    /** Far zone triangle: little triangle in back of field. (47,0) → (72,24) → (72,-24). */
    public static double farTriX1 = 47.0, farTriY1 = 0.0;     // left vertex
    public static double farTriX2 = 72.0, farTriY2 = 24.0;    // top-right
    public static double farTriX3 = 72.0, farTriY3 = -24.0;   // bottom-right

    /** Far shooting zone (distance band): distance from goal in [farMinIn, farMaxIn]. */
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

    /** True if (x, y) is inside the far zone triangle (back of field). Same point-in-triangle logic as close. */
    public static boolean isInFarZoneTriangle(double x, double y) {
        double s1 = crossSign(farTriX1, farTriY1, farTriX2, farTriY2, farTriX3, farTriY3);
        double p1 = crossSign(farTriX1, farTriY1, farTriX2, farTriY2, x, y);
        if (Math.signum(s1) != Math.signum(p1) && p1 != 0) return false;
        double s2 = crossSign(farTriX2, farTriY2, farTriX3, farTriY3, farTriX1, farTriY1);
        double p2 = crossSign(farTriX2, farTriY2, farTriX3, farTriY3, x, y);
        if (Math.signum(s2) != Math.signum(p2) && p2 != 0) return false;
        double s3 = crossSign(farTriX3, farTriY3, farTriX1, farTriY1, farTriX2, farTriY2);
        double p3 = crossSign(farTriX3, farTriY3, farTriX1, farTriY1, x, y);
        return Math.signum(s3) == Math.signum(p3) || p3 == 0;
    }

    /** Radius (inches) of circle around robot for zone overlap check. If circle crosses zone boundary, ready to shoot. */
    public static double robotZoneCircleRadius = 8.0;

    /** Distance from point (px,py) to line segment (ax,ay)-(bx,by). */
    private static double pointToSegmentDist(double px, double py, double ax, double ay, double bx, double by) {
        double dx = bx - ax, dy = by - ay;
        double lenSq = dx * dx + dy * dy;
        if (lenSq == 0) return Math.hypot(px - ax, py - ay);
        double t = Math.max(0, Math.min(1, ((px - ax) * dx + (py - ay) * dy) / lenSq));
        double qx = ax + t * dx, qy = ay + t * dy;
        return Math.hypot(px - qx, py - qy);
    }

    /** Min distance from point (px,py) to triangle (x1,y1)-(x2,y2)-(x3,y3). Returns 0 if point inside. */
    private static double pointToTriangleDist(double px, double py, double x1, double y1, double x2, double y2, double x3, double y3) {
        if (isPointInTriangle(px, py, x1, y1, x2, y2, x3, y3)) return 0;
        double d1 = pointToSegmentDist(px, py, x1, y1, x2, y2);
        double d2 = pointToSegmentDist(px, py, x2, y2, x3, y3);
        double d3 = pointToSegmentDist(px, py, x3, y3, x1, y1);
        return Math.min(d1, Math.min(d2, d3));
    }

    private static boolean isPointInTriangle(double px, double py, double x1, double y1, double x2, double y2, double x3, double y3) {
        double s1 = crossSign(x1, y1, x2, y2, x3, y3);
        double p1 = crossSign(x1, y1, x2, y2, px, py);
        if (Math.signum(s1) != Math.signum(p1) && p1 != 0) return false;
        double s2 = crossSign(x2, y2, x3, y3, x1, y1);
        double p2 = crossSign(x2, y2, x3, y3, px, py);
        if (Math.signum(s2) != Math.signum(p2) && p2 != 0) return false;
        double s3 = crossSign(x3, y3, x1, y1, x2, y2);
        double p3 = crossSign(x3, y3, x1, y1, px, py);
        return Math.signum(s3) == Math.signum(p3) || p3 == 0;
    }

    /** True if circle of given radius around (robotX, robotY) intersects the close launch triangle. */
    public static boolean circleIntersectsCloseLaunchTriangle(double robotX, double robotY, double radius) {
        double d = pointToTriangleDist(robotX, robotY, closeTriX1, closeTriY1, closeTriX2, closeTriY2, closeTriX3, closeTriY3);
        return d <= radius;
    }

    /** True if circle of given radius around (robotX, robotY) intersects the far zone triangle. */
    public static boolean circleIntersectsFarZoneTriangle(double robotX, double robotY, double radius) {
        double d = pointToTriangleDist(robotX, robotY, farTriX1, farTriY1, farTriX2, farTriY2, farTriX3, farTriY3);
        return d <= radius;
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
