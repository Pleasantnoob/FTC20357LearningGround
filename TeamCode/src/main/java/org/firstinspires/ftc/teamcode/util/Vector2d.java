package org.firstinspires.ftc.teamcode.util;

/**
 * 2D vector (field inches). Used with Pose2d for compatibility across drive systems.
 */
public final class Vector2d {
    public final double x;
    public final double y;

    public Vector2d(double x, double y) {
        this.x = x;
        this.y = y;
    }

    public Vector2d plus(Vector2d other) {
        return new Vector2d(x + other.x, y + other.y);
    }

    public Vector2d times(double s) {
        return new Vector2d(x * s, y * s);
    }
}
