package org.firstinspires.ftc.teamcode.util;

/**
 * Robot pose (x, y in field inches, heading in radians). Used for Drawing, PoseBridge, and Pedro path conversion.
 */
public final class Pose2d {

    /** Convert Road Runner Pose2d to util Pose2d (for RR package and autos still using RR localizer). */
    public static Pose2d fromRR(com.acmerobotics.roadrunner.Pose2d r) {
        return new Pose2d(r.position.x, r.position.y, r.heading.toDouble());
    }
    public final Vector2d position;
    /** Heading in radians. */
    public final double heading;

    public Pose2d(double x, double y, double headingRad) {
        this.position = new Vector2d(x, y);
        this.heading = headingRad;
    }

    public Pose2d(Vector2d position, double headingRad) {
        this.position = position;
        this.heading = headingRad;
    }

    /** Unit vector in heading direction. */
    public Vector2d headingVec() {
        return new Vector2d(Math.cos(heading), Math.sin(heading));
    }
}
