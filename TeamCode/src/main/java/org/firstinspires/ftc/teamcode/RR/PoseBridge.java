package org.firstinspires.ftc.teamcode.RR;

import com.acmerobotics.roadrunner.Pose2d;

/**
 * Bridges field position, heading, and alliance from Autonomous to Teleop.
 * Static state persists while the app is running, so when you run Auto then Teleop
 * in the same session, Teleop can start from the pose Auto ended at and the same alliance.
 *
 * <p>Auto: call {@link #save(Pose2d)} with the current localizer pose (e.g. at end of autonomous
 * or each loop). Call {@link #saveAlliance(boolean)} with true for Blue, false for Red.
 * Teleop: in init, use {@link #getPose()} and pass to your localizer's setPose
 * if {@link #hasPose()} is true. Use {@link #getAlliance()} if {@link #hasAlliance()} to set
 * the alliance (true = Blue, false = Red).
 */
public final class PoseBridge {
    private static volatile boolean hasPose;
    private static volatile double x;
    private static volatile double y;
    private static volatile double headingRad;
    private static volatile boolean hasAlliance;
    private static volatile boolean isBlue;

    private PoseBridge() {}

    /** Saves a pose so Teleop can use it as its starting pose. */
    public static void save(Pose2d pose) {
        x = pose.position.x;
        y = pose.position.y;
        headingRad = pose.heading.toDouble();
        hasPose = true;
    }

    /** Saves alliance for Teleop. true = Blue, false = Red. */
    public static void saveAlliance(boolean blue) {
        isBlue = blue;
        hasAlliance = true;
    }

    /** Returns true if an alliance was saved. */
    public static boolean hasAlliance() {
        return hasAlliance;
    }

    /** Returns saved alliance: true = Blue, false = Red. */
    public static boolean getAlliance() {
        return isBlue;
    }

    /** Returns true if a pose was saved (e.g. from a previous Auto run). */
    public static boolean hasPose() {
        return hasPose;
    }

    /**
     * Returns the saved pose, or null if none. Does not clear the saved pose;
     * call {@link #clear()} after applying so you don't reuse stale data.
     */
    public static Pose2d getPose() {
        if (!hasPose) return null;
        return new Pose2d(x, y, headingRad);
    }

    /** Clears the saved pose so it isn't reused on a later Teleop start. */
    public static void clear() {
        hasPose = false;
    }

    /** Clears the saved alliance. */
    public static void clearAlliance() {
        hasAlliance = false;
    }
}
