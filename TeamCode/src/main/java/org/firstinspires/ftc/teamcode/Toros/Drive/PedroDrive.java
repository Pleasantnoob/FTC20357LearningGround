package org.firstinspires.ftc.teamcode.Toros.Drive;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.util.Pose2d;

/**
 * Pedro Pathing drive wrapper. Converts between field coords (center origin, inches)
 * and Pedro coords (origin bottom-left, 0–144). Provides getPose/setPose in field coords.
 * Autos use {@link #buildPath(Pose2d, Pose2d)} and {@link #buildPathChain(Pose2d...)}; pathing logic is not changed here.
 */
public final class PedroDrive {
    private static final double PEDRO_OFFSET = 72.0;

    private final Follower follower;

    public PedroDrive(HardwareMap hardwareMap, Pose2d initialPose) {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(toPedro(initialPose));
    }

    /** Convert field pose (center origin) to Pedro pose. */
    public static Pose toPedro(Pose2d p) {
        return new Pose(p.position.x + PEDRO_OFFSET, p.position.y + PEDRO_OFFSET, p.heading);
    }

    /** Convert Pedro pose to field pose (center origin). */
    public static Pose2d fromPedro(Pose p) {
        return new Pose2d(p.getX() - PEDRO_OFFSET, p.getY() - PEDRO_OFFSET, p.getHeading());
    }

    public Pose2d getPose() {
        return fromPedro(follower.getPose());
    }

    /** Set current pose (e.g. from PoseBridge at teleop start). */
    public void setPose(Pose2d pose) {
        follower.setStartingPose(toPedro(pose));
    }

    /** Build a path chain from consecutive waypoints (straight segments with linear heading). */
    public PathChain buildPathChain(Pose2d... waypoints) {
        if (waypoints == null || waypoints.length < 2) {
            throw new IllegalArgumentException("Need at least 2 waypoints");
        }
        com.pedropathing.paths.PathBuilder b = follower.pathBuilder();
        for (int i = 0; i < waypoints.length - 1; i++) {
            Pose start = toPedro(waypoints[i]);
            Pose end = toPedro(waypoints[i + 1]);
            b.addPath(new BezierLine(start, end))
                    .setLinearHeadingInterpolation(start.getHeading(), end.getHeading());
        }
        return b.build();
    }

    /** Build a single-segment path from start to end. */
    public PathChain buildPath(Pose2d start, Pose2d end) {
        Pose ps = toPedro(start);
        Pose pe = toPedro(end);
        return follower.pathBuilder()
                .addPath(new BezierLine(ps, pe))
                .setLinearHeadingInterpolation(ps.getHeading(), pe.getHeading())
                .build();
    }

    public void followPath(PathChain pathChain) {
        follower.followPath(pathChain);
    }

    /** Follow path and hold end pose when complete (e.g. while running intake). */
    public void followPath(PathChain pathChain, boolean holdEnd) {
        follower.followPath(pathChain, holdEnd);
    }

    public boolean isBusy() {
        return follower.isBusy();
    }

    /** Call every loop during auto or teleop. */
    public void update() {
        follower.update();
    }

    /** Call at start of teleop. */
    public void startTeleopDrive() {
        follower.startTeleopDrive();
    }

    /** Cancel path following and return to teleop drive. Call when driver takes over (e.g. exits auto-park). */
    public void breakFollowing() {
        follower.breakFollowing();
        follower.startTeleopDrive();
    }

    /** Robot-centric: lx, ly, rx are -1..1 (stick values). fieldCentric true = field-relative. */
    public void setTeleOpDrive(double lx, double ly, double rx, boolean fieldCentric) {
        follower.setTeleOpDrive(lx, ly, rx, fieldCentric);
    }

    /** Velocity (vx, vy, omega) in robot frame: [vx, vy, omega] for display/compensation. Returns zeros if unavailable. */
    public double[] getVelocity() {
        try {
            com.pedropathing.math.Vector v = follower.getVelocity();
            if (v == null) return ZERO_VELOCITY;
            double vx = v.getXComponent();
            double vy = v.getYComponent();
            double omega = follower.getAngularVelocity();
            return new double[]{vx, vy, omega};
        } catch (Throwable t) {
            return ZERO_VELOCITY;
        }
    }

    private static final double[] ZERO_VELOCITY = new double[]{0, 0, 0};
}
