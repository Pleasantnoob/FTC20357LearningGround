package teamcode.OpModes.Auto.Solo;

import static com.pedropathing.ivy.commands.Commands.instant;
import static com.pedropathing.ivy.commands.Commands.waitMs;
import static com.pedropathing.ivy.commands.Commands.waitUntil;
import static com.pedropathing.ivy.groups.Groups.race;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.api.Paths;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.interpolator.Interpolator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import teamcode.ComplexMath.ShotRegression;
import teamcode.ComplexMath.Storage.PoseStorage;
import teamcode.RobotContainer.Robot;
import teamcode.Utility.DashboardField;
import teamcode.Utility.Field;
import teamcode.pedro.Constants;

/**
 * Garden + flowers auto for both alliances. The poses below are the RED version, straight from
 * the Pedro visualizer ((0,0) = corner by the red garden). The BLUE version is the same auto
 * rotated 180 deg around the field center: x -> 141.5 - x, y -> 141.5 - y, heading + 180
 * (141.5 = the visualizer's field size, see Utility/Field).
 * The field is built that way (red garden / flowers / loading zone rotate onto blue's), and a
 * rotation doesn't mirror the robot, so the same robot works on both sides.
 * Don't mirror across one axis: that flips the robot left-right and lands on a corner with no garden.
 *
 * The turret aims the whole time with the same rule as TeleOp: our CELL on the half of
 * the field the robot is on. So shots from the start / path 6 go into the low-y CELL and
 * shots from the far wall (after paths 2 and 3) go into the high-y CELL.
 *
 *   (red version; blue is the same with high/low swapped by the rotation)
 *   start ... shoot 4 preloads (low CELL: 3 NECTAR + 4 = tips)
 *   path 1 .. drive along the wall into the corner with the intake on (garden, 4 POLLEN)
 *   path 2 .. cross to the far wall, shoot 4 (high CELL)
 *   path 3 .. ramp down, creep into the flower, sit there intaking, ramp up, shoot 4 (high CELL, 8 = tips)
 *   path 4-5  same at the second flower (intake only)
 *   path 6 .. back to the start wall, shoot 4 (low CELL again)
 *   path 7 .. park in the loading zone
 *
 * The robot can only hold 4 at a time (G407), which is why every pickup has a volley in between.
 * Mechanisms that aren't marked tuned in Tuned.java do nothing here, so this is safe to run early.
 */
@Config
public abstract class GardenFlowersAuto extends LinearOpMode {

    /** RED or BLUE. Subclasses (RedGardenFlowersAuto / BlueGardenFlowersAuto) pick it. */
    protected abstract PoseStorage.Alliance alliance();

    // ---- Timing (ms), tune on the field ----
    /** Longest wait for flywheel + turret to be ready before firing anyway. */
    public static double READY_TIMEOUT_MS = 1500;
    /** How long the gate stays open for a volley of 4. */
    public static double VOLLEY_MS = 1200;
    /** How long to sit at a flower with the ramp down and intake running. */
    public static double FLOWER_DWELL_MS = 1000;
    /** Time for the ramp servos to reach deployed before driving in. */
    public static double RAMP_MS = 300;
    /** Flywheel RPM counts as "ready" within this much of the target. */
    public static double RPM_TOLERANCE = 40;

    private Follower follower;
    private Robot robot;
    private final PoseFactory poseFactory = PoseFactory.degrees();

    /** Red pose as drawn in the visualizer; rotated 180 deg for blue. */
    private Pose p(double x, double y, double headingDeg) {
        if (alliance() == PoseStorage.Alliance.BLUE) {
            return poseFactory.of(Field.rotX(x), Field.rotY(y), headingDeg + 180);
        }
        return poseFactory.of(x, y, headingDeg);
    }

    // ---- Poses: RED values (Pedro inches / degrees), straight from the visualizer ----
    private final Pose start = p(53.75, 6.75, 90);   // from the .pp.json; turns to 180 during path 1
    private final Pose gardenCorner = p(7.75, 6.75, 180);         // end of path 1
    private final Pose point2 = p(46.75, 125.75, 90);             // far wall, shoot spot
    private final Pose point2Control1 = p(24.75, 72.75, 0);
    private final Pose point2Segment1Start = p(46.75, 125.75, 180);
    private final Pose point2Segment1End = p(46.75, 125.75, 90);
    private final Pose point2Segment2Start = p(46.75, 125.75, 90);
    private final Pose point2Segment2End = p(46.75, 125.75, 90);
    private final Pose point3 = p(46.75, 130.75, 90);             // flower 1 (far wall)
    private final Pose point4 = p(15.75, 46.75, 180);
    private final Pose point4Control1 = p(20.75, 51.75, 0);
    private final Pose point4Segment1Start = p(15.75, 46.75, 90);
    private final Pose point4Segment1End = p(15.75, 46.75, 90);
    private final Pose point4Segment2Start = p(15.75, 46.75, 90);
    private final Pose point4Segment2End = p(15.75, 46.75, 180);
    private final Pose point5 = p(8.75, 46.75, 180);              // flower 2 (red wall)
    private final Pose point6 = p(37.75, 6.75, 180);              // shoot spot by the start
    private final Pose point7 = p(9.75, 94.75, 90);               // park (loading zone)
    private final Pose point7Segment1Start = p(9.75, 94.75, 180);
    private final Pose point7Segment1End = p(9.75, 94.75, 90);
    private final Pose point7Segment2Start = p(9.75, 94.75, 90);
    private final Pose point7Segment2End = p(9.75, 94.75, 90);

    // ---- Paths ----
    private Path path1() { return Paths.line(start, gardenCorner).linear(start, gardenCorner); }

    private Path path2() {
        return Paths.curve(gardenCorner, point2Control1, point2).heading(Interpolator.piecewise()
                .until(0.1544, Interpolator.linear(point2Segment1Start, point2Segment1End))
                .until(1, Interpolator.linear(point2Segment2Start, point2Segment2End)));
    }

    private Path path3() { return Paths.line(point2, point3).tangent(); }

    private Path path4() {
        return Paths.curve(point3, point4Control1, point4).heading(Interpolator.piecewise()
                .until(0.703, Interpolator.linear(point4Segment1Start, point4Segment1End))
                .until(1, Interpolator.linear(point4Segment2Start, point4Segment2End)));
    }

    private Path path5() { return Paths.line(point4, point5).tangent(); }

    private Path path6() { return Paths.line(point5, point6).constant(point6); }

    private Path path7() {
        return Paths.line(point6, point7).heading(Interpolator.piecewise()
                .until(0.2466, Interpolator.linear(point7Segment1Start, point7Segment1End))
                .until(1, Interpolator.linear(point7Segment2Start, point7Segment2End)));
    }

    // ---- Building blocks ----

    /** True when the flywheel is at speed and the turret has arrived. */
    private boolean readyToFire() {
        int target = robot.shooter.getTargetRPM();
        return target > 0
                && Math.abs(target - robot.shooter.getRPM()) < RPM_TOLERANCE
                && robot.turret.isOnTarget();
    }

    /** Wait until ready (or the timeout), open the gate and feed for VOLLEY_MS, then close. */
    private Command volley() {
        return sequential(
                race(waitUntil(this::readyToFire), waitMs(READY_TIMEOUT_MS)),
                instant(() -> { robot.turretGate.moveGateToOpenPos(); robot.intake.collect(); }),
                waitMs(VOLLEY_MS),
                instant(() -> { robot.turretGate.moveGateToClosePos(); robot.intake.stop(); })
        );
    }

    private Command intakeOn() { return instant(() -> robot.intake.collect()); }
    private Command intakeOff() { return instant(() -> robot.intake.stop()); }

    /** Ramp down + intake on, creep into the flower, sit there, ramp up. */
    private Command flower(Path creepIn) {
        return sequential(
                instant(() -> { robot.matchLoader.deploy(); robot.intake.collect(); }),
                waitMs(RAMP_MS),
                follow(follower, creepIn),
                waitMs(FLOWER_DWELL_MS),
                instant(() -> { robot.matchLoader.stow(); robot.intake.stop(); })
        );
    }

    private Command autoRoutine() {
        return sequential(
                volley(),                         // 4 preloads -> low CELL
                intakeOn(),
                follow(follower, path1()),        // sweep the red garden
                follow(follower, path2()),        // intake still on while crossing
                intakeOff(),
                volley(),                         // garden 4 -> high CELL
                flower(path3()),                  // flower 1
                volley(),                         // flower 1's 4 -> high CELL
                follow(follower, path4()),
                flower(path5()),                  // flower 2
                follow(follower, path6()),
                volley(),                         // flower 2's 4 -> low CELL
                follow(follower, path7())         // park
        );
    }

    @Override
    public void runOpMode() {
        Scheduler.reset();
        follower = Constants.create(hardwareMap);
        robot = new Robot(hardwareMap);
        try {
            PoseStorage poseStorage = new PoseStorage();
            poseStorage.setAlliance(alliance());
            follower.setPose(start);
            follower.update();

            telemetry.addLine(alliance() + " Garden + Flowers ready");
            telemetry.update();
            waitForStart();
            Scheduler.schedule(autoRoutine());

            while (opModeIsActive()) {
                follower.update();
                Pose pose = follower.pose();
                double x = pose.x(), y = pose.y(), h = pose.heading();
                poseStorage.savePoseToStorage(x, y, h);

                // turret aims every loop; flywheel spins the whole auto at the RPM for this distance
                robot.turret.setMeasuredVelocity(follower.velocity().vx, follower.velocity().vy, follower.velocity().omega);
                robot.turret.autoAim(x, y, h);
                robot.shooter.setBaseTargetRPM((int) Math.round(ShotRegression.rpmFor(robot.turret.getDistanceToTarget())));
                robot.shooter.calculateShooterPID();
                robot.turretHood.setBase(ShotRegression.hoodFor(robot.turret.getDistanceToTarget()));
                robot.turretHood.hold();

                Scheduler.execute();
                DashboardField.send(x, y, h, robot.turret);

                telemetry.addData("pose", "%.1f, %.1f, %.0f deg", x, y, Math.toDegrees(h));
                telemetry.addData("aiming at", robot.turret.isOnHighSide() ? "high-y CELL" : "low-y CELL");
                telemetry.addData("ready to fire", readyToFire());
                if (follower.currentPath() != null) telemetry.addData("path", follower.pathIndex());
                telemetry.update();
            }
        } finally {
            robot.stop();
        }
    }
}
