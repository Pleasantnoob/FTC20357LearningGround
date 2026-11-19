package teamcode.RobotContainer;

import com.qualcomm.robotcore.hardware.Gamepad;

import teamcode.ComplexMath.ShotRegression;
import teamcode.ComplexMath.Storage.PoseStorage;

/**
 * GAMEPAD 1 - everything for a match
 *   Left stick ........ drive / strafe        Right stick X ... turn
 *   Left trigger ...... SHOOT (hold): spins up, fires when RPM + turret are on target
 *   Right trigger ..... collect               Right bumper .... eject
 *   Left bumper ....... toggle match loader (flower ramp) deploy / stow
 *
 * GAMEPAD 2 - clean-up and fine adjust
 *   Right stick X ..... manual turret (auto-aim resumes on release)
 *   A ................. hold turret centered    B ...... release back to auto-aim
 *   Dpad left / right . turret aim trim -/+ 1 deg
 *   Dpad up / down .... hood up / down
 *   X ................. alliance = BLUE         Y ...... alliance = RED
 *   Left bumper ....... reset turret trim + hood to default
 *   Right bumper ...... toggle shoot-on-the-move ON / OFF
 *   Back / Share ...... reset robot position (handled in MainDrive)
 */
public class DriveProcesses {

    /** Used until the ShotRegression RPM equation is filled in. */
    private static final int TARGET_RPM = 2240;
    private static final double RPM_TOLERANCE = 40; // 28-CPR velocity is noisy; 15 almost never settled
    private static final double TRIGGER_THRESHOLD = 0.1;

    private final Gamepad gamepad1;
    private final Gamepad gamepad2;
    private final Robot robot;
    private final PoseStorage poseStorage = new PoseStorage();

    private boolean volleyLatched = false;

    // previous button states for press (edge) detection
    private boolean p1LB, p2DL, p2DR, p2DU, p2DD, p2X, p2Y, p2LB, p2RB;

    public DriveProcesses(Robot robot, Gamepad gamepad1, Gamepad gamepad2) {
        this.gamepad1 = gamepad1;
        this.gamepad2 = gamepad2;
        this.robot = robot;
    }

    public void update(double posX, double posY, double headingRad, double dt) {
        gamepad2FineAdjust();

        // Turret first, so isOnTarget() reflects this loop's aim.
        robot.turret.update(posX, posY, headingRad, gamepad2, dt);

        // Distance -> RPM + hood from the regression (falls back to TARGET_RPM / hood default).
        double distance = robot.turret.getDistanceToTarget();
        int shotRpm = ShotRegression.hasData()
                ? (int) Math.round(ShotRegression.rpmFor(distance))
                : TARGET_RPM;
        robot.turretHood.setBase(ShotRegression.hoodFor(distance)); // NaN = hood default
        robot.turretHood.hold();

        if (pressed(gamepad1.left_bumper, p1LB)) robot.matchLoader.toggle();
        p1LB = gamepad1.left_bumper;

        boolean shoot = gamepad1.left_trigger > TRIGGER_THRESHOLD;
        boolean eject = gamepad1.right_bumper;
        boolean collect = gamepad1.right_trigger > TRIGGER_THRESHOLD;

        if (shoot) {
            robot.shooter.setBaseTargetRPM(shotRpm);
            boolean ready = Math.abs(shotRpm - robot.shooter.getRPM()) < RPM_TOLERANCE
                    && robot.turret.isOnTarget();
            // Once a volley starts, stay open while the trigger is held. Each ball dips
            // the RPM, and re-closing the gate on every dip jams the next ball.
            if (ready) volleyLatched = true;
            if (volleyLatched) {
                robot.turretGate.moveGateToOpenPos();
                robot.intake.collect();
            } else {
                robot.turretGate.moveGateToClosePos();
                robot.intake.stop();
            }
        } else {
            volleyLatched = false;
            robot.shooter.setBaseTargetRPM(0);
            robot.turretGate.moveGateToClosePos();
            if (eject) {
                robot.intake.eject();
            } else if (collect) {
                robot.intake.collect();
            } else {
                robot.intake.stop();
            }
        }

        robot.shooter.calculateShooterPID();
    }

    private void gamepad2FineAdjust() {
        if (pressed(gamepad2.dpad_left, p2DL)) robot.turret.nudgeTrim(-1);
        if (pressed(gamepad2.dpad_right, p2DR)) robot.turret.nudgeTrim(+1);
        if (pressed(gamepad2.dpad_up, p2DU)) robot.turretHood.nudge(+1);
        if (pressed(gamepad2.dpad_down, p2DD)) robot.turretHood.nudge(-1);
        if (pressed(gamepad2.x, p2X)) poseStorage.setAlliance(PoseStorage.Alliance.BLUE);
        if (pressed(gamepad2.y, p2Y)) poseStorage.setAlliance(PoseStorage.Alliance.RED);
        if (pressed(gamepad2.left_bumper, p2LB)) {
            robot.turret.resetTrim();
            robot.turretHood.resetToDefault();
        }
        p2DL = gamepad2.dpad_left;
        p2DR = gamepad2.dpad_right;
        p2DU = gamepad2.dpad_up;
        p2DD = gamepad2.dpad_down;
        p2X = gamepad2.x;
        p2Y = gamepad2.y;
        if (pressed(gamepad2.right_bumper, p2RB)) robot.turret.toggleShootOnTheMove();
        p2LB = gamepad2.left_bumper;
        p2RB = gamepad2.right_bumper;
    }

    private static boolean pressed(boolean now, boolean before) {
        return now && !before;
    }
}
