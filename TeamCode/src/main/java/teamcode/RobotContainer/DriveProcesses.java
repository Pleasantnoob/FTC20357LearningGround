package teamcode.RobotContainer;

import com.qualcomm.robotcore.hardware.Gamepad;

public class DriveProcesses {

    private static final int TARGET_RPM = 2240;
    private static final double RPM_TOLERANCE = 15;
    private static final double TRIGGER_THRESHOLD = 0.1;

    private final Gamepad gamepad1;
    private final Gamepad gamepad2;
    private final Robot robot;

    public DriveProcesses(Robot robot, Gamepad gamepad1, Gamepad gamepad2) {
        this.gamepad1 = gamepad1;
        this.gamepad2 = gamepad2;
        this.robot = robot;
    }

    // Y, Start, Back, and Guide are unbound. Gamepad 2 cannot feed or shoot.
    public void update(double posX, double posY, double headingRad, double dt) {
        boolean shoot = gamepad1.left_trigger > TRIGGER_THRESHOLD;
        boolean eject = gamepad1.right_bumper;
        boolean collect = gamepad1.right_trigger > TRIGGER_THRESHOLD;

        if (shoot) {
            robot.shooter.setBaseTargetRPM(TARGET_RPM);
            if (Math.abs(TARGET_RPM - robot.shooter.getRPM()) < RPM_TOLERANCE) {
                robot.turretGate.moveGateToOpenPos();
                robot.intake.collect();
            } else {
                robot.turretGate.moveGateToClosePos();
                robot.intake.stop();
            }
        } else if (eject) {
            robot.shooter.setBaseTargetRPM(0);
            robot.turretGate.moveGateToClosePos();
            robot.intake.eject();
        } else if (collect) {
            robot.shooter.setBaseTargetRPM(0);
            robot.turretGate.moveGateToClosePos();
            robot.intake.collect();
        } else {
            robot.shooter.setBaseTargetRPM(0);
            robot.turretGate.moveGateToClosePos();
            robot.intake.stop();
        }

        robot.shooter.calculateShooterPID();
        robot.turret.update(posX, posY, headingRad, gamepad2, dt);
    }
}
