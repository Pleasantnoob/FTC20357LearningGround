package teamcode.Subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import teamcode.ComplexMath.Storage.PoseStorage;

@Config
public class Turret {

    public enum AimMode { IDLE, AUTO, MANUAL, HOLD_ZERO }

    // Axon Max servos, geared 1:1 with the turret. No programmer, so 0.5 is straight
    // ahead and the factory travel is about 350 degrees. ±110° of turret is only the
    // middle of that travel; it is not stretched across the full 0-1 range.
    public static double travelLimitDeg = 110;
    public static double servoFullScaleDeg = 350;
    public static boolean reverseRight = false;
    public static double slewDegPerSec = 150;
    public static double stickDeadband = 0.08;

    private final Servo leftTurret;
    private final Servo rightTurret;
    private final PoseStorage poseStorage = new PoseStorage();

    private double turretTargetX = 72;
    private double turretTargetY = 72;
    private double commandedDeg = 0;
    private double lastLeft = Double.NaN;
    private double lastRight = Double.NaN;
    private boolean zeroHeld = false;
    private AimMode aimMode = AimMode.IDLE;

    public Turret(HardwareMap hardwareMap) {
        leftTurret = hardwareMap.get(Servo.class, "leftTurret");
        rightTurret = hardwareMap.get(Servo.class, "rightTurret");
    }

    public void update(double posX, double posY, double robotHeadingRad, Gamepad overridePad, double dt) {
        double stick = overridePad.right_stick_x;
        if (Math.abs(stick) > stickDeadband) {
            zeroHeld = false;
            slew(stick, dt);
            aimMode = AimMode.MANUAL;
            return;
        }

        if (overridePad.b) {
            zeroHeld = false;
        } else if (overridePad.a) {
            zeroHeld = true;
        }

        if (zeroHeld) {
            commandDegrees(0);
            aimMode = AimMode.HOLD_ZERO;
            return;
        }

        PoseStorage.Alliance alliance = poseStorage.getAlliance();
        if (alliance == null || alliance == PoseStorage.Alliance.NONE) {
            aimMode = AimMode.IDLE;
            return;
        }

        updateTurretTarget(posY);
        track(posX, posY, robotHeadingRad);
        aimMode = AimMode.AUTO;
    }

    public void setTarget(double x, double y) {
        turretTargetX = x;
        turretTargetY = y;
    }

    public void track(double posX, double posY, double robotHeadingRad) {
        commandDegrees(robotRelativeDegrees(posX, posY, robotHeadingRad));
        aimMode = AimMode.AUTO;
    }

    public void slew(double stick, double dt) {
        double cappedDt = Math.min(Math.max(dt, 0), 0.05);
        double next = commandedDeg + stick * slewDegPerSec * cappedDt;
        commandDegrees(next);
        aimMode = AimMode.MANUAL;
    }

    public double robotRelativeDegrees(double posX, double posY, double robotHeadingRad) {
        double disX = turretTargetX - posX;
        double disY = turretTargetY - posY;
        double targetFieldBearing = Math.toDegrees(Math.atan2(disY, disX));
        double robotHeadingDeg = Math.toDegrees(robotHeadingRad);
        double robotRelative = normalizeDegrees(targetFieldBearing - robotHeadingDeg);
        double limit = Math.abs(travelLimitDeg);
        return Range.clip(robotRelative, -limit, limit);
    }

    public void updateTurretTarget(double y) {
        PoseStorage.Alliance alliance = poseStorage.getAlliance();
        if (alliance == PoseStorage.Alliance.RED) {
            if (y < 72) {
                turretTargetX = 84.79;
                turretTargetY = 87.32;
            } else {
                turretTargetX = 83.84;
                turretTargetY = 50.27;
            }
        } else if (alliance == PoseStorage.Alliance.BLUE) {
            if (y < 72) {
                turretTargetX = 58.05;
                turretTargetY = 86.41;
            } else {
                turretTargetX = 57.82;
                turretTargetY = 53.41;
            }
        } else {
            turretTargetX = 72;
            turretTargetY = 72;
        }
    }

    public double getTurretTargetX() {
        return turretTargetX;
    }

    public double getTurretTargetY() {
        return turretTargetY;
    }

    public double getTurretHeading() {
        return commandedDeg;
    }

    public double getLeftServoPosition() {
        return servoPosition(commandedDeg);
    }

    public AimMode getAimMode() {
        return aimMode;
    }

    private void commandDegrees(double degrees) {
        double limit = Math.abs(travelLimitDeg);
        commandedDeg = Range.clip(degrees, -limit, limit);
        writeServos();
    }

    private void writeServos() {
        double left = servoPosition(commandedDeg);
        double right = reverseRight ? 1.0 - left : left;
        if (left != lastLeft) {
            leftTurret.setPosition(left);
            lastLeft = left;
        }
        if (right != lastRight) {
            rightTurret.setPosition(right);
            lastRight = right;
        }
    }

    private double servoPosition(double degrees) {
        double scale = servoFullScaleDeg == 0 ? 350 : Math.abs(servoFullScaleDeg);
        return Range.clip(0.5 + degrees / scale, 0.0, 1.0);
    }

    private double normalizeDegrees(double degrees) {
        while (degrees > 180) degrees -= 360;
        while (degrees < -180) degrees += 360;
        return degrees;
    }
}
