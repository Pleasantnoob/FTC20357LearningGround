package teamcode.Subsystems.Microsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

/** goBILDA torque servo that moves out of the way to let balls into the flywheel. */
@Config
public class TurretGate {

    // PLACEHOLDERS - tune on the robot (FTC Dashboard -> TurretGate)
    public static double closeGatePos = 0.0;
    public static double openGatePos = 0.3;

    private final Servo turretGate;
    private double lastCommand = Double.NaN;

    // false = never written, servo stays limp (see RobotContainer/Tuned)
    private boolean enabled = true;
    public void setEnabled(boolean on) { enabled = on; }
    public boolean isEnabled() { return enabled; }

    public TurretGate(HardwareMap hardwareMap) {
        turretGate = hardwareMap.get(Servo.class, "turretGate");
    }

    public void moveGateToOpenPos() { command(openGatePos); }
    public void moveGateToClosePos() { command(closeGatePos); }

    public double getGatePos() { return lastCommand; }
    public double getOpenGatePos() { return openGatePos; }
    public double getCloseGatePos() { return closeGatePos; }
    public boolean isOpen() { return lastCommand == openGatePos; }

    private void command(double position) {
        if (!enabled || position == lastCommand) return;
        turretGate.setPosition(position);
        lastCommand = position;
    }
}
