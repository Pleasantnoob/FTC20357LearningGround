package teamcode.Subsystems.Microsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class TurretGate {

    private final Servo turretGate;
    private final double openGatePos = 0;
    private final double closeGatePos = 1;
    private double lastCommand = Double.NaN;

    public TurretGate(HardwareMap hardwareMap) {
        turretGate = hardwareMap.get(Servo.class, "turretGate");
    }

    public void moveGateToOpenPos() {
        command(openGatePos);
    }

    public void moveGateToClosePos() {
        command(closeGatePos);
    }

    public double getGatePos() {
        return lastCommand;
    }

    public double getOpenGatePos() {
        return openGatePos;
    }

    public double getCloseGatePos() {
        return closeGatePos;
    }

    private void command(double position) {
        if (position == lastCommand) {
            return;
        }
        turretGate.setPosition(position);
        lastCommand = position;
    }
}
