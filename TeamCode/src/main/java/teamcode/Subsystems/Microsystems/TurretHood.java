package teamcode.Subsystems.Microsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class TurretHood {

    private final Servo turretHood;
    private double lastCommand = Double.NaN;

    public TurretHood(HardwareMap hardwareMap){
        turretHood = hardwareMap.get(Servo.class, "turretHood");
    }

    public void setHoodAngle(double angle){
        if (angle == lastCommand) {
            return;
        }
        turretHood.setPosition(angle);
        lastCommand = angle;
    }

}
