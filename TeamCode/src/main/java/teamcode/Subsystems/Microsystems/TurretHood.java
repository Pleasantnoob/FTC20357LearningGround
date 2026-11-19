package teamcode.Subsystems.Microsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

/**
 * Adjustable hood (goBILDA torque servo). Travel not measured yet.
 * Position = base (default or regression) + offset (gamepad 2 nudges), clipped to min/max.
 */
@Config
public class TurretHood {

    // PLACEHOLDERS - keep min/max inside the real travel so the torque servo never stalls on a hard stop
    public static double minPos = 0.3;
    public static double maxPos = 0.7;
    public static double defaultPos = 0.5;
    public static double nudgeStep = 0.02;

    private final Servo turretHood;
    private double basePos = Double.NaN;
    private double offset = 0;
    private double lastCommand = Double.NaN;

    // false = never written, servo stays limp (see RobotContainer/Tuned)
    private boolean enabled = true;
    public void setEnabled(boolean on) { enabled = on; }
    public boolean isEnabled() { return enabled; }

    public TurretHood(HardwareMap hardwareMap) {
        turretHood = hardwareMap.get(Servo.class, "turretHood");
    }

    /** Call every loop. Writes base + offset (repeat values are skipped). */
    public void hold() {
        write(getTargetPos());
    }

    /** Base position from the regression. NaN = use defaultPos. */
    public void setBase(double position) { basePos = position; }

    public void nudge(int direction) { offset += direction * nudgeStep; }

    public void resetToDefault() {
        offset = 0;
        basePos = Double.NaN;
    }

    public double getTargetPos() {
        double base = Double.isNaN(basePos) ? defaultPos : basePos;
        return Range.clip(base + offset, minPos, maxPos);
    }

    /** Direct set (clears the offset). */
    public void setHoodAngle(double position) {
        basePos = position;
        offset = 0;
        hold();
    }

    private void write(double position) {
        if (!enabled || position == lastCommand) return;
        turretHood.setPosition(position);
        lastCommand = position;
    }
}
