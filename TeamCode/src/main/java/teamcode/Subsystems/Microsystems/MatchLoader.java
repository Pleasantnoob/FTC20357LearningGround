package teamcode.Subsystems.Microsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

/** Match loader: the deployable flower intake ramp (goBILDA torque servo, config name "matchLoader"). */
@Config
public class MatchLoader {

    // PLACEHOLDERS - tune on the robot (FTC Dashboard -> MatchLoader)
    public static double stowedPos = 0.0;
    public static double deployedPos = 0.5;

    private final Servo matchLoader;
    private boolean deployed = false;
    private double lastCommand = Double.NaN;

    // false = never written, servo stays limp (see RobotContainer/Tuned)
    private boolean enabled = true;
    public void setEnabled(boolean on) { enabled = on; }
    public boolean isEnabled() { return enabled; }

    public MatchLoader(HardwareMap hardwareMap) {
        matchLoader = hardwareMap.get(Servo.class, "matchLoader");
    }

    public void deploy() { deployed = true; command(deployedPos); }
    public void stow() { deployed = false; command(stowedPos); }
    public void toggle() { if (deployed) stow(); else deploy(); }
    public boolean isDeployed() { return deployed; }

    // old names, still used by SubsystemCommands
    public void lowerMatchLoader() { deploy(); }
    public void raiseMatchLoader() { stow(); }

    private void command(double position) {
        if (!enabled || position == lastCommand) return;
        matchLoader.setPosition(position);
        lastCommand = position;
    }
}
