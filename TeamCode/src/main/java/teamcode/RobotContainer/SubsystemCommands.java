package teamcode.RobotContainer;

import static com.pedropathing.ivy.commands.Commands.infinite;
import static com.pedropathing.ivy.commands.Commands.instant;

import com.pedropathing.ivy.Command;
import com.qualcomm.robotcore.hardware.HardwareMap;

import teamcode.ComplexMath.ShotRegression;

/**
 * Ivy commands for the autos. Each method wraps a subsystem action so it can be dropped
 * into sequential()/parallel() groups next to path following.
 *   instant(...)  runs once and finishes.
 *   infinite(...) runs every loop until whatever group it's in ends it (use inside deadline()).
 */
public class SubsystemCommands {


    Robot robot;

    public SubsystemCommands(HardwareMap hardwareMap, Robot robot){
        this.robot = robot;




    }















    public Command activateMatchLoader(){

        return instant(() -> robot.matchLoader.lowerMatchLoader());
    }

    public Command restMatchLoader(){
        return instant(() -> robot.matchLoader.raiseMatchLoader());
    }




    public Command openGate(){
        return instant(() -> robot.turretGate.moveGateToOpenPos());
    }

    public Command closeGate(){
        return instant(() -> robot.turretGate.moveGateToClosePos());
    }




    public Command updateShooterRPM(){
        // add the autoRPM algorithm in the setBaseTargetRPM algorithm
        // Distance -> RPM + hood from the ShotRegression equations (2240 + hood default until filled in).
        // Needs robot.turret.autoAim(...) running every loop so the distance is current.
        return infinite(() -> {
            double d = robot.turret.getDistanceToTarget();
            robot.shooter.setBaseTargetRPM((int) Math.round(ShotRegression.rpmFor(d)));
            robot.turretHood.setBase(ShotRegression.hoodFor(d));
            robot.turretHood.hold();
        });
    }
    public Command zeroShooterRPM(){
        return instant(() -> robot.shooter.setBaseTargetRPM(0));
    }

    /** Runs the flywheel controller every loop. Keep this in parallel for the whole auto. */
    public Command updateShooterPID(){
        return infinite(() -> robot.shooter.calculateShooterPID());
    }



    public Command fullIntakeCommand(){

        return instant(() -> robot.intake.fullIntake());
    }
    public Command zeroIntakeCommand(){
        return instant(() -> robot.intake.zeroIntake());
    }








}
