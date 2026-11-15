package teamcode.RobotContainer;

import com.qualcomm.robotcore.hardware.HardwareMap;

import teamcode.Subsystems.Intake;
import teamcode.Subsystems.Microsystems.MatchLoader;
import teamcode.Subsystems.Microsystems.TurretGate;
import teamcode.Subsystems.Microsystems.TurretHood;
import teamcode.Subsystems.Shooter;
import teamcode.Subsystems.Turret;
import teamcode.Subsystems.Drivetrain;
import teamcode.Subsystems.Vision;

public class Robot {



    //public Follower follower;


    public Drivetrain drivetrain;
    public Vision vision;
    public Intake intake;
    public Shooter shooter;
    public Turret turret;
    public TurretGate turretGate;
    public MatchLoader matchLoader;

    public TurretHood turretHood;


    // Gamepad 1 drives, collects, and shoots. Gamepad 2 only overrides the turret.



    public Robot(HardwareMap hardwareMap) {

        drivetrain = new Drivetrain(hardwareMap);
        vision = new Vision(hardwareMap);
        intake = new Intake(hardwareMap);
        shooter = new Shooter(hardwareMap);
        turret = new Turret(hardwareMap);

        /*
        Eventually create discrete Subsystems and Microsystems 'hub' classes to keep the objects more organized. I
        am not doing that for meet 2 because it will take time I don't have.
        */

        turretGate = new TurretGate(hardwareMap);
        matchLoader = new MatchLoader(hardwareMap);
        turretHood = new TurretHood(hardwareMap);

    }

    public void stop() {
        drivetrain.stop();
        intake.forceStop();
        shooter.emergencyShooterStop();
    }

    //figure out a way to either make the follower public in this file or make follower public in
    // the op mode files and allow the advanced shoot formula methods to access the robot position








    //public void initRobotFollower(HardwareMap hardwareMap, Pose startPos){
    //    follower = Constants.create(hardwareMap);
    //    follower.setPose(startPos);
    //}


}
