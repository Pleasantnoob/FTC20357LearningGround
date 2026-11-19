package teamcode.OpModes.Auto.Base;

import static com.pedropathing.api.Paths.line;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import teamcode.Utility.DashboardField;
import teamcode.ComplexMath.Storage.PoseStorage;
import teamcode.RobotContainer.Robot;
import teamcode.pedro.Constants;

/**
 * Near Blue base auto: drives from the start pose to the shooting spot, then parks.
 * Shooting isn't written yet (see the "add shoot" spots below).
 * The turret auto-aims the whole time and the position is saved for TeleOp.
 * Start pose: (15, 135, 270) (Pedro field inches/degrees). Sets alliance = BLUE.
 */
@Autonomous
public class NearBlueBaseAuto extends LinearOpMode {


    private Follower follower;
    private Robot robot;
    private final PoseFactory poseFactory = PoseFactory.degrees();
    private PoseStorage poseStorage;

    // -----
    // Poses
    // -----
    private final Pose startPos = poseFactory.of(15, 135, 270);
    // check if the distance from the cell is the same for each side. It should be relatively simple and just measure
    // the y difference
    private final Pose park = poseFactory.of(11, 120, 270);

    // -------
    // Paths
    // -------
    private Path startToPark() {
        return line(startPos, park).linear(startPos, park);
    }

    // ---------
    // Commands
    // ---------
    private Command goToPark() {
        return follow(follower, startToPark());
    }


    // ------------
    // AutoRoutine
    // ------------
    private Command autoRoutine(){
        return sequential(

                //check if cell is raised
                    //if it is, perform the following action
                    //if not, waitUntil with conditional or something similar
                        //add redundancy in case the other team cannot




                //repeat 4 times (we start with 4 preloaded pollen)
                    //set TargetRPM
                    //when at specific RPM (or in error range)
                        //openServoGate
                        // loadLauncher
                        // close servoGate
                        // zeroTransfer

                        // fullFrontIntake
                        //zeroFrontIntake





                //go to Far blue shooting pos or start there
                //hive is always tipped up on far blue, so I don't need to check aprilTags
                //shoot
                //go to park

                goToPark()
                //robot.shooter.basicShoot(),
                //Repeat 4 times
                    //if a certain RPM margin of error is reached, load a ball for shooting

        );
    }




    @Override
    public void runOpMode() {
        Scheduler.reset();
        follower = Constants.create(hardwareMap);
        robot = new Robot(hardwareMap);
        try {
        follower.setPose(startPos);
        poseStorage = new PoseStorage();
        poseStorage.setAlliance(PoseStorage.Alliance.BLUE);
        Scheduler.schedule(autoRoutine());
        waitForStart();

        robot.vision.resetScanner();
        while (opModeIsActive()) {
            follower.update();
            poseStorage.savePoseToStorage(follower.pose().x(), follower.pose().y(), follower.pose().heading());
            robot.turret.setMeasuredVelocity(follower.velocity().vx, follower.velocity().vy, follower.velocity().omega);
            robot.turret.autoAim(follower.pose().x(), follower.pose().y(), follower.pose().heading());
            DashboardField.send(follower.pose().x(), follower.pose().y(), follower.pose().heading(), robot.turret);
            //robot.vision.scanForClusters();
            //robot.turret.update(follower.pose().x(), follower.pose().y(), follower.pose().heading(), gamepad2, 0);
            //robot.shooter.calculateShooterPID();
            Scheduler.execute();

            telemetry.update();
            telemetry.addData("Robot PosX", follower.pose().x());
            telemetry.addData("Robot PosY", follower.pose().y());
            telemetry.addData("Robot Heading", follower.pose().heading());

        }
        } finally {
            robot.stop();
        }
    }
}
