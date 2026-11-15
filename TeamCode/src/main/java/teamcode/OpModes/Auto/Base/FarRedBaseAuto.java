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

import teamcode.ComplexMath.Storage.PoseStorage;
import teamcode.RobotContainer.Robot;
import teamcode.pedro.Constants;

@Autonomous
public class FarRedBaseAuto extends LinearOpMode {


    private Follower follower;
    private Robot robot;
    private final PoseFactory poseFactory = PoseFactory.degrees();
    private PoseStorage poseStorage;

    // -----
    // Poses
    // -----
    private final Pose startPos = poseFactory.of(107, 10, 90);
    // check if the distance from the cell is the same for each side. It should be relatively simple and just measure
    // the y difference
    private final Pose nearCellShootPos = poseFactory.of(107, 30, 90);
    private final Pose park = poseFactory.of(130, 30, 90);

    // -------
    // Paths
    // -------
    private Path startToNearRedShoot() {
        return line(startPos, nearCellShootPos).linear(startPos, nearCellShootPos);
    }
    private Path nearRedCellToPark() {
        return line(nearCellShootPos, park).linear(nearCellShootPos, park);
    }

    // ---------
    // Commands
    // ---------
    private Command goToNearRed() {
        return follow(follower, startToNearRedShoot());
    }

    private Command goToParkFromNearCell() {
        return follow(follower, nearRedCellToPark());
    }

    // ------------
    // AutoRoutine
    // ------------
    private Command autoRoutine(){
        return sequential(

                //go to shootPos
                    //run revShooter in parallel
                    //run calculateTurretPosition in parallel the entire time, make sure it starts set to 0
                //once at shootPos, intake balls
                    //intake trans, then frontIntake, then both
                    //wait a set amount of time (could use a deadline or similar command to not continue until the time has passed)
                //turn
                //move to garden to collect from garden









                goToNearRed(),
                //add shoot
                //waitMs(2000),
                //robot.shooter.basicShoot(),
                goToParkFromNearCell()
        );
    }




    @Override
    public void runOpMode() {
        Scheduler.reset();
        follower = Constants.create(hardwareMap);
        robot = new Robot(hardwareMap);
        try {
            poseStorage = new PoseStorage();
            follower.setPose(startPos);
            poseStorage.setAlliance(PoseStorage.Alliance.RED);
            Scheduler.schedule(autoRoutine());
            waitForStart();
            while (opModeIsActive()) {
                follower.update();
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
