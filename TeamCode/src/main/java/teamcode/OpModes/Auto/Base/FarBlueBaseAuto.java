package teamcode.OpModes.Auto.Base;

import static com.pedropathing.api.Paths.curve;
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
 * Far Blue base auto: drives from the start pose to the near-cell shooting spot, then parks.
 * Shooting isn't written yet (see the "add shoot" spots below).
 * The turret auto-aims the whole time and the position is saved for TeleOp.
 * Start pose: (8, 34, 0) (Pedro field inches/degrees). Sets alliance = BLUE.
 */
@Autonomous
public class FarBlueBaseAuto extends LinearOpMode {


    private Follower follower;
    private Robot robot;
    private PoseStorage poseStorage;
    private final PoseFactory poseFactory = PoseFactory.degrees();

    // -----
    // Poses
    // -----
    private final Pose startPos = poseFactory.of(8, 34, 0);
    // check if the distance from the cell is the same for each side. It should be relatively simple and just measure
    // the y difference
    private final Pose nearCellShootPos = poseFactory.of(56, 34, 0);
    private final Pose nearCellShootPosControl = poseFactory.of(11, 29, 0); //reminder: irrelevant control point heading
    private final Pose park = poseFactory.of(14.5, 98, 0);

    // -------
    // Paths
    // -------
    private Path startToNearBlueShoot() {
        return line(startPos, nearCellShootPos).linear(startPos, nearCellShootPos, 0.4);
    }
    private Path nearBlueCellToPark() {
        return curve(nearCellShootPos, nearCellShootPosControl, park).linear(nearCellShootPos, park);
    }

    // ---------
    // Commands
    // ---------
    private Command goToNearBlue() {
        return follow(follower, startToNearBlueShoot());
    }

    private Command goToParkFromNearCell() {
        return follow(follower, nearBlueCellToPark());
    }

    // ------------
    // AutoRoutine
    // ------------
    private Command autoRoutine(PoseStorage.Alliance alliance){
        return sequential(
                goToNearBlue(),
                //add shoot
                //change 1000 to targetRPM
                //robot.vision.useAprilTagsToSetCellState(alliance),
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
        poseStorage.setAlliance(PoseStorage.Alliance.BLUE);
        Scheduler.schedule(autoRoutine(poseStorage.getAlliance()));


        waitForStart();

        while (opModeIsActive()) {
            follower.update();
            poseStorage.savePoseToStorage(follower.pose().x(), follower.pose().y(), follower.pose().heading());
            robot.turret.setMeasuredVelocity(follower.velocity().vx, follower.velocity().vy, follower.velocity().omega);
            robot.turret.autoAim(follower.pose().x(), follower.pose().y(), follower.pose().heading());
            DashboardField.send(follower.pose().x(), follower.pose().y(), follower.pose().heading(), robot.turret);
            Scheduler.execute();

            telemetry.update();
            telemetry.addData("Robot PosX", follower.pose().x());
            telemetry.addData("Robot PosY", follower.pose().y());
            telemetry.addData("Robot Heading", follower.pose().heading());


            //may need to add something to zero the turret so it saves easier to teleOp

        }
        } finally {
            robot.stop();
        }
    }




}
