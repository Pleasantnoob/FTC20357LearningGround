package teamcode.OpModes.Auto.Base;

import static com.pedropathing.api.Paths.curve;
import static com.pedropathing.api.Paths.line;
import static com.pedropathing.ivy.commands.Commands.conditional;
import static com.pedropathing.ivy.commands.Commands.infinite;
import static com.pedropathing.ivy.commands.Commands.instant;
import static com.pedropathing.ivy.commands.Commands.waitMs;
import static com.pedropathing.ivy.groups.Groups.deadline;
import static com.pedropathing.ivy.groups.Groups.parallel;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import teamcode.ComplexMath.ShotRegression;
import teamcode.Utility.DashboardField;
import teamcode.ComplexMath.Storage.PoseStorage;
import teamcode.RobotContainer.Robot;
import teamcode.pedro.Constants;

/**
 * Near Red base auto: drives from the start pose to the far-cell shooting spot, then parks.
 * Shooting isn't written yet (see the "add shoot" spots below).
 * Spins the flywheel the whole auto (RPM from ShotRegression).
 * The turret auto-aims the whole time and the position is saved for TeleOp.
 * Start pose: (133, 108, 180) (Pedro field inches/degrees). Sets alliance = RED.
 */
@Config
@Autonomous
public class NearRedBaseAuto extends LinearOpMode {


    private Follower follower;
    private Robot robot;
    private final PoseFactory poseFactory = PoseFactory.degrees();

    private PoseStorage poseStorage;
    // -----
    // Poses
    // -----
    private final Pose startPos = poseFactory.of(133, 108, 180);
    private final Pose farCellShootPos = poseFactory.of(120, 130, 180);
    private final Pose park = poseFactory.of(128, 40, 180);

    // -------
    // Paths
    // -------
    private Path startToFarRedShoot() {
        return line(startPos, farCellShootPos).linear(startPos, farCellShootPos);
    }
    private Path farRedCellToPark() {
        return line(farCellShootPos, park).linear(farCellShootPos, park);
    }

    // ---------
    // Commands
    // ---------
    private Command goToFarRed() {
        return follow(follower, startToFarRedShoot());
    }

    private Command goToParkFromFarCell() {
        return follow(follower, farRedCellToPark());
    }








    /*
    private Command shoot(){
        return infinite(() -> parallel(
                setShooterRPM(),
                fullIntakeCommand()
                )

        );


    }
    */



    // ------------
    // AutoRoutine
    // ------------
    private Command autoRoutine(){
        return  sequential(
                goToFarRed(),
                //add shoot
                goToParkFromFarCell()
        );
    }



    @Override
    public void runOpMode() {
        Scheduler.reset();
        follower = Constants.create(hardwareMap);
        robot = new Robot(hardwareMap);
        try {
            poseStorage = new PoseStorage();
            poseStorage.setAlliance(PoseStorage.Alliance.RED);
            follower.setPose(startPos);
            Scheduler.schedule(autoRoutine());
            waitForStart();

            while (opModeIsActive()) {
                follower.update();
                poseStorage.savePoseToStorage(follower.pose().x(), follower.pose().y(), follower.pose().heading());
                robot.turret.setMeasuredVelocity(follower.velocity().vx, follower.velocity().vy, follower.velocity().omega);
                robot.turret.autoAim(follower.pose().x(), follower.pose().y(), follower.pose().heading());
                DashboardField.send(follower.pose().x(), follower.pose().y(), follower.pose().heading(), robot.turret);
                robot.shooter.setBaseTargetRPM((int) Math.round(ShotRegression.rpmFor(robot.turret.getDistanceToTarget())));
                robot.shooter.calculateShooterPID();
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
