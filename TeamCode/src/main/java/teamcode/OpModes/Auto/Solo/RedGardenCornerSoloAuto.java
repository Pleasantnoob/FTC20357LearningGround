package teamcode.OpModes.Auto.Solo;

import static com.pedropathing.ivy.commands.Commands.waitMs;
import static com.pedropathing.ivy.groups.Groups.deadline;
import static com.pedropathing.ivy.groups.Groups.parallel;
import static com.pedropathing.ivy.groups.Groups.race;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;

import com.pedropathing.api.Paths;
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
import teamcode.RobotContainer.SubsystemCommands;
import teamcode.pedro.Constants;

/**
 * Red solo auto from the garden corner, start (133.9, 107.1, 180).
 * Drives to the first shooting spot and shoots the preloads, collects from the garden,
 * then drives to the park-side spot and shoots again. Parking isn't written yet.
 * The flywheel PID and turret aim run the whole time; position is saved for TeleOp.
 */
@Autonomous
public class RedGardenCornerSoloAuto extends LinearOpMode {


    Follower follower;
    Robot robot;
    PoseStorage poseStorage;
    SubsystemCommands subsystemCommands;



    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose startRedGarden = poseFactory.of(133.8808, 107.1101, 180);
    private final Pose startToFirstShoot = poseFactory.of(91, 126, 265);
    private final Pose firstShootToGardenStart = poseFactory.of(91, 126, 0);
    private final Pose firstShootToGarden = poseFactory.of(133.3849, 134.1465, 0);
    private final Pose firstShootToGardenControl1 = poseFactory.of(66, 141, 0);
    private final Pose gardenToParkShootStart = poseFactory.of(133.3849, 134.1465, 0);
    private final Pose gardenToParkShoot = poseFactory.of(81.5004, 18.7214, 0);
    private final Pose gardenToParkShootControl1 = poseFactory.of(73.0562, 141.5, 0);
    private final Pose parkShootToGardenFlower = poseFactory.of(132.1386, 100.2068, 335);
    private final Pose parkShootToGardenFlowerControl1 = poseFactory.of(70.0464, 139.0067, 0);
    private final Pose gardenFlowerToFlowerShootStart = poseFactory.of(132.1386, 100.2068, 0);
    private final Pose gardenFlowerToFlowerShoot = poseFactory.of(90.572, 119.3808, 260);
    private final Pose point6Start = poseFactory.of(90.572, 119.3808, 180);
    private final Pose point6 = poseFactory.of(132.6193, 50.4211, 180);

    // ------
    // PATHS
    // ------

    public Path startToFirstShootPath() {
        return Paths.line(startRedGarden, startToFirstShoot).linear(startRedGarden, startToFirstShoot);
    }

    public Path firstShootToGardenPath() {
        return Paths.curve(firstShootToGardenStart, firstShootToGardenControl1, firstShootToGarden).linear(firstShootToGardenStart, firstShootToGarden);
    }

    public Path gardenToParkShootPath() {
        return Paths.curve(gardenToParkShootStart, gardenToParkShootControl1, gardenToParkShoot).linear(gardenToParkShootStart, gardenToParkShoot);
    }

    public Path parkShootToGardenFlowerPath() {
        return Paths.curve(gardenToParkShoot, parkShootToGardenFlowerControl1, parkShootToGardenFlower).constant(parkShootToGardenFlower);
    }

    public Path gardenFlowerToFlowerShootPath() {
        return Paths.line(gardenFlowerToFlowerShootStart, gardenFlowerToFlowerShoot).linear(gardenFlowerToFlowerShootStart, gardenFlowerToFlowerShoot);
    }

    public Path flowerShootToParkPath() {
        return Paths.line(point6Start, point6).linear(point6Start, point6);
    }

    // ---------
    // Commands
    // ---------

    public Command goToFirstShoot(){
        return follow(follower, startToFirstShootPath());
    }

    public Command goToGarden(){
        return follow(follower, firstShootToGardenPath());
    }

    public Command goToParkShoot(){
        return follow(follower, gardenToParkShootPath());
    }

    public Command goToGardenFlower(){
        return follow(follower, parkShootToGardenFlowerPath());
    }

    public Command goToFlowerShoot(){
        return follow(follower, gardenFlowerToFlowerShootPath());
    }

    public Command park(){
        return follow(follower, flowerShootToParkPath());
    }












    public Command autoRoutine() {

        return parallel(

                subsystemCommands.updateShooterPID(),
                //add turret and calculate RPM and Hood
                sequential(
                        //need to plug in RPM Calculator here (actually in the underlying
                        //command (which should also be moved to the root method)

                        //FIRST SHOOT ACTION
                        //does not need to be deadline because setShooterRPM is an
                        // instant command, not an infinite command
                        deadline(
                                // must run continously parallel to sequential because
                                // it uses coordinates to calculate RPM
                                // so it must be running the entire time so it
                                // will take the least amount of time at the end
                                sequential(
                                        //move to first shoot with preloaded 4 balls
                                        goToFirstShoot(),
                                        subsystemCommands.openGate(),
                                        //so the intake and gate don't collide
                                        waitMs(200),
                                        subsystemCommands.fullIntakeCommand(),
                                        // pause so all balls can be shot
                                        waitMs(2000)
                                ),
                                subsystemCommands.updateShooterRPM()
                                //add update hood position
                        ),

                        // close gate so the intake when collecting from garden does not
                        // cause it to shoot
                        subsystemCommands.closeGate(),
                        // rest the shooter
                        subsystemCommands.zeroShooterRPM(),

                        sequential(
                                // redundant because it is already running, but I want
                                // to remind myself it is running
                                subsystemCommands.fullIntakeCommand(),
                                goToGarden(),
                                // wait some time to ensure we collected everything
                                waitMs(300)
                        ),
                        // stop intake so it does not overcollide with the gate
                        subsystemCommands.zeroIntakeCommand(),
                        //SECOND SHOOT ACTION
                        deadline(
                                //needs deadline or parallel will cause it to run
                                // infinitely becase the setShooterRP
                                sequential(

                                        goToParkShoot(),
                                        //must be run after park shoot
                                        //open gate for shoot
                                        subsystemCommands.openGate(),
                                        waitMs(200),
                                        subsystemCommands.fullIntakeCommand(),
                                        waitMs(2000)
                                        //finished shooting
                                ),

                                subsystemCommands.updateShooterRPM()
                                //add update hood position

                        ),
                        subsystemCommands.closeGate(),
                        subsystemCommands.zeroShooterRPM(),
                        sequential(



                        )



                        )


                );






        //Pseudocode:
                //run the flywheel with an infinite command the entire
                    //time in parallel (maybe not, this could cause it to overexhert the motor)
                 //move from startPos by flower to shootPos
                //turn trans on, wait a time (roughly a second), then turn full intake on
                //after a certain amount of time that it takes to shoot all 4 balls, turn off full intake
                //move to midPos where we could collect from the garden or flower







    }









    public void runOpMode(){
        Scheduler.reset();

        robot = new Robot(hardwareMap);
        try {
        poseStorage = new PoseStorage();
        subsystemCommands = new SubsystemCommands(hardwareMap, robot);

        poseStorage.setAlliance(PoseStorage.Alliance.RED);

        follower = Constants.create(hardwareMap);
        follower.setPose(startRedGarden);

        Scheduler.schedule(autoRoutine());

        waitForStart();


        while(opModeIsActive()){

            Scheduler.execute();
            follower.update();
            poseStorage.savePoseToStorage(follower.pose().x(), follower.pose().y(), follower.pose().heading());
            robot.turret.setMeasuredVelocity(follower.velocity().vx, follower.velocity().vy, follower.velocity().omega);
            robot.turret.autoAim(follower.pose().x(), follower.pose().y(), follower.pose().heading());
            DashboardField.send(follower.pose().x(), follower.pose().y(), follower.pose().heading(), robot.turret);
            //add concurrent PID methods





            telemetry.update();


        }
        } finally {
            robot.stop();
        }
    }
















}
