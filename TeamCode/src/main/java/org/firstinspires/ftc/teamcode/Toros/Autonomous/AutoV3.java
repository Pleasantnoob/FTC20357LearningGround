package org.firstinspires.ftc.teamcode.Toros.Autonomous;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.RR.MecanumDrive;
import org.firstinspires.ftc.teamcode.RR.PoseBridge;
@Config
@Autonomous(name = "AUTOV3")
@Disabled
public class AutoV3 extends LinearOpMode {


    public class Claw{
        private Servo specClaw;

        public Claw(HardwareMap hardwareMap){
            specClaw = hardwareMap.get(Servo.class,"specClaw");

            specClaw.setPosition(1);

        }
        public class OpenFingers implements Action{
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                specClaw.setPosition(0.0);
                return false;
            }
        }
        public Action openFingers(){
            return new OpenFingers();
        }
        public class closeFingers implements Action{
            @Override
            public boolean run(@NonNull TelemetryPacket Packet) {

                specClaw.setPosition(1.0);
                return false;
            }
        }
        public Action closeFingers(){
            return new closeFingers();
        }

    }



    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d initialPose = new Pose2d(9,-61,Math.toRadians(90));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
        Claw claw = new Claw(hardwareMap);

//        Actions.runBlocking(claw.closeFingers());
//        TrajectoryActionBuilder tab1 = drive.actionBuilder(initialPose)
//                        .strafeTo(new Vector2d(0,1))
//                .stopAndAdd(lift.changeTarget(-1500))
//                        .waitSeconds(1);
//        Action trajectoryActionCloseOut = tab1.endTrajectory().fresh()
//
//                .waitSeconds(3)
//                        .strafeTo(new Vector2d(0,0))
//                                .build();
//        Action tab1A = tab1.build();

        Action traj1 = drive.actionBuilder(initialPose)
                .strafeTo(new Vector2d(9,-30))
                .build();
        Action traj2 = drive.actionBuilder(new Pose2d(9,-35,Math.toRadians(90)))
                .strafeToLinearHeading(new Vector2d(30, -38), Math.toRadians(90))
                .turnTo(Math.toRadians(270))
                //.strafeTo(new Vector2d(40, -12))
                .splineToConstantHeading(new Vector2d(49,-18), Math.toRadians(0))
                .strafeTo(new Vector2d(49,-60))//push spec 3
                .strafeTo(new Vector2d(49, -59))
                .build();
        //pick up spec 2
        Action traj3 = drive.actionBuilder(new Pose2d(49,-60,Math.toRadians(270)))
                //.strafeToLinearHeading(new Vector2d(3, -38), Math.toRadians(90))
                .strafeTo(new Vector2d(20, -38))
                .turnTo(Math.toRadians(90))
                .strafeTo(new Vector2d(3, -30))
                .build();
        //Hang spec 2
        Action traj4 = drive.actionBuilder(new Pose2d(3,-38,Math.toRadians(90)))
                .strafeToLinearHeading(new Vector2d(30,-65), Math.toRadians(270))
                .splineToConstantHeading(new Vector2d(60,-35), Math.toRadians(270))
                .strafeTo(new Vector2d(55, -60))//push spec 4
                .build();
        //pick up spec 3
        Action traj5 = drive.actionBuilder(new Pose2d(55,-60,Math.toRadians(90)))
                .strafeToLinearHeading(new Vector2d(0, -35), Math.toRadians(90))
                //.stopAndAdd(lift.changetarget(1))
                .build();
        //Hang spec 3
        Action traj6 = drive.actionBuilder(new Pose2d(0,-35,Math.toRadians(90)))
                .strafeToLinearHeading(new Vector2d(20,-38), Math.toRadians(270))
                .splineToConstantHeading(new Vector2d(62,-15), Math.toRadians(270))
                .strafeTo(new Vector2d(62, -60))//push spec 5
                .build();
        Action traj7 = drive.actionBuilder(new Pose2d(62,-60,Math.toRadians(270)))
                .strafeToLinearHeading(new Vector2d(-3, -35), Math.toRadians(90))
                .build();
        Action park = drive.actionBuilder(new Pose2d(9,-61,Math.toRadians(90)))
                .strafeTo(new Vector2d(65,-65))
                .build();
        //Pick up spec 4

        waitForStart();
        Actions.runBlocking(
                new SequentialAction(
                        claw.closeFingers(),
                        traj1,
                        new SleepAction(1),
                        claw.openFingers(),
                        park
                )
        );

        PoseBridge.save(org.firstinspires.ftc.teamcode.util.Pose2d.fromRR(drive.localizer.getPose()));
        PoseBridge.saveAlliance(true);  // Blue

        telemetry.update();
    }
}